`ifndef _ICACHE_BRAM_V_
`define _ICACHE_BRAM_V_

`timescale 1ns/1ps

// BRAM-backed direct-mapped instruction cache.
//
// Replaces the old cache_core-based icache, whose tag/data arrays were read
// COMBINATIONALLY and therefore could only infer distributed LUTRAM + F7/F8
// mux trees - which capped the icache at 2KB (1024 sets failed DRC needing
// 50186 F7 muxes against the device's 26600). Every array here is read
// inside always @(posedge clk) instead, the pattern Xilinx BRAM requires,
// which is what makes 32KB affordable in ~10 of the ~124 free BRAM tiles.
//
// THE KEY IDEA - why a registered-read cache costs no extra stall cycle:
// the arrays are addressed with fetch_addr_next (the value `pc` will hold
// NEXT cycle, sourced from program_counter itself), not with the current
// pc. So the BRAM output registers latch the right line at the same clock
// edge that pc advances to that address, and the data is already sitting
// there, readable combinationally, on the cycle it's needed. Hits cost
// zero extra cycles.
//
// WHAT THE ZERO-CYCLE HIT COSTS: the array address is data-dependent on the
// instruction currently being fetched, so it closes a combinational loop through
// the fetch path in a single cycle. Since 2026-09-20 this is the design's
// critical path on the Zynq build, at +0.726 ns of 16.667 ns:
//
//   TCM BRAM read                      2.125 ns
//   instruction assembly + RVC          ~0.7     (inst_out[*], 5 LUT levels)
//   branch decode + PHT lookup          ~0.5     (predicted_taken, ghr)
//   branch/JAL target adder             0.343    (CARRY4; b_imm and j_imm sums
//                                                 run in PARALLEL and the mux
//                                                 sits after them - see cpu.v)
//   next-PC mux -> I-cache index adder  ~0.6     (CARRY4 x2 -> esel)
//   ... into this module's ADDRBWRADDR
//
// Two things are worth knowing before trying to shorten it.
//
// It is ROUTING-dominated, not logic-dominated: 9.686 ns of the 14.949 ns
// datapath is net delay across 12 nets, against 5.263 ns of BRAM and LUT/carry
// delay. The path starts at RAMB36_X4Y6 (TCM) and ends at RAMB36_X3Y9 (this
// cache) via ten slices scattered between X54 and X75. So logic-level
// micro-optimisation buys much less here than placement does; a pblock holding
// the TCM, this cache and the fetch/predict cluster together is the first lever.
//
// And the adder-before-mux trick is ALREADY applied on the target path
// (cpu.v's if_pc_plus_b_imm / if_pc_plus_j_imm), so that particular win is spent.
// What remains structural is the loop itself: removing it means a second fetch
// stage, which buys timing back at the cost of a bubble on every taken branch -
// the exact trade this design made in the other direction on purpose.
//
// This distinction is load-bearing, not incidental: tcm.v registers its
// i_ready and therefore charges 1 cycle on EVERY access, which is exactly
// why relocating CoreMark into TCM measured 15M cycles against the cache's
// 9.7M. A memory that cannot miss still loses badly to one that is usually
// a same-cycle hit.
//
// Correctness bar here is higher than the branch predictors': those are
// all re-verified in EX, so a bug costs flush cycles. This feeds actual
// instruction bits into the pipeline, so a bug means running the wrong
// program. Hence spec_ok and S_RECHECK below, both of which trade a cycle
// for a guarantee.
// TARGET SELECTION - the four arrays have two implementations, chosen by
// USE_SRAM_MACRO, from identical surrounding logic:
//
//   0 (default)  inferred Block RAM. FPGA and every testbench.
//   1            six sky130 OpenRAM macros via sram_sky130.v. ASIC.
//
// The macro is physically 512 words x 32 bits, so USE_SRAM_MACRO=1 REQUIRES
// NUM_SETS=512 (8KB here, against 32KB on FPGA). Enforced below rather than
// documented, because a mismatch would be a silent index truncation.
//
// Six macros, because two of the arrays are 64 bits wide:
//   dat_even -> 2   dat_odd -> 2   tagA -> 1   tagB -> 1
//
// Unlike dcache_bram.v this needs NO store->load bypass. The CPU never
// writes this cache; the only writes are refills, and S_RECHECK already
// places the write edge strictly before the next read (see the FSM comment
// below), with fetch_ready gated on state == S_IDLE. Do not add one.
module icache_bram #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS   = 2048,  // 2048 * 16B = 32KB; must be a power of 2
    parameter USE_SRAM_MACRO = 0
) (
    input  wire clk,
    input  wire rst,

    // CPU fetch. fetch_addr is this cycle's pc; fetch_addr_next is the
    // value pc will hold next cycle (== pc while stalled). The arrays are
    // addressed from fetch_addr_next, the tags compared against fetch_addr.
    input  wire fetch_valid,
    input  wire [ADDR_WIDTH-1:0] fetch_addr,
    input  wire [ADDR_WIDTH-1:0] fetch_addr_next,
    output wire [31:0] fetch_rdata,
    output wire fetch_ready,

    // Lower-memory line interface (to mem_arbiter -> AXI)
    output wire mem_req_valid,
    output wire [ADDR_WIDTH-1:0] mem_req_addr,
    input  wire [LINE_BYTES*8-1:0] mem_rline,
    input  wire mem_ready
);
    localparam LINE_BITS = LINE_BYTES * 8;
    localparam OFF  = $clog2(LINE_BYTES);       // 4  byte-offset bits
    localparam IDX  = $clog2(NUM_SETS);         // 11 set-index bits
    localparam TAG  = ADDR_WIDTH - OFF - IDX;   // 17 tag bits
    localparam WIDX = IDX + 2;                  // 13 word-index bits

    localparam [ADDR_WIDTH-1:0] RESET_PC = 32'h4000_0000;

    localparam S_INIT    = 2'd0;
    localparam S_IDLE    = 2'd1;
    localparam S_FILL    = 2'd2;
    localparam S_RECHECK = 2'd3;

    // ---- Arrays ------------------------------------------------------
    // Even/odd halfword banking, borrowed from tcm.v: a 32-bit instruction
    // may start on any 2-byte boundary once RVC is in play, so its two
    // halves can live in different words (and different LINES). Splitting
    // by halfword parity lets both halves be fetched with two independent
    // indices in the same cycle instead of costing a second fetch.
    //
    // Each bank row holds the four SAME-PARITY halfwords of one line, so a
    // refill is one 64-bit write per bank. Four separate 16-bit writes into
    // one array would be a 4-write-port array and would not infer BRAM.
    //
    // {valid, tag} lives in two MIRRORED arrays rather than one true-dual-
    // port array: the straddle case needs tags for line N and line N+1 in
    // the same cycle, and mirroring keeps that as two plain 1-read/1-write
    // arrays instead of depending on TDP read/write collision semantics,
    // which is not something to bet instruction correctness on.
    //
    // All four arrays live in the generate block below. Four INDEPENDENT
    // read addresses (erow, orow, idxA, idxB) is the whole reason there are
    // four of them, and it maps one-to-one onto one read port per macro.

    reg [1:0]            state;
    reg [IDX-1:0]        sweep_cnt;
    reg [ADDR_WIDTH-1:0] miss_addr_q;

    // ---- Cycle N: address the arrays from the speculated next PC -------
    wire [ADDR_WIDTH-1:0] a = fetch_addr_next;

    wire [WIDX-1:0] word   = a[WIDX+1:2];
    // Odd bank supplies the halfword AT `a`; even bank supplies the one
    // after it, which is in the next word when a is 2-byte (not 4-byte)
    // aligned - tcm.v's parity trick, here also carrying into the next set.
    wire [WIDX-1:0] eword  = word + {{(WIDX-1){1'b0}}, a[1]};
    wire [IDX-1:0]  erow   = eword[WIDX-1:2];
    wire [1:0]      esel   = eword[1:0];
    wire [1:0]      osel   = word[1:0];

    wire [IDX-1:0]  idxA   = a[OFF+IDX-1:OFF];
    wire [IDX-1:0]  orow   = idxA;              // == word[WIDX-1:2] by construction
    // The second halfword falls in the NEXT line only when this one is the
    // last halfword of the current line (byte offset 14).
    wire            crossN = &a[OFF-1:1];
    wire [IDX-1:0]  idxB   = idxA + {{(IDX-1){1'b0}}, crossN};

    // ---- Write ports ---------------------------------------------------
    wire wr_en = (state == S_FILL) && mem_ready;
    wire [IDX-1:0] wr_idx = miss_addr_q[OFF+IDX-1:OFF];
    wire [TAG-1:0] wr_tag = miss_addr_q[ADDR_WIDTH-1:OFF+IDX];

    // BRAM contents cannot be bulk-reset, so valid bits are cleared by a
    // post-reset sweep instead. Doubles as a whole-cache flush hook if a
    // fence.i is ever needed.
    wire init_en = (state == S_INIT);
    wire tag_we  = init_en | wr_en;
    wire [IDX-1:0] tag_waddr = init_en ? sweep_cnt : wr_idx;
    wire [TAG:0]   tag_wdata = init_en ? {(TAG+1){1'b0}} : {1'b1, wr_tag};

    // Registered array outputs, driven by whichever implementation the
    // generate selects. On the macro path these ARE the macro output
    // registers; on the BRAM path they are inferred output registers.
    wire [63:0]           q_even, q_odd;
    wire [TAG:0]          q_tagA, q_tagB;
    reg [1:0]             q_esel, q_osel;
    reg                   q_unal;
    reg [ADDR_WIDTH-1:0]  q_a;

    // Refill data, split by halfword parity. Named once so both targets
    // write exactly the same value.
    wire [63:0] wd_even = {mem_rline[3*32    +: 16], mem_rline[2*32    +: 16],
                           mem_rline[1*32    +: 16], mem_rline[0*32    +: 16]};
    wire [63:0] wd_odd  = {mem_rline[3*32+16 +: 16], mem_rline[2*32+16 +: 16],
                           mem_rline[1*32+16 +: 16], mem_rline[0*32+16 +: 16]};

    // ---- The four arrays: one implementation per target ----
    generate
    if (USE_SRAM_MACRO) begin : g_sram
        // The macro is 512 words deep, so erow/orow/idxA/idxB/wr_idx must be
        // exactly 9 bits. A NUM_SETS mismatch would truncate silently at the
        // wrapper port; the unresolvable module name below IS the error.
        if (NUM_SETS != 512) begin : g_depth_check
            icache_NUM_SETS_must_be_512_when_USE_SRAM_MACRO_is_set bad ();
        end

        // A 64-bit bank spans two 32-bit macros, low word then high word.
        // Both halves share the bank's read index, so the straddle still
        // gets even and odd at independent addresses. Refills are
        // full-width, hence wmask 4'hF everywhere.
        sram_sky130 u_de_lo (.clk(clk), .raddr(erow), .rdata(q_even[31:0]),
                             .we(wr_en), .waddr(wr_idx),
                             .wdata(wd_even[31:0]),  .wmask(4'hF));
        sram_sky130 u_de_hi (.clk(clk), .raddr(erow), .rdata(q_even[63:32]),
                             .we(wr_en), .waddr(wr_idx),
                             .wdata(wd_even[63:32]), .wmask(4'hF));
        sram_sky130 u_do_lo (.clk(clk), .raddr(orow), .rdata(q_odd[31:0]),
                             .we(wr_en), .waddr(wr_idx),
                             .wdata(wd_odd[31:0]),   .wmask(4'hF));
        sram_sky130 u_do_hi (.clk(clk), .raddr(orow), .rdata(q_odd[63:32]),
                             .we(wr_en), .waddr(wr_idx),
                             .wdata(wd_odd[63:32]),  .wmask(4'hF));

        // tagA/tagB are mirrors: identical writes, different read index.
        // TAG+1 bits (20 at 512 sets) of a 32-bit word; padding written 0
        // and dropped on read.
        wire [31:0] tagA_rd, tagB_rd;
        sram_sky130 u_tagA (.clk(clk), .raddr(idxA), .rdata(tagA_rd),
                            .we(tag_we), .waddr(tag_waddr),
                            .wdata({{(32-(TAG+1)){1'b0}}, tag_wdata}),
                            .wmask(4'hF));
        sram_sky130 u_tagB (.clk(clk), .raddr(idxB), .rdata(tagB_rd),
                            .we(tag_we), .waddr(tag_waddr),
                            .wdata({{(32-(TAG+1)){1'b0}}, tag_wdata}),
                            .wmask(4'hF));
        assign q_tagA = tagA_rd[TAG:0];
        assign q_tagB = tagB_rd[TAG:0];

    end else begin : g_bram
        (* ram_style = "block" *) reg [63:0]  dat_even [0:NUM_SETS-1];
        (* ram_style = "block" *) reg [63:0]  dat_odd  [0:NUM_SETS-1];
        (* ram_style = "block" *) reg [TAG:0] tagA     [0:NUM_SETS-1];
        (* ram_style = "block" *) reg [TAG:0] tagB     [0:NUM_SETS-1];

        reg [63:0]  q_even_r, q_odd_r;
        reg [TAG:0] q_tagA_r, q_tagB_r;

        always @(posedge clk) begin
            if (wr_en) begin
                dat_even[wr_idx] <= wd_even;
                dat_odd [wr_idx] <= wd_odd;
            end
            if (tag_we) begin
                tagA[tag_waddr] <= tag_wdata;
                tagB[tag_waddr] <= tag_wdata;
            end

            q_even_r <= dat_even[erow];
            q_odd_r  <= dat_odd [orow];
            q_tagA_r <= tagA[idxA];
            q_tagB_r <= tagB[idxB];
        end

        assign q_even = q_even_r;
        assign q_odd  = q_odd_r;
        assign q_tagA = q_tagA_r;
        assign q_tagB = q_tagB_r;
    end
    endgenerate

    // ---- Bookkeeping, identical on both targets ----
    always @(posedge clk) begin
        if (rst) begin
            q_esel <= 2'b0;
            q_osel <= 2'b0;
            q_unal <= 1'b0;
            q_a    <= RESET_PC;
        end else begin
            q_esel <= esel;
            q_osel <= osel;
            q_unal <= a[1];
            q_a    <= a;
        end
    end

    // ---- Cycle N+1: stitch the instruction, check the tags -------------
    wire [15:0] he = q_even[{q_esel, 4'b0000} +: 16];
    wire [15:0] ho = q_odd [{q_osel, 4'b0000} +: 16];
    assign fetch_rdata = q_unal ? {he, ho} : {ho, he};   // same stitch as tcm.v

    wire [ADDR_WIDTH-1:0] fa2 = fetch_addr + 32'd2;
    // NB: not named `cross` - that's a SystemVerilog keyword (covergroup
    // cross coverage) and breaks under -g2012.
    wire cross_line = &fetch_addr[OFF-1:1];

    wire hitA = q_tagA[TAG] && (q_tagA[TAG-1:0] == fetch_addr[ADDR_WIDTH-1:OFF+IDX]);
    wire hitB = q_tagB[TAG] && (q_tagB[TAG-1:0] == fa2[ADDR_WIDTH-1:OFF+IDX]);

    // Confirms the address speculated last cycle is in fact the one being
    // fetched now. Combined with the tag compares above (which cover
    // [31:OFF+IDX]) this checks every address bit, so a speculation bug
    // degrades to one stall cycle rather than executing a wrong
    // instruction. First thing to drop if timing needs the slack back.
    wire spec_ok = (q_a[OFF+IDX-1:1] == fetch_addr[OFF+IDX-1:1]);

    // Deliberately no is_compressed term: requiring hitB on every cross
    // keeps fetch_ready dependent only on REGISTERED tags and registered
    // pc, with no dependence on the BRAM data outputs at all. The cost is
    // occasionally refilling line L+1 for a compressed instruction sitting
    // at offset 14 - a line execution enters next cycle anyway.
    assign fetch_ready = (state == S_IDLE) && fetch_valid && spec_ok &&
                         hitA && (!cross_line || hitB);

    wire [ADDR_WIDTH-1:0] lineA = {fetch_addr[ADDR_WIDTH-1:OFF], {OFF{1'b0}}};
    wire [ADDR_WIDTH-1:0] lineB = {fa2[ADDR_WIDTH-1:OFF], {OFF{1'b0}}};
    wire [ADDR_WIDTH-1:0] miss_addr = !hitA ? lineA : lineB;

    // ---- Refill FSM ----------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_INIT;
            sweep_cnt   <= {IDX{1'b0}};
            miss_addr_q <= {ADDR_WIDTH{1'b0}};
        end else begin
            case (state)
                S_INIT: begin
                    sweep_cnt <= sweep_cnt + 1'b1;
                    // One settle cycle via S_RECHECK before serving hits,
                    // so the last swept entry can never be read back
                    // read-during-write.
                    if (sweep_cnt == {IDX{1'b1}}) state <= S_RECHECK;
                end
                S_IDLE: begin
                    if (fetch_valid && !fetch_ready) begin
                        if (!spec_ok) begin
                            // Not a miss - the speculated address just
                            // didn't match. Re-read rather than fetching a
                            // line that may already be resident.
                            state <= S_RECHECK;
                        end else begin
                            miss_addr_q <= miss_addr;
                            state       <= S_FILL;
                        end
                    end
                end
                S_FILL: begin
                    if (mem_ready) state <= S_RECHECK;
                end
                // Puts the re-read edge strictly after the write edge, so
                // correctness does not depend on the BRAM's read-during-
                // write mode (WRITE_FIRST/READ_FIRST/NO_CHANGE). One cycle
                // on a 10+ cycle miss. A double miss needs no special case:
                // S_IDLE re-evaluates, misses on B, and fills again.
                S_RECHECK: state <= S_IDLE;
                default:   state <= S_IDLE;
            endcase
        end
    end

    assign mem_req_valid = (state == S_FILL);
    assign mem_req_addr  = miss_addr_q;

endmodule

`endif // _ICACHE_BRAM_V_
