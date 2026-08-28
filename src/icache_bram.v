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
module icache_bram #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS   = 2048   // 2048 * 16B = 32KB; must be a power of 2
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
    (* ram_style = "block" *) reg [63:0]  dat_even [0:NUM_SETS-1];
    (* ram_style = "block" *) reg [63:0]  dat_odd  [0:NUM_SETS-1];
    // {valid, tag}. Two MIRRORED arrays rather than one true-dual-port
    // array: the straddle case needs tags for line N and line N+1 in the
    // same cycle, and mirroring keeps that as two plain 1-read/1-write
    // arrays instead of depending on TDP read/write collision semantics,
    // which is not something to bet instruction correctness on.
    (* ram_style = "block" *) reg [TAG:0] tagA     [0:NUM_SETS-1];
    (* ram_style = "block" *) reg [TAG:0] tagB     [0:NUM_SETS-1];

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

    reg [63:0]            q_even, q_odd;
    reg [TAG:0]           q_tagA, q_tagB;
    reg [1:0]             q_esel, q_osel;
    reg                   q_unal;
    reg [ADDR_WIDTH-1:0]  q_a;

    always @(posedge clk) begin
        if (wr_en) begin
            dat_even[wr_idx] <= {mem_rline[3*32    +: 16], mem_rline[2*32    +: 16],
                                 mem_rline[1*32    +: 16], mem_rline[0*32    +: 16]};
            dat_odd [wr_idx] <= {mem_rline[3*32+16 +: 16], mem_rline[2*32+16 +: 16],
                                 mem_rline[1*32+16 +: 16], mem_rline[0*32+16 +: 16]};
        end
        if (tag_we) begin
            tagA[tag_waddr] <= tag_wdata;
            tagB[tag_waddr] <= tag_wdata;
        end

        q_even <= dat_even[erow];
        q_odd  <= dat_odd [orow];
        q_tagA <= tagA[idxA];
        q_tagB <= tagB[idxB];

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
