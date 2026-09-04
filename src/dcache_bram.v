`timescale 1ns/1ps

// BRAM-backed direct-mapped write-back data cache.
//
// Replaces the cache_core-based dcache, whose arrays were read
// COMBINATIONALLY and so could only become distributed LUTRAM + F7/F8 mux
// trees: measured at 11,064 LUTs and 19,987 FFs, roughly 58% of all LUTs
// and 77% of all FFs in the entire design. Every array here is read inside
// always @(posedge clk), the pattern Xilinx BRAM requires.
//
// Read latency is hidden the same way icache_bram.v does it: the arrays are
// addressed with the address MEM will use NEXT cycle (cpu_req_addr_next,
// derived in EX), so the output registers hold the right line exactly when
// MEM needs it. Zero added stall on a hit.
//
// Direct-mapped is load-bearing, not just simpler: the victim line is the
// line already speculatively read, so writeback needs no extra array read
// and no saved_victim_line. Two-way would need way-select before knowing
// what to write back, plus an LRU array written on every hit.
//
// CORRECTNESS NOTE - this carries program DATA, so unlike the branch
// predictors (where EX re-verifies everything) a bug here silently
// corrupts memory. Two guards exist for that reason: spec_ok, which
// downgrades any address-speculation error to one stall cycle, and the
// store->load bypass below, which covers a genuinely undefined BRAM
// behaviour rather than a mere timing nicety.
// TARGET SELECTION - the five cache arrays have two implementations, chosen
// by USE_SRAM_MACRO, from identical surrounding logic:
//
//   0 (default)  inferred Block RAM. FPGA and every testbench.
//   1            five sky130 OpenRAM macros via sram_sky130.v. ASIC.
//
// The macro is physically 512 words x 32 bits, so USE_SRAM_MACRO=1 REQUIRES
// NUM_SETS=512 (8KB here, against 16KB on FPGA). That is enforced below
// rather than documented, because a mismatch would otherwise show up as a
// silent port-width truncation on the index.
//
// The macro's 1RW+1R port structure fits this cache better than BRAM did:
// the design already reads at one address (a_idx, speculative) and writes at
// another (waddr) in the same cycle, which is exactly the two ports. Reads
// go on port 1 and writes on port 0 - see the header of sram_sky130.v for
// why that assignment is mandatory rather than stylistic.
module dcache_bram #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS   = 1024,  // 1024 * 16B = 16KB; power of 2
    parameter USE_SRAM_MACRO = 0
) (
    input  wire clk,
    input  wire rst,

    // CPU side. cpu_req_addr is the MEM-stage address; cpu_req_addr_next
    // is the index+offset MEM will present next cycle (see cpu.v).
    input  wire cpu_req_valid,
    input  wire cpu_req_write,
    input  wire [ADDR_WIDTH-1:0] cpu_req_addr,
    input  wire [$clog2(NUM_SETS)+$clog2(LINE_BYTES)-1:0] cpu_req_addr_next,
    input  wire [31:0] cpu_wdata,
    input  wire [3:0]  cpu_wmask,
    output wire [31:0] cpu_rdata,
    output wire cpu_ready,

    // Lower-memory line interface
    output wire mem_req_valid,
    output wire mem_req_write,
    output wire [ADDR_WIDTH-1:0] mem_req_addr,
    output wire [LINE_BYTES*8-1:0] mem_wline,
    input  wire [LINE_BYTES*8-1:0] mem_rline,
    input  wire mem_ready
);
    localparam LINE_BITS = LINE_BYTES * 8;
    localparam OFF = $clog2(LINE_BYTES);        // 4
    localparam IDX = $clog2(NUM_SETS);          // 10
    localparam TAG = ADDR_WIDTH - OFF - IDX;    // 18

    localparam S_INIT    = 3'd0;
    localparam S_IDLE    = 3'd1;
    localparam S_CAP     = 3'd2;
    localparam S_WB      = 3'd3;
    localparam S_FILL    = 3'd4;
    localparam S_RECHECK = 3'd5;

    // The four 32-bit data banks and the {valid,tag} array live in the
    // generate block further down - one bank per array rather than a single
    // 128-bit array, so a store touches exactly one bank and uses native
    // byte-write-enables on both targets.
    //
    // Flops on both targets, deliberately: dirty is one bit per set, and it
    // is read COMBINATIONALLY by wb_needed. A registered-read memory (BRAM
    // or macro alike) could not serve that without adding a cycle to the
    // miss path. Not reset - a line's dirty bit is only ever consulted when
    // its valid bit is set, and every line becomes valid via a refill,
    // which clears it.
    reg dirty [0:NUM_SETS-1];

    reg [2:0]       state;
    reg [IDX-1:0]   sweep_cnt;
    reg [IDX-1:0]   miss_idx;
    reg [TAG-1:0]   miss_tag, wb_tag;
    reg [LINE_BITS-1:0] wb_line;

    // ---- Cycle N: address the arrays from the speculated MEM address ----
    wire [IDX-1:0] a_idx = cpu_req_addr_next[OFF+IDX-1:OFF];

    // ---- MEM-stage (cycle N+1) decode ----
    wire [IDX-1:0] m_idx  = cpu_req_addr[OFF+IDX-1:OFF];
    wire [TAG-1:0] m_tag  = cpu_req_addr[ADDR_WIDTH-1:OFF+IDX];
    wire [1:0]     m_bank = cpu_req_addr[OFF-1:2];

    // Registered array outputs, driven by whichever implementation the
    // generate below selects. On the macro path these ARE the macro output
    // registers; on the BRAM path they are inferred output registers. Either
    // way the read is one cycle, which is what the a_idx speculation hides.
    wire [31:0]  q0, q1, q2, q3;
    wire [TAG:0] q_tagv;
    reg  [IDX-1:0] q_a_idx;

    // Confirms the address speculated last cycle is the one MEM actually
    // wants. Combined with the tag compare (which covers [31:OFF+IDX]) every
    // address bit is checked, so a speculation bug costs a stall, not data.
    wire spec_ok = (q_a_idx == m_idx);
    wire hit = q_tagv[TAG] && (q_tagv[TAG-1:0] == m_tag) && spec_ok;

    assign cpu_ready = (state == S_IDLE) && cpu_req_valid && hit;

    // Store commits only on a confirmed hit in S_IDLE - never mid-writeback
    // or mid-refill, which would corrupt a line being evicted.
    wire wr_hit = (state == S_IDLE) && cpu_req_valid && cpu_req_write && hit;

    // ---- Write ports (stores and refills share one address/data mux) ----
    wire fill_en = (state == S_FILL) && mem_ready;
    wire init_en = (state == S_INIT);

    wire [IDX-1:0] waddr = fill_en ? miss_idx : m_idx;

    wire [3:0] be0 = fill_en ? 4'hF : ((wr_hit && m_bank == 2'd0) ? cpu_wmask : 4'h0);
    wire [3:0] be1 = fill_en ? 4'hF : ((wr_hit && m_bank == 2'd1) ? cpu_wmask : 4'h0);
    wire [3:0] be2 = fill_en ? 4'hF : ((wr_hit && m_bank == 2'd2) ? cpu_wmask : 4'h0);
    wire [3:0] be3 = fill_en ? 4'hF : ((wr_hit && m_bank == 2'd3) ? cpu_wmask : 4'h0);

    wire [31:0] wd0 = fill_en ? mem_rline[31:0]   : cpu_wdata;
    wire [31:0] wd1 = fill_en ? mem_rline[63:32]  : cpu_wdata;
    wire [31:0] wd2 = fill_en ? mem_rline[95:64]  : cpu_wdata;
    wire [31:0] wd3 = fill_en ? mem_rline[127:96] : cpu_wdata;

    wire tagv_we = fill_en | init_en;
    wire [IDX-1:0] tagv_waddr = init_en ? sweep_cnt : miss_idx;
    wire [TAG:0]   tagv_wdata = init_en ? {(TAG+1){1'b0}} : {1'b1, miss_tag};

    // ---- Store->load bypass (mandatory, not an optimization) ----
    // A store's write edge and the speculative read for the very next cycle
    // can target the same set. Cross-port read-during-write is UNDEFINED on
    // BOTH targets - on Xilinx SDP BRAM because WRITE_FIRST does not apply
    // across ports, and on the sky130 macro explicitly (its model prints a
    // warning for it). The load would silently see stale data. One-deep is
    // provably sufficient because the output registers re-latch every cycle.
    //
    // DO NOT DELETE THIS ON THE STRENGTH OF A PASSING MACRO-PATH TEST.
    // Deleting the bypass and running Testbenches/tb_dcache_hazard.v gives:
    //
    //     BRAM path (default)   4 of 7 checks FAIL   <- bug caught
    //     macro path (-DUSE_SRAM)  all 7 PASS        <- bug NOT caught
    //
    // The OpenRAM behavioral model schedules its port-0 write and its port-1
    // read on the same negedge, and iverilog happens to order the write
    // first - so the model resolves the undefined race as write-first and
    // masks precisely the failure this bypass exists to prevent. That
    // ordering is a property of one simulator, not a guarantee from the
    // macro. The BRAM mutation above is the only mechanised evidence that
    // the bypass is load-bearing; treat it as covering both targets, because
    // the macro path cannot produce that evidence itself.
    reg        byp_v;
    reg [1:0]  byp_word;
    reg [3:0]  byp_be;
    reg [31:0] byp_data;

    // ---- The five cache arrays: one implementation per target ----
    generate
    if (USE_SRAM_MACRO) begin : g_sram
        // The macro is physically 512 words deep, so a_idx / waddr /
        // tagv_waddr must be exactly 9 bits. A NUM_SETS mismatch would be
        // truncated silently at the wrapper port and mis-index every
        // access, so it is made an elaboration error instead - the
        // unresolvable module name below IS the error message.
        if (NUM_SETS != 512) begin : g_depth_check
            dcache_NUM_SETS_must_be_512_when_USE_SRAM_MACRO_is_set bad ();
        end

        // tagv occupies TAG+1 bits (20 at 512 sets) of a 32-bit macro word.
        // The padding is written as zero and dropped on read; wmask is 4'hF
        // because the word is only ever written whole.
        wire [31:0] tagv_rd;

        sram_sky130 u_d0 (.clk(clk), .raddr(a_idx), .rdata(q0),
                          .we(|be0), .waddr(waddr), .wdata(wd0), .wmask(be0));
        sram_sky130 u_d1 (.clk(clk), .raddr(a_idx), .rdata(q1),
                          .we(|be1), .waddr(waddr), .wdata(wd1), .wmask(be1));
        sram_sky130 u_d2 (.clk(clk), .raddr(a_idx), .rdata(q2),
                          .we(|be2), .waddr(waddr), .wdata(wd2), .wmask(be2));
        sram_sky130 u_d3 (.clk(clk), .raddr(a_idx), .rdata(q3),
                          .we(|be3), .waddr(waddr), .wdata(wd3), .wmask(be3));

        sram_sky130 u_tagv (.clk(clk), .raddr(a_idx), .rdata(tagv_rd),
                            .we(tagv_we), .waddr(tagv_waddr),
                            .wdata({{(32-(TAG+1)){1'b0}}, tagv_wdata}),
                            .wmask(4'hF));

        assign q_tagv = tagv_rd[TAG:0];

    end else begin : g_bram
        (* ram_style = "block" *) reg [31:0]  d0 [0:NUM_SETS-1];
        (* ram_style = "block" *) reg [31:0]  d1 [0:NUM_SETS-1];
        (* ram_style = "block" *) reg [31:0]  d2 [0:NUM_SETS-1];
        (* ram_style = "block" *) reg [31:0]  d3 [0:NUM_SETS-1];
        // Valid lives in the BRAM word so the hit compare is
        // register-to-register; BRAM can't be bulk-reset, hence S_INIT.
        (* ram_style = "block" *) reg [TAG:0] tagv [0:NUM_SETS-1];

        reg [31:0]  q0_r, q1_r, q2_r, q3_r;
        reg [TAG:0] q_tagv_r;

        integer b;
        always @(posedge clk) begin
            q0_r     <= d0[a_idx];
            q1_r     <= d1[a_idx];
            q2_r     <= d2[a_idx];
            q3_r     <= d3[a_idx];
            q_tagv_r <= tagv[a_idx];

            for (b = 0; b < 4; b = b + 1) begin
                if (be0[b]) d0[waddr][b*8 +: 8] <= wd0[b*8 +: 8];
                if (be1[b]) d1[waddr][b*8 +: 8] <= wd1[b*8 +: 8];
                if (be2[b]) d2[waddr][b*8 +: 8] <= wd2[b*8 +: 8];
                if (be3[b]) d3[waddr][b*8 +: 8] <= wd3[b*8 +: 8];
            end

            if (tagv_we) tagv[tagv_waddr] <= tagv_wdata;
        end

        assign q0 = q0_r;
        assign q1 = q1_r;
        assign q2 = q2_r;
        assign q3 = q3_r;
        assign q_tagv = q_tagv_r;
    end
    endgenerate

    // ---- Bookkeeping, identical on both targets ----
    always @(posedge clk) begin
        q_a_idx <= a_idx;

        // Refills clear dirty; stores and AMO writes set it. Missing the
        // AMO case would silently lose the atomic update.
        if (fill_en)      dirty[miss_idx] <= 1'b0;
        else if (wr_hit)  dirty[m_idx]    <= 1'b1;

        // Refill writes are covered by S_RECHECK, so never bypass them -
        // doing so would forward stale store data over a fresh line.
        if (rst) begin
            byp_v <= 1'b0;
        end else begin
            byp_v    <= wr_hit && !fill_en && (m_idx == a_idx);
            byp_word <= m_bank;
            byp_be   <= cpu_wmask;
            byp_data <= cpu_wdata;
        end
    end

    // ---- Read data: select the word, then apply the bypass per byte ----
    reg [31:0] qsel;
    always @(*) begin
        case (m_bank)
            2'd0:    qsel = q0;
            2'd1:    qsel = q1;
            2'd2:    qsel = q2;
            default: qsel = q3;
        endcase
    end

    wire byp_use = byp_v && (byp_word == m_bank);

    genvar gb;
    generate
        for (gb = 0; gb < 4; gb = gb + 1) begin : byp_byte
            assign cpu_rdata[gb*8 +: 8] = (byp_use && byp_be[gb]) ? byp_data[gb*8 +: 8]
                                                                  : qsel[gb*8 +: 8];
        end
    endgenerate

    // ---- FSM ----
    // The victim line needed for writeback is whatever the arrays hold at
    // the miss index. Because a miss asserts cpu_ready=0 (and hence
    // global_mem_stall), the pipeline freezes, cpu_req_addr stays put, and
    // the arrays are re-read from it - so by S_CAP the output registers
    // hold the victim, captured strictly after any store's write edge.
    wire wb_needed = q_tagv[TAG] && dirty[m_idx];

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_INIT;
            sweep_cnt <= {IDX{1'b0}};
            miss_idx  <= {IDX{1'b0}};
            miss_tag  <= {TAG{1'b0}};
        end else begin
            case (state)
                S_INIT: begin
                    sweep_cnt <= sweep_cnt + 1'b1;
                    // One settle cycle so the last swept entry can't be
                    // read back read-during-write.
                    if (sweep_cnt == {IDX{1'b1}}) state <= S_RECHECK;
                end
                S_IDLE: begin
                    if (cpu_req_valid && !hit) begin
                        if (!spec_ok) begin
                            // Not a miss - the speculated address simply
                            // didn't match. Re-read; don't fetch a line
                            // that may already be resident.
                            state <= S_RECHECK;
                        end else begin
                            miss_idx <= m_idx;
                            miss_tag <= m_tag;
                            state    <= S_CAP;
                        end
                    end
                end
                S_CAP: begin
                    wb_line <= {q3, q2, q1, q0};
                    wb_tag  <= q_tagv[TAG-1:0];
                    state   <= wb_needed ? S_WB : S_FILL;
                end
                S_WB: begin
                    if (mem_ready) state <= S_FILL;
                end
                S_FILL: begin
                    if (mem_ready) state <= S_RECHECK;
                end
                // Puts the write edge strictly before the re-read edge, so
                // correctness doesn't depend on BRAM read-during-write mode.
                S_RECHECK: state <= S_IDLE;
                default:   state <= S_IDLE;
            endcase
        end
    end

    assign mem_req_valid = (state == S_WB) || (state == S_FILL);
    assign mem_req_write = (state == S_WB);
    assign mem_req_addr  = (state == S_WB) ? {wb_tag,   miss_idx, {OFF{1'b0}}}
                                           : {miss_tag, miss_idx, {OFF{1'b0}}};
    assign mem_wline     = wb_line;

endmodule
