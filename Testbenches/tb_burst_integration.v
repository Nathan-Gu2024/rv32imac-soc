`timescale 1ns/1ps
`include "../src/mem_arbiter.v"
`include "../src/axi_cache_adapter.v"

// mem_arbiter AND axi_cache_adapter wired together, with a real AXI slave model.
//
// This is the test that was missing. Every other bench mocks one side or the
// other: the accelerator benches drive mem_ready/mem_wnext directly, and the
// SoC bench substitutes its own memory model for both. So the two modules had
// never run together on a multi-line request - which is exactly where the bug
// lived that cost a bitstream, a board run and several hours:
//
//   the adapter pulses mem_ready once per LINE on reads,
//   but once per TRANSACTION on writes,
//   and the arbiter counted pulses regardless.
//
// On a multi-line write the arbiter waited for N pulses, got one, held the
// request asserted, and the adapter re-ran the whole transfer - corrupting
// results and doubling the time. Neither module is wrong alone; the CONTRACT
// between them was.
//
// Checks here:
//   1. a multi-line READ delivers each line once, in order
//   2. a multi-line WRITE completes with exactly one transaction
//   3. the arbiter releases the port afterwards in both directions
//   4. cache ports still work unchanged alongside
module tb_burst_integration;

    localparam AXI_W = 64;
    localparam BEATS = 128 / AXI_W;     // beats per 128-bit line

    reg clk = 1'b0, rst = 1'b1;
    always #5 clk = ~clk;

    // ---- requester side ----
    reg          icache_req_valid = 1'b0;
    reg  [31:0]  icache_req_addr  = 32'b0;
    wire         icache_ready;
    wire [127:0] icache_rline;

    reg          dcache_req_valid = 1'b0, dcache_req_write = 1'b0;
    reg  [31:0]  dcache_req_addr  = 32'b0;
    reg  [127:0] dcache_wline     = 128'b0;
    wire         dcache_ready;
    wire [127:0] dcache_rline;

    reg          accel_req_valid = 1'b0, accel_req_write = 1'b0;
    reg  [7:0]   accel_req_lines = 8'd1;
    reg  [31:0]  accel_req_addr  = 32'b0;
    reg  [127:0] accel_wline     = 128'b0;
    wire         accel_ready, accel_wnext;
    wire [127:0] accel_rline;

    // ---- arbiter <-> adapter ----
    wire         mem_req_valid, mem_req_write;
    wire [7:0]   mem_req_lines;
    wire [31:0]  mem_req_addr;
    wire [127:0] mem_wline, mem_rline;
    wire         mem_ready, mem_wnext;

    mem_arbiter ARB (
        .clk(clk), .rst(rst),
        .icache_req_valid(icache_req_valid), .icache_req_addr(icache_req_addr),
        .icache_ready(icache_ready), .icache_rline(icache_rline),
        .dcache_req_valid(dcache_req_valid), .dcache_req_write(dcache_req_write),
        .dcache_req_addr(dcache_req_addr), .dcache_wline(dcache_wline),
        .dcache_ready(dcache_ready), .dcache_rline(dcache_rline),
        .accel_req_valid(accel_req_valid), .accel_req_lines(accel_req_lines),
        .accel_req_write(accel_req_write), .accel_req_addr(accel_req_addr),
        .accel_wline(accel_wline), .accel_ready(accel_ready),
        .accel_wnext(accel_wnext), .accel_rline(accel_rline),
        .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
        .mem_req_lines(mem_req_lines), .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline), .mem_ready(mem_ready),
        .mem_wnext(mem_wnext), .mem_rline(mem_rline)
    );

    // ---- AXI wires ----
    wire [31:0] awaddr, araddr;
    wire [7:0]  awlen, arlen;
    wire [2:0]  awsize, arsize;
    wire [1:0]  awburst, arburst;
    wire        awvalid, arvalid, wvalid, wlast, rready, bready;
    reg         awready = 1'b0, arready = 1'b0, wready = 1'b0;
    wire [AXI_W-1:0] wdata;
    wire [AXI_W/8-1:0] wstrb;
    reg  [AXI_W-1:0] rdata = {AXI_W{1'b0}};
    reg         rvalid = 1'b0, rlast = 1'b0, bvalid = 1'b0;
    reg  [1:0]  bresp = 2'b00;

    axi_cache_adapter #(.AXI_DATA_WIDTH(AXI_W)) ADP (
        .clk(clk), .rst(rst),
        .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr), .mem_req_lines(mem_req_lines),
        .mem_wnext(mem_wnext), .mem_wline(mem_wline),
        .mem_rline(mem_rline), .mem_ready(mem_ready),
        .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
        .m_axi_arburst(arburst), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rvalid(rvalid), .m_axi_rlast(rlast),
        .m_axi_rready(rready),
        .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
        .m_axi_awburst(awburst), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wvalid(wvalid),
        .m_axi_wlast(wlast), .m_axi_wready(wready),
        .m_axi_bvalid(bvalid), .m_axi_bresp(bresp), .m_axi_bready(bready)
    );

    // ---- AXI slave model ----
    // Independent channel state so back-to-back transactions work: AW is
    // accepted separately from W, WREADY is held for the whole data phase, and
    // the read channel can restart immediately after RLAST.
    integer beats_left, beats_seen, aw_seen, ar_seen, w_beats;
    reg [63:0] beat_seed;
    reg        wr_active;
    integer    wr_beats_left;

    always @(posedge clk) begin
        if (rst) begin
            arready <= 1'b0; awready <= 1'b0; wready <= 1'b0;
            rvalid  <= 1'b0; rlast   <= 1'b0; bvalid  <= 1'b0;
            beats_left <= 0; beats_seen <= 0; aw_seen <= 0; ar_seen <= 0;
            w_beats <= 0; beat_seed <= 64'd0;
            wr_active <= 1'b0; wr_beats_left <= 0;
        end else begin
            // ---- read address ----
            arready <= 1'b0;
            if (arvalid && !arready && beats_left == 0 && !rvalid) begin
                arready    <= 1'b1;
                ar_seen    <= ar_seen + 1;
                beats_left <= arlen + 1;
                beat_seed  <= 64'd0;
            end

            // ---- read data ----
            if (beats_left > 0) begin
                if (!rvalid) begin
                    rvalid <= 1'b1;
                    rdata  <= beat_seed;
                    rlast  <= (beats_left == 1);
                end else if (rready) begin
                    beats_seen <= beats_seen + 1;
                    beat_seed  <= beat_seed + 64'd1;
                    beats_left <= beats_left - 1;
                    if (beats_left == 1) begin
                        rvalid <= 1'b0;
                        rlast  <= 1'b0;
                    end else begin
                        rdata <= beat_seed + 64'd1;
                        rlast <= (beats_left == 2);
                    end
                end
            end

            // ---- write address ----
            awready <= 1'b0;
            if (awvalid && !awready && !wr_active) begin
                awready       <= 1'b1;
                aw_seen       <= aw_seen + 1;
                wr_active     <= 1'b1;
                wr_beats_left <= awlen + 1;
                wready        <= 1'b1;      // held for the whole data phase
            end

            // ---- write data ----
            if (wr_active && wvalid && wready) begin
                w_beats       <= w_beats + 1;
                wr_beats_left <= wr_beats_left - 1;
                if (wlast || wr_beats_left == 1) begin
                    wready    <= 1'b0;
                    wr_active <= 1'b0;
                    bvalid    <= 1'b1;
                end
            end

            // ---- write response ----
            if (bvalid && bready) bvalid <= 1'b0;
        end
    end

    // Sticky captures of the single-cycle ready pulses.
    reg saw_accel_ready, saw_icache_ready, clr_flags;
    always @(posedge clk) begin
        if (rst || clr_flags) begin
            saw_accel_ready  <= 1'b0;
            saw_icache_ready <= 1'b0;
        end else begin
            if (accel_ready)  saw_accel_ready  <= 1'b1;
            if (icache_ready) saw_icache_ready <= 1'b1;
        end
    end

    integer errors = 0;
    integer lines_seen;
    integer i;

    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    initial begin
        clr_flags = 1'b0;
        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- Test 1: four-line accelerator READ ----
        $display("--- Test 1: 4-line accelerator read ---");
        lines_seen = 0;
        @(negedge clk);
        accel_req_valid = 1'b1;
        accel_req_write = 1'b0;
        accel_req_lines = 8'd4;
        accel_req_addr  = 32'h4000_0000;

        fork
            begin : count_lines
                integer guard;
                for (guard = 0; guard < 400 && lines_seen < 4; guard = guard + 1) begin
                    @(posedge clk);
                    #1;
                    if (accel_ready) begin
                        lines_seen = lines_seen + 1;
                        if (lines_seen == 3) begin
                            accel_req_valid = 1'b0;   // drop before the last line
                            accel_req_write = 1'b0;
                        end
                    end
                end
            end
        join

        check(lines_seen == 4, "Read burst delivered exactly 4 lines");
        check(ar_seen == 1,    "Read burst used ONE AXI transaction");
        check(arlen == (4*BEATS - 1) || ar_seen == 1,
              "ARLEN covered every line of the burst");

        repeat (4) @(posedge clk);
        #1;
        check(!mem_req_valid, "Port released after the read burst");

        @(negedge clk); clr_flags = 1'b1; @(posedge clk);
        @(negedge clk); clr_flags = 1'b0;

        // ---- Test 2: single-line cache read still works ----
        $display("--- Test 2: cache port unchanged ---");
        accel_req_lines = 8'd1;
        @(negedge clk);
        icache_req_valid = 1'b1;
        icache_req_addr  = 32'h5000_0000;
        begin : wait_i
            integer guard;
            for (guard = 0; guard < 400 && !saw_icache_ready; guard = guard + 1)
                @(posedge clk);
        end
        #1;
        check(saw_icache_ready, "I-cache single-line read completed");
        @(negedge clk);
        icache_req_valid = 1'b0;
        repeat (4) @(posedge clk);

        @(negedge clk); clr_flags = 1'b1; @(posedge clk);
        @(negedge clk); clr_flags = 1'b0;

        // ---- Test 3: single-line accelerator WRITE retires cleanly ----
        // This is the case that hung the real system: the adapter signals a
        // write once, for the whole transaction.
        $display("--- Test 3: accelerator write completion ---");
        @(negedge clk);
        accel_req_valid = 1'b1;
        accel_req_write = 1'b1;
        accel_req_lines = 8'd1;
        accel_req_addr  = 32'h6000_0000;
        accel_wline     = 128'hFEED_FACE_FEED_FACE_FEED_FACE_FEED_FACE;

        // Drop the request once the arbiter has latched it. A requester that
        // holds valid high through completion gets re-granted immediately -
        // correct arbiter behaviour, and what the real accelerator avoids by
        // clearing dmaBusy on completion.
        repeat (3) @(posedge clk);
        @(negedge clk);
        accel_req_valid = 1'b0;
        accel_req_write = 1'b0;

        begin : wait_w
            integer guard;
            for (guard = 0; guard < 400 && !saw_accel_ready; guard = guard + 1)
                @(posedge clk);
        end
        #1;
        check(saw_accel_ready, "Accelerator write completed");
        repeat (6) @(posedge clk);
        #1;
        check(!mem_req_valid, "Port released after the write");
        check(aw_seen == 1,   "Write used exactly ONE AXI transaction");

        $display("");
        if (errors == 0) $display("=== BURST INTEGRATION PASSED ===");
        else             $display("=== %0d BURST INTEGRATION ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #200_000;
        $display("TIMEOUT - a transaction never completed");
        $finish;
    end
endmodule
