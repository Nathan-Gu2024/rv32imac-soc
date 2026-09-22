`timescale 1ns/1ps
`include "../src/axi_rw_engine.v"

// axi_rw_engine against an AXI slave that serves reads and writes concurrently.
//
// The claim under test is not "reads work" or "writes work" - axi_cache_adapter
// already did both. It is that a read and a write can be IN FLIGHT AT THE SAME
// TIME over one AXI master, which axi_cache_adapter cannot do because its single
// FSM branches IDLE to one or the other.
//
// So Test 3 is the real test, and it checks two separate things:
//   * both transfers deliver correct data, and
//   * the engines were actually non-idle together - otherwise the test would
//     pass just as well on a design that quietly serialised them, which is
//     exactly the bug it exists to catch.
module tb_rw_engine;
    localparam AXI_W = 64;
    localparam BEATS = 128 / AXI_W;

    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    // ---- upstream read channel ----
    reg         rd_req_valid = 0;
    reg  [31:0] rd_req_addr  = 0;
    reg  [7:0]  rd_req_lines = 0;
    wire [127:0] rd_rline;
    wire        rd_ready;

    // ---- upstream write channel ----
    reg         wr_req_valid = 0;
    reg  [31:0] wr_req_addr  = 0;
    reg  [7:0]  wr_req_lines = 0;
    reg  [127:0] wr_wline    = 0;
    wire        wr_next, wr_ready;

    // ---- AXI ----
    wire [31:0] araddr, awaddr;
    wire [7:0]  arlen, awlen;
    wire [2:0]  arsize, awsize;
    wire [1:0]  arburst, awburst;
    wire        arvalid, awvalid, rready, wvalid, wlast, bready;
    wire [AXI_W-1:0] wdata;
    wire [AXI_W/8-1:0] wstrb;
    reg         arready = 0, awready = 0, wready = 0;
    reg  [AXI_W-1:0] rdata = 0;
    reg         rvalid = 0, rlast = 0, bvalid = 0;
    reg  [1:0]  bresp = 0;
    wire [15:0] rd_retries, wr_retries;

    axi_rw_engine #(.AXI_DATA_WIDTH(AXI_W), .TIMEOUT_LIMIT(16'd100)) DUT (
        .clk(clk), .rst(rst),
        .rd_req_valid(rd_req_valid), .rd_req_addr(rd_req_addr),
        .rd_req_lines(rd_req_lines), .rd_rline(rd_rline), .rd_ready(rd_ready),
        .wr_req_valid(wr_req_valid), .wr_req_addr(wr_req_addr),
        .wr_req_lines(wr_req_lines), .wr_wline(wr_wline),
        .wr_next(wr_next), .wr_ready(wr_ready),
        .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
        .m_axi_arburst(arburst), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rvalid(rvalid), .m_axi_rlast(rlast),
        .m_axi_rready(rready),
        .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
        .m_axi_awburst(awburst), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .rd_retries(rd_retries), .wr_retries(wr_retries)
    );

    // ---------------- slave: reads and writes served independently ----------
    reg [AXI_W-1:0] mem [0:2047];
    integer mi;

    // read side
    reg        rd_active = 0;
    reg [31:0] r_addr;
    reg [8:0]  r_left;
    reg [3:0]  r_lat;
    // write side
    reg        wr_active = 0;
    reg [31:0] w_addr;
    reg [8:0]  w_left;
    integer    w_beats_seen = 0;
    // Transaction counts, for the held-valid test: a duplicate transaction shows
    // up as a second AW/AR acceptance, which is the only unambiguous evidence.
    integer    aw_count = 0, ar_count = 0;
    // Swallows BRESP, to provoke the lost-BRESP timeout in Test 5.
    reg        force_bstall = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            arready <= 0; rvalid <= 0; rlast <= 0; rd_active <= 0;
            awready <= 0; wready <= 0; bvalid <= 0; wr_active <= 0;
            w_beats_seen <= 0;
        end else begin
            // ---- AR / R, entirely independent of the write side ----
            arready <= 1'b0;
            if (arvalid && !arready && !rd_active) begin
                arready   <= 1'b1;
                ar_count  <= ar_count + 1;
                rd_active <= 1'b1;
                r_addr    <= araddr;
                r_left    <= arlen + 1;
                r_lat     <= 3;
                rvalid    <= 1'b0;
            end else if (rd_active && !rvalid) begin
                if (r_lat == 0) begin
                    rvalid <= 1'b1;
                    rdata  <= mem[r_addr[13:3]];
                    rlast  <= (r_left == 1);
                end else r_lat <= r_lat - 1;
            end else if (rd_active && rvalid && rready) begin
                r_addr <= r_addr + (AXI_W/8);
                r_left <= r_left - 1;
                if (r_left == 1) begin
                    rvalid <= 1'b0; rlast <= 1'b0; rd_active <= 1'b0;
                end else begin
                    rdata <= mem[(r_addr + (AXI_W/8)) >> 3 & 11'h7FF];
                    rlast <= (r_left == 2);
                end
            end

            // ---- AW / W / B, entirely independent of the read side ----
            awready <= 1'b0;
            if (awvalid && !awready && !wr_active) begin
                awready   <= 1'b1;
                aw_count  <= aw_count + 1;
                wr_active <= 1'b1;
                w_addr    <= awaddr;
                w_left    <= awlen + 1;
                wready    <= 1'b1;
            end
            if (wr_active && wvalid && wready) begin
                mem[w_addr[13:3]] <= wdata;
                w_beats_seen <= w_beats_seen + 1;
                w_addr <= w_addr + (AXI_W/8);
                w_left <= w_left - 1;
                if (wlast || w_left == 1) begin
                    wready <= 1'b0; wr_active <= 1'b0; bvalid <= 1'b1;
                end
            end
            if (bvalid && bready) bvalid <= 1'b0;
            // Last word on bvalid, so it overrides the assertion after WLAST.
            if (force_bstall) bvalid <= 1'b0;
        end
    end

    // ---- overlap observation: both engines non-idle on the same cycle ----
    integer overlap_cycles = 0;
    always @(posedge clk) begin
        if (!rst && DUT.rd_state != 2'd0 && DUT.wr_state != 3'd0)
            overlap_cycles = overlap_cycles + 1;
    end

    integer errors = 0;
    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    // Sticky capture of the completion pulses.
    //
    // wr_ready is combinational on WR_DONE and lasts exactly ONE cycle, so a
    // procedural `while (!wr_ready)` poll races against it. That poll only ever
    // worked because a held valid used to start DUPLICATE transactions, giving
    // it repeated pulses to catch - i.e. the bench was relying on the very defect
    // the served flag removes. Latch the pulse instead.
    reg wr_done_seen = 1'b0, clr_done = 1'b0;
    always @(posedge clk) begin
        if (clr_done) wr_done_seen <= 1'b0;
        else if (!rst && wr_ready) wr_done_seen <= 1'b1;
    end

    // ---- capture delivered read lines ----
    reg [127:0] RLINE [0:63];
    integer rl_n = 0;
    always @(posedge clk) if (!rst && rd_ready) begin
        RLINE[rl_n] = rd_rline;
        rl_n = rl_n + 1;
    end

    integer i, bad;
    reg [15:0] ln;

    initial begin
        for (mi = 0; mi < 2048; mi = mi + 1) mem[mi] = {2{16'hDEAD, mi[15:0]}};

        repeat (4) @(posedge clk);
        rst = 0;
        @(negedge clk);

        // ---------------- Test 1: 4-line read ----------------
        $display("--- Test 1: 4-line read ---");
        rl_n = 0;
        rd_req_addr = 32'h0000_1000; rd_req_lines = 8'd4; rd_req_valid = 1'b1;
        wait (rl_n == 4);
        @(negedge clk); rd_req_valid = 1'b0;
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (RLINE[i][63:0]   !== mem[(32'h1000 >> 3) + 2*i])   bad = bad + 1;
            if (RLINE[i][127:64] !== mem[(32'h1000 >> 3) + 2*i+1]) bad = bad + 1;
        end
        check(bad == 0, "4-line read delivered the right bytes in order");

        // ---------------- Test 2: 4-line write ----------------
        $display("--- Test 2: 4-line write ---");
        @(negedge clk);
        ln = 16'd0;
        wr_wline = {16'hA1A1, ln, 16'hB2B2, ln, 16'hC3C3, ln, 16'hD4D4, ln};
        @(negedge clk); clr_done = 1'b1; @(negedge clk); clr_done = 1'b0;
        wr_req_addr = 32'h0000_2000; wr_req_lines = 8'd4; wr_req_valid = 1'b1;
        fork
            begin
                while (!wr_done_seen) begin
                    @(posedge clk); #1;
                    if (wr_next) begin
                        ln = ln + 16'd1;
                        wr_wline = {16'hA1A1, ln, 16'hB2B2, ln,
                                    16'hC3C3, ln, 16'hD4D4, ln};
                    end
                end
            end
        join
        @(negedge clk); wr_req_valid = 1'b0;
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h2000 >> 3) + 2*i]   !== {16'hC3C3, i[15:0], 16'hD4D4, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h2000 >> 3) + 2*i+1] !== {16'hA1A1, i[15:0], 16'hB2B2, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "4-line write placed every line at its own address");

        // ---------------- Test 3: concurrent read AND write ----------------
        // The point of the module. Both are issued on the same cycle.
        $display("--- Test 3: read and write CONCURRENTLY ---");
        overlap_cycles = 0;
        rl_n = 0;
        @(negedge clk);
        ln = 16'd0;
        wr_wline = {16'h5555, ln, 16'h6666, ln, 16'h7777, ln, 16'h8888, ln};
        rd_req_addr = 32'h0000_3000; rd_req_lines = 8'd4; rd_req_valid = 1'b1;
        @(negedge clk); clr_done = 1'b1; @(negedge clk); clr_done = 1'b0;
        wr_req_addr = 32'h0000_2800; wr_req_lines = 8'd4; wr_req_valid = 1'b1;
        fork
            begin : feedw
                while (!wr_done_seen) begin
                    @(posedge clk); #1;
                    if (wr_next) begin
                        ln = ln + 16'd1;
                        wr_wline = {16'h5555, ln, 16'h6666, ln,
                                    16'h7777, ln, 16'h8888, ln};
                    end
                end
                @(negedge clk); wr_req_valid = 1'b0;
            end
            begin : waitr
                wait (rl_n == 4);
                @(negedge clk); rd_req_valid = 1'b0;
            end
        join
        repeat (10) @(posedge clk);

        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (RLINE[i][63:0]   !== mem[(32'h3000 >> 3) + 2*i])   bad = bad + 1;
            if (RLINE[i][127:64] !== mem[(32'h3000 >> 3) + 2*i+1]) bad = bad + 1;
        end
        check(bad == 0, "concurrent read still delivered correct data");

        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h2800 >> 3) + 2*i]   !== {16'h7777, i[15:0], 16'h8888, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h2800 >> 3) + 2*i+1] !== {16'h5555, i[15:0], 16'h6666, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "concurrent write still placed every line correctly");
        if (bad != 0) begin
            $display("        w_beats_seen=%0d wr_retries=%0d", w_beats_seen,
                     wr_retries);
            for (i = 0; i < 4; i = i + 1)
                $display("        line %0d got %h %h", i,
                         mem[(32'h2800 >> 3) + 2*i],
                         mem[(32'h2800 >> 3) + 2*i+1]);
        end

        // Without this the test would pass on a design that serialised them.
        check(overlap_cycles > 0,
              "the two engines were genuinely in flight together");
        $display("    overlap cycles = %0d", overlap_cycles);

        // ---------------- Test 4: requester HOLDS valid past completion --------
        // A requester is entitled to hold valid until it observes ready, so the
        // engine sees valid still high on the completion cycle. If it treats that
        // as a fresh request it runs a DUPLICATE transaction - and for a write
        // that re-sends from a FIFO the requester has already drained, which is
        // how tb_mm_accel_c_test's model came to stamp line 0 across a whole
        // result buffer. One AW acceptance is the only acceptable answer.
        $display("--- Test 4: valid held past completion -> ONE transaction ---");
        aw_count = 0; ar_count = 0;
        @(negedge clk);
        ln = 16'd0;
        wr_wline = {16'hE1E1, ln, 16'hE2E2, ln, 16'hE3E3, ln, 16'hE4E4, ln};
        @(negedge clk); clr_done = 1'b1; @(negedge clk); clr_done = 1'b0;
        wr_req_addr = 32'h0000_3800; wr_req_lines = 8'd4; wr_req_valid = 1'b1;
        fork
            begin
                while (!wr_done_seen) begin
                    @(posedge clk); #1;
                    if (wr_next) begin
                        ln = ln + 16'd1;
                        wr_wline = {16'hE1E1, ln, 16'hE2E2, ln,
                                    16'hE3E3, ln, 16'hE4E4, ln};
                    end
                end
            end
        join
        // Deliberately do NOT drop valid for a while.
        repeat (20) @(posedge clk);
        check(aw_count == 1, "held valid produced exactly ONE write transaction");
        if (aw_count != 1) $display("        aw_count = %0d", aw_count);
        @(negedge clk); wr_req_valid = 1'b0;
        repeat (4) @(posedge clk);
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h3800 >> 3) + 2*i]   !== {16'hE3E3, i[15:0], 16'hE4E4, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h3800 >> 3) + 2*i+1] !== {16'hE1E1, i[15:0], 16'hE2E2, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "and memory was not corrupted by a replay");

        // Read twin: hold valid past the last line.
        aw_count = 0; ar_count = 0; rl_n = 0;
        @(negedge clk);
        rd_req_addr = 32'h0000_1800; rd_req_lines = 8'd4; rd_req_valid = 1'b1;
        wait (rl_n == 4);
        repeat (20) @(posedge clk);
        check(ar_count == 1, "held valid produced exactly ONE read transaction");
        check(rl_n == 4, "and exactly 4 lines were delivered, not 8");
        if (ar_count != 1 || rl_n != 4)
            $display("        ar_count = %0d, lines = %0d", ar_count, rl_n);
        @(negedge clk); rd_req_valid = 1'b0;

        // ---------------- Test 5: served flag must not break the RETRY --------
        // The timeout path deliberately re-presents the SAME request with valid
        // still HIGH. So any hardening phrased as "require a valid-low cycle" or
        // "accept only on a rising edge of valid" deadlocks it - which is exactly
        // what the first draft of the served flag would have done. This test is
        // here to catch that, not to test the timeout itself.
        $display("--- Test 5: retry still works with valid held high ---");
        aw_count = 0;
        @(negedge clk); clr_done = 1'b1; @(negedge clk); clr_done = 1'b0;
        ln = 16'd0;
        wr_wline = {16'hF1F1, ln, 16'hF2F2, ln, 16'hF3F3, ln, 16'hF4F4, ln};
        force_bstall = 1'b1;              // swallow BRESP -> lost-BRESP timeout
        wr_req_addr = 32'h0000_2400; wr_req_lines = 8'd4; wr_req_valid = 1'b1;
        fork
            begin
                while (!wr_done_seen) begin
                    @(posedge clk); #1;
                    if (wr_next) begin
                        ln = ln + 16'd1;
                        wr_wline = {16'hF1F1, ln, 16'hF2F2, ln,
                                    16'hF3F3, ln, 16'hF4F4, ln};
                    end
                end
            end
            begin
                repeat (400) @(posedge clk);   // bounded, so a hang is visible
            end
        join_any
        check(wr_done_seen,
              "transfer completed despite the lost BRESP (no deadlock)");
        check(wr_retries > 16'd0, "and the timeout actually fired");
        force_bstall = 1'b0;
        @(negedge clk); wr_req_valid = 1'b0;
        repeat (4) @(posedge clk);
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h2400 >> 3) + 2*i]   !== {16'hF3F3, i[15:0], 16'hF4F4, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h2400 >> 3) + 2*i+1] !== {16'hF1F1, i[15:0], 16'hF2F2, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "and the data survived the recovery intact");

        $display("");
        if (errors == 0) $display("=== RW ENGINE PASSED ===");
        else             $display("=== %0d RW ENGINE ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #500_000;
        $display("TIMEOUT - a transfer never completed");
        $display("  rd_state=%0d rd_served=%b rd_req_valid=%b rl_n=%0d",
                 DUT.rd_state, DUT.rd_served, rd_req_valid, rl_n);
        $display("  wr_state=%0d wr_served=%b wr_req_valid=%b wr_sent=%0d",
                 DUT.wr_state, DUT.wr_served, wr_req_valid, DUT.wr_sent);
        $finish;
    end
endmodule
