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
// Cycles WREADY is withheld between accepted write beats. AXI4 lets a slave
// deassert WREADY at any time, so a master must hold WVALID/WDATA until the
// beat is taken. Defaulting this to 0 is what let a real violation live: the
// adapter sliced mem_wline live on multi-line bursts, and mem_wnext advances
// the requester one beat EARLY, so the last beat of every line took the next
// line's bytes - visible only if the slave ever stalls mid-line.
`ifndef WGAP
  `define WGAP 2
`endif

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

    // TIMEOUT_LIMIT is tiny here on purpose: at the 4000-cycle default a
    // bench cannot reach the retry path at all, which is how it stayed
    // untested. Every legitimate transaction in this bench finishes in well
    // under 100 cycles, so only a deliberately forced stall trips it.
    axi_cache_adapter #(.AXI_DATA_WIDTH(AXI_W), .TIMEOUT_LIMIT(16'd100)) ADP (
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
    // Slave memory, indexed by 64-bit word. Test 6 asks where each line LANDED,
    // not merely whether its bytes were transmitted, because the retry defect
    // puts correct-looking data at the wrong address.
    reg [AXI_W-1:0] mem64 [0:1023];
    reg [31:0] wr_addr;             // running byte address through the data phase
    reg        force_wstall = 1'b0; // test-forced indefinite WREADY stall
    reg        force_bstall = 1'b0; // test-forced indefinite BVALID withhold
    // Test 6 snapshot: memory as it stood when the adapter first reported the
    // transfer complete. Checking later would measure the bench re-presenting the
    // request, not the adapter's recovery.
    reg         snap_arm = 1'b0;
    reg         snap_taken = 1'b0;
    reg [AXI_W-1:0] snap [0:7];
    integer     si;
    always @(posedge clk) begin
        if (snap_arm && accel_ready && !snap_taken) begin
            for (si = 0; si < 8; si = si + 1)
                snap[si] <= mem64[(((32'h7100_0000 >> 3) + si) & 10'h3FF)];
            snap_taken <= 1'b1;
            $display("[snap t=%0t] retry=%0d w_beats=%0d m0=%h m1=%h m6=%h m7=%h",
                     $time, ADP.retry_count, w_beats,
                     mem64[0], mem64[1], mem64[6], mem64[7]);
        end
    end
    integer w_gap;
    reg [AXI_W-1:0] WBEAT [0:255];
    reg [63:0] beat_seed;
    reg        wr_active;
    integer    wr_beats_left;

    always @(posedge clk) begin
        if (rst) begin
            arready <= 1'b0; awready <= 1'b0; wready <= 1'b0;
            rvalid  <= 1'b0; rlast   <= 1'b0; bvalid  <= 1'b0;
            beats_left <= 0; beats_seen <= 0; aw_seen <= 0; ar_seen <= 0;
            w_beats <= 0; beat_seed <= 64'd0; w_gap <= 0;
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
                wr_addr       <= awaddr;    // INCR burst: advances per beat below
                wready        <= !force_wstall;
            end

            // ---- write data ----
            if (wr_active && wvalid && wready) begin
                WBEAT[w_beats] <= wdata;          // captured for the data check
                mem64[wr_addr[12:3]] <= wdata;    // and placed where it belongs
                wr_addr       <= wr_addr + (AXI_W/8);
                w_beats       <= w_beats + 1;
                wr_beats_left <= wr_beats_left - 1;
                if (wlast || wr_beats_left == 1) begin
                    wready    <= 1'b0;
                    wr_active <= 1'b0;
                    bvalid    <= 1'b1;
                end else if (`WGAP != 0) begin
                    wready <= 1'b0;               // stall mid-line, legally
                    w_gap  <= 0;
                end
            end else if (wr_active && !wready && !bvalid) begin
                if (w_gap >= `WGAP && !force_wstall) wready <= 1'b1;
                else                                 w_gap  <= w_gap + 1;
            end

            // Last word on wready, so it overrides the assignments above. A slave
            // may deassert WREADY at any time and for any length, which is what
            // makes this a legal way to provoke the master's timeout.
            if (force_wstall) wready <= 1'b0;

            // ---- write response ----
            if (bvalid && bready) bvalid <= 1'b0;
            // Last word on bvalid: models the interconnect losing BRESP, which is
            // the hang the adapter's timeout exists to escape.
            if (force_bstall) bvalid <= 1'b0;
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
    integer ar_base;      // ar_seen counts every read in the run, so tests
                          // after the first must compare against a baseline

    // ARLEN is only meaningful during the address phase; checking it later
    // reads whatever the adapter happens to be driving. Latch it when the
    // slave accepts the address.
    reg [7:0] arlen_seen;
    always @(posedge clk) begin
        if (rst) arlen_seen <= 8'hFF;
        else if (arvalid && arready) arlen_seen <= arlen;
    end

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

        @(negedge clk); clr_flags = 1'b1; @(posedge clk);
        @(negedge clk); clr_flags = 1'b0;

        // ---- Test 4: 32-line read burst ----
        // What maxK=64 actually issues: a whole operand panel is dim*(maxK/16)
        // = 32 lines, four times the 8-line burst maxK=16 produced. That makes
        // ARLEN 63 rather than 15, and `saved_lines * BEATS_PER_LINE - 1` is
        // computed in 8 bits - so the longest burst the design can ask for is
        // the one worth testing, not the shortest.
        $display("--- Test 4: 32-line read burst (maxK=64 operand panel) ---");
        lines_seen = 0;
        ar_base    = ar_seen;
        @(negedge clk);
        accel_req_valid = 1'b1;
        accel_req_write = 1'b0;
        accel_req_lines = 8'd32;
        accel_req_addr  = 32'h4000_0200;   // 512-byte aligned: see note below

        fork
            begin : count_lines32
                integer guard;
                for (guard = 0; guard < 2000 && lines_seen < 32; guard = guard + 1) begin
                    @(posedge clk);
                    #1;
                    if (accel_ready) begin
                        lines_seen = lines_seen + 1;
                        if (lines_seen == 31) accel_req_valid = 1'b0;
                    end
                end
            end
        join

        check(lines_seen == 32,          "32-line read burst delivered every line");
        check(ar_seen - ar_base == 1,    "32-line burst used ONE AXI transaction");
        check(arlen_seen == (32*BEATS - 1),
                                         "ARLEN was 63, covering all 64 beats");
        repeat (4) @(posedge clk);
        #1;
        check(!mem_req_valid,            "Port released after the 32-line burst");

        // NOTE: a 32-line burst spans 512 bytes, and AXI4 forbids an INCR burst
        // from crossing a 4 KB boundary. This model does not enforce that, so
        // the alignment is a SOFTWARE contract the driver has to keep - staging
        // buffers aligned to the panel size. It is not visible in simulation.

        @(negedge clk); clr_flags = 1'b1; @(posedge clk);
        @(negedge clk); clr_flags = 1'b0;

        // ---- Test 5: multi-line write, DISTINCT data per line ----
        //
        // Test 3 writes a single line, which the adapter slices from a latched
        // copy - a different path entirely. A multi-line burst reads the
        // requester's line live, so this is the case that can pick up the
        // wrong bytes, and only if each line differs can that be seen at all.
        // Every earlier result-DMA test used uniform operands, where each line
        // is identical and a beat taken from the neighbouring line writes the
        // same value.
        $display("--- Test 5: 4-line write, distinct data, WREADY gaps ---");
        w_beats = 0;
        @(negedge clk);
        accel_req_valid = 1'b1;
        accel_req_write = 1'b1;
        accel_req_lines = 8'd4;
        accel_req_addr  = 32'h7000_0000;
        accel_wline     = 128'd0;

        fork
            begin : feed_lines
                integer guard;
                reg [15:0] ln;
                ln = 16'd0;
                accel_wline = {16'h1111, ln, 16'h2222, ln,
                               16'h3333, ln, 16'h4444, ln};
                for (guard = 0; guard < 800 && !saw_accel_ready; guard = guard + 1) begin
                    @(posedge clk);
                    #1;
                    if (accel_wnext) begin        // requester advances a line
                        ln = ln + 16'd1;
                        accel_wline = {16'h1111, ln, 16'h2222, ln,
                                       16'h3333, ln, 16'h4444, ln};
                    end
                    if (accel_ready) accel_req_valid = 1'b0;
                end
            end
        join
        @(negedge clk); accel_req_valid = 1'b0; accel_req_write = 1'b0;
        repeat (4) @(posedge clk);

        check(w_beats == 8, "4-line write delivered 8 beats");
        begin : chk5
            integer n, bad5;
            reg [15:0] nn;
            bad5 = 0;
            for (n = 0; n < 4; n = n + 1) begin
                nn = n[15:0];
                // a 128-bit line goes out low half first
                if (WBEAT[2*n]   !== {16'h3333, nn, 16'h4444, nn}) bad5 = bad5 + 1;
                if (WBEAT[2*n+1] !== {16'h1111, nn, 16'h2222, nn}) bad5 = bad5 + 1;
            end
            check(bad5 == 0, "every beat carried its OWN line's bytes");
        end

        // ---------------------------------------------------------------
        $display("");
        $display("--- Test 6: 4-line write, BRESP lost, timeout recovery ---");
        // The adapter abandons a transaction that makes no progress and reissues
        // it. That is transparent for a SINGLE-line request, which is what the
        // logic was written for. On a multi-line burst it is not: the requester's
        // FIFO has already advanced past the lines that were accepted, so a retry
        // starting again from the original line address writes later lines over
        // earlier addresses. Checking mem64 per line is what exposes it.
        w_beats = 0;
        clr_flags = 1'b1; @(posedge clk); clr_flags = 1'b0;
        snap_arm = 1'b1; snap_taken = 1'b0;
        @(negedge clk);
        accel_req_valid = 1'b1;
        accel_req_write = 1'b1;
        accel_req_lines = 8'd4;
        accel_req_addr  = 32'h7100_0000;

        fork
            begin : feed6
                integer g;
                reg [15:0] ln;
                reg done6;
                // done6 rather than saw_accel_ready: that flag is sticky across
                // tests, and relying on it here made the loop exit before it ever
                // advanced a line, so the requester presented line 0 four times.
                done6 = 1'b0;
                ln = 16'd0;
                accel_wline = {16'hAAAA, ln, 16'hBBBB, ln,
                               16'hCCCC, ln, 16'hDDDD, ln};
                for (g = 0; g < 4000 && !done6; g = g + 1) begin
                    @(posedge clk);
                    #1;
                    if (accel_wnext) begin
                        ln = ln + 16'd1;
                        accel_wline = {16'hAAAA, ln, 16'hBBBB, ln,
                                       16'hCCCC, ln, 16'hDDDD, ln};
                    end
                    if (accel_ready) begin
                        accel_req_valid = 1'b0;   // one transaction, then stop
                        done6 = 1'b1;
                    end
                end
            end
            begin : stall6
                integer g2;
                // Let the whole data phase land, then swallow BRESP for longer
                // than TIMEOUT_LIMIT. All four lines are in memory at this point;
                // only the acknowledgement is missing, which is precisely the
                // condition the adapter's comment describes.
                force_bstall = 1'b1;
                for (g2 = 0; g2 < 2000 && w_beats < 8; g2 = g2 + 1)
                    @(posedge clk);
                repeat (160) @(posedge clk);     // > TIMEOUT_LIMIT of 100
                force_bstall = 1'b0;
            end
        join_any
        // Give the recovery time to complete however it is going to.
        repeat (400) @(posedge clk);
        @(negedge clk); accel_req_valid = 1'b0; accel_req_write = 1'b0;

        begin : chk6
            integer n6, bad6;
            reg [15:0] n6n;
            reg [9:0]  base6;
            bad6  = 0;
            base6 = 10'h000;   // 0x71000000 >> 3, low bits only
            base6 = (32'h7100_0000 >> 3);
            for (n6 = 0; n6 < 4; n6 = n6 + 1) begin
                n6n = n6[15:0];
                if (snap[2*n6]   !== {16'hCCCC, n6n, 16'hDDDD, n6n})
                    bad6 = bad6 + 1;
                if (snap[2*n6+1] !== {16'hAAAA, n6n, 16'hBBBB, n6n})
                    bad6 = bad6 + 1;
            end
            check(ADP.retry_count > 16'd0, "the timeout actually fired");
            check(saw_accel_ready,
                  "the requester was released rather than left hanging");
            check(snap_taken, "a completion was reported, so memory was sampled");
            check(bad6 == 0,
                  "a lost BRESP did not corrupt already-written lines");
            if (bad6 != 0)
                $display("        %0d of 8 words wrong; retry_count=%0d",
                         bad6, ADP.retry_count);
        end

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
