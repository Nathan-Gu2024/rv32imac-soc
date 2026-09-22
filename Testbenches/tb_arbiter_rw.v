`timescale 1ns/1ps
`include "../src/mem_arbiter_rw.v"
`include "../src/axi_rw_engine.v"

// mem_arbiter_rw + axi_rw_engine against a slave that serves both directions.
//
// The claim is that a requester's READ no longer waits for a DIFFERENT
// requester's WRITE. Under mem_arbiter that could not happen: one grant, one
// downstream channel, so a 63-line accelerator result burst held the port and
// instruction fetch stalled behind it.
//
// Test 3 issues an icache read while an accelerator write burst is in flight and
// checks two things - both transfers correct, AND the two sides genuinely
// overlapping. The overlap check is the one that matters: correct data alone
// would pass just as happily on a design that serialised them, which is the
// behaviour being replaced.
//
// ADDRESSES ARE ALL BELOW 0x4000 on purpose: the model indexes mem[addr>>3] into
// a 2048-entry array, so anything at or above 0x4000 indexes past the end and
// reads X. Three separate checks in this session were written against
// out-of-range addresses and blamed the RTL for the X they got back.
module tb_arbiter_rw;
    localparam AXI_W = 64;

    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    // requesters
    reg         ic_valid = 0;  reg [31:0] ic_addr = 0;
    wire        ic_ready;      wire [127:0] ic_rline;
    reg         dc_valid = 0, dc_write = 0;  reg [31:0] dc_addr = 0;
    reg  [127:0] dc_wline = 0;
    wire        dc_ready;      wire [127:0] dc_rline;
    reg         ac_rd_valid = 0;  reg [31:0] ac_rd_addr = 0;
    reg  [7:0]  ac_rd_lines = 0;
    reg         ac_wr_valid = 0;  reg [31:0] ac_wr_addr = 0;
    reg  [7:0]  ac_wr_lines = 0;  reg [127:0] ac_wline = 0;
    wire        ac_rd_ready, ac_wr_ready, ac_wnext;  wire [127:0] ac_rline;

    // arbiter <-> engine
    wire        rd_rv;  wire [31:0] rd_ra;  wire [7:0] rd_rl;
    wire        rd_rdy; wire [127:0] rd_rln;
    wire        wr_rv;  wire [31:0] wr_ra;  wire [7:0] wr_rl;
    wire [127:0] wr_wl; wire wr_nx, wr_rdy;

    mem_arbiter_rw ARB (
        .clk(clk), .rst(rst),
        .icache_req_valid(ic_valid), .icache_req_addr(ic_addr),
        .icache_ready(ic_ready), .icache_rline(ic_rline),
        .dcache_req_valid(dc_valid), .dcache_req_write(dc_write),
        .dcache_req_addr(dc_addr), .dcache_wline(dc_wline),
        .dcache_ready(dc_ready), .dcache_rline(dc_rline),
        .accel_rd_req_valid(ac_rd_valid), .accel_rd_req_addr(ac_rd_addr),
        .accel_rd_req_lines(ac_rd_lines),
        .accel_wr_req_valid(ac_wr_valid), .accel_wr_req_addr(ac_wr_addr),
        .accel_wr_req_lines(ac_wr_lines),
        .accel_wline(ac_wline),
        .accel_rd_ready(ac_rd_ready), .accel_wr_ready(ac_wr_ready),
        .accel_wnext(ac_wnext), .accel_rline(ac_rline),
        .rd_req_valid(rd_rv), .rd_req_addr(rd_ra), .rd_req_lines(rd_rl),
        .rd_ready(rd_rdy), .rd_rline(rd_rln),
        .wr_req_valid(wr_rv), .wr_req_addr(wr_ra), .wr_req_lines(wr_rl),
        .wr_wline(wr_wl), .wr_next(wr_nx), .wr_ready(wr_rdy)
    );

    // AXI
    wire [31:0] araddr, awaddr;  wire [7:0] arlen, awlen;
    wire [2:0] arsize, awsize;   wire [1:0] arburst, awburst;
    wire arvalid, awvalid, rready, wvalid, wlast, bready;
    wire [AXI_W-1:0] wdata;  wire [AXI_W/8-1:0] wstrb;
    reg arready = 0, awready = 0, wready = 0;
    reg [AXI_W-1:0] rdata = 0;
    reg rvalid = 0, rlast = 0, bvalid = 0;
    wire [15:0] rd_retries, wr_retries;

    axi_rw_engine #(.AXI_DATA_WIDTH(AXI_W)) ENG (
        .clk(clk), .rst(rst),
        .rd_req_valid(rd_rv), .rd_req_addr(rd_ra), .rd_req_lines(rd_rl),
        .rd_rline(rd_rln), .rd_ready(rd_rdy),
        .wr_req_valid(wr_rv), .wr_req_addr(wr_ra), .wr_req_lines(wr_rl),
        .wr_wline(wr_wl), .wr_next(wr_nx), .wr_ready(wr_rdy),
        .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
        .m_axi_arburst(arburst), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rvalid(rvalid), .m_axi_rlast(rlast),
        .m_axi_rready(rready),
        .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
        .m_axi_awburst(awburst), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(2'b00), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .rd_retries(rd_retries), .wr_retries(wr_retries)
    );

    // ---- slave, both directions independent ----
    reg [AXI_W-1:0] mem [0:2047];
    integer mi;
    reg rd_act = 0, wr_act = 0;
    reg [31:0] r_a, w_a;
    reg [8:0] r_l, w_l;
    reg [3:0] r_lat;

    always @(posedge clk) begin
        if (rst) begin
            arready <= 0; rvalid <= 0; rlast <= 0; rd_act <= 0;
            awready <= 0; wready <= 0; bvalid <= 0; wr_act <= 0;
        end else begin
            arready <= 1'b0;
            if (arvalid && !arready && !rd_act) begin
                arready <= 1; rd_act <= 1; r_a <= araddr;
                r_l <= arlen + 1; r_lat <= 3; rvalid <= 0;
            end else if (rd_act && !rvalid) begin
                if (r_lat == 0) begin
                    rvalid <= 1; rdata <= mem[r_a[13:3]]; rlast <= (r_l == 1);
                end else r_lat <= r_lat - 1;
            end else if (rd_act && rvalid && rready) begin
                r_a <= r_a + (AXI_W/8); r_l <= r_l - 1;
                if (r_l == 1) begin rvalid <= 0; rlast <= 0; rd_act <= 0; end
                else begin
                    rdata <= mem[((r_a + (AXI_W/8)) >> 3) & 11'h7FF];
                    rlast <= (r_l == 2);
                end
            end

            awready <= 1'b0;
            if (awvalid && !awready && !wr_act) begin
                awready <= 1; wr_act <= 1; w_a <= awaddr;
                w_l <= awlen + 1; wready <= 1;
            end
            if (wr_act && wvalid && wready) begin
                mem[w_a[13:3]] <= wdata;
                w_a <= w_a + (AXI_W/8); w_l <= w_l - 1;
                if (wlast || w_l == 1) begin
                    wready <= 0; wr_act <= 0; bvalid <= 1;
                end
            end
            if (bvalid && bready) bvalid <= 0;
        end
    end

    // both sides of the ARBITER busy at once
    integer overlap = 0;
    always @(posedge clk)
        if (!rst && ARB.r_state != 2'd0 && ARB.w_state != 2'd0)
            overlap = overlap + 1;

    // Count the accelerator's two completions separately, and catch any cycle
    // where one is asserted while the corresponding side is NOT the granted
    // requester - that would mean a completion leaked across directions.
    integer ac_rd_pulses = 0, ac_wr_pulses = 0;
    integer rd_wr_ready_collisions = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (ac_rd_ready) begin
                ac_rd_pulses = ac_rd_pulses + 1;
                if (ARB.r_state != 2'd3) rd_wr_ready_collisions =
                                         rd_wr_ready_collisions + 1;
            end
            if (ac_wr_ready) begin
                ac_wr_pulses = ac_wr_pulses + 1;
                if (ARB.w_state != 2'd2) rd_wr_ready_collisions =
                                         rd_wr_ready_collisions + 1;
            end
        end
    end

    integer errors = 0;
    task check(input cond, input [8*76-1:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    reg [127:0] ic_got;
    reg ic_done;
    always @(posedge clk) if (!rst && ic_ready) begin
        ic_got = ic_rline; ic_done = 1'b1;
    end

    integer i, bad;
    reg [15:0] ln;

    initial begin
        for (mi = 0; mi < 2048; mi = mi + 1) mem[mi] = {2{16'hC0DE, mi[15:0]}};
        ic_done = 0;
        repeat (4) @(posedge clk);
        rst = 0;
        @(negedge clk);

        // ---- Test 1: icache read alone ----
        $display("--- Test 1: icache read ---");
        ic_addr = 32'h0000_1000; ic_valid = 1'b1;
        wait (ic_done);
        @(negedge clk); ic_valid = 1'b0;
        check(ic_got[63:0]   === mem[32'h1000 >> 3] &&
              ic_got[127:64] === mem[(32'h1000 >> 3) + 1],
              "icache read returned its line");

        // ---- Test 2: accelerator 4-line write alone ----
        $display("--- Test 2: accelerator 4-line write ---");
        @(negedge clk);
        ln = 0;
        ac_wline = {16'hAAAA, ln, 16'hBBBB, ln, 16'hCCCC, ln, 16'hDDDD, ln};
        ac_wr_addr = 32'h0000_2000; ac_wr_lines = 8'd4;
        ac_wr_valid = 1'b1;
        while (!ac_wr_ready) begin
            @(posedge clk); #1;
            if (ac_wnext) begin
                ln = ln + 1;
                ac_wline = {16'hAAAA, ln, 16'hBBBB, ln, 16'hCCCC, ln, 16'hDDDD, ln};
            end
        end
        @(negedge clk); ac_wr_valid = 1'b0;
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h2000 >> 3) + 2*i]   !== {16'hCCCC, i[15:0], 16'hDDDD, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h2000 >> 3) + 2*i+1] !== {16'hAAAA, i[15:0], 16'hBBBB, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "accelerator write placed every line correctly");

        // ---- Test 3: icache READ during accelerator WRITE burst ----
        $display("--- Test 3: icache read DURING accelerator write burst ---");
        overlap = 0; ic_done = 0;
        @(negedge clk);
        ln = 0;
        ac_wline = {16'h1111, ln, 16'h2222, ln, 16'h3333, ln, 16'h4444, ln};
        ac_wr_addr = 32'h0000_2800; ac_wr_lines = 8'd4;
        ac_wr_valid = 1'b1;
        ic_addr = 32'h0000_1800; ic_valid = 1'b1;   // issued on the same cycle
        fork
            begin
                while (!ac_wr_ready) begin
                    @(posedge clk); #1;
                    if (ac_wnext) begin
                        ln = ln + 1;
                        ac_wline = {16'h1111, ln, 16'h2222, ln,
                                    16'h3333, ln, 16'h4444, ln};
                    end
                end
                @(negedge clk); ac_wr_valid = 1'b0;
            end
            begin
                wait (ic_done);
                @(negedge clk); ic_valid = 1'b0;
            end
        join
        repeat (10) @(posedge clk);

        check(ic_got[63:0]   === mem[32'h1800 >> 3] &&
              ic_got[127:64] === mem[(32'h1800 >> 3) + 1],
              "icache read was correct alongside the write");
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h2800 >> 3) + 2*i]   !== {16'h3333, i[15:0], 16'h4444, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h2800 >> 3) + 2*i+1] !== {16'h1111, i[15:0], 16'h2222, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "accelerator write was correct alongside the read");
        check(overlap > 0, "read and write sides were granted CONCURRENTLY");
        $display("    overlap cycles = %0d", overlap);

        // The accelerator's two completions must be DISTINCT signals. Merging
        // them (as the first version of this arbiter did) is harmless while the
        // accelerator has one request outstanding, and re-creates the direction
        // ambiguity the whole refactor removes as soon as it has two. A read
        // completion must never appear on the write completion, or vice versa.
        check(rd_wr_ready_collisions == 0,
              "accel completions never leaked across directions");

        // ---- Test 4: accelerator READ concurrent with accelerator WRITE ----
        // THE case stage 3 exists for. Tests 1-3 read via the icache, so they
        // never put two requests from the SAME requester in flight - which is why
        // splitting only the completions would have looked fine here and failed
        // in mm_accel.
        $display("--- Test 4: accel read CONCURRENT with accel write ---");
        overlap = 0;
        ac_rd_pulses = 0; ac_wr_pulses = 0; rd_wr_ready_collisions = 0;
        @(negedge clk);
        ln = 0;
        ac_wline = {16'h9999, ln, 16'hAAAA, ln, 16'hBBBB, ln, 16'hCCCC, ln};
        ac_wr_addr = 32'h0000_3000; ac_wr_lines = 8'd4; ac_wr_valid = 1'b1;
        ac_rd_addr = 32'h0000_1400; ac_rd_lines = 8'd4; ac_rd_valid = 1'b1;
        fork
            begin
                while (!ac_wr_ready) begin
                    @(posedge clk); #1;
                    if (ac_wnext) begin
                        ln = ln + 1;
                        ac_wline = {16'h9999, ln, 16'hAAAA, ln,
                                    16'hBBBB, ln, 16'hCCCC, ln};
                    end
                end
                @(negedge clk); ac_wr_valid = 1'b0;
            end
            begin
                // Hold the read request until all four lines have landed.
                while (ac_rd_pulses < 4) @(posedge clk);
                @(negedge clk); ac_rd_valid = 1'b0;
            end
        join
        repeat (10) @(posedge clk);

        check(ac_rd_pulses == 4, "accel read delivered exactly 4 line completions");
        check(ac_wr_pulses == 1, "accel write reported exactly 1 completion");
        check(rd_wr_ready_collisions == 0,
              "no completion leaked across directions under concurrency");
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (mem[(32'h3000 >> 3) + 2*i]   !== {16'hBBBB, i[15:0], 16'hCCCC, i[15:0]})
                bad = bad + 1;
            if (mem[(32'h3000 >> 3) + 2*i+1] !== {16'h9999, i[15:0], 16'hAAAA, i[15:0]})
                bad = bad + 1;
        end
        check(bad == 0, "accel write correct while its own read was in flight");
        check(overlap > 0, "accel read and accel write overlapped");
        $display("    overlap cycles = %0d", overlap);

        $display("");
        if (errors == 0) $display("=== ARBITER RW PASSED ===");
        else             $display("=== %0d ARBITER RW ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #500_000;
        $display("TIMEOUT - a transfer never completed");
        $finish;
    end
endmodule
