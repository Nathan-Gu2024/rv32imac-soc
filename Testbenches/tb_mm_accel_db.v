`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"
`include "../src/accel_port_join.v"

// Double-buffering test: can the accelerator prefetch the NEXT B panel while
// the array is still computing from the current one, and is the overlap real?
//
// The mechanism is the B_PANEL_USE / B_PANEL_LOAD split plus CTRL bit4, the
// B-only load. A full load would rewrite aRowBuf, which the array reads during
// a run; B-only leaves A alone, so it is safe to issue mid-compute as long as
// the load panel differs from the use panel.
//
// What this checks, in the order things would silently pass otherwise:
//
//   1. NO CORRUPTION - the run overlapping the prefetch must still produce the
//      RIGHT answer for its own panel. A prefetch that scribbled on the panel
//      in use, or on aRowBuf, would show up here and nowhere else.
//   2. THE PREFETCH LANDED - the following run on the newly filled panel must
//      match ITS reference. Panels carry distinct operands so a prefetch that
//      silently did nothing cannot pass.
//   3. OVERLAP IS REAL - timed against doing the same work serially.
//
// Readback is excluded from the timing on purpose: it dominates at DIM=8 and
// would bury the effect being measured. This isolates load+compute.
`ifndef DIM
  `define DIM 8
`endif
`ifndef KLEN
  `define KLEN 16
`endif
`ifndef MEM_LATENCY
  `define MEM_LATENCY 3
`endif

module tb_mm_accel_db;
    localparam DIM  = `DIM;
    localparam KLEN = `KLEN;
    localparam ACCEL_BASE = 32'h0000_5000;

    localparam A_SRC  = 32'h0004_0000;
    localparam B0_SRC = 32'h0005_0000;
    localparam B1_SRC = 32'h0006_0000;
    // A lane is KLEN bytes = LPL lines, and "packed" means a stride of one
    // LANE, not a fixed 16. Both were hardcoded for KLEN=16, so at KLEN=64 the
    // bench staged a quarter of each panel and overlapped the rest.
    localparam STRIDE = KLEN;
    localparam LPL    = (KLEN + 15) / 16;

    reg clk = 1'b0, rst = 1'b1;
    always #5 clk = ~clk;

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    reg         d_req = 1'b0, d_we = 1'b0;
    reg  [31:0] d_addr = 32'b0, d_wdata = 32'b0;
    wire [31:0] d_rdata;
    wire        d_ready;

    wire [31:0] m_awaddr, m_wdata, m_araddr, m_rdata;
    wire [3:0]  m_wstrb;
    wire [1:0]  m_bresp, m_rresp;
    wire        m_awvalid, m_awready, m_wvalid, m_wready, m_bvalid, m_bready;
    wire        m_arvalid, m_arready, m_rvalid, m_rready;

    wire         mem_req_valid, mem_req_write;
    wire [7:0]   mem_req_lines;
    wire [31:0]  mem_req_addr;
    wire [127:0] mem_wline;
    reg          mem_ready = 1'b0;
    reg  [127:0] mem_rline = 128'b0;

    reg [127:0] LMEM [0:65535];
    integer lat, nread;

    // Burst-capable: one address phase costing MEM_LATENCY, then mem_req_lines
    // line pulses back to back. The read data is REGISTERED with the pulse it
    // belongs to - presenting it combinationally from the burst cursor returns
    // the NEXT line, because the cursor advances on the same edge.
    integer burst_left;
    reg [31:0] burst_addr;
    reg        in_burst, write_l;

    always @(posedge clk) begin
        if (rst) begin
            mem_ready <= 1'b0; lat <= 0; nread <= 0;
            in_burst <= 1'b0; burst_left <= 0;
        end else if (in_burst) begin
            if (burst_left > 0) begin
                mem_ready <= 1'b1;
                mem_rline <= LMEM[burst_addr >> 4];
                if (write_l) LMEM[burst_addr >> 4] <= mem_wline;
                else         nread <= nread + 1;
                burst_addr <= burst_addr + 32'd16;
                burst_left <= burst_left - 1;
                if (burst_left == 1) in_burst <= 1'b0;
            end else begin
                mem_ready <= 1'b0; in_burst <= 1'b0;
            end
        end else if (mem_req_valid && !mem_ready) begin
            if (lat >= `MEM_LATENCY) begin
                write_l    <= mem_req_write;
                burst_addr <= mem_req_addr;
                burst_left <= (mem_req_lines == 8'd0) ? 1 : mem_req_lines;
                in_burst   <= 1'b1;
                lat        <= 0;
                mem_ready  <= 1'b0;
            end else lat <= lat + 1;
        end else begin
            mem_ready <= 1'b0;
        end
    end

    axi_lite_bridge BRIDGE (
        .clk(clk), .rst(rst),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata),
        .d_rdata(d_rdata), .d_ready(d_ready),
        .m_axi_awaddr(m_awaddr), .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb), .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
        .m_axi_bresp(m_bresp), .m_axi_bvalid(m_bvalid), .m_axi_bready(m_bready),
        .m_axi_araddr(m_araddr), .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
        .m_axi_rdata(m_rdata), .m_axi_rresp(m_rresp), .m_axi_rvalid(m_rvalid), .m_axi_rready(m_rready)
    );

    // mm_accel's port is split; accel_port_join below re-serialises it onto the
    // single-port model this bench already had, which is therefore unchanged.
    wire        aj_rd_valid, aj_wr_valid, aj_wnext, aj_rd_ready, aj_wr_ready;
    wire [31:0] aj_rd_addr,  aj_wr_addr;
    wire [7:0]  aj_rd_lines, aj_wr_lines;
    wire [127:0] aj_wline, aj_rline;

    mm_accel ACCEL (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(m_awaddr), .s_axi_awvalid(m_awvalid), .s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata), .s_axi_wstrb(m_wstrb), .s_axi_wvalid(m_wvalid), .s_axi_wready(m_wready),
        .s_axi_bresp(m_bresp), .s_axi_bvalid(m_bvalid), .s_axi_bready(m_bready),
        .s_axi_araddr(m_araddr), .s_axi_arvalid(m_arvalid), .s_axi_arready(m_arready),
        .s_axi_rdata(m_rdata), .s_axi_rresp(m_rresp), .s_axi_rvalid(m_rvalid), .s_axi_rready(m_rready),
        .mem_rd_req_valid(aj_rd_valid), .mem_rd_req_addr(aj_rd_addr),
        .mem_rd_req_lines(aj_rd_lines), .mem_rd_ready(aj_rd_ready),
        .mem_rline(aj_rline),
        .mem_wr_req_valid(aj_wr_valid), .mem_wr_req_addr(aj_wr_addr),
        .mem_wr_req_lines(aj_wr_lines), .mem_wline(aj_wline),
        .mem_wnext(aj_wnext), .mem_wr_ready(aj_wr_ready)
    );

    task do_write(input [31:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            d_req <= 1'b1; d_we <= 1'b1; d_addr <= addr; d_wdata <= data;
            @(posedge clk);
            while (!d_ready) @(posedge clk);
            d_req <= 1'b0; d_we <= 1'b0;
            @(posedge clk);
        end
    endtask

    task do_read(input [31:0] addr, output [31:0] data);
        begin
            @(posedge clk);
            d_req <= 1'b1; d_we <= 1'b0; d_addr <= addr;
            @(posedge clk);
            while (!d_ready) @(posedge clk);
            data = d_rdata;
            d_req <= 1'b0;
            @(posedge clk);
        end
    endtask

    function [31:0] wr; input integer w; begin wr = ACCEL_BASE + (w * 4); end endfunction

    reg signed [7:0]  A [0:DIM-1][0:KLEN-1];
    reg signed [7:0]  B [0:1][0:DIM-1][0:KLEN-1];
    reg signed [31:0] CREF [0:1][0:DIM-1][0:DIM-1];

    integer i, j, k, c, p, errors, acc;
    integer t0, c_serial, c_overlap;
    reg [31:0] rd;
    reg [127:0] lw;

    task check(input integer p_, input [63:0] tag);
        begin
            do_write(wr(7), 0);
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    do_read(wr(8), rd);
                    if ($signed(rd) !== CREF[p_][i][j]) begin
                        if (errors < 6)
                            $display("  %0s MISMATCH panel %0d C[%0d][%0d]: got %0d expected %0d",
                                     tag, p_, i, j, $signed(rd), CREF[p_][i][j]);
                        errors = errors + 1;
                    end
                end
        end
    endtask

    initial begin
        errors = 0;

        for (i = 0; i < DIM; i = i + 1)
            for (k = 0; k < KLEN; k = k + 1)
                A[i][k] = $signed((i*5 + k*3) % 13) - 6;
        for (p = 0; p < 2; p = p + 1)
            for (j = 0; j < DIM; j = j + 1)
                for (k = 0; k < KLEN; k = k + 1)
                    B[p][j][k] = $signed((j*7 + k*2 + p*5) % 11) - 5;

        for (p = 0; p < 2; p = p + 1)
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    acc = 0;
                    for (k = 0; k < KLEN; k = k + 1) acc = acc + A[i][k]*B[p][j][k];
                    CREF[p][i][j] = acc;
                end

        for (i = 0; i < DIM; i = i + 1)
            for (c = 0; c < LPL; c = c + 1) begin
                for (k = 0; k < 16; k = k + 1)
                    lw[8*k +: 8] = ((c*16+k) < KLEN) ? A[i][c*16+k][7:0] : 8'h0;
                LMEM[(A_SRC + i*STRIDE + c*16) >> 4] = lw;
                for (k = 0; k < 16; k = k + 1)
                    lw[8*k +: 8] = ((c*16+k) < KLEN) ? B[0][i][c*16+k][7:0] : 8'h0;
                LMEM[(B0_SRC + i*STRIDE + c*16) >> 4] = lw;
                for (k = 0; k < 16; k = k + 1)
                    lw[8*k +: 8] = ((c*16+k) < KLEN) ? B[1][i][c*16+k][7:0] : 8'h0;
                LMEM[(B1_SRC + i*STRIDE + c*16) >> 4] = lw;
            end

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        $display("INFO: dim=%0d maxK=%0d bPanels=%0d | K=%0d lat=%0d",
                 rd[7:0], rd[15:8], rd[23:16], KLEN, `MEM_LATENCY);
        if (rd[23:16] < 2) begin
            $display("FAIL: need >=2 B panels, hardware has %0d", rd[23:16]);
            $finish;
        end

        do_write(wr(2), KLEN);
        do_write(wr(14), STRIDE);

        // ---- SERIAL: load B panel, then compute. Twice. ----
        do_write(wr(12), A_SRC);
        do_write(wr(13), B0_SRC);
        do_write(wr(16), 0);
        do_write(wr(0), 32'd8);                       // full load -> panel 0
        rd = 0; while (!rd[5]) do_read(wr(1), rd);

        // Result CHECKS are deliberately outside every timed region: they cost
        // ~65 MMIO transactions each and would bury the effect being measured.
        // Both schedules below are timed over load+compute only, and both do
        // the same checking work.
        t0 = cyc;
        do_write(wr(15), 0);
        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        c_serial = cyc - t0;
        check(0, "SERIAL-run1");

        t0 = cyc;
        do_write(wr(13), B1_SRC);
        do_write(wr(16), 1);
        do_write(wr(0), 32'd16);                      // B-only -> panel 1
        rd = 0; while (!rd[5]) do_read(wr(1), rd);
        do_write(wr(15), 1);
        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        c_serial = c_serial + (cyc - t0);
        check(1, "SERIAL-run2");

        // ---- OVERLAPPED: start compute, prefetch next panel DURING it ----
        do_write(wr(13), B0_SRC);
        do_write(wr(16), 0);
        do_write(wr(0), 32'd16);                      // refill panel 0
        rd = 0; while (!rd[5]) do_read(wr(1), rd);

        t0 = cyc;
        do_write(wr(15), 0);
        do_write(wr(0), 32'd1);                       // START compute on panel 0
        // Do NOT wait. Kick the prefetch of panel 1 while the array runs.
        do_write(wr(13), B1_SRC);
        do_write(wr(16), 1);
        do_write(wr(0), 32'd16);                      // B-only -> panel 1, concurrent
        rd = 0; while (!rd[1]) do_read(wr(1), rd);    // compute done
        rd = 0; while (!rd[5]) do_read(wr(1), rd);    // prefetch done
        c_overlap = cyc - t0;
        // The overlapped run must still be right for PANEL 0 - a prefetch that
        // scribbled on the panel in use, or on aRowBuf, shows up here.
        check(0, "OVERLAP-run1");

        t0 = cyc;
        do_write(wr(15), 1);
        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        c_overlap = c_overlap + (cyc - t0);
        check(1, "OVERLAP-run2");

        $display("");
        $display("  SERIAL     (load then compute) %0d cyc", c_serial);
        $display("  OVERLAPPED (prefetch during)   %0d cyc", c_overlap);
        if (c_overlap > 0)
            $display("  double-buffer speedup x100     %0d", (c_serial * 100) / c_overlap);

        $display("");
        if (errors == 0) $display("=== DIM=%0d: DOUBLE BUFFERING CORRECT ===", DIM);
        else             $display("=== DIM=%0d: %0d ERROR(S) ===", DIM, errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("TIMEOUT");
        $finish;
    end

    accel_port_join AJ (
        .clk(clk), .rst(rst),
        .accel_rd_req_valid(aj_rd_valid), .accel_rd_req_addr(aj_rd_addr),
        .accel_rd_req_lines(aj_rd_lines), .accel_rd_ready(aj_rd_ready),
        .accel_rline(aj_rline),
        .accel_wr_req_valid(aj_wr_valid), .accel_wr_req_addr(aj_wr_addr),
        .accel_wr_req_lines(aj_wr_lines), .accel_wline(aj_wline),
        .accel_wnext(aj_wnext), .accel_wr_ready(aj_wr_ready),
        .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr), .mem_req_lines(mem_req_lines),
        .mem_wline(mem_wline), .mem_ready(mem_ready),
        .mem_wnext(), .mem_rline(mem_rline)
    );

endmodule
