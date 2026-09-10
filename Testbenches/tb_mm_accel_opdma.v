`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"

// Operand-DMA test: does the accelerator fetching its own operands over the
// 128-bit line port actually work, and what is it worth against pushing them
// through the 32-bit register window?
//
// This is the measurement that decides the operand path. Everything projected
// about it so far - load 874 -> ~145 cycles, ~27x over scalar - came from
// dividing measured per-transaction costs, never from running it.
//
// Checked here, in order of what would silently pass otherwise:
//
//   1. CORRECTNESS - results match a reference computed from the SAME bytes
//      that were placed in memory, so a mis-addressed fetch cannot pass
//   2. STRIDE      - SRC_STRIDE is set LARGER than a lane (32 vs 16 bytes) so
//      panels are spread out; a fetch that ignored stride would read the wrong
//      lanes and produce wrong answers rather than merely slower ones
//   3. DIRECTION   - the mock only writes memory on write requests, so a read
//      that asserted mem_req_write would corrupt its own operands
//   4. COST        - timed against the MMIO push path on identical data
//
// The mock inserts wait states: this port shares mem_arbiter with two caches,
// and a zero-latency responder would flatter the DMA.
`ifndef DIM
  `define DIM 8
`endif
`ifndef KLEN
  `define KLEN 16
`endif
`ifndef MEM_LATENCY
  `define MEM_LATENCY 3
`endif
`ifndef STRIDE
  `define STRIDE 32
`endif

module tb_mm_accel_opdma;
    localparam DIM  = `DIM;
    localparam KLEN = `KLEN;
    localparam ACCEL_BASE = 32'h0000_5000;

    // Panels live at distinct, non-zero bases; stride is DELIBERATELY wider
    // than one lane so a stride-ignoring fetch reads the wrong bytes.
    localparam A_SRC  = 32'h0004_0000;
    localparam B_SRC  = 32'h0005_0000;
    // 16 = lanes packed back-to-back, which the accelerator can cover with a
    // single INCR burst. Anything larger leaves gaps between lanes and forces
    // the per-line fallback. Both paths must produce identical results.
    localparam STRIDE = `STRIDE;

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

    // ---- line-memory mock (reads AND writes) ----
    wire         mem_req_valid, mem_req_write;
    wire [7:0]   mem_req_lines;
    wire [31:0]  mem_req_addr;
    wire [127:0] mem_wline;
    reg          mem_ready = 1'b0;
    reg  [127:0] mem_rline = 128'b0;

    reg [127:0] LMEM [0:65535];        // indexed by addr>>4
    integer lat, nread, nwrite;
    reg [31:0] addr_l;
    reg        write_l;

    // Burst-capable mock: ONE address phase costing MEM_LATENCY, then
    // mem_req_lines line pulses back to back. That is the point of a burst -
    // the round trip is paid once, not per line - so a mock that charged
    // latency per line would hide the entire benefit being measured.
    //
    // Writes still modify memory only on a write request: an operand fetch
    // that asserted mem_req_write would overwrite the lines it is reading.
    integer burst_left;
    reg [31:0] burst_addr;
    reg        in_burst;

    always @(posedge clk) begin
        if (rst) begin
            mem_ready <= 1'b0; lat <= 0; nread <= 0; nwrite <= 0;
            in_burst <= 1'b0; burst_left <= 0;
        end else if (in_burst) begin
            if (burst_left > 0) begin
                mem_ready  <= 1'b1;
                addr_l     <= burst_addr;
                // Register the line alongside the ready pulse it belongs to.
                // Presenting it combinationally from burst_addr returned data
                // one line AHEAD, because burst_addr advances on this same
                // edge - the accelerator then wrote every lane with its
                // successor's bytes and the whole tile came out wrong.
                mem_rline  <= LMEM[burst_addr >> 4];
                if (write_l) begin
                    LMEM[burst_addr >> 4] <= mem_wline;
                    nwrite <= nwrite + 1;
                end else nread <= nread + 1;
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

    // mem_rline is driven in the sequential block above, registered with the
    // ready pulse it accompanies.

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

    mm_accel ACCEL (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(m_awaddr), .s_axi_awvalid(m_awvalid), .s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata), .s_axi_wstrb(m_wstrb), .s_axi_wvalid(m_wvalid), .s_axi_wready(m_wready),
        .s_axi_bresp(m_bresp), .s_axi_bvalid(m_bvalid), .s_axi_bready(m_bready),
        .s_axi_araddr(m_araddr), .s_axi_arvalid(m_arvalid), .s_axi_arready(m_arready),
        .s_axi_rdata(m_rdata), .s_axi_rresp(m_rresp), .s_axi_rvalid(m_rvalid), .s_axi_rready(m_rready),
        .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr), .mem_wline(mem_wline),
        .mem_req_lines(mem_req_lines),
        .mem_rline(mem_rline), .mem_ready(mem_ready)
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
    reg signed [7:0]  B [0:DIM-1][0:KLEN-1];
    reg signed [31:0] CREF [0:DIM-1][0:DIM-1];

    integer i, j, k, g, errors, acc, maxk, exp_reads;
    integer t0, c_push, c_dma;
    reg [31:0] rd, pw;
    reg [127:0] lineword;

    task check_results(input [63:0] tag);
        begin
            do_write(wr(7), 0);
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    do_read(wr(8), rd);
                    if ($signed(rd) !== CREF[i][j]) begin
                        if (errors < 6)
                            $display("  %0s MISMATCH C[%0d][%0d]: got %0d expected %0d",
                                     tag, i, j, $signed(rd), CREF[i][j]);
                        errors = errors + 1;
                    end
                end
        end
    endtask

    initial begin
        errors = 0;

        for (i = 0; i < DIM; i = i + 1)
            for (k = 0; k < KLEN; k = k + 1) begin
                A[i][k] = $signed((i*5 + k*3) % 13) - 6;
                B[i][k] = $signed((i*7 + k*2) % 11) - 5;
            end

        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                acc = 0;
                for (k = 0; k < KLEN; k = k + 1) acc = acc + A[i][k]*B[j][k];
                CREF[i][j] = acc;
            end

        // Place the panels in line memory at their strided lane addresses. This
        // is the same data the push path sends, so the two paths are compared
        // on identical operands.
        for (i = 0; i < DIM; i = i + 1) begin
            for (k = 0; k < 16; k = k + 1)
                lineword[8*k +: 8] = (k < KLEN) ? A[i][k][7:0] : 8'h00;
            LMEM[(A_SRC + i*STRIDE) >> 4] = lineword;
            for (k = 0; k < 16; k = k + 1)
                lineword[8*k +: 8] = (k < KLEN) ? B[i][k][7:0] : 8'h00;
            LMEM[(B_SRC + i*STRIDE) >> 4] = lineword;
        end

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        maxk = rd[15:8];
        $display("INFO: dim=%0d maxK=%0d bPanels=%0d | K=%0d stride=%0d lat=%0d",
                 rd[7:0], rd[15:8], rd[23:16], KLEN, STRIDE, `MEM_LATENCY);

        do_write(wr(2), KLEN);

        // ---- baseline: push operands through the register window ----
        t0 = cyc;
        do_write(wr(4), 0);
        for (i = 0; i < DIM; i = i + 1)
            for (g = 0; g < KLEN/4; g = g + 1) begin
                pw = { A[i][4*g+3][7:0], A[i][4*g+2][7:0], A[i][4*g+1][7:0], A[i][4*g+0][7:0] };
                do_write(wr(5), pw);
            end
        do_write(wr(4), 0);
        for (j = 0; j < DIM; j = j + 1)
            for (g = 0; g < KLEN/4; g = g + 1) begin
                pw = { B[j][4*g+3][7:0], B[j][4*g+2][7:0], B[j][4*g+1][7:0], B[j][4*g+0][7:0] };
                do_write(wr(6), pw);
            end
        c_push = cyc - t0;

        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        check_results("PUSH");

        // ---- clear the buffers so the DMA cannot pass on leftovers ----
        // Without this, a DMA that fetched nothing at all would still produce
        // correct results from the push path's data.
        do_write(wr(4), 0);
        for (i = 0; i < DIM; i = i + 1)
            for (g = 0; g < KLEN/4; g = g + 1) do_write(wr(5), 32'h0);
        do_write(wr(4), 0);
        for (j = 0; j < DIM; j = j + 1)
            for (g = 0; g < KLEN/4; g = g + 1) do_write(wr(6), 32'h0);

        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        do_write(wr(7), 0);
        do_read(wr(8), rd);
        if ($signed(rd) !== 0) begin
            $display("  SETUP FAIL: buffers not cleared, C00=%0d", $signed(rd));
            errors = errors + 1;
        end

        // ---- operand DMA ----
        do_write(wr(12), A_SRC);
        do_write(wr(13), B_SRC);
        do_write(wr(14), STRIDE);
        t0 = cyc;
        do_write(wr(0), 32'd8);                 // CTRL bit3 = START_LOAD
        rd = 0;
        while (!rd[5]) do_read(wr(1), rd);      // STATUS bit5 = LOAD_DONE
        c_dma = cyc - t0;

        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        check_results("DMA");

        $display("");
        $display("  operand push (MMIO)  %0d cyc", c_push);
        $display("  operand DMA  (line)  %0d cyc", c_dma);
        if (c_dma > 0)
            $display("  speedup x100         %0d", (c_push * 100) / c_dma);
        $display("  line reads %0d, line writes %0d, stride %0d (%0s)",
                 nread, nwrite, STRIDE,
                 (STRIDE == 16) ? "burst" : "per-line");
        if (nwrite != 0) begin
            $display("  FAIL: operand fetch issued WRITE requests");
            errors = errors + 1;
        end
        // Lines per lane is maxK/16, not 1: at maxK=32 a lane spans two lines
        // and the fetch issues twice as many reads. Hardcoding 2*DIM here
        // reported a hardware failure when the hardware was correct.
        exp_reads = 2 * DIM * ((maxk > 16) ? (maxk/16) : 1);
        if (nread != exp_reads) begin
            $display("  FAIL: expected %0d line reads, saw %0d", exp_reads, nread);
            errors = errors + 1;
        end

        $display("");
        if (errors == 0) $display("=== DIM=%0d: OPERAND DMA CORRECT ===", DIM);
        else             $display("=== DIM=%0d: %0d ERROR(S) ===", DIM, errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("TIMEOUT - LOAD_DONE never asserted");
        $finish;
    end
endmodule
