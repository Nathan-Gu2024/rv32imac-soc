`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"

// Result-DMA test for the Chisel GEMM generator.
//
// The accelerator drains its accumulators over a 128-bit mem_arbiter-style
// line port instead of through the 32-bit AXI4-Lite register window. This
// bench stands a fake line memory in for the arbiter and checks three things
// the register-window path cannot break for us:
//
//   1. VALUES  - every accumulator lands in memory, correct and sign-extended
//   2. ORDER   - row-major, four accumulators per 128-bit line, low word first
//   3. ADDRESS - lines land at DEST_ADDR + 16*n, and DEST_ADDR is respected
//
// It also times the drain against the equivalent register-window readback, on
// the same data, because the whole justification for the DMA is a measured
// 70%-of-runtime readback cost - so the speedup is the number that decides
// whether the port earned its area.
//
// The fake memory deliberately inserts wait states (MEM_LATENCY) rather than
// accepting a line every cycle. A DMA that only works against zero-latency
// memory is not a DMA, and mem_arbiter shares this port with two caches.
`ifndef DIM
  `define DIM 4
`endif
`ifndef KLEN
  `define KLEN 8
`endif
`ifndef MEM_LATENCY
  `define MEM_LATENCY 3
`endif

module tb_mm_accel_dma;
    localparam DIM   = `DIM;
    localparam KLEN  = `KLEN;
    localparam NLINE = (DIM*DIM)/4;        // 128-bit lines per tile
    localparam DEST  = 32'h0002_1000;      // 16-byte aligned, deliberately not 0
    // Row pitch of a hypothetical larger C that this tile is one piece of.
    // A multiple of 16 and strictly larger than one tile row (DIM*4), so a
    // wrong stride cannot accidentally produce the contiguous layout and pass.
    localparam STRIDE = (DIM*4) + 32;

    localparam ACCEL_BASE = 32'h0000_5000;

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

    // ---- accelerator master port -> fake line memory ----
    wire         mem_req_valid, mem_req_write;
    wire [31:0]  mem_req_addr;
    wire [127:0] mem_wline;
    reg          mem_ready = 1'b0;

    // Sparse line memory, indexed by (addr-DEST)/16.
    reg [127:0] MEM  [0:1023];
    reg         WRIT [0:1023];
    integer     lat, nlines_seen;

    // Fake mem_arbiter: hold ready low for MEM_LATENCY cycles, then accept one
    // line. Mirrors a port that must wait its turn behind the caches.
    always @(posedge clk) begin
        if (rst) begin
            mem_ready <= 1'b0; lat <= 0; nlines_seen <= 0;
        end else if (mem_req_valid && !mem_ready) begin
            if (lat >= `MEM_LATENCY) begin
                mem_ready <= 1'b1;
                MEM [(mem_req_addr - DEST) >> 4] <= mem_wline;
                WRIT[(mem_req_addr - DEST) >> 4] <= 1'b1;
                nlines_seen <= nlines_seen + 1;
                lat <= 0;
            end else begin
                lat <= lat + 1;
            end
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

    mm_accel ACCEL (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(m_awaddr), .s_axi_awvalid(m_awvalid), .s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata), .s_axi_wstrb(m_wstrb), .s_axi_wvalid(m_wvalid), .s_axi_wready(m_wready),
        .s_axi_bresp(m_bresp), .s_axi_bvalid(m_bvalid), .s_axi_bready(m_bready),
        .s_axi_araddr(m_araddr), .s_axi_arvalid(m_arvalid), .s_axi_arready(m_arready),
        .s_axi_rdata(m_rdata), .s_axi_rresp(m_rresp), .s_axi_rvalid(m_rvalid), .s_axi_rready(m_rready),
        .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr), .mem_wline(mem_wline), .mem_ready(mem_ready)
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
    reg signed [7:0]  B [0:DIM-1][0:KLEN-1];   // B[j][k] is column j
    reg signed [31:0] CREF [0:DIM-1][0:DIM-1];

    integer i, j, k, g, errors, idx, acc, lineno;
    integer t_dma, t_mmio, t0;
    reg [31:0] rd, pw;
    reg signed [31:0] got;

    initial begin
        errors = 0;
        for (i = 0; i < 1024; i = i + 1) WRIT[i] = 1'b0;

        for (i = 0; i < DIM; i = i + 1)
            for (k = 0; k < KLEN; k = k + 1)
                A[i][k] = $signed((i*5 + k*3) % 13) - 6;
        for (j = 0; j < DIM; j = j + 1)
            for (k = 0; k < KLEN; k = k + 1)
                B[j][k] = $signed((j*7 + k*2) % 11) - 5;

        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                acc = 0;
                for (k = 0; k < KLEN; k = k + 1) acc = acc + A[i][k]*B[j][k];
                CREF[i][j] = acc;
            end

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        $display("INFO: dim=%0d maxK=%0d | K=%0d | %0d lines of 128b | mem latency %0d",
                 rd[7:0], rd[15:8], KLEN, NLINE, `MEM_LATENCY);
        if (rd[7:0] !== DIM) begin
            $display("FAIL: built for DIM=%0d, hardware says %0d", DIM, rd[7:0]);
            $finish;
        end

        do_write(wr(2), KLEN);

        // load operands (one LOAD_LANE write per panel, lanes auto-advance)
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

        do_write(wr(0), 32'd1);            // START
        rd = 0;
        while (!rd[1]) do_read(wr(1), rd); // wait DONE

        // ---- time the register-window readback on this same result ----
        t0 = cyc;
        do_write(wr(7), 0);
        for (idx = 0; idx < DIM*DIM; idx = idx + 1) do_read(wr(8), rd);
        t_mmio = cyc - t0;

        // ---- now drain the same accumulators over the DMA port ----
        do_write(wr(10), DEST);            // DEST_ADDR
        t0 = cyc;
        do_write(wr(0), 32'd4);            // CTRL bit2 = START_DMA
        rd = 0;
        while (!rd[3]) do_read(wr(1), rd); // wait DMA_DONE
        t_dma = cyc - t0;

        // ---- 1 & 2: values and row-major order ----
        for (idx = 0; idx < DIM*DIM; idx = idx + 1) begin
            i   = idx / DIM;
            j   = idx % DIM;
            got = MEM[idx/4][32*(idx%4) +: 32];
            if (got !== CREF[i][j]) begin
                if (errors < 6)
                    $display("  MISMATCH line %0d word %0d (C[%0d][%0d]): got %0d expected %0d",
                             idx/4, idx%4, i, j, got, CREF[i][j]);
                errors = errors + 1;
            end
        end

        // ---- 3: exactly NLINE lines written, all at DEST + 16n ----
        if (nlines_seen !== NLINE) begin
            $display("  WRONG LINE COUNT: saw %0d, expected %0d", nlines_seen, NLINE);
            errors = errors + 1;
        end
        for (idx = 0; idx < NLINE; idx = idx + 1)
            if (!WRIT[idx]) begin
                $display("  LINE %0d NEVER WRITTEN (addr %h)", idx, DEST + 16*idx);
                errors = errors + 1;
            end
        if (WRIT[NLINE]) begin
            $display("  WROTE PAST THE END: line %0d touched", NLINE);
            errors = errors + 1;
        end

        $display("");
        $display("  register-window readback : %0d cyc", t_mmio);
        $display("  result DMA               : %0d cyc", t_dma);
        if (t_dma > 0)
            $display("  speedup                  : %0d.%02dx",
                     t_mmio/t_dma, ((100*t_mmio)/t_dma)%100);

        // ---- 4: STRIDED writeback (OpenGeMM "programmable strided access") ----
        //
        // The real use: drop this tile into its place inside a bigger M x N C,
        // where each tile row is contiguous but consecutive rows are N*4 bytes
        // apart. Without this the DMA can only fill a scratch buffer that
        // software then copies - reintroducing exactly the CPU-mediated
        // movement the DMA exists to remove.
        //
        // dim=2 is excluded: one 128-bit line spans both rows there, so there
        // is no row boundary to stride at.
        if (DIM >= 4) begin
            for (idx = 0; idx < 1024; idx = idx + 1) WRIT[idx] = 1'b0;
            nlines_seen = 0;

            do_write(wr(10), DEST);          // DEST_ADDR
            do_write(wr(11), STRIDE);        // DEST_STRIDE = row pitch of big C
            do_write(wr(0), 32'd4);          // START_DMA
            rd = 0;
            while (!rd[3]) do_read(wr(1), rd);

            for (idx = 0; idx < DIM*DIM; idx = idx + 1) begin
                i = idx / DIM;
                j = idx % DIM;
                // element (i,j) must land at DEST + i*STRIDE + j*4
                lineno = (i*STRIDE + (j/4)*16) >> 4;
                got    = MEM[lineno][32*(j%4) +: 32];
                if (got !== CREF[i][j]) begin
                    if (errors < 6)
                        $display("  STRIDED MISMATCH C[%0d][%0d] @line %0d word %0d: got %0d expected %0d",
                                 i, j, lineno, j%4, got, CREF[i][j]);
                    errors = errors + 1;
                end
            end

            if (nlines_seen !== NLINE) begin
                $display("  STRIDED WRONG LINE COUNT: saw %0d expected %0d", nlines_seen, NLINE);
                errors = errors + 1;
            end
            $display("  strided writeback (stride=%0d bytes) checked", STRIDE);
        end

        $display("");
        if (errors == 0) $display("=== DIM=%0d K=%0d: DMA CORRECT (%0d lines) ===", DIM, KLEN, NLINE);
        else             $display("=== DIM=%0d K=%0d: %0d DMA ERROR(S) ===", DIM, KLEN, errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("TIMEOUT - DMA never asserted DMA_DONE");
        $finish;
    end
endmodule
