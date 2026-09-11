`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"

// Descriptor-queue test: can a batch of tiles run with no CPU round trip
// between them, and is the control saving real?
//
// Measured on hardware, each tile costs ~10 MMIO transactions of control -
// program the source and destination addresses, select panels, kick each of the
// three phases, poll each for completion. At 16.3 cycles per write and 8.7 per
// read that is ~160 cycles, which becomes roughly two thirds of a tile once
// burst DMA cuts the data movement.
//
// A descriptor carries everything one tile needs. Software pushes a batch and
// kicks once; the sequencer walks them itself.
//
// Two schedules on identical data, both verified against the same reference:
//
//   STEPPED  drive each phase from software, polling between - what we had
//   QUEUED   push N descriptors, kick once, poll once at the end
//
// Correctness matters as much as cycles: every tile is checked against ITS OWN
// operands, and each descriptor uses a distinct B panel, so a sequencer that
// dropped, repeated or reordered a descriptor produces wrong answers rather
// than merely different timing.
`ifndef DIM
  `define DIM 8
`endif
`ifndef KLEN
  `define KLEN 16
`endif
`ifndef NTILE
  `define NTILE 4
`endif
`ifndef MEM_LATENCY
  `define MEM_LATENCY 3
`endif

module tb_mm_accel_queue;
    localparam DIM   = `DIM;
    localparam KLEN  = `KLEN;
    localparam NTILE = `NTILE;
    localparam ACCEL_BASE = 32'h0000_5000;

    localparam A_SRC  = 32'h0004_0000;
    localparam B_SRC  = 32'h0005_0000;   // panel p at B_SRC + p*0x1000
    localparam DEST   = 32'h0008_0000;   // tile p at DEST  + p*0x1000
    // A lane is KLEN bytes = LPL lines, and "packed" means a stride of one
    // LANE, not a fixed 16. Both were hardcoded for KLEN=16, so at KLEN=64 the
    // bench staged a quarter of each panel and overlapped the rest.
    localparam STRIDE = KLEN;            // packed -> burst
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
    reg          mem_ready = 1'b0, mem_wnext = 1'b0;
    reg  [127:0] mem_rline = 128'b0;

    reg [127:0] LMEM [0:262143];
    integer lat, burst_left;
    integer nread = 0;              // lines actually fetched, to prove traffic fell
    reg [31:0] burst_addr;
    reg        in_burst, write_l, beat_phase;

    // Burst-capable line memory: one address phase, then a line every two
    // cycles (BEATS_PER_LINE = 2), wnext one line ahead on writes.
    always @(posedge clk) begin
        if (rst) begin
            mem_ready <= 1'b0; lat <= 0; in_burst <= 1'b0;
            burst_left <= 0; mem_wnext <= 1'b0; beat_phase <= 1'b0;
        end else if (in_burst) begin
            mem_wnext <= 1'b0;
            mem_ready <= 1'b0;
            if (burst_left > 0) begin
                if (write_l) begin
                    if (beat_phase == 1'b0) begin
                        LMEM[burst_addr >> 4] <= mem_wline;
                        if (burst_left > 1) mem_wnext <= 1'b1;
                        beat_phase <= 1'b1;
                    end else begin
                        beat_phase <= 1'b0;
                        burst_addr <= burst_addr + 32'd16;
                        burst_left <= burst_left - 1;
                        if (burst_left == 1) begin
                            in_burst  <= 1'b0;
                            mem_ready <= 1'b1;   // one completion per write
                        end
                    end
                end else begin
                    mem_ready  <= 1'b1;          // one per line on reads
                    nread      <= nread + 1;
                    mem_rline  <= LMEM[burst_addr >> 4];
                    burst_addr <= burst_addr + 32'd16;
                    burst_left <= burst_left - 1;
                    if (burst_left == 1) in_burst <= 1'b0;
                end
            end
        end else if (mem_req_valid && !mem_ready) begin
            if (lat >= `MEM_LATENCY) begin
                write_l    <= mem_req_write;
                burst_addr <= mem_req_addr;
                burst_left <= (mem_req_lines == 8'd0) ? 1 : mem_req_lines;
                in_burst   <= 1'b1;
                beat_phase <= 1'b0;
                lat        <= 0;
            end else lat <= lat + 1;
        end else begin
            mem_ready <= 1'b0;
            mem_wnext <= 1'b0;
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
        .mem_req_addr(mem_req_addr), .mem_wline(mem_wline),
        .mem_req_lines(mem_req_lines), .mem_wnext(mem_wnext),
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
    reg signed [7:0]  B [0:NTILE-1][0:DIM-1][0:KLEN-1];
    reg signed [31:0] CREF [0:NTILE-1][0:DIM-1][0:DIM-1];

    integer i, j, k, c, p, errors, acc;
    integer t0, c_stepped, c_queued, c_reuse;
    integer rd_queued, rd_reuse, exp_reuse;
    reg [31:0] rd;
    reg [127:0] lw;

    task place_panels;
        begin
            for (i = 0; i < DIM; i = i + 1)
                for (c = 0; c < LPL; c = c + 1) begin
                    for (k = 0; k < 16; k = k + 1)
                        lw[8*k +: 8] = ((c*16+k) < KLEN) ? A[i][c*16+k][7:0] : 8'h0;
                    LMEM[(A_SRC + i*STRIDE + c*16) >> 4] = lw;
                    for (p = 0; p < NTILE; p = p + 1) begin
                        for (k = 0; k < 16; k = k + 1)
                            lw[8*k +: 8] = ((c*16+k) < KLEN) ? B[p][i][c*16+k][7:0] : 8'h0;
                        LMEM[(B_SRC + p*32'h1000 + i*STRIDE + c*16) >> 4] = lw;
                    end
                end
        end
    endtask

    task check_tile(input integer p_, input [95:0] tag);
        begin : ct
            integer idx;
            reg signed [31:0] got;
            for (idx = 0; idx < DIM*DIM; idx = idx + 1) begin
                i = idx / DIM;
                j = idx % DIM;
                got = LMEM[(DEST + p_*32'h1000 + (idx/4)*16) >> 4][32*(idx%4) +: 32];
                if (got !== CREF[p_][i][j]) begin
                    if (errors < 6)
                        $display("  %0s MISMATCH tile %0d C[%0d][%0d]: got %0d expected %0d",
                                 tag, p_, i, j, got, CREF[p_][i][j]);
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
        for (p = 0; p < NTILE; p = p + 1)
            for (j = 0; j < DIM; j = j + 1)
                for (k = 0; k < KLEN; k = k + 1)
                    B[p][j][k] = $signed((j*7 + k*2 + p*5) % 11) - 5;

        for (p = 0; p < NTILE; p = p + 1)
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    acc = 0;
                    for (k = 0; k < KLEN; k = k + 1) acc = acc + A[i][k]*B[p][j][k];
                    CREF[p][i][j] = acc;
                end

        place_panels();

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        $display("INFO: dim=%0d maxK=%0d bPanels=%0d | %0d tiles, K=%0d",
                 rd[7:0], rd[15:8], rd[23:16], NTILE, KLEN);

        do_write(wr(14), STRIDE);

        // ---- STEPPED: software drives every phase ----
        t0 = cyc;
        for (p = 0; p < NTILE; p = p + 1) begin
            do_write(wr(2),  KLEN);
            do_write(wr(12), A_SRC);
            do_write(wr(13), B_SRC + p*32'h1000);
            do_write(wr(16), p % 4);
            do_write(wr(0),  32'd8);                     // START_LOAD
            rd = 0; while (!rd[5]) do_read(wr(1), rd);
            do_write(wr(15), p % 4);
            do_write(wr(0),  32'd1);                     // START compute
            rd = 0; while (!rd[1]) do_read(wr(1), rd);
            do_write(wr(10), DEST + p*32'h1000);
            do_write(wr(11), DIM*4);
            do_write(wr(0),  32'd4);                     // START_DMA
            rd = 0; while (!rd[3]) do_read(wr(1), rd);
        end
        c_stepped = cyc - t0;
        for (p = 0; p < NTILE; p = p + 1) check_tile(p, "STEPPED");

        // wipe the destinations so the queued run cannot pass on leftovers
        for (p = 0; p < NTILE; p = p + 1)
            for (i = 0; i < (DIM*DIM)/4; i = i + 1)
                LMEM[(DEST + p*32'h1000 + i*16) >> 4] = 128'h0;

        // ---- QUEUED: push descriptors, kick once, poll once ----
        do_write(wr(11), DIM*4);                          // DEST_STRIDE
        rd_queued = nread;
        t0 = cyc;
        for (p = 0; p < NTILE; p = p + 1) begin
            do_write(wr(17), A_SRC);                      // DESC_A_SRC
            do_write(wr(18), B_SRC + p*32'h1000);         // DESC_B_SRC
            do_write(wr(19), DEST + p*32'h1000);          // DESC_DEST
            do_write(wr(20), (( (p%4) & 4'hF) << 12) |
                             (( (p%4) & 4'hF) << 8)  | KLEN);   // DESC_PUSH
        end
        do_write(wr(0), 32'd32);                          // CTRL bit5: START_QUEUE
        rd = 0; while (!rd[7]) do_read(wr(1), rd);        // STATUS bit7: QUEUE_DONE
        c_queued = cyc - t0;
        for (p = 0; p < NTILE; p = p + 1) check_tile(p, "QUEUED");

        rd_queued = nread - rd_queued;

        // ---- QUEUED + A REUSE: same batch, A fetched once ----
        //
        // C[ti][tj] = A[ti]*B[tj], so a row of tiles shares one A panel. Every
        // descriptor here sets bOnly, and the A panel is made resident once by
        // an explicit full load beforehand.
        //
        // The A panel in MEMORY is then overwritten with a value that produces
        // a different answer. That is the real check: counting lines only shows
        // fewer fetches, but corrupting the source proves the array is using
        // the RESIDENT A rather than re-reading it. A hardware that ignored
        // bOnly would reload the garbage and every tile would be wrong.
        for (p = 0; p < NTILE; p = p + 1)
            for (i = 0; i < (DIM*DIM)/4; i = i + 1)
                LMEM[(DEST + p*32'h1000 + i*16) >> 4] = 128'h0;

        do_write(wr(2),  KLEN);
        do_write(wr(12), A_SRC);
        do_write(wr(13), B_SRC);
        do_write(wr(16), 0);
        do_write(wr(0),  32'd8);                          // full load: A resident
        rd = 0; while (!rd[5]) do_read(wr(1), rd);

        for (i = 0; i < DIM; i = i + 1)                   // poison the A source
            for (c = 0; c < LPL; c = c + 1)
                LMEM[(A_SRC + i*STRIDE + c*16) >> 4] = {16{8'h5A}};

        rd_reuse = nread;
        do_write(wr(11), DIM*4);
        t0 = cyc;
        for (p = 0; p < NTILE; p = p + 1) begin
            do_write(wr(17), A_SRC);
            do_write(wr(18), B_SRC + p*32'h1000);
            do_write(wr(19), DEST + p*32'h1000);
            do_write(wr(20), 32'h1_0000 |                 // ctl[16] = bOnly
                             (((p%4) & 4'hF) << 12) |
                             (((p%4) & 4'hF) << 8)  | KLEN);
        end
        do_write(wr(0), 32'd32);
        rd = 0; while (!rd[7]) do_read(wr(1), rd);
        c_reuse  = cyc - t0;
        rd_reuse = nread - rd_reuse;
        for (p = 0; p < NTILE; p = p + 1) check_tile(p, "REUSE");

        // B only: one panel per tile instead of two.
        exp_reuse = NTILE * DIM * LPL;
        if (rd_reuse != exp_reuse) begin
            $display("  FAIL: A-reuse fetched %0d lines, expected %0d",
                     rd_reuse, exp_reuse);
            errors = errors + 1;
        end

        $display("");
        $display("  STEPPED (software drives each phase) %0d cyc", c_stepped);
        $display("  QUEUED  (push batch, kick once)      %0d cyc", c_queued);
        if (c_queued > 0)
            $display("  control speedup x100                 %0d",
                     (c_stepped * 100) / c_queued);
        $display("  QUEUED + A REUSE (B panel only)      %0d cyc", c_reuse);
        if (c_reuse > 0)
            $display("  A-reuse speedup x100                 %0d",
                     (c_queued * 100) / c_reuse);
        $display("  per tile: stepped %0d, queued %0d, reuse %0d",
                 c_stepped / NTILE, c_queued / NTILE, c_reuse / NTILE);
        $display("  operand lines per batch: queued %0d, reuse %0d (%0d tiles)",
                 rd_queued, rd_reuse, NTILE);

        $display("");
        if (errors == 0) $display("=== DIM=%0d: QUEUE CORRECT (%0d tiles) ===", DIM, NTILE);
        else             $display("=== DIM=%0d: %0d QUEUE ERROR(S) ===", DIM, errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("TIMEOUT - queue never reported done");
        $finish;
    end
endmodule
