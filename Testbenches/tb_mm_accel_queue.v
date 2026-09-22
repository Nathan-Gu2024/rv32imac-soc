`timescale 1ns/1ps
// REGRESSION_VARIANT: RW_DIRECT
//
// -DRW_DIRECT replaces the join and its single-port model with two
// independent channels over one shared memory, so a read and a write can
// actually be served together. The default build keeps the join, because
// that is what fpga_top builds at MEM_PATH_RW=0 - so the default bench
// matches the default hardware, and the variant measures the overlap the
// split port exists for.
`include "../src/axi_lite_bridge.v"
`include "../src/accel_port_join.v"

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

    wire irq_batch, irq_dma;

    // mm_accel's port is split; accel_port_join below re-serialises it onto the
    // single-port model this bench already had, which is therefore unchanged.
`ifdef RW_DIRECT
    reg         aj_rd_ready_r = 1'b0, aj_wr_ready_r = 1'b0, aj_wnext_r = 1'b0;
    reg  [127:0] aj_rline_r = 128'b0;
    wire        aj_rd_ready = aj_rd_ready_r;
    wire        aj_wr_ready = aj_wr_ready_r;
    wire        aj_wnext    = aj_wnext_r;
    wire [127:0] aj_rline   = aj_rline_r;
    wire        aj_rd_valid, aj_wr_valid;
`else
    wire        aj_rd_valid, aj_wr_valid, aj_wnext, aj_rd_ready, aj_wr_ready;
`endif
    wire [31:0] aj_rd_addr,  aj_wr_addr;
    wire [7:0]  aj_rd_lines, aj_wr_lines;
`ifdef RW_DIRECT
    wire [127:0] aj_wline;
`else
    wire [127:0] aj_wline, aj_rline;
`endif

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
        .mem_wnext(aj_wnext), .mem_wr_ready(aj_wr_ready),
        .irq_batch(irq_batch), .irq_dma(irq_dma)
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

    // Completion interrupt lines. These are sticky LEVELS off qDone/dmaDone,
    // and src/intc.v captures the 0->1 edge, so what has to hold is that a
    // whole batch yields exactly ONE edge - not one per tile. done and ldDone
    // both toggle per tile while a batch runs, so the difference is the entire
    // reason only these two are exported.
    // Declared up at the DUT instantiation instead of here: connecting a net in
    // a port map before declaring it makes Icarus create an implicit wire, which
    // it then resolves to the right width - but strict Verilog-2005 says an
    // implicit net is 1-bit scalar. These two happen to BE 1-bit, so nothing
    // broke; the habit is what is worth not having.
    reg  irq_batch_prev = 1'b0, irq_dma_prev = 1'b0;
    integer irq_batch_edges = 0, irq_dma_edges = 0;
    reg count_irq = 1'b0;
    always @(posedge clk) begin
        irq_batch_prev <= irq_batch;
        irq_dma_prev   <= irq_dma;
        if (count_irq) begin
            if (irq_batch && !irq_batch_prev) irq_batch_edges = irq_batch_edges + 1;
            if (irq_dma   && !irq_dma_prev)   irq_dma_edges   = irq_dma_edges   + 1;
        end
    end

    // ---- How much is there to gain from overlapping load with compute? ----
    //
    // Measurement, not a change. The claim is that the queue sequencer
    // serialises load -> compute -> store per tile, so the memory port sits idle
    // for the whole compute window, and a shadow descriptor context would fill
    // it. That predicts ~22-28%.
    //
    // It is measured rather than trusted because two estimates in this audit
    // derived the same way missed by 20x and 100x. What decides the value of a
    // shadow context is specifically ARRAY BUSY WHILE THE PORT IS IDLE - a cycle
    // where the port is busy cannot be improved by more overlap, and a cycle
    // where the array is idle is not part of the compute window.
    //
    // count_irq already brackets exactly the queued batch, so it is reused here.
    integer window_cyc = 0, port_busy_cyc = 0, array_busy_cyc = 0;
    integer overlap_opp_cyc = 0;

    // ---- T3.2 step 6 instrumentation ----
    //
    // Written BEFORE the serialisation terms come out, so the control reading is
    // taken against RTL that provably cannot overlap. Without that, a later
    // nonzero reading proves nothing: the first attempt at operand prefetch
    // reported a 25% speedup that was really a tile of zeros, because the
    // sequencer had stopped waiting for the data rather than got faster.
    //
    // rd_wr_req_overlap is THE CLAIM: both channels presenting a request on the
    // same cycle. It is provably 0 before the guards come out, because
    // mem_rd_req_valid is ldDrive = ldBusy && ldReq && !dmaBusy while
    // mem_wr_req_valid is dmaDrive = dmaBusy && fifo.valid - one requires
    // !dmaBusy and the other requires dmaBusy, so they are mutually exclusive by
    // construction.
    //
    // ld_dma_overlap - both engines merely BUSY - is kept as a secondary stat and
    // is NOT the claim. The first version of this monitor used it and read 84
    // cycles in the control, because a load can be started mid-store and then sit
    // with its request masked by ldDrive. Busy is not progress.
    integer rd_wr_req_overlap = 0;
    integer ld_dma_overlap = 0;
    // Exact line accounting. Not >=, because the failure this guards against is
    // a completion landing on the WRONG channel, which shows up as one counter
    // gaining what the other lost.
    integer rd_line_pulses = 0, wr_txn_pulses = 0;
    // The direct H1 monitor: dmaSent must advance ONLY on a write completion.
    // A read pulse crediting the store is the exact double fault the split
    // removed, and it is invisible in the result data whenever the store happens
    // to finish anyway.
    integer dma_sent_bad = 0;
    reg [8:0] dma_sent_prev = 9'd0;
    // A delivered line must always have an owner.
    integer rd_orphan = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (ACCEL.ldBusy && ACCEL.dmaBusy) ld_dma_overlap = ld_dma_overlap + 1;
            if (ACCEL.mem_rd_req_valid && ACCEL.mem_wr_req_valid)
                rd_wr_req_overlap = rd_wr_req_overlap + 1;
            if (ACCEL.mem_rd_ready) begin
                rd_line_pulses = rd_line_pulses + 1;
                if (!ACCEL.ldBusy) rd_orphan = rd_orphan + 1;
            end
            if (ACCEL.mem_wr_ready) wr_txn_pulses = wr_txn_pulses + 1;
            if (ACCEL.dmaSent !== dma_sent_prev && !ACCEL.mem_wr_ready
                && ACCEL.dmaBusy)
                dma_sent_bad = dma_sent_bad + 1;
            dma_sent_prev <= ACCEL.dmaSent;
        end
    end
    always @(posedge clk) begin
        if (count_irq) begin
            window_cyc = window_cyc + 1;
            if (mem_req_valid)                  port_busy_cyc   = port_busy_cyc + 1;
            if (ACCEL.busy)                     array_busy_cyc  = array_busy_cyc + 1;
            if (ACCEL.busy && !mem_req_valid)   overlap_opp_cyc = overlap_opp_cyc + 1;
        end
    end

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
        irq_batch_edges = 0; irq_dma_edges = 0; count_irq = 1'b1;
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
        count_irq = 1'b0;
        if (irq_batch_edges != 1) begin
            $display("  FAIL: irq_batch fired %0d times for one %0d-tile batch, want 1",
                     irq_batch_edges, NTILE);
            errors = errors + 1;
        end
        // dmaDone is cleared and re-set by the sequencer once per tile, so it
        // is NOT batch-granular - which is why software leaves source 3 masked
        // in queue mode. Assert the per-tile behaviour so the distinction is
        // pinned down rather than assumed.
        if (irq_dma_edges != NTILE) begin
            $display("  FAIL: irq_dma fired %0d times over %0d tiles, want %0d",
                     irq_dma_edges, NTILE, NTILE);
            errors = errors + 1;
        end
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
        $display("    window %0d cyc | port busy %0d (%0d%%) | array busy %0d (%0d%%)",
                 window_cyc, port_busy_cyc, (port_busy_cyc * 100) / window_cyc,
                 array_busy_cyc, (array_busy_cyc * 100) / window_cyc);
        $display("    OVERLAP OPPORTUNITY (array busy, port idle) %0d cyc = %0d%% of batch",
                 overlap_opp_cyc, (overlap_opp_cyc * 100) / window_cyc);
        $display("    RD/WR REQUEST OVERLAP (the claim)           %0d cyc",
                 rd_wr_req_overlap);
        $display("    both engines busy (secondary, not the claim) %0d cyc",
                 ld_dma_overlap);
        $display("    line accounting: rd completions %0d, wr completions %0d",
                 rd_line_pulses, wr_txn_pulses);
        if (dma_sent_bad != 0) begin
            $display("    ERROR: dmaSent advanced %0d times without a write completion",
                     dma_sent_bad);
            errors = errors + 1;
        end
        if (rd_orphan != 0) begin
            $display("    ERROR: %0d read line(s) delivered with no load in flight",
                     rd_orphan);
            errors = errors + 1;
        end
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
        $display("  interrupts per %0d-tile batch: irq_batch %0d, irq_dma %0d",
                 NTILE, irq_batch_edges, irq_dma_edges);

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

`ifdef RW_DIRECT
    // ---- DUAL-CHANNEL model: read and write served independently ----
    //
    // No join. Each channel gets its own copy of the same line model, sharing
    // LMEM, so a read and a write really can be in flight together. This is what
    // the accelerator's split port is for; the single-port default cannot show it
    // however correct the design is.
    reg         d_rd_busy = 0, d_wr_busy = 0, d_wr_phase = 0;
    reg [31:0]  d_rd_addr, d_wr_addr;
    reg [8:0]   d_rd_left, d_wr_left;
    reg [3:0]   d_rd_lat,  d_wr_lat;

    always @(posedge clk) begin
        if (rst) begin
            d_rd_busy <= 0; d_wr_busy <= 0; aj_rd_ready_r <= 0;
            aj_wr_ready_r <= 0; aj_wnext_r <= 0; d_wr_phase <= 0;
        end else begin
            aj_rd_ready_r <= 1'b0;
            aj_wr_ready_r <= 1'b0;
            aj_wnext_r    <= 1'b0;

            // ---- read channel ----
            // !aj_rd_ready_r: never accept on a completion cycle. The requester
            // still holds valid while it observes ready, so without this the
            // model restarts the transaction it just finished. This is the
            // FOURTH model in this repo to need the guard - it belongs in every
            // one of them by default, not as an afterthought.
            if (!d_rd_busy && aj_rd_valid && !aj_rd_ready_r) begin
                d_rd_busy <= 1'b1;
                d_rd_addr <= aj_rd_addr;
                d_rd_left <= (aj_rd_lines == 8'd0) ? 9'd1 : {1'b0, aj_rd_lines};
                d_rd_lat  <= `MEM_LATENCY;
            end else if (d_rd_busy) begin
                if (d_rd_lat != 0) d_rd_lat <= d_rd_lat - 1;
                else begin
                    aj_rline_r    <= LMEM[d_rd_addr >> 4];
                    aj_rd_ready_r <= 1'b1;
                    nread         <= nread + 1;
                    d_rd_addr     <= d_rd_addr + 32'd16;
                    d_rd_left     <= d_rd_left - 1;
                    if (d_rd_left == 1) d_rd_busy <= 1'b0;
                end
            end

            // ---- write channel, two phases per line as the adapter takes ----
            if (!d_wr_busy && aj_wr_valid && !aj_wr_ready_r) begin
                d_wr_busy  <= 1'b1;
                d_wr_addr  <= aj_wr_addr;
                d_wr_left  <= (aj_wr_lines == 8'd0) ? 9'd1 : {1'b0, aj_wr_lines};
                d_wr_lat   <= `MEM_LATENCY;
                d_wr_phase <= 1'b0;
            end else if (d_wr_busy) begin
                if (d_wr_lat != 0) d_wr_lat <= d_wr_lat - 1;
                else if (d_wr_phase == 1'b0) begin
                    LMEM[d_wr_addr >> 4] <= aj_wline;
                    if (d_wr_left > 1) aj_wnext_r <= 1'b1;
                    d_wr_phase <= 1'b1;
                end else begin
                    d_wr_phase <= 1'b0;
                    d_wr_addr  <= d_wr_addr + 32'd16;
                    d_wr_left  <= d_wr_left - 1;
                    if (d_wr_left == 1) begin
                        d_wr_busy     <= 1'b0;
                        aj_wr_ready_r <= 1'b1;
                    end
                end
            end
        end
    end
`else
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
        .mem_wnext(mem_wnext), .mem_rline(mem_rline)
    );
`endif

endmodule
