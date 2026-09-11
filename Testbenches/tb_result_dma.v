`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"
`include "../src/mem_arbiter.v"
`include "../src/axi_cache_adapter.v"

// Why does a RESULT write cost 11.18 cycles per line on hardware when an
// OPERAND read costs 3.53?
//
// Both burst, both cross the same mem_arbiter -> axi_cache_adapter path, both
// move 128-bit lines over the same 64-bit AXI master. The write is 3.2x worse
// per line, and that gap is now the largest single item in a tile.
//
// Every existing bench hides this. The accelerator benches drive mem_ready and
// mem_wnext directly, so they never model the adapter's beat pacing at all;
// tb_burst_integration models the plumbing but drives it from a synthetic
// requester with a zero-wait-state slave, so the accelerator's own ability to
// FEED a burst is never tested. This bench puts the real accelerator, the real
// arbiter and the real adapter together and asks where the cycles go.
//
// The measurement that matters is not the total - it is the split:
//
//   STARVED    adapter is in its write data phase but wvalid is low: the
//              accelerator has no line ready. A producer problem.
//   BACKPRESS  wvalid high but wready low: memory will not take it. A consumer
//              problem, and nothing inside the accelerator can fix it.
//
// Guessing between those two without measuring is how the earlier "FIFO
// starvation" theory survived - plausible, untestable against the benches that
// existed, and wrong.
`ifndef DIM
  `define DIM 8
`endif
`ifndef KLEN
  `define KLEN 64
`endif
// Address-phase latency of the slave, both directions. Hardware measured ~28
// cycles of round trip for a single line, so a zero-latency slave flatters
// both paths equally and says nothing about the asymmetry.
`ifndef SLAVE_LAT
  `define SLAVE_LAT 12
`endif
// Cycles from the last write beat to BVALID. The adapter holds the transaction
// open until the response arrives, so this lands directly on the accelerator's
// DMA_DONE. A model that answers instantly assumes the write is complete the
// moment the last beat leaves, which no real memory controller does.
`ifndef BRESP_LAT
  `define BRESP_LAT 0
`endif
// Cycles WREADY is withheld between accepted beats. Zero means the sink takes
// a beat every cycle for the whole burst.
`ifndef WREADY_GAP
  `define WREADY_GAP 0
`endif

module tb_result_dma;
    localparam DIM   = `DIM;
    localparam KLEN  = `KLEN;
    localparam LPL   = (KLEN + 15) / 16;
    localparam AXI_W = 64;
    localparam ACCEL_BASE = 32'h0000_5000;
    localparam A_SRC = 32'h0004_0000;
    localparam B_SRC = 32'h0005_0000;
    localparam DEST  = 32'h0006_0000;
    localparam DEST2 = 32'h0007_0000;
    localparam QDEST = 32'h0008_0000;
    localparam A8_SRC = 32'h0009_0000;
    localparam B8_SRC = 32'h000A_0000;
    localparam D8     = 32'h000B_0000;
    localparam NTILE = 4;
    localparam NGROUPS = (DIM*DIM)/4;          // result lines per tile

    reg clk = 1'b0, rst = 1'b1;
    always #5 clk = ~clk;
    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    // ---- MMIO ----
    reg         d_req = 1'b0, d_we = 1'b0;
    reg  [31:0] d_addr = 32'b0, d_wdata = 32'b0;
    wire [31:0] d_rdata;
    wire        d_ready;
    wire [31:0] m_awaddr, m_wdata, m_araddr, m_rdata;
    wire [3:0]  m_wstrb;
    wire [1:0]  m_bresp, m_rresp;
    wire        m_awvalid, m_awready, m_wvalid, m_wready, m_bvalid, m_bready;
    wire        m_arvalid, m_arready, m_rvalid, m_rready;

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

    // ---- accelerator line port ----
    wire         a_req_valid, a_req_write, a_ready, a_wnext;
    wire [7:0]   a_req_lines;
    wire [31:0]  a_req_addr;
    wire [127:0] a_wline, a_rline;

    mm_accel ACCEL (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(m_awaddr), .s_axi_awvalid(m_awvalid), .s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata), .s_axi_wstrb(m_wstrb), .s_axi_wvalid(m_wvalid), .s_axi_wready(m_wready),
        .s_axi_bresp(m_bresp), .s_axi_bvalid(m_bvalid), .s_axi_bready(m_bready),
        .s_axi_araddr(m_araddr), .s_axi_arvalid(m_arvalid), .s_axi_arready(m_arready),
        .s_axi_rdata(m_rdata), .s_axi_rresp(m_rresp), .s_axi_rvalid(m_rvalid), .s_axi_rready(m_rready),
        .mem_req_valid(a_req_valid), .mem_req_write(a_req_write),
        .mem_req_addr(a_req_addr), .mem_wline(a_wline), .mem_req_lines(a_req_lines),
        .mem_wnext(a_wnext), .mem_rline(a_rline), .mem_ready(a_ready)
    );

    // ---- arbiter: caches idle, so the accelerator is alone on the port ----
    wire         mem_req_valid, mem_req_write, mem_ready, mem_wnext;
    wire [7:0]   mem_req_lines;
    wire [31:0]  mem_req_addr;
    wire [127:0] mem_wline, mem_rline;

    mem_arbiter ARB (
        .clk(clk), .rst(rst),
        .icache_req_valid(1'b0), .icache_req_addr(32'b0),
        .icache_ready(), .icache_rline(),
        .dcache_req_valid(1'b0), .dcache_req_write(1'b0),
        .dcache_req_addr(32'b0), .dcache_wline(128'b0),
        .dcache_ready(), .dcache_rline(),
        .accel_req_valid(a_req_valid), .accel_req_lines(a_req_lines),
        .accel_req_write(a_req_write), .accel_req_addr(a_req_addr),
        .accel_wline(a_wline), .accel_ready(a_ready),
        .accel_wnext(a_wnext), .accel_rline(a_rline),
        .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
        .mem_req_lines(mem_req_lines), .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline), .mem_ready(mem_ready),
        .mem_wnext(mem_wnext), .mem_rline(mem_rline)
    );

    // ---- AXI ----
    wire [31:0] awaddr, araddr;
    wire [7:0]  awlen, arlen;
    wire [2:0]  awsize, arsize;
    wire [1:0]  awburst, arburst;
    wire        awvalid, arvalid, wvalid, wlast, rready, bready;
    reg         awready = 1'b0, arready = 1'b0, wready = 1'b0;
    wire [AXI_W-1:0]   wdata;
    wire [AXI_W/8-1:0] wstrb;
    reg  [AXI_W-1:0]   rdata = {AXI_W{1'b0}};
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

    // ---- AXI slave with a realistic address-phase latency ----
    reg [127:0] LMEM [0:65535];
    integer ar_lat, aw_lat, beats_left, wr_beats_left;
    integer b_lat, w_gap;
    reg     wr_active, b_pending;
    reg [31:0] rd_addr, wr_addr;
    reg        rd_half, wr_half;
    reg [63:0] wr_acc;

    always @(posedge clk) begin
        if (rst) begin
            arready <= 1'b0; awready <= 1'b0; wready <= 1'b0;
            rvalid  <= 1'b0; rlast   <= 1'b0; bvalid  <= 1'b0;
            beats_left <= 0; wr_beats_left <= 0; wr_active <= 1'b0;
            ar_lat <= 0; aw_lat <= 0; rd_half <= 1'b0; wr_half <= 1'b0;
            b_lat <= 0; w_gap <= 0; b_pending <= 1'b0;
        end else begin
            // ---- read address ----
            arready <= 1'b0;
            if (arvalid && !arready && beats_left == 0 && !rvalid) begin
                if (ar_lat >= `SLAVE_LAT) begin
                    arready    <= 1'b1;
                    beats_left <= arlen + 1;
                    rd_addr    <= araddr;
                    rd_half    <= 1'b0;
                    ar_lat     <= 0;
                end else ar_lat <= ar_lat + 1;
            end
            // ---- read data: one beat per cycle once started ----
            if (beats_left > 0) begin
                if (!rvalid) begin
                    rvalid <= 1'b1;
                    rdata  <= rd_half ? LMEM[rd_addr >> 4][127:64]
                                      : LMEM[rd_addr >> 4][63:0];
                    rlast  <= (beats_left == 1);
                end else if (rready) begin
                    beats_left <= beats_left - 1;
                    if (rd_half) rd_addr <= rd_addr + 32'd16;
                    rd_half <= ~rd_half;
                    if (beats_left == 1) begin
                        rvalid <= 1'b0; rlast <= 1'b0;
                    end else begin
                        rdata <= (~rd_half) ? LMEM[rd_addr >> 4][127:64]
                                            : LMEM[(rd_addr + 32'd16) >> 4][63:0];
                        rlast <= (beats_left == 2);
                    end
                end
            end
            // ---- write address ----
            awready <= 1'b0;
            if (awvalid && !awready && !wr_active) begin
                if (aw_lat >= `SLAVE_LAT) begin
                    awready       <= 1'b1;
                    wr_active     <= 1'b1;
                    wr_beats_left <= awlen + 1;
                    wr_addr       <= awaddr;
                    wr_half       <= 1'b0;
                    wready        <= 1'b1;
                    aw_lat        <= 0;
                end else aw_lat <= aw_lat + 1;
            end
            // ---- write data ----
            if (wr_active && wvalid && wready) begin
                if (!wr_half) wr_acc <= wdata;
                else begin
                    LMEM[wr_addr >> 4] <= {wdata, wr_acc};
                    wr_addr <= wr_addr + 32'd16;
                end
                wr_half       <= ~wr_half;
                wr_beats_left <= wr_beats_left - 1;
                if (wlast || wr_beats_left == 1) begin
                    wready <= 1'b0; wr_active <= 1'b0; b_pending <= 1'b1;
                    b_lat  <= 0;
                end else if (`WREADY_GAP != 0) begin
                    wready <= 1'b0;              // make the sink pause
                    w_gap  <= 0;
                end
            end else if (wr_active && !wready && !b_pending) begin
                if (w_gap >= `WREADY_GAP) wready <= 1'b1;
                else                      w_gap  <= w_gap + 1;
            end
            // ---- write response ----
            if (b_pending && !bvalid) begin
                if (b_lat >= `BRESP_LAT) begin bvalid <= 1'b1; b_pending <= 1'b0; end
                else                          b_lat  <= b_lat + 1;
            end
            if (bvalid && bready) bvalid <= 1'b0;
        end
    end

    // ---- the measurement: where do the write cycles actually go? ----
    integer w_starved, w_backpress, w_moving, w_addr_wait;
    integer r_stalled, r_moving, r_addr_wait;
    always @(posedge clk) begin
        if (rst) begin
            w_starved <= 0; w_backpress <= 0; w_moving <= 0; w_addr_wait <= 0;
            r_stalled <= 0; r_moving <= 0; r_addr_wait <= 0;
        end else begin
            if (awvalid && !awready) w_addr_wait <= w_addr_wait + 1;
            if (arvalid && !arready) r_addr_wait <= r_addr_wait + 1;
            if (wr_active) begin
                if (wvalid && wready)  w_moving    <= w_moving + 1;
                else if (!wvalid)      w_starved   <= w_starved + 1;
                else                   w_backpress <= w_backpress + 1;
            end
            if (beats_left > 0) begin
                if (rvalid && rready)  r_moving  <= r_moving + 1;
                else                   r_stalled <= r_stalled + 1;
            end
        end
    end

    // The point of overlapping store with compute is that the ARRAY and the
    // PORT are different resources. Counting both says whether the batch is
    // now limited by memory (as it should be) or still serialising.
    reg qtiming;
    initial qtiming = 1'b0;

    // Per-PHASE accounting, not just port-vs-array. Subtracting modelled port
    // and compute time from the batch left ~109 cycles per tile unaccounted on
    // hardware, and attributing that by arithmetic is how the last two
    // estimates went wrong. qState says exactly where the sequencer sits.
    integer q_idle, q_load, q_loadw, q_comp, q_compw, q_store, q_storew;
    always @(posedge clk) begin
        if (rst) begin
            port_busy <= 0; array_busy <= 0;
            q_idle <= 0; q_load <= 0; q_loadw <= 0; q_comp <= 0;
            q_compw <= 0; q_store <= 0; q_storew <= 0;
        end else if (qtiming) begin
            if (mem_req_valid || ADP.state != 3'd0) port_busy  <= port_busy + 1;
            if (ACCEL.busy)                         array_busy <= array_busy + 1;
            case (ACCEL.qState)
                3'd0: q_idle   <= q_idle   + 1;
                3'd1: q_load   <= q_load   + 1;
                3'd2: q_loadw  <= q_loadw  + 1;
                3'd3: q_comp   <= q_comp   + 1;
                3'd4: q_compw  <= q_compw  + 1;
                3'd5: q_store  <= q_store  + 1;
                3'd6: q_storew <= q_storew + 1;
            endcase
        end
    end

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

    function [31:0] wr; input integer w; begin wr = ACCEL_BASE + (w*4); end endfunction

    integer i, j, c, k, p, t0, c_load, c_comp, c_store, c_store2, bad, got;
    integer c_queue, qbad, port_busy, array_busy;
    integer c_store8, bad8, expq, sat;
    reg signed [7:0] got8;
    reg [31:0] rd;
    reg [127:0] lw;

    initial begin
        // A and B all ones, so every C[i][j] = KLEN and a mis-stored line is
        // obvious rather than plausible.
        for (i = 0; i < DIM; i = i + 1)
            for (c = 0; c < LPL; c = c + 1) begin
                for (k = 0; k < 16; k = k + 1)
                    lw[8*k +: 8] = ((c*16+k) < KLEN) ? 8'd1 : 8'd0;
                LMEM[(A_SRC + i*KLEN + c*16) >> 4] = lw;
                LMEM[(B_SRC + i*KLEN + c*16) >> 4] = lw;
            end

        repeat (4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        $display("INFO: dim=%0d maxK=%0d | K=%0d, slave latency %0d cyc",
                 rd[7:0], rd[15:8], KLEN, `SLAVE_LAT);

        do_write(wr(2),  KLEN);
        do_write(wr(12), A_SRC);
        do_write(wr(13), B_SRC);
        do_write(wr(14), KLEN);          // packed lanes -> burst
        do_write(wr(16), 0);
        do_write(wr(15), 0);

        t0 = cyc;
        do_write(wr(0), 32'd8);                       // START_LOAD
        rd = 0; while (!rd[5]) do_read(wr(1), rd);
        c_load = cyc - t0;

        t0 = cyc;
        do_write(wr(0), 32'd1);                       // START compute
        rd = 0; while (!rd[1]) do_read(wr(1), rd);
        c_comp = cyc - t0;

        do_write(wr(10), DEST);
        do_write(wr(11), DIM*4);                      // contiguous destination
        t0 = cyc;
        do_write(wr(0), 32'd4);                       // START_DMA
        rd = 0; while (!rd[3]) do_read(wr(1), rd);
        c_store = cyc - t0;

        // ---- same results, STRIDED destination ----
        //
        // A non-contiguous destination makes the DMA issue dim/4 lines per
        // burst instead of the whole tile, so the same 16 lines cross the AXI
        // master as 8 transactions rather than 1. That is the discriminator:
        //
        //   response-latency bound -> cost scales with TRANSACTIONS, so this
        //                             is roughly 8x worse
        //   beat-acceptance bound  -> cost scales with BEATS, so this is about
        //                             the same, plus 7 extra address phases
        //
        // Both fit the hardware number when only the contiguous case is
        // measured, which is why measuring only that case settles nothing.
        do_write(wr(10), DEST2);
        do_write(wr(11), DIM*4*2);                    // rows spread out
        t0 = cyc;
        do_write(wr(0), 32'd4);
        rd = 0; while (!rd[3]) do_read(wr(1), rd);
        c_store2 = cyc - t0;

        bad = 0;
        for (i = 0; i < DIM*DIM; i = i + 1) begin
            got = LMEM[(DEST + (i/4)*16) >> 4][32*(i%4) +: 32];
            if (got !== KLEN) bad = bad + 1;
        end
        if (bad != 0) $display("  FAIL: %0d of %0d result words wrong", bad, DIM*DIM);
        else          $display("  results correct");

        // ---- queued batch of NTILE tiles, A resident across the row ----
        for (p = 0; p < NTILE; p = p + 1)
            for (i = 0; i < DIM; i = i + 1)
                for (c = 0; c < LPL; c = c + 1) begin
                    for (k = 0; k < 16; k = k + 1)
                        lw[8*k +: 8] = ((c*16+k) < KLEN) ? (p + 1) : 8'd0;
                    LMEM[(B_SRC + p*32'h1000 + i*KLEN + c*16) >> 4] = lw;
                end
        for (p = 0; p < NTILE; p = p + 1)
            for (i = 0; i < (DIM*DIM)/4; i = i + 1)
                LMEM[(QDEST + p*32'h1000 + i*16) >> 4] = 128'h0;

        do_write(wr(11), DIM*4);
        qtiming = 1'b1;
        t0 = cyc;
        for (p = 0; p < NTILE; p = p + 1) begin
            do_write(wr(17), A_SRC);
            do_write(wr(18), B_SRC + p*32'h1000);
            do_write(wr(19), QDEST + p*32'h1000);
            do_write(wr(20), (p != 0 ? 32'h1_0000 : 32'h0) |
                             ((p & 4'hF) << 12) | ((p & 4'hF) << 8) | KLEN);
        end
        do_write(wr(0), 32'd32);
        rd = 0; while (!rd[7]) do_read(wr(1), rd);
        c_queue = cyc - t0;
        qtiming = 1'b0;

        qbad = 0;
        for (p = 0; p < NTILE; p = p + 1)
            for (i = 0; i < DIM*DIM; i = i + 1) begin
                got = LMEM[(QDEST + p*32'h1000 + (i/4)*16) >> 4][32*(i%4) +: 32];
                if (got !== KLEN*(p+1)) qbad = qbad + 1;
            end
        if (qbad != 0) $display("  FAIL: %0d queued result words wrong", qbad);
        else           $display("  queued batch correct (%0d tiles)", NTILE);

        // ---- INT8 requantized output ----
        //
        // A[i][k] = i+1 and B[j][k] = j+1, so C[i][j] = KLEN*(i+1)*(j+1) and
        // every element differs by position. Uniform operands would make a
        // transposed, rotated or mis-packed line indistinguishable from a
        // correct one, which is exactly the failure this mode can produce:
        // a line now holds 16/dim result ROWS rather than 4 words of one.
        //
        // shift 6 divides out the KLEN=64 factor exactly, so the expected
        // value is (i+1)*(j+1), 1..64 - inside INT8 with no clipping. The
        // saturating case is checked separately below at shift 5, where the
        // top corner overflows on purpose.
        for (i = 0; i < DIM; i = i + 1)
            for (c = 0; c < LPL; c = c + 1) begin
                for (k = 0; k < 16; k = k + 1)
                    lw[8*k +: 8] = ((c*16+k) < KLEN) ? (i + 1) : 8'd0;
                LMEM[(A8_SRC + i*KLEN + c*16) >> 4] = lw;
                LMEM[(B8_SRC + i*KLEN + c*16) >> 4] = lw;
            end
        for (i = 0; i < 64; i = i + 1) LMEM[(D8 + i*16) >> 4] = 128'h0;

        do_write(wr(12), A8_SRC);
        do_write(wr(13), B8_SRC);
        do_write(wr(14), KLEN);
        do_write(wr(16), 0);
        do_write(wr(15), 0);
        do_write(wr(0), 32'd8);
        rd = 0; while (!rd[5]) do_read(wr(1), rd);
        do_write(wr(0), 32'd1);
        rd = 0; while (!rd[1]) do_read(wr(1), rd);

        // Check the ACCUMULATORS first, through the register window, before
        // blaming the packing. The earlier correctness test used all-ones
        // operands, so every C[i][j] was identical and a row- or column-swap
        // in the array would have passed it unnoticed.
        bad = 0;
        do_write(wr(7), 0);
        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                do_read(wr(8), rd);
                if ($signed(rd) !== KLEN*(i+1)*(j+1)) begin
                    if (bad < 4)
                        $display("  ACC MISMATCH C[%0d][%0d]: got %0d expected %0d",
                                 i, j, $signed(rd), KLEN*(i+1)*(j+1));
                    bad = bad + 1;
                end
            end
        if (bad != 0) $display("  FAIL: %0d ACCUMULATORS wrong - not a packing bug", bad);
        else          $display("  accumulators correct with position-varying operands");

        // INT32 store with the SAME position-varying operands, before touching
        // INT8 at all. Every other DMA check in this bench uses uniform
        // operands, where a line- or beat-level reordering writes the same
        // bytes either way and passes. If the INT32 path is skewed too, the
        // fault is in the shared write path, not in requantized packing.
        do_write(wr(10), D8);
        do_write(wr(11), DIM*4);
        do_write(wr(0), 32'd4);
        rd = 0; while (!rd[3]) do_read(wr(1), rd);
        bad = 0;
        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                got = LMEM[(D8 + ((i*DIM+j)/4)*16) >> 4][32*((i*DIM+j)%4) +: 32];
                if (got !== KLEN*(i+1)*(j+1)) begin
                    if (bad < 4)
                        $display("  INT32-DMA MISMATCH C[%0d][%0d]: got %0d expected %0d",
                                 i, j, got, KLEN*(i+1)*(j+1));
                    bad = bad + 1;
                end
            end
        if (bad != 0)
            $display("  FAIL: %0d INT32 DMA results wrong with position-varying data", bad);
        else
            $display("  INT32 DMA correct with position-varying data");
        for (i = 0; i < 64; i = i + 1) LMEM[(D8 + i*16) >> 4] = 128'h0;

        do_write(wr(22), 32'h100 | 32'd6);            // OUT_CTRL: int8, shift 6
        do_write(wr(10), D8);
        do_write(wr(11), 0);
        t0 = cyc;
        do_write(wr(0), 32'd4);
        rd = 0; while (!rd[3]) do_read(wr(1), rd);
        c_store8 = cyc - t0;

        $display("  INT8 grid (expect (i+1)*(j+1)):");
        for (i = 0; i < DIM; i = i + 1) begin
            $write("   row %0d:", i);
            for (j = 0; j < DIM; j = j + 1) begin
                got8 = LMEM[(D8 + ((i*DIM+j)/16)*16) >> 4][8*((i*DIM+j)%16) +: 8];
                $write(" %0d", got8);
            end
            $display("");
        end
        $display("  raw lines:");
        for (i = 0; i < (DIM*DIM)/16; i = i + 1)
            $display("   line %0d = %032x", i, LMEM[(D8 + i*16) >> 4]);

        bad8 = 0;
        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                got8 = LMEM[(D8 + ((i*DIM+j)/16)*16) >> 4][8*((i*DIM+j)%16) +: 8];
                if (got8 !== ((i+1)*(j+1))) begin
                    if (bad8 < 4)
                        $display("  INT8 MISMATCH C[%0d][%0d]: got %0d expected %0d",
                                 i, j, got8, (i+1)*(j+1));
                    bad8 = bad8 + 1;
                end
            end

        // saturation: shift 5 leaves 2*(i+1)*(j+1), which clips above 127
        do_write(wr(22), 32'h100 | 32'd5);
        do_write(wr(10), D8);
        do_write(wr(0), 32'd4);
        rd = 0; while (!rd[3]) do_read(wr(1), rd);
        sat = 0;
        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                got8 = LMEM[(D8 + ((i*DIM+j)/16)*16) >> 4][8*((i*DIM+j)%16) +: 8];
                expq = 2*(i+1)*(j+1);
                if (expq > 127) expq = 127;
                if (got8 !== expq) begin
                    if (sat < 4)
                        $display("  SAT MISMATCH C[%0d][%0d]: got %0d expected %0d",
                                 i, j, got8, expq);
                    sat = sat + 1;
                end
            end
        do_write(wr(22), 32'd0);                      // back to INT32

        if (bad8 != 0) $display("  FAIL: %0d INT8 results wrong", bad8);
        else if (sat != 0) $display("  FAIL: %0d saturated results wrong", sat);
        else $display("  INT8 requantized output correct (incl. saturation)");

        $display("");
        $display("  operand load  %0d cyc for %0d lines  (%0d.%02d cyc/line)",
                 c_load, 2*DIM*LPL,
                 c_load/(2*DIM*LPL), ((c_load*100)/(2*DIM*LPL)) % 100);
        $display("  compute       %0d cyc", c_comp);
        $display("  result store  %0d cyc for %0d lines  (%0d.%02d cyc/line)  [1 burst]",
                 c_store, NGROUPS,
                 c_store/NGROUPS, ((c_store*100)/NGROUPS) % 100);
        $display("  strided store %0d cyc for %0d lines  (%0d.%02d cyc/line)  [%0d bursts]",
                 c_store2, NGROUPS,
                 c_store2/NGROUPS, ((c_store2*100)/NGROUPS) % 100, DIM/2);
        $display("  strided / contiguous x100  %0d", (c_store2*100)/c_store);
        $display("");
        $display("  WRITE data phase: moving %0d, STARVED %0d, backpressure %0d",
                 w_moving, w_starved, w_backpress);
        $display("  WRITE address wait  %0d  (bresp lat %0d, wready gap %0d)",
                 w_addr_wait, `BRESP_LAT, `WREADY_GAP);
        $display("  READ  data phase: moving %0d, stalled %0d", r_moving, r_stalled);
        $display("  READ  address wait  %0d", r_addr_wait);
        $display("");
        $display("  QUEUED BATCH  %0d cyc, %0d cyc/tile", c_queue, c_queue/NTILE);
        $display("    port busy   %0d  (%0d%% of batch)",
                 port_busy, (port_busy*100)/c_queue);
        $display("    array busy  %0d  (%0d%% of batch)",
                 array_busy, (array_busy*100)/c_queue);
        $display("    overlap: array+port %0d vs batch %0d",
                 array_busy + port_busy, c_queue);
        $display("    per-phase cycles over %0d tiles (per tile in brackets):", NTILE);
        $display("      qIdle  %5d [%3d]   descriptor pop / batch end",
                 q_idle,   q_idle/NTILE);
        $display("      qLoad  %5d [%3d] + qLoadW %5d [%3d]   operand fetch",
                 q_load,  q_load/NTILE,  q_loadw,  q_loadw/NTILE);
        $display("      qComp  %5d [%3d] + qCompW %5d [%3d]   array run",
                 q_comp,  q_comp/NTILE,  q_compw,  q_compw/NTILE);
        $display("      qStore %5d [%3d] + qStoreW%5d [%3d]   result write",
                 q_store, q_store/NTILE, q_storew, q_storew/NTILE);
        $display("");
        $display("  store INT32 %0d cyc (%0d lines)   store INT8 %0d cyc (%0d lines)",
                 c_store, NGROUPS, c_store8, (DIM*DIM)/16);
        if (c_store8 > 0)
            $display("  requantized writeback speedup x100  %0d",
                     (c_store * 100) / c_store8);
        $display("");
        if (w_starved > w_moving)
            $display("  => the accelerator cannot FEED the burst (producer bound)");
        else if (w_backpress > w_moving)
            $display("  => memory will not accept the data (consumer bound)");
        else
            $display("  => write data phase runs at rate; the cost is elsewhere");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
