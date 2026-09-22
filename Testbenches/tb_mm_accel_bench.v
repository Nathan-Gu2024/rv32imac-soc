`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"
`include "../src/accel_port_join.v"

// Performance benchmark for the Chisel GEMM generator - a REAL tiled GEMM,
// not a correctness check. tb_mm_accel_gen.v already proves the arithmetic;
// this one measures where the cycles actually go.
//
// Computes C[M][N] = A[M][K] * B[K][N] in DIM x DIM output tiles and reports
// the cycle split between operand load, compute and readback, because the
// headline claim about this accelerator - that it is MMIO-bound rather than
// compute-bound - has so far been an estimate rather than a measurement.
//
// Two schedules are timed on identical data:
//
//   NAIVE  reload A and B for every output tile.
//   TILED  hoist the A load out of the inner loop. The operand buffers persist
//          across runs, so for a fixed row-tile the A panel can be loaded once
//          and reused for every column-tile. Costs nothing in hardware.
//
// Both must produce the same, verified-correct C. The gap between them is the
// value of tiling alone, with no RTL change.
//
// Build:  iverilog -g2012 -o bench -DDIM=n -DKLEN=k -I../src \
//                   tb_mm_accel_bench.v mm_accel.sv
`ifndef DIM
  `define DIM 4
`endif
`ifndef KLEN
  `define KLEN 8
`endif
`ifndef TILES
  `define TILES 2
`endif

module tb_mm_accel_bench;
    localparam DIM   = `DIM;
    localparam KLEN  = `KLEN;      // must be a multiple of 4 (packed pushes)
    localparam TILES = `TILES;     // output tiles per dimension
    localparam M     = DIM * TILES;
    localparam N     = DIM * TILES;

    localparam ACCEL_BASE = 32'h0000_5000;

    reg clk = 1'b0, rst = 1'b1;
    always #5 clk = ~clk;

    // free-running cycle counter - the only clock this bench trusts
    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    // Split the per-transaction cost into "time the bridge+accelerator held the
    // request" versus "time the requester spent around it". Without this the
    // ~7 cycles/transaction is unattributable, and the fix would be aimed at
    // the wrong side of the interface.
    integer req_cycles = 0, req_count = 0;
    reg     d_req_prev = 1'b0;
    always @(posedge clk) begin
        if (d_req)               req_cycles <= req_cycles + 1;
        if (d_req && !d_req_prev) req_count <= req_count + 1;
        d_req_prev <= d_req;
    end

    // ---- accelerator result-DMA port + mock line memory ----
    // Stands in for mem_arbiter. MEM_LATENCY is non-zero because the real port
    // shares the arbiter with two caches; a zero-wait responder would also
    // flatter the DMA and hide the settle cycle it needs per line.
    wire         a_req_valid, a_req_write;
    wire [31:0]  a_req_addr;
    wire [127:0] a_wline;
    reg          a_ready = 1'b0;

    localparam DMA_BASE    = 32'h0010_0000;
    localparam MEM_LATENCY = 3;

    reg [31:0] DMEM [0:4*1024-1];       // word-addressed, base DMA_BASE
    integer    a_lat, dma_lines;

    // Burst-capable, matching tb_mm_accel_queue.v's model.
    //
    // This was a single-line responder: it ignored mem_req_lines, did not drive
    // mem_wnext at all, and wrote exactly one line per request. With
    // enableWriteBursts the accelerator asks for a whole tile in ONE request, so
    // 15 of 16 lines were dropped and the upper result columns read back X - which
    // presented as a design regression rather than a stale bench.
    //
    // mem_wnext was not even in the instantiation below, so it was a dangling
    // INPUT on mm_accel: the requester's lineFifo advance was driven by X.
    //
    // Protocol: one address phase, a line every BEATS_PER_LINE=2 cycles, wnext
    // one line AHEAD (gated on a next line existing), and one mem_ready for the
    // whole write transaction.
    wire [7:0] a_req_lines;
    reg        a_wnext;
    reg [31:0] a_baddr;
    reg [31:0] a_left;
    reg        a_in_burst, a_write_l, a_phase;

    always @(posedge clk) begin
        if (rst) begin
            a_ready <= 1'b0; a_lat <= 0; dma_lines <= 0;
            a_in_burst <= 1'b0; a_left <= 0; a_phase <= 1'b0; a_wnext <= 1'b0;
        end else if (a_in_burst) begin
            a_wnext <= 1'b0;
            a_ready <= 1'b0;
            if (a_left > 0) begin
                if (a_phase == 1'b0) begin
                    if (a_write_l) begin
                        DMEM[((a_baddr - DMA_BASE) >> 2) + 0] <= a_wline[31:0];
                        DMEM[((a_baddr - DMA_BASE) >> 2) + 1] <= a_wline[63:32];
                        DMEM[((a_baddr - DMA_BASE) >> 2) + 2] <= a_wline[95:64];
                        DMEM[((a_baddr - DMA_BASE) >> 2) + 3] <= a_wline[127:96];
                        dma_lines <= dma_lines + 1;
                        if (a_left > 1) a_wnext <= 1'b1;
                    end
                    a_phase <= 1'b1;
                end else begin
                    a_phase <= 1'b0;
                    a_baddr <= a_baddr + 32'd16;
                    a_left  <= a_left - 1;
                    if (a_left == 1) begin
                        a_in_burst <= 1'b0;
                        a_ready    <= 1'b1;   // one completion per write
                    end
                end
            end
        end else if (a_req_valid && !a_ready) begin
            if (a_lat >= MEM_LATENCY) begin
                a_write_l  <= a_req_write;
                a_baddr    <= a_req_addr;
                a_left     <= (a_req_lines == 8'd0) ? 32'd1 : {24'd0, a_req_lines};
                a_in_burst <= 1'b1;
                a_phase    <= 1'b0;
                a_lat      <= 0;
            end else a_lat <= a_lat + 1;
        end else begin
            a_ready <= 1'b0;
            a_wnext <= 1'b0;
        end
    end

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

    reg signed [7:0]  A [0:M-1][0:KLEN-1];
    reg signed [7:0]  B [0:KLEN-1][0:N-1];
    reg signed [31:0] CREF [0:M-1][0:N-1];
    reg signed [31:0] CDUT [0:M-1][0:N-1];

    integer i, j, k, g, ti, tj, errors;
    integer acc;
    reg [31:0] rd, pw;

    // cycle accounting
    integer t0, c_load, c_comp, c_read, c_total;

    // LOAD_LANE auto-advances when a lane's k-groups are exhausted, so a whole
    // panel loads as ONE index write followed by DIM*(KLEN/4) back-to-back
    // pushes - no per-lane bookkeeping.
    task push_a_panel(input integer ti_);
        begin
            do_write(wr(4), 0);                 // LOAD_LANE = 0, once
            for (i = 0; i < DIM; i = i + 1)
                for (g = 0; g < KLEN/4; g = g + 1) begin
                    pw = { A[ti_*DIM+i][4*g+3][7:0], A[ti_*DIM+i][4*g+2][7:0],
                           A[ti_*DIM+i][4*g+1][7:0], A[ti_*DIM+i][4*g+0][7:0] };
                    do_write(wr(5), pw);        // A_PUSH
                end
        end
    endtask

    task push_b_panel(input integer tj_);
        begin
            do_write(wr(4), 0);
            for (j = 0; j < DIM; j = j + 1)
                for (g = 0; g < KLEN/4; g = g + 1) begin
                    pw = { B[4*g+3][tj_*DIM+j][7:0], B[4*g+2][tj_*DIM+j][7:0],
                           B[4*g+1][tj_*DIM+j][7:0], B[4*g+0][tj_*DIM+j][7:0] };
                    do_write(wr(6), pw);        // B_PUSH
                end
        end
    endtask

    task run_tile;
        begin
            do_write(wr(0), 32'd1);             // START
            rd = 0;
            while (!rd[1]) do_read(wr(1), rd);  // poll STATUS.DONE
        end
    endtask

    // RESULT_IDX auto-increments on each RESULT read, so the whole tile is one
    // index write followed by DIM*DIM reads - half the transactions the
    // set-index-then-read sequence needed.
    task read_tile(input integer ti_, input integer tj_);
        begin
            do_write(wr(7), 0);                 // RESULT_IDX = 0, once per tile
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    do_read (wr(8), rd);        // RESULT (auto-increments)
                    CDUT[ti_*DIM + i][tj_*DIM + j] = $signed(rd);
                end
        end
    endtask

    // Result readback over the DMA instead of the register window. The tile
    // lands directly in its place inside the full M x N C: DEST is the tile
    // corner and STRIDE is the whole matrix row pitch, so no software copy is
    // needed afterwards. That is what makes this a fair comparison against the
    // register-window path, which also leaves results in their final position.
    task read_tile_dma(input integer ti_, input integer tj_);
        begin
            do_write(wr(10), DMA_BASE + (ti_*DIM)*N*4 + (tj_*DIM)*4);  // DEST_ADDR
            do_write(wr(11), N*4);                                     // DEST_STRIDE
            do_write(wr(0), 32'd4);                                    // START_DMA
            rd = 0;
            while (!rd[3]) do_read(wr(1), rd);                         // DMA_DONE
        end
    endtask

    // Pull the DMA-written results back out of the mock memory for checking.
    task harvest_dma;
        begin
            for (i = 0; i < M; i = i + 1)
                for (j = 0; j < N; j = j + 1)
                    CDUT[i][j] = $signed(DMEM[i*N + j]);
        end
    endtask

    // ---- schedule 2: tiled operands + DMA readback ----
    task sched_tiled_dma;
        begin
            c_load = 0; c_comp = 0; c_read = 0;
            for (ti = 0; ti < TILES; ti = ti + 1) begin
                t0 = cyc;
                push_a_panel(ti);
                c_load = c_load + (cyc - t0);

                for (tj = 0; tj < TILES; tj = tj + 1) begin
                    t0 = cyc;
                    push_b_panel(tj);
                    c_load = c_load + (cyc - t0);

                    t0 = cyc; run_tile();            c_comp = c_comp + (cyc - t0);
                    t0 = cyc; read_tile_dma(ti,tj);  c_read = c_read + (cyc - t0);
                end
            end
            harvest_dma();
        end
    endtask

    // ---- schedule 0: reload both panels every tile ----
    task sched_naive;
        begin
            c_load = 0; c_comp = 0; c_read = 0;
            for (ti = 0; ti < TILES; ti = ti + 1)
                for (tj = 0; tj < TILES; tj = tj + 1) begin
                    t0 = cyc;
                    push_a_panel(ti);
                    push_b_panel(tj);
                    c_load = c_load + (cyc - t0);

                    t0 = cyc; run_tile();      c_comp = c_comp + (cyc - t0);
                    t0 = cyc; read_tile(ti,tj); c_read = c_read + (cyc - t0);
                end
        end
    endtask

    // ---- schedule 1: hoist the A load out of the inner loop ----
    task sched_tiled;
        begin
            c_load = 0; c_comp = 0; c_read = 0;
            for (ti = 0; ti < TILES; ti = ti + 1) begin
                t0 = cyc;
                push_a_panel(ti);
                c_load = c_load + (cyc - t0);

                for (tj = 0; tj < TILES; tj = tj + 1) begin
                    t0 = cyc;
                    push_b_panel(tj);
                    c_load = c_load + (cyc - t0);

                    t0 = cyc; run_tile();      c_comp = c_comp + (cyc - t0);
                    t0 = cyc; read_tile(ti,tj); c_read = c_read + (cyc - t0);
                end
            end
        end
    endtask

    task check_and_report(input [127:0] label);
        begin
            errors = 0;
            for (i = 0; i < M; i = i + 1)
                for (j = 0; j < N; j = j + 1)
                    if (CDUT[i][j] !== CREF[i][j]) begin
                        if (errors < 4)
                            $display("  MISMATCH C[%0d][%0d] = %0d, expected %0d",
                                     i, j, CDUT[i][j], CREF[i][j]);
                        errors = errors + 1;
                    end
            c_total = c_load + c_comp + c_read;
            $display("  %0s%0s", label, errors ? "  *** INCORRECT ***" : "");
            $display("    load    %7d cyc  (%0d%%)", c_load, (100*c_load)/c_total);
            $display("    compute %7d cyc  (%0d%%)", c_comp, (100*c_comp)/c_total);
            $display("    readback%7d cyc  (%0d%%)", c_read, (100*c_read)/c_total);
            $display("    total   %7d cyc", c_total);
            $display("    MACs %0d -> %0d.%02d MACs/cyc, array utilisation %0d.%0d%%",
                     M*N*KLEN,
                     (M*N*KLEN)/c_total, ((100*(M*N*KLEN))/c_total)%100,
                     (100*M*N*KLEN)/(DIM*DIM*c_total),
                     ((1000*M*N*KLEN)/(DIM*DIM*c_total))%10);
        end
    endtask

    integer naive_total, tiled_total;

    initial begin
        // operands: small signed values, deterministic but not trivial
        for (i = 0; i < M; i = i + 1)
            for (k = 0; k < KLEN; k = k + 1)
                A[i][k] = $signed((i*7 + k*3) % 11) - 5;
        for (k = 0; k < KLEN; k = k + 1)
            for (j = 0; j < N; j = j + 1)
                B[k][j] = $signed((k*5 + j*2) % 9) - 4;

        for (i = 0; i < M; i = i + 1)
            for (j = 0; j < N; j = j + 1) begin
                acc = 0;
                for (k = 0; k < KLEN; k = k + 1) acc = acc + A[i][k]*B[k][j];
                CREF[i][j] = acc;
            end

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        $display("INFO: dim=%0d maxK=%0d | GEMM %0dx%0dx%0d in %0dx%0d tiles",
                 rd[7:0], rd[15:8], M, N, KLEN, TILES, TILES);
        if (rd[7:0] !== DIM) begin
            $display("FAIL: built for DIM=%0d but hardware reports %0d", DIM, rd[7:0]);
            $finish;
        end

        do_write(wr(2), KLEN);   // K_LEN

        sched_naive();
        check_and_report("NAIVE (reload A and B per tile)");
        naive_total = c_total;

        $display("");
        sched_tiled();
        check_and_report("TILED (A hoisted out of inner loop)");

        tiled_total = c_total;

        $display("");
        sched_tiled_dma();
        check_and_report("TILED + RESULT DMA (128-bit line port)");
        $display("    %0d DMA line writes", dma_lines);

        $display("");
        // c_total is the DMA run at this point, so the tiling comparison must
        // use tiled_total explicitly - using c_total here reported the
        // end-to-end figure under the "tiling" label.
        $display("  tiling alone (naive -> tiled, both MMIO): %0d.%02dx",
                 naive_total/tiled_total, ((100*naive_total)/tiled_total)%100);
        $display("  DMA vs MMIO readback, same schedule:      %0d.%02dx",
                 tiled_total/c_total, ((100*tiled_total)/c_total)%100);
        $display("  end-to-end vs naive baseline:             %0d.%02dx",
                 naive_total/c_total, ((100*naive_total)/c_total)%100);
        $display("");
        $display("  transactions %0d, cycles held by bridge+accel %0d",
                 req_count, req_cycles);
        $display("  -> %0d.%0d cyc/transaction inside the interface",
                 req_cycles/req_count, ((10*req_cycles)/req_count)%10);
        $display("=== BENCH DONE ===");
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
        .mem_req_valid(a_req_valid), .mem_req_write(a_req_write),
        .mem_req_addr(a_req_addr), .mem_req_lines(a_req_lines),
        .mem_wline(a_wline), .mem_ready(a_ready),
        .mem_wnext(a_wnext), .mem_rline(128'b0)
    );

endmodule
