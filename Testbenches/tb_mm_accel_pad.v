`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"

// B-scratchpad test: does holding several B column-panels locally actually
// remove the per-tile operand reload, and what is it worth?
//
// For output tile (ti,tj) the array needs A row-panel ti and B column-panel tj.
// Software already hoists the A load out of the inner loop, but B was reloaded
// for EVERY tile - which is why operand load stayed 81% of tile time on
// hardware even after packed pushes and lane auto-advance.
//
// Two schedules on identical data, both verified against the same reference:
//
//   RELOAD    load B panel, run, read. Repeat per tile. (what we had)
//   RESIDENT  load all panels once, then per tile just select and run.
//
// The gap is the value of the scratchpad. Correctness matters as much as the
// cycles here: selecting the wrong panel would still produce plausible-looking
// numbers, so every tile is checked against ITS OWN panel's reference. Each
// panel gets distinct operands precisely so a stuck or ignored B_PANEL_USE
// cannot pass.
`ifndef DIM
  `define DIM 8
`endif
`ifndef KLEN
  `define KLEN 16
`endif
`ifndef PANELS
  `define PANELS 4
`endif

module tb_mm_accel_pad;
    localparam DIM    = `DIM;
    localparam KLEN   = `KLEN;
    localparam PANELS = `PANELS;
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

    wire         a_req_valid, a_req_write;
    wire [31:0]  a_req_addr;
    wire [127:0] a_wline;

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
        .mem_req_valid(a_req_valid), .mem_req_write(a_req_write),
        .mem_req_addr(a_req_addr), .mem_wline(a_wline),
        .mem_req_lines(),
        .mem_rline(128'b0), .mem_ready(1'b0)
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
    reg signed [7:0]  B [0:PANELS-1][0:DIM-1][0:KLEN-1];
    reg signed [31:0] CREF [0:PANELS-1][0:DIM-1][0:DIM-1];

    integer i, j, k, g, p, errors, acc;
    integer t0, c_reload, c_resident;
    reg [31:0] rd, pw;

    task push_a;
        begin
            do_write(wr(4), 0);
            for (i = 0; i < DIM; i = i + 1)
                for (g = 0; g < KLEN/4; g = g + 1) begin
                    pw = { A[i][4*g+3][7:0], A[i][4*g+2][7:0],
                           A[i][4*g+1][7:0], A[i][4*g+0][7:0] };
                    do_write(wr(5), pw);
                end
        end
    endtask

    // Push B panel p into scratchpad slot `slot`.
    task push_b(input integer p_, input integer slot);
        begin
            do_write(wr(16), slot);             // B_PANEL_LOAD
            do_write(wr(4), 0);                 // LOAD_LANE
            for (j = 0; j < DIM; j = j + 1)
                for (g = 0; g < KLEN/4; g = g + 1) begin
                    pw = { B[p_][j][4*g+3][7:0], B[p_][j][4*g+2][7:0],
                           B[p_][j][4*g+1][7:0], B[p_][j][4*g+0][7:0] };
                    do_write(wr(6), pw);
                end
        end
    endtask

    task run_and_check(input integer p_);
        begin
            do_write(wr(0), 32'd1);             // START
            rd = 0;
            while (!rd[1]) do_read(wr(1), rd);  // DONE
            do_write(wr(7), 0);                 // RESULT_IDX = 0
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    do_read(wr(8), rd);
                    if ($signed(rd) !== CREF[p_][i][j]) begin
                        if (errors < 6)
                            $display("  MISMATCH panel %0d C[%0d][%0d]: got %0d expected %0d",
                                     p_, i, j, $signed(rd), CREF[p_][i][j]);
                        errors = errors + 1;
                    end
                end
        end
    endtask

    initial begin
        errors = 0;

        for (i = 0; i < DIM; i = i + 1)
            for (k = 0; k < KLEN; k = k + 1)
                A[i][k] = $signed((i*3 + k*5) % 11) - 5;

        // Each panel gets clearly distinct operands: a stuck B_PANEL_USE must
        // produce wrong answers rather than coincidentally right ones.
        for (p = 0; p < PANELS; p = p + 1)
            for (j = 0; j < DIM; j = j + 1)
                for (k = 0; k < KLEN; k = k + 1)
                    B[p][j][k] = $signed((j*2 + k*3 + p*7) % 13) - 6;

        for (p = 0; p < PANELS; p = p + 1)
            for (i = 0; i < DIM; i = i + 1)
                for (j = 0; j < DIM; j = j + 1) begin
                    acc = 0;
                    for (k = 0; k < KLEN; k = k + 1) acc = acc + A[i][k]*B[p][j][k];
                    CREF[p][i][j] = acc;
                end

        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_read(wr(9), rd);
        $display("INFO: dim=%0d maxK=%0d bPanels=%0d | testing %0d panels, K=%0d",
                 rd[7:0], rd[15:8], rd[23:16], PANELS, KLEN);
        if (rd[23:16] < PANELS) begin
            $display("FAIL: hardware has %0d panels, test needs %0d", rd[23:16], PANELS);
            $finish;
        end

        do_write(wr(2), KLEN);
        push_a();

        // ---- schedule 1: reload B every tile (always into slot 0) ----
        t0 = cyc;
        for (p = 0; p < PANELS; p = p + 1) begin
            push_b(p, 0);
            do_write(wr(15), 0);                // B_PANEL_USE = 0
            run_and_check(p);
        end
        c_reload = cyc - t0;

        // ---- schedule 2: panels resident, select per tile ----
        for (p = 0; p < PANELS; p = p + 1) push_b(p, p);   // fill once, untimed
        t0 = cyc;
        for (p = 0; p < PANELS; p = p + 1) begin
            do_write(wr(15), p);                // B_PANEL_USE = p, no reload
            run_and_check(p);
        end
        c_resident = cyc - t0;

        $display("");
        $display("  RELOAD   (B pushed per tile) %0d cyc", c_reload);
        $display("  RESIDENT (panels pre-loaded) %0d cyc", c_resident);
        if (c_resident > 0)
            $display("  scratchpad speedup x100      %0d",
                     (c_reload * 100) / c_resident);

        $display("");
        if (errors == 0)
            $display("=== DIM=%0d PANELS=%0d: SCRATCHPAD CORRECT ===", DIM, PANELS);
        else
            $display("=== DIM=%0d PANELS=%0d: %0d ERROR(S) ===", DIM, PANELS, errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
