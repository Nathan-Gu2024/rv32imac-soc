`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"

// Generator testbench for the Chisel mm_accel (chisel/src/MmAccel.scala).
//
// Unlike tb_mm_accel.v, which is hardwired to the 2x2 register map, this drives
// the INDEXED map and computes its own reference GEMM, so the SAME testbench
// validates every array size the generator emits. That is the point of the
// exercise: one source, one bench, four designs.
//
//   scala-cli run chisel/src/MmAccel.scala          (MM_DIM=n MM_OUT=dir)
//   iverilog -g2012 -DDIM=n -o /tmp/tb -s tb_mm_accel_gen \
//       Testbenches/tb_mm_accel_gen.v dir/mm_accel.sv
//   vvp /tmp/tb
//
// DIM must match the MM_DIM the Verilog was elaborated with - the design has no
// Verilog parameter to read it from. The bench checks that against the INFO
// register (word 9) and aborts on a mismatch rather than producing nonsense.
//
// Register map driven here (word offsets, byte address = word*4):
//   0 CTRL  1 STATUS  2 K_LEN  3 LOAD_K  4 LOAD_LANE
//   5 A_PUSH  6 B_PUSH  7 RESULT_IDX  8 RESULT  9 INFO

`ifndef DIM
  `define DIM 2
`endif
`ifndef KLEN
  `define KLEN 8
`endif

module tb_mm_accel_gen;
    localparam DIM  = `DIM;
    localparam KLEN = `KLEN;          // must be a multiple of 4 (packed pushes)

    localparam ACCEL_BASE = 32'h0000_5000;

    reg clk = 1'b0, rst = 1'b1;
    always #5 clk = ~clk;

    // CPU-side handshake into the AXI-Lite bridge, same as tb_mm_accel.v
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

    mm_accel ACCEL (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(m_awaddr), .s_axi_awvalid(m_awvalid), .s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata), .s_axi_wstrb(m_wstrb), .s_axi_wvalid(m_wvalid), .s_axi_wready(m_wready),
        .s_axi_bresp(m_bresp), .s_axi_bvalid(m_bvalid), .s_axi_bready(m_bready),
        .s_axi_araddr(m_araddr), .s_axi_arvalid(m_arvalid), .s_axi_arready(m_arready),
        .s_axi_rdata(m_rdata), .s_axi_rresp(m_rresp), .s_axi_rvalid(m_rvalid), .s_axi_rready(m_rready)
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

    // Reference operands and result. A is DIM x KLEN, B is KLEN x DIM.
    reg signed [7:0]  a [0:DIM-1][0:KLEN-1];
    reg signed [7:0]  b [0:DIM-1][0:KLEN-1];   // b[j][k] is column j
    reg signed [31:0] cref [0:DIM-1][0:DIM-1];

    integer i, j, k, g, errors;
    reg [31:0] rd, packed_w;
    integer acc;

    initial begin
        errors = 0;

        // deterministic pseudo-random operands, small enough to eyeball
        for (i = 0; i < DIM; i = i + 1)
            for (k = 0; k < KLEN; k = k + 1) begin
                a[i][k] = $signed((i * 7 + k * 3) % 11) - 5;
                b[i][k] = $signed((i * 5 + k * 2) % 9)  - 4;
            end

        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                acc = 0;
                for (k = 0; k < KLEN; k = k + 1) acc = acc + a[i][k] * b[j][k];
                cref[i][j] = acc;
            end

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // geometry check: the design carries its own DIM, so a mismatch between
        // -DDIM and MM_DIM is caught here instead of silently failing later
        do_read(wr(9), rd);
        if (rd[7:0] !== DIM[7:0]) begin
            $display("FATAL: built for DIM=%0d but design reports DIM=%0d", DIM, rd[7:0]);
            $finish;
        end
        $display("INFO: dim=%0d maxK=%0d, running K=%0d", rd[7:0], rd[15:8], KLEN);

        do_write(wr(2), KLEN);

        // push A: one lane at a time, 4 packed INT8 per write, LOAD_K auto-increments
        for (i = 0; i < DIM; i = i + 1) begin
            do_write(wr(4), i);                       // select lane, resets LOAD_K
            for (g = 0; g < KLEN / 4; g = g + 1) begin
                packed_w = { a[i][g*4+3][7:0], a[i][g*4+2][7:0],
                             a[i][g*4+1][7:0], a[i][g*4+0][7:0] };
                do_write(wr(5), packed_w);
            end
        end

        // push B the same way
        for (j = 0; j < DIM; j = j + 1) begin
            do_write(wr(4), j);
            for (g = 0; g < KLEN / 4; g = g + 1) begin
                packed_w = { b[j][g*4+3][7:0], b[j][g*4+2][7:0],
                             b[j][g*4+1][7:0], b[j][g*4+0][7:0] };
                do_write(wr(6), packed_w);
            end
        end

        do_write(wr(0), 32'h1);                       // START

        rd = 0;
        while (rd[1] !== 1'b1) do_read(wr(1), rd);    // poll DONE

        for (i = 0; i < DIM; i = i + 1)
            for (j = 0; j < DIM; j = j + 1) begin
                do_write(wr(7), i * DIM + j);
                do_read(wr(8), rd);
                if ($signed(rd) !== cref[i][j]) begin
                    $display("FAIL: C[%0d][%0d] = %0d, expected %0d",
                             i, j, $signed(rd), cref[i][j]);
                    errors = errors + 1;
                end
            end

        if (errors == 0)
            $display("=== DIM=%0d K=%0d: ALL %0d RESULTS CORRECT ===", DIM, KLEN, DIM*DIM);
        else
            $display("=== DIM=%0d K=%0d: %0d of %0d WRONG ===", DIM, KLEN, errors, DIM*DIM);
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
