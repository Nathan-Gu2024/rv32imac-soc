`timescale 1ns/1ps
`include "../src/axi_lite_bridge.v"

// This bench is hardwired to the 2x2 register map of src/mm_accel.v: fixed
// push words 4..7 and fixed result words 8..11.
//
// The Chisel generator (chisel/src/MmAccel.scala) reproduced this map exactly
// and passed these checks cycle-for-cycle - 8/8, finishing at the same
// 2345000 ps - before its map was deliberately changed to the INDEXED form so
// the array could scale past DIM=7. See Testbenches/tb_mm_accel_gen.v, which
// drives that map and validates every size the generator emits.
// Points at the LEGACY core, not src/mm_accel.v. The SoC now instantiates the
// Chisel-generated DIM=8 accelerator, whose register map is indexed rather
// than fixed-word and which takes no parameters (dim/maxK are baked in at
// elaboration), so this bench cannot drive it - it would not even elaborate,
// because of the #(.MAX_K(16)) below.
//
// Kept pointed at the 2x2 original because it is the reference the Chisel port
// was validated against: 8/8 checks, cycle-identical at 2345000 ps. Retiring it
// would throw away the evidence that the port started out bit-exact.
// Testbenches/tb_mm_accel_gen.v is the equivalent for the generated core.
`include "../src/mm_accel_v2x2_legacy.v"

// Standalone bridge+accelerator test, bypassing cpu.v entirely: drives the
// same d_req/d_we/d_addr/d_wdata handshake cpu.v would, straight into the
// AXI4-Lite bridge, to isolate AXI protocol correctness and the systolic
// skew/timing math from any CPU pipeline concerns.
//
// Computes C = A*B for A=[[3,-2],[5,7]], B=[[4,1],[-3,6]] (signed INT8
// operands, chosen to exercise negative values):
//   C00 = 3*4 + (-2)*(-3)  = 18
//   C01 = 3*1 + (-2)*6     = -9
//   C10 = 5*4 + 7*(-3)     = -1
//   C11 = 5*1 + 7*6        = 47
module tb_mm_accel;
    reg clk, rst;
    reg d_req, d_we;
    reg [31:0] d_addr, d_wdata;
    wire [31:0] d_rdata;
    wire d_ready;

    wire [31:0] m_awaddr, m_wdata, m_araddr, m_rdata;
    wire [3:0] m_wstrb;
    wire m_awvalid, m_awready, m_wvalid, m_wready, m_bvalid, m_bready;
    wire [1:0] m_bresp, m_rresp;
    wire m_arvalid, m_arready, m_rvalid, m_rready;

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

    mm_accel_v2x2_legacy #(.MAX_K(16)) ACCEL (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(m_awaddr), .s_axi_awvalid(m_awvalid), .s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata), .s_axi_wstrb(m_wstrb), .s_axi_wvalid(m_wvalid), .s_axi_wready(m_wready),
        .s_axi_bresp(m_bresp), .s_axi_bvalid(m_bvalid), .s_axi_bready(m_bready),
        .s_axi_araddr(m_araddr), .s_axi_arvalid(m_arvalid), .s_axi_arready(m_arready),
        .s_axi_rdata(m_rdata), .s_axi_rresp(m_rresp), .s_axi_rvalid(m_rvalid), .s_axi_rready(m_rready)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer errors = 0;

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

    task check(input [255:0] name, input signed [31:0] actual, input signed [31:0] expected);
        begin
            if (actual === expected)
                $display("PASS: %0s = %0d", name, actual);
            else begin
                $display("FAIL: %0s = %0d, expected %0d", name, actual, expected);
                errors = errors + 1;
            end
        end
    endtask

    reg [31:0] status_val;
    reg [31:0] r00, r01, r10, r11;
    integer timeout;

    initial begin
        rst = 1; d_req = 0; d_we = 0; d_addr = 0; d_wdata = 0;
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        do_write(32'h08, 32'd2); // K_LEN=2

        do_write(32'h0C, 32'd0); // LOAD_IDX=0
        do_write(32'h10, 32'sd3);  // a_row0[0]=3
        do_write(32'h14, 32'sd5);  // a_row1[0]=5
        do_write(32'h18, 32'sd4);  // b_col0[0]=4
        do_write(32'h1C, 32'sd1);  // b_col1[0]=1

        do_write(32'h0C, 32'd1); // LOAD_IDX=1
        do_write(32'h10, -32'sd2); // a_row0[1]=-2
        do_write(32'h14, 32'sd7);  // a_row1[1]=7
        do_write(32'h18, -32'sd3); // b_col0[1]=-3
        do_write(32'h1C, 32'sd6);  // b_col1[1]=6

        do_write(32'h00, 32'd1); // START

        timeout = 0;
        status_val = 0;
        while (!status_val[1] && timeout < 1000) begin
            do_read(32'h04, status_val);
            timeout = timeout + 1;
        end
        if (timeout >= 1000) begin
            $display("FAIL: timed out waiting for DONE");
            errors = errors + 1;
        end

        do_read(32'h20, r00);
        do_read(32'h24, r01);
        do_read(32'h28, r10);
        do_read(32'h2C, r11);

        check("C00", r00, 18);
        check("C01", r01, -9);
        check("C10", r10, -1);
        check("C11", r11, 47);

        // Second consecutive run, different operands: A=[[1,2],[3,4]],
        // B=[[5,6],[7,8]] -> C=[[19,22],[43,50]]. Exercises the START-must-
        // clear-stale-DONE fix - if DONE didn't clear, the poll loop below
        // would return immediately with the FIRST run's stale results.
        do_write(32'h0C, 32'd0);
        do_write(32'h10, 32'sd1);
        do_write(32'h14, 32'sd3);
        do_write(32'h18, 32'sd5);
        do_write(32'h1C, 32'sd6);
        do_write(32'h0C, 32'd1);
        do_write(32'h10, 32'sd2);
        do_write(32'h14, 32'sd4);
        do_write(32'h18, 32'sd7);
        do_write(32'h1C, 32'sd8);
        do_write(32'h00, 32'd1); // START

        timeout = 0;
        status_val = 0;
        while (!status_val[1] && timeout < 1000) begin
            do_read(32'h04, status_val);
            timeout = timeout + 1;
        end
        if (timeout >= 1000) begin
            $display("FAIL: run 2 timed out waiting for DONE");
            errors = errors + 1;
        end

        do_read(32'h20, r00);
        do_read(32'h24, r01);
        do_read(32'h28, r10);
        do_read(32'h2C, r11);

        check("run2_C00", r00, 19);
        check("run2_C01", r01, 22);
        check("run2_C10", r10, 43);
        check("run2_C11", r11, 50);

        if (errors == 0)
            $display("=== ALL MATMUL CHECKS PASSED ===");
        else
            $display("=== %0d CHECK(S) FAILED ===", errors);

        $finish;
    end

    initial begin
        #20000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
