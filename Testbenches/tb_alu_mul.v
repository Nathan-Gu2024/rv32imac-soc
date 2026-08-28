`timescale 1ns/1ps
`include "../src/alu.v"

// Directed test for the four RV32M multiply forms, which all share one
// 33x33 signed multiplier in alu.v (operands sign- or zero-extended per
// instruction). MULHSU in particular is exercised by no other test in this
// repo - CoreMark never emits it - so without this it would be unverified.
//
// The load-bearing vectors are the ones where MULH, MULHSU and MULHU each
// produce a DIFFERENT answer for identical operands; those are what prove
// the signedness selection is actually wired per-instruction rather than
// one form accidentally shadowing another.
module tb_alu_mul;
    reg [31:0] a, b;
    reg [4:0] sel;
    wire [31:0] res;

    integer errors = 0;

    localparam SEL_MUL    = 5'd8;
    localparam SEL_MULH   = 5'd9;
    localparam SEL_MULHSU = 5'd10;
    localparam SEL_MULHU  = 5'd11;

    alu DUT (.a(a), .b(b), .alu_sel(sel), .alu_res(res));

    task chk;
        input [31:0] ta, tb;
        input [4:0]  tsel;
        input [31:0] expected;
        input [127:0] name;
        begin
            a = ta; b = tb; sel = tsel;
            #1;
            if (res !== expected) begin
                $display("FAIL %0s a=%h b=%h -> got %h, want %h",
                         name, ta, tb, res, expected);
                errors = errors + 1;
            end else begin
                $display("PASS %0s a=%h b=%h -> %h", name, ta, tb, res);
            end
        end
    endtask

    initial begin
        // ---- MUL: low 32 bits, signedness irrelevant ----
        chk(32'd3,          32'd4,          SEL_MUL, 32'd12,       "mul");
        chk(32'hFFFFFFFD,   32'd4,          SEL_MUL, 32'hFFFFFFF4, "mul-neg");   // -3 * 4 = -12

        // ---- The discriminating case: a = b = 0xFFFFFFFF ----
        // as signed   : -1
        // as unsigned : 4294967295
        //   MULH   (-1 * -1)                 = 1                  -> hi 0x00000000
        //   MULHSU (-1 * 4294967295)         = -4294967295        -> hi 0xFFFFFFFF
        //   MULHU  (4294967295 * 4294967295) = 18446744065119617025 -> hi 0xFFFFFFFE
        chk(32'hFFFFFFFF, 32'hFFFFFFFF, SEL_MULH,   32'h00000000, "mulh-m1m1");
        chk(32'hFFFFFFFF, 32'hFFFFFFFF, SEL_MULHSU, 32'hFFFFFFFF, "mhsu-m1m1");
        chk(32'hFFFFFFFF, 32'hFFFFFFFF, SEL_MULHU,  32'hFFFFFFFE, "mulhu-m1m1");

        // ---- b's sign matters: a positive, b = 0x80000000 ----
        //   MULH   (2 * -2147483648) = -4294967296 -> hi 0xFFFFFFFF
        //   MULHSU (2 *  2147483648) =  4294967296 -> hi 0x00000001  (b unsigned)
        //   MULHU  (2 *  2147483648) =  4294967296 -> hi 0x00000001
        chk(32'd2, 32'h80000000, SEL_MULH,   32'hFFFFFFFF, "mulh-bneg");
        chk(32'd2, 32'h80000000, SEL_MULHSU, 32'h00000001, "mhsu-bneg");
        chk(32'd2, 32'h80000000, SEL_MULHU,  32'h00000001, "mulhu-bneg");

        // ---- a's sign matters: a = 0x80000000, b positive ----
        //   MULH   (-2147483648 * 2) = -4294967296 -> hi 0xFFFFFFFF
        //   MULHSU (-2147483648 * 2) = -4294967296 -> hi 0xFFFFFFFF  (a still signed)
        //   MULHU  ( 2147483648 * 2) =  4294967296 -> hi 0x00000001
        chk(32'h80000000, 32'd2, SEL_MULH,   32'hFFFFFFFF, "mulh-aneg");
        chk(32'h80000000, 32'd2, SEL_MULHSU, 32'hFFFFFFFF, "mhsu-aneg");
        chk(32'h80000000, 32'd2, SEL_MULHU,  32'h00000001, "mulhu-aneg");

        // ---- plain positive overflow into the high word ----
        chk(32'h40000000, 32'd4, SEL_MULH,   32'h00000001, "mulh-pos");
        chk(32'h40000000, 32'd4, SEL_MULHSU, 32'h00000001, "mhsu-pos");
        chk(32'h40000000, 32'd4, SEL_MULHU,  32'h00000001, "mulhu-pos");

        // ---- zero ----
        chk(32'h80000000, 32'd0, SEL_MULHSU, 32'h00000000, "mhsu-zero");

        if (errors == 0) $display("\nALL MULTIPLY TESTS PASSED");
        else             $display("\n%0d FAILURES", errors);
        $finish;
    end
endmodule
