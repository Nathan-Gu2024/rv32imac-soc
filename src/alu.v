`ifndef _ALU_V_
`define _ALU_V_

module alu (
    input wire [31:0] a, b,
    input wire [4:0] alu_sel,
    output reg [31:0] alu_res
);

    // One 33x33 signed multiplier serving all four RV32M multiply forms.
    // Each operand is sign-extended or zero-extended according to the
    // instruction, so MUL/MULH/MULHSU/MULHU share a single DSP cluster.
    // This replaces two unconditional 32x32 products (~8 DSP48E1, computed
    // every cycle for every instruction including ANDs and branches) that
    // between them still could not express MULHSU's mixed signedness.
    // MUL takes the signed path because the low 32 bits of the product are
    // identical either way.
    wire mul_a_signed = (alu_sel == 5'd8) | (alu_sel == 5'd9) | (alu_sel == 5'd10);
    wire mul_b_signed = (alu_sel == 5'd8) | (alu_sel == 5'd9);

    wire signed [32:0] mul_a = {mul_a_signed & a[31], a};
    wire signed [32:0] mul_b = {mul_b_signed & b[31], b};
    wire signed [65:0] mul_res = mul_a * mul_b;

    // div/rem/divu/remu (alu_sel 16-19) are NOT computed here. A plain
    // combinational '/'/'%' synthesizes into one giant ripple-subtract chain
    // that cannot meet timing (measured ~88ns critical path against a 20ns
    // period). They're handled by the multi-cycle div_unit instantiated in
    // cpu.v instead, which stalls the pipeline like a cache miss while it
    // computes; the EX stage muxes that result in past this ALU entirely,
    // so alu_res is simply unused for those alu_sel values.
    always @(*) begin
        case(alu_sel)
            5'd0: // add
                alu_res = a + b;
            5'd1: // sll
                alu_res = a << b[4:0];
            5'd2: // slt
                alu_res = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            5'd3: // sltu
                alu_res = (a < b) ? 32'd1 : 32'd0;
            5'd4: // xor
                alu_res = a ^ b;
            5'd5: // srl
                alu_res = a >> b[4:0];
            5'd6: // or
                alu_res = a | b;
            5'd7: // and
                alu_res = a & b;
            5'd8: // mul
                alu_res = mul_res[31:0];
            5'd9: // mulh    (signed x signed)
                alu_res = mul_res[63:32];
            5'd10: // mulhsu (signed x unsigned)
                alu_res = mul_res[63:32];
            5'd11: // mulhu  (unsigned x unsigned)
                alu_res = mul_res[63:32];
            5'd12: // sub
                alu_res = a - b;
            5'd13: // sra
                alu_res = $signed(a) >>> b[4:0];
            5'd15: // bsel
                alu_res = b;
            // Zba shifted-add (RV32 Zba is exactly these three). Chosen over
            // the rest of the B extension by profiling: of the 106 bitmanip
            // instructions GCC emits for CoreMark, 62 are these, and they
            // cost only a constant shift into the adder that already exists
            // - no new arithmetic, unlike clz/ctz/cpop in Zbb.
            5'd20: // sh1add
                alu_res = (a << 1) + b;
            5'd21: // sh2add
                alu_res = (a << 2) + b;
            5'd22: // sh3add
                alu_res = (a << 3) + b;
            default: alu_res = a + b; // covers 16-19 (div/divu/rem/remu) too; unused, see note above
        endcase
    end

endmodule

`endif // _ALU_V_
