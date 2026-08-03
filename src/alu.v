module alu (
    input wire [31:0] a, b,
    input wire [4:0] alu_sel,
    output reg [31:0] alu_res
);

    wire [63:0] signed_mul_res;
    wire [63:0] unsigned_mul_res;

    assign signed_mul_res = $signed(a) * $signed(b);
    assign unsigned_mul_res = a * b;

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
                alu_res = signed_mul_res[31:0];
            5'd9: // mulh
                alu_res = signed_mul_res[63:32];
            5'd11: // mulhu
                alu_res = unsigned_mul_res[63:32];
            5'd12: // sub
                alu_res = a - b;
            5'd13: // sra
                alu_res = $signed(a) >>> b[4:0];
            5'd15: // bsel
                alu_res = b;
            default: alu_res = a + b; // covers 16-19 (div/divu/rem/remu) too; unused, see note above
        endcase
    end

endmodule

