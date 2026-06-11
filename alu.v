module alu (
    input wire [31:0] a, b,
    input wire [3:0] alu_sel,
    output reg [31:0] alu_res
); 

    wire [63:0] signed_mul_res;
    wire [63:0] unsigned_mul_res;

    assign signed_mul_res = $signed(a) * $signed(b);
    assign unsigned_mul_res = a * b;


    always @(*) begin
        case(alu_sel)
            4'd0: // add
                alu_res = a + b;
            4'd1: // sll
                alu_res = a << b[4:0];
            4'd2: // slt
                alu_res = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            4'd3: // sltu
                alur_res = (a < b) ? 32'd1 : 32'd0;
            4'd4: // xor
                alu_res = a ^ b;
            4'd5: // srl
                alu_res = a >> b[4:0];
            4'd6: // or
                alu_res = a | b;
            4'd7: // and 
                alu_res = a & b;
            4'd8: // mul
                alu_res = signed_mul_res[31:0];
            4'd9: // mulh
                alu_res = signed_mul_res[63:32];
            4'd11: // mulhu
                alu_res = unsigned_mul_res[63:32];
            4'd12: // sub
                alu_res = a - b;
            4'd13: // sra
                alu_res = $signed(a) >>> b[4:0];
            4'd15: // bsel
                alu_res = b;
            default: alu_res = a + b;
        endcase
    end 

endmodule

