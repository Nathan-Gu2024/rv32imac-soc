`ifndef _PARTIAL_STORE_V_
`define _PARTIAL_STORE_V_

module partial_store (
    input wire [31:0] inst, mem_address, data_from_reg,
    input wire mem_rw,
    output wire [3:0] mem_write_mask, 
    output reg [31:0] data_to_mem
);

    wire [31:0] sb, sh;
    wire [3:0] sh_mask;
    reg [3:0] sb_mask, temp;
    assign sb = {4{data_from_reg[7:0]}};
    assign sh = {{2{data_from_reg[15:0]}}};
    assign sh_mask = mem_address[1] ? 4'b1100 : 4'b0011;
    always @(*) begin
        case (mem_address[1:0])
            2'b00: sb_mask = 4'b0001;
            2'b01: sb_mask = 4'b0010;
            2'b10: sb_mask = 4'b0100;
            2'b11: sb_mask = 4'b1000;
        endcase
    end

    always @(*) begin
        case (inst[13:12]) 
            2'b00:
                begin
                    data_to_mem = sb;
                    temp = sb_mask;
                end 
            2'b01:
                begin
                    data_to_mem = sh;
                    temp = sh_mask;
                end 
            2'b10:
                begin
                    data_to_mem = data_from_reg;
                    temp = 4'b1111;
                end
            default: 
                begin
                    data_to_mem = data_from_reg;
                    temp = 4'b1111;
                end 
        endcase
    end 

    assign mem_write_mask = temp & {{4{mem_rw}}};

endmodule

`endif // _PARTIAL_STORE_V_
