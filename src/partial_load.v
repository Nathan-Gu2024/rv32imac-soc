`ifndef _PARTIAL_LOAD_V_
`define _PARTIAL_LOAD_V_

module partial_load (
    input wire [31:0] inst, mem_address, data_from_mem,
    output reg [31:0] data_to_reg
);

    wire [2:0] select = inst[14:12];
    reg [7:0] byte_sel;

    always @(*) begin
        case (mem_address[1:0])
            2'b00:
                byte_sel = data_from_mem[7:0];
            2'b01:
                byte_sel = data_from_mem[15:8];
            2'b10:
                byte_sel = data_from_mem[23:16];
            2'b11:
                byte_sel = data_from_mem[31:24];
        endcase
    end

    wire [31:0] lb = {{24{byte_sel[7]}}, byte_sel};

    wire [31:0] lbu = {24'b0, byte_sel};

    wire [31:0] lh = mem_address[1] ? {{16{data_from_mem[31]}}, data_from_mem[31:16]} 
                    : {{16{data_from_mem[15]}}, data_from_mem[15:0]};

    wire [31:0] lhu = mem_address[1] ? {16'b0, data_from_mem[31:16]} 
                    : {16'b0, data_from_mem[15:0]};

    always @(*) begin
        case (select)
            3'd0:
                data_to_reg = lb;
            3'd1:
                data_to_reg = lh;
            3'd2:
                data_to_reg = data_from_mem;
            3'd4:
                data_to_reg = lbu;
            3'd5:
                data_to_reg = lhu;
            default: data_to_reg = 32'b0;
        endcase
    end

endmodule

`endif // _PARTIAL_LOAD_V_
