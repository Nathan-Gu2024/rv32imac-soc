module immgen (
    input wire [31:0]inst,    
    input wire [2:0] imm_sel, 
    output reg [31:0] imm  
);

    always @(*) begin
        case (imm_sel)
            3'b000: // i type
                imm = {{21{inst[31]}}, inst[30:20]};
            3'b001: // s
                imm = {{21{inst[31]}}, inst[30:25], inst[11:7]};               
            3'b010: // b
                imm = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};              
            3'b011: // u
                imm = {inst[31:12], 12'b0};             
            3'b100: // j
                imm = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};            
            default: 
                imm = 32'b0;
        endcase
    end
endmodule
