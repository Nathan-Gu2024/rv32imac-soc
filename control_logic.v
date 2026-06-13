module control_logic (
    input wire [31:0] inst,
    output wire reg_wen, a_sel, b_sel, mem_rw,
    output wire [1:0] wb_sel,
    output wire [2:0] imm_sel,
    output wire [3:0] alu_sel
);

    wire [5:0] rom_address;

    rom_decoder decoder (
        .inst(inst),
        .rom_address(rom_address)
    );

    rom rom_inst (
        .rom_address(rom_address),
        .reg_wen(reg_wen),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .mem_rw(mem_rw),
        .wb_sel(wb_sel),
        .imm_sel(imm_sel),
        .alu_sel(alu_sel)
    );

endmodule

module rom (
    input wire [5:0] rom_address,
    output reg reg_wen, a_sel, b_sel, mem_rw,
    output reg [1:0] wb_sel,
    output reg [2:0] imm_sel,
    output reg [3:0] alu_sel
);
    reg [15:0] mem [0:35];

    initial begin
        // {RegWEn, ImmSel[2:0], BrUn, ASel, BSel, ALUSel[3:0], MemRW, WBSel[1:0]}
        mem[0] = 16'h1001; // add
        mem[1] = 16'h1401; // mul
        mem[2] = 16'h1601; // sub
        mem[3] = 16'h1081; // sll
        mem[4] = 16'h1481; // mulh
        mem[5] = 16'h1581; // mulhu
        mem[6] = 16'h1101; // slt
        mem[7] = 16'h1201; // xor
        mem[8] = 16'h1281; // srl
        mem[9] = 16'h1681; // sra
        mem[10] = 16'h1301; // or
        mem[11] = 16'h1381; // and
        mem[12] = 16'h0041; // lb
        mem[13] = 16'h0041; // lh
        mem[14] = 16'h0041; // lw
        mem[15] = 16'h1041; // addi
        mem[16] = 16'h10C1; // slli
        mem[17] = 16'h1141; // slti
        mem[18] = 16'h1241; // xori
        mem[19] = 16'h12C1; // srli
        mem[20] = 16'h16C1; // srai
        mem[21] = 16'h1341; // ori
        mem[22] = 16'h13C1; // andi
        mem[23] = 16'h0842; // sb
        mem[24] = 16'h0842; // sh
        mem[25] = 16'h0842; // sw
        mem[26] = 16'h0064; // beq
        mem[27] = 16'h0064; // bne
        mem[28] = 16'h0064; // blt
        mem[29] = 16'h0064; // bge
        mem[30] = 16'h0074; // bltu
        mem[31] = 16'h0074; // bgeu
        mem[32] = 16'h1067; // auipc
        mem[33] = 16'h17C7; // lui
        mem[34] = 16'h2069; // jal
        mem[35] = 16'h2041; // jalr
    end

    wire [15:0] rom_out = mem[rom_address];
    always @(*) begin
        reg_wen = rom_out[0];
        imm_sel = rom_out[3:1];
        a_sel = rom_out[5];
        b_sel = rom_out[6];
        alu_sel = rom_out[10:7];
        mem_rw = rom_out[11];
        wb_sel = rom_out[13:12];
    end

endmodule


module rom_decoder (
    input wire [31:0] inst,
    output reg [5:0] rom_address
);
    wire [4:0] opcode = inst[6:2];
    wire [2:0] funct3 = inst[14:12];
    wire f7_bit5 = inst[30];
    wire f7_bit0 = inst[25];

    always @(*) begin
        casex ({opcode, funct3, f7_bit5, f7_bit0})
            // R-Types (All bits matter)
            10'b01100_000_00: rom_address = 6'd0; // add
            10'b01100_000_01: rom_address = 6'd1; // mul
            10'b01100_000_10: rom_address = 6'd2; // sub
            10'b01100_001_00: rom_address = 6'd3; // sll
            10'b01100_001_01: rom_address = 6'd4; // mulh
            10'b01100_011_01: rom_address = 6'd5; // mulhu
            10'b01100_010_00: rom_address = 6'd6; // slt
            10'b01100_100_00: rom_address = 6'd7; // xor
            10'b01100_101_00: rom_address = 6'd8; // srl
            10'b01100_101_10: rom_address = 6'd9; // sra
            10'b01100_110_00: rom_address = 6'd10; // or
            10'b01100_111_00: rom_address = 6'd11; // and
            
            // Memory Loads (I-Type: f7 bits are part of immediate)
            10'b00000_000_?_?: rom_address = 6'd12; // lb
            10'b00000_001_?_?: rom_address = 6'd13; // lh
            10'b00000_010_?_?: rom_address = 6'd14; // lw
            
            // Memory Stores (S-Type: f7 bits are part of immediate)
            10'b01000_000_?_?: rom_address = 6'd23; // sb
            10'b01000_001_?_?: rom_address = 6'd24; // sh
            10'b01000_010_?_?: rom_address = 6'd25; // sw
            
            // I-Type ALU (f7 bits are part of immediate)
            10'b00100_000_?_?: rom_address = 6'd15; // addi
            10'b00100_010_?_?: rom_address = 6'd17; // slti
            10'b00100_100_?_?: rom_address = 6'd18; // xori
            10'b00100_110_?_?: rom_address = 6'd21; // ori
            10'b00100_111_?_?: rom_address = 6'd22; // andi
            
            // I-Type Shifts (f7_bit5 is a modifier, f7_bit0 mask to be safe)
            10'b00100_001_0_?: rom_address = 6'd16; // slli
            10'b00100_101_0_?: rom_address = 6'd19; // srli
            10'b00100_101_1_?: rom_address = 6'd20; // srai
            
            // B-Type Branches (f7 bits are part of immediate)
            10'b11000_000_?_?: rom_address = 6'd26; // beq
            10'b11000_001_?_?: rom_address = 6'd27; // bne
            10'b11000_100_?_?: rom_address = 6'd28; // blt
            10'b11000_101_?_?: rom_address = 6'd29; // bge
            10'b11000_110_?_?: rom_address = 6'd30; // bltu
            10'b11000_111_?_?: rom_address = 6'd31; // bgeu
            
            // U-Type and J-Type (funct3 and f7 bits are all part of immediate)
            10'b00101_???_?_?: rom_address = 6'd32; // auipc
            10'b01101_???_?_?: rom_address = 6'd33; // lui
            10'b11011_???_?_?: rom_address = 6'd34; // jal
            
            // JALR (I-Type, funct3 is 000)
            10'b11001_000_?_?: rom_address = 6'd35; // jalr
            
            default: rom_address = 6'd0;
        endcase
    end
endmodule