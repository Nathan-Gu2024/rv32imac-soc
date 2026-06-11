module control_logic (
    input  wire [31:0] inst,
    input  wire br_eq, br_lt,
    output wire pc_sel, reg_wen, br_un, a_sel, b_sel, mem_rw,
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
        .br_un(br_un),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .mem_rw(mem_rw),
        .wb_sel(wb_sel),
        .imm_sel(imm_sel),
        .alu_sel(alu_sel)
    );

    wire is_beq = (rom_address == 6'd26);
    wire is_bne = (rom_address == 6'd27);
    wire is_blt = (rom_address == 6'd28);
    wire is_bge = (rom_address == 6'd29);
    wire is_bltu = (rom_address == 6'd30);
    wire is_bgeu = (rom_address == 6'd31);
    wire is_jal = (rom_address == 6'd34);
    wire is_jalr = (rom_address == 6'd35);

    assign pc_sel = (br_eq  &  is_beq)|
                    (~br_eq &  is_bne) |
                    (br_lt  & (is_blt | is_bltu)) |
                    (~br_lt & (is_bge | is_bgeu)) |
                    is_jal | is_jalr;

endmodule

module rom (
    input wire [5:0] rom_address,
    output reg reg_wen, br_un, a_sel, b_sel, mem_rw,
    output reg [1:0] wb_sel,
    output reg [2:0] imm_sel,
    output reg [3:0] alu_sel
);

    reg [15:0] mem [0:35];

    initial begin
        // {RegWEn, ImmSel[2:0], BrUn, ASel, BSel, ALUSel[3:0], MemRW, WBSel[1:0]}
        mem[0]  = 16'h1001; // add
        mem[1]  = 16'h1401; // mul
        mem[2]  = 16'h1601; // sub
        mem[3]  = 16'h1081; // sll
        mem[4]  = 16'h1481; // mulh
        mem[5]  = 16'h1581; // mulhu
        mem[6]  = 16'h1101; // slt
        mem[7]  = 16'h1201; // xor
        mem[8]  = 16'h1281; // srl
        mem[9]  = 16'h1681; // sra
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
        br_un = rom_out[4];
        a_sel = rom_out[5];
        b_sel = rom_out[6];
        alu_sel = rom_out[10:7];
        mem_rw = rom_out[11];
        wb_sel = rom_out[13:12];
    end

endmodule


module rom_decoder (
    input  wire [31:0] inst,
    output reg  [5:0]  rom_address
);

    wire [4:0] opcode = inst[6:2];
    wire [2:0] funct3 = inst[14:12];
    wire f7_bit5 = inst[30];
    wire f7_bit0 = inst[25];

    always @(*) begin
        case ({opcode, funct3, f7_bit5, f7_bit0})
            9'b01100_000_00: rom_address = 6'd0; // add
            9'b01100_000_01: rom_address = 6'd1; // mul
            9'b01100_000_10: rom_address = 6'd2; // sub
            9'b01100_001_00: rom_address = 6'd3; // sll
            9'b01100_001_01: rom_address = 6'd4; // mulh
            9'b01100_011_01: rom_address = 6'd5; // mulhu
            9'b01100_010_00: rom_address = 6'd6; // slt
            9'b01100_100_00: rom_address = 6'd7; // xor
            9'b01100_101_00: rom_address = 6'd8; // srl
            9'b01100_101_10: rom_address = 6'd9; // sra
            9'b01100_110_00: rom_address = 6'd10; // or
            9'b01100_111_00: rom_address = 6'd11; // and
            9'b00000_000_00: rom_address = 6'd12; // lb
            9'b00000_001_00: rom_address = 6'd13; // lh
            9'b00000_010_00: rom_address = 6'd14; // lw
            9'b00100_000_00: rom_address = 6'd15; // addi
            9'b00100_001_00: rom_address = 6'd16; // slli
            9'b00100_010_00: rom_address = 6'd17; // slti
            9'b00100_100_00: rom_address = 6'd18; // xori
            9'b00100_101_00: rom_address = 6'd19; // srli
            9'b00100_101_10: rom_address = 6'd20; // srai
            9'b00100_110_00: rom_address = 6'd21; // ori
            9'b00100_111_00: rom_address = 6'd22; // andi
            9'b01000_000_00: rom_address = 6'd23; // sb
            9'b01000_001_00: rom_address = 6'd24; // sh
            9'b01000_010_00: rom_address = 6'd25; // sw
            9'b11000_000_00: rom_address = 6'd26; // beq
            9'b11000_001_00: rom_address = 6'd27; // bne
            9'b11000_100_00: rom_address = 6'd28; // blt
            9'b11000_101_00: rom_address = 6'd29; // bge
            9'b11000_110_00: rom_address = 6'd30; // bltu
            9'b11000_111_00: rom_address = 6'd31; // bgeu
            9'b00101_000_00: rom_address = 6'd32; // auipc
            9'b01101_000_00: rom_address = 6'd33; // lui
            9'b11011_000_00: rom_address = 6'd34; // jal
            9'b11001_000_00: rom_address = 6'd35; // jalr
            default: rom_address = 6'd0;
        endcase
    end

endmodule

