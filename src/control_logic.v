module control_logic (
    input wire [31:0] inst,
    output wire reg_wen, a_sel, b_sel, mem_rw,
    output wire [1:0] wb_sel,
    output wire [2:0] imm_sel,
    output wire [4:0] alu_sel,
    output wire out_is_lr, out_is_sc, out_is_amo,
    output wire [4:0] out_atomic_op,
    output wire csr_wen, 

    output wire [1:0] csr_op, 
    output wire csr_use_imm, 
    output wire [4:0] csr_uimm
    
);

    wire [5:0] rom_address;
    wire is_atomic_inst = (inst[6:0] == 7'b0101111);
    wire [4:0] atomic_funct5 = inst[31:27];
    wire is_lr = is_atomic_inst && (atomic_funct5 == 5'b00010);
    wire is_sc = is_atomic_inst && (atomic_funct5 == 5'b00011);
    wire is_amo = is_atomic_inst && !is_lr && !is_sc;
    
    wire is_system_inst = (inst[6:0] == 7'b1110011);

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
    
    assign out_is_lr = is_lr;
    assign out_is_sc = is_sc;
    assign out_is_amo = is_amo;
    assign out_atomic_op = atomic_funct5;
    wire [2:0] csr_funct3 = inst[14:12];
    // funct3 000/100 are non-CSR system instructions (ecall/ebreak/mret/etc)
    assign csr_wen = is_system_inst && (csr_funct3 != 3'b000) && (csr_funct3 != 3'b100);
    // funct3[1:0] perfectly matches our operations: 01 = RW, 10 = RS, 11 = RC
    assign csr_op = csr_funct3[1:0];
    // If funct3 bit 2 is high (101, 110, 111), it's an immediate variant
    assign csr_use_imm = is_system_inst && csr_funct3[2];
    assign csr_uimm = inst[19:15]; // The 5-bit unsigned immediate field
    
endmodule

module rom (
    input wire [5:0] rom_address,
    output reg reg_wen, a_sel, b_sel, mem_rw,
    output reg [1:0] wb_sel,
    output reg [2:0] imm_sel,
    output reg [4:0] alu_sel
);

    reg [15:0] rom_out;

    always @(*) begin
        case (rom_address)
            6'd0: rom_out = 16'h1001; // add
            6'd1: rom_out = 16'h1401; // mul
            6'd2: rom_out = 16'h1601; // sub
            6'd3: rom_out = 16'h1081; // sll
            6'd4: rom_out = 16'h1481; // mulh
            6'd5: rom_out = 16'h1581; // mulhu
            6'd6: rom_out = 16'h1101; // slt
            6'd7: rom_out = 16'h1201; // xor
            6'd8: rom_out = 16'h1281; // srl
            6'd9: rom_out = 16'h1681; // sra
            6'd10: rom_out = 16'h1301; // or
            6'd11: rom_out = 16'h1381; // and
            6'd12: rom_out = 16'h0041; // lb
            6'd13: rom_out = 16'h0041; // lh
            6'd14: rom_out = 16'h0041; // lw
            6'd15: rom_out = 16'h1041; // addi
            6'd16: rom_out = 16'h10C1; // slli
            6'd17: rom_out = 16'h1141; // slti
            6'd18: rom_out = 16'h1241; // xori
            6'd19: rom_out = 16'h12C1; // srli
            6'd20: rom_out = 16'h16C1; // srai
            6'd21: rom_out = 16'h1341; // ori
            6'd22: rom_out = 16'h13C1; // andi
            6'd23: rom_out = 16'h0842; // sb
            6'd24: rom_out = 16'h0842; // sh
            6'd25: rom_out = 16'h0842; // sw
            6'd26: rom_out = 16'h0064; // beq
            6'd27: rom_out = 16'h0064; // bne
            6'd28: rom_out = 16'h0064; // blt
            6'd29: rom_out = 16'h0064; // bge
            6'd30: rom_out = 16'h0074; // bltu
            6'd31: rom_out = 16'h0074; // bgeu
            6'd32: rom_out = 16'h1067; // auipc
            6'd33: rom_out = 16'h17C7; // lui
            6'd34: rom_out = 16'h2069; // jal
            6'd35: rom_out = 16'h2041; // jalr
            6'd36: rom_out = 16'h004F; // lr.w
            6'd37: rom_out = 16'h184F; // sc.w 
            6'd38: rom_out = 16'h8030; // csrrw
            // bit14 is the new alu_sel[4] bit (was spare/0 in every entry above,
            // so none of the existing 39 entries change meaning).
            6'd39: rom_out = 16'h5001; // div  (alu_sel=5'd16)
            6'd40: rom_out = 16'h5081; // divu (alu_sel=5'd17)
            6'd41: rom_out = 16'h5101; // rem  (alu_sel=5'd18)
            6'd42: rom_out = 16'h5181; // remu (alu_sel=5'd19)
            6'd43: rom_out = 16'h1181; // sltu  (alu_sel=5'd3, same shape as slt but b_sel=reg not imm)
            6'd44: rom_out = 16'h11C1; // sltiu (alu_sel=5'd3, same shape as slti but unsigned)
            default: rom_out = 16'h0000;
        endcase
    end

    always @(*) begin
        reg_wen = rom_out[0];  
        imm_sel = rom_out[3:1];
        a_sel = rom_out[5];
        b_sel = rom_out[6];
        alu_sel = {rom_out[14], rom_out[10:7]};
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
    wire [4:0] funct5 = inst[31:27];

    always @(*) begin
        // Intercept Atomics (Opcode: 01011)
        if (opcode == 5'b01011) begin
            if (funct5 == 5'b00010)
                rom_address = 6'd36; // lr.w
            else if (funct5 == 5'b00011)
                rom_address = 6'd37; // sc.w
            else
                rom_address = 6'd0; // Fallback for AMO (add) until implemented / needed
        end else if (inst[6:2] == 5'b11100) begin
            rom_address = 6'd0; 
        end else begin 
            casex ({opcode, funct3, f7_bit5, f7_bit0})
                // R-Types (All bits matter)
                10'b01100_000_00: rom_address = 6'd0; // add
                10'b01100_000_01: rom_address = 6'd1; // mul
                10'b01100_000_10: rom_address = 6'd2; // sub
                10'b01100_001_00: rom_address = 6'd3; // sll
                10'b01100_001_01: rom_address = 6'd4; // mulh
                10'b01100_011_01: rom_address = 6'd5; // mulhu
                10'b01100_010_00: rom_address = 6'd6; // slt
                10'b01100_011_00: rom_address = 6'd43; // sltu
                10'b01100_100_00: rom_address = 6'd7; // xor
                10'b01100_101_00: rom_address = 6'd8; // srl
                10'b01100_101_10: rom_address = 6'd9; // sra
                10'b01100_110_00: rom_address = 6'd10; // or
                10'b01100_111_00: rom_address = 6'd11; // and
                10'b01100_100_01: rom_address = 6'd39; // div
                10'b01100_101_01: rom_address = 6'd40; // divu
                10'b01100_110_01: rom_address = 6'd41; // rem
                10'b01100_111_01: rom_address = 6'd42; // remu

                // Memory Loads (I-Type: f7 bits are part of immediate)
                10'b00000_000_?_?: rom_address = 6'd12; // lb
                10'b00000_001_?_?: rom_address = 6'd13; // lh
                10'b00000_010_?_?: rom_address = 6'd14; // lw
                10'b00000_100_?_?: rom_address = 6'd12; // lbu (same control word as lb; partial_load.v handles sign/zero-extend from funct3)
                10'b00000_101_?_?: rom_address = 6'd13; // lhu (same control word as lh; partial_load.v handles sign/zero-extend from funct3)

                // Memory Stores (S-Type: f7 bits are part of immediate)
                10'b01000_000_?_?: rom_address = 6'd23; // sb
                10'b01000_001_?_?: rom_address = 6'd24; // sh
                10'b01000_010_?_?: rom_address = 6'd25; // sw
                
                // I-Type ALU (f7 bits are part of immediate)
                10'b00100_000_?_?: rom_address = 6'd15; // addi
                10'b00100_010_?_?: rom_address = 6'd17; // slti
                10'b00100_011_?_?: rom_address = 6'd44; // sltiu
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
    end
endmodule