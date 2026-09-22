`ifndef _RVC_EXPANSION_V_
`define _RVC_EXPANSION_V_

module rvc_expand (
    input wire [15:0] inst_c,
    output reg [31:0] inst_expanded,
    output reg is_compressed
);
    wire [1:0] quadrant = inst_c[1:0];
    wire [2:0] funct3 = inst_c[15:13];

    // CL/CS/CB/CA "prime" registers: 3-bit field maps to x8..x15
    wire [4:0] rs1p = {2'b01, inst_c[9:7]};
    wire [4:0] rs2p = {2'b01, inst_c[4:2]};
    wire [4:0] rdp = {2'b01, inst_c[4:2]};

    // CI/CR full 5-bit register fields
    wire [4:0] rs1_full = inst_c[11:7];
    wire [4:0] rs2_full = inst_c[6:2];
    wire [4:0] rd_full = inst_c[11:7];

    always @(*) begin
        is_compressed = (quadrant != 2'b11);
        inst_expanded = 32'h00000013; // default: NOP (addi x0, x0, 0)
        case (quadrant)
            2'b00: 
                begin // Q0
                    case (funct3)
                        3'b000: 
                            begin
                                // C.ADDI4SPN -> addi rd', x2, nzuimm
                                // nzuimm[9:2] = {inst_c[10:7], inst_c[12:11], inst_c[5], inst_c[6]}
                                // I-type 12-bit imm (always positive): {2'b0, inst_c[10:7], inst_c[12:11], inst_c[5], inst_c[6], 2'b00}
                                inst_expanded = {
                                    2'b00, inst_c[10:7], inst_c[12:11], inst_c[5], inst_c[6], 2'b00,
                                    5'b00010, // rs1 = x2 (sp)
                                    3'b000,
                                    rdp,
                                    7'b0010011 // ADDI
                                };
                            end
                        3'b010: 
                            begin
                                // C.LW -> lw rd', offset(rs1')
                                // offset[6:2] = {inst_c[5], inst_c[12:10], inst_c[6]}, offset[1:0]=00
                                // I-type 12-bit imm = {5'b0, inst_c[5], inst_c[12:10], inst_c[6], 2'b00}
                                inst_expanded = {
                                    5'b00000, inst_c[5], inst_c[12:10], inst_c[6], 2'b00,
                                    rs1p,
                                    3'b010,
                                    rdp,
                                    7'b0000011 // LOAD
                                };
                            end
                        3'b110: 
                            begin
                                // C.SW -> sw rs2', offset(rs1')
                                // offset[6:2] = {inst_c[5], inst_c[12:10], inst_c[6]}, offset[1:0]=00
                                // S-type imm[11:5] = {5'b0, inst_c[5], inst_c[12]} (7 bits)
                                // S-type imm[4:0] = {inst_c[11:10], inst_c[6], 2'b00} (5 bits)
                                inst_expanded = {
                                    5'b00000, inst_c[5], inst_c[12],
                                    rs2p,
                                    rs1p,
                                    3'b010,
                                    inst_c[11:10], inst_c[6], 2'b00,
                                    7'b0100011 // STORE
                                };
                            end
                        default: inst_expanded = 32'h00000000; // illegal / FP not implemented
                    endcase
            end
            2'b01: // Q1
                begin 
                    case (funct3)
                        3'b000: begin
                            // C.ADDI -> addi rd, rd, nzimm (C.NOP when rd==x0, imm==0)
                            // imm[5:0] = {inst_c[12], inst_c[6:2]}, sign-extended to 12 bits
                            // I-type imm is 12 bits: {{6{sign}}, inst_c[12], inst_c[6:2]}
                            inst_expanded = {
                                {6{inst_c[12]}}, inst_c[12], inst_c[6:2],
                                rd_full,
                                3'b000,
                                rd_full,
                                7'b0010011 // ADDI
                            };
                        end
                        3'b001: 
                            begin
                                // C.JAL (RV32 only) -> jal x1, offset
                                // CJ-format immediate mapping (12-bit signed, bit 0 always 0):
                                // imm[11] = inst_c[12]
                                // imm[10] = inst_c[8]
                                // imm[9:8] = inst_c[10:9]
                                // imm[7] = inst_c[6]
                                // imm[6] = inst_c[7]
                                // imm[5] = inst_c[2]
                                // imm[4] = inst_c[11]
                                // imm[3:1] = inst_c[5:3]
                                // imm[0] = 0 (implicit)
                                // JAL 32-bit encoding: {imm[20], imm[10:1], imm[11], imm[19:12], rd, opcode}
                                // imm[20] = sign = inst_c[12]
                                // imm[19:12]= {8{inst_c[12]}} (offset fits in 12 bits, sign-extend)
                                // imm[11] = inst_c[12]
                                // imm[10:1] = {inst_c[8], inst_c[10:9], inst_c[6], inst_c[7], inst_c[2], inst_c[11], inst_c[5:3]}
                                inst_expanded = {
                                    inst_c[12], // imm[20]
                                    inst_c[8], inst_c[10:9], inst_c[6], // imm[10:7]
                                    inst_c[7], inst_c[2], inst_c[11], inst_c[5:3], // imm[6:1]
                                    inst_c[12], // imm[11]
                                    {8{inst_c[12]}}, // imm[19:12]
                                    5'b00001, // rd = x1
                                    7'b1101111 // JAL
                                };
                            end
                        3'b010: 
                            begin
                                // C.LI -> addi rd, x0, imm
                                // imm[5:0] = {inst_c[12], inst_c[6:2]}, sign-extended to 12 bits
                                inst_expanded = {
                                    {6{inst_c[12]}}, inst_c[12], inst_c[6:2],
                                    5'b00000, // rs1 = x0
                                    3'b000,
                                    rd_full,
                                    7'b0010011 // ADDI
                                };
                            end
                        3'b011: 
                            begin
                                // C.ADDI16SP (rd==x2) -> addi x2, x2, nzimm
                                // C.LUI (rd!=x2) -> lui rd, nzuimm
                                if (rd_full == 5'b00010) begin
                                    // C.ADDI16SP
                                    // nzimm[9:4] = {inst_c[12], inst_c[4:3], inst_c[5], inst_c[2], inst_c[6]}
                                    // 12-bit signed imm = {{2{sign}}, inst_c[12], inst_c[4:3], inst_c[5], inst_c[2], inst_c[6], 4'b0}
                                    // 2+1+2+1+1+1+4 = 12 bits
                                    inst_expanded = {
                                        {2{inst_c[12]}}, inst_c[12], inst_c[4:3], inst_c[5], inst_c[2], inst_c[6], 4'b0000,
                                        5'b00010, // rs1 = x2
                                        3'b000,
                                        5'b00010, // rd = x2
                                        7'b0010011 // ADDI
                                    };
                                end else begin
                                    // C.LUI -> lui rd, nzuimm
                                    // nzuimm[17:12] = {inst_c[12], inst_c[6:2]}
                                    // U-type imm[31:12] (20 bits) = {{14{sign}}, inst_c[12], inst_c[6:2]}
                                    inst_expanded = {
                                        {14{inst_c[12]}}, inst_c[12], inst_c[6:2],
                                        rd_full,
                                        7'b0110111 // LUI
                                    };
                                end
                            end
                        3'b100: 
                            begin
                                case (inst_c[11:10])
                                    2'b00: 
                                        begin
                                        // C.SRLI -> srli rs1', rs1', shamt
                                        // RV32: shamt is 5 bits (inst_c[6:2]); inst_c[12] must be 0
                                        // I-type shift: funct7[6:1]=6'b000000, funct7[0]=inst_c[12], shamt=inst_c[6:2]
                                            inst_expanded = {
                                                6'b000000, inst_c[12], // funct7 (bit 0 = shamt[5], must be 0 for RV32)
                                                inst_c[6:2], // shamt[4:0]
                                                rs1p,
                                                3'b101, // SRLI/SRAI
                                                rs1p, // rd = rs1'
                                                7'b0010011
                                            };
                                        end
                                    2'b01: 
                                        begin
                                            // C.SRAI -> srai rs1', rs1', shamt
                                            // funct7[6:1]=6'b010000, funct7[0]=inst_c[12]
                                            inst_expanded = {
                                                6'b010000, inst_c[12], // funct7
                                                inst_c[6:2], // shamt[4:0]
                                                rs1p,
                                                3'b101,
                                                rs1p,
                                                7'b0010011
                                            };
                                        end

                                    2'b10: 
                                        begin
                                        // C.ANDI -> andi rs1', rs1', imm
                                        // imm[5:0] = {inst_c[12], inst_c[6:2]}, sign-extended to 12 bits
                                        inst_expanded = {
                                            {6{inst_c[12]}}, inst_c[12], inst_c[6:2],
                                            rs1p,
                                            3'b111, // AND
                                            rs1p,
                                            7'b0010011
                                        };
                                    end

                                    2'b11:
                                        begin
                                            // CA format: inst_c[6:5] selects operation; inst_c[12]=0 for RV32
                                            case (inst_c[6:5])
                                                2'b00: begin
                                                    // C.SUB -> sub rd', rd', rs2'
                                                    inst_expanded = {
                                                        7'b0100000,
                                                        rs2p, rs1p,
                                                        3'b000,
                                                        rs1p,
                                                        7'b0110011 // OP
                                                    };
                                                end
                                                2'b01: 
                                                    begin
                                                        // C.XOR -> xor rd', rd', rs2'
                                                        inst_expanded = {
                                                            7'b0000000,
                                                            rs2p, rs1p,
                                                            3'b100,
                                                            rs1p,
                                                            7'b0110011
                                                        };
                                                    end
                                                2'b10: 
                                                    begin
                                                        // C.OR -> or rd', rd', rs2'
                                                        inst_expanded = {
                                                            7'b0000000,
                                                            rs2p, rs1p,
                                                            3'b110,
                                                            rs1p,
                                                            7'b0110011
                                                        };
                                                    end
                                                2'b11: 
                                                    begin
                                                        // C.AND -> and rd', rd', rs2'
                                                        inst_expanded = {
                                                            7'b0000000,
                                                            rs2p, rs1p,
                                                            3'b111,
                                                            rs1p,
                                                            7'b0110011
                                                        };
                                                    end
                                            endcase
                                        end
                                endcase
                            end 
                        3'b101: 
                            begin
                                // C.J -> jal x0, offset (same imm encoding as C.JAL, rd=x0)
                                inst_expanded = {
                                    inst_c[12], inst_c[8], inst_c[10:9], inst_c[6], inst_c[7], inst_c[2], inst_c[11], inst_c[5:3], inst_c[12], {8{inst_c[12]}}, // imm[11], imm[19:12]
                                    5'b00000, // rd = x0 
                                    7'b1101111 // JAL
                                };
                            end
                        3'b110: 
                            begin
                                // C.BEQZ -> beq rs1', x0, offset
                                inst_expanded = {
                                    inst_c[12], // imm[12] (Sign bit)    
                                    // imm[10:5]: Top 3 bits are sign-extended, then mapped bits
                                    {3{inst_c[12]}}, inst_c[6:5], inst_c[2],                                   
                                    5'b00000, // rs2 = x0
                                    rs1p, // rs1 = rs1'
                                    3'b000, // BEQ
                                    inst_c[11:10], inst_c[4:3], // imm[4:1]                        
                                    inst_c[12], // imm[11] (Sign bit, NOT 1'b0!)
                                    7'b1100011 // BRANCH
                                };
                            end
                        3'b111: 
                            begin
                                // C.BNEZ -> bne rs1', x0, offset
                                inst_expanded = {
                                    inst_c[12], // imm[12]
                                    {3{inst_c[12]}}, inst_c[6:5], inst_c[2], // imm[10:5]
                                    5'b00000, // rs2 = x0
                                    rs1p,
                                    3'b001, // BNE
                                    inst_c[11:10], inst_c[4:3], // imm[4:1]
                                    inst_c[12], // imm[11]
                                    7'b1100011 // BRANCH
                                };
                            end
                    endcase
                end
            2'b10: // Q2
                begin
                    case (funct3)
                        3'b000: 
                            begin
                                // C.SLLI -> slli rd, rd, shamt
                                // RV32: shamt[4:0] = inst_c[6:2]; inst_c[12] must be 0
                                inst_expanded = {
                                    6'b000000, inst_c[12], // funct7
                                    inst_c[6:2], // shamt[4:0]
                                    rd_full,
                                    3'b001, // SLLI
                                    rd_full,
                                    7'b0010011
                                };
                            end
                        3'b010: 
                            begin
                                // C.LWSP -> lw rd, offset(x2)
                                // offset[7:2] = {inst_c[3:2], inst_c[12], inst_c[6:4]}, offset[1:0]=00
                                // I-type 12-bit imm = {4'b0, inst_c[3:2], inst_c[12], inst_c[6:4], 2'b00}
                                inst_expanded = {
                                    4'b0000, inst_c[3:2], inst_c[12], inst_c[6:4], 2'b00,
                                    5'b00010, // rs1 = x2 (sp)
                                    3'b010,
                                    rd_full,
                                    7'b0000011 // LOAD
                                };
                            end
                        3'b110: 
                            begin
                                // C.SWSP -> sw rs2, offset(x2)
                                // offset[7:6] = inst_c[8:7], offset[5:2] = inst_c[12:9], offset[1:0]=00
                                // S-type imm[11:5] = {4'b0, inst_c[8:7], inst_c[12]} (7 bits)
                                // S-type imm[4:0] = {inst_c[11:9], 2'b00} (5 bits)
                                inst_expanded = {
                                    4'b0000, inst_c[8:7], inst_c[12],
                                    rs2_full,
                                    5'b00010, // rs1 = x2 (sp)
                                    3'b010,
                                    inst_c[11:9], 2'b00,
                                    7'b0100011 // STORE
                                };
                            end
                        3'b100: 
                            begin
                                // Decoded by inst_c[12] (j-bit) then rs2_full
                                if (inst_c[12] == 1'b0) begin
                                    if (rs2_full == 5'b00000) begin
                                        // C.JR -> jalr x0, 0(rs1)
                                        inst_expanded = {
                                            12'b0,
                                            rd_full, // rs1 = inst_c[11:7]
                                            3'b000,
                                            5'b00000, // rd = x0
                                            7'b1100111 // JALR
                                        };
                                    end else begin
                                        // C.MV -> add rd, x0, rs2
                                        inst_expanded = {
                                            7'b0000000,
                                            rs2_full,
                                            5'b00000, // rs1 = x0
                                            3'b000,
                                            rd_full,
                                            7'b0110011 // ADD
                                        };
                                    end
                                end else begin // inst_c[12] == 1
                                    if (rd_full == 5'b00000 && rs2_full == 5'b00000) begin
                                        // C.EBREAK -> ebreak
                                        inst_expanded = 32'h00100073;
                                    end else if (rs2_full == 5'b00000) begin
                                        // C.JALR -> jalr x1, 0(rs1)
                                        inst_expanded = {
                                            12'b0,
                                            rd_full, // rs1
                                            3'b000,
                                            5'b00001, // rd = x1
                                            7'b1100111 // JALR
                                        };
                                    end else begin
                                        // C.ADD -> add rd, rd, rs2
                                        inst_expanded = {
                                            7'b0000000,
                                            rs2_full,
                                            rd_full, // rs1 = rd
                                            3'b000,
                                            rd_full,
                                            7'b0110011 // ADD
                                        };
                                    end
                                end
                            end
                        default: inst_expanded = 32'h00000000; // illegal
                    endcase
                end 
        endcase
    end

endmodule

`endif // _RVC_EXPANSION_V_
