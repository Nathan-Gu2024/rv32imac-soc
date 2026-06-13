`include "imem.v"
`include "dmem.v"
`include "regfile.v"
`include "alu.v"
`include "branch_comp.v"
`include "control_logic.v"
`include "immgen.v"
`include "partial_load.v"
`include "partial_store.v"
`include "hazard_unit.v"

// Single cycle 
// module cpu_single_cycle (
//     input wire clk, rst
// );
//     wire [31:0] pc, exe_pc;
//     wire [31:0] inst;

//     wire pc_sel, reg_wen, br_un, a_sel, b_sel, mem_rw;
//     wire [1:0] wb_sel;
//     wire [2:0] imm_sel;
//     wire [3:0] alu_sel;

//     wire [31:0] rs1_data, rs2_data;

//     wire [31:0] imm; 

//     wire [31:0] alu_out, alu_a, alu_b;

//     wire br_eq, br_lt;

//     wire [31:0] mem_read_data, store_data;
//     wire [3:0] mem_write_mask;

//     wire [31:0] wb_data, partial_load_out;

//     assign exe_pc = pc;
//     program_counter PC (
//         .clk(clk), 
//         .rst(rst),
//         .pc_sel(pc_sel),
//         .mem_address(alu_out),
//         .pc(pc),
//     );

//     imem IMEM (
//         .pc(pc),
//         .inst(inst)
//     );

    
//     control_logic CL (
//         .inst(inst),
//         .br_eq(br_eq), 
//         .br_lt(br_lt),
//         .pc_sel(pc_sel),
//         .reg_wen(reg_wen),
//         .imm_sel(imm_sel),
//         .br_un(br_un),
//         .a_sel(a_sel),
//         .b_sel(b_sel),
//         .alu_sel(alu_sel),
//         .mem_rw(mem_rw),
//         .wb_sel(wb_sel)
//     );

//     regfile RF (
//         .clk(clk),
//         .reg_wen(reg_wen),
//         .read_index1(inst[19:15]),
//         .read_index2(inst[24:20]),
//         .write_index(inst[11:7]), 
//         .write_data(wb_data),
//         .read_data1(rs1_data),
//         .read_data2(rs2_data)
//     );

//     immgen IMM (
//         .inst(inst), 
//         .imm_sel(imm_sel),
//         .imm(imm)
//     );

//     branch_comp BC (
//         .br_data1(rs1_data),
//         .br_data2(rs2_data),
//         .br_un(br_un),
//         .br_eq(br_eq),
//         .br_lt(br_lt)
//     );

//     assign alu_a = a_sel ? exe_pc : rs1_data;
//     assign alu_b = b_sel ? imm : rs2_data;

//     alu ALU (
//         .a(alu_a),
//         .b(alu_b),
//         .alu_sel(alu_sel),
//         .alu_res(alu_out)
//     );

//     partial_store PS (
//         .inst(inst),
//         .mem_address(alu_out),
//         .data_from_reg(rs2_data),
//         .mem_rw(mem_rw),
//         .mem_write_mask(mem_write_mask),
//         .data_to_mem(store_data)
//     );

//     dmem DMEM (
//         .clk(clk), 
//         .mem_address(alu_out),
//         .mem_write_data(store_data),
//         .mem_write_mask(mem_write_mask),
//         .mem_read_data(mem_read_data)
//     ); 

//     partial_load PL (
//         .inst(inst),
//         .mem_address(alu_out), 
//         .data_from_mem(mem_read_data),
//         .data_to_reg(partial_load_out)
//     );

//     assign wb_data = (wb_sel == 2'b00) ? alu_out : 
//                     (wb_sel == 2'b01) ? partial_load_out :
//                     (wb_sel == 2'b10) ? (pc + 32'd4) :
//                     32'b0;

// endmodule



// 5 Stage 
module cpu_pipelined ( 
    input wire clk, rst
);
    // IF 
    wire [31:0] pc, if_inst;

    // IF/ID out
    wire [31:0] if_id_pc, if_id_inst;

    // ID
    wire [31:0] rs1_data, rs2_data, imm;
    wire pc_sel, reg_wen, a_sel, b_sel, mem_rw;
    wire [1:0] wb_sel;
    wire [2:0] imm_sel;
    wire [3:0] alu_sel;

    // ID/EX out
    wire [31:0] id_ex_pc, id_ex_rs1, id_ex_rs2, id_ex_imm, id_ex_inst;
    wire [4:0] id_ex_rd;
    wire id_ex_reg_wen, id_ex_mem_rw, id_ex_a_sel, id_ex_b_sel;
    wire [1:0] id_ex_wb_sel;
    wire [3:0] id_ex_alu_sel;

    // EX
    wire [31:0] alu_a, alu_b, alu_out, fwd_rs1, fwd_rs2;
    wire [1:0] fwd_a, fwd_b;

    // EX/MEM out
    wire [31:0] ex_mem_alu, ex_mem_rs2, ex_mem_inst, ex_mem_pc;
    wire [4:0] ex_mem_rd;
    wire ex_mem_reg_wen, ex_mem_mem_rw;
    wire [1:0] ex_mem_wb_sel;

    // MEM
    wire [31:0] mem_read_data, store_data, partial_load_out;
    wire [3:0] mem_write_mask;

    // MEM/WB out
    wire [31:0] mem_wb_alu, mem_wb_memdata, mem_wb_pc, mem_wb_inst;
    wire [4:0] mem_wb_rd;
    wire mem_wb_reg_wen;
    wire [1:0] mem_wb_wb_sel;

    // WB
    wire [31:0] wb_data;
    wire stall;

    // IF
    program_counter PC (
        .clk(clk), 
        .rst(rst),
        .stall(stall), 
        .pc_sel(pc_sel), 
        .mem_address(alu_out), 
        .pc(pc)
    );

    imem IMEM (
        .pc(pc),
        .inst(if_inst)
    ); 

    if_id_reg IF_ID (
        .clk(clk), 
        .rst(rst), 
        .stall(stall), 
        .flush(pc_sel), 
        .pc_in(pc), 
        .inst_in(if_inst), 
        .pc_out(if_id_pc),
        .inst_out(if_id_inst)
    ); 

    // ID
    control_logic CL (
        .inst(if_id_inst), 
        .reg_wen(reg_wen), 
        .imm_sel(imm_sel), 
        .a_sel(a_sel), 
        .b_sel(b_sel),
        .alu_sel(alu_sel),
        .mem_rw(mem_rw), 
        .wb_sel(wb_sel)
    );

    regfile RF (
        .clk(clk), 
        .reg_wen(mem_wb_reg_wen), 
        .read_index1(if_id_inst[19:15]),
        .read_index2(if_id_inst[24:20]),
        .write_index(mem_wb_rd), 
        .write_data(wb_data), 
        .read_data1(rs1_data),
        .read_data2(rs2_data)
    );

    immgen IMM (
        .inst(if_id_inst), 
        .imm_sel(imm_sel), 
        .imm(imm)
    );

id_ex_reg ID_EX (
        .clk(clk), 
        .rst(rst),
        .flush(pc_sel || stall), // Connected to .flush instead of .stall
        .pc_in(if_id_pc), 
        .rs1_in(rs1_data), 
        .rs2_in(rs2_data),
        .imm_in(imm), 
        .rd_in(if_id_inst[11:7]),
        .inst_in(if_id_inst),
        .reg_wen_in(reg_wen), 
        .mem_rw_in(mem_rw),
        .a_sel_in(a_sel), 
        .b_sel_in(b_sel), 
        .wb_sel_in(wb_sel),
        .alu_sel_in(alu_sel), 
        .pc_out(id_ex_pc), 
        .rs1_out(id_ex_rs1), 
        .rs2_out(id_ex_rs2), 
        .imm_out(id_ex_imm), 
        .rd_out(id_ex_rd), 
        .inst_out(id_ex_inst), 
        .reg_wen_out(id_ex_reg_wen), 
        .mem_rw_out(id_ex_mem_rw),
        .a_sel_out(id_ex_a_sel), 
        .b_sel_out(id_ex_b_sel), 
        .wb_sel_out(id_ex_wb_sel), 
        .alu_sel_out(id_ex_alu_sel)
    );

    // EX 
    // Forwarding
    assign fwd_rs1 = (fwd_a == 2'b01) ? ex_mem_alu :
                    (fwd_a == 2'b10) ? wb_data : 
                    id_ex_rs1;

    assign fwd_rs2 = (fwd_b == 2'b01) ? ex_mem_alu :
                    (fwd_b == 2'b10) ? wb_data : 
                    id_ex_rs2;
    
    assign alu_a = id_ex_a_sel ? id_ex_pc : fwd_rs1;
    assign alu_b = id_ex_b_sel ? id_ex_imm : fwd_rs2;

    alu ALU (
        .a(alu_a), 
        .b(alu_b),
        .alu_sel(id_ex_alu_sel),
        .alu_res(alu_out)
    );

    wire [6:0] ex_opcode = id_ex_inst[6:0];
    wire [2:0] ex_funct3 = id_ex_inst[14:12];

    wire id_ex_is_branch = (ex_opcode == 7'b1100011);
    wire id_ex_is_beq = id_ex_is_branch && (ex_funct3 == 3'b000);
    wire id_ex_is_bne = id_ex_is_branch && (ex_funct3 == 3'b001);
    wire id_ex_is_blt = id_ex_is_branch && (ex_funct3 == 3'b100);
    wire id_ex_is_bge = id_ex_is_branch && (ex_funct3 == 3'b101);
    wire id_ex_is_bltu = id_ex_is_branch && (ex_funct3 == 3'b110);
    wire id_ex_is_bgeu = id_ex_is_branch && (ex_funct3 == 3'b111);
    wire id_ex_is_jal = (ex_opcode == 7'b1101111);
    wire id_ex_is_jalr = (ex_opcode == 7'b1100111);

    wire id_ex_br_eq, id_ex_br_lt;
    branch_comp BC (
        .br_data1(fwd_rs1),
        .br_data2(fwd_rs2),
        .br_un(ex_funct3[1]),
        .br_eq(id_ex_br_eq),
        .br_lt(id_ex_br_lt)
    );

    assign pc_sel = (id_ex_br_eq & id_ex_is_beq) |
                (~id_ex_br_eq & id_ex_is_bne) |
                (id_ex_br_lt & (id_ex_is_blt|id_ex_is_bltu)) |
                (~id_ex_br_lt & (id_ex_is_bge|id_ex_is_bgeu)) |
                id_ex_is_jal | id_ex_is_jalr;

    hazard_unit HU (
        .id_ex_rd(id_ex_rd), 
        .id_ex_wb_sel(id_ex_wb_sel),
        .if_id_rs1(if_id_inst[19:15]), 
        .if_id_rs2(if_id_inst[24:20]),
        .ex_mem_rd(ex_mem_rd),
        .mem_wb_rd(mem_wb_rd),
        .ex_mem_reg_wen(ex_mem_reg_wen), 
        .mem_wb_reg_wen(mem_wb_reg_wen),
        .id_ex_rs1(id_ex_inst[19:15]), 
        .id_ex_rs2(id_ex_inst[24:20]),
        .pc_sel(pc_sel),
        .stall(stall), 
        .fwd_a(fwd_a), 
        .fwd_b(fwd_b)
    );
    
    ex_mem_reg EX_MEM (
        .clk(clk), 
        .rst(rst), 
        .alu_res_in(alu_out), 
        .rs2_in(fwd_rs2), 
        .inst_in(id_ex_inst), 
        .pc_in(id_ex_pc),
        .rd_in(id_ex_rd), 
        .reg_wen_in(id_ex_reg_wen),
        .mem_rw_in(id_ex_mem_rw), 
        .wb_sel_in(id_ex_wb_sel),
        .alu_res_out(ex_mem_alu),
        .rs2_out(ex_mem_rs2), 
        .inst_out(ex_mem_inst),
        .pc_out(ex_mem_pc),
        .rd_out(ex_mem_rd), 
        .reg_wen_out(ex_mem_reg_wen), 
        .mem_rw_out(ex_mem_mem_rw), 
        .wb_sel_out(ex_mem_wb_sel)
    );

    // MEM
    partial_store PS (
        .inst(ex_mem_inst), 
        .mem_address(ex_mem_alu), 
        .data_from_reg(ex_mem_rs2), 
        .mem_rw(ex_mem_mem_rw), 
        .mem_write_mask(mem_write_mask), 
        .data_to_mem(store_data)
    ); 

    dmem DMEM (
        .clk(clk), 
        .mem_address(ex_mem_alu),
        .mem_write_data(store_data), 
        .mem_write_mask(mem_write_mask),
        .mem_read_data(mem_read_data)
    );

    mem_wb_reg MEM_WB (
        .clk(clk),
        .rst(rst), 
        .inst_in(ex_mem_inst),
        .alu_res_in(ex_mem_alu), 
        .mem_data_in(mem_read_data), 
        .pc_in(ex_mem_pc),
        .rd_in(ex_mem_rd),
        .reg_wen_in(ex_mem_reg_wen),
        .wb_sel_in(ex_mem_wb_sel), 
        .inst_out(mem_wb_inst),
        .alu_res_out(mem_wb_alu), 
        .mem_data_out(mem_wb_memdata),
        .pc_out(mem_wb_pc),
        .rd_out(mem_wb_rd), 
        .reg_wen_out(mem_wb_reg_wen), 
        .wb_sel_out(mem_wb_wb_sel)
    ); 

    // WB
    partial_load PL (
        .inst(mem_wb_inst), 
        .mem_address(mem_wb_alu), 
        .data_from_mem(mem_wb_memdata),
        .data_to_reg(partial_load_out)
    );

    assign wb_data = (mem_wb_wb_sel == 2'b01) ? mem_wb_alu : // ALU
                    (mem_wb_wb_sel == 2'b00) ? partial_load_out : // MEM
                    (mem_wb_wb_sel == 2'b10) ? (mem_wb_pc + 32'd4) : // PC + 4
                    32'b0;

endmodule

module program_counter (
    input wire [31:0] mem_address,
    input wire clk, rst, pc_sel, stall,
    output reg [31:0] pc
);
    wire [31:0] next_pc = pc_sel ? mem_address : (pc + 32'd4);

    always @(posedge clk) begin
        if (rst) 
            pc <= 32'b0;
        else if (!stall) 
            pc <= next_pc;
    end

endmodule



module if_id_reg (
    input wire clk, rst, stall, flush,
    input wire [31:0] pc_in, inst_in,
    output reg [31:0] pc_out, inst_out
);
    always @(posedge clk) begin
        if (rst || flush) begin
            pc_out <= 32'b0;
            inst_out <= 32'h00000013;
        end else if (!stall) begin
            pc_out <= pc_in;
            inst_out <= inst_in;
        end
    end
    
endmodule

module id_ex_reg (
    input wire clk, rst, flush,
    input wire [31:0] pc_in, rs1_in, rs2_in, imm_in, 
    input wire [4:0] rd_in, 
    input wire reg_wen_in, mem_rw_in, a_sel_in, b_sel_in, 
    input wire [1:0] wb_sel_in, 
    input wire [3:0] alu_sel_in,
    input wire [31:0] inst_in,
    output reg [31:0] pc_out, rs1_out, rs2_out, imm_out,
    output reg [4:0] rd_out,
    output reg reg_wen_out, mem_rw_out, a_sel_out, b_sel_out, 
    output reg [1:0] wb_sel_out, 
    output reg [3:0] alu_sel_out, 
    output reg [31:0] inst_out
); 

    always @(posedge clk) begin
        if (rst || flush) begin
            mem_rw_out <= 0;
            rd_out <= 5'b0;
            pc_out <= 0;
            rs1_out <= 0;
            rs2_out <= 0;
            imm_out <= 0;
            reg_wen_out <= 0;
            a_sel_out <= 0;
            b_sel_out <= 0;
            wb_sel_out <= 0;
            alu_sel_out <= 0;
            inst_out <= 32'h00000013;
        end else begin
            mem_rw_out <= mem_rw_in;
            rd_out <= rd_in;
            pc_out <= pc_in;
            rs1_out <= rs1_in;
            rs2_out <= rs2_in;
            imm_out <= imm_in;
            reg_wen_out <= reg_wen_in;
            a_sel_out <= a_sel_in;
            b_sel_out <= b_sel_in;
            wb_sel_out <= wb_sel_in;
            alu_sel_out <= alu_sel_in;
            inst_out <= inst_in;
        end 
    end 

endmodule


module ex_mem_reg (
    input wire  clk, rst,
    input wire [31:0] alu_res_in, rs2_in, inst_in, pc_in,
    input wire [4:0] rd_in,
    input wire reg_wen_in, mem_rw_in,
    input wire [1:0] wb_sel_in,
    output reg [31:0] alu_res_out, rs2_out, inst_out, pc_out, 
    output reg [4:0] rd_out,
    output reg reg_wen_out, mem_rw_out,
    output reg [1:0] wb_sel_out
);
    always @(posedge clk) begin
        if (rst) begin
            reg_wen_out <= 0;  
            mem_rw_out <= 0;
            rd_out <= 0;  
            alu_res_out <= 0;
            rs2_out <= 0;
             wb_sel_out <= 0;
            inst_out <= 32'h00000013;
            pc_out <= 0;                    
        end else begin
            alu_res_out <= alu_res_in;
            rs2_out <= rs2_in;
            inst_out <= inst_in;
            rd_out <= rd_in;
            reg_wen_out <= reg_wen_in;
            mem_rw_out <= mem_rw_in;
            wb_sel_out <= wb_sel_in;
            pc_out <= pc_in;                
        end
    end

endmodule


module mem_wb_reg (
    input wire clk, rst,
    input wire [31:0] alu_res_in, mem_data_in, pc_in, inst_in,
    input wire [4:0] rd_in,
    input wire reg_wen_in,
    input wire [1:0] wb_sel_in,
    output reg [31:0] alu_res_out, mem_data_out, pc_out, inst_out,
    output reg [4:0] rd_out,
    output reg reg_wen_out,
    output reg [1:0] wb_sel_out
);
    always @(posedge clk) begin
        if (rst) begin
            reg_wen_out <= 0;  
            rd_out <= 0;
            alu_res_out <= 0;  
            mem_data_out <= 0;
            pc_out <= 0;  
            wb_sel_out <= 0;
            inst_out <= 32'h00000013;
        end else begin
            alu_res_out <= alu_res_in;
            mem_data_out <= mem_data_in;
            pc_out <= pc_in;           
            rd_out <= rd_in;
            reg_wen_out <= reg_wen_in;
            wb_sel_out <= wb_sel_in;
            inst_out <= inst_in;
        end
    end

endmodule
