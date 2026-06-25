`include "../fpga/imem.v"
`include "../fpga/dmem.v"
`include "../src/regfile.v"
`include "../src/alu.v"
`include "../src/branch_comp.v"
`include "../src/control_logic.v"
`include "../src/immgen.v"
`include "../src/partial_load.v"
`include "../src/partial_store.v"
`include "../src/hazard_unit.v"
`include "../src/direct_mapped_cache.v"
`include "../src/rvc_expansion.v"
`include "../src/reservation_monitor.v"

module cpu_pipelined ( 
    input wire clk, rst, uart_tx_ready, 
    output reg uart_tx_start, 
    output reg [7:0] uart_tx_data, 
    output reg [3:0] leds,

    output wire [31:0] icache_mem_req_addr, 
    output wire icache_mem_req_valid, 
    input wire [127:0] icache_mem_read_data, 
    input wire icache_mem_ready, 
    
    output wire [31:0] dmem_req_addr, 
    output wire [31:0] store_data, 
    output wire [3:0] mem_write_mask, 
    output wire dcache_mem_req_valid, 
    input wire [127:0] dcache_mem_read_data_block, 
    input wire dcache_mem_ready
);
    // IF 
    wire [31:0] pc, if_inst, inst_expanded;

    // IF/ID out
    wire [31:0] if_id_pc, if_id_inst;
    wire is_compressed;
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
    wire [31:0] mem_read_data, partial_load_out;
    // wire [31:0] store_data;
    // wire [3:0] mem_write_mask;

    // MEM/WB out
    wire [31:0] mem_wb_alu, mem_wb_memdata, mem_wb_pc, mem_wb_inst;
    wire [4:0] mem_wb_rd;
    wire mem_wb_reg_wen;
    wire [1:0] mem_wb_wb_sel;

    // WB
    wire [31:0] wb_data;
    wire stall;


    // Caching / Memory flags
    // CPU asserts Valid if it is a Load or Store instruction in the MEM stage
    wire is_load = (ex_mem_wb_sel == 2'b00) && ex_mem_reg_wen; 
    wire is_store = (ex_mem_mem_rw == 1'b1);  
    wire dcache_valid = is_load | is_store;
    
    wire dcache_ready; 
    wire dmem_stall = dcache_valid & (~dcache_ready);
    wire [31:0] dcache_read_data;

    wire cache_ready;
    // wire [31:0] icache_mem_req_addr;
    // wire [127:0] icache_mem_read_data;
    // wire icache_mem_ready;
    // wire icache_mem_req_valid; 

    direct_mapped_cache ICACHE (
        .clk(clk), 
        .rst(rst), 
        .cpu_req_addr(pc),
        .cpu_write_data(32'b0), 
        .cpu_read_req(1'b1), 
        .cpu_write_req(1'b0), 
        .mem_write_mask(4'b0000),
        .mem_ready(icache_mem_ready), 
        .mem_read_data(icache_mem_read_data), 
        .cpu_read_data(if_inst), 
        .mem_req_addr(icache_mem_req_addr), 
        .cpu_ready(cache_ready), 
        .mem_req_valid(icache_mem_req_valid)
    ); 

    // Same for the Instruction Memory
    wire icache_valid = 1'b1; // The CPU is ALWAYS trying to fetch instructions!
    wire imem_stall = icache_valid & (~cache_ready);

    // Master Pipeline Stall
    // Freeze the whole CPU if either memory is stalling
    wire global_mem_stall = dmem_stall | imem_stall;

    wire [127:0] imem_read_data;
    wire imem_ready;
    wire [31:0] icache_mem_addr;

    // wire dcache_mem_req_valid;
    // wire dcache_mem_ready;
    wire [127:0] dcache_mem_read_data;
    wire [31:0] dcache_mem_req_addr;

    // wire [127:0] dcache_mem_read_data_block;
    
    direct_mapped_cache DCACHE (
        .clk(clk),
        .rst(rst),
        .cpu_req_addr(ex_mem_alu),  
        .cpu_write_data(store_data), 
        .mem_write_mask(mem_write_mask),
        .cpu_read_req(is_load),
        .cpu_write_req(is_store),        
        .mem_ready(dcache_mem_ready),
        .mem_read_data(dcache_mem_read_data_block),        
        .cpu_read_data(dcache_read_data),
        .mem_req_addr(dcache_mem_req_addr),
        .cpu_ready(dcache_ready),         
        .mem_req_valid(dcache_mem_req_valid)
    );

    assign dmem_req_addr = is_store ? ex_mem_alu : dcache_mem_req_addr;
    // dmem DMEM (
    //     .clk(clk),
    //     .mem_req_valid(dcache_mem_req_valid),
    //     .mem_address(is_store ? ex_mem_alu : dcache_mem_req_addr),
    //     .mem_write_data(store_data),    
    //     .mem_write_mask(mem_write_mask),    
    //     .mem_read_data(mem_read_data),
    //     .mem_ready(dcache_mem_ready),
    //     .mem_read_data_block(dcache_mem_read_data_block)
    // );

    // IF    
    // Fetch buffer state
    reg [15:0] fetch_buffer;
    reg buffer_valid;

    // Detection
    wire [1:0] opcode_check = pc[1] ? if_inst[17:16] : if_inst[1:0];
    wire is_32_bit_opcode = (opcode_check == 2'b11);
    wire unaligned_32_bit_fetch = (pc[1] == 1'b1) && is_32_bit_opcode && !buffer_valid;

    always @(posedge clk) begin
        if (rst || pc_sel) begin
            buffer_valid <= 1'b0;
            fetch_buffer <= 16'b0;
        end else if (!global_mem_stall && !stall) begin
            if (unaligned_32_bit_fetch) begin
                fetch_buffer <= if_inst[31:16];
                buffer_valid <= 1'b1;
            end else begin
                buffer_valid <= 1'b0;
            end 
        end 
    end

    // Instruction assembly
    wire [31:0] raw_inst = buffer_valid ? {if_inst[15:0], fetch_buffer} : (pc[1] ? {16'b0, if_inst[31:16]} : if_inst);

    rvc_expand RVC (
        .inst_c(raw_inst[15:0]), 
        .inst_expanded(inst_expanded), 
        .is_compressed(is_compressed)
    );
    // Pipeline routing
    wire [31:0] final_inst = is_compressed ? inst_expanded : raw_inst;

    wire [31:0] muxed_if_inst = unaligned_32_bit_fetch ? 32'h00000013 : final_inst;

    wire [31:0] pc_inc = (unaligned_32_bit_fetch || buffer_valid || is_compressed) ? 32'd2 : 32'd4;

    program_counter PC (
        .clk(clk), 
        .rst(rst),
        .stall(stall | global_mem_stall), 
        .pc_sel(pc_sel), 
        .mem_address(alu_out), 
        .pc_inc(pc_inc),
        .pc(pc)
    );

    if_id_reg IF_ID (
        .clk(clk), 
        .rst(rst), 
        .stall(stall), 
        .mem_stall(global_mem_stall),
        .flush(pc_sel), 
        .pc_in(pc), 
        .inst_in(muxed_if_inst), 
        .pc_out(if_id_pc),
        .inst_out(if_id_inst)
    ); 
                
    // imem IMEM (
    //     .clk(clk), 
    //     .rst(rst), 
    //     .mem_req_valid(icache_mem_req_valid), 
    //     .mem_req_addr(icache_mem_req_addr), 
    //     .mem_read_data(icache_mem_read_data), 
    //     .mem_ready(icache_mem_ready)
    // );

    // ID
    wire is_lr, is_sc, is_amo;
    wire [4:0] atomic_op;

    control_logic CL (
        .inst(if_id_inst), 
        .reg_wen(reg_wen), 
        .imm_sel(imm_sel), 
        .a_sel(a_sel), 
        .b_sel(b_sel),
        .alu_sel(alu_sel),
        .mem_rw(mem_rw), 
        .wb_sel(wb_sel),
        .out_is_lr(is_lr),
        .out_is_sc(is_sc),
        .out_is_amo(is_amo),
        .out_atomic_op(atomic_op)
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
    wire id_ex_is_lr, id_ex_is_sc, id_ex_is_amo; 
    wire [4:0] id_ex_atomic_op;
    id_ex_reg ID_EX (
        .clk(clk), 
        .rst(rst),
        .flush(pc_sel || stall), 
        .mem_stall(global_mem_stall),
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
        .is_lr_in(is_lr), 
        .is_sc_in(is_sc), 
        .is_amo_in(is_amo), 
        .atomic_op_in(atomic_op), 
        .is_lr_out(id_ex_is_lr), 
        .is_sc_out(id_ex_is_sc), 
        .is_amo_out(id_ex_is_amo), 
        .atomic_op_out(id_ex_atomic_op), 
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
    wire ex_mem_is_lr, ex_mem_is_sc, ex_mem_is_amo;
    wire [4:0] ex_mem_atomic_op;
    ex_mem_reg EX_MEM (
        .clk(clk), 
        .rst(rst), 
        .mem_stall(global_mem_stall),
        .alu_res_in(alu_out), 
        .rs2_in(fwd_rs2), 
        .inst_in(id_ex_inst), 
        .pc_in(id_ex_pc),
        .rd_in(id_ex_rd), 
        .reg_wen_in(id_ex_reg_wen),
        .mem_rw_in(id_ex_mem_rw), 
        .wb_sel_in(id_ex_wb_sel),
        .is_lr_in(id_ex_is_lr), 
        .is_sc_in(id_ex_is_sc), 
        .is_amo_in(id_ex_is_amo), 
        .atomic_op_in(id_ex_atomic_op), 
        .is_lr_out(ex_mem_is_lr), 
        .is_sc_out(ex_mem_is_sc), 
        .is_amo_out(ex_mem_is_amo), 
        .atomic_op_out(ex_mem_atomic_op), 
        .alu_res_out(ex_mem_alu),
        .rs2_out(ex_mem_rs2), 
        .inst_out(ex_mem_inst),
        .pc_out(ex_mem_pc),
        .rd_out(ex_mem_rd), 
        .reg_wen_out(ex_mem_reg_wen), 
        .mem_rw_out(ex_mem_mem_rw), 
        .wb_sel_out(ex_mem_wb_sel)
    );

    // always @(posedge clk) begin
    //     if (rst) begin
    //         leds <= 16'b0;
    //     end else if (ex_mem_mem_rw) begin
    //         if (ex_mem_alu == 32'h00002000) begin
    //             leds <= ex_mem_rs2[15:0];
    //         end 
    //     end 
    // end 

    // always @(posedge clk) begin
    //     if (rst) begin
    //         leds <= 3'b0; 
    //         uart_tx_start <= 1'b0;
    //     end else begin
    //         uart_tx_start <= 1'b0; 
    //         if (ex_mem_mem_rw) begin
    //             if (ex_mem_alu == 32'h00002000) begin
    //                 leds <= ex_mem_rs2[2:0]; 
    //             end else if (ex_mem_alu == 32'h00003000) begin
    //                 uart_tx_data <= ex_mem_rs2[7:0];
    //                 uart_tx_start <= 1'b1;
    //             end 
    //         end 
    //     end 
    // end 
    always @(posedge clk) begin
        if (rst) begin
            leds <= 4'b0;
            uart_tx_start <= 1'b0;
            uart_tx_data <= 8'b0;
        end else begin
            uart_tx_start <= 1'b0;
            if (ex_mem_mem_rw) begin
                if (ex_mem_alu == 32'h00002000) begin
                    leds <= ex_mem_rs2[3:0];
                end else if (ex_mem_alu == 32'h00003000) begin
                    uart_tx_data <= ex_mem_rs2[7:0];
                    uart_tx_start <= 1'b1;
                end 
            end 
        end 
    end


    // MEM
    wire [3:0] raw_write_mask;
    partial_store PS (
        .inst(ex_mem_inst), 
        .mem_address(ex_mem_alu), 
        .data_from_reg(ex_mem_rs2), 
        .mem_rw(ex_mem_mem_rw), 
        .mem_write_mask(raw_write_mask), 
        .data_to_mem(store_data)
    ); 

    wire sc_success_flag;
    
    reservation_monitor RM (
    .clk(clk),
    .rst(rst),
    .lr_en(ex_mem_is_lr & ~global_mem_stall),
    .sc_en(ex_mem_is_sc & ~global_mem_stall),
    .any_store_en(is_store & ~ex_mem_is_sc & ~global_mem_stall),
    .trap_taken(1'b0), // temp
    .mem_addr(ex_mem_alu),
    .sc_successful(sc_success_flag)
     );

    wire block_sc_store = ex_mem_is_sc & ~sc_success_flag;
    assign mem_write_mask = block_sc_store ? 4'b0000 : raw_write_mask;
    wire [31:0] final_alu_to_wb;
    assign final_alu_to_wb = ex_mem_is_sc ? 
                             (sc_success_flag ? 32'd0 : 32'd1) :
                             ex_mem_alu;  

    
    mem_wb_reg MEM_WB (
        .clk(clk),
        .rst(rst), 
        .mem_stall(global_mem_stall),
        .inst_in(ex_mem_inst),
        .alu_res_in(final_alu_to_wb), 
        .mem_data_in(dcache_read_data), 
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
    input wire [31:0] mem_address, pc_inc, 
    input wire clk, rst, pc_sel, stall,
    output reg [31:0] pc
);
    wire [31:0] next_pc = pc_sel ? mem_address : pc + pc_inc;

    always @(posedge clk) begin
        if (rst) 
            pc <= 32'b0;
        else if (!stall) 
            pc <= next_pc;
    end

endmodule



module if_id_reg (
    input wire clk, rst, stall, flush, mem_stall, 
    input wire [31:0] pc_in, inst_in,
    output reg [31:0] pc_out, inst_out
);
    always @(posedge clk) begin
        if (rst) begin
            pc_out <= 32'b0;
            inst_out <= 32'h00000013;
        end else if (mem_stall) begin
            // Freeze
        end else if (flush) begin
            pc_out <= 32'b0;
            inst_out <= 32'h00000013;
        end else if (!stall) begin
            pc_out <= pc_in;
            inst_out <= inst_in;
        end 
    end    
endmodule

module id_ex_reg (
    input wire clk, rst, flush, mem_stall,
    input wire [31:0] pc_in, rs1_in, rs2_in, imm_in, 
    input wire [4:0] rd_in, 
    input wire reg_wen_in, mem_rw_in, a_sel_in, b_sel_in, 
    input wire [1:0] wb_sel_in, 
    input wire [3:0] alu_sel_in,
    input wire [31:0] inst_in,
    input wire is_lr_in, is_sc_in, is_amo_in, 
    input wire [4:0] atomic_op_in,
    output reg is_lr_out, is_sc_out, is_amo_out, 
    output reg [4:0] atomic_op_out,
    output reg [31:0] pc_out, rs1_out, rs2_out, imm_out,
    output reg [4:0] rd_out,
    output reg reg_wen_out, mem_rw_out, a_sel_out, b_sel_out, 
    output reg [1:0] wb_sel_out, 
    output reg [3:0] alu_sel_out, 
    output reg [31:0] inst_out
); 

    always @(posedge clk) begin
        if (rst) begin
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
        end else if (mem_stall) begin
            // Freeze 
        end else if (flush) begin
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
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            atomic_op_out <= 5'b0;
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
            is_lr_out <= is_lr_in;
            is_sc_out <= is_sc_in;
            is_amo_out <= is_amo_in;
            atomic_op_out <= atomic_op_in;
        end 
    end

endmodule


module ex_mem_reg (
    input wire clk, rst, mem_stall,
    input wire [31:0] alu_res_in, rs2_in, inst_in, pc_in,
    input wire [4:0] rd_in,
    input wire reg_wen_in, mem_rw_in,
    input wire [1:0] wb_sel_in,
    input wire is_lr_in, is_sc_in, is_amo_in, 
    input wire [4:0] atomic_op_in,
    output reg is_lr_out, is_sc_out, is_amo_out,
    output reg [4:0] atomic_op_out,
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
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            atomic_op_out <= 5'b0;    
        end else if (mem_stall) begin
            // Freeze
        end else begin
            alu_res_out <= alu_res_in;
            rs2_out <= rs2_in;
            inst_out <= inst_in;
            rd_out <= rd_in;
            reg_wen_out <= reg_wen_in;
            mem_rw_out <= mem_rw_in;
            wb_sel_out <= wb_sel_in;
            pc_out <= pc_in;   
            is_lr_out <= is_lr_in;
            is_sc_out <= is_sc_in;
            is_amo_out <= is_amo_in;
            atomic_op_out <= atomic_op_in;
        end
    end

endmodule


module mem_wb_reg (
    input wire clk, rst, mem_stall,
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
        end else if (mem_stall) begin
            // Freeze 
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