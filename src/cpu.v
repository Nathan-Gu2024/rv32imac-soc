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
`include "../src/rvc_expansion.v"
`include "../src/reservation_monitor.v"
`include "../src/csr_file.v"
`include "../src/trap_controller.v"
`include "../src/clint_timer.v"
`include "../src/dcache.v"
`include "../src/icache.v"
`include "../src/cache_core.v"
`include "../src/tcm.v"

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

    // D-cache lower-memory line interface (fake line memory now, AXI adapter later)
    output wire dcache_mem_req_valid,
    output wire dcache_mem_req_write,
    output wire [127:0] dcache_mem_wline,
    input wire [127:0] dcache_mem_read_data_block,
    input wire dcache_mem_ready
);
    // Control / stalls
    wire stall;
    wire global_mem_stall;

    // IF
    wire [31:0] pc;
    wire [31:0] if_inst;
    wire [31:0] raw_inst;
    wire [31:0] inst_expanded;
    wire [31:0] final_inst;
    wire [31:0] muxed_if_inst;
    wire [31:0] pc_inc;
    wire [31:0] actual_jump_target;
    wire [1:0] opcode_check;
    wire is_32_bit_opcode;
    wire unaligned_32_bit_fetch;
    wire is_compressed;
    wire actual_pc_sel;
    reg [15:0] fetch_buffer;
    reg buffer_valid;

    // icache
    wire cache_ready;
    wire icache_valid;
    wire imem_stall;

    // IF/ID
    wire [31:0] if_id_pc;
    wire [31:0] if_id_inst;

    // ID
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    wire [31:0] imm;
    wire pc_sel;
    wire reg_wen;
    wire a_sel;
    wire b_sel;
    wire mem_rw;
    wire is_lr;
    wire is_sc;
    wire is_amo;
    wire [1:0] wb_sel;
    wire [2:0] imm_sel;
    wire [3:0] alu_sel;
    wire [4:0] atomic_op;

    // ID/EX
    wire [31:0] id_ex_pc;
    wire [31:0] id_ex_rs1;
    wire [31:0] id_ex_rs2;
    wire [31:0] id_ex_imm;
    wire [31:0] id_ex_inst;
    wire [4:0] id_ex_rd;
    wire [4:0] id_ex_atomic_op;
    wire id_ex_reg_wen;
    wire id_ex_mem_rw;
    wire id_ex_a_sel;
    wire id_ex_b_sel;
    wire id_ex_is_lr;
    wire id_ex_is_sc;
    wire id_ex_is_amo;
    wire [1:0] id_ex_wb_sel;
    wire [3:0] id_ex_alu_sel;

    // EX
    wire [31:0] alu_a;
    wire [31:0] alu_b;
    wire [31:0] alu_out;
    wire [31:0] fwd_rs1;
    wire [31:0] fwd_rs2;
    wire [31:0] ex_mem_forward_data;
    wire [31:0] csr_rdata;
    wire [31:0] mtvec_out;
    wire [31:0] mepc_out;
    wire [31:0] trap_cause;
    wire [31:0] trap_pc;
    wire [31:0] trap_target_pc;
    wire [31:0] actual_ex_result;
    wire [6:0] ex_opcode;
    wire [2:0] ex_funct3;
    wire [1:0] fwd_a;
    wire [1:0] fwd_b;
    wire id_ex_is_branch;
    wire id_ex_is_beq;
    wire id_ex_is_bne;
    wire id_ex_is_blt;
    wire id_ex_is_bge;
    wire id_ex_is_bltu;
    wire id_ex_is_bgeu;
    wire id_ex_is_jal;
    wire id_ex_is_jalr;
    wire id_ex_br_eq;
    wire id_ex_br_lt;
    wire ex_is_csrrw;
    wire ex_csr_wen;
    wire timer_interrupt;
    wire gated_interrupt;
    wire trap_taken;
    wire mret_exec;
    wire flush_if;
    wire flush_id;
    wire flush_ex;
    wire pc_trap_override;
    reg mie;

    // EX/MEM
    wire [31:0] ex_mem_alu;
    wire [31:0] ex_mem_rs2;
    wire [31:0] ex_mem_inst;
    wire [31:0] ex_mem_pc;
    wire [4:0] ex_mem_rd;
    wire [4:0] ex_mem_atomic_op;
    wire ex_mem_reg_wen;
    wire ex_mem_mem_rw;
    wire ex_mem_is_lr;
    wire ex_mem_is_sc;
    wire ex_mem_is_amo;
    wire [1:0] ex_mem_wb_sel;

    // MEM
    wire [31:0] dcache_read_data;
    wire [31:0] final_mem_read_data;
    wire [31:0] partial_load_out;
    wire [31:0] final_alu_to_wb;
    wire [31:0] clint_rdata;
    wire [31:0] dcache_mem_req_addr;
    wire [3:0] raw_write_mask;
    wire is_mmio;
    wire is_led;
    wire is_uart;
    wire is_clint;
    wire is_load;
    wire is_store;
    wire store_commits;
    wire dcache_valid;
    wire dcache_ren;
    wire dcache_wen;
    wire dcache_ready;
    wire dmem_stall;
    wire lr_reservation_set;
    wire sc_reservation_clear;
    wire normal_store_reservation_clear;
    wire sc_success_flag;
    wire block_sc_store;
    wire clint_wen;

    // MEM/WB 
    wire [31:0] mem_wb_alu;
    wire [31:0] mem_wb_memdata;
    wire [31:0] mem_wb_pc;
    wire [31:0] mem_wb_inst;
    wire [4:0] mem_wb_rd;
    wire mem_wb_reg_wen;
    wire [1:0] mem_wb_wb_sel;

    // WB 
    wire [31:0] wb_data;

    // Cache / memory stall control
    localparam [31:0] LED_ADDR = 32'h0000_2000;
    localparam [31:0] UART_ADDR = 32'h0000_3000;
    localparam [31:0] CLINT_BASE = 32'h0200_0000;
    localparam [31:0] CLINT_MASK = 32'hFFFF_0000;
    localparam [31:0] TCM_BASE = 32'h4000_0000;
    localparam [31:0] TCM_BYTES = 32'h0001_0000;

    assign icache_valid = 1'b1;
    assign imem_stall = icache_valid & ~cache_ready;

    // Keep MMIO exact. Do not decode all 0x4xxxxxxx as MMIO because that
    // collides with the TCM region at 0x4000_0000.
    assign is_led   = (ex_mem_alu == LED_ADDR);
    assign is_uart  = (ex_mem_alu == UART_ADDR);
    assign is_clint = ((ex_mem_alu & CLINT_MASK) == CLINT_BASE);
    assign is_mmio  = is_led | is_uart | is_clint;

    assign is_load = (ex_mem_wb_sel == 2'b00) && ex_mem_reg_wen;
    assign is_store = ex_mem_mem_rw;
    assign store_commits = is_store && (~ex_mem_is_sc || sc_success_flag);

    // TCM is intentionally not MMIO here. TCM accesses go into dcache.v,
    // which bypasses the cache and forwards them to tcm.v.
    assign dcache_ren = is_load & ~is_mmio;
    assign dcache_wen = store_commits & ~is_mmio;
    assign dcache_valid = dcache_ren || dcache_wen;
    assign dmem_stall = dcache_valid && !dcache_ready;
    assign global_mem_stall = dmem_stall | imem_stall;

    // TCM
    wire tcm_i_req, tcm_i_ready;
    wire [31:0] tcm_i_addr, tcm_i_rdata;
    
    wire tcm_d_req, tcm_d_we, tcm_d_ready;
    wire [31:0] tcm_d_addr, tcm_d_wdata, tcm_d_rdata;
    wire [3:0] tcm_d_wmask;

    tcm #(
        .ADDR_WIDTH(32),
        .TCM_BASE(TCM_BASE),
        .TCM_BYTES(TCM_BYTES)
    ) TCM (
        .clk(clk), 
        .rst(rst), 

        .i_req(tcm_i_req), 
        .i_addr(tcm_i_addr), 
        .i_rdata(tcm_i_rdata), 
        .i_ready(tcm_i_ready), 

        .d_req(tcm_d_req), 
        .d_we(tcm_d_we), 
        .d_addr(tcm_d_addr), 
        .d_wdata(tcm_d_wdata), 
        .d_wmask(tcm_d_wmask), 
        .d_rdata(tcm_d_rdata), 
        .d_ready(tcm_d_ready)
    ); 

    icache #(
        .ADDR_WIDTH(32), 
        .LINE_BYTES(16), 
        .NUM_SETS(64), 
        .NUM_WAYS(2), 
        .TCM_BASE(TCM_BASE), 
        .TCM_BYTES(TCM_BYTES)
    ) ICACHE (
        .clk(clk), 
        .rst(rst),
        
        .cpu_req_valid(icache_valid), 
        .cpu_req_addr(pc), 
        .cpu_rdata(if_inst), 
        .cpu_ready(cache_ready), 

        .tcm_req_valid(tcm_i_req), 
        .tcm_req_addr(tcm_i_addr), 
        .tcm_rdata(tcm_i_rdata), 
        .tcm_ready(tcm_i_ready),

        .mem_req_valid(icache_mem_req_valid), 
        .mem_req_addr(icache_mem_req_addr), 
        .mem_rline(icache_mem_read_data), 
        .mem_ready(icache_mem_ready)
    ); 

    dcache #(
        .ADDR_WIDTH(32), 
        .LINE_BYTES(16), 
        .NUM_SETS(64), 
        .NUM_WAYS(2), 
        .TCM_BASE(TCM_BASE), 
        .TCM_BYTES(TCM_BYTES)
    ) DCACHE (
        .clk(clk), 
        .rst(rst),

        .cpu_req_valid(dcache_valid), 
        .cpu_req_write(dcache_wen), 
        .cpu_req_addr(ex_mem_alu), 
        .cpu_wdata(store_data), 
        .cpu_wmask(mem_write_mask),
        .cpu_rdata(dcache_read_data), 
        .cpu_ready(dcache_ready), 

        .tcm_req_valid(tcm_d_req),
        .tcm_req_write(tcm_d_we), 
        .tcm_req_addr(tcm_d_addr), 
        .tcm_wdata(tcm_d_wdata), 
        .tcm_wmask(tcm_d_wmask),
        .tcm_rdata(tcm_d_rdata), 
        .tcm_ready(tcm_d_ready), 

        .mem_req_valid(dcache_mem_req_valid), 
        .mem_req_write(dcache_mem_req_write), 
        .mem_req_addr(dcache_mem_req_addr), 
        .mem_wline(dcache_mem_wline), 
        .mem_rline(dcache_mem_read_data_block), 
        .mem_ready(dcache_mem_ready)
    ); 


    // direct_mapped_cache ICACHE (
    //     .clk(clk), 
    //     .rst(rst), 
    //     .cpu_req_addr(pc),
    //     .cpu_write_data(32'b0), 
    //     .cpu_read_req(1'b1),        
    //     .cpu_write_req(1'b0), 
    //     .mem_write_mask(4'b0000),
    //     .mem_ready(icache_mem_ready), 
    //     .mem_read_data(icache_mem_read_data), 
    //     .cpu_read_data(if_inst), 
    //     .mem_req_addr(icache_mem_req_addr), 
    //     .cpu_ready(cache_ready), 
    //     .mem_req_valid(icache_mem_req_valid)
    // ); 

    // direct_mapped_cache DCACHE (
    //     .clk(clk),
    //     .rst(rst),
    //     .cpu_req_addr(ex_mem_alu),  
    //     .cpu_write_data(store_data), 
    //     .mem_write_mask(mem_write_mask),
    //     .cpu_read_req(dcache_ren),      
    //     .cpu_write_req(dcache_wen),    
    //     .mem_ready(dcache_mem_ready),
    //     .mem_read_data(dcache_mem_read_data_block),        
    //     .cpu_read_data(dcache_read_data),
    //     .mem_req_addr(dcache_mem_req_addr),
    //     .cpu_ready(dcache_ready),         
    //     .mem_req_valid(dcache_mem_req_valid)
    // );

    // assign dmem_req_addr = dcache_mem_req_addr;
    

    // IF 
    assign opcode_check = pc[1] ? if_inst[17:16] : if_inst[1:0];
    assign is_32_bit_opcode = (opcode_check == 2'b11);
    assign unaligned_32_bit_fetch = (pc[1] == 1'b1) && is_32_bit_opcode && !buffer_valid;
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
    assign raw_inst = buffer_valid ? {if_inst[15:0], fetch_buffer} :
                      (pc[1] ? {16'b0, if_inst[31:16]} : if_inst);

    rvc_expand RVC (
        .inst_c(raw_inst[15:0]), 
        .inst_expanded(inst_expanded), 
        .is_compressed(is_compressed)
    );

    // Pipeline routing
    assign final_inst = is_compressed ? inst_expanded : raw_inst;
    assign muxed_if_inst = unaligned_32_bit_fetch ? 32'h00000013 : final_inst;
    assign pc_inc = (unaligned_32_bit_fetch || buffer_valid || is_compressed) ? 32'd2 : 32'd4;
    assign actual_pc_sel = pc_trap_override | pc_sel;
    assign actual_jump_target = pc_trap_override ? trap_target_pc : alu_out;
    program_counter PC (
        .clk(clk), 
        .rst(rst),
        .stall(stall | global_mem_stall), 
        .pc_sel(actual_pc_sel), 
        .mem_address(actual_jump_target), 
        .pc_inc(pc_inc),
        .pc(pc)
    );

    if_id_reg IF_ID (
        .clk(clk), 
        .rst(rst), 
        .stall(stall), 
        .mem_stall(global_mem_stall),
        .flush(pc_sel | flush_if), 
        .pc_in(pc), 
        .inst_in(muxed_if_inst), 
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
        .wb_sel(wb_sel),
        .out_is_lr(is_lr),
        .out_is_sc(is_sc),
        .out_is_amo(is_amo),
        .out_atomic_op(atomic_op),
        .csr_wen(csr_wen)
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
        .flush(pc_sel || stall || flush_id), 
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
    // SC writes 0/1 to rd
    // JAL/JALR write PC+4
    assign ex_mem_forward_data = ex_mem_is_sc ? final_alu_to_wb :
                                 (ex_mem_wb_sel == 2'b10) ? (ex_mem_pc + 32'd4) :
                                 ex_mem_alu;

    assign fwd_rs1 = (fwd_a == 2'b01) ? ex_mem_forward_data :
                    (fwd_a == 2'b10) ? wb_data : 
                    id_ex_rs1;

    assign fwd_rs2 = (fwd_b == 2'b01) ? ex_mem_forward_data :
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

    assign ex_opcode = id_ex_inst[6:0];
    assign ex_funct3 = id_ex_inst[14:12];

    assign id_ex_is_branch = (ex_opcode == 7'b1100011);
    assign id_ex_is_beq = id_ex_is_branch && (ex_funct3 == 3'b000);
    assign id_ex_is_bne = id_ex_is_branch && (ex_funct3 == 3'b001);
    assign id_ex_is_blt = id_ex_is_branch && (ex_funct3 == 3'b100);
    assign id_ex_is_bge = id_ex_is_branch && (ex_funct3 == 3'b101);
    assign id_ex_is_bltu = id_ex_is_branch && (ex_funct3 == 3'b110);
    assign id_ex_is_bgeu = id_ex_is_branch && (ex_funct3 == 3'b111);
    assign id_ex_is_jal = (ex_opcode == 7'b1101111);
    assign id_ex_is_jalr = (ex_opcode == 7'b1100111);

    branch_comp BC (
        .br_data1(fwd_rs1),
        .br_data2(fwd_rs2),
        .br_un(ex_funct3[1]),
        .br_eq(id_ex_br_eq),
        .br_lt(id_ex_br_lt)
    );

    assign pc_sel = (id_ex_br_eq & id_ex_is_beq) |
                (~id_ex_br_eq & id_ex_is_bne) |
                (id_ex_br_lt & (id_ex_is_blt | id_ex_is_bltu)) |
                (~id_ex_br_lt & (id_ex_is_bge | id_ex_is_bgeu)) |
                id_ex_is_jal | id_ex_is_jalr;

    wire id_ex_mem_read = (id_ex_wb_sel == 2'b00) && id_ex_reg_wen;
    hazard_unit HU (
        .id_ex_mem_read(id_ex_mem_read),
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

    // Machine Interrupt Enable (MIE)
    always @(posedge clk) begin
        if (rst) 
            mie <= 1'b1;
        else if (trap_taken) 
            mie <= 1'b0; // Disable interrupts inside the OS kernel
        else if (mret_exec) 
            mie <= 1'b1;  // Re-enable when returning to user code
    end

    assign gated_interrupt = timer_interrupt & mie & ~global_mem_stall & ~stall;
    trap_controller TRAP_CTRL (
        .ex_pc(id_ex_pc),
        .ex_inst(id_ex_inst),
        .external_interrupt(gated_interrupt),
        .mtvec_out(mtvec_out),
        .mepc_out(mepc_out),
        .trap_taken(trap_taken),
        .trap_cause(trap_cause),
        .trap_pc(trap_pc),
        .mret_exec(mret_exec),
        .flush_if(flush_if),
        .flush_id(flush_id),
        .flush_ex(flush_ex),
        .pc_trap_override(pc_trap_override),
        .trap_target_pc(trap_target_pc)
    );

    assign ex_is_csrrw = (id_ex_inst[6:0] == 7'b1110011) && (id_ex_inst[14:12] == 3'b001);
    assign ex_csr_wen = ex_is_csrrw && !global_mem_stall && !stall;
    csr_file CSR (
        .clk(clk), 
        .rst(rst), 
        .csr_addr(id_ex_inst[31:20]), 
        .csr_wdata(fwd_rs1),
        .csr_wen(ex_csr_wen), 
        .csr_rdata(csr_rdata),
        .trap_taken(trap_taken),
        .trap_pc(trap_pc),
        .trap_cause(trap_cause),
        .mret_exec(mret_exec),
        .mtvec_out(mtvec_out),
        .mepc_out(mepc_out)
    ); 

    assign actual_ex_result = ex_is_csrrw ? csr_rdata : alu_out;
    ex_mem_reg EX_MEM (
        .clk(clk), 
        .rst(rst), 
        .mem_stall(global_mem_stall),
        .alu_res_in(actual_ex_result), 
        .rs2_in(fwd_rs2), 
        .inst_in(flush_ex ? 32'h00000013 : id_ex_inst), 
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

    always @(posedge clk) begin
        if (rst) begin
            leds <= 4'b0;
            uart_tx_start <= 1'b0;
            uart_tx_data <= 8'b0;
        end else begin
            uart_tx_start <= 1'b0;

            // Gate MMIO side effects so stores do not repeat while the pipeline is frozen.
            if (!global_mem_stall && store_commits) begin
                if (is_led) begin
                    leds <= ex_mem_rs2[3:0];
                end else if (is_uart && uart_tx_ready) begin
                    uart_tx_data <= ex_mem_rs2[7:0];
                    uart_tx_start <= 1'b1;
                end
            end
        end 
    end

    // MEM
    partial_store PS (
        .inst(ex_mem_inst), 
        .mem_address(ex_mem_alu), 
        .data_from_reg(ex_mem_rs2), 
        .mem_rw(ex_mem_mem_rw), 
        .mem_write_mask(raw_write_mask), 
        .data_to_mem(store_data)
    ); 

    // LR/SC reservation bookkeeping should follow the MEM-stage operation,
    // not unrelated front-end stalls. LR may complete while the I-cache is
    // fetching the next line, so set the reservation when the D-cache load is ready
    // Keep SC clear gated by global_mem_stall so the combinational SC result
    // remains stable until the SC instruction can advance to WB
    assign lr_reservation_set = ex_mem_is_lr && dcache_ren && dcache_ready;
    assign sc_reservation_clear = ex_mem_is_sc && ~global_mem_stall;
    assign normal_store_reservation_clear = is_store && !ex_mem_is_sc &&
                                             (is_mmio || (dcache_wen && dcache_ready));

    reservation_monitor RM (
        .clk(clk),
        .rst(rst),
        .lr_en(lr_reservation_set),
        .sc_en(sc_reservation_clear),
        .any_store_en(normal_store_reservation_clear),
        .trap_taken(trap_taken),
        .mem_addr(ex_mem_alu),
        .sc_successful(sc_success_flag)
    );

    assign block_sc_store = ex_mem_is_sc & ~sc_success_flag;
    assign mem_write_mask = block_sc_store ? 4'b0000 : raw_write_mask;
    assign final_alu_to_wb = ex_mem_is_sc ?
                             (sc_success_flag ? 32'd0 : 32'd1) :
                             ex_mem_alu;

    assign clint_wen = store_commits && is_clint && !global_mem_stall;
    clint_timer CLINT (
        .clk(clk),
        .rst(rst),
        .addr(ex_mem_alu),
        .wdata(ex_mem_rs2),
        .wen(clint_wen),
        .rdata(clint_rdata),
        .timer_interrupt(timer_interrupt)
    );

    assign final_mem_read_data = is_clint ? clint_rdata :
                                 is_led ? {28'b0, leds} :
                                 is_uart ? 32'b0 :
                                            dcache_read_data;
    mem_wb_reg MEM_WB (
        .clk(clk),
        .rst(rst), 
        .mem_stall(global_mem_stall),
        .inst_in(ex_mem_inst),
        .alu_res_in(final_alu_to_wb), 
        .mem_data_in(final_mem_read_data), 
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
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            atomic_op_out <= 5'b0;
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
