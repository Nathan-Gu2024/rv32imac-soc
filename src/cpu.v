`include "../src/regfile.v"
`include "../src/alu.v"
`include "../src/div_unit.v"
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
`include "../src/uart_tx.v"
`include "../src/uart_rx.v"
`include "../src/uart_mmio.v"
`include "../src/intc.v"

module cpu_pipelined (
    input wire clk, rst,
    output wire uart_tx,
    input wire uart_rx,
    output reg [3:0] leds,

    output wire [31:0] icache_mem_req_addr, 
    output wire icache_mem_req_valid, 
    input wire [127:0] icache_mem_read_data, 
    input wire icache_mem_ready, 
    
    output wire [31:0] dmem_req_addr, 
    output wire [31:0] store_data, 
    output wire [3:0] mem_write_mask,

    // D-cache lower-memory line interface
    output wire dcache_mem_req_valid,
    output wire dcache_mem_req_write,
    output wire [127:0] dcache_mem_wline,
    input wire [127:0] dcache_mem_read_data_block,
    input wire dcache_mem_ready,
    
    // debug
    output wire [31:0] debug_pc, 
    output wire [31:0] debug_instr, 
    output wire debug_dcache_valid,
    output wire debug_dcache_ready,
    output wire debug_tcm_d_req,
    output wire debug_tcm_d_ready,
    output wire debug_global_mem_stall
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
    wire is_compressed;
    wire actual_pc_sel;

    // icache
    wire cache_ready;
    wire icache_valid;
    wire imem_stall;

    // IF/ID
    wire [31:0] if_id_pc;
    wire [31:0] if_id_inst;
    wire if_id_compressed;
    wire if_id_valid;

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
    wire [4:0] alu_sel;
    wire [4:0] atomic_op;
    wire csr_wen; 
    wire [1:0] csr_op; 
    wire csr_use_imm; 
    wire [4:0] csr_uimm; 

    // ID/EX
    wire id_ex_compressed;
    wire id_ex_valid;
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
    wire [4:0] id_ex_alu_sel;
    wire id_ex_csr_wen; 
    wire [1:0] id_ex_csr_op; 
    wire id_ex_csr_use_imm; 
    wire [4:0] id_ex_csr_uimm;
 
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
    wire mstatus_mie;

    // EX/MEM
    wire ex_mem_compressed;
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
    wire is_intc;
    wire uart_ready;
    wire [31:0] uart_rdata;
    wire uart_tx_irq;
    wire uart_rx_irq;
    wire intc_ready;
    wire [31:0] intc_rdata;
    wire intc_irq_out;
    wire mie_mtie, mie_meie;
    wire timer_fires, external_fires;
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
    wire mem_wb_compressed;
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
    localparam [31:0] INTC_MMIO_BASE = 32'h0000_4000;
    localparam [31:0] INTC_MMIO_MASK = 32'hFFFF_F000;
    localparam [31:0] UART_MMIO_BASE = 32'h4000_1000;
    localparam [31:0] UART_MMIO_MASK = 32'hFFFF_F000;
    localparam [31:0] CLINT_BASE = 32'h0200_0000;
    localparam [31:0] CLINT_MASK = 32'hFFFF_0000;
    localparam [31:0] TCM_BASE = 32'h4000_0000;
    localparam [31:0] TCM_BYTES = 32'h0001_0000;

    assign icache_valid = 1'b1;
    assign imem_stall = icache_valid & ~cache_ready;

    // Don't decode all 0x4xxxxxxx as MMIO because that
    // collides with the TCM region at 0x4000_0000
    // led/clint ack instantly, needs its own req/ready handshake
    assign is_led = (ex_mem_alu == LED_ADDR);
    assign is_uart = ((ex_mem_alu & UART_MMIO_MASK) == UART_MMIO_BASE);
    assign is_clint = ((ex_mem_alu & CLINT_MASK) == CLINT_BASE);
    assign is_intc = ((ex_mem_alu & INTC_MMIO_MASK) == INTC_MMIO_BASE);
    assign is_mmio = is_led | is_clint;

    assign is_load = (ex_mem_wb_sel == 2'b00) && ex_mem_reg_wen;
    assign is_store = ex_mem_mem_rw;
    assign store_commits = is_store && (~ex_mem_is_sc || sc_success_flag);

    // TCM accesses go into dcache.v which bypass the cache and forward to tcm.v
    assign dcache_ren = is_load & ~is_mmio & ~is_uart & ~is_intc;
    assign dcache_wen = store_commits & ~is_mmio & ~is_uart & ~is_intc;
    assign dcache_valid = dcache_ren || dcache_wen;
    assign dmem_stall = dcache_valid && !dcache_ready;

    wire uart_valid = is_uart & (is_load | store_commits);
    wire uart_req = uart_valid & ~uart_pending;
    reg uart_pending;
    always @(posedge clk) begin
        if (rst) uart_pending <= 1'b0;
        else if (uart_req) uart_pending <= 1'b1;
        else if (uart_ready) uart_pending <= 1'b0;
    end
    wire uart_stall = uart_valid && !uart_ready;

    // intc: same two-tier req/pending pattern as uart_mmio above - it also
    // has real registered latency, not an instant ack like led/clint.
    wire intc_valid = is_intc & (is_load | store_commits);
    wire intc_req = intc_valid & ~intc_pending;
    reg intc_pending;
    always @(posedge clk) begin
        if (rst) intc_pending <= 1'b0;
        else if (intc_req) intc_pending <= 1'b1;
        else if (intc_ready) intc_pending <= 1'b0;
    end
    wire intc_stall = intc_valid && !intc_ready;

    assign global_mem_stall = dmem_stall | imem_stall | div_stall | uart_stall | intc_stall;

    // TCM
    wire tcm_i_req, tcm_i_ready;
    wire [31:0] tcm_i_addr, tcm_i_rdata;
    
    wire tcm_d_req, tcm_d_we, tcm_d_ready;
    wire [31:0] tcm_d_addr, tcm_d_wdata, tcm_d_rdata;
    wire [3:0] tcm_d_wmask;

    tcm #(
        .ADDR_WIDTH(32),
        .TCM_BASE(TCM_BASE),
        .TCM_BYTES(TCM_BYTES),
        .INIT_FILE("C:/Users/natha/OneDrive/Desktop/RV32-5-stage-processor-main/fpga/main_cv7.mem")
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

    //debug 
    assign debug_dcache_valid = dcache_valid;
    assign debug_dcache_ready = dcache_ready;
    assign debug_tcm_d_req = tcm_d_req;
    assign debug_tcm_d_ready = tcm_d_ready;
    assign debug_global_mem_stall = global_mem_stall;    
    
    // IF
    assign raw_inst = if_inst;

    rvc_expand RVC (
        .inst_c(raw_inst[15:0]),
        .inst_expanded(inst_expanded),
        .is_compressed(is_compressed)
    );

    // Pipeline routing
    assign final_inst = is_compressed ? inst_expanded : raw_inst;
    assign muxed_if_inst = final_inst;
    assign pc_inc = is_compressed ? 32'd2 : 32'd4;
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
        .compressed_in(is_compressed),
        .pc_out(if_id_pc),
        .inst_out(if_id_inst),
        .compressed_out(if_id_compressed),
        .valid_out(if_id_valid)
    );
    
    assign debug_instr = if_id_inst;

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
        .csr_wen(csr_wen),
        .csr_op(csr_op), 
        .csr_use_imm(csr_use_imm),
        .csr_uimm(csr_uimm)
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
        .compressed_in(if_id_compressed),
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
        .csr_wen_in(csr_wen),
        .csr_op_in(csr_op),
        .csr_use_imm_in(csr_use_imm),
        .csr_uimm_in(csr_uimm),
        .valid_in(if_id_valid),
        .is_lr_out(id_ex_is_lr),
        .is_sc_out(id_ex_is_sc), 
        .is_amo_out(id_ex_is_amo), 
        .atomic_op_out(id_ex_atomic_op), 
        .csr_wen_out(id_ex_csr_wen),
        .csr_op_out(id_ex_csr_op),
        .csr_use_imm_out(id_ex_csr_use_imm),
        .csr_uimm_out(id_ex_csr_uimm),
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
        .alu_sel_out(id_ex_alu_sel),
        .compressed_out(id_ex_compressed),
        .valid_out(id_ex_valid)
    );

    // EX 
    // Forwarding
    // SC writes 0/1 to rd
    // JAL/JALR write PC+4, or PC+2 if the original instruction was compressed,
    // the link address must point at the next
    // real instruction, only 2 bytes after a compressed one
    assign ex_mem_forward_data = ex_mem_is_sc ? final_alu_to_wb :
                                 (ex_mem_wb_sel == 2'b10) ? (ex_mem_pc + (ex_mem_compressed ? 32'd2 : 32'd4)) :
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

    // Comb divide fails timing so this runs as a
    // ~33-cycle multi-cycle op that stalls the whole pipeline via global_mem_stall
    wire is_div_op = id_ex_alu_sel[4];
    wire div_is_signed = ~id_ex_alu_sel[0];
    wire div_want_rem = id_ex_alu_sel[1];
    wire div_busy, div_done;
    wire [31:0] div_quotient, div_remainder;
    reg div_result_ready;

    wire div_start = is_div_op && !div_busy && !div_done && !div_result_ready;
    wire div_stall = is_div_op && !div_result_ready;

    always @(posedge clk) begin
        if (rst) div_result_ready <= 1'b0;
        else if (div_done) div_result_ready <= 1'b1;
        else if (!is_div_op) div_result_ready <= 1'b0;
    end

    div_unit DIV (
        .clk(clk),
        .rst(rst),
        .start(div_start),
        .a(alu_a),
        .b(alu_b),
        .is_signed(div_is_signed),
        .busy(div_busy),
        .done(div_done),
        .quotient(div_quotient),
        .remainder(div_remainder)
    );

    wire [31:0] ex_result = is_div_op ? (div_want_rem ? div_remainder : div_quotient) : alu_out;

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

    // Machine Interrupt Enable comes straight from mstatus.MIE (csr_file),
    // which correctly resets to 0 and tracks trap/mret/software CSR writes.
    // Each class also needs its own mie bit (mie_mtie/mie_meie) and live
    // mip truth (timer_interrupt/intc_irq_out) before it's allowed to fire -
    // gated_interrupt is kept only as a combined debug/status signal.
    //
    // id_ex_valid additionally gates both: EX can be holding a
    // flush-inserted bubble (pc=0, inst=NOP) rather than a real instruction
    // - e.g. every taken branch/jump produces one for a cycle, including a
    // tight self-loop's own redirect. Taking an interrupt on such a cycle
    // would capture trap_pc=0 as the resume address instead of a real one,
    // so mret would later jump to address 0 and hang. Deferring by a cycle
    // until EX is valid again is safe since the interrupt condition stays
    // latched (pending in intc / mtime>=mtimecmp) rather than pulsing.
    assign timer_fires = mstatus_mie & mie_mtie & timer_interrupt & id_ex_valid & ~global_mem_stall & ~stall;
    assign external_fires = mstatus_mie & mie_meie & intc_irq_out & id_ex_valid & ~global_mem_stall & ~stall;
    assign gated_interrupt = timer_fires | external_fires;
    trap_controller TRAP_CTRL (
        .ex_pc(id_ex_pc),
        .ex_inst(id_ex_inst),
        .timer_irq(timer_fires),
        .external_irq(external_fires),
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

    assign ex_csr_wen = id_ex_csr_wen && !global_mem_stall && !stall;
    csr_file CSR (
        .clk(clk),
        .rst(rst),
        .csr_addr(id_ex_inst[31:20]),
        .csr_wdata(fwd_rs1),
        .csr_wen(ex_csr_wen),
        .csr_op(id_ex_csr_op),
        .csr_use_imm(id_ex_csr_use_imm),
        .csr_uimm(id_ex_csr_uimm),
        .csr_rdata(csr_rdata),
        .trap_taken(trap_taken),
        .trap_pc(trap_pc),
        .trap_cause(trap_cause),
        .mret_exec(mret_exec),
        .timer_pending(timer_interrupt),
        .external_pending(intc_irq_out),
        .mtvec_out(mtvec_out),
        .mepc_out(mepc_out),
        .mstatus_mie(mstatus_mie),
        .mie_mtie(mie_mtie),
        .mie_meie(mie_meie)
    );

    assign actual_ex_result = id_ex_csr_wen ? csr_rdata : ex_result;
    ex_mem_reg EX_MEM (
        .clk(clk), 
        .rst(rst), 
        .mem_stall(global_mem_stall),
        .alu_res_in(actual_ex_result), 
        .rs2_in(fwd_rs2), 
        .inst_in(flush_ex ? 32'h00000013 : id_ex_inst),
        .pc_in(id_ex_pc),
        .compressed_in(id_ex_compressed),
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
        .compressed_out(ex_mem_compressed),
        .rd_out(ex_mem_rd),
        .reg_wen_out(ex_mem_reg_wen),
        .mem_rw_out(ex_mem_mem_rw),
        .wb_sel_out(ex_mem_wb_sel)
    );

    always @(posedge clk) begin
        if (rst) begin
            leds <= 4'b0;
        end else begin
            // Gate MMIO side effects so stores do not repeat while the pipeline is frozen
            if (!global_mem_stall && store_commits) begin
                if (is_led) begin
                    leds <= ex_mem_rs2[3:0];
                end
            end
        end
    end

    // uart_mmio owns its own uart_tx instance and drives the physical tx
    // pin directly; its d_req is the pending-gated pulse computed above so
    // a stalled multi-cycle transaction doesn't re-trigger the write or
    // re-latch the status read every cycle
    uart_mmio UART (
        .clk(clk),
        .rst(rst),
        .d_req(uart_req),
        .d_we(store_commits),
        .d_addr(ex_mem_alu),
        .d_wdata(ex_mem_rs2),
        .d_rdata(uart_rdata),
        .d_ready(uart_ready),
        .tx(uart_tx),
        .rx(uart_rx),
        .tx_irq(uart_tx_irq), 
        .rx_irq(uart_rx_irq)
    );

    // intc: source 0 is UART TX-complete; sources 1-7 are reserved for
    // future peripherals (tie 0 until wired up).
    intc #(
        .NUM_SOURCES(8)
    ) INTC (
        .clk(clk),
        .rst(rst),
        .d_req(intc_req),
        .d_we(store_commits),
        .d_addr(ex_mem_alu),
        .d_wdata(ex_mem_rs2),
        .d_rdata(intc_rdata),
        .d_ready(intc_ready),
        .irq_in({6'b0, uart_rx_irq, uart_tx_irq}),
        .irq_out(intc_irq_out)
    );

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
                                             (is_mmio || (dcache_wen && dcache_ready) || (is_uart && uart_ready) || (is_intc && intc_ready));

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
                                 is_uart ? uart_rdata :
                                 is_intc ? intc_rdata :
                                            dcache_read_data;
    mem_wb_reg MEM_WB (
        .clk(clk),
        .rst(rst), 
        .mem_stall(global_mem_stall),
        .inst_in(ex_mem_inst),
        .alu_res_in(final_alu_to_wb), 
        .mem_data_in(final_mem_read_data),
        .pc_in(ex_mem_pc),
        .compressed_in(ex_mem_compressed),
        .rd_in(ex_mem_rd),
        .reg_wen_in(ex_mem_reg_wen),
        .wb_sel_in(ex_mem_wb_sel),
        .inst_out(mem_wb_inst),
        .alu_res_out(mem_wb_alu),
        .mem_data_out(mem_wb_memdata),
        .pc_out(mem_wb_pc),
        .compressed_out(mem_wb_compressed),
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

    // JAL/JALR link value: PC+4 normally, but PC+2 if the original
    // instruction was compressed (c.jal/c.jalr) - a compressed call is only
    // 2 bytes, so the return address must point 2 bytes past it, not 4.
    assign wb_data = (mem_wb_wb_sel == 2'b01) ? mem_wb_alu : // ALU
                    (mem_wb_wb_sel == 2'b00) ? partial_load_out : // MEM
                    (mem_wb_wb_sel == 2'b10) ? (mem_wb_pc + (mem_wb_compressed ? 32'd2 : 32'd4)) : // PC + 4/2
                    32'b0;

endmodule

module program_counter (
    input wire [31:0] mem_address, pc_inc, 
    input wire clk, rst, pc_sel, stall,
    output reg [31:0] pc
);
    wire [31:0] next_pc = pc_sel ? mem_address : pc + pc_inc;

    always @(posedge clk) begin
//        if (rst) 
//            pc <= 32'b0;
//        else if (!stall) 
//            pc <= next_pc;
        if (rst) 
            pc <= 32'h4000_0000;
        else if (!stall)
            pc <= next_pc;
    end

endmodule


module if_id_reg (
    input wire clk, rst, stall, flush, mem_stall,
    input wire [31:0] pc_in, inst_in,
    input wire compressed_in,
    output reg [31:0] pc_out, inst_out,
    output reg compressed_out,
    // 0 whenever this slot holds a flush-inserted bubble rather than a
    // genuinely fetched instruction (indistinguishable from a real NOP by
    // inst_out/pc_out alone - both read as 0x13/0 either way). Needed so
    // downstream logic (interrupt-taking) can tell the difference.
    output reg valid_out
);
    always @(posedge clk) begin
        if (rst) begin
            pc_out <= 32'b0;
            inst_out <= 32'h00000013;
            compressed_out <= 1'b0;
            valid_out <= 1'b0;
        end else if (mem_stall) begin
            // Freeze
        end else if (flush) begin
            pc_out <= 32'b0;
            inst_out <= 32'h00000013;
            compressed_out <= 1'b0;
            valid_out <= 1'b0;
        end else if (!stall) begin
            pc_out <= pc_in;
            inst_out <= inst_in;
            compressed_out <= compressed_in;
            valid_out <= 1'b1;
        end
    end
endmodule


module id_ex_reg (
    input wire clk, rst, flush, mem_stall,
    input wire [31:0] pc_in, rs1_in, rs2_in, imm_in,
    input wire [4:0] rd_in,
    input wire reg_wen_in, mem_rw_in, a_sel_in, b_sel_in,
    input wire [1:0] wb_sel_in,
    input wire [4:0] alu_sel_in,
    input wire [31:0] inst_in,
    input wire compressed_in,
    input wire is_lr_in, is_sc_in, is_amo_in,
    input wire [4:0] atomic_op_in,
    input wire csr_wen_in,
    input wire [1:0] csr_op_in,
    input wire csr_use_imm_in,
    input wire [4:0] csr_uimm_in,
    input wire valid_in,
    output reg is_lr_out, is_sc_out, is_amo_out,
    output reg [4:0] atomic_op_out,
    output reg [31:0] pc_out, rs1_out, rs2_out, imm_out,
    output reg [4:0] rd_out,
    output reg reg_wen_out, mem_rw_out, a_sel_out, b_sel_out,
    output reg [1:0] wb_sel_out,
    output reg [4:0] alu_sel_out,
    output reg [31:0] inst_out,
    output reg compressed_out,
    output reg csr_wen_out,
    output reg [1:0] csr_op_out,
    output reg csr_use_imm_out,
    output reg [4:0] csr_uimm_out,
    // See if_id_reg's valid_out for what this means; propagated forward
    // (not unconditionally set) so a bubble already flushed upstream stays
    // marked invalid all the way through EX.
    output reg valid_out
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
            compressed_out <= 1'b0;
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            atomic_op_out <= 5'b0;
            csr_wen_out <= 1'b0;
            csr_op_out <= 2'b0;
            csr_use_imm_out <= 1'b0;
            csr_uimm_out <= 5'b0;
            valid_out <= 1'b0;
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
            compressed_out <= 1'b0;
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            atomic_op_out <= 5'b0;
            csr_wen_out <= 1'b0;
            csr_op_out <= 2'b0;
            csr_use_imm_out <= 1'b0;
            csr_uimm_out <= 5'b0;
            valid_out <= 1'b0;
        end else begin
            mem_rw_out <= mem_rw_in;
            rd_out <= rd_in;
            pc_out <= pc_in;
            rs1_out <= rs1_in;
            rs2_out <= rs2_in;
            imm_out <= imm_in;
            reg_wen_out <= reg_wen_in;
            valid_out <= valid_in;
            a_sel_out <= a_sel_in;
            b_sel_out <= b_sel_in;
            wb_sel_out <= wb_sel_in;
            alu_sel_out <= alu_sel_in;
            inst_out <= inst_in;
            compressed_out <= compressed_in;
            is_lr_out <= is_lr_in;
            is_sc_out <= is_sc_in;
            is_amo_out <= is_amo_in;
            atomic_op_out <= atomic_op_in;
            csr_wen_out <= csr_wen_in;
            csr_op_out <= csr_op_in;
            csr_use_imm_out <= csr_use_imm_in;
            csr_uimm_out <= csr_uimm_in;
        end
    end

endmodule


module ex_mem_reg (
    input wire clk, rst, mem_stall,
    input wire [31:0] alu_res_in, rs2_in, inst_in, pc_in,
    input wire compressed_in,
    input wire [4:0] rd_in,
    input wire reg_wen_in, mem_rw_in,
    input wire [1:0] wb_sel_in,
    input wire is_lr_in, is_sc_in, is_amo_in,
    input wire [4:0] atomic_op_in,
    output reg is_lr_out, is_sc_out, is_amo_out,
    output reg [4:0] atomic_op_out,
    output reg [31:0] alu_res_out, rs2_out, inst_out, pc_out,
    output reg compressed_out,
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
            compressed_out <= 1'b0;
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
            compressed_out <= compressed_in;
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
    input wire compressed_in,
    input wire [4:0] rd_in,
    input wire reg_wen_in,
    input wire [1:0] wb_sel_in,
    output reg [31:0] alu_res_out, mem_data_out, pc_out, inst_out,
    output reg compressed_out,
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
            compressed_out <= 1'b0;
            wb_sel_out <= 0;
            inst_out <= 32'h00000013;
        end else if (mem_stall) begin
        // Freeze
        end else begin
            alu_res_out <= alu_res_in;
            mem_data_out <= mem_data_in;
            pc_out <= pc_in;
            compressed_out <= compressed_in;
            rd_out <= rd_in;
            reg_wen_out <= reg_wen_in;
            wb_sel_out <= wb_sel_in;
            inst_out <= inst_in;
        end
    end

endmodule
