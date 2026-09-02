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
`include "../src/icache_bram.v"
`include "../src/dcache_bram.v"
`include "../src/tcm.v"
`include "../src/uart_tx.v"
`include "../src/uart_rx.v"
`include "../src/uart_mmio.v"
`include "../src/intc.v"
`include "../src/axi_lite_bridge.v"
`include "../src/mm_accel.v"

// Memory sizing is parameterized because the FPGA and ASIC targets want
// very different points, from identical RTL.
//
// On FPGA these arrays infer Block RAM and are essentially free. An ASIC
// flow without SRAM macros has no equivalent - they synthesize as flip
// flops, and at these defaults that is roughly a million of them, some 35x
// the rest of the design. So the OpenLane flow overrides them downward via
// SYNTH_PARAMETERS in openlane/cpu/config.json rather than forking the RTL.
//
// Defaults are the full FPGA configuration on purpose: every testbench
// instantiates this module without overrides, so the verified numbers stay
// the ones you get by default.
module cpu_pipelined #(
    parameter IC_NUM_SETS = 2048,          // 2048 x 16B = 32KB I-cache
    parameter DC_NUM_SETS = 1024,          // 1024 x 16B = 16KB D-cache
    parameter TCM_BYTES   = 32'h0001_0000, // 64KB tightly-coupled memory
    // 1 for FPGA/simulation (TCM preloaded with the boot image from the
    // bitstream), 0 for ASIC, where there is no such mechanism and the
    // preload would cost ~TCM_WORDS x 32 flip-flops for nothing. Plumbed
    // from here because SYNTH_PARAMETERS only reaches the top module.
    parameter TCM_INIT_ENABLE = 1
) (
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

    // debug (ILA probes; debug_pc/debug_instr are the ID-stage pc/inst,
    // debug_raw_pc is one stage earlier - the raw IF-stage fetch)
    output wire [31:0] debug_pc,
    output wire [31:0] debug_instr,
    output wire debug_dcache_valid,
    output wire debug_dcache_ready,
    output wire debug_tcm_d_req,
    output wire debug_tcm_d_ready,
    output wire debug_global_mem_stall,
    output wire [31:0] debug_raw_pc,
    output wire debug_id_predicted_taken,
    output wire debug_cache_ready
);
    // Control / stalls
    wire stall;
    wire global_mem_stall;

    // IF
    wire [31:0] pc;
    // The value pc will hold next cycle - drives icache_bram's array
    // addresses so a cache hit costs no extra stall cycle.
    wire [31:0] pc_next;
    wire [31:0] if_inst;
    wire [31:0] raw_inst;
    wire [31:0] inst_expanded;
    wire [31:0] final_inst;
    wire [31:0] muxed_if_inst;
    wire [31:0] pc_inc;
    wire [31:0] actual_jump_target;
    wire is_compressed;
    wire actual_pc_sel;

    wire [31:0] ex_redirect_target;

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

    // Gshare dynamic branch prediction + RAS, computed in IF (see assigns
    // below) for a same-cycle redirect;
    // if_id_* are the IF-stage prediction relayed one stage forward
    // through if_id_reg (NOT recomputed in ID) for id_ex_reg to consume,
    // exactly like if_id_pc/if_id_inst already are.
    wire if_is_branch;
    wire if_predicted_taken;
    wire [31:0] if_predicted_target;
    wire [9:0] if_pht_index;
    wire if_id_predicted_taken;
    wire [31:0] if_id_predicted_target;
    wire [9:0] if_id_pht_index;
    wire [9:0] id_ex_pht_index;
    wire [9:0] ghr_before;

    // Return Address Stack (RAS): predicts JALR-return targets (push on
    // call, pop on ret). JAL is also redirected here since its target is
    // trivially PC+imm, same as a taken branch - see assigns below.
    wire if_predict_commit;
    wire if_is_jal;
    wire if_is_jalr;
    wire if_rd_is_link;
    wire if_rs1_is_link;
    wire if_ras_push_pattern;
    wire if_ras_pop_pattern;
    wire ras_empty;
    wire [31:0] ras_top;
    wire if_ras_hit;
    wire if_jal_taken;
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
    wire id_ex_predicted_taken;
    wire [31:0] id_ex_predicted_target;
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
    wire branch_actual_taken;
    wire branch_mispredicted;
    wire jalr_ras_mispredicted;
    wire [31:0] id_ex_pc_plus_inc;
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
    wire is_accel;
    wire uart_ready;
    wire [31:0] uart_rdata;
    wire uart_tx_irq;
    wire uart_rx_irq;
    wire intc_ready;
    wire [31:0] intc_rdata;
    wire intc_irq_out;
    wire accel_ready;
    wire [31:0] accel_rdata;
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
    // INTC/UART/CLINT masks below match each peripheral's REAL register
    // window (intc.v: ENABLE+PENDING, 8 bytes; uart_mmio.v: 4 word regs,
    // 16 bytes; clint_timer.v: addr[3:0], 16 bytes) - NOT a whole 4KB/64KB
    // page. They used to be page-wide, which silently misrouted any real
    // DDR-resident program data/stack that happened to land in that page
    // through the peripheral's own register logic instead of memory
    // (writes got acknowledged but went nowhere real; reads came back as
    // whatever the peripheral defaults its d_rdata/rdata to, usually 0) -
    // exactly the same class of bug as dcache.v's old page-wide MMIO
    localparam [31:0] LED_ADDR = 32'h0000_2000;
    localparam [31:0] INTC_MMIO_BASE = 32'h0000_4000;
    localparam [31:0] INTC_MMIO_MASK = 32'hFFFF_FFF8;
    localparam [31:0] UART_MMIO_BASE = 32'h4000_1000;
    localparam [31:0] UART_MMIO_MASK = 32'hFFFF_FFF0;
    localparam [31:0] CLINT_BASE = 32'h0200_0000;
    localparam [31:0] CLINT_MASK = 32'hFFFF_FFF0;
    localparam [31:0] ACCEL_MMIO_BASE = 32'h0000_5000;
    localparam [31:0] ACCEL_MMIO_MASK = 32'hFFFF_FF00;
    localparam [31:0] TCM_BASE = 32'h4000_0000;

    // Derived from the DC_NUM_SETS parameter rather than pinned, so the
    // EX-stage speculative address adder automatically narrows or widens
    // with the cache. Getting these out of sync would silently mis-index
    // the array (dcache_bram's spec_ok would catch it, but as a permanent
    // stall rather than a hit).
    localparam DC_IDX_BITS = $clog2(DC_NUM_SETS);
    localparam DC_OFF_BITS = 4;    // $clog2(LINE_BYTES = 16)

    assign icache_valid = 1'b1;
    assign imem_stall = icache_valid & ~cache_ready;

    // Narrow masks, not a whole 0x4xxxxxxx page match - that would collide
    // with the TCM region at 0x4000_0000. led/clint ack instantly; uart/intc
    // need their own req/ready handshake (see below).
    assign is_led = (ex_mem_alu == LED_ADDR);
    assign is_uart = ((ex_mem_alu & UART_MMIO_MASK) == UART_MMIO_BASE);
    assign is_clint = ((ex_mem_alu & CLINT_MASK) == CLINT_BASE);
    assign is_intc = ((ex_mem_alu & INTC_MMIO_MASK) == INTC_MMIO_BASE);
    assign is_accel = ((ex_mem_alu & ACCEL_MMIO_MASK) == ACCEL_MMIO_BASE);
    assign is_mmio = is_led | is_clint;

    // AMO's ROM entry shares wb_sel=00/reg_wen=1 with plain loads (it reuses
    // the load writeback path to return the old value) - exclude it here so
    // it doesn't also match is_load, which would otherwise make uart_valid/
    // intc_valid (which key off is_load directly) fire a spurious peripheral
    // read for an AMO targeting a UART/INTC address.
    assign is_load = (ex_mem_wb_sel == 2'b00) && ex_mem_reg_wen && ~ex_mem_is_amo;
    assign is_store = ex_mem_mem_rw;
    assign store_commits = is_store && (~ex_mem_is_sc || sc_success_flag);

    // No explicit TCM check needed here - dcache.v routes TCM-range
    // addresses to tcm.v internally, bypassing the cache transparently.
    wire mem_addr_is_cacheable = ~is_mmio & ~is_uart & ~is_intc & ~is_accel;
    assign dcache_ren = (is_load | amo_read_phase) & mem_addr_is_cacheable;
    assign dcache_wen = (store_commits | amo_write_phase) & mem_addr_is_cacheable;
    assign dcache_valid = dcache_ren || dcache_wen;
    assign dmem_stall = dcache_valid && !dcache_ready;

    // uart_ready/intc_ready below are one-shot registered pulses (the
    // peripheral's own d_ready, high for exactly one cycle after d_req),
    // unlike dcache_ready which is combinational/level-sensitive and
    // simply stays high as long as the same address keeps hitting. That
    // difference used to cause a livelock: uart_pending/intc_pending
    // would clear the instant the one-shot ready pulse fired, and since
    // uart_valid/intc_valid (derived from the still-frozen ex_mem stage)
    // stays asserted for as long as the pipeline is stalled for ANY
    // other reason too (e.g. a concurrent multi-cycle icache miss), a
    // fresh req/ready cycle would immediately re-fire - so uart_stall/
    // intc_stall kept re-asserting on whichever cycle happened to be out
    // of phase with the other stall source, and global_mem_stall (an OR
    // of all sources) never saw every source clear on the same cycle.
    // uart_done/intc_done latch "this exact access has already been
    // acknowledged" across that re-arming, and only release once the
    // pipeline genuinely retires (global_mem_stall actually drops),
    // matching how dcache_ready naturally persists.
    // uart_rdata/intc_rdata (the peripheral's own d_rdata) are ALSO only
    // valid for that same one cycle uart_ready/intc_ready pulses - but
    // uart_done/intc_done deliberately keep the pipeline stalled for one
    // MORE cycle after that (see above), so by the time the pipeline
    // actually commits, the live uart_rdata/intc_rdata wire has already
    // fallen back to the peripheral's default (0) for a load. Latch the
    // data at the exact same moment as the done flag so it survives to
    // the delayed commit - final_mem_read_data below reads THESE, not the
    // live uart_rdata/intc_rdata wires, for is_uart/is_intc loads.
    reg [31:0] uart_rdata_latched;
    reg [31:0] intc_rdata_latched;
    reg [31:0] accel_rdata_latched;

    wire uart_valid = is_uart & (is_load | store_commits);
    wire uart_req = uart_valid & ~uart_pending & ~uart_done;
    reg uart_pending;
    reg uart_done;
    always @(posedge clk) begin
        if (rst) begin
            uart_pending <= 1'b0;
            uart_done <= 1'b0;
            uart_rdata_latched <= 32'b0;
        end else begin
            if (uart_req) uart_pending <= 1'b1;
            else if (uart_ready) uart_pending <= 1'b0;

            if (uart_ready) begin
                uart_done <= 1'b1;
                uart_rdata_latched <= uart_rdata;
            end else if (!global_mem_stall) uart_done <= 1'b0;
        end
    end
    wire uart_stall = uart_valid && !uart_done;

    // intc: same two-tier req/pending pattern as uart_mmio above
    // has real registered latency, not instant ack like led/clint.
    wire intc_valid = is_intc & (is_load | store_commits);
    wire intc_req = intc_valid & ~intc_pending & ~intc_done;
    reg intc_pending;
    reg intc_done;
    always @(posedge clk) begin
        if (rst) begin
            intc_pending <= 1'b0;
            intc_done <= 1'b0;
            intc_rdata_latched <= 32'b0;
        end else begin
            if (intc_req) intc_pending <= 1'b1;
            else if (intc_ready) intc_pending <= 1'b0;

            if (intc_ready) begin
                intc_done <= 1'b1;
                intc_rdata_latched <= intc_rdata;
            end else if (!global_mem_stall) intc_done <= 1'b0;
        end
    end
    wire intc_stall = intc_valid && !intc_done;

    // accel: same two-tier req/pending pattern as uart/intc above. Its
    // d_ready comes from axi_lite_bridge and is a one-shot pulse just like
    // uart/intc's, even though internally it takes several cycles longer
    // (a full AXI4-Lite AW+W+B or AR+R handshake) - this pattern already
    // tolerates arbitrary internal latency, proven by uart/intc.
    wire accel_valid = is_accel & (is_load | store_commits);
    wire accel_req = accel_valid & ~accel_pending & ~accel_done;
    reg accel_pending;
    reg accel_done;
    always @(posedge clk) begin
        if (rst) begin
            accel_pending <= 1'b0;
            accel_done <= 1'b0;
            accel_rdata_latched <= 32'b0;
        end else begin
            if (accel_req) accel_pending <= 1'b1;
            else if (accel_ready) accel_pending <= 1'b0;

            if (accel_ready) begin
                accel_done <= 1'b1;
                accel_rdata_latched <= accel_rdata;
            end else if (!global_mem_stall) accel_done <= 1'b0;
        end
    end
    wire accel_stall = accel_valid && !accel_done;

    assign global_mem_stall = dmem_stall | imem_stall | div_stall | uart_stall | intc_stall | amo_stall | accel_stall;

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
        // Relative to src/ (matches the `include convention used throughout
        // this file) so $readmemh resolves correctly regardless of build
        // environment - was a machine-specific absolute Windows path,
        // which is invisible to iverilog on WSL and to the OpenLane Docker
        // container alike (neither has C:/Users/... mounted).
        .INIT_FILE("../fpga/ddr_fixed.mem"),
        .INIT_ENABLE(TCM_INIT_ENABLE)
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
        // Direct-mapped, in real Block RAM (see icache_bram.v). At the
        // default 2048 sets that is 32KB - enough to hold CoreMark's whole
        // ~23KB .text+.rodata, so steady-state instruction misses go to
        // ~zero for ~10 of the ~124 free BRAM tiles. The old cache_core
        // version was stuck at 2KB because its arrays were read
        // combinationally and could only be distributed LUTRAM.
        .NUM_SETS(IC_NUM_SETS),
        .TCM_BASE(TCM_BASE),
        .TCM_BYTES(TCM_BYTES)
    ) ICACHE (
        .clk(clk),
        .rst(rst),

        .cpu_req_valid(icache_valid),
        .cpu_req_addr(pc),
        .cpu_req_addr_next(pc_next),
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
        .NUM_SETS(DC_NUM_SETS),
        .TCM_BASE(TCM_BASE),
        .TCM_BYTES(TCM_BYTES)
    ) DCACHE (
        .clk(clk),
        .rst(rst),

        .cpu_req_valid(dcache_valid),
        .cpu_req_write(dcache_wen),
        .cpu_req_addr(ex_mem_alu),
        .cpu_req_addr_next(dc_addr_next),
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

    assign dmem_req_addr = dcache_mem_req_addr;

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

    // Gshare dynamic branch prediction + Return Address Stack, computed
    // HERE in IF rather than ID, giving a true same-cycle redirect: zero
    // bubbles for a correctly-predicted taken branch / JAL / RAS-hit JALR,
    // versus the one-bubble cost of predicting a stage later.
    //
    // The cost is a same-cycle icache dependency, since if_is_branch and
    // if_is_jal key off muxed_if_inst. That path is timing-critical, so
    // the logic here is deliberately minimal - opcode/rd/rs1 bit-checks
    // plus a dedicated B/J-type immediate extraction that bypasses
    // control_logic's ROM entirely, which none of these checks need.
    //
    // Correctness never depends on prediction accuracy, wherever this
    // sits: every branch is re-verified in EX against branch_actual_taken
    // (see branch_mispredicted below), so a bug here costs extra flush
    // cycles and nothing more.
    assign if_is_branch = (muxed_if_inst[6:0] == 7'b1100011);
    assign if_is_jal = (muxed_if_inst[6:0] == 7'b1101111);
    assign if_is_jalr = (muxed_if_inst[6:0] == 7'b1100111);
    assign if_rd_is_link = (muxed_if_inst[11:7] == 5'd1) | (muxed_if_inst[11:7] == 5'd5);
    assign if_rs1_is_link = (muxed_if_inst[19:15] == 5'd1) | (muxed_if_inst[19:15] == 5'd5);
    assign if_ras_push_pattern = if_rd_is_link & (if_is_jal | if_is_jalr);
    assign if_ras_pop_pattern = if_is_jalr & if_rs1_is_link & ~if_rd_is_link;
    assign if_jal_taken = if_is_jal & ~stall & ~global_mem_stall;

    // Dedicated B-type/J-type immediate extraction - opcode alone
    // determines the format for these two cases, so this bypasses
    // immgen/control_logic's general ROM-based imm_sel decode entirely.
    wire [31:0] if_b_imm = {{20{muxed_if_inst[31]}}, muxed_if_inst[7],
                             muxed_if_inst[30:25], muxed_if_inst[11:8], 1'b0};
    wire [31:0] if_j_imm = {{12{muxed_if_inst[31]}}, muxed_if_inst[19:12],
                             muxed_if_inst[20], muxed_if_inst[30:21], 1'b0};
    // Both sums computed in parallel off pc (not gated behind an
    // if_is_branch-selected immediate first) so the branch/JAL mux sits
    // AFTER the adder, not in series before it - structurally shortens
    // this timing-critical path by one mux level, same technique used by
    // redirect_target_adder in EX.
    wire [31:0] if_pc_plus_b_imm = pc + if_b_imm;
    wire [31:0] if_pc_plus_j_imm = pc + if_j_imm;
    wire [31:0] if_branch_or_jal_target = if_is_branch ? if_pc_plus_b_imm : if_pc_plus_j_imm;

    (* ram_style = "distributed" *) reg [1:0] pht [0:1023];
    reg [9:0] ghr;
    integer pht_init_i;
    initial begin
        for (pht_init_i = 0; pht_init_i < 1024; pht_init_i = pht_init_i + 1)
            pht[pht_init_i] = 2'b01; // weakly not-taken; self-trains regardless
    end

    // Shared by GHR's speculative shift and RAS push/pop below: this
    // IF-stage instruction is actually advancing to ID (not stalled) and
    // isn't about to be squashed by an EX-stage redirect or a trap.
    assign if_predict_commit = ~stall & ~global_mem_stall & ~pc_sel & ~flush_if & ~flush_id;

    assign if_pht_index = pc[10:1] ^ ghr;
    assign if_predicted_taken = (if_is_branch & pht[if_pht_index][1] & ~stall & ~global_mem_stall)
                               | if_jal_taken
                               | if_ras_hit;
    assign if_predicted_target = if_ras_hit ? ras_top : if_branch_or_jal_target;
    // XOR is self-inverse: id_ex_pht_index (= the pc/ghr pair used when this
    // branch was predicted) XORed with that same pc recovers the GHR value
    // from just before this branch's speculative shift, for rollback below.
    assign ghr_before = id_ex_pht_index ^ id_ex_pc[10:1];

    always @(posedge clk) begin
        if (rst) begin
            ghr <= 10'b0;
        end else if (branch_mispredicted) begin
            // Rollback wins: the flush this mispredict triggers also
            // discards the current (younger) IF-stage instruction, so any
            // speculative shift it would contribute this same cycle is for
            // a path that's being thrown away anyway.
            ghr <= {ghr_before[8:0], branch_actual_taken};
        end else if (if_is_branch & if_predict_commit) begin
            // Speculative shift-in of the PREDICTED bit (0 or 1 - gated on
            // a prediction existing, via if_is_branch, not on its value;
            // gating on if_predicted_taken itself would silently skip a
            // history entry for every not-taken prediction, corrupting GHR
            // into something that stops correlating with real branch
            // behavior).
            ghr <= {ghr[8:0], if_predicted_taken};
        end
    end

    always @(posedge clk) begin
        if (id_ex_is_branch) begin
            if (branch_actual_taken)
                pht[id_ex_pht_index] <= (pht[id_ex_pht_index] == 2'b11) ? 2'b11 : pht[id_ex_pht_index] + 2'b01;
            else
                pht[id_ex_pht_index] <= (pht[id_ex_pht_index] == 2'b00) ? 2'b00 : pht[id_ex_pht_index] - 2'b01;
        end
    end

    // Return Address Stack: predicts JALR-return targets by pushing the
    // link address on call (rd = x1/x5) and popping it on return
    // (rs1 = x1/x5, rd not a link reg), per the RISC-V-suggested RAS hint
    // encoding. JAL is folded in here too since its target is trivially
    // PC+imm - it just always "hits" (if_jal_taken), no stack involved.
    //
    // v1 collapses the rare "pop-then-push" case (rd AND rs1 both link
    // regs, rd != rs1 - e.g. a tail call through a register) into
    // push-only: if_ras_push_pattern fires on rd being a link register,
    // full stop, regardless of rs1. This still leaves a genuine return
    // later unmatched to a real push, but per the correctness invariant
    // above that only costs a missed prediction, never a wrong result.
    localparam RAS_DEPTH = 8;
    reg [31:0] ras_stack [0:RAS_DEPTH-1];
    reg [2:0] ras_sp;    // next free push slot
    reg [3:0] ras_count; // 0..8, saturates (overflow just overwrites oldest)

    assign ras_empty = (ras_count == 4'd0);
    assign ras_top = ras_stack[ras_sp - 3'd1];
    assign if_ras_hit = if_ras_pop_pattern & ~ras_empty & if_predict_commit;

    // if_predict_commit already excludes any IF-stage instruction that
    // won't survive to ID, so - unlike GHR - no misprediction rollback
    // path is needed here: nothing ever gets pushed/popped speculatively
    // only to be undone later. The one accepted gap: a trap taken *after*
    // this commit, while the JAL/JALR itself sits further down the
    // pipeline, replays it from mepc post-mret and can double-push or
    // extra-pop, drifting RAS depth over time - symmetric to GHR's own
    // accepted trap-replay exposure, and bounded to costing extra flush
    // cycles, never correctness.
    always @(posedge clk) begin
        if (rst) begin
            ras_sp <= 3'b0;
            ras_count <= 4'b0;
        end else if (if_ras_push_pattern & if_predict_commit) begin
            ras_stack[ras_sp] <= pc + (is_compressed ? 32'd2 : 32'd4);
            ras_sp <= ras_sp + 3'd1;
            ras_count <= (ras_count == RAS_DEPTH[3:0]) ? RAS_DEPTH[3:0] : ras_count + 4'd1;
        end else if (if_ras_pop_pattern & if_predict_commit & ~ras_empty) begin
            ras_sp <= ras_sp - 3'd1;
            ras_count <= ras_count - 4'd1;
        end
    end

    // Priority: trap (highest) > EX-stage redirect (misprediction fixup or
    // an unconditional jump - always correct, since it belongs to an
    // older instruction than whatever IF is currently fetching) > this
    // cycle's own IF-stage prediction > sequential fetch (lowest).
    assign actual_pc_sel = pc_trap_override | pc_sel | if_predicted_taken;
    assign actual_jump_target = pc_trap_override ? trap_target_pc :
                                 pc_sel           ? ex_redirect_target :
                                                     if_predicted_target;
    program_counter PC (
        .clk(clk),
        .rst(rst),
        .stall(stall | global_mem_stall),
        .pc_sel(actual_pc_sel),
        .mem_address(actual_jump_target),
        .pc_inc(pc_inc),
        .pc(pc),
        .pc_next_out(pc_next)
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
        .predicted_taken_in(if_predicted_taken),
        .predicted_target_in(if_predicted_target),
        .pht_index_in(if_pht_index),
        .pc_out(if_id_pc),
        .inst_out(if_id_inst),
        .compressed_out(if_id_compressed),
        .predicted_taken_out(if_id_predicted_taken),
        .predicted_target_out(if_id_predicted_target),
        .pht_index_out(if_id_pht_index),
        .valid_out(if_id_valid)
    );

    assign debug_instr = if_id_inst;
    assign debug_pc = if_id_pc; // pairs with debug_instr - same if_id stage
    assign debug_raw_pc = pc;
    assign debug_id_predicted_taken = if_id_predicted_taken;
    assign debug_cache_ready = cache_ready;

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
        .predicted_taken_in(if_id_predicted_taken),
        .predicted_target_in(if_id_predicted_target),
        .pht_index_in(if_id_pht_index),
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
        .predicted_taken_out(id_ex_predicted_taken),
        .predicted_target_out(id_ex_predicted_target),
        .pht_index_out(id_ex_pht_index),
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

    // Dedicated to the branch/jump redirect path (ex_redirect_target,
    // jalr_ras_mispredicted) - both only ever need alu_sel=0 (add),
    // verified against every ROM entry that reaches them (branches/jal/
    // jalr all decode alu_sel=0 in control_logic.v). Bypasses the general
    // alu module's full 14-way case-select mux (including two unconditional
    // 32x32 multiplies), which alu_out can't avoid since it's also shared
    // with register writeback, which needs every alu_sel case - a real
    // structural shortening of the tightest timing path (WNS=0.092ns
    // measured on real hardware pre-this-change), not something synthesis
    // could specialize on its own.
    wire [31:0] redirect_target_adder = alu_a + alu_b;

    alu ALU (
        .a(alu_a),
        .b(alu_b),
        .alu_sel(id_ex_alu_sel),
        .alu_res(alu_out)
    );

    // Comb divide fails timing so this runs as a
    // ~33-cycle multi-cycle op that stalls the whole pipeline via global_mem_stall
    // alu_sel 16-19 (div/divu/rem/remu) exactly - NOT simply alu_sel[4].
    // The old "bit 4 means divide" shortcut silently claimed the whole
    // upper half of the alu_sel space, so any later extension using a
    // value >= 20 would stall on the divider and write back a quotient.
    // Zba (sh1add/sh2add/sh3add, alu_sel 20-22) is the first such case.
    wire is_div_op = (id_ex_alu_sel[4:2] == 3'b100);
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

    // Whether this conditional branch (if it is one) actually resolves
    // taken, independent of what was predicted back in ID.
    assign branch_actual_taken = (id_ex_br_eq & id_ex_is_beq) |
                (~id_ex_br_eq & id_ex_is_bne) |
                (id_ex_br_lt & (id_ex_is_blt | id_ex_is_bltu)) |
                (~id_ex_br_lt & (id_ex_is_bge | id_ex_is_bgeu));

    assign branch_mispredicted = id_ex_is_branch &
                (branch_actual_taken != id_ex_predicted_taken);

    // JALR's target is register-computed, so unlike JAL/branches (whose IF-
    // stage and EX-stage target computations are structurally identical off
    // the same threaded pc/imm) a RAS-predicted JALR's IF-stage guess can
    // genuinely disagree with the real target. id_ex_is_jalr &
    // id_ex_predicted_taken unambiguously means "this JALR was RAS-hit
    // predicted", since if_predicted_taken's only JALR-reachable term is
    // if_ras_hit.
    assign jalr_ras_mispredicted = id_ex_is_jalr & id_ex_predicted_taken &
                (redirect_target_adder != id_ex_predicted_target);

    // pc_sel means "EX needs to redirect fetch": a JAL/JALR that ID didn't
    // already correctly redirect (predicted_taken=0 - e.g. RAS-empty JALR,
    // or the rare same-cycle-stall edge case), a RAS-predicted JALR whose
    // guess was wrong, or a conditional branch whose prediction was wrong.
    // When none of these hold, ID's own early redirect (JAL: always-taken
    // PC+imm; JALR: RAS pop) was already correct, so no redundant
    // re-redirect/flush happens here - that's the actual performance win.
    assign pc_sel = (id_ex_is_jal & ~id_ex_predicted_taken) |
                (id_ex_is_jalr & (~id_ex_predicted_taken | jalr_ras_mispredicted)) |
                branch_mispredicted;

    // Redirect target for pc_sel: redirect_target_adder is always ground
    // truth (PC+imm for JAL/taken-branch, rs1+imm for JALR) regardless of
    // why pc_sel fired, so this needs no change for the RAS/JAL-in-ID
    // cases above. Uses the dedicated adder (not alu_out) to bypass the
    // general ALU's case-select mux on this timing-critical path - see
    // redirect_target_adder's declaration above.
    assign id_ex_pc_plus_inc = id_ex_pc + (id_ex_compressed ? 32'd2 : 32'd4);
    assign ex_redirect_target =
        (id_ex_is_jal | id_ex_is_jalr | branch_actual_taken) ? redirect_target_adder
                                                               : id_ex_pc_plus_inc;

    // Speculative dcache index: the address MEM will present next cycle.
    // Only the low DC_SPEC_W bits ever reach the array address pins, so
    // this is a narrow dedicated adder run in parallel with the ALU rather
    // than a tap off alu_out - ALU-to-BRAM-address-setup is the one new
    // timing arc this conversion introduces (same reasoning as
    // redirect_target_adder above). Loads and stores both compute
    // rs1 + imm, matching alu_a/alu_b's a_sel=rs1 / b_sel=imm selection.
    //
    // global_mem_stall (and NOT stall) is the right gate: ex_mem_reg has no
    // stall port, so on a load-use stall EX's real instruction still
    // advances to MEM next cycle and the speculation stays correct. When
    // frozen, MEM holds its instruction, so ex_mem_alu is the right source.
    // AMO uses rs1 only; if its immediate isn't zero the speculation simply
    // misses and dcache_bram's spec_ok costs one stall cycle - never data.
    localparam DC_SPEC_W = DC_IDX_BITS + DC_OFF_BITS;
    wire [DC_SPEC_W-1:0] dc_ea_spec = fwd_rs1[DC_SPEC_W-1:0] + id_ex_imm[DC_SPEC_W-1:0];
    wire [DC_SPEC_W-1:0] dc_addr_next = global_mem_stall ? ex_mem_alu[DC_SPEC_W-1:0]
                                                         : dc_ea_spec;

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
        .flush(flush_ex),
        .alu_res_in(actual_ex_result),
        .rs2_in(fwd_rs2),
        .inst_in(id_ex_inst),
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

    // AI accelerator (2x2 output-stationary systolic matmul, INT8/INT32):
    // axi_lite_bridge converts this same d_req-style handshake into a real
    // AXI4-Lite master transaction, talking to mm_accel's AXI4-Lite slave
    // port on-chip - see src/mm_accel.v for the register map.
    wire [31:0] accel_axi_awaddr, accel_axi_wdata, accel_axi_araddr, accel_axi_rdata;
    wire [3:0] accel_axi_wstrb;
    wire accel_axi_awvalid, accel_axi_awready, accel_axi_wvalid, accel_axi_wready;
    wire accel_axi_bvalid, accel_axi_bready;
    wire [1:0] accel_axi_bresp, accel_axi_rresp;
    wire accel_axi_arvalid, accel_axi_arready, accel_axi_rvalid, accel_axi_rready;

    axi_lite_bridge ACCEL_BRIDGE (
        .clk(clk),
        .rst(rst),
        .d_req(accel_req),
        .d_we(store_commits),
        .d_addr(ex_mem_alu),
        .d_wdata(ex_mem_rs2),
        .d_rdata(accel_rdata),
        .d_ready(accel_ready),
        .m_axi_awaddr(accel_axi_awaddr),
        .m_axi_awvalid(accel_axi_awvalid),
        .m_axi_awready(accel_axi_awready),
        .m_axi_wdata(accel_axi_wdata),
        .m_axi_wstrb(accel_axi_wstrb),
        .m_axi_wvalid(accel_axi_wvalid),
        .m_axi_wready(accel_axi_wready),
        .m_axi_bresp(accel_axi_bresp),
        .m_axi_bvalid(accel_axi_bvalid),
        .m_axi_bready(accel_axi_bready),
        .m_axi_araddr(accel_axi_araddr),
        .m_axi_arvalid(accel_axi_arvalid),
        .m_axi_arready(accel_axi_arready),
        .m_axi_rdata(accel_axi_rdata),
        .m_axi_rresp(accel_axi_rresp),
        .m_axi_rvalid(accel_axi_rvalid),
        .m_axi_rready(accel_axi_rready)
    );

    mm_accel ACCEL (
        .clk(clk),
        .rst(rst),
        .s_axi_awaddr(accel_axi_awaddr),
        .s_axi_awvalid(accel_axi_awvalid),
        .s_axi_awready(accel_axi_awready),
        .s_axi_wdata(accel_axi_wdata),
        .s_axi_wstrb(accel_axi_wstrb),
        .s_axi_wvalid(accel_axi_wvalid),
        .s_axi_wready(accel_axi_wready),
        .s_axi_bresp(accel_axi_bresp),
        .s_axi_bvalid(accel_axi_bvalid),
        .s_axi_bready(accel_axi_bready),
        .s_axi_araddr(accel_axi_araddr),
        .s_axi_arvalid(accel_axi_arvalid),
        .s_axi_arready(accel_axi_arready),
        .s_axi_rdata(accel_axi_rdata),
        .s_axi_rresp(accel_axi_rresp),
        .s_axi_rvalid(accel_axi_rvalid),
        .s_axi_rready(accel_axi_rready)
    );

    // MEM
    wire [31:0] ps_store_data;
    partial_store PS (
        .inst(ex_mem_inst),
        .mem_address(ex_mem_alu),
        .data_from_reg(ex_mem_rs2),
        .mem_rw(ex_mem_mem_rw),
        .mem_write_mask(raw_write_mask),
        .data_to_mem(ps_store_data)
    );
    assign store_data = amo_write_phase ? amo_new_value : ps_store_data;

    // AMO sequencer: an atomic read-modify-write (read old word, compute new
    // word, write it back, return old word to rd) needs two dcache accesses,
    // but the pipeline only does one per instruction - drives its own
    // two-phase dcache_ren/dcache_wen sequence, holding the pipeline frozen
    // via amo_stall (same idea as div_unit's multi-cycle stall, but div_unit
    // never touches memory).
    //
    // amo_result_ready is latched (mirrors div_result_ready) rather than a
    // live check so amo_stall can't drop between the read and write phases:
    // interrupts are sampled gated by ~global_mem_stall, so an early drop
    // could let a trap land mid-sequence - after the read but before the
    // write commits, or flushing the write away after the read already
    // happened. Holding the stall until one cycle after the write's own
    // dcache_ready means the flush boundary only ever sees "not started" or
    // "fully done".
    localparam AMO_IDLE = 2'd0, AMO_READ = 2'd1, AMO_WRITE = 2'd2;
    reg [1:0] amo_state;
    reg [31:0] amo_old_value;
    reg amo_result_ready;

    wire amo_read_phase  = ex_mem_is_amo && (amo_state == AMO_READ);
    wire amo_write_phase = ex_mem_is_amo && (amo_state == AMO_WRITE) && !amo_result_ready;
    wire amo_stall = ex_mem_is_amo && !amo_result_ready;

    reg [31:0] amo_new_value;
    always @(*) begin
        case (ex_mem_atomic_op)
            5'b00001: amo_new_value = ex_mem_rs2;                                                                // AMOSWAP
            5'b00000: amo_new_value = amo_old_value + ex_mem_rs2;                                                // AMOADD
            5'b00100: amo_new_value = amo_old_value ^ ex_mem_rs2;                                                // AMOXOR
            5'b01100: amo_new_value = amo_old_value & ex_mem_rs2;                                                // AMOAND
            5'b01000: amo_new_value = amo_old_value | ex_mem_rs2;                                                // AMOOR
            5'b10000: amo_new_value = ($signed(amo_old_value) < $signed(ex_mem_rs2)) ? amo_old_value : ex_mem_rs2; // AMOMIN
            5'b10100: amo_new_value = ($signed(amo_old_value) > $signed(ex_mem_rs2)) ? amo_old_value : ex_mem_rs2; // AMOMAX
            5'b11000: amo_new_value = (amo_old_value < ex_mem_rs2) ? amo_old_value : ex_mem_rs2;                 // AMOMINU
            5'b11100: amo_new_value = (amo_old_value > ex_mem_rs2) ? amo_old_value : ex_mem_rs2;                 // AMOMAXU
            default:  amo_new_value = ex_mem_rs2;
        endcase
    end

    always @(posedge clk) begin
        if (rst || !ex_mem_is_amo) begin
            amo_state <= AMO_IDLE;
            amo_result_ready <= 1'b0;
        end else begin
            case (amo_state)
                AMO_IDLE: amo_state <= AMO_READ;
                AMO_READ: if (dcache_ready) begin
                    amo_old_value <= dcache_read_data;
                    amo_state <= AMO_WRITE;
                end
                AMO_WRITE: if (dcache_ready) amo_result_ready <= 1'b1;
                default: amo_state <= AMO_IDLE;
            endcase
        end
    end

    // LR/SC reservation bookkeeping should follow the MEM-stage operation,
    // not unrelated front-end stalls. LR may complete while the I-cache is
    // fetching the next line, so set the reservation when the D-cache load is ready
    // Keep SC clear gated by global_mem_stall so the combinational SC result
    // remains stable until the SC instruction can advance to WB
    assign lr_reservation_set = ex_mem_is_lr && dcache_ren && dcache_ready;
    assign sc_reservation_clear = ex_mem_is_sc && ~global_mem_stall;
    assign normal_store_reservation_clear = (is_store && !ex_mem_is_sc &&
                                             (is_mmio || (dcache_wen && dcache_ready) || (is_uart && uart_ready) || (is_intc && intc_ready) || (is_accel && accel_ready))) ||
                                             (amo_write_phase && dcache_ready);

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
    assign mem_write_mask = amo_write_phase ? 4'b1111 : (block_sc_store ? 4'b0000 : raw_write_mask);
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

    assign final_mem_read_data = ex_mem_is_amo ? amo_old_value :
                                 is_clint ? clint_rdata :
                                 is_led ? {28'b0, leds} :
                                 is_uart ? uart_rdata_latched :
                                 is_intc ? intc_rdata_latched :
                                 is_accel ? accel_rdata_latched :
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
    output reg [31:0] pc,
    // The value `pc` will actually hold next cycle. icache_bram addresses
    // its BRAMs from this so a hit costs no extra cycle (see icache_bram.v).
    // Exported from here rather than recomputed at the call site on purpose:
    // if the two ever diverged, the cache would return the wrong line for
    // the PC being fetched, i.e. execute a wrong instruction. Note the
    // `stall` term - plain next_pc is NOT what pc holds while stalled.
    output wire [31:0] pc_next_out
);
    localparam [31:0] RESET_PC = 32'h4000_0000;

    wire [31:0] next_pc = pc_sel ? mem_address : pc + pc_inc;

    assign pc_next_out = rst ? RESET_PC : (stall ? pc : next_pc);

    always @(posedge clk) begin
        if (rst)
            pc <= RESET_PC;
        else if (!stall)
            pc <= next_pc;
    end

endmodule


module if_id_reg (
    input wire clk, rst, stall, flush, mem_stall,
    input wire [31:0] pc_in, inst_in,
    input wire compressed_in,
    input wire predicted_taken_in,
    input wire [31:0] predicted_target_in,
    input wire [9:0] pht_index_in,
    output reg [31:0] pc_out, inst_out,
    output reg compressed_out,
    output reg predicted_taken_out,
    output reg [31:0] predicted_target_out,
    output reg [9:0] pht_index_out,
    // 0 whenever this slot holds a flush-inserted bubble rather than a
    // genuinely fetched instruction (indistinguishable from a real NOP by
    // inst_out/pc_out alone - both read as 0x13/0 either way). Needed so
    // downstream logic (interrupt-taking) can tell the difference.
    output reg valid_out
);
    task clear;
        begin
            pc_out <= 32'b0;
            inst_out <= 32'h00000013;
            compressed_out <= 1'b0;
            predicted_taken_out <= 1'b0;
            predicted_target_out <= 32'b0;
            pht_index_out <= 10'b0;
            valid_out <= 1'b0;
        end
    endtask

    always @(posedge clk) begin
        if (rst) clear;
        else if (mem_stall) begin
            // Freeze
        end else if (flush) clear;
        else if (!stall) begin
            pc_out <= pc_in;
            inst_out <= inst_in;
            compressed_out <= compressed_in;
            predicted_taken_out <= predicted_taken_in;
            predicted_target_out <= predicted_target_in;
            pht_index_out <= pht_index_in;
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
    input wire predicted_taken_in,
    input wire [31:0] predicted_target_in,
    input wire [9:0] pht_index_in,
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
    output reg predicted_taken_out,
    output reg [31:0] predicted_target_out,
    output reg [9:0] pht_index_out,
    output reg csr_wen_out,
    output reg [1:0] csr_op_out,
    output reg csr_use_imm_out,
    output reg [4:0] csr_uimm_out,
    // See if_id_reg's valid_out for what this means; propagated forward
    // (not unconditionally set) so a bubble already flushed upstream stays
    // marked invalid all the way through EX.
    output reg valid_out
);
    task clear;
        begin
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
            predicted_taken_out <= 1'b0;
            predicted_target_out <= 32'b0;
            pht_index_out <= 10'b0;
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            atomic_op_out <= 5'b0;
            csr_wen_out <= 1'b0;
            csr_op_out <= 2'b0;
            csr_use_imm_out <= 1'b0;
            csr_uimm_out <= 5'b0;
            valid_out <= 1'b0;
        end
    endtask

    always @(posedge clk) begin
        if (rst) clear;
        else if (mem_stall) begin
            // Freeze
        end else if (flush) clear;
        else begin
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
            predicted_taken_out <= predicted_taken_in;
            predicted_target_out <= predicted_target_in;
            pht_index_out <= pht_index_in;
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
    input wire clk, rst, mem_stall, flush,
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
        end else if (flush) begin
            // Only the control fields that can cause a side effect
            // (register writeback, memory write, LR/SC/AMO sequencing) need
            // clearing here - without this, a trap taken while this
            // instruction sat in EX would let it leak through and complete
            // normally in MEM/WB even though mepc points back at it for
            // re-execution after mret, causing it to run twice.
            reg_wen_out <= 1'b0;
            mem_rw_out <= 1'b0;
            is_lr_out <= 1'b0;
            is_sc_out <= 1'b0;
            is_amo_out <= 1'b0;
            inst_out <= 32'h00000013;
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
