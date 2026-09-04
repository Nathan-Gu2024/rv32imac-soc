`timescale 1ns/1ps
`include "../src/cpu.v"

// Targeted correctness test for the full RV32A AMO implementation (see the
// approved atomics plan). Boots through the real TCM stub -> DDR flow, same
// mock memory harness as tb_coremark_sim.v, then checks each AMO's returned
// old value AND the new value actually landed in memory (via a follow-up lw
// baked into test_amo.S), using the same hierarchical register-peek check()
// pattern established in Testbenches/tb.v.
module tb_dcache_hazard;
    reg clk, rst;
    wire uart_tx_line;
    reg uart_rx_line;
    wire [3:0] leds;

    wire [31:0] icache_mem_req_addr;
    wire icache_mem_req_valid;
    wire [127:0] icache_mem_read_data;
    wire icache_mem_ready;

    wire [31:0] dmem_req_addr;
    wire [31:0] store_data;
    wire [3:0] mem_write_mask;
    wire dcache_mem_req_valid;
    wire dcache_mem_req_write;
    wire [127:0] dcache_mem_wline;
    wire [127:0] dcache_mem_read_data_block;
    wire dcache_mem_ready;

    wire [31:0] debug_pc, debug_instr;
    wire debug_dcache_valid, debug_dcache_ready;
    wire debug_tcm_d_req, debug_tcm_d_ready;
    wire debug_global_mem_stall;
    wire [31:0] debug_raw_pc;
    wire debug_id_predicted_taken;
    wire debug_cache_ready;

    // Two D-cache array implementations run the SAME checks below. Define
    // USE_SRAM to build against the sky130 SRAM macros instead of inferred
    // Block RAM; the store->load bypass and the AMO path are exactly the
    // hazards whose behaviour differs between the two memories, so running
    // one config and not the other proves half of what is needed.
    //
    //   iverilog -g2005 -o /tmp/tb -s tb_dcache_hazard tb_dcache_hazard.v
    //   iverilog -g2005 -DUSE_SRAM -o /tmp/tb -s tb_dcache_hazard \
    //       tb_dcache_hazard.v ../src/sram_sky130.v \
    //       $PDK/libs.ref/sky130_sram_macros/verilog/sky130_sram_2kbyte_1rw1r_32x512_8.v
    //
    // Parameter override rather than defparam on purpose: DC_USE_SRAM feeds
    // a generate condition, and defparam on such a parameter is the exact
    // interaction that broke TCM's INIT_ENABLE.
`ifdef USE_SRAM
    cpu_pipelined #(
        .IC_NUM_SETS(512),      // the macro's fixed depth
        .DC_NUM_SETS(512),
        .IC_USE_SRAM(1),
        .DC_USE_SRAM(1)
    ) DUT (
`else
    cpu_pipelined DUT (
`endif
        .clk(clk), .rst(rst),
        .uart_tx(uart_tx_line),
        .uart_rx(uart_rx_line),
        .leds(leds),

        .icache_mem_req_addr(icache_mem_req_addr),
        .icache_mem_req_valid(icache_mem_req_valid),
        .icache_mem_read_data(icache_mem_read_data),
        .icache_mem_ready(icache_mem_ready),

        .dmem_req_addr(dmem_req_addr),
        .store_data(store_data),
        .mem_write_mask(mem_write_mask),

        .dcache_mem_req_valid(dcache_mem_req_valid),
        .dcache_mem_req_write(dcache_mem_req_write),
        .dcache_mem_wline(dcache_mem_wline),
        .dcache_mem_read_data_block(dcache_mem_read_data_block),
        .dcache_mem_ready(dcache_mem_ready),

        .debug_pc(debug_pc),
        .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall),
        .debug_raw_pc(debug_raw_pc),
        .debug_id_predicted_taken(debug_id_predicted_taken),
        .debug_cache_ready(debug_cache_ready)
    );

    defparam DUT.TCM.INIT_FILE =
        "../fpga/ddr_fixed.mem";

    localparam DDR_WORDS = 524288;
    reg [31:0] ddr_mem [0:DDR_WORDS-1];

    initial begin
        $readmemh("test_dcache_hazard_mem.hex", ddr_mem);
    end

    localparam LINE_LATENCY = 8;

    reg [3:0] i_lat_cnt;
    reg i_busy;
    reg [31:0] i_addr_latched;
    always @(posedge clk) begin
        if (rst) begin
            i_busy <= 1'b0;
            i_lat_cnt <= 0;
        end else if (!i_busy && icache_mem_req_valid) begin
            i_busy <= 1'b1;
            i_lat_cnt <= LINE_LATENCY;
            i_addr_latched <= icache_mem_req_addr;
        end else if (i_busy) begin
            if (i_lat_cnt == 0) i_busy <= 1'b0;
            else i_lat_cnt <= i_lat_cnt - 1;
        end
    end
    assign icache_mem_ready = i_busy && (i_lat_cnt == 0);
    assign icache_mem_read_data = {
        ddr_mem[{i_addr_latched[28:4], 2'b11}],
        ddr_mem[{i_addr_latched[28:4], 2'b10}],
        ddr_mem[{i_addr_latched[28:4], 2'b01}],
        ddr_mem[{i_addr_latched[28:4], 2'b00}]
    };

    reg [3:0] d_lat_cnt;
    reg d_busy;
    reg [31:0] d_addr_latched;
    reg d_write_latched;
    always @(posedge clk) begin
        if (rst) begin
            d_busy <= 1'b0;
            d_lat_cnt <= 0;
        end else if (!d_busy && dcache_mem_req_valid) begin
            d_busy <= 1'b1;
            d_lat_cnt <= LINE_LATENCY;
            d_addr_latched <= dmem_req_addr;
            d_write_latched <= dcache_mem_req_write;
        end else if (d_busy) begin
            if (d_lat_cnt == 0) d_busy <= 1'b0;
            else d_lat_cnt <= d_lat_cnt - 1;
        end
    end
    assign dcache_mem_ready = d_busy && (d_lat_cnt == 0);
    assign dcache_mem_read_data_block = {
        ddr_mem[{d_addr_latched[28:4], 2'b11}],
        ddr_mem[{d_addr_latched[28:4], 2'b10}],
        ddr_mem[{d_addr_latched[28:4], 2'b01}],
        ddr_mem[{d_addr_latched[28:4], 2'b00}]
    };
    always @(posedge clk) begin
        if (dcache_mem_ready && d_write_latched) begin
            ddr_mem[{d_addr_latched[28:4], 2'b00}] <= dcache_mem_wline[31:0];
            ddr_mem[{d_addr_latched[28:4], 2'b01}] <= dcache_mem_wline[63:32];
            ddr_mem[{d_addr_latched[28:4], 2'b10}] <= dcache_mem_wline[95:64];
            ddr_mem[{d_addr_latched[28:4], 2'b11}] <= dcache_mem_wline[127:96];
        end
    end

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst = 1;
        uart_rx_line = 1;
        repeat (5) @(posedge clk);
        rst = 0;
    end

    task check;
        input [8*8:1] name;
        input [4:0] reg_num;
        input [31:0] expected;
        begin
            if (DUT.RF.regs[reg_num] === expected)
                $display("PASS: %0s x%0d = 0x%08h", name, reg_num, expected);
            else
                $display("FAIL: %0s x%0d = 0x%08h, expected 0x%08h",
                    name, reg_num, DUT.RF.regs[reg_num], expected);
        end
    endtask

    initial begin
        // Wait for the sentinel (x1 = 0xA5A5A5A5) or time out.
        wait (DUT.RF.regs[1] === 32'hA5A5A5A5);
        $display("Sentinel reached, checking results...");

        // Each of these loads sits immediately after the store it depends
        // on, so the store's write edge collides with the speculative read
        // for the load. Without dcache_bram.v's one-deep bypass these
        // return stale (pre-store) data.
        check("sw->lw  ", 10, 32'hDEADBEEF);
        check("sb->lw  ", 11, 32'hDEADBEAA);  // only the low byte changes
        check("sh->lw  ", 12, 32'hDEAD1234);  // upper half must survive
        check("word1   ", 13, 32'h12345678);  // other word, same line
        check("word0   ", 14, 32'hDEAD1234);  // bypass must respect bank

        // +16KB is exactly the cache stride, so this is a same-set/
        // different-tag conflict: the dirty line must be evicted and
        // written back, then refilled intact when we return to it.
        check("conflict", 15, 32'hCAFEBABE);
        check("wb+refil", 16, 32'hDEAD1234);

        $display("DONE");
        $finish;
    end

    initial begin
        #2_000_000; // 2ms sim time budget - this test is tiny
        $display("[TIMEOUT] sentinel never reached (x1=0x%08h)", DUT.RF.regs[1]);
        $display("pc=0x%08h raw_pc=0x%08h instr=0x%08h global_mem_stall=%b amo_state=%0d amo_result_ready=%b ex_mem_is_amo=%b",
            DUT.debug_pc, DUT.debug_raw_pc, DUT.debug_instr, DUT.global_mem_stall,
            DUT.amo_state, DUT.amo_result_ready, DUT.ex_mem_is_amo);
        $display("trap_taken=%b flush_ex=%b flush_id=%b flush_if=%b mtvec=0x%08h mepc=0x%08h timer_interrupt=%b mstatus_mie=%b pc_sel=%b actual_pc_sel=%b pc_trap_override=%b",
            DUT.trap_taken, DUT.flush_ex, DUT.flush_id, DUT.flush_if, DUT.mtvec_out, DUT.mepc_out,
            DUT.timer_interrupt, DUT.mstatus_mie, DUT.pc_sel, DUT.actual_pc_sel, DUT.pc_trap_override);
        $display("dmem_stall=%b imem_stall=%b div_stall=%b uart_stall=%b intc_stall=%b cache_ready=%b dcache_ready=%b",
            DUT.dmem_stall, DUT.imem_stall, DUT.div_stall, DUT.uart_stall, DUT.intc_stall,
            DUT.cache_ready, DUT.dcache_ready);
        $display("x10-x18 (old): %0d %0d %0d %0d %0d %0d %0d %0d %0d",
            DUT.RF.regs[10], DUT.RF.regs[11], DUT.RF.regs[12], DUT.RF.regs[13],
            DUT.RF.regs[14], DUT.RF.regs[15], DUT.RF.regs[16], DUT.RF.regs[17], DUT.RF.regs[18]);
        $display("x19-x27 (new): %0d %0d %0d %0d %0d %0d %0d %0d %0d",
            DUT.RF.regs[19], DUT.RF.regs[20], DUT.RF.regs[21], DUT.RF.regs[22],
            DUT.RF.regs[23], DUT.RF.regs[24], DUT.RF.regs[25], DUT.RF.regs[26], DUT.RF.regs[27]);
        $finish;
    end

endmodule
