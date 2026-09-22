`timescale 1ns/1ps
`include "../src/cpu.v"

// The stall-progress watchdog, tested from both sides.
//
// A watchdog that never fires is useless and a watchdog that fires on healthy
// behaviour is worse than none, so both halves are checked here against the
// same threshold.
//
// WHERE THE NO-FALSE-POSITIVE EVIDENCE LIVES. It is not here, and it cannot
// be: this bench reaches DDR only by hanging, and the longest legitimate
// stall in the design is the first DDR fetch waiting out the I-cache reset
// sweep (IC_NUM_SETS cycles) - which in this bench is contiguous with the
// hang and so cannot be told apart from it.
//
// That case was measured separately on the three benches that do run a real
// program to completion: tb_dcache_hazard and tb_amo_test both peak at
// exactly 2047 cycles of unbroken stall, tb_irqtest at 3, and all three end
// with stall_hung low. Against the shipping STALL_WATCHDOG_LIMIT of 65535
// that is 32x margin. While the PC is inside TCM the I-cache is bypassed
// entirely (icache.v routes the TCM range internally), so TCM execution
// contributes no stall at all - which is what the first check below pins.
//
// The threshold here is lowered to 4096 purely to keep the test short.
//
// The hang is real, not forced: fpga/ddr_fixed.mem is the actual TCM boot
// stub, which writes 5 to the LEDs and then jumps to DDR at 0x00100000. With
// icache_mem_ready held low that jump requests a line refill which never
// completes, and the pipeline is wedged exactly as it would be by a dead AXI
// slave or the arbiter livelock this watchdog exists to catch.
module tb_stall_watchdog;

    localparam integer WD_LIMIT = 4096;

    reg clk = 1'b0, rst = 1'b1;
    always #10 clk = ~clk;              // 50 MHz

    wire uart_tx_pin;
    wire [3:0] leds;

    wire [31:0]  icache_mem_req_addr;
    wire         icache_mem_req_valid;
    // Never ready: this is what turns the stub's jump to DDR into a hang.
    wire [127:0] icache_mem_read_data = 128'h0;
    wire         icache_mem_ready     = 1'b0;

    wire [31:0]  dmem_req_addr;
    wire [31:0]  store_data;
    wire [3:0]   mem_write_mask;
    wire         dcache_mem_req_valid;
    wire         dcache_mem_req_write;
    wire [127:0] dcache_mem_wline;
    wire [127:0] dcache_mem_read_data_block = 128'h0;
    wire         dcache_mem_ready           = 1'b0;

    wire        debug_dcache_valid, debug_dcache_ready;
    wire        debug_tcm_d_req, debug_tcm_d_ready, debug_global_mem_stall;
    wire [31:0] debug_pc, debug_instr;

    cpu_pipelined #(
        .STALL_WATCHDOG_LIMIT(WD_LIMIT)
    ) DUT (
        .debug_pc(debug_pc),
        .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall),

        .clk(clk), .rst(rst),
        .uart_tx(uart_tx_pin),
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
        .dcache_mem_ready(dcache_mem_ready)
    );

    defparam DUT.TCM.INIT_FILE = "../fpga/ddr_fixed.mem";

    integer errors = 0;
    integer wd_peak_healthy = 0;
    integer i;
    reg     saw_stub = 1'b0;

    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    // Track the peak stall length reached while the machine is still healthy,
    // i.e. up to the moment the stub's store to the LEDs retires.
    always @(posedge clk) begin
        if (!rst && !saw_stub && DUT.stall_watchdog > wd_peak_healthy)
            wd_peak_healthy = DUT.stall_watchdog;
    end

    initial begin
        repeat (3) @(posedge clk);
        rst = 1'b0;

        // ---- healthy: the reset sweep must NOT trip the watchdog ----
        // The stub writes 5 to the LEDs before it jumps, so leds==0101 is
        // proof the core executed real instructions after the sweep.
        wait (leds === 4'b0101);
        saw_stub = 1'b1;
        $display("--- stub ran: leds=%b, peak stall so far %0d cycles ---",
                 leds, wd_peak_healthy);
        check(DUT.stall_hung === 1'b0,
              "watchdog quiet while the core is executing");
        check(wd_peak_healthy < 100,
              "TCM execution contributes essentially no stall (I-cache bypassed)");

        // ---- hung: the jump to DDR never completes ----
        for (i = 0; i < WD_LIMIT + 200; i = i + 1) @(posedge clk);
        check(DUT.stall_hung === 1'b1,
              "watchdog fired on a permanently stalled pipeline");
        check(leds === 4'b1111,
              "all four LEDs show the hang, overriding the stub value");
        check(DUT.stall_watchdog === WD_LIMIT[15:0],
              "counter saturates at the limit rather than wrapping");

        // Sticky: it must not clear itself while still hung.
        for (i = 0; i < 500; i = i + 1) @(posedge clk);
        check(DUT.stall_hung === 1'b1, "hang flag is sticky");

        $display("");
        if (errors == 0) $display("=== STALL WATCHDOG TESTS PASSED ===");
        else             $display("=== %0d STALL WATCHDOG ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("TIMEOUT - the stub never reached the LED store");
        $finish;
    end
endmodule
