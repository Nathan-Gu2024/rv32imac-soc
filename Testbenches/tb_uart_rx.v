`include "../src/cpu.v"

module tb_uart_rx;
    reg clk = 0, rst;
    reg uart_rx_pin = 1'b1;
    wire uart_tx_pin;
    wire [3:0] leds;

    wire [31:0] icache_mem_req_addr;
    wire icache_mem_req_valid;
    wire [127:0] icache_mem_read_data = 128'h0;
    wire icache_mem_ready = 1'b0;

    wire [31:0] dmem_req_addr;
    wire [31:0] store_data;
    wire [3:0] mem_write_mask;
    wire dcache_mem_req_valid;
    wire dcache_mem_req_write;
    wire [127:0] dcache_mem_wline;
    wire [127:0] dcache_mem_read_data_block = 128'h0;
    wire dcache_mem_ready = 1'b0;

    wire debug_dcache_valid, debug_dcache_ready, debug_tcm_d_req, debug_tcm_d_ready, debug_global_mem_stall;
    wire [31:0] debug_pc, debug_instr;

    cpu_pipelined DUT (
        .debug_pc(debug_pc),
        .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall),

        .clk(clk), .rst(rst),
        .uart_tx(uart_tx_pin),
        .uart_rx(uart_rx_pin),
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

    localparam CLOCKS_PER_BIT = 50_000_000 / 115200;
    always #10 clk = ~clk; // 50MHz -> 20ns period

    reg [31:0] temp_mem [0:16383];
    integer i;

    // Bit-bang a standard 8N1 UART frame onto uart_rx_pin
    task send_byte;
        input [7:0] b;
        integer bi;
        begin
            uart_rx_pin = 1'b0; // start bit
            # (CLOCKS_PER_BIT * 20);
            for (bi = 0; bi < 8; bi = bi + 1) begin
                uart_rx_pin = b[bi];
                # (CLOCKS_PER_BIT * 20);
            end
            uart_rx_pin = 1'b1; // stop bit
            # (CLOCKS_PER_BIT * 20);
        end
    endtask

    reg [31:0] last_pc = 32'hFFFFFFFF;
    always @(posedge clk) begin
        if (DUT.pc !== last_pc) begin
            $display("t=%0t PC-CHANGE pc=%h if_id_inst=%h trap_taken=%b pc_sel=%b pc_trap_override=%b gms=%b actual_jump_target=%h alu_out=%h",
                $time, DUT.pc, DUT.if_id_inst, DUT.trap_taken, DUT.pc_sel, DUT.pc_trap_override,
                DUT.global_mem_stall, DUT.actual_jump_target, DUT.alu_out);
            last_pc = DUT.pc;
        end
        if (DUT.trap_taken)
            $display("t=%0t TRAP-TAKEN ex_pc(id_ex_pc)=%h trap_cause=%h timer_fires=%b external_fires=%b mtvec=%h",
                $time, DUT.id_ex_pc, DUT.trap_cause, DUT.timer_fires, DUT.external_fires, DUT.mtvec_out);
    end

    initial begin
        rst = 1;
        for (i = 0; i < 16384; i = i + 1)
            temp_mem[i] = 32'h00000013;
        $readmemh("../fpga/zephyr_isr_test.mem", temp_mem);
        for (i = 0; i < 16384; i = i + 1) begin
            DUT.TCM.mem_even[i] = temp_mem[i][15:0];
            DUT.TCM.mem_odd[i]  = temp_mem[i][31:16];
        end

        repeat(3) @(posedge clk);
        rst = 0;

        // give the CPU time to run _start's setup (mtvec/intc enable/mie/mstatus)
        # 5000;
        $display("t=%0t sending 0x5A ('Z') on uart_rx", $time);
        send_byte(8'h5A);

        // capture the echoed byte back out on uart_tx
        wait (uart_tx_pin === 1'b0); // start bit
        # (CLOCKS_PER_BIT * 20 * 3 / 2); // to mid of first data bit
        begin : capture
            reg [7:0] got;
            integer bi;
            for (bi = 0; bi < 8; bi = bi + 1) begin
                got[bi] = uart_tx_pin;
                # (CLOCKS_PER_BIT * 20);
            end
            if (got === 8'h5A)
                $display("PASS: echoed byte = 0x%h", got);
            else
                $display("FAIL: echoed byte = 0x%h, expected 0x5A", got);
        end

        # 2000;
        $display("mcause after isr = %h  intc.pending=%b  mstatus_mie=%b",
            DUT.CSR.mcause, DUT.INTC.pending, DUT.mstatus_mie);

        for (i = 0; i < 60; i = i + 1) begin
            @(posedge clk);
            $display("t=%0t pc=%h trap_taken=%b timer_fires=%b external_fires=%b intc_pending=%b intc_enable=%b mstatus=%h",
                $time, DUT.pc, DUT.trap_taken, DUT.timer_fires, DUT.external_fires,
                DUT.INTC.pending, DUT.INTC.enable, DUT.CSR.mstatus);
        end

        // Confirm the CPU actually settled back into wait_loop (0x40000024)
        // and isn't storming on a repeated/uncleared interrupt.
        if (DUT.pc == 32'h40000024 || DUT.pc == 32'h40000026)
            $display("PASS: back in wait_loop, pc=%h", DUT.pc);
        else
            $display("CHECK: pc=%h (expected to have settled around 0x40000024)", DUT.pc);

        $display("FINAL leds=%b", leds);
        $finish;
    end

    initial begin
        #200_000;
        $display("WATCHDOG timeout pc=%h leds=%b mcause=%h pending=%b",
            DUT.pc, leds, DUT.CSR.mcause, DUT.INTC.pending);
        $finish;
    end
endmodule
