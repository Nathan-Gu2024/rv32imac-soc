`include "../src/cpu.v"

module tb_irqtest;
    reg clk = 0, rst;
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

    always #10 clk = ~clk; // 50MHz -> 20ns period

    reg [31:0] temp_mem [0:16383];
    integer i;

    initial begin
        rst = 1;
        for (i = 0; i < 16384; i = i + 1)
            temp_mem[i] = 32'h00000013;
        $readmemh("../fpga/irqtest.mem", temp_mem);
        for (i = 0; i < 16384; i = i + 1) begin
            DUT.TCM.mem_even[i] = temp_mem[i][15:0];
            DUT.TCM.mem_odd[i]  = temp_mem[i][31:16];
        end

        repeat(3) @(posedge clk);
        rst = 0;

        // marker lives at 0x40000068 -> word index (0x68/4)=26
        wait (DUT.TCM.mem_even[26] !== 16'h0000 || DUT.TCM.mem_odd[26] !== 16'h0000);
        # 100;
        $display("marker = %h", {DUT.TCM.mem_odd[26], DUT.TCM.mem_even[26]});
        if ({DUT.TCM.mem_odd[26], DUT.TCM.mem_even[26]} === 32'h8000000B)
            $display("PASS: mcause == 0x8000000B (external interrupt, cause 11)");
        else
            $display("FAIL: unexpected mcause value");

        wait (leds !== 4'b0000);
        $display("PASS: LEDs show %b (low nibble of mcause) - main loop resumed after mret", leds);

        for (i = 0; i < 30; i = i + 1) begin
            @(posedge clk);
            $display("t=%0t pc=%h if_id_inst=%h trap_taken=%b timer_fires=%b external_fires=%b mepc=%h mstatus=%h mie=%h pending=%b uart_irq=%b tx_ready=%b",
                $time, DUT.pc, DUT.if_id_inst, DUT.trap_taken, DUT.timer_fires, DUT.external_fires,
                DUT.mepc_out, DUT.CSR.mstatus, DUT.CSR.mie, DUT.INTC.pending, DUT.uart_irq, DUT.UART.tx_ready);
        end
        $display("FINAL pc=%h leds=%b", DUT.pc, leds);
        $finish;
    end

    initial begin
        #200_000;
        $display("WATCHDOG timeout pc=%h leds=%b marker=%h mie=%b mstatus_mie=%b intc_irq_out=%b",
            DUT.pc, leds, {DUT.TCM.mem_odd[26], DUT.TCM.mem_even[26]},
            {DUT.mie_meie, DUT.mie_mtie}, DUT.mstatus_mie, DUT.intc_irq_out);
        $finish;
    end
endmodule
