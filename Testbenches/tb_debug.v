`include "../src/cpu.v"

module tb_debug;
    reg clk = 0, rst;
    reg uart_tx_ready = 1;
    wire uart_tx_start;
    wire [7:0] uart_tx_data;
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
        .uart_tx_ready(uart_tx_ready), .uart_tx_start(uart_tx_start), .uart_tx_data(uart_tx_data),
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

    always #5 clk = ~clk;

    reg [31:0] words [0:154];
    integer i;

    initial begin
`include "mem_words_v7.vh"

        rst = 1;
        for (i = 0; i < 155; i = i + 1)
            DUT.TCM.mem[i] = words[i];

        repeat(3) @(posedge clk);
        rst = 0;

        repeat(20000) begin
            @(posedge clk);
            if (DUT.is_led && DUT.store_commits && !DUT.global_mem_stall)
                $display("t=%0t LED WRITE value=%0d (0b%b) pc=%h", $time, DUT.ex_mem_rs2[3:0], DUT.ex_mem_rs2[3:0], DUT.ex_mem_pc);
        end
        $display("FINAL leds=%b", leds);
        $finish;
    end
endmodule
