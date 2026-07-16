module fpga_top (
    input wire clk, 
    input wire rst, 
    input wire uart_tx_ready, 
    output wire uart_tx_start, 
    output wire [7:0] uart_tx_data, 
    output wire [3:0] leds
);

    // IMEM
    wire [31:0] imem_req_addr;
    wire imem_req_valid;
    wire [127:0] imem_read_data;
    wire imem_ready;

    // DMEM
    wire [31:0] dmem_req_addr;
    wire [31:0] dmem_write_data;
    wire [3:0] dmem_write_mask;
    wire dmem_req_valid;
    wire [127:0] dmem_read_data_block;
    wire dmem_ready;

    cpu_pipelined CPU_CORE (
        .clk(clk),
        .rst(rst),
        .uart_tx_ready(uart_tx_ready),
        .uart_tx_start(uart_tx_start),
        .uart_tx_data(uart_tx_data),
        .leds(leds),
        .icache_mem_req_addr(imem_req_addr),
        .icache_mem_req_valid(imem_req_valid),
        .icache_mem_read_data(imem_read_data),
        .icache_mem_ready(imem_ready),
        .dmem_req_addr(dmem_req_addr),
        .store_data(dmem_write_data),
        .mem_write_mask(dmem_write_mask),
        .dcache_mem_req_valid(dmem_req_valid),
        .dcache_mem_read_data_block(dmem_read_data_block),
        .dcache_mem_ready(dmem_ready)
    );

    imem IMEM (
        .clk(clk),
        .rst(rst),
        .mem_req_valid(imem_req_valid),
        .mem_req_addr(imem_req_addr),
        .mem_read_data(imem_read_data),
        .mem_ready(imem_ready)
    );

    dmem DMEM (
        .clk(clk),
        .mem_req_valid(dmem_req_valid),
        .mem_address(dmem_req_addr),
        .mem_write_data(dmem_write_data),
        .mem_write_mask(dmem_write_mask),
        .mem_read_data_block(dmem_read_data_block),
        .mem_ready(dmem_ready)
    );

endmodule