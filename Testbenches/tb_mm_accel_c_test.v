`timescale 1ns/1ps
`include "../src/cpu.v"

// Sim pre-check for fpga/tests/test_amo.c before handing it to hardware -
// same mock memory harness as tb_coremark_sim.v, UART output captured the
// same way (snoop uart_mmio's tx_start/tx_data) since this test reports via
// UART print, not register peeks (unlike tb_amo_test.v).
module tb_mm_accel_c_test;
    reg clk, rst;
    wire uart_tx_line;
    reg uart_rx_line;
    wire [3:0] leds;

    wire [31:0] icache_mem_req_addr;
    wire icache_mem_req_valid;
    wire [127:0] icache_mem_read_data;
    wire icache_mem_ready;

    // mm_accel result-DMA port, the third requester on mem_arbiter in the real
    // SoC. Given its own mock line memory here so the C driver exercises the
    // DMA path, not only its register-window accesses.
    wire accel_mem_req_valid;
    wire accel_mem_req_write;
    wire [31:0] accel_mem_req_addr;
    wire [127:0] accel_mem_wline;
    wire accel_mem_ready;

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

    cpu_pipelined DUT (
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

        .accel_mem_req_valid(accel_mem_req_valid),
        .accel_mem_req_write(accel_mem_req_write),
        .accel_mem_req_addr(accel_mem_req_addr),
        .accel_mem_wline(accel_mem_wline),
        .accel_mem_ready(accel_mem_ready),

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
        $readmemh("test_mm_accel_c_mem.hex", ddr_mem);
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

    integer char_count = 0;
    always @(posedge clk) begin
        if (DUT.UART.tx_start) begin
            $write("%c", DUT.UART.tx_data);
            char_count = char_count + 1;
        end
    end

    // Mock line memory for the accelerator DMA. Deliberately not zero-latency:
    // the DMA line mux is registered, and a zero-wait responder is what exposed
    // the settle-cycle bug where dim>=8 published the previous group at the new
    // address.
    reg [2:0] a_lat_cnt;
    reg a_busy;
    reg [31:0] a_addr_latched;
    reg [127:0] a_wline_latched;
    always @(posedge clk) begin
        if (rst) begin
            a_busy <= 1'b0;
            a_lat_cnt <= 0;
        end else if (!a_busy && accel_mem_req_valid) begin
            a_busy <= 1'b1;
            a_lat_cnt <= 3;
            a_addr_latched <= accel_mem_req_addr;
            a_wline_latched <= accel_mem_wline;
        end else if (a_busy) begin
            if (a_lat_cnt == 0) a_busy <= 1'b0;
            else a_lat_cnt <= a_lat_cnt - 1;
        end
    end
    assign accel_mem_ready = a_busy && (a_lat_cnt == 0);
    always @(posedge clk) begin
        if (accel_mem_ready) begin
            ddr_mem[{a_addr_latched[28:4], 2'b00}] <= a_wline_latched[31:0];
            ddr_mem[{a_addr_latched[28:4], 2'b01}] <= a_wline_latched[63:32];
            ddr_mem[{a_addr_latched[28:4], 2'b10}] <= a_wline_latched[95:64];
            ddr_mem[{a_addr_latched[28:4], 2'b11}] <= a_wline_latched[127:96];
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

    integer quiet_cycles = 0;
    integer last_char_count = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (char_count != last_char_count) begin
                quiet_cycles = 0;
                last_char_count = char_count;
            end else if (char_count > 10) begin
                quiet_cycles = quiet_cycles + 1;
                if (quiet_cycles > 50000) begin
                    $display("\n[DONE] UART output quiesced");
                    $finish;
                end
            end
        end
    end

    initial begin
        #20_000_000;
        $display("\n[TIMEOUT] Simulation time budget exhausted");
        $finish;
    end

endmodule
