`timescale 1ns/1ps
`include "../src/cpu.v"

// Reproduce echo_bot's environment in simulation: a Zephyr image running from
// DDR with BOTH caches active, and a real byte driven onto the UART RX pin.
//
// WHY THIS EXISTS
//
// echo_bot stopped working on hardware after a bitstream rebuild, and no
// existing bench covers what it does:
//
//   tb_coremark_sim   runs from DDR through both caches, but UART TX only and
//                     with interrupts never firing (INTC=0 in its report).
//   tb_uart_rx        drives the RX pin and takes the RX interrupt, but runs
//                     from TCM - so instruction fetch never touches the I-cache
//                     or the AXI line path at all.
//
// echo_bot is the intersection: DDR-resident code, both caches, interrupt-driven
// RX, and a TX echo. That intersection was untested, which is why simulation was
// green while the board was not.
//
// Build:
//   iverilog -g2005 -o /tmp/echo -s tb_echo_ddr Testbenches/tb_echo_ddr.v
//   vvp /tmp/echo
module tb_echo_ddr;
    reg clk = 0, rst;
    wire uart_tx_line;
    reg  uart_rx_line = 1'b1;
    wire [3:0] leds;

    wire [31:0]  icache_mem_req_addr;
    wire         icache_mem_req_valid;
    wire [127:0] icache_mem_read_data;
    wire         icache_mem_ready;

    wire [31:0]  dmem_req_addr;
    wire [31:0]  store_data;
    wire [3:0]   mem_write_mask;
    wire         dcache_mem_req_valid, dcache_mem_req_write;
    wire [127:0] dcache_mem_wline;
    wire [127:0] dcache_mem_read_data_block;
    wire         dcache_mem_ready;

    wire [31:0] debug_pc, debug_instr, debug_raw_pc;
    wire debug_dcache_valid, debug_dcache_ready;
    wire debug_tcm_d_req, debug_tcm_d_ready, debug_global_mem_stall;
    wire debug_id_predicted_taken, debug_cache_ready;

    cpu_pipelined DUT (
        .clk(clk), .rst(rst),
        .uart_tx(uart_tx_line), .uart_rx(uart_rx_line),
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
        // Split port; the accelerator is unused in this bench.
        .accel_mem_rd_ready(1'b0),
        .accel_mem_wr_ready(1'b0),
        .accel_mem_wnext(1'b0),
        .accel_mem_rline(128'h0),
        .debug_pc(debug_pc), .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall),
        .debug_raw_pc(debug_raw_pc),
        .debug_id_predicted_taken(debug_id_predicted_taken),
        .debug_cache_ready(debug_cache_ready)
    );

    // Boot stub: jumps to 0x00100000, which is where the image is loaded.
    defparam DUT.TCM.INIT_FILE = "../fpga/ddr_fixed.mem";

    // ---- DDR mock, shared by both cache line ports ----
    localparam DDR_WORDS = 524288;
    reg [31:0] ddr_mem [0:DDR_WORDS-1];
    integer zi;
    initial begin
        // ZERO FIRST, then load. The image covers ~3k words of a 512k-word
        // array, and Zephyr's .bss and stack live in DDR beyond it. Leaving the
        // rest at X makes the very first stack read return X, which propagates
        // into the PC and looks exactly like a boot failure - a false positive
        // from the model, not the design. Real DDR holds whatever was there
        // last; zero is the honest stand-in.
        for (zi = 0; zi < DDR_WORDS; zi = zi + 1) ddr_mem[zi] = 32'h0;
        $readmemh("/tmp/echo_bot_mem.hex", ddr_mem);
    end

    localparam LINE_LATENCY = 8;

    reg [3:0] i_lat; reg i_busy; reg [31:0] i_addr;
    always @(posedge clk) begin
        if (rst) begin i_busy <= 0; i_lat <= 0; end
        else if (!i_busy && icache_mem_req_valid) begin
            i_busy <= 1; i_lat <= LINE_LATENCY; i_addr <= icache_mem_req_addr;
        end else if (i_busy) begin
            if (i_lat == 0) i_busy <= 0; else i_lat <= i_lat - 1;
        end
    end
    assign icache_mem_ready = i_busy && (i_lat == 0);
    assign icache_mem_read_data = {ddr_mem[{i_addr[28:4],2'b11}],
                                   ddr_mem[{i_addr[28:4],2'b10}],
                                   ddr_mem[{i_addr[28:4],2'b01}],
                                   ddr_mem[{i_addr[28:4],2'b00}]};

    reg [3:0] d_lat; reg d_busy; reg [31:0] d_addr; reg d_wr;
    always @(posedge clk) begin
        if (rst) begin d_busy <= 0; d_lat <= 0; end
        else if (!d_busy && dcache_mem_req_valid) begin
            d_busy <= 1; d_lat <= LINE_LATENCY;
            d_addr <= dmem_req_addr; d_wr <= dcache_mem_req_write;
        end else if (d_busy) begin
            if (d_lat == 0) d_busy <= 0; else d_lat <= d_lat - 1;
        end
    end
    assign dcache_mem_ready = d_busy && (d_lat == 0);
    assign dcache_mem_read_data_block = {ddr_mem[{d_addr[28:4],2'b11}],
                                         ddr_mem[{d_addr[28:4],2'b10}],
                                         ddr_mem[{d_addr[28:4],2'b01}],
                                         ddr_mem[{d_addr[28:4],2'b00}]};
    always @(posedge clk) begin
        if (dcache_mem_ready && d_wr) begin
            ddr_mem[{d_addr[28:4],2'b00}] <= dcache_mem_wline[31:0];
            ddr_mem[{d_addr[28:4],2'b01}] <= dcache_mem_wline[63:32];
            ddr_mem[{d_addr[28:4],2'b10}] <= dcache_mem_wline[95:64];
            ddr_mem[{d_addr[28:4],2'b11}] <= dcache_mem_wline[127:96];
        end
    end

    // ---- probe: every cycle a misaligned load sits in EX ----
    // Shows whether the fault condition is even being formed, and what is
    // suppressing it if the trap never fires.
    integer probe_n = 0;
    always @(posedge clk) begin
        if (!rst && DUT.ex_misalign_load && probe_n < 12) begin
            $display("[probe] misalign_load=1 ex_valid=%b gms=%b stall=%b fault=%b trap_taken=%b pc=%h addr=%h",
                     DUT.id_ex_valid, DUT.global_mem_stall, DUT.stall,
                     DUT.TRAP_CTRL.fault_ld_align, DUT.trap_taken,
                     DUT.id_ex_pc, DUT.ex_data_addr);
            probe_n = probe_n + 1;
        end
    end

    // ---- TX capture by snooping uart_mmio, as tb_coremark_sim does ----
    integer tx_count = 0;
    reg [7:0] last_tx;
    always @(posedge clk) begin
        if (DUT.UART.tx_start) begin
            $write("%c", DUT.UART.tx_data);
            $fflush;
            last_tx  = DUT.UART.tx_data;
            tx_count = tx_count + 1;
        end
    end

    // ---- RX driver: 115200 at 60 MHz -> 520 clocks per bit ----
    localparam CPB = 520;
    task send_byte;
        input [7:0] b;
        integer k;
        begin
            uart_rx_line = 1'b0;                       // start
            repeat (CPB) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                uart_rx_line = b[k];
                repeat (CPB) @(posedge clk);
            end
            uart_rx_line = 1'b1;                       // stop
            repeat (CPB) @(posedge clk);
        end
    endtask

    always #10 clk = ~clk;                             // 50 MHz sim clock

    integer boot_tx;
    initial begin
        rst = 1;
        repeat (5) @(posedge clk);
        rst = 0;

        // Let Zephyr boot and print its banner.
        repeat (2_000_000) @(posedge clk);
        boot_tx = tx_count;
        $display("\n--- after boot: %0d chars transmitted ---", boot_tx);

        if (boot_tx == 0) begin
            $display("FAIL: no UART output at all - Zephyr did not reach its");
            $display("      banner. That is a boot failure, not an echo failure.");
            $display("      pc=%h  raw_pc=%h  global_mem_stall=%b",
                     debug_pc, debug_raw_pc, debug_global_mem_stall);
            $display("      stall_hung=%b", DUT.stall_hung);
            $finish;
        end

        // Now the actual echo test.
        //
        // Zephyr's echo_bot BUFFERS a line and only prints when it sees enter -
        // it does not echo each character. Sending a bare 'A' produces no TX and
        // looks like a broken echo path; the carriage return is what makes it
        // speak.
        $display("--- sending \"A\" + CR on RX ---");
        send_byte(8'h41);                  // 'A'
        repeat (20_000) @(posedge clk);
        send_byte(8'h0D);                  // CR - this is what triggers the echo
        repeat (600_000) @(posedge clk);

        $display("\n--- echo result ---");
        if (tx_count > boot_tx) begin
            $display("PASS: %0d char(s) echoed, last = 0x%02h ('%c')",
                     tx_count - boot_tx, last_tx, last_tx);
        end else begin
            $display("FAIL: byte received on RX produced NO TX - the echo path");
            $display("      is broken (RX capture, RX interrupt, or TX).");
            $display("      intc pending=%b enable=%b",
                     DUT.INTC.pending, DUT.INTC.enable);
            $display("      pc=%h  stall_hung=%b", debug_pc, DUT.stall_hung);
        end
        $finish;
    end
endmodule
