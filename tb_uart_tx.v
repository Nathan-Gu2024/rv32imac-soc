`timescale 1ns/1ps
// Standalone testbench for uart_tx (NOT instantiated through fpga_top,
// since we want a clean 50MHz reference clock and don't want to wait
// on the .xdc / divided cpu_clk)

// Sequence:
//   1. Drive a 50MHz clock.
//   2. Hold rst high a few cycles, then release it.
//   3. Load tx_data = 8'h48 ('H').
//   4. Pulse tx_start high for exactly one clock cycle.
//   5. Self-check tx: start bit (0), 8 data bits LSB-first
//      (0,0,0,1,0,0,1,0 for 0x48), stop bit (1), then tx_ready
//      returning high.

module tb_uart_tx;

    parameter CLK_FREQ = 50_000_000;
    parameter BAUD_RATE = 115200;

    localparam CLK_PERIOD = 1_000_000_000 / CLK_FREQ; // ns per clk (20ns @ 50MHz)
    localparam CLOCKS_PER_BIT = CLK_FREQ / BAUD_RATE; // matches DUT's internal localparam (434)
    localparam BIT_PERIOD = CLOCKS_PER_BIT * CLK_PERIOD; // ns per UART bit (8680ns)

    reg clk;
    reg rst;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx;
    wire tx_ready;

    reg [7:0] expected_byte;
    reg [7:0] received_byte;
    integer i;
    reg test_failed;

    // DUT
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) DUT (
        .clk(clk),
        .rst(rst),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .tx(tx),
        .tx_ready(tx_ready)
    );

    // 50MHz clock
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Optional VCD dump (handy outside Vivado's own waveform viewer)
    initial begin
        $dumpfile("tb_uart_tx.vcd");
        $dumpvars(0, tb_uart_tx);
    end

    // Watchdog: bail out if something hangs
    initial begin
        #(BIT_PERIOD * 15);
        $display("ERROR: TIMEOUT - frame did not complete as expected");
        $finish;
    end

    // Main stimulus + self-check
    initial begin
        $display("CLOCKS_PER_BIT = %0d, BIT_PERIOD = %0dns", CLOCKS_PER_BIT, BIT_PERIOD);

        rst = 1'b1;
        tx_start = 1'b0;
        tx_data = 8'h00;
        test_failed = 1'b0;

        // Hold reset for a few cycles
        repeat (4) @(negedge clk);
        rst = 1'b0;
        @(negedge clk);

        if (tx !== 1'b1 || tx_ready !== 1'b1) begin
            $display("FAIL: line not idle after reset (tx=%b tx_ready=%b)", tx, tx_ready);
            test_failed = 1'b1;
        end else begin
            $display("PASS: line idle (tx=1, tx_ready=1) after reset");
        end

        // Load the byte and pulse tx_start for exactly one clock cycle
        tx_data = 8'h48; // 'H'
        expected_byte = tx_data;
        @(negedge clk);
        tx_start = 1'b1;
        @(negedge clk);
        tx_start = 1'b0;
        $display("t=%0t: tx_start pulsed for one cycle, sending 8'h%02h", $time, tx_data);

        // ---- START BIT ----
        @(negedge tx); // wait for line to actually drop
        #(BIT_PERIOD/2);
        if (tx !== 1'b0) begin
            $display("FAIL: start bit expected 0, got %b", tx);
            test_failed = 1'b1;
        end else begin
            $display("PASS: start bit = 0");
        end

        // 8 DATA BITS, LSB first
        for (i = 0; i < 8; i = i + 1) begin
            #(BIT_PERIOD);
            received_byte[i] = tx;
            if (tx !== expected_byte[i]) begin
                $display("FAIL: data bit %0d expected %b, got %b", i, expected_byte[i], tx);
                test_failed = 1'b1;
            end else begin
                $display("PASS: data bit %0d = %b", i, tx);
            end
        end

        // STOP BIT
        #(BIT_PERIOD);
        if (tx !== 1'b1) begin
            $display("FAIL: stop bit expected 1, got %b", tx);
            test_failed = 1'b1;
        end else begin
            $display("PASS: stop bit = 1");
        end

        // tx_ready should return high once the frame is done
        @(posedge tx_ready);
        $display("PASS: tx_ready returned high at t=%0t, UART idle again", $time);

        $display("Received byte = 8'h%02h (expected 8'h%02h)", received_byte, expected_byte);
        if (!test_failed && received_byte === expected_byte)
            $display("RESULT: ALL CHECKS PASSED");
        else
            $display("RESULT: TEST FAILED - see log above");

        #(BIT_PERIOD * 2);
        $finish;
    end

endmodule