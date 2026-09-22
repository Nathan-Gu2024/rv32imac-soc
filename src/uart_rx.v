`ifndef _UART_RX_V_
`define _UART_RX_V_

module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst,
    input wire rx,
    output reg [7:0] rx_data,
    output reg rx_valid
);
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;
    
    reg [1:0] state;
    reg [31:0] clk_count;
    reg [2:0] bit_index;

    // Two-flop synchroniser on the rx PAD.
    //
    // rx arrives from outside the clock domain entirely - fpga_top.v wires the
    // pin straight through - and it was previously sampled directly by the FSM
    // below. A metastable capture can both false-trigger the start-bit detect in
    // IDLE and corrupt a sampled data bit, and neither failure is reported: the
    // byte simply arrives wrong, or a frame begins where there was none.
    //
    // Two stages is the standard minimum, and the cost here is genuinely
    // nothing: the FSM samples at CLKS_PER_BIT/2 = 260 clocks into each bit at
    // 115200/60 MHz, so two cycles of added latency is 0.8% of a bit period,
    // far inside the sampling margin. rx_sync is what the FSM must use - never
    // the raw pad.
    reg rx_meta, rx_sync;
    always @(posedge clk) begin
        if (rst) begin
            // Idle high, so a reset does not look like a start bit.
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            rx_valid <= 1'b0;
            rx_data <= 8'b0;
            clk_count <= 32'b0;
            bit_index <= 3'b0;
        end else begin
            rx_valid <= 1'b0; // Default to low
            
            case (state)
                IDLE: begin
                    if (rx_sync == 1'b0) begin // Start bit edge
                        state <= START;
                        clk_count <= CLKS_PER_BIT / 2; // Wait half a bit period
                    end
                end
                START: begin
                    if (clk_count == 0) begin
                        if (rx_sync == 1'b0) begin // Confirm start bit
                            state <= DATA;
                            clk_count <= CLKS_PER_BIT;
                            bit_index <= 0;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        clk_count <= clk_count - 1;
                    end
                end
                DATA: begin
                    if (clk_count == 0) begin
                        rx_data[bit_index] <= rx_sync;
                        clk_count <= CLKS_PER_BIT;
                        if (bit_index == 7)
                            state <= STOP;
                        else
                            bit_index <= bit_index + 1;
                    end else begin
                        clk_count <= clk_count - 1;
                    end
                end
                STOP: begin
                    if (clk_count == 0) begin
                        rx_valid <= 1'b1; // Output valid pulse
                        state <= IDLE;
                    end else begin
                        clk_count <= clk_count - 1;
                    end
                end
            endcase
        end
    end
    
endmodule

`endif // _UART_RX_V_
