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
                    if (rx == 1'b0) begin // Start bit edge
                        state <= START;
                        clk_count <= CLKS_PER_BIT / 2; // Wait half a bit period
                    end
                end
                START: begin
                    if (clk_count == 0) begin
                        if (rx == 1'b0) begin // Confirm start bit
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
                        rx_data[bit_index] <= rx;
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