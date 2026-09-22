`ifndef _UART_TX_V_
`define _UART_TX_V_

module uart_tx #(
    parameter CLK_FREQ = 50_000_000,  
    parameter BAUD_RATE = 115200      // Standard PC terminal speed
)(
    input wire clk,
    input wire rst,
    input wire tx_start, // Pulse high for 1 cycle to send
    input wire [7:0] tx_data, // The ASCII character to send
    output reg tx, // The physical wire going to the PC
    output reg tx_ready // High when ready to accept new data
);

    // Calculate how many clock cycles are in one baud period
    localparam CLOCKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // FSM States
    localparam IDLE = 2'b00;
    localparam START = 2'b01;
    localparam DATA = 2'b10;
    localparam STOP = 2'b11;

    reg [1:0] state;
    reg [15:0] clock_count;
    reg [2:0] bit_index;
    reg [7:0] saved_data;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            tx <= 1'b1;
            tx_ready <= 1'b1;
            clock_count <= 0;
            bit_index <= 0;
            saved_data <= 0;
        end else begin
            case (state)
                IDLE: 
                    begin
                        tx <= 1'b1;
                        clock_count <= 0;
                        bit_index <= 0;
                        if (tx_start) begin
                            tx_ready <= 1'b0;     
                            saved_data <= tx_data;
                            state <= START;
                        end else begin
                            tx_ready <= 1'b1;
                        end
                    end
                START: 
                    begin
                        tx <= 1'b0; // Send Start Bit (0)
                        if (clock_count < CLOCKS_PER_BIT - 1) begin
                            clock_count <= clock_count + 1;
                        end else begin
                            clock_count <= 0;
                            state <= DATA;
                        end
                    end
                DATA: 
                    begin
                        tx <= saved_data[bit_index]; 
                        if (clock_count < CLOCKS_PER_BIT - 1) begin
                            clock_count <= clock_count + 1;
                        end else begin
                            clock_count <= 0;
                            if (bit_index < 7) begin
                                bit_index <= bit_index + 1;
                            end else begin
                                state <= STOP;
                            end
                        end
                    end
                STOP: 
                    begin
                        tx <= 1'b1; 
                        if (clock_count < CLOCKS_PER_BIT - 1) begin
                            clock_count <= clock_count + 1;
                        end else begin
                            clock_count <= 0;
                            state <= IDLE;
                        end
                    end 
                default: state <= IDLE;
            endcase
        end
    end
endmodule

`endif // _UART_TX_V_
