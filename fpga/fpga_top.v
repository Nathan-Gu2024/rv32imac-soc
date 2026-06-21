`include "uart_tx.v"

module fpga_top (
    input wire clk_hz,
    input wire [1:0] btn, 
    output wire [3:0] led,
    output wire uart_tx_out
);
    reg [3:0] clk_div;
    always @(posedge clk_hz) begin
        clk_div <= clk_div + 1;
    end 

    wire slow_clk = clk_div[3];
    wire cpu_clk;

    BUFG clk_buffer (
        .I(slow_clk),
        .O(cpu_clk)
    ); 

    reg [15:0] tick_counter;
      
    always @(posedge cpu_clk) begin
        if (tick_counter == 16'd62500)
            tick_counter <= 16'd0;
        else 
            tick_counter <= tick_counter + 1;
    end 

    wire m_tick = (tick_counter == 16'd62500);

    wire [3:0] clean_btn;    

    debouncer db0 (.clk(cpu_clk), .reset(1'b0), .sw(btn[0]), .m_tick(m_tick), .db(clean_btn[0]));
    debouncer db1 (.clk(cpu_clk), .reset(1'b0), .sw(btn[1]), .m_tick(m_tick), .db(clean_btn[1]));
    // debouncer db2 (.clk(cpu_clk), .reset(1'b0), .sw(btn[2]), .m_tick(m_tick), .db(clean_btn[2]));
    // debouncer db3 (.clk(cpu_clk), .reset(1'b0), .sw(btn[3]), .m_tick(m_tick), .db(clean_btn[3]));

    wire clean_rst = clean_btn[0];
    wire cpu_tx_start;
    wire [7:0] cpu_tx_data;
    wire tx_is_ready; 

    cpu_pipelined CPU (
        .clk(cpu_clk),
        .rst(clean_rst),
        .leds(led), 
        .uart_tx_start(cpu_tx_start),
        .uart_tx_data(cpu_tx_data), 
        .uart_tx_ready(tx_is_ready)
    ); 

    
    uart_tx #(
        .CLK_FREQ(50_000_000), 
        .BAUD_RATE(115200)
        ) UART (
        .clk(clk_hz), 
        .rst(clean_rst), 
        .tx_start(cpu_tx_start), 
        .tx_data(cpu_tx_data), 
        .tx(tx_is_ready), 
        .tx_ready(uart_tx_out)
    );

endmodule