module fpga_top (
    input wire clk_hz,
    input wire [1:0] btn, 
    output wire [3:0] led
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

    cpu_pipelined CPU (
        .clk(cpu_clk),
        .rst(clean_rst),
        .leds(led)
    ); 

endmodule