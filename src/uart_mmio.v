module uart_mmio (
    input wire clk,
    input wire rst,

    // CPU Data Bus Interface (Matching your TCM interface style)
    input wire d_req, 
    input wire d_we, 
    input wire [31:0] d_addr, 
    input wire [31:0] d_wdata,
    output reg [31:0] d_rdata, 
    output reg d_ready,

    // Physical UART wire going to the PC
    output wire tx
);

    // Wires to connect to your existing UART module
    wire tx_ready;
    reg tx_start;
    reg [7:0] tx_data;

    // Instantiate your exact UART module here
    uart_tx #(
        .CLK_FREQ(50_000_000),  
        .BAUD_RATE(115200)      
    ) tx_inst (
        .clk(clk),
        .rst(rst),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .tx(tx),
        .tx_ready(tx_ready)
    );

    // Memory Map decoding
    wire is_uart_addr = (d_addr[31:12] == 20'h40001); // 0x4000_1XXX
    wire is_tx_data = (d_addr[11:0] == 12'h000); // 0x4000_1000
    wire is_tx_status = (d_addr[11:0] == 12'h004); // 0x4000_1004

    always @(posedge clk) begin
        if (rst) begin
            tx_start <= 1'b0;
            tx_data <= 8'b0;
            d_ready <= 1'b0;
            d_rdata <= 32'b0;
        end else begin
            // Default states: drop the start pulse and ready flag
            tx_start <= 1'b0; 
            d_ready <= 1'b0;
            d_rdata <= 32'b0;

            if (d_req && is_uart_addr) begin
                d_ready <= 1'b1; // Acknowledge the bus transaction

                if (d_we && is_tx_data) begin
                    // CPU wants to send a character
                    tx_data <= d_wdata[7:0];
                    tx_start <= 1'b1; 
                end 
                else if (!d_we && is_tx_status) begin
                    // CPU is checking if the UART is busy
                    d_rdata <= {31'b0, tx_ready};
                end
            end
        end
    end
endmodule