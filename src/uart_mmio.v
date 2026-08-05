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

    // Physical UART wires to/from the PC
    output wire tx,
    input wire rx,

    // Pulses high for 1 cycle whenever a transmit finishes (tx_ready's
    // 0->1 edge) - the interrupt controller latches this as "TX complete"
    output reg tx_irq,
    // rx_valid from uart_rx is already a single-cycle pulse, so this is a
    // direct passthrough - no extra edge detection needed like tx_irq above
    output wire rx_irq
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

    wire [7:0] rx_byte;
    wire rx_valid;

    uart_rx #(
        .CLK_FREQ(50_000_000),
        .BAUD_RATE(115200)
    ) rx_inst (
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .rx_data(rx_byte),
        .rx_valid(rx_valid)
    );

    assign rx_irq = rx_valid;

    // Edge-detect tx_ready so the interrupt fires once per completed byte,
    // not continuously while the line happens to be idle.
    reg tx_ready_prev;
    always @(posedge clk) begin
        if (rst) begin
            tx_ready_prev <= 1'b1;
            tx_irq <= 1'b0;
        end else begin
            tx_ready_prev <= tx_ready;
            tx_irq <= tx_ready && !tx_ready_prev;
        end
    end

    // Memory Map decoding
    wire is_uart_addr = (d_addr[31:12] == 20'h40001); // 0x4000_1XXX
    wire is_tx_data = (d_addr[11:0] == 12'h000); // 0x4000_1000
    wire is_tx_status = (d_addr[11:0] == 12'h004); // 0x4000_1004
    wire is_rx_data = (d_addr[11:0] == 12'h008); // 0x4000_1008
    wire is_rx_status = (d_addr[11:0] == 12'h00C); // 0x4000_100C

    reg [7:0] rx_holding;
    reg rx_has_data;

    always @(posedge clk) begin
        if (rst) begin
            tx_start <= 1'b0;
            tx_data <= 8'b0;
            d_ready <= 1'b0;
            d_rdata <= 32'b0;
            rx_holding <= 8'b0;
            rx_has_data <= 1'b0;
        end else begin
            // Default states: drop the start pulse and ready flag
            tx_start <= 1'b0;
            d_ready <= 1'b0;
            d_rdata <= 32'b0;

            // A newly-arrived byte always latches, on its own cycle,
            // independent of whatever bus transaction (if any) is
            // happening this same cycle.
            if (rx_valid) begin
                rx_holding <= rx_byte;
                rx_has_data <= 1'b1;
            end

            if (d_req && is_uart_addr) begin
                d_ready <= 1'b1; // Acknowledge the bus transaction

                if (d_we && is_tx_data) begin
                    // CPU wants to send a character
                    tx_data <= d_wdata[7:0];
                    tx_start <= 1'b1;
                end
                else if (!d_we) begin
                    if (is_tx_status)
                        d_rdata <= {31'b0, tx_ready};
                    else if (is_rx_data) begin
                        d_rdata <= {24'b0, rx_holding};
                        // Reading RX_DATA consumes it, unless a new byte is
                        // landing this exact cycle (handled above; that set
                        // wins so a back-to-back byte isn't lost).
                        if (!rx_valid) rx_has_data <= 1'b0;
                    end
                    else if (is_rx_status)
                        d_rdata <= {31'b0, rx_has_data};
                end
            end
        end
    end
endmodule
