module imem (
    input wire clk, rst,
    input wire mem_req_valid,          
    input wire [31:0] mem_req_addr,
    output reg [127:0] mem_read_data,
    output reg mem_ready
);
    reg [31:0] rom [0:16383]; // 16KiB
    reg [2:0] delay_counter;

    initial begin
        $readmemh("C:\Users\natha\OneDrive\Desktop\FPGA\test.hex", rom);
    end 
    
    always @(posedge clk) begin
        if (rst) begin
            delay_counter <= 0;
            mem_ready <= 0;
            mem_read_data <= 128'b0;
        end else if (!mem_req_valid) begin
            delay_counter <= 0;
            mem_ready <= 0;
        end else begin
            if (delay_counter == 3'd4) begin
                mem_ready <= 1'b1;
                delay_counter <= 0;
                
                // Fetch 4 consecutive 32-bit words to build the 128-bit block
                mem_read_data <= {
                    rom[{mem_req_addr[15:4], 2'b11}],
                    rom[{mem_req_addr[15:4], 2'b10}], 
                    rom[{mem_req_addr[15:4], 2'b01}],
                    rom[{mem_req_addr[15:4], 2'b00}]
                };
            end else begin
                mem_ready <= 1'b0;
                delay_counter <= delay_counter + 1;
            end
        end
    end
endmodule