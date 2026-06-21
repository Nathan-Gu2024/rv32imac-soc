module dmem (
    input wire clk, 
    input wire mem_req_valid,                     
    input wire [31:0] mem_address, mem_write_data,
    input wire [3:0] mem_write_mask,
    
    output wire [31:0] mem_read_data,
    output wire mem_ready,                        
    output wire [127:0] mem_read_data_block       
);
    wire [13:0] word_addr = mem_address[15:2];
    reg [31:0] ram [0:16383]; // 16KiB

    // Cache Interface 
    assign mem_ready = mem_req_valid;
    assign mem_read_data_block = {
        ram[word_addr + 3], 
        ram[word_addr + 2], 
        ram[word_addr + 1], 
        ram[word_addr]
    };
    
    assign mem_read_data = ram[word_addr];

    always @(posedge clk) begin
        if (mem_write_mask[0]) 
            ram[word_addr][7:0] <= mem_write_data[7:0];
        if (mem_write_mask[1]) 
            ram[word_addr][15:8] <= mem_write_data[15:8];
        if (mem_write_mask[2]) 
            ram[word_addr][23:16] <= mem_write_data[23:16];
        if (mem_write_mask[3]) 
            ram[word_addr][31:24] <= mem_write_data[31:24];
    end 
endmodule