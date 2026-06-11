module dmem (
    input wire clk,
    input wire [31:0] mem_address, mem_write_data,
    input wire [3:0] mem_write_mask,
    output wire [31:0] mem_read_data
);
    wire [13:0] word_addr = mem_address[15:2];
    reg [31:0] ram [0:16383]; // 16KiB

    // Read
    assign mem_read_data = ram[word_addr];

    // Write
    always @(posedge clk) begin
        if (mem_write_mask[0]) // 0th high -> write lowest byte
            ram[word_addr] [7:0] <= mem_write_data[7:0];
        if (mem_write_mask[1]) // 1st high -> write second byte
            ram[word_addr] [15:8] <= mem_write_data[15:8];
        if (mem_write_mask[2]) // 2nd high -> write third byte
            ram[word_addr] [23:16] <= mem_write_data[23:16];
        if (mem_write_mask[3]) // 3rd high -> write highest byte
            ram[word_addr] [31:24] <= mem_write_data[31:24];
    end 

endmodule