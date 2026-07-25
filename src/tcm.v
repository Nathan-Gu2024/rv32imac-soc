 `timescale 1ns/1ps

module tcm #( 
    parameter ADDR_WIDTH = 32, 
    parameter TCM_BASE = 32'h4000_0000, 
    parameter TCM_BYTES = 65536
) (
    input wire clk, rst,
    // Port A, read, instructions
    input wire i_req, 
    input wire [ADDR_WIDTH - 1:0] i_addr, 
    output reg [31:0] i_rdata, 
    output reg i_ready, 

    // Port B, read/write, data
    input wire d_req, d_we, 
    input wire [ADDR_WIDTH - 1 : 0] d_addr, 
    input wire [31:0] d_wdata,
    input wire [3:0] d_wmask, 
    output reg [31:0] d_rdata, 
    output reg d_ready
); 
    localparam TCM_WORDS = TCM_BYTES / 4;
    localparam INDEX_BITS = $clog2(TCM_WORDS);

    // reg [31:0] mem [0 : TCM_WORDS - 1];
    (* ram_style = "block" *) reg [31:0] mem [0 : TCM_WORDS - 1];

    wire [ADDR_WIDTH - 1 : 0] i_offset_bytes = i_addr - TCM_BASE; 
    wire [ADDR_WIDTH - 1 : 0] d_offset_bytes = d_addr - TCM_BASE; 

    // wire [$clog2(TCM_WORDS) - 1 : 0] i_index = i_offset_bytes[INDEX_BITS + 1 : 2];
    // wire [$clog2(TCM_WORDS) - 1 : 0] d_index = d_offset_bytes[INDEX_BITS + 1 : 2];
    
    (* ram_style = "block" *) reg [31:0] mem [0 : TCM_WORDS - 1];

    wire [ADDR_WIDTH-1:0] i_offset = i_addr - TCM_BASE;
    wire [ADDR_WIDTH-1:0] d_offset = d_addr - TCM_BASE;

    wire [INDEX_BITS-1:0] i_index = i_offset[INDEX_BITS+1:2];
    wire [INDEX_BITS-1:0] d_index = d_offset[INDEX_BITS+1:2];

    wire i_in_range = (i_addr >= TCM_BASE) && (i_addr < (TCM_BASE + TCM_BYTES));
    wire d_in_range = (d_addr >= TCM_BASE) && (d_addr < (TCM_BASE + TCM_BYTES));


    reg [31:0] d_merged_word;
    always @(*) begin
        d_merged_word = mem[d_index]; 
        if (d_wmask[0]) d_merged_word[7:0] = d_wdata[7:0];
        if (d_wmask[1]) d_merged_word[15:8] = d_wdata[15:8];
        if (d_wmask[2]) d_merged_word[23:16] = d_wdata[23:16];
        if (d_wmask[3]) d_merged_word[31:24] = d_wdata[31:24];
    end 

    // Sync 1 cycle TCM; if inst and data ports touch same word 
    // -> return newly merged word for write-first
    always @(posedge clk) begin
        if (rst) begin
            i_rdata <= 32'b0;
            d_rdata <= 32'b0;
            i_ready <= 1'b0;
            d_ready <= 1'b0;
        end else begin
            i_ready <= 1'b0;
            d_ready <= 1'b0;

            if (d_req && d_in_range && d_we) begin 
                mem[d_index] <= d_merged_word;
            end 

            if (i_req && i_in_range) begin
                if (d_req && d_in_range && d_we && (i_index == d_index)) begin
                    i_rdata <= d_merged_word; 
                end else begin
                    i_rdata <= mem[i_index]; 
                end 
                i_ready <= 1'b1;
            end 

            if (d_req && d_in_range) begin
                d_rdata <= d_we ? d_merged_word : mem[d_index];
                d_ready <= 1'b1;
            end 
        end 
    end 

    // sim
    `ifndef SYNTHESIS
    integer i;
    initial begin
        for (i = 0; i < TCM_WORDS; i = i + 1) begin
            mem[i] = 32'h0000_0000;
        end
    end
    `endif
    
endmodule