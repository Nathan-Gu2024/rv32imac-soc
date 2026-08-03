 `timescale 1ns/1ps

module tcm #( 
    parameter ADDR_WIDTH = 32, 
    parameter TCM_BASE = 32'h4000_0000, 
    parameter TCM_BYTES = 65536, 
    parameter INIT_FILE = "main.mem"
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

    // fill every word with a NOP before loading the real program to prevent corruption when fetchcing
    integer nop_i;
    initial begin
        for (nop_i = 0; nop_i < TCM_WORDS; nop_i = nop_i + 1)
            mem[nop_i] = 32'h00000013;
    end

    initial begin
        if (INIT_FILE != "") begin
            $display("Loading TCM file %s", INIT_FILE);
            $readmemh(INIT_FILE, mem);
        end
    end
    

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
    
    // Port B (Data) Write masking and Synchronous Read/Write
    always @(posedge clk) begin
        if (rst) begin
            d_rdata <= 32'b0;
            d_ready <= 1'b0;
        end else begin
            d_ready <= 1'b0;

            if (d_req && d_in_range) begin
                if (d_we) begin
                    // BRAM inferred byte-enable writes
                    if (d_wmask[0]) mem[d_index][7:0]   <= d_wdata[7:0];
                    if (d_wmask[1]) mem[d_index][15:8]  <= d_wdata[15:8];
                    if (d_wmask[2]) mem[d_index][23:16] <= d_wdata[23:16];
                    if (d_wmask[3]) mem[d_index][31:24] <= d_wdata[31:24];
                end
                
                // Synchronous read (will map to BRAM read port)
                d_rdata <= mem[d_index];
                d_ready <= 1'b1;
            end 
        end 
    end 

    // Port A (Instruction) Synchronous Read
    always @(posedge clk) begin
        if (rst) begin
            i_rdata <= 32'b0;
            i_ready <= 1'b0;
        end else begin
            i_ready <= 1'b0;
            
            if (i_req && i_in_range) begin
                // Synchronous read (will map to second BRAM read port)
                i_rdata <= mem[i_index]; 
                i_ready <= 1'b1;
            end 
        end 
    end

    always @(posedge clk) begin
        if (d_req)
            $display("TCM %s addr=%h data=%h",
                     d_we ? "WRITE" : "READ",
                     d_addr,
                     d_we ? d_wdata : d_rdata);
    end
    // sim
//    `ifndef SYNTHESIS
//    integer i;
//    initial begin
//        for (i = 0; i < TCM_WORDS; i = i + 1) begin
//            mem[i] = 32'h0000_0000;
//        end
//    end
//    `endif
    
endmodule