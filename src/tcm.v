`timescale 1ns/1ps

// Banked instruction memory: mem_even holds bits[15:0] and mem_odd holds
// bits[31:16] of each 32-bit-aligned word, as two independently-addressed
// 16-bit BRAMs. Because the two banks can be read with two DIFFERENT
// indices in the same cycle, a 32-bit instruction that straddles a word
// boundary (the low half in one word, the high half in the next - the
// common case once compressed 16-bit instructions are mixed in) can be
// fetched and stitched in a single cycle, instead of the two-cycle
// fetch-buffer stall the plain `tcm` module needs for that case.
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

    // 1. Banked Memory Arrays (Block RAM)
    (* ram_style = "block" *) reg [15:0] mem_even [0 : TCM_WORDS - 1];
    (* ram_style = "block" *) reg [15:0] mem_odd [0 : TCM_WORDS - 1];

    // 2. Initialization (NOPs + Readmemh)
    reg [31:0] temp_mem [0 : TCM_WORDS - 1];
    integer i;

    initial begin
        // Fill every word with a NOP before loading the real program
        for (i = 0; i < TCM_WORDS; i = i + 1) begin
            temp_mem[i] = 32'h00000013;
        end

        // Load the actual program
        if (INIT_FILE != "") begin
            $display("Loading TCM file %s", INIT_FILE);
            $readmemh(INIT_FILE, temp_mem);
        end

        // Split the 32-bit temporary array into the 16-bit physical banks
        for (i = 0; i < TCM_WORDS; i = i + 1) begin
            mem_even[i] = temp_mem[i][15:0];
            mem_odd[i] = temp_mem[i][31:16];
        end
    end

    // 3. Address Decoding & Alignment Logic
    wire [ADDR_WIDTH-1:0] i_offset = i_addr - TCM_BASE;
    wire [ADDR_WIDTH-1:0] d_offset = d_addr - TCM_BASE;

    wire [INDEX_BITS-1:0] i_index = i_offset[INDEX_BITS+1:2];
    wire [INDEX_BITS-1:0] d_index = d_offset[INDEX_BITS+1:2];

    wire i_in_range = (i_addr >= TCM_BASE) && (i_addr < (TCM_BASE + TCM_BYTES));
    wire d_in_range = (d_addr >= TCM_BASE) && (d_addr < (TCM_BASE + TCM_BYTES));

    // Instruction alignment calculations
    wire is_unaligned = i_offset[1];
    wire [INDEX_BITS-1:0] even_idx = is_unaligned ? (i_index + 1) : i_index;
    wire [INDEX_BITS-1:0] odd_idx = i_index;

    // Registers to hold BRAM outputs (1 cycle delay)
    reg [15:0] i_rdata_even, i_rdata_odd;
    reg i_is_unaligned_reg;

    reg [15:0] d_rdata_even, d_rdata_odd;

    // Port A: Instruction Fetch (Synchronous)
    always @(posedge clk) begin
        if (rst) begin
            i_rdata_even <= 16'b0;
            i_rdata_odd <= 16'b0;
            i_is_unaligned_reg <= 1'b0;
            i_ready <= 1'b0;
        end else begin
            i_ready <= 1'b0;
            if (i_req && i_in_range) begin
                i_rdata_even <= mem_even[even_idx];
                i_rdata_odd <= mem_odd[odd_idx];

                // Track if this specific fetch was unaligned so we can stitch correctly
                i_is_unaligned_reg <= is_unaligned;
                i_ready <= 1'b1;
            end
        end
    end

    // Stitch instruction combinationally after the synchronous BRAM read
    always @(*) begin
        if (i_is_unaligned_reg)
            i_rdata = {i_rdata_even, i_rdata_odd};
        else
            i_rdata = {i_rdata_odd, i_rdata_even};
    end

    // Port B: Data Memory (Synchronous)
    always @(posedge clk) begin
        if (rst) begin
            d_rdata_even <= 16'b0;
            d_rdata_odd <= 16'b0;
            d_ready <= 1'b0;
        end else begin
            d_ready <= 1'b0;
            if (d_req && d_in_range) begin
                if (d_we) begin
                    // BRAM inferred byte-enable writes mapped to the split banks
                    if (d_wmask[0]) mem_even[d_index][7:0] <= d_wdata[7:0];
                    if (d_wmask[1]) mem_even[d_index][15:8]<= d_wdata[15:8];
                    if (d_wmask[2]) mem_odd[d_index][7:0] <= d_wdata[23:16];
                    if (d_wmask[3]) mem_odd[d_index][15:8] <= d_wdata[31:24];
                end

                d_rdata_even <= mem_even[d_index];
                d_rdata_odd <= mem_odd[d_index];
                d_ready <= 1'b1;
            end
        end
    end

    // Stitch data combinationally (Data fetches in RV32 are always aligned)
    always @(*) begin
        d_rdata = {d_rdata_odd, d_rdata_even};
    end

    // Debug Display
    always @(posedge clk) begin
        if (d_req)
            $display("TCM %s addr=%h data=%h",
                     d_we ? "WRITE" : "READ",
                     d_addr,
                     d_we ? d_wdata : d_rdata);
    end

endmodule