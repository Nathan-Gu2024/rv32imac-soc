`timescale 1ns/1ps

module dcache #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS = 64,
    parameter NUM_WAYS = 2,
    parameter TCM_BASE = 32'h4000_0000,
    parameter TCM_BYTES = 65536
) (
    input wire clk,
    input wire rst,

    // CPU-side 32-bit load/store request
    input wire cpu_req_valid, cpu_req_write,
    input wire [ADDR_WIDTH-1:0] cpu_req_addr,
    input wire [31:0] cpu_wdata,
    input wire [3:0] cpu_wmask,
    output wire [31:0] cpu_rdata,
    output wire cpu_ready,

    // TCM direct BRAM path
    output wire tcm_req_valid, tcm_req_write,
    output wire [ADDR_WIDTH-1:0] tcm_req_addr,
    output wire [31:0] tcm_wdata,
    output wire [3:0] tcm_wmask,
    input wire [31:0] tcm_rdata,
    input wire tcm_ready,

    // Lower-memory line interface
    output wire mem_req_valid, mem_req_write,
    output wire [ADDR_WIDTH-1:0] mem_req_addr,
    output wire [LINE_BYTES*8-1:0] mem_wline,
    input wire [LINE_BYTES*8-1:0] mem_rline,
    input wire mem_ready
);
    localparam LINE_BITS = LINE_BYTES * 8;
    localparam OFFSET_BITS = $clog2(LINE_BYTES);

    wire cpu_addr_is_tcm = (cpu_req_addr >= TCM_BASE) &&
                           (cpu_req_addr < (TCM_BASE + TCM_BYTES));

    // Route TCM requests with a 1-cycle pulse mask to prevent ghost writes/reads
    reg tcm_req_pending;
    always @(posedge clk) begin
        if (rst) begin
            tcm_req_pending <= 1'b0;
        end else if (tcm_req_valid) begin
            tcm_req_pending <= 1'b1;
        end else if (tcm_ready) begin
            tcm_req_pending <= 1'b0;
        end
    end

    assign tcm_req_valid = cpu_req_valid && cpu_addr_is_tcm && !tcm_req_pending;
    assign tcm_req_write = cpu_req_write;
    assign tcm_req_addr = cpu_req_addr;
    assign tcm_wdata = cpu_wdata;
    assign tcm_wmask = cpu_wmask;

    // 2. Instantly expand 32-bit CPU writes into 128-bit cache line writes
    wire [1:0] word_offset = cpu_req_addr[OFFSET_BITS-1:2];
    reg [LINE_BITS-1:0] cache_wline;
    reg [LINE_BYTES-1:0] cache_wmask;

    always @(*) begin
        cache_wline = {LINE_BITS{1'b0}};
        cache_wmask = {LINE_BYTES{1'b0}};

        case (word_offset)
            2'd0: begin 
                cache_wline[31:0] = cpu_wdata; 
                cache_wmask[3:0] = cpu_wmask; 
            end 2'd1: begin 
                cache_wline[63:32] = cpu_wdata; 
                cache_wmask[7:4] = cpu_wmask; 
            end 2'd2: begin 
                cache_wline[95:64] = cpu_wdata; 
                cache_wmask[11:8] = cpu_wmask; 
            end 2'd3: begin 
                cache_wline[127:96] = cpu_wdata; 
                cache_wmask[15:12] = cpu_wmask; 
            end
            default: begin 
                cache_wline = {LINE_BITS{1'b0}}; 
                cache_wmask = {LINE_BYTES{1'b0}}; 
            end
        endcase
    end

    // Instantiate Cache Core combinationally (TCM already routed above)
    wire core_req_valid = cpu_req_valid && !cpu_addr_is_tcm;
    wire [LINE_BITS-1:0] cache_rline;
    wire cache_ready, cache_hit;

    cache_core #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BYTES(LINE_BYTES),
        .NUM_SETS(NUM_SETS),
        .NUM_WAYS(NUM_WAYS)
    ) core_inst (
        .clk(clk),
        .rst(rst),
        .req_valid(core_req_valid),
        .req_write(cpu_req_write),
        .req_addr(cpu_req_addr),
        .req_wline(cache_wline),
        .req_wmask(cache_wmask),
        .mem_rline(mem_rline),
        .mem_ready(mem_ready),
        .resp_rline(cache_rline),
        .resp_ready(cache_ready),
        .hit(cache_hit),
        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline)
    );

    // Instantly multiplex the specific 32-bit word out of the 128-bit cache line
    reg [31:0] cache_rword;
    always @(*) begin
        case (word_offset)
            2'd0: cache_rword = cache_rline[31:0];
            2'd1: cache_rword = cache_rline[63:32];
            2'd2: cache_rword = cache_rline[95:64];
            2'd3: cache_rword = cache_rline[127:96];
            default: cache_rword = 32'h0;
        endcase
    end

    // Instantly return data and ready signal to CPU based on address target
    assign cpu_rdata = cpu_addr_is_tcm ? tcm_rdata : cache_rword;
    assign cpu_ready = cpu_addr_is_tcm ? tcm_ready : cache_ready;

endmodule