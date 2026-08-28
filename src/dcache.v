`timescale 1ns/1ps

// Data-side memory front end: routes a request either to the TCM (direct,
// uncached) or to the BRAM-backed cache (dcache_bram.v).
//
// This used to hold a cache_core instance plus the 32-bit-to-128-bit write
// expansion and the 128-bit-to-32-bit read extraction. All of that is gone:
// dcache_bram stores each line as four 32-bit banks, so a store writes one
// bank directly with native byte enables and a load selects one bank - no
// full-line marshalling in either direction.
module dcache #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS = 1024,
    parameter TCM_BASE = 32'h4000_0000,
    parameter TCM_BYTES = 65536
) (
    input wire clk,
    input wire rst,

    // CPU-side 32-bit load/store request. cpu_req_addr_next is the
    // index+offset the MEM stage will present next cycle - see
    // dcache_bram.v for why the arrays are addressed from it.
    input wire cpu_req_valid, cpu_req_write,
    input wire [ADDR_WIDTH-1:0] cpu_req_addr,
    input wire [$clog2(NUM_SETS)+$clog2(LINE_BYTES)-1:0] cpu_req_addr_next,
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
    wire cpu_addr_is_tcm = (cpu_req_addr >= TCM_BASE) &&
                           (cpu_req_addr < (TCM_BASE + TCM_BYTES));

    // Route TCM requests with a 1-cycle pulse mask to prevent ghost
    // writes/reads (unchanged).
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

    wire [31:0] bram_rdata;
    wire bram_ready;

    dcache_bram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BYTES(LINE_BYTES),
        .NUM_SETS(NUM_SETS)
    ) core_inst (
        .clk(clk),
        .rst(rst),

        .cpu_req_valid(cpu_req_valid && !cpu_addr_is_tcm),
        .cpu_req_write(cpu_req_write),
        .cpu_req_addr(cpu_req_addr),
        .cpu_req_addr_next(cpu_req_addr_next),
        .cpu_wdata(cpu_wdata),
        .cpu_wmask(cpu_wmask),
        .cpu_rdata(bram_rdata),
        .cpu_ready(bram_ready),

        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline),
        .mem_rline(mem_rline),
        .mem_ready(mem_ready)
    );

    assign cpu_rdata = cpu_addr_is_tcm ? tcm_rdata : bram_rdata;
    assign cpu_ready = cpu_addr_is_tcm ? tcm_ready : bram_ready;

endmodule
