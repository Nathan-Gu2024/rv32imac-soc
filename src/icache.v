`timescale 1ns/1ps

// Instruction-fetch front end: routes a fetch either to the TCM boot RAM
// (direct, no caching) or to the BRAM-backed cache (icache_bram.v).
//
// This used to hold the cache itself - a cache_core instance plus RVC
// straddle handling (line2_active/saved_addr/saved_last_half) and a
// next-line prefetcher. All of that is gone:
//   - the cache moved to icache_bram.v, which infers real Block RAM and so
//     can be 32KB instead of the 2KB the old combinational-read arrays
//     capped it at;
//   - straddle handling is free there, via even/odd halfword banking, so it
//     no longer costs a cycle or needs a state machine;
//   - the prefetcher is deleted. It only ever fired on repeat_hit (i.e.
//     while some OTHER stall froze the PC, leaving the single cache port
//     idle), but icache_bram uses its read port EVERY cycle for the
//     speculative next-PC lookup. There is no idle port left to steal, so
//     the prefetcher is structurally dead in this design - independent of
//     the fact that it only ever measured +0.13% on CoreMark.
module icache #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS = 2048,
    // Passed straight to icache_bram; see its header. 0 = inferred BRAM
    // (FPGA, testbenches), 1 = sky130 SRAM macros (ASIC, requires
    // NUM_SETS=512).
    parameter USE_SRAM_MACRO = 0,
    parameter TCM_BASE = 32'h4000_0000,
    parameter TCM_BYTES = 65536
) (
    input wire clk,
    input wire rst,

    // CPU instruction fetch request. cpu_req_addr_next is the value the PC
    // will hold next cycle - see icache_bram.v for why the cache is
    // addressed from it rather than from cpu_req_addr.
    input wire cpu_req_valid,
    input wire [ADDR_WIDTH-1:0] cpu_req_addr,
    input wire [ADDR_WIDTH-1:0] cpu_req_addr_next,
    output wire [31:0] cpu_rdata,
    output wire cpu_ready,

    // TCM instruction port
    output wire tcm_req_valid,
    output wire [ADDR_WIDTH-1:0] tcm_req_addr,
    input wire [31:0] tcm_rdata,
    input wire tcm_ready,

    // Lower-memory/cache-line read interface
    output wire mem_req_valid,
    output wire [ADDR_WIDTH-1:0] mem_req_addr,
    input wire [LINE_BYTES*8-1:0] mem_rline,
    input wire mem_ready
);
    localparam [ADDR_WIDTH-1:0] TCM_LIMIT = TCM_BASE + TCM_BYTES;

    wire req_is_tcm = (cpu_req_addr >= TCM_BASE) && (cpu_req_addr < TCM_LIMIT);

    wire [31:0] bram_rdata;
    wire bram_ready;

    icache_bram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BYTES(LINE_BYTES),
        .NUM_SETS(NUM_SETS),
        .USE_SRAM_MACRO(USE_SRAM_MACRO)
    ) BRAM_CACHE (
        .clk(clk),
        .rst(rst),

        .fetch_valid(cpu_req_valid && !req_is_tcm),
        .fetch_addr(cpu_req_addr),
        .fetch_addr_next(cpu_req_addr_next),
        .fetch_rdata(bram_rdata),
        .fetch_ready(bram_ready),

        .mem_req_valid(mem_req_valid),
        .mem_req_addr(mem_req_addr),
        .mem_rline(mem_rline),
        .mem_ready(mem_ready)
    );

    // TCM req-ack handshake: assert valid only on the first cycle of a
    // request so the synchronous TCM doesn't see an echo.
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

    assign tcm_req_valid = cpu_req_valid && req_is_tcm && !tcm_req_pending;
    assign tcm_req_addr = cpu_req_addr;

    assign cpu_rdata = req_is_tcm ? tcm_rdata : bram_rdata;
    assign cpu_ready = req_is_tcm ? tcm_ready : bram_ready;

endmodule
