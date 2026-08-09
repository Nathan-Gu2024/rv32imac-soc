`timescale 1ns/1ps

module icache #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BYTES = 16,
    parameter NUM_SETS = 64,
    parameter NUM_WAYS = 2,
    parameter TCM_BASE = 32'h4000_0000,
    parameter TCM_BYTES = 65536
) (
    input wire clk,
    input wire rst,

    // CPU instruction fetch request
    input wire cpu_req_valid,
    input wire [ADDR_WIDTH-1:0] cpu_req_addr,
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
    localparam LINE_BITS = LINE_BYTES * 8;
    localparam OFFSET_BITS = $clog2(LINE_BYTES);
    localparam [ADDR_WIDTH-1:0] TCM_LIMIT = TCM_BASE + TCM_BYTES;

    // determine where the request should go
    wire req_is_tcm = (cpu_req_addr >= TCM_BASE) && (cpu_req_addr < TCM_LIMIT);

    // Cache-backed (DDR/AXI) path, cache_core hands back one whole
    // 128-bit line per access, stitches the potentially misaligned compressed instructions
    reg line2_active;
    reg [ADDR_WIDTH-1:0] saved_addr;
    reg [15:0] saved_last_half;

    wire [2:0] half_idx = cpu_req_addr[OFFSET_BITS-1:1];
    wire needs_line2 = (half_idx == 3'd7);

    wire cache_req_valid = line2_active ? 1'b1 : (cpu_req_valid && !req_is_tcm);
    wire [ADDR_WIDTH-1:0] cache_req_addr = line2_active ? (saved_addr + LINE_BYTES) : cpu_req_addr;

    wire cache_ready, cache_hit;
    wire [LINE_BITS-1:0] cache_rline;

    cache_core #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BYTES(LINE_BYTES),
        .NUM_SETS(NUM_SETS),
        .NUM_WAYS(NUM_WAYS)
    ) core (
        .clk(clk),
        .rst(rst),

        .req_valid(cache_req_valid),
        .req_write(1'b0),
        .req_addr(cache_req_addr),
        .req_wline({LINE_BITS{1'b0}}),
        .req_wmask({LINE_BYTES{1'b0}}),

        .mem_rline(mem_rline),
        .mem_ready(mem_ready),

        .resp_rline(cache_rline),
        .resp_ready(cache_ready),
        .hit(cache_hit),

        .mem_req_valid(mem_req_valid),
        .mem_req_write(), // Unused for I-Cache
        .mem_req_addr(mem_req_addr),
        .mem_wline() // Unused for I-Cache
    );

    // multiplex the 32-bit window starting at half_idx out of the 128-bit line
    reg [31:0] line1_window;
    always @(*) begin
        case (half_idx)
            3'd0: line1_window = cache_rline[31:0];
            3'd1: line1_window = cache_rline[47:16];
            3'd2: line1_window = cache_rline[63:32];
            3'd3: line1_window = cache_rline[79:48];
            3'd4: line1_window = cache_rline[95:64];
            3'd5: line1_window = cache_rline[111:80];
            3'd6: line1_window = cache_rline[127:96];
            default: line1_window = {16'b0, cache_rline[127:112]};
        endcase
    end

    // Combinational result passes straight through at cache_core's own
    // hit/miss timing for the common (non-straddling) case, same as
    // before. Only the straddling case needs the extra line2_active state.
    wire [31:0] cache_result_data_comb = line2_active ? {cache_rline[15:0], saved_last_half} : line1_window;
    wire cache_result_ready_comb = line2_active ? cache_ready : (cache_ready && !needs_line2);

    always @(posedge clk) begin
        if (rst) begin
            line2_active <= 1'b0;
            saved_addr <= {ADDR_WIDTH{1'b0}};
            saved_last_half <= 16'b0;
        end else if (!line2_active) begin
            if (cache_req_valid && cache_ready && needs_line2) begin
                saved_addr <= cpu_req_addr;
                saved_last_half <= cache_rline[127:112];
                line2_active <= 1'b1;
            end
        end else begin
            if (cache_ready) begin
                line2_active <= 1'b0;
            end
        end
    end

    // req-ack handshake
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

    // assert valid on the first cycle of the request
    assign tcm_req_valid = cpu_req_valid && req_is_tcm && !tcm_req_pending;
    assign tcm_req_addr = cpu_req_addr;
    
    assign cpu_rdata = req_is_tcm ? tcm_rdata : cache_result_data_comb;
    assign cpu_ready = req_is_tcm ? tcm_ready : cache_result_ready_comb;
    
endmodule