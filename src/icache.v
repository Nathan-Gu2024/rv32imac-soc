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

    wire cache_ready, cache_hit;
    wire [LINE_BITS-1:0] cache_rline;
    wire cache_result_ready_comb;
    wire [31:0] cache_result_data_comb;

    // Idle-cycle next-line prefetcher. Latches the last successfully-
    // answered CACHE-PATH (not TCM) address/data. When the CPU re-presents
    // this same address next cycle - which only happens while some OTHER
    // stall (dcache/div/uart/amo/accel) is freezing the PC, since
    // program_counter holds pc (and thus cpu_req_addr) constant during any
    // global_mem_stall - we already know the answer without asking
    // cache_core at all, freeing its single port for a prefetch with zero
    // risk to the real demand's correctness.
    reg [ADDR_WIDTH-1:0] last_addr;
    reg [31:0] last_data;
    reg last_valid;

    wire repeat_hit = last_valid && !req_is_tcm && cpu_req_valid && (cpu_req_addr == last_addr);

    // One prefetch attempt per line, off the *current* line (last_addr's
    // line) - reset once execution actually moves into a new line.
    wire [ADDR_WIDTH-1:0] current_line = {last_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    wire [ADDR_WIDTH-1:0] prefetch_target = current_line + LINE_BYTES;

    reg prefetched_for_this_line;
    reg [ADDR_WIDTH-1:0] prefetched_line;
    reg prefetch_in_flight;

    wire want_prefetch = repeat_hit & ~line2_active & ~prefetch_in_flight &
                          ~(prefetched_for_this_line & (current_line == prefetched_line));

    // Combinational, not just prefetch_in_flight: on the very cycle
    // want_prefetch fires, cache_req_addr already points at prefetch_target,
    // but prefetch_in_flight (a reg) hasn't latched yet. Gating only on the
    // registered flag would leave a one-cycle window where cache_core's
    // live resp_ready/hit/rline reflect the PREFETCH's address while the
    // outputs below still trust them for the real (frozen) cpu_req_addr -
    // reporting ready=1 with wrong data. prefetch_owns_port covers both
    // that launch cycle and, if it missed, the multi-cycle refill through
    // its RESPOND completion.
    wire prefetch_owns_port = want_prefetch | prefetch_in_flight;

    always @(posedge clk) begin
        if (rst) begin
            last_valid <= 1'b0;
        end else if (cpu_req_valid && !req_is_tcm && cache_result_ready_comb && !prefetch_owns_port) begin
            last_addr <= cpu_req_addr;
            last_data <= cache_result_data_comb;
            last_valid <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            prefetched_for_this_line <= 1'b0;
            prefetched_line <= {ADDR_WIDTH{1'b0}};
            prefetch_in_flight <= 1'b0;
        end else if (want_prefetch) begin
            prefetched_for_this_line <= 1'b1;
            prefetched_line <= current_line;
            // cache_hit reflects THIS cycle's presented request
            // (prefetch_target, driven into cache_core below) - if it's a
            // miss, cache_core latches it into WRITEBACK/REFILL_REQ and
            // won't re-examine req_valid/req_addr again until the refill
            // completes, so one cycle of presenting prefetch_target is
            // sufficient to hand it off.
            if (!cache_hit) prefetch_in_flight <= 1'b1;
        end else if (prefetch_in_flight && cache_ready) begin
            prefetch_in_flight <= 1'b0;
        end
    end

    wire cache_req_valid = line2_active  ? 1'b1 :
                            want_prefetch ? 1'b1 :
                                            (cpu_req_valid && !req_is_tcm);
    wire [ADDR_WIDTH-1:0] cache_req_addr = line2_active  ? (saved_addr + LINE_BYTES) :
                                            want_prefetch ? prefetch_target :
                                                             cpu_req_addr;

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
    // While a prefetch owns the port, cache_core's live outputs belong to
    // the prefetch, not the real cpu_req_addr - fall back to the
    // already-verified latched answer instead (see prefetch_owns_port
    // above; repeat_hit is guaranteed true whenever prefetch_owns_port is,
    // since want_prefetch itself requires repeat_hit).
    assign cache_result_data_comb = prefetch_owns_port ? last_data :
                                     line2_active        ? {cache_rline[15:0], saved_last_half} :
                                                            line1_window;
    assign cache_result_ready_comb = prefetch_owns_port ? repeat_hit :
                                      line2_active        ? cache_ready :
                                                             (cache_ready && !needs_line2);

    always @(posedge clk) begin
        if (rst) begin
            line2_active <= 1'b0;
            saved_addr <= {ADDR_WIDTH{1'b0}};
            saved_last_half <= 16'b0;
        end else if (!line2_active) begin
            // prefetch_owns_port guard: cache_ready/cache_rline can belong
            // to an in-flight prefetch (including its own RESPOND
            // completion cycle, after the mux has already reverted to
            // presenting the real cpu_req_addr) rather than the real
            // demand - without this, needs_line2 being coincidentally true
            // for the real (frozen) address at that moment would latch
            // saved_last_half from the prefetch's line instead.
            if (!prefetch_owns_port && cache_req_valid && cache_ready && needs_line2) begin
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