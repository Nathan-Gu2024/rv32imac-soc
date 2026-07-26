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
    output reg [31:0] cpu_rdata,
//    output reg cpu_ready,
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
    localparam WORDS_PER_LINE = LINE_BYTES / 4;
    localparam WORD_SEL_BITS = $clog2(WORDS_PER_LINE);
    localparam OFFSET_BITS = $clog2(LINE_BYTES);

    localparam [ADDR_WIDTH-1:0] TCM_LIMIT = TCM_BASE + TCM_BYTES;

    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_START_TCM = 3'd1;
    localparam [2:0] S_WAIT_TCM = 3'd2;
    localparam [2:0] S_WAIT_CACHE = 3'd3;
    localparam [2:0] S_DONE = 3'd4;

    reg [2:0] state;
    reg [ADDR_WIDTH-1:0] saved_addr;
    reg [WORD_SEL_BITS-1:0] saved_word_offset;

    wire req_is_tcm_addr = (cpu_req_addr >= TCM_BASE) && (cpu_req_addr < TCM_LIMIT);

    wire cache_ready, cache_hit;
    wire [LINE_BITS-1:0] cache_rline;

    wire unused_mem_req_write;
    wire [LINE_BITS-1:0] unused_mem_wline;

    wire cache_req_valid = (state == S_WAIT_CACHE);

    assign tcm_req_valid = (state == S_WAIT_TCM);
    assign tcm_req_addr = saved_addr;

    reg core_req_valid, core_req_sent;

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
        .req_addr(saved_addr),
        .req_wline({LINE_BITS{1'b0}}),
        .req_wmask({LINE_BYTES{1'b0}}),

        .mem_rline(mem_rline),
        .mem_ready(mem_ready),

        .resp_rline(cache_rline),
        .resp_ready(cache_ready),
        .hit(cache_hit),

        .mem_req_valid(mem_req_valid),
        .mem_req_write(unused_mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_wline(unused_mem_wline)
    );
    assign cpu_ready = (state == S_DONE) && (cpu_req_addr == saved_addr);
    
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            saved_addr <= {ADDR_WIDTH{1'b0}};
            saved_word_offset <= {WORD_SEL_BITS{1'b0}};
            core_req_valid <= 1'b0;
            core_req_sent <= 1'b0;
            cpu_rdata <= 32'h0;
//            cpu_ready <= 1'b0;
        end else begin
//            cpu_ready <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (cpu_req_valid) begin
                        saved_addr <= cpu_req_addr;
                        saved_word_offset <= cpu_req_addr[OFFSET_BITS-1:2];
                        if (req_is_tcm_addr) begin
                            state <= S_WAIT_TCM;
                        end else begin
                            state <= S_WAIT_CACHE;
                        end
                    end
                end

                S_WAIT_CACHE: begin
                    if (!core_req_sent) begin
                        core_req_sent <= 1'b1;
                    end

                    if (cache_ready) begin
                        case (saved_word_offset)
                            2'd0: cpu_rdata <= cache_rline[31:0];
                            2'd1: cpu_rdata <= cache_rline[63:32];
                            2'd2: cpu_rdata <= cache_rline[95:64];
                            2'd3: cpu_rdata <= cache_rline[127:96];
                            default: cpu_rdata <= 32'h0;
                        endcase
//                        cpu_ready <= 1'b1;
                        core_req_sent <= 1'b0;
                        state <= S_DONE; // Transition to holding state
                    end
                end                
                
                S_START_TCM: begin
                    state <= S_WAIT_TCM;
                end

                S_WAIT_TCM: begin
                    if (tcm_ready) begin
                        cpu_rdata <= tcm_rdata;
//                        cpu_ready <= 1'b1;
                        state <= S_DONE; // Transition to holding state
                    end
                end
                
                S_DONE: begin
                    if (cpu_req_valid && (cpu_req_addr == saved_addr)) begin
                        // Hold ready high until the pipeline advances and the PC changes
//                        cpu_ready <= 1'b1; 
                    end else begin
//                        cpu_ready <= 1'b0;
                        // Immediately fetch the new instruction to avoid a 1-cycle penalty
                        if (cpu_req_valid) begin
                            saved_addr <= cpu_req_addr;
                            saved_word_offset <= cpu_req_addr[OFFSET_BITS-1:2];
                            if ((cpu_req_addr >= TCM_BASE) && (cpu_req_addr < TCM_LIMIT)) begin
                                state <= S_WAIT_TCM;
                            end else begin
                                state <= S_WAIT_CACHE;
                            end
                        end else begin
                            state <= S_IDLE;
                        end
                    end
                end

                default: begin
                    core_req_sent <= 1'b0;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule