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
    output reg [31:0] cpu_rdata,
    output reg cpu_ready,

    // TCM direct BRAM path
    output wire tcm_req_valid, tcm_req_write,
    output wire [ADDR_WIDTH-1:0] tcm_req_addr,
    output wire [31:0] tcm_wdata,
    output wire [3:0] tcm_wmask,
    input wire [31:0] tcm_rdata,
    input wire tcm_ready,

    // Lower-memory line interface, later wrapped by AXI
    output wire mem_req_valid, mem_req_write,
    output wire [ADDR_WIDTH-1:0] mem_req_addr,
    output wire [LINE_BYTES*8-1:0] mem_wline,
    input wire [LINE_BYTES*8-1:0] mem_rline,
    input wire mem_ready
);
    localparam LINE_BITS = LINE_BYTES * 8;

    localparam S_IDLE = 2'd0;
    localparam S_WAIT_TCM = 2'd1;
    localparam S_WAIT_CACHE = 2'd2;

    reg [1:0] state;

    reg [ADDR_WIDTH-1:0] saved_addr;
    reg saved_write;
    reg [31:0] saved_wdata;
    reg [3:0] saved_wmask;
    reg [1:0] saved_word_offset;
    reg core_req_sent;

    wire cpu_addr_is_tcm = (cpu_req_addr >= TCM_BASE) &&
                           (cpu_req_addr < (TCM_BASE + TCM_BYTES));

    // TCM request path
    assign tcm_req_valid = (state == S_WAIT_TCM);
    assign tcm_req_write = saved_write;
    assign tcm_req_addr = saved_addr;
    assign tcm_wdata = saved_wdata;
    assign tcm_wmask = saved_wmask;

    // Cache-line store expansion
    reg [LINE_BITS-1:0] cache_wline;
    reg [LINE_BYTES-1:0] cache_wmask;

    always @(*) begin
        cache_wline = {LINE_BITS{1'b0}};
        cache_wmask = {LINE_BYTES{1'b0}};

        case (saved_word_offset)
            2'd0: begin
                cache_wline[31:0] = saved_wdata;
                cache_wmask[3:0]= saved_wmask;
            end
            2'd1: begin
                cache_wline[63:32] = saved_wdata;
                cache_wmask[7:4] = saved_wmask;
            end
            2'd2: begin
                cache_wline[95:64] = saved_wdata;
                cache_wmask[11:8] = saved_wmask;
            end
            2'd3: begin
                cache_wline[127:96] = saved_wdata;
                cache_wmask[15:12] = saved_wmask;
            end
            default: begin
                cache_wline = {LINE_BITS{1'b0}};
                cache_wmask = {LINE_BYTES{1'b0}};
            end
        endcase
    end

    // Cache core
    wire [LINE_BITS-1:0] cache_rline;
    wire cache_ready;
    wire  cache_hit;

    wire core_req_valid = (state == S_WAIT_CACHE) && !core_req_sent;

    cache_core #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BYTES(LINE_BYTES),
        .NUM_SETS(NUM_SETS),
        .NUM_WAYS(NUM_WAYS)
    ) core_inst (
        .clk(clk),
        .rst(rst),

        .req_valid(core_req_valid),
        .req_write(saved_write),
        .req_addr(saved_addr),
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

    function [31:0] select_word;
        input [LINE_BITS-1:0] line;
        input [1:0] offset;
        begin
            case (offset)
                2'd0: select_word = line[31:0];
                2'd1: select_word = line[63:32];
                2'd2: select_word = line[95:64];
                2'd3: select_word = line[127:96];
                default: select_word = 32'h0;
            endcase
        end
    endfunction

    // Request/response FSM
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            saved_addr <= {ADDR_WIDTH{1'b0}};
            saved_write <= 1'b0;
            saved_wdata <= 32'h0;
            saved_wmask <= 4'h0;
            saved_word_offset <= 2'b00;
            core_req_sent <= 1'b0;
            cpu_rdata <= 32'h0;
            cpu_ready <= 1'b0;
        end else begin
            cpu_ready <= 1'b0;
            case (state)
                S_IDLE: begin
                    core_req_sent <= 1'b0;
                    if (cpu_req_valid) begin
                        saved_addr <= cpu_req_addr;
                        saved_write <= cpu_req_write;
                        saved_wdata <= cpu_wdata;
                        saved_wmask <= cpu_wmask;
                        saved_word_offset <= cpu_req_addr[3:2];

                        if (cpu_addr_is_tcm) begin
                            state <= S_WAIT_TCM;
                        end else begin
                            state <= S_WAIT_CACHE;
                        end
                    end
                end
                S_WAIT_TCM: begin
                    if (tcm_ready) begin
                        if (!saved_write) begin
                            cpu_rdata <= tcm_rdata;
                        end
                        cpu_ready <= 1'b1;
                        state <= S_IDLE;
                    end
                end
                S_WAIT_CACHE: begin
                    if (!core_req_sent) begin
                        core_req_sent <= 1'b1;
                    end
                    if (cache_ready) begin
                        if (!saved_write) begin
                            cpu_rdata <= select_word(cache_rline, saved_word_offset);
                        end
                        cpu_ready <= 1'b1;
                        core_req_sent <= 1'b0;
                        state <= S_IDLE;
                    end
                end
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule