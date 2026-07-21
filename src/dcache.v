module dcache #(
    parameter ADDR_WIDTH = 32, 
    parameter LINE_BYTES = 16, 
    parameter TCM_BASE = 32'h4000_0000, 
    parameter TCM_MASK = 32'hFFFF_0000
  ) (
    input wire clk, rst,
    // cpu
    input wire cpu_req_valid, cpu_req_write, 
    input wire [ADDR_WIDTH - 1 : 0] cpu_req_addr, 
    input wire [31:0] cpu_wdata, 
    input wire [3:0] cpu_wmask, 
    output reg [31:0] cpu_rdata, 
    output reg cpu_ready, 

    // tcm (direct bram)
    output wire tcm_req_valid, tcm_req_write, 
    output wire [ADDR_WIDTH - 1 : 0] tcm_req_addr, 
    output wire [31:0] tcm_wdata, 
    output wire [3:0] tcm_wmask, 
    input wire [31:0] tcm_rdata, 
    input wire tcm_ready, 

    // lower mem (AXI)
    output wire mem_req_valid, mem_req_write, 
    output wire [ADDR_WIDTH - 1 : 0] mem_req_addr, 
    output wire [(LINE_BYTES * 8) - 1 : 0] mem_wline, 
    input wire [(LINE_BYTES * 8) - 1 : 0] mem_rline, 
    input wire mem_ready
  ); 

  localparam LINE_BITS = LINE_BYTES * 8;
  
  wire [1:0] word_offset = cpu_req_addr[3:2]; 
  // TCM bypass
  wire is_tcm_addr = (cpu_req_addr & TCM_MASK) == TCM_BASE;
  wire cache_req_valid = cpu_req_valid && !is_tcm_addr;
  assign tcm_req_valid = cpu_req_valid && is_tcm_addr;
  assign tcm_req_write = cpu_req_write; 
  assign tcm_req_addr = cpu_req_addr; 
  assign tcm_wdata = cpu_wdata; 
  assign tcm_wmask = cpu_wmask; 

    // Data and masking
    reg [LINE_BITS - 1 : 0] cache_wline;
    reg [LINE_BYTES - 1 : 0] cache_wmask; 

    always @(*) begin
        cache_wline = {LINE_BITS{1'b0}}; 
        cache_wmask = {LINE_BYTES{1'b0}};
        case (word_offset)
            2'b00: 
                begin
                    cache_wline[31:0] = cpu_wdata; 
                    cache_wmask[3:0] = cpu_wmask;
                end 
            2'b01:
                begin
                    cache_wline[63:32] = cpu_wdata; 
                    cache_wmask[7:4] = cpu_wmask;
                end 
            2'b10:
                begin
                    cache_wline[95:64] = cpu_wdata;
                    cache_wmask[11:8] = cpu_wmask;
                end 
            2'b11:
                begin
                    cache_wline[127:96] = cpu_wdata;
                    cache_wmask[15:12] = cpu_wmask;
                end 
            default: 
                begin
                    cache_wline = {LINE_BITS{1'b0}};
                    cache_wmask = {LINE_BYTES{1'b0}};
                end 
        endcase 
    end 

    // cache core 
     wire [LINE_BITS - 1 : 0] cache_rline;
     wire cache_ready, cache_hit;

     cache_core #(
        .ADDR_WIDTH(ADDR_WIDTH), 
        .LINE_BYTES(LINE_BYTES)
     ) core_inst ( 
        .clk(clk),
        .rst(rst),
        .req_valid(cache_req_valid),
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

    function [31:0] select_word;
        input [LINE_BITS - 1 : 0] line;
        input [1:0] offset; 
        begin
            case (offset) 
                2'b00: select_word = line[31:0]; 
                2'b01: select_word = line[63:32]; 
                2'b10: select_word = line[95:64]; 
                2'b11: select_word = line[127:96]; 
                default: select_word = 32'b0;
            endcase 
        end 
    endfunction

    reg [1:0] saved_word_offset; 
    reg waiting_for_cache_miss; 
    always @(posedge clk) begin
        if (rst) begin
            saved_word_offset <= 2'b00;
            waiting_for_cache_miss <= 1'b0;
        end else begin
            if (cache_req_valid && !cache_ready && !waiting_for_cache_miss) begin
                saved_word_offset <= word_offset; 
                waiting_for_cache_miss <= 1'b1;
            end else if (cache_ready) begin
                waiting_for_cache_miss <= 1'b0;
            end
        end 
    end 

    wire [1:0] read_word_offset = waiting_for_cache_miss ? saved_word_offset : word_offset;

    always @(*) begin
        cpu_rdata = 32'b0;
        cpu_ready = 1'b0;
        if (cpu_req_valid) begin
            if (is_tcm_addr) begin
                cpu_rdata = tcm_rdata;
                cpu_ready = tcm_ready;
            end else begin
                cpu_rdata = select_word(cache_rline, read_word_offset); 
                cpu_ready = cache_ready;
            end 
        end 
    end 
                    
endmodule