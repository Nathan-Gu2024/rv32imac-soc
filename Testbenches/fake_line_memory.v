`timescale 1ns/1ps

module fake_line_memory #(
    parameter ADDR_WIDTH = 32, 
    parameter LINE_BYTES = 16, 
    parameter DEPTH = 4096, 
    parameter LATENCY = 2
) (
    input wire clk, rst, 
    input wire mem_req_valid, mem_req_write, 
    input wire [ADDR_WIDTH - 1 : 0] mem_req_addr, 
    input wire [LINE_BYTES * 8 - 1 : 0] mem_wline, 
    output reg [LINE_BYTES * 8 - 1 : 0] mem_rline, 
    output wire mem_ready, 
    output reg [31:0] read_count, write_count, 
    output reg [ADDR_WIDTH - 1 : 0] last_write_addr, 
    output reg [LINE_BYTES * 8 - 1 : 0] last_write_line
); 
    localparam LINE_BITS = LINE_BYTES * 8; 

    reg [LINE_BITS - 1 : 0] mem [0 : DEPTH - 1]; 
    reg valid [0 : DEPTH - 1]; 
    reg busy, saved_write; 
    reg [ADDR_WIDTH - 1 : 0] saved_addr; 
    reg [LINE_BITS - 1 : 0] saved_wline; 
    integer countdown; 

    function [11:0] line_index; 
        input [ADDR_WIDTH - 1 : 0] addr; 
        begin
            line_index = addr[15:4]; 
        end 
    endfunction

    function [LINE_BITS - 1 : 0] default_line; 
        input [ADDR_WIDTH - 1 : 0] addr; 
        reg [ADDR_WIDTH - 1 : 0] aligned; 

        begin
            aligned = {addr[ADDR_WIDTH - 1 : 4], 4'b0000}; 
            // Little endian
            default_line = {
                aligned + 32'd12,
                aligned + 32'd8,
                aligned + 32'd4,
                aligned
            };
        end 
    endfunction

    assign mem_ready = busy && (countdown == 0); 

    integer i;
    initial begin
        for (i = 0; i  < DEPTH; i = i + 1) begin
            mem[i] = {LINE_BITS{1'b0}}; 
            valid[i] = 1'b0;
        end
    end 

    // Combinationally drive the requested read line
    always @(*) begin
        if (valid[line_index(saved_addr)]) begin
            // Return previously written data if valid
            mem_rline = mem[line_index(saved_addr)];
        end else begin
            // Return the dynamically generated testbench data
            mem_rline = default_line(saved_addr);
        end
    end
    
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; 
            saved_write <= 1'b0;
            saved_addr <= {ADDR_WIDTH{1'b0}}; 
            saved_wline <= {LINE_BITS{1'b0}};
            countdown <= 0;
            read_count <= 0; 
            write_count <= 0;
            last_write_addr <= {ADDR_WIDTH{1'b0}}; 
            last_write_line <= {LINE_BITS{1'b0}}; 
        end else begin
            if (busy) begin
                if (countdown > 0) begin 
                    countdown <= countdown - 1;
                end else begin
                    if (saved_write) begin
                        mem[line_index(saved_addr)] <= saved_wline; 
                        valid[line_index(saved_addr)] <= 1'b1;
                        write_count <= write_count + 1;
                        last_write_addr <= saved_addr; 
                        last_write_line <= saved_wline; 
                    end else begin
                        read_count <= read_count + 1;
                    end 
                    busy <= 1'b0;
                end 
            end else if (mem_req_valid) begin
                busy <= 1'b1;
                saved_write <= mem_req_write;
                saved_addr <= {mem_req_addr[ADDR_WIDTH - 1 : 4], 4'b0000}; 
                saved_wline <= mem_wline; 
                countdown <= LATENCY;
            end
        end 
    end 
    
endmodule