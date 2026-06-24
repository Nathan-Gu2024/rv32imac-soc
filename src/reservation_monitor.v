module reservation_monitor (
    input wire clk, rst, is_lr_mem, is_sc_mem, trap_taken,
    input wire [31:0] mem_req_addr, 
    output reg sc_success, 
    output wire block_sc_store
);
    reg [31:0] lock_addr; 
    reg lock_valid; 

    always @(posedge clk) begin
        if (rst || trap_taken) begin
            lock_valid <= 1'b0;
            lock_addr <= 32'b0;
            sc_success <= 1'b0;
        end else begin
            sc_success <= 1'b0;
            
            if (is_lr_mem) begin
                lock_addr <= mem_req_addr;
                lock_valid <= 1'b1;
            end else if (is_sc_mem) begin
                if (lock_valid && (lock_addr == mem_req_addr)) begin
                    sc_success <= 1'b1;
                end else begin
                    sc_success <= 1'b0;
                end 

                lock_valid <= 1'b0;
            end 
        end 
    end 
    // if SC instruction but lock invalid -> block the memory write
    assign block_sc_store = is_sc_mem && !(lock_valid && (lock_addr == mem_req_addr));

endmodule
