`ifndef _RESERVATION_MONITOR_V_
`define _RESERVATION_MONITOR_V_

module reservation_monitor (
    input wire clk,
    input wire rst,
    input wire lr_en,        
    input wire sc_en,       
    input wire any_store_en,
    input wire trap_taken,
    input wire [31:0] mem_addr,
    output wire sc_successful 
);

    reg [31:0] reserved_addr;
    reg lock_valid;

    // SC succeeds ONLY if the lock is valid AND the target address matches the reservation
    assign sc_successful = lock_valid & (mem_addr == reserved_addr);
    always @(posedge clk) begin
        if (rst) begin
            lock_valid <= 1'b0;
            reserved_addr <= 32'b0;
        end else begin
            // Invalidate on interrupts/traps 
            if (trap_taken) begin
                lock_valid <= 1'b0;
            end
            
            // Load-Reserved places the reservation
            else if (lr_en) begin
                lock_valid <= 1'b1;
                reserved_addr <= mem_addr;
            end
            
            // Store-Conditional always clears the reservation
            else if (sc_en) begin
                lock_valid <= 1'b0;
            end
            
            // Any standard store invalidates the reservation
            else if (any_store_en) begin
                lock_valid <= 1'b0;
            end
        end
    end
endmodule

`endif // _RESERVATION_MONITOR_V_
