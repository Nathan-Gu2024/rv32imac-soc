module mem_arbiter (
    input wire clk, 
    input wire rst,

    // icache 
    input wire icache_req_valid,
    input wire [31:0] icache_req_addr,
    output wire icache_ready,
    output wire [127:0] icache_rline,

    // dcache 
    input wire dcache_req_valid, dcache_req_write,
    input wire [31:0] dcache_req_addr,
    input wire [127:0] dcache_wline,
    output wire dcache_ready,
    output wire [127:0] dcache_rline,

    // main mem / DDR 
    output wire mem_req_valid, mem_req_write,
    output wire [31:0] mem_req_addr,
    output wire [127:0] mem_wline,
    input wire mem_ready,
    input wire [127:0] mem_rline
);

    localparam IDLE = 2'b00;
    localparam SVC_D = 2'b01;
    localparam SVC_I = 2'b10;

    (* mark_debug = "true" *) reg [1:0] state;

    reg saved_req_write;
    reg [31:0] saved_req_addr;
    reg [127:0] saved_wline;

    // Round-robin fairness: each cache can only ever have one outstanding
    // request at a time (both stall their own pipeline side until their
    // own mem_ready), so this never affects a lone requester - it only
    // matters on the rare cycle both are pending simultaneously, where
    // dcache used to always win outright. Not the fix for the confirmed
    // AXI_ERRS_RID hang (that's downstream in axi_cache_adapter, already
    // fixed there via timeout/retry), but removes a real, if narrow,
    // starvation risk under sustained concurrent traffic.
    (* mark_debug = "true" *) reg last_granted; // 0 = dcache granted last, 1 = icache granted last
    (* mark_debug = "true" *) wire grant_d = (state == IDLE) && dcache_req_valid && (!icache_req_valid || last_granted);
    (* mark_debug = "true" *) wire grant_i = (state == IDLE) && icache_req_valid && (!dcache_req_valid || !last_granted);

    // Debug-only mirrors of the input ports so they can be probed directly
    // (attributing multi-signal port declarations is awkward, so mirror
    // instead) - used to catch icache starvation behind a stuck/late-
    // clearing dcache_req_valid at the exact moment of a hang.
    (* mark_debug = "true" *) wire dbg_icache_req_valid = icache_req_valid;
    (* mark_debug = "true" *) wire dbg_dcache_req_valid = dcache_req_valid;
    (* mark_debug = "true" *) wire [31:0] dbg_icache_req_addr = icache_req_addr;
    (* mark_debug = "true" *) wire [31:0] dbg_dcache_req_addr = dcache_req_addr;
    (* mark_debug = "true" *) wire dbg_mem_ready = mem_ready;

    // Counts consecutive cycles icache has requested but not been granted -
    // saturates instead of wrapping, so a stuck/starved icache request
    // shows up as a large, non-zero value sitting still in the capture.
    (* mark_debug = "true" *) reg [15:0] icache_wait_cycles;
    always @(posedge clk) begin
        if (rst) begin
            icache_wait_cycles <= 16'd0;
        end else if (icache_req_valid && !grant_i && state != SVC_I) begin
            if (icache_wait_cycles != 16'hFFFF)
                icache_wait_cycles <= icache_wait_cycles + 16'd1;
        end else begin
            icache_wait_cycles <= 16'd0;
        end
    end

    // FSM
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            saved_req_write <= 1'b0;
            saved_req_addr <= 32'b0;
            saved_wline <= 128'b0;
            last_granted <= 1'b0;
        end else begin
            case (state)
                IDLE:
                    begin
                        if (grant_d) begin
                            state <= SVC_D;
                            saved_req_write <= dcache_req_write;
                            saved_req_addr <= dcache_req_addr;
                            saved_wline <= dcache_wline;
                            last_granted <= 1'b0;
                        end else if (grant_i) begin
                            state <= SVC_I;
                            saved_req_write <= 1'b0;
                            saved_req_addr <= icache_req_addr;
                            saved_wline <= 128'b0;
                            last_granted <= 1'b1;
                        end
                    end
                SVC_D: 
                    begin
                        if (mem_ready) begin
                            state <= IDLE;
                        end 
                    end
                SVC_I:
                    begin
                        if (mem_ready) begin
                            state <= IDLE; 
                        end 
                    end 
                default: state <= IDLE;
            endcase 
        end
    end


    // Drive exactly one stable request to the shared lower-memory path.
    assign mem_req_valid = (state == SVC_D) || (state == SVC_I);
    assign mem_req_write = (state == SVC_D) ? saved_req_write : 1'b0;
    assign mem_req_addr = ((state == SVC_D) || (state == SVC_I)) ? saved_req_addr : 32'b0;
    assign mem_wline = (state == SVC_D) ? saved_wline : 128'b0;

    // signals back to the caches
    assign dcache_ready = (state == SVC_D) & mem_ready;
    assign icache_ready = (state == SVC_I) & mem_ready;

    // safe to broadcast read data; ready signals gate actual latching
    assign dcache_rline = mem_rline;
    assign icache_rline = mem_rline;

endmodule