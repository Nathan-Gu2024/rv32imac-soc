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

    // accelerator result DMA - third requester, same one-outstanding-request
    // contract as the two caches. mm_accel drains its accumulators here as
    // 128-bit lines instead of through its 32-bit AXI4-Lite register window,
    // which measured 70% of GEMM runtime at DIM=16.
    //
    // accel_req_write exists for Phase 2 (operand fetch); Phase 1 only writes.
    // Tie accel_req_valid low to get exactly the previous two-port behaviour.
    input wire accel_req_valid, accel_req_write,
    input wire [31:0] accel_req_addr,
    input wire [127:0] accel_wline,
    output wire accel_ready,
    output wire [127:0] accel_rline,

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
    localparam SVC_A = 2'b11;   // accelerator result DMA

    localparam LAST_D = 2'd0;
    localparam LAST_I = 2'd1;
    localparam LAST_A = 2'd2;

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
    // Extended from a single bit to a 3-way rotating priority when the
    // accelerator DMA became a third requester. Whoever was granted last drops
    // to the back of the queue, so no requester can be starved by the other
    // two alternating - which a fixed priority order would allow, and which is
    // the exact failure mode the two-port round-robin was introduced to remove.
    //
    //   last = D  ->  I, A, D
    //   last = I  ->  A, D, I
    //   last = A  ->  D, I, A
    (* mark_debug = "true" *) reg [1:0] last_granted;

    (* mark_debug = "true" *) wire grant_d = (state == IDLE) && dcache_req_valid &&
        ((last_granted == LAST_A) ? 1'b1                                        // first
       : (last_granted == LAST_I) ? !accel_req_valid                            // second
       :                            (!icache_req_valid && !accel_req_valid));   // third

    (* mark_debug = "true" *) wire grant_i = (state == IDLE) && icache_req_valid &&
        ((last_granted == LAST_D) ? 1'b1
       : (last_granted == LAST_A) ? !dcache_req_valid
       :                            (!accel_req_valid && !dcache_req_valid));

    (* mark_debug = "true" *) wire grant_a = (state == IDLE) && accel_req_valid &&
        ((last_granted == LAST_I) ? 1'b1
       : (last_granted == LAST_D) ? !icache_req_valid
       :                            (!dcache_req_valid && !icache_req_valid));

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
            last_granted <= LAST_D;
        end else begin
            case (state)
                IDLE:
                    begin
                        if (grant_d) begin
                            state <= SVC_D;
                            saved_req_write <= dcache_req_write;
                            saved_req_addr <= dcache_req_addr;
                            saved_wline <= dcache_wline;
                            last_granted <= LAST_D;
                        end else if (grant_i) begin
                            state <= SVC_I;
                            saved_req_write <= 1'b0;
                            saved_req_addr <= icache_req_addr;
                            saved_wline <= 128'b0;
                            last_granted <= LAST_I;
                        end else if (grant_a) begin
                            state <= SVC_A;
                            saved_req_write <= accel_req_write;
                            saved_req_addr <= accel_req_addr;
                            saved_wline <= accel_wline;
                            last_granted <= LAST_A;
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
                SVC_A:
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
    // SVC_A carries saved_req_write because the accelerator port is a writer in
    // Phase 1 and will read in Phase 2; SVC_I is always a read.
    assign mem_req_valid = (state == SVC_D) || (state == SVC_I) || (state == SVC_A);
    assign mem_req_write = ((state == SVC_D) || (state == SVC_A)) ? saved_req_write : 1'b0;
    assign mem_req_addr = (state != IDLE) ? saved_req_addr : 32'b0;
    assign mem_wline = ((state == SVC_D) || (state == SVC_A)) ? saved_wline : 128'b0;

    // signals back to the requesters
    assign dcache_ready = (state == SVC_D) & mem_ready;
    assign icache_ready = (state == SVC_I) & mem_ready;
    assign accel_ready  = (state == SVC_A) & mem_ready;

    // safe to broadcast read data; ready signals gate actual latching
    assign dcache_rline = mem_rline;
    assign icache_rline = mem_rline;
    assign accel_rline  = mem_rline;

endmodule