`ifndef _MEM_ARBITER_RW_V_
`define _MEM_ARBITER_RW_V_

`timescale 1ns/1ps

// Three-requester arbiter with INDEPENDENT read and write paths.
//
// WHY THIS EXISTS (T3.2, stage 2)
//
// mem_arbiter has one state register, one grant and one downstream channel, so a
// read waits behind a write and vice versa - regardless of the fact that they go
// to completely different AXI channels. That is the middle of the three layers
// that serialise reads against writes here; axi_rw_engine removed the bottom one.
//
// Two rotations, sharing nothing:
//
//   READ   dcache, icache, accel      three-way
//   WRITE  dcache, accel              two-way; the icache never writes
//
// The CACHES keep one port each and are routed by their direction bit, because
// neither can have two requests outstanding: dcache_bram.v drives mem_req_valid
// from a single state and serialises its writeback before its fill.
//
// The ACCELERATOR's port is split on both the request and the completion side, so
// it can have an operand fetch and a result store in flight simultaneously. That
// is the case the whole refactor exists for, and it is why a merged
// accel_req_valid/accel_req_write pair would not have been enough: one valid can
// only carry one direction per cycle, so the arbiter would have been physically
// unable to accept both however the completions were wired.
//
// Two wins, worth keeping distinct because only the first is available without
// splitting the accelerator itself:
//   * a requester's read no longer waits on a DIFFERENT requester's write - a
//     63-line result burst stops blocking instruction fetch for ~130 cycles,
//     which is the arbiter half of the audit's non-preemptive note;
//   * the accelerator's own load and store can overlap.
//
// COMPLETION SEMANTICS, which is where the original got bitten. A read completes
// one LINE at a time, so a multi-line burst must count lines before releasing the
// port; a write completes ONCE for the whole transaction. The old arbiter carried
// both meanings on one mem_ready and had to branch on saved_req_write to know
// which it was looking at. Here rd_ready and wr_ready are separate signals and
// the ambiguity does not exist.
module mem_arbiter_rw (
    input wire clk,
    input wire rst,

    // ---- icache: read only ----
    input wire icache_req_valid,
    input wire [31:0] icache_req_addr,
    output wire icache_ready,
    output wire [127:0] icache_rline,

    // ---- dcache: single-line, either direction ----
    input wire dcache_req_valid, dcache_req_write,
    input wire [31:0] dcache_req_addr,
    input wire [127:0] dcache_wline,
    output wire dcache_ready,
    output wire [127:0] dcache_rline,

    // ---- accelerator: multi-line, INDEPENDENT read and write ----
    //
    // Fully split, request side included - not just the completions. A single
    // accel_req_valid + accel_req_write pair can only ever present ONE direction
    // at a time, so leaving the request side merged would mean the arbiter
    // physically cannot accept an operand fetch and a result store together,
    // which is the entire thing this refactor is for. The caches keep their
    // single ports because neither can have two requests outstanding.
    input wire accel_rd_req_valid,
    input wire [31:0] accel_rd_req_addr,
    input wire [7:0] accel_rd_req_lines,
    input wire accel_wr_req_valid,
    input wire [31:0] accel_wr_req_addr,
    input wire [7:0] accel_wr_req_lines,
    input wire [127:0] accel_wline,
    // SEPARATE completions, unlike the caches. The accelerator is the one
    // requester that will have a read and a write outstanding at the same time
    // (that is the point of splitting its port), so merging these onto a single
    // accel_ready would re-create exactly the direction ambiguity this whole
    // refactor exists to remove - one layer further up. See the dcache_ready
    // note at the bottom for why the caches may still merge theirs.
    output wire accel_rd_ready,
    output wire accel_wr_ready,
    output wire accel_wnext,
    output wire [127:0] accel_rline,

    // ---- downstream READ channel ----
    output wire rd_req_valid,
    output wire [31:0] rd_req_addr,
    output wire [7:0] rd_req_lines,
    input wire rd_ready,                 // one pulse per LINE
    input wire [127:0] rd_rline,

    // ---- downstream WRITE channel ----
    output wire wr_req_valid,
    output wire [31:0] wr_req_addr,
    output wire [7:0] wr_req_lines,
    output wire [127:0] wr_wline,
    input wire wr_next,
    input wire wr_ready                  // one pulse per TRANSACTION
);

    // The caches have ONE port each, so their direction is decoded from the
    // request. The accelerator presents both directions independently, so there
    // is nothing to decode.
    wire d_rd_req = dcache_req_valid && !dcache_req_write;
    wire d_wr_req = dcache_req_valid &&  dcache_req_write;
    wire a_rd_req = accel_rd_req_valid;
    wire a_wr_req = accel_wr_req_valid;

    // ================================================================
    // READ side: three-way rotation
    // ================================================================
    localparam [1:0] R_IDLE = 2'd0, R_D = 2'd1, R_I = 2'd2, R_A = 2'd3;
    localparam [1:0] RL_D = 2'd0, RL_I = 2'd1, RL_A = 2'd2;

    reg [1:0] r_state, r_last;
    reg [31:0] r_addr;
    reg [7:0]  r_lines, r_count;

    // Rotating priority: whoever went last drops to the back. Same shape as
    // mem_arbiter's, which was traced starvation-free across all nine cases.
    wire r_grant_d = (r_state == R_IDLE) && d_rd_req &&
        ((r_last == RL_A) ? 1'b1 :
         (r_last == RL_I) ? !a_rd_req : (!a_rd_req && !icache_req_valid));
    wire r_grant_i = (r_state == R_IDLE) && icache_req_valid && !r_grant_d &&
        ((r_last == RL_D) ? 1'b1 :
         (r_last == RL_A) ? !d_rd_req : (!d_rd_req && !a_rd_req));
    wire r_grant_a = (r_state == R_IDLE) && a_rd_req && !r_grant_d && !r_grant_i;

    always @(posedge clk) begin
        if (rst) begin
            r_state <= R_IDLE;
            r_last  <= RL_D;
            r_addr  <= 32'b0;
            r_lines <= 8'd1;
            r_count <= 8'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    r_count <= 8'd0;
                    if (r_grant_d) begin
                        r_state <= R_D; r_addr <= dcache_req_addr;
                        r_lines <= 8'd1; r_last <= RL_D;
                    end else if (r_grant_i) begin
                        r_state <= R_I; r_addr <= icache_req_addr;
                        r_lines <= 8'd1; r_last <= RL_I;
                    end else if (r_grant_a) begin
                        r_state <= R_A; r_addr <= accel_rd_req_addr;
                        r_lines <= (accel_rd_req_lines == 8'd0) ? 8'd1 : accel_rd_req_lines;
                        r_last  <= RL_A;
                    end
                end
                // Caches are single-line, so one pulse retires them.
                R_D, R_I: if (rd_ready) r_state <= R_IDLE;
                // A burst must count lines: releasing on the first pulse would
                // drop the port while lines were still arriving.
                R_A: if (rd_ready) begin
                    if (r_count + 8'd1 >= r_lines) begin
                        r_state <= R_IDLE;
                        r_count <= 8'd0;
                    end else begin
                        r_count <= r_count + 8'd1;
                    end
                end
                default: r_state <= R_IDLE;
            endcase
        end
    end

    // ================================================================
    // WRITE side: two-way rotation. No icache.
    // ================================================================
    localparam [1:0] W_IDLE = 2'd0, W_D = 2'd1, W_A = 2'd2;

    reg [1:0] w_state;
    reg w_last_a;                 // 1 when the accelerator went last
    reg [31:0] w_addr;
    reg [7:0]  w_lines;
    reg [127:0] w_line_latched;

    wire w_grant_d = (w_state == W_IDLE) && d_wr_req && (w_last_a || !a_wr_req);
    wire w_grant_a = (w_state == W_IDLE) && a_wr_req && !w_grant_d;

    always @(posedge clk) begin
        if (rst) begin
            w_state  <= W_IDLE;
            w_last_a <= 1'b1;     // so the dcache wins the first tie
            w_addr   <= 32'b0;
            w_lines  <= 8'd1;
            w_line_latched <= 128'b0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    if (w_grant_d) begin
                        w_state <= W_D; w_addr <= dcache_req_addr;
                        w_lines <= 8'd1; w_line_latched <= dcache_wline;
                        w_last_a <= 1'b0;
                    end else if (w_grant_a) begin
                        w_state <= W_A; w_addr <= accel_wr_req_addr;
                        w_lines <= (accel_wr_req_lines == 8'd0) ? 8'd1 : accel_wr_req_lines;
                        w_line_latched <= accel_wline;
                        w_last_a <= 1'b1;
                    end
                end
                // One pulse per transaction, burst or not.
                W_D, W_A: if (wr_ready) w_state <= W_IDLE;
                default: w_state <= W_IDLE;
            endcase
        end
    end

    // ---- downstream drives ----
    assign rd_req_valid = (r_state != R_IDLE);
    assign rd_req_addr  = (r_state != R_IDLE) ? r_addr  : 32'b0;
    assign rd_req_lines = (r_state != R_IDLE) ? r_lines : 8'd1;

    assign wr_req_valid = (w_state != W_IDLE);
    assign wr_req_addr  = (w_state != W_IDLE) ? w_addr  : 32'b0;
    assign wr_req_lines = (w_state != W_IDLE) ? w_lines : 8'd1;
    // A multi-line accelerator write takes the LIVE line: the requester advances
    // it on wr_next, so a latched copy would repeat line 0 for the whole burst.
    // Everything else keeps the latched copy.
    assign wr_wline = ((w_state == W_A) && (w_lines > 8'd1)) ? accel_wline
                                                            : w_line_latched;

    // ---- back to the requesters ----
    assign icache_ready = (r_state == R_I) && rd_ready;

    // dcache_ready MAY merge the two directions, and that is safe by invariant
    // rather than by luck: dcache_bram.v:348-349 drives
    //   mem_req_valid = (state == S_WB) || (state == S_FILL)
    //   mem_req_write = (state == S_WB)
    // from ONE state register, and :335 is `S_WB: if (mem_ready) state <= S_FILL`,
    // so a writeback strictly precedes its fill and the cache never has two
    // requests outstanding. If the D-cache is ever given independent writeback
    // and fill ports, this merge has to split too.
    assign dcache_ready = ((r_state == R_D) && rd_ready) ||
                          ((w_state == W_D) && wr_ready);

    assign accel_rd_ready = (r_state == R_A) && rd_ready;
    assign accel_wr_ready = (w_state == W_A) && wr_ready;
    assign accel_wnext    = (w_state == W_A) && wr_next;

    // Read data is safe to broadcast; the ready signals gate latching.
    assign icache_rline = rd_rline;
    assign dcache_rline = rd_rline;
    assign accel_rline  = rd_rline;

endmodule

`endif // _MEM_ARBITER_RW_V_
