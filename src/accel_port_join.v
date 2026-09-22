`ifndef _ACCEL_PORT_JOIN_V_
`define _ACCEL_PORT_JOIN_V_

`timescale 1ns/1ps

// Re-serialise the accelerator's split read/write ports onto ONE single-port
// request, and demux the single completion back to the right channel.
//
// WHY THIS EXISTS (T3.2, stage 3)
//
// Once mm_accel's memory port splits, two consumers still expect the old single
// port: the legacy mem_arbiter + axi_cache_adapter path kept selectable for A/B
// on hardware, and the eight testbench memory models that already work. This lets
// both keep working unchanged - the benches get this module inserted between the
// DUT and their existing mock, about ten lines each, instead of eight memory
// models being rewritten. That matters: bench-model bugs have been a real cost
// here, including one that replayed a whole burst and stamped line 0 across a
// result buffer.
//
// THE DIRECTION TAG IS THE STATE, which is the one property worth insisting on.
// The defect this whole refactor removes is that a single `mem_ready` carried no
// direction, so the load engine and the store engine each had to guess with
// `!dmaBusy` - and a read line arriving while a store was in flight was BOTH
// dropped by the loader AND miscredited as a write completion. Here the grant
// state says which channel a completion belongs to, so that ambiguity cannot
// reappear inside this module:
//
//     accel_rd_ready = (state == S_RD) && mem_ready
//     accel_wr_ready = (state == S_WR) && mem_ready
//
// COMPLETION SEMANTICS are asymmetric, exactly as mem_arbiter's SVC_A path had
// to handle: a read pulses mem_ready once per LINE, so a burst counts lines
// before releasing; a write pulses once for the whole TRANSACTION. Counting on a
// write would wait for pulses that never arrive.
//
// It costs one cycle of grant latency per accelerator transaction, and drops
// mem_req_valid for a cycle after each completion. Both are deliberate: several
// bench mocks accept a new request whenever `mem_req_valid && !mem_ready`, so a
// request held continuously across a completion would start a duplicate
// transaction in the mock.
module accel_port_join (
    input wire clk,
    input wire rst,

    // ---- split side: from mm_accel ----
    input wire accel_rd_req_valid,
    input wire [31:0] accel_rd_req_addr,
    input wire [7:0] accel_rd_req_lines,
    output wire accel_rd_ready,
    output wire [127:0] accel_rline,

    input wire accel_wr_req_valid,
    input wire [31:0] accel_wr_req_addr,
    input wire [7:0] accel_wr_req_lines,
    input wire [127:0] accel_wline,
    output wire accel_wnext,
    output wire accel_wr_ready,

    // ---- single-port side: to mem_arbiter, or a bench mock ----
    output wire mem_req_valid,
    output wire mem_req_write,
    output wire [31:0] mem_req_addr,
    output wire [7:0] mem_req_lines,
    output wire [127:0] mem_wline,
    input wire mem_ready,
    input wire mem_wnext,
    input wire [127:0] mem_rline
);

    localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_WR = 2'd2;

    reg [1:0] state;
    reg [31:0] saved_addr;
    reg [7:0] saved_lines, line_count;

    // Write priority on a tie, matching the precedence the single-port design
    // already had: mem_req_write was driven straight from dmaBusy, so a store in
    // flight always owned the port.
    wire grant_wr = (state == S_IDLE) && accel_wr_req_valid;
    wire grant_rd = (state == S_IDLE) && accel_rd_req_valid && !grant_wr;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            saved_addr <= 32'b0;
            saved_lines <= 8'd1;
            line_count <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    line_count <= 8'd0;
                    if (grant_wr) begin
                        state <= S_WR;
                        saved_addr <= accel_wr_req_addr;
                        saved_lines <= (accel_wr_req_lines == 8'd0)
                                       ? 8'd1 : accel_wr_req_lines;
                    end else if (grant_rd) begin
                        state <= S_RD;
                        saved_addr <= accel_rd_req_addr;
                        saved_lines <= (accel_rd_req_lines == 8'd0)
                                       ? 8'd1 : accel_rd_req_lines;
                    end
                end
                // A read completes one LINE at a time: releasing on the first
                // pulse would drop the port while lines were still arriving.
                S_RD: if (mem_ready) begin
                    if (line_count + 8'd1 >= saved_lines) begin
                        state <= S_IDLE;
                        line_count <= 8'd0;
                    end else begin
                        line_count <= line_count + 8'd1;
                    end
                end
                // A write completes ONCE, burst or not.
                S_WR: if (mem_ready) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

    assign mem_req_valid = (state != S_IDLE);
    assign mem_req_write = (state == S_WR);
    assign mem_req_addr  = (state != S_IDLE) ? saved_addr  : 32'b0;
    assign mem_req_lines = (state != S_IDLE) ? saved_lines : 8'd1;
    // A multi-line write takes the LIVE line: the requester advances it on
    // wnext, so a latched copy would repeat line 0 for the whole burst.
    assign mem_wline     = accel_wline;

    assign accel_rd_ready = (state == S_RD) && mem_ready;
    assign accel_wr_ready = (state == S_WR) && mem_ready;
    assign accel_wnext    = (state == S_WR) && mem_wnext;
    assign accel_rline    = mem_rline;

endmodule

`endif // _ACCEL_PORT_JOIN_V_
