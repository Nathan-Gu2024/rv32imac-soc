`ifndef _AXI_RW_ENGINE_V_
`define _AXI_RW_ENGINE_V_

`timescale 1ns/1ps

// Independent read and write engines over ONE AXI4 master.
//
// WHY THIS EXISTS (T3.2, stage 1)
//
// axi_cache_adapter runs a single FSM whose IDLE branches to either READ_ADDR or
// WRITE_ADDR, so a read and a write can never be in flight together. That is the
// lowest of three layers that serialise reads against writes in this SoC; the
// other two are mem_arbiter's single grant and mm_accel's single port. All three
// have to go before the accelerator can overlap an operand fetch with a result
// store, and this is the one that can be built and tested on its own.
//
// The key observation is that AXI4 already separates them. AR/R and AW/W/B share
// no signals, so two FSMs can drive one master concurrently with no arbitration
// between them at all - the protocol was designed for exactly this. Nothing here
// is clever; the old adapter simply never used the separation.
//
// This is a NEW module rather than a rewrite of axi_cache_adapter, so nothing
// currently working can regress while the upstream layers are still single-port.
// The adapter keeps serving mem_arbiter until stage 2 migrates it.
//
// WHAT IS CARRIED OVER FROM axi_cache_adapter, and why (see its header for the
// full history - these were all found the hard way):
//
//   * mem_ready is per-LINE on reads and per-TRANSACTION on writes. Here that
//     ambiguity is gone by construction: rd_ready and wr_ready are separate
//     signals, which is the entire point of the split.
//   * A write burst must not slice the requester's line live. wr_next is pulsed
//     one beat EARLY so the requester's FIFO can present the next line in time,
//     which means the current line changes while its last beat is outstanding.
//     wline_hold freezes it.
//   * A timeout abandons a transaction the interconnect has stopped answering.
//     It resumes from lines already transferred rather than restarting, clears
//     its strobes, and - for a write whose data is all accepted - COMPLETES
//     rather than retrying, because only the BRESP was lost and re-sending would
//     overwrite correct memory from a drained FIFO.
//   * The timeout counts cycles without forward progress, not cycles since the
//     transaction began, so a long healthy burst cannot abort itself.
module axi_rw_engine #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BITS = 128,
    parameter AXI_DATA_WIDTH = 64,
    parameter [15:0] TIMEOUT_LIMIT = 16'd4000
)(
    input wire clk,
    input wire rst,

    // ---- read channel, upstream ----
    input wire rd_req_valid,
    input wire [ADDR_WIDTH-1:0] rd_req_addr,
    input wire [7:0] rd_req_lines,
    output reg [LINE_BITS-1:0] rd_rline,
    output reg rd_ready,                 // one pulse per LINE delivered

    // ---- write channel, upstream ----
    input wire wr_req_valid,
    input wire [ADDR_WIDTH-1:0] wr_req_addr,
    input wire [7:0] wr_req_lines,
    input wire [LINE_BITS-1:0] wr_wline,
    output reg wr_next,                  // requester should present the next line
    output reg wr_ready,                 // one pulse per TRANSACTION

    // ---- AXI read address ----
    output reg [ADDR_WIDTH-1:0] m_axi_araddr,
    output reg [7:0] m_axi_arlen,
    output reg [2:0] m_axi_arsize,
    output reg [1:0] m_axi_arburst,
    output reg m_axi_arvalid,
    input wire m_axi_arready,

    // ---- AXI read data ----
    input wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input wire m_axi_rvalid,
    input wire m_axi_rlast,
    output reg m_axi_rready,

    // ---- AXI write address ----
    output reg [ADDR_WIDTH-1:0] m_axi_awaddr,
    output reg [7:0] m_axi_awlen,
    output reg [2:0] m_axi_awsize,
    output reg [1:0] m_axi_awburst,
    output reg m_axi_awvalid,
    input wire m_axi_awready,

    // ---- AXI write data ----
    output reg [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output reg [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg m_axi_wlast,
    output reg m_axi_wvalid,
    input wire m_axi_wready,

    // ---- AXI write response ----
    input wire [1:0] m_axi_bresp,
    input wire m_axi_bvalid,
    output reg m_axi_bready,

    // Debug visibility, same spirit as the adapter's retry_count.
    output reg [15:0] rd_retries,
    output reg [15:0] wr_retries
);

    localparam integer LINE_BYTES = LINE_BITS / 8;
    localparam integer BEATS_PER_LINE = LINE_BITS / AXI_DATA_WIDTH;
    localparam integer OFFSET_BITS = $clog2(LINE_BYTES);
    localparam [7:0] AXI_LEN = BEATS_PER_LINE - 1;

    localparam [2:0] AXI_SIZE =
        (AXI_DATA_WIDTH == 32)  ? 3'b010 :
        (AXI_DATA_WIDTH == 64)  ? 3'b011 :
        (AXI_DATA_WIDTH == 128) ? 3'b100 :
                                  3'b000;

    // ================================================================
    // READ ENGINE
    // ================================================================
    localparam [1:0] RD_IDLE = 2'd0, RD_ADDR = 2'd1, RD_DATA = 2'd2, RD_DONE = 2'd3;

    reg [1:0] rd_state, rd_next;
    reg [7:0] rd_beat;            // beat index WITHIN the current line
    reg [7:0] rd_done_lines;      // lines of this request already delivered
    reg [ADDR_WIDTH-1:0] rd_addr_s;
    reg [7:0] rd_lines_s;
    reg rd_retrying;
    reg [15:0] rd_tmo;
    // "This request already completed; refuse to restart it until the requester
    // deasserts." Guards against a requester that holds valid across the
    // completion cycle, which would otherwise run a duplicate transaction - the
    // exact defect that made tb_mm_accel_c_test's memory model replay a burst and
    // stamp line 0 across a result buffer.
    //
    // Keyed on RD_DONE and NEVER on the start, which is what makes it compatible
    // with the timeout retry: a retry re-presents the SAME request with valid
    // still high, so a rising-edge or valid-low requirement would deadlock it.
    // The timeout branch leaves rd_served clear for precisely that reason.
    reg rd_served;

    wire [ADDR_WIDTH-1:0] rd_line_addr =
        {rd_addr_s[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    wire [ADDR_WIDTH-1:0] rd_base =
        rd_line_addr + ({{(ADDR_WIDTH-8){1'b0}}, rd_done_lines} << OFFSET_BITS);
    wire [7:0] rd_remaining =
        (rd_done_lines < rd_lines_s) ? (rd_lines_s - rd_done_lines) : 8'd1;

    wire rd_eligible = (rd_state != RD_IDLE) && (rd_state != RD_DONE);
    wire rd_progress = (m_axi_arvalid && m_axi_arready) ||
                       (m_axi_rvalid  && m_axi_rready);

    always @(*) begin
        rd_next = rd_state;
        case (rd_state)
            RD_IDLE: if (rd_req_valid && !rd_served) rd_next = RD_ADDR;
            RD_ADDR: if (m_axi_arvalid && m_axi_arready) rd_next = RD_DATA;
            RD_DATA: if (m_axi_rvalid && m_axi_rready && m_axi_rlast)
                         rd_next = RD_DONE;
            RD_DONE: rd_next = RD_IDLE;
            default: rd_next = RD_IDLE;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            rd_state <= RD_IDLE;
            rd_beat <= 8'd0;
            rd_done_lines <= 8'd0;
            rd_addr_s <= {ADDR_WIDTH{1'b0}};
            rd_lines_s <= 8'd1;
            rd_ready <= 1'b0;
            rd_rline <= {LINE_BITS{1'b0}};
            rd_retrying <= 1'b0;
            rd_tmo <= 16'd0;
            rd_retries <= 16'd0;
            rd_served <= 1'b0;
        end else if (rd_eligible && rd_tmo >= TIMEOUT_LIMIT) begin
            // Abandon and resume from what has already been delivered.
            rd_state <= RD_IDLE;
            rd_beat <= 8'd0;
            rd_tmo <= 16'd0;
            rd_retries <= rd_retries + 16'd1;
            rd_retrying <= 1'b1;
            rd_ready <= 1'b0;   // never announce a line that did not arrive
        end else begin
            rd_state <= rd_next;
            rd_tmo <= rd_eligible ? (rd_progress ? 16'd0 : rd_tmo + 16'd1) : 16'd0;

            // Set on completion, cleared only when the requester withdraws.
            if (rd_state == RD_DONE)    rd_served <= 1'b1;
            else if (!rd_req_valid)     rd_served <= 1'b0;

            if (rd_state == RD_IDLE && rd_req_valid && !rd_served) begin
                rd_addr_s  <= rd_req_addr;
                rd_lines_s <= (rd_req_lines == 8'd0) ? 8'd1 : rd_req_lines;
                rd_beat    <= 8'd0;
                // A retry re-presents the same request; only a new one restarts.
                if (!rd_retrying) rd_done_lines <= 8'd0;
                rd_retrying <= 1'b0;
            end

            rd_ready <= 1'b0;
            if (rd_state == RD_DATA && m_axi_rvalid && m_axi_rready) begin
                rd_rline[rd_beat*AXI_DATA_WIDTH +: AXI_DATA_WIDTH] <= m_axi_rdata;
                if (rd_beat == BEATS_PER_LINE - 1) begin
                    rd_beat <= 8'd0;
                    rd_ready <= 1'b1;              // publish next cycle
                    rd_done_lines <= rd_done_lines + 8'd1;
                end else begin
                    rd_beat <= rd_beat + 8'd1;
                end
            end

            if (rd_state == RD_DONE) rd_beat <= 8'd0;
        end
    end

    // ================================================================
    // WRITE ENGINE
    // ================================================================
    localparam [2:0] WR_IDLE = 3'd0, WR_ADDR = 3'd1, WR_DATA = 3'd2,
                     WR_RESP = 3'd3, WR_DONE = 3'd4;

    reg [2:0] wr_state, wr_next_s;
    reg [7:0] wr_beat;
    reg [7:0] wr_sent;            // lines of this request already accepted
    reg [ADDR_WIDTH-1:0] wr_addr_s;
    reg [7:0] wr_lines_s;
    reg [LINE_BITS-1:0] wline_hold;
    reg [LINE_BITS-1:0] wline_first;   // single-line requests slice this
    reg wr_retrying, wr_fifo_ahead, wr_resume_hold;
    reg [15:0] wr_tmo;
    // See rd_served. On this side a duplicate transaction is worse than wasteful:
    // it re-sends from a requester FIFO that has already drained, overwriting
    // correct memory.
    reg wr_served;

    wire [ADDR_WIDTH-1:0] wr_line_addr =
        {wr_addr_s[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    wire [ADDR_WIDTH-1:0] wr_base =
        wr_line_addr + ({{(ADDR_WIDTH-8){1'b0}}, wr_sent} << OFFSET_BITS);
    wire [7:0] wr_remaining =
        (wr_sent < wr_lines_s) ? (wr_lines_s - wr_sent) : 8'd1;

    wire wr_eligible = (wr_state != WR_IDLE) && (wr_state != WR_DONE);
    wire wr_progress = (m_axi_awvalid && m_axi_awready) ||
                       (m_axi_wvalid  && m_axi_wready)  ||
                       (m_axi_bvalid  && m_axi_bready);
    // All data accepted, waiting only on the response: the transfer succeeded and
    // only its acknowledgement was lost, so complete instead of re-sending.
    wire wr_bresp_only = (wr_state == WR_RESP) && (wr_sent >= wr_lines_s);

    always @(*) begin
        wr_next_s = wr_state;
        case (wr_state)
            WR_IDLE: if (wr_req_valid && !wr_served) wr_next_s = WR_ADDR;
            WR_ADDR: if (m_axi_awvalid && m_axi_awready) wr_next_s = WR_DATA;
            WR_DATA: if (m_axi_wvalid && m_axi_wready && m_axi_wlast)
                         wr_next_s = WR_RESP;
            WR_RESP: if (m_axi_bvalid && m_axi_bready) wr_next_s = WR_DONE;
            WR_DONE: wr_next_s = WR_IDLE;
            default: wr_next_s = WR_IDLE;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_state <= WR_IDLE;
            wr_beat <= 8'd0;
            wr_sent <= 8'd0;
            wr_addr_s <= {ADDR_WIDTH{1'b0}};
            wr_lines_s <= 8'd1;
            wline_hold <= {LINE_BITS{1'b0}};
            wline_first <= {LINE_BITS{1'b0}};
            wr_next <= 1'b0;
            wr_retrying <= 1'b0;
            wr_fifo_ahead <= 1'b0;
            wr_resume_hold <= 1'b0;
            wr_tmo <= 16'd0;
            wr_retries <= 16'd0;
            wr_served <= 1'b0;
        end else if (wr_eligible && wr_tmo >= TIMEOUT_LIMIT && wr_bresp_only) begin
            wr_state <= WR_DONE;      // lost BRESP: complete, do not re-send
            wr_beat <= 8'd0;
            wr_tmo <= 16'd0;
            wr_retries <= wr_retries + 16'd1;
            wr_next <= 1'b0;
        end else if (wr_eligible && wr_tmo >= TIMEOUT_LIMIT) begin
            wr_state <= WR_IDLE;
            wr_beat <= 8'd0;
            wr_tmo <= 16'd0;
            wr_retries <= wr_retries + 16'd1;
            wr_retrying <= 1'b1;
            // If wr_next already went out for the line at wr_sent, the requester
            // has moved past it and cannot rewind - replay it from wline_hold.
            wr_resume_hold <= wr_fifo_ahead;
            wr_next <= 1'b0;
        end else begin
            wr_state <= wr_next_s;
            wr_tmo <= wr_eligible ? (wr_progress ? 16'd0 : wr_tmo + 16'd1) : 16'd0;

            // Set on completion, cleared only when the requester withdraws. The
            // lost-BRESP timeout routes through WR_DONE, so it lands here too;
            // the retry timeout goes to WR_IDLE and leaves this clear.
            if (wr_state == WR_DONE)    wr_served <= 1'b1;
            else if (!wr_req_valid)     wr_served <= 1'b0;

            if (wr_state == WR_IDLE && wr_req_valid && !wr_served) begin
                wr_addr_s  <= wr_req_addr;
                wr_lines_s <= (wr_req_lines == 8'd0) ? 8'd1 : wr_req_lines;
                wr_beat    <= 8'd0;
                if (!wr_retrying) begin
                    wr_sent       <= 8'd0;
                    wline_first   <= wr_wline;
                    wr_fifo_ahead <= 1'b0;
                end
                wr_retrying <= 1'b0;
            end

            // Freeze the line being transmitted. Not while a replay is owed:
            // wline_hold IS that line and wr_wline has already moved on.
            if (wr_state == WR_DATA && wr_beat == 8'd0 && !wr_resume_hold)
                wline_hold <= wr_wline;

            wr_next <= 1'b0;
            if (wr_state == WR_DATA && m_axi_wvalid && m_axi_wready) begin
                if (wr_beat == BEATS_PER_LINE - 1) begin
                    wr_beat <= 8'd0;
                    wr_sent <= wr_sent + 8'd1;
                    wr_fifo_ahead  <= 1'b0;
                    wr_resume_hold <= 1'b0;
                end else begin
                    wr_beat <= wr_beat + 8'd1;
                end
                // One beat EARLY so the requester's FIFO can present the next
                // line before its first beat is needed.
                if ((wr_beat == BEATS_PER_LINE - 2) &&
                    (wr_sent + 8'd1 < wr_lines_s)) begin
                    wr_next <= 1'b1;
                    wr_fifo_ahead <= 1'b1;
                end
            end

            if (wr_state == WR_DONE) wr_beat <= 8'd0;
        end
    end

    // ================================================================
    // AXI outputs. The two engines touch disjoint channels, so there is no
    // arbitration here - that is the whole reason this split is cheap.
    // ================================================================
    always @(*) begin
        // read address / data
        m_axi_araddr  = rd_base;
        m_axi_arlen   = (rd_remaining * BEATS_PER_LINE) - 8'd1;
        m_axi_arsize  = AXI_SIZE;
        m_axi_arburst = 2'b01;                    // INCR
        m_axi_arvalid = (rd_state == RD_ADDR);
        m_axi_rready  = (rd_state == RD_DATA);

        // write address / data / response
        m_axi_awaddr  = wr_base;
        m_axi_awlen   = (wr_remaining * BEATS_PER_LINE) - 8'd1;
        m_axi_awsize  = AXI_SIZE;
        m_axi_awburst = 2'b01;                    // INCR
        m_axi_awvalid = (wr_state == WR_ADDR);
        m_axi_wvalid  = (wr_state == WR_DATA);
        m_axi_wstrb   = {(AXI_DATA_WIDTH/8){1'b1}};   // full-line writes only
        m_axi_bready  = (wr_state == WR_RESP);

        m_axi_wdata = (wr_lines_s == 8'd1)
            ? wline_first[wr_beat*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]
            : (wr_resume_hold
                ? wline_hold[wr_beat*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]
                : (wr_beat == 8'd0
                    ? wr_wline[wr_beat*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]
                    : wline_hold[wr_beat*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]));

        // WLAST marks the end of the burst, which after a resume is the end of
        // what remains of the request.
        m_axi_wlast = (wr_beat == AXI_LEN) && (wr_sent == wr_lines_s - 8'd1);
    end

    // wr_ready: one pulse per transaction, in WR_DONE.
    always @(*) begin
        wr_ready = (wr_state == WR_DONE);
    end

endmodule

`endif // _AXI_RW_ENGINE_V_
