`timescale 1ns/1ps

// Single output-stationary systolic PE: acc += a_in*b_in each enabled
// cycle (or acc <= a_in*b_in on clear_acc, for the first valid term of a
// new run rather than accumulating onto a stale prior-run value).
// a_in/b_in pass through to neighbors with exactly one cycle of registered
// delay - this is what makes the skewed external feed (see mm_accel below)
// line up correctly in time as values propagate through the array.
module systolic_pe #(
    parameter WIDTH = 8,
    parameter ACC_WIDTH = 32
) (
    input wire clk, rst, en, clear_acc,
    input wire signed [WIDTH-1:0] a_in, b_in,
    output reg signed [WIDTH-1:0] a_out, b_out,
    output reg signed [ACC_WIDTH-1:0] acc
);
    always @(posedge clk) begin
        if (rst) begin
            a_out <= 0;
            b_out <= 0;
            acc <= 0;
        end else if (en) begin
            a_out <= a_in;
            b_out <= b_in;
            acc <= clear_acc ? ($signed(a_in) * $signed(b_in))
                              : acc + ($signed(a_in) * $signed(b_in));
        end
    end
endmodule

// 2x2 output-stationary systolic matrix-multiply accelerator, INT8 multiply
// / INT32 accumulate, controlled over a real AXI4-Lite slave interface.
//
// Register map (word offsets, 2x2-specific - a future NxN revision would
// need an indexed result read like LOAD_IDX instead of 4 fixed registers):
//   0x00 CTRL      (W)  bit0=START (pulse, ignored while BUSY), bit1=SOFT_RST
//   0x04 STATUS    (R)  bit0=BUSY, bit1=DONE
//   0x08 K_LEN     (RW) reduction depth for the next run, <= MAX_K
//   0x0C LOAD_IDX  (RW) write index for the four PUSH registers below
//   0x10 A_ROW0_PUSH (W) writes signed INT8 (bits[7:0]) into a_row_buf[0][LOAD_IDX]
//   0x14 A_ROW1_PUSH (W) same, row 1
//   0x18 B_COL0_PUSH (W) same, b_col_buf[0][LOAD_IDX]
//   0x1C B_COL1_PUSH (W) same, column 1
//   0x20 RESULT00  (R)  C[0][0], valid once STATUS.DONE=1
//   0x24 RESULT01  (R)  C[0][1]
//   0x28 RESULT10  (R)  C[1][0]
//   0x2C RESULT11  (R)  C[1][1]
//
// K_LEN/LOAD_IDX/PUSH writes are ignored while BUSY=1 so the CPU can't
// corrupt an in-flight run; CTRL is always accepted. START synchronously
// clears DONE (and sets BUSY) the same cycle it's latched, before the AXI
// write response completes - critical so a poll loop right after a second
// START can't observe a stale DONE=1 left over from the previous run.
// START itself is non-blocking: the AXI write completes as soon as the
// register latches, independent of how long the actual compute takes -
// the CPU polls STATUS afterward, like a real accelerator/DMA engine.
module mm_accel #(
    parameter ARRAY_DIM = 2,
    parameter MAX_K = 16,
    parameter IDX_BITS = $clog2(MAX_K)
) (
    input wire clk, rst,

    // AXI4-Lite slave
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready
);
    reg busy, done;
    reg [7:0] k_len;
    reg [IDX_BITS-1:0] load_idx;
    reg [7:0] t; // cycle counter within the current run

    reg signed [7:0] a_row_buf [0:ARRAY_DIM-1][0:MAX_K-1];
    reg signed [7:0] b_col_buf [0:ARRAY_DIM-1][0:MAX_K-1];

    // ---- AXI4-Lite write channel ----
    // Bridge master always presents AWVALID+WVALID together, so accepting
    // both in the same cycle (rather than staging AW and W separately) is
    // sufficient and keeps this side simple.
    localparam W_IDLE = 1'b0, W_RESP = 1'b1;
    reg w_state;
    wire do_write = s_axi_awvalid && s_axi_wvalid && (w_state == W_IDLE);
    wire [5:0] waddr_word = s_axi_awaddr[7:2];

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp <= 2'b00;
            w_state <= W_IDLE;
        end else begin
            case (w_state)
                W_IDLE: begin
                    if (do_write) begin
                        s_axi_awready <= 1'b1;
                        s_axi_wready <= 1'b1;
                        s_axi_bvalid <= 1'b1;
                        w_state <= W_RESP;
                    end else begin
                        s_axi_awready <= 1'b0;
                        s_axi_wready <= 1'b0;
                    end
                end
                W_RESP: begin
                    s_axi_awready <= 1'b0;
                    s_axi_wready <= 1'b0;
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        w_state <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // ---- AXI4-Lite read channel ----
    localparam R_IDLE = 1'b0, R_RESP = 1'b1;
    reg r_state;
    reg [5:0] raddr_word;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rresp <= 2'b00;
            r_state <= R_IDLE;
        end else begin
            case (r_state)
                R_IDLE: begin
                    if (s_axi_arvalid) begin
                        s_axi_arready <= 1'b1;
                        raddr_word <= s_axi_araddr[7:2];
                        s_axi_rvalid <= 1'b1;
                        r_state <= R_RESP;
                    end else begin
                        s_axi_arready <= 1'b0;
                    end
                end
                R_RESP: begin
                    s_axi_arready <= 1'b0;
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        r_state <= R_IDLE;
                    end
                end
            endcase
        end
    end

    // ---- Register writes ----
    wire start_pulse = do_write && (waddr_word == 6'd0) && s_axi_wdata[0] && !busy;
    wire soft_rst_pulse = do_write && (waddr_word == 6'd0) && s_axi_wdata[1];

    always @(posedge clk) begin
        if (rst) begin
            k_len <= 8'd0;
            load_idx <= {IDX_BITS{1'b0}};
        end else if (do_write && !busy) begin
            case (waddr_word)
                6'd2: k_len <= s_axi_wdata[7:0];
                6'd3: load_idx <= s_axi_wdata[IDX_BITS-1:0];
                6'd4: a_row_buf[0][load_idx] <= s_axi_wdata[7:0];
                6'd5: a_row_buf[1][load_idx] <= s_axi_wdata[7:0];
                6'd6: b_col_buf[0][load_idx] <= s_axi_wdata[7:0];
                6'd7: b_col_buf[1][load_idx] <= s_axi_wdata[7:0];
                default: ; // CTRL (word 0) handled below; reads-only words no-op
            endcase
        end
    end

    // ---- Run sequencer ----
    // Total run length K_LEN + 2*(ARRAY_DIM-1) cycles: a value fed to row i
    // needs j more hops to reach PE(i,j), so PE(i,j)'s last valid term
    // (k=K_LEN-1) actually lands at t=(K_LEN-1)+i+j, not just +i or +j.
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            t <= 8'd0;
        end else if (soft_rst_pulse) begin
            busy <= 1'b0;
            done <= 1'b0;
            t <= 8'd0;
        end else if (start_pulse) begin
            busy <= 1'b1;
            done <= 1'b0;
            t <= 8'd0;
        end else if (busy) begin
            if (t == k_len + 2*(ARRAY_DIM-1) - 1) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
            t <= t + 8'd1;
        end
    end

    // ---- Skewed edge feed: row i's k-th value at t=k+i, column j's at t=k+j ----
    genvar gi, gj;
    wire signed [7:0] a_edge [0:ARRAY_DIM-1];
    wire signed [7:0] b_edge [0:ARRAY_DIM-1];

    generate
        for (gi = 0; gi < ARRAY_DIM; gi = gi + 1) begin : row_feed
            wire signed [8:0] k_idx = $signed({1'b0, t}) - gi;
            wire k_valid = (k_idx >= 0) && (k_idx < $signed({1'b0, k_len}));
            assign a_edge[gi] = k_valid ? a_row_buf[gi][k_idx[IDX_BITS-1:0]] : 8'sd0;
        end
        for (gj = 0; gj < ARRAY_DIM; gj = gj + 1) begin : col_feed
            wire signed [8:0] k_idx = $signed({1'b0, t}) - gj;
            wire k_valid = (k_idx >= 0) && (k_idx < $signed({1'b0, k_len}));
            assign b_edge[gj] = k_valid ? b_col_buf[gj][k_idx[IDX_BITS-1:0]] : 8'sd0;
        end
    endgenerate

    // ---- PE array ----
    wire signed [7:0] a_chain [0:ARRAY_DIM-1][0:ARRAY_DIM];
    wire signed [7:0] b_chain [0:ARRAY_DIM][0:ARRAY_DIM-1];
    wire signed [31:0] pe_acc [0:ARRAY_DIM-1][0:ARRAY_DIM-1];

    generate
        for (gi = 0; gi < ARRAY_DIM; gi = gi + 1) begin : edge_a
            assign a_chain[gi][0] = a_edge[gi];
        end
        for (gj = 0; gj < ARRAY_DIM; gj = gj + 1) begin : edge_b
            assign b_chain[0][gj] = b_edge[gj];
        end
        for (gi = 0; gi < ARRAY_DIM; gi = gi + 1) begin : pe_row
            for (gj = 0; gj < ARRAY_DIM; gj = gj + 1) begin : pe_col
                systolic_pe #(.WIDTH(8), .ACC_WIDTH(32)) PE (
                    .clk(clk),
                    .rst(rst),
                    .en(busy),
                    .clear_acc(t == (gi + gj)),
                    .a_in(a_chain[gi][gj]),
                    .b_in(b_chain[gi][gj]),
                    .a_out(a_chain[gi][gj+1]),
                    .b_out(b_chain[gi+1][gj]),
                    .acc(pe_acc[gi][gj])
                );
            end
        end
    endgenerate

    // ---- Read mux ----
    always @(*) begin
        case (raddr_word)
            6'd1: s_axi_rdata = {30'b0, done, busy};
            6'd2: s_axi_rdata = {24'b0, k_len};
            6'd3: s_axi_rdata = {{(32-IDX_BITS){1'b0}}, load_idx};
            6'd8: s_axi_rdata = pe_acc[0][0];
            6'd9: s_axi_rdata = pe_acc[0][1];
            6'd10: s_axi_rdata = pe_acc[1][0];
            6'd11: s_axi_rdata = pe_acc[1][1];
            default: s_axi_rdata = 32'b0;
        endcase
    end

endmodule
