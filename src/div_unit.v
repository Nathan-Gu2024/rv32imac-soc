// Multi-cycle restoring divider for RV32M div/divu/rem/remu.
//
// A single-cycle combinational 32-bit divider (plain Verilog '/' and '%')
// synthesizes as one huge ripple-subtract chain and does not meet timing at
// any reasonable clock frequency (measured: ~88ns combinational path against
// a 20ns/50MHz period). This does the same restoring-division algorithm one
// bit per cycle instead, stalling the pipeline for the duration via
// global_mem_stall, exactly like a cache miss already does.
module div_unit (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,      // level-high while an undone divide sits in EX
    input  wire [31:0] a,          // dividend, raw two's complement bits
    input  wire [31:0] b,          // divisor,  raw two's complement bits
    input  wire        is_signed,  // 1: div/rem semantics, 0: divu/remu
    output reg         busy,
    output reg         done,       // 1-cycle pulse when quotient/remainder are valid
    output reg  [31:0] quotient,
    output reg  [31:0] remainder
);
    reg [4:0]  count;
    reg [31:0] divisor_abs;
    reg [63:0] rq;       // {remainder-in-progress, quotient-in-progress}
    reg        neg_q, neg_r;

    wire [31:0] a_abs = (is_signed && a[31]) ? (~a + 32'd1) : a;
    wire [31:0] b_abs = (is_signed && b[31]) ? (~b + 32'd1) : b;

    wire [63:0] rq_shifted = {rq[62:0], 1'b0};
    // 33-bit subtraction so the borrow bit reliably reflects whether
    // rq_shifted[63:32] < divisor_abs (plain 32-bit subtraction wraps and
    // its top bit alone does not indicate underflow).
    wire [32:0] trial33    = {1'b0, rq_shifted[63:32]} - {1'b0, divisor_abs};
    wire        borrow     = trial33[32];
    wire [63:0] rq_next    = borrow ? rq_shifted
                                     : {trial33[31:0], rq_shifted[31:1], 1'b1};

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !busy && !done) begin
                if (b == 32'd0) begin
                    // RISC-V spec-mandated divide-by-zero results.
                    quotient  <= 32'hFFFFFFFF;
                    remainder <= a;
                    done      <= 1'b1;
                end else if (is_signed && a == 32'h80000000 && b == 32'hFFFFFFFF) begin
                    // RISC-V spec-mandated signed overflow (INT_MIN / -1).
                    quotient  <= 32'h80000000;
                    remainder <= 32'd0;
                    done      <= 1'b1;
                end else begin
                    rq          <= {32'd0, a_abs};
                    divisor_abs <= b_abs;
                    neg_q       <= is_signed && (a[31] ^ b[31]);
                    neg_r       <= is_signed && a[31];
                    count       <= 5'd0;
                    busy        <= 1'b1;
                end
            end else if (busy) begin
                rq <= rq_next;
                if (count == 5'd31) begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    quotient  <= neg_q ? (~rq_next[31:0]  + 32'd1) : rq_next[31:0];
                    remainder <= neg_r ? (~rq_next[63:32] + 32'd1) : rq_next[63:32];
                end else begin
                    count <= count + 5'd1;
                end
            end
        end
    end
endmodule
