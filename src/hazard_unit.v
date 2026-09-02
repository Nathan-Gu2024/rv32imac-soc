// Load-use hazard detection and operand forwarding.
//
// stall freezes IF/ID and the PC for one cycle when an instruction in ID
// needs a register that a load still in EX has not produced yet. fwd_a and
// fwd_b steer each ALU operand to the newest in-flight value.
module hazard_unit (
    // For load-use stall
    // 1 only for a real load, so flush-inserted bubbles cannot stall.
    input wire id_ex_mem_read,
    input wire [4:0] id_ex_rd,
    input wire [1:0] id_ex_wb_sel,   // unused by the stall logic below
    input wire [4:0] if_id_rs1, if_id_rs2,

    // For forwarding
    input wire [4:0] ex_mem_rd, mem_wb_rd,
    input wire ex_mem_reg_wen, mem_wb_reg_wen,
    input wire [4:0] id_ex_rs1, id_ex_rs2,

    // For branch flush
    input wire pc_sel,

    // Outputs
    output reg stall, // freeze IF/ID and PC
    output reg [1:0] fwd_a, fwd_b // 00=reg, 01=EX/MEM, 10=MEM/WB
);
    always @(*) begin
        // Load-use stall, gated on id_ex_mem_read so bubbles never stall.
        stall = id_ex_mem_read && (id_ex_rd != 5'b0) &&
                ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2));

        // Forwarding A
        if (ex_mem_reg_wen && (ex_mem_rd == id_ex_rs1) && (ex_mem_rd != 0))
            fwd_a = 2'b01;
        else if (mem_wb_reg_wen && (mem_wb_rd == id_ex_rs1) && (mem_wb_rd != 0))
            fwd_a = 2'b10;
        else
            fwd_a = 2'b00;

        // Forwarding B
        if (ex_mem_reg_wen && (ex_mem_rd == id_ex_rs2) && (ex_mem_rd != 0))
            fwd_b = 2'b01;
        else if (mem_wb_reg_wen && (mem_wb_rd == id_ex_rs2) && (mem_wb_rd != 0))
            fwd_b = 2'b10;
        else
            fwd_b = 2'b00;
    end
endmodule