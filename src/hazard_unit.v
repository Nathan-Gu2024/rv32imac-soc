`ifndef _HAZARD_UNIT_V_
`define _HAZARD_UNIT_V_

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
    // Whether the ID-stage instruction actually READS those two fields.
    //
    // if_id_rs1/rs2 are raw bit slices inst[19:15] and inst[24:20], and in
    // I-type, U-type and J-type instructions those positions are IMMEDIATE
    // payload. Without these qualifiers a numeric coincidence between an
    // immediate and the load's rd produced a stall for a dependence that does
    // not exist - `lw x10,0(sp)` followed by `addi a1,a2,10` stalled because
    // 10 == x10. Roughly 1/32 per exposed field, and U/J expose both.
    input wire if_id_rs1_used, if_id_rs2_used,

    // For forwarding
    input wire [4:0] ex_mem_rd, mem_wb_rd,
    input wire ex_mem_reg_wen, mem_wb_reg_wen,
    input wire [4:0] id_ex_rs1, id_ex_rs2,
    // wb_sel of the instruction in EX/MEM. 2'b00 means its rd comes from
    // MEMORY, and cpu.v's ex_mem_forward_data cannot supply that - for a load,
    // an LR or an AMO it resolves to ex_mem_alu, which is the effective
    // ADDRESS. See the guard below.
    input wire [1:0] ex_mem_wb_sel,

    // For branch flush
    input wire pc_sel,

    // Outputs
    output reg stall, // freeze IF/ID and PC
    output reg [1:0] fwd_a, fwd_b // 00=reg, 01=EX/MEM, 10=MEM/WB
);
    // An instruction in EX/MEM whose rd comes from MEMORY cannot be forwarded
    // from, because cpu.v's ex_mem_forward_data has no case for it: at
    // wb_sel==2'b00 it falls through to ex_mem_alu, which is the effective
    // ADDRESS rather than the loaded value. That covers loads, LR and AMOs.
    //
    // This is UNREACHABLE today and the guard is a safety net, not a bug fix.
    // The load-use stall below fires whenever a real instruction in ID depends
    // on a load in EX, and that stall flushes ID/EX - so on the cycle the load
    // occupies EX/MEM, ID/EX holds a bubble, and a bubble is the only thing that
    // can select this path. A bubble has reg_wen=0, so nothing observes it.
    //
    // It is written down and enforced because the invariant is invisible at the
    // mux in cpu.v, and the next change to the interlock (qualifying it by which
    // register fields the consumer actually reads, or letting a store proceed
    // and take its data in MEM) removes the very stall that makes it hold. Left
    // unguarded, that change turns into silent address-for-data corruption.
    wire ex_mem_fwdable = ex_mem_reg_wen && (ex_mem_wb_sel != 2'b00);

    always @(*) begin
        // Load-use stall, gated on id_ex_mem_read so bubbles never stall, and on
        // whether the consumer reads each field at all (see if_id_rs*_used).
        stall = id_ex_mem_read && (id_ex_rd != 5'b0) &&
                ((if_id_rs1_used && (id_ex_rd == if_id_rs1)) ||
                 (if_id_rs2_used && (id_ex_rd == if_id_rs2)));

        // Forwarding A
        if (ex_mem_fwdable && (ex_mem_rd == id_ex_rs1) && (ex_mem_rd != 0))
            fwd_a = 2'b01;
        else if (mem_wb_reg_wen && (mem_wb_rd == id_ex_rs1) && (mem_wb_rd != 0))
            fwd_a = 2'b10;
        else
            fwd_a = 2'b00;

        // Forwarding B
        if (ex_mem_fwdable && (ex_mem_rd == id_ex_rs2) && (ex_mem_rd != 0))
            fwd_b = 2'b01;
        else if (mem_wb_reg_wen && (mem_wb_rd == id_ex_rs2) && (mem_wb_rd != 0))
            fwd_b = 2'b10;
        else
            fwd_b = 2'b00;
    end
endmodule

`endif // _HAZARD_UNIT_V_
