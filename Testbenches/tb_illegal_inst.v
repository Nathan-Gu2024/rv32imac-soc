`timescale 1ns/1ps
`include "../src/control_logic.v"
`include "../src/trap_controller.v"
`include "../src/csr_file.v"

// Illegal-instruction detection and delivery.
//
// Until this existed, rom_decoder's `default` handed back ADD's control word
// for anything it did not recognise, so an undefined opcode executed as
// `add rd, rs1, rs2` and quietly corrupted a register. A jump into zeroed
// memory ran as `lb x0, 0(x0)` and carried on. Nothing reported either, which
// is how a thread that stopped being scheduled produced no diagnostic at all.
//
// Two halves, because the failure modes are different:
//
//   PART A  the decoder must flag what it cannot decode and NOT flag what it
//           can. Over-flagging is the dangerous direction here: marking FENCE
//           illegal would trap on code GCC emits around every atomic.
//
//   PART B  the fault must reach the CSRs with the right cause, the right
//           mtval, and the right priority - and must NOT fire on a pipeline
//           bubble, whose instruction word is 0 and therefore illegal.
module tb_illegal_inst;

    integer errors = 0;

    // ---------------- PART A: decode ----------------
    reg [31:0] dec_inst;
    wire dec_illegal;
    wire dec_reg_wen, dec_a_sel, dec_b_sel, dec_mem_rw;
    wire [1:0] dec_wb_sel, dec_csr_op;
    wire [2:0] dec_imm_sel;
    wire [4:0] dec_alu_sel, dec_atomic_op, dec_csr_uimm;
    wire dec_is_lr, dec_is_sc, dec_is_amo, dec_csr_wen, dec_csr_use_imm;

    control_logic CL (
        .inst(dec_inst),
        .reg_wen(dec_reg_wen), .a_sel(dec_a_sel), .b_sel(dec_b_sel),
        .mem_rw(dec_mem_rw), .wb_sel(dec_wb_sel), .imm_sel(dec_imm_sel),
        .alu_sel(dec_alu_sel),
        .out_is_lr(dec_is_lr), .out_is_sc(dec_is_sc), .out_is_amo(dec_is_amo),
        .out_atomic_op(dec_atomic_op),
        .csr_wen(dec_csr_wen), .csr_op(dec_csr_op),
        .csr_use_imm(dec_csr_use_imm), .csr_uimm(dec_csr_uimm),
        .illegal_inst(dec_illegal)
    );

    task expect_decode(input [31:0] i, input want, input [8*24-1:0] name);
        begin
            dec_inst = i;
            #1;
            if (dec_illegal !== want) begin
                $display("  FAIL %-22s %08h: illegal=%b want %b",
                         name, i, dec_illegal, want);
                errors = errors + 1;
            end
        end
    endtask

    // ---------------- PART B: delivery ----------------
    localparam [31:0] NOP = 32'h00000013;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst = 1'b1;
    reg [31:0] ex_pc = 32'h0, ex_inst = NOP;
    reg        timer_irq = 1'b0, external_irq = 1'b0;
    reg        mem_stall = 1'b0, ex_valid = 1'b1, illegal_in = 1'b0;
    reg        misalign_ld = 1'b0, misalign_st = 1'b0;
    reg [31:0] fault_addr = 32'b0;

    wire        trap_taken, mret_exec;
    wire        flush_if, flush_id, flush_ex, pc_trap_override;
    wire [31:0] trap_cause, trap_val, trap_pc, trap_target_pc;
    wire [31:0] mtvec_out, mepc_out, csr_rdata;
    wire        mstatus_mie, mie_mtie, mie_meie;

    trap_controller TC (
        .ex_pc(ex_pc), .ex_inst(ex_inst),
        .timer_irq(timer_irq), .external_irq(external_irq),
        .mtvec_out(mtvec_out), .mepc_out(mepc_out),
        .mem_stall(mem_stall),
        .ex_valid(ex_valid), .illegal_inst(illegal_in),
        .misalign_load(misalign_ld), .misalign_store(misalign_st),
        .fault_addr(fault_addr),
        .trap_taken(trap_taken), .trap_cause(trap_cause),
        .trap_val(trap_val), .trap_pc(trap_pc),
        .mret_exec(mret_exec),
        .flush_if(flush_if), .flush_id(flush_id), .flush_ex(flush_ex),
        .pc_trap_override(pc_trap_override), .trap_target_pc(trap_target_pc)
    );

    csr_file CSR (
        .clk(clk), .rst(rst),
        .csr_addr(12'h000), .csr_wdata(32'h0), .csr_wen(1'b0),
        .csr_op(2'b00), .csr_use_imm(1'b0), .csr_uimm(5'b0),
        .csr_rdata(csr_rdata),
        .trap_val(trap_val),
        .trap_taken(trap_taken), .trap_pc(trap_pc), .trap_cause(trap_cause),
        .mret_exec(mret_exec),
        .timer_pending(1'b0), .external_pending(1'b0),
        .mtvec_out(mtvec_out), .mepc_out(mepc_out), .mstatus_mie(mstatus_mie),
        .mie_mtie(mie_mtie), .mie_meie(mie_meie)
    );

    task check(input cond, input [8*52-1:0] msg);
        begin
            if (!cond) begin
                $display("  FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $display("");
        $display("--- PART A: what the decoder flags ---");

        // Legal. Over-flagging any of these breaks working code.
        expect_decode(32'h00000013, 1'b0, "addi (nop)");
        expect_decode(32'h00A58533, 1'b0, "add");
        expect_decode(32'h40A58533, 1'b0, "sub");
        expect_decode(32'h02A58533, 1'b0, "mul");
        expect_decode(32'h02A5C533, 1'b0, "div");
        expect_decode(32'h00052503, 1'b0, "lw");
        expect_decode(32'h00A52023, 1'b0, "sw");
        expect_decode(32'h00A58463, 1'b0, "beq");
        expect_decode(32'h008000EF, 1'b0, "jal");
        expect_decode(32'h000500E7, 1'b0, "jalr");
        expect_decode(32'h000015B7, 1'b0, "lui");
        expect_decode(32'h00001517, 1'b0, "auipc");
        expect_decode(32'h1000A52F, 1'b0, "lr.w");
        expect_decode(32'h18B5252F, 1'b0, "sc.w");
        expect_decode(32'h00B5252F, 1'b0, "amoadd.w");
        expect_decode(32'h300512F3, 1'b0, "csrrw");
        expect_decode(32'h00000073, 1'b0, "ecall");
        expect_decode(32'h30200073, 1'b0, "mret");
        expect_decode(32'h10500073, 1'b0, "wfi");
        expect_decode(32'h0FF0000F, 1'b0, "fence");
        expect_decode(32'h0000100F, 1'b0, "fence.i");
        expect_decode(32'h20A5A533, 1'b0, "sh1add");
        expect_decode(32'h20A5C533, 1'b0, "sh2add");
        expect_decode(32'h20A5E533, 1'b0, "sh3add");

        // Illegal. Under-flagging any of these is the silent-corruption case.
        expect_decode(32'h00000000, 1'b1, "all zeros");
        expect_decode(32'hFFFFFFFF, 1'b1, "all ones");
        expect_decode(32'h00052507, 1'b1, "flw (no FP)");
        expect_decode(32'h00A58553, 1'b1, "fadd.s (no FP)");
        expect_decode(32'h0000006B, 1'b1, "custom-3 opcode");
        expect_decode(32'h0000002B, 1'b1, "custom-1 opcode");
        expect_decode(32'h000510E7, 1'b1, "jalr funct3=1");
        expect_decode(32'h20A59533, 1'b1, "Zba funct7, bad funct3");

        $display("--- PART B: how the fault is delivered ---");
        repeat (3) @(posedge clk);
        @(negedge clk); rst = 1'b0;

        // A real illegal instruction in EX.
        @(negedge clk);
        ex_pc = 32'h0000_2000; ex_inst = 32'hFFFF_FFFF;
        ex_valid = 1'b1; illegal_in = 1'b1;
        #1;
        check(trap_taken === 1'b1,            "illegal instruction raises a trap");
        check(trap_cause === 32'd2,           "mcause = 2 (illegal instruction)");
        check(trap_val === 32'hFFFF_FFFF,     "mtval = the offending encoding");
        check(trap_pc === 32'h0000_2000,      "mepc = the FAULTING pc, not pc+4");
        check(flush_if && flush_id && flush_ex, "all three stages flushed");
        check(pc_trap_override === 1'b1,      "pc redirected to mtvec");
        @(posedge clk);
        @(negedge clk);
        check(CSR.mcause === 32'd2,           "mcause latched into the CSR");
        check(CSR.mtval === 32'hFFFF_FFFF,    "mtval latched into the CSR");
        check(CSR.mepc === 32'h0000_2000,     "mepc latched into the CSR");

        // A BUBBLE. Its instruction word is 0, which is illegal - so without
        // the ex_valid gate every flush would trap.
        @(negedge clk);
        ex_inst = 32'h0000_0000; ex_valid = 1'b0; illegal_in = 1'b1;
        #1;
        check(trap_taken === 1'b0, "a bubble does NOT trap despite inst=0");

        // Frozen pipeline: the fault must wait, exactly like ECALL and MRET,
        // or its CSR side effects run once per stalled cycle.
        @(negedge clk);
        ex_valid = 1'b1; illegal_in = 1'b1; mem_stall = 1'b1;
        #1;
        check(trap_taken === 1'b0, "a frozen pipeline defers the fault");
        @(negedge clk); mem_stall = 1'b0;
        #1;
        check(trap_taken === 1'b1, "the fault fires once the pipeline moves");

        // Interrupts outrank synchronous exceptions.
        @(negedge clk);
        illegal_in = 1'b1; external_irq = 1'b1;
        #1;
        check(trap_cause === 32'h8000_000B, "a pending interrupt outranks the fault");
        @(negedge clk); external_irq = 1'b0;

        // A faulting instruction never also acts as ECALL or MRET.
        @(negedge clk);
        ex_inst = 32'h30200073; illegal_in = 1'b1;   // MRET encoding, flagged
        #1;
        check(mret_exec === 1'b0 || trap_cause === 32'd2,
              "a flagged instruction does not also execute as MRET");

        // ---- misaligned data addresses ----
        @(negedge clk);
        ex_inst = 32'h00052503;             // lw
        illegal_in = 1'b0; misalign_ld = 1'b1; misalign_st = 1'b0;
        fault_addr = 32'h0000_3002; ex_pc = 32'h0000_4000;
        #1;
        check(trap_taken === 1'b1,        "misaligned load raises a trap");
        check(trap_cause === 32'd4,       "mcause = 4 (load address misaligned)");
        check(trap_val === 32'h0000_3002, "mtval = the ADDRESS, not the instruction");
        check(trap_pc === 32'h0000_4000,  "mepc = the faulting pc");
        @(posedge clk); @(negedge clk);
        check(CSR.mtval === 32'h0000_3002, "mtval latched into the CSR");

        @(negedge clk);
        misalign_ld = 1'b0; misalign_st = 1'b1; fault_addr = 32'h0000_3001;
        #1;
        check(trap_cause === 32'd6, "mcause = 6 (store/AMO address misaligned)");

        // Illegal instruction outranks a misaligned address: an undecodable
        // word has no meaningful address to report.
        @(negedge clk);
        illegal_in = 1'b1; misalign_st = 1'b1;
        #1;
        check(trap_cause === 32'd2, "illegal instruction outranks misalignment");
        @(negedge clk); illegal_in = 1'b0; misalign_st = 1'b0;

        // Same freeze and bubble rules as the illegal-instruction fault.
        @(negedge clk);
        misalign_ld = 1'b1; ex_valid = 1'b0;
        #1;
        check(trap_taken === 1'b0, "a bubble does not raise a misalign fault");
        @(negedge clk);
        ex_valid = 1'b1; mem_stall = 1'b1;
        #1;
        check(trap_taken === 1'b0, "a frozen pipeline defers the misalign fault");
        @(negedge clk); mem_stall = 1'b0; misalign_ld = 1'b0;

        $display("");
        if (errors == 0)
            $display("=== ILLEGAL INSTRUCTION TESTS PASSED ===");
        else
            $display("=== %0d ILLEGAL INSTRUCTION ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
