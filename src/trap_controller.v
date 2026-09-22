`ifndef _TRAP_CONTROLLER_V_
`define _TRAP_CONTROLLER_V_

module trap_controller (
    input wire [31:0] ex_pc, // pc of the instruction in EX
    input wire [31:0] ex_inst, // raw instruction currently in EX
    input wire timer_irq,    // CLINT timer interrupt, enabled+pending+globally-on
    input wire external_irq, // intc aggregate interrupt, enabled+pending+globally-on
    // from the CSR
    input wire [31:0] mtvec_out, // OS Kernel Address
    input wire [31:0] mepc_out, // Saved Return Address
    // Pipeline frozen this cycle (global_mem_stall | load-use stall).
    //
    // ECALL and MRET are decoded COMBINATIONALLY from ex_inst, so a frozen
    // instruction re-asserts its effect every cycle it sits in EX - and the
    // CSR side effects below are not idempotent. See the two comment blocks
    // at mret_exec and the ECALL arm for what that destroyed.
    input wire mem_stall,
    // Synchronous exceptions, raised in EX by the instruction in EX.
    //
    // ex_valid matters: a bubble injected by a flush or a load-use stall has
    // no instruction in it, and its ex_inst is whatever the clear left behind
    // (0 for the pipeline registers here) - which is itself an illegal
    // encoding. Without this gate every flush would trap.
    input wire ex_valid,
    input wire illegal_inst,
    // Misaligned data address, detected in EX from the same adder the branch
    // redirect uses. It has to be EX: flush_ex clearing EX/MEM is the only
    // mechanism in this design for cancelling a memory side effect, and by
    // the time the address reaches ex_mem_alu the cache enables, store commit
    // and every MMIO request line are already derived from it.
    input wire misalign_load,
    input wire misalign_store,
    input wire [31:0] fault_addr,
    // to the CSR
    output reg trap_taken,
    output reg [31:0] trap_cause,
    output reg [31:0] trap_val,
    output reg [31:0] trap_pc,
    output wire mret_exec,
    // to the pipeline (takeover)
    output reg flush_if, // Flush IF/ID register
    output reg flush_id, // Flush ID/EX register
    output reg flush_ex, // Flush EX/MEM register
    output reg pc_trap_override, // force global pc to jump
    output reg [31:0] trap_target_pc // address to jump to
);
    wire is_system = (ex_inst[6:0] == 7'b1110011);
    // ECALL: 12'b000000000000 in the top 12 bits
    wire is_ecall = is_system && (ex_inst[31:20] == 12'b000000000000);
    // MRET: 12'b001100000010 in the top 12 bits
    wire is_mret = is_system && (ex_inst[31:20] == 12'b001100000010);

    // A frozen ECALL/MRET must not act until the cycle the pipeline actually
    // moves. Same gate cpu.v already applies to ex_csr_wen, and for the same
    // reason: csr_file's trap and mret arms are read-modify-write on mstatus,
    // so running them N times is not running them once.
    //
    // mret_exec was previously `= is_mret`, ungated. An I-cache miss while an
    // MRET sat in EX restored mstatus.MIE from MPIE on the FIRST stalled
    // cycle, i.e. before the MRET retired. Interrupts are suppressed during a
    // stall, so nothing fired then - but when the stall dropped, the MRET was
    // still frozen in EX with MIE now 1, an interrupt won priority over the
    // is_mret arm below, and mepc was overwritten with the MRET's OWN address.
    // The handler then returned to the MRET, which returned to itself, for
    // ever. No nested interrupt and no ECALL required.
    wire mret_fires  = is_mret  && !mem_stall;
    wire ecall_fires = is_ecall && !mem_stall;
    assign mret_exec = mret_fires;

    // A real instruction in EX that decode could not recognise.
    wire fault_illegal   = ex_valid && illegal_inst   && !mem_stall;
    wire fault_ld_align  = ex_valid && misalign_load  && !mem_stall;
    wire fault_st_align  = ex_valid && misalign_store && !mem_stall;

    // Standard RISC-V machine-level priority when both are pending at once:
    // external before timer (there's no software-interrupt source wired up).
    //
    // ~is_mret makes MRET atomic against interrupts, which is the other half
    // of the hang above: an interrupt must never preempt an MRET that has not
    // retired, or it captures the MRET's address as the return address. The
    // interrupt is only deferred, not lost - timer_irq/external_irq are level
    // conditions derived from latched mip bits (see cpu.v's note on why
    // deferring an interrupt across a stall is safe), so it is taken on the
    // cycle after the MRET completes. Gated on is_mret rather than mret_fires
    // so the exclusion holds even while frozen, independent of how the caller
    // happens to gate timer_irq/external_irq.
    wire hw_irq = (timer_irq | external_irq) && !is_mret;

    always @(*) begin
        // Default
        trap_taken = 1'b0;
        trap_cause = 32'b0;
        trap_val = 32'b0;
        trap_pc = 32'b0;
        flush_if = 1'b0;
        flush_id = 1'b0;
        flush_ex = 1'b0;
        pc_trap_override = 1'b0;
        trap_target_pc = 32'b0;
        // HW interrupts cpu
        if (hw_irq) begin
            trap_taken = 1'b1;
            // mcause's top bit marks a hardware interrupt (vs. an
            // exception); the low bits are the standard machine-mode cause
            // codes: 7 = timer, 11 = external.
            trap_cause = external_irq ? 32'h8000_000B : 32'h8000_0007;
            trap_pc = ex_pc; // Save the PC so we can resume later
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1; // Nuke the pipeline
            pc_trap_override = 1'b1;
            trap_target_pc = mtvec_out; // Jump to OS Kernel
        end else if (fault_illegal) begin
            // Illegal instruction. Below interrupts and above ECALL/MRET:
            // an interrupt is asynchronous and may preempt anything, but a
            // faulting instruction never gets to act as an ECALL or an MRET.
            //
            // mepc is the FAULTING instruction, not the one after it, so a
            // handler that fixes the cause can resume by returning. That is
            // also why ECALL's `+4` had to go - two conventions in one
            // controller would be unresolvable from the handler's side.
            trap_taken = 1'b1;
            trap_cause = 32'd2;      // Illegal instruction
            trap_val = ex_inst;      // mtval = the offending encoding
            trap_pc = ex_pc;
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1;
            pc_trap_override = 1'b1;
            trap_target_pc = mtvec_out;
        end else if (fault_ld_align || fault_st_align) begin
            // Misaligned load/store/AMO. Below illegal-instruction in the
            // spec's priority order, which is also the only sensible one: an
            // undecodable word has no meaningful address to report.
            //
            // mtval is the ADDRESS here, not the instruction - that is the
            // whole reason mtval is a separate register from mcause.
            trap_taken = 1'b1;
            trap_cause = fault_ld_align ? 32'd4 : 32'd6;
            trap_val = fault_addr;
            trap_pc = ex_pc;
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1;
            pc_trap_override = 1'b1;
            trap_target_pc = mtvec_out;
        end else if (ecall_fires) begin // Software ASKED for interrupt (ECALL)
            trap_taken = 1'b1;
            trap_cause = 32'd11; // Environment Call from M-Mode
            // mepc = the ECALL's OWN address, per the privileged spec. The
            // HANDLER advances it before mret - which is what Zephyr's
            // arch/riscv/core/isr.S does.
            //
            // This was `ex_pc + 32'd4`. Two things were wrong with it: it
            // silently skipped the instruction after the ecall for any
            // spec-conforming handler, and the +4 was unconditional, so it
            // also ignored id_ex_compressed. It is changed here rather than
            // later because the access-fault and illegal-instruction causes
            // MUST report the faulting address, and two opposite mepc
            // conventions inside one trap_controller is how that becomes
            // unfixable. Nothing depends on the old behaviour: there is no
            // ecall anywhere in fpga/tests/ or zephyr/ today.
            trap_pc = ex_pc;
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1;
            pc_trap_override = 1'b1;
            trap_target_pc = mtvec_out;
        end else if (mret_fires) begin // Software RETURNING from interrupt (MRET)
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1; // flush
            pc_trap_override = 1'b1;
            trap_target_pc = mepc_out; // Jump back to where we were before the trap
        end
    end
endmodule

`endif // _TRAP_CONTROLLER_V_
