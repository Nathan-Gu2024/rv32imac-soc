`timescale 1ns/1ps
`include "../src/trap_controller.v"
`include "../src/csr_file.v"

// Trap-path behaviour when the pipeline is FROZEN.
//
// trap_controller decodes ECALL and MRET combinationally from ex_inst, so a
// frozen instruction re-asserts its effect every cycle it sits in EX - and
// csr_file's trap and mret arms are read-modify-write on mstatus, so running
// them N times is not running them once. Nothing in the suite covered that,
// because every other trap test runs with an unstalled pipeline.
//
// Three bugs this pins down, all of which passed every pre-existing test:
//
//   F1a  ECALL under a stall destroyed MPIE. Cycle 1 saved the real MIE into
//        MPIE and cleared MIE; cycle 2 saved the now-zero MIE over it. The
//        later mret then restored MIE=0 and interrupts were dead for good.
//
//   F1b  MRET was not atomic against interrupts, and mret_exec was ungated.
//        MIE came back from MPIE on the first stalled cycle, so when the
//        stall dropped with the MRET still frozen in EX an interrupt won
//        priority and captured the MRET's OWN address as mepc. The handler
//        returned to the MRET, which returned to itself, for ever.
//
//   F1c  ECALL set mepc = pc+4 rather than the ECALL's own address, which is
//        neither what the spec says nor what Zephyr's isr.S expects.
//
// These are module-level rather than CPU-level on purpose: arranging a cache
// miss that lands exactly while an MRET sits in EX is far harder to set up
// than driving mem_stall directly, and proves less.
module tb_trap_stall;

    localparam [31:0] ECALL = 32'h00000073;
    localparam [31:0] MRET  = 32'h30200073;
    localparam [31:0] NOP   = 32'h00000013;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst          = 1'b1;
    reg [31:0] ex_pc        = 32'h0;
    reg [31:0] ex_inst      = NOP;
    reg        timer_irq    = 1'b0;
    reg        external_irq = 1'b0;
    reg        mem_stall    = 1'b0;

    wire        trap_taken, mret_exec;
    wire        flush_if, flush_id, flush_ex, pc_trap_override;
    wire [31:0] trap_cause, trap_pc, trap_target_pc, trap_val;
    wire [31:0] mtvec_out, mepc_out, csr_rdata;
    wire        mstatus_mie, mie_mtie, mie_meie;

    // This bench is about the stall interaction (MPIE, MRET atomicity), not
    // about faults, so the fault inputs are tied off rather than driven - but
    // they are tied off EXPLICITLY. Left dangling they read X, and
    // trap_controller.v:69 computes fault_illegal = ex_valid && illegal_inst
    // && !mem_stall, so the priority chain below would be selecting on X. The
    // tests still passed that way, because Verilog takes the else branch on
    // `if (X)` - they passed by accident of X semantics rather than because
    // the arms under test were the ones reached.
    trap_controller TC (
        .ex_pc(ex_pc), .ex_inst(ex_inst),
        .timer_irq(timer_irq), .external_irq(external_irq),
        .mtvec_out(mtvec_out), .mepc_out(mepc_out),
        .mem_stall(mem_stall),
        .ex_valid(1'b1),
        .illegal_inst(1'b0),
        .misalign_load(1'b0), .misalign_store(1'b0), .fault_addr(32'h0),
        .trap_taken(trap_taken), .trap_cause(trap_cause), .trap_pc(trap_pc),
        .trap_val(trap_val),
        .mret_exec(mret_exec),
        .flush_if(flush_if), .flush_id(flush_id), .flush_ex(flush_ex),
        .pc_trap_override(pc_trap_override), .trap_target_pc(trap_target_pc)
    );

    csr_file CSR (
        .clk(clk), .rst(rst),
        .csr_addr(12'h000), .csr_wdata(32'h0), .csr_wen(1'b0),
        .csr_op(2'b00), .csr_use_imm(1'b0), .csr_uimm(5'b0),
        .csr_rdata(csr_rdata),
        .trap_taken(trap_taken), .trap_pc(trap_pc), .trap_cause(trap_cause),
        .trap_val(trap_val),
        .mret_exec(mret_exec),
        .timer_pending(1'b0), .external_pending(1'b0),
        .mtvec_out(mtvec_out), .mepc_out(mepc_out), .mstatus_mie(mstatus_mie),
        .mie_mtie(mie_mtie), .mie_meie(mie_meie)
    );

    integer errors = 0;

    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst = 1'b0;
        @(negedge clk);

        // ---- F1a: an ECALL frozen for 5 cycles must act exactly once ----
        $display("--- F1a: ECALL under a stall must not destroy MPIE ---");
        CSR.mstatus = 32'h0000_0008;      // MIE=1, MPIE=0
        CSR.mtvec   = 32'h0000_0100;
        CSR.mepc    = 32'h0;
        ex_pc       = 32'h0000_4000;
        ex_inst     = ECALL;
        mem_stall   = 1'b1;
        repeat (5) @(negedge clk);
        check(CSR.mstatus[3] === 1'b1 && CSR.mstatus[7] === 1'b0,
              "frozen ECALL has no CSR side effect at all");
        mem_stall = 1'b0;
        @(posedge clk);                    // the one cycle it is allowed to fire
        @(negedge clk);
        ex_inst = NOP;                     // stands in for flush_ex
        check(CSR.mstatus[7] === 1'b1, "MPIE captured MIE=1, not a destroyed 0");
        check(CSR.mstatus[3] === 1'b0, "MIE cleared on trap entry");

        // ---- F1c: mepc is the ECALL's own address ----
        $display("--- F1c: ECALL mepc convention ---");
        @(negedge clk);
        CSR.mstatus = 32'h0000_0008;
        CSR.mepc    = 32'h0;
        ex_pc       = 32'h0000_4000;
        ex_inst     = ECALL;
        mem_stall   = 1'b0;
        @(posedge clk);
        @(negedge clk);
        ex_inst = NOP;
        check(CSR.mepc === 32'h0000_4000,
              "mepc = the ECALL address (handler advances it), not pc+4");

        // ---- F1b: an interrupt must not preempt a stalled MRET ----
        $display("--- F1b: MRET atomicity against a pending interrupt ---");
        @(negedge clk);
        CSR.mstatus = 32'h0000_0080;      // MPIE=1, MIE=0: inside a handler
        CSR.mepc    = 32'h0000_DEAD;      // the OUTER return address
        CSR.mtvec   = 32'h0000_0100;
        ex_pc       = 32'h0000_2000;      // the MRET's own address
        ex_inst     = MRET;
        mem_stall   = 1'b1;
        repeat (3) @(negedge clk);
        check(CSR.mstatus[3] === 1'b0,
              "frozen MRET does not restore MIE early");
        check(CSR.mepc === 32'h0000_DEAD,
              "frozen MRET leaves mepc alone");

        // Stall drops with a timer interrupt now pending. Pre-fix this is
        // where mepc became 0x2000 and the machine never escaped.
        mem_stall = 1'b0;
        timer_irq = 1'b1;
        @(posedge clk);
        @(negedge clk);
        check(CSR.mepc === 32'h0000_DEAD,
              "MRET won over the interrupt: mepc is still the outer address");
        check(trap_target_pc === 32'h0000_DEAD && pc_trap_override === 1'b1,
              "MRET redirected to the outer return address");
        check(CSR.mstatus[3] === 1'b1, "MRET restored MIE from MPIE");

        // The deferred interrupt must still be taken, one cycle later.
        ex_inst = NOP;
        ex_pc   = 32'h0000_2004;
        @(posedge clk);
        @(negedge clk);
        check(CSR.mcause === 32'h8000_0007 && CSR.mepc === 32'h0000_2004,
              "deferred timer interrupt is taken next cycle, not lost");
        timer_irq = 1'b0;

        // ---- regression: ordinary interrupt, ordinary MRET ----
        $display("--- regression: plain interrupt and plain MRET ---");
        @(negedge clk);
        CSR.mstatus  = 32'h0000_0008;
        CSR.mepc     = 32'h0;
        ex_pc        = 32'h0000_5000;
        ex_inst      = NOP;
        external_irq = 1'b1;
        @(posedge clk);
        @(negedge clk);
        external_irq = 1'b0;
        check(CSR.mepc === 32'h0000_5000 && CSR.mcause === 32'h8000_000B,
              "external interrupt: mepc = interrupted pc, mcause = 11 + top bit");

        @(negedge clk);
        CSR.mstatus = 32'h0000_0080;
        CSR.mepc    = 32'h0000_BEEF;
        ex_inst     = MRET;
        @(posedge clk);
        @(negedge clk);
        check(trap_target_pc === 32'h0000_BEEF && mret_exec === 1'b1 &&
              CSR.mstatus[3] === 1'b1,
              "unstalled MRET still returns and restores MIE");

        $display("");
        if (errors == 0) $display("=== TRAP STALL TESTS PASSED ===");
        else             $display("=== %0d TRAP STALL ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
