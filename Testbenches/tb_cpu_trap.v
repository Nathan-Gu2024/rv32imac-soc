// CPU-LEVEL trap test: illegal instruction and misaligned load/store, checked
// through the real cpu_pipelined rather than a hand-wired trap_controller +
// csr_file pair.
//
// This bench exists because tb_illegal_inst.v cannot catch the class of bug
// that actually shipped. That bench instantiates trap_controller and csr_file
// side by side and connects .trap_val itself, so it proves the two modules
// agree - and says nothing about whether cpu.v connects them. It did not:
// csr_file's trap_val input was left dangling, so mtval latched X on every
// trap in real hardware while every module-level test passed.
//
// The rule this encodes: a value that crosses a module boundary has to be
// checked on the far side of that boundary, in the integration, or the wire
// itself is untested.
//
// Every DUT input is tied off explicitly. Icarus does not error on a missing
// named port - it warns only under -Wall, which no build script here passes -
// so an unconnected input is invisible unless the bench refuses to leave any.

`include "../src/cpu.v"

module tb_cpu_trap;
    reg clk = 0;
    reg rst;
    reg uart_rx_i = 1'b1;          // idle high

    // Off-chip memory is never answered: the test program lives entirely in
    // TCM, so anything that reaches the cache fabric is a bug rather than a
    // stall to be serviced.
    wire [127:0] icache_rdata = 128'h0;
    wire         icache_rdy   = 1'b0;
    wire [127:0] dcache_rdata = 128'h0;
    wire         dcache_rdy   = 1'b0;
    wire         accel_rdy    = 1'b0;
    // Observed only - the DUT drives these and nothing reads them.
    wire         accel_rd_req_valid, accel_wr_req_valid;
    wire [31:0]  accel_rd_req_addr,  accel_wr_req_addr;
    wire [7:0]   accel_rd_req_lines, accel_wr_req_lines;
    wire         accel_wnext  = 1'b0;
    wire [127:0] accel_rline  = 128'h0;

    wire uart_tx_o;
    wire [3:0] leds_o;
    wire [31:0] icache_req_addr;
    wire icache_req_valid;
    wire [31:0] dmem_addr, store_data_o;
    wire [3:0]  wmask_o;
    wire dcache_req_valid, dcache_req_write;
    wire [127:0] dcache_wline_o;
    wire [127:0] accel_wline_o;
    wire [31:0] dbg_pc, dbg_instr, dbg_raw_pc;
    wire dbg_dcache_valid, dbg_dcache_ready, dbg_tcm_d_req, dbg_tcm_d_ready;
    wire dbg_global_mem_stall, dbg_id_pred_taken, dbg_cache_ready;

    cpu_pipelined DUT (
        .clk(clk), .rst(rst),
        .uart_tx(uart_tx_o), .uart_rx(uart_rx_i),
        .leds(leds_o),
        .icache_mem_req_addr(icache_req_addr),
        .icache_mem_req_valid(icache_req_valid),
        .icache_mem_read_data(icache_rdata),
        .icache_mem_ready(icache_rdy),
        .dmem_req_addr(dmem_addr),
        .store_data(store_data_o),
        .mem_write_mask(wmask_o),
        .dcache_mem_req_valid(dcache_req_valid),
        .dcache_mem_req_write(dcache_req_write),
        .dcache_mem_wline(dcache_wline_o),
        .dcache_mem_read_data_block(dcache_rdata),
        .dcache_mem_ready(dcache_rdy),
        // Split port. This bench never drives a transfer; the accelerator is
        // tied off and only observed, so both channels are simply named.
        .accel_mem_rd_req_valid(accel_rd_req_valid),
        .accel_mem_rd_req_addr(accel_rd_req_addr),
        .accel_mem_rd_req_lines(accel_rd_req_lines),
        .accel_mem_rd_ready(accel_rdy),
        .accel_mem_rline(accel_rline),
        .accel_mem_wr_req_valid(accel_wr_req_valid),
        .accel_mem_wr_req_addr(accel_wr_req_addr),
        .accel_mem_wr_req_lines(accel_wr_req_lines),
        .accel_mem_wline(accel_wline_o),
        .accel_mem_wnext(accel_wnext),
        .accel_mem_wr_ready(accel_rdy),
        .debug_pc(dbg_pc),
        .debug_instr(dbg_instr),
        .debug_dcache_valid(dbg_dcache_valid),
        .debug_dcache_ready(dbg_dcache_ready),
        .debug_tcm_d_req(dbg_tcm_d_req),
        .debug_tcm_d_ready(dbg_tcm_d_ready),
        .debug_global_mem_stall(dbg_global_mem_stall),
        .debug_raw_pc(dbg_raw_pc),
        .debug_id_predicted_taken(dbg_id_pred_taken),
        .debug_cache_ready(dbg_cache_ready)
    );

    always #10 clk = ~clk;         // 50 MHz

    localparam TCM_WORDS = 16384;
    localparam HANDLER_W = 16;     // word index of the handler
    // Word index of a SECOND spin, placed between the instruction under test
    // and the handler, whose only job is to catch the fall-through path.
    //
    // Without it, checking the PC proves nothing: every untouched TCM word is a
    // NOP, so a trap whose redirect never took effect just walks forward
    // through the fill and arrives at the handler anyway, a dozen cycles later
    // and for entirely the wrong reason. With it the two outcomes land at two
    // different addresses, and the PC becomes decisive.
    localparam FALLTHRU_W = 8;     // 0x40000020
    localparam [31:0] HANDLER_PC  = 32'h4000_0000 + (HANDLER_W  * 4);
    localparam [31:0] FALLTHRU_PC = 32'h4000_0000 + (FALLTHRU_W * 4);
    localparam [31:0] NOP = 32'h0000_0013;   // addi x0,x0,0

    integer fails = 0;
    integer i;

    task check;
        input cond;
        input [8*56:1] label;
        begin
            if (cond) $display("  PASS  %0s", label);
            else begin
                $display("  FAIL  %0s", label);
                fails = fails + 1;
            end
        end
    endtask

    // Preamble, then the instruction under test, then a handler that spins.
    //
    // mtvec resets to 0 (csr_file.v:83) and TCM lives at 0x40000000, so the
    // program has to point mtvec somewhere real before it faults - otherwise
    // the trap vectors into unmapped space and the failure mode is a hang
    // rather than a readable mcause.
    // follow_inst is the instruction placed immediately AFTER the faulting one.
    //
    // It is a parameter rather than always a NOP because the instruction that
    // follows changes whether the trap fires at all. trap_controller gates every
    // fault on !mem_stall, and cpu.v feeds that `global_mem_stall | stall` where
    // `stall` is the load-use interlock - so a misaligned load whose result the
    // next instruction consumes has its fault suppressed on the only cycle it
    // could fire, and ID/EX is flushed the cycle after, losing it forever.
    //
    // A bench that puts NOPs here therefore passes against that bug. This one
    // did, which is exactly why the bug shipped.
    task run_case;
        input [31:0] fault_inst;
        input [31:0] follow_inst;
        input [31:0] exp_cause;
        input [31:0] exp_tval;
        input [8*56:1] label;
        integer guard;
        begin
            $display("%0s", label);
            rst = 1;
            @(posedge clk);

            for (i = 0; i < TCM_WORDS; i = i + 1) begin
                DUT.TCM.mem_even[i] = 16'h0013;  // nop = addi x0,x0,0
                DUT.TCM.mem_odd[i]  = 16'h0000;
            end

            // lui  t0, 0x40000     -> t0 = 0x40000000
            DUT.TCM.mem_even[0] = 16'h02B7; DUT.TCM.mem_odd[0] = 16'h4000;
            // addi t0, t0, 0x40    -> t0 = 0x40000040, the handler
            DUT.TCM.mem_even[1] = 16'h8293; DUT.TCM.mem_odd[1] = 16'h0402;
            // csrw mtvec, t0
            DUT.TCM.mem_even[2] = 16'h9073; DUT.TCM.mem_odd[2] = 16'h3052;
            // the instruction under test, at 0x4000000C
            DUT.TCM.mem_even[3] = fault_inst[15:0];
            DUT.TCM.mem_odd[3]  = fault_inst[31:16];
            // and its successor, which decides whether a load-use stall
            // coincides with the fault
            DUT.TCM.mem_even[4] = follow_inst[15:0];
            DUT.TCM.mem_odd[4]  = follow_inst[31:16];
            // fall-through catcher: j self. Reached only if the trap failed to
            // transfer control.
            DUT.TCM.mem_even[FALLTHRU_W] = 16'h006F;
            DUT.TCM.mem_odd[FALLTHRU_W]  = 16'h0000;
            // handler: j handler  (jal x0, 0) - spin so the CSRs stay put
            DUT.TCM.mem_even[HANDLER_W] = 16'h006F;
            DUT.TCM.mem_odd[HANDLER_W]  = 16'h0000;

            repeat (3) @(posedge clk);
            rst = 0;

            guard = 0;
            while (DUT.CSR.mcause === 32'b0 && guard < 400) begin
                @(posedge clk);
                guard = guard + 1;
            end

            check(guard < 400, "trap fired");
            check(DUT.CSR.mcause === exp_cause, "mcause");
            // The whole point of this bench. mtval is driven by
            // trap_controller and consumed by csr_file, and nothing below the
            // top level can tell whether cpu.v joined them up.
            check(DUT.CSR.mtval === exp_tval,
                  "mtval (crosses trap_controller -> csr_file)");
            check(DUT.CSR.mepc === 32'h4000_000C,
                  "mepc = the faulting instruction");
            if (DUT.CSR.mtval !== exp_tval)
                $display("        mtval expected %h, got %h",
                         exp_tval, DUT.CSR.mtval);

            // Did control actually REACH the handler?
            //
            // Every check above reads a CSR, and all of them are written by the
            // same trap_taken pulse - so they pass whether or not the pipeline
            // ever redirected. That is not hypothetical: the PC freeze from the
            // load-use interlock sat ABOVE pc_sel inside program_counter, so on
            // the DEPENDENT case the fault fired, all three flushes fired,
            // mcause/mepc/mtval all latched correctly, and the PC held anyway.
            // The handler never ran and the program continued straight past the
            // faulting load. This bench reported 16/16 against that bug, which
            // is the only reason the check below exists.
            guard = 0;
            while (DUT.pc !== HANDLER_PC && DUT.pc !== FALLTHRU_PC
                   && guard < 60) begin
                @(posedge clk);
                guard = guard + 1;
            end
            check(DUT.pc === HANDLER_PC,
                  "control REACHED the handler, not just mcause written");
            if (DUT.pc !== HANDLER_PC)
                $display("        pc = %h; handler is %h, fall-through is %h",
                         DUT.pc, HANDLER_PC, FALLTHRU_PC);
        end
    endtask

    initial begin
        $dumpfile("tb_cpu_trap.vcd");
        $dumpvars(0, tb_cpu_trap);

        $display("=== CPU-level trap test ===");

        // 0xFFFFFFFF is one of the two encodings control_logic.v:168 rejects
        // outright. mtval is the offending word itself.
        run_case(32'hFFFF_FFFF, NOP, 32'd2, 32'hFFFF_FFFF,
                 "illegal instruction (all ones)");

        // lw t1, 2(x0) - word access to address 2. Caught in EX off
        // redirect_target_adder, before partial_load ever sees it.
        run_case(32'h0020_2303, NOP, 32'd4, 32'd2,
                 "misaligned lw -> mcause 4, mtval = address");

        // sw t0, 2(x0)
        run_case(32'h0050_2123, NOP, 32'd6, 32'd2,
                 "misaligned sw -> mcause 6, mtval = address");

        // THE REGRESSION CASE. Identical misaligned lw, but followed by
        // `addi t2, t1, 1` which reads the load's own rd (t1/x6). That raises
        // the load-use interlock on the same cycle the fault wants to fire.
        //
        // A load is the only instruction that can do this: `stall` requires
        // id_ex_mem_read, so an ECALL, MRET or illegal instruction in EX can
        // never coincide with it. Stores cannot either - they have reg_wen=0,
        // so nothing can depend on them.
        run_case(32'h0020_2303, 32'h0013_0393, 32'd4, 32'd2,
                 "misaligned lw + DEPENDENT next inst (load-use stall)");

        $display("=== %0d failures ===", fails);
        if (fails == 0) $display("ALL PASS");
        $finish;
    end
endmodule
