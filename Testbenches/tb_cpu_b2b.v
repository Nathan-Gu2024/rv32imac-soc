// Back-to-back multi-cycle operations at the CPU level.
//
// This covers a class of bug the existing suite cannot reach. `div_unit` and the
// AMO datapath are both correct in isolation; what fails is cpu.v's HANDSHAKE
// with them, on the single cycle where one multi-cycle op retires and the next
// enters the same stage. A unit test of the divider would pass against the bug,
// which is why there has never been a divider bench and the bug shipped anyway.
//
// The shape, in both cases, is a "result ready" flag cleared by the OPERAND TYPE
// of the instruction in the stage rather than by retirement:
//
//     else if (!is_div_op) div_result_ready <= 1'b0;
//
// Non-blocking assignment samples the OUTGOING instruction, which is still a
// divide, so the clear never happens across a div->div boundary and the second
// divide never starts.
//
// WHY COREMARK NEVER CAUGHT IT: GCC's divmod idiom is `div rq,a,b` followed by
// `rem rr,a,b` on the SAME operands, and div_unit produces quotient and
// remainder in one pass - so the stale registers happen to hold the right
// answer. The test below reproduces that idiom deliberately (t6) alongside the
// case that breaks (t5), so the masking is visible rather than mysterious.

`include "../src/cpu.v"

module tb_cpu_b2b;
    reg clk = 0;
    reg rst;
    reg uart_rx_i = 1'b1;

    // Nothing off-chip answers: the program lives entirely in TCM, so any
    // request reaching the cache fabric is a bug rather than a stall.
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

    always #10 clk = ~clk;

    localparam TCM_WORDS = 16384;

    integer fails = 0;
    integer i;

    task check;
        input [31:0] got;
        input [31:0] want;
        input [8*52:1] label;
        begin
            if (got === want) begin
                $display("  PASS  %0s = %0d", label, got);
            end else begin
                $display("  FAIL  %0s = %0d, want %0d", label, got, want);
                fails = fails + 1;
            end
        end
    endtask

    task put;
        input integer w;
        input [31:0] inst;
        begin
            DUT.TCM.mem_even[w] = inst[15:0];
            DUT.TCM.mem_odd[w]  = inst[31:16];
        end
    endtask

    initial begin
        $dumpfile("tb_cpu_b2b.vcd");
        $dumpvars(0, tb_cpu_b2b);

        $display("=== back-to-back multi-cycle ops ===");

        rst = 1;
        @(posedge clk);

        for (i = 0; i < TCM_WORDS; i = i + 1) begin
            DUT.TCM.mem_even[i] = 16'h0013;   // nop
            DUT.TCM.mem_odd[i]  = 16'h0000;
        end

        put(0, 32'h0640_0293);   // addi t0, x0, 100
        put(1, 32'h0070_0313);   // addi t1, x0, 7
        put(2, 32'h0140_0393);   // addi t2, x0, 20
        put(3, 32'h0030_0E13);   // addi t3, x0, 3

        // The pair under test: two divides with DIFFERENT operands, adjacent.
        put(4, 32'h0262_CEB3);   // div  t4, t0, t1   -> 100/7 = 14
        put(5, 32'h03C3_CF33);   // div  t5, t2, t3   ->  20/3 = 6

        // Control: a separated divide op always worked, so this must stay green
        // and proves the fix did not simply disable the divider.
        put(6, 32'h0000_0013);   // nop
        put(7, 32'h0262_EFB3);   // rem  t6, t0, t1   -> 100%7 = 2

        // ---- AMO phase ----
        //
        // Target 0x40000400 / 0x40000410 - inside the TCM, which dcache.v:51-52
        // routes straight to the TCM port rather than through the cache array,
        // so this needs no external memory (mem_ready is tied low here). Word
        // 256/260, far past the program at words 0-20.
        put(8,  32'h4000_0437);  // lui  s0, 0x40000
        put(9,  32'h4004_0413);  // addi s0, s0, 1024   -> 0x40000400
        put(10, 32'h0104_0493);  // addi s1, s0, 16     -> 0x40000410
        put(11, 32'h00A0_0513);  // addi a0, x0, 10
        put(12, 32'h0640_0593);  // addi a1, x0, 100
        put(13, 32'h00A4_2023);  // sw   a0, 0(s0)
        put(14, 32'h00B4_A023);  // sw   a1, 0(s1)
        put(15, 32'h0050_0613);  // addi a2, x0, 5
        put(16, 32'h0070_0693);  // addi a3, x0, 7

        // The pair under test: two AMOs to DIFFERENT addresses, adjacent.
        put(17, 32'h00C4_272F);  // amoadd.w a4, a2, (s0)  -> a4=10,  mem=15
        put(18, 32'h00D4_A7AF);  // amoadd.w a5, a3, (s1)  -> a5=100, mem=107

        put(19, 32'h0004_A803);  // lw   a6, 0(s1)         -> 107

        // ---- JALR target alignment ----
        //
        // Shares this harness deliberately rather than duplicating 60 lines of
        // DUT wiring. RISC-V requires the JALR target to be (rs1+imm) & ~1.
        //
        // Note the observable: tcm.v:101/108 index by i_offset[..:2] and
        // i_offset[1] and never look at bit 0, so an unmasked odd target still
        // fetches the RIGHT instruction - it does not execute garbage. What
        // breaks is that every PC from then on stays odd, which corrupts mepc on
        // any later trap and any auipc/PC-relative result. So the check is PC
        // parity, not whether the marker ran.
        put(20, 32'h4000_02B7);  // lui  t0, 0x40000
        put(21, 32'h0692_8293);  // addi t0, t0, 0x69   -> 0x40000069 (ODD)
        put(22, 32'h0000_0913);  // addi s2, x0, 0      (clear marker)
        put(23, 32'h0002_8067);  // jalr x0, 0(t0)      -> must land at 0x...68
        put(24, 32'h0000_006F);  // j self  (skipped when the jump works)
        put(25, 32'h0000_0013);  // nop
        put(26, 32'h02A0_0913);  // addi s2, x0, 42     <- 0x40000068, the target
        put(27, 32'h0000_006F);  // j self

        repeat (3) @(posedge clk);
        rst = 0;

        // Each divide is ~34 cycles; 400 is ample for the whole program.
        repeat (400) @(posedge clk);

        // t4 is correct even with the bug - the FIRST divide always runs.
        check(DUT.RF.regs[29], 32'd14, "t4 = 100/7 (first divide)");

        // t4 == t5 is the signature of the bug: the second divide never started
        // and handed back the first one's quotient.
        check(DUT.RF.regs[30], 32'd6,  "t5 = 20/3  (SECOND divide, adjacent)");

        check(DUT.RF.regs[31], 32'd2,  "t6 = 100%7 (separated, control)");

        // a4 is correct even with the bug - the FIRST AMO always runs.
        check(DUT.RF.regs[14], 32'd10,  "a4 = old(addr1)   (first AMO)");

        // a4 == a5 is the signature: the second AMO never issued a read, so
        // final_mem_read_data handed back the first AMO's amo_old_value.
        check(DUT.RF.regs[15], 32'd100, "a5 = old(addr2)   (SECOND AMO)");

        // And the write half: with the bug the second AMO asserts neither
        // dcache_ren nor dcache_wen, so memory is never modified at all.
        check(DUT.RF.regs[16], 32'd107, "mem[addr2] = 100+7 (AMO wrote)");

        // Passes either way - proves the jump was taken and the marker ran, so a
        // PC-parity failure below cannot be mistaken for "the jump didn't work".
        check(DUT.RF.regs[18], 32'd42, "s2 = 42 (jalr reached its target)");

        // The actual spec check: bit 0 must have been masked out of the target.
        check({31'd0, dbg_raw_pc[0]}, 32'd0, "PC bit0 after odd-target jalr");

        $display("=== %0d failures ===", fails);
        if (fails == 0) $display("B2B TESTS PASSED");
        $finish;
    end
endmodule
