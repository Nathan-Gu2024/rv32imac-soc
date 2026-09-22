// Load-use interlock precision: does a stall happen only when a real register
// dependence exists?
//
// The interlock compares if_id_inst[19:15] and [24:20] against the load's rd
// with no qualification by whether the consumer actually READS those fields. In
// I-type, U-type and J-type instructions those bit positions are immediate
// payload, so a numeric coincidence between an immediate and a register number
// produces a stall for a dependence that does not exist.
//
// This is a PERFORMANCE bug, not a correctness one: the machine computes the
// right answer either way, just a cycle later. So the check has to count stall
// cycles rather than compare register values - a value-checking bench cannot see
// it at all, which is why nothing caught it.
//
// Method: run two loops that are identical in instruction count, addressing and
// data, and differ ONLY in whether the follower's immediate happens to equal the
// load's rd. Any cycle difference between them is false stalls.

`include "../src/cpu.v"

module tb_cpu_interlock;
    reg clk = 0;
    reg rst;
    reg uart_rx_i = 1'b1;

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
    localparam [31:0] NOP = 32'h0000_0013;

    integer i;
    integer stalls;
    integer fails = 0;

    // Count cycles in which the load-use interlock is asserted. DUT.stall is the
    // interlock specifically, not global_mem_stall.
    always @(posedge clk) begin
        if (!rst && DUT.stall) stalls = stalls + 1;
    end

    task put;
        input integer w;
        input [31:0] inst;
        begin
            DUT.TCM.mem_even[w] = inst[15:0];
            DUT.TCM.mem_odd[w]  = inst[31:16];
        end
    endtask

    // Load into x10, then an arbitrary follower instruction, eight times.
    task run_seq;
        input [31:0] follow;
        input [8*40:1] label;
        integer guard;
        begin
            rst = 1;
            @(posedge clk);
            for (i = 0; i < TCM_WORDS; i = i + 1) begin
                DUT.TCM.mem_even[i] = 16'h0013;
                DUT.TCM.mem_odd[i]  = 16'h0000;
            end

            // sp-relative base that is safely inside TCM and 4-byte aligned.
            put(0, 32'h4000_0537);            // lui  a0, 0x40000
            put(1, 32'h4005_0513);            // addi a0, a0, 1024  -> 0x40000400

            // Eight copies of: lw -> x10 from (a0), then the follower.
            for (i = 0; i < 8; i = i + 1) begin
                // lw x10, 0(a0)   : imm=0 rs1=10 funct3=010 rd=10 op=0000011
                put(2 + i * 2, 32'h0005_2503);
                put(3 + i * 2, follow);
            end
            put(18, 32'h0000_006F);           // j self

            repeat (3) @(posedge clk);
            stalls = 0;
            rst = 0;

            guard = 0;
            while (dbg_raw_pc != 32'h4000_0048 && guard < 300) begin
                @(posedge clk);
                guard = guard + 1;
            end
            repeat (4) @(posedge clk);

            $display("  %0s: %0d interlock stall cycles", label, stalls);
        end
    endtask

    integer stalls_collide, stalls_clear, stalls_dep_rs1, stalls_dep_rs2;

    initial begin
        $display("=== load-use interlock precision ===");

        // --- Part 1: false stalls must be gone ---
        //
        // addi a1, a2, 10 - an I-type. [24:20] holds imm=10, which is NOT a
        // register read, but numerically equals the load's rd (x10).
        run_seq(32'h00A6_0593, "addi a1,a2,10  (imm collides) ");
        stalls_collide = stalls;

        // addi a1, a2, 3 - identical in every way except the immediate.
        run_seq(32'h0036_0593, "addi a1,a2,3   (no collision) ");
        stalls_clear = stalls;

        // --- Part 2: REAL dependences must still stall ---
        //
        // Without these, this bench would pass just as happily against an
        // interlock that had been disabled outright, which is the failure mode
        // that matters most when relaxing a hazard check.
        //
        // addi a1, x10, 3 - rs1 IS the load's rd. A genuine load-use hazard.
        run_seq(32'h0035_0593, "addi a1,x10,3  (REAL rs1 dep) ");
        stalls_dep_rs1 = stalls;

        // add a1, a2, x10 - R-type, so [24:20] really is rs2, and it is the
        // load's rd. A genuine hazard through the other field.
        run_seq(32'h00A6_05B3, "add  a1,a2,x10 (REAL rs2 dep) ");
        stalls_dep_rs2 = stalls;

        $display("");
        if (stalls_collide == stalls_clear) begin
            $display("  PASS  no false stalls: %0d == %0d",
                     stalls_collide, stalls_clear);
        end else begin
            $display("  FAIL  %0d false stall cycles - an immediate that happens",
                     stalls_collide - stalls_clear);
            $display("        to equal the load's rd is treated as a dependence");
            fails = fails + 1;
        end

        if (stalls_dep_rs1 > 0) begin
            $display("  PASS  real rs1 dependence still stalls (%0d cycles)",
                     stalls_dep_rs1);
        end else begin
            $display("  FAIL  real rs1 dependence did NOT stall - hazard");
            $display("        detection is broken, not merely refined");
            fails = fails + 1;
        end

        if (stalls_dep_rs2 > 0) begin
            $display("  PASS  real rs2 dependence still stalls (%0d cycles)",
                     stalls_dep_rs2);
        end else begin
            $display("  FAIL  real rs2 dependence did NOT stall - the rs2_used");
            $display("        qualifier is excluding R-type by mistake");
            fails = fails + 1;
        end

        $display("=== %0d failures ===", fails);
        if (fails == 0) $display("INTERLOCK TESTS PASSED");
        $finish;
    end
endmodule
