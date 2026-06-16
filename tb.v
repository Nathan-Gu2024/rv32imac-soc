`include "cpu.v"

module testbench;
    reg clk, rst;
    cpu_pipelined DUT (.clk(clk), .rst(rst));

    integer i;
    integer cycle;

    initial clk = 0;
    always #5 clk = ~clk;

    task check;
        input [4:0]  reg_num;
        input [31:0] expected;
        begin
            if (DUT.RF.regs[reg_num] === expected)
                $display("PASS: x%0d = %0d", reg_num, expected);
            else
                $display("FAIL: x%0d = %0d, expected %0d",
                          reg_num, DUT.RF.regs[reg_num], expected);
        end
    endtask

    // Main Test Sequence
    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, testbench);

        // Basic ALU 
        reset_pipeline();
        $readmemh("Mems/test_alu_basics.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 1: Basic ALU");
        $monitor("Time: %0t | PC: %0d | Stall: %b | Cache State: %b | Hit: %b", 
          $time, DUT.PC.pc, DUT.global_mem_stall, DUT.ICACHE.state, DUT.ICACHE.is_hit);
        check(3, 32'd8); // add
        check(4, 32'd2); // sub
        check(5, 32'd1); // and
        check(6, 32'd7); // or
        check(7, 32'd6); // xor

        // EX Forwarding
        reset_pipeline();
        $readmemh("Mems/test_ex_forwarding.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 2: EX Forwarding");
        check(2, 32'd20);
        check(3, 32'd30);
        check(4, 32'd50);

        // MEM Forwarding 
        reset_pipeline();
        $readmemh("Mems/test_mem_ex_forwarding.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 3: MEM Forwarding");
        check(5, 32'd16);

        // Load-Use Stall 
        reset_pipeline();
        $readmemh("Mems/test_load_use_stall.mem", DUT.IMEM.rom);
        reset_dut();
        DUT.DMEM.ram[0] = 32'd42;
        repeat(50) @(posedge clk);

        $display("DMEM[0] = %h", DUT.DMEM.ram[0]);
        $display("x1=%0d x2=%0d x3=%0d", 
          DUT.RF.regs[1], DUT.RF.regs[2], DUT.RF.regs[3]);

        $display("Test 4: Load-Use Stall");
        check(3, 32'd42);
        check(4, 32'd42);
        check(5, 32'd43);

        // Store then Load
        reset_pipeline();
        $readmemh("Mems/test_store_load.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 5: Store then Load");
        check(4, 32'd100);
        check(5, 32'd200);

        // Branch Not Taken / Taken 
        reset_pipeline();
        $readmemh("Mems/test_branch_taken.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 6: Branch");
        check(3, 32'd1);

        // Branch on Forwarded Values
        reset_pipeline();
        $readmemh("Mems/test_branch_taken.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 7: Branch + Forwarding");
        check(3, 32'd1);

        // JAL / JALR 
        reset_pipeline();
        $readmemh("Mems/test_jal_jalr.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 8: JAL/JALR");
        check(1, 32'd2);

        // Load-Use + Branch
        reset_pipeline();
        $readmemh("Mems/test_load_use_stall_before_branch.mem", DUT.IMEM.rom);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("--- Test 9: Load-Use + Branch");
        check(4, 32'd1);

        $finish;
    end

    // Global simulation watchdog timeout
    initial begin
        #10000;
        $display("TIMEOUT");
        $finish;
    end

    task reset_pipeline;
        integer k; 
        begin
            // Clear IMEM so stale instructions don't execute
            for (k = 0; k < 16384; k = k + 1)
                DUT.IMEM.rom[k] = 32'h00000013; 
        end
    endtask

    task reset_dut;
        integer k;
        begin
            rst = 1;
            repeat(2) @(posedge clk);
            rst = 0;
            // Clear regfile to 0 between tests
            for (k = 0; k < 32; k = k + 1)
                DUT.RF.regs[k] = 32'b0;
            // Clear DMEM
            for (k = 0; k < 1024; k = k + 1)
                DUT.DMEM.ram[k] = 32'b0;
        end
    endtask
endmodule