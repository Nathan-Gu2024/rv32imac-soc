`timescale 1ns/1ps

module cpu_tb;
    reg clk;
    reg rst;

    // Instantiate your Pipelined CPU
    // Assuming your CPU has internal register monitoring or exposes top-level ports
    cpu_pipelined uut (
        .clk(clk),
        .rst(rst)
    );

    // Clock Generation (50MHz)
    always #10 clk = ~clk;

    initial begin
        $dumpfile("cpu_pipeline_results.vcd");
        $dumpvars(0, cpu_tb);

        // Initialize signals
        clk = 0;
        rst = 1;
        
        // Load your assembled program into the CPU's IMEM
        // Make sure your imem implementation matches this path
        $readmemh("hazard_test.hex", uut.IMEM.mem); 

        // Hold reset for 2 cycles
        #20;
        rst = 0;

        // Run the simulation for enough time to complete the test program
        #500;

        // --- Self-Checking Assertions ---
        // Modify these paths to point directly to your regfile array variables
        $display("=== SIMULATION COMPLETE: EVALUATING ARCHITECTURAL STATE ===");
        
        if (uut.REGFILE.registers[2] == 32'd20) 
            $display("[PASS] Test 1A: EX-to-EX Forwarding Correct.");
        else 
            $display("[FAIL] Test 1A: Expected x2=20, got %d", uut.REGFILE.registers[2]);

        if (uut.REGFILE.registers[2] == 32'd84)
            $display("[PASS] Test 2: Load-Use Hazard Stall Resolved Correctly.");
        else
            $display("[FAIL] Test 2: Expected x2=84, got %d", uut.REGFILE.registers[2]);

        if (uut.REGFILE.registers[3] == 32'd0)
            $display("[PASS] Test 3: Branch Shadow Instruction successfully flushed.");
        else
            $display("[FAIL] Test 3: Flushes failed! Shadow instruction executed and wrote x3=%d", uut.REGFILE.registers[3]);

        $finish;
    end
endmodule