`include "../src/cpu.v"

module testbench;
    reg clk, rst;
    reg uart_tx_ready;
    wire uart_tx_start;
    wire [7:0] uart_tx_data;
    wire [3:0] leds;

    // CPU Memory Bus Signals
    wire [31:0] icache_mem_req_addr; 
    wire icache_mem_req_valid; 
    wire [127:0] icache_mem_read_data; 
    wire icache_mem_ready; 
    
    wire [31:0] dmem_req_addr; 
    wire [31:0] store_data; 
    wire [3:0] mem_write_mask; 
    wire dcache_mem_req_valid; 
    wire [127:0] dcache_mem_read_data_block; 
    wire dcache_mem_ready;

    // Instantiate Device Under Test (DUT)
    cpu_pipelined DUT (
        .clk(clk), .rst(rst), .uart_tx_ready(uart_tx_ready),
        .uart_tx_start(uart_tx_start), .uart_tx_data(uart_tx_data), .leds(leds),
        
        .icache_mem_req_addr(icache_mem_req_addr),
        .icache_mem_req_valid(icache_mem_req_valid),
        .icache_mem_read_data(icache_mem_read_data),
        .icache_mem_ready(icache_mem_ready),
        
        .dmem_req_addr(dmem_req_addr),
        .store_data(store_data),
        .mem_write_mask(mem_write_mask),
        .dcache_mem_req_valid(dcache_mem_req_valid),
        .dcache_mem_read_data_block(dcache_mem_read_data_block),
        .dcache_mem_ready(dcache_mem_ready)
    );

    reg [31:0] mock_imem [0:16383]; // 64KB Instruction Memory array
    reg [31:0] mock_dmem [0:4095];  // 16KB Data Memory array

    // Instruction Memory interface (serves 128-bit lines to I-Cache)
    assign icache_mem_ready = icache_mem_req_valid;
    assign icache_mem_read_data = {
        mock_imem[{icache_mem_req_addr[31:4], 2'b11}],
        mock_imem[{icache_mem_req_addr[31:4], 2'b10}],
        mock_imem[{icache_mem_req_addr[31:4], 2'b01}],
        mock_imem[{icache_mem_req_addr[31:4], 2'b00}]
    };

    // Data Memory Interface (serves 128-bit lines to D-Cache)
    assign dcache_mem_ready = dcache_mem_req_valid;
    assign dcache_mem_read_data_block = {
        mock_dmem[{dmem_req_addr[31:4], 2'b11}],
        mock_dmem[{dmem_req_addr[31:4], 2'b10}],
        mock_dmem[{dmem_req_addr[31:4], 2'b01}],
        mock_dmem[{dmem_req_addr[31:4], 2'b00}]
    };

    // Handle standard memory write requests straight to mock_dmem
    always @(posedge clk) begin
        if (dcache_mem_req_valid && mem_write_mask != 4'b0000) begin
            if (mem_write_mask[0]) mock_dmem[dmem_req_addr[31:2]][7:0]   <= store_data[7:0];
            if (mem_write_mask[1]) mock_dmem[dmem_req_addr[31:2]][15:8]  <= store_data[15:8];
            if (mem_write_mask[2]) mock_dmem[dmem_req_addr[31:2]][23:16] <= store_data[23:16];
            if (mem_write_mask[3]) mock_dmem[dmem_req_addr[31:2]][31:24] <= store_data[31:24];
        end
    end

    integer i;
    integer cycle;

    initial clk = 0;
    always #5 clk = ~clk;

    task check;
        input [4:0] reg_num;
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
        $readmemh("../Mems/test_alu_basics.mem", mock_imem);
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
        $readmemh("../Mems/test_ex_forwarding.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 2: EX Forwarding");
        check(2, 32'd20);
        check(3, 32'd30);
        check(4, 32'd50);

        // MEM Forwarding 
        reset_pipeline();
        $readmemh("../Mems/test_mem_ex_forwarding.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 3: MEM Forwarding");
        check(5, 32'd16);

        // Load-Use Stall 
        reset_pipeline();
        $readmemh("../Mems/test_load_use_stall.mem", mock_imem);
        reset_dut();
        mock_dmem[0] = 32'd42;
        repeat(50) @(posedge clk);

        // $display("DMEM[0] = %h", DUT.DMEM.ram[0]);
        // $display("x1=%0d x2=%0d x3=%0d", 
        // DUT.RF.regs[1], DUT.RF.regs[2], DUT.RF.regs[3]);

        $display("Test 4: Load-Use Stall");
        check(3, 32'd42);
        check(4, 32'd42);
        check(5, 32'd43);

        // Store then Load
        reset_pipeline();
        $readmemh("../Mems/test_store_load.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 5: Store then Load");
        check(4, 32'd100);
        check(5, 32'd200);

        // Branch Not Taken / Taken 
        reset_pipeline();
        $readmemh("../Mems/test_branch_taken.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 6: Branch");
        check(3, 32'd1);

        // Branch on Forwarded Values
        reset_pipeline();
        $readmemh("../Mems/test_branch_taken.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 7: Branch + Forwarding");
        check(3, 32'd1);

        // JAL / JALR 
        reset_pipeline();
        $readmemh("../Mems/test_jal_jalr.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 8: JAL/JALR");
        check(1, 32'd2);

        // Load-Use + Branch
        reset_pipeline();
        $readmemh("../Mems/test_load_use_stall_before_branch.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 9: Load-Use + Branch");
        check(4, 32'd1);

        // RVC
        reset_pipeline();
        $readmemh("../Mems/test_rvc_basics.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test10: RVC Compressed Instructions");
        check(1, 32'd5); // x1 should be 5
        check(2, 32'd2); // x2 should be 2
        check(3, 32'd7); // x3 should be 7

        // RVC Corner Cases
        reset_pipeline();
        $readmemh("../Mems/test_rvc_corner.mem", mock_imem);
        reset_dut();
        repeat(50) @(posedge clk);
        $display("Test 11: RVC Corner Cases (Hazards & Negatives)");
        check(1, 32'd5); // 10 - 5 = 5
        check(2, 32'd5); // 0 + 5 = 5
        check(3, 32'd15); // 5 + 10 = 15

        // RVC Loop
        reset_pipeline();
        $readmemh("../Mems/test_rvc_loop.mem", mock_imem);
        reset_dut();
        repeat(100) @(posedge clk); // Needs more time for loops!
        $display("Test 12: RVC Loop Accumulator");
        check(1, 32'd0); // Counter should reach 0
        check(2, 32'd15); // Sum should be 15

        // RVC Buffer
        reset_pipeline();
        $readmemh("../Mems/test_rvc_buffer.mem", mock_imem);
        reset_dut();
        repeat(100) @(posedge clk); 
        $display("Test 13: RVC Buffer");
        check(1, 32'd5);  
        // Check the compressed instruction immediately following it
        check(2, 32'd10); 
        // Check the subsequent 32-bit instruction
        check(3, 32'd15);        

        // Atomic Success
        reset_pipeline();
        $readmemh("../Mems/test_atomic_success.mem", mock_imem);
        reset_dut();
        repeat(100) @(posedge clk); 
        $display("Test 15: Atomic Success");     
        check(5, 32'd16); // x5 should be 16
        check(6, 32'd42); // x6 should be 42
        check(8, 32'd0); // x8 should be 0 because Store-Conditional succeeded
        check(10, 32'd42);


        // Atomic Fail
        reset_pipeline();
        $readmemh("../Mems/test_atomic_fail.mem", mock_imem);
        reset_dut();
        repeat(100) @(posedge clk); 
        $display("Test 16: Atomic Fail");     
        check(5, 32'd16); // x5 should be 16
        check(6, 32'd42); // x6 should be 42
        check(9, 32'd89); // x9 should be 89
        check(8, 32'd1); 
        check(10, 32'd89);    
        $finish;

        // CSR Read/Write (ALU Hijack)
        reset_pipeline();
        $readmemh("../Mems/test_csr_rw.mem", mock_imem);
        reset_dut();
        repeat(100) @(posedge clk); 
        $display("Test 17: CSR Read/Write");     
        // Check 1: Did the first csrrw read the default reset value of mtvec?
        check(6, 32'd0);   // x6 should be 0
        
        // Check 2: Did the second csrrw successfully read the 89 we wrote earlier?
        check(8, 32'd89);  // x8 should be 89
        
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
                mock_imem[k] = 32'h00000013; 
        end
    endtask

    task reset_dut;
        integer k;
        begin
            rst = 1;
            uart_tx_ready = 1;
            repeat(2) @(posedge clk);
            rst = 0;
            // Clear regfile to 0 between tests
            for (k = 0; k < 32; k = k + 1)
                DUT.RF.regs[k] = 32'b0;
            // Clear DMEM
            for (k = 0; k < 4096; k = k + 1)
                mock_dmem[k] = 32'b0;
        end
    endtask

endmodule