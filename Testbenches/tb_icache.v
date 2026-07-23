`timescale 1ns/1ps

`include "../src/cache_core.v"
`include "../src/tcm.v"
`include "../src/icache.v"
`include "../Testbenches/fake_line_memory.v"

module tb_icache();
    // System Signals
    reg clk, rst;
    
    // CPU Signals
    reg cpu_req_valid;
    reg [31:0] cpu_req_addr;
    wire [31:0] cpu_rdata;
    wire cpu_ready;
    
    // TCM Signals
    wire tcm_req_valid, tcm_ready;
    wire [31:0] tcm_req_addr, tcm_rdata;

    // Mem Signals
    wire mem_req_valid, mem_ready; 
    wire [127:0] mem_rline; 
    wire [31:0] mem_req_addr; 
    wire [31:0] mem_read_count; 

    integer errors;
    reg [31:0] tmp, before_reads;

    // Instantiate your icache
    icache uut (
        .clk(clk), 
        .rst(rst),
        .cpu_req_valid(cpu_req_valid), 
        .cpu_req_addr(cpu_req_addr), 
        .cpu_rdata(cpu_rdata), 
        .cpu_ready(cpu_ready),
        .tcm_req_valid(tcm_req_valid), 
        .tcm_req_addr(tcm_req_addr), 
        .tcm_rdata(tcm_rdata), 
        .tcm_ready(tcm_ready),
        .mem_req_valid(mem_req_valid), 
        .mem_req_addr(mem_req_addr), 
        .mem_rline(mem_rline), 
        .mem_ready(mem_ready)
    );

    // Reuse TCM - Connect only to Port A (Instruction Port)
    tcm #(
        .ADDR_WIDTH(32), 
        .TCM_BASE(32'h4000_0000), 
        .TCM_BYTES(65536)
    ) tcm0 (
        .clk(clk),
        .rst(rst), 
        .i_req(tcm_req_valid), 
        .i_addr(tcm_req_addr), 
        .i_rdata(tcm_rdata), 
        .i_ready(tcm_ready), 
        // Tie off Data Port (Not used here)
        .d_req(1'b0), 
        .d_we(1'b0), 
        .d_addr(32'h0), 
        .d_wdata(32'h0), 
        .d_wmask(4'h0), 
        .d_rdata(), 
        .d_ready()
    );

    // Reuse Fake Line Memory
    fake_line_memory #(
        .LATENCY(2)
    ) line_mem0 (
        .clk(clk), 
        .rst(rst), 
        .mem_req_valid(mem_req_valid), 
        .mem_req_write(1'b0), 
        .mem_req_addr(mem_req_addr), 
        .mem_wline(128'h0), 
        .mem_rline(mem_rline), 
        .mem_ready(mem_ready), 
        .read_count(mem_read_count), 
        .write_count(), 
        .last_write_addr(), 
        .last_write_line()
    );

    always #5 clk = ~clk;

    task check32;
        input [31:0] got; 
        input [31:0] expected;
        input [639:0] name;
        begin
            if (got !== expected) begin
                $display("FAIL: %0s got=%h expected=%h", name, got, expected);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s = %h", name, got); 
            end 
        end 
    endtask

    task cpu_fetch; 
        input [31:0] addr; 
        output [31:0] rdata; 
        integer cycles; 
        begin
            @(negedge clk);
            cpu_req_valid = 1'b1;
            cpu_req_addr = addr; 

            cycles = 0; 
            @(posedge clk); #1;
            while (cpu_ready !== 1'b1 && cycles < 100) begin
                cycles = cycles + 1;
                @(posedge clk); #1;
            end 

            if (cpu_ready !== 1'b1) begin
                $display("FAIL: timeout waiting for cpu_ready addr=%h", addr);
                errors = errors + 1;
                rdata = 32'hXXXX_XXXX;
            end else begin
                rdata = cpu_rdata; 
            end 

            @(negedge clk);
            cpu_req_valid = 1'b0;
            cpu_req_addr = 32'h0;
            @(negedge clk);
        end 
    endtask

    initial begin
        $dumpfile("tb_icache.vcd");
        $dumpvars(0, tb_icache);
        
        clk = 1'b0;
        rst = 1'b1;
        errors = 0;
        cpu_req_valid = 1'b0;
        cpu_req_addr = 32'h0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        $display("\nTest 1: TCM IF"); 

        before_reads = mem_read_count; 
        cpu_fetch(32'h4000_0100, tmp); 
        check32(tmp, 32'h0000_0000, "TCM fetch returned 0x0"); 
        check32(mem_read_count, before_reads, "TCM fetch bypassed main memory"); 

        $display("\nTest 2: I-Cache main mem miss");
        before_reads = mem_read_count; 
        cpu_fetch(32'h8000_0008, tmp); 
        check32(tmp, 32'h8000_0008, "DDR fetch returned correct offset word");
        check32(mem_read_count, before_reads + 1, "DDR fetch caused exactly one line read"); 

        $display("\nTest 3: I-Cache main mem hit");
        before_reads = mem_read_count; 

        cpu_fetch(32'h8000_000C, tmp); 
        check32(tmp, 32'h8000_000C, "Cache hit returned next offset word"); 
        check32(mem_read_count, before_reads, "Cache hit did NOT cause a new line read");

        $display("\nSummary");
        if (errors == 0) begin
            $display("ALL TESTS PASSED");
        end else begin
            $display("TESTS FAILED: %0d errors", errors); 
        end 

        #20; 
        $finish; 
    end 

endmodule