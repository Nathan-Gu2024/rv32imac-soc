`timescale 1ns/1ps

`include "../src/cache_core.v"
`include "../src/tcm.v"
`include "../src/dcache.v"
`include "../Testbenches/fake_line_memory.v"

module tb_dcache();
    // System Signals
    reg clk, rst;
    
    // CPU Signals
    reg cpu_req_valid, cpu_req_write, cpu_ready;
    reg [31:0] cpu_req_addr, cpu_wdata, cpu_rdata;
    reg [3:0] cpu_wmask;
    
    wire tcm_req_valid, tcm_req_write, tcm_ready;
    wire [31:0] tcm_req_addr, tcm_wdata, tcm_rdata;
    wire [3:0] tcm_wmask;

    wire mem_req_valid, mem_req_write, mem_ready; 
    wire [127:0] mem_wline, mem_rline; 
    wire [31:0] mem_req_addr; 

    wire [31:0] mem_read_count, mem_write_count, last_write_addr; 
    wire [127:0] last_write_line;

    integer errors;
    reg [31:0] tmp, before_reads, before_writes;



    // Instantiate your fixed dcache
    dcache uut (
        .clk(clk), 
        .rst(rst),

        .cpu_req_valid(cpu_req_valid), 
        .cpu_req_write(cpu_req_write),
        .cpu_req_addr(cpu_req_addr), 
        .cpu_wdata(cpu_wdata), 
        .cpu_wmask(cpu_wmask),
        .cpu_rdata(cpu_rdata), 
        .cpu_ready(cpu_ready),

        .tcm_req_valid(tcm_req_valid), 
        .tcm_req_write(tcm_req_write),
        .tcm_req_addr(tcm_req_addr), 
        .tcm_wdata(tcm_wdata), 
        .tcm_wmask(tcm_wmask),
        .tcm_rdata(tcm_rdata), 
        .tcm_ready(tcm_ready),

        .mem_req_valid(mem_req_valid), 
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr), 
        .mem_wline(mem_wline),
        .mem_rline(mem_rline), 
        .mem_ready(mem_ready)
    );
        tcm #(
            .ADDR_WIDTH(32), 
            .TCM_BASE(32'h4000_0000), 
            .TCM_BYTES(65536)
        ) tcm0 (
            .clk(clk),

            .i_req(1'b0), 
            .i_addr(32'h0), 
            .i_rdata(), 
            .i_ready(), 

            .d_req(tcm_req_valid), 
            .d_we(tcm_req_write), 
            .d_addr(tcm_req_addr), 
            .d_wdata(tcm_wdata), 
            .d_wmask(tcm_wmask), 
            .d_rdata(tcm_rdata), 
            .d_ready(tcm_ready)
        );

        fake_line_memory #(
            .LATENCY(2)
        ) line_mem0 (
            .clk(clk), 
            .rst(rst), 
            .mem_req_valid(mem_req_valid), 
            .mem_req_write(mem_req_write), 
            .mem_req_addr(mem_req_addr), 
            .mem_wline(mem_wline), 
            .mem_rline(mem_rline), 
            .mem_ready(mem_ready), 
            .read_count(mem_read_count), 
            .write_count(mem_write_count), 
            .last_write_addr(last_write_addr), 
            .last_write_line(last_write_line)
        );

    // Clock Generation
    always #5 clk = ~clk;

    task check32;
        input [31:0] got; 
        input [31:0] expected;
        input [639 : 0] name;
        begin
            if (got != expected) begin
                $display("FAIL: %0s got=%h expected=%h", name, got, expected);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s = %h", name, got); 
            end 
        end 
    endtask

    task check128;
        input [127:0] got; 
        input [127:0] expected;
        input [639 : 0] name;
        begin
            if (got != expected) begin
                $display("FAIL: %0s got=%h expected=%h", name, got, expected);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s = %h", name); 
            end 
        end 
    endtask

    task cpu_access; 
        input write; 
        input [31:0] addr; 
        input [31:0] wdata; 
        input [3:0] wmask; 
        output [31:0] rdata; 
        integer cycles; 
        begin
            @(negedge clk);
            cpu_req_valid = 1'b1;
            cpu_req_write = write; 
            cpu_req_addr = addr; 
            cpu_wdata = wdata; 
            cpu_wmask = wmask; 

            cycles = 0; 
            @(posedge clk); #1;
            while (cpu_ready != 1'b1 && cycles < 100) begin
                cycles = cycles + 1;
                @(posedge clk); #1;
            end 

            if (cpu_ready != 1'b1) begin
                $display("FAIL: timeout waiting for cpu_Ready addr=$h write=%b", addr, write);
                errors = errors + 1;
                rdata = 32'hXXXX_XXXX;
            end else begin
                rdata = cpu_rdata; 
            end 
            @(negedge clk);
            cpu_req_valid = 1'b0;
            cpu_req_write = 1'b0;
            cpu_req_addr = 32'h0;
            cpu_wdata = 32'h0;
            cpu_wmask = 4'h0;
            @(negedge clk);
        end 
    endtask

    initial begin
        $dumpfile("tb_dcache_with_tcm.vcd");
        $dumpvars(0, tb_dcache);
        
        clk = 1'b0;
        rst = 1'b1;
        errors = 0;

        cpu_req_valid = 1'b0;
        cpu_req_write = 1'b0; 
        cpu_req_addr = 32'h0;
        cpu_wdata = 32'h0;
        cpu_wmask = 4'h0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        $display("\n Test 1: TCM bypass write / read"); 
        before_reads = mem_read_count; 
        before_writes = mem_write_count; 

        cpu_access(1'b1, 32'h4000_0004, 32'hAABB_CCDD, 4'b1111, tmp); 
        cpu_access(1'b0, 32'h4000_0004, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'hAABB_CCDD, "TCM readback"); 
        check32(mem_read_count, before_reads, "TCM did not cause DDR/cache-line reads"); 
        check32(mem_write_count, before_writes, "TCM did not cause DDR/cache-line writes"); 

        $display("\n Test 2: cached DDR read miss then read hit");
        before_reads = mem_read_count; 

        cpu_access(1'b0, 32'h8000_0008, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'h8000_0008, "DDR read miss returned correct word");
        check32(mem_read_count, before_reads + 1, "DDR read miss caused one line read"); 

        before_reads = mem_read_count; 
        cpu_access(1'b0, 32'h8000_0008, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'h8000_0008m "DDR read hit returned correct word"); 
        check32(mem_read_count, before_reads, "DDR read hit caused no new line read");

        $display("\n Test 3: write hit updates cache, no immediate wb"); 
        before_writes = mem_write_count; 

        cpu_access(1'b1, 32'h8000_0008, 32'hDEAD_BEEF, 4'b1111, tmp); 
        check32(mem_write_count, before_writes, "write hit caused no immediate DDR write");

        cpu_access(1'b0, 32'h8000_0008, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'hDEAD_BEEF, "read after write hit"); 

        $display("\n Test 4: dirty eviction wb old line");
        cpu_access(1'b0, 32'h8000_0400, 32'h0000_0000, 4'b0000, tmp); // fill B
        cpu_access(1'b0, 32'h8000_0000, 32'h0000_0000, 4'b0000, tmp); // touch A
        cpu_access(1'b0, 32'h8000_0400, 32'h0000_0000, 4'b0000, tmp); // touch B, A become LRU
        
        before_writes = mem_write_count;
        cpu_access(1'b0, 32'h8000_0800, 32'h0000_0000, 4'b0000, tmp); // fill C, evict A

        check32(mem_write_count, before_writes + 1, "dirty eviction caused one line wb"); 
        check32(last_write_addr, 32'h8000_0000, "dirty eviction wb address"); 
        check128(last_write_line, {32'h8000_000C, 32'hDEAD_BEEF, 32'h8000_0004, 32'h8000_0000}, 
                                    "dirty eviction wb line contents"); 

        $display("\n Summary");
        if (errors == 0) begin
            $display("ALL TESTS PASSED");
        end else begin
            $display("TESTS FAILED: %0d errors", errors); 
        end 

        #20; 
        $finish; 
    end 


    // initial begin
    //     // Initialize
    //     clk = 0; rst = 1;
    //     cpu_req_valid = 0; cpu_req_write = 0;
    //     cpu_req_addr = 0; cpu_wdata = 0; cpu_wmask = 0;
    //     tcm_rdata = 32'hDEADBEEF; tcm_ready = 1; // TCM always ready
    //     mem_rline = 128'h0; mem_ready = 0;
        
    //     #20 rst = 0;

    //     // TEST 1: TCM Bypass (Should assert tcm_req_valid, keep mem_req_valid low)
    //     #10;
    //     $display("--- Test 1: TCM Write ---");
    //     cpu_req_valid = 1; cpu_req_write = 1;
    //     cpu_req_addr = 32'h4000_0004; // Inside TCM range
    //     cpu_wdata = 32'hAABBCCDD;
    //     cpu_wmask = 4'b1111;
    //     #10;
    //     if (tcm_req_valid && !uut.cache_req_valid) $display("PASS: Routed to TCM.");
    //     else $display("FAIL: Did not route to TCM correctly.");
    //     cpu_req_valid = 0;

    //     // TEST 2: Cache Request (Should assert cache_req_valid, keep tcm_req_valid low)
    //     #20;
    //     $display("--- Test 2: Normal Memory Write ---");
    //     cpu_req_valid = 1; cpu_req_write = 1;
    //     cpu_req_addr = 32'h8000_0008; // Outside TCM range
    //     cpu_wdata = 32'h11223344;
    //     #10;
    //     if (!tcm_req_valid && uut.cache_req_valid) $display("PASS: Routed to Cache.");
    //     else $display("FAIL: Did not route to Cache correctly.");
    //     cpu_req_valid = 0;

    //     #50 $finish;
    // end
endmodule