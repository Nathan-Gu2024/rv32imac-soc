`timescale 1ns/1ps

`include "../src/dcache_bram.v"
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
        .cpu_req_addr_next(cpu_req_addr[13:0]), 
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
    // tcm #(
    //     .ADDR_WIDTH(32), 
    //     .TCM_BASE(32'h4000_0000), 
    //     .TCM_BYTES(65536)
    // ) tcm0 (
    //     .clk(clk),

    //     .i_req(1'b0), 
    //     .i_addr(32'h0), 
    //     .i_rdata(), 
    //     .i_ready(), 

    //     .d_req(tcm_req_valid), 
    //     .d_we(tcm_req_write), 
    //     .d_addr(tcm_req_addr), 
    //     .d_wdata(tcm_wdata), 
    //     .d_wmask(tcm_wmask), 
    //     .d_rdata(tcm_rdata), 
    //     .d_ready(tcm_ready)
    // );

    tcm #(
        .ADDR_WIDTH(32), 
        .TCM_BASE(32'h4000_0000),
        .TCM_BYTES(65536)
    ) TCM (
        .clk(clk),
        .rst(rst),

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
            @(posedge clk);
            #1;
            while (cpu_ready !== 1'b1 && cycles < 4000) begin
                cycles = cycles + 1;
                @(posedge clk);
                #1;
            end
            if (cycles >= 4000) begin
                $display("FAIL: timeout waiting for cpu_ready at addr=%h", addr);
                $finish;
            end
            rdata = cpu_rdata;
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

        // 1. Force a miss to fetch the 16-byte line containing 8000_0000 -> 8000_000F
        cpu_access(1'b0, 32'h8000_0008, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'h8000_0008, "DDR read miss returned correct word");
        check32(mem_read_count, before_reads + 1, "DDR read miss caused one line read"); 

        before_reads = mem_read_count; 
        
        // 2. Read hit on the original word (Offset 2)
        cpu_access(1'b0, 32'h8000_0008, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'h8000_0008, "DDR read hit (word 1) returned correct word"); 
        
        // 3. Read hit on the adjacent word (Offset 3) - Brought over from Block 2!
        cpu_access(1'b0, 32'h8000_000C, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'h8000_000C, "DDR read hit (word 2) returned correct word"); 
        
        // Verify neither hit caused a new memory request
        check32(mem_read_count, before_reads, "DDR read hits caused no new line read");

        $display("\n Test 3: write hit updates cache, no immediate wb"); 
        before_writes = mem_write_count; 

        cpu_access(1'b1, 32'h8000_0008, 32'hDEAD_BEEF, 4'b1111, tmp); 
        check32(mem_write_count, before_writes, "write hit caused no immediate DDR write");

        cpu_access(1'b0, 32'h8000_0008, 32'h0000_0000, 4'b0000, tmp); 
        check32(tmp, 32'hDEAD_BEEF, "read after write hit"); 

        $display("\n Test 4: dirty eviction wb old line");
        // The cache is now 16KB DIRECT-MAPPED (1024 sets x 16B), so the set
        // index is addr[13:4] and the conflict stride is 0x4000. The old
        // sequence here used +0x400/+0x800, which collided only in the
        // previous 64-set 2-way geometry and now lands in three different
        // sets - producing no eviction at all. Direct-mapped also needs no
        // LRU priming: one conflicting access evicts outright.
        //
        // Test 3 above left line 0x8000_0000 resident and dirty (it wrote
        // 0xDEADBEEF to word 2), so a single conflicting access must write
        // that whole line back.
        before_writes = mem_write_count;
        cpu_access(1'b0, 32'h8000_4000, 32'h0000_0000, 4'b0000, tmp); // same set, new tag -> evict

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


endmodule