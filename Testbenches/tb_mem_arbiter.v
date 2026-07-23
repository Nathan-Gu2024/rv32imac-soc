`timescale 1ns/1ps
`include "../src/mem_arbiter.v"

module tb_mem_arbiter;

    reg clk;
    reg rst;

    // icache 
    reg icache_req_valid;
    reg [31:0] icache_req_addr;
    wire icache_ready;
    wire [127:0] icache_rline;

    // dcache 
    reg dcache_req_valid;
    reg dcache_req_write;
    reg [31:0] dcache_req_addr;
    reg [127:0] dcache_wline;
    wire dcache_ready;
    wire [127:0] dcache_rline;

    // DDR / fake line-memory 
    wire mem_req_valid;
    wire mem_req_write;
    wire [31:0] mem_req_addr;
    wire [127:0] mem_wline;
    reg mem_ready;
    reg [127:0] mem_rline;

    integer errors;

    mem_arbiter uut (
        .clk(clk),
        .rst(rst),

        .icache_req_valid(icache_req_valid),
        .icache_req_addr(icache_req_addr),
        .icache_ready(icache_ready),
        .icache_rline(icache_rline),

        .dcache_req_valid(dcache_req_valid),
        .dcache_req_write(dcache_req_write),
        .dcache_req_addr(dcache_req_addr),
        .dcache_wline(dcache_wline),
        .dcache_ready(dcache_ready),
        .dcache_rline(dcache_rline),

        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline),
        .mem_ready(mem_ready),
        .mem_rline(mem_rline)
    );

    always #5 clk = ~clk;

    task check;
        input condition;
        input [8*80-1:0] msg;
        begin
            if (condition) begin
                $display("PASS: %0s", msg);
            end else begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    task reset_dut;
        begin
            clk = 1'b0;
            rst = 1'b1;

            icache_req_valid = 1'b0;
            icache_req_addr = 32'h0;

            dcache_req_valid = 1'b0;
            dcache_req_write = 1'b0;
            dcache_req_addr = 32'h0;
            dcache_wline = 128'h0;

            mem_ready = 1'b0;
            mem_rline = 128'h0;
            errors = 0;

            repeat (3) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task expect_mem_read;
        input [31:0] exp_addr;
        input [8*80-1:0] msg;
        begin
            check(mem_req_valid && !mem_req_write && mem_req_addr == exp_addr, msg);
        end
    endtask

    task expect_mem_write;
        input [31:0] exp_addr;
        input [127:0] exp_wline;
        input [8*80-1:0] msg;
        begin
            check(mem_req_valid && mem_req_write &&
                  mem_req_addr == exp_addr &&
                  mem_wline == exp_wline, msg);
        end
    endtask

    // Memory completion must be checked while mem_ready is high.
    // In this arbiter, icache_ready/dcache_ready are same-cycle pulses:
    //     *_ready = active_owner && mem_ready
    // After the next clock edge, state returns to IDLE and ready drops.
    task complete_i_read;
        input [127:0] line;
        begin
            @(negedge clk);
            mem_rline = line;
            mem_ready = 1'b1;
            #1;
            check(icache_ready && icache_rline == line,
                  "I-cache receives ready/data during mem_ready cycle");

            @(posedge clk);
            #1;
            mem_ready = 1'b0;
            mem_rline = 128'h0;
        end
    endtask

    task complete_d_read;
        input [127:0] line;
        begin
            @(negedge clk);
            mem_rline = line;
            mem_ready = 1'b1;
            #1;
            check(dcache_ready && dcache_rline == line,
                  "D-cache receives ready/data during mem_ready cycle");

            @(posedge clk);
            #1;
            mem_ready = 1'b0;
            mem_rline = 128'h0;
        end
    endtask

    task complete_d_write;
        begin
            @(negedge clk);
            mem_ready = 1'b1;
            #1;
            check(dcache_ready,
                  "D-cache receives write completion during mem_ready cycle");

            @(posedge clk);
            #1;
            mem_ready = 1'b0;
        end
    endtask

    initial begin
        reset_dut();

        // Test 1: I-cache-only read
        $display("--- Test 1: I-Cache Read ---");

        @(negedge clk);
        icache_req_valid = 1'b1;
        icache_req_addr = 32'h1000_0000;

        @(posedge clk);
        #1;
        expect_mem_read(32'h1000_0000,
                        "Memory sees I-cache read request");

        complete_i_read(128'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_1111_2222);

        @(negedge clk);
        icache_req_valid = 1'b0;
        icache_req_addr = 32'h0;

        @(posedge clk);
        #1;

        // Test 2: D-cache read
        $display("--- Test 2: D-Cache Read ---");

        @(negedge clk);
        dcache_req_valid = 1'b1;
        dcache_req_write = 1'b0;
        dcache_req_addr = 32'h2000_0000;

        @(posedge clk);
        #1;
        expect_mem_read(32'h2000_0000,
                        "Memory sees D-cache read request");

        complete_d_read(128'h1111_2222_3333_4444_5555_6666_7777_8888);

        @(negedge clk);
        dcache_req_valid = 1'b0;
        dcache_req_write = 1'b0;
        dcache_req_addr = 32'h0;

        @(posedge clk);
        #1;

        // Test 3: D-cache dirty writeback
        $display("--- Test 3: D-Cache Dirty Writeback ---");

        @(negedge clk);
        dcache_req_valid = 1'b1;
        dcache_req_write = 1'b1;
        dcache_req_addr = 32'h3000_0000;
        dcache_wline = 128'hDEAD_BEEF_DEAD_BEEF_DEAD_BEEF_DEAD_BEEF;

        @(posedge clk);
        #1;
        expect_mem_write(32'h3000_0000,
                         128'hDEAD_BEEF_DEAD_BEEF_DEAD_BEEF_DEAD_BEEF,
                         "Memory sees D-cache write request and correct wline");

        complete_d_write();

        @(negedge clk);
        dcache_req_valid = 1'b0;
        dcache_req_write = 1'b0;
        dcache_req_addr = 32'h0;
        dcache_wline = 128'h0;

        @(posedge clk);
        #1;

        // Test 4: Simultaneous requests; D-cache priority
        $display("--- Test 4: Simultaneous Requests (D-Cache Priority) ---");

        @(negedge clk);
        icache_req_valid = 1'b1;
        icache_req_addr = 32'h4000_0000;

        dcache_req_valid = 1'b1;
        dcache_req_write = 1'b1;
        dcache_req_addr = 32'h5000_0000;
        dcache_wline = 128'hCAFE_BAFE_CAFE_BAFE_CAFE_BAFE_CAFE_BAFE;

        @(posedge clk);
        #1;
        expect_mem_write(32'h5000_0000,
                         128'hCAFE_BAFE_CAFE_BAFE_CAFE_BAFE_CAFE_BAFE,
                         "D-cache was served first");

        complete_d_write();

        @(negedge clk);
        dcache_req_valid = 1'b0;
        dcache_req_write = 1'b0;
        dcache_req_addr = 32'h0;
        dcache_wline = 128'h0;

        // Arbiter returns to IDLE on the D-cache completion edge. It grants
        // the still-pending I-cache request on the next clock edge.
        @(posedge clk);
        #1;
        expect_mem_read(32'h4000_0000,
                        "I-cache served after D-cache finished");

        complete_i_read(128'h0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);

        @(negedge clk);
        icache_req_valid = 1'b0;
        icache_req_addr = 32'h0;

        @(posedge clk);
        #1;

        // latch behavior test
        $display("--- Test 5: Latched Request Stability ---");

        @(negedge clk);
        icache_req_valid = 1'b1;
        icache_req_addr = 32'h6000_0000;

        @(posedge clk);
        #1;
        expect_mem_read(32'h6000_0000,
                        "I-cache request granted");

        // Drop the requester before memory responds. A robust arbiter should
        // keep mem_req_valid/mem_req_addr stable from its saved request.
        @(negedge clk);
        icache_req_valid = 1'b0;
        icache_req_addr = 32'h0;
        #1;
        expect_mem_read(32'h6000_0000,
                        "Arbiter holds latched I-cache request stable");

        complete_i_read(128'hAAAA_AAAA_BBBB_BBBB_CCCC_CCCC_DDDD_DDDD);

        if (errors == 0)
            $display("ALL MEM_ARBITER TESTS PASSED");
        else
            $display("MEM_ARBITER TESTS FAILED: errors=%0d", errors);

        $finish;
    end

endmodule
