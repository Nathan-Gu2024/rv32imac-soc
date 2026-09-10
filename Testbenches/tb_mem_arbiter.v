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

    // accelerator result DMA (third requester)
    reg accel_req_valid;
    // Burst length for the accelerator port. Left undriven it is X, which
    // makes the arbiter's end-of-burst compare undefined and strands SVC_A,
    // so every round-robin fairness check fails for a reason that has
    // nothing to do with fairness.
    reg [7:0] accel_req_lines;
    reg       mem_wnext;
    reg accel_req_write;
    reg [31:0] accel_req_addr;
    reg [127:0] accel_wline;
    wire accel_ready;
    wire [127:0] accel_rline;

    // DDR / fake line-memory
    wire mem_req_valid;
    wire mem_req_write;
    wire [31:0] mem_req_addr;
    wire [7:0]  mem_req_lines;
    wire        accel_wnext;
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

        .accel_req_valid(accel_req_valid),
        .accel_req_lines(accel_req_lines),
        .accel_req_write(accel_req_write),
        .accel_req_addr(accel_req_addr),
        .accel_wline(accel_wline),
        .accel_ready(accel_ready),
        .accel_wnext(accel_wnext),
        .accel_rline(accel_rline),

        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_req_lines(mem_req_lines),
        .mem_wline(mem_wline),
        .mem_ready(mem_ready),
        .mem_wnext(mem_wnext),
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

            accel_req_valid = 1'b0;
            accel_req_lines = 8'd1;
            mem_wnext = 1'b0;
            accel_req_write = 1'b0;
            accel_req_addr = 32'h0;
            accel_wline = 128'h0;

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

        // Test 4: Simultaneous requests. The arbiter is ROUND-ROBIN, not
        // D-cache priority.
        //
        // This test used to assert "D-cache was served first" and had been
        // failing since mem_arbiter moved to round-robin fairness - the
        // expectation encoded the OLD policy, where dcache always won outright
        // and could starve the I-cache. Test 3 served the D-cache, so
        // last_granted == 0 and grant_i wins this tie by design. Asserting
        // D-first here would be asserting the starvation bug.
        $display("--- Test 4: Simultaneous Requests (round-robin) ---");

        @(negedge clk);
        icache_req_valid = 1'b1;
        icache_req_addr = 32'h4000_0000;

        dcache_req_valid = 1'b1;
        dcache_req_write = 1'b1;
        dcache_req_addr = 32'h5000_0000;
        dcache_wline = 128'hCAFE_BAFE_CAFE_BAFE_CAFE_BAFE_CAFE_BAFE;

        @(posedge clk);
        #1;
        expect_mem_read(32'h4000_0000,
                        "I-cache won the tie (D-cache went last)");

        complete_i_read(128'h0123_4567_89AB_CDEF_FEDC_BA98_7654_3210);

        @(negedge clk);
        icache_req_valid = 1'b0;
        icache_req_addr = 32'h0;

        // Arbiter returns to IDLE on the I-cache completion edge and grants the
        // still-pending D-cache request on the next clock edge.
        @(posedge clk);
        #1;
        expect_mem_write(32'h5000_0000,
                         128'hCAFE_BAFE_CAFE_BAFE_CAFE_BAFE_CAFE_BAFE,
                         "D-cache served after I-cache finished");

        complete_d_write();

        @(negedge clk);
        dcache_req_valid = 1'b0;
        dcache_req_write = 1'b0;
        dcache_req_addr = 32'h0;
        dcache_wline = 128'h0;

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

        @(negedge clk);
        icache_req_valid = 1'b0;
        icache_req_addr = 32'h0;
        @(posedge clk);
        #1;

        // ---- Test 6: accelerator result-DMA port on its own ----
        $display("--- Test 6: Accelerator DMA Port ---");

        @(negedge clk);
        accel_req_valid = 1'b1;
        accel_req_write = 1'b1;
        accel_req_addr  = 32'h2100_0000;
        accel_wline     = 128'h1111_2222_3333_4444_5555_6666_7777_8888;

        @(posedge clk);
        #1;
        expect_mem_write(32'h2100_0000,
                         128'h1111_2222_3333_4444_5555_6666_7777_8888,
                         "Accelerator DMA line reaches memory");

        @(negedge clk);
        mem_ready = 1'b1;
        #1;
        check(accel_ready, "Accelerator sees ready during mem_ready cycle");
        check(!icache_ready && !dcache_ready,
              "Caches do NOT see ready on an accelerator grant");
        @(posedge clk);
        @(negedge clk);
        mem_ready = 1'b0;
        accel_req_valid = 1'b0;
        accel_req_write = 1'b0;
        accel_req_addr  = 32'h0;
        accel_wline     = 128'h0;
        @(posedge clk);
        #1;

        // ---- Test 8: multi-line accelerator burst ----
        //
        // The adapter answers a burst with ONE mem_ready per line and holds the
        // transaction open in between. The arbiter must stay in SVC_A for the
        // whole run: releasing on the first pulse (correct for the single-line
        // caches) would drop the port with lines still in flight.
        $display("--- Test 8: Multi-line accelerator burst ---");

        begin : burst_test
            integer b;
            integer stayed;
            stayed = 1;

            @(negedge clk);
            accel_req_valid = 1'b1;
            accel_req_write = 1'b1;
            accel_req_lines = 8'd4;              // four lines in one transaction
            accel_req_addr  = 32'h2200_0000;
            accel_wline     = 128'hAAAA_0000_BBBB_1111_CCCC_2222_DDDD_3333;

            @(posedge clk);
            #1;
            check(mem_req_valid && mem_req_write && mem_req_addr == 32'h2200_0000,
                  "Burst request reaches memory");
            check(mem_req_lines == 8'd4,
                  "Burst length is passed through to the adapter");

            // Four ready pulses, one per line. The port must remain granted to
            // the accelerator for all of them.
            for (b = 0; b < 4; b = b + 1) begin
                @(negedge clk);
                // Drop the request before the LAST line completes. The arbiter
                // latched saved_* at grant so this is safe mid-burst, and it
                // stops the arbiter re-granting the moment it retires - which
                // would open a second transaction with no mem_ready behind it
                // and strand the port in SVC_A.
                if (b == 3) begin
                    accel_req_valid = 1'b0;
                    accel_req_write = 1'b0;
                end
                mem_ready = 1'b1;
                #1;
                if (b < 3) begin
                    if (!mem_req_valid) stayed = 0;      // released too early
                    if (icache_ready || dcache_ready) stayed = 0;
                end
                check(accel_ready, "Accelerator sees ready for each burst line");
                @(posedge clk);
                @(negedge clk);
                mem_ready = 1'b0;
                #1;
            end

            check(stayed,
                  "Arbiter held the port for the whole burst");

            // Drop the request so the arbiter can retire to IDLE. Leaving it
            // asserted makes the arbiter re-grant immediately - correct
            // behaviour, but it means the release check below would be testing
            // the next transaction rather than the end of this one.
            @(negedge clk);
            accel_req_lines = 8'd1;
            accel_req_addr  = 32'h0;
            accel_wline     = 128'h0;
            @(posedge clk);
            #1;
            check(!mem_req_valid,
                  "Port released after the final burst line");

            // wnext must be gated by ownership: with the accelerator idle, a
            // pulse from the adapter must not reach it.
            @(negedge clk);
            mem_wnext = 1'b1;
            #1;
            check(!accel_wnext,
                  "wnext is gated off when the accelerator does not own the port");
            @(posedge clk);
            @(negedge clk);
            mem_wnext = 1'b0;
            @(posedge clk);
            #1;
        end

        // ---- Test 7: three-way contention must not starve anyone ----
        //
        // The point of rotating priority. With all three requesters holding
        // their requests high, a fixed priority order would let the top two
        // alternate forever and never serve the third. Each of the three must
        // be granted exactly once across three consecutive grants - the order
        // depends on who went last, so this checks COVERAGE, not sequence.
        $display("--- Test 7: Three-way contention, no starvation ---");

        begin : three_way
            integer n;
            integer saw_i, saw_d, saw_a;
            saw_i = 0; saw_d = 0; saw_a = 0;

            @(negedge clk);
            icache_req_valid = 1'b1; icache_req_addr = 32'h4000_0000;
            dcache_req_valid = 1'b1; dcache_req_write = 1'b1;
            dcache_req_addr  = 32'h5000_0000;
            dcache_wline     = 128'hCAFE_BAFE_CAFE_BAFE_CAFE_BAFE_CAFE_BAFE;
            accel_req_valid  = 1'b1; accel_req_write = 1'b1;
            accel_req_addr   = 32'h2100_0010;
            accel_wline      = 128'hDEAD_BEEF_DEAD_BEEF_DEAD_BEEF_DEAD_BEEF;

            for (n = 0; n < 3; n = n + 1) begin
                @(posedge clk);
                #1;
                check(mem_req_valid, "A requester is granted each round");

                @(negedge clk);
                mem_ready = 1'b1;
                #1;
                if (icache_ready) begin
                    saw_i = saw_i + 1;
                    icache_req_valid = 1'b0;
                end
                if (dcache_ready) begin
                    saw_d = saw_d + 1;
                    dcache_req_valid = 1'b0;
                end
                if (accel_ready) begin
                    saw_a = saw_a + 1;
                    accel_req_valid = 1'b0;
                end
                @(posedge clk);
                @(negedge clk);
                mem_ready = 1'b0;
            end

            check(saw_i == 1, "I-cache served exactly once in three rounds");
            check(saw_d == 1, "D-cache served exactly once in three rounds");
            check(saw_a == 1, "Accelerator served exactly once in three rounds");
        end

        if (errors == 0)
            $display("ALL MEM_ARBITER TESTS PASSED");
        else
            $display("MEM_ARBITER TESTS FAILED: errors=%0d", errors);

        $finish;
    end

endmodule
