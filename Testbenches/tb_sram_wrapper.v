`timescale 1ns/1ps

// Verifies sram_sky130 against the OpenRAM behavioral model before any of it
// reaches PnR. Every check here corresponds to an assumption dcache_bram.v
// already makes about its memory; the point is to find out which of them the
// macro does NOT satisfy while a failure still costs seconds.
//
// Run from the repo root:
//   iverilog -g2005 -o /tmp/tb_sram -s tb_sram_wrapper \
//     Testbenches/tb_sram_wrapper.v src/sram_sky130.v \
//     $PDK/libs.ref/sky130_sram_macros/verilog/sky130_sram_2kbyte_1rw1r_32x512_8.v
//   vvp /tmp/tb_sram
module tb_sram_wrapper;

    localparam PERIOD = 25;

    reg clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;

    reg  [8:0]  raddr = 9'd0;
    reg         we    = 1'b0;
    reg  [8:0]  waddr = 9'd0;
    reg  [31:0] wdata = 32'd0;
    reg  [3:0]  wmask = 4'hF;
    wire [31:0] rdata;

    sram_sky130 dut (
        .clk   (clk),
        .raddr (raddr),
        .rdata (rdata),
        .we    (we),
        .waddr (waddr),
        .wdata (wdata),
        .wmask (wmask)
    );

    // A realistic consumer. The model drives dout to X at posedge+T_HOLD and
    // only restores it at negedge+DELAY, so sampling `rdata` directly from
    // procedural testbench code races with that X injection. Registering it
    // the way the cache registers q0..q3 is both race-free (T_HOLD is what
    // makes the value survive the sampling edge) and the access pattern the
    // real design uses.
    reg [31:0] rdata_q;
    always @(posedge clk) rdata_q <= rdata;

    integer errors = 0;

    task check;
        input [511:0] name;
        input [31:0]  got;
        input [31:0]  exp;
        begin
            if (got !== exp) begin
                $display("  FAIL  %0s: got %h, expected %h", name, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok    %0s = %h", name, got);
            end
        end
    endtask

    // Present a write for exactly one cycle.
    task wr;
        input [8:0]  a;
        input [31:0] d;
        input [3:0]  m;
        begin
            @(posedge clk);
            we <= 1'b1; waddr <= a; wdata <= d; wmask <= m;
            @(posedge clk);
            we <= 1'b0;
        end
    endtask

    // Read latency, spelled out once so no individual check can get it wrong:
    //   edge 0  raddr driven (settles in the NBA region)
    //   edge 1  macro captures addr1 into addr1_reg
    //   edge 2  dout1 (valid since the previous negedge) is sampled by rdata_q
    //   +1ns    rdata_q's own nonblocking update has settled and can be read
    // The trailing #1 is the part that is easy to miss: without it the task
    // reads rdata_q in the same timestep its update is scheduled, and every
    // check silently returns the PREVIOUS read's value.
    task rd_check;
        input [511:0] name;
        input [8:0]   a;
        input [31:0]  exp;
        begin
            @(posedge clk);
            raddr <= a;
            @(posedge clk);
            @(posedge clk);
            #1;
            check(name, rdata_q, exp);
        end
    endtask

    initial begin
        $display("=== sram_sky130 wrapper checks ===");

        repeat (2) @(posedge clk);

        // ------------------------------------------------------------------
        // 1. Basic write / read-back through the two separate ports.
        // ------------------------------------------------------------------
        wr(9'd5,   32'hDEAD_BEEF, 4'hF);
        wr(9'd6,   32'hCAFE_BABE, 4'hF);
        wr(9'd511, 32'h1234_5678, 4'hF);

        rd_check("read addr 5",                 9'd5,   32'hDEAD_BEEF);
        rd_check("read addr 6",                 9'd6,   32'hCAFE_BABE);
        rd_check("read addr 511 (top of array)",9'd511, 32'h1234_5678);

        // ------------------------------------------------------------------
        // 2. THE assumption the wrapper exists to protect: a write on port 0
        //    must not disturb the port 1 read output. This is what breaks if
        //    reads are moved to port 0, and it breaks silently as X rather
        //    than as a visibly wrong value.
        // ------------------------------------------------------------------
        rd_check("read addr 5 parked", 9'd5, 32'hDEAD_BEEF);

        @(posedge clk);
        we <= 1'b1; waddr <= 9'd9; wdata <= 32'hFFFF_0000; wmask <= 4'hF;
        @(posedge clk);
        we <= 1'b0;
        @(posedge clk);
        #1;
        check("read holds across a port-0 write", rdata_q, 32'hDEAD_BEEF);

        @(posedge clk);
        @(posedge clk);
        #1;
        check("read holds while idle", rdata_q, 32'hDEAD_BEEF);

        // ------------------------------------------------------------------
        // 3. Byte masks - the dcache needs sub-word stores to touch only the
        //    masked lanes.
        // ------------------------------------------------------------------
        wr(9'd20, 32'h0000_0000, 4'hF);
        wr(9'd20, 32'hAABB_CCDD, 4'b0011);        // low half only
        rd_check("byte mask 0011 -> low half only", 9'd20, 32'h0000_CCDD);

        wr(9'd20, 32'h1122_3344, 4'b1100);        // high half only
        rd_check("byte mask 1100 -> high half only", 9'd20, 32'h1122_CCDD);

        // ------------------------------------------------------------------
        // 4. Back-to-back writes with no idle cycles - the refill pattern.
        // ------------------------------------------------------------------
        @(posedge clk);
        we <= 1'b1; wmask <= 4'hF;
        waddr <= 9'd100; wdata <= 32'h1111_1111;
        @(posedge clk); waddr <= 9'd101; wdata <= 32'h2222_2222;
        @(posedge clk); waddr <= 9'd102; wdata <= 32'h3333_3333;
        @(posedge clk); we <= 1'b0;

        rd_check("refill word 0", 9'd100, 32'h1111_1111);
        rd_check("refill word 1", 9'd101, 32'h2222_2222);
        rd_check("refill word 2", 9'd102, 32'h3333_3333);

        // ------------------------------------------------------------------
        // 5. Same-address read-during-write across ports. UNDEFINED on this
        //    macro (the model prints a warning), exactly as it was on Xilinx
        //    SDP BRAM. Nothing is asserted about the value the racing read
        //    returns - asserting on undefined behaviour would only bake in
        //    whatever this simulator happens to do today. What IS checked is
        //    that the write still lands, and the warning below is the
        //    evidence that the hazard is real and the store->load bypass in
        //    dcache_bram.v remains load-bearing after the port swap.
        // ------------------------------------------------------------------
        $display("  note  a same-address RDW warning is EXPECTED next:");
        @(posedge clk);
        raddr <= 9'd200;
        we <= 1'b1; waddr <= 9'd200; wdata <= 32'h9999_9999; wmask <= 4'hF;
        @(posedge clk);
        we <= 1'b0;

        rd_check("write lands despite concurrent same-addr read",
                 9'd200, 32'h9999_9999);

        $display("=== %0d error(s) ===", errors);
        if (errors == 0) $display("PASS");
        else             $display("FAIL");
        $finish;
    end

    initial begin
        #(PERIOD * 800);
        $display("TIMEOUT");
        $finish;
    end

endmodule
