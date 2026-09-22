`timescale 1ns/1ps
`include "../src/cpu.v"
`include "../src/accel_port_join.v"

// CPU-integrated accelerator test: real sw/lw instructions through the
// pipeline to the accelerator's MMIO window, exercising axi_lite_bridge's
// AXI4-Lite master and mm_accel's AXI4-Lite slave end to end (not just the
// standalone bridge+accelerator testbench). Same mock-DDR harness as
// tb_amo_test.v/tb_coremark_sim.v.
module tb_mm_accel_cpu;
    reg clk, rst;
    wire uart_tx_line;
    reg uart_rx_line;
    wire [3:0] leds;

    wire [31:0] icache_mem_req_addr;
    wire icache_mem_req_valid;
    wire [127:0] icache_mem_read_data;
    wire icache_mem_ready;

    // mm_accel result-DMA port. In the real SoC this is mem_arbiter's third
    // requester; here it gets its own mock so the DMA can be exercised through
    // the CPU rather than only against tb_mm_accel_dma.v's standalone harness.
    wire accel_mem_req_valid;
    wire accel_mem_req_write;
    wire [31:0] accel_mem_req_addr;
    wire [127:0] accel_mem_wline;
    wire accel_mem_ready;
    wire [7:0] accel_mem_req_lines;
    reg accel_mem_wnext;

    wire [31:0] dmem_req_addr;
    wire [31:0] store_data;
    wire [3:0] mem_write_mask;
    wire dcache_mem_req_valid;
    wire dcache_mem_req_write;
    wire [127:0] dcache_mem_wline;
    wire [127:0] dcache_mem_read_data_block;
    wire dcache_mem_ready;

    wire [31:0] debug_pc, debug_instr;
    wire debug_dcache_valid, debug_dcache_ready;
    wire debug_tcm_d_req, debug_tcm_d_ready;
    wire debug_global_mem_stall;
    wire [31:0] debug_raw_pc;
    wire debug_id_predicted_taken;
    wire debug_cache_ready;

    // cpu_pipelined's accelerator port is split; accel_port_join below
    // re-serialises it onto the single-port model this bench already had, which
    // is therefore unchanged.
    wire        aj_rd_valid, aj_wr_valid, aj_wnext_o, aj_rd_ready, aj_wr_ready;
    wire [31:0] aj_rd_addr,  aj_wr_addr;
    wire [7:0]  aj_rd_lines, aj_wr_lines;
    wire [127:0] aj_wline_o;

    wire [127:0] aj_rline_w;

    cpu_pipelined DUT (
        .clk(clk), .rst(rst),
        .uart_tx(uart_tx_line),
        .uart_rx(uart_rx_line),
        .leds(leds),

        .icache_mem_req_addr(icache_mem_req_addr),
        .icache_mem_req_valid(icache_mem_req_valid),
        .icache_mem_read_data(icache_mem_read_data),
        .icache_mem_ready(icache_mem_ready),

        .dmem_req_addr(dmem_req_addr),
        .store_data(store_data),
        .mem_write_mask(mem_write_mask),

        .dcache_mem_req_valid(dcache_mem_req_valid),
        .dcache_mem_req_write(dcache_mem_req_write),
        .dcache_mem_wline(dcache_mem_wline),
        .dcache_mem_read_data_block(dcache_mem_read_data_block),
        .dcache_mem_ready(dcache_mem_ready),

        .accel_mem_rd_req_valid(aj_rd_valid),
        .accel_mem_rd_req_addr(aj_rd_addr),
        .accel_mem_rd_req_lines(aj_rd_lines),
        .accel_mem_rd_ready(aj_rd_ready),
        .accel_mem_wr_req_valid(aj_wr_valid),
        .accel_mem_wr_req_addr(aj_wr_addr),
        .accel_mem_wr_req_lines(aj_wr_lines),
        .accel_mem_wnext(aj_wnext_o),
        .accel_mem_wline(aj_wline_o),
        .accel_mem_wr_ready(aj_wr_ready),
        // req_lines was left unconnected and wnext tied to 0, which is why this
        // bench reported "DMA issued 1 line writes, expected 16": with wnext
        // never pulsed the accelerator's lineFifo is never advanced, so it
        // presents line 0 for the whole burst, and the memory model only ever
        // wrote one line anyway. Both halves of the burst protocol were missing
        // because the bench predates enableWriteBursts.
        .accel_mem_rline(aj_rline_w),
        // Operands reach the array by MMIO push in this bench, never by operand
        // DMA, so no read data is needed - but tie it off rather than leave an
        // input dangling.

        .debug_pc(debug_pc),
        .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall),
        .debug_raw_pc(debug_raw_pc),
        .debug_id_predicted_taken(debug_id_predicted_taken),
        .debug_cache_ready(debug_cache_ready)
    );

    defparam DUT.TCM.INIT_FILE =
        "../fpga/ddr_fixed.mem";

    localparam DDR_WORDS = 524288;
    reg [31:0] ddr_mem [0:DDR_WORDS-1];

    initial begin
        $readmemh("test_mm_accel_mem.hex", ddr_mem);
    end

    localparam LINE_LATENCY = 8;

    reg [3:0] i_lat_cnt;
    reg i_busy;
    reg [31:0] i_addr_latched;
    always @(posedge clk) begin
        if (rst) begin
            i_busy <= 1'b0;
            i_lat_cnt <= 0;
        end else if (!i_busy && icache_mem_req_valid) begin
            i_busy <= 1'b1;
            i_lat_cnt <= LINE_LATENCY;
            i_addr_latched <= icache_mem_req_addr;
        end else if (i_busy) begin
            if (i_lat_cnt == 0) i_busy <= 1'b0;
            else i_lat_cnt <= i_lat_cnt - 1;
        end
    end
    assign icache_mem_ready = i_busy && (i_lat_cnt == 0);
    assign icache_mem_read_data = {
        ddr_mem[{i_addr_latched[28:4], 2'b11}],
        ddr_mem[{i_addr_latched[28:4], 2'b10}],
        ddr_mem[{i_addr_latched[28:4], 2'b01}],
        ddr_mem[{i_addr_latched[28:4], 2'b00}]
    };

    reg [3:0] d_lat_cnt;
    reg d_busy;
    reg [31:0] d_addr_latched;
    reg d_write_latched;
    always @(posedge clk) begin
        if (rst) begin
            d_busy <= 1'b0;
            d_lat_cnt <= 0;
        end else if (!d_busy && dcache_mem_req_valid) begin
            d_busy <= 1'b1;
            d_lat_cnt <= LINE_LATENCY;
            d_addr_latched <= dmem_req_addr;
            d_write_latched <= dcache_mem_req_write;
        end else if (d_busy) begin
            if (d_lat_cnt == 0) d_busy <= 1'b0;
            else d_lat_cnt <= d_lat_cnt - 1;
        end
    end
    assign dcache_mem_ready = d_busy && (d_lat_cnt == 0);
    assign dcache_mem_read_data_block = {
        ddr_mem[{d_addr_latched[28:4], 2'b11}],
        ddr_mem[{d_addr_latched[28:4], 2'b10}],
        ddr_mem[{d_addr_latched[28:4], 2'b01}],
        ddr_mem[{d_addr_latched[28:4], 2'b00}]
    };
    always @(posedge clk) begin
        if (dcache_mem_ready && d_write_latched) begin
            ddr_mem[{d_addr_latched[28:4], 2'b00}] <= dcache_mem_wline[31:0];
            ddr_mem[{d_addr_latched[28:4], 2'b01}] <= dcache_mem_wline[63:32];
            ddr_mem[{d_addr_latched[28:4], 2'b10}] <= dcache_mem_wline[95:64];
            ddr_mem[{d_addr_latched[28:4], 2'b11}] <= dcache_mem_wline[127:96];
        end
    end

    // Mock line memory for the accelerator DMA, same shape as the cache mocks
    // above. Non-zero latency on purpose: the DMA's line mux is registered, and
    // a zero-latency responder is what exposed the settle-cycle bug that made
    // dim>=8 publish the previous group's data at the new address.
    reg [2:0] a_lat_cnt;
    reg a_busy;
    // Burst-capable accelerator write port.
    //
    // The protocol is taken from tb_mm_accel_queue.v, which is the bench that
    // models it correctly: one address phase, then a line every BEATS_PER_LINE=2
    // cycles, mem_wnext pulsed ONE LINE AHEAD so the requester's FIFO advances,
    // and exactly ONE mem_ready for the whole write transaction (reads signal
    // per line instead - not needed here, this bench pushes operands by MMIO).
    //
    // Getting wnext's timing wrong is the trap: pulse it too late and the last
    // line is written twice, too early and line 0 is skipped. One line ahead,
    // gated on there being a next line, is what the adapter does.
    reg [31:0]  a_addr;
    reg [31:0]  a_left;
    reg         a_in_burst, a_write, a_beat;
    reg         a_ready_r;
    integer     accel_lines_written = 0;

    always @(posedge clk) begin
        if (rst) begin
            a_ready_r      <= 1'b0;
            a_lat_cnt      <= 0;
            a_in_burst     <= 1'b0;
            a_left         <= 0;
            accel_mem_wnext <= 1'b0;
            a_beat         <= 1'b0;
        end else if (a_in_burst) begin
            accel_mem_wnext <= 1'b0;
            a_ready_r       <= 1'b0;
            if (a_left > 0) begin
                if (a_write) begin
                    if (a_beat == 1'b0) begin
                        ddr_mem[{a_addr[28:4], 2'b00}] <= accel_mem_wline[31:0];
                        ddr_mem[{a_addr[28:4], 2'b01}] <= accel_mem_wline[63:32];
                        ddr_mem[{a_addr[28:4], 2'b10}] <= accel_mem_wline[95:64];
                        ddr_mem[{a_addr[28:4], 2'b11}] <= accel_mem_wline[127:96];
                        accel_lines_written <= accel_lines_written + 1;
                        if (a_left > 1) accel_mem_wnext <= 1'b1;
                        a_beat <= 1'b1;
                    end else begin
                        a_beat <= 1'b0;
                        a_addr <= a_addr + 32'd16;
                        a_left <= a_left - 1;
                        if (a_left == 1) begin
                            a_in_burst <= 1'b0;
                            a_ready_r  <= 1'b1;   // one completion per write
                        end
                    end
                end else begin
                    // Reads are not exercised here (rline is tied off), but
                    // terminate the burst rather than hang if one ever appears.
                    a_ready_r  <= 1'b1;
                    a_addr     <= a_addr + 32'd16;
                    a_left     <= a_left - 1;
                    if (a_left == 1) a_in_burst <= 1'b0;
                end
            end
        end else if (accel_mem_req_valid && !a_ready_r) begin
            if (a_lat_cnt >= 3) begin
                a_write    <= accel_mem_req_write;
                a_addr     <= accel_mem_req_addr;
                // 0 means one line: the pre-burst interface used lines=0 for a
                // single-line request and mm_accel still emits that for the
                // non-burst paths.
                a_left     <= (accel_mem_req_lines == 8'd0)
                              ? 32'd1 : {24'd0, accel_mem_req_lines};
                a_in_burst <= 1'b1;
                a_beat     <= 1'b0;
                a_lat_cnt  <= 0;
            end else a_lat_cnt <= a_lat_cnt + 1;
        end else begin
            a_ready_r       <= 1'b0;
            accel_mem_wnext <= 1'b0;
        end
    end
    assign accel_mem_ready = a_ready_r;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst = 1;
        uart_rx_line = 1;
        repeat (5) @(posedge clk);
        rst = 0;
    end

    task check;
        input [8*8:1] name;
        input [4:0] reg_num;
        input signed [31:0] expected;
        begin
            if (DUT.RF.regs[reg_num] === expected)
                $display("PASS: %0s x%0d = %0d", name, reg_num, $signed(DUT.RF.regs[reg_num]));
            else
                $display("FAIL: %0s x%0d = %0d, expected %0d",
                    name, reg_num, $signed(DUT.RF.regs[reg_num]), expected);
        end
    endtask

    // DIM=8 with A[i][k]=i+1, B[j][k]=j+1, K=4  ->  C[i][j] = 4*(i+1)*(j+1).
    // Every expected value depends on BOTH indices, so a transposed array or a
    // collapsed row cannot pass by coincidence.
    integer ei, ej, dma_errors, widx;
    reg signed [31:0] dgot, dexp;

    initial begin
        wait (DUT.RF.regs[1] === 32'hA5A5A5A5);
        $display("Sentinel reached, checking results...");

        // geometry first: a wrong-DIM build invalidates every other check
        // INFO = {bPanels[23:16], maxK[15:8], dim[7:0]}.
        // Was 32'h00001008 (maxK=16, bPanels=0), which predated the maxK=64 /
        // bPanels=4 generator config and had been failing ever since.
        check("INFO", 24, 32'h00044008);   // bPanels=4, maxK=64, dim=8

        check("C00", 20, 4);     // 4*1*1
        check("C12", 21, 24);    // 4*2*3
        check("C35", 22, 96);    // 4*4*6
        check("C77", 23, 256);   // 4*8*8

        check("DMADONE", 25, 1);

        // ---- the DMA'd copy in memory must match, at DEST + row*STRIDE ----
        // This is the part the register window cannot verify: it checks that
        // the accumulators actually left over the 128-bit line port, in the
        // right order, at the right addresses.
        dma_errors = 0;
        for (ei = 0; ei < 8; ei = ei + 1)
            for (ej = 0; ej < 8; ej = ej + 1) begin
                widx = (32'h00180000 + ei*32 + ej*4) >> 2;
                dgot = ddr_mem[widx];
                dexp = 4 * (ei+1) * (ej+1);
                if (dgot !== dexp) begin
                    if (dma_errors < 5)
                        $display("FAIL: DMA C[%0d][%0d] = %0d, expected %0d",
                                 ei, ej, dgot, dexp);
                    dma_errors = dma_errors + 1;
                end
            end
        if (dma_errors == 0)
            $display("PASS: all 64 results DMA'd correctly to 0x00180000");
        else
            $display("FAIL: %0d DMA result mismatches", dma_errors);

        if (accel_lines_written === 16)
            $display("PASS: DMA issued 16 line writes (64 words / 4)");
        else
            $display("FAIL: DMA issued %0d line writes, expected 16", accel_lines_written);

        $display("DONE");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("[TIMEOUT] sentinel never reached (x1=0x%08h)", DUT.RF.regs[1]);
        $display("pc=0x%08h instr=0x%08h global_mem_stall=%b accel_pending=%b accel_done=%b",
            DUT.debug_pc, DUT.debug_instr, DUT.global_mem_stall,
            DUT.accel_pending, DUT.accel_done);
        $finish;
    end

    accel_port_join AJ (
        .clk(clk), .rst(rst),
        .accel_rd_req_valid(aj_rd_valid), .accel_rd_req_addr(aj_rd_addr),
        .accel_rd_req_lines(aj_rd_lines), .accel_rd_ready(aj_rd_ready),
        .accel_rline(aj_rline_w),
        .accel_wr_req_valid(aj_wr_valid), .accel_wr_req_addr(aj_wr_addr),
        .accel_wr_req_lines(aj_wr_lines), .accel_wline(aj_wline_o),
        .accel_wnext(aj_wnext_o), .accel_wr_ready(aj_wr_ready),
        .mem_req_valid(accel_mem_req_valid), .mem_req_write(accel_mem_req_write),
        .mem_req_addr(accel_mem_req_addr), .mem_req_lines(accel_mem_req_lines),
        .mem_wline(accel_mem_wline), .mem_ready(accel_mem_ready),
        .mem_wnext(accel_mem_wnext), .mem_rline(128'b0)
    );

endmodule
