`timescale 1ns/1ps
`include "../src/cpu.v"

// CoreMark sim-verification testbench for the branch predictor work.
// Uses a small flat "DDR" word memory (2MB, matching Testbenches/linker_sim.ld's
// reduced stack-top window) behind both the icache and dcache line
// interfaces, and a fixed per-line memory latency (LINE_LATENCY cycles from
// req_valid to ready) standing in for the real AXI/mem_arbiter path - not
// latency-accurate to hardware, but sufficient to verify functional
// correctness (CoreMark CRCs) and get an internally-consistent relative
// cycle-count comparison against the pre-predictor baseline when both are
// run through this same mock.
module tb_coremark_sim;
    reg clk, rst;
    wire uart_tx_line;
    reg uart_rx_line;
    wire [3:0] leds;

    wire [31:0] icache_mem_req_addr;
    wire icache_mem_req_valid;
    wire [127:0] icache_mem_read_data;
    wire icache_mem_ready;

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

    // Define USE_SRAM to run CoreMark against the sky130 SRAM-macro D-cache
    // instead of inferred BRAM. The CRCs are the strongest correctness gate
    // in the repo: they check millions of cache accesses end to end, so a
    // mismatch here means corrupted data, not a tuning difference. Note the
    // cache is 8KB in this config against 16KB by default, so the CYCLE
    // COUNT legitimately differs - only the CRCs must match.
    //
    // Five configurations. The first three attribute a macro-path failure;
    // the last two exist to MEASURE IPC, because the ASIC's CoreMark/MHz can
    // only be quoted against the cache geometry the ASIC actually builds.
    //
    //   (default)     BRAM, IC 2048 / DC 1024   32KB I + 16KB D - the FPGA
    //                                            config, where 3.17
    //                                            CoreMark/MHz was measured
    //   -DSMALL_DC    BRAM, IC 2048 / DC  512   isolates D-cache SIZE
    //   -DUSE_SRAM    macro, IC 2048 / DC 512   isolates the MACRO
    //   -DASIC_TARGET BRAM, IC  512 / DC  512    8KB I +  8KB D - what the
    //                                            macro ASIC will build
    //   -DASIC_NOW    BRAM, IC   16 / DC   16   256B each - today's ASIC
    //                                            config, the number the
    //                                            signed-off 3.328 mm^2 run
    //                                            actually corresponds to
    //
    // ASIC_TARGET and ASIC_NOW use the BRAM path deliberately: it is
    // cycle-identical to the macro path at the same geometry (verified
    // bit-for-bit on every counter), and it simulates far faster.
`ifdef USE_SRAM
    cpu_pipelined #(
        .IC_NUM_SETS(512),      // the macro's fixed depth
        .DC_NUM_SETS(512),
        .IC_USE_SRAM(1),
        .DC_USE_SRAM(1)
    ) DUT (
`elsif SMALL_DC
    cpu_pipelined #(
        .DC_NUM_SETS(512),
        .DC_USE_SRAM(0)
    ) DUT (
`elsif ASIC_TARGET
    cpu_pipelined #(
        .IC_NUM_SETS(512),
        .DC_NUM_SETS(512)
    ) DUT (
`elsif ASIC_NOW
    cpu_pipelined #(
        .IC_NUM_SETS(16),
        .DC_NUM_SETS(16)
    ) DUT (
`else
    cpu_pipelined DUT (
`endif
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

        .debug_pc(debug_pc),
        .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall)
    );

    // TCM boot stub, as a repo-relative path so any clone can run this.
    // ddr_fixed.mem jumps to 0x00100000, matching linker_sim.ld's .text base.
    defparam DUT.TCM.INIT_FILE = "../fpga/ddr_fixed.mem";

    // ---- Mock "DDR" word memory, 2MB (word idx 0 .. 524287) ----
    localparam DDR_WORDS = 524288;
    reg [31:0] ddr_mem [0:DDR_WORDS-1];

    initial begin
        $readmemh("coremark_sim_mem.hex", ddr_mem);
    end

    localparam LINE_LATENCY = 8; // cycles from req_valid to ready, both caches

    // Icache line reads
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

    // Dcache line reads/writes
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

    // ---- UART TX capture: snoop uart_mmio's internal tx_start/tx_data
    // instead of decoding the serial line, since we only care about the
    // printed CoreMark report text, not physical-layer timing. ----
    integer char_count = 0;
    always @(posedge clk) begin
        if (DUT.UART.tx_start) begin
            $write("%c", DUT.UART.tx_data);
            char_count = char_count + 1;
        end
    end

    // ---- Clock/reset ----
    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst = 1;
        uart_rx_line = 1;
        repeat (5) @(posedge clk);
        rst = 0;
    end

    // ---- Cycle counter (from end of reset) ----
    integer cycle_count = 0;
    always @(posedge clk) begin
        if (!rst) cycle_count = cycle_count + 1;
    end

    integer branch_count = 0;
    integer mispredict_count = 0;
    always @(posedge clk) begin
        if (!rst && DUT.id_ex_is_branch) begin
            branch_count = branch_count + 1;
            if (DUT.branch_mispredicted) mispredict_count = mispredict_count + 1;
        end
    end

    integer jalr_count = 0;
    integer jalr_ras_hit_count = 0;
    integer jalr_ras_mispredict_count = 0;
    always @(posedge clk) begin
        if (!rst && DUT.id_ex_is_jalr) begin
            jalr_count = jalr_count + 1;
            if (DUT.id_ex_predicted_taken) begin
                jalr_ras_hit_count = jalr_ras_hit_count + 1;
                if (DUT.jalr_ras_mispredicted) jalr_ras_mispredict_count = jalr_ras_mispredict_count + 1;
            end
        end
    end

    // Stall breakdown: global_mem_stall is an OR of these sub-causes
    // (cpu.v:425) - counting each separately (instead of just the combined
    // total) shows where cycles are actually going, the same
    // instrumentation-first approach used to size the branch/RAS wins
    // before deciding what to build next.
    integer stall_cycles = 0;          // load-use hazard (hazard_unit)
    integer store_data_stall_cycles = 0; // subset of the above a T3.3 bypass could remove
    integer imem_stall_cycles = 0;     // icache miss
    integer dmem_stall_cycles = 0;     // dcache miss
    integer div_stall_cycles = 0;      // in-flight DIV/REM
    integer uart_stall_cycles = 0;
    integer intc_stall_cycles = 0;
    integer amo_stall_cycles = 0;
    integer accel_stall_cycles = 0;
    integer global_mem_stall_cycles = 0; // union of the above (may overlap)
    always @(posedge clk) begin
        if (!rst) begin
            if (DUT.stall) stall_cycles = stall_cycles + 1;
            // How many interlock stalls could a load->store-data bypass remove?
            //
            // Measurement, not a fix. A store's data is not needed until
            // partial_store in MEM, one stage later than the interlock assumes,
            // so a stall whose ONLY dependence is the store's rs2 is
            // architecturally unnecessary. Counting them first because the
            // estimate for this optimisation came from the same
            // uniform-probability model that over-predicted the rs1/rs2
            // qualifier by 20x, and the fix needs new pipeline state (the
            // store's rs2 INDEX carried into MEM) rather than a decode tweak.
            //
            // Store opcode is 5'b01000; rs2-only means rd matches [24:20] and
            // does not match [19:15].
            if (DUT.stall &&
                (DUT.if_id_inst[6:2] == 5'b01000) &&
                (DUT.id_ex_rd == DUT.if_id_inst[24:20]) &&
                (DUT.id_ex_rd != DUT.if_id_inst[19:15]))
                store_data_stall_cycles = store_data_stall_cycles + 1;
            if (DUT.imem_stall) imem_stall_cycles = imem_stall_cycles + 1;
            if (DUT.dmem_stall) dmem_stall_cycles = dmem_stall_cycles + 1;
            if (DUT.div_stall) div_stall_cycles = div_stall_cycles + 1;
            if (DUT.uart_stall) uart_stall_cycles = uart_stall_cycles + 1;
            if (DUT.intc_stall) intc_stall_cycles = intc_stall_cycles + 1;
            if (DUT.amo_stall) amo_stall_cycles = amo_stall_cycles + 1;
            if (DUT.accel_stall) accel_stall_cycles = accel_stall_cycles + 1;
            if (DUT.global_mem_stall) global_mem_stall_cycles = global_mem_stall_cycles + 1;
        end
    end

    // Icache refill activity. The prefetcher is gone (see icache.v) - what
    // matters now is how often the BRAM cache actually has to go to memory,
    // which should fall to near-zero once the working set is resident.
    integer icache_refills = 0;
    always @(posedge clk) begin
        if (!rst && DUT.ICACHE.BRAM_CACHE.mem_req_valid &&
                    DUT.ICACHE.BRAM_CACHE.mem_ready)
            icache_refills = icache_refills + 1;
    end

    // Stuck-PC watchdog: if PC hasn't moved in a very long time, bail out
    // instead of burning the full timeout budget.
    reg [31:0] last_pc;
    integer stuck_count = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (debug_pc == last_pc) stuck_count = stuck_count + 1;
            else stuck_count = 0;
            last_pc = debug_pc;
            if (stuck_count > 200000) begin
                $display("\n[STUCK] PC has not moved for 200000 cycles at pc=0x%08h", debug_pc);
                $display("FINAL_CYCLES=%0d BRANCHES=%0d MISPREDICTS=%0d JALR=%0d RAS_HITS=%0d RAS_MISS=%0d", cycle_count, branch_count, mispredict_count, jalr_count, jalr_ras_hit_count, jalr_ras_mispredict_count);
            $display("STOREDATA=%0d STALL=%0d IMEM=%0d DMEM=%0d DIV=%0d UART=%0d INTC=%0d AMO=%0d ACCEL=%0d MEMSTALL_UNION=%0d", store_data_stall_cycles, stall_cycles, imem_stall_cycles, dmem_stall_cycles, div_stall_cycles, uart_stall_cycles, intc_stall_cycles, amo_stall_cycles, accel_stall_cycles, global_mem_stall_cycles);
            $display("ICACHE_REFILLS=%0d", icache_refills);
                $finish;
            end
        end
    end

    initial begin
        #200_000_000; // 200ms sim time budget
        $display("\n[TIMEOUT] Simulation time budget exhausted");
        $display("FINAL_CYCLES=%0d BRANCHES=%0d MISPREDICTS=%0d", cycle_count, branch_count, mispredict_count);
        $finish;
    end

    // Detect end of run: CoreMark prints "Correct operation validated" (or
    // an error) near the very end, right before returning from main(). We
    // just watch for the char count going quiet for a while after having
    // seen a decent amount of output, as a simple "done" heuristic, backed
    // up by the hard timeout above.
    integer quiet_cycles = 0;
    integer last_char_count = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (char_count != last_char_count) begin
                quiet_cycles = 0;
                last_char_count = char_count;
            end else if (char_count > 100) begin
                quiet_cycles = quiet_cycles + 1;
                if (quiet_cycles > 500000) begin
                    $display("\n[DONE] UART output quiesced");
                    $display("FINAL_CYCLES=%0d BRANCHES=%0d MISPREDICTS=%0d JALR=%0d RAS_HITS=%0d RAS_MISS=%0d", cycle_count, branch_count, mispredict_count, jalr_count, jalr_ras_hit_count, jalr_ras_mispredict_count);
            $display("STOREDATA=%0d STALL=%0d IMEM=%0d DMEM=%0d DIV=%0d UART=%0d INTC=%0d AMO=%0d ACCEL=%0d MEMSTALL_UNION=%0d", store_data_stall_cycles, stall_cycles, imem_stall_cycles, dmem_stall_cycles, div_stall_cycles, uart_stall_cycles, intc_stall_cycles, amo_stall_cycles, accel_stall_cycles, global_mem_stall_cycles);
            $display("ICACHE_REFILLS=%0d", icache_refills);
                    $finish;
                end
            end
        end
    end

endmodule
