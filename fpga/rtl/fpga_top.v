module fpga_top #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BITS = 128,
    parameter AXI_DATA_WIDTH = 64,
    // Which memory path the accelerator's split port feeds. Both are live and
    // both are simulated by tb_result_dma from the same stimulus and the same
    // AXI slave (-DRW_SPLIT selects 1), so this is a real A/B and not a flag
    // with one tested setting:
    //
    //   1  mem_arbiter_rw + axi_rw_engine. Reads and writes are independent,
    //      so an operand fetch overlaps a result store. 805 cyc/batch in sim.
    //   0  accel_port_join + mem_arbiter + axi_cache_adapter. The join
    //      re-serialises the split port onto one request, reproducing the
    //      behaviour that was on silicon before this branch. 840 cyc/batch.
    //
    // Set to 0 to fall back; it is a one-parameter rebuild, and the legacy
    // modules are still in the tree and still in the Vivado project for it.
    parameter MEM_PATH_RW = 1
)(
    input wire clk,
    input wire rst,

    // Physical UART pins - driven/read directly by cpu_pipelined's internal
    // uart_mmio peripheral (MMIO at 0x4000_1000; see uart_mmio.v).
    output wire uart_tx,
    input wire uart_rx,

    output wire [3:0] leds,

    // AXI4 Master Interface -> connect to Zynq PS S_AXI_HP* through AXI interconnect/smartconnect

    // Read address channel
    output wire [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0] m_axi_arlen,
    output wire [2:0] m_axi_arsize,
    output wire [1:0] m_axi_arburst,
    output wire m_axi_arvalid,
    input wire m_axi_arready,

    // Read data channel
    input wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input wire m_axi_rvalid,
    input wire m_axi_rlast,
    output wire m_axi_rready,

    // Write address channel
    output wire [ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0] m_axi_awlen,
    output wire [2:0] m_axi_awsize,
    output wire [1:0] m_axi_awburst,
    output wire m_axi_awvalid,
    input wire m_axi_awready,

    // Write data channel
    output wire [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire m_axi_wlast,
    output wire m_axi_wvalid,
    input wire m_axi_wready,

    // Write response channel
    input wire [1:0] m_axi_bresp,
    input wire m_axi_bvalid,
    output wire m_axi_bready
);

    // CPU I-cache lower-memory line interface
    wire icache_req_valid;
    wire [31:0] icache_req_addr;
    wire icache_ready;
    wire [127:0] icache_rline;

    // CPU D-cache lower-memory line interface
    wire dcache_req_valid;
    wire dcache_req_write;
    wire [31:0] dcache_req_addr;
    wire [127:0] dcache_wline;
    wire dcache_ready;
    wire [127:0] dcache_rline;

    // mm_accel memory port, split into independent read and write channels.
    wire         accel_rd_req_valid, accel_wr_req_valid;
    wire [31:0]  accel_rd_req_addr,  accel_wr_req_addr;
    wire [7:0]   accel_rd_req_lines, accel_wr_req_lines;
    wire         accel_rd_ready,     accel_wr_ready;
    wire [127:0] accel_wline;
    wire [127:0] accel_rline;
    wire [7:0]  mem_req_lines;
    wire        accel_wnext;
    wire        mem_wnext;

    // Join <-> legacy arbiter, used only when MEM_PATH_RW = 0.
    wire         accel_req_valid, accel_req_write, accel_ready_j, accel_wnext_j;
    wire [31:0]  accel_req_addr;
    wire [7:0]   accel_req_lines;
    wire [127:0] accel_wline_j, accel_rline_j;

    // Arbiter <-> AXI adapter shared line interface
    wire mem_req_valid;
    wire mem_req_write;
    wire [31:0] mem_req_addr;
    wire [127:0] mem_wline;
    wire [127:0] mem_rline;
    wire mem_ready;

    wire [3:0] cpu_leds;
    // Was ILA probe wiring (mark_debug) for tracing PC/instruction/memory-
    // stall behavior - attributes removed since (a) hardware validation has
    // been happening without needing them, and (b) mark_debug was found to
    // parasitically lengthen a real timing-critical path (Vivado shares
    // LUTs between functional and debug logic when they share fan-in;
    // debug_cache_ready's own logic showed up at the start of the worst
    // reported setup path). Left as plain wires - unused now, so synthesis
    // trims them, without needing to touch cpu_pipelined's port list.
    wire [31:0] debug_pc, debug_instr;

    wire debug_dcache_valid;
    wire debug_dcache_ready;
    wire debug_tcm_d_req;
    wire debug_tcm_d_ready;
    wire debug_global_mem_stall;
    wire [31:0] debug_raw_pc;
    wire debug_id_predicted_taken;
    wire debug_cache_ready;

    cpu_pipelined CPU_CORE (
        .debug_pc(debug_pc),
        .debug_instr(debug_instr),
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req),
        .debug_tcm_d_ready(debug_tcm_d_ready),
        .debug_global_mem_stall(debug_global_mem_stall),
        .debug_raw_pc(debug_raw_pc),
        .debug_id_predicted_taken(debug_id_predicted_taken),
        .debug_cache_ready(debug_cache_ready),


        .clk(clk),
        .rst(rst),

        .uart_tx(uart_tx),
        .uart_rx(uart_rx),
        .leds(cpu_leds),

        // I-cache lower-memory line interface
        .icache_mem_req_addr(icache_req_addr),
        .icache_mem_req_valid(icache_req_valid),
        .icache_mem_read_data(icache_rline),
        .icache_mem_ready(icache_ready),

        // D-cache lower-memory line interface
        // cpu_pipelined still names the address output dmem_req_addr internally.
        .dmem_req_addr(dcache_req_addr),
        .store_data(),
        .mem_write_mask(),
        .dcache_mem_req_valid(dcache_req_valid),
        .dcache_mem_req_write(dcache_req_write),
        .dcache_mem_wline(dcache_wline),
        .dcache_mem_read_data_block(dcache_rline),
        .dcache_mem_ready(dcache_ready),

        .accel_mem_rd_req_valid(accel_rd_req_valid),
        .accel_mem_rd_req_addr(accel_rd_req_addr),
        .accel_mem_rd_req_lines(accel_rd_req_lines),
        .accel_mem_rd_ready(accel_rd_ready),
        .accel_mem_rline(accel_rline),
        .accel_mem_wr_req_valid(accel_wr_req_valid),
        .accel_mem_wr_req_addr(accel_wr_req_addr),
        .accel_mem_wr_req_lines(accel_wr_req_lines),
        .accel_mem_wline(accel_wline),
        .accel_mem_wnext(accel_wnext),
        .accel_mem_wr_ready(accel_wr_ready)
    );

    // ================================================================
    // Memory path, selectable.
    //
    //   MEM_PATH_RW = 1  mem_arbiter_rw + axi_rw_engine. Reads and writes are
    //                    independent all the way to the AXI master, so the
    //                    accelerator's operand fetch can overlap its result
    //                    store and a cache read no longer waits behind a
    //                    63-line GEMM burst.
    //   MEM_PATH_RW = 0  accel_port_join + mem_arbiter + axi_cache_adapter.
    //                    The path that is on silicon today, byte for byte, with
    //                    the join re-serialising the accelerator's split port
    //                    onto the single-port arbiter.
    //
    // Kept selectable so the two can be A/B'd on hardware. The Vivado project's
    // file list is mirror-owned and lives outside this repo, so a `generate`
    // beats swapping module names: every module stays in the project and
    // switching is a one-integer edit.
    //
    // DEFAULTS TO 0 in the commit that introduces it, on purpose - the port
    // split is a no-behaviour-change step, so a bitstream from here must behave
    // exactly as the current one does. Flipping the default is a separate step,
    // which is what makes any hardware difference attributable to the flip.
    // ================================================================
    generate
    if (MEM_PATH_RW) begin : g_rw_split

        wire        rd_req_valid, wr_req_valid, wr_next_w, rd_ready_w, wr_ready_w;
        wire [31:0] rd_req_addr,  wr_req_addr;
        wire [7:0]  rd_req_lines, wr_req_lines;
        wire [127:0] rd_rline_w,  wr_wline_w;

        mem_arbiter_rw ARBITER (
            .clk(clk), .rst(rst),
        .icache_req_valid(icache_req_valid),
        .icache_req_addr(icache_req_addr),
        .icache_ready(icache_ready), .icache_rline(icache_rline),
        .dcache_req_valid(dcache_req_valid),
        .dcache_req_write(dcache_req_write),
        .dcache_req_addr(dcache_req_addr), .dcache_wline(dcache_wline),
        .dcache_ready(dcache_ready), .dcache_rline(dcache_rline),
            .accel_rd_req_valid(accel_rd_req_valid),
            .accel_rd_req_addr(accel_rd_req_addr),
            .accel_rd_req_lines(accel_rd_req_lines),
            .accel_wr_req_valid(accel_wr_req_valid),
            .accel_wr_req_addr(accel_wr_req_addr),
            .accel_wr_req_lines(accel_wr_req_lines),
            .accel_wline(accel_wline),
            .accel_rd_ready(accel_rd_ready), .accel_wr_ready(accel_wr_ready),
            .accel_wnext(accel_wnext), .accel_rline(accel_rline),
            .rd_req_valid(rd_req_valid), .rd_req_addr(rd_req_addr),
            .rd_req_lines(rd_req_lines), .rd_ready(rd_ready_w),
            .rd_rline(rd_rline_w),
            .wr_req_valid(wr_req_valid), .wr_req_addr(wr_req_addr),
            .wr_req_lines(wr_req_lines), .wr_wline(wr_wline_w),
            .wr_next(wr_next_w), .wr_ready(wr_ready_w)
        );

        axi_rw_engine #(
            .ADDR_WIDTH(ADDR_WIDTH), .LINE_BITS(LINE_BITS),
            .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
        ) AXI_ENGINE (
            .clk(clk), .rst(rst),
            .rd_req_valid(rd_req_valid), .rd_req_addr(rd_req_addr),
            .rd_req_lines(rd_req_lines), .rd_rline(rd_rline_w),
            .rd_ready(rd_ready_w),
            .wr_req_valid(wr_req_valid), .wr_req_addr(wr_req_addr),
            .wr_req_lines(wr_req_lines), .wr_wline(wr_wline_w),
            .wr_next(wr_next_w), .wr_ready(wr_ready_w),
        .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rlast(m_axi_rlast), .m_axi_rready(m_axi_rready),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),
            .rd_retries(), .wr_retries()
        );

    end else begin : g_legacy

        accel_port_join ACCEL_JOIN (
            .clk(clk), .rst(rst),
            .accel_rd_req_valid(accel_rd_req_valid),
            .accel_rd_req_addr(accel_rd_req_addr),
            .accel_rd_req_lines(accel_rd_req_lines),
            .accel_rd_ready(accel_rd_ready), .accel_rline(accel_rline),
            .accel_wr_req_valid(accel_wr_req_valid),
            .accel_wr_req_addr(accel_wr_req_addr),
            .accel_wr_req_lines(accel_wr_req_lines),
            .accel_wline(accel_wline), .accel_wnext(accel_wnext),
            .accel_wr_ready(accel_wr_ready),
            .mem_req_valid(accel_req_valid), .mem_req_write(accel_req_write),
            .mem_req_addr(accel_req_addr), .mem_req_lines(accel_req_lines),
            .mem_wline(accel_wline_j), .mem_ready(accel_ready_j),
            .mem_wnext(accel_wnext_j), .mem_rline(accel_rline_j)
        );

        mem_arbiter ARBITER (
            .clk(clk), .rst(rst),
        .icache_req_valid(icache_req_valid),
        .icache_req_addr(icache_req_addr),
        .icache_ready(icache_ready), .icache_rline(icache_rline),
        .dcache_req_valid(dcache_req_valid),
        .dcache_req_write(dcache_req_write),
        .dcache_req_addr(dcache_req_addr), .dcache_wline(dcache_wline),
        .dcache_ready(dcache_ready), .dcache_rline(dcache_rline),
            .accel_req_valid(accel_req_valid),
            .accel_req_write(accel_req_write),
            .accel_req_addr(accel_req_addr),
            .accel_req_lines(accel_req_lines),
            .accel_wline(accel_wline_j),
            .accel_ready(accel_ready_j),
            .accel_wnext(accel_wnext_j),
            .accel_rline(accel_rline_j),
            .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
            .mem_req_addr(mem_req_addr), .mem_req_lines(mem_req_lines),
            .mem_wline(mem_wline), .mem_ready(mem_ready),
            .mem_wnext(mem_wnext), .mem_rline(mem_rline)
        );

        axi_cache_adapter #(
            .ADDR_WIDTH(ADDR_WIDTH), .LINE_BITS(LINE_BITS),
            .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
        ) AXI_ADAPTER (
            .clk(clk), .rst(rst),
            .mem_req_valid(mem_req_valid), .mem_req_write(mem_req_write),
            .mem_req_addr(mem_req_addr), .mem_req_lines(mem_req_lines),
            .mem_wnext(mem_wnext), .mem_wline(mem_wline),
            .mem_rline(mem_rline), .mem_ready(mem_ready),
        .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rlast(m_axi_rlast), .m_axi_rready(m_axi_rready),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready)
        );

    end
    endgenerate
    reg [26:0] heartbeat;
    always @(posedge clk) begin
        if (rst)
            heartbeat <= 27'd0;
        else
            heartbeat <= heartbeat + 1'b1;
    end

    assign leds[3:0] = cpu_leds[3:0];

//    assign leds[2] = debug_pc[4];
//    assign leds[1] = debug_pc[3];
//    assign leds[0] = debug_pc[2];
//    assign leds = 4'b1111;
endmodule
