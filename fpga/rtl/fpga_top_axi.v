module fpga_top #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BITS = 128,
    parameter AXI_DATA_WIDTH = 64
)(
    input wire clk,
    input wire rst_n, // Active-low reset from Zynq or board button

    // Peripheral Outputs
    output wire [3:0] leds,
    output wire       uart_tx,

    // AXI4 Master Interface -> Connects to Zynq HP0 Port
    // Read Address Channel
    output wire [ADDR_WIDTH-1:0]     m_axi_araddr,
    output wire [7:0]                m_axi_arlen,
    output wire [2:0]                m_axi_arsize,
    output wire [1:0]                m_axi_arburst,
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,

    // Read Data Channel
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rvalid,
    input  wire                      m_axi_rlast,
    output wire                      m_axi_rready,

    // Write Address Channel
    output wire [ADDR_WIDTH-1:0]     m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,
    output wire [2:0]                m_axi_awsize,
    output wire [1:0]                m_axi_awburst,
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,

    // Write Data Channel
    output wire [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,

    // Write Response Channel
    input  wire [1:0]                m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready
);

    // Active-High Reset for internal logic
    wire rst = ~rst_n;

    // CPU <-> Arbiter Signals
    wire        icache_req_valid;
    wire [31:0] icache_req_addr;
    wire        icache_ready;
    wire [127:0] icache_rline;

    wire        dcache_req_valid;
    wire        dcache_req_write;
    wire [31:0] dcache_req_addr;
    wire [127:0] dcache_wline;
    wire        dcache_ready;
    wire [127:0] dcache_rline;

    // mm_accel result DMA -> mem_arbiter third port
    wire        accel_req_valid;
    wire        accel_req_write;
    wire [31:0] accel_req_addr;
    wire [127:0] accel_wline;
    wire        accel_ready;
    wire [127:0] accel_rline;
    wire [7:0]  accel_req_lines;
    wire [7:0]  mem_req_lines;
    wire        accel_wnext;
    wire        mem_wnext;

    // Arbiter <-> AXI Adapter Signals
    wire        mem_req_valid;
    wire        mem_req_write;
    wire [31:0] mem_req_addr;
    wire [127:0] mem_wline;
    wire [127:0] mem_rline;
    wire        mem_ready;

    // 1. CPU Core Instantiation
    cpu_pipelined CPU_CORE (
        .clk(clk),
        .rst(rst),
        .leds(leds),
        .uart_tx(uart_tx),

        // I-Cache bus
        .icache_mem_req_valid(icache_req_valid),
        .icache_mem_req_addr(icache_req_addr),
        .icache_mem_ready(icache_ready),
        .icache_mem_read_data(icache_rline),

        // D-Cache bus
        .dcache_mem_req_valid(dcache_req_valid),
        .dcache_mem_req_write(dcache_req_write),
        .dcache_mem_req_addr(dcache_req_addr),
        .dcache_mem_wline(dcache_wline),
        .dcache_mem_ready(dcache_ready),
        .dcache_mem_read_data_block(dcache_rline),

        .accel_mem_req_valid(accel_req_valid),
        .accel_mem_req_write(accel_req_write),
        .accel_mem_req_addr(accel_req_addr),
        .accel_mem_req_lines(accel_req_lines),
        .accel_mem_wline(accel_wline),
        .accel_mem_ready(accel_ready),
        .accel_mem_wnext(accel_wnext),
        .accel_mem_rline(accel_rline)
    );

    // 2. Memory Arbiter
    mem_arbiter ARBITER (
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
        // Accelerator result-DMA port, driven by mm_accel through cpu.
        .accel_req_valid(accel_req_valid),
        .accel_req_write(accel_req_write),
        .accel_req_addr(accel_req_addr),
        .accel_req_lines(accel_req_lines),
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

    // 3. AXI Cache Adapter
    axi_cache_adapter #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BITS(LINE_BITS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) AXI_ADAPTER (
        .clk(clk),
        .rst(rst),
        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_req_lines(mem_req_lines),
        .mem_wnext(mem_wnext),
        .mem_wline(mem_wline),
        .mem_rline(mem_rline),
        .mem_ready(mem_ready),
        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_rready(m_axi_rready),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready)
    );

endmodule