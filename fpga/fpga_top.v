module fpga_top #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BITS = 128,
    parameter AXI_DATA_WIDTH = 64
)(
    input wire clk,
    input wire rst,          

    // Physical UART TX pin - driven directly by cpu_pipelined's internal
    // uart_mmio peripheral (MMIO at 0x4000_1000; see uart_mmio.v).
    output wire uart_tx,

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

    // Internal reset is active-high.
//    wire rst = ~rst_n;

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

    // Arbiter <-> AXI adapter shared line interface
    wire mem_req_valid;
    wire mem_req_write;
    wire [31:0] mem_req_addr;
    wire [127:0] mem_wline;
    wire [127:0] mem_rline;
    wire mem_ready;

    wire [3:0] cpu_leds;
    wire [31:0] debug_pc, debug_instr;

    wire debug_dcache_valid;
    wire debug_dcache_ready;
    wire debug_tcm_d_req;
    wire debug_tcm_d_ready;
    wire debug_global_mem_stall;
    
    cpu_pipelined CPU_CORE (
        .debug_pc(debug_pc),
        .debug_instr(debug_instr), 
        .debug_dcache_valid(debug_dcache_valid),
        .debug_dcache_ready(debug_dcache_ready),
        .debug_tcm_d_req(debug_tcm_d_req), 
        .debug_tcm_d_ready(debug_tcm_d_ready), 
        .debug_global_mem_stall(debug_global_mem_stall), 
        
        
        .clk(clk),
        .rst(rst),

        .uart_tx(uart_tx),
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
        .dcache_mem_ready(dcache_ready)
    );

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

        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline),
        .mem_ready(mem_ready),
        .mem_rline(mem_rline)
    );

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
    reg [26:0] heartbeat; 
    always @(posedge clk) begin
        if (rst) 
            heartbeat <= 27'd0;
        else
            heartbeat <= heartbeat + 1'b1;
    end     
    
    reg seen_dcache_valid;
    reg seen_tcm_d_req;
    reg seen_tcm_d_ready;
    always @(posedge clk) begin
        if (rst) begin
            seen_dcache_valid <= 1'b0;
            seen_tcm_d_req <= 1'b0;
            seen_tcm_d_ready <= 1'b0;
        end else begin
            if (debug_dcache_valid)
                seen_dcache_valid <= 1'b1;
            if (debug_tcm_d_req)
                seen_tcm_d_req <= 1'b1;
            if (debug_tcm_d_ready)
                seen_tcm_d_ready <= 1'b1;
        end
    end
    
    reg [31:0] counter;

    always @(posedge clk) begin
        if (rst)
            counter <= 0;
        else
            counter <= counter + 1;
    end

//    assign leds = counter[27:24];
    
//    assign leds[3] = heartbeat[25];

//    assign leds[2] = seen_dcache_valid;
//    assign leds[1] = seen_tcm_d_req;
//    assign leds[0] = seen_tcm_d_ready;
    
     assign leds[3:0] = cpu_leds[3:0];
//    assign leds[2:0] = debug_instr[2:0];

//    assign leds[2] = debug_pc[4];
//    assign leds[1] = debug_pc[3];
//    assign leds[0] = debug_pc[2];
//    assign leds = 4'b1111;
endmodule
