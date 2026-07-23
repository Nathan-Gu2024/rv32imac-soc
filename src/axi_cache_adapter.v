module axi_cache_adapter #(
    parameter ADDR_WIDTH     = 32,
    parameter LINE_BITS      = 128,
    parameter AXI_DATA_WIDTH = 64
)(
    input  wire clk,
    input  wire rst,

    // cache-line side
    input  wire                  mem_req_valid,
    input  wire                  mem_req_write,
    input  wire [ADDR_WIDTH-1:0] mem_req_addr,
    input  wire [LINE_BITS-1:0]  mem_wline,
    output reg  [LINE_BITS-1:0]  mem_rline,
    output reg                   mem_ready,

    // AXI read address
    output reg  [ADDR_WIDTH-1:0] m_axi_araddr,
    output reg  [7:0]            m_axi_arlen,
    output reg  [2:0]            m_axi_arsize,
    output reg  [1:0]            m_axi_arburst,
    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,

    // AXI read data
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                     m_axi_rvalid,
    input  wire                     m_axi_rlast,
    output reg                      m_axi_rready,

    // AXI write address
    output reg  [ADDR_WIDTH-1:0] m_axi_awaddr,
    output reg  [7:0]            m_axi_awlen,
    output reg  [2:0]            m_axi_awsize,
    output reg  [1:0]            m_axi_awburst,
    output reg                   m_axi_awvalid,
    input  wire                  m_axi_awready,

    // AXI write data
    output reg  [AXI_DATA_WIDTH-1:0]   m_axi_wdata,
    output reg  [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg                         m_axi_wlast,
    output reg                         m_axi_wvalid,
    input  wire                        m_axi_wready,

    // AXI write response
    input  wire [1:0] m_axi_bresp,
    input  wire       m_axi_bvalid,
    output reg        m_axi_bready
);


endmodule