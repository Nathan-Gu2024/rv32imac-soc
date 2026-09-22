`ifndef _AXI_LITE_BRIDGE_V_
`define _AXI_LITE_BRIDGE_V_

`timescale 1ns/1ps

// Converts the CPU-facing single-shot d_req/d_we/d_addr/d_wdata/d_rdata/
// d_ready handshake (the same interface every other MMIO peripheral in this
// design uses - see uart_mmio.v/intc.v) into a genuine AXI4-Lite MASTER
// transaction. Single outstanding transaction at a time; d_ready pulses
// for exactly one cycle once the full AXI handshake completes (write:
// AW+W then B; read: AR then R), matching the one-shot-ready contract
// cpu.v's generic peripheral stall/pending logic already expects - so no
// changes are needed on the CPU side to tolerate this taking several
// cycles longer than uart/intc's single-cycle ack.
module axi_lite_bridge (
    input wire clk, rst,

    // CPU-facing
    input wire d_req, d_we,
    input wire [31:0] d_addr, d_wdata,
    output reg [31:0] d_rdata,
    output reg d_ready,

    // AXI4-Lite master
    output reg [31:0] m_axi_awaddr,
    output reg m_axi_awvalid,
    input wire m_axi_awready,
    output reg [31:0] m_axi_wdata,
    output reg [3:0] m_axi_wstrb,
    output reg m_axi_wvalid,
    input wire m_axi_wready,
    input wire [1:0] m_axi_bresp,
    input wire m_axi_bvalid,
    output reg m_axi_bready,
    output reg [31:0] m_axi_araddr,
    output reg m_axi_arvalid,
    input wire m_axi_arready,
    input wire [31:0] m_axi_rdata,
    input wire [1:0] m_axi_rresp,
    input wire m_axi_rvalid,
    output reg m_axi_rready
);
    localparam IDLE            = 3'd0,
               WRITE_ADDR_DATA = 3'd1,
               WRITE_RESP      = 3'd2,
               READ_ADDR       = 3'd3,
               READ_DATA       = 3'd4;
    reg [2:0] state;
    // Latch which of AW/W has already handshaken, since AWREADY/WREADY are
    // allowed to fire on different cycles per the AXI4 spec.
    reg aw_done, w_done;

    // Edge-detect d_req rather than level-triggering on it: a caller may
    // legitimately hold d_req high for the whole transaction (not just
    // pulse it), and IDLE is re-entered on the very same cycle d_ready
    // pulses - a level check there would see the still-asserted d_req from
    // the transaction that JUST completed and immediately relaunch a
    // spurious duplicate using the stale d_addr, before the caller gets a
    // chance to change it for the next request.
    reg d_req_prev;
    always @(posedge clk) begin
        if (rst) d_req_prev <= 1'b0;
        else d_req_prev <= d_req;
    end
    wire d_req_edge = d_req && !d_req_prev;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid <= 1'b0;
            m_axi_bready <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready <= 1'b0;
            d_ready <= 1'b0;
            aw_done <= 1'b0;
            w_done <= 1'b0;
        end else begin
            d_ready <= 1'b0; // one-shot pulse: default low unless set below

            case (state)
                IDLE: begin
                    if (d_req_edge && d_we) begin
                        m_axi_awaddr <= d_addr;
                        m_axi_wdata <= d_wdata;
                        m_axi_wstrb <= 4'b1111; // every register here is a full word
                        m_axi_awvalid <= 1'b1;
                        m_axi_wvalid <= 1'b1;
                        aw_done <= 1'b0;
                        w_done <= 1'b0;
                        state <= WRITE_ADDR_DATA;
                    end else if (d_req_edge && !d_we) begin
                        m_axi_araddr <= d_addr;
                        m_axi_arvalid <= 1'b1;
                        state <= READ_ADDR;
                    end
                end

                WRITE_ADDR_DATA: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        aw_done <= 1'b1;
                    end
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        w_done <= 1'b1;
                    end
                    if ((aw_done || (m_axi_awvalid && m_axi_awready)) &&
                        (w_done || (m_axi_wvalid && m_axi_wready))) begin
                        m_axi_bready <= 1'b1; // level signal, ready before BVALID arrives
                        state <= WRITE_RESP;
                    end
                end

                WRITE_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        d_ready <= 1'b1;
                        state <= IDLE;
                    end
                end

                READ_ADDR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready <= 1'b1; // level signal, ready before RVALID arrives
                        state <= READ_DATA;
                    end
                end

                READ_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axi_rready <= 1'b0;
                        d_rdata <= m_axi_rdata;
                        d_ready <= 1'b1;
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule

`endif // _AXI_LITE_BRIDGE_V_
