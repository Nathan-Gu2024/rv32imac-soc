// Minimal interrupt controller: aggregates NUM_SOURCES external interrupt
// lines (peripherals - UART today, more later) into the single external/
// "machine external interrupt" line the CPU's mip.MEIP bit expects, the
// same role a real PLIC plays, just with no priority levels or per-hart
// context - one shared enable mask and one shared pending register.
//
// Each source's raw irq_in line can be a level or a pulse - it's
// edge-detected here (0->1) and latched into `pending`, so a source only
// has to pulse once to guarantee the CPU sees it even if the pulse doesn't
// line up with when software gets around to reading the register.
//
// Register map (word-addressed, matching the d_req/d_we/d_addr/d_wdata
// bus style every other MMIO peripheral on this core uses):
//   BASE+0x0  ENABLE   RW  bit i: 1 = source i can assert the aggregate line
//   BASE+0x4  PENDING  RW  bit i: 1 = source i has an unacknowledged edge
//                          read returns latched pending bits
//                          write: bits written 1 CLEAR the corresponding
//                          pending bit (write-1-to-clear); a source that's
//                          still asserting its raw line at the moment of
//                          the clear will simply re-latch on its next edge
module intc #(
    parameter NUM_SOURCES = 8
) (
    input wire clk,
    input wire rst,

    // CPU data bus interface
    input wire d_req,
    input wire d_we,
    input wire [31:0] d_addr,
    input wire [31:0] d_wdata,
    output reg [31:0] d_rdata,
    output reg d_ready,

    // Peripheral interrupt request lines
    input wire [NUM_SOURCES-1:0] irq_in,

    // Aggregate line feeding mip.MEIP: high whenever any enabled source is pending
    output wire irq_out
);
    reg [NUM_SOURCES-1:0] enable;
    reg [NUM_SOURCES-1:0] pending;
    reg [NUM_SOURCES-1:0] irq_in_prev;

    assign irq_out = |(pending & enable);

    wire [NUM_SOURCES-1:0] irq_edge = irq_in & ~irq_in_prev;

    wire is_intc_addr   = (d_addr[31:12] == 20'h00004); // 0x0000_4XXX
    wire is_enable_reg  = (d_addr[11:0] == 12'h000);    // 0x0000_4000
    wire is_pending_reg = (d_addr[11:0] == 12'h004);    // 0x0000_4004

    always @(posedge clk) begin
        if (rst) begin
            enable <= {NUM_SOURCES{1'b0}};
            pending <= {NUM_SOURCES{1'b0}};
            irq_in_prev <= {NUM_SOURCES{1'b0}};
            d_ready <= 1'b0;
            d_rdata <= 32'b0;
        end else begin
            irq_in_prev <= irq_in;
            d_ready <= 1'b0;
            d_rdata <= 32'b0;

            // New edges latch every cycle regardless of bus activity, so a
            // source firing while software is elsewhere is never missed.
            pending <= pending | irq_edge;

            if (d_req && is_intc_addr) begin
                d_ready <= 1'b1;

                if (d_we && is_enable_reg) begin
                    enable <= d_wdata[NUM_SOURCES-1:0];
                end else if (d_we && is_pending_reg) begin
                    // Write-1-to-clear, combined with this same cycle's new
                    // edges so a source firing exactly as software clears
                    // it isn't silently dropped.
                    pending <= (pending | irq_edge) & ~d_wdata[NUM_SOURCES-1:0];
                end else if (!d_we && is_enable_reg) begin
                    d_rdata <= {{(32-NUM_SOURCES){1'b0}}, enable};
                end else if (!d_we && is_pending_reg) begin
                    d_rdata <= {{(32-NUM_SOURCES){1'b0}}, pending};
                end
            end
        end
    end
endmodule
