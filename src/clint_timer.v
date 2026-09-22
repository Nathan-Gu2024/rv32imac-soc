`ifndef _CLINT_TIMER_V_
`define _CLINT_TIMER_V_

module clint_timer (
    input wire clk,
    input wire rst,

    input wire [31:0] addr,
    input wire [31:0] wdata,
    input wire wen,
    output reg [31:0] rdata,

    output wire timer_interrupt
);
    reg [63:0] mtime;
    reg [63:0] mtimecmp;
    assign timer_interrupt = (mtime >= mtimecmp);
    always @(*) begin
        case (addr[3:0])
            4'h0: rdata = mtime[31:0];
            4'h4: rdata = mtime[63:32];
            4'h8: rdata = mtimecmp[31:0];
            4'hC: rdata = mtimecmp[63:32];
            default: rdata = 32'b0;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            mtime <= 64'b0;
            mtimecmp <= 64'h0000_0000_FFFF_FFFF;
        end else begin
            mtime <= mtime + 64'd1;
            if (wen) begin
                case (addr[3:0])
                    4'h0: mtime[31:0] <= wdata;
                    4'h4: mtime[63:32] <= wdata;
                    // Writes the LOW half only, leaving the high half alone -
                    // which is what the register map implies and what software
                    // is entitled to assume.
                    //
                    // This used to also clear mtimecmp[63:32]. Zephyr's
                    // riscv_machine_timer writes hi=-1, lo=new, hi=new, so the
                    // final high write restored what the low write cleared and
                    // the sequence still landed correctly. The exposure was the
                    // MIDDLE state: with the high half zeroed, mtimecmp is only
                    // reliably in the future while mtime[63:32] is still 0 -
                    // about 71 s of uptime at 60 MHz. Past that, a timer
                    // reprogrammed at the wrong moment sees mtime >= mtimecmp,
                    // fires early, and the tick handler re-arms into a storm.
                    //
                    // The old behaviour was documented rather than fixed on the
                    // grounds that it was invisible from the register map. That
                    // is a reason to write it down, not a reason to keep it -
                    // and it cost only this one line to remove.
                    4'h8: mtimecmp[31:0] <= wdata;

                    4'hC: mtimecmp[63:32] <= wdata;
                endcase
            end
        end
    end

endmodule

`endif // _CLINT_TIMER_V_
