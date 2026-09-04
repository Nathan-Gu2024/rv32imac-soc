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
                    // NON-STANDARD: writing the LOW half also clears the HIGH
                    // half. Documented because it is invisible from the
                    // register map and changes what update sequences are safe.
                    //
                    // Zephyr's riscv_machine_timer writes hi=-1, lo=new,
                    // hi=new. That still lands correctly here - the final high
                    // write restores what the low write cleared:
                    //
                    //   hi=-1   FFFFFFFF_old
                    //   lo=new  00000000_new      <- high cleared by this write
                    //   hi=new  newhi____new
                    //
                    // The exposure is the middle state. With the high half
                    // zeroed, mtimecmp is only reliably in the future while
                    // mtime[63:32] is still 0 - about 71 s of uptime at 60 MHz.
                    // Past that, a timer reprogrammed at the wrong moment can
                    // see mtime >= mtimecmp and fire early. Nothing in the
                    // current bring-up runs that long, which is why this has
                    // not bitten; a long-running Zephyr application could.
                    //
                    // Writing the high half LAST is what makes it safe, so any
                    // new timer code must do the same.
                    4'h8:
                        begin
                            mtimecmp[31:0]  <= wdata;
                            mtimecmp[63:32] <= 32'b0;
                        end

                    4'hC: mtimecmp[63:32] <= wdata;
                endcase
            end
        end
    end

endmodule