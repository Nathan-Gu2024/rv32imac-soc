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