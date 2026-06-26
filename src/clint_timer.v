module clint_timer (
    input wire clk,
    input wire rst,
    // MMIO Interface
    input wire [31:0] addr,
    input wire [31:0] wdata,
    input wire wen,
    output reg [31:0] rdata,
    // Interrupt Output
    output wire timer_interrupt
);
    reg [63:0] mtime;
    reg [63:0] mtimecmp;
    // The alarm triggers when the current time is greater than or equal to the alarm time
    assign timer_interrupt = (mtime >= mtimecmp);
    // read
    always @(*) begin
        case (addr[3:0])
            4'h0: rdata = mtime[31:0];
            4'h4: rdata = mtime[63:32];
            4'h8: rdata = mtimecmp[31:0];
            4'hC: rdata = mtimecmp[63:32];
            default: rdata = 32'b0;
        endcase
    end

    // write & tick 
    always @(posedge clk) begin
        if (rst) begin
            mtime <= 64'b0;
            mtimecmp <= 64'hFFFF_FFFF_FFFF_FFFF; // Set alarm infinitely far away by default
        end else begin
            // clock ticks forward every cycle
            mtime <= mtime + 1;

            // cpu can write to the registers
            if (wen) begin
                case (addr[3:0])
                    4'h0: mtime[31:0] <= wdata;
                    4'h4: mtime[63:32] <= wdata;
                    4'h8: mtimecmp[31:0] <= wdata;
                    4'hC: mtimecmp[63:32] <= wdata;
                endcase
            end
        end
    end

endmodule