module tcm_stub #(
    parameter TCM_BASE  = 32'h4000_0000,
    parameter TCM_BYTES = 65536
)(
    input wire clk,
    input wire req_valid,
    input wire req_write,
    input wire [31:0] req_addr,
    input wire [31:0] req_wdata,
    input wire [3:0]  req_wmask,
    output reg [31:0] resp_rdata,
    output wire resp_ready
);

    reg [7:0] mem [0:TCM_BYTES-1];

    wire [31:0] offset = req_addr - TCM_BASE;

    assign resp_ready = 1'b1;

    always @(posedge clk) begin
        if (req_valid && req_write) begin
            if (req_wmask[0]) mem[offset + 0] <= req_wdata[7:0];
            if (req_wmask[1]) mem[offset + 1] <= req_wdata[15:8];
            if (req_wmask[2]) mem[offset + 2] <= req_wdata[23:16];
            if (req_wmask[3]) mem[offset + 3] <= req_wdata[31:24];
        end

        if (req_valid && !req_write) begin
            resp_rdata <= {
                mem[offset + 3],
                mem[offset + 2],
                mem[offset + 1],
                mem[offset + 0]
            };
        end
    end

endmodule