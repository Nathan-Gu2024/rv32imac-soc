module regfile (
    input wire [4:0] read_index1, read_index2, write_index,
    input wire [31:0] write_data,
    input wire reg_wen, clk,
    output wire [31:0] read_data1, read_data2
);

    // signals 
    reg [31:0] regs [0:31];
    initial regs[0] = 32'd0;

    genvar i;
    generate 
        for (i = 1; i < 32; i = i + 1) begin
            always @(posedge clk) begin
                if (reg_wen && (write_index == i))
                    regs[i] <= write_data;
            end
        end
    endgenerate

    // Internaal bypass added as well 
    assign read_data1 = (read_index1 == 5'b0) ? 32'b0 :
                        (reg_wen && write_index == read_index1) ? write_data :
                        regs[read_index1];

    assign read_data2 = (read_index2 == 5'b0) ? 32'b0 :
                        (reg_wen && write_index == read_index2) ? write_data :
                        regs[read_index2];

endmodule
