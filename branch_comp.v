module branch_comp(
    input wire [31:0] br_data1, br_data2,
    input wire br_un,
    output reg br_eq, br_lt
);

    wire signed_lt = $signed(br_data1) < $signed(br_data2);
    wire unsigned_lt = br_data1 < br_data2;

    always @(*) begin
        br_lt = br_un ? unsigned_lt : signed_lt;
        br_eq = (br_data1 == br_data2);
    end

endmodule