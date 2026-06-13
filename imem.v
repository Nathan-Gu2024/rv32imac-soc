module imem (
    input  wire [31:0] pc,
    output wire [31:0] inst
);
    reg [31:0] rom [0:16383]; // 16KiB

    assign inst = rom[pc[15:2]]; 
endmodule