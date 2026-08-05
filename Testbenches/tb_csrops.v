`include "../src/csr_file.v"

module tb_csrops;
    reg clk = 0, rst;
    reg [11:0] csr_addr;
    reg [31:0] csr_wdata;
    reg csr_wen;
    reg [1:0] csr_op;
    reg csr_use_imm;
    reg [4:0] csr_uimm;
    wire [31:0] csr_rdata;
    reg trap_taken = 0;
    reg [31:0] trap_pc = 0, trap_cause = 0;
    reg mret_exec = 0;
    reg timer_pending = 0, external_pending = 0;
    wire [31:0] mtvec_out, mepc_out;
    wire mstatus_mie, mie_mtie, mie_meie;

    csr_file DUT (
        .clk(clk), .rst(rst),
        .csr_addr(csr_addr), .csr_wdata(csr_wdata), .csr_wen(csr_wen),
        .csr_op(csr_op), .csr_use_imm(csr_use_imm), .csr_uimm(csr_uimm),
        .csr_rdata(csr_rdata),
        .trap_taken(trap_taken), .trap_pc(trap_pc), .trap_cause(trap_cause), .mret_exec(mret_exec),
        .timer_pending(timer_pending), .external_pending(external_pending),
        .mtvec_out(mtvec_out), .mepc_out(mepc_out),
        .mstatus_mie(mstatus_mie), .mie_mtie(mie_mtie), .mie_meie(mie_meie)
    );

    always #5 clk = ~clk;

    task do_write;
        input [11:0] addr;
        input [31:0] wdata;
        input [1:0] op;
        input use_imm;
        input [4:0] uimm;
        begin
            @(negedge clk);
            csr_addr = addr; csr_wdata = wdata; csr_op = op; csr_use_imm = use_imm; csr_uimm = uimm;
            csr_wen = 1;
            @(negedge clk);
            csr_wen = 0;
        end
    endtask

    task check;
        input [255:0] name;
        input [31:0] got, expected;
        begin
            if (got === expected)
                $display("PASS %0s got=%h", name, got);
            else
                $display("FAIL %0s got=%h expected=%h", name, got, expected);
        end
    endtask

    initial begin
        rst = 1; csr_wen = 0; csr_addr=0; csr_wdata=0; csr_op=0; csr_use_imm=0; csr_uimm=0;
        @(negedge clk); @(negedge clk);
        rst = 0;

        // mie: CSRRW then CSRRS then CSRRC
        do_write(12'h304, 32'h00000100, 2'b01, 0, 0); // mie = 0x100
        check("mie after csrrw", DUT.mie, 32'h00000100);

        do_write(12'h304, 32'h00000800, 2'b10, 0, 0); // mie |= 0x800
        check("mie after csrrs", DUT.mie, 32'h00000900);

        do_write(12'h304, 32'h00000100, 2'b11, 0, 0); // mie &= ~0x100
        check("mie after csrrc", DUT.mie, 32'h00000800);

        // mstatus: this is exactly where the original bugs were (typo'd
        // register name + logical-and-instead-of-bitwise for the clear case)
        do_write(12'h300, 32'h000000FF, 2'b01, 0, 0); // mstatus = 0xFF
        check("mstatus after csrrw", DUT.mstatus, 32'h000000FF);

        do_write(12'h300, 32'h00000100, 2'b10, 0, 0); // mstatus |= 0x100
        check("mstatus after csrrs", DUT.mstatus, 32'h000001FF);

        do_write(12'h300, 32'h000000FF, 2'b11, 0, 0); // mstatus &= ~0xFF
        check("mstatus after csrrc", DUT.mstatus, 32'h00000100);

        // Immediate forms (csrrsi/csrrci): uimm=5'b00011=3
        do_write(12'h300, 32'h0, 2'b10, 1, 5'b00011); // mstatus |= 3
        check("mstatus after csrrsi", DUT.mstatus, 32'h00000103);

        do_write(12'h300, 32'h0, 2'b11, 1, 5'b00001); // mstatus &= ~1
        check("mstatus after csrrci", DUT.mstatus, 32'h00000102);

        // mtvec/mepc/mcause sanity (regression: these worked before this change)
        do_write(12'h305, 32'h40000100, 2'b01, 0, 0);
        check("mtvec after csrrw", DUT.mtvec, 32'h40000100);

        $display("DONE");
        $finish;
    end
endmodule
