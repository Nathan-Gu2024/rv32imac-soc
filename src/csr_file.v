module csr_file (
    input wire clk,
    input wire rst,

    // Software (Zicsr Instructions)
    input wire [11:0] csr_addr, // 12-bit CSR address from the instruction
    input wire [31:0] csr_wdata, 
    input wire csr_wen, // wen from control logic
    output reg [31:0] csr_rdata,
    // Hardware (Trap Controller)
    input wire trap_taken, // High when a hardware interrupt/exception occurs
    input wire [31:0] trap_pc, // pc of the interrupted instruction
    input wire [31:0] trap_cause, // reason for trap
    input wire mret_exec, // High when the 'mret' instruction executes
    // Outputs to the cpu pc mux
    output wire [31:0] mtvec_out, // where to jump on a trap
    output wire [31:0] mepc_out, // where to jump on an 'mret'
    output wire mstatus_mie // live MIE bit, for gating hardware interrupts
);

    // OS Registers
    reg [31:0] mstatus;
    reg [31:0] mtvec;
    reg [31:0] mepc;
    reg [31:0] mcause;

    // Route critical registers continuously to the hardware trap controller
    assign mtvec_out = mtvec;
    assign mepc_out = mepc;
    assign mstatus_mie = mstatus[3];

    // Read Logic (Combinational)
    always @(*) begin
        case (csr_addr)
            12'h300: csr_rdata = mstatus;
            12'h305: csr_rdata = mtvec;
            12'h341: csr_rdata = mepc;
            12'h342: csr_rdata = mcause;
            default: csr_rdata = 32'b0; // Unknown CSR reads as 0
        endcase
    end

    // Write Logic (Sequential)
    always @(posedge clk) begin
        if (rst) begin
            mstatus <= 32'b0;
            mtvec <= 32'b0;
            mepc <= 32'b0;
            mcause <= 32'b0;
        end else begin
            // HW trap handler -> HW automatically overwrites mepc and mcause when a trap fires
            if (trap_taken) begin
                mepc <= trap_pc;
                mcause <= trap_cause;
                // Disable global interrupts (MIE bit is bit 3) by moving it to MPIE (bit 7)
                mstatus[7] <= mstatus[3]; 
                mstatus[3] <= 1'b0;       
            end 
            // (mret) restores the interrupt enable state
            else if (mret_exec) begin
                mstatus[3] <= mstatus[7];
            end 
            // Software Write (csrrw, csrrs, csrrc instructions)
            else if (csr_wen) begin
                case (csr_addr)
                    12'h300: mstatus <= csr_wdata;
                    12'h305: mtvec <= csr_wdata;
                    12'h341: mepc <= csr_wdata;
                    12'h342: mcause <= csr_wdata;
                endcase
            end
        end
    end
endmodule