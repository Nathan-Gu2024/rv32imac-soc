module csr_file (
    input wire clk,
    input wire rst,

    // Software (Zicsr Instructions)
    input wire [11:0] csr_addr, // 12-bit CSR address from the instruction
    input wire [31:0] csr_wdata,
    input wire csr_wen, // wen from control logic

    input wire [1:0] csr_op,
    input wire csr_use_imm,
    input wire [4:0] csr_uimm,

    output reg [31:0] csr_rdata,

    // Hardware (Trap Controller)
    input wire trap_taken, // High when a hardware interrupt/exception occurs
    input wire [31:0] trap_pc, // pc of the interrupted instruction
    input wire [31:0] trap_cause, // reason for trap
    input wire mret_exec, // High when the 'mret' instruction executes

    // Live hardware interrupt-pending state (raw, not enable-gated) -
    // mip must always reflect present truth for these bits, so they're
    // wired straight from the CLINT and the interrupt controller rather
    // than being a software-writable snapshot.
    input wire timer_pending,    // CLINT: mtime >= mtimecmp
    input wire external_pending, // intc: any enabled peripheral source pending

    // Outputs to the cpu pc mux
    output wire [31:0] mtvec_out, // where to jump on a trap
    output wire [31:0] mepc_out, // where to jump on an 'mret'
    output wire mstatus_mie, // live MIE bit, for gating hardware interrupts

    // Per-class enable bits (mie), for the CPU's interrupt-priority mux
    output wire mie_mtie, // machine timer interrupt enable
    output wire mie_meie  // machine external interrupt enable
);

    // OS Registers
    reg [31:0] mstatus;
    reg [31:0] mtvec;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mie;

    // Route critical registers continuously to the hardware trap controller
    assign mtvec_out = mtvec;
    assign mepc_out = mepc;
    assign mstatus_mie = mstatus[3];
    assign mie_mtie = mie[7];
    assign mie_meie = mie[11];

    // mip is computed, not stored: bit 7 (MTIP) and bit 11 (MEIP) always
    // mirror live hardware state. Software-pending bits (e.g. MSIP) aren't
    // implemented, so the rest of the register reads as 0.
    wire [31:0] mip = {20'b0, external_pending, 3'b0, timer_pending, 7'b0};
    
    wire [31:0] actual_wdata = csr_use_imm ? {27'b0, csr_uimm} : csr_wdata;
    // Read Logic (Combinational)
    always @(*) begin
        case (csr_addr)
            12'h300: csr_rdata = mstatus;
            12'h304: csr_rdata = mie;
            12'h305: csr_rdata = mtvec;
            12'h341: csr_rdata = mepc;
            12'h342: csr_rdata = mcause;
            12'h344: csr_rdata = mip;
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
            mie <= 32'b0;
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
                    12'h300:
                        mstatus <= (csr_op == 2'b01) ? actual_wdata :
                                    (csr_op == 2'b10) ? (mstatus | actual_wdata) :
                                    (mstatus & ~actual_wdata);
                    12'h304: 
                        mie <= (csr_op == 2'b01) ? actual_wdata : 
                                    (csr_op == 2'b10) ? (mie | actual_wdata) : 
                                    (mie & ~actual_wdata);
                                                            
                    12'h305: 
                        mtvec <= (csr_op == 2'b01) ? actual_wdata : 
                                    (csr_op == 2'b10) ? (mtvec | actual_wdata) : 
                                    (mtvec & ~actual_wdata);
                                                            
                    12'h341: 
                        mepc <= (csr_op == 2'b01) ? actual_wdata : 
                                    (csr_op == 2'b10) ? (mepc | actual_wdata) : 
                                    (mepc & ~actual_wdata);
                                                            
                    12'h342: 
                        mcause <= (csr_op == 2'b01) ? actual_wdata : 
                                    (csr_op == 2'b10) ? (mcause | actual_wdata) : 
                                    (mcause & ~actual_wdata);
                endcase
            end
        end
    end
endmodule
