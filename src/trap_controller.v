module trap_controller (
    input wire [31:0] ex_pc, // pc of the instruction in EX
    input wire [31:0] ex_inst, // raw instruction currently in EX
    input wire external_interrupt, // A pin from the outside world 
    // from the CSR
    input wire [31:0] mtvec_out, // OS Kernel Address
    input wire [31:0] mepc_out, // Saved Return Address
    // to the CSR
    output reg trap_taken,
    output reg [31:0] trap_cause,
    output reg [31:0] trap_pc,
    output wire mret_exec,
    // to the pipeline (takeover)
    output reg flush_if, // Flush IF/ID register
    output reg flush_id, // Flush ID/EX register
    output reg flush_ex, // Flush EX/MEM register
    output reg pc_trap_override, // force global pc to jump
    output reg [31:0] trap_target_pc // address to jump to
);
    wire is_system = (ex_inst[6:0] == 7'b1110011);
    // ECALL: 12'b000000000000 in the top 12 bits
    wire is_ecall = is_system && (ex_inst[31:20] == 12'b000000000000);
    // MRET: 12'b001100000010 in the top 12 bits
    wire is_mret = is_system && (ex_inst[31:20] == 12'b001100000010);
    assign mret_exec = is_mret;
    always @(*) begin
        // Default
        trap_taken = 1'b0;
        trap_cause = 32'b0;
        trap_pc = 32'b0;
        flush_if = 1'b0;
        flush_id = 1'b0;
        flush_ex = 1'b0;
        pc_trap_override = 1'b0;
        trap_target_pc = 32'b0;
        // HW interrupts cpu
        if (external_interrupt) begin
            trap_taken = 1'b1;
            trap_cause = 32'd7; // RISC-V code for Machine Timer Interrupt
            trap_pc = ex_pc; // Save the PC so we can resume later
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1; // Nuke the pipeline
            pc_trap_override = 1'b1;
            trap_target_pc = mtvec_out; // Jump to OS Kernel
        end else if (is_ecall) begin // Software ASKED for interrupt (ECALL)
            trap_taken = 1'b1;
            trap_cause = 32'd11; // Environment Call from M-Mode
            trap_pc = ex_pc; 
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1;
            pc_trap_override = 1'b1;
            trap_target_pc = mtvec_out; 
        end else if (is_mret) begin // Software RETURNING from interrupt (MRET)
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1; // flush
            pc_trap_override = 1'b1;
            trap_target_pc = mepc_out; // Jump back to where we were before the trap
        end
    end
endmodule