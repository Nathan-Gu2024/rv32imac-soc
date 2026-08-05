.section .text.init
.global _start

_start:
    # Set trap vector to our handler
    la t0, trap_handler
    csrrw x0, mtvec, t0

    # Enable INTC source 0 (UART TX-complete) via INTC_ENABLE (0x0000_4000)
    li t0, 0x00004000
    li t1, 1
    sw t1, 0(t0)

    # Enable mie.MEIE (bit 11) - listen for external interrupts
    li t0, 0x800
    csrrw x0, mie, t0

    # Send a UART character - once it finishes transmitting, uart_mmio's
    # irq line pulses, intc latches it, and (once MIE below is set) the
    # CPU should trap into trap_handler.
    li t0, 0x40001000
    li t1, 65 # 'A'
    sw t1, 0(t0)

    # Enable mstatus.MIE (bit 3) - arms the CPU to actually take the trap
    li t0, 0x8
    csrrw x0, mstatus, t0

wait_loop:
    la t3, marker
    lw t2, 0(t3)
    beqz t2, wait_loop

    # ISR ran - show it landed by writing the captured mcause onto the LEDs
    li t0, 0x2000
    sw t2, 0(t0)

halt:
    j halt

.align 2
trap_handler:
    # Only a single CSRRW-form CSR op is supported right now, so this reads
    # mcause via swap-with-x0 (clobbers mcause with 0, which is fine - this
    # test never needs it again before mret).
    csrrw t0, mcause, x0

    la t1, marker
    sw t0, 0(t1)

    # Acknowledge the INTC source: write-1-to-clear bit 0 in INTC_PENDING
    li t0, 0x00004004
    li t1, 1
    sw t1, 0(t0)

    mret

.section .data
.align 2
marker:
    .word 0
