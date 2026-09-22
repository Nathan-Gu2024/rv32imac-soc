.equ UART_BASE,  0x40001000
.equ TX_DATA,    0x000
.equ TX_STATUS,  0x004
.equ RX_DATA,    0x008
.equ RX_STATUS,  0x00C

.equ INTC_BASE,    0x00004000
.equ INTC_ENABLE,  0x000
.equ INTC_PENDING, 0x004
.equ INTC_SRC_UART_TX, 0x1  # source 0
.equ INTC_SRC_UART_RX, 0x2  # source 1

.equ MIE_BIT,    0x8        # mstatus.MIE is bit 3
.equ MEIE_BIT,   0x800      # mie.MEIE is bit 11

.section .text
.globl _start

_start:
    # 1. Setup trap vector
    la t0, isr
    csrw mtvec, t0

    # 2. Enable the RX source at the interrupt controller. Without this,
    # intc's aggregate line never asserts no matter what mie/mstatus say -
    # its own ENABLE mask resets to 0.
    li t0, INTC_BASE
    li t1, INTC_SRC_UART_RX
    sw t1, INTC_ENABLE(t0)

    # 3. Enable MEIE (Machine External Interrupt Enable) in mie
    li t0, MEIE_BIT
    csrs mie, t0     # Note: csrs is the pseudo-op for csrrs

    # 4. Enable Global Interrupts (MIE) in mstatus
    li t0, MIE_BIT
    csrs mstatus, t0

wait_loop:
    # Idle, waiting for PuTTY input
    j wait_loop

.align 4
isr:
    # 1. Check mcause (External interrupts are usually 0x8000000B)
    csrr t0, mcause
    li t1, 0x8000000B
    bne t0, t1, isr_end

    # --- ARCH_IRQ_LOCK ---
    # Clear MIE bit in mstatus using csrrc.
    # The OLD value of mstatus is safely saved into t0.
    li t1, MIE_BIT
    csrrc t0, mstatus, t1

    # --- CRITICAL SECTION ---
    # Read the byte from RX_DATA
    li t1, UART_BASE
    lw t2, RX_DATA(t1)

    # Echo it back to TX_DATA
    sw t2, TX_DATA(t1)

    # Acknowledge the source at the interrupt controller (write-1-to-clear).
    # Without this the pending bit stays latched, and the instant mstatus.MIE
    # is restored below the same condition retriggers immediately - an
    # infinite interrupt storm that never lets wait_loop run again.
    li t1, INTC_BASE
    li t3, INTC_SRC_UART_RX
    sw t3, INTC_PENDING(t1)
    # --- END CRITICAL SECTION ---

    # --- ARCH_IRQ_UNLOCK ---
    # Check if MIE was originally enabled before we locked it.
    andi t0, t0, MIE_BIT
    beqz t0, skip_unlock

    # It was enabled, so restore it using csrrs
    li t1, MIE_BIT
    csrrs zero, mstatus, t1
skip_unlock:

isr_end:
    mret
