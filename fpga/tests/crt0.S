.section .text.init
.global _start

_start:
    /* 1. Initialize the Stack Pointer (sp / x2) */
    /* The linker script defines __stack_top at the end of the TCM */
    la sp, __stack_top

    /* 1b. Explicitly disable interrupts and point mtvec at the trap reporter.
     * Unlike Zephyr (which sets these up as part of its own boot/arch
     * init), this bare-metal crt0 never touched them before - if mie/
     * mstatus.MIE happened to be left enabled with mtvec at its default
     * (0, i.e. _start itself), any spurious interrupt (e.g. the CLINT
     * timer, which free-runs unconditionally every cycle) would silently
     * trap back to _start and restart the whole boot sequence, which
     * looks exactly like a hang.
     *
     * mtvec used to point straight at `halt`, which was safe but mute. The
     * core now traps on illegal instructions and on misaligned loads and
     * stores, so the cases that reach here are no longer only spurious
     * interrupts - they are program bugs that carry a cause, a PC and a
     * faulting value. Halting without printing them throws away the entire
     * reason mtval was added, and presents as a silent hang. */
    csrw mie, zero
    csrw mstatus, zero
    la t0, trap_report
    csrw mtvec, t0

    /* 2. Clear the .bss section (Zero out uninitialized data) */
    la t0, __bss_start
    la t1, __bss_end
    bgeu t0, t1, 2f       /* If start >= end, there is no bss to clear */
1:
    sw zero, 0(t0)        /* Store 0 to current address */
    addi t0, t0, 4        /* Advance by 4 bytes (1 word) */
    bltu t0, t1, 1b       /* Loop if current address < end address */
2:

    /* 3. Jump to your C code */
    call main

    /* 4. Catch the CPU if main() ever returns */
halt:
    j halt

/* ------------------------------------------------------------------------
 * Trap reporter.
 *
 * Prints  TRAP mcause=... mepc=... mtval=...  on the UART, then halts.
 *
 * Two constraints shape this code:
 *
 *  - It must not touch the stack. A misaligned store through a corrupt sp is
 *    one of the faults that lands here, so sp cannot be assumed usable.
 *    Everything lives in registers.
 *
 *  - It never returns, so it is free to clobber every register, including
 *    the ones the faulting code was using. That is the tradeoff being made
 *    deliberately: the register state is spent to get the message out. If a
 *    future handler needs to resume, it has to save context first - and mepc
 *    is already the faulting instruction, not the one after it, so resuming
 *    is possible once the cause is fixed.
 *
 * mtvec is written in DIRECT mode, so this must be 4-byte aligned.
 * ------------------------------------------------------------------------ */
    .align 2
trap_report:
    la   a0, tr_msg_head
    jal  ra, tr_puts
    csrr a0, mcause
    jal  ra, tr_hex

    la   a0, tr_msg_mepc
    jal  ra, tr_puts
    csrr a0, mepc
    jal  ra, tr_hex

    la   a0, tr_msg_mtval
    jal  ra, tr_puts
    csrr a0, 0x343            /* mtval - spelled numerically so this builds
                               * on toolchains that predate the name */
    jal  ra, tr_hex

    la   a0, tr_msg_tail
    jal  ra, tr_puts
    j    halt

/* a0 = NUL-terminated string. Clobbers t0-t2. Leaf: does not touch ra. */
tr_puts:
    li   t0, 0x40001000       /* UART TX data; status is at +4 */
1:  lbu  t1, 0(a0)
    beqz t1, 3f
2:  lw   t2, 4(t0)            /* status nonzero = TX ready */
    beqz t2, 2b
    sw   t1, 0(t0)
    addi a0, a0, 1
    j    1b
3:  ret

/* a0 = 32-bit value, printed as 8 hex digits, MSB first.
 * Clobbers t0-t4. Leaf: does not touch ra. */
tr_hex:
    li   t0, 0x40001000
    li   t3, 28               /* shift for the top nibble */
1:  srl  t1, a0, t3
    andi t1, t1, 0xF
    li   t4, 9
    bgt  t1, t4, 2f
    addi t1, t1, 48           /* '0' */
    j    3f
2:  addi t1, t1, 87           /* 'a' - 10 */
3:  lw   t2, 4(t0)
    beqz t2, 3b
    sw   t1, 0(t0)
    addi t3, t3, -4
    bgez t3, 1b
    ret

    .section .rodata
tr_msg_head:  .asciz "\r\n*** TRAP mcause="
tr_msg_mepc:  .asciz " mepc="
tr_msg_mtval: .asciz " mtval="
tr_msg_tail:  .asciz "\r\n"
