.section .text.init
.global _start

_start:
    /* 1. Initialize the Stack Pointer (sp / x2) */
    /* The linker script defines __stack_top at the end of the TCM */
    la sp, __stack_top

    /* 1b. Explicitly disable interrupts and point mtvec somewhere safe.
     * Unlike Zephyr (which sets these up as part of its own boot/arch
     * init), this bare-metal crt0 never touched them before - if mie/
     * mstatus.MIE happened to be left enabled with mtvec at its default
     * (0, i.e. _start itself), any spurious interrupt (e.g. the CLINT
     * timer, which free-runs unconditionally every cycle) would silently
     * trap back to _start and restart the whole boot sequence, which
     * looks exactly like a hang. */
    csrw mie, zero
    csrw mstatus, zero
    la t0, halt
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