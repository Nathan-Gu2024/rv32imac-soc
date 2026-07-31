.section .text.init
.global _start

_start:
    /* 1. Initialize the Stack Pointer (sp / x2) */
    /* The linker script defines __stack_top at the end of the TCM */
    la sp, __stack_top

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