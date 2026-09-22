/* Deliberately trap, and prove the reporter in crt0.S prints something useful.
 *
 * Two things are under test at once:
 *
 *  1. crt0.S's trap reporter. mtvec used to point at `halt: j halt`, so any
 *     trap was a silent hang. It now prints mcause/mepc/mtval over the UART.
 *     That path has never executed on hardware.
 *
 *  2. The misaligned-load fix. The load below is followed by an instruction
 *     that CONSUMES its result, which raises the load-use interlock on the same
 *     cycle the fault wants to fire. trap_controller was being fed
 *     `global_mem_stall | stall`, so the fault was suppressed on the only cycle
 *     it could fire and then discarded with the flushed instruction - the trap
 *     was lost and the load silently returned the wrong halfword.
 *
 *     Written in inline asm rather than C because the dependent instruction has
 *     to stay ADJACENT: at -O2 the compiler is free to schedule something
 *     between them, which is exactly the case that still worked before the fix.
 *
 * Expected on a working build:
 *
 *     trap test: issuing a misaligned load with a dependent use
 *     *** TRAP mcause=00000004 mepc=00100xxx mtval=00180002
 *
 * mcause 4 = load address misaligned. mtval = the offending ADDRESS.
 * Reaching "FAIL" means no trap fired.
 */

#include <stdint.h>

#define UART_TX_DATA   *((volatile uint32_t *)0x40001000)
#define UART_TX_STATUS *((volatile uint32_t *)0x40001004)

static void uart_putchar(char c)
{
    while (UART_TX_STATUS == 0) { }
    UART_TX_DATA = c;
}

static void uart_print(const char *s)
{
    while (*s) uart_putchar(*s++);
}

int main(void)
{
    uart_print("trap test: issuing a misaligned load with a dependent use\r\n");

    /* 0x00180002 is word-misaligned (bit 1 set) and inside DDR, so the fault is
     * the alignment check rather than anything about the address being invalid.
     */
    __asm__ volatile (
        "li   t0, 0x00180002\n"   /* misaligned word address        */
        "lw   t1, 0(t0)\n"        /* -> mcause 4, mtval = 0x180002  */
        "addi t2, t1, 1\n"        /* DEPENDENT: raises the interlock */
        ::: "t0", "t1", "t2"
    );

    /* Only reachable if the trap did not fire. */
    uart_print("FAIL: no trap - misaligned load was silently accepted\r\n");
    for (;;) { }
    return 0;
}
