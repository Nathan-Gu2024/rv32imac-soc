/* Standalone hardware test for the full RV32A AMO implementation
 * (AMOSWAP/ADD/XOR/AND/OR/MIN/MAX/MINU/MAXU). Each op is checked against a
 * freshly-reset memory word, verifying both the OLD value returned to rd and
 * the NEW value actually committed to memory (via a follow-up plain load).
 * Reports PASS/FAIL per check over UART, mirroring the style already
 * established in fpga/tests/uart.c.
 *
 * Built with -march=rv32imac_zicsr (adding the 'a' extension just for this
 * test binary - the main project build intentionally omits it since AMO
 * support didn't exist until now).
 */
#include <stdint.h>

#define UART_TX_DATA   *((volatile uint32_t*)0x40001000)
#define UART_TX_STATUS *((volatile uint32_t*)0x40001004)

static void uart_putchar(char c) {
    while (UART_TX_STATUS == 0) {}
    UART_TX_DATA = c;
}

static void uart_print(const char *str) {
    while (*str) uart_putchar(*str++);
}

static void uart_print_hex32(uint32_t v) {
    static const char digits[] = "0123456789abcdef";
    uart_print("0x");
    for (int i = 7; i >= 0; i--) {
        uart_putchar(digits[(v >> (i * 4)) & 0xF]);
    }
}

static int fail_count = 0;

static void check(const char *name, uint32_t actual, uint32_t expected) {
    if (actual == expected) {
        uart_print("PASS: ");
        uart_print(name);
        uart_print("\r\n");
    } else {
        fail_count++;
        uart_print("FAIL: ");
        uart_print(name);
        uart_print(" actual=");
        uart_print_hex32(actual);
        uart_print(" expected=");
        uart_print_hex32(expected);
        uart_print("\r\n");
    }
}

/* Force each op through a real memory word, not a register-only shortcut -
 * "mem" is volatile so the compiler can't reorder/elide these around the
 * inline asm's own memory clobber. */
static volatile uint32_t test_word;

int main() {
    uart_print("=== AMO test start ===\r\n");
    uint32_t old, cur;

    // AMOSWAP: reset=10, operand=99 -> old=10, mem=99
    test_word = 10;
    __asm__ volatile("amoswap.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(99) : "memory");
    cur = test_word;
    check("swap_old", old, 10);
    check("swap_new", cur, 99);

    // AMOADD: reset=10, operand=5 -> old=10, mem=15
    test_word = 10;
    __asm__ volatile("amoadd.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(5) : "memory");
    cur = test_word;
    check("add_old", old, 10);
    check("add_new", cur, 15);

    // AMOXOR: reset=10(0xA), operand=6(0x6) -> old=10, mem=12(0xC)
    test_word = 10;
    __asm__ volatile("amoxor.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(6) : "memory");
    cur = test_word;
    check("xor_old", old, 10);
    check("xor_new", cur, 12);

    // AMOAND: reset=12(0xC), operand=10(0xA) -> old=12, mem=8(0x8)
    test_word = 12;
    __asm__ volatile("amoand.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(10) : "memory");
    cur = test_word;
    check("and_old", old, 12);
    check("and_new", cur, 8);

    // AMOOR: reset=8(0x8), operand=3(0x3) -> old=8, mem=11(0xB)
    test_word = 8;
    __asm__ volatile("amoor.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(3) : "memory");
    cur = test_word;
    check("or_old", old, 8);
    check("or_new", cur, 11);

    // AMOMIN (signed): reset=-5, operand=3 -> old=-5, mem=-5
    test_word = (uint32_t)-5;
    __asm__ volatile("amomin.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(3) : "memory");
    cur = test_word;
    check("min_old", old, (uint32_t)-5);
    check("min_new", cur, (uint32_t)-5);

    // AMOMAX (signed): reset=-5, operand=3 -> old=-5, mem=3
    test_word = (uint32_t)-5;
    __asm__ volatile("amomax.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(3) : "memory");
    cur = test_word;
    check("max_old", old, (uint32_t)-5);
    check("max_new", cur, 3);

    // AMOMINU (unsigned): reset=20, operand=7 -> old=20, mem=7
    test_word = 20;
    __asm__ volatile("amominu.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(7) : "memory");
    cur = test_word;
    check("minu_old", old, 20);
    check("minu_new", cur, 7);

    // AMOMAXU (unsigned): reset=20, operand=7 -> old=20, mem=20
    test_word = 20;
    __asm__ volatile("amomaxu.w %0, %2, (%1)" : "=r"(old) : "r"(&test_word), "r"(7) : "memory");
    cur = test_word;
    check("maxu_old", old, 20);
    check("maxu_new", cur, 20);

    if (fail_count == 0) {
        uart_print("=== ALL AMO CHECKS PASSED ===\r\n");
    } else {
        uart_print("=== AMO FAILURES: ");
        uart_putchar('0' + (fail_count > 9 ? 9 : fail_count));
        uart_print(" ===\r\n");
    }

    while (1) {}
    return 0;
}
