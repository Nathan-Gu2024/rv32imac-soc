/* Standalone hardware test for the 2x2 output-stationary systolic
 * matmul accelerator (src/mm_accel.v, driven over a real AXI4-Lite link via
 * src/axi_lite_bridge.v). Computes C = A*B for A=[[3,-2],[5,7]],
 * B=[[4,1],[-3,6]] -> C=[[18,-9],[-1,47]] (same operands as
 * Testbenches/tb_mm_accel.v and test_mm_accel.S), then a second run with
 * A=[[1,2],[3,4]], B=[[5,6],[7,8]] -> C=[[19,22],[43,50]] to exercise the
 * "START must clear stale DONE" fix. Reports PASS/FAIL per check over UART,
 * mirroring fpga/tests/test_amo.c's style.
 */
#include <stdint.h>

#define UART_TX_DATA   *((volatile uint32_t*)0x40001000)
#define UART_TX_STATUS *((volatile uint32_t*)0x40001004)

#define ACCEL_BASE 0x00005000
#define ACCEL_CTRL      *((volatile uint32_t*)(ACCEL_BASE + 0x00))
#define ACCEL_STATUS    *((volatile uint32_t*)(ACCEL_BASE + 0x04))
#define ACCEL_KLEN      *((volatile uint32_t*)(ACCEL_BASE + 0x08))
#define ACCEL_LOAD_IDX  *((volatile uint32_t*)(ACCEL_BASE + 0x0C))
#define ACCEL_A_ROW0    *((volatile uint32_t*)(ACCEL_BASE + 0x10))
#define ACCEL_A_ROW1    *((volatile uint32_t*)(ACCEL_BASE + 0x14))
#define ACCEL_B_COL0    *((volatile uint32_t*)(ACCEL_BASE + 0x18))
#define ACCEL_B_COL1    *((volatile uint32_t*)(ACCEL_BASE + 0x1C))
#define ACCEL_RESULT00  *((volatile uint32_t*)(ACCEL_BASE + 0x20))
#define ACCEL_RESULT01  *((volatile uint32_t*)(ACCEL_BASE + 0x24))
#define ACCEL_RESULT10  *((volatile uint32_t*)(ACCEL_BASE + 0x28))
#define ACCEL_RESULT11  *((volatile uint32_t*)(ACCEL_BASE + 0x2C))

static void uart_putchar(char c) {
    while (UART_TX_STATUS == 0) {}
    UART_TX_DATA = c;
}

static void uart_print(const char *str) {
    while (*str) uart_putchar(*str++);
}

static void uart_print_int32(int32_t v) {
    char buf[12];
    int i = 0;
    uint32_t u;

    if (v < 0) {
        uart_putchar('-');
        u = (uint32_t)(-v);
    } else {
        u = (uint32_t)v;
    }
    if (u == 0) {
        uart_putchar('0');
        return;
    }
    while (u) {
        buf[i++] = '0' + (u % 10);
        u /= 10;
    }
    while (i > 0) uart_putchar(buf[--i]);
}

static int fail_count = 0;

static void check(const char *name, int32_t actual, int32_t expected) {
    if (actual == expected) {
        uart_print("PASS: ");
        uart_print(name);
        uart_print(" = ");
        uart_print_int32(actual);
        uart_print("\r\n");
    } else {
        fail_count++;
        uart_print("FAIL: ");
        uart_print(name);
        uart_print(" actual=");
        uart_print_int32(actual);
        uart_print(" expected=");
        uart_print_int32(expected);
        uart_print("\r\n");
    }
}

/* Loads A/B into the accelerator, runs a K_LEN=2 matmul, and blocks (polling
 * STATUS) until DONE - a real system would do other work while polling, but
 * this test just wants the result. */
static void run_matmul(int8_t a_row0[2], int8_t a_row1[2],
                        int8_t b_col0[2], int8_t b_col1[2]) {
    ACCEL_KLEN = 2;
    for (int k = 0; k < 2; k++) {
        ACCEL_LOAD_IDX = k;
        ACCEL_A_ROW0 = (uint32_t)(uint8_t)a_row0[k];
        ACCEL_A_ROW1 = (uint32_t)(uint8_t)a_row1[k];
        ACCEL_B_COL0 = (uint32_t)(uint8_t)b_col0[k];
        ACCEL_B_COL1 = (uint32_t)(uint8_t)b_col1[k];
    }
    ACCEL_CTRL = 1; // START
    while (!(ACCEL_STATUS & 0x2)) {} // poll DONE
}

int main() {
    uart_print("=== matmul accelerator test start ===\r\n");

    {
        int8_t a_row0[2] = {3, -2};
        int8_t a_row1[2] = {5, 7};
        int8_t b_col0[2] = {4, -3};
        int8_t b_col1[2] = {1, 6};
        run_matmul(a_row0, a_row1, b_col0, b_col1);
        check("C00", (int32_t)ACCEL_RESULT00, 18);
        check("C01", (int32_t)ACCEL_RESULT01, -9);
        check("C10", (int32_t)ACCEL_RESULT10, -1);
        check("C11", (int32_t)ACCEL_RESULT11, 47);
    }

    {
        int8_t a_row0[2] = {1, 2};
        int8_t a_row1[2] = {3, 4};
        int8_t b_col0[2] = {5, 7};
        int8_t b_col1[2] = {6, 8};
        run_matmul(a_row0, a_row1, b_col0, b_col1);
        check("run2_C00", (int32_t)ACCEL_RESULT00, 19);
        check("run2_C01", (int32_t)ACCEL_RESULT01, 22);
        check("run2_C10", (int32_t)ACCEL_RESULT10, 43);
        check("run2_C11", (int32_t)ACCEL_RESULT11, 50);
    }

    if (fail_count == 0) {
        uart_print("=== ALL MATMUL CHECKS PASSED ===\r\n");
    } else {
        uart_print("=== MATMUL FAILURES: ");
        uart_putchar('0' + (fail_count > 9 ? 9 : fail_count));
        uart_print(" ===\r\n");
    }

    while (1) {}
    return 0;
}
