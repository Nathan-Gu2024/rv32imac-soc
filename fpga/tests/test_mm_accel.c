/* Hardware test for the Chisel-generated systolic matmul accelerator
 * (src/mm_accel.v, driven over a real AXI4-Lite link via src/axi_lite_bridge.v),
 * including the result DMA that leaves over mem_arbiter's 128-bit line port.
 *
 * REWRITTEN for the indexed register map. The previous version drove the
 * hand-written 2x2 core's fixed-word map - a separate MMIO word per operand
 * lane and per accumulator, which is 2*DIM + DIM*DIM words and stops fitting a
 * 64-word window well before DIM=16. The map below is CONSTANT SIZE in DIM.
 *
 * The array size is DISCOVERED from the INFO register rather than hardcoded,
 * so this same binary drives any build the generator emits. That matters here
 * because dim is fixed at Chisel elaboration, not by a Verilog parameter - a
 * driver that assumed 2x2 would silently compute garbage on a DIM=8 build.
 */
#include <stdint.h>

#define UART_TX_DATA   *((volatile uint32_t*)0x40001000)
#define UART_TX_STATUS *((volatile uint32_t*)0x40001004)

#define ACCEL_BASE 0x00005000
#define ACCEL_REG(w) (*((volatile uint32_t*)(ACCEL_BASE + 4*(w))))

#define ACCEL_CTRL        ACCEL_REG(0)
#define ACCEL_STATUS      ACCEL_REG(1)
#define ACCEL_KLEN        ACCEL_REG(2)
#define ACCEL_LOAD_K      ACCEL_REG(3)
#define ACCEL_LOAD_LANE   ACCEL_REG(4)
#define ACCEL_A_PUSH      ACCEL_REG(5)
#define ACCEL_B_PUSH      ACCEL_REG(6)
#define ACCEL_RESULT_IDX  ACCEL_REG(7)
#define ACCEL_RESULT      ACCEL_REG(8)
#define ACCEL_INFO        ACCEL_REG(9)
#define ACCEL_DEST_ADDR   ACCEL_REG(10)
#define ACCEL_DEST_STRIDE ACCEL_REG(11)

#define CTRL_START     0x1
#define CTRL_SOFT_RST  0x2
#define CTRL_START_DMA 0x4

#define ST_BUSY     0x1
#define ST_DONE     0x2
#define ST_DMA_BUSY 0x4
#define ST_DMA_DONE 0x8

/* Where the DMA drops results. Must be 16-byte aligned. */
#define RESULT_BUF 0x00180000

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
    while (i) uart_putchar(buf[--i]);
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

/* Load one panel. LOAD_LANE is written once: LOAD_K auto-increments per push
 * and LOAD_LANE auto-advances when a lane's k-groups are exhausted, so a whole
 * dim x klen panel is one index write plus dim*(klen/4) pushes. Per-lane index
 * writes used to be a third of operand traffic at klen=8.
 *
 * val[lane] is replicated across all klen entries of that lane, which keeps
 * this driver short while still giving results that depend on both indices.
 */
static void push_panel(volatile uint32_t *port, const uint8_t *val,
                       int dim, int klen) {
    ACCEL_LOAD_LANE = 0;
    for (int lane = 0; lane < dim; lane++) {
        uint32_t w = ((uint32_t)val[lane]) * 0x01010101u;
        for (int g = 0; g < klen / 4; g++) *port = w;
    }
}

int main() {
    uart_print("=== matmul accelerator test start ===\r\n");

    uint32_t info = ACCEL_INFO;
    int dim  = info & 0xFF;
    int maxk = (info >> 8) & 0xFF;

    uart_print("geometry: dim=");
    uart_print_int32(dim);
    uart_print(" maxK=");
    uart_print_int32(maxk);
    uart_print("\r\n");

    if (dim < 2 || dim > 16 || (dim & (dim - 1)) != 0) {
        uart_print("=== BAD GEOMETRY, ABORTING ===\r\n");
        while (1) {}
    }

    /* A[i][k] = i+1, B[j][k] = j+1, klen = 4  ->  C[i][j] = 4*(i+1)*(j+1).
     * Every expected value depends on both indices, so a transposed array or
     * a collapsed row cannot pass by accident. */
    const int klen = 4;
    uint8_t va[16], vb[16];
    for (int i = 0; i < dim; i++) { va[i] = (uint8_t)(i + 1); vb[i] = (uint8_t)(i + 1); }

    ACCEL_KLEN = klen;
    push_panel(&ACCEL_A_PUSH, va, dim, klen);
    push_panel(&ACCEL_B_PUSH, vb, dim, klen);

    ACCEL_CTRL = CTRL_START;
    while (!(ACCEL_STATUS & ST_DONE)) {}

    /* Spot-check through the register window. RESULT_IDX auto-increments on
     * each RESULT read, so a full sweep is one index write plus dim*dim reads
     * rather than two transactions per element. */
    ACCEL_RESULT_IDX = 0;
    int32_t c00 = (int32_t)ACCEL_RESULT;         /* idx 0 */
    ACCEL_RESULT_IDX = dim + 1;                  /* C[1][1] */
    int32_t c11 = (int32_t)ACCEL_RESULT;
    ACCEL_RESULT_IDX = dim * dim - 1;            /* C[dim-1][dim-1] */
    int32_t clast = (int32_t)ACCEL_RESULT;

    check("C00",   c00,   4 * 1 * 1);
    check("C11",   c11,   4 * 2 * 2);
    check("Clast", clast, 4 * dim * dim);

    /* ---- result DMA ----
     * STRIDE = one tile row (dim*4 bytes) writes the tile contiguously. Set it
     * to a larger matrix's row pitch instead and the tile lands directly in
     * place inside that matrix, with no software copy - which is the whole
     * reason the stride is programmable.
     */
    /* COHERENCE: the DMA writes DRAM through mem_arbiter, BEHIND the D-cache.
     * This SoC has no cache flush or invalidate, and no uncached DRAM window -
     * the MMIO decodes are device windows only. So any line of RESULT_BUF that
     * is already in the D-cache when the DMA runs will be read back STALE.
     *
     * This is not hypothetical: pre-zeroing this buffer (the obvious way to
     * prove the DMA really wrote it) put 64 zeroed lines in the cache and made
     * every single result read back as 0, while the accumulators themselves
     * were provably correct through the register window.
     *
     * RESULT_BUF is therefore left untouched before the DMA, so the reads below
     * miss and fetch the DMA-written data from DRAM. That is a real constraint
     * on this hardware, not a trick to make the test pass: software must treat
     * a DMA destination as untouched-then-read, and anything more general needs
     * either cache maintenance or an uncached window. Note that OpenGeMM sidesteps
     * this entirely by having the accelerator write a tightly-coupled scratchpad
     * the core reads directly, rather than cacheable DRAM.
     *
     * The expected values (4*(i+1)*(j+1), ranging 4..256) are distinctive
     * enough that uninitialised or stale memory cannot match by chance.
     */
    volatile int32_t *buf = (volatile int32_t *)RESULT_BUF;

    ACCEL_DEST_ADDR   = RESULT_BUF;
    ACCEL_DEST_STRIDE = dim * 4;
    ACCEL_CTRL        = CTRL_START_DMA;
    while (!(ACCEL_STATUS & ST_DMA_DONE)) {}

    int dma_bad = 0;
    for (int i = 0; i < dim; i++)
        for (int j = 0; j < dim; j++)
            if (buf[i * dim + j] != 4 * (i + 1) * (j + 1)) dma_bad++;

    if (dma_bad == 0) {
        uart_print("PASS: DMA wrote all ");
        uart_print_int32(dim * dim);
        uart_print(" results\r\n");
    } else {
        fail_count++;
        uart_print("FAIL: DMA mismatches=");
        uart_print_int32(dma_bad);
        uart_print("\r\n");
    }

    if (fail_count == 0) {
        uart_print("=== ALL MATMUL CHECKS PASSED ===\r\n");
    } else {
        uart_print("=== MATMUL FAILURES: ");
        uart_print_int32(fail_count);
        uart_print(" ===\r\n");
    }

    while (1) {}
    return 0;
}
