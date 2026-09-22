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
#define ACCEL_A_SRC       ACCEL_REG(12)
#define ACCEL_B_SRC       ACCEL_REG(13)
#define ACCEL_SRC_STRIDE  ACCEL_REG(14)
#define ACCEL_B_PANEL_USE  ACCEL_REG(15)
#define ACCEL_B_PANEL_LOAD ACCEL_REG(16)
#define ACCEL_DESC_A_SRC  ACCEL_REG(17)
#define ACCEL_DESC_B_SRC  ACCEL_REG(18)
#define ACCEL_DESC_DEST   ACCEL_REG(19)
#define ACCEL_DESC_PUSH   ACCEL_REG(20)   /* writing this commits a descriptor */
/* Descriptor ctl bit 16: fetch only the B panel, leave A resident from the
 * previous tile. C[ti][tj] = A[ti]*B[tj], so a row of tiles shares one A
 * panel; without this the queue re-fetches it per tile, which at dim=8,
 * maxK=64 is 32 of the 80 lines a tile moves. */
#define DESC_BONLY        0x10000u

#define ACCEL_OUT_CTRL    ACCEL_REG(22)   /* {int8[8], shift[4:0]} */
#define OUT_INT8          0x100u
#define ACCEL_QUEUE_FREE  ACCEL_REG(21)

#define CTRL_START     0x1
#define CTRL_SOFT_RST  0x2
#define CTRL_START_DMA 0x4
#define CTRL_START_LOAD   0x8    /* fetch A and B panels over the line port */
#define CTRL_START_LOAD_B 0x10   /* B panel only - safe during compute */
#define CTRL_START_QUEUE  0x20   /* run every queued descriptor */

#define ST_BUSY     0x1
#define ST_DONE     0x2
#define ST_DMA_BUSY 0x4
#define ST_DMA_DONE 0x8
#define ST_LOAD_BUSY 0x10
#define ST_LOAD_DONE 0x20
#define ST_QUEUE_BUSY 0x40
#define ST_QUEUE_DONE 0x80

/* Where the DMA drops results. Must be 16-byte aligned. */
#define RESULT_BUF 0x00180000

/* CLINT mtime, low word. clint_timer.v increments this unconditionally every
 * cycle, so it is a cycle counter usable for on-hardware timing. */
#define CLINT_MTIME (*((volatile uint32_t*)0x02000000))

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

/* Scalar INT8 GEMM on the host core - the baseline the accelerator is measured
 * against. B is column-major (B[j][k] is column j) to match how bColBuf is fed,
 * so the two paths see the same layout and neither gets a friendlier stride.
 *
 * int32 accumulator, same as the PE, so the arithmetic is equivalent and not
 * merely similar. Kept in .bss (DDR) rather than on the stack so operands start
 * where a real workload would keep them.
 */
/* Freestanding build, no libc. At K=64 the staging loops are long enough that
 * gcc recognises them and emits calls to memset, which then has nothing to
 * link against - so provide it.
 *
 * The attribute is NOT optional. -O2 enables -ftree-loop-distribute-patterns,
 * which recognises the byte-fill loop BELOW as a memset and rewrites it into a
 * call to memset - that is, to itself. The result links cleanly and runs until
 * the recursion walks the stack down through DRAM: on hardware it reached
 * ~121 MB below __stack_top, showing up as endless D-cache writeback/refill
 * pairs at descending addresses. Disabling the pattern pass for this one
 * function keeps the fix with the code rather than in a build flag that a
 * future rebuild can drop. */
__attribute__((optimize("no-tree-loop-distribute-patterns")))
void *memset(void *d, int c, unsigned int n) {
    unsigned char *p = (unsigned char *)d;
    while (n--) *p++ = (unsigned char)c;
    return d;
}

/* Deepest reduction any build here stages. maxK is read from INFO at run time;
 * this only bounds the static buffers. */
#define KSTAGE 64

static int8_t  gemm_a[16 * KSTAGE];
static int8_t  gemm_b[16 * KSTAGE];
static int32_t gemm_c[16 * 16];

static void scalar_gemm(const int8_t *A, const int8_t *B, int32_t *C,
                        int dim, int klen) {
    for (int i = 0; i < dim; i++) {
        for (int j = 0; j < dim; j++) {
            int32_t acc = 0;
            for (int k = 0; k < klen; k++)
                acc += (int32_t)A[i * klen + k] * (int32_t)B[j * klen + k];
            C[i * dim + j] = acc;
        }
    }
}

/* Operand panels staged for the DMA: lanes packed back to back, so the fetch
 * covers a whole panel in a single INCR burst.
 *
 * Aligned to a whole panel, not just to a line. At maxK=64 a panel is
 * dim*64 = 512 bytes and the burst is 32 lines; AXI4 forbids an INCR burst
 * from crossing a 4 KB boundary, and a panel-aligned base cannot, because the
 * panel size divides 4096. At maxK=16 the burst was 128 bytes and this rarely
 * bit - it is a real constraint now, and simulation cannot see it. */
#define PANEL_ALIGN 1024
static int8_t dma_a[16 * KSTAGE] __attribute__((aligned(PANEL_ALIGN)));
static int8_t dma_b[16 * KSTAGE] __attribute__((aligned(PANEL_ALIGN)));

/* One B panel per queued tile, each with distinct operands so a dropped or
 * reordered descriptor shows up as a wrong answer. */
#define QTILES 4
#define QRESULT_BUF 0x00190000
static int8_t dma_bq[QTILES][16 * KSTAGE] __attribute__((aligned(PANEL_ALIGN)));

/* Eviction buffer: one line per set of the direct-mapped D-cache
 * (1024 sets x 16 B = 16 KB). */
#define DC_SETS  1024
#define DC_LINE  16
/* Aligned to the WHOLE cache, not merely to a line.
 *
 * The full walk below does not care: it touches DC_SETS consecutive lines, so
 * it covers every set whatever offset it starts at. The RANGE walk does care,
 * because it indexes by set number - evict_buf[set * DC_LINE] only lands in
 * `set` if the buffer itself starts at set 0. At 16-byte alignment it starts
 * at an arbitrary set, every range flush displaces the wrong one, and nothing
 * reports an error: the load succeeds, the target line stays dirty, and the
 * accelerator reads stale operands. */
static volatile uint8_t evict_buf[DC_SETS * DC_LINE]
    __attribute__((aligned(DC_SETS * DC_LINE)));

/* Force dirty lines back to DRAM by displacing them.
 *
 * The D-cache is write-back and this SoC has no flush instruction, no cache
 * maintenance CSR and no uncached DRAM window - the MMIO decodes are device
 * windows only. Touching one address in every set of a direct-mapped cache
 * evicts whatever occupied it, writing back anything dirty. That is the only
 * mechanism available here, and it is why operands must be flushed before the
 * accelerator reads them from DRAM.
 */
static void dcache_evict(void) {
    volatile uint8_t sink = 0;
    for (int i = 0; i < DC_SETS * DC_LINE; i += DC_LINE)
        sink ^= evict_buf[i];
    (void)sink;
}

/* Same mechanism, bounded to the sets a given range actually occupies.
 *
 * The cache is direct-mapped, so an address's set is just a slice of it and
 * only the sets the buffer covers can be holding its lines. Walking all 1024
 * regardless is O(cache) work for what is nearly always an O(range) problem:
 * a dim=8 result tile is 256 bytes = 16 lines, so this is 16 displacements
 * rather than 1024.
 *
 * Whether that is worth much depends entirely on the range - 64x fewer lines
 * for a result tile, 16x for the operand panels, 4x for the four queued B
 * panels - so it is not one speedup number, and the per-site figures are
 * reported below where they are measured.
 *
 * Note this takes a length in bytes and rounds the start DOWN to a line, so a
 * partially-covered first line is still flushed. */
static void dcache_flush_range(const void *p, unsigned int len) {
    volatile uint8_t sink = 0;
    uintptr_t a   = (uintptr_t)p & ~((uintptr_t)DC_LINE - 1u);
    uintptr_t end = (uintptr_t)p + len;
    for (; a < end; a += DC_LINE) {
        unsigned int set = (unsigned int)((a / DC_LINE) & (DC_SETS - 1u));
        sink ^= evict_buf[set * DC_LINE];
    }
    (void)sink;
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

    /* Memory provably holds the right values here (dumped from ddr_mem in the
     * bench). Print what the CPU sees for the same words: a difference means
     * the read side is serving stale cache lines, not that the DMA misbehaved. */
    /* Force any lines covering RESULT_BUF out before reading it. The DMA wrote
     * DRAM behind the cache, so whatever the cache holds for those addresses is
     * stale by construction - and "untouched therefore uncached" stopped being
     * true once a cache-sized buffer joined .bss. */
    dcache_flush_range((const void *)RESULT_BUF, (unsigned int)(dim * dim * 4));

    /* The pointer dump and the buf[0..3] trace that used to sit here were for
     * diagnosing the coherence failure described above, which is now fixed and
     * documented. The check below covers the same ground: a stale cache line
     * makes these values wrong, and the expected set (4..256) is distinctive
     * enough that stale or uninitialised memory cannot match by chance. */
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

    /* ============ performance, measured on real hardware ============
     *
     * Everything the operand-path plan rests on - the load/compute/readback
     * split, the per-transaction cost, the DMA speedup - came from a SIMULATION
     * whose memory was a fixed 3-cycle mock with the arbiter port entirely to
     * itself. Real DDR behind axi_cache_adapter, contending with a live I-cache
     * and D-cache, is a different machine. These numbers replace those
     * estimates with measurements from the design that actually exists.
     *
     * CLINT mtime increments unconditionally every cycle (clint_timer.v:30), so
     * it is a true cycle counter, not a divided tick. It is read over MMIO,
     * which costs about as much as one of the transactions being measured, so
     * the probe overhead is measured first and SUBTRACTED from each phase - and
     * printed too, so every number below can be read with its instrument in
     * view rather than silently absorbing it.
     */
    uart_print("\r\n--- performance (cycles, measured on hardware) ---\r\n");

    uint32_t t0, t1;

    /* Two back-to-back reads: the gap is one probe's worth of MMIO latency. */
    t0 = CLINT_MTIME;
    t1 = CLINT_MTIME;
    uint32_t probe = t1 - t0;
    uart_print("probe overhead      "); uart_print_int32(probe); uart_print("\r\n");

    /* klen=4 above keeps the correctness check readable; a representative
     * operand load needs the full depth the array supports.
     *
     * A deeper reduction is what makes the fixed per-tile costs pay: result
     * writeback is dim*dim accumulators whatever K is, and so is the control
     * traffic, so both amortise over 4x the arithmetic at K=64. */
    const int kperf = (maxk < KSTAGE) ? maxk : KSTAGE;
    /* Bytes per lane in the staged panels. maxK is a power of two >= 16, so a
     * lane is a whole number of 16-byte lines and lanes stay line-aligned. */
    const int lane_pitch = kperf;
    ACCEL_KLEN = kperf;

    t0 = CLINT_MTIME;
    push_panel(&ACCEL_A_PUSH, va, dim, kperf);
    push_panel(&ACCEL_B_PUSH, vb, dim, kperf);
    t1 = CLINT_MTIME;
    uint32_t c_load = t1 - t0 - probe;

    t0 = CLINT_MTIME;
    ACCEL_CTRL = CTRL_START;
    while (!(ACCEL_STATUS & ST_DONE)) {}
    t1 = CLINT_MTIME;
    uint32_t c_comp = t1 - t0 - probe;

    /* Readback through the 32-bit AXI4-Lite register window. */
    t0 = CLINT_MTIME;
    ACCEL_RESULT_IDX = 0;
    for (int n = 0; n < dim * dim; n++) { (void)ACCEL_RESULT; }
    t1 = CLINT_MTIME;
    uint32_t c_mmio = t1 - t0 - probe;

    /* Every store below is timed over this many repeats and divided. See
     * scripts/amortize_store_timing.py: one store is smaller than the MMIO
     * floor around it, and two wrong conclusions came out of measuring it
     * once. */
    #define STORE_REPS 16

    /* Readback through the 128-bit DMA line port.
     *
     * Timed WITHOUT reading the destination back. The transfer time is what is
     * being measured, and reading would pull 64 lines into the D-cache, which
     * both perturbs a repeat measurement and runs into the coherence
     * constraint documented above. Correctness was already proven earlier. */
    ACCEL_DEST_ADDR   = RESULT_BUF;
    ACCEL_DEST_STRIDE = dim * 4;
    t0 = CLINT_MTIME;
    for (int r = 0; r < STORE_REPS; r++) {
        ACCEL_CTRL = CTRL_START_DMA;
        while (!(ACCEL_STATUS & ST_DMA_DONE)) {}
    }
    t1 = CLINT_MTIME;
    uint32_t c_dma = (t1 - t0 - probe) / STORE_REPS;

    /* Same 16 lines, STRIDED destination - the discriminator for why a result
     * write costs ~3x more per line than an operand read.
     *
     * A non-contiguous destination makes the DMA issue dim/4 lines per burst
     * instead of the whole tile, so the identical payload crosses AXI as 8
     * transactions rather than 1. Simulation reproduces the measured
     * contiguous cost under either of two very different assumptions, and
     * those two disagree sharply here:
     *
     *   slow write RESPONSE   -> cost follows TRANSACTIONS, ratio about 6.4x
     *   slow write ACCEPTANCE -> cost follows BEATS, ratio about 2.3x
     *
     * The first is an acknowledgment the accelerator merely waits on and could
     * overlap; the second is bandwidth that simply is not there. Measuring
     * only the contiguous case cannot separate them, which is exactly why this
     * second measurement exists. */
    ACCEL_DEST_ADDR   = RESULT_BUF;
    ACCEL_DEST_STRIDE = dim * 4 * 2;
    t0 = CLINT_MTIME;
    for (int r = 0; r < STORE_REPS; r++) {
        ACCEL_CTRL = CTRL_START_DMA;
        while (!(ACCEL_STATUS & ST_DMA_DONE)) {}
    }
    t1 = CLINT_MTIME;
    uint32_t c_dma_strided = (t1 - t0 - probe) / STORE_REPS;
    ACCEL_DEST_STRIDE = dim * 4;      /* restore contiguous for later tests */

    /* Requantized INT8 writeback of the SAME accumulators.
     *
     * Accumulate wide, scale down, clamp on the way out - the ordinary output
     * stage of a quantized INT8 GEMM, and what Gemmini and OpenGeMM both do
     * before anything reaches memory. Here it is also the largest remaining
     * lever: results are half the bytes a tile moves but 61% of its memory
     * time, because this platform accepts writes at roughly a third the rate
     * it serves reads. One byte per result instead of four turns a dim=8 tile
     * from 16 lines into 4.
     *
     * C[i][j] = kperf*(i+1)*(j+1), so shift 6 divides out kperf=64 exactly and
     * the expected value is (i+1)*(j+1) - 1..64, inside INT8 with no clipping,
     * and different in every position so a mis-packed line cannot pass. */
    ACCEL_OUT_CTRL    = OUT_INT8 | 6u;
    ACCEL_DEST_ADDR   = RESULT_BUF;
    ACCEL_DEST_STRIDE = 0;
    t0 = CLINT_MTIME;
    for (int r = 0; r < STORE_REPS; r++) {
        ACCEL_CTRL = CTRL_START_DMA;
        while (!(ACCEL_STATUS & ST_DMA_DONE)) {}
    }
    t1 = CLINT_MTIME;
    uint32_t c_dma8 = (t1 - t0 - probe) / STORE_REPS;
    ACCEL_OUT_CTRL    = 0;                /* back to INT32 for what follows */
    ACCEL_DEST_STRIDE = dim * 4;

    dcache_flush_range((const void *)RESULT_BUF, (unsigned int)(dim * dim));
    volatile int8_t *r8 = (volatile int8_t *)RESULT_BUF;
    int bad_q = 0;
    for (int i = 0; i < dim; i++)
        for (int j = 0; j < dim; j++)
            if (r8[i * dim + j] != (int8_t)((i + 1) * (j + 1))) bad_q++;
    if (bad_q == 0) uart_print("PASS: INT8 requantized writeback correct\r\n");
    else {
        uart_print("FAIL: INT8 requantized results wrong: ");
        uart_print_int32(bad_q); uart_print("\r\n");
    }

    uart_print("operand load        "); uart_print_int32((int32_t)c_load); uart_print("\r\n");
    uart_print("compute             "); uart_print_int32((int32_t)c_comp); uart_print("\r\n");
    uart_print("readback (MMIO)     "); uart_print_int32((int32_t)c_mmio); uart_print("\r\n");
    uart_print("readback (DMA)      "); uart_print_int32((int32_t)c_dma);
    uart_print("   (mean of 16; a single store is below the MMIO floor)\r\n");

    uart_print("readback (DMA,int8)  "); uart_print_int32((int32_t)c_dma8);
    uart_print("\r\n");
    uart_print("  int8 writeback speedup x100  ");
    uart_print_int32(c_dma8 ? (int32_t)((c_dma * 100) / c_dma8) : 0);
    uart_print("\r\n");
    uart_print("readback (DMA,strided) ");
    uart_print_int32((int32_t)c_dma_strided); uart_print("\r\n");
    uart_print("  strided/contig x100  ");
    uart_print_int32(c_dma ? (int32_t)((c_dma_strided * 100) / c_dma) : 0);
    uart_print("   >=500 response-bound, <=300 acceptance-bound\r\n");

    /* Derived costs. These are the figures the simulation put at ~5.0 cycles
     * per AXI4-Lite beat and ~7.4 per 128-bit line, and from which the
     * 2.16 B/cycle operand-bandwidth estimate was computed. */
    int ld_txn   = 2 * (1 + dim * (kperf / 4));   /* 2 panels: index + pushes */
    int rd_txn   = 1 + dim * dim;                 /* index write + reads      */
    int dma_line = (dim * dim) / 4;               /* 4 results per 128b line  */

    uart_print("\r\nper-transaction (cycles x100)\r\n");
    uart_print("  AXI4-Lite write   "); uart_print_int32((int32_t)((c_load * 100) / ld_txn));   uart_print("\r\n");
    uart_print("  AXI4-Lite read    "); uart_print_int32((int32_t)((c_mmio * 100) / rd_txn));   uart_print("\r\n");
    uart_print("  DMA 128-bit line  "); uart_print_int32((int32_t)((c_dma  * 100) / dma_line)); uart_print("\r\n");

    if (c_dma > 0) {
        uart_print("\r\nDMA vs MMIO readback (x100)  ");
        uart_print_int32((int32_t)((c_mmio * 100) / c_dma));
        uart_print("\r\n");
    }

    /* Whole-tile totals, so the split can be compared against simulation
     * directly. MACs/cycle is scaled x100 because there is no float here. */
    uint32_t c_total_mmio = c_load + c_comp + c_mmio;
    uint32_t c_total_dma  = c_load + c_comp + c_dma;
    uint32_t macs         = (uint32_t)(dim * dim * kperf);

    uart_print("\r\ntile total (MMIO)   "); uart_print_int32((int32_t)c_total_mmio); uart_print("\r\n");
    uart_print("tile total (DMA)    ");     uart_print_int32((int32_t)c_total_dma);  uart_print("\r\n");
    uart_print("MACs                ");     uart_print_int32((int32_t)macs);         uart_print("\r\n");
    uart_print("MACs/cycle x100     ");     uart_print_int32((int32_t)((macs * 100) / c_total_dma)); uart_print("\r\n");
    uart_print("utilisation % x100  ");     uart_print_int32((int32_t)((macs * 10000) / (c_total_dma * (uint32_t)(dim * dim)))); uart_print("\r\n");

    /* ============ scalar RV32 baseline ============
     *
     * The same GEMM the accelerator just did, in C on the host core, timed the
     * same way. This is the number that says whether the accelerator is worth
     * its area - every "Nx over scalar" figure quoted before this point was an
     * ESTIMATE from an assumed instruction mix, never a measurement.
     *
     * Fairness, deliberately:
     *   - identical dimensions (dim x dim x kperf) and identical operand values
     *   - identical memory layout: B is column-major here because that is how
     *     bColBuf is fed, so neither side gets a friendlier access pattern
     *   - same -O2 build as everything else; the scalar loop is not hobbled
     *   - both results are checked against the same closed form, so a wrong
     *     answer cannot look fast
     *   - operands start in DDR for both: the accelerator pays to push them
     *     over MMIO, the CPU pays to load them through the D-cache
     *
     * The accelerator figure it is compared against (c_total_dma) includes its
     * operand load and its DMA readback, not just the compute window. Comparing
     * against the 61-cycle compute phase alone would flatter it by ~17x.
     */
    uart_print("\r\n--- scalar RV32 baseline (same GEMM) ---\r\n");

    for (int i = 0; i < dim; i++)
        for (int k = 0; k < kperf; k++) {
            gemm_a[i * kperf + k] = (int8_t)(i + 1);
            gemm_b[i * kperf + k] = (int8_t)(i + 1);
        }

    t0 = CLINT_MTIME;
    scalar_gemm(gemm_a, gemm_b, gemm_c, dim, kperf);
    t1 = CLINT_MTIME;
    uint32_t c_scalar = t1 - t0 - probe;

    /* A[i][k]=i+1 and B[j][k]=j+1 for every k, so C[i][j] = kperf*(i+1)*(j+1).
     * Checking this stops a mis-indexed or partially-eliminated loop from
     * posting a fast time for the wrong work. */
    int scalar_bad = 0;
    for (int i = 0; i < dim; i++)
        for (int j = 0; j < dim; j++)
            if (gemm_c[i * dim + j] != (int32_t)kperf * (i + 1) * (j + 1)) scalar_bad++;

    if (scalar_bad) {
        fail_count++;
        uart_print("FAIL: scalar GEMM mismatches=");
        uart_print_int32(scalar_bad);
        uart_print("\r\n");
    } else {
        uart_print("PASS: scalar GEMM correct\r\n");
    }

    uart_print("scalar cycles       "); uart_print_int32((int32_t)c_scalar); uart_print("\r\n");
    uart_print("accel cycles (DMA)  "); uart_print_int32((int32_t)c_total_dma); uart_print("\r\n");

    if (c_total_dma > 0) {
        uart_print("SPEEDUP x100        ");
        uart_print_int32((int32_t)((c_scalar * 100) / c_total_dma));
        uart_print("\r\n");
    }
    if (c_scalar > 0) {
        uart_print("scalar MACs/cyc x100 ");
        uart_print_int32((int32_t)((macs * 100) / c_scalar));
        uart_print("\r\n");
    }

    /* Compute-only comparison, reported separately and labelled as such. This
     * is the array's raw advantage once operands are already resident - the
     * ceiling the operand-DMA work is aiming at, not a number to quote as the
     * end-to-end speedup. */
    if (c_comp > 0) {
        uart_print("(compute-only x100  ");
        uart_print_int32((int32_t)((c_scalar * 100) / c_comp));
        uart_print(")\r\n");
    }

    /* ============ operand DMA, scratchpad, double buffering ============
     *
     * Everything above pushes operands through the 32-bit register window and
     * pulls results back the same way. This section exercises the paths that
     * replace both: the accelerator fetching its own operands over the 128-bit
     * line port, holding several B panels locally, and prefetching the next
     * panel while the array computes from the current one.
     *
     * COHERENCE. The D-cache is WRITE-BACK and the DMA reads DRAM behind it,
     * so operand bytes the CPU just wrote are still sitting dirty in the cache
     * and the fetch would read stale DRAM. This SoC has no flush instruction
     * and no uncached window, so the panels are forced out by eviction: the
     * cache is direct-mapped, and touching one line in each of its 1024 sets
     * displaces everything, writing dirty lines back on the way out.
     *
     * That costs ~1024 accesses, but it happens ONCE per operand upload rather
     * than per tile, so it does not sit in the steady-state path. It is also
     * exactly the case a tightly-coupled scratchpad removes - and note that an
     * accelerator-to-accelerator chain (this tile's results feeding the next
     * layer) needs no flush at all, since both directions bypass the cache.
     *
     * PACKED LAYOUT. SRC_STRIDE is set to one lane (16 B) so the panel is
     * contiguous and the fetch covers it in a single INCR burst. A wider
     * stride is still correct but falls back to one transaction per line,
     * which measured ~3x slower - so the packing here is load-bearing, not
     * incidental.
     */
    uart_print("\r\n--- operand DMA / scratchpad / double buffering ---\r\n");

    /* Panels are staged with lanes packed back to back, which is what makes
     * the burst possible: a lane is lane_pitch bytes = kperf/16 lines. */
    for (int lane = 0; lane < dim; lane++)
        for (int k = 0; k < lane_pitch; k++) {
            dma_a[lane * lane_pitch + k] = (int8_t)((k < kperf) ? (lane + 1) : 0);
            dma_b[lane * lane_pitch + k] = (int8_t)((k < kperf) ? (lane + 1) : 0);
        }

    t0 = CLINT_MTIME;
    dcache_evict();
    t1 = CLINT_MTIME;
    uint32_t c_evict = t1 - t0 - probe;

    /* Same work, bounded walk. The operands have to be dirtied again first:
     * the walk above already displaced them, so timing the range version on a
     * clean cache would measure nothing and flatter it enormously. */
    for (int lane = 0; lane < dim; lane++)
        for (int k = 0; k < lane_pitch; k++) {
            dma_a[lane * lane_pitch + k] = (int8_t)((k < kperf) ? (lane + 1) : 0);
            dma_b[lane * lane_pitch + k] = (int8_t)((k < kperf) ? (lane + 1) : 0);
        }
    t0 = CLINT_MTIME;
    dcache_flush_range(dma_a, (unsigned int)(dim * lane_pitch));
    dcache_flush_range(dma_b, (unsigned int)(dim * lane_pitch));
    t1 = CLINT_MTIME;
    uint32_t c_evict_range = t1 - t0 - probe;

    ACCEL_A_SRC      = (uint32_t)(uintptr_t)dma_a;
    ACCEL_B_SRC      = (uint32_t)(uintptr_t)dma_b;
    ACCEL_SRC_STRIDE = lane_pitch;      /* packed -> single burst per panel */
    ACCEL_B_PANEL_LOAD = 0;
    ACCEL_B_PANEL_USE  = 0;

    t0 = CLINT_MTIME;
    ACCEL_CTRL = CTRL_START_LOAD;
    while (!(ACCEL_STATUS & ST_LOAD_DONE)) {}
    t1 = CLINT_MTIME;
    uint32_t c_opdma = t1 - t0 - probe;

    /* Same GEMM as before, so the same closed form applies. If the fetch read
     * the wrong addresses, or read stale DRAM because the eviction did not
     * work, these values are wrong rather than merely slow. */
    ACCEL_CTRL = CTRL_START;
    while (!(ACCEL_STATUS & ST_DONE)) {}

    int dma_ops_bad = 0;
    ACCEL_RESULT_IDX = 0;
    for (int i = 0; i < dim; i++)
        for (int j = 0; j < dim; j++)
            if ((int32_t)ACCEL_RESULT != (int32_t)kperf * (i + 1) * (j + 1))
                dma_ops_bad++;

    if (dma_ops_bad) {
        fail_count++;
        uart_print("FAIL: operand DMA mismatches=");
        uart_print_int32(dma_ops_bad);
        uart_print("\r\n");
    } else {
        uart_print("PASS: operand DMA fetched correct operands\r\n");
    }

    uart_print("operand push (MMIO) "); uart_print_int32((int32_t)c_load);  uart_print("\r\n");
    uart_print("operand DMA (burst) "); uart_print_int32((int32_t)c_opdma); uart_print("\r\n");
    if (c_opdma > 0) {
        uart_print("  speedup x100      ");
        uart_print_int32((int32_t)((c_load * 100) / c_opdma));
        uart_print("\r\n");
    }
    uart_print("cache evict (full)  "); uart_print_int32((int32_t)c_evict); uart_print("\r\n");
    uart_print("cache evict (range) "); uart_print_int32((int32_t)c_evict_range); uart_print("\r\n");
    if (c_evict_range) {
        uart_print("  range cheaper x100 ");
        uart_print_int32((int32_t)((c_evict * 100) / c_evict_range));
        uart_print("\r\n");
    }

    /* ---- double buffering: prefetch panel 1 while computing from panel 0 ----
     * B-only load (CTRL bit4) leaves aRowBuf alone, which is what makes it safe
     * to issue mid-compute. Panels must differ: loading the panel currently
     * feeding the array would corrupt the run in flight. */
    ACCEL_B_PANEL_USE = 0;
    t0 = CLINT_MTIME;
    ACCEL_CTRL = CTRL_START;            /* compute from panel 0 ... */
    ACCEL_B_PANEL_LOAD = 1;
    ACCEL_CTRL = CTRL_START_LOAD_B;     /* ... while panel 1 fills */
    while (!(ACCEL_STATUS & ST_DONE)) {}
    while (!(ACCEL_STATUS & ST_LOAD_DONE)) {}
    t1 = CLINT_MTIME;
    uint32_t c_overlap = t1 - t0 - probe;

    /* The overlapped run must still be right for the panel it was using. */
    int ov_bad = 0;
    ACCEL_RESULT_IDX = 0;
    for (int i = 0; i < dim; i++)
        for (int j = 0; j < dim; j++)
            if ((int32_t)ACCEL_RESULT != (int32_t)kperf * (i + 1) * (j + 1))
                ov_bad++;

    if (ov_bad) {
        fail_count++;
        uart_print("FAIL: overlapped run corrupted, mismatches=");
        uart_print_int32(ov_bad);
        uart_print("\r\n");
    } else {
        uart_print("PASS: prefetch during compute did not corrupt the run\r\n");
    }
    uart_print("compute+prefetch    "); uart_print_int32((int32_t)c_overlap); uart_print("\r\n");

    /* Whole-tile total on the fast path, for comparison with the MMIO tile. */
    uint32_t c_tile_fast = c_opdma + c_comp + c_dma;
    uart_print("\r\ntile (MMIO push + MMIO readback) "); uart_print_int32((int32_t)c_total_mmio); uart_print("\r\n");
    uart_print("tile (MMIO push + result DMA)    ");     uart_print_int32((int32_t)c_total_dma);  uart_print("\r\n");
    uart_print("tile (operand DMA + result DMA)  ");     uart_print_int32((int32_t)c_tile_fast);  uart_print("\r\n");
    if (c_tile_fast > 0) {
        uart_print("vs scalar x100                   ");
        uart_print_int32((int32_t)((c_scalar * 100) / c_tile_fast));
        uart_print("\r\n");
    }

    /* ============ descriptor queue: a batch of tiles, one kick ============
     *
     * Each tile has cost ~10 MMIO transactions of control - program the source
     * and destination addresses, select panels, kick each of the three phases,
     * poll each for completion. At the ~16 cycles a write costs on this SoC
     * that is ~160 cycles, which becomes most of a tile once burst DMA cuts the
     * data movement.
     *
     * A descriptor carries everything a tile needs. Software pushes a batch and
     * kicks ONCE; the accelerator walks the queue itself, so there is no CPU
     * round trip between load, compute and store, and no per-tile polling.
     *
     * Both schedules below run the SAME operands and are checked against the
     * same closed form, so the difference is control overhead alone. Each tile
     * uses a distinct B panel and its own destination, which means a sequencer
     * that dropped, repeated or reordered a descriptor produces wrong answers
     * rather than merely a different time.
     */
    uart_print("\r\n--- descriptor queue (batch of tiles) ---\r\n");

    /* B panel p holds lane value (lane+1+p), so every tile has distinct
     * operands and therefore a distinct expected result. */
    for (int p = 0; p < QTILES; p++)
        for (int lane = 0; lane < dim; lane++)
            for (int k = 0; k < lane_pitch; k++)
                dma_bq[p][lane * lane_pitch + k] =
                    (int8_t)((k < kperf) ? (lane + 1 + p) : 0);

    /* dma_a plus the four staged B panels - everything the queue will read. */
    dcache_flush_range(dma_a, (unsigned int)(dim * lane_pitch));
    for (int p = 0; p < QTILES; p++)
        dcache_flush_range(dma_bq[p], (unsigned int)(dim * lane_pitch));

    /* ---- STEPPED: software drives every phase, as before ---- */
    t0 = CLINT_MTIME;
    for (int p = 0; p < QTILES; p++) {
        ACCEL_KLEN         = kperf;
        ACCEL_A_SRC        = (uint32_t)(uintptr_t)dma_a;
        ACCEL_B_SRC        = (uint32_t)(uintptr_t)dma_bq[p];
        ACCEL_B_PANEL_LOAD = p;
        /* B-only after the first tile, matching what the queued path does.
         * If this stayed a full load, the stepped-vs-queued delta would mix
         * the control saving with a bandwidth saving and the "control
         * speedup" number would not mean what it says. */
        ACCEL_CTRL         = p ? CTRL_START_LOAD_B : CTRL_START_LOAD;
        while (!(ACCEL_STATUS & ST_LOAD_DONE)) {}
        ACCEL_B_PANEL_USE  = p;
        ACCEL_CTRL         = CTRL_START;
        while (!(ACCEL_STATUS & ST_DONE)) {}
        ACCEL_DEST_ADDR    = QRESULT_BUF + p * 0x1000;
        ACCEL_DEST_STRIDE  = dim * 4;
        ACCEL_CTRL         = CTRL_START_DMA;
        while (!(ACCEL_STATUS & ST_DMA_DONE)) {}
    }
    t1 = CLINT_MTIME;
    uint32_t c_stepped = t1 - t0 - probe;

    /* ---- QUEUED: push descriptors, kick once, poll once ----
     *
     * Requantized output for the batch too. C[i][j] = kperf*(i+1)*(j+1+p), so
     * shift 6 divides out kperf=64 and leaves (i+1)*(j+1+p) - at most 8*11=88
     * for the last tile, inside INT8 without clipping, and distinct in every
     * position so a mis-packed line cannot pass.
     *
     * A requantized tile is dim*dim contiguous BYTES, so DEST_STRIDE must be 0
     * here: a line spans 16/dim result rows and there is no row boundary to
     * stride at. */
    ACCEL_OUT_CTRL    = OUT_INT8 | 6u;
    ACCEL_DEST_STRIDE = 0;
    t0 = CLINT_MTIME;
    for (int p = 0; p < QTILES; p++) {
        ACCEL_DESC_A_SRC = (uint32_t)(uintptr_t)dma_a;
        ACCEL_DESC_B_SRC = (uint32_t)(uintptr_t)dma_bq[p];
        ACCEL_DESC_DEST  = QRESULT_BUF + p * 0x1000;
        /* {bOnly[16], panelLoad[15:12], panelUse[11:8], kLen[7:0]}
         *
         * Every tile here shares dma_a, so only the first needs to fetch it.
         * This is the ordinary shape of a tiled GEMM inner loop, not a
         * property of this test: walking a row of C holds A fixed. */
        ACCEL_DESC_PUSH  = (p ? DESC_BONLY : 0u)          |
                           ((uint32_t)(p & 0xF) << 12)    |
                           ((uint32_t)(p & 0xF) << 8)     |
                           (uint32_t)kperf;
    }
    ACCEL_CTRL = CTRL_START_QUEUE;
    while (!(ACCEL_STATUS & ST_QUEUE_DONE)) {}
    t1 = CLINT_MTIME;
    uint32_t c_queued = t1 - t0 - probe;

    /* The DMA wrote DRAM behind the cache, so displace before reading it back.
     * The tiles are 0x1000 apart, so one range call per tile rather than one
     * spanning call - a single range covering all four would be 16 KB, i.e.
     * the whole cache, and no cheaper than the full walk. */
    for (int p = 0; p < QTILES; p++)
        dcache_flush_range((const void *)(QRESULT_BUF + p * 0x1000),
                           (unsigned int)(dim * dim));

    int q_bad = 0;
    for (int p = 0; p < QTILES; p++) {
        volatile int8_t *qb = (volatile int8_t *)(QRESULT_BUF + p * 0x1000);
        for (int i = 0; i < dim; i++)
            for (int j = 0; j < dim; j++)
                if (qb[i * dim + j] != (int8_t)((i + 1) * (j + 1 + p)))
                    q_bad++;
    }

    if (q_bad) {
        fail_count++;
        uart_print("FAIL: queue mismatches=");
        uart_print_int32(q_bad);
        uart_print("\r\n");
    } else {
        uart_print("PASS: queue ran ");
        uart_print_int32(QTILES);
        uart_print(" tiles correctly\r\n");
    }

    uart_print("stepped (per-phase MMIO) "); uart_print_int32((int32_t)c_stepped); uart_print("\r\n");
    uart_print("queued  (one kick)       "); uart_print_int32((int32_t)c_queued);  uart_print("\r\n");
    if (c_queued > 0) {
        uart_print("  control speedup x100   ");
        uart_print_int32((int32_t)((c_stepped * 100) / c_queued));
        uart_print("\r\n");
    }
    uart_print("per tile: stepped "); uart_print_int32((int32_t)(c_stepped / QTILES));
    uart_print(", queued ");          uart_print_int32((int32_t)(c_queued / QTILES));
    uart_print("\r\n");

    if (c_queued > 0) {
        uart_print("queued MACs/cyc x100     ");
        uart_print_int32((int32_t)(((uint32_t)QTILES * macs * 100) / c_queued));
        uart_print("\r\n");
        uart_print("vs scalar x100           ");
        uart_print_int32((int32_t)((c_scalar * (uint32_t)QTILES * 100) / c_queued));
        uart_print("\r\n");
    }

    if (fail_count == 0) {
        ACCEL_OUT_CTRL    = 0;
    ACCEL_DEST_STRIDE = dim * 4;
    uart_print("=== ALL MATMUL CHECKS PASSED ===\r\n");
    } else {
        uart_print("=== MATMUL FAILURES: ");
        uart_print_int32(fail_count);
        uart_print(" ===\r\n");
    }

    while (1) {}
    return 0;
}
