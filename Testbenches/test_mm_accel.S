/* CPU-integrated test for the Chisel-generated DIM=8 systolic matmul
 * accelerator: real sw/lw instructions through the pipeline to the MMIO window
 * (ACCEL_MMIO_BASE=0x00005000), exercising axi_lite_bridge's AXI4-Lite master
 * and mm_accel's slave end to end - then the RESULT DMA, which leaves through
 * mem_arbiter's third port rather than the register window.
 *
 * REWRITTEN for the indexed register map. The previous version drove the
 * hand-written 2x2 core's fixed-word map (pushes at 0x10..0x1C, results at
 * 0x20..0x2C), which does not exist any more: that map needed 2*DIM+DIM*DIM
 * words and could not fit DIM=16 in a 64-word window, which is why it was
 * replaced by an indexed map of constant size.
 *
 *   word 0  0x00 CTRL       bit0=START, bit1=SOFT_RST, bit2=START_DMA
 *   word 1  0x04 STATUS     bit0=BUSY, bit1=DONE, bit2=DMA_BUSY, bit3=DMA_DONE
 *   word 2  0x08 K_LEN
 *   word 3  0x0C LOAD_K
 *   word 4  0x10 LOAD_LANE  (writing it resets LOAD_K)
 *   word 5  0x14 A_PUSH     4 packed INT8, LOAD_K auto-increments,
 *                           LOAD_LANE auto-advances when the lane is full
 *   word 6  0x18 B_PUSH     same
 *   word 7  0x1C RESULT_IDX
 *   word 8  0x20 RESULT     auto-increments RESULT_IDX on read
 *   word 9  0x24 INFO       {maxK[15:8], dim[7:0]}
 *   word 10 0x28 DEST_ADDR
 *   word 11 0x2C DEST_STRIDE
 *
 * Operands are chosen so every result depends on BOTH its row and its column -
 * a bug that transposes the array, or collapses a row, changes the answer:
 *   A[i][k] = i+1   for k = 0..3      (lane i is four copies of i+1)
 *   B[j][k] = j+1   for k = 0..3
 *   C[i][j] = sum_k (i+1)(j+1) = 4*(i+1)*(j+1)
 *
 * Checked (via hierarchical DUT.RF.regs[] peek, as tb_amo_test.v does):
 *   x20 = C[0][0] = 4*1*1 =   4
 *   x21 = C[1][2] = 4*2*3 =  24
 *   x22 = C[3][5] = 4*4*6 =  96
 *   x23 = C[7][7] = 4*8*8 = 256
 *   x24 = INFO, expected 0x00001008 (maxK=16, dim=8) - geometry discovery, so
 *         a mismatched build fails loudly instead of producing wrong numbers
 *   x25 = DMA_DONE observed (1) - the result DMA completed
 */
.section .text.init
.global _start

_start:
    csrw mie, zero
    csrw mstatus, zero
    la t0, halt
    csrw mtvec, t0

    li x5, 0x00005000

    /* Geometry check first: if this build is not DIM=8 the operand schedule
     * below is wrong and every later check would fail confusingly. */
    lw x24, 0x24(x5)

    /* K_LEN = 4 -> one packed push word per lane, so LOAD_LANE advances on
     * every push and a whole panel loads with a single LOAD_LANE write. */
    li x2, 4
    sw x2, 0x08(x5)

    /* ---- A panel: lane i = four copies of (i+1) ---- */
    li x2, 0
    sw x2, 0x10(x5)              /* LOAD_LANE = 0 */
    li x6, 1                     /* value i+1 */
    li x7, 9                     /* stop after 8 lanes */
a_loop:
    slli x8, x6, 8
    or   x8, x8, x6
    slli x9, x8, 16
    or   x8, x8, x9              /* 0x0v0v0v0v */
    sw   x8, 0x14(x5)            /* A_PUSH */
    addi x6, x6, 1
    blt  x6, x7, a_loop

    /* ---- B panel: lane j = four copies of (j+1) ---- */
    li x2, 0
    sw x2, 0x10(x5)              /* LOAD_LANE = 0 */
    li x6, 1
    li x7, 9
b_loop:
    slli x8, x6, 8
    or   x8, x8, x6
    slli x9, x8, 16
    or   x8, x8, x9
    sw   x8, 0x18(x5)            /* B_PUSH */
    addi x6, x6, 1
    blt  x6, x7, b_loop

    /* ---- run ---- */
    li x2, 1
    sw x2, 0x00(x5)              /* CTRL.START */
wait_done:
    lw x6, 0x04(x5)
    andi x6, x6, 2               /* STATUS.DONE */
    beqz x6, wait_done

    /* ---- spot-check four accumulators through the register window ---- */
    li x2, 0                     /* C[0][0] -> idx 0 */
    sw x2, 0x1C(x5)
    lw x20, 0x20(x5)

    li x2, 10                    /* C[1][2] -> idx 1*8+2 */
    sw x2, 0x1C(x5)
    lw x21, 0x20(x5)

    li x2, 29                    /* C[3][5] -> idx 3*8+5 */
    sw x2, 0x1C(x5)
    lw x22, 0x20(x5)

    li x2, 63                    /* C[7][7] -> idx 7*8+7 */
    sw x2, 0x1C(x5)
    lw x23, 0x20(x5)

    /* ---- result DMA: drain the same accumulators over the line port ----
     * DEST is in DDR, 16-byte aligned. STRIDE is set to one tile row
     * (8 words = 32 bytes) so the tile lands contiguously; the testbench
     * checks placement against that. */
    li x2, 0x00180000
    sw x2, 0x28(x5)              /* DEST_ADDR - 1.5 MB into the mock DDR, well
                                  * clear of .text at 0x100000 and inside the
                                  * bench's 2 MB ddr_mem; 16-byte aligned as
                                  * the line port requires */
    li x2, 32
    sw x2, 0x2C(x5)              /* DEST_STRIDE */
    li x2, 4
    sw x2, 0x00(x5)              /* CTRL.START_DMA */
wait_dma:
    lw x25, 0x04(x5)
    andi x25, x25, 8             /* STATUS.DMA_DONE */
    beqz x25, wait_dma
    srli x25, x25, 3             /* normalise to 1 */

    /* sentinel: testbench waits on this before checking */
    li x1, 0xA5A5A5A5

halt:
    j halt
