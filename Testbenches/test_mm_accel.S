/* CPU-integrated test for the 2x2 systolic matmul accelerator: issues real
 * sw/lw instructions through the pipeline to the accelerator's MMIO window
 * (ACCEL_MMIO_BASE=0x00005000), exercising the axi_lite_bridge AXI4-Lite
 * master and mm_accel's AXI4-Lite slave end to end, not just the standalone
 * bridge+accelerator testbench.
 *
 * Same A/B operands as Testbenches/tb_mm_accel.v's first run:
 *   A=[[3,-2],[5,7]], B=[[4,1],[-3,6]] -> C=[[18,-9],[-1,47]]
 *
 * Register map (checked via hierarchical DUT.RF.regs[] peek, mirroring
 * Testbenches/tb_amo_test.v's check() task pattern):
 *   x5  = accelerator MMIO base (0x00005000)
 *   x2  = scratch write value
 *   x6  = scratch status read value
 *   x20..x23 = RESULT00/01/10/11 read back after DONE
 */
.section .text.init
.global _start

_start:
    csrw mie, zero
    csrw mstatus, zero
    la t0, halt
    csrw mtvec, t0

    li x5, 0x00005000

    /* K_LEN = 2 */
    li x2, 2
    sw x2, 0x08(x5)

    /* k=0: A[.][0]=3,5  B[0][.]=4,1 */
    li x2, 0
    sw x2, 0x0C(x5)
    li x2, 3
    sw x2, 0x10(x5)
    li x2, 5
    sw x2, 0x14(x5)
    li x2, 4
    sw x2, 0x18(x5)
    li x2, 1
    sw x2, 0x1C(x5)

    /* k=1: A[.][1]=-2,7  B[1][.]=-3,6 */
    li x2, 1
    sw x2, 0x0C(x5)
    li x2, -2
    sw x2, 0x10(x5)
    li x2, 7
    sw x2, 0x14(x5)
    li x2, -3
    sw x2, 0x18(x5)
    li x2, 6
    sw x2, 0x1C(x5)

    /* START */
    li x2, 1
    sw x2, 0x00(x5)

poll_status:
    lw x6, 0x04(x5)
    andi x6, x6, 2   /* DONE bit */
    beqz x6, poll_status

    lw x20, 0x20(x5)
    lw x21, 0x24(x5)
    lw x22, 0x28(x5)
    lw x23, 0x2C(x5)

    /* Sentinel so the testbench can detect completion without guessing a
     * cycle count: x1 = 0xA5A5A5A5 once results have been read back. */
    li x1, 0xA5A5A5A5

halt:
    j halt
    /* Padding runway in case the IF stage prefetches sequentially ahead of
     * halt's self-jump before it resolves - keeps any such speculative
     * fetch from reading genuinely uninitialized (X in sim) memory. */
    .rept 32
    nop
    .endr
