/* Standalone AMO test program - exercises all 9 RV32A AMO ops, each against
 * a freshly-reset memory word, checking both the OLD value returned to rd
 * and the NEW value actually landed in memory (via a follow-up lw). Runs
 * through the real TCM boot stub -> DDR flow via Testbenches/linker_sim.ld,
 * same as the CoreMark sim harness, so it exercises the real cache/AXI-mock
 * memory path rather than a synthetic direct-inject test.
 *
 * Register map (checked via hierarchical DUT.RF.regs[] peek in the
 * testbench, mirroring Testbenches/tb.v's check() task pattern):
 *   x5  = test address (0x00180000, a scratch word inside the sim's 2MB
 *         DDR mock window, safely above .bss)
 *   x2  = scratch "reset value" written before each op
 *   x3  = scratch "operand" (rs2) for each op
 *   x10..x18 = old value returned by each AMO (swap,add,xor,and,or,min,max,minu,maxu)
 *   x19..x27 = memory value read back after each AMO (same op order)
 */
.section .text.init
.global _start

_start:
    csrw mie, zero
    csrw mstatus, zero
    la t0, halt
    csrw mtvec, t0

    li x5, 0x00180000

    /* AMOSWAP: reset=10, operand=99 -> old=10, mem=99 */
    li x2, 10
    sw x2, 0(x5)
    li x3, 99
    amoswap.w x10, x3, (x5)
    lw x19, 0(x5)

    /* AMOADD: reset=10, operand=5 -> old=10, mem=15 */
    li x2, 10
    sw x2, 0(x5)
    li x3, 5
    amoadd.w x11, x3, (x5)
    lw x20, 0(x5)

    /* AMOXOR: reset=10(0xA), operand=6(0x6) -> old=10, mem=12(0xC) */
    li x2, 10
    sw x2, 0(x5)
    li x3, 6
    amoxor.w x12, x3, (x5)
    lw x21, 0(x5)

    /* AMOAND: reset=12(0xC), operand=10(0xA) -> old=12, mem=8(0x8) */
    li x2, 12
    sw x2, 0(x5)
    li x3, 10
    amoand.w x13, x3, (x5)
    lw x22, 0(x5)

    /* AMOOR: reset=8(0x8), operand=3(0x3) -> old=8, mem=11(0xB) */
    li x2, 8
    sw x2, 0(x5)
    li x3, 3
    amoor.w x14, x3, (x5)
    lw x23, 0(x5)

    /* AMOMIN (signed): reset=-5, operand=3 -> old=-5, mem=-5 (min(-5,3)=-5) */
    li x2, -5
    sw x2, 0(x5)
    li x3, 3
    amomin.w x15, x3, (x5)
    lw x24, 0(x5)

    /* AMOMAX (signed): reset=-5, operand=3 -> old=-5, mem=3 (max(-5,3)=3) */
    li x2, -5
    sw x2, 0(x5)
    li x3, 3
    amomax.w x16, x3, (x5)
    lw x25, 0(x5)

    /* AMOMINU (unsigned): reset=20, operand=7 -> old=20, mem=7 (minu(20,7)=7) */
    li x2, 20
    sw x2, 0(x5)
    li x3, 7
    amominu.w x17, x3, (x5)
    lw x26, 0(x5)

    /* AMOMAXU (unsigned): reset=20, operand=7 -> old=20, mem=20 (maxu(20,7)=20) */
    li x2, 20
    sw x2, 0(x5)
    li x3, 7
    amomaxu.w x18, x3, (x5)
    lw x27, 0(x5)

    /* Sentinel so the testbench can detect completion without guessing a
     * cycle count: x1 = 0xA5A5A5A5 once every op above has retired. */
    li x1, 0xA5A5A5A5

halt:
    j halt
    /* Padding runway in case the IF stage prefetches sequentially ahead of
     * halt's self-jump before it resolves - keeps any such speculative
     * fetch from reading genuinely uninitialized (X in sim) memory. */
    .rept 32
    nop
    .endr
