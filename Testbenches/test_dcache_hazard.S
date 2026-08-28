/* Directed store->load hazard test for the BRAM data cache.
 *
 * dcache_bram.v addresses its arrays with the address MEM will use NEXT
 * cycle (speculated in EX), so a store's write edge and the speculative
 * read for the very next access can target the same set in the same cycle.
 * Cross-port read-during-write on Xilinx SDP BRAM is UNDEFINED, so without
 * the one-deep bypass in dcache_bram.v the following load silently returns
 * stale data. Neither tb_amo_test nor CoreMark reliably hits this, hence
 * this test.
 *
 * Every load below is placed IMMEDIATELY after the store it depends on, so
 * the store is in MEM exactly when the load's address is being speculated
 * in EX - the precise collision the bypass exists to cover.
 *
 * Register map (checked via hierarchical DUT.RF.regs[] peek, same pattern
 * as test_amo.S):
 *   x5  = test address (0x00180000, scratch inside the 2MB DDR mock window)
 *   x10 = word read back immediately after a word store   -> 0xDEADBEEF
 *   x11 = word read back immediately after a byte store   -> 0xDEADBEAA
 *   x12 = word read back immediately after a halfword store -> 0xDEAD1234
 *   x13 = second word of the same line, after storing it   -> 0x12345678
 *   x14 = re-read of the first word (still 0xDEAD1234)
 *   x15 = word in a DIFFERENT set, stored then loaded      -> 0xCAFEBABE
 *   x16 = re-read of the first word after the other-set access -> 0xDEAD1234
 *   x1  = 0xA5A5A5A5 sentinel, written last
 */
.section .text.init
.global _start

_start:
    li      x5, 0x00180000

    /* 1. word store -> immediate word load (the core hazard) */
    li      x2, 0xDEADBEEF
    sw      x2, 0(x5)
    lw      x10, 0(x5)

    /* 2. byte store -> immediate word load. Also checks the bypass is
     *    applied PER BYTE: only the low byte may change. */
    li      x3, 0xAA
    sb      x3, 0(x5)
    lw      x11, 0(x5)

    /* 3. halfword store -> immediate word load, upper half must survive */
    li      x4, 0x1234
    sh      x4, 0(x5)
    lw      x12, 0(x5)

    /* 4. store to a different word in the SAME line, load it back, then
     *    re-read word 0 - catches a bypass that ignores the word offset
     *    and forwards across banks. */
    li      x6, 0x12345678
    sw      x6, 4(x5)
    lw      x13, 4(x5)
    lw      x14, 0(x5)

    /* 5. +16KB is exactly the cache stride, so this address maps to the
     *    SAME set with a different tag: a conflict miss that must evict the
     *    now-dirty line. x16's re-read below therefore proves the whole
     *    writeback path - stores landed, line went dirty, eviction wrote it
     *    out, refill brought it back intact. Also catches a bypass whose
     *    valid bit is never cleared across the eviction. */
    li      x7, 0x00184000
    li      x8, 0xCAFEBABE
    sw      x8, 0(x7)
    lw      x15, 0(x7)
    lw      x16, 0(x5)

    /* sentinel: testbench waits for this before checking */
    li      x1, 0xA5A5A5A5

halt:
    j       halt
