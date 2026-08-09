.equ CLINT_BASE,   0x02000000
.equ MTIME_LO,     0x0
.equ MTIME_HI,     0x4
.equ MTIMECMP_LO,  0x8
.equ MTIMECMP_HI,  0xC

.equ MTIE_BIT,     0x80   # mie.MTIE is bit 7
.equ MIE_BIT,      0x8    # mstatus.MIE is bit 3

.equ LED_ADDR,     0x00002000
.equ TICK_DELTA,   25000000  # ~0.5s at 50MHz - visible blink rate on hardware

# DDR-resident counterpart to timer_isr_test.s: identical logic, but
# linked to run from DDR (via linker_ddr.ld) instead of TCM. The point is
# to exercise the DDR/AXI icache path for an INTERRUPT-DRIVEN fetch, not
# just straight-line execution - mtvec points into DDR here, so when the
# timer trap fires, the CPU has to fetch trap_handler's instructions
# through icache.v's cache-backed path (mem_arbiter/axi_cache_adapter),
# the same path whose halfword-stitching bug was fixed earlier, rather
# than through tcm.v like every previous interrupt test used.
.section .text
.globl _start

_start:
    # 1. Setup trap vector - a DDR address this time, not TCM
    la t0, trap_handler
    csrw mtvec, t0

    # 2. Arm the first tick: mtimecmp = current mtime + TICK_DELTA, full
    # 64-bit add (see timer_isr_test.s for why the carry matters).
    li t0, CLINT_BASE
    lw t1, MTIME_LO(t0)
    lw t5, MTIME_HI(t0)
    li t2, TICK_DELTA
    add t6, t1, t2
    sltu t4, t6, t1
    add t5, t5, t4
    sw t6, MTIMECMP_LO(t0)
    sw t5, MTIMECMP_HI(t0)

    # 3. Enable mie.MTIE
    li t0, MTIE_BIT
    csrs mie, t0

    # 4. Enable mstatus.MIE
    li t0, MIE_BIT
    csrs mstatus, t0

wait_loop:
    j wait_loop

.align 4
trap_handler:
    csrr t0, mcause
    li t1, 0x80000007
    bne t0, t1, trap_end

    # Rearm relative to current mtime - full 64-bit add, same as the
    # initial arm above.
    li t0, CLINT_BASE
    lw t1, MTIME_LO(t0)
    lw t5, MTIME_HI(t0)
    li t2, TICK_DELTA
    add t6, t1, t2
    sltu t4, t6, t1
    add t5, t5, t4
    sw t6, MTIMECMP_LO(t0)
    sw t5, MTIMECMP_HI(t0)

    la t3, tick_count
    lw t4, 0(t3)
    addi t4, t4, 1
    sw t4, 0(t3)
    li t0, LED_ADDR
    sw t4, 0(t0)

trap_end:
    mret

.section .data
.align 2
tick_count:
    .word 0
