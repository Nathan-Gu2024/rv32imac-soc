.equ CLINT_BASE,   0x02000000
.equ MTIME_LO,     0x0
.equ MTIME_HI,     0x4
.equ MTIMECMP_LO,  0x8
.equ MTIMECMP_HI,  0xC

.equ MTIE_BIT,     0x80   # mie.MTIE is bit 7
.equ MIE_BIT,      0x8    # mstatus.MIE is bit 3

.equ LED_ADDR,     0x00002000
.equ TICK_DELTA,   25000000  # ~0.5s at 50MHz - visible blink rate on hardware

.section .text
.globl _start

_start:
    # 1. Setup trap vector
    la t0, trap_handler
    csrw mtvec, t0

    # 2. Arm the first tick: mtimecmp = current mtime + TICK_DELTA, as a
    # proper 64-bit add (mtime is a genuine free-running 64-bit counter -
    # its low word alone wraps every 2^32 cycles, ~86s at 50MHz - so a
    # 32-bit-only add here would silently produce a too-small mtimecmp
    # once mtime approaches that boundary, not just at boot).
    li t0, CLINT_BASE
    lw t1, MTIME_LO(t0)
    lw t5, MTIME_HI(t0)
    li t2, TICK_DELTA
    add t6, t1, t2
    sltu t4, t6, t1     # t4 = 1 if the low-word add overflowed (carry)
    add t5, t5, t4
    sw t6, MTIMECMP_LO(t0)
    sw t5, MTIMECMP_HI(t0)

    # 3. Enable mie.MTIE (Machine Timer Interrupt Enable)
    li t0, MTIE_BIT
    csrs mie, t0

    # 4. Enable mstatus.MIE - arms the CPU to actually take the trap
    li t0, MIE_BIT
    csrs mstatus, t0

wait_loop:
    j wait_loop

.align 4
trap_handler:
    # Confirm this is really the timer (mcause == 0x80000007), not some
    # other trap sharing this vector.
    csrr t0, mcause
    li t1, 0x80000007
    bne t0, t1, trap_end

    # Rearm the NEXT tick relative to the CURRENT mtime (periodic, not
    # one-shot), as a full 64-bit add - see the comment at the initial arm
    # above for why the carry into the high word matters here too. Getting
    # this wrong doesn't just mistime one tick: once mtime[63:32] genuinely
    # increments (~86s in) while mtimecmp[63:32] stays wrong, the 64-bit
    # comparison mtime>=mtimecmp becomes permanently true and the ISR has
    # no way to ever clear it again - a storm with no recovery.
    #
    # timer_interrupt is level-triggered on mtime>=mtimecmp - unlike the
    # edge-latched INTC pending bit there's no separate "pending" register
    # to acknowledge; rearming mtimecmp past the current mtime IS the
    # acknowledgment, and is required before mret restores mstatus.MIE or
    # the interrupt refires instantly.
    li t0, CLINT_BASE
    lw t1, MTIME_LO(t0)
    lw t5, MTIME_HI(t0)
    li t2, TICK_DELTA
    add t6, t1, t2
    sltu t4, t6, t1
    add t5, t5, t4
    sw t6, MTIMECMP_LO(t0)
    sw t5, MTIMECMP_HI(t0)

    # Bump and display a tick counter on the LEDs (register is 4 bits wide,
    # so this visibly wraps 0-15 every 16 ticks - proof the timer keeps
    # firing reliably, not just once).
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
