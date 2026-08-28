/*
 * Platform port of EEMBC CoreMark for the custom RV32IMC 5-stage pipelined
 * RISC-V CPU (Zynq-7020 FPGA soft-core). Based on the upstream barebones
 * port template (coremark/barebones/core_portme.c).
 *
 * Timing uses clint_timer.v's mtime register directly: mtime increments by
 * exactly 1 every clock cycle (see clint_timer.v: "mtime <= mtime + 1"
 * unconditionally, every cycle), so it is a genuine, cycle-accurate
 * free-running counter - no separate cycle CSR needed (this CPU doesn't
 * implement mcycle/minstret).
 *
 * Because mtime counts CYCLES, CLOCKS_PER_SEC below must track the actual
 * PL clock. Getting it wrong does not fail loudly - it silently scales the
 * reported Iterations/Sec, so a clock increase can appear to do nothing at
 * all while the tick count quietly proves otherwise.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "coremark.h"
#include "core_portme.h"

#define CLINT_MTIME_LO (*(volatile ee_u32 *)0x02000000)

/* Must match PS7 FCLK_CLK0 and uart_mmio.v's CLK_FREQ parameter (and
 * Zephyr's SYS_CLOCK_HW_CYCLES_PER_SEC in Kconfig.defconfig). */
#define CLOCKS_PER_SEC 60000000

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
/* Default iteration count (used since MAIN_HAS_NOARGC=1 means there's no
 * argv[4] to override it) - overridable at compile time with
 * -DITERATIONS=N. */
#ifndef ITERATIONS
#define ITERATIONS 0
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

CORETIMETYPE
barebones_clock(void)
{
    return (CORETIMETYPE)CLINT_MTIME_LO;
}

#define GETMYTIME(_t)              (*_t = barebones_clock())
#define MYTIMEDIFF(fin, ini)       ((fin) - (ini))
#define TIMER_RES_DIVIDER          1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC           (CLOCKS_PER_SEC / TIMER_RES_DIVIDER)

static CORETIMETYPE start_time_val, stop_time_val;

void
start_time(void)
{
    GETMYTIME(&start_time_val);
}

void
stop_time(void)
{
    GETMYTIME(&stop_time_val);
}

CORE_TICKS
get_time(void)
{
    CORE_TICKS elapsed
        = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
    return elapsed;
}

secs_ret
time_in_secs(CORE_TICKS ticks)
{
    secs_ret retval = ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
    return retval;
}

ee_u32 default_num_contexts = 1;

void
portable_init(core_portable *p, int *argc, char *argv[])
{
    /* No board init needed here - crt0.S already set up sp/bss before
     * calling main(), and uart_mmio.v's baud rate is fixed in hardware at
     * synthesis time (uart_tx.v's CLK_FREQ/BAUD_RATE params), so there is
     * no runtime UART config step the way a real UART peripheral might
     * need. */
    (void)argc;
    (void)argv;

    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf(
            "ERROR! Please define ee_ptr_int to a type that holds a "
            "pointer!\n");
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    }
    p->portable_id = 1;
}

void
portable_fini(core_portable *p)
{
    p->portable_id = 0;
}
