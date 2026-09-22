/*
 * Cache maintenance for the RV32 5-stage custom SoC.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHY THIS EXISTS
 *
 * The GEMM accelerator (src/mm_accel.v) reads its operands and writes its
 * results over mem_arbiter's third 128-bit port, which sits BEHIND the
 * write-back D-cache. So CPU-written operands sitting dirty in the cache are
 * invisible to it, and DMA-written results are shadowed by stale cache lines.
 * Until now every caller open-coded a full-cache eviction loop (see
 * dcache_evict() in fpga/tests/test_mm_accel.c) and any Zephyr thread driving
 * the accelerator would have had to do the same.
 *
 * HOW IT WORKS, GIVEN THE HARDWARE HAS NO MAINTENANCE INSTRUCTIONS
 *
 * There is no cbo.flush, no fence.i, no maintenance CSR and no uncached DRAM
 * window - every MMIO decode in cpu.v is a device window. What the hardware
 * does have is a DIRECT-MAPPED write-back D-cache, and that is enough:
 * touching any address that maps to the same set as a target line displaces
 * that line, and the existing S_CAP -> S_WB -> S_FILL path in dcache_bram.v
 * writes it back if it was dirty. The eviction is a side effect of an
 * ordinary load, so dcache_valid is set and dmem_stall covers the pipeline
 * stall for free - no new RTL, no new stall source, no FSM state.
 *
 * The improvement over the open-coded version is that this walks only the
 * sets the RANGE occupies instead of all of them. A 256-byte result tile
 * touches 16 sets rather than 1024.
 *
 * WHAT IT CANNOT DO, AND WHY invd_range IS NOT WHAT ITS NAME SUGGESTS
 *
 * Displacement always writes a dirty line back; there is no way to discard
 * one. So invalidate cannot be separated from flush here, and
 * cache_data_invd_range() below is the same walk as the flush.
 *
 * That matters for the DMA-read case, and getting it wrong has already cost
 * a debug cycle on this project: if the CPU has dirty lines covering a DMA
 * DESTINATION, "invalidating" them writes that stale data over the DMA
 * results. Pre-zeroing a destination buffer and then reading it back made
 * every result come out as 0 for exactly this reason.
 *
 * The usage that is safe, and the one the accelerator driver follows:
 *
 *   flush_range(dst)   before starting the DMA   - pushes dirty lines out and
 *                                                  leaves the sets holding
 *                                                  this driver's own lines
 *   ... accelerator DMA writes DRAM ...
 *   invd_range(dst)    before reading results    - the target lines are no
 *                                                  longer resident, so the
 *                                                  CPU refetches from DRAM
 *
 * After the first call the target lines are gone from the cache, so the
 * second call cannot write anything stale back over the DMA.
 */

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/util.h>

#define DC_LINE  CONFIG_DCACHE_LINE_SIZE
#define DC_BYTES CONFIG_CACHE_RV32_5STAGE_DCACHE_BYTES
#define DC_SETS  (DC_BYTES / DC_LINE)

BUILD_ASSERT(DC_BYTES % DC_LINE == 0,
	     "D-cache size must be a whole number of lines");
BUILD_ASSERT((DC_SETS & (DC_SETS - 1)) == 0,
	     "set count must be a power of two: the set index is address bits");

/*
 * One line per set, aligned to the whole cache so that displace_buf[set *
 * DC_LINE] lands in exactly `set`. The alignment is load-bearing: at merely
 * line alignment the buffer starts at an arbitrary set and every displacement
 * would target the wrong one - and silently, because a load that misses the
 * intended set still succeeds, it just evicts nothing useful.
 */
static volatile uint8_t displace_buf[DC_BYTES] __aligned(DC_BYTES);

static void displace_range(uintptr_t addr, size_t size)
{
	volatile uint8_t sink = 0;
	uintptr_t a = ROUND_DOWN(addr, DC_LINE);
	uintptr_t end = addr + size;

	for (; a < end; a += DC_LINE) {
		size_t set = (a / DC_LINE) & (DC_SETS - 1);

		sink ^= displace_buf[set * DC_LINE];
	}

	(void)sink;
}

static void displace_all(void)
{
	volatile uint8_t sink = 0;

	for (size_t set = 0; set < DC_SETS; set++) {
		sink ^= displace_buf[set * DC_LINE];
	}

	(void)sink;
}

/* ---- data cache ---- */

void cache_data_enable(void)
{
	/* Always on; there is no enable bit in the hardware. */
}

void cache_data_disable(void)
{
	/* Cannot be disabled. Deliberately not an error: callers use this
	 * defensively and failing here would be worse than doing nothing.
	 */
}

int cache_data_flush_all(void)
{
	displace_all();
	return 0;
}

int cache_data_invd_all(void)
{
	/* See the header comment: this writes dirty lines back rather than
	 * discarding them. Same walk as the flush.
	 */
	displace_all();
	return 0;
}

int cache_data_flush_and_invd_all(void)
{
	displace_all();
	return 0;
}

int cache_data_flush_range(void *addr, size_t size)
{
	displace_range((uintptr_t)addr, size);
	return 0;
}

int cache_data_invd_range(void *addr, size_t size)
{
	/* NOT a true invalidate - see the header comment. Safe only when the
	 * range holds no dirty CPU lines, which is what a preceding
	 * cache_data_flush_range() guarantees.
	 */
	displace_range((uintptr_t)addr, size);
	return 0;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
	displace_range((uintptr_t)addr, size);
	return 0;
}

size_t cache_data_line_size_get(void)
{
	return DC_LINE;
}

/* ---- instruction cache ---- */
/*
 * icache_bram.v has no CPU-side write path at all - the only writes are
 * refills, and its header says so explicitly. Its reset sweep would serve as
 * a whole-cache invalidate hook, but nothing reaches it from software yet, so
 * these report unsupported rather than silently doing nothing. Nothing on
 * this SoC loads code at runtime, so no caller needs them today.
 */

void cache_instr_enable(void)
{
}

void cache_instr_disable(void)
{
}

int cache_instr_flush_all(void)
{
	return -ENOTSUP;
}

int cache_instr_invd_all(void)
{
	return -ENOTSUP;
}

int cache_instr_flush_and_invd_all(void)
{
	return -ENOTSUP;
}

int cache_instr_flush_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
	return -ENOTSUP;
}

int cache_instr_invd_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
	return -ENOTSUP;
}

int cache_instr_flush_and_invd_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
	return -ENOTSUP;
}

size_t cache_instr_line_size_get(void)
{
	return DC_LINE;
}
