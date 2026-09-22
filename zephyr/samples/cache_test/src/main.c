/*
 * Cache maintenance smoke test for the RV32 5-stage SoC.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Checks two things the set-displacement implementation could plausibly get
 * wrong, and measures what the range-limited walk is actually worth:
 *
 *   1. A flush must not CORRUPT the range it flushes. Displacement works by
 *      provoking cache misses on purpose, so a wrong set index or a bad
 *      scratch alignment would evict the wrong line - and on a write-back
 *      cache that means writing one line's data under another line's address.
 *      Pattern-check every byte afterwards.
 *
 *   2. flush_range must cost O(range), not O(cache). That is the entire point
 *      of the driver over the open-coded full-cache loop it replaces, so it is
 *      worth measuring rather than assuming.
 *
 * This runs without the accelerator, so it cannot prove DMA coherence - only
 * that the primitive behaves. Coherence is proved by the accelerator test.
 */

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/printk.h>

#define BUF_BYTES 4096
#define LINE      CONFIG_DCACHE_LINE_SIZE

static uint8_t buf[BUF_BYTES] __aligned(LINE);

static uint8_t pattern_of(size_t i)
{
	/* Position-dependent, so a displaced-wrong-line bug shows up as a
	 * mismatch rather than as plausible-looking data.
	 */
	return (uint8_t)((i * 31u + 7u) & 0xFFu);
}

int main(void)
{
	uint32_t t0, t_range, t_all;
	int rc;
	int bad = 0;

	printk("\n=== cache maintenance test ===\n");
	printk("line size %d, configured d-cache %d bytes\n",
	       (int)sys_cache_data_line_size_get(),
	       CONFIG_CACHE_RV32_5STAGE_DCACHE_BYTES);

	for (size_t i = 0; i < BUF_BYTES; i++) {
		buf[i] = pattern_of(i);
	}

	/* ---- 1. correctness ---- */
	rc = sys_cache_data_flush_range(buf, BUF_BYTES);
	if (rc != 0) {
		printk("FAIL: flush_range returned %d\n", rc);
		return 0;
	}

	for (size_t i = 0; i < BUF_BYTES; i++) {
		if (buf[i] != pattern_of(i)) {
			if (bad < 4) {
				printk("  MISMATCH at %u: got %02x want %02x\n",
				       (unsigned)i, buf[i], pattern_of(i));
			}
			bad++;
		}
	}
	printk(bad ? "FAIL: %d bytes corrupted by flush\n"
		   : "PASS: flush preserved all %d bytes\n",
	       bad ? bad : BUF_BYTES);

	/* ---- 2. cost ---- */
	for (size_t i = 0; i < BUF_BYTES; i++) {
		buf[i] = pattern_of(i) ^ 0xFFu;   /* dirty every line again */
	}
	t0 = k_cycle_get_32();
	sys_cache_data_flush_range(buf, BUF_BYTES);
	t_range = k_cycle_get_32() - t0;

	for (size_t i = 0; i < BUF_BYTES; i++) {
		buf[i] = pattern_of(i);
	}
	t0 = k_cycle_get_32();
	sys_cache_data_flush_all();
	t_all = k_cycle_get_32() - t0;

	printk("flush_range(%d B, %d lines)  %u cycles\n",
	       BUF_BYTES, BUF_BYTES / LINE, t_range);
	printk("flush_all  (%d B, %d lines)  %u cycles\n",
	       CONFIG_CACHE_RV32_5STAGE_DCACHE_BYTES,
	       CONFIG_CACHE_RV32_5STAGE_DCACHE_BYTES / LINE, t_all);
	if (t_range) {
		printk("range is %u.%02ux cheaper\n",
		       t_all / t_range, (t_all * 100u / t_range) % 100u);
	}

	printk("=== done ===\n");
	return 0;
}
