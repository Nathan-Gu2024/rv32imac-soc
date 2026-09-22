/*
 * INT8 systolic-array GEMM accelerator driver (src/mm_accel.v).
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS DRIVER IS FOR
 *
 * The accelerator was previously driven by bare-metal code poking MMIO and
 * spinning on STATUS - 23 busy-wait loops in fpga/tests/test_mm_accel.c, and a
 * 4-tile batch spends about 1310 cycles doing nothing but re-reading a status
 * word. Here the completion arrives as an interrupt, a thread blocks on a
 * semaphore, and the scheduler gets those cycles back.
 *
 * WHAT STAYS IN HARDWARE
 *
 * Only control goes through this driver. Operands and results move over
 * mem_arbiter's third 128-bit port straight to DRAM and never touch the CPU -
 * that is why the accelerator is fast, and routing the data through the kernel
 * would undo it. What the driver owns is submission, completion, mutual
 * exclusion between threads, and cache coherence.
 *
 * COHERENCE IS THE DRIVER'S JOB, NOT THE CALLER'S
 *
 * That DMA is behind the write-back D-cache, so operands written by the CPU
 * can still be sitting dirty in cache when the accelerator reads DRAM, and
 * results written by the accelerator are shadowed by stale cache lines. Every
 * previous caller open-coded a full-cache eviction loop and had to remember to
 * do it in the right places. gemm_run_batch() does it, in the order that is
 * safe on hardware that cannot invalidate without writing back:
 *
 *   flush operands      -> dirty lines reach DRAM before the fetch
 *   ... batch runs ...
 *   displace dests      -> target lines are gone, so the CPU refetches
 *
 * The destination pass must come AFTER the batch and the operand pass BEFORE
 * it. Displacing a dirty destination line after the DMA has written DRAM would
 * push stale CPU data over the results - that exact mistake once made every
 * result read back as zero.
 */

#define DT_DRV_COMPAT rv32_5stage_gemm

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/irq_nextlevel.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>

#include <rv32_5stage/gemm.h>

/* ---- register map (word index * 4), see chisel/src/MmAccel.scala ---- */
#define REG_CTRL         0x00
#define REG_STATUS       0x04
#define REG_INFO         0x24
#define REG_DEST_STRIDE  0x2C
#define REG_DESC_A_SRC   0x44
#define REG_DESC_B_SRC   0x48
#define REG_DESC_DEST    0x4C
#define REG_DESC_PUSH    0x50
#define REG_QUEUE_FREE   0x54
#define REG_OUT_CTRL     0x58

#define CTRL_SOFT_RST    BIT(1)
#define CTRL_START_QUEUE BIT(5)

#define ST_QUEUE_BUSY    BIT(6)
#define ST_QUEUE_DONE    BIT(7)

#define OUT_INT8         BIT(8)

/* DESC_PUSH: {bOnly[16], panelLoad[15:12], panelUse[11:8], kLen[7:0]} */
#define DESC_BONLY       BIT(16)

struct gemm_config {
	uint32_t base;
	const struct device *intc_dev;
	uint8_t batch_irq;          /* local (level-2) source number */
	uint8_t dma_irq;
	void (*irq_config_func)(const struct device *dev);
};

struct gemm_data {
	struct k_sem done;          /* given by the ISR on batch completion */
	struct k_mutex lock;        /* one batch in flight at a time */
	struct gemm_info info;
	struct gemm_stats stats;
	bool poll_mode;
	/* Written in interrupt context, read by the submitting thread. Two
	 * fields rather than a sentinel timestamp: 0 is a legal value of
	 * k_cycle_get_32(), so "did the ISR run" has to be its own flag or a
	 * batch that completed at cycle 0 would be discarded as stale.
	 */
	volatile uint32_t t_isr;
	volatile bool     isr_fired;
};

static inline uint32_t rd(const struct gemm_config *cfg, uint32_t off)
{
	return sys_read32(cfg->base + off);
}

static inline void wr(const struct gemm_config *cfg, uint32_t off, uint32_t v)
{
	sys_write32(v, cfg->base + off);
}

static void gemm_isr(const struct device *dev)
{
	const struct gemm_config *cfg = dev->config;
	struct gemm_data *data = dev->data;

	/* The aggregator has already cleared its pending bit before dispatching
	 * (intc_rv32_5stage.c), and the accelerator's qDone is cleared by the
	 * next queue kick rather than by an acknowledge register - so there is
	 * nothing to ack here. Just release the waiter.
	 */
	ARG_UNUSED(cfg);

	/* Timestamp BEFORE k_sem_give. The whole point of this measurement is to
	 * separate the wake path from the accelerator's runtime, and k_sem_give
	 * from an ISR can mark a higher-priority thread ready - so anything
	 * sampled after it has already absorbed part of what is being measured.
	 *
	 * t_isr is published before isr_fired so the reader can never see the
	 * flag set alongside a stale timestamp.
	 */
	data->t_isr = k_cycle_get_32();
	data->isr_fired = true;

	k_sem_give(&data->done);
}

void gemm_get_info(const struct device *dev, struct gemm_info *info)
{
	struct gemm_data *data = dev->data;

	*info = data->info;
}

int gemm_flush_operands(const struct device *dev, const struct gemm_tile *tiles,
			size_t n_tiles)
{
	struct gemm_data *data = dev->data;
	size_t bytes;

	if (tiles == NULL || n_tiles == 0) {
		return -EINVAL;
	}

	for (size_t i = 0; i < n_tiles; i++) {
		bytes = (size_t)data->info.dim * tiles[i].k_len;
		if (tiles[i].a_src != NULL) {
			sys_cache_data_flush_range((void *)tiles[i].a_src, bytes);
		}
		if (tiles[i].b_src != NULL) {
			sys_cache_data_flush_range((void *)tiles[i].b_src, bytes);
		}
	}
	return 0;
}

void gemm_set_poll_mode(const struct device *dev, bool poll)
{
	struct gemm_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->poll_mode = poll;
	k_mutex_unlock(&data->lock);
}

void gemm_get_stats(const struct device *dev, struct gemm_stats *st)
{
	struct gemm_data *data = dev->data;

	*st = data->stats;
}

void gemm_reset_stats(const struct device *dev)
{
	struct gemm_data *data = dev->data;

	memset(&data->stats, 0, sizeof(data->stats));
}

int gemm_run_batch(const struct device *dev, const struct gemm_tile *tiles,
		   size_t n_tiles, size_t dest_stride, bool out_int8,
		   uint8_t out_shift, k_timeout_t timeout)
{
	const struct gemm_config *cfg = dev->config;
	struct gemm_data *data = dev->data;
	size_t tile_bytes;
	size_t operand_bytes;
	int rc = 0;

	if (tiles == NULL || n_tiles == 0) {
		return -EINVAL;
	}
	if (n_tiles > rd(cfg, REG_QUEUE_FREE)) {
		return -EBUSY;   /* descriptor queue cannot hold the batch */
	}

	/* An INT8 tile is dim*dim contiguous bytes: a 128-bit line spans
	 * 16/dim result rows, so there is no row boundary to stride at.
	 */
	if (out_int8 && dest_stride != 0) {
		return -EINVAL;
	}

	tile_bytes = out_int8 ? (size_t)data->info.dim * data->info.dim
			      : (size_t)data->info.dim * data->info.dim * 4;

	k_mutex_lock(&data->lock, K_FOREVER);

	uint32_t t_mark = k_cycle_get_32();
	uint32_t flush_in_submit = 0;

	wr(cfg, REG_DEST_STRIDE, dest_stride);
	wr(cfg, REG_OUT_CTRL, out_int8 ? (OUT_INT8 | (out_shift & 0x1F)) : 0);

	for (size_t i = 0; i < n_tiles; i++) {
		const struct gemm_tile *t = &tiles[i];

		/* Push operands out before the accelerator reads DRAM. A b_only
		 * tile reuses the resident A panel and does not refetch it, so
		 * flushing A again would be wasted work.
		 */
		operand_bytes = (size_t)data->info.dim * t->k_len;
		uint32_t t_flush = k_cycle_get_32();
		if (t->src_coherent) {
			/* Caller guarantees these have not been written since
			 * the last gemm_flush_operands(). Skipping this is the
			 * single largest saving available per batch.
			 */
		} else if (!t->b_only && t->a_src != NULL) {
			sys_cache_data_flush_range((void *)t->a_src, operand_bytes);
		}
		if (t->b_src != NULL) {
			sys_cache_data_flush_range((void *)t->b_src, operand_bytes);
		}
		flush_in_submit += k_cycle_get_32() - t_flush;

		wr(cfg, REG_DESC_A_SRC, (uint32_t)(uintptr_t)t->a_src);
		wr(cfg, REG_DESC_B_SRC, (uint32_t)(uintptr_t)t->b_src);
		wr(cfg, REG_DESC_DEST, (uint32_t)(uintptr_t)t->dest);
		wr(cfg, REG_DESC_PUSH,
		   (t->b_only ? DESC_BONLY : 0u) |
		   ((uint32_t)(t->panel_load & 0xF) << 12) |
		   ((uint32_t)(t->panel_use & 0xF) << 8) |
		   (uint32_t)(t->k_len & 0xFF));
	}

	/* Submit time EXCLUDING the operand flushes done inside the loop, so
	 * the reported figures partition the batch instead of overlapping.
	 */
	data->stats.submit_cycles += (k_cycle_get_32() - t_mark) - flush_in_submit;
	data->stats.flush_cycles += flush_in_submit;

	k_sem_reset(&data->done);
	data->isr_fired = false;
	wr(cfg, REG_CTRL, CTRL_START_QUEUE);
	uint32_t t_kick = k_cycle_get_32();

	/* Counted BEFORE the take. k_sem_take with a timeout has to insert a
	 * timeout record and reprogram the timer before it can pend, which is
	 * far from free - if the completion interrupt lands during that
	 * prologue the call returns on the fast path and never yields.
	 */
	if (k_sem_count_get(&data->done) > 0) {
		data->stats.already_done++;
	}

	t_mark = k_cycle_get_32();
	if (data->poll_mode) {
		/* The bare-metal behaviour, kept honest: burn CPU reading a
		 * status word. The ISR still runs and still gives the
		 * semaphore - it is reset above and simply ignored here - so
		 * the only difference between the two modes is whether this
		 * thread yields the CPU while the accelerator works.
		 */
		while (!(rd(cfg, REG_STATUS) & ST_QUEUE_DONE)) {
		}
	} else if (k_sem_take(&data->done, timeout) != 0) {
		data->stats.wait_cycles += k_cycle_get_32() - t_mark;
		/* The batch never signalled. Leave the hardware quiescent rather
		 * than letting a late interrupt land on the next caller's
		 * semaphore.
		 */
		wr(cfg, REG_CTRL, CTRL_SOFT_RST);
		rc = -ETIMEDOUT;
		goto out;
	}

	uint32_t t_resume = k_cycle_get_32();

	data->stats.wait_cycles += t_resume - t_mark;
	data->stats.batches++;

	/* Split the wait into the part the hardware owns and the part the kernel
	 * owns. Without this split the two are one number, and a slow wake path
	 * is indistinguishable from a slow accelerator.
	 *
	 * In polling mode the ISR still runs (see the note above), so both halves
	 * are meaningful there too: isr_cycles should match the interrupt run -
	 * same hardware, same work - and resume_cycles becomes the spin loop's
	 * own reaction time, which is the bar the wake path has to beat.
	 */
	if (data->isr_fired) {
		data->stats.isr_cycles    += data->t_isr - t_kick;
		data->stats.resume_cycles += t_resume - data->t_isr;
		data->stats.isr_seen++;
	}

	/* Results are in DRAM now; drop the stale lines shadowing them. */
	t_mark = k_cycle_get_32();
	for (size_t i = 0; i < n_tiles; i++) {
		if (tiles[i].dest != NULL) {
			sys_cache_data_invd_range(tiles[i].dest, tile_bytes);
		}
	}
	data->stats.flush_cycles += k_cycle_get_32() - t_mark;

out:
	k_mutex_unlock(&data->lock);
	return rc;
}

static int gemm_init(const struct device *dev)
{
	const struct gemm_config *cfg = dev->config;
	struct gemm_data *data = dev->data;
	uint32_t info;

	k_sem_init(&data->done, 0, 1);
	k_mutex_init(&data->lock);

	info = rd(cfg, REG_INFO);
	data->info.dim = info & 0xFF;
	data->info.max_k = (info >> 8) & 0xFF;
	data->info.panels = (info >> 16) & 0xFF;

	cfg->irq_config_func(dev);

	/* Only the batch line is unmasked. dmaDone is restarted by the queue
	 * sequencer once per TILE, so enabling source 3 in queue mode would
	 * enter the ISR once per tile for a completion nobody is waiting on -
	 * tb_mm_accel_queue.v asserts exactly that asymmetry (1 batch edge,
	 * NTILE dma edges).
	 */
	irq_enable_next_level(cfg->intc_dev, cfg->batch_irq);

	return 0;
}

/* DT_INST_IRQN_BY_NAME gives the multi-level-encoded Zephyr IRQ number; this
 * recovers the local level-2 bit position from it. Copied from
 * uart_rv32_5stage.c rather than using irq_from_level_2(), which goes through
 * a union of bitfields and so is not a constant expression usable in a static
 * initializer.
 */
#define GEMM_IRQ_FROM_L2(irq)                                                                      \
	((((irq) >> CONFIG_1ST_LEVEL_INTERRUPT_BITS) &                                             \
	  BIT_MASK(CONFIG_2ND_LEVEL_INTERRUPT_BITS)) - 1)

#define GEMM_INIT(inst)                                                                            \
	static void gemm_irq_config_##inst(const struct device *dev)                               \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, batch), 0, gemm_isr,                        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
	}                                                                                          \
	static struct gemm_data gemm_data_##inst;                                                  \
	static const struct gemm_config gemm_cfg_##inst = {                                        \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.intc_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), interrupt_parent)),        \
		.batch_irq = GEMM_IRQ_FROM_L2(DT_INST_IRQN_BY_NAME(inst, batch)),                  \
		.dma_irq = GEMM_IRQ_FROM_L2(DT_INST_IRQN_BY_NAME(inst, dma)),                      \
		.irq_config_func = gemm_irq_config_##inst,                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, gemm_init, NULL, &gemm_data_##inst, &gemm_cfg_##inst,          \
			      POST_KERNEL, CONFIG_GEMM_RV32_5STAGE_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(GEMM_INIT)
