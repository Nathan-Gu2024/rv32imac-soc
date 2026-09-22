/*
 * Public API for the RV32 5-stage INT8 GEMM accelerator.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_RV32_5STAGE_GEMM_H_
#define ZEPHYR_INCLUDE_RV32_5STAGE_GEMM_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Geometry fixed at Chisel elaboration, read back from the INFO register. */
struct gemm_info {
	uint8_t dim;      /**< array is dim x dim */
	uint8_t max_k;    /**< deepest reduction the operand buffers hold */
	uint8_t panels;   /**< B scratchpad panels resident at once */
};

/** One output tile: C = A * B, with B taken from a scratchpad panel. */
struct gemm_tile {
	const void *a_src;   /**< A row-panel, dim lanes of k_len bytes, packed */
	const void *b_src;   /**< B column-panel, same shape */
	void       *dest;    /**< where the result DMA writes */
	uint16_t    k_len;   /**< reduction depth, <= max_k */
	uint8_t     panel_use;   /**< scratchpad panel the array reads */
	uint8_t     panel_load;  /**< scratchpad panel this tile's B lands in */
	bool        b_only;      /**< reuse the resident A panel, fetch only B */
	/**
	 * The operand buffers are already coherent with DRAM and this batch
	 * need not flush them.
	 *
	 * Set this ONLY if nothing has written a_src or b_src since the last
	 * gemm_flush_operands(). It is the caller's guarantee, not something
	 * the driver can check: the same pointer with different contents is
	 * indistinguishable from the same pointer unchanged.
	 *
	 * Worth setting. Operand flushing is ~160 of the ~224 cache lines a
	 * 4-tile batch walks, and staged panels are typically written once and
	 * reused across many batches.
	 */
	bool        src_coherent;
};

/** Read the geometry the hardware was built with. */
void gemm_get_info(const struct device *dev, struct gemm_info *info);

/**
 * Where a batch's time actually went.
 *
 * The point of the interrupt path is that the submitting thread SLEEPS while
 * the accelerator works, so another thread can run. @ref wait_cycles is the
 * evidence: if it is near zero the completion arrived before the thread
 * reached k_sem_take() and nothing was ever yielded, which looks identical
 * from the outside to a scheduler that simply never ran the other thread.
 */
struct gemm_stats {
	uint32_t batches;        /**< batches submitted */
	uint32_t wait_cycles;    /**< cycles spent inside k_sem_take() */
	uint32_t flush_cycles;   /**< cycles spent on cache maintenance */
	uint32_t submit_cycles;  /**< cycles spent pushing descriptors */
	/** Batches whose completion had ALREADY arrived before the submitting
	 *  thread reached k_sem_take. Those never pend, so the CPU is never
	 *  yielded and no other thread can run - which is indistinguishable
	 *  from a scheduler that refuses to pick one, unless it is counted.
	 */
	uint32_t already_done;
	/** Cycles from the queue kick to ISR entry.
	 *
	 * This is the accelerator's own busy time plus the interrupt plumbing,
	 * and the hardware does not know how the CPU is waiting - so it should
	 * come out the SAME in both wait modes. That makes it the cross-check
	 * that an A/B really did identical work: if it differs, the two runs are
	 * not comparable and nothing else in the table means anything.
	 */
	uint32_t isr_cycles;
	/** Cycles from ISR entry to the submitting thread running again.
	 *
	 * In interrupt mode this is the pure wake path - k_sem_give in interrupt
	 * context, reschedule, switch back - and it is the quantity that decides
	 * whether sleeping can ever beat spinning. In polling mode it is how long
	 * the spin loop takes to notice STATUS, which is the bar the interrupt
	 * has to clear.
	 */
	uint32_t resume_cycles;
	/** Batches where the ISR was actually observed.
	 *
	 * Divisor for the two figures above. Without it a batch whose interrupt
	 * never landed would silently contribute a stale timestamp to the average
	 * instead of being excluded.
	 */
	uint32_t isr_seen;
};

/**
 * Choose how gemm_run_batch() waits for a batch to finish.
 *
 * @param poll  true  - spin on the STATUS register, exactly as bare-metal
 *                      code does. The CPU is never released.
 *              false - sleep on a semaphore released by the completion
 *                      interrupt (the default).
 *
 * The hardware is identical either way - the interrupt still fires, the
 * accelerator still takes the same time. Only the submitting thread's way of
 * noticing differs, which is what makes the two modes comparable.
 */
/**
 * Push the operand buffers of @p tiles out to DRAM.
 *
 * Call once after staging operands, then set src_coherent on the tiles so the
 * per-batch flush is skipped. Separating this from gemm_run_batch() is the
 * difference between paying for operand coherence once and paying per batch.
 */
int gemm_flush_operands(const struct device *dev, const struct gemm_tile *tiles,
			size_t n_tiles);

void gemm_set_poll_mode(const struct device *dev, bool poll);

void gemm_get_stats(const struct device *dev, struct gemm_stats *st);
void gemm_reset_stats(const struct device *dev);

/**
 * Run a batch of tiles and block until the whole batch completes.
 *
 * Coherence is handled here: operand ranges are flushed before the
 * accelerator reads them and destination ranges are displaced before the
 * caller reads results, because the DMA moves data behind the write-back
 * D-cache. Callers do not need their own cache maintenance.
 *
 * @param out_int8  produce requantized INT8 output, shifted right by
 *                  @p out_shift and saturated, instead of raw INT32
 *                  accumulators. An INT8 tile is dim*dim contiguous bytes
 *                  and must be written to a contiguous destination.
 *
 * @retval 0 on success, -EINVAL on a bad argument, -EBUSY if the accelerator
 *         did not accept the batch, -ETIMEDOUT if it never signalled done.
 */
int gemm_run_batch(const struct device *dev, const struct gemm_tile *tiles,
		   size_t n_tiles, size_t dest_stride, bool out_int8,
		   uint8_t out_shift, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_RV32_5STAGE_GEMM_H_ */
