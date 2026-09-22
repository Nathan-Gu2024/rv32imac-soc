/*
 * What an interrupt-driven accelerator is actually worth, measured.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two threads share the CPU:
 *
 *   gemm    submits 4-tile batches and waits for completion
 *   worker  runs a fixed integer kernel and counts how much it finished
 *
 * The worker's throughput WHILE the accelerator is running is the whole
 * measurement. Under the bare-metal driver the CPU spins on STATUS for about
 * 1310 cycles per 4-tile batch and that time is simply gone; under the driver
 * the batch-completion interrupt releases a semaphore and the scheduler hands
 * those cycles to the worker instead.
 *
 * Both modes run the SAME accelerator work and the same number of batches, so
 * the difference in worker progress is the reclaimed time and nothing else.
 * The GEMM results are checked in both modes, because a coherence mistake
 * would otherwise show up as a suspiciously fast but wrong run.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <rv32_5stage/gemm.h>

#define BATCHES   16
/* Batch size is swept, because the interrupt-vs-polling answer depends on it:
 * the wake path costs a fixed amount per batch while the accelerator time it
 * hides scales with tiles, so there is a break-even size and the only way to
 * find it is to measure across it.
 *
 * 8 is not a round number, it is the HARDWARE CEILING. The descriptor queue is
 * `descDepth = 8` deep (MmAccel.scala), and gemm_run_batch() refuses a batch
 * larger than REG_QUEUE_FREE with -EBUSY rather than silently truncating it. So
 * this sweep covers the entire available range, and if break-even lands at the
 * top of it then batch size has run out as a lever and the queue depth is the
 * thing to change.
 */
#define MAX_TILES  8
static const int tile_sweep[] = { 1, 2, 4, 8 };
#define KLEN      64
#define STACK     4096
#define PRIO_GEMM   5
/* LOWER priority than gemm, so gemm preempts the instant the completion
 * interrupt fires and the worker only ever fills genuine idle time.
 *
 * Equal priority looked like it worked and did not: gemm cannot preempt an
 * equal-priority running thread, so after every completion it waited a full
 * CONFIG_TIMESLICE_SIZE of 20 ms - 1.2M cycles, measured - and the run
 * inflated 129x. That measures timeslice latency, not accelerator overlap.
 */
#define PRIO_WORK   6

/* Operand panels: lanes packed back to back so the fetch is one INCR burst,
 * and aligned to a whole panel so a 512-byte burst cannot cross a 4 KB
 * boundary, which AXI4 forbids.
 */
#define PANEL_ALIGN 1024
static int8_t  a_panel[16 * KLEN] __aligned(PANEL_ALIGN);
static int8_t  b_panel[MAX_TILES][16 * KLEN] __aligned(PANEL_ALIGN);
/* INT8 output, not INT32 accumulators.
 *
 * A requantized tile is dim*dim BYTES - 64 here, four cache lines - against
 * 256 bytes and sixteen lines for raw accumulators. The destination
 * invalidate cannot be hoisted the way the operand flush was, because the
 * accelerator writes new results every batch, so the only way to make it
 * cheaper is for there to be less of it.
 *
 * shift 6 divides out k_len = 64 exactly, leaving (i+1)*(j+1+p), at most
 * 8*11 = 88 - inside INT8 with no clipping, and still position-dependent so
 * a mis-packed line cannot pass unnoticed.
 */
#define OUT_SHIFT 6
static int8_t dest[MAX_TILES][16 * 16] __aligned(16);

static K_THREAD_STACK_DEFINE(gemm_stack, STACK);
static K_THREAD_STACK_DEFINE(work_stack, STACK);
static struct k_thread gemm_thread, work_thread;

/* Kept only so the worker's loop has an observable side effect. These are NOT
 * the measurement any more: work_polls read zero while the kernel's runtime
 * stats showed the worker executing tens of thousands of cycles, so the
 * counters are not trustworthy and the kernel's accounting is.
 */
static volatile uint32_t work_done;
static volatile uint32_t work_polls;
static volatile uint32_t work_sink;      /* keeps churn() from being elided */
static volatile bool     work_run;

static int dim, k_len;
static int gemm_bad;
static int words_checked;
static int panels;            /* B scratchpad panels the hardware reports */

/* Which timeout the submitting thread waits with.
 *
 * A global rather than a thread argument because k_timeout_t is a struct and
 * k_thread_create only carries word-sized values. It is written before the
 * thread is created and read once inside it, so there is no race.
 *
 * This exists because the first sweep showed the completion interrupt arriving
 * at a near-CONSTANT ~2000 cycles after the queue kick no matter how much work
 * the batch contained, while the same hardware measured 454..2033 under
 * polling. The only code on the interrupt path that polling does not execute is
 * k_sem_take, which locks interrupts across wait-queue insertion, z_add_timeout
 * and the context switch. K_FOREVER skips the timeout insertion, so comparing
 * the two isolates how much of that window z_add_timeout owns.
 */
static k_timeout_t batch_timeout;


/* A small integer kernel with a loop-carried dependency, so it measures CPU
 * time rather than memory bandwidth.
 *
 * "Cannot be optimised away" was wrong the first time: the result fed only a
 * local that nothing ever read, so GCC deleted the whole call and the loop
 * collapsed to a single volatile increment - 8.01 cycles per iteration, which
 * is what gave it away. The result now reaches a volatile every iteration, so
 * the dependency chain has to be computed.
 */
static uint32_t churn(uint32_t seed)
{
	/* 4 rounds, not 64. At 64 this cost ~400 cycles per iteration with the
	 * counter at the top of the loop, and the gemm thread only blocks for
	 * ~2900 cycles at a time - so between two context switches the worker
	 * never completed a pass and reported zero progress while genuinely
	 * running. The work unit has to be small enough to resolve the window
	 * being measured.
	 */
	for (int i = 0; i < 4; i++) {
		seed = seed * 1664525u + 1013904223u;
		seed ^= seed >> 13;
	}
	return seed;
}

static void worker(void *a, void *b, void *c)
{
	uint32_t acc = 1;

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (true) {
		/* Counted on EVERY pass, whatever work_run says. If this stays
		 * flat across phase 2 the thread was never scheduled at all;
		 * if it climbs while work_done does not, the flag never
		 * reached this thread. Those two failures are identical when
		 * all you have is work_done.
		 */
		work_polls++;
		if (work_run) {
			acc = churn(acc);
			work_sink = acc;
			work_done++;
		} else {
			k_yield();
		}
	}
}

static void stage_operands(void)
{
	for (int lane = 0; lane < dim; lane++) {
		for (int k = 0; k < k_len; k++) {
			a_panel[lane * k_len + k] = (int8_t)(lane + 1);
			/* Staged for every tile index the sweep can reach, once,
			 * so no sweep step has to restage and every step compares
			 * against the same operands.
			 *
			 * The tile index has to stay in the pattern - it is what
			 * makes a result wrong if a tile lands in the wrong place -
			 * and it also has to keep the product inside INT8 after the
			 * shift. At MAX_TILES=8 the worst case is
			 * dim*(dim+MAX_TILES-1) = 8*15 = 120, just inside 127. Raise
			 * MAX_TILES past 8 and this clips, silently, and the checker
			 * will blame the accelerator.
			 */
			for (int p = 0; p < MAX_TILES; p++) {
				b_panel[p][lane * k_len + k] = (int8_t)(lane + 1 + p);
			}
		}
	}
}

/* C[i][j] = k_len * (i+1) * (j+1+p) for tile p */
static void check_results(int n_tiles)
{
	words_checked += n_tiles * dim * dim;

	for (int p = 0; p < n_tiles; p++) {
		for (int i = 0; i < dim; i++) {
			for (int j = 0; j < dim; j++) {
				/* k_len divides out exactly at OUT_SHIFT */
				int8_t want = (int8_t)((i + 1) * (j + 1 + p));

				if (dest[p][i * dim + j] != want) {
					if (gemm_bad < 4) {
						printk("  MISMATCH t%d C[%d][%d]: got %d want %d\n",
						       p, i, j, dest[p][i * dim + j], want);
					}
					gemm_bad++;
				}
			}
		}
	}
}

static void gemm_runner(void *devp, void *ntp, void *c)
{
	const struct device *dev = devp;
	int n_tiles = (int)(intptr_t)ntp;
	struct gemm_tile tiles[MAX_TILES];

	ARG_UNUSED(c);

	for (int p = 0; p < n_tiles; p++) {
		tiles[p].a_src = a_panel;
		tiles[p].b_src = b_panel[p];
		tiles[p].dest = dest[p];
		tiles[p].k_len = k_len;
		/* Wrapped at the panel count the hardware actually has. There are
		 * only `panels` B scratchpads (4 here) but the sweep goes to 8
		 * tiles, and panel_use/panel_load are 4-bit fields the driver
		 * masks - so an unwrapped index would quietly select panel p&0xF
		 * and read whichever panel happened to be loaded there.
		 *
		 * Aliasing is safe because the queue runs tiles in order and each
		 * tile loads its own B panel before using it, so tile 4 cannot
		 * overwrite panel 0 until tile 0 has finished with it.
		 */
		tiles[p].panel_use = p % panels;
		tiles[p].panel_load = p % panels;
		/* Every tile shares a_panel, so only the first fetches it - the
		 * ordinary shape of a tiled GEMM inner loop.
		 */
		tiles[p].b_only = (p != 0);
		/* stage_operands() wrote these once, before any batch ran, and
		 * nothing touches them afterwards - so flushing them on every
		 * one of 16 batches was 15 redundant walks out of 16.
		 */
		tiles[p].src_coherent = true;
	}

	/* Pay for operand coherence exactly once. */
	gemm_flush_operands(dev, tiles, n_tiles);

	for (int n = 0; n < BATCHES; n++) {
		/* dest_stride MUST be 0 for INT8: a 128-bit line spans 16/dim
		 * result rows, so there is no row boundary to stride at and the
		 * driver rejects a nonzero stride.
		 */
		int rc = gemm_run_batch(dev, tiles, n_tiles, 0, true, OUT_SHIFT,
					batch_timeout);

		if (rc != 0) {
			/* -EBUSY here means n_tiles exceeded the descriptor queue,
			 * which is the sweep's own upper bound being wrong rather
			 * than a hardware fault - worth saying plainly, because it
			 * otherwise reads as a failed batch.
			 */
			printk("FAIL: %d-tile batch %d returned %d%s\n",
			       n_tiles, n, rc,
			       rc == -EBUSY ? " (exceeds descriptor queue depth)" : "");
			break;
		}
	}
	check_results(n_tiles);
}

struct run_result {
	uint32_t wall;        /* cycles for the whole batch loop        */
	uint32_t gemm_ran;    /* cycles the submitting thread executed  */
	uint32_t worker_ran;  /* cycles the other thread executed       */
	struct gemm_stats st;
};

/* Per-thread CPU attribution, when the kernel is configured to account for it.
 *
 * It is optional because the accounting is NOT free and it is not free in the
 * worst possible place: CONFIG_SCHED_THREAD_USAGE hooks the context switch, and
 * on this core reading the cycle counter is an MMIO access to the CLINT (~9
 * cycles) done with interrupts disabled. Together with CONFIG_STACK_SENTINEL's
 * per-switch check, the instrument lengthens the switch it is measuring - and
 * since a device interrupt cannot be serviced until the switch releases
 * interrupts, it also lengthens the completion latency this demo reports.
 *
 * Building without it loses the subCPU/worker columns but leaves wall, accel and
 * wake intact, because those come from the driver's own k_cycle_get_32() rather
 * than from thread accounting. That is the point: it separates the measurement
 * from the thing measured.
 */
static uint32_t thread_cycles(struct k_thread *t)
{
#ifdef CONFIG_THREAD_RUNTIME_STATS
	k_thread_runtime_stats_t rs;

	rs.execution_cycles = 0;
	k_thread_runtime_stats_get((k_tid_t)t, &rs);
	return (uint32_t)rs.execution_cycles;
#else
	ARG_UNUSED(t);
	return 0;
#endif
}

/* Identical work, identical hardware, one binary - only the wait differs.
 *
 * Measured with the kernel's own per-thread accounting rather than a counter
 * in the worker loop: that counter read zero while the runtime stats showed
 * the worker executing tens of thousands of cycles, so it cannot be trusted
 * and the kernel's can.
 */
static struct run_result run_mode(const struct device *dev, bool poll, int n_tiles,
				  k_timeout_t timeout)
{
	struct run_result r;
	uint32_t w_before, t0;

	batch_timeout = timeout;
	gemm_set_poll_mode(dev, poll);
	gemm_reset_stats(dev);

	work_run = true;
	w_before = thread_cycles(&work_thread);

	k_thread_create(&gemm_thread, gemm_stack, K_THREAD_STACK_SIZEOF(gemm_stack),
			gemm_runner, (void *)dev, (void *)(intptr_t)n_tiles, NULL,
			PRIO_GEMM, 0, K_NO_WAIT);
	k_thread_name_set(&gemm_thread, "gemm");

	t0 = k_cycle_get_32();
	k_thread_join(&gemm_thread, K_FOREVER);
	r.wall = k_cycle_get_32() - t0;

	/* gemm is a fresh thread each run, so its total IS this run's total. */
	r.gemm_ran = thread_cycles(&gemm_thread);
	r.worker_ran = thread_cycles(&work_thread) - w_before;
	work_run = false;

	gemm_get_stats(dev, &r.st);
	return r;
}

/* One line per configuration, everything per batch.
 *
 * A table rather than paragraphs because the question is how these numbers MOVE
 * with batch size, and that is only visible with the rows next to each other.
 *
 * accel  queue kick -> ISR entry: the accelerator's own time, PLUS any delay in
 *        servicing the interrupt.
 *
 *        This was introduced as a cross-check on the theory that the hardware
 *        cannot tell how the CPU is waiting, so the figure had to match across
 *        modes. That theory is wrong in one specific way, and measuring it
 *        found the error: the accelerator's DMA shares the mem_arbiter line
 *        port with the CPU's cache misses, so a worker thread running during
 *        the wait genuinely slows the accelerator down. Interrupt mode runs the
 *        worker and polling mode does not, so some of the gap is contention
 *        rather than deferral, and the two cannot be separated from this column
 *        alone. Treat a mismatch as a question, not as proof of deferral.
 * wake   ISR entry -> submitter running again. The scheduler's cost under
 *        interrupts, the spin loop's reaction time under polling.
 *
 * early  batches whose completion had already arrived before the thread reached
 *        the wait. Those take a fast path and never block, so a high count
 *        means the row is not measuring the wake path at all.
 *
 * One asymmetry to read subCPU with: the kernel charges interrupt time to
 * whichever thread was running when the interrupt landed. Polling mode is
 * spinning in the gemm thread, so the ISR is billed to gemm; interrupt mode is
 * running the worker, so the ISR is billed to the worker. That biases subCPU in
 * the interrupt run's FAVOUR - so if interrupt mode still shows a higher subCPU,
 * the real gap is wider than the column says, not narrower.
 */
static void report_row(int n_tiles, const char *mode, const struct run_result *r)
{
	uint32_t seen = r->st.isr_seen ? r->st.isr_seen : 1;

	printk("%5d  %-9s %6u %7u %7u %6u %6u %6u %5u\n",
	       n_tiles, mode,
	       r->wall / BATCHES,
	       r->gemm_ran / BATCHES,
	       r->worker_ran / BATCHES,
	       r->st.flush_cycles / BATCHES,
	       r->st.isr_cycles / seen,
	       r->st.resume_cycles / seen,
	       r->st.already_done);
}

static void report_header(void)
{
	printk("\ntiles  mode        wall  subCPU  worker  cache  accel   wake early\n");
	printk("------------------------------------------------------------------\n");
}

/* Every thread the kernel knows about, with state, priority and the cycles it
 * actually executed. execution_cycles is the figure that matters: state and
 * priority are what the kernel INTENDS, this is what happened.
 */
#ifdef CONFIG_THREAD_MONITOR
static void dump_thread(const struct k_thread *t, void *ud)
{
	char st[16];
	const char *name = k_thread_name_get((k_tid_t)t);

	ARG_UNUSED(ud);
	printk("  %-12s prio %2d  state %-10s ran %u cyc\n",
	       (name && name[0]) ? name : "(unnamed)",
	       k_thread_priority_get((k_tid_t)t),
	       k_thread_state_str((k_tid_t)t, st, sizeof(st)),
	       thread_cycles((struct k_thread *)t));
}
#endif

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(gemm0));
	struct gemm_info info;
	uint32_t t0, baseline, baseline_cycles;

	printk("\n=== accelerator as an OS device ===\n");

	if (!device_is_ready(dev)) {
		printk("FAIL: gemm device not ready\n");
		return 0;
	}
	gemm_get_info(dev, &info);
	dim = info.dim;
	k_len = (info.max_k < KLEN) ? info.max_k : KLEN;
	/* Read from INFO, not hardcoded: the panel count is a generator parameter
	 * and the sweep indexes panels modulo it.
	 */
	panels = info.panels ? info.panels : 1;
	printk("geometry: dim=%d maxK=%d panels=%d, running K=%d\n",
	       info.dim, info.max_k, info.panels, k_len);

	stage_operands();

	/* K_THREAD_STACK_SIZEOF, not the raw macro: the stack object carries
	 * guard/alignment overhead, so the usable size is not what was asked
	 * for and passing the raw value understates or overstates it.
	 */
	k_thread_create(&work_thread, work_stack, K_THREAD_STACK_SIZEOF(work_stack),
			worker, NULL, NULL, NULL, PRIO_WORK, 0, K_NO_WAIT);
	k_thread_name_set(&work_thread, "worker");

	/* --- 1. worker alone: how fast it goes with the whole CPU --- */
	work_done = 0;
	work_polls = 0;
	work_run = true;
	t0 = k_cycle_get_32();
	k_sleep(K_MSEC(200));
	baseline_cycles = k_cycle_get_32() - t0;
	work_run = false;
	baseline = work_done;
	printk("worker alone          %u iterations in %u cycles (%u cyc/iter)\n",
	       baseline, baseline_cycles, baseline ? baseline_cycles / baseline : 0);

	/* --- 2. the same work, two wait strategies, swept over batch size --- */
	printk("\n--- %d batches per point, interrupt vs polling, swept ---\n",
	       BATCHES);
	report_header();

	int crossover = 0;

	for (unsigned s = 0; s < ARRAY_SIZE(tile_sweep); s++) {
		int n = tile_sweep[s];
		struct run_result irq_run, poll_run, fvr_run;

		poll_run = run_mode(dev, true, n, K_MSEC(500));
		irq_run  = run_mode(dev, false, n, K_MSEC(500));
		/* Same interrupt path, no timeout record to insert. If `accel`
		 * drops towards the polling figure here, the deferral was
		 * z_add_timeout holding interrupts off - not the accelerator and
		 * not the context switch.
		 */
		fvr_run  = run_mode(dev, false, n, K_FOREVER);

		report_row(n, "poll", &poll_run);
		report_row(n, "irq/500ms", &irq_run);
		report_row(n, "irq/forever", &fvr_run);

		/* The verdict per point, in the only two terms that matter: does
		 * the submitter burn less CPU, and does the job finish sooner.
		 * They can disagree - the interrupt can hand CPU to another thread
		 * while making the batch itself slower - so both are printed
		 * rather than collapsed into one "faster/slower".
		 */
		uint32_t p_sub = poll_run.gemm_ran / BATCHES;
		uint32_t p_wall = poll_run.wall / BATCHES;
		/* Judge the better of the two interrupt variants, so a verdict is
		 * never an artefact of the timeout choice.
		 */
		const struct run_result *best =
			fvr_run.wall < irq_run.wall ? &fvr_run : &irq_run;
		uint32_t i_sub = best->gemm_ran / BATCHES;
		uint32_t i_wall = best->wall / BATCHES;
		uint32_t i_work = best->worker_ran / BATCHES;

		if (i_sub < p_sub) {
			printk("       -> irq frees %u submitter cyc/batch",
			       p_sub - i_sub);
		} else {
			printk("       -> irq COSTS %u extra submitter cyc/batch",
			       i_sub - p_sub);
		}
		printk(", wall %u -> %u (%s%u)\n", p_wall, i_wall,
		       i_wall > p_wall ? "+" : "-",
		       i_wall > p_wall ? i_wall - p_wall : p_wall - i_wall);

		/* Freed submitter CPU and added latency can both be true at once,
		 * so the crossover is only real when the work the other thread got
		 * done exceeds the wall time it cost. Declaring a win on freed CPU
		 * alone counts a thread that merely spent the latency as a gain.
		 *
		 * Without per-thread accounting there is no i_work to weigh, and a
		 * verdict computed from a zero would always say polling wins - a
		 * confident wrong answer. The lean build measures latency, not
		 * throughput, and has to say so rather than judge.
		 */
#ifndef CONFIG_THREAD_RUNTIME_STATS
		printk("          net: cannot judge - this build cannot see worker"
		       " progress\n");
		ARG_UNUSED(i_work);
#else
		if (i_wall <= p_wall) {
			printk("          net: faster AND frees CPU\n");
			if (!crossover) {
				crossover = n;
			}
		} else if (i_work > i_wall - p_wall) {
			printk("          net: +%u worker cyc for +%u wall - ahead by %u\n",
			       i_work, i_wall - p_wall, i_work - (i_wall - p_wall));
			if (!crossover) {
				crossover = n;
			}
		} else {
			printk("          net: +%u worker cyc does not cover +%u wall\n",
			       i_work, i_wall - p_wall);
		}
#endif
	}

	printk("\n--- verdict ---\n");
#ifndef CONFIG_THREAD_RUNTIME_STATS
	printk("  none - this build measures LATENCY (wall/accel/wake/cache) with\n");
	printk("  the per-switch instrumentation removed, so it cannot see how much\n");
	printk("  work the freed CPU actually did. Read its wall cost against the\n");
	printk("  worker figures from the instrumented build.\n");
	ARG_UNUSED(crossover);
#else
	if (crossover) {
		printk("  interrupts start paying off at %d tiles/batch\n", crossover);
	} else {
		printk("  polling wins at every batch size up to %d - the descriptor\n",
		       MAX_TILES);
		printk("  queue depth (%d) caps the batch before the accelerator's own\n",
		       MAX_TILES);
		printk("  time can grow enough to cover the wake path. Batch size has\n");
		printk("  run out as a lever; the wake path is what has to get cheaper.\n");
	}
#endif

	if (gemm_bad) {
		printk("\nFAIL: %d wrong result words of %d\n",
		       gemm_bad, words_checked);
	} else {
		printk("\nPASS: all %d result words correct\n", words_checked);
	}

#ifdef CONFIG_THREAD_MONITOR
	printk("\n--- every thread the kernel knows ---\n");
	k_thread_foreach(dump_thread, NULL);
#else
	printk("\n(thread table and subCPU/worker columns need "
	       "CONFIG_THREAD_MONITOR + CONFIG_THREAD_RUNTIME_STATS;\n");
	printk(" this is the lean build, which drops them to keep them out of "
	       "the context switch)\n");
#endif

	printk("=== done ===\n");
	return 0;
}
