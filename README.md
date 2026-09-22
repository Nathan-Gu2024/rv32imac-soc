# RV32IMAC SoC — RISC-V Core, Caches, AI Accelerator, RTOS

A from-scratch RV32IMAC_Zba RISC-V system-on-chip in Verilog, running on a
Xilinx Zynq-7020 FPGA and taken through an open-source ASIC flow. Includes a
5-stage pipelined core with dynamic branch prediction, BRAM-backed L1 caches,
a Chisel-generated 8×8 INT8 systolic-array GEMM accelerator, and a Zephyr RTOS
board port.

**190.5 CoreMark @ 60 MHz — 3.17 CoreMark/MHz** on FPGA; a **135× measured
speedup** on INT8 GEMM against the same core running the same kernel; and a
**signed-off 10.58 mm² Sky130 GDSII** with eleven SRAM macros, LVS-clean and
XOR-clean.

---

## Results

### FPGA — Zynq-7020 (`xc7z020clg400-2`)

| Metric | Value |
|---|---|
| CoreMark | **190.49 iterations/sec** @ 60 MHz (2000 iterations, validated run) |
| CoreMark/MHz | **3.17** |
| Fmax | 60 MHz, WNS +0.144 ns — critical path is reset-net routing (0 logic levels, 97.6% route, fanout 5222), not logic |
| Utilization | 13% LUT · 6% FF · 22% BRAM · 2% DSP |
| On-chip power | 1.51 W |
| ISA | RV32IMAC + Zicsr + Zba, machine mode |

### GEMM accelerator — measured on the Zynq board

INT8 GEMM, 8×8 tile, K=64, against the same GEMM compiled for the scalar RV32
core. Every figure is a hardware measurement, not a projection.

| Stage | cyc/tile | MACs/cycle | vs scalar |
|---|---|---|---|
| Register-window baseline | 1622 | 0.63 | 5.7× |
| \+ operand DMA, scratchpad, double buffering | 1001 | 1.02 | 10.0× |
| \+ read bursts | 696 | 1.47 | 15.3× |
| \+ descriptor queue | 594 | 1.72 | 16.8× |
| \+ write bursts | 273 | 3.74 | 36.4× |
| \+ `K_LEN` 16 → 64 | 444 | 9.22 | 77.8× |
| \+ A-panel reuse | 346 | 11.8 | 99.8× |
| \+ INT8 requantized output | 327 | 12.5 | 105× |
| **\+ split read/write memory path** | **257** | **15.9** | **135×** |

Rows after `K_LEN` 16 → 64 do four times the arithmetic per tile, which is why
cycles per tile rise while throughput does.

The last row is the modal value of three board runs, which gave 1030 / 1030 /
1065 cycles for the 4-tile queued batch — so **257–266 cyc/tile and 130–135×**.
Earlier rows are single runs and should be read with the same tolerance.

**The comparison is MAC-for-MAC** — 4096 multiply-accumulates either way — but
the final row writes INT8 where the scalar reference writes INT32, so it is a
*requantized* INT8 GEMM, not a bare speedup over an identical computation.

The array is busy 78 of those 257 cycles. What remained was memory: an operand panel
and a result tile shared one 128-bit port with both caches, and the port sat idle
while the array computed. Closing that needed two tiles in flight — issuing the
next panel fetch before the current store — and the last row is that change. The
accelerator's port is now split into independent read and write channels carried
by `mem_arbiter_rw` and `axi_rw_engine` as independent AXI4 read and write
transactions, so an operand fetch overlaps a result store.

What is left is the result-write path, and there are two candidate explanations
with different consequences. `fpga/tests/test_mm_accel.c` times the same 16
result lines twice, contiguous (one AXI transaction) and strided (eight), because
the two predict different ratios: a slow write *response*, which the accelerator
merely waits on and can therefore overlap, scales with transactions (~5×); write
*acceptance* the memory cannot sustain scales with beats (~3×).

**Measured 3.02× on the board — acceptance-bound. So further overlapping cannot
help; the next lever is a wider port or the accelerator's own AXI master.**

That number took two attempts, and the first one was not evidence. A single store
is smaller than the MMIO floor around it: what the test timed was a `CTRL` write
plus a `STATUS` poll loop, and an AXI4-Lite read costs ~10.15 cycles here, so the
ratio was really (S_strided + F)/(S_contig + F) — biased toward 1 by the floor F.
It read 2.86×, which happened to point at the right answer for the wrong reason.
Averaging each store over 16 repeats divides F out: the contiguous store fell
116 → 76 cycles, i.e. by the ~40 the floor was estimated at, and the per-line cost
with it from 7.25 to **4.75 cycles**.

What makes 3.02× trustworthy is not the board alone. `tb_result_dma`, which has no
DDR and no MMIO floor, independently reports **3.11×** — two models that share no
error source agreeing to 3%. That matters because averaging introduced a bias of
its own: 16 back-to-back stores into one buffer enjoy DDR page locality a cold
single store would not, and the strided figure fell by 102 cycles rather than the
40 the floor explains. Simulation has no DRAM at all, so its agreement is what
bounds that bias.

The same floor produced one outright false reading, kept here because it is the
cheaper lesson. INT8 writeback measured **slower** than INT32 — 127 then 143
cycles against 116 — despite moving a quarter of the bytes in the same single
transaction (`nGroups8 = dim²/16` is 4 lines, and `wBurstLines` bursts all of
them). Two runs of the identical binary differed by 16 cycles on that store while
every figure above the floor reproduced exactly, which is the floor measuring
itself: the whole "penalty" was one to two 10.15-cycle poll iterations.
`scripts/sweep_int8_store.sh` sweeps write-response latency against WREADY
backpressure over nine combinations and cannot invert the order under any of
them — backpressure is a per-beat cost, and INT8 moves 8 beats against INT32's
32, so it makes INT8 relatively *better*. Averaged, the board agrees: **51 vs 76
cycles, INT8 1.49× faster**, against simulation's 1.8×. Nothing was wrong with
the design; the measurement could not resolve the difference it was asked about.

### ASIC — OpenLane / Sky130, RTL-to-GDSII

| Metric | Value |
|---|---|
| Die | **10.584 mm²** (4900 × 2160 µm), 13.6% std-cell utilization |
| Memory | **11 OpenRAM SRAM macros** — 8 KB I-cache (6), 8 KB D-cache (5) |
| Cells | 88,932 synthesized → 109,133 placed |
| Timing | **WNS/TNS 0.0** — no setup or hold violations |
| Clock | 25 ns (**40 MHz**) chip-level; core logic closes at **19.82 ns (50.5 MHz)** |
| LVS | **Clean** — 110,523 nets, both sides |
| KLayout XOR | **0 differences** |
| Antenna | 45 pin / 42 net |
| CoreMark/MHz | **3.16** (see below) |

The clock is I/O-bound, not core-bound: **every** setup violation is an output
port and **zero** register-to-register paths fail. `openlane/cpu/pin_order.cfg`
groups the pins by function so the timing-critical ones sit on the east and
west edges, against the logic channel — the macro rows wall off north and
south. That moved `store_data`'s arrival from 20.30 ns to 19.09 ns and the
minimum legal period from 25.69 ns to 24.17 ns, and shortened total wire
length 5.3% while cutting antenna violations ~25%.

What remains is **logic depth, not placement**: the `store_data` path traverses
roughly 58 gates through the AMO sequencer and partial-store alignment, so no
further pin work touches it. Registering the memory-side outputs would reach
the core's own limit of 19.82 ns / 50.5 MHz, at the cost of a cycle on the
store path.

Two notes for anyone reproducing this. `base.sdc` sets
`output_delay = CLOCK_PERIOD × IO_PCT`, so the output budget **scales with the
clock** and OpenLane's `suggested_clock_period` is wrong whenever outputs are
critical — solve `P − 0.2P − 0.25 > arrival` instead. And the SRAM macros are
**not** on any critical path: 0.53 ns clock-to-out against a 25 ns period.

### Figure of merit

$$\text{FOM} = 10^{10} \times \frac{f_{max}}{\text{cycles} \times \sqrt{\text{area}}}$$

with `f_max` in Hz, `area` in mm², and `cycles` the full CoreMark run from
`tb_coremark_sim.v` at each build's own cache geometry (boot and UART output
included — a fixed overhead that slightly favours the slower build).

| Build | f_max | Cycles | Area | **FOM** | |
|---|---|---|---|---|---|
| Flip-flop, 256 B/256 B | 50.0 MHz | 16,648,911 | 3.328 mm² | **1.65 × 10¹⁰** | — |
| SRAM macro, default pin placement | 37.0 MHz | 8,975,373 | 10.584 mm² | **1.27 × 10¹⁰** | 0.77× |
| **SRAM macro, pins grouped** | **40.0 MHz** | 8,975,373 | 10.584 mm² | **1.37 × 10¹⁰** | **0.83×** |

**The macro build still trails on this metric, and that is worth stating
plainly:** it buys 1.37× the performance (3.00 → 4.13 benchmark runs/sec) for
3.2× the area, and √area charges 1.78× for that. CoreMark/MHz alone (1.70 →
3.16) hides the trade; this does not.

The remaining gap is **area, not speed**. The die is only **13.6% utilized** —
roughly 3.1 mm² of macros plus ~2.8 mm² of logic inside 10.58 mm², because the
routing channels were sized for convergence after six failed attempts rather
than for area. Break-even at the current clock is **6.28 mm²**, and shrinking
toward that is pure floorplan work: no RTL risk, no cycle cost. Registering the
memory-side outputs would add ~10.5 MHz on top, but trades cycles for
frequency, which partly self-cancels in this metric.

## Architecture

```
        ┌──────────────────────────── cpu_pipelined ────────────────────────────┐
        │  IF          ID          EX            MEM              WB            │
        │  ├─ gshare   ├─ decode   ├─ ALU        ├─ AMO seq       ├─ regfile    │
        │  ├─ RAS      ├─ immgen   ├─ mul/div    ├─ LR/SC monitor │             │
        │  └─ RVC exp  └─ hazard   └─ branch cmp └─ partial ld/st │             │
        └───────┬──────────────────────────────────────┬─────────────────────────┘
                │                                      │
        ┌───────▼────────┐                    ┌────────▼────────┐
        │ icache 32KB    │                    │ dcache 16KB     │   ┌──────────┐
        │ direct-mapped  │                    │ DM, write-back  │   │ TCM 64KB │
        │ (Block RAM)    │                    │ (Block RAM)     │   │ dual-port│
        └───────┬────────┘                    └────────┬────────┘   └──────────┘
         sizes shown are the FPGA build; on ASIC both caches are 8KB in
         SRAM macros, from the same RTL — see "SRAM macros on ASIC" below
                └──────────────┬───────────────────────┘
                       ┌───────▼────────┐      ┌──────────────────────────────┐
                       │ mem_arbiter_rw │      │ MMIO: UART · CLINT · INTC    │
                       │  read · write  │      │       LEDs · MM accelerator  │
                       └───────┬────────┘      └──────────────────────────────┘
                       ┌───────▼────────┐    the accelerator is a requester here
                       │ axi_rw_engine  │    too, on both channels, so an operand
                       │ AR/R  ·  AW/W  │    fetch overlaps a result store
                       └───────┬────────┘
                               └──▶ Zynq PS7 / DDR
```

### Core
- 5-stage pipeline (IF/ID/EX/MEM/WB) with full forwarding and load-use
  hazard interlock
- **IF-stage branch prediction**: gshare (1024-entry PHT, 10-bit global
  history, speculative update with misprediction rollback) plus an 8-entry
  return-address stack. Predicting in fetch rather than decode removes the
  bubble on correctly-predicted taken branches.
- **RV32A atomics**: LR/SC with a reservation monitor, and all 9 AMO
  operations via a MEM-stage read-modify-write sequencer
- **Zba** (`sh1add`/`sh2add`/`sh3add`), selected by profiling which
  bit-manipulation instructions GCC actually emits
- Machine-mode CSRs, traps, and interrupts (timer + external via CLINT/INTC)
- **Precise exceptions** with `mcause`/`mepc`/`mtval`: illegal instruction,
  misaligned load, misaligned store, ECALL and MRET. `mepc` holds the *faulting*
  instruction rather than the one after it, so a handler that fixes the cause can
  resume by returning, and `mtval` carries the offending address or instruction
  word. Confirmed on hardware, not only in simulation.

### Memory system
Both L1 caches are **Block RAM-backed**. Their arrays are read synchronously
and addressed one cycle ahead — the icache from the next PC, the dcache from
a dedicated EX-stage adder — so a hit still costs zero extra cycles despite
the registered read. The dcache includes a store-to-load bypass covering
cross-port read-during-write, which is undefined behaviour on Xilinx SDP
block RAM.

An earlier revision read these arrays combinationally, which cannot infer
block RAM and instead consumed ~11K LUTs and ~20K flip-flops as distributed
RAM plus F7/F8 mux trees. Converting them cut LUT usage 63% and FF usage 75%
while eliminating 99% of instruction-fetch and 95% of data-side stall cycles.

### SRAM macros on ASIC
The same RTL builds for both targets. `IC_USE_SRAM` / `DC_USE_SRAM` select,
via `generate`, between inferred Block RAM and instantiated
`sky130_sram_2kbyte_1rw1r_32x512_8` macros — five for the D-cache (four
32-bit banks plus tags), six for the I-cache (its even/odd banks are 64 bits
wide, so two macros each, plus two mirrored tag arrays).

This matters because **without macros the cache arrays synthesize to
flip-flops**. The FPGA configuration would be roughly a million of them —
about 78 mm² — so the first ASIC build had to shrink both caches to 256 bytes,
and at that size the design takes **86% more cycles** than the FPGA geometry.
With macros the ASIC runs 8 KB + 8 KB, within **0.31%** of the 32 KB/16 KB
FPGA configuration, which is what makes its CoreMark/MHz directly comparable:

| Build | Caches | CoreMark/MHz | Die | Clock |
|---|---|---|---|---|
| Flip-flop arrays | 256 B / 256 B | 1.70 | 3.328 mm² | 50 MHz |
| **SRAM macros** | **8 KB / 8 KB** | **3.16** | 10.584 mm² | 40 MHz |

The macro's `1RW+1R` ports map onto the existing design without restructuring:
both caches already read at a speculated address and write at a different one
in the same cycle. Reads go on port 1 and writes on port 0 — mandatory, not
stylistic, because the read-write port drives its output to `X` on every clock
edge and only restores it on a read cycle, which would break the assumption
that the output registers hold across stalls.

`openlane/dcache/` and `openlane/icache/` harden each cache standalone, which
keeps floorplan iteration to ~10 minutes instead of ~4 hours in the full CPU.

### Accelerator
An **8×8 output-stationary INT8 systolic array** with INT32 accumulation,
generated from Chisel (`chisel/src/MmAccel.scala`) and emitted as Verilog-2001
that drops into the same SoC as the hand-written RTL. Skewed operand feed,
`K_LEN + 2·(DIM−1)` cycle schedule, `K_LEN` up to 64.

It has two interfaces, and the split is the whole design:

- an **AXI4-Lite slave** for control — a constant-size 23-word register map
  indexed by lane and accumulator rather than one word per element, so the
  window does not grow with `DIM`
- **two independent 128-bit line ports**, one read and one write, as requesters
  on `mem_arbiter_rw`, sharing the cache path to DRAM, over which it fetches its
  own operands and writes its own results. They are separate so that an operand
  fetch can overlap a result store; `MEM_PATH_RW = 0` in `fpga/rtl/fpga_top.v`
  rejoins them onto the older single-port path, which is how the two are
  A/B-compared in simulation and on hardware

Everything that mattered for throughput lives on the second one. Driving 64
accumulators out through a 32-bit register window costs ~5 cycles of protocol
per 4 bytes, which made result readback 70% of runtime before the line port
existed.

| Feature | What it buys |
|---|---|
| Operand DMA over the line port | the array fetches its own panels; no CPU in the loop |
| INCR bursts, up to 32 lines | one AXI round trip per panel instead of per line |
| 4-panel B scratchpad | a strip of tiles reloads nothing |
| Double-buffered panel load | the next panel fills while the current one feeds a run |
| Queue-driven operand prefetch | tile N+1's B panel loads during tile N's compute — 21% off an A-reuse batch (469 → 371 cyc) |
| Descriptor queue (8 deep) | a batch of tiles runs with one kick and one poll |
| A-panel reuse (`bOnly`) | a row of tiles shares one A panel — half the operand traffic |
| INT8 requantized output | shift/round/saturate on the way out; a tile is 4 lines, not 16 |

**Operand buffers are 16-byte rows in synchronous-read memories**, addressed one
cycle ahead — the same trick both L1 caches use. Read combinationally as flat
registers they were 20 Kbit behind a 64:1 mux per lane per panel, and firtool
emitted 102,735 lines of Verilog for one accelerator; as 1R1W byte-masked RAM
the same design is 3,618 lines and **79% fewer flip-flops**.

**Coherence** is handled by set-displacement eviction. The D-cache is
write-back and this SoC has no cache-maintenance instruction, no maintenance
CSR and no uncached DRAM window, so software touches one address in every set
of the direct-mapped cache to force dirty operand lines out before the
accelerator reads them from DRAM. It costs ~17k cycles and is paid once per
upload, not per tile.

### Software
- **Zephyr RTOS** board port: custom SoC/board definition, devicetree,
  UART and timer drivers, second-level interrupt controller
- Bare-metal C and assembly tests, plus a full EEMBC CoreMark port

## Verification

| Test | Coverage |
|---|---|
| `tb_coremark_sim.v` | Full CoreMark against golden CRCs, plus branch/RAS/stall instrumentation |
| `tb_cpu_trap.v` | Trap delivery at CPU level: illegal instruction, misaligned load/store, and a misaligned load whose result the **next instruction consumes** — the case that coincides with the load-use interlock. Asserts control *reached the handler*, not merely that `mcause` was written |
| `tb_cpu_b2b.v` | Back-to-back divide, AMO and JALR with **different** operands — same-operand pairs pass against the divide bug |
| `tb_cpu_interlock.v` | Counts interlock stall cycles, so removing false stalls is proven not to remove real ones |
| `tb_illegal_inst.v` | Undefined encodings trap instead of decoding as `add` |
| `tb_trap_stall.v`, `tb_stall_watchdog.v` | Traps taken under memory stalls; watchdog on a pipeline that stops retiring |
| `tb_echo_ddr.v` | DDR-resident code, both caches, a driven RX pin and a TX echo — the intersection `tb_coremark_sim` (TX only, no interrupts) and `tb_uart_rx` (TCM, no I-cache) each miss |
| `tb_amo_test.v` | All 9 AMO ops — returned old value and committed memory value (18 checks) |
| `tb_dcache_hazard.v` | Store→load bypass, byte/halfword merge, dirty eviction + writeback (7 checks) |
| `tb_alu_mul.v` | MUL/MULH/MULHSU/MULHU, including operands where all three high-forms differ |
| `tb_dcache.v`, `tb_icache.v` | Standalone cache unit tests |
| `tb_sram_wrapper.v` | SRAM macro wrapper against the PDK behavioural model (12 checks) |
| `tb_mm_accel.v` | Accelerator + AXI4-Lite bridge |
| `tb_mm_accel_opdma.v` | Operand DMA: correctness, stride, direction, cost vs the register window |
| `tb_mm_accel_pad.v` | B scratchpad — each panel gets distinct operands, so a stuck panel select fails |
| `tb_mm_accel_db.v` | Double buffering — prefetch during compute must not corrupt the run |
| `tb_mm_accel_queue.v` | Descriptor queue, and A-panel reuse with the A source **poisoned** in memory |
| `tb_mem_arbiter.v` | Three-port arbitration, round-robin, burst passthrough |
| `tb_burst_integration.v` | `mem_arbiter` + `axi_cache_adapter` + an AXI slave, multi-line both directions |
| `tb_arbiter_rw.v` | `mem_arbiter_rw`'s split rotations, incl. an accelerator read concurrent with an accelerator write |
| `tb_rw_engine.v` | `axi_rw_engine`'s independent read/write FSMs, and a held `req_valid` across completion producing exactly one transaction |
| `tb_accel_join.v` | `accel_port_join` re-serialising the split port onto one request, for the `MEM_PATH_RW = 0` fallback |
| `tb_result_dma.v` | Real accelerator through real arbiter and adapter, against a slave with configurable latency and WREADY stalls. `-DRW_SPLIT` rebuilds the identical stimulus against the `MEM_PATH_RW = 1` chain, so the two hardware configurations are one bench's A/B rather than two loosely related tests |

The cache benches run in **both** memory configurations — inferred Block RAM
and SRAM macros — from one source, via `-DUSE_SRAM`. At matched geometry the
two are **bit-identical on every counter** (cycles, branches, mispredicts, RAS
hits, stalls, refills), which is the evidence that the macro path introduces
no behavioural difference.

One caveat is recorded in `dcache_bram.v` rather than left to be rediscovered:
the store→load bypass **cannot** be validated on the macro path. Deleting it
fails 4 of 7 checks against Block RAM but passes all 7 against the macro,
because the OpenRAM model resolves the undefined same-address read-during-write
as write-first in simulation. The Block RAM mutation is the only mechanised
proof, and it has to cover both targets.

**Uniform test data hides whole classes of bug.** Every accelerator bench
originally drove operands where each result came out identical, and a DMA that
reordered lines, or took a beat from its neighbour, wrote the same bytes either
way. Rebuilding the checks around `C[i][j] = K·(i+1)(j+1)` — distinct in every
position — immediately exposed an AXI protocol violation that had survived a
bitstream, a board run and every existing test: `axi_cache_adapter` sliced
`mem_wline` live on multi-line bursts while `mem_wnext` advanced the requester
a beat early, so any slave that deasserted `WREADY` mid-line got the next
line's bytes. `tb_burst_integration.v` now stalls `WREADY` deliberately, and
that test was checked to **fail without the fix**.

Correctness is anchored on CoreMark's golden CRCs
(`0xe714`/`0x1fd7`/`0x8e3a`/`0x4983`) — any mismatch means real data
corruption, not a tuning regression. The dcache hazard bench was
**mutation-tested**: disabling the bypass makes 4 of its 7 checks fail with a
stale-data signature, confirming the test detects the bug it targets.

## Repository layout

```
src/          RTL — core, caches, TCM, peripherals, accelerator, AXI
chisel/       Chisel generator for the accelerator (src/mm_accel.v is its output)
Testbenches/  Simulation benches, directed assembly tests, CoreMark harness
fpga/         Zynq top level, linker scripts, bare-metal tests, CoreMark port
constraints/  Vivado XDC
openlane/     OpenLane/Sky130 ASIC flow configuration
zephyr/       Zephyr SoC + board port, drivers
Mems/         Memory initialization images
```

## Building

**Simulation** (Icarus Verilog):
```sh
cd Testbenches
iverilog -g2012 -o sim_coremark tb_coremark_sim.v && vvp sim_coremark
iverilog -g2012 -o sim_amo      tb_amo_test.v     && vvp sim_amo
```

**CoreMark binary** (RISC-V GCC):
```sh
bash fpga/coremark/build.sh 2000     # ITERATIONS=2000
```

**Accelerator RTL** (Chisel → Verilog-2001, needs `scala-cli`):
```sh
cd chisel
MM_DIM=8 MM_MAXK=64 MM_BPANELS=4 MM_OUT=generated scala-cli run src/MmAccel.scala
cp generated/mm_accel.sv ../src/mm_accel.v
```
`MM_DIM`, `MM_MAXK` and `MM_BPANELS` are the only knobs; the emitted file is
Verilog-2001 (`disallowPackedArrays,disallowLocalVariables,noAlwaysComb`) so
one artifact drops into Vivado, Icarus and OpenLane alike. Sweep it with
`openlane/mm_accel/sweep.sh`.

> `MM_BPANELS` was reachable only as a Scala default for a while, so every
> sweep silently built the same 4-panel design. When it was finally wired
> through, `bPanels=2` failed to elaborate — a packed-array lowering that
> happened to survive at 1, 4 and 8. Sweep a parameter before trusting it.

**FPGA**: build `fpga/rtl/fpga_top.v` in Vivado as a module reference inside
a block design containing the ZYNQ7 PS (FCLK_CLK0 = 60 MHz, AXI HP0 to DDR).
Load the CoreMark binary to DDR over JTAG; output arrives on UART at 115200.

> The PL clock frequency appears in three places that must agree:
> `uart_mmio.v`'s `CLK_FREQ`, `core_portme.c`'s `CLOCKS_PER_SEC`, and
> Zephyr's `SYS_CLOCK_HW_CYCLES_PER_SEC`. A stale value there is silent —
> the benchmark simply misreports its score.

**ASIC** (OpenLane/Sky130, ~4.5 h and ~17 GB peak for the full CPU):
```sh
cd $OPENLANE_ROOT && make mount
./flow.tcl -design ./designs/RV32-5-stage-processor/openlane/cpu -tag signoff
```
`openlane/cpu/` builds the whole SoC with all eleven macros;
`openlane/dcache/` and `openlane/icache/` harden each cache alone for fast
floorplan work, and `openlane/sram_test/` is a single-macro bring-up vehicle.

> `openlane/cpu/macro_placement.cfg` carries the floorplan's full revision
> history — six routing attempts, two of whose "fixes" made things worse or
> did nothing. Read it before changing any dimension. Likewise
> `openlane/dcache/NOTES.md` records two **disproven** DRC hypotheses.
>
> Magic DRC is not a usable gate once macros are present: `MAGIC_DRC_USE_GDS`
> off reports thousands of false `nwell.4` violations because LEF abstracts
> hide the taps (which are demonstrably there), and on reports ~28 M because
> the open PDK deck lacks rules for Sky130's optical-proximity-shrunk SRAM
> transistors. LVS and XOR are the meaningful checks. CVC cannot run at all —
> the PDK ships no CDL for `sky130_ef_sc_hd__decap_12`.

## License

CoreMark is EEMBC's, under its own license (`fpga/coremark/src/LICENSE.md`).
