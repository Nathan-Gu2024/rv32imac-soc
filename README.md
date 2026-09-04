# RV32IMAC SoC — RISC-V Core, Caches, AI Accelerator, RTOS

A from-scratch RV32IMAC_Zba RISC-V system-on-chip in Verilog, running on a
Xilinx Zynq-7020 FPGA and taken through an open-source ASIC flow. Includes a
5-stage pipelined core with dynamic branch prediction, BRAM-backed L1 caches,
an AXI4-Lite INT8 systolic-array accelerator, and a Zephyr RTOS board port.

**190.5 CoreMark @ 60 MHz — 3.17 CoreMark/MHz** on FPGA, using 13% of the
device's LUTs and 6% of its flip-flops; and a **signed-off 10.58 mm² Sky130
GDSII** with eleven SRAM macros, LVS-clean and XOR-clean.

---

## Results

### FPGA — Zynq-7020 (`xc7z020clg400-2`)

| Metric | Value |
|---|---|
| CoreMark | **190.49 iterations/sec** @ 60 MHz (2000 iterations, validated run) |
| CoreMark/MHz | **3.17** |
| Fmax | 60 MHz, WNS +0.72 ns |
| Utilization | 13% LUT · 6% FF · 22% BRAM · 2% DSP |
| On-chip power | 1.51 W |
| ISA | RV32IMAC + Zicsr + Zba, machine mode |

### ASIC — OpenLane / Sky130, RTL-to-GDSII

| Metric | Value |
|---|---|
| Die | **10.584 mm²** (4900 × 2160 µm), 13.6% std-cell utilization |
| Memory | **11 OpenRAM SRAM macros** — 8 KB I-cache (6), 8 KB D-cache (5) |
| Cells | 88,932 synthesized → 109,133 placed |
| Timing | **WNS/TNS 0.0** — no setup or hold violations |
| Clock | 27 ns (**37 MHz**) chip-level; core logic closes at **17.34 ns (57.7 MHz)** |
| LVS | **Clean** — 110,178 nets, both sides |
| KLayout XOR | **0 differences** |
| CoreMark/MHz | **3.16** (see below) |

The 37 MHz figure is I/O-bound, not core-bound: at a 20 ns constraint every
setup violation was an output port (`store_data[*]`, crossing a 4900 µm die to
a perimeter pin) and **zero** register-to-register paths failed. The worst of
958 reg-to-reg paths has 2.66 ns of slack at 20 ns, putting the core's own
limit at **17.34 ns / 57.7 MHz**; that path is the IF-stage next-PC mux chain
(gshare / RAS / branch target) feeding the I-cache address. Closing the gap
means constraining I/O pin placement or registering the memory-side outputs,
not faster tooling. Notably the SRAM macros are **not** on any critical path —
0.53 ns clock-to-out against a 27 ns period.

### Figure of merit

$$\text{FOM} = 10^{10} \times \frac{f_{max}}{\text{cycles} \times \sqrt{\text{area}}}$$

with `f_max` in Hz, `area` in mm², and `cycles` the full CoreMark run from
`tb_coremark_sim.v` at each build's own cache geometry (boot and UART output
included — a fixed overhead that slightly favours the slower build).

| Build | f_max | Cycles | Area | **FOM** | |
|---|---|---|---|---|---|
| Flip-flop, 256 B/256 B | 50.0 MHz | 16,648,911 | 3.328 mm² | **1.65 × 10¹⁰** | — |
| SRAM macro, 8 KB/8 KB | 37.0 MHz | 8,975,373 | 10.584 mm² | **1.27 × 10¹⁰** | 0.77× |
| SRAM macro, I/O fixed | 57.7 MHz | 8,975,373 | 10.584 mm² | **1.98 × 10¹⁰** | 1.20× |

**The macro build trails on this metric as built, and that is worth stating
plainly:** it buys 1.37× the performance (3.00 → 4.13 benchmark runs/sec) for
3.2× the area, and √area charges 1.78× for that. CoreMark/MHz alone (1.70 →
3.16) hides the trade; this does not.

Two independent routes close it. The die is only **13.6% utilized** — roughly
3.1 mm² of macros plus ~2.8 mm² of logic inside 10.58 mm², because the routing
channels were sized for convergence rather than area — so break-even at the
current clock is **6.28 mm²**. Alternatively, fixing the I/O paths raises
break-even to **15.23 mm²**, which the design already clears. The I/O fix is
the cheaper of the two: it is an SDC and pin-placement change, not silicon.

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
                       │  mem_arbiter   │      │ MMIO: UART · CLINT · INTC    │
                       └───────┬────────┘      │       LEDs · MM accelerator  │
                       ┌───────▼────────┐      └──────────────────────────────┘
                       │ AXI4 adapter   │──▶ Zynq PS7 / DDR
                       └────────────────┘
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
| **SRAM macros** | **8 KB / 8 KB** | **3.16** | 10.584 mm² | 37 MHz |

The macro's `1RW+1R` ports map onto the existing design without restructuring:
both caches already read at a speculated address and write at a different one
in the same cycle. Reads go on port 1 and writes on port 0 — mandatory, not
stylistic, because the read-write port drives its output to `X` on every clock
edge and only restores it on a read cycle, which would break the assumption
that the output registers hold across stalls.

`openlane/dcache/` and `openlane/icache/` harden each cache standalone, which
keeps floorplan iteration to ~10 minutes instead of ~4 hours in the full CPU.

### Accelerator
A 2×2 output-stationary INT8 systolic array with INT32 accumulation, mapped
as an AXI4-Lite slave. Skewed operand feed, `K_LEN + 2·(DIM−1)` cycle
schedule, driven from C on the core.

### Software
- **Zephyr RTOS** board port: custom SoC/board definition, devicetree,
  UART and timer drivers, second-level interrupt controller
- Bare-metal C and assembly tests, plus a full EEMBC CoreMark port

## Verification

| Test | Coverage |
|---|---|
| `tb_coremark_sim.v` | Full CoreMark against golden CRCs, plus branch/RAS/stall instrumentation |
| `tb_amo_test.v` | All 9 AMO ops — returned old value and committed memory value (18 checks) |
| `tb_dcache_hazard.v` | Store→load bypass, byte/halfword merge, dirty eviction + writeback (7 checks) |
| `tb_alu_mul.v` | MUL/MULH/MULHSU/MULHU, including operands where all three high-forms differ |
| `tb_dcache.v`, `tb_icache.v` | Standalone cache unit tests |
| `tb_sram_wrapper.v` | SRAM macro wrapper against the PDK behavioural model (12 checks) |
| `tb_mm_accel.v` | Accelerator + AXI4-Lite bridge |

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

Correctness is anchored on CoreMark's golden CRCs
(`0xe714`/`0x1fd7`/`0x8e3a`/`0x4983`) — any mismatch means real data
corruption, not a tuning regression. The dcache hazard bench was
**mutation-tested**: disabling the bypass makes 4 of its 7 checks fail with a
stale-data signature, confirming the test detects the bug it targets.

## Repository layout

```
src/          RTL — core, caches, TCM, peripherals, accelerator, AXI
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
