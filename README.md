# RV32IMAC SoC — RISC-V Core, Caches, AI Accelerator, RTOS

A from-scratch RV32IMAC_Zba RISC-V system-on-chip in Verilog, running on a
Xilinx Zynq-7020 FPGA and taken through an open-source ASIC flow. Includes a
5-stage pipelined core with dynamic branch prediction, BRAM-backed L1 caches,
an AXI4-Lite INT8 systolic-array accelerator, and a Zephyr RTOS board port.

**190.5 CoreMark @ 60 MHz — 3.17 CoreMark/MHz**, using 13% of the device's
LUTs and 6% of its flip-flops.

---

## Results

| Metric | Value |
|---|---|
| CoreMark | **190.49 iterations/sec** @ 60 MHz (2000 iterations, validated run) |
| CoreMark/MHz | **3.17** |
| Fmax | 60 MHz (Zynq-7020, `xc7z020clg400-2`), WNS +0.72 ns |
| Utilization | 13% LUT · 6% FF · 22% BRAM · 2% DSP |
| On-chip power | 1.51 W |
| ISA | RV32IMAC + Zicsr + Zba, machine mode |

For reference, [ultraembedded/riscv](https://github.com/ultraembedded/riscv),
a well-known open-source RV32IM core, reports 2.94 CoreMark/MHz. This design
reaches 3.17 while additionally implementing compressed instructions,
atomics, and branch prediction. (Benchmark conditions differ — that core's
memory configuration isn't documented — so treat it as a rough reference
point rather than a controlled comparison.)

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
| `tb_mm_accel.v` | Accelerator + AXI4-Lite bridge |

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

**ASIC**: `openlane/cpu/config.json` drives the OpenLane/Sky130 flow.
RTL-to-GDSII was completed on an earlier core revision; the current design
has not been re-run through it.

## License

CoreMark is EEMBC's, under its own license (`fpga/coremark/src/LICENSE.md`).
