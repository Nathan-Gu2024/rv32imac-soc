# Phase 2a — five-macro D-cache hardening

`dcache_bram` built standalone with `USE_SRAM_MACRO=1 NUM_SETS=512`, so the
five-macro floorplan could be settled in ~10-minute runs instead of inside a
3-hour full-CPU run.

## Result

`flow completed` — 3.708 mm², 4,206 → 6,010 cells, **WNS/TNS 0.0, no setup or
hold violations**, GDS written, 12 m 05 s.

## The floorplan recipe

Five macros in a **single row, all `N`**. Each macro carries all 125 signal
pins on met4 along its *bottom* edge and blocks met1–met3 across its whole
footprint, so every pin edge must face open standard-cell area. One row gives
one shared channel below that serves all five, keeping the FSM, word muxes and
512 dirty flops in a single region close to every macro.

**An inter-macro gap is not a routing channel of its nominal width.**
`PL_MACRO_HALO` reserves 10 µm per side and the PDN halo takes more. The first
attempt used 30 µm gaps and failed `GRT-0119` with 5 vertical-congestion
violations (`capacity:0`), all in the gap between macros 2 and 3 — while 125
pins per macro tried to escape downward through it. 100 µm gaps and a
350 µm below-channel route cleanly.

| | first attempt | working |
|---|---|---|
| inter-macro gap | 30 µm | **100 µm** |
| channel below | 250 µm | **350 µm** |
| die | 3840 × 800 | 4120 × 900 |

Fallback if a future variant congests: two rows with the upper one mirrored
(`MX`) so its pins face up, giving a squarer ~2350 × 1300 die at the cost of
splitting control logic across two channels.

## Magic DRC is not a usable signoff gate here — read this before "fixing" it

Both settings produce large false-positive counts, and there is no third
option without an SRAM-aware DRC deck:

| `MAGIC_DRC_USE_GDS` | violations | why |
|---|---|---|
| **0 (use this)** | 1,733 | LEF/DEF abstracts hide standard-cell internals, so Magic reports `nwell.4` "no metal-connected N+ tap" for nwells whose taps it cannot see |
| 1 (default) | **27,871,483** | checks real macro GDS; sky130 SRAM cells use optical-proximity-shrunk transistors whose rules the open PDK deck lacks, so every internal transistor flags. The flow fails outright. |

The `nwell.4` count is a measurement artifact, not missing taps. Taps are
demonstrably present: 29,777 in the design at the correct ~12.9 µm pitch
(`FP_TAPCELL_DIST 13`), 132 of them inside a violation region that Magic
reported as untapped.

Two hypotheses were tested and **disproven** — do not retry them:

1. *Efabless decap cells cause it.* Restricting `DECAP_CELL` to
   `sky130_fd_sc_hd__*` left `nwell.4` at exactly 1,707, unchanged.
2. *GDS-based DRC would be more accurate.* It is 16,000× worse (above).

Timing, routing and LVS-relevant structure are all clean; Magic DRC on a
macro design in this flow simply is not informative. The CPU's own signoff run
(no macros) uses the default `1` and reports 0 violations — that number is
meaningful, this one is not.
