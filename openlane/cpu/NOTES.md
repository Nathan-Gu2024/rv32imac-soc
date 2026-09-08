# CPU ASIC build notes

Two companion files carry their own history and should be read before
changing anything here:

- `macro_placement.cfg` — the 11-macro floorplan, with the full revision log
  of six routing attempts (two of whose "fixes" made things worse or did
  nothing).
- `../dcache/NOTES.md` — why Magic DRC is not a usable gate once macros are
  present, and two **disproven** DRC hypotheses.

## `pin_order.cfg` — why the pins are grouped this way

**The file cannot contain comments.** `scripts/odbpy/io_place.py` splits every
line on whitespace and exits with `Only one entry allowed per line.` if a line
has more than one token. Only blank lines are skipped; even a single-token
`#comment` fails, because a `#`-prefixed token is matched against the valid
directive list (`#N #E #S #W`, optional `R` suffix, `#BUS_SORT`) and anything
else is an error. Hence this file.

### The problem it solves

With the default `FP_IO_MODE=1` (random equidistant) the 602 pins were
scattered over all four edges — `store_data` alone had 11 north, 12 south,
7 west and 2 east. Their drivers sit in the central logic channel
(y 476..1680), so individual bits ran diagonally across a 4900 × 2160 µm die.

In the `signoff_macro` run **every** setup violation was an output port,
`store_data[30]` worst at −4.55 ns. That is what forced the clock to 27 ns
(37 MHz), even though the worst of 958 register-to-register paths has 2.66 ns
of slack at 20 ns — a core limit of **17.34 ns / 57.7 MHz**.

### The constraint

The macro rows wall off the north and south edges: the I-cache row occupies
y 1680..2096 and the D-cache row y 60..476, and each blocks met1–met3 across
its footprint. Only **east and west** sit directly against the logic channel.

So timing-critical signals go east/west, and the wide buses that have slack go
north/south where they can afford to climb over a macro on met4/met5.

| Edge | Contents | Pins | Pitch | Why |
|---|---|---|---|---|
| W | `store_data`, `dmem_req_addr`, `mem_write_mask`, D-cache control | 71 | 17.9 µm | the critical group |
| E | `dcache_mem_wline` | 128 | 16.9 µm | output, but leaves the cache FSM with slack |
| N | I-cache interface, all `debug_*` | 265 | 18.5 µm | `base.sdc` false-paths every `debug_*` port |
| S | D-cache refill data, clk/rst/uart/leds | 136 | 36.0 µm | an *input* — required-time, not arrival-time |

### The `$25` padding is load-bearing

`$<n>` inserts `n` virtual pin slots. Twenty-five before and after the west
group puts those 71 pins in roughly the middle 60% of the 2138 µm edge, i.e.
against the logic channel. Without the padding they spread the full edge
height and about half would land beside a macro row — which is the situation
this file exists to avoid.

All 600 signal pins must be matched: `FP_IO_UNMATCHED_ERROR` defaults to 1.
(`vccd1`/`vssd1` are handled by the PDN, not here.)
