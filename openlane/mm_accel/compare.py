#!/usr/bin/env python3
"""Side-by-side of the SYNTH_STRATEGY A/B, plus the four-point scaling table.

Fmax comes from the worst REGISTER-TO-REGISTER setup path on the max corner,
excluding output ports - the same rule report.sh uses, and for the same reason:
base.sdc sets output_delay = CLOCK_PERIOD * IO_PCT, so period+WNS tracks the
pad ring rather than the array.
"""
import csv, os, re, glob

H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")


def core_ns(tag):
    rs = glob.glob(os.path.join(H, tag, "reports", "signoff", "*sta.max.rpt"))
    cfg = os.path.join(H, tag, "config.tcl")
    if not rs or not os.path.exists(cfg):
        return None
    m = re.search(r'CLOCK_PERIOD\)\s*"([0-9.]+)"', open(cfg).read())
    if not m:
        return None
    period = float(m.group(1))
    text = open(rs[0], errors="ignore").read()
    best = None
    for blk in text.split("Startpoint:")[1:]:
        end = re.search(r"Endpoint:\s*(.+)", blk)
        slk = re.search(r"(-?[0-9.]+)\s+slack", blk)
        if not (end and slk):
            continue
        if "output port" in blk.split("Endpoint:")[1][:100]:
            continue
        v = float(slk.group(1))
        best = v if best is None else min(best, v)
    return period - best if best is not None else None


def row(tag):
    m = os.path.join(H, tag, "reports", "metrics.csv")
    if not os.path.exists(m):
        return None
    d = list(csv.DictReader(open(m)))[0]
    return d.get("flow_status"), d.get("DIEAREA_mm^2"), d.get("NonPhysCells"), core_ns(tag)


print("  %-5s%-10s%10s%9s%9s%8s   %s" % ("DIM", "strategy", "die mm^2", "cells", "core ns", "MHz", "status"))
data = {}
for d in (2, 4, 8, 16):
    for tag, lab in ((f"dim{d}", "AREA 1"), (f"dim{d}_dly", "DELAY 0")):
        r = row(tag)
        if not r:
            continue
        st, area, cells, c = r
        data[(d, lab)] = (float(area) if area else None, int(cells) if cells else None, c)
        print("  %-5d%-10s%10s%9s%9s%8s   %s" % (
            d, lab,
            f"{float(area):.3f}" if area else "-",
            cells or "-",
            f"{c:.2f}" if c else "-",
            f"{1000/c:.1f}" if c else "-",
            st))

print("\n  --- DELAY 0 vs AREA 1 ---")
for d in (2, 4, 8, 16):
    a, b = data.get((d, "AREA 1")), data.get((d, "DELAY 0"))
    if not (a and b and a[2] and b[2]):
        continue
    fa, fb = 1000 / a[2], 1000 / b[2]
    print("  DIM=%-3d  Fmax %6.1f -> %6.1f MHz (%+5.1f%%)   area %.3f -> %.3f mm2 (%+5.1f%%)   cells %+.1f%%" % (
        d, fa, fb, 100 * (fb - fa) / fa,
        a[0], b[0], 100 * (b[0] - a[0]) / a[0],
        100 * (b[1] - a[1]) / a[1]))
