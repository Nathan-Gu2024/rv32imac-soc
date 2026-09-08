#!/usr/bin/env bash
# Summarise the DIM sweep: area, cells and achievable frequency per array size.
#
# Fmax is derived from the worst REGISTER-TO-REGISTER setup path, not from
# period+WNS. Two reasons that matters here, both learned the hard way on the
# CPU build:
#   - the default SDC sets output_delay = CLOCK_PERIOD * IO_PCT, so the output
#     budget scales with the clock and period+WNS overstates the real limit
#   - reg-to-reg is the number that describes the ARRAY; output paths describe
#     the AXI wrapper and the pad ring, which is not what this study is about
set -u
RUNS=$(dirname "$0")/runs

printf "  %-5s %8s %10s %9s %8s %9s %9s\n" DIM PEs "die mm^2" cells util "core ns" "MHz"
for d in 2 4 8 16; do
  m="$RUNS/dim$d/reports/metrics.csv"
  [ -f "$m" ] || { printf "  %-5s %8s %10s\n" "$d" "$((d*d))" "(no run)"; continue; }

  # worst reg-to-reg setup path from the max corner, excluding output ports
  rpt=$(ls "$RUNS/dim$d"/reports/signoff/*sta.max.rpt 2>/dev/null | head -1)
  core=$(python3 - "$rpt" "$RUNS/dim$d/config.tcl" <<'PY' 2>/dev/null
import re,sys
try:
    rpt,cfg = sys.argv[1], sys.argv[2]
    P=float(re.search(r'CLOCK_PERIOD\)\s*"([0-9.]+)"', open(cfg).read()).group(1))
    t=open(rpt,errors="ignore").read()
    best=None
    for b in t.split("Startpoint:")[1:]:
        m=re.search(r"Endpoint:\s*(.+)",b); s=re.search(r"(-?[0-9.]+)\s+slack",b)
        if not(m and s) or "output port" in b.split("Endpoint:")[1][:100]: continue
        v=float(s.group(1))
        best=v if best is None else min(best,v)
    print(f"{P-best:.2f}" if best is not None else "")
except Exception: print("")
PY
)
  python3 - "$m" "$d" "$core" <<'PY'
import csv,sys
m,d,core=sys.argv[1],int(sys.argv[2]),sys.argv[3]
r=list(csv.DictReader(open(m)))[0]
def g(k,dflt="-"):
    v=r.get(k,""); return v if v not in ("","-1") else dflt
mhz = f"{1000/float(core):.1f}" if core else "-"
corestr = core if core else "-"
print("  %-5d %8d %10s %9s %8s %9s %9s" % (
    d, d*d, g("DIEAREA_mm^2"), g("NonPhysCells"), g("OpenDP_Util"), corestr, mhz))
PY
done
