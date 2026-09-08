#!/usr/bin/env bash
# Confirmation pass: close timing at each size instead of inferring Fmax.
#
# The sweep so far asked every size for 10 ns and read the achieved critical
# path out of the max-corner report. That is a valid MEASUREMENT, but three of
# the four runs end in "flow failed" because they cannot hold 10 ns - so the
# study proves nothing was signed off. This pass sets each size's clock to what
# it actually achieved plus ~5% margin, so a pass here means the design really
# closes.
#
# Clocks come from the AREA 1 rerun on the two-stage-readback RTL:
#     dim2 10.38  dim4 11.18  dim8 11.58  dim16 13.07 ns
#
# Hold margins go 0.3/0.25 -> 0.4/0.35. dim16 missed hold by only 0.03-0.05 ns,
# and a small bump is the right size of correction: on the CPU, over-correcting
# hold (0.3/0.25 -> 0.8/0.7 for a 0.39 ns miss) inserted 990 buffers and
# stalled routing. Do not reach for 0.8 here.
#
# Strategy stays AREA 1: the DELAY 0 A/B bought +15-20% Fmax for +31% area,
# which is break-even to negative under f/(cycles*sqrt(area)), and it crashed
# the resizer at DIM=8 (RSZ-0075 makeBufferedNet).
set -u

REPO=/home/nathangu/OpenLane/designs/RV32-5-stage-processor
DESIGN=./designs/RV32-5-stage-processor/openlane/mm_accel
HERE="$REPO/openlane/mm_accel"
IMAGE=ghcr.io/the-openroad-project/openlane:ff5509f65b17bfa4068d5336495ab1718987ff69-amd64
CFG="$HERE/config.json"
MAXK=16

cp "$CFG" "$CFG.bak"
restore() { mv -f "$CFG.bak" "$CFG" 2>/dev/null && echo "  config.json restored"; }
trap restore EXIT

run_one() {
  d=$1; period=$2
  echo "=============================================================="
  echo "  DIM=$d  signoff attempt at ${period} ns"
  echo "=============================================================="

  python3 - "$CFG" "$period" <<'PY'
import json, sys
p, period = sys.argv[1], float(sys.argv[2])
d = json.load(open(p))
d["CLOCK_PERIOD"] = period
d["SYNTH_STRATEGY"] = "AREA 1"
d["PL_RESIZER_HOLD_SLACK_MARGIN"] = 0.4
d["GLB_RESIZER_HOLD_SLACK_MARGIN"] = 0.35
json.dump(d, open(p, "w"), indent=4); open(p, "a").write("\n")
print(f"  CLOCK_PERIOD={period}  hold margins 0.4/0.35  strategy AREA 1")
PY

  ( cd "$REPO/chisel" \
    && COURSIER_CACHE=$HOME/.cache/coursier \
       MM_DIM=$d MM_MAXK=$MAXK MM_OUT="$HERE/rtl" \
       scala-cli run src/MmAccel.scala 2>&1 | grep -E "wrote|error" ) || {
    echo "  elaboration FAILED for DIM=$d"; return 1; }

  ( cd /home/nathangu/OpenLane \
    && docker run --rm \
        -v /home/nathangu/OpenLane:/openlane \
        -v /home/nathangu/OpenLane/designs:/openlane/install \
        -v /home/nathangu:/home/nathangu \
        -v /home/nathangu/.ciel:/home/nathangu/.ciel \
        -e PDK_ROOT=/home/nathangu/.ciel -e PDK=sky130A \
        --user "$(id -u):$(id -g)" --network host \
        --security-opt seccomp=unconfined \
        "$IMAGE" \
        bash -c "cd /openlane && export PWD=/openlane && ./flow.tcl -design $DESIGN -tag dim${d}_so -overwrite" \
      2>&1 | tail -4 )

  st=$(python3 -c "
import csv,os
p='$HERE/runs/dim${d}_so/reports/metrics.csv'
print(list(csv.DictReader(open(p)))[0].get('flow_status') if os.path.exists(p) else 'no metrics')" 2>/dev/null)
  echo "  --> DIM=$d at ${period} ns: $st"
}

run_one 2  11.0
run_one 4  11.8
run_one 8  12.2
run_one 16 13.8

echo
echo "=== signoff pass complete ==="
python3 "$HERE/compare.py" 2>/dev/null || true
