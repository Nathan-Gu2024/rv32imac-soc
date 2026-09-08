#!/usr/bin/env bash
# Two-phase follow-up to the DIM=16 run.
#
# Phase 1 re-runs DIM 2/4/8 on the CURRENT RTL (two-stage readback) at
# SYNTH_STRATEGY "AREA 1" - the same strategy DIM=16 used. That is what makes
# the four-point scaling table legitimate: one RTL, one strategy, four sizes.
# The small sizes never needed the readback fix (their muxes were 4:1 and
# 16:1); this is for methodological consistency, not repair.
#
# Phase 2 re-runs the same three sizes at "DELAY 0" to measure how much Fmax
# the area-oriented strategy was leaving on the table. Tagged dim<N>_dly so it
# sits beside phase 1 rather than overwriting it, giving a controlled A/B at
# three sizes.
#
# DIM=16 is deliberately NOT re-run at DELAY here - it costs 4+ hours. Decide
# that after seeing whether the A/B is worth it.
set -u
HERE=/home/nathangu/OpenLane/designs/RV32-5-stage-processor/openlane/mm_accel

# --- wait for the in-flight DIM=16 run to release the machine ---
while docker ps --format '{{.ID}}' | grep -q .; do sleep 60; done
echo "=== DIM=16 finished, machine free ==="
if [ -f "$HERE/runs/dim16/reports/metrics.csv" ]; then
  echo "--- dim16 result ---"; ( cd "$HERE" && ./report.sh )
else
  echo "--- dim16 produced no metrics.csv (check runs/dim16/openlane.log) ---"
fi

echo
echo "=================== PHASE 1: 2/4/8 at AREA 1 ==================="
MM_STRATEGY="AREA 1" MM_SUFFIX="" "$HERE/sweep.sh" 2 4 8

echo
echo "--- four-point scaling study, consistent RTL + strategy ---"
( cd "$HERE" && ./report.sh )

echo
echo "=================== PHASE 2: 2/4/8 at DELAY 0 =================="
MM_STRATEGY="DELAY 0" MM_SUFFIX="_dly" "$HERE/sweep.sh" 2 4 8

# restore the strategy the study is built on, so the config on disk matches
# the four-point table rather than the last experiment run
python3 - <<'PY'
import json
p="/home/nathangu/OpenLane/designs/RV32-5-stage-processor/openlane/mm_accel/config.json"
d=json.load(open(p)); d["SYNTH_STRATEGY"]="AREA 1"
json.dump(d,open(p,"w"),indent=4); open(p,"a").write("\n")
print("  config.json SYNTH_STRATEGY restored to 'AREA 1'")
PY

echo
echo "=== done - phase 1 in runs/dim{2,4,8}, phase 2 in runs/dim{2,4,8}_dly ==="
