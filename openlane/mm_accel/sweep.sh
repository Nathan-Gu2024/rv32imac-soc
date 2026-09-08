#!/usr/bin/env bash
# Area / frequency scaling study for the Chisel GEMM generator.
#
# For each DIM: elaborate the Chisel, drop the emitted SystemVerilog into
# rtl/mm_accel.sv, and run the OpenLane flow under tag dim<N>. One source, N
# designs - which is the point of writing the array as a generator.
#
#   ./sweep.sh            # all sizes
#   ./sweep.sh 2 4        # just these
#
# Results land in runs/dim<N>/reports/metrics.csv; summarise with report.sh.
set -u

DIMS=${@:-"2 4 8 16"}
MAXK=16

# Optional A/B knobs. MM_STRATEGY patches SYNTH_STRATEGY in config.json before
# the runs; MM_SUFFIX appends to the run tag so an A/B pair does not overwrite
# itself (runs/dim8 vs runs/dim8_dly). Both default to "unset", in which case
# this behaves exactly as before.
SUFFIX=${MM_SUFFIX:-}
STRATEGY=${MM_STRATEGY:-}
if [ -n "$STRATEGY" ]; then
  python3 - "$STRATEGY" <<'PY'
import json,sys
p="/home/nathangu/OpenLane/designs/RV32-5-stage-processor/openlane/mm_accel/config.json"
d=json.load(open(p)); d["SYNTH_STRATEGY"]=sys.argv[1]
json.dump(d,open(p,"w"),indent=4); open(p,"a").write("\n")
print(f"  SYNTH_STRATEGY set to '{sys.argv[1]}'")
PY
fi

REPO=/home/nathangu/OpenLane/designs/RV32-5-stage-processor
DESIGN=./designs/RV32-5-stage-processor/openlane/mm_accel
HERE="$REPO/openlane/mm_accel"
IMAGE=ghcr.io/the-openroad-project/openlane:ff5509f65b17bfa4068d5336495ab1718987ff69-amd64

mkdir -p "$HERE/rtl"

for d in $DIMS; do
  echo "=============================================================="
  echo "  DIM=$d  maxK=$MAXK"
  echo "=============================================================="

  # 1. elaborate this size straight into the design's rtl/ directory
  ( cd "$REPO/chisel" \
    && COURSIER_CACHE=$HOME/.cache/coursier \
       MM_DIM=$d MM_MAXK=$MAXK MM_OUT="$HERE/rtl" \
       scala-cli run src/MmAccel.scala 2>&1 | grep -E "wrote|error" ) || {
    echo "  elaboration FAILED for DIM=$d"; continue; }

  cells=$(grep -c "SystolicPE " "$HERE/rtl/mm_accel.sv" 2>/dev/null || echo "?")
  echo "  rtl/mm_accel.sv: $(wc -l < "$HERE/rtl/mm_accel.sv") lines"

  # 2. harden it
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
        bash -c "cd /openlane && export PWD=/openlane && ./flow.tcl -design $DESIGN -tag dim$d$SUFFIX -overwrite" \
      2>&1 | tail -4 )
done

echo
echo "done - summarise with ./report.sh"
