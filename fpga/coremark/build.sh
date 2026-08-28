#!/bin/bash
# Build script for the bare-metal CoreMark port targeting the custom
# RV32IMC 5-stage pipelined CPU. Reuses the same crt0.S/linker_ddr.ld
# already proven working for other DDR-resident hand-written tests
# (fpga/tests/), so the resulting .bin boots via the same TCM boot stub
# jump-to-DDR-0 flow as everything else.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Override with: RISCV_PREFIX=/path/to/riscv-none-elf- bash build.sh
TOOLCHAIN="${RISCV_PREFIX:-riscv-none-elf-}"
CC="${TOOLCHAIN}gcc"
OBJCOPY="${TOOLCHAIN}objcopy"

ITERATIONS="${1:-200}"

OUT_DIR="$SCRIPT_DIR/build"
mkdir -p "$OUT_DIR"

CORE_SRCS="$SCRIPT_DIR/src/core_main.c $SCRIPT_DIR/src/core_list_join.c $SCRIPT_DIR/src/core_matrix.c $SCRIPT_DIR/src/core_state.c $SCRIPT_DIR/src/core_util.c"
PORT_SRCS="$SCRIPT_DIR/core_portme.c $SCRIPT_DIR/ee_printf.c $SCRIPT_DIR/cvt.c"

"$CC" \
    -march=rv32imac_zicsr_zba -mabi=ilp32 -mcmodel=medlow \
    -ffreestanding -nostartfiles -O2 -g \
    -Wall \
    -I "$SCRIPT_DIR/src" -I "$SCRIPT_DIR" \
    -DPERFORMANCE_RUN=1 -DITERATIONS="$ITERATIONS" -DFLAGS_STR='"-O2"' \
    "$SCRIPT_DIR/../tests/crt0.S" $CORE_SRCS $PORT_SRCS \
    -T "$SCRIPT_DIR/../linker/linker_ddr.ld" \
    -o "$OUT_DIR/coremark.elf"

"$OBJCOPY" -O binary "$OUT_DIR/coremark.elf" "$OUT_DIR/coremark.bin"

echo "Built $OUT_DIR/coremark.elf and $OUT_DIR/coremark.bin (ITERATIONS=$ITERATIONS)"
ls -la "$OUT_DIR/coremark.elf" "$OUT_DIR/coremark.bin"
