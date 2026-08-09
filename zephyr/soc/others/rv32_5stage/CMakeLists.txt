# Custom RV32IMC 5-stage pipelined RISC-V SoC (FPGA soft-core)
# SPDX-License-Identifier: Apache-2.0

# No SoC-specific reset/isr/init code needed yet - the generic
# arch/riscv/core reset.S and isr.S cover a plain single-hart, standard
# mstatus/mie/mip trap model, which is what this CPU implements. Revisit if
# a build shows we need a soc.c hook (e.g. for a custom cache/PMP quirk).
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
