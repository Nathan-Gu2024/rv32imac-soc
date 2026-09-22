# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_GEMM_RV32_5STAGE gemm_rv32_5stage.c)
zephyr_include_directories_ifdef(CONFIG_GEMM_RV32_5STAGE ../../include)
