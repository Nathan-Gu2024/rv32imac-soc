# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_CACHE_RV32_5STAGE cache_rv32_5stage.c)
