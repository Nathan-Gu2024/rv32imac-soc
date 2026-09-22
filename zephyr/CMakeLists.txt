# rv32_5stage out-of-tree Zephyr module: board+SoC files live in
# boards/ and soc/ (discovered directly via BOARD_ROOT/SOC_ROOT, no
# CMake integration needed for those), custom peripheral drivers live
# in drivers/ and need to be pulled in here.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(drivers/serial)
add_subdirectory(drivers/interrupt_controller)
add_subdirectory(drivers/cache)
add_subdirectory(drivers/misc)
