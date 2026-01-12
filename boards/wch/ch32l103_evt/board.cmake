# Copyright (c) 2026 Scott King
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd --target wch_riscv)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
