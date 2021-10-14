# SPDX-License-Identifier: Apache-2.0

board_runner_args(nulink "-f")

set(OPENOCD "~/.local/openocd/bin/openocd" CACHE FILEPATH "" FORCE)
set(OPENOCD_DEFAULT_PATH ~/.local/openocd/share/openocd/scripts)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nulink.board.cmake)
