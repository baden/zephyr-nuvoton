# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SPI_M48X spi_m48x.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/spi)
