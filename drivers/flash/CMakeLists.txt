# SPDX-License-Identifier: Apache-2.0

zephyr_library()
#zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_M48X flash_m48x.c)
zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_M48X soc_flash_m48x.c)
zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_SPIM_M48X flash_spim_m48x.c)


zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/flash)
#zephyr_library_sources(flash_m48x.c)
