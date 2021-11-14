zephyr_library_named(nuvoton__drivers__gpio)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_SOC_M487 gpio_m48x.c)
zephyr_library_sources_ifndef(CONFIG_SOC_M487 ${ZEPHYR_BASE}/misc/empty_file.c)
