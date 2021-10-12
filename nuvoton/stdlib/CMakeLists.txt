zephyr_library_named(baden-nuvoton-sources)

zephyr_library_include_directories(include)

zephyr_library_sources(src/uuart.c)

zephyr_library_link_libraries(nuvoton)
