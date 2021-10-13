# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_UART_DELFAST uart_numicro.c)

if(CONFIG_UART_NATIVE_POSIX)
  zephyr_library_compile_definitions(NO_POSIX_CHEATS)
  zephyr_library_sources(uart_native_posix.c)
endif()

zephyr_library_sources_ifdef(CONFIG_SERIAL_TEST		serial_test.c)
