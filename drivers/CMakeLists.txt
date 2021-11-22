# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SENSOR sensor)
add_subdirectory_ifdef(CONFIG_PINMUX pinmux)
add_subdirectory_ifdef(CONFIG_GPIO gpio)
add_subdirectory_ifdef(CONFIG_SERIAL_HAS_DRIVER serial)
#add_subdirectory_ifdef(CONFIG_FLASH_HAS_DRIVER_ENABLED flash)
add_subdirectory(flash)
