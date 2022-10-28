/*
 * Copyright (c) 2021 BaDen Ukraine
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_M48X_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_M48X_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/* m48x supports GPIO D0..D6 */
#define m48x_GPIO_MAX 6

int m48x_gpio_set_output(const struct device *dev, uint8_t pin);

int m48x_gpio_set_input(const struct device *dev, uint8_t pin);

int m48x_gpio_set_pin_value(const struct device *dev, uint8_t pin,
				bool value);

int m48x_gpio_get_pin_value(const struct device *dev, uint8_t pin,
				bool *value);

int m48x_gpio_port_get_raw(const struct device *dev,
			       gpio_port_value_t *value);

int m48x_gpio_port_set_masked_raw(const struct device *dev,
				      gpio_port_pins_t mask,
				      gpio_port_value_t value);

int m48x_gpio_port_set_bits_raw(const struct device *dev,
				    gpio_port_pins_t pins);

int m48x_gpio_port_clear_bits_raw(const struct device *dev,
				      gpio_port_pins_t pins);

int m48x_gpio_port_toggle_bits(const struct device *dev,
				   gpio_port_pins_t pins);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_m48x_H_ */
