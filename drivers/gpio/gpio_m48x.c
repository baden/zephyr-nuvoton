/*
 * Copyright (c) 2021 BaDen, Ukraine
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief GPIO driver for the Nuvoton M480 MPU series.
 */

#include "gpio_m48x.h"


#define DT_DRV_COMPAT delfast_gpio

#include <drivers/gpio.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_m48x);


// #include "drivers/gpio_utils.h"

struct gpio_m48x_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	const struct device *parent;
};

struct gpio_m48x_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
};

static int gpio_m48x_config(const struct device *dev,
				gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_m48x_config *config = dev->config;
	int err = 0;

	if (pin > m48x_GPIO_MAX) {
		return -EINVAL;
	}

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		return -ENOTSUP;
	}

	if (flags & GPIO_INT_ENABLE) {
		/* m48x GPIOs do not support interrupts */
		return -ENOTSUP;
	}

	switch (flags & GPIO_DIR_MASK) {
	case GPIO_INPUT:
		err = m48x_gpio_set_input(config->parent, pin);
		break;
	case GPIO_OUTPUT:
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			err = m48x_gpio_set_pin_value(config->parent, pin,
							  true);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			err = m48x_gpio_set_pin_value(config->parent, pin,
							  false);
		}

		if (err) {
			return err;
		}
		err = m48x_gpio_set_output(config->parent, pin);
		break;
	default:
		return -ENOTSUP;
	}

	return err;
}

static int gpio_m48x_port_get_raw(const struct device *dev,
				      gpio_port_value_t *value)
{
	const struct gpio_m48x_config *config = dev->config;

	return m48x_gpio_port_get_raw(config->parent, value);
}

static int gpio_m48x_port_set_masked_raw(const struct device *dev,
					     gpio_port_pins_t mask,
					     gpio_port_value_t value)
{
	const struct gpio_m48x_config *config = dev->config;

	return m48x_gpio_port_set_masked_raw(config->parent, mask, value);
}

static int gpio_m48x_port_set_bits_raw(const struct device *dev,
					   gpio_port_pins_t pins)
{
	const struct gpio_m48x_config *config = dev->config;

	return m48x_gpio_port_set_bits_raw(config->parent, pins);
}

static int gpio_m48x_port_clear_bits_raw(const struct device *dev,
					     gpio_port_pins_t pins)
{
	const struct gpio_m48x_config *config = dev->config;

	return m48x_gpio_port_clear_bits_raw(config->parent, pins);
}

static int gpio_m48x_port_toggle_bits(const struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_m48x_config *config = dev->config;

	return m48x_gpio_port_toggle_bits(config->parent, pins);
}

static int gpio_m48x_pin_interrupt_configure(const struct device *dev,
						 gpio_pin_t pin,
						 enum gpio_int_mode mode,
						 enum gpio_int_trig trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(mode);
	ARG_UNUSED(trig);

	return -ENOTSUP;
}

static int gpio_m48x_init(const struct device *dev)
{
	const struct gpio_m48x_config *config = dev->config;

	if (!device_is_ready(config->parent)) {
		LOG_ERR("parent m48x device '%s' not ready",
			config->parent->name);
		return -EINVAL;
	}

	return 0;
}

static const struct gpio_driver_api gpio_m48x_api = {
	.pin_configure = gpio_m48x_config,
	.port_set_masked_raw = gpio_m48x_port_set_masked_raw,
	.port_set_bits_raw = gpio_m48x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_m48x_port_clear_bits_raw,
	.port_toggle_bits = gpio_m48x_port_toggle_bits,
	.pin_interrupt_configure = gpio_m48x_pin_interrupt_configure,
	.port_get_raw = gpio_m48x_port_get_raw,
};


#define GPIO_m48x_DEVICE(id)					\
	static const struct gpio_m48x_config gpio_m48x_##id##_cfg = {\
		.common = {                                             \
			.port_pin_mask =                                \
				 GPIO_PORT_PIN_MASK_FROM_DT_INST(id)	\
		},                                                      \
		.parent = DEVICE_DT_GET(DT_INST_BUS(id)),		\
	};								\
									\
	static struct gpio_m48x_data gpio_m48x_##id##_data;	\
									\
	DEVICE_DT_INST_DEFINE(id,					\
			    &gpio_m48x_init,			\
			    NULL,					\
			    &gpio_m48x_##id##_data,			\
			    &gpio_m48x_##id##_cfg, POST_KERNEL,	\
			    CONFIG_GPIO_m48x_INIT_PRIORITY,		\
			    &gpio_m48x_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_m48x_DEVICE)
