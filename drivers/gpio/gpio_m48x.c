/*
 * Copyright (c) 2021 BaDen, Ukraine
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief GPIO driver for the Nuvoton M480 MPU series.
 */
#define DT_DRV_COMPAT nuvoton_m48x_gpio

#include "gpio_m48x.h"

#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>

// #include <drivers/gpio/gpio_utils.h>
#include "gpio_utils.h"

// #define LOG_LEVEL  LOG_LEVEL_DBG	//CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_m48x, LOG_LEVEL_DBG);	//CONFIG_GPIO_LOG_LEVEL


struct gpio_m48x_config {
	struct gpio_driver_config common;	/* gpio_driver_config needs to be first */
	GPIO_T *regs;
	// TODO: Look it later
	// #ifdef CONFIG_SAM0_EIC
	// 	uint8_t id;
	// #endif
};

struct gpio_m48x_data {
	struct gpio_driver_data common;	/* gpio_driver_data needs to be first */
	const struct device *dev;
	gpio_port_pins_t debounce;
	// #ifdef CONFIG_SAM0_EIC
	// 	sys_slist_t callbacks;
	// #endif
};

#define DEV_CFG(dev) \
	((const struct gpio_m48x_config *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct gpio_m48x_data *const)(dev)->data)

#ifdef CONFIG_SAM0_EIC
	static void gpio_m48x_isr(uint32_t pins, void *arg)
	{
		struct gpio_m48x_data *const data = (struct gpio_m48x_data *)arg;

		gpio_fire_callbacks(&data->callbacks, data->dev, pins);
	}
#endif

static int gpio_m48x_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);
	GPIO_T *regs = config->regs;


	LOG_DBG("gpio_m48x_config(%s, pin:%d/0x%02X, flags:0x%08X)", dev->name, (unsigned)pin, (unsigned)BIT(pin), (unsigned)flags);

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		LOG_DBG(" -> as single-ended mode (open drain or open source)");
	}

	if ((flags & GPIO_INPUT) != 0) {
		LOG_DBG(" -> as INPUT");
		GPIO_SetMode(regs, BIT(pin), GPIO_MODE_INPUT);
	}
	if ((flags & GPIO_OUTPUT) != 0) {
		LOG_DBG(" -> as OUTPUT");
		// Цель - получить это:
		// GPIO_SetMode(PF, (1<<8), GPIO_MODE_OUTPUT);
		GPIO_SetMode(regs, BIT(pin), GPIO_MODE_OUTPUT);
		// Но только через
		// regs->MODE |= BIT(pin)
	}

	return 0;
}

static int gpio_m48x_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);

	LOG_DBG("gpio_n48x_port_get_raw [%s, %d]", dev->name, config->regs->PIN);
	// *value = config->regs->PIN;
	*value = PD12;

	return -ENOTSUP;
}
#if 0
static int gpio_m48x_port_set_masked_raw(const struct device *dev,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);
	uint32_t out = config->regs->DOUT;

	// LOG_DBG("gpio_m48x_port_set_masked_raw");
	config->regs->DOUT = (out & ~mask) | (value & mask);

	return -ENOTSUP;
}
#endif
static int gpio_m48x_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);
	// GPIO_T *regs = config->regs;

	// LOG_DBG("gpio_m48x_port_set_bits_raw regs:0x%08X 0x%08X", (unsigned)config->regs, (unsigned)pins);
	// LOG_DBG("  -> common.port_pin_mask = 0x%08X", (unsigned)config->common.port_pin_mask);
	config->regs->DOUT |= pins;
	// config->regs->OUTSET.reg = pins;

	return 0;
}

static int gpio_m48x_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);
	GPIO_T *regs = config->regs;
	LOG_DBG("gpio_m48x_port_clear_bits_raw regs:0x%08X 0x%08X", (unsigned)config->regs, (unsigned)pins);
	LOG_DBG("  -> common.port_pin_mask = 0x%08X", (unsigned)config->common.port_pin_mask);

	regs->DOUT &= ~pins;
	// config->regs->OUTCLR.reg = pins;

	return 0;
}

#if 0
static int gpio_m48x_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	// const struct gpio_m48x_config *config = DEV_CFG(dev);
	LOG_DBG("gpio_m48x_port_toggle_bits");

	// config->regs->OUTTGL.reg = pins;

	return 0;
}
#endif

static const struct gpio_driver_api gpio_m48x_api = {
	.pin_configure = gpio_m48x_config,
	.port_get_raw = gpio_m48x_port_get_raw,
	// .port_set_masked_raw = gpio_m48x_port_set_masked_raw,
	.port_set_bits_raw = gpio_m48x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_m48x_port_clear_bits_raw,
	// .port_toggle_bits = gpio_m48x_port_toggle_bits,
	// #ifdef CONFIG_SAM0_EIC
	// 	.pin_interrupt_configure = gpio_m48x_pin_interrupt_configure,
	// 	.manage_callback = gpio_m48x_manage_callback,
	// 	.get_pending_int = gpio_m48x_get_pending_int,
	// #endif
};


static int gpio_m48x_init(const struct device *dev) {
	LOG_DBG("gpio_m48x_init [%s]", dev->name);

	// LOG_DBG("PA = 0x%08X", (unsigned)PA);
	// LOG_DBG("PB = 0x%08X", (unsigned)PB);
	// LOG_DBG("PC = 0x%08X", (unsigned)PC);
	// LOG_DBG("PD = 0x%08X", (unsigned)PD);
	// LOG_DBG("PE = 0x%08X", (unsigned)PE);
	// LOG_DBG("PF = 0x%08X", (unsigned)PF);
	// LOG_DBG("PG = 0x%08X", (unsigned)PG);
	// LOG_DBG("PH = 0x%08X", (unsigned)PH);
	return 0;
}

#if 1

/* Port A */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porta), okay)

static const struct gpio_m48x_config gpio_m48x_config_0 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(porta)),
};

static struct gpio_m48x_data gpio_m48x_data_0;

DEVICE_DT_DEFINE(DT_NODELABEL(porta),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_0, &gpio_m48x_config_0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);
#endif

/* Port B */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portb), okay)

static const struct gpio_m48x_config gpio_m48x_config_1 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(1),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portb)),
};

static struct gpio_m48x_data gpio_m48x_data_1;

DEVICE_DT_DEFINE(DT_NODELABEL(portb),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_1, &gpio_m48x_config_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);
#endif

/* Port C */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portc), okay)

static const struct gpio_m48x_config gpio_m48x_config_2 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(2),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portc)),
};

static struct gpio_m48x_data gpio_m48x_data_2;

DEVICE_DT_DEFINE(DT_NODELABEL(portc),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_2, &gpio_m48x_config_2,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* Port D */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portd), okay)

static const struct gpio_m48x_config gpio_m48x_config_3 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(3),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portd)),
};

static struct gpio_m48x_data gpio_m48x_data_3;

DEVICE_DT_DEFINE(DT_NODELABEL(portd),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_3, &gpio_m48x_config_3,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* Port E */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porte), okay)

static const struct gpio_m48x_config gpio_m48x_config_4 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(4),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(porte)),
};

static struct gpio_m48x_data gpio_m48x_data_4;

DEVICE_DT_DEFINE(DT_NODELABEL(porte),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_4, &gpio_m48x_config_4,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif


/* Port F */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portf), okay)

static const struct gpio_m48x_config gpio_m48x_config_5 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(5),
	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portf)),
	// #ifdef CONFIG_SAM0_EIC
	// 	.id = 0,
	// #endif
};

static struct gpio_m48x_data gpio_m48x_data_5;

DEVICE_DT_DEFINE(DT_NODELABEL(portf),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_5, &gpio_m48x_config_5,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);
#endif

/* Port G */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portg), okay)

static const struct gpio_m48x_config gpio_m48x_config_6 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(6),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portg)),
};

static struct gpio_m48x_data gpio_m48x_data_6;

DEVICE_DT_DEFINE(DT_NODELABEL(portg),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_6, &gpio_m48x_config_6,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* Port H */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porth), okay)

static const struct gpio_m48x_config gpio_m48x_config_7 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(7),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(porth)),
};

static struct gpio_m48x_data gpio_m48x_data_7;

DEVICE_DT_DEFINE(DT_NODELABEL(porth),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_7, &gpio_m48x_config_7,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif



#endif // 0

//.parent = DEVICE_DT_GET(DT_INST_BUS(id)),
#if 0
#define GPIO_m48x_DEVICE(id)											\
	static const struct gpio_m48x_config gpio_m48x_##id##_cfg = {		\
		.common = {                                             		\
			.port_pin_mask =                                			\
				 GPIO_PORT_PIN_MASK_FROM_DT_INST(id)					\
		},                                                      		\
		.regs = (GPIO_T *)DT_REG_ADDR(id),				\
	};																	\
																		\
	static struct gpio_m48x_data gpio_m48x_##id##_data;					\
																		\
	DEVICE_DT_INST_DEFINE(id,											\
			    &gpio_m48x_init,										\
			    NULL,													\
			    &gpio_m48x_##id##_data,									\
			    &gpio_m48x_##id##_cfg, POST_KERNEL,						\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,						\
			    &gpio_m48x_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_m48x_DEVICE)

#endif



















#if 0
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
#endif
