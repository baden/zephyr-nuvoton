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
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/m48x.h>

// #include <drivers/gpio/gpio_utils.h>
#include "gpio_utils.h"

// #define LOG_LEVEL  LOG_LEVEL_DBG	//CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(gpio_m48x, LOG_LEVEL_DBG);	//CONFIG_GPIO_LOG_LEVEL
LOG_MODULE_REGISTER(gpio_m48x, LOG_LEVEL_INF);	//CONFIG_GPIO_LOG_LEVEL

typedef void (*config_func_t)(const struct device *dev);

struct gpio_m48x_config {
	struct gpio_driver_config common;	/* gpio_driver_config needs to be first */
	GPIO_T *regs;
	config_func_t config_func;
	// uint32_t periph_id;
};

struct gpio_m48x_data {
	struct gpio_driver_data common;	/* gpio_driver_data needs to be first */
	const struct device *dev;
	gpio_port_pins_t debounce;
	sys_slist_t cb;
};

#define DEV_CFG(dev) \
	((const struct gpio_m48x_config *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct gpio_m48x_data *const)(dev)->data)

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

	if (flags & M48X_GPIO_INT_DEBOUNCE) {
		// GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_256);
		GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_512);
	    GPIO_ENABLE_DEBOUNCE(regs, BIT(pin));
	}

	return 0;
}

static int gpio_m48x_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);

	// LOG_DBG("gpio_n48x_port_get_raw [%s, %d]", dev->name, config->regs->PIN);
	*value = config->regs->PIN;
	// *value = PD12;

	return 0;
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

// static void gpio_m48x_isr(uint32_t pins, void *arg)
static void gpio_m48x_isr(const struct device *dev)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);
	// struct gpio_m48x_data *const data = (struct gpio_m48x_data *)arg;
	struct gpio_m48x_data *context = dev->data;
	GPIO_T *regs = config->regs;
	uint32_t int_stat;

	int_stat = regs->INTSRC;
	// GPIO_CLR_INT_FLAG(PD, BIT(12));
	// Write it back for clearing interrupt
	regs->INTSRC = int_stat;

	// printk(":%p:0x%08X", dev, int_stat);

	// gpio_fire_callbacks(&data->cb, data->dev, pins);
	gpio_fire_callbacks(&context->cb, dev, int_stat);
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

static int gpio_m48x_pin_interrupt_configure(const struct device *dev,
					    gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	const struct gpio_m48x_config *config = DEV_CFG(dev);
	// struct gpio_m48x_data *context = dev->data;
	GPIO_T *regs = config->regs;

	LOG_DBG("gpio_m48x_pin_interrupt_configure regs:%p 0x%08X m:0x%04X t:0x%04X", regs, (unsigned)pin, (int)mode, (int)trig);

	GPIO_DisableInt(regs, pin);
	GPIO_CLR_INT_FLAG(regs, pin);

	if (mode != GPIO_INT_MODE_DISABLED) {
		if (mode == GPIO_INT_MODE_EDGE) {
			if (trig == GPIO_INT_TRIG_BOTH) {
				GPIO_EnableInt(regs, pin, GPIO_INT_BOTH_EDGE);
			} else if (trig == GPIO_INT_TRIG_HIGH) {
				GPIO_EnableInt(regs, pin, GPIO_INT_RISING);
			} else { /* GPIO_INT_TRIG_LOW */
				GPIO_EnableInt(regs, pin, GPIO_INT_FALLING);
			}
		} else {
			return -ENOTSUP;
		}
	} else {
	}

	// TODO: mode, trig
	return 0;
}

static int gpio_m48x_manage_callback(const struct device *dev,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_m48x_data *context = dev->data;
	LOG_DBG("gpio_m48x_manage_callback regs:%p %d", DEV_CFG(dev)->regs, (unsigned)set);

	return gpio_manage_callback(&context->cb, callback, set);
}


static const struct gpio_driver_api gpio_m48x_api = {
	.pin_configure = gpio_m48x_config,
	.port_get_raw = gpio_m48x_port_get_raw,
	// .port_set_masked_raw = gpio_m48x_port_set_masked_raw,
	.port_set_bits_raw = gpio_m48x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_m48x_port_clear_bits_raw,
	.pin_interrupt_configure = gpio_m48x_pin_interrupt_configure,
	.manage_callback = gpio_m48x_manage_callback,
};


static int gpio_m48x_init(const struct device *dev) {
	const struct gpio_m48x_config * const cfg = DEV_CFG(dev);
	LOG_DBG("gpio_m48x_init/%p [%s]", dev, dev->name);
	cfg->config_func(dev);
	return 0;
}

#if 1

/* ====================================== Port A  ========================= */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porta), okay)

ISR_DIRECT_DECLARE(GPA_IRQHandler)
{
	// printk("<a>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(porta)));
	return 0;
}

static void port_a_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPA_IRQn, 0, GPA_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPA_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_0 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(porta)),
	.config_func = port_a_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_0;

DEVICE_DT_DEFINE(DT_NODELABEL(porta),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_0, &gpio_m48x_config_0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* ====================== Port B ==================== */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portb), okay)

ISR_DIRECT_DECLARE(GPB_IRQHandler)
{
	// printk("<b>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(portb)));
	return 0;
}

static void port_b_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPB_IRQn, 0, GPB_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPB_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_1 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(1),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portb)),
	.config_func = port_b_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_1;

DEVICE_DT_DEFINE(DT_NODELABEL(portb),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_1, &gpio_m48x_config_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);
#endif

/* ================== Port C ================ */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portc), okay)

ISR_DIRECT_DECLARE(GPC_IRQHandler)
{
	// printk("<c>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(portc)));
	return 0;
}

static void port_c_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPC_IRQn, 0, GPC_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPC_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_2 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(2),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portc)),
	.config_func = port_c_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_2;

DEVICE_DT_DEFINE(DT_NODELABEL(portc),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_2, &gpio_m48x_config_2,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* ====================== Port D ================= */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portd), okay)

ISR_DIRECT_DECLARE(GPD_IRQHandler)
{
	// printk("<d>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(portd)));
	return 0;
}

static void port_d_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPD_IRQn, 0, GPD_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPD_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_3 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(3),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portd)),
	.config_func = port_d_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_3;

DEVICE_DT_DEFINE(DT_NODELABEL(portd),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_3, &gpio_m48x_config_3,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* ==================== Port E ================= */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porte), okay)

ISR_DIRECT_DECLARE(GPE_IRQHandler)
{
	// printk("<e>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(porte)));
	return 0;
}

static void port_e_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPE_IRQn, 0, GPE_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPE_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_4 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(4),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(porte)),
	.config_func = port_e_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_4;

DEVICE_DT_DEFINE(DT_NODELABEL(porte),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_4, &gpio_m48x_config_4,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif


/* =================== Port F ================ */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portf), okay)

ISR_DIRECT_DECLARE(GPF_IRQHandler)
{
	// printk("<f>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(portf)));
	return 0;
}

static void port_f_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPF_IRQn, 0, GPF_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPF_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_5 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(5),
	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portf)),
	.config_func = port_f_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_5;

DEVICE_DT_DEFINE(DT_NODELABEL(portf),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_5, &gpio_m48x_config_5,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);
#endif

/* ================= Port G ================= */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portg), okay)

ISR_DIRECT_DECLARE(GPG_IRQHandler)
{
	// printk("<g>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(portg)));
	return 0;
}

static void port_g_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPG_IRQn, 0, GPG_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPG_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_6 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(6),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(portg)),
	.config_func = port_g_m48x_config_func,
};

static struct gpio_m48x_data gpio_m48x_data_6;

DEVICE_DT_DEFINE(DT_NODELABEL(portg),
		    gpio_m48x_init, NULL,
		    &gpio_m48x_data_6, &gpio_m48x_config_6,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_m48x_api);

#endif

/* ==================== Port H =============== */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porth), okay)

ISR_DIRECT_DECLARE(GPH_IRQHandler)
{
	// printk("<h>");
	gpio_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(porth)));
	return 0;
}

static void port_h_m48x_config_func(const struct device *dev)
{
	IRQ_DIRECT_CONNECT(
		GPH_IRQn, 0, GPH_IRQHandler,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
    irq_enable(GPH_IRQn);
}

static const struct gpio_m48x_config gpio_m48x_config_7 = {
	.common = {		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(7),	},
	.regs = (GPIO_T *)DT_REG_ADDR(DT_NODELABEL(porth)),
	.config_func = port_h_m48x_config_func,
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

// #define GPIO_m48x_INIT(n)
// 	int fffff_##n = DT_DRV_INST(n);
// 	extern int fffff_##n;
//
// DT_INST_FOREACH_STATUS_OKAY(GPIO_m48x_INIT)
