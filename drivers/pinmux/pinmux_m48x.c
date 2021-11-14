#include <drivers/pinmux.h>
#include <soc.h>

// Not realy needed
#define DT_DRV_COMPAT nuvoton_m48x_pinmux

// TODO: Remove after finished
#include <logging/log.h>
LOG_MODULE_REGISTER(pinmux_m48x_pinmux, LOG_LEVEL_DBG);

// TODO:
//  For GPIO, use: (GPIO_T *) : PA, PB, PC, ...

struct pinmux_m48x_config {
	// PortGroup *regs;
	SYS_T *regs;
};

static int pinmux_m48x_set(const struct device *dev, uint32_t pin, uint32_t func)
{
	// const struct pinmux_sam0_config *cfg = dev->config;
	LOG_DBG("pinmux_m48x_set");
	return -ENOTSUP;
}

static int pinmux_m48x_get(const struct device *dev, uint32_t pin, uint32_t *func)
{
	// const struct pinmux_m48x_config *cfg = dev->config;
	LOG_DBG("pinmux_m48x_get");
	return -ENOTSUP;
}

static int pinmux_m48x_pullup(const struct device *dev, uint32_t pin, uint8_t func)
{
	// const struct pinmux_m48x_config *cfg = dev->config;
	LOG_DBG("pinmux_m48x_pullup");
	return -ENOTSUP;
}

static int pinmux_m48x_input(const struct device *dev, uint32_t pin, uint8_t func)
{
	// const struct pinmux_m48x_config *cfg = dev->config;
	LOG_DBG("pinmux_m48x_input");
	return -ENOTSUP;
}

// Multi-function pin
// SYS->GPA_MFPL

static int pinmux_m48x_init(const struct device *dev)
{
	/* Nothing to do. */
	const struct pinmux_m48x_config *cfg = dev->config;
	LOG_DBG("init regs=%08X", (unsigned)cfg->regs);
	LOG_DBG("SYS_BASE=%08X", (unsigned)SYS_BASE);
	LOG_DBG("SYS=%08X", (unsigned)SYS);
	LOG_DBG("UART0_BASE=%08X", (unsigned)UART0_BASE);
	LOG_DBG("SYS->GPA_MFPL=%08X", (unsigned)&SYS->GPA_MFPL);
	LOG_DBG("SYS->GPB_MFPL=%08X", (unsigned)&SYS->GPB_MFPL);
	LOG_DBG("SYS->GPC_MFPL=%08X", (unsigned)&SYS->GPC_MFPL);
	LOG_DBG("SYS->GPD_MFPL=%08X", (unsigned)&SYS->GPD_MFPL);
	LOG_DBG("SYS->GPE_MFPL=%08X", (unsigned)&SYS->GPE_MFPL);
	LOG_DBG("SYS->GPF_MFPL=%08X", (unsigned)&SYS->GPF_MFPL);
	LOG_DBG("SYS->GPG_MFPL=%08X", (unsigned)&SYS->GPG_MFPL);
	LOG_DBG("SYS->GPH_MFPL=%08X", (unsigned)&SYS->GPH_MFPL);
	return 0;
}


const struct pinmux_driver_api pinmux_m48x_api = {
	.set = pinmux_m48x_set,
	.get = pinmux_m48x_get,
	.pullup = pinmux_m48x_pullup,
	.input = pinmux_m48x_input,
};


// #if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_a), okay)
static const struct pinmux_m48x_config pinmux_m48x_config_0 = {
	.regs = (SYS_T *)DT_REG_ADDR(DT_NODELABEL(pinmux_a)),
};

DEVICE_DT_DEFINE(DT_NODELABEL(pinmux_a), pinmux_m48x_init,
		    NULL, NULL, &pinmux_m48x_config_0,
		    PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY,
		    &pinmux_m48x_api);
// #endif
