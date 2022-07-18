/*
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_m48x_watchdog

#include <soc.h>
#include <zephyr/drivers/watchdog.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(wdt_m48x);

//  WDT_TIMEOUT_2POW14	// ~ 1.6375 sec period
//  WDT_TIMEOUT_2POW16	// ~ 6.55 sec period
#define CONFIG_FIXED_INTERVAL 	WDT_TIMEOUT_2POW18	// ~ 26.2 sec period

#define WDT_REGS ((Wdt *)DT_INST_REG_ADDR(0))

struct wdt_m48x_dev_data {
	wdt_callback_t cb;
	uint8_t flags;
};

static struct wdt_m48x_dev_data wdt_m48x_data = { 0 };

static inline void wdt_m48x_enable()
{
	unsigned int key = irq_lock();
	SYS_UnlockReg();
	WDT->CTL &= ~WDT_CTL_IF_Msk;
	WDT->CTL |= WDT_CTL_WDTEN_Msk;
	SYS_LockReg();
	irq_unlock(key);
}

static void wdt_m48x_isr(const struct device *dev)
{
	struct wdt_m48x_dev_data *data = dev->data;

	WDT->CTL |= WDT_CTL_IF_Msk;
	PA11 ^= 1;

	if (data->cb != NULL) {
		data->cb(dev, 0);
	}
	if (data->flags != WDT_FLAG_RESET_NONE) {
		LOG_PANIC();
	}
}

static int wdt_m48x_feed(const struct device *dev, int channel_id);

static int wdt_m48x_setup(const struct device *dev, uint8_t options)
{
	// struct wdt_m48x_dev_data *data = dev->data;

	LOG_DBG("Watchdog setup [0x%02x]", options);

	/* Enable watchdog */
	wdt_m48x_enable();
	wdt_m48x_feed(dev, 0);

	return 0;
}

static int wdt_m48x_disable(const struct device *dev)
{
	unsigned int key = irq_lock();
	SYS_UnlockReg();

	WDT->CTL &= ~(WDT_CTL_WDTEN_Msk);
	// WDT->CTL &= ~(WDT_CTL_INTEN_Msk |  | WDT_CTL_IF_Msk );

	SYS_LockReg();
	irq_unlock(key);

	return 0;
}

static int wdt_m48x_install_timeout(const struct device *dev,
				    const struct wdt_timeout_cfg *cfg)
{
	struct wdt_m48x_dev_data *data = dev->data;

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		LOG_ERR("Upper limit timeout out of range");
		return -EINVAL;
	}

	data->flags = cfg->flags;

	uint32_t u32EnableReset = 0;

	/* Set mode of watchdog and callback */
	switch (cfg->flags) {
	case WDT_FLAG_RESET_SOC:
		LOG_DBG("Configuring reset SOC mode");
		u32EnableReset = 1;
		break;

	case WDT_FLAG_RESET_NONE:
		LOG_DBG("Configuring non-reset mode");
		u32EnableReset = 0;
		break;

	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -EINVAL;
	}

	#define u32TimeoutInterval WDT_TIMEOUT_2POW18	// ~ 26.2 sec period
	// #define u32TimeoutInterval WDT_TIMEOUT_2POW4
	#define u32ResetDelay WDT_RESET_DELAY_18CLK
	#define u32EnableWakeup 1
	//WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, true, true);

	unsigned int key = irq_lock();
	SYS_UnlockReg();	// Is this needed?
    WDT->CTL = u32TimeoutInterval | WDT_CTL_WDTEN_Msk |
               (u32EnableReset << WDT_CTL_RSTEN_Pos) |
               (u32EnableWakeup << WDT_CTL_WKEN_Pos);

	/* Only enable IRQ if a callback was provided */
	data->cb = cfg->callback;
	if (data->cb) {
		// WDT_REGS->INTENSET.reg = WDT_INTENSET_EW;
		WDT->CTL |= WDT_CTL_INTEN_Msk;
	} else {
		WDT->CTL &= ~(WDT_CTL_INTEN_Msk);
		// WDT_REGS->INTENCLR.reg = WDT_INTENCLR_EW;
		// WDT_REGS->INTFLAG.reg = WDT_INTFLAG_EW;
	}
	SYS_LockReg();
	irq_unlock(key);

	return 0;
}

static int wdt_m48x_feed(const struct device *dev, int channel_id)
{
	// struct wdt_m48x_dev_data *data = dev->data;
	LOG_DBG("feed(%d)", channel_id);
	bool inIsr = k_is_in_isr();
	if (!inIsr) {
		unsigned int key = irq_lock();
		SYS_UnlockReg();
		WDT_RESET_COUNTER();
		SYS_LockReg();
		irq_unlock(key);
	} else {
		SYS_UnlockReg();
		WDT_RESET_COUNTER();
		SYS_LockReg();
	}

	return 0;
}

static const struct wdt_driver_api wdt_m48x_api = {
	.setup = wdt_m48x_setup,
	.disable = wdt_m48x_disable,
	.install_timeout = wdt_m48x_install_timeout,
	.feed = wdt_m48x_feed,
};

static int wdt_m48x_init(const struct device *dev)
{
	SYS_UnlockReg();

	#if 0
	/*
     * There is a M480 issue that wakeup interrupt controller (WIC) will stay
     * in a wrong state if the system is reset by WDT from NPD/FWPD/LLPD mode.
     * So it is recommended to disable WDT reset function before enter power
     * down mode and re-enable after wakeup.
     *
     * If keep WDT reset function during power down is required, force M480 to
     * execute a deep power down / wake up cycle before enable interrupt can
     * recover the WIC from the wrong state if the system is reset by WDT from
     * power down mode.
     *
     * Please check the section "WDT reset under Power-down mode" in M480 errata
     * for the detailed description of this issue.
     */
    if (SYS_IS_WDT_RST()) {
        /* Set up DPD power down mode */
        CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_DPD);

        CLK_SET_WKTMR_INTERVAL(CLK_PMUCTL_WKTMRIS_128);
        CLK_ENABLE_WKTMR();

        CLK_PowerDown();
    }
	#endif
	CLK_EnableModuleClock(WDT_MODULE);
	CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

	//#define u32TimeoutInterval WDT_TIMEOUT_2POW14	// ~ 1.6375 sec period
	// #define u32TimeoutInterval WDT_TIMEOUT_2POW16	// ~ 6.55 sec period

	// Set maximum period
	#define u32TimeoutInterval WDT_TIMEOUT_2POW18	// ~ 26.2 sec period
	// #define u32TimeoutInterval WDT_TIMEOUT_2POW4
	#define u32ResetDelay WDT_RESET_DELAY_18CLK
	#define u32EnableReset 0
	#define u32EnableWakeup 1
	//WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, true, true);

	WDT->ALTCTL = u32ResetDelay;
    WDT->CTL = u32TimeoutInterval /*| WDT_CTL_WDTEN_Msk*/ |
               (u32EnableReset << WDT_CTL_RSTEN_Pos) |
               (u32EnableWakeup << WDT_CTL_WKEN_Pos);

	// /* Enable WDT NVIC */

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority), wdt_m48x_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	/* Enable WDT interrupt function */
	NVIC_EnableIRQ(WDT_IRQn);
    WDT_EnableInt();

	/* Lock protected registers */
    SYS_LockReg();

	#ifndef CONFIG_WDT_DISABLE_AT_BOOT
		wdt_m48x_enable(dev);
	#endif

	return 0;
}

static struct wdt_m48x_dev_data wdt_m48x_data;

DEVICE_DT_INST_DEFINE(0, wdt_m48x_init, NULL,
		    &wdt_m48x_data, NULL, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_m48x_api);
