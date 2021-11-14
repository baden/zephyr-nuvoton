/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Linumiz
 * Author: Saravanan Sekar <saravanan@linumiz.com>
 */

 // look zephyr/drivers/serial/usart_sam.c for inspiration

#include <drivers/uart.h>
#include <NuMicro.h>
#include <string.h>

#define DT_DRV_COMPAT nuvoton_m48x_uart

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(uart_numicro);

/* Device data structure */
#define DEV_CFG(dev)						\
	((const struct uart_numicro_config * const)(dev)->config)

#define DRV_DATA(dev)						\
	((struct uart_numicro_data * const)(dev)->data)

#define UART_STRUCT(dev)					\
	((UART_T *)(DEV_CFG(dev))->devcfg.base)

struct uart_numicro_config {
	struct uart_device_config devcfg;
	uint32_t idx;				// TODO: Not ideal solution
};

struct uart_numicro_data {
	const struct device *clock;
	struct uart_config ucfg;
};

// TODO: Try to make is by #define
static inline uint32_t uart_rst(const uint32_t periph_id)
{
	switch(periph_id) {
		case 0: return UART0_RST;
		case 1: return UART1_RST;
		case 2: return UART2_RST;
		case 3: return UART3_RST;
		case 4: return UART4_RST;
		case 5: return UART5_RST;
		case 6: return UART6_RST;
		case 7: return UART7_RST;
	}
	return UART0_RST;
}

static inline uint32_t uart_module(const uint32_t periph_id)
{
	switch(periph_id) {
		case 0: return UART0_MODULE;
		case 1: return UART1_MODULE;
		case 2: return UART2_MODULE;
		case 3: return UART3_MODULE;
		case 4: return UART4_MODULE;
		case 5: return UART5_MODULE;
		case 6: return UART6_MODULE;
		case 7: return UART7_MODULE;
	}
	return UART0_MODULE;
}


static int uart_numicro_poll_in(const struct device *dev, unsigned char *c)
{
	uint32_t count;

	count = UART_Read(UART_STRUCT(dev), c, 1);
	if (!count) {
		return -1;
	}

	return 0;
}

static void uart_numicro_poll_out(const struct device *dev, unsigned char c)
{
	UART_Write(UART_STRUCT(dev), &c, 1);
}

static int uart_numicro_err_check(const struct device *dev)
{
	return 0;
}

static inline int32_t uart_numicro_convert_stopbit(enum uart_config_stop_bits sb)
{
	switch (sb) {
	case UART_CFG_STOP_BITS_1:
		return UART_STOP_BIT_1;
	case UART_CFG_STOP_BITS_1_5:
		return UART_STOP_BIT_1_5;
	case UART_CFG_STOP_BITS_2:
		return UART_STOP_BIT_2;
	default:
		return -ENOTSUP;
	}
};

static inline int32_t uart_numicro_convert_datalen(enum uart_config_data_bits db)
{
	switch (db) {
	case UART_CFG_DATA_BITS_5:
		return UART_WORD_LEN_5;
	case UART_CFG_DATA_BITS_6:
		return UART_WORD_LEN_6;
	case UART_CFG_DATA_BITS_7:
		return UART_WORD_LEN_7;
	case UART_CFG_DATA_BITS_8:
		return UART_WORD_LEN_8;
	default:
		return -ENOTSUP;
	}
}

static inline uint32_t uart_numicro_convert_parity(enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return UART_PARITY_ODD;
	case UART_CFG_PARITY_EVEN:
		return UART_PARITY_EVEN;
	case UART_CFG_PARITY_MARK:
		return UART_PARITY_MARK;
	case UART_CFG_PARITY_SPACE:
		return UART_PARITY_SPACE;
	case UART_CFG_PARITY_NONE:
	default:
		return UART_PARITY_NONE;
	}
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_numicro_configure(const struct device *dev,
				  const struct uart_config *cfg)
{
	struct uart_numicro_data *ddata = DRV_DATA(dev);
	int32_t databits, stopbits;
	uint32_t parity;

	databits = uart_numicro_convert_datalen(cfg->data_bits);
	if (databits < 0) {
		return databits;
	}

	stopbits = uart_numicro_convert_stopbit(cfg->stop_bits);
	if (stopbits < 0) {
		return stopbits;
	}

	if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_NONE) {
		UART_DisableFlowCtrl(UART_STRUCT(dev));
	} else if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
		UART_EnableFlowCtrl(UART_STRUCT(dev));
	} else {
		return -ENOTSUP;
	}

	parity = uart_numicro_convert_parity(cfg->parity);

	UART_SetLineConfig(UART_STRUCT(dev), cfg->baudrate, databits,
			   parity, stopbits);

	memcpy(&ddata->ucfg, cfg, sizeof(*cfg));

	return 0;
}

static int uart_numicro_config_get(const struct device *dev,
				   struct uart_config *cfg)
{
	struct uart_numicro_data *ddata = DRV_DATA(dev);

	memcpy(cfg, &ddata->ucfg, sizeof(*cfg));

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

// #define SYS_REG(n, v) DT_CAT(SYS->, n) &= v


// SYS_GPC_MFPH_PC11MFP_UART0_RXD

#define _CAT3(_n1, _n2, _n3) _n1 ## _n2 ## _n3
#define CAT3(_n1, _n2, _n3) _CAT3(_n1, _n2, _n3)

#define _CAT6(_n1, _n2, _n3, _n4, _n5, _n6) _n1 ## _n2 ## _n3 ## _n4 ## _n5 ## _n6
#define CAT6(_n1, _n2, _n3, _n4, _n5, _n6) _CAT6(_n1, _n2, _n3, _n4, _n5, _n6)

#define NODENAME(id) uart ## id

#define _GPA_MFPL SYS->GPA_MFPL
#define _GPB_MFPL SYS->GPB_MFPL
#define _GPC_MFPL SYS->GPC_MFPL
#define _GPD_MFPL SYS->GPD_MFPL
#define _GPE_MFPL SYS->GPE_MFPL
#define _GPF_MFPL SYS->GPF_MFPL
#define _GPH_MFPL SYS->GPH_MFPL

#define _GPA_MFPH SYS->GPA_MFPH
#define _GPB_MFPH SYS->GPB_MFPH
#define _GPC_MFPH SYS->GPC_MFPH
#define _GPD_MFPH SYS->GPD_MFPH
#define _GPE_MFPH SYS->GPE_MFPH
#define _GPF_MFPH SYS->GPF_MFPH
#define _GPH_MFPH SYS->GPH_MFPH

#define _MFP0	_MFPL
#define _MFP1	_MFPL
#define _MFP2	_MFPL
#define _MFP3	_MFPL
#define _MFP4	_MFPL
#define _MFP5	_MFPL
#define _MFP6	_MFPL
#define _MFP7	_MFPL
#define _MFP8	_MFPH
#define _MFP9	_MFPH
#define _MFP10	_MFPH
#define _MFP11	_MFPH
#define _MFP12	_MFPH
#define _MFP13	_MFPH
#define _MFP14	_MFPH
#define _MFP15	_MFPH

#define _PORT_LETTER(_pid, _port)	\
	DT_STRING_TOKEN(DT_NODELABEL(NODENAME(_pid)), _port)
#define _PIN_NUMBER(_pid, _pin)	\
	DT_STRING_TOKEN(DT_NODELABEL(NODENAME(_pid)), _pin)

#define SYS_MFP(_pid, _port, _pin)				\
	CAT3(										\
		_GP,									\
		_PORT_LETTER(_pid, _port),				\
		_CONCAT(_MFP, _PIN_NUMBER(_pid, _pin))	\
	)

//SYS_GP
//DT_N_S_soc_S_serial_40071000_P_rx_port_STRING_TOKEN
// _MFPH_PDT_N_S_soc_S_serial_40071000_P_rx_port_STRING_TOKENDT_N_S_soc_S_serial_40071000_P_rx_pin_STRING_TOKENMFP_Msk
// GPC_MFPH_PC11MFP
#define RX_MASK(_pid) 	CAT6(									\
	SYS_GP,														\
	_PORT_LETTER(_pid, rx_port),								\
	_CONCAT(_CONCAT(_MFP, _PIN_NUMBER(_pid, rx_pin)), _P),		\
	_PORT_LETTER(_pid, rx_port),								\
	_PIN_NUMBER(_pid, rx_pin),									\
	MFP_Msk)
#define RX_PIN(_pid) 	CAT6(									\
	SYS_GP,														\
	_PORT_LETTER(_pid, rx_port),								\
	_CONCAT(_CONCAT(_MFP, _PIN_NUMBER(_pid, rx_pin)), _P),		\
	_PORT_LETTER(_pid, rx_port),								\
	_PIN_NUMBER(_pid, rx_pin),									\
	CAT3(MFP_UART,_pid,_RXD))
#define TX_MASK(_pid) 	CAT6(									\
	SYS_GP,														\
	_PORT_LETTER(_pid, tx_port),								\
	_CONCAT(_CONCAT(_MFP, _PIN_NUMBER(_pid, tx_pin)), _P),		\
	_PORT_LETTER(_pid, tx_port),								\
	_PIN_NUMBER(_pid, tx_pin),									\
	MFP_Msk)
#define TX_PIN(_pid) 	CAT6(									\
	SYS_GP,														\
	_PORT_LETTER(_pid, tx_port),								\
	_CONCAT(_CONCAT(_MFP, _PIN_NUMBER(_pid, tx_pin)), _P),		\
	_PORT_LETTER(_pid, tx_port),								\
	_PIN_NUMBER(_pid, tx_pin),									\
	CAT3(MFP_UART,_pid,_TXD))

#define MuxUartPort(_pid)	\
	SYS_MFP(_pid, tx_port, tx_pin) &= ~( TX_MASK(_pid) );	\
	SYS_MFP(_pid, tx_port, tx_pin) |=  ( TX_PIN(_pid)  );	\
	SYS_MFP(_pid, rx_port, rx_pin) &= ~( RX_MASK(_pid) );	\
	SYS_MFP(_pid, rx_port, rx_pin) |=  ( RX_PIN(_pid)  );


// _GPDT_N_NODELABEL_uartuart3_P_tx_port_STRING_TOKEN_MFPDT_N_NODELABEL_uartuart3_P_tx_pin_STRING_TOKEN

static int uart_numicro_init(const struct device *dev)
{
	const struct uart_numicro_config *config = DEV_CFG(dev);
	struct uart_numicro_data *ddata = DRV_DATA(dev);

	LOG_INF("UART device init (%d, TX:%s, RX:%s)", config->idx, "TBD", "TBD");

	// const char *nl = DT_STRING_TOKEN(DT_NODELABEL(uart0), pins);
	// SYS->GPB_MFPH

	// Значение из строки:reg = <0x40073000 0x1000>;
	// LOG_INF("  -> (DEV_CFG(dev))->devcfg.base = %08X", (unsigned)((DEV_CFG(dev))->devcfg.base));

    //  TODO: Set pins!!!!
	/* Set pinctrl for UART0 RXD and TXD */

	// StartKit
	// SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
	// SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD |
	// 		  SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	LOG_INF("  * Init UART%d", config->idx);
	SYS_ResetModule(uart_rst(config->idx));
	SYS_UnlockReg();
	CLK_EnableModuleClock(uart_module(config->idx));	/* Enable UART module clock */

	// TODO: Temporrary solution!!!!
	switch(config->idx) {
		case 0: // UART0
			CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PLL, CLK_CLKDIV0_UART0(0));	/* Select UART0 clock source is PLL */
			MuxUartPort(0);
			break;
		case 1: // UART1
			CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_PLL, CLK_CLKDIV0_UART1(0));	/* Select UART1 clock source is PLL */
			MuxUartPort(1);
			break;
		case 2: // UART2
			CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_PLL, CLK_CLKDIV4_UART2(0));	/* Select UART1 clock source is PLL */
			MuxUartPort(2);
			break;
		case 3: // UART3
			// UART3: PC-001
			CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_PLL, CLK_CLKDIV4_UART3(0));	/* Select UART3 clock source is PLL */
   			MuxUartPort(3);
			break;
		case 4: // UART4
			// UART3: PC-001
			CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_PLL, CLK_CLKDIV4_UART4(0));	/* Select UART4 clock source is PLL */
			MuxUartPort(4);
			break;
		case 5: // UART5
			// UART5: PC-001
			CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_PLL, CLK_CLKDIV4_UART5(0));	/* Select UART5 clock source is PLL */
			MuxUartPort(5);
			break;
		// case 6: // UART6
		// 	// UART5: PC-001
		// 	CLK_SetModuleClock(UART6_MODULE, CLK_CLKSEL3_UART6SEL_PLL, CLK_CLKDIV4_UART6(0));	/* Select UART6 clock source is PLL */
		// 	// MuxUartPort(6);
		// 	break;
		// case 7: // UART7
		// 	// UART5: PC-001
		// 	CLK_SetModuleClock(UART7_MODULE, CLK_CLKSEL3_UART7SEL_PLL, CLK_CLKDIV4_UART7(0));	/* Select UART7 clock source is PLL */
		// 	// MuxUartPort(7);
		// 	break;

	}

	SYS_LockReg();

	UART_Open(UART_STRUCT(dev), ddata->ucfg.baudrate);

	return 0;
}

static const struct uart_driver_api uart_numicro_driver_api = {
	.poll_in          = uart_numicro_poll_in,
	.poll_out         = uart_numicro_poll_out,
	.err_check        = uart_numicro_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure        = uart_numicro_configure,
	.config_get       = uart_numicro_config_get,
#endif
};


// #define _UART_RST(n) UART#n#_RST
// #define UART_RST(n) _UART_RST(n)

#define NUMICRO_INIT(index)													\
																			\
	static const struct uart_numicro_config uart_numicro_cfg_##index = {	\
		.devcfg = {															\
			.base = (uint8_t *)DT_INST_REG_ADDR(index),						\
		},																	\
																			\
		.idx = DT_INST_PROP(index, peripheral_id),							\
	};																		\
																			\
	static struct uart_numicro_data uart_numicro_data_##index = {			\
		.ucfg = {															\
			.baudrate = DT_INST_PROP(index, current_speed),					\
		},																	\
	};																		\
																			\
	DEVICE_DT_INST_DEFINE(index,											\
			    &uart_numicro_init,											\
			    NULL,														\
			    &uart_numicro_data_##index,									\
			    &uart_numicro_cfg_##index,									\
			    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
			    &uart_numicro_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NUMICRO_INIT)

//