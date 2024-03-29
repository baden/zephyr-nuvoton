/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Linumiz
 * Author: Saravanan Sekar <saravanan@linumiz.com>
 */

 // look zephyr/drivers/serial/usart_sam.c for inspiration

#include <zephyr/drivers/uart.h>
#include <NuMicro.h>
#include <string.h>

#define DT_DRV_COMPAT nuvoton_m48x_uart

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_numicro);

typedef void (*uart_pinmux_config_func_t)(const struct device *dev);

struct uart_numicro_config {
	uint8_t* base;			// TODO: Remove me. For debug only
	uint32_t idx;			// TODO: Romove me after refactoring.
	UART_T* uart;
	uint32_t id_rst;
	uint32_t id_clk;
	uint32_t clk_src;
	uint32_t clk_div;

	// TODO: Bad solution. Use pinmux mechanic!
	uart_pinmux_config_func_t	pinmux_func;
	uint32_t* rx_mfp;
	uint32_t rx_mask;
	uint32_t rx_bits;

	#if 1 //def CONFIG_UART_INTERRUPT_DRIVEN
		uart_irq_config_func_t	irq_config_func;
	#endif
};

// struct uart_numicro_config {
// 	struct uart_device_config devcfg;
// 	uint32_t idx;				// TODO: Not ideal solution
// 	#if 1 //def CONFIG_UART_INTERRUPT_DRIVEN
// 		uart_irq_config_func_t	irq_config_func;
// 	#endif
// };

struct uart_numicro_data {
	const struct device *clock;
	struct uart_config ucfg;
	uart_irq_callback_user_data_t irq_cb;	/* Interrupt Callback */
	void *irq_cb_data;			/* Interrupt Callback Arg */
};

/*
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
*/

static int uart_numicro_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_numicro_config *config = dev->config;
	uint32_t count;

	count = UART_Read(config->uart, c, 1);
	if (!count) {
		return -1;
	}

	return 0;

	// // uint32_t count;
	// UART_T* uart = UART_STRUCT(dev);
	//
	// if(uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) return -EBUSY;
	// // Maybe need use UART_READ(uart) macro?
	// *c = (unsigned char)uart->DAT;
	//
	// return 0;
}

static void uart_numicro_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_numicro_config *config = dev->config;

	UART_Write(config->uart, &c, 1);

	// // UART_Write(UART_STRUCT(dev), &c, 1);
	// UART_T* uart = UART_STRUCT(dev);
	// /* Wait for transmitter to be ready */
	// // Maybe need use UART_GET_TX_FULL(uart) macro
	// while(uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {
	// }
	// // Maybe need use UART_WRITE(uart, c) macro
	// uart->DAT = c;
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
	const struct uart_numicro_config *config = dev->config;
	struct uart_numicro_data *ddata = dev->data;
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
		UART_DisableFlowCtrl(config->uart);
	} else if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
		UART_EnableFlowCtrl(config->uart);
	} else {
		return -ENOTSUP;
	}

	parity = uart_numicro_convert_parity(cfg->parity);

	UART_SetLineConfig(config->uart, cfg->baudrate, databits, parity,
			   stopbits);

	memcpy(&ddata->ucfg, cfg, sizeof(*cfg));

	return 0;
}

static int uart_numicro_config_get(const struct device *dev,
				   struct uart_config *cfg)
{
	struct uart_numicro_data *ddata = dev->data;

	memcpy(cfg, &ddata->ucfg, sizeof(*cfg));

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */


#if 1 // def CONFIG_UART_INTERRUPT_DRIVEN

/*static*/ int usart_m48x_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int size)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;


	/* Wait for transmitter to be ready. */
	while(UART_IS_TX_FULL(uart));  /* Wait Tx is not full to transmit data */
	// while ((usart->US_CSR & US_CSR_TXRDY) == 0) {}

	// TODO: Why here is just one byte send?
	// usart->US_THR = *tx_data;
	UART_WRITE(uart, *tx_data);
	return 1;
}

/*static*/ int usart_m48x_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	int bytes_read;

	bytes_read = 0;

	// TODO: Wich i must used of:?
	// #define UART_GET_RX_EMPTY(uart)    ((uart)->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)
	// #define UART_IS_RX_READY(uart)    (((uart)->INTSTS & UART_INTSTS_RDAIF_Msk)>>UART_INTSTS_RDAIF_Pos)

	while (bytes_read < size) {
		// if( UART_IS_RX_READY(uart) ) {
		if( UART_GET_RX_EMPTY(uart) == 0 ) {
		// if (usart->US_CSR & US_CSR_RXRDY) {
			rx_data[bytes_read] = UART_READ(uart);
			bytes_read++;
		} else {
			break;
		}
	}

	return bytes_read;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_m48x_irq_tx_enable(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	UART_ENABLE_INT(uart, UART_INTEN_THREIEN_Msk);
}
#endif

/*static*/ void usart_m48x_irq_tx_disable(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	UART_DISABLE_INT(uart, UART_INTEN_THREIEN_Msk);
}

// Maybe need use UART_GET_TX_FULL(uart) macro
// while(uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {
// }

// #define UART_GET_TX_EMPTY(uart)    ((uart)->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk)
// #define UART_GET_TX_FULL(uart)    ((uart)->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int usart_m48x_irq_tx_ready(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	// UART_GET_TX_FULL()=0   	Tx FIFO is not full.
	// UART_GET_TX_FULL()=1		Tx FIFO is full.
	return !UART_GET_TX_FULL(uart);
	// return (usart->US_CSR & US_CSR_TXRDY);
}
#endif

/*static*/ void usart_m48x_irq_rx_enable(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	UART_ENABLE_INT(uart, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

/*static*/ void usart_m48x_irq_rx_disable(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	UART_DISABLE_INT(uart, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int usart_m48x_irq_tx_complete(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	// 0   Tx FIFO is not empty
	// >=1 Tx FIFO is empty
	return UART_GET_TX_EMPTY(uart);
}
#endif

/*static*/ int usart_m48x_irq_rx_ready(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	// 0   Rx FIFO is not empty
	// >=1 Rx FIFO is empty
	return (UART_GET_RX_EMPTY(uart) == 0);
	// return (usart->US_CSR & US_CSR_RXRDY);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_m48x_irq_err_enable(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;

	// TODO: other errrors?
	UART_ENABLE_INT(uart, (UART_INTEN_RXTOIEN_Msk));
	// usart->US_IER = US_IER_OVRE | US_IER_FRAME | US_IER_PARE;
}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_m48x_irq_err_disable(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;

	// TODO: Other errors?
	UART_DISABLE_INT(uart, (UART_INTEN_RXTOIEN_Msk));
	// usart->US_IDR = US_IDR_OVRE | US_IDR_FRAME | US_IDR_PARE;
}
#endif

// TODO: Do we need check INTEN bits here?
// If i clearly understand, RDAINT flag is set, when RDAIEN and RDAIF are both set to 1
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int usart_m48x_irq_is_pending(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	uint32_t u32IntSts = uart->INTSTS;

	return (
		(u32IntSts & UART_INTSTS_RDAINT_Msk) ||
		(u32IntSts & UART_INTSTS_THREINT_Msk)
	);

	// return (usart->US_IMR & (US_IMR_TXRDY | US_IMR_RXRDY)) &
	// 	(usart->US_CSR & (US_CSR_TXRDY | US_CSR_RXRDY));
}
#endif

/*static*/ int usart_m48x_irq_update(const struct device *dev)
{
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	// TODO: Move this to post of usart_m48x_isr after resroting usart_m48x_irq_callback_
	// ARG_UNUSED(dev);
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	//
	// if(uart->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    // {
        uart->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    // }

	return 1;
}

#if 1 //def CONFIG_UART_INTERRUPT_DRIVEN
/*static */void usart_m48x_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
//	/*volatile*/ UART_T* uart = UART_STRUCT(dev);
	// const struct uart_numicro_config *config = dev->config;
	// UART_T* uart = config->uart;
	struct uart_numicro_data *dev_data = dev->data;
	LOG_DBG("Set isr callback %p and data %p for %s", cb, cb_data, dev->name);

	dev_data->irq_cb = cb;
	dev_data->irq_cb_data = cb_data;

}
#endif

#if 1 //def CONFIG_UART_INTERRUPT_DRIVEN
static void usart_m48x_isr(const struct device *dev)
{
	// /*volatile*/ UART_T* uart = UART_STRUCT(dev);
	const struct uart_numicro_config *config = dev->config;
	UART_T* uart = config->uart;
	struct uart_numicro_data *dev_data = dev->data;

	if (dev_data->irq_cb) {
		dev_data->irq_cb(dev, dev_data->irq_cb_data);
	}

	if(uart->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
		PE12 ^= 1;
        uart->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    }

}
#endif

#endif

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

#if 0
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

#endif
// _GPDT_N_NODELABEL_uartuart3_P_tx_port_STRING_TOKEN_MFPDT_N_NODELABEL_uartuart3_P_tx_pin_STRING_TOKEN

#define FIND_A_BUG() //{GPIO_SetMode(PE, BIT(12), GPIO_MODE_OUTPUT); PE12 = 0;}

static int uart_numicro_init(const struct device *dev)
{
	const struct uart_numicro_config *config = dev->config;
	struct uart_numicro_data *ddata = dev->data;

	FIND_A_BUG();

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
	// LOG_INF("  * Init UART%d", config->idx);
	// SYS_ResetModule(uart_rst(config->idx));
	SYS_ResetModule(config->id_rst);
	unsigned int key = irq_lock();
	SYS_UnlockReg();
	// CLK_EnableModuleClock(uart_module(config->idx));	/* Enable UART module clock */
	CLK_EnableModuleClock(config->id_clk);

	/* Connect pins to the peripheral */
	/* TODO: Make something like this */
	// soc_gpio_configure(&cfg->pin_rx);
	// soc_gpio_configure(&cfg->pin_tx);
	CLK_SetModuleClock(config->id_clk, config->clk_src, config->clk_div);

	#if 0
	// TODO: Temporrary solution!!!!
	switch(config->idx) {
		case 0: // UART0
			// // CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PLL, CLK_CLKDIV0_UART0(0));	/* Select UART0 clock source is PLL */
			// CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));	/* Select UART clock source from HXT */
			MuxUartPort(0);
			break;
		case 1: // UART1
			// // CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_PLL, CLK_CLKDIV0_UART1(0));	// Select UART1 clock source is PLL
			// CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));	/* Select UART clock source from HXT */
			MuxUartPort(1);
			break;
		case 2: // UART2
			// // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_PLL, CLK_CLKDIV4_UART2(0));	// Select UART1 clock source is PLL
			// CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HXT, CLK_CLKDIV4_UART2(1));	/* Select UART clock source from HXT */
			MuxUartPort(2);
			break;
		case 3: // UART3
			// UART3: PC-001
			// // CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_PLL, CLK_CLKDIV4_UART3(0));	// Select UART3 clock source is PLL
			// CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_HXT, CLK_CLKDIV4_UART3(1));	/* Select UART clock source from HXT */
   			MuxUartPort(3);
			break;
		case 4: // UART4
			// UART4: PC-001
			// // CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_PLL, CLK_CLKDIV4_UART4(0));	// Select UART4 clock source is PLL
			// CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HXT, CLK_CLKDIV4_UART4(1));	/* Select UART clock source from HXT */
			MuxUartPort(4);
			break;
		case 5: // UART5
			// UART5: PC-001
			// // CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_PLL, CLK_CLKDIV4_UART5(0));	// Select UART5 clock source is PLL
			// CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_HXT, CLK_CLKDIV4_UART5(1));	/* Select UART clock source from HXT */
			MuxUartPort(5);
			break;

		// case 6: // UART6
		// 	// UART5: PC-001
		// 	CLK_SetModuleClock(UART6_MODULE, CLK_CLKSEL3_UART6SEL_PLL, CLK_CLKDIV4_UART6(0));	// Select UART6 clock source is PLL
		// 	// MuxUartPort(6);
		// 	break;
		// case 7: // UART7
		// 	// UART5: PC-001
		// 	CLK_SetModuleClock(UART7_MODULE, CLK_CLKSEL3_UART7SEL_PLL, CLK_CLKDIV4_UART7(0));	// Select UART7 clock source is PLL
		// 	// MuxUartPort(7);
		// 	break;

	}
	#endif
	config->pinmux_func(dev);

	SYS_LockReg();
	irq_unlock(key);

	#if 1 //def CONFIG_UART_INTERRUPT_DRIVEN
		config->irq_config_func(dev);
	#endif

	UART_Open(config->uart, ddata->ucfg.baudrate);

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
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_m48x_fifo_fill,
	.fifo_read = usart_m48x_fifo_read,
	.irq_tx_enable = usart_m48x_irq_tx_enable,
	.irq_tx_disable = usart_m48x_irq_tx_disable,
	.irq_tx_ready = usart_m48x_irq_tx_ready,
	.irq_rx_enable = usart_m48x_irq_rx_enable,
	.irq_rx_disable = usart_m48x_irq_rx_disable,
	.irq_tx_complete = usart_m48x_irq_tx_complete,
	.irq_rx_ready = usart_m48x_irq_rx_ready,
	.irq_err_enable = usart_m48x_irq_err_enable,
	.irq_err_disable = usart_m48x_irq_err_disable,
	.irq_is_pending = usart_m48x_irq_is_pending,
	.irq_update = usart_m48x_irq_update,
	.irq_callback_set = usart_m48x_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

// ISR_DIRECT_DECLARE(UART3_IRQHandler)
// {
// 	modem_iface_uart_isr(uart_dev, NULL);
// 	return 0;
// }

/*
ISR_DIRECT_DECLARE(UART0_IRQHandler)
{
	usart_m48x_isr(DEVICE_DT_GET(DT_NODELABEL(uart0)));
	// usart_m48x_isr(DEVICE_DT_GET(DT_ALIAS(uart0)));
}

static void usart0_m48x_irq_config_func(const struct device *port)
{
	IRQ_DIRECT_CONNECT(UART0_IRQn, 0, UART0_IRQHandler, IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
	irq_enable(UART0_IRQn);
	irq_enable(DT_INST_IRQN(0));
}
*/

/*
static void usart0_m48x_irq_config_func(const struct device *port)
{
	IRQ_CONNECT(DT_INST_IRQN(0),
			0, //DT_INST_IRQ(0, priority),
			usart_m48x_isr,
			DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}
*/



// #define _UART_RST(n) UART#n#_RST
// #define UART_RST(n) _UART_RST(n)

#if 1 // def CONFIG_UART_INTERRUPT_DRIVEN
#define USART_M48X_CONFIG_FUNC(n)											\
	static void usart##n##_m48x_irq_config_func(const struct device *port)	\
	{																		\
		IRQ_CONNECT(DT_INST_IRQN(n),										\
			    DT_INST_IRQ(n, priority),									\
			    usart_m48x_isr,												\
			    DEVICE_DT_INST_GET(n), 0);									\
		irq_enable(DT_INST_IRQN(n));										\
	}

#define USART_M48X_IRQ_CFG_FUNC_INIT(n)										\
	.irq_config_func = usart##n##_m48x_irq_config_func

#define USART_M48X_INIT_CFG(n)												\
	USART_M48X_DECLARE_CFG(n, USART_M48X_IRQ_CFG_FUNC_INIT(n))
#else
#define USART_M48X_CONFIG_FUNC(n)
#define USART_M48X_IRQ_CFG_FUNC_INIT
#define USART_M48X_INIT_CFG(n)												\
	USART_M48X_DECLARE_CFG(n, USART_M48X_IRQ_CFG_FUNC_INIT)
#endif

#define __STR1(x) #x
#define __STR(x) __STR1([ x ])

#define __RST1(id) UART##id##_RST
#define __RST(id) __RST1(id)
#define __MODULE1(id) UART##id##_MODULE
#define __MODULE(id) __MODULE1(id)

/*
__WARN(__STR( Label: DT_INST_PROP(index, label) )) \
__WARN(__STR( Index: index )) \
__WARN(__STR( Id1: DT_INST_PROP(index, peripheral_id) )) \
__WARN(__STR( Id2: DT_INST_ENUM_IDX(index, peripheral_id) )) \
__WARN(__STR( DT_INST_REG_ADDR(index) )) \
__WARN(__STR( RST1: UART##index##_RST )) \
__WARN(__STR( RST2: __RST( DT_INST_PROP(index, peripheral_id) ) )) \
__WARN(__STR( MODULE1: UART##index##_MODULE )) \
__WARN(__STR( MODULE2: __MODULE( DT_INST_PROP(index, peripheral_id) ) )) \
__WARN(__STR( CLK-SRC: DT_INST_PROP(index, clk_src) )) \
__WARN(__STR( CLK-DIV: DT_INST_PROP(index, clk_div) )) \
*/

// Pin MUX
// SYS->GPB_MFPH
// AND: SYS_GPB_MFPH_PB12MFP_Msk
// OR:  SYS_GPB_MFPH_PB12MFP_UART0_RXD

// rx-port
// rx-pin

#define RX_PORT(idx) DT_STRING_TOKEN(DT_DRV_INST(idx), rx_port)
#define RX_PIN(idx) DT_STRING_TOKEN(DT_DRV_INST(idx), rx_pin)
#define TX_PORT(idx) DT_STRING_TOKEN(DT_DRV_INST(idx), tx_port)
#define TX_PIN(idx) DT_STRING_TOKEN(DT_DRV_INST(idx), tx_pin)
#define ID(idx) DT_INST_PROP(idx, peripheral_id)

#define SYS2_MFP(port, pin)				\
	CAT3(										\
		_GP,									\
		port,				\
		_CONCAT(_MFP, pin)	\
	)

#define RX2_MASK(idx) 	CAT6(									\
	SYS_GP,														\
	RX_PORT(idx),												\
	_CONCAT(_CONCAT(_MFP, RX_PIN(idx)), _P),					\
	RX_PORT(idx),												\
	RX_PIN(idx),												\
	MFP_Msk)
#define RX2_PIN(idx) 	CAT6(									\
	SYS_GP,														\
	RX_PORT(idx),												\
	_CONCAT(_CONCAT(_MFP, RX_PIN(idx)), _P),					\
	RX_PORT(idx),												\
	RX_PIN(idx),												\
	CAT3(MFP_UART,ID(idx),_RXD))
#define TX2_MASK(idx) 	CAT6(									\
	SYS_GP,														\
	TX_PORT(idx),												\
	_CONCAT(_CONCAT(_MFP, TX_PIN(idx)), _P),					\
	TX_PORT(idx),												\
	TX_PIN(idx),												\
	MFP_Msk)
#define TX2_PIN(idx) 	CAT6(									\
	SYS_GP,														\
	TX_PORT(idx),												\
	_CONCAT(_CONCAT(_MFP, TX_PIN(idx)), _P),					\
	TX_PORT(idx),												\
	TX_PIN(idx),												\
	CAT3(MFP_UART,ID(idx),_TXD))

#define USART_M48X_PINMUX_FUNC(n)											\
	static void usart##n##_m48x_pinmux_config_func(const struct device *port)	\
	{																		\
		SYS2_MFP(RX_PORT(n), RX_PIN(n)) &= ~(RX2_MASK(n));	 				\
		SYS2_MFP(TX_PORT(n), TX_PIN(n)) &= ~(TX2_MASK(n));	 				\
		SYS2_MFP(RX_PORT(n), RX_PIN(n)) |= RX2_PIN(n);	 					\
		SYS2_MFP(TX_PORT(n), TX_PIN(n)) |= TX2_PIN(n);	 					\
	}



#define GPIO_PORT_PIN_MASK_FROM_NGPIOS(ngpios)			\
	((gpio_port_pins_t)(((uint64_t)1 << (ngpios)) - 1U))


#define NUMICRO_INIT(index)													\
	USART_M48X_CONFIG_FUNC(index)											\
	USART_M48X_PINMUX_FUNC(index)											\
																			\
	static const struct uart_numicro_config uart_numicro_cfg_##index = {	\
		.base = (uint8_t *)DT_INST_REG_ADDR(index),							\
		.idx = DT_INST_PROP(index, peripheral_id),							\
																			\
		.uart = (UART_T *)DT_INST_REG_ADDR(index),							\
		.id_rst = __RST( DT_INST_PROP(index, peripheral_id) ),				\
		.id_clk = __MODULE( DT_INST_PROP(index, peripheral_id) ),			\
		.clk_src = DT_INST_PROP(index, clk_src),							\
		.clk_div = DT_INST_PROP(index, clk_div),							\
		.pinmux_func = usart##index##_m48x_pinmux_config_func,				\
		USART_M48X_IRQ_CFG_FUNC_INIT(index)									\
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
			    &uart_numicro_driver_api);									\

//				USART_M48X_INIT_CFG(index);

DT_INST_FOREACH_STATUS_OKAY(NUMICRO_INIT)
