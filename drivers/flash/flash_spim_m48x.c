/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// TODO: We need use flash driver insteed spi

#define DT_DRV_COMPAT nuvoton_m48x_spim_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)


#include <kernel.h>
#include <device.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <drivers/flash.h>
#include <soc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(flash_spim, CONFIG_FLASH_LOG_LEVEL);


struct flash_spim_dev_config {
	SPIM_T *controller;
	// esp_rom_spiflash_chip_t *chip;
};

struct flash_spim_dev_data {
	struct k_sem sem;
};

static const struct flash_parameters flash_spim_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

#define DEV_DATA(dev) ((struct flash_spim_dev_data *const)(dev)->data)
#define DEV_CFG(dev) ((const struct flash_spim_dev_config *const)(dev)->config)


static int flash_spim_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	// const struct flash_spim_dev_config *const cfg = DEV_CFG(dev);
	LOG_ERR("TODO: flash_spim_read");
	return 0;
}

static int flash_spim_write(const struct device *dev,
			     off_t address,
			     const void *buffer,
			     size_t length)
{
	// const struct flash_spim_dev_config *const cfg = DEV_CFG(dev);
	LOG_ERR("TODO: flash_spim_write");
	return 0;
}

static int flash_spim_erase(const struct device *dev, off_t start, size_t len)
{
	// uint32_t sector_size = DEV_CFG(dev)->chip->sector_size;
	// uint32_t chip_size = DEV_CFG(dev)->chip->chip_size;
	LOG_ERR("TODO: flash_spim_erase");
	return 0;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_spim_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

void flash_spim_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	*layout = &flash_spim_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *flash_spim_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_spim_parameters;
}



static int flash_spim_init(const struct device *dev)
{
	struct flash_spim_dev_data *const dev_data = DEV_DATA(dev);
	LOG_ERR("TODO: flash_spim_init");
	return 0;
}


static const struct flash_driver_api flash_spim_driver_api = {
	.read = flash_spim_read,
	.write = flash_spim_write,
	.erase = flash_spim_erase,
	.get_parameters = flash_spim_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_spim_page_layout,
#endif
};


static struct flash_spim_dev_data flash_spim_data;

static const struct flash_spim_dev_config flash_spim_config = {
	.controller = (SPIM_T *) DT_INST_REG_ADDR(0),
	// .chip = &esp_flashchip_info
};


DEVICE_DT_INST_DEFINE(0, flash_spim_init,
		      NULL,
		      &flash_spim_data, &flash_spim_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &flash_spim_driver_api);















#if 0	// Old

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spim_m48x);

#include <device.h>
#include <drivers/spi.h>
#include <soc.h>
#include "spi_context.h"


/* Device constant configuration parameters */
struct spim_config {
	SPIM_T *regs;
};

/* Device run time data */
struct spim_data {
	struct spi_context ctx;
};


static int transceive_sync(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	// const struct spi_qmspi_config *cfg = dev->config;
	// struct spi_qmspi_data *data = dev->data;
	// SPIM_T *regs = cfg->regs;
	LOG_ERR("TODO: transceive_sync");
	// return qmspi_transceive(dev, config, tx_bufs, rx_bufs);
	return 0;
}

static int release(const struct device *dev,
			 const struct spi_config *config)
{
	struct spim_data *data = dev->data;
	// const struct spi_qmspi_config *cfg = dev->config;
	// SPIM_T *regs = cfg->regs;
	LOG_ERR("TODO: release");
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static int spim_init(const struct device *dev)
{
	int err;
	const struct spim_config *cfg = dev->config;
	struct spim_data *data = dev->data;
	// SPIM_T *regs = cfg->regs;

	LOG_ERR("TODO: spim_init");

	uint8_t     idBuf[3];

    SYS_UnlockReg();    /* Unlock protected registers */

    CLK_EnableModuleClock(SPIM_MODULE); /* Enable SPIM module clock */
    SystemCoreClockUpdate();    /* Update System Core Clock   TODO: Realy needed? */

    //  Delfast PC-001 hardcoded!!!
    /* Init SPIM multi-function pins, MOSI(PE.2), MISO(PE.3), CLK(PE.4), SS(PE.5), D3(PE.6), and D2(PE.7) */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE7MFP_Msk | SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk |
                        SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE3MFP_Msk | SYS_GPE_MFPL_PE2MFP_Msk);
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE7MFP_SPIM_D2 | SYS_GPE_MFPL_PE6MFP_SPIM_D3 |
                        SYS_GPE_MFPL_PE5MFP_SPIM_SS | SYS_GPE_MFPL_PE4MFP_SPIM_CLK |
                        SYS_GPE_MFPL_PE3MFP_SPIM_MISO | SYS_GPE_MFPL_PE2MFP_SPIM_MOSI);

    PC->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PC->SLEWCTL = (PC->SLEWCTL & 0xFFFFF000) |
                  (0x1<<GPIO_SLEWCTL_HSREN0_Pos) | (0x1<<GPIO_SLEWCTL_HSREN1_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN2_Pos) | (0x1<<GPIO_SLEWCTL_HSREN3_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN4_Pos) | (0x1<<GPIO_SLEWCTL_HSREN5_Pos);


    SYS_LockReg();  /* Lock protected registers */

    // LOG_INF("+-------------------------------------------+\n");
    // LOG_INF("|    M480 SPIM DMA mode read/write sample   |\n");
    // LOG_INF("+-------------------------------------------+\n");

    // TODO: 192MHZ ONLY!
    SYS_UnlockReg();                   /* Unlock register lock protect */
    SPIM_SET_CLOCK_DIVIDER(2);        /* Set SPIM clock as HCLK divided by 4 */
    SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */
    SPIM_SET_DCNUM(8);                /* Set 8 dummy cycle. */
    // SYS_LockReg();  /* Lock protected registers */

    if (SPIM_InitFlash(1) != 0)        /* Initialized SPI flash */
    {
        LOG_ERR("SPIM flash initialize failed!\n");
        return -ENODEV;
    }

    SPIM_ReadJedecId(idBuf, sizeof (idBuf), 1);
    // SPIM get JEDEC ID=0xEF, 0x70, 0x18
    LOG_INF("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X", idBuf[0], idBuf[1], idBuf[2]);
    switch(idBuf[0]) {
        case MFGID_WINBOND:
            LOG_INF( "Manufacturer: WINBOND"
                    "  Capacity: %d MB",
                    1<<(idBuf[2] - 0x14)
            );
			SPIM_WinbondUnlock(1);
            break;
        case MFGID_SPANSION:
            LOG_INF("Manufacturer: SPANSION");
            break;
        case MFGID_EON:
            LOG_INF("Manufacturer: EON");
            break;
        case MFGID_ISSI:
            LOG_INF("Manufacturer: ISSI");
            break;
        case MFGID_MXIC:
            LOG_INF("Manufacturer: MXIC");
            break;
        default:
            LOG_INF("Manufacturer: -unknown-");
            break;
    }
    // idBuf[1] - Memory Type (ID15-ID8)
    // idBuf[2] - Capacity (ID7-ID0)

    // printf("printf is worked?\n");

    SPIM_Enable_4Bytes_Mode(0, 1);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spim_driver_api = {
	.transceive = transceive_sync,
	#ifdef CONFIG_SPI_ASYNC
		.transceive_async = transceive_async,
	#endif
	.release = release,
};


#if DT_NODE_HAS_STATUS(DT_INST(0, nuvoton_m48x_spim), okay)

static const struct spim_config spim_0_config = {
	.regs = (SPIM_T *)DT_INST_REG_ADDR(0),
};

static struct spim_data spim_0_dev_data = {
};

DEVICE_DT_INST_DEFINE(0,
		    &spim_init, NULL, &spim_0_dev_data,
		    &spim_0_config, POST_KERNEL,
		    CONFIG_SPI_INIT_PRIORITY, &spim_driver_api);


#endif
#endif // Old
