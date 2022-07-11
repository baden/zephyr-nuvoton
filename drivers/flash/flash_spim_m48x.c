/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// TODO: We need use flash driver insteed spi

#define DT_DRV_COMPAT nuvoton_m48x_spim_flash_controller
//#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define SOC_NV_FLASH_NODE DT_INST(0, serial_flash)

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

#if defined(CONFIG_MULTITHREADING)
	struct k_sem sem;
#endif

};

static const struct flash_parameters flash_spim_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

#define DEV_DATA(dev) ((struct flash_spim_dev_data *const)(dev)->data)
#define DEV_CFG(dev) ((const struct flash_spim_dev_config *const)(dev)->config)

static inline void flash_spim_sem_take(const struct device *dev)
{
#if defined(CONFIG_MULTITHREADING)
	k_sem_take(&DEV_DATA(dev)->sem, K_FOREVER);
#endif
}

static inline void flash_spim_sem_give(const struct device *dev)
{
#if defined(CONFIG_MULTITHREADING)
	k_sem_give(&DEV_DATA(dev)->sem);
#endif
}


static int flash_spim_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	// const struct flash_spim_dev_config *const cfg = DEV_CFG(dev);
	// LOG_ERR("TODO: flash_spim_read (0x%08x)", address);

	if (length == 0) {
		return 0;
	}
	if (buffer == NULL /*|| address > chip_size || address + length > chip_size*/) {
		return -EINVAL;
	}

	flash_spim_sem_take(dev);

	SPIM_Enable_4Bytes_Mode(0, 1);
    SPIM_SET_DCNUM(8);
    // SPIM_SetQuadEnable(1, 1);    // For CMD_DMA_FAST_QUAD_READ
    SPIM_SetQuadEnable(0, 1);    	// For CMD_DMA_FAST_READ, OPCODE_FAST_READ

	#ifdef CONFIG_SPIM_FLASH_M48X_USE_DMA
		// TODO: buffer must be alligned to 32 bytes!!!
    	SPIM_DMA_Read(address, 0, length, buffer, CMD_DMA_FAST_READ, 1);
    // memset(buffer, 0, length);
	#else
    	SPIM_IO_Read(address, 0, length, buffer, OPCODE_FAST_READ, 1, 1, 1, 1);
	#endif

	flash_spim_sem_give(dev);
	return 0;
}

static int flash_spim_write(const struct device *dev, off_t address, const void *buffer, size_t length)
{
	// const struct flash_spim_dev_config *const cfg = DEV_CFG(dev);
	// LOG_ERR("TODO: flash_spim_write");

	if (length == 0) {
		return 0;
	}
	/*
	if (address + length > chip_size) {
		return -EINVAL;
	}
	*/


	flash_spim_sem_take(dev);

	//
	// SPIM_DMA_Write(address, 0, length, buffer, CMD_QUAD_PAGE_PROGRAM_WINBOND);

	#ifdef CONFIG_SPIM_FLASH_M48X_USE_DMA
		// TODO: buffer must be alligned to 32 bytes!!!
		SPIM_DMA_Write(address, 0, length, (uint8_t*)buffer, CMD_NORMAL_PAGE_PROGRAM);
	#else
    	SPIM_IO_Write(address, 0 /*USE_4_BYTES_MODE*/, length, (uint8_t*)buffer, OPCODE_PP, 1, 1, 1);
	#endif

	flash_spim_sem_give(dev);
	return 0;
}

// Variants:
#define _4K (4*1024)
#define _32K (32*1024)
#define _64K (64*1024)
// SPIM_EraseBlock(alignAddr, 0, OPCODE_SE_4K, 1, 1);		// 4K
// SPIM_EraseBlock(alignAddr, 0, OPCODE_BE_32K, 1, 1);		// 32K
// SPIM_EraseBlock(alignAddr, 0, OPCODE_BE_64K, 1, 1);		// 64K

static int flash_spim_erase(const struct device *dev, off_t start, size_t len)
{
	// uint32_t sector_size = DEV_CFG(dev)->chip->sector_size;
	// uint32_t chip_size = DEV_CFG(dev)->chip->chip_size;
	// LOG_ERR("TODO: flash_spim_erase");

	if (start % FLASH_ERASE_BLK_SZ != 0) {
		LOG_ERR("Start must be aligned to 64K");
		return -EINVAL;
	}
	if (len % FLASH_ERASE_BLK_SZ != 0) {
		LOG_ERR("Length must be aligned to 64K");
		return -EINVAL;
	}
	/*
	if (len + start > chip_size) {
		return -EINVAL;
	}
	*/

	flash_spim_sem_take(dev);
	while (len >= FLASH_ERASE_BLK_SZ) {
		#if FLASH_ERASE_BLK_SZ == _64K
			SPIM_EraseBlock(start, 0, OPCODE_BE_64K, 1, 1);
		#elif FLASH_ERASE_BLK_SZ == _32K
			SPIM_EraseBlock(start, 0, OPCODE_BE_32K, 1, 1);
		#elif FLASH_ERASE_BLK_SZ == _4K
			SPIM_EraseBlock(start, 0, OPCODE_SE_4K, 1, 1);
		#else
			#error  FLASH_ERASE_BLK_SZ must be 4K, 32K or 64K
		#endif
		start += FLASH_ERASE_BLK_SZ;
		len -= FLASH_ERASE_BLK_SZ;
	}
	SPIM_INVALID_CACHE();
	flash_spim_sem_give(dev);
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

#if defined(CONFIG_MULTITHREADING)
	struct flash_spim_dev_data *const dev_data = DEV_DATA(dev);

	k_sem_init(&dev_data->sem, 1, 1);
#endif

	// LOG_ERR("TODO: flash_spim_init(%p)", dev);
	// LOG_ERR("FLASH_WRITE_BLK_SZ = %d", FLASH_WRITE_BLK_SZ);
	// LOG_ERR("FLASH_ERASE_BLK_SZ = %d", FLASH_ERASE_BLK_SZ);
	// int err;
	// const struct spim_config *cfg = dev->config;
	// struct spim_data *data = dev->data;
	// SPIM_T *regs = cfg->regs;

	uint8_t     idBuf[3];

	unsigned int key = irq_lock();
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
	irq_unlock(key);

    // LOG_INF("+-------------------------------------------+\n");
    // LOG_INF("|    M480 SPIM DMA mode read/write sample   |\n");
    // LOG_INF("+-------------------------------------------+\n");

    // TODO: 192MHZ ONLY!
    // SYS_UnlockReg();                   /* Unlock register lock protect */
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

    SPIM_Enable_4Bytes_Mode(0, 1);


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
