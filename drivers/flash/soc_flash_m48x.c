#include "soc_flash_m48x.h"

#include <zephyr/kernel.h>
#include <errno.h>
#include <soc.h>
#include <zephyr/drivers/flash.h>

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(soc_flash_m48x);

#if DT_NODE_HAS_STATUS(DT_INST(0, nuvoton_m48x_flash_controller), okay)
#define DT_DRV_COMPAT nuvoton_m48x_flash_controller
#else
#error No matching compatible for soc_flash_nrf.c
#endif

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#if defined(CONFIG_MULTITHREADING)
/* semaphore for locking flash resources (tickers) */
static struct k_sem sem_lock;
#define SYNC_INIT() k_sem_init(&sem_lock, 1, 1)
#define SYNC_LOCK() k_sem_take(&sem_lock, K_FOREVER)
#define SYNC_UNLOCK() k_sem_give(&sem_lock)
#else
#define SYNC_INIT()
#define SYNC_LOCK()
#define SYNC_UNLOCK()
#endif

static const struct flash_parameters flash_nuvoton_parameters = {
	.write_block_size = 4 /* DT_PROP(SOC_NV_FLASH_NODE, write_block_size)*/,
	.erase_value = 0xff,
};

static inline bool is_within_bounds(off_t addr, size_t len, off_t boundary_start,
				    size_t boundary_size)
{
	return (addr >= boundary_start &&
			(addr < (boundary_start + boundary_size)) &&
			(len <= (boundary_start + boundary_size - addr)));
}

static inline bool is_regular_addr_valid(off_t addr, size_t len)
{
	return is_within_bounds(addr, len, 0, DT_REG_SIZE(SOC_NV_FLASH_NODE));
}


static int flash_nuvoton_read(const struct device *dev, off_t addr, void *data, size_t len)
{
	// LOG_DBG("flash_nuvoton_read(0x%08lX, %d)", addr, len);
	if (is_regular_addr_valid(addr, len)) {
		addr += DT_REG_ADDR(SOC_NV_FLASH_NODE);
	} else {
		LOG_ERR("invalid address: 0x%08lx:%zu", (unsigned long)addr, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}
	// LOG_DBG("actual reading from 0x%08lX (%d bytes)", addr, len);
	memcpy(data, (void *)addr, len);
	return 0;

/*
	int ret;
	LOG_DBG("flash_nuvoton_read(0x%08lX, %d)", addr, len);
	LOG_ERR("SOC FLASH READ IS NOT IMPLIMENTED YET");
	ret = -EINVAL;
	return ret;
*/
}

static inline bool is_aligned_32(uint32_t data)
{
	return (data & 0x3) ? false : true;
}

static int flash_nuvoton_write(const struct device *dev, off_t addr, const void *data, size_t len)
{
	LOG_DBG("flash_nuvoton_write(0x%08lX, %d)", addr, len);
	LOG_HEXDUMP_DBG(data, len, "write data");

	if (is_regular_addr_valid(addr, len)) {
		addr += DT_REG_ADDR(SOC_NV_FLASH_NODE);
	} else {
		LOG_ERR("invalid address: 0x%08lx:%zu", (unsigned long)addr, len);
		return -EINVAL;
	}

	if (!is_aligned_32(addr) || (len % sizeof(uint32_t))) {
		LOG_ERR("not word-aligned: 0x%08lx:%zu", (unsigned long)addr, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}
	// ret = write(addr, data, len);

	uint32_t* p = (uint32_t*)data;

	SYNC_LOCK();
	unsigned int key = irq_lock();
	SYS_UnlockReg();
	FMC_Open();
	FMC_ENABLE_AP_UPDATE();

	while (len >= sizeof(uint32_t)) {
		FMC_Write(addr, *p++);
		addr += 4;
		len -= 4;
	}

	// /* Write remaining unaligned data */
	// if (len) {
	// 	LOG_ERR("TODO: You need implement per byte write");
	// }

	FMC_DISABLE_AP_UPDATE();
	FMC_Close();
	SYS_LockReg();
	irq_unlock(key);
	SYNC_UNLOCK();
	return 0;
}

static int flash_nuvoton_erase(const struct device *dev, off_t addr, size_t size)
{
	uint32_t pg_size = FMC_FLASH_PAGE_SIZE;
	uint32_t n_pages = size / pg_size;
	// int ret;

	LOG_DBG("flash_nuvoton_erase(0x%08lX, %d)", addr, size);

	if (is_regular_addr_valid(addr, size)) {
		/* Erase can only be done per page */
		if (((addr % pg_size) != 0) || ((size % pg_size) != 0)) {
			LOG_ERR("unaligned address: 0x%08lx:%zu", (unsigned long)addr, size);
			return -EINVAL;
		}

		if (!n_pages) {
			return 0;
		}

		addr += DT_REG_ADDR(SOC_NV_FLASH_NODE);
	} else {
		LOG_ERR("invalid address: 0x%08lx:%zu", (unsigned long)addr, size);
		return -EINVAL;
	}

	SYNC_LOCK();
	unsigned int key = irq_lock();
	SYS_UnlockReg();
	FMC_Open();
	FMC_ENABLE_AP_UPDATE();             /* enable APROM update */
	for(int i=0; i<n_pages; i++) {
		FMC_Erase(addr);
		addr += FMC_FLASH_PAGE_SIZE;
	}
	FMC_DISABLE_AP_UPDATE();
	FMC_Close();
	SYS_LockReg();
	irq_unlock(key);
	SYNC_UNLOCK();

	// int ret;
	// LOG_ERR("SOC FLASH ERASE IS NOT IMPLIMENTED YET");
	// ret = -EINVAL;
	return 0;
}


#if defined(CONFIG_FLASH_PAGE_LAYOUT)
// static struct flash_pages_layout dev_layout;
// TODO: I think we can replace:
// pages_size = FMC_FLASH_PAGE_SIZE
static const struct flash_pages_layout dev_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

static void flash_nuvoton_pages_layout(const struct device *dev, const struct flash_pages_layout **layout, size_t *layout_size)
{
	LOG_DBG("flash_nuvoton_pages_layout");
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters* flash_nuvoton_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &flash_nuvoton_parameters;
}


static const struct flash_driver_api flash_nuvoton_api = {
	.read = flash_nuvoton_read,
	.write = flash_nuvoton_write,
	.erase = flash_nuvoton_erase,
	.get_parameters = flash_nuvoton_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_nuvoton_pages_layout,
#endif
};


static int nuvoton_flash_init(const struct device *dev)
{
	SYNC_INIT();

	// nrf_flash_sync_init();

// 	// TODO: Move to dts?
// #if defined(CONFIG_FLASH_PAGE_LAYOUT)
// 	// dev_layout.pages_count = 2; 		//nrfx_nvmc_flash_page_count_get();
// 	// dev_layout.pages_size = KB(256); 	//nrfx_nvmc_flash_page_size_get();
// 	dev_layout.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / DT_PROP(SOC_NV_FLASH_NODE, erase_block_size);
// 	dev_layout.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size);
// #endif

	return 0;
}

DEVICE_DT_INST_DEFINE(0, nuvoton_flash_init, NULL,
		 NULL, NULL,
		 POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		 &flash_nuvoton_api);
