#define DT_DRV_COMPAT nuvoton_m48x_nvmctrl

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(flash_m48x);

#include <device.h>
#include <drivers/flash.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>
#include <string.h>
