/*
 * Copyright (c) 2018 Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(board, CONFIG_APP_LOG_LEVEL);
// LOG_MODULE_REGISTER(board);

/* Aah? */
#ifdef CONFIG_BOARD_PC_001_INIT
#endif /* CONFIG_BOARD_PC_001_INIT */

static int init_board(const struct device *dev)
{
	ARG_UNUSED(dev);

	printk("PC-001 post init (TBD)\n");

	return 0;
}

static int board_init_app(const struct device *dev)
{
	LOG_ERR("App Init: %d\n", 42);
	printk("PC-001 App init (TBD)\n");
	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(init_board, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

SYS_INIT(board_init_app, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
