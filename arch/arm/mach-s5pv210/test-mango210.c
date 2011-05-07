/* linux/arch/arm/mach-s5pv210/button-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - Button Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <plat/map-base.h>
#include <plat/gpio-cfg.h>

#include <mach/gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/regs-irq.h>
#include <linux/gpio.h>

static void mango_view_reg(unsigned int reg)
{
	printk(KERN_INFO "0x%x : 0x%x\n", reg, __raw_readl(reg));
}

static int __init mango_test_init(void)
{
	int err;

	printk(KERN_INFO "---------------------------------------\n");
	mango_view_reg(S5P_CLK_SRC0);
	mango_view_reg(S5P_CLK_DIV0);
	mango_view_reg(S5P_CLKGATE_IP1);
	

	printk(KERN_INFO "---------------------------------------\n");
	
	return 0;
}
late_initcall(mango_test_init);
