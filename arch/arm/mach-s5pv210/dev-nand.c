/*
 * linux/arch/arm/mach-s5pv210/dev-onenand.c
 *
 *  Copyright (c) 2008-2010 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 * S5PC110 series device definition for OneNAND devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>

#include <mach/irqs.h>
#include <mach/map.h>

static struct resource s5pv210_nand_resources[] = {
	[0] = {
		.start	= S5P_PA_NAND,
		.end	= S5P_PA_NAND + S5P_SZ_NAND - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device s5pv210_device_nand = {
	.name		= "s5pv210-nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s5pv210_nand_resources),
	.resource	= s5pv210_nand_resources,
};

#if 0
void s5pv210_nand_set_platdata(struct nand_platform_data *pdata)
{
	struct nand_platform_data *pd;

	pd = kmemdup(pdata, sizeof(struct nand_platform_data), GFP_KERNEL);
	if (!pd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	s5pv210_device_nand.dev.platform_data = pd;
}
#endif
