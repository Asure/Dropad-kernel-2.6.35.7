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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <plat/map-base.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-gpio.h>
#include <mach/regs-irq.h>
#include <linux/gpio.h>

static unsigned int mango210_wifi_init(void)
{
	u32 err;

	err = gpio_request(S5PV210_GPC1(1), "GPC1");
	if (err) {
		printk(KERN_INFO "gpio request error : %d\n", err);
	} else {
		s3c_gpio_setpull(S5PV210_GPC1(1), S3C_GPIO_PULL_NONE);
		gpio_direction_output(S5PV210_GPC1(1), 0);
		mdelay(100);
		gpio_direction_output(S5PV210_GPC1(1), 1);
	}

	gpio_free(S5PV210_GPC1(1));

	return err;
}

static int __init mango_wifi_init(void)
{
	printk(KERN_INFO "MANGO210 wifi init function \n");

	if (mango210_wifi_init()) {
		printk(KERN_ERR "%s failed\n", __func__);
		return 0;
	}

	return 0;
}
device_initcall(mango_wifi_init);
