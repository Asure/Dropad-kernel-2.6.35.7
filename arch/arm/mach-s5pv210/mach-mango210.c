/* linux/arch/arm/mach-s5pv210/mach-mango210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/skbuff.h>
#include <linux/console.h>
#include <linux/pwm_backlight.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/system.h>

#include <mach/adc.h>
#include <mach/map.h>
#include <mach/regs-adc.h>
#include <mach/regs-clock.h>
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/gpio-mango210.h>
#include <mach/param.h>
#include <mach/system.h>
#include <mach/ts.h>

#include <linux/usb/gadget.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/wlan_plat.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#include <plat/media.h>
#include <mach/media.h>
#endif

#include <media/noon130pc20_platform.h>
#include <linux/smsc911x.h>
#include <linux/vd5376.h>

#include <plat/regs-serial.h>
#include <plat/s5pv210.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/fb.h>
#include <plat/mfc.h>
#include <plat/iic.h>
#include <plat/pm.h>
#include <mach/regs-mem.h>
#include <plat/sdhci.h>
#include <plat/fimc.h>
#include <plat/jpeg.h>
#include <plat/clock.h>
#include <plat/regs-otg.h>

#include <linux/mango_keys.h>
#include <linux/input.h>

#include <../../../drivers/video/samsung/s3cfb.h>
#include <linux/switch.h>

#include "herring.h"

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

void (*sec_set_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_set_param_value);

void (*sec_get_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_get_param_value);

#define KERNEL_REBOOT_MASK      0xFFFFFFFF
#define REBOOT_MODE_FAST_BOOT		7

static int mango210_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = REBOOT_MODE_NONE;
	unsigned int temp;

	if ((code == SYS_RESTART) && _cmd) {
		if (!strcmp((char *)_cmd, "recovery"))
			mode = REBOOT_MODE_RECOVERY;
		else if (!strcmp((char *)_cmd, "bootloader"))
			mode = REBOOT_MODE_FAST_BOOT;
		else
			mode = REBOOT_MODE_NONE;
	}
	temp = __raw_readl(S5P_INFORM6);
	temp |= KERNEL_REBOOT_MASK;
	temp &= mode;
	__raw_writel(temp, S5P_INFORM6);

	return NOTIFY_DONE;
}

static struct notifier_block mango210_reboot_notifier = {
	.notifier_call = mango210_notifier_call,
};

static void uart_switch_init(void)
{
	int ret;
	struct device *uartswitch_dev;

	uartswitch_dev = device_create(sec_class, NULL, 0, NULL, "uart_switch");
	if (IS_ERR(uartswitch_dev)) {
		pr_err("Failed to create device(uart_switch)!\n");
		return;
	}

	ret = gpio_request(GPIO_UART_SEL, "UART_SEL");
	if (ret < 0) {
		pr_err("Failed to request GPIO_UART_SEL!\n");
		return;
	}
	s3c_gpio_setpull(GPIO_UART_SEL, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_UART_SEL, S3C_GPIO_OUTPUT);
	gpio_direction_output(GPIO_UART_SEL, 1);

	gpio_export(GPIO_UART_SEL, 1);

	gpio_export_link(uartswitch_dev, "UART_SEL", GPIO_UART_SEL);
}

static void mango210_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");

	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");
};

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define S5PV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define S5PV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define S5PV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg mango210_uartcfgs[] __initdata = {
	{
		.hwport		= 0,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
#ifndef CONFIG_FIQ_DEBUGGER
	{
		.hwport		= 1,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
#endif
	{
		.hwport		= 2,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	{
		.hwport		= 3,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_ANDROID_PMEM
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC0 (24576 * SZ_1K) //by Asure
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC1 (9900 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC2 (24576 * SZ_1K) //by Asure
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0 (36864 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1 (36864 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMD (3000 * SZ_1K) // By Asure
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_JPEG (8192 * SZ_1K)

//need PMEM_GPU1 fixed 10485760 bytes
//need PMEM fixed 16777216 bytes
//need PMEM_adsp fixed? 1048576 bytes

static struct s5p_media_device mango210_media_devs[] = {
	[0] = {
		.id = S5P_MDEV_MFC,
		.name = "mfc",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0,
		.paddr = 0,
	},
	[1] = {
		.id = S5P_MDEV_MFC,
		.name = "mfc",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1,
		.paddr = 0,
	},
	[2] = {
		.id = S5P_MDEV_FIMC0,
		.name = "fimc0",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC0,
		.paddr = 0,
	},
	[3] = {
		.id = S5P_MDEV_FIMC1,
		.name = "fimc1",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC1,
		.paddr = 0,
	},
	[4] = {
		.id = S5P_MDEV_FIMC2,
		.name = "fimc2",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC2,
		.paddr = 0,
	},
	[5] = {
		.id = S5P_MDEV_JPEG,
		.name = "jpeg",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_JPEG,
		.paddr = 0,
	},
	[6] = {
		.id = S5P_MDEV_FIMD,
		.name = "fimd",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMD,
		.paddr = 0,
	},
/* pmem */
	[7] = {
		.id = S5P_MDEV_PMEM,
		.name = "pmem",
		.memsize = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K,
		.paddr = 0,
		.bank = 0, //OneDRAM
	},
	[8] = {
		.id = S5P_MDEV_PMEM_GPU1,
		.name = "pmem_gpu1",
		.memsize = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K,
		.paddr = 0,
		.bank = 0, //OneDRAM 
	},
	[9] = {
		.id = S5P_MDEV_PMEM_ADSP,
		.name = "pmem_adsp",
		.memsize = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_ADSP * SZ_1K,
		.paddr = 0,
		.bank = 0, //OneDRAM
	},
};
#endif

struct platform_device sec_device_dpram = {
	.name	= "dpram-device",
	.id	= -1,
};

#ifdef CONFIG_TOUCHSCREEN_S3C
static struct s3c_ts_mach_info s3c_ts_platform __initdata = {
	.delay              = 10000,
	.presc              = 49,
	.oversampling_shift = 2,
	.resol_bit          = 12,
	.s3c_adc_con        = ADC_TYPE_2,
};

static struct s3c_adc_mach_info s3c_adc_platform __initdata = {
	.delay 			= 0xff,
	.presc 			= 49,
	.resolution 	= 12,
};
#endif

#ifdef CONFIG_KEYBOARD_MANGO
static struct mango_keys_button mango_gpio_keys_table[] = {
	{
		.code			= KEY_BACK,
		.gpio			= S5PV210_GPH0(1),
		.active_low		= 1,
		.desc			= "GPH0",
		.type			= EV_KEY,
		.wakeup			= 1,
//		.debounce_interval	= 5,
		.irq			= IRQ_EINT1,
		.config			= (0xf << 4),
		.pull			= S3C_GPIO_PULL_NONE,
		.active_low		= 1,
		.led_gpio		= S5PV210_GPJ3(7),
		.led_desc		= "GPJ3",
		.led_config		= (0x1 << (4 * 7)),
		.led_active_low		= 1,
	},
	{
		.code			= KEY_MENU,
		.gpio			= S5PV210_GPH0(2),
		.active_low		= 1,
		.desc			= "GPH0",
		.type			= EV_KEY,
		.wakeup			= 1,
//		.debounce_interval	= 5,
		.irq			= IRQ_EINT2,
		.config			= (0xf<<8),
		.pull			= S3C_GPIO_PULL_NONE,
		.active_low		= 1,
		.led_gpio		= S5PV210_GPJ4(0),
		.led_desc		= "GPJ4",
		.led_config		= (0x1 << (4 * 0)),
		.led_active_low		= 1,
	},
};

static struct mango_keys_platform_data mango_gpio_keys_data = {
	.buttons	= mango_gpio_keys_table,
	.nbuttons	= ARRAY_SIZE(mango_gpio_keys_table),
	.rep		= 0,
};

static struct platform_device mango_device_gpiokeys = {
	.name	= "mango-keys",
	.dev	= {
		.platform_data	= &mango_gpio_keys_data,
	},
};
#endif

#if defined(CONFIG_HAVE_PWM)
static struct platform_pwm_backlight_data mango_backlight_data = {
	.pwm_id  = 0,
	.max_brightness = 255,
	.dft_brightness = 110,
//	.pwm_period_ns  = 89284/2,
	.pwm_period_ns  = 78770,
};

static struct platform_device mango_backlight_device = {
	.name      = "pwm-backlight",
	.id        = -1,
	.dev        = {
		.parent = &s3c_device_timer[0].dev,
		.platform_data = &mango_backlight_data,
	},
};
#endif

#ifdef CONFIG_BATTERY_MANGO_DUMMY
static struct platform_device mango_battery = {
	.name = "dummy-battery",
};
#endif

#ifdef CONFIG_MOUSE_VD5376_FINGER
static struct vd5376_platform_data vd5376_table[] = {
	{
		.btn_gpio = S5PV210_GPH2(0),
		.btn_desc = "GPH2",
		.btn_irq = IRQ_EINT(16),
		.btn_config = (0xf << 0),
		.btn_active_low = 1,
		.pd_gpio = S5PV210_GPH2(1),
		.pd_desc = "GPH2",
		.gpio = S5PV210_GPH2(2),
		.desc = "GPH2",
		.irq = IRQ_EINT(18),
		.config = (0xf << 8),
		.active_low = 1,
		.delay = 20,
	},
	{
		.btn_gpio = S5PV210_GPH2(3),
		.btn_desc = "GPH2",
		.btn_irq = IRQ_EINT(19),
		.btn_config = (0xf << 12),
		.btn_active_low = 1,
		.pd_gpio = S5PV210_GPH2(4),
		.pd_desc = "GPH2",
		.gpio = S5PV210_GPH2(5),
		.desc = "GPH2",
		.irq = IRQ_EINT(21),
		.config = (0xf << 20),
		.active_low = 1,
		.delay = 20,
	},
};

struct vd5376_mouse_platform_data vd5376_mouse_data = {
	.data = vd5376_table,
	.ndatas	= ARRAY_SIZE(vd5376_table),
	.scan_ms = 50,
};

static struct platform_device vd5376_mouse_device = {
	.name	= "vd5376-mouse",
	.dev	= {
		.platform_data	= &vd5376_mouse_data,
	},
};
#endif

static  struct  i2c_gpio_platform_data  i2c0_platdata = {
	.sda_pin		= GPIO_I2C_SDA0,
	.scl_pin		= GPIO_I2C_SCL0,
	.udelay			= 4,    /* 125KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_gpio_device_i2c0 = {
	.name			= "i2c-gpio",
	.id			= 0,
	.dev.platform_data	= &i2c0_platdata,
};

static  struct  i2c_gpio_platform_data  i2c2_platdata = {
	.sda_pin		= GPIO_I2C_SDA2,
	.scl_pin		= GPIO_I2C_SCL2,
	.udelay			= 2,    /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_gpio_device_i2c2 = {
	.name			= "i2c-gpio",
	.id			= 2,
	.dev.platform_data	= &i2c2_platdata,
};

/*
 * Guide for Camera Configuration for Crespo board
 * ITU CAM CH A: LSI noon130pc20
 */
/* External camera module setting */
static DEFINE_MUTEX(noon130pc20_lock);

static bool noon130pc20_powered_on;

static int noon130pc20_request_gpio(void)
{
	int err;

	/* CAM_VGA_nSTBY - GPB(0) */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPB0");
	if (err) {
		pr_err("Failed to request GPB0 for camera control\n");
		return -EINVAL;
	}

	/* CAM_VGA_nRST - GPB(2) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPB2");
	if (err) {
		pr_err("Failed to request GPB2 for camera control\n");
		gpio_free(GPIO_CAM_VGA_nSTBY);
		return -EINVAL;
	}

	return 0;
}


static int noon130pc20_power_on(void)
{
	int err = 0;
	int result;


	/* CAM_VGA_nSTBY HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 1);

	udelay(10);

	/* Mclk enable */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(0x02));
	udelay(430);

	/* CAM_VGA_nRST HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	gpio_set_value(GPIO_CAM_VGA_nRST, 1);
	mdelay(5);

	return 0;
}

static int noon130pc20_power_off(void)
{
	int err;

	/* CAM_VGA_nRST LOW */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	gpio_set_value(GPIO_CAM_VGA_nRST, 0);
	udelay(430);

	/* Mclk disable */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);

	udelay(1);

	/* CAM_VGA_nSTBY LOW */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 0);

	return err;
}

static int noon130pc20_power_en(int onoff)
{
	int err = 0;
	mutex_lock(&noon130pc20_lock);
	/* we can be asked to turn off even if we never were turned
	 * on if something odd happens and we are closed
	 * by camera framework before we even completely opened.
	 */
	if (onoff != noon130pc20_powered_on) {
		if (onoff)
			err = noon130pc20_power_on();
		else {
			err = noon130pc20_power_off();
//			s3c_i2c0_force_stop();
		}
		if (!err)
			noon130pc20_powered_on = onoff;
	}
	mutex_unlock(&noon130pc20_lock);

	return err;
}

static struct noon130pc20_platform_data noon130pc20_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 24000000,
	.is_mipi = 0,

	.cam_power = noon130pc20_power_en,
};

static struct i2c_board_info noon130pc20_i2c_info = {
	I2C_BOARD_INFO("NOON130PC20", 0x20),
	.platform_data = &noon130pc20_plat,
};

static struct s3c_platform_camera noon130pc20 = {
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.i2c_busnum	= 0,
	.info		= &noon130pc20_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam",
	.clk_rate	= 24000000,
	.line_length	= 480,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
	.cam_power	= noon130pc20_power_en,
};


/* Interface setting */
static struct s3c_platform_fimc fimc_plat_lsi = {
	.srclk_name	= "mout_mpll",
	.clk_name	= "sclk_fimc",
	.lclk_name	= "sclk_fimc_lclk",
	.clk_rate	= 166750000,
	.default_cam	= CAMERA_PAR_A,
	.camera		= {
		&noon130pc20,
		&noon130pc20,
	},
	.hw_ver		= 0x43,
};

#ifdef CONFIG_VIDEO_JPEG_V2
static struct s3c_platform_jpeg jpeg_plat __initdata = {
	.max_main_width	= 800,
	.max_main_height	= 480,
	.max_thumb_width	= 320,
	.max_thumb_height	= 240,
};
#endif

#ifdef CONFIG_USB_SUPPORT
static void __init mango210_usb_host_set(void)
{
	int err;

	err = gpio_request(S5PV210_ETC2(7), "ETC2");
	if (err)
		printk(KERN_ERR "#### failed to request ETC2 for USB host\n");

	s3c_gpio_setpull(S5PV210_ETC2(7), S3C_GPIO_PULL_DOWN);
	gpio_free(S5PV210_ETC2(7));
}
#endif

#if defined(CONFIG_FB_S3C_LB070WV6) || defined(CONFIG_FB_S3C_LTN101NT05) || \
	defined(CONFIG_FB_S3C_AT070TN90) || defined (CONFIG_FB_S3C_UTLCD7B)
static struct s3c_platform_fb mango_fb_data __initdata = {
	.hw_ver	= 0x62,
	.nr_wins = 5,
	.default_win = CONFIG_FB_S3C_DEFAULT_WINDOW,
	.swap = FB_SWAP_WORD | FB_SWAP_HWORD,
};
#endif

/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
	},
#endif
#ifdef CONFIG_MOUSE_VD5376_FINGER
	{
//		I2C_BOARD_INFO("vd5376", 0x10),
		I2C_BOARD_INFO("vd5376", 0x53),
		.platform_data = &vd5376_table[0],
	},
#endif
#ifdef CONFIG_SENSORS_BMA150
	{
		I2C_BOARD_INFO("bma150", 0x38),
	},
#endif
#ifdef CONFIG_BMP085
	{
		I2C_BOARD_INFO("bmp085", 0x77),
	},
#endif
};

/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
#ifdef CONFIG_VIDEO_TV20
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74>>1)),
	},
#endif
};

/* I2C2 */
static struct i2c_board_info i2c_devs2[] __initdata = {
#ifdef CONFIG_MOUSE_VD5376_FINGER
	{
//		I2C_BOARD_INFO("vd5376", 0x10),
		I2C_BOARD_INFO("vd5376", 0x53),
		.platform_data = &vd5376_table[1],
	},
#endif
};

#if 0
static struct resource ram_console_resource[] = {
	{
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
	.start = 0,
	.size = 0,
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.no_allocator = 1,
	.cached = 1,
	.buffered = 1,
	.start = 0,
	.size = 0,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 1,
	.cached = 1,
	.buffered = 1,
	.start = 0,
	.size = 0,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_gpu1_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static void __init android_pmem_set_platdata(void)
{
	pmem_pdata.start = (u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM, 0);
	pmem_pdata.size = (u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM, 0);

	pmem_gpu1_pdata.start =
		(u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM_GPU1, 0);
	pmem_gpu1_pdata.size =
		(u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM_GPU1, 0);

	pmem_adsp_pdata.start =
		(u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM_ADSP, 0);
	pmem_adsp_pdata.size =
		(u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM_ADSP, 0);
}
#endif

#ifdef CONFIG_SMSC911X
static struct smsc911x_platform_config smsc911x_config = {
    .irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
    .irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,
    .flags          = SMSC911X_USE_32BIT,
    .phy_interface  = PHY_INTERFACE_MODE_MII,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start = S5PV210_PA_SMSC9220,
		.end   = S5PV210_PA_SMSC9220 + 0xFF,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EINT11,
		.end   = IRQ_EINT11,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	}
};

struct platform_device device_smsc911x = {
	.name		= "smsc911x",
	.id			=  0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	}
};

static void __init smsc911x_set(void)
{
	unsigned int tmp;

	tmp = ((0<<28)|(4<<24)|(13<<16)|(1<<12)|(4<<8)|(6<<4)|(0<<0));
	__raw_writel(tmp, (S5P_SROM_BW + 0x08));

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(0xf << 4);
	tmp |= (0x1 << 4) | (0x1 << 5);
	__raw_writel(tmp, S5P_SROM_BW);

	tmp = __raw_readl(S5PV210_MP01_BASE);
	tmp &= ~(0xf << 4);
	tmp |= (2 << 4);
	__raw_writel(tmp, S5PV210_MP01_BASE);
}
#else
static void __init smsc911x_set(void) {}
#endif

#ifdef CONFIG_DM9000
static void __init smdkv210_dm9000_set(void)
{
	unsigned int tmp;

	tmp = ((0<<28)|(0<<24)|(7<<16)|(0<<12)|(0<<8)|(0<<4)|(0<<0));
	__raw_writel(tmp, (S5P_SROM_BC1));

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(0xf<<4);
	tmp |= (1<<5);
#ifdef CONFIG_DM9000_16BIT
	tmp |= (1<<4);
#else
	tmp |= (0<<4);
#endif

	__raw_writel(tmp, S5P_SROM_BW);

/*
 * MP01CON[7:4] defaults to CSn[1], no need to set it
 *
	tmp  = __raw_readl(S5PV210_MP01CON);
	tmp &= ~(0xf << 4);
	tmp |= (2 << 4);

	__raw_writel(tmp,(S5PV210_MP01CON));
*/
}
#endif

#define S3C_GPIO_SETPIN_ZERO         0
#define S3C_GPIO_SETPIN_ONE          1
#define S3C_GPIO_SETPIN_NONE	     2

static void mango210_power_off(void)
{
	while (1) {
		mdelay(1000);
	}
}

static struct platform_device watchdog_device = {
	.name = "watchdog",
	.id = -1,
};

static struct platform_device *mango210_devices[] __initdata = {
	&watchdog_device,
#ifdef CONFIG_MTD_NAND
	&s5pv210_device_nand,
#endif
	&s3c_device_adc,
#ifdef CONFIG_TOUCHSCREEN_S3C
	&s3c_device_ts,
#endif
#ifdef CONFIG_FIQ_DEBUGGER
	&s5pv210_device_fiqdbg_uart1,
#endif

#ifdef CONFIG_RTC_DRV_S3C
	&s5p_device_rtc,
#endif
	&s5pv210_device_iis0,
	&s3c_device_wdt,

#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
#endif

#ifdef CONFIG_SMSC911X
	&device_smsc911x,
#endif

#ifdef CONFIG_DM9000
	&s5p_device_dm9000,
#endif

#ifdef CONFIG_VIDEO_MFC50
	&s3c_device_mfc,
#endif
#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
#endif

#ifdef CONFIG_VIDEO_JPEG_V2
	&s3c_device_jpeg,
#endif

	&s3c_device_g3d,
	&s3c_device_lcd,

#ifdef CONFIG_MOUSE_VD5376_FINGER
	&vd5376_mouse_device,
#endif

	&s3c_gpio_device_i2c0,
#if defined(CONFIG_S3C_DEV_I2C1)
	&s3c_device_i2c1,
#endif

#if defined(CONFIG_S3C_DEV_I2C2)
	&s3c_gpio_device_i2c2,
#endif
#ifdef CONFIG_USB
	&s3c_device_usb_ehci,
	&s3c_device_usb_ohci,
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif

#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&s3c_device_usb_mass_storage,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	&s3c_device_acm,
#endif
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif


#ifdef CONFIG_VIDEO_TV20
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif

#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
	&pmem_adsp_device,
#endif

#ifdef CONFIG_HAVE_PWM
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
#endif
#ifdef CONFIG_KEYBOARD_MANGO
	&mango_device_gpiokeys,
#endif
#ifdef CONFIG_BATTERY_MANGO_DUMMY
	&mango_battery,
#endif
#ifdef CONFIG_HAVE_PWM
	&mango_backlight_device,
#endif

//     &ram_console_device,

#ifdef CONFIG_SND_S3C_SOC_AC97
	&s5pv210_device_ac97,
#endif
};

unsigned int HWREV;
EXPORT_SYMBOL(HWREV);

static void __init mango210_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(mango210_uartcfgs, ARRAY_SIZE(mango210_uartcfgs));
#ifdef CONFIG_ANDROID_PMEM
	s5p_reserve_bootmem(mango210_media_devs, ARRAY_SIZE(mango210_media_devs));
#endif
#ifdef CONFIG_MTD_NAND
	s5pv210_device_nand.name = "s5pv210-nand";
#endif
}

//unsigned int pm_debug_scratchpad;

//static unsigned int ram_console_start;
//static unsigned int ram_console_size;

static void __init mango210_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline,
		struct meminfo *mi)
{
	mi->bank[0].start = 0x20000000;
	mi->bank[0].size = 256 * SZ_1M;
	mi->bank[0].node = 0;

	mi->bank[1].start = 0x40000000;
	mi->bank[1].size = 256 * SZ_1M;
	mi->bank[1].node = 0;

	mi->nr_banks = 2;

//	ram_console_start = mi->bank[1].start;
//	ram_console_size = SZ_1M - SZ_4K;

//	pm_debug_scratchpad = ram_console_start + ram_console_size;
}

/* this function are used to detect s5pc110 chip version temporally */
int s5pc110_version ;

void _hw_version_check(void)
{
	void __iomem *phy_address ;
	int temp;

	phy_address = ioremap(0x40, 1);

	temp = __raw_readl(phy_address);

	if (temp == 0xE59F010C)
		s5pc110_version = 0;
	else
		s5pc110_version = 1;

	printk(KERN_INFO "S5PC110 Hardware version : EVT%d\n",
				s5pc110_version);

	iounmap(phy_address);
}

/*
 * Temporally used
 * return value 0 -> EVT 0
 * value 1 -> evt 1
 */

int hw_version_check(void)
{
	return s5pc110_version ;
}
EXPORT_SYMBOL(hw_version_check);

#if 0
static void __init setup_ram_console_mem(void)
{
	ram_console_resource[0].start = ram_console_start;
	ram_console_resource[0].end = ram_console_start + ram_console_size - 1;
}
#endif
static bool console_flushed;

static void flush_console(void)
{
	if (console_flushed)
		return;

	console_flushed = true;

	printk("\n");
	pr_emerg("Restarting %s\n", linux_banner);
	if (!try_acquire_console_sem()) {
		release_console_sem();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (try_acquire_console_sem())
		pr_emerg("flush_console: console was locked! busting!\n");
	else
		pr_emerg("flush_console: console was locked!\n");
	release_console_sem();
}

static void mango210_pm_restart(char mode, const char *cmd)
{
	flush_console();
	arm_machine_restart(mode, cmd);
}

#ifdef CONFIG_VIDEO_TV20
void s3c_set_qos()
{
	/* VP QoS */
	__raw_writel(0x00400001, S5P_VA_DMC0 + 0xC8);
	__raw_writel(0x387F0022, S5P_VA_DMC0 + 0xCC);
	/* MIXER QoS */
	__raw_writel(0x00400001, S5P_VA_DMC0 + 0xD0);
	__raw_writel(0x3FFF0062, S5P_VA_DMC0 + 0xD4);
	/* LCD1 QoS */
	__raw_writel(0x00800001, S5P_VA_DMC1 + 0x90);
	__raw_writel(0x3FFF005B, S5P_VA_DMC1 + 0x94);
	/* LCD2 QoS */
	__raw_writel(0x00800001, S5P_VA_DMC1 + 0x98);
	__raw_writel(0x3FFF015B, S5P_VA_DMC1 + 0x9C);
	/* VP QoS */
	__raw_writel(0x00400001, S5P_VA_DMC1 + 0xC8);
	__raw_writel(0x387F002B, S5P_VA_DMC1 + 0xCC);
	/* DRAM Controller QoS */
	__raw_writel((__raw_readl(S5P_VA_DMC0)&~(0xFFF<<16)|(0x100<<16)),
			S5P_VA_DMC0 + 0x0);
	__raw_writel((__raw_readl(S5P_VA_DMC1)&~(0xFFF<<16)|(0x100<<16)),
			S5P_VA_DMC1 + 0x0);
	/* BUS QoS AXI_DSYS Control */
	__raw_writel(0x00000007, S5P_VA_BUS_AXI_DSYS + 0x400);
	__raw_writel(0x00000007, S5P_VA_BUS_AXI_DSYS + 0x420);
	__raw_writel(0x00000030, S5P_VA_BUS_AXI_DSYS + 0x404);
	__raw_writel(0x00000030, S5P_VA_BUS_AXI_DSYS + 0x424);
}
#endif

static void __init mango210_machine_init(void)
{
	arm_pm_restart = mango210_pm_restart;

//	setup_ram_console_mem();

//	s3c_usb_set_serial();
	platform_add_devices(mango210_devices, ARRAY_SIZE(mango210_devices));

	/* Find out S5PC110 chip version */
	_hw_version_check();

	pm_power_off = mango210_power_off ;

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_set_platdata();
#endif

	/* i2c */
	s3c_i2c0_set_platdata(NULL);
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
#endif

#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
#endif
	/* H/W I2C lines */
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));

#if defined(CONFIG_FB_S3C_LB070WV6) || defined(CONFIG_FB_S3C_LTN101NT05) || \
	defined(CONFIG_FB_S3C_AT070TN90) || defined(CONFIG_FB_S3C_UTLCD7B)
	s3cfb_set_platdata(&mango_fb_data);
#endif

#if defined(CONFIG_TOUCHSCREEN_S3C)
	s3c_adc_set_platdata(&s3c_adc_platform);
	s3c_ts_set_platdata(&s3c_ts_platform);
#endif

#if defined(CONFIG_PM)
	s3c_pm_init();
#endif

	noon130pc20_request_gpio();

#ifdef CONFIG_VIDEO_FIMC
	/* fimc */
	s3c_fimc0_set_platdata(&fimc_plat_lsi);
	s3c_fimc1_set_platdata(&fimc_plat_lsi);
	s3c_fimc2_set_platdata(&fimc_plat_lsi);
#endif

#ifdef CONFIG_VIDEO_JPEG_V2
	s3c_jpeg_set_platdata(&jpeg_plat);
#endif

#ifdef CONFIG_VIDEO_MFC50
	/* mfc */
	s3c_mfc_set_platdata(NULL);
#endif

#ifdef CONFIG_VIDEO_TV20
//	s3c_set_qos();
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
	s5pv210_default_sdhci0();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s5pv210_default_sdhci1();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s5pv210_default_sdhci2();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s5pv210_default_sdhci3();
#endif
#ifdef CONFIG_S5PV210_SETUP_SDHCI
	s3c_sdhci_set_platdata();
#endif

#ifdef CONFIG_SMSC911X
	smsc911x_set();
#endif

#ifdef CONFIG_DM9000
	smdkv210_dm9000_set();
#endif

#ifdef CONFIG_USB_SUPPORT
	mango210_usb_host_set();
#endif

	register_reboot_notifier(&mango210_reboot_notifier);

	mango210_switch_init();

	uart_switch_init();
}

#ifdef CONFIG_USB_SUPPORT
/* Initializes OTG Phy. */
void otg_phy_init(void)
{
	/* USB PHY0 Enable */
	writel(readl(S5P_USB_PHY_CONTROL) | (0x1<<0),
			S5P_USB_PHY_CONTROL);
	writel((readl(S3C_USBOTG_PHYPWR) & ~(0x3<<3) & ~(0x1<<0)) | (0x1<<5),
			S3C_USBOTG_PHYPWR);
	writel((readl(S3C_USBOTG_PHYCLK) & ~(0x5<<2)) | (0x3<<0),
			S3C_USBOTG_PHYCLK);
	writel((readl(S3C_USBOTG_RSTCON) & ~(0x3<<1)) | (0x1<<0),
			S3C_USBOTG_RSTCON);
	msleep(1);
	writel(readl(S3C_USBOTG_RSTCON) & ~(0x7<<0),
			S3C_USBOTG_RSTCON);
	msleep(1);

	/* rising/falling time */
	writel(readl(S3C_USBOTG_PHYTUNE) | (0x1<<20),
			S3C_USBOTG_PHYTUNE);

	/* set DC level as 6 (6%) */
	writel((readl(S3C_USBOTG_PHYTUNE) & ~(0xf)) | (0x1<<2) | (0x1<<1),
			S3C_USBOTG_PHYTUNE);
}
EXPORT_SYMBOL(otg_phy_init);

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(64)));

/* OTG PHY Power Off */
void otg_phy_off(void)
{
	writel(readl(S3C_USBOTG_PHYPWR) | (0x3<<3),
			S3C_USBOTG_PHYPWR);
	writel(readl(S5P_USB_PHY_CONTROL) & ~(1<<0),
			S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(otg_phy_off);

void usb_host_phy_init(void)
{
	struct clk *otg_clk;

	otg_clk = clk_get(NULL, "otg");
	clk_enable(otg_clk);

	if (readl(S5P_USB_PHY_CONTROL) & (0x1<<1))
		return;

	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL) | (0x1<<1),
			S5P_USB_PHY_CONTROL);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
			& ~(0x1<<7) & ~(0x1<<6)) | (0x1<<8) | (0x1<<5),
			S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) & ~(0x1<<7)) | (0x3<<0),
			S3C_USBOTG_PHYCLK);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)) | (0x1<<4) | (0x1<<3),
			S3C_USBOTG_RSTCON);
	__raw_writel(__raw_readl(S3C_USBOTG_RSTCON) & ~(0x1<<4) & ~(0x1<<3),
			S3C_USBOTG_RSTCON);
}
EXPORT_SYMBOL(usb_host_phy_init);

void usb_host_phy_off(void)
{
	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR) | (0x1<<7)|(0x1<<6),
			S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL) & ~(1<<1),
			S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(usb_host_phy_off);
#endif

MACHINE_START(MANGO210, "MANGO210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.fixup		= mango210_fixup,
	.init_irq	= s5pv210_init_irq,
	.map_io		= mango210_map_io,
	.init_machine	= mango210_machine_init,
#if	defined(CONFIG_S5P_HIGH_RES_TIMERS)
	.timer		= &s5p_systimer,
#else
	.timer		= &s3c24xx_timer,
#endif
MACHINE_END

void s3c_setup_uart_cfg_gpio(unsigned char port)
{
	switch (port) {
	case 0:
		s3c_gpio_cfgpin(GPIO_BT_RXD, S3C_GPIO_SFN(GPIO_BT_RXD_AF));
		s3c_gpio_setpull(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_TXD, S3C_GPIO_SFN(GPIO_BT_TXD_AF));
		s3c_gpio_setpull(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_CTS, S3C_GPIO_SFN(GPIO_BT_CTS_AF));
		s3c_gpio_setpull(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_RTS, S3C_GPIO_SFN(GPIO_BT_RTS_AF));
		s3c_gpio_setpull(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_TXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_CTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 1:
		s3c_gpio_cfgpin(GPIO_AP_RXD, S3C_GPIO_SFN(GPIO_AP_RXD_AF));
		s3c_gpio_setpull(GPIO_AP_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_TXD, S3C_GPIO_SFN(GPIO_AP_TXD_AF));
		s3c_gpio_setpull(GPIO_AP_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_CTS, S3C_GPIO_SFN(GPIO_AP_CTS_AF));
		s3c_gpio_setpull(GPIO_AP_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_RTS, S3C_GPIO_SFN(GPIO_AP_RTS_AF));
		s3c_gpio_setpull(GPIO_AP_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 2:
		s3c_gpio_cfgpin(GPIO_GPS_RXD, S3C_GPIO_SFN(GPIO_GPS_RXD_AF));
		s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_TXD, S3C_GPIO_SFN(GPIO_GPS_TXD_AF));
		s3c_gpio_setpull(GPIO_GPS_TXD, S3C_GPIO_PULL_NONE);
		break;
	case 3:
		s3c_gpio_cfgpin(GPIO_FLM_RXD, S3C_GPIO_SFN(GPIO_FLM_RXD_AF));
		s3c_gpio_setpull(GPIO_FLM_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_FLM_TXD, S3C_GPIO_SFN(GPIO_FLM_TXD_AF));
		s3c_gpio_setpull(GPIO_FLM_TXD, S3C_GPIO_PULL_NONE);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(s3c_setup_uart_cfg_gpio);
