/*
 * TL2796 LCD Panel Driver for the Samsung Universal board
 *
 * Copyright (c) 2008 Samsung Electronics
 * Author: InKi Dae  <inki.dae@samsung.com>
 *
 * Derived from drivers/video/omap/lcd-apollon.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

#include <plat/gpio-cfg.h>
#include <plat/regs-fb.h>

#include "s3cfb.h"

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00
#define COMMAND_ONLY		0xFE
#define DATA_ONLY		0xFF

#ifdef CONFIG_BACKLIGHT_TL2796_AMOLED

//define dbg printk
#define dbg(fmt...)

static int locked = 0;
struct s5p_lcd{
	struct spi_device *g_spi;
	struct lcd_device *lcd_dev;
	struct backlight_device *bl_dev;

};
static struct s5p_lcd lcd;
#else
static struct spi_device *g_spi;
#endif //CONFIG_BACKLIGHT_TL2796_AMOLED

const unsigned short SEQ_DISPLAY_ON[] = {
	0x14, 0x03,

	ENDDEF, 0x0000
};

const unsigned short SEQ_DISPLAY_OFF[] = {
	0x14, 0x00,

	ENDDEF, 0x0000
};

const unsigned short SEQ_STANDBY_ON[] = {
	0x1D, 0xA1,
	SLEEPMSEC, 200,

	ENDDEF, 0x0000
};

const unsigned short SEQ_STANDBY_OFF[] = {
	0x1D, 0xA0,
	SLEEPMSEC, 250,

	ENDDEF, 0x0000
};

#ifndef CONFIG_UNIVERSAL_S5PC110_VER2 /* smdkc110, Universal 1'st version */
const unsigned short SEQ_SETTING[] = {
	0x31, 0x08, /* panel setting */
	0x32, 0x14,
	0x30, 0x02,
	0x27, 0x03, /* 0x27, 0x01 */
	0x12, 0x08,
	0x13, 0x08,
	0x15, 0x10, /* 0x15, 0x00 */
	0x16, 0x00, /* 24bit line and 16M color */

	0xef, 0xd0, /* pentile key setting */
	DATA_ONLY, 0xe8,

	0x39, 0x44,
	0x40, 0x00, 
	0x41, 0x3f, 
	0x42, 0x2b, 
	0x43, 0x1f, 
	0x44, 0x24, 
	0x45, 0x1b, 
	0x46, 0x29, 
	0x50, 0x00, 
	0x51, 0x00, 
	0x52, 0x00, 
	0x53, 0x1b, 
	0x54, 0x22, 
	0x55, 0x1b, 
	0x56, 0x2a, 
	0x60, 0x00, 
	0x61, 0x3f, 
	0x62, 0x25, 
	0x63, 0x1c, 
	0x64, 0x21, 
	0x65, 0x18, 
	0x66, 0x3e, 

	0x17, 0x22,	//Boosting Freq
	0x18, 0x33,	//power AMP Medium
	0x19, 0x03,	//Gamma Amp Medium
	0x1a, 0x01,	//Power Boosting 
	0x22, 0xa4,	//Vinternal = 0.65*VCI
	0x23, 0x00,	//VLOUT1 Setting = 0.98*VCI
	0x26, 0xa0,	//Display Condition LTPS signal generation : Reference= DOTCLK

	0x1d, 0xa0,
	SLEEPMSEC, 300,

	0x14, 0x03,

	ENDDEF, 0x0000
};
#else /* universal */
const unsigned short SEQ_SETTING[] = {
	0x31, 0x08, /* panel setting */
	0x32, 0x14,
	0x30, 0x02,
	0x27, 0x01,
	0x12, 0x08,
	0x13, 0x08,
	0x15, 0x00,
	0x16, 0x00, /* 24bit line and 16M color */

	0xEF, 0xD0, /* pentile key setting */
	DATA_ONLY, 0xE8,

	0x39, 0x44, /* gamma setting */
	0x40, 0x00,
	0x41, 0x00,
	0x42, 0x00,
	0x43, 0x19,
	0x44, 0x19,
	0x45, 0x16,
	0x46, 0x34,

	0x50, 0x00,
	0x51, 0x00,
	0x52, 0x00,
	0x53, 0x16,
	0x54, 0x19,
	0x55, 0x16,
	0x56, 0x39,

	0x60, 0x00,
	0x61, 0x00,
	0x62, 0x00,
	0x63, 0x17,
	0x64, 0x18,
	0x65, 0x14,
	0x66, 0x4a,

	0x17, 0x22, /* power setting */
	0x18, 0x33,
	0x19, 0x03,
	0x1A, 0x01,
	0x22, 0xA4,
	0x23, 0x00,
	0x26, 0xA0,

	ENDDEF, 0x0000
};
#endif

/* FIXME: will be moved to platform data */
static struct s3cfb_lcd tl2796 = {
	.width = 480,
	.height = 800,
	.bpp = 24,

#if 1
	/* smdkc110, For Android */
	.freq = 60,
	.timing = {
		.h_fp = 9,
		.h_bp = 9,
		.h_sw = 2,
		.v_fp = 5,
		.v_fpe = 1,
		.v_bp = 5,
		.v_bpe = 1,
		.v_sw = 2,
	},
#else	
	/* universal */
	.freq = 60,
	.timing = {
		.h_fp = 66,
		.h_bp = 2,
		.h_sw = 4,
		.v_fp = 15,
		.v_fpe = 1,
		.v_bp = 3,
		.v_bpe = 1,
		.v_sw = 5,
	},
#endif
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 1,
	},
};


static int tl2796_spi_write_driver(int addr, int data)
{
	u16 buf[1];
	struct spi_message msg;
	int ret;

	struct spi_transfer xfer = {
		.len	= 2,
		.tx_buf	= buf,
	};

	buf[0] = (addr << 8) | data;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

#ifdef CONFIG_BACKLIGHT_TL2796_AMOLED
	locked  = 1;
	ret = spi_sync(lcd.g_spi, &msg);
	locked = 0;

	return ret ;
#else
	return spi_sync(g_spi, &msg);
#endif //CONFIG_BACKLIGHT_TL2796_AMOLED
}


static void tl2796_spi_write(unsigned char address, unsigned char command)
{
	if (address != DATA_ONLY)
		tl2796_spi_write_driver(0x70, address);

	tl2796_spi_write_driver(0x72, command);
}

static void tl2796_panel_send_sequence(const unsigned short *wbuf)
{
	int i = 0;

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC)
			tl2796_spi_write(wbuf[i], wbuf[i+1]);
		else
			msleep(wbuf[i+1]);
			//mdelay(wbuf[i+1]);
		i += 2;
	}
}

void tl2796_ldi_init(void)
{
	tl2796_panel_send_sequence(SEQ_SETTING);
	tl2796_panel_send_sequence(SEQ_STANDBY_OFF);
}

void tl2796_ldi_enable(void)
{
	tl2796_panel_send_sequence(SEQ_DISPLAY_ON);
	/* Added by james */
	tl2796_panel_send_sequence(SEQ_STANDBY_OFF);
	
}

void tl2796_ldi_disable(void)
{
	tl2796_panel_send_sequence(SEQ_DISPLAY_OFF);
	/* Added by james for fixing LCD suspend problem */
	tl2796_panel_send_sequence(SEQ_STANDBY_ON);
	
}

void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	tl2796.init_ldi = NULL;
	ctrl->lcd = &tl2796;
}

//backlight operations and functions
#ifdef CONFIG_BACKLIGHT_TL2796_AMOLED
static int s5p_bl_update_status(struct backlight_device* bd)
{
	int bl = bd->props.brightness;
	int level = 0;	
	dbg("\nUpdate brightness=%d\n",bd->props.brightness);
	
	if(!locked)
        {
		if((bl >= 0) && (bl <= 80))
                        level = 1;
                else if((bl > 80) && (bl <= 100))
                        level = 2;
                else if((bl > 100) && (bl <= 150))
                        level = 3;
                else if((bl > 150) && (bl <= 180))
			level = 4;
                else if((bl > 180) && (bl <= 200))
                        level = 5;
                else if((bl > 200) && (bl <= 255))
                        level = 6;
		
		if(level){
			tl2796_spi_write(0x39,((bl/64 +1)<<4)&(bl/64+1));	

			switch (level){
				/*If bl is not halved, variation in brightness is observed as a curve with 
 		 		the middle region being brightest and the sides being darker. It is required
				that brightness increases gradually from left to right.*/
				case 1:			
					tl2796_spi_write(0x46, (bl/2)+6); //R
        		        	tl2796_spi_write(0x56, (bl/2)+4); //G
                			tl2796_spi_write(0x66, (bl/2)+30); //B
					break;
				case 2:			
					tl2796_spi_write(0x46, (bl/2)+4); //R
        		        	tl2796_spi_write(0x56, (bl/2)+2); //G
                			tl2796_spi_write(0x66, (bl/2)+28); //B
					break;
				case 3:			
					tl2796_spi_write(0x46, (bl/2)+6); //R
        		        	tl2796_spi_write(0x56, (bl/2)+1); //G
                			tl2796_spi_write(0x66, (bl/2)+32); //B
					break;
				case 4:			
					tl2796_spi_write(0x46, (bl/2)+6); //R
        		        	tl2796_spi_write(0x56, (bl/2)+1); //G
                			tl2796_spi_write(0x66, (bl/2)+38); //B
					break;
				case 5:
					tl2796_spi_write(0x46, (bl/2)+7); //R
        	        		tl2796_spi_write(0x56, (bl/2)); //G
                			tl2796_spi_write(0x66, (bl/2)+40); //B
					break;
				case 6:
					tl2796_spi_write(0x46, (bl/2)+10); //R
        	        		tl2796_spi_write(0x56, (bl/2)); //G
                			tl2796_spi_write(0x66, (bl/2)+48); //B
					break;
				default:
					break;
			}
		}//level check over
	}else{
		dbg("\nLOCKED!!!Brightness cannot be changed now! locked=%d", locked);
	}
	return 0;
}

static int s5p_bl_get_brightness(struct backlight_device *bd)
{
	dbg("\n reading brightness \n");
	return 0;
}

static struct backlight_ops s5p_bl_ops = {
	.update_status = s5p_bl_update_status,
	.get_brightness = s5p_bl_get_brightness,	
};
#endif //CONFIG_BACKLIGHT_TL2796_AMOLED

static int __init tl2796_probe(struct spi_device *spi)
{
	int ret;

	spi->bits_per_word = 16;
	ret = spi_setup(spi);
#ifdef CONFIG_BACKLIGHT_TL2796_AMOLED
	lcd.g_spi = spi;

	/* The node is named as pwm-backlight even though PWM control is not being done
 	since Eclair interface is looking for "pwm-backlight" for backlight brightness control*/
	lcd.bl_dev = backlight_device_register("pwm-backlight",&spi->dev,&lcd,&s5p_bl_ops);
	lcd.bl_dev->props.max_brightness = 255;

	dev_set_drvdata(&spi->dev,&lcd);
#else
	g_spi = spi;
#endif //CONFIG_BACKLIGHT_TL2796_AMOLED

	tl2796_ldi_init();
	tl2796_ldi_enable();

	if (ret < 0)
		return 0;

	return ret;
}
#ifdef CONFIG_PM // add by ksoo (2009.09.07)
int tl2796_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("++++++++[ksoo] tl2796_suspend\n");
	tl2796_ldi_disable();
	return 0;
}

int tl2796_resume(struct platform_device *pdev, pm_message_t state)
{
	printk("++++++++[ksoo] tl2796_resume\n");
	tl2796_ldi_init();
	tl2796_ldi_enable();

	return 0;
}
#endif

static struct spi_driver tl2796_driver = {
	.driver = {
		.name	= "tl2796",
		.owner	= THIS_MODULE,
	},
	.probe		= tl2796_probe,
	.remove		= __exit_p(tl2796_remove),
	.suspend	= NULL,
	.resume		= NULL,
};

static int __init tl2796_init(void)
{
	return spi_register_driver(&tl2796_driver);
}

static void __exit tl2796_exit(void)
{
	spi_unregister_driver(&tl2796_driver);
}

module_init(tl2796_init);
module_exit(tl2796_exit);
