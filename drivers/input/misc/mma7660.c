/*
 * mma7660.c
 *
 * Copyright 2009 Freescale Semiconductor Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/autoconf.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/input-polldev.h>
#include <linux/input.h>
#include <linux/hwmon.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>


#define MMA7660_I2C_ADDR	0x4C
#define ONEGVALUE		21
#define DEVICE_NAME		"mma7660"
#define ABS_DEVICE_NAME		"mma7660abs"
#define POLL_INTERVAL		20
#define RES_BUFF_LEN		160
#define ATKBD_SPECIAL		248
#define	AXIS_MAX		(ONEGVALUE/2)

#define ATKBD_SPECIAL		248
#define LSB_G			32f/1.5f

#define DRV_VERSION		"0.0.6"

enum {
	AXIS_DIR_X	= 0,
	AXIS_DIR_X_N,
	AXIS_DIR_Y,
	AXIS_DIR_Y_N,
	AXIS_DIR_Z,
	AXIS_DIR_Z_N,
};

enum {
	SLIDER_X_UP	= 0,
	SLIDER_X_DOWN	,
	SLIDER_Y_UP	,
	SLIDER_Y_DOWN	,
};

static char *axis_dir[6] = {
	[AXIS_DIR_X] 		= "x",
	[AXIS_DIR_X_N] 		= "-x",
	[AXIS_DIR_Y]		= "y",
	[AXIS_DIR_Y_N]		= "-y",
	[AXIS_DIR_Z]		= "z",
	[AXIS_DIR_Z_N]		= "-z",
};

enum {
	REG_XOUT = 0x00,
	REG_YOUT,
	REG_ZOUT,
	REG_TILT,
	REG_SRST,
	REG_SPCNT,
	REG_INTSU,
	REG_MODE,
	REG_SR,
	REG_PDET,
	REG_PD,
	REG_END,
};

enum {
	STANDBY_MODE = 0,
	ACTIVE_MODE,
};


enum {
	INT_1L_2P = 0,
	INT_1P_2L,
	INT_1SP_2P,
};

struct mma7660_status {
	u8 mode;
	u8 ctl1;
	u8 ctl2;
	u8 axis_dir_x;
	u8 axis_dir_y;
	u8 axis_dir_z;
	u8 slider_key[4];
};


static unsigned char atkbd_keycode[512] = {
	  0, 67, 65, 63, 61, 59, 60, 88,  0, 68, 66, 64, 62, 15, 41,117,
	  0, 56, 42, 93, 29, 16,  2,  0,  0,  0, 44, 31, 30, 17,  3,  0,
	  0, 46, 45, 32, 18,  5,  4, 95,  0, 57, 47, 33, 20, 19,  6,183,
	  0, 49, 48, 35, 34, 21,  7,184,  0,  0, 50, 36, 22,  8,  9,185,
	  0, 51, 37, 23, 24, 11, 10,  0,  0, 52, 53, 38, 39, 25, 12,  0,
	  0, 89, 40,  0, 26, 13,  0,  0, 58, 54, 28, 27,  0, 43,  0, 85,
	  0, 86, 91, 90, 92,  0, 14, 94,  0, 79,124, 75, 71,121,  0,  0,
	 82, 83, 80, 76, 77, 72,  1, 69, 87, 78, 81, 74, 55, 73, 70, 99,

	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	217,100,255,  0, 97,165,  0,  0,156,  0,  0,  0,  0,  0,  0,125,
	173,114,  0,113,  0,  0,  0,126,128,  0,  0,140,  0,  0,  0,127,
	159,  0,115,  0,164,  0,  0,116,158,  0,172,166,  0,  0,  0,142,
	157,  0,  0,  0,  0,  0,  0,  0,155,  0, 98,  0,  0,163,  0,  0,
	226,  0,  0,  0,  0,  0,  0,  0,  0,255, 96,  0,  0,  0,143,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,107,  0,105,102,  0,  0,112,
	110,111,108,112,106,103,  0,119,  0,118,109,  0, 99,104,119,  0,

	  0,  0,  0, 65, 99,
};


static ssize_t mma7660_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t mma7660_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count);
static int mma7660_suspend(struct i2c_client *client, pm_message_t state);
static int mma7660_resume(struct i2c_client *client);
#if 0
static int mma7660_open(struct input_dev *dev);
static int mma7660_close(struct input_dev *dev);
#endif 
static void mma7660_poll_work(struct work_struct *work);

static struct i2c_driver mma7660_driver;
static int mma7660_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int mma7660_remove(struct i2c_client *client);
static int mma7660_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

static const struct i2c_device_id mma7660_id[] = {
	{"mma7660", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mma7660_id);

static struct i2c_client 	*mma7660_client;
static struct input_polled_dev 	*mma7660_idev;
static struct input_dev		*mma7660_abs_dev;
//static u8			mma_open = 0;
static u8			mma_cal	= 0;
static int 			debug = 0;
//static int 			debug = 1;
static int 			poll_int = POLL_INTERVAL;	
static char			res_buff[RES_BUFF_LEN];
static int 			emu_joystick = 0;

static struct workqueue_struct *mma7660_workqueue;

static DECLARE_WORK(mma7660_work, mma7660_poll_work);

#undef DBG
#define DBG(format, arg...) do { if (debug) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg); } while (0)
//#define DBG //

static unsigned short normal_i2c[] = {MMA7660_I2C_ADDR,I2C_CLIENT_END};

I2C_CLIENT_INSMOD_1(mma7660);


static struct i2c_driver mma7660_driver = {
	.driver 	= {
				.name = "mma7660",
			},
	.class		= I2C_CLASS_HWMON,
	.probe		= mma7660_probe,
	.remove		= mma7660_remove,
	.id_table	= mma7660_id,
	.detect		= mma7660_detect,
	.address_data	= &addr_data,
	.suspend 	= mma7660_suspend,
	.resume 	= mma7660_resume,
};

static struct device_attribute mma7660_dev_attr = {
	.attr 	= {
		 .name = "mma7660_ctl",
		 .mode = S_IRUGO | S_IWUGO,
	},
	.show 	= mma7660_show,
	.store 	= mma7660_store,
};

static struct mma7660_status mma_status = {
	.mode 		= 0,
	.ctl1 		= 0,
	.ctl2 		= 0,
	.axis_dir_x	= AXIS_DIR_X,
	.axis_dir_y	= AXIS_DIR_Y,
	.axis_dir_z	= AXIS_DIR_Z,
	.slider_key	= {0,0,0,0},
};

enum {
	MMA_REG_R = 0,		/* read register 	*/
	MMA_REG_W,		/* write register 	*/
	MMA_DUMPREG,		/* dump registers	*/
	MMA_REMAP_AXIS,		/* remap directiron of 3-axis	*/
	MMA_DUMPAXIS,		/* dump current axis mapping	*/
	MMA_ENJOYSTICK,		/* enable joystick	emulation	*/
	MMA_DISJOYSTICK,	/* disable joystick emulation	*/
	MMA_SLIDER_KEY,		/* set slider key		*/
	MMA_DUMP_SLKEY,		/* dump slider key code setting	*/
	MMA_CMD_MAX
};

static char *command[MMA_CMD_MAX] = {
	[MMA_REG_R] 		= "readreg",
	[MMA_REG_W] 		= "writereg",
	[MMA_DUMPREG]		= "dumpreg",
	[MMA_REMAP_AXIS]	= "remapaxis",
	[MMA_DUMPAXIS]		= "dumpaxis",
	[MMA_ENJOYSTICK]	= "enjoystick",
	[MMA_DISJOYSTICK]	= "disjoystick",
	[MMA_SLIDER_KEY]	= "sliderkey",
	[MMA_DUMP_SLKEY]	= "dumpslkey",
};
s32 mma7660_Read_Alert(u8 RegAdd);

s32 mma7660_Read_Alert(u8 RegAdd){
	s32 res;

 	do{
		res = i2c_smbus_read_byte_data(mma7660_client, RegAdd);
		if (res < 0){
			return res;
		}
 	} while (res & 0x40);			// while Alert bit = 1;
	
	return res;
}


/* read sensor data from mma7660 */
static int mma7660_read_data(short *x, short *y, short *z, short *tilt) {
	u8	tmp_data[4];
	s32	res;

	res = (i2c_smbus_read_i2c_block_data(mma7660_client,REG_XOUT,REG_TILT-REG_XOUT,tmp_data) < REG_TILT-REG_XOUT);

	if  (res < 0){
		dev_err(&mma7660_client->dev, "i2c block read failed\n");
			return res;
	}

	*x=tmp_data[0];
	*y=tmp_data[1];
	*z=tmp_data[2];
	*tilt=tmp_data[3];

	if (*x & 0x40){
		res = mma7660_Read_Alert(REG_XOUT);
		if (res < 0){
			return res;
		}
		*x = res;
	}

	if (*y & 0x40){	
		res = mma7660_Read_Alert(REG_YOUT);
		if (res < 0){
			return res;
		}
		*y = res;
	}

	if (*z & 0x40){	
		res = mma7660_Read_Alert(REG_ZOUT);
		if (res < 0){
			return res;
		}
		*z = res;
	}

	if (*tilt * 0x40){
		res = mma7660_Read_Alert(REG_TILT);
		if (res < 0){
			return res;
		}
		*tilt = res;
	}

	if (*x & 0x20) {	/* check for the bit 5 */
		*x |= 0xffc0;
	}

	if (*y & 0x20) {	/* check for the bit 5 */
		*y |= 0xffc0;
	}

	if (*z & 0x20) {	/* check for the bit 5 */
		*z |= 0xffc0;
	}

	return 0;
}


/* parse command argument */
static void parse_arg(const char *arg, int *reg, int *value)
{
	const char *p;

	for (p = arg;; p++) {
		if (*p == ' ' || *p == '\0')
			break;
	}

	p++;

	*reg 	= simple_strtoul(arg, NULL, 16);
	*value 	= simple_strtoul(p, NULL, 16);
}

static void cmd_read_reg(const char *arg)
{
	int reg, value, ret;

	parse_arg(arg, &reg, &value);
	ret = i2c_smbus_read_byte_data(mma7660_client, reg);
	dev_info(&mma7660_client->dev, "read reg0x%x = %x\n", reg, ret);
	sprintf(res_buff,"OK:read reg 0x%02x = 0x%02x\n",reg,ret);
}

/* write register command */
static void cmd_write_reg(const char *arg)
{
	int reg, value, ret, modereg;

	parse_arg(arg, &reg, &value);

	modereg = i2c_smbus_read_byte_data(mma7660_client, REG_MODE);

	ret = i2c_smbus_write_byte_data(mma7660_client, REG_MODE, modereg & (~0x01));
	ret = i2c_smbus_write_byte_data(mma7660_client, reg, value);
	ret = i2c_smbus_write_byte_data(mma7660_client, REG_MODE, modereg);

	dev_info(&mma7660_client->dev, "write reg result %s\n",
		 ret ? "failed" : "success");
	sprintf(res_buff, "OK:write reg 0x%02x data 0x%02x result %s\n",reg,value,
		 ret ? "failed" : "success");
}

/* dump register command */
static void cmd_dumpreg(void) {
	int reg,ret;

	sprintf(res_buff,"OK:dumpreg result:\n");

	for (reg = REG_XOUT; reg <= REG_END; reg++) {
		ret = i2c_smbus_read_byte_data(mma7660_client, reg);
		sprintf(&(res_buff[strlen(res_buff)]),"%02x ",ret);
		if ((reg % 16) == 15) {
			sprintf(&(res_buff[strlen(res_buff)]),"\n");
		}
	}

	sprintf(&(res_buff[strlen(res_buff)]),"\n");
}

/* do the axis translation */
static void remap_axis(short *rem_x,short *rem_y,short *rem_z,short x,short y,short z) {

	switch (mma_status.axis_dir_x) {
		case AXIS_DIR_X:
			*rem_x		= x;
			break;
		case AXIS_DIR_X_N:
			*rem_x		= -x;
			break;
		case AXIS_DIR_Y:
			*rem_x		= y;
			break;
		case AXIS_DIR_Y_N:
			*rem_x		= -y;
			break;
		case AXIS_DIR_Z:
			*rem_x		= z;
			break;
		case AXIS_DIR_Z_N:
			*rem_x		= -z;
			break;
	}
	
	switch (mma_status.axis_dir_y) {
		case AXIS_DIR_X:
			*rem_y		= x;
			break;
		case AXIS_DIR_X_N:
			*rem_y		= -x;
			break;
		case AXIS_DIR_Y:
			*rem_y		= y;
			break;
		case AXIS_DIR_Y_N:
			*rem_y		= -y;
			break;
		case AXIS_DIR_Z:
			*rem_y		= z;
			break;
		case AXIS_DIR_Z_N:
			*rem_y		= -z;
			break;
	}
	
	switch (mma_status.axis_dir_z) {
		case AXIS_DIR_X:
			*rem_z		= x;
			break;
		case AXIS_DIR_X_N:
			*rem_z		= -x;
			break;
		case AXIS_DIR_Y:
			*rem_z		= y;
			break;
		case AXIS_DIR_Y_N:
			*rem_z		= -y;
			break;
		case AXIS_DIR_Z:
			*rem_z		= z;
			break;
		case AXIS_DIR_Z_N:
			*rem_z		= -z;
			break;
	}
}


/* set the Zslider key mapping */
static void cmd_sliderkey(const char *arg) 
{
	unsigned char key_code[4];
	int i;
	char *endptr,*p;

	p = (char *)arg;

	for ( i = 0; i < 4 ; i++ ) {
		if (*p == '\0') {
			break;
		}

		key_code[i] = (char)simple_strtoul(p,&endptr,16);
		p = endptr +1;

	}

	if (i != 4) {
		sprintf (res_buff,"FAIL:sliderkey command failed,only %d args provided\n",i);
		DBG ("%s: Failed to set slider key, not enough key code in command\n",__FUNCTION__);
		return;
	}


	DBG("%s: set slider key code  %02x %02x %02x %02x \n",__FUNCTION__,
		key_code[0],key_code[1],key_code[2],key_code[3]);


	for (i = 0; i < 4; i++) {
		mma_status.slider_key[i] = key_code[i];
	}

	sprintf(res_buff,"OK:set slider key ok, key code %02x %02x %02x %02x \n",
		key_code[0],key_code[1],key_code[2],key_code[3]);
}

/* remap the axis */
static void cmd_remap_axis(const char *arg,size_t count) 
{
	int 	buff_len; 	
	char	delimiters[] = " ,";

	char 	*token = NULL;
	
	int 	axis_cnt = 0;
	u8	axis_map[3];
	
	if (count > 63) {
		buff_len = 63;
	} else {
		buff_len = count;
	}

	while ((token = strsep((char **)&arg,delimiters)) != NULL) {

		if (strcmp(token,"")) {
			if (strcasecmp(token,"x") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_X;
			} else if (strcasecmp(token,"-x") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_X_N;
			} else if (strcasecmp(token,"y") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Y;
			} else if (strcasecmp(token,"-y") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Y_N;
			} else if (strcasecmp(token,"z") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Z;
			} else if (strcasecmp(token,"-z") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Z_N;
			} else {
				sprintf (res_buff,"FAIL:remapaxis error,wrong argument\n");
				return;
			}

			axis_cnt ++;

			if (axis_cnt == 3) {
				break;
			}
		}
	}

	if (axis_cnt != 3) {
		sprintf(res_buff,"FAIL:remapaxis error, not enough parameters(%d)\n",axis_cnt);
		return;
	}


	if (((axis_map[0] & 0xfe) == (axis_map[1] & 0xfe)) || 
		((axis_map[0] & 0xfe) == (axis_map[2] & 0xfe)) ||
		((axis_map[2] & 0xfe) == (axis_map[1] & 0xfe))) {

		sprintf(res_buff,"FAIL:remapaxis error, duplicate axis mapping\n");
		return;
	}


	mma_status.axis_dir_x	= axis_map[0];
	mma_status.axis_dir_y	= axis_map[1];
	mma_status.axis_dir_z	= axis_map[2];

	sprintf(res_buff,"OK:remapaxis ok, new mapping : %s %s %s\n",axis_dir[mma_status.axis_dir_x],
		axis_dir[mma_status.axis_dir_y],axis_dir[mma_status.axis_dir_z]);
}

/* parse the command passed into driver, and execute it */
static int exec_command(const char *buf, size_t count)
{
	const char *p, *s;
	const char *arg;
	int i, value = 0;

	for (p = buf;; p++) {
		if (*p == ' ' || *p == '\0' || p - buf >= count)
			break;
	}
	arg = p + 1;

	for (i = MMA_REG_R; i < MMA_CMD_MAX; i++) {
		s = command[i];
		if (s && !strncmp(buf, s, p - buf - 1)) {
			dev_info(&mma7660_client->dev, "command %s\n", s);
			goto mma_exec_command;
		}
	}

	dev_err(&mma7660_client->dev, "command not found\n");
	sprintf(res_buff,"FAIL:command [%s] not found\n",s);
	return -1;

mma_exec_command:
	if (i != MMA_REG_R && i != MMA_REG_W)
		value = simple_strtoul(arg, NULL, 16);

	switch (i) {
	case MMA_REG_R:
		cmd_read_reg(arg);
		break;
	case MMA_REG_W:
		cmd_write_reg(arg);
		break;
	case MMA_DUMPREG:		/* dump registers	*/
		cmd_dumpreg();
		break;
	case MMA_REMAP_AXIS:		/* remap 3 axis's direction	*/
		cmd_remap_axis(arg,(count - (arg - buf)));
		break;
	case MMA_DUMPAXIS:		/* remap 3 axis's direction	*/
		sprintf(res_buff,"OK:dumpaxis : %s %s %s\n",axis_dir[mma_status.axis_dir_x],
			axis_dir[mma_status.axis_dir_y],axis_dir[mma_status.axis_dir_z]);
		break;
	case MMA_ENJOYSTICK: 
		emu_joystick = 1;
		sprintf(res_buff,"OK:joystick movement emulation enabled\n");
		break;
	case MMA_DISJOYSTICK:
		emu_joystick = 0;
		sprintf(res_buff,"OK:joystick movement emulation disabled\n");
		break;
	case MMA_SLIDER_KEY:		/* load offset drift registers	*/
		cmd_sliderkey(arg);
		break;
	case MMA_DUMP_SLKEY:
		sprintf(res_buff,"OK:dumpslkey 0x%02x 0x%02x 0x%02x 0x%02x\n",mma_status.slider_key[0],
			mma_status.slider_key[1],mma_status.slider_key[2],mma_status.slider_key[3]);
		break;
	default:
		dev_err(&mma7660_client->dev, "command is not found\n");
		sprintf (res_buff,"FAIL:no such command %d\n",i);
		break;
	}

	return 0;
}

/* show the command result */
static ssize_t mma7660_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s",res_buff);
}

/* accept the command */
static ssize_t mma7660_store(struct device *dev,struct device_attribute *attr, 
		const char *buf,size_t count)
{
	exec_command(buf, count);

	return count;
}


typedef enum {NOSLIDE, UPWARD, DOWNWARD} SLIDEOUT;

#define NOSLIDE		0
#define SLIDEUPWARD	1
#define SLIDEUPWARD2	2
#define SLIDEDOWNWARD 	3
#define SLIDEDOWNWARD2	4

#define MAXSLIDEWIDTH 	10
#define SLIDERAVCOUNT 	4

#define SLIDE_THR	16
#define SLIDE_THR2	(36)

struct slider {
	int 		wave[SLIDERAVCOUNT];	/* for long term average */
	unsigned char	wave_idx;
	short		ref;
	unsigned char 	dir;
	unsigned char	report;
	unsigned char 	mode;
	unsigned char	cnt;	
};

static struct slider zsl_x,zsl_y,zsl_z;

/* check for the zslider event */
static int zslider(struct slider *slider, short axis) {
	int i;
	short average = 0;	

	slider->wave[slider->wave_idx] 	= axis;

	for (i = 0; i < SLIDERAVCOUNT; i++) 
		average += slider->wave[i];

	average /= SLIDERAVCOUNT;

	switch (slider->mode) {
		case 0:
			if (slider->wave_idx == (SLIDERAVCOUNT - 1)) 
				slider->mode = 1;
			break;
		case 1:
			if (axis > (average + SLIDE_THR)) {	/* slide up trigger 		*/
				slider->dir	= SLIDEUPWARD;
				slider->mode 	= 2;
				slider->cnt	= MAXSLIDEWIDTH;
				slider->ref	= average;
			} else if (axis < (average - SLIDE_THR)) {	/* slide down trigger 		*/
				slider->dir	= SLIDEDOWNWARD;
				slider->mode 	= 2;
				slider->cnt	= MAXSLIDEWIDTH;
				slider->ref	= average;
			}
			slider->report = NOSLIDE;
			break;
		case 2: 
			slider->cnt--;

			if (!slider->cnt) {				/* return to normal */
				slider->mode	= 1; 	
				slider->dir	= NOSLIDE;
				break;
			}

			if ((axis < (slider->ref + SLIDE_THR)) && (axis > (slider->ref - SLIDE_THR)) && (slider->cnt < MAXSLIDEWIDTH-2)) {
/* report the slide event */
				switch (slider->dir) {
					case SLIDEUPWARD:
						DBG("slide upward\n");
						break;
					case SLIDEDOWNWARD:
						DBG("slide downward\n");
						break;
				}

				slider->mode 	= 3;
				slider->cnt	= 5;    		/* add additonal delay */
				slider->report	= slider->dir; 
			}
			break;

		case 3:
			slider->report = NOSLIDE;
			slider->cnt --;
			if (!slider->cnt) {
				slider->mode = 1;
			}
			break;
		default:
			break;
	}

	slider->wave_idx++;
	if (slider->wave_idx == SLIDERAVCOUNT) 
		slider->wave_idx=0;

        return slider->report;
}

/* workqueue function to poll mma7660 data */
static void mma7660_poll_work(struct work_struct *work)  
{
	short 	x,y,z,tilt;
	short 	x_remap,y_remap,z_remap;
	u8 	zslide_x,zslide_y;
//	int	swing;

	if (mma7660_read_data(&x,&y,&z,&tilt) != 0) {
		DBG("mma7660 data read failed\n");
		return;
	}

/* report the absulate sensor data to input device */
	input_report_abs(mma7660_abs_dev, ABS_X, x);	
	input_report_abs(mma7660_abs_dev, ABS_Y, y);	
	input_report_abs(mma7660_abs_dev, ABS_Z, z);
	input_report_abs(mma7660_abs_dev, ABS_MISC, tilt);
	input_sync(mma7660_abs_dev);

	remap_axis(&x_remap,&y_remap,&z_remap,x,y,z);

/* report joystick movement */

	if (emu_joystick)  {	
		input_report_abs(mma7660_idev->input, ABS_X, x_remap);
		input_report_abs(mma7660_idev->input, ABS_Y, y_remap);
		input_report_abs(mma7660_idev->input, ABS_Z, z_remap-ONEGVALUE);
		input_sync(mma7660_idev->input);
	}


/* check the zslider event */

	zslide_x 	= zslider(&zsl_x,x_remap);
	zslide_y	= zslider(&zsl_y,y_remap);
	
	switch (zslide_x) {
		case SLIDEUPWARD: 
			DBG("X report slide upward \n");
			if (mma_status.slider_key[SLIDER_X_UP]) {
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_X_UP], 1);
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_X_UP], 0);
			}
			break;
		case SLIDEDOWNWARD:
			DBG("X report slide downward \n");
			if (mma_status.slider_key[SLIDER_X_DOWN]) {
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_X_DOWN], 1);
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_X_DOWN], 0);
			}
			break;
	}
	
	switch (zslide_y) {
		case SLIDEUPWARD: 
			DBG("Y report slide upward \n");
			if (mma_status.slider_key[SLIDER_Y_UP]) {
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_Y_UP], 1);
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_Y_UP], 0);
			}
			break;
		case SLIDEDOWNWARD:
			DBG("Y report slide downward \n");
			if (mma_status.slider_key[SLIDER_Y_DOWN]) {
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_Y_DOWN], 1);
				input_report_key(mma7660_idev->input, mma_status.slider_key[SLIDER_Y_DOWN], 0);
			}
			break;
	}
	

	if ((zslide_x) || (zslide_y)) {
		input_sync(mma7660_idev->input);
	};

	return;

}

 
/* polling callback function of input polled device */
static void mma7660_dev_poll(struct input_polled_dev *dev)
{
	if (mma_cal) {
		DBG("mma7660 is now doing calibration\n");
		return;
	}

/* Since the the data reading would take a long time, 
   schedule the real work to a workqueue */
	queue_work(mma7660_workqueue, &mma7660_work);
	return;
}

#if 0
static int mma7660_open(struct input_dev *dev) {
	mma_open = 1;

/* init slider structure */

	DBG("mma7660_open() called\n");
/* reset the zslider structure */
	memset(&zsl_x,0,sizeof(struct slider));
	memset(&zsl_y,0,sizeof(struct slider));
	memset(&zsl_z,0,sizeof(struct slider));

	return 0;
}

static int mma7660_close(struct input_dev *dev) {
	mma_open = 0;
	return 0;
}

#endif 
/* detecting mma7660 chip */
static int mma7660_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
        struct i2c_adapter *adapter = client->adapter;

        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE
                                     | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
                return -ENODEV;                                        

       strlcpy(info->type, "mma7660", I2C_NAME_SIZE);

        return 0;
}

static int mma7660_probe(struct i2c_client *client,const struct i2c_device_id *id) {
	struct input_dev 	*idev;
	int 			rc; 
	int 			ret;
	int 			i;

	printk(KERN_INFO "try to detect mma7455 chip id %02x\n",client->addr);
	mma7660_workqueue = create_singlethread_workqueue("mma7660");
        if (!mma7660_workqueue) {
	        return -ENOMEM; /* most expected reason */
        }

	printk(KERN_INFO "1 try to create file\n");


	mma7660_client = client;

	printk(KERN_INFO "2 try to create file\n");

/* create device file in sysfs as user interface */
	ret = device_create_file(&client->dev, &mma7660_dev_attr);
	printk(KERN_INFO "create file\n");
	if (ret) {
		printk(KERN_INFO "create file failed\n");	
		dev_err(&client->dev, "create device file failed!\n");
		rc = -EINVAL;
		goto err_detach_client;
	}

	printk(KERN_INFO "3 try to create file\n");


/* allocate & register input polling device  */
	mma7660_idev = input_allocate_polled_device();
	if (!mma7660_idev) {
		dev_err(&client->dev, "allocate poll device failed!\n");
		rc = -ENOMEM;
		goto err_free_input;
	}

	mma7660_abs_dev = input_allocate_device();

	if (!mma7660_abs_dev) {
		dev_err(&client->dev, "allocate poll device failed!\n");
		rc = -ENOMEM;
		goto err_free_abs;
	}


	mma7660_idev->poll 		= mma7660_dev_poll;
	mma7660_idev->poll_interval 	= poll_int ;

	idev 			= mma7660_idev->input;
	idev->name 		= DEVICE_NAME;
	idev->id.bustype 	= BUS_I2C;
	idev->dev.parent 	= &client->dev;

	set_bit(EV_REL,idev->evbit);
	set_bit(EV_KEY,idev->evbit);

	input_set_abs_params(mma7660_idev->input, ABS_X, -AXIS_MAX, AXIS_MAX, 0, 0);
	input_set_abs_params(mma7660_idev->input, ABS_Y, -AXIS_MAX, AXIS_MAX, 0, 0);
	input_set_abs_params(mma7660_idev->input, ABS_Z, -AXIS_MAX, AXIS_MAX, 0, 0);
	set_bit(EV_ABS, mma7660_idev->input->evbit);

	idev->keycode = atkbd_keycode;
	idev->keycodesize = sizeof(unsigned char);
	idev->keycodemax = ARRAY_SIZE(atkbd_keycode);

	for (i = 0; i < 512; i++)
		if (atkbd_keycode[i] && atkbd_keycode[i] < ATKBD_SPECIAL)
			set_bit(atkbd_keycode[i], idev->keybit);

	mma7660_abs_dev->name		= ABS_DEVICE_NAME;
	mma7660_abs_dev->id.bustype	= BUS_I2C;
	mma7660_abs_dev->dev.parent	= &client->dev;

	set_bit(EV_ABS,mma7660_abs_dev->evbit);
	set_bit(ABS_X,mma7660_abs_dev->absbit);
	set_bit(ABS_Y,mma7660_abs_dev->absbit);
	set_bit(ABS_Z,mma7660_abs_dev->absbit);
	set_bit(ABS_MISC,mma7660_abs_dev->absbit);


	ret = input_register_polled_device(mma7660_idev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		rc = -EINVAL;
		goto err_unreg_input;
	}


	ret = input_register_device(mma7660_abs_dev);
	if (ret) {
		dev_err(&client->dev, "register abs device failed!\n");
		rc = -EINVAL;
		goto err_unreg_abs_input;
	}

	memset(&zsl_x,0,sizeof(struct slider));
	memset(&zsl_y,0,sizeof(struct slider));
	memset(&zsl_z,0,sizeof(struct slider));
	memset(res_buff,0,RES_BUFF_LEN);

/* enable gSensor mode 8g, measure */
	i2c_smbus_write_byte_data(client, REG_MODE, 0x00);		/* standby mode   */
	i2c_smbus_write_byte_data(client, REG_SPCNT, 0x00);		/* no sleep count */
	i2c_smbus_write_byte_data(client, REG_INTSU, 0x00);		/* no interrupt   */
	i2c_smbus_write_byte_data(client, REG_PDET, 0xE0);		/* disable tap    */
	i2c_smbus_write_byte_data(client, REG_SR, 0x6B);		/* 4 measurement, 16 sample  */
	i2c_smbus_write_byte_data(client, REG_MODE, 0x01);		/* active mode   */

	return 0;
err_unreg_abs_input:
	input_unregister_device(mma7660_abs_dev);
err_unreg_input:
	input_unregister_polled_device(mma7660_idev);
err_free_abs:
	input_free_device(mma7660_abs_dev);
err_free_input:
	input_free_polled_device(mma7660_idev);

err_detach_client:
	destroy_workqueue(mma7660_workqueue);

	kfree(client);
	return rc; 
}


static int mma7660_remove(struct i2c_client *client)  {

	input_unregister_device(mma7660_abs_dev);
	input_unregister_polled_device(mma7660_idev);
	input_free_device(mma7660_abs_dev);
	input_free_polled_device(mma7660_idev);

	device_remove_file(&client->dev, &mma7660_dev_attr);

	kfree(i2c_get_clientdata(client));
	flush_workqueue(mma7660_workqueue);
	destroy_workqueue(mma7660_workqueue);

	DBG("MMA7660 device detatched\n");	
        return 0;
}


static int mma7660_suspend(struct i2c_client *client, pm_message_t state)
{
	mma_status.mode = i2c_smbus_read_byte_data(mma7660_client, REG_MODE);
	i2c_smbus_write_byte_data(mma7660_client, REG_MODE,mma_status.mode & ~0x3);
	DBG("MMA7660 suspended\n");
	return 0;
}

static int mma7660_resume(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(mma7660_client, REG_MODE, mma_status.mode);
	DBG("MMA7660 resumed\n");
	return 0;
}


static int __init init_mma7660(void)
{
/* register driver */
	int res;

	res = i2c_add_driver(&mma7660_driver);
	if (res < 0){
		printk(KERN_INFO "add mma7660 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add mma7660 i2c driver\n");

	return (res);
}

static void __exit exit_mma7660(void)
{
	printk(KERN_INFO "remove mma7660 i2c driver.\n");
	return i2c_del_driver(&mma7660_driver);
}


module_init(init_mma7660);
module_exit(exit_mma7660);

module_param(debug, bool, S_IRUGO | S_IWUSR);
module_param(emu_joystick, bool, S_IRUGO | S_IWUSR);
module_param(poll_int, int, S_IRUGO | S_IWUSR);

MODULE_PARM_DESC(debug, "1: Enable verbose debugging messages");
MODULE_PARM_DESC(emu_joystick, "1: Enable emulate joystick movement by tilt");
MODULE_PARM_DESC(poll_int, "set the poll interval of gSensor data (unit: ms)");

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA7660 sensor driver");
MODULE_LICENSE("GPL");
