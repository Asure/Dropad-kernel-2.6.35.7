/* drivers/input/misc/bma150.c
 *
 * Tri-axial acceleration sensor(BMA150) driver
 *
 * Copyright (C) 2008 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <asm/io.h>
#include <linux/delay.h>//add to test //diyu@lge.com
#include <asm/gpio.h>//add to test //diyu@lge.com
#include <mach/vreg.h> //LGE_CHANGE [diyu@lge.com] To set vreg

//#include <linux/bma150.h>
//#define USE_HRTIMER
#define __BMA150_H

#define ACCEL_IO_TYPE	'B'

#define ACCEL_IOC_ENABLE	_IO(ACCEL_IO_TYPE, 1)
#define ACCEL_IOC_DISABLE	_IO(ACCEL_IO_TYPE, 2)
#define ACCEL_IOCS_SAMPLERATE	_IOW(ACCEL_IO_TYPE, 3, int) /* in ms */
#define ACCEL_IOCG_SAMPLERATE	_IO(ACCEL_IO_TYPE, 4) /* in ms */


#define DUMP_REGS		1
#define USE_IRQ			1
#define DEF_SAMPLE_RATE		20000000 /* 20 ms */
#if 1
/* Miscellaneous device */
#define MISC_DEV_NAME		"bma150" /*"mma7660"*/
#else 
/* Miscellaneous device */
#define MISC_DEV_NAME		"mma7660"
#endif

/* BMA150 specific definitions */
#define DEV_CHIP_ID		0x02

/* sensor IDs */
#define EVENT_TYPE_ACCEL_X	ABS_X
#define EVENT_TYPE_ACCEL_Y	ABS_Y
#define EVENT_TYPE_ACCEL_Z	ABS_Z
#define EVENT_TYPE_ACCLE_STATUS	ABS_WHEEL

#define EVENT_TYPE_TEMPERATURE	ABS_THROTTLE

#define ACCEL_THRES		9 /*10*/
#define GPIO_MOTION_IRQ 49

static int mma7660_enabled = 1; /*0*/

static struct workqueue_struct *accelerometer_wq;

struct mma7660_device {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;
#ifdef USE_HRTIMER
	struct hrtimer timer;
#endif
	int coordinate[3];
	int thres;
	int use_irq;
	int enabled;
	int sample_rate;
};

static void *_dev = NULL;

static int  degree_xy[64] = {
	0,
	3  ,  5 , 8  , 11 , 14,
	16 , 19 , 22 , 25 , 28,
	31 , 34 , 38 , 41 , 45,
	49 , 53 , 58 , 63 , 70,
	80 , 90 , 95 , 100,  105 , 
	110 , 115 , 120 , 125 , 130,
	135 , -140 , -135 , -130 , -125,
	-120 , -115 , -110 , -105 , -100,
	-95 , -90, -80, -70, -63,
	-58, -53 , -49, -45, -41,
	-38, -34 , -31, -28, -25,
	-22, -19 , -16, -14, -11,
	-8,  -5  , -3
};

 
//ARRAY_SIZE(multivalue_degree)
static int  degree_z[64] = {
  90,
  87 , 85 ,  82,  79,  76,
  74 , 71 ,  68,  65,  62,
  59 , 56 ,  52,  49,  45,
  41 , 37 ,  32,  27,  20,
  10 , 0  ,  -5 ,  -10,   -15 , 
  -20  , -25  ,  -30 ,  -35, -40  ,
  -45  , 45  ,  40 ,  35,   30, 
  25  , 20  ,  15 ,  10,   5, 
  0  , -5  , -10, -20, -27,
 -32 , -37, -41, -45, -49,
 -52 , -56, -59, -62, -65,
 -68 , -71, -74, -77, -80,
 -82, -85 , -87
};



#if 0 /*DUMP_REGS*/
static void dump_regs(void)
{
	struct mma7660_device *dev = _dev;
	int ret;
	int i;
	int retry;
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);

	if (NULL == dev)
		return;

	dev_info(&dev->client->dev, "DUMP REGISTERS:\n");
	for (i = 0x0; i < 0x16; i++) {
		retry = 3;
retry:
		ret = i2c_smbus_read_byte_data(dev->client, i);
		if (ret < 0 &&  retry <= 0) {
			dev_err(&dev->client->dev, "i2c_smbus_read_byte_data(%x) failed\n", i);
			continue;
		}
		else if (ret < 0 && retry > 0) {
			retry--;
			goto retry;
		}
		printk(KERN_INFO "   reg %02x => %02x\n", i, ret & 0xff);
	}
}
#endif

static unsigned char get_mode(struct i2c_client *client )
{

	u8 v, v1,v2,v3,v4, output;
    output = i2c_smbus_read_word_data( client,0x07);
	
	v1 = (output& 0xff )& 0x03; /* MODE */
	v2 = (output& 0xff )& 0x04; /* TON */
	v3 = (output& 0xff )& 0x20; /* AWE */
	v4 = (output& 0xff )& 0x80; /* ASE  */

	
	/* Front or Back */
		switch(v1){
			case 0x00: 
				/*status*/printk("Unknown condition of front or back \n"); 
				break;
			case 0x01:
				/*status*/printk("lying on its FRONT \n"); 
				break;
			case 0x10:
				/*status*/printk("lying on its BACK \n"); 
				break;
			}
	
		switch(v2){
			case 0x000: 
				/*status*/printk("Unknown up/down/left/right \n"); 
				break;
			case 0x001:
				/*status*/printk("LEFT \n"); 
				break;
			case 0x010:
				/*status*/printk("RIGHT \n"); 
				break;
			case 0x101:
				/*status*/printk("DOWN \n"); 
				break;
			case 0x110:
				/*status*/printk("UP \n"); 
				break;
			
			}
			
		switch(v3){
			case 0x0: 
				/*status*/printk( "NOT pulse  \n"); 
				break;
			case 0x1:
				/*status*/printk("Equipment has not detected a pulse \n"); 
				break;
		}
	
		switch(v4){
			case 0x0: 
				/*status*/printk("NOT Shaking"); 
				break;
			case 0x1:
				/*status*/printk("shaking"); 
				break;
		}
	
		return output;
}


static inline int get_tilt_status(struct i2c_client *client )
{
	int ret, direction;
	u8 v, v1,v2,v3,v4, output;
	
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);
	
	output = i2c_smbus_read_word_data(client,0x03);
	v = (output& 0xff )& 0x40; /* Alert to check*/
	if ( v ==1 ) {
		dev_err(&client->dev, "i2c_smbus_read_word_data failed\n");
		return -EIO;
	}else{
		output = i2c_smbus_read_word_data(client,0x03);
	}
	
	v1 = (output& 0xff )& 0x03; /* Front or Back */
	v2 = (output& 0xff )& 0x1c; /* Up or Down or Left or Right*/
	v3 = (output& 0xff )& 0x20; /* Pulse Read*/
	v4 = (output& 0xff )& 0x80; /* Shake Read */

	/* Front or Back */
	switch(v1){
		case 0x00: 
			/*status*/printk("Unknown condition of front or back \n"); 
		    break;
		case 0x01:
			/*status*/printk("lying on its FRONT \n"); 
		    break;
		case 0x10:
			/*status*/printk("lying on its BACK \n"); 
		    break;
		}

	switch(v2){
		case 0x000: 
			/*status*/printk("Unknown up/down/left/right \n"); 
		    break;
		case 0x001:
			/*status*/printk("LEFT \n"); 
		    break;
		case 0x010:
			/*status*/printk("RIGHT \n"); 
		    break;
		case 0x101:
			/*status*/printk("DOWN \n"); 
		    break;
		case 0x110:
			/*status*/printk("UP \n"); 
		    break;
		
		}
		
	switch(v3){
		case 0x0: 
			/*status*/printk( "NOT pulse  \n"); 
		    break;
		case 0x1:
			/*status*/printk("Equipment has not detected a pulse \n"); 
		    break;
	}

	switch(v4){
		case 0x0: 
			/*status*/printk("NOT Shaking"); 
		    break;
		case 0x1:
			/*status*/printk("shaking"); 
		    break;
	}

	return ret;
}
#if 0 
static ssize_t acc_xyz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_dev *i2c_dev = i2c_dev_get_by_minor(MINOR(dev->devt));
	//struct bma150_device *dev= i2c_dev_get_by_minor(MINOR(dev->devt));
	int c[3];
	int d, i;
	u8	v[]={0x00, 0x01, 0x02};

	for (i = 0; i < 3; i++) {
		
		c[i] = get_acc_xyz(dev->client, v[i] );/*x, y, z -axis output*/
	}	

	return sprintf(buf, "%d\n", c);
}
#endif

//static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, vibrator_enable_show, vibrator_enable_store);
//static DEVICE_ATTR(acc_xyz, S_IRUGO | S_IWUSR, acc_xyz_show, NULL);
#if 1

static inline int get_acc_xyz(struct i2c_client *client, u8 command)
{
	int ret, direction;
	u8 v, output;
	
	/*temp1*/printk("\n\nCheck point : %s\n", __FUNCTION__);
	
	#if 1
	/*LGE_CHANGE_S: diYu@lge.com*/
	/*This is x or y or z -axis to read*/
	output = i2c_smbus_read_word_data(client,command);
	if (output < 0) {
		dev_err(&client->dev, "i2c_smbus_read_word_data failed\n");
		return -EIO;
	}
	#if 0
	if (ret < 0) {
		dev_err(&client->dev, "i2c_smbus_read_word_data failed\n");
		return -EIO;
	}
	#endif 
	v = (output& 0xff )& 0x7f; /* Alert + XOUT[5]+ XOUT[4]+ XOUT[3]+ XOUT[2]+ XOUT[1]+ XOUT[0] */
	/*temp3*///printk("Get_acc_xyz ret1 : axis %x (%d)\n", output, output);

	if( (v & 0x20) == 0x20){
		direction = -1; 	
		ret = (int)( (~v & 0x3f) + 0x01 ) * (-1);
		/*temp3*////printk(" -1 : %08x (%08x)\n", v,  ret );
	}else if( (v & 0x20) == 0x00){
		direction = 1; 
		ret = (int)(v & 0x3f );
		/*temp3*///printk("  1 : %08x (%08x)\n", v,  ret );
	}
	//output = i2c_smbus_read_word_data(client, 0x03);
	
	/*temp3*///printk("0x03 : %x (%d)\n", output,  output );
	
	/*LGE_CHANGE_E: diYu@lge.com*/	
	#endif 
	return ret;
}
#endif

static void mma7660_work_func(struct work_struct *work)
{
	struct mma7660_device *dev = container_of(work, struct mma7660_device, work);
	int i;
	int c[3];
	int d;
	int mtrans;
	int do_report = 0;
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);
	u8	v[]={0x00, 0x01, 0x02};

	for (i = 0; i < 3; i++) {
		
		c[i] = get_acc_xyz(dev->client, v[i] );/*x, y, z -axis output*/
		#if 1
		#if 0
		if (c[i] < 0)
			return;

		c[i] -= 0x200;
		#endif
		d = dev->coordinate[i] - c[i];
		
		if (d < 0) d = -d;
				
		if (d >= dev->thres){
			do_report = 1;
		}		
		#endif
	}	

	/*temp3*///printk("diyu Accelometer- c1: %d, c2: %d, c3: %d \n", c[0], c[1], c[2]);
	if (do_report) {

		for (i = 0; i < 3; i++) {
			//input_report_abs(dev->input_dev, ABS_X+i, c[i]);
			//input_report_abs(dev->input_dev, ABS_RX+i, c[i]);\
			mtrans = c[i]
			if(c[i]<0){
				mtrans = 64 + c[i];
			}else
				mtrans = c[i];

				
			//ret = i2c_smbus_read_byte_data(client, 0x00);
#if 1
			if(i == 0) /*x*/
				input_report_abs(dev->input_dev, ABS_RX+i, degree_xy[mtrans]);
			if(i == 1) /*y*/
				input_report_abs(dev->input_dev, ABS_RX+i, -degree_xy[mtrans]);
			if(i == 2) /*z*/
				input_report_abs(dev->input_dev, ABS_RX+i,  degree_z[mtrans]);
#endif
						
			#if 0 
			if(i==2)/*z-axis*/
				input_report_abs(dev->input_dev, ABS_RX+0, c[i]);
			if(i==0)/*x-axis*/
				input_report_abs(dev->input_dev, ABS_RX+1, c[i]);
			if(i==1)/*y-axis*/
				input_report_abs(dev->input_dev, ABS_RX+2, c[i]);
			#endif 
		}
		input_sync(dev->input_dev);
		

		for (i = 0; i < 3; i++) {
			dev->coordinate[i] = c[i];
		}
		//printk("diyu  dev->client->dev (x, y ,z) = (%d, %d, %d)\n", c[0], c[1], c[2]);

		//dev_info(&dev->client->dev, "diyu (x, y ,z) = (%d, %d, %d)\n", c[0], c[1], c[2]);
	}

	if (dev->use_irq){
		//printk("- \n");
		enable_irq(dev->client->irq);
	}
}


#ifdef USE_HRTIMER
static enum hrtimer_restart mma7660_timer_func(struct hrtimer *timer)
{
	struct mma7660_device *dev = container_of(timer, struct mma7660_device, timer);
	/*temp1*/printk("\n\nCheck point : %s\n", __FUNCTION__);

	if (dev->enabled) {
		queue_work(accelerometer_wq, &dev->work);
		hrtimer_start(&dev->timer, ktime_set(0, dev->sample_rate), HRTIMER_MODE_REL);
		//printk(" %s dev->sample_rate:  %d \n", __FUNCTION__, dev->sample_rate);
	}
	return HRTIMER_NORESTART;
}
#endif

static int mma7660_open(struct inode *inode, struct file *filp)
{
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);
	return 0;
}

static int mma7660_release(struct inode *inode, struct file *filp)
{
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);

	return 0;
}

static int mma7660_ioctl(struct inode *inode, struct file *filp, 
		unsigned int cmd, unsigned long arg)
{
	struct mma7660_device *dev = (struct mma7660_device *)_dev;
	/*temp1*///printk("\n\nCheck point : %s-1\n", __FUNCTION__);
	if (NULL == dev)
		return -ENODEV;
#if 1
	switch (cmd) {
		case ACCEL_IOC_ENABLE:
			if (!dev->enabled) {
				dev->enabled = 1;
	#ifdef USE_HRTIMER
				/*T*///printk("\n\nCheck point : %s-2\n", __FUNCTION__);
				/*hrtimer_start(&dev->timer, 
						ktime_set(0, dev->sample_rate), 
						HRTIMER_MODE_REL);*/
	#endif
			}
			break;
		case ACCEL_IOC_DISABLE:
			if (dev->enabled)
				dev->enabled = 0;
			/*T*///printk("\n\nCheck point : %s-3\n", __FUNCTION__);
			break;
		case ACCEL_IOCS_SAMPLERATE:
			if (arg < 10)
				return -EINVAL;
			dev->sample_rate = arg * NSEC_PER_MSEC; /* in ns */
			/*T*///printk("\n\nCheck point : %s-4\n", __FUNCTION__);
			break;
		case ACCEL_IOCG_SAMPLERATE:
			/*T*///printk("\n\nCheck point : %s-5\n", __FUNCTION__);
			return do_div(dev->sample_rate, NSEC_PER_MSEC);
			break;
		default:
			return -ENOTTY;
	}
#endif

	return 0;
}

/* use miscdevice for ioctls */
static struct file_operations fops = {
	.owner   = THIS_MODULE,
	.open    = mma7660_open,
	.release = mma7660_release,
	.ioctl   = mma7660_ioctl,
};

static struct miscdevice misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = MISC_DEV_NAME,
	.fops  = &fops,
};

static int mma7660_init_chip(struct i2c_client *client)
{
#define BMA_ANY_MOTION_THRES	5
	int ret;
	u8 v;
	int ntest=0;
	/*temp1*///printk("\n\n==========Check point++ : %s=========\n", __FUNCTION__);

	//ret = i2c_smbus_read_byte_data(client, 0x00);
	/*LGE_CHANGE_S: diYu@lge.com*/
	//Standby Mode
	#if 1
	// $00, $01, $02 : X, Y, Z output value register  :
	// $03               : Tilt register  : read only
	// $04               : Sampling Rate Status :(Read only)// $05 		     : Sleep Count Register    :(Read/Write)
	// $06 		     : Interrupt Setup 
	// $07: Mode register  : IAH IPP SCPS  ASE AWE TON X  MODE  : 1011 10 X1 (0xB9)
 	v = 0x00 & 0xff;
	ret =  i2c_smbus_write_byte_data(client, 0x07, v);

	// $06: Interrupt Setup  :SHINTX: SHINTY: SHINTZ: GINT:  ASINT: PDINT: PLINT: FBINT 
	//	v = 0xff & 0xff;
	v = 0x03 & 0xff;
	/*T*///printk("2-3: v %08x \n", v);
	ret =  i2c_smbus_write_byte_data(client, 0x06, v);
	/*T*///printk("2-1: write ret %08x \n", ret );
	

	// $08: Auto-Wake and Active Mode Portrait/Landscape  : FILT AWSR AMSR : 001 00 000
	v = 0x20 & 0xff;
	/*T*///printk("2-3: v %08x \n", v);
	ret =  i2c_smbus_write_byte_data(client, 0x08, v);
	/*T*///printk("2-2: write ret %08x \n", ret );

	
	// $07: Mode register  : IAH IPP SCPS  ASE AWE TON X  MODE  : 1011 10 X1 (0xB9)
	v = 0x49 & 0xff;
	/*T*///printk("2-3: v %08x \n", v);
	ret =  i2c_smbus_write_byte_data(client, 0x07, v);
	/*T*///printk("2-3: write ret %08x \n", ret );
	#endif
	
	#if 0 /*DUMP_REGS*/
	dump_regs();
	#endif
	while(ntest++< 100)
{
		ret = i2c_smbus_read_byte_data(client, 0x00);
		/*T*///printk("3: ret %08x (%d)\n", ret,ret);
		}
	/*T*///printk("\n============ Check point : %s=============\n", __FUNCTION__);
#if 0 //++source before used
	/* set range as +/- 2g and bandwidth as 190 Hz */
	
#endif 
	/* set any_motion threshold */
	
#if 0	
	/* set any_motion_dur as 3 */
	
	/* enable adv_INT */
	
	/* enable any_motion */
	
#endif //--source before used
	return 0;
}

static int mma7660_probe_chip(struct i2c_client *client)
{
	int ret;
	u8 v;

    /*T*///printk("\n\n %s \n", __FUNCTION__);\
//	ret = cam_i2c_read_byte(0, 0x2F, client); //
    //0x0000101
    #if 0
	v = 0x01 & 0xff;
	ret =  i2c_smbus_write_byte_data(client, 0x07, v);
	printk("\n\n2-1 write ret %08x (%d)\n", ret,ret);
	ret = i2c_smbus_read_byte_data(client, 0x00); /* read the chip id *//*original*/
	if (ret < 0) {
		dev_err(&client->dev, "i2c_smbus_read_byte_data failed\n");
		return -EIO;
	}
	dev_info(&client->dev, "chip version %02x\n", (ret & 0xff));
	#endif 

#if 0 /*++test */
	if (DEV_CHIP_ID != (ret & 0x7)) {
		dev_info(&client->dev, "unknown chip %x\n", (ret & 0x7));
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(client, 0x01); /* read the version */
	if (ret < 0) {
		dev_err(&client->dev, "i2c_smbus_read_byte_data failed\n");
		return -EIO;
	}
	dev_info(&client->dev, "chip version %02x\n", (ret & 0xff));
#endif /*--test */ 	
	ret = mma7660_init_chip(client);

	return ret;
}

static int mma7660_irq_handler(int irq, void *dev_id)
{
	struct mma7660_device *dev = dev_id;
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);
	disable_irq(dev->client->irq);
	queue_work(accelerometer_wq, &dev->work);
	return IRQ_HANDLED;
}

static int mma7660_probe(struct i2c_client *client)
{
	int ret;
	struct mma7660_device *dev;
	struct input_dev *input_dev;
	struct vreg *vreg_touch;
	int rc = -1;
		
	vreg_touch = vreg_get(0, "synt");
	vreg_enable(vreg_touch);
	rc = vreg_set_level(vreg_touch, 2800);
	if (rc != 0) {
		printk("diyu vreg_touch failed\n");
		return -1;
	}

	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality;
	}

	ret = mma7660_probe_chip(client);
	if (ret) 
		goto err_probe_chip;

	dev = kzalloc(sizeof(struct mma7660_device), GFP_KERNEL);
	if (NULL == dev) {
		ret = -ENOMEM;
		goto err_alloc_data;
	}
	_dev = dev; /* for miscdevice */
	INIT_WORK(&dev->work, mma7660_work_func);
	dev->client = client;
	dev->use_irq = USE_IRQ;
//	dev->client->irq = GPIO_MOTION_IRQ ;//Do Not need. To assign at board_adam_gpio_i2c.c
	dev->thres = ACCEL_THRES;
	dev->sample_rate = DEF_SAMPLE_RATE;
	dev->enabled = mma7660_enabled;
	i2c_set_clientdata(client, dev);

#if 0 /*DUMP_REGS*/
	dump_regs();
#endif

	ret = misc_register(&misc_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register miscdevice\n");
		goto err_miscdevice;
	}

	dev->input_dev = input_allocate_device();
	if (NULL == dev->input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto err_input_allocate_device;
	}
	input_dev = dev->input_dev;
	input_dev->name = "accelerometer";
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	/* TODO: input_set_abs_params() */
	input_set_abs_params(input_dev, ABS_X, -32, 31, 0, 0); /*512, 511, 0, 0);*/
	input_set_abs_params(input_dev, ABS_Y, -32, 31, 0, 0); /*512, 511, 0, 0);*/
	input_set_abs_params(input_dev, ABS_Z, -32, 31, 0, 0); /*512, 511, 0, 0);*/

	/*yaw*/
	input_set_abs_params(input_dev, ABS_RX, -90, 90, 0, 0); /*512, 511, 0, 0);*/
	/*pitch*/
	input_set_abs_params(input_dev, ABS_RY, -90, 90, 0, 0); /*512, 511, 0, 0);*/
	/*roll*/
	input_set_abs_params(input_dev, ABS_RZ, -90, 90, 0, 0); 
	
	input_set_abs_params(input_dev, ABS_WHEEL, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_THROTTLE, 0, 255, 0, 0);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input device\n");
		goto err_input_register_device;
	}
	/*T*///printk("\nMotion IRQ11 Check \n############\n dev->client->irq %d\n",dev->client->irq);

	if (dev->use_irq && dev->client->irq) {
		ret = request_irq(dev->client->irq, mma7660_irq_handler,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,/* IRQF_TRIGGER_HIGH,*/
			"accelerometer", dev);
		/*T*///printk("\nMotion IRQ Check \n############\n dev->client->irq %d\n",dev->client->irq);
		if (ret) {
			dev_err(&client->dev, "failed to request irq\n");
			goto err_request_irq;
		}
	}
/*
#ifdef USE_HRTIMER
	if (!dev->use_irq) {
		hrtimer_init(&dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		dev->timer.function = mma7660_timer_func;
		if (dev->enabled)
			hrtimer_start(&dev->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#endif
	*/
	//printk(" %s dev->sample_rate:  %d \n", __FUNCTION__, dev->sample_rate);

	dev_info(&client->dev, "accelleration sensor(MMA7660) probed\n");
	return 0;

err_request_irq:
	input_unregister_device(input_dev);
err_input_register_device:
	input_free_device(input_dev);
err_input_allocate_device:
	misc_deregister(&misc_dev);
err_miscdevice:
	kfree(dev);
err_alloc_data:
err_probe_chip:
err_check_functionality:
	return ret;
}

static int mma7660_remove(struct i2c_client *client)
{
	struct mma7660_device *dev = i2c_get_clientdata(client);
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);

	cancel_work_sync(&dev->work);

	if (dev->use_irq && dev->client->irq)
		free_irq(dev->client->irq, dev);
#ifdef USE_HRTIMER
	if (!dev->use_irq) 
		hrtimer_cancel(&dev->timer);
#endif

	input_unregister_device(dev->input_dev);
	input_free_device(dev->input_dev);
	misc_deregister(&misc_dev);
	kfree(dev);
	return 0;
}

#ifdef CONFIG_PM
static int mma7660_suspend(struct i2c_client *client, pm_message_t state)
{
	struct mma7660_device *dev = i2c_get_clientdata(client);
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);

	if (PM_EVENT_SUSPEND == state.event) {
		cancel_work_sync(&dev->work);

#ifdef USE_HRTIMER
		if (!dev->use_irq) 
			hrtimer_cancel(&dev->timer);
#endif
	}

	return 0;
}

static int mma7660_resume(struct i2c_client *client)
{
	struct mma7660_device *dev = i2c_get_clientdata(client);
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);

	if (mma7660_init_chip(client))
		dev_err(&client->dev, "initialization of chip failed on resuming");

#ifdef USE_HRTIMER
	if (dev->enabled)
		hrtimer_start(&dev->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif

	return 0;
}
#endif

static struct i2c_device_id mma7660_idtable[] = {
        { "accel_mma7660", 1 },
//        { }
};


static struct i2c_driver mma7660_driver = {
	.probe		= mma7660_probe,
	.remove		= mma7660_remove,
	.id_table 	= mma7660_idtable,
#ifdef CONFIG_PM
	.suspend	= mma7660_suspend,
	.resume		= mma7660_resume,
#endif
	.driver		= {
		.name	= "accel_mma7660",
	},
};

static int __init mma7660_enable_setup(char *__unused)
{
	mma7660_enabled = 1;
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);

	return 1;
}

__setup("accel_enable", mma7660_enable_setup);


static int __devinit mma7660_init(void)
{
	int ret;
	/*temp1*///printk("\n\nCheck point1 : %s\n", __FUNCTION__);
	accelerometer_wq = create_singlethread_workqueue("accelerometer_wq");
	if (!accelerometer_wq)
		{
	/*temp1*///printk("\n\nCheck point2 : %s\n", __FUNCTION__);
		
		return -ENOMEM;
		}
	ret = i2c_add_driver(&mma7660_driver);
	/*temp1*///printk("\n\nCheck point3 : %d\n",ret);
	
	return ret; 
}

static void __exit mma7660_exit(void)
{
	i2c_del_driver(&mma7660_driver);
	/*temp1*///printk("\n\nCheck point : %s\n", __FUNCTION__);
	if (accelerometer_wq)
		destroy_workqueue(accelerometer_wq);
}

module_init(mma7660_init);
module_exit(mma7660_exit);

MODULE_DESCRIPTION("Acceleration MMA7660 Sensor Driver");
MODULE_AUTHOR("Dae il, yu <diyu@lge.com>");
MODULE_LICENSE("GPL");

