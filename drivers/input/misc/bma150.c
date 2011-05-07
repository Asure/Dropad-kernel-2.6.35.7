/*
 * BMA150 accelerometer driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#define BMA150_VERSION "1.2.0"

#define BMA150_NAME "bma150"

/* for debugging */
#define DEBUG 0
#define DEBUG_THRESHOLD 0
#define TRACE_FUNC() pr_debug(BMA150_NAME ": <trace> %s()\n", __FUNCTION__)

/*
 * Default parameters
 */
#define SENSOR_DELAY_FASTEST            0       // delay = 10
#define SENSOR_DELAY_GAME               1       // delay = 20
#define SENSOR_DELAY_UI                 2       // delay = 60
#define SENSOR_DELAY_NORMAL             3       // delay = 200

#define BMA150_DEFAULT_DELAY            SENSOR_DELAY_NORMAL
#define BMA150_MAX_DELAY                SENSOR_DELAY_NORMAL
#define BMA150_DEFAULT_THRESHOLD        0
/*
 * Registers
 */
#define BMA150_CHIP_ID_REG              0x00
#define BMA150_CHIP_ID                  0x02

#define BMA150_AL_VERSION_REG           0x01
#define BMA150_AL_VERSION_MASK          0x0f
#define BMA150_AL_VERSION_SHIFT         0

#define BMA150_ML_VERSION_REG           0x01
#define BMA150_ML_VERSION_MASK          0xf0
#define BMA150_ML_VERSION_SHIFT         4

#define BMA150_ACC_REG                  0x02

#define BMA150_SOFT_RESET_REG           0x0a
#define BMA150_SOFT_RESET_MASK          0x02
#define BMA150_SOFT_RESET_SHIFT         1

#define BMA150_SLEEP_REG                0x0a
#define BMA150_SLEEP_MASK               0x01
#define BMA150_SLEEP_SHIFT              0

#define BMA150_RANGE_REG                0x14
#define BMA150_RANGE_MASK               0x18
#define BMA150_RANGE_SHIFT              3
#define BMA150_RANGE_2G                 0
#define BMA150_RANGE_4G                 1
#define BMA150_RANGE_8G                 2

#define BMA150_BANDWIDTH_REG            0x14
#define BMA150_BANDWIDTH_MASK           0x07
#define BMA150_BANDWIDTH_SHIFT          0
#define BMA150_BANDWIDTH_25HZ           0
#define BMA150_BANDWIDTH_50HZ           1
#define BMA150_BANDWIDTH_100HZ          2
#define BMA150_BANDWIDTH_190HZ          3
#define BMA150_BANDWIDTH_375HZ          4
#define BMA150_BANDWIDTH_750HZ          5
#define BMA150_BANDWIDTH_1500HZ         6

#define BMA150_SHADOW_DIS_REG           0x15
#define BMA150_SHADOW_DIS_MASK          0x08
#define BMA150_SHADOW_DIS_SHIFT         3

#define BMA150_WAKE_UP_PAUSE_REG        0x15
#define BMA150_WAKE_UP_PAUSE_MASK       0x06
#define BMA150_WAKE_UP_PAUSE_SHIFT      1

#define BMA150_WAKE_UP_REG              0x15
#define BMA150_WAKE_UP_MASK             0x01
#define BMA150_WAKE_UP_SHIFT            0

/*
 * Acceleration measurement
 */
#define BMA150_RESOLUTION               256

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)

struct acceleration {
	int x;
	int y;
	int z;
};

/*
 * Output data rate
 */
struct bma150_odr {
        unsigned long delay;            /* min delay (msec) in the range of ODR */
        u8 odr;                         /* bandwidth register value */
};

static const struct bma150_odr bma150_odr_table[] = {
	{1,  BMA150_BANDWIDTH_1500HZ},
	{2,  BMA150_BANDWIDTH_750HZ},
	{3,  BMA150_BANDWIDTH_375HZ},
	{6,  BMA150_BANDWIDTH_190HZ},
	{10, BMA150_BANDWIDTH_100HZ},
	{20, BMA150_BANDWIDTH_50HZ},
	{40, BMA150_BANDWIDTH_25HZ},
};

/*
 * Transformation matrix for chip mounting position
 */
static const int bma150_position_map[][3][3] = {
	{{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}}, /* top/upper-left */
	{{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}}, /* top/upper-right */
	{{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}}, /* top/lower-right */
	{{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}}, /* top/lower-left */
	{{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}}, /* bottom/upper-right */
	{{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}}, /* bottom/upper-left */
	{{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}}, /* bottom/lower-left */
	{{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}}, /* bottom/lower-right */
};

/*
 * driver private data
 */
struct bma150_data {
	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	atomic_t position;              /* attribute value */
	atomic_t threshold;             /* attribute value */
	struct acceleration last;       /* last measured data */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
#if DEBUG
	int suspend;
#endif
};
static struct bma150_data *bma150_ptr;

#define BMAIO			       	0x50
#define BMA_IOCTL_ENABLE		_IO(BMAIO, 0x31)
#define BMA_IOCTL_DISABLE		_IO(BMAIO, 0x32)
#define BMA_IOCTL_IS_ENABLE		_IOR(BMAIO, 0x33, int)
#define BMA_IOCTL_DELAY_GET		_IOR(BMAIO, 0x34, int)
#define BMA_IOCTL_DELAY_SET		_IOW(BMAIO, 0x35, int)
#define BMA_IOCTL_POSITION_GET		_IOR(BMAIO, 0x36, int)
#define BMA_IOCTL_POSITION_SET		_IOW(BMAIO, 0x37, int)
#define BMA_IOCTL_THRESHOLD_GET		_IOR(BMAIO, 0x38, int)
#define BMA_IOCTL_THRESHOLD_SET		_IOW(BMAIO, 0x39, int)
#define BMA_IOCTL_DATA			_IOR(BMAIO, 0x3a, int[3])

static int bma150_device_open( struct inode*, struct file* );
static int bma150_device_release( struct inode*, struct file* );
static int bma150_device_ioctl( struct inode*, struct file*, unsigned int, unsigned long );
static ssize_t bma150_device_read( struct file *filp, char *buf, size_t count, loff_t *ofs );
static ssize_t bma150_device_write( struct file *filp, const char *buf, size_t count, loff_t *ofs );
static unsigned int bma150_device_poll( struct file *filp, struct poll_table_struct *pwait );

static struct file_operations bma150_device_fops =
{
	.owner = THIS_MODULE,
	.open = bma150_device_open,
	.ioctl = bma150_device_ioctl,
	.release = bma150_device_release,
	.read = bma150_device_read,
	.write = bma150_device_write,
	.poll = bma150_device_poll,
};

static struct miscdevice bma150_misc =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = BMA150_NAME,
	.fops = &bma150_device_fops,
};

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

/* register access functions */
#define bma150_read_bits(p,r) \
	((i2c_smbus_read_byte_data((p)->client, r##_REG) & r##_MASK) >> r##_SHIFT)

#define bma150_update_bits(p,r,v) \
        i2c_smbus_write_byte_data((p)->client, r##_REG, \
                ((i2c_smbus_read_byte_data((p)->client,r##_REG) & ~r##_MASK) | ((v) << r##_SHIFT)))

/*
 * Device dependant operations
 */
static int bma150_power_up(struct bma150_data *bma150)
{
	bma150_update_bits(bma150, BMA150_SLEEP, 0);
	/* wait 1ms for wake-up time from sleep to operational mode */
	msleep(1);

	return 0;
}

static int bma150_power_down(struct bma150_data *bma150)
{
	bma150_update_bits(bma150, BMA150_SLEEP, 1);

	return 0;
}

static int bma150_hw_init(struct bma150_data *bma150)
{
	/* reset hardware */
	bma150_power_up(bma150);
	i2c_smbus_write_byte_data(bma150->client,
				  BMA150_SOFT_RESET_REG, BMA150_SOFT_RESET_MASK);

	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);

	bma150_update_bits(bma150, BMA150_RANGE, BMA150_RANGE_2G);

	bma150_power_down(bma150);

	return 0;
}

static int bma150_get_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	return atomic_read(&bma150->enable);
}

static void bma150_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);
	int delay = atomic_read(&bma150->delay);

	mutex_lock(&bma150->enable_mutex);

	if (enable)
	{                   /* enable if state will be changed */
		if (!atomic_cmpxchg(&bma150->enable, 0, 1))
		{
			bma150_power_up(bma150);
			schedule_delayed_work(&bma150->work,
					      delay_to_jiffies(delay) + 1);
		}
	}
	else
	{                        /* disable if state will be changed */
		if (atomic_cmpxchg(&bma150->enable, 1, 0))
		{
			cancel_delayed_work_sync(&bma150->work);
			bma150_power_down(bma150);
		}
	}
	atomic_set(&bma150->enable, enable);

	mutex_unlock(&bma150->enable_mutex);
}

static int bma150_get_delay(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	return atomic_read(&bma150->delay);
}

static void bma150_set_delay(struct device *dev, int delay)
{
	int delay_ms;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);
	u8 odr;
	int i;

	/* determine optimum ODR */
	for (i = 1; (i < ARRAY_SIZE(bma150_odr_table)) &&
		     (actual_delay(delay) >= bma150_odr_table[i].delay); i++)
		;
	odr = bma150_odr_table[i-1].odr;
	atomic_set(&bma150->delay, delay);

	mutex_lock(&bma150->enable_mutex);

	if (bma150_get_enable(dev))
	{
#if 0
		switch(delay)
		{
		case SENSOR_DELAY_FASTEST:
			delay_ms = 50;
			break;
		case SENSOR_DELAY_GAME:
			delay_ms = 80;
			break;
		case SENSOR_DELAY_UI:
			delay_ms = 100;
			break;
		default:
		case SENSOR_DELAY_NORMAL:
			delay_ms = 200;
			break;
		}
#else
		delay_ms = delay;printk(KERN_INFO "test delay :%d\n", delay_ms);
#endif		
		cancel_delayed_work_sync(&bma150->work);
		bma150_update_bits(bma150, BMA150_BANDWIDTH, odr);
		schedule_delayed_work(&bma150->work,
				      delay_to_jiffies(delay_ms) + 1);
	}
	else
	{
		bma150_power_up(bma150);
		bma150_update_bits(bma150, BMA150_BANDWIDTH, odr);
		bma150_power_down(bma150);
	}

	mutex_unlock(&bma150->enable_mutex);
}

static int bma150_get_position(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	return atomic_read(&bma150->position);
}

static void bma150_set_position(struct device *dev, int position)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	atomic_set(&bma150->position, position);
}

static int bma150_get_threshold(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	return atomic_read(&bma150->threshold);
}

static void bma150_set_threshold(struct device *dev, int threshold)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

        atomic_set(&bma150->threshold, threshold);
}

static int bma150_data_filter(struct bma150_data *bma150, struct acceleration *accel, int data[])
{
	int threshold = atomic_read(&bma150->threshold);
#if DEBUG_THRESHOLD
	struct i2c_client *client = bma150->client;
        int update;
#endif
#if DEBUG_THRESHOLD
        update = 0;
#endif
	mutex_lock(&bma150->data_mutex);
	if ((abs(bma150->last.x - data[0]) > threshold) ||
            (abs(bma150->last.y - data[1]) > threshold) ||
            (abs(bma150->last.z - data[2]) > threshold))
	{
		accel->x = (-1)*data[0];
		accel->y = data[1];
		accel->z = data[2];
#if DEBUG_THRESHOLD
                update = 1;
#endif
        }
	else
	{
		*accel = bma150->last;
	}

#if DEBUG_THRESHOLD
        if (update == 1)
	{
            dev_info(&client->dev, "threshold=%d x(%d) y(%d) z(%d) accel(%d,%d,%d) ****\n", threshold,
                     bma150->last.x - data[0], bma150->last.y - data[1], bma150->last.z - data[2], accel->x, accel->y, accel->z);
        }
	else
	{
            dev_info(&client->dev, "threshold=%d x(%d) y(%d) z(%d) accel(%d,%d,%d)\n", threshold,
                     bma150->last.x - data[0], bma150->last.y - data[1], bma150->last.z - data[2], accel->x, accel->y, accel->z);
        }
#endif
	mutex_unlock(&bma150->data_mutex);

    return 0;
}

static int bma150_measure(struct bma150_data *bma150, struct acceleration *accel)
{
	struct i2c_client *client = bma150->client;
	u8 buf[6];
	int raw[3], data[3];
	int pos = atomic_read(&bma150->position);
	long long g;
	int i, j;

	/* read acceleration data */
	if (i2c_smbus_read_i2c_block_data(client, BMA150_ACC_REG, 6, buf) != 6)
	{
		dev_err(&client->dev,
			"I2C block read error: addr=0x%02x, len=%d\n",
			BMA150_ACC_REG, 6);
		for (i = 0; i < 3; i++)
			raw[i] = 0;
	}
	else
	{
		for (i = 0; i < 3; i++)
			raw[i] = (*(s16 *)&buf[i*2]) >> 6;
	}

	/* for X, Y, Z axis */
	for (i = 0; i < 3; i++)
	{
		/* coordinate transformation */
		data[i] = 0;
		for (j = 0; j < 3; j++)
			data[i] += raw[j] * bma150_position_map[pos][i][j];

		/* normalization */
		g = (long long)data[i] * GRAVITY_EARTH / BMA150_RESOLUTION;
		data[i] = g;
	}

	dev_dbg(&client->dev, "raw(%5d,%5d,%5d) => norm(%8d,%8d,%8d)\n",
		raw[0], raw[1], raw[2], data[0], data[1], data[2]);

        bma150_data_filter(bma150, accel, data);

	return 0;
}

static void bma150_work_func(struct work_struct *work)
{
	struct bma150_data *bma150 = container_of((struct delayed_work *)work,
						  struct bma150_data, work);
	struct acceleration accel;
	unsigned long delay = delay_to_jiffies(atomic_read(&bma150->delay));

	bma150_measure(bma150, &accel);

	input_report_abs(bma150->input, ABS_X, accel.x);
	input_report_abs(bma150->input, ABS_Y, accel.y);
	input_report_abs(bma150->input, ABS_Z, accel.z);
	input_sync(bma150->input);

	mutex_lock(&bma150->data_mutex);
	bma150->last = accel;
	mutex_unlock(&bma150->data_mutex);

	schedule_delayed_work(&bma150->work, delay);
}

/*
 * Input device interface
 */
static int bma150_input_init(struct bma150_data *bma150)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "bma150";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SYN, dev->evbit);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, bma150);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	bma150->input = dev;

	return 0;
}

static void bma150_input_fini(struct bma150_data *bma150)
{
	struct input_dev *dev = bma150->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/*
 * sysfs device attributes
 */
static ssize_t bma150_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bma150_get_enable(dev));
}

static ssize_t bma150_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if ((enable == 0) || (enable == 1))
		bma150_set_enable(dev, enable);

	return count;
}

static ssize_t bma150_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bma150_get_delay(dev));
}

static ssize_t bma150_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

//	if (delay > BMA150_MAX_DELAY)
//		delay = BMA150_MAX_DELAY;

	bma150_set_delay(dev, delay);

	return count;
}

static ssize_t bma150_position_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bma150_get_position(dev));
}

static ssize_t bma150_position_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long position;

	position = simple_strtoul(buf, NULL,10);
	if ((position >= 0) && (position <= 7))
		bma150_set_position(dev, position);

	return count;
}

static ssize_t bma150_threshold_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bma150_get_threshold(dev));
}

static ssize_t bma150_threshold_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long threshold;

	threshold = simple_strtoul(buf, NULL,10);
        if (threshold >= 0 && threshold <= ABSMAX_2G)
		bma150_set_threshold(dev, threshold);

	return count;
}

static ssize_t bma150_wake_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

	return count;
}

static ssize_t bma150_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma150_data *bma150 = input_get_drvdata(input);
	struct acceleration accel;

	mutex_lock(&bma150->data_mutex);
	accel = bma150->last;
	mutex_unlock(&bma150->data_mutex);

	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

#if DEBUG
static ssize_t bma150_debug_reg_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma150_data *bma150 = input_get_drvdata(input);
	struct i2c_client *client = bma150->client;
	ssize_t count = 0;
	u8 reg[0x16];
	int i;

	i2c_smbus_read_i2c_block_data(client, 0x00, 0x16, reg);
	for (i = 0; i < 0x16; i++)
		count += sprintf(&buf[count], "%02x: %d\n", i, reg[i]);

	return count;
}

static int bma150_suspend(struct i2c_client *client, pm_message_t mesg);
static int bma150_resume(struct i2c_client *client);

static ssize_t bma150_debug_suspend_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma150_data *bma150 = input_get_drvdata(input);

	return sprintf(buf, "%d\n", bma150->suspend);
}

static ssize_t bma150_debug_suspend_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma150_data *bma150 = input_get_drvdata(input);
	struct i2c_client *client = bma150->client;
	unsigned long suspend = simple_strtoul(buf, NULL, 10);

	if (suspend)
	{
		pm_message_t msg;
		bma150_suspend(client, msg);
	}
	else
	{
		bma150_resume(client);
	}

	return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		   bma150_enable_show, bma150_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		   bma150_delay_show, bma150_delay_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR,
		   bma150_position_show, bma150_position_store);
static DEVICE_ATTR(threshold, S_IRUGO|S_IWUSR,
		   bma150_threshold_show, bma150_threshold_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		   NULL, bma150_wake_store);
static DEVICE_ATTR(data, S_IRUGO,
		   bma150_data_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_reg, S_IRUGO,
		   bma150_debug_reg_show, NULL);
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
		   bma150_debug_suspend_show, bma150_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *bma150_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_threshold.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group bma150_attribute_group = {
	.attrs = bma150_attributes
};

/* bma150 file operation */
static int bma150_device_open( struct inode* inode, struct file* file)
{
	return 0;
}

static int bma150_device_release( struct inode* inode, struct file* file)
{
	return 0;
}

static int bma150_device_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret=0;
	int kbuf = 0;
	int axis_data[3];

	switch( cmd )
	{
	case BMA_IOCTL_ENABLE:
//		printk(KERN_INFO "test : enabled\n");
		bma150_set_enable(&bma150_ptr->client->dev, 1);
		break;
	case BMA_IOCTL_DISABLE:
//		printk(KERN_INFO "test : disabled\n");
		bma150_set_enable(&bma150_ptr->client->dev, 0);
		break;
	case BMA_IOCTL_IS_ENABLE:
//		printk(KERN_INFO "test : is enabled\n");
		kbuf = bma150_get_enable(&bma150_ptr->client->dev);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_DELAY_GET:
		kbuf = bma150_get_delay(&bma150_ptr->client->dev);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_DELAY_SET:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
//		printk(KERN_INFO "test : delay : %d\n", kbuf);
//		if(kbuf > BMA150_MAX_DELAY)
//			kbuf = BMA150_MAX_DELAY;
		bma150_set_delay(&bma150_ptr->client->dev, kbuf);
		break;
	case BMA_IOCTL_POSITION_GET:
		kbuf = bma150_get_position(&bma150_ptr->client->dev);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_POSITION_SET:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
		if ((kbuf >= 0) && (kbuf <= 7))
			kbuf = CONFIG_SENSORS_BMA150_POSITION_V2;
		bma150_set_position(&bma150_ptr->client->dev, kbuf);
		break;
	case BMA_IOCTL_THRESHOLD_GET:
		kbuf = bma150_get_threshold(&bma150_ptr->client->dev);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_THRESHOLD_SET:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
        	if (kbuf >= 0 && kbuf <= ABSMAX_2G)
			kbuf = BMA150_DEFAULT_THRESHOLD;
		bma150_set_threshold(&bma150_ptr->client->dev, kbuf);
		break;
	case BMA_IOCTL_DATA:
		axis_data[0] = bma150_ptr->last.x;
		axis_data[1] = bma150_ptr->last.y;
		axis_data[2] = bma150_ptr->last.z;
		if(copy_to_user(argp, &axis_data, sizeof(axis_data)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static ssize_t bma150_device_read( struct file *filp, char *buf, size_t count, loff_t *ofs )
{
	return 0;
}

static ssize_t bma150_device_write( struct file *filp, const char *buf, size_t count, loff_t *ofs )
{
	return 0;
}

static unsigned int bma150_device_poll( struct file *filp, struct poll_table_struct *pwait )
{
	return 0;
}

/*
 * I2C client
 */
static int bma150_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int id;

	id = i2c_smbus_read_byte_data(client, BMA150_CHIP_ID_REG);
	if (id != BMA150_CHIP_ID)
		return -ENODEV;

	return 0;
}

static int bma150_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bma150_data *bma150;
	int err;

	/* setup private data */
	bma150 = kzalloc(sizeof(struct bma150_data), GFP_KERNEL);
	if (!bma150)
	{
		err = -ENOMEM;
		goto error_0;
	}
	mutex_init(&bma150->enable_mutex);
	mutex_init(&bma150->data_mutex);

	/* setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		err = -ENODEV;
		goto error_1;
	}
	i2c_set_clientdata(client, bma150);
	bma150->client = client;

	/* detect and init hardware */
	if ((err = bma150_detect(client, NULL)))
		goto error_1;

	dev_info(&client->dev, "%s found\n", id->name);
	dev_info(&client->dev, "al_version=%d, ml_version=%d\n",
		 bma150_read_bits(bma150, BMA150_AL_VERSION),
		 bma150_read_bits(bma150, BMA150_ML_VERSION));

        bma150_hw_init(bma150);
	//bma150_set_delay(&client->dev, BMA150_DEFAULT_DELAY);
	bma150_set_delay(&client->dev, 100);
	bma150_set_position(&client->dev, CONFIG_SENSORS_BMA150_POSITION_V2);
	bma150_set_threshold(&client->dev, BMA150_DEFAULT_THRESHOLD);

	/* setup driver interfaces */
	INIT_DELAYED_WORK(&bma150->work, bma150_work_func);

	err = bma150_input_init(bma150);
	if (err < 0)
		goto error_1;

	err = sysfs_create_group(&bma150->input->dev.kobj, &bma150_attribute_group);
	if (err < 0)
		goto error_2;

	err = misc_register(&bma150_misc);
	if (err < 0)
		goto error_3;

	bma150_ptr = bma150;

	return 0;

error_3:
	sysfs_remove_group(&bma150->input->dev.kobj, &bma150_attribute_group);
error_2:
	bma150_input_fini(bma150);
error_1:
	kfree(bma150);
error_0:
	return err;
}

static int bma150_remove(struct i2c_client *client)
{
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	bma150_set_enable(&client->dev, 0);

	misc_deregister(&bma150_misc);
	sysfs_remove_group(&bma150->input->dev.kobj, &bma150_attribute_group);
	bma150_input_fini(bma150);
	kfree(bma150);

	return 0;
}

static int bma150_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	TRACE_FUNC();

	mutex_lock(&bma150->enable_mutex);

	if (bma150_get_enable(&client->dev))
	{
		cancel_delayed_work_sync(&bma150->work);
		bma150_power_down(bma150);
	}
#if DEBUG
	bma150->suspend = 1;
#endif

	mutex_unlock(&bma150->enable_mutex);

	return 0;
}

static int bma150_resume(struct i2c_client *client)
{
	struct bma150_data *bma150 = i2c_get_clientdata(client);
	int delay = atomic_read(&bma150->delay);

	TRACE_FUNC();

	bma150_hw_init(bma150);
	bma150_set_delay(&client->dev, delay);

	mutex_lock(&bma150->enable_mutex);

	if (bma150_get_enable(&client->dev))
	{
		bma150_power_up(bma150);
		schedule_delayed_work(&bma150->work,
				      delay_to_jiffies(delay) + 1);
	}
#if DEBUG
	bma150->suspend = 0;
#endif

	mutex_unlock(&bma150->enable_mutex);

	return 0;
}

static const struct i2c_device_id bma150_id[] =
{
	{BMA150_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bma150_id);

struct i2c_driver bma150_driver =
{
	.driver = {
		.name = "bma150",
		.owner = THIS_MODULE,
	},
	.probe = bma150_probe,
	.remove = bma150_remove,
	.suspend = bma150_suspend,
	.resume = bma150_resume,
	.id_table = bma150_id,
};

/*
 * Module init and exit
 */
static int __init bma150_init(void)
{
	return i2c_add_driver(&bma150_driver);
}
module_init(bma150_init);

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
}
module_exit(bma150_exit);

MODULE_DESCRIPTION("BMA150 accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BMA150_VERSION);

