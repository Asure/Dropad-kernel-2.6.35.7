/*
 * drivers/input/mouse/vd5376_mouse_right.c
 *
 * Copyright (c) 2010 Crztech Co., Ltd.
 *	PyeongJeong Lee <leepjung@crz-tech.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/vd5376.h>
#include "vd5376_mouse.h"

#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define DEVICE_NAME	"vd5376"

static int vd5376_reset_config(struct vd5376 *mouse)
{
	int i, err = 0;
	struct vd5376_platform_data *pdata = mouse->pdata;

	err = gpio_request(pdata->pd_gpio, pdata->pd_desc);
	if (err) {
		printk(KERN_INFO "gpio request error : %d\n", err);
	} else {
		s3c_gpio_setpull(pdata->pd_gpio, S3C_GPIO_PULL_NONE);
		gpio_direction_output(pdata->pd_gpio, 1);
		udelay(200);
	}
	gpio_free(pdata->pd_gpio);

	for ( i = 0; i < VD5376_INIT_REGS; i++) {
		if(vd5376_init_reg[i][0] == Wait_for_Delay)
			udelay(vd5376_init_reg[i][1]);
		else {
			err = i2c_smbus_write_byte_data(mouse->client,
							vd5376_init_reg[i][0],
							vd5376_init_reg[i][1]);
			if (err < 0) break;
		}
	}

	if (err < 0)
		dev_err(&mouse->client->dev, "vd5376 initialization failed\n");

	return err;
}

static void vd5376_jesture_reschedule_work(struct vd5376 *mouse, unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&mouse->lock, flags);

	__cancel_delayed_work(&mouse->jesture_dwork);
	schedule_delayed_work(&mouse->jesture_dwork, delay);

	spin_unlock_irqrestore(&mouse->lock, flags);	
}

static void vd5376_reschedule_work(struct vd5376 *mouse, unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&mouse->lock, flags);

	__cancel_delayed_work(&mouse->dwork);
	schedule_delayed_work(&mouse->dwork, delay);

	spin_unlock_irqrestore(&mouse->lock, flags);	
}

static void vd5376_send_message(struct vd5376_platform_data *pdata, int status, s8 x, s8 y)
{
	struct vd5376_mouse_data data;
	data.id = pdata->id;
	data.status = status;
	data.x = x;
	data.y = y;

	pdata->vd5376_callback(&data);
}

static void vd5376_jesture_work_handler(struct work_struct *work)
{
	struct vd5376 *mouse = container_of(work, struct vd5376, jesture_dwork.work);
	struct vd5376_platform_data *pdata = mouse->pdata;

	if(mouse->jesture == 1) {
		if(!((gpio_get_value(pdata->gpio))^(pdata->active_low))) {
			mouse->jesture = 0;
			if(pdata->vd5376_callback != NULL)
				vd5376_send_message(pdata, STATUS_JESTURE_STOP, 0, 0);
		}
	}
}

static void vd5376_work_handler(struct work_struct *work)
{
	int err;
	s8 data[2];
	struct vd5376 *mouse = container_of(work, struct vd5376, dwork.work);
	struct vd5376_platform_data *pdata = mouse->pdata;

	if((gpio_get_value(pdata->gpio))^(pdata->active_low))
	{
		err = i2c_smbus_read_i2c_block_data(mouse->client,
					      Device_X_Motion, 
					      2, data);
		if(err < 0) {
			printk(KERN_ERR "i2c read error(%d)!\n", err);
		} else {
			if(pdata->vd5376_callback == NULL) {
				input_report_rel(mouse->input, REL_X, data[0]);
				input_report_rel(mouse->input, REL_Y, data[1]);
				input_sync(mouse->input);
			} else {
				vd5376_send_message(pdata, STATUS_MOVE,
						    data[0],
						    data[1]);

			}
		}

		vd5376_reschedule_work(mouse, 0);
	} else {
		if(mouse->mode == INTERRUPT_MODE) {
			enable_irq(pdata->irq);
			vd5376_jesture_reschedule_work(mouse, 40);
		} else {
			vd5376_reschedule_work(mouse,
					       msecs_to_jiffies(pdata->delay));
		}
	}
}

static irqreturn_t vd5376_irq_handler(int irq, void *handle)
{
	struct vd5376 *mouse = handle;
	struct vd5376_platform_data *pdata = mouse->pdata;

	if(mouse->mode == INTERRUPT_MODE)
		disable_irq_nosync(pdata->irq);

	if(irq == pdata->irq) {
		if(mouse->jesture == 0) {
			mouse->jesture = 1;
			if(pdata->vd5376_callback != NULL) 
				vd5376_send_message(pdata, STATUS_JESTURE_START, 0, 0);
		}
		vd5376_reschedule_work(mouse, 0);
	} else {
		if(pdata->btn_irq)
			disable_irq_nosync(pdata->btn_irq);

		if(pdata->vd5376_callback == NULL) {
			input_report_key(mouse->input, BTN_LEFT,
					 gpio_get_value(pdata->btn_gpio)^(pdata->btn_active_low));
			input_sync(mouse->input);
		} else {
			if(gpio_get_value(pdata->btn_gpio) != (pdata->btn_active_low)) 
				vd5376_send_message(pdata, STATUS_BTN_ON, 0, 0);
			else 
				vd5376_send_message(pdata,STATUS_BTN_OFF, 0, 0);
		}

		if(pdata->btn_irq)
			enable_irq(pdata->btn_irq);

		if(mouse->mode == INTERRUPT_MODE)
			enable_irq(pdata->irq);
	}

	return IRQ_HANDLED;
}

static void vd5376_mouse_input_params(struct vd5376 *mouse)
{
	struct input_dev *input = mouse->input;

        input->name = mouse->client->name;
        input->phys = mouse->client->adapter->name;
        input->id.bustype = BUS_I2C;
        input->dev.parent = &mouse->client->dev;
        input_set_drvdata(input, mouse);

	input_set_capability(input, EV_REL, REL_X);
        input_set_capability(input, EV_REL, REL_Y);
	input_set_capability(input, EV_KEY, BTN_LEFT);
	input_set_capability(input, EV_KEY, BTN_RIGHT);
}

static void vd5376_mouse_gpio_params(struct vd5376 *mouse)
{
	int err;
	struct vd5376_platform_data *pdata = mouse->pdata;

	if(pdata->btn_gpio) {
	        err = gpio_request(pdata->btn_gpio, pdata->btn_desc);
		if (err) {
			printk(KERN_INFO "gpio request error : %d\n", err);
		} else {
			s3c_gpio_cfgpin(pdata->btn_gpio, pdata->btn_config);
			s3c_gpio_setpull(pdata->btn_gpio, S3C_GPIO_PULL_NONE);
		}
		gpio_free(pdata->btn_gpio);
	}

	if(pdata->gpio) {
	        err = gpio_request(pdata->gpio, pdata->desc);
		if (err) {
			printk(KERN_INFO "gpio request error : %d\n", err);
		} else {
			s3c_gpio_cfgpin(pdata->gpio, pdata->config);
			s3c_gpio_setpull(pdata->gpio, S3C_GPIO_PULL_NONE);
		}
		gpio_free(pdata->gpio);
	}
}

static int vd5376_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	struct vd5376 *mouse;
	struct vd5376_platform_data *pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		printk(KERN_ERR "%s: need I2C", __func__);
		return -ENODEV;
	}

	mouse = kzalloc(sizeof(struct vd5376), GFP_KERNEL);
	if (!mouse) {
		err = -ENOMEM;
		goto err_free;
	}

	mouse->client = client;
	mouse->pdata = pdata;

	vd5376_mouse_gpio_params(mouse);

	err = vd5376_reset_config(mouse);
	if(err < 0)
		goto err_free_mem;

	if(pdata->vd5376_callback == NULL) {
		mouse->input = input_allocate_device();
		if (!mouse->input) {
			err = -ENOMEM;
			printk(KERN_ERR "%s: Failed to allocate input device\n", 
				__func__);
			goto err_free_mem;
		}

		vd5376_mouse_input_params(mouse);
	
		err = input_register_device(mouse->input);
		if (err) {
			printk(KERN_ERR "%s: Unable to register %s input device\n",
				__func__, mouse->input->name);
			goto err_free_device;
		}
	}

	INIT_DELAYED_WORK(&mouse->dwork, vd5376_work_handler);
	INIT_DELAYED_WORK(&mouse->jesture_dwork, vd5376_jesture_work_handler);
	spin_lock_init(&mouse->lock);

	if(pdata->btn_irq) {
		err = request_irq(pdata->btn_irq, vd5376_irq_handler,
				  (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
				  client->name, mouse);
		if (err < 0) {
			dev_err(&client->dev, "button irq %d busy?\n",
				pdata->btn_irq);
			goto err_free_register;
		}
	}

	if(pdata->irq) {
		mouse->mode = INTERRUPT_MODE;

		err = request_irq(pdata->irq, vd5376_irq_handler,
				  (pdata->active_low)?IRQF_TRIGGER_LOW:IRQF_TRIGGER_HIGH,
				  client->name, mouse);
		if (err < 0) {
			dev_err(&client->dev, "motion irq %d busy?\n",
			pdata->irq);
			goto err_free_register;
		}

		printk(KERN_INFO "%s: Start Finger Mouse in interrupt mode\n", 
		       __func__);
	} else {
		mouse->mode = POLLING_MODE;

		vd5376_reschedule_work(mouse,
				       msecs_to_jiffies(pdata->delay));

		printk(KERN_INFO "%s: Start Finger Mouse in polling mode\n", 
			 __func__);
	}

	i2c_set_clientdata(client, mouse);

	return 0;

 err_free_register:
	if(pdata->vd5376_callback == NULL) {
		input_unregister_device(mouse->input);
	}
 err_free_device:
	input_free_device(mouse->input);
 err_free_mem:
	kfree(mouse);
 err_free:
	return err;
}

static int vd5376_remove(struct i2c_client *client)
{
	struct vd5376 *mouse = i2c_get_clientdata(client);
	struct vd5376_platform_data *pdata = mouse->pdata;

	if(pdata->irq)
		free_irq(pdata->irq, mouse);
	if(pdata->btn_irq)
		free_irq(pdata->btn_irq, mouse);

	cancel_delayed_work_sync(&mouse->dwork);
	cancel_delayed_work_sync(&mouse->jesture_dwork);
	if(pdata->vd5376_callback == NULL) {
		input_unregister_device(mouse->input);
		input_free_device(mouse->input);
	}
	kfree(mouse);

	return 0;
}

static struct i2c_device_id vd5376_idtable[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, vd5376_idtable);

static struct i2c_driver vd5376_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DEVICE_NAME
	},
	.id_table	= vd5376_idtable,
	.probe		= vd5376_probe,
	.remove		= vd5376_remove,
};

static int __init vd5376_init(void)
{
	return i2c_add_driver(&vd5376_driver);
}

static void __exit vd5376_exit(void)
{
	i2c_del_driver(&vd5376_driver);
}

module_init(vd5376_init);
module_exit(vd5376_exit);

MODULE_AUTHOR("Pyeongjeong Lee <leepjung@crz-tech.com>");
MODULE_DESCRIPTION("VD5376 Finger Mouse Driver");
MODULE_LICENSE("GPL");

