/*
 * drivers/input/mouse/vd5376_mouse_left.c
 *
 * Copyright (c) 2010 Crztech Co., Ltd.
 *	Pyeongjeong Lee <leepjung@crz-tech.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/vd5376.h>
#include <linux/platform_device.h>
#include "vd5376_mouse.h"

#include <linux/delay.h>

#define DEVICE_NAME	"vd5376-mouse"

struct vd5376_mouse
{
	struct platform_device			*pdev;
	struct vd5376_mouse_platform_data	*pdata;
	struct input_dev			*input;
	struct delayed_work			dwork;
	spinlock_t				lock;
};

static struct vd5376_mouse *input_mouse;

static void vd5376_reschedule_work(struct vd5376_mouse *mouse, unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&mouse->lock, flags);

	__cancel_delayed_work(&mouse->dwork);
	schedule_delayed_work(&mouse->dwork, delay);

	spin_unlock_irqrestore(&mouse->lock, flags);
}

static void vd5376_mouse_callback(struct vd5376_mouse_data *data)
{
	static int jesture_start_count = 0;

	switch(data->status)
	{
	case STATUS_JESTURE_START:
		jesture_start_count++;
		break;
	case STATUS_JESTURE_STOP:
		jesture_start_count--;
		break;
	case STATUS_BTN_ON:
		switch(data->id)
		{
		case 0:
			input_report_key(input_mouse->input, BTN_LEFT, 1);
			break;
		case 1:
			input_report_key(input_mouse->input, BTN_RIGHT, 1);
			break;
		default:
		case 2:
			input_report_key(input_mouse->input, BTN_MIDDLE, 1);
			break;
		}
		input_sync(input_mouse->input);
		break;
	case STATUS_BTN_OFF:
		switch(data->id)
		{
		case 0:
			input_report_key(input_mouse->input, BTN_LEFT, 0);
			break;
		case 1:
			input_report_key(input_mouse->input, BTN_RIGHT, 0);
			break;
		default:
		case 2:
			input_report_key(input_mouse->input, BTN_MIDDLE, 0);
			break;
		}
		input_sync(input_mouse->input);
		break;
	case STATUS_MOVE:
		if(jesture_start_count == 2) {	// multi scheduler

		} else {
			input_report_rel(input_mouse->input, REL_X, data->x);
			input_report_rel(input_mouse->input, REL_Y, data->y);
			input_sync(input_mouse->input);
		}
		break;
	}
}

static void vd5376_polling_handler(struct vd5376_mouse *mouse)
{
	printk(KERN_ERR "test\n");
}

static void vd5376_mouse_input_params(struct vd5376_mouse *mouse)
{
	struct input_dev *input = mouse->input;

        input->name = mouse->pdev->name;
        input->phys = mouse->pdev->name;
        input->id.bustype = BUS_HOST;
        input->dev.parent = &mouse->pdev->dev;
        input_set_drvdata(input, mouse);

	input_set_capability(input, EV_REL, REL_X);
        input_set_capability(input, EV_REL, REL_Y);
	input_set_capability(input, EV_KEY, BTN_LEFT);
	input_set_capability(input, EV_KEY, BTN_RIGHT);
	input_set_capability(input, EV_KEY, BTN_MIDDLE);
}

static int vd5376_mouse_probe(struct platform_device *pdev)
{
	int err, i;
	struct vd5376_mouse_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input_dev;
	struct vd5376_mouse *mouse;

	for(i=0;i<(pdata->ndatas);i++) {
		struct vd5376_platform_data *ddata = &pdata->data[i];
		ddata->vd5376_callback = (void*)vd5376_mouse_callback;
		ddata->id = i;
	}

	mouse = kzalloc(sizeof(struct vd5376_mouse), GFP_KERNEL);
	if (!mouse) {
		err = -ENOMEM;
		goto err_free;
	}

	mouse->pdev = pdev;
	mouse->pdata = pdata;

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input device\n", 
			__func__);
		goto err_free_mem;
	}

	mouse->input = input_dev;
	input_mouse = mouse;

	vd5376_mouse_input_params(mouse);

	err = input_register_device(input_dev);
	if (err) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, input_dev->name);
		goto err_free_device;
	}

	INIT_DELAYED_WORK(&mouse->dwork, vd5376_polling_handler);
	spin_lock_init(&mouse->lock);

	printk(KERN_INFO "%s: Start Finger Mouse Scheduler\n", 
		 __func__, input_dev->name);

	platform_set_drvdata(pdev, mouse);

	return 0;

 err_free_device:
	input_free_device(input_dev);
 err_free_mem:
	kfree(mouse);
 err_free:
	return err;
}

static int vd5376_mouse_remove(struct platform_device *pdev)
{
	struct vd5376_mouse *mouse = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&mouse->dwork);
	input_unregister_device(mouse->input);
	input_free_device(mouse->input);
	kfree(mouse);

	return 0;
}

static struct platform_driver vd5376_device_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DEVICE_NAME
	},
	.probe		= vd5376_mouse_probe,
	.remove		= vd5376_mouse_remove,
};

static int __init vd5376_schduler_init(void)
{
	return platform_driver_register(&vd5376_device_driver);
}

static void __exit vd5376_schduler_exit(void)
{
	platform_driver_unregister(&vd5376_device_driver);
}

module_init(vd5376_schduler_init);
module_exit(vd5376_schduler_exit);

MODULE_AUTHOR("Pyeongjeong Lee <leepjung@crz-tech.com>");
MODULE_DESCRIPTION("VD5376 Finger Mouse Scheduler Driver");
MODULE_LICENSE("GPL");

