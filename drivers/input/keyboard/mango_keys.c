/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mango_keys.h>
#include <plat/gpio-cfg.h>

#include <asm/gpio.h>

struct mango_button_data {
	struct mango_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
};

struct mango_keys_drvdata {
	struct input_dev *input;
	struct mango_button_data data[0];
};

static void mango_keys_report_event(struct mango_button_data *bdata)
{
	struct mango_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;

	if(button->led_gpio) {
		int led_state = state ^ button->led_active_low;
		gpio_direction_output(button->led_gpio, led_state);
	}

	input_event(input, type, button->code, !!state);
	input_sync(input);
}

static void mango_check_button(unsigned long _data)
{
	struct mango_button_data *data = (struct mango_button_data *)_data;

	mango_keys_report_event(data);
}

static irqreturn_t mango_keys_isr(int irq, void *dev_id)
{
	struct mango_button_data *bdata = dev_id;
	struct mango_keys_button *button = bdata->button;

	BUG_ON(irq != button->irq);

	if (button->debounce_interval)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(button->debounce_interval));
	else
		mango_keys_report_event(bdata);

	return IRQ_HANDLED;
}

static int __devinit mango_keys_probe(struct platform_device *pdev)
{
	struct mango_keys_platform_data *pdata = pdev->dev.platform_data;
	struct mango_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	ddata = kzalloc(sizeof(struct mango_keys_drvdata) +
			pdata->nbuttons * sizeof(struct mango_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "mango-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	ddata->input = input;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct mango_keys_button *button = &pdata->buttons[i];
		struct mango_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;
		setup_timer(&bdata->timer,
			    mango_check_button, (unsigned long)bdata);

		error = gpio_request(button->gpio, button->desc ?: "mango_keys");
		if (error < 0) {
			pr_err("mango-keys: failed to request GPIO %d,"
				" error %d\n", button->gpio, error);
			goto fail2;
		}

		s3c_gpio_cfgpin(button->gpio, button->config);
		s3c_gpio_setpull(button->gpio, button->pull);

		error = request_irq(button->irq, mango_keys_isr,
				    IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
				    button->desc ? button->desc : "mango_keys",
				    bdata);
		if (error) {
			pr_err("mango-keys: Unable to claim irq %d; error %d\n",
				button->irq, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		if (button->wakeup)
			wakeup = 1;

		if (button->led_gpio) {
			int led_state = button->led_active_low ? 1 : 0;

			error = gpio_request(button->led_gpio, button->led_desc ?: "mango_keys");
			if (error < 0) {
				pr_err("gpio-keys: failed to request GPIO %d,"
					" error %d\n", button->gpio, error);
				goto fail2;
			}

			s3c_gpio_cfgpin(button->led_gpio, button->led_config);
			s3c_gpio_setpull(button->led_gpio, S3C_GPIO_PULL_NONE);

			gpio_direction_output(button->led_gpio, led_state);
		}

		input_set_capability(input, type, button->code);
	}

	error = input_register_device(input);
	if (error) {
		pr_err("mango-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(pdata->buttons[i].irq, &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
		if(pdata->buttons[i].led_gpio)
			gpio_free(pdata->buttons[i].led_gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit mango_keys_remove(struct platform_device *pdev)
{
	struct mango_keys_platform_data *pdata = pdev->dev.platform_data;
	struct mango_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = pdata->buttons[i].irq;
		free_irq(irq, &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
		if(pdata->buttons[i].led_gpio)
			gpio_free(pdata->buttons[i].led_gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int mango_keys_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mango_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct mango_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = button->irq;
				enable_irq_wake(irq);
			}
		}
	}

	return 0;
}

static int mango_keys_resume(struct platform_device *pdev)
{
	struct mango_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct mango_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = button->irq;
				disable_irq_wake(irq);
			}
		}
	}

	return 0;
}
#else
#define mango_keys_suspend	NULL
#define mango_keys_resume	NULL
#endif

static struct platform_driver mango_keys_device_driver = {
	.probe		= mango_keys_probe,
	.remove		= __devexit_p(mango_keys_remove),
	.suspend	= mango_keys_suspend,
	.resume		= mango_keys_resume,
	.driver		= {
		.name	= "mango-keys",
		.owner	= THIS_MODULE,
	}
};

static int __init mango_keys_init(void)
{
	return platform_driver_register(&mango_keys_device_driver);
}

static void __exit mango_keys_exit(void)
{
	platform_driver_unregister(&mango_keys_device_driver);
}

module_init(mango_keys_init);
module_exit(mango_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pyeongjeong Lee <leepjung@crz-tech.com>");
MODULE_DESCRIPTION("Keyboard driver for MANGO");
