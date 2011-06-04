/*
 *  Dummy driver for Linux.
 *
 * 	This program is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU General Public License
 * 	as published by the Free Software Foundation; either version 2
 * 	of the License, or (at your option) any later version.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *	Karel Boek <karel.boek@raskenlund.com>
 */
static irqreturn_t dummy_irq(int irq, void *dev_id)
{
	printk(KERN_DEBUG "This dummy device handled an interrupt request!");
	return IRQ_HANDLED;
}

static int __devinit
dummy_probe(struct platform_device *pdev)
{
	struct resource *irq_res;
	int    ret;
	
	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		dev_err(&pdev->dev, "dummy_probe: IRQ not provided!\n");
		return -ENXIO;
	}
	
	ret = request_irq(irq_res->start, dummy_irq,
					IRQF_DISABLED, "dummy", NULL);
	if (ret < 0) {
		printk(KERN_ERR "dummy_probe: interrupt request failed.\n");
	}
	
	return 0;
}

static int __devexit
dummy_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver dummy_driver = {
	.driver	= {
		.name    = "dummy",
		.owner	 = THIS_MODULE,
	},
	.probe   = dummy_probe,
	.remove  = __devexit_p(dummy_remove),
};

static int __init
dummy_init(void)
{
	printk(KERN_INFO "Dummy Driver, V%s\n", CARDNAME, DRV_VERSION);

	return platform_driver_register(&dm9000_driver);
}

static void __exit
dummy_exit(void)
{
	platform_driver_unregister(&dm9000_driver);
}

module_init(dummy_init);
module_exit(dummy_exit);

MODULE_AUTHOR("Karel Boek");
MODULE_DESCRIPTION("Dummy driver used for testing IRQs");
MODULE_LICENSE("GPL");
