#include <linux/module.h> 
#include <linux/err.h> 
#include <linux/platform_device.h> 
#include <linux/power_supply.h> 
#include <linux/types.h> 
#include <linux/pci.h> 
#include <linux/interrupt.h> 
#include <asm/io.h> 
#include <linux/slab.h>

#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
struct wake_lock	power_wakelock;
#endif
 
struct dummy_battery_data { 
	struct power_supply	battery; 
	struct power_supply	ac;
	struct power_supply	usb;

#if defined(CONFIG_HAS_WAKELOCK)
	int			locked;
#endif

	int			usb_online;
	int			ac_online;
} *dummy_data;

#if defined(CONFIG_HAS_WAKELOCK)
void Wakelock_Check(void)
{
	if(dummy_data->ac_online || dummy_data->usb_online)
	{
		if(dummy_data->locked == 0)
		{
			wake_lock(&power_wakelock);
			dummy_data->locked = 1;
		}
	}
	else
	{
		if(dummy_data->locked == 1)
		{
			wake_unlock(&power_wakelock);
			dummy_data->locked = 0;
		}
	}
}	
#endif
 
void isUSBconnected(bool usb_connect)
{
	if(!dummy_data) return;

	dummy_data->usb_online = usb_connect;
#if defined(CONFIG_HAS_WAKELOCK)
	Wakelock_Check();
#endif
}
EXPORT_SYMBOL(isUSBconnected);

void isACconnected(bool ac_connect)
{
	if(!dummy_data) return;
	
	dummy_data->ac_online = ac_connect;
#if defined(CONFIG_HAS_WAKELOCK)
	Wakelock_Check();
#endif
}
EXPORT_SYMBOL(isACconnected);

static int dummy_ac_get_property(struct power_supply *psy, 
                        enum power_supply_property psp, 
                        union power_supply_propval *val) 
{
	struct dummy_battery_data *data = container_of(psy, 
		struct dummy_battery_data, ac); 
	int ret = 0; 

	switch (psp) { 
	case POWER_SUPPLY_PROP_ONLINE: 
	        val->intval = data->ac_online; 
	         break; 
	default: 
	        ret = -EINVAL; 
	        break; 
	}
	return ret; 
} 

static int dummy_usb_get_property(struct power_supply *psy, 
                        enum power_supply_property psp, 
                        union power_supply_propval *val) 
{ 
	struct dummy_battery_data *data = container_of(psy, 
		struct dummy_battery_data, usb); 
	int ret = 0; 

	switch (psp) { 
	case POWER_SUPPLY_PROP_ONLINE: 
		val->intval = data->usb_online; 
		break; 
	default: 
		ret = -EINVAL; 
		break; 
	} 
	return ret; 
} 
 
static int dummy_battery_get_property(struct power_supply *psy, 
                                 enum power_supply_property psp, 
                                 union power_supply_propval *val) 
{
// Not Used
//        struct dummy_battery_data *data = container_of(psy, 
//                struct dummy_battery_data, battery); 
	int ret = 0; 

	switch (psp) { 
	case POWER_SUPPLY_PROP_STATUS: 
//		val->intval = POWER_SUPPLY_STATUS_CHARGING; 
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN; 
		break; 
	case POWER_SUPPLY_PROP_HEALTH: 
		val->intval = POWER_SUPPLY_HEALTH_GOOD; 
		break; 
	case POWER_SUPPLY_PROP_PRESENT: 
//		val->intval = 1; 
		val->intval = 0; 
		isACconnected(1);
		break; 
	case POWER_SUPPLY_PROP_TECHNOLOGY: 
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION; 
		break; 
	case POWER_SUPPLY_PROP_CAPACITY: 
		val->intval = 50; 
		break; 
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 250;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 3300;
		break;
	default: 
		ret = -EINVAL; 
		break; 
	} 

	return ret; 
} 
 
static enum power_supply_property dummy_battery_props[] = { 
	POWER_SUPPLY_PROP_STATUS, 
	POWER_SUPPLY_PROP_HEALTH, 
	POWER_SUPPLY_PROP_PRESENT, 
	POWER_SUPPLY_PROP_TECHNOLOGY, 
	POWER_SUPPLY_PROP_CAPACITY, 
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
}; 
 
static enum power_supply_property dummy_ac_props[] = { 
	POWER_SUPPLY_PROP_ONLINE, 
}; 

static enum power_supply_property dummy_usb_props[] = { 
	POWER_SUPPLY_PROP_ONLINE, 
}; 
  
static int dummy_battery_probe(struct platform_device *pdev) 
{ 
	int ret; 
	struct dummy_battery_data *data; 

	data = kzalloc(sizeof(*data), GFP_KERNEL); 
	if (data == NULL) { 
		ret = -ENOMEM; 
		goto err_data_alloc_failed; 
	}

	dummy_data = data;

	data->battery.properties = dummy_battery_props; 
	data->battery.num_properties = ARRAY_SIZE(dummy_battery_props); 
	data->battery.get_property = dummy_battery_get_property; 
	data->battery.name = "battery"; 
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY; 

	data->ac.properties = dummy_ac_props; 
	data->ac.num_properties = ARRAY_SIZE(dummy_ac_props); 
	data->ac.get_property = dummy_ac_get_property; 
	data->ac.name = "ac"; 
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	data->usb.properties = dummy_usb_props; 
	data->usb.num_properties = ARRAY_SIZE(dummy_usb_props); 
	data->usb.get_property = dummy_usb_get_property; 
	data->usb.name = "usb"; 
	data->usb.type = POWER_SUPPLY_TYPE_USB; 

	data->ac_online = 1;
	data->usb_online = 0;

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_init(&power_wakelock, WAKE_LOCK_SUSPEND, "power_wakelock");
	data->locked = 0;
	Wakelock_Check();
#endif


	ret = power_supply_register(&pdev->dev, &data->ac); 
	if (ret) 
		goto err_ac_failed; 

	ret = power_supply_register(&pdev->dev, &data->usb); 
	if (ret) 
		goto err_usb_failed; 

	ret = power_supply_register(&pdev->dev, &data->battery); 
	if (ret) 
		goto err_battery_failed; 

	platform_set_drvdata(pdev, data); 

	return 0; 
  
err_battery_failed: 
	power_supply_unregister(&data->usb); 
err_usb_failed:
	power_supply_unregister(&data->ac);
err_ac_failed: 
	kfree(data); 
err_data_alloc_failed: 
	return ret; 
} 
  
static int dummy_battery_remove(struct platform_device *pdev) 
{
	struct dummy_battery_data *data = platform_get_drvdata(pdev); 

	power_supply_unregister(&data->battery); 
	power_supply_unregister(&data->usb); 
	power_supply_unregister(&data->ac); 

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_destroy(&power_wakelock);
#endif

	kfree(data); 
	dummy_data = NULL; 

	return 0; 
} 
 
static struct platform_driver dummy_battery_device = { 
	.probe		 = dummy_battery_probe, 
	.remove		 = dummy_battery_remove, 
	.driver = { 
		.name = "dummy-battery" 
	}
}; 
 
static int __init dummy_battery_init(void) 
{ 
	return platform_driver_register(&dummy_battery_device); 
} 
 
static void __exit dummy_battery_exit(void) 
{ 
	platform_driver_unregister(&dummy_battery_device); 
} 
  
module_init(dummy_battery_init); 
module_exit(dummy_battery_exit); 
 
MODULE_AUTHOR("Pyeong Jeong Lee leepjung@crz-tech.com"); 
MODULE_LICENSE("GPL"); 
MODULE_DESCRIPTION("Battery driver for system using AC  & USB supplied dummy battery"); 

