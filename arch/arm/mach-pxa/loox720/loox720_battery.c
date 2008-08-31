#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <linux/pda_power.h>
#include <linux/power_supply.h>

#include "../generic.h"

/*
 * External power
 */

static int __devinit loox720_battery_probe(struct platform_device *dev)
{
//	platform_device_register(&power_supply);
	return 0;
}

static struct platform_driver loox720_battery_driver = {
    .driver = {
	.name = "loox720-battery",
    },
    .probe = loox720_battery_probe,
};

static int __init loox720_battery_init(void)
{
    if (!machine_is_loox720())
	return -ENODEV;
    return platform_driver_register(&loox720_battery_driver);
}

static void __exit loox720_battery_exit(void)
{
    platform_driver_unregister(&loox720_battery_driver);
}

module_init(loox720_battery_init);
module_exit(loox720_battery_exit);

MODULE_AUTHOR ("Piotr Czechowicz");
MODULE_DESCRIPTION ("Battery support for FSC Loox 720");
MODULE_LICENSE ("GPL");
