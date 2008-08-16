/*
 * Touchscreen driver for Axim X50/X51(v).
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Copyright (C) 2007 Pierre Gaufillet
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ads7846.h>
#include <linux/touchscreen-adc.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach/irq.h>
#include <asm/mach/arch.h>
#include <asm/arch/bitfield.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/loox720-gpio.h>

#include "../generic.h"

static struct ads7846_ssp_platform_data loox720_ts_ssp_params = {
    .port = 1,
    .pd_bits = 1,
    .freq = 50000,
};
static struct platform_device ads7846_ssp     = {
    .name = "ads7846-ssp",
    .id = -1,
    .dev = {
        .platform_data = &loox720_ts_ssp_params,
    }
};

static struct tsadc_platform_data loox720_ts_params = {
//    .pen_irq = LOOX720_IRQ(TOUCHPANEL_IRQ_N),
    .pen_gpio = GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N,
    .x_pin = "ads7846-ssp:x",
    .y_pin = "ads7846-ssp:y",
    .z1_pin = "ads7846-ssp:z1",
    .z2_pin = "ads7846-ssp:z2",
    .pressure_factor = 100000,
    .min_pressure = 5,
    .delayed_pressure=1,
    .max_jitter = 8,
    .num_xy_samples = 10,
};
static struct resource loox720_pen_irq = {
    .start = IRQ_GPIO(GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N),
    .end = IRQ_GPIO(GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N),
    .flags = IORESOURCE_IRQ,
};
static struct platform_device loox720_ts        = {
    .name = "ts-adc",
    .id = -1,
    .resource = &loox720_pen_irq,
    .num_resources = 1,
    .dev = {
        .platform_data = &loox720_ts_params,
    }
};

static int __devinit loox720_ts_probe(struct platform_device *dev)
{
    platform_device_register(&ads7846_ssp);
    platform_device_register(&loox720_ts);
    return 0;
}

static struct platform_driver loox720_ts_driver = {
    .driver        = {
        .name       = "loox720-ts",
    },
    .probe          = loox720_ts_probe,
};

static int __init loox720_ts_init(void)
{
    if (!machine_is_loox720())
        return -ENODEV;

    return platform_driver_register(&loox720_ts_driver);
}

static void __exit loox720_ts_exit(void)
{
    platform_driver_unregister(&loox720_ts_driver);
}


module_init(loox720_ts_init);
module_exit(loox720_ts_exit);

MODULE_AUTHOR ("Pierre Gaufillet");
MODULE_DESCRIPTION ("Touchscreen support for LOOX718/720");
MODULE_LICENSE ("GPL");
