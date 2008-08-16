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

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <asm/arch/pxa2xx_spi.h>

//#include <linux/touchscreen-adc.h>
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


/* borrowed from lubbock.c */

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 0,
};

static struct platform_device pxa_ssp = {
	.name		= "pxa2xx-spi",
	.id		= 1,
	.dev = {
		.platform_data	= &pxa_ssp_master_info,
	},
};

static int loox720_ads7846_pendown_state(void)
{
	/* TS_BUSY is bit 8 in LUB_MISC_RD, but pendown is irq-only */
	return 0;
}

static struct ads7846_platform_data ads_info = {
	.model			= 7846,
	.vref_delay_usecs	= 100,		/* internal, no cap */
	.get_pendown_state	= loox720_ads7846_pendown_state,
	// .x_plate_ohms		= 500,	/* GUESS! */
	// .y_plate_ohms		= 500,	/* GUESS! */
};
static void ads7846_cs(u32 command)
{
	// no idea what this should do.. 

	//static const unsigned	TS_nCS = 1 << 11;
	//lubbock_set_misc_wr(TS_nCS, (command == PXA2XX_CS_ASSERT) ? 0 : TS_nCS);
}

static struct pxa2xx_spi_chip ads_hw = {
	.tx_threshold		= 1,
	.rx_threshold		= 2,
	.cs_control		= ads7846_cs,
};

static struct spi_board_info spi_board_info[] __initdata = { {
	.modalias	= "ads7846",
	.platform_data	= &ads_info,
	.controller_data = &ads_hw,
	.irq		= LOOX720_IRQ(TOUCHPANEL_IRQ_N),
	.max_speed_hz	= 120000 /* max sample rate at 3V */
				* 26 /* command + data + overhead */,
	.bus_num	= 1,
	.chip_select	= 0,
},
};

#if 0
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
#endif

static int __devinit loox720_ts_probe(struct platform_device *dev)
{
    //platform_device_register(&ads7846_ssp);
    //platform_device_register(&loox720_ts);
    spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
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
