/* Bluetooth interface driver for TI BRF6150 on Loox 720
 * (Based on TI BRF6150 driver for HX4700)
 *
 * Copyright (c) 2005 SDG Systems, LLC
 *
 * 2005-04-21   Todd Blumer             Created.
 * 2007-07-25	Tomasz Figa		Modified for Loox 720.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/leds.h>

#include <asm/hardware.h>
#include <asm/arch/serial.h>
#include <asm/arch/loox720.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/loox720-cpld.h>

static void
loox720_bt_configure( int state )
{
	int tries;

	printk( KERN_NOTICE "loox720 configure bluetooth: %d\n", state );
	switch (state) {
	
	case PXA_UART_CFG_PRE_STARTUP:
		break;

	case PXA_UART_CFG_POST_STARTUP:
		/* pre-serial-up hardware configuration */
		loox720_egpio_set_bit( LOOX720_CPLD_BLUETOOTH_POWER, 1 );
		loox720_egpio_set_bit( LOOX720_CPLD_BLUETOOTH_RADIO, 1 );
		loox720_enable_led( LOOX720_LED_LEFT, LOOX720_LED_COLOR_A | LOOX720_LED_BLINK );
		SET_LOOX720_GPIO( CPU_BT_RESET_N, 0 );
		mdelay(1);
		SET_LOOX720_GPIO( CPU_BT_RESET_N, 1 );

		/*
		 * BRF6150's RTS goes low when firmware is ready
		 * so check for CTS=1 (nCTS=0 -> CTS=1). Typical 150ms
		 */
		printk( KERN_NOTICE "loox720_bt.c: Waiting for firmware...\n");
		tries = 0;
		do {
			mdelay(10);
		} while ((BTMSR & MSR_CTS) == 0 && tries++ < 50);
		if(tries>49)
		{
			printk( KERN_NOTICE "loox720_bt.c: Firmware timeout!\n");
		}
		break;

	case PXA_UART_CFG_PRE_SHUTDOWN:

		loox720_egpio_set_bit( LOOX720_CPLD_BLUETOOTH_RADIO, 0 );
		loox720_egpio_set_bit( LOOX720_CPLD_BLUETOOTH_POWER, 0 );
		loox720_disable_led( LOOX720_LED_LEFT, LOOX720_LED_COLOR_A );
		mdelay(1);
		break;

	default:
		break;
	}
}


static int
loox720_bt_probe( struct platform_device *pdev )
{
	struct loox720_bt_funcs *funcs = pdev->dev.platform_data;

	printk( KERN_NOTICE "loox720_bt.c: Probing device...\n");

	/* configure bluetooth UART */
	pxa_gpio_mode( GPIO_NR_LOOX720_BT_RXD_MD );
	pxa_gpio_mode( GPIO_NR_LOOX720_BT_TXD_MD );
	pxa_gpio_mode( GPIO_NR_LOOX720_BT_UART_CTS_MD );
	pxa_gpio_mode( GPIO_NR_LOOX720_BT_UART_RTS_MD );

	funcs->configure = loox720_bt_configure;

	return 0;
}

static int
loox720_bt_remove( struct platform_device *pdev )
{
	struct loox720_bt_funcs *funcs = pdev->dev.platform_data;

	funcs->configure = NULL;

	return 0;
}

static struct platform_driver bt_driver = {
	.driver = {
		.name     = "loox720-bt",
	},
	.probe    = loox720_bt_probe,
	.remove   = loox720_bt_remove,
};

static int __init
loox720_bt_init( void )
{
	printk(KERN_NOTICE "loox720 Bluetooth Driver\n");
	return platform_driver_register( &bt_driver );
}

static void __exit
loox720_bt_exit( void )
{
	platform_driver_unregister( &bt_driver );
}

module_init( loox720_bt_init );
module_exit( loox720_bt_exit );

MODULE_AUTHOR("Todd Blumer, SDG Systems, LLC");
MODULE_DESCRIPTION("loox720 Bluetooth Support Driver");
MODULE_LICENSE("GPL");

/* vim600: set noexpandtab sw=8 ts=8 :*/

