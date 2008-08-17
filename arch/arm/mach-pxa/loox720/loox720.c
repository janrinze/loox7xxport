/*
 *
 * Hardware definitions for HP iPAQ Handheld Computers
 *
 * Copyright 2004 Hewlett-Packard Company.
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * History:
 *
 * 2004-11-2004	Michael Opdenacker	Preliminary version
 * 2004-12-16   Todd Blumer
 * 2004-12-22   Michael Opdenacker	Added USB management
 * 2005-01-30   Michael Opdenacker	Improved Asic3 settings and initialization
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/gpio.h>

#include <linux/serial.h>
#include <asm/arch/loox720.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/regs-ssp.h>
#include <linux/gpio_keys.h>
#include <asm/arch/pxa2xx-regs.h>
#include <asm/arch/pxa2xx_spi.h>
#include <asm/arch/udc.h>
#include <asm/arch/audio.h>
#include <asm/arch/ohci.h>
#include <asm/arch/irda.h>
#include <asm/arch/mmc.h>
#include <asm/arch/irqs.h>

#include <linux/backlight.h>

#include "../generic.h"
#include "loox720_core.h"
#include <asm/arch/loox720-cpld.h>
/*
#include <linux/adc_battery.h>
*/

/*
 * IRDA
 */

static void loox_irda_transceiver_mode(struct device *dev, int mode)
{
	unsigned long flags;

	local_irq_save(flags);
/*
	if (mode & IR_FIRMODE)
		SET_LOOX720_GPIO_N(IR_ON, 1);
	else
		SET_LOOX720_GPIO_N(IR_ON, 0);
*/
	if (mode & IR_OFF)
		SET_LOOX720_GPIO_N(IR_ON, 1);
	else
		SET_LOOX720_GPIO_N(IR_ON, 0);

	local_irq_restore(flags);
}

static struct pxaficp_platform_data loox_ficp_info = {
	.transceiver_cap  = IR_SIRMODE | IR_FIRMODE | IR_OFF,
	.transceiver_mode = loox_irda_transceiver_mode,
};

/* Uncomment the following line to get serial console via SIR work from
 * the very early booting stage. This is not useful for end-user.
 */
/* #define EARLY_SIR_CONSOLE */
/*
#define IR_TRANSCEIVER_ON \
	SET_LOOX720_GPIO_N(IR_ON, 1)

#define IR_TRANSCEIVER_OFF \
	SET_LOOX720_GPIO_N(IR_ON, 0)

static void loox_irda_configure(int state)
{
	// Switch STUART RX/TX pins to SIR
	pxa_gpio_mode(GPIO_NR_LOOX720_STD_RXD_MD);
	pxa_gpio_mode(GPIO_NR_LOOX720_STD_TXD_MD);

	// make sure FIR ICP is off
	ICCR0 = 0;

	switch (state) {

	case PXA_UART_CFG_POST_STARTUP:
		// configure STUART for SIR
		STISR = STISR_XMODE | STISR_RCVEIR | STISR_RXPL;
		IR_TRANSCEIVER_ON;
		break;

	case PXA_UART_CFG_PRE_SHUTDOWN:
		STISR = 0;
		IR_TRANSCEIVER_OFF;
		break;
	}
}

static void loox_irda_set_txrx(int txrx)
{
	unsigned old_stisr = STISR;
	unsigned new_stisr = old_stisr;

	if (txrx & PXA_SERIAL_TX) {
		// Ignore RX if TX is set
		txrx &= PXA_SERIAL_TX;
		new_stisr |= STISR_XMITIR;
	} else
		new_stisr &= ~STISR_XMITIR;

	if (txrx & PXA_SERIAL_RX)
		new_stisr |= STISR_RCVEIR;
	else
		new_stisr &= ~STISR_RCVEIR;

	if (new_stisr != old_stisr) {
		while (!(STLSR & LSR_TEMT))
			;
		IR_TRANSCEIVER_OFF;
		STISR = new_stisr;
		IR_TRANSCEIVER_ON;
	}
}

static int loox_irda_get_txrx (void)
{
	return ((STISR & STISR_XMITIR) ? PXA_SERIAL_TX : 0) |
	       ((STISR & STISR_RCVEIR) ? PXA_SERIAL_RX : 0);
}

static struct platform_pxa_serial_funcs loox_pxa_irda_funcs = {
	.configure = loox_irda_configure,
	.set_txrx  = loox_irda_set_txrx,
	.get_txrx  = loox_irda_get_txrx,
};


// Initialization code
static void __init loox_map_io(void)
{
	pxa_map_io();
	pxa_set_stuart_info(&loox_pxa_irda_funcs);
#ifdef EARLY_SIR_CONSOLE
	loox_irda_configure(NULL, 1);
	loox_irda_set_txrx(NULL, PXA_SERIAL_TX);
#endif
}
*/

/*
 * Bluetooth - Relies on other loadable modules, like ASIC3 and Core,
 * so make the calls indirectly through pointers. Requires that the
 * loox720 bluetooth module be loaded before any attempt to use
 * bluetooth (obviously).
 */

#ifdef CONFIG_LOOX720_BT

static struct loox720_bt_funcs bt_funcs;

static void
loox720_bt_configure( int state )
{
        if (bt_funcs.configure != NULL)
                bt_funcs.configure( state );
}

static struct platform_pxa_serial_funcs loox720_pxa_bt_funcs = {
        .configure = loox720_bt_configure,
};

#endif

/* PXA2xx Keys */

static struct gpio_keys_button loox720_button_table[] = {
	{ KEY_POWER, GPIO_NR_LOOX720_KEY_ON, 1 },
};

static struct gpio_keys_platform_data loox720_pxa_keys_data = {
	.buttons = loox720_button_table,
	.nbuttons = ARRAY_SIZE(loox720_button_table),
};

static struct platform_device loox720_pxa_keys = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &loox720_pxa_keys_data,
	},
};

#if 0
// SPI START

static struct resource pxa_spi_nssp_resources[] = {
	[0] = {
		.start	= __PREG(SSCR0_P1), /* Start address of NSSP */
		.end	= __PREG(SSCR0_P1) + 0x2c, /* Range of registers */
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP, /* NSSP IRQ */
		.end	= IRQ_SSP,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pxa2xx_spi_master pxa_nssp_master_info = {
	.ssp_type = PXA27x_SSP, /* Type of SSP */
	.clock_enable = CKEN23_SSP1, /* NSSP Peripheral clock */
	.num_chipselect = 1, /* Matches the number of chips attached to NSSP */
	.enable_dma = 0, /* Enables NSSP DMA */
};

static struct platform_device pxa_spi_nssp = {
	.name = "pxa2xx-spi", /* MUST BE THIS VALUE, so device match driver */
	.id = 1, /* Bus number, MUST MATCH SSP number 1..n */
	.resource = pxa_spi_nssp_resources,
	.num_resources = ARRAY_SIZE(pxa_spi_nssp_resources),
	.dev = {
		.platform_data = &pxa_nssp_master_info, /* Passed to driver */
	},
};

// SPI END
#endif

static struct platform_device loox720_ts = {
	.name = "loox720-ts",
};

#if 0
#define LOOX720_MAX_INTENSITY 0xc8
#define LOOX720_DEFAULT_INTENSITY (LOOX720_MAX_INTENSITY / 4)

static void loox720_set_bl_intensity(int intensity)
{
	if (intensity < 7) intensity = 0;

	PWM_CTRL0 = 1;
	PWM_PWDUTY0 = intensity;
	PWM_PERVAL0 = LOOX720_MAX_INTENSITY;

	if (intensity > 0) {
		loox720_egpio_set_bit(LOOX720_CPLD_BACKLIGHT_BIT, 1);
		pxa_set_cken(CKEN_PWM0, 1);
	} else {
		loox720_egpio_set_bit(LOOX720_CPLD_BACKLIGHT_BIT, 0);
                pxa_set_cken(CKEN_PWM0, 0);
	}
}

static struct corgibl_machinfo loox720_bl_machinfo = {
        .default_intensity = LOOX720_DEFAULT_INTENSITY,
        .limit_mask = 0xff,
        .max_intensity = LOOX720_MAX_INTENSITY,
        .set_bl_intensity = loox720_set_bl_intensity,
};

struct platform_device loox720_bl = {
        .name = "corgi-bl",
        .dev = {
    		.platform_data = &loox720_bl_machinfo,
	},
};
#endif

static struct platform_device loox720_buttons = {
	.name = "loox720-buttons",
};

static struct platform_device loox720_battery = {
	.name = "loox720-battery",
};

static struct loox720_core_funcs core_funcs;

static struct platform_device loox720_core = {
	.name = "loox720-core",
	.id		= -1,
	.dev = {
		.platform_data = &core_funcs,
	},
};

static int
udc_detect(void)
{
        int detected = (GET_LOOX720_GPIO(USB_DETECT_N)==0);
	printk (KERN_NOTICE "udc_detect: %d\n", detected);
	return detected;
}

static void
udc_command(int cmd)
{
	switch (cmd)
	{
		case PXA2XX_UDC_CMD_DISCONNECT:
			printk (KERN_NOTICE "USB cmd disconnect\n");
			loox720_egpio_set_bit(LOOX720_CPLD_USB_PULLUP_BIT, 0);
			break;

		case PXA2XX_UDC_CMD_CONNECT:
			printk (KERN_NOTICE "USB cmd connect\n");
			loox720_egpio_set_bit(LOOX720_CPLD_USB_PULLUP_BIT, 1);
			break;
		default:
			printk (KERN_ERR "USB: invalid command: %d\n", cmd);
	}
}

static struct pxa2xx_udc_mach_info loox720_udc_info __initdata = {
	.udc_is_connected = udc_detect,
	.udc_command      = udc_command,
};

static int loox720_ohci_init(struct device *dev)
{
	/* missing GPIO setup here */

	/* no idea what this does, got the values from haret
	UHCHR = (UHCHR | UHCHR_SSEP2 | UHCHR_PCPL | UHCHR_CGR) &
			    ~(UHCHR_SSEP1 | UHCHR_SSEP3 | UHCHR_SSE);
		we don't know yet how to init.. */
	UHCHR = (UHCHR | UHCHR_SSEP3 | UHCHR_PSPL | UHCHR_CGR) &
			    ~(UHCHR_SSEP1 | UHCHR_SSEP2 | UHCHR_SSE);
	return 0;
}

static struct pxaohci_platform_data loox720_ohci_info = {
	        .port_mode = PMM_PERPORT_MODE,
		.init = loox720_ohci_init,
};

#ifdef CONFIG_LOOX720_BT
/* Bluetooth */

static struct platform_device loox720_bt = {
        .name = "loox720-bt",
        .id = -1,
        .dev = {
                .platform_data = &bt_funcs,
        },
};
#endif

extern struct platform_device loox7xx_flash;

/*
 * MMC/SD
 */

static int loox7xx_mci_init(struct device *dev,
				irq_handler_t detect_irq, void *data)
{
	int err;
	err = gpio_request(GPIO_NR_LOOX720_MMC_DETECT_N, "SD_DETECT_N");
	if (err)
		goto err_request_detect;
	gpio_direction_input(GPIO_NR_LOOX720_MMC_DETECT_N);
	err = request_irq(IRQ_GPIO(GPIO_NR_LOOX720_MMC_DETECT_N), detect_irq,
				IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"SD card detect", data);
	if (err)
		goto err_request_irq;
	err = gpio_request(GPIO_NR_LOOX720_MMC_RO, "SD_READONLY");
	if (err)
		goto err_request_readonly;
	gpio_direction_input(GPIO_NR_LOOX720_MMC_RO);

	return 0;

err_request_readonly:
	free_irq(IRQ_GPIO(GPIO_NR_LOOX720_MMC_DETECT_N), data);
err_request_irq:
	gpio_free(GPIO_NR_LOOX720_MMC_DETECT_N);
err_request_detect:
	return err;
}

static void loox7xx_mci_setpower(struct device *dev, unsigned int vdd)
{
	struct pxamci_platform_data *pdata = dev->platform_data;

	loox720_egpio_set_bit(LOOX720_CPLD_SD_BIT, (1 << vdd) & pdata->ocr_mask);
}

static int loox7xx_mci_get_ro(struct device *dev)
{
	return gpio_get_value(GPIO_NR_LOOX720_MMC_RO);
}

static void loox7xx_mci_exit(struct device *dev, void *data)
{
	gpio_free(GPIO_NR_LOOX720_MMC_RO);
	free_irq(IRQ_GPIO(GPIO_NR_LOOX720_MMC_DETECT_N), data);
	gpio_free(GPIO_NR_LOOX720_MMC_DETECT_N);
}

static struct pxamci_platform_data loox7xx_mci_info = {
	.ocr_mask 	= MMC_VDD_32_33|MMC_VDD_33_34,
	.detect_delay	= 20,
	.init     	= loox7xx_mci_init,
	.get_ro   	= loox7xx_mci_get_ro,
	.setpower 	= loox7xx_mci_setpower,
	.exit     	= loox7xx_mci_exit,
};

/*
 * Loox 720
 */
 
static struct platform_device *devices[] __initdata = {
	&loox720_core,
#if 0
	&pxa_spi_nssp,
	&loox720_buttons,
#endif
#ifdef CONFIG_LOOX720_TS
	&loox720_ts,
#endif
#if 0
	&loox720_pxa_keys,
	&loox720_bl,
	&loox720_battery,
#endif
#ifdef CONFIG_LOOX720_BT
	&loox720_bt,
#endif
#ifdef CONFIG_LOOX720_FLASH
	&loox7xx_flash,
#endif
};

static void __init loox720_init( void )
{
#if 0	// keep for reference, from bootldr
	GPSR0 = 0x0935ede7;
	GPSR1 = 0xffdf40f7;
	GPSR2 = 0x0173c9f6;
	GPSR3 = 0x01f1e342;
	GPCR0 = ~0x0935ede7;
	GPCR1 = ~0xffdf40f7;
	GPCR2 = ~0x0173c9f6;
	GPCR3 = ~0x01f1e342;
	GPDR0 = 0xda7a841c;
	GPDR1 = 0x68efbf83;
	GPDR2 = 0xbfbff7db;
	GPDR3 = 0x007ffff5;
	GAFR0_L = 0x80115554;
	GAFR0_U = 0x591a8558;
	GAFR1_L = 0x600a9558;
	GAFR1_U = 0x0005a0aa;
	GAFR2_L = 0xa0000000;
	GAFR2_U = 0x00035402;
	GAFR3_L = 0x00010000;
	GAFR3_U = 0x00001404;
	MSC0 = 0x25e225e2;
	MSC1 = 0x12cc2364;
	MSC2 = 0x16dc7ffc;
#endif

	ARB_CNTRL = ARB_CORE_PARK | 0x234;

	pxa_set_udc_info(&loox720_udc_info);
	pxa_set_ficp_info(&loox_ficp_info);
	pxa_set_ohci_info(&loox720_ohci_info);
#ifdef CONFIG_LOOX720_BT
	pxa_set_btuart_info(&loox720_pxa_bt_funcs);
#endif
	loox7xx_mci_info.detect_delay = msecs_to_jiffies(250);
	pxa_set_mci_info(&loox7xx_mci_info);

	platform_add_devices( devices, ARRAY_SIZE(devices) );
}

/* Loox720 has 128MB RAM */
//static void __init loox720_fixup(struct machine_desc *desc,
//                      struct tag *tags, char **cmdline, struct meminfo *mi)
//{
//        mi->nr_banks=1;
//        mi->bank[0].start = 0xa8000000;
//        mi->bank[0].node = 0;
//        mi->bank[0].size = (128*1024*1024);
//}

MACHINE_START(LOOX720, "FSC Loox 720")
//	BOOT_MEM(0xaa000000, 0x40000000, io_p2v(0x40000000))
	.phys_io = 0x40000000,
	.io_pg_offst = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params	= CONFIG_DRAM_BASE + 0x100,
	.map_io		= pxa_map_io,
	.init_irq	= pxa27x_init_irq,
	.timer		= &pxa_timer,
//	.fixup		= loox720_fixup,
	.init_machine	= loox720_init,
MACHINE_END
