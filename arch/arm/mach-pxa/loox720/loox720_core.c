/* Core Hardware driver for FSC Loox 720
 *
 * Authors: Giuseppe Zompatori <giuseppe_zompatori@yahoo.it>
 *
 * based on previews work, see below:
 *
 * Copyright (c) 2005 SDG Systems, LLC
 *
 * 2005-03-29   Todd Blumer             Converted  basic structure to support hx4700
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/loox720-cpld.h>
#include <asm/arch/loox720.h>
#include <asm/arch/irqs.h>

//#include <linux/dpm.h>
//#include <asm/arch/pxa-pm_ll.h>

#include "loox720_core.h"

static u32 save[6];

static unsigned int ac_irq = 0xffffffff;
static unsigned int battery_irq = 0xffffffff;
static unsigned int usb_irq = 0xffffffff;

static void update_battery_charging(void)
{
	int usb_conn = (gpio_get_value(GPIO_NR_LOOX720_USB_DETECT_N) == 0);
	int ac_conn = (gpio_get_value(GPIO_NR_LOOX720_AC_IN_N) == 0);

	if (ac_conn || usb_conn) {
		int battery_full = gpio_get_value(GPIO_NR_LOOX720_BATTERY_FULL_N) == 0;
		printk(KERN_INFO "battery: ac: %d; usb: %d; battery_full: %d\n", ac_conn, usb_conn, battery_full);
		if (ac_conn)
			gpio_set_value(GPIO_NR_LOOX720_USB_CHARGE_N, 1);
		else
			gpio_set_value(GPIO_NR_LOOX720_USB_CHARGE_N, 0);
		if (!battery_full) {
			gpio_set_value(GPIO_NR_LOOX720_CHARGE_EN_N, 0);
			loox720_enable_led(LOOX720_LED_RIGHT, LOOX720_LED_COLOR_C | LOOX720_LED_BLINK);
		} else {
			gpio_set_value(GPIO_NR_LOOX720_CHARGE_EN_N, 1);
			loox720_enable_led(LOOX720_LED_RIGHT, LOOX720_LED_COLOR_C);
		}
	} else {
		printk(KERN_INFO "battery: external power is disconnected.\n");
		gpio_set_value(GPIO_NR_LOOX720_CHARGE_EN_N, 1);
		loox720_disable_led(LOOX720_LED_RIGHT, LOOX720_LED_COLOR_C);
	}
}

static void update_battery_chip(void)
{
	int connected = (gpio_get_value(GPIO_NR_LOOX720_AC_IN_N) == 0) || (gpio_get_value(GPIO_NR_LOOX720_USB_DETECT_N) == 0);
	printk( KERN_INFO "battery: charging controller enabled=%d\n", connected );
	loox720_egpio_set_bit(LOOX720_CPLD_BATTERY_BIT, connected);
}

static int
battery_isr(int irq, void *data)
{
	int full = gpio_get_value(GPIO_NR_LOOX720_BATTERY_FULL_N) == 0;
	printk( KERN_INFO "battery_isr: battery_full=%d\n", full);

	update_battery_charging();

	return IRQ_HANDLED;
}

static int
ac_isr(int irq, void *data)
{
	int connected;

	if (irq != ac_irq)
		return IRQ_NONE;

	connected = gpio_get_value(GPIO_NR_LOOX720_AC_IN_N) == 0;
	printk( KERN_INFO "ac_isr: connected=%d\n", connected );

	update_battery_charging();
	update_battery_chip();

	return IRQ_HANDLED;
}

static int
usb_isr(int irq, void *data)
{
	int connected;

	if (irq != usb_irq)
		return IRQ_NONE;

	connected = gpio_get_value(GPIO_NR_LOOX720_USB_DETECT_N) == 0;
	printk( KERN_INFO "usb_isr: connected=%d\n", connected );

	update_battery_charging();
	update_battery_chip();

	return IRQ_HANDLED;
}

static int loox720_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* 0x20c2 is HTC clock value
	 * CLOCK_CDEX_SOURCE            2
	 * CLOCK_CDEX_SPI               0
	 * CLOCK_CDEX_OWM               0
	 *
	 * CLOCK_CDEX_PWM0              0
	 * CLOCK_CDEX_PWM1              0
	 * CLOCK_CDEX_LED0              1
	 * CLOCK_CDEX_LED1              1
	 *
	 * CLOCK_CDEX_LED2              0
	 * CLOCK_CDEX_SD_HOST           0
	 * CLOCK_CDEX_SD_BUS            0
	 * CLOCK_CDEX_SMBUS             0
	 *
	 * CLOCK_CDEX_CONTROL_CX        0
	 * CLOCK_CDEX_EX0               1
	 * CLOCK_CDEX_EX1               0
	 *
	asic3_set_clock_cdex(&loox720_asic3.dev, 0xffff, 0x21c2);

	*egpios = 0;    // turn off all egpio power */

	/*
	 * Note that WEP1 wake up event is used by bootldr to set the
	 * LEDS when power is applied/removed for charging.
	 */
	PWER = PWER_RTC | PWER_GPIO0 | PWER_GPIO1 | PWER_GPIO9 | PWER_GPIO11 | PWER_GPIO12 | PWER_GPIO13;
	PFER = PWER_GPIO0 | PWER_GPIO1 | PWER_GPIO9 | PWER_GPIO11 | PWER_GPIO13;                                // Falling Edge Detect
	PRER = PWER_GPIO0 | PWER_GPIO1 | PWER_GPIO9 | PWER_GPIO12 | PWER_GPIO13;                // Rising Edge Detect
	PKWR = PWER_GPIO0;

	PGSR0 = 0x01308018; // hx4700: 0x080DC01C
	PGSR1 = 0x008F0112; // hx4700: 0x34CF0002
	PGSR2 = 0x0C03C000; // hx4700: 0x0123C18C
	PGSR3 = 0x00000C80; // hx4700: 0x00100202

	/* These next checks are specifically for charging.  We want to enable
	* it if it is already enabled */
	/* Check for charge enable, GPIO 72 */
//      if(GPLR2 & (1 << 8)) {
	/* Set it */
//              PGSR2 |= (1U << 8);
//      } else {
	/* Clear it */
//              PGSR2 &= ~(1U << 8);
//      }
	/* Check for USB_CHARGE_RATE, GPIO 96 */
//      if(GPLR3 & (1 << 0)) {
	/* Set it */
//              PGSR3 |= (1U << 0);
//      } else {
	/* Clear it */
//              PGSR3 &= ~(1U << 0);
//      }

	PCFR = PCFR_GPROD|PCFR_GPR_EN|PCFR_OPDE|PCFR_PI2CEN; /* was 0x1091; */
	/* The 2<<2 below turns on the Power Island state preservation
	 * and counters.  This allows us to wake up bootldr after a
	 * period of time, and it can set the LEDs correctly based on
	 * the power state.  The bootldr turns it off when it's
	 * charged.
	z        */
	PSLR=0xcc400000 | (2 << 2);

	return 0;
}

static int loox720_resume(struct platform_device *pdev)
{
	loox720_cpld_resume();
	__asm__ volatile ("mrc p14, 0, r3, c6, c0, 0\n"
	                  "orr r3, r3, #0x0008\n"
	                  "mcr p14, 0, r3, c6, c0, 0\n"
                  :::"r3");

	return 0;
}

static void test_resume(void) __attribute__((naked));
static void test_resume(void)
{
	// our private resume handler restores the data at 0xA8024000
	// and then turns over control to the pxa_resume handler
	asm volatile("mov r0,#0xa8000000 \n");      // base memory
	asm volatile("orr r1,r0,#0x24000 \n");              // memory offset
	asm volatile("ldr r2,[r1,#20] \n");         // r2 is store
	asm volatile("ldr r3,[r1,#16] \n");         // r3 is our resume

	asm volatile("ldr r0,[r2],#4 \n");          // restore word at 0
	asm volatile("str r0,[r1],#4 \n");          // ..
	asm volatile("ldr r0,[r2],#4 \n");          // restore a word at 4
	asm volatile("str r0,[r1],#4 \n");          // ..
	asm volatile("ldr r0,[r2],#4 \n");          // restore a word at 8
	asm volatile("str r0,[r1],#4 \n");          // ..
	asm volatile("ldr r0,[r2],#4 \n");          // restore a word at 12
	asm volatile("str r0,[r1],#4 \n");          // ..
	asm volatile("ldr r0,[r2],#4 \n");          // restore a word at 16
	asm volatile("str r0,[r1],#4 \n");          // ..
	asm volatile("ldr r0,[r2],#4 \n");          // restore a word at 20
	asm volatile("str r0,[r1],#4 \n");          // ..
//    asm volatile("mov r0,#0x10000000 \n");      // CPLD address
//    asm volatile("mov r1,#0xa800 \n");          // blue led
//    asm volatile("orr r1,r1,#0x0C \n");         // ..
//    asm volatile("strh r1,[r0,#16] \n");        // set LEDs

	asm volatile("mov r15,r3 \n");
	asm volatile("mov r0,r0 \n");
	asm volatile("mov r0,r0 ");
}
#if ( CONFIG_DRAM_BASE == 0xA8000000)

static void
loox720_pxa_ll_pm_suspend(unsigned long resume_addr)
{
//    register unsigned int R10 asm("r10");
	int i;
	u32 csum, tmp;
	u32 * p;

	tmp =  virt_to_phys((unsigned int)save);
	if ((tmp>=0xa8024000)&&(tmp<=0xa824036))
		panic("Resume storage should not be overwritten");

	tmp = virt_to_phys((unsigned int)test_resume);

	if ((tmp>=0xa8024000)&&(tmp<=0xa824036))
		panic("Resume handler should not be overwritten");
	/* Make sure that bootloader will not found reset pattern
	 * by setting special memory places to 0x00 */

	p = phys_to_virt(0xA8025304);
	if (!p)
		panic("Tried to put sleep informations into unmapped memory!");
	if (*p == 0x1A2B3C4D)
		*p = 0;

	p = phys_to_virt(0xA8025314);
	if (!p)
		panic("Tried to put sleep informations into unmapped memory!");
	if (*p == 0x1A2B3C4D)
		*p = 0;

	/* Save the first 4 words from 0xa8024000. */
	p = phys_to_virt(0xA8024000);
	if (!p)
		panic("Tried to put sleep informations into unmapped memory!");
	for (i = 0; i < 6; i++)
		save[i] = p[i];

	/* Set the first four words at 0xa8024000 to:
	 * resume address; MMU control; TLB base addr; domain id */
	p[0] = tmp;//(unsigned int) test_resume;//resume_addr;

asm( "mrc\tp15, 0, %0, c1, c0, 0" : "=r" (tmp) );
	p[1] = tmp & (~0xFFFF3987);         /* mmu off */

asm( "mrc\tp15, 0, %0, c2, c0, 0" : "=r" (tmp) );
	p[2] = tmp & 0xFFFFC000;        /* Shouldn't matter, since MMU will be off. */

asm( "mrc\tp15, 0, %0, c3, c0, 0" : "=r" (tmp) );
	p[3] = tmp;     /* Shouldn't matter, since MMU will be off. */
	p[4] = resume_addr;
	p[5] = virt_to_phys((unsigned int)save);

	/* Set PSPR to the checksum the HTC bootloader wants to see. */
	//pr_debug("loox720_pxa_ll_pm_resume: Saving system state...\n");
	for (csum = 0, i = 0; i < 54; i++) {
		tmp = p[i] & 0x1;
		tmp = tmp << 31;
		tmp |= tmp >> 1;
		csum += tmp;
		//      pr_debug("offset: %2x - value: %8x - checksum: %8x", i*4, p[i], csum);
	}

	PSPR = csum;
}
#else
static void
loox720_pxa_ll_pm_suspend(unsigned long resume_addr)
{
//    register unsigned int R10 asm("r10");
	int i;
	u32 csum, tmp;
	u32 * p;

	tmp =  virt_to_phys((unsigned int)save);
	if ((tmp>=0xa8024000)&&(tmp<=0xa824036))
		panic("Resume storage should not be overwritten");

	tmp = virt_to_phys((unsigned int)test_resume);

	if ((tmp>=0xa8024000)&&(tmp<=0xa824036))
		panic("Resume handler should not be overwritten");
	/* Make sure that bootloader will not found reset pattern
	 * by setting special memory places to 0x00 */

	p = (unsigned int *)ioremap_nocache(0xA8025300,0x20);//phys_to_virt(0xA8025304);
	if (!p)
		panic("Tried to put sleep informations into unmapped memory!");
	if (p[1] == 0x1A2B3C4D)
		p[1] = 0;

//      p = phys_to_virt(0xA8025314);
//    if(!p)
//        panic("Tried to put sleep informations into unmapped memory!");
	if (p[5] == 0x1A2B3C4D)
		p[5] = 0;

	/* Save the first 4 words from 0xa8024000. */
	p = (unsigned int *)ioremap_nocache(0xA8024000,0x100);//phys_to_virt(0xA8024000);
	if (!p)
		panic("Tried to put sleep informations into unmapped memory!");
	for (i = 0; i < 6; i++)
		save[i] = p[i];

	/* Set the first four words at 0xa8024000 to:
	 * resume address; MMU control; TLB base addr; domain id */
	p[0] = tmp;//(unsigned int) test_resume;//resume_addr;

asm( "mrc\tp15, 0, %0, c1, c0, 0" : "=r" (tmp) );
	p[1] = tmp & (~0xFFFF3987);         /* mmu off */

asm( "mrc\tp15, 0, %0, c2, c0, 0" : "=r" (tmp) );
	p[2] = tmp & 0xFFFFC000;        /* Shouldn't matter, since MMU will be off. */

asm( "mrc\tp15, 0, %0, c3, c0, 0" : "=r" (tmp) );
	p[3] = tmp;     /* Shouldn't matter, since MMU will be off. */
	p[4] = resume_addr;
	p[5] = virt_to_phys((unsigned int)save);

	/* Set PSPR to the checksum the HTC bootloader wants to see. */
	//pr_debug("loox720_pxa_ll_pm_resume: Saving system state...\n");
	for (csum = 0, i = 0; i < 54; i++) {
		tmp = p[i] & 0x1;
		tmp = tmp << 31;
		tmp |= tmp >> 1;
		csum += tmp;
		//      pr_debug("offset: %2x - value: %8x - checksum: %8x", i*4, p[i], csum);
	}

	PSPR = csum;
}
#endif
static void
loox720_pxa_ll_pm_resume(void)
{
	int i;
	u32 *p;

	/* Restore the data at 0xa8024000. */
	//for (p = phys_to_virt(0xa8024000), i = 0; i < 5; i++)
	//      p[i] = save[i];

	/* XXX Do we need to flush the cache? */
	/*
	__asm__ volatile ("mrc p14, 0, r3, c6, c0, 0\n"
	        "orr r3, r3, #0x0008\n"
	        "mcr p14, 0, r3, c6, c0, 0\n"
	        :::"r3");
	        */

}
#if 0
struct pxa_ll_pm_ops loox720_ll_pm_ops = {
	.suspend = loox720_pxa_ll_pm_suspend,
	.resume  = loox720_pxa_ll_pm_resume,
};
#endif


static int
loox720_core_probe( struct platform_device *pdev )
{
	int battery_full, usb_connected, ac_connected;

	printk( KERN_NOTICE "Loox 720 Core Hardware Driver\n" );

	ac_irq = IRQ_GPIO(GPIO_NR_LOOX720_AC_IN_N);
	battery_irq = IRQ_GPIO(GPIO_NR_LOOX720_BATTERY_FULL_N);
	usb_irq = IRQ_GPIO(GPIO_NR_LOOX720_USB_DETECT_N);

	ac_connected = gpio_get_value(GPIO_NR_LOOX720_AC_IN_N) == 0;
	usb_connected = gpio_get_value(GPIO_NR_LOOX720_USB_DETECT_N) == 0;
	battery_full = gpio_get_value(GPIO_NR_LOOX720_BATTERY_FULL_N) == 0;

	printk( KERN_INFO "AC: connected=%d\n", ac_connected );
	if (request_irq( ac_irq, ac_isr, IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	                 "Loox 720 AC Detect", NULL ) != 0) {
		printk( KERN_ERR "Unable to configure AC detect interrupt.\n" );
		return -ENODEV;
	}

	printk( KERN_INFO "USB: connected=%d\n", usb_connected );
	if (request_irq( usb_irq, usb_isr, IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "Loox 720 USB Detect", NULL) != 0) {
		printk( KERN_ERR "Unable to configure USB detect interrupt.\n" );
		free_irq( ac_irq, NULL );
		return -ENODEV;
	}

	if (request_irq( battery_irq, battery_isr, IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "Loox 720 Battery Full", NULL) != 0) {
		printk( KERN_ERR "Unable to configure battery-full detect interrupt.\n" );
		free_irq( ac_irq, NULL );
		free_irq( usb_irq, NULL );
		return -ENODEV;
	}

	update_battery_chip();
	update_battery_charging();

#if 0
	pxa_pm_set_ll_ops(&loox720_ll_pm_ops);
#endif

	return 0;
}

static int
loox720_core_remove( struct platform_device *dev )
{
	if (ac_irq != 0xffffffff)
		free_irq( ac_irq, NULL );
	if (battery_irq != 0xffffffff)
		free_irq( battery_irq, NULL );
	if (usb_irq != 0xffffffff)
		free_irq( usb_irq, NULL );
	return 0;
}

struct platform_driver loox720_core_driver = {
	.driver = {
		.name     = "loox720-core",
	},
	.probe    = loox720_core_probe,
	            .remove   = loox720_core_remove,
	                        .suspend  = loox720_suspend,
	                                    .resume   = loox720_resume,

                                                };

static int __init
loox720_core_init( void )
{
	return platform_driver_register( &loox720_core_driver );
}


static void __exit
loox720_core_exit( void )
{
	platform_driver_unregister( &loox720_core_driver );
}

module_init( loox720_core_init );
module_exit( loox720_core_exit );

MODULE_AUTHOR("Piotr Czechowicz, Tomasz Figa");
MODULE_DESCRIPTION("Loox 720 Core Hardware Driver");
MODULE_LICENSE("GPL");

