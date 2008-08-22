/*
 * LCD driver for Fujitsu-Siemens Loox 720
 *
 * Authors: ...
 *
 * based on previous work, see below:
 *
 * Machine initialization for Dell Axim X3
 *
 * Authors: Andrew Zabolotny <zap@homelink.ru>
 *
 * For now this is mostly a placeholder file; since I don't own an Axim X3
 * it is supposed the project to be overtaken by somebody else. The code
 * in here is *supposed* to work so that you can at least boot to the command
 * line, but there is no guarantee.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/err.h>

//#include <asm/arch/aximx3-init.h>
//#include <asm/arch/aximx3-gpio.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/loox720-cpld.h>
#include <linux/fb.h>
#include <asm/mach-types.h>
#include "asm/arch/pxa-regs.h"
#include "asm/arch/pxafb.h"
#include <linux/platform_device.h>
//#include <linux/corgi_bl.h>


static struct pxafb_mode_info loox720_lcd_mode_info = {
	.pixclock		= 80000, // Since we now use double pixel clock
	.bpp			= 16,
	.xres			= 480,
	.yres			= 640,
	.hsync_len		= 4,
	.left_margin		= 20,
	.right_margin		= 8,
	.vsync_len		= 1,
	.upper_margin		= 6,
	.lower_margin		= 9,
	.sync			= 0,
};

static void loox720_backlight_power(int on)
{
	loox720_egpio_set_bit(LOOX720_CPLD_BACKLIGHT_BIT, on);
}

static void loox720_lcd_power(int on, struct fb_var_screeninfo *var)
{
	if(on) {
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_BIT2, 1);
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_CONSOLE_BIT, 1);
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_BIT, 1);
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_COLOR_BIT, 1);
	} else {
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_COLOR_BIT, 0);
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_CONSOLE_BIT, 0);
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_BIT2, 0);
		loox720_egpio_set_bit(LOOX720_CPLD_LCD_BIT, 0);
	}
}

static struct pxafb_mach_info loox720_fb_info =
{
	.modes			= &loox720_lcd_mode_info,
	.num_modes		= 1,
	.lccr0			= LCCR0_Act | LCCR0_Sngl | LCCR0_Color ,
	.lccr3			= LCCR3_OutEnH | LCCR3_PixRsEdg, // | LCCR3_DPC,
	.pxafb_backlight_power	= loox720_backlight_power,
	.pxafb_lcd_power	= loox720_lcd_power,
};

static int __init
loox720_lcd_init (void)
{
	if (! machine_is_loox720 ())
		return -ENODEV;

	set_pxa_fb_info(&loox720_fb_info);

	return 0;
}

static void __exit
loox720_lcd_exit (void)
{

}

module_init (loox720_lcd_init);
module_exit (loox720_lcd_exit);

MODULE_AUTHOR("...");
MODULE_DESCRIPTION("Loox 720 Core frambuffer driver");
MODULE_LICENSE("GPL");

