/*
 * include/asm/arm/arch-pxa/loox720-cpld.h
 *
 * Authors: Piotr Czechowicz <p.czechowicz@admitech.pl>
 *
 */

#ifndef _LOOX720_CPLD_H_
#define _LOOX720_CPLD_H_

#include <asm/arch/pxa-regs.h>

#define LOOX720_CPLD_PHYS	PXA_CS4_PHYS
#define LOOX720_CPLD_SIZE	32

/*
    adapted these methods for 16 bit..
    Hopefully noone else uses these..
    a carefull grep showed noone yet does..
    the names are too generic..
*/

#define BIT_POS(x)	(x>>4)
#define BIT_SHIFT(x)	(x&15)
#define BIT_MSK(x)	(u32)(1<<BIT_SHIFT(x))

/*
Bitmasks of LED modes in LEDs' cache.
*/

#define LOOX720_LED1_BLUE		1
#define LOOX720_LED1_GREEN		2
#define LOOX720_LED1_BLINK		4
#define LOOX720_LED1_BLINK2		8

#define LOOX720_LED2_GREEN		16
#define LOOX720_LED2_RED		32
#define LOOX720_LED2_ORANGE		48
#define LOOX720_LED2_BLINK	 	64
#define LOOX720_LED2_BLINK2	  	128

/*
Bitmasks of bits in reg #4:
*/

#define LOOX720_CPLD_LED1_BIT_A 256
#define LOOX720_CPLD_LED1_BIT_B 512
#define LOOX720_CPLD_LED1_BIT_C 1024
#define LOOX720_CPLD_LED1_BIT_D 2048
#define LOOX720_CPLD_LED1_BLINK 32768

#define LOOX720_CPLD_LED2_BIT_A 16
#define LOOX720_CPLD_LED2_BIT_B 32
#define LOOX720_CPLD_LED2_BIT_C 64
#define LOOX720_CPLD_LED2_BIT_D 128
#define LOOX720_CPLD_LED2_BLINK 16384

/*
CPLD output bits numbers
*/

#define LOOX720_CPLD_CF_3V3_BIT			140
#define LOOX720_CPLD_BATTERY_BIT		144
#define LOOX720_CPLD_USB_PULLUP_BIT		145
#define LOOX720_CPLD_LCD_CONSOLE_BIT		146	/* If it's low, then FB console doesn't show up */
#define LOOX720_CPLD_LCD_BIT			147
#define LOOX720_CPLD_LED2_EN_1			148
#define LOOX720_CPLD_LED2_EN_2			149
#define LOOX720_CPLD_LED2_EN_3			150
#define LOOX720_CPLD_LED1_EN_1			152
#define LOOX720_CPLD_LED1_EN_2			153
#define LOOX720_CPLD_LCD_COLOR_BIT		160
#define LOOX720_CPLD_LCD_BIT2			161
#define LOOX720_CPLD_SERIAL_BIT			163
#define LOOX720_CPLD_BLUETOOTH_POWER		192
#define LOOX720_CPLD_CF_RESET_N			193
#define LOOX720_CPLD_CAMERA_FLASH_BIT		195
#define LOOX720_CPLD_SOUND_BIT			208
#define LOOX720_CPLD_SND_AMPLIFIER_BIT		209
#define LOOX720_CPLD_BLUETOOTH_RADIO    	210
#define LOOX720_CPLD_WIFI_POWER			211
#define LOOX720_CPLD_SD_BIT			225
#define LOOX720_CPLD_CF_5V_BIT			241
#define LOOX720_CPLD_BACKLIGHT_BIT		242

/*
CPLD input bitmasks definitions

These should be bit positions!!

*/

#define LOOX720_CPLD_CF_DETECT_N		0x04
#define LOOX720_CPLD_CF_READY			0x10
#define LOOX720_CPLD_WIFI_READY			0x20
#define LOOX720_CPLD_WIFI_ENABLED		0x40

extern int loox720_cpld_irq_base;

/*
CPLD interrupt multiplexer definitions
*/

#define LOOX720_IRQ_BASE IRQ_BOARD_START

#define LOOX720_CPLD_IRQ(irq) \
    (LOOX720_CPLD_IRQ_ ## irq) + LOOX720_IRQ_BASE

#define LOOX720_CPLD_IRQ_COUNT 16

#define LOOX720_CPLD_IRQ_WIFI 		5
#define LOOX720_CPLD_IRQ_CF 		4
#define LOOX720_CPLD_IRQ_CARD_DETECT 	2

/*
Function definitions
*/

extern void loox720_set_leds(int mode);
extern void loox720_egpio_set_bit(int bit, int val);
extern u32  loox720_cpld_reg_read(int regno);
extern void loox720_cpld_reg_write(int regno, u32 value);
extern void loox720_cpld_write_masked(int regno,u32 mask ,u32 value);
extern u32 loox720_cpld_reg_test(int regno,u32 mask);
extern void loox720_cpld_resume(void);

#endif
