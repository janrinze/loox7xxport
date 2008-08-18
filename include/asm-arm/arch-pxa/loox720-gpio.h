/*
 * include/asm-arm/arch-pxa/loox720-gpio.h
 * 
 * Loox 720 GPIO list
 * Reused the hx4700-gpio.h file as a template.
 *
 * History:
 * 2008/08/17 - Tomasz Figa - Big cleanup - removed most of unneeded definitions, archaic macros and confirmed some GPIOs
 */

/*
 * !!! ATTENTION !!!
 *
 * Following GPIOs definitions are taken from another PDA header (hx4700).
 * We should check them all, to make sure that they aren't misused.
 * My suggestion is to mark all GPIOs that we are sure or unsure with some comment
 * so if you are sure of some GPIO, please add some comment after its definition!
 *
 * THANKS!
 * Tomasz Figa <tom3q@staszic.waw.pl>
 */

#ifndef _LOOX720_GPIO_H_
#define _LOOX720_GPIO_H_

#include <asm/arch/pxa-regs.h>

/*
 * GPIOs operating as Application-Specific GPIO
 */

#define GPIO_NR_LOOX720_KEY_ON			0 // sure - not machine specific
#define GPIO_NR_LOOX720_GP_RST_N		1 // sure - not machine specific

#define GPIO_NR_LOOX720_PWR_SCL			3
#define GPIO_NR_LOOX720_PWR_SDA			4
#define GPIO_NR_LOOX720_PWR_CAP0		5
#define GPIO_NR_LOOX720_PWR_CAP1		6
#define GPIO_NR_LOOX720_PWR_CAP2		7
#define GPIO_NR_LOOX720_PWR_CAP3		8
#define GPIO_NR_LOOX720_AC_IN_N			9 // sure - low when AC Adapter is connected

#define GPIO_NR_LOOX720_BATTERY_FULL_N		11 // sure - low when battery is full
#define GPIO_NR_LOOX720_CPLD_INT		12 // sure - CPLD external interrupt pin used by IRQ demuxer
#define GPIO_NR_LOOX720_USB_DETECT_N		13 // not 100% sure - detects connection to USB client controller
#define GPIO_NR_LOOX720_WIFI_PWR		14 // sure - CF WIFI power pin - high to enable ACX100 (must be used in pair with CPLD bit)

#define GPIO_NR_LOOX720_WIFI_RST	 	19 // sure - CF WIFI reset pin - high to enable ACX100 reset state, low to disable

#define GPIO_NR_LOOX720_CPU_BT_RESET_N		22 // sure - Bluetooth chip reset pin - low to enable BRF6150 reset state, low to disable

#define GPIO_NR_LOOX720_MMC_DETECT_N		52 // sure - low when an SD card is inserted

#define GPIO_NR_LOOX720_MMC_RO			74 // sure - high when a write-protected SD card is inserted
#define GPIO_NR_LOOX720_EARPHONE_DET_N		75 // INVALID -> please detect correct function!

#define GPIO_NR_LOOX720_IR_ON_N			81

#define GPIO_NR_LOOX720_TOUCHPANEL_SPI_DI	86
#define GPIO_NR_LOOX720_TOUCHPANEL_SPI_DO	87

#define GPIO_NR_LOOX720_FLASH_VPEN		91

#define GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N	94
#define GPIO_NR_LOOX720_BATT_OFF		95
#define GPIO_NR_LOOX720_USB_CHARGE_RATE		96

#define GPIO_NR_LOOX720_CHARGE_EN_N		107 // sure - set low to disable battery charging

#endif /* _LOOX720_GPIO_H */
