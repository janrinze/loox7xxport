/*
 * include/asm-arm/arch-pxa/hx4700-gpio.h
 * History:
 *
 * 2004-12-10 Michael Opdenacker. Wrote down GPIO settings as identified by Jamey Hicks.
 *            Reused the h2200-gpio.h file as a template.
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
 */

#ifndef _LOOX720_GPIO_H_
#define _LOOX720_GPIO_H_

#include <asm/arch/pxa-regs.h>

#define GET_LOOX720_GPIO(gpio) \
	(GPLR(GPIO_NR_LOOX720_ ## gpio) & GPIO_bit(GPIO_NR_LOOX720_ ## gpio))

#define SET_LOOX720_GPIO(gpio, setp) \
do { \
if (setp) \
	GPSR(GPIO_NR_LOOX720_ ## gpio) = GPIO_bit(GPIO_NR_LOOX720_ ## gpio); \
else \
	GPCR(GPIO_NR_LOOX720_ ## gpio) = GPIO_bit(GPIO_NR_LOOX720_ ## gpio); \
} while (0)

#define SET_LOOX720_GPIO_N(gpio, setp) \
do { \
if (setp) \
	GPCR(GPIO_NR_LOOX720_ ## gpio ## _N) = GPIO_bit(GPIO_NR_LOOX720_ ## gpio ## _N); \
else \
	GPSR(GPIO_NR_LOOX720_ ## gpio ## _N) = GPIO_bit(GPIO_NR_LOOX720_ ## gpio ## _N); \
} while (0)

#define LOOX720_IRQ(gpio) \
	IRQ_GPIO(GPIO_NR_LOOX720_ ## gpio)

#define GPIO_NR_LOOX720_KEY_ON			0 // sure - not machine specific
#define GPIO_NR_LOOX720_GP_RST_N		1 // sure - not machine specific

#define GPIO_NR_LOOX720_PWR_SCL			3
#define GPIO_NR_LOOX720_PWR_SDA			4
#define GPIO_NR_LOOX720_PWR_CAP0		5
#define GPIO_NR_LOOX720_PWR_CAP1		6
#define GPIO_NR_LOOX720_PWR_CAP2		7
#define GPIO_NR_LOOX720_PWR_CAP3		8
#define GPIO_NR_LOOX720_AC_IN_N			9 // sure - low when AC Adapter is connected
#define GPIO_NR_LOOX720_CLK_TOUT_32KHz		10
#define GPIO_NR_LOOX720_BATTERY_FULL_N		11 // sure - low when battery is full
#define GPIO_NR_LOOX720_CPLD_INT		12 // sure - CPLD external interrupt pin used by IRQ demuxer
#define GPIO_NR_LOOX720_USB_DETECT_N	13 // not 100% sure - detects connection to USB client controller
#define GPIO_NR_LOOX720_WIFI_PWR		14 // sure - CF WIFI power pin - high to enable ACX100 (must be used in pair with CPLD bit)
#define GPIO_NR_LOOX720_CS1_N			15

#define GPIO_NR_LOOX720_BACKLIGHT		17 // unsure -> old comment: Tied to PWM0 when Alt function == 2

#define GPIO_NR_LOOX720_RDY				18
#define GPIO_NR_LOOX720_WIFI_RST	 	19 // sure - CF WIFI reset pin - high to enable ACX100 reset state, low to disable
#define GPIO_NR_LOOX720_SDCS2_N			20
#define GPIO_NR_LOOX720_SDCS3_N			21
#define GPIO_NR_LOOX720_CPU_BT_RESET_N	22 // sure - Bluetooth chip reset pin - low to enable BRF6150 reset state, low to disable
#define GPIO_NR_LOOX720_SPI_CLK			23
#define GPIO_NR_LOOX720_SPI_CS_N		24
#define GPIO_NR_LOOX720_SPI_DO			25
#define GPIO_NR_LOOX720_SPI_DI			26
#define GPIO_NR_LOOX720_CODEC_ON		27
#define GPIO_NR_LOOX720_I2S_BCK			28
#define GPIO_NR_LOOX720_I2S_DIN			29
#define GPIO_NR_LOOX720_I2S_DOUT		30
#define GPIO_NR_LOOX720_I2S_SYNC		31
#define GPIO_NR_LOOX720_RS232_ON		32
#define GPIO_NR_LOOX720_CS5_N			33
#define GPIO_NR_LOOX720_COM_RXD			34
#define GPIO_NR_LOOX720_COM_CTS			35
#define GPIO_NR_LOOX720_COM_DCD			36
#define GPIO_NR_LOOX720_COM_DSR			37
#define GPIO_NR_LOOX720_COM_RING		38
#define GPIO_NR_LOOX720_COM_TXD			39
#define GPIO_NR_LOOX720_COM_DTR			40
#define GPIO_NR_LOOX720_COM_RTS			41
#define GPIO_NR_LOOX720_BT_RXD			42
#define GPIO_NR_LOOX720_BT_TXD			43
#define GPIO_NR_LOOX720_BT_UART_CTS		44
#define GPIO_NR_LOOX720_BT_UART_RTS		45

#define GPIO_NR_LOOX720_STD_RXD			46
#define GPIO_NR_LOOX720_STD_TXD			47

#define GPIO_NR_LOOX720_POE_N			48
#define GPIO_NR_LOOX720_PWE_N			49
#define GPIO_NR_LOOX720_PIOR_N			50
#define GPIO_NR_LOOX720_PIOW_N			51
#define GPIO_NR_LOOX720_CPU_BATT_FAULT_N	52

#define GPIO_NR_LOOX720_PCE2_N			54
#define GPIO_NR_LOOX720_PREG_N			55
#define GPIO_NR_LOOX720_PWAIT_N			56

#define GPIO_NR_LOOX720_CF_DETECT_N		57 // INVALID -> please detect correct function!

//#define GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N	58
#define GPIO_NR_LOOX720_LCD_PC1			59
#define GPIO_NR_LOOX720_CF_RNB			60
#define GPIO_NR_LOOX720_W3220_RESET_N		61
#define GPIO_NR_LOOX720_LCD_RESET_N		62
#define GPIO_NR_LOOX720_CPU_SS_RESET_N		63

#define GPIO_NR_LOOX720_TOUCHPANEL_PEN_PU	65
#define GPIO_NR_LOOX720_ASIC3_SDIO_INT_N	66
#define GPIO_NR_LOOX720_EUART_PS		67

#define GPIO_NR_LOOX720_LCD_SLIN1		70
#define GPIO_NR_LOOX720_ASIC3_RESET_N		71 // INVALID (we don't have ASIC) -> please detect correct function!
//#define GPIO_NR_LOOX720_CHARGE_EN_N		72 // INVALID -> please detect correct function!
#define GPIO_NR_LOOX720_LCD_UD_1		73

#define GPIO_NR_LOOX720_EARPHONE_DET_N		75 // INVALID -> please detect correct function!
#define GPIO_NR_LOOX720_USB_PUEN		76 // INVALID -> please detect correct function!

#define GPIO_NR_LOOX720_CS2_N			78
#define GPIO_NR_LOOX720_CS3_N			79
#define GPIO_NR_LOOX720_CS4_N			80
//#define GPIO_NR_LOOX720_CPU_GP_RESET_N		81
#define GPIO_NR_LOOX720_IR_ON_N			81
#define GPIO_NR_LOOX720_EUART_RESET		82
#define GPIO_NR_LOOX720_WLAN_RESET_N		83
#define GPIO_NR_LOOX720_LCD_SQN			84
#define GPIO_NR_LOOX720_PCE1_N			85
#define GPIO_NR_LOOX720_TOUCHPANEL_SPI_DI	86
#define GPIO_NR_LOOX720_TOUCHPANEL_SPI_DO	87
#define GPIO_NR_LOOX720_TOUCHPANEL_SPI_CS_N	88

#define GPIO_NR_LOOX720_FLASH_VPEN		91
#define GPIO_NR_LOOX720_HP_DRIVER		92
#define GPIO_NR_LOOX720_EUART_INT		93
//#define GPIO_NR_LOOX720_KEY_AP3		94
#define GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N	94
#define GPIO_NR_LOOX720_BATT_OFF		95
#define GPIO_NR_LOOX720_USB_CHARGE_RATE		96

//#define GPIO_NR_LOOX720_BL_DETECT_N		97
//#define GPIO_NR_LOOX720_KEY_AP1		99
//#define GPIO_NR_LOOX720_AUTO_SENSE		100	/* to backlight circuit */
//#define GPIO_NR_LOOX720_SYNAPTICS_POWER_ON	102
//#define GPIO_NR_LOOX720_SYNAPTICS_INT		103
//#define GPIO_NR_LOOX720_PSKTSEL		104
//#define GPIO_NR_LOOX720_IR_ON_N		105

#define GPIO_NR_LOOX720_CHARGE_EN_N		107 // sure - set low to disable battery charging

#define GPIO_NR_LOOX720_CODEC_ON_N		109
#define GPIO_NR_LOOX720_LCD_LVDD_3V3_ON		110
#define GPIO_NR_LOOX720_LCD_AVDD_3V3_ON		111
#define GPIO_NR_LOOX720_LCD_N2V7_7V3_ON		112
#define GPIO_NR_LOOX720_I2S_SYSCLK		113 // sure - I2S System Clock used by sound codec
#define GPIO_NR_LOOX720_CF_RESET		114	/* HaRET: O 0 0 */
#define GPIO_NR_LOOX720_USB2_DREQ		115
#define GPIO_NR_LOOX720_CPU_HW_RESET_N		116
#define GPIO_NR_LOOX720_I2C_SCL			117
#define GPIO_NR_LOOX720_I2C_SDA			118

#define EGPIO0_VCC_3V3_EN		(1<<0)	/* what use? */
#define EGPIO1_WL_VREG_EN		(1<<1)	/* WLAN Power */
#define EGPIO2_VCC_2V1_WL_EN		(1<<2)	/* unused */
#define EGPIO3_SS_PWR_ON		(1<<3)	/* smart slot power on */
#define EGPIO4_CF_3V3_ON		(1<<4)	/* CF 3.3V enable */
#define EGPIO5_BT_3V3_ON		(1<<5)	/* Bluetooth 3.3V enable */
#define EGPIO6_WL1V8_EN			(1<<6)	/* WLAN 1.8V enable */
#define EGPIO7_VCC_3V3_WL_EN		(1<<7)	/* WLAN 3.3V enable */
#define EGPIO8_USB_3V3_ON		(1<<8)	/* unused */

#define GPIO_NR_LOOX720_AC_IN_N_MD		( 9 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_SDCS2_N_MD		(20 | GPIO_ALT_FN_1_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_SDCS3_N_MD		(21 | GPIO_ALT_FN_1_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_SYNAPTICS_SPI_CLK_MD	(23 | GPIO_ALT_FN_2_IN)
#define GPIO_NR_LOOX720_SYNAPTICS_SPI_CS_N_MD	(24 | GPIO_ALT_FN_2_IN)
#define GPIO_NR_LOOX720_SYNAPTICS_SPI_DO_MD	(25 | GPIO_ALT_FN_2_OUT)
#define GPIO_NR_LOOX720_SYNAPTICS_SPI_DI_MD	(26 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_I2S_BCK_MD		(28 | GPIO_ALT_FN_1_OUT)
#define GPIO_NR_LOOX720_I2S_DIN_MD		(29 | GPIO_ALT_FN_2_IN)
#define GPIO_NR_LOOX720_I2S_DOUT_MD		(30 | GPIO_ALT_FN_1_OUT)
#define GPIO_NR_LOOX720_I2S_SYNC_MD		(31 | GPIO_ALT_FN_1_OUT)
#define GPIO_NR_LOOX720_COM_RXD_MD		(34 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_COM_CTS_MD		(35 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_COM_DCD_MD		(36 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_COM_DSR_MD		(37 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_COM_RING_MD		(38 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_COM_TXD_MD		(39 | GPIO_ALT_FN_2_OUT)
#define GPIO_NR_LOOX720_COM_DTR_MD		(40 | GPIO_ALT_FN_2_OUT)
#define GPIO_NR_LOOX720_COM_RTS_MD		(41 | GPIO_ALT_FN_2_OUT)
#define GPIO_NR_LOOX720_BT_RXD_MD                (42 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_BT_TXD_MD                (43 | GPIO_ALT_FN_2_OUT)
#define GPIO_NR_LOOX720_BT_UART_CTS_MD           (44 | GPIO_ALT_FN_1_IN)
#define GPIO_NR_LOOX720_BT_UART_RTS_MD           (45 | GPIO_ALT_FN_2_OUT)

#define GPIO_NR_LOOX720_STD_RXD_MD		(46 | GPIO_ALT_FN_2_IN)
#define GPIO_NR_LOOX720_STD_TXD_MD		(47 | GPIO_ALT_FN_1_OUT)

#define GPIO_NR_LOOX720_POE_N_MD		(48 | GPIO_ALT_FN_2_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PWE_N_MD		(49 | GPIO_ALT_FN_2_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PIOR_N_MD		(50 | GPIO_ALT_FN_2_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PIOW_N_MD		(51 | GPIO_ALT_FN_2_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PCE2_N_MD		(54 | GPIO_ALT_FN_2_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PREG_N_MD		(55 | GPIO_ALT_FN_2_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PWAIT_N_MD		(56 | GPIO_ALT_FN_1_IN | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PIOIS16_N_MD		(57 | GPIO_ALT_FN_1_IN | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PCE1_N_MD		(85 | GPIO_ALT_FN_1_OUT | GPIO_DFLT_HIGH)
#define GPIO_NR_LOOX720_PSKTSEL_MD		(104 | GPIO_ALT_FN_1_OUT)
#define GPIO_NR_LOOX720_I2S_SYSCLK_MD		(113 | GPIO_ALT_FN_1_OUT) // sure, look at GPIO 113 definition

#endif /* _LOOX720_GPIO_H */
