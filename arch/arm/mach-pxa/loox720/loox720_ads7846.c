#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/input.h>

#include <asm/hardware.h>
#include <asm/arch/loox720.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/loox720-cpld.h>
#include <asm/arch/loox720_ads7846.h>

	// =======================================================
	// hardcoded values for testing
	// we should be using the pdev struct to retrieve this info..
	// =======================================================
	
static loox720_ads7846_device_info loox720_ads7846_dev = {
	.clock	= GPIO_NR_LOOX720_TOUCHPANEL_SPI_CLK,
	.cs	= GPIO_NR_LOOX720_TOUCHPANEL_SPI_CS,
	.mosi	= GPIO_NR_LOOX720_TOUCHPANEL_SPI_DO,
	.miso	= GPIO_NR_LOOX720_TOUCHPANEL_SPI_DI,
	.pen	= GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N,
	.irq	= IRQ_GPIO(GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N),

	.half_clock_time=4, // 8 usec setup for 125kHz max operation
	.sample_rate=10, // 20 msec for 50Hz sampling
};
	// =======================================================
	// with 8 usec per bit we have 15 * 5 * 8 = 600 usec per conversion.
	// this will then allow for max 1333 conversions per second.
	// But! during a conversion the kernel is 'quiet'..
	// so we will miss 750 usec / 5000 usec =  15 % CPU during the conversion..!!
	// =======================================================

/* sequence grabbed with Haret for one touch screen message
001.916(0328839) 095717ec: e5851000(str)         # a9000010 =00006980           
001.916(0000044) 09571824: e5851000(str)         # a9000010 =00006800           
001.916(0000015) 0957185c: e5851000(str)         # a9000010 =00004980           
001.916(0000014) 09571894: e5851000(str)         # a9000010 =00004800           
001.916(0000013) 09571898: e5859000(str)         # a9000010 =00000000           
001.916(0008857) 095718e0: e5910000(ldr)         # a9000010==0000401e           
001.916(0000077) 095718e0: e5910000(ldr)         # a9000010==0000282c           
001.916(0000034) 095718e0: e5910000(ldr)         # a9000010==00000c22           
001.916(0000035) 095718e0: e5910000(ldr)         # a9000010==00007c26           
001.916(0000039) 095718e0: e5910000(ldr)         # a9000010==00004600  
*/

static loox720_ads7846_spi_message ads_cmds[5] = {
	{ .cmd = 0x6980, },
	{ .cmd = 0x6800, },
	{ .cmd = 0x4980, },
	{ .cmd = 0x4800, },
	{ .cmd = 0x0000, }
};

/* ==========================================================

	bitbanging routines

   ========================================================== */

#define bittogglefunc( gpio ) 							\
static inline void set ## gpio(loox720_ads7846_device_info *dev)		\
{										\
	if(dev)									\
		gpio_set_value(dev-> gpio ,1);					\
}										\
static inline void clr ## gpio(loox720_ads7846_device_info *dev)		\
{										\
	if(dev)									\
		gpio_set_value(dev-> gpio ,0);					\
}

#define bitreadfunc( gpio )							\
static inline unsigned int read ## gpio(loox720_ads7846_device_info *dev)	\
{										\
	if(dev)									\
		return (gpio_get_value(dev-> gpio )?1:0);			\
	printk( KERN_INFO "ADS7846 trying to read data with empty dev\n");	\
	return 0;								\
}

bittogglefunc( mosi )
bittogglefunc( clock )
bittogglefunc( cs )


bitreadfunc( miso )
bitreadfunc( pen )
/*
static inline unsigned int readmiso(loox720_ads7846_device_info *dev)
{
	if(dev)
		return (gpio_get_value(dev->miso)?1:0);
	printk( KERN_INFO "ADS7846 trying to read data with empty dev\n");
	return 0;
}
static inline unsigned int readpen(loox720_ads7846_device_info *dev)
{
	if(dev)
		return (gpio_get_value(dev->pen)?1:0);
	return 0;
}

static inline void setmosi(loox720_ads7846_device_info *dev)
{
	if(dev)
		gpio_set_value(dev->mosi,1);
}
static inline void clrmosi(loox720_ads7846_device_info *dev)
{
	if(dev)
		gpio_set_value(dev->mosi,0);
}
static inline void setclock(loox720_ads7846_device_info *dev)
{
	if(dev)
		gpio_set_value(dev->clock,1);
}
static inline void clrclock(loox720_ads7846_device_info *dev)
{
	if(dev)
		gpio_set_value(dev->clock,0);
}
static inline void setcs(loox720_ads7846_device_info *dev)
{
	if(dev)
		gpio_set_value(dev->cs,1);
}
static inline void clrcs(loox720_ads7846_device_info *dev)
{
	if(dev)
		gpio_set_value(dev->cs,0);
}
*/

static inline void spidelay(loox720_ads7846_device_info *dev)
{
	if(dev)
		udelay(dev->half_clock_time);
}

/* ==========================================================

	SPI byte transfer

   ========================================================== */

unsigned int SPIBitBang15BitsMode1(loox720_ads7846_device_info * dev,unsigned int input)
{       
    unsigned int bit,output;

    output = 0;
 
    if (dev==NULL) printk( KERN_INFO "ADS7846 trying to read data with empty dev\n");

    for (bit = 0; bit < 15; bit++) {
        /* write MOSI on trailing edge of previous clock */
        if (input & (0x4000))
            setmosi(dev);
        else
            clrmosi(dev);
        input <<= 1;
 
        /* half a clock cycle before leading/rising edge */
        spidelay(dev);
        setclock(dev);

 	 /* read MISO on rising edge */
        output = (output<<1) | readmiso(dev);

        /* half a clock cycle before trailing/falling edge */
        spidelay(dev);
 	/* read MISO on rising edge */     
        clrclock(dev);
    }
 
    return output;
}
/* ==========================================================

	ads7846 command routines

   ========================================================== */


void loox720_ads7846_send_commands(loox720_ads7846_device_info * dev,loox720_ads7846_spi_message * msg,int num_commands)
{
	clrmosi(dev);
	clrcs(dev);
	spidelay(dev);
	while (num_commands--) {
		msg->data = SPIBitBang15BitsMode1(dev,msg->cmd);
		msg++;
	}
	clrmosi(dev);
	setcs(dev);
}

void loox720_ads7846_report(loox720_ads7846_device_info * dev, unsigned int Rt, unsigned int x, unsigned int y)
{
		input_report_abs(dev->input, ABS_X, x);
		input_report_abs(dev->input, ABS_Y, y);
		input_report_abs(dev->input, ABS_PRESSURE, Rt);
		input_sync(dev->input);
}

#define convert_data_12( pos ) ((ads_cmds[ pos ].data &0x3f)<<6)|((ads_cmds[ pos + 1].data &0xfc00)>>10)

static volatile int loox720_ads7846_busy;

static void read_loox720_ads7846(void * ads_dev)
{
	unsigned int i,x,y,z1,z2;
	loox720_ads7846_spi_message * cmds = ads_cmds;
	loox720_ads7846_device_info * dev = ads_dev;

	//printk( KERN_INFO "ADS7846 v2 starting to read data\n");
	
	loox720_ads7846_send_commands( dev , cmds , ARRAY_SIZE(ads_cmds));

	x  = convert_data_12( 0 );
	y  = convert_data_12( 3 );
	z1 = convert_data_12( 1 );
	z2 = convert_data_12( 2 );
	
	if (x&&z1) // valid measurement
	{
		unsigned int Rt; // calculated pressure for 400 ohms
		Rt = (((z2-z1)*x*400)/z1 +2047)>>12;
		loox720_ads7846_report(dev, Rt, x,y);
	}

}

static void looping_read_loox720_ads7846(/*void * ads_dev*/)
{
	loox720_ads7846_device_info * dev = &loox720_ads7846_dev;
	if (loox720_ads7846_busy==0)
	{
		loox720_ads7846_busy=1;
		input_report_key(dev->input, BTN_TOUCH, 1);

		while (dev->pendown)
		{
			int num_slices=100;
			read_loox720_ads7846(dev);
			if (dev->pendown) msleep(dev->sample_rate);	
		}
		input_report_key(dev->input, BTN_TOUCH, 0);
	}
	loox720_ads7846_busy=0;
}

/* ==========================================================

	interrupt routines

   ========================================================== */

static DECLARE_WORK(printcoords_work, looping_read_loox720_ads7846);

irqreturn_t pendown_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	if (readpen(&loox720_ads7846_dev)==0)
	{
		
		loox720_ads7846_dev.pendown=1;
		if (loox720_ads7846_busy==0)
		{
			//loox720_ads7846_busy=1;
			//tasklet_schedule(&printcoords_tasklet);
			schedule_work(&printcoords_work);
		}
	} else {
		loox720_ads7846_dev.pendown=0;
	}
   
    return IRQ_HANDLED;
}

/* ==========================================================

	module init and exit routines

   ========================================================== */

static int __init loox720_ads7846_probe(struct platform_device *pdev)
{
	struct input_dev *input_dev;
	int err;

	// =======================================================
	//	setup our input device layer
	// =======================================================

	input_dev = input_allocate_device();
	
	if ( !(input_dev = input_allocate_device()) )
	{
		err = -ENOMEM;
		goto err_no_mem;
	}
	
	input_dev->name = "Loox720 ADS7846 Touchscreen";
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X,0,MAX_12BIT,0, 0);
	input_set_abs_params(input_dev, ABS_Y,0,MAX_12BIT,0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0,MAX_12BIT, 0, 0);

	// =======================================================
	/* 	setup the GPIOs as we want to use them */
	// =======================================================

	if(gpio_request(loox720_ads7846_dev.mosi, "loox720 ads7846 MOSI") != 0) {
		printk(KERN_ERR "Failed to request loox720 ads7846 MOSI GPIO\n");
		err = -ENODEV;
		goto err_remove_input;
	}
	
	if(gpio_request(loox720_ads7846_dev.miso, "loox720 ads7846 MISO") != 0) {
		printk(KERN_ERR "Failed to request loox720 ads7846 MISO GPIO\n");
		// and release the already allocated gpios
		
		err = -ENODEV;
		goto err_remove_mosi;
	}

	if(gpio_request(loox720_ads7846_dev.clock, "loox720 ads7846 CLK") != 0) {
		printk(KERN_ERR "Failed to request loox720 ads7846 CLK GPIO\n");
		// and release the already allocated gpios

		err = -ENODEV;
		goto err_remove_miso;
	}
	if(gpio_request(loox720_ads7846_dev.cs, "loox720 ads7846 CS") != 0) {
		printk(KERN_ERR "Failed to request loox720 ads7846 CS GPIO\n");
		// and release the already allocated gpios
		err = -ENODEV;
		goto err_remove_clock;
	}

	// setup the gpios correctly

	gpio_direction_output(loox720_ads7846_dev.mosi,0);
	gpio_direction_input(loox720_ads7846_dev.miso);
	gpio_direction_output(loox720_ads7846_dev.clock,0);
	gpio_direction_output(loox720_ads7846_dev.cs,1);


	// =======================================================
	//	register our input device
	// =======================================================

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_clock;

	loox720_ads7846_dev.input = input_dev;
	// =======================================================
	//	do a quick read
	// =======================================================

	read_loox720_ads7846(&loox720_ads7846_dev);
	return 0;

err_remove_clock:
	gpio_free(loox720_ads7846_dev.clock);
err_remove_miso:
	gpio_free(loox720_ads7846_dev.miso);
err_remove_mosi:
	gpio_free(loox720_ads7846_dev.mosi);
err_remove_input:
	input_free_device(input_dev);
err_no_mem:	
	return err;
}

void loox720_ads7846_remove()
{
	gpio_free(loox720_ads7846_dev.mosi);
	gpio_free(loox720_ads7846_dev.miso);
	gpio_free(loox720_ads7846_dev.clock);
	gpio_free(loox720_ads7846_dev.cs);
	if (loox720_ads7846_dev.input) input_unregister_device(loox720_ads7846_dev.input);
}


static struct platform_driver loox720_ads7846_driver = {
	.driver		= {
		.name	= "loox720_ads7846",
	},
	//.probe		= loox720_ads7846_probe,
	//.remove		= loox720_ads7846_remove,
};

static int __init loox720_ads7846_init(void)
{
/*
	can we use a timer to start our bitbanging interface?

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = ads7846_timer;
*/	

	loox720_ads7846_device_info * dev = &loox720_ads7846_dev;
	loox720_ads7846_probe(NULL);

	if (request_irq(dev->irq, pendown_interrupt, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
			loox720_ads7846_driver.driver.name, dev)) {
		printk(KERN_ERR "Failed to request loox720 ads7846 IRQ\n");
		};

	return platform_driver_register(&loox720_ads7846_driver);
}

static void __exit loox720_ads7846_exit(void)
{
	loox720_ads7846_device_info * spi = &loox720_ads7846_dev;
	free_irq(spi->irq,spi);
	loox720_ads7846_remove();
	platform_driver_unregister(&loox720_ads7846_driver);
}


module_init(loox720_ads7846_init);
module_exit(loox720_ads7846_exit);

MODULE_AUTHOR ("Jan Rinze Peterzon");
MODULE_DESCRIPTION ("bitbanging Touchscreen support for LOOX718/720");
MODULE_LICENSE ("GPL");

