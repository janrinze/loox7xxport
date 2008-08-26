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

#include <linux/spi/spi.h>
#include <linux/input.h>

#include <asm/hardware.h>
#include <asm/arch/loox720.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/loox720-cpld.h>
#include <asm/arch/loox720_spi_ads7846.h>

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

	.sample_rate=5, // 5 msec for 200Hz sampling
};

/* ==========================================================

	ads7846 commands 

   ========================================================== */

#if 0
static loox720_ads7846_spi_message ads_cmds[5] = {
	{ .cmd = 0x6980, }, // get x D3
	{ .cmd = 0x6800, }, // get y D0
	{ .cmd = 0x4980, }, // get z1 93
	{ .cmd = 0x4800, }, // get z2 90
	{ .cmd = 0x0000, }  // powerdown
};
#else
/*
static loox720_ads7846_spi_message ads_cmds[5] = {
	{ .cmd = 0xD3<<7, }, // get x 
	{ .cmd = 0x93<<7, }, // get y 
	{ .cmd = 0xB3<<7, }, // get z1 
	{ .cmd = 0xC3<<7, }, // get z2
	{ .cmd = 0x0000, }  // powerdown
};*/
static loox720_ads7846_spi_message ads_cmds[5] = {
	{ .cmd = 0xD3<<7, }, // get x 
	{ .cmd = 0xD0<<7, }, // get z1 
	{ .cmd = 0x93<<7, }, // get y 
	{ .cmd = 0x90<<7, }, // get z2
	{ .cmd = 0x0000, }  // powerdown
};

#endif


/* ==========================================================

	SPI block read/write interface

   ========================================================== */

int SPI_read_write_block(loox720_ads7846_device_info * dev,loox720_ads7846_spi_message * msg,int num_commands)
{
	int i;
	struct spi_transfer *x = &dev->xfer[0];
	struct spi_message  *m = &dev->msg[0];

	// copy the messages to our block
	for (i=0;i<num_commands;i++)
		dev->sendblock[i]=msg[i].cmd;

	spi_message_init(m);
	x->tx_buf = &dev->sendblock[0];
	x->rx_buf = &dev->receiveblock[0];
	x->len = num_commands*sizeof(dev->sendblock[0]); // 5 messages
	spi_message_add_tail(x, m);

	return spi_sync(dev->spi,m);
}

/* ==========================================================

	input layer reporting interface

   ========================================================== */

void loox720_ads7846_report(loox720_ads7846_device_info * dev, unsigned int Rt, unsigned int x, unsigned int y)
{
		if (dev->report_button)
		{
			// report button will be 2 or 1 
			// 2 is pressed, 1 is released			
			input_report_key(dev->input, BTN_TOUCH, 1);
			dev->report_button=0;
		}
		input_report_abs(dev->input, ABS_X, x);
		input_report_abs(dev->input, ABS_Y, y);
		input_report_abs(dev->input, ABS_PRESSURE, Rt);
		input_sync(dev->input);
}


#define convert_data_12( pos ) ((dev->receiveblock[ pos ] &0x3f)<<6)|((dev->receiveblock[ pos + 1] &0xfc00)>>10)


static volatile int loox720_ads7846_busy;

static void read_loox720_ads7846(void * ads_dev)
{
	// use safe values
	static int xmin = 400,xmax = 3610;
	static int ymin = 333,ymax = 3780;

	int i,x,y,z1,z2;
	loox720_ads7846_spi_message * cmds = ads_cmds;
	loox720_ads7846_device_info * dev = ads_dev;

	//printk( KERN_INFO "ADS7846 v2 starting to read data\n");
	
	SPI_read_write_block( dev , cmds , ARRAY_SIZE(ads_cmds));

	x  = convert_data_12( 1);
	y  = convert_data_12( 3 );
	z1 = convert_data_12( 0 );
	z2 = convert_data_12( 2 );
	
	if ((x>10)&&(y>10)) // valid measurement
	{
		// x min : 400	xmax: 3610
		// y min : 333	ymax: 3780

		// auto-adjust config:
		if ((x<xmin)&&(y<ymin)) { xmin = x;ymin = y;}
		if ((x>ymax)&&(y>ymax)) { xmax = x;ymax = y;}

		// normalize to 480 x 640
		x = (480* (x-xmin))/(xmax-xmin);
		y = (640* (y-ymin))/(ymax-ymin);
		x = 479-x; 
		// keep x,y in the screen
		if (x<0) x=0;if (x>479) x=479;
		if (y<0) y=0;if (y>639) y=639;
		dev->x=x;
		dev->y=y;		
		loox720_ads7846_report(dev, MAX_12BIT -1, dev->x ,dev->y);
	}
}
/* ==========================================================

	task that loops during pen-down

   ========================================================== */

static void looping_read_loox720_ads7846(/*void * ads_dev*/)
{
	loox720_ads7846_device_info * dev = &loox720_ads7846_dev;
	if (loox720_ads7846_busy==0)
	{
		loox720_ads7846_busy=1;
		// report pen down
		dev->report_button=1;
		do
		{
			read_loox720_ads7846(dev);
			if ((volatile int)dev->pendown==0) break; 
			msleep(dev->sample_rate);	
		} while ((volatile int)dev->pendown);
		// report pen up
		loox720_ads7846_report(dev, 0, dev->x ,dev->y);
		input_report_key(dev->input, BTN_TOUCH, 0);
		input_report_abs(dev->input, ABS_PRESSURE, 0);
		input_sync(dev->input);
	}
	loox720_ads7846_busy=0;
}

/* ==========================================================

	interrupt routines

   ========================================================== */

static DECLARE_WORK(printcoords_work, looping_read_loox720_ads7846);

static inline unsigned int readpen(loox720_ads7846_device_info *dev)	\
{										\
	if(dev)									\
		return (gpio_get_value(dev->pen )?1:0);			\
	printk( KERN_INFO "ADS7846 trying to read data with empty dev\n");	\
	return 0;								\
}


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
static char * myname="loox720_spi_ads7846";

static int __init loox720_ads7846_probe(struct spi_device *spi)
{
	struct input_dev *input_dev;
	int err;

	dev_set_drvdata(&spi->dev, &loox720_ads7846_dev);
	loox720_ads7846_dev.spi = spi;
	// =======================================================
	//	setup SPI to use 15 bit words in MODE_0
	// =======================================================


	spi->bits_per_word = 15;
	spi->mode = SPI_MODE_0;
	
	err = spi_setup(spi);
	if (err < 0)
		return err;

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
	input_set_abs_params(input_dev, ABS_X,0,480,0, 0);
	input_set_abs_params(input_dev, ABS_Y,0,640,0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0,MAX_12BIT, 0, 0);

	// =======================================================
	//	register our input device
	// =======================================================

	err = input_register_device(input_dev);
	if (err)
		goto err_no_mem;

	loox720_ads7846_dev.input = input_dev;

	if (request_irq(loox720_ads7846_dev.irq, pendown_interrupt,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
			myname, &loox720_ads7846_dev)) {
		printk(KERN_ERR "Failed to request loox720 ads7846 IRQ\n");
		err = -ENOMEM;
		goto err_remove_input;
		};

	return 0;

err_remove_input:
	input_free_device(input_dev);
err_no_mem:
	return err;
}

static int __devexit loox720_ads7846_remove(struct spi_device *spi)
{
	loox720_ads7846_device_info * dev_info = dev_get_drvdata(&spi->dev);
	free_irq(dev_info->irq,dev_info);
	if (dev_info->input) input_unregister_device(dev_info->input);
	return 0;
}

static struct spi_driver loox720_ads7846_driver = {
	.driver = {
		.name	= "loox720_spi_ads7846",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= loox720_ads7846_probe,
	.remove		= __devexit_p(loox720_ads7846_remove),
	//.suspend	= loox720_ads7846_suspend,
	//.resume	= loox720_ads7846_resume,
};

static int __init loox720_ads7846_init(void)
{
	return spi_register_driver(&loox720_ads7846_driver);
}
module_init(loox720_ads7846_init);

static void __exit loox720_ads7846_exit(void)
{
	spi_unregister_driver(&loox720_ads7846_driver);
}
module_exit(loox720_ads7846_exit);

MODULE_AUTHOR ("Jan Rinze Peterzon");
MODULE_DESCRIPTION ("bitbanging Touchscreen support for LOOX718/720");
MODULE_LICENSE ("GPL");

