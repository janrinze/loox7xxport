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

#define TS_INITIAL_DELAY	(1 * 1000000)
#define TS_POLL_DELAY(x)	((x) * 1000000)

// =======================================================
// hardcoded values for testing
// we should be using the pdev struct to retrieve this info..
// =======================================================
	
/*static loox720_ads7846_device_info loox720_ads7846_dev = {
	.pen	= GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N,
	.irq	= IRQ_GPIO(GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N),

	.sample_rate=5, // 5 msec for 200Hz sampling
};*/

/* ==========================================================
	ads7846 commands 
   ========================================================== */

static loox720_ads7846_spi_message ads_cmds[5] = {
	{ .cmd = 0xD3<<7, }, // setup x 
	{ .cmd = 0xD0<<7, }, // read x 
	{ .cmd = 0x93<<7, }, // setup y
	{ .cmd = 0x90<<7, }, // read y
	{ .cmd = 0x0000, }  // powerdown
};

/* ==========================================================
	SPI block read/write interface
   ========================================================== */

int SPI_read_write_block(loox720_ads7846_device_info * dev, loox720_ads7846_spi_message * msg, int num_commands, void (*callback)(void *))
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
	
	m->complete = callback;
	m->context = dev;

	return spi_async(dev->spi,m);
}

/* ==========================================================
	input layer reporting interface
   ========================================================== */

void loox720_ads7846_report(loox720_ads7846_device_info * dev, unsigned int Rt, unsigned int x, unsigned int y)
{
		if (dev->report_button)
		{
			input_report_key(dev->input, BTN_TOUCH, 1);
			dev->report_button=0;
		}
		input_report_abs(dev->input, ABS_X, x);
		input_report_abs(dev->input, ABS_Y, y);
		input_report_abs(dev->input, ABS_PRESSURE, Rt);
		input_sync(dev->input);
}


#define convert_data_12( data, pos ) (((data)->receiveblock[ pos ] &0x3f)<<6)|(((data)->receiveblock[ pos + 1] &0xfc00)>>10)

/* ==========================================================
	spi routines
   ========================================================== */

static void loox720_ads7846_callback(void *data)
{
	loox720_ads7846_device_info * ads = data;
	int x,y /*,z1,z2*/;

	x  = convert_data_12( ads, 1);
	y  = convert_data_12( ads, 3 );
//	z1 = convert_data_12( ads, 0 );
//	z2 = convert_data_12( ads, 2 );
	
	if ((x>10)&&(y>10)) // valid measurement
	{
		// x min : 400	xmax: 3610
		// y min : 333	ymax: 3780

#ifdef CONFIG_LOOX720_ADS7846_AUTOCAL
		// auto-adjust config:
		if ((x<ads->xmin)&&(y<ads->ymin)) { ads->xmin = x;ads->ymin = y;}
		if ((x>ads->ymax)&&(y>ads->ymax)) { ads->xmax = x;ads->ymax = y;}

		// normalize to 480 x 640
		// tom3q: this is slow imho... two multiplications and two divisions
		x = (480* (x-ads->xmin))/(ads->xmax-ads->xmin);
		y = (640* (y-ads->ymin))/(ads->ymax-ads->ymin);
		x = 479-x; 
		// keep x,y in the screen
		if (x<0) x=0;if (x>479) x=479;
		if (y<0) y=0;if (y>639) y=639;
#endif
		ads->oldx = ads->x;
		ads->oldy = ads->y;
		ads->x=x;
		ads->y=y;		
		loox720_ads7846_report(ads, MAX_12BIT -1, ads->x ,ads->y);
	}
	
	if (ads->pendown) {
		hrtimer_start(&ads->timer, ktime_set(0, TS_POLL_DELAY(ads->sample_rate)),
			      HRTIMER_MODE_REL);
	} else {
		// report pen up
		loox720_ads7846_report(ads, 0, ads->oldx, ads->oldy);
		input_report_key(ads->input, BTN_TOUCH, 0);
		input_report_abs(ads->input, ABS_PRESSURE, 0);
		input_sync(ads->input);
		ads->busy=0;
	}
}

/* ==========================================================
	reading routines
   ========================================================== */

static void read_loox720_ads7846(void * ads_dev)
{
	loox720_ads7846_spi_message * cmds = ads_cmds;
	loox720_ads7846_device_info * ads = ads_dev;

	//printk( KERN_INFO "ADS7846 v2 starting to read data\n");
	
	SPI_read_write_block( ads , cmds , ARRAY_SIZE(ads_cmds), loox720_ads7846_callback);
}

/* ==========================================================
	timer routines
   ========================================================== */

static enum hrtimer_restart loox720_ads7846_timer(struct hrtimer *handle)
{
	loox720_ads7846_device_info *ads = container_of(handle, loox720_ads7846_device_info, timer);
	
	if(ads->pendown) {
		read_loox720_ads7846(ads);
	} else {
		// report pen up
		loox720_ads7846_report(ads, 0, ads->oldx, ads->oldy);
		input_report_key(ads->input, BTN_TOUCH, 0);
		input_report_abs(ads->input, ABS_PRESSURE, 0);
		input_sync(ads->input);
		ads->busy=0;
	}
	
	return HRTIMER_NORESTART;
}

/* ==========================================================
	work routines
   ========================================================== */

static void loox720_ads7846_work(struct work_struct *work)
{
	loox720_ads7846_device_info * ads = container_of(work, loox720_ads7846_device_info, work);
	
	if (ads->busy==0)
	{
		ads->busy=1;
		// report pen down
		ads->report_button=1;
		read_loox720_ads7846(ads);
	}
}

/* ==========================================================
	interrupt routines
   ========================================================== */

static inline unsigned int readpen(loox720_ads7846_device_info *dev)	\
{										\
	if(dev)									\
		return (gpio_get_value(dev->pen )?1:0);			\
	printk( KERN_INFO "ADS7846 trying to read data with empty dev\n");	\
	return 0;								\
}


irqreturn_t pendown_interrupt(int irq, void *data)
{
	loox720_ads7846_device_info *ads = data;
	
	if (readpen(ads)==0)
	{
		ads->pendown=1;
		if (ads->busy==0)
		{
			schedule_work(&ads->work);
		}
	} else {
		ads->pendown=0;
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
	loox720_ads7846_device_info *ads;
	
	ads = kzalloc(sizeof(*ads), GFP_KERNEL);
	if(!ads)
		return -ENOMEM;
	
	dev_set_drvdata(&spi->dev, ads);
	ads->spi = spi;
	// =======================================================
	//	setup SPI to use 15 bit words in MODE_0
	// =======================================================

	hrtimer_init(&ads->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ads->timer.function = loox720_ads7846_timer;
	
	INIT_WORK(&ads->work, loox720_ads7846_work);

	spi->bits_per_word = 15;
	spi->mode = SPI_MODE_0;
	
	ads->xmin = 400;
	ads->xmax = 3610;
	ads->ymin = 333;
	ads->ymax = 3780;
	
	ads->pen = GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N;
	ads->irq = IRQ_GPIO(GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N);
	ads->sample_rate = 5; // delay in ms
	
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

	ads->input = input_dev;

	if (request_irq(ads->irq, pendown_interrupt,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
			myname, ads)) {
		printk(KERN_ERR "Failed to request loox720 ads7846 IRQ\n");
		err = -ENOMEM;
		goto err_remove_input;
		};

	return 0;

err_remove_input:
	input_free_device(input_dev);
err_no_mem:
	kfree(ads);
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

MODULE_AUTHOR ("Jan Rinze Peterzon & Tomasz Figa");
MODULE_DESCRIPTION ("SPI Touchscreen support for PocketLOOX 700 Series");
MODULE_LICENSE ("GPL");

