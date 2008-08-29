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
#include <linux/spinlock.h>

#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/hwmon.h>

#include <asm/hardware.h>
#include <asm/arch/loox720.h>
#include <asm/arch/loox720-gpio.h>
#include <asm/arch/loox720-cpld.h>
#include <asm/arch/loox720_spi_ads7846.h>

#define TS_POLL_DELAY(x)	(((x) - 1) * 1000000) // sample rate = +/- 5Hz with 200kHz spi bus

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

static loox720_ads7846_spi_message ads_cmds[] = {
	{ .cmd = READ_X(1) << 7, }, // setup x 
	{ .cmd = READ_X(0) << 7, }, // read x 
	{ .cmd = READ_Y(1) << 7, }, // setup y
	{ .cmd = READ_Y(0) << 7, }, // read y
	{ .cmd = READ_Z1(1) << 7, }, // read z1
	{ .cmd = READ_Z2(1) << 7, }, // read z2
	{ .cmd = READ_X(1) << 7, }, // penirq fix
	{ .cmd = PWRDOWN << 7, }, // nop
};

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)

static loox720_ads7846_spi_message ads_hwmon_cmds[] = {
	{ .cmd = READ_12BIT_SER(temp0) << 7, }, // read temp0
	{ .cmd = READ_12BIT_SER(vbatt) << 7, }, // read vbatt
	{ .cmd = READ_12BIT_SER(vaux) << 7, }, // read vaux
	{ .cmd = READ_12BIT_SER(temp1) << 7, }, // read temp1
	{ .cmd = READ_X(1) << 7, }, // fix penirq
	{ .cmd = READ_X(0) << 7, }, // fix penirq
	{ .cmd = PWRDOWN << 7, }, // power down
	{ .cmd = 0x0000, }, // nop
};

#endif

/* ==========================================================
	SPI block read/write interface
   ========================================================== */

int SPI_read_write_block(loox720_ads7846_device_info * dev, loox720_ads7846_spi_message * msg, int num_commands, void (*callback)(void *))
{
	int i;
	struct spi_transfer *x = &dev->xfer;
	struct spi_message  *m = &dev->msg;

	// copy the messages to our block
	for (i=0;i<num_commands;i++)
		dev->sendblock[i]=msg[i].cmd;

	spi_message_init(m);
	x->tx_buf = &dev->sendblock[0];
	x->rx_buf = &dev->receiveblock[0];
	x->len = num_commands*sizeof(dev->sendblock[0]); // 8 messages
	spi_message_add_tail(x, m);
	
	m->complete = callback;
	m->context = dev;

	return spi_async(dev->spi,m);
}

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
/* ==========================================================
	hwmon interface
   ========================================================== */

static void SPI_hwmon_read_write_block(loox720_ads7846_device_info * ads, loox720_ads7846_spi_message * msg, int num_commands)
{
	int i;
	struct spi_transfer *x = &ads->hwmon_xfer;
	struct spi_message  *m = &ads->hwmon_msg;

	// copy the messages to our block
	for (i=0;i<num_commands;i++)
		ads->hwmon_sendblock[i]=msg[i].cmd;

	spi_message_init(m);
	x->tx_buf = &ads->hwmon_sendblock[0];
	x->rx_buf = &ads->hwmon_receiveblock[0];
	x->len = num_commands*sizeof(ads->hwmon_sendblock[0]); // 8 messages
	spi_message_add_tail(x, m);
	
	printk(KERN_DEBUG "loox720_spi_ads7846: Reading hwmon data\n");

	spi_sync(ads->spi,m);
	
	ads->hwmon_data.temp0  = convert_hwmon_data_12( ads, 0);
	ads->hwmon_data.vbatt  = convert_hwmon_data_12( ads, 1 );
	ads->hwmon_data.vaux = convert_hwmon_data_12( ads, 2 );
	ads->hwmon_data.temp1 = convert_hwmon_data_12( ads, 3 );
}

#define SHOW(name, var, adjust) static ssize_t \
name ## _show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	ssize_t v; \
	loox720_ads7846_device_info *ads = dev_get_drvdata(dev); \
	\
	SPI_hwmon_read_write_block( ads , ads_hwmon_cmds , ARRAY_SIZE(ads_hwmon_cmds)); \
	v = ads->hwmon_data.var; \
	\
	if (v < 0) \
		return v; \
	return sprintf(buf, "%u\n", adjust(ads, v)); \
} \
static DEVICE_ATTR(name, S_IRUGO, name ## _show, NULL);


/* Sysfs conventions report temperatures in millidegrees Celcius.
 * ADS7846 could use the low-accuracy two-sample scheme, but can't do the high
 * accuracy scheme without calibration data.  For now we won't try either;
 * userspace sees raw sensor values, and must scale/calibrate appropriately.
 */
static inline unsigned null_adjust(loox720_ads7846_device_info *ads, ssize_t v)
{
	return v;
}
SHOW(temp0, temp0, null_adjust)		/* temp1_input */
SHOW(temp1, temp1, null_adjust)		/* temp2_input */

/* sysfs conventions report voltages in millivolts.  We can convert voltages
 * if we know vREF.  userspace may need to scale vAUX to match the board's
 * external resistors; we assume that vBATT only uses the internal ones.
 */
static inline unsigned vaux_adjust(loox720_ads7846_device_info *ads, ssize_t v)
{
	unsigned retval = v;

	/* external resistors may scale vAUX into 0..vREF */
	retval *= ads->vref_mv;
	retval = retval >> 12;
	return retval;
}
SHOW(in0_input, vaux, vaux_adjust)

static inline unsigned vbatt_adjust(loox720_ads7846_device_info *ads, ssize_t v)
{
	unsigned retval = vaux_adjust(ads, v);

	retval *= 4;
	return retval;
}
SHOW(in1_input, vbatt, vbatt_adjust)

static struct attribute *ads7846_attributes[] = {
	&dev_attr_temp0.attr,
	&dev_attr_temp1.attr,
	&dev_attr_in0_input.attr,
	&dev_attr_in1_input.attr,
	NULL,
};

static struct attribute_group ads7846_attr_group = {
	.attrs = ads7846_attributes,
};

static int ads784x_hwmon_register(struct spi_device *spi, loox720_ads7846_device_info *ads)
{
	struct device *hwmon;
	int err;

	if (!ads->vref_mv) {
		dev_dbg(&spi->dev, "assuming 2.5V internal vREF\n");
		ads->vref_mv = 2500;
	}
	
	ads->attr_group = &ads7846_attr_group;

	err = sysfs_create_group(&spi->dev.kobj, ads->attr_group);
	if (err)
		return err;

	hwmon = hwmon_device_register(&spi->dev);
	if (IS_ERR(hwmon)) {
		sysfs_remove_group(&spi->dev.kobj, ads->attr_group);
		return PTR_ERR(hwmon);
	}

	ads->hwmon = hwmon;
	return 0;
}

static void ads784x_hwmon_unregister(struct spi_device *spi,
				     loox720_ads7846_device_info *ads)
{
	if (ads->hwmon) {
		sysfs_remove_group(&spi->dev.kobj, ads->attr_group);
		hwmon_device_unregister(ads->hwmon);
	}
}

#endif

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

/* ==========================================================
	spi routines
   ========================================================== */

static void loox720_ads7846_callback(void *data)
{
	loox720_ads7846_device_info * ads = data;
	int x, y, z1, z2;
	
	if(ads->x > 10 && ads->y > 10)
		if(!readpen(ads))
			loox720_ads7846_report(ads, ads->rt, ads->x ,ads->y);

	x  = convert_data_12( ads, 1);
	y  = convert_data_12( ads, 3 );
	z1 = convert_data_12( ads, 4 );
	z2 = convert_data_12( ads, 5 );
	
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
		if (likely(x && z1)) {
			/* compute touch pressure resistance using equation #2 */
			ads->rt = z2;
			ads->rt -= z1;
			ads->rt *= x;
			ads->rt *= 400;
			ads->rt /= z1;
			ads->rt = (ads->rt + 2047) >> 12;
		} else
			ads->rt = 0;
		
		ads->x=x;
		ads->y=y;		
	}
	
	if (!readpen(ads)) {
		hrtimer_start(&ads->timer, ktime_set(0, TS_POLL_DELAY(ads->sample_rate)),
			      HRTIMER_MODE_REL);
	} else {
		// report pen up
		//loox720_ads7846_report(ads, 0, ads->x, ads->y);
		ads->x = 0;
		ads->y = 0;
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
	
	if(!readpen(ads)) {
		read_loox720_ads7846(ads);
	} else {
		// report pen up
//		loox720_ads7846_report(ads, 0, ads->oldx, ads->oldy);
		ads->x = 0;
		ads->y = 0;
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
	
	// report pen down
	ads->report_button=1;
	read_loox720_ads7846(ads);
}

/* ==========================================================
	interrupt routines
   ========================================================== */

irqreturn_t pendown_interrupt(int irq, void *data)
{
	loox720_ads7846_device_info *ads = data;
	
	if (readpen(ads)==0)
	{
		if (ads->busy==0)
		{
			ads->busy=1;
			schedule_work(&ads->work);
		}
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
	ads->pen_recheck_usecs = 100;
	
	ads->pen = GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N;
	ads->irq = IRQ_GPIO(GPIO_NR_LOOX720_TOUCHPANEL_IRQ_N);
	ads->sample_rate = 10; // delay in ms
	
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
			IRQF_TRIGGER_FALLING,
			myname, ads)) {
		printk(KERN_ERR "Failed to request loox720 ads7846 IRQ\n");
		err = -ENOMEM;
		goto err_remove_input;
		};
	
#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	ads784x_hwmon_register(spi, ads);
#endif

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
#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	ads784x_hwmon_unregister(spi, dev_info);
#endif
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
