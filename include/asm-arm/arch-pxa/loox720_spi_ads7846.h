
/*--------------------------------------------------------------------------*/

/*
	The ADS7845 has touchscreen and other sensors.
 */

#define	MAX_12BIT	((1<<12)-1)


typedef struct loox720_ads7846_spi_message
{
	unsigned int		cmd;
	unsigned int		data;
} loox720_ads7846_spi_message;

typedef struct loox720_ads7846_device_info {
	unsigned int 		irq,pen;
	volatile unsigned int 	x, y, oldx, oldy;
	unsigned int		xmin, xmax, ymin, ymax;
	unsigned int 		sample_rate;
	volatile int		busy;
	volatile int		pendown;
	void * 			input;
	struct spi_device 	*spi;
	struct spi_transfer	xfer[5];
	struct spi_message	msg[5];
	unsigned short 		sendblock[10] __attribute__ ((aligned (8)));
	unsigned short		receiveblock[10] __attribute__ ((aligned (8)));
	unsigned int		report_button;
	struct hrtimer timer;
	struct work_struct work;	
} loox720_ads7846_device_info;

