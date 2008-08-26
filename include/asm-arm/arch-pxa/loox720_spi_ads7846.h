
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
	unsigned int 		mosi,miso,clock,cs,irq,pen;
	unsigned int 		x,y;
	unsigned int 		sample_rate,pendown;
	void * 			input;
	struct spi_device 	*spi;
	struct spi_transfer	xfer[5];
	struct spi_message	msg[5];
	unsigned short 		sendblock[10];
	unsigned short		receiveblock[10];
	unsigned int		report_button;
} loox720_ads7846_device_info;

