/*
 * loox7xx.c  --  SoC audio for Loox 7xx
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  Mainstone audio amplifier code taken from arch/arm/mach-pxa/mainstone.c
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    30th Oct 2005   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>
#include <asm/hardware.h>

#include "../codecs/wm8750.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-i2s.h"

#include <asm/arch/loox720-cpld.h>
#include <asm/arch/loox720-gpio.h>

#include <asm/arch/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>

static int loox_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params);
static int loox_wm8750_init(struct snd_soc_codec *codec);

static struct snd_soc_machine loox;


static struct snd_soc_ops loox_ops = {
//	.startup = loox_startup,
	.hw_params = loox_hw_params,
};

static struct snd_soc_dai_link loox_dai = {
	.name = "WM8750",
	.stream_name = "WM8750 HiFi",
	.cpu_dai = &pxa_i2s_dai,
	.codec_dai = &wm8750_dai,
	.init = loox_wm8750_init,
	.ops = &loox_ops,
};

static struct snd_soc_machine loox = {
	.name = "Loox Audio",
	.dai_link = &loox_dai,
	.num_links = 1,
};

static struct wm8750_setup_data loox_wm8750_setup = {
	.i2c_address = 0x1a,
};

static struct snd_soc_device loox_snd_devdata = {
	.machine = &loox,
	.platform = &pxa2xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8750,
	.codec_data = &loox_wm8750_setup,
};

static struct platform_device *loox_snd_device;



/* irq for headphone switch GPIO */
static unsigned int hp_switch_irq ;

/* for headphone switch IRQ handling */
static struct work_struct hp_switch_task;
static struct workqueue_struct *hp_switch_wq;


static int loox_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		clk = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, WM8750_SYSCLK, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	// we have setup all registers when we booted..
	// so we probably don't need this anymore..
	// also.. it does not compile ;-)
	//pxa_gpio_mode(GPIO_NR_LOOX720_I2S_SYSCLK_MD);

	return 0;
}

static void loox_wm8750_hp_switch() 
{
	int reg;
	int hp_in;
	if (loox_snd_devdata.dev != NULL) {
	    hp_in = gpio_get_value(GPIO_NR_LOOX720_HEADPHONE_DET);
	    if (hp_in) {
		reg = loox_snd_devdata.codec->read(loox_snd_devdata.codec, WM8750_PWR2);
		loox_snd_devdata.codec->write(loox_snd_devdata.codec, WM8750_PWR2, (reg & 0xffe5) | 0x0040);
	    } else {
		reg = loox_snd_devdata.codec->read(loox_snd_devdata.codec, WM8750_PWR2);
		loox_snd_devdata.codec->write(loox_snd_devdata.codec, WM8750_PWR2, (reg & 0xffbf) | 0x0022); 
	    }
	    enable_irq(IRQ_GPIO(GPIO_NR_LOOX720_HEADPHONE_DET));
	}
}

static int loox_wm8750_hp_switch_isr(int irq, void *data)
{
	disable_irq(IRQ_GPIO(GPIO_NR_LOOX720_HEADPHONE_DET));
	PREPARE_WORK(&hp_switch_task, loox_wm8750_hp_switch);
	queue_work(hp_switch_wq, &hp_switch_task);
	return IRQ_HANDLED;
}

static int loox_wm8750_init(struct snd_soc_codec *codec)
{
	hp_switch_irq = IRQ_GPIO(GPIO_NR_LOOX720_HEADPHONE_DET);
	if (request_irq( hp_switch_irq, loox_wm8750_hp_switch_isr,IRQF_DISABLED|
	    IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "Headphone detect", NULL) != 0) {
		printk( KERN_ERR "Unable to aquire headphone detect IRQ.\n" );
		free_irq( hp_switch_irq, NULL );
		hp_switch_irq =0;
		return 0;
	}
	hp_switch_wq = create_workqueue("HP_SWITCH_WQ");
	disable_irq(IRQ_GPIO(GPIO_NR_LOOX720_HEADPHONE_DET));
	INIT_WORK(&hp_switch_task, loox_wm8750_hp_switch);
	queue_work(hp_switch_wq, &hp_switch_task);	
return 0;
}

static int __init loox_init(void)
{
	int ret;
	printk(KERN_INFO "Loox 720 SoC sound Driver\n");
	loox720_egpio_set_bit(LOOX720_CPLD_SOUND_BIT, 1);
	loox720_egpio_set_bit(LOOX720_CPLD_SND_AMPLIFIER_BIT, 1);

	loox_snd_device = platform_device_alloc("soc-audio", -1);
	if (!loox_snd_device)
		return -ENOMEM;

	platform_set_drvdata(loox_snd_device, &loox_snd_devdata);
	loox_snd_devdata.dev = &loox_snd_device->dev;
	ret = platform_device_add(loox_snd_device);
	if (ret)
		platform_device_put(loox_snd_device);
	return ret;
}

static void __exit loox_exit(void)
{
	platform_device_unregister(loox_snd_device);
	if (hp_switch_irq != 0)
	    free_irq( hp_switch_irq, NULL);
    loox720_egpio_set_bit(LOOX720_CPLD_SOUND_BIT, 0);
    loox720_egpio_set_bit(LOOX720_CPLD_SND_AMPLIFIER_BIT, 0);
}

module_init(loox_init);
module_exit(loox_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC Loox");
MODULE_LICENSE("GPL");
