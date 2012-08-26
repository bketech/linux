/*
 * beat-thang.c
 * ASoC driver for Beat Thang
 * Copyright 2009, IVL Audio Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <asm/div64.h>
#include "beat-thang.h"

#define BTAUDIO_VERSION "0.1"

/*
 * Debug
 */

static u16 btaudio_regs[1] = { 0x7 };

struct pll_state {
	unsigned int in;
	unsigned int out;
};

struct btaudio_priv {
	struct pll_state a;
};

/*
 * we have three "registers" which all report that the DACs/ADCs are permanently on
 * don't allow modification
 */
static int btaudio_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	BUG_ON(reg > ARRAY_SIZE(btaudio_regs));
	pr_debug("%s: %u %u\n", __func__, reg, value);
	return 0;
}

static unsigned int btaudio_read(struct snd_soc_codec *codec, unsigned int reg)
{
	BUG_ON(reg > ARRAY_SIZE(btaudio_regs));
	pr_debug("%s: %u %u\n", __func__, reg, btaudio_regs[reg]);
	return btaudio_regs[reg];
}

static const struct snd_kcontrol_new btaudio_snd_controls[] = {
		/* empty */
};

/* Add non-DAPM controls */
static int btaudio_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(btaudio_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&btaudio_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static const struct snd_soc_dapm_widget btaudio_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC1", "Playback", 0, 0, 0),
	SND_SOC_DAPM_DAC("DAC2", "Playback", 0, 1, 0),

	SND_SOC_DAPM_OUTPUT("VOUT1L"),
	SND_SOC_DAPM_OUTPUT("VOUT1R"),
	SND_SOC_DAPM_OUTPUT("VOUT2L"),
	SND_SOC_DAPM_OUTPUT("VOUT2R"),

	SND_SOC_DAPM_ADC("ADC", "Capture", 0, 2, 0),

	SND_SOC_DAPM_INPUT("AINL"),
	SND_SOC_DAPM_INPUT("AINR"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"VOUT1L", NULL, "DAC1"},
	{"VOUT1R", NULL, "DAC1"},

	{"VOUT2L", NULL, "DAC2"},
	{"VOUT2R", NULL, "DAC2"},

	{"ADC", NULL, "AINL"},
	{"ADC", NULL, "AINR"},
};

static int btaudio_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, btaudio_dapm_widgets,
				  ARRAY_SIZE(btaudio_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int btaudio_set_dai_pll(struct snd_soc_dai *codec_dai,
			      int pll_id, unsigned int freq_in,
			      unsigned int freq_out)
{
	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int btaudio_paif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int btaudio_set_paif_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int fmt)
{
	return 0;
}

static int btaudio_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
				 int div_id, int div)
{
	return 0;
}

static int btaudio_set_paif_dai_sysclk(struct snd_soc_dai *codec_dai,
                                int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int btaudio_digital_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int btaudio_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}

	codec->bias_level = level;
	return 0;
}

#define BTAUDIO_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE \
		| SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops btaudio_dai_ops = {
		.hw_params = btaudio_paif_hw_params,
		.set_fmt = btaudio_set_paif_dai_fmt,
		.set_sysclk = btaudio_set_paif_dai_sysclk,
		.set_clkdiv = btaudio_set_dai_clkdiv,
		.set_pll = btaudio_set_dai_pll,
		.digital_mute = btaudio_digital_mute,
};

struct snd_soc_dai btaudio_dai[] = {
	{
		.name = "BTAudio TXRX",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_44100,
			.formats = BTAUDIO_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100,
			.formats = BTAUDIO_FORMATS,
		},
		.ops = &btaudio_dai_ops,
	},
};

EXPORT_SYMBOL_GPL(btaudio_dai);

static struct snd_soc_codec *btaudio_codec;

static int btaudio_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	codec = btaudio_codec;
	socdev->card->codec = btaudio_codec;

	/* register pcms */
	pr_info("btaudio_probe: snd_soc_new_pcms\n");
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		return ret;
	}

	btaudio_add_controls(codec);
	btaudio_add_widgets(codec);

	btaudio_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	pr_info("btaudio_probe: snd_soc_init_card\n");
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		pr_err("beat-thang: failed to register audio card\n");
		snd_soc_free_pcms(socdev);
		snd_soc_dapm_free(socdev);
		return ret;
	}

	return 0;
}

/* power down chip */
static int btaudio_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		btaudio_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

/*
 * initialize the driver
 * register the mixer and dsp interfaces with the kernel
 */
static int btaudio_register(void)
{
	struct snd_soc_codec *codec;
	struct btaudio_priv *btaudio;
	int ret = 0;

	if (btaudio_codec) {
		pr_err("Multiple BTAudio devices not supported\n");
		return -ENOMEM;
	}

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	btaudio = kzalloc(sizeof(struct btaudio_priv), GFP_KERNEL);
	if (btaudio == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = btaudio;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->control_data = NULL;

	codec->dev = NULL;
	codec->name = "BTAudio";
	codec->owner = THIS_MODULE;
	codec->read = btaudio_read;
	codec->write = btaudio_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = btaudio_set_bias_level;
	codec->dai = btaudio_dai;
	codec->num_dai = ARRAY_SIZE(btaudio_dai);
	codec->reg_cache_size = ARRAY_SIZE(btaudio_regs);
	codec->reg_cache_step = sizeof(btaudio_regs[0]);
	codec->reg_cache = kmemdup(btaudio_regs, sizeof(btaudio_regs),
				   GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	btaudio_codec = codec;
	btaudio_dai[0].dev = NULL;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err_codec_reg;
	}

	ret = snd_soc_register_dais(btaudio_dai, ARRAY_SIZE(btaudio_dai));
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_codec_reg;
	}

	return 0;

err_codec_reg:
	kfree(btaudio);
	kfree(codec);
	return ret;
}

struct snd_soc_codec_device soc_codec_dev_btaudio = {
	.probe = btaudio_probe,
	.remove = btaudio_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_btaudio);

static int __init btaudio_modinit(void)
{
	return btaudio_register();
}
module_init(btaudio_modinit);

static void __exit btaudio_exit(void)
{

}
module_exit(btaudio_exit);

MODULE_DESCRIPTION("ASoC Beat Thang driver");
MODULE_AUTHOR("IVL Audio Inc.");
MODULE_LICENSE("GPL");
