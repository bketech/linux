/*
 * imx-ivlboard-beat-thang.c  --  SoC audio for imx_ivlboard
 *
 * Copyright 2008-2009 Freescale  Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/mxc.h>

#include "imx-pcm.h"
#include "imx-esai.h"
#include "../codecs/beat-thang.h"

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
#error ASRC not supported for Beat Thang codec
#endif

struct imx_ivlboard_pcm_state {
	int lr_clk_active;
};
extern void gpio_activate_esai_ports(void);
extern void gpio_deactivate_esai_ports(void);
extern void gpio_audiodetect_active(void);
extern void gpio_audiodetect_inactive(void);

extern unsigned int gpio_hp1_detect(void);
extern unsigned int gpio_hp2_detect(void);
extern unsigned int gpio_mic_in_detect(void);
extern unsigned int gpio_line_in_detect(void);

extern void gpio_headphone1_amp_enable(bool enable);
extern void gpio_headphone2_amp_enable(bool enable);
extern void gpio_regulators_active(void);
extern void gpio_line_out_amp_enable(bool enable);
extern void gpio_adc_enable(bool enable);
extern void gpio_mic_phantom_enable(bool enable);
extern void gpio_audio_in_pwr_enable(bool enable);
extern void gpio_audio_attn_enable(bool enable);
extern void gpio_vdd_an_enable(bool enable);

static struct imx_ivlboard_pcm_state clk_state;
static struct platform_device *btaudio_pdev;
static struct i2c_client *tpa6130_client[2];

static int imx_ivlboard_startup(struct snd_pcm_substream *substream)
{
	clk_state.lr_clk_active++;
	pr_debug("%s: lr_clk_active = %i\n", __func__, clk_state.lr_clk_active);
	return 0;
}

static void imx_ivlboard_shutdown(struct snd_pcm_substream *substream)
{
	clk_state.lr_clk_active--;
	pr_debug("%s: lr_clk_active = %i\n", __func__, clk_state.lr_clk_active);
}

static int imx_ivlboard_pcm_link_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *pcm_link = rtd->dai;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int rate = params_rate(params);
	u32 dai_format;
	unsigned int channel = params_channels(params);
	struct imx_esai *esai_mode = (struct imx_esai *)cpu_dai->private_data;
	struct clk *ckie;
	unsigned long mclk_rate, bitclk_rate, divisor;

	ckie = clk_get(NULL, "ckie"); /* ckie is the audio clock */
	if (IS_ERR(ckie)) {
		pr_err("Can't access ckie\n");
		return;
	}
	mclk_rate = clk_get_rate(ckie);
	clk_put(ckie);

	pr_debug("%s: lr_clk_active = %i\n", __func__, clk_state.lr_clk_active);

	if (rate != 44100) {
		pr_info("Rate not support.\n");
		return -EINVAL;
	}

	/* I2S, don't invert clocks, ESAI is master clock, sync ADC/DAC */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	esai_mode->sync_mode = 1;
	esai_mode->network_mode = 1;

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai, channel == 1 ? 0x1 : 0x3, 2);

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);

	/* set the ESAI system clock as output */
	snd_soc_dai_set_sysclk(cpu_dai, ESAI_CLK_EXTAL, 0, SND_SOC_CLOCK_OUT);

	/* for ESAI the bitclk is the frame rate times 64 bits per frame, since
	 * the two words sent in a frame each take 32 bits
	 */
	bitclk_rate = 64 * rate;
	divisor = mclk_rate / bitclk_rate;

	/* ESAI automatically divides mclk by 2, so compensate for this */
	divisor /= 2;
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PSR,
			ESAI_TCCR_TPSR_BYPASS); /* divide by 1 */
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PM, 0); /* divide by 1 */
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_FP, divisor - 1); /* divide by divisor */

	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PSR,
			ESAI_RCCR_RPSR_BYPASS); /* divide by 1 */
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PM, 0); /* divide by 1 */
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_FP, divisor - 1); /* divide by divisor */

	return 0;
}

/*
 * imx_ivlboard btaudio DAI operations.
 */
static struct snd_soc_ops imx_ivlboard_pcm_link_ops = {
	.startup = imx_ivlboard_startup,
	.shutdown = imx_ivlboard_shutdown,
	.hw_params = imx_ivlboard_pcm_link_hw_params,
};

static int lineout_amp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_info("btaudio: Line out amp enabled\n");
		gpio_line_out_amp_enable(true);
	} else {
		pr_info("btaudio: Line out amp disabled\n");
		gpio_line_out_amp_enable(false);
	}

	return 0;
}

static int linein_event(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *kcontrol, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_info("Line in on\n");
		gpio_adc_enable(true);
	} else {
		pr_info("Line in off\n");
		gpio_adc_enable(false);
	}

	return 0;
}

/*
 * PHANTOM POWER
 */

static int btaudio_audio_in_state;
static int btaudio_phantom_state;

static int btaudio_get_audio_in(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = btaudio_audio_in_state;
	return 0;
}

static int btaudio_set_audio_in(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	if (btaudio_audio_in_state == ucontrol->value.enumerated.item[0])
		return 0;
	btaudio_audio_in_state = (bool)ucontrol->value.enumerated.item[0];
	gpio_audio_in_pwr_enable(btaudio_audio_in_state);
	return 0;
}

static int btaudio_get_phantom(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = btaudio_phantom_state;
	return 0;
}

static int btaudio_set_phantom(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	if (btaudio_phantom_state == ucontrol->value.enumerated.item[0])
		return 0;
	btaudio_phantom_state = (bool)ucontrol->value.enumerated.item[0];
	gpio_mic_phantom_enable(btaudio_phantom_state);
	return 0;
}

/*
 * MIC/LINE/HEADPHONE INSERT DETECT
 */
static int btaudio_hpjack_func[2];

struct btaudio_jack {
	struct driver_attribute *driver_attr;

	unsigned int (*gpio_access)(void);
	void (*do_insert)(void);
	void (*do_eject)(void);
	int irq;
	struct delayed_work	delayed_work;
};

static void jack_detect_handler(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct btaudio_jack *jack = container_of(dwork, struct btaudio_jack, delayed_work);
	struct platform_device *pdev = btaudio_pdev;

	BUG_ON(!jack->gpio_access);

	sysfs_notify(&pdev->dev.kobj, NULL, jack->driver_attr->attr.name);

	if (jack->gpio_access()) {
		pr_debug("%s in\n", jack->driver_attr->attr.name);
		set_irq_type(jack->irq, IRQ_TYPE_EDGE_FALLING);
		if (jack->do_insert)
			jack->do_insert();
	} else {
		pr_debug("%s out\n", jack->driver_attr->attr.name);
		set_irq_type(jack->irq, IRQ_TYPE_EDGE_RISING);
		if (jack->do_eject)
			jack->do_eject();
	}
	enable_irq(jack->irq);
}

static irqreturn_t jack_irq_handler(int irq, void *data)
{
	struct btaudio_jack *jack = data;

	disable_irq_nosync(jack->irq);
	schedule_delayed_work(&jack->delayed_work, msecs_to_jiffies(200));

	return IRQ_HANDLED;
}

static ssize_t jack_show_attr(struct btaudio_jack *jack, char *buf)
{
	unsigned int status;

	status = jack->gpio_access();
	strcpy(buf, status ? "1\n" : "0\n");

	return strlen(buf);
}

static ssize_t jack_headphone1_show(struct device_driver *dev, char *buf);
static DRIVER_ATTR(jack_headphone1, S_IRUGO, jack_headphone1_show, NULL);
static struct btaudio_jack jack_headphone1 = {
		.driver_attr = &driver_attr_jack_headphone1,
		.gpio_access = gpio_hp1_detect,
};
static ssize_t jack_headphone1_show(struct device_driver *dev, char *buf)
{
	return jack_show_attr(&jack_headphone1, buf);
}

static ssize_t jack_headphone2_show(struct device_driver *dev, char *buf);
static DRIVER_ATTR(jack_headphone2, S_IRUGO, jack_headphone2_show, NULL);
static struct btaudio_jack jack_headphone2 = {
		.driver_attr = &driver_attr_jack_headphone2,
		.gpio_access = gpio_hp2_detect,
};
static ssize_t jack_headphone2_show(struct device_driver *dev, char *buf)
{
	return jack_show_attr(&jack_headphone2, buf);
}
static ssize_t jack_mic_show(struct device_driver *dev, char *buf);
static DRIVER_ATTR(jack_mic, S_IRUGO, jack_mic_show, NULL);
static struct btaudio_jack jack_mic = {
		.driver_attr = &driver_attr_jack_mic,
		.gpio_access = gpio_mic_in_detect,
};
static ssize_t jack_mic_show(struct device_driver *dev, char *buf)
{
	return jack_show_attr(&jack_mic, buf);
}

static ssize_t jack_line_in_show(struct device_driver *dev, char *buf);
static DRIVER_ATTR(jack_line_in, S_IRUGO, jack_line_in_show, NULL);
static struct btaudio_jack jack_line_in = {
		.driver_attr = &driver_attr_jack_line_in,
		.gpio_access = gpio_line_in_detect,
};
static ssize_t jack_line_in_show(struct device_driver *dev, char *buf)
{
	return jack_show_attr(&jack_line_in, buf);
}

static int btaudio_jack_setup(struct btaudio_jack *jack, struct platform_device *pdev, int irq)
{
	int ret;
	ret = driver_create_file(pdev->dev.driver, jack->driver_attr);
	if (ret < 0) {
		pr_err("%s:failed to create attribute %s\n", __func__, jack->driver_attr->attr.name);
		return ret;
	}

	INIT_DELAYED_WORK(&jack->delayed_work, jack_detect_handler);

	jack->irq = irq;
	ret = request_irq(jack->irq, jack_irq_handler,
			jack->gpio_access() ? IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING,
			pdev->name, jack);
	if (ret < 0) {
		pr_err("%s: request irq failed\n", __func__);
		return ret;
	}

	return ret;
}

static const char *hpjack_function[] = { "off", "on" };
static const char *attn_function[] = { "+4dBu", "-10dBV" };

static const struct soc_enum btaudio_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, hpjack_function),
	SOC_ENUM_SINGLE_EXT(2, attn_function),
};

static int btaudio_get_hpjack1(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = btaudio_hpjack_func[0];
	return 0;
}

static int btaudio_get_hpjack2(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = btaudio_hpjack_func[1];
	return 0;
}

static int btaudio_set_hpjack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol,
			     char *related_pin, int *state)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (*state == ucontrol->value.enumerated.item[0])
		return 0;

	*state = ucontrol->value.enumerated.item[0];
	if (*state)
		snd_soc_dapm_enable_pin(codec, related_pin);
	else
		snd_soc_dapm_disable_pin(codec, related_pin);

	snd_soc_dapm_sync(codec);
	return 1;

}

static int btaudio_set_hpjack1(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	return btaudio_set_hpjack(kcontrol, ucontrol, "Headphone 1 Jack", &btaudio_hpjack_func[0]);
}

static int btaudio_set_hpjack2(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	return btaudio_set_hpjack(kcontrol, ucontrol, "Headphone 2 Jack", &btaudio_hpjack_func[1]);
}

#define TPA6130_REG_CONTROL				1
#define TPA6130_REG_CONTROL_ENABLE_LR	((1 << 7) | (1 << 6))	/* enable L & R amps */
#define TPA6130_REG_CONTROL_SWS			(1 << 0)				/* H = software shutdown */

#define TPA6130_REG_VOLUME				2
#define TPA6130_REG_MUTE				((1 << 7) | (1 << 6))
#define TPA6130_REG_VOLUME_MASK			(0x3f)

static int hpjack1_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	s32 val;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		i2c_smbus_write_byte_data(tpa6130_client[0], TPA6130_REG_CONTROL,
				TPA6130_REG_CONTROL_ENABLE_LR); /* all on */
	} else {
		/* Enable software shutdown */
		val = i2c_smbus_read_byte_data(tpa6130_client[0], TPA6130_REG_CONTROL);
		val |= TPA6130_REG_CONTROL_SWS;
		i2c_smbus_write_byte_data(tpa6130_client[0], TPA6130_REG_CONTROL, val);
	}

	return 0;
}

static int hpjack2_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	s32 val;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		i2c_smbus_write_byte_data(tpa6130_client[1], TPA6130_REG_CONTROL,
				TPA6130_REG_CONTROL_ENABLE_LR); /* all on */
	} else {
		/* Enable software shutdown */
		val = i2c_smbus_read_byte_data(tpa6130_client[1], TPA6130_REG_CONTROL);
		val |= TPA6130_REG_CONTROL_SWS;
		i2c_smbus_write_byte_data(tpa6130_client[1], TPA6130_REG_CONTROL, val);
	}

	return 0;
}

static struct i2c_client * tpa6130_get_client(struct snd_kcontrol *kcontrol)
{
	if (strcmp(kcontrol->id.name, "Headphone 1 Volume") == 0)
		return tpa6130_client[0];
	else if (strcmp(kcontrol->id.name, "Headphone 2 Volume") == 0)
		return tpa6130_client[1];
	else
		return NULL;
}

static int tpa6130_i2c_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	s32 val;
	struct i2c_client *client;

	client = tpa6130_get_client(kcontrol);
	if (client) {
		val = i2c_smbus_read_byte_data(client, TPA6130_REG_VOLUME);
		pr_debug("i2c_get %i\n", val);
	} else {
		WARN_ONCE(!client, "I2C client for headphones not registered.  Do\n"
				"you need to enable a I2C client in kernel config?\n");
		val = 0;
	}

	if (val < 0) {
		if (val == -EREMOTEIO) {
			ucontrol->value.integer.value[0] = 0;	/* If I2C is broken, say we're muted */
			return 0;
		}
		return val;
	}

	if (val & TPA6130_REG_MUTE) {
		ucontrol->value.integer.value[0] = 0;	/* Mute flag(s) set */
	} else {
		/* +1 :Translate offset */
		ucontrol->value.integer.value[0] = (val & TPA6130_REG_VOLUME_MASK) + 1;
	}
	return 0;
}

static int tpa6130_i2c_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	s32 val;
	int err;
	struct i2c_client *client;

	client = tpa6130_get_client(kcontrol);
	if (!client) {
		ucontrol->value.integer.value[0] = 0;	/* If I2C is broken, say we're muted */
		return 0;
	}

	if (ucontrol->value.integer.value[0] == 0) {
		val = TPA6130_REG_MUTE;
	} else {
		/* -1 :Translate offset */
		val = (ucontrol->value.integer.value[0] - 1) & TPA6130_REG_VOLUME_MASK;
	}

	pr_debug("i2c_put %i\n", val);

	err = i2c_smbus_write_byte_data(client, TPA6130_REG_VOLUME, val);
	if (err < 0) {
		if (err == -EREMOTEIO) {
			ucontrol->value.integer.value[0] = 0;	/* If I2C is broken, say we're muted */
			return 0;
		}
		return err;
	}

	return err;
}

/* There is almost no documentation anywhere to be found on this feature.
 * The data in this table is a piecewise linear mapping of the headphone
 * IC's register values to their corresponding dB values with error not
 * exceeding 1.0 dB for -30 dB to the maximum.  Below -30 dB the error is
 * worse because it doesn't matter that much.
 * The underlying table is a cubic function with the end tapered off.
 * I still can't find a definition of the term "TLV".
 */
static const unsigned int db_scale_headphones[] = {
		/* Number of items in the freeform table */
		TLV_DB_RANGE_HEAD(4),

		/* Set of piecewise linear approximations to the IC's volume curve.
		 * 0 is mute (indicated by the third parameter being 1)
		 * 1 to 11 is -56.88 + 2.62 * [idx - 0] dB
		 * 11 to 21 is -33.30 + 1.28 * [idx - 11] dB
		 * etc.
		 * The chip's table is [0-63] with the top bits being mute.
		 * The driver expects mute to be at the bottom, we adjust the mapping
		 * to [0] = chip mute, [1-64] --> chip [0-63]
		 */
		0, 11, TLV_DB_SCALE_ITEM(-5688, 262, 1),
		11, 21, TLV_DB_SCALE_ITEM(-3330, 128, 0),
		21, 41, TLV_DB_SCALE_ITEM(-2050, 72, 0),
		41, 64, TLV_DB_SCALE_ITEM(-610, 44, 0),
};

/*
 * LINE OUT ATTENUATION
 */
static int line_out_attn;
static int btaudio_get_attn(struct snd_kcontrol *kcontrol,
	     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = line_out_attn;
	return 0;
}

static int btaudio_set_attn(struct snd_kcontrol *kcontrol,
	     struct snd_ctl_elem_value *ucontrol)
{
	line_out_attn = ucontrol->value.enumerated.item[0];
	gpio_audio_attn_enable(line_out_attn);
	return 0;
}

static const struct snd_kcontrol_new btaudio_machine_controls[] = {
	SOC_ENUM_EXT("Audio Input", btaudio_enum[0], btaudio_get_audio_in,
			btaudio_set_audio_in),
	SOC_ENUM_EXT("Phantom Power", btaudio_enum[0], btaudio_get_phantom,
			btaudio_set_phantom),
	SOC_ENUM_EXT("Headphone 1 Toggle", btaudio_enum[0], btaudio_get_hpjack1,
			btaudio_set_hpjack1),
	SOC_ENUM_EXT("Headphone 2 Toggle", btaudio_enum[0], btaudio_get_hpjack2,
			btaudio_set_hpjack2),
	SOC_ENUM_EXT("Line Out Attenuation", btaudio_enum[1], btaudio_get_attn,
			btaudio_set_attn),
	SOC_SINGLE_EXT_TLV("Headphone 1 Volume", 2, 0, 64, 0,
			tpa6130_i2c_get, tpa6130_i2c_put, db_scale_headphones),
	SOC_SINGLE_EXT_TLV("Headphone 2 Volume", 2, 0, 64, 0,
			tpa6130_i2c_get, tpa6130_i2c_put, db_scale_headphones),
};



static int tpa6130_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	pr_info("%s id=%s client=%p\n", __func__, id->name, client);
	BUG_ON(id->driver_data >= ARRAY_SIZE(tpa6130_client));
	tpa6130_client[id->driver_data] = client;
	return 0;
}

static int tpa6130_i2c_remove(struct i2c_client *client)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tpa6130_client); i++) {
		if (tpa6130_client[i] == client) {
			tpa6130_client[i] = NULL;
			return 0;
		}
	}

	return -EINVAL;
}


static const struct i2c_device_id tpa6130_id[] = {
	{ "tpa6130-headphone1", 0 },
	{ "tpa6130-headphone2", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, tpa6130_id);


static struct i2c_driver tpa6130_i2c_driver = {
		.driver = {
				.name = "TPA6130 Headphone Amp",
				.owner = THIS_MODULE,
		},
		.probe = tpa6130_i2c_probe,
		.remove = tpa6130_i2c_remove,
		.id_table = tpa6130_id,
};

static int btaudio_add_i2c_hpamp(struct platform_device *pdev)
{
	int ret;

	ret = i2c_add_driver(&tpa6130_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "Can't add i2c driver\n");
		return ret;
	}

	return 0;
}

/* imx_ivlboard machine dapm widgets */
static const struct snd_soc_dapm_widget imx_ivlboard_dapm_widgets[] = {
//	SND_SOC_DAPM_MIC("Mic In", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", linein_event),
	SND_SOC_DAPM_LINE("Line Out Jack", lineout_amp_event),
	SND_SOC_DAPM_HP("Headphone 1 Jack", hpjack1_event),
	SND_SOC_DAPM_HP("Headphone 2 Jack", hpjack2_event),
};

/* machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {
		{"AINL", "Audio Input", "Line In Jack"},
		{"AINR", "Audio Input", "Line In Jack"},

		{"Line Out Jack", NULL, "VOUT1L"},
		{"Line Out Jack", NULL, "VOUT1R"},
		{"Headphone 1 Jack", NULL, "VOUT2L"},
		{"Headphone 1 Jack", NULL, "VOUT2R"},
		{"Headphone 2 Jack", NULL, "VOUT2L"},
		{"Headphone 2 Jack", NULL, "VOUT2R"},
};




static int imx_ivlboard_btaudio_init(struct snd_soc_codec *codec)
{
	int i;
	int ret;

	/* Add machine specific controls */
	for (i = 0; i < ARRAY_SIZE(btaudio_machine_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&btaudio_machine_controls[i],
					       codec, NULL));
		if (ret < 0)
			return ret;
	}

	snd_soc_dapm_new_controls(codec, imx_ivlboard_dapm_widgets,
				  ARRAY_SIZE(imx_ivlboard_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link imx_ivlboard_dai = {
	.name = "btaudio",
	.stream_name = "btaudio",
	.codec_dai = btaudio_dai,
	.init = imx_ivlboard_btaudio_init,
	.ops = &imx_ivlboard_pcm_link_ops,
};

static int imx_ivlboard_card_remove(struct platform_device *pdev)
{
/*	struct snd_soc_device *socdev = platform_get_drvdata(pdev); */
	return 0;
}

static struct snd_soc_card snd_soc_card_imx_ivlboard = {
	.name = "imx-ivlboard",
	.platform = &imx_soc_platform,
	.dai_link = &imx_ivlboard_dai,
	.num_links = 1,
	.remove = imx_ivlboard_card_remove,
};

static struct snd_soc_device imx_ivlboard_snd_devdata = {
	.card = &snd_soc_card_imx_ivlboard,
	.codec_dev = &soc_codec_dev_btaudio,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_ivlboard_btaudio_probe(struct platform_device *pdev)
{
	struct mxc_btaudio_platform_data *plat = pdev->dev.platform_data;
	int ret;

	imx_ivlboard_dai.cpu_dai = &imx_esai_dai[2];

	btaudio_pdev = pdev;

	/* Configure audio ports */
	gpio_activate_esai_ports();
	gpio_regulators_active();
	gpio_audiodetect_active();

	/* Start headphone amps (for now) in "standby" mode */
	gpio_headphone1_amp_enable(true);
	gpio_headphone2_amp_enable(true);
	btaudio_hpjack_func[0] = btaudio_hpjack_func[1] = 1;
	gpio_line_out_amp_enable(true);

	ret = btaudio_jack_setup(&jack_headphone1, pdev, plat->hp_irq[0]);
	if (ret < 0)
		goto err_card_reg;
	ret = btaudio_jack_setup(&jack_headphone2, pdev, plat->hp_irq[1]);
	if (ret < 0)
		goto err_card_reg;
	if (plat->mic_in_irq >= 0) {
		ret = btaudio_jack_setup(&jack_mic, pdev, plat->mic_in_irq);
		if (ret < 0)
			goto err_card_reg;
	}
	ret = btaudio_jack_setup(&jack_line_in, pdev, plat->line_in_irq);
	if (ret < 0)
		goto err_card_reg;

	btaudio_add_i2c_hpamp(btaudio_pdev);

	return 0;

err_card_reg:
	gpio_headphone2_amp_enable(false);
	gpio_headphone1_amp_enable(false);
	gpio_audiodetect_inactive();
	gpio_deactivate_esai_ports();
	return ret;
}

static int __devexit imx_ivlboard_btaudio_remove(struct platform_device *pdev)
{
	/* TODO really should release btaudio jacks and IRQs here */
	gpio_headphone2_amp_enable(false);
	gpio_headphone1_amp_enable(false);
	gpio_audiodetect_inactive();
	gpio_deactivate_esai_ports();
	return 0;
}

static struct platform_driver imx_ivlboard_btaudio_driver = {
	.probe = imx_ivlboard_btaudio_probe,
	.remove = __devexit_p(imx_ivlboard_btaudio_remove),
	.driver = {
		   .name = "ivlboard-btaudio",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_ivlboard_snd_device;

static int __init imx_ivlboard_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_ivlboard_btaudio_driver);
	if (ret < 0)
		goto exit;
	imx_ivlboard_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_ivlboard_snd_device)
		goto err_device_alloc;
	platform_set_drvdata(imx_ivlboard_snd_device, &imx_ivlboard_snd_devdata);
	imx_ivlboard_snd_devdata.dev = &imx_ivlboard_snd_device->dev;
	ret = platform_device_add(imx_ivlboard_snd_device);
	if (0 == ret)
		goto exit;

	platform_device_put(imx_ivlboard_snd_device);
err_device_alloc:
	platform_driver_unregister(&imx_ivlboard_btaudio_driver);
exit:
	return ret;
}

static void __exit imx_ivlboard_asoc_exit(void)
{
	platform_driver_unregister(&imx_ivlboard_btaudio_driver);
	platform_device_unregister(imx_ivlboard_snd_device);
}

module_init(imx_ivlboard_asoc_init);
module_exit(imx_ivlboard_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC beat-thang imx_ivlboard");
MODULE_LICENSE("GPL");
