/*
 *  mango210_wm8960.c
 *
 *  Copyright (c) 2010 Crz-Tech Co. Ltd
 *  Author: Pyeongjeong Lee <leepjung@crz-tech.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/regs-clock.h>
#include <plat/regs-iis.h>
#include "../codecs/wm8960.h"
#include "s3c-dma.h"
#include "s5pc1xx-i2s.h"
#include "s3c-i2s-v2.h"

#include <linux/io.h>

#define I2S_NUM 0
#define SRC_CLK 66738000

/* #define CONFIG_SND_DEBUG */
#ifdef CONFIG_SND_DEBUG
#define debug_msg(x...) printk(x)
#else
#define debug_msg(x...)
#endif

/*  BLC(bits-per-channel) --> BFS(bit clock shud be >= FS*(Bit-per-channel)*2)*/
/*  BFS --> RFS(must be a multiple of BFS)                                  */
/*  RFS & SRC_CLK --> Prescalar Value(SRC_CLK / RFS_VAL / fs - 1)           */
static int mango210_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int bfs, rfs, ret;
	u32 ap_codec_clk;
	struct clk    *clk_out, *clk_epll;
	int psr;

	/* Choose BFS and RFS values combination that is supported by
	 * both the WM8994 codec as well as the S5P AP
	 *
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	/* Can take any RFS value for AP */
			bfs = 16;
			rfs = 256;
			break;
	case SNDRV_PCM_FORMAT_S16_LE:
	/* Can take any RFS value for AP */
			bfs = 32;
			rfs = 256;
			break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
			bfs = 48;
			rfs = 512;
			break;
	/* Impossible, as the AP doesn't support 64fs or more BFS */
	case SNDRV_PCM_FORMAT_S32_LE:
	default:
			return -EINVAL;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
					params_rate(params), SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;
#ifdef USE_CLKAUDIO
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C_CLKSRC_CLKAUDIO,
					params_rate(params), SND_SOC_CLOCK_OUT);

	if (ret < 0) {
		printk(KERN_ERR
			"%s : AP sys clock setting error!\n", __func__);
		return ret;
	}
#endif
	clk_out = clk_get(NULL, "clk_out");
	if (IS_ERR(clk_out)) {
			printk(KERN_ERR
				"failed to get CLK_OUT\n");
			return -EBUSY;
	}

	clk_epll = clk_get(NULL, "fout_epll");
	if (IS_ERR(clk_epll)) {
		printk(KERN_ERR
			"failed to get fout_epll\n");
		clk_put(clk_out);
		return -EBUSY;
	}

	if (clk_set_parent(clk_out, clk_epll)) {
		printk(KERN_ERR
			"failed to set CLK_EPLL as parent of CLK_OUT\n");
		clk_put(clk_out);
		clk_put(clk_epll);
		return -EBUSY;
	}


	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		clk_set_rate(clk_out, 12288000);
		ap_codec_clk = SRC_CLK/4;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	default:
		clk_set_rate(clk_out, 11289600);
		ap_codec_clk = SRC_CLK/6;
		break;
	}


		/* Calculate Prescalare/PLL values for supported Rates */
	psr = SRC_CLK / rfs / params_rate(params);
	ret = SRC_CLK / rfs - psr * params_rate(params);
	/* round off */
	if (ret >= params_rate(params)/2)
		psr += 1;

	psr -= 1;
	printk(KERN_INFO
		"SRC_CLK=%d PSR=%d RFS=%d BFS=%d\n", SRC_CLK, psr, rfs, bfs);

	/* Set the AP Prescalar/Pll */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER, psr);

	if (ret < 0) {
		printk(KERN_ERR
			"%s :\
				AP prescalar setting error!\n", __func__);
		return ret;
	}

	/* Set the AP RFS */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, rfs);
	if (ret < 0) {
		printk(KERN_ERR
			"%s : AP RFS setting error!\n", __func__);
		return ret;
	}

	/* Set the AP BFS */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_BCLK, bfs);

	if (ret < 0) {
		printk(KERN_ERR
			"%s : AP BCLK setting error!\n", __func__);
		return ret;
	}

	clk_put(clk_epll);
	clk_put(clk_out);

	return 0;
}

/*
 * MANGO210 WM8960 DAI operations.
 */
static struct snd_soc_ops mango210_ops = {
	.hw_params = mango210_hw_params,
};

/* MANGO210 Playback widgets */
static const struct snd_soc_dapm_widget wm8960_dapm_widgets_pbk[] = {
	SND_SOC_DAPM_HP("Speaker-L/R", NULL),
	SND_SOC_DAPM_HP("HP-L/R", NULL),
};

/* MANGO210 Capture widgets */
static const struct snd_soc_dapm_widget wm8960_dapm_widgets_cpt[] = {
	SND_SOC_DAPM_MIC("MicIn", NULL),
};

/* SMDK-PAIFTX connections */
static const struct snd_soc_dapm_route audio_map_tx[] = {
	/* MicIn feeds LINPUT1 */
	{"LINPUT1", NULL, "MicIn"},
};

/* SMDK-PAIFRX connections */
static const struct snd_soc_dapm_route audio_map_rx[] = {
	{"Speaker-L/R", NULL, "SPK_LP"},
	{"Speaker-L/R", NULL, "SPK_LN"},
	{"Speaker-L/R", NULL, "SPK_RP"},
	{"Speaker-L/R", NULL, "SPK_RN"},
	{"HP-L/R", NULL, "HP_L"},
	{"HP-L/R", NULL, "HP_R"},
};

static int mango210_wm8960_init_paiftx(struct snd_soc_codec *codec)
{
	int ret;

	/* Add mango210 specific Capture widgets */
	snd_soc_dapm_new_controls(codec, wm8960_dapm_widgets_cpt,
				  ARRAY_SIZE(wm8960_dapm_widgets_cpt));

	/* Set up PAIFTX audio path */
	snd_soc_dapm_add_routes(codec, audio_map_tx, ARRAY_SIZE(audio_map_tx));

	/* Enabling the microphone requires the fitting of a 0R
	 * resistor to connect the line from the microphone jack.
	 */
//	snd_soc_dapm_disable_pin(codec, "MicIn");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(&wm8960_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;
	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(&s3c64xx_i2s_dai[I2S_NUM], SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set WM8960 to drive MCLK from its MCLK-pin */
	ret = snd_soc_dai_set_clkdiv(&wm8960_dai, WM8960_SYSCLKSEL,
					WM8960_SYSCLK_MCLK);
	if (ret < 0)
		return ret;

	return 0;
}

static int mango210_wm8960_init_paifrx(struct snd_soc_codec *codec)
{
	int ret;

	/* Add mango210 specific Playback widgets */
	snd_soc_dapm_new_controls(codec, wm8960_dapm_widgets_pbk,
				  ARRAY_SIZE(wm8960_dapm_widgets_pbk));

	/* add iDMA controls */
//	ret = snd_soc_add_controls(codec, &s5p_idma_control, 1);
//	if (ret < 0)
//		return ret;

	/* Set up PAIFRX audio path */
	snd_soc_dapm_add_routes(codec, audio_map_rx, ARRAY_SIZE(audio_map_rx));

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(&wm8960_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(&s3c64xx_i2s_dai[I2S_NUM], SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set WM8960 to drive MCLK from its MCLK-pin */
	ret = snd_soc_dai_set_clkdiv(&wm8960_dai, WM8960_SYSCLKSEL,
					WM8960_SYSCLK_MCLK);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_dai_link mango210_dai[] = {
{ /* Primary Playback i/f */
	.name = "WM8960 PAIF RX",
	.stream_name = "Playback",
	.cpu_dai = &s3c64xx_i2s_dai[I2S_NUM],
	.codec_dai = &wm8960_dai,
	.init = mango210_wm8960_init_paifrx,
	.ops = &mango210_ops,
},
{ /* Primary Capture i/f */
	.name = "WM8960 PAIF TX",
	.stream_name = "Capture",
	.cpu_dai = &s3c64xx_i2s_dai[I2S_NUM],
	.codec_dai = &wm8960_dai,
	.init = mango210_wm8960_init_paiftx,
	.ops = &mango210_ops,
},
};

static struct snd_soc_card mango210 = {
	.name = "mango",
	.platform = &s3c_dma_wrapper,
	.dai_link = mango210_dai,
	.num_links = ARRAY_SIZE(mango210_dai),
};

static struct snd_soc_device mango210_snd_devdata = {
	.card = &mango210,
	.codec_dev = &soc_codec_dev_wm8960,
};

static struct platform_device *mango210_snd_device;
static int __init mango210_audio_init(void)
{
	int ret;

	mango210_snd_device = platform_device_alloc("soc-audio", 0);
	if (!mango210_snd_device)
		return -ENOMEM;

	platform_set_drvdata(mango210_snd_device, &mango210_snd_devdata);
	mango210_snd_devdata.dev = &mango210_snd_device->dev;
	ret = platform_device_add(mango210_snd_device);

	if (ret)
		platform_device_put(mango210_snd_device);

	return ret;
}

static void __exit mango210_audio_exit(void)
{
	debug_msg("%s\n", __func__);

	platform_device_unregister(mango210_snd_device);
}
module_init(mango210_audio_init);
module_exit(mango210_audio_exit);

MODULE_AUTHOR("Pyeongjeong Lee, leepjung@crz-tech.com");
MODULE_DESCRIPTION("ALSA SoC MANGO210 WM8960");
MODULE_LICENSE("GPL");
