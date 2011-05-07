/*
 * Driver for NOON130PC20 (UXGA camera) from SILICON-FILE
 * 
 * 1.3Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 2009, Pyeongjeong Lee <leepjung@naver.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#include <media/noon130pc20_platform.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include "noon130pc20.h"

#define NOON130PC20_DRIVER_NAME	"NOON130PC20"

#define VGA_CAM_DEBUG

#ifdef VGA_CAM_DEBUG
#define dev_dbg	dev_err
#endif

/* Default resolution & pixelformat. plz ref noon130pc20_platform.h */
#define DEFAULT_RES		VGA	/* Index of resoultion */
#define DEFAUT_FPS_INDEX	NOON130PC20_15FPS
#define DEFUALT_MCLK            24000000		/* 24MHz default */
#define DEFAULT_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */
#define POLL_TIME_MS		10

/*
 * Specification
 * Parallel : ITU-R. 656/601 YUV422, RGB565, RGB888 (Up to VGA), RAW10 
 * Serial : MIPI CSI2 (single lane) YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Resolution : 1280 (H) x 1024 (V)
 * Image control : Brightness, Contrast, Saturation, Sharpness, Glamour
 * Effect : Mono, Negative, Sepia, Aqua, Sketch
 * FPS : 15fps @full resolution, 30fps @VGA, 24fps @720p
 * Max. pixel clock frequency : 48MHz(upto)
 * Internal PLL (6MHz to 27MHz input frequency)
 */

static int noon130pc20_init(struct v4l2_subdev *sd, u32 val);
/* Camera functional setting values configured by user concept */
struct noon130pc20_userset {
	int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;	/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;	/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;	/* Color FX (AKA Color tone) */
	unsigned int contrast;	/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct noon130pc20_state {
	struct noon130pc20_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct noon130pc20_userset userset;
	struct v4l2_streamparm strm;
	int framesize_index;
	int freq;	/* MCLK in KHz */
	int is_mipi;
	int isize;
	int ver;
	int fps;
	int vt_mode;		/*For VT camera */
	int check_dataline;
	int check_previewdata;
};

enum {
	NOON130PC20_PREVIEW_VGA,
    //NOON130PC20_PREVIEW_SVGA,
};

struct noon130pc20_enum_framesize {
	unsigned int index;
	unsigned int width;
	unsigned int height;
};

struct noon130pc20_enum_framesize noon130pc20_framesize_list[] = {
	{ NOON130PC20_PREVIEW_VGA, 640, 480 }
    //{ NOON130PC20_PREVIEW_SVGA, 800, 600 }
};
static inline struct noon130pc20_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct noon130pc20_state, sd);
}

struct noon130pc20_regset_table {
	struct noon130pc20_reg	*regset;
	int			array_size;
};

#define NOON130PC20_REGSET_TABLE_ELEMENT(x, y)		\
	[(x)] = {					\
		.regset		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
	}

static struct noon130pc20_regset_table brightness_vt[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_4, noon130pc20_ev_vt_m4),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_3, noon130pc20_ev_vt_m3),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_2, noon130pc20_ev_vt_m2),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_1, noon130pc20_ev_vt_m1),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_DEFAULT, noon130pc20_ev_vt_default),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_1, noon130pc20_ev_vt_p1),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_2, noon130pc20_ev_vt_p2),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_3, noon130pc20_ev_vt_p3),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_4, noon130pc20_ev_vt_p4),
};

static struct noon130pc20_regset_table brightness[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_4, noon130pc20_ev_m4),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_3, noon130pc20_ev_m3),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_2, noon130pc20_ev_m2),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_MINUS_1, noon130pc20_ev_m1),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_DEFAULT, noon130pc20_ev_default),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_1, noon130pc20_ev_p1),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_2, noon130pc20_ev_p2),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_3, noon130pc20_ev_p3),
	NOON130PC20_REGSET_TABLE_ELEMENT(EV_PLUS_4, noon130pc20_ev_p4),
};

static struct noon130pc20_regset_table white_balance[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(WHITE_BALANCE_AUTO,
						noon130pc20_wb_auto),
	NOON130PC20_REGSET_TABLE_ELEMENT(WHITE_BALANCE_SUNNY,
						noon130pc20_wb_sunny),
	NOON130PC20_REGSET_TABLE_ELEMENT(WHITE_BALANCE_CLOUDY,
						noon130pc20_wb_cloudy),
	NOON130PC20_REGSET_TABLE_ELEMENT(WHITE_BALANCE_TUNGSTEN,
						noon130pc20_wb_tungsten),
	NOON130PC20_REGSET_TABLE_ELEMENT(WHITE_BALANCE_FLUORESCENT,
						noon130pc20_wb_fluorescent),
};

static struct noon130pc20_regset_table effects[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(IMAGE_EFFECT_NONE,
						noon130pc20_effect_none),
	NOON130PC20_REGSET_TABLE_ELEMENT(IMAGE_EFFECT_BNW,
						noon130pc20_effect_gray),
	NOON130PC20_REGSET_TABLE_ELEMENT(IMAGE_EFFECT_SEPIA,
						noon130pc20_effect_sepia),
	NOON130PC20_REGSET_TABLE_ELEMENT(IMAGE_EFFECT_NEGATIVE,
						noon130pc20_effect_negative),
};

static struct noon130pc20_regset_table fps_vt_table[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(0, noon130pc20_vt_fps_7),
	NOON130PC20_REGSET_TABLE_ELEMENT(1, noon130pc20_vt_fps_10),
	NOON130PC20_REGSET_TABLE_ELEMENT(2, noon130pc20_vt_fps_15),
};

static struct noon130pc20_regset_table fps_table[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(0, noon130pc20_fps_7),
	NOON130PC20_REGSET_TABLE_ELEMENT(1, noon130pc20_fps_10),
	NOON130PC20_REGSET_TABLE_ELEMENT(2, noon130pc20_fps_15),
};

static struct noon130pc20_regset_table blur_vt[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_0, noon130pc20_blur_vt_none),
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_1, noon130pc20_blur_vt_p1),
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_2, noon130pc20_blur_vt_p2),
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_3, noon130pc20_blur_vt_p3),
};

static struct noon130pc20_regset_table blur[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_0, noon130pc20_blur_none),
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_1, noon130pc20_blur_p1),
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_2, noon130pc20_blur_p2),
	NOON130PC20_REGSET_TABLE_ELEMENT(BLUR_LEVEL_3, noon130pc20_blur_p3),
};

static struct noon130pc20_regset_table dataline[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(0, noon130pc20_dataline),
};

static struct noon130pc20_regset_table dataline_stop[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(0, noon130pc20_dataline_stop),
};

static struct noon130pc20_regset_table init_reg[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(0, noon130pc20_init_reg),
};

static struct noon130pc20_regset_table init_vt_reg[] = {
	NOON130PC20_REGSET_TABLE_ELEMENT(0, noon130pc20_init_vt_reg),
};

static int noon130pc20_reset(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_platform_data *pdata;

	pdata = client->dev.platform_data;

	if (pdata->cam_power) {
		pdata->cam_power(0);
		msleep(5);
		pdata->cam_power(1);
		msleep(5);
		noon130pc20_init(sd, 0);
	}

	return 0;
}

static int noon130pc20_i2c_write_multi(struct i2c_client *client,
				    unsigned short addr, unsigned int w_data)
{
	int retry_count = 5;
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };
	int ret;

	buf[0] = addr;
	buf[1] = w_data;
#if 0
#ifdef VGA_CAM_DEBUG
	int i;
	for (i = 0; i < 2; i++) {
		dev_err(&client->dev, "buf[%d] = %x  ", i, buf[i]);
		if (i == 1)
			dev_err(&client->dev, "\n");
	}
#endif
#endif
	while (retry_count--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;
		msleep(POLL_TIME_MS);
	}
	if (ret != 1)
		dev_err(&client->dev, "I2C is not working.\n");

	return (ret == 1) ? 0 : -EIO;
}

static int noon130pc20_write_regs(struct v4l2_subdev *sd,
			       struct noon130pc20_reg regs[], int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, err;

	for (i = 0; i < size; i++) {
		err = noon130pc20_i2c_write_multi(client,
					       regs[i].addr, regs[i].val);
		if (err < 0) {
			v4l_info(client, "%s: register set failed\n", __func__);
			break;
		}
	}
	if (err < 0)
		return -EIO;

	return 0;
}

static int noon130pc20_write_regset_table(struct v4l2_subdev *sd,
				struct noon130pc20_regset_table *regset_table)
{
	int err;

	if (regset_table->regset)
		err = noon130pc20_write_regs(sd, regset_table->regset,
						regset_table->array_size);
	else
		err = -EINVAL;

	return err;
}

static int noon130pc20_set_from_table(struct v4l2_subdev *sd,
				const char *setting_name,
				const struct noon130pc20_regset_table *table,
				int table_size, int index)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "%s: set %s index %d\n",
		__func__, setting_name, index);

	if ((index < 0) || (index >= table_size)) {
		dev_err(&client->dev,
			"%s: index(%d) out of range[0:%d] for table for %s\n",
			__func__, index, table_size, setting_name);
		return -EINVAL;
	}
	table += index;
	if (table == NULL)
		return -EINVAL;
	return noon130pc20_write_regset_table(sd, table);
}

static int noon130pc20_set_parameter(struct v4l2_subdev *sd,
				int *current_value_ptr,
				int new_value,
				const char *setting_name,
				const struct noon130pc20_regset_table *table,
				int table_size)
{
	int err;

	if (*current_value_ptr == new_value)
		return 0;

	err = noon130pc20_set_from_table(sd, setting_name, table,
				table_size, new_value);

	if (!err)
		*current_value_ptr = new_value;

	return err;
}
static struct v4l2_queryctrl noon130pc20_controls[] = {
	/* Add here if needed */
};

const char **noon130pc20_ctrl_get_menu(u32 id)
{
	pr_debug("%s is called... id : %d\n", __func__, id);

	switch (id) {
	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *noon130pc20_find_qctrl(int id)
{
	int i;

	pr_debug("%s is called... id : %d\n", __func__, id);

	for (i = 0; i < ARRAY_SIZE(noon130pc20_controls); i++)
		if (noon130pc20_controls[i].id == id)
			return &noon130pc20_controls[i];

	return NULL;
}

static int noon130pc20_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	pr_debug("%s is called...\n", __func__);

	for (i = 0; i < ARRAY_SIZE(noon130pc20_controls); i++) {
		if (noon130pc20_controls[i].id == qc->id) {
			memcpy(qc, &noon130pc20_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int noon130pc20_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	pr_debug("%s is called...\n", __func__);

	qctrl.id = qm->id;
	noon130pc20_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, noon130pc20_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 * 	freq : in Hz
 * 	flag : not supported for now
 */
static int noon130pc20_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	pr_debug("%s is called...\n", __func__);

	return err;
}

static int noon130pc20_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	pr_debug("%s is called...\n", __func__);

	return err;
}

static int noon130pc20_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	pr_debug("%s is called...\n", __func__);

	return err;
}
static int noon130pc20_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	int num_entries = sizeof(noon130pc20_framesize_list) /
				sizeof(struct noon130pc20_enum_framesize);
	struct noon130pc20_enum_framesize *elem;
	int index = 0;
	int i = 0;

	pr_debug("%s is called...\n", __func__);

	/* The camera interface should read this value, this is the resolution
	 * at which the sensor would provide framedata to the camera i/f
	 *
	 * In case of image capture,
	 * this returns the default camera resolution (WVGA)
	 */
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	index = state->framesize_index;

	for (i = 0; i < num_entries; i++) {
		elem = &noon130pc20_framesize_list[i];
		if (elem->index == index) {
			fsize->discrete.width =
			    noon130pc20_framesize_list[index].width;
			fsize->discrete.height =
			    noon130pc20_framesize_list[index].height;
			return 0;
		}
	}

	return -EINVAL;
}

static int noon130pc20_enum_frameintervals(struct v4l2_subdev *sd,
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	pr_debug("%s is called...\n", __func__);

	return err;
}

static int noon130pc20_enum_fmt(struct v4l2_subdev *sd,
			     struct v4l2_fmtdesc *fmtdesc)
{
	int err = 0;

	pr_debug("%s is called...\n", __func__);

	return err;
}

static int noon130pc20_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	pr_debug("%s is called...\n", __func__);

	return err;
}

static int noon130pc20_g_parm(struct v4l2_subdev *sd,
			   struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	return err;
}

static int noon130pc20_s_parm(struct v4l2_subdev *sd,
			   struct v4l2_streamparm *param)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct sec_cam_parm *new_parms =
		(struct sec_cam_parm *)&param->parm.raw_data;
	struct sec_cam_parm *parms =
		(struct sec_cam_parm *)&state->strm.parm.raw_data;

	dev_dbg(&client->dev, "%s: start\n", __func__);

	/* we return an error if one happened but don't stop trying to
	 * set all parameters passed
	 */

	err = noon130pc20_set_parameter(sd, &parms->brightness,
				new_parms->brightness, "brightness",
				brightness, ARRAY_SIZE(brightness));
	err |= noon130pc20_set_parameter(sd, &parms->effects, new_parms->effects,
				"effects", effects,
				ARRAY_SIZE(effects));
	err |= noon130pc20_set_parameter(sd, &parms->white_balance,
				new_parms->white_balance,
				"white_balance", white_balance,
				ARRAY_SIZE(white_balance));

	dev_dbg(&client->dev, "%s: returning %d\n", __func__, err);

	return err;
}

/* set sensor register values for adjusting brightness */
static int noon130pc20_set_brightness(struct v4l2_subdev *sd,
				   struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct noon130pc20_regset_table *ev;
	int err = -EINVAL;
	int ev_index;
	int array_size;

	dev_dbg(&client->dev, "%s: value : %d state->vt_mode %d\n",
			__func__, ctrl->value, state->vt_mode);

	pr_debug("state->vt_mode : %d\n", state->vt_mode);

	if ((ctrl->value < 0) || (ctrl->value >= EV_MAX))
		ev_index = EV_DEFAULT;
	else
		ev_index = ctrl->value;

	if (state->vt_mode == 1) {
		ev = brightness_vt;
		array_size = ARRAY_SIZE(brightness_vt);
	} else {
		ev = brightness;
		array_size = ARRAY_SIZE(brightness);
	}

	if (ev_index >= array_size)
		ev_index = EV_DEFAULT;

	ev += ev_index;

	err = noon130pc20_write_regset_table(sd, ev);

	if (err)
		dev_dbg(&client->dev, "%s: register set failed\n", __func__);

	return err;
}

/*
 * set sensor register values for
 * adjusting whitebalance, both auto and manual
 */
static int noon130pc20_set_wb(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_regset_table *wb = white_balance;
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d\n", __func__, ctrl->value);

	if ((ctrl->value < WHITE_BALANCE_BASE) ||
		(ctrl->value > WHITE_BALANCE_MAX) ||
		(ctrl->value >= ARRAY_SIZE(white_balance))) {
		dev_dbg(&client->dev, "%s: Value(%d) out of range([%d:%d])\n",
			__func__, ctrl->value,
			WHITE_BALANCE_BASE, WHITE_BALANCE_MAX);
		dev_dbg(&client->dev, "%s: Value out of range\n", __func__);
		goto out;
	}

	wb += ctrl->value;

	err = noon130pc20_write_regset_table(sd, wb);

	if (err)
		dev_dbg(&client->dev, "%s: register set failed\n", __func__);
out:
	return err;
}

/* set sensor register values for adjusting color effect */
static int noon130pc20_set_effect(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_regset_table *effect = effects;
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d\n", __func__, ctrl->value);

	if ((ctrl->value < IMAGE_EFFECT_BASE) ||
		(ctrl->value > IMAGE_EFFECT_MAX) ||
		(ctrl->value >= ARRAY_SIZE(effects))) {
		dev_dbg(&client->dev, "%s: Value(%d) out of range([%d:%d])\n",
			__func__, ctrl->value,
			IMAGE_EFFECT_BASE, IMAGE_EFFECT_MAX);
		goto out;
	}

	effect += ctrl->value;

	err = noon130pc20_write_regset_table(sd, effect);

	if (err)
		dev_dbg(&client->dev, "%s: register set failed\n", __func__);
out:
	return err;
}

/* set sensor register values for frame rate(fps) setting */
static int noon130pc20_set_frame_rate(struct v4l2_subdev *sd,
				   struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct noon130pc20_regset_table *fps;

	int err = -EINVAL;
	int fps_index;

	dev_dbg(&client->dev, "%s: value : %d\n", __func__, ctrl->value);

	pr_debug("state->vt_mode : %d\n", state->vt_mode);

	switch (ctrl->value) {
	case 7:
		fps_index = 0;
		break;

	case 10:
		fps_index = 1;
		break;

	case 15:
		fps_index = 2;
		break;

	default:
		dev_dbg(&client->dev, "%s: Value(%d) is not supported\n",
			__func__, ctrl->value);
		goto out;
	}

	if (state->vt_mode == 1)
		fps = fps_vt_table;
	else
		fps = fps_table;

	fps += fps_index;

	err = noon130pc20_write_regset_table(sd, fps);
out:
	return err;
}

/* set sensor register values for adjusting blur effect */
static int noon130pc20_set_blur(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct noon130pc20_regset_table *bl;
	int err = -EINVAL;
	int array_size;

	dev_dbg(&client->dev, "%s: value : %d\n", __func__, ctrl->value);

	pr_debug("state->vt_mode : %d\n", state->vt_mode);

	if ((ctrl->value < BLUR_LEVEL_0) || (ctrl->value > BLUR_LEVEL_MAX)) {
		dev_dbg(&client->dev, "%s: Value(%d) out of range([%d:%d])\n",
			__func__, ctrl->value,
			BLUR_LEVEL_0, BLUR_LEVEL_MAX);
		goto out;
	}

	if (state->vt_mode == 1) {
		bl = blur_vt;
		array_size = ARRAY_SIZE(blur_vt);
	} else {
		bl = blur;
		array_size = ARRAY_SIZE(blur);
	}

	if (ctrl->value >= array_size) {
		dev_dbg(&client->dev, "%s: Value(%d) out of range([%d:%d))\n",
			__func__, ctrl->value,
			BLUR_LEVEL_0, array_size);
		goto out;
	}

	bl += ctrl->value;

	err = noon130pc20_write_regset_table(sd, bl);
out:
	return err;
}

static int noon130pc20_check_dataline_stop(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s\n", __func__);
#if 0
	err = s5ka3dfx_write_regset_table(sd, dataline_stop);
	if (err < 0) {
		v4l_info(client, "%s: register set failed\n", __func__);
		return -EIO;
	}

	state->check_dataline = 0;
	err = noon130pc20_reset(sd);
	if (err < 0) {
		v4l_info(client, "%s: register set failed\n", __func__);
		return -EIO;
	}
	return err;
#else
	return 0;
#endif
}

/* returns the real iso currently used by sensor due to lighting
 * conditions, not the requested iso we sent using s_ctrl.
 */
static int noon130pc20_get_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	s32 read_value;
	int gain;
#if 0
	err = s5ka3dfx_i2c_write_multi(client, 0xEF, 0x02);
	if (err < 0)
		return err;

	read_value = i2c_smbus_read_byte_data(client, 0x1D);
	if (read_value < 0)
		return read_value;

	read_value &= 0x7F;
	gain = (128 * 100) / (128 - read_value);

	if (gain > 280)
		ctrl->value = ISO_400;
	else if (gain > 230)
		ctrl->value = ISO_200;
	else if (gain > 190)
		ctrl->value = ISO_100;
	else if (gain > 100)
		ctrl->value = ISO_50;
	else
		ctrl->value = gain;
#endif
	dev_dbg(&client->dev, "%s: get iso == %d (0x%x)\n",
			__func__, ctrl->value, read_value);

	return err;
}

static int noon130pc20_get_shutterspeed(struct v4l2_subdev *sd,
		struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state = to_state(sd);
	s32 read_value;
	int cintr;
	int err;
#if 0
	err = noon130pc20_i2c_write_multi(client, 0xEF, 0x02);
	if (err < 0)
		return err;

	read_value = i2c_smbus_read_byte_data(client, 0x0E);
	if (read_value < 0)
		return read_value;
	cintr = (read_value & 0x1F) << 8;

	read_value = i2c_smbus_read_byte_data(client, 0x0F);
	if (read_value < 0)
		return read_value;
	cintr |= read_value & 0xFF;

	/* A3D Shutter Speed (Sec.) = MCLK / (2 * (cintr - 1) * 814) */
	ctrl->value =  ((cintr - 1) * 1628) / (state->freq / 1000);
#endif
	dev_dbg(&client->dev,
			"%s: get shutterspeed == %d\n", __func__, ctrl->value);

	return err;
}

static int noon130pc20_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct noon130pc20_userset userset = state->userset;
	int err = 0;

	dev_dbg(&client->dev, "%s: id : 0x%08x\n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ctrl->value = userset.exposure_bias;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = userset.auto_wb;
		break;

	case V4L2_CID_WHITE_BALANCE_PRESET:
		ctrl->value = userset.manual_wb;
		break;

	case V4L2_CID_COLORFX:
		ctrl->value = userset.effect;
		break;

	case V4L2_CID_CONTRAST:
		ctrl->value = userset.contrast;
		break;

	case V4L2_CID_SATURATION:
		ctrl->value = userset.saturation;
		break;

	case V4L2_CID_SHARPNESS:
		ctrl->value = userset.saturation;
		break;

	case V4L2_CID_CAMERA_GET_ISO:
		err = noon130pc20_get_iso(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_GET_SHT_TIME:
		err = noon130pc20_get_shutterspeed(sd, ctrl);
		break;

	case V4L2_CID_ESD_INT:
		ctrl->value = 0;
		break;

	default:
		dev_dbg(&client->dev, "%s: no such ctrl\n", __func__);
		err = -EINVAL;
		break;
	}
	
	return err;
}

static int noon130pc20_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);

	int err = -EINVAL;

	dev_dbg(&client->dev, "%s : ctrl->id 0x%08x, ctrl->value %d\n",
			__func__, ctrl->id, ctrl->value);

	switch (ctrl->id) {

	case V4L2_CID_CAMERA_BRIGHTNESS:
		dev_dbg(&client->dev, "%s: "
				"V4L2_CID_CAMERA_BRIGHTNESS\n", __func__);
		err = noon130pc20_set_brightness(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:
		dev_dbg(&client->dev, "%s: "
				"V4L2_CID_AUTO_WHITE_BALANCE\n", __func__);
		err = noon130pc20_set_wb(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_EFFECT:
		dev_dbg(&client->dev, "%s: "
				"V4L2_CID_CAMERA_EFFECT\n", __func__);
		err = noon130pc20_set_effect(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_FRAME_RATE:
		dev_dbg(&client->dev, "%s: "
				"V4L2_CID_CAMERA_FRAME_RATE\n", __func__);
		err = noon130pc20_set_frame_rate(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_VGA_BLUR:
		dev_dbg(&client->dev, "%s: "
				"V4L2_CID_CAMERA_VGA_BLUR\n", __func__);
		err = noon130pc20_set_blur(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_VT_MODE:
		state->vt_mode = ctrl->value;
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_VT_MODE : "
				"state->vt_mode %d\n",
				__func__, state->vt_mode);
		err = 0;
		break;

	case V4L2_CID_CAMERA_CHECK_DATALINE:
		state->check_dataline = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_CHECK_DATALINE_STOP:
		err = noon130pc20_check_dataline_stop(sd);
		break;

	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if (state->check_previewdata == 0)
			err = 0;
		else
			err = -EIO;
		break;

	case V4L2_CID_CAMERA_RESET:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_RESET\n", __func__);
		err = noon130pc20_reset(sd);
		break;	

	default:
		dev_dbg(&client->dev, "%s: no support control "
				"in camera sensor, NOON130PC20\n", __func__);
		err = 0;
		break;
	}

	if (err < 0)
		goto out;
	else
		return 0;

out:
	dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);
	return err;
}

static void noon130pc20_init_parameters(struct v4l2_subdev *sd)
{
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct sec_cam_parm *parms =
		(struct sec_cam_parm *)&state->strm.parm.raw_data;

	parms->effects = IMAGE_EFFECT_NONE;
	parms->brightness = EV_DEFAULT;
	parms->white_balance = WHITE_BALANCE_AUTO;

}

static int noon130pc20_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	int err = -EINVAL, i;

	pr_debug("camera initialization start, state->vt_mode : %d\n",
			state->vt_mode);
	pr_debug("state->check_dataline : %d\n", state->check_dataline);

	noon130pc20_init_parameters(sd);
//	if (state->vt_mode == 0) {
//		if (state->check_dataline)
//			err = noon130pc20_write_regset_table(sd, dataline);
//		else
			err = noon130pc20_write_regset_table(sd, init_reg);
//	} else
//		err = noon130pc20_write_regset_table(sd, init_vt_reg);

	if (err < 0) {
		/* This is preview fail */
		state->check_previewdata = 100;
		v4l_err(client,
			"%s: camera initialization failed. err(%d)\n",
			__func__, state->check_previewdata);
		return -EIO;
	}

	/* This is preview success */
	state->check_previewdata = 0;
	return 0;
}

/*
 * s_config subdev ops
 * With camera device,
 * we need to re-initialize every single opening time therefor,
 * it is not necessary to be initialized on probe time.
 * except for version checking
 * NOTE: version checking is optional
 */
static int noon130pc20_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);
	struct noon130pc20_platform_data *pdata;

	dev_dbg(&client->dev, "fetching platform data\n");

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (!(pdata->default_width && pdata->default_height)) {
		/* TODO: assign driver default resolution */
	} else {
		state->pix.width = pdata->default_width;
		state->pix.height = pdata->default_height;
	}

	if (!pdata->pixelformat)
		state->pix.pixelformat = DEFAULT_FMT;
	else
		state->pix.pixelformat = pdata->pixelformat;

	if (pdata->freq == 0)
		state->freq = DEFUALT_MCLK;
	else
		state->freq = pdata->freq;

	if (!pdata->is_mipi) {
		state->is_mipi = 0;
		dev_dbg(&client->dev, "parallel mode\n");
	} else
		state->is_mipi = pdata->is_mipi;

	return 0;
}

static const struct v4l2_subdev_core_ops noon130pc20_core_ops = {
	.init = noon130pc20_init,	/* initializing API */
	.s_config = noon130pc20_s_config,	/* Fetch platform data */
	.queryctrl = noon130pc20_queryctrl,
	.querymenu = noon130pc20_querymenu,
	.g_ctrl = noon130pc20_g_ctrl,
	.s_ctrl = noon130pc20_s_ctrl,
};

static const struct v4l2_subdev_video_ops noon130pc20_video_ops = {
	.s_crystal_freq = noon130pc20_s_crystal_freq,
	.g_fmt = noon130pc20_g_fmt,
	.s_fmt = noon130pc20_s_fmt,
	.enum_framesizes = noon130pc20_enum_framesizes,
	.enum_frameintervals = noon130pc20_enum_frameintervals,
	.enum_fmt = noon130pc20_enum_fmt,
	.try_fmt = noon130pc20_try_fmt,
	.g_parm = noon130pc20_g_parm,
	.s_parm = noon130pc20_s_parm,
};

static const struct v4l2_subdev_ops noon130pc20_ops = {
	.core = &noon130pc20_core_ops,
	.video = &noon130pc20_video_ops,
};

/*
 * noon130pc20_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int noon130pc20_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct noon130pc20_state *state;
	struct v4l2_subdev *sd;

	state = kzalloc(sizeof(struct noon130pc20_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, NOON130PC20_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &noon130pc20_ops);

	dev_dbg(&client->dev, "noon130pc20 has been probed\n");
	return 0;
}


static int noon130pc20_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct noon130pc20_state *state =
		container_of(sd, struct noon130pc20_state, sd);

	v4l2_device_unregister_subdev(sd);
	kfree(state);
	return 0;
}

static const struct i2c_device_id noon130pc20_id[] = {
	{ NOON130PC20_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, noon130pc20_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = NOON130PC20_DRIVER_NAME,
	.probe = noon130pc20_probe,
	.remove = noon130pc20_remove,
	.id_table = noon130pc20_id,
};

MODULE_DESCRIPTION("SILICON FILE NOON130PC20 SXGA camera driver");
MODULE_AUTHOR("Pyeongjeong Lee <leepjung@naver.com>");
MODULE_LICENSE("GPL");

