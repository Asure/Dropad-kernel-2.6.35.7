/* linux/drivers/media/video/samsung/tv20/s5p_tv_base.c
 *
 * Entry file for Samsung TVOut driver
 *
 * Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <mach/map.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>

#include <linux/earlysuspend.h>
#if CONFIG_CPU_FREQ_S5PV210
#include <mach/cpu-freq-v210.h>
#endif /* CONFIG_CPU_FREQ_S5PV210 */

#ifdef CONFIG_S5PV210_PM
#include <mach/pd.h>
#endif

#include "s5p_tv.h"

#ifdef COFIG_TVOUT_DBG
#define S5P_TV_BASE_DEBUG 1
#endif

#ifdef S5P_TV_BASE_DEBUG
#define BASEPRINTK(fmt, args...) \
	printk(KERN_INFO "[TVBASE] %s: " fmt, __func__ , ## args)
#else
#define BASEPRINTK(fmt, args...)
#endif

#define TVOUT_CLK_INIT(dev, clk, name)

#define TVOUT_IRQ_INIT(x, ret, dev, num, jump, ftn, m_name) 		\
	do {								\
		x = platform_get_irq(dev, num); 			\
		if (x < 0) {						\
			printk(KERN_ERR					\
			"failed to get %s irq resource\n", m_name);	\
			ret = -ENOENT; 					\
			goto jump;					\
		}							\
		ret = request_irq(x, ftn, IRQF_DISABLED,		\
			dev->name, dev);				\
		if (ret != 0) {						\
			printk(KERN_ERR					\
			"failed to install %s irq (%d)\n", m_name, ret);\
			goto jump;					\
		}							\
	} while (0);


static struct mutex	*mutex_for_fo;


struct s5p_tv_status 	s5ptv_status;
struct s5p_tv_vo 	s5ptv_overlay[2];
bool			clks_enabled;

#ifdef CONFIG_CPU_S5PV210
int tv_phy_power(bool on)
{
	if (on) {
            __s5p_tv_poweron();
		/* on */
		clk_enable(s5ptv_status.i2c_phy_clk);
		__s5p_hdmi_phy_power(true);

	} else {
		/*
		 * for preventing hdmi hang up when restart
		 * switch to internal clk - SCLK_DAC, SCLK_PIXEL
		 */
		clk_set_parent(s5ptv_status.sclk_mixer,
					s5ptv_status.sclk_dac);
		clk_set_parent(s5ptv_status.sclk_hdmi,
					s5ptv_status.sclk_pixel);

		__s5p_hdmi_phy_power(false);
		clk_disable(s5ptv_status.i2c_phy_clk);
		 __s5p_tv_poweroff();
	}

	return 0;
}


int s5p_tv_clk_gate(bool on)
{
	BASEPRINTK("tv clks are: %d\n", on);
	if(clks_enabled == on)
		return 0;

	if (on) {

		clk_enable(s5ptv_status.vp_clk);

		clk_enable(s5ptv_status.mixer_clk);

		clk_enable(s5ptv_status.tvenc_clk);

		clk_enable(s5ptv_status.hdmi_clk);
		clks_enabled = true;

	} else {

		/* off */
		clk_disable(s5ptv_status.vp_clk);

		clk_disable(s5ptv_status.mixer_clk);

		clk_disable(s5ptv_status.tvenc_clk);

		clk_disable(s5ptv_status.hdmi_clk);

		clks_enabled = false;
	}

	return 0;
}
EXPORT_SYMBOL(s5p_tv_clk_gate);

#define TV_CLK_GET_WITH_ERR_CHECK(clk, pdev, clk_name)			\
	do {							        \
		clk = clk_get(&pdev->dev, clk_name);			\
		if (IS_ERR(clk)) {					\
			printk(KERN_ERR					\
			"failed to find clock \"%s\"\n", clk_name); 	\
			return ENOENT;					\
		}							\
	} while (0);

static int __devinit tv_clk_get(struct platform_device *pdev,
	struct s5p_tv_status *ctrl)
{
	struct clk	*ext_xtal_clk,
			*mout_vpll_src,
			*fout_vpll,
			*mout_vpll;

	TV_CLK_GET_WITH_ERR_CHECK(ctrl->tvenc_clk,	pdev, "tvenc");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->vp_clk,		pdev, "vp");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->mixer_clk,	pdev, "mixer");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->hdmi_clk,	pdev, "hdmi");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->i2c_phy_clk,	pdev, "i2c-hdmiphy");

	TV_CLK_GET_WITH_ERR_CHECK(ctrl->sclk_dac,	pdev, "sclk_dac");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->sclk_mixer,	pdev, "sclk_mixer");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->sclk_hdmi,	pdev, "sclk_hdmi");

	TV_CLK_GET_WITH_ERR_CHECK(ctrl->sclk_pixel,	pdev, "sclk_pixel");
	TV_CLK_GET_WITH_ERR_CHECK(ctrl->sclk_hdmiphy,	pdev, "sclk_hdmiphy");

	TV_CLK_GET_WITH_ERR_CHECK(ext_xtal_clk,		pdev, "ext_xtal");
	TV_CLK_GET_WITH_ERR_CHECK(mout_vpll_src,	pdev, "vpll_src");
	TV_CLK_GET_WITH_ERR_CHECK(fout_vpll,		pdev, "fout_vpll");
	TV_CLK_GET_WITH_ERR_CHECK(mout_vpll,		pdev, "sclk_vpll");

	clk_set_parent(mout_vpll_src, ext_xtal_clk);
	clk_set_parent(mout_vpll, fout_vpll);

	/* sclk_dac's parent is fixed as mout_vpll */
	clk_set_parent(ctrl->sclk_dac, mout_vpll);

	/* clk_set_rate(fout_vpll, 54000000); */
	clk_set_rate(ctrl->sclk_pixel, 54000000);

	clk_enable(ctrl->sclk_dac);
	clk_enable(ctrl->sclk_mixer);
	clk_enable(ctrl->sclk_hdmi);

	clk_enable(mout_vpll_src);
	clk_enable(fout_vpll);
	clk_enable(mout_vpll);

	clk_put(ext_xtal_clk);
	clk_put(mout_vpll_src);
	clk_put(fout_vpll);
	clk_put(mout_vpll);

	return 0;
}
#else
#define s5p_tv_clk_gate NULL
#define tv_phy_power NULL
#define tv_clk_get NULL
#endif

/*
 * ftn for irq
 */
static irqreturn_t s5p_tvenc_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

#ifdef CONFIG_TV_FB
static int s5p_tv_open(struct file *file)
{
	return 0;
}

static int s5p_tv_release(struct file *file)
{
	s5ptv_status.hdcp_en = false;
	return 0;
}

static int s5p_tv_vid_open(struct file *file)
{
	int ret = 0;

	mutex_lock(mutex_for_fo);

	if (s5ptv_status.vp_layer_enable) {
		printk(KERN_ERR "video layer. already used !!\n");
		ret =  -EBUSY;
	}

	mutex_unlock(mutex_for_fo);
	return ret;
}

static int s5p_tv_vid_release(struct file *file)
{
	s5ptv_status.vp_layer_enable = false;

	_s5p_vlayer_stop();

	return 0;
}
#else

/*
 * ftn for video
 */
static int s5p_tv_v_open(struct file *file)
{
	int ret = 0;

	mutex_lock(mutex_for_fo);

	if (s5ptv_status.tvout_output_enable) {
		BASEPRINTK("tvout drv. already used !!\n");
		ret =  -EBUSY;
		goto drv_used;
	}

#ifdef CONFIG_CPU_S5PV210
#ifdef CONFIG_PM 
	if((s5ptv_status.hpd_status) && !(s5ptv_status.suspend_status))
	{
		BASEPRINTK("tv is turned on\n");
#endif
#ifdef CONFIG_CPU_FREQ_S5PV210
		if((s5ptv_status.hpd_status))
			s5pv210_set_cpufreq_level(RESTRICT_TABLE);
#endif /* CONFIG_CPU_FREQ_S5PV210 */
		s5p_tv_clk_gate( true );
		tv_phy_power( true );
#ifdef CONFIG_PM
	}
	else
		BASEPRINTK("tv is off\n");
#endif
#endif
	_s5p_tv_if_init_param();

	s5p_tv_v4l2_init_param();

	mutex_unlock(mutex_for_fo);


	return 0;

drv_used:
	mutex_unlock(mutex_for_fo);
	return ret;
}

int s5p_tv_v_read(struct file *filp, char *buf, size_t count,
		  loff_t *f_pos)
{
	return 0;
}

int s5p_tv_v_write(struct file *filp, const char *buf, size_t
		   count, loff_t *f_pos)
{
	return 0;
}

int s5p_tv_v_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return 0;
}

int s5p_tv_v_release(struct file *filp)
{
#if defined(CONFIG_CPU_S5PV210) && defined(CONFIG_PM)
	if((s5ptv_status.hpd_status) && !(s5ptv_status.suspend_status))
	{
#endif
		if(s5ptv_status.vp_layer_enable)
			_s5p_vlayer_stop();
		if(s5ptv_status.tvout_output_enable)
			_s5p_tv_if_stop();
#if defined(CONFIG_CPU_S5PV210) && defined(CONFIG_PM)
	}else
		s5ptv_status.vp_layer_enable = false;
#endif 

	s5ptv_status.hdcp_en = false;

	s5ptv_status.tvout_output_enable = false;

	/*
	 * drv. release
	 *        - just check drv. state reg. or not.
	 */

#ifdef CONFIG_CPU_S5PV210
#ifdef CONFIG_PM
	if((s5ptv_status.hpd_status) && !(s5ptv_status.suspend_status))
	{
#endif
		s5p_tv_clk_gate(false);
		tv_phy_power( false );
#ifdef CONFIG_CPU_FREQ_S5PV210
		if(s5ptv_status.hpd_status)
			s5pv210_set_cpufreq_level(NORMAL_TABLE);
#endif /* CONFIG_CPU_FREQ_S5PV210 */

#ifdef CONFIG_PM
	}
#endif
#endif

	return 0;
}

static int vo_open(int layer, struct file *file)
{
	int ret = 0;

	mutex_lock(mutex_for_fo);

	/* check tvout path available!! */
	if (!s5ptv_status.tvout_output_enable) {
		BASEPRINTK("check tvout start !!\n");
		ret =  -EACCES;
		goto resource_busy;
	}

	if (s5ptv_status.grp_layer_enable[layer]) {
		BASEPRINTK("grp %d layer is busy!!\n", layer);
		ret =  -EBUSY;
		goto resource_busy;
	}

	/* set layer info.!! */
	s5ptv_overlay[layer].index = layer;

	/* set file private data.!! */
	file->private_data = &s5ptv_overlay[layer];

	mutex_unlock(mutex_for_fo);

	return 0;

resource_busy:
	mutex_unlock(mutex_for_fo);

	return ret;
}

int vo_release(int layer, struct file *filp)
{

	_s5p_grp_stop(layer);

	return 0;
}

/* modified for 2.6.29 v4l2-dev.c */
static int s5p_tv_vo0_open(struct file *file)
{
	vo_open(0, file);
	return 0;
}

static int s5p_tv_vo0_release(struct file *file)
{
	vo_release(0, file);
	return 0;
}

static int s5p_tv_vo1_open(struct file *file)
{
	vo_open(1, file);
	return 0;
}

static int s5p_tv_vo1_release(struct file *file)
{
	vo_release(1, file);
	return 0;
}
#endif

#ifdef CONFIG_TV_FB
static int s5ptvfb_alloc_framebuffer(void)
{
	int ret;

	/* alloc for each framebuffer */
	s5ptv_status.fb = framebuffer_alloc(sizeof(struct s5ptvfb_window),
					 s5ptv_status.dev_fb);
	if (!s5ptv_status.fb) {
		dev_err(s5ptv_status.dev_fb, "not enough memory\n");
		ret = -ENOMEM;
		goto err_alloc_fb;
	}

	ret = s5ptvfb_init_fbinfo(5);
	if (ret) {
		dev_err(s5ptv_status.dev_fb,
			"failed to allocate memory for fb for tv\n");
		ret = -ENOMEM;
		goto err_alloc_fb;
	}
#ifndef CONFIG_USER_ALLOC_TVOUT
	if (s5ptvfb_map_video_memory(s5ptv_status.fb)) {
		dev_err(s5ptv_status.dev_fb,
			"failed to map video memory "
			"for default window \n");
		ret = -ENOMEM;
		goto err_alloc_fb;
	}
#endif
	return 0;

err_alloc_fb:
	if (s5ptv_status.fb)
		framebuffer_release(s5ptv_status.fb);

	kfree(s5ptv_status.fb);

	return ret;
}

int s5ptvfb_free_framebuffer(void)
{
#ifndef CONFIG_USER_ALLOC_TVOUT
	if (s5ptv_status.fb)
		s5ptvfb_unmap_video_memory(s5ptv_status.fb);
#endif

	if (s5ptv_status.fb)
		framebuffer_release(s5ptv_status.fb);

	return 0;
}

int s5ptvfb_register_framebuffer(void)
{
	int ret;

	ret = register_framebuffer(s5ptv_status.fb);
	if (ret) {
		dev_err(s5ptv_status.dev_fb, "failed to register "
			"framebuffer device\n");
		return -EINVAL;
	}
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
#ifndef CONFIG_USER_ALLOC_TVOUT
	s5ptvfb_check_var(&s5ptv_status.fb->var, s5ptv_status.fb);
	s5ptvfb_set_par(s5ptv_status.fb);
	s5ptvfb_draw_logo(s5ptv_status.fb);
#endif
#endif

	return 0;
}

#endif
/*
 * struct for video
 */
#ifdef CONFIG_TV_FB
static struct v4l2_file_operations s5p_tv_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_open,
	.ioctl		= s5p_tv_ioctl,
	.release	= s5p_tv_release
};
static struct v4l2_file_operations s5p_tv_vid_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_vid_open,
	.ioctl		= s5p_tv_vid_ioctl,
	.release	= s5p_tv_vid_release
};


 #else
static struct v4l2_file_operations s5p_tv_v_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_v_open,
	.read		= s5p_tv_v_read,
	.write		= s5p_tv_v_write,
	.ioctl		= s5p_tv_v_ioctl,
	.mmap		= s5p_tv_v_mmap,
	.release	= s5p_tv_v_release
};

/*
 * struct for graphic0
 */
static struct v4l2_file_operations s5p_tv_vo0_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_vo0_open,
	.ioctl		= s5p_tv_vo_ioctl,
	.release	= s5p_tv_vo0_release
};

/*
 * struct for graphic1
 */
static struct v4l2_file_operations s5p_tv_vo1_fops = {
	.owner		= THIS_MODULE,
	.open		= s5p_tv_vo1_open,
	.ioctl		= s5p_tv_vo_ioctl,
	.release	= s5p_tv_vo1_release
};
#endif


void s5p_tv_vdev_release(struct video_device *vdev)
{
	kfree(vdev);
}

struct video_device s5p_tvout[] = {

#ifdef CONFIG_TV_FB
	[0] = {
		.name = "S5PC1xx TVOUT ctrl",
		.fops = &s5p_tv_fops,
		.ioctl_ops = &s5p_tv_v4l2_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_TVOUT,
		.tvnorms = V4L2_STD_ALL_HD,
	},
	[1] = {
		.name = "S5PC1xx TVOUT for Video",
		.fops = &s5p_tv_vid_fops,
		.ioctl_ops = &s5p_tv_v4l2_vid_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_VID,
		.tvnorms = V4L2_STD_ALL_HD,
	},
#else
	[0] = {
		.name = "S5PC1xx TVOUT Video",
		.fops = &s5p_tv_v_fops,
		.ioctl_ops = &s5p_tv_v4l2_v_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_VIDEO,
		.tvnorms = V4L2_STD_ALL_HD,
	},
	[1] = {
		.name = "S5PC1xx TVOUT Overlay0",
		.fops = &s5p_tv_vo0_fops,
		.ioctl_ops = &s5p_tv_v4l2_vo_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_GRP0,
		.tvnorms = V4L2_STD_ALL_HD,
	},
	[2] = {
		.name = "S5PC1xx TVOUT Overlay1",
		.fops = &s5p_tv_vo1_fops,
		.ioctl_ops = &s5p_tv_v4l2_vo_ops,
		.release  = s5p_tv_vdev_release,
		.minor = TVOUT_MINOR_GRP1,
		.tvnorms = V4L2_STD_ALL_HD,
	},
#endif
};

void s5p_handle_cable(void)
{
    char env_buf[120];
    char *envp[2];
    int env_offset = 0;

    if(s5ptv_status.tvout_param.out_mode != TVOUT_OUTPUT_HDMI &&
		    s5ptv_status.tvout_param.out_mode != TVOUT_OUTPUT_HDMI_RGB &&
		    s5ptv_status.tvout_param.out_mode != TVOUT_OUTPUT_DVI)
        return;

    bool previous_hpd_status = s5ptv_status.hpd_status;
#ifdef CONFIG_HDMI_HPD
    s5ptv_status.hpd_status= s5p_hpd_get_state();
#else
    return;
#endif
    
    memset(env_buf, 0, sizeof(env_buf));

    if(previous_hpd_status == s5ptv_status.hpd_status)
    {
        BASEPRINTK("same hpd_status value: %d\n", previous_hpd_status);
        return;
    }

    if(s5ptv_status.hpd_status)
    {
        BASEPRINTK("\n hdmi cable is connected \n");			
        sprintf(env_buf, "HDMI_STATE=online");
        envp[env_offset++] = env_buf;
        envp[env_offset] = NULL;
        kobject_uevent_env(&(s5p_tvout[0].dev.kobj), KOBJ_CHANGE, envp);

        if(s5ptv_status.suspend_status)
            return;

#ifdef CONFIG_CPU_FREQ_S5PV210
        s5pv210_set_cpufreq_level(RESTRICT_TABLE);
#endif /* CONFIG_CPU_FREQ_S5PV210 */

    #ifdef CONFIG_PM
        s5p_tv_clk_gate( true );
        tv_phy_power( true );
    #endif
        /* tv on */
        if ( s5ptv_status.tvout_output_enable )
                _s5p_tv_if_start();

        /* video layer start */
        if ( s5ptv_status.vp_layer_enable )
                _s5p_vlayer_start();

        /* grp0 layer start */
        if ( s5ptv_status.grp_layer_enable[0] )
                _s5p_grp_start(VM_GPR0_LAYER);

        /* grp1 layer start */
        if ( s5ptv_status.grp_layer_enable[1] )
                _s5p_grp_start(VM_GPR1_LAYER);

    }
    else{
        BASEPRINTK("\n hdmi cable is disconnected \n");

        sprintf(env_buf, "HDMI_STATE=offline");
        envp[env_offset++] = env_buf;
        envp[env_offset] = NULL;
        kobject_uevent_env(&(s5p_tvout[0].dev.kobj), KOBJ_CHANGE, envp);

        if(s5ptv_status.suspend_status)
            return;

        if ( s5ptv_status.vp_layer_enable ) {
            _s5p_vlayer_stop();
            s5ptv_status.vp_layer_enable = true;

        }

        /* grp0 layer stop */
        if ( s5ptv_status.grp_layer_enable[0] ) {
            _s5p_grp_stop(VM_GPR0_LAYER);
            s5ptv_status.grp_layer_enable[VM_GPR0_LAYER] = true;
        }

        /* grp1 layer stop */
        if ( s5ptv_status.grp_layer_enable[1] ) {
            _s5p_grp_stop(VM_GPR1_LAYER);
            s5ptv_status.grp_layer_enable[VM_GPR0_LAYER] = true;
        }

        /* tv off */
        if ( s5ptv_status.tvout_output_enable ) {
            _s5p_tv_if_stop();
            s5ptv_status.tvout_output_enable = true;
            s5ptv_status.tvout_param_available = true;
        }

    #ifdef CONFIG_PM
        /* clk & power off */
        s5p_tv_clk_gate( false );
        tv_phy_power( false );
    #endif

#ifdef CONFIG_CPU_FREQ_S5PV210
        s5pv210_set_cpufreq_level(NORMAL_TABLE);
#endif /* CONFIG_CPU_FREQ_S5PV210 */
    }
}

#define S5P_TVMAX_CTRLS 	ARRAY_SIZE(s5p_tvout)
/*
 *  Probe
 */

static int __devinit s5p_tv_probe(struct platform_device *pdev)
{
	int 	irq_num;
	int 	ret;
	int 	i;

	s5ptv_status.dev_fb = &pdev->dev;
	clks_enabled = 0;

	__s5p_sdout_probe(pdev, 0);
	__s5p_vp_probe(pdev, 1);
	__s5p_mixer_probe(pdev, 2);

#ifdef CONFIG_CPU_S5PV210
	tv_clk_get(pdev, &s5ptv_status);
	s5p_tv_clk_gate( true );
#endif
#ifdef CONFIG_CPU_S5PV210	
	__s5p_hdmi_probe(pdev, 3, 4);
	__s5p_hdcp_init( );
#endif	

#ifdef FIX_27M_UNSTABLE_ISSUE /* for smdkc100 pop */
	writel(0x1, S5PC1XX_GPA0_BASE + 0x56c);
#endif

	/* check EINT init state */

#ifdef CONFIG_CPU_S5PV210
#ifdef CONFIG_HDMI_HPD
    s5ptv_status.hpd_status= s5p_hpd_get_state();
#else
    s5ptv_status.hpd_status= 0;
#endif    
#endif
	dev_info(&pdev->dev, "hpd status is cable %s\n", 
		s5ptv_status.hpd_status ? "inserted":"removed");


	/* interrupt */
	TVOUT_IRQ_INIT(irq_num, ret, pdev, 0, out, __s5p_mixer_irq, "mixer");
	TVOUT_IRQ_INIT(irq_num, ret, pdev, 1, out_hdmi_irq, __s5p_hdmi_irq , "hdmi");
	TVOUT_IRQ_INIT(irq_num, ret, pdev, 2, out_tvenc_irq, s5p_tvenc_irq, "tvenc");

	/* v4l2 video device registration */
	for (i = 0; i < S5P_TVMAX_CTRLS; i++) {
		s5ptv_status.video_dev[i] = &s5p_tvout[i];

		if (video_register_device(s5ptv_status.video_dev[i],
				VFL_TYPE_GRABBER, s5p_tvout[i].minor) != 0) {

			dev_err(&pdev->dev,
				"Couldn't register tvout driver.\n");
			return 0;
		}
	}


#ifdef CONFIG_TV_FB
	mutex_init(&s5ptv_status.fb_lock);

	/* for default start up */
	_s5p_tv_if_init_param();

	s5ptv_status.tvout_param.disp_mode = TVOUT_720P_60;
	s5ptv_status.tvout_param.out_mode  = TVOUT_OUTPUT_HDMI;

#ifndef CONFIG_USER_ALLOC_TVOUT
	s5p_tv_clk_gate(true);
	tv_phy_power(true);
	_s5p_tv_if_set_disp();
#endif

	s5ptvfb_set_lcd_info(&s5ptv_status);

	/* prepare memory */
	if (s5ptvfb_alloc_framebuffer())
		goto err_alloc;

	if (s5ptvfb_register_framebuffer())
		goto err_alloc;
#ifndef CONFIG_USER_ALLOC_TVOUT
	s5ptvfb_display_on(&s5ptv_status);
#endif
#endif

	mutex_for_fo = kmalloc(sizeof(struct mutex),
		GFP_KERNEL);

	if (mutex_for_fo == NULL) {
		dev_err(&pdev->dev,
			"failed to create mutex handle\n");
		goto out;
	}

	mutex_init(mutex_for_fo);

#ifdef CONFIG_CPU_S5PV210
	/* added for phy cut off when boot up */
	clk_enable(s5ptv_status.i2c_phy_clk);
	__s5p_hdmi_phy_power(false);
	clk_disable(s5ptv_status.i2c_phy_clk);
	s5p_tv_clk_gate( false );
#endif
	return 0;

#ifdef CONFIG_TV_FB
err_alloc:
#endif

out_tvenc_irq:
	free_irq(IRQ_HDMI, pdev);

out_hdmi_irq:
	free_irq(IRQ_MIXER, pdev);

out:
	printk(KERN_ERR "not found (%d). \n", ret);

	return ret;
}

/*
 *  Remove
 */
static int s5p_tv_remove(struct platform_device *pdev)
{
	__s5p_hdmi_release(pdev);
	__s5p_sdout_release(pdev);
	__s5p_mixer_release(pdev);
	__s5p_vp_release(pdev);

	clk_disable(s5ptv_status.tvenc_clk);
	clk_disable(s5ptv_status.vp_clk);
	clk_disable(s5ptv_status.mixer_clk);
	clk_disable(s5ptv_status.hdmi_clk);
	clk_disable(s5ptv_status.sclk_hdmi);
	clk_disable(s5ptv_status.sclk_mixer);
	clk_disable(s5ptv_status.sclk_dac);

	clk_put(s5ptv_status.tvenc_clk);
	clk_put(s5ptv_status.vp_clk);
	clk_put(s5ptv_status.mixer_clk);
	clk_put(s5ptv_status.hdmi_clk);
	clk_put(s5ptv_status.sclk_hdmi);
	clk_put(s5ptv_status.sclk_mixer);
	clk_put(s5ptv_status.sclk_dac);
	clk_put(s5ptv_status.sclk_pixel);
	clk_put(s5ptv_status.sclk_hdmiphy);

	free_irq(IRQ_MIXER, pdev);
	free_irq(IRQ_HDMI, pdev);
	free_irq(IRQ_TVENC, pdev);

	mutex_destroy(mutex_for_fo);

	return 0;
}


#ifdef CONFIG_PM
/*
 *  Suspend
 */
int s5p_tv_early_suspend(struct platform_device *dev, pm_message_t state)
{
    BASEPRINTK("(hpd_status = %d)++\n", s5ptv_status.hpd_status);

    mutex_lock(mutex_for_fo);
    s5ptv_status.suspend_status = true;

    if(!(s5ptv_status.hpd_status))
    {
	    mutex_unlock(mutex_for_fo);
	    return 0;
    }
    else
    {
	    /* video layer stop */
	    if ( s5ptv_status.vp_layer_enable ) {
		    _s5p_vlayer_stop();
		    s5ptv_status.vp_layer_enable = true;

	    }

	    /* grp0 layer stop */
	    if ( s5ptv_status.grp_layer_enable[0] ) {
		    _s5p_grp_stop(VM_GPR0_LAYER);
		    s5ptv_status.grp_layer_enable[VM_GPR0_LAYER] = true;
	    }

	    /* grp1 layer stop */
	    if ( s5ptv_status.grp_layer_enable[1] ) {
		    _s5p_grp_stop(VM_GPR1_LAYER);
		    s5ptv_status.grp_layer_enable[VM_GPR0_LAYER] = true;
	    }

	    /* tv off */
	    if ( s5ptv_status.tvout_output_enable ) {
		    _s5p_tv_if_stop();
		    s5ptv_status.tvout_output_enable = true;
		    s5ptv_status.tvout_param_available = true;
	    }

	    /* clk & power off */
	    s5p_tv_clk_gate( false );
	    if(s5ptv_status.tvout_param.out_mode == TVOUT_OUTPUT_HDMI || s5ptv_status.tvout_param.out_mode == TVOUT_OUTPUT_HDMI_RGB)
		    tv_phy_power( false );

#ifdef CONFIG_CPU_FREQ_S5PV210
	    s5pv210_set_cpufreq_level(NORMAL_TABLE);
#endif /* CONFIG_CPU_FREQ_S5PV210 */
    }

    mutex_unlock(mutex_for_fo);
    BASEPRINTK("()--\n");
    return 0;
}

/*
 *  Resume
 */
int s5p_tv_late_resume(struct platform_device *dev)
{
    BASEPRINTK("(hpd_status = %d)++\n", s5ptv_status.hpd_status);

    mutex_lock(mutex_for_fo);
    s5ptv_status.suspend_status = false;

    if(!(s5ptv_status.hpd_status))
    {
	    mutex_unlock(mutex_for_fo);
	    return 0;
    }
    else
    {
#ifdef CONFIG_CPU_FREQ_S5PV210
		s5pv210_set_cpufreq_level(RESTRICT_TABLE);
#endif /* CONFIG_CPU_FREQ_S5PV210 */

	/* clk & power on */
	s5p_tv_clk_gate( true );
	if(s5ptv_status.tvout_param.out_mode == TVOUT_OUTPUT_HDMI || s5ptv_status.tvout_param.out_mode == TVOUT_OUTPUT_HDMI_RGB)
	    tv_phy_power( true );

	/* tv on */
	if (s5ptv_status.tvout_output_enable) {
		_s5p_tv_if_start();
	}

	/* video layer start */
	if (s5ptv_status.vp_layer_enable)
		_s5p_vlayer_start();

	/* grp0 layer start */
	if (s5ptv_status.grp_layer_enable[0])
		_s5p_grp_start(VM_GPR0_LAYER);

	/* grp1 layer start */
	if (s5ptv_status.grp_layer_enable[1])
		_s5p_grp_start(VM_GPR1_LAYER);

#ifdef CONFIG_TV_FB
	if (s5ptv_status.tvout_output_enable) {
		s5ptvfb_display_on(&s5ptv_status);
		s5ptvfb_set_par(s5ptv_status.fb);
	}
#endif
    }
    mutex_unlock(mutex_for_fo);
    BASEPRINTK("()--\n");
    return 0;
}
#else
#define s5p_tv_suspend NULL
#define s5p_tv_resume NULL
#endif

#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend s5p_tv_early_suspend_desc = {
     .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
     .suspend = s5p_tv_early_suspend,
     .resume = s5p_tv_late_resume,
};
#endif
#endif

static struct platform_driver s5p_tv_driver = {
	.probe		= s5p_tv_probe,
	.remove		= s5p_tv_remove,
	#ifdef CONFIG_PM
        #ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= s5p_tv_early_suspend,
	.resume		= s5p_tv_late_resume,
        #endif
	#else
	.suspend 	= NULL,
	.resume  	= NULL,
	#endif
	.driver		= {
		.name	= "s5p-tvout",
		.owner	= THIS_MODULE,
	},
};

static char banner[] __initdata =
	KERN_INFO "S5PC1XX TVOUT Driver, (c) 2009 Samsung Electronics\n";

int __init s5p_tv_init(void)
{
	int ret;

	printk(banner);

	ret = platform_driver_register(&s5p_tv_driver);

	if (ret) {
		printk(KERN_ERR "Platform Device Register Failed %d\n", ret);
		return -1;
	}

#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend(&s5p_tv_early_suspend_desc);
#endif
#endif
	return 0;
}

static void __exit s5p_tv_exit(void)
{
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&s5p_tv_early_suspend_desc);
#endif
#endif
	platform_driver_unregister(&s5p_tv_driver);
}

late_initcall(s5p_tv_init);
module_exit(s5p_tv_exit);

MODULE_AUTHOR("SangPil Moon");
MODULE_DESCRIPTION("SS5PC1XX TVOUT driver");
MODULE_LICENSE("GPL");
