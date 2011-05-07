#ifndef __VD5376_H__
#define __VD5376_H__

struct vd5376_mouse_data
{
	int id;
	int status;
	s8 x;
	s8 y;
};

struct vd5376_platform_data
{
	/* Configuration parameters */
	int	btn_gpio;	// use Button (option)
	char	*btn_desc;	// use Button (option)
	int	btn_irq;	// use Button (option)
	int	btn_config;	// use Button (option)
	int	btn_active_low;	// use Button (option)
	int	pd_gpio;	// Power Down (need)
	char	*pd_desc;	// Power Down (need)
	int	gpio;		// use Mode (option)
	char	*desc;		// use Mode (option)
	int	irq;		// use Mode (option)
	int	config;		// use Mode (option)
	int	active_low;	// Active State (need)
	int	delay;		// use Polling Mode (need)
	int	id;		// Multi Finger ID (need)
	void	(*vd5376_callback)(struct vd5376_mouse_data *data);
};

struct vd5376_mouse_platform_data
{
	struct vd5376_platform_data	*data;
	int				ndatas;
	int scan_ms;
};
#endif
