#ifndef _MANGO_KEYS_H
#define _MANGO_KEYS_H

struct mango_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	int irq;
	int config;
	int pull;
	int led_gpio;
	char *led_desc;
	int led_config;
	int led_active_low;
};

struct mango_keys_platform_data {
	struct mango_keys_button *buttons;
	int nbuttons;
	unsigned int rep:1;		/* enable input subsystem auto repeat */
};

#endif
