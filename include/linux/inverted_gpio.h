#ifndef __LINUX_INVERTED_GPIO_H
#define __LINUX_INVERTED_GPIO_H

/*
 * Platform Data for the virtual inverted gpio driver
 *
 * Copyright (C) 2010 Crystalfontz America, Inc. <www.crystalfontz.com>
 */

struct inverted_gpio_data {
	int		actual_gpio;
	const char*	name;
	int		real_inverter:1;
};

struct inverted_gpio_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;
	/* count of inverted gpios in this instance */
	unsigned	gpio_count;

	struct inverted_gpio_data *inverted_gpios;

	void		*context;	/* param to setup/teardown */

};

#endif /* __LINUX_INVERTED_GPIO_H */

