/*
 *  Inverting.c - Inverting GPIO Expansion
 *
 *  Copyright (C) 2010 Crystalfontz America, Inc. <www.crystalfontz.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/inverted_gpio.h>

struct inverted_gpio_chip {
	struct gpio_chip gpio_chip;
	struct inverted_gpio_data *data;
};

static int inverted_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct inverted_gpio_chip *chip;

	chip = container_of(gc, struct inverted_gpio_chip, gpio_chip);

	return !gpio_get_value(chip->data[off].actual_gpio);
}


static void inverted_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct inverted_gpio_chip *chip;

	chip = container_of(gc, struct inverted_gpio_chip, gpio_chip);

	gpio_set_value(chip->data[off].actual_gpio, !val);
}


static int inverted_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct inverted_gpio_chip *chip;

	chip = container_of(gc, struct inverted_gpio_chip, gpio_chip);

	return gpio_direction_input(chip->data[off].actual_gpio);
}


static int inverted_gpio_direction_output(struct gpio_chip *gc, unsigned off, int val)
{
	struct inverted_gpio_chip *chip;

	chip = container_of(gc, struct inverted_gpio_chip, gpio_chip);

	return gpio_direction_output(chip->data[off].actual_gpio, !val);
}


static int inverted_gpio_probe(struct platform_device *pdev)
{
	struct inverted_gpio_platform_data *pdata;
	struct inverted_gpio_chip *chip;
	int ret, i;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_dbg(&pdev->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct inverted_gpio_chip), GFP_KERNEL);
	if (chip == NULL) {
		ret = -ENOMEM;
		goto probe_fail;
	}

	chip->data = kzalloc(sizeof(struct inverted_gpio_data) * pdata->gpio_count, GFP_KERNEL);
	if (chip->data == NULL) {
		ret = -ENOMEM;
		goto probe_fail;
	}

	for (i = 0; i < pdata->gpio_count; i++) {
		chip->data[i] = pdata->inverted_gpios[i];
		gpio_request(chip->data[i].actual_gpio, chip->data[i].name);
	}

	chip->gpio_chip.direction_input = inverted_gpio_direction_input;
	chip->gpio_chip.direction_output = inverted_gpio_direction_output;
	chip->gpio_chip.set = inverted_gpio_set_value;
	chip->gpio_chip.get = inverted_gpio_get_value;
	chip->gpio_chip.can_sleep = 1;

	chip->gpio_chip.base = pdata->gpio_base;
	chip->gpio_chip.ngpio = pdata->gpio_count;
	chip->gpio_chip.label = "inverted gpios";
	chip->gpio_chip.owner = THIS_MODULE;

	platform_set_drvdata(pdev, chip);

	ret = gpiochip_add(&chip->gpio_chip);

	if (!ret) {
		printk("inverting_gpios: %u inverting gpios configured\n", chip->gpio_chip.ngpio);
	}

probe_fail:
	if (ret && chip) {
		if (chip->data)
			kfree(chip->data);
		kfree(chip);
	}
	return ret;
}

static int __devexit inverted_gpio_remove(struct platform_device *pdev)
{
	struct inverted_gpio_chip *chip = platform_get_drvdata(pdev);
	int ret, i;

	for (i = 0; i < chip->gpio_chip.ngpio; i++) {
		gpio_free(chip->data[i].actual_gpio);
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&pdev->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip->data);
	kfree(chip);
	return 0;
}

static struct platform_driver inverted_gpio_driver = {
	.driver = {
		.name	= "inverted_gpio",
		.owner	= THIS_MODULE,
	},
	.probe		= inverted_gpio_probe,
	.remove		= __devexit_p(inverted_gpio_remove),
};

static int __init inverted_gpio_init(void)
{
	return platform_driver_probe(&inverted_gpio_driver, inverted_gpio_probe);
}
subsys_initcall(inverted_gpio_init);

static void __exit inverted_gpio_exit(void)
{
	platform_driver_unregister(&inverted_gpio_driver);
}
module_exit(inverted_gpio_exit);

MODULE_AUTHOR("Rob Emanuele <rje@crystalfontz.com>");
MODULE_DESCRIPTION("Virtual GPIO Inverter");
MODULE_LICENSE("GPL");
