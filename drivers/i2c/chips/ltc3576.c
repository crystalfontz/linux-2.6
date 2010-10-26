/*
    ltc3576.c - driver for Linear Tech's ltc3576

    Copyright (C) 2010  Crystalfontz America, Inc. <www.crystalfontz.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/ltc3576.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define SHORT_NAME			"ltc3576"
#define PIN_POLL_FREQ			msecs_to_jiffies(5000)

#define R_A	0x0
#define		R_A_SW_REG1_V(v)	((v&0xF)<<4)
#define		R_A_SW_REG2_V(v)	((v&0xF))
#define R_B	0x1
#define		R_B_SW_REG3_V(v)	((v&0xF)<<4)
#define 	R_B_ENABLE3		(1<<3)
#define 	R_B_ENABLE2		(1<<2)
#define 	R_B_ENABLE1		(1<<1)
#define 	R_B_ENABLE_OTG		(1<<0)
#define R_C	0x2
#define		R_C_SW_REG1_MODE(v)	((v&0x3)<<6)
#define		R_C_SW_REG2_MODE(v)	((v&0x3)<<4)
#define		R_C_SW_REG3_MODE(v)	((v&0x3)<<2)
#define		R_C_CURRENT_LIMIT(v)	((v&0x3)<<0)
#define R_D	0x3
#define 	R_D_DIS_BATT_CHRG	(1<<7)
#define 	R_D_HIGH_POW_SUSPEND	(1<<6)


struct ltc3576_data {
	struct mutex		iolock;
	struct i2c_client	*client;
	struct delayed_work	charge_work;
	struct delayed_work	acpr_work;
	struct ltc3576_platform_data	pdata;
#define CHARGE_NOT_CHARGING	0
#define CHARGE_CHARGING		1
#define CHARGE_NTC_FAULT	2
#define CHARGE_BAD_BATTERY	3
	int			charge_status;
	int			nchrg_duty;
	int			on_ac;
};

static struct ltc3576_data *singleton;

static int ltc3576_remove(struct i2c_client *client);
static unsigned int regulator_settings = 0xfff0;
module_param(regulator_settings, int, 0);

static ssize_t ltc3576_charge_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	const char* state;
	unsigned int duty = 0;
	struct ltc3576_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->iolock);
	if (gpio_is_valid(data->pdata.nchrg_pin)) {
		duty = data->nchrg_duty;
		switch (data->charge_status) {
		case CHARGE_NOT_CHARGING: state = "not charging"; break;
		case CHARGE_CHARGING: state = "charging"; break;
		case CHARGE_NTC_FAULT: state = "NTC fault"; break;
		case CHARGE_BAD_BATTERY: state = "bad battery / OTG short "; break;
		default: state = "undefined"; break;
		}
	}
	else
		state = "unknown";

	mutex_unlock(&data->iolock);

	return sprintf(buf,"%s, duty cycle %u\n",state,duty);
}
static DEVICE_ATTR(nchrg_status, 0444, ltc3576_charge_show, NULL);

static ssize_t ltc3576_acpr_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	const char* state;
	struct ltc3576_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->iolock);
	if (gpio_is_valid(data->pdata.nacpr_pin))
		state = data->on_ac?"On":"NOT on";
	else
		state = "unknwon if on";
	mutex_unlock(&data->iolock);

	return sprintf(buf,"%s AC power\n",state);
}
static DEVICE_ATTR(acpr_status, 0444, ltc3576_acpr_show, NULL);

/* ------------------------- Device ---------------------------------- */

static void ltc3576_finish_work(struct ltc3576_data* data)
{
	cancel_delayed_work(&data->charge_work);
	flush_scheduled_work();
}

static void ltc3576_poll_nchrg(struct work_struct *work)
{
	int i;
	unsigned int try = 0, duty, highs = 0;
	u64 end;
	int status = CHARGE_NOT_CHARGING;
	if (!singleton)
		return;

	/* you might be off on the duty cycle, datasheet says try again */
	while (try == 0 || (try < 2 && (duty > 40 && duty < 60))) {
		try++;
		/* read the pin over 24/35000ths of a second , ~7ms*/
		end = get_jiffies_64() + msecs_to_jiffies(7);
		for(i=0; time_before64(get_jiffies_64(), end); i++) {
			if (gpio_get_value(singleton->pdata.nchrg_pin))
				highs++;
		}
		duty = highs*100/i;
	}

	if (duty >= 98)
		status = CHARGE_NOT_CHARGING;
	else if (duty <= 2)
		status = CHARGE_CHARGING;
	else if (duty <= 8 || duty >= 92)
		status = CHARGE_NTC_FAULT;
	else if (duty <= 14 || duty >= 86)
		status = CHARGE_BAD_BATTERY;
	else
		status = -1;

	/* set the status */
	mutex_lock(&singleton->iolock);
	singleton->charge_status = status;
	singleton->nchrg_duty = duty;
	mutex_unlock(&singleton->iolock);

	schedule_delayed_work(&singleton->charge_work, PIN_POLL_FREQ);
}


static void ltc3576_poll_nacpr(struct work_struct *work)
{
	int status = !gpio_get_value(singleton->pdata.nacpr_pin);

	/* set the status */
	mutex_lock(&singleton->iolock);
	singleton->on_ac = status;
	mutex_unlock(&singleton->iolock);

	schedule_delayed_work(&singleton->acpr_work, PIN_POLL_FREQ);
}


static int ltc3576_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ltc3576_data *data;
	int err = 0;

	if (singleton) {
		dev_err(&client->dev, "only one " SHORT_NAME " chip allowed\n");
		return -ENODEV;
	}

	if (!(data = kzalloc(sizeof(struct ltc3576_data), GFP_KERNEL)))
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->iolock);
	INIT_DELAYED_WORK(&data->charge_work, ltc3576_poll_nchrg);
	INIT_DELAYED_WORK(&data->acpr_work, ltc3576_poll_nacpr);

	err = device_create_file(&client->dev, &dev_attr_nchrg_status);
	if (err) {
		dev_err(&client->dev, "cannot create nchrg status device file, err %d\n", err);
	}

	err = device_create_file(&client->dev, &dev_attr_acpr_status);
	if (err) {
		dev_err(&client->dev, "cannot create acpr status device file, err %d\n", err);
	}

	if (client->dev.platform_data) {
		data->pdata = *((struct ltc3576_platform_data*)client->dev.platform_data);
	} else {
		data->pdata.nchrg_pin = -1;
		data->pdata.nacpr_pin = -1;
	}

	err = gpio_request(data->pdata.nchrg_pin, SHORT_NAME "_nchrg");
	if (err) {
		printk(KERN_INFO SHORT_NAME " no nchrg pin available.  Ignoring.\n");
		data->pdata.nchrg_pin = -1;
	} else {
		gpio_direction_input(data->pdata.nchrg_pin);
	}
	err = gpio_request(data->pdata.nacpr_pin, SHORT_NAME "_nacpr");
	if (err) {
		printk(KERN_INFO SHORT_NAME " no nacpr pin available.  Ignoring.\n");
		data->pdata.nacpr_pin = -1;
	} else {
		gpio_direction_input(data->pdata.nacpr_pin);
	}

	printk(KERN_INFO SHORT_NAME " driver ready, using, using 0x%04x for regulator settings\n", regulator_settings);
	singleton = data;

	err = i2c_smbus_write_byte_data(singleton->client,R_A,(regulator_settings>>8)&0xFF);
	if (err) {
		dev_err(&client->dev, "failed to set subaddress A, err %d\n", err);
	}
	err = i2c_smbus_write_byte_data(singleton->client,R_B,regulator_settings&0xFF);
	if (err) {
		dev_err(&client->dev, "failed to set subaddress B, err %d\n", err);
	}

	if (gpio_is_valid(singleton->pdata.nchrg_pin))
		schedule_work(&singleton->charge_work.work);

	if (gpio_is_valid(singleton->pdata.nacpr_pin))
		schedule_work(&singleton->acpr_work.work);

	err = 0;

	return err;
}

static int ltc3576_remove(struct i2c_client *client)
{
	struct ltc3576_data *data = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_nchrg_status);
	device_remove_file(&client->dev, &dev_attr_acpr_status);

	ltc3576_finish_work(data);

	gpio_free(data->pdata.nchrg_pin);
	gpio_free(data->pdata.nacpr_pin);

	kfree(data);
	singleton = NULL;

	return 0;
}

/*----------------------- Driver ----------------------------------------- */

static const struct i2c_device_id ltc3576_id[] = {
	{ SHORT_NAME, 0x09 },
	{ }
};

static struct i2c_driver ltc3576_driver = {
	.driver = {
		.name	= SHORT_NAME,
	},
	.id_table	= ltc3576_id,
	.probe		= ltc3576_probe,
	.remove		= ltc3576_remove,
};

static int __init ltc3576_init(void)
{
	return i2c_add_driver(&ltc3576_driver);
}

static void __exit ltc3576_exit(void)
{
	i2c_del_driver(&ltc3576_driver);
}


MODULE_AUTHOR("Rob Emanuele <rje@crystalfontz.com>");
MODULE_DESCRIPTION("Linear Tech LTC3576 driver");
MODULE_LICENSE("GPL");

subsys_initcall(ltc3576_init);
module_exit(ltc3576_exit);
