/*
    tps65180.c - driver for TI tps65180

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
#include <linux/i2c/tps65180.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define SHORT_NAME			"tps65180"
#define REGS_COUNT			18
#define	POLL_FREQ			msecs_to_jiffies(1000)
#define WAKEUP_DELAY_USECS		1800
#define VCOM_NEG_MAX			2750

#define R_TMST_VALUE	0x0
#define R_ENABLE	0x1
#define 	R_ENABLE_ACTIVE		(1<<7)
#define 	R_ENABLE_STANDBY	(1<<6)
#define 	R_ENABLE_V3P3_SWITCH_EN	(1<<5)
#define 	R_ENABLE_VCOM_EN	(1<<4)
#define 	R_ENABLE_VDDH_EN	(1<<3)
#define 	R_ENABLE_VPOS_EN	(1<<2)
#define 	R_ENABLE_VEE_EN		(1<<1)
#define 	R_ENABLE_VNEG_EN	(1<<0)
#define R_VP_ADJUST	0x2
#define R_VN_ADJUST	0x3
#define R_VCOM_ADJUST	0x4
#define R_INT_ENABLE1	0x5
#define R_INT_ENABLE2	0x6
#define R_INT_STATUS1	0x7
#define R_INT_STATUS2	0x8
#define 	R_INT1_TSDN		(1<<6)
#define 	R_INT1_HOT		(1<<5)
#define 	R_INT1_TMST_HOT		(1<<4)
#define 	R_INT1_TMST_COOL	(1<<3)
#define 	R_INT1_UVLOW		(1<<2)
#define 	R_INT2_VB_UV		(1<<7)
#define 	R_INT2_VDDH_UV		(1<<6)
#define 	R_INT2_VN_UV		(1<<5)
#define 	R_INT2_VPOS_UV		(1<<4)
#define 	R_INT2_VEE_UV		(1<<3)
#define 	R_INT2_VNEG_UV		(1<<1)
#define 	R_INT2_EOC		(1<<0)
#define R_PWR_SEQ0	0x9
#define R_PWR_SEQ1	0xA
#define R_PWR_SEQ2	0xB
#define R_TMST_CONFIG	0xC
#define 	R_TMST_CONFIG_READ_THERM	(1<<7)
#define 	R_TMST_CONFIG_CONV_END		(1<<5)
#define 	R_TMST_CONFIG_FAULT_QUE_GET(v)	((v&0x18)>>3)
#define 	R_TMST_CONFIG_FAULT_QUE_SET(v)	((v&0x3)<<3)
#define 	R_TMST_CONFIG_FAULT_QUE_CLR	(1<<2)
#define R_TMST_OS	0xD
#define R_TMST_HYST	0xE
#define R_PG_STATUS	0xF
#define R_REVID		0x10
#define 	R_REVID_VERSION(v)		(v&0xF)
#define 	R_REVID_VERSION_TPS65180	(0)
#define R_FIX_READ_PTR	0x11

struct tps65180_data {
	unsigned long		flags;
#define F_AWAKE			0
#define F_POLL_MODE		1
#define F_TEMP_READY		2
	struct mutex		iolock;
	struct i2c_client	*client;
	struct delayed_work	interrupt_work;
	int			wakeup_pin;
	int			pwr_good_pin;
	int			int_pin;
	char			temperature;
};

static struct tps65180_data *singleton;
static int tps65180_remove(struct i2c_client *client);
static void tps65180_finish_work(struct tps65180_data* data);
static void tps65180_wakeup_setup(struct tps65180_data * data);
static unsigned int vcom_neg_mv = 1250;
module_param(vcom_neg_mv, int, 0);
static unsigned int start_awake = 0;
module_param(start_awake, bool, 0);

void tps65180_temp_conv_start_internal(void)
{
	char val;
	/* Enable the ADC Conversion Interrupt */
	val = i2c_smbus_read_byte_data(singleton->client,R_INT_ENABLE2);
	i2c_smbus_write_byte_data(singleton->client,R_INT_ENABLE2,val|R_INT2_EOC);

	/* Start the Conversion */
	clear_bit(F_TEMP_READY,&singleton->flags);
	val = i2c_smbus_read_byte_data(singleton->client,R_TMST_CONFIG);
	i2c_smbus_write_byte_data(singleton->client,R_TMST_CONFIG,val|R_TMST_CONFIG_READ_THERM);
}

int tps65180_temp_conv_start(void)
{
	char err = 0;
	if (!singleton)
		return -ENODEV;

	mutex_lock(&singleton->iolock);

	if (test_bit(F_AWAKE,&singleton->flags)) {
		tps65180_temp_conv_start_internal();
	} else {
		err = -EAGAIN;
	}
	mutex_unlock(&singleton->iolock);
	return err;
}
EXPORT_SYMBOL(tps65180_temp_conv_start);


int tps65180_temp_conv_ready(void)
{
	if (!singleton)
		return -ENODEV;

	return test_bit(F_TEMP_READY,&singleton->flags);
}
EXPORT_SYMBOL(tps65180_temp_conv_ready);


int tps65180_temp_read(void)
{
	return (singleton->temperature);
}
EXPORT_SYMBOL(tps65180_temp_read);


int tps65180_wakeup(unsigned enable)
{
	int err = 0;

	if (!singleton)
		return -ENODEV;

	mutex_lock(&singleton->iolock);

	if (gpio_is_valid(singleton->wakeup_pin)) {
		err =  gpio_direction_output(singleton->wakeup_pin,enable);
		udelay(WAKEUP_DELAY_USECS);
	}
	else
		err = 0;

	if (!err) {
		if (enable) {
			tps65180_wakeup_setup(singleton);
			if (test_bit(F_POLL_MODE,&singleton->flags)) {
				schedule_work(&singleton->interrupt_work.work);
			}
		} else {
			clear_bit(F_AWAKE,&singleton->flags);
			tps65180_finish_work(singleton);
		}
	}
	mutex_unlock(&singleton->iolock);

	return err;
}
EXPORT_SYMBOL(tps65180_wakeup);


/* ---------------- sysfs interface ---------------------- */

static ssize_t tps65180_read(struct kobject *kobj,
			    struct bin_attribute *bin_attr,
			    char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct tps65180_data *data = i2c_get_clientdata(client);

	if (off < REGS_COUNT) {
		unsigned int i;
		count = ((count<(REGS_COUNT - off))?count:(REGS_COUNT - off));

		mutex_lock(&data->iolock);

		for (i = 0; i < count; i++)
			buf[i] = i2c_smbus_read_byte_data(client,i+off);

		mutex_unlock(&data->iolock);
	} else {
		count = 0;
	}

	return count;
}

static ssize_t tps65180_write(struct kobject *kobj,
			    struct bin_attribute *bin_attr,
			    char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct tps65180_data *data = i2c_get_clientdata(client);
	int ret = 0;

	if (off < REGS_COUNT) {
		unsigned int i;
		count = ((count<(REGS_COUNT - off))?count:(REGS_COUNT - off));

		mutex_lock(&data->iolock);

		for (i = 0; i < count && !ret; i++)
			ret = i2c_smbus_write_byte_data(client,i+off,buf[i]);

		mutex_unlock(&data->iolock);

		if (!ret)
			ret = count;
	}

	return ret;
}

static struct bin_attribute tps65180_sys[] = {
	{
		.attr = {
			.name = "regs",
			.mode = S_IRUGO,
		},
		.size = REGS_COUNT,
		.read = tps65180_read,
		.write = tps65180_write,
	},
};

static ssize_t tps65180_wakeup_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	unsigned int wake;
	int ret;

	ret = sscanf(buf, "%u", &wake);
	if (ret < 1) {
		dev_err(dev, "invalid value\n");
                return -EINVAL;
	}

	tps65180_wakeup(wake?1:0);
	return n;
}

static ssize_t tps65180_wakeup_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct tps65180_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", test_bit(F_AWAKE,&data->flags));
}
static DEVICE_ATTR(wakeup, 0644, tps65180_wakeup_show, tps65180_wakeup_store);

/* ------------------------- Device ---------------------------------- */

static irqreturn_t tps65180_dispatch_irq(int irq, void *dev_id)
{
	if (!singleton)
		return IRQ_NONE;

        schedule_work(&singleton->interrupt_work.work);

	return IRQ_HANDLED;
}

static void tps65180_finish_work(struct tps65180_data* data)
{
	cancel_delayed_work(&data->interrupt_work);
	flush_scheduled_work();
}

static void tps65180_wakeup_setup(struct tps65180_data *data)
{
	set_bit(F_AWAKE,&singleton->flags);
	tps65180_temp_conv_start_internal();
	i2c_smbus_write_byte_data(singleton->client,R_ENABLE,
				  R_ENABLE_V3P3_SWITCH_EN|R_ENABLE_VCOM_EN|R_ENABLE_VDDH_EN|
				  R_ENABLE_VPOS_EN|R_ENABLE_VEE_EN|R_ENABLE_VNEG_EN);
	i2c_smbus_write_byte_data(singleton->client,R_VCOM_ADJUST,vcom_neg_mv*0xff/VCOM_NEG_MAX);
}

static void tps65180_poll_interrupts(struct work_struct *work)
{
	char bits1, bits2, val;

	if (!singleton)
		return;

	mutex_lock(&singleton->iolock);

	if (test_bit(F_AWAKE,&singleton->flags)) {

		/* Handle Interrupt 1*/
		bits1 = i2c_smbus_read_byte_data(singleton->client,R_INT_STATUS1);

		/* Handle Interrupt 2*/
		bits2 = i2c_smbus_read_byte_data(singleton->client,R_INT_STATUS2);
		if (bits2 & R_INT2_EOC) {
			/* Temperature Ready */
			singleton->temperature = i2c_smbus_read_byte_data(singleton->client,R_TMST_VALUE);
			set_bit(F_TEMP_READY,&singleton->flags);
			val = i2c_smbus_read_byte_data(singleton->client,R_INT_ENABLE2);
			i2c_smbus_write_byte_data(singleton->client,R_INT_ENABLE2,val&~R_INT2_EOC);
			bits2 &= ~R_INT2_EOC;
		}

		if (bits1||bits2)
			dev_err(&singleton->client->dev, " unhandled interrupt bits INT1: 0x%02x INT2: 0x%02x.\n", bits1, bits2);

		if (test_bit(F_POLL_MODE,&singleton->flags)) {
			schedule_delayed_work(&singleton->interrupt_work,POLL_FREQ);
		}
	}

	mutex_unlock(&singleton->iolock);
}


static int tps65180_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tps65180_data *data;
	int err = 0;
	int i;
	char revid;

	if (singleton) {
		dev_err(&client->dev, "only one " SHORT_NAME " chip allowed\n");
		return -ENODEV;
	}

	if (!(data = kzalloc(sizeof(struct tps65180_data), GFP_KERNEL)))
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->iolock);
	INIT_DELAYED_WORK(&data->interrupt_work, tps65180_poll_interrupts);

	for(i=0; i < ARRAY_SIZE(tps65180_sys) && !err; i++)
		err = sysfs_create_bin_file(&client->dev.kobj, &(tps65180_sys[i]));

	err = device_create_file(&client->dev, &dev_attr_wakeup);

	if (client->dev.platform_data) {
		struct tps65180_platform_data *pdata = (struct tps65180_platform_data*)client->dev.platform_data;
		data->wakeup_pin = pdata->wakeup_pin;
		data->pwr_good_pin = pdata->pwr_good_pin;
		data->int_pin = pdata->int_pin;
	} else {
		data->wakeup_pin = -1;
		data->pwr_good_pin = -1;
		data->int_pin = -1;
	}

	/* Set up the VCOM Voltage */
	if (vcom_neg_mv > VCOM_NEG_MAX) {
		dev_err(&client->dev, SHORT_NAME " VCOM negative voltage, -%u mV, too low, using default\n", vcom_neg_mv);
		vcom_neg_mv = 1250;
	}

	/* PWR_GOOD is currently unused */
	err = gpio_request(data->pwr_good_pin, SHORT_NAME "_pwr_good");
	if (err) {
		printk(KERN_INFO SHORT_NAME " no pwr_good pin available.  Ignoring.\n");
		data->pwr_good_pin = -1;
	} else {
		gpio_direction_input(data->pwr_good_pin);
	}
	err = gpio_request(data->int_pin, SHORT_NAME "_int");
	if (err) {
		printk(KERN_INFO SHORT_NAME " no interrupt pin available.  Using Poll mode.\n");
		data->int_pin = -1;
		set_bit(F_POLL_MODE,&data->flags);
	} else {
		err = request_irq(gpio_to_irq(data->int_pin), tps65180_dispatch_irq,
				  IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				  SHORT_NAME "_int", NULL);
		if (err) {
			dev_err(&client->dev, "request_irq %d failed: %d.  Using Poll mode.\n", gpio_to_irq(data->int_pin), err);
			data->int_pin = -1;
			set_bit(F_POLL_MODE,&data->flags);
		}
	}
	if (gpio_request(data->wakeup_pin, SHORT_NAME "_wakeup")) {
		printk(KERN_INFO SHORT_NAME " no wakeup pin available.  Wakeup will only restore state.\n");
		data->wakeup_pin = -1;
	} else {
		/* Start it turned on to get the REVID */
		gpio_direction_output(data->wakeup_pin,1);
		/* wait to be sure the chip is ready */
		udelay(WAKEUP_DELAY_USECS);
	}

	revid = i2c_smbus_read_byte_data(client,R_REVID);
	if (gpio_is_valid(data->wakeup_pin))
		gpio_direction_output(data->wakeup_pin,0);

	if (R_REVID_VERSION(revid) == R_REVID_VERSION_TPS65180) {
		printk(KERN_INFO SHORT_NAME " found, version 0x%02x, using -%u mV for VCOM\n", revid, vcom_neg_mv);
		singleton = data;
		if (start_awake)
			tps65180_wakeup(1);
		err = 0;
	} else {
		tps65180_wakeup(0);
		dev_err(&client->dev, SHORT_NAME " not found when probed (REVID=0x%02x), removing\n", revid);
		tps65180_remove(client);
		err = -ENODEV;
	}

	return err;
}

static int tps65180_remove(struct i2c_client *client)
{
	struct tps65180_data *data = i2c_get_clientdata(client);
	int i;

	for(i=0; i < ARRAY_SIZE(tps65180_sys); i++)
		sysfs_remove_bin_file(&client->dev.kobj, &(tps65180_sys[i]));

	device_remove_file(&client->dev, &dev_attr_wakeup);

	tps65180_finish_work(data);

	if (!test_bit(F_POLL_MODE,&data->flags))
		free_irq(gpio_to_irq(data->int_pin), NULL);

	gpio_free(data->pwr_good_pin);
	gpio_free(data->int_pin);
	gpio_free(data->wakeup_pin);

	kfree(data);
	singleton = NULL;

	return 0;
}

/*----------------------- Driver ----------------------------------------- */

static const struct i2c_device_id tps65180_id[] = {
	{ SHORT_NAME, 0x48 },
	{ }
};

static struct i2c_driver tps65180_driver = {
	.driver = {
		.name	= SHORT_NAME,
	},
	.id_table	= tps65180_id,
	.probe		= tps65180_probe,
	.remove		= tps65180_remove,
};

static int __init tps65180_init(void)
{
	return i2c_add_driver(&tps65180_driver);
}

static void __exit tps65180_exit(void)
{
	i2c_del_driver(&tps65180_driver);
}


MODULE_AUTHOR("Rob Emanuele <rje@crystalfontz.com>");
MODULE_DESCRIPTION("TI TPS65180 driver");
MODULE_LICENSE("GPL");

subsys_initcall(tps65180_init);
module_exit(tps65180_exit);
