/*
 * BQ27500 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION			"1.0.0"

#define BQ27500_REG_TEMP		0x06
#define BQ27500_REG_VOLT		0x08
#define BQ27500_REG_RSOC		0x2C /* Relative State-of-Charge */
#define BQ27500_REG_AI			0x14
#define BQ27500_REG_FLAGS		0x0A
#define BQ27500_REG_RCAP		0x10

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27500_device_info;
struct bq27500_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27500_device_info *di);
};

struct bq27500_device_info {
	struct device 		*dev;
	int			id;
	int			voltage_uV;
	int			current_uA;
	int			temp_C;
	int			charge_rsoc;
	struct bq27500_access_methods	*bus;
	struct power_supply	bat;

	struct i2c_client	*client;
};

static enum power_supply_property bq27500_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

/*
 * Common code for BQ27x00 devices
 */

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27500_device_info *di)
{
	int ret;

	ret = di->bus->read(reg, rt_value, b_single, di);
	*rt_value = be16_to_cpu(*rt_value);

	return ret;
}

/*
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
static int bq27500_battery_temperature(struct bq27500_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27x00_read(BQ27500_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	return (temp >> 2) - 273;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27500_battery_voltage(struct bq27500_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27x00_read(BQ27500_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27500_battery_current(struct bq27500_device_info *di)
{
	int ret;
	int curr = 0;
	int flags = 0;

	ret = bq27x00_read(BQ27500_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}
	ret = bq27x00_read(BQ27500_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return 0;
	}
	if ((flags & (1 << 7)) != 0) {
		dev_dbg(di->dev, "negative current!\n");
		return -curr;
	}
	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27500_battery_rsoc(struct bq27500_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = bq27x00_read(BQ27500_REG_RSOC, &rsoc, 1, di);
	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return rsoc >> 8;
}

#define to_bq27500_device_info(x) container_of((x), \
				struct bq27500_device_info, bat);

static int bq27500_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27500_device_info *di = to_bq27500_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27500_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27500_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27500_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27500_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void bq27500_powersupply_init(struct bq27500_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27500_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27500_battery_props);
	di->bat.get_property = bq27500_battery_get_property;
	di->bat.external_power_changed = NULL;
}

/*
 * BQ27500 specific code
 */

static int bq27500_read(u8 reg, int *rt_value, int b_single,
			struct bq27500_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_be16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static int bq27500_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27500_device_info *di;
	struct bq27500_access_methods *bus;
	int num;
	int retval = 0;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "bq27500-%d", num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	/* for android
	di->bat.name = name;
	*/
	di->bat.name = "battery";

	bus->read = &bq27500_read;
	di->bus = bus;
	di->client = client;

	bq27500_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27500_battery_remove(struct i2c_client *client)
{
	struct bq27500_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id bq27500_id[] = {
	{ "bq27500", 0 },
	{},
};

static struct i2c_driver bq27500_battery_driver = {
	.driver = {
		.name = "bq27500-battery",
	},
	.probe = bq27500_battery_probe,
	.remove = bq27500_battery_remove,
	.id_table = bq27500_id,
};

static int __init bq27500_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27500_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27500 driver\n");

	return ret;
}
module_init(bq27500_battery_init);

static void __exit bq27500_battery_exit(void)
{
	i2c_del_driver(&bq27500_battery_driver);
}
module_exit(bq27500_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27500 battery monitor driver");
MODULE_LICENSE("GPL");
