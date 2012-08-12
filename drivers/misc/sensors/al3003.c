/* drivers/i2c/chips/al3003.c - al3003 light sensor driver
 *
 * Copyright (C) 2010 Gigabyte Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include<linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include "al3003.h"

/* #define DEBUG 1 */
/* #define AL3003_DEBUG 1 */

#define IR_LED_LEAK_CHECK 1 /* when ir leak, close P-Sensor */
#define AUTO_IR_LED_LEAK_CHECK 1
#define AL3003_EARLY_SUSPEND_CHECK 1
#ifdef IR_LED_LEAK_CHECK
static long ir_leak;
struct workqueue_struct	*queue_ir_leak;
struct work_struct		work_ir_leak;
#endif

#define YAMAHA_SENSORS 1
#ifdef YAMAHA_SENSORS
#define ALS_SENSOR_NAME		"light"
#define PXY_SENSOR_NAME		"proximity"
#define SENSOR_DEFAULT_DELAY	(200)   /* 200 ms */
#define SENSOR_MAX_DELAY		(2000)  /* 2000 ms */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)
#endif

static struct i2c_client *al3003_client;
/*
static long lux_table[64] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
	60, 61, 62, 63
};
*/
static long lux_table[64] = {
	0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
	100, 110, 130, 150, 170, 200, 250, 300, 350, 400,
	450, 500, 550, 600, 650, 700, 800, 900, 1000, 1500,
	2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500,
	7000, 7500, 8000, 8500, 9000, 9500, 10000, 10500, 11000, 12000,
	12500, 13000, 13396, 16082, 19307, 23178, 27826, 33405, 40103, 48144,
	57797, 69386, 83298, 100000,
};

struct al3003_data {
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
	struct mutex mutex;
#ifdef YAMAHA_SENSORS
	int enabled_als;
	int enabled_pxy;
	int delay_als;
	int delay_pxy;
#endif
};

static int data_pxy, data_als;
static short al3003_no_early_suspend;
static int al3003_early_suspend_check;

#ifdef YAMAHA_SENSORS
static struct input_dev *als_input_data;
static struct input_dev *pxy_input_data;
static int counter_als, counter_pxy;
#endif

static long get_lux_data(long);
static int al3003_i2c_rxdata(char*, int);
static long al3003_read_data(long);
static void al3003_suspend(void);
static void al3003_resume(void);
static int al3003_auto_set_threshold(void);

static int al3003_i2c_rxdata(char *rxData, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
		{
			.addr = al3003_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = al3003_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (retry = 0; retry <= 5; retry++) {
		if (i2c_transfer(al3003_client->adapter, msgs, 2) > 0) {
			#ifdef DEBUG
			printk(KERN_INFO "read data=%x %x %x %x\n", \
				rxData[0], rxData[1], rxData[2], rxData[3]);
			#endif
			break;
		} else
			msleep(30);
	}
	if (retry > 5) {
		printk(KERN_ERR "%s: retry over 5\n", __func__);
		return -EIO;
	} else
		return 0;
}

static int al3003_i2c_txdata(char *txData, int length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = al3003_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (retry = 0; retry <= 5; retry++) {
		if (i2c_transfer(al3003_client->adapter, msg, 1) > 0)
			break;
		else
			msleep(30);
	}
	if (retry > 5) {
		printk(KERN_ERR "%s: retry over 5\n", __func__);
		return -EIO;
	} else
		return 0;
}

static int al3003_initial_chip(char addr, char value)
{
	char buffer[2];
	int ret;

	buffer[0] = addr;
	buffer[1] = value;
	ret = al3003_i2c_txdata(buffer, 2);
	if (ret < 0)
		return -1;
	return 0;
}

#ifdef IR_LED_LEAK_CHECK
static void workqueue_ir_leak(struct work_struct *work_ptr)
{
	al3003_auto_set_threshold();
}

static void ir_leak_check(void)
{
	ir_leak = 0;
	queue_ir_leak = NULL;
	queue_ir_leak = create_singlethread_workqueue("queue_struct");
	if (!queue_ir_leak)
		return;
	INIT_WORK(&work_ir_leak, workqueue_ir_leak);
	queue_work(queue_ir_leak, &work_ir_leak);
}
#endif

static int al3003_init(void)
{
	int err;

	err = al3003_initial_chip(CONFIG, CONFIG_INITIAL);
	if (err < 0)
		return err;

	msleep(10);
	err = al3003_initial_chip(TIMING_CONTROL, TIMING_CONTROL_INITIAL);
	if (err < 0)
		return err;

	msleep(10);
	err = al3003_initial_chip(ALS_CONTROL, ALS_CONTROL_INITIAL);
	if (err < 0)
		return err;

	msleep(10);
	err = al3003_initial_chip(ALS_WINDOW, ALS_WINDOW_INITIAL);
	if (err < 0)
		return err;

	msleep(10);
	err = al3003_initial_chip(PXY_CONTROL, PXY_CONTROL_INITIAL);
	if (err < 0)
		return err;

	return 0;
}

static long get_lux_data(long data)
{
	int i;

	for (i = 0; i < sizeof(lux_table); i++) {
		if (data != i)
			continue;
		return lux_table[i];
	}
	return -1;
}

static long al3003_read_data(long data)
{
	char buff;
	int ret;

	buff = AL3003_DATA;
	ret = al3003_i2c_rxdata(&buff, 1);
	if (ret < 0)
		return ret;

	if (data == AL3003_READ_DATA_PXY) {
#ifdef IR_LED_LEAK_CHECK
		if (ir_leak)
			data = 0;
		else
#endif
			data = (long) data_read_pxy(buff);
	} else
		data = (long) data_read_als(buff);

	return data;
}

static int al3003_set_mode(short mode)
{
	char buffer[2];
	int ret;

	buffer[0] = CONFIG;
	ret = al3003_i2c_rxdata(buffer, 1);
	if (ret < 0)
		return -1;
	switch (mode) {
	case	AL3003_MODE_POWER_UP:
		buffer[1] = config_mode_power_up(buffer[0]);
		break;
	case	AL3003_MODE_POWER_DOWN:
		buffer[1] = CONFIG_MODE_POWER_DOWN;
		break;
	case	AL3003_MODE_RESET:
		buffer[1] = CONFIG_MODE_RESET;
		break;
	case	AL3003_MODE_ALS_ACTIVE:
		buffer[1] = CONFIG_MODE_ALS_ACTIVE;
		break;
	case	AL3003_MODE_PXY_ACTIVE:
		buffer[1] = CONFIG_MODE_PXY_ACTIVE;
		break;
	case	AL3003_MODE_ALL_ACTIVE:
		buffer[1] = CONFIG_MODE_ALL_ACTIVE;
		break;
	case	AL3003_MODE_IDLE:
		buffer[1] = CONFIG_MODE_IDLE;
		break;
	default:
		break;
	}
	buffer[0] = CONFIG;
	ret = al3003_i2c_txdata(buffer, 2);
#ifdef AL3003_DEBUG
	printk(KERN_ERR "al3003: %s set mode = %d\n", __func__, buffer[1]);
#endif
	return ret;
}

static int al3003_set_loss(short data)
{
	short loss1[16] = {0, 17, 31, 42, 52, 60, 65, 67, 72, 81,
				84, 86, 89, 91, 92, 94};
	short loss2[16] = {
		ALS_WINDOW_LOSS_0_PERCENT, ALS_WINDOW_LOSS_17_PERCENT,
		ALS_WINDOW_LOSS_31_PERCENT, ALS_WINDOW_LOSS_42_PERCENT,
		ALS_WINDOW_LOSS_52_PERCENT, ALS_WINDOW_LOSS_60_PERCENT,
		ALS_WINDOW_LOSS_65_PERCENT, ALS_WINDOW_LOSS_67_PERCENT,
		ALS_WINDOW_LOSS_72_PERCENT, ALS_WINDOW_LOSS_81_PERCENT,
		ALS_WINDOW_LOSS_84_PERCENT, ALS_WINDOW_LOSS_86_PERCENT,
		ALS_WINDOW_LOSS_89_PERCENT, ALS_WINDOW_LOSS_91_PERCENT,
		ALS_WINDOW_LOSS_92_PERCENT, ALS_WINDOW_LOSS_94_PERCENT
	};
	char buffer[2];
	int ret;
	int i = 0;

	buffer[0] = ALS_WINDOW;
	buffer[1] = ALS_WINDOW_LOSS_0_PERCENT;
	if (data >= 94)
		buffer[1] = ALS_WINDOW_LOSS_94_PERCENT;
	else{
		for (i = 0; i < 16; i++) {
			if (data < loss1[i])
				break;
			buffer[1] = loss2[i];
		}
	}

	ret = al3003_i2c_txdata(&buffer[0], 2);
	if (ret < 0)
		return ret;
	return ret;
}

static int al3003_set_threshold(short data)
{
	char buffer[2];
	int ret;

	if (data > 31 || data < 0)
		return -1;
	buffer[0] = PXY_CONTROL;
	buffer[1] = data + (0x40);
	ret = al3003_i2c_txdata(buffer, 2);
	if (ret < 0)
		return ret;
	return 0;
}

static int al3003_get_int(void)
{
	int ret;
	/* ret = gpio_get_value(pdata->intr); */
	ret = gpio_get_value(AL3003_INTERRUPT_GPIO);
	return ret;
}

static int al3003_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int al3003_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int al3003_ioctl(struct inode *inode, struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	char rwbuf[8];
	int ret = -1;
	short temp;
	long *p = (long *)arg;
	long data = 0;

	switch (cmd) {
	case AL3003_IOCTL_READ_DATA:
		data = (long)*p;
		break;
	case AL3003_IOCTL_READ:
	case AL3003_IOCTL_WRITE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case AL3003_IOCTL_NO_EARLY_SUSPEND:
	case AL3003_IOCTL_SET_MODE:
	case AL3003_IOCTL_SET_LOSS:
#ifdef IR_LED_LEAK_CHECK
	case AL3003_IOCTL_P_SENSOR_FORCE_ENABLE:
#endif
	case AL3003_IOCTL_SET_THRESHOLD:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		temp = (short)rwbuf[0];
		printk(KERN_INFO "%s: get input data = %d\n", __func__, temp);
		break;
	default:
		break;
	}

	switch (cmd) {
	case AL3003_IOCTL_INITIAL:
		ret = al3003_init();
		if (ret < 0)
			return ret;
		break;
	case AL3003_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		ret = al3003_i2c_txdata(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case AL3003_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		ret = al3003_i2c_rxdata(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case AL3003_IOCTL_READ_DATA:
		data = get_lux_data(al3003_read_data(data));
		break;
	case AL3003_IOCTL_NO_EARLY_SUSPEND:
		al3003_no_early_suspend = temp;
		break;
#ifdef IR_LED_LEAK_CHECK
	case AL3003_IOCTL_P_SENSOR_FORCE_ENABLE:
		ir_leak = (long)(temp > 0 ? 0 : 1);
		break;
#endif
	case AL3003_IOCTL_SET_MODE:
		al3003_set_mode(temp);
		break;
	case AL3003_IOCTL_SET_LOSS:
		al3003_set_loss(temp);
		break;
	case AL3003_IOCTL_SET_THRESHOLD:
		al3003_set_threshold(temp);
		break;
	case AL3003_IOCTL_GET_INT:
		temp = al3003_get_int();
		break;
	case AL3003_IOCTL_SYSTEM_SUSPEND:
		al3003_set_mode(AL3003_MODE_IDLE);
		break;
	case AL3003_IOCTL_SYSTEM_RESUME:
		al3003_resume();
		break;
	case AL3003_IOCTL_AUTO_SET_THRESHOLD:
		temp = al3003_auto_set_threshold();
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case AL3003_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case AL3003_IOCTL_READ_DATA:
		if (copy_to_user(argp, &data, sizeof(data)))
			return -EFAULT;
		break;
	case AL3003_IOCTL_GET_INT:
	case AL3003_IOCTL_AUTO_SET_THRESHOLD:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static int al3003_auto_set_threshold(void)
{
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
	int err = -1;
	int i = 0;
	int mode = AL3003_MODE_IDLE;
	char buff;

#ifdef IR_LED_LEAK_CHECK
	ir_leak = -1;
#endif

	/* enable p-sensor*/
	if (!data->enabled_als && !data->enabled_pxy) {
		mode = AL3003_MODE_IDLE;
		err = al3003_set_mode(AL3003_MODE_PXY_ACTIVE);
		if (err < 0)
			return err;
	} else if (!data->enabled_als && data->enabled_pxy) {
		mode = AL3003_MODE_PXY_ACTIVE;
	} else if (data->enabled_als && data->enabled_pxy) {
		mode = AL3003_MODE_ALL_ACTIVE;
	} else {
		mode = AL3003_MODE_ALS_ACTIVE;
		err = al3003_set_mode(AL3003_MODE_ALL_ACTIVE);
		if (err < 0)
			return err;
	}

	while (i < 10) {
		/* set p-sensor threshold level = 0 */
		msleep(40);
		al3003_initial_chip(PXY_CONTROL, (0x40 + i));

		i++;
		msleep(40);

		buff = AL3003_DATA;
		err = al3003_i2c_rxdata(&buff, 1);
		if (err == 0) /* read reg ok */
			err = (int) data_read_pxy(buff); /* get pxy data */

		if (!err) { /* pxy == 0 , no ir leak */
			msleep(40);
			al3003_initial_chip(PXY_CONTROL, (0x40 + i + 5));
			printk(KERN_ERR "%s: set threshold level = %d\n",
				__func__, (i + 5));
#ifdef IR_LED_LEAK_CHECK
			ir_leak = 0;
#endif
			break;
		}
	}

	printk(KERN_ERR "%s: ir_leak = %d\n", __func__, (int)ir_leak);

	/* diable p-sensor*/
	msleep(40);
	al3003_set_mode(mode);
	i += 5;

	return i;
}

#ifdef YAMAHA_SENSORS
/* Sysfs interface */
static ssize_t al3003_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
	int delay;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	mutex_lock(&data->mutex);
	if (strcmp(input_data->name, "light") == 0)
		delay = data->delay_als;
	else
		delay = data->delay_pxy;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

static ssize_t al3003_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
	long value, err;
	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	if (value < 0)
		return count;

	if (SENSOR_MAX_DELAY < value)
		value = SENSOR_MAX_DELAY;

	mutex_lock(&data->mutex);
	if (strcmp(input_data->name, "light") == 0) {
		data->delay_als = value;
		if (!al3003_early_suspend_check)
		input_report_abs(input_data, ABS_CONTROL_REPORT,
			(data->enabled_als<<16) | value);
	} else {
		data->delay_pxy = value;
		if (!al3003_early_suspend_check)
		input_report_abs(input_data, ABS_CONTROL_REPORT,
			(data->enabled_pxy<<16) | value);
	}
	mutex_unlock(&data->mutex);

	printk(KERN_INFO "%s: value=%d\n", __func__, (int)value);
	return count;
}

static ssize_t al3003_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
	int enabled;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	mutex_lock(&data->mutex);
	if (strcmp(input_data->name, "light") == 0)
		enabled = data->enabled_als;
	else
		enabled = data->enabled_pxy;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

static struct private_lock
{
	struct wake_lock wlock;
	int count;
}pxy_lock;

static void wake_lock_pxy(void)
{
	if(pxy_lock.count)
		return;
	else
	{
		pxy_lock.count++;
		wake_lock(&pxy_lock.wlock);
	}
}

static void wake_unlock_pxy(void)
{
	if(pxy_lock.count)
	{
		pxy_lock.count--;
		wake_unlock(&pxy_lock.wlock);
	}
	else
		return;
}

static ssize_t al3003_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
	long value, err;
	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	if (value != 0 && value != 1)
		return count;

	mutex_lock(&data->mutex);
	if (strcmp(input_data->name, "light") == 0)
		data->enabled_als = (int)value;
	else
	{
		data->enabled_pxy = (int)value;

		if(data->enabled_pxy)
			wake_lock_pxy();
		else		
			wake_unlock_pxy();
	}
		
	if (!data->enabled_als && !data->enabled_pxy && !value)
		al3003_suspend();
	if (data->enabled_als || data->enabled_pxy)
		al3003_resume();

	if (strcmp(input_data->name, "light") == 0) {
		if (!al3003_early_suspend_check)
		input_report_abs(input_data, ABS_CONTROL_REPORT,
			(value<<16) | data->delay_als);
	} else {
		if (!al3003_early_suspend_check)
		input_report_abs(input_data, ABS_CONTROL_REPORT,
			(value<<16) | data->delay_pxy);
	}
	mutex_unlock(&data->mutex);

	printk(KERN_INFO "%s set enabled als=%d   pxy=%d\n",
		__func__, data->enabled_als, data->enabled_pxy);
	return count;
}

static ssize_t al3003_wake_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

static ssize_t al3003_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);

	struct al3003_data *data = i2c_get_clientdata(al3003_client);
	char buff = AL3003_DATA;
	int ret;

#ifdef DEBUG
	printk(KERN_INFO "%s: %s\n", input_data->name, __func__);
#endif

	if (data == NULL)
		return -1;

#ifdef AL3003_EARLY_SUSPEND_CHECK
	if (data->enabled_als == 0)
		counter_als = 0;
	if (counter_als > 5 || counter_pxy > 5) {
#endif
		ret = al3003_i2c_rxdata(&buff, 1);
		if (ret < 0)
			return -1;

		msleep(10);
		data_als = (int) get_lux_data(data_read_als(buff));
#ifdef IR_LED_LEAK_CHECK
		if (ir_leak)
			data_pxy = 0;
		else
#endif
			data_pxy = (int) data_read_pxy(buff);
#ifdef AL3003_EARLY_SUSPEND_CHECK
	}
#endif

	if (strcmp(input_data->name, "light") == 0) {
#ifdef AL3003_EARLY_SUSPEND_CHECK
		if (counter_als > 5) {
			counter_als = 0;
			if (!al3003_early_suspend_check)
#endif
				input_report_abs(input_data, ABS_X, data_als);
#ifdef AL3003_EARLY_SUSPEND_CHECK
		} else
			counter_als++;
#endif
		return sprintf(buf, "%d\n", data_als);
	} else {
#ifdef AL3003_EARLY_SUSPEND_CHECK
		if (counter_pxy > 5) {
			counter_pxy = 0;
			if (!al3003_early_suspend_check)
#endif
				input_report_abs(input_data, ABS_X, data_pxy);
#ifdef AL3003_EARLY_SUSPEND_CHECK
		} else
			counter_pxy++;
#endif
		return sprintf(buf, "%d\n", data_pxy);
	}
}

static ssize_t al3003_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int status;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	spin_lock_irqsave(&input_data->event_lock, flags);
	status = input_data->abs[ABS_STATUS];
	spin_unlock_irqrestore(&input_data->event_lock, flags);

	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
	al3003_delay_show, al3003_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
	al3003_enable_show, al3003_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
	NULL, al3003_wake_store);
static DEVICE_ATTR(data, S_IRUGO, al3003_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, al3003_status_show, NULL);

static struct attribute *al3003_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group al3003_attribute_group = {
	.attrs = al3003_attributes
};

static int regist_input_device(struct al3003_data *data, int kind)
{
	struct input_dev *input_data = NULL;
	int input_registered = 0;
	int err = -1;

	input_data = input_allocate_device();
	if (!input_data) {
		err = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input_data device\n",
			__func__);
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);
	input_set_capability(input_data, EV_ABS, ABS_STATUS); /* status */
	input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
	input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT);
		/* enabled/delay */
	if (kind == 1)
		input_data->name = ALS_SENSOR_NAME;
	else
		input_data->name = PXY_SENSOR_NAME;

	err = input_register_device(input_data);
	if (err) {
		printk(KERN_ERR "%s: Unable to register input_data device: %s\n",
			__func__, input_data->name);
		goto err;
	}
	input_set_drvdata(input_data, data);
	input_registered = 1;

	err = sysfs_create_group(&input_data->dev.kobj,
		&al3003_attribute_group);
	if (err) {
		printk(KERN_ERR
			"%s: sysfs_create_group failed[%s]\n",
			__func__, input_data->name);
		goto err;
	}

	if (kind == 1)
		als_input_data = input_data;
	else
		pxy_input_data = input_data;
	return 0;
err:
	if (input_data != NULL) {
		if (input_registered)
			input_unregister_device(input_data);
		else
			input_free_device(input_data);
		input_data = NULL;
	}
	return err;
}
#endif

static void al3003_suspend(void)
{
	al3003_set_mode(AL3003_MODE_IDLE);
}

static void al3003_resume(void)
{
	struct al3003_data *data = i2c_get_clientdata(al3003_client);

	if (data->enabled_als && data->enabled_pxy)
		al3003_set_mode(AL3003_MODE_ALL_ACTIVE);
	else if (data->enabled_als)
		al3003_set_mode(AL3003_MODE_ALS_ACTIVE);
	else if (data->enabled_pxy)
		al3003_set_mode(AL3003_MODE_PXY_ACTIVE);
	else
		al3003_set_mode(AL3003_MODE_IDLE);
}

static void al3003_early_suspend(struct early_suspend *handler)
{
#ifdef YAMAHA_SENSORS
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
#endif

	if (al3003_no_early_suspend)
		return;

#ifdef YAMAHA_SENSORS
#ifdef AL3003_EARLY_SUSPEND_CHECK
	al3003_early_suspend_check = 1;
#endif
	mutex_lock(&data->mutex);
	if (data->enabled_als || data->enabled_pxy)
		al3003_suspend();
	mutex_unlock(&data->mutex);
#else
	al3003_suspend();
#endif
	printk(KERN_ERR "al3003: %s\n", __func__);
}

static void al3003_early_resume(struct early_suspend *handler)
{
#ifdef YAMAHA_SENSORS
	struct al3003_data *data = i2c_get_clientdata(al3003_client);
#endif
	if (al3003_no_early_suspend)
		return;
#ifdef YAMAHA_SENSORS
	mutex_lock(&data->mutex);
	if (data->enabled_als || data->enabled_pxy)
		al3003_resume();
	mutex_unlock(&data->mutex);
#else
	al3003_resume();
#endif
	al3003_early_suspend_check = 0;
	printk(KERN_ERR "al3003: %s\n", __func__);
}

static const struct file_operations al3003_fops = {
	.owner = THIS_MODULE,
	.open = al3003_open,
	.release = al3003_release,
	.ioctl = al3003_ioctl,
};

static struct miscdevice al3003_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "al3003",
	.fops = &al3003_fops,
};

static int al3003_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct al3003_data *data;
	int err = -1;
#ifdef YAMAHA_SENSORS
	int input_registered = 0, sysfs_created = 0;
	als_input_data = NULL;
	pxy_input_data = NULL;
	counter_als = 0;
	counter_pxy = 0;
#endif
	al3003_client = NULL;
	data_als = 0;
	data_pxy = 0;
	al3003_no_early_suspend = 0;
	al3003_early_suspend_check = 0;

	printk(KERN_INFO "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto init_err;
	}
	printk(KERN_ERR "%s: i2c_check_functionality ok!!\n", __func__);

	data = kzalloc(sizeof(struct al3003_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto init_err;
	}

	i2c_set_clientdata(client, data);
	al3003_client = client;

	err = al3003_init();
	if (err < 0) {
		printk(KERN_ERR "%s: al3003_init failed\n", __func__);
		goto err;
	}
	printk(KERN_ERR "%s: al3003_init ok!!\n", __func__);

	err = misc_register(&al3003_device);
	if (err) {
		printk(KERN_ERR "%s: device register failed\n", __func__);
		goto err;
	}
	printk(KERN_ERR "%s: misc_register ok!!\n", __func__);

	al3003_client = client;
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	data->early_suspend.suspend = al3003_early_suspend;
	data->early_suspend.resume = al3003_early_resume;
	register_early_suspend(&data->early_suspend);

#ifdef YAMAHA_SENSORS
	data->enabled_als = 0;
	data->enabled_pxy = 0;
	data->delay_als = SENSOR_DEFAULT_DELAY;
	data->delay_pxy = SENSOR_DEFAULT_DELAY;
	err = regist_input_device(data, 1); /* create als sysfs_create_group */
	if (err < 0)
		goto err;
	printk(KERN_ERR "%s: regist_input_device1 ok!!\n", __func__);
	input_registered = 1;
	sysfs_created = 1;
	err = regist_input_device(data, 2); /* create pxy sysfs_create_group */
	if (err < 0)
		goto err;
	printk(KERN_ERR "%s: regist_input_device2 ok!!\n", __func__);
	mutex_init(&data->mutex);
#endif

#ifdef AUTO_IR_LED_LEAK_CHECK
	ir_leak_check();
#endif
	pxy_lock.count = 0;
	wake_lock_init(&pxy_lock.wlock,WAKE_LOCK_SUSPEND, "proximity wake lock");
	
	return 0;

err:
#ifdef YAMAHA_SENSORS
	if (data != NULL) {
		if (als_input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&als_input_data->dev.kobj,
					&al3003_attribute_group);
			}
			if (input_registered)
				input_unregister_device(als_input_data);
			else
				input_free_device(als_input_data);
			als_input_data = NULL;
		}
		kfree(data);
	}
#else
	kfree(data);
#endif
init_err:
	return err;
}

static int al3003_remove(struct i2c_client *client)
{
	struct al3003_data *data = i2c_get_clientdata(client);

#ifdef YAMAHA_SENSORS
	if (als_input_data != NULL) {
		sysfs_remove_group(&als_input_data->dev.kobj,
			&al3003_attribute_group);
		input_unregister_device(als_input_data);
	}
	if (pxy_input_data != NULL) {
		sysfs_remove_group(&pxy_input_data->dev.kobj,
			&al3003_attribute_group);
		input_unregister_device(pxy_input_data);
	}
#endif
	wake_lock_destroy(&pxy_lock.wlock);
	
	i2c_unregister_device(client);
	misc_deregister(&al3003_device);
	kfree(data);
	return 0;
}

static struct i2c_device_id al3003_idtable[] = {
	{ "al3003", 0 },
	{ }
};

static struct i2c_driver al3003_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "al3003"
	},
	.id_table		= al3003_idtable,
	.probe		= al3003_probe,
	.remove		= al3003_remove,
};

static int __init al3003_i2c_init(void)
{
	printk(KERN_INFO "%s: light sensor driver init\n", __func__);
	return i2c_add_driver(&al3003_driver);
}

static void __exit al3003_i2c_exit(void)
{
	printk(KERN_INFO "%s: light sensor driver exit\n", __func__);
	i2c_del_driver(&al3003_driver);
}

module_init(al3003_i2c_init);
module_exit(al3003_i2c_exit);
MODULE_DEVICE_TABLE(i2c, al3003_idtable);
MODULE_DESCRIPTION("AL3003 LightSensor Driver");
MODULE_LICENSE("GPL");
