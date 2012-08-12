/* drivers/i2c/chips/al3002.c - al3002 light sensor driver
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
#include <linux/input.h>
#include <linux/al3002.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include<linux/earlysuspend.h>
/* #define DEBUG */

static struct i2c_client *this_client;
static int gpio = 28;
static long lux_table[64] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
	60, 61, 62, 63
};

struct al3002_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend;
};

static int als_i2c_rxdata(char *rxData, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (retry = 0; retry <= 100; retry++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			#ifdef DEBUG
			printk(KERN_ERR "read data=%x %x %x %x\n", \
				rxData[0], rxData[1], rxData[2], rxData[3]);
			#endif
			break;
	} else
			mdelay(10);
	}
	if (retry > 100) {
		printk(KERN_ERR "%s: retry over 100\n", __func__);
		return -EIO;
	} else
		return 0;
}

static int als_i2c_txdata(char *txData, int length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (retry = 0; retry <= 100; retry++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;
		else
			mdelay(10);
	}
	if (retry > 100) {
		printk(KERN_ERR "%s: retry over 100\n", __func__);
		return -EIO;
	} else
		return 0;
}

static int als_init(void)
{
	char buffer[2];
	int ret;

	buffer[0] = CONFIG;
	buffer[1] = CONFIG_INITIAL;
	ret = als_i2c_txdata(buffer, 2);
	if (ret < 0)
		return -1;
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

static long als_read_data(long data)
{
	char buff;
	int ret;

	buff = ALS_DATA;
	ret = als_i2c_rxdata(&buff, 1);
	if (ret < 0)
		return ret;

	if (data == ALS_READ_DATA_PXY)
		data = (long) data_read_pxy(buff);
	else
		data = (long) data_read_als(buff);

	return data;
}

static int als_set_mode(short mode)
{
	char buffer[2];
	int ret;

	buffer[0] = CONFIG;
	ret = als_i2c_rxdata(buffer, 1);
	if (ret < 0)
		return -1;
	switch (mode) {
	case	ALS_MODE_POWER_UP:
		buffer[1] = config_mode_power_up(buffer[0]);
		break;
	case	ALS_MODE_POWER_DOWN:
		buffer[1] = config_mode_power_down(buffer[0]);
		break;
	case	ALS_MODE_RESET:
		buffer[1] = config_mode_reset(buffer[0]);
		break;
	case	ALS_MODE_ALS_ACTIVE:
		buffer[1] = config_mode_als_active(buffer[0]);
		break;
	case	ALS_MODE_PXY_ACTIVE:
		buffer[1] = config_mode_pxy_active(buffer[0]);
		break;
	case	ALS_MODE_ALL_ACTIVE:
		buffer[1] = config_mode_all_active(buffer[0]);
		break;
	case	ALS_MODE_IDLE:
		buffer[1] = config_mode_idle(buffer[0]);
		break;
	default:
		break;
	}
	buffer[0] = CONFIG;
	ret = als_i2c_txdata(buffer, 2);
	return ret;
}

static int als_set_loss(short data)
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

	ret = als_i2c_txdata(&buffer[0], 2);
	if (ret < 0)
		return ret;
	return ret;
}

static int als_set_threshold(short data)
{
	char buffer[2];
	int ret;

	if (data > 31 || data < 0)
		return -1;
	buffer[0] = ALS_CONTROL;
	ret = als_i2c_rxdata(&buffer[0], 1);
	if (ret < 0)
		return ret;
	buffer[1] = als_control_low_lux_threshold(buffer[0], (char)data);
	buffer[0] = ALS_CONTROL;
	ret = als_i2c_txdata(&buffer[0], 2);
	if (ret < 0)
		return ret;
	return ret;
}

static int als_get_int(void)
{
	int ret;
	/* ret = gpio_get_value(pdata->intr); */
	ret = gpio_get_value(gpio);
	return ret;
}

static int als_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int als_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int als_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	char rwbuf[8];
	int ret = -1;
	short temp;
	long *p = (long *)arg;
	long data = 0;

	switch (cmd) {
	case ALS_IOCTL_READ_DATA:
		data = (long)*p;
		break;
	case ALS_IOCTL_READ:
	case ALS_IOCTL_WRITE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ALS_IOCTL_SET_MODE:
	case ALS_IOCTL_SET_LOSS:
	case ALS_IOCTL_SET_THRESHOLD:
		temp = (short)arg;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ALS_IOCTL_INITIAL:
		ret = als_init();
		if (ret < 0)
			return ret;
		break;
	case ALS_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		ret = als_i2c_txdata(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case ALS_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		ret = als_i2c_rxdata(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case ALS_IOCTL_READ_DATA:
		data = get_lux_data(als_read_data(data));
		break;
	case ALS_IOCTL_SET_MODE:
		als_set_mode(temp);
		break;
	case ALS_IOCTL_SET_LOSS:
		als_set_loss(temp);
		break;
	case ALS_IOCTL_SET_THRESHOLD:
		als_set_threshold(temp);
		break;
	case ALS_IOCTL_GET_INT:
		temp = als_get_int();
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ALS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ALS_IOCTL_READ_DATA:
		if (copy_to_user(argp, &data, sizeof(data)))
			return -EFAULT;
		break;
	case ALS_IOCTL_GET_INT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static void al3002_early_suspend(struct early_suspend *handler)
{
	als_set_mode(ALS_MODE_IDLE);
	return;
}

static void al3002_early_resume(struct early_suspend *handler)
{
	als_set_mode(ALS_MODE_ALL_ACTIVE);
	return;
}

static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
	.ioctl = als_ioctl,
};

static struct miscdevice als_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "al3002",
	.fops = &als_fops,
};

static int al3002_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct al3002_data *als;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	als = kzalloc(sizeof(struct al3002_data), GFP_KERNEL);
	if (!als) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, als);

	this_client = client;

	err = als_init();
	if (err < 0) {
		printk(KERN_ERR "al3002_probe: als_init failed\n");
		goto exit_init_failed;
	}

	err = misc_register(&als_device);
	if (err) {
		printk(KERN_ERR "al3002_probe: device register failed\n");
		goto exit_misc_device_register_failed;
	}

	als->early_suspend.suspend = al3002_early_suspend;
	als->early_suspend.resume = al3002_early_resume;
	register_early_suspend(&als->early_suspend);

	return 0;

exit_misc_device_register_failed:
exit_init_failed:
	kfree(als);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int al3002_remove(struct i2c_client *client)
{
	struct al3002_data *als = i2c_get_clientdata(client);

	i2c_unregister_device(client);
	misc_deregister(&als_device);
	kfree(als);
	return 0;
}

static struct i2c_device_id al3002_idtable[] = {
	{ "al3002", 0 },
	{ }
};

static struct i2c_driver al3002_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "al3002"
	},
	.id_table	= al3002_idtable,
	.probe		= al3002_probe,
	.remove		= al3002_remove,
};

static int __init al3002_init(void)
{
	printk(KERN_INFO "AL3002 light sensor driver: init\n");
	return i2c_add_driver(&al3002_driver);
}

static void __exit al3002_exit(void)
{
	printk(KERN_INFO "AL3002 light sensor driver: exit\n");
	i2c_del_driver(&al3002_driver);
}

module_init(al3002_init);
module_exit(al3002_exit);
MODULE_DEVICE_TABLE(i2c, al3002_idtable);
MODULE_DESCRIPTION("AL3002 LightSensor Driver");
MODULE_LICENSE("GPL");
