/*  $Date: 2009/11/10 17:37:35 $
 *  $Revision: 1.0 $
*/

/*
* Copyright (C) 2009 Bosch Sensortec GmbH
*
*	BMA220 linux driver
*
* Usage:	BMA220 driver by i2c for linux
*
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in
  compliance with the License and the following stipulations. The Apache License , Version 2.0 is applicable unless
  otherwise stated by the stipulations of the disclaimer below.

* You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0


Disclaimer

 * Common:
 * This Work is developed for the consumer goods industry. It may only be used
 * within the parameters of the respective valid product data sheet.  The Work
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * It is not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * the Work is not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of the Work are at the purchaser own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any Work or Derivative Work use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased Work and Derivative Works, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This Work and any related information (hereinafter called "Information") is provided free of charge
 * for the sole purpose to support your application work. The Woek and Information is subject to the
 * following terms and conditions:
 *
 * The Work is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Work or Derivative Works if you do not have the
 * proper experience or training. Do not use this Work or Derivative Works fot other products than Bosch Sensortec products.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 */

/*! \file bma220_driver.c
    \brief This file contains all function implementations for the BMA220 in linux

    Details.
*/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include "bma220.h"
#include<linux/earlysuspend.h>
#include <linux/mutex.h>

#define BMA220_MAJOR	101
#define BMA220_MINOR	0

#define BMA220_IOC_MAGIC 'B'

#define BMA220_SET_SLEEP_EN \
	_IOWR(BMA220_IOC_MAGIC, 0, unsigned char)
#define BMA220_SET_SUSPEND \
	_IOW(BMA220_IOC_MAGIC, 1, unsigned char)
#define BMA220_READ_ACCEL_X \
	_IOWR(BMA220_IOC_MAGIC, 2, signed char)
#define BMA220_READ_ACCEL_Y \
	_IOWR(BMA220_IOC_MAGIC, 3, signed char)
#define BMA220_READ_ACCEL_Z \
	_IOWR(BMA220_IOC_MAGIC, 4, signed char)
#define BMA220_SET_MODE \
	_IOWR(BMA220_IOC_MAGIC, 5, unsigned char)
#define BMA220_GET_MODE \
	_IOWR(BMA220_IOC_MAGIC, 6, unsigned char)
#define BMA220_SET_RANGE \
	_IOWR(BMA220_IOC_MAGIC, 7, unsigned char)
#define BMA220_GET_RANGE \
	_IOWR(BMA220_IOC_MAGIC, 8, unsigned char)
#define BMA220_SET_BANDWIDTH \
	_IOWR(BMA220_IOC_MAGIC, 9, unsigned char)
#define BMA220_GET_BANDWIDTH \
	_IOWR(BMA220_IOC_MAGIC, 10, unsigned char)
#define BMA220_SET_SC_FILT_CONFIG \
	_IOWR(BMA220_IOC_MAGIC, 11, unsigned char)
#define BMA220_RESET_INTERRUPT \
	_IO(BMA220_IOC_MAGIC, 12)
#define BMA220_GET_DIRECTION_STATUS_REGISTER \
	_IOWR(BMA220_IOC_MAGIC, 13, unsigned char)
#define BMA220_GET_INTERRUPT_STATUS_REGISTER \
	_IOWR(BMA220_IOC_MAGIC, 14, unsigned char)
#define BMA220_SOFT_RESET \
	_IO(BMA220_IOC_MAGIC, 15)
#define BMA220_SET_LATCH_INT \
	_IOWR(BMA220_IOC_MAGIC, 16, unsigned char)
#define BMA220_SET_EN_HIGH_XYZ \
	_IOWR(BMA220_IOC_MAGIC, 17, unsigned char)
#define BMA220_SET_HIGH_TH \
	_IOWR(BMA220_IOC_MAGIC, 18, unsigned char)
#define BMA220_SET_HIGH_HY \
	_IOWR(BMA220_IOC_MAGIC, 19, unsigned char)
#define BMA220_SET_HIGH_DUR \
	_IOWR(BMA220_IOC_MAGIC, 20, unsigned char)
#define BMA220_SET_EN_LOW \
	_IOWR(BMA220_IOC_MAGIC, 21, unsigned char)
#define BMA220_SET_LOW_TH \
	_IOWR(BMA220_IOC_MAGIC, 22, unsigned char)
#define BMA220_SET_LOW_HY \
	_IOWR(BMA220_IOC_MAGIC, 23, unsigned char)
#define BMA220_SET_LOW_DUR \
	_IOWR(BMA220_IOC_MAGIC, 24, unsigned char)
#define BMA220_SET_SERIAL_HIGH_BW \
	_IOWR(BMA220_IOC_MAGIC, 25, unsigned char)
#define BMA220_READ_ACCEL_XYZ \
	_IOWR(BMA220_IOC_MAGIC, 26, signed char)
#define BMA220_SET_EN_ORIENT \
	_IOWR(BMA220_IOC_MAGIC, 27, unsigned char)
#define BMA220_SET_ORIENT_EX \
	_IOWR(BMA220_IOC_MAGIC, 28, unsigned char)
#define BMA220_GET_ORIENTATION \
	_IOWR(BMA220_IOC_MAGIC, 29, unsigned char)
#define BMA220_SET_EN_TT_XYZ \
	_IOWR(BMA220_IOC_MAGIC, 30, unsigned char)
#define BMA220_SET_TT_TH \
	_IOWR(BMA220_IOC_MAGIC, 31, unsigned char)
#define BMA220_SET_TT_DUR \
	_IOWR(BMA220_IOC_MAGIC, 32, unsigned char)
#define BMA220_SET_TT_FILT \
	_IOWR(BMA220_IOC_MAGIC, 33, unsigned char)
#define BMA220_SET_EN_SLOPE_XYZ \
	_IOWR(BMA220_IOC_MAGIC, 34, unsigned char)
#define BMA220_SET_EN_DATA \
	_IOWR(BMA220_IOC_MAGIC, 35, unsigned char)
#define BMA220_SET_SLOPE_TH \
	_IOWR(BMA220_IOC_MAGIC, 36, unsigned char)
#define BMA220_SET_SLOPE_DUR \
	_IOWR(BMA220_IOC_MAGIC, 37, unsigned char)
#define BMA220_SET_SLOPE_FILT \
	_IOWR(BMA220_IOC_MAGIC, 38, unsigned char)
#define BMA220_SET_CAL_TRIGGER \
	_IOWR(BMA220_IOC_MAGIC, 39, unsigned char)
#define BMA220_GET_CAL_RDY \
	_IOWR(BMA220_IOC_MAGIC, 40, unsigned char)
#define BMA220_SET_HP_XYZ_EN \
	_IOWR(BMA220_IOC_MAGIC, 41, unsigned char)
#define BMA220_SET_OFFSET_TARGET_X \
	_IOWR(BMA220_IOC_MAGIC, 42, unsigned char)
#define BMA220_SET_OFFSET_TARGET_Y \
	_IOWR(BMA220_IOC_MAGIC, 43, unsigned char)
#define BMA220_SET_OFFSET_TARGET_Z \
	_IOWR(BMA220_IOC_MAGIC, 44, unsigned char)
#define BMA220_SET_SLEEP_DUR \
	_IOWR(BMA220_IOC_MAGIC, 45, unsigned char)
#define BMA220_GET_SLEEP_DUR \
	_IOWR(BMA220_IOC_MAGIC, 46, unsigned char)
#define BMA220_SET_OFFSET_RESET \
	_IOWR(BMA220_IOC_MAGIC, 47, unsigned char)
#define BMA220_SET_CUT_OFF_SPEED \
	_IOWR(BMA220_IOC_MAGIC, 48, unsigned char)
#define BMA220_SET_CAL_MANUAL \
	_IOWR(BMA220_IOC_MAGIC, 49, unsigned char)
#define BMA220_SET_SBIST \
	_IOWR(BMA220_IOC_MAGIC, 50, unsigned char)
#define BMA220_SET_INTERRUPT_REGISTER \
	_IOWR(BMA220_IOC_MAGIC, 51, unsigned char)
#define BMA220_SET_DIRECTION_INTERRUPT_REGISTER \
	_IOWR(BMA220_IOC_MAGIC, 52, unsigned char)
#define BMA220_GET_ORIENT_INT \
	_IOWR(BMA220_IOC_MAGIC, 53, unsigned char)
#define BMA220_SET_ORIENT_BLOCKING \
	_IOWR(BMA220_IOC_MAGIC, 54, unsigned char)
#define BMA220_GET_CHIP_ID \
	_IOWR(BMA220_IOC_MAGIC, 55, unsigned char)
#define BMA220_GET_SC_FILT_CONFIG \
	_IOWR(BMA220_IOC_MAGIC, 56, unsigned char)
#define BMA220_GET_SLEEP_EN \
	_IOWR(BMA220_IOC_MAGIC, 57, unsigned char)
#define BMA220_GET_SERIAL_HIGH_BW \
	_IOWR(BMA220_IOC_MAGIC, 58, unsigned char)
#define BMA220_GET_LATCH_INT \
	_IOWR(BMA220_IOC_MAGIC, 59, unsigned char)
#define BMA220_GET_EN_DATA \
	_IOWR(BMA220_IOC_MAGIC, 60, unsigned char)
#define BMA220_GET_EN_HIGH_XYZ \
	_IOWR(BMA220_IOC_MAGIC, 61, unsigned char)
#define BMA220_GET_HIGH_TH \
	_IOWR(BMA220_IOC_MAGIC, 62, unsigned char)
#define BMA220_GET_HIGH_HY \
	_IOWR(BMA220_IOC_MAGIC, 63, unsigned char)
#define BMA220_GET_HIGH_DUR \
	_IOWR(BMA220_IOC_MAGIC, 64, unsigned char)
#define BMA220_GET_EN_LOW \
	_IOWR(BMA220_IOC_MAGIC, 65, unsigned char)
#define BMA220_GET_LOW_TH \
	_IOWR(BMA220_IOC_MAGIC, 66, unsigned char)
#define BMA220_GET_LOW_HY \
	_IOWR(BMA220_IOC_MAGIC, 67, unsigned char)
#define BMA220_GET_LOW_DUR \
	_IOWR(BMA220_IOC_MAGIC, 68, unsigned char)
#define BMA220_GET_EN_ORIENT \
	_IOWR(BMA220_IOC_MAGIC, 69, unsigned char)
#define BMA220_GET_ORIENT_EX \
	_IOWR(BMA220_IOC_MAGIC, 70, unsigned char)
#define BMA220_GET_ORIENT_BLOCKING \
	_IOWR(BMA220_IOC_MAGIC, 71, unsigned char)
#define BMA220_GET_EN_TT_XYZ \
	_IOWR(BMA220_IOC_MAGIC, 72, unsigned char)
#define BMA220_GET_TT_TH \
	_IOWR(BMA220_IOC_MAGIC, 73, unsigned char)
#define BMA220_GET_TT_DUR \
	_IOWR(BMA220_IOC_MAGIC, 74, unsigned char)
#define BMA220_GET_TT_FILT \
	_IOWR(BMA220_IOC_MAGIC, 75, unsigned char)
#define BMA220_SET_TT_SAMP \
	_IOWR(BMA220_IOC_MAGIC, 76, unsigned char)
#define BMA220_GET_TT_SAMP \
	_IOWR(BMA220_IOC_MAGIC, 77, unsigned char)
#define BMA220_SET_TIP_EN \
	_IOWR(BMA220_IOC_MAGIC, 78, unsigned char)
#define BMA220_GET_TIP_EN \
	_IOWR(BMA220_IOC_MAGIC, 79, unsigned char)
#define BMA220_GET_EN_SLOPE_XYZ	\
	_IOWR(BMA220_IOC_MAGIC, 80, unsigned char)
#define BMA220_GET_SLOPE_TH		\
	_IOWR(BMA220_IOC_MAGIC, 81, unsigned char)
#define BMA220_GET_SLOPE_DUR	\
	_IOWR(BMA220_IOC_MAGIC, 82, unsigned char)
#define BMA220_GET_SLOPE_FILT	\
	_IOWR(BMA220_IOC_MAGIC, 83, unsigned char)
#define BMA220_GET_HP_XYZ_EN	\
	_IOWR(BMA220_IOC_MAGIC, 84, unsigned char)
#define BMA220_GET_OFFSET_TARGET_X	\
	_IOWR(BMA220_IOC_MAGIC, 85, unsigned char)
#define BMA220_GET_OFFSET_TARGET_Y	\
	_IOWR(BMA220_IOC_MAGIC, 86, unsigned char)
#define BMA220_GET_OFFSET_TARGET_Z	\
	_IOWR(BMA220_IOC_MAGIC, 87, unsigned char)
#define BMA220_GET_CUT_OFF_SPEED	\
	_IOWR(BMA220_IOC_MAGIC, 88, unsigned char)
#define BMA220_GET_CAL_MANUAL	\
	_IOWR(BMA220_IOC_MAGIC, 89, unsigned char)
#define BMA220_SET_OFFSET_XYZ	\
	_IOWR(BMA220_IOC_MAGIC, 90, signed char)
#define BMA220_GET_OFFSET_XYZ	\
	_IOWR(BMA220_IOC_MAGIC, 91, signed char)
#define BMA220_IOCTL_READ	\
	_IOWR(BMA220_IOC_MAGIC, 92, signed char)
#define BMA220_IOCTL_WRITE	\
	_IOR(BMA220_IOC_MAGIC, 93, signed char)
#define BMA220_IOCTL_CALIBRATION	\
	_IO(BMA220_IOC_MAGIC, 94)
#define BMA220_IOCTL_EVENT_CTRL	\
	_IOW(BMA220_IOC_MAGIC, 95, signed char)

#define BMA220_IOC_MAXNR				95
#define YAMAHA_SENSORS					1
/* #define DEBUG 1 */
/* #define BMA220_DEBUG 1 */

#ifdef YAMAHA_SENSORS
#define GRAVITY_EARTH			9806550
#define ABSMIN_2G				(-GRAVITY_EARTH * 2)
#define ABSMAX_2G				(GRAVITY_EARTH * 2)
#define SENSOR_NAME				"accelerometer"
#define SENSOR_DEFAULT_DELAY	(200)   /* 200 ms */
#define SENSOR_MAX_DELAY		(2000)  /* 2000 ms */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)
#endif

/* i2c operation for bma220 API */
static char bma220_i2c_write(unsigned char sla, unsigned char reg_addr,
	unsigned char *data, unsigned char len);
static char bma220_i2c_read(unsigned char sla, unsigned char reg_addr,
	unsigned char *data, unsigned char len);
static void bma220_i2c_delay(unsigned int msec);

/* globe variant */
static struct i2c_client *bma220_client;
struct bma220_data {
	bma220_t		bma220;
	struct mutex mutex;
	struct workqueue_struct	*queue_struct;
	struct work_struct	work;
	struct early_suspend early_suspend;
	int enabled;
#ifdef YAMAHA_SENSORS
	struct input_dev *input;
	int delay;
#endif
};

#ifdef YAMAHA_SENSORS
/*
 * Transformation matrix for chip mounting position
 */
static const int bma220_position_map[][3][3] = {
	{{0, -1, 0}, {1, 0, 0}, {0, 0, 1} }, /* top/upper-left */ /* 7627 */
	{{1, 0, 0}, {0, 1, 0}, {0, 0, 1} }, /* top/upper-right */ /* 7625 */
	{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1} }, /* top/lower-right */
	{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1} }, /* top/lower-left */
	{{0, 1, 0}, {1, 0, 0}, {0, 0, -1} }, /* bottom/upper-right */
	{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1} }, /* bottom/upper-left */
	{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1} }, /* bottom/lower-left */
	{{1, 0, 0} , {0, -1, 0} , {0, 0, -1} }, /* bottom/lower-right */
};
#endif

static int workqueue_locking = -1;
static int bma220_x, bma220_y, bma220_z;
static int bma220_no_event;

#ifdef YAMAHA_SENSORS
static void report_data(void)
{
	struct bma220_data *data = i2c_get_clientdata(bma220_client);

	if (bma220_no_event)
		return;

	input_report_abs(data->input, ABS_X, bma220_x * (GRAVITY_EARTH / 16));
	input_report_abs(data->input, ABS_Y, bma220_y * (GRAVITY_EARTH / 16));
	input_report_abs(data->input, ABS_Z, bma220_z * (GRAVITY_EARTH / 16));
	input_sync(data->input);
}
#endif

static int bma220_set_suspend_mode(int mode)
{
	unsigned char buf[1];
	struct bma220_data *data = i2c_get_clientdata(bma220_client);
	int err = 0;

	if (mode == 1 && data->enabled != 1)
		/* 1: normal mode, 0: suspend mode */
		err = bma220_set_suspend((unsigned char *)buf);
	else if (mode == 0 && data->enabled != 0)
		err = bma220_set_suspend((unsigned char *)buf);
	else
		return -EFAULT;
	if (err < 0) {
		printk(KERN_ERR "bma220_set_suspend error = %x\n", err);
		return err;
	}
	data->enabled = ((buf[0] == 0xFF) ? 1 : 0);

	if (mode != data->enabled) {
		bma220_i2c_delay(10);
		bma220_set_suspend_mode(mode);
		return err;
	}

#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s : mode=%d enabled=%d\n",
		__func__, mode, data->enabled);
#endif
	return err;
}

static void bma220_suspend(void)
{
	bma220_set_suspend_mode(0);
}

static void bma220_resume(void)
{
	bma220_set_suspend_mode(1);
}

static void bma220_early_suspend(struct early_suspend *handler)
{
#ifdef YAMAHA_SENSORS
	struct bma220_data *data = i2c_get_clientdata(bma220_client);

	mutex_lock(&data->mutex);
	if (data->enabled)
		bma220_suspend();
	mutex_unlock(&data->mutex);
#else
	bma220_suspend();
#endif
	printk(KERN_INFO "%s\n", __func__);
}

static void bma220_early_resume(struct early_suspend *handler)
{
#ifdef YAMAHA_SENSORS
	struct bma220_data *data = i2c_get_clientdata(bma220_client);

	mutex_lock(&data->mutex);
	if (data->enabled)
		bma220_resume();
	mutex_unlock(&data->mutex);
#else
	bma220_resume();
#endif
	printk(KERN_INFO "%s\n", __func__);
}

static int accel_xyz_transformation(int index, bma220acc_t acc)
{
	int data = 0;

	data += acc.x * bma220_position_map[CONFIG_INPUT_BMA220_POSITION][index][0];
	data += acc.y * bma220_position_map[CONFIG_INPUT_BMA220_POSITION][index][1];
	data += acc.z * bma220_position_map[CONFIG_INPUT_BMA220_POSITION][index][2];

	return data;
}

static void bma220_get_accel_xyz(void)
{
	bma220acc_t acc;

	bma220_read_accel_xyz(&acc);
#ifdef BMA220_DEBUG
	printk(KERN_INFO "BMA220: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);
#endif

	/* coordinate transformation */
	bma220_x = accel_xyz_transformation(0, acc);
	bma220_y = accel_xyz_transformation(1, acc);
	bma220_z = accel_xyz_transformation(2, acc);
}

static void bma220_do_polling(struct work_struct *work_ptr)
{
	struct bma220_data *data = i2c_get_clientdata(bma220_client);

	while ((workqueue_locking == 1) && (data->enabled == 1)) {
		bma220_get_accel_xyz();
		bma220_i2c_delay(10);
#ifdef YAMAHA_SENSORS
		report_data();
		bma220_i2c_delay(data->delay);
#else
		bma220_i2c_delay(200);
#endif
	}
	workqueue_locking = 0;
}

/*	i2c delay routine for eeprom	*/
static inline void bma220_i2c_delay(unsigned int msec)
{
	msleep(msec);
}

/*	i2c write routine for bma220	*/
static inline char bma220_i2c_write(unsigned char sla,
	unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;
	int i = 0;
	unsigned char addr;
	/* No global client pointer? */
	if (bma220_client == NULL)
		return -1;
	addr = reg_addr<<1;		/*bma220 i2c addr left shift*/
	dummy = i2c_smbus_write_byte_data(bma220_client, addr, data[0]);

	while (i < len) {
		addr = reg_addr<<1;		/*bma220 i2c addr left shift*/
		dummy = i2c_smbus_write_byte_data(bma220_client, addr, data[0]);
		reg_addr++;
		data++;
		i++;
		if (dummy < 0)
			return -1;
	}

	return 0;
}

/*	i2c read routine for bma220	*/
static inline char bma220_i2c_read(unsigned char sla,
	unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy = 0;
	int i = 0;
	unsigned char addr;
	/* No global client pointer? */
	if (bma220_client == NULL)
		return -1;
	while (i < len) {
		addr = reg_addr<<1;	/*bma220 i2c addr left shift*/
		dummy = i2c_smbus_read_word_data(bma220_client, addr);
		data[i] = dummy & 0x00ff;
		i++;
		reg_addr++;
		dummy = len;
	}
	return 0;
}

/*	read command for BMA220 device file	*/
static ssize_t bma220_read(struct file *file, char __user *buf,
	size_t count, loff_t *offset)
{
	bma220acc_t acc;
	int ret;
	if (bma220_client == NULL)
#ifdef BMA220_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;

	bma220_read_accel_xyz(&acc);
#ifdef BMA220_DEBUG
	printk(KERN_INFO "BMA220: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);
#endif

	if (count != sizeof(acc))
		return -1;
	ret = copy_to_user(buf, &acc, sizeof(acc));
	if (ret != 0) {
#ifdef BMA220_DEBUG
		printk(KERN_INFO "BMA220: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}

/*	write command for BMA220 device file	*/
static ssize_t bma220_write(struct file *file,
	const char __user *buf, size_t count, loff_t *offset)
{
	if (bma220_client == NULL)
		return -1;
#ifdef BMA220_DEBUG
	printk(KERN_INFO "BMA220 should be accessed with ioctl command\n");
#endif
	return 0;
}

/*	open command for BMA220 device file	*/
static int bma220_open(struct inode *inode, struct file *file)
{
#ifdef BMA220_DEBUG
		printk(KERN_INFO "%s\n", __func__);
#endif

	if (bma220_client == NULL) {
#ifdef BMA220_DEBUG
		printk(KERN_ERR "I2C driver not install\n");
#endif
		return -1;
	}

#ifdef BMA220_DEBUG
	printk(KERN_INFO "BMA220 has been opened\n");
#endif
	return 0;
}

/*	release command for BMA220 device file	*/
static int bma220_close(struct inode *inode, struct file *file)
{
#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	return 0;
}

int bma220_read_accel_avg(int num_avg, bma220acc_t *min,
	bma220acc_t *max, bma220acc_t *avg)
{
	long x_avg = 0, y_avg = 0, z_avg = 0;
	int comres = 0;
	int i;
	bma220acc_t accel; /* read accel data */

	x_avg = 0;
	y_avg = 0;
	z_avg = 0;
	max->x = -32;
	max->y = -32;
	max->z = -32;
	min->x = 31;
	min->y = 31;
	min->z = 31;

	for (i = 0; i < num_avg; i++) {
		comres += bma220_read_accel_xyz(&accel);
			/* read 10 acceleration data triples */

		if (accel.x > max->x)
			max->x = accel.x;
		if (accel.x < min->x)
			min->x = accel.x;

		if (accel.y > max->y)
			max->y = accel.y;
		if (accel.y < min->y)
			min->y = accel.y;

		if (accel.z > max->z)
			max->z = accel.z;
		if (accel.z < min->z)
			min->z = accel.z;

		x_avg += accel.x;
		y_avg += accel.y;
		z_avg += accel.z;

		bma220_i2c_delay(30);
	}
	avg->x = (x_avg / num_avg); /* calculate averages, min and max values */
	avg->y = (y_avg / num_avg);
	avg->z = (z_avg / num_avg);
	return comres;
}

int bma220_verify_min_max(bma220acc_t min, bma220acc_t max, bma220acc_t avg)
{
	signed char dx, dy, dz;
	int ver_ok = 1;

	dx =  ((max.x) - (min.x)); /* calc delta max-min */
	dy =  ((max.y) - (min.y));
	dz =  ((max.z) - (min.z));


	if ((dx > 2) || (dx < -2))
		ver_ok = 0;
	if ((dy > 2) || (dy < -2))
		ver_ok = 0;
	if ((dz > 2) || (dz < -2))
		ver_ok = 0;

	return ver_ok;
}

static int bma220_calibration(void)
{
	int min_max_ok = 0;
	bma220acc_t min, max, avg;
	int err;
	unsigned char data[3];

	bma220_i2c_delay(300); /* needed to prevent CALIB_ERR_MOV */
	bma220_read_accel_avg(10, &min, &max, &avg);
		/* read acceleration data min, max, avg */
	min_max_ok = bma220_verify_min_max(min, max, avg);
	if (!min_max_ok) /* check if calibration is possible */
		return -1;

	data[0] = (unsigned char) avg.x;
	data[1] = (unsigned char) avg.y;
	data[2] = (unsigned char) (avg.z - 16);

	printk(KERN_INFO "%s: %d %d %d\n", __func__, data[0], data[1], data[2]);

	err = bma220_set_offset_xyz(*(bma220acc_t *)data);
	if (err) {
		printk(KERN_ERR "%s: bma220_set_offset_xyz error!\n", __func__);
		return err;
	}

	return 0;
}

static int bma220_reset_offset_xyz(void)
{
	unsigned char data[3] = {0, 0, 0};
	return bma220_set_offset_xyz(*(bma220acc_t *)data);
}

/*	ioctl command for BMA220 device file	*/
static int bma220_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	struct bma220_data *sensor_data = i2c_get_clientdata(bma220_client);

	/* check cmd */
	if (_IOC_TYPE(cmd) != BMA220_IOC_MAGIC) {
		#ifdef DEBUG
		printk(KERN_ERR "cmd magic type error\n");
		#endif
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > BMA220_IOC_MAXNR) {
		#ifdef DEBUG
		printk(KERN_ERR "cmd number error\n");
		#endif
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));
	if (err) {
		#ifdef DEBUG
		printk(KERN_ERR "cmd access_ok error\n");
		#endif
		return -EFAULT;
	}
	/* check bam120_client */
	if (bma220_client == NULL) {
		#ifdef DEBUG
		printk(KERN_ERR "I2C driver not install\n");
		#endif
		return -EFAULT;
	}

	/* cmd mapping */
	switch (cmd) {
	case BMA220_IOCTL_EVENT_CTRL:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		printk(KERN_INFO "%s: set bma220_no_event = %d\n",
			__func__, data[0]);
		bma220_no_event = (int)data[0];
		return err;
	case BMA220_IOCTL_CALIBRATION:
		if (sensor_data->enabled == 0) {
			bma220_set_suspend_mode(1);
			bma220_i2c_delay(30);
			err = bma220_reset_offset_xyz();
			if (err)
				printk(KERN_ERR "%s: reset offset error!\n",
					__func__);
			else
				err = bma220_calibration();
			bma220_i2c_delay(30);
			bma220_set_suspend_mode(0);
			return err;
		}
		err = bma220_reset_offset_xyz();
		if (err)
			printk(KERN_ERR "%s: reset offset error!\n", __func__);
		else
			err = bma220_calibration();
		return err;
	case BMA220_IOCTL_WRITE:
		if (copy_from_user(data, (unsigned char *)arg, 2) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		printk(KERN_INFO "%s: write = %x %x\n",
			__func__, data[1], data[0]);
		err = bma220_i2c_write(BMA220_I2C_ADDR, data[1], &data[0], 1);
		return err;

	case BMA220_IOCTL_READ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_i2c_read(BMA220_I2C_ADDR, data[0], &data[0], 1);
		printk(KERN_INFO "%s: read = %d\n", __func__, data[0]);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SOFT_RESET:
		err = bma220_soft_reset();
		return err;

	case BMA220_SET_SUSPEND:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		bma220_set_suspend_mode((int)data[0]);
		return err;

	case BMA220_SET_OFFSET_TARGET_X:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_x(*data);
		return err;

	case BMA220_SET_OFFSET_TARGET_Y:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_y(*data);
		return err;

	case BMA220_SET_OFFSET_TARGET_Z:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_z(*data);
		return err;

	case BMA220_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_range(*data);
		return err;

	case BMA220_GET_RANGE:
		err = bma220_get_range(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_mode(*data);
		return err;

	case BMA220_GET_MODE:
		err = bma220_get_mode(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_BANDWIDTH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_bandwidth(*data);
		return err;

	case BMA220_GET_BANDWIDTH:
		err = bma220_get_bandwidth(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_LOW_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_low_th(*data);
		return err;

	case BMA220_SET_LOW_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_low_dur(*data);
		return err;

	case BMA220_SET_HIGH_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_high_th(*data);
		return err;

	case BMA220_SET_HIGH_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_high_dur(*data);
		return err;

	case BMA220_RESET_INTERRUPT:
		err = bma220_reset_int();
		return err;

	case BMA220_READ_ACCEL_X:
		err = bma220_read_accel_x((signed char *)data);
		if (copy_to_user((signed char *)arg,
			(signed char *)data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_READ_ACCEL_Y:
		err = bma220_read_accel_y((signed char *)data);
		if (copy_to_user((signed char *)arg,
			(signed char *)data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_READ_ACCEL_Z:
		err = bma220_read_accel_z((signed char *)data);
		if (copy_to_user((signed char *)arg,
			(signed char *)data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_EN_LOW:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_low(*data);
		return err;

	case BMA220_SET_EN_HIGH_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_high_xyz(*data);
		return err;

	case BMA220_SET_LATCH_INT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_latch_int(*data);
		return err;

	case BMA220_SET_LOW_HY:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_low_hy(*data);
		return err;

	case BMA220_SET_HIGH_HY:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_high_hy(*data);
		return err;

	case BMA220_READ_ACCEL_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_read_accel_xyz((bma220acc_t *)data);
		if (copy_to_user((bma220acc_t *)arg,
			(bma220acc_t *)data, 3) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_XYZ:
		err = bma220_get_offset_xyz((bma220acc_t *)data);
		if (copy_to_user((bma220acc_t *)arg,
			(bma220acc_t *)data, 3) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_OFFSET_XYZ:
		if (copy_from_user((bma220acc_t *)data,
			(bma220acc_t *)arg, 3) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		printk(KERN_INFO "%s: set offset %d %d %d\n", __func__,
			data[0], data[1], data[2]);
		if (sensor_data->enabled == 0) {
			bma220_set_suspend_mode(1);
			bma220_i2c_delay(10);
			err = bma220_set_offset_xyz(*(bma220acc_t *)data);
			bma220_i2c_delay(10);
			bma220_set_suspend_mode(0);
			return err;
		}
		err = bma220_set_offset_xyz(*(bma220acc_t *)data);
		return err;

	case BMA220_SET_SLEEP_EN:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sleep_en(*data);
		return err;

	case BMA220_SET_SC_FILT_CONFIG:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sc_filt_config(*data);
		return err;

	case BMA220_SET_SERIAL_HIGH_BW:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_serial_high_bw(*data);
		return err;

	case BMA220_SET_EN_ORIENT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_orient(*data);
		return err;

	case BMA220_SET_ORIENT_EX:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_orient_ex(*data);
		return err;

	case BMA220_SET_ORIENT_BLOCKING:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_orient_blocking(*data);
		return err;

	case BMA220_SET_EN_TT_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_tt_xyz(*data);
		return err;

	case BMA220_SET_TT_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_th(*data);
		return err;

	case BMA220_SET_TT_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_dur(*data);
		return err;

	case BMA220_SET_TT_FILT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_filt(*data);
		return err;

	case BMA220_SET_EN_SLOPE_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_slope_xyz(*data);
		return err;

	case BMA220_SET_EN_DATA:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_data(*data);
		return err;

	case BMA220_SET_SLOPE_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_slope_th(*data);
		return err;

	case BMA220_SET_SLOPE_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_slope_dur(*data);
		return err;

	case BMA220_SET_SLOPE_FILT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_slope_filt(*data);
		return err;

	case BMA220_SET_CAL_TRIGGER:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_cal_trigger(*data);
		return err;

	case BMA220_SET_HP_XYZ_EN:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_hp_xyz_en(*data);
		return err;

	case BMA220_SET_SLEEP_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sleep_dur(*data);
		return err;

	case BMA220_SET_OFFSET_RESET:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_reset(*data);
		return err;

	case BMA220_SET_CUT_OFF_SPEED:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_cut_off_speed(*data);
		return err;

	case BMA220_SET_CAL_MANUAL:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_cal_manual(*data);
		return err;

	case BMA220_SET_SBIST:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sbist(*data);
		return err;

	case BMA220_SET_INTERRUPT_REGISTER:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_interrupt_register(*data);
		return err;

	case BMA220_SET_DIRECTION_INTERRUPT_REGISTER:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_direction_interrupt_register(*data);
		return err;

	case BMA220_GET_DIRECTION_STATUS_REGISTER:
		err = bma220_get_direction_status_register(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_INTERRUPT_STATUS_REGISTER:
		err = bma220_get_interrupt_status_register(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENTATION:
		err = bma220_get_orientation(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_INT:
		err = bma220_get_orient_int(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CHIP_ID:
		err = bma220_get_chip_id(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		printk(KERN_INFO "bma220_get_chip_id = %d\n", data[0]);
		return err;

	case BMA220_GET_SC_FILT_CONFIG:
		err = bma220_get_sc_filt_config(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLEEP_EN:
		err = bma220_get_sleep_en(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SERIAL_HIGH_BW:
		err = bma220_get_serial_high_bw(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LATCH_INT:
		err = bma220_get_latch_int(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_DATA:
		err = bma220_get_en_data(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_HIGH_XYZ:
		err = bma220_get_en_high_xyz(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HIGH_TH:
		err = bma220_get_high_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HIGH_HY:
		err = bma220_get_high_hy(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HIGH_DUR:
		err = bma220_get_high_g_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_LOW:
		err = bma220_get_en_low(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LOW_TH:
		err = bma220_get_low_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LOW_HY:
		err = bma220_get_low_hy(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LOW_DUR:
		err = bma220_get_low_g_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_ORIENT:
		err = bma220_get_en_orient(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_EX:
		err = bma220_get_orient_ex(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_BLOCKING:
		err = bma220_get_orient_blocking(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_TT_XYZ:
		err = bma220_get_en_tt_xyz(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TT_TH:
		err = bma220_get_tt_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TT_DUR:
		err = bma220_get_tt_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TT_FILT:
		err = bma220_get_tt_filt(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_TT_SAMP:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_samp(*data);
		return err;

	case BMA220_SET_TIP_EN:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tip_en(*data);
		return err;

	case BMA220_GET_TT_SAMP:
		err = bma220_get_tt_samp(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TIP_EN:
		err = bma220_get_tip_en(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_SLOPE_XYZ:
		err = bma220_get_en_slope_xyz(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLOPE_TH:
		err = bma220_get_slope_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLOPE_DUR:
		err = bma220_get_slope_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLOPE_FILT:
		err = bma220_get_slope_filt(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HP_XYZ_EN:
		err = bma220_get_hp_xyz_en(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_TARGET_X:
		err = bma220_get_offset_target_x(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_TARGET_Y:
		err = bma220_get_offset_target_y(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_TARGET_Z:
		err = bma220_get_offset_target_z(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CUT_OFF_SPEED:
		err = bma220_get_cut_off_speed(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CAL_MANUAL:
		err = bma220_get_cal_manual(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CAL_RDY:
		err = bma220_get_cal_rdy(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLEEP_DUR:
		err = bma220_get_sleep_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#ifdef DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	default:
		return 0;
	}
}

#ifdef YAMAHA_SENSORS
/* Sysfs interface */
static ssize_t bma220_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bma220_data *data = i2c_get_clientdata(bma220_client);
	int delay;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	mutex_lock(&data->mutex);
	delay = data->delay;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

static ssize_t bma220_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct bma220_data *data = i2c_get_clientdata(bma220_client);
	long value, err;
	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

	if (value < 0)
		return count;

	if (SENSOR_MAX_DELAY < value)
		value = SENSOR_MAX_DELAY;

	mutex_lock(&data->mutex);
	data->delay = value;
	input_report_abs(input_data, ABS_CONTROL_REPORT,
		(data->enabled<<16) | value);
	mutex_unlock(&data->mutex);

	printk(KERN_INFO "%s: value=%d\n", __func__, (int)value);
	return count;
}

static ssize_t bma220_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bma220_data *data = i2c_get_clientdata(bma220_client);
	int enabled;

#ifdef DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	mutex_lock(&data->mutex);
	enabled = data->enabled;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t bma220_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct bma220_data *data = i2c_get_clientdata(bma220_client);
	long value, err;
	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

	if (value != 0 && value != 1)
		return count;

	mutex_lock(&data->mutex);
	if (data->enabled && !value)
		bma220_suspend();
	if (!data->enabled && value)
		bma220_resume();

	data->enabled = value;
	input_report_abs(input_data, ABS_CONTROL_REPORT,
		(value<<16) | data->delay);
	mutex_unlock(&data->mutex);

	if (data->enabled == 1) {
		workqueue_locking = 1;
		queue_work(data->queue_struct, &data->work);
			/* start polling */
	}

	printk(KERN_INFO "%s: set value=%d enabled=%d\n",
		__func__, (int)value, data->enabled);

	return count;
}

static ssize_t bma220_wake_store(struct device *dev,
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

static ssize_t bma220_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bma220_get_accel_xyz();
	bma220_i2c_delay(10);
#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s = %d %d %d\n", __func__ ,
		bma220_x, bma220_y, bma220_z);
#endif
	report_data();

	if (bma220_no_event)
		return sprintf(buf, "%d %d %d\n", 0, 0, 0);
	else
		return sprintf(buf, "%d %d %d\n",
				bma220_x * (GRAVITY_EARTH / 16),
				bma220_y * (GRAVITY_EARTH / 16),
				bma220_z * (GRAVITY_EARTH / 16));
}

static ssize_t bma220_status_show(struct device *dev,
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
	bma220_delay_show, bma220_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
	bma220_enable_show, bma220_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
	NULL, bma220_wake_store);
static DEVICE_ATTR(data, S_IRUGO, bma220_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, bma220_status_show, NULL);

static struct attribute *bma220_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group bma220_attribute_group = {
	.attrs = bma220_attributes
};
#endif


static const struct file_operations bma220_fops = {
	.owner = THIS_MODULE,
	.read = bma220_read,
	.write = bma220_write,
	.open = bma220_open,
	.release = bma220_close,
	.ioctl = bma220_ioctl,
};

static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma220",
	.fops = &bma220_fops,
};

static int bma220_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma220", I2C_NAME_SIZE);

	return 0;
}

#ifdef YAMAHA_SENSORS
static int bma220_input_init(struct bma220_data *bma220)
{
	struct input_dev *dev;
	int err;
	int input_registered = 0;

	dev = input_allocate_device(); /* create input device */
	if (!dev)
		return -ENOMEM;
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, bma220);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	input_registered = 1;

	bma220->input = dev;

	err = sysfs_create_group(&dev->dev.kobj,
		&bma220_attribute_group);
	if (err) {
		printk(KERN_ERR
			"%s: sysfs_create_group failed[%s]\n",
			__func__, dev->name);
		goto err;
	}
	return 0;
err:
	if (dev != NULL) {
		if (input_registered)
			input_unregister_device(dev);
		input_free_device(dev);
		dev = NULL;
	}
	return -1;
}
#endif

static void release_workqueue(struct bma220_data *data)
{
	if (data->queue_struct != NULL) {
		destroy_workqueue(data->queue_struct);
		data->queue_struct = NULL;
	}
}

static int bma220_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma220_data *data;

	bma220_client = NULL;
	bma220_no_event = 0;

	printk(KERN_INFO "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma220_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	/* read chip id */
	tempvalue = 0;
#ifdef BMA220_SMBUS
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
#else
	i2c_master_send(client, (char *)&tempvalue, 1);
	i2c_master_recv(client, (char *)&tempvalue, 1);
#endif
	if ((tempvalue & 0x00FF) == 0x00dd) {
		printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA220 registered I2C driver!\n");
		bma220_client = client;
	} else {
		printk(KERN_ERR "Bosch Sensortec Device not found, "\
			"i2c error %d \n", tempvalue);
		err = -1;
		goto kfree_exit;
	}

#ifdef YAMAHA_SENSORS
	mutex_init(&data->mutex);
#endif
	data->queue_struct = NULL;
	data->queue_struct = create_singlethread_workqueue("queue_struct");
	if (data->queue_struct == NULL) {
		printk(KERN_ERR "bma220 queue_struct error!!!!\n");
		return -ENOMEM;
	}

	INIT_WORK(&data->work, bma220_do_polling);
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	data->early_suspend.suspend = bma220_early_suspend;
	data->early_suspend.resume = bma220_early_resume;
	register_early_suspend(&data->early_suspend);

	i2c_set_clientdata(bma220_client, data);
	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma220 device register failed\n");
		goto kfree_exit;
	}
	printk(KERN_INFO "bma220 device create ok\n");

	/* bma220 sensor initial */
	data->bma220.bus_write = bma220_i2c_write;
	data->bma220.bus_read = bma220_i2c_read;
	data->bma220.delay_msec = bma220_i2c_delay;

	bma220_init(&(data->bma220));
	bma220_set_bandwidth(2);	/* bandwidth 250Hz */
	bma220_set_range(0);		/* range +/- 2G */
	data->enabled = 1; /* 0: suspend mode , 1: normal mode */
	bma220_set_suspend_mode(0);
#ifdef YAMAHA_SENSORS
	if (bma220_input_init(data) < 0)
		goto kfree_exit;
	data->delay = SENSOR_DEFAULT_DELAY;
#endif
	bma220_reset_int();
	return 0;

kfree_exit:
	release_workqueue(data);
	/* i2c_detach_client(client); */
	kfree(data);
	bma220_client = NULL;
exit:
	return err;
}


static int bma220_remove(struct i2c_client *client)
{
	struct bma220_data *data = i2c_get_clientdata(client);

	printk(KERN_INFO "%s\n", __func__);
	misc_deregister(&bma_device);
#ifdef YAMAHA_SENSORS
	if (data->input != NULL) {
		sysfs_remove_group(&data->input->dev.kobj,
			&bma220_attribute_group);
		input_unregister_device(data->input);
		input_free_device(data->input);
		data->input = NULL;
	}
#endif
	release_workqueue(data);
	/* i2c_detach_client(client); */
	kfree(data);
	bma220_client = NULL;
	return 0;
}

static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(bma220);

static const struct i2c_device_id bma220_id[] = {
	{ "bma220", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma220_id);

static struct i2c_driver bma220_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "bma220",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma220_id,
	.address_data	= &addr_data,
	.probe		= bma220_probe,
	.remove		= bma220_remove,
	.detect		= bma220_detect,
};

static int __init BMA220_init(void)
{
	printk(KERN_INFO "%s: G-Sensor driver init\n", __func__);
	return i2c_add_driver(&bma220_driver);
}

static void __exit BMA220_exit(void)
{
	printk(KERN_INFO "%s: G-Sensor driver exit\n", __func__);
	i2c_del_driver(&bma220_driver);
}

module_init(BMA220_init);
module_exit(BMA220_exit);
