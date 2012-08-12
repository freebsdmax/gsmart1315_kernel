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

#include <linux/bma220.h>

#define BMA220_MAJOR	101
#define BMA220_MINOR	0

#define BMA220_IOC_MAGIC 'B'

#define BMA220_SET_SLEEP_EN \
	_IOWR(BMA220_IOC_MAGIC, 0, unsigned char)
#define BMA220_SET_SUSPEND \
	_IO(BMA220_IOC_MAGIC, 1)
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


#define BMA220_IOC_MAXNR		92
#define BMA220_ENABLE_IRQ		1
#define BMA220_ORIENTATION_INTERRUPT		1

#define DEBUG 0
/* #define BMA220_DEBUG 1 */


/* i2c operation for bma220 API */
static char bma220_i2c_write(unsigned char sla, unsigned char reg_addr,
	unsigned char *data, unsigned char len);
static char bma220_i2c_read(unsigned char sla, unsigned char reg_addr,
	unsigned char *data, unsigned char len);
static void bma220_i2c_delay(unsigned int msec);
#ifdef BMA220_ORIENTATION_INTERRUPT
static void bma220_workqueue(struct work_struct *work_ptr);
#endif

/* globe variant */
static struct i2c_client *bma220_client = NULL;
struct bma220_data {
	bma220_t		bma220;
	#ifdef BMA220_ENABLE_IRQ
	int IRQ;
	struct fasync_struct *async_queue;
	#ifdef BMA220_ORIENTATION_INTERRUPT
	struct workqueue_struct	*queue_struct;
	struct workqueue_struct	*queue_struct2;
	struct work_struct	work;
	struct work_struct	work2;
	#endif
	#endif
};

/*definition for GPIO*/
#ifdef BMA220_ENABLE_IRQ
static int bma220_interrupt_config(void);
#define BMA220_IRQ_PIN	94
#define BMA220_IRQ	OMAP_GPIO_IRQ(BMA220_IRQ_PIN)
#endif

#ifdef BMA220_ENABLE_IRQ
static int bma220_interrupt_config()
{
#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif

#ifdef BMA220_ORIENTATION_INTERRUPT
	if (gpio_request(BMA220_IRQ_PIN, "bma220") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for bma220 IRQ\n",
			BMA220_IRQ_PIN);
		return -1;
	}

	if (gpio_direction_input(BMA220_IRQ_PIN) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for bma220 IRQ\n",
			BMA220_IRQ_PIN);
		return -1;
	}
#else
/* config interrupt pin for ZOOM_1 omap3430*/
	if (omap_request_gpio(BMA220_IRQ_PIN) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for bma220 IRQ\n",
			BMA220_IRQ_PIN);
		return -1;
	}
	omap_set_gpio_direction(BMA220_IRQ_PIN, 1);
#endif
	return 0;
}
#endif


#ifdef BMA220_ORIENTATION_INTERRUPT
static int workqueue_locking = -1;
static int bma220_mode = 2; /* suspend mode */
static unsigned char bma220_orient;
static int bma220_x, bma220_y, bma220_z;

static void bma220_get_accel_xyz(void)
{
	bma220acc_t acc;

	bma220_read_accel_xyz(&acc);
	#ifdef BMA220_DEBUG
	printk(KERN_INFO "BMA220: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);
	#endif
	bma220_x = (int)acc.x;
	bma220_y = (int)acc.y;
	bma220_z = (int)acc.z;
}

static void bma220_workqueue(struct work_struct *work_ptr)
{
	struct bma220_data *data;
	data = i2c_get_clientdata(bma220_client);
	if (data == NULL)
		return;

	bma220_get_orientation(&bma220_orient);
	#ifdef BMA220_DEBUG
	printk(KERN_INFO "orient=%d locking=%d mode=%d\n",
		bma220_orient, workqueue_locking, bma220_mode);
	#endif
	bma220_get_accel_xyz();
	if (workqueue_locking == -1) {
		workqueue_locking = 0;
		return;
	}
	if (!workqueue_locking) {
		workqueue_locking = 1;
		queue_work(data->queue_struct2, &data->work2);
	}
}

static void bma220_workqueue2(struct work_struct *work_ptr)
{
	while ((bma220_orient == 0 || bma220_orient == 3) &&
		(workqueue_locking == 1) && (bma220_mode == 0)) {
		msleep(500);
		bma220_get_accel_xyz();
		msleep(200);
		if ((bma220_x * bma220_x + bma220_y * bma220_y - 9) >
			(bma220_z * bma220_z)) {
			printk(KERN_INFO "onOrientationChanged,"
				"rotation changed to %d\n",
				(bma220_orient == 0 ? 1 : 0));
			workqueue_locking = 0;
		}
		msleep(300);
	}
	workqueue_locking = 0;
}
#endif

#ifdef BMA220_ENABLE_IRQ
static irqreturn_t bma220_irq_handler(int irq, void *_id)
{
	struct bma220_data *data;
	unsigned long flags;

	if (((bma220_t *)_id)->chip_id != 0xdd) {
#ifdef BMA220_DEBUG
		printk(KERN_INFO "%s error\n", __func__);
#endif
		return IRQ_HANDLED;
	}
	if (bma220_client == NULL)
		return IRQ_HANDLED;

#ifdef BMA220_DEBUG
	printk(KERN_INFO "bma220 irq handler\n");
#endif
	data = i2c_get_clientdata(bma220_client);
	if (data == NULL)
		return IRQ_HANDLED;
	local_irq_save(flags);
	if (data->async_queue)
		kill_fasync(&data->async_queue, SIGIO, POLL_IN);
	local_irq_restore(flags);
#ifdef BMA220_ORIENTATION_INTERRUPT
	queue_work(data->queue_struct, &data->work);
#endif
	return IRQ_HANDLED;
}
#endif

/*	i2c delay routine for eeprom	*/
static inline void bma220_i2c_delay(unsigned int msec)
{
	mdelay(msec);
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
		printk(KERN_INFO "I2C driver not install\n");
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

/*	ioctl command for BMA220 device file	*/
static int bma220_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check cmd */
	if (_IOC_TYPE(cmd) != BMA220_IOC_MAGIC) {
		#if DEBUG
		printk(KERN_INFO "cmd magic type error\n");
		#endif
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > BMA220_IOC_MAXNR) {
		#if DEBUG
		printk(KERN_INFO "cmd number error\n");
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
		#if DEBUG
		printk(KERN_INFO "cmd access_ok error\n");
		#endif
		return -EFAULT;
	}
	/* check bam120_client */
	if (bma220_client == NULL) {
		#if DEBUG
		printk(KERN_INFO "I2C driver not install\n");
		#endif
		return -EFAULT;
	}

	/* cmd mapping */

	switch (cmd) {
	case BMA220_SOFT_RESET:
		err = bma220_soft_reset();
		return err;

	case BMA220_SET_SUSPEND:
		err = bma220_set_suspend();
		#ifdef BMA220_ORIENTATION_INTERRUPT
		if (err == 0) {
			if (bma220_mode == 0)
				bma220_mode = 2;
			else
				bma220_mode = 0;
		}
		printk(KERN_INFO "BMA220_SET_MODE = %d\n", bma220_mode);
		#endif
		return err;

	case BMA220_SET_OFFSET_TARGET_X:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_x(*data);
		return err;

	case BMA220_SET_OFFSET_TARGET_Y:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_y(*data);
		return err;

	case BMA220_SET_OFFSET_TARGET_Z:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_z(*data);
		return err;

	case BMA220_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_range(*data);
		return err;

	case BMA220_GET_RANGE:
		err = bma220_get_range(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_mode(*data);
		return err;

	case BMA220_GET_MODE:
		err = bma220_get_mode(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_BANDWIDTH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_bandwidth(*data);
		return err;

	case BMA220_GET_BANDWIDTH:
		err = bma220_get_bandwidth(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_LOW_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_low_th(*data);
		return err;

	case BMA220_SET_LOW_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_low_dur(*data);
		return err;

	case BMA220_SET_HIGH_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_high_th(*data);
		return err;

	case BMA220_SET_HIGH_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_high_dur(*data);
		return err;

	case BMA220_RESET_INTERRUPT:
		err = bma220_reset_int();
		return err;

	case BMA220_READ_ACCEL_X:
		err = bma220_read_accel_x((signed char*)data);
		if (copy_to_user((signed char *)arg,
			(signed char*)data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_READ_ACCEL_Y:
		err = bma220_read_accel_y((signed char*)data);
		if (copy_to_user((signed char *)arg,
			(signed char *)data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_READ_ACCEL_Z:
		err = bma220_read_accel_z((signed char*)data);
		if (copy_to_user((signed char *)arg,
			(signed char *)data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_EN_LOW:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_low(*data);
		return err;

	case BMA220_SET_EN_HIGH_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_high_xyz(*data);
		return err;

	case BMA220_SET_LATCH_INT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_latch_int(*data);
		return err;

	case BMA220_SET_LOW_HY:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_low_hy(*data);
		return err;

	case BMA220_SET_HIGH_HY:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_high_hy(*data);
		return err;

	case BMA220_READ_ACCEL_XYZ:
#ifdef BMA220_ORIENTATION_INTERRUPT
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		/*
		[SensorManager mode]
			SENSOR_DELAY_FASTEST = 0
			SENSOR_DELAY_GAME = 20
			SENSOR_DELAY_UI = 60
			SENSOR_DELAY_NORMAL = 200
			SENSOR_DELAY_ORIENTATION = 255
		*/
		if (data[0] != 255)
			err = bma220_read_accel_xyz((bma220acc_t *)data);
		else{
			data[0] = bma220_x;
			data[1] = bma220_y;
			data[2] = bma220_z;
		}
#else
		err = bma220_read_accel_xyz((bma220acc_t *)data);
#endif
		if (copy_to_user((bma220acc_t *)arg,
			(bma220acc_t *)data, 3) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_XYZ:
		err = bma220_get_offset_xyz((bma220acc_t *)data);
		if (copy_to_user((bma220acc_t *)arg,
			(bma220acc_t *)data, 3) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_OFFSET_XYZ:
		if (copy_from_user((bma220acc_t *)data,
			(bma220acc_t *)arg, 3) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_xyz(*(bma220acc_t *)data);
		return err;

	case BMA220_SET_SLEEP_EN:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sleep_en(*data);
		return err;

	case BMA220_SET_SC_FILT_CONFIG:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sc_filt_config(*data);
		return err;

	case BMA220_SET_SERIAL_HIGH_BW:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_serial_high_bw(*data);
		return err;

	case BMA220_SET_EN_ORIENT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_orient(*data);
		return err;

	case BMA220_SET_ORIENT_EX:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_orient_ex(*data);
		return err;

	case BMA220_SET_ORIENT_BLOCKING:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_orient_blocking(*data);
		return err;

	case BMA220_SET_EN_TT_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_tt_xyz(*data);
		return err;

	case BMA220_SET_TT_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_th(*data);
		return err;

	case BMA220_SET_TT_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_dur(*data);
		return err;

	case BMA220_SET_TT_FILT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_filt(*data);
		return err;

	case BMA220_SET_EN_SLOPE_XYZ:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_slope_xyz(*data);
		return err;

	case BMA220_SET_EN_DATA:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_en_data(*data);
		return err;

	case BMA220_SET_SLOPE_TH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_slope_th(*data);
		return err;

	case BMA220_SET_SLOPE_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_slope_dur(*data);
		return err;

	case BMA220_SET_SLOPE_FILT:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_slope_filt(*data);
		return err;

	case BMA220_SET_CAL_TRIGGER:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_cal_trigger(*data);
		return err;

	case BMA220_SET_HP_XYZ_EN:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_hp_xyz_en(*data);
		return err;

	case BMA220_SET_SLEEP_DUR:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sleep_dur(*data);
		return err;

	case BMA220_SET_OFFSET_RESET:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_offset_reset(*data);
		return err;

	case BMA220_SET_CUT_OFF_SPEED:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_cut_off_speed(*data);
		return err;

	case BMA220_SET_CAL_MANUAL:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_cal_manual(*data);
		return err;

	case BMA220_SET_SBIST:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_sbist(*data);
		return err;

	case BMA220_SET_INTERRUPT_REGISTER:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_interrupt_register(*data);
		return err;

	case BMA220_SET_DIRECTION_INTERRUPT_REGISTER:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_direction_interrupt_register(*data);
		return err;

	case BMA220_GET_DIRECTION_STATUS_REGISTER:
		err = bma220_get_direction_status_register(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_INTERRUPT_STATUS_REGISTER:
		err = bma220_get_interrupt_status_register(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENTATION:
		err = bma220_get_orientation(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_INT:
		err = bma220_get_orient_int(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CHIP_ID:
		err = bma220_get_chip_id(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SC_FILT_CONFIG:
		err = bma220_get_sc_filt_config(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLEEP_EN:
		err = bma220_get_sleep_en(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SERIAL_HIGH_BW:
		err = bma220_get_serial_high_bw(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LATCH_INT:
		err = bma220_get_latch_int(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_DATA:
		err = bma220_get_en_data(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_HIGH_XYZ:
		err = bma220_get_en_high_xyz(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HIGH_TH:
		err = bma220_get_high_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HIGH_HY:
		err = bma220_get_high_hy(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HIGH_DUR:
		err = bma220_get_high_g_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_LOW:
		err = bma220_get_en_low(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LOW_TH:
		err = bma220_get_low_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LOW_HY:
		err = bma220_get_low_hy(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_LOW_DUR:
		err = bma220_get_low_g_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_ORIENT:
		err = bma220_get_en_orient(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_EX:
		err = bma220_get_orient_ex(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_BLOCKING:
		err = bma220_get_orient_blocking(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_TT_XYZ:
		err = bma220_get_en_tt_xyz(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TT_TH:
		err = bma220_get_tt_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TT_DUR:
		err = bma220_get_tt_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TT_FILT:
		err = bma220_get_tt_filt(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_TT_SAMP:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tt_samp(*data);
		return err;

	case BMA220_SET_TIP_EN:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = bma220_set_tip_en(*data);
		return err;

	case BMA220_GET_TT_SAMP:
		err = bma220_get_tt_samp(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_TIP_EN:
		err = bma220_get_tip_en(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_EN_SLOPE_XYZ:
		err = bma220_get_en_slope_xyz(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLOPE_TH:
		err = bma220_get_slope_th(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLOPE_DUR:
		err = bma220_get_slope_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLOPE_FILT:
		err = bma220_get_slope_filt(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_HP_XYZ_EN:
		err = bma220_get_hp_xyz_en(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_TARGET_X:
		err = bma220_get_offset_target_x(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_TARGET_Y:
		err = bma220_get_offset_target_y(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_OFFSET_TARGET_Z:
		err = bma220_get_offset_target_z(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CUT_OFF_SPEED:
		err = bma220_get_cut_off_speed(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CAL_MANUAL:
		err = bma220_get_cal_manual(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CAL_RDY:
		err = bma220_get_cal_rdy(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLEEP_DUR:
		err = bma220_get_sleep_dur(data);
		if (copy_to_user((unsigned char *)arg, data, 1) != 0) {
			#if DEBUG
			printk(KERN_INFO "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	default:
		return 0;
	}
}

#ifdef BMA220_ENABLE_IRQ
static int bma220_fasync(int fd, struct file *file, int mode)
{
	struct bma220_data *data;
	unsigned char orient[2];

	bma220_get_orientation(&orient[0]);
#ifdef BMA220_DEBUG
	printk(KERN_INFO "orient=%d", orient[0]);
	printk(KERN_INFO "%s\n", __func__);
#endif
	data = i2c_get_clientdata(bma220_client);
	return fasync_helper(fd, file, mode, &data->async_queue);
}
#endif

static const struct file_operations bma220_fops = {
	.owner = THIS_MODULE,
	.read = bma220_read,
	.write = bma220_write,
	.open = bma220_open,
	.release = bma220_close,
	.ioctl = bma220_ioctl,
#ifdef BMA220_ENABLE_IRQ
	.fasync = bma220_fasync,
#endif
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

static int bma220_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma220_data *data;

#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
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
	if ((tempvalue&0x00FF) == 0x00dd) {
		printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA220 registered I2C driver!\n");
		bma220_client = client;
	} else {
		printk(KERN_INFO "Bosch Sensortec Device not found, "\
			"i2c error %d \n", tempvalue);
		/* i2c_detach_client(client); */
		bma220_client = NULL;
		err = -1;
		goto kfree_exit;
	}

#ifdef BMA220_ORIENTATION_INTERRUPT
	data->queue_struct = NULL;
	data->queue_struct = create_singlethread_workqueue("queue_struct");
	if (data->queue_struct == NULL) {
		printk(KERN_INFO "bma220 queue_struct error!!!!\n");
		return -ENOMEM;
	}
	data->queue_struct2 = NULL;
	data->queue_struct2 = create_singlethread_workqueue("queue_struct2");
	if (data->queue_struct2 == NULL) {
		printk(KERN_INFO "bma220 queue_struct2 error!!!!\n");
		return -ENOMEM;
	}
	INIT_WORK(&data->work, bma220_workqueue);
	INIT_WORK(&data->work2, bma220_workqueue2);
#endif

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
	bma220_set_bandwidth(2); /* bandwidth 250Hz */
	bma220_set_range(0);	/* range +/- 2G */

#ifdef BMA220_ORIENTATION_INTERRUPT
	err = bma220_interrupt_config();
	if (err < 0)
		goto exit_dereg;
	err = request_irq(data->IRQ = gpio_to_irq(BMA220_IRQ_PIN), \
		bma220_irq_handler, IRQF_TRIGGER_RISING, "bma220",
		&data->bma220);
	bma220_set_en_orient(1);
	bma220_reset_int();
#else
#ifdef	BMA220_ENABLE_IRQ
	/* register interrupt */
	err = bma220_interrupt_config();
	if (err < 0)
		goto exit_dereg;
	data->IRQ = BMA220_IRQ;
	err = request_irq(data->IRQ, bma220_irq_handler, IRQF_TRIGGER_RISING,
		"bma220", &data->bma220);
	if (err) {
		printk(KERN_ERR "could not request irq\n");
		goto exit_dereg;
	}
	bma220_set_en_tt_xyz(0);
	bma220_set_en_slope_xyz(0);
	bma220_set_en_high_xyz(0);
	bma220_set_en_low(0);
	bma220_set_en_orient(0);
	bma220_reset_int();
#endif
#endif
	return 0;

#ifdef BMA220_ENABLE_IRQ

exit_dereg:
	misc_deregister(&bma_device);
#endif
kfree_exit:
	kfree(data);
exit:
	return err;
}


static int bma220_remove(struct i2c_client *client)
{
	struct bma220_data *data = i2c_get_clientdata(client);
#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	misc_deregister(&bma_device);
#ifdef BMA220_ENABLE_IRQ
	free_irq(data->IRQ, &data->bma220);
#endif
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
#ifdef BMA220_DEBUG
	printk(KERN_INFO "%s\n", __func__);
#endif
	return i2c_add_driver(&bma220_driver);
}

static void __exit BMA220_exit(void)
{
	i2c_del_driver(&bma220_driver);
	printk(KERN_ERR "BMA220 exit\n");
}



module_init(BMA220_init);
module_exit(BMA220_exit);

