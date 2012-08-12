/*
    Copyright (C) 2011 GIGA-BYTE COMMUNICATION INC.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met
    1. Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation andor other materials provided with the distribution.
    3. For alternatively licensed files, this software is distributed under the terms
        of GNU General Public License version 2. 
    4. Neither the name of GIGA-BYTE COMMUNICATION INC. nor the names of 
        its contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * drivers/media/radio/si470x/radio-si470x-i2c.c
 *
 * I2C driver for radios with Silicon Labs Si470x FM Radio Receivers
 *
 * Copyright (c) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
/*
 * ToDo:
 * - RDS support
 */
/* driver definitions */
#define DRIVER_AUTHOR "Joonyoung Shim <jy0922.shim@samsung.com>";
#define DRIVER_KERNEL_VERSION KERNEL_VERSION(1, 0, 0)
#define DRIVER_CARD "Silicon Labs Si470x FM Radio Receiver"
#define DRIVER_DESC "I2C radio driver for Si470x FM Radio Receivers"
#define DRIVER_VERSION "1.0.0"
/* kernel includes */
#include <linux/i2c.h>
#include <linux/delay.h>
#include "radio-si470x.h"
/* I2C Device ID List */
#include <linux/debugfs.h>
static const struct i2c_device_id si470x_i2c_id[] = {
	/* Generic Entry */
	{ "si470x-fm", 0 },
	/* Terminating entry */
	{ }
};
#include <asm/gpio.h>
static int fm_power = 0;

struct si470x_device *radio;
struct RDS_buf* rb;

MODULE_DEVICE_TABLE(i2c, si470x_i2c_id);
/**************************************************************************
 * Module Parameters
 **************************************************************************/
/* Radio Nr */
static int radio_nr = -1;
module_param(radio_nr, int, 0444);
MODULE_PARM_DESC(radio_nr, "Radio Nr");
/**************************************************************************
 * I2C Definitions
 **************************************************************************/
/* Write starts with the upper byte of register 0x02 */
#define WRITE_REG_NUM           8
#define WRITE_INDEX(i)          (i + 0x02)
/* Read starts with the upper byte of register 0x0a */
#define READ_REG_NUM            RADIO_REGISTER_NUM
#define READ_INDEX(i)           ((i + RADIO_REGISTER_NUM - 0x0a) % READ_REG_NUM)
/**************************************************************************
 * General Driver Functions - REGISTERs
 **************************************************************************/
/*
 * si470x_get_register - read register
 */
int si470x_get_register(struct si470x_device *radio, int regnr)
{
	u16 buf[READ_REG_NUM];
	struct i2c_msg msgs[1] = {
		{ radio->client->addr, I2C_M_RD, sizeof(u16) * READ_REG_NUM,
			(void *)buf },
	};

	if (i2c_transfer(radio->client->adapter, msgs, 1) != 1)
		return -EIO;
	radio->registers[regnr] = __be16_to_cpu(buf[READ_INDEX(regnr)]);

	return 0;
}
/*
 * si470x_set_register - write register
 */
int si470x_set_register(struct si470x_device *radio, int regnr)
{
	int i;
	u16 buf[WRITE_REG_NUM];
	struct i2c_msg msgs[1] = {
		{ radio->client->addr, 0, sizeof(u16) * WRITE_REG_NUM,
			(void *)buf },
	};
	for (i = 0; i < WRITE_REG_NUM; i++)
		buf[i] = __cpu_to_be16(radio->registers[WRITE_INDEX(i)]);
	if (i2c_transfer(radio->client->adapter, msgs, 1) != 1)
		return -EIO;

	return 0;
}
/**************************************************************************
 * General Driver Functions - ENTIRE REGISTERS
 **************************************************************************/
/*
 * si470x_get_all_registers - read entire registers
 */
static int si470x_get_all_registers(struct si470x_device *radio)
{
	int i;
	u16 buf[READ_REG_NUM];
	struct i2c_msg msgs[1] = {
		{ radio->client->addr, I2C_M_RD, sizeof(u16) * READ_REG_NUM,
			(void *)buf },
	};
#if 0
	printk(KERN_ERR"[FM]  radio->client->addr=%x \n",
			(unsigned int)radio->client->addr);
	printk(KERN_ERR"[FM]  radio->client->adapter=%x\n",
			(unsigned int)radio->client->adapter);
	printk(KERN_ERR"[FM] i2c read! start !!!!!!!!!!!\n");
#endif
	if (i2c_transfer(radio->client->adapter, msgs, 1) != 1)
		return -EIO;
	for (i = 0; i < READ_REG_NUM; i++) {
		radio->registers[i] = __be16_to_cpu(buf[READ_INDEX(i)]);
#if 0
		printk(KERN_ERR"[FM] Reg %d=%x\n", i, radio->registers[i]);
#endif
	}
	return 0;
}
/**************************************************************************
 * General Driver Functions - DISCONNECT_CHECK
 **************************************************************************/
/*
 * si470x_disconnect_check - check whether radio disconnects
 */
int si470x_disconnect_check(struct si470x_device *radio)
{
	return 0;
}
/**************************************************************************
 * File Operations Interface
 **************************************************************************/
/*
 * si470x_fops_open - file open
 */
static int si470x_fops_open(struct file *file)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;
	mutex_lock(&radio->lock);
	radio->users++;
	if (radio->users == 1)
		/* start radio */
		retval = si470x_start(radio);
	mutex_unlock(&radio->lock);
	return retval;
}
/*
 * si470x_fops_release - file release
 */
static int si470x_fops_release(struct file *file)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;
	/* safety check */
	if (!radio)
		return -ENODEV;
	mutex_lock(&radio->lock);
	radio->users--;
	if (radio->users == 0)
		/* stop radio */
		retval = si470x_stop(radio);
	mutex_unlock(&radio->lock);
	return retval;
}
/*
 * si470x_fops - file operations interface
 */
const struct v4l2_file_operations si470x_fops = {
	.owner          = THIS_MODULE,
	.ioctl          = video_ioctl2,
	.open           = si470x_fops_open,
	.release        = si470x_fops_release,
};
/**************************************************************************
 * Video4Linux Interface
 **************************************************************************/
/*
 * si470x_vidioc_querycap - query device capabilities
 */
int si470x_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *capability)
{
	strlcpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
	strlcpy(capability->card, DRIVER_CARD, sizeof(capability->card));
	capability->version = DRIVER_KERNEL_VERSION;
	capability->capabilities = V4L2_CAP_HW_FREQ_SEEK |
		V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	return 0;
}
/**************************************************************************
 * I2C Interface
 **************************************************************************/
/*
 * si470x_i2c_probe - probe for the device
 */
static int si470x_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int retval = 0;
	unsigned char version_warning = 0;
	printk(KERN_ERR"[FM] i2c probe!!!!!!\n");
	/* private data allocation and initialization */
	radio = kzalloc(sizeof(struct si470x_device), GFP_KERNEL);
	if (!radio) {
		retval = -ENOMEM;
		goto err_initial;
	}
	radio->users = 0;
	radio->client = client;
	mutex_init(&radio->lock);
	/* video device allocation and initialization */
	radio->videodev = video_device_alloc();
	if (!radio->videodev) {
		retval = -ENOMEM;
		goto err_radio;
	}
	memcpy(radio->videodev, &si470x_viddev_template,
			sizeof(si470x_viddev_template));
	video_set_drvdata(radio->videodev, radio);


	radio->registers[POWERCFG] = POWERCFG_DISABLE | 1;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		goto err_all;
	}
	msleep(110);

	/* power up : need 110ms */
	radio->registers[POWERCFG] = POWERCFG_ENABLE & ~POWERCFG_DISABLE;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		goto err_all;
	}
	msleep(110);

	/* get device and chip versions */
	if (si470x_get_all_registers(radio) < 0) {
		retval = -EIO;
		goto err_video;
	}
	dev_info(&client->dev, "DeviceID=0x%4.4hx ChipID=0x%4.4hx\n",
			radio->registers[DEVICEID], radio->registers[CHIPID]);
	if ((radio->registers[CHIPID] & CHIPID_FIRMWARE) < RADIO_FW_VERSION) {
		dev_warn(&client->dev,
				"This driver is known to work with "
				"firmware version %hu,\n", RADIO_FW_VERSION);
		dev_warn(&client->dev,
				"but the device has firmware version %hu.\n",
				radio->registers[CHIPID] & CHIPID_FIRMWARE);
		version_warning = 1;
	}

	/* give out version warning */
	if (version_warning == 1) {
		dev_warn(&client->dev,
				"If you have some trouble using this driver,\n");
		dev_warn(&client->dev,
				"please report to V4L ML at "
				"linux-media@vger.kernel.org\n");
	}


	radio->registers[POWERCFG] |= POWERCFG_DMUTE | POWERCFG_RDSM;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		goto err_all;
	}
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_RDS;
	if(si470x_set_register(radio, SYSCONFIG1) < 0){
		retval = -EIO;
		goto err_all;
	}
	radio->registers[SYSCONFIG2] = SYSCONFIG2_VOLUME;
	if (si470x_set_register(radio, SYSCONFIG2) < 0) {
		retval = -EIO;
		goto err_all;
	}

	radio->registers[SYSCONFIG3] |= 0x00; //SKSNR, SKCNT
	if(si470x_set_register(radio, SYSCONFIG3) < 0) {
		retval = -EIO;
		goto err_all;
	}

	/* set initial frequency */
	si470x_set_freq(radio, 87.5 * FREQ_MUL);

	/* register video device */
	retval = video_register_device(radio->videodev, VFL_TYPE_RADIO,
			radio_nr);
	if (retval) {
		dev_warn(&client->dev, "Could not register video device\n");
		goto err_all;
	}
	i2c_set_clientdata(client, radio);

	gpio_direction_output(31,0);
	printk("[FM] power down...\n");
	msleep(110);

	return 0;

err_all:
err_video:
	video_device_release(radio->videodev);
err_radio:
	kfree(radio);
err_initial:
	return retval;
}

void si470x_set_power_up(void){
	int retval = 0;

	gpio_direction_output(31,0);
	msleep(110);
	gpio_direction_output(31,1);
	printk("[FM] supply power \n");

	radio->registers[POWERCFG] = POWERCFG_DISABLE | 1;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		printk("[FM] power up fail <1>!!!\n");
		goto err_all;
	}
	msleep(110);

	/* power up : need 110ms */
	radio->registers[POWERCFG] = POWERCFG_ENABLE & ~POWERCFG_DISABLE;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		printk("[FM] power up fail <2>!!!\n");
		goto err_all;
	}
	msleep(110);

	radio->registers[POWERCFG] |= POWERCFG_DMUTE | POWERCFG_RDSM;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		printk("[FM] Dmute fail!!!\n");
		goto err_all;
	}
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_RDS;
	if(si470x_set_register(radio, SYSCONFIG1) < 0){
		retval = -EIO;
		goto err_all;
	}
	radio->registers[SYSCONFIG2] = SYSCONFIG2_VOLUME;
	if (si470x_set_register(radio, SYSCONFIG2) < 0) {
		retval = -EIO;
		printk("[FM] Set vol fail!!!\n");
		goto err_all;
	}
	radio->registers[SYSCONFIG3] |= 0x00; //SKSNR, SKCNT
	if(si470x_set_register(radio, SYSCONFIG3) < 0) {
		retval = -EIO;
		printk("[FM] Set SNR fail!!!\n");
		goto err_all;
	}

	/* set initial frequency */
	si470x_set_freq(radio, 87.5 * FREQ_MUL);

	return;
err_all:
	video_device_release(radio->videodev);
}

void si470x_set_power_down(void){
	int retval;

	/* sysconfig 1 */
	radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_RDS;
	retval = si470x_set_register(radio, SYSCONFIG1);
	if (retval < 0){
		printk(KERN_ERR"[FM] power down fail!!!\n");
		goto done;
	}

	/* powercfg */
	radio->registers[POWERCFG] &= ~POWERCFG_DMUTE;
	/* POWERCFG_ENABLE has to automatically go low */
	radio->registers[POWERCFG] |= POWERCFG_ENABLE |	POWERCFG_DISABLE;
	retval = si470x_set_register(radio, POWERCFG);

	gpio_direction_output(31,0);
	printk(KERN_ERR"[FM] power down...\n");
	msleep(110);

done:
	printk(KERN_ERR"out of power down\n");
}

static void si470x_set_volume(int vol){
	int retval = 0;
	if(vol>15 && vol<0){
		vol=15;
	}
	radio->registers[SYSCONFIG2] &= ~SYSCONFIG2_VOLUME;
	radio->registers[SYSCONFIG2] |= vol;

	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		printk(KERN_ERR"[FM] Set reg Fail!!!\n");
	}
}

static void si470x_set_stereo(int s){
	int retval = 0;

	switch(s){
	case 1:
		radio->registers[POWERCFG] |= POWERCFG_MONO;
		printk(KERN_ERR"[FM] Set FM audio to MONO");
		break;
	default:
		radio->registers[POWERCFG] &= ~POWERCFG_MONO;
		printk(KERN_ERR"[FM] Set FM audio to stereo");
		break;
	}

	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		printk(KERN_ERR"[FM] Set FM audio Fail!!!\n");
	}
}

static unsigned int seek_timeout = 5000;
static int si470x_seek(struct si470x_device *radio, unsigned int wrap_around, unsigned int seek_upward) {
        int retval = 0;
	unsigned long timeout;
	bool timed_out = 0;

        radio->registers[POWERCFG] |= POWERCFG_SEEK;
        if (wrap_around == 1)
                radio->registers[POWERCFG] &= ~POWERCFG_SKMODE;
        else
                radio->registers[POWERCFG] |= POWERCFG_SKMODE;

        if (seek_upward == 1)
                radio->registers[POWERCFG] |= POWERCFG_SEEKUP;
        else
                radio->registers[POWERCFG] &= ~POWERCFG_SEEKUP;

        retval = si470x_set_register(radio, POWERCFG);

        if (retval < 0)
                goto done;

	/* wait till tune operation has completed */
	timeout = jiffies + msecs_to_jiffies(seek_timeout);
        do {
                retval = si470x_get_register(radio, STATUSRSSI);
                if (retval < 0)
                        goto stop;

		timed_out = time_after(jiffies, timeout);
        } while ((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0 && (!timed_out));
	si470x_get_register(radio, READCHAN);
	printk("ch=%d, RSSI=%d\n", radio->registers[READCHAN]&READCHAN_READCHAN, radio->registers[STATUSRSSI]&STATUSRSSI_RSSI);

        if ((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0)
                dev_warn(&radio->videodev->dev, "seek does not complete\n");

        if (radio->registers[STATUSRSSI] & STATUSRSSI_SF)
                dev_warn(&radio->videodev->dev, "seek failed / band limit reached\n");
		
stop:
        /* stop seeking */
        radio->registers[POWERCFG] &= ~POWERCFG_SEEK;
        retval = si470x_set_register(radio, POWERCFG);

done:
        /* try again, if timed out */
        if (retval == 0)
                retval = -EAGAIN;

        return retval;
}

static unsigned int tune_timeout = 3000;
static int si470x_set_chan(struct si470x_device *radio, unsigned short chan)
{
	int retval;
	unsigned long timeout;
	bool timed_out = 0;

	/* start tuning */
	radio->registers[CHANNEL] &= ~CHANNEL_CHAN;
	radio->registers[CHANNEL] |= CHANNEL_TUNE | chan;
	retval = si470x_set_register(radio, CHANNEL);
	if (retval < 0)
		goto done;

	/* wait till tune operation has completed */
	timeout = jiffies + msecs_to_jiffies(tune_timeout);
	do {
		retval = si470x_get_register(radio, STATUSRSSI);
		if (retval < 0)
			goto stop;
		timed_out = time_after(jiffies, timeout);
	} while (((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0) &&
		(!timed_out));
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0)
		dev_warn(&radio->videodev->dev, "tune does not complete\n");
	if (timed_out)
		dev_warn(&radio->videodev->dev,
			"tune timed out after %u ms\n", tune_timeout);

stop:
	/* stop tuning */
	radio->registers[CHANNEL] &= ~CHANNEL_TUNE;
	retval = si470x_set_register(radio, CHANNEL);

done:
	return retval;
}

struct RDS_buf{
	int i;
	unsigned short rds[4][4];
};

static short chan;
static char rdsData[10] = "";
static unsigned short buf[4] = {0};
void si470x_get_rds(struct si470x_device *radio) {
	if (si470x_get_all_registers(radio) < 0)
		printk(KERN_ERR"[FM] Show RDS reg Fail!!!\n");

	/* Droping data if RDS got error */
	if(((radio->registers[STATUSRSSI]&STATUSRSSI_BLERA)>>9) ||
		((radio->registers[READCHAN]&READCHAN_BLERB) >> 14) ||
		((radio->registers[READCHAN]&READCHAN_BLERC) >> 12) ||
		((radio->registers[READCHAN]&READCHAN_BLERD) >> 10) ) {
			printk("[FM] RDS ERROR !!!");
			radio->registers[RDSA] =0;
			radio->registers[RDSB] =0;
			radio->registers[RDSC] =0;
			radio->registers[RDSD] =0;
			if((si470x_set_register(radio, RDSA)<0) || (si470x_set_register(radio, RDSB)<0) ||
				(si470x_set_register(radio, RDSC)<0) || (si470x_set_register(radio, RDSD)<0))
				printk("[FM] reset RDS data fail\n");

			return ;
	}

	/* If channel changed, we reallocate memory to buffer */
	if(chan != (radio->registers[READCHAN]&READCHAN_READCHAN)){
		rb = kzalloc(sizeof(struct RDS_buf), GFP_KERNEL);
		chan = radio->registers[READCHAN]&READCHAN_READCHAN;
		printk("chan=%d", chan);
	}

	if( ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSR)>>15) &&
		((radio->registers[STATUSRSSI] & STATUSRSSI_RDSS)>>11) ) {
		if((((radio->registers[RDSB]>>11)&0x1F)==0x00) ||
			(((radio->registers[RDSB]>>11)&0x1F)==0x01) ){
		unsigned short s1, s3, s4, pty;
		char c1, c2;
		int x, y=0;
		buf[0] = radio->registers[RDSA];
		buf[1] = radio->registers[RDSB];
		buf[2] = radio->registers[RDSC];
		buf[3] = radio->registers[RDSD];
		s1 = buf[0];
		pty = buf[1]&0x03;
		s3 = buf[2];
		s4 = buf[3];

		printk(KERN_ERR"[FM] RDS buf num=%d\n", rb->i);
		if(rb->rds[0][0]==0){ //if buf empty
			memcpy(rb->rds[rb->i], buf, 10);
			rb->i++;
		}
		else {
			if(s1 != rb->rds[0][0]){ //if it is not the current frequency
				while(rb->i > 0){ //clear
					for(x=0; x<4; x++)
						rb->rds[rb->i][x] = 0;

					rb->i--;
				}
				memcpy(rb->rds[rb->i], buf, 8);
				rb->i++;
				return;
			}

			/* Update info */
			for(x=0; x<rb->i; x++){
				printk(KERN_ERR"[FM] RDS: rb->rds[x][1]&0x03=%d, pty=%d\n", rb->rds[x][1]&0x03, pty);
				if(((rb->rds[x][1]&0x03)==pty) && (rb->rds[x][0])) {
					rb->rds[x][3] = s4;
					goto readRDS;
				}
			}

			memcpy(rb->rds[rb->i], buf, 8);
			printk(KERN_ERR"[FM] RDS: %x, %x, %x, %x\n", rb->rds[rb->i][0], rb->rds[rb->i][1], rb->rds[rb->i][2], rb->rds[rb->i][3]);
			rb->i++;
readRDS:
			if(rb->i < 4) return;

			printk(KERN_ERR"[FM] RDS Before sorting!!!\n");
			printk(KERN_ERR"[FM] RDS: %x, %x, %x, %x\n", rb->rds[0][3], rb->rds[1][3], rb->rds[2][3], rb->rds[3][3]);
			printk(KERN_ERR"[FM] RDS pty= %x, %x, %x, %x\n", rb->rds[0][1]&0x03, rb->rds[1][1]&0x03, rb->rds[2][1]&0x03, rb->rds[3][1]&0x03);
			for(x=0; x<rb->i; x++){ //sort
				for(y=x+1; y<rb->i; y++){
					if((rb->rds[x][1]&0x03) > (rb->rds[y][1]&0x03)){
						unsigned short tmp[4] = {0};
						memcpy(tmp, rb->rds[y], 8);
						memcpy(rb->rds[y], rb->rds[x], 8);
						memcpy(rb->rds[x], tmp, 8);
					}
				}
			}
			printk(KERN_ERR"[FM] RDS After sorting!!!\n");
			printk(KERN_ERR"[FM] RDS: %x, %x, %x, %x\n", rb->rds[0][3], rb->rds[1][3], rb->rds[2][3], rb->rds[3][3]);
			printk(KERN_ERR"[FM] RDS pty= %x, %x, %x, %x\n", rb->rds[0][1]&0x03, rb->rds[1][1]&0x03, rb->rds[2][1]&0x03, rb->rds[3][1]&0x03);

			for(x=0,y=0; x<rb->i; x++,y++){
				c1 = rb->rds[x][3]>>8;
				strncat(rdsData, &c1, 1);
				c2 = rb->rds[x][3]&0xFF;
				strncat(rdsData, &c2, 1);

				radio->rds_PS[y] = rb->rds[x][3];
				printk(KERN_ERR"rds_PS[%d]=%x   rb->rds[%d][3]=%x\n",y,radio->rds_PS[y],x,rb->rds[x][3]);
			}
			printk(KERN_ERR"[FM] RDS Data= %s\n", rdsData);

			while(rb->i > 0){ //reset
				rb->i--;
				for(x=0; x<4; x++)
					rb->rds[rb->i][x] = 0;
			}

			/* reset RDS data*/
			memcpy(rdsData, "", 10);

			radio->rds_valid = true;
			return;
		}
	    }
	}
	else {
		radio->registers[RDSA] =0;
		radio->registers[RDSB] =0;
		radio->registers[RDSC] =0;
		radio->registers[RDSD] =0;
		if((si470x_set_register(radio, RDSA)<0) || (si470x_set_register(radio, RDSB)<0) ||
			(si470x_set_register(radio, RDSC)<0) || (si470x_set_register(radio, RDSD)<0)) {
			printk(KERN_ERR"[FM] reset RDS data fail\n");
			return;
		}
		printk(KERN_ERR"[FM] RDS info not available!!!\n");
	}
}

struct RDS_RT_buf* rtb;

struct RDS_RT_buf{
	int i;
	unsigned short rds[][4];
};

bool showRDS_RT;
static char rdsRTData[20] = "";
static unsigned short rtbuf[4] = {0};
static unsigned short rt_buf[4] = {0};
static void si470x_get_rds_rt(struct si470x_device *radio) { //RDS radio text
	if (si470x_get_all_registers(radio) < 0)
		printk(KERN_ERR"[FM] Show RDS reg Fail!!!\n");
	if((((radio->registers[RDSB]>>11)&0x1F) != 0x04) ||
		(((radio->registers[RDSB]>>11)&0x1F) != 0x05))
		printk(KERN_ERR"[FM] No Radio Text info!!!\n");

	switch(((radio->registers[RDSB]>>11)&0x1F)){
	  case 0x04: {
	     printk("[FM] RT Group Type 2A\r\n");

		/* Droping data if RDS got error */
		if(((radio->registers[STATUSRSSI]&STATUSRSSI_BLERA)>>9) ||
			((radio->registers[READCHAN]&READCHAN_BLERB) >> 14) ||
			((radio->registers[READCHAN]&READCHAN_BLERC) >> 12) ||
			((radio->registers[READCHAN]&READCHAN_BLERD) >> 10) ) {
				printk("[FM] RDS RT ERROR !!!");
				radio->registers[RDSA] =0;
				radio->registers[RDSB] =0;
				radio->registers[RDSC] =0;
				radio->registers[RDSD] =0;
				if((si470x_set_register(radio, RDSA)<0) || (si470x_set_register(radio, RDSB)<0) ||
					(si470x_set_register(radio, RDSC)<0) || (si470x_set_register(radio, RDSD)<0))
					printk("[FM] reset RDS data fail\n");

				return ;
		}

		/* If channel changed, we reallocate memory to buffer */
		if(chan != (radio->registers[READCHAN]&READCHAN_READCHAN)){
			rtb = kzalloc(sizeof(struct RDS_RT_buf), GFP_KERNEL);
			chan = radio->registers[READCHAN]&READCHAN_READCHAN;
			printk("chan=%d", chan);
		}

		if( ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSR)>>15) &&
			((radio->registers[STATUSRSSI] & STATUSRSSI_RDSS)>>11) ) {
			unsigned short s1, s3, s4, pty;
			char c1, c2;
			int x, y;
			rtbuf[0] = radio->registers[RDSA];
			rtbuf[1] = radio->registers[RDSB];
			rtbuf[2] = radio->registers[RDSC];
			rtbuf[3] = radio->registers[RDSD];
			s1 = rtbuf[0];
			pty = rtbuf[1]&0x0F;
			s3 = rtbuf[2];
			s4 = rtbuf[3];

	              int iCurretnTextAB = (rtbuf[1]>>4 & 0x01);
	              static int iPreTextAB;

	              if ( (pty & 0x0F) == 0x0D) {
	                printk("Get 0x0D To Show RT......\n");
	                showRDS_RT = false;
	              }

			if( (rtbuf[2]>>8& 0xFF) <= 0x1F || (rtbuf[2]& 0xFF) <= 0x1F || (rtbuf[2]>>8& 0xFF)>=0x80 || (rtbuf[2] & 0xFF)>=0x80 
			|| (rtbuf[3]>>8& 0xFF) <= 0x1F || (rtbuf[3]& 0xFF) <= 0x1F || (rtbuf[3]>>8& 0xFF)>=0x80 || (rtbuf[3] & 0xFF)>=0x80){
	                printk("===========(1)exit here");
			  return;
			}

			printk(KERN_ERR"[FM] RDS RT buf num=%d\n", rb->i);
			if(rtb->rds[0][0]==0){ //if buf empty
				iPreTextAB = iCurretnTextAB;
				memcpy(rtb->rds[rtb->i], rtbuf, 10);
				rtb->i++;
			}
			else {
				int iRTLastIndex = 0;
				bool bRTIsNoGap;

				if(s1 != rtb->rds[0][0]){ //if it is not the current frequency
					while(rtb->i > 0){ //clear
						for(x=0; x<4; x++)
							rtb->rds[rtb->i][x] = 0;

						rtb->i--;
					}
					memcpy(rtb->rds[rtb->i], rtbuf, 10);
					rtb->i++;
					showRDS_RT = false;
					return;
				}

				/* Update info */
				for(x=0; x<rtb->i; x++){
					if(iPreTextAB != iCurretnTextAB){
						printk("Clean RT to receive Next RT......\n");
						while(rtb->i > 0){ //clear
							for(x=0; x<4; x++)
								rtb->rds[rtb->i][x] = 0;

							rtb->i--;
						}
						memcpy(rtb->rds[rtb->i], rtbuf, 10);
						rtb->i++;
						iPreTextAB = iCurretnTextAB;
						return;
					}
					else if((rtb->rds[x][1]&0x0F) == (pty&0x0F)) {
						printk(KERN_ERR"[FM] RDS: rb->rds[x][1]&0x03=%d, pty=%d\n", rtb->rds[x][1]&0x0F, pty);
						rt_buf[2] = rtbuf[2];
						rt_buf[3] = rtbuf[3];
						goto readRDS_2A;
					}
				}

				memcpy(rtb->rds[rtb->i], rtbuf, 8);
				rtb->i++;

readRDS_2A:
				printk(KERN_ERR"[FM] RT Before sorting!!!\n");
				unsigned short tmp[4] = {0};
				for(x=0; x<rtb->i; x++){ //sort
					for(y=x+1; y<rtb->i; y++){
						if((rtb->rds[x][1]&0x03) > (rtb->rds[y][1]&0x03)){
							memcpy(tmp, rtb->rds[y], 8);
							memcpy(rtb->rds[y], rtb->rds[x], 8);
							memcpy(rtb->rds[x], tmp, 8);
						}
					}
				}
				printk(KERN_ERR"[FM] RT After sorting!!!\n");

				for(x=0; x<rtb->i; x++)
					iRTLastIndex = rtb->rds[x][1]&0x0F;

				if(rtb->i == iRTLastIndex+1)
					bRTIsNoGap = true;
				else
					bRTIsNoGap = false;

				if ( (showRDS_RT == true) && bRTIsNoGap == true ) {
					for(x=0; x<rtb->i; x++){
						c1 = rtb->rds[x][2]>>8;
						strncat(rdsData, &c1, 1);
						c2 = rtb->rds[x][2]&0xFF;
						strncat(rdsData, &c2, 1);
						c1 = rtb->rds[x][3]>>8;
						strncat(rdsData, &c1, 1);
						c2 = rtb->rds[x][3]&0xFF;
						strncat(rdsData, &c2, 1);
					}
					printk(KERN_ERR"[FM] RDS RT Data= %s\n", rdsRTData);
				}
				else
					return;
				}
		} else {
			radio->registers[RDSA] =0;
			radio->registers[RDSB] =0;
			radio->registers[RDSC] =0;
			radio->registers[RDSD] =0;
			if((si470x_set_register(radio, RDSA)<0) || (si470x_set_register(radio, RDSB)<0) ||
				(si470x_set_register(radio, RDSC)<0) || (si470x_set_register(radio, RDSD)<0)) {
				printk(KERN_ERR"[FM] reset RDS RT data fail\n");
				return;
			}
			printk(KERN_ERR"[FM] RDS RT info not available!!!\n");
		}
	}
	break;


       case 0x05: {
		printk("[FM] RT Group Type 2B\r\n");
		if (si470x_get_all_registers(radio) < 0)
			printk(KERN_ERR"[FM] Show RDS reg Fail!!!\n");

		/* Droping data if RDS got error */
		if(((radio->registers[STATUSRSSI]&STATUSRSSI_BLERA)>>9) ||
			((radio->registers[READCHAN]&READCHAN_BLERB) >> 14) ||
			((radio->registers[READCHAN]&READCHAN_BLERC) >> 12) ||
			((radio->registers[READCHAN]&READCHAN_BLERD) >> 10) ) {
				printk("[FM] RDS RT ERROR !!!");
				radio->registers[RDSA] =0;
				radio->registers[RDSB] =0;
				radio->registers[RDSC] =0;
				radio->registers[RDSD] =0;
				if((si470x_set_register(radio, RDSA)<0) || (si470x_set_register(radio, RDSB)<0) ||
					(si470x_set_register(radio, RDSC)<0) || (si470x_set_register(radio, RDSD)<0))
					printk("[FM] reset RDS data fail\n");

				return ;
		}

		/* If channel changed, we reallocate memory to buffer */
		if(chan != (radio->registers[READCHAN]&READCHAN_READCHAN)){
			rtb = kzalloc(sizeof(struct RDS_RT_buf), GFP_KERNEL);
			chan = radio->registers[READCHAN]&READCHAN_READCHAN;
			printk("chan=%d", chan);
		}

		if( ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSR)>>15) &&
			((radio->registers[STATUSRSSI] & STATUSRSSI_RDSS)>>11) ) {
			unsigned short s1, s3, s4, pty;
			char c1, c2;
			int x, y;
			rtbuf[0] = radio->registers[RDSA];
			rtbuf[1] = radio->registers[RDSB];
			rtbuf[2] = radio->registers[RDSC];
			rtbuf[3] = radio->registers[RDSD];
			s1 = rtbuf[0];
			pty = rtbuf[1]&0x1F;
			s3 = rtbuf[2];
			s4 = rtbuf[3];

	              int iCurretnTextAB = (rtbuf[1]>>4 & 0x01);
	              static int iPreTextAB;

			if( (rtbuf[2]>>8& 0xFF) <= 0x1F || (rtbuf[2]& 0xFF) <= 0x1F || (rtbuf[2]>>8& 0xFF)>=0x80 || (rtbuf[2] & 0xFF)>=0x80 
			|| (rtbuf[3]>>8& 0xFF) <= 0x1F || (rtbuf[3]& 0xFF) <= 0x1F || (rtbuf[3]>>8& 0xFF)>=0x80 || (rtbuf[3] & 0xFF)>=0x80){
		                return;

	              if ( (pty & 0x0F) == 0x0D) {
	                printk("Get 0x0D To Show RT......\n");
	                showRDS_RT = true;
	              }

			printk(KERN_ERR"[FM] RDS RT buf num=%d\n", rb->i);
			if(rtb->rds[0][0]==0){ //if buf empty
				iPreTextAB = iCurretnTextAB;
				memcpy(rtb->rds[rtb->i], rtbuf, 10);
				rtb->i++;
			}
			else {
				int iRTLastIndex = 0;
				bool bRTIsNoGap;

				if(s1 != rtb->rds[0][0]){  //if it is not the current frequency
					while(rtb->i > 0){ //clear
						for(x=0; x<4; x++)
							rtb->rds[rtb->i][x] = 0;

						rtb->i--;
					}
					memcpy(rtb->rds[rtb->i], rtbuf, 10);
					rtb->i++;
					showRDS_RT = false;
					return;
				}

				/* Update info */
				for(x=0; x<rtb->i; x++){
					if((rtbuf[1]>>4 & 0x01) != (rtb->rds[rtb->i][1]>>4 & 0x01)){ //if flag different
						while(rtb->i > 0){ //clear
							for(x=0; x<4; x++)
								rtb->rds[rtb->i][x] = 0;

							rtb->i--;
						}
						memcpy(rtb->rds[rtb->i], rtbuf, 10);
						rtb->i++;
						iPreTextAB = iCurretnTextAB;
						return;
					}
					if((rtb->rds[x][1]&0x0F) == (pty&0x0F)) {
						printk(KERN_ERR"[FM] RDS: rb->rds[x][1]&0x03=%d, pty=%d\n", rtb->rds[x][1]&0x0F, pty);
						rt_buf[2] = rtbuf[2];
						rt_buf[3] = rtbuf[3];
						goto readRDS_2B;
					}
				}

				memcpy(rtb->rds[rtb->i], rtbuf, 8);
				rtb->i++;

readRDS_2B:  if((showRDS_RT == true) && (rtb->i >= 14)){
				printk(KERN_ERR"[FM] RT Before sorting!!!\n");
				unsigned short tmp[4] = {0};
				for(x=0; x<rtb->i; x++){ //sort
					for(y=x+1; y<rtb->i; y++){
						if((rtb->rds[x][1]&0x03) > (rtb->rds[y][1]&0x03)){
							memcpy(tmp, rtb->rds[y], 8);
							memcpy(rtb->rds[y], rtb->rds[x], 8);
							memcpy(rtb->rds[x], tmp, 8);
						}
					}
				}
				printk(KERN_ERR"[FM] RT After sorting!!!\n");

				for(x=0; x<rtb->i; x++){
					c1 = rtb->rds[x][2]>>8;
					strncat(rdsData, &c1, 1);
					c2 = rtb->rds[x][2]&0xFF;
					strncat(rdsData, &c2, 1);
					c1 = rtb->rds[x][3]>>8;
					strncat(rdsData, &c1, 1);
					c2 = rtb->rds[x][3]&0xFF;
					strncat(rdsData, &c2, 1);
				}
				printk(KERN_ERR"[FM] RDS RT Data= %s\n", rdsRTData);

			  } else {
				return;
			  }
		    }
			}else {
			radio->registers[RDSA] =0;
			radio->registers[RDSB] =0;
			radio->registers[RDSC] =0;
			radio->registers[RDSD] =0;
			if((si470x_set_register(radio, RDSA)<0) || (si470x_set_register(radio, RDSB)<0) ||
				(si470x_set_register(radio, RDSC)<0) || (si470x_set_register(radio, RDSD)<0)) {
				printk(KERN_ERR"[FM] reset RDS RT data fail\n");
				return;
			}
			printk(KERN_ERR"[FM] RDS RT info not available!!!\n");
		    }
		}
		break;
	}
  }
}


/*
 * si470x_vidioc_s_tuner - set tuner attributes
 */
static int si470x_set_region(int region)
{
	int retval = -EINVAL;
	int retval2 = -EINVAL;

	switch(region) {
	case 1: //EU
		radio->registers[SYSCONFIG1] |= SYSCONFIG1_DE; //50us
		radio->registers[SYSCONFIG2] |= 0x00; // 87.5~108MHz
		radio->registers[SYSCONFIG2] |= 0x10; //100 KHz
		break;
	case 2: //JP
		radio->registers[SYSCONFIG1] |= SYSCONFIG1_DE; //50us
		radio->registers[SYSCONFIG2] |= 0x40; // 76~108MHz
		radio->registers[SYSCONFIG2] |= 0x10; //50 KHz
		break;
	case 0: //US
	default:
		radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_DE; //75us
		radio->registers[SYSCONFIG2] |= 0x00; // 87.5~108MHz
		radio->registers[SYSCONFIG2] |= 0x00; //200 KHz
		break;
	}

	retval = si470x_set_register(radio, SYSCONFIG1);
	if (retval < 0)
		goto done;
	retval2 = si470x_set_register(radio, SYSCONFIG2);
	if (retval2 < 0)
		goto done;

done:
	if (retval < 0)
		dev_warn(&radio->videodev->dev,
			"set tuner spaceor band failed with %d\n", retval);
	else if (retval2 < 0)
		dev_warn(&radio->videodev->dev,
			"set tuner de failed with %d\n", retval2);

	return retval;
}


/*
 * si470x_i2c_remove - remove the device
 */
static int si470x_i2c_remove(struct i2c_client *client)
{
	struct si470x_device *radio = i2c_get_clientdata(client);
	video_unregister_device(radio->videodev);
	kfree(radio);
	i2c_set_clientdata(client, NULL);
	return 0;
}
/*
 * si470x_i2c_driver - i2c driver interface
 */
static struct i2c_driver si470x_i2c_driver = {
	.driver = {
		.name           = "si470x-fm",
		.owner          = THIS_MODULE,
	},
	.probe                  = si470x_i2c_probe,
	.remove                 = si470x_i2c_remove,
	.id_table               = si470x_i2c_id,
};
/**************************************************************************
 * Module Interface
 **************************************************************************/
/*
 * si470x_i2c_init - module init
 */
static int __init si470x_i2c_init(void)
{
	int ret = 0;
//	printk(KERN_INFO DRIVER_DESC ", Version " DRIVER_VERSION "\n");
#if 0
	printk(KERN_ERR"[FM] init!!!\n");
#endif
	ret = i2c_add_driver(&si470x_i2c_driver);
	if (ret)
		printk(KERN_ERR"[FM] I2C register driver fail!!!!!!!!!\n");
	return ret;
}
/*
 * si470x_i2c_exit - module exit
 */
static void __exit si470x_i2c_exit(void)
{
	i2c_del_driver(&si470x_i2c_driver);
}
module_init(si470x_i2c_init);
module_exit(si470x_i2c_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);

#if defined(CONFIG_DEBUG_FS)
static void fm_debug_show_reg(void){
	int retval = 0;
	msleep(110);
	/* get device and chip versions */
	if (si470x_get_all_registers(radio) < 0) {
		retval = -EIO;
		printk(KERN_ERR"[FM] Show reg Fail!!!\n");
	}
}

static int fm_vol_debug_set(void *data, u64 val)
{
	si470x_set_volume(val);
	return 0;
}

static int fm_vol_debug_get(void *data, u64 *val)
{
	fm_debug_show_reg();
	printk(KERN_ERR"[FM] Vol=%d\n",radio->registers[SYSCONFIG2] & SYSCONFIG2_VOLUME);
	return 0;
}

static int fm_stereo_debug_set(void *data, u64 val)
{
	si470x_set_stereo(val);
	return 0;
}

static int fm_stereo_debug_get(void *data, u64 *val)
{
	fm_debug_show_reg();
	printk(KERN_ERR"[FM] Set audio Mono=%d\n",(radio->registers[POWERCFG] & POWERCFG_MONO)>>13);
	return 0;
}

static int fm_freq_debug_get(void *data, u64 *val)
{
	unsigned int f;
	si470x_get_freq(radio,&f);
	printk(KERN_ERR"[FM] Freq=%d\n",f);
	return 0;
}
static int fm_freq_debug_set(void *data, u64 val)
{
	printk(KERN_ERR"[FM] set FREQ=%d\n",(unsigned int)(val * 160));
	si470x_set_freq(radio, val * 160);
	return 0;
}

static int fm_seek_debug_get(void *data, u64 *val)
{
	unsigned int f;
        si470x_get_freq(radio,&f);
        printk(KERN_ERR"[FM] Freq=%d, ch=%d\n", f, radio->registers[READCHAN]&READCHAN_READCHAN);
	return 0;
}

static int fm_seek_debug_set(void *data, u64 val)
{
	switch(val){
	case 0:
		si470x_seek(radio, 1, 0);
		break;
	case 1:
	default:
		si470x_seek(radio, 1, 1);
		break;
	}
	return 0;
}

static int fm_power_debug_get(void *data, u64 *val)
{
	fm_debug_show_reg();
	printk("[FM] FM_power = %s\n", fm_power?"on":"off");
	return 0;
}

static int fm_power_debug_set(void *data, u64 val)
{
	switch(val){
	case 0:
		si470x_set_power_down();
		fm_power = 0;
		printk("[FM] Set FM power down!!!\n");
		break;
	case 1:
		si470x_set_power_up();
		fm_power = 1;
		printk("[FM] Set FM power up!!!\n");
		break;
	default:
		printk("[FM] you set wrong number to fm power!!!\n");
	}
	return 0;
}

static int fm_rds_debug_set(void *data, u64 val)
{
	int i=0;
	do{
		si470x_get_rds(radio);
	}while(i++<val);
//	if (si470x_get_all_registers(radio) < 0)
//		printk(KERN_ERR"[FM] Show RDS reg Fail!!!\n");

	return 0;
}

static int fm_rds_debug_get(void *data, u64 *val)
{
	si470x_get_rds(radio);
	return 0;
}

static int fm_rdsrt_debug_set(void *data, u64 val)
{
	int i=0;
	do{
		si470x_get_rds_rt(radio);
	}while(i++<val);

	return 0;
}

static int fm_rdsrt_debug_get(void *data, u64 *val)
{
	si470x_get_rds_rt(radio);
	return 0;
}

static int fm_ch_debug_set(void *data, u64 val){
	unsigned int f;
	short ch;
	si470x_get_register(radio, READCHAN);
	ch =  radio->registers[READCHAN]&READCHAN_READCHAN;
	switch(val){
	case 0:
		ch--;
		if(ch<0)
			ch = (short)((108-87.5)/0.2);
		break;
	case 1:
	default:
		ch++;
		if(ch> (short)((108-87.5)/0.2))
			ch = 0;
		break;
	}
	si470x_set_chan(radio, ch);
       si470x_get_freq(radio,&f);
       printk(KERN_ERR"[FM] Freq=%d, ch=%d\n", f, radio->registers[READCHAN]&READCHAN_READCHAN);
	return 0;
}

static int fm_ch_debug_get(void *data, u64 *val){
	unsigned int f;
       si470x_get_freq(radio,&f);
       printk(KERN_ERR"[FM] Freq=%d, ch=%d\n", f, radio->registers[READCHAN]&READCHAN_READCHAN);
	return 0;
}

static int fm_reg_debug_set(void *data, u64 val)
{
	si470x_set_region(val);
	return 0;
}

static int fm_reg_debug_get(void *data, u64 *val)
{
	fm_debug_show_reg();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fm_vol_fops, fm_vol_debug_get, fm_vol_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_freq_fops, fm_freq_debug_get, fm_freq_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_seek_fops, fm_seek_debug_get, fm_seek_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_power_fops, fm_power_debug_get, fm_power_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_rds_fops, fm_rds_debug_get, fm_rds_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_ch_fops, fm_ch_debug_get, fm_ch_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_reg_fops, fm_reg_debug_get, fm_reg_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_rdsrt_fops, fm_rdsrt_debug_get, fm_rdsrt_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fm_stereo_fops, fm_stereo_debug_get, fm_stereo_debug_set, "%llu\n");

static int __init fm_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("fm", 0);
	if (IS_ERR(dent))
		return 0;
	(void) debugfs_create_file("vol", 0644,
			dent, 0, &fm_vol_fops);
	(void) debugfs_create_file("freq", 0644,
			dent, 0, &fm_freq_fops);
	(void) debugfs_create_file("seek", 0644,
			dent, 0, &fm_seek_fops);
	(void) debugfs_create_file("power", 0644,
			dent, 0, &fm_power_fops);
	(void) debugfs_create_file("rds", 0644,
			dent, 0, &fm_rds_fops);
	(void) debugfs_create_file("ch", 0644,
			dent, 0, &fm_ch_fops);
	(void) debugfs_create_file("reg", 0644,
			dent, 0, &fm_reg_fops);
	(void) debugfs_create_file("rt", 0644,
			dent, 0, &fm_rdsrt_fops);
	(void) debugfs_create_file("stereo", 0644,
			dent, 0, &fm_stereo_fops);
	return 0;
}

device_initcall(fm_debug_init);
#endif


