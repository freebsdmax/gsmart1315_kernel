/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include "msm_fb.h"

#define CS(a)		gpio_set_value(spi_cs, a)
#define SCLK(a)		gpio_set_value(spi_clk, a)
#define SDI(a)		gpio_set_value(spi_sdo, a)
#define SDO			gpio_get_value(spi_sdi)
#define RESET(a)	gpio_set_value(lcd_reset, a)

#define DEVICE_ID	0x80E83402
#define BIT_CMD  0
#define BIT_DATA 1

#define WRITEBIT(bit) \
do{ \
	SCLK(0);	\
	SDI(bit);	\
	udelay(1);	\
	SCLK(1);	\
	udelay(1);	\
} while(0)

static int spi_cs;
static int spi_clk;
static int spi_sdi;
static int spi_sdo;
static int lcd_reset;

struct driver_state_type{
	boolean disp_initialized;
	boolean display_on;
	int backlight_level;
};

static struct driver_state_type driver_state = { 0 };

/* platfrom device data */
static struct msm_panel_common_pdata *platform_data;

static void rm68042_hw_reset(void);
static void rm68042_set_backlight(struct msm_fb_data_type *mfd);

#ifdef CONFIG_DEBUG_FS
static int __init rm68042_debug_init(void);
#endif

static void LCDSPI_InitCMD(unsigned short command)
{
	unsigned short i;

	CS(0);
	udelay(1);
	SCLK(1);
	SDI(1);
	udelay(1);
	/* Write Command */
	SCLK(0);
	SDI(0);
	udelay(1);
	SCLK(1);
	udelay(1);
	for (i = 0; i < 8; ++i)	{
		SCLK(0);
		if (command&0x0080)
			SDI(1);
		else
			SDI(0);
		udelay(1);
		SCLK(1);
		udelay(1);
		command <<= 1;
	}
	CS(1);
}

static void LCDSPI_InitDAT(unsigned short data)
{
	unsigned short i;

	CS(0);
	udelay(1);
	SCLK(1);
	SDI(1);
	udelay(1);
	/* Write Data */
	SCLK(0);
	SDI(1);
	udelay(1);
	SCLK(1);
	udelay(1);
	for (i = 0; i < 8; ++i)	{
		SCLK(0);
		if (data&0x0080)
			SDI(1);
		else
			SDI(0);
		udelay(1);
		SCLK(1);
		udelay(1);
		data <<= 1;
	}
	CS(1);
}

static unsigned readbyte(void)
{
	unsigned byte=0;
	unsigned i;

	for(i = 0; i < 8; ++i) {
		byte <<= 1;
		SCLK(0);
		udelay(1);
		byte |= SDO;
		SCLK(1);
		udelay(1);
	}
	return byte;
}

static void writebyte(unsigned byte)
{
	unsigned n;
	for(n = 0; n < 8; ++n)
	{
		if(byte & 0x80)
			WRITEBIT(1);
		else
			WRITEBIT(0);
		byte <<= 1;
	}
}

static unsigned readID(void)
{
	unsigned id;
	unsigned n;

	CS(0);
	udelay(1);
	WRITEBIT(BIT_CMD);
	writebyte(0xBF);
	for(n=0, id=0; n<4; ++n)
		id = (id << 8) + readbyte();
	
	CS(1);

	return id;
}

static void Lcd_init(void)
{
	//**********************Start initial squence******************
	LCDSPI_InitCMD(0x11);//dstb
	mdelay(20);
	LCDSPI_InitCMD(0XD0);//Power_Setting (D0h)
	LCDSPI_InitDAT(0X02);
	LCDSPI_InitDAT(0X41);
	LCDSPI_InitDAT(0X1E);

	LCDSPI_InitCMD(0XD1);//VCOM Control (D1h)
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X08);
	LCDSPI_InitDAT(0X0D);

	LCDSPI_InitCMD(0XD2); /* Power_Setting for Normal Mode */
	LCDSPI_InitDAT(0X01);
	LCDSPI_InitDAT(0X11);

	LCDSPI_InitCMD(0XC0); /* Panel Driving Setting */
	LCDSPI_InitDAT(0X10);
	LCDSPI_InitDAT(0X3B);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X12);
	LCDSPI_InitDAT(0X01);

	LCDSPI_InitCMD(0XC1);
	LCDSPI_InitDAT(0X10);
	LCDSPI_InitDAT(0X13);
	LCDSPI_InitDAT(0X88);

	LCDSPI_InitCMD(0XC5);
	LCDSPI_InitDAT(0X02);

	LCDSPI_InitCMD(0XB3); /* Frame Memory Access and Interface Setting */
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X20);

	LCDSPI_InitCMD(0XC6); /* polarity */
	LCDSPI_InitDAT(0X1B);

	LCDSPI_InitCMD(0XC8);
	LCDSPI_InitDAT(0X02);
	LCDSPI_InitDAT(0X46);
	LCDSPI_InitDAT(0X14);
	LCDSPI_InitDAT(0X31);
	LCDSPI_InitDAT(0X0A);
	LCDSPI_InitDAT(0X04);
	LCDSPI_InitDAT(0X37);
	LCDSPI_InitDAT(0X24);
	LCDSPI_InitDAT(0X57);
	LCDSPI_InitDAT(0X13);
	LCDSPI_InitDAT(0X06);
	LCDSPI_InitDAT(0X0C);

	LCDSPI_InitCMD(0XF3);
	LCDSPI_InitDAT(0X40);
	LCDSPI_InitDAT(0X0A);

	LCDSPI_InitCMD(0XF6);
	LCDSPI_InitDAT(0X80);

	LCDSPI_InitCMD(0XF7);
	LCDSPI_InitDAT(0X80);

	LCDSPI_InitCMD(0X36);
	LCDSPI_InitDAT(0X0A);

	LCDSPI_InitCMD(0X3A);
	LCDSPI_InitDAT(0X66);

	LCDSPI_InitCMD(0XB4);
	LCDSPI_InitDAT(0X11);

	LCDSPI_InitCMD(0X2A);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X01);
	LCDSPI_InitDAT(0X3F);

	LCDSPI_InitCMD(0X2B);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X00);
	LCDSPI_InitDAT(0X01);
	LCDSPI_InitDAT(0XDF);
	mdelay(120);
	LCDSPI_InitCMD(0X29);   
	LCDSPI_InitCMD(0x2C);
	mdelay(10);
}

#ifdef CONFIG_DEBUG_FS

typedef struct rm68042_reg_cmd_struct {
        unsigned char val;
        unsigned char reg;
        unsigned short reserved;
/* 0xYYZZ, YY command and ZZ argument */
/* 0x00ZZ, more argument */
/* 0xFF00, terminate */
/* 0xFFZZ, delay ZZ ms */
} rm68042_reg_cmd;

static unsigned long rm68042_init_table[128] = {
	0x1100, // dstb
	0xFF14, // delay 20ms
	0xD002, 0x41, 0x1E, //Power_Setting (D0h)
	0xD100, 0x08, 0x0D, //VCOM Control (D1h)
	0xD201,	0x11, /* Power_Setting for Normal Mode */
	0xC010, 0x3B, 0x00, 0x12, 0x01, /* Panel Driving Setting */
	0xC110, 0x13, 0x88,
	0xC502,
	0xB300, 0x00, 0x00, 0x20, /* Frame Memory Access and Interface Setting */
	0xC61B, /* polarity */
	0xC802, 0x46, 0x14, 0x31, 0x0A, 0x04, 0x37, 0x24, 0x57, 0x13, 0x06, 0x0C,
	0xF340, 0x0A,
	0xF680,
	0xF780,
	0x360A,
	0x3A66,
	0xB411,
	0x2A00, 0x00, 0x01, 0x3F,
	0x2B00, 0x00, 0x01, 0xDF,
	0xFF78,	/* delay 120 ms */
	0x2900,
	0x2C00,
	0xFF0A, /* delay 10 ms */
	0xFF00, /* terminate */
};

static void do_init_table(rm68042_reg_cmd *tbl)
{
	CS(0);
	udelay(1);
	do {
		if(tbl->reg == 0xFF)
			if(tbl->val)
				mdelay(tbl->val);
			else /* 0xFF00 */
				break;
		else {
			if(tbl->reg) {
				WRITEBIT(BIT_CMD);
				writebyte(tbl->reg);
			}
			WRITEBIT(BIT_DATA);
			writebyte(tbl->val);
		}
		++tbl;
	} while(1);
	CS(1);
}

#endif

static void lcdc_rm68042_config_gpios(void)
{
	/* Setting the Default GPIO's */
	spi_sdi = platform_data->gpio_num[0];
	spi_sdo = platform_data->gpio_num[1];
	lcd_reset = platform_data->gpio_num[2];
	spi_cs  = platform_data->gpio_num[3];
	spi_clk = platform_data->gpio_num[4];

	platform_data->panel_config_gpio(GPIO_ENABLE);

	RESET(1);
	SCLK(1);
	CS(1);

}

static void rm68042_hw_reset(void)
{
	RESET(0);
	msleep(1);
	RESET(1);
	msleep(120);
}

static int rm68042_on(struct platform_device *pdev)
{
	if (!driver_state.disp_initialized) {
		/* Power Supply -> reset -> initail */
		lcdc_rm68042_config_gpios();
		rm68042_hw_reset();
#ifdef CONFIG_DEBUG_FS
		do_init_table((rm68042_reg_cmd *)rm68042_init_table);
#else
		Lcd_init();
#endif
		driver_state.disp_initialized = TRUE;
	}

	driver_state.display_on = TRUE;

	return 0;
}

static int rm68042_off(struct platform_device *pdev)
{
	if (driver_state.display_on) {
		driver_state.display_on = FALSE;
                /* we may cut off power, so that we have to initial every time */
                driver_state.disp_initialized = FALSE;
                /* turn off backlight, that we won't see white screen */
                platform_data->pmic_backlight(0);
	}
	return 0;
}

static void rm68042_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;

        if( driver_state.display_on )
                if(0 == bl_level)
                        printk("[LCDC] bkl off with disp on");
        else if( bl_level && 0 == driver_state.backlight_level)
                printk("[LCDC] bkl on with disp off");

	if (driver_state.disp_initialized)
		driver_state.backlight_level =
			platform_data->pmic_backlight(bl_level);
}


static struct msm_fb_panel_data panel_data = {
        .on = rm68042_on,
        .off = rm68042_off,
        .set_backlight = rm68042_set_backlight,
};

static struct platform_device this_device = {
        .name   = "lcdc_device",
        .id     = -1,
        .dev    = {
                .platform_data = &panel_data,
        }
};

static int __init lcdc_rm68042_probe(struct platform_device *pdev)
{
	int ret = 0;

	if(platform_data) return -ENODEV;

	platform_data = pdev->dev.platform_data;
	lcdc_rm68042_config_gpios();
#ifndef CONFIG_LCD_PRE_RESET_IN_FASTBOOT
	rm68042_hw_reset();
#endif
	if(DEVICE_ID == readID()) {
		printk("[LCDC] rm68042 probed on id %d\n", pdev->id);
#ifdef CONFIG_DEBUG_FS
		rm68042_debug_init();
#endif
	} else {
		platform_data = NULL;
		ret = -ENODEV;
	}

	return ret;
}

static struct platform_driver this_driver = {
        .probe  = lcdc_rm68042_probe,
        .driver = {
                .name   = "lcdc_device",
        },
};

static int __init lcdc_rm68042_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &panel_data.panel_info;
	pinfo->xres = 320;
	pinfo->yres = 480;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 8192000;
	pinfo->bl_max = 10;
	pinfo->bl_min = 0;

	pinfo->lcdc.h_back_porch = 8;
	pinfo->lcdc.h_front_porch = 8;
	pinfo->lcdc.h_pulse_width = 4;
	pinfo->lcdc.v_back_porch = 4;
	pinfo->lcdc.v_front_porch = 4;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

	if (platform_data)
		msm_fb_add_device(&this_device);
	else
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_rm68042_init);

#ifdef CONFIG_DEBUG_FS

#include "linux/debugfs.h"

static ssize_t default_read_file(struct file *file, char __user *user_buf,
                                 size_t count, loff_t *ppos)
{
	unsigned int *iter = (void *)&rm68042_init_table[0];
	do {
		printk("0x%4X\n", *iter);
	} while(0xFF00 != *iter++);
	return 0;
}

static ssize_t default_write_file(struct file *file, const char __user *buf,
                                   size_t count, loff_t *ppos)
{
	char * ptr = (void *)buf;
	unsigned long pair;
	rm68042_reg_cmd *target=(void *)&rm68042_init_table[0];

	if(!ptr) return 0;
	do {
		pair = simple_strtoul(ptr, &ptr, 16);
		*(unsigned long*)target = pair;
		++target;
		ptr = strchr(ptr, ',');
	} while(ptr++);

	return count;
}

static const struct file_operations rm68042_init_tbl_fops = {
	.read =  default_read_file,
	.write = default_write_file,
};

static int rm68042_hw_reset_set_value(void *data, u64 val)
{
	lcdc_rm68042_config_gpios();
	rm68042_hw_reset();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rm68042_hw_reset_fops, NULL,
			rm68042_hw_reset_set_value, "%llu\n");

static int rm68042_bl_set_value(void *data, u64 val)
{
	driver_state.backlight_level =
		platform_data->pmic_backlight((int)val);
	return 0;
}
static int rm68042_bl_get_value(void *data, u64 *val)
{
	*val = driver_state.backlight_level;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rm68042_bl_fops, rm68042_bl_get_value,
			rm68042_bl_set_value, "%llu\n");

static int rm68042_do_init_set_value(void *data, u64 val)
{
	do_init_table((rm68042_reg_cmd *)rm68042_init_table);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rm68042_do_init_fops, NULL,
			rm68042_do_init_set_value, "%llu\n");

static int __init rm68042_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("rm68042", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("init_tbl", 0644, dent, 0, &rm68042_init_tbl_fops);
	debugfs_create_file("do_init", 0644, dent, 0, &rm68042_do_init_fops);
	debugfs_create_file("bl", 0644, dent, 0, &rm68042_bl_fops);
	debugfs_create_file("hw_reset", 0666, dent, 0, &rm68042_hw_reset_fops);

	return 0;
}

#endif /* CONFIG_DEBUG_FS */
