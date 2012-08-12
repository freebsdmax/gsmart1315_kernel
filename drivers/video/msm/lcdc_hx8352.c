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
#define SDO		gpio_get_value(spi_sdi)
#define RESET(a)	gpio_set_value(lcd_reset, a)
#define gdelay(a)	udelay(a)

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

static void hx8352_hw_reset(void);
static void hx8352_set_backlight(struct msm_fb_data_type *mfd);

#ifdef CONFIG_DEBUG_FS
static void hx8352_do_init_tbl(void);
static int __init hx8352_debugfs_init(void);
#endif

#define mDevID	(0x74)
#define RS(a)	(a << 1)
#define RW(a)	(a << 0)

static void writebyte(unsigned byte)
{
	unsigned short i;
	for (i = 0; i < 8; ++i)	{
		SCLK(0);
		gdelay(1);
		if (byte&0x0080)
			SDI(1);
		else
			SDI(0);
		SCLK(1);
		gdelay(1);
		byte <<= 1;
	}
}

static void WriteCommand(unsigned cmd)
{
	unsigned fb=mDevID+RS(0)+RW(0);
	SCLK(1);
	SDI(1);
	CS(0);
	gdelay(1);
	writebyte(fb);
	writebyte(cmd);
	CS(1);
}

static void WriteData(unsigned cmd)
{
	unsigned fb=mDevID+RS(1)+RW(0);
	SCLK(1);
	SDI(1);
	CS(0);
	gdelay(1);
	writebyte(fb);
	writebyte(cmd);
	CS(1);	
}

static unsigned readbyte(void)
{
	unsigned byte=0;
	unsigned i;
	for(i = 0; i < 8; ++i) {
		byte <<= 1;
		SCLK(0);
		gdelay(1);
		byte |= SDO;
		SCLK(1);
		gdelay(1);
	}
	return byte;
}

static unsigned readreg(unsigned reg)
{
	unsigned byte=mDevID+RS(1)+RW(1);
	WriteCommand(reg);
	mdelay(10);
	CS(0);
	writebyte(byte);
	byte = readbyte(); //dummy 
	byte = readbyte();
	CS(1);
	return byte;
}

static void Lcd_init(void) 
{
	WriteCommand(0x83);WriteData(0x02);/* TEST_MODE on */
	WriteCommand(0x85);WriteData(0x03);/* VDDD control */
	WriteCommand(0x8B);WriteData(0x00);/* source gamma resistor */
	WriteCommand(0x8C);WriteData(0x13);/* source gamma resistor */
	WriteCommand(0x91);WriteData(0x01);/* sync */
	WriteCommand(0x83);WriteData(0x00);/* TEST_MODE off */
	mdelay(5);
	 
	/* Gamma Setting */
	WriteCommand(0x3E);WriteData(0xC4);
	WriteCommand(0x3F);WriteData(0x44);
	WriteCommand(0x40);WriteData(0x22);
	WriteCommand(0x41);WriteData(0x57);
	WriteCommand(0x42);WriteData(0x03);
	WriteCommand(0x43);WriteData(0x47);
	WriteCommand(0x44);WriteData(0x02);
	WriteCommand(0x45);WriteData(0x55);
	WriteCommand(0x46);WriteData(0x06);
	WriteCommand(0x47);WriteData(0x4C);
	WriteCommand(0x48);WriteData(0x06);
	WriteCommand(0x49);WriteData(0x8C);

	/* Power Supply Setting */
	WriteCommand(0x2B);WriteData(0xF9);
	mdelay(20);
	WriteCommand(0x17);WriteData(0x90);
	WriteCommand(0x18);WriteData(0x3A);
	WriteCommand(0x1B);WriteData(0x14);/* BT(VCL,VGH,VGL) */
	WriteCommand(0x1A);WriteData(0x11);/* VC1 VC3 */
	WriteCommand(0x1C);WriteData(0x0A);/* VRH */
	WriteCommand(0x1F);WriteData(0x58);/* VCM */
	mdelay(30);
	WriteCommand(0x19);WriteData(0x0A);/* power */
	WriteCommand(0x19);WriteData(0x1A);
	mdelay(50);
	WriteCommand(0x19);WriteData(0x12);
	mdelay(50);
	WriteCommand(0x1E);WriteData(0x2E);/* VCOMG */
	mdelay(100);
	WriteCommand(0x5A);WriteData(0x01); /* DUG FUNCTION ON */
	WriteCommand(0x5C); /* LUT */
		WriteData(0x00); /* RED LUT */
		WriteData(0x03);
		WriteData(0x0A);
		WriteData(0x0F);
		WriteData(0x13);
		WriteData(0x16);
		WriteData(0x19);
		WriteData(0x1C);
		WriteData(0x1E);
		WriteData(0x1F);
		WriteData(0x25);
		WriteData(0x2A);
		WriteData(0x30);
		WriteData(0x35);
		WriteData(0x39);
		WriteData(0x3D);
		WriteData(0x41);
		WriteData(0x45);
		WriteData(0x48);
		WriteData(0x4C);
		WriteData(0x4F);
		WriteData(0x53);
		WriteData(0x58);
		WriteData(0x5D);
		WriteData(0x61);
		WriteData(0x66);
		WriteData(0x6A);
		WriteData(0x6E);
		WriteData(0x72);
		WriteData(0x76);
		WriteData(0x7A);
		WriteData(0x7E);
		WriteData(0x82);
		WriteData(0x85);
		WriteData(0x89);
		WriteData(0x8D);
		WriteData(0x90);
		WriteData(0x94);
		WriteData(0x96);
		WriteData(0x9A);
		WriteData(0x9D);
		WriteData(0xA1);
		WriteData(0xA4);
		WriteData(0xA8);
		WriteData(0xAB);
		WriteData(0xAF);
		WriteData(0xB3);
		WriteData(0xB7);
		WriteData(0xBB);
		WriteData(0xBF);
		WriteData(0xC3);
		WriteData(0xC8);
		WriteData(0xCC);
		WriteData(0xD1);
		WriteData(0xD6);
		WriteData(0xDB);
		WriteData(0xDE);
		WriteData(0xE1);
		WriteData(0xE5);
		WriteData(0xE7);
		WriteData(0xEC);
		WriteData(0xEF);
		WriteData(0xF4);
		WriteData(0xFF);
		WriteData(0x00);/* GLUT */
		WriteData(0x03);
		WriteData(0x0A);
		WriteData(0x0F);
		WriteData(0x13);
		WriteData(0x16);
		WriteData(0x19);
		WriteData(0x1C);
		WriteData(0x1E);
		WriteData(0x1F);
		WriteData(0x25);
		WriteData(0x2A);
		WriteData(0x30);
		WriteData(0x35);
		WriteData(0x39);
		WriteData(0x3D);
		WriteData(0x41);
		WriteData(0x45);
		WriteData(0x48);
		WriteData(0x4C);
		WriteData(0x4F);
		WriteData(0x53);
		WriteData(0x58);
		WriteData(0x5D);
		WriteData(0x61);
		WriteData(0x66);
		WriteData(0x6A);
		WriteData(0x6E);
		WriteData(0x72);
		WriteData(0x76);
		WriteData(0x7A);
		WriteData(0x7E);
		WriteData(0x82);
		WriteData(0x85);
		WriteData(0x89);
		WriteData(0x8D);
		WriteData(0x90);
		WriteData(0x94);
		WriteData(0x96);
		WriteData(0x9A);
		WriteData(0x9D);
		WriteData(0xA1);
		WriteData(0xA4);
		WriteData(0xA8);
		WriteData(0xAB);
		WriteData(0xAF);
		WriteData(0xB3);
		WriteData(0xB7);
		WriteData(0xBB);
		WriteData(0xBF);
		WriteData(0xC3);
		WriteData(0xC8);
		WriteData(0xCC);
		WriteData(0xD1);
		WriteData(0xD6);
		WriteData(0xDB);
		WriteData(0xDE);
		WriteData(0xE1);
		WriteData(0xE5);
		WriteData(0xE7);
		WriteData(0xEC);
		WriteData(0xEF);
		WriteData(0xF4);
		WriteData(0xFF);
		WriteData(0x00); /* B LUT */
		WriteData(0x03);
		WriteData(0x0A);
		WriteData(0x0F);
		WriteData(0x13);
		WriteData(0x16);
		WriteData(0x19);
		WriteData(0x1C);
		WriteData(0x1E);
		WriteData(0x1F);
		WriteData(0x25);
		WriteData(0x2A);
		WriteData(0x30);
		WriteData(0x35);
		WriteData(0x39);
		WriteData(0x3D);
		WriteData(0x41);
		WriteData(0x45);
		WriteData(0x48);
		WriteData(0x4C);
		WriteData(0x4F);
		WriteData(0x53);
		WriteData(0x58);
		WriteData(0x5D);
		WriteData(0x61);
		WriteData(0x66);
		WriteData(0x6A);
		WriteData(0x6E);
		WriteData(0x72);
		WriteData(0x76);
		WriteData(0x7A);
		WriteData(0x7E);
		WriteData(0x82);
		WriteData(0x85);
		WriteData(0x89);
		WriteData(0x8D);
		WriteData(0x90);
		WriteData(0x94);
		WriteData(0x96);
		WriteData(0x9A);
		WriteData(0x9D);
		WriteData(0xA1);
		WriteData(0xA4);
		WriteData(0xA8);
		WriteData(0xAB);
		WriteData(0xAF);
		WriteData(0xB3);
		WriteData(0xB7);
		WriteData(0xBB);
		WriteData(0xBF);
		WriteData(0xC3);
		WriteData(0xC8);
		WriteData(0xCC);
		WriteData(0xD1);
		WriteData(0xD6);
		WriteData(0xDB);
		WriteData(0xDE);
		WriteData(0xE1);
		WriteData(0xE5);
		WriteData(0xE7);
		WriteData(0xEC);
		WriteData(0xEF);
		WriteData(0xF4);
		WriteData(0xFF);
	/* Display ON Setting */
	WriteCommand(0x3C);WriteData(0xC0);
	WriteCommand(0x3D);WriteData(0x1C);
	WriteCommand(0x34);WriteData(0x38);
	WriteCommand(0x35);WriteData(0x38);
	WriteCommand(0x24);WriteData(0x38);
	mdelay(50);
	WriteCommand(0x24);WriteData(0x3C);
	WriteCommand(0x16);WriteData(0x1C);
	WriteCommand(0x3A);WriteData(0xCE);/* polarity */
	WriteCommand(0x01);WriteData(0x06);/* pixel invert */
	WriteCommand(0x55);WriteData(0x00);
}
static void lcdc_hx8352_config_gpios(void)
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

static void hx8352_hw_reset(void)
{
	RESET(0);
	msleep(1);
	RESET(1);
	msleep(120);
}

static int hx8352_on(struct platform_device *pdev)
{
	if (!driver_state.disp_initialized) {
		/* Power Supply -> reset -> initail */
		lcdc_hx8352_config_gpios();
		hx8352_hw_reset();
#ifdef CONFIG_DEBUG_FS
		hx8352_do_init_tbl();
#else
		Lcd_init();
#endif
		if(0x52 == readreg(0x0))
			driver_state.disp_initialized = TRUE;
	}

	driver_state.display_on = TRUE;

	return 0;
}

static int hx8352_off(struct platform_device *pdev)
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

static void hx8352_set_backlight(struct msm_fb_data_type *mfd)
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
        .on = hx8352_on,
        .off = hx8352_off,
        .set_backlight = hx8352_set_backlight,
};

static void hx8352_release(struct device *dev)
{
	(void) dev;
	return;
}

static struct platform_device this_device = {
        .name   = "lcdc_device",
        .id     = -1,
        .dev    = {
                .platform_data = &panel_data,
		.release = hx8352_release,
        }
};

static int __init lcdc_hx8352_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("[LCDC] lcdc_hx8352_probe, id=%d\n", pdev->id);
	if(platform_data)
		return -ENODEV;

	platform_data = pdev->dev.platform_data;
	lcdc_hx8352_config_gpios();
#ifndef CONFIG_LCD_PRE_RESET_IN_FASTBOOT
	hx8352_hw_reset();
#endif
	if(0x52 != readreg(0x0)) {
		platform_data = NULL; 
		ret = -ENODEV;
	} else {
		printk("[LCDC] hx8352 probed on id %d\n", pdev->id);
#ifdef CONFIG_DEBUG_FS
		hx8352_debugfs_init();
#endif
		ret = 0;
	}
	return ret;
}

static struct platform_driver this_driver = {
	.probe  = lcdc_hx8352_probe,
	.driver = {
		.name   = "lcdc_device",
	},
};

static int __init lcdc_hx8352_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &panel_data.panel_info;
	pinfo->xres = 240;
	pinfo->yres = 400;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 8192000;
	pinfo->bl_max = 10;
	pinfo->bl_min = 0;

	pinfo->lcdc.h_back_porch = 8;
	pinfo->lcdc.h_front_porch = 16;
	pinfo->lcdc.h_pulse_width = 8;
	pinfo->lcdc.v_back_porch = 4;
	pinfo->lcdc.v_front_porch = 8;
	pinfo->lcdc.v_pulse_width = 4;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

        if (platform_data)
                msm_fb_add_device(&this_device);
        else
                platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_hx8352_init);

#ifdef CONFIG_DEBUG_FS

#include "linux/debugfs.h"

typedef struct hx8352_reg_cmd_struct {
	unsigned char val;
	unsigned char reg;
	unsigned short reserved;
/* 0xYYZZ, YY command and ZZ argument */
/* 0x00ZZ, more argument */
/* 0xFF00, terminate */
/* 0xFFZZ, delay ZZ ms */
} hx8352_reg_cmd;

static unsigned long hx8352_init_table[256] = {
	0x8302,/* TEST_MODE on */
	0x8503,/* VDDD control */
	0x8B00,/* source gamma resistor */
	0x8C13,/* source gamma resistor */
	0x9101,/* sync */
	0x8300,/* TEST_MODE off */
	0xFF05,/* delay 5 ms */
	/* Gamma Setting */
	0x3EC4, 0x3F44, 0x4022, 0x4157, 0x4203, 0x4347, 0x4402, 0x4555,
	0x4606, 0x474C, 0x4806, 0x498C,
	/* Power Supply Setting */
	0x2BF9,
	0xFF14,/* delay 20 ms */
	0x1790,
	0x183A,
	0x1B14,/* BT(VCL,VGH,VGL) */
	0x1A11,/* VC1 VC3 */
	0x1C0A,/* VRH */
	0x1F58,/* VCM */
	0xFF03,/* delay 3 ms */
	0x190A,/* power */
	0x191A,
	0xFF05,/* delay 5 ms */
	0x1912,
	0xFF05,/* delay 5 ms */
	0x1E2E,/* VCOMG */
	0xFF0a,/* delay 10 ms */
	0x5A01, /* DUG FUNCTION ON */
	/* RED LUT */
	0x5C00, 0x03, 0x0A, 0x0F, 0x13, 0x16, 0x19, 0x1C, 0x1E, 0x1F, 0x25, 0x2A,
	0x30, 0x35, 0x39, 0x3D, 0x41, 0x45, 0x48, 0x4C, 0x4F, 0x53, 0x58, 0x5D,
	0x61, 0x66, 0x6A, 0x6E, 0x72, 0x76, 0x7A, 0x7E, 0x82, 0x85, 0x89, 0x8D,
	0x90, 0x94, 0x96, 0x9A, 0x9D, 0xA1, 0xA4, 0xA8, 0xAB, 0xAF, 0xB3, 0xB7,
	0xBB, 0xBF, 0xC3, 0xC8, 0xCC, 0xD1, 0xD6, 0xDB, 0xDE, 0xE1, 0xE5, 0xE7,
	0xEC, 0xEF, 0xF4, 0xFF, 
	/* GLUT */
	0x00, 0x03, 0x0A, 0x0F, 0x13, 0x16, 0x19, 0x1C, 0x1E, 0x1F, 0x25, 0x2A, 
	0x30, 0x35, 0x39, 0x3D, 0x41, 0x45, 0x48, 0x4C, 0x4F, 0x53, 0x58, 0x5D,
	0x61, 0x66, 0x6A, 0x6E, 0x72, 0x76, 0x7A, 0x7E, 0x82, 0x85, 0x89, 0x8D,
	0x90, 0x94, 0x96, 0x9A, 0x9D, 0xA1, 0xA4, 0xA8, 0xAB, 0xAF, 0xB3, 0xB7,
	0xBB, 0xBF, 0xC3, 0xC8, 0xCC, 0xD1, 0xD6, 0xDB, 0xDE, 0xE1, 0xE5, 0xE7,
	0xEC, 0xEF, 0xF4, 0xFF,
	/* B LUT */
	0x00, 0x03, 0x0A, 0x0F, 0x13, 0x16, 0x19, 0x1C, 0x1E, 0x1F, 0x25, 0x2A,
	0x30, 0x35, 0x39, 0x3D, 0x41, 0x45, 0x48, 0x4C, 0x4F, 0x53, 0x58, 0x5D,
	0x61, 0x66, 0x6A, 0x6E, 0x72, 0x76, 0x7A, 0x7E, 0x82, 0x85, 0x89, 0x8D,
	0x90, 0x94, 0x96, 0x9A, 0x9D, 0xA1, 0xA4, 0xA8, 0xAB, 0xAF, 0xB3, 0xB7,
	0xBB, 0xBF, 0xC3, 0xC8, 0xCC, 0xD1, 0xD6, 0xDB, 0xDE, 0xE1, 0xE5, 0xE7,
	0xEC, 0xEF, 0xF4, 0xFF,
	/* Display ON Setting */
	0x3CC0, 0x3D1C, 0x3438, 0x3538, 0x2438, 0xFF05, 0x243C, 0x161C,
	0x3ACE,/* polarity */
	0x0106,/* pixel invert */
	0x5500,
	0xFF00, /* end up here */
};

static ssize_t default_read_file(struct file *file, char __user *user_buf,
                                 size_t count, loff_t *ppos)
{
	unsigned int *iter = (void *)&hx8352_init_table[0];
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
	hx8352_reg_cmd *target=(void *)&hx8352_init_table[0];

	if(!ptr) return 0;
	do {
		pair = simple_strtoul(ptr, &ptr, 16);
		*(unsigned long*)target = pair;
		++target;
		ptr = strchr(ptr, ',');
	} while(ptr++);

	return count;
}

static const struct file_operations hx8352_init_tbl_fops = {
	.read =  default_read_file,
	.write = default_write_file,
};

static void hx8352_do_init_tbl(void)
{
	unsigned long *iter = &hx8352_init_table[0];
	hx8352_reg_cmd *cmd;
	do {
		cmd = (void*)iter;
		if(0xFF == cmd->reg) { /* 0xFF means delay */
			if(cmd->val)
				mdelay(cmd->val);
		} else if(0x0 == cmd->reg) { /* pure data */
			WriteData(cmd->val);
		} else {
			WriteCommand(cmd->reg);
			WriteData(cmd->val);
		}
	} while(0xFF00 != *iter++);

	return;
}

static int hx8352_do_init_set_value(void *data, u64 val)
{
	hx8352_do_init_tbl();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(hx8352_do_init_fops, NULL,
			hx8352_do_init_set_value, "%llu\n");


static int hx8352_bl_set_value(void *data, u64 val)
{
	driver_state.backlight_level =
		platform_data->pmic_backlight((int)val);
	return 0;
}
static int hx8352_bl_get_value(void *data, u64 *val)
{
	*val = driver_state.backlight_level;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(hx8352_bl_fops, hx8352_bl_get_value,
			hx8352_bl_set_value, "%llu\n");

static int hx8352_hw_reset_set_value(void *data, u64 val)
{
	hx8352_hw_reset();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(hx8352_hw_reset_fops, NULL,
			hx8352_hw_reset_set_value, "%llu\n");


static u32 regval;
static int hx8352_reg_set_value(void *data, u64 val)
{
	regval = (unsigned)val & 0xFF;
	WriteCommand( regval & 0xFF) ;
	return 0;
}
static int hx8352_reg_get_value(void *data, u64 *val)
{
	*val = (u64)regval;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(hx8352_reg_fops, hx8352_reg_get_value,
			hx8352_reg_set_value, "%llu\n");


static int hx8352_val_set_value(void *data, u64 val)
{
	WriteData((unsigned)(val & 0xFF));
	return 0;
}
static int hx8352_val_get_value(void *data, u64 *val)
{
	unsigned byte=mDevID+RS(1)+RW(1);
	CS(0);
	writebyte(byte);
	byte = readbyte(); //dummy 
	byte = readbyte();
	CS(1);	
	*val = byte;
	printk("[LCDC] reg 0x%X=0x%x\n", regval, (unsigned)*val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(hx8352_val_fops, hx8352_val_get_value,
			hx8352_val_set_value, "%llu\n");

static int __init hx8352_debugfs_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("hx8352", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("init_tbl", 0644, dent, 0, &hx8352_init_tbl_fops);
	debugfs_create_file("do_init", 0644, dent, 0, &hx8352_do_init_fops);
	debugfs_create_file("bl", 0644, dent, 0, &hx8352_bl_fops);
	debugfs_create_file("hw_reset", 0666, dent, 0, &hx8352_hw_reset_fops);
	debugfs_create_file("reg", 0666, dent, 0, &hx8352_reg_fops);
	debugfs_create_file("val", 0666, dent, 0, &hx8352_val_fops);

	return 0;
}

#endif /* CONFIG_DEBUG_FS */
