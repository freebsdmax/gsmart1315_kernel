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

static int spi_cs;
static int spi_clk;
static int spi_sdi;
static int spi_sdo;
static int lcd_reset;

struct driver_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean is_sleep;
	int backlight_level;
};

static struct driver_state_type driver_state = { 0 };

/* platfrom device data */
static struct msm_panel_common_pdata *platform_data = NULL;

static void ili9481b_hw_reset(void);
static void ili9481b_set_backlight(struct msm_fb_data_type *mfd);
static void ili9481b_sleep(int);

static void MAINLCD_CMD(unsigned short command)
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

static void MAINLCD_DATA(unsigned short data)
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


static void writecmd(unsigned cmd)
{
	unsigned mask=0x100;
	unsigned n;

	for(n=9; n; --n)
	{
		SCLK(0);
		if(cmd & mask)
			SDI(1);
		else
			SDI(0);
		udelay(1);
		SCLK(1);
		udelay(1);
		cmd <<= 1;
	}
}


static unsigned readID(void)
{
	unsigned byte[6];
	unsigned n;
	unsigned id;
	CS(0);
	udelay(1);
	writecmd(0xBF);
	for(n=0; n<6; ++n)
		byte[n] = readbyte();
	CS(1);

	id = (byte[0]<<24)+(byte[1]<<16)+(byte[2]<<8)+byte[3];
		

	printk("[LCDC] read ili9481b DeviceID=0x%2X%2X%2X%2X%2X%2X\n",
		byte[0], byte[1], byte[2], byte[3], byte[4], byte[5]);
	
	return id;
}

#define Delay	mdelay
void RGB_initial()
{
	MAINLCD_CMD(0x0001); /* soft reset */
	Delay(20);
	MAINLCD_CMD(0x0011);
	MAINLCD_CMD(0x0036);
	MAINLCD_DATA(0x000A);
	MAINLCD_CMD(0x003a);
	MAINLCD_DATA(0x0066);
	MAINLCD_CMD(0x00d0); /* power vci1 */
	MAINLCD_DATA(0x0007); /* vci1=1xVci=2.72 */
	MAINLCD_DATA(0x0042); /* DDVDH=vci1x2,VCL=-vci1,VGH=vci1x6,VGL=-vci1x3  42 */
	MAINLCD_DATA(0x001b); /* VREG1OUT=2.5x1.7=4.25V */
	MAINLCD_CMD(0x00d1);  /* vcomh / vcomac  */
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0024);
	MAINLCD_DATA(0x0012);
	MAINLCD_CMD(0x00d2); /* power freq */
	MAINLCD_DATA(0x0002);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00d3); 
	MAINLCD_DATA(0x0001);
	MAINLCD_DATA(0x0022);
	MAINLCD_CMD(0x00d4);
	MAINLCD_DATA(0x0001);
	MAINLCD_DATA(0x0022);
	MAINLCD_CMD(0x00c0); /* panel scan */
	MAINLCD_DATA(0x0010);
	MAINLCD_DATA(0x003b);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0002);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00c1);
	MAINLCD_DATA(0x0010);
	MAINLCD_DATA(0x0010);
	MAINLCD_DATA(0x0022);
	MAINLCD_CMD(0x00c5);
	MAINLCD_DATA(0x0004); /* 03=72Hz 04=56Hz */
	MAINLCD_CMD(0x00c6);
	MAINLCD_DATA(0x001B); /* - - - !vsync !hsync - !en !pclk */
/* =========gamma====================== */
	MAINLCD_CMD(0x00c8);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0037);
	MAINLCD_DATA(0x0002);
	MAINLCD_DATA(0x0026);
	MAINLCD_DATA(0x000b);
	MAINLCD_DATA(0x0008);
	MAINLCD_DATA(0x0046);
	MAINLCD_DATA(0x0003);
	MAINLCD_DATA(0x0077);
	MAINLCD_DATA(0x0052);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x001e);
	MAINLCD_CMD(0x00e0);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00e1);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00e2);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00e3);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00b0);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00b1);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00b3);
	MAINLCD_DATA(0x0002);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0000);
	MAINLCD_DATA(0x0000);
	MAINLCD_CMD(0x00b4);
	MAINLCD_DATA(0x0011);
	MAINLCD_CMD(0x00F3);
	MAINLCD_DATA(0x0040);
	MAINLCD_DATA(0x000F);
	MAINLCD_CMD(0x00f6);
	MAINLCD_DATA(0x0080);
	MAINLCD_CMD(0x00f7);
	MAINLCD_DATA(0x0080);
	MAINLCD_DATA(0x0001);
	MAINLCD_CMD(0x0029);
	MAINLCD_CMD(0x002c);
}

static void lcdc_ili9481b_config_gpios(void)
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

static void ili9481b_hw_reset(void)
{
	RESET(0);
	mdelay(10);
	RESET(1);
	mdelay(120);
}

static void ili9481b_disp_on(int onoff)
{
	if (onoff)
		(void)onoff;	/* set display on */
	else
		(void)onoff;	/* set display off */
}

static void ili9481b_sleep(int onoff)
{
	if (onoff) {
		(void)onoff;	/* enter sleep mode */
		mdelay(10);
		driver_state.is_sleep = TRUE;
	} else {
		(void)onoff;  /* sleep out */
		mdelay(120);
		driver_state.is_sleep = FALSE;
	}
}


static int ili9481b_on(struct platform_device *pdev)
{
	if (!driver_state.disp_initialized) {
		/* Power Supply -> reset -> initail */
		lcdc_ili9481b_config_gpios();
		ili9481b_hw_reset();
		RGB_initial();
		mdelay(66); /* wait for screen update */
		driver_state.disp_initialized = TRUE;
	} else
		ili9481b_sleep(0);

	driver_state.display_on = TRUE;

	return 0;
}

static int ili9481b_off(struct platform_device *pdev)
{
	if (driver_state.display_on) {
		ili9481b_sleep(1);
		driver_state.display_on = FALSE;
		driver_state.disp_initialized = FALSE; /* force initial every time */
		MAINLCD_CMD(0x10); /* enter standby mode */
	}
	return 0;
}

static void ili9481b_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;

	if(driver_state.display_on) 
		driver_state.backlight_level =
			platform_data->pmic_backlight(bl_level);
}


static struct msm_fb_panel_data panel_data = {
        .on = ili9481b_on,
        .off = ili9481b_off,
        .set_backlight = ili9481b_set_backlight,
};

static struct platform_device this_device = {
        .name   = "lcdc_device",
        .id     = -1,
        .dev    = {
                .platform_data = &panel_data,
        }
};


static int __init lcdc_ili9481b_probe(struct platform_device *pdev)
{
	int ret = 0;

	if(platform_data) return -ENODEV;

	printk("[LCDC] lcdc_ili9481b_probe, id=%d\n", pdev->id);
	
	platform_data = pdev->dev.platform_data;
	lcdc_ili9481b_config_gpios();
	ili9481b_hw_reset();
	
	if(readID() != 0x01024A40) { /* mismatch */
		platform_data->panel_config_gpio(GPIO_DISABLE);
		platform_data = NULL;
		ret = -ENODEV;
	} else {
		printk("[LCDC] ili9481b probed on id %d\n", pdev->id);
		ret = 0;
	}

	return ret;
}


static struct platform_driver this_driver = {
        .probe  = lcdc_ili9481b_probe,
        .driver = {
                .name   = "lcdc_device",
        },
};



static int __init lcdc_ili9481b_init(void)
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

	pinfo->lcdc.h_back_porch = 32;
	pinfo->lcdc.h_front_porch = 20;
	pinfo->lcdc.h_pulse_width = 12;
	pinfo->lcdc.v_back_porch = 4;
	pinfo->lcdc.v_front_porch = 2;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

	if (platform_data) /* only when ID is sucessfully read */
		msm_fb_add_device(&this_device);
	else
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_ili9481b_init);

#ifdef CONFIG_DEBUG_FS

#include "linux/debugfs.h"

static int ili9481b_sleep_set_value(void *data, u64 val)
{
	if (val)
		ili9481b_sleep(1);
	else
		ili9481b_sleep(0);
	return 0;
}
static int ili9481b_sleep_get_value(void *data, u64 *val)
{
	*val = driver_state.is_sleep;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481b_sleep_fops, ili9481b_sleep_get_value,
			ili9481b_sleep_set_value, "%llu\n");

static int ili9481b_disp_on_set_value(void *data, u64 val)
{
	if (val)
		ili9481b_disp_on(1);
	else
		ili9481b_disp_on(0);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481b_disp_on_fops, NULL,
			ili9481b_disp_on_set_value, "%llu\n");


static int ili9481b_bl_set_value(void *data, u64 val)
{
	driver_state.backlight_level =
		platform_data->pmic_backlight((int)val);
	return 0;
}
static int ili9481b_bl_get_value(void *data, u64 *val)
{
	*val = driver_state.backlight_level;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481b_bl_fops, ili9481b_bl_get_value,
			ili9481b_bl_set_value, "%llu\n");

static int ili9481b_init_set_value(void *data, u64 val)
{
	driver_state.disp_initialized = FALSE;
        ili9481b_hw_reset();
        RGB_initial();
	//ili9481b_on(NULL);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481b_init_fops, NULL,
			ili9481b_init_set_value, "%llu\n");
		
	
static int __init ili9481b_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("ili9481b", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("init", 0644, dent, 0, &ili9481b_init_fops);
	debugfs_create_file("sleep", 0644, dent, 0, &ili9481b_sleep_fops);
	debugfs_create_file("bl", 0644, dent, 0, &ili9481b_bl_fops);
	debugfs_create_file("onoff", 0666, dent, 0, &ili9481b_disp_on_fops);

	return 0;
}

device_initcall(ili9481b_debug_init);
#endif /* CONFIG_DEBUG_FS */
