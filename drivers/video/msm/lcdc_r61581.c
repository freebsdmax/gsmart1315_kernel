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
#define SDO(a)		gpio_set_value(spi_sdi, a)
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
static struct msm_panel_common_pdata *platform_data;

static void r61581_hw_reset(void);
static void r61581_set_backlight(struct msm_fb_data_type *mfd);
static void r61581_sleep(int);


void WMLCDCOM(unsigned short command)
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

void WMLCDDATA(unsigned short data)
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

void spi_init_cmds(void)
{
	WMLCDCOM(0x00B0);
	WMLCDDATA(0x0000);

	WMLCDCOM(0x00B3);
	WMLCDDATA(0x0002);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0000);

	WMLCDCOM(0x00B4);
	WMLCDDATA(0x0011);

	WMLCDCOM(0x00C6);
	WMLCDDATA(0x0033);

	WMLCDCOM(0x00C0);
	WMLCDDATA(0x0003);
	WMLCDDATA(0x003B);

	WMLCDDATA(0x0000);
	WMLCDDATA(0x0002);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0001);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0043);

	WMLCDCOM(0x00C1);
	WMLCDDATA(0x0008);
	WMLCDDATA(0x0017);
	WMLCDDATA(0x0008);
	WMLCDDATA(0x0008);

	WMLCDCOM(0x00C4);
	WMLCDDATA(0x0022);
	WMLCDDATA(0x0002);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0000);

	WMLCDCOM(0x00C8);
	WMLCDDATA(0x0009);
	WMLCDDATA(0x0008);
	WMLCDDATA(0x0010);
	WMLCDDATA(0x0085);
	WMLCDDATA(0x0007);
	WMLCDDATA(0x0008);
	WMLCDDATA(0x0016);
	WMLCDDATA(0x0005);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0032);
	WMLCDDATA(0x0005);
	WMLCDDATA(0x0016);
	WMLCDDATA(0x0008);
	WMLCDDATA(0x0088);
	WMLCDDATA(0x0009);
	WMLCDDATA(0x0010);
	WMLCDDATA(0x0009);
	WMLCDDATA(0x0004);
	WMLCDDATA(0x0032);
	WMLCDDATA(0x0000);

	WMLCDCOM(0x002A);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0001);
	WMLCDDATA(0x003F);

	WMLCDCOM(0x002B);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0001);
	WMLCDDATA(0x00DF);

	WMLCDCOM(0x0035);
	WMLCDDATA(0x0000);

	WMLCDCOM(0x003A);
	WMLCDDATA(0x0066);

	WMLCDCOM(0x0044);
	WMLCDDATA(0x0000);
	WMLCDDATA(0x0001);

	WMLCDCOM(0x002C);
	WMLCDCOM(0x0011);

	mdelay(150);

	WMLCDCOM(0x00D0);
	WMLCDDATA(0x0007);
	WMLCDDATA(0x0007);
	WMLCDDATA(0x0016);
	WMLCDDATA(0x0072);

	WMLCDCOM(0x00D1);
	WMLCDDATA(0x0003);
	WMLCDDATA(0x003A);
	WMLCDDATA(0x000A);

	WMLCDCOM(0x00D2);
	WMLCDDATA(0x0002);
	WMLCDDATA(0x0044);
	WMLCDDATA(0x0004);

	WMLCDCOM(0x0029);

	WMLCDCOM(0x002C);

	mdelay(10);

}

static void lcdc_r61581_config_gpios(void)
{
	/* Setting the Default GPIO's */
	spi_sdi = platform_data->gpio_num[0];
	spi_sdo = platform_data->gpio_num[1];
	lcd_reset = platform_data->gpio_num[2];
	spi_cs  = platform_data->gpio_num[3];
	spi_clk = platform_data->gpio_num[4];

	platform_data->panel_config_gpio(GPIO_ENABLE);

	RESET(0);
	SCLK(1);
	CS(1);

}

static void r61581_hw_reset(void)
{
	RESET(0);
	mdelay(5);
	RESET(1);
	mdelay(5);
}


static void r61581_disp_on(int onoff)
{
	if (onoff)
		WMLCDCOM(0x0029);	/* set display on */
	else
		WMLCDCOM(0x0028);	/* set display off */
}

static void r61581_sleep(int onoff)
{
	if (onoff) {
		WMLCDCOM(0x0010);	/* enter sleep mode */
		mdelay(10);
		driver_state.is_sleep = TRUE;
	} else {
		WMLCDCOM(0x0011);  /* sleep out */
		mdelay(120);
		driver_state.is_sleep = FALSE;
	}
}

static int r61581_on(struct platform_device *pdev)
{
	if (!driver_state.disp_initialized) {
		/* Power Supply -> reset -> initail */
		r61581_hw_reset();
		spi_init_cmds();
		driver_state.disp_initialized = TRUE;
	} else
		r61581_sleep(0);

	driver_state.display_on = TRUE;

	return 0;
}

static int r61581_off(struct platform_device *pdev)
{
	if (driver_state.display_on) {
		r61581_sleep(1);
		driver_state.display_on = FALSE;
	}
	return 0;
}

static void r61581_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;

	driver_state.backlight_level =
		platform_data->pmic_backlight(bl_level);
}

static int __init lcdc_r61581_probe(struct platform_device *pdev)
{
	if (pdev->id == 0)
		platform_data = pdev->dev.platform_data;
	if (pdev->id == 1) {
		lcdc_r61581_config_gpios();
		msm_fb_add_device(pdev);
	}
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = lcdc_r61581_probe,
	.driver = {
		.name   = "lcdc_r61581",
	},
};

static struct msm_fb_panel_data panel_data = {
	.on = r61581_on,
	.off = r61581_off,
	.set_backlight = r61581_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_r61581",
	.id	= 1,
	.dev	= {
		.platform_data = &panel_data,
	}
};

static int __init lcdc_r61581_init(void)
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
	pinfo->bl_max = 18;
	pinfo->bl_min = 0;

	pinfo->lcdc.h_back_porch = 20;
	pinfo->lcdc.h_front_porch = 10;
	pinfo->lcdc.h_pulse_width = 10;
	pinfo->lcdc.v_back_porch = 2;
	pinfo->lcdc.v_front_porch = 4;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_r61581_init);

#ifdef CONFIG_DEBUG_FS

#include "linux/debugfs.h"

static int r61581_sleep_set_value(void *data, u64 val)
{
	if (val)
		r61581_sleep(1);
	else
		r61581_sleep(0);
	return 0;
}
static int r61581_sleep_get_value(void *data, u64 *val)
{
	*val = driver_state.is_sleep;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(r61581_sleep_fops, r61581_sleep_get_value,
			r61581_sleep_set_value, "%llu\n");

static int r61581_disp_on_set_value(void *data, u64 val)
{
	if (val)
		r61581_disp_on(1);
	else
		r61581_disp_on(0);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(r61581_disp_on_fops, NULL,
			r61581_disp_on_set_value, "%llu\n");


static int r61581_bl_set_value(void *data, u64 val)
{
	driver_state.backlight_level =
		platform_data->pmic_backlight((int)val);
	return 0;
}
static int r61581_bl_get_value(void *data, u64 *val)
{
	*val = driver_state.backlight_level;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(r61581_bl_fops, r61581_bl_get_value,
			r61581_bl_set_value, "%llu\n");

static int r61581_init_set_value(void *data, u64 val)
{
	driver_state.disp_initialized = FALSE;
	r61581_on(NULL);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(r61581_init_fops, NULL,
			r61581_init_set_value, "%llu\n");

static int __init r61581_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("r61581", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("init", 0644, dent, 0, &r61581_init_fops);
	debugfs_create_file("sleep", 0644, dent, 0, &r61581_sleep_fops);
	debugfs_create_file("bl", 0644, dent, 0, &r61581_bl_fops);
	debugfs_create_file("onoff", 0666, dent, 0, &r61581_disp_on_fops);

	return 0;
}

device_initcall(r61581_debug_init);
#endif /* CONFIG_DEBUG_FS */
