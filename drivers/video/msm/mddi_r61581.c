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

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#include <mach/gpio.h>

static struct msm_panel_common_pdata *r61581_pdata;

#define RESET(a)	gpio_set_value(r61581_pdata->gpio, a)

#define GPIO_HIGH_VALUE 1
#define GPIO_LOW_VALUE	0

static u64 r61581_bl_level;
static int r61581_initialized;

static int mddi_r61581_lcd_on(struct platform_device *pdev);
static int mddi_r61581_lcd_off(struct platform_device *pdev);
static void mddi_r61581_sleep(u32 sleepin);
static boolean serigo(uint32 reg, uint32 len, const char *params, boolean wait);
static void mddi_r61581_set_backlight(struct msm_fb_data_type *fb_data);
static void mddi_r61581_hw_reset(void);
static void mddi_r61581_sleep(u32 sleepin);

static void mddi_r61581_set_backlight(struct msm_fb_data_type *fb_data)
{
	r61581_bl_level = fb_data->bl_level;
	r61581_pdata->pmic_backlight(r61581_bl_level);
}

static void mddi_r61581_hw_reset(void)
{
	RESET(GPIO_LOW_VALUE);
	mddi_wait(10);
	RESET(GPIO_HIGH_VALUE);
	mddi_wait(10);
}

static boolean serigo(uint32 reg, uint32 len, const char *params, boolean wait)
{
	uint32 rv;
	uint32 data_list[20];
	uint i;

	for (i = 0; i < len; ++i)
		data_list[i] = (u32)params[i] & 0xFF;

	if (len > 0)
		rv = mddi_host_register_multiwrite(reg, len, data_list, wait, \
							NULL, MDDI_HOST_PRIM);
	else
		rv = mddi_queue_register_write(reg, 0, TRUE, 0);

	mddi_wait(10);
	return rv;
}


static int mddi_r61581_lcd_on(struct platform_device *pdev)
{
	MDDI_MSG_DEBUG("[MDDI] turning r61581 on\n");
	if (r61581_initialized) {
		mddi_r61581_sleep(0);
	} else {
		mddi_r61581_hw_reset();

		mddi_wait(10);

		serigo(0x00B0, 1, "\x0", TRUE);
		serigo(0x00B3, 4, "\x02\x00\x00\x00", TRUE);
		serigo(0x00B4, 1, "\x00", TRUE);
		serigo(0x00C0, 8, "\x03\x3B\x00\x02\x00\x01\x00\x43", TRUE);
		serigo(0x00C1, 4, "\x08\x17\x08\x08", TRUE);

		serigo(0x00C4, 4, "\x22\x02\x00\x00", TRUE);
		serigo(0x00C8, 20, "\x09\x08\x10\x85\x07\x08\x16\x05\x00\x32"
			"\x05\x16\x08\x88\x09\x10\x09\x04\x32\x00", TRUE);

		serigo(0x002A, 4, "\x00\x00\x01\x3F", TRUE);
		serigo(0x002B, 4, "\x00\x00\x01\xDF", TRUE);
		serigo(0x0035, 1, "\x00", TRUE);
		serigo(0x003A, 1, "\x66", TRUE);
		serigo(0x0044, 2, "\x00\x01", TRUE);
		serigo(0x002C, 0, "\x00", TRUE);
		serigo(0x0011, 0, "\x00", TRUE);

		mddi_wait(150);

		serigo(0x00D0, 4, "\x07\x07\x16\x72", TRUE);
		serigo(0x00D1, 3, "\x03\x3A\x0A", TRUE);
		serigo(0x00D2, 3, "\x02\x44\x04", TRUE);
		serigo(0x0029, 0, "\x00", TRUE);
		serigo(0x002C, 0, "\x00", TRUE);

		mddi_wait(10);
		r61581_initialized = TRUE;
	}
	return 0;
}

static int mddi_r61581_lcd_off(struct platform_device *pdev)
{
	MDDI_MSG_DEBUG("[MDDI] turning r61581 off\n");

	if (r61581_initialized)
		mddi_r61581_sleep(1);

	return 0;
}

static int __init mddi_r61581_probe(struct platform_device *pdev)
{
	if (0 == pdev->id) {
		r61581_pdata = pdev->dev.platform_data;
		r61581_pdata->panel_config_gpio(0);
		r61581_pdata->pmic_backlight(2);
		return 0;
	}
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mddi_r61581_probe,
	.driver = {
		.name   = "mddi_r61581",
	},
};

static struct msm_fb_panel_data r61581_panel_data = {
	.on = mddi_r61581_lcd_on,
	.off = mddi_r61581_lcd_off,
	.set_backlight = mddi_r61581_set_backlight,
};

static struct platform_device this_device = {
	.name   = "mddi_r61581",
	.id	= 1,
	.dev	= {
		.platform_data = &r61581_panel_data,
	}
};

static int __init mddi_r61581_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	printk(KERN_DEBUG"[MDDI] Initializing R61581\n");

	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &r61581_panel_data.panel_info;
		pinfo->xres = 320;
		pinfo->yres = 480;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 18;
		pinfo->fb_num = 2;
		pinfo->clk_rate = 120000000;
		pinfo->clk_min =  120000000;
		pinfo->clk_max =  120000000;
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.refx100 = 6050;
		pinfo->lcd.v_back_porch = 2;
		pinfo->lcd.v_front_porch = 4;
		pinfo->lcd.v_pulse_width = 2;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;
		pinfo->bl_max = 18;
		pinfo->bl_min = 0;

		ret = platform_device_register(&this_device);
		if (ret)
			platform_driver_unregister(&this_driver);
	}

	return ret;
}
module_init(mddi_r61581_init);


static void mddi_r61581_sleep(u32 sleepin)
{
	if (sleepin) {
		serigo(0x0028, 0, "\x00", TRUE);
		serigo(0x0010, 0, "\x00", TRUE);
	} else {
		serigo(0x0011, 0, "\x00", TRUE);
		mddi_wait(120);
		serigo(0x0029, 0, "\x00", TRUE);
	}
}

#ifdef CONFIG_DEBUG_FS

#include "linux/debugfs.h"

static int mddi_r61581_sleep_set_value(void *data, u64 val)
{
	if (val)
		mddi_r61581_sleep(1);
	else
		mddi_r61581_sleep(0);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mddi_r61581_sleep_fops, 0,
			mddi_r61581_sleep_set_value, "%llu\n");

static int mddi_r61581_bl_set_value(void *data, u64 val)
{
	r61581_bl_level = val;
	r61581_pdata->pmic_backlight((int)val);
	return 0;
}
static int mddi_r61581_bl_get_value(void *data, u64 *val)
{
	*val = r61581_bl_level;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mddi_r61581_bl_fops, mddi_r61581_bl_get_value,
			mddi_r61581_bl_set_value, "%llu\n");

static int __init mddi_r61581_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("mddi_r61581", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("sleep", 0644, dent, 0, &mddi_r61581_sleep_fops);
	debugfs_create_file("bl", 0644, dent, 0, &mddi_r61581_bl_fops);

	return 0;
}
device_initcall(mddi_r61581_debug_init);
#endif /* CONFIG_DEBUG_FS */

