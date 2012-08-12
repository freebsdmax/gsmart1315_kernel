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
#include <linux/platform_device.h>

static int fake_onoff(struct platform_device *pdev)
{
	printk(">>>>>>>>>>> fake_onoff <<<<<<<<<<<<\n");
	(void)pdev;
	return 0;
}

static void fake_set_backlight(struct msm_fb_data_type *mfd)
{
	(void)mfd;
}

static struct msm_fb_panel_data panel_data = {
        .on = fake_onoff,
        .off = fake_onoff,
        .set_backlight = fake_set_backlight,
};

static struct platform_device this_device = {
        .name   = "psudo_device",
        .id     = -1,
        .dev    = {
                .platform_data = &panel_data,
        }
};

static int __init fake_probe(struct platform_device *pdev)
{
	if(pdev->id == -1)
		msm_fb_add_device(&this_device);
	return 0;
}

static struct platform_driver this_driver = {
        .probe  = fake_probe,
        .driver = {
                .name   = "lcdc_device",
        },
};

static int __init fake_init(void)
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

	printk("[Display] display is not detected, use a psudo device.\n");
	msm_fb_add_device(&this_device);

	return ret;
}

module_init(fake_init);

static void fake_exit(void)
{
	platform_driver_unregister(&this_driver);	
}
module_exit(fake_exit);


MODULE_AUTHOR("Sig <sigmund.lou@gigabyte.com.tw>");
MODULE_DESCRIPTION("a psudo panel deriver");
MODULE_LICENSE("GPL");

