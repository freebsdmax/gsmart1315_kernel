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

#define DEVICE_ID	0x01024A40
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

static void ili9481_hw_reset(void);
static void ili9481_set_backlight(struct msm_fb_data_type *mfd);

#ifdef CONFIG_DEBUG_FS
static int __init ili9481_debug_init(void);
#endif

static void LCD_ILI9481_CMD(unsigned short command)
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

static void LCD_ILI9481_PMR(unsigned short data)
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
	LCD_ILI9481_CMD(0x11);		//Sleep out

	mdelay(120);		//waiting for internal operation,* Must be satisfied*

//POWER SETTING
LCD_ILI9481_CMD(0xD0);
	LCD_ILI9481_PMR(0x07);	//VCI1=VCI
	LCD_ILI9481_PMR(0x42);	//PON=1,BT2-0=2
	LCD_ILI9481_PMR(0x1B);	//VREG1OUT=4.375V

LCD_ILI9481_CMD(0xD1);		//VCOM
	LCD_ILI9481_PMR(0x00);	//REGISTER SETTING
	LCD_ILI9481_PMR(0x14);	//VCOMH=0.785 X VREG1OUT
	LCD_ILI9481_PMR(0x1B);	//VDV=1.24 X VREG1OUT

LCD_ILI9481_CMD(0xD2);		
	LCD_ILI9481_PMR(0x01);	//AP2-0=1.0
	LCD_ILI9481_PMR(0x12);	//DC/DC2=FOSC/32, DC/DC1=FOSC/4
//

//PANEL SETTING
LCD_ILI9481_CMD(0xC0);		//Panel Driving Setting
	LCD_ILI9481_PMR(0x00);	//04 GS=0
	LCD_ILI9481_PMR(0x3B);	//480 LINE
	LCD_ILI9481_PMR(0x00);	//SCAN START LINE:1
	LCD_ILI9481_PMR(0x02);	//NDL
	LCD_ILI9481_PMR(0x01);	//PTG

/*
//Default setting
//
//Display_Timing_Setting for Normal Mode
LCD_ILI9481_CMD(0xC1);		//
	LCD_ILI9481_PMR(0x10);	//Line inversion,DIV1[1:0]
	LCD_ILI9481_PMR(0x10);	//RTN1[4:0]
	LCD_ILI9481_PMR(0x88);	//BP and FP


//Display_Timing_Setting for Partial Mode
LCD_ILI9481_CMD(0xC2);		//
	LCD_ILI9481_PMR(0x10);	//Line inversion,DIV1[1:0]
	LCD_ILI9481_PMR(0x10);	//RTN1[4:0]
	LCD_ILI9481_PMR(0x88);	//BP and FP
	

//Display_Timing_Setting for Idle Mode
LCD_ILI9481_CMD(0xC3);		//
	LCD_ILI9481_PMR(0x10);	//Line inversion,DIV1[1:0]
	LCD_ILI9481_PMR(0x10);	//RTN1[4:0]
	LCD_ILI9481_PMR(0x88);	//BP and FP
*/	
	
//DISPLAY MODE SETTING
LCD_ILI9481_CMD(0xC5);		//Frame rate and Inversion Control
	LCD_ILI9481_PMR(0x02);


//======RGB IF setting========
//RGB OR SYS INTERFACE
LCD_ILI9481_CMD(0xB4);
	LCD_ILI9481_PMR(0x10);//RGB

LCD_ILI9481_CMD(0xC6);
	LCD_ILI9481_PMR(0x1B);
//============================


LCD_ILI9481_CMD(0xC8);		//Gamma Setting
	LCD_ILI9481_PMR(0x00);
	LCD_ILI9481_PMR(0x46);
	LCD_ILI9481_PMR(0x44);
	LCD_ILI9481_PMR(0x50);
	LCD_ILI9481_PMR(0x04);
	LCD_ILI9481_PMR(0x16);
	LCD_ILI9481_PMR(0x33);
	LCD_ILI9481_PMR(0x13);
	LCD_ILI9481_PMR(0x77);
	LCD_ILI9481_PMR(0x05);
	LCD_ILI9481_PMR(0x0F);
	LCD_ILI9481_PMR(0x00);
//

////Internal command
LCD_ILI9481_CMD(0xE4);		//Internal LSI TEST Registers
	LCD_ILI9481_PMR(0xA0);
	
LCD_ILI9481_CMD(0xF0);
	LCD_ILI9481_PMR(0x01);
	
LCD_ILI9481_CMD(0xF3);
	LCD_ILI9481_PMR(0x40);
	LCD_ILI9481_PMR(0x0A);
	
LCD_ILI9481_CMD(0xF7);
	LCD_ILI9481_PMR(0x80);


//Address
LCD_ILI9481_CMD(0x36);		//Set_address_mode
	LCD_ILI9481_PMR(0x0A);	//1B

//
LCD_ILI9481_CMD(0x3A);		//Set_pixel_format
	LCD_ILI9481_PMR(0x66);	//5-16bit,6-18bit

	
//
LCD_ILI9481_CMD(0x2A);		//Set_column_address
	LCD_ILI9481_PMR(0x00);
	LCD_ILI9481_PMR(0x00);
	LCD_ILI9481_PMR(0x01);
	LCD_ILI9481_PMR(0x3F);

LCD_ILI9481_CMD(0x2B);		//Set_page_address
	LCD_ILI9481_PMR(0x00);
	LCD_ILI9481_PMR(0x00);
	LCD_ILI9481_PMR(0x01);
	LCD_ILI9481_PMR(0xDF);

//
LCD_ILI9481_CMD(0x13);		//NORMAL DISPLAY
	
mdelay(120);

LCD_ILI9481_CMD(0x29);		//Display on


//
LCD_ILI9481_CMD(0x002C);	//Write_memory_start


}

#ifdef CONFIG_DEBUG_FS

typedef struct ili9481_reg_cmd_struct {
        unsigned char val;
        unsigned char reg;
        unsigned short reserved;
/* 0xYYZZ, YY command and ZZ argument */
/* 0x00ZZ, more argument */
/* 0xFF00, terminate */
/* 0xFFZZ, delay ZZ ms */
} ili9481_reg_cmd;

static unsigned long ili9481_init_table[128] = {
//SLEEP OUT
0x1100,
0xFF78,
//POWER SETTING
0xD007, //VCI1=VCI
0x42, //PON=1,BT2-0=2
0x1B, //VREG1OUT=4.375V
//VCOM
0xD100, //REGISTER SETTING
0x14, //VCOMH=0.785 X VREG1OUT
0x1B, //VDV=1.24 X VREG1OUT

0xD201, //AP2-0=1.0
0x12, //DC/DC2=FOSC/32, DC/DC1=FOSC/4
//PANEL SETTING
//Panel Driving Setting
0xC000, //04 GS=0
0x3B, //480 LINE
0x00, //SCAN START LINE:1
0x02, //NDL
0x01, //PTG

/*
//Default setting
//
//Display_Timing_Setting for Normal Mode

0xC110, //Line inversion,DIV1[1:0]
0x0010, //RTN1[4:0]
0x0088, //BP and FP

//Display_Timing_Setting for Partial Mode
0xC210, //Line inversion,DIV1[1:0]
0x0010, //RTN1[4:0]
0x0088, //BP and FP

//Display_Timing_Setting for Idle Mode
0xC310, //Line inversion,DIV1[1:0]
0x0010, //RTN1[4:0]
0x0088, //BP and FP
*/	
	
//DISPLAY MODE SETTING
0xC502, //Frame rate and Inversion Control

//======RGB IF setting========
//RGB OR SYS INTERFACE
0xB410, //RGB
0xC61B, //POLARITY
//============================

//Gamma Setting
0xC800,
0x46,
0x44,
0x50,
0x04,
0x16,
0x33,
0x13,
0x77,
0x05,
0x0F,
0x00,
//

////Internal command
0xE4A0, //Internal LSI TEST Registers

0xF001,

0xF340,
0x0A,

0xF780,

//Address
0x360A, //Set_address_mode

//Set_pixel_format
0x3A66,	//5-16bit,6-18bit

	
//Set_column_address
0x2A00,
0x00,
0x01,
0x3F,

//Set_page_address
0x2B00,
0x00,
0x01,
0xDF,

//NORMAL DISPLAY
0x1300,
	
0xFF78, //mdelay

0x2900, //Display on

0x2C00, //Write_memory_start

0xFF00,
};

static void do_init_table(ili9481_reg_cmd *tbl)
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

static void lcdc_ili9481_config_gpios(void)
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

static void ili9481_hw_reset(void)
{
	RESET(0);
	msleep(1);
	RESET(1);
	msleep(100);
}

static int ili9481_on(struct platform_device *pdev)
{
	if (!driver_state.disp_initialized) {
		/* Power Supply -> reset -> initail */
		lcdc_ili9481_config_gpios();
		ili9481_hw_reset();
#ifdef CONFIG_DEBUG_FS
		do_init_table((ili9481_reg_cmd *)ili9481_init_table);
#else
		Lcd_init();
#endif
		driver_state.disp_initialized = TRUE;
	}

	driver_state.display_on = TRUE;

	return 0;
}

static int ili9481_off(struct platform_device *pdev)
{
	if (driver_state.display_on) {
		driver_state.display_on = FALSE;
		driver_state.disp_initialized = FALSE; /* initial every time */
	}
	return 0;
}

static void ili9481_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;

	if (driver_state.disp_initialized)
		driver_state.backlight_level =
			platform_data->pmic_backlight(bl_level);

	/* workaround: avoid fast resume issue */
	if (0 == driver_state.backlight_level)
		driver_state.disp_initialized = FALSE;
}


static struct msm_fb_panel_data panel_data = {
        .on = ili9481_on,
        .off = ili9481_off,
        .set_backlight = ili9481_set_backlight,
};

static struct platform_device this_device = {
        .name   = "lcdc_device",
        .id     = -1,
        .dev    = {
                .platform_data = &panel_data,
        }
};

static int __init lcdc_ili9481_probe(struct platform_device *pdev)
{
	int ret = 0;

	if(platform_data) return -ENODEV;

	platform_data = pdev->dev.platform_data;
	lcdc_ili9481_config_gpios();
#ifndef CONFIG_LCD_PRE_RESET_IN_FASTBOOT
	ili9481_hw_reset();
#endif
	if(DEVICE_ID == readID()) {
		printk("[LCDC] ili9481 probed on id %d\n", pdev->id);
#ifdef CONFIG_DEBUG_FS
		ili9481_debug_init();
#endif
	} else {
		platform_data = NULL;
		ret = -ENODEV;
	}

	return ret;
}

static struct platform_driver this_driver = {
        .probe  = lcdc_ili9481_probe,
        .driver = {
                .name   = "lcdc_device",
        },
};

static int __init lcdc_ili9481_init(void)
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

module_init(lcdc_ili9481_init);

#ifdef CONFIG_DEBUG_FS

#include "linux/debugfs.h"

static ssize_t default_read_file(struct file *file, char __user *user_buf,
                                 size_t count, loff_t *ppos)
{
	unsigned int *iter = (void *)&ili9481_init_table[0];
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
	ili9481_reg_cmd *target=(void *)&ili9481_init_table[0];

	if(!ptr) return 0;
	do {
		pair = simple_strtoul(ptr, &ptr, 16);
		*(unsigned long*)target = pair;
		++target;
		ptr = strchr(ptr, ',');
	} while(ptr++);

	return count;
}

static const struct file_operations ili9481_init_tbl_fops = {
	.read =  default_read_file,
	.write = default_write_file,
};

static int ili9481_hw_reset_set_value(void *data, u64 val)
{
	lcdc_ili9481_config_gpios();
	ili9481_hw_reset();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481_hw_reset_fops, NULL,
			ili9481_hw_reset_set_value, "%llu\n");

static int ili9481_bl_set_value(void *data, u64 val)
{
	driver_state.backlight_level =
		platform_data->pmic_backlight((int)val);
	return 0;
}
static int ili9481_bl_get_value(void *data, u64 *val)
{
	*val = driver_state.backlight_level;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481_bl_fops, ili9481_bl_get_value,
			ili9481_bl_set_value, "%llu\n");

static int ili9481_do_init_set_value(void *data, u64 val)
{
	do_init_table((ili9481_reg_cmd *)ili9481_init_table);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ili9481_do_init_fops, NULL,
			ili9481_do_init_set_value, "%llu\n");

static int __init ili9481_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("ili9481", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("init_tbl", 0644, dent, 0, &ili9481_init_tbl_fops);
	debugfs_create_file("do_init", 0644, dent, 0, &ili9481_do_init_fops);
	debugfs_create_file("bl", 0644, dent, 0, &ili9481_bl_fops);
	debugfs_create_file("hw_reset", 0666, dent, 0, &ili9481_hw_reset_fops);

	return 0;
}

#endif /* CONFIG_DEBUG_FS */
