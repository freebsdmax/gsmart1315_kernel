/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/pmic.h>

#include <mach/oem_rapi_client.h>

/*
  Keyboard backlight adjust
  255 ==> 10 mA
  127 ==> 20 mA
   85 ==> 30 mA
*/

#define MAX_KEYPAD_BL_LEVEL	255
#define on	1
#define off	0
#define OEM_RAPI_CLIENT_EVENT_led_blink 21

static long blink_on = 50;
static long blink_off = 50;

static u8 blinkled[4];
static int grpfreq = 0;

void send_led_blink(void)
{

   struct msm_rpc_client *oem_client = oem_rapi_client_init();
   struct oem_rapi_client_streaming_func_arg arg;
   struct oem_rapi_client_streaming_func_ret ret;
   int input, retval, outlen, rc;

	// blinkled[1]=0; blink on/off
	// blinkled[2]=0; red on
	// blinkled[3]=0; green on
	
	if(blinkled[1] == on)
	{
		if(blinkled[2] == on)
		{
			if(grpfreq == 54)
			{
				blinkled[0] = 3; //red2 blink
			}
			else
			{
				blinkled[0] = 5; //red5 blink
			}
		}
		else
		{
			blinkled[0] = 4; //green blink
		}
	}
	else
	{
		if(blinkled[2] == off && blinkled[3] == off)
		{
			blinkled[0] = 0; //all off
		}
		else if(blinkled[2] == on && blinkled[3] == off)
		{
			blinkled[0] = 1; //red on
		}
		else if(blinkled[2] == off && blinkled[3] == on)
		{
			blinkled[0] = 2; //green on
		}
		else
		{
			blinkled[0] = 0; //all off
		}
	}
	
	arg.event = OEM_RAPI_CLIENT_EVENT_led_blink;
	arg.cb_func = NULL;
	arg.handle = 0;
	arg.in_len = 4;
	arg.input = (char *)blinkled;
	arg.out_len_valid = 1;
	arg.output_size = sizeof(retval);
	arg.output_valid = 1;

	outlen = sizeof(retval);
	ret.out_len = &outlen;
	ret.output = NULL;

	printk(KERN_ERR "leds send rpc [ %d , %d , %d, %d ]...\n",blinkled[0],blinkled[1],blinkled[2],blinkled[3]);
	rc = oem_rapi_client_streaming_function(oem_client, &arg, &ret);
 

	retval = *(u32 *)ret.output;
   
	if(ret.output != 0)
	{
		kfree(ret.output);
	}
	return 0;
}

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	printk(KERN_ERR "msm_keypad_bl_led_set= %d\n",value);
	if (value == 0)	pmic_set_led_intensity(LED_KEYPAD, 0);
	else 	pmic_set_led_intensity(LED_KEYPAD, 1);
}

static void msm_led_red_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	switch (value)
	{
		case LED_FULL:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif			
			blinkled[2] = on;
			break;
		case LED_HALF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif			
			blinkled[2] = on;
			break;
		case LED_OFF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#endif			
			blinkled[2] = off;
			break;
	}
}

static void msm_led_green_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	switch (value)
	{
		case LED_FULL:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif
			blinkled[3] = on;
			break;
		case LED_HALF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif
			blinkled[3] = on;
			break;
		case LED_OFF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#endif
			blinkled[3] = off;
			break;
	}
}
static void msm_led_blue_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	switch (value)
	{
		case LED_FULL:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_16,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_20,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif
			break;
		case LED_HALF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_16,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_20,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif
			break;
		case LED_OFF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_16,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_20,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#endif
			break;
	}
}


static void msm_led_amber_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	switch (value)
	{
		case LED_FULL:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
			pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
			pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif
			break;
			
		case LED_HALF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
			pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
			pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
#endif
			break;
			
		case LED_OFF:
#ifdef CONFIG_MACH_MSM7X25_QRD
			pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
			pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#else
			pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
			pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#endif
			break;
	}

}

static int msm_led_amber_blink_set (struct led_classdev *led_cdev, 
									unsigned long *delay_on,
									unsigned long *delay_off)
{
#ifdef CONFIG_MACH_MSM7X25_QRD
	pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
	pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);

	pmic_secure_mpp_config_i_sink(PM_MPP_14,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
	pmic_secure_mpp_config_i_sink(PM_MPP_15,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);

#else
	pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
	pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);

	pmic_secure_mpp_config_i_sink(PM_MPP_1,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
	pmic_secure_mpp_config_i_sink(PM_MPP_19,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
#endif
	//mdelay(*delay_on);
	
	//mdelay(*delay_off);
	
	return 0;
}


static struct led_classdev msm_kp_bl_led = {
	.name			= "button-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static struct led_classdev msm_led_red = {
	.name			= "red",
	.brightness_set	= msm_led_red_set,
	.brightness		= LED_OFF,

};

static struct led_classdev msm_led_green = {
	.name			= "green",
	.brightness_set	= msm_led_green_set,
	.brightness		= LED_OFF,
};

static struct led_classdev msm_led_blue = {
	.name			= "blue",
	.brightness_set	= msm_led_blue_set,
	.brightness		= LED_OFF,
};

static struct led_classdev msm_led_amber = {
	.name			= "amber",
	.brightness_set	= msm_led_amber_set,
	.brightness		= LED_OFF,
	.blink_set  	= msm_led_amber_blink_set,
};


/* Declare dev_attr_grpfreq */
static ssize_t led_grpfreq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t led_grpfreq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_ERR "leds grpfreq=%d\n", *buf);
	grpfreq = *buf;
	
	return 0;
}
static DEVICE_ATTR(grpfreq, 0644 , led_grpfreq_show, led_grpfreq_store);


/* Declare dev_attr_grppwm */
static ssize_t led_grppwm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return *buf;
}

static ssize_t led_grppwm_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_ERR "leds grppwm=%d\n",*buf);
	return 0;
}
static DEVICE_ATTR(grppwm, 0644 , led_grppwm_show, led_grppwm_store);


static ssize_t led_blink_solid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	printk(KERN_ERR "leds blink=%d\n",*buf);
	return *buf;
}

static ssize_t led_blink_solid_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	printk(KERN_ERR "leds blink=%d\n",*buf);
	if(*buf == 48)
	{
		blinkled[1] = off;
		send_led_blink();
	}
	else
	{
		blinkled[1] = on;
		send_led_blink();
	}
	return 0;
}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	led_classdev_register(&pdev->dev, &msm_led_red);
	led_classdev_register(&pdev->dev, &msm_led_green);
	led_classdev_register(&pdev->dev, &msm_led_blue);
	led_classdev_register(&pdev->dev, &msm_led_amber);
	led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	device_create_file(&pdev->dev,&dev_attr_grpfreq);
	device_create_file(&pdev->dev,&dev_attr_grppwm);
	device_create_file(&pdev->dev,&dev_attr_blink);
	

	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	msm_led_amber_set(&msm_led_amber,LED_OFF);
	msm_led_blue_set(&msm_led_blue,LED_OFF);
	msm_led_green_set(&msm_led_green,LED_OFF);
	msm_led_red_set(&msm_led_red,LED_OFF);
	
	
	return 0;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_led_red);
	led_classdev_unregister(&msm_led_green);
	led_classdev_unregister(&msm_led_blue);
	led_classdev_unregister(&msm_led_amber);
	led_classdev_unregister(&msm_kp_bl_led);
	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
