#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ov5640.h"
#include <mach/vreg.h>

#include <mach/pmic.h>
//Camera Power
#ifdef CONFIG_MACH_MSM7X25_QRD
static struct vreg *vreg_CAM_gp3;      //power 1.5v
static struct vreg *vreg_CAM_gp6;      //power 2.8v
static struct vreg *vreg_CAM_wlan;     //power 2.8v
#else
static struct vreg *vreg_CAM_gp6;      //power 2.8v
#endif 

//Camera init flag for check if it is inited.
//static bool camera_init_flag;

struct ov5640_work {
   struct work_struct work;
};
static struct ov5640_work *ov5640_sensorw;
static struct i2c_client    *ov5640_client;
static DECLARE_WAIT_QUEUE_HEAD(ov5640_wait_queue);
DEFINE_MUTEX(ov5640_mutex);
static u8 ov5640_i2c_buf[4];
static u8 ov5640_counter = 0;
static int16_t ov5640_effect = CAMERA_EFFECT_OFF;
static int is_autoflash =0;
static int effect_value = 0;	
struct __ov5640_ctrl 
{
	const struct msm_camera_sensor_info *sensordata;
	int sensormode;
	uint fps_divider; /* init to 1 * 0x00000400 */
	uint pict_fps_divider; /* init to 1 * 0x00000400 */
	u16 curr_step_pos;
	u16 curr_lens_pos;
	u16 init_curr_lens_pos;
	u16 my_reg_gain;
	u16 my_reg_line_count;
	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
};
static struct __ov5640_ctrl *ov5640_ctrl;
int afinit = -1;

static int ov5640_i2c_remove(struct i2c_client *client);
static int ov5640_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int ov5640_sensor_start_af(void);

static int ov5640_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(ov5640_client->adapter, msg, 1) < 0)	return -EIO;
	else return 0;
}

static int ov5640_i2c_write(unsigned short saddr, unsigned int waddr,
	unsigned short bdata,u8 trytimes)
{
   int rc = -EIO;
   ov5640_counter = 0;
   ov5640_i2c_buf[0] = (waddr & 0xFF00)>>8;
   ov5640_i2c_buf[1] = (waddr & 0x00FF);
   //ov5640_i2c_buf[2] = (bdata & 0xFF00)>>8;
   ov5640_i2c_buf[2] = (bdata & 0x00FF);
   
   while ( (ov5640_counter<trytimes) &&(rc != 0) )
   {
      rc = ov5640_i2c_txdata(saddr, ov5640_i2c_buf, 3);
      if (rc < 0)
      {
      	ov5640_counter++;
      	printk(KERN_ERR "--CAMERA-- i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,ov5640_counter,rc);
      	msleep(4);
      }
      else 
	  {
		//printk(KERN_ERR "--CAMERA--i2c_write_w ok!\n");
	  }
   }
   return rc;
}



/*
static int ov5640_i2c_write_table(u32 p[][2],u8 trytimes)
{
	int i=0,rc=0;
	for(;i<(sizeof(p)/sizeof(p[0])/2);i++)
	{
		rc = ov5640_i2c_write(ov5640_client->addr,p[i][0],p[i][1],trytimes);
		if (rc < 0) return rc;
	}
	return rc;
}
*/
static int ov5640_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ov5640_client->adapter, msgs, 2) < 0) {
		CDBG("ov5640_i2c_rxdata failed!\n");
		printk(KERN_ERR "ov5640_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t ov5640_i2c_read(unsigned short   saddr,
	unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	#ifdef CAM_DBG
	printk(KERN_ERR "+ov5640_i2c_read\n");
	#endif
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov5640_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)	CDBG("ov5640_i2c_read failed!\n");
	#ifdef CAM_DBG
	printk(KERN_ERR "-ov5640_i2c_read\n");
	#endif
	return rc;
}

static int32_t ov5640_i2c_read_byte(unsigned short   saddr,
    unsigned int raddr, unsigned int *rdata)
{
    int rc = 0;
    unsigned char buf[2];
    #ifdef CAM_DBG
    printk(KERN_ERR "+ov5640_i2c_read\n");
    #endif
    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00)>>8;
    buf[1] = (raddr & 0x00FF);

    rc = ov5640_i2c_rxdata(saddr, buf, 1);
    if (rc < 0) return rc;
    *rdata = buf[0];

    if (rc < 0) CDBG("ov5640_i2c_read failed!\n");
    #ifdef CAM_DBG
    printk(KERN_ERR "-ov5640_i2c_read\n");
    #endif
    return rc;
}

static void camera_power_onoff(u8 v)
{
#ifdef CONFIG_MACH_MSM7X25_QRD
	if (v==1)
	{
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- power on~!!\n");
		#endif
		vreg_enable(vreg_CAM_wlan);
		msleep(5);
		vreg_enable(vreg_CAM_gp3);
		msleep(5);
		vreg_enable(vreg_CAM_gp6);
		msleep(5);
	   	afinit = 1;	//af state
	}
	else
	{
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- power off~!!\n");
		#endif
		vreg_disable(vreg_CAM_gp6);
		msleep(5);
		vreg_disable(vreg_CAM_gp3);
		msleep(5);
		vreg_disable(vreg_CAM_wlan);
		afinit = 0;	//af state
	}
#else
	if (v==1)
	{
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- power on~!!\n");
		#endif
		vreg_enable(vreg_CAM_gp6);
		msleep(1);
		gpio_set_value(1, 1);
		msleep(1);
		gpio_set_value(0, 1);
		msleep(1);
		afinit = 1;	//af state
	}else
	{
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- power off~!!\n");
		#endif
		msleep(1);
		gpio_set_value(1, 0);
		msleep(1);
		vreg_disable(vreg_CAM_gp6);
		msleep(1);
		afinit = 0;	//af state
	}
#endif
}

static int ov5640_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
   #ifdef CAM_DBG
   printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
   #endif
   // (1) set Camera PMIC and power on
   //vreg_CAM_gp3 = vreg_get(NULL, "gp3");
   //vreg_CAM_gp6 = vreg_get(NULL, "gp6");
   //vreg_CAM_wlan = vreg_get(NULL, "wlan");
   
   /* --- can Move to AMSS -------------- */
   //vreg_set_level(vreg_CAM_wlan, 2800);
   //vreg_set_level(vreg_CAM_gp3,  1500);
   //vreg_set_level(vreg_CAM_gp6,  2800);
   /* ----------------------------------- */   
 
   // (2) config pwd and rest pin
   #ifdef CAM_DBG
   printk(KERN_ERR "--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);
   #endif
   //gpio_request(data->sensor_pwd, "ov5640");
   //gpio_request(data->sensor_reset, "ov5640");
   //gpio_direction_output(data->sensor_pwd, 1);
   //gpio_direction_output(data->sensor_reset, 1);

   // (3) Set Clock = 48 MHz
   //msm_camio_clk_rate_set(24000000);
   //mdelay(5);
   //msm_camio_camif_pad_reg_reset();
   //mdelay(5);

   // (4) Power On
   //camera_power_onoff(1);
   //gpio_set_value(data->sensor_pwd, 0);
  // mdelay(1);
   //gpio_set_value(data->sensor_reset, 1);
   //mdelay(5);
   //gpio_set_value(data->sensor_reset, 0);
   //mdelay(5);
   //gpio_set_value(data->sensor_reset, 1);
   
   //camera_init_flag=true;
   // (5) Into Stand by mode
   //ov5640_i2c_write(ov5640_client->addr,REG_MODE_SELECT,MODE_SELECT_SW_STANDBY,10);
   
   // (6) Read Device ID
   //ov5640_i2c_read(ov5640_client->addr, 0x300A, &device_id);
   
   //printk(KERN_ERR "--CAMERA-- %s ok , device id=0x%x\n",__func__,device_id);
   return 0;
}

static int ov5640_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	u8 tmp;
	int len;
	int i = 0;
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- %s (Start...), rupdate=%d \n",__func__,rupdate);
	#endif
	len = sizeof(preview_ov5640_reg)/sizeof(preview_ov5640_reg[0]);
	
	switch (rupdate)
	{
		case S_UPDATE_PERIODIC:
			break; /* UPDATE_PERIODIC */
		case S_REG_INIT:
			printk(KERN_ERR "--CAMERA-- S_REG_INIT (Start)\n");

			//set register setting
			for(i = 0;i<len;i++)
			{
			   rc = ov5640_i2c_write(ov5640_client->addr, \
			        preview_ov5640_reg[i][0], preview_ov5640_reg[i][1],10);
			   //udelay(500);
			}	

      			rc =ov5640_i2c_read_byte(ov5640_client->addr, \
                                    0x4740, &tmp);			   
      			printk(KERN_ERR "--CAMERA-- %s init 0x4740 value=0x%x\n",__func__,tmp);

			if(tmp !=0x21)
			{
				 rc = ov5640_i2c_write(ov5640_client->addr,0x4740,0x21,10);
				 msleep(10);
				 rc =ov5640_i2c_read_byte(ov5640_client->addr, \
                                    0x4740, &tmp);			   
      				printk(KERN_ERR "--CAMERA-- %s WG 0x4740 value=0x%x\n",__func__,tmp);
			}
				
			/* reset fps_divider */
			ov5640_ctrl->fps_divider = 1 * 0x0400;
			printk(KERN_ERR "--CAMERA-- S_REG_INIT (End)\n");
			break; /* case REG_INIT: */
		default:
			break;
	} /* switch (rupdate) */
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);
	#endif
	return rc;
}

static int ov5640_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = -ENOMEM;
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- %s\n",__func__);
	#endif
	ov5640_ctrl = kzalloc(sizeof(struct __ov5640_ctrl), GFP_KERNEL);
	if (!ov5640_ctrl)
	{
		printk(KERN_ERR "--CAMERA-- kzalloc ov5640_ctrl error !!\n");
		kfree(ov5640_ctrl);
		return rc;
	}
	ov5640_ctrl->fps_divider = 1 * 0x00000400;
	ov5640_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov5640_ctrl->set_test = S_TEST_OFF;
	ov5640_ctrl->prev_res = S_QTR_SIZE;
	ov5640_ctrl->pict_res = S_FULL_SIZE;
	
	if (data) ov5640_ctrl->sensordata = data;

	/* enable mclk = 50 MHz first */
	msm_camio_clk_rate_set(24000000);
	//mdelay(5);

	msm_camio_camif_pad_reg_reset();
	//mdelay(5);

	camera_power_onoff(1);

	gpio_set_value(data->sensor_pwd, 1);
	msleep(1);
	gpio_set_value(data->sensor_pwd, 0);
	msleep(5);
	gpio_set_value(data->sensor_reset, 1);
   	msleep(5);
   	gpio_set_value(data->sensor_reset, 0);
   	msleep(5);
   	gpio_set_value(data->sensor_reset, 1);
	msleep(1);
	
	if (ov5640_ctrl->prev_res == S_QTR_SIZE)
		rc = ov5640_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = ov5640_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		printk(KERN_ERR "--CAMERA-- %s : ov5640_setting failed. rc = %d\n",__func__,rc);
		kfree(ov5640_ctrl);
		return rc;
	}
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA--re_init_sensor ok!!\n");
	#endif
	return rc;
}

static int ov5640_sensor_release(void)
{
	printk(KERN_ERR "--CAMERA--ov5640_sensor_release!!\n");

	mutex_lock(&ov5640_mutex);

	camera_power_onoff(0);
	
	gpio_set_value(ov5640_ctrl->sensordata->sensor_reset, 1);
	
	//gpio_free(ov5640_ctrl->sensordata->sensor_reset);
	//gpio_free(ov5640_ctrl->sensordata->sensor_pwd);

	kfree(ov5640_ctrl);
	ov5640_ctrl = NULL;

	mutex_unlock(&ov5640_mutex);
	return 0;
}


			       
static const struct i2c_device_id ov5640_i2c_id[] = {
	{"ov5640", 0},{}
};

static int ov5640_i2c_remove(struct i2c_client *client)
{
   return 0;
}


static int ov5640_init_client(struct i2c_client *client)
{
   /* Initialize the MSM_CAMI2C Chip */
   init_waitqueue_head(&ov5640_wait_queue);
   return 0;
}
/*
static int ov5640_raw_snapshot_config(int mode)
{
	int rc;
	rc = ov5640_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)	return rc;
	ov5640_ctrl->curr_res = ov5640_ctrl->pict_res;
	ov5640_ctrl->sensormode = mode;
	return rc;
}
*/
static long ov5640_set_effect(int mode, int effect)
{
	uint16_t i = 0;
	uint16_t len;
	long rc = 0;
	#ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
	#endif

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...SENSOR_PREVIEW_MODE\n",__func__);
		#endif
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...SENSOR_SNAPSHOT_MODE\n",__func__);
		#endif
		break;

	default:
		break;
	}

      effect_value = effect;

	switch (effect) {
	case CAMERA_EFFECT_OFF: {
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...CAMERA_EFFECT_OFF\n",__func__);
		#endif
		len = sizeof(ov5640_effect_normal_tbl)/sizeof(ov5640_effect_normal_tbl[0]);

			for(i = 0;i<len;i++)
			{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_effect_normal_tbl[i][0],
					     ov5640_effect_normal_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
			}
	}
			break;

	case CAMERA_EFFECT_MONO: {
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...CAMERA_EFFECT_MONO(Not Support)\n",__func__);
		#endif
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...CAMERA_EFFECT_NEGATIVE\n",__func__);
		#endif	
		len = sizeof(ov5640_effect_negative_tbl)/sizeof(ov5640_effect_negative_tbl[0]);

			for(i = 0;i<len;i++)
			{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_effect_negative_tbl[i][0],
					     ov5640_effect_negative_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
			}
	}
		break;

	case CAMERA_EFFECT_SOLARIZE: {
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...CAMERA_EFFECT_SOLARIZE(Not Support)\n",__func__);
		#endif
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s ...CAMERA_EFFECT_SEPIA\n",__func__);
		#endif
		len = sizeof(ov5640_effect_sepia_tbl)/sizeof(ov5640_effect_sepia_tbl[0]);

			for(i = 0;i<len;i++)
			{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_effect_sepia_tbl[i][0],
					     ov5640_effect_sepia_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
			}
			
	}
		break;

	default: {
		
		printk(KERN_ERR "--CAMERA-- %s ...Default(Not Support)\n",__func__);
	}
	}
	ov5640_effect = effect;
	/* Refresh Sequencer */
	#ifdef CAM_DBG
       printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
	#endif
	return rc;
}

static int ov5640_set_brightness(int8_t brightness)
{
    long rc = 0;
    int i =0;
    #ifdef CAM_DBG
    printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
    #endif
    #ifdef CAM_DBG
    printk(KERN_ERR "--CAMERA-- %s ...brightness = %d\n",__func__ , brightness);
    #endif

    switch (brightness)
    {
        case CAMERA_BRIGHTNESS_LV0:
	     #ifdef CAM_DBG
            printk(KERN_ERR "--CAMERA--CAMERA_BRIGHTNESS_LV0\n");
	     #endif
            for(i = 0;i<6;i++)
            {
                rc = ov5640_i2c_write(ov5640_client->addr,
                        ov5640_brightness_lv0_tbl[i][0],
                        ov5640_brightness_lv0_tbl[i][1],
                        10);
                if (rc < 0)	return rc;     
            }				
            break;
        case CAMERA_BRIGHTNESS_LV1:
	     #ifdef CAM_DBG
            printk(KERN_ERR "--CAMERA--CAMERA_BRIGHTNESS_LV1\n");
	     #endif
            for(i = 0;i<6;i++)
            {
                rc = ov5640_i2c_write(ov5640_client->addr,
                        ov5640_brightness_lv1_tbl[i][0],
                        ov5640_brightness_lv1_tbl[i][1],
                        10);
                if (rc < 0)	return rc;     
            }				
            break;
        case CAMERA_BRIGHTNESS_LV2:
	     #ifdef CAM_DBG
            printk(KERN_ERR "--CAMERA--CAMERA_BRIGHTNESS_LV2\n");
	     #endif
            for(i = 0;i<6;i++)
            {
                rc = ov5640_i2c_write(ov5640_client->addr,
                        ov5640_brightness_lv2_default_tbl[i][0],
                        ov5640_brightness_lv2_default_tbl[i][1],
                        10);
                if (rc < 0)	return rc;     
            }				
            break;
        case CAMERA_BRIGHTNESS_LV3:
	     #ifdef CAM_DBG
            printk(KERN_ERR "--CAMERA--CAMERA_BRIGHTNESS_LV3\n");
	     #endif
            for(i = 0;i<6;i++)
            {
                rc = ov5640_i2c_write(ov5640_client->addr,
                        ov5640_brightness_lv3_tbl[i][0],
                        ov5640_brightness_lv3_tbl[i][1],
                        10);
                if (rc < 0)	return rc;     
            }				
            break;
        case CAMERA_BRIGHTNESS_LV4:
	     #ifdef CAM_DBG
            printk(KERN_ERR "--CAMERA--CAMERA_BRIGHTNESS_LV4\n");
	     #endif
            for(i = 0;i<6;i++)
            {
                rc = ov5640_i2c_write(ov5640_client->addr,
                        ov5640_brightness_lv4_tbl[i][0],
                        ov5640_brightness_lv4_tbl[i][1],
                        10);
                if (rc < 0)	return rc;     
            }				
            break;
        default:
            printk(KERN_ERR "--CAMERA--CAMERA_BRIGHTNESS_ERROR COMMAND\n");
            break;
    }
    #ifdef CAM_DBG
    printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
    #endif
    return rc;
}

static int ov5640_set_contrast(int contrast)
{
	long rc = 0;
	int i =0;
	#ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
	#endif
	#ifdef CAM_DBG
    	printk(KERN_ERR "--CAMERA-- %s ...contrast = %d\n",__func__ , contrast);
	#endif

    	if(effect_value == CAMERA_EFFECT_OFF)
	{ 
	switch (contrast)
		{
		    case CAMERA_CONTRAST_LV0:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV0\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_contrast_lv0_tbl[i][0],
					     ov5640_contrast_lv0_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_CONTRAST_LV1:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV1\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_contrast_lv1_tbl[i][0],
					     ov5640_contrast_lv1_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_CONTRAST_LV2:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV2\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_contrast_lv2_tbl[i][0],
					     ov5640_contrast_lv2_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_CONTRAST_LV3:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV3\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_contrast_default_lv3_tbl[i][0],
					     ov5640_contrast_default_lv3_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_CONTRAST_LV4:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV4\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_contrast_lv4_tbl[i][0],
					     ov5640_contrast_lv4_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_CONTRAST_LV5:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV5\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_contrast_lv5_tbl[i][0],
					     ov5640_contrast_lv5_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    default:
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_ERROR COMMAND\n");
				break;
  		}
    	}
	#ifdef CAM_DBG
       printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
	#endif
	return rc;
}

static int ov5640_set_sharpness(int sharpness)
{
	long rc = 0;
	int i =0;
	#ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
	#endif
	#ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...sharpness = %d\n",__func__ , sharpness);
	#endif
	  
    	if(effect_value == CAMERA_EFFECT_OFF)
	{ 
	switch (sharpness)
		{
		    case CAMERA_SHARPNESS_LV0:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_LV0\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_sharpness_lv0_tbl[i][0],
					     ov5640_sharpness_lv0_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SHARPNESS_LV1:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_LV1\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_sharpness_lv1_tbl[i][0],
					     ov5640_sharpness_lv1_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SHARPNESS_LV2:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_LV2\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_sharpness_default_lv2_tbl[i][0],
					     ov5640_sharpness_default_lv2_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SHARPNESS_LV3:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_LV3\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_sharpness_lv3_tbl[i][0],
					     ov5640_sharpness_lv3_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SHARPNESS_LV4:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_LV4\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_sharpness_lv4_tbl[i][0],
					     ov5640_sharpness_lv4_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SHARPNESS_LV5:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_LV5\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_sharpness_lv5_tbl[i][0],
					     ov5640_sharpness_lv5_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    default:
				printk(KERN_ERR "--CAMERA--CAMERA_SHARPNESS_ERROR COMMAND\n");
				break;
  		}
    	}
	#ifdef CAM_DBG
       printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
	#endif
	return rc;
}

static int ov5640_set_saturation(int saturation)
{
	long rc = 0;
	int i =0;
      #ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
      #endif
      #ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...saturation = %d\n",__func__ , saturation);
      #endif
	  
    	if(effect_value == CAMERA_EFFECT_OFF)
	{ 
	switch (saturation)
		{
		    case CAMERA_SATURATION_LV0:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV0\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_saturation_lv0_tbl[i][0],
					     ov5640_saturation_lv0_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SATURATION_LV1:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV1\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_saturation_lv1_tbl[i][0],
					     ov5640_saturation_lv1_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SATURATION_LV2:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV2\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_saturation_lv2_tbl[i][0],
					     ov5640_saturation_lv2_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SATURATION_LV3:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV3\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_saturation_default_lv3_tbl[i][0],
					     ov5640_saturation_default_lv3_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SATURATION_LV4:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV4\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_saturation_lv4_tbl[i][0],
					     ov5640_saturation_lv4_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    case CAMERA_SATURATION_LV5:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_LV5\n");
				#endif
				for(i = 0;i<5;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_saturation_lv5_tbl[i][0],
					     ov5640_saturation_lv5_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}				
				break;
		    default:
				printk(KERN_ERR "--CAMERA--CAMERA_CONTRAST_ERROR COMMAND\n");
				break;
  		}
    	}
	#ifdef CAM_DBG
       printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
	#endif
	return rc;
}

static long ov5640_set_antibanding(int antibanding)
{
	long rc = 0;
	int i =0;
      #ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
	#endif
      #ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...antibanding = %d\n",__func__ , antibanding);
      #endif

	switch (antibanding)
		{
		    case CAMERA_ANTIBANDING_OFF:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_ANTIBANDING_OFF\n");
				#endif
				break;
		    case CAMERA_ANTIBANDING_60HZ:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_ANTIBANDING_60HZ\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_antibanding_60z_tbl[i][0],
					     ov5640_antibanding_60z_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}
		    		break;
		    case CAMERA_ANTIBANDING_50HZ:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_ANTIBANDING_50HZ\n");
				#endif
				for(i = 0;i<2;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_antibanding_50z_tbl[i][0],
					     ov5640_antibanding_50z_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}
		    		break;
		    case CAMERA_ANTIBANDING_AUTO:
				#ifdef CAM_DBG
				printk(KERN_ERR "--CAMERA--CAMERA_ANTIBANDING_AUTO\n");
				#endif
				for(i = 0;i<14;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_antibanding_auto_tbl[i][0],
					     ov5640_antibanding_auto_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}
		    		break;
		    default:
				printk(KERN_ERR "--CAMERA--CAMERA_ANTIBANDING_ERROR COMMAND\n");
				break;
  		}
	#ifdef CAM_DBG
       printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
	#endif
	return rc;
}

static long ov5640_set_exposure_mode(int mode)
{
	long rc = 0;
	//int i =0;
      #ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...(Start)\n",__func__);
      #endif
      #ifdef CAM_DBG
      printk(KERN_ERR "--CAMERA-- %s ...mode = %d\n",__func__ , mode);
      #endif
/*
	switch (mode)
		{
		    case CAMERA_SETAE_AVERAGE:
				printk(KERN_ERR "--CAMERA--CAMERA_SETAE_AVERAGE\n");
				for(i = 0;i<8;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_ae_average_tbl[i][0],
					     ov5640_ae_average_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}
				break;
		    case CAMERA_SETAE_CENWEIGHT:
				printk(KERN_ERR "--CAMERA--CAMERA_SETAE_CENWEIGHT\n");
				for(i = 0;i<8;i++)
				{
					rc = ov5640_i2c_write(ov5640_client->addr,
					     ov5640_ae_centerweight_tbl[i][0],
					     ov5640_ae_centerweight_tbl[i][1],
					     10);
					if (rc < 0)	return rc;     
				}
		    		break;
		    default:
				printk(KERN_ERR "--CAMERA--ERROR COMMAND OR NOT SUPPORT\n");
				break;
  		}
*/
	#ifdef CAM_DBG
       printk(KERN_ERR "--CAMERA-- %s ...(End)\n",__func__);
	#endif
	return rc;
}

static int32_t ov5640_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;
	int i = 0;
	int len;

	len = sizeof(ov5640_lens_shading_on_tbl)/sizeof(ov5640_lens_shading_on_tbl[0]);
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA--%s: ...(Start). enable = %d\n", __func__, is_enable);
	#endif

	if(is_enable)
	{
		#ifdef CAM_DBG
		printk(KERN_ERR "%s: enable~!!\n", __func__);
		#endif
		for(i = 0;i<len;i++)
		{
			rc = ov5640_i2c_write(ov5640_client->addr,
			     					ov5640_lens_shading_on_tbl[i][0],
			     					ov5640_lens_shading_on_tbl[i][1],
			     					10);
			if (rc < 0)	return rc;     
		}
	}
	else
	{
		#ifdef CAM_DBG
		printk(KERN_ERR "%s: disable~!!\n", __func__);
		#endif
		rc = ov5640_i2c_write(ov5640_client->addr,
			     				ov5640_lens_shading_off_tbl[0][0],
			     				ov5640_lens_shading_off_tbl[0][1],
			     				10);
			if (rc < 0)	return rc; 	
	}
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA--%s: ...(End). rc = %d\n", __func__, rc);
	#endif
	return rc;
}

static int ov5640_set_sensor_mode(int mode, int res)
{
	int rc = -EINVAL;
	int len2;
	int len3;
	int len4;
	int i =0;

    uint32_t prv_exp=0;
    uint32_t prv_gain=0;
    uint32_t prv_maxlines=0;
    uint32_t cap_maxlines =0;
    uint32_t cap_exp =0;
    uint32_t cap_gain = 0;
    uint32_t cap_banding_filter = 0;
    unsigned int tmp;
    int8_t tmp2;
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- ov5640_set_sensor_mode mode = %d, res = %d\n", mode, res);
	#endif

	len2 = sizeof(ov5640_capture_tbl_1)/sizeof(ov5640_capture_tbl_1[0]);

	len3 = sizeof(ov5640_preview_tbl_1)/sizeof(ov5640_preview_tbl_1[0]);

	len4 = sizeof(ov5640_afinit_tbl)/sizeof(ov5640_afinit_tbl[0]);
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- len4 = %d\n",len4);
	#endif
	
	switch (mode)
	{
		case SENSOR_PREVIEW_MODE:
		printk(KERN_ERR "--CAMERA-- SENSOR_PREVIEW_MODE\n");

		if(afinit == 1)
		{
			printk(KERN_ERR "--CAMERA-- AF_init~!!\n");

			ov5640_i2c_write(ov5640_client->addr,0x3000,0x20,10);

			rc = ov5640_i2c_txdata(ov5640_client->addr,ov5640_afinit_tbl,len4);

			if (rc < 0)
				{
				printk(KERN_ERR "--CAMERA-- AF_fail~!!\n");
				return rc;
				}

			ov5640_i2c_write(ov5640_client->addr,0x3022,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3023,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3024,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3025,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3026,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3027,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3028,0x00,10);
			ov5640_i2c_write(ov5640_client->addr,0x3029,0x7f,10);
			ov5640_i2c_write(ov5640_client->addr,0x3000,0x00,10);

			afinit = 0;
			
		}
		
      		if(is_autoflash == 1)
              {
                  pmic_flash_led_set_current(0);
              }	
  			for(i = 0;i<len3;i++)
  			{
  					rc = ov5640_i2c_write(ov5640_client->addr,
  					     ov5640_preview_tbl_1[i][0],
  					     ov5640_preview_tbl_1[i][1],
  					     10);
  					if (rc < 0)	return rc;     
  			}
			
            //ov5640_i2c_write(ov5640_client->addr,0x3503, 0x00,10);
			break;
		case SENSOR_SNAPSHOT_MODE:
            printk(KERN_ERR "--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
            ov5640_i2c_read_byte(ov5640_client->addr,0x3503,&tmp);
	     #ifdef CAM_DBG
            printk(KERN_ERR "[kylin] 0x3503: %x\r\n",tmp);
	     #endif

            //ov5640_i2c_write(ov5640_client->addr,0x3503, 0x07,10);
            //msleep(250);
            //get preview exp
/*            ov5640_i2c_read_byte(ov5640_client->addr,0x3500,&tmp);
            prv_exp  = prv_exp << 8;
            prv_exp += (tmp &0x0f );
//            
            ov5640_i2c_read_byte(ov5640_client->addr,0x3501,&tmp);
            prv_exp  = prv_exp << 8;
            prv_exp += tmp ;
//
            ov5640_i2c_read_byte(ov5640_client->addr,0x3502,&tmp);
            prv_exp  = prv_exp << 8;
            prv_exp += tmp ;
            //
            printk(KERN_ERR "[kylin] prv_exp : %d\r\n",prv_exp);
            //get preview gain
            ov5640_i2c_read_byte(ov5640_client->addr,0x350b,&tmp);
            prv_gain = tmp;
            printk(KERN_ERR "[kylin] prv_gain : %d\r\n",prv_gain);
            //get preview maxlines
            ov5640_i2c_read_byte(ov5640_client->addr,0x380e,&tmp);
            printk(KERN_ERR "[kylin] 0x380e: %x\r\n",tmp);
            prv_maxlines += tmp;
            prv_maxlines = prv_maxlines << 8;
            ov5640_i2c_read_byte(ov5640_client->addr,0x380f,&tmp);
            printk(KERN_ERR "[kylin] 0x380f: %x\r\n",tmp);
            //
            prv_maxlines += tmp;
            printk(KERN_ERR "[kylin] prv_maxlines : %d\r\n",prv_maxlines);
            */
            if(is_autoflash == 1){
                ov5640_i2c_read_byte(ov5640_client->addr,0x350b,&tmp2);
		   #ifdef CAM_DBG
                printk(KERN_ERR "--CAMERA-- GAIN VALUE : %x %d\n",tmp2,tmp2);
		   #endif
                if(tmp2 > 0)
                {
                    pmic_flash_led_set_current(0);
                }else{
                    pmic_flash_led_set_current(100);
                }
            }
            for(i = 0;i<len2;i++)
            {
                rc = ov5640_i2c_write(ov5640_client->addr,
                        ov5640_capture_tbl_1[i][0],
                        ov5640_capture_tbl_1[i][1],
                        10);
                if (rc < 0)	return rc;     
            }
            //get capture maxlines
/*            ov5640_i2c_read_byte(ov5640_client->addr,0x380e,&tmp);
            printk(KERN_ERR "[kylin] 0x380e: %x\r\n",tmp);

            cap_maxlines += tmp;
            cap_maxlines = cap_maxlines << 8;
            ov5640_i2c_read_byte(ov5640_client->addr,0x380f,&tmp);
            printk(KERN_ERR "[kylin] 0x380f: %x\r\n",tmp);

            cap_maxlines += tmp;*/
            //printk(KERN_ERR "[kylin] cap_maxlines %d\r\n",cap_maxlines);
            //calc capture exp
//            cap_exp = prv_exp * 375 * cap_maxlines / (1500*prv_maxlines);
            //cap_exp = (prv_exp /4) ;
//            printk(KERN_ERR "[kylin] cap_maxlines %d\r\n",cap_exp);
            // printk(KERN_ERR "[kylin]cpature exp: %d\r\n",cap_exp);
            //capture banding filter
            //todo : check banding 50 hz
            //cap_band_filter = 1.875 * cap_maxlines / 120;    
            //capture gain
            
            /*tmp = (cap_exp & 0x00ff0000) >> 16;
            ov5640_i2c_write(ov5640_client->addr,0x3500, tmp,10);
            tmp = (cap_exp & 0x0000ff00) >> 8;
            ov5640_i2c_write(ov5640_client->addr,0x3501, tmp,10);
            tmp = (cap_exp & 0x000000ff);
            ov5640_i2c_write(ov5640_client->addr,0x3502, tmp,10);

            cap_gain = cap_exp*prv_gain/ cap_maxlines ;
            tmp = cap_gain & 0x0ff;*/
            //ov5640_i2c_write(ov5640_client->addr,0x350a, 0x00,10);
            //ov5640_i2c_write(ov5640_client->addr,0x350b, tmp,10);

            //msleep(250);
			break;
		case SENSOR_RAW_SNAPSHOT_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_RAW_SNAPSHOT_MODE\n");
		//	rc = ov5640_raw_snapshot_config(mode);
			break;
		default:
			printk(KERN_ERR "--CAMERA--ov5640_set_sensor_mode  DEFAULT\n");
			break;
	}

	return 1;
}
static int ov5640_set_wb_oem(uint8_t param)
{
    int rc,i;
    unsigned int tmp=0;
    int8_t tmp2;
    int len = sizeof(ov5640_wb_def)/sizeof(ov5640_wb_def[0]);
    #ifdef CAM_DBG
    printk(KERN_ERR "[kylin] %s \r\n",__func__ );
    #endif
    ov5640_i2c_read_byte(ov5640_client->addr,0x350b,&tmp2);
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- GAIN VALUE : %x\n",tmp2);
	#endif

    if(param == CAMERA_WB_AUTO)
    {
         for(i = 0;i<len;i++)
         {
             rc = ov5640_i2c_write(ov5640_client->addr,
                     ov5640_wb_def[i][0],
                     ov5640_wb_def[i][1],
                     10);
             if (rc < 0) return rc;
         }

    }
    else if(param == CAMERA_WB_CUSTOM)
    {
        ov5640_i2c_read(ov5640_client->addr,0x3406,&tmp);
        tmp = tmp | 0x01;
        ov5640_i2c_write(ov5640_client->addr,0x3406,tmp,10);
        ov5640_i2c_write(ov5640_client->addr,0x3400,0x04,10); //;R Gain
        ov5640_i2c_write(ov5640_client->addr,0x3401,0x00,10);
        ov5640_i2c_write(ov5640_client->addr,0x3402,0x04,10); //;G Gain
        ov5640_i2c_write(ov5640_client->addr,0x3403,0x00,10);
        ov5640_i2c_write(ov5640_client->addr,0x3404,0x04,10); //;B Gain
        ov5640_i2c_write(ov5640_client->addr,0x3405,0x00,10);
    }
    else if(param == CAMERA_WB_INCANDESCENT)
    {
        ov5640_i2c_read(ov5640_client->addr,0x3406,&tmp);
        tmp = tmp | 0x01;
        ov5640_i2c_write(ov5640_client->addr,0x3406,tmp,10);
        ov5640_i2c_write(ov5640_client->addr,0x3400,0x05,10); //;R Gain
        ov5640_i2c_write(ov5640_client->addr,0x3401,0x48,10);
        ov5640_i2c_write(ov5640_client->addr,0x3402,0x04,10); //;G Gain
        ov5640_i2c_write(ov5640_client->addr,0x3403,0x00,10);
        ov5640_i2c_write(ov5640_client->addr,0x3404,0x07,10); //;B Gain
        ov5640_i2c_write(ov5640_client->addr,0x3405,0x0c,10);
    }
    else if(param == CAMERA_WB_DAYLIGHT)
    {
        ov5640_i2c_read(ov5640_client->addr,0x3406,&tmp);
        tmp = tmp | 0x01;
        ov5640_i2c_write(ov5640_client->addr,0x3406,tmp,10);
        ov5640_i2c_write(ov5640_client->addr,0x3400,0x06,10); //;R Gain
        ov5640_i2c_write(ov5640_client->addr,0x3401,0x1c,10);
        ov5640_i2c_write(ov5640_client->addr,0x3402,0x04,10); //;G Gain
        ov5640_i2c_write(ov5640_client->addr,0x3403,0x00,10);
        ov5640_i2c_write(ov5640_client->addr,0x3404,0x04,10); //;B Gain
        ov5640_i2c_write(ov5640_client->addr,0x3405,0xf3,10);
    }
    else if(param == CAMERA_WB_CLOUDY_DAYLIGHT)
    {
        ov5640_i2c_read(ov5640_client->addr,0x3406,&tmp);
        tmp = tmp | 0x01;
        ov5640_i2c_write(ov5640_client->addr,0x3406,tmp,10);
        ov5640_i2c_write(ov5640_client->addr,0x3400,0x06,10); //;R Gain
        ov5640_i2c_write(ov5640_client->addr,0x3401,0x48,10);
        ov5640_i2c_write(ov5640_client->addr,0x3402,0x04,10); //;G Gain
        ov5640_i2c_write(ov5640_client->addr,0x3403,0x00,10);
        ov5640_i2c_write(ov5640_client->addr,0x3404,0x04,10); //;B Gain
        ov5640_i2c_write(ov5640_client->addr,0x3405,0xd3,10);
    }
    return 1;
}
static int ov5640_set_touchaec(uint32_t x,uint32_t y)
{
    uint8_t aec_arr[8]={0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
    int idx = 0;
    int i;
    #ifdef CAM_DBG
    printk(KERN_ERR "[kylin] %s x: %d ,y: %d\r\n",__func__ ,x,y);
    #endif
    idx = x /2 + y *2;
    #ifdef CAM_DBG
    printk(KERN_ERR "[kylin] idx: %d\r\n",idx);
    #endif
    if(x %2 == 0)
    {
        aec_arr[idx] = 0x10 | 0x0a;
    }
    else
    {
        aec_arr[idx] = 0x01 | 0xa0;
    }
    for(i==0;i<8;i++)
    {
    	 #ifdef CAM_DBG
        printk("write : %x val : %x ",0x5688+i,aec_arr[i]);
	 #endif
        
        ov5640_i2c_write(ov5640_client->addr,0x5688+i,aec_arr[i],10);
    }
    return 1;
}
static int ov5640_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long  rc = 0;

	if (copy_from_user(&cdata,(void *)argp,sizeof(struct sensor_cfg_data))) 
		return -EFAULT;
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- %s %d\n",__func__,cdata.cfgtype);
	#endif	
	mutex_lock(&ov5640_mutex);
	switch (cdata.cfgtype)
	{
		case CFG_SET_MODE:   // 0
			rc =ov5640_set_sensor_mode(cdata.mode, cdata.rs);
			break;
		case CFG_SET_EFFECT: // 1
		       #ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_EFFECT mode=%d, effect = %d !!\n",cdata.mode, cdata.cfg.effect);
			#endif
			rc = ov5640_set_effect(cdata.mode, cdata.cfg.effect);
			break;
		case CFG_START:      // 2
			printk(KERN_ERR "--CAMERA-- CFG_START (Not Support) !!\n");
			// Not Support
			break;
		case CFG_PWR_UP:     // 3
			printk(KERN_ERR "--CAMERA-- CFG_PWR_UP (Not Support) !!\n");
			// Not Support
			break;
		case CFG_PWR_DOWN:   // 4
			printk(KERN_ERR "--CAMERA-- CFG_PWR_DOWN (Not Support) \n");
			camera_power_onoff(0);
			break;
		case CFG_SET_DEFAULT_FOCUS:  // 06
			printk(KERN_ERR "--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
			break;		
		case CFG_MOVE_FOCUS:     //  07
			printk(KERN_ERR "--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
			break;
		case CFG_SET_BRIGHTNESS:     //  12
		       #ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_BRIGHTNESS  !!\n");
			#endif
			rc = ov5640_set_brightness(cdata.cfg.brightness);
			break;
		case CFG_SET_CONTRAST:     //  13
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_CONTRAST  !!\n");
			#endif
			rc = ov5640_set_contrast(cdata.cfg.contrast);
			break;			
		case CFG_SET_EXPOSURE_MODE:     //  15
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_EXPOSURE_MODE !!\n");
			#endif
			rc = ov5640_set_exposure_mode(cdata.cfg.ae_mode);
			break;
		case CFG_SET_ANTIBANDING:     //  17
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_ANTIBANDING antibanding = %d!!\n", cdata.cfg.antibanding);
			#endif
			rc = ov5640_set_antibanding(cdata.cfg.antibanding);
			break;
		case CFG_SET_LENS_SHADING:     //  20
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_LENS_SHADING !!\n");
			#endif
			rc = ov5640_lens_shading_enable(cdata.cfg.lens_shading);
			break;
		case CFG_SET_SATURATION:     //  30
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_SATURATION !!\n");
			#endif
			rc = ov5640_set_saturation(cdata.cfg.saturation);
			break;
		case CFG_SET_SHARPNESS:     //  31
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_SHARPNESS !!\n");
			#endif
			rc = ov5640_set_sharpness(cdata.cfg.sharpness);
			break;
        	case CFG_SET_WB:
			#ifdef CAM_DBG
			printk(KERN_ERR "--CAMERA-- CFG_SET_WB!!\n");
			#endif
            		ov5640_set_wb_oem(cdata.cfg.wb_val);
            		rc = 0 ;
            		break;
             case CFG_SET_TOUCHAEC:
			#ifdef CAM_DBG
                 	printk(KERN_ERR "--CAMERA-- CFG_SET_TOUCHAEC!!\n");
			#endif
                 	ov5640_set_touchaec(cdata.cfg.aec_cord.x,cdata.cfg.aec_cord.y);
                 	rc = 0 ;
                 	break;
             case CFG_SET_AUTO_FOCUS:
			#ifdef CAM_DBG
                 	printk(KERN_ERR "--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
			#endif
                 	rc = ov5640_sensor_start_af();
                	// rc = 0 ;
                 	break;
             case CFG_SET_AUTOFLASH:
			#ifdef CAM_DBG
                 	printk(KERN_ERR "--CAMERA-- CFG_SET_AUTO_FLASH !\n");
			#endif
                 	is_autoflash = cdata.cfg.is_autoflash;
			#ifdef CAM_DBG
                 	printk(KERN_ERR "[kylin] is autoflash %d\r\n",is_autoflash);
			#endif
                 	rc = 0;
                 	break;
     		default:
     			printk(KERN_ERR "--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
     			rc = -EINVAL;
     		break;	
	}
	mutex_unlock(&ov5640_mutex);
	return rc;	
}

static struct i2c_driver ov5640_i2c_driver = {
	.id_table = ov5640_i2c_id,
	.probe  = ov5640_i2c_probe,
	.remove = ov5640_i2c_remove,
	.driver = {
		.name = "ov5640",
	},
};

static int ov5640_sensor_start_af(void)
{
    u8 i;
	u8 af_st = 0;
	u8 af_ack = 0;
    u8 tmp = 0;
    int rc = 0;
	printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
//    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
//    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x08,10);
//
//    for(i=0;i<50;i++)
//    {
//        ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
//        if(af_ack == 0)
//            break;
//        msleep(50);
//    }

	
	ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
	ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x03,10);

    for(i=0;i<50;i++)
    {
	    ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
        if(af_ack == 0)
            break;
        msleep(50);
    }
	#ifdef CAM_DBG
	printk(KERN_ERR "--CAMERA-- %s af_ack = 0x%x\n", __func__, af_ack);
	#endif

//	if(af_ack == 0)
	{
//		mdelay(1000);

		ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_FW_STATUS,&af_st);
		#ifdef CAM_DBG
		printk(KERN_ERR "--CAMERA-- %s af_st = %d\n", __func__, af_st);
		#endif

		if(af_st == 0x10)
		{
			printk(KERN_ERR "--CAMERA-- %s AF ok and release AF setting~!!\n", __func__);
		}
		else printk(KERN_ERR "--CAMERA-- %s AF not ready!!\n", __func__);
	}

//  ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
//  ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x08,10);
    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x07,10);
    for(i=0;i<70;i++)
    {
        ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
        if(af_ack == 0)
            break;
        msleep(25);
    }

    ov5640_i2c_read_byte(ov5640_client->addr,0x3024,&tmp);
    #ifdef CAM_DBG
    printk(KERN_ERR "0x3024 = %x \n",tmp);
    #endif
    rc = ((tmp == 0) ? 1:0) ;

    ov5640_i2c_read_byte(ov5640_client->addr,0x3025,&tmp);
    #ifdef CAM_DBG
    printk(KERN_ERR "0x3025 = %x \n",tmp);
    #endif
    rc = ((tmp == 0) ? 1:0) ;

    ov5640_i2c_read_byte(ov5640_client->addr,0x3026,&tmp);
    #ifdef CAM_DBG	
    printk(KERN_ERR "0x3026 = %x \n",tmp);
    #endif
    rc = ((tmp == 0) ? 1:0) ;

    ov5640_i2c_read_byte(ov5640_client->addr,0x3027,&tmp);
    #ifdef CAM_DBG
    printk(KERN_ERR "0x3027 = %x \n",tmp);
    #endif
    rc = ((tmp == 0) ? 1:0) ;

    ov5640_i2c_read_byte(ov5640_client->addr,0x3028,&tmp);
    #ifdef CAM_DBG	
    printk(KERN_ERR "0x3028 = %x \n",tmp);
    #endif
    rc = ((tmp == 0) ? 1:0) ;
    
    printk(KERN_ERR "--CAMERA-- %s rc = %d(End...)\n",__func__,rc);
	return rc;
}

static int ov5640_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
  //camera_init_flag = false;
  #ifdef CAM_DBG
  printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
  #endif
  rc = i2c_add_driver(&ov5640_i2c_driver);
  if ((rc < 0 ) || (ov5640_client == NULL))
  {
  	printk(KERN_ERR "--CAMERA-- i2c_add_driver FAILS!!\n");
   	return rc;
  }
  //printk(KERN_ERR "--CAMERA-- msm_camio_clk_rate_set = 56 MHz\n");
  //msm_camio_clk_rate_set(56000000);  // mclk=56 MHZ
  //printk(KERN_ERR "--CAMERA-- msm_camio_clk_rate_set = 56 MHz ok ...\n");
  //mdelay(200);

  rc = ov5640_probe_init_sensor(info);
  if (rc < 0)
  {
  	printk(KERN_ERR "--CAMERA--ov5640_probe_init_sensor Fail !!~~~~!!\n");
  	return rc;
  }
  s->s_init = ov5640_sensor_open_init;
  s->s_release = ov5640_sensor_release;
  s->s_config  = ov5640_sensor_config;
  //s->s_AF = ov5640_sensor_set_af;
  //camera_init_flag = true;
  #ifdef CAM_DBG
  printk(KERN_ERR "--CAMERA-- %s (End...)\n",__func__);
  #endif
  return rc;
}

static void power_off()
{
   #ifdef CAM_DBG
   printk(KERN_ERR "--CAMERA-- %s ... (Start...)\n",__func__);
   #endif

   camera_power_onoff(0);
   gpio_set_value(23, 1);
   gpio_set_value(89, 1);	
   #ifdef CAM_DBG
   printk(KERN_ERR "--CAMERA-- %s ... (End...)\n",__func__);
   #endif
}

static int ov5640_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
  u32 device_id = 0;
   #ifdef CAM_DBG
   printk(KERN_ERR "--CAMERA-- %s ... (Start...)\n",__func__);
   #endif
   
   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
   {
      printk(KERN_ERR "--CAMERA--i2c_check_functionality failed\n");
      return -ENOMEM;
   }
   
   ov5640_sensorw = kzalloc(sizeof(struct ov5640_work), GFP_KERNEL);
   if (!ov5640_sensorw)
   {
      printk(KERN_ERR "--CAMERA--kzalloc failed\n");
      return -ENOMEM;
   }
   i2c_set_clientdata(client, ov5640_sensorw);
   ov5640_init_client(client);
   ov5640_client = client;

   // (1) set Camera PMIC and power on
#ifdef CONFIG_MACH_MSM7X25_QRD
   vreg_CAM_gp3 = vreg_get(NULL, "gp3");
   vreg_CAM_gp6 = vreg_get(NULL, "gp6");
   vreg_CAM_wlan = vreg_get(NULL, "wlan");   
#else
   vreg_CAM_gp6 = vreg_get(NULL, "gp6");
   vreg_set_level(vreg_CAM_gp6,  2850);
#endif
   // (2) config pwd and rest pin
   //printk(KERN_ERR "--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);
   gpio_request(23, "ov5640");	//GPIO23 <- power down pin
   gpio_request(89, "ov5640");	//GPIO89 <-reset pin
   gpio_direction_output(23, 1); //disable  power down pin
   gpio_direction_output(89, 1); //disable reset pin

   // (3) Set Clock = 24 MHz
   msm_camio_clk_rate_set(24000000);
   //mdelay(5);

   // (4) Power On
   camera_power_onoff(1);
   gpio_set_value(23, 0);	//enable power down pin 
   gpio_set_value(89, 1);	//reset camera reset pin
   msleep(5);
   gpio_set_value(89, 0);
   msleep(5);
   gpio_set_value(89, 1);
   msleep(1);
   
   // (6) Read Device ID
   ov5640_i2c_read(ov5640_client->addr, 0x300A, &device_id);	//0x300A ,sensor ID register

    if(device_id != 0x5640)	//0x5640, ov5640 chip id
   {
	printk(KERN_ERR "--CAMERA-- %s ok , device id error %d\r\n",__func__ , device_id);
	power_off();
	return -1;
   }
   
   printk(KERN_ERR "--CAMERA-- %s ok , device id=0x%x\n",__func__,device_id);

   power_off();
   #ifdef CAM_DBG
   printk("--CAMERA-- %s ... (End...)\n",__func__);
   #endif
   return 0;
}

static int __ov5640_probe(struct platform_device *pdev)
{
   return msm_camera_drv_start(pdev, ov5640_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __ov5640_probe,
    .driver = {
        .name = "msm_camera_ov5640",
        .owner = THIS_MODULE,
    },
};

static int __init ov5640_init(void)
{
	//memset(ov5640_i2c_buf,0,sizeof(ov5640_i2c_buf));
	ov5640_i2c_buf[0]=0x5A;
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov5640_init);
