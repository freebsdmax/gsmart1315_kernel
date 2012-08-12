#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "s5k4eagx.h"
#include <mach/vreg.h>

//Camera Power
static struct vreg *vreg_CAM_gp3;      //power 1.5v
static struct vreg *vreg_CAM_gp6;      //power 2.8v
static struct vreg *vreg_CAM_wlan;     //power 2.8v

//Camera init flag for check if it is inited.
static bool camera_init_flag;

struct s5k4eagx_work {
   struct work_struct work;
};
static struct s5k4eagx_work *s5k4eagx_sensorw;
static struct i2c_client    *s5k4eagx_client;
static DECLARE_WAIT_QUEUE_HEAD(s5k4eagx_wait_queue);
DEFINE_MUTEX(s5k4eagx_mutex);
static u8 s5k4eagx_i2c_buf[4];
static u8 s5k4eagx_counter = 0;

struct __s5k4eagx_ctrl 
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
static struct __s5k4eagx_ctrl *s5k4eagx_ctrl;
//static u32 exp_gain_tbl[3][2];

static int s5k4eagx_i2c_remove(struct i2c_client *client);
static int s5k4eagx_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);

static int s5k4eagx_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(s5k4eagx_client->adapter, msg, 1) < 0)	return -EIO;
	else return 0;
}

static int s5k4eagx_i2c_write(unsigned short saddr, unsigned int waddr,
	unsigned short bdata,u8 trytimes)
{
   int rc = -EIO;
   s5k4eagx_counter = 0;
   s5k4eagx_i2c_buf[0] = (waddr & 0xFF00)>>8;
   s5k4eagx_i2c_buf[1] = (waddr & 0x00FF);
   s5k4eagx_i2c_buf[2] = (bdata & 0xFF00)>>8;
   s5k4eagx_i2c_buf[3] = (bdata & 0x00FF);
   
   while ( (s5k4eagx_counter<trytimes) &&(rc != 0) )
   {
      rc = s5k4eagx_i2c_txdata(saddr, s5k4eagx_i2c_buf, 4);
      if (rc < 0)
      {
      	s5k4eagx_counter++;
      	printk(KERN_ERR "***s5k4eagx i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,s5k4eagx_counter,rc);
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
static int s5k4eagx_i2c_write_table(u32 p[][2],u8 trytimes)
{
	int i=0,rc=0;
	for(;i<(sizeof(p)/sizeof(p[0])/2);i++)
	{
		rc = s5k4eagx_i2c_write(s5k4eagx_client->addr,p[i][0],p[i][1],trytimes);
		if (rc < 0) return rc;
	}
	return rc;
}
*/
static int s5k4eagx_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(s5k4eagx_client->adapter, msgs, 2) < 0) {
		CDBG("s5k4eagx_i2c_rxdata failed!\n");
		printk(KERN_ERR "s5k4eagx_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4eagx_i2c_read(unsigned short   saddr,
	unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	printk(KERN_ERR "+s5k4eagx_i2c_read\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k4eagx_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)	CDBG("s5k4eagx_i2c_read failed!\n");

	printk(KERN_ERR "-s5k4eagx_i2c_read\n");
	return rc;
}



static void camera_power_onoff(u8 v)
{
	if (v==1)
	{
		printk(KERN_ERR "--CAMERA-- power on~!!\n");
		vreg_enable(vreg_CAM_wlan);
		vreg_enable(vreg_CAM_gp3);
		vreg_enable(vreg_CAM_gp6);
		msleep(5);
	}else
	{
		printk(KERN_ERR "--CAMERA-- power off~!!\n");
		vreg_disable(vreg_CAM_gp6);
		vreg_disable(vreg_CAM_gp3);
		vreg_disable(vreg_CAM_wlan);
	}
}

static int s5k4eagx_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	u32 device_id=0;
   printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
   
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
   printk(KERN_ERR "--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);
   //gpio_request(data->sensor_pwd, "s5k4eagx");
   //gpio_request(data->sensor_reset, "s5k4eagx");
   //gpio_direction_output(data->sensor_pwd, 0);
   //gpio_direction_output(data->sensor_reset, 0);

   // (3) Set Clock = 48 MHz
   //msm_camio_clk_rate_set(24000000);
   //mdelay(5);
   //msm_camio_camif_pad_reg_reset();
   //mdelay(5);

   // (4) Power On
   //camera_power_onoff(1);
   //gpio_set_value(data->sensor_pwd, 1);
   //mdelay(1);
   //gpio_set_value(data->sensor_reset, 1);
  // mdelay(5);
   camera_init_flag=true;
   // (5) Into Stand by mode
   //s5k4eagx_i2c_write(s5k4eagx_client->addr,REG_MODE_SELECT,MODE_SELECT_SW_STANDBY,10);
   
   // (6) Read Device ID
   //s5k4eagx_i2c_read(s5k4eagx_client->addr, 0x0156, &device_id);
   
   printk(KERN_ERR "--CAMERA-- %s ok , device id=0x%x\n",__func__,device_id);
   return 0;
}

static int s5k4eagx_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	//u16 num_lperf;
	int len;
	int i = 0;
	printk(KERN_ERR "--CAMERA-- %s (Start...), rupdate=%d \n",__func__,rupdate);

	len = sizeof(preview_s5k4eagx_reg)/sizeof(preview_s5k4eagx_reg[0]);
	
	switch (rupdate)
	{
		case S_UPDATE_PERIODIC:
			break; /* UPDATE_PERIODIC */
		case S_REG_INIT:
			printk(KERN_ERR "--CAMERA-- S_REG_INIT (Start)\n");
			//rc = s5k4eagx_i2c_write_table(preview_s5k4e1g_reg,10);

			//set register setting
			for(i = 0;i<len;i++)
			{
			   rc = s5k4eagx_i2c_write(s5k4eagx_client->addr, \
			        preview_s5k4eagx_reg[i][0], preview_s5k4eagx_reg[i][1],10);
				msleep(1);
			   //mdelay(20);	
			}	

			/* reset fps_divider */
			s5k4eagx_ctrl->fps_divider = 1 * 0x0400;
			printk(KERN_ERR "--CAMERA-- S_REG_INIT (End)\n");
			break; /* case REG_INIT: */
		default:
			break;
	} /* switch (rupdate) */

	printk(KERN_ERR "--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);

	return rc;
}

static int s5k4eagx_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = -ENOMEM;
	printk(KERN_ERR "--CAMERA-- %s\n",__func__);

	s5k4eagx_ctrl = kzalloc(sizeof(struct __s5k4eagx_ctrl), GFP_KERNEL);
	if (!s5k4eagx_ctrl)
	{
		printk(KERN_ERR "--CAMERA-- kzalloc s5k4eagx_ctrl error !!\n");
		kfree(s5k4eagx_ctrl);
		return rc;
	}
	s5k4eagx_ctrl->fps_divider = 1 * 0x00000400;
	s5k4eagx_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k4eagx_ctrl->set_test = S_TEST_OFF;
	s5k4eagx_ctrl->prev_res = S_QTR_SIZE;
	s5k4eagx_ctrl->pict_res = S_FULL_SIZE;
	
	if (data) s5k4eagx_ctrl->sensordata = data;

	/* enable mclk = 50 MHz first */
	msm_camio_clk_rate_set(24000000);
	//mdelay(20);

	msm_camio_camif_pad_reg_reset();
	//mdelay(20);

	camera_power_onoff(1);
	gpio_set_value(data->sensor_reset, 1);
	mdelay(1);
	gpio_set_value(data->sensor_pwd, 1);
	mdelay(1);
	
	if (s5k4eagx_ctrl->prev_res == S_QTR_SIZE)
		rc = s5k4eagx_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = s5k4eagx_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		printk(KERN_ERR "--CAMERA-- %s : s5k4eagx_setting failed. rc = %d\n",__func__,rc);
		kfree(s5k4eagx_ctrl);
		return rc;
	}

	printk(KERN_ERR "--CAMERA--re_init_sensor ok!!\n");
	return rc;
}

static int s5k4eagx_sensor_release(void)
{
	printk(KERN_ERR "--CAMERA--s5k4eagx_sensor_release!!\n");

	mutex_lock(&s5k4eagx_mutex);

	camera_power_onoff(0);
	
	gpio_set_value(s5k4eagx_ctrl->sensordata->sensor_reset, 0);
	
	gpio_free(s5k4eagx_ctrl->sensordata->sensor_reset);
	gpio_free(s5k4eagx_ctrl->sensordata->sensor_pwd);

	kfree(s5k4eagx_ctrl);
	s5k4eagx_ctrl = NULL;

	mutex_unlock(&s5k4eagx_mutex);
	return 0;
}


			       
static const struct i2c_device_id s5k4eagx_i2c_id[] = {
	{"s5k4eagx", 0},{}
//	{"qrd_camera", 0},{}
};

static int s5k4eagx_i2c_remove(struct i2c_client *client)
{
   return 0;
}


static int s5k4eagx_init_client(struct i2c_client *client)
{
   /* Initialize the MSM_CAMI2C Chip */
   init_waitqueue_head(&s5k4eagx_wait_queue);
   return 0;
}
#if 0
static int s5k4eagx_write_exp_gain(uint16_t gain, uint32_t line)
{
	int rc = 0;
	u16 max_legal_gain = 0x0200;
	u32 ll_ratio; /* Q10 */
	u32 ll_pck, fl_lines;
	u16 offset = 4;
	u32 gain_msb, gain_lsb;
	u32 intg_t_msb, intg_t_lsb;
	u32 ll_pck_msb, ll_pck_lsb;
	
	if (s5k4eagx_ctrl->sensormode == SENSOR_PREVIEW_MODE)
	{

		s5k4eagx_ctrl->my_reg_gain = gain;
		s5k4eagx_ctrl->my_reg_line_count = (uint16_t)line;
		fl_lines = preview_size_h + preview_blk_l;  // 980+12
		ll_pck   = preview_size_w + preview_blk_p; // 1304+1434
	}
	else
	{
		fl_lines = capture_size_h + capture_blk_l; // 1960+12
		ll_pck   = capture_size_w + capture_blk_p; // 2608+130
	}

	if (gain > max_legal_gain) gain = max_legal_gain;

	/* in Q10 */
	line = (line * s5k4eagx_ctrl->fps_divider);

	if (fl_lines < (line / 0x400)) ll_ratio = (line / (fl_lines - offset));
	else ll_ratio = 0x400;

	/* update gain registers */
	gain_msb = (gain & 0xFF00) >> 8;
	gain_lsb = gain & 0x00FF;
	exp_gain_tbl[0][0] = S5K4EAGX_REG_GROUP_PARAMETER_HOLD;
	exp_gain_tbl[0][1] = S5K4EAGX_GROUP_PARAMETER_HOLD;
	exp_gain_tbl[1][0] = REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB;
	exp_gain_tbl[1][1] = gain_msb;
	exp_gain_tbl[2][0] = REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB;
	exp_gain_tbl[2][1] = gain_lsb;
	
	rc = s5k4eagx_i2c_write_table(exp_gain_tbl,10);
	if (rc < 0)	return rc;     


	ll_pck = ll_pck * ll_ratio;
	ll_pck_msb = ((ll_pck / 0x400) & 0xFF00) >> 8;
	ll_pck_lsb = (ll_pck / 0x400) & 0x00FF;
	exp_gain_tbl[0][0] = REG_LINE_LENGTH_PCK_MSB;
	exp_gain_tbl[0][1] = ll_pck_msb;
	exp_gain_tbl[1][0] = REG_LINE_LENGTH_PCK_LSB;
	exp_gain_tbl[1][1] = ll_pck_lsb;

	rc = s5k4eagx_i2c_write_table(exp_gain_tbl,10);
	if (rc < 0)	return rc;

	line = line / ll_ratio;
	intg_t_msb = (line & 0xFF00) >> 8;
	intg_t_lsb = (line & 0x00FF);
	exp_gain_tbl[0][0] = REG_COARSE_INTEGRATION_TIME;
	exp_gain_tbl[0][1] = intg_t_msb;
	exp_gain_tbl[1][0] = REG_COARSE_INTEGRATION_TIME_LSB;
	exp_gain_tbl[1][1] = intg_t_lsb;
	exp_gain_tbl[2][0] = S5K4EAGX_REG_GROUP_PARAMETER_HOLD;
	exp_gain_tbl[2][1] = S5K4EAGX_GROUP_PARAMETER_UNHOLD;

	rc = s5k4eagx_i2c_write_table(exp_gain_tbl,10);

	return rc;
}
#endif
/*
static int s5k4eagx_video_config(int mode, int res)
{
	int rc = 0;
	printk(KERN_ERR "--CAMERA--s5k4eagx_video_config  res = %d!!\n", res);
	switch (res)
	{
		case S_QTR_SIZE:
			printk(KERN_ERR "--CAMERA--%s S_RES_PREVIEW!!\n",__func__);
			rc = s5k4eagx_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
			break;
		
		case S_FULL_SIZE:
			printk(KERN_ERR "--CAMERA--%s S_RES_CAPTURE!!\n",__func__);
			rc = s5k4eagx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
			break;
		default:
			printk(KERN_ERR "--CAMERA--s5k4eagx_video_config DEFAULT!!\n");
			return 0;
	} 
	if (rc < 0) return rc;

	s5k4eagx_ctrl->prev_res = res;
	s5k4eagx_ctrl->curr_res = res;
	s5k4eagx_ctrl->sensormode = mode;

	//rc = s5k4eagx_write_exp_gain(s5k4eagx_ctrl->my_reg_gain, \
	 //    s5k4eagx_ctrl->my_reg_line_count);

	return rc;
}
*/
/*
static int s5k4eagx_snapshot_config(int mode)
{
	int rc;
	rc = s5k4eagx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)	return rc;

	s5k4eagx_ctrl->curr_res = s5k4eagx_ctrl->pict_res;
	s5k4eagx_ctrl->sensormode = mode;
	return rc;
}
*/
static int s5k4eagx_raw_snapshot_config(int mode)
{
	int rc;
	rc = s5k4eagx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)	return rc;
	s5k4eagx_ctrl->curr_res = s5k4eagx_ctrl->pict_res;
	s5k4eagx_ctrl->sensormode = mode;
	return rc;
}

static int s5k4eagx_set_sensor_mode(int mode, int res)
{
	int rc = -EINVAL;
	int len2;
	int len3;
	//int i =0;

	printk(KERN_ERR "--CAMERA-- s5k4eagx_set_sensor_mode mode = %d, res = %d\n", mode, res);

	len2 = sizeof(s5k4eagx_capture_tbl_1)/sizeof(s5k4eagx_capture_tbl_1[0]);

	len3 = sizeof(s5k4eagx_preview_tbl_1)/sizeof(s5k4eagx_preview_tbl_1[0]);
	
	switch (mode)
	{
		case SENSOR_PREVIEW_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_PREVIEW_MODE\n");
			/*
			for(i = 0;i<len3;i++)
			{
					rc = s5k4eagx_i2c_write(s5k4eagx_client->addr,
					     preview_tbl_1[i][0],
					     preview_tbl_1[i][1],
					     10);
					if (rc < 0)	return rc;     
			}
			*/
			break;
		case SENSOR_SNAPSHOT_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
				/*
				for(i = 0;i<len2;i++)
				{
					rc = s5k4eagx_i2c_write(s5k4eagx_client->addr,
					     capture_tbl_1[i][0],
					     capture_tbl_1[i][1],
					     10);
					if (rc < 0)	return rc;     
				}
				*/
			break;
		case SENSOR_RAW_SNAPSHOT_MODE:
			printk(KERN_ERR "--CAMERA-- SENSOR_RAW_SNAPSHOT_MODE\n");
			rc = s5k4eagx_raw_snapshot_config(mode);
			break;
		default:
			printk(KERN_ERR "--CAMERA--s5k4eagx_set_sensor_mode  DEFAULT\n");
			break;
	}
	return 1;
}
#if 0
static void s5k4eagx_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	
	u32 d1 = (u32)(
	               ((preview_size_h +	preview_blk_l) *	0x400) / 
	               (capture_size_h +	capture_blk_l));
	u32 d2 = (u32)(
	               ((preview_size_w +	preview_blk_p) *	0x400) / 
	               (capture_size_w +	capture_blk_p));
	u32 divider = (uint32_t) (d1 * d2) / 0x00000400;
	
	/* Verify PCLK settings and frame sizes. */
	*pfps = (u16)(fps * divider / 0x00000400);
}

static int32_t s5k4eagx_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;

	s5k4eagx_ctrl->fps_divider = fps->fps_div;
	rc = s5k4eagx_i2c_write(s5k4eagx_client->addr,
		REG_FRAME_LENGTH_LINES_MSB,
		(((preview_size_h + preview_blk_l) *
			s5k4eagx_ctrl->fps_divider / 0x400) & 0xFF00) >> 8,10);
	if (rc < 0) return rc;

	rc = s5k4eagx_i2c_write(s5k4eagx_client->addr,
		REG_FRAME_LENGTH_LINES_LSB,
		(((preview_size_h +
			preview_blk_l) *
			s5k4eagx_ctrl->fps_divider / 0x400) & 0x00FF),10);
	return rc;
}
#endif
static int s5k4eagx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long  rc = 0;
	if (copy_from_user(&cdata,(void *)argp,sizeof(struct sensor_cfg_data))) 
		return -EFAULT;

	printk(KERN_ERR "--CAMERA-- %s %d\n",__func__,cdata.cfgtype);
		
	mutex_lock(&s5k4eagx_mutex);
	switch (cdata.cfgtype)
	{
		case CFG_SET_MODE:   // 0
			rc =s5k4eagx_set_sensor_mode(cdata.mode, cdata.rs);
			break;
		case CFG_SET_EFFECT: // 1
			printk(KERN_ERR "--CAMERA-- CFG_SET_EFFECT (Not Support) !!\n");
			// Not Support
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
			//camera_power_onoff(0);
			break;
		case CFG_SET_DEFAULT_FOCUS:  // 06
			printk(KERN_ERR "--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
			break;		
		case CFG_MOVE_FOCUS:     //  07
			printk(KERN_ERR "--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
			break;

		default:
			printk(KERN_ERR "--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
			rc = -EINVAL;
		break;	
	}
	mutex_unlock(&s5k4eagx_mutex);
	return rc;	
}

static struct i2c_driver s5k4eagx_i2c_driver = {
	.id_table = s5k4eagx_i2c_id,
	.probe  = s5k4eagx_i2c_probe,
	.remove = s5k4eagx_i2c_remove,
	.driver = {
		.name = "s5k4eagx",
//		.name = "qrd_camera",
	},
};


static int s5k4eagx_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
  //camera_init_flag = false;
  printk(KERN_ERR "--CAMERA-- %s (Start...)\n",__func__);
  
  rc = i2c_add_driver(&s5k4eagx_i2c_driver);
  if ((rc < 0 ) || (s5k4eagx_client == NULL))
  {
  	//i2c_del_driver(&s5k4eagx_i2c_driver);
  	printk(KERN_ERR "--CAMERA-- i2c_add_driver FAILS!!\n");
   	return rc;
  }

  rc = s5k4eagx_probe_init_sensor(info);
  if (rc < 0)
  {
  	printk(KERN_ERR "--CAMERA--s5k4eagx_probe_init_sensor Fail !!~~~~!!\n");
  	return rc;
  }
  s->s_init = s5k4eagx_sensor_open_init;
  s->s_release = s5k4eagx_sensor_release;
  s->s_config  = s5k4eagx_sensor_config;
  
  //camera_init_flag = true;
  printk(KERN_ERR "--CAMERA-- %s (End...)\n",__func__);
  return rc;
}

static void power_off()
{
   printk(KERN_ERR "--CAMERA-- %s ... (Start...)\n",__func__);

   camera_power_onoff(0);
   gpio_set_value(23, 0);
   gpio_set_value(89, 0);	

   printk(KERN_ERR "--CAMERA-- %s ... (End...)\n",__func__);
}

static int s5k4eagx_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
   u32 device_id=0;
   
   printk(KERN_ERR "--CAMERA-- %s ... (Start...)\n",__func__);
   
   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
   {
      printk(KERN_ERR "--CAMERA--i2c_check_functionality failed\n");
      return -ENOMEM;
   }
   
   s5k4eagx_sensorw = kzalloc(sizeof(struct s5k4eagx_work), GFP_KERNEL);
   if (!s5k4eagx_sensorw)
   {
      printk(KERN_ERR "--CAMERA--kzalloc failed\n");
      return -ENOMEM;
   }
   i2c_set_clientdata(client, s5k4eagx_sensorw);
   s5k4eagx_init_client(client);
   s5k4eagx_client = client;

    //power setting
   vreg_CAM_gp3 = vreg_get(NULL, "gp3");
   vreg_CAM_gp6 = vreg_get(NULL, "gp6");
   vreg_CAM_wlan = vreg_get(NULL, "wlan");

   // (2) config pwd and rest pin
   //printk(KERN_ERR "--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);
   gpio_request(23, "s5k4eagx");
   gpio_request(89, "s5k4eagx");
   gpio_direction_output(23, 0);
   gpio_direction_output(89, 0);

   // (3) Set Clock = 24 MHz
   msm_camio_clk_rate_set(24000000);
   //mdelay(5);

   // (4) Power On
   camera_power_onoff(1);
   gpio_set_value(23, 1);
   mdelay(1);
   gpio_set_value(89, 1);
   mdelay(5);;
   
   // (6) Read Device ID
   s5k4eagx_i2c_write(s5k4eagx_client->addr,0x002c,0x7000,10);
   s5k4eagx_i2c_write(s5k4eagx_client->addr,0x002e,0x01e4,10);
   s5k4eagx_i2c_read(s5k4eagx_client->addr, 0x0f12, &device_id);

   if(device_id != 0x04ea)
   {
	printk(KERN_ERR "--CAMERA-- %s ok , device id error\r\n",__func__);
	power_off();
	return -1;
   }
   else printk(KERN_ERR "--CAMERA-- %s ok , device id=0x%x\n",__func__,device_id);

   power_off();
   
   printk("--CAMERA-- %s ... (End...)\n",__func__);
   return 0;
}

static int __s5k4eagx_probe(struct platform_device *pdev)
{
   return msm_camera_drv_start(pdev, s5k4eagx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
  .probe = __s5k4eagx_probe,
	.driver = {
		.name = "msm_camera_s5k4eagx",
		.owner = THIS_MODULE,
		},
};

static int __init s5k4eagx_init(void)
{
	//memset(s5k4eagx_i2c_buf,0,sizeof(s5k4eagx_i2c_buf));
	s5k4eagx_i2c_buf[0]=0x5A;
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k4eagx_init);
