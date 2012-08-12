#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <mach/vreg.h>
#include <linux/firmware.h>
#include <linux/kthread.h>

#define CTP_NAME		"FT5202"
#define	TOUCHPANEL_GPIO_INT	18
#define TOUCHPANEL_GPIO_RST	76
#define GPIO_HIGH_VALUE		1
#define GPIO_LOW_VALUE		0
#ifdef CONFIG_MACH_MSM7X25_QRD
#define TOUCH_X			480
#define TOUCH_Y			720
#define POWER_ON		1
#define POWER_OFF		0
#define FIRMWARE_NAME		"touch_ft5202_fm_25.bin" 
#else
#define TOUCH_X			320
#define TOUCH_Y			480
#define FIRMWARE_NAME		"touch_ft5202_fm.bin"
#endif
/* == Main struct data == */
struct ft5202_data {
#ifdef CONFIG_MACH_MSM7X25_QRD
        uint8_t				status[5];
#endif
         uint8_t      ctp_data[27];
	struct i2c_client		*client;
	struct i2c_msg			i2cmsg[2];
	struct input_dev		*device;
	struct workqueue_struct		*queue;
	struct work_struct		work;
	struct early_suspend		suspend_ptr;
	struct task_struct		*update_firmware_tsk;
};

/* == Functions Prototype == */
static int ft5202_probe(struct i2c_client *, const struct i2c_device_id *);
static irqreturn_t ft5202_irq(int, void *);
static int update_fw_func(void *);
static int ecc_maker(struct i2c_client *, uint8_t, uint8_t *, uint16_t);
static void unregister_process(uint8_t, struct ft5202_data *);
static void ft5202_resume(struct early_suspend *);
static void ft5202_suspend(struct early_suspend *);

/* == Interrupt == */
static irqreturn_t ft5202_irq(int irq, void *handle)
{
	struct ft5202_data *ptr = (struct ft5202_data *)handle;
	queue_work(ptr->queue, &ptr->work);
	return IRQ_HANDLED;
}

/* == Working Function == */
static void ctp_work_function(struct work_struct *work_ptr)
{
	uint8_t		index = 0, base = 0, event = 0;
	uint16_t	x, y;
	struct ft5202_data *ptr = container_of( \
					work_ptr, struct ft5202_data, work);

	/* == Read CTP DATA via I2C == */
	if (i2c_transfer(ptr->client->adapter, ptr->i2cmsg, 2) < 0) {
		dev_err(&ptr->client->dev, \
			"[Work Queue] Read Touchpanel Data ERROR!\n");
		return;
	}

	/* ==  == */
	for (index = 0; index < 5; index++) {
		base = (index << 2) + 6;

		if ((ptr->ctp_data[base + 2] >> 4) > 4)
			continue;

		x = (((ptr->ctp_data[base] & 0x0f) << 8) \
			+ ptr->ctp_data[base + 1]);
		y = (((ptr->ctp_data[base + 2] & 0x0f) << 8) \
			+ ptr->ctp_data[base + 3]);
		event = ptr->ctp_data[base] >> 6;

		if (event != 1)
			input_report_abs(ptr->device, ABS_MT_TOUCH_MAJOR, 1);
		else
			input_report_abs(ptr->device, ABS_MT_TOUCH_MAJOR, 0);
  
  #ifdef CONFIG_MACH_MSM7X25_QRD
		input_report_abs(ptr->device, ABS_MT_POSITION_X, x);
		input_report_abs(ptr->device, ABS_MT_POSITION_Y, y);
  #else
    input_report_abs(ptr->device, ABS_MT_POSITION_X, TOUCH_X - x);
	  input_report_abs(ptr->device, ABS_MT_POSITION_Y, TOUCH_Y - y);
  #endif
	
		input_mt_sync(ptr->device);
	}

	input_sync(ptr->device);
}

/* == Probe Function == */
static int ft5202_probe(struct i2c_client *client, \
			const struct i2c_device_id *id)
{
	struct ft5202_data	*ptr = NULL;

#ifdef CONFIG_MACH_MSM7X27_QRD
  uint32_t   ctp_gpio_table[] = {
		  GPIO_CFG(TOUCHPANEL_GPIO_INT, 0, \
			GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
		  GPIO_CFG(TOUCHPANEL_GPIO_RST, 0, \
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	};
	/* == Initize GPIO == */
	gpio_tlmm_config(ctp_gpio_table[0], GPIO_ENABLE);
	gpio_tlmm_config(ctp_gpio_table[1], GPIO_ENABLE);
	gpio_set_value(TOUCHPANEL_GPIO_RST, GPIO_HIGH_VALUE);
#endif  
  
  
	/* == Check I2C Functions == */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "[Probe] Need I2C_FUNC_I2C functions \n");
		return -ENODEV;
	}

	/* == Allocate pointer == */
	ptr = kmalloc(sizeof(struct ft5202_data), GFP_KERNEL);
	if (!ptr) {
		dev_err(&client->dev, "[Probe] MEM allocate ERROR! \n");
		return -ENOMEM;
	}
	ptr->client = client;

	/* == Create and register input device == */
	ptr->device = input_allocate_device();
	if (!ptr->device) {
		dev_err(&ptr->client->dev, \
			"[Probe] Failed to allocate input Touch device\n");
		unregister_process(0, ptr);
		return -ENOMEM;
	}
	ptr->device->name = "FT5202_Touchscreen_Touch";
	set_bit(EV_SYN, ptr->device->evbit);
#ifdef CONFIG_MACH_MSM7X25_QRD
	set_bit(EV_KEY, ptr->device->evbit);
#endif
  set_bit(EV_ABS, ptr->device->evbit);

                        
  input_set_abs_params(ptr->device, ABS_MT_POSITION_X, 0, TOUCH_X, 0, 0);
#ifdef CONFIG_MACH_MSM7X25_QRD
  input_set_abs_params(ptr->device, ABS_MT_POSITION_Y, 0, TOUCH_Y, 0, 0);
#else
  input_set_abs_params(ptr->device, ABS_MT_POSITION_Y, 0, TOUCH_Y - 40, 0, 0);
#endif

	input_set_abs_params(ptr->device, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	if (input_register_device(ptr->device)) {
		dev_err(&ptr->client->dev, \
			"[Probe] Can't to register Touch device\n");
		unregister_process(1, ptr);
		return -EINVAL;
	}

	/* == Create work queue == */
	ptr->queue = create_singlethread_workqueue("FT5202_Queue");
	if (!ptr->queue) {
		dev_err(&client->dev, "[Probe] Create work queue ERROR! \n");
		unregister_process(2, ptr);
		return -EIO;
	}
	INIT_WORK(&ptr->work, ctp_work_function);

	/* == Create Kernel Threading == */
	ptr->update_firmware_tsk = kthread_create( \
		update_fw_func, ptr, "update_firmware");
	if (IS_ERR(ptr->update_firmware_tsk)) {
		PTR_ERR(ptr->update_firmware_tsk);
		unregister_process(3, ptr);
		return -EINVAL;
	}
	wake_up_process(ptr->update_firmware_tsk);

	return 0;
}

/* == Unregissger Process function == */
static void unregister_process(uint8_t flag, struct ft5202_data *ptr)
{
	switch (flag) {
	case 6:
		unregister_early_suspend(&ptr->suspend_ptr);
	case 5:
		free_irq(ptr->client->irq, &ptr->client->dev);
	case 4:
		gpio_free(TOUCHPANEL_GPIO_INT);
	case 3:
		destroy_workqueue(ptr->queue);
	case 2:
		input_unregister_device(ptr->device);
	case 1:
		input_free_device(ptr->device);
	case 0:
		kfree(ptr);
	};
}

/* == Resume function == */
static void ft5202_suspend(struct early_suspend *h)
{
	uint8_t			cmd[4] = {0xfc, 0x3a, 0x03, 0xc5};
	struct i2c_msg		i2cmsg;
	struct ft5202_data *ptr = container_of( \
					h, struct ft5202_data, suspend_ptr);

	cancel_work_sync(&ptr->work);
	i2cmsg.addr	= ptr->client->addr;
	i2cmsg.flags	= 0;
	i2cmsg.len	= 4;
	i2cmsg.buf	= cmd;

	if (i2c_transfer(ptr->client->adapter, &i2cmsg, 1) <= 0) {
		dev_err(&ptr->client->dev, "Change sleep mode ERROR!\n");
		return;
	}
	dev_err(&ptr->client->dev, "FT5202 is sleeping!\n");
	disable_irq(ptr->client->irq);
}

/* == Suspend function == */
static void ft5202_resume(struct early_suspend *h)
{
	uint32_t	ctp_gpio_table[] = {
		GPIO_CFG(TOUCHPANEL_GPIO_RST, \
		0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	};
	struct ft5202_data *ptr = container_of( \
					h, struct ft5202_data, suspend_ptr);

	gpio_tlmm_config(ctp_gpio_table[0], GPIO_ENABLE);
	gpio_set_value(TOUCHPANEL_GPIO_RST, GPIO_LOW_VALUE);
	msleep(5);
	gpio_set_value(TOUCHPANEL_GPIO_RST, GPIO_HIGH_VALUE);
	dev_err(&ptr->client->dev, "FT5202 wakes up!\n");
	enable_irq(ptr->client->irq);
}

/* == Regist Device Data Struct == */
static const struct i2c_device_id ft5202_id[] = {
	{ CTP_NAME, 0 },
	{ }
};
static struct i2c_driver ft5202_d = {
	.probe		= ft5202_probe,
	.id_table	= ft5202_id,
	.driver		= {.name = CTP_NAME,},
};

/* == Initize Function == */
static int __init ft5202_init(void)
{
	return i2c_add_driver(&ft5202_d);
}

/* == Exit Function == */
static void __exit ft5202_exit(void)
{
	i2c_del_driver(&ft5202_d);
}

module_init(ft5202_init);
module_exit(ft5202_exit);
MODULE_DESCRIPTION("FT5202 Touchscreen Driver");
MODULE_LICENSE("GPLv2");

/* == Comput ECC Code == */
static int ecc_maker(struct i2c_client *client,
			  uint8_t sn,
			  uint8_t *buf,
			  uint16_t len)
{
	uint16_t		crc = 0, size = len;
	uint8_t			buffer[133] = {0}, *ptr = buf;
	struct i2c_msg		i2cmsg;

	while (size--) {
		uint8_t		i = 0;

		for (i = 0x80; i != 0; i >>= 1) {
			if ((crc & 0x8000) != 0) {
				crc <<= 1;
				crc ^= 0x1021;
			} else {
				crc <<= 1;
			}

			if ((*ptr & i) != 0)
				crc ^= 0x1021;
		}
		ptr++;
	}

	buffer[0] = 0x01;
	buffer[1] = sn;
	buffer[2] = ~sn;
	memcpy(&buffer[3], buf, len);
	buffer[131] = crc >> 8;
	buffer[132] = crc & 0x0ff;

	i2cmsg.addr	= client->addr;
	i2cmsg.flags	= 0;
	i2cmsg.len	= 133;
	i2cmsg.buf	= buffer;

	return i2c_transfer(client->adapter, &i2cmsg, 1);
}

/* == Update firmware function == */
static int update_fw_func(void *pointer)
{
	int			i = 0;
	uint8_t			cmd[2] = {0, 0}, *buffer = NULL;
	struct i2c_msg		i2cmsg[2];
	const struct firmware	*fw;
	struct ft5202_data	*ptr = (struct ft5202_data *)pointer;

	/* == Load Firmware == */
	if (request_firmware(&fw, FIRMWARE_NAME, &ptr->client->dev)) {
		dev_err(&ptr->client->dev, \
			"[update_fw_func] Request Firmware ERROR!\n");
		unregister_process(3, ptr);
		return -EIO;
	} else {
		dev_err(&ptr->client->dev, \
	"[update_fw_func] Firmware %d bytes has been loaded!\n", fw->size);
	}

	/* == Check Handshack ID == */
	i2cmsg[0].addr	= ptr->client->addr;
	i2cmsg[0].flags	= 1;
	i2cmsg[0].len	= 1;
	i2cmsg[0].buf	= cmd;
	if (i2c_transfer(ptr->client->adapter, &i2cmsg[0], 1) > 0) {
		dev_err(&ptr->client->dev, \
		"[update_fw_func] Handshack ID has been read %x\n", cmd[0]);
		if (cmd[0] != 0x43) {
			dev_err(&ptr->client->dev, \
		"[update_fw_func] Handshack ID ERROR!\n");
			unregister_process(3, ptr);
			return -EIO;
		}
	} else {
		dev_err(&ptr->client->dev, \
		"[update_fw_func] Read Handshack ID ERROR!\n");
		unregister_process(3, ptr);
		return -EIO;
	}

	/* == Send data to CTP IC == */
	buffer = (uint8_t *)fw->data;
	while ((i << 7) < fw->size) {
		if (ecc_maker(ptr->client, i + 1, &buffer[i << 7], 128) < 0) {
			dev_err(&ptr->client->dev, \
		"[update_fw_func] Sending data failed!\n");
			unregister_process(3, ptr);
			return -EIO;
		}

		msleep(20);

		if (i2c_transfer(ptr->client->adapter, &i2cmsg[0], 1) < 0) {
			dev_err(&ptr->client->dev, \
		"[update_fw_func] Read %d Responds ACK ERROR!\n", i);
			unregister_process(3, ptr);
			return -EIO;
		}
		if (cmd[0] != 0x06) {
			dev_err(&ptr->client->dev, \
		"[update_fw_func] Responds Check ERROR on %d packages!\n", i);
			unregister_process(3, ptr);
			return -EIO;
		}
		i++;
	}

	/* == Send CTP reset command == */
	cmd[0] = 0x04;
	i2cmsg[0].flags	= 0;
	if (i2c_transfer(ptr->client->adapter, &i2cmsg[0], 1) < 0) {
		dev_err(&ptr->client->dev, \
			"[update_fw_func] Send reset ERROR!\n");
			unregister_process(3, ptr);
			return -EIO;
	} else {
		dev_err(&ptr->client->dev, \
		"[update_fw_func] %d packages has been updated!\n", i-1);
	}
	msleep(100);

	/* == Check CTP status == */
	cmd[0]		= 0xfc;
	cmd[1]		= 0x7c;
	i2cmsg[0].addr	= ptr->client->addr;
	i2cmsg[0].flags	= 0;
	i2cmsg[0].len	= 2;
	i2cmsg[0].buf	= cmd;
	i2cmsg[1].addr	= ptr->client->addr;
	i2cmsg[1].flags	= 1;
	i2cmsg[1].len	= 2;
	i2cmsg[1].buf	= cmd;

	if (i2c_transfer(ptr->client->adapter, i2cmsg, 2) > 0) {
		if (cmd[0] != 1) {
			dev_err(&ptr->client->dev, \
	"[update_fw_func] Touchpanel is not currect working %x,%x\n", \
			cmd[0], cmd[1]);
			unregister_process(3, ptr);
			return -EIO;
		} else {
			dev_err(&ptr->client->dev, \
				"[update_fw_func] Touchpanel is working!\n");
		}
	} else {
		dev_err(&ptr->client->dev, \
	"[update_fw_func] Read Touch Panel Working Status ERROR via I2C !\n");
		unregister_process(3, ptr);
		return -EIO;
	}

	/* == Read CTP firmware verison == */
	cmd[0]		= 0xfc;
	cmd[1]		= 0x7b;
	if (i2c_transfer(ptr->client->adapter, i2cmsg, 2) > 0) {
		dev_err(&ptr->client->dev, \
		"Touchpanel verison is %d\n", cmd[0]);
	} else {
		dev_err(&ptr->client->dev, \
		"[update_fw_func] Read Touch Panel verison ERROR via I2C !\n");
		unregister_process(3, ptr);
		return -EIO;
	}

	/* == Setting GPIO and IRQ == */
	if (gpio_request(TOUCHPANEL_GPIO_INT, "TouchPanel") < 0) {
		dev_err(&ptr->client->dev, \
			"[update_fw_func] GPIO request Fail\n");
		unregister_process(3, ptr);
		return -EIO;
	}

	if (gpio_direction_input(TOUCHPANEL_GPIO_INT) < 0) {
		dev_err(&ptr->client->dev, \
			"[update_fw_func] Can not input GPIO\n");
		unregister_process(4, ptr);
		return -EIO;
	}

	ptr->client->irq = gpio_to_irq(TOUCHPANEL_GPIO_INT);
	if (request_irq(ptr->client->irq, ft5202_irq, \
			IRQF_TRIGGER_FALLING, "TouchPanel", ptr)) {
		dev_err(&ptr->client->dev, \
			"[update_fw_func] Can request IRQ\n");
		unregister_process(5, ptr);
		return -EIO;
	}

	/* == Setting suspend and resume state == */
	ptr->suspend_ptr.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ptr->suspend_ptr.suspend	= ft5202_suspend;
	ptr->suspend_ptr.resume		= ft5202_resume;
	register_early_suspend(&ptr->suspend_ptr);

	/* == Initize Read Data Message == */
	ptr->i2cmsg[0].addr	= ptr->client->addr;
	ptr->i2cmsg[0].flags	= 0;
	ptr->i2cmsg[0].len	= 1;
	ptr->i2cmsg[0].buf	= &ptr->ctp_data[0];
	ptr->i2cmsg[1].addr	= ptr->client->addr;
	ptr->i2cmsg[1].flags	= 1;
	ptr->i2cmsg[1].len	= 26;
	ptr->i2cmsg[1].buf	= &ptr->ctp_data[1];
	ptr->ctp_data[0]	= 0xf9;

	release_firmware(fw);
	return 0;
}
