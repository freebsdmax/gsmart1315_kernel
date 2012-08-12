/*
 * Definitions for AL3003 light sensor chip.
*/

#ifndef AL3003_H
#define AL3003_H

#include <linux/ioctl.h>

#define AL3003_I2C_NAME			"al3003"
#define AL3003_IOCTL_NUMBER			0xA2
#define AL3003_INTERRUPT_GPIO	28

/* AL3003 register address */
#define CONFIG				0x00
#define TIMING_CONTROL		0x01
#define ALS_CONTROL			0x02
#define INTERRUPT_STATUS	0x03
#define PXY_CONTROL			0x04
#define AL3003_DATA			0x05
#define ALS_WINDOW			0x08

/* const */
#define CONFIG_INITIAL				0x0b /* power down & idle mode */
#define TIMING_CONTROL_INITIAL		0x00 /* 1 PS integration cycles & 1 ALS integration cycles */
#define ALS_CONTROL_INITIAL			0xaa /* 64 levels & 0x00 Low Lux */
#define PXY_CONTROL_INITIAL			0x48 /* PS threshold level */
#define ALS_WINDOW_INITIAL			0x0a /* 0x00 loss compensation */
#define AL3003_MODE_POWER_UP		1
#define AL3003_MODE_POWER_DOWN	2
#define AL3003_MODE_RESET			3
#define AL3003_MODE_ALS_ACTIVE		4
#define AL3003_MODE_PXY_ACTIVE		5
#define AL3003_MODE_ALL_ACTIVE		6
#define AL3003_MODE_IDLE				7
#define AL3003_READ_DATA_ALS		0
#define AL3003_READ_DATA_PXY		1
#define CONFIG_MODE_ALS_ACTIVE		0x00
#define CONFIG_MODE_PXY_ACTIVE	0x01
#define CONFIG_MODE_ALL_ACTIVE		0x02
#define CONFIG_MODE_IDLE			0x0b
#define CONFIG_MODE_POWER_DOWN	0x0b
#define CONFIG_MODE_RESET			0x0c

/* IOCTLs*/
#define AL3003_IOCTL_INITIAL		_IO(AL3003_IOCTL_NUMBER, 1)
#define AL3003_IOCTL_WRITE		_IOW(AL3003_IOCTL_NUMBER, 2, char[5])
#define AL3003_IOCTL_READ		_IOR(AL3003_IOCTL_NUMBER, 3, char[5])
#define AL3003_IOCTL_READ_DATA	_IOR(AL3003_IOCTL_NUMBER, 4, long)
#define AL3003_IOCTL_SET_MODE	_IOW(AL3003_IOCTL_NUMBER, 5, short)
#define AL3003_IOCTL_SET_LOSS		_IOW(AL3003_IOCTL_NUMBER, 6, short)
#define AL3003_IOCTL_SET_THRESHOLD \
	_IOW(AL3003_IOCTL_NUMBER, 7, short)
#define AL3003_IOCTL_GET_INT		_IOR(AL3003_IOCTL_NUMBER, 8, short)
#define AL3003_IOCTL_SYSTEM_SUSPEND	_IO(AL3003_IOCTL_NUMBER, 9)
#define AL3003_IOCTL_SYSTEM_RESUME	_IO(AL3003_IOCTL_NUMBER, 10)
#define AL3003_IOCTL_AUTO_SET_THRESHOLD _IOR(AL3003_IOCTL_NUMBER, 11, short)
#define AL3003_IOCTL_NO_EARLY_SUSPEND		_IOR(AL3003_IOCTL_NUMBER, 12, short)
#define AL3003_IOCTL_P_SENSOR_FORCE_ENABLE _IOW(AL3003_IOCTL_NUMBER, 13, short)

/* macros */
#define config_mode_power_up(data)		((data & 0xF3))
#define als_control_low_lux_threshold(data, lux)		((data & 0xE0) | lux)
#define data_read_als(data)			(data & 0x3f)
#define data_read_pxy(data)			((data & 0xc0) >> 7)
#define ALS_WINDOW_LOSS_0_PERCENT	0x00
#define ALS_WINDOW_LOSS_17_PERCENT	0x01
#define ALS_WINDOW_LOSS_31_PERCENT	0x02
#define ALS_WINDOW_LOSS_42_PERCENT	0x03
#define ALS_WINDOW_LOSS_52_PERCENT	0x04
#define ALS_WINDOW_LOSS_60_PERCENT	0x05
#define ALS_WINDOW_LOSS_65_PERCENT	0x06
#define ALS_WINDOW_LOSS_67_PERCENT	0x07
#define ALS_WINDOW_LOSS_72_PERCENT	0x08
#define ALS_WINDOW_LOSS_81_PERCENT	0x09
#define ALS_WINDOW_LOSS_84_PERCENT	0x0a
#define ALS_WINDOW_LOSS_86_PERCENT	0x0b
#define ALS_WINDOW_LOSS_89_PERCENT	0x0c
#define ALS_WINDOW_LOSS_91_PERCENT	0x0d
#define ALS_WINDOW_LOSS_92_PERCENT	0x0e
#define ALS_WINDOW_LOSS_94_PERCENT	0x0f

#endif
