/*
 * Definitions for AL3002 light sensor chip.
*/

#ifndef AL3002_H
#define AL3002_H

#include <linux/ioctl.h>

#define AL3002_I2C_NAME	"al3002"
#define IOCTL_NUMBER	0xA2

/* AL3002 register address */
#define CONFIG				0x00
#define TIMING_CONTROL		0x01
#define ALS_CONTROL			0x02
#define INTERRUPT_STATUS	0x03
#define PXY_CONTROL			0x04
#define ALS_DATA				0x05
#define ALS_WINDOW			0x08

/* const */
#define CONFIG_INITIAL		0x03
#define ALS_MODE_POWER_UP		1
#define ALS_MODE_POWER_DOWN	2
#define ALS_MODE_RESET			3
#define ALS_MODE_ALS_ACTIVE	4
#define ALS_MODE_PXY_ACTIVE	5
#define ALS_MODE_ALL_ACTIVE	6
#define ALS_MODE_IDLE			7
#define ALS_READ_DATA_ALS		0
#define ALS_READ_DATA_PXY		1

/* IOCTLs*/
#define ALS_IOCTL_INITIAL			_IO(IOCTL_NUMBER, 1)
#define ALS_IOCTL_WRITE			_IOW(IOCTL_NUMBER, 2, char[5])
#define ALS_IOCTL_READ			_IOR(IOCTL_NUMBER, 3, char[5])
#define ALS_IOCTL_READ_DATA	_IOR(IOCTL_NUMBER, 4, long)
#define ALS_IOCTL_SET_MODE		_IOW(IOCTL_NUMBER, 5, short)
#define ALS_IOCTL_SET_LOSS		_IOW(IOCTL_NUMBER, 6, short)
#define ALS_IOCTL_SET_THRESHOLD	_IOW(IOCTL_NUMBER, 7, short)
#define ALS_IOCTL_GET_INT		_IOR(IOCTL_NUMBER, 8, short)

/* TOUCH SCREEN IOCTLs */
#define CTS_IOCTL_SUSPEND		_IO(IOCTL_NUMBER, 9)
#define CTS_IOCTL_RESUME		_IO(IOCTL_NUMBER, 10)

/* macros */
#define config_mode_als_active(data)		((data & 0xFC) | 0x00)
#define config_mode_pxy_active(data)	((data & 0xFC) | 0x01)
#define config_mode_all_active(data)		((data & 0xFC) | 0x02)
#define config_mode_idle(data)			((data & 0xFC) | 0x03)
#define config_mode_power_up(data)		((data & 0xF3))
#define config_mode_power_down(data)	((data & 0xF3) | (0x02<<2))
#define config_mode_reset(data)		((data & 0xF3) | (0x03<<2))
#define als_control_low_lux_threshold(data, lux)	((data & 0xE0) | lux)
#define data_read_als(data)			(data & 0x3f)
#define data_read_pxy(data)			((data & 0xc0) >> 7)
#define ALS_WINDOW_LOSS_0_PERCENT		0x00
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
