/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <asm/mach-types.h>

static unsigned int keypad_row_gpios[] = { 35, 34, 33, 32 };

static unsigned int keypad_col_gpios[] = { 41, 40, 39, 38, 37 };

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))

static const unsigned short keypad_keymap_qrd[ARRAY_SIZE(keypad_col_gpios) *
					  ARRAY_SIZE(keypad_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_UP,
	[KEYMAP_INDEX(0, 1)] = KEY_REPLY,
	[KEYMAP_INDEX(0, 2)] = KEY_LEFT,
	[KEYMAP_INDEX(0, 3)] = KEY_RIGHT,
	[KEYMAP_INDEX(0, 4)] = KEY_DOWN,

	[KEYMAP_INDEX(1, 1)] = KEY_SEND,		/* CALL -> AC Send */
	[KEYMAP_INDEX(1, 2)] = KEY_END,

	[KEYMAP_INDEX(2, 2)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(2, 3)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(3, 0)] = KEY_HP,			/* CAMERA1 */
	[KEYMAP_INDEX(3, 1)] = KEY_CAMERA		/* CAMERA2 */
};

static struct gpio_event_matrix_info qrd_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_qrd,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *qrd_keypad_info[] = {
	&qrd_keypad_matrix_info.info
};

static struct gpio_event_platform_data qrd_keypad_data = {
	.name		= "qrd_keypad",
	.info		= qrd_keypad_info,
	.info_count	= ARRAY_SIZE(qrd_keypad_info)
};

struct platform_device keypad_device_qrd = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &qrd_keypad_data,
	},
};

