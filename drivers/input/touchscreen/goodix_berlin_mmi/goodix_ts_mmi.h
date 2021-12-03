/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __GOODIX_TS_MMI_H__
#define __GOODIX_TS_MMI_H__

#include <linux/platform_device.h>
#include <linux/touchscreen_mmi.h>
#include <linux/mmi_wake_lock.h>

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
int goodix_ts_mmi_dev_register(struct platform_device *ts_device);
void goodix_ts_mmi_dev_unregister(struct platform_device *ts_device);
#else
static int inline goodix_ts_mmi_dev_register(struct platform_device *ts_device) {
	return -ENOSYS;
}
static void inline goodix_ts_mmi_dev_unregister(struct platform_device *ts_device) {
	return -ENOSYS;
}
#endif

#define LEATHER_MODE_SWITCH_CMD    0x26
#define ENTER_LEATHER_MODE         0x01
#define EXIT_LEATHER_MODE          0x00

#define FILM_MODE_SWITCH_CMD       0x27
#define ENTER_FILM_MODE            0x01
#define EXIT_FILM_MODE             0x00

#define EDGE_SWITCH_CMD            0x17
#define ROTATE_DEFAULT_0           0x00
#define ROTATE_LEFT_90             0X40
#define ROTATE_RIGHT_90            0x80
#define DEFAULT_EDGE               0x00
#define SMALL_EDGE                 0x40
#define BIG_EDGE                   0x80

#define STYLUS_MODE_SWITCH_CMD     0xA4

#define INTERPOLATION_SWITCH_CMD   0xC2
#define REPORT_RATE_DEFAULT        0x00
#define REPORT_RATE_720HZ          0x01
#define REPORT_RATE_576HZ          0x02
#define REPORT_RATE_480HZ          0x03

#endif
