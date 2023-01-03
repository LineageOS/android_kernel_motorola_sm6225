/*
 * Copyright (C) 2023 Motorola Mobility LLC
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
#ifndef __ZINITIX_TS_MMI_H__
#define __ZINITIX_TS_MMI_H__

#include <linux/touchscreen_mmi.h>
#include <linux/mmi_wake_lock.h>
#include "zinitix_touch.h"

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
int zinitix_ts_mmi_dev_register(struct device *dev);
void zinitix_ts_mmi_dev_unregister(struct device *dev);
#else
static int inline zinitix_ts_mmi_dev_register(struct device *dev) {
	return -ENOSYS;
}
static void inline zinitix_ts_mmi_dev_unregister(struct device *dev) {
	return ;
}
#endif

#endif
