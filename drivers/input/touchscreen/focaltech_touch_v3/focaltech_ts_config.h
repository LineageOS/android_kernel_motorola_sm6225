/*
 * Copyright (C) 2022 Motorola Mobility LLC
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
#ifndef __FOCALTECH_TS_CONFIG_H__
#define __FOCALTECH_TS_CONFIG_H__

#include <linux/touchscreen_mmi.h>
#include "focaltech_core.h"

#define MAX_REPORT_RATE_CONFIG 12

#define REPORT_RATE_CMD_240HZ 0
#define REPORT_RATE_CMD_360HZ 1


struct report_rate_config {
	bool interpolation_flag; //if enable interpolation report rate
	u16 refresh_rate[2]; //display refresh rate
	u16 report_rate; //touch report rate
	u16 command; //report rate switch command
};

struct focaltech_ic_report_rate_config {
	u8 rate_config_count; //the count of report rate combination
	bool refresh_rate_ctrl; //if support report rate change according to refresh rate
	bool interpolation_ctrl; //if support interpolation report rate
	struct report_rate_config report_rate_info[MAX_REPORT_RATE_CONFIG];
};

int fts_mmi_get_report_rate(struct fts_ts_data *ts_data);

#endif
