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
#include "focaltech_ts_config.h"

struct focaltech_ic_report_rate_config report_rate_config_info = {
#if defined(PRODUCT_BANGKK)
	.rate_config_count = 2,
	.refresh_rate_ctrl = 0,
	.interpolation_ctrl = 1,
	{
		{
			.interpolation_flag = 0,
			.report_rate = 240,
			.command = 0,
		},
		{
			.interpolation_flag = 1,
			.report_rate = 360,
			.command = 1,
		},
	}
#endif
};

int fts_mmi_get_report_rate(struct fts_ts_data *ts_data)
{
	int refresh_rate_ctrl = 0;
	int interpolation_ctrl = 0;
	int interpolation_flag = 0;
	int refresh_rate = 0;
	int i = 0;

	refresh_rate_ctrl = ts_data->pdata->report_rate_ctrl;
	interpolation_ctrl = ts_data->pdata->interpolation_ctrl;

	interpolation_flag = ts_data->get_mode.interpolation;
	refresh_rate = ts_data->refresh_rate;

	FTS_DEBUG("refresh_rate_ctrl: %d, interpolation_ctrl: %d, interpolation_flag: %d, refresh_rate: %d",
		refresh_rate_ctrl, interpolation_ctrl, interpolation_flag, refresh_rate);

	if (refresh_rate_ctrl == 0 && interpolation_ctrl == 1) {
		for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
			if (interpolation_flag == report_rate_config_info.report_rate_info[i].interpolation_flag) {
				break;
			}
		}
	} else if (refresh_rate_ctrl == 1 && interpolation_ctrl == 1) {
		for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
			if ((interpolation_flag == report_rate_config_info.report_rate_info[i].interpolation_flag) &&
				((refresh_rate >= report_rate_config_info.report_rate_info[i].refresh_rate[0]) &&
				(refresh_rate <= report_rate_config_info.report_rate_info[i].refresh_rate[1]))) {
				break;
			}
		}
	} else if (refresh_rate_ctrl == 1 && interpolation_ctrl == 0) {
		for (i = 0; i < report_rate_config_info.rate_config_count; i++) {
			if ((refresh_rate >= report_rate_config_info.report_rate_info[i].refresh_rate[0]) &&
				(refresh_rate <= report_rate_config_info.report_rate_info[i].refresh_rate[1])) {
				break;
			}
		}
	} else {
		//refresh_rate_ctrl = 0, interpolation_ctrl = 0
		i = 0;
	}

	if (i == report_rate_config_info.rate_config_count) {
		FTS_ERROR("Get config report rate fail");
		return -1;
	} else {
		FTS_DEBUG("Get config report rate %dHZ, command : 0x%02x ",
			report_rate_config_info.report_rate_info[i].report_rate,
			report_rate_config_info.report_rate_info[i].command);
		return report_rate_config_info.report_rate_info[i].command;
	}
}

