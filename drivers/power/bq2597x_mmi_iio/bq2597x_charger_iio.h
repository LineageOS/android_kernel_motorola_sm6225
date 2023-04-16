/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#ifndef __BQ2597X_IIO_H
#define __BQ2597X_IIO_H

#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>

struct bq2597x_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define bq2597x_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define bq2597x_CHAN_INDEX(_name, _num)			\
	bq2597x_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define bq2597x_CHAN_CUR(_name, _num)			\
	bq2597x_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define bq2597x_CHAN_VOLT(_name, _num)			\
	bq2597x_IIO_CHAN(_name, _num, IIO_VOLTAGE,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct bq2597x_iio_channels bq2597x_iio_psy_channels[] = {
	bq2597x_CHAN_INDEX("bq2597x_cp_enabled", PSY_IIO_CP_ENABLE)
	bq2597x_CHAN_INDEX("bq2597x_online", PSY_IIO_ONLINE)
	bq2597x_CHAN_CUR("bq2597x_current_now", PSY_IIO_CURRENT_NOW)
	bq2597x_CHAN_VOLT("bq2597x_voltage_now", PSY_IIO_VOLTAGE_NOW)
	bq2597x_CHAN_INDEX("bq2597x_input_current_now", PSY_IIO_MMI_CP_INPUT_CURRENT_NOW)
	bq2597x_CHAN_INDEX("bq2597x_input_voltage_settled", PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW)
	bq2597x_CHAN_INDEX("bq2597x_cp_status1", PSY_IIO_CP_STATUS1)
	bq2597x_CHAN_INDEX("bq2597x_cp_clear_error", PSY_IIO_CP_CLEAR_ERROR)
};

#endif
