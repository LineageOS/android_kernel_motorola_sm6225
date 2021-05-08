/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#ifndef __SC8549_IIO_H
#define __SC8549_IIO_H

#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>

struct sc8549_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define SC8549_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define SC8549_CHAN_INDEX(_name, _num)			\
	SC8549_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8549_CHAN_CUR(_name, _num)			\
	SC8549_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8549_CHAN_VOLT(_name, _num)			\
	SC8549_IIO_CHAN(_name, _num, IIO_VOLTAGE,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct sc8549_iio_channels sc8549_iio_psy_channels[] = {
	SC8549_CHAN_INDEX("sc8549_cp_enable", PSY_IIO_CP_ENABLE)
	SC8549_CHAN_CUR("sc8549_input_current_now", PSY_IIO_MMI_CP_INPUT_CURRENT_NOW)
	SC8549_CHAN_VOLT("sc8549_input_voltage_now", PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW)
	SC8549_CHAN_INDEX("sc8549_cp_status1", PSY_IIO_CP_STATUS1)
	SC8549_CHAN_INDEX("sc8549_cp_clear_error", PSY_IIO_MMI_CP_CLEAR_ERROR)
};

#endif