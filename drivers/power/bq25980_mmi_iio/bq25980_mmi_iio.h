/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/ */

#ifndef BQ25980_IIO_H
#define BQ25980_IIO_H


#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>


struct bq25980_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

enum {
	MMI_DISABLE_ADC = 0,
	MMI_ENABLE_ADC,
};

#define bq25980_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define bq25980_CHAN_INDEX(_name, _num)			\
	bq25980_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define bq25980_CHAN_CUR(_name, _num)			\
	bq25980_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define bq25980_CHAN_VOLT(_name, _num)			\
	bq25980_IIO_CHAN(_name, _num, IIO_VOLTAGE,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct bq25980_iio_channels bq25980_iio_psy_channels[] = {
	bq25980_CHAN_INDEX("bq25980_cp_enabled", PSY_IIO_CP_ENABLE)
	bq25980_CHAN_INDEX("bq25980_online", PSY_IIO_ONLINE)
	bq25980_CHAN_CUR("bq25980_current_now", PSY_IIO_CURRENT_NOW)
	bq25980_CHAN_VOLT("bq25980_voltage_now", PSY_IIO_VOLTAGE_NOW)
	bq25980_CHAN_INDEX("bq25980_input_current_now", PSY_IIO_MMI_CP_INPUT_CURRENT_NOW)
	bq25980_CHAN_INDEX("bq25980_input_voltage_settled", PSY_IIO_MMI_CP_INPUT_VOLTAGE_NOW)
	bq25980_CHAN_INDEX("bq25980_cp_status1", PSY_IIO_CP_STATUS1)
	bq25980_CHAN_INDEX("bq25980_cp_clear_error", PSY_IIO_CP_CLEAR_ERROR)
};

#endif /* BQ25980_IIO_H */
