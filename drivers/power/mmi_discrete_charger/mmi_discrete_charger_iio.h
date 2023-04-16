/*
 * Copyright (C) 2020 Motorola Mobility LLC
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

#ifndef __MMI_DISCRETE_CHARGER_IIO_H__
#define __MMI_DISCRETE_CHARGER_IIO_H__

#include <linux/platform_device.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>
#include "mmi_discrete_charger_core.h"

struct mmi_discrete_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define MMI_DISCRETE_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define MMI_DISCRETE_CHAN_COUNT(_name, _num)			\
	MMI_DISCRETE_IIO_CHAN(_name, _num,IIO_COUNT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define MMI_DISCRETE_CHAN_INDEX(_name, _num)			\
	MMI_DISCRETE_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct mmi_discrete_iio_channels mmi_discrete_iio_psy_channels[] = {
	MMI_DISCRETE_CHAN_INDEX("usb_real_type", PSY_IIO_USB_REAL_TYPE)
	MMI_DISCRETE_CHAN_INDEX("otg_enable", PSY_IIO_MMI_OTG_ENABLE)
	MMI_DISCRETE_CHAN_INDEX("typec_mode", PSY_IIO_TYPEC_MODE)
	MMI_DISCRETE_CHAN_INDEX("pd_active", PSY_IIO_PD_ACTIVE)
	MMI_DISCRETE_CHAN_INDEX("hw_current_max", PSY_IIO_HW_CURRENT_MAX)
	MMI_DISCRETE_CHAN_INDEX("charging_enabled", PSY_IIO_USB_CHARGING_ENABLED)
	MMI_DISCRETE_CHAN_INDEX("input_current_settled", PSY_IIO_INPUT_CURRENT_SETTLED)
	MMI_DISCRETE_CHAN_INDEX("termination_enabled", PSY_IIO_USB_TERMINATION_ENABLED)
	MMI_DISCRETE_CHAN_COUNT("battery_dp_dm", PSY_IIO_DP_DM)
	MMI_DISCRETE_CHAN_INDEX("usb_qc3p_power", PSY_IIO_MMI_QC3P_POWER)
};

int mmi_discrete_init_iio_psy(struct mmi_discrete_charger *chip,
				struct platform_device *pdev);

#endif