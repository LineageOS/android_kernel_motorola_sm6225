/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#ifndef __MMI_CHRG_CORE_IIO_H
#define __MMI_CHRG_CORE_IIO_H

#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>

struct mmi_charger_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define MMI_CHARGER_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define MMI_CHARGER_CHAN_INDEX(_name, _num)			\
	MMI_CHARGER_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct mmi_charger_iio_channels mmi_charger_iio_psy_channels[] = {
	MMI_CHARGER_CHAN_INDEX("cp_charging_enabled", PSY_IIO_CP_ENABLE)
};
enum mmi_charger_ext_iio_channels {
	/*smb5*/
	SMB5_HW_CURRENT_MAX,
	SMB5_CHARGING_ENABLED,
	SMB5_TYPEC_MODE,
	SMB5_USB_INPUT_CURRENT_SETTLED,
	SMB5_USB_PD_ACTIVE,
	/*qc3p*/
	SMB5_USB_REAL_TYPE,
	SMB5_DP_DM,
	SMB5_QC3P_POWER,
	/*cp*/
	CP_ENABLE,
	CP_INPUT_CURRENT_NOW,
	CP_INPUT_VOLTAGE_NOW,
	CP_STATUS1,
	CP_CLEAR_ERROR,
};

static const char * const mmi_charger_ext_iio_chan_name[] = {
	/*smb5*/
	[SMB5_HW_CURRENT_MAX] = "usb_hw_current_max",
	[SMB5_CHARGING_ENABLED] = "charging_enabled",
	[SMB5_TYPEC_MODE] = "usb_typec_mode",
	[SMB5_USB_INPUT_CURRENT_SETTLED] = "usb_input_current_settled",
	[SMB5_USB_PD_ACTIVE] = "usb_pd_active",
	/*qc3p*/
	[SMB5_USB_REAL_TYPE] = "usb_real_type",
	[SMB5_DP_DM] = "battery_dp_dm",
	[SMB5_QC3P_POWER] = "usb_qc3p_power",
	/*cp*/
	[CP_ENABLE] = "cp_enable",
	[CP_INPUT_CURRENT_NOW] = "cp_input_current_now",
	[CP_INPUT_VOLTAGE_NOW] = "cp_input_voltage_now",
	[CP_STATUS1] = "cp_status1",
	[CP_CLEAR_ERROR] = "cp_clear_error",
};

int mmi_charger_read_iio_chan(struct mmi_charger_manager *chip,
	enum mmi_charger_ext_iio_channels chan, int *val);

int mmi_charger_write_iio_chan(struct mmi_charger_manager *chip,
	enum mmi_charger_ext_iio_channels chan, int val);

#endif
