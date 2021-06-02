/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#ifndef __MMI_CHRG_CORE_IIO_H
#define __MMI_CHRG_CORE_IIO_H

#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>

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
	/*sc8549*/
	SC8549_CP_ENABLE,
	SC8549_INPUT_CURRENT_NOW,
	SC8549_INPUT_VOLTAGE_NOW,
	SC8549_CP_STATUS1,
	SC8549_CP_CLEAR_ERROR,
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
	/*sc8549*/
	[SC8549_CP_ENABLE] = "sc8549_cp_enable",
	[SC8549_INPUT_CURRENT_NOW] = "sc8549_input_current_now",
	[SC8549_INPUT_VOLTAGE_NOW] = "sc8549_input_voltage_now",
	[SC8549_CP_STATUS1] = "sc8549_cp_status1",
	[SC8549_CP_CLEAR_ERROR] = "sc8549_cp_clear_error",
};

int mmi_charger_read_iio_chan(struct mmi_charger_manager *chip,
	enum mmi_charger_ext_iio_channels chan, int *val);

int mmi_charger_write_iio_chan(struct mmi_charger_manager *chip,
	enum mmi_charger_ext_iio_channels chan, int val);

#endif
