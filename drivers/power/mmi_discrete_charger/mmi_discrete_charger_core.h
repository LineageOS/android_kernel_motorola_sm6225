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
 #ifndef __MMI_DISCRETE_CHARGER_CORE_H__
#define __MMI_DISCRETE_CHARGER_CORE_H__

#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/extcon-provider.h>
#include "mmi_charger.h"
#include <linux/usb/mmi_discrete_typec.h>
#include <linux/usb/adapter_class.h>

#define MMI_HB_VOTER			"MMI_HB_VOTER"
#define USER_VOTER			"USER_VOTER"
#define USB_PSY_VOTER			"USB_PSY_VOTER"
#define BATT_PROFILE_VOTER		"BATT_PROFILE_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"
#define WBC_VOTER			"WBC_VOTER"
#define SW_ICL_MAX_VOTER		"SW_ICL_MAX_VOTER"
#define HW_LIMIT_VOTER			"HW_LIMIT_VOTER"
#define DEFAULT_VOTER			"DEFAULT_VOTER"
#define PD_VOTER			"PD_VOTER"
#define QC3P_VOTER			"QC3P_VOTER"

#define SDP_100_MA			100000
#define SDP_CURRENT_UA			500000
#define CDP_CURRENT_UA			1500000
#define DCP_CURRENT_UA			1500000
#define HVDCP_CURRENT_UA		3000000
#define TYPEC_DEFAULT_CURRENT_UA	900000
#define TYPEC_MEDIUM_CURRENT_UA		1500000
#define TYPEC_HIGH_CURRENT_UA		3000000
#define WIRELESS_CURRENT_DEFAULT_UA	1000000

static const unsigned int mmi_discrete_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum {
	MMI_POWER_SUPPLY_DP_DM_UNKNOWN = 0,
	MMI_POWER_SUPPLY_DP_DM_DP_PULSE = 1,
	MMI_POWER_SUPPLY_DP_DM_DM_PULSE = 2,
};

struct mmi_discrete_chg_client {
	struct mmi_discrete_charger		*chip;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_driver	*driver;
	struct power_supply		*client_batt_psy;
	u32				chrg_taper_cnt;
};

struct mmi_discrete_charger {
	struct device		*dev;
	char				*name;
	struct power_supply	*mmi_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*charger_psy;
	struct power_supply	*usb_port_psy;
	struct power_supply	*dc_psy;
	struct power_supply 	*wls_psy;

	struct votable 		*chg_disable_votable;
	struct votable			*fcc_votable;
	struct votable			*fv_votable;
	struct votable			*usb_icl_votable;
	struct votable			*dc_suspend_votable;

	struct delayed_work	charger_work;
	struct mmi_charger_info	chg_info;
	struct mmi_charger_constraint	constraint;
	int					chg_client_num;
	struct mmi_discrete_chg_client	*chg_clients;

	struct charger_device	*master_chg_dev;
	struct notifier_block	master_chg_nb;

	bool			vbus_enabled;
	int			real_charger_type;
	int 			hvdcp2_max_icl_ua;
	int 			wls_max_icl_ua;

	struct delayed_work	monitor_ibat_work;
	int			bat_ocp_ua;

	/* extcon for VBUS / ID notification to USB for type-c only */
	struct extcon_dev	*extcon;

	/*IIO*/
	struct iio_dev		*indio_dev;
	struct iio_chan_spec	*iio_chan;
	struct iio_channel	*int_iio_chans;

	/*PD*/
	struct adapter_device *pd_adapter_dev;
	int			pd_active;
	bool			pd_supported;

	int			dc_cl_ma;
	int			batt_profile_fv_uv;
	int			batt_profile_fcc_ua;
	int			hw_max_icl_ua;
	int			otg_cl_ua;

	/*qc3.5*/
	bool			qc3p5_detected;

	bool			*debug_enabled;
	void			*ipc_log;

	/* cached status */
	int			thermal_levels;
	int			system_temp_level;
	int			*thermal_mitigation;
	int			typec_mode;
	bool			use_extcon;
};

int mmi_discrete_otg_enable(struct mmi_discrete_charger *chip, bool en);
int mmi_discrete_is_usb_suspended(struct mmi_discrete_charger *chip);
int mmi_discrete_config_typec_mode(struct mmi_discrete_charger *chip, int val);
int mmi_discrete_config_pd_active(struct mmi_discrete_charger *chip, int val);
int mmi_discrete_get_hw_current_max(struct mmi_discrete_charger *chip, int *val);
int mmi_discrete_config_charging_enabled(struct mmi_discrete_charger *chip, int val);
int mmi_discrete_get_charging_enabled(struct mmi_discrete_charger *chip, bool *val);
int mmi_discrete_config_input_current_settled(struct mmi_discrete_charger *chip, int val);
int mmi_discrete_config_termination_enabled(struct mmi_discrete_charger *chip, int val);
int mmi_discrete_get_qc3p_power(struct mmi_discrete_charger *chip, int *val);
int mmi_discrete_get_pulse_cnt(struct mmi_discrete_charger *chip, int *count);
int mmi_discrete_set_dp_dm(struct mmi_discrete_charger *chip, int val);
int mmi_discrete_get_charger_suspend(struct mmi_discrete_charger *chip, bool *val);
int mmi_discrete_get_typec_accessory_mode(struct mmi_discrete_charger *chip, int *val);

#endif
