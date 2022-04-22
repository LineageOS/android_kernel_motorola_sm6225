/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/ipc_logging.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/mmi_wake_lock.h>

#include "mmi_discrete_charger_core.h"
#include "mmi_discrete_voter.h"
#include "mmi_discrete_charger_iio.h"
#include "mmi_discrete_factory_tcmd.h"
#include <linux/mmi_discrete_charger_class.h>

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for MMI DISCRETE CHARGER driver");

static struct mmi_discrete_charger *this_chip = NULL;

#define typec_rp_med_high(chg, typec_mode)			\
	((typec_mode == MMI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM	\
	|| typec_mode == MMI_POWER_SUPPLY_TYPEC_SOURCE_HIGH))

#define MICRO_1PA			1000000
#define MICRO_1P5A			1500000
#define MICRO_3PA			3000000
static int mmi_discrete_parse_dts(struct mmi_discrete_charger *chip)
{
	int rc = 0, byte_len;
	struct device_node *node = chip->dev->of_node;

	rc = of_property_read_u32(node,
				"mmi,max-fv-mv", &chip->batt_profile_fv_uv);
	if (rc < 0)
		chip->batt_profile_fv_uv = -EINVAL;
	else
		chip->batt_profile_fv_uv = chip->batt_profile_fv_uv * 1000;

	rc = of_property_read_u32(node,
			"mmi,max-fcc-ma", &chip->batt_profile_fcc_ua);
	if (rc < 0)
		chip->batt_profile_fcc_ua = -EINVAL;
	else
		chip->batt_profile_fcc_ua = chip->batt_profile_fcc_ua * 1000;

	rc = of_property_read_u32(node,
				"mmi,hw-max-icl-ua", &chip->hw_max_icl_ua);
	if (rc < 0)
		chip->hw_max_icl_ua = MICRO_3PA;

	rc = of_property_read_u32(node,
				"mmi,otg-cl-ua", &chip->otg_cl_ua);
	if (rc < 0)
		chip->otg_cl_ua = MICRO_1PA;

	rc = of_property_read_u32(node,
				"mmi,bat-ocp-ua", &chip->bat_ocp_ua);
	if (rc < 0)
		chip->bat_ocp_ua = 0;

	of_property_read_u32(node, "mmi,hvdcp2-max-icl-ua",
					&chip->hvdcp2_max_icl_ua);
	if (chip->hvdcp2_max_icl_ua <= 0)
		chip->hvdcp2_max_icl_ua = MICRO_1P5A;

	of_property_read_u32(node, "mmi,wls-max-icl-ua",
					&chip->wls_max_icl_ua);
	if (chip->wls_max_icl_ua <= 0)
		chip->wls_max_icl_ua = WIRELESS_CURRENT_DEFAULT_UA;

	rc = of_property_read_u32(node, "mmi,dc-icl-ma",
				  &chip->dc_cl_ma);
	if (rc)
		chip->dc_cl_ma = -EINVAL;

	if (of_find_property(node, "mmi,thermal-mitigation", &byte_len)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev, byte_len,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL)
			return -ENOMEM;

		chip->thermal_levels = byte_len / sizeof(u32);
		rc = of_property_read_u32_array(node,
				"mmi,thermal-mitigation",
				chip->thermal_mitigation,
				chip->thermal_levels);
		if (rc < 0) {
			mmi_err(chip, "Couldn't read thermal limits rc = %d\n", rc);
			return rc;
		}
	}

	chip->pd_supported = of_property_read_bool(node, "mmi,usb-pd-supported");

	return 0;
}

int get_prop_batt_present(struct mmi_discrete_charger *chg,
				union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_PRESENT, val);
	if (rc)
		mmi_err(chg, "Couldn't get batt present prop rc=%d\n", rc);

	return rc;
}

int get_prop_batt_charge_type(struct mmi_discrete_charger *chg,
				union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->charger_psy,
				       POWER_SUPPLY_PROP_CHARGE_TYPE, val);
	if (rc)
		mmi_err(chg, "Couldn't get batt charge type prop rc=%d\n", rc);

	return rc;
}

int get_prop_system_temp_level(struct mmi_discrete_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int get_prop_system_temp_level_max(struct mmi_discrete_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels;
	return 0;
}

int get_batt_current_now(struct mmi_discrete_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	if (!rc)
		val->intval *= (-1);
	else
		mmi_err(chg, "Couldn't get current_now prop rc=%d\n", rc);

	return rc;
}

int get_prop_input_suspend(struct mmi_discrete_charger *chg,
				union power_supply_propval *val)
{
	val->intval = (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
			&& get_client_vote(chg->dc_suspend_votable, USER_VOTER);

	return 0;
}

int set_prop_input_suspend(struct mmi_discrete_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		mmi_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		mmi_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);

	return rc;
}

int set_prop_system_temp_level(struct mmi_discrete_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;

	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

static int get_prop_usb_voltage_now(struct mmi_discrete_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->charger_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

static int get_prop_usb_current_now(struct mmi_discrete_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->charger_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	if (rc)
		mmi_err(chip, "Couldn't get usb current now prop rc=%d\n", rc);

	return rc;
}

static int get_prop_usb_present(struct mmi_discrete_charger *chip,
				union power_supply_propval *val)
{
	int rc;

	if (!chip->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->charger_psy,
				       POWER_SUPPLY_PROP_PRESENT, val);
	if (rc)
		mmi_err(chip, "Couldn't get usb present prop rc=%d\n", rc);

	return rc;
}

int get_prop_usb_online(struct mmi_discrete_charger *chip,
			       union power_supply_propval *val)
{
	int rc;

	if (!chip->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->charger_psy,
				       POWER_SUPPLY_PROP_ONLINE, val);
	if (rc)
		mmi_err(chip, "Couldn't get usb online prop rc=%d\n", rc);

	return rc;
}

static int get_usb_online(struct mmi_discrete_charger *chip,
				union power_supply_propval *val)
{
	int rc;

	rc = get_prop_usb_online(chip, val);
	if (!val->intval)
		return rc;

	//typec logic mask, bring up bc1.2 firstly
	if ((chip->typec_mode == MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT)
		&& chip->real_charger_type == POWER_SUPPLY_TYPE_USB)
		val->intval = 0;
	else
		val->intval = 1;

	if (chip->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
		val->intval = 0;

	return rc;
}

static int get_usb_port_online(struct mmi_discrete_charger *chip,
				union power_supply_propval *val)
{
	int rc;

	rc = get_prop_usb_online(chip, val);
	if (!val->intval)
		return rc;

	//typec logic mask, bring up bc1.2 firstly
	if ((chip->typec_mode ==
		MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT)
		&& chip->real_charger_type == POWER_SUPPLY_TYPE_USB)
		val->intval = 1;
	else
		val->intval = 0;

	return rc;
}

static int get_prop_input_current_settled(struct mmi_discrete_charger *chip,
					  union power_supply_propval *val)
{
	int rc;

	if (!chip->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->charger_psy,
				       POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, val);
	if (rc)
		mmi_err(chip, "Couldn't get usb input current limit prop rc=%d\n", rc);

	return rc;
}

static int get_prop_dc_present(struct mmi_discrete_charger *chip,
				union power_supply_propval *val)
{
	int rc;

	if (!chip->wls_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->wls_psy,
				       POWER_SUPPLY_PROP_PRESENT, val);
	if (rc)
		mmi_err(chip, "Couldn't get dc present prop rc=%d\n", rc);

	return rc;
}

static int get_prop_dc_online(struct mmi_discrete_charger *chip,
			       union power_supply_propval *val)
{
	int rc;

	if (!chip->wls_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->wls_psy,
				       POWER_SUPPLY_PROP_ONLINE, val);
	if (rc)
		mmi_err(chip, "Couldn't get dc online prop rc=%d\n", rc);

	return rc;
}

static int get_prop_dc_voltage_now(struct mmi_discrete_charger *chip,
				    union power_supply_propval *val)
{
	int rc = 0;

	if (!chip->wls_psy) {
		chip->wls_psy = power_supply_get_by_name("wireless");
		if (!chip->wls_psy) {
			val->intval = -EINVAL;
			return 0;
		}
	}

	rc = power_supply_get_property(chip->wls_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW,
				val);
	if (rc < 0) {
		mmi_err(chip, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n",
				rc);
		return rc;
	}
	return rc;
}

static int get_prop_dc_current_now(struct mmi_discrete_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy) {
			val->intval = -EINVAL;
			return 0;
		}
	}

	if (chg->wls_psy) {
		rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW,
				val);
		if (rc < 0) {
			mmi_err(chg, "Couldn't get VOLTAGE_MAX, rc=%d\n",
					rc);
			return rc;
		}
	}
	return rc;
}

static int get_prop_dc_current_max(struct mmi_discrete_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->wls_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->wls_psy,
				       POWER_SUPPLY_PROP_CURRENT_MAX, val);
	if (rc)
		mmi_err(chip, "Couldn't get dc current max prop rc=%d\n", rc);

	return rc;
}

static int get_prop_dc_voltage_max(struct mmi_discrete_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy) {
			val->intval = -EINVAL;
			return 0;
		}
	}

	if (chg->wls_psy) {
		rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX,
				val);
		if (rc < 0) {
			mmi_err(chg, "Couldn't get VOLTAGE_MAX, rc=%d\n",
					rc);
			return rc;
		}
	}
	return 0;
}

static int set_prop_dc_current_max(struct mmi_discrete_charger *chip,
				    const union power_supply_propval *val)
{
	int rc;

	if (!chip->wls_psy)
		return -EINVAL;

	rc = power_supply_set_property(chip->wls_psy,
				       POWER_SUPPLY_PROP_CURRENT_MAX, val);
	if (rc)
		mmi_err(chip, "Couldn't set dc current max prop rc=%d\n", rc);

	return rc;
}

static int mmi_get_rp_based_dcp_current(struct mmi_discrete_charger *chg, int typec_mode)
{
	int rp_ua;

	switch (typec_mode) {
	case MMI_POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		rp_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case MMI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		rp_ua = TYPEC_MEDIUM_CURRENT_UA;
		break;
	case MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	/* fall through */
	default:
		rp_ua = DCP_CURRENT_UA;
		if (chg->constraint.dcp_pmax > 0)
			rp_ua = (chg->constraint.dcp_pmax / 5) * 1000;
	}

	return rp_ua;
}

static int mmi_discrete_handle_usb_current(struct mmi_discrete_charger *chg,
					int usb_current)
{
	int rc = 0, rp_ua;
	union power_supply_propval val = {0, };

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		if (usb_current == -ETIMEDOUT) {
				/*
				 * Valid FLOAT charger, report the current
				 * based of Rp.
				 */
				rp_ua = mmi_get_rp_based_dcp_current(chg,
								chg->typec_mode);
				rc = vote(chg->usb_icl_votable,
						SW_ICL_MAX_VOTER, true, rp_ua);
				if (rc < 0)
					return rc;
		} else {
			/*
			 * FLOAT charger detected as SDP by USB driver,
			 * charge with the requested current and update the
			 * real_charger_type
			 */
			chg->real_charger_type = POWER_SUPPLY_TYPE_USB;
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
						true, usb_current);
			if (rc < 0)
				return rc;
			rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER,
							false, 0);
			if (rc < 0)
				return rc;
		}
	} else {
		rc = get_prop_usb_present(chg, &val);
		if (!rc && !val.intval)
			return 0;

		if (typec_rp_med_high(chg, chg->typec_mode)) {
			vote(chg->usb_icl_votable, USB_PSY_VOTER,
							false, 0);
			return 0;
		}

		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
							usb_current);
		if (rc < 0) {
			mmi_err(chg, "Couldn't vote ICL USB_PSY_VOTER rc=%d\n", rc);
			return rc;
		}

		rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		if (rc < 0) {
			mmi_err(chg, "Couldn't remove SW_ICL_MAX vote rc=%d\n", rc);
			return rc;
		}

	}

	return 0;
}

static int set_prop_sdp_current_max(struct mmi_discrete_charger *chg,
				    int val)
{
	union power_supply_propval pval;
	int rc = 0;

	if (!chg->pd_active) {
		rc = get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			mmi_err(chg, "Couldn't get usb present rc = %d\n",
						rc);
			return rc;
		}

		/* handle the request only when USB is present */
		if (pval.intval)
			rc = mmi_discrete_handle_usb_current(chg, val);
	}

	return rc;
}

static int is_wls_online(struct mmi_discrete_charger *chip)
{
	int rc;
	union power_supply_propval val;

	if (!chip->wls_psy)
		return 0;

	rc = power_supply_get_property(chip->wls_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		mmi_err(chip, "Error wls online rc = %d\n", rc);
		return 0;
	}

	return val.intval;
}

static int mmi_discrete_update_usb_type(struct mmi_discrete_charger *chip)
{
	int rc = 0;
	int chg_type = 0;

	if (!chip->master_chg_dev)
		return -EINVAL;

	if (is_wls_online(chip)) {
		mmi_info(chip, "wireless charger detected\n");
		chip->real_charger_type = POWER_SUPPLY_TYPE_WIRELESS;

		return 0;
	}

	rc = charger_dev_get_real_charger_type(chip->master_chg_dev, &chg_type);

	if (rc) {
		mmi_err(chip, "Can't get apsd result\n");
		return rc;
	}

	if (chip->qc3p5_detected) {
		chip->real_charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3P5;
	} else {
		chip->real_charger_type = chg_type;
	}

	mmi_info(chip, "APSD=%d PD=%d QC3P5=%d\n",
			chip->real_charger_type, chip->pd_active, chip->qc3p5_detected);
	return rc;
}

int mmi_discrete_set_usb_suspend(struct mmi_discrete_charger *chip, bool suspend)
{
	int rc = 0;
	static int old_current = 0;
	static bool old_suspend = false;

	if (!chip->constraint.factory_mode || !chip->usb_icl_votable)
		return rc;

	pr_info("%s:suspend=%d, old_suspend=%d, old_current=%d\n",
			__func__, suspend, old_suspend, old_current);

	if (suspend == true && old_suspend == false) {
		old_suspend = true;
		old_current = get_effective_result(chip->usb_icl_votable);
		/* vote 0mA when suspended */
		pmic_vote_force_val_set(chip->usb_icl_votable, 0);
		pmic_vote_force_active_set(chip->usb_icl_votable, 1);
	} else if (suspend == false && old_suspend == true) {
		old_suspend = false;
		if (get_effective_result(chip->usb_icl_votable) == 0) {
			pmic_vote_force_val_set(chip->usb_icl_votable, (u32)old_current);
			pmic_vote_force_active_set(chip->usb_icl_votable, 1);
		}
	}

	return rc;
}

static int mmi_discrete_set_dc_suspend(struct mmi_discrete_charger *chip, bool suspend)
{
	//need to set dc suspend
	return 0;
}

int mmi_discrete_is_usb_suspended(struct mmi_discrete_charger *chip)
{
	bool enable = 0;

	if (chip->usb_icl_votable)
		enable = get_effective_result(chip->usb_icl_votable) == 0;

	return enable;
}

static int mmi_discrete_disable_hw_jeita(struct mmi_discrete_charger *chip)
{
	int rc = 0;

	rc = charger_dev_enable_hw_jeita(chip->master_chg_dev, false);

	if (rc)
		mmi_err(chip, "Couldn't disable hw jeita rc=%d\n", rc);

	return rc;
}

int mmi_discrete_otg_enable(struct mmi_discrete_charger *chip, bool en)
{
	int rc = 0;

	if (en == !chip->vbus_enabled) {
		rc = charger_dev_enable_otg(chip->master_chg_dev, en);
		if (rc) {
			mmi_err(chip, "Unable to %s otg (%d)\n", en?"enable":"disable", rc);
			return rc;
		}
		chip->vbus_enabled = en;

		if (chip->bat_ocp_ua) {
			if (en == true)
				schedule_delayed_work(&chip->monitor_ibat_work, msecs_to_jiffies(1000));
			else
				cancel_delayed_work(&chip->monitor_ibat_work);
		}
	}

	return rc;
}

static void mmi_discrete_monitor_ibat_work(struct work_struct *work)
{
	struct mmi_discrete_charger *chip = container_of(work,
				struct mmi_discrete_charger,
				monitor_ibat_work.work);
	union power_supply_propval val = {0, };
	int ret = 0;
	int total_current = 0;
	int average_current = 0;
	int i = 0;

	for (i = 0; i < 3; i++) {
		ret = get_batt_current_now(chip, &val);
		if (ret) {
			val.intval = 0;
		}

		total_current += val.intval;
		val.intval = 0;
		msleep(100);
	}

	average_current = total_current / i;

	if (average_current < 0)
		average_current = average_current * (-1);

	if (average_current > chip->bat_ocp_ua) {
		mmi_discrete_otg_enable(chip, false);
		mmi_info(chip, "battery current %d > %d, disable OTG\n",
					average_current, chip->bat_ocp_ua);
	} else
		schedule_delayed_work(&chip->monitor_ibat_work, msecs_to_jiffies(1000));

}

static void mmi_discrete_check_otg_power(struct mmi_discrete_charger *chip)
{

	if (chip->vbus_enabled) {
		mmi_discrete_otg_enable(chip, false);
		mmi_info(chip, "VBUS OTG Disable due to Charger\n");
	}
}

static void mmi_set_pd20_cap(struct mmi_discrete_charger *chg,
	int pdo, int mv, int ma)
{
	int ret;

	ret = adapter_dev_set_cap(chg->pd_adapter_dev, MMI_PD_FIXED,
			pdo, mv, ma);
	if (ret != MMI_ADAPTER_OK)
		mmi_info(chg, "set pd20 cap fail ret=%d\n", ret);
}

static void mmi_pd30_start(struct mmi_discrete_charger *chg,
	int pdo, int mv, int ma)
{
	int ret;

	ret = adapter_dev_set_cap(chg->pd_adapter_dev, MMI_PD_APDO_START,
		pdo, mv, ma);
	if (ret != MMI_ADAPTER_OK)
		mmi_info(chg, "set pd30 cap start fail ret=%d\n", ret);
}

static void mmi_set_pd30_cap(struct mmi_discrete_charger *chg,
	int pdo, int mv, int ma)
{
	int ret;

	ret = adapter_dev_set_cap(chg->pd_adapter_dev, MMI_PD_APDO,
		pdo, mv, ma);
	if (ret != MMI_ADAPTER_OK)
		mmi_info(chg, "set pd30 cap fail ret=%d\n", ret);
}


#define CHARGER_POWER_18W	18000
#define PD_POWER_MIN		15000
#define PPS_VOLT_MV_MIN		5000
#define PPS_CURR_MA_MIN		2000
#define PD_CURR_MA_MAX		3000
#define MICRO_MV_5V		5000
#define MICRO_MV_9V		9000
#define MICRO_MV_1V		1000
static void mmi_discrete_config_pd_charger(struct mmi_discrete_charger *chg)
{
	int rc = -EINVAL, i =0, req_pdo = -1, req_pd_volt = 0, req_pd_curr = 0;
	int fixed_pd_volt = 0;
	union power_supply_propval val = {0, };
	bool vbus_present = false, pps_active = false, fixed_active = false;
	static bool pps_start = false, fixed_power_done = false;
	struct adapter_power_cap cap;
	int vbus_mv;

	rc = get_prop_usb_present(chg, &val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return;
	}
	vbus_present = val.intval;
	if (chg->pd_active == MMI_POWER_SUPPLY_PD_INACTIVE
		|| !vbus_present) {
		pps_start = false;
		fixed_power_done = false;
		return;
	}

	if (!chg->pd_adapter_dev) {
		chg->pd_adapter_dev = get_adapter_by_name("pd_adapter");
			if (!chg->pd_adapter_dev) {
			mmi_err(chg, "Failed: pd_adapter has not registered yet\n");
			return;
		}
	}

	for (i = 0; i < ADAPTER_CAP_MAX_NR; i++) {
               cap.max_mv[i] = 0;
               cap.min_mv[i] = 0;
               cap.ma[i] = 0;
               cap.type[i] = 0;
               cap.pwr_limit[i] = 0;
       }

	rc = adapter_dev_get_cap(chg->pd_adapter_dev, MMI_PD_ALL, &cap);
	if (MMI_ADAPTER_OK != rc) {
		mmi_err(chg, "Couldn't get pdo rc = %d\n", rc);
		return;
	}

	rc = get_prop_usb_voltage_now(chg, &val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't get usb vbus rc = %d\n", rc);
		return;
	}
	vbus_mv = val.intval;

	for (i = 0; i < cap.nr; i++) {
		if (cap.type[i] == MMI_PD_APDO
			&& cap.max_mv[i] >= PPS_VOLT_MV_MIN
			&& cap.ma[i] >= PPS_CURR_MA_MIN
			&& cap.max_mv[i] * cap.ma[i] / 1000 >= this_chip->constraint.pd_pmax) {
			req_pdo = i;
			req_pd_curr = cap.ma[i];
			req_pd_curr = min(req_pd_curr, PD_CURR_MA_MAX);
			req_pd_volt = (this_chip->constraint.pd_pmax* 1000 / req_pd_curr)  % 20;
			req_pd_volt = (this_chip->constraint.pd_pmax * 1000 / req_pd_curr) - req_pd_volt;
			req_pd_volt = max(req_pd_volt, PPS_VOLT_MV_MIN);
			req_pd_volt = min(req_pd_volt, cap.max_mv[i]);
			pps_active = true;
			mmi_info(chg, "pps active: i=%d,pps_volt=%d, pps_curr=%d\n",
				i, req_pd_volt, req_pd_curr);
			break;
		}
	}

	if (!pps_active) {
		pps_start = false;

		if (this_chip->constraint.pd_pmax < CHARGER_POWER_18W)
			fixed_pd_volt = MICRO_MV_5V;
		else
			fixed_pd_volt = MICRO_MV_9V;

		for (i = 0; i < cap.nr; i++) {
			if ((cap.type[i] == MMI_PD_FIXED)
				&& cap.max_mv[i] <= fixed_pd_volt
				&& cap.max_mv[i] >= MICRO_MV_5V) {
				req_pdo = i;
				req_pd_volt = cap.max_mv[i];
				req_pd_curr =  this_chip->constraint.pd_pmax * 1000 / req_pd_volt;
				req_pd_curr = min(req_pd_curr, cap.ma[i]);
				req_pd_curr = min(req_pd_curr, PD_CURR_MA_MAX);
				fixed_active = true;
				mmi_info(chg, "fixed pdo active:i=%d,fixed_volt=%d, fixed_curr=%d\n",
					i, req_pd_volt, req_pd_curr);
			}
		}

		if (fixed_active && vbus_mv < req_pd_volt - MICRO_MV_1V)
			fixed_power_done = false;
	}

	if (!pps_active && !fixed_active)
		return;

	if (!fixed_power_done && fixed_active) {
		mmi_set_pd20_cap(chg, req_pdo, req_pd_volt, req_pd_curr);
		fixed_power_done =  true;
		mmi_info(chg, "Request fixed power , re-vote USB_ICL %dmA\n", req_pd_curr);
	} else if (pps_active) {
		if (!pps_start) {
			mmi_pd30_start(chg, req_pdo, req_pd_volt, req_pd_curr);
			pps_start = true;
		}
		mmi_set_pd30_cap(chg, req_pdo, req_pd_volt, req_pd_curr);
	}

	/*
	 * Third charging ic IINLIM bits will be changed auto
	 * when BC1.2 done. So we need to Ignore the PD
	 * vote unless BC1.2 done.
	 */
	if (chg->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, req_pd_curr * 1000);
		if (rc < 0)
			mmi_info(chg, "vote PD USB_ICL  failed %duA\n", req_pd_curr);
	} else
		mmi_info(chg, "Ignore PD VOTER ICL before bc1.2\n");

	mmi_info(chg,
			"Request PD power, fixed %d, pps %d, pdo %d, "
			"req volt %dmV, req curr %dmA, vbus %dmV\n",
			fixed_active,
			pps_active,
			req_pdo,
			req_pd_volt,
			req_pd_curr,
			vbus_mv);
	return;
}

#define HVDCP_POWER_MIN			15000
#define HVDCP_VOLTAGE_BASIC		(this_chip->constraint.hvdcp_pmax / 3)
#define HVDCP_VOLTAGE_NOM		(HVDCP_VOLTAGE_BASIC - 200)
#define HVDCP_VOLTAGE_MAX		(HVDCP_VOLTAGE_BASIC + 200)
#define HVDCP_VOLTAGE_MIN		4000
#define HVDCP_PULSE_COUNT_MAX 	((HVDCP_VOLTAGE_BASIC - 5000) / 200 + 2)
static void mmi_discrete_config_qc_charger(struct mmi_discrete_charger *chg)
{
	int rc = -EINVAL;
	int pulse_cnt;
	int vbus_mv;
	bool stepwise;
	union power_supply_propval val = {0, };

	rc = charger_dev_get_pulse_cnt(chg->master_chg_dev, &pulse_cnt);
	if (rc < 0)
		return;

	/*
	 * Two hvdcp voltage regulation algorithm are supported:
	 * stepwise and supplement. stepwise will be used if QC3.0
	 * charger power is more than 15W.
	 * It only runs once for stepwise algorithm after QC3.0
	 * charger is detected, while it will run all the time in
	 * supplement algorithm if QC3.0 charger is present.
	 */
	stepwise = chg->constraint.hvdcp_pmax > HVDCP_POWER_MIN;
	if (stepwise && pulse_cnt)
		return;

	if (stepwise) {
		vote(chg->chg_disable_votable, MMI_HB_VOTER, true, 0);
		mmi_info(chg, "Disable charging before stepwise configure\n");
	}

	do {

		if (chg->real_charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3) {
			mmi_warn(chg, "Exit for QC3.0 charger removal\n");
			break;
		}

		rc = get_prop_usb_voltage_now(chg, &val);
		if (rc < 0)
			break;
		vbus_mv = val.intval / 1000;

		mmi_info(chg, "pulse_cnt=%d, vbus_mv=%d\n", pulse_cnt, vbus_mv);
		if (vbus_mv < HVDCP_VOLTAGE_NOM &&
		    pulse_cnt < HVDCP_PULSE_COUNT_MAX)
			val.intval = MMI_POWER_SUPPLY_DP_DM_DP_PULSE;
		else if (vbus_mv > HVDCP_VOLTAGE_MAX &&
		    pulse_cnt > 0 && !stepwise)
			val.intval = MMI_POWER_SUPPLY_DP_DM_DM_PULSE;
		else {
			mmi_info(chg, "QC3.0 output configure completed\n");
			break;
		}

		rc = charger_dev_set_dp_dm(chg->master_chg_dev, val.intval);
		if (rc < 0)
			break;
		else if (!stepwise) {
			power_supply_changed(chg->usb_psy);
			break;
		}
		msleep(100);

		rc = charger_dev_get_pulse_cnt(chg->master_chg_dev, &pulse_cnt);
		if (rc < 0)
			break;

	} while (stepwise);

	if (stepwise) {
		charger_dev_set_input_voltage(chg->master_chg_dev,
					(HVDCP_VOLTAGE_NOM - 1000)*1000);
		vote(chg->chg_disable_votable, MMI_HB_VOTER, false, 0);
		mmi_info(chg, "Recover charging after stepwise configure\n");
	}
}

#define WLS_POWER_MIN 5000
static void mmi_discrete_config_wls_charger(struct mmi_discrete_charger *chg)
{
	mmi_dbg(chg, "Configure wireless charger\n");
}

static void mmi_discrete_config_charger_input(struct mmi_discrete_charger *chip)
{
	int rc;
	int i;
	bool charger_suspend = false;
	union power_supply_propval val;
	struct mmi_charger_cfg *cfg;

	if (!chip->chg_clients || !chip->chg_client_num) {
		mmi_err(chip, "Invalid charger client\n");
		return;
	}

	for (i = 0; i < chip->chg_client_num; i++) {
		cfg = &chip->chg_clients[i].chg_cfg;
		if (cfg->charger_suspend) {
			charger_suspend = true;
			break;
		}
	}

	if (charger_suspend) {
		if (chip->constraint.factory_mode) {
			mmi_discrete_set_usb_suspend(chip, true);
			mmi_discrete_set_dc_suspend(chip, true);
		} else {
			vote(chip->usb_icl_votable, MMI_HB_VOTER, true, 0);
			vote(chip->dc_suspend_votable, MMI_HB_VOTER, true, 1);
		}
		return;
	} else {
		if (chip->constraint.factory_mode) {
			mmi_discrete_set_usb_suspend(chip, false);
			mmi_discrete_set_dc_suspend(chip, false);
		} else {
			rc = get_prop_dc_present(chip, &val);
			if (!rc && val.intval)
				vote(chip->dc_suspend_votable, MMI_HB_VOTER,
							false, 0);
			else
				vote(chip->dc_suspend_votable, MMI_HB_VOTER,
							true, 1);

			vote(chip->usb_icl_votable, MMI_HB_VOTER, false, 0);
		}
	}

	if (chip->usb_psy &&
	    chip->constraint.pd_pmax >= PD_POWER_MIN)
		mmi_discrete_config_pd_charger(chip);

	if (chip->charger_psy && chip->usb_psy &&
	    chip->constraint.hvdcp_pmax >= HVDCP_POWER_MIN &&
	    chip->chg_info.chrg_mv > HVDCP_VOLTAGE_MIN &&
	    chip->chg_info.chrg_type == POWER_SUPPLY_TYPE_USB_HVDCP_3 &&
	    chip->pd_active == MMI_POWER_SUPPLY_PD_INACTIVE)
		mmi_discrete_config_qc_charger(chip);

	if (chip->wls_psy &&
	    chip->constraint.wls_pmax > WLS_POWER_MIN &&
	    chip->chg_info.chrg_type == POWER_SUPPLY_TYPE_WIRELESS)
		mmi_discrete_config_wls_charger(chip);
}

static void mmi_discrete_config_charger_output(struct mmi_discrete_charger *chip)
{
	int i;
	int target_fcc = INT_MAX;
	int target_fv = INT_MAX;
	bool chg_dis;
	bool charging_reset = false;
	bool charging_disable = false;
	struct mmi_charger_cfg *cfg;

	if (!chip->chg_clients || !chip->chg_client_num) {
		mmi_err(chip, "Invalid charger client\n");
		return;
	}

	chg_dis = get_effective_result(chip->chg_disable_votable);
	for (i = 0; i < chip->chg_client_num; i++) {
		cfg = &chip->chg_clients[i].chg_cfg;
		target_fcc = min(target_fcc, cfg->target_fcc);
		target_fv = min(target_fv, cfg->target_fv);
		if (cfg->charging_disable || target_fcc < 0)
			charging_disable = true;
		if (cfg->charging_reset)
			charging_reset = true;
		if (cfg->taper_kickoff)
			chip->chg_clients[i].chrg_taper_cnt = 0;
	}

	if (charging_reset && !charging_disable && !chg_dis) {
		vote(chip->chg_disable_votable, MMI_HB_VOTER, true, 0);
		mmi_warn(chip, "Charge Halt..Toggle\n");
		msleep(50);
	}

	vote(chip->fv_votable, MMI_HB_VOTER, true, target_fv * 1000);
	if (charging_disable) {
		vote(chip->fcc_votable, MMI_HB_VOTER, true, 0);
		vote(chip->chg_disable_votable, MMI_HB_VOTER, true, 0);
	} else {
		vote(chip->fcc_votable, MMI_HB_VOTER, true, target_fcc * 1000);
		vote(chip->chg_disable_votable, MMI_HB_VOTER, false, 0);
	}

	charging_disable = get_effective_result(chip->chg_disable_votable);
	/* Rerun AICL to recover USB ICL from recharge */
	if (chg_dis != charging_disable || charging_reset) {
		charger_dev_rerun_aicl(chip->master_chg_dev);
		mmi_warn(chip, "need to rerun AICL\n");
	}
}

static void mmi_discrete_charger_work(struct work_struct *work)
{
	struct mmi_discrete_charger *chip = container_of(work,
				struct mmi_discrete_charger,
				charger_work.work);

	pm_stay_awake(chip->dev);
	mmi_dbg(chip, "MMI Discrete Charger Work!\n");

	if (chip->chg_info.chrg_present) {
		mmi_discrete_check_otg_power(chip);
	}
	mmi_discrete_config_charger_input(chip);
	mmi_discrete_config_charger_output(chip);

	mmi_info(chip, "FV=%d, FCC=%d, CHGDIS=%d, USBICL=%d, USBDIS=%d\n",
		get_effective_result(chip->fv_votable),
		get_effective_result(chip->fcc_votable),
		get_effective_result(chip->chg_disable_votable),
		get_effective_result(chip->usb_icl_votable),
		mmi_discrete_is_usb_suspended(chip));
	pm_relax(chip->dev);
}

/************************
 * BATTERY PSY REGISTRATION *
 ************************/
static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int batt_get_prop(struct power_supply *psy,
			 enum power_supply_property psp,
			 union power_supply_propval *val)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = power_supply_get_property(chip->mmi_psy, psp, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = get_prop_batt_present(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = get_prop_batt_charge_type(chip, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = get_prop_input_suspend(chip, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = get_client_vote(chip->fv_votable, BATT_PROFILE_VOTER);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = get_batt_current_now(chip, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = get_effective_result(chip->fcc_votable);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = get_client_vote(chip->fcc_votable, BATT_PROFILE_VOTER);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = get_effective_result(chip->fcc_votable);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		rc = get_prop_system_temp_level_max(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		rc = get_prop_system_temp_level(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = power_supply_get_property(chip->bms_psy, psp, val);
		break;
	default:
		mmi_err(chip, "Get not support prop %d rc = %d\n", psp, rc);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}

	return rc;
}

static int batt_set_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 const union power_supply_propval *val)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		rc = set_prop_system_temp_level(chip, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = set_prop_input_suspend(chip, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		rc = power_supply_set_property(chip->mmi_psy, prop, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval < 0) {
			vote(chip->fcc_votable, USER_VOTER, false, 0);
		} else
			vote(chip->fcc_votable, USER_VOTER, true, val->intval);
		break;
	default:
		mmi_dbg(chip, "Set not support prop %d rc = %d\n", prop, rc);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int batt_prop_is_writeable(struct power_supply *psy,
				  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static void batt_external_power_changed(struct power_supply *psy)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);

	cancel_delayed_work(&chip->charger_work);
	schedule_delayed_work(&chip->charger_work, msecs_to_jiffies(0));

	if (chip->batt_psy)
		power_supply_changed(chip->batt_psy);
}

static const struct power_supply_desc batt_psy_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= batt_get_prop,
	.set_property	= batt_set_prop,
	.external_power_changed = batt_external_power_changed,
	.property_is_writeable = batt_prop_is_writeable,
	.properties	= batt_props,
	.num_properties	= ARRAY_SIZE(batt_props),
};

/************************
 * USB PSY REGISTRATION *
 ************************/
static enum power_supply_property mmi_discrete_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int mmi_discrete_usb_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	val->intval = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		/*when in COM, pd will trigger hard reset to let source send source_cap.
		*but pd hard reset will closed vbus and re-enable vbus by adaptor, so usb
		*present will return 0 to let COM shutdown. Typec mode is always on.*/
		if (chip->pd_supported &&
			(chip->typec_mode == MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
			|| chip->typec_mode == MMI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM
			|| chip->typec_mode == MMI_POWER_SUPPLY_TYPEC_SOURCE_HIGH)) {
			val->intval = 1;
			break;
		}
		rc = get_prop_usb_present(chip, val);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		rc = get_usb_online(chip, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = get_prop_usb_voltage_now(chip, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = get_prop_usb_current_now(chip, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = mmi_discrete_get_hw_current_max(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB_PD;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* USB uses this to set SDP current */
		val->intval = get_client_vote(chip->usb_icl_votable,
					      USB_PSY_VOTER);
		break;
	default:
		pr_err("get prop %d is not supported in usb\n", psp);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}

	return 0;
}

static int mmi_discrete_usb_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = set_prop_sdp_current_max(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval < 0) {
			vote(chip->usb_icl_votable, USER_VOTER, false, 0);
		} else
			vote(chip->usb_icl_votable, USER_VOTER, true, val->intval);
		break;
	default:
		pr_err("Set prop %d is not supported in usb psy\n",
				psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int mmi_discrete_usb_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc usb_psy_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB_PD,
	.properties = mmi_discrete_usb_props,
	.num_properties = ARRAY_SIZE(mmi_discrete_usb_props),
	.get_property = mmi_discrete_usb_get_prop,
	.set_property = mmi_discrete_usb_set_prop,
	.property_is_writeable = mmi_discrete_usb_prop_is_writeable,
};

static int mmi_discrete_init_usb_psy(struct mmi_discrete_charger *chip)
{
	struct power_supply_config usb_cfg = {};

	usb_cfg.drv_data = chip;
	usb_cfg.of_node = chip->dev->of_node;
	chip->usb_psy = devm_power_supply_register(chip->dev,
						  &usb_psy_desc,
						  &usb_cfg);
	if (IS_ERR(chip->usb_psy)) {
		mmi_err(chip, "Couldn't register USB power supply\n");
		return PTR_ERR(chip->usb_psy);
	}

	return 0;
}

/********************************
 * USB PC_PORT PSY REGISTRATION *
 ********************************/
static enum power_supply_property mmi_discrete_usb_port_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int mmi_discrete_usb_port_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		rc = get_usb_port_online(chip, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = get_prop_input_current_settled(chip, val);
		break;
	default:
		mmi_err(chip, "Get prop %d is not supported in pc_port\n",
				psp);
		return -EINVAL;
	}

	if (rc < 0) {
		mmi_dbg(chip, "Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}

	return 0;
}

static int mmi_discrete_usb_port_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	default:
		mmi_err(chip, "Set prop %d is not supported in pc_port\n",
				psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct power_supply_desc usb_port_psy_desc = {
	.name		= "pc_port",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= mmi_discrete_usb_port_props,
	.num_properties	= ARRAY_SIZE(mmi_discrete_usb_port_props),
	.get_property	= mmi_discrete_usb_port_get_prop,
	.set_property	= mmi_discrete_usb_port_set_prop,
};

static int mmi_discrete_init_usb_port_psy(struct mmi_discrete_charger *chip)
{
	struct power_supply_config usb_port_cfg = {};

	usb_port_cfg.drv_data = chip;
	usb_port_cfg.of_node = chip->dev->of_node;
	chip->usb_port_psy = devm_power_supply_register(chip->dev,
						  &usb_port_psy_desc,
						  &usb_port_cfg);
	if (IS_ERR(chip->usb_port_psy)) {
		mmi_err(chip, "Couldn't register USB pc_port power supply\n");
		return PTR_ERR(chip->usb_port_psy);
	}

	return 0;
}

/*************************
 * DC PSY REGISTRATION   *
 *************************/

static enum power_supply_property mmi_discrete_dc_props[] = {
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int mmi_discrete_dc_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* For DC, INPUT_CURRENT_LIMIT equates to INPUT_SUSPEND */
		val->intval = get_effective_result(chip->dc_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = get_prop_dc_present(chip, val);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		rc = get_prop_dc_online(chip, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = get_prop_dc_voltage_now(chip, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = get_prop_dc_current_max(chip, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = get_prop_dc_voltage_max(chip, val);
		break;
	default:
		return -EINVAL;
	}
	if (rc < 0) {
		mmi_dbg(chip,"Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int mmi_discrete_dc_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct mmi_discrete_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = vote(chip->dc_suspend_votable, WBC_VOTER,
			(bool)val->intval, 0);
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = set_prop_dc_current_max(chip, val);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int mmi_discrete_dc_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc dc_psy_desc = {
	.name = "dc",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = mmi_discrete_dc_props,
	.num_properties = ARRAY_SIZE(mmi_discrete_dc_props),
	.get_property = mmi_discrete_dc_get_prop,
	.set_property = mmi_discrete_dc_set_prop,
	.property_is_writeable = mmi_discrete_dc_prop_is_writeable,
};

static int mmi_discrete_init_dc_psy(struct mmi_discrete_charger *chip)
{
	struct power_supply_config dc_cfg = {};

	dc_cfg.drv_data = chip;
	dc_cfg.of_node = chip->dev->of_node;
	chip->dc_psy = devm_power_supply_register(chip->dev,
						  &dc_psy_desc,
						  &dc_cfg);
	if (IS_ERR(chip->dc_psy)) {
		mmi_err(chip, "Couldn't register dc power supply\n");
		return PTR_ERR(chip->dc_psy);
	}

	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int register_dump_read(struct seq_file *m, void *data)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = m->private;

	rc = charger_dev_dump_registers(chip->master_chg_dev, m);

	if (rc)
		mmi_err(chip, "Couldn't dump charger registers\n");

	return rc;
}

static int register_dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct mmi_discrete_charger *chip = inode->i_private;

	return single_open(file, register_dump_read, chip);
}

static const struct file_operations register_dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= register_dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void mmi_discrete_create_debugfs(struct mmi_discrete_charger *chip)
{
	struct dentry *dfs_root, *file;

	dfs_root = debugfs_create_dir("charger_mmi", NULL);
	if (IS_ERR_OR_NULL(dfs_root)) {
		mmi_err(chip, "Couldn't create charger debugfs rc=%ld\n",
			(long)dfs_root);
		return;
	}

	file = debugfs_create_file("register_dump",
			    S_IRUSR | S_IRGRP | S_IROTH,
			    dfs_root, chip, &register_dump_debugfs_ops);
	if (IS_ERR_OR_NULL(file))
		mmi_err(chip, "Couldn't create register_dump file rc=%ld\n",
			(long)file);
}
#else
static void mmi_discrete_create_debugfs(struct mmi_discrete_charger *chip)
{}
#endif

static int chg_disable_vote_callback(struct votable *votable,
		void *data, int chg_disable, const char *client)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = data;

	rc = charger_dev_enable_charging(chip->master_chg_dev,
				chg_disable ? false : true);
	if (rc)
		mmi_err(chip, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);

	return rc;
}

static int dc_suspend_vote_callback(struct votable *votable,
		void *data, int dc_suspend, const char *client)
{
	//need to set dc suspend
	return 0;
}

static int fcc_vote_callback(struct votable *votable, void *data,
			int total_fcc_ua, const char *client)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = data;

	rc = charger_dev_set_charging_current(chip->master_chg_dev, total_fcc_ua);

	if (rc)
		mmi_err(chip, "Couldn't set charging current rc=%d\n", rc);

	return rc;
}

static int fv_vote_callback(struct votable *votable, void *data,
			int fv_uv, const char *client)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = data;

	rc = charger_dev_set_constant_voltage(chip->master_chg_dev, fv_uv);

	if (rc)
		mmi_err(chip, "Couldn't set constant voltage rc=%d\n", rc);

	return rc;
}

static int usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	int rc = 0;
	struct mmi_discrete_charger *chip = data;

	rc = charger_dev_set_input_current(chip->master_chg_dev, icl_ua);

	if (rc)
		mmi_err(chip, "Couldn't set input current rc=%d\n", rc);

	return rc;
}


int mmi_get_prop_typec_cc_orientation(struct mmi_discrete_charger *chg,
					 int *val)
{
	int rc = 0;
	//need to get typec cc orientation
	return rc;
}

static void mmi_handle_hvdcp_check_timeout(struct mmi_discrete_charger *chg,
					      bool hvdcp_done)
{
	u32 hvdcp_ua = 0;

	if (chg->pd_active)
		return;

	if (hvdcp_done) {
		hvdcp_ua = (chg->real_charger_type ==
				POWER_SUPPLY_TYPE_USB_HVDCP) ?
				chg->hvdcp2_max_icl_ua :
				HVDCP_CURRENT_UA;

		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
				hvdcp_ua);
	}

	mmi_dbg(chg, "notify: %s %s\n", __func__,
		   hvdcp_done ? "hvdcp done" : "no hvdcp");
}

static void mmi_notify_extcon_props(struct mmi_discrete_charger *chg, int id)
{
	union extcon_property_value val;
	int prop_val;

	mmi_get_prop_typec_cc_orientation(chg, &prop_val);
	val.intval = ((prop_val == 2) ? 1 : 0);
	extcon_set_property(chg->extcon, id,
			EXTCON_PROP_USB_TYPEC_POLARITY, val);
	val.intval = true;
	extcon_set_property(chg->extcon, id,
			EXTCON_PROP_USB_SS, val);
}

static void mmi_notify_device_mode(struct mmi_discrete_charger *chg, bool enable)
{
	if (enable)
		mmi_notify_extcon_props(chg, EXTCON_USB);

	extcon_set_state_sync(chg->extcon, EXTCON_USB, enable);
}

static void update_sw_icl_max(struct mmi_discrete_charger *chg)
{
	int rp_ua;

	/* while PD is active it should have complete ICL control */
	if (chg->pd_active)
		return;

	if (chg->typec_mode == MMI_POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
		return;
	}

	/*
	 * HVDCP 2/3, handled separately
	 */
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP
			|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
			|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5)
		return;

	/* TypeC rp med or high, use rp value */
	if (typec_rp_med_high(chg, chg->typec_mode)) {
		rp_ua = mmi_get_rp_based_dcp_current(chg, chg->typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		return;
	}

	/* rp-std or legacy, USB BC 1.2 */
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB:
		if (get_client_vote(chg->usb_icl_votable,
				USB_PSY_VOTER) < SDP_CURRENT_UA) {
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
					SDP_CURRENT_UA);
		}
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					CDP_CURRENT_UA);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		rp_ua = mmi_get_rp_based_dcp_current(chg, chg->typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_CURRENT_UA);
		break;
	case POWER_SUPPLY_TYPE_WIRELESS:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					chg->wls_max_icl_ua);
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
	default:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_100_MA);
		break;
	}
}

static void mmi_handle_apsd_done(struct mmi_discrete_charger *chip, bool apsd_done)
{
	if (!apsd_done)
		return;

	mmi_discrete_update_usb_type(chip);

	update_sw_icl_max(chip);

	switch (chip->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB:
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (chip->use_extcon)
			mmi_notify_device_mode(chip, true);
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
	case POWER_SUPPLY_TYPE_USB_DCP:
		break;
	default:
		break;
	}

	mmi_dbg(chip, "notify: apsd-done; %d detected\n",
		   chip->real_charger_type);
}

int mmi_discrete_config_typec_mode(struct mmi_discrete_charger *chip, int val)
{
	mmi_info(chip, "config typec mode is %d old mode is %d\n", val, chip->typec_mode);

	if (val != chip->typec_mode) {
		chip->typec_mode = val;

		/*
		 * Third charging ic IINLIM bits will be changed auto
		 * when BC1.2 done. So we need to Ignore the Rp
		 * changes unless BC1.2 done.
		 */
		if (chip->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN)
			update_sw_icl_max(chip);

		if (chip->usb_psy)
			power_supply_changed(chip->usb_psy);
	}

	return 0;
}

int mmi_discrete_get_typec_accessory_mode(struct mmi_discrete_charger *chip, int *val)
{
	mmi_info(chip, ": %d\n", chip->typec_mode);

	switch (chip->typec_mode) {
	case MMI_POWER_SUPPLY_TYPEC_NONE:
		*val = MMI_POWER_SUPPLY_TYPEC_ACCESSORY_NONE;
		break;
	case MMI_POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
		*val = MMI_POWER_SUPPLY_TYPEC_ACCESSORY_AUDIO;
		break;
	case MMI_POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY:
		*val = MMI_POWER_SUPPLY_TYPEC_ACCESSORY_DEBUG;
		break;
	default:
		*val = -EINVAL;
	}
	return 0;
}

int mmi_discrete_get_hw_current_max(struct mmi_discrete_charger *chip, int *total_current_ua)
{
	int current_ua = 0;

	if (chip->pd_active) {
		*total_current_ua =
			get_client_vote_locked(chip->usb_icl_votable, PD_VOTER);
		return 0;
	}

	/* QC 2.0/3.0 adapter */
	if (chip->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		*total_current_ua = chip->hvdcp2_max_icl_ua;
		return 0;
	} else if (chip->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
			|| chip->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
		*total_current_ua = HVDCP_CURRENT_UA;
		return 0;
	}

	switch (chip->typec_mode) {
	case MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (chip->real_charger_type) {
		case POWER_SUPPLY_TYPE_USB_CDP:
			current_ua = CDP_CURRENT_UA;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			current_ua = DCP_CURRENT_UA;
			break;
		case POWER_SUPPLY_TYPE_USB_FLOAT:
		case POWER_SUPPLY_TYPE_USB:
			current_ua = SDP_CURRENT_UA;
			break;
		case POWER_SUPPLY_TYPE_WIRELESS:
			current_ua = chip->wls_max_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case MMI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		current_ua = TYPEC_MEDIUM_CURRENT_UA;
		break;
	case MMI_POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		current_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case MMI_POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case MMI_POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = current_ua;

	mmi_dbg(chip, "get input current max :%d\n",
	   *total_current_ua);
	return 0;
}

int mmi_discrete_config_charging_enabled(struct mmi_discrete_charger *chip, int val)
{
	int rc = 0;

	mmi_dbg(chip, "mmi_discrete_config_charging_enabled val:%d\n",val);

	rc = charger_dev_enable_charging(chip->master_chg_dev,
				val ? true:false);
	if (rc)
		mmi_err(chip, "Couldn't %s charging rc=%d\n",
			val ? "enable":"disable", rc);

	return rc;
}

int mmi_discrete_config_termination_enabled(struct mmi_discrete_charger *chip, int val)
{
	int rc = 0;

	mmi_dbg(chip, "mmi_discrete_config_termination_enabled val:%d\n",val);

	rc = charger_dev_enable_termination(chip->master_chg_dev,
				val ? true:false);
	if (rc)
		mmi_err(chip, "Couldn't %s termination rc=%d\n",
			val ? "enable":"disable", rc);

	return rc;
}

int mmi_discrete_get_charging_enabled(struct mmi_discrete_charger *chip, bool *val)
{
	int rc = 0;

	rc = charger_dev_is_enabled_charging(chip->master_chg_dev, val);
	if (rc)
		mmi_err(chip, "Couldn't get charging status rc=%d\n", rc);

	mmi_dbg(chip, "mmi_discrete_get_charging_enabled val:%d\n",*val);

	return rc;
}

int mmi_discrete_get_charger_suspend(struct mmi_discrete_charger *chip, bool *val)
{
	int i;
	bool charger_suspend = false;
	struct mmi_charger_cfg *cfg;

	if (!chip->chg_clients || !chip->chg_client_num) {
		mmi_err(chip, "Invalid charger client\n");
		return -1;
	}

	for (i = 0; i < chip->chg_client_num; i++) {
		cfg = &chip->chg_clients[i].chg_cfg;
		if (cfg->charger_suspend) {
			charger_suspend = true;
			break;
		}
	}

	*val = charger_suspend;
	mmi_dbg(chip, "mmi_discrete_get_charger_suspend val:%d\n",*val);

	return 0;
}

int mmi_discrete_get_qc3p_power(struct mmi_discrete_charger *chip, int *val)
{
	int rc = 0;

	rc = charger_dev_get_qc3p_power(chip->master_chg_dev, val);
	if (rc)
		mmi_err(chip, "Couldn't get qc3p power rc=%d\n", rc);

	mmi_dbg(chip, "mmi_discrete_get_qc3p_power val:%d\n",*val);

	return rc;
}

int mmi_discrete_get_pulse_cnt(struct mmi_discrete_charger *chip, int *count)
{
	int rc = 0;

	rc = charger_dev_get_pulse_cnt(chip->master_chg_dev, count);
	if (rc)
		mmi_err(chip, "Couldn't get pulse cnt rc=%d\n", rc);

	mmi_dbg(chip, "mmi_discrete_get_pulse_cnt count:%d\n",*count);

	return rc;
}

int mmi_discrete_set_dp_dm(struct mmi_discrete_charger *chip, int val)
{
	int rc = 0;

	rc = charger_dev_set_dp_dm(chip->master_chg_dev, val);
	if (rc)
		mmi_err(chip, "Couldn't set dp dm rc=%d\n", rc);

	mmi_dbg(chip, "mmi_discrete_set_dp_dm val:%d\n",val);

	return rc;
}

int mmi_discrete_config_input_current_settled(struct mmi_discrete_charger *chip, int val)
{

	update_sw_icl_max(chip);

	mmi_dbg(chip, "mmi_discrete_config_input_current_settled val:%d\n",val);

	vote(chip->usb_icl_votable, QC3P_VOTER, true, val*1000);
	if (chip->usb_psy)
		power_supply_changed(chip->usb_psy);
	return 0;
}

#define MMI_USBIN_500MA	500000
int mmi_discrete_config_pd_active(struct mmi_discrete_charger *chip, int val)
{
	mmi_info(chip, "config pd active is %d old mode is %d\n", val, chip->pd_active);

	if (chip->pd_active && chip->pd_active == val)
		return 0;

	chip->pd_active = val;
	update_sw_icl_max(chip);

	if (chip->pd_active) {
		vote(chip->usb_icl_votable, PD_VOTER, true, MMI_USBIN_500MA);
		vote(chip->usb_icl_votable, USB_PSY_VOTER, false, 0);
		vote(chip->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
	} else {
		vote(chip->usb_icl_votable, PD_VOTER, false, 0);

	}

	if (chip->usb_psy)
		power_supply_changed(chip->usb_psy);

	return 0;
}
static int mmi_discrete_usb_plugout(struct mmi_discrete_charger * chip)
{
	chip->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->pd_active = MMI_POWER_SUPPLY_PD_INACTIVE;
	if (chip->use_extcon)
		mmi_notify_device_mode(chip, false);
	vote(chip->usb_icl_votable, SW_ICL_MAX_VOTER, true, SDP_100_MA);
	vote(chip->usb_icl_votable, USB_PSY_VOTER, false, 0);
	vote(chip->usb_icl_votable, PD_VOTER, false, 0);
	vote(chip->usb_icl_votable, QC3P_VOTER, false, 0);
	vote(chip->chg_disable_votable, MMI_HB_VOTER, true, 0);
	power_supply_changed(chip->usb_psy);
	mmi_info(chip, "%s\n",__func__);
	return 0;
}

static int usb_source_change_notify_handler(struct notifier_block *nb, unsigned long event,
				void *v)
{
	struct mmi_discrete_charger *chip =
			container_of(nb, struct mmi_discrete_charger, master_chg_nb);
	struct chgdev_notify *data = v;
	union power_supply_propval pval;
	int rc = 0;

	rc = get_prop_usb_present(chip, &pval);
	if (rc < 0) {
		mmi_err(chip, "Couldn't get usb present rc = %d\n", rc);
		return NOTIFY_DONE;
	}

	if (!pval.intval) {
		mmi_discrete_usb_plugout(chip);
		return NOTIFY_DONE;
	}

	mmi_handle_apsd_done(chip, data->apsd_done);

	mmi_handle_hvdcp_check_timeout(chip, data->hvdcp_done);

	power_supply_changed(chip->usb_psy);

	mmi_dbg(chip, "apsd done %d, hvdcp done %d\n",
				data->apsd_done, data->hvdcp_done);
	return NOTIFY_DONE;
}

static int mmi_discrete_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
        int rc;
	union power_supply_propval pval;
       struct mmi_discrete_chg_client *chg = data;
	struct mmi_discrete_charger *chip = chg->chip;
	struct power_supply *client_batt_psy;

	if (!chip->charger_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->charger_psy,
					POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Status rc = %d\n", rc);
	} else if (chg->chg_cfg.full_charged)
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
	else
		chg->batt_info.batt_status = pval.intval;

	if (chg->client_batt_psy == chip->batt_psy)
		client_batt_psy = chip->bms_psy;
	else
		client_batt_psy = chg->client_batt_psy;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Voltage rc = %d\n", rc);
	} else
		chg->batt_info.batt_mv = pval.intval / 1000;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Current rc = %d\n", rc);
	} else
		chg->batt_info.batt_ma = pval.intval / 1000;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Capacity rc = %d\n", rc);
	} else
		chg->batt_info.batt_soc = pval.intval;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Temperature rc = %d\n", rc);
	} else
		chg->batt_info.batt_temp = pval.intval / 10;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt charge_full rc = %d\n", rc);
	} else
		chg->batt_info.batt_full_uah = pval.intval;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt charge_full_design rc = %d\n", rc);
	} else
		chg->batt_info.batt_design_uah = pval.intval;

        memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));

        return rc;
}

static int mmi_discrete_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
       int rc;
	int usb_type;
	int usb_icl;
	int vbus;
	union power_supply_propval val;
	struct mmi_discrete_chg_client *chg = data;
	struct mmi_discrete_charger *chip = chg->chip;

	/* USB charger */
	val.intval = 0;
	rc = get_prop_usb_present(chip, &val);
	if (rc < 0)
		mmi_err(chip, "Couldn't read usb present rc=%d\n", rc);
	else if (val.intval) {
		chip->chg_info.chrg_present = 1;
		val.intval = 0;
		rc = get_prop_usb_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read usb voltage rc=%d\n", rc);
		chip->chg_info.chrg_mv = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_usb_current_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read usb current rc=%d\n", rc);
		chip->chg_info.chrg_ma = val.intval / 1000;

		chip->chg_info.chrg_type = chip->real_charger_type;

		usb_type = chip->chg_info.chrg_type;
		if (usb_type == POWER_SUPPLY_TYPE_USB)
			chip->chg_info.chrg_pmax_mw = 2500;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_CDP)
			chip->chg_info.chrg_pmax_mw = 7500;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_DCP)
			chip->chg_info.chrg_pmax_mw = chip->constraint.dcp_pmax;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_HVDCP)
			chip->chg_info.chrg_pmax_mw = 7500;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
			chip->chg_info.chrg_pmax_mw = chip->constraint.hvdcp_pmax;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5)
			chip->chg_info.chrg_pmax_mw = 30000;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_PD)
			chip->chg_info.chrg_pmax_mw = chip->constraint.pd_pmax;
		else if (usb_type == POWER_SUPPLY_TYPE_WIRELESS)
			chip->chg_info.chrg_pmax_mw = chip->constraint.wls_pmax;
		else
			chip->chg_info.chrg_pmax_mw = 2500;

		usb_icl = get_effective_result(chip->usb_icl_votable);
		val.intval = 0;
		rc = get_prop_usb_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read usb voltage rc=%d\n", rc);
		vbus = (val.intval /1000 + 500) /1000;
		if ((usb_type != POWER_SUPPLY_TYPE_USB) && (usb_type != POWER_SUPPLY_TYPE_USB_CDP)) {
			if (chip->chg_info.chrg_pmax_mw < (usb_icl * vbus / 1000))
				chip->chg_info.chrg_pmax_mw = usb_icl * vbus / 1000;
		}

		rc = 0;
		goto completed;
	}

	/* DC charger */
	val.intval = 0;
	rc = get_prop_dc_present(chip, &val);
	if (rc < 0)
		mmi_err(chip, "Couldn't read dc present rc=%d\n", rc);
	else if (val.intval) {
		chip->chg_info.chrg_present = 1;
		val.intval = 0;
		rc = get_prop_dc_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read dc voltage rc=%d\n", rc);
		chip->chg_info.chrg_mv = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_dc_current_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read dc current rc=%d\n", rc);
		chip->chg_info.chrg_ma = val.intval / 1000;

		chip->chg_info.chrg_type = POWER_SUPPLY_TYPE_WIRELESS;
		chip->chg_info.chrg_pmax_mw = chip->constraint.wls_pmax;

		rc = 0;
		goto completed;
	}

	/* Wireless charger */
	if (is_wls_online(chip)) {
		chip->chg_info.chrg_present = 1;
		val.intval = 0;
		rc = get_prop_dc_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read wls voltage rc=%d\n", rc);
		chip->chg_info.chrg_mv = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_dc_current_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read wls current rc=%d\n", rc);
		chip->chg_info.chrg_ma = val.intval / 1000;

		chip->chg_info.chrg_type = POWER_SUPPLY_TYPE_WIRELESS;
		chip->chg_info.chrg_pmax_mw = chip->constraint.wls_pmax;

		rc = 0;
		goto completed;
	}

        chip->chg_info.chrg_mv = 0;
        chip->chg_info.chrg_ma = 0;
	chip->chg_info.chrg_type = 0;
	chip->chg_info.chrg_pmax_mw = 0;
	chip->chg_info.chrg_present = 0;

completed:
	memcpy(chg_info, &chip->chg_info, sizeof(struct mmi_charger_info));

        return rc;
}

static int mmi_discrete_config_charge(void *data, struct mmi_charger_cfg *config)
{
	struct mmi_discrete_chg_client *chg = data;

	memcpy(&chg->chg_cfg, config, sizeof(struct mmi_charger_cfg));
	cancel_delayed_work(&chg->chip->charger_work);
	schedule_delayed_work(&chg->chip->charger_work,
				msecs_to_jiffies(0));

	return 0;
}

#define TAPER_COUNT 2
#define TAPER_DROP_MA 100
static bool mmi_discrete_has_current_tapered(void *data, int taper_ma)
{
	bool change_state = false;
	int allowed_fcc, target_ma;
	int batt_ma;
	struct mmi_discrete_chg_client *chg = data;

	if (!chg) {
		mmi_err(chg->chip, "Charger chip is unavailable!\n");
		return false;
	}

	allowed_fcc = get_effective_result(chg->chip->fcc_votable) / 1000;

	if (allowed_fcc >= taper_ma)
		target_ma = taper_ma;
	else
		target_ma = allowed_fcc - TAPER_DROP_MA;

	batt_ma = chg->batt_info.batt_ma;
	if (batt_ma < 0) {
		batt_ma *= -1;
		if (batt_ma <= target_ma)
			if (chg->chrg_taper_cnt >= TAPER_COUNT) {
				change_state = true;
				chg->chrg_taper_cnt = 0;
			} else
				chg->chrg_taper_cnt++;
		else
			chg->chrg_taper_cnt = 0;
	} else {
		if (chg->chrg_taper_cnt >= TAPER_COUNT) {
			change_state = true;
			chg->chrg_taper_cnt = 0;
		} else
			chg->chrg_taper_cnt++;
	}

	return change_state;
}

static bool mmi_discrete_charge_halted(void *data)
{
	bool flag = false;
	int rc = 0;
	struct mmi_discrete_chg_client *chg = data;
	struct mmi_discrete_charger *chip = chg->chip;

	rc = charger_dev_is_charge_halted(chip->master_chg_dev, &flag);

	if (rc)
		mmi_err(chip, "Couldn't get if charge halted rc =%d\n",rc);

	return flag;
}

static void mmi_discrete_set_constraint(void *data,
			struct mmi_charger_constraint *constraint)
{
	struct mmi_discrete_chg_client *chg = data;

	if (memcmp(constraint, &chg->chip->constraint,
		sizeof(struct mmi_charger_constraint))) {
		memcpy(&chg->chip->constraint, constraint,
			sizeof(struct mmi_charger_constraint));
		cancel_delayed_work(&chg->chip->charger_work);
		schedule_delayed_work(&chg->chip->charger_work,
			msecs_to_jiffies(0));
	}
}

static int mmi_discrete_charger_init(struct mmi_discrete_charger *chip)
{
	int rc;
	int i;
	int count;
	const char *str = NULL;
	struct power_supply *batt_psy;
	struct mmi_charger_driver *driver;
	struct mmi_discrete_chg_client *client;
	struct device_node *np = chip->dev->of_node;
	const char *batt_sn = NULL;
	const char *df_sn = NULL;

	chip->constraint.factory_mode = mmi_is_factory_mode();

	count = of_property_count_strings(np, "battery-names");
	if (count <= 0) {
		mmi_err(chip, "Invalid battery-names in dt, count=%d\n", count);
		return -EINVAL;
	}

	client = devm_kzalloc(chip->dev,
				sizeof(struct mmi_discrete_chg_client) * count,
				GFP_KERNEL);
	driver = devm_kzalloc(chip->dev,
				sizeof(struct mmi_charger_driver) * count,
				GFP_KERNEL);
	if (!client || !driver) {
		rc = -ENOMEM;
		goto free_mem;
	}

	batt_sn = mmi_get_battery_serialnumber();
	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(np, "battery-names", i, &str);
		if (rc)
			break;
		batt_psy = power_supply_get_by_name(str);
		if (!batt_psy) {
			rc = -ENODEV;
			break;
		}

		client[i].chip = chip;
		client[i].driver = &driver[i];
		client[i].client_batt_psy = batt_psy;

		driver[i].name = str;
		driver[i].dev = batt_psy->dev.parent;
		driver[i].data = &client[i];
		driver[i].get_batt_info = mmi_discrete_get_batt_info;
		driver[i].get_chg_info = mmi_discrete_get_chg_info;
		driver[i].config_charge = mmi_discrete_config_charge;
		driver[i].is_charge_tapered = mmi_discrete_has_current_tapered;
		driver[i].is_charge_halt = mmi_discrete_charge_halted;
		driver[i].set_constraint = mmi_discrete_set_constraint;

		if (!batt_sn) {
			df_sn = "unknown-sn";
			of_property_read_string(driver[i].dev->of_node,
						"mmi,df-serialnum", &df_sn);
			strcpy(client[i].batt_info.batt_sn, df_sn);
		} else {
			strcpy(client[i].batt_info.batt_sn, batt_sn);
		}
		mmi_info(chip, "%s Serial Number is %s\n",
					str, client[i].batt_info.batt_sn);

		rc = mmi_register_charger_driver(&driver[i]);
		if (rc)
			break;

		power_supply_changed(client[i].client_batt_psy);
		mmi_info(chip, "charger driver on %s init successfully\n",
					driver[i].name);
	}

	chip->chg_client_num = i;
	if (!chip->chg_client_num)
		goto free_mem;

	chip->chg_clients = client;

	if (chip->master_chg_dev) {
		chip->master_chg_nb.notifier_call = usb_source_change_notify_handler;
		register_charger_device_notifier(chip->master_chg_dev,
						&chip->master_chg_nb);
	} else {
		goto free_mem;
	}

	if (chip->constraint.factory_mode) {
		mmi_info(chip, "Entering Factory Mode !\n");

		if(chip->chg_disable_votable) {
			pmic_vote_force_val_set(chip->chg_disable_votable, 1);
			pmic_vote_force_active_set(chip->chg_disable_votable, 1);
		}
		if(chip->fcc_votable) {
			pmic_vote_force_val_set(chip->fcc_votable, 3000000);
			pmic_vote_force_active_set(chip->fcc_votable, 1);
		}
		if(chip->fv_votable) {
			pmic_vote_force_val_set(chip->fv_votable, 4400000);
			pmic_vote_force_active_set(chip->fv_votable, 1);
		}
		if (chip->usb_icl_votable) {
			pmic_vote_force_val_set(chip->usb_icl_votable, 3000000);
			pmic_vote_force_active_set(chip->usb_icl_votable, 1);
		}

		mmi_discrete_create_factory_testcase(chip);
	}

	return 0;
free_mem:
	if (client)
		devm_kfree(chip->dev, client);
	if (driver)
		devm_kfree(chip->dev, driver);
	chip->chg_client_num = 0;
	chip->chg_clients = NULL;

	return rc;
}

static void mmi_discrete_charger_deinit(struct mmi_discrete_charger *chip)
{
	int i;

	for (i = 0; i < chip->chg_client_num; i++) {
		mmi_unregister_charger_driver(chip->chg_clients[i].driver);
		power_supply_put(chip->chg_clients[i].client_batt_psy);
	}

	if (chip->chg_client_num) {
		devm_kfree(chip->dev, chip->chg_clients->driver);
		devm_kfree(chip->dev, chip->chg_clients);
	}
}

static int mmi_discrete_check_battery_supplies(struct mmi_discrete_charger *chip)
{
	int i;
	int rc = 0;
	int count;

	if (chip->batt_psy->num_supplies && chip->batt_psy->supplied_from)
		goto print_supplies;

	count = of_property_count_strings(chip->dev->of_node, "supplied-from");
	if (count <= 0) {
		mmi_err(chip, "Invalid supplied-from in DT, rc=%d\n", count);
		return -EINVAL;
	}

	chip->batt_psy->supplied_from = devm_kmalloc_array(&chip->batt_psy->dev,
						count,
						sizeof(char *),
						GFP_KERNEL);
	if (!chip->batt_psy->supplied_from)
		return -ENOMEM;

	rc = of_property_read_string_array(chip->dev->of_node, "supplied-from",
				(const char **)chip->batt_psy->supplied_from,
				count);
	if (rc < 0) {
		mmi_err(chip, "Failed to get supplied-from, rc=%d\n", rc);
		return rc;
	}

	chip->batt_psy->num_supplies = count;

print_supplies:
	for (i = 0; i < chip->batt_psy->num_supplies; i++) {
		mmi_info(chip, "battery supplied_from[%d]=%s\n", i,
					chip->batt_psy->supplied_from[i]);
	}

	return 0;
}

static int mmi_discrete_create_votables(struct mmi_discrete_charger *chip)
{
	int rc = 0;

	chip->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					chg_disable_vote_callback,
					chip);
	if (IS_ERR(chip->chg_disable_votable)) {
		rc = PTR_ERR(chip->chg_disable_votable);
		chip->chg_disable_votable = NULL;
		return rc;
	}

	chip->fcc_votable = create_votable("FCC", VOTE_MIN,
					fcc_vote_callback,
					chip);
	if (IS_ERR(chip->fcc_votable)) {
		rc = PTR_ERR(chip->fcc_votable);
		chip->fcc_votable = NULL;
		return rc;
	}

	chip->fv_votable = create_votable("FV", VOTE_MIN,
					fv_vote_callback,
					chip);
	if (IS_ERR(chip->fv_votable)) {
		rc = PTR_ERR(chip->fv_votable);
		chip->fv_votable = NULL;
		return rc;
	}

	chip->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					usb_icl_vote_callback,
					chip);
	if (IS_ERR(chip->usb_icl_votable)) {
		rc = PTR_ERR(chip->usb_icl_votable);
		chip->usb_icl_votable = NULL;
		return rc;
	}

	chip->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					dc_suspend_vote_callback,
					chip);
	if (IS_ERR(chip->dc_suspend_votable)) {
		rc = PTR_ERR(chip->dc_suspend_votable);
		chip->dc_suspend_votable = NULL;
		return rc;
	}

	return 0;
}

static void mmi_discrete_destroy_votables(struct mmi_discrete_charger *chip)
{
	if (chip->chg_disable_votable)
		destroy_votable(chip->chg_disable_votable);
	if (chip->fv_votable)
		destroy_votable(chip->fv_votable);
	if (chip->fcc_votable)
		destroy_votable(chip->fcc_votable);
	if (chip->usb_icl_votable)
		destroy_votable(chip->usb_icl_votable);
	if (chip->dc_suspend_votable)
		destroy_votable(chip->dc_suspend_votable);
}

static int mmi_discrete_init(struct mmi_discrete_charger *chip)
{
	int rc = 0;

	rc = mmi_discrete_create_votables(chip);
	if (rc < 0) {
		mmi_err(chip, "Couldn't create votables rc=%d\n",
			rc);
		return rc;
	}

	return 0;
}
static int mmi_discrete_deinit(struct mmi_discrete_charger *chip)
{
	mmi_discrete_destroy_votables(chip);

	return 0;
}

static int mmi_discrete_extcon_init(struct mmi_discrete_charger *chip)
{
	int rc;

	/* extcon registration */
	chip->extcon = devm_extcon_dev_allocate(chip->dev, mmi_discrete_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		rc = PTR_ERR(chip->extcon);
		mmi_err(chip, "failed to allocate extcon device rc=%d\n",
				rc);
		return rc;
	}

	rc = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (rc < 0) {
		mmi_err(chip, "failed to register extcon device rc=%d\n",
				rc);
		return rc;
	}

	/* Support reporting polarity and speed via properties */
	rc = extcon_set_property_capability(chip->extcon,
			EXTCON_USB, EXTCON_PROP_USB_TYPEC_POLARITY);
	rc |= extcon_set_property_capability(chip->extcon,
			EXTCON_USB, EXTCON_PROP_USB_SS);
	rc |= extcon_set_property_capability(chip->extcon,
			EXTCON_USB_HOST, EXTCON_PROP_USB_TYPEC_POLARITY);
	rc |= extcon_set_property_capability(chip->extcon,
			EXTCON_USB_HOST, EXTCON_PROP_USB_SS);
	if (rc < 0)
		mmi_err(chip,
			"failed to configure extcon capabilities\n");

	return rc;
}

static int mmi_discrete_init_hw(struct mmi_discrete_charger *chip)
{
	int rc = 0;

	/* set OTG current limit */
	rc = charger_dev_set_boost_current_limit(chip->master_chg_dev,
			chip->otg_cl_ua);
	if (rc < 0) {
		mmi_err(chip, "Couldn't set otg current limit rc=%d\n", rc);
		return rc;
	}

	vote(chip->fcc_votable,
		BATT_PROFILE_VOTER, chip->batt_profile_fcc_ua > 0,
		chip->batt_profile_fcc_ua);
	vote(chip->fv_votable,
		BATT_PROFILE_VOTER, chip->batt_profile_fv_uv > 0,
		chip->batt_profile_fv_uv);

	/* Some h/w limit maximum supported ICL */
	vote(chip->usb_icl_votable, HW_LIMIT_VOTER,
			chip->hw_max_icl_ua > 0, chip->hw_max_icl_ua);

	/* enable the charging path */
	rc = vote(chip->chg_disable_votable, DEFAULT_VOTER, false, 0);
	if (rc < 0) {
		mmi_err(chip, "Couldn't enable charging rc=%d\n", rc);
		return rc;
	}

	/* Ensure HW JEITA is DISABLED */
	mmi_info(chip, "DISABLE all HW JEITA\n");
	mmi_discrete_disable_hw_jeita(chip);

	if (chip->dc_cl_ma >= 0) {
		//need to set dc icl
		mmi_err(chip, "need to set dc icl\n");
	}

	return 0;
}

static int mmi_discrete_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct mmi_discrete_charger *chip;
	struct iio_dev *indio_dev;
	struct power_supply_config psy_cfg = {};

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	if (!chip) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for mmi_discrete_iio\n");
		return -ENOMEM;
	}

	chip->indio_dev = indio_dev;

	chip->dev = &pdev->dev;
	chip->name = "mmi_discrete";
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;
	platform_set_drvdata(pdev, chip);
	this_chip = chip;
	device_init_wakeup(chip->dev, true);

	chip->master_chg_dev = get_charger_by_name("master_chg");
	if (chip->master_chg_dev) {
		mmi_info(chip, "Found master charger\n");
		charger_dev_set_drvdata(chip->master_chg_dev, chip);
	} else {
		mmi_err(chip, "*** Error : can't find master charger ***\n");
		return -EPROBE_DEFER;
	}

	chip->mmi_psy = power_supply_get_by_name("mmi_battery");
	if (!chip->mmi_psy) {
		mmi_err(chip, "Failed: mmi_psy has not registered yet\n");
		return -EPROBE_DEFER;
	}

	chip->bms_psy = power_supply_get_by_name("bms");
	if (!chip->bms_psy) {
		mmi_err(chip, "Failed: bms_psy has not registered yet\n");
		return -EPROBE_DEFER;
	}

	chip->charger_psy = power_supply_get_by_name("charger");
	if (!chip->charger_psy) {
		mmi_err(chip, "Failed: charger_psy has not registered yet\n");
		return -EPROBE_DEFER;
	}

	chip->wls_psy = power_supply_get_by_name("wireless");

	rc = mmi_discrete_init_iio_psy(chip, pdev);
	if (rc < 0) {
		mmi_err(chip, "Failed to init iio psy\n");
		return rc;
	}

	rc = mmi_discrete_parse_dts(chip);
	if (rc < 0) {
		mmi_err(chip, "Failed to parse dts\n");
		return rc;
	}

	rc = mmi_discrete_init(chip);
	if (rc < 0) {
		mmi_err(chip, "mmi_discrete_init Failed rc = %d\n", rc);
		goto cleanup;
	}

	rc = mmi_discrete_extcon_init(chip);
	if (rc < 0) {
		mmi_err(chip, "mmi_discrete_extcon_init Failed rc = %d\n", rc);
		goto cleanup;
	}

	rc = mmi_discrete_init_hw(chip);
	if (rc < 0) {
		mmi_err(chip, "mmi_discrete_init_hw Failed rc = %d\n", rc);
		goto cleanup;
	}

	chip->debug_enabled = &debug_enabled;
	chip->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chip->ipc_log)
		mmi_err(chip, "Failed to create MMI DISCRETE IPC log\n");
	else
		mmi_info(chip, "IPC logging is enabled for MMI DISCRETE\n");

	INIT_DELAYED_WORK(&chip->charger_work, mmi_discrete_charger_work);
	INIT_DELAYED_WORK(&chip->monitor_ibat_work, mmi_discrete_monitor_ibat_work);

	chip->batt_psy = devm_power_supply_register(chip->dev,
						    &batt_psy_desc,
						    &psy_cfg);
	if (IS_ERR(chip->batt_psy)) {
		mmi_err(chip,
			"Failed: batt power supply register\n");
		rc = PTR_ERR(chip->batt_psy);
		goto cleanup;
	}

	rc = mmi_discrete_check_battery_supplies(chip);
	if (rc) {
		mmi_err(chip, "No valid battery supplies\n");
		goto cleanup;
	}

	rc = mmi_discrete_init_usb_psy(chip);
	if (rc) {
		mmi_err(chip, "init usb psy error\n");
		goto cleanup;
	}

	rc = mmi_discrete_init_usb_port_psy(chip);
	if (rc < 0) {
		mmi_err(chip, "Couldn't initialize usb pc_port psy rc=%d\n", rc);
		goto cleanup;
	}

	rc = mmi_discrete_init_dc_psy(chip);
	if (rc < 0) {
		mmi_err(chip, "Couldn't initialize dc psy rc=%d\n", rc);
		goto cleanup;
	}

	mmi_discrete_charger_init(chip);

	usb_source_change_notify_handler(&chip->master_chg_nb, 0, &chip->master_chg_dev->noti);

	mmi_discrete_create_debugfs(chip);

	mmi_info(chip, "MMI Discrete Charger Core probed successfully!\n");

	return rc;

cleanup:
	mmi_discrete_deinit(chip);
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int mmi_discrete_remove(struct platform_device *pdev)
{
	struct mmi_discrete_charger *chip = platform_get_drvdata(pdev);

	cancel_delayed_work(&chip->charger_work);
	mmi_discrete_charger_deinit(chip);

	if (chip->mmi_psy)
		power_supply_put(chip->mmi_psy);
	if (chip->bms_psy)
		power_supply_put(chip->bms_psy);
	if (chip->charger_psy)
		power_supply_put(chip->charger_psy);
	if (chip->wls_psy)
		power_supply_put(chip->wls_psy);

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,mmi-discrete-charger", },
	{ },
};

static struct platform_driver mmi_discrete_driver = {
	.driver		= {
		.name		= "mmi,mmi-discrete-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe		= mmi_discrete_probe,
	.remove		= mmi_discrete_remove,
};
module_platform_driver(mmi_discrete_driver);

MODULE_DESCRIPTION("MMI Discrete Charger Policy Driver");
MODULE_LICENSE("GPL v2");

