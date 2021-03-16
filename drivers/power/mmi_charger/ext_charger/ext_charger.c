/*
 * Copyright (C) 2021 Motorola Mobility LLC
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

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include "mmi_charger.h"

#define BATT_SN_UNKNOWN "unknown-sn"

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for ext charger driver");

struct charger_profile_info {
	int fg_iterm;
	int chrg_iterm;
	int max_fv_uv;
	int max_fcc_ua;
	int vfloat_comp_uv;
	int demo_fv_uv;
	int data_bk_size; /* number of byte for each data block */
	int data_size; /* number of byte for while data */
	int profile_id; /* profile id for charging profile selection in ADSP */
};

struct ext_mmi_chg_t {
	char				*name;
	struct device			*dev;
	bool			*debug_enabled;
	u32				chrg_taper_cnt;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_info		chg_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_constraint	constraint;
	struct mmi_charger_driver	*driver;
	u32				*profile_data;
	struct charger_profile_info	profile_info;
	void				*ipc_log;
	struct power_supply	*fg_psy;
	struct power_supply	*chg_psy;
	struct power_supply	*usb_psy;
};

static int get_ps_int_prop(struct power_supply *psy, enum power_supply_property prop)
{
	int rc;
	union power_supply_propval val;

	if (!psy)
		return -1;

	rc = power_supply_get_property(psy, prop, &val);
	if (rc < 0) {
		pr_err("Error getting prop %d rc = %d\n", prop, rc);
		return -1;
	}

	return val.intval;
}

static int set_ps_int_prop(struct power_supply *psy, enum power_supply_property prop, int value)
{
	int rc;
	union power_supply_propval val;

	if (!psy)
		return -1;

	val.intval = value;

	rc = power_supply_set_property(psy, prop, &val);
	if (rc < 0) {
		pr_err("Error getting prop %d rc = %d\n", prop, rc);
		return -1;
	}

	return 0;
}

static int ext_mmi_chg_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
	struct ext_mmi_chg_t *chg = data;

	if (!chg || !chg->fg_psy)
		return -1;

	chg->batt_info.batt_mv = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	chg->batt_info.batt_ma = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	chg->batt_info.batt_soc = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_CAPACITY);
	chg->batt_info.batt_temp = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_TEMP);
	chg->batt_info.batt_status = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_STATUS);
	chg->batt_info.batt_full_uah = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL);
	chg->batt_info.batt_design_uah = get_ps_int_prop(chg->fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);

	if (chg->chg_cfg.full_charged)
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;

	chg->batt_info.batt_ma /= 1000;
	chg->batt_info.batt_mv /= 1000;
	chg->batt_info.batt_temp /= 10;
	memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));

	return 0;
}

static int ext_mmi_chg_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
	int rc = 0;
	struct ext_mmi_chg_t *chg = data;
	int current_max, voltage_max;

	chg->chg_info.chrg_mv = get_ps_int_prop(chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	chg->chg_info.chrg_ma = get_ps_int_prop(chg->usb_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	chg->chg_info.chrg_type = get_ps_int_prop(chg->usb_psy, POWER_SUPPLY_PROP_USB_TYPE);
	chg->chg_info.chrg_present = get_ps_int_prop(chg->usb_psy, POWER_SUPPLY_PROP_ONLINE);

	current_max = get_ps_int_prop(chg->usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT);
	voltage_max = get_ps_int_prop(chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX);

	if (current_max > 0 && voltage_max > 0)
		chg->chg_info.chrg_pmax_mw = ((current_max/1000) * (voltage_max/1000)) / 1000;
	else
		chg->chg_info.chrg_pmax_mw = 0;

	chg->chg_info.chrg_mv /= 1000;
	chg->chg_info.chrg_ma /= 1000;
	memcpy(chg_info, &chg->chg_info, sizeof(struct mmi_charger_info));

	return rc;
}

static int ext_mmi_chg_config_charge(void *data, struct mmi_charger_cfg *config)
{
	int rc = 0;
	u32 value;
	struct ext_mmi_chg_t *chg = data;

	/* configure the charger if changed */
	if (config->target_fv != chg->chg_cfg.target_fv) {
		value = config->target_fv * 1000;
		rc = set_ps_int_prop(chg->chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX, value);
		if (!rc)
			chg->chg_cfg.target_fv = config->target_fv;
	}
	if (config->target_fcc != chg->chg_cfg.target_fcc) {
		value = config->target_fcc * 1000;
		rc = set_ps_int_prop(chg->chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, value);
		if (!rc)
			chg->chg_cfg.target_fcc = config->target_fcc;
	}
	if (config->charger_suspend != chg->chg_cfg.charger_suspend) {
		value = config->charger_suspend;
		/* TODO write suspend, how is it different than disable */
		if (!rc)
			chg->chg_cfg.charger_suspend = config->charger_suspend;
	}
	if (config->charging_disable != chg->chg_cfg.charging_disable) {
		value = config->charging_disable;
		rc = set_ps_int_prop(chg->chg_psy, POWER_SUPPLY_PROP_ONLINE, value);
		if (!rc)
			chg->chg_cfg.charging_disable = config->charging_disable;
	}

	if (config->taper_kickoff != chg->chg_cfg.taper_kickoff) {
		chg->chg_cfg.taper_kickoff = config->taper_kickoff;
		chg->chrg_taper_cnt = 0;
	}

	if (config->full_charged != chg->chg_cfg.full_charged) {
		chg->chg_cfg.full_charged = config->full_charged;
	}

	if (config->chrg_iterm != chg->chg_cfg.chrg_iterm) {
		/* Unused configure in device tree for charger */
	}
	if (config->fg_iterm != chg->chg_cfg.fg_iterm) {
		/* Unused configure in device tree for fuel gauge or fuel gauge driver */
	}

	if (config->charging_reset != chg->chg_cfg.charging_reset) {
		if (config->charging_reset) {
			value = 1;
			rc = set_ps_int_prop(chg->chg_psy, POWER_SUPPLY_PROP_ONLINE, value);
			msleep(200);
			value = 0;
			rc = set_ps_int_prop(chg->chg_psy, POWER_SUPPLY_PROP_ONLINE, value);
		}
		chg->chg_cfg.charging_reset = config->charging_reset;
	}

	return 0;
}

#define TAPER_COUNT 2
static bool ext_mmi_chg_is_charge_tapered(void *data, int tapered_ma)
{
	bool is_tapered = false;
	struct ext_mmi_chg_t *chg = data;

	if (abs(chg->batt_info.batt_ma) <= tapered_ma) {
		if (chg->chrg_taper_cnt >= TAPER_COUNT) {
			is_tapered = true;
			chg->chrg_taper_cnt = 0;
		} else
			chg->chrg_taper_cnt++;
	} else
		chg->chrg_taper_cnt = 0;

	return is_tapered;
}

static bool ext_mmi_chg_is_charge_halt(void *data)
{
	struct ext_mmi_chg_t *chg = data;

	if (chg->batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
	    chg->batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		return true;

	return false;
}

static void ext_mmi_chg_set_constraint(void *data,
			struct mmi_charger_constraint *constraint)
{
	int rc = 0;
	struct ext_mmi_chg_t *chg = data;

	if (constraint->demo_mode != chg->constraint.demo_mode) {
		/*value = constraint->demo_mode;
		TODO write value */
		if (!rc)
			chg->constraint.demo_mode = constraint->demo_mode;
	}

	if (constraint->factory_version != chg->constraint.factory_version) {
		/*value = constraint->factory_version;
		TODO write value */
		if (!rc)
			chg->constraint.factory_version = constraint->factory_version;
	}

	if (constraint->factory_mode != chg->constraint.factory_mode) {
		/*value = constraint->factory_mode;
		TODO write value */
		if (!rc)
			chg->constraint.factory_mode = constraint->factory_mode;
	}

	if (constraint->dcp_pmax != chg->constraint.dcp_pmax) {
		/*value = constraint->dcp_pmax;
		TODO write value */
		if (!rc)
			chg->constraint.dcp_pmax = constraint->dcp_pmax;
	}

	if (constraint->hvdcp_pmax != chg->constraint.hvdcp_pmax) {
		/*value = constraint->hvdcp_pmax;
		TODO write value */
		if (!rc)
			chg->constraint.hvdcp_pmax = constraint->hvdcp_pmax;
	}

	if (constraint->pd_pmax != chg->constraint.pd_pmax) {
		/*value = constraint->pd_pmax;
		TODO write value */
		if (!rc)
			chg->constraint.pd_pmax = constraint->pd_pmax;
	}

	if (constraint->wls_pmax != chg->constraint.wls_pmax) {
		/*value = constraint->wls_pmax;
		TODO write value */
		if (!rc)
			chg->constraint.wls_pmax = constraint->wls_pmax;
	}
}

static int find_profile_id(const char *batt_sn)
{
	int i;
	struct profile_sn_map {
		int id;
		const char *sn;
	} map_table[] = {
		{0, BATT_SN_UNKNOWN},
	};

	i = sizeof(map_table) / sizeof(struct profile_sn_map);
	while (i > 0 && map_table[--i].sn && strcmp(map_table[i].sn, batt_sn));

	return map_table[i].id;
}

static int ext_mmi_chg_parse_dt(struct ext_mmi_chg_t *chg)
{
	int rc;
	int i, j;
	int n = 0;
	int bk_num;
	int bk_size;
	char bk_buf[128];
	int byte_len;
	const char *df_sn = NULL, *dev_sn = NULL;
	struct device_node *node;
	const char *fg_psy_name, *chg_psy_name;

	node = chg->dev->of_node;

	rc = of_property_read_string(node, "mmi,fg-psy", &fg_psy_name);
	rc |= of_property_read_string(node, "mmi,chg-psy", &chg_psy_name);
	if (!rc && fg_psy_name && chg_psy_name) {
		chg->fg_psy = power_supply_get_by_name(fg_psy_name);
		chg->chg_psy = power_supply_get_by_name(chg_psy_name);
		if (!chg->fg_psy || !chg->chg_psy) {
			mmi_dbg(chg, "Could not get fg psy and chg psy, maybe we are early - defer.");
			return -EPROBE_DEFER;
		}
	}

	dev_sn = mmi_get_battery_serialnumber();
	if (!dev_sn) {
		rc = of_property_read_string(node, "mmi,df-serialnum",
						&df_sn);
		if (!rc && df_sn) {
			mmi_dbg(chg, "Default Serial Number %s\n", df_sn);
		} else {
			mmi_err(chg, "No Default Serial Number defined\n");
			df_sn = BATT_SN_UNKNOWN;
		}
		strcpy(chg->batt_info.batt_sn, df_sn);
	} else {
		strcpy(chg->batt_info.batt_sn, dev_sn);
	}
	chg->profile_info.profile_id = find_profile_id(chg->batt_info.batt_sn);

	rc = of_property_read_u32(node, "mmi,chrg-iterm-ma",
				  &chg->profile_info.chrg_iterm);
	if (rc) {
		chg->profile_info.chrg_iterm = 300000;
	} else {
		chg->profile_info.chrg_iterm *= 1000;
	}

	rc = of_property_read_u32(node, "mmi,fg-iterm-ma",
				  &chg->profile_info.fg_iterm);
	if (rc) {
		chg->profile_info.fg_iterm =
			chg->profile_info.chrg_iterm + 50000;
	} else {
		chg->profile_info.fg_iterm *= 1000;
	}

	rc = of_property_read_u32(node, "mmi,vfloat-comp-uv",
				  &chg->profile_info.vfloat_comp_uv);
	if (rc)
		chg->profile_info.vfloat_comp_uv = 0;

	rc = of_property_read_u32(node, "mmi,max-fv-mv",
				  &chg->profile_info.max_fv_uv);
	if (rc)
		chg->profile_info.max_fv_uv = 4400;
	chg->profile_info.max_fv_uv *= 1000;

	rc = of_property_read_u32(node, "mmi,max-fcc-ma",
				  &chg->profile_info.max_fcc_ua);
	if (rc)
		chg->profile_info.max_fcc_ua = 4000;
	chg->profile_info.max_fcc_ua *= 1000;

	rc = of_property_read_u32(node, "mmi,demo-fv-mv",
				  &chg->profile_info.demo_fv_uv);
	if (rc)
		chg->profile_info.demo_fv_uv = 4000;
	chg->profile_info.demo_fv_uv *= 1000;

	rc = of_property_read_u32(node, "mmi,profile-data-block-size",
				  &chg->profile_info.data_bk_size);
	if (rc)
		chg->profile_info.data_bk_size = 4;
	chg->profile_info.data_bk_size *= 4;

	chg->profile_info.data_size = 0;
	if (of_find_property(node, "mmi,profile-data", &byte_len)) {
		if (byte_len % chg->profile_info.data_bk_size) {
			mmi_err(chg, "DT error wrong profile data\n");
			chg->profile_info.data_bk_size = 0;
			return -ENODEV;
		}
		bk_num = byte_len / chg->profile_info.data_bk_size;
		chg->profile_data = (u32 *)devm_kzalloc(chg->dev, byte_len,
							GFP_KERNEL);
		if (chg->profile_data == NULL) {
			chg->profile_info.data_bk_size = 0;
			return -ENOMEM;
		}

		rc = of_property_read_u32_array(node,
				"mmi,profile-data",
				chg->profile_data,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_err(chg, "Couldn't read profile data, rc = %d\n", rc);
			devm_kfree(chg->dev, chg->profile_data);
			chg->profile_data = NULL;
			chg->profile_info.data_bk_size = 0;
			return rc;
		}

		chg->profile_info.data_size = byte_len;
		mmi_info(chg, "profile data: block size: %d, num: %d\n",
				chg->profile_info.data_bk_size, bk_num);
		bk_size = chg->profile_info.data_bk_size / 4;
		for (i = 0; i < bk_num; i++) {
			memset(bk_buf, '\0', sizeof(bk_buf));
			n = sprintf(bk_buf, "block%d:", i);
			for (j = 0; j < bk_size; j++) {
				n += sprintf(bk_buf + n, " %d",
					chg->profile_data[i * bk_size + j]);
			}
			mmi_info(chg, "%s\n", bk_buf);
		}
	}

	return 0;
}

static int ext_mmi_chg_init_driver(struct ext_mmi_chg_t *chg)
{
	struct mmi_charger_driver *driver;
	int rc;

	if (chg->driver) {
		mmi_warn(chg, "charger has already inited\n");
		return 0;
	}

	driver = devm_kzalloc(chg->dev,
				sizeof(struct mmi_charger_driver),
				GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	/* init driver */
	driver->name = chg->name;
	driver->dev = chg->dev;
	driver->data = chg;
	driver->get_batt_info = ext_mmi_chg_get_batt_info;
	driver->get_chg_info = ext_mmi_chg_get_chg_info;
	driver->config_charge = ext_mmi_chg_config_charge;
	driver->is_charge_tapered = ext_mmi_chg_is_charge_tapered;
	driver->is_charge_halt = ext_mmi_chg_is_charge_halt;
	driver->set_constraint = ext_mmi_chg_set_constraint;
	chg->driver = driver;

	/* register driver to mmi charger */
	rc = mmi_register_charger_driver(driver);
	if (rc) {
		mmi_err(chg, "mmi charger init failed, rc=%d\n", rc);
		return rc;
	} else {
		mmi_info(chg, "mmi charger init successfully\n");
	}

	// TODO force update in mmi_charger on register to set correct parameters.

	return 0;
}

static int ext_mmi_chg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ext_mmi_chg_t *chg;
	int rc;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->name = "ext_charger";
	chg->debug_enabled = &debug_enabled;

	chg->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chg->ipc_log)
		mmi_warn(chg, "Error in creating ipc_log_context\n");

	rc = ext_mmi_chg_parse_dt(chg);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			mmi_err(chg, "dt parser failed, rc=%d\n", rc);
		return rc;
	}

	chg->usb_psy = power_supply_get_by_name("usb");
	if (!chg->usb_psy) {
		mmi_dbg(chg, "Could not get usb psy, maybe we are early - defer.\n");
		return -EPROBE_DEFER;
	}

	if (ext_mmi_chg_init_driver(chg)) {
		mmi_err(chg, "Failed to init chg driver %d\n", rc);
		return -ENODEV;
	}

	return 0;
}

static int ext_mmi_chg_remove(struct platform_device *pdev)
{
	// TODO implement
	return 0;
}

static const struct of_device_id ext_mmi_chg_match_table[] = {
	{.compatible = "mmi,ext-mmi-chg"},
	{},
};

static struct platform_driver ext_mmi_chg = {
	.driver	= {
		.name = "ext_charger",
		.of_match_table = ext_mmi_chg_match_table,
	},
	.probe	= ext_mmi_chg_probe,
	.remove	= ext_mmi_chg_remove,
};

module_platform_driver(ext_mmi_chg);

MODULE_DESCRIPTION("External Charger Driver");
MODULE_LICENSE("GPL v2");
