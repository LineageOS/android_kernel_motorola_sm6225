/*
 * Copyright (c) 2018 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include "mmi_charger_class.h"
#include <linux/power_supply.h>
#include "mmi_charger_core.h"

#define	BAT_OVP_FAULT_SHIFT			8
#define	BAT_OCP_FAULT_SHIFT			9
#define	BUS_OVP_FAULT_SHIFT			10
#define	BUS_OCP_FAULT_SHIFT			11
#define	BAT_THERM_FAULT_SHIFT			12
#define	BUS_THERM_FAULT_SHIFT			13
#define	DIE_THERM_FAULT_SHIFT			14
#define	CONV_OCP_FAULT_SHIFT			15
#define	SS_TIMEOUT_FAULT_SHIFT			16
#define	TS_SHUT_FAULT_SHIFT			17

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)
#define	CONV_OCP_FAULT_MASK		(1 << CONV_OCP_FAULT_SHIFT)
#define	SS_TIMEOUT_FAULT_MASK		(1 << SS_TIMEOUT_FAULT_SHIFT)
#define	TS_SHUT_FAULT_MASK		(1 << TS_SHUT_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BUS_UCP_FAULT_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define	BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BUS_UCP_FAULT_MASK		(1 << BUS_UCP_FAULT_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

static int bq2597x_enable_charging(struct mmi_charger_device *chrg, bool en)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	prop.intval = en;
	rc = power_supply_set_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);

	if (!rc) {
		chrg->charger_enabled = !!prop.intval;
	} else
		chrg->charger_enabled  = false;

	return rc;
}

static int bq2597x_is_charging_enabled(struct mmi_charger_device *chrg, bool *en)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (!rc) {
		chrg->charger_enabled = !!prop.intval;
	} else
		chrg->charger_enabled  = false;

	*en = chrg->charger_enabled;

	return rc;
}

static int bq2597x_get_charging_current(struct mmi_charger_device *chrg, u32 *uA)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (!rc)
		*uA = !!prop.intval;

	return rc;
}

static int bq2597x_get_input_current(struct mmi_charger_device *chrg, u32 *uA)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &prop);
	if (!rc)
		*uA = prop.intval;

	return rc;
}

static int bq2597x_update_charger_status(struct mmi_charger_device *chrg)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (!rc)
		chrg->charger_data.vbatt_volt = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (!rc)
		chrg->charger_data.ibatt_curr = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_TEMP, &prop);
	if (!rc)
		chrg->charger_data.batt_temp = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED, &prop);
	if (!rc)
		chrg->charger_data.vbus_volt = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &prop);
	if (!rc)
		chrg->charger_data.ibus_curr = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_PRESENT, &prop);
	if (!rc)
		chrg->charger_data.vbus_pres = !!prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (!rc)
		chrg->charger_enabled = !!prop.intval;

	chrg_dev_info(chrg, "BQ2597x chrg: %s status update: --- info---\n",chrg->name);
	chrg_dev_info(chrg, "vbatt %d\n", chrg->charger_data.vbatt_volt);
	chrg_dev_info(chrg, "ibatt %d\n", chrg->charger_data.ibatt_curr);
	chrg_dev_info(chrg, "batt temp %d\n", chrg->charger_data.batt_temp);
	chrg_dev_info(chrg, "vbus %d\n", chrg->charger_data.vbus_volt);
	chrg_dev_info(chrg, "ibus %d\n", chrg->charger_data.ibus_curr);
	chrg_dev_info(chrg, "vbus pres %d\n", chrg->charger_data.vbus_pres);
	chrg_dev_info(chrg, "charger_enabled %d\n", chrg->charger_enabled);

	return rc;
}

static int bq2597x_update_charger_error_status(struct mmi_charger_device *chrg)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CP_STATUS1, &prop);
	if (!rc) {
		chrg->charger_error.chrg_err_type =
			((!!(prop.intval & BAT_OVP_ALARM_MASK)) ?
			1 << MMI_BAT_OVP_ALARM_BIT :
			0 << MMI_BAT_OVP_ALARM_BIT) |
			((!!(prop.intval & BAT_OCP_ALARM_MASK)) ?
			1 << MMI_BAT_OCP_ALARM_BIT :
			0 << MMI_BAT_OCP_ALARM_BIT) |
			((!!(prop.intval & BAT_UCP_ALARM_MASK)) ?
			1 << MMI_BAT_UCP_ALARM_BIT :
			0 << MMI_BAT_UCP_ALARM_BIT) |
			((!!(prop.intval & BUS_OVP_ALARM_MASK)) ?
			1 << MMI_BUS_OVP_ALARM_BIT :
			0 << MMI_BUS_OVP_ALARM_BIT) |
			((!!(prop.intval & BUS_OCP_ALARM_MASK)) ?
			1 << MMI_BUS_OCP_ALARM_BIT :
			0 << MMI_BUS_OCP_ALARM_BIT) |
			((!!(prop.intval & BAT_OVP_FAULT_MASK)) ?
			1 << MMI_BAT_OVP_FAULT_BIT :
			0 << MMI_BAT_OVP_FAULT_BIT) |
			((!!(prop.intval & BAT_OCP_FAULT_MASK)) ?
			1 << MMI_BAT_OCP_FAULT_BIT :
			0 << MMI_BAT_OCP_FAULT_BIT) |
			((!!(prop.intval & BUS_OVP_FAULT_MASK)) ?
			1 << MMI_BUS_OVP_FAULT_BIT :
			0 << MMI_BUS_OVP_FAULT_BIT) |
			((!!(prop.intval & BUS_OCP_FAULT_MASK)) ?
			1 << MMI_BUS_OCP_FAULT_BIT :
			0 << MMI_BUS_OCP_FAULT_BIT) |
			((!!(prop.intval & BUS_UCP_FAULT_MASK)) ?
			1 << MMI_BUS_UCP_FAULT_BIT :
			0 << MMI_BUS_UCP_FAULT_BIT) |
			((!!(prop.intval & CONV_OCP_FAULT_MASK)) ?
			1 << MMI_CONV_OCP_FAULT_BIT :
			0 << MMI_CONV_OCP_FAULT_BIT);
	}
	return rc;
}

static int bq2597x_clear_charger_error(struct mmi_charger_device *chrg)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_set_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_UPDATE_NOW, &prop);

	return rc;
}

struct mmi_charger_ops bq2597x_charger_ops = {
	.enable = bq2597x_enable_charging,
	.is_enabled = bq2597x_is_charging_enabled,
	.get_charging_current = bq2597x_get_charging_current,
	.get_input_current = bq2597x_get_input_current,
	.update_charger_status = bq2597x_update_charger_status,
	.update_charger_error = bq2597x_update_charger_error_status,
	.clear_charger_error = bq2597x_clear_charger_error,
};
