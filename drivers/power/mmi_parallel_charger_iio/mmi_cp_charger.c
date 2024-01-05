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
#include "mmi_charger_core_iio.h"

static int cp_enable_charging(struct mmi_charger_device *chrg, bool en)
{
	int rc;
	struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);

	if (!chip)
		return -ENODEV;

	rc = mmi_charger_write_iio_chan(chip, CP_ENABLE, en);

	if (!rc) {
		chrg->charger_enabled = !!en;
	} else
		chrg->charger_enabled  = false;

	return rc;
}

static int cp_is_charging_enabled(struct mmi_charger_device *chrg, bool *en)
{
	int rc;
	int value;
	struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);

	if (!chip)
		return -ENODEV;

	rc = mmi_charger_read_iio_chan(chip, CP_ENABLE, &value);

	if (!rc) {
		chrg->charger_enabled = !!value;
	} else
		chrg->charger_enabled  = false;

	*en = chrg->charger_enabled;

	return rc;
}

static int cp_get_charging_current(struct mmi_charger_device *chrg, u32 *uA)
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

static int cp_get_vbus(struct mmi_charger_device *chrg, u32 *mv)
{
	int rc, val;
       struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);
	if (!chip)
		return -ENODEV;

	rc = mmi_charger_read_iio_chan(chip, CP_INPUT_VOLTAGE_NOW, &val);
	if (!rc)
		*mv = val;

	return rc;
}

static int cp_get_input_current(struct mmi_charger_device *chrg, u32 *uA) //ibus
{
	int rc;
	int value;
	struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);

	if (!chip)
		return -ENODEV;

	rc = mmi_charger_read_iio_chan(chip, CP_INPUT_CURRENT_NOW, &value);

	if (!rc)
		*uA = value;

	return rc;
}

static int cp_update_charger_status(struct mmi_charger_device *chrg)
{
	int rc;
	struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);
	union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &prop);
	if (!rc)
		chrg->charger_data.ibus_curr= prop.intval;

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

	rc = mmi_charger_read_iio_chan(chip, CP_INPUT_VOLTAGE_NOW, &prop.intval);
	if (!rc)
		chrg->charger_data.vbus_volt = prop.intval;

	rc = mmi_charger_read_iio_chan(chip, CP_INPUT_CURRENT_NOW, &prop.intval);
	if (!rc)
		chrg->charger_data.ibus_curr = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_PRESENT, &prop);
	if (!rc)
		chrg->charger_data.vbus_pres = !!prop.intval;

	rc = mmi_charger_read_iio_chan(chip, CP_ENABLE, &prop.intval);
	if (!rc)
		chrg->charger_enabled = !!prop.intval;

	chrg_dev_info(chrg, "CP chrg: %s status update: --- info---\n",chrg->name);
	chrg_dev_info(chrg, "vbatt %d\n", chrg->charger_data.vbatt_volt);
	chrg_dev_info(chrg, "ibatt %d\n", chrg->charger_data.ibatt_curr);
	chrg_dev_info(chrg, "batt temp %d\n", chrg->charger_data.batt_temp);
	chrg_dev_info(chrg, "vbus %d\n", chrg->charger_data.vbus_volt);
	chrg_dev_info(chrg, "ibus %d\n", chrg->charger_data.ibus_curr);
	chrg_dev_info(chrg, "vbus pres %d\n", chrg->charger_data.vbus_pres);
	chrg_dev_info(chrg, "charger_enabled %d\n", chrg->charger_enabled);

	return rc;
}

static int cp_update_charger_error_status(struct mmi_charger_device *chrg)
{
	int rc;
	int value = 0;
	struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);

	if (!chip)
		return -ENODEV;

	rc = mmi_charger_read_iio_chan(chip, CP_STATUS1, &value);
	if (!rc) {
		chrg->charger_error.chrg_err_type = value;
	}
	return rc;
}

static int cp_clear_charger_error(struct mmi_charger_device *chrg)
{
	int rc;
	int value = 0;
	struct mmi_charger_manager *chip = dev_get_drvdata(&chrg->dev);

	if (!chip)
		return -ENODEV;

	rc = mmi_charger_write_iio_chan(chip, CP_CLEAR_ERROR, value);

	return rc;
}

struct mmi_charger_ops cp_charger_ops = {
	.enable = cp_enable_charging,
	.is_enabled = cp_is_charging_enabled,
	.get_charging_current = cp_get_charging_current,
	.get_vbus = cp_get_vbus,
	.get_input_current = cp_get_input_current,
	.update_charger_status = cp_update_charger_status,
	.update_charger_error = cp_update_charger_error_status,
	.clear_charger_error = cp_clear_charger_error,
};

