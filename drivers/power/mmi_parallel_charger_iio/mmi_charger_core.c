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

#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include "mmi_charger_class.h"
#include "mmi_charger_core.h"
#include "mmi_charger_core_iio.h"
#include "mmi_charger_policy.h"
#include "mmi_qc3p.h"
#include <linux/iio/consumer.h>
#include <linux/qti_power_supply.h>

static int __debug_mask = 0x85;
module_param_named(
	debug_mask, __debug_mask, int, S_IRUSR | S_IWUSR
);

struct mmi_cp_policy_dev g_chrg_list = {0};
const struct mmi_chrg_dev_ops dev_ops[] = {
	{
		.dev_name = "pmic-sw",
		.ops = &qpnp_pmic_charger_ops,
	},
	{
		.dev_name = "cp-master",
		.ops = &sc8549_charger_ops,
	},
	{
		.dev_name = "cp-slave",
		.ops = &sc8549_charger_ops,
	},
};

bool is_chan_valid(struct mmi_charger_manager *chip,
		enum mmi_charger_ext_iio_channels chan)
{
	int rc;

	if (IS_ERR(chip->ext_iio_chans[chan]))
		return false;

	if (!chip->ext_iio_chans[chan]) {
		chip->ext_iio_chans[chan] = iio_channel_get(chip->dev,
					mmi_charger_ext_iio_chan_name[chan]);
		if (IS_ERR(chip->ext_iio_chans[chan])) {
			rc = PTR_ERR(chip->ext_iio_chans[chan]);
			if (rc == -EPROBE_DEFER)
				chip->ext_iio_chans[chan] = NULL;

			mmi_chrg_err(chip, "Failed to get IIO channel %s, rc=%d\n",
				mmi_charger_ext_iio_chan_name[chan], rc);
			return false;
		}
	}

	return true;
}

int mmi_charger_read_iio_chan(struct mmi_charger_manager *chip,
	enum mmi_charger_ext_iio_channels chan, int *val)
{
	int rc;

	if (is_chan_valid(chip, chan)) {
		rc = iio_read_channel_processed(
				chip->ext_iio_chans[chan], val);
		return (rc < 0) ? rc : 0;
	}

	return -EINVAL;
}

int mmi_charger_write_iio_chan(struct mmi_charger_manager *chip,
	enum mmi_charger_ext_iio_channels chan, int val)
{
	if (is_chan_valid(chip, chan))
		return iio_write_channel_raw(chip->ext_iio_chans[chan],
						val);

	return -EINVAL;
}

static int mmi_charger_init_iio_psy(struct mmi_charger_manager *chip,
				struct platform_device *pdev)
{
	int rc = 0;
	chip->ext_iio_chans = devm_kcalloc(chip->dev,
				ARRAY_SIZE(mmi_charger_ext_iio_chan_name),
				sizeof(*chip->ext_iio_chans),
				GFP_KERNEL);
	if (!chip->ext_iio_chans)
		return -ENOMEM;

	return rc;
}

static int qc3p_volt_max_init = 0;
static int pd_volt_max_init = 0;
static int pd_curr_max_init = 0;
static int *dt_temp_zones;
static struct mmi_chrg_dts_info *chrg_name_list;
static char *charge_rate[] = {
	"None", "Normal", "Weak", "Turbo", "Turbo_30W", "Hyper"
};
#define MIN_TEMP_C -20
#define MAX_TEMP_C 60
#define HYSTEREISIS_DEGC 2
void mmi_chrg_rate_check(struct mmi_charger_manager *chg);
bool mmi_find_temp_zone(struct mmi_charger_manager *chip, int temp_c, bool ignore_hysteresis_degc)
{
	int prev_zone, num_zones;
	struct mmi_chrg_temp_zone *zones;
	int hotter_t = 0, hotter_fcc = 0;
	int colder_t = 0, colder_fcc = 0;
	int i;
	int max_temp;

	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return false;
	}

	zones = chip->temp_zones;
	num_zones = chip->num_temp_zones;
	prev_zone = chip->pres_temp_zone;

	mmi_chrg_info(chip, "prev zone %d, temp_c %d\n",prev_zone, temp_c);
	max_temp = zones[num_zones - 1].temp_c;

	if (prev_zone == ZONE_NONE) {
		for (i = num_zones - 1; i >= 0; i--) {
			if (temp_c >= zones[i].temp_c) {
				if (i == num_zones - 1)
					chip->pres_temp_zone = ZONE_HOT;
				else
					chip->pres_temp_zone = i + 1;
				return true;
			}
		}
		chip->pres_temp_zone = ZONE_COLD;
		return true;
	}

	if (prev_zone == ZONE_COLD) {
		if (temp_c >= MIN_TEMP_C + HYSTEREISIS_DEGC)
			chip->pres_temp_zone = ZONE_FIRST;
	} else if (prev_zone == ZONE_HOT) {
		if (temp_c <=  max_temp - HYSTEREISIS_DEGC)
			chip->pres_temp_zone = num_zones - 1;
	} else {
		if (prev_zone == ZONE_FIRST) {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = MIN_TEMP_C;
			hotter_fcc = zones[prev_zone + 1].chrg_step_power->chrg_step_curr;
			colder_fcc = 0;
		} else if (prev_zone == num_zones - 1) {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = zones[prev_zone - 1].temp_c;
			hotter_fcc = 0;
			colder_fcc = zones[prev_zone - 1].chrg_step_power->chrg_step_curr;
		} else {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = zones[prev_zone - 1].temp_c;
			hotter_fcc = zones[prev_zone + 1].chrg_step_power->chrg_step_curr;
			colder_fcc = zones[prev_zone - 1].chrg_step_power->chrg_step_curr;
		}

		if (!ignore_hysteresis_degc) {
			if (zones[prev_zone].chrg_step_power->chrg_step_curr < hotter_fcc)
				hotter_t += HYSTEREISIS_DEGC;
			if (zones[prev_zone].chrg_step_power->chrg_step_curr < colder_fcc)
				colder_t -= HYSTEREISIS_DEGC;
		}

		if (temp_c <= MIN_TEMP_C)
			chip->pres_temp_zone = ZONE_COLD;
		else if (temp_c >= max_temp)
			chip->pres_temp_zone = ZONE_HOT;
		else if (temp_c >= hotter_t)
			chip->pres_temp_zone++;
		else if (temp_c < colder_t)
			chip->pres_temp_zone--;
	}

	mmi_chrg_info(chip, "batt temp_c %d, prev zone %d, pres zone %d, "
						"hotter_fcc %dmA, colder_fcc %dmA, "
						"hotter_t %dC, colder_t %dC\n",
						temp_c,prev_zone, chip->pres_temp_zone,
						hotter_fcc, colder_fcc, hotter_t, colder_t);

	if (prev_zone != chip->pres_temp_zone) {
		mmi_chrg_info(chip,
			   "Entered Temp Zone %d!\n",
			   chip->pres_temp_zone);
		return true;
	}
	return false;
}

bool mmi_find_chrg_step(struct mmi_charger_manager *chip, int temp_zone, int vbatt_volt)
{
	int batt_volt, i;
	bool find_step = false;
	struct mmi_chrg_temp_zone zone;
	struct mmi_chrg_step_power *chrg_steps;
	struct mmi_chrg_step_info chrg_step_inline;
	struct mmi_chrg_step_info prev_step;

	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return false;
	}

	if (chip->pres_temp_zone == ZONE_HOT ||
		chip->pres_temp_zone == ZONE_COLD ||
		chip->pres_temp_zone < ZONE_FIRST) {
		mmi_chrg_err(chip, "pres temp zone is HOT or COLD, "
							"can't find chrg step\n");
		return false;
	}

	zone = chip->temp_zones[chip->pres_temp_zone];
	chrg_steps = zone.chrg_step_power;
	prev_step = chip->chrg_step;

	batt_volt = vbatt_volt;
	chrg_step_inline.temp_c = zone.temp_c;

	mmi_chrg_info(chip, "batt_volt %d, chrg step %d, step nums %d\n",
						batt_volt, prev_step.pres_chrg_step,
						chip->chrg_step_nums);

	/*In the first search cycle, find out the vbatt is less than step volt*/

	for (i = 0; i < chip->chrg_step_nums; i++) {
		mmi_chrg_info(chip,
					"first cycle,i %d, step volt %d, batt_volt %d\n",
					i, chrg_steps[i].chrg_step_volt, batt_volt);
		if (chrg_steps[i].chrg_step_volt > 0
			&& batt_volt < chrg_steps[i].chrg_step_volt) {
			if ( (i + 1) < chip->chrg_step_nums
				&& chrg_steps[i + 1].chrg_step_volt > 0) {
				chrg_step_inline.chrg_step_cv_tapper_curr =
					chrg_steps[i + 1].chrg_step_curr;
			} else
				chrg_step_inline.chrg_step_cv_tapper_curr =
					chrg_steps[i].chrg_step_curr;
			chrg_step_inline.chrg_step_cc_curr =
				chrg_steps[i].chrg_step_curr;
			chrg_step_inline.chrg_step_cv_volt =
				chrg_steps[i].chrg_step_volt;
			chrg_step_inline.pres_chrg_step = i;
			find_step = true;
			mmi_chrg_info(chip, "find chrg step\n");
			break;
		}
	}

	if (find_step) {
		mmi_chrg_info(chip, "chrg step %d, "
					"step cc curr %d, step cv volt %d, "
					"step cv tapper curr %d\n",
					chrg_step_inline.pres_chrg_step,
					chrg_step_inline.chrg_step_cc_curr,
					chrg_step_inline.chrg_step_cv_volt,
					chrg_step_inline.chrg_step_cv_tapper_curr);
		chip->chrg_step = chrg_step_inline;
	} else {

	/*If can't find out any chrg step in the first search cycle,*/
	/*it means that vbatt is already greater than all chrg step volt, */
	/*therefore, start to enter second search cycle, */
	/*to find out the maximal chrg step volt*/
		for (i = 0; i < chip->chrg_step_nums; i++) {
			mmi_chrg_info(chip,
						"second cycle, i %d, step volt %d, batt_volt %d\n",
						i, chrg_steps[i].chrg_step_volt, batt_volt);
			if (chrg_steps[i].chrg_step_volt > 0
				&& batt_volt > chrg_steps[i].chrg_step_volt) {
				if ( (i + 1) < chip->chrg_step_nums
					&& chrg_steps[i + 1].chrg_step_volt > 0) {
					chrg_step_inline.chrg_step_cv_tapper_curr =
						chrg_steps[i + 1].chrg_step_curr;
				} else
					chrg_step_inline.chrg_step_cv_tapper_curr =
						chrg_steps[i].chrg_step_curr;
				chrg_step_inline.chrg_step_cc_curr =
					chrg_steps[i].chrg_step_curr;
				chrg_step_inline.chrg_step_cv_volt =
					chrg_steps[i].chrg_step_volt;
				chrg_step_inline.pres_chrg_step = i;
				find_step = true;
				mmi_chrg_info(chip, "find chrg step\n");
			}
		}

		if (find_step) {
			mmi_chrg_info(chip, "chrg step %d, "
					"step cc curr %d, step cv volt %d, "
					"step cv tapper curr %d\n",
					chrg_step_inline.pres_chrg_step,
					chrg_step_inline.chrg_step_cc_curr,
					chrg_step_inline.chrg_step_cv_volt,
					chrg_step_inline.chrg_step_cv_tapper_curr);
			chip->chrg_step = chrg_step_inline;
		}
	}


	if (find_step) {
		if (chip->chrg_step.chrg_step_cc_curr ==
			chip->chrg_step.chrg_step_cv_tapper_curr)
			chip->chrg_step.last_step = true;
		else
			chip->chrg_step.last_step = false;

		mmi_chrg_info(chip,"Temp zone %d, "
				"select chrg step %d, step cc curr %d,"
				"step cv volt %d, step cv tapper curr %d, "
				"is the last chrg step %d\n",
				chip->pres_temp_zone,
				chip->chrg_step.pres_chrg_step,
				chip->chrg_step.chrg_step_cc_curr,
				chip->chrg_step.chrg_step_cv_volt,
				chip->chrg_step.chrg_step_cv_tapper_curr,
				chip->chrg_step.last_step);

		if (prev_step.pres_chrg_step != chip->chrg_step.pres_chrg_step) {
			mmi_chrg_info(chip, "Find the next chrg step\n");
			return true;
		}
	}
	return false;
}

void mmi_set_pps_result_history(struct mmi_charger_manager *chip, int pps_result)
{
	if (chip->pps_result_history_idx >= PPS_RET_HISTORY_SIZE -1)
		chip->pps_result_history_idx = 0;

	if (pps_result < 0)
		chip->pps_result_history[chip->pps_result_history_idx] = 1;
	else
		chip->pps_result_history[chip->pps_result_history_idx] = 0;

	chip->pps_result_history_idx++;
}

bool mmi_get_pps_result_history(struct mmi_charger_manager *chip)
{
	int i = 0;
	int result = 0;
	for (i = 0; i < PPS_RET_HISTORY_SIZE; i++)
		result += chip->pps_result_history[i];

	if (result >= PPS_RET_HISTORY_SIZE)
		return RESET_POWER;
	else if (result >= PPS_RET_HISTORY_SIZE / 2)
		return BLANCE_POWER;
	else
		return NO_ERROR;
}

void mmi_clear_pps_result_history(struct mmi_charger_manager *chip)
{
	int i = 0;;
	for (i = 0; i < PPS_RET_HISTORY_SIZE; i++)
		chip->pps_result_history[i] = 0;
}

int mmi_calculate_delta_volt(int pps_voltage, int pps_current, int delta_curr)
{
	u64 power;
	int delta_volt;
	u64 pps_volt;
	u64 pps_curr;

	pps_volt = pps_voltage;
	pps_curr = pps_current;
	power = pps_volt * pps_curr;

	delta_volt = (int)(power / (pps_curr - delta_curr) - pps_volt);
	delta_volt -= delta_volt % 20000;

	if (delta_volt > 0)
		return delta_volt;
	else
		return 0;
}

void mmi_update_all_charger_status(struct mmi_charger_manager *chip)
{
	int chrg_num, i;
	struct mmi_charger_device *chrg_dev;
	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return;
	}

	chrg_num = chip->mmi_chrg_dev_num;
	for (i = 0; i < chrg_num; i++) {
		chrg_dev = chip->chrg_list[i];
		mmi_update_charger_status(chrg_dev);
	}
	return;
}

void mmi_update_all_charger_error(struct mmi_charger_manager *chip)
{
	int chrg_num, i;
	struct mmi_charger_device *chrg_dev;
	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return;
	}

	chrg_num = chip->mmi_chrg_dev_num;
	for (i = 0; i < chrg_num; i++) {
		chrg_dev = chip->chrg_list[i];
		mmi_update_charger_error(chrg_dev);
	}
	return;
}

void mmi_dump_charger_error(struct mmi_charger_manager *chip,
									struct mmi_charger_device *chrg_dev)
{
	if (!chip || !chrg_dev) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return;
	}
	mmi_chrg_dbg(chip, PR_MOTO, "%s: charger error type %d, "
					"bus ucp err cnt %d, bus ocp err cnt %d\n",
					chrg_dev->name,
					chrg_dev->charger_error.chrg_err_type,
					chrg_dev->charger_error.bus_ucp_err_cnt,
					chrg_dev->charger_error.bus_ocp_err_cnt);
	return;
}
#if 0
static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_QNOVO,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_QNOVO,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SW_JEITA_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_PARALLEL_DISABLE,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_RECHARGE_SOC,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_FORCE_RECHARGE,
	POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_NUM_SYSTEM_TEMP_LEVELS,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_RATE,
	POWER_SUPPLY_PROP_AGE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int batt_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);
	union power_supply_propval prop = {0,};
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = power_supply_get_property(chip->extrn_psy,
							psp, &prop);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d from extrn_psy rc = %d\n",
								psp, rc);
			rc = power_supply_get_property(chip->qcom_psy,
							psp, &prop);
		}
		if (!rc)
			val->intval = prop.intval;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (chip->use_batt_age) {
			val->intval = chip->cycles;
		} else {
			rc = power_supply_get_property(chip->qcom_psy,
							psp, &prop);
			if (rc < 0) {
				mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
				rc = 0;
				val->intval = -EINVAL;
			}
			if (!rc)
				val->intval = prop.intval;
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		rc = power_supply_get_property(chip->qcom_psy,
							psp, &prop);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
			rc = 0;
			val->intval = -EINVAL;
		} else {
			val->intval = prop.intval;
			if (prop.intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
				rc = power_supply_get_property(chip->extrn_psy,
									psp, &prop);
				if (!rc && (prop.intval == POWER_SUPPLY_STATUS_CHARGING
					|| prop.intval == POWER_SUPPLY_STATUS_FULL))
					val->intval = prop.intval;
			}
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_RATE:
		mmi_chrg_rate_check(chip);
		val->intval = chip->charger_rate;
		break;
	default:
		rc = power_supply_get_property(chip->qcom_psy,
							psp, &prop);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
			rc = 0;
			val->intval = -EINVAL;
		}
		if (!rc)
			val->intval = prop.intval;
		break;

	}
	return rc;
}

static int batt_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		chip->cycles = val->intval;
		break;
	default:
		rc = power_supply_set_property(chip->qcom_psy,
						prop, val);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", prop, rc);
			rc = 0;
		}
		break;
	}
	return rc;
}

static int batt_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_DIE_HEALTH:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		return 1;
	default:
		break;
	}
	return 0;
}
static const struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = batt_props,
	.num_properties = ARRAY_SIZE(batt_props),
	.get_property = batt_get_property,
	.set_property = batt_set_property,
	.property_is_writeable = batt_is_writeable,
};
static int batt_psy_register(struct mmi_charger_manager *chip)
{
	struct power_supply_config batt_cfg = {};

	batt_cfg.drv_data = chip;
	batt_cfg.of_node = chip->dev->of_node;
	chip->batt_psy = power_supply_register(chip->dev,
						&batt_psy_desc,
						&batt_cfg);
	if (IS_ERR(chip->batt_psy)) {
		pr_err("Couldn't register batt_psy power supply\n");
		return PTR_ERR(chip->batt_psy);
	}

	pr_info("power supply register batt_psy successfully\n");
	return 0;
}
#endif
static enum power_supply_property mmi_chrg_mgr_props[] = {
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int mmi_chrg_mgr_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);
	union power_supply_propval prop = {0,};
	int rc, i, chrg_cnt = 0;

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		if (!rc)
			val->intval = prop.intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		if (!rc)
			val->intval = prop.intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		if (chip->mmi_chrg_dev_num) {
			for (i = 0; i < chip->mmi_chrg_dev_num; i++) {
			 if (chip->chrg_list[i]->charger_enabled)
			 	chrg_cnt++;
			}
			val->intval = chrg_cnt;
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;

	}
	return 0;
}

static int mmi_chrg_mgr_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{

	switch (prop) {
	default:
		return -EINVAL;
	}

	return 0;
}

static int mmi_chrg_mgr_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	default:
		ret = 0;
		break;
	}
	return ret;
}

static const struct power_supply_desc mmi_chrg_mgr_psy_desc = {
	.name = "mmi_chrg_manager",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = mmi_chrg_mgr_props,
	.num_properties = ARRAY_SIZE(mmi_chrg_mgr_props),
	.get_property = mmi_chrg_mgr_get_property,
	.set_property = mmi_chrg_mgr_set_property,
	.property_is_writeable = mmi_chrg_mgr_is_writeable,
};

static int mmi_chrg_mgr_psy_register(struct mmi_charger_manager *chip)
{
	struct power_supply_config mmi_chrg_mgr_cfg = {};

	mmi_chrg_mgr_cfg.drv_data = chip;
	mmi_chrg_mgr_cfg.of_node = chip->dev->of_node;
	chip->mmi_chrg_mgr_psy = power_supply_register(chip->dev,
						&mmi_chrg_mgr_psy_desc,
						&mmi_chrg_mgr_cfg);
	if (IS_ERR(chip->mmi_chrg_mgr_psy)) {
		pr_err("Couldn't register mmi_chrg_mgr_psy power supply\n");
		return PTR_ERR(chip->mmi_chrg_mgr_psy);
	}

	pr_info("power supply register mmi_chrg_mgr_psy successfully\n");
	return 0;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct mmi_charger_manager *chip =
					container_of(nb, struct mmi_charger_manager, psy_nb);

	if (!chip->usb_psy) {
		mmi_chrg_err(chip, "usb psy is NULL, direct return\n");
		return NOTIFY_OK;
	}

	if (ptr == chip->usb_psy && evt == PSY_EVENT_PROP_CHANGED) {
		schedule_work(&chip->psy_changed_work);

		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(1000));

	}

	return NOTIFY_OK;
}

static int get_prop_charger_present(struct mmi_charger_manager *chg,
				    union power_supply_propval *val)
{
	int rc = -EINVAL;
	val->intval = 0;

	rc = mmi_charger_read_iio_chan(chg, SMB5_TYPEC_MODE, &val->intval);
	if (rc < 0) {
		mmi_chrg_err(chg, "Couldn't read TypeC Mode rc=%d\n", rc);
		return rc;
	}

	switch (val->intval) {
	case QTI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	case QTI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case QTI_POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		val->intval = 1;
		break;
	default:
		val->intval = 0;
		break;
	}

	return rc;
}

#define WEAK_CHRG_THRSH 450
#define TURBO_CHRG_THRSH 2500
#define TURBO_30W_CHRG_THRSH_UW 25000000
#define HYPER_CHRG_THRSH_UW 40000000

void mmi_chrg_rate_check(struct mmi_charger_manager *chg)
{
	union power_supply_propval val;
	int chrg_cm_ma = 0;
	int chrg_cs_ma = 0;
	int charger_power = 0;
	int prev_chg_rate = chg->charger_rate;
	int rc = -EINVAL;

	if (!chg->usb_psy) {
		mmi_chrg_err(chg, "No usb PSY\n");
		return;
	}

	val.intval = 0;
	rc = get_prop_charger_present(chg, &val);
	if (rc < 0) {
		mmi_chrg_err(chg, "Error getting Charger Present rc = %d\n", rc);
		return;
	}

	if (val.intval) {
		val.intval = 0;
		rc = mmi_charger_read_iio_chan(chg, SMB5_HW_CURRENT_MAX, &val.intval);
		if (rc < 0) {
			mmi_chrg_info(chg, "Error getting HW Current Max rc = %d\n", rc);
			return;
		}
		chrg_cm_ma = val.intval / 1000;

		val.intval = 0;
		rc = mmi_charger_read_iio_chan(chg, SMB5_USB_INPUT_CURRENT_SETTLED, &val.intval);
		if (rc < 0) {
			mmi_chrg_err(chg, "Error getting ICL Settled rc = %d\n", rc);
			return;
		}
		chrg_cs_ma = val.intval / 1000;
	} else {
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NONE;
		goto end_rate_check;
	}

	mmi_chrg_info(chg, "SMBMMI: cm %d, cs %d\n", chrg_cm_ma, chrg_cs_ma);
	if (chrg_cm_ma >= TURBO_CHRG_THRSH)
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_TURBO;
	else if ((chrg_cm_ma > WEAK_CHRG_THRSH) && (chrg_cs_ma < WEAK_CHRG_THRSH))
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_WEAK;
	else if (prev_chg_rate == POWER_SUPPLY_CHARGE_RATE_NONE)
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;

	if (chg->pd_pps_support) {
		charger_power = (chg->pd_curr_max / 1000) * (chg->pd_volt_max / 1000);
		if (charger_power >= HYPER_CHRG_THRSH_UW)
			chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_HYPER;
		else if (charger_power >= TURBO_30W_CHRG_THRSH_UW)
			chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_TURBO_30W;
	}
end_rate_check:
	if (prev_chg_rate != chg->charger_rate)
		mmi_chrg_info(chg, "%s Charger Detected\n",
		       charge_rate[chg->charger_rate]);

}
#if 0
#define CHG_SHOW_MAX_SIZE 50
static ssize_t factory_image_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid factory image mode value = %lu\n", mode);
		return -EINVAL;
	}

	mmi_set_factory_image_mode(mode);

	return r ? r : count;
}

static ssize_t factory_image_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	state = mmi_get_factory_image_mode() ? 1 : 0;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_image_mode, 0644,
		factory_image_mode_show,
		factory_image_mode_store);

static ssize_t factory_charge_upper_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	state = mmi_get_factory_charge_upper();

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_charge_upper, 0444,
		factory_charge_upper_show,
		NULL);

static ssize_t force_demo_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid demo  mode value = %lu\n", mode);
		return -EINVAL;
	}

	mmi_set_demo_mode(mode);

	return r ? r : count;
}

static ssize_t force_demo_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	state = 	mmi_get_demo_mode();

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_demo_mode, 0644,
		force_demo_mode_show,
		force_demo_mode_store);

static ssize_t force_max_chrg_temp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid max temp value = %lu\n", mode);
		return -EINVAL;
	}

	mmi_set_max_chrg_temp(mode);

	return r ? r : count;
}

static ssize_t force_max_chrg_temp_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	state = mmi_get_max_chrg_temp();

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_max_chrg_temp, 0644,
		force_max_chrg_temp_show,
		force_max_chrg_temp_store);
#endif
static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}


static void kick_sm(struct mmi_charger_manager *chip, int ms)
{
//	union power_supply_propval val;
//	int ret;

	if (!chip->sm_work_running) {
		mmi_chrg_dbg(chip, PR_INTERRUPT,
					"launch mmi chrg sm work\n");
		mmi_chrg_policy_clear(chip);
		schedule_delayed_work(&chip->mmi_chrg_sm_work,
				msecs_to_jiffies(ms));
		chip->sm_work_running = true;
/*		val.intval = true;
		ret = power_supply_set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CP_ENABLE, &val);
		if (ret)
			mmi_chrg_err(chip, "Unable to enable CP: %d\n", ret);*/
	} else
		mmi_chrg_dbg(chip, PR_INTERRUPT,
					"mmi chrg sm work already existed\n");
}

static void cancel_sm(struct mmi_charger_manager *chip)
{
//	union power_supply_propval val;
//	int ret;

	cancel_delayed_work_sync(&chip->mmi_chrg_sm_work);
	flush_delayed_work(&chip->mmi_chrg_sm_work);
	mmi_chrg_policy_clear(chip);
	chip->sm_work_running = false;
/*	val.intval = false;
	ret = power_supply_set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CP_ENABLE, &val);
	if (ret)
		mmi_chrg_err(chip, "Unable to disable CP: %d\n", ret);*/
	chip->pd_volt_max = pd_volt_max_init;
	chip->pd_curr_max = pd_curr_max_init;
	mmi_chrg_dbg(chip, PR_INTERRUPT,
					"cancel sync and flush mmi chrg sm work\n");
}

void chrg_policy_error_clear(struct mmi_charger_manager *chip,
					struct mmi_cp_policy_dev *chrg_list)
{
	int chrg_num = 0, i = 0;
	struct mmi_charger_device *chrg_dev;
	chrg_num = chip->mmi_chrg_dev_num;

	for (i = 0; i < chrg_num; i++) {
		switch (i) {
			case PMIC_SW:
				break;

			case CP_MASTER:
				if (is_charger_exist(dev_ops[CP_MASTER].dev_name)) {
					chrg_dev = chrg_list->chrg_dev[CP_MASTER];
					mmi_clear_charger_error(chrg_dev);
				}
				break;

			case CP_SLAVE:
				break;

			default:
				mmi_chrg_err(chip,"mmi_chrg_dev not found %d !\n",i);
				break;
		}
	}

	return;
}

void clear_chrg_dev_error_cnt(struct mmi_charger_manager *chip, struct mmi_cp_policy_dev *chrg_list)
{
	int chrg_num, i;
	struct mmi_charger_device *chrg_dev;
	chrg_num = chip->mmi_chrg_dev_num;

	for (i = 0; i < chrg_num; i++) {
		if (is_charger_exist(dev_ops[i].dev_name)) {
			chrg_dev = chrg_list->chrg_dev[i];
			chrg_dev->charger_error.chrg_err_type = 0;
			chrg_dev->charger_error.bus_ucp_err_cnt = 0;
			chrg_dev->charger_error.bus_ocp_err_cnt = 0;
		}
	}
	return;
}

static void kick_qc3p_sm(struct mmi_charger_manager *chip, int ms)
{


	if (!chip->qc3p_sm_work_running) {
		mmi_chrg_dbg(chip, PR_INTERRUPT,
					"launch mmi qc3p chrg sm work\n");
		mmi_qc3p_chrg_policy_clear(chip); //todo
		schedule_delayed_work(&chip->mmi_qc3p_chrg_sm_work, //
				msecs_to_jiffies(ms)); //todo
		chip->qc3p_sm_work_running = true;
	} else
		mmi_chrg_dbg(chip, PR_INTERRUPT,
					"mmi chrg qc3p sm work already existed\n");
}

static void cancel_qc3p_sm(struct mmi_charger_manager *chip)
{
	cancel_delayed_work_sync(&chip->mmi_qc3p_chrg_sm_work);
	flush_delayed_work(&chip->mmi_qc3p_chrg_sm_work);
	mmi_qc3p_chrg_policy_clear(chip);
	chip->qc3p_sm_work_running = false;
	chip->qc3p_volt_max = qc3p_volt_max_init;
	mmi_chrg_dbg(chip, PR_INTERRUPT,
					"cancel sync and flush mmi chrg qc3p sm work\n");
}

static void mmi_awake_vote(struct mmi_charger_manager *chip, bool awake)
{
	if (awake == chip->awake)
		return;

	chip->awake = awake;
	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);
}

static void mmi_batt_age_init(struct mmi_charger_manager *chip)
{
	int rc = 0;
	union power_supply_propval pval;
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return;
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt status\n");
		return;
	}
	chip->pres_batt_status = pval.intval;
	chip->prev_batt_status = chip->pres_batt_status;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt capacity\n");
		return;
	}

	chip->soc_cycles_start= pval.intval;
	chip->batt_cap_delta = 0;
	return;
}

static void mmi_cycle_counts(struct mmi_charger_manager *chip)
{
	int rc = 0;
	int batt_design_capacity = 0, new_cycles = 0;
	int soc_report = 0, soc_delta = 0, batt_delta_capacity = 0;
	union power_supply_propval pval;
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return;
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt design capacity\n");
		return;
	}
		batt_design_capacity = pval.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt status\n");
		return;
	}
	chip ->pres_batt_status = pval.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt capacity\n");
		return;
	}
	soc_report = pval.intval;

	mmi_chrg_dbg(chip, PR_MOTO, "cycle_count %d, pres %d, prev %d, soc_report %d, "
					"soc_cycles_start %d, batt_cap_delta %d, "
					"design_capcaity %d, batt_soc_delta %d\n",
					chip->cycles, chip->pres_batt_status,
					chip->prev_batt_status,
					soc_report, chip->soc_cycles_start,
					chip->batt_cap_delta, batt_design_capacity,
					chip->batt_soc_delta);

	if (chip->prev_batt_status == POWER_SUPPLY_STATUS_CHARGING
		&& chip->pres_batt_status != POWER_SUPPLY_STATUS_CHARGING) {
		chip->soc_cycles_start = soc_report;
	} else if (chip->prev_batt_status != POWER_SUPPLY_STATUS_CHARGING
		&& chip->pres_batt_status == POWER_SUPPLY_STATUS_CHARGING) {

		soc_delta = chip->soc_cycles_start - soc_report;
		if (soc_delta > 0) {
			chip->batt_soc_delta += soc_delta;

			batt_delta_capacity = (chip->batt_soc_delta * batt_design_capacity) / 100
								+ chip->batt_cap_delta;

			new_cycles = batt_delta_capacity / batt_design_capacity;
			if (new_cycles > 0) {
				chip->cycles += batt_delta_capacity / batt_design_capacity;
				chip->batt_cap_delta = batt_delta_capacity % batt_design_capacity;
				chip->batt_soc_delta = 0;
			}
		}
		chip->soc_cycles_start = soc_report;
	}

	chip->prev_batt_status = chip->pres_batt_status;
}

static void mmi_power_supply_changed(struct power_supply *psy,
					 char *envp_ext[])
{
	dev_err(&psy->dev, "%s: %s\n", __func__, envp_ext[0]);

	kobject_uevent_env(&psy->dev.kobj, KOBJ_CHANGE, envp_ext);
	power_supply_changed(psy);
}

#define HEARTBEAT_DELAY_MS 60000
#define SMBCHG_HEARTBEAT_INTRVAL_NS	70000000000
#define CHG_SHOW_MAX_SIZE 50
static void mmi_heartbeat_work(struct work_struct *work)
{
	struct mmi_charger_manager *chip = container_of(work,
				struct mmi_charger_manager, heartbeat_work.work);
	int hb_resch_time = 0, ret = 0, i = 0;
	char *chrg_rate_string = NULL;
	char *envp[2];
	union power_supply_propval val;
	bool pd_active;

	mmi_chrg_info(chip, "MMI: Heartbeat!\n");
	/* Have not been resumed so wait another 100 ms */
	if (chip->suspended) {
		mmi_chrg_err(chip, "SMBMMI: HB running before Resume\n");
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));
		return;
	}

	mmi_awake_vote(chip, true);

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy) {
			mmi_chrg_err(chip,
				"Could not get USB power_supply, deferring probe\n");
			return;
		}
	}

	ret = power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		mmi_chrg_err(chip, "Unable to read USB PRESENT: %d\n", ret);
		return;
	}
	chip->vbus_present = val.intval;

	hb_resch_time = HEARTBEAT_DELAY_MS;

	if (!chip->vbus_present)
		mmi_awake_vote(chip, false);

	ret = mmi_charger_read_iio_chan(chip, SMB5_USB_PD_ACTIVE, &val.intval);
	if (ret) {
		mmi_chrg_err(chip, "Unable to read PD ACTIVE: %d\n", ret);
		goto schedule_work;
	}
	pd_active = val.intval;

	if (!chip->pd_handle) {
		chip->pd_handle = devm_usbpd_get_by_phandle(chip->dev,
						    "qcom,usbpd-phandle");
		if (IS_ERR_OR_NULL(chip->pd_handle)) {
			mmi_chrg_err(chip, "Error getting the pd phandle %ld\n",
				PTR_ERR(chip->pd_handle));
			chip->pd_handle = NULL;
			goto schedule_work;
		}
	}

	if (!chip->sm_work_running && chip->vbus_present
		&& pd_active) {
		usbpd_get_pdo_info(chip->pd_handle, chip->mmi_pdo_info,PD_MAX_PDO_NUM);
		mmi_chrg_info(chip, "check all effective pdo info\n");
		for (i = 0; i < PD_MAX_PDO_NUM; i++) {
			if ((chip->mmi_pdo_info[i].type ==
					PD_SRC_PDO_TYPE_AUGMENTED)
				&& chip->mmi_pdo_info[i].uv_max >= PUMP_CHARGER_PPS_MIN_VOLT
				&& chip->mmi_pdo_info[i].ua >= chip->typec_middle_current) {
					chip->mmi_pd_pdo_idx = chip->mmi_pdo_info[i].pdo_pos;
					mmi_chrg_info(chip,
							"pd charger support pps, pdo %d, "
							"volt %d, curr %d \n",
							chip->mmi_pd_pdo_idx,
							chip->mmi_pdo_info[i].uv_max,
							chip->mmi_pdo_info[i].ua);
					chip->pd_pps_support = true;

					if (chip->mmi_pdo_info[i].uv_max <
							chip->pd_volt_max) {
						chip->pd_volt_max =
						chip->mmi_pdo_info[i].uv_max;
					}
					if (chip->mmi_pdo_info[i].ua <
							chip->pd_curr_max) {
						chip->pd_curr_max =
						chip->mmi_pdo_info[i].ua;
					}
				break;
			}
		}
	}
	if (!chip->qc3p_sm_work_running && chip->vbus_present)
		chip->qc3p_active = mmi_qc3p_power_active(chip);

	if (chip->pd_pps_support
		&& !chip->factory_mode) {
		mmi_chrg_info(chip, "MMI: Heartbeat!, launch sm work\n");
		kick_sm(chip, 100);
	} else if(chip->qc3p_active
		&& !chip->factory_mode) {
		mmi_chrg_info(chip, "MMI: Heartbeat!, launch qc3p sm work\n");
		kick_qc3p_sm(chip, 100);
	}

	if (chip->use_batt_age)
		mmi_cycle_counts(chip);

	mmi_chrg_rate_check(chip);
       if (!chip->extrn_fg) {
/*		val.intval = chip->charger_rate;
		ret = power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CHARGE_RATE, &val);
		if (ret)
			mmi_chrg_err(chip, "Unable to set charge rate: %d\n", ret);
*/       }

	chrg_rate_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (!chrg_rate_string) {
		mmi_chrg_err(chip, "SMBMMI: Failed to Get Uevent Mem\n");
		envp[0] = NULL;
	} else {
		scnprintf(chrg_rate_string, CHG_SHOW_MAX_SIZE,
			  "POWER_SUPPLY_CHARGE_RATE=%s",
			  charge_rate[chip->charger_rate]);
		envp[0] = chrg_rate_string;
		envp[1] = NULL;
	}

	if (chip->qcom_psy) {
		mmi_power_supply_changed(chip->batt_psy, envp);
	}

	kfree(chrg_rate_string);

schedule_work:
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));
}

static void psy_changed_work_func(struct work_struct *work)
{
	struct mmi_charger_manager *chip = container_of(work,
				struct mmi_charger_manager, psy_changed_work);
	union power_supply_propval val;
	bool pd_active;
	int ret, i;

	mmi_chrg_info(chip, "kick psy changed work.\n");

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy) {
			mmi_chrg_err(chip,
				"Could not get USB power_supply, deferring probe\n");
			return;
		}
	}

	ret = power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		mmi_chrg_err(chip, "Unable to read USB PRESENT: %d\n", ret);
		return;
	}
	chip->vbus_present = val.intval;

	ret = mmi_charger_read_iio_chan(chip, SMB5_USB_PD_ACTIVE, &val.intval);
	if (ret) {
		mmi_chrg_err(chip, "Unable to read PD ACTIVE: %d\n", ret);
		return;
	}
	pd_active = val.intval;

	if (!chip->pd_handle) {
		chip->pd_handle = devm_usbpd_get_by_phandle(chip->dev,
						    "qcom,usbpd-phandle");
		if (IS_ERR_OR_NULL(chip->pd_handle)) {
			mmi_chrg_err(chip, "Error getting the pd phandle %ld\n",
				PTR_ERR(chip->pd_handle));
			chip->pd_handle = NULL;
			return;
		}
	}

	if (pd_active && chip->vbus_present) {
		usbpd_get_pdo_info(chip->pd_handle, chip->mmi_pdo_info,PD_MAX_PDO_NUM);
		mmi_chrg_info(chip, "check all effective pdo info\n");
		for (i = 0; i < PD_MAX_PDO_NUM; i++) {
			if ((chip->mmi_pdo_info[i].type ==
					PD_SRC_PDO_TYPE_AUGMENTED)
				&& chip->mmi_pdo_info[i].uv_max >= PUMP_CHARGER_PPS_MIN_VOLT
				&& chip->mmi_pdo_info[i].ua >= chip->typec_middle_current) {
					chip->mmi_pd_pdo_idx = chip->mmi_pdo_info[i].pdo_pos;
					mmi_chrg_info(chip,
							"pd charger support pps, pdo %d, "
							"volt %d, curr %d \n",
							chip->mmi_pd_pdo_idx,
							chip->mmi_pdo_info[i].uv_max,
							chip->mmi_pdo_info[i].ua);
					chip->pd_pps_support = true;

					if (chip->mmi_pdo_info[i].uv_max <
							chip->pd_volt_max) {
						chip->pd_volt_max =
						chip->mmi_pdo_info[i].uv_max;
					}

					if (chip->mmi_pdo_info[i].ua <
							chip->pd_curr_max) {
						chip->pd_curr_max =
						chip->mmi_pdo_info[i].ua;
					}

				break;
			}
		}
	}

	mmi_chrg_info(chip, "vbus present %d, pd pps support %d, "
					"pps max voltage %d, pps max curr %d\n",
					chip->vbus_present,
					chip->pd_pps_support,
					chip->pd_volt_max,
					chip->pd_curr_max);
	if (chip->vbus_present)
		chip->qc3p_active = mmi_qc3p_power_active(chip);

	if (chip->vbus_present
		&& chip->pd_pps_support
		&& !chip->factory_mode) {
		kick_sm(chip, 100);
	} else if (chip->vbus_present
		&& chip->qc3p_active
		&& !chip->factory_mode) {
		kick_qc3p_sm(chip, 100);
	} else {
		cancel_sm(chip);
		cancel_qc3p_sm(chip);
		chip->pd_pps_support =  false;
		chip->qc3p_active =  false;
	}

	return;
}
#define DEFAULT_IBUS_MAX_MA 3000
#define DEFAULT_QC3P_VOLT_STEPS	20000
#define DEFAULT_QC3P_VOLT_MAX	11000000
#define DEFAULT_BATT_OVP_LMT		4475000
#define DEFAULT_PL_CHRG_VBATT_MIN	3600000
#define DEFAULT_PPS_VOLT_STEPS	20000
#define DEFAULT_PPS_CURR_STEPS	50000
#define DEFAULT_PPS_VOLT_MAX	11000000
#define DEFAULT_PPS_CURR_MAX	4000000
#define MMI_TEMP_ZONES	5
static int mmi_chrg_manager_parse_dt(struct mmi_charger_manager *chip)
{
	struct device_node *node = chip->dev->of_node, *child;
	int rc, byte_len, i, j, chrg_idx = 0, step_cnt = 0, idx = 0;
	const int *ptr;

	rc = of_property_read_u32(node,
				"mmi,batt-ovp-lmt",
				&chip->batt_ovp_lmt);
	if (rc < 0)
		chip->batt_ovp_lmt =
				DEFAULT_BATT_OVP_LMT;

	rc = of_property_read_u32(node,
				"mmi,pl-chrg-vbatt-min",
				&chip->pl_chrg_vbatt_min);
	if (rc < 0)
		chip->pl_chrg_vbatt_min =
				DEFAULT_PL_CHRG_VBATT_MIN;

	chip->use_batt_age = of_property_read_bool(node,
				"mmi,use-batt-age");

	chip->extrn_fg = of_property_read_bool(node,
				"mmi,extrn-fg");

	if (chip->extrn_fg) {
		byte_len = of_property_count_strings(node, "mmi,extrn-fg-name");
		if (byte_len <= 0) {
			mmi_chrg_err(chip, "Cannot parse mmi, extrn-fg: %d\n", byte_len);
			return byte_len;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(node, "mmi,extrn-fg-name",
					i, &chip->extrn_fg_name);
			if (rc < 0) {
				mmi_chrg_err(chip, "Cannot parse extrn-fg-name\n");
				return rc;
			}
		}
	}

	chip->extrn_sense = of_property_read_bool(node,
			"mmi,extrn-sense");

	chip->dont_rerun_aicl= of_property_read_bool(node,
			"mmi,dont-rerun-aicl");

	rc = of_property_read_u32(node,
				"mmi,typec-middle-current",
				&chip->typec_middle_current);
	if (rc < 0)
		chip->typec_middle_current =
				TYPEC_MIDDLE_CURRENT_UA;

	rc = of_property_read_u32(node,
				"mmi,step-first-current-comp",
				&chip->step_first_curr_comp);
	if (rc < 0)
		chip->step_first_curr_comp =
				STEP_FIREST_CURR_COMP;

	rc = of_property_read_u32(node,
				"mmi,pps-volt-steps",
				&chip->pps_volt_steps);
	if (rc < 0)
		chip->pps_volt_steps =
				DEFAULT_PPS_VOLT_STEPS;

	rc = of_property_read_u32(node,
				"mmi,pps-curr-steps",
				&chip->pps_curr_steps);
	if (rc < 0)
		chip->pps_curr_steps =
				DEFAULT_PPS_CURR_STEPS;

	rc = of_property_read_u32(node,
				"mmi,pd-volt-max",
				&chip->pd_volt_max);
	if (rc < 0)
		chip->pd_volt_max =
				DEFAULT_PPS_VOLT_MAX;
	pd_volt_max_init = chip->pd_volt_max;
	rc = of_property_read_u32(node,
				"mmi,pd-curr-max",
				&chip->pd_curr_max);
	if (rc < 0)
		chip->pd_curr_max =
				DEFAULT_PPS_CURR_MAX;
	pd_curr_max_init = chip->pd_curr_max;

	rc = of_property_read_u32(node,
				"mmi,qc3p-max-ibus-ma",
				&chip->qc3p_max_ibus_ma);
	if (rc < 0)
		chip->qc3p_max_ibus_ma =
				DEFAULT_IBUS_MAX_MA;
	rc = of_property_read_u32(node,
				"mmi,qc3p-volt-steps",
				&chip->qc3p_volt_steps);
	if (rc < 0)
		chip->qc3p_volt_steps =
				DEFAULT_QC3P_VOLT_STEPS;

	rc = of_property_read_u32(node,
				"mmi,qc3p-volt-max",
				&chip->qc3p_volt_max);
	if (rc < 0)
		chip->qc3p_volt_max =
				DEFAULT_QC3P_VOLT_MAX;
	qc3p_volt_max_init = chip->qc3p_volt_max;

	for_each_child_of_node(node, child)
		chip->mmi_chrg_dev_num++;

	if (!chip->mmi_chrg_dev_num) {
		mmi_chrg_err(chip,"No mmi_chrg_dev  list !\n");
		return -ENODEV;
	}

	chrg_name_list = (struct mmi_chrg_dts_info *)devm_kzalloc(chip->dev,
					sizeof(struct mmi_chrg_dts_info) *
					chip->mmi_chrg_dev_num, GFP_KERNEL);
	if (!chrg_name_list) {
		mmi_chrg_err(chip,"No memory for mmi charger device name list !\n");
		goto cleanup;
	}

	chip->chrg_list = (struct mmi_charger_device **)devm_kzalloc(chip->dev,
					sizeof(struct mmi_charger_device *) *
					chip->mmi_chrg_dev_num, GFP_KERNEL);
	if (!chip->chrg_list) {
		mmi_chrg_err(chip,"No memory for mmi charger device list !\n");
		goto cleanup;
	}

	for_each_child_of_node(node, child) {
		byte_len = of_property_count_strings(child, "chrg-name");
		if (byte_len <= 0) {
			mmi_chrg_err(chip, "Cannot parse chrg-name: %d\n", byte_len);
			goto cleanup;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(child, "chrg-name",
					i, &chrg_name_list[chrg_idx].chrg_name);
			if (rc < 0) {
				mmi_chrg_err(chip, "Cannot parse chrg-name\n");
				goto cleanup;
			}
		}

		byte_len = of_property_count_strings(child, "psy-name");
		if (byte_len <= 0) {
			mmi_chrg_err(chip, "Cannot parse psy-name: %d\n", byte_len);
			goto cleanup;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(child, "psy-name",
					i, &chrg_name_list[chrg_idx].psy_name);
			if (rc < 0) {
				mmi_chrg_err(chip, "Cannot parse psy-name\n");
				goto cleanup;
			}
		}

		mmi_chrg_info(chip, "mmi,chrg-name: %s, psy-name: %s\n",
					chrg_name_list[chrg_idx].chrg_name,
					chrg_name_list[chrg_idx].psy_name);

		of_property_read_u32(child, "charging-curr-limited",
			&chrg_name_list[chrg_idx].charging_curr_limited);

		of_property_read_u32(child, "charging-curr-min",
			&chrg_name_list[chrg_idx].charging_curr_min);

		chrg_idx++;
	}

	rc = of_property_read_u32(node,
				"mmi,chrg-temp-zones-num",
				&chip->num_temp_zones);
	if (rc < 0)
		chip->num_temp_zones =
				MMI_TEMP_ZONES;

	chip->temp_zones = (struct mmi_chrg_temp_zone *)
					devm_kzalloc(chip->dev,
					sizeof(struct mmi_chrg_temp_zone )
					* chip->num_temp_zones,
					GFP_KERNEL);
	if (chip->temp_zones == NULL) {
		rc = -ENOMEM;
		goto cleanup;
	}

	if (of_find_property(node, "mmi,mmi-chrg-temp-zones", &byte_len)) {
		if (byte_len <= 0) {
			mmi_chrg_err(chip,
					"Cannot parse mmi-chrg-temp-zones %d\n",byte_len);
			goto cleanup;
		}

		step_cnt = (byte_len / chip->num_temp_zones - sizeof(u32))
				/ (sizeof(u32) * 2);
		mmi_chrg_info(chip, "mmi chrg step number is %d\n", step_cnt);

		dt_temp_zones = (u32 *) devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);
		if (dt_temp_zones == NULL) {
			rc = -ENOMEM;
			goto cleanup;
		}

		rc = of_property_read_u32_array(node,
				"mmi,mmi-chrg-temp-zones",
				(u32 *)dt_temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_chrg_err(chip,
				   "Couldn't read mmi chrg temp zones rc = %d\n", rc);
			goto zones_failed;
		}

		mmi_chrg_err(chip, "temp zone nums: %d , byte_len %d, "
						"num %d, row size %d\n",
						chip->num_temp_zones, byte_len,
			   			(int)(byte_len / sizeof(u32)),
			   			(int)(sizeof(struct mmi_chrg_step_power) * step_cnt));
		ptr = dt_temp_zones;

		for (i = 0; i < byte_len / sizeof(u32); i++) {
			if (!(i % 9)) {
			  printk("\n");
			}
			printk("%u,", ptr[i]);
		}
		printk("\n");

		for (i = 0; i < chip->num_temp_zones; i++) {
			idx = (int)((sizeof(u32) +
				sizeof(struct mmi_chrg_step_power) * step_cnt )
				* i / sizeof(u32));
			chip->temp_zones[i].temp_c = dt_temp_zones[idx];
		}

		for (i = 0; i < chip->num_temp_zones; i++) {
			idx = (int)(((sizeof(u32) +
					sizeof(struct mmi_chrg_step_power) * step_cnt )
					* i + sizeof(u32)) / sizeof(u32));
			chip->temp_zones[i].chrg_step_power =
					(struct mmi_chrg_step_power *)&dt_temp_zones[idx];
		}

		for (i = 0; i < chip->num_temp_zones; i++) {

			printk( "mmi temp zones: Zone: %d, Temp: %d C " , i,
				   chip->temp_zones[i].temp_c);
			for (j = 0; j < step_cnt; j++) {
				chip->temp_zones[i].chrg_step_power[j].chrg_step_curr *= 1000;
				chip->temp_zones[i].chrg_step_power[j].chrg_step_volt *= 1000;

				printk("step_volt %dmV, step_curr  %dmA, ",
				   chip->temp_zones[i].chrg_step_power[j].chrg_step_volt,
				   chip->temp_zones[i].chrg_step_power[j].chrg_step_curr);
			}
			printk("\n");
		}

		chip->pres_temp_zone = ZONE_NONE;
		chip->chrg_step_nums = step_cnt;
	}

	if (of_find_property(node, "mmi,thermal-mitigation", &byte_len)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev, byte_len,
				GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			rc = -ENOMEM;
			goto zones_failed;
		}
		chip->thermal_levels = byte_len / sizeof(u32);
		rc = of_property_read_u32_array(node,
			"mmi,thermal-mitigation",
			chip->thermal_mitigation,
			chip->thermal_levels);
		if (rc < 0) {
			mmi_chrg_err(chip,
				   "Couldn't parse thermal-mitigation rc = %d\n", rc);
			goto thermal_failed;
		}
	}

	return rc;
thermal_failed:
	devm_kfree(chip->dev,chip->thermal_mitigation);
zones_failed:
	devm_kfree(chip->dev,chip->temp_zones);
cleanup:
	chip->mmi_chrg_dev_num = 0;
	devm_kfree(chip->dev,chip->chrg_list);
	return rc;
}

void chrg_dev_init(struct mmi_charger_manager *chip, struct mmi_cp_policy_dev *chrg_list)
{
       int chrg_num, i;
       chrg_num = chip->mmi_chrg_dev_num;
       mmi_chrg_err(chip,"runing in chrg dev init!\n");

       for (i = 0; i < chrg_num; i++) {

               switch (i) {
               case PMIC_SW:
                       if (is_charger_exist(dev_ops[PMIC_SW].dev_name)) {
                               chrg_list->pmic_sw = true;
                               chrg_list->chrg_dev[PMIC_SW] = chip->chrg_list[PMIC_SW];
                               }
                      break;
               case CP_MASTER:
                       if (is_charger_exist(dev_ops[CP_MASTER].dev_name)) {
                               chrg_list->cp_master = true;
                               chrg_list->chrg_dev[CP_MASTER] = chip->chrg_list[CP_MASTER];
                               }
                       break;
               case CP_SLAVE:
                       if (is_charger_exist(dev_ops[CP_SLAVE].dev_name)) {
                               chrg_list->cp_slave = true;
                               chrg_list->cp_clave_later = false;
                               chrg_list->chrg_dev[CP_SLAVE] = chip->chrg_list[CP_SLAVE];
                               }
                       break;
               default:
                       mmi_chrg_err(chip,"No mmi_chrg_dev found %d !\n",i);
                       break;
               }
       }
       return;
}

int mmi_chrg_policy_init(struct mmi_charger_manager *chip,
					struct mmi_chrg_dts_info *chrg_dts,
					int chrg_cnt) {
	int i = 0, ops_cnt = 0, chrg_idx = 0;
	struct mmi_charger_device *chrg_dev;

	if (!chip ||!chrg_dts) {
		mmi_chrg_err(chip, "invalid input param!\n");
		return -EINVAL;
	}

	ops_cnt = ARRAY_SIZE(dev_ops);
	if (chrg_cnt > ops_cnt ||
		chrg_cnt > chip->mmi_chrg_dev_num) {
		mmi_chrg_err(chip, "invalid input param!, chrg_cnt %d , ops_cnt %d, "
						"mmi_chrg_dev_num %d\n",
			chrg_cnt, ops_cnt, chip->mmi_chrg_dev_num);
		return -EINVAL;
	}

	mmi_chrg_err(chip, "chrg_cnt %d\n", chrg_cnt);
	for (i = 0; i < chrg_cnt; i++) {
		if (!strcmp(chrg_dts[i].chrg_name, dev_ops[i].dev_name)) {

			chrg_dev = mmi_charger_device_register(chrg_dts[i].chrg_name,
				chrg_dts[i].psy_name, chip->dev, chip,
				dev_ops[i].ops);
			if (IS_ERR_OR_NULL(chrg_dev)
				|| !is_charger_exist(chrg_dts[i].chrg_name)) {
				mmi_chrg_err(chip,
					"register mmi charger %s failed\n",
					chrg_dts[i].chrg_name);
				return -EINVAL;
			} else {
				mmi_chrg_info(chip,
				"register mmi charger %s successfully, i %d, chrg_idx %d\n",
				chrg_dts[i].chrg_name, i, chrg_idx);
				chrg_dev->charging_curr_limited =
					chrg_dts[i].charging_curr_limited;
				mmi_chrg_err(chip, "charging_curr_limited %d\n",
					chrg_dts[i].charging_curr_limited);
				chrg_dev->charging_curr_min =
					chrg_dts[i].charging_curr_min;
				mmi_chrg_err(chip, "charging_curr_min %d, chrg_id %d\n",
								chrg_dts[i].charging_curr_min, chrg_idx);
				chip->chrg_list[chrg_idx] = chrg_dev;
				mmi_chrg_err(chip, "--- over ----\n");
				chrg_idx++;
			}
		}
	}

	mmi_chrg_info(chip,"chrg_cnt %d, ops_cnt %d, mmi_chrg_dev_num %d, "
				"chrg_idx %d\n",
				chrg_cnt, ops_cnt, chip->mmi_chrg_dev_num, chrg_idx);

	if (chrg_idx != chip->mmi_chrg_dev_num
		&& chrg_idx > 0) {

		mmi_chrg_err(chip, "chrg_id %d != charger num %d\n",
						chrg_idx, chip->mmi_chrg_dev_num);
		for (i = 0; i < chrg_idx; i++)
			mmi_charger_device_unregister(chip->chrg_list[i]);
		return -EINVAL;
	}

	chrg_dev_init(chip, &g_chrg_list);
	chip->pps_volt_comp = PPS_INIT_VOLT_COMP;
	INIT_DELAYED_WORK(&chip->mmi_chrg_sm_work, mmi_chrg_sm_work_func);
	chip->qc3p_volt_comp = QC3P_INIT_VOLT_COMP;
	INIT_DELAYED_WORK(&chip->mmi_qc3p_chrg_sm_work, mmi_qc3p_chrg_sm_work_func);
	return 0;
}

static int mmi_chrg_manager_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mmi_charger_manager *chip;

	if (!pdev)
		return -ENODEV;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct mmi_charger_manager),
								GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for mmi_charger_manager\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->name = "mmi_chrg_manager";
	chip->debug_mask = &__debug_mask;
	chip->suspended = false;

	ret = mmi_charger_class_init();
	if (ret < 0) {
		dev_err(&pdev->dev,
			"mmi charger class init failed\n");
		goto cleanup;
	}

	ret = mmi_chrg_manager_parse_dt(chip);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"parse dt failed\n");
		goto cleanup;
	}

	ret = mmi_charger_init_iio_psy(chip, pdev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"mmi charger iio psy init failed\n");
		goto cleanup;
	}

	chip->factory_mode = mmi_factory_check();

	chip->qcom_psy = power_supply_get_by_name("qcom_battery");
#if 0
	if (chip->qcom_psy && chip->extrn_fg) {

		chip->extrn_psy = power_supply_get_by_name(chip->extrn_fg_name);
		if (!chip->extrn_psy) {
			mmi_chrg_err(chip, "Get extrn psy failed\n");
			goto cleanup;
		}
		ret = batt_psy_register(chip);
		if (ret) {
			mmi_chrg_err(chip, "Batt psy register failed\n");
			goto cleanup;
		}

		ret = device_create_file(chip->dev,
					&dev_attr_factory_image_mode);
		if (ret) {
			mmi_chrg_err(chip, "couldn't create factory_image_mode\n");
		}

		ret = device_create_file(chip->dev,
					&dev_attr_force_demo_mode);
		if (ret) {
			mmi_chrg_err(chip, "couldn't create force_demo_mode\n");
		}

		ret = device_create_file(chip->dev,
					&dev_attr_force_max_chrg_temp);
		if (ret) {
			mmi_chrg_err(chip, "couldn't create force_max_chrg_temp\n");
		}

		ret = device_create_file(chip->dev,
					&dev_attr_factory_charge_upper);
		if (ret)
			mmi_chrg_err(chip, "couldn't create factory_charge_upper\n");

	} else
#endif		
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy) {
		mmi_chrg_err(chip, "Could not get battery power_supply\n");
		goto cleanup;
	}


	ret = mmi_chrg_policy_init(chip, chrg_name_list,
					chip->mmi_chrg_dev_num);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"mmi chrg policy init failed\n");
		goto cleanup;
	}

	chip->pd_handle =
			devm_usbpd_get_by_phandle(chip->dev, "qcom,usbpd-phandle");
	if (IS_ERR_OR_NULL(chip->pd_handle)) {
		dev_err(&pdev->dev, "Error getting the pd phandle %ld\n",
							PTR_ERR(chip->pd_handle));
		chip->pd_handle = NULL;
	}

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy)
			mmi_chrg_err(chip, "Could not get USB power_supply\n");
	}

	INIT_WORK(&chip->psy_changed_work, psy_changed_work_func);
	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_heartbeat_work);

	ret = mmi_chrg_mgr_psy_register(chip);
	if (ret)
		goto cleanup;

	chip->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_nb);
	if (ret)
		goto cleanup;

	mmi_batt_age_init(chip);

	init_completion(&chip->sm_completion);
	platform_set_drvdata(pdev, chip);

	//create_sysfs_entries(chip);
	schedule_work(&chip->psy_changed_work);
	mmi_chrg_info(chip, "mmi chrg manager initialized successfully, ret %d\n", ret);
	return 0;
cleanup:
	mmi_charger_class_exit();
	devm_kfree(&pdev->dev, chip);
	return ret;
}

static int mmi_chrg_manager_remove(struct platform_device *pdev)
{
	struct mmi_charger_manager *chip =  platform_get_drvdata(pdev);

	//remove_sysfs_entries(chip);
	//cancel_delayed_work_sync(&chip->mmi_chrg_sm_work);
	power_supply_unreg_notifier(&chip->psy_nb);
	power_supply_unregister(chip->mmi_chrg_mgr_psy );
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mmi_chrg_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_charger_manager *chip = platform_get_drvdata(pdev);

	chip->suspended = true;

	return 0;
}

static int mmi_chrg_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_charger_manager *chip = platform_get_drvdata(pdev);

	chip->suspended = false;

	return 0;
}
#else
#define smb_mmi_suspend NULL
#define smb_mmi_resume NULL
#endif

static const struct dev_pm_ops mmi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmi_chrg_suspend, mmi_chrg_resume)
};

static const struct of_device_id mmi_chrg_manager_match_table[] = {
	{.compatible = "mmi,chrg-manager"},
	{},
};
MODULE_DEVICE_TABLE(of, mmi_chrg_manager_match_table);

static struct platform_driver mmi_chrg_manager_driver = {
	.probe = mmi_chrg_manager_probe,
	.remove = mmi_chrg_manager_remove,
	.driver = {
		.name = "mmi_chrg_manager",
		.owner = THIS_MODULE,
		.pm = &mmi_dev_pm_ops,
		.of_match_table = mmi_chrg_manager_match_table,
	},
};

static int __init mmi_chrg_manager_init(void)
{
	int ret;
	ret = platform_driver_register(&mmi_chrg_manager_driver);
	if (ret) {
		pr_err("mmi_chrg_manager failed to register driver\n");
		return ret;
	}
	return 0;
}

static void __exit mmi_chrg_manager_exit(void)
{
	platform_driver_unregister(&mmi_chrg_manager_driver);
}

module_init(mmi_chrg_manager_init);
module_exit(mmi_chrg_manager_exit);

MODULE_ALIAS("platform:mmi parallel charger");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Motorola Mobility parallel charger");
MODULE_LICENSE("GPL");
