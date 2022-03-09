/*
 * Copyright (c) 2021 Motorola Mobility, LLC.
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
#include "mmi_charger_core.h"
#include "mmi_qc3p.h"

#define SWITCH_CHARGER_QC3P_VOLT		5300000
typedef enum  {
	PM_QC3P_STATE_DISCONNECT,
	PM_QC3P_STATE_ENTRY,
	PM_QC3P_STATE_SW_ENTRY,
	PM_QC3P_STATE_SW_LOOP,
	PM_QC3P_STATE_CHRG_PUMP_ENTRY,
	PM_QC3P_STATE_SINGLE_CP_ENTRY,
	PM_QC3P_STATE_DULE_CP_ENTRY,
	PM_QC3P_STATE_TUNNING_VOLT,
	PM_QC3P_STATE_CP_CC_LOOP,
	PM_QC3P_STATE_CP_CV_LOOP,
	PM_QC3P_STATE_CP_QUIT,
	PM_QC3P_STATE_RECOVERY_SW,
	PM_QC3P_STATE_STOP_CHARGE,
} pm_sm_qc3p_state_t;

const unsigned char *pm_qc3p_state_str[] = {
	"PM_QC3P_STATE_DISCONNECT",
	"PM_QC3P_STATE_ENTRY",
	"PM_QC3P_STATE_SW_ENTRY",
	"PM_QC3P_STATE_SW_LOOP",
	"PM_QC3P_STATE_CHRG_PUMP_ENTRY",
	"PM_QC3P_STATE_SINGLE_CP_ENTRY",
	"PM_QC3P_STATE_DULE_CP_ENTRY",
	"PM_QC3P_STATE_TUNNING_VOLT",
	"PM_QC3P_STATE_CP_CC_LOOP",
	"PM_QC3P_STATE_CP_CV_LOOP",
	"PM_QC3P_STATE_CP_QUIT",
	"PM_QC3P_STATE_RECOVERY_SW",
	"PM_QC3P_STATE_STOP_CHARGE",
};

static pm_sm_qc3p_state_t	qc3p_sm_state = PM_QC3P_STATE_DISCONNECT;
static int qc3p_chrg_cc_power_tunning_cnt = 0;
static int qc3p_chrg_cv_taper_tunning_cnt = 0;
static int qc3p_chrg_cv_delta_volt = 0;
static int qc3p_quit_slave_chrg_cnt = 0;
static int qc3p_batt_curr_roof = 0;
static int qc3p_constant_power_cnt = 0;

static void mmi_chrg_qc3p_sm_move_state(struct mmi_charger_manager *chip, pm_sm_qc3p_state_t state)
{
	mmi_chrg_dbg(chip, PR_INTERRUPT, "pm_state change:%s -> %s\n",
		pm_qc3p_state_str[qc3p_sm_state], pm_qc3p_state_str[state]);
	qc3p_sm_state = state;
	qc3p_constant_power_cnt = 0;
	qc3p_batt_curr_roof = 0;
}

#define QC3P_VOLT_COMP_DELTA	300000
static void qc3p_chrg_policy_error_recovery(struct mmi_charger_manager *chip,
										struct mmi_cp_policy_dev *chrg_list)
{
	int chrg_num = 0, i = 0, chrg_error_type= 0;
	struct mmi_charger_device *chrg_dev;
	chrg_num = chip->mmi_chrg_dev_num;

	for (i = 0; i < chrg_num; i++) {

		switch (i) {
		case PMIC_SW:
			break;
		case CP_MASTER:
		if (is_charger_exist(dev_ops[CP_MASTER].dev_name)) {
				chrg_dev = chrg_list->chrg_dev[CP_MASTER];
				chrg_error_type = chrg_dev->charger_error.chrg_err_type;
			if (chrg_error_type & (1 << MMI_BUS_UCP_ALARM_BIT) ||
				chrg_error_type & (1 << MMI_BUS_UCP_FAULT_BIT)) {
				mmi_chrg_info(chip,"CP master bus ucp error %d, cnt %d,"
						"qc3p volt comp %dmV\n",
						chrg_error_type,
						chrg_dev->charger_error.bus_ucp_err_cnt,
						chip->qc3p_volt_comp);

				if (chrg_dev->charger_error.bus_ucp_err_cnt > 3) {
					if (chrg_list->cp_slave) {
						chrg_list->cp_slave = false;
						chrg_dev->charger_error.bus_ucp_err_cnt = 0;
						mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_CHRG_PUMP_ENTRY);
					} else {
						chip->qc3p_recovery_pmic_chrg = true;
						mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_SW_ENTRY);
						chrg_dev->charger_error.bus_ucp_err_cnt = 0;
					}
				} else if (chrg_dev->charger_error.bus_ucp_err_cnt > 6) {
					chip->qc3p_recovery_pmic_chrg = true;
					mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_SW_ENTRY);
					chrg_dev->charger_error.bus_ucp_err_cnt = 0;
				}

				chrg_dev->charger_error.bus_ucp_err_cnt++;
				chip->qc3p_volt_comp += QC3P_VOLT_COMP_DELTA;
				mmi_chrg_info(chip,"Restart charging, "
						"increase qc3p volt comp %dmV\n",
						chip->qc3p_volt_comp);
				mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_CHRG_PUMP_ENTRY);
				mmi_clear_charger_error(chrg_dev);
			}else if (chrg_error_type & (1 << MMI_BUS_OCP_ALARM_BIT) ||
				chrg_error_type & (1 << MMI_BUS_OCP_FAULT_BIT) ||
				chrg_error_type & (1 << MMI_CONV_OCP_FAULT_BIT)) {
				mmi_chrg_info(chip,"CP master ocp error %d, cnt %d,"
					"qc3p volt comp %dmV\n",
					chrg_error_type,
					chrg_dev->charger_error.bus_ocp_err_cnt,
					chip->qc3p_volt_comp);
				if (chrg_dev->charger_error.bus_ocp_err_cnt > 3) {
					chip->qc3p_recovery_pmic_chrg = true;
					mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_SW_ENTRY);
					chrg_dev->charger_error.bus_ocp_err_cnt = 0;
				}
				chrg_dev->charger_error.bus_ocp_err_cnt++;
				chip->qc3p_volt_comp -= QC3P_VOLT_COMP_DELTA;
				if (chip->qc3p_volt_comp < 0)
					chip->qc3p_volt_comp = 0;
				mmi_chrg_info(chip,"Restart charging, "
						"decrease qc3p volt comp %dmV\n",
						chip->qc3p_volt_comp);
				mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_CHRG_PUMP_ENTRY);
				mmi_clear_charger_error(chrg_dev);
			}
		}
			break;
		case CP_SLAVE:
			break;
		default:
			mmi_chrg_err(chip,"No mmi_chrg_dev found %d !\n",i);
			break;
		}
	}
	return;
}

void mmi_qc3p_chrg_enable_all_cp(struct mmi_charger_manager *chip, int val)
{
	struct mmi_cp_policy_dev *chrg_list = &g_chrg_list;
	bool enable = !!val;

	mmi_chrg_dbg(chip, PR_MOTO,"enable all cp = %d\n", enable);
	if (enable) {
		if (chip->cp_disable == false)
			return;

		cancel_delayed_work_sync(&chip->mmi_qc3p_chrg_sm_work);
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CHRG_PUMP_ENTRY);
		chip->cp_disable = false;
		schedule_delayed_work(&chip->mmi_qc3p_chrg_sm_work,
				msecs_to_jiffies(0));

	} else {
		if(chrg_list->cp_master
			&& !chrg_list->chrg_dev[CP_MASTER]->charger_enabled)
			return;

		cancel_delayed_work_sync(&chip->mmi_qc3p_chrg_sm_work);

		chip->cp_disable = true;
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master
			&& chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
		}

		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_RECOVERY_SW);
		schedule_delayed_work(&chip->mmi_qc3p_chrg_sm_work,
				msecs_to_jiffies(0));
	}
}

#define QC3P_HEARTBEAT_SHORT_DELAY_MS 1000
#define QC3P_HEARTBEAT_lOOP_WAIT_MS 3000
#define QC3P_HEARTBEAT_TUNNING_MS 100
#define QC3P_HEARTBEAT_SHORT_DELAY_MS 1000
#define QC3P_HEARTBEAT_NEXT_STATE_MS 100
#define QC3P_HEARTBEAT_CANCEL -1
#define QC3P_CC_CURR_DEBOUNCE 100000
#define QC3P_CV_TAPPER_COUNT 3
#define QC3P_CC_POWER_COUNT 3
#define QC3P_CV_DELTA_VOLT 100000
#define QC3P_COOLING_DELTA_POWER 100000
#define QC3P_COOLING_MAX_CNT 5
#define QC3P_DISABLE_CHRG_LIMIT -1
#define QC3P_CP_CHRG_SOC_LIMIT 90
#define QC3P_CONT_PWR_CNT 5
#define CP_CV_VOLT_TOLERANCE 10000  //10mV

void qc3p_clear_chg_manager(struct mmi_charger_manager *chip)
{
	mmi_chrg_dbg(chip, PR_INTERRUPT, "clear mmi qc3p chrg manager!\n");
	chip->qc3p_request_volt = 0;
	chip->qc3p_request_volt_prev = 0;
	chip->qc3p_target_volt = 0;
	chip->qc3p_batt_therm_volt = 0;
	chip->qc3p_sys_therm_volt = 0;
	chip->qc3p_recovery_pmic_chrg = false;
	chip->qc3p_sys_therm_cooling= false;
	chip->qc3p_sys_therm_force_pmic_chrg = false;
	chip->qc3p_batt_therm_cooling = false;
	chip->qc3p_batt_therm_cooling_cnt = 0;

}

void mmi_qc3p_chrg_policy_clear(struct mmi_charger_manager *chip) {
	struct mmi_cp_policy_dev *chrg_list = &g_chrg_list;
	chrg_dev_init(chip, &g_chrg_list);
	clear_chrg_dev_error_cnt(chip, &g_chrg_list);
	qc3p_clear_chg_manager(chip);
	if (chrg_list->cp_slave)
		mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
	if (chrg_list->cp_master)
		mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
	mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
		QC3P_DISABLE_CHRG_LIMIT);
	chrg_list->chrg_dev[PMIC_SW]->charger_limited = false;
	qc3p_sm_state = PM_QC3P_STATE_DISCONNECT;
	chip->qc3p_volt_comp = QC3P_INIT_VOLT_COMP;
	qc3p_quit_slave_chrg_cnt = 0;
	qc3p_chrg_cc_power_tunning_cnt = 0;
	qc3p_chrg_cv_taper_tunning_cnt = 0;
	qc3p_chrg_cv_delta_volt = 0;
	qc3p_constant_power_cnt = 0;
	qc3p_batt_curr_roof = 0;
	return;
}

void mmi_qc3p_chrg_sm_work_func(struct work_struct *work)
{
	struct mmi_charger_manager *chip = container_of(work,
				struct mmi_charger_manager, mmi_qc3p_chrg_sm_work.work);
	int i = 0, rc = 0;
	int ibatt_curr = 0, vbatt_volt = 0, batt_temp = 0, vbus_pres = 0;
	int batt_soc = 0;
	int heartbeat_dely_ms = 0;
	int pmic_sys_therm_level = 0;
	bool zone_change = false;
	int ibus_total_ma = 0;
	int cooling_volt = 0;
	bool ignore_hysteresis_degc = false;
	struct mmi_chrg_step_info *chrg_step;
	union power_supply_propval prop = {0,};
	struct mmi_cp_policy_dev *chrg_list = &g_chrg_list;
	int thermal_cooling_current = 0;

	mmi_chrg_dbg(chip, PR_MOTO, "\n\n\n");

	mmi_chrg_dbg(chip, PR_MOTO, "schedule SM work, sm state %s \n",
					pm_qc3p_state_str[qc3p_sm_state]);

	mmi_chrg_dbg(chip, PR_MOTO, "pmic-sw is exist: %d  "
								"cp-master is exist: %d  "
								"cp-slave is exist: %d\n",
								chrg_list->pmic_sw,
								chrg_list->cp_master,
								chrg_list->cp_slave);
	if (!chrg_list->pmic_sw) {
		mmi_chrg_err(chip,"PMIC-SW isn't exist, force quite mmi chrg sm work !\n");
		return;
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &prop);
	if (!rc)
		pmic_sys_therm_level = prop.intval;

	ibatt_curr = chrg_list->chrg_dev[PMIC_SW]->charger_data.ibatt_curr;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_TEMP, &prop);
	if (!rc)
		batt_temp = prop.intval / 10;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (!rc)
		batt_soc = prop.intval;


	if (ibatt_curr < 0)
		ibatt_curr *= -1;


	mmi_update_all_charger_status(chip);
	mmi_update_all_charger_error(chip);

	if (chip->extrn_sense) {
		ibatt_curr = chrg_list->chrg_dev[CP_MASTER]->charger_data.ibatt_curr;
		ibatt_curr *= 1000;
		if (ibatt_curr < 0)
			ibatt_curr *= -1;
	}
	vbatt_volt = chrg_list->chrg_dev[CP_MASTER]->charger_data.vbatt_volt;
	vbatt_volt *= 1000;

	vbus_pres = chrg_list->chrg_dev[PMIC_SW]->charger_data.vbus_pres;
	if (!vbus_pres) {
		for (i = 0; i < 3; i++) {
			mmi_update_all_charger_status(chip);
			vbus_pres = chrg_list->chrg_dev[PMIC_SW]->charger_data.vbus_pres;
			mmi_chrg_info(chip, "Retry check charger status, vbus %d\n", vbus_pres);
			if (vbus_pres) {
				mmi_chrg_info(chip, "Get vbus present , continue to charging\n");
				break;
			}
			msleep(100);
		}
	}

	mmi_chrg_info(chip, "battery current %d\n", ibatt_curr);
	mmi_chrg_info(chip, "battery voltage %d\n", vbatt_volt);
	mmi_chrg_info(chip, "battery temp %d\n", batt_temp);
	mmi_chrg_info(chip, "battery capacity %d\n", batt_soc);

	if (vbus_pres && mmi_is_cable_plugout(chip))
		vbus_pres = 0;

	if (vbus_pres &&
		(qc3p_sm_state == PM_QC3P_STATE_TUNNING_VOLT
		|| qc3p_sm_state == PM_QC3P_STATE_CP_CC_LOOP
		|| qc3p_sm_state == PM_QC3P_STATE_CP_CV_LOOP)) {
		mmi_dump_charger_error(chip, chrg_list->chrg_dev[CP_MASTER]);
		qc3p_chrg_policy_error_recovery(chip, chrg_list);
	}

	if (qc3p_sm_state == PM_QC3P_STATE_CP_CV_LOOP)
		ignore_hysteresis_degc = true;
	zone_change = mmi_find_temp_zone(chip, batt_temp, ignore_hysteresis_degc);
	chrg_step = &chip->chrg_step;

	if (chip->pres_temp_zone == ZONE_COLD
		|| chip->pres_temp_zone == ZONE_HOT
		|| !chrg_list->chrg_dev[PMIC_SW]->charger_enabled
		|| vbatt_volt > chip->batt_ovp_lmt) {

		mmi_chrg_info(chip, "Force stop charging, "
						"pres_temp_zone %d, "
						"pmic charger enabled %d, "
						"vbatt_volt %dmv, "
						"batt ovp limit %dmv\n",
						chip->pres_temp_zone,
						chrg_list->chrg_dev[PMIC_SW]->charger_enabled,
						vbatt_volt, chip->batt_ovp_lmt);
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_STOP_CHARGE);
	}

	if (!vbus_pres) {
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_DISCONNECT);
	} else if (qc3p_sm_state == PM_QC3P_STATE_ENTRY
			|| qc3p_sm_state == PM_QC3P_STATE_STOP_CHARGE) {
		mmi_find_chrg_step(chip, chip->pres_temp_zone, vbatt_volt);
	} else if (qc3p_sm_state == PM_QC3P_STATE_DISCONNECT) {
		mmi_get_input_current_settled(chrg_list->chrg_dev[PMIC_SW],
					&chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
		mmi_find_chrg_step(chip, chip->pres_temp_zone, vbatt_volt);
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_ENTRY);
	} else if (zone_change &&
			chip->pres_temp_zone != ZONE_COLD &&
			chip->pres_temp_zone != ZONE_HOT) {

			if (batt_temp >= chrg_step->temp_c) {
				mmi_chrg_info(chip, "battery temp %d, temp thre %d "
						"Enter into COOLING LOOP !\n",
						batt_temp, chrg_step->temp_c);
				chip->qc3p_batt_therm_cooling = true;
				chip->qc3p_batt_therm_volt = chip->qc3p_request_volt_prev;
			} else if (!chip->qc3p_batt_therm_cooling &&
					!ignore_hysteresis_degc) {
				mmi_chrg_info(chip, "battery temp %d, temp thre %d "
						"Restart select chrg step and temp zone !\n",
						batt_temp, chrg_step->temp_c);
				mmi_find_chrg_step(chip, chip->pres_temp_zone, vbatt_volt);
				mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_ENTRY);
			}
	}

	mmi_chrg_dbg(chip, PR_MOTO, "temp zone %d, is_changed %d, "
					"chrg_step %d, "
					"step cc curr %d, step cv volt %d, "
					"step cv tapper curr %d\n",
					chip->pres_temp_zone, zone_change,
					chrg_step->pres_chrg_step,
					chrg_step->chrg_step_cc_curr,
					chrg_step->chrg_step_cv_volt,
					chrg_step->chrg_step_cv_tapper_curr);
	switch (qc3p_sm_state) {
	case PM_QC3P_STATE_DISCONNECT:
		mmi_chrg_info(chip,"vbus disconnect !, jump to PM_QC3P_STATE_DISCONNECT,"
					"recovery PMIC-SW limitation, and close CP charg\n");
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master
			&& chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
		}

		if (chrg_list->chrg_dev[PMIC_SW]->charger_limited) {
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_chrg_info(chip,"recovery PMIC-SW ichg lmt ,%d uA\n",
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							QC3P_DISABLE_CHRG_LIMIT);
			chrg_list->chrg_dev[PMIC_SW]->charger_limited = false;
		}

		heartbeat_dely_ms = QC3P_HEARTBEAT_CANCEL;
		break;
	case PM_QC3P_STATE_ENTRY:
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master
			&& chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
		}

		if (chip->qc3p_active
			&& chrg_list->cp_master
			&& chrg_step->pres_chrg_step != chip->chrg_step_nums - 1
			&& chrg_step->chrg_step_cc_curr >=
				chrg_list->chrg_dev[CP_MASTER]->charging_curr_min
			&& batt_soc < QC3P_CP_CHRG_SOC_LIMIT) {

			mmi_chrg_dbg(chip, PR_MOTO, "Enter into CHRG PUMP, "
							"vbatt %d uV, "
							"qc3p support %d, "
							"chrg step %d, "
							"chrg step cc curr %d uA, "
							"CP master charging curr min %d uA\n",
							vbatt_volt,
							chip->qc3p_active,
			  				chrg_step->pres_chrg_step,
			  				chrg_step->chrg_step_cc_curr,
			  				chrg_list->chrg_dev[CP_MASTER]->charging_curr_min);
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CHRG_PUMP_ENTRY);

		} else {
			mmi_chrg_dbg(chip, PR_MOTO, "Enter into PMIC switch charging, "
							"the reason is : vbatt %d uV, "
							"qc3p support %d, "
							"chrg step %d\n",
							vbatt_volt,
							chip->qc3p_active,
			  				chrg_step->pres_chrg_step);
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_ENTRY);

		}
		chip->qc3p_batt_therm_cooling = false;
		chip->qc3p_batt_therm_cooling_cnt= 0;
		chip->qc3p_sys_therm_volt=  chip->qc3p_request_volt_prev;
		chip->qc3p_batt_therm_volt = chip->qc3p_request_volt_prev;
		chip->qc3p_ibus_max_volt = chip->qc3p_request_volt_prev;
		chip->qc3p_sys_therm_cooling = false;
		chip->qc3p_sys_therm_force_pmic_chrg = false;
		chip->qc3p_recovery_pmic_chrg = false;
		heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		break;
	case PM_QC3P_STATE_SW_ENTRY:
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master
			&& chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
		}

		if (chrg_list->chrg_dev[PMIC_SW]->charger_limited) {
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_chrg_info(chip, "Recovery PMIC-SW ichg lmt ,%d uA\n",
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							QC3P_DISABLE_CHRG_LIMIT);
			chrg_list->chrg_dev[PMIC_SW]->charger_limited = false;
		}

		if (!chip->dont_rerun_aicl) {
			mmi_chrg_info(chip, "Do an rerun usb AICL for PMIC-SW\n");
			mmi_enable_charging(chrg_list->chrg_dev[PMIC_SW], false);
			msleep(100);
			mmi_enable_charging(chrg_list->chrg_dev[PMIC_SW], true);
		}
		chip->qc3p_request_volt = SWITCH_CHARGER_QC3P_VOLT;
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_LOOP);
		heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		break;
	case PM_QC3P_STATE_SW_LOOP:
		if (chip->qc3p_active
			&& chip->cp_disable == false
			&& chrg_step->pres_chrg_step != chip->chrg_step_nums - 1
			&& chrg_step->chrg_step_cc_curr >=
				chrg_list->chrg_dev[CP_MASTER]->charging_curr_min
			&& !chip->qc3p_recovery_pmic_chrg
			&& !chip->qc3p_sys_therm_force_pmic_chrg
			&& batt_soc < QC3P_CP_CHRG_SOC_LIMIT) {
			mmi_chrg_info(chip, "Enter CP, the reason is : "
							"qc3p support %d, "
							"vbatt %duV, chrg step %d\n",
							chip->qc3p_active,
							vbatt_volt, chrg_step->pres_chrg_step);
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CHRG_PUMP_ENTRY);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		} else {
			mmi_chrg_dbg(chip, PR_MOTO, "Continue to SW charging, "
						"vbatt %d uV, ibatt %d uA\n",
						vbatt_volt, ibatt_curr);
			heartbeat_dely_ms = QC3P_HEARTBEAT_lOOP_WAIT_MS;
		}
		break;
	case PM_QC3P_STATE_CHRG_PUMP_ENTRY:
		mmi_chrg_info(chip,"CP master exist %d, CP slave exist %d !\n",
							chrg_list->cp_master,
							chrg_list->cp_slave);
		if (chrg_list->cp_slave) {
			mmi_chrg_info(chip,"CP slave is exist !\n");
			mmi_chrg_info(chip,"chrg step cc curr %d uA, "
							"CP slave charging curr min %d uA\n",
			  				chrg_step->chrg_step_cc_curr,
			  				chrg_list->chrg_dev[CP_SLAVE]->charging_curr_min);
			if (chrg_step->chrg_step_cc_curr >=
					chrg_list->chrg_dev[CP_SLAVE]->charging_curr_min) {
				mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_DULE_CP_ENTRY);
			} else
				mmi_chrg_qc3p_sm_move_state(chip,
							PM_QC3P_STATE_SINGLE_CP_ENTRY);
		} else {
			mmi_chrg_info(chip,"CP slave isn't exist !\n");
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SINGLE_CP_ENTRY);
		}

		mmi_chrg_info(chip,"Set PMIC SW FCC limits!\n");
		if (!chrg_list->chrg_dev[PMIC_SW]->charger_limited
			&& chrg_list->chrg_dev[PMIC_SW]->charging_curr_limited > 0) {
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							chrg_list->chrg_dev[PMIC_SW]->charging_curr_limited);
			mmi_chrg_info(chip,"Set PMIC-SW ichg lmt ,%d uA\n",
							chrg_list->chrg_dev[PMIC_SW]->charging_curr_limited);
			chrg_list->chrg_dev[PMIC_SW]->charger_limited = true;
		}

		/*Initial setup qc3p request power by the battery voltage*/
		chip->qc3p_request_volt = (2 * vbatt_volt) % 20000;
		chip->qc3p_request_volt = 2 * vbatt_volt - chip->qc3p_request_volt
							+ chip->qc3p_volt_comp;
		mmi_chrg_info(chip,"qc3p init , volt %dmV, volt comp %dmv\n",
			chip->qc3p_request_volt, chip->qc3p_volt_comp);
		chrg_policy_error_clear(chip, chrg_list);
		heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		break;
	case PM_QC3P_STATE_SINGLE_CP_ENTRY:
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_chrg_info(chip,"Disable Slave Charger Pump !\n");
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], true);
			mmi_chrg_info(chip,"Enable Master Charger Pump !\n");
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_TUNNING_VOLT);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		}
		break;
	case PM_QC3P_STATE_DULE_CP_ENTRY:
		if (chrg_list->cp_master) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], true);
			mmi_chrg_info(chip,"Enable Master Charger Pump !\n");
		}

		if (chrg_list->cp_slave) {
			chrg_list->cp_clave_later = true;
			mmi_chrg_info(chip,"For ibus UCP, "
			"delay start Slave Charger Pump in TUNNING_VOLT stage !\n");
		}

		if (chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_TUNNING_VOLT);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		}
		break;
	case PM_QC3P_STATE_TUNNING_VOLT:
		heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		if (chrg_list->cp_slave
			&& chrg_list->cp_clave_later
			&& !chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], true);
			mmi_chrg_info(chip,"Enable Slave Charger Pump !\n");
		}

		if (chrg_list->cp_master
			&& (!chrg_list->chrg_dev[CP_MASTER]->charger_enabled
			|| (chrg_list->chrg_dev[CP_MASTER]->charger_error.chrg_err_type & (1<< MMI_CP_SWITCH_BIT)))) {
			mmi_chrg_info(chip,"CP MASTER was disabled, "
							"Enter into SW directly\n");
			chip->qc3p_volt_comp = QC3P_INIT_VOLT_COMP;
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_ENTRY);
		} else if (vbatt_volt > chrg_step->chrg_step_cv_volt) {
			chip->qc3p_request_volt -= chip->qc3p_volt_steps;
			mmi_chrg_qc3p_sm_move_state(chip,
						PM_QC3P_STATE_CP_CC_LOOP);
			mmi_chrg_info(chip,"Duing the volt going up process, "
						"the chrg step was changed,"
						"stop increase qc3p volt and"
						" Enter into CC stage as soon!\n");
		} else if (chip->qc3p_request_volt + chip->qc3p_volt_steps
				<= chip->qc3p_volt_max
				&& vbatt_volt < chrg_step->chrg_step_cv_volt
				&& ibatt_curr < ((chrg_step->pres_chrg_step == STEP_FIRST) ?
				chrg_step->chrg_step_cc_curr + chip->step_first_curr_comp:
				chrg_step->chrg_step_cc_curr)) {
				chip->qc3p_request_volt += chip->qc3p_volt_steps;
				mmi_chrg_dbg(chip, PR_MOTO, "Increase qc3p volt %d\n",
								chip->qc3p_request_volt);
				heartbeat_dely_ms = QC3P_HEARTBEAT_TUNNING_MS;
		} else {
			mmi_chrg_info(chip,"Enter into CC loop stage !\n");
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CP_CC_LOOP);
		}

		if (ibatt_curr > qc3p_batt_curr_roof) {
			qc3p_batt_curr_roof = ibatt_curr;
		}
		if (ibatt_curr < qc3p_batt_curr_roof)
			qc3p_constant_power_cnt++;
		else
			qc3p_constant_power_cnt = 0;

		if (qc3p_constant_power_cnt > QC3P_CONT_PWR_CNT) {
			mmi_chrg_info(chip,"QC3P adapter was ready in constant power state, "
							"Enter into CC loop stage !\n");
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CP_CC_LOOP);
		}
		mmi_chrg_info(chip, "qc3p_constant_power_cnt %d, "
							"qc3p_batt_curr_roof %d\n",
							qc3p_constant_power_cnt,
							qc3p_batt_curr_roof);
		break;
	case PM_QC3P_STATE_CP_CC_LOOP:
		heartbeat_dely_ms = QC3P_HEARTBEAT_lOOP_WAIT_MS;
		mmi_chrg_dbg(chip, PR_MOTO,
								"Chrg CC loop, chrg_step %d, "
								"vbatt %dmV, ibatt %dmA, "
								"CC target curr %dmA, "
								"next CV target volt %dmV\n ",
								chrg_step->pres_chrg_step,
								vbatt_volt, ibatt_curr,
								chrg_step->chrg_step_cc_curr,
								chrg_step->chrg_step_cv_volt);
		if (chrg_list->cp_master
			&& (!chrg_list->chrg_dev[CP_MASTER]->charger_enabled
			|| (chrg_list->chrg_dev[CP_MASTER]->charger_error.chrg_err_type & (1<< MMI_CP_SWITCH_BIT)))) {
			mmi_chrg_info(chip,"CP MASTER was disabled, Enter into SW directly\n");
			chip->qc3p_volt_comp = QC3P_INIT_VOLT_COMP;
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_ENTRY);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
			goto schedule;
		}

		if (!chip->qc3p_sys_therm_cooling
			&& !chip->qc3p_batt_therm_cooling
			&& qc3p_constant_power_cnt <= QC3P_CONT_PWR_CNT) {
			if (ibatt_curr < chrg_step->chrg_step_cc_curr
				&& chip->qc3p_request_volt < chip->qc3p_volt_max
				&& qc3p_chrg_cc_power_tunning_cnt >=
					QC3P_CC_POWER_COUNT) {
					if (chip->qc3p_request_volt + chip->qc3p_volt_steps
							< chip->qc3p_volt_max) {
						chip->qc3p_request_volt +=
							chip->qc3p_volt_steps;
						mmi_chrg_dbg(chip, PR_MOTO,
									"Request volt increass %duV\n ",
									chip->qc3p_request_volt);
					}
				qc3p_chrg_cc_power_tunning_cnt = 0;
			}else if (ibatt_curr < chrg_step->chrg_step_cc_curr
				&& chip->qc3p_request_volt < chip->qc3p_volt_max) {
				qc3p_chrg_cc_power_tunning_cnt++;
				mmi_chrg_dbg(chip, PR_MOTO,
									"Chrg CC tunning cnt %d\n",
									qc3p_chrg_cc_power_tunning_cnt);
			} else if (ibatt_curr > chrg_step->chrg_step_cc_curr
								+ QC3P_CC_CURR_DEBOUNCE) {
				chip->qc3p_request_volt -= chip->qc3p_volt_steps;
				mmi_chrg_dbg(chip, PR_MOTO,
					"In the CC step , the ibatt is greater than CC curr, "
					"Request volt decreass %duV to remain CC step\n ",
					chip->qc3p_request_volt);
			} else
				qc3p_chrg_cc_power_tunning_cnt = 0;

			if(ibatt_curr < chrg_step->chrg_step_cc_curr)
				heartbeat_dely_ms = QC3P_HEARTBEAT_SHORT_DELAY_MS;
		}

		if (vbatt_volt > chrg_step->chrg_step_cv_volt) {
			if (qc3p_chrg_cv_taper_tunning_cnt >
				QC3P_CV_TAPPER_COUNT) {
				mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CP_CV_LOOP);
				qc3p_chrg_cv_taper_tunning_cnt = 0;
				qc3p_chrg_cv_delta_volt = QC3P_CV_DELTA_VOLT;
				heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
			} else {
				qc3p_chrg_cv_taper_tunning_cnt++;
				mmi_chrg_dbg(chip, PR_MOTO,
								"Chrg CV taper cnt %d, "
								"chrg step cv volt %dmV, "
								"vbatt %dmV\n",
								qc3p_chrg_cv_taper_tunning_cnt,
								chrg_step->chrg_step_cv_volt,
								vbatt_volt);
			}
		} else
			qc3p_chrg_cv_taper_tunning_cnt = 0;

		break;
	case PM_QC3P_STATE_CP_CV_LOOP:
		heartbeat_dely_ms = QC3P_HEARTBEAT_SHORT_DELAY_MS;
		mmi_chrg_dbg(chip, PR_MOTO,
								"Chrg CV loop, chrg_step %d, "
								"vbatt %dmV, ibatt %dmA, "
								"CV target volt %dmV, "
								"CV taper curr %dmA\n ",
								chrg_step->pres_chrg_step,
								vbatt_volt, ibatt_curr,
								chrg_step->chrg_step_cv_volt,
								chrg_step->chrg_step_cv_tapper_curr);
		if (chrg_list->cp_master
			&& (!chrg_list->chrg_dev[CP_MASTER]->charger_enabled
			|| (chrg_list->chrg_dev[CP_MASTER]->charger_error.chrg_err_type & (1<< MMI_CP_SWITCH_BIT)))) {
			mmi_chrg_info(chip,"CP MASTER was disabled, Enter into SW directly\n");
			chip->qc3p_volt_comp = QC3P_INIT_VOLT_COMP;
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_ENTRY);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
			goto schedule;
		}

		if (((vbatt_volt + CP_CV_VOLT_TOLERANCE) >= chrg_step->chrg_step_cv_volt)
			&& ((!chrg_step->last_step &&
				ibatt_curr < chrg_step->chrg_step_cv_tapper_curr)
				|| ibatt_curr < chrg_list->chrg_dev[CP_MASTER]->charging_curr_min)) {
			if (qc3p_chrg_cv_taper_tunning_cnt >= QC3P_CV_TAPPER_COUNT) {
				if (ibatt_curr <
					chrg_list->chrg_dev[CP_MASTER]->charging_curr_min) {
					mmi_chrg_info(chip, "Ready quite CP chrg stage, "
								"and Enter into PMIC switch chrg stage, "
								"chrg step %d, ibatt %dmA\n",
			  					chrg_step->pres_chrg_step, ibatt_curr);
					mmi_find_chrg_step(chip,
							chip->pres_temp_zone, vbatt_volt + CP_CV_VOLT_TOLERANCE);
					mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CP_QUIT);
					heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
				} else {
					if (mmi_find_chrg_step(chip,
							chip->pres_temp_zone, vbatt_volt + CP_CV_VOLT_TOLERANCE)) {
						heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
						mmi_chrg_info(chip,"Jump to next chrg step\n");
						mmi_chrg_qc3p_sm_move_state(chip,
									PM_QC3P_STATE_CP_CC_LOOP);
					} else {
						mmi_chrg_info(chip,"Can't find next chrg step\n");
						mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_CP_QUIT);
					}
				}
				qc3p_chrg_cv_taper_tunning_cnt = 0;
			} else {
				qc3p_chrg_cv_taper_tunning_cnt++;
				mmi_chrg_dbg(chip, PR_MOTO, "chrg cv taper cnt ++, %d\n",
								qc3p_chrg_cv_taper_tunning_cnt);
			}

		}else if (!chip->qc3p_sys_therm_cooling
			&& !chip->qc3p_batt_therm_cooling)  {

			qc3p_chrg_cv_taper_tunning_cnt = 0;
			if (vbatt_volt > chrg_step->chrg_step_cv_volt + 10000) {
				if (qc3p_chrg_cv_delta_volt > 20000)
					chip->qc3p_request_volt -= qc3p_chrg_cv_delta_volt;
				else
					chip->qc3p_request_volt -= 20000;
				mmi_chrg_info(chip,
					"For keeping CV stage, decrease volt %dmV, "
					"cv delta volt %dmV\n",
					chip->qc3p_request_volt, qc3p_chrg_cv_delta_volt);
			} else if (vbatt_volt < chrg_step->chrg_step_cv_volt - 10000) {
				qc3p_chrg_cv_delta_volt -= 20000;
				chip->qc3p_request_volt += 20000;
				mmi_chrg_info(chip,
					"For keeping CV stage, increase volt %dmV, "
					"cv delta volt %dmV\n",
					chip->qc3p_request_volt, qc3p_chrg_cv_delta_volt);
			} else {
					mmi_chrg_dbg(chip, PR_MOTO, "CV loop work well, "
						"keep qc3p power, volt %dmV\n",
						chip->qc3p_request_volt);
			}

		}else {
		/*In this case, qc3p_sys_therm_cooling or qc3p_batt_therm_cooling is ture*/

			if (vbatt_volt > chrg_step->chrg_step_cv_volt + 10000) {
				if (qc3p_chrg_cv_delta_volt > 20000)
					chip->qc3p_request_volt -= qc3p_chrg_cv_delta_volt;
				else
					chip->qc3p_request_volt -= 20000;
				mmi_chrg_info(chip,
					"For keeping CV stage, decrease volt %dmV, "
					"cv delta volt %dmV\n",
					chip->qc3p_request_volt, qc3p_chrg_cv_delta_volt);
			} else {
					mmi_chrg_dbg(chip, PR_MOTO, "CV loop work well, "
							"keep qc3p power, volt %dmV\n",
							chip->qc3p_request_volt);
			}
		}

		if (chrg_list->cp_slave) {
			if (qc3p_quit_slave_chrg_cnt > 3
				&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
				mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
				mmi_chrg_info(chip,"Quit slave chrg, the reason is :ibatt %duA, "
							"CP slave charging curr min %d uA\n",
			  				ibatt_curr,
			  				chrg_list->chrg_dev[CP_SLAVE]->charging_curr_min);
				msleep(100);
				mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], true);
				mmi_chrg_info(chip,"Restart CP master again\n");
			}

			if (ibatt_curr <
					chrg_list->chrg_dev[CP_SLAVE]->charging_curr_min)
				qc3p_quit_slave_chrg_cnt++;
			else
				qc3p_quit_slave_chrg_cnt = 0;
		}
		break;
	case PM_QC3P_STATE_CP_QUIT:
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master
			&& chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
		}

		chip->qc3p_batt_therm_cooling = false;
		chip->qc3p_batt_therm_cooling_cnt = 0;
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_RECOVERY_SW);
		heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		break;
	case PM_QC3P_STATE_RECOVERY_SW:
		heartbeat_dely_ms = QC3P_HEARTBEAT_SHORT_DELAY_MS;
		if (chrg_list->chrg_dev[PMIC_SW]->charger_limited) {
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_chrg_info(chip,"Recovery PMIC-SW ichg lmt ,%d uA\n",
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							QC3P_DISABLE_CHRG_LIMIT);
			chrg_list->chrg_dev[PMIC_SW]->charger_limited = false;

			if (!chip->dont_rerun_aicl) {
				mmi_chrg_info(chip,"Do an rerun usb AICL for PMIC-SW\n");
				mmi_enable_charging(chrg_list->chrg_dev[PMIC_SW], false);
				msleep(100);
				mmi_enable_charging(chrg_list->chrg_dev[PMIC_SW], true);
			}
			chip->qc3p_recovery_pmic_chrg = true;
			qc3p_chrg_cv_taper_tunning_cnt = 0;
		}

		mmi_chrg_info(chip,"ibatt : %dmA, step cc curr : %dmA\n",
						ibatt_curr, chrg_step->chrg_step_cc_curr);
		if (ibatt_curr > chrg_step->chrg_step_cc_curr) {
			chip->qc3p_request_volt -= QC3P_CV_DELTA_VOLT;

			mmi_chrg_dbg(chip, PR_MOTO, "Reduce qc3p volt %dmV\n ",
						chip->qc3p_request_volt);
			heartbeat_dely_ms = QC3P_HEARTBEAT_TUNNING_MS;
			qc3p_chrg_cv_taper_tunning_cnt = 0;
		} else {
			qc3p_chrg_cv_taper_tunning_cnt++;

		}

		if (qc3p_chrg_cv_taper_tunning_cnt > QC3P_CV_TAPPER_COUNT){
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_LOOP);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		}
		break;
	case PM_QC3P_STATE_STOP_CHARGE:
		heartbeat_dely_ms = QC3P_HEARTBEAT_lOOP_WAIT_MS;
		if (chrg_list->cp_slave
			&& chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_SLAVE], false);
		}

		if (chrg_list->cp_master
			&& chrg_list->chrg_dev[CP_MASTER]->charger_enabled) {
			mmi_enable_charging(chrg_list->chrg_dev[CP_MASTER], false);
		}

		if (chrg_list->chrg_dev[PMIC_SW]->charger_limited) {
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_chrg_info(chip,"Recovery PMIC-SW ichg lmt ,%d uA\n",
							chrg_list->chrg_dev[PMIC_SW]->input_curr_setted);
			mmi_set_charing_current(chrg_list->chrg_dev[PMIC_SW],
							QC3P_DISABLE_CHRG_LIMIT);
			chrg_list->chrg_dev[PMIC_SW]->charger_limited = false;
		}

		if (chip->pres_temp_zone != ZONE_COLD
		&& chip->pres_temp_zone != ZONE_HOT
		&& chrg_list->chrg_dev[PMIC_SW]->charger_enabled
		&& chrg_step->chrg_step_cc_curr > 0) {
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_ENTRY);
			heartbeat_dely_ms = QC3P_HEARTBEAT_NEXT_STATE_MS;
		}
		chip->qc3p_request_volt = SWITCH_CHARGER_QC3P_VOLT;
		break;
	}

schedule:

	chip->qc3p_target_volt = min(chip->qc3p_request_volt, chip->qc3p_volt_max);

	if (chip->system_thermal_level == 0) {
		chip->qc3p_sys_therm_cooling= false;
		chip->qc3p_sys_therm_force_pmic_chrg = false;
	} else if ((chip->system_thermal_level == chip->thermal_levels - 1)
	&& !chip->qc3p_sys_therm_force_pmic_chrg) {
		chip->qc3p_sys_therm_cooling = true;
		chip->qc3p_sys_therm_force_pmic_chrg = true;
		mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_SW_ENTRY);
		mmi_chrg_info(chip, "Thermal is the highest, level %d, "
						"Force enter into single pmic charging !\n",
						chip->system_thermal_level);

	} else if (chip->system_thermal_level > 0 &&
		(qc3p_sm_state == PM_QC3P_STATE_CP_CC_LOOP ||
		qc3p_sm_state == PM_QC3P_STATE_CP_CV_LOOP)) {

		mmi_chrg_dbg(chip, PR_MOTO, "Thermal level is %d\n",
								chip->system_thermal_level);
		if (!chip->qc3p_sys_therm_cooling) {
			chip->qc3p_sys_therm_cooling = true;
			chip->qc3p_sys_therm_volt = chip->qc3p_request_volt_prev;
		}

		thermal_cooling_current = min(chip->thermal_mitigation[chip->system_thermal_level],chrg_step->chrg_step_cc_curr);
		if (ibatt_curr >
			thermal_cooling_current + QC3P_CC_CURR_DEBOUNCE) {
			if (ibatt_curr - thermal_cooling_current > 300000)
			  chip->qc3p_sys_therm_volt -= chip->qc3p_volt_steps *3;
			else
				chip->qc3p_sys_therm_volt -= chip->qc3p_volt_steps;
			mmi_chrg_dbg(chip, PR_MOTO, "For thermal, decrease qc3 volt %d\n",
								chip->qc3p_sys_therm_volt);
		} else if ((ibatt_curr <
			thermal_cooling_current)) {
			if (thermal_cooling_current - ibatt_curr > 300000)
				chip->qc3p_sys_therm_volt += chip->qc3p_volt_steps *3;
			else
				chip->qc3p_sys_therm_volt += chip->qc3p_volt_steps;
			chip->qc3p_sys_therm_volt = min(chip->qc3p_volt_max, chip->qc3p_sys_therm_volt);
		       mmi_chrg_dbg(chip, PR_MOTO, "For thermal, increase qc3 volt %d\n",
						chip->qc3p_sys_therm_volt);
		}

		heartbeat_dely_ms = QC3P_HEARTBEAT_TUNNING_MS;
	} else if (chip->system_thermal_level > 0 &&
		chip->system_thermal_level != chip->thermal_levels - 1 &&
		qc3p_sm_state == PM_QC3P_STATE_SW_LOOP &&
		chip->qc3p_sys_therm_force_pmic_chrg) {
			mmi_chrg_dbg(chip, PR_MOTO, "Try to recovery charger pump!\n");
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_ENTRY);
	}


	if (chip->qc3p_sys_therm_cooling){
		mmi_chrg_dbg(chip, PR_MOTO, "at thermal cooling status ,judge  ibatt_curr:%d, thermal_current_limited:%d ,chrg_step_cc_curr:%d,thermal_cooling_curr:%d\n",
						 ibatt_curr, chip->thermal_mitigation[chip->system_thermal_level],chrg_step->chrg_step_cc_curr,thermal_cooling_current);
		mmi_chrg_dbg(chip, PR_MOTO, "at thermal cooling status ,qc3p_target_volt:%d, qc3p_sys_therm_volt:%d\n", chip->qc3p_target_volt, chip->qc3p_sys_therm_volt);
		if(qc3p_sm_state == PM_QC3P_STATE_CP_CC_LOOP )
			chip->qc3p_target_volt = chip->qc3p_sys_therm_volt ;
		else
			chip->qc3p_target_volt = min(chip->qc3p_target_volt, chip->qc3p_sys_therm_volt);

		chip->qc3p_request_volt = chip->qc3p_target_volt;
		chip->qc3p_sys_therm_volt = chip->qc3p_target_volt;
		heartbeat_dely_ms = QC3P_HEARTBEAT_SHORT_DELAY_MS;
	}

	if (chip->qc3p_batt_therm_cooling && !chip->qc3p_sys_therm_force_pmic_chrg) {
		if (batt_temp > chrg_step->temp_c + COOLING_HYSTERISIS_DEGC) {

			mmi_chrg_info(chip,"Batt temp %d, Cooling loop failed, "
							"force enter PM_QC3P_STATE_ENTRY "
							"restart this charger process !\n",
						batt_temp);
			chip->qc3p_batt_therm_cooling = false;
			chip->qc3p_batt_therm_cooling_cnt= 0;
			mmi_find_chrg_step(chip, chip->pres_temp_zone,
										vbatt_volt);
			mmi_chrg_qc3p_sm_move_state(chip,
						PM_QC3P_STATE_ENTRY);

		} else if (batt_temp > chrg_step->temp_c) {
			cooling_volt = (2 * vbatt_volt) % 20000;
			cooling_volt = 2 * vbatt_volt - cooling_volt
					+ chip->qc3p_volt_comp;
 			if (ibatt_curr > TYPEC_HIGH_CURRENT_UA
				&& chip->qc3p_batt_therm_volt > cooling_volt) {
				chip->qc3p_batt_therm_volt -= QC3P_COOLING_DELTA_POWER;
				mmi_chrg_info(chip, "Do chrg power cooling"
					"qc3p request volt %dmA, "
					"battery temp %d\n",
					chip->qc3p_batt_therm_volt, batt_temp);
			} else {
				if (chip->qc3p_batt_therm_cooling_cnt > QC3P_COOLING_MAX_CNT) {
					mmi_chrg_info(chip, "Do chrg power cooling failed"
						"qc3p_batt_therm_volt %dmV"
						"battery temp %d, "
						"Restart PM_STATE_ENTRY !\n",
						chip->qc3p_batt_therm_volt,
						batt_temp);
					mmi_find_chrg_step(chip, chip->pres_temp_zone,
											vbatt_volt);
					chip->qc3p_batt_therm_cooling = false;
					chip->qc3p_batt_therm_cooling_cnt = 0;
					mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_ENTRY);
				} else {
					mmi_chrg_info(chip, "It's already the lowest cooling chrg power"
						"waiting for a while, cooling cnt %d, "
						"battery temp %d\n",
						batt_temp, chip->qc3p_batt_therm_cooling);
						chip->qc3p_batt_therm_cooling_cnt++;
					}
			}
		} else if (batt_temp < chrg_step->temp_c - COOLING_HYSTERISIS_DEGC) {

			mmi_chrg_info(chip,"Batt temp %d, "
							"Exit successfully from COOLing loop!\n",
							batt_temp);

			if (qc3p_sm_state == PM_QC3P_STATE_CP_CC_LOOP) {
				mmi_chrg_info(chip,"Jump into CURR tunning"
								"for chrg power poerformance!\n");
					mmi_chrg_qc3p_sm_move_state(chip,
						PM_QC3P_STATE_TUNNING_VOLT);
			}
			chip->qc3p_batt_therm_cooling = false;
			chip->qc3p_batt_therm_cooling_cnt = 0;
		}
		heartbeat_dely_ms = QC3P_HEARTBEAT_lOOP_WAIT_MS;
	}

	if (chip->qc3p_batt_therm_cooling)
		chip->qc3p_target_volt = min(chip->qc3p_target_volt, chip->qc3p_batt_therm_volt);

	ibus_total_ma = chrg_list->chrg_dev[PMIC_SW]->charger_data.ibus_curr / 1000 + chrg_list->chrg_dev[CP_MASTER]->charger_data.ibus_curr;
	if (chrg_list->cp_slave && chrg_list->chrg_dev[CP_SLAVE]->charger_enabled) {
		ibus_total_ma += chrg_list->chrg_dev[CP_SLAVE]->charger_data.ibus_curr;
	}

	if(ibus_total_ma > chip->qc3p_max_ibus_ma) {
		chip->qc3p_ibus_max_volt -= 2 * chip->qc3p_volt_steps;
		chip->qc3p_target_volt = min(chip->qc3p_target_volt, chip->qc3p_ibus_max_volt);
		chip->qc3p_request_volt = chip->qc3p_target_volt;
		mmi_chrg_info(chip,"Limit ibus current: ibus is %dmA, max allow is %dmA, ibus volt is %duV, target volt is %duV \n",
			ibus_total_ma, chip->qc3p_max_ibus_ma, chip->qc3p_ibus_max_volt, chip->qc3p_target_volt);
	}

	mmi_chrg_dbg(chip, PR_MOTO, "chrg sm work,%s, "
								"battery soc %d, "
								"battery temp %d, "
								"battery current %d, "
								"battery voltage %d, "
								"sys therm level %d, "
								"pmic therm level %d, "
								"sys therm cooling %d, "
								"batt therm cooling %d, "
								"sys therm force pmic chrg %d, "
								"recovery pmic chrg %d",
								pm_qc3p_state_str[qc3p_sm_state],
								batt_soc, batt_temp,
								ibatt_curr, vbatt_volt,
								chip->system_thermal_level,
								pmic_sys_therm_level,
								chip->qc3p_sys_therm_cooling,
								chip->qc3p_batt_therm_cooling,
								chip->qc3p_sys_therm_force_pmic_chrg,
								chip->qc3p_recovery_pmic_chrg);

	mmi_chrg_dbg(chip, PR_MOTO, 	"qc3p request volt %dmV, "
								"qc3p target volt %dmV, "
								"sys therm volt %dmV, "
								"batt therm volt %dmV,"
								"qc3p_ibus_max_volt  %dmV,"
								"qc3p_max_ibus_ma  %dmA,"
								"ibus total %dmA\n",
								chip->qc3p_request_volt,
								chip->qc3p_target_volt,
								chip->qc3p_sys_therm_volt,
								chip->qc3p_batt_therm_volt,
								chip->qc3p_ibus_max_volt,
								chip->qc3p_max_ibus_ma,
								ibus_total_ma);

	if (chip->qc3p_target_volt < SWITCH_CHARGER_QC3P_VOLT) {

		if (qc3p_sm_state == PM_QC3P_STATE_TUNNING_VOLT
			|| qc3p_sm_state == PM_QC3P_STATE_CP_CC_LOOP
			|| qc3p_sm_state == PM_QC3P_STATE_CP_CV_LOOP) {

			mmi_chrg_err(chip, "%s, wrong qc3p voltage or current , "
						"request qc3p voltage %d, ",
						pm_qc3p_state_str[qc3p_sm_state],
						chip->qc3p_target_volt);
			mmi_chrg_qc3p_sm_move_state(chip, PM_QC3P_STATE_ENTRY);
		}
		goto skip_pd_select;
	}

	 rc = mmi_qc3p_set_vbus_voltage(chip, chip->qc3p_target_volt / 1000);
	if (rc > 0) {
		chip->qc3p_request_volt_prev = chip->qc3p_target_volt;
		chip->qc3p_ibus_max_volt = chip->qc3p_target_volt;
	}
skip_pd_select:

	if (heartbeat_dely_ms > 0) {
		mmi_chrg_err(chip, "schedule work timer %dms\n", heartbeat_dely_ms);
		schedule_delayed_work(&chip->mmi_qc3p_chrg_sm_work,
				msecs_to_jiffies(heartbeat_dely_ms));
	} else {
		mmi_qc3p_chrg_policy_clear(chip);
		chip->sm_work_running = false;
		chip->qc3p_active = false;
		mmi_chrg_err(chip, "exit sm work\n");
	}

	return;
}


