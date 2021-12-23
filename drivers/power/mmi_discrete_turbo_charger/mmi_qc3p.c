/*
 * Copyright (c) 2012 Motorola Mobility, LLC.
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
#include "mmi_charger_core.h"
#include "mmi_charger_core_iio.h"
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

bool mmi_qc3p_start_detection(struct mmi_charger_manager *chg)
{
	int ret, iio_val;

	ret = mmi_charger_read_iio_chan(chg, SMB5_QC3P_START_DETECT, &iio_val);
	if (ret < 0) {
		mmi_chrg_err(chg, "Couldn't operate qc3p detection rc=%d\n", ret);
		return false;
	}
	return true;
}

bool mmi_qc3p_power_active(struct mmi_charger_manager *chg)
{
	int ret, iio_val;

	ret = mmi_charger_read_iio_chan(chg, SMB5_USB_REAL_TYPE, &iio_val);
	if (ret < 0) {
		mmi_chrg_err(chg, "Couldn't read charger type rc=%d\n", ret);
		return false;
	}
	if (iio_val != QTI_POWER_SUPPLY_QC3P_27W && iio_val != QTI_POWER_SUPPLY_QC3P_45W)
		return false;

	ret = mmi_charger_read_iio_chan(chg, SMB5_QC3P_POWER, &iio_val);
	if (ret < 0) {
		mmi_chrg_err(chg, "Couldn't read qc3p power rc=%d\n", ret);
		return false;
	}
	chg->qc3p_power =  iio_val;
	mmi_chrg_info(chg, "qc3p_power =%d\n", chg->qc3p_power);

	if (chg->qc3p_power == QTI_POWER_SUPPLY_QC3P_27W ||
		chg->qc3p_power == QTI_POWER_SUPPLY_QC3P_45W)
		return true;
	else
		return false;
}

int mmi_qc3p_set_vbus_voltage(struct mmi_charger_manager *chg, int target_mv)
{
	int rc = -EINVAL;
	int  curr_vbus_mv = 0, step = 0,curr_vbus_ma = 0;
	int  impendance_vubs_volt = 0;
	int sleep_ms_wt = 0;

	rc = mmi_get_vbus(chg->chrg_list[CP_MASTER],
							&curr_vbus_mv);
	if (rc) {
		mmi_chrg_err(chg, "Unable to read USB voltage: %d\n", rc);
		return -EINVAL;
	}

	rc = mmi_get_input_current(chg->chrg_list[CP_MASTER],
							&curr_vbus_ma);
	if (rc) {
		mmi_chrg_err(chg, "Unable to read USB ma: %d\n", rc);
		return -EINVAL;
	}

//	impendance_vubs_volt = (DEFAULT_VBUS_RESISTANCE * curr_vbus_ma) / 1000;
	impendance_vubs_volt = 0;//200;
	curr_vbus_mv -= impendance_vubs_volt ;

	if(target_mv < curr_vbus_mv) {
		step = (curr_vbus_mv - target_mv) / 20;
			step = step *-1;
	} else {
		step = (target_mv - curr_vbus_mv) / 20;
	}

	mmi_chrg_info(chg, "curr_vbus= %d, target_vbus = %d, step = %d, impent_v:%d\n",curr_vbus_mv, target_mv, step,impendance_vubs_volt);
	rc = mmi_charger_write_iio_chan(chg, SMB5_DP_DM, step);
	if (rc < 0) {
		mmi_chrg_err(chg, "Couldn't set dpdm pulse rc=%d\n", rc);
		return -EINVAL;
	}

	//need wait dp/dm send out
	sleep_ms_wt = abs(step) * 10;
	msleep(sleep_ms_wt);

	rc = mmi_get_vbus(chg->chrg_list[CP_MASTER],
							&curr_vbus_mv);
	if (rc) {
		mmi_chrg_err(chg, "Unable to read VBUS: %d\n", rc);
		return -EINVAL;
	}
	mmi_chrg_info(chg, "result_vbus= %d\n",curr_vbus_mv);
	return curr_vbus_mv;
}