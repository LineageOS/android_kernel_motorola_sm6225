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
#include <linux/qti_power_supply.h>

bool mmi_qc3p_power_active(struct mmi_charger_manager *chg)
{
	int ret, iio_val;

	ret = mmi_charger_read_iio_chan(chg, SMB5_USB_REAL_TYPE, &iio_val);
	if (ret < 0) {
		mmi_chrg_err(chg, "Couldn't read charger type rc=%d\n", ret);
		return false;
	}
	if (iio_val != QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3P5)
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

#define QC3P_PULSE_COUNT_MAX 	((11000 - 5000) / 20 + 10)
int mmi_qc3p_set_vbus_voltage(struct mmi_charger_manager *chg, int target_mv)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };
	int pulse_cnt, curr_vbus_mv, step, i;

	rc = mmi_charger_read_iio_chan(chg, SMB5_DP_DM, &val.intval);
	if (rc < 0) {
		mmi_chrg_err(chg, "Couldn't read dpdm pulse count rc=%d\n", rc);
		return -EINVAL;
	} else {
		mmi_chrg_info(chg, "DP DM pulse count = %d\n", val.intval);
		pulse_cnt = val.intval;
	}

	rc = mmi_get_vbus(chg->chrg_list[CP_MASTER],
							&curr_vbus_mv);
	if (rc) {
		mmi_chrg_err(chg, "Unable to read USB voltage: %d\n", rc);
		return -EINVAL;
	}

	if(target_mv < curr_vbus_mv) {
		step = (curr_vbus_mv - target_mv) / 20;
		val.intval = QTI_POWER_SUPPLY_DP_DM_DM_PULSE;
		if (pulse_cnt <= step)
			step = pulse_cnt;
	} else {
		step = (target_mv - curr_vbus_mv) / 20;
		val.intval = QTI_POWER_SUPPLY_DP_DM_DP_PULSE;
		if (step + pulse_cnt > QC3P_PULSE_COUNT_MAX)
			step = QC3P_PULSE_COUNT_MAX - pulse_cnt;
	}

	mmi_chrg_info(chg, "curr_vbus= %d, target_vbus = %d, step = %d\n",curr_vbus_mv, target_mv, step);
	for (i = 0; i < step; i++) {
		rc = mmi_charger_write_iio_chan(chg, SMB5_DP_DM, val.intval);
		if (rc < 0) {
			mmi_chrg_err(chg, "Couldn't set dpdm pulse rc=%d\n", rc);
			break;
		}

		if (i < step - 1)
			udelay(5000);
	}

	rc = mmi_get_vbus(chg->chrg_list[CP_MASTER],
							&curr_vbus_mv);
	if (rc) {
		mmi_chrg_err(chg, "Unable to read VBUS: %d\n", rc);
		return -EINVAL;
	}
	mmi_chrg_info(chg, "result_vbus= %d\n",curr_vbus_mv);
	return curr_vbus_mv;
}
