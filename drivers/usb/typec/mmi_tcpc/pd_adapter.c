/* Copyright (c)  2020, 2021 The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/usb/typec.h>
#include <linux/version.h>

#include "inc/tcpci_typec.h"
#include <linux/usb/adapter_class.h>

struct mmi_pd_adapter_info {
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	struct adapter_device *adapter_dev;
	struct task_struct *adapter_task;
	const char *adapter_dev_name;
	bool enable_kpoc_shdn;
	int pd_type;
};

static int pd_get_property(struct adapter_device *dev,
	enum adapter_property sta)
{
	struct mmi_pd_adapter_info *info;

	info = (struct mmi_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return -1;

	switch (sta) {
	case TYPEC_RP_LEVEL:
		{
			return tcpm_inquire_typec_remote_rp_curr(info->tcpc);
		}
		break;
	case PD_TYPE:
		{
			return info->pd_type;
		}
		break;
	default:
		{
		}
		break;
	}
	return -1;
}

static int pd_set_cap(struct adapter_device *dev, enum adapter_cap_type type,
		int pdo_idx, int mV, int mA)
{
	int ret = MMI_ADAPTER_OK;
	int tcpm_ret = TCPM_SUCCESS;
	struct mmi_pd_adapter_info *info;

	pr_notice("[%s] type:%d mV:%d mA:%d\n",
		__func__, type, mV, mA);

	pdo_idx = 0;
	info = (struct mmi_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL) {
		pr_notice("[%s] info null\n", __func__);
		return -1;
	}

	if (type == MMI_PD_APDO_START) {
		tcpm_ret = tcpm_set_apdo_charging_policy(info->tcpc,
			DPM_CHARGING_POLICY_PPS, mV, mA, NULL);
	} else if (type == MMI_PD_APDO_END) {
		tcpm_ret = tcpm_reset_pd_charging_policy(info->tcpc, NULL);
	} else if (type == MMI_PD_APDO) {
		tcpm_ret = tcpm_dpm_pd_request(info->tcpc, mV, mA, NULL);
	} else if (type == MMI_PD_FIXED) {
		tcpm_ret = tcpm_dpm_pd_request(info->tcpc, mV,
					mA, NULL);
	}

	pr_notice("[%s] type:%d mV:%d mA:%d ret:%d\n",
		__func__, type, mV, mA, tcpm_ret);


	if (tcpm_ret == TCP_DPM_RET_REJECT)
		return MMI_ADAPTER_REJECT;
	else if (tcpm_ret != 0)
		return MMI_ADAPTER_ERROR;

	return ret;
}

int pd_get_output(struct adapter_device *dev, int *mV, int *mA)
{
	int ret = MMI_ADAPTER_OK;
	int tcpm_ret = TCPM_SUCCESS;
	struct pd_pps_status pps_status;
	struct mmi_pd_adapter_info *info;

	info = (struct mmi_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return MMI_ADAPTER_NOT_SUPPORT;


	tcpm_ret = tcpm_dpm_pd_get_pps_status(info->tcpc, NULL, &pps_status);
	if (tcpm_ret == TCP_DPM_RET_NOT_SUPPORT)
		return MMI_ADAPTER_NOT_SUPPORT;
	else if (tcpm_ret != 0)
		return MMI_ADAPTER_ERROR;

	*mV = pps_status.output_mv;
	*mA = pps_status.output_ma;

	return ret;
}

int pd_get_status(struct adapter_device *dev,
	struct adapter_status *sta)
{
	struct pd_status TAstatus = {0,};
	int ret = MMI_ADAPTER_OK;
	int tcpm_ret = TCPM_SUCCESS;
	struct mmi_pd_adapter_info *info;

	info = (struct mmi_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return MMI_ADAPTER_ERROR;

	tcpm_ret = tcpm_dpm_pd_get_status(info->tcpc, NULL, &TAstatus);

	sta->temperature = TAstatus.internal_temp;
	sta->ocp = TAstatus.event_flags & PD_STASUS_EVENT_OCP;
	sta->otp = TAstatus.event_flags & PD_STATUS_EVENT_OTP;
	sta->ovp = TAstatus.event_flags & PD_STATUS_EVENT_OVP;

	if (tcpm_ret == TCP_DPM_RET_NOT_SUPPORT)
		return MMI_ADAPTER_NOT_SUPPORT;
	else if (tcpm_ret == TCP_DPM_RET_TIMEOUT)
		return MMI_ADAPTER_TIMEOUT;
	else if (tcpm_ret == TCP_DPM_RET_SUCCESS)
		return MMI_ADAPTER_OK;
	else
		return MMI_ADAPTER_ERROR;

	return ret;

}

static int pd_get_cap(struct adapter_device *dev,
	enum adapter_cap_type type,
	struct adapter_power_cap *tacap)
{
	struct tcpm_power_cap_val apdo_cap;
	struct tcpm_remote_power_cap pd_cap;
	struct pd_source_cap_ext cap_ext;

	uint8_t cap_i = 0;
	int ret;
	int idx = 0;
	unsigned int i, j;
	struct mmi_pd_adapter_info *info;

	info = (struct mmi_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return MMI_ADAPTER_ERROR;

	if (type == MMI_PD_APDO) {
		while (1) {
			ret = tcpm_inquire_pd_source_apdo(info->tcpc,
					TCPM_POWER_CAP_APDO_TYPE_PPS,
					&cap_i, &apdo_cap);
			if (ret == TCPM_ERROR_NOT_FOUND) {
				break;
			} else if (ret != TCPM_SUCCESS) {
				pr_notice("[%s] tcpm_inquire_pd_source_apdo failed(%d)\n",
					__func__, ret);
				break;
			}

			ret = tcpm_dpm_pd_get_source_cap_ext(info->tcpc,
					NULL, &cap_ext);
			if (ret == TCPM_SUCCESS)
				tacap->pdp = cap_ext.source_pdp;
			else {
				tacap->pdp = 0;
				pr_notice("[%s] tcpm_dpm_pd_get_source_cap_ext failed(%d)\n",
					__func__, ret);
			}

			tacap->pwr_limit[idx] = apdo_cap.pwr_limit;
			/* If TA has PDP, we set pwr_limit as true */
			if (tacap->pdp > 0 && !tacap->pwr_limit[idx])
				tacap->pwr_limit[idx] = 1;
			tacap->ma[idx] = apdo_cap.ma;
			tacap->max_mv[idx] = apdo_cap.max_mv;
			tacap->min_mv[idx] = apdo_cap.min_mv;
			tacap->maxwatt[idx] = apdo_cap.max_mv * apdo_cap.ma;
			tacap->minwatt[idx] = apdo_cap.min_mv * apdo_cap.ma;
			tacap->type[idx] = MMI_PD_APDO;

			idx++;
			pr_notice("pps_boundary[%d], %d mv ~ %d mv, %d ma pl:%d\n",
				cap_i,
				apdo_cap.min_mv, apdo_cap.max_mv,
				apdo_cap.ma, apdo_cap.pwr_limit);
			if (idx >= ADAPTER_CAP_MAX_NR) {
				pr_notice("CAP NR > %d\n", ADAPTER_CAP_MAX_NR);
				break;
			}
		}
		tacap->nr = idx;

		for (i = 0; i < tacap->nr; i++) {
			pr_notice("pps_cap[%d:%d], %d mv ~ %d mv, %d ma pl:%d pdp:%d\n",
				i, (int)tacap->nr, tacap->min_mv[i],
				tacap->max_mv[i], tacap->ma[i],
				tacap->pwr_limit[i], tacap->pdp);
		}

		if (cap_i == 0)
			pr_notice("no APDO for pps\n");

	} else if (type == MMI_PD_FIXED) {
		pd_cap.nr = 0;
		pd_cap.selected_cap_idx = 0;
		tcpm_get_remote_power_cap(info->tcpc, &pd_cap);

		if (pd_cap.nr != 0) {

			tacap->selected_cap_idx = pd_cap.selected_cap_idx - 1;
			pr_notice("[%s] nr:%d idx:%d\n",
			__func__, pd_cap.nr, pd_cap.selected_cap_idx - 1);

			j = 0;
			pr_notice("adapter cap: nr:%d\n", pd_cap.nr);
			for (i = 0; i < pd_cap.nr; i++) {
				if (pd_cap.type[i] == TCPM_POWER_CAP_VAL_TYPE_FIXED &&
					j >= 0 &&
					j < ADAPTER_CAP_MAX_NR) {
					tacap->type[j] = MMI_PD_FIXED;
					tacap->ma[j] = pd_cap.ma[i];
					tacap->max_mv[j] = pd_cap.max_mv[i];
					tacap->min_mv[j] = pd_cap.min_mv[i];
					tacap->maxwatt[j] =
					tacap->max_mv[j] * tacap->ma[i];
					tacap->minwatt[j] =
					tacap->min_mv[j] * tacap->ma[i];
					j++;
				}

				pr_notice("[%s]:%d mv:[%d,%d] mA:%d type:%d %d\n",
					__func__, i, pd_cap.min_mv[i],
					pd_cap.max_mv[i], pd_cap.ma[i],
					pd_cap.type[i], pd_cap.type[i]);
			}

			tacap->nr = j;
			pr_notice("pd cap: nr:%d\n", tacap->nr);
			for (i = 0; i < tacap->nr; i++) {
				pr_notice("[%s]:%d mv:[%d,%d] mA:%d max:%d min:%d type:%d %d\n",
					__func__, i, tacap->min_mv[i],
					tacap->max_mv[i], tacap->ma[i],
					tacap->maxwatt[i], tacap->minwatt[i],
					tacap->type[i], tacap->type[i]);
			}
		}
	} else if (type == MMI_PD_ALL) {
		pd_cap.nr = 0;
		pd_cap.selected_cap_idx = 0;
		tcpm_get_remote_power_cap(info->tcpc, &pd_cap);
		tacap->nr = pd_cap.nr;

		if (pd_cap.nr != 0) {

			tacap->selected_cap_idx = pd_cap.selected_cap_idx - 1;
			pr_notice("[%s] nr:%d idx:%d\n",
			__func__, pd_cap.nr, pd_cap.selected_cap_idx - 1);

			for (i = 0; i < pd_cap.nr; i++) {
				tacap->ma[i] = pd_cap.ma[i];
				tacap->max_mv[i] = pd_cap.max_mv[i];
				tacap->min_mv[i] = pd_cap.min_mv[i];
				tacap->maxwatt[i] =
					tacap->max_mv[i] * tacap->ma[i];
				if (pd_cap.type[i] == TCPM_POWER_CAP_VAL_TYPE_FIXED)
					tacap->type[i] = MMI_PD_FIXED;
				else if (pd_cap.type[i] == TCPM_POWER_CAP_VAL_TYPE_AUGMENT)
					tacap->type[i] = MMI_PD_APDO;
				else
					tacap->type[i] = MMI_CAP_TYPE_UNKNOWN;

				pr_notice("[%s]:%d mv:[%d,%d] %d max:%d min:%d type:%d %d\n",
					__func__, i, tacap->min_mv[i],
					tacap->max_mv[i], tacap->ma[i],
					tacap->maxwatt[i], tacap->minwatt[i],
					tacap->type[i], pd_cap.type[i]);
			}
		}
	}

	return MMI_ADAPTER_OK;
}

static struct adapter_ops adapter_ops = {
	.get_status = pd_get_status,
	.set_cap = pd_set_cap,
	.get_output = pd_get_output,
	.get_property = pd_get_property,
	.get_cap = pd_get_cap,
};

int pd_adapter_create(struct platform_device *pdev)
{
	int ret = 0;
	struct mmi_pd_adapter_info *info = NULL;
	static bool is_deferred;

	pr_info("%s\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(struct mmi_pd_adapter_info),
			GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->adapter_dev = adapter_device_register("pd_adapter",
		&pdev->dev, info, &adapter_ops, NULL);
	if (IS_ERR_OR_NULL(info->adapter_dev)) {
		pr_info("%s: adapter_device_register err\n", __func__);
		ret = PTR_ERR(info->adapter_dev);
		goto err_register_adapter_dev;
	}

	adapter_dev_set_drvdata(info->adapter_dev, info);

	info->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (info->tcpc == NULL) {
		if (is_deferred == false) {
			pr_info("%s: tcpc device not ready, defer\n", __func__);
			is_deferred = true;
			ret = -EPROBE_DEFER;
		} else {
			pr_info("%s: failed to get tcpc device\n", __func__);
			ret = -EINVAL;
		}
		goto err_get_tcpc_dev;
	}

	return 0;

err_get_tcpc_dev:
	adapter_device_unregister(info->adapter_dev);
err_register_adapter_dev:
	devm_kfree(&pdev->dev, info);

	return ret;
}
EXPORT_SYMBOL(pd_adapter_create);
