/*
 * Copyright (C) 2019 Motorola Mobility LLC
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
#include <linux/gpio.h>
#include <linux/touchscreen_mmi.h>
#include <linux/regulator/consumer.h>
#include "syna_tcm2_runtime.h"
#include "syna_tcm2_platform.h"
#include "syna_tcm2.h"
#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_reflash.h"

static int syna_ts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "synaptics");
}

static int syna_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if(!dev){
		LOGE("dev is null");
	}
	if(!dev->p){
		LOGE("dev->p is null");
	}
	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	return scnprintf(TO_CHARP(cdata), 6, "%s", tcm->tcm_dev->id_info.part_number);
}

static int syna_ts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	unsigned int buildid;
	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	buildid = tcm->tcm_dev->packrat_number;
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%d", buildid);
}

/*return firmware version*/
static int syna_ts_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%s", tcm->tcm_dev->app_info.customer_config_id);
}

static int syna_ts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	const struct syna_hw_interface *hw_if;
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	hw_if = tcm->hw_if;
	if (!hw_if) {
		LOGE("Hardware interface not found\n");
		return -ENODEV;
	}
	TO_INT(idata) = hw_if->bdata_io.type;
	return 0;
}

static int syna_ts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	struct syna_hw_interface *hw_if;
	struct syna_hw_attn_data *attn;
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	if (!pdev) {
		LOGE("Failed to get platform device");
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	hw_if = tcm->hw_if;
	if (!hw_if) {
		LOGE("Hardware interface not found\n");
		return -ENODEV;
	}	
	attn = &hw_if->bdata_attn;
	if (!attn) {
		LOGE("Failed to get ts attn");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(attn->irq_gpio);
	return 0;
}

static int syna_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	struct syna_hw_attn_data *attn;
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	attn = &tcm->hw_if->bdata_attn;
	if (!attn) {
		LOGE("Failed to get ts attn");
		return -ENODEV;
	}
	TO_INT(idata) = attn->irq_enabled ? 1 : 0;
	return 0;
}

static int syna_ts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	TO_INT(idata) = tcm->pwr_state ? 1 : 0;
	return 0;
}

static int syna_ts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	int app_status;

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	app_status = syna_pal_le2_to_uint(tcm->tcm_dev->app_info.status);
	if (IS_NOT_FW_MODE(tcm->tcm_dev->id_info.mode) ||
			app_status != APP_STATUS_OK)
		TO_INT(idata) = 1;
	else
		TO_INT(idata) = 0;

	return 0;
}

static int syna_ts_mmi_methods_drv_irq(struct device *dev, int state)
{
	int retval;
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	mutex_lock(&tcm->tp_event_mutex);
	if (state == 0) {
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
			goto exit;
	} else if (state == 1) {
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Failed to enable interrupt\n");
			goto exit;
		}
	} else {
		retval = -EINVAL;
		goto exit;
	}

exit:
	mutex_unlock(&tcm->tp_event_mutex);
	return retval;
}

static int syna_ts_mmi_methods_reset(struct device *dev, int type)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	if (tcm->hw_if->ops_hw_reset)
	{
		tcm->hw_if->ops_hw_reset(tcm->hw_if);
		return 0;
	}
	return -EFAULT;
}

static int syna_ts_firmware_update(struct device *dev, char *fwname) {
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	int retval;

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	LOGI("HW request update fw");

	retval = syna_set_fw_name(tcm, fwname);
	if (retval < 0)
	{
		LOGE("Failed to set fw_name");
		return -EFAULT;
	}
	retval = syna_reflash_do_reflash(tcm, fwname);
	if (retval < 0)
	{
		LOGE("Failed to do reflash");
		return -EFAULT;
	}
	return 0;
}

static int syna_ts_mmi_methods_power(struct device *dev, int on)
{
	int retval;
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	if (!on) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (tcm->hw_if->bdata_pwr.vdd_reg_dev) {
		retval = regulator_enable(tcm->hw_if->bdata_pwr.vdd_reg_dev);
		if (retval < 0) {
			LOGE("Fail to enable vdd regulator\n");
			goto exit;
		}
	}

	if (tcm->hw_if->bdata_pwr.avdd_reg_dev) {
		retval = regulator_enable(tcm->hw_if->bdata_pwr.avdd_reg_dev);
		if (retval < 0) {
			LOGE("Fail to enable avdd regulator\n");
			goto disable_bus_reg;
		}
		syna_pal_sleep_ms(tcm->hw_if->bdata_pwr.power_on_delay_ms);
	}
	return 0;

disable_pwr_reg:
	if (tcm->hw_if->bdata_pwr.avdd_reg_dev)
		retval = regulator_enable(tcm->hw_if->bdata_pwr.avdd_reg_dev);
disable_bus_reg:
	if (tcm->hw_if->bdata_pwr.vdd_reg_dev)
		retval = regulator_enable(tcm->hw_if->bdata_pwr.vdd_reg_dev);
exit:
	return retval;
}

static int syna_ts_mmi_pre_resume(struct device *dev)
{
	int retval;
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		return -ENODEV;
	}
	if (!tcm->pwr_state)
	{
		return 0;
	}
	retval = syna_tcm_sleep(tcm->tcm_dev, false);
	if (retval < 0) {
		LOGE("Failed to exit deep sleep\n");
		goto exit;
	}

	tcm->dev_resume(&tcm->pdev->dev);
	tcm->pwr_state = true;
	return 0;

exit:
	tcm->pwr_state = false;
	return retval;
}

static int syna_ts_mmi_post_suspend(struct device *dev)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	LOGI("post_suspend enter\n");

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	if (!tcm->pwr_state)
	{
		return 0;
	}

	LOGI("post_suspend exit\n");

	return 0;
}

static int syna_ts_mmi_pre_suspend(struct device *dev)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);

	LOGI("pre_suspend enter\n");

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	if (!tcm->pwr_state)
		return 0;

	LOGI("pre_suspend exit\n");

	return 0;
}

static int syna_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	int retval = 0;
	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}
	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	switch (to) {
		case TS_MMI_PM_DEEPSLEEP:
			tcm->lpwg_enabled = false;
			retval = syna_dev_early_suspend(&tcm->pdev->dev);
			if (retval < 0) {
				LOGE("early suspend fail");
			}
			tcm->dev_suspend(&tcm->pdev->dev);
			break;
		case TS_MMI_PM_GESTURE:
			tcm->lpwg_enabled = true;
			tcm->dev_suspend(&tcm->pdev->dev);
			break;
		case TS_MMI_PM_ACTIVE:
			break;
		default:
			LOGI("panel mode %d is invalid.\n",to);
			return -EINVAL;
	}
	return 0;
};
static int syna_ts_mmi_charger_mode(struct device *dev, int mode)
{
	struct syna_tcm *tcm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	unsigned short cval = 0;
	int retval = 0;
	struct syna_hw_attn_data *attn;

	if (!pdev) {
		LOGE("Failed to get platform device");
		return -ENODEV;
	}

	tcm = platform_get_drvdata(pdev);
	if (!tcm) {
		LOGE("Failed to get driver data");
		return -ENODEV;
	}
	attn = &tcm->hw_if->bdata_attn;
	if(attn->irq_enabled == false) {
		LOGI("Interrupt is closed, so cannot access CHARGER_CONNECTED\n");
		return -EINVAL;
	}

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev,DC_ENABLE_CHARGER_CONNECTED,&cval,RESP_IN_ATTN);
	if(retval < 0) {
		LOGE("Failed to get charger_connected mode\n");
		goto exit;
	}
	if(cval != mode){
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,DC_ENABLE_CHARGER_CONNECTED,mode,RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("Failed to set charger_connected mode\n");
			goto exit;
		}
		LOGI("%s: charger mode success %d\n",__func__,cval);
	} else {
		LOGI("%s: charger mode already %d\n",__func__,cval);
	}
exit:

	return 0;
};


static struct ts_mmi_methods syna_ts_mmi_methods = {
	.get_vendor = syna_ts_mmi_methods_get_vendor,
	.get_productinfo = syna_ts_mmi_methods_get_productinfo,
	.get_build_id = syna_ts_mmi_methods_get_build_id,
	.get_config_id = syna_ts_mmi_methods_get_config_id,
	.get_bus_type = syna_ts_mmi_methods_get_bus_type,
	.get_irq_status = syna_ts_mmi_methods_get_irq_status,
	.get_drv_irq = syna_ts_mmi_methods_get_drv_irq,
	.get_flashprog = syna_ts_mmi_methods_get_flashprog,
	.get_poweron = syna_ts_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  syna_ts_mmi_methods_reset,
	.drv_irq = syna_ts_mmi_methods_drv_irq,
	.power = syna_ts_mmi_methods_power,
	.charger_mode = syna_ts_mmi_charger_mode,
	/* Firmware */
	.firmware_update = syna_ts_firmware_update,
	/* PM callback */
	.panel_state = syna_ts_mmi_panel_state,
	.pre_resume = syna_ts_mmi_pre_resume,
	.pre_suspend = syna_ts_mmi_pre_suspend,
	.post_suspend = syna_ts_mmi_post_suspend,
};

int syna_ts_mmi_dev_register(struct syna_tcm *tcm) {
	int ret;
	ret = ts_mmi_dev_register(tcm->pdev->dev.parent, &syna_ts_mmi_methods);
	if (ret) {
		LOGE("Failed to register ts mmi ret = %d\n", ret);
		return ret;
	}
	return 0;
}

void syna_ts_mmi_dev_unregister(struct syna_tcm *tcm) {
	ts_mmi_dev_unregister(tcm->pdev->dev.parent);
}
