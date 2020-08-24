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
#include "synaptics_core.h"

#define GET_SYNA_DATA(dev) { \
	pdev = dev_get_drvdata(dev); \
	if (!pdev) { \
		LOGE(dev, "Failed to get platform device");\
		return -ENODEV; \
	} \
	tcm_hcd = platform_get_drvdata(pdev); \
	if (!tcm_hcd) { \
		LOGE(dev, "Failed to get driver data"); \
		return -ENODEV; \
	} \
}

struct pill_region_data {
       unsigned short y_start_l;
       unsigned short y_end_l;
       unsigned short y_start_r;
       unsigned short y_end_r;
};

static int syna_ts_mmi_methods_hold_distance(struct device *dev, int value)
{
	int retval;
	unsigned short buffer;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	buffer = (unsigned short)value;
	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_HOLD_DISTANCE,
		buffer);

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to write hold distance (%d)\n", retval);
		goto exit;
	}

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_get_hold_distance(struct device *dev, void *idata)
{
	int retval;
	unsigned short value;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, DC_HOLD_DISTANCE, &value);
	if (retval < 0)
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read hold distance info (%d)\n", retval);
	else
		TO_INT(idata) = value;

	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_suppression(struct device *dev, int value)
{
	int retval;
	unsigned short buffer;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	buffer = (unsigned short)value;
	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_GRIP_SUPPRESSION_INFO,
		buffer);

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to write suppression info (%d)\n", retval);
		goto exit;
	}

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_get_suppression(struct device *dev, void *idata)
{
	int retval;
	unsigned short value;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, DC_GRIP_SUPPRESSION_INFO, &value);
	if (retval < 0)
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read suppression info (%d)\n", retval);
	else
		TO_INT(idata) = value;

	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_gs_distance(struct device *dev, int value)
{
	int retval;
	unsigned short buffer;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	buffer = (unsigned short)value;
	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_X_WIDTH,
		buffer);

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to enable wakeup gesture mode\n");
		goto exit;
	}

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_get_gs_distance(struct device *dev, void *idata)
{
	int retval;
	unsigned short value;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, DC_SUPP_X_WIDTH, &value);
	if (retval < 0)
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read gs_distance (%d)\n", retval);
	else
		TO_INT(idata) = value;

	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_get_pill_region(struct syna_tcm_hcd *tcm_hcd, struct pill_region_data *region_data)
{
	int retval;

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_START_L, &region_data->y_start_l);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read LEFT_Y_START (%d)\n", retval);
		return retval;
	}

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_END_L, &region_data->y_end_l);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read LEFT_Y_END (%d)\n", retval);
		return retval;
	}

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_START_R, &region_data->y_start_r);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read RIGHT_Y_START (%d)\n", retval);
		return retval;
	}

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_END_R, &region_data->y_end_r);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read RIGHT_Y_END (%d)\n", retval);
		return retval;
	}

	return retval;
}

static int syna_ts_set_pill_region(struct syna_tcm_hcd *tcm_hcd, struct pill_region_data region_data)
{
	int retval;

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_START_L, region_data.y_start_l);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to set LEFT_Y_START of pill_region\n");
		return retval;
	}

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_END_L, region_data.y_end_l);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to set LEFT_Y_END of pill_region\n");
		return retval;
	}

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_START_R, region_data.y_start_r);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to set RIGHT_Y_START of pill_region\n");
		return retval;
	}

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_END_R, region_data.y_end_r);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to set RIGHT_Y_END of pill_region\n");
		return retval;
	}

	return retval;
}

static int syna_ts_mmi_methods_pill_region(struct device *dev, int *value)
{
	int retval;
	struct pill_region_data region_data;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	dev_dbg(dev, "%s: program pill region:0x%02x 0x%04x 0x%04x\n",
		__func__, value[0], value[1], value[2]);

	switch (value[0]) {
		case 0:
			region_data.y_start_l = 0x00;
			region_data.y_end_l = 0x00;
			region_data.y_start_r = 0x00;
			region_data.y_end_r = 0x00;
			break;

		case 1:
			region_data.y_start_l = (unsigned short)value[1];
			region_data.y_end_l = (unsigned short)value[2];
			region_data.y_start_r = 0x00;
			region_data.y_end_r = 0x00;
			break;

		case 2:
			region_data.y_start_l = 0x00;
			region_data.y_end_l = 0x00;
			region_data.y_start_r = (unsigned short)value[1];
			region_data.y_end_r = (unsigned short)value[2];
			break;

		default:
			LOGE(dev, "The commond is not support!\n");
			return -EINVAL;
	}
	mutex_lock(&tcm_hcd->extif_mutex);

	retval = syna_ts_set_pill_region(tcm_hcd, region_data);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to set pill_region none\n");
		goto exit;
	}

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_get_pill_region(struct device *dev, void *idata)
{
	int retval;
	struct pill_region_data region_data;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = syna_ts_get_pill_region(tcm_hcd, &region_data);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to get pill_region value (%d)\n", retval);
		goto exit;
	}

	if (region_data.y_start_l == 0 && region_data.y_end_l == 0) {
		if (region_data.y_start_r == 0 && region_data.y_end_r == 0)
			((int *)idata)[0] = 0x00;
		else
			((int *)idata)[0] = 0x02;

		((int *)idata)[1] = region_data.y_start_r;
		((int *)idata)[2] = region_data.y_end_r;

	} else {
			((int *)idata)[0] = 0x01;
			((int *)idata)[1] = region_data.y_start_l;
			((int *)idata)[2] = region_data.y_end_l;
	}
	dev_dbg(dev, "%s: program pill region show %02x, %x, %x",__func__,((int *)idata)[0], ((int *)idata)[1], ((int *)idata)[2]);

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "synaptics");
}

static int syna_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	return scnprintf(TO_CHARP(cdata), 6, "%s", tcm_hcd->id_info.part_number);
}

static int syna_ts_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	unsigned int buildid;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	buildid = tcm_hcd->packrat_number;
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%06x", buildid);
}

/*return firmware version*/
static int syna_ts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%s", tcm_hcd->app_info.customer_config_id);
}

static int syna_ts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	const struct syna_tcm_hw_interface *hw_if;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	hw_if = tcm_hcd->hw_if;
	if (!hw_if) {
		LOGE(dev, "Hardware interface not found\n");
		return -ENODEV;
	}
	TO_INT(idata) = hw_if->bus_io->type;
	return 0;
}

static int syna_ts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	const struct syna_tcm_hw_interface *hw_if;
	struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	hw_if = tcm_hcd->hw_if;
	if (!hw_if) {
		LOGE(dev, "Hardware interface not found\n");
		return -ENODEV;
	}
	bdata = hw_if->bdata;
	if (!bdata) {
		LOGE(dev, "Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(bdata->irq_gpio);
	return 0;
}

static int syna_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	bdata = tcm_hcd->hw_if->bdata;
	if (!bdata) {
		LOGE(dev, "Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = tcm_hcd->irq_enabled ? 1 : 0;
	return 0;
}

static int syna_ts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	TO_INT(idata) = tcm_hcd->power_status == 0 ? 1 : 0;
	return 0;
}

static int syna_ts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK)
		TO_INT(idata) = 1;
	else
		TO_INT(idata) = 0;

	return 0;
}

static int syna_ts_mmi_methods_drv_irq(struct device *dev, int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	mutex_lock(&tcm_hcd->extif_mutex);
	if (state == 0) {
		retval = tcm_hcd->enable_irq(tcm_hcd, false, true);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to disable interrupt\n");
			goto exit;
		}
	} else if (state == 1) {
		retval = tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable interrupt\n");
			goto exit;
		}
	} else {
		retval = -EINVAL;
		goto exit;
	}

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_reset(struct device *dev, int type)
{
	bool hw_reset = !!type;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	if (tcm_hcd->reset)
		return tcm_hcd->reset_n_reinit(tcm_hcd, hw_reset, true);
	return -EFAULT;
}

static int syna_ts_firmware_update(struct device *dev, char *fwname) {
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	LOGI(dev, "HW request update fw");
	/* set firmware image name */
	if (tcm_hcd->set_fw_name)
		tcm_hcd->set_fw_name(fwname);
	else {
		LOGE(dev, "fw update handler not ready, can't set fw_name");
		return -EFAULT;
	}
	if (tcm_hcd->do_fw_update) {
		tcm_hcd->do_fw_update();
	} else {
		LOGE(dev, "fw update handler not ready.");
		return -EFAULT;
	}
	return 0;
}

static int syna_ts_mmi_methods_power(struct device *dev, int on)
{
	int retval;
	const struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	bdata = tcm_hcd->hw_if->bdata;
	if (!bdata) {
		LOGE(dev, "Failed to get ts board data");
		return -ENODEV;
	}
	if (!on) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (tcm_hcd->bus_reg) {
		retval = regulator_enable(tcm_hcd->bus_reg);
		if (retval < 0) {
			LOGE(dev, "Failed to enable bus regulator\n");
			goto exit;
		}
	}

	if (tcm_hcd->pwr_reg) {
		retval = regulator_enable(tcm_hcd->pwr_reg);
		if (retval < 0) {
			LOGE(dev, "Failed to enable power regulator\n");
			goto disable_bus_reg;
		}
		msleep(bdata->power_delay_ms);
	}
	tcm_hcd->power_status = true;
	return 0;

disable_pwr_reg:
	if (tcm_hcd->pwr_reg)
		regulator_disable(tcm_hcd->pwr_reg);
disable_bus_reg:
	if (tcm_hcd->bus_reg)
		regulator_disable(tcm_hcd->bus_reg);
exit:
	tcm_hcd->power_status = false;
	return retval;
}

static int syna_ts_mmi_wait_for_ready(struct device *dev)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	msleep(RESET_ON_RESUME_DELAY_MS);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, true);
#endif

	return 0;
}

static int syna_ts_mmi_post_resume(struct device *dev)
{
	struct platform_device *pdev;
	pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE(dev, "Failed to get platform device");
		return -ENODEV;
	}

	return syna_tcm_resume(&pdev->dev);
}

static int syna_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	switch (to) {
	case TS_MMI_PM_DEEPSLEEP:
		tcm_hcd->wakeup_gesture_enabled = false;
		syna_tcm_early_suspend(&pdev->dev);
		syna_tcm_suspend(&pdev->dev);
		break;

	case TS_MMI_PM_GESTURE:
		tcm_hcd->wakeup_gesture_enabled = true;
		syna_tcm_early_suspend(&pdev->dev);
		syna_tcm_suspend(&pdev->dev);
		break;

	case TS_MMI_PM_ACTIVE:
		break;

	default:
		dev_warn(dev, "panel mode %d is invalid.\n", to);
		return -EINVAL;
	}
	return 0;
}

static int syna_ts_mmi_charger_mode(struct device *dev, int mode)
{
	int retval;
	unsigned short cval = 0;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	if (tcm_hcd->irq_enabled == false) {
		dev_info(dev, "%s, Interrupt is closed, so cannot access CHARGER_CONNECTED\n",
				__func__);
		return -EINVAL;
	}

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->get_dynamic_config(tcm_hcd, DC_CHARGER_CONNECTED, &cval);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get charger_connected mode\n");
		goto exit;
	}

	dev_dbg(dev, "%s: Set charger mode:%d, charger current mode:%d\n",
				__func__, mode, cval);

	if (cval != mode) {
		retval = tcm_hcd->set_dynamic_config(tcm_hcd, DC_CHARGER_CONNECTED, mode);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set charger_connected mode\n");
			goto exit;
		}
	} else
		dev_info(dev, "%s: charger mode already %d\n", __func__, cval);

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return 0;
}

static int syna_ts_mmi_update_baseline(struct device *dev, int mode)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;
	int retval;

	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);

	if (mode && tcm_hcd->delay_baseline_update) {
		if (atomic_read(&tcm_hcd->helper.task) == HELP_NONE) {
			dev_info(dev, "%s: Start to update baseline\n", __func__);
			atomic_set(&tcm_hcd->helper.task,
					HELP_SEND_REZERO_COMMAND);
			queue_work(tcm_hcd->helper.workqueue,
					&tcm_hcd->helper.work);
		} else {
			retval = -EINVAL;
			goto exit;
		}
	}
	retval = 0;
	tcm_hcd->delay_baseline_update = !mode;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

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
	.get_suppression = syna_ts_mmi_methods_get_suppression,
	.get_gs_distance = syna_ts_mmi_methods_get_gs_distance,
	.get_pill_region = syna_ts_mmi_methods_get_pill_region,
	.get_hold_distance = syna_ts_mmi_methods_get_hold_distance,
	/* SET methods */
	.reset =  syna_ts_mmi_methods_reset,
	.drv_irq = syna_ts_mmi_methods_drv_irq,
	.power = syna_ts_mmi_methods_power,
	.charger_mode = syna_ts_mmi_charger_mode,
	.suppression = syna_ts_mmi_methods_suppression,
	.gs_distance = syna_ts_mmi_methods_gs_distance,
	.pill_region = syna_ts_mmi_methods_pill_region,
	.update_baseline = syna_ts_mmi_update_baseline,
	.hold_distance = syna_ts_mmi_methods_hold_distance,
	/* Firmware */
	.firmware_update = syna_ts_firmware_update,
	/* PM callback */
	.wait_for_ready = syna_ts_mmi_wait_for_ready,
	.panel_state = syna_ts_mmi_panel_state,
	.post_resume = syna_ts_mmi_post_resume,
};

int syna_ts_mmi_dev_register(struct syna_tcm_hcd *tcm_hcd) {
	int ret;

	ret = ts_mmi_dev_register(tcm_hcd->pdev->dev.parent, &syna_ts_mmi_methods);
	if (ret) {
		dev_err(tcm_hcd->pdev->dev.parent, "Failed to register ts mmi\n");
		return ret;
	}

	/* initialize class imported methods */
	tcm_hcd->imports = &syna_ts_mmi_methods.exports;

	return 0;
}

void syna_ts_mmi_dev_unregister(struct syna_tcm_hcd *tcm_hcd) {
	ts_mmi_dev_unregister(tcm_hcd->pdev->dev.parent);
}
