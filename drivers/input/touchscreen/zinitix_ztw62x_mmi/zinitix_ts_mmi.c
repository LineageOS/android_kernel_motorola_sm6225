/*
 * Zinitix touchscreen driver MMI class extention
 *
 * Copyright (C) 2023 Motorola Mobility
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "zinitix_ts_mmi.h"

#define ASSERT_PTR(p) if (p == NULL) { \
		dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
	}

static int zinitix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "zinitix");
}

static int zinitix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	char* ic_info;
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct bt541_ts_platform_data *pdata = info->pdata;
	ASSERT_PTR(pdata);

	ic_info = strstr(pdata->ic_name, ",");
	ic_info++;

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", ic_info);
}

static int zinitix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", 0);
}

/*return firmware version*/
static int zinitix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct capa_info *cap = &(info->cap_info);
	ASSERT_PTR(cap);

	zinitix_printk("firmware_version = 0x%x, minor_fw_version = 0x%x, data_version_reg = 0x%x\n",
		cap->fw_version, cap->fw_minor_version, cap->reg_data_version);

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02d%02d%02d",
		cap->fw_version,
		cap->fw_minor_version,
		cap->reg_data_version);
}

static int zinitix_ts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int zinitix_ts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct bt541_ts_platform_data *pdata = info->pdata;
	ASSERT_PTR(pdata);

	TO_INT(idata) = gpio_get_value(pdata->gpio_int);
	return 0;
}

static int zinitix_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	TO_INT(idata) = info->irq_enabled;
	return 0;
}

static int zinitix_ts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	int avdd_gpio = info->pdata->avdd_gpio;
	int iovdd_gpio = info->pdata->iovdd_gpio;

	if (avdd_gpio > 0) {
		if (gpio_get_value(avdd_gpio) == 0) {
			TO_INT(idata) = 0;
			return 0;
		}
	}
	if (iovdd_gpio > 0) {
		if (gpio_get_value(iovdd_gpio) == 0) {
			TO_INT(idata) = 0;
			return 0;
		}
	}

	if (info->vdd != NULL) {
		if(!regulator_is_enabled(info->vdd)) {
			TO_INT(idata) = 0;
			return 0;
		}
	}
	if (info->vcc_i2c != NULL) {
		if(!regulator_is_enabled(info->vcc_i2c)) {
			TO_INT(idata) = 0;
			return 0;
		}
	}

	TO_INT(idata) = 1;
	return 0;
}

static int zinitix_ts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	TO_INT(idata) = info->work_state == UPGRADE ? 1 : 0;
	return 0;
}

static int zinitix_ts_mmi_methods_reset(struct device *dev, int type)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	zinitix_hw_reset(info,true);
	return 0;
}

static int zinitix_ts_firmware_update(struct device *dev, char *fwname)
{
	int ret = 0;
	u8 *fw_file_buf = NULL;
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	dev_info(dev, "%s, HW request update fw, %s", __func__, fwname);

	if (!fwname) {
		dev_err(dev, "%s: Error, firmware name is null\n",
				__func__);
		return -EINVAL;
	}

	ret= zinitix_read_file(fwname, &fw_file_buf);
	if (ret != info->cap_info.ic_fw_size) {
		dev_err(dev, "%s: invalid fw size!!\n", __func__);
		goto error_upgrade;
	}

	dev_info(dev, "%s: fw is loaded!!\n", __func__);
	info->checkUMSmode = true;
	ret = ts_upgrade_sequence((u8 *)fw_file_buf);
	info->checkUMSmode = false;
	if(ret<0) {
		dev_err(dev, "%s: fw upgrade fail!!\n", __func__);
		goto error_upgrade;
	}
	dev_info(dev, "%s: fw upgrade success!!\n", __func__);

error_upgrade:
	if (fw_file_buf) {
		vfree(fw_file_buf);
		fw_file_buf = NULL;
	}

	return ret;
}

static int zinitix_ts_mmi_methods_power(struct device *dev, int on)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	if (on == TS_MMI_POWER_ON)
	{
		dev_info(dev, "%s: Power on touch IC \n", __func__);
		bt541_power_control(info, POWER_ON_SEQUENCE);
		mini_init_touch(info);
	}
	else if(on == TS_MMI_POWER_OFF)
	{
		dev_info(dev, "%s: Power off touch IC \n", __func__);
		bt541_power_control(info, POWER_OFF);
	}

	return 0;
}

static int zinitix_ts_mmi_methods_drv_irq(struct device *dev, int state)
{
	int ret = 0;
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	if (state == 1) {
		dev_info(dev, "%s: enable irq \n", __func__);
		if (info->irq_enabled == false) {
			enable_irq(info->irq);
			info->irq_enabled = true;
		}
	} else if (state == 0) {
		dev_info(dev, "%s: disable irq \n", __func__);
		if (info->irq_enabled == true) {
			disable_irq(info->irq);
			info->irq_enabled = false;
		}
	} else {
		dev_err(dev, "%s: invalid value\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static int zinitix_ts_mmi_panel_state(struct device *dev,
    enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	switch (to) {
	case TS_MMI_PM_GESTURE:
		if (zinitix_ts_mmi_gesture_suspend(dev) < 0) {
			dev_err(dev, "%s: Gesture suspend error.\n", __func__);
			return -EINVAL;
		}
		dev_info(dev, "%s: Gesture suspend success.\n", __func__);
		info->gesture_enabled = true;
	break;

	case TS_MMI_PM_DEEPSLEEP:
		dev_info(dev, "%s: Enter TS_MMI_PM_DEEPSLEEP mode\n", __func__);
		info->gesture_enabled = false;
	break;

	case TS_MMI_PM_ACTIVE:
		if (info->gesture_enabled == true) {
			if (zinitix_ts_mmi_gesture_resume(dev) < 0) {
				dev_err(dev, "%s: Gesture resume error.\n", __func__);
				return -EINVAL;
			}
			dev_info(dev, "%s: Gesture resume success.\n", __func__);
			info->gesture_enabled = false;
		}
	break;

	default:
		dev_err(dev, "%s: Invalid power state parameter %d.\n", __func__, to);
		return -EINVAL;
	}

	return 0;
}

static int zinitix_ts_mmi_pre_suspend(struct device *dev)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	dev_info(dev, "%s: Suspend start \n", __func__);

	down(&info->work_lock);
	//stop ESD
	#if ESD_TIMER_INTERVAL
		flush_work(&info->tmr_work);
		esd_timer_stop(info);
	#endif
	up(&info->work_lock);

	return 0;
}

static int zinitix_ts_mmi_post_suspend(struct device *dev)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	down(&info->work_lock);
	//clear report data
	clear_report_data(info);
	//set work state
	info->work_state = SUSPEND;
	up(&info->work_lock);

	dev_info(dev, "%s: Suspend end \n", __func__);

	return 0;
}

static int zinitix_ts_mmi_pre_resume(struct device *dev)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	dev_info(dev, "%s: Resume start\n", __func__);
	down(&info->work_lock);
	info->work_state = RESUME;
	up(&info->work_lock);

	return 0;
}

static int zinitix_ts_mmi_post_resume(struct device *dev)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	down(&info->work_lock);
	#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);
	#endif
	info->work_state = NOTHING;
	up(&info->work_lock);
	dev_info(dev, "%s: Resume end\n", __func__);
	return 0;
}

static struct ts_mmi_methods zinitix_ts_mmi_methods = {
	.get_vendor = zinitix_ts_mmi_methods_get_vendor,
	.get_productinfo = zinitix_ts_mmi_methods_get_productinfo,
	.get_build_id = zinitix_ts_mmi_methods_get_build_id,
	.get_config_id = zinitix_ts_mmi_methods_get_config_id,
	.get_bus_type = zinitix_ts_mmi_methods_get_bus_type,
	.get_irq_status = zinitix_ts_mmi_methods_get_irq_status,
	.get_drv_irq = zinitix_ts_mmi_methods_get_drv_irq,
	.get_poweron = zinitix_ts_mmi_methods_get_poweron,
	.get_flashprog = zinitix_ts_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  zinitix_ts_mmi_methods_reset,
	.power = zinitix_ts_mmi_methods_power,
	.drv_irq = zinitix_ts_mmi_methods_drv_irq,
	/* Firmware */
	.firmware_update = zinitix_ts_firmware_update,
	/* PM callback */
	.panel_state = zinitix_ts_mmi_panel_state,
	.pre_suspend = zinitix_ts_mmi_pre_suspend,
	.post_suspend = zinitix_ts_mmi_post_suspend,
	.pre_resume = zinitix_ts_mmi_pre_resume,
	.post_resume = zinitix_ts_mmi_post_resume,
};

int zinitix_ts_mmi_dev_register(struct device *dev) {
	int ret;

	ret = ts_mmi_dev_register(dev, &zinitix_ts_mmi_methods);
	if (ret) {
		dev_err(dev, "%s, Failed to register ts mmi\n", __func__);
		return ret;
	}
	dev_info(dev, "%s, Register ts mmi success\n", __func__);

	return 0;
}

void zinitix_ts_mmi_dev_unregister(struct device *dev) {
	struct bt541_ts_info *info = dev_get_drvdata(dev);

	if (!info)
		dev_err(dev ,"Failed to get driver data");
	ts_mmi_dev_unregister(dev);
}

