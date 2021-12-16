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

#include <linux/touchscreen_mmi.h>
#include "goodix_ts_core.h"
#include <linux/delay.h>

#define GET_GOODIX_DATA(dev) { \
	pdev = dev_get_drvdata(dev); \
	if (!pdev) { \
		ts_err("Failed to get platform device"); \
		return -ENODEV; \
	} \
	core_data = platform_get_drvdata(pdev); \
	if (!core_data) { \
		ts_err("Failed to get driver data"); \
		return -ENODEV; \
	} \
}

extern int gsx_gesture_before_suspend_mmi(struct goodix_ts_core *core_data);

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}

static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", GOODIX_TP_IC_TYPE);
}

static int goodix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	int ret;
	struct goodix_ts_version fw_ver;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	ts_dev = core_data->ts_dev;
	if (!ts_dev) {
		ts_err("Failed to get platform data");
		return -ENODEV;
	}
	ret = ts_dev->hw_ops->read_version(ts_dev, &fw_ver);
	if (!ret && fw_ver.valid) {
		ret = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fw_ver.vid[0]);
		ts_info("get config id:%d ", fw_ver.vid[0]);
		return ret;
	} else
		return -ENODEV;
}

/*return firmware version*/
static int goodix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	int ret;
	struct goodix_ts_version fw_ver;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	ts_dev = core_data->ts_dev;
	if (!ts_dev) {
		ts_err("Failed to get platform data");
		return -ENODEV;
	}
	ret = ts_dev->hw_ops->read_version(ts_dev, &fw_ver);
	if (!ret && fw_ver.valid) {
		ret = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fw_ver.vid[3]);
		ts_info("get build id:%d ", fw_ver.vid[3]);
		return ret;
	} else
		return -ENODEV;
}

static int goodix_ts_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int goodix_ts_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct goodix_ts_board_data *ts_bdata;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(ts_bdata->irq_gpio);
	return 0;
}

static int goodix_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	TO_INT(idata) = atomic_read(&core_data->irq_enabled);
	return 0;
}

static int goodix_ts_mmi_methods_get_poweron(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	TO_INT(idata) = core_data->power_on;
	return 0;
}

static int goodix_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	goodix_ts_irq_enable(core_data, !(!state));
	return 0;
}

/* reset chip
 * type：can control software reset and hardware reset,
 * but GOODIX has no software reset,
 * so the type parameter is not used here.
 */
static int goodix_ts_mmi_methods_reset(struct device *dev, int type) {
	int ret = -ENODEV;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	ts_dev = core_data->ts_dev;
	if (ts_dev->hw_ops->reset)
		ret = ts_dev->hw_ops->reset(ts_dev);
	return ret;
}

static int goodix_ts_mmi_methods_power(struct device *dev, int on) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	if (on == TS_MMI_POWER_ON)
		return goodix_ts_power_on(core_data);
	else if(on == TS_MMI_POWER_OFF)
		return goodix_ts_power_off(core_data);
	else {
		ts_err("Invalid power parameter %d.\n", on);
		return -EINVAL;
	}
}

#define CHARGER_MODE_CMD    0xAF
static int goodix_ts_mmi_charger_mode(struct device *dev, int mode)
{
	return 0;
}

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	ts_dev = core_data->ts_dev;

	switch (to) {
	case TS_MMI_PM_GESTURE:
		gsx_gesture_before_suspend_mmi(core_data);
		msleep(16);
		enable_irq_wake(core_data->irq);
		core_data->gesture_enable = true;
		break;
	case TS_MMI_PM_DEEPSLEEP:
		/* enter sleep mode or power off */
		if (ts_dev && ts_dev->hw_ops->suspend)
			ts_dev->hw_ops->suspend(ts_dev);
		core_data->gesture_enable = false;
		break;
	case TS_MMI_PM_ACTIVE:
		if (ts_dev && ts_dev->hw_ops->resume)
			ts_dev->hw_ops->resume(ts_dev);
		core_data->gesture_enable = false;
		break;
	default:
		ts_err("Invalid power state parameter %d.\n", to);
		return -EINVAL;
	}

	return 0;
}

static int goodix_ts_mmi_pre_resume(struct device *dev)
{
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);
	ts_dev = core_data->ts_dev;

	atomic_set(&core_data->suspended, 0);
	if (core_data->gesture_enable)
		disable_irq_wake(core_data->irq);

	return 0;
}

static int goodix_ts_mmi_post_resume(struct device *dev)
{
	return 0;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);
	return 0;
}


static int goodix_ts_mmi_post_suspend(struct device *dev)
{
	ts_info("Suspend end");
	return 0;
}

static struct ts_mmi_methods goodix_ts_mmi_methods = {
	.get_vendor = goodix_ts_mmi_methods_get_vendor,
	.get_productinfo = goodix_ts_mmi_methods_get_productinfo,
	.get_build_id = goodix_ts_mmi_methods_get_build_id,
	.get_config_id = goodix_ts_mmi_methods_get_config_id,
	.get_bus_type = goodix_ts_mmi_methods_get_bus_type,
	.get_irq_status = goodix_ts_mmi_methods_get_irq_status,
	.get_drv_irq = goodix_ts_mmi_methods_get_drv_irq,
	.get_poweron = goodix_ts_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  goodix_ts_mmi_methods_reset,
	.drv_irq = goodix_ts_mmi_methods_drv_irq,
	.power = goodix_ts_mmi_methods_power,
	.charger_mode = goodix_ts_mmi_charger_mode,
	/* PM callback */
	.panel_state = goodix_ts_mmi_panel_state,
	.pre_resume = goodix_ts_mmi_pre_resume,
	.post_resume = goodix_ts_mmi_post_resume,
	.pre_suspend = goodix_ts_mmi_pre_suspend,
	.post_suspend = goodix_ts_mmi_post_suspend,
};

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ret = ts_mmi_dev_register(core_data->ts_dev->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		return ret;
	}

	core_data->imports = &goodix_ts_mmi_methods.exports;

	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_err("Failed to get driver data");
	ts_mmi_dev_unregister(&pdev->dev);
}
