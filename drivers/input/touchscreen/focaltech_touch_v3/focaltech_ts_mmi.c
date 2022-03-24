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
#include "focaltech_core.h"

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
//extern int ts_mmi_dev_register(struct device *parent, struct ts_mmi_methods *mdata);
//extern void ts_mmi_dev_unregister(struct device *parent);
#endif

#define GET_TS_DATA(dev) { \
	ts_data = dev_get_drvdata(dev); \
	if (!ts_data) { \
		FTS_ERROR("Failed to get driver data"); \
		return -ENODEV; \
	} \
}
static int fts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "focaltech");
}

static int fts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", FTS_CHIP_NAME);
}

static int fts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;
	ssize_t num_read_chars = 0;
	u8 fwver = 0;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;
	mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif
	fts_read_reg(FTS_REG_FW_VER, &fwver);
#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif
	num_read_chars = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fwver);

	mutex_unlock(&input_dev->mutex);
	return num_read_chars;
}

/*return firmware version*/
static int fts_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", 0);
}

static int fts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;
	TO_INT(idata) = input_dev->id.bustype == BUS_SPI ?
			TOUCHSCREEN_MMI_BUS_TYPE_SPI : TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int fts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);
	pdata = ts_data->pdata;

	TO_INT(idata) = gpio_get_value(pdata->irq_gpio);
	return 0;
}

static int fts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);
	TO_INT(idata) = ts_data->irq_disabled ? 0 : 1;
	return 0;
}

static int fts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	TO_INT(idata) = ts_data->power_disabled == false ? 1 : 0;
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int fts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);
	TO_INT(idata) = (ts_data->fw_loading) ? 1 : 0;

	return 0;
}

static int fts_mmi_methods_drv_irq(struct device *dev, int state)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;
	int ret = 0;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (state == 1) {
		FTS_INFO("enable irq");
		fts_irq_enable();
	} else if (state == 0) {
		FTS_INFO("disable irq");
		fts_irq_disable();
	} else {
		dev_err(dev, "%s: invalid value\n", __func__);
		ret = -EINVAL;
	}

	mutex_unlock(&input_dev->mutex);
	return ret;
}

#if FTS_POWER_SOURCE_CUST_EN
static int fts_mmi_methods_power(struct device *dev, int state)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);

	return fts_power_source_ctrl(ts_data, state);
}
#endif

static int fts_mmi_methods_reset(struct device *dev, int type)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	fts_reset_proc(0);
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int fts_mmi_methods_pinctrl(struct device *dev, int on)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	if (on == TS_MMI_PINCTL_ON) {
		mutex_lock(&input_dev->mutex);
		fts_reset_proc(150);
		mutex_unlock(&input_dev->mutex);
	}

	return 0;
}

static int fts_mmi_firmware_update(struct device *dev, char *fwname)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	ts_data->force_reflash = true;

	mutex_lock(&input_dev->mutex);
	fts_fw_update_vendor_name(fwname);
	fts_fwupg_bin();
	mutex_unlock(&input_dev->mutex);

	return 0;
}

#if FTS_USB_DETECT_EN
static int fts_mmi_charger_mode(struct device *dev, int mode)
{
	struct fts_ts_data *ts_data;
	int ret = 0;

	GET_TS_DATA(dev);
	ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, mode);
	if(ret < 0){
		FTS_ERROR("Failed to set charger mode\n");
	}

	FTS_INFO("Success to %s charger mode\n", mode ? "Enable" : "Disable");

	return 0;
}
#endif

static int fts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *ts_data;
	int ret = 0;

	GET_TS_DATA(dev);
	pdata = ts_data->pdata;

	switch (to) {
		case TS_MMI_PM_DEEPSLEEP:
			ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
			ts_data->gesture_support = false;
			if (ret < 0)
				FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);
			break;

		case TS_MMI_PM_GESTURE:
#if FTS_GESTURE_EN
			if (fts_gesture_suspend(ts_data) == 0) {
			/* Enter into gesture mode(suspend) */
#ifdef FOCALTECH_SENSOR_EN
				ts_data->wakeable = true;
				ts_data->gesture_support = true;
#endif
			}
#endif
			break;

		case TS_MMI_PM_ACTIVE:
			break;
		default:
			dev_warn(dev, "panel mode %d is invalid.\n", to);
			ret = -EINVAL;
			break;
	}

	return ret;

}

static int fts_mmi_pre_resume(struct device *dev)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);

	FTS_FUNC_ENTER();

	fts_release_all_finger();

	FTS_FUNC_EXIT();
	return 0;
}

static int fts_mmi_post_resume(struct device *dev)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);

	if (ts_data->gesture_support == true) {
		FTS_INFO("Reset IC in resume");
		fts_reset_proc(200);
	}

	fts_tp_state_recovery(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif

#if FTS_GESTURE_EN
#ifdef FOCALTECH_SENSOR_EN
	if (ts_data->wakeable) {
#endif
		if (fts_gesture_resume(ts_data) == 0) {
#ifdef FOCALTECH_SENSOR_EN
			ts_data->wakeable = false;
			FTS_INFO("Exit from gesture suspend mode.");
#endif
			goto exit;
		}
#ifdef FOCALTECH_SENSOR_EN
	}
#endif
#endif

#if FTS_GESTURE_EN
exit:
#endif
	FTS_FUNC_EXIT();
	ts_data->suspended = false;

	return 0;
}

static int fts_mmi_pre_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);

	FTS_FUNC_ENTER();

#if FTS_ESDCHECK_EN
	FTS_INFO("fts_esdcheck_suspend");
	fts_esdcheck_suspend();
#endif

	FTS_FUNC_EXIT();

    return 0;
}

static int fts_mmi_post_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);

	FTS_FUNC_ENTER();

#ifdef FOCALTECH_SENSOR_EN
	if (!ts_data->wakeable) {
#endif
		fts_release_all_finger();
#ifdef FOCALTECH_SENSOR_EN
	}
#endif

	ts_data->suspended = true;
	FTS_FUNC_EXIT();

	return 0;
}

#ifdef FOCALTECH_PALM_SENSOR_EN
int fts_mmi_palm_set_enable(struct device *dev, unsigned int enable)
{
	fts_palm_sensor_set_enable(enable);
	return 0;
}
#endif

static struct ts_mmi_methods fts_mmi_methods = {
	.get_vendor = fts_mmi_methods_get_vendor,
	.get_productinfo = fts_mmi_methods_get_productinfo,
	.get_build_id = fts_mmi_methods_get_build_id,
	.get_config_id = fts_mmi_methods_get_config_id,
	.get_bus_type = fts_mmi_methods_get_bus_type,
	.get_irq_status = fts_mmi_methods_get_irq_status,
	.get_drv_irq = fts_mmi_methods_get_drv_irq,
	.get_flashprog = fts_mmi_methods_get_flashprog,
	.get_poweron = fts_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  fts_mmi_methods_reset,
	.pinctrl =  fts_mmi_methods_pinctrl,
	.drv_irq = fts_mmi_methods_drv_irq,
#if FTS_POWER_SOURCE_CUST_EN
	.power = fts_mmi_methods_power,
#endif
#if FTS_USB_DETECT_EN
	.charger_mode = fts_mmi_charger_mode,
#endif
#ifdef FOCALTECH_PALM_SENSOR_EN
	.palm_set_enable = fts_mmi_palm_set_enable,
#endif
	/* Firmware */
	.firmware_update = fts_mmi_firmware_update,
	/* PM callback */
	.panel_state = fts_mmi_panel_state,
	.pre_resume = fts_mmi_pre_resume,
	.post_resume = fts_mmi_post_resume,
	.pre_suspend = fts_mmi_pre_suspend,
	.post_suspend = fts_mmi_post_suspend,

};

int fts_mmi_dev_register(struct fts_ts_data *ts_data) {
	int ret;

	ret = ts_mmi_dev_register(ts_data->dev, &fts_mmi_methods);
	if (ret) {
		dev_err(ts_data->dev, "Failed to register ts mmi\n");
		return ret;
	}

	/* initialize class imported methods */
	ts_data->imports = &fts_mmi_methods.exports;

	return 0;
}

void fts_mmi_dev_unregister(struct fts_ts_data *ts_data) {
	ts_mmi_dev_unregister(ts_data->dev);
}

