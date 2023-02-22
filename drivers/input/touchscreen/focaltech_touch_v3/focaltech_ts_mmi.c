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

#define MAX_ATTRS_ENTRIES 10

#define ADD_ATTR(name) { \
	if (idx < MAX_ATTRS_ENTRIES)  { \
		dev_info(dev, "%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
		ext_attributes[idx] = &dev_attr_##name.attr; \
		idx++; \
	} else { \
		dev_err(dev, "%s: cannot add attribute '%s'\n", __func__, #name); \
	} \
}

static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

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

static int fts_mmi_firmware_update(struct device *dev, char *fwname)
{
	struct fts_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;

	ts_data->force_reflash = true;

	fts_fw_update_vendor_name(fwname);
	fts_fwupg_bin();

	return 0;
}

#if FTS_USB_DETECT_EN
static int fts_mmi_charger_mode(struct device *dev, int mode)
{
	struct fts_ts_data *ts_data;
	int ret = 0;
	uint8_t read_data = 0;

	GET_TS_DATA(dev);
	mutex_lock(&ts_data->mode_lock);

	ret = fts_read_reg(FTS_REG_CHARGER_MODE_EN, &read_data);
	if (ret < 0) {
		FTS_ERROR("read 8b register fail, ret=%d", ret);
		mutex_unlock(&ts_data->mode_lock);
		return -EINVAL;
	}

	read_data |= (!!mode);
	ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, read_data);
	if(ret < 0){
		FTS_ERROR("Failed to set charger mode\n");
		mutex_unlock(&ts_data->mode_lock);
		return -EINVAL;
	}

	FTS_INFO("Success to %s charger mode, 0x8B = 0x%02x\n", mode ? "Enable" : "Disable",
		read_data);

	mutex_unlock(&ts_data->mode_lock);
	return 0;
}
#endif

static int fts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *ts_data;
	int ret = 0;
#if defined(CONFIG_FTS_DOUBLE_TAP_CONTROL)
	u8 gesture_command = 0;
	unsigned char gesture_type = 0;
#endif

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
#if defined(CONFIG_FTS_DOUBLE_TAP_CONTROL)
			if (ts_data->imports && ts_data->imports->get_gesture_type) {
				ret = ts_data->imports->get_gesture_type(ts_data->dev, &gesture_type);
			}

			if (gesture_type & TS_MMI_GESTURE_SINGLE) {
				gesture_command |= (1 << 7);
				FTS_INFO("Enable GESTURE_CLI_SINGLE command: %02x", gesture_command);
			}
			if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
				gesture_command |= (1 << 4);
				FTS_INFO("Enable GESTURE_CLI_DOUBLE command: %02x", gesture_command);
			}

			FTS_INFO("CLI GESTURE SWITCH command: %02x", gesture_command);
			ts_data->gsx_cmd = gesture_command;
#endif

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
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;
	FTS_FUNC_ENTER();

	fts_release_all_finger();

	if (ts_data->gesture_support == false) {
		mutex_lock(&input_dev->mutex);
		fts_reset_proc(150);
		FTS_INFO("Reset IC in %s for deep sleep", __func__);
		mutex_unlock(&input_dev->mutex);
	}


	FTS_FUNC_EXIT();
	return 0;
}

static int fts_mmi_post_resume(struct device *dev)
{
	struct fts_ts_data *ts_data;
	int ret = 0;
	uint8_t read_data = 0;

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

	mutex_lock(&ts_data->mode_lock);
	/* All IC status are cleared after reset */
	memset(&ts_data->set_mode, 0 , sizeof(ts_data->set_mode));
	/* restore data */
	if (ts_data->pdata->pocket_mode_ctrl && ts_data->get_mode.pocket_mode) {
		if(fts_read_reg(FTS_REG_POCKET_MODE_EN, &read_data) >= 0) {
			read_data |= (1 << 7);
			ret = fts_write_reg(FTS_REG_POCKET_MODE_EN, read_data);
			if(ret < 0){
				FTS_ERROR("Failed to set pocket mode\n");
			} else {
				ts_data->set_mode.pocket_mode = ts_data->get_mode.pocket_mode;
				FTS_INFO("Success enable pocket mode, 0x8B = 0x%02x\n", read_data);
			}
		} else {
			FTS_ERROR("read 8b register fail, ret=%d", ret);
		}
	}

	mutex_unlock(&ts_data->mode_lock);

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

#ifdef CONFIG_FTS_LAST_TIME
static ssize_t fts_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data;
	ktime_t last_ktime;
	struct timespec64 last_ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	mutex_lock(&ts_data->mode_lock);
	last_ktime = ts_data->last_event_time;
	ts_data->last_event_time = 0;
	mutex_unlock(&ts_data->mode_lock);

	last_ts = ktime_to_timespec64(last_ktime);

	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
static DEVICE_ATTR(timestamp, S_IRUGO, fts_ts_timestamp_show, NULL);
#endif

static ssize_t fts_mmi_pocket_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data;
	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ts_data->set_mode.pocket_mode);
}

static ssize_t fts_mmi_pocket_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret = 0;
	struct fts_ts_data *ts_data;
	uint8_t read_data = 0;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	FTS_FUNC_ENTER();

	mutex_lock(&ts_data->mode_lock);
	ret = kstrtoul(buf, 0, &value);
	if (ret < 0) {
		dev_err(dev, "pocket_mode: Failed to convert value\n");
		mutex_unlock(&ts_data->mode_lock);
		FTS_FUNC_EXIT();
		return -EINVAL;
	}
	switch (value) {
		case 0x10:
		case 0x20:
			dev_info(dev, "%s: touch pocket mode disable\n", __func__);
			ts_data->get_mode.pocket_mode = 0;
			break;
		case 0x11:
		case 0x21:
			dev_info(dev, "%s: touch pocket mode enable\n", __func__);
			ts_data->get_mode.pocket_mode = 1;
			break;
		default:
			dev_info(dev, "%s: unsupport pocket mode type, value = %lu\n", __func__, value);
			mutex_unlock(&ts_data->mode_lock);
			FTS_FUNC_EXIT();
			return -EINVAL;
	}

	if (ts_data->set_mode.pocket_mode == ts_data->get_mode.pocket_mode) {
		FTS_INFO("The value = %d is same, so not to write", ts_data->get_mode.pocket_mode);
		goto exit;
	}

	if (ts_data->power_disabled == 1) {
		FTS_INFO("The touch is in sleep state, restore the value when resume\n");
		goto exit;
	}

	ret = fts_read_reg(FTS_REG_POCKET_MODE_EN, &read_data);
	if (ret < 0) {
		FTS_ERROR("read 8b register fail, ret=%d", ret);
		goto exit;
	}

	if(ts_data->get_mode.pocket_mode) {
		read_data |= (1 << 7);
	} else {
		read_data &= ~(1 << 7);
	}

	ret = fts_write_reg(FTS_REG_POCKET_MODE_EN, read_data);
	if(ret < 0){
		FTS_ERROR("Failed to set pocket mode\n");
		goto exit;
	}

	ts_data->set_mode.pocket_mode = ts_data->get_mode.pocket_mode;
	msleep(20);

	FTS_INFO("Success set %d to pocket mode, 0x8B = 0x%02x\n", ts_data->set_mode.pocket_mode,
		read_data);
exit:
	mutex_unlock(&ts_data->mode_lock);
	FTS_FUNC_EXIT();
	return size;
}

static DEVICE_ATTR(pocket_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_pocket_mode_show, fts_mmi_pocket_mode_store);

static int fts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	struct fts_ts_platform_data *pdata = NULL;
	struct fts_ts_data *ts_data;

	GET_TS_DATA(dev);
	pdata = ts_data->pdata;

#ifdef CONFIG_FTS_LAST_TIME
	ADD_ATTR(timestamp);
#endif

	if(pdata->pocket_mode_ctrl)
		ADD_ATTR(pocket_mode);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

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
	/* vendor specific attribute group */
	.extend_attribute_group = fts_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = fts_mmi_panel_state,
	.pre_resume = fts_mmi_pre_resume,
	.post_resume = fts_mmi_post_resume,
	.pre_suspend = fts_mmi_pre_suspend,
	.post_suspend = fts_mmi_post_suspend,

};

int fts_mmi_dev_register(struct fts_ts_data *ts_data) {
	int ret;

	mutex_init(&ts_data->mode_lock);
	ret = ts_mmi_dev_register(ts_data->dev, &fts_mmi_methods);
	if (ret) {
		dev_err(ts_data->dev, "Failed to register ts mmi\n");
		mutex_destroy(&ts_data->mode_lock);
		return ret;
	}

	/* initialize class imported methods */
	ts_data->imports = &fts_mmi_methods.exports;

	return 0;
}

void fts_mmi_dev_unregister(struct fts_ts_data *ts_data) {
	mutex_destroy(&ts_data->mode_lock);
	ts_mmi_dev_unregister(ts_data->dev);
}

