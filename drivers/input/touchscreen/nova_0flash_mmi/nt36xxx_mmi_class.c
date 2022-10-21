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

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/touchscreen_mmi.h>
#include <linux/regulator/consumer.h>
#include "nt36xxx.h"

#define GET_TS_DATA(dev) { \
	ts_data = dev_get_drvdata(dev); \
	if (!ts_data) { \
		NVT_ERR("Failed to get driver data"); \
		return -ENODEV; \
	} \
}

static ssize_t nvt_mmi_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t nvt_mmi_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t nvt_mmi_first_filter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t nvt_mmi_first_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t nvt_mmi_jitter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t nvt_mmi_jitter_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t nvt_mmi_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t nvt_mmi_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
	nvt_mmi_interpolation_show, nvt_mmi_interpolation_store);
static DEVICE_ATTR(first_filter, (S_IRUGO | S_IWUSR | S_IWGRP),
	nvt_mmi_first_filter_show, nvt_mmi_first_filter_store);
static DEVICE_ATTR(jitter, (S_IRUGO | S_IWUSR | S_IWGRP),
	nvt_mmi_jitter_show, nvt_mmi_jitter_store);
static DEVICE_ATTR(edge, (S_IRUGO | S_IWUSR | S_IWGRP),
	nvt_mmi_edge_show, nvt_mmi_edge_store);

#define MAX_ATTRS_ENTRIES 10
#define UI_FEATURE_ENABLE   1
#define UI_FEATURE_DISABLE   0
#define CMD_ON 0x71
#define CMD_OFF 0x72
#define ROTATE_0   0
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define DEFAULT_MODE   0
#define BIG_MODE   1
#define SMALL_MODE   2
#define CLOSE_MODE   3
#define DEFAULT_EDGE   0
#define BIG_EDGE   1
#define SMALL_EDGE   2
#define CLOSE_EDGE   3

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

#ifdef PALM_GESTURE
static ssize_t nvt_palm_settings_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", ts->palm_enabled);
}

static ssize_t nvt_palm_settings_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)

{
	int value;
	int err = 0;

	if (count > 2)
		return -EINVAL;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	err = count;

	switch (value) {
		case 0:
			ts->palm_enabled = false;
			break;
		case 1:
			ts->palm_enabled = true;
			break;
		default:
			err = -EINVAL;
			ts->palm_enabled = false;
			NVT_ERR("Invalid Value! %d\n", value);
			break;
	}

	nvt_palm_set(ts->palm_enabled);

	return err;
}

static DEVICE_ATTR(palm_settings, S_IRUGO | S_IWUSR | S_IWGRP, nvt_palm_settings_show, nvt_palm_settings_store);
#endif

static int nvt_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;

	if (ts->interpolation_ctrl)
		ADD_ATTR(interpolation);

	if (ts->jitter_ctrl)
		ADD_ATTR(jitter);

	if (ts->first_filter_ctrl)
		ADD_ATTR(first_filter);

	if (ts->edge_ctrl)
		ADD_ATTR(edge);

#ifdef PALM_GESTURE
	ADD_ATTR(palm_settings);
#endif

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

int32_t nvt_cmd_write(uint8_t u8Cmd, uint8_t u8subCmd, uint8_t u8subCmd2, uint16_t len)
{
	int i, retry = 5;
	uint8_t buf[8] = {0};
	int32_t ret = 0;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		mutex_unlock(&ts->lock);
		return ret;
	}

	for (i = 0; i < retry; i++) {
		//---set cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8subCmd;
		buf[3] = u8subCmd2;
		CTP_SPI_WRITE(ts->client, buf, len+1);
		msleep(20);

		//---read cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);
		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i == retry)) {
		NVT_LOG("send Cmd 0x%02X 0x%02X 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, u8subCmd, u8subCmd2, buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send Cmd 0x%02X 0x%02X 0x%02X success, tried %d times\n", u8Cmd, u8subCmd, u8subCmd2, i);
	}

	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t nvt_mmi_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	uint8_t mode = 0;
	uint8_t cmd = 0;

	ret = kstrtou8(buf, 0, &mode);
	if (ret < 0) {
		NVT_ERR("Failed to convert value.\n");
		return -EINVAL;
	}
	NVT_LOG("mode=%d\n", mode);

	if (UI_FEATURE_ENABLE == mode)
		cmd = CMD_ON;
	else
		cmd = CMD_OFF;

	if (ts->interpolation_cmd[0] != cmd) {
		ts->interpolation_cmd[0] = cmd;
		nvt_cmd_write(ts->interpolation_cmd[0], ts->interpolation_cmd[1], 0, 2);
	}

	return size;
}

static ssize_t nvt_mmi_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ts->interpolation_cmd[0]);
}

static ssize_t nvt_mmi_jitter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	uint8_t mode = 0;
	uint8_t cmd = 0;

	ret = kstrtou8(buf, 0, &mode);
	if (ret < 0) {
		NVT_ERR("Failed to convert value.\n");
		return -EINVAL;
	}
	NVT_LOG("mode=%d\n", mode);

	if (UI_FEATURE_ENABLE == mode)
		cmd = CMD_ON;
	else
		cmd = CMD_OFF;

	if (ts->jitter_cmd[0] != cmd) {
		ts->jitter_cmd[0] = cmd;
		nvt_cmd_write(ts->jitter_cmd[0], ts->jitter_cmd[1], 0, 2);
	}

	return size;
}

static ssize_t nvt_mmi_jitter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ts->jitter_cmd[0]);
}

static ssize_t nvt_mmi_first_filter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	uint8_t mode = 0;
	uint8_t cmd = 0;

	ret = kstrtou8(buf, 0, &mode);
	if (ret < 0) {
		NVT_ERR("Failed to convert value.\n");
		return -EINVAL;
	}
	NVT_LOG("mode=%d\n", mode);

	if (UI_FEATURE_ENABLE == mode)
		cmd = CMD_ON;
	else
		cmd = CMD_OFF;

	if (ts->first_filter_cmd[0] != cmd) {
		ts->first_filter_cmd[0] = cmd;
		nvt_cmd_write(ts->first_filter_cmd[0], ts->first_filter_cmd[1], 0, 2);
	}

	return size;
}

static ssize_t nvt_mmi_first_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ts->first_filter_cmd[0]);
}

static ssize_t nvt_mmi_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned int args[2] = { 0 };
	uint8_t mode = 0;
	uint8_t rotate = 0;

	ret = sscanf(buf, "%d %d", &args[0], &args[1]);
	if (ret < 2)
		return -EINVAL;
	NVT_LOG("mode=%d, rotate=%d\n", args[0], args[1]);
	
	if (DEFAULT_MODE == args[0]) {
		ts->edge_cmd[0] = CMD_OFF;
		ts->edge_cmd[2] = DEFAULT_EDGE;
		nvt_cmd_write(ts->edge_cmd[0], ts->edge_cmd[1], 0, 2);
		return size;
	} else if (BIG_MODE == args[0])
		mode = BIG_EDGE;
	else if (SMALL_MODE == args[0])
		mode = SMALL_EDGE;
	else if (CLOSE_MODE == args[0])
		mode = CLOSE_EDGE;
	else {
		NVT_ERR("Invalid mode %d!\n", args[0]);
		return size;
	}

	if (ts->edge_cmd[2] != mode) {
		ts->edge_cmd[0] = CMD_ON;
		ts->edge_cmd[2] = mode;
		nvt_cmd_write(ts->edge_cmd[0], ts->edge_cmd[1], ts->edge_cmd[2], 3);
	}

	if (ROTATE_0 == args[1])
		rotate = EDGE_REJECT_VERTICLE_CMD;
	else if (ROTATE_90 == args[1])
		rotate = EDGE_REJECT_LEFT_UP;
	else if (ROTATE_270 == args[1])
		rotate = EDGE_REJECT_RIGHT_UP;
	else {
		NVT_ERR("Invalid rotate %d!\n", args[1]);
		return size;
	}

	if (ts->rotate_cmd!= rotate) {
		ts->rotate_cmd = rotate;
		nvt_cmd_write(ts->rotate_cmd, 0, 0, 1);
	}

	return size;
}

static ssize_t nvt_mmi_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n", ts->edge_cmd[2], ts->rotate_cmd);
}

static int nvt_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "novatek_ts");
}

static int nvt_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", ts->product_id);
}

static int nvt_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	struct nvt_ts_data *ts_data;
	int buildid;

	GET_TS_DATA(dev);
	buildid = (ts->nvt_pid ? ts->nvt_pid : ts->config_id);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", buildid);
}

/* Read fw_version, date_Y, date_M, date_D */
static int nvt_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct nvt_ts_data *ts_data;
	uint8_t tmp_buf[8] = {0};
	uint8_t fw_version = 0;
	uint8_t date_Y = 0;
	uint8_t date_M = 0;
	uint8_t date_D = 0;

	GET_TS_DATA(dev);

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_data->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);


	tmp_buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts_data->client, tmp_buf, 2);
	fw_version = tmp_buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_data->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);
	tmp_buf[0] = EVENT_MAP_PROJECTID;
	CTP_SPI_READ(ts_data->client, tmp_buf, 6);
	date_Y = tmp_buf[3];
	date_M = tmp_buf[4];
	date_D = tmp_buf[5];
	nvt_set_page(ts_data->mmap->EVENT_BUF_ADDR);

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02d%02d%02d%02x",
						 date_Y, date_M, date_D, fw_version);
}

static int nvt_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	struct nvt_ts_data *ts_data;
	struct input_dev *input_dev;

	GET_TS_DATA(dev);
	input_dev = ts_data->input_dev;
	TO_INT(idata) = ts->input_dev->id.bustype == BUS_SPI ?
			TOUCHSCREEN_MMI_BUS_TYPE_SPI : TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int nvt_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	TO_INT(idata) = gpio_get_value(ts_data->irq_gpio);
	return 0;
}

static int nvt_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	TO_INT(idata) = ts_data->irq_enabled ? 1 : 0;
	return 0;
}

static int nvt_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	TO_INT(idata) = 1;

	return 0;
}

static int nvt_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	TO_INT(idata) = atomic_read(&ts_data->loading_fw);

	return 0;
}

#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
static int nvt_mmi_get_flash_mode(struct device *dev, void *idata)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	TO_INT(idata) = ts_data->flash_mode;

	return 0;
}

static int nvt_mmi_flash_mode(struct device *dev, int mode)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	ts_data->flash_mode = mode;

	return 0;
}
#endif

static int nvt_mmi_methods_drv_irq(struct device *dev, int state)
{
	int ret = 0;

	if (state == 1)
		nvt_irq_enable(true);
	else if (state == 0) {
		nvt_irq_enable(false);
	} else {
		dev_err(dev, "%s: invalid value\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static int nvt_mmi_charger_mode(struct device *dev, int mode)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		NVT_ERR("Failed to lock in mutex_lock_interruptible(&ts->lock).\n");
		return -EINVAL;
	}

	nvt_set_charger(mode);
	NVT_LOG("Charger mode %s!\n", !!mode ? "Enabled" : "Disabled");
	mutex_unlock(&ts->lock);

	return 0;
}

static int nvt_mmi_methods_reset(struct device *dev, int type)
{
	nvt_sw_reset();

	return 0;
}

static int nvt_mmi_firmware_update(struct device *dev, char *fwname)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);

	mutex_lock(&ts->lock);
	if (strncmp(fwname, "Default", sizeof("Default"))) {
		snprintf(nvt_boot_firmware_name, NVT_FILE_NAME_LENGTH, "%s", fwname);
		snprintf(nvt_mp_firmware_name, NVT_FILE_NAME_LENGTH, "mp-%s", fwname);
	}
	nvt_update_firmware(nvt_boot_firmware_name);
	mutex_unlock(&ts->lock);

#ifdef NOVATECH_PEN_NOTIFIER
	if(!ts->fw_ready_flag)
		ts->fw_ready_flag = true;
	nvt_mcu_pen_detect_set(ts->nvt_pen_detect_flag);
#endif

	return 0;
}

static int nvt_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct nvt_ts_data *ts_data;
	static uint8_t gesture_cmd = 0x00;

	GET_TS_DATA(dev);
	dev_dbg(dev, "%s: panel state change: %d->%d\n", __func__, from, to);
	NVT_LOG("panel state change: %d->%d\n", from, to);
	mutex_lock(&ts->lock);
	switch (to) {
		case TS_MMI_PM_GESTURE:
#ifdef NVT_SET_TOUCH_STATE
			ts->gesture_enabled = true;
			touch_set_state(TS_MMI_PM_GESTURE, TOUCH_PANEL_IDX_PRIMARY);
#else
			ts->gesture_enabled = false;
#endif
			break;

		case TS_MMI_PM_GESTURE_SINGLE:
			gesture_cmd |= 0x02;
			NVT_LOG("enable single gesture mode cmd 0x%04x\n", gesture_cmd);
			break;
		case TS_MMI_PM_GESTURE_DOUBLE:
			gesture_cmd |= 0x04;
			NVT_LOG("enable double gesture mode cmd 0x%04x\n", gesture_cmd);
			break;
		case TS_MMI_PM_GESTURE_SWITCH:
#ifdef NVT_SET_TOUCH_STATE
			ts->gesture_enabled = true;
			touch_set_state(TS_MMI_PM_GESTURE, TOUCH_PANEL_IDX_PRIMARY);
#endif
			//---write command to enter "wakeup gesture mode"---
			nvt_cmd_ext_store(DOUBLE_TAP_GESTURE_MODE_CMD, gesture_cmd);
			gesture_cmd = 0x00;
			break;
		case TS_MMI_PM_DEEPSLEEP:
			ts->gesture_enabled = false;
#ifdef NVT_SET_TOUCH_STATE
			touch_set_state(TS_MMI_PM_DEEPSLEEP, TOUCH_PANEL_IDX_PRIMARY);
#endif
			break;

		case TS_MMI_PM_ACTIVE:
			break;
		default:
			mutex_unlock(&ts->lock);
			dev_warn(dev, "panel mode %d is invalid.\n", to);
			return -EINVAL;
	}
	mutex_unlock(&ts->lock);
	NVT_LOG("IRQ is %s\n", ts_data->irq_enabled ? "EN" : "DIS");
	return 0;
}

int32_t nvt_mmi_post_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};

	mutex_lock(&ts->lock);
	NVT_LOG("enter\n");

	if (ts->gesture_enabled) {
		//---write command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);
		enable_irq_wake(ts->client->irq);
		NVT_LOG("Enabled touch wakeup gesture\n");
	} else {
		//---write command to enter "deep sleep mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);
	}
	ts->bTouchIsAwake = 0;

	mutex_unlock(&ts->lock);

	release_all_touches();
	msleep(50);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nvt_mmi_pre_resume(struct device *dev)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	NVT_LOG("enter\n");

#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
	if (ts_data->flash_mode == FW_PARAM_MODE)
	{
		NVT_LOG("update firmware through touchUpg service\n");
		return 0;
	}
#endif

	mutex_lock(&ts->lock);
	NVT_LOG("update firmware: %s\n", nvt_boot_firmware_name);
	nvt_update_firmware(nvt_boot_firmware_name);

	mutex_unlock(&ts->lock);

	return 0;
}

static int nvt_mmi_post_resume(struct device *dev)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	NVT_LOG("enter\n");

	mutex_lock(&ts->lock);
	if (ts->gesture_enabled)
		disable_irq_wake(ts_data->client->irq);

	ts->bTouchIsAwake = 1;
	mutex_unlock(&ts->lock);

	/* restore data */
	if (ts->interpolation_ctrl) {
		nvt_cmd_write(ts->interpolation_cmd[0], ts->interpolation_cmd[1], 0, 2);
	}

	if (ts->jitter_ctrl) {
		nvt_cmd_write(ts->jitter_cmd[0], ts->jitter_cmd[1], 0, 2);
	}

	if (ts->first_filter_ctrl) {
		nvt_cmd_write(ts->first_filter_cmd[0], ts->first_filter_cmd[1], 0, 2);
	}

	if (ts->edge_ctrl) {
		if (ts->edge_cmd[0] == CMD_ON)
			nvt_cmd_write(ts->edge_cmd[0], ts->edge_cmd[1], ts->edge_cmd[2], 3);
		else
			nvt_cmd_write(ts->edge_cmd[0], ts->edge_cmd[1], 0, 2);

		nvt_cmd_write(ts->rotate_cmd, 0, 0, 1);
	}

	NVT_LOG("IRQ is %s\n", ts_data->irq_enabled ? "EN" : "DIS");
	return 0;
}

static struct ts_mmi_methods nvt_mmi_methods = {
	.get_vendor = nvt_mmi_methods_get_vendor,
	.get_productinfo = nvt_mmi_methods_get_productinfo,
	.get_build_id = nvt_mmi_methods_get_build_id,
	.get_config_id = nvt_mmi_methods_get_config_id,
	.get_bus_type = nvt_mmi_methods_get_bus_type,
	.get_irq_status = nvt_mmi_methods_get_irq_status,
	.get_drv_irq = nvt_mmi_methods_get_drv_irq,
	.get_flashprog = nvt_mmi_methods_get_flashprog,
#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
	.get_flash_mode = nvt_mmi_get_flash_mode,
#endif
	.get_poweron = nvt_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  nvt_mmi_methods_reset,
	.drv_irq = nvt_mmi_methods_drv_irq,
	.charger_mode = nvt_mmi_charger_mode,
	/* Firmware */
#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
	.flash_mode = nvt_mmi_flash_mode,
#endif
	.firmware_update = nvt_mmi_firmware_update,
	.extend_attribute_group = nvt_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = nvt_mmi_panel_state,
	.pre_resume = nvt_mmi_pre_resume,
	.post_resume = nvt_mmi_post_resume,
	.post_suspend = nvt_mmi_post_suspend,

};

int nvt_mmi_init(struct nvt_ts_data *ts_data, bool enable) {
	int ret = 0;

	if (enable) {
		ret = ts_mmi_dev_register(&ts_data->client->dev, &nvt_mmi_methods);
		if (ret)
			dev_err(&ts_data->client->dev, "Failed to register ts mmi\n");
		/* initialize class imported methods */
		ts_data->imports = &nvt_mmi_methods.exports;
	} else
		ts_mmi_dev_unregister(&ts_data->client->dev);

	return ret;
}
