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
#include "ili9882.h"

static int ili_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), 16, "%s", "ilitek");
}

static int ili_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), 16, "%s", "6666");
}

static int ili_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), 16, "%04x", 0x8888);
}

/* Read fw_version, date_Y, date_M, date_D */
static int ili_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), 16, "%02d%02d%02d%02x",
						 22, 9, 8, 1);
}

static int ili_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	return 0;
}

static int ili_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	return 0;
}

static int ili_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	return 0;
}

static int ili_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	return 0;
}

static int ili_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	return 0;
}

static int ili_mmi_methods_drv_irq(struct device *dev, int state)
{
	return 0;
}

static int ili_mmi_charger_mode(struct device *dev, int mode)
{
	int ret = 0;
	mutex_lock(&ilits->touch_mutex);
	ret = ili_ic_func_ctrl("plug", !mode);
	if(ret<0) {
		ILI_ERR("Write plug in failed\n");
	}
	ILI_INFO("%s: %d\n", __func__, !mode);
	mutex_unlock(&ilits->touch_mutex);

	return 0;
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

static ssize_t ili_palm_settings_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", ilits->canvas_value);
}

static ssize_t ili_palm_settings_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)

{
	int value;
	int err = 0;

	mutex_lock(&ilits->touch_mutex);
	if (count > 2)
		return -EINVAL;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	err = count;

	if (ili_ic_func_ctrl("canvas", value) < 0)
		ILI_ERR("Write canvas enable fail\n");
	else
		ilits->canvas_value = value;
	ILI_INFO("%s: %d\n", __func__, ilits->canvas_value);
	mutex_unlock(&ilits->touch_mutex);

	return err;
}

#ifdef ILI_TOUCH_LAST_TIME
static ssize_t ili_mmi_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ktime_t last_ktime;
	struct timespec64 last_ts;

	mutex_lock(&ilits->touch_mutex);
	last_ktime = ilits->last_event_time;
	ilits->last_event_time = 0;
	mutex_unlock(&ilits->touch_mutex);

	last_ts = ktime_to_timespec64(last_ktime);

	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
static DEVICE_ATTR(timestamp, S_IRUGO, ili_mmi_timestamp_show, NULL);
#endif

static DEVICE_ATTR(palm_settings, S_IRUGO | S_IWUSR | S_IWGRP, ili_palm_settings_show, ili_palm_settings_store);

static int ili_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;

	ADD_ATTR(palm_settings);
#ifdef ILI_TOUCH_LAST_TIME
	ADD_ATTR(timestamp);
#endif

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}


static int ili_mmi_methods_reset(struct device *dev, int type)
{
	return 0;
}

static int ili_mmi_firmware_update(struct device *dev, char *fwname)
{
	return 0;
}

static int ili_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
        struct ilitek_ts_data __maybe_unused *ts_data;
#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
        unsigned char gesture_type = 0;
#endif
        GET_TS_DATA(dev);

	ILI_INFO("%s: panel state change: %d->%d\n", __func__, from, to);
	ILI_INFO("panel state change: %d->%d\n", from, to);
	switch (to) {
		case TS_MMI_PM_GESTURE:
#ifdef ILI_SET_TOUCH_STATE
			ilits->should_enable_gesture = true;
			touch_set_state(TS_MMI_PM_GESTURE, TOUCH_PANEL_IDX_PRIMARY);
#else
			ilits->should_enable_gesture = false;
#endif

#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
                        if (ts_data->imports && ts_data->imports->get_gesture_type) {
                                ts_data->imports->get_gesture_type(ts_data->dev, &gesture_type);
		        }

		        if (gesture_type & TS_MMI_GESTURE_SINGLE) {
			        ilits->ges_sym.single_tap = SINGLE_TAP;
			        ILI_INFO("enable single gesture mode\n");
		        }

		        if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
			        ilits->ges_sym.double_tap = DOUBLE_TAP;
			        ILI_INFO("enable double gesture mode\n");
		        }

			/*---write command to enter "wakeup gesture mode"---*/
			ili_sleep_handler(TP_SUSPEND);
			ilits->ges_sym.single_tap = OFF;
			ilits->ges_sym.double_tap = OFF;
#endif
			break;

		case TS_MMI_PM_DEEPSLEEP:
			ilits->should_enable_gesture = false;
#ifdef ILI_SET_TOUCH_STATE
			touch_set_state(TS_MMI_PM_DEEPSLEEP, TOUCH_PANEL_IDX_PRIMARY);
#endif
			ili_sleep_handler(TP_DEEP_SLEEP);
			break;

		case TS_MMI_PM_ACTIVE:
			break;
		default:
			ILI_INFO("panel mode %d is invalid.\n", to);
			return -EINVAL;
	}
	return 0;
}

int32_t ili_mmi_post_suspend(struct device *dev)
{
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int ili_mmi_pre_resume(struct device *dev)
{
	return 0;
}

static int ili_mmi_post_resume(struct device *dev)
{
	ili_sleep_handler(TP_RESUME);
	return 0;
}

static struct ts_mmi_methods ili_mmi_methods = {
	.get_vendor = ili_mmi_methods_get_vendor,
	.get_productinfo = ili_mmi_methods_get_productinfo,
	.get_build_id = ili_mmi_methods_get_build_id,
	.get_config_id = ili_mmi_methods_get_config_id,
	.get_bus_type = ili_mmi_methods_get_bus_type,
	.get_irq_status = ili_mmi_methods_get_irq_status,
	.get_drv_irq = ili_mmi_methods_get_drv_irq,
	.get_flashprog = ili_mmi_methods_get_flashprog,
	.get_poweron = ili_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  ili_mmi_methods_reset,
	.drv_irq = ili_mmi_methods_drv_irq,
	.charger_mode = ili_mmi_charger_mode,
	/* Firmware */
	.firmware_update = ili_mmi_firmware_update,
	.extend_attribute_group = ili_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = ili_mmi_panel_state,
	.pre_resume = ili_mmi_pre_resume,
	.post_resume = ili_mmi_post_resume,
	.post_suspend = ili_mmi_post_suspend,

};

int ili_mmi_init(struct ilitek_ts_data *ts_data, bool enable) {
	int ret = 0;

	if (enable) {
		ret = ts_mmi_dev_register(&ts_data->spi->dev, &ili_mmi_methods);
		if (ret)
			ILI_INFO("Failed to register ts mmi\n");
		/* initialize class imported methods */
		ts_data->imports = &ili_mmi_methods.exports;
	} else
		ts_mmi_dev_unregister(&ts_data->spi->dev);

	return ret;
}
