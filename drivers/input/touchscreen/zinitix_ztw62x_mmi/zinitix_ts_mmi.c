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

#define ADD_ATTR(name) { \
	if (idx < MAX_ATTRS_ENTRIES)  { \
		dev_info(dev, "%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
		ext_attributes[idx] = &dev_attr_##name.attr; \
		idx++; \
	} else { \
		dev_err(dev, "%s: cannot add attribute '%s'\n", __func__, #name); \
	} \
}

#define GET_TS_DATA(dev) { \
	ts_info = dev_get_drvdata(dev); \
	if (!ts_info) { \
		dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
	} \
}


static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

volatile int tpd_halt = 0;

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

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02d%02d%02d%02x",
		cap->fw_version,
		cap->fw_minor_version,
		cap->fw_day_version,
		le32_to_cpu(cap->reg_data_version));
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

	info->ts_mmi_power_state = on;
	schedule_delayed_work(&info->work, 0);

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
	unsigned char gesture_type = 0;
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	switch (to) {
	case TS_MMI_PM_GESTURE:
		if (info->imports && info->imports->get_gesture_type) {
			if (info->imports->get_gesture_type(dev, &gesture_type) < 0) {
				dev_err(dev, "%s: get gesture type error.\n", __func__);
				return -EINVAL;
			}
		}

		info->gesture_command = 0;

		if (gesture_type & TS_MMI_GESTURE_SINGLE) {
		    info->gesture_command |= 0x0089;
		    dev_info(dev, "Enable GESTURE_CLI_SINGLE command: 0x%04x", info->gesture_command);
		}
		if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
		    info->gesture_command |= 0x0091;
		    dev_info(dev, "Enable GESTURE_CLI_DOUBLE command: 0x%04x", info->gesture_command);
		}
		dev_info(dev, "CLI GESTURE SWITCH command: 0x%04x", info->gesture_command);

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
	tpd_halt = 1;
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

	//set stow mode
	mutex_lock(&info->mode_lock);
	if ((info->pdata->stow_mode_ctrl) && (info->ic_power_state == TS_MMI_POWER_ON) &&
			atomic_read(&info->get_stowed_state)) {
		dev_info(dev, "%s: stowed, disable all gestures after suspend 0x%04x\n", __func__,
			info->gesture_command);
		zinitix_ts_mmi_disable_gesture(dev);
		atomic_set(&info->set_stowed_state, 1);
	}
	mutex_unlock(&info->mode_lock);

	dev_info(dev, "%s: Suspend end \n", __func__);

	return 0;
}

static int zinitix_ts_mmi_pre_resume(struct device *dev)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	down(&info->work_lock);
	dev_info(dev, "%s: Resume start\n", __func__);
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
	tpd_halt = 0;
	info->work_state = NOTHING;
	//Clear stowed state
	atomic_set(&info->set_stowed_state, 0);
	up(&info->work_lock);
	dev_info(dev, "%s: Resume end\n", __func__);
	return 0;
}

static ssize_t zinitix_ts_stowed_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bt541_ts_info *ts_info;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	ASSERT_PTR(ts_info);

	return scnprintf(buf, PAGE_SIZE, "stowed state:%d\n", atomic_read(&ts_info->set_stowed_state));
}

static ssize_t zinitix_ts_stowed_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int mode = 0;
	int err = 0;
	struct bt541_ts_info *ts_info;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	err = sscanf(buf, "%d", &mode);
	if (err < 0) {
		dev_err(dev, "stowed: Failed to convert value\n");
		return -EINVAL;
	}

	mutex_lock(&ts_info->mode_lock);

	atomic_set(&ts_info->get_stowed_state, !!mode);

	if (atomic_read(&ts_info->set_stowed_state) == atomic_read(&ts_info->get_stowed_state)) {
		dev_info(dev, "%s: Stowed mode is same, not to set \n", __func__);
		mutex_unlock(&ts_info->mode_lock);
		return size;
	}

	if ((ts_info->ic_power_state == TS_MMI_POWER_ON) && (ts_info->work_state == SUSPEND)) {
		if (mode) {
			dev_info(dev, "%s: stowed, disable all gestures 0x%04x\n", __func__, ts_info->gesture_command);
			zinitix_ts_mmi_disable_gesture(dev);
		} else {
			dev_info(dev, "%s:  unstowed, restore gestures 0x%04x\n", __func__, ts_info->gesture_command);
			zinitix_ts_mmi_restore_gesture(dev);
		}
		atomic_set(&ts_info->set_stowed_state, !!mode);
	} else {
		dev_info(dev, "%s: touch state is not proper to enter stow mode, work state:%d, \
			ic power state: %d\n", __func__, ts_info->work_state, ts_info->ic_power_state);
	}
	mutex_unlock(&ts_info->mode_lock);

	return size;
}
static DEVICE_ATTR(stowed, (S_IWUSR | S_IWGRP | S_IRUGO), zinitix_ts_stowed_show,
	zinitix_ts_stowed_store);

static int zinitix_mmi_extend_attribute_group(struct device *dev,
	struct attribute_group **group)
{
	int idx = 0;
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct bt541_ts_platform_data *pdata = info->pdata;
	ASSERT_PTR(pdata);

	if (pdata->stow_mode_ctrl)
		ADD_ATTR(stowed);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

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
	/* vendor specific attribute group */
	.extend_attribute_group = zinitix_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = zinitix_ts_mmi_panel_state,
	.pre_suspend = zinitix_ts_mmi_pre_suspend,
	.post_suspend = zinitix_ts_mmi_post_suspend,
	.pre_resume = zinitix_ts_mmi_pre_resume,
	.post_resume = zinitix_ts_mmi_post_resume,
};

static void ts_mmi_worker_func(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct bt541_ts_info *info =
		container_of(dw, struct bt541_ts_info, work);
	struct i2c_client *client = info->client;

	down(&info->work_lock);

	if (info->ts_mmi_power_state == TS_MMI_POWER_ON)
	{
		dev_info(&client->dev, "%s: Power on touch IC \n", __func__);
		bt541_power_control(info, POWER_ON_SEQUENCE);
		mini_init_touch(info);
	}
	else if(info->ts_mmi_power_state == TS_MMI_POWER_OFF)
	{
		dev_info(&client->dev, "%s: Power off touch IC \n", __func__);
		bt541_power_control(info, POWER_OFF);
	}

	up(&info->work_lock);
}

int zinitix_ts_mmi_dev_register(struct device *dev) {
	int ret;
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	ASSERT_PTR(info);

	INIT_DELAYED_WORK(&info->work, ts_mmi_worker_func);

	mutex_init(&info->mode_lock);
	ret = ts_mmi_dev_register(dev, &zinitix_ts_mmi_methods);
	if (ret) {
		dev_err(dev, "%s, Failed to register ts mmi\n", __func__);
		mutex_destroy(&info->mode_lock);
		return ret;
	}
	dev_info(dev, "%s, Register ts mmi success\n", __func__);

	info->imports = &zinitix_ts_mmi_methods.exports;

#if defined(CONFIG_GTP_LIMIT_USE_SUPPLIER)
	if (info->imports && info->imports->get_supplier) {
		ret = info->imports->get_supplier(dev, &info->supplier);
	}
#endif

	return 0;
}

void zinitix_ts_mmi_dev_unregister(struct device *dev) {
	struct bt541_ts_info *info = dev_get_drvdata(dev);

	if (!info)
		dev_err(dev ,"Failed to get driver data");
	mutex_destroy(&info->mode_lock);
	ts_mmi_dev_unregister(dev);
}

