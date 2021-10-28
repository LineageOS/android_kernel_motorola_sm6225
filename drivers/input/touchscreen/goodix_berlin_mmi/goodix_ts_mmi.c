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

#include "goodix_ts_mmi.h"
#include "goodix_ts_core.h"
#include <linux/input/mt.h>

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

static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(stylus_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_stylus_mode_show, goodix_ts_stylus_mode_store);

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

static int goodix_ts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->board_data.stylus_mode_ctrl)
		ADD_ATTR(stylus_mode);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

#define STYLUS_MODE_SWITCH_CMD    0xA4
static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct goodix_ts_cmd cmd;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	if (core_data->stylus_mode == mode) {
		ts_debug("value is same,so not write.\n");
		return size;
	}

	cmd.cmd = STYLUS_MODE_SWITCH_CMD;
	cmd.len = 5;
	cmd.data[0] = mode;
	ret = hw_ops->send_cmd(core_data, &cmd);
	if (ret < 0) {
		ts_err("failed send stylus mode cmd");
	}

	core_data->stylus_mode = mode;
	ts_info("Success to %s stylus mode", mode ? "Enable" : "Disable");
	return size;
}

static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("Stylus mode = %d.\n", core_data->stylus_mode);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->stylus_mode);
}

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}

static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_ts_board_data *ts_bdata;
	char* ic_info;

	GET_GOODIX_DATA(dev);

	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}

	ic_info = strstr(ts_bdata->ic_name, ",");
	ic_info++;

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", ic_info);
}

#define TOUCH_CFG_VERSION_ADDR    0x10076
static int goodix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	u32 cfg_id;

	GET_GOODIX_DATA(dev);

	ret = core_data->hw_ops->read(core_data, TOUCH_CFG_VERSION_ADDR,
			(u8*)&cfg_id,  sizeof(cfg_id));
	if (ret) {
		ts_info("failed get fw version data, %d", ret);
		return -EINVAL;
	}

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", le32_to_cpu(cfg_id));
}

#define TOUCH_FW_VERSION_ADDR    0x1007E
/*return firmware version*/
static int goodix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	int ret;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	u8 fw_id[4] = {0};

	GET_GOODIX_DATA(dev);

	ret = core_data->hw_ops->read(core_data, TOUCH_FW_VERSION_ADDR,
			fw_id, sizeof(fw_id));
	if (ret) {
		ts_info("failed get fw version data, %d", ret);
		return -EINVAL;
	}

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x%02x%02x%02x",
			fw_id[0], fw_id[1], fw_id[2], fw_id[3]);
}

static int goodix_ts_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = core_data->bus->bus_type == GOODIX_BUS_TYPE_I2C ?
			TOUCHSCREEN_MMI_BUS_TYPE_I2C : TOUCHSCREEN_MMI_BUS_TYPE_SPI;
	return 0;
}

static int goodix_ts_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	struct goodix_ts_board_data *ts_bdata;

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
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = atomic_read(&core_data->irq_enabled);
	return 0;
}

static int goodix_ts_mmi_methods_get_poweron(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = core_data->power_on;
	return 0;
}

static int goodix_ts_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	TO_INT(idata) = core_data->update_status;
	return 0;
}

static int goodix_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->hw_ops->irq_enable)
		ret = core_data->hw_ops->irq_enable(core_data, !(!state));

	return ret;
}

/* reset chip
 * type：can control software reset and hardware reset,
 * but GOODIX has no software reset,
 * so the type parameter is not used here.
 */
static int goodix_ts_mmi_methods_reset(struct device *dev, int type) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	if (core_data->hw_ops->reset)
		ret = core_data->hw_ops->reset(core_data, GOODIX_NORMAL_RESET_DELAY_MS);
	return ret;
}

static int goodix_ts_firmware_update(struct device *dev, char *fwname) {
	int ret = -ENODEV;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	ts_info("HW request update fw, %s", fwname);
	/* set firmware image name */
	if (core_data->set_fw_name)
		core_data->set_fw_name(fwname);

	ret = goodix_do_fw_update(core_data->ic_configs[CONFIG_TYPE_NORMAL],
				UPDATE_MODE_SRC_REQUEST | UPDATE_MODE_BLOCK | UPDATE_MODE_FORCE);
	if (ret)
		ts_err("failed do fw update");

	return 0;
}

static int goodix_ts_mmi_methods_power(struct device *dev, int on) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

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
	int ret = 0;
	struct goodix_ts_cmd cmd;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	cmd.cmd = CHARGER_MODE_CMD;
	cmd.len = 5;
	cmd.data[0] = mode;
	ret = hw_ops->send_cmd(core_data, &cmd);
	if (ret < 0) {
		ts_err("Failed to set charger mode\n");
	}
	ts_err("Success to %s charger mode\n", mode ? "Enable" : "Disable");

	return 0;
}

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	switch (to) {
	case TS_MMI_PM_GESTURE:
		if (hw_ops->gesture)
			hw_ops->gesture(core_data, 0);
		enable_irq_wake(core_data->irq);
		core_data->gesture_enabled = true;
		break;
	case TS_MMI_PM_DEEPSLEEP:
		/* enter sleep mode or power off */
		if (hw_ops->suspend)
			hw_ops->suspend(core_data);
		core_data->gesture_enabled = false;
		break;
	case TS_MMI_PM_ACTIVE:
		break;
	default:
		ts_err("Invalid power state parameter %d.\n", to);
		return -EINVAL;
	}

	return 0;
}

static int goodix_ts_mmi_pre_resume(struct device *dev) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	ts_info("Resume start");
	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	atomic_set(&core_data->suspended, 0);
	if (core_data->gesture_enabled)
		disable_irq_wake(core_data->irq);

	return 0;
}

static int goodix_ts_mmi_post_resume(struct device *dev) {
	int ret = 0;
	struct goodix_ts_cmd cmd;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	/* restore data */
	if (core_data->board_data.stylus_mode_ctrl) {
		cmd.cmd = STYLUS_MODE_SWITCH_CMD;
		cmd.len = 5;
		cmd.data[0] = core_data->stylus_mode;
		ret = hw_ops->send_cmd(core_data, &cmd);
		if (!ret) {
			ts_info(" Successfully to restore stylus mode = %d.",
				core_data->stylus_mode);
		}
	}

	return 0;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev) {
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);
	return 0;
}

static int goodix_ts_mmi_post_suspend(struct device *dev) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	goodix_ts_release_connects(core_data);

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
	.get_flashprog = goodix_ts_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  goodix_ts_mmi_methods_reset,
	.drv_irq = goodix_ts_mmi_methods_drv_irq,
	.power = goodix_ts_mmi_methods_power,
	.charger_mode = goodix_ts_mmi_charger_mode,
	/* Firmware */
	.firmware_update = goodix_ts_firmware_update,
	/* vendor specific attribute group */
	.extend_attribute_group = goodix_ts_mmi_extend_attribute_group,
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
	ret = ts_mmi_dev_register(core_data->bus->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		return ret;
	}

	core_data->imports = &goodix_ts_mmi_methods.exports;

#if defined(CONFIG_GTP_LIMIT_USE_SUPPLIER)
	if (core_data->imports && core_data->imports->get_supplier) {
		ret = core_data->imports->get_supplier(core_data->bus->dev, &core_data->supplier);
	}
#endif
	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_err("Failed to get driver data");
	ts_mmi_dev_unregister(&pdev->dev);
}
