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
#include <linux/delay.h>
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

static ssize_t goodix_ts_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_sensitivity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(edge, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_edge_show, goodix_ts_edge_store);
static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_interpolation_show, goodix_ts_interpolation_store);
static DEVICE_ATTR(stylus_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_stylus_mode_show, goodix_ts_stylus_mode_store);
static DEVICE_ATTR(sensitivity, (S_IRUGO | S_IWUSR | S_IWGRP),
	NULL, goodix_ts_sensitivity_store);

/* hal settings */
#define ROTATE_0   0
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define BIG_MODE   1
#define SMALL_MODE    2
#define DEFAULT_MODE   0
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

	if (core_data->board_data.edge_ctrl)
		ADD_ATTR(edge);

	if (core_data->board_data.interpolation_ctrl)
		ADD_ATTR(interpolation);

	if (core_data->board_data.stylus_mode_ctrl)
		ADD_ATTR(stylus_mode);

	if (core_data->board_data.sensitivity_ctrl)
		ADD_ATTR(sensitivity);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

static int goodix_ts_send_cmd(struct goodix_ts_core *core_data,
		u8 cmd, u8 len, u8 subCmd, u8 subCmd2)
{
	int ret = 0;
	struct goodix_ts_cmd ts_cmd;

	ts_cmd.cmd = cmd;
	ts_cmd.len = len;
	ts_cmd.data[0] = subCmd;
	ts_cmd.data[1] = subCmd2;

	ret = core_data->hw_ops->send_cmd(core_data, &ts_cmd);
	return ret;
}

static int goodix_ts_film_mode(struct goodix_ts_core *core_data, int mode)
{
	int ret;

	if (core_data->film_mode == mode) {
		ts_debug("value is same,so not write.\n");
		return 0;
	}

	if (core_data->power_on == 0) {
		core_data->film_mode = mode;
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return 0;
	}

	ret = goodix_ts_send_cmd(core_data, FILM_MODE_SWITCH_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("failed to send leather mode cmd");
		return -EINVAL;
	}

	msleep(20);
	core_data->film_mode = mode;
	ts_info("Success to %s film mode", mode ? "Enable" : "Disable");
	return ret;
}

static int goodix_ts_leather_mode(struct goodix_ts_core *core_data, int mode)
{
	int ret;

	if (core_data->leather_mode == mode) {
		ts_debug("value is same,so not write.\n");
		return 0;
	}

	if (core_data->power_on == 0) {
		core_data->leather_mode = mode;
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return 0;
	}

	ret = goodix_ts_send_cmd(core_data, LEATHER_MODE_SWITCH_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("failed to send leather mode cmd");
		return -EINVAL;
	}

	msleep(20);
	core_data->leather_mode = mode;
	ts_info("Success to %s leather mode", mode ? "Enable" : "Disable");
	return ret;
}
/*
 * This is a common interface to the HAL layer.
 * mode = 0x00/0x01 Exit/Enter film sensitivity mode.
 * mode = 0x10/0x11 Exit/Enter leahter sensitivity mode.
 */
static ssize_t goodix_ts_sensitivity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	switch (mode) {
		case 0x00:
			if (core_data->board_data.film_mode_ctrl)
				ret = goodix_ts_film_mode(core_data, EXIT_FILM_MODE);
			break;
		case 0x01:
			if (core_data->board_data.film_mode_ctrl)
				ret = goodix_ts_film_mode(core_data, ENTER_FILM_MODE);
			break;
		case 0x10:
			if (core_data->board_data.leather_mode_ctrl)
				ret = goodix_ts_leather_mode(core_data, EXIT_LEATHER_MODE);
			break;
		case 0x11:
			if (core_data->board_data.leather_mode_ctrl)
				ret = goodix_ts_leather_mode(core_data, ENTER_LEATHER_MODE);
			break;
		default:
			ts_err("The mode = %lu does not support\n", mode);
			return size;
	}
	if (!ret)
		return size;
	else
		return ret;
}

static int goodix_clock_enable(struct goodix_ts_core *core_data, bool mode) {
	int ret = 0;
	struct goodix_ts_board_data *ts_bdata;

	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}

	if (mode) {
		if (!strcmp(ts_bdata->stylus_clk_src, STYLUS_CLK_SRC_PMIC)) {
			if (IS_ERR_OR_NULL(core_data->stylus_clk)) {
				ts_err("failed to get stylus clk\n");
				return -EINVAL;
			}
			ret = clk_prepare_enable(core_data->stylus_clk);
			if (ret) {
				ts_err("failed to enable stylus clk\n");
				return ret;
			}
		} else if (!strcmp(ts_bdata->stylus_clk_src, STYLUS_CLK_SRC_GPIO)){
			if (IS_ERR_OR_NULL(core_data->stylus_clk_active)) {
				ts_err("Failed to get state clk pinctrl state:%s",
					PINCTRL_STYLUS_CLK_ACTIVE);
				core_data->stylus_clk_active = NULL;
				return -EINVAL;
			}
			ret = pinctrl_select_state(core_data->pinctrl,
						core_data->stylus_clk_active);
			if (ret < 0) {
				ts_err("Failed to select active stylus clk state, ret:%d", ret);
				return ret;
			}
		} else {
			ts_err("stylus clock source is invalid");
			return -EINVAL;
		}
		ts_info("success to enable stylus clk\n");
	} else {
		if (!strcmp(ts_bdata->stylus_clk_src, STYLUS_CLK_SRC_PMIC)) {
			if (IS_ERR_OR_NULL(core_data->stylus_clk)) {
				ts_err("failed to get stylus clk\n");
				return -EINVAL;
			}
			clk_disable_unprepare(core_data->stylus_clk);
		} else if (!strcmp(ts_bdata->stylus_clk_src, STYLUS_CLK_SRC_GPIO)){
			if (IS_ERR_OR_NULL(core_data->stylus_clk_suspend)) {
				ts_err("Failed to get state clk pinctrl state:%s",
					PINCTRL_STYLUS_CLK_SUSPEND);
				core_data->stylus_clk_suspend = NULL;
				return -EINVAL;
			}
			ret = pinctrl_select_state(core_data->pinctrl,
						core_data->stylus_clk_suspend);
			if (ret < 0) {
				ts_err("Failed to select stylus clk suspend state, ret:%d", ret);
				return ret;
			}
		} else {
			ts_err("stylus clock source is invalid");
			return -EINVAL;
		}
		ts_info("success to disable stylus clk\n");
	}
	return ret;
}

static int goodix_stylus_mode(struct goodix_ts_core *core_data, int mode) {
	int ret = 0;

	if (mode) {
		goodix_clock_enable(core_data, mode);
		ret = goodix_ts_send_cmd(core_data, STYLUS_MODE_SWITCH_CMD, 5, mode, 0x00);
		msleep(20);
	} else {
		ret = goodix_ts_send_cmd(core_data, STYLUS_MODE_SWITCH_CMD, 5, mode, 0x00);
		msleep(20);
		goodix_clock_enable(core_data, mode);
	}

	return ret;
}

static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
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

	if (core_data->power_on == 0) {
		core_data->stylus_mode = mode;
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return size;
	}

	ret = goodix_stylus_mode(core_data, mode);
	if (ret) {
		return ret;
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

#define DSI_REFRESH_RATE_144			144
static int goodix_ts_mmi_set_report_rate(struct goodix_ts_core *core_data)
{
	int ret = 0;
	int mode = 0;

	if (core_data->interpolation == 0x00) {
		mode = REPORT_RATE_DEFAULT;
		goto ts_send_cmd;
	}

	if (core_data->board_data.report_rate_ctrl) {
		if (core_data->refresh_rate == DSI_REFRESH_RATE_144)
			mode = REPORT_RATE_576HZ;
		else
			mode = REPORT_RATE_480HZ;
	} else
		mode = REPORT_RATE_720HZ;

ts_send_cmd:
	if (mode == core_data->report_rate_mode) {
		ts_debug("value is same, so not to write");
		return 0;
	}

	core_data->report_rate_mode = mode;
	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return 0;
	}

	ret = goodix_ts_send_cmd(core_data, INTERPOLATION_SWITCH_CMD, 5,
						core_data->report_rate_mode, 0x00);
	if (ret < 0) {
		ts_err("failed to set report rate, mode = %d", mode);
		return -EINVAL;
	}

	msleep(20);
	ts_info("Success to set %s\n", mode == 0x00 ? "Default" :
				(mode == 0x01 ? "REPORT_RATE_720HZ" :
				(mode == 0x02 ? "REPORT_RATE_576HZ" :
				"REPORT_RATE_480HZ")));

	return ret;
}

static ssize_t goodix_ts_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	unsigned long mode = 0;
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

	core_data->interpolation = mode;
	ret = goodix_ts_mmi_set_report_rate(core_data);
	if (ret)
		return ret;
	else
		return size;
}

static ssize_t goodix_ts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("interpolation = %d.\n", core_data->interpolation);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->interpolation);
}

static int goodix_ts_mmi_refresh_rate(struct device *dev, int freq)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	core_data->refresh_rate = freq;

	if (core_data->board_data.interpolation_ctrl)
		goodix_ts_mmi_set_report_rate(core_data);

	return 0;
}

/*
 * HAL: args[0] suppression area, args[1] rotation direction.
 * CMD: [06 17 data0 data1],
 *      data[0] rotation direction, data[1] suppression area.
 */
static ssize_t goodix_ts_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	int edge_cmd[2] = { 0 };
	unsigned int args[2] = { 0 };
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	ret = sscanf(buf, "%d %d", &args[0], &args[1]);
	if (ret < 2)
		return -EINVAL;

	if (DEFAULT_MODE == args[0]) {
		edge_cmd[1] = DEFAULT_EDGE;
	} else if (SMALL_MODE == args[0]) {
		edge_cmd[1] = SMALL_EDGE;
	} else if (BIG_MODE == args[0]) {
		edge_cmd[1] = BIG_EDGE;
	} else {
		ts_err("Invalid edge mode: %d!\n", args[0]);
		return -EINVAL;
	}

	if (ROTATE_0 == args[1]) {
		edge_cmd[0] = ROTATE_DEFAULT_0;
	} else if (ROTATE_90 == args[1]) {
		edge_cmd[0] = ROTATE_RIGHT_90;
	} else if (ROTATE_270 == args[1]) {
		edge_cmd[0] = ROTATE_LEFT_90;
	} else {
		ts_err("Invalid rotation mode: %d!\n", args[1]);
		return -EINVAL;
	}

	if (!memcmp(core_data->edge_mode, edge_cmd, sizeof(edge_cmd))) {
		pr_debug("value is same,so not write.\n");
		return size;
	}

	if (core_data->power_on == 0) {
		memcpy(core_data->edge_mode, edge_cmd, sizeof(edge_cmd));
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return size;
	}

	ret = goodix_ts_send_cmd(core_data, EDGE_SWITCH_CMD, 6, edge_cmd[0], edge_cmd[1]);
	if (ret < 0) {
		ts_err("failed to send edge switch cmd");
		return -EINVAL;
	}

	msleep(20);
	memcpy(core_data->edge_mode, edge_cmd, sizeof(edge_cmd));
	ts_info("Success to set edge = %02x, rotation = %02x", edge_cmd[1], edge_cmd[0]);
	return size;
}

static ssize_t goodix_ts_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("edge area = %02x, rotation = %02x\n",
		core_data->edge_mode[1], core_data->edge_mode[0]);
	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x",
		core_data->edge_mode[1], core_data->edge_mode[0]);
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
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	goodix_ts_send_cmd(core_data, CHARGER_MODE_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("Failed to set charger mode\n");
	}
	msleep(16);
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
		msleep(16);
		enable_irq_wake(core_data->irq);
		core_data->gesture_enabled = true;
		break;
	case TS_MMI_PM_DEEPSLEEP:
		/* enter sleep mode or power off */
		/* if (hw_ops->suspend)
		       hw_ops->suspend(core_data);
		 */
		core_data->gesture_enabled = false;
		break;
	case TS_MMI_PM_ACTIVE:
		if (hw_ops->resume)
			hw_ops->resume(core_data);
		core_data->gesture_enabled = false;
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
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	/* restore data */
	if (core_data->board_data.stylus_mode_ctrl && core_data->stylus_mode) {
		ret = goodix_stylus_mode(core_data, core_data->stylus_mode);
		if (!ret)
			ts_info("Success to %s stylus mode", core_data->stylus_mode ? "Enable" : "Disable");
	}

	if (core_data->board_data.leather_mode_ctrl && core_data->leather_mode) {
		ret = goodix_ts_send_cmd(core_data, LEATHER_MODE_SWITCH_CMD, 5,
						core_data->leather_mode, 0x00);
		if (!ret) {
			ts_info("Success to %s leather mode", core_data->leather_mode ? "Enable" : "Disable");
			msleep(20);
		}
	}

	if (core_data->board_data.film_mode_ctrl && core_data->film_mode) {
		ret = goodix_ts_send_cmd(core_data, FILM_MODE_SWITCH_CMD, 5,
						core_data->film_mode, 0x00);
		if (!ret) {
			ts_info("Success to %s film mode", core_data->film_mode ? "Enable" : "Disable");
			msleep(20);
		}
	}

	if (core_data->board_data.interpolation_ctrl && core_data->interpolation) {
		ret = goodix_ts_send_cmd(core_data, INTERPOLATION_SWITCH_CMD, 5,
						core_data->report_rate_mode, 0x00);
		if (!ret) {
			ts_info("Success to %s interpolation mode\n",
					core_data->report_rate_mode == 0x00 ? "Default" :
					(core_data->report_rate_mode == 0x01 ? "REPORT_RATE_720HZ" :
					(core_data->report_rate_mode == 0x02 ? "REPORT_RATE_576HZ" :
					"REPORT_RATE_480HZ")));
			msleep(20);
		}
	}

	if (core_data->board_data.edge_ctrl) {
		ret = goodix_ts_send_cmd(core_data, EDGE_SWITCH_CMD, 6,
						core_data->edge_mode[0], core_data->edge_mode[1]);
		if (!ret)
			ts_info("Success to set edge area = %02x, rotation = %02x",
				core_data->edge_mode[1], core_data->edge_mode[0]);
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
	int ret = 0;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	if (core_data->board_data.stylus_mode_ctrl && core_data->stylus_mode) {
		ret = goodix_stylus_mode(core_data, 0);
		if (!ret)
			ts_info("Success to exit stylus mode");
	}

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
	.refresh_rate = goodix_ts_mmi_refresh_rate,
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
