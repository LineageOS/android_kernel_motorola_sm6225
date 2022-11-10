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
#include "goodix_ts_config.h"

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
static ssize_t goodix_ts_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t goodix_ts_sensitivity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
#ifdef CONFIG_GTP_LAST_TIME
static ssize_t goodix_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf);
#endif

static DEVICE_ATTR(edge, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_edge_show, goodix_ts_edge_store);
static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_interpolation_show, goodix_ts_interpolation_store);
static DEVICE_ATTR(sample, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_sample_show, goodix_ts_sample_store);
static DEVICE_ATTR(stylus_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
	goodix_ts_stylus_mode_show, goodix_ts_stylus_mode_store);
static DEVICE_ATTR(sensitivity, (S_IRUGO | S_IWUSR | S_IWGRP),
	NULL, goodix_ts_sensitivity_store);
#ifdef CONFIG_GTP_LAST_TIME
static DEVICE_ATTR(timestamp, S_IRUGO, goodix_ts_timestamp_show, NULL);
#endif

/* hal settings */
#define ROTATE_0   0
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define BIG_MODE   1
#define SMALL_MODE    2
#define DEFAULT_MODE   0
#define MAX_ATTRS_ENTRIES 10

#define NORMAL_DEFAULT_MODE 10
#define NORMAL_SMALL_MODE 11
#define NORMAL_BIG_MODE 12

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

	if (core_data->board_data.sample_ctrl)
		ADD_ATTR(sample);

	if (core_data->board_data.stylus_mode_ctrl)
		ADD_ATTR(stylus_mode);

	if (core_data->board_data.sensitivity_ctrl)
		ADD_ATTR(sensitivity);

#ifdef CONFIG_GTP_LAST_TIME
	ADD_ATTR(timestamp);
#endif

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
	int ret = 0;

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.film_mode = mode;
	if (core_data->set_mode.film_mode == mode) {
		ts_debug("The value = %d is same,so not write.\n", mode);
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		goto exit;
	}

	ret = goodix_ts_send_cmd(core_data, FILM_MODE_SWITCH_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("failed to send leather mode cmd");
		goto exit;
	}

	core_data->set_mode.film_mode = mode;
	msleep(20);
	ts_info("Success to %s film mode", mode ? "Enable" : "Disable");
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static int goodix_ts_leather_mode(struct goodix_ts_core *core_data, int mode)
{
	int ret = 0;

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.leather_mode = mode;
	if (core_data->set_mode.leather_mode == mode) {
		ts_debug("The value = %d is same,so not write.\n", mode);
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		goto exit;
	}

	ret = goodix_ts_send_cmd(core_data, LEATHER_MODE_SWITCH_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("failed to send leather mode cmd");
		goto exit;
	}
	core_data->set_mode.leather_mode = mode;
	msleep(20);
	ts_info("Success to %s leather mode", mode ? "Enable" : "Disable");
exit:
	mutex_unlock(&core_data->mode_lock);
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
		msleep(50);
		ret = goodix_ts_send_cmd(core_data, STYLUS_MODE_SWITCH_CMD, 5, mode, 0x00);
		if (ret < 0) {
			ts_err("Failed to Disable stylus mode\n");
			return ret;
		}
		msleep(20);
	} else {
		ret = goodix_ts_send_cmd(core_data, STYLUS_MODE_SWITCH_CMD, 5, mode, 0x00);
		if (ret < 0) {
			ts_err("Failed to Disable stylus mode\n");
			return ret;
		}
		msleep(50);
		goodix_clock_enable(core_data, mode);
	}

	ts_info("Success to %s stylus mode", mode ? "Enable" : "Disable");
	return ret;
}

static ssize_t goodix_ts_stylus_mode_store(struct device *dev,
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

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.stylus_mode = mode;
	if (core_data->set_mode.stylus_mode == mode) {
		ts_debug("The value = %lu is same,so not write.\n", mode);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		ret = size;
		goto exit;
	}

	ret = goodix_stylus_mode(core_data, mode);
	if (!ret)
		core_data->set_mode.stylus_mode = mode;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_stylus_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("Stylus mode = %d.\n", core_data->set_mode.stylus_mode);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.stylus_mode);
}

static int goodix_ts_mmi_set_report_rate(struct goodix_ts_core *core_data)
{
	int ret = 0;
	int mode = 0;

	mode = goodix_ts_mmi_get_report_rate(core_data);
	if (mode == -1) {
		return -EINVAL;
	}

	core_data->get_mode.report_rate_mode = mode;
	if (core_data->set_mode.report_rate_mode == mode) {
		ts_debug("The value = %d is same, so not to write", mode);
		return 0;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		return 0;
	}

	//if now on high report rate and need switch to low report rate
	if ((((core_data->set_mode.report_rate_mode >> 8) & 0xFF) == REPORT_RATE_CMD_HIGH) &&
		(((mode >> 8) & 0xFF) == REPORT_RATE_CMD_LOW)) {
		ts_info("exit high report rate");
		ret = goodix_ts_send_cmd(core_data, EXIT_HIGH_REPORT_RATE_CMD >> 8, 5,
							EXIT_HIGH_REPORT_RATE_CMD & 0xFF, 0x00);
		if (ret < 0) {
			ts_err("failed to exit high report rate");
			return -EINVAL;
		}
		msleep(20);
	}

	//send switch command
	ret = goodix_ts_send_cmd(core_data, mode >> 8, 5,
						mode & 0xFF, 0x00);
	if (ret < 0) {
		ts_err("failed to set report rate, mode = %d", mode);
		return -EINVAL;
	}
	msleep(20);

	core_data->set_mode.report_rate_mode = mode;

	ts_info("Success to set %s\n", mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
				(mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_360HZ" :
				(mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
				(mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
				(mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
				"Unsupported")))));

	return ret;
}

static ssize_t goodix_ts_interpolation_store(struct device *dev,
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

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.interpolation = mode;
	ret = goodix_ts_mmi_set_report_rate(core_data);
	if (ret < 0)
		goto exit;

	ret = size;
	core_data->set_mode.interpolation = mode;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("interpolation = %d.\n", core_data->set_mode.interpolation);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.interpolation);
}

static ssize_t goodix_ts_sample_store(struct device *dev,
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

	mutex_lock(&core_data->mode_lock);
	core_data->get_mode.sample= mode;
	if (core_data->set_mode.sample == mode) {
		ts_debug("The value = %lu is same, so not to write", mode);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		ret = size;
		goto exit;
	}

	ret = goodix_ts_send_cmd(core_data, SAMPLE_SWITCH_CMD, 5,
						core_data->get_mode.sample, 0x00);
	if (ret < 0) {
		ts_err("failed to set sample rate, mode = %lu", mode);
		goto exit;
	}

	core_data->set_mode.sample = mode;
	msleep(20);
	ts_info("Success to set %lu\n", mode);

	ret = size;
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("sample = %d.\n", core_data->set_mode.sample);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", core_data->set_mode.sample);
}

static int goodix_ts_mmi_refresh_rate(struct device *dev, int freq)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	mutex_lock(&core_data->mode_lock);
	core_data->refresh_rate = freq;

	if (core_data->board_data.interpolation_ctrl)
		goodix_ts_mmi_set_report_rate(core_data);
	mutex_unlock(&core_data->mode_lock);
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

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ret = sscanf(buf, "%d %d", &args[0], &args[1]);
	if (ret < 2)
		return -EINVAL;

	switch (args[0]) {
	case DEFAULT_MODE:
		edge_cmd[1] = DEFAULT_EDGE;
		break;
	case SMALL_MODE:
		edge_cmd[1] = SMALL_EDGE;
		break;
	case BIG_MODE:
		edge_cmd[1] = BIG_EDGE;
		break;
	case NORMAL_DEFAULT_MODE:
		edge_cmd[1] = NORMAL_DEFAULT_EDGE;
		break;
	case NORMAL_SMALL_MODE:
		edge_cmd[1] = NORMAL_SMALL_EDGE;
		break;
	case NORMAL_BIG_MODE:
		edge_cmd[1] = NORMAL_BIG_EDGE;
		break;
	default:
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

	mutex_lock(&core_data->mode_lock);
	memcpy(core_data->get_mode.edge_mode, edge_cmd, sizeof(edge_cmd));
	if (!memcmp(core_data->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd))) {
		ts_debug("The value (%02x %02x) is same,so not write.\n",
					edge_cmd[0], edge_cmd[1]);
		ret = size;
		goto exit;
	}

	if (core_data->power_on == 0) {
		ts_debug("The touch is in sleep state, restore the value when resume\n");
		ret = size;
		goto exit;
	}

	ret = goodix_ts_send_cmd(core_data, EDGE_SWITCH_CMD, 6, edge_cmd[0], edge_cmd[1]);
	if (ret < 0) {
		ts_err("failed to send edge switch cmd");
		goto exit;
	}

	memcpy(core_data->set_mode.edge_mode, edge_cmd, sizeof(edge_cmd));
	msleep(20);
	ret = size;
	ts_info("Success to set edge = %02x, rotation = %02x", edge_cmd[1], edge_cmd[0]);
exit:
	mutex_unlock(&core_data->mode_lock);
	return ret;
}

static ssize_t goodix_ts_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	ts_info("edge area = %02x, rotation = %02x\n",
		core_data->set_mode.edge_mode[1], core_data->set_mode.edge_mode[0]);
	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x",
		core_data->set_mode.edge_mode[1], core_data->set_mode.edge_mode[0]);
}

#ifdef CONFIG_GTP_LAST_TIME
static ssize_t goodix_ts_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	ktime_t last_ktime;
	struct timespec64 last_ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_GOODIX_DATA(dev);

	mutex_lock(&core_data->mode_lock);
	last_ktime = core_data->last_event_time;
	core_data->last_event_time = 0;
	mutex_unlock(&core_data->mode_lock);

	last_ts = ktime_to_timespec64(last_ktime);

	return scnprintf(buf, PAGE_SIZE, "%lld.%ld\n", last_ts.tv_sec, last_ts.tv_nsec);
}
#endif

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
	int timeout = 50;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	/* 5000ms timeout */
	while (core_data->init_stage < CORE_INIT_STAGE2 && timeout--)
		msleep(100);

	mutex_lock(&core_data->mode_lock);
	ret = goodix_ts_send_cmd(core_data, CHARGER_MODE_CMD, 5, mode, 0x00);
	if (ret < 0) {
		ts_err("Failed to set charger mode\n");
	}
	msleep(20);
	ts_err("Success to %s charger mode\n", mode ? "Enable" : "Disable");
	mutex_unlock(&core_data->mode_lock);

	return 0;
}

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;
	const struct goodix_ts_hw_ops *hw_ops;
	int ret = 0;
#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
	unsigned short gesture_cmd = 0xFFFF;
	unsigned char gesture_type = 0;
#endif

	GET_GOODIX_DATA(dev);
	hw_ops = core_data->hw_ops;

	switch (to) {
	case TS_MMI_PM_GESTURE:
		hw_ops->irq_enable(core_data, false);
		if (hw_ops->gesture)
#if defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
		if (core_data->imports && core_data->imports->get_gesture_type) {
			ret = core_data->imports->get_gesture_type(core_data->bus->dev, &gesture_type);
		}
		if (gesture_type & TS_MMI_GESTURE_ZERO) {
			gesture_cmd &= ~(1 << 5);
			ts_info("enable zero gesture mode cmd 0x%04x\n", gesture_cmd);
		}
		if (gesture_type & TS_MMI_GESTURE_SINGLE) {
			gesture_cmd &= ~(1 << 4);
			ts_info("enable single gesture mode cmd 0x%04x\n", gesture_cmd);
		}
		if (gesture_type & TS_MMI_GESTURE_DOUBLE) {
			gesture_cmd &= ~(1 << 15);
			ts_info("enable double gesture mode cmd 0x%04x\n", gesture_cmd);
		}

		ret = goodix_ts_send_cmd(core_data, ENTER_GESTURE_MODE_CMD, 6, 0xFF, 0xFF);
		if (ret < 0) {
			ts_err("Failed to send enter gesture mode\n");
		}
		ret = goodix_ts_send_cmd(core_data, ENTER_GESTURE_MODE_CMD, 6, gesture_cmd >> 8,
			gesture_cmd & 0xFF);
		if (ret < 0) {
			ts_err("Failed to send enable gesture mode\n");
		}
		ts_info("Send enable gesture mode 0x%04x, 0x%02x\n", gesture_cmd, gesture_type);
#else
#if defined(PRODUCT_MIAMI)
			hw_ops->gesture(core_data, 0x80);
#else
			hw_ops->gesture(core_data, 0);
#endif
#endif
		msleep(16);
		hw_ops->irq_enable(core_data, true);
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
		if (core_data->gesture_enabled) {
			core_data->gesture_enabled = false;
			hw_ops->irq_enable(core_data, true);
		}
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

	ts_info("Resume start");
	GET_GOODIX_DATA(dev);

	atomic_set(&core_data->suspended, 0);
	if (core_data->gesture_enabled) {
		core_data->hw_ops->irq_enable(core_data, false);
		disable_irq_wake(core_data->irq);
	}

	return 0;
}

static int goodix_ts_mmi_post_resume(struct device *dev) {
	int ret = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	/* open esd */
	goodix_ts_blocking_notify(NOTIFY_RESUME, NULL);

	mutex_lock(&core_data->mode_lock);
	/* All IC status are cleared after reset */
	memset(&core_data->set_mode, 0 , sizeof(core_data->set_mode));
	/* restore data */
	if (core_data->board_data.stylus_mode_ctrl && core_data->get_mode.stylus_mode) {
		ret = goodix_stylus_mode(core_data, core_data->get_mode.stylus_mode);
		if (!ret) {
			core_data->set_mode.stylus_mode = core_data->get_mode.stylus_mode;
			ts_info("Success to %s stylus mode", core_data->get_mode.stylus_mode ? "Enable" : "Disable");
		}
	}

	if (core_data->board_data.interpolation_ctrl && core_data->get_mode.interpolation) {
		//send switch command
		ret = goodix_ts_send_cmd(core_data, (core_data->get_mode.report_rate_mode) >> 8, 5,
						(core_data->get_mode.report_rate_mode) & 0xFF, 0x00);
		if (!ret) {
			core_data->set_mode.interpolation = core_data->get_mode.interpolation;
			core_data->set_mode.report_rate_mode = core_data->get_mode.report_rate_mode;
			msleep(20);

			ts_info("Success to %s interpolation mode\n",
				core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_240HZ ? "REPORT_RATE_240HZ" :
				(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_360HZ ? "REPORT_RATE_360HZ" :
				(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_480HZ ? "REPORT_RATE_480HZ" :
				(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_576HZ ? "REPORT_RATE_576HZ" :
				(core_data->get_mode.report_rate_mode == REPORT_RATE_CMD_720HZ ? "REPORT_RATE_720HZ" :
				"Unsupported")))));
		}
	}

	if (core_data->board_data.sample_ctrl && core_data->get_mode.sample) {
		ret = goodix_ts_send_cmd(core_data, SAMPLE_SWITCH_CMD, 5,
						core_data->get_mode.sample, 0x00);
		if (!ret) {
			core_data->set_mode.sample = core_data->get_mode.sample;
			msleep(20);
			ts_info("Success to %d sample mode\n", core_data->get_mode.sample);
		}
	}

	if (core_data->board_data.leather_mode_ctrl && core_data->get_mode.leather_mode) {
		ret = goodix_ts_send_cmd(core_data, LEATHER_MODE_SWITCH_CMD, 5,
						core_data->get_mode.leather_mode, 0x00);
		if (!ret) {
			core_data->set_mode.leather_mode = core_data->get_mode.leather_mode;
			msleep(20);
			ts_info("Success to %s leather mode", core_data->get_mode.leather_mode ? "Enable" : "Disable");
		}
	}

	if (core_data->board_data.film_mode_ctrl && core_data->get_mode.film_mode) {
		ret = goodix_ts_send_cmd(core_data, FILM_MODE_SWITCH_CMD, 5,
						core_data->get_mode.film_mode, 0x00);
		if (!ret) {
			core_data->set_mode.film_mode = core_data->get_mode.film_mode;
			msleep(20);
			ts_info("Success to %s film mode", core_data->get_mode.film_mode ? "Enable" : "Disable");
		}
	}

	if (core_data->board_data.edge_ctrl) {
		ret = goodix_ts_send_cmd(core_data, EDGE_SWITCH_CMD, 6,
						core_data->get_mode.edge_mode[0], core_data->get_mode.edge_mode[1]);
		if (!ret) {
			memcpy(core_data->set_mode.edge_mode, core_data->get_mode.edge_mode,
					sizeof(core_data->get_mode.edge_mode));
			msleep(20);
			ts_info("Success to set edge area = %02x, rotation = %02x",
				core_data->get_mode.edge_mode[1], core_data->get_mode.edge_mode[0]);
		}
	}
	mutex_unlock(&core_data->mode_lock);
#ifdef CONFIG_GTP_FOD
	if(core_data->zerotap_data[0]) {
		ts_info("FOD is down during PM resume  fod_enable=%d",core_data->fod_enable);
	}
	core_data->zerotap_data[0] = 0;
#endif
	return 0;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev) {
	int ret = 0;
	struct platform_device *pdev;
	struct goodix_ts_core *core_data;

	GET_GOODIX_DATA(dev);

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);

	if (core_data->board_data.stylus_mode_ctrl && core_data->set_mode.stylus_mode) {
		mutex_lock(&core_data->mode_lock);
		ret = goodix_stylus_mode(core_data, 0x00);
		if (!ret) {
			ts_info("Success to exit stylus mode");
			core_data->set_mode.stylus_mode = 0x00;
		}
		mutex_unlock(&core_data->mode_lock);
	}

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_blocking_notify(NOTIFY_SUSPEND, NULL);

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
#ifdef CONFIG_GTP_FOD
static int goodix_ts_mmi_update_fps_mode(struct device *dev, int mode) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	core_data->fod_enable = (mode >0) ? 0x01 : 0x00;
	ts_info("  update_fps_mode %s:%d\n", (mode > 0) ? "enable" : "disable", mode);
	return 0;
}
#endif

#define Y_MIN_ID 2
#define Y_MAX_ID 3
#define SCREEN_X_MAX 1080
#define SCREEN_Y_MAX 2992
#define SCREEN_EXTENDED 2600
#define SCREEN_PRIM_COMPACT 1980
#define DISP_MODE_REAR 3
#define DISP_MODE_FULL 4
#define DISP_MODE_EXTENDED 0
#define DISP_MODE_PRIM_COMPACT 1
#define DISP_MODE_PRIM_PEEK 2

static int rdArray[4];

static int goodix_ts_mmi_active_region(struct device *dev, int *reg_data)
{
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;
	int ret, dmode = -1;

	GET_GOODIX_DATA(dev);

	memcpy(rdArray, reg_data, sizeof(rdArray));
	if (rdArray[Y_MIN_ID] > 0) { /* REAR */
		dmode = DISP_MODE_REAR; /* Mode4 */
	} else { /* FULL or PRIMARY */
		if (rdArray[Y_MAX_ID] == SCREEN_Y_MAX)
			dmode = DISP_MODE_FULL; /* Mode5 */
		else if (rdArray[Y_MAX_ID] > SCREEN_EXTENDED)
			dmode = DISP_MODE_EXTENDED; /* Mode1 */
		else if (rdArray[Y_MAX_ID] > SCREEN_PRIM_COMPACT)
			dmode = DISP_MODE_PRIM_COMPACT; /* Mode2 */
		else
			dmode = DISP_MODE_PRIM_PEEK; /* Mode3 */
	}
	if (dmode >= 0 ) {
		ret = core_data->hw_ops->display_mode(core_data, dmode);
		if (!ret)
			ts_info("set active region: %d %d %d %d; dmode=%d\n",
				rdArray[0], rdArray[1], rdArray[Y_MIN_ID], rdArray[Y_MAX_ID], dmode);
	} else
		dev_err(&pdev->dev, "Invalid display mode; reqion %d %d %d %d\n",
			rdArray[0], rdArray[1], rdArray[Y_MIN_ID], rdArray[Y_MAX_ID]);

	return 0;
}

static int goodix_ts_mmi_methods_get_active_region(struct device *dev, void *uidata)
{
	struct goodix_ts_core *core_data;
	struct platform_device *pdev;

	GET_GOODIX_DATA(dev);

	memcpy((int *)uidata, rdArray, sizeof(rdArray));
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
	.get_active_region = goodix_ts_mmi_methods_get_active_region,
	/* SET methods */
	.reset =  goodix_ts_mmi_methods_reset,
	.drv_irq = goodix_ts_mmi_methods_drv_irq,
	.power = goodix_ts_mmi_methods_power,
	.charger_mode = goodix_ts_mmi_charger_mode,
	.refresh_rate = goodix_ts_mmi_refresh_rate,
	.active_region = goodix_ts_mmi_active_region,
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
#ifdef CONFIG_GTP_FOD
	.update_fod_mode = goodix_ts_mmi_update_fps_mode,
#endif
};

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	mutex_init(&core_data->mode_lock);
	ret = ts_mmi_dev_register(core_data->bus->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		mutex_destroy(&core_data->mode_lock);
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
	mutex_destroy(&core_data->mode_lock);
	ts_mmi_dev_unregister(core_data->bus->dev);
}
