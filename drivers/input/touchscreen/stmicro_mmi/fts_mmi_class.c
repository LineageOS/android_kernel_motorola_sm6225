/*
* Copyright (C) 2020 Motorola Mobility LLC
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
#define pr_fmt(fmt) "[ FTS-MMI ] %s: " fmt, __func__

#include "fts.h"
#include "fts_mmi.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsTool.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsGesture.h"
#include <linux/regulator/consumer.h>

#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
int flash_mode; /* global variable */
#endif

extern void fts_resume_func(struct fts_ts_info *info);
extern void fts_suspend_func(struct fts_ts_info *info);
extern int fts_fw_update(struct fts_ts_info *info);

#define ASSERT_PTR(p) if (p == NULL) { \
	dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
	}

static ssize_t fts_mmi_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fts_mmi_linearity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_linearity_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fts_mmi_first_filter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_first_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fts_mmi_jitter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_jitter_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fts_mmi_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(calibrate, (S_IWUSR | S_IWGRP), NULL, fts_mmi_calibrate_store);
static DEVICE_ATTR(interpolation, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_interpolation_show, fts_mmi_interpolation_store);
static DEVICE_ATTR(linearity, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_linearity_show, fts_mmi_linearity_store);
static DEVICE_ATTR(first_filter, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_first_filter_show, fts_mmi_first_filter_store);
static DEVICE_ATTR(jitter, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_jitter_show, fts_mmi_jitter_store);
static DEVICE_ATTR(edge, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_edge_show, fts_mmi_edge_store);

#define MAX_ATTRS_ENTRIES 10
#define ROTATE_90   1
#define ROTATE_180   2
#define ROTATE_270  3
#define BIG_MODE   1
#define DEFAULT_MODE   0
#define SMALL_EDGE   0
#define BIG_EDGE   1
#define CLOSE_EDGE   2
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


static int fts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	int idx = 0;

	ASSERT_PTR(ts);
	if (ts->board->interpolation_ctrl)
		ADD_ATTR(interpolation);

	if (ts->board->jitter_ctrl)
		ADD_ATTR(jitter);

	if (ts->board->linearity_ctrl)
		ADD_ATTR(linearity);

	if (ts->board->first_filter_ctrl)
		ADD_ATTR(first_filter);

	if (ts->board->edge_ctrl)
		ADD_ATTR(edge);

	if (strncmp(mmi_bl_bootmode(), "mot-factory", strlen("mot-factory")) == 0) {
		ADD_ATTR(calibrate);
	}

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}
/*
 * To enable/disable interpolation algorithm to modify report rate.
 * mode = 0x00, Disable interpolation algorithm
 * mode = 0x01, Enable interpolation algorithm
 * ...
 */
static int fts_mmi_set_interpolation(struct fts_ts_info *ts, unsigned int mode)
{
	int ret = 0;

	if (mode == 0x00 || mode == 0x01) {
		ts->board->interpolation_cmd[2] = mode;
		ret = fts_write(ts->board->interpolation_cmd, ARRAY_SIZE(ts->board->interpolation_cmd));
	} else {
		pr_info("The interpolation mode=%d is valid.\n", mode);
		return -EINVAL;
	}

	if (ret == OK) {
		ts->interpolation_val = mode;
		pr_info("Successfully to set inpolation = %d!\n", ts->interpolation_val);
	} else
		pr_info("Failed to set inpolation = %d!\n", ts->interpolation_val);

	return ret;
}


static ssize_t fts_mmi_interpolation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct fts_ts_info *ts;
	int ret = 0;
	unsigned long mode = 0;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	ret = kstrtoul(buf, 0, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	if (ts->interpolation_val == mode) {
		pr_debug("value is same,so not write.\n");
		return size;
	}

	ret = fts_mmi_set_interpolation(ts, mode);

	return size;
}

static ssize_t fts_mmi_interpolation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	pr_info("Interpolation mode = %d.\n", ts->interpolation_val);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", ts->interpolation_val);
}

static ssize_t fts_mmi_jitter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct fts_ts_info *ts;
	struct fts_hw_platform_data *board;
	int ret = 0;
	unsigned int args[3] = { 0 };

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	board = ts->board;

	ret = sscanf(buf, "0x%x 0x%x 0x%x", &args[0], &args[1], &args[2]);
	if (ret < 3)
		return -EINVAL;

	/* tap x jitter */
	board->jitter_cmd[2] = (args[0] >> 8) & 0xff;
	/* tap y jitter */
	board->jitter_cmd[3] = args[0] & 0xff;
	/* slow slidding x jitter */
	board->jitter_cmd[4] = (args[1] >> 8) & 0xff;
	/* slow slidding y jitter */
	board->jitter_cmd[5] = args[1] & 0xff;
	/* fast slidding y jitter */
	board->jitter_cmd[6] = (args[2] >> 8) & 0xff;
	/* fsst slidding y jitter */
	board->jitter_cmd[7] = args[2] & 0xff;

	if (!memcmp(board->jitter_cmd, ts->jitter_val, sizeof(ts->jitter_val))) {
		pr_debug("value is same,so not write.\n");
		return size;
	}

	memcpy(ts->jitter_val, board->jitter_cmd, sizeof(ts->jitter_val));
	ret = fts_write(board->jitter_cmd, ARRAY_SIZE(board->jitter_cmd));
	if (ret == OK) {
		pr_info("Successfully to set jitter mode:\n");
		pr_info("tap: %02x %02x, slow: %02x %02x, fast: %02x %02x!\n",
			ts->jitter_val[2], ts->jitter_val[3], ts->jitter_val[4],
			ts->jitter_val[5], ts->jitter_val[6], ts->jitter_val[7]);
	} else {
		pr_info("Failed to set jitter mode:\n");
		pr_info("tap: %02x %02x, slow: %02x %02x, fast: %02x %02x!\n",
			ts->jitter_val[2], ts->jitter_val[3], ts->jitter_val[4],
			ts->jitter_val[5], ts->jitter_val[6], ts->jitter_val[7]);
	}

	return size;
}

static ssize_t fts_mmi_jitter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	return scnprintf(buf, PAGE_SIZE, "0x%04x 0x%04x 0x%04x",
		(ts->jitter_val[2] << 8 | ts->jitter_val[3]),
		(ts->jitter_val[4] << 8 | ts->jitter_val[5]),
		(ts->jitter_val[6] << 8 | ts->jitter_val[7]));
}

static ssize_t fts_mmi_linearity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct fts_ts_info *ts;
	struct fts_hw_platform_data *board;
	int ret = 0;
	unsigned long mode = 0;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	board = ts->board;

	ret = kstrtoul(buf, 16, &mode);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	board->linearity_cmd[2] = mode;
	if (!memcmp(board->linearity_cmd, ts->linearity_val, sizeof(ts->linearity_val))) {
		pr_debug("value is same,so not write.\n");
		return size;
	}

	memcpy(ts->linearity_val, board->linearity_cmd, sizeof(ts->linearity_val));
	ret = fts_write(board->linearity_cmd, ARRAY_SIZE(board->linearity_cmd));
	if (ret == OK)
		pr_info("Successfully to %s linearity mode!\n", (!!mode ? "Enable" : "Disable"));
	else
		pr_info("Failed to %s linearity mode!\n", (!!mode ? "Enable" : "Disable"));

	return size;
}

static ssize_t fts_mmi_linearity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	pr_info("Linearity mode is %s!\n", (!!ts->linearity_val[2] ? "Enable" : "Disable"));
	return scnprintf(buf, PAGE_SIZE, "0x%02x", ts->linearity_val[2]);
};

static ssize_t fts_mmi_first_filter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct fts_ts_info *ts;
	struct fts_hw_platform_data *board;
	int ret = 0;
	unsigned int args[2] = { 0 };

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	board = ts->board;

	ret = sscanf(buf, "0x%x 0x%x", &args[0], &args[1]);
	if (ret < 2)
		return -EINVAL;

	board->first_filter_cmd[2] = args[0];
	board->first_filter_cmd[3] = args[1];

	if (!memcmp(board->first_filter_cmd, ts->first_filter_val, sizeof(ts->first_filter_val))) {
		pr_debug("value is same,so not write.\n");
		return size;
	}

	memcpy(ts->first_filter_val, board->first_filter_cmd, sizeof(ts->first_filter_val));
	ret = fts_write(board->first_filter_cmd, ARRAY_SIZE(board->first_filter_cmd));
	if (ret == OK)
		pr_info("Successfully to set first_filter mode: %02x %02x!\n",
			ts->first_filter_val[2], ts->first_filter_val[3]);
	else
		pr_info("Failed to set first_filter mode: %02x %02x!\n",
			ts->first_filter_val[2], ts->first_filter_val[3]);

	return size;
}

static ssize_t fts_mmi_first_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	pr_info("finger filter: 1st = %02x, 2nd = %02x.\n",
		ts->first_filter_val[2], ts->first_filter_val[3]);
	return scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x",
		ts->first_filter_val[2], ts->first_filter_val[3]);
}

static ssize_t fts_mmi_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct fts_ts_info *ts;
	struct fts_hw_platform_data *board;
	int ret = 0;
	unsigned int args[2] = { 0 };

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	board = ts->board;

	ret = sscanf(buf, "%d %d", &args[0], &args[1]);
	if (ret < 2)
		return -EINVAL;
	/* UI will not update when rotato to 180 degree.*/
	if(ROTATE_180 == args[1])
		return size;
	/* here need set cmd 02 to fw when 270 degree.*/
	if(ROTATE_270 == args[1])
		args[1]--;

	board->edge_cmd[2] = CLOSE_EDGE * 3 +args[1];

	if(BIG_MODE == args[0]) {
		board->edge_cmd[2] = BIG_EDGE* 3 +args[1];
	}

	if (!memcmp(board->edge_cmd, ts->edge_val, sizeof(ts->edge_val))) {
		pr_debug("value is same,so not write.\n");
		return size;
	}

	memcpy(ts->edge_val, board->edge_cmd, sizeof(ts->edge_val));
	ret = fts_write(board->edge_cmd, ARRAY_SIZE(board->edge_cmd));
	if (ret == OK)
		pr_info("Successfully to set edge mode: %02x!\n", ts->edge_val[2]);
	else
		pr_info("Failed to set edge mode: %02x!\n", ts->edge_val[2]);

	return size;
}

static ssize_t fts_mmi_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	pr_info("edge = %02x\n", ts->board->edge_cmd[2]);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", ts->board->edge_cmd[2]);
}

static ssize_t fts_mmi_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	unsigned long value = 0;

	ret = kstrtoul(buf, 0, &value);
	if (ret < 0) {
		pr_info("Failed to convert value.\n");
		return -EINVAL;
	}

	if (value == 3) {
		u8 cmd[3] = { FTS_CMD_SYSTEM, SYS_CMD_SPECIAL, SPECIAL_FULL_PANEL_INIT };
		pr_info("Sending Event [0x%02x 0x%02x 0x%02x]...\n",
				cmd[0], cmd[1], cmd[2]);

		ret = fts_writeFwCmd(cmd, 3);
		if (ret != OK) {
			pr_info("TP Calibration Failed!\n");
			return -EINVAL;
		}
		pr_info("TP Calibration Success!\n");
	} else {
		pr_info("The value is invalid\n");
		return -EINVAL;
	}

	return size;
}

static int fts_mmi_get_class_entry_name(struct device *dev, void *cdata) {
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "primary");
}

static int fts_mmi_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "stmicro");
}

static int fts_mmi_get_productinfo(struct device *dev, void *cdata) {
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	u8 readData[3];
	char buffer[64];
	ssize_t blen = 0;
	ASSERT_PTR(ts);
	fts_writeReadU8UX(FTS_CMD_HW_REG_R, ADDR_SIZE_HW_REG,
			ADDR_DCHIP_ID, readData, 2, DUMMY_FIFO);
	blen += scnprintf(buffer+blen, sizeof(buffer)-blen, "fts%02x%02x",
		readData[0], readData[1]);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", buffer);
}

static int fts_mmi_get_build_id(struct device *dev, void *cdata) {
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	char buffer[64];
	ssize_t blen = 0;
	ASSERT_PTR(ts);
	blen += scnprintf(buffer+blen, sizeof(buffer)-blen, "%04x%04x",
		ts->sysinfo->u16_fwVer, ts->sysinfo->u16_cfgVer);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%s", buffer);
}

static int fts_mmi_get_config_id(struct device *dev, void *cdata) {
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	char buffer[64];
	ssize_t blen = 0;
	ASSERT_PTR(ts);
	blen += scnprintf(buffer+blen, sizeof(buffer)-blen, "%02x%02x%02x%02x",
		ts->sysinfo->u8_releaseInfo[3], ts->sysinfo->u8_releaseInfo[2],
		ts->sysinfo->u8_releaseInfo[1], ts->sysinfo->u8_releaseInfo[0]);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%s", buffer);
}

static int fts_mmi_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int fts_mmi_get_irq_status(struct device *dev, void *idata) {
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	TO_INT(idata) = gpio_get_value(ts->board->irq_gpio);
	return 0;
}

static int fts_mmi_get_drv_irq(struct device *dev, void *idata) {
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	TO_INT(idata) = fts_is_InterruptEnabled();
	return 0;
}

static int fts_mmi_get_poweron(struct device *dev, void *idata)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	TO_INT(idata) = (regulator_is_enabled(ts->avdd_reg) &&
				regulator_is_enabled(ts->vdd_reg)) ? 1 : 0;
	return 0;
}

static int fts_mmi_get_flashprog(struct device *dev, void *idata)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	//TO_INT(idata) = ts->in_bootloader ? 1 : 0;

	return 0;
}

#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
static int fts_mmi_get_flash_mode(struct device *dev, void *idata)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	TO_INT(idata) = flash_mode;

	return 0;
}

static int fts_mmi_flash_mode(struct device *dev, int mode)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);

	flash_mode = mode;

	return 0;
}
#endif

static int fts_mmi_drv_irq(struct device *dev, int state)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	pr_info("enter %d\n", state);
	dev_dbg(dev, "%s\n", __func__);
	switch (state) {
	case 0: /* Disable irq */
		fts_disableInterrupt();
			break;
	case 1: /* Enable irq */
		fts_enableInterrupt();
			break;
	default:
		dev_err(dev, "%s: invalid value\n", __func__);
		return -EINVAL;
	}
	pr_info("IRQ is %s\n", fts_is_InterruptEnabled() ? "EN" : "DIS");
	return 0;
}

static int fts_mmi_reset(struct device *dev, int type)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	pr_info("enter\n");
	dev_dbg(dev, "%s\n", __func__);
	return fts_system_reset();
}

static int fts_mmi_panel_state(struct device *dev,
		enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	u8 mask[4] = { 0 };
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s: panel state change: %d->%d\n", __func__, from, to);
	pr_info("panel state change: %d->%d\n", from, to);
	switch (to) {
	case TS_MMI_PM_GESTURE:
		/* support single tap gesture */
		fromIDtoMask(GEST_ID_SIGTAP, mask, GESTURE_MASK_SIZE);
		updateGestureMask(mask, GESTURE_MASK_SIZE, 1);
		ts->gesture_enabled = 1;
	case TS_MMI_PM_DEEPSLEEP:
		fts_suspend_func(ts);
			break;
	case TS_MMI_PM_ACTIVE:
		ts->gesture_enabled = 0;
			break;
	default:
		dev_warn(dev, "invalid panel state %d\n", to);
		return -EINVAL;
	}
	pr_info("IRQ is %s\n", fts_is_InterruptEnabled() ? "EN" : "DIS");
	return 0;
}

static int fts_mmi_power(struct device *dev, int on)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	pr_info("enter %d\n", on);
	dev_dbg(dev, "%s\n", __func__);
	fts_chip_power_switch(ts, on == 1);
	pr_info("IRQ is %s\n", fts_is_InterruptEnabled() ? "EN" : "DIS");
	return 0;
}

static int fts_mmi_pinctrl(struct device *dev, int on)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	pr_info("enter %d\n", on);
	dev_dbg(dev, "%s\n", __func__);
	fts_pinctrl_state(ts, on == 1);
	pr_info("IRQ is %s\n", fts_is_InterruptEnabled() ? "EN" : "DIS");
	return 0;
}

static int fts_mmi_post_resume(struct device *dev)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	struct fts_hw_platform_data *board;
	int ret = 0;

	ASSERT_PTR(ts);
	board = ts->board;
	pr_info("enter\n");
	dev_dbg(dev, "%s\n", __func__);
	fts_resume_func(ts);
	pr_info("IRQ is %s\n", fts_is_InterruptEnabled() ? "EN" : "DIS");

	/* restore data */
	if (ts->board->interpolation_ctrl)
		fts_mmi_set_interpolation(ts, ts->interpolation_val);

	if (ts->board->jitter_ctrl) {
		ret = fts_write(board->jitter_cmd, ARRAY_SIZE(board->jitter_cmd));
		if (ret == OK)
			dev_dbg(dev, "%s: Successfully to restore jitter mode\n", __func__);
	}

	if (ts->board->linearity_ctrl) {
		ret = fts_write(board->linearity_cmd, ARRAY_SIZE(board->linearity_cmd));
		if (ret == OK)
			dev_dbg(dev, "%s: Successfully to restore linearity mode!\n", __func__);
	}

	if (ts->board->first_filter_ctrl) {
		ret = fts_write(board->first_filter_cmd, ARRAY_SIZE(board->first_filter_cmd));
		if (ret == OK)
			dev_dbg(dev, "%s: Successfully to restore first_filter mode!\n", __func__);
	}

	if (ts->board->report_rate_ctrl) {
		ret = fts_write(board->report_rate_cmd, ARRAY_SIZE(board->report_rate_cmd));
		if (ret == OK)
			dev_dbg(dev, "%s: Successfully to restore report rate mode!\n", __func__);
	}

	if (ts->board->edge_ctrl) {
		ret = fts_write(board->edge_cmd, ARRAY_SIZE(board->edge_cmd));
		if (ret == OK)
			dev_dbg(dev, "%s: Successfully to restore edge mode!\n", __func__);
	}

	return 0;
}

static int fts_mmi_fw_update(struct device *dev, char *fwname)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	int ret;
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);

	fts_disableInterrupt();
	fts_interrupt_uninstall(ts);

	ts->fw_file = fwname;
	ts->force_reflash = true;
	ret = fts_fw_update(ts);
	ts->fw_file = NULL;
	return ret;
}

static int fts_mmi_charger_mode(struct device *dev, int mode)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	int ret = OK;
	u8 settings[4] = { 0 };
	ASSERT_PTR(ts);

	if (mode == ts->charger_enabled) {
		logError(1, "%s %s: Charger mode is already %s.\n",
			tag, __func__, !!mode ? "Enable" : "Disable");
		return ret;
	}

	ts->mode = MODE_NOTHING;	/* initialize the mode to nothing in
					 * order to be updated depending on the
					 * features enabled */
	ts->charger_enabled = mode;
	settings[0] = ts->charger_enabled;
	ret = setFeatures(FEAT_SEL_CHARGER, settings, 1);
	if (ret < OK)
		logError(1, "%s %s: error during setting CHARGER_MODE! ERROR %08X\n",
			tag, __func__, ret);

	if (ret >= OK && ts->charger_enabled == FEAT_ENABLE) {
		fromIDtoMask(FEAT_SEL_CHARGER, (u8 *)&ts->mode, sizeof(ts->mode));
		logError(1, "%s %s: CHARGER_MODE Enabled!\n",
			tag, __func__);
	} else
		logError(1, "%s %s: CHARGER_MODE Disabled!\n", tag, __func__);

	return ret;
}

#define DSI_REFRESH_RATE_144			144
static int fts_mmi_refresh_rate(struct device *dev, int freq)
{
	struct fts_hw_platform_data *board;
	struct fts_ts_info *ts;

	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	board = ts->board;

	if (freq == DSI_REFRESH_RATE_144)
		board->report_rate_cmd[2] = 0x01;
	else
		board->report_rate_cmd[2] = 0x00;

	if (ts->report_rate == board->report_rate_cmd[2]) {
		dev_dbg(dev, "%s: value is same, so not write.\n", __func__);
		return 0;
	}

	ts->report_rate = board->report_rate_cmd[2];
	fts_write(board->report_rate_cmd, ARRAY_SIZE(board->report_rate_cmd));

	logError(1, "%s %s: Successfully to write refresh rate %dhz!\n", tag, __func__, freq);
	return 0;
}

static int fts_mmi_wait4ready(struct device *dev)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	fts_wait_for_ready();
	return 0;
}

static struct ts_mmi_methods fts_mmi_methods = {
	.get_vendor = fts_mmi_get_vendor,
	.get_productinfo = fts_mmi_get_productinfo,
	.get_build_id = fts_mmi_get_build_id,
	.get_config_id = fts_mmi_get_config_id,
	.get_bus_type = fts_mmi_get_bus_type,
	.get_irq_status = fts_mmi_get_irq_status,
	.get_drv_irq = fts_mmi_get_drv_irq,
	.get_poweron = fts_mmi_get_poweron,
	.get_flashprog = fts_mmi_get_flashprog,
#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
	.get_flash_mode = fts_mmi_get_flash_mode,
#endif
	.get_class_entry_name = fts_mmi_get_class_entry_name,
	/* SET methods */
	.reset =  fts_mmi_reset,
	.drv_irq = fts_mmi_drv_irq,
	.charger_mode = fts_mmi_charger_mode,
	.refresh_rate = fts_mmi_refresh_rate,
	.power = fts_mmi_power,
	.pinctrl = fts_mmi_pinctrl,
	.wait_for_ready = fts_mmi_wait4ready,
	/* Firmware */
#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
	.flash_mode = fts_mmi_flash_mode,
#endif
	.firmware_update = fts_mmi_fw_update,
	/* vendor specific attribute group */
	.extend_attribute_group = fts_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = fts_mmi_panel_state,
	.post_resume = fts_mmi_post_resume,
};

int fts_mmi_init(struct fts_ts_info *ts, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = ts_mmi_dev_register(ts->dev, &fts_mmi_methods);
		if (ret)
			dev_err(ts->dev, "Failed to register ts mmi\n");
		/* initialize class imported methods */
		ts->imports = &fts_mmi_methods.exports;

		dev_info(ts->dev, "MMI start sensing\n");
		fts_init_sensing(ts);
		fts_enableInterrupt();

		/* Disable interpolation algorithm */
		if (ts->board->interpolation_ctrl)
			ts->interpolation_val = 0x00;

	} else
		ts_mmi_dev_unregister(ts->dev);
	return ret;
}
