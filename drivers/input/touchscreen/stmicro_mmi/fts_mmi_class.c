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
#include <linux/regulator/consumer.h>

extern void fts_resume_func(struct fts_ts_info *info);
extern void fts_suspend_func(struct fts_ts_info *info);
extern int fts_fw_update(struct fts_ts_info *info);

#define ASSERT_PTR(p) if (p == NULL) { \
	dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
	}

static ssize_t fts_mmi_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_report_rate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fts_mmi_report_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(calibrate, (S_IWUSR | S_IWGRP), NULL, fts_mmi_calibrate_store);
static DEVICE_ATTR(report_rate, (S_IRUGO | S_IWUSR | S_IWGRP),
	fts_mmi_report_rate_show, fts_mmi_report_rate_store);

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


static int fts_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	int idx = 0;

	ASSERT_PTR(ts);
	if (ts->board->report_rate_ctrl)
		ADD_ATTR(report_rate);

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
/* Configure different reporting rates
 * mode = 0x00, Set report rate 240Hz
 * mode = 0x01, Set report rate 480Hz
 * ...
 */
static int fts_mmi_set_report_rate(struct fts_ts_info *ts, unsigned int mode)
{
	int ret = 0;

	if (mode == 0x00 || mode == 0x01) {
		ts->board->report_rate_cmd[2] = mode;
		ret = fts_write(ts->board->report_rate_cmd, ARRAY_SIZE(ts->board->report_rate_cmd));
			pr_info("len=%d ", ARRAY_SIZE(ts->board->report_rate_cmd));
	} else {
		pr_info("The report rate mode=%d is valid.\n", mode);
		return -EINVAL;
	}

	if (ret == OK) {
		ts->report_rate = mode;
		pr_info("Successfully to set report rate mode=%d!\n", ts->report_rate);
	} else
		pr_info("Failed to set report rate mode=%d!\n", ts->report_rate);

	return ret;
}


static ssize_t fts_mmi_report_rate_store(struct device *dev,
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

	if (ts->report_rate == mode) {
		pr_info("value is same,so not write.\n");
		return size;
	}

	ret = fts_mmi_set_report_rate(ts, mode);

	return size;
}

static ssize_t fts_mmi_report_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);

	pr_info("report_rate mode=%d.\n", ts->report_rate);
	return scnprintf(buf, PAGE_SIZE, "0x%02x", ts->report_rate);
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
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s: panel state change: %d->%d\n", __func__, from, to);
	pr_info("panel state change: %d->%d\n", from, to);
	switch (to) {
	case TS_MMI_PM_GESTURE:
	case TS_MMI_PM_DEEPSLEEP:
		fts_suspend_func(ts);
			break;
	case TS_MMI_PM_ACTIVE:
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
	ASSERT_PTR(ts);
	pr_info("enter\n");
	dev_dbg(dev, "%s\n", __func__);
	fts_resume_func(ts);
	pr_info("IRQ is %s\n", fts_is_InterruptEnabled() ? "EN" : "DIS");
	/* restore data */
	if (ts->board->report_rate_ctrl)
		fts_mmi_set_report_rate(ts, ts->report_rate);
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
	int res = OK;
	int ret = OK;
	u8 settings[4] = { 0 };
	ASSERT_PTR(ts);

	ts->mode = MODE_NOTHING;	/* initialize the mode to nothing in
					 * order to be updated depending on the
					 * features enabled */
	ts->charger_enabled = mode;
	settings[0] = ts->charger_enabled;
	ret = setFeatures(FEAT_SEL_CHARGER, settings, 1);
	if (ret < OK)
		logError(1, "%s %s: error during setting CHARGER_MODE! ERROR %08X\n",
			tag, __func__, ret);

	res |= ret;

	if (ret >= OK && ts->charger_enabled == FEAT_ENABLE) {
		fromIDtoMask(FEAT_SEL_CHARGER, (u8 *)&ts->mode, sizeof(ts->mode));
		logError(1, "%s %s: CHARGER_MODE Enabled!\n",
			tag, __func__);
	} else
		logError(1, "%s %s: CHARGER_MODE Disabled!\n", tag, __func__);

	return res;
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
	.get_class_entry_name = fts_mmi_get_class_entry_name,
	/* SET methods */
	.reset =  fts_mmi_reset,
	.drv_irq = fts_mmi_drv_irq,
	.charger_mode = fts_mmi_charger_mode,
	.power = fts_mmi_power,
	.pinctrl = fts_mmi_pinctrl,
	.wait_for_ready = fts_mmi_wait4ready,
	/* Firmware */
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
		ret = fts_chip_initialization(ts, SPECIAL_FULL_PANEL_INIT);
		if (ret)
			dev_err(ts->dev, "Failed chip init\n");
		fts_init_sensing(ts);
		fts_enableInterrupt();

		/* initialize value */
		/* The report rate defaults to 240Hz */
		if (ts->board->report_rate_ctrl)
			ts->report_rate = 0x00;

	} else
		ts_mmi_dev_unregister(ts->dev);
	return ret;
}
