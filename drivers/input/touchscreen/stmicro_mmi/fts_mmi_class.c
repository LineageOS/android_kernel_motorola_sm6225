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

extern void fts_resume_func(struct fts_ts_info *info);
extern void fts_suspend_func(struct fts_ts_info *info);
extern int fts_fw_update(struct fts_ts_info *info);

#define ASSERT_PTR(p) if (p == NULL) { \
	dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
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
	char buffer[64];
	ssize_t blen = 0;
	ASSERT_PTR(ts);
	blen += scnprintf(buffer+blen, sizeof(buffer)-blen, "fts%04x",
		ts->sysinfo->u16_chip0Id);
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
	pr_info("enter\n");
	//TO_INT(idata) = (atomic_read(&ts->touch_stopped) == 0 && ts->flash_enabled) ? 1 : 0;
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
	ret = fts_fw_update(ts);
	ts->fw_file = NULL;
	return ret;
}

static int fts_mmi_charger_mode(struct device *dev, int mode)
{
	struct fts_ts_info *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
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

		dev_info(ts->dev, "MMI start sensing\n");
		ret = fts_chip_initialization(ts, SPECIAL_FULL_PANEL_INIT);
		if (ret)
			dev_err(ts->dev, "Failed chip init\n");
		fts_init_sensing(ts);
		fts_enableInterrupt();
	} else
		ts_mmi_dev_unregister(ts->dev);
	return ret;
}
