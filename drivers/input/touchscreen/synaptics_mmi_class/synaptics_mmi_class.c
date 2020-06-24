/*
 * Synaptics DSX touchscreen driver MMI class extention
 *
 * Copyright (C) 2019 Motorola Mobility
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
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/touchscreen_mmi.h>

#include "synaptics_dsx_i2c.h"

#define ASSERT_PTR(p) if (p == NULL) { \
	dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
	}

static int synaptics_mmi_get_class_entry_name(struct device *dev, void *cdata) {
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", ts->class_entry_name);
}

static int synaptics_mmi_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "synaptics");
}

static int synaptics_mmi_get_productinfo(struct device *dev, void *cdata) {
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s",
		ts->rmi4_mod_info.product_id_string);
}

static int synaptics_mmi_get_build_id(struct device *dev, void *cdata) {
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	unsigned int firmware_id;
	struct synaptics_rmi4_device_info *rmi;

	ASSERT_PTR(ts);
	rmi = &(ts->rmi4_mod_info);
	batohui(&firmware_id, rmi->build_id, sizeof(rmi->build_id));

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%x", firmware_id);
}

static int synaptics_mmi_get_config_id(struct device *dev, void *cdata) {
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	unsigned int config_id;
	struct synaptics_rmi4_device_info *rmi;

	ASSERT_PTR(ts);
	rmi = &(ts->rmi4_mod_info);
	batohui(&config_id, rmi->config_id, sizeof(rmi->config_id));

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%08x", config_id);
}

static int synaptics_mmi_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int synaptics_mmi_get_irq_status(struct device *dev, void *idata) {
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	TO_INT(idata) = gpio_get_value(ts->board.irq_gpio);

	return 0;
}

static int synaptics_mmi_get_drv_irq(struct device *dev, void *idata) {
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	TO_INT(idata) = ts->irq_enabled ? 1 : 0;

	return 0;
}

static int synaptics_mmi_get_poweron(struct device *dev, void *idata)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	TO_INT(idata) = (atomic_read(&ts->touch_stopped) == 0 && ts->flash_enabled) ? 1 : 0;

	return 0;
}

static int synaptics_mmi_get_flashprog(struct device *dev, void *idata)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	TO_INT(idata) = ts->in_bootloader ? 1 : 0;

	return 0;
}

static int synaptics_mmi_drv_irq(struct device *dev, int state)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);

	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	switch (state) {
	case 0: /* Disable irq */
		synaptics_rmi4_irq_enable(ts, false);
			break;
	case 1: /* Enable irq */
		synaptics_rmi4_irq_enable(ts, true);
			break;
	default:
		dev_err(dev, "%s: invalid value\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int synaptics_mmi_reset(struct device *dev, int type)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	int ret;

	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	ret = synaptics_dsx_ic_reset(ts, type);
	if (ret > 0)
		dev_dbg(dev,"%s: successful reset took %dms\n", __func__, ret);
	else
		dev_warn(dev, "%s: timed out waiting for idle\n", __func__);

	return 0;
}

static int synaptics_mmi_panel_state(struct device *dev,
		enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s: panel state change: %d->%d\n", __func__, from, to);
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	switch (to) {
	case TS_MMI_PM_GESTURE:
	case TS_MMI_PM_DEEPSLEEP:
		synaptics_rmi4_suspend(dev);
			break;
	case TS_MMI_PM_ACTIVE:
			break;
	default:
		dev_warn(dev, "invalid panel state %d\n", to);
		return -EINVAL;
	}
	return 0;
}

static int synaptics_mmi_post_resume(struct device *dev)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	synaptics_rmi4_resume(dev);
	return 0;
}

static int synaptics_mmi_fw_update(struct device *dev, char *fwname)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu;
	const char *name = fwname;
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	fwu = ts->fwu_data;
	if (!fwu || !fwu->firmware_update)
		return -ENOSYS;
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	atomic_set(&ts->query_done, 0);
	return fwu->firmware_update(dev, name);
}

static int synaptics_mmi_fw_erase(struct device *dev)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu;
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	fwu = ts->fwu_data;
	if (!fwu || !fwu->firmware_erase)
		return -ENOSYS;
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	return fwu->firmware_erase(dev);
}

static int synaptics_mmi_charger_mode(struct device *dev, int mode)
{
	struct synaptics_rmi4_data *ts = dev_get_drvdata(dev);
	ASSERT_PTR(ts);
	dev_dbg(dev, "%s\n", __func__);
	if (atomic_read(&ts->query_done) != 1)
		return -EBUSY;
	synaptics_dsx_charger_mode(ts, mode);
	return 0;
}

static struct ts_mmi_methods synaptics_mmi_methods = {
	.get_vendor = synaptics_mmi_get_vendor,
	.get_productinfo = synaptics_mmi_get_productinfo,
	.get_build_id = synaptics_mmi_get_build_id,
	.get_config_id = synaptics_mmi_get_config_id,
	.get_bus_type = synaptics_mmi_get_bus_type,
	.get_irq_status = synaptics_mmi_get_irq_status,
	.get_drv_irq = synaptics_mmi_get_drv_irq,
	.get_poweron = synaptics_mmi_get_poweron,
	.get_flashprog = synaptics_mmi_get_flashprog,
	.get_class_entry_name = synaptics_mmi_get_class_entry_name,
	/* SET methods */
	.reset =  synaptics_mmi_reset,
	.drv_irq = synaptics_mmi_drv_irq,
	.charger_mode = synaptics_mmi_charger_mode,
	/* Firmware */
	.firmware_update = synaptics_mmi_fw_update,
	.firmware_erase = synaptics_mmi_fw_erase,
	/* vendor specific attribute group */
	/* PM callback */
	.panel_state = synaptics_mmi_panel_state,
	.post_resume = synaptics_mmi_post_resume,
};

int synaptics_mmi_data_init(struct synaptics_rmi4_data *ts, bool enable)
{
	int ret = 0;
	if (enable) {
		ret = ts_mmi_dev_register(&ts->i2c_client->dev, &synaptics_mmi_methods);
		if (ret)
			dev_err(&ts->i2c_client->dev, "Failed to register ts mmi\n");

	} else
		ts_mmi_dev_unregister(&ts->i2c_client->dev);

	return ret;
}
