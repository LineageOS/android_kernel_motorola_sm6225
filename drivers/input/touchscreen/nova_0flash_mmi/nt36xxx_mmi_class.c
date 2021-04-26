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
	snprintf(nvt_boot_firmware_name, NVT_FILE_NAME_LENGTH, "%s", fwname);
	snprintf(nvt_mp_firmware_name, NVT_FILE_NAME_LENGTH, "mp-%s", fwname);
	nvt_update_firmware(nvt_boot_firmware_name);
	mutex_unlock(&ts->lock);

	return 0;
}

static int nvt_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	dev_dbg(dev, "%s: panel state change: %d->%d\n", __func__, from, to);
	pr_info("panel state change: %d->%d\n", from, to);
	switch (to) {
		case TS_MMI_PM_GESTURE:
		case TS_MMI_PM_DEEPSLEEP:
			nvt_ts_suspend(&ts_data->client->dev);
			break;

		case TS_MMI_PM_ACTIVE:
			break;
		default:
			dev_warn(dev, "panel mode %d is invalid.\n", to);
			return -EINVAL;
	}

	pr_info("IRQ is %s\n", ts_data->irq_enabled ? "EN" : "DIS");
	return 0;

}
static int nvt_mmi_post_resume(struct device *dev)
{
	struct nvt_ts_data *ts_data;

	GET_TS_DATA(dev);
	pr_info("enter\n");
	dev_dbg(dev, "%s\n", __func__);
	nvt_ts_resume(&ts_data->client->dev);
	pr_info("IRQ is %s\n", ts_data->irq_enabled ? "EN" : "DIS");
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
	.get_poweron = nvt_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  nvt_mmi_methods_reset,
	.drv_irq = nvt_mmi_methods_drv_irq,
	.charger_mode = nvt_mmi_charger_mode,
	/* Firmware */
	.firmware_update = nvt_mmi_firmware_update,
	/* PM callback */
	.panel_state = nvt_mmi_panel_state,
	.post_resume = nvt_mmi_post_resume,

};

int nvt_mmi_init(struct nvt_ts_data *ts_data, bool enable) {
	int ret = 0;

	if (enable) {
		ret = ts_mmi_dev_register(&ts_data->client->dev, &nvt_mmi_methods);
		if (ret)
			dev_err(&ts_data->client->dev, "Failed to register ts mmi\n");
	} else
		ts_mmi_dev_unregister(&ts_data->client->dev);

	return ret;
}
