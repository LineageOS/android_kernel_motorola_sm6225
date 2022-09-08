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
	/* PM callback */
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
