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

#include <linux/touchscreen_mmi.h>
#include "goodix_ts_mmi.h"
#include "goodix_cfg_bin.h"
extern int goodix_ts_unregister_notifier(struct notifier_block *nb);

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}
static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", GOODIX_TP_IC_TYPE);
}
static int goodix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	int ret;
	struct goodix_ts_version fw_ver;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;
	if (!ts_dev) {
		ts_err("Failed to get platform data");
		return -ENODEV;
	}
	ret = ts_dev->hw_ops->read_version(ts_dev, &fw_ver);
	if (!ret && fw_ver.valid) {
		ret = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fw_ver.vid[0]);
		ts_info("get config id:%d ", fw_ver.vid[0]);
		return ret;
	} else
		return -ENODEV;
}
/*return firmware version*/
static int goodix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	int ret;
	struct goodix_ts_version fw_ver;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;
	if (!ts_dev) {
		ts_err("Failed to get platform data");
		return -ENODEV;
	}
	ret = ts_dev->hw_ops->read_version(ts_dev, &fw_ver);
	if (!ret && fw_ver.valid) {
		ret = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fw_ver.vid[3]);
		ts_info("get build id:%d ", fw_ver.vid[3]);
		return ret;
	} else
		return -ENODEV;
}
static int goodix_ts_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}
static int goodix_ts_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct goodix_ts_board_data *ts_bdata;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(ts_bdata->irq_gpio);
	return 0;
}
static int goodix_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	TO_INT(idata) = atomic_read(&core_data->irq_enabled);
	return 0;
}
static int goodix_ts_mmi_methods_get_poweron(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	TO_INT(idata) = core_data->power_on;
	return 0;
}
static int goodix_ts_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	if (core_data->ts_mmi_info.need_reflash)
		TO_INT(idata) = 1;
	else
		TO_INT(idata) = core_data->ts_mmi_info.update_status;
	return 0;
}
static int goodix_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	goodix_ts_irq_enable(core_data, !(!state));
	return 0;
}
/* reset chip
 * type：can control software reset and hardware reset,
 * but GOODIX has no software reset,
 * so the type parameter is not used here.
 */
static int goodix_ts_mmi_methods_reset(struct device *dev, int type) {
	int ret = -ENODEV;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;
	if (ts_dev->hw_ops->reset)
		ret = ts_dev->hw_ops->reset(ts_dev);
	return ret;
}
static int goodix_ts_firmware_update(struct device *dev, char *fwname) {
	struct goodix_ts_board_data *ts_bdata;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}

	ts_debug("HW request update fw");
	/* set firmware image name */
	if (core_data->set_fw_name)
		core_data->set_fw_name(fwname);
	else
		ts_err("fw update handler not ready");

	if (core_data->do_fw_update)
		core_data->do_fw_update(UPDATE_MODE_SRC_REQUEST |
					UPDATE_MODE_BLOCK |
					UPDATE_MODE_FORCE);
	else {
		ts_err("fw update handler not ready.");
		return -EFAULT;
	}
	core_data->ts_mmi_info.need_reflash = 0;
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
	/* Firmware */
	.firmware_update = goodix_ts_firmware_update,
};

static int mmi_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct goodix_ts_mmi *mmi_info = container_of(nb,
			struct goodix_ts_mmi, mmi_notifier);
	struct goodix_ts_core *core_data = container_of(mmi_info,
			struct goodix_ts_core, ts_mmi_info);
	switch (action) {
	case NOTIFY_FWUPDATE_START:
		ts_info("Firmware update start");
		core_data->ts_mmi_info.update_status = 1;
		break;
	case NOTIFY_FWUPDATE_FAILED:
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_FWUPDATE_SKIP:
		core_data->ts_mmi_info.update_status = 0;
		break;
	default:
		ts_err("Firmware state unknown");
		break;
	}
	return 0;
}

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	core_data->ts_mmi_info.mmi_notifier.notifier_call = mmi_notifier_callback;
	goodix_ts_register_notifier(&core_data->ts_mmi_info.mmi_notifier);
	ret = ts_mmi_dev_register(&pdev->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		return ret;
	}
	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_err("Failed to get driver data");
	ts_mmi_dev_unregister(&pdev->dev);
	goodix_ts_unregister_notifier(&core_data->ts_mmi_info.mmi_notifier);
}
