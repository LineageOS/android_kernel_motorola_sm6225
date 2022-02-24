/*
 * Parade tma5xx touchscreen driver MMI class extention
 *
 * Copyright (C) 2022 Motorola Mobility
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

#include "cyttsp5_ts_mmi.h"
#include "cyttsp5_regs.h"

#define ASSERT_PTR(p) if (p == NULL) { \
		dev_err(dev, "Failed to get driver data"); \
		return -ENODEV; \
	}

extern int cyttsp5_core_suspend(struct device *dev);
extern int cyttsp5_core_resume(struct device *dev);

static int cyttsp5_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "parade");
}

static int cyttsp5_mmi_get_class_entry_name(struct device *dev, void *cdata) {
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	ASSERT_PTR(pdata);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", pdata->core_pdata->class_entry_name);
}

static int cyttsp5_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	char* ic_info;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	ASSERT_PTR(pdata);

	ic_info = strstr(pdata->ic_name, ",");
	ic_info++;

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", ic_info);
}

static int cyttsp5_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	u32 fw_ver_img;
	u32 fw_revctrl_img;
	u16 fw_config_ver_img;
	struct cyttsp5_sysinfo *si;

	struct cyttsp5_core_commands *cmd = cyttsp5_get_commands();
	if (!cmd)
		return -EINVAL;

	si = cmd->request_sysinfo(dev);
	if (!si) {
		dev_err(dev, "%s: Fail get sysinfo pointer from core\n", __func__);
		return -EINVAL;
	}

	fw_ver_img = si->cydata.fw_ver_major << 8;
	fw_ver_img += si->cydata.fw_ver_minor;
	fw_revctrl_img = si->cydata.revctrl;
	fw_config_ver_img = si->cydata.fw_ver_conf;

	parade_debug(dev, DEBUG_LEVEL_1,
		"%s: img vers:0x%04X ,revctrl vers:0x%04X, fw_config vers:0x%04X\n", __func__,
			fw_ver_img, fw_revctrl_img, fw_config_ver_img);

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", le32_to_cpu(fw_config_ver_img));
}

/*return firmware version*/
static int cyttsp5_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	u32 fw_ver_img;
	u32 fw_revctrl_img;
	struct cyttsp5_sysinfo *si;

	struct cyttsp5_core_commands *cmd = cyttsp5_get_commands();
	if (!cmd)
		return -EINVAL;

	si = cmd->request_sysinfo(dev);
	if (!si) {
		dev_err(dev, "%s: Fail get sysinfo pointer from core\n", __func__);
		return -EINVAL;
	}

	fw_ver_img = si->cydata.fw_ver_major << 8;
	fw_ver_img += si->cydata.fw_ver_minor;
	fw_revctrl_img = si->cydata.revctrl;

	parade_debug(dev, DEBUG_LEVEL_1,
		"%s: img vers:0x%04X ,revctrl vers:0x%04X\n", __func__,
			fw_ver_img, fw_revctrl_img);

	return snprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%04x", le32_to_cpu(fw_revctrl_img));
}

static int cyttsp5_ts_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	struct cyttsp5_core_data *cd =  dev_get_drvdata(dev);
	ASSERT_PTR(cd);

	TO_INT(idata) = cd->bus_ops->bustype == BUS_I2C ?
			TOUCHSCREEN_MMI_BUS_TYPE_I2C : TOUCHSCREEN_MMI_BUS_TYPE_SPI;
	return 0;
}

static int cyttsp5_ts_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	ASSERT_PTR(pdata);

	TO_INT(idata) = gpio_get_value(pdata->core_pdata->irq_gpio);
	return 0;
}

static int cyttsp5_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct cyttsp5_core_data *cd =  dev_get_drvdata(dev);
	ASSERT_PTR(cd);

	TO_INT(idata) = cd ->irq_enabled;
	return 0;
}

static int cyttsp5_ts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	ASSERT_PTR(pdata);

	TO_INT(idata) = (regulator_is_enabled(pdata->core_pdata->avdd) &&
		regulator_is_enabled(pdata->core_pdata->iovdd)) ? 1 : 0;

	return 0;
}

static int cyttsp5_ts_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	int rc;
	u8 mode;
	struct cyttsp5_core_commands *cmd = cyttsp5_get_commands();
	if (!cmd)
		return -EINVAL;

	rc = cmd->request_get_mode(dev, 0, &mode);
	if (rc < 0)
		return rc;

	TO_INT(idata) = mode == CY_MODE_BOOTLOADER ? 1 : 0;
	return 0;
}

static int cyttsp5_ts_mmi_methods_reset(struct device *dev, int type) {
	struct cyttsp5_core_commands *cmd = cyttsp5_get_commands();
	if (!cmd)
		return -EINVAL;

	cmd->request_reset(dev);
	return 0;
}

static int cyttsp5_ts_firmware_update(struct device *dev, char *fwname) {
	struct cyttsp5_core_data *cd =  dev_get_drvdata(dev);
	ASSERT_PTR(cd);

	dev_info(dev, "%s, HW request update fw, %s", __func__, fwname);

	if (!fwname) {
		dev_err(dev, "%s: Error, firmware name is null\n",
				__func__);
		return -EINVAL;
	}

	if ( !cd->firmware_update)
	    return -ENOSYS;

	cd->force_fw_upgrade = 1;
	snprintf(cd->firmware_name, CYTTSP5_FIRMWARE_NAME_MAX_LEN, "%s", fwname);

	if(cd->sleep_state != SS_SLEEP_OFF) {
		dev_err(dev, "%s: Error, touch IC is on sleep state, can not do upgrade\n",
				__func__);
		return -EINVAL;
	}

	return cd->firmware_update(dev, fwname);
}

static __maybe_unused int cyttsp5_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	int ret = -ENODEV;
	struct cyttsp5_core_data *cd =  dev_get_drvdata(dev);
	ASSERT_PTR(cd);

	mutex_lock(&cd->system_lock);
	switch (state) {
	case 0:
		if (cd->irq_enabled) {
			cd->irq_enabled = false;
			/* Disable IRQ */
			disable_irq_nosync(cd->irq);
			dev_info(dev, "%s: Driver IRQ now disabled\n",
				__func__);
		} else
			dev_info(dev, "%s: Driver IRQ already disabled\n",
				__func__);
		ret = 0;
		break;

	case 1:
		if (cd->irq_enabled == false) {
			cd->irq_enabled = true;
			/* Enable IRQ */
			enable_irq(cd->irq);
			dev_info(dev, "%s: Driver IRQ now enabled\n",
				__func__);
		} else
			dev_info(dev, "%s: Driver IRQ already enabled\n",
				__func__);
		ret = 0;
		break;

	default:
		dev_err(dev, "%s: Invalid value\n", __func__);
	}
	mutex_unlock(&(cd->system_lock));

	return ret;
}

static int cyttsp5_ts_mmi_methods_power(struct device *dev, int on) {
	int rc = 0;
	atomic_t is_ignore;
	struct cyttsp5_core_data *cd =  dev_get_drvdata(dev);
	ASSERT_PTR(cd);

	atomic_set(&is_ignore, 1);
	/* Call platform power function */
	if (cd->cpdata->power) {
		dev_info(cd->dev, "%s: Touch IC power on/off \n", __func__);
		if(on == TS_MMI_POWER_ON)
		{
			rc = cd->cpdata->power(cd->cpdata, TS_MMI_POWER_ON, cd->dev, &is_ignore);
			if (rc) {
				dev_info(cd->dev, "%s: Power on touch IC success\n", __func__);
				rc = -ENODEV;
			}
		}
		else if(on == TS_MMI_POWER_OFF)
		{
			rc = cd->cpdata->power(cd->cpdata, TS_MMI_POWER_OFF, cd->dev, &is_ignore);
			if (rc) {
				dev_info(cd->dev, "%s: Power on touch IC failed \n", __func__);
				rc = -ENODEV;
			}
		}
	}
	else {
		dev_info(cd->dev, "%s: No Power operation func\n", __func__);
		rc = -ENODEV;
	}

	return rc;
}

static int cyttsp5_ts_mmi_post_suspend(struct device *dev) {
	pr_info("%s: Post suspend end", __func__);
	cyttsp5_core_suspend(dev);
	return 0;
}

static int cyttsp5_ts_mmi_post_resume(struct device *dev) {
	pr_info("%s: Post resume end", __func__);
	cyttsp5_core_resume(dev);
	return 0;
}

static struct ts_mmi_methods cyttsp5_ts_mmi_methods = {
	.get_vendor = cyttsp5_ts_mmi_methods_get_vendor,
	.get_class_entry_name = cyttsp5_mmi_get_class_entry_name,
	.get_productinfo = cyttsp5_ts_mmi_methods_get_productinfo,
	.get_build_id = cyttsp5_ts_mmi_methods_get_build_id,
	.get_config_id = cyttsp5_ts_mmi_methods_get_config_id,
	.get_bus_type = cyttsp5_ts_mmi_methods_get_bus_type,
	.get_irq_status = cyttsp5_ts_mmi_methods_get_irq_status,
	.get_drv_irq = cyttsp5_ts_mmi_methods_get_drv_irq,
	.get_poweron = cyttsp5_ts_mmi_methods_get_poweron,
	.get_flashprog = cyttsp5_ts_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  cyttsp5_ts_mmi_methods_reset,
	.power = cyttsp5_ts_mmi_methods_power,
	/* Firmware */
	.firmware_update = cyttsp5_ts_firmware_update,
	/* PM callback */
	.post_resume = cyttsp5_ts_mmi_post_resume,
	.post_suspend = cyttsp5_ts_mmi_post_suspend,
};

int cyttsp5_ts_mmi_dev_register(struct device *dev) {
	int ret;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	ret = ts_mmi_dev_register(dev, &cyttsp5_ts_mmi_methods);
	if (ret) {
		dev_err(dev, "%s, Failed to register ts mmi\n", __func__);
		return ret;
	}
	dev_info(dev, "%s, Register ts mmi success\n", __func__);

	cd->imports = &cyttsp5_ts_mmi_methods.exports;

	return 0;
}

void cyttsp5_ts_mmi_dev_unregister(struct device *dev) {
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (!cd)
		dev_err(dev ,"Failed to get driver data");
	ts_mmi_dev_unregister(dev);
}

