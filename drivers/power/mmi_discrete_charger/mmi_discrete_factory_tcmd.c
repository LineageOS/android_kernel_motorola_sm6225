/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/string.h>
#include <linux/mmi_discrete_charger_class.h>
#include "mmi_discrete_charger_core.h"


#define CHG_SHOW_MAX_SIZE 50
static ssize_t force_chg_usb_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long  r;
	unsigned long mode;

	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("mmi_discrete_charger: Invalid usb suspend mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		return -ENODEV;
	}
	r = charger_dev_set_usb_suspend(mmi_chip->master_chg_dev, (bool)mode);
	if (r < 0)
		mmi_err(mmi_chip, "Couldn't set usb suspend rc=%d\n", (int)r);

	return r ? r : count;
}

static ssize_t force_chg_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	bool state;
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		return -ENODEV;
	}
	ret =  charger_dev_is_usb_suspend(mmi_chip->master_chg_dev, &state);
	if (ret) {
		mmi_err(mmi_chip, "USBIN_SUSPEND_BIT failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_usb_suspend, 0664,
		force_chg_usb_suspend_show,
		force_chg_usb_suspend_store);

static ssize_t force_chg_auto_enable_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long  r;
	bool mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtobool(buf, &mode);
	if (r) {
		pr_err("mmi_discrete_charger: Invalid chrg enable value = %d\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		return -ENODEV;
	}
	r = charger_dev_enable_charging(mmi_chip->master_chg_dev, mode);
	if (r < 0) {
		mmi_err(mmi_chip, "Factory Couldn't %s charging rc=%d\n",
			   mode ? "enable" : "disable", (int)r);
		return r;
	}

	return r ? r : count;
}

static ssize_t force_chg_auto_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	int state;
	bool value;

	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		state = -ENODEV;
		goto end;
	}
	ret = charger_dev_is_enabled_charging(mmi_chip->master_chg_dev, &value);
	if (ret) {
		mmi_err(mmi_chip, "CHG_EN_BIT failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}
	state = value;
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_auto_enable, 0664,
		force_chg_auto_enable_show,
		force_chg_auto_enable_store);


static ssize_t force_chg_ibatt_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long  r;
	unsigned long chg_current;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		pr_err("mmi_discrete_charger: Invalid ibatt value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		return -ENODEV;
	}

	chg_current *= 1000; /* Convert to uA */
	r = charger_dev_set_charging_current(mmi_chip->master_chg_dev, chg_current);
	if (r < 0)
		mmi_err(mmi_chip, "Couldn't set ibatt value rc=%d\n", (int)r);

	return r ? r : count;
}

static ssize_t force_chg_ibatt_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	u32 state;

	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		state = -ENODEV;
		goto end;
	}
	ret = charger_dev_get_charging_current(mmi_chip->master_chg_dev, &state);
	if (ret < 0) {
		mmi_err(mmi_chip, "Factory Couldn't get ibatt  rc=%d\n", ret);
		return ret;
	}

	state /= 1000; /* Convert to mA */
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%ul\n", state);
}

static DEVICE_ATTR(force_chg_ibatt, 0664,
		force_chg_ibatt_show,
		force_chg_ibatt_store);

static ssize_t force_chg_iusb_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned long  r;
	unsigned long usb_curr;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &usb_curr);
	if (r) {
		pr_err("mmi_discrete_charger: Invalid iusb value = %lu\n", usb_curr);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		return -ENODEV;
	}

	usb_curr *= 1000; /* Convert to uA */
	r =charger_dev_set_input_current(mmi_chip->master_chg_dev, usb_curr);
	if (r < 0)
		mmi_err(mmi_chip, "Couldn't set USBIN_AICL_OPTIONS rc=%d\n", (int)r);

	return r ? r : count;
}

static ssize_t force_chg_iusb_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 usb_curr;
	unsigned long r;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		usb_curr = -ENODEV;
		goto end;
	}
	r = charger_dev_get_input_current(mmi_chip->master_chg_dev, &usb_curr);

	if (r < 0) {
		mmi_err(mmi_chip, "Factory Couldn't get usb_icl rc=%d\n", (int)r);
		return r;
	}
	usb_curr /= 1000; /* Convert to mA */
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%ul\n", usb_curr);
}

static DEVICE_ATTR(force_chg_iusb, 0664,
		force_chg_iusb_show,
		force_chg_iusb_store);


int mmi_discrete_create_factory_testcase(struct mmi_discrete_charger *chip)
{
	int rc;

	rc = device_create_file(chip->dev,
				&dev_attr_force_chg_usb_suspend);
	if (rc) {
		mmi_err(chip,
			   "Couldn't create force_chg_usb_suspend\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_force_chg_auto_enable);
	if (rc) {
		mmi_err(chip,
			   "Couldn't create force_chg_auto_enable\n");
	}

	rc = device_create_file(chip->dev,
			&dev_attr_force_chg_ibatt);
	if (rc) {
		mmi_err(chip,
			"Couldn't create force_chg_ibatt\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_force_chg_iusb);
	if (rc) {
		mmi_err(chip,
			"Couldn't create force_chg_iusb\n");
	}

	return rc;
}

