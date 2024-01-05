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
#include "mmi_discrete_voter.h"


#define CHG_SHOW_MAX_SIZE 50
static ssize_t force_chg_usb_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long  r;
	unsigned long mode;
	static int old_current = 0;
	static bool old_suspend = false;

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
	pr_info("%s:mode=%d, old_suspend=%d, old_current=%d\n", __func__,(bool)mode, old_suspend, old_current);
	if (mode == true && old_suspend == false) {
		old_suspend = true;
		old_current = get_effective_result(mmi_chip->usb_icl_votable);
		/* vote 0mA when suspended */
		pmic_vote_force_val_set(mmi_chip->usb_icl_votable, 0);
		pmic_vote_force_active_set(mmi_chip->usb_icl_votable, 1);
	} else if (mode == false && old_suspend == true) {
		old_suspend = false;
		if (get_effective_result(mmi_chip->usb_icl_votable) == 0) {
			pmic_vote_force_val_set(mmi_chip->usb_icl_votable, (u32)old_current);
			pmic_vote_force_active_set(mmi_chip->usb_icl_votable, 1);
		}
	}

	return r ? r : count;
}

static ssize_t force_chg_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	bool state;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("mmi_discrete_charger: chip not valid\n");
		return -ENODEV;
	}

	state = mmi_discrete_is_usb_suspended(mmi_chip);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d", state);
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
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d", state);
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
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%ul", state);
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
	pmic_vote_force_val_set(mmi_chip->usb_icl_votable, (u32)usb_curr);
	pmic_vote_force_active_set(mmi_chip->usb_icl_votable, 1);

	return r ? r : count;
}

static ssize_t force_chg_iusb_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int usb_curr;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		usb_curr = -ENODEV;
		goto end;
	}
	usb_curr = get_effective_result(mmi_chip->usb_icl_votable);

	if (usb_curr < 0) {
		mmi_err(mmi_chip, "Factory Couldn't get usb_icl rc=%d\n", usb_curr);
		return usb_curr;
	}
	usb_curr /= 1000; /* Convert to mA */
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d", usb_curr);
}

static DEVICE_ATTR(force_chg_iusb, 0664,
		force_chg_iusb_show,
		force_chg_iusb_store);

static ssize_t chg_type_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	pr_err("%s un-supported\n", __func__);
	return count;
}

static ssize_t chg_type_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int charger_type;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmi_discrete_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		charger_type = -ENODEV;
		goto end;
	}
	//Because, Mtk platform TYPE DCP is 4.
	// In order to be compatible with MTK platform. return "4" when real_charge_type=DCP
	if (POWER_SUPPLY_TYPE_USB_DCP == mmi_chip->real_charger_type)
		charger_type = 4;
	else
		charger_type = 1; // 1 is SDP in mkt platform
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d", charger_type);
}

static DEVICE_ATTR(chg_type, 0664,
		chg_type_show,
		chg_type_store);


static ssize_t force_chg_fail_clear_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("mmi_discrete_charger: Invalid chg fail mode value = %lu\n", mode);
		return -EINVAL;
	}

	/* do nothing for mmi_discrete_charger */
	r = 0;

	return r ? r : count;
}

static ssize_t force_chg_fail_clear_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* do nothing for mmi_discrete_charger */
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0");
}

static DEVICE_ATTR(force_chg_fail_clear, 0664,
		force_chg_fail_clear_show,
		force_chg_fail_clear_store);


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

	rc = device_create_file(chip->dev,
				&dev_attr_force_chg_fail_clear);
	if (rc) {
		mmi_err(chip,
			"Couldn't create force_chg_fail_clear\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_chg_type);
	if (rc) {
		mmi_err(chip,
			"Couldn't create chg_type\n");
	}


	return rc;
}

