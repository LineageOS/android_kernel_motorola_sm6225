/* Copyright (c) 2017, 2018 The Linux Foundation. All rights reserved.
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
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#ifndef PM8150B_SUBTYPE
#define PM8150B_SUBTYPE PM855B_SUBTYPE
#endif

#define CHGR_BASE	0x1000
#define DCDC_BASE	0x1100
#define BATIF_BASE	0x1200
#define USBIN_BASE	0x1300
#define DCIN_BASE	0x1400
#define TYPEC_BASE	0X1500
#define MISC_BASE	0x1600

#define USBIN_CMD_ICL_OVERRIDE_REG		(USBIN_BASE + 0x42)
#define USBIN_ICL_OVERRIDE_BIT			BIT(0)
#define USBIN_ICL_OPTIONS_REG			(USBIN_BASE + 0x66)
#define USBIN_MODE_CHG_BIT			BIT(0)
#define USBIN_LOAD_CFG_REG			(USBIN_BASE + 0x65)
#define ICL_OVERRIDE_AFTER_APSD_BIT		BIT(4)
#define USBIN_AICL_OPTIONS_CFG_REG		(USBIN_BASE + 0x80)
#define LEGACY_CABLE_CFG_REG			(TYPEC_BASE + 0x5A)
#define USBIN_ADAPTER_ALLOW_CFG_REG		(USBIN_BASE + 0x60)
#define USBIN_ADAPTER_ALLOW_MASK		GENMASK(3, 0)
enum {
	USBIN_ADAPTER_ALLOW_5V		= 0,
	USBIN_ADAPTER_ALLOW_9V		= 2,
	USBIN_ADAPTER_ALLOW_5V_OR_9V	= 3,
	USBIN_ADAPTER_ALLOW_12V		= 4,
	USBIN_ADAPTER_ALLOW_5V_OR_12V	= 5,
	USBIN_ADAPTER_ALLOW_9V_TO_12V	= 6,
	USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V = 7,
	USBIN_ADAPTER_ALLOW_5V_TO_9V	= 8,
	USBIN_ADAPTER_ALLOW_5V_TO_12V	= 12,
};

struct smb_mmi_chg_param {
	const char	*name;
	u16		reg;
	int		min_u;
	int		max_u;
	int		step_u;
	int		(*get_proc)(struct smb_mmi_chg_param *param,
				    u8 val_raw);
	int		(*set_proc)(struct smb_mmi_chg_param *param,
				    int val_u,
				    u8 *val_raw);
};

struct smb_mmi_params {
	struct smb_mmi_chg_param	fcc;
	struct smb_mmi_chg_param	fv;
	struct smb_mmi_chg_param	usb_icl;
	struct smb_mmi_chg_param	dc_icl;
};

struct smb_mmi_charger {
	struct device		*dev;
	struct regmap 		*regmap;
	struct smb_mmi_params	param;
	char			*name;
	int			smb_version;

	bool			factory_mode;
	int			demo_mode;
	struct notifier_block	smb_reboot;

	struct power_supply	*mmi_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*main_psy;
	struct power_supply	*pc_port_psy;
	struct notifier_block	mmi_psy_notifier;
	struct delayed_work	heartbeat_work;

	struct votable 		*chg_dis_votable;
	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;

	bool			enable_charging_limit;
	bool			is_factory_image;
	//enum charging_limit_modes	charging_limit_modes;
	int			upper_limit_capacity;
	int			lower_limit_capacity;
	int			max_chrg_temp;
	int			last_iusb_ua;
	bool			factory_kill_armed;
};

#define CHGR_FAST_CHARGE_CURRENT_CFG_REG	(CHGR_BASE + 0x61)
#define CHGR_FLOAT_VOLTAGE_CFG_REG		(CHGR_BASE + 0x70)
#define USBIN_CURRENT_LIMIT_CFG_REG		(USBIN_BASE + 0x70)
#define DCIN_CURRENT_LIMIT_CFG_REG		(DCIN_BASE + 0x70)
static struct smb_mmi_params smb5_pm8150b_params = {
	.fcc			= {
		.name   = "fast charge current",
		.reg    = CHGR_FAST_CHARGE_CURRENT_CFG_REG,
		.min_u  = 0,
		.max_u  = 8000000,
		.step_u = 50000,
	},
	.fv			= {
		.name   = "float voltage",
		.reg    = CHGR_FLOAT_VOLTAGE_CFG_REG,
		.min_u  = 3600000,
		.max_u  = 4790000,
		.step_u = 10000,
	},
	.usb_icl		= {
		.name   = "usb input current limit",
		.reg    = USBIN_CURRENT_LIMIT_CFG_REG,
		.min_u  = 0,
		.max_u  = 5000000,
		.step_u = 50000,
	},
	.dc_icl		= {
		.name   = "dc input current limit",
		/* TODO: For now USBIN seems to be the way to set this */
		.reg    = USBIN_CURRENT_LIMIT_CFG_REG,
		.min_u  = 0,
		.max_u  = 5000000,
		.step_u = 50000,
	},
};

static struct smb_mmi_params smb5_pmi632_params = {
	.fcc			= {
		.name   = "fast charge current",
		.reg    = CHGR_FAST_CHARGE_CURRENT_CFG_REG,
		.min_u  = 0,
		.max_u  = 3000000,
		.step_u = 50000,
	},
	.fv			= {
		.name   = "float voltage",
		.reg    = CHGR_FLOAT_VOLTAGE_CFG_REG,
		.min_u  = 3600000,
		.max_u  = 4800000,
		.step_u = 10000,
	},
	.usb_icl		= {
		.name   = "usb input current limit",
		.reg    = USBIN_CURRENT_LIMIT_CFG_REG,
		.min_u  = 0,
		.max_u  = 3000000,
		.step_u = 50000,
	},
	.dc_icl		= {
		.name   = "dc input current limit",
		/* TODO: For now USBIN seems to be the way to set this */
		.reg    = USBIN_CURRENT_LIMIT_CFG_REG,
		.min_u  = 0,
		.max_u  = 3000000,
		.step_u = 50000,
	},
};

static struct smb_mmi_params smb2_pm660_params = {
	.fcc			= {
		.name	= "fast charge current",
		.reg	= CHGR_FAST_CHARGE_CURRENT_CFG_REG,
		.min_u	= 0,
		.max_u	= 4500000,
		.step_u	= 25000,
	},
	.fv			= {
		.name	= "float voltage",
		.reg	= CHGR_FLOAT_VOLTAGE_CFG_REG,
		.min_u	= 3487500,
		.max_u	= 4920000,
		.step_u	= 7500,
	},
	.usb_icl		= {
		.name	= "usb input current limit",
		.reg	= USBIN_CURRENT_LIMIT_CFG_REG,
		.min_u	= 0,
		.max_u	= 4800000,
		.step_u	= 25000,
	},
	.dc_icl			= {
		.name	= "dc input current limit",
		.reg	= DCIN_CURRENT_LIMIT_CFG_REG,
		.min_u	= 0,
		.max_u	= 6000000,
		.step_u	= 25000,
	},
};

int smblib_read_mmi(struct smb_mmi_charger *chg, u16 addr, u8 *val)
{
	unsigned int value;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &value);
	if (rc >= 0)
		*val = (u8)value;

	return rc;
}

int smblib_write_mmi(struct smb_mmi_charger *chg, u16 addr, u8 val)
{
	return regmap_write(chg->regmap, addr, val);
}

int smblib_masked_write_mmi(struct smb_mmi_charger *chg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(chg->regmap, addr, mask, val);
}

#define CHG_SHOW_MAX_SIZE 50
static ssize_t factory_image_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid factory image mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	mmi_chip->is_factory_image = (mode) ? true : false;

	return r ? r : count;
}

static ssize_t factory_image_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = (mmi_chip->is_factory_image) ? 1 : 0;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_image_mode, 0644,
		factory_image_mode_show,
		factory_image_mode_store);

static ssize_t factory_charge_upper_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = mmi_chip->upper_limit_capacity;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_charge_upper, 0444,
		factory_charge_upper_show,
		NULL);

static ssize_t force_demo_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid demo  mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if ((mode >= 35) && (mode <= 80))
		mmi_chip->demo_mode = mode;
	else
		mmi_chip->demo_mode = 35;

	return r ? r : count;
}

static ssize_t force_demo_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = mmi_chip->demo_mode;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_demo_mode, 0644,
		force_demo_mode_show,
		force_demo_mode_store);

#define MIN_TEMP_C -20
#define MAX_TEMP_C 60
#define MIN_MAX_TEMP_C 47
static ssize_t force_max_chrg_temp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid max temp value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if ((mode >= MIN_MAX_TEMP_C) && (mode <= MAX_TEMP_C))
		mmi_chip->max_chrg_temp = mode;
	else
		mmi_chip->max_chrg_temp = MAX_TEMP_C;

	return r ? r : count;
}

static ssize_t force_max_chrg_temp_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = mmi_chip->max_chrg_temp;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_max_chrg_temp, 0644,
		force_max_chrg_temp_show,
		force_max_chrg_temp_store);

#define USBIN_CMD_IL_REG	(USBIN_BASE + 0x40)
#define USBIN_SUSPEND_BIT	BIT(0)
int smblib_set_usb_suspend(struct smb_mmi_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write_mmi(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				     suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		pr_err("Couldn't write %s to USBIN suspend rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_get_usb_suspend(struct smb_mmi_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read_mmi(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		pr_err("Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

static ssize_t force_chg_usb_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid usb suspend mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}
	r = smblib_set_usb_suspend(mmi_chip, (bool)mode);

	return r ? r : count;
}

static ssize_t force_chg_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}
	ret = smblib_get_usb_suspend(mmi_chip, &state);
	if (ret) {
		pr_err("USBIN_SUSPEND_BIT failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_usb_suspend, 0664,
		force_chg_usb_suspend_show,
		force_chg_usb_suspend_store);

static ssize_t force_chg_fail_clear_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid chg fail mode value = %lu\n", mode);
		return -EINVAL;
	}

	/* do nothing for SMBCHG */
	r = 0;

	return r ? r : count;
}

static ssize_t force_chg_fail_clear_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* do nothing for SMBCHG */
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0\n");
}

static DEVICE_ATTR(force_chg_fail_clear, 0664,
		force_chg_fail_clear_show,
		force_chg_fail_clear_store);

#define CHARGING_ENABLE_CMD_REG		(CHGR_BASE + 0x42)
#define CHARGING_ENABLE_CMD_BIT		BIT(0)
static ssize_t force_chg_auto_enable_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid chrg enable value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	r = smblib_masked_write_mmi(mmi_chip, CHARGING_ENABLE_CMD_REG,
				    CHARGING_ENABLE_CMD_BIT,
				    mode ? CHARGING_ENABLE_CMD_BIT : 0);
	if (r < 0) {
		pr_err("Factory Couldn't %s charging rc=%d\n",
			   mode ? "enable" : "disable", (int)r);
		return r;
	}

	return r ? r : count;
}

static ssize_t force_chg_auto_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int ret;
	u8 value;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_read_mmi(mmi_chip, CHARGING_ENABLE_CMD_REG, &value);
	if (ret) {
		pr_err("CHG_EN_BIT failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

	state = (CHARGING_ENABLE_CMD_BIT & value) ? 1 : 0;
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_auto_enable, 0664,
		force_chg_auto_enable_show,
		force_chg_auto_enable_store);

int smblib_set_charge_param(struct smb_mmi_charger *chg,
			    struct smb_mmi_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u)
			pr_debug("%s: %d is out of range [%d, %d]\n",
				 param->name, val_u,
				 param->min_u, param->max_u);

		if (val_u > param->max_u)
			val_u = param->max_u;
		if (val_u < param->min_u)
			val_u = param->min_u;

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write_mmi(chg, param->reg, val_raw);
	if (rc < 0) {
		pr_err("%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	pr_debug("%s = %d (0x%02x)\n", param->name, val_u, val_raw);

	return rc;
}

int smblib_get_charge_param(struct smb_mmi_charger *chg,
			    struct smb_mmi_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read_mmi(chg, param->reg, &val_raw);
	if (rc < 0) {
		pr_err("%s: Couldn't read from 0x%04x rc=%d\n",
		       param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	pr_debug("%s = %d (0x%02x)\n", param->name, *val_u, val_raw);

	return rc;
}

static ssize_t force_chg_ibatt_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long chg_current;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		pr_err("Invalid ibatt value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	chg_current *= 1000; /* Convert to uA */
	r = smblib_set_charge_param(mmi_chip, &mmi_chip->param.fcc, chg_current);
	if (r < 0) {
		pr_err("Factory Couldn't set masterfcc = %d rc=%d\n",
		       (int)chg_current, (int)r);
		return r;
	}

	return r ? r : count;
}

static ssize_t force_chg_ibatt_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_get_charge_param(mmi_chip, &mmi_chip->param.fcc, &state);
	if (ret < 0) {
		pr_err("Factory Couldn't get master fcc rc=%d\n", (int)ret);
		return ret;
	}

	state /= 1000; /* Convert to mA */
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_ibatt, 0664,
		force_chg_ibatt_show,
		force_chg_ibatt_store);

static ssize_t force_chg_iusb_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned long r;
	unsigned long usb_curr;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &usb_curr);
	if (r) {
		pr_err("Invalid iusb value = %lu\n", usb_curr);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	usb_curr *= 1000; /* Convert to uA */
	r = smblib_set_charge_param(mmi_chip, &mmi_chip->param.usb_icl, usb_curr);
	if (r < 0) {
		pr_err("Factory Couldn't set usb icl = %d rc=%d\n",
		       (int)usb_curr, (int)r);
		return r;
	}

	r = smblib_masked_write_mmi(mmi_chip, USBIN_ICL_OPTIONS_REG,
				    USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
	if (r < 0)
		pr_err("Couldn't set USBIN_ICL_OPTIONS r=%d\n", (int)r);

	r = smblib_masked_write_mmi(mmi_chip, USBIN_LOAD_CFG_REG,
				    ICL_OVERRIDE_AFTER_APSD_BIT,
				    ICL_OVERRIDE_AFTER_APSD_BIT);
	if (r < 0)
		pr_err("Couldn't set USBIN_LOAD_CFG rc=%d\n", (int)r);

	r = smblib_masked_write_mmi(mmi_chip, USBIN_AICL_OPTIONS_CFG_REG,
				    0xFF, 0x00);
	if (r < 0)
		pr_err("Couldn't set USBIN_AICL_OPTIONS rc=%d\n", (int)r);

	return r ? r : count;
}

static ssize_t force_chg_iusb_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int r;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		r = -ENODEV;
		goto end;
	}

	r = smblib_get_charge_param(mmi_chip, &mmi_chip->param.usb_icl, &state);
	if (r < 0) {
		pr_err("Factory Couldn't get usb_icl rc=%d\n", (int)r);
		return r;
	}
	state /= 1000; /* Convert to mA */
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_iusb, 0664,
		force_chg_iusb_show,
		force_chg_iusb_store);

static ssize_t force_chg_idc_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long dc_curr;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &dc_curr);
	if (r) {
		pr_err("Invalid idc value = %lu\n", dc_curr);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}
	dc_curr *= 1000; /* Convert to uA */
	r = smblib_set_charge_param(mmi_chip, &mmi_chip->param.dc_icl, dc_curr);
	if (r < 0) {
		pr_err("Factory Couldn't set dc icl = %d rc=%d\n",
		       (int)dc_curr, (int)r);
		return r;
	}

	return r ? r : count;
}

static ssize_t force_chg_idc_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int r;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		r = -ENODEV;
		goto end;
	}

	r = smblib_get_charge_param(mmi_chip, &mmi_chip->param.dc_icl, &state);
	if (r < 0) {
		pr_err("Factory Couldn't get dc_icl rc=%d\n", (int)r);
		return r;
	}
	state /= 1000; /* Convert to mA */
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_idc, 0664,
		force_chg_idc_show,
		force_chg_idc_store);

#define PRE_CHARGE_CURRENT_CFG_REG		(CHGR_BASE + 0x60)
#define PRE_CHARGE_CURRENT_SETTING_MASK		0x07
#define PRE_CHARGE_CONV_MV 50
#define PRE_CHARGE_MAX 7
#define PRE_CHARGE_MIN 100
#define PRE_CHARGE_SMB2_CONV_MV 25
#define PRE_CHARGE_SMB2_MAX 0x3F
static ssize_t force_chg_itrick_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long r;
	unsigned long chg_current;
	u8 value, mask;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		pr_err("Invalid pre-charge value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	switch (mmi_chip->smb_version) {
	case PM8150B_SUBTYPE:
	case PMI632_SUBTYPE:
		mask = PRE_CHARGE_CURRENT_SETTING_MASK;
		if (chg_current < PRE_CHARGE_MIN)
			chg_current = 0;
		else
			chg_current -= PRE_CHARGE_MIN;

		chg_current /= PRE_CHARGE_CONV_MV;

		if (chg_current > PRE_CHARGE_MAX)
			value = PRE_CHARGE_MAX;
		else
			value = (u8)chg_current;
		break;
	case PM660_SUBTYPE:
		mask = PRE_CHARGE_SMB2_MAX;
		chg_current /= PRE_CHARGE_SMB2_CONV_MV;

		if (chg_current > PRE_CHARGE_SMB2_MAX)
			value = PRE_CHARGE_SMB2_MAX;
		else
			value = (u8)chg_current;
		break;
	default:
		pr_err("SMBMMI:Set ITRICK PMIC subtype %d not supported\n",
		       mmi_chip->smb_version);
		return -EINVAL;
	}

	r = smblib_masked_write_mmi(mmi_chip, PRE_CHARGE_CURRENT_CFG_REG,
				    mask,
				    value);
	if (r < 0) {
		pr_err("Factory Couldn't set ITRICK %d  mV rc=%d\n",
			   (int)value, (int)r);
		return r;
	}

	return r ? r : count;
}

static ssize_t force_chg_itrick_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int state;
	int ret;
	u8 value;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_read_mmi(mmi_chip, PRE_CHARGE_CURRENT_CFG_REG, &value);
	if (ret) {
		pr_err("Pre Chg ITrick failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

	switch (mmi_chip->smb_version) {
	case PM8150B_SUBTYPE:
	case PMI632_SUBTYPE:
		value &= PRE_CHARGE_CURRENT_SETTING_MASK;
		state = (value * PRE_CHARGE_CONV_MV) + PRE_CHARGE_MIN;
		break;
	case PM660_SUBTYPE:
		value &= PRE_CHARGE_SMB2_MAX;
		state = value * PRE_CHARGE_SMB2_CONV_MV;
		break;
	default:
		pr_err("SMBMMI:Get ITRICK PMIC subtype %d not supported\n",
		       mmi_chip->smb_version);
		return -EINVAL;
	}
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_itrick, 0664,
		   force_chg_itrick_show,
		   force_chg_itrick_store);

static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

static enum power_supply_property smb_mmi_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smb_mmi_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int smb_mmi_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct smb_mmi_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;
	int override = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval) {
			rc = power_supply_set_property(chip->usb_psy,
					       POWER_SUPPLY_PROP_CTM_CURRENT_MAX,
					       val);
			rc = power_supply_set_property(chip->main_psy,
					       POWER_SUPPLY_PROP_CURRENT_MAX,
					       val);
			override = USBIN_ICL_OVERRIDE_BIT;
		}
		pr_warn("SMBMMI: Request for ICL to %d uA\n", val->intval);
		rc = smblib_masked_write_mmi(chip, USBIN_CMD_ICL_OVERRIDE_REG,
					     USBIN_ICL_OVERRIDE_BIT,
					     override);

		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);
#define TWO_VOLT 2000000
static void mmi_heartbeat_work(struct work_struct *work)
{
	struct smb_mmi_charger *chip = container_of(work,
						struct smb_mmi_charger,
						heartbeat_work.work);
	int hb_resch_time;
	union power_supply_propval pval;
	int rc, usb_suspend, usbin_uv;

	if (chip->factory_mode)
		hb_resch_time = 1000;
	else
		hb_resch_time = 60000;

	if (chip->factory_mode) {
		rc = smblib_get_usb_suspend(chip, &usb_suspend);
		if (rc < 0)
			goto sch_hb;

		rc = power_supply_get_property(chip->usb_psy,
					       POWER_SUPPLY_PROP_VOLTAGE_NOW,
					       &pval);
		if (rc < 0)
			goto sch_hb;
		else
			usbin_uv = pval.intval;

		rc = power_supply_get_property(chip->pc_port_psy,
					       POWER_SUPPLY_PROP_ONLINE,
					       &pval);
		if (rc < 0)
			goto sch_hb;



		if (pval.intval || (usb_suspend && (usbin_uv > TWO_VOLT))) {
			pr_debug("SMBMMI: Factory Kill Armed\n");
			chip->factory_kill_armed = true;
		} else if (chip->factory_kill_armed && !factory_kill_disable) {
			pr_err("SMBMMI:Factory kill power off\n");
			kernel_power_off();
		} else
			chip->factory_kill_armed = false;
	}

sch_hb:
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));

}
static int mmi_psy_notifier_call(struct notifier_block *nb, unsigned long val,
				 void *v)
{
	struct smb_mmi_charger *chip = container_of(nb,
				struct smb_mmi_charger, mmi_psy_notifier);
	struct power_supply *psy = v;

	if (!chip) {
		pr_err("called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (chip->factory_mode &&
	    psy &&
	    (strcmp(psy->desc->name, "usb") == 0)) {
		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

static int smbchg_reboot(struct notifier_block *nb,
			 unsigned long event, void *unused)
{
	struct smb_mmi_charger *chg = container_of(nb, struct smb_mmi_charger,
						smb_reboot);
	union power_supply_propval val;
	int rc;

	pr_err("SMBMMI: Reboot/POFF\n");
	if (!chg) {
		pr_err("called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (chg->factory_mode) {
		switch (event) {
		case SYS_POWER_OFF:
			/* Disable Factory Kill */
			factory_kill_disable = true;
			/* Disable Charging */
			smblib_masked_write_mmi(chg, CHARGING_ENABLE_CMD_REG,
						CHARGING_ENABLE_CMD_BIT,
						0);

			/* Suspend USB and DC */
			smblib_set_usb_suspend(chg, true);
			rc = power_supply_get_property(chg->usb_psy,
						 POWER_SUPPLY_PROP_VOLTAGE_NOW,
						 &val);
			while ((rc >= 0) && (val.intval > TWO_VOLT)) {
				msleep(100);
				rc = power_supply_get_property(
						chg->usb_psy,
						 POWER_SUPPLY_PROP_VOLTAGE_NOW,
						 &val);
				pr_err("Wait for VBUS to decay\n");
			}

			pr_err("VBUS UV wait 1 sec!\n");
			/* Delay 1 sec to allow more VBUS decay */
			msleep(1000);
			break;
		default:
			break;
		}
	}

	return NOTIFY_DONE;
}

static const struct power_supply_desc mmi_psy_desc = {
	.name		= "mmi_battery",
	.type		= POWER_SUPPLY_TYPE_MAIN,
	.get_property	= smb_mmi_get_property,
	.set_property	= smb_mmi_set_property,
	.properties	= smb_mmi_battery_props,
	.num_properties	= ARRAY_SIZE(smb_mmi_battery_props),
};

static int smb_mmi_chg_config_init(struct smb_mmi_charger *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(chip->dev->of_node,
					  "qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR_OR_NULL(pmic_rev_id)) {
		/*
		 * the revid peripheral must be registered, any failure
		 * here only indicates that the rev-id module has not
		 * probed yet.
		 */
		return -EPROBE_DEFER;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PM8150B_SUBTYPE:
		chip->smb_version = PM8150B_SUBTYPE;
		chip->name = "pm8150b_charger";
		chip->param = smb5_pm8150b_params;
		break;
	case PMI632_SUBTYPE:
		chip->smb_version = PMI632_SUBTYPE;
		chip->name = "pmi632_charger";
		chip->param = smb5_pmi632_params;
		break;
	case PM660_SUBTYPE:
		chip->smb_version = PM660_SUBTYPE;
		chip->name = "pm660_charger";
		chip->param = smb2_pm660_params;
		break;
	default:
		pr_err("PMIC subtype %d not supported\n",
				pmic_rev_id->pmic_subtype);
		return -EINVAL;
	}

	pr_err("SMBMMI: PMIC %d is %s\n", chip->smb_version, chip->name);

	return 0;
}

static int smb_mmi_probe(struct platform_device *pdev)
{
	struct smb_mmi_charger *chip;
	int rc = 0;
	union power_supply_propval val;
	struct power_supply_config psy_cfg = {};

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	psy_cfg.drv_data = chip;
	platform_set_drvdata(pdev, chip);

	smb_mmi_chg_config_init(chip);

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_heartbeat_work);

	chip->mmi_psy = devm_power_supply_register(chip->dev, &mmi_psy_desc,
						   &psy_cfg);
	if (IS_ERR(chip->mmi_psy)) {
		dev_err(chip->dev, "failed: mmi power supply register\n");
		return PTR_ERR(chip->mmi_psy);
	}

	chip->batt_psy = power_supply_get_by_name("battery");
	chip->bms_psy = power_supply_get_by_name("bms");
	chip->usb_psy = power_supply_get_by_name("usb");
	chip->main_psy = power_supply_get_by_name("main");
	chip->pc_port_psy = power_supply_get_by_name("pc_port");

	chip->chg_dis_votable = find_votable("CHG_DISABLE");
	if (IS_ERR(chip->chg_dis_votable))
		chip->chg_dis_votable = NULL;
	chip->fcc_votable = find_votable("FCC");
	if (IS_ERR(chip->fcc_votable))
		chip->fcc_votable = NULL;
	chip->fv_votable = find_votable("FV");
	if (IS_ERR(chip->fv_votable))
		chip->fv_votable = NULL;
	chip->usb_icl_votable = find_votable("USB_ICL");
	if (IS_ERR(chip->usb_icl_votable))
		chip->usb_icl_votable = NULL;

	if (chip->smb_version == PM8150B_SUBTYPE)
		if (smblib_masked_write_mmi(chip, LEGACY_CABLE_CFG_REG,
					    0xFF, 0))
			pr_err("Could Not set Legacy Cable CFG\n");

	if ((chip->smb_version == PM8150B_SUBTYPE) ||
	    (chip->smb_version == PM660_SUBTYPE))
		if (smblib_masked_write_mmi(chip, USBIN_ADAPTER_ALLOW_CFG_REG,
					    USBIN_ADAPTER_ALLOW_MASK,
					    USBIN_ADAPTER_ALLOW_5V_TO_9V))
			pr_err("Could Not set USB Adapter CFG\n");

	chip->factory_mode = mmi_factory_check();

	rc = device_create_file(chip->dev,
				&dev_attr_force_demo_mode);
	if (rc) {
		dev_err(chip->dev, "couldn't create force_demo_mode\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_force_max_chrg_temp);
	if (rc) {
		dev_err(chip->dev, "couldn't create force_max_chrg_temp\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_factory_image_mode);
	if (rc)
		dev_err(chip->dev, "couldn't create factory_image_mode\n");

	rc = device_create_file(chip->dev,
				&dev_attr_factory_charge_upper);
	if (rc)
		dev_err(chip->dev, "couldn't create factory_charge_upper\n");

	/* Register the notifier for the psy updates*/
	chip->mmi_psy_notifier.notifier_call = mmi_psy_notifier_call;
	rc = power_supply_reg_notifier(&chip->mmi_psy_notifier);
	if (rc)
		pr_err("SMBMMI: failed to reg notifier: %d\n", rc);

	if (chip->factory_mode) {
		dev_err(chip->dev, "Entering Factory Mode SMB!\n");
		rc = smblib_masked_write_mmi(chip, USBIN_ICL_OPTIONS_REG,
					     USBIN_MODE_CHG_BIT,
					     USBIN_MODE_CHG_BIT);
		if (rc < 0)
			pr_err("Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);

		rc = smblib_masked_write_mmi(chip, USBIN_LOAD_CFG_REG,
					     ICL_OVERRIDE_AFTER_APSD_BIT,
					     ICL_OVERRIDE_AFTER_APSD_BIT);
		if (rc < 0)
			pr_err("Couldn't set USBIN_LOAD_CFG rc=%d\n", rc);

		rc = smblib_masked_write_mmi(chip, USBIN_AICL_OPTIONS_CFG_REG,
					     0xFF, 0x00);
		if (rc < 0)
			pr_err("Couldn't set USBIN_AICL_OPTIONS rc=%d\n", rc);

		chip->smb_reboot.notifier_call = smbchg_reboot;
		chip->smb_reboot.next = NULL;
		chip->smb_reboot.priority = 1;
		rc = register_reboot_notifier(&chip->smb_reboot);
		if (rc)
			pr_err("SMBMMI: register for reboot failed\n");
		rc = power_supply_get_property(chip->pc_port_psy,
					       POWER_SUPPLY_PROP_ONLINE,
					       &val);
		if (rc >= 0 && val.intval) {
			pr_err("SMBMMI: Factory Kill Armed\n");
			chip->factory_kill_armed = true;
		}

		if(chip->chg_dis_votable) {
			pmic_vote_force_val_set(chip->chg_dis_votable, 1);
			pmic_vote_force_active_set(chip->chg_dis_votable, 1);
		}
		if(chip->fcc_votable) {
			pmic_vote_force_val_set(chip->fcc_votable, 3000000);
			pmic_vote_force_active_set(chip->fcc_votable, 1);
		}
		if(chip->fv_votable) {
			pmic_vote_force_val_set(chip->fv_votable, 4400000);
			pmic_vote_force_active_set(chip->fv_votable, 1);
		}
		if(chip->usb_icl_votable) {
			pmic_vote_force_val_set(chip->usb_icl_votable,
						3000000);
			pmic_vote_force_active_set(chip->usb_icl_votable, 1);
		}

		/* Some Cables need a more forced approach */
		rc = smblib_set_charge_param(chip, &chip->param.usb_icl,
					     3000000);
		if (rc < 0)
			pr_err("Factory Couldn't set usb icl = 3000 rc=%d\n",
			       (int)rc);

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_usb_suspend);
		if (rc) {
			dev_err(chip->dev,
				   "couldn't create force_chg_usb_suspend\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_fail_clear);
		if (rc) {
			dev_err(chip->dev,
				   "couldn't create force_chg_fail_clear\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_auto_enable);
		if (rc) {
			dev_err(chip->dev,
				   "couldn't create force_chg_auto_enable\n");
		}

		rc = device_create_file(chip->dev,
				&dev_attr_force_chg_ibatt);
		if (rc) {
			dev_err(chip->dev,
				"couldn't create force_chg_ibatt\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_iusb);
		if (rc) {
			dev_err(chip->dev,
				"couldn't create force_chg_iusb\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_idc);
		if (rc) {
			dev_err(chip->dev, "couldn't create force_chg_idc\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_itrick);
		if (rc) {
			dev_err(chip->dev, "couldn't create force_chg_itrick\n");
		}

	}

	pr_info("QPNP SMB MMI probed successfully!\n");

	return rc;
}

static void smb_mmi_shutdown(struct platform_device *pdev)
{
	return;
}

#ifdef CONFIG_PM_SLEEP
static int smb_mmi_suspend(struct device *device)
{
	return 0;
}

static int smb_mmi_resume(struct device *device)
{
	return 0;
}
#else
#define smb2_suspend NULL
#define smb2_resume NULL
#endif

static const struct dev_pm_ops smb_mmi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(smb_mmi_suspend, smb_mmi_resume)
};

static const struct of_device_id match_table[] = {
	{ .compatible = "qcom,qpnp-smbcharger-mmi", },
	{ },
};

static struct platform_driver smb_mmi_driver = {
	.driver		= {
		.name		= "qcom,qpnp-smbcharger-mmi",
		.owner		= THIS_MODULE,
		.pm		= &smb_mmi_dev_pm_ops,
		.of_match_table	= match_table,
	},
	.probe		= smb_mmi_probe,
	.shutdown	= smb_mmi_shutdown,
};
module_platform_driver(smb_mmi_driver);

MODULE_DESCRIPTION("QPNP SMB MMI Charger Driver");
MODULE_LICENSE("GPL v2");
