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
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
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

#define BATTERY_CHARGER_STATUS_1_REG		(CHGR_BASE + 0x06)
#define BATTERY_CHARGER_STATUS_MASK		(GENMASK(2, 0))
#define PM8150B_JEITA_EN_CFG_REG		(CHGR_BASE + 0x90)

#define USBIN_INT_RT_STS			(USBIN_BASE + 0x10)
#define USBIN_PLUGIN_RT_STS_BIT			BIT(4)
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

#define REV_BST_THRESH 4700
#define REV_BST_DROP 150
#define REV_BST_BULK_DROP 100
#define REV_BST_MA -10
#define BOOST_BACK_VOTER		"BOOST_BACK_VOTER"
#define MMI_HB_VOTER			"MMI_HB_VOTER"
#define BATT_PROFILE_VOTER		"BATT_PROFILE_VOTER"
#define HYST_STEP_MV 50
#define HYST_STEP_FLIP_MV (HYST_STEP_MV*2)
#define DEMO_MODE_HYS_SOC 5
#define DEMO_MODE_VOLTAGE 4000
#define WARM_TEMP 45
#define COOL_TEMP 0

#define HEARTBEAT_DELAY_MS 60000
#define HEARTBEAT_DUAL_DELAY_MS 10000
#define HEARTBEAT_FACTORY_MS 1000

enum {
	TRICKLE_CHARGE = 0,
	PRE_CHARGE,
	FAST_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	INHIBIT_CHARGE,
	DISABLE_CHARGE,
};

enum {
	INHIBIT_CHARGE_V5 = 0,
	TRICKLE_CHARGE_V5,
	PRE_CHARGE_V5,
	FAST_CHARGE_V5,
	FULLON_CHARGE_V5,
	TAPER_CHARGE_V5,
	TERMINATE_CHARGE_V5,
	PAUSE_CHARGE_V5,
	DISABLE_CHARGE_V5,
};

enum {
	POWER_SUPPLY_CHARGE_RATE_NONE = 0,
	POWER_SUPPLY_CHARGE_RATE_NORMAL,
	POWER_SUPPLY_CHARGE_RATE_WEAK,
	POWER_SUPPLY_CHARGE_RATE_TURBO,
};

static char *charge_rate[] = {
	"None", "Normal", "Weak", "Turbo"
};

struct mmi_temp_zone {
	int		temp_c;
	int		norm_mv;
	int		fcc_max_ma;
	int		fcc_norm_ma;
};

#define MAX_NUM_STEPS 10
enum mmi_temp_zones {
	ZONE_FIRST = 0,
	/* states 0-9 are reserved for zones */
	ZONE_LAST = MAX_NUM_STEPS + ZONE_FIRST - 1,
	ZONE_HOT,
	ZONE_COLD,
	ZONE_NONE = 0xFF,
};

enum {
	BASE_BATT = 0,
	MAIN_BATT,
	FLIP_BATT,
};

enum mmi_chrg_step {
	STEP_MAX,
	STEP_NORM,
	STEP_FULL,
	STEP_FLOAT,
	STEP_DEMO,
	STEP_STOP,
	STEP_NONE = 0xFF,
};

static char *stepchg_str[] = {
	[STEP_MAX]		= "MAX",
	[STEP_NORM]		= "NORMAL",
	[STEP_FULL]		= "FULL",
	[STEP_FLOAT]		= "FLOAT",
	[STEP_DEMO]		= "DEMO",
	[STEP_STOP]		= "STOP",
	[STEP_NONE]		= "NONE",
};

struct smb_mmi_chg_status {
	int batt_mv;
	int batt_ma;
	int batt_soc;
	int batt_temp;
	int usb_mv;
	int charger_present;
	int vbus_present;
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

struct mmi_sm_params {
	int			num_temp_zones;
	struct mmi_temp_zone	*temp_zones;
	enum mmi_temp_zones	pres_temp_zone;
	enum mmi_chrg_step	pres_chrg_step;
	int			chrg_taper_cnt;
	int			batt_health;
	int			chrg_iterm;
	int			target_fcc;
	int			target_fv;
	int			ocp[MAX_NUM_STEPS];
};

enum charging_limit_modes {
	CHARGING_LIMIT_OFF,
	CHARGING_LIMIT_RUN,
	CHARGING_LIMIT_UNKNOWN,
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
	struct power_supply	*qcom_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*main_psy;
	struct power_supply	*pc_port_psy;
	struct power_supply	*max_main_psy;
	struct power_supply	*max_flip_psy;
	struct notifier_block	mmi_psy_notifier;
	struct delayed_work	heartbeat_work;

	struct votable 		*chg_dis_votable;
	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;

	bool			enable_charging_limit;
        bool                    enable_factory_poweroff;
	bool			is_factory_image;
	enum charging_limit_modes	charging_limit_modes;
	int			upper_limit_capacity;
	int			lower_limit_capacity;
	int			max_chrg_temp;
	int			last_iusb_ua;
	bool			factory_kill_armed;
	int			gen_log_rate_s;

	/* Charge Profile */
	struct mmi_sm_params	sm_param[3];
	int			base_fv_mv;
	int			vfloat_comp_mv;
	struct wakeup_source	smb_mmi_hb_wake_source;
	struct alarm		heartbeat_alarm;
	bool			suspended;
	bool			awake;
	int 			last_reported_soc;
	int 			last_reported_status;
	struct regulator	*vbus;
	bool			vbus_enabled;
	int 			charger_rate;
	int 			age;
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

#define MMI_CHIP_MODE_LOWER_LIMIT 35
#define MMI_CHIP_MODE_UPPER_LIMIT 80
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

	if ((mode >= MMI_CHIP_MODE_LOWER_LIMIT) &&
	    (mode <= MMI_CHIP_MODE_UPPER_LIMIT))
		mmi_chip->demo_mode = mode;
	else
		mmi_chip->demo_mode = MMI_CHIP_MODE_LOWER_LIMIT;

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
#define HYSTERESIS_DEGC 2
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
		state = -ENODEV;
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
		state = -ENODEV;
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
	POWER_SUPPLY_PROP_UPDATE_NOW,
	POWER_SUPPLY_PROP_USB_OTG,
};

static int smb_mmi_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct smb_mmi_charger *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = -EINVAL;
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		val->intval = chip->gen_log_rate_s;
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		if (chip->vbus &&
		    regulator_is_enabled(chip->vbus) &&
		    chip->vbus_enabled)
			val->intval = 1;
		else
			val->intval = 0;
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
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		chip->gen_log_rate_s = val->intval;
		power_supply_changed(psy);
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		if (!chip->vbus) {
			chip->vbus = devm_regulator_get(chip->dev, "vbus");
			if (IS_ERR(chip->vbus)) {
				pr_err("SMBMMI: Unable to get vbus\n");
				return -EINVAL;
			}
		}

		if (val->intval) {
			rc = regulator_enable(chip->vbus);
			pr_info("SMBMMI: VBUS Enable\n");
		} else if (chip->vbus_enabled) {
			rc = regulator_disable(chip->vbus);
			pr_info("SMBMMI: VBUS Disable\n");
		}

		if (rc)
			pr_err("SMBMMI: Unable to %s vbus (%d)\n",
			       val->intval ? "enable" : "disable", rc);
		else if (val->intval)
			chip->vbus_enabled = true;
		else
			chip->vbus_enabled = false;

		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

static bool mmi_find_temp_zone(struct smb_mmi_charger *chg,
			       struct mmi_sm_params *chip,
			       int temp_c)
{
	int prev_zone, num_zones;
	struct mmi_temp_zone *zones;
	int hotter_t, hotter_fcc;
	int colder_t, colder_fcc;
	int i;
	int max_temp;

	if (!chg) {
		pr_debug("called before chg valid!\n");
		return false;
	}

	zones = chip->temp_zones;
	num_zones = chip->num_temp_zones;
	prev_zone = chip->pres_temp_zone;

	if (chg->max_chrg_temp >= MIN_MAX_TEMP_C)
		max_temp = chg->max_chrg_temp;
	else
		max_temp = zones[num_zones - 1].temp_c;

	if (prev_zone == ZONE_NONE) {
		for (i = num_zones - 1; i >= 0; i--) {
			if (temp_c >= zones[i].temp_c) {
				if (i == num_zones - 1)
					chip->pres_temp_zone = ZONE_HOT;
				else
					chip->pres_temp_zone = i + 1;
				return true;
			}
		}
		chip->pres_temp_zone = ZONE_COLD;
		return true;
	}

	if (prev_zone == ZONE_COLD) {
		if (temp_c >= MIN_TEMP_C + HYSTERESIS_DEGC)
			chip->pres_temp_zone = ZONE_FIRST;
	} else if (prev_zone == ZONE_HOT) {
		if (temp_c <=  max_temp - HYSTERESIS_DEGC)
			chip->pres_temp_zone = num_zones - 1;
	} else {
		if (prev_zone == ZONE_FIRST) {
			hotter_fcc = zones[prev_zone + 1].fcc_max_ma;
			colder_fcc = 0;
			hotter_t = zones[prev_zone].temp_c;
			colder_t = MIN_TEMP_C;
		} else if (prev_zone == num_zones - 1) {
			hotter_fcc = 0;
			colder_fcc = zones[prev_zone - 1].fcc_max_ma;
			hotter_t = zones[prev_zone].temp_c;
			colder_t = zones[prev_zone - 1].temp_c;
		} else {
			hotter_fcc = zones[prev_zone + 1].fcc_max_ma;
			colder_fcc = zones[prev_zone - 1].fcc_max_ma;
			hotter_t = zones[prev_zone].temp_c;
			colder_t = zones[prev_zone - 1].temp_c;
		}

		if (zones[prev_zone].fcc_max_ma < hotter_fcc)
			hotter_t += HYSTERESIS_DEGC;

		if (zones[prev_zone].fcc_max_ma < colder_fcc)
			colder_t -= HYSTERESIS_DEGC;

		if (temp_c < MIN_TEMP_C)
			chip->pres_temp_zone = ZONE_COLD;
		else if (temp_c >= max_temp)
			chip->pres_temp_zone = ZONE_HOT;
		else if (temp_c >= hotter_t)
			chip->pres_temp_zone++;
		else if (temp_c < colder_t)
			chip->pres_temp_zone--;
	}

	if (prev_zone != chip->pres_temp_zone) {
		pr_debug("Entered Temp Zone %d!\n",
			   chip->pres_temp_zone);
		return true;
	}

	return false;
}

static int get_prop_batt_voltage_now(struct smb_mmi_charger *chg,
				     struct power_supply *psy,
				     union power_supply_propval *val)
{
	int rc;

	if (!psy)
		return -EINVAL;

	rc = power_supply_get_property(psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

static int get_prop_batt_current_now(struct smb_mmi_charger *chg,
				     struct power_supply *psy,
				     union power_supply_propval *val)
{
	int rc;

	if (!psy)
		return -EINVAL;

	rc = power_supply_get_property(psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

static int get_prop_usb_voltage_now(struct smb_mmi_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->usb_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

static int get_prop_batt_temp(struct smb_mmi_charger *chg,
			      struct power_supply *psy,
			      union power_supply_propval *val)
{
	int rc;

	if (!psy)
		return -EINVAL;

	rc = power_supply_get_property(psy,
				       POWER_SUPPLY_PROP_TEMP, val);
	return rc;
}

static int get_prop_batt_capacity(struct smb_mmi_charger *chg,
				  struct power_supply *psy,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (psy)
		rc = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
	return rc;
}

static int get_prop_usb_present(struct smb_mmi_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read_mmi(chg, USBIN_INT_RT_STS, &stat);
	if (rc < 0) {
		pr_err("Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	return 0;
}

static int get_prop_charger_present(struct smb_mmi_charger *chg,
				    union power_supply_propval *val)
{
	int rc = -EINVAL;

	val->intval = 0;

	if (chg->usb_psy)
		rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_MODE, val);
	if (rc < 0) {
		pr_err("Couldn't read TypeC Mode rc=%d\n", rc);
		return rc;
	}

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		val->intval = 1;
		break;
	default:
		break;
	}

	return rc;
}

#define WEAK_CHRG_THRSH 450
#define TURBO_CHRG_THRSH 2500
void mmi_chrg_rate_check(struct smb_mmi_charger *chg)
{
	union power_supply_propval val;
	int chrg_cm_ma = 0;
	int chrg_cs_ma = 0;
	int prev_chg_rate = chg->charger_rate;
	int rc = -EINVAL;

	if (!chg->usb_psy) {
		pr_err("No usb PSY\n");
		return;
	}

	val.intval = 0;
	rc = get_prop_charger_present(chg, &val);
	if (rc < 0) {
		pr_err("Error getting Charger Present rc = %d\n", rc);
		return;
	}

	if (val.intval) {
		val.intval = 0;
		rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_HW_CURRENT_MAX, &val);
		if (rc < 0) {
			pr_err("Error getting HW Current Max rc = %d\n", rc);
			return;
		}
		chrg_cm_ma = val.intval / 1000;

		val.intval = 0;
		rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &val);
		if (rc < 0) {
			pr_err("Error getting ICL Settled rc = %d\n", rc);
			return;
		}
		chrg_cs_ma = val.intval / 1000;
	} else {
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NONE;
		goto end_rate_check;
	}

	pr_debug("SMBMMI: cm %d, cs %d\n", chrg_cm_ma, chrg_cs_ma);
	if (chrg_cm_ma >= TURBO_CHRG_THRSH)
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_TURBO;
	else if ((chrg_cm_ma > WEAK_CHRG_THRSH) && (chrg_cs_ma < WEAK_CHRG_THRSH))
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_WEAK;
	else if (prev_chg_rate == POWER_SUPPLY_CHARGE_RATE_NONE)
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;

end_rate_check:
	if (prev_chg_rate != chg->charger_rate)
		pr_err("%s Charger Detected\n",
		       charge_rate[chg->charger_rate]);

}

#define TAPER_COUNT 2
#define TAPER_DROP_MA 100
static bool mmi_has_current_tapered(struct smb_mmi_charger *chg,
				    struct mmi_sm_params *chip,
				    int batt_ma, int taper_ma)
{
	bool change_state = false;
	int allowed_fcc, target_ma;

	if (!chg) {
		pr_debug("called before chip valid!\n");
		return false;
	}

	allowed_fcc = get_effective_result(chg->fcc_votable) / 1000;

	if (allowed_fcc >= taper_ma)
		target_ma = taper_ma;
	else
		target_ma = allowed_fcc - TAPER_DROP_MA;

	if (batt_ma < 0) {
		batt_ma *= -1;
		if (batt_ma <= target_ma)
			if (chip->chrg_taper_cnt >= TAPER_COUNT) {
				change_state = true;
				chip->chrg_taper_cnt = 0;
			} else
				chip->chrg_taper_cnt++;
		else
			chip->chrg_taper_cnt = 0;
	} else {
		if (chip->chrg_taper_cnt >= TAPER_COUNT) {
			change_state = true;
			chip->chrg_taper_cnt = 0;
		} else
			chip->chrg_taper_cnt++;
	}

	return change_state;
}

bool mmi_charge_halted(struct smb_mmi_charger *chg)
{
	u8 stat;
	int rc;
	bool flag = false;

	rc = smblib_read_mmi(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return flag;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (chg->smb_version == PM8150B_SUBTYPE) {
		switch (stat) {
		case TERMINATE_CHARGE_V5:
		case INHIBIT_CHARGE_V5:
			flag = true;
			break;
		default:
			break;
		}
	} else {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			flag = true;
			break;
		default:
			break;
		}
	}

	return flag;
}

static enum alarmtimer_restart mmi_heartbeat_alarm_cb(struct alarm *alarm,
						      ktime_t now)
{
	struct smb_mmi_charger *chip = container_of(alarm,
						    struct smb_mmi_charger,
						    heartbeat_alarm);

	pr_err("SMBMMI: HB alarm fired\n");

	__pm_stay_awake(&chip->smb_mmi_hb_wake_source);
	cancel_delayed_work(&chip->heartbeat_work);
	/* Delay by 500 ms to allow devices to resume. */
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(500));

	return ALARMTIMER_NORESTART;
}

static int mmi_dual_charge_sm(struct smb_mmi_charger *chg,
			      struct smb_mmi_chg_status *stat,
			      int batt, int fv_offset)
{
	int max_fv_mv;
	int i;
	struct mmi_temp_zone *zone;
	struct mmi_sm_params *chip = &chg->sm_param[batt];
	int start_tz = chip->pres_temp_zone;
	int start_step = chip->pres_chrg_step;

	if (!chip->temp_zones) {
		pr_debug("SMBMMI: No Temp Zone Defined for batt %d!\n", batt);
		return -ENODEV;
	}

	if (chg->base_fv_mv == 0) {
		chg->base_fv_mv = get_effective_result(chg->fv_votable);
		chg->base_fv_mv /= 1000;
		vote(chg->fv_votable,
		     BATT_PROFILE_VOTER, false, 0);
	}
	max_fv_mv = chg->base_fv_mv;

	mmi_find_temp_zone(chg, chip, stat->batt_temp);
	zone = &chip->temp_zones[chip->pres_temp_zone];

	if (!stat->charger_present) {
		chip->pres_chrg_step = STEP_NONE;
		for (i = 0; i < MAX_NUM_STEPS; i++)
			chip->ocp[i] = 0;
	} else if ((chip->pres_temp_zone == ZONE_HOT) ||
		   (chip->pres_temp_zone == ZONE_COLD) ||
		   (chg->charging_limit_modes == CHARGING_LIMIT_RUN)) {
		chip->pres_chrg_step = STEP_STOP;
	} else if (chg->demo_mode) {
		/* TODO: Ignore Demo Mode for now */
	} else if ((chip->pres_chrg_step == STEP_NONE) ||
		   (chip->pres_chrg_step == STEP_STOP)) {
		if (zone->norm_mv && (stat->batt_mv >= zone->norm_mv)) {
			if (zone->fcc_norm_ma)
				chip->pres_chrg_step = STEP_NORM;
			else
				chip->pres_chrg_step = STEP_STOP;
		} else
			chip->pres_chrg_step = STEP_MAX;
	} else if (chip->pres_chrg_step == STEP_MAX) {
		if (!zone->norm_mv) {
			/* No Step in this Zone */
			chip->chrg_taper_cnt = 0;
			if ((stat->batt_mv + fv_offset) >= max_fv_mv)
				chip->pres_chrg_step = STEP_NORM;
			else
				chip->pres_chrg_step = STEP_MAX;
		} else if ((stat->batt_mv + fv_offset) < zone->norm_mv) {
			chip->chrg_taper_cnt = 0;
			chip->pres_chrg_step = STEP_MAX;
		} else if (!zone->fcc_norm_ma)
			chip->pres_chrg_step = STEP_FLOAT;
		else if (mmi_has_current_tapered(chg, chip, stat->batt_ma,
						 zone->fcc_norm_ma)) {
			chip->chrg_taper_cnt = 0;
			for (i = 0; i < MAX_NUM_STEPS; i++)
				chip->ocp[i] = 0;
			chip->pres_chrg_step = STEP_NORM;
		}
	} else if (chip->pres_chrg_step == STEP_NORM) {
		if (!zone->fcc_norm_ma)
			chip->pres_chrg_step = STEP_STOP;
		else if ((stat->batt_soc < 100) ||
			 (stat->batt_mv + fv_offset) < max_fv_mv) {
			chip->chrg_taper_cnt = 0;
			chip->pres_chrg_step = STEP_NORM;
		} else if (mmi_has_current_tapered(chg, chip, stat->batt_ma,
						   chip->chrg_iterm)) {
				chip->pres_chrg_step = STEP_FULL;
		}
	} else if (chip->pres_chrg_step == STEP_FULL) {
		if (stat->batt_soc <= 99) {
			chip->chrg_taper_cnt = 0;
			chip->pres_chrg_step = STEP_NORM;
		}
	} else if (chip->pres_chrg_step == STEP_FLOAT) {
		if ((zone->fcc_norm_ma) ||
		    ((stat->batt_mv + fv_offset) < zone->norm_mv))
			chip->pres_chrg_step = STEP_MAX;
	}

	/* Take State actions */
	switch (chip->pres_chrg_step) {
	case STEP_FLOAT:
	case STEP_MAX:
		if (!zone->norm_mv)
			chip->target_fv = max_fv_mv;
		else
			chip->target_fv = zone->norm_mv;
		chip->target_fcc = zone->fcc_max_ma;
		break;
	case STEP_FULL:
		chip->target_fv = max_fv_mv;
		chip->target_fcc = -EINVAL;
		break;
	case STEP_NORM:
		chip->target_fv = max_fv_mv + chg->vfloat_comp_mv;
		chip->target_fcc = zone->fcc_norm_ma;
		break;
	case STEP_NONE:
		chip->target_fv = max_fv_mv;
		chip->target_fcc = zone->fcc_norm_ma;
		break;
	case STEP_STOP:
		chip->target_fv = max_fv_mv;
		chip->target_fcc = -EINVAL;
		break;
	case STEP_DEMO:
		chip->target_fv = DEMO_MODE_VOLTAGE;
		chip->target_fcc = zone->fcc_max_ma;
		break;
	default:
		chip->target_fv = max_fv_mv;
		chip->target_fcc = zone->fcc_norm_ma;
		break;
	}

	if (chip->pres_temp_zone == ZONE_HOT) {
		chip->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (chip->pres_temp_zone == ZONE_COLD) {
		chip->batt_health = POWER_SUPPLY_HEALTH_COLD;
	} else if (stat->batt_temp >= WARM_TEMP) {
		if (chip->pres_chrg_step == STEP_STOP)
			chip->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (stat->batt_temp <= COOL_TEMP) {
		if (chip->pres_chrg_step == STEP_STOP)
			chip->batt_health = POWER_SUPPLY_HEALTH_COLD;
		else
			chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else
		chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;

	if ((start_tz != chip->pres_temp_zone) ||
	    (start_step != chip->pres_chrg_step)) {
		pr_info("SMB_MMI:"
			"Batt %d: batt_mv = %d, batt_ma %d, batt_soc %d,"
			" batt_temp %d, usb_mv %d, cp %d, vp %d\n",
			batt,
			stat->batt_mv,
			stat->batt_ma,
			stat->batt_soc,
			stat->batt_temp,
			stat->usb_mv,
			stat->charger_present,
			stat->vbus_present);
		pr_info("SMBMMI:"
			"Batt %d Step State = %s, Temp Zone %d, Health %d\n",
			batt,
			stepchg_str[(int)chip->pres_chrg_step],
			chip->pres_temp_zone,
			chip->batt_health);
		return 1;
	} else {
		pr_debug("SMB_MMI:"
			 "Batt %d: batt_mv = %d, batt_ma %d, batt_soc %d,"
			 " batt_temp %d, usb_mv %d, cp %d, vp %d\n",
			 batt,
			 stat->batt_mv,
			 stat->batt_ma,
			 stat->batt_soc,
			 stat->batt_temp,
			 stat->usb_mv,
			 stat->charger_present,
			 stat->vbus_present);
		pr_debug("SMBMMI:"
			 "Batt %d Step State = %s, Temp Zone %d, Health %d\n",
			 batt,
			 stepchg_str[(int)chip->pres_chrg_step],
			 chip->pres_temp_zone,
			 chip->batt_health);
	}

	return 0;
}

static int mmi_dual_charge_control(struct smb_mmi_charger *chg,
				    struct smb_mmi_chg_status *stat)
{
	int rc;
	int target_fcc;
	int target_fv;
	int effective_fv;
	int effective_fcc;
	int ocp;
	int sm_update;
	int sched_time = HEARTBEAT_DUAL_DELAY_MS;
	struct smb_mmi_chg_status chg_stat_main, chg_stat_flip;
	union power_supply_propval pval;
	struct mmi_sm_params *main_p = &chg->sm_param[MAIN_BATT];
	struct mmi_sm_params *flip_p = &chg->sm_param[FLIP_BATT];

	chg_stat_main.charger_present = stat->charger_present;
	chg_stat_flip.charger_present = stat->charger_present;
	chg_stat_main.vbus_present = stat->vbus_present;
	chg_stat_flip.vbus_present = stat->vbus_present;
	chg_stat_main.usb_mv = stat->usb_mv;
	chg_stat_flip.usb_mv = stat->usb_mv;

	rc = get_prop_batt_voltage_now(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Main Batt Voltage rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_mv = pval.intval / 1000;

	rc = get_prop_batt_voltage_now(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Flip Batt Voltage rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_mv = pval.intval / 1000;

	rc = get_prop_batt_current_now(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Main Batt Current rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_ma = (pval.intval / 1000) * -1;

	rc = get_prop_batt_current_now(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Flip Batt Current rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_ma = (pval.intval / 1000) * -1;

	rc = get_prop_batt_capacity(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Main Batt Capacity rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_soc = pval.intval;

	rc = get_prop_batt_capacity(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Flip Batt Capacity rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_soc = pval.intval;

	rc = get_prop_batt_temp(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Main Batt Temperature rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_temp = pval.intval / 10;

	rc = get_prop_batt_temp(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Flip Batt Temperature rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_temp = pval.intval / 10;

	sm_update = mmi_dual_charge_sm(chg, &chg_stat_main,
				       MAIN_BATT, HYST_STEP_MV);
	if (sm_update < 0)
		return sched_time;

	sm_update = mmi_dual_charge_sm(chg, &chg_stat_flip,
				       FLIP_BATT, HYST_STEP_FLIP_MV);
	if (sm_update < 0)
		return sched_time;

	effective_fv = get_effective_result(chg->fv_votable) / 1000;
	effective_fcc = get_effective_result(chg->fcc_votable);

	/* Check for Charge None */
	if ((main_p->pres_chrg_step == STEP_NONE) ||
	    (flip_p->pres_chrg_step == STEP_NONE)) {
		target_fcc = main_p->target_fcc;
		target_fv = chg->base_fv_mv;
		sched_time = HEARTBEAT_DELAY_MS;
		goto vote_now;
	/* Check for Charge FULL from each */
	} else if ((main_p->pres_chrg_step == STEP_FULL) &&
		   (flip_p->pres_chrg_step == STEP_FULL)) {
		target_fcc = -EINVAL;
		target_fv = chg->base_fv_mv;
		sched_time = HEARTBEAT_DELAY_MS;
		chg->last_reported_status = POWER_SUPPLY_STATUS_FULL;
		goto vote_now;
	/* Align FULL between batteries */
	} else if ((main_p->pres_chrg_step == STEP_FULL) &&
		   ((flip_p->pres_chrg_step == STEP_MAX) ||
		    (flip_p->pres_chrg_step == STEP_NORM)) &&
		   (chg_stat_flip.batt_soc >= 95) &&
		   ((chg_stat_flip.batt_mv + HYST_STEP_FLIP_MV) >=
		    chg->base_fv_mv)) {
		target_fcc = flip_p->target_fcc;
		target_fv = flip_p->target_fv;
		pr_info("SMBMMI: Align Flip to Main FULL\n");
		goto vote_now;
	} else if ((flip_p->pres_chrg_step == STEP_FULL) &&
		   ((main_p->pres_chrg_step == STEP_MAX) ||
		    (main_p->pres_chrg_step == STEP_NORM)) &&
		   (chg_stat_main.batt_soc >= 95) &&
		   ((chg_stat_main.batt_mv + HYST_STEP_MV) >=
		    chg->base_fv_mv)) {
		target_fcc = main_p->target_fcc;
		target_fv = main_p->target_fv;
		pr_info("SMBMMI: Align Main to Flip FULL\n");
		goto vote_now;
	/* Check for Charge Disable from each */
	} else if ((main_p->target_fcc < 0) ||
		   (flip_p->target_fcc < 0)) {
		target_fcc = -EINVAL;
		target_fv = chg->base_fv_mv;
		goto vote_now;
	}

	chg->last_reported_status = -1;
	if (main_p->target_fv < flip_p->target_fv)
		target_fv = main_p->target_fv;
	else
		target_fv = flip_p->target_fv;

	if (chg_stat_main.batt_ma < 0) {
		chg_stat_main.batt_ma *= -1;
		if (chg_stat_main.batt_ma > main_p->target_fcc) {
			ocp = chg_stat_main.batt_ma - main_p->target_fcc;
			main_p->ocp[main_p->pres_temp_zone] += ocp;
			pr_info("SMBMMI: Main Exceed by %d mA\n",
				main_p->ocp[main_p->pres_temp_zone]);
		}
	}

	if (chg_stat_flip.batt_ma < 0) {
		chg_stat_flip.batt_ma *= -1;
		if (chg_stat_flip.batt_ma > flip_p->target_fcc) {
			ocp = chg_stat_flip.batt_ma - flip_p->target_fcc;
			flip_p->ocp[flip_p->pres_temp_zone] += ocp;
			pr_info("SMBMMI: Flip Exceed by %d mA\n",
				flip_p->ocp[flip_p->pres_temp_zone]);
		}
	}

	target_fcc = main_p->target_fcc + flip_p->target_fcc;
	target_fcc -= main_p->ocp[main_p->pres_temp_zone];
	target_fcc -= flip_p->ocp[flip_p->pres_temp_zone];
	if (target_fcc < main_p->target_fcc) {
		pr_info("SMBMMI: Target FCC adjust too much\n");
		target_fcc = main_p->target_fcc;
	}

	if (((main_p->pres_chrg_step == STEP_MAX) ||
	     (flip_p->pres_chrg_step == STEP_MAX) ||
	     (main_p->pres_chrg_step == STEP_NORM) ||
	     (flip_p->pres_chrg_step == STEP_NORM)) &&
	    mmi_charge_halted(chg)) {
		vote(chg->chg_dis_votable,
		     MMI_HB_VOTER, true, 0);
		pr_err("SMBMMI: Charge Halt..Toggle\n");
		msleep(50);
	}

vote_now:
	/* Votes for State */
	vote(chg->fv_votable, MMI_HB_VOTER, true, target_fv * 1000);

	vote(chg->chg_dis_votable, MMI_HB_VOTER,
	     (target_fcc < 0), 0);

	vote(chg->fcc_votable, MMI_HB_VOTER,
	     true, (target_fcc >= 0) ? (target_fcc * 1000) : 0);

	if (sm_update)
		pr_info("SMBMMI:"
			"IMPOSED: FV = %d, CDIS = %d, FCC = %d, USBICL = %d\n",
			effective_fv,
			get_effective_result(chg->chg_dis_votable),
			effective_fcc,
			get_effective_result(chg->usb_icl_votable));
	else
		pr_debug("SMBMMI:"
			"IMPOSED: FV = %d, CDIS = %d, FCC = %d, USBICL = %d\n",
			effective_fv,
			get_effective_result(chg->chg_dis_votable),
			effective_fcc,
			get_effective_result(chg->usb_icl_votable));

	return sched_time;
}

static void mmi_basic_charge_sm(struct smb_mmi_charger *chip,
				struct smb_mmi_chg_status *stat)
{
	int target_fcc;
	int target_fv;
	int max_fv_mv;
	struct mmi_temp_zone *zone;
	struct mmi_sm_params *prm = &chip->sm_param[BASE_BATT];

	pr_info("SMBMMI: batt_mv = %d, batt_ma %d, batt_soc %d,"
		" batt_temp %d, usb_mv %d, cp %d, vp %d\n",
		stat->batt_mv,
		stat->batt_ma,
		stat->batt_soc,
		stat->batt_temp,
		stat->usb_mv,
		stat->charger_present,
		stat->vbus_present);

	if (!prm->temp_zones) {
		pr_debug("SMBMMI: Skipping SM since No Temp Zone Defined!\n");
		return;
	}

	if (chip->base_fv_mv == 0) {
		chip->base_fv_mv = get_effective_result(chip->fv_votable);
		chip->base_fv_mv /= 1000;
		vote(chip->fv_votable,
		     BATT_PROFILE_VOTER, false, 0);
	}
	max_fv_mv = chip->base_fv_mv;

	mmi_find_temp_zone(chip, prm, stat->batt_temp);
	zone = &prm->temp_zones[prm->pres_temp_zone];

	if (!stat->charger_present) {
		prm->pres_chrg_step = STEP_NONE;
	} else if ((prm->pres_temp_zone == ZONE_HOT) ||
		   (prm->pres_temp_zone == ZONE_COLD) ||
		   (chip->charging_limit_modes == CHARGING_LIMIT_RUN)) {
		prm->pres_chrg_step = STEP_STOP;
	} else if (chip->demo_mode) {
		/* TODO: Ignore Demo Mode for now */
	} else if ((prm->pres_chrg_step == STEP_NONE) ||
		   (prm->pres_chrg_step == STEP_STOP)) {
		if (zone->norm_mv && (stat->batt_mv >= zone->norm_mv)) {
			if (zone->fcc_norm_ma)
				prm->pres_chrg_step = STEP_NORM;
			else
				prm->pres_chrg_step = STEP_STOP;
		} else
			prm->pres_chrg_step = STEP_MAX;
	} else if (prm->pres_chrg_step == STEP_MAX) {
		if (!zone->norm_mv) {
			/* No Step in this Zone */
			prm->chrg_taper_cnt = 0;
			if ((stat->batt_mv + HYST_STEP_MV) >= max_fv_mv)
				prm->pres_chrg_step = STEP_NORM;
			else
				prm->pres_chrg_step = STEP_MAX;
		} else if ((stat->batt_mv + HYST_STEP_MV) < zone->norm_mv) {
			prm->chrg_taper_cnt = 0;
			prm->pres_chrg_step = STEP_MAX;
		} else if (!zone->fcc_norm_ma)
			prm->pres_chrg_step = STEP_FLOAT;
		else if (mmi_has_current_tapered(chip, prm, stat->batt_ma,
						 zone->fcc_norm_ma)) {
			prm->chrg_taper_cnt = 0;

			if (mmi_charge_halted(chip)) {
				vote(chip->chg_dis_votable,
				     MMI_HB_VOTER, true, 0);
				pr_err("SMBMMI: Charge Halt..Toggle\n");
				msleep(50);
			}

			prm->pres_chrg_step = STEP_NORM;
		}
	} else if (prm->pres_chrg_step == STEP_NORM) {
		if (!zone->fcc_norm_ma)
			prm->pres_chrg_step = STEP_STOP;
		else if ((stat->batt_soc < 100) ||
			 (stat->batt_mv + HYST_STEP_MV) < max_fv_mv) {
			prm->chrg_taper_cnt = 0;
			prm->pres_chrg_step = STEP_NORM;
		} else if (mmi_has_current_tapered(chip, prm, stat->batt_ma,
						   prm->chrg_iterm)) {
				prm->pres_chrg_step = STEP_FULL;
		}
	} else if (prm->pres_chrg_step == STEP_FULL) {
		if (stat->batt_soc <= 99) {
			prm->chrg_taper_cnt = 0;
			prm->pres_chrg_step = STEP_NORM;
		}
	} else if (prm->pres_chrg_step == STEP_FLOAT) {
		if ((zone->fcc_norm_ma) ||
		    ((stat->batt_mv + HYST_STEP_MV) < zone->norm_mv))
			prm->pres_chrg_step = STEP_MAX;
	}

	/* Take State actions */
	switch (prm->pres_chrg_step) {
	case STEP_FLOAT:
	case STEP_MAX:
		if (!zone->norm_mv)
			target_fv = max_fv_mv;
		else
			target_fv = zone->norm_mv;
		target_fcc = zone->fcc_max_ma;
		chip->last_reported_status = -1;
		break;
	case STEP_FULL:
		target_fv = max_fv_mv;
		target_fcc = -EINVAL;
		chip->last_reported_status = POWER_SUPPLY_STATUS_FULL;
		break;
	case STEP_NORM:
		target_fv = max_fv_mv + chip->vfloat_comp_mv;
		target_fcc = zone->fcc_norm_ma;
		chip->last_reported_status = -1;
		break;
	case STEP_NONE:
		target_fv = max_fv_mv;
		target_fcc = zone->fcc_norm_ma;
		chip->last_reported_status = -1;
		break;
	case STEP_STOP:
		target_fv = max_fv_mv;
		target_fcc = -EINVAL;
		chip->last_reported_status = -1;
		break;
	case STEP_DEMO:
		target_fv = DEMO_MODE_VOLTAGE;
		target_fcc = zone->fcc_max_ma;
		chip->last_reported_status = -1;
		break;
	default:
		target_fv = max_fv_mv;
		target_fcc = zone->fcc_norm_ma;
		chip->last_reported_status = -1;
		break;
	}

	/* Votes for State */
	vote(chip->fv_votable, MMI_HB_VOTER, true, target_fv * 1000);

	vote(chip->chg_dis_votable, MMI_HB_VOTER,
	     (target_fcc < 0), 0);

	vote(chip->fcc_votable, MMI_HB_VOTER,
	     true, (target_fcc >= 0) ? (target_fcc * 1000) : 0);

	if (prm->pres_temp_zone == ZONE_HOT) {
		prm->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (prm->pres_temp_zone == ZONE_COLD) {
		prm->batt_health = POWER_SUPPLY_HEALTH_COLD;
	} else if (stat->batt_temp >= WARM_TEMP) {
		if (prm->pres_chrg_step == STEP_STOP)
			prm->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			prm->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (stat->batt_temp <= COOL_TEMP) {
		if (prm->pres_chrg_step == STEP_STOP)
			prm->batt_health = POWER_SUPPLY_HEALTH_COLD;
		else
			prm->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else
		prm->batt_health = POWER_SUPPLY_HEALTH_GOOD;

	pr_info("SMBMMI: Step State = %s, Temp Zone %d, Health %d\n",
		stepchg_str[(int)prm->pres_chrg_step],
		prm->pres_temp_zone,
		prm->batt_health);
	pr_info("SMBMMI: IMPOSED: FV = %d, CDIS = %d, FCC = %d, USBICL = %d\n",
		get_effective_result(chip->fv_votable),
		get_effective_result(chip->chg_dis_votable),
		get_effective_result(chip->fcc_votable),
		get_effective_result(chip->usb_icl_votable));
}

static void smb_mmi_awake_vote(struct smb_mmi_charger *chip, bool awake)
{
	if (awake == chip->awake)
		return;

	chip->awake = awake;
	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);
}

void update_charging_limit_modes(struct smb_mmi_charger *chip, int batt_soc)
{
	enum charging_limit_modes charging_limit_modes;

	charging_limit_modes = chip->charging_limit_modes;
	if ((charging_limit_modes != CHARGING_LIMIT_RUN)
	    && (batt_soc >= chip->upper_limit_capacity))
		charging_limit_modes = CHARGING_LIMIT_RUN;
	else if ((charging_limit_modes != CHARGING_LIMIT_OFF)
		   && (batt_soc <= chip->lower_limit_capacity))
		charging_limit_modes = CHARGING_LIMIT_OFF;

	if (charging_limit_modes != chip->charging_limit_modes)
		chip->charging_limit_modes = charging_limit_modes;
}

static bool __smb_mmi_ps_is_supplied_by(struct power_supply *supplier,
					struct power_supply *supply)
{
	int i;

	if (!supply->supplied_from && !supplier->supplied_to)
		return false;

	/* Support both supplied_to and supplied_from modes */
	if (supply->supplied_from) {
		if (!supplier->desc->name)
			return false;
		for (i = 0; i < supply->num_supplies; i++)
			if (!strcmp(supplier->desc->name,
				    supply->supplied_from[i]))
				return true;
	} else {
		if (!supply->desc->name)
			return false;
		for (i = 0; i < supplier->num_supplicants; i++)
			if (!strcmp(supplier->supplied_to[i],
				    supply->desc->name))
				return true;
	}

	return false;
}

static int __smb_mmi_ps_changed(struct device *dev, void *data)
{
	struct power_supply *psy = data;
	struct power_supply *pst = dev_get_drvdata(dev);

	if (__smb_mmi_ps_is_supplied_by(psy, pst)) {
		if (pst->desc->external_power_changed)
			pst->desc->external_power_changed(pst);
	}

	return 0;
}

static void smb_mmi_power_supply_changed(struct power_supply *psy,
					 char *envp_ext[])
{
	unsigned long flags;

	dev_err(&psy->dev, "%s: %s\n", __func__, envp_ext[0]);

	spin_lock_irqsave(&psy->changed_lock, flags);
	/*
	 * Check 'changed' here to avoid issues due to race between
	 * power_supply_changed() and this routine. In worst case
	 * power_supply_changed() can be called again just before we take above
	 * lock. During the first call of this routine we will mark 'changed' as
	 * false and it will stay false for the next call as well.
	 */
	if (likely(psy->changed)) {
		psy->changed = false;
		spin_unlock_irqrestore(&psy->changed_lock, flags);
		class_for_each_device(power_supply_class, NULL, psy,
				      __smb_mmi_ps_changed);
		atomic_notifier_call_chain(&power_supply_notifier,
				PSY_EVENT_PROP_CHANGED, psy);
		kobject_uevent_env(&psy->dev.kobj, KOBJ_CHANGE, envp_ext);
		spin_lock_irqsave(&psy->changed_lock, flags);
	}

	spin_unlock_irqrestore(&psy->changed_lock, flags);
}

static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);
static int suspend_wakeups;
module_param(suspend_wakeups, int, 0644);
#define TWO_VOLT 2000000
#define SMBCHG_HEARTBEAT_INTRVAL_NS	70000000000
#define MONOTONIC_SOC 2 /* 2 percent */
static void mmi_heartbeat_work(struct work_struct *work)
{
	struct smb_mmi_charger *chip = container_of(work,
						struct smb_mmi_charger,
						heartbeat_work.work);
	int hb_resch_time;
	union power_supply_propval pval;
	int rc, usb_suspend;
	int batt_cap = 0;
	int main_cap = 0;
	int main_cap_full = 0;
	int main_age = 0;
	int flip_cap = 0;
	int flip_cap_full = 0;
	int flip_age = 0;
	int cap_err;
	int report_cap;
	int pc_online;
	static int prev_vbus_mv = -1;
	char *chrg_rate_string = NULL;
	char *envp[2];

	struct smb_mmi_chg_status chg_stat;

	/* Have not been resumed so wait another 100 ms */
	if (chip->suspended) {
		pr_err("SMBMMI: HB running before Resume\n");
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));
		return;
	}

	smb_mmi_awake_vote(chip, true);
	alarm_try_to_cancel(&chip->heartbeat_alarm);

	pr_debug("SMBMMI: Heartbeat!\n");
	if (chip->factory_mode)
		hb_resch_time = HEARTBEAT_FACTORY_MS;
	else
		hb_resch_time = HEARTBEAT_DELAY_MS;

	chg_stat.charger_present = false;
	rc = get_prop_batt_voltage_now(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Batt Voltage rc = %d\n", rc);
		goto sch_hb;
	} else
		chg_stat.batt_mv = pval.intval / 1000;

	rc = get_prop_batt_current_now(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Batt Current rc = %d\n", rc);
		goto sch_hb;
	} else
		chg_stat.batt_ma = pval.intval / 1000;

	rc = get_prop_usb_voltage_now(chip, &pval);
	if (rc < 0) {
		pr_err("Error getting USB Voltage rc = %d\n", rc);
		goto sch_hb;
	} else
		chg_stat.usb_mv = pval.intval / 1000;

	if (prev_vbus_mv == -1)
		prev_vbus_mv = chg_stat.usb_mv;

	rc = get_prop_batt_capacity(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Batt Capacity rc = %d\n", rc);
		return;
	} else
		chg_stat.batt_soc = pval.intval;

	if (chip->last_reported_soc != -1)
		chg_stat.batt_soc = chip->last_reported_soc;

	rc = get_prop_batt_temp(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		pr_err("Error getting Batt Temperature rc = %d\n", rc);
		return;
	} else
		chg_stat.batt_temp = pval.intval / 10;

	rc = get_prop_usb_present(chip, &pval);
	if (rc < 0) {
		pr_err("Error getting USB Present rc = %d\n", rc);
		return;
	} else
		chg_stat.vbus_present = pval.intval;

	rc = get_prop_charger_present(chip, &pval);
	if (rc < 0) {
		pr_err("Error getting Charger Present rc = %d\n", rc);
		return;
	} else
		chg_stat.charger_present = pval.intval & chg_stat.vbus_present;

	mmi_chrg_rate_check(chip);

	if (chip->vbus_enabled && chip->vbus && chg_stat.charger_present) {
		rc = regulator_disable(chip->vbus);
		if (rc)
			pr_err("SMBMMI: Unable to disable vbus (%d)\n", rc);
		else {
			chip->vbus_enabled = false;
			pr_info("SMBMMI: VBUS Disable due to Charger\n");
		}
	}

	if (chip->enable_charging_limit && chip->is_factory_image)
		update_charging_limit_modes(chip, chg_stat.batt_soc);

	if (chip->charging_limit_modes == CHARGING_LIMIT_RUN)
		pr_warn("Factory Mode/Image so Limiting Charging!!!\n");

	if (chip->max_main_psy && chip->max_flip_psy) {
		cap_err = 0;
		rc = power_supply_get_property(chip->max_main_psy,
					       POWER_SUPPLY_PROP_CAPACITY,
					       &pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get maxim main capacity\n");
			cap_err = rc;
		} else
			main_cap = pval.intval;

		rc = power_supply_get_property(chip->max_main_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
					&pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get maxim main chrg full\n");
			cap_err = rc;
		} else
			main_cap_full = pval.intval;

		rc = power_supply_get_property(chip->max_flip_psy,
					       POWER_SUPPLY_PROP_CAPACITY,
					       &pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get maxim flip capacity\n");
			cap_err = rc;
		} else
			flip_cap = pval.intval;

		rc = power_supply_get_property(chip->max_flip_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
					&pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get maxim flip chrg full\n");
			cap_err = rc;
		} else
			flip_cap_full = pval.intval;

		report_cap = main_cap * main_cap_full;
		report_cap += flip_cap * flip_cap_full;
		report_cap /= main_cap_full + flip_cap_full;

		if (report_cap < 0)
			report_cap = 0;
		else if (report_cap > 100)
			report_cap = 100;

		batt_cap = chip->last_reported_soc;

		if ((batt_cap == -1) ||
		    ((report_cap != batt_cap) &&
		     (report_cap <= (batt_cap + MONOTONIC_SOC)) &&
		     (report_cap >= (batt_cap - MONOTONIC_SOC)))) {
			pr_info("SMBMMI: Updating Reported Capacity to %d\n",
				report_cap);
			chip->last_reported_soc = report_cap;
		}else if ((batt_cap < 100) && (report_cap > (batt_cap + MONOTONIC_SOC))) {
			chip->last_reported_soc++;
			pr_info("SMBMMI: Alter Up Reported Capacity to %d target %d\n",
			chip->last_reported_soc, report_cap);
		}else if ((batt_cap > 0) && (report_cap < (batt_cap - MONOTONIC_SOC))) {
			chip->last_reported_soc--;
			pr_info("SMBMMI: Alter Down Reported Capacity to %d target %d\n",
			chip->last_reported_soc, report_cap);
		}

		/* Age calculation */
		rc = power_supply_get_property(chip->max_main_psy,
					       POWER_SUPPLY_PROP_CHARGE_FULL,
					       &pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get maxim main charge full\n");
			cap_err = rc;
		} else
			main_cap = pval.intval;

		rc = power_supply_get_property(chip->max_flip_psy,
					       POWER_SUPPLY_PROP_CHARGE_FULL,
					       &pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get maxim flip charge full\n");
			cap_err = rc;
		} else
			flip_cap = pval.intval;

		main_age = ((main_cap / 10) / (main_cap_full / 1000));
		flip_age = ((flip_cap / 10) / (flip_cap_full / 1000));

		/* Block age output for now until FG can be vetted */
		if (cap_err == 0)
			chip->age = 100;

		pr_debug("SMBMMI: Age %d, Main Age %d, Flip Age %d\n",
			 chip->age, main_age, flip_age);

		/* Dual Step and Thermal Charging */
		hb_resch_time = mmi_dual_charge_control(chip, &chg_stat);
	} else if (!chip->factory_mode) {
		cap_err = 0;
		rc = power_supply_get_property(chip->bms_psy,
					       POWER_SUPPLY_PROP_CHARGE_FULL,
					       &pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get charge full\n");
			cap_err = rc;
		} else
			main_cap = pval.intval;

		rc = power_supply_get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
					&pval);
		if (rc < 0) {
			pr_err("SMBMMI: Couldn't get charge full design\n");
			cap_err = rc;
		} else
			main_cap_full = pval.intval;

		if (cap_err == 0)
			chip->age = ((main_cap / 10) / (main_cap_full / 1000));

		pr_debug("SMBMMI: Age %d\n", chip->age);

		/* Fall here for Basic Step and Thermal Charging */
		mmi_basic_charge_sm(chip, &chg_stat);
	}

	pr_debug("SMBMMI: batt_mv %d, usb_mv %d, prev_usb_mv %d batt_ma %d\n",
		 chg_stat.batt_mv, chg_stat.usb_mv,
		 prev_vbus_mv, chg_stat.batt_ma);
	if ((abs(chg_stat.usb_mv - chg_stat.batt_mv) < REV_BST_BULK_DROP) &&
	    ((chg_stat.usb_mv*1000) > TWO_VOLT)) {
		if (((chg_stat.usb_mv < REV_BST_THRESH) &&
		    ((prev_vbus_mv - REV_BST_DROP) > chg_stat.usb_mv)) ||
		    (chg_stat.batt_ma > REV_BST_MA)) {
			pr_err("Reverse Boosted: Clear, USB Suspend\n");
			if (chip->factory_mode)
				smblib_set_usb_suspend(chip, true);
			else
				vote(chip->usb_icl_votable, BOOST_BACK_VOTER,
				     true, 0);
			msleep(50);
			if (chip->factory_mode)
				smblib_set_usb_suspend(chip, false);
			else
				vote(chip->usb_icl_votable, BOOST_BACK_VOTER,
				     false, 0);
		} else {
			pr_err("Reverse Boosted: USB %d mV PUSB %d mV\n",
				   chg_stat.usb_mv, prev_vbus_mv);
		}
	}
	prev_vbus_mv = chg_stat.usb_mv;

	if (chip->factory_mode ||
	    (chip->is_factory_image && chip->enable_factory_poweroff)) {
		rc = smblib_get_usb_suspend(chip, &usb_suspend);
		if (rc < 0)
			goto sch_hb;

		rc = power_supply_get_property(chip->pc_port_psy,
					       POWER_SUPPLY_PROP_ONLINE,
					       &pval);
		if (rc < 0)
			goto sch_hb;

		pc_online = pval.intval;

		rc = power_supply_get_property(chip->usb_psy,
					       POWER_SUPPLY_PROP_ONLINE,
					       &pval);
		if (rc < 0)
			goto sch_hb;

		pr_debug("SMBMMI: Factory Kill check pc %d, usb %d, susp %d\n",
			 pc_online, pval.intval, usb_suspend);
		if (pc_online ||
		    pval.intval ||
		    (usb_suspend && ((chg_stat.usb_mv*1000) > TWO_VOLT))) {
			pr_debug("SMBMMI: Factory Kill Armed\n");
			chip->factory_kill_armed = true;
		} else if (chip->factory_kill_armed && !factory_kill_disable) {
			pr_err("SMBMMI:Factory kill power off\n");
			orderly_poweroff(true);
		} else
			chip->factory_kill_armed = false;
	}

sch_hb:
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));
	if (suspend_wakeups || chg_stat.charger_present)
		alarm_start_relative(&chip->heartbeat_alarm,
				     ns_to_ktime(SMBCHG_HEARTBEAT_INTRVAL_NS));

	if (!chg_stat.charger_present)
		smb_mmi_awake_vote(chip, false);

	chrg_rate_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (!chrg_rate_string) {
		pr_err("SMBMMI: Failed to Get Uevent Mem\n");
		envp[0] = NULL;
	} else {
		scnprintf(chrg_rate_string, CHG_SHOW_MAX_SIZE,
			  "POWER_SUPPLY_CHARGE_RATE=%s",
			  charge_rate[chip->charger_rate]);
		envp[0] = chrg_rate_string;
		envp[1] = NULL;
	}

	if (chip->batt_psy) {
		chip->batt_psy->changed = true;
		smb_mmi_power_supply_changed(chip->batt_psy, envp);
	} else if (chip->qcom_psy) {
		chip->qcom_psy->changed = true;
		smb_mmi_power_supply_changed(chip->qcom_psy, envp);
	}

	kfree(chrg_rate_string);

	__pm_relax(&chip->smb_mmi_hb_wake_source);
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

	if (psy &&
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

static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_QNOVO,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_QNOVO,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SW_JEITA_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_PARALLEL_DISABLE,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_RECHARGE_SOC,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE,
};

static int batt_get_prop(struct power_supply *psy,
			 enum power_supply_property psp,
			 union power_supply_propval *val)
{
	struct smb_mmi_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (chip->last_reported_status == -1)
			rc = power_supply_get_property(chip->qcom_psy,
						       psp, val);
		else
			val->intval = chip->last_reported_status;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->last_reported_soc == -1)
			rc = power_supply_get_property(chip->qcom_psy,
						       psp, val);
		else
			val->intval = chip->last_reported_soc;
		break;
	default:
		rc = power_supply_get_property(chip->qcom_psy, psp, val);
		if (rc < 0) {
			pr_debug("Get Unknown prop %d rc = %d\n", psp, rc);
			/* soft fail so uevents are not blocked */
			rc = 0;
			val->intval = -EINVAL;
		}
		break;
	}

	return rc;
}

static int batt_set_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 const union power_supply_propval *val)
{
	int rc = 0;
	struct smb_mmi_charger *chip = power_supply_get_drvdata(psy);

	rc = power_supply_set_property(chip->qcom_psy, prop, val);

	return rc;
}

static int batt_prop_is_writeable(struct power_supply *psy,
				  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
	case POWER_SUPPLY_PROP_DIE_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc batt_psy_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= batt_get_prop,
	.set_property	= batt_set_prop,
	.property_is_writeable = batt_prop_is_writeable,
	.properties	= batt_props,
	.num_properties	= ARRAY_SIZE(batt_props),
};

static int parse_mmi_dt(struct smb_mmi_charger *chg)
{
	struct device_node *node = chg->dev->of_node;
	int rc = 0;
	int byte_len;
	int i;
	struct mmi_sm_params *chip;

	if (!node) {
		pr_err("mmi dtree info. missing\n");
		return -ENODEV;
	}

	chip = &chg->sm_param[BASE_BATT];
	if (of_find_property(node, "qcom,mmi-temp-zones", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			pr_err("DT error wrong mmi temp zones\n");
			return -ENODEV;
		}

		chip->temp_zones = (struct mmi_temp_zone *)
			devm_kzalloc(chg->dev, byte_len, GFP_KERNEL);

		if (chip->temp_zones == NULL)
			return -ENOMEM;

		chip->num_temp_zones =
			byte_len / sizeof(struct mmi_temp_zone);

		rc = of_property_read_u32_array(node,
				"qcom,mmi-temp-zones",
				(u32 *)chip->temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			pr_err("Couldn't read mmi temp zones rc = %d\n", rc);
			return rc;
		}

		pr_err("mmi temp zones: Num: %d\n", chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			pr_err("mmi temp zones: Zone %d, Temp %d C, "	\
				"Step Volt %d mV, Full Rate %d mA, " \
				"Taper Rate %d mA\n", i,
				chip->temp_zones[i].temp_c,
				chip->temp_zones[i].norm_mv,
				chip->temp_zones[i].fcc_max_ma,
				chip->temp_zones[i].fcc_norm_ma);
		}
		chip->pres_temp_zone = ZONE_NONE;
	}

	rc = of_property_read_u32(node, "qcom,iterm-ma",
				  &chip->chrg_iterm);
	if (rc)
		chip->chrg_iterm = 150;

	rc = of_property_read_u32(node, "qcom,vfloat-comp-uv",
				  &chg->vfloat_comp_mv);
	if (rc)
		chg->vfloat_comp_mv = 0;
	chg->vfloat_comp_mv /= 1000;

	chg->enable_charging_limit =
		of_property_read_bool(node, "qcom,enable-charging-limit");

        chg->enable_factory_poweroff =
                of_property_read_bool(node, "qcom,enable-factory-poweroff");

	rc = of_property_read_u32(node, "qcom,upper-limit-capacity",
				  &chg->upper_limit_capacity);
	if (rc)
		chg->upper_limit_capacity = 100;

	rc = of_property_read_u32(node, "qcom,lower-limit-capacity",
				  &chg->lower_limit_capacity);
	if (rc)
		chg->lower_limit_capacity = 0;

	return rc;
}

static int parse_mmi_dual_dt(struct smb_mmi_charger *chg)
{
	struct device_node *node = chg->dev->of_node;
	int rc = 0;
	int byte_len;
	int i;
	struct mmi_sm_params *chip;

	if (!node) {
		pr_err("mmi dtree info. missing\n");
		return -ENODEV;
	}

	chip = &chg->sm_param[MAIN_BATT];
	if (of_find_property(node, "qcom,mmi-temp-zones-main", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			pr_err("DT error wrong mmi temp zones\n");
			return -ENODEV;
		}

		chip->temp_zones = (struct mmi_temp_zone *)
			devm_kzalloc(chg->dev, byte_len, GFP_KERNEL);

		if (chip->temp_zones == NULL)
			return -ENOMEM;

		chip->num_temp_zones =
			byte_len / sizeof(struct mmi_temp_zone);

		rc = of_property_read_u32_array(node,
				"qcom,mmi-temp-zones-main",
				(u32 *)chip->temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			pr_err("Couldn't read mmi temp zones rc = %d\n", rc);
			return rc;
		}

		pr_err("mmi temp zones main: Num: %d\n", chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			pr_err("mmi temp zones: Zone %d, Temp %d C, "	\
				"Step Volt %d mV, Full Rate %d mA, " \
				"Taper Rate %d mA\n", i,
				chip->temp_zones[i].temp_c,
				chip->temp_zones[i].norm_mv,
				chip->temp_zones[i].fcc_max_ma,
				chip->temp_zones[i].fcc_norm_ma);
		}
		chip->pres_temp_zone = ZONE_NONE;
	}

	rc = of_property_read_u32(node, "qcom,iterm-ma-main",
				  &chip->chrg_iterm);
	if (rc)
		chip->chrg_iterm = 150;

	chip = &chg->sm_param[FLIP_BATT];
	if (of_find_property(node, "qcom,mmi-temp-zones-flip", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			pr_err("DT error wrong mmi temp zones\n");
			return -ENODEV;
		}

		chip->temp_zones = (struct mmi_temp_zone *)
			devm_kzalloc(chg->dev, byte_len, GFP_KERNEL);

		if (chip->temp_zones == NULL)
			return -ENOMEM;

		chip->num_temp_zones =
			byte_len / sizeof(struct mmi_temp_zone);

		rc = of_property_read_u32_array(node,
				"qcom,mmi-temp-zones-flip",
				(u32 *)chip->temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			pr_err("Couldn't read mmi temp zones rc = %d\n", rc);
			return rc;
		}

		pr_err("mmi temp zones main: Num: %d\n", chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			pr_err("mmi temp zones: Zone %d, Temp %d C, "	\
				"Step Volt %d mV, Full Rate %d mA, " \
				"Taper Rate %d mA\n", i,
				chip->temp_zones[i].temp_c,
				chip->temp_zones[i].norm_mv,
				chip->temp_zones[i].fcc_max_ma,
				chip->temp_zones[i].fcc_norm_ma);
		}
		chip->pres_temp_zone = ZONE_NONE;
	}

	rc = of_property_read_u32(node, "qcom,iterm-ma-flip",
				  &chip->chrg_iterm);
	if (rc)
		chip->chrg_iterm = 150;

	return rc;
}

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

static ssize_t charge_rate_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct power_supply *psy;
	struct smb_mmi_charger *chip;
	psy = dev_get_drvdata(dev);
	if (psy &&
	    (strcmp(psy->desc->name, "battery") == 0))
		chip = power_supply_get_drvdata(psy);
	else {
		pr_err("SMBMMI: Not Correct PSY\n");
		return 0;
	}

	if (!chip) {
		pr_err("SMBMMI: Can't find mmi_chip\n");
		return 0;
	}
	mmi_chrg_rate_check(chip);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n",
			 charge_rate[chip->charger_rate]);
}
static DEVICE_ATTR(charge_rate, S_IRUGO, charge_rate_show, NULL);

static ssize_t age_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct power_supply *psy;
	struct smb_mmi_charger *chip;
	psy = dev_get_drvdata(dev);
	if (psy &&
	    (strcmp(psy->desc->name, "battery") == 0))
		chip = power_supply_get_drvdata(psy);
	else {
		pr_err("SMBMMI: Not Correct PSY\n");
		return 0;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chip->age);
}
static DEVICE_ATTR(age, S_IRUGO, age_show, NULL);

static struct attribute * mmi_g[] = {
	&dev_attr_charge_rate.attr,
	&dev_attr_age.attr,
	NULL,
};

static const struct attribute_group power_supply_mmi_attr_group = {
	.attrs = mmi_g,
};

static int smb_mmi_probe(struct platform_device *pdev)
{
	struct smb_mmi_charger *chip;
	int rc = 0;
	union power_supply_propval val;
	struct power_supply_config psy_cfg = {};
	const char *max_main_name, *max_flip_name;
	union power_supply_propval pval;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	psy_cfg.drv_data = chip;
	platform_set_drvdata(pdev, chip);
	chip->suspended = false;
	chip->awake = false;
	device_init_wakeup(chip->dev, true);

	smb_mmi_chg_config_init(chip);

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}

	parse_mmi_dt(chip);

	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_heartbeat_work);
	wakeup_source_init(&chip->smb_mmi_hb_wake_source,
			   "smb_mmi_hb_wake");
	alarm_init(&chip->heartbeat_alarm, ALARM_BOOTTIME,
		   mmi_heartbeat_alarm_cb);

	chip->mmi_psy = devm_power_supply_register(chip->dev, &mmi_psy_desc,
						   &psy_cfg);
	if (IS_ERR(chip->mmi_psy)) {
		dev_err(chip->dev, "failed: mmi power supply register\n");
		return PTR_ERR(chip->mmi_psy);
	}

	chip->age = 100;
	chip->last_reported_soc = -1;
	chip->last_reported_status = -1;
	chip->qcom_psy = power_supply_get_by_name("qcom_battery");
	if (chip->qcom_psy) {
		chip->batt_psy = devm_power_supply_register(chip->dev,
							    &batt_psy_desc,
							    &psy_cfg);
		if (IS_ERR(chip->batt_psy)) {
			dev_err(chip->dev,
				"failed: batt power supply register\n");
			return PTR_ERR(chip->batt_psy);
		}

		rc = sysfs_create_group(&chip->batt_psy->dev.kobj,
					&power_supply_mmi_attr_group);
		if (rc)
			dev_err(chip->dev, "failed: attr create\n");
	} else {
		chip->qcom_psy = power_supply_get_by_name("battery");
		chip->batt_psy = NULL;

		rc = sysfs_create_group(&chip->qcom_psy->dev.kobj,
					&power_supply_mmi_attr_group);
		if (rc)
			dev_err(chip->dev, "failed: attr create\n");
	}

	chip->bms_psy = power_supply_get_by_name("bms");
	chip->usb_psy = power_supply_get_by_name("usb");
	chip->main_psy = power_supply_get_by_name("main");
	chip->pc_port_psy = power_supply_get_by_name("pc_port");
	/* parse the dc power supply configuration */
	rc = of_property_read_string(pdev->dev.of_node,
				     "mmi,max-main-psy", &max_main_name);
	rc |= of_property_read_string(pdev->dev.of_node,
				      "mmi,max-flip-psy", &max_flip_name);
	if (!rc && max_main_name && max_flip_name) {
		chip->max_main_psy = power_supply_get_by_name(max_main_name);
		chip->max_flip_psy = power_supply_get_by_name(max_flip_name);
		parse_mmi_dual_dt(chip);
	}

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

	if (chip->smb_version == PM8150B_SUBTYPE) {
		if (smblib_masked_write_mmi(chip, LEGACY_CABLE_CFG_REG,
					    0xFF, 0))
			pr_err("SMBMMI: Could Not set Legacy Cable CFG\n");

		/* Ensure SW JEITA is DISABLED */
		pval.intval = 0;
		power_supply_set_property(chip->qcom_psy,
					  POWER_SUPPLY_PROP_SW_JEITA_ENABLED,
					  &pval);
		/* Ensure HW JEITA is DISABLED */
		if (smblib_masked_write_mmi(chip, PM8150B_JEITA_EN_CFG_REG,
					    0xFF, 0x00))
			pr_err("SMBMMI: Could Not Disable JEITA CFG\n");
	}

	if ((chip->smb_version == PM8150B_SUBTYPE) ||
	    (chip->smb_version == PM660_SUBTYPE))
		if (smblib_masked_write_mmi(chip, USBIN_ADAPTER_ALLOW_CFG_REG,
					    USBIN_ADAPTER_ALLOW_MASK,
					    USBIN_ADAPTER_ALLOW_5V_TO_9V))
			pr_err("Could Not set USB Adapter CFG\n");

	chip->factory_mode = mmi_factory_check();
	chip->is_factory_image = false;
	chip->charging_limit_modes = CHARGING_LIMIT_UNKNOWN;

	pval.intval = 0;
	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_REAL_TYPE,
				       &pval);
	if (!rc &&
	    chip->factory_mode &&
	    (pval.intval != POWER_SUPPLY_TYPE_UNKNOWN) &&
	    (pval.intval != POWER_SUPPLY_TYPE_USB) &&
	    (pval.intval != POWER_SUPPLY_TYPE_USB_CDP)) {
		dev_err(chip->dev, "Charger Present; Dis Factory Mode\n");
		chip->factory_mode = false;
	}

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

	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(0));

	pr_info("QPNP SMB MMI probed successfully!\n");

	return rc;
}

static void smb_mmi_shutdown(struct platform_device *pdev)
{
	struct smb_mmi_charger *chip = platform_get_drvdata(pdev);

	if (chip->qcom_psy)
		power_supply_put(chip->qcom_psy);
	if (chip->bms_psy)
		power_supply_put(chip->bms_psy);
	if (chip->usb_psy)
		power_supply_put(chip->usb_psy);
	if (chip->main_psy)
		power_supply_put(chip->main_psy);
	if (chip->pc_port_psy)
		power_supply_put(chip->pc_port_psy);
	if (chip->max_main_psy)
		power_supply_put(chip->max_main_psy);
	if (chip->max_flip_psy)
		power_supply_put(chip->max_flip_psy);

	wakeup_source_trash(&chip->smb_mmi_hb_wake_source);

	return;
}

#ifdef CONFIG_PM_SLEEP
static int smb_mmi_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct smb_mmi_charger *chip = platform_get_drvdata(pdev);

	chip->suspended = true;

	return 0;
}

static int smb_mmi_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct smb_mmi_charger *chip = platform_get_drvdata(pdev);

	chip->suspended = false;

	return 0;
}
#else
#define smb_mmi_suspend NULL
#define smb_mmi_resume NULL
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
