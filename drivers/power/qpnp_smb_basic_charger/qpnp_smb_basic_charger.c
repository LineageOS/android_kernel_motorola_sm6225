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
#include <linux/ipc_logging.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/usb/usbpd.h>
#include <linux/mmi_wake_lock.h>

#include "mmi_charger.h"

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for QPNP SMB BASIC MMI driver");

static struct smb_mmi_charger *this_chip = NULL;

#ifndef PM8150B_SUBTYPE
#define PM8150B_SUBTYPE PM855B_SUBTYPE
#endif
#ifndef PM7250B_SUBTYPE
#define PM7250B_SUBTYPE 0x2E
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

#define POWER_PATH_STATUS_REG			(DCDC_BASE + 0x0B)
#define MISC_POWER_PATH_STATUS_REG		(MISC_BASE + 0x0B)
#define USBIN_SUSPEND_STS_BIT			BIT(6)

#define DCDC_CFG_REF_MAX_PSNS_REG		(DCDC_BASE + 0x8C)

#define APSD_STATUS_REG				(USBIN_BASE + 0x07)
#define APSD_STATUS_7_BIT			BIT(7)
#define HVDCP_CHECK_TIMEOUT_BIT			BIT(6)
#define SLOW_PLUGIN_TIMEOUT_BIT			BIT(5)
#define ENUMERATION_DONE_BIT			BIT(4)
#define VADP_CHANGE_DONE_AFTER_AUTH_BIT		BIT(3)
#define QC_AUTH_DONE_STATUS_BIT			BIT(2)
#define QC_CHARGER_BIT				BIT(1)
#define APSD_DTC_STATUS_DONE_BIT		BIT(0)
#define APSD_RESULT_STATUS_REG			(USBIN_BASE + 0x08)
#define APSD_RESULT_STATUS_7_BIT		BIT(7)
#define APSD_RESULT_STATUS_MASK			GENMASK(6, 0)
#define QC_3P0_BIT				BIT(6)
#define QC_2P0_BIT				BIT(5)
#define FLOAT_CHARGER_BIT			BIT(4)
#define DCP_CHARGER_BIT				BIT(3)
#define CDP_CHARGER_BIT				BIT(2)
#define OCP_CHARGER_BIT				BIT(1)
#define SDP_CHARGER_BIT				BIT(0)
#define USBIN_INT_RT_STS			(USBIN_BASE + 0x10)
#define USBIN_PLUGIN_RT_STS_BIT			BIT(4)
#define USBIN_INT_EN_CLR			(USBIN_BASE + 0x16)
#define USBIN_OV_EN_CLR				BIT(3)
#define CMD_APSD_REG				(USBIN_BASE + 0x41)
#define APSD_RERUN_BIT				BIT(0)
#define ICL_OVERRIDE_BIT			BIT(1)
#define USBIN_CMD_ICL_OVERRIDE_REG		(USBIN_BASE + 0x42)
#define USBIN_ICL_OVERRIDE_BIT			BIT(0)
#define HVDCP_PULSE_COUNT_MAX_REG		(USBIN_BASE + 0x5B)
#define HVDCP_PULSE_COUNT_MAX_QC2_MASK		GENMASK(7, 6)
#define HVDCP_PULSE_COUNT_MAX_QC3_MASK		GENMASK(5, 0)
#define USBIN_ICL_OPTIONS_REG			(USBIN_BASE + 0x66)
#define USBIN_MODE_CHG_BIT			BIT(0)
#define USBIN_LOAD_CFG_REG			(USBIN_BASE + 0x65)
#define ICL_OVERRIDE_AFTER_APSD_BIT		BIT(4)
#define USBIN_AICL_OPTIONS_CFG_REG		(USBIN_BASE + 0x80)
#define LEGACY_CABLE_CFG_REG			(TYPEC_BASE + 0x5A)
#define USBIN_ADAPTER_ALLOW_CFG_REG		(USBIN_BASE + 0x60)
#define USBIN_ADAPTER_ALLOW_MASK		GENMASK(3, 0)

/* DCIN Interrupt Bits */
#define DCIN_PLUGIN_RT_STS_BIT			BIT(4)
#define DCIN_INT_RT_STS				(DCIN_BASE + 0x10)

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

#define BOOST_BACK_VOTER		"BOOST_BACK_VOTER"
#define MMI_HB_VOTER			"MMI_HB_VOTER"
#define BATT_PROFILE_VOTER		"BATT_PROFILE_VOTER"
#define DEMO_VOTER			"DEMO_VOTER"
#define HYST_STEP_MV 50
#define HYST_STEP_FLIP_MV (HYST_STEP_MV*2)
#define DEMO_MODE_HYS_SOC 5
#define DEMO_MODE_VOLTAGE 4000
#define WARM_TEMP 45
#define COOL_TEMP 0
#define WEAK_CHARGER_WAIT 30000
#define EVENT_WEAK_CHARGER_WAIT 0

#define HEARTBEAT_DELAY_MS 5000
#define HEARTBEAT_DUAL_DELAY_MS 10000
#define HEARTBEAT_FACTORY_MS 1000
#define HEARTBEAT_DISCHARGE_MS 60000

#define EMPTY_CYCLES 101

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
	FULLON_CHARGE_V5,
	TAPER_CHARGE_V5,
	TERMINATE_CHARGE_V5,
	PAUSE_CHARGE_V5,
	DISABLE_CHARGE_V5,
};

enum {
	CHG_BC1P2_SDP = 0,
	CHG_BC1P2_OCP,
	CHG_BC1P2_CDP,
	CHG_BC1P2_DCP,
	CHG_BC1P2_FLOAT,
	CHG_BC1P2_QC_2P0,
	CHG_BC1P2_QC_3P0,
	CHG_BC1P2_UNKNOWN,
};

enum {
        CHG_CLIENT_PRIMARY = 0,
        CHG_CLIENT_SECONDARY,
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
	struct smb_mmi_chg_param	aicl_cont_threshold;
	struct smb_mmi_chg_param	freq_switcher;
};

struct smb_mmi_chg_freq {
	unsigned int	freq_5V;
	unsigned int	freq_6V_8V;
	unsigned int	freq_9V;
	unsigned int	freq_12V;
};

struct smb_mmi_chg_client {
	struct smb_mmi_charger		*chip;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_driver	*driver;
	struct power_supply		*client_batt_psy;
	u32				chrg_taper_cnt;
};

struct smb_mmi_charger {
	struct device		*dev;
	struct regmap 		*regmap;
	struct smb_mmi_params	param;
	struct smb_mmi_chg_freq	chg_freq;
	const char		*name;
	int			smb_version;

	struct power_supply	*mmi_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*qcom_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*main_psy;
	struct power_supply	*pc_port_psy;
	struct power_supply	*dc_psy;
	struct power_supply 	*wls_psy;
	struct delayed_work	charger_work;

	struct votable 		*chg_dis_votable;
	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct votable		*dc_suspend_votable;

	struct mmi_charger_info	chg_info;
	struct mmi_charger_constraint constraint;
	int			chg_client_num;
	struct smb_mmi_chg_client *chg_clients;
	struct regulator	*vbus;
	bool			vbus_enabled;
	int			dc_cl_ma;

	bool			*debug_enabled;
	void			*ipc_log;
	struct usbpd		*pd_handle;
	int			pd_current_pdo;
};

#define CHGR_FAST_CHARGE_CURRENT_CFG_REG	(CHGR_BASE + 0x61)
#define CHGR_FLOAT_VOLTAGE_CFG_REG		(CHGR_BASE + 0x70)
#define USBIN_CURRENT_LIMIT_CFG_REG		(USBIN_BASE + 0x70)
#define DCIN_CURRENT_LIMIT_CFG_REG		(DCIN_BASE + 0x70)
#define USBIN_CONT_AICL_THRESHOLD_REG		(USBIN_BASE + 0x84)
#define DCDC_FSW_SEL_REG			(DCDC_BASE + 0x50)
#define AICL_RANGE2_MIN_MV		5600
#define AICL_RANGE2_STEP_DELTA_MV	200
#define AICL_RANGE2_OFFSET		16
int smblib_get_aicl_cont_threshold(struct smb_mmi_chg_param *param, u8 val_raw)
{
	int base = param->min_u;
	u8 reg = val_raw;
	int step = param->step_u;


	if (val_raw >= AICL_RANGE2_OFFSET) {
		reg = val_raw - AICL_RANGE2_OFFSET;
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
	}

	return base + (reg * step);
}

int smblib_set_aicl_cont_threshold(struct smb_mmi_chg_param *param,
				int val_u, u8 *val_raw)
{
	int base = param->min_u;
	int offset = 0;
	int step = param->step_u;

	if (val_u > param->max_u)
		val_u = param->max_u;
	if (val_u < param->min_u)
		val_u = param->min_u;

	if (val_u >= AICL_RANGE2_MIN_MV) {
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
		offset = AICL_RANGE2_OFFSET;
	};

	*val_raw = ((val_u - base) / step) + offset;

	return 0;
}

/********************
 * REGISTER SETTERS *
 ********************/
 struct smb_buck_boost_freq {
	int freq_khz;
	u8 val;
};
static const struct smb_buck_boost_freq chg_freq_list[] = {
	[0] = {
		.freq_khz	= 2400,
		.val		= 7,
	},
	[1] = {
		.freq_khz	= 2100,
		.val		= 8,
	},
	[2] = {
		.freq_khz	= 1600,
		.val		= 11,
	},
	[3] = {
		.freq_khz	= 1200,
		.val		= 15,
	},
};

int smblib_set_chg_freq(struct smb_mmi_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i].freq_khz == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = chg_freq_list[i].val;

	return 0;
}

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
		.name   = "DC input current limit",
		.reg    = DCDC_CFG_REF_MAX_PSNS_REG,
		.min_u  = 0,
		.max_u  = 1500000,
		.step_u = 50000,
	},
	.aicl_cont_threshold		= {
		.name   = "AICL CONT threshold",
		.reg    = USBIN_CONT_AICL_THRESHOLD_REG,
		.min_u  = 4000,
		.max_u  = 11800,
		.step_u = 100,
		.get_proc = smblib_get_aicl_cont_threshold,
		.set_proc = smblib_set_aicl_cont_threshold,
	},
	.freq_switcher		= {
		.name	= "switching frequency",
		.reg	= DCDC_FSW_SEL_REG,
		.min_u	= 600,
		.max_u	= 1200,
		.step_u	= 400,
		.set_proc = smblib_set_chg_freq,
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
	.freq_switcher		= {
		.name	= "switching frequency",
		.reg	= DCDC_FSW_SEL_REG,
		.min_u	= 600,
		.max_u	= 1200,
		.step_u	= 400,
		.set_proc = smblib_set_chg_freq,
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
	.freq_switcher		= {
		.name	= "switching frequency",
		.reg	= DCDC_FSW_SEL_REG,
		.min_u	= 600,
		.max_u	= 1200,
		.step_u	= 400,
		.set_proc = smblib_set_chg_freq,
	},
};

static int smblib_read_mmi(struct smb_mmi_charger *chg, u16 addr, u8 *val)
{
	unsigned int value;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &value);
	if (rc >= 0)
		*val = (u8)value;

	return rc;
}

static int smblib_write_mmi(struct smb_mmi_charger *chg, u16 addr, u8 val)
{
	return regmap_write(chg->regmap, addr, val);
}

static int smblib_masked_write_mmi(struct smb_mmi_charger *chg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(chg->regmap, addr, mask, val);
}

static int smblib_get_apsd_result(struct smb_mmi_charger *chg, int *type)
{
	int rc = 0;
	u8 stat;
	int apsd;

	rc = smblib_read_mmi(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		return rc;
	}
	if (!(stat & APSD_DTC_STATUS_DONE_BIT))
		return -EBUSY;

	rc = smblib_read_mmi(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read APSD_RESULT_STATUS_REG rc=%d\n", rc);
		return rc;
	}

	if (stat & QC_3P0_BIT)
		apsd = CHG_BC1P2_QC_3P0;
	else if (stat & QC_2P0_BIT)
		apsd = CHG_BC1P2_QC_2P0;
	else if (stat & FLOAT_CHARGER_BIT)
		apsd = CHG_BC1P2_FLOAT;
	else if (stat & DCP_CHARGER_BIT)
		apsd = CHG_BC1P2_DCP;
	else if (stat & CDP_CHARGER_BIT)
		apsd = CHG_BC1P2_CDP;
	else if (stat & OCP_CHARGER_BIT)
		apsd = CHG_BC1P2_OCP;
	else if (stat & SDP_CHARGER_BIT)
		apsd = CHG_BC1P2_SDP;
	else
		apsd = CHG_BC1P2_UNKNOWN;

	*type = apsd;

	return rc;
}

#define DCIN_CMD_IL_REG		(DCIN_BASE + 0x40)
#define DCIN_SUSPEND_BIT	BIT(0)
static int smblib_set_dc_suspend(struct smb_mmi_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write_mmi(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		mmi_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

#define USBIN_CMD_IL_REG	(USBIN_BASE + 0x40)
#define USBIN_SUSPEND_BIT	BIT(0)
static int smblib_set_usb_suspend(struct smb_mmi_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write_mmi(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				     suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		mmi_err(chg, "Couldn't write %s to USBIN suspend rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_is_usb_suspended(struct smb_mmi_charger *chg)
{
	int rc = 0;
	u8 temp = 0;

	if (chg->smb_version == PM660_SUBTYPE) {
		rc = smblib_read_mmi(chg, MISC_POWER_PATH_STATUS_REG, &temp);
	} else
		rc = smblib_read_mmi(chg, POWER_PATH_STATUS_REG, &temp);
	if (rc < 0)
		mmi_err(chg, "Couldn't read POWER_PATH_STATUS_REG rc=%d\n", rc);

	return (temp & USBIN_SUSPEND_STS_BIT);
}

static int smblib_set_charge_param(struct smb_mmi_charger *chg,
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
			mmi_dbg(chg, "%s: %d is out of range [%d, %d]\n",
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
		mmi_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	mmi_dbg(chg, "%s = %d (0x%02x)\n", param->name, val_u, val_raw);

	return rc;
}

static int smblib_get_charge_param(struct smb_mmi_charger *chg,
			    struct smb_mmi_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read_mmi(chg, param->reg, &val_raw);
	if (rc < 0) {
		mmi_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
		       param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	mmi_dbg(chg, "%s = %d (0x%02x)\n", param->name, *val_u, val_raw);

	return rc;
}

static ssize_t force_chg_usb_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(chip, "Invalid usb suspend mode value = %lu\n", mode);
		return -EINVAL;
	}

	r = smblib_set_usb_suspend(chip, (bool)mode);

	return r ? r : count;
}

static ssize_t force_chg_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	state = smblib_is_usb_suspended(chip);
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(chip, "Invalid chg fail mode value = %lu\n", mode);
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(chip, "Invalid chrg enable value = %lu\n", mode);
		return -EINVAL;
	}

	r = smblib_masked_write_mmi(chip, CHARGING_ENABLE_CMD_REG,
				    CHARGING_ENABLE_CMD_BIT,
				    mode ? CHARGING_ENABLE_CMD_BIT : 0);
	if (r < 0) {
		mmi_err(chip, "Factory Couldn't %s charging rc=%d\n",
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_read_mmi(chip, CHARGING_ENABLE_CMD_REG, &value);
	if (ret) {
		mmi_err(chip, "CHG_EN_BIT failed ret = %d\n", ret);
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

static ssize_t force_chg_ibatt_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long chg_current;
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		mmi_err(chip, "Invalid ibatt value = %lu\n", chg_current);
		return -EINVAL;
	}

	chg_current *= 1000; /* Convert to uA */
	if(chip->fcc_votable) {
		pmic_vote_force_val_set(chip->fcc_votable, chg_current);
		pmic_vote_force_active_set(chip->fcc_votable, 1);
	}

	return r ? r : count;
}

static ssize_t force_chg_ibatt_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int ret;
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_get_charge_param(chip, &chip->param.fcc, &state);
	if (ret < 0) {
		mmi_err(chip, "Factory Couldn't get master fcc rc=%d\n", (int)ret);
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &usb_curr);
	if (r) {
		mmi_err(chip, "Invalid iusb value = %lu\n", usb_curr);
		return -EINVAL;
	}

	usb_curr *= 1000; /* Convert to uA */
	if (chip->usb_icl_votable) {
		pmic_vote_force_val_set(chip->usb_icl_votable, usb_curr);
		pmic_vote_force_active_set(chip->usb_icl_votable, 1);
	}

	r = smblib_masked_write_mmi(chip, USBIN_ICL_OPTIONS_REG,
				    USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
	if (r < 0)
		mmi_err(chip, "Couldn't set USBIN_ICL_OPTIONS r=%d\n", (int)r);

	r = smblib_masked_write_mmi(chip, USBIN_LOAD_CFG_REG,
				    ICL_OVERRIDE_AFTER_APSD_BIT,
				    ICL_OVERRIDE_AFTER_APSD_BIT);
	if (r < 0)
		mmi_err(chip, "Couldn't set USBIN_LOAD_CFG rc=%d\n", (int)r);

	r = smblib_masked_write_mmi(chip, USBIN_AICL_OPTIONS_CFG_REG,
				    0xFF, 0x00);
	if (r < 0)
		mmi_err(chip, "Couldn't set USBIN_AICL_OPTIONS rc=%d\n", (int)r);

	return r ? r : count;
}

static ssize_t force_chg_iusb_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int r;
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	r = smblib_get_charge_param(chip, &chip->param.usb_icl, &state);
	if (r < 0) {
		mmi_err(chip, "Factory Couldn't get usb_icl rc=%d\n", (int)r);
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &dc_curr);
	if (r) {
		mmi_err(chip, "Invalid idc value = %lu\n", dc_curr);
		return -EINVAL;
	}

	dc_curr *= 1000; /* Convert to uA */
	r = smblib_set_charge_param(chip, &chip->param.dc_icl, dc_curr);
	if (r < 0) {
		mmi_err(chip, "Factory Couldn't set dc icl = %d rc=%d\n",
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	r = smblib_get_charge_param(chip, &chip->param.dc_icl, &state);
	if (r < 0) {
		mmi_err(chip, "Factory Couldn't get dc_icl rc=%d\n", (int)r);
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		mmi_err(chip, "Invalid pre-charge value = %lu\n", chg_current);
		return -EINVAL;
	}

	switch (chip->smb_version) {
	case PM8150B_SUBTYPE:
	case PM7250B_SUBTYPE:
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
		mmi_err(chip, "Set ITRICK PMIC subtype %d not supported\n",
			chip->smb_version);
		return -EINVAL;
	}

	r = smblib_masked_write_mmi(chip, PRE_CHARGE_CURRENT_CFG_REG,
				    mask,
				    value);
	if (r < 0) {
		mmi_err(chip, "Factory Couldn't set ITRICK %d  mV rc=%d\n",
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
	struct smb_mmi_charger *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_read_mmi(chip, PRE_CHARGE_CURRENT_CFG_REG, &value);
	if (ret) {
		mmi_err(chip, "Pre Chg ITrick failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

	switch (chip->smb_version) {
	case PM8150B_SUBTYPE:
	case PM7250B_SUBTYPE:
	case PMI632_SUBTYPE:
		value &= PRE_CHARGE_CURRENT_SETTING_MASK;
		state = (value * PRE_CHARGE_CONV_MV) + PRE_CHARGE_MIN;
		break;
	case PM660_SUBTYPE:
		value &= PRE_CHARGE_SMB2_MAX;
		state = value * PRE_CHARGE_SMB2_CONV_MV;
		break;
	default:
		mmi_err(chip, "Get ITRICK PMIC subtype %d not supported\n",
			chip->smb_version);
		return -EINVAL;
	}
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_itrick, 0664,
		   force_chg_itrick_show,
		   force_chg_itrick_store);

static int get_prop_usb_real_type(struct smb_mmi_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_REAL_TYPE, val);
	return rc;
}

static int get_prop_usb_voltage_now(struct smb_mmi_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

static int get_prop_dc_voltage_now(struct smb_mmi_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->dc_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->dc_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

static int get_prop_usb_current_now(struct smb_mmi_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

static int get_prop_dc_current_now(struct smb_mmi_charger *chip,
				    union power_supply_propval *val)
{
	int rc;

	if (!chip->dc_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->dc_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

static int get_prop_usb_present(struct smb_mmi_charger *chip,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read_mmi(chip, USBIN_INT_RT_STS, &stat);
	if (rc < 0) {
		mmi_err(chip, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	return rc;
}

static int get_prop_dc_present(struct smb_mmi_charger *chip,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	/*
	 * PMI632 has no DCIN support and address at 0x14XX is undefined,
	 * so just return 0 to avoid too much err log and heatbeat_work
	 * break
	 */
	if (chip->smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_read_mmi(chip, DCIN_INT_RT_STS, &stat);
	if (rc < 0) {
		mmi_err(chip, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);

	return 0;
}

static int is_wls_online(struct smb_mmi_charger *chip)
{
	int rc;
	union power_supply_propval val;

	if (!chip->wls_psy)
		return 0;

	rc = power_supply_get_property(chip->wls_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		mmi_err(chip, "Error wls online rc = %d\n", rc);
		return 0;
	}

	return val.intval;
}

#define TAPER_COUNT 2
#define TAPER_DROP_MA 100
static bool smb_mmi_has_current_tapered(void *data, int taper_ma)
{
	bool change_state = false;
	int allowed_fcc, target_ma;
	int batt_ma;
	struct smb_mmi_chg_client *chg = data;

	if (!chg) {
		mmi_err(chg->chip, "Charger chip is unavailable!\n");
		return false;
	}

	allowed_fcc = get_effective_result(chg->chip->fcc_votable) / 1000;

	if (allowed_fcc >= taper_ma)
		target_ma = taper_ma;
	else
		target_ma = allowed_fcc - TAPER_DROP_MA;

	batt_ma = chg->batt_info.batt_ma;
	if (batt_ma < 0) {
		batt_ma *= -1;
		if (batt_ma <= target_ma)
			if (chg->chrg_taper_cnt >= TAPER_COUNT) {
				change_state = true;
				chg->chrg_taper_cnt = 0;
			} else
				chg->chrg_taper_cnt++;
		else
			chg->chrg_taper_cnt = 0;
	} else {
		if (chg->chrg_taper_cnt >= TAPER_COUNT) {
			change_state = true;
			chg->chrg_taper_cnt = 0;
		} else
			chg->chrg_taper_cnt++;
	}

	return change_state;
}

static bool smb_mmi_charge_halted(void *data)
{
	u8 stat;
	int rc;
	bool flag = false;
	struct smb_mmi_chg_client *chg = data;

	rc = smblib_read_mmi(chg->chip, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		mmi_err(chg->chip, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return flag;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (chg->chip->smb_version == PM8150B_SUBTYPE ||
	    chg->chip->smb_version == PM7250B_SUBTYPE ||
	    chg->chip->smb_version == PMI632_SUBTYPE) {
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

static int smb_mmi_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
        int rc;
	union power_supply_propval pval;
        struct smb_mmi_chg_client *chg = data;
	struct smb_mmi_charger *chip = chg->chip;
	struct power_supply *client_batt_psy;

	rc = power_supply_get_property(chip->qcom_psy,
					POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Status rc = %d\n", rc);
	} else if (chg->chg_cfg.full_charged)
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
	else
		chg->batt_info.batt_status = pval.intval;

	if (chg->client_batt_psy == chip->batt_psy)
		client_batt_psy = chip->qcom_psy;
	else
		client_batt_psy = chg->client_batt_psy;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Voltage rc = %d\n", rc);
	} else
		chg->batt_info.batt_mv = pval.intval / 1000;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Current rc = %d\n", rc);
	} else
		chg->batt_info.batt_ma = pval.intval / 1000;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Capacity rc = %d\n", rc);
	} else
		chg->batt_info.batt_soc = pval.intval;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Temperature rc = %d\n", rc);
	} else
		chg->batt_info.batt_temp = pval.intval / 10;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt charge_full rc = %d\n", rc);
	} else
		chg->batt_info.batt_full_uah = pval.intval;

	rc = power_supply_get_property(client_batt_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt charge_full_design rc = %d\n", rc);
	} else
		chg->batt_info.batt_design_uah = pval.intval;

        memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));

        return rc;
}

static int smb_mmi_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
        int rc;
	int usb_type;
	int usb_icl;
	union power_supply_propval val;
	struct smb_mmi_chg_client *chg = data;
	struct smb_mmi_charger *chip = chg->chip;

	/* USB charger */
	val.intval = 0;
	rc = get_prop_usb_present(chip, &val);
	if (rc < 0)
		mmi_err(chip, "Couldn't read usb present rc=%d\n", rc);
	else if (val.intval) {
		chip->chg_info.chrg_present = 1;
		val.intval = 0;
		rc = get_prop_usb_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read usb voltage rc=%d\n", rc);
		chip->chg_info.chrg_mv = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_usb_current_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read usb current rc=%d\n", rc);
		chip->chg_info.chrg_ma = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_usb_real_type(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read usb type rc=%d\n", rc);
		chip->chg_info.chrg_type = val.intval;

		usb_type = chip->chg_info.chrg_type;
		if (usb_type == POWER_SUPPLY_TYPE_USB)
			chip->chg_info.chrg_pmax_mw = 2500;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_CDP)
			chip->chg_info.chrg_pmax_mw = 7500;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_DCP)
			chip->chg_info.chrg_pmax_mw = chip->constraint.dcp_pmax;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_HVDCP)
			chip->chg_info.chrg_pmax_mw = 7500;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_HVDCP_3 ||
			usb_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5)
			chip->chg_info.chrg_pmax_mw = chip->constraint.hvdcp_pmax;
		else if (usb_type == POWER_SUPPLY_TYPE_USB_PD)
			chip->chg_info.chrg_pmax_mw = chip->constraint.pd_pmax;
		else
			chip->chg_info.chrg_pmax_mw = 2500;

		usb_icl = get_effective_result(chip->usb_icl_votable);
		if (chip->chg_info.chrg_pmax_mw < (usb_icl * 5 / 1000))
			chip->chg_info.chrg_pmax_mw = usb_icl * 5 / 1000;

		rc = 0;
		goto completed;
	}

	/* DC charger */
	val.intval = 0;
	rc = get_prop_dc_present(chip, &val);
	if (rc < 0)
		mmi_err(chip, "Couldn't read dc present rc=%d\n", rc);
	else if (val.intval) {
		chip->chg_info.chrg_present = 1;
		val.intval = 0;
		rc = get_prop_dc_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read dc voltage rc=%d\n", rc);
		chip->chg_info.chrg_mv = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_dc_current_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read dc current rc=%d\n", rc);
		chip->chg_info.chrg_ma = val.intval / 1000;

		chip->chg_info.chrg_type = POWER_SUPPLY_TYPE_WIPOWER;
		chip->chg_info.chrg_pmax_mw = chip->constraint.wls_pmax;

		rc = 0;
		goto completed;
	}

	/* Wireless charger */
	if (is_wls_online(chip)) {
		chip->chg_info.chrg_present = 1;
		val.intval = 0;
		rc = get_prop_dc_voltage_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read wls voltage rc=%d\n", rc);
		chip->chg_info.chrg_mv = val.intval / 1000;
		val.intval = 0;
		rc = get_prop_dc_current_now(chip, &val);
		if (rc < 0)
			mmi_err(chip, "Couldn't read wls current rc=%d\n", rc);
		chip->chg_info.chrg_ma = val.intval / 1000;

		chip->chg_info.chrg_type = POWER_SUPPLY_TYPE_WIPOWER;
		chip->chg_info.chrg_pmax_mw = chip->constraint.wls_pmax;

		rc = 0;
		goto completed;
	}

        chip->chg_info.chrg_mv = 0;
        chip->chg_info.chrg_ma = 0;
	chip->chg_info.chrg_type = 0;
	chip->chg_info.chrg_pmax_mw = 0;
	chip->chg_info.chrg_present = 0;

completed:
	memcpy(chg_info, &chip->chg_info, sizeof(struct mmi_charger_info));

        return rc;
}

static int smb_mmi_config_charge(void *data, struct mmi_charger_cfg *config)
{
	struct smb_mmi_chg_client *chg = data;

	if (memcmp(config, &chg->chg_cfg, sizeof(struct mmi_charger_cfg))) {
		memcpy(&chg->chg_cfg, config, sizeof(struct mmi_charger_cfg));
		cancel_delayed_work(&chg->chip->charger_work);
		schedule_delayed_work(&chg->chip->charger_work,
							msecs_to_jiffies(0));
	}

	return 0;
}

static void smb_mmi_set_constraint(void *data,
			struct mmi_charger_constraint *constraint)
{
	struct smb_mmi_chg_client *chg = data;

	if (memcmp(constraint, &chg->chip->constraint,
		sizeof(struct mmi_charger_constraint))) {
		memcpy(&chg->chip->constraint, constraint,
			sizeof(struct mmi_charger_constraint));
		cancel_delayed_work(&chg->chip->charger_work);
		schedule_delayed_work(&chg->chip->charger_work,
			msecs_to_jiffies(0));
	}
}

#define USBIN_500MA     500000
#define USBIN_900MA     900000
static int smb_mmi_usb_icl_override(struct smb_mmi_charger *chg, int icl)
{
	int rc = 0;
	int apsd;
	u8 usb51_mode, icl_override, apsd_override;
	union power_supply_propval val;

	if (chg->constraint.factory_mode)
		return rc;

	if (chg->usb_psy) {
		val.intval = POWER_SUPPLY_TYPEC_NONE;
		rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_MODE, &val);
		if (rc < 0 || val.intval != POWER_SUPPLY_TYPEC_NONE)
			return rc;
	}

	rc = smblib_get_apsd_result(chg, &apsd);
	if (rc < 0 || apsd == CHG_BC1P2_UNKNOWN)
		return rc;

	if (apsd == CHG_BC1P2_SDP &&
	    (icl == USBIN_900MA || icl <= USBIN_500MA)) {
		usb51_mode = 0;
		apsd_override = 0;
		if (chg->smb_version == PM660_SUBTYPE)
			icl_override = ICL_OVERRIDE_BIT;
		else
			icl_override = USBIN_ICL_OVERRIDE_BIT;
	} else {
		usb51_mode = USBIN_MODE_CHG_BIT;
		apsd_override = ICL_OVERRIDE_AFTER_APSD_BIT;
		if (chg->smb_version == PM660_SUBTYPE)
			icl_override = ICL_OVERRIDE_BIT;
		else
			icl_override = USBIN_ICL_OVERRIDE_BIT;
	}

	if (chg->main_psy) {
		val.intval = icl;
		rc = power_supply_set_property(chg->main_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX,
				&val);
		if (rc < 0) {
			mmi_err(chg, "Couldn't set usb icl, rc=%d\n", rc);
		}
	}

	rc = smblib_masked_write_mmi(chg, USBIN_ICL_OPTIONS_REG,
				     USBIN_MODE_CHG_BIT,
				     usb51_mode);
	if (rc < 0)
		mmi_err(chg,
			"Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);

	if (chg->smb_version == PM660_SUBTYPE) {
		rc = smblib_masked_write_mmi(chg, CMD_APSD_REG,
				     ICL_OVERRIDE_BIT,
				     icl_override);
		if (rc < 0)
			mmi_err(chg,
				"Couldn't set ICL_OVERRIDE_BIT rc=%d\n", rc);
	} else {
		rc = smblib_masked_write_mmi(chg, USBIN_CMD_ICL_OVERRIDE_REG,
				     USBIN_ICL_OVERRIDE_BIT,
				     icl_override);
		if (rc < 0)
			mmi_err(chg,
				"Couldn't set USBIN_CMD_ICL_OVERRIDE rc=%d\n", rc);
	}

	rc = smblib_masked_write_mmi(chg, USBIN_LOAD_CFG_REG,
				     ICL_OVERRIDE_AFTER_APSD_BIT,
				     apsd_override);
	if (rc < 0)
		mmi_err(chg,
			"Couldn't set USBIN_LOAD_CFG rc=%d\n", rc);

	return rc;
}

#define HVDCP_POWER_MIN			15000
#define HVDCP_VOLTAGE_BASIC		(this_chip->constraint.hvdcp_pmax / 3)
#define HVDCP_VOLTAGE_NOM		(HVDCP_VOLTAGE_BASIC - 200)
#define HVDCP_VOLTAGE_MAX		(HVDCP_VOLTAGE_BASIC + 200)
#define HVDCP_VOLTAGE_MIN		4000
#define HVDCP_PULSE_COUNT_MAX 	((HVDCP_VOLTAGE_BASIC - 5000) / 200 + 2)
static void smb_mmi_config_qc_charger(struct smb_mmi_charger *chg)
{
	int rc = -EINVAL;
	int pulse_cnt;
	int vbus_mv;
	bool stepwise;
	union power_supply_propval val = {0, };

	rc = power_supply_get_property(chg->qcom_psy,
			POWER_SUPPLY_PROP_DP_DM, &val);
	if (rc < 0)
		return;
	pulse_cnt = val.intval;

	/*
	 * Two hvdcp voltage regulation algorithm are supported:
	 * stepwise and supplement. stepwise will be used if QC3.0
	 * charger power is more than 15W.
	 * It only runs once for stepwise algorithm after QC3.0
	 * charger is detected, while it will run all the time in
	 * supplement algorithm if QC3.0 charger is present.
	 */
	stepwise = chg->constraint.hvdcp_pmax > HVDCP_POWER_MIN;
	if (stepwise && pulse_cnt)
		return;

	if (!pulse_cnt) {
		rc = smblib_masked_write_mmi(chg,
					HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK |
					HVDCP_PULSE_COUNT_MAX_QC3_MASK,
					HVDCP_PULSE_COUNT_MAX);
		if (rc < 0) {
			mmi_err(chg, "Couldn't set dpdm pulse max\n");
			return;
		}

		rc = smblib_set_charge_param(chg,
					&chg->param.aicl_cont_threshold,
					HVDCP_VOLTAGE_NOM - 1000);
		if (rc < 0) {
			mmi_err(chg, "Couldn't set aicl_cont_threshold\n");
			return;
		}
		mmi_info(chg, "Configure QC3.0 output range:(%d-%d)mV\n",
					HVDCP_VOLTAGE_NOM, HVDCP_VOLTAGE_MAX);
	}

	if (stepwise) {
		vote(chg->chg_dis_votable, MMI_HB_VOTER, true, 0);
		mmi_info(chg, "Disable charging before stepwise configure\n");
	}

	do {
		rc = get_prop_usb_real_type(chg, &val);
		if (rc < 0)
			break;
		else if (val.intval != POWER_SUPPLY_TYPE_USB_HVDCP_3) {
			mmi_warn(chg, "Exit for QC3.0 charger removal\n");
			break;
		}

		rc = get_prop_usb_voltage_now(chg, &val);
		if (rc < 0)
			break;
		vbus_mv = val.intval / 1000;

		mmi_info(chg, "pulse_cnt=%d, vbus_mv=%d\n", pulse_cnt, vbus_mv);
		if (vbus_mv < HVDCP_VOLTAGE_NOM &&
		    pulse_cnt < HVDCP_PULSE_COUNT_MAX)
			val.intval = POWER_SUPPLY_DP_DM_DP_PULSE;
		else if (vbus_mv > HVDCP_VOLTAGE_MAX &&
		    pulse_cnt > 0 && !stepwise)
			val.intval = POWER_SUPPLY_DP_DM_DM_PULSE;
		else {
			mmi_info(chg, "QC3.0 output configure completed\n");
			break;
		}

		rc = power_supply_set_property(chg->qcom_psy,
				POWER_SUPPLY_PROP_DP_DM, &val);
		if (rc < 0)
			break;
		else if (!stepwise) {
			power_supply_changed(chg->usb_psy);
			break;
		}
		msleep(100);

		rc = power_supply_get_property(chg->qcom_psy,
				POWER_SUPPLY_PROP_DP_DM, &val);
		if (rc < 0)
			break;
		pulse_cnt = val.intval;
	} while (stepwise);

	if (stepwise) {
		vote(chg->chg_dis_votable, MMI_HB_VOTER, false, 0);
		mmi_info(chg, "Recover charging after stepwise configure\n");
	}
}

#define PD_POWER_MIN			15000
#ifdef CONFIG_PD_CHARGER_MMI
#define MICRO_1V			1000000
#define MICRO_5V			5000000
#define MICRO_9V			9000000
#define MICRO_12V			12000000
#define PPS_VOLT_MIN			5000000
#define PPS_CURR_MIN			2000000
#define PPS_CURR_MAX			3000000
#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3
#define PD_HEARTBEAT_DELAY_MS		10000
static int smblib_set_usb_pd_fsw(struct smb_mmi_charger *chg, int voltage)
{
	int rc;

	if (voltage == MICRO_5V)
		rc = smblib_set_charge_param(chg,
					&chg->param.freq_switcher,
					chg->chg_freq.freq_5V);
	else if (voltage > MICRO_5V && voltage < MICRO_9V)
		rc = smblib_set_charge_param(chg,
					&chg->param.freq_switcher,
					chg->chg_freq.freq_6V_8V);
	else if (voltage >= MICRO_9V && voltage < MICRO_12V)
		rc = smblib_set_charge_param(chg,
					&chg->param.freq_switcher,
					chg->chg_freq.freq_9V);
	else if (voltage == MICRO_12V)
		rc = smblib_set_charge_param(chg,
					&chg->param.freq_switcher,
					chg->chg_freq.freq_12V);
	else {
		mmi_err(chg, "Couldn't set Fsw: invalid voltage %d\n",
				voltage);
		rc = -EINVAL;
	}

	return rc;
}

static void smb_mmi_config_pd_charger(struct smb_mmi_charger *chg)
{
	int rc;
	int i;
	int vbus_mv;
	int pdo_pmax;
	int pdo_vmax;
	int req_volt;
	int req_curr;
	int req_type = -1;
	int req_pdo = -1;
	struct usbpd_pdo_info *info;
	struct usbpd_pdo_info pdo_list[PD_MAX_PDO_NUM];
	union power_supply_propval val = {0, };

	rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_PRESENT, &val);
	if (rc < 0 || !val.intval) {
		mmi_warn(chg, "USB is not present, rc=%d\n", rc);
		return;
	}

	rc  = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	if (rc < 0 || !val.intval) {
		mmi_err(chg, "USB PD is not active, rc=%d\n", rc);
		return;
	}

	rc = get_prop_usb_voltage_now(chg, &val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't get usb voltage, rc=%d\n", rc);
		return;
	}
	vbus_mv = val.intval / 1000;

	memset(pdo_list, 0, sizeof(struct usbpd_pdo_info) * PD_MAX_PDO_NUM);
	rc = usbpd_get_pdo_info(chg->pd_handle, pdo_list, PD_MAX_PDO_NUM);
	if (rc < 0) {
		mmi_err(chg, "Couldn't get pdo info, rc=%d\n", rc);
		return;
	}

	for (i = 0; i < PD_MAX_PDO_NUM; i++) {
		info = &pdo_list[i];
		if (info->type == PD_SRC_PDO_TYPE_AUGMENTED) {
			pdo_pmax = (info->uv_max / 1000) * (info->ua / 1000);
			pdo_pmax /= 1000;
			if (info->uv_max >= PPS_VOLT_MIN &&
			    info->ua >= PPS_CURR_MIN &&
			    pdo_pmax >= chg->constraint.pd_pmax) {
				req_pdo = info->pdo_pos;
				req_curr = min(info->ua, PPS_CURR_MAX);
				req_volt = (chg->constraint.pd_pmax * 1000 \
						/ (req_curr / 1000)) * 1000;
				req_volt -= (req_volt % 20000);
				req_volt = max(req_volt, PPS_VOLT_MIN);
				req_volt = min(req_volt, info->uv_max);
				req_type = PD_SRC_PDO_TYPE_AUGMENTED;
				break;
			}
		} else if (info->type == PD_SRC_PDO_TYPE_FIXED) {
			if (chg->constraint.pd_pmax <= PD_POWER_MIN)
				pdo_vmax = MICRO_5V;
			else
				pdo_vmax = MICRO_9V;

			if (info->uv_max >= MICRO_5V &&
			    info->uv_max <= pdo_vmax) {
				req_pdo = info->pdo_pos;
				req_volt = info->uv_max;
				req_curr = (chg->constraint.pd_pmax * 1000 \
						/ (req_volt / 1000)) * 1000;
				req_type = PD_SRC_PDO_TYPE_FIXED;
			}
		}
	}

	if (req_pdo < 0) {
		mmi_info(chg, "No matched pdo is found\n");
		return;
	}

	if (req_type == PD_SRC_PDO_TYPE_FIXED &&
	    vbus_mv < (req_volt - MICRO_1V) / 1000)
		chg->pd_current_pdo = -1;
	if (chg->pd_current_pdo == req_pdo &&
	    req_type == PD_SRC_PDO_TYPE_FIXED)
		return;
	else if (chg->pd_current_pdo != req_pdo) {
		smblib_set_usb_pd_fsw(chg, req_volt);
		mmi_info(chg, "Request %s pdo%d(%duV, %duA), vbus:%dmV\n",
			req_type != PD_SRC_PDO_TYPE_FIXED? "pps" : "fixed",
			req_pdo, req_volt, req_curr, vbus_mv);
	}

	rc = usbpd_select_pdo(chg->pd_handle, req_pdo, req_volt, req_curr);
	if (rc < 0) {
		mmi_err(chg, "Couldn't select pdo%d, rc=%d\n", req_pdo, rc);
	} else {
		if (req_type == PD_SRC_PDO_TYPE_FIXED) {
			val.intval = req_curr;
			rc = power_supply_set_property(chg->usb_psy,
					POWER_SUPPLY_PROP_PD_CURRENT_MAX,
					&val);
			if (rc)
				mmi_err(chg, "Couldn't set PD current max\n");
		} else {
			schedule_delayed_work(&chg->charger_work,
				msecs_to_jiffies(PD_HEARTBEAT_DELAY_MS));
		}
		chg->pd_current_pdo = req_pdo;
	}
}
#else
static void smb_mmi_config_pd_charger(struct smb_mmi_charger *chg)
{
	mmi_dbg(chg, "Configure PD charger is not supported\n");
}
#endif

#define WLS_POWER_MIN 5000
static void smb_mmi_config_wls_charger(struct smb_mmi_charger *chg)
{
	mmi_dbg(chg, "Configure wireless charger\n");
}

static void smb_mmi_config_charger_input(struct smb_mmi_charger *chip)
{
	int rc;
	int i;
	int usb_icl;
	bool charger_suspend = false;
	union power_supply_propval val;
	struct mmi_charger_cfg *cfg;

	if (!chip->chg_clients || !chip->chg_client_num) {
		mmi_err(chip, "Invalid charger client\n");
		return;
	}

	for (i = 0; i < chip->chg_client_num; i++) {
		cfg = &chip->chg_clients[i].chg_cfg;
		if (cfg->charger_suspend) {
			charger_suspend = true;
			break;
		}
	}

	if (charger_suspend) {
		if (chip->constraint.factory_mode) {
			smblib_set_usb_suspend(chip, true);
			smblib_set_dc_suspend(chip, true);
		} else {
			vote(chip->usb_icl_votable, MMI_HB_VOTER, true, 0);
			vote(chip->dc_suspend_votable, MMI_HB_VOTER, true, 1);
		}
		return;
	} else {
		if (chip->constraint.factory_mode) {
			smblib_set_usb_suspend(chip, false);
			smblib_set_dc_suspend(chip, false);
		} else {
			rc = get_prop_dc_present(chip, &val);
			if (!rc && val.intval)
				vote(chip->dc_suspend_votable, MMI_HB_VOTER,
							false, 0);
			else
				vote(chip->dc_suspend_votable, MMI_HB_VOTER,
							true, 1);
			vote(chip->usb_icl_votable, MMI_HB_VOTER, false, 0);
			usb_icl = get_effective_result(chip->usb_icl_votable);
			smb_mmi_usb_icl_override(chip, usb_icl);
		}
	}

	if (chip->pd_handle && chip->usb_psy &&
	    chip->constraint.pd_pmax >= PD_POWER_MIN &&
	    chip->chg_info.chrg_type == POWER_SUPPLY_TYPE_USB_PD)
		smb_mmi_config_pd_charger(chip);
	else
		chip->pd_current_pdo = -1;

	if (chip->qcom_psy && chip->usb_psy &&
	    chip->constraint.hvdcp_pmax >= HVDCP_POWER_MIN &&
	    chip->chg_info.chrg_mv > HVDCP_VOLTAGE_MIN &&
	    chip->chg_info.chrg_type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		smb_mmi_config_qc_charger(chip);

	if (chip->wls_psy &&
	    chip->constraint.wls_pmax > WLS_POWER_MIN &&
	    chip->chg_info.chrg_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_mmi_config_wls_charger(chip);
}

static void smb_mmi_config_charger_output(struct smb_mmi_charger *chip)
{
	int i;
	int target_fcc = INT_MAX;
	int target_fv = INT_MAX;
	bool chg_dis;
	bool charging_reset = false;
	bool charging_disable = false;
	struct mmi_charger_cfg *cfg;

	if (!chip->chg_clients || !chip->chg_client_num) {
		mmi_err(chip, "Invalid charger client\n");
		return;
	}

	chg_dis = get_effective_result(chip->chg_dis_votable);
	for (i = 0; i < chip->chg_client_num; i++) {
		cfg = &chip->chg_clients[i].chg_cfg;
		target_fcc = min(target_fcc, cfg->target_fcc);
		target_fv = min(target_fv, cfg->target_fv);
		if (cfg->charging_disable || target_fcc < 0)
			charging_disable = true;
		if (cfg->charging_reset)
			charging_reset = true;
		if (cfg->taper_kickoff)
			chip->chg_clients[i].chrg_taper_cnt = 0;
	}

	if (charging_reset && !charging_disable && !chg_dis) {
		vote(chip->chg_dis_votable, MMI_HB_VOTER, true, 0);
		mmi_warn(chip, "Charge Halt..Toggle\n");
		msleep(50);
	}

	vote(chip->fv_votable, MMI_HB_VOTER, true, target_fv * 1000);
	if (charging_disable) {
		vote(chip->fcc_votable, MMI_HB_VOTER, true, 0);
		vote(chip->chg_dis_votable, MMI_HB_VOTER, true, 0);
	} else {
		vote(chip->fcc_votable, MMI_HB_VOTER, true, target_fcc * 1000);
		vote(chip->chg_dis_votable, MMI_HB_VOTER, false, 0);
	}

	charging_disable = get_effective_result(chip->chg_dis_votable);
	/* Rerun AICL to recover USB ICL from recharge */
	if (chg_dis != charging_disable || charging_reset) {
		power_supply_set_property(chip->qcom_psy,
				POWER_SUPPLY_PROP_RERUN_AICL,
				NULL);
	}
}

#define REV_BST_THRESH 4700
#define REV_BST_DROP 150
#define REV_BST_BULK_DROP 100
#define REV_BST_MA -10
static void smb_mmi_check_reversed_boost(struct smb_mmi_charger *chip)
{
	int rc;
	int batt_ma;
	int batt_mv;
	int vbus_mv;
	int usb_suspend;
	union power_supply_propval val;
	static int prev_vbus_mv = -1;

	usb_suspend = smblib_is_usb_suspended(chip);
	if (usb_suspend)
		return;

	rc = get_prop_usb_voltage_now(chip, &val);
	if (rc < 0) {
		mmi_err(chip, "Failed to get usb voltage, rc=%d\n", rc);
		return;
	}
	vbus_mv = val.intval / 1000;

	rc = power_supply_get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&val);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Voltage rc = %d\n", rc);
		return;
	} else
		batt_mv = val.intval / 1000;

	rc = power_supply_get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW,
					&val);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Current rc = %d\n", rc);
		return;
	} else
		batt_ma = val.intval / 1000;

	prev_vbus_mv = prev_vbus_mv < 0 ? vbus_mv : prev_vbus_mv;
	if (abs(vbus_mv - batt_mv) < REV_BST_BULK_DROP) {
		if (((vbus_mv < REV_BST_THRESH) &&
		    ((prev_vbus_mv - REV_BST_DROP) > vbus_mv)) ||
		    (batt_ma > REV_BST_MA)) {
			mmi_err(chip, "Clear Reversed Boost, Suspend USBIN.\n");
			if (chip->constraint.factory_mode)
				smblib_set_usb_suspend(chip, true);
			else
				vote(chip->usb_icl_votable, BOOST_BACK_VOTER,
				     true, 0);
			msleep(50);
			if (chip->constraint.factory_mode)
				smblib_set_usb_suspend(chip, false);
			else
				vote(chip->usb_icl_votable, BOOST_BACK_VOTER,
				     false, 0);
		} else {
			mmi_err(chip, "Suspicious Reversed Boost.\n");
		}
		mmi_warn(chip, "vbus:%d, prev_vbus:%d, batt_mv:%d, batt_ma:%d\n",
			vbus_mv,
			prev_vbus_mv,
			batt_mv,
			batt_ma);
	}
	prev_vbus_mv = vbus_mv;
}

static void smb_mmi_check_otg_power(struct smb_mmi_charger *chip)
{
	int rc;

	if (chip->vbus_enabled && chip->vbus) {
		rc = regulator_disable(chip->vbus);
		if (rc)
			mmi_err(chip, "Unable to disable vbus (%d)\n", rc);
		else {
			chip->vbus_enabled = false;
			mmi_info(chip, "VBUS Disable due to Charger\n");
		}
	}
}

static void smb_mmi_charger_work(struct work_struct *work)
{
	struct smb_mmi_charger *chip = container_of(work,
				struct smb_mmi_charger,
				charger_work.work);

	pm_stay_awake(chip->dev);
	mmi_dbg(chip, "SMB MMI Charger Work!\n");

	if (chip->chg_info.chrg_present) {
		smb_mmi_check_otg_power(chip);
		smb_mmi_check_reversed_boost(chip);
	}
	smb_mmi_config_charger_input(chip);
	smb_mmi_config_charger_output(chip);

	mmi_info(chip, "FV=%d, FCC=%d, CHGDIS=%d, USBICL=%d, USBDIS=%d\n",
		get_effective_result(chip->fv_votable),
		get_effective_result(chip->fcc_votable),
		get_effective_result(chip->chg_dis_votable),
		get_effective_result(chip->usb_icl_votable),
		smblib_is_usb_suspended(chip));
	pm_relax(chip->dev);
}

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
	POWER_SUPPLY_PROP_CURRENT_MAX,
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
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE,
	POWER_SUPPLY_PROP_USB_OTG,
};

static int batt_get_prop(struct power_supply *psy,
			 enum power_supply_property psp,
			 union power_supply_propval *val)
{
	int rc = 0;
	struct smb_mmi_charger *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = power_supply_get_property(chip->mmi_psy, psp, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		rc = power_supply_get_property(chip->mmi_psy,
					POWER_SUPPLY_PROP_STATUS, val);
		if (!rc && val->intval == POWER_SUPPLY_STATUS_FULL)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = get_effective_result(chip->fcc_votable);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (chip->smb_version == PMI632_SUBTYPE) {
			val->intval = get_effective_result(chip->fcc_votable);
			break;
		}
		rc = power_supply_get_property(chip->qcom_psy, psp, val);
		if (rc < 0) {
			/* soft fail so uevents are not blocked */
			rc = 0;
			val->intval = -EINVAL;
		}
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
		rc = power_supply_get_property(chip->qcom_psy, psp, val);
		if (rc < 0) {
			mmi_dbg(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
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

	switch (prop) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		rc = power_supply_set_property(chip->mmi_psy, prop, val);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		/* Skip the hvdcp request from hvdcp daemon */
		if (chip->constraint.hvdcp_pmax)
			break;
	case POWER_SUPPLY_PROP_USB_OTG:
		if (!chip->vbus) {
			chip->vbus = devm_regulator_get(chip->dev, "vbus");
			if (IS_ERR(chip->vbus)) {
				mmi_err(chip, "Unable to get vbus\n");
				return -EINVAL;
			}
		}

		if (val->intval) {
			rc = regulator_enable(chip->vbus);
			mmi_info(chip, "VBUS Enable\n");
		} else if (chip->vbus_enabled) {
			rc = regulator_disable(chip->vbus);
			mmi_info(chip, "VBUS Disable\n");
		}

		if (rc)
			mmi_err(chip, "Unable to %s vbus (%d)\n",
			       val->intval ? "enable" : "disable", rc);
		else if (val->intval)
			chip->vbus_enabled = true;
		else
			chip->vbus_enabled = false;

		break;
	default:
		rc = power_supply_set_property(chip->qcom_psy, prop, val);
		if (rc < 0) {
			mmi_dbg(chip, "Get Unknown prop %d rc = %d\n", prop, rc);
			/* soft fail so uevents are not blocked */
			rc = 0;
		}
		break;
	}
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
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static void batt_external_power_changed(struct power_supply *psy)
{
	struct smb_mmi_charger *chip = power_supply_get_drvdata(psy);

	cancel_delayed_work(&chip->charger_work);
	schedule_delayed_work(&chip->charger_work, msecs_to_jiffies(0));

	power_supply_changed(chip->batt_psy);
}

static const struct power_supply_desc batt_psy_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= batt_get_prop,
	.set_property	= batt_set_prop,
	.external_power_changed = batt_external_power_changed,
	.property_is_writeable = batt_prop_is_writeable,
	.properties	= batt_props,
	.num_properties	= ARRAY_SIZE(batt_props),
};

#if defined(CONFIG_DEBUG_FS)
static int register_dump_read(struct seq_file *m, void *data)
{
	int rc;
	u8 stat;
	int i;
	struct smb_mmi_charger *chip = m->private;

	for (i = CHGR_BASE; i < MISC_BASE + 0x100; i++) {
		rc = smblib_read_mmi(chip, i, &stat);
		if (rc < 0)
			continue;
		seq_printf(m, "REG:0x%x: 0x%x\n", i, stat);
	}

	return 0;
}

static int register_dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb_mmi_charger *chip = inode->i_private;

	return single_open(file, register_dump_read, chip);
}

static const struct file_operations register_dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= register_dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void smb_mmi_create_debugfs(struct smb_mmi_charger *chip)
{
	struct dentry *dfs_root, *file;

	dfs_root = debugfs_create_dir("charger_mmi", NULL);
	if (IS_ERR_OR_NULL(dfs_root)) {
		mmi_err(chip, "Couldn't create charger debugfs rc=%ld\n",
			(long)dfs_root);
		return;
	}

	file = debugfs_create_file("register_dump",
			    S_IRUSR | S_IRGRP | S_IROTH,
			    dfs_root, chip, &register_dump_debugfs_ops);
	if (IS_ERR_OR_NULL(file))
		mmi_err(chip, "Couldn't create register_dump file rc=%ld\n",
			(long)file);
}
#else
static void smb_mmi_create_debugfs(struct smb_mmi_charger *chip)
{}
#endif

static int smb_mmi_charger_init(struct smb_mmi_charger *chip)
{
	int rc;
	int i;
	int count;
	const char *str = NULL;
        union power_supply_propval pval;
	struct power_supply *batt_psy;
	struct mmi_charger_driver *driver;
	struct smb_mmi_chg_client *client;
	struct device_node *np = chip->dev->of_node;
	const char *batt_sn = NULL;
	const char *df_sn = NULL;

	chip->constraint.factory_mode = mmi_is_factory_mode();

	count = of_property_count_strings(np, "battery-names");
	if (count <= 0) {
		mmi_err(chip, "Invalid battery-names in dt, count=%d\n", count);
		return -EINVAL;
	}

	client = devm_kzalloc(chip->dev,
				sizeof(struct smb_mmi_chg_client) * count,
				GFP_KERNEL);
	driver = devm_kzalloc(chip->dev,
				sizeof(struct mmi_charger_driver) * count,
				GFP_KERNEL);
	if (!client || !driver) {
		rc = -ENOMEM;
		goto free_mem;
	}

	batt_sn = mmi_get_battery_serialnumber();
	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(np, "battery-names", i, &str);
		if (rc)
			break;
		batt_psy = power_supply_get_by_name(str);
		if (!batt_psy) {
			rc = -ENODEV;
			break;
		}

		client[i].chip = chip;
		client[i].driver = &driver[i];
		client[i].client_batt_psy = batt_psy;

		driver[i].name = str;
		driver[i].dev = batt_psy->dev.parent;
		driver[i].data = &client[i];
		driver[i].get_batt_info = smb_mmi_get_batt_info;
		driver[i].get_chg_info = smb_mmi_get_chg_info;
		driver[i].config_charge = smb_mmi_config_charge;
		driver[i].is_charge_tapered = smb_mmi_has_current_tapered;
		driver[i].is_charge_halt = smb_mmi_charge_halted;
		driver[i].set_constraint = smb_mmi_set_constraint;

		if (!batt_sn) {
			df_sn = "unknown-sn";
			of_property_read_string(driver[i].dev->of_node,
						"mmi,df-serialnum", &df_sn);
			strcpy(client[i].batt_info.batt_sn, df_sn);
		} else {
			strcpy(client[i].batt_info.batt_sn, batt_sn);
		}
		mmi_info(chip, "%s Serial Number is %s\n",
					str, client[i].batt_info.batt_sn);

		rc = mmi_register_charger_driver(&driver[i]);
		if (rc)
			break;

		power_supply_changed(client[i].client_batt_psy);
		mmi_info(chip, "charger driver on %s init successfully\n",
					driver[i].name);
	}

	chip->chg_client_num = i;
	if (!chip->chg_client_num)
		goto free_mem;

	chip->chg_clients = client;

	/* Configure the charger chip */
	if (chip->smb_version == PM8150B_SUBTYPE) {
		rc = smblib_masked_write_mmi(chip, LEGACY_CABLE_CFG_REG,
						0xFF, 0);
		if (rc)
			mmi_err(chip, "Could not set Legacy Cable CFG\n");
	}

	if (chip->smb_version == PM8150B_SUBTYPE ||
	    chip->smb_version == PM7250B_SUBTYPE ||
	    chip->smb_version == PMI632_SUBTYPE ||
	    chip->smb_version == PM660_SUBTYPE) {
		mmi_info(chip, "DISABLE all QCOM JEITA\n");
		/* Ensure SW JEITA is DISABLED */
		pval.intval = 0;
		power_supply_set_property(chip->qcom_psy,
					  POWER_SUPPLY_PROP_SW_JEITA_ENABLED,
					  &pval);
		/* Ensure HW JEITA is DISABLED */
		rc = smblib_masked_write_mmi(chip, PM8150B_JEITA_EN_CFG_REG,
						0xFF, 0x00);
		if (rc)
			mmi_err(chip, "Could not disable JEITA CFG\n");
	}

	if ((chip->smb_version == PM8150B_SUBTYPE) ||
	    (chip->smb_version == PM7250B_SUBTYPE) ||
	    (chip->smb_version == PMI632_SUBTYPE) ||
	    (chip->smb_version == PM660_SUBTYPE)) {
		rc = smblib_masked_write_mmi(chip, USBIN_ADAPTER_ALLOW_CFG_REG,
						USBIN_ADAPTER_ALLOW_MASK,
						USBIN_ADAPTER_ALLOW_5V_TO_9V);
		if (rc)
			mmi_err(chip, "Could not set USB Adapter CFG\n");
	}

	if (chip->dc_cl_ma >= 0) {
		rc = smblib_set_charge_param(chip, &chip->param.dc_icl,
					chip->dc_cl_ma * 1000);
		if (rc)
			mmi_err(chip, "Failed to set DC ICL %d\n",
					chip->dc_cl_ma);
	}

	/* Workaround for some cables that collapse on boot */
	if (!chip->constraint.factory_mode) {
		mmi_err(chip, "Suspending USB for 50 ms to clear\n");
		smblib_set_usb_suspend(chip, true);
		msleep(50);
		smblib_set_usb_suspend(chip, false);
	}

	if (chip->constraint.factory_mode) {
		mmi_info(chip, "Entering Factory Mode SMB!\n");
		if (chip->smb_version == PMI632_SUBTYPE) {
			rc = smblib_masked_write_mmi(chip, USBIN_INT_EN_CLR,
						0xFF, USBIN_OV_EN_CLR);
			if (rc)
				mmi_err(chip,
					"Could Not disable usbin ov irq\n");
		}

		rc = smblib_masked_write_mmi(chip, USBIN_ICL_OPTIONS_REG,
					     USBIN_MODE_CHG_BIT,
					     USBIN_MODE_CHG_BIT);
		if (rc < 0)
			mmi_err(chip,
				"Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);

		rc = smblib_masked_write_mmi(chip, USBIN_LOAD_CFG_REG,
					     ICL_OVERRIDE_AFTER_APSD_BIT,
					     ICL_OVERRIDE_AFTER_APSD_BIT);
		if (rc < 0)
			mmi_err(chip,
				"Couldn't set USBIN_LOAD_CFG rc=%d\n", rc);

		rc = smblib_masked_write_mmi(chip, USBIN_AICL_OPTIONS_CFG_REG,
					     0xFF, 0x00);
		if (rc < 0)
			mmi_err(chip,
				"Couldn't set USBIN_AICL_OPTIONS rc=%d\n", rc);

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

		/* Some Cables need a more forced approach */
		rc = smblib_set_charge_param(chip, &chip->param.usb_icl,
					     3000000);
		if (rc < 0)
			mmi_err(chip, "Failed to set 3000mA icl rc=%d\n", rc);
	}

	return 0;
free_mem:
	if (client)
		devm_kfree(chip->dev, client);
	if (driver)
		devm_kfree(chip->dev, driver);
	chip->chg_client_num = 0;
	chip->chg_clients = NULL;

	return rc;
}

static void smb_mmi_charger_deinit(struct smb_mmi_charger *chip)
{
	int i;

	for (i = 0; i < chip->chg_client_num; i++) {
		mmi_unregister_charger_driver(chip->chg_clients[i].driver);
		power_supply_put(chip->chg_clients[i].client_batt_psy);
	}

	if (chip->chg_client_num) {
		devm_kfree(chip->dev, chip->chg_clients->driver);
		devm_kfree(chip->dev, chip->chg_clients);
	}
}

static int smb_mmi_chip_setup(struct smb_mmi_charger *chip)
{
	int rc;
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(chip->dev->of_node,
					  "mmi,pmic-revid", 0);
	if (!revid_dev_node) {
		mmi_err(chip, "Missing qcom,pmic-revid property\n");
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
	case PM7250B_SUBTYPE:
		chip->smb_version = PM7250B_SUBTYPE;
		chip->name = "pm7250b_charger";
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
		mmi_err(chip, "PMIC subtype %d not supported\n",
				pmic_rev_id->pmic_subtype);
		return -EINVAL;
	}

	chip->chg_freq.freq_5V = 600;
	chip->chg_freq.freq_6V_8V = 800;
	chip->chg_freq.freq_9V = 1050;
	chip->chg_freq.freq_12V = 1200;

	chip->pd_handle = devm_usbpd_get_by_phandle(chip->dev,
					"mmi,usbpd-phandle");
	if (IS_ERR_OR_NULL(chip->pd_handle)) {
		mmi_err(chip, "Error getting the pd phandle %ld\n",
			PTR_ERR(chip->pd_handle));
		chip->pd_handle = NULL;
	} else {
		chip->pd_current_pdo = -1;
	}

	rc = of_property_read_u32(chip->dev->of_node, "mmi,dc-icl-ma",
				  &chip->dc_cl_ma);
	if (rc)
		chip->dc_cl_ma = -EINVAL;

	mmi_info(chip, "PMIC %d is %s\n", chip->smb_version, chip->name);

	return 0;
}

static int smb_mmi_check_battery_supplies(struct smb_mmi_charger *chip)
{
	int i;
	int rc = 0;
	int count;

	if (chip->batt_psy->num_supplies && chip->batt_psy->supplied_from)
		goto print_supplies;

	count = of_property_count_strings(chip->dev->of_node, "supplied-from");
	if (count <= 0) {
		mmi_err(chip, "Invalid supplied-from in DT, rc=%d\n", count);
		return -EINVAL;
	}

	chip->batt_psy->supplied_from = devm_kmalloc_array(&chip->batt_psy->dev,
						count,
						sizeof(char *),
						GFP_KERNEL);
	if (!chip->batt_psy->supplied_from)
		return -ENOMEM;

	rc = of_property_read_string_array(chip->dev->of_node, "supplied-from",
				(const char **)chip->batt_psy->supplied_from,
				count);
	if (rc < 0) {
		mmi_err(chip, "Failed to get supplied-from, rc=%d\n", rc);
		return rc;
	}

	chip->batt_psy->num_supplies = count;

print_supplies:
	for (i = 0; i < chip->batt_psy->num_supplies; i++) {
		mmi_info(chip, "battery supplied_from[%d]=%s\n", i,
					chip->batt_psy->supplied_from[i]);
	}

	return 0;
}

static int smb_mmi_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct smb_mmi_charger *chip;
	struct power_supply_config psy_cfg = {};

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;
	platform_set_drvdata(pdev, chip);
	this_chip = chip;
	device_init_wakeup(chip->dev, true);

	rc = smb_mmi_chip_setup(chip);
	if (rc < 0) {
		mmi_err(chip, "Failed to init smb chip\n");
		return rc;
	}

	chip->debug_enabled = &debug_enabled;
	chip->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chip->ipc_log)
		mmi_err(chip, "Failed to create SMBMMI IPC log\n");
	else
		mmi_info(chip, "IPC logging is enabled for SMBMMI\n");

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		mmi_err(chip, "Parent regmap is missing\n");
		return -EINVAL;
	}

	chip->mmi_psy = power_supply_get_by_name("mmi_battery");
	if (!chip->mmi_psy) {
		mmi_err(chip, "Failed: mmi_battery has not registered yet\n");
		return -EPROBE_DEFER;
	}

	chip->qcom_psy = power_supply_get_by_name("qcom_battery");
	if (chip->qcom_psy) {
		chip->batt_psy = devm_power_supply_register(chip->dev,
							    &batt_psy_desc,
							    &psy_cfg);
		if (IS_ERR(chip->batt_psy)) {
			mmi_err(chip,
				"Failed: batt power supply register\n");
			return PTR_ERR(chip->batt_psy);
		}

		rc = smb_mmi_check_battery_supplies(chip);
		if (rc) {
			mmi_err(chip, "No valid battery supplies\n");
			return rc;
		}
	} else {
		chip->qcom_psy = power_supply_get_by_name("battery");
		chip->batt_psy = NULL;
	}

	chip->bms_psy = power_supply_get_by_name("bms");
	chip->usb_psy = power_supply_get_by_name("usb");
	chip->main_psy = power_supply_get_by_name("main");
	chip->pc_port_psy = power_supply_get_by_name("pc_port");
	chip->dc_psy = power_supply_get_by_name("dc");
	chip->wls_psy = power_supply_get_by_name("wireless");

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
	chip->dc_suspend_votable = find_votable("DC_SUSPEND");
	if (IS_ERR(chip->dc_suspend_votable))
		chip->dc_suspend_votable = NULL;

	INIT_DELAYED_WORK(&chip->charger_work, smb_mmi_charger_work);
	smb_mmi_charger_init(chip);

	if (chip->constraint.factory_mode) {
		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_usb_suspend);
		if (rc) {
			mmi_err(chip,
				   "Couldn't create force_chg_usb_suspend\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_fail_clear);
		if (rc) {
			mmi_err(chip,
				   "Couldn't create force_chg_fail_clear\n");
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
					&dev_attr_force_chg_idc);
		if (rc) {
			mmi_err(chip, "Couldn't create force_chg_idc\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_force_chg_itrick);
		if (rc) {
			mmi_err(chip, "Couldn't create force_chg_itrick\n");
		}

	}

	smb_mmi_create_debugfs(chip);

	mmi_info(chip, "QPNP SMB BASIC MMI Charger probed successfully!\n");

	return rc;
}

static int smb_mmi_remove(struct platform_device *pdev)
{
	struct smb_mmi_charger *chip = platform_get_drvdata(pdev);

	cancel_delayed_work(&chip->charger_work);
	smb_mmi_charger_deinit(chip);

	if (chip->mmi_psy)
		power_supply_put(chip->mmi_psy);
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
	if (chip->dc_psy)
		power_supply_put(chip->dc_psy);

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,qpnp-smb-basic-charger", },
	{ },
};

static struct platform_driver smb_mmi_driver = {
	.driver		= {
		.name		= "mmi,qpnp-smb-basic-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe		= smb_mmi_probe,
	.remove		= smb_mmi_remove,
};
module_platform_driver(smb_mmi_driver);

MODULE_DESCRIPTION("QPNP SMB BASIC MMI Charger Driver");
MODULE_LICENSE("GPL v2");
