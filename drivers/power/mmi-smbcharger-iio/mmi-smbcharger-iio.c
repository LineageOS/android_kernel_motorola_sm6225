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
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 61))
#include <linux/mmi-pmic-voter.h>
#else
#include <linux/pmic-voter.h>
#endif
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/ipc_logging.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/mmi_wake_lock.h>
#include <linux/iio/consumer.h>
#include <linux/qti_power_supply.h>
#include <linux/usb/usbpd.h>
#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>

#define MODULE_LOG "SMBMMI"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 61))
#define get_effective_result(votable) \
	mmi_get_effective_result(votable)

#define vote(votable, client_str, enabled, val) \
	mmi_vote(votable, client_str, enabled, val)

#define find_votable(name) mmi_find_votable(name)

#define pmic_vote_force_val_set(votable, val) \
	mmi_pmic_vote_force_val_set(votable, val)

#define pmic_vote_force_active_set(votable, val) \
	mmi_pmic_vote_force_active_set(votable, val)
#endif

#define mmi_err(chg, fmt, ...)					\
	do {							\
		pr_err("%s: %s: %s: " fmt, chg->name,		\
		       MODULE_LOG, __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,			\
		"ERR:%s: " fmt, __func__, ##__VA_ARGS__); 	\
	} while (0)

#define mmi_warn(chg, fmt, ...)					\
	do {							\
		pr_warn("%s: %s: %s: " fmt, chg->name,		\
		       MODULE_LOG, __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,			\
		"WARN:%s: " fmt, __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_info(chg, fmt, ...)					\
	do {							\
		pr_info("%s: %s: %s: " fmt, chg->name,		\
		       MODULE_LOG, __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,			\
		"INFO:%s: " fmt, __func__, ##__VA_ARGS__); 	\
	} while (0)

#define mmi_dbg(chg, fmt, ...)						\
	do {								\
		if (*chg->debug_enabled)				\
			pr_info("%s: %s: %s: " fmt, chg->name,		\
				MODULE_LOG, __func__, ##__VA_ARGS__);	\
		else							\
			pr_debug("%s: %s: %s: " fmt, chg->name,		\
				MODULE_LOG, __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,				\
			"DBG:%s: " fmt, __func__, ##__VA_ARGS__); 	\
	} while (0)

#define MMI_LOG_PAGES (50)

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for SMBMMI driver");

struct mmi_smbcharger_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define MMI_SMBCHARGER_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define MMI_SMBCHARGER_CHAN_INDEX(_name, _num)			\
	MMI_SMBCHARGER_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct mmi_smbcharger_iio_channels mmi_smbcharger_iio_psy_channels[] = {
	MMI_SMBCHARGER_CHAN_INDEX("mmi_cp_enabled_status", PSY_IIO_CP_ENABLE)
};

static struct smb_mmi_charger *this_chip = NULL;

enum pmic_type {
	PM8150B,
	PM7250B,
	PM6150,
	PMI632,
};

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
#define USBIN_INT_EN_CLR				(USBIN_BASE + 0x16)
#define USBIN_OV_EN_CLR					BIT(3)
#define CMD_APSD_REG				(USBIN_BASE + 0x41)
#define APSD_RERUN_BIT					BIT(0)
#define ICL_OVERRIDE_BIT				BIT(1)
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

#define LOW_BATT_MV 3300
#define SHUT_BATT_MV 3100
#define REV_BST_THRESH 4700
#define REV_BST_DROP 150
#define REV_BST_BULK_DROP 100
#define REV_BST_MA -10
#define BOOST_BACK_VOTER		"BOOST_BACK_VOTER"
#define MMI_HB_VOTER			"MMI_HB_VOTER"
#define BATT_PROFILE_VOTER		"BATT_PROFILE_VOTER"
#define DEMO_VOTER			"DEMO_VOTER"
#define MMI_VOTER			"MMI_VOTER"
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
#define HEARTBEAT_DUAL_DELAY_OCP_MS 1000
#define HEARTBEAT_OCP_SETTLE_CNT 7
#define HEARTBEAT_FACTORY_MS 1000
#define HEARTBEAT_DISCHARGE_MS 100000

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
	POWER_SUPPLY_CHARGE_RATE_NONE = 0,
	POWER_SUPPLY_CHARGE_RATE_NORMAL,
	POWER_SUPPLY_CHARGE_RATE_WEAK,
	POWER_SUPPLY_CHARGE_RATE_TURBO,
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
	int dc_mv;
	int charger_present;
	int vbus_present;
	int dc_present;
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

struct smb_mmi_chg_freq {
	unsigned int		freq_5V;
	unsigned int		freq_6V_8V;
	unsigned int		freq_9V;
	unsigned int		freq_12V;
};

struct smb_mmi_params {
	struct smb_mmi_chg_param	fcc;
	struct smb_mmi_chg_param	fv;
	struct smb_mmi_chg_param	usb_icl;
	struct smb_mmi_chg_param	dc_icl;
	struct smb_mmi_chg_param	aicl_cont_threshold;
	struct smb_mmi_chg_param	freq_switcher;
};

struct mmi_ffc_zone  {
       int             ffc_max_mv;
       int             ffc_chg_iterm;
       int             ffc_qg_iterm;
};

struct mmi_sm_params {
	int			num_temp_zones;
	struct mmi_ffc_zone     *ffc_zones;
	struct mmi_temp_zone	*temp_zones;
	enum mmi_temp_zones	pres_temp_zone;
	enum mmi_chrg_step	pres_chrg_step;
	int			chrg_taper_cnt;
	int			batt_health;
	int			chrg_iterm;
	int			target_fcc;
	int			target_fv;
	int			ocp[MAX_NUM_STEPS];
	int			demo_mode_prev_soc;
};

enum charging_limit_modes {
	CHARGING_LIMIT_OFF,
	CHARGING_LIMIT_RUN,
	CHARGING_LIMIT_UNKNOWN,
};

#define IS_SUSPENDED BIT(0)
#define WAS_SUSPENDED BIT(1)

struct smb_mmi_charger {
	struct device		*dev;
	struct regmap 		*regmap;
	struct smb_mmi_params	param;
	struct smb_mmi_chg_freq	chg_freq;
	char			*name;
	int			smb_version;

	struct iio_channel	**ext_iio_chans;
	struct iio_dev		*indio_dev;
	struct iio_chan_spec	*iio_chan;
	struct iio_channel	*int_iio_chans;
	bool			cp_active;

	bool			factory_mode;
	int			demo_mode;
	struct notifier_block	smb_reboot;

	struct power_supply	*mmi_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*qcom_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*pc_port_psy;
	struct power_supply	*dc_psy;
	struct power_supply 	*wls_psy;
	struct power_supply	*max_main_psy;
	struct power_supply	*max_flip_psy;
	struct notifier_block	mmi_psy_notifier;
	struct delayed_work	heartbeat_work;
	struct delayed_work	weakcharger_work;

	struct votable 		*chg_dis_votable;
	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct votable		*dc_suspend_votable;

	bool			enable_charging_limit;
        bool                    enable_factory_poweroff;
	bool			is_factory_image;
	enum charging_limit_modes	charging_limit_modes;
	int			upper_limit_capacity;
	int			lower_limit_capacity;
	int			max_chrg_temp;
	int			last_iusb_ua;
	bool			factory_kill_armed;
	bool			demo_mode_usb_suspend;
	unsigned long		flags;

	/* Charge Profile */
	struct mmi_sm_params	sm_param[3];
	int			base_fv_mv;
	int			vfloat_comp_mv;
	int			suspended;
	bool			awake;
	int 			last_reported_soc;
	int 			last_reported_status;
	struct regulator	*vbus;
	bool			vbus_enabled;
	int			prev_chg_rate;
	int 			charger_rate;
	int 			age;
	int 			cycles;
	int 			soc_cycles_start;
	bool			shut_batt;
	int			dc_cl_ma;

	bool			*debug_enabled;
	void			*ipc_log;

	struct usbpd	*pd_handle;
	struct usbpd_pdo_info	mmi_pdo_info[PD_MAX_PDO_NUM];
	int			pd_power_max;

	int			hvdcp_power_max;
	int			inc_hvdcp_cnt;
	int			hb_startup_cnt;
	bool		ocp_flag;
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
	.freq_switcher	= {
		.name	= "switching frequency",
		.reg	= DCDC_FSW_SEL_REG,
		.min_u	= 600,
		.max_u	= 1200,
		.step_u	= 400,
		.set_proc = smblib_set_chg_freq,
	},
};

enum smb_mmi_ext_iio_channels {
	//smb5
	SMB5_CTM_CURRENT_MAX = 0,
	SMB5_CURRENT_MAX,
	SMB5_TYPEC_MODE,
	SMB5_DP_DM,
	SMB5_PD_ACTIVE,
	SMB5_USB_REAL_TYPE,
	SMB5_HW_CURRENT_MAX,
	SMB5_USB_INPUT_CURRENT_SETTLED,
	SMB5_RERUN_AICL,
	SMB5_SW_JEITA_ENABLED,
	SMB5_PD_CURRENT_MAX,
	//bms
	SMB5_QG_VOLTAGE_NOW,
	SMB5_QG_CURRENT_NOW,
	SMB5_QG_TEMP,
	SMB5_QG_CAPACITY,
	SMB5_QG_CHARGE_FULL,
	SMB5_QG_CHARGE_FULL_DESIGN,
	SMB5_QG_BATT_FULL_CURRENT,
};

static const char * const smb_mmi_ext_iio_chan_name[] = {
	//smb5
	[SMB5_CTM_CURRENT_MAX] = "usb_ctm_current_max",
	[SMB5_CURRENT_MAX] = "main_current_max",
	[SMB5_TYPEC_MODE] = "usb_typec_mode",
	[SMB5_DP_DM] = "battery_dp_dm",
	[SMB5_PD_ACTIVE] = "usb_pd_active",
	[SMB5_USB_REAL_TYPE] = "usb_real_type",
	[SMB5_HW_CURRENT_MAX] = "usb_hw_current_max",
	[SMB5_USB_INPUT_CURRENT_SETTLED] = "usb_input_current_settled",
	[SMB5_RERUN_AICL] = "battery_rerun_aicl",
	[SMB5_SW_JEITA_ENABLED] = "battery_sw_jeita_enabled",
	[SMB5_PD_CURRENT_MAX] = "usb_pd_current_max",
	//bms
	[SMB5_QG_VOLTAGE_NOW] = "voltage_now",
	[SMB5_QG_CURRENT_NOW] = "current_now",
	[SMB5_QG_TEMP] = "temp",
	[SMB5_QG_CAPACITY] = "capacity",
	[SMB5_QG_CHARGE_FULL] = "charge_full",
	[SMB5_QG_CHARGE_FULL_DESIGN] = "charge_full_design",
	[SMB5_QG_BATT_FULL_CURRENT] = "batt_full_current",
};

bool is_chan_valid(struct smb_mmi_charger *chip,
		enum smb_mmi_ext_iio_channels chan)
{
	int rc;

	if (IS_ERR(chip->ext_iio_chans[chan]))
		return false;

	if (!chip->ext_iio_chans[chan]) {
		chip->ext_iio_chans[chan] = iio_channel_get(chip->dev,
					smb_mmi_ext_iio_chan_name[chan]);
		if (IS_ERR(chip->ext_iio_chans[chan])) {
			rc = PTR_ERR(chip->ext_iio_chans[chan]);
			if (rc == -EPROBE_DEFER)
				chip->ext_iio_chans[chan] = NULL;

			pr_err("Failed to get IIO channel %s, rc=%d\n",
				smb_mmi_ext_iio_chan_name[chan], rc);
			return false;
		}
	}

	return true;
}

int smb_mmi_read_iio_chan(struct smb_mmi_charger *chip,
	enum smb_mmi_ext_iio_channels chan, int *val)
{
	int rc;

	if (is_chan_valid(chip, chan)) {
		rc = iio_read_channel_processed(
				chip->ext_iio_chans[chan], val);
		return (rc < 0) ? rc : 0;
	}

	return -EINVAL;
}

int smb_mmi_write_iio_chan(struct smb_mmi_charger *chip,
	enum smb_mmi_ext_iio_channels chan, int val)
{
	if (is_chan_valid(chip, chan))
		return iio_write_channel_raw(chip->ext_iio_chans[chan],
						val);

	return -EINVAL;
}

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

int smblib_get_apsd_result(struct smb_mmi_charger *chg, int *type)
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

#define CHG_SHOW_MAX_SIZE 50
static u16 smblib_mmi_address = 0;
static ssize_t smblib_mmi_address_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long rc;
	unsigned long mode;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	rc = kstrtoul(buf, 0, &mode);
	if (rc) {
		pr_err("SMBMMI: Invalid smblib mmi address value = %lu\n", mode);
		return -EINVAL;
	}

	smblib_mmi_address = (u16)mode;
	mmi_info(mmi_chip, "smblib mmi address value = 0x%04x\n", \
		smblib_mmi_address);

	return rc ? rc : count;
}

static DEVICE_ATTR(smblib_mmi_address, 0644,
		NULL,
		smblib_mmi_address_store);

static ssize_t smblib_mmi_data_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	u8 data = 0;
	int rc = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	rc = smblib_read_mmi(mmi_chip, smblib_mmi_address, &data);
	if (rc) {
		mmi_err(mmi_chip, "Couldn't read address 0x%x rc=%d\n", \
			smblib_mmi_address, rc);
		return rc;
	}

	mmi_info(mmi_chip, "smblib mmi read data value = 0x%02x\n", data);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%04x: %02x\n", \
		smblib_mmi_address, data);
}

static ssize_t smblib_mmi_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long data = 0;
	int rc = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	rc = kstrtoul(buf, 0, &data);
	if (rc) {
		pr_err("SMBMMI: Invalid smblib mmi data value = %lu\n", data);
		return -EINVAL;
	}

	rc = smblib_write_mmi(mmi_chip, smblib_mmi_address, (u8)data);
	if (rc) {
		mmi_err(mmi_chip, "Couldn't write address 0x%x rc=%d\n", \
			smblib_mmi_address, rc);
		return rc;
	}

	mmi_info(mmi_chip, "smblib mmi write data value = 0x%02x\n", (u8)data);

	return rc ? rc : count;
}

static DEVICE_ATTR(smblib_mmi_data, 0644,
		smblib_mmi_data_show,
		smblib_mmi_data_store);

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
		pr_err("SMBMMI: Invalid factory image mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: Invalid demo mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: Invalid max temp value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: chip not valid\n");
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
		mmi_err(chg, "Couldn't write %s to USBIN suspend rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_get_usb_suspend(struct smb_mmi_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp = 0;

	rc = smblib_read_mmi(chg, POWER_PATH_STATUS_REG, &temp);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read POWER_PATH_STATUS_REG rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_STS_BIT;

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
		pr_err("SMBMMI: Invalid usb suspend mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
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
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}
	ret = smblib_get_usb_suspend(mmi_chip, &state);
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

static ssize_t force_chg_fail_clear_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("SMBMMI: Invalid chg fail mode value = %lu\n", mode);
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
		pr_err("SMBMMI: Invalid chrg enable value = %lu\n", mode);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	r = smblib_masked_write_mmi(mmi_chip, CHARGING_ENABLE_CMD_REG,
				    CHARGING_ENABLE_CMD_BIT,
				    mode ? CHARGING_ENABLE_CMD_BIT : 0);
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
	int state;
	int ret;
	u8 value;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_read_mmi(mmi_chip, CHARGING_ENABLE_CMD_REG, &value);
	if (ret) {
		mmi_err(mmi_chip, "CHG_EN_BIT failed ret = %d\n", ret);
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

int smblib_get_charge_param(struct smb_mmi_charger *chg,
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
		pr_err("SMBMMI: Invalid ibatt value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	chg_current *= 1000; /* Convert to uA */
	if(mmi_chip->fcc_votable) {
		pmic_vote_force_val_set(mmi_chip->fcc_votable, chg_current);
		pmic_vote_force_active_set(mmi_chip->fcc_votable, 1);
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
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_get_charge_param(mmi_chip, &mmi_chip->param.fcc, &state);
	if (ret < 0) {
		mmi_err(mmi_chip, "Factory Couldn't get master fcc rc=%d\n", (int)ret);
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
		pr_err("SMBMMI: Invalid iusb value = %lu\n", usb_curr);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	usb_curr *= 1000; /* Convert to uA */
	if (mmi_chip->usb_icl_votable) {
		pmic_vote_force_val_set(mmi_chip->usb_icl_votable, usb_curr);
		pmic_vote_force_active_set(mmi_chip->usb_icl_votable, 1);
	}

	r = smblib_masked_write_mmi(mmi_chip, USBIN_ICL_OPTIONS_REG,
				    USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
	if (r < 0)
		mmi_err(mmi_chip, "Couldn't set USBIN_ICL_OPTIONS r=%d\n", (int)r);

	r = smblib_masked_write_mmi(mmi_chip, USBIN_LOAD_CFG_REG,
				    ICL_OVERRIDE_AFTER_APSD_BIT,
				    ICL_OVERRIDE_AFTER_APSD_BIT);
	if (r < 0)
		mmi_err(mmi_chip, "Couldn't set USBIN_LOAD_CFG rc=%d\n", (int)r);

	r = smblib_masked_write_mmi(mmi_chip, USBIN_AICL_OPTIONS_CFG_REG,
				    0xFF, 0x00);
	if (r < 0)
		mmi_err(mmi_chip, "Couldn't set USBIN_AICL_OPTIONS rc=%d\n", (int)r);

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
		mmi_err(mmi_chip, "Factory Couldn't get usb_icl rc=%d\n", (int)r);
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
		pr_err("SMBMMI: Invalid idc value = %lu\n", dc_curr);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}
	dc_curr *= 1000; /* Convert to uA */
	r = smblib_set_charge_param(mmi_chip, &mmi_chip->param.dc_icl, dc_curr);
	if (r < 0) {
		mmi_err(mmi_chip, "Factory Couldn't set dc icl = %d rc=%d\n",
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
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	r = smblib_get_charge_param(mmi_chip, &mmi_chip->param.dc_icl, &state);
	if (r < 0) {
		mmi_err(mmi_chip, "Factory Couldn't get dc_icl rc=%d\n", (int)r);
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
		pr_err("SMBMMI: Invalid pre-charge value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	switch (mmi_chip->smb_version) {
	case PM8150B:
	case PM7250B:
	case PMI632:
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
	default:
		mmi_err(mmi_chip, "Set ITRICK PMIC subtype %d not supported\n",
			mmi_chip->smb_version);
		return -EINVAL;
	}

	r = smblib_masked_write_mmi(mmi_chip, PRE_CHARGE_CURRENT_CFG_REG,
				    mask,
				    value);
	if (r < 0) {
		mmi_err(mmi_chip, "Factory Couldn't set ITRICK %d  mV rc=%d\n",
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
		pr_err("SMBMMI: chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smblib_read_mmi(mmi_chip, PRE_CHARGE_CURRENT_CFG_REG, &value);
	if (ret) {
		mmi_err(mmi_chip, "Pre Chg ITrick failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

	switch (mmi_chip->smb_version) {
	case PM8150B:
	case PM7250B:
	case PMI632:
		value &= PRE_CHARGE_CURRENT_SETTING_MASK;
		state = (value * PRE_CHARGE_CONV_MV) + PRE_CHARGE_MIN;
		break;
	default:
		mmi_err(mmi_chip, "Get ITRICK PMIC subtype %d not supported\n",
			mmi_chip->smb_version);
		return -EINVAL;
	}
end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_itrick, 0664,
		   force_chg_itrick_show,
		   force_chg_itrick_store);

enum {
	MMI_FACTORY_MODE,
	MMI_FACTORY_BUILD,
};

static bool mmi_factory_check(int type)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;
	const char *bootargs = NULL;
	char *bl_version = NULL;
	char *end = NULL;

	if (!np)
		return factory;

	switch (type) {
	case MMI_FACTORY_MODE:
		factory = of_property_read_bool(np,
					"mmi,factory-cable");
		break;
	case MMI_FACTORY_BUILD:
		if (!of_property_read_string(np, "bootargs", &bootargs)) {
			bl_version = strstr(bootargs, "androidboot.bootloader=");
			if (bl_version) {
				end = strpbrk(bl_version, " ");
				bl_version = strpbrk(bl_version, "=");
			}
			if (bl_version && end > bl_version &&
			    strnstr(bl_version, "factory", end - bl_version)) {
				factory = true;
			}
		}
		break;
	default:
		factory = false;
	}
	of_node_put(np);

	return factory;
}

#define CHARGER_POWER_15W 15000
#define CHARGER_POWER_18W 18000
#define CHARGER_POWER_20W 20000
static void mmi_charger_power_support(struct smb_mmi_charger *chg)
{
	struct device_node *np = chg->dev->of_node;
	const char *charger_ability = NULL;
	int retval;

	retval = of_property_read_u32(np, "qcom,pd-power-max",
				  &chg->pd_power_max);
	if (retval) {
		chg->pd_power_max = CHARGER_POWER_18W;
	}

	chg->pd_handle =
			devm_usbpd_get_by_phandle(chg->dev, "qcom,usbpd-phandle");
	if (IS_ERR_OR_NULL(chg->pd_handle)) {
		dev_err(chg->dev, "Error getting the pd phandle %ld\n",
							PTR_ERR(chg->pd_handle));
		chg->pd_handle = NULL;
	}

	if (of_property_read_bool(np, "qcom,force-hvdcp-5v")) {
		chg->hvdcp_power_max = CHARGER_POWER_15W;
		return;
	}

	retval = of_property_read_u32(np, "qcom,hvdcp-power-max",
				  &chg->hvdcp_power_max);
	if (retval) {
		chg->hvdcp_power_max = 0;
		return;
	}

	np = of_find_node_by_path("/chosen");

	if (!np)
		return;

	retval = of_property_read_string(np, "mmi,charger",
						 &charger_ability);

	if ((retval == -EINVAL) || !charger_ability) {
		mmi_err(chg, "mmi,charger unused\n");
		of_node_put(np);
		return;

	} else
		mmi_info(chg, "charger = %s\n", charger_ability);

	if (strstr(charger_ability, "15W"))
		chg->hvdcp_power_max = CHARGER_POWER_15W;
	else if (strstr(charger_ability, "18W"))
		chg->hvdcp_power_max = CHARGER_POWER_18W;
	else if (strstr(charger_ability, "20W"))
		chg->hvdcp_power_max = CHARGER_POWER_20W;

	of_node_put(np);

	return;
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
			rc = smb_mmi_write_iio_chan(chip,
				SMB5_CTM_CURRENT_MAX, val->intval);
			rc = smb_mmi_write_iio_chan(chip,
				SMB5_CURRENT_MAX, val->intval);
			override = USBIN_ICL_OVERRIDE_BIT;
		}
		mmi_warn(chip, "Request for ICL to %d uA\n", val->intval);

		rc = smblib_masked_write_mmi(chip,
					USBIN_CMD_ICL_OVERRIDE_REG,
					USBIN_ICL_OVERRIDE_BIT,
					override);
		if (rc < 0)
			mmi_err(chip,
				"Couldn't set USBIN_CMD_ICL_OVERRIDE rc=%d\n", rc);
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
		pr_debug("SMBMMI: called before chg valid!\n");
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
		mmi_dbg(chg, "Entered Temp Zone %d!\n",
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

	if (!strcmp(psy->desc->name, "bms")) {
		rc = smb_mmi_read_iio_chan(chg, SMB5_QG_VOLTAGE_NOW, &val->intval);
	} else
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

	if (!strcmp(psy->desc->name, "bms")) {
		rc = smb_mmi_read_iio_chan(chg, SMB5_QG_CURRENT_NOW, &val->intval);
	} else
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

static int get_prop_dc_voltage_now(struct smb_mmi_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->dc_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->dc_psy,
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

	if (!strcmp(psy->desc->name, "bms")) {
		rc = smb_mmi_read_iio_chan(chg, SMB5_QG_TEMP, &val->intval);
	} else
		rc = power_supply_get_property(psy,
				       POWER_SUPPLY_PROP_TEMP, val);
	return rc;
}

static int get_prop_batt_capacity(struct smb_mmi_charger *chg,
				  struct power_supply *psy,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (!psy)
		return -EINVAL;

	if (!strcmp(psy->desc->name, "bms")) {
		rc = smb_mmi_read_iio_chan(chg, SMB5_QG_CAPACITY, &val->intval);
	} else
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
		mmi_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	return 0;
}

static int get_prop_dc_present(struct smb_mmi_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	/*
	 * PMI632 has no DCIN support and address at 0x14XX is undefined,
	 * so just return 0 to avoid too much err log and heatbeat_work
	 * break
	 */
	if (chg->smb_version == PMI632) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_read_mmi(chg, DCIN_INT_RT_STS, &stat);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);

	return 0;
}

static int get_prop_charger_present(struct smb_mmi_charger *chg,
				    union power_supply_propval *val)
{
	int rc = -EINVAL;

	val->intval = 0;

	if (chg->usb_psy) {
		rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, val);
		if (rc < 0)
			mmi_err(chg, "Couldn't read USB online rc=%d\n", rc);
		else if (val->intval)
			return rc;
	}

	if (chg->pc_port_psy) {
		rc = power_supply_get_property(chg->pc_port_psy,
				POWER_SUPPLY_PROP_ONLINE, val);
		if (rc < 0)
			mmi_err(chg, "Couldn't read PC online rc=%d\n", rc);
		else if (val->intval)
			return rc;
	}

	rc = smb_mmi_read_iio_chan(chg,
			SMB5_TYPEC_MODE, &val->intval);
	if (rc < 0)
		mmi_err(chg, "Couldn't read typec mode rc=%d\n", rc);
	else if (val->intval == QTI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT ||
		 val->intval == QTI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM ||
		 val->intval == QTI_POWER_SUPPLY_TYPEC_SOURCE_HIGH) {
		val->intval = 1;
		return rc;
	}

	if (chg->wls_psy) {
		rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_ONLINE, val);
		if (rc < 0)
			mmi_err(chg, "Couldn't read wls online rc=%d\n", rc);
		else if (val->intval)
			return rc;
	}

	//intval can be changed by get_property call we should reset to 0
	//this is true for a OTG cable
	val->intval = 0;
	return rc;
}

#define USBIN_500MA     500000
#define USBIN_900MA     900000
int mmi_usb_icl_override(struct smb_mmi_charger *chg, int icl)
{
	int rc = 0;
	int apsd;
	u8 usb51_mode, icl_override, apsd_override;
	union power_supply_propval val;

	if (chg->factory_mode)
		return rc;

	val.intval = QTI_POWER_SUPPLY_TYPEC_NONE;
	rc = smb_mmi_read_iio_chan(chg,
			SMB5_TYPEC_MODE, &val.intval);
	if (rc < 0 || val.intval != QTI_POWER_SUPPLY_TYPEC_NONE)
		return rc;

	rc = smblib_get_apsd_result(chg, &apsd);
	if (rc < 0 || apsd == CHG_BC1P2_UNKNOWN)
		return rc;

	if (apsd == CHG_BC1P2_SDP &&
	    (icl == USBIN_900MA || icl <= USBIN_500MA)) {
		usb51_mode = 0;
		apsd_override = 0;
		icl_override = USBIN_ICL_OVERRIDE_BIT;
	} else {
		usb51_mode = USBIN_MODE_CHG_BIT;
		apsd_override = ICL_OVERRIDE_AFTER_APSD_BIT;
		icl_override = USBIN_ICL_OVERRIDE_BIT;
	}

	rc = smb_mmi_write_iio_chan(chg,
			SMB5_CURRENT_MAX, icl);
	if (rc < 0) {
		mmi_err(chg, "Couldn't set usb icl, rc=%d\n", rc);
	}

	rc = smblib_masked_write_mmi(chg, USBIN_ICL_OPTIONS_REG,
				     USBIN_MODE_CHG_BIT,
				     usb51_mode);
	if (rc < 0)
		mmi_err(chg,
			"Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);

	rc = smblib_masked_write_mmi(chg, USBIN_CMD_ICL_OVERRIDE_REG,
			     USBIN_ICL_OVERRIDE_BIT,
			     icl_override);
	if (rc < 0)
		mmi_err(chg,
			"Couldn't set USBIN_CMD_ICL_OVERRIDE rc=%d\n", rc);

	rc = smblib_masked_write_mmi(chg, USBIN_LOAD_CFG_REG,
				     ICL_OVERRIDE_AFTER_APSD_BIT,
				     apsd_override);
	if (rc < 0)
		mmi_err(chg,
			"Couldn't set USBIN_LOAD_CFG rc=%d\n", rc);

	return rc;
}

#define HVDCP_VOLTAGE_BASIC		(this_chip->hvdcp_power_max / 3)
#define HVDCP_VOLTAGE_NOM		(HVDCP_VOLTAGE_BASIC - 200)
#define HVDCP_VOLTAGE_MAX		(HVDCP_VOLTAGE_BASIC + 200)
#define HVDCP_VOLTAGE_MIN		4000
#define HVDCP_PULSE_COUNT_MAX 	((HVDCP_VOLTAGE_BASIC - 5000) / 200 + 2)
static int mmi_increase_vbus_power(struct smb_mmi_charger *chg, int cur_mv)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };
	int pulse_cnt;

	rc = smb_mmi_read_iio_chan(chg, SMB5_DP_DM, &val.intval);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read dpdm pulse count rc=%d\n", rc);
		return cur_mv;
	} else {
		mmi_info(chg, "DP DM pulse count = %d\n", val.intval);
		pulse_cnt = val.intval;
	}

	/* Maintain vbus voltage for QC3.0 power/3A charging */
	if (pulse_cnt == 0 || chg->inc_hvdcp_cnt != HVDCP_PULSE_COUNT_MAX) {

		chg->inc_hvdcp_cnt = HVDCP_PULSE_COUNT_MAX;
		vote(chg->chg_dis_votable, MMI_HB_VOTER, true, 0);

		while (cur_mv < HVDCP_VOLTAGE_NOM && pulse_cnt < chg->inc_hvdcp_cnt) {
			val.intval = QTI_POWER_SUPPLY_DP_DM_DP_PULSE;
			rc = smb_mmi_write_iio_chan(chg, SMB5_DP_DM, val.intval);
			if (rc < 0) {
				mmi_err(chg, "Couldn't set dpdm pulse rc=%d\n", rc);
				break;
			}

			msleep(100);

			rc = get_prop_usb_voltage_now(chg, &val);
			if (rc < 0) {
				mmi_err(chg, "Error getting USB Voltage rc = %d\n", rc);
				break;
			} else
				cur_mv = val.intval / 1000;

			rc = smb_mmi_read_iio_chan(chg, SMB5_DP_DM, &val.intval);
			if (rc < 0) {
				mmi_err(chg, "Couldn't read dpdm pulse count rc=%d\n", rc);
				break;
			} else {
				pulse_cnt = val.intval;
			}

			mmi_info(chg, "pulse count = %d, cur_mv = %d\n", pulse_cnt, cur_mv);
		}

		rc = smblib_set_charge_param(chg, &chg->param.aicl_cont_threshold,
					     HVDCP_VOLTAGE_NOM - 1000);
		if (rc < 0) {
			mmi_err(chg, "Couldn't set aicl cont threshold to 9V rc=%d\n", rc);
		}

		vote(chg->chg_dis_votable, MMI_HB_VOTER, false, 0);
	}

	return cur_mv;
}

static ssize_t force_hvdcp_power_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long power;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &power);
	if (r) {
		pr_err("SMBMMI: Invalid hvdcp_power_max value = %lu\n", power);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	if ((power >= CHARGER_POWER_15W) &&
	    (power <= CHARGER_POWER_20W) &&
	    (mmi_chip->hvdcp_power_max != power)) {
		mmi_chip->hvdcp_power_max = power;

		r = smblib_masked_write_mmi(mmi_chip, HVDCP_PULSE_COUNT_MAX_REG,
					    HVDCP_PULSE_COUNT_MAX_QC2_MASK |
					    HVDCP_PULSE_COUNT_MAX_QC3_MASK,
					    HVDCP_PULSE_COUNT_MAX);
		if (r < 0) {
			mmi_err(mmi_chip, "Could not set HVDCP pulse count max\n");
			return r;
		}
		cancel_delayed_work(&mmi_chip->heartbeat_work);
		schedule_delayed_work(&mmi_chip->heartbeat_work,
					      msecs_to_jiffies(100));
		mmi_info(mmi_chip, "Reset hvdcp power max as %d, "
					"Reschedule heartbeat\n",
					mmi_chip->hvdcp_power_max);
	}

	return r ? r : count;
}

static ssize_t force_hvdcp_power_max_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int power;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	power = mmi_chip->hvdcp_power_max;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", power);
}

static DEVICE_ATTR(force_hvdcp_power_max, 0644,
		force_hvdcp_power_max_show,
		force_hvdcp_power_max_store);

static ssize_t force_pd_power_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long power;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	r = kstrtoul(buf, 0, &power);
	if (r) {
		pr_err("SMBMMI: Invalid pd_power_max value = %lu\n", power);
		return -EINVAL;
	}

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	if ((power >= CHARGER_POWER_15W) &&
	    (power <= CHARGER_POWER_20W) &&
	    (mmi_chip->pd_power_max != power)) {
		mmi_chip->pd_power_max = power;

		cancel_delayed_work(&mmi_chip->heartbeat_work);
		schedule_delayed_work(&mmi_chip->heartbeat_work,
					      msecs_to_jiffies(100));
		mmi_info(mmi_chip, "Reset pd power max as %d, "
					"Reschedule heartbeat\n",
					mmi_chip->pd_power_max);
	}

	return r ? r : count;
}

static ssize_t force_pd_power_max_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int power;
	struct platform_device *pdev = to_platform_device(dev);
	struct smb_mmi_charger *mmi_chip = platform_get_drvdata(pdev);

	if (!mmi_chip) {
		pr_err("SMBMMI: chip not valid\n");
		return -ENODEV;
	}

	power = mmi_chip->pd_power_max;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", power);
}

static DEVICE_ATTR(force_pd_power_max, 0644,
		force_pd_power_max_show,
		force_pd_power_max_store);

static int smblib_set_opt_switcher_freq(struct smb_mmi_charger *chg, int fsw_khz)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher, fsw_khz);
	if (rc < 0)
		mmi_err(chg, "Error in setting freq_buck rc=%d\n", rc);

	return rc;
}

#define MICRO_1V	1000000
#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_fsw(struct smb_mmi_charger *chg, int voltage)
{
	int rc = 0;

	if (voltage == MICRO_5V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_5V);
	else if (voltage > MICRO_5V && voltage < MICRO_9V)
		rc = smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (voltage >= MICRO_9V && voltage < MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_9V);
	else if (voltage == MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_12V);
	else {
		mmi_err(chg, "Couldn't set Fsw: invalid voltage %d\n",
				voltage);
		return -EINVAL;
	}

	return rc;
}

#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3
#define PPS_VOLT_MIN		5000000
#define PPS_CURR_MIN		2000000
#define PPS_CURR_MAX		3000000
static void mmi_chrg_usb_vin_pd_config(struct smb_mmi_charger *chg, int vbus_mv)
{
	int rc = -EINVAL, i =0, req_pdo = -1, req_pd_volt = 0, req_pd_curr = 0;
	int fixed_pd_volt = 0;
	union power_supply_propval val = {0, };
	bool pd_active = false, vbus_present = false, pps_active = false, fixed_active = false;
	static bool fsw_setted = false, fixed_power_done = false;

	if (!chg->usb_psy || !chg->pd_handle || chg->cp_active)
		return;

	rc = get_prop_usb_present(chg, &val);
	if (rc) {
		mmi_err(chg, "Unable to read USB PRESENT: %d\n", rc);
		return;
	}
	vbus_present = val.intval;

	rc  = smb_mmi_read_iio_chan(chg, SMB5_PD_ACTIVE, &val.intval);
	if (rc) {
		mmi_err(chg, "Unable to read PD ACTIVE: %d\n", rc);
		return;
	}
	pd_active = val.intval;

	if (!pd_active || !vbus_present) {
		fsw_setted = false;
		fixed_power_done = false;
		return;
	}

	memset(chg->mmi_pdo_info, 0,
			sizeof(struct usbpd_pdo_info) * PD_MAX_PDO_NUM);

	rc = usbpd_get_pdo_info(chg->pd_handle, chg->mmi_pdo_info, PD_MAX_PDO_NUM);
	if (rc ) {
		mmi_err(chg, "PD not ready, %d\n", rc);
		return;
	}

	for (i = 0; i < PD_MAX_PDO_NUM; i++) {
		if ((chg->mmi_pdo_info[i].type == PD_SRC_PDO_TYPE_AUGMENTED)
			&& chg->mmi_pdo_info[i].uv_max >= PPS_VOLT_MIN
			&& chg->mmi_pdo_info[i].ua >= PPS_CURR_MIN
			&& (chg->mmi_pdo_info[i].uv_max / 1000) * (chg->mmi_pdo_info[i].ua / 1000) / 1000
				>= chg->pd_power_max) {
			req_pdo = chg->mmi_pdo_info[i].pdo_pos;
			req_pd_curr = chg->mmi_pdo_info[i].ua;

			if (req_pd_curr > PPS_CURR_MAX)
				req_pd_curr = PPS_CURR_MAX;

			req_pd_volt = (chg->pd_power_max * 1000 / (req_pd_curr / 1000)) * 1000 % 20000;
			req_pd_volt = (chg->pd_power_max * 1000 / (req_pd_curr / 1000)) * 1000 - req_pd_volt;
			req_pd_volt = max(req_pd_volt, PPS_VOLT_MIN);
			req_pd_volt = min(req_pd_volt, chg->mmi_pdo_info[i].uv_max);
			pps_active = true;
			break;
		}
	}

	if (!pps_active) {

		if (chg->pd_power_max < CHARGER_POWER_18W)
			fixed_pd_volt = MICRO_5V;
		else
			fixed_pd_volt = MICRO_9V;

		for (i = 0; i < PD_MAX_PDO_NUM; i++) {
			if ((chg->mmi_pdo_info[i].type ==
					PD_SRC_PDO_TYPE_FIXED)
				&& chg->mmi_pdo_info[i].uv_max <= fixed_pd_volt
				&& chg->mmi_pdo_info[i].uv_max >= MICRO_5V) {
				req_pdo = chg->mmi_pdo_info[i].pdo_pos;
				req_pd_volt = chg->mmi_pdo_info[i].uv_max;
				req_pd_curr =  (chg->pd_power_max / (req_pd_volt / 1000)) * 1000;
				fixed_active = true;
			}
		}

		if (fixed_active == true && vbus_mv * 1000 < req_pd_volt - MICRO_1V)
			fixed_power_done = false;
	}

	if (!pps_active && !fixed_active)
		return;
	else if (!fsw_setted){
		smblib_set_usb_pd_fsw(chg, req_pd_volt);
		fsw_setted = true;
	}

	if (!fixed_power_done && fixed_active) {
		rc = usbpd_select_pdo(chg->pd_handle, req_pdo, req_pd_volt, req_pd_curr);
		if (rc < 0) {
			mmi_err(chg, "select pdo failed\n");
			return;
		}
		val.intval = req_pd_curr * 1000; /* mA->uA */
		rc = smb_mmi_write_iio_chan(chg, SMB5_PD_CURRENT_MAX, val.intval);
		if (rc) {
			mmi_err(chg, "set PD current MAX failed\n");
			return;
		}
		fixed_power_done =  true;
		mmi_err(chg, "Request fixed power , re-vote USB_ICL %duA\n", val.intval);
	} else if (pps_active){
		rc = usbpd_select_pdo(chg->pd_handle, req_pdo, req_pd_volt, req_pd_curr);
		if (rc < 0) {
			mmi_err(chg, "select pdo failed\n");
			return;
		}
	}

	mmi_err(chg,
			"Request PD power, fixed %d, pps %d, pdo %d, "
			"req volt %dmV, req curr %dmA, vbus %dmV\n",
			fixed_active,
			pps_active,
			req_pdo,
			req_pd_volt,
			req_pd_curr,
			vbus_mv);
	return;
}

static void mmi_chrg_usb_vin_config(struct smb_mmi_charger *chg, int cur_mv)
{
	int rc = -EINVAL;
	int val = 0;

	if (!chg->usb_psy || !chg->qcom_psy)
		return;

	if (cur_mv < HVDCP_VOLTAGE_MIN)
		return;

	rc = smb_mmi_read_iio_chan(chg, SMB5_PD_ACTIVE, &val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read PD active rc=%d\n", rc);
	} else if (val) {
		mmi_dbg(chg, "Skip usb vbus voltage config for PD charger\n");
		return;
	}

	rc = smb_mmi_read_iio_chan(chg, SMB5_USB_REAL_TYPE, &val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read charger type rc=%d\n", rc);
		return;
	}
	if (val != QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3)
		return;

	if (chg->hvdcp_power_max > CHARGER_POWER_15W) {
		mmi_increase_vbus_power(chg, cur_mv);
		return;
	}

	/* Maintain vbus voltage for QC3.0 5V/3A 15W charging */
	rc = smb_mmi_read_iio_chan(chg, SMB5_DP_DM, &val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't read dpdm pulse count rc=%d\n", rc);
		return;
	} else
		mmi_info(chg, "DP DM pulse count = %d\n", val);

	if (cur_mv < HVDCP_VOLTAGE_NOM && val < chg->inc_hvdcp_cnt)
		val = QTI_POWER_SUPPLY_DP_DM_DP_PULSE;
	else if (cur_mv > HVDCP_VOLTAGE_MAX && val > 0)
		val = QTI_POWER_SUPPLY_DP_DM_DM_PULSE;
	else {
		mmi_dbg(chg, "No need to change hvdcp voltage\n");
		return;
	}
	rc = smb_mmi_write_iio_chan(chg, SMB5_DP_DM, val);
	if (rc < 0) {
		mmi_err(chg, "Couldn't set dpdm pulse rc=%d\n", rc);
		return;
	}
	power_supply_changed(chg->usb_psy);
}

static void mmi_chrg_input_config(struct smb_mmi_charger *chg,
				  struct smb_mmi_chg_status *stat)
{
	if (chg->hvdcp_power_max)
		mmi_chrg_usb_vin_config(chg, stat->usb_mv);

	if (chg->pd_power_max)
		mmi_chrg_usb_vin_pd_config(chg, stat->usb_mv);
}

static void mmi_weakcharger_work(struct work_struct *work)
{
	struct smb_mmi_charger *chip = container_of(work,
						struct smb_mmi_charger,
						weakcharger_work.work);

	set_bit(EVENT_WEAK_CHARGER_WAIT, &chip->flags);
	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));

	mmi_dbg(chip, "Weak timer expired\n");
}

int is_wls_online(struct smb_mmi_charger *chg)
{
	int rc;
	union power_supply_propval val;

	if (!chg->wls_psy)
		return 0;

	rc = power_supply_get_property(chg->wls_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		mmi_err(chg, "Error wls online rc = %d\n", rc);
		return 0;
	}

	return val.intval;
}

#define WEAK_CHRG_THRSH 450
#define TURBO_CHRG_THRSH 2500
#define PD_CHRG_THRSH 8500
void mmi_chrg_rate_check(struct smb_mmi_charger *chg)
{
	union power_supply_propval val;
	union power_supply_propval wls_val;
	int chrg_v_mv = 0;
	int chrg_cm_ma = 0;
	int chrg_cs_ma = 0;
	int rc = -EINVAL;

	if (!chg->usb_psy) {
		mmi_err(chg, "No usb PSY\n");
		return;
	}

	val.intval = 0;
	chg->prev_chg_rate = chg->charger_rate;

	rc = get_prop_charger_present(chg, &val);
	if (rc < 0) {
		mmi_err(chg, "Error getting Charger Present rc = %d\n", rc);
		return;
	}

	if (val.intval && is_wls_online(chg)) {
		rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &wls_val);
		if (rc >= 0 && wls_val.intval == POWER_SUPPLY_CHARGE_TYPE_FAST)
			chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_TURBO;
		else
			chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;
		goto end_rate_check;
	}

	if (val.intval) {
		val.intval = 0;
		rc = smb_mmi_read_iio_chan(chg, SMB5_HW_CURRENT_MAX, &val.intval);
		if (rc < 0) {
			mmi_err(chg, "Error getting HW Current Max rc = %d\n", rc);
			return;
		}
		chrg_cm_ma = val.intval / 1000;

		val.intval = 0;
		rc = smb_mmi_read_iio_chan(chg,
				SMB5_USB_INPUT_CURRENT_SETTLED, &val.intval);
		if (rc < 0) {
			mmi_err(chg, "Error getting ICL Settled rc = %d\n", rc);
			return;
		}
		chrg_cs_ma = val.intval / 1000;

		rc = get_prop_usb_voltage_now(chg, &val);
		if (rc != -ENOMEM && rc < 0) {
			mmi_err(chg, "Error getting USB Input Voltage rc = %d\n", rc);
			return;
		}
		chrg_v_mv = val.intval / 1000;
	} else {
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NONE;
		goto end_rate_check;
	}

	mmi_dbg(chg, "cm %d, cs %d, v %d\n", chrg_cm_ma, chrg_cs_ma, chrg_v_mv);
	if (chrg_cm_ma >= TURBO_CHRG_THRSH || chrg_v_mv >= PD_CHRG_THRSH)
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_TURBO;
	else if ((chrg_cm_ma > WEAK_CHRG_THRSH) &&
			(chrg_cs_ma < WEAK_CHRG_THRSH) &&
			test_bit(EVENT_WEAK_CHARGER_WAIT, &chg->flags))
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_WEAK;
	else if (chg->prev_chg_rate == POWER_SUPPLY_CHARGE_RATE_NONE)
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;
	else if ((chg->prev_chg_rate == POWER_SUPPLY_CHARGE_RATE_WEAK) &&
			(chrg_cm_ma > WEAK_CHRG_THRSH) &&
			(chrg_cs_ma >= WEAK_CHRG_THRSH))
		chg->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;

end_rate_check:
	if (chg->prev_chg_rate != chg->charger_rate) {
		clear_bit(EVENT_WEAK_CHARGER_WAIT, &chg->flags);
		cancel_delayed_work(&chg->weakcharger_work);
		if(chg->charger_rate == POWER_SUPPLY_CHARGE_RATE_NORMAL)
			schedule_delayed_work(&chg->weakcharger_work,
					msecs_to_jiffies(WEAK_CHARGER_WAIT));

		mmi_info(chg, "%s Charger Detected\n",
		       charge_rate[chg->charger_rate]);
	}
}

#define TAPER_COUNT 2
#define TAPER_DROP_MA 100
static bool mmi_has_current_tapered(int batt, struct smb_mmi_charger *chg,
				    struct mmi_sm_params *chip,
				    int batt_ma, int taper_ma)
{
	bool change_state = false;
	int allowed_fcc, target_ma;

	if (!chg) {
		mmi_dbg(chg, "called before chip valid!\n");
		return false;
	}

	allowed_fcc = get_effective_result(chg->fcc_votable) / 1000;

	if (allowed_fcc >= taper_ma)
		target_ma = taper_ma;
	else
		target_ma = allowed_fcc - TAPER_DROP_MA;

	if (batt == BASE_BATT)
		batt_ma *= -1;

	if (batt_ma < 0) {
		if (chip->chrg_taper_cnt >= TAPER_COUNT) {
			change_state = true;
			chip->chrg_taper_cnt = 0;
		} else
			chip->chrg_taper_cnt++;
	} else {
		if (batt_ma <= target_ma)
			if (chip->chrg_taper_cnt >= TAPER_COUNT) {
				change_state = true;
				chip->chrg_taper_cnt = 0;
			} else
				chip->chrg_taper_cnt++;
		else
			chip->chrg_taper_cnt = 0;
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
		mmi_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return flag;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (chg->smb_version == PM8150B ||
	    chg->smb_version == PM7250B ||
	    chg->smb_version == PMI632) {
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
		mmi_err(chg, "No Temp Zone Defined for batt %d!\n", batt);
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
	} else if (chg->demo_mode) { /* Demo Mode */
		chip->pres_chrg_step = STEP_DEMO;
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
		else if (mmi_has_current_tapered(batt, chg, chip, stat->batt_ma,
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
		} else if (mmi_has_current_tapered(batt, chg, chip, stat->batt_ma,
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
		mmi_info(chg, "Batt %d: batt_mv = %d, batt_ma %d, batt_soc %d,"
			" batt_temp %d, usb_mv %d, cp %d, vp %d\n",
			batt,
			stat->batt_mv,
			stat->batt_ma,
			stat->batt_soc,
			stat->batt_temp,
			stat->usb_mv,
			stat->charger_present,
			stat->vbus_present);
		mmi_info(chg, "Batt %d Step State = %s, Temp Zone %d, Health %d\n",
			batt,
			stepchg_str[(int)chip->pres_chrg_step],
			chip->pres_temp_zone,
			chip->batt_health);
		chg->hb_startup_cnt = HEARTBEAT_OCP_SETTLE_CNT;
		chg->ocp_flag = true;	// 1x settle per state chng
		return 1;
	} else {
		mmi_dbg(chg, "Batt %d: batt_mv = %d, batt_ma %d, batt_soc %d,"
			 " batt_temp %d, usb_mv %d, cp %d, vp %d\n",
			 batt,
			 stat->batt_mv,
			 stat->batt_ma,
			 stat->batt_soc,
			 stat->batt_temp,
			 stat->usb_mv,
			 stat->charger_present,
			 stat->vbus_present);
		mmi_dbg(chg, "Batt %d Step State = %s, Temp Zone %d, Health %d\n",
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
	int scaled_fcc =0;
	int ocp;
	int sm_update;
	bool voltage_full;
	int i;
	int sched_time = HEARTBEAT_DUAL_DELAY_MS;
	struct smb_mmi_chg_status chg_stat_main, chg_stat_flip;
	union power_supply_propval pval;
	struct mmi_sm_params *main_p = &chg->sm_param[MAIN_BATT];
	struct mmi_sm_params *flip_p = &chg->sm_param[FLIP_BATT];
	static int demo_full_soc = 100;
	bool is_chg_dis = get_effective_result(chg->chg_dis_votable);

	chg_stat_main.charger_present = stat->charger_present;
	chg_stat_flip.charger_present = stat->charger_present;
	chg_stat_main.vbus_present = stat->vbus_present;
	chg_stat_flip.vbus_present = stat->vbus_present;
	chg_stat_main.usb_mv = stat->usb_mv;
	chg_stat_flip.usb_mv = stat->usb_mv;

	rc = get_prop_batt_voltage_now(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Main Batt Voltage rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_mv = pval.intval / 1000;

	rc = get_prop_batt_voltage_now(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Flip Batt Voltage rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_mv = pval.intval / 1000;

	rc = get_prop_batt_current_now(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Main Batt Current rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_ma = (pval.intval / 1000) * -1;

	rc = get_prop_batt_current_now(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Flip Batt Current rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_ma = (pval.intval / 1000) * -1;

	rc = get_prop_batt_capacity(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Main Batt Capacity rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_soc = pval.intval;

	rc = get_prop_batt_capacity(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Flip Batt Capacity rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_flip.batt_soc = pval.intval;

	rc = get_prop_batt_temp(chg, chg->max_main_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Main Batt Temperature rc = %d\n", rc);
		return sched_time;
	} else
		chg_stat_main.batt_temp = pval.intval / 10;

	rc = get_prop_batt_temp(chg, chg->max_flip_psy, &pval);
	if (rc < 0) {
		mmi_err(chg, "Error getting Flip Batt Temperature rc = %d\n", rc);
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

	/* OCP Check - Scale down fcc if too high for initial set */
	if (chg->hb_startup_cnt > 1) {
		chg->hb_startup_cnt--;
		sched_time = HEARTBEAT_DUAL_DELAY_OCP_MS;
	} else if ( (effective_fcc >= (main_p->target_fcc + flip_p->target_fcc) *1000) &&
			(chg->ocp_flag) ) {
		chg->ocp_flag = false;
		sched_time = HEARTBEAT_DUAL_DELAY_OCP_MS;
		chg->hb_startup_cnt = HEARTBEAT_OCP_SETTLE_CNT;
		scaled_fcc = effective_fcc /10;
		effective_fcc -= scaled_fcc;
		mmi_dbg(chg, "fccScaled: %d, AdjFCC: %d\n", scaled_fcc, effective_fcc);
		main_p->ocp[main_p->pres_temp_zone] =0;
		flip_p->ocp[flip_p->pres_temp_zone] =0;
	}

	/* Check for Charge None */
	if ((main_p->pres_chrg_step == STEP_NONE) ||
	    (flip_p->pres_chrg_step == STEP_NONE)) {
		chg->sm_param[BASE_BATT].pres_chrg_step = STEP_NONE;
		target_fcc = main_p->target_fcc;
		target_fv = chg->base_fv_mv;
		sched_time = HEARTBEAT_DELAY_MS;
		goto vote_now;
	/* Check for Charge Demo */
	} else if ((main_p->pres_chrg_step == STEP_DEMO) ||
	    (flip_p->pres_chrg_step == STEP_DEMO)) {
		mmi_info(chg, "Battery in Demo Mode charging limited "
			"%d%%\n", chg->demo_mode);

		voltage_full = ((chg->demo_mode_usb_suspend == false) &&
		    ((stat->batt_mv + HYST_STEP_MV) >= DEMO_MODE_VOLTAGE)
		    && mmi_has_current_tapered(MAIN_BATT, chg, main_p, stat->batt_ma,
					       (main_p->chrg_iterm * 2)));

		if ((chg->demo_mode_usb_suspend == false) &&
		    ((chg->last_reported_soc >= chg->demo_mode) ||
		     voltage_full)) {
			demo_full_soc = chg->last_reported_soc;
			vote(chg->usb_icl_votable, DEMO_VOTER, true, 0);
			vote(chg->dc_suspend_votable, DEMO_VOTER, true, 1);
			chg->demo_mode_usb_suspend = true;
			main_p->chrg_taper_cnt = 0;
			flip_p->chrg_taper_cnt = 0;
			for (i = 0; i < MAX_NUM_STEPS; i++) {
				main_p->ocp[i] = 0;
				flip_p->ocp[i] = 0;
			}
		} else if (chg->demo_mode_usb_suspend == true &&
			   (chg->last_reported_soc <=
			    (demo_full_soc - DEMO_MODE_HYS_SOC))) {
			vote(chg->usb_icl_votable, DEMO_VOTER, false, 0);
			vote(chg->dc_suspend_votable, DEMO_VOTER, false, 0);
			chg->demo_mode_usb_suspend = false;
			main_p->chrg_taper_cnt = 0;
			flip_p->chrg_taper_cnt = 0;
			for (i = 0; i < MAX_NUM_STEPS; i++) {
				main_p->ocp[i] = 0;
				flip_p->ocp[i] = 0;
			}
		}

		target_fv = DEMO_MODE_VOLTAGE;
		target_fcc = main_p->target_fcc;
		goto vote_now;
	/* Check for Charge FULL from each */
	} else if ((main_p->pres_chrg_step == STEP_FULL) &&
		   (flip_p->pres_chrg_step == STEP_FULL)) {
		chg->sm_param[BASE_BATT].pres_chrg_step = STEP_FULL;
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
		chg->sm_param[BASE_BATT].pres_chrg_step =
			flip_p->pres_chrg_step;
		target_fcc = flip_p->target_fcc;
		target_fv = flip_p->target_fv;
		mmi_info(chg, "Align Flip to Main FULL\n");
		sched_time = HEARTBEAT_DUAL_DELAY_MS;
		goto vote_now;
	} else if ((flip_p->pres_chrg_step == STEP_FULL) &&
		   ((main_p->pres_chrg_step == STEP_MAX) ||
		    (main_p->pres_chrg_step == STEP_NORM)) &&
		   (chg_stat_main.batt_soc >= 95) &&
		   ((chg_stat_main.batt_mv + HYST_STEP_MV) >=
		    chg->base_fv_mv)) {
		chg->sm_param[BASE_BATT].pres_chrg_step =
			main_p->pres_chrg_step;
		target_fcc = main_p->target_fcc;
		target_fv = main_p->target_fv;
		mmi_info(chg, "Align Main to Flip FULL\n");
		sched_time = HEARTBEAT_DUAL_DELAY_MS;
		goto vote_now;
	/* Check for Charge Disable from each */
	} else if ((main_p->target_fcc < 0) ||
		   (flip_p->target_fcc < 0)) {
		chg->sm_param[BASE_BATT].pres_chrg_step = STEP_STOP;
		target_fcc = -EINVAL;
		target_fv = chg->base_fv_mv;
		goto vote_now;
	}

	chg->sm_param[BASE_BATT].pres_chrg_step = main_p->pres_chrg_step;
	chg->last_reported_status = -1;
	if (main_p->target_fv < flip_p->target_fv)
		target_fv = main_p->target_fv;
	else
		target_fv = flip_p->target_fv;

	if ((chg_stat_main.batt_ma > main_p->target_fcc) &&
		 chg->hb_startup_cnt %2) { //Allow settle time
		sched_time = HEARTBEAT_DUAL_DELAY_OCP_MS;
		ocp = chg_stat_main.batt_ma - main_p->target_fcc;
		main_p->ocp[main_p->pres_temp_zone] += ocp;
		mmi_info(chg, "Main Exceed by %d mA\n",
			main_p->ocp[main_p->pres_temp_zone]);
	}
	if ((chg_stat_flip.batt_ma > flip_p->target_fcc) &&
		 chg->hb_startup_cnt %2) { //Allow settle time
		sched_time = HEARTBEAT_DUAL_DELAY_OCP_MS;
		ocp = chg_stat_flip.batt_ma - flip_p->target_fcc;
		flip_p->ocp[flip_p->pres_temp_zone] += ocp;
		mmi_info(chg, "Flip Exceed by %d mA\n",
			flip_p->ocp[flip_p->pres_temp_zone]);
	}

	target_fcc = main_p->target_fcc + flip_p->target_fcc;
	target_fcc -= scaled_fcc/1000;
	target_fcc -= main_p->ocp[main_p->pres_temp_zone];
	target_fcc -= flip_p->ocp[flip_p->pres_temp_zone];
	if ( (target_fcc < main_p->target_fcc) &&
		(chg_stat_flip.batt_ma < flip_p->target_fcc *7/10) ) {
		mmi_info(chg, "Target FCC adjust too much\n");
		chg->hb_startup_cnt = HEARTBEAT_OCP_SETTLE_CNT;
		if (main_p->target_fcc < flip_p->target_fcc)
			target_fcc = (main_p->target_fcc)*5/2;
		else
			target_fcc = (flip_p->target_fcc)*5/2;
		main_p->ocp[main_p->pres_temp_zone] =0;
		flip_p->ocp[flip_p->pres_temp_zone] =0;
	}

	if (((main_p->pres_chrg_step == STEP_MAX) ||
	     (flip_p->pres_chrg_step == STEP_MAX) ||
	     (main_p->pres_chrg_step == STEP_NORM) ||
	     (flip_p->pres_chrg_step == STEP_NORM)) &&
	    mmi_charge_halted(chg)) {
		vote(chg->chg_dis_votable,
		     MMI_HB_VOTER, true, 0);
		mmi_err(chg, "Charge Halt..Toggle\n");
		msleep(50);
	}

vote_now:
	/* Votes for State */
	vote(chg->fv_votable, MMI_HB_VOTER, true, target_fv * 1000);

	vote(chg->chg_dis_votable, MMI_HB_VOTER,
	     (target_fcc < 0), 0);

	vote(chg->fcc_votable, MMI_HB_VOTER,
	     true, (target_fcc >= 0) ? (target_fcc * 1000) : 0);

	/* Rerun AICL to recover USB ICL from recharge */
	if (is_chg_dis != get_effective_result(chg->chg_dis_votable)) {
		smb_mmi_write_iio_chan(chg,
					  SMB5_RERUN_AICL, 0);
	}

	if (!stat->charger_present &&
	    (chg_stat_main.batt_mv <= LOW_BATT_MV) &&
	    (chg_stat_flip.batt_mv <= LOW_BATT_MV)) {
		if ((chg_stat_main.batt_mv <= SHUT_BATT_MV) ||
		    (chg_stat_flip.batt_mv <= SHUT_BATT_MV))
		    chg->shut_batt = true;
	} else
		chg->shut_batt = false;

	if (sm_update)
		mmi_info(chg, "IMPOSED: FV = %d, CDIS = %d, FCC = %d, USBICL = %d\n",
			effective_fv,
			get_effective_result(chg->chg_dis_votable),
			target_fcc,
			get_effective_result(chg->usb_icl_votable));
	else
		mmi_dbg(chg, "IMPOSED: FV = %d, CDIS = %d, FCC = %d, USBICL = %d\n",
			effective_fv,
			get_effective_result(chg->chg_dis_votable),
			target_fcc,
			get_effective_result(chg->usb_icl_votable));

	return sched_time;
}

static int mmi_get_ffc_fv(struct smb_mmi_charger *chip, int zone)
{
       int rc;
       int ffc_max_fv;
	struct mmi_sm_params *prm = &chip->sm_param[BASE_BATT];

       if (prm->ffc_zones == NULL || zone >= prm->num_temp_zones)
               return 0;

	rc = smb_mmi_write_iio_chan(chip,
			SMB5_QG_BATT_FULL_CURRENT, prm->ffc_zones[zone].ffc_qg_iterm);
	if (rc < 0) {
		mmi_err(chip, "Couldn't set batt full current, rc=%d\n", rc);
	}

       prm->chrg_iterm = prm->ffc_zones[zone].ffc_chg_iterm;
       ffc_max_fv = prm->ffc_zones[zone].ffc_max_mv;
       mmi_info(chip,
               "FFC temp zone %d, fv %d mV, chg iterm %d mA, qg iterm %d mA\n",
                 zone, ffc_max_fv, prm->chrg_iterm, prm->ffc_zones[zone].ffc_qg_iterm);

       return ffc_max_fv;
}

static void mmi_basic_charge_sm(struct smb_mmi_charger *chip,
				struct smb_mmi_chg_status *stat)
{
	int target_fcc;
	int target_fv;
	int max_fv_mv;
	struct mmi_temp_zone *zone;
	struct mmi_sm_params *prm = &chip->sm_param[BASE_BATT];
	bool voltage_full;
	bool is_chg_dis = get_effective_result(chip->chg_dis_votable);
	static int demo_full_soc = 100;

	mmi_info(chip, "batt_mv = %d, batt_ma %d, batt_soc %d,"
		" batt_temp %d, usb_mv %d, dc_mv %d, cp %d, vp %d dp %d\n",
		stat->batt_mv,
		stat->batt_ma,
		stat->batt_soc,
		stat->batt_temp,
		stat->usb_mv,
		stat->dc_mv,
		stat->charger_present,
		stat->vbus_present,
		stat->dc_present);

	if (!prm->temp_zones) {
		mmi_dbg(chip, "Skipping SM since No Temp Zone Defined!\n");
		return;
	}

	if (chip->base_fv_mv == 0) {
		chip->base_fv_mv = get_effective_result(chip->fv_votable);
		chip->base_fv_mv /= 1000;
		vote(chip->fv_votable,
		     BATT_PROFILE_VOTER, false, 0);
	}
	max_fv_mv = mmi_get_ffc_fv(chip, prm->pres_temp_zone);
       if (max_fv_mv == 0)
               max_fv_mv = chip->base_fv_mv;

	mmi_find_temp_zone(chip, prm, stat->batt_temp);
	if (prm->pres_temp_zone >=  prm->num_temp_zones)
		zone = &prm->temp_zones[0];
	else
		zone = &prm->temp_zones[prm->pres_temp_zone];

	if (!stat->charger_present && !is_wls_online(chip)) {
		prm->pres_chrg_step = STEP_NONE;
	} else if ((prm->pres_temp_zone == ZONE_HOT) ||
		   (prm->pres_temp_zone == ZONE_COLD) ||
		   (chip->charging_limit_modes == CHARGING_LIMIT_RUN)) {
		prm->pres_chrg_step = STEP_STOP;
	} else if (chip->demo_mode) { /* Demo Mode */
		prm->pres_chrg_step = STEP_DEMO;
		mmi_info(chip, "Battery in Demo Mode charging limited "
			"%d%%\n", chip->demo_mode);

		voltage_full = ((chip->demo_mode_usb_suspend == false) &&
		    ((stat->batt_mv + HYST_STEP_MV) >= DEMO_MODE_VOLTAGE)
		    && mmi_has_current_tapered(BASE_BATT, chip, prm, stat->batt_ma,
			prm->chrg_iterm));

		if ((chip->demo_mode_usb_suspend == false) &&
		    ((stat->batt_soc >= chip->demo_mode) || voltage_full)) {
			demo_full_soc = stat->batt_soc;
			vote(chip->usb_icl_votable, DEMO_VOTER, true, 0);
			vote(chip->dc_suspend_votable, DEMO_VOTER, true, 0);
			chip->demo_mode_usb_suspend = true;
		} else if (chip->demo_mode_usb_suspend == true &&
		    (stat->batt_soc <= (demo_full_soc - DEMO_MODE_HYS_SOC))) {
			vote(chip->usb_icl_votable, DEMO_VOTER, false, 0);
			vote(chip->dc_suspend_votable, DEMO_VOTER, false, 0);
			chip->demo_mode_usb_suspend = false;
			prm->chrg_taper_cnt = 0;
		}
	} else if ((prm->pres_chrg_step == STEP_NONE) ||
		   (prm->pres_chrg_step == STEP_STOP)) {
		if (zone->norm_mv && ((stat->batt_mv + HYST_STEP_MV) >= zone->norm_mv)) {
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
		else if (mmi_has_current_tapered(BASE_BATT, chip, prm, stat->batt_ma,
						 zone->fcc_norm_ma)) {
			prm->chrg_taper_cnt = 0;

			if (mmi_charge_halted(chip)) {
				vote(chip->chg_dis_votable,
				     MMI_HB_VOTER, true, 0);
				mmi_warn(chip, "Charge Halt..Toggle\n");
				msleep(50);
			}

			prm->pres_chrg_step = STEP_NORM;
		}
	} else if (prm->pres_chrg_step == STEP_NORM) {
		if (!zone->fcc_norm_ma)
			prm->pres_chrg_step = STEP_FLOAT;
		else if ((stat->batt_soc < 100) ||
			 (stat->batt_mv + HYST_STEP_MV) < max_fv_mv) {
			prm->chrg_taper_cnt = 0;
			prm->pres_chrg_step = STEP_NORM;
		} else if (mmi_has_current_tapered(BASE_BATT, chip, prm, stat->batt_ma,
						   prm->chrg_iterm)) {
				prm->pres_chrg_step = STEP_FULL;
		}
	} else if (prm->pres_chrg_step == STEP_FULL) {
		if ((stat->batt_soc <= 99) ||
			stat->batt_mv < (max_fv_mv - HYST_STEP_MV * 2)) {
			prm->chrg_taper_cnt = 0;
			prm->pres_chrg_step = STEP_NORM;
		}
	} else if (prm->pres_chrg_step == STEP_FLOAT) {
		if ((zone->fcc_norm_ma) ||
		    ((stat->batt_mv + HYST_STEP_MV) < zone->norm_mv))
			prm->pres_chrg_step = STEP_MAX;
		else if (mmi_has_current_tapered(BASE_BATT, chip, prm, stat->batt_ma,
						   prm->chrg_iterm))
			prm->pres_chrg_step = STEP_STOP;
	}

	/* Take State actions */
	switch (prm->pres_chrg_step) {
	case STEP_FLOAT:
	case STEP_MAX:
		if (!zone->norm_mv)
			target_fv = max_fv_mv + chip->vfloat_comp_mv;
		else
			target_fv = zone->norm_mv + chip->vfloat_comp_mv;
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
		if (is_wls_online(chip))
			target_fcc = zone->fcc_norm_ma;
		else if (stat->dc_present)
			target_fcc = 500;
		else
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
	if (stat->dc_present)
		vote(chip->dc_suspend_votable, MMI_HB_VOTER, false, 0);
	else
		vote(chip->dc_suspend_votable, MMI_HB_VOTER, true, 1);

	vote(chip->fv_votable, MMI_HB_VOTER, true, target_fv * 1000);

	vote(chip->chg_dis_votable, MMI_HB_VOTER,
	     (target_fcc < 0), 0);

	vote(chip->fcc_votable, MMI_HB_VOTER,
	     true, (target_fcc >= 0) ? (target_fcc * 1000) : 0);

	/* Override USB ICL using the target value */
	if (stat->charger_present) {
		mmi_usb_icl_override(chip,
		get_effective_result(chip->usb_icl_votable));
	}

	/* Rerun AICL to recover USB ICL from recharge */
	if (is_chg_dis != get_effective_result(chip->chg_dis_votable)) {
		smb_mmi_write_iio_chan(chip,
				SMB5_RERUN_AICL, 0);
	}

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

	mmi_info(chip, "Step State = %s, Temp Zone %d, Health %d\n",
		stepchg_str[(int)prm->pres_chrg_step],
		prm->pres_temp_zone,
		prm->batt_health);
	mmi_info(chip, "IMPOSED: FV = %d, CDIS = %d, FCC = %d, USBICL = %d\n",
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

static void smb_mmi_power_supply_changed(struct power_supply *psy,
					 char *envp_ext[])
{
	if (this_chip && this_chip->prev_chg_rate != this_chip->charger_rate) {
		dev_err(&psy->dev, "SMBMMI: %s: %s\n", __func__, envp_ext[0]);
	}

	kobject_uevent_env(&psy->dev.kobj, KOBJ_CHANGE, envp_ext);
	power_supply_changed(psy);
}

static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);
#define TWO_VOLT 2000000
#define MONOTONIC_SOC 2 /* 2 percent */
static void mmi_heartbeat_work(struct work_struct *work)
{
	struct smb_mmi_charger *chip = container_of(work,
						struct smb_mmi_charger,
						heartbeat_work.work);
	int hb_resch_time = HEARTBEAT_DELAY_MS;
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
	int pres_chrg_step, prev_chrg_step;
	static int prev_vbus_mv = -1;
	char *chrg_rate_string = NULL;
	char *envp[2];

	struct smb_mmi_chg_status chg_stat;

	/* Have not been resumed so wait another 100 ms */
	if (chip->suspended & IS_SUSPENDED) {
		mmi_err(chip, "HB running before Resume\n");
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));
		return;
	}

	smb_mmi_awake_vote(chip, true);

	mmi_dbg(chip, "Heartbeat!\n");

	chg_stat.charger_present = false;
	rc = get_prop_batt_voltage_now(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Voltage rc = %d\n", rc);
		goto sch_hb;
	} else
		chg_stat.batt_mv = pval.intval / 1000;

	rc = get_prop_batt_current_now(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Current rc = %d\n", rc);
		goto sch_hb;
	} else
		chg_stat.batt_ma = pval.intval / 1000;

	rc = get_prop_usb_voltage_now(chip, &pval);
	if (rc < 0) {
		chg_stat.usb_mv = 0;
		if (rc != -ENODATA)
			mmi_err(chip, "Error getting USB Voltage rc = %d\n", rc);
	} else
		chg_stat.usb_mv = pval.intval / 1000;

	if (prev_vbus_mv == -1)
		prev_vbus_mv = chg_stat.usb_mv;

	rc = get_prop_dc_voltage_now(chip, &pval);
	if (rc < 0) {
		mmi_dbg(chip, "Error getting DC Voltage rc = %d\n", rc);
		chg_stat.dc_mv = 0;
	} else
		chg_stat.dc_mv = pval.intval / 1000;

	rc = get_prop_batt_capacity(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Capacity rc = %d\n", rc);
		return;
	} else
		chg_stat.batt_soc = pval.intval;

	if (chip->last_reported_soc != -1)
		chg_stat.batt_soc = chip->last_reported_soc;

	rc = get_prop_batt_temp(chip, chip->bms_psy, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Batt Temperature rc = %d\n", rc);
		return;
	} else
		chg_stat.batt_temp = pval.intval / 10;

	rc = get_prop_usb_present(chip, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting USB Present rc = %d\n", rc);
		return;
	} else
		chg_stat.vbus_present = pval.intval;

	rc = get_prop_dc_present(chip, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting DC Present rc = %d\n", rc);
		return;
	} else
		chg_stat.dc_present = pval.intval;

	rc = get_prop_charger_present(chip, &pval);
	if (rc < 0) {
		mmi_err(chip, "Error getting Charger Present rc = %d\n", rc);
		return;
	} else
		chg_stat.charger_present = pval.intval & chg_stat.vbus_present;

	if (chip->factory_mode)
		hb_resch_time = HEARTBEAT_FACTORY_MS;
	else if (chg_stat.charger_present || chg_stat.dc_present)
		hb_resch_time = HEARTBEAT_DELAY_MS;
	else
		hb_resch_time = HEARTBEAT_DISCHARGE_MS;

	mmi_chrg_rate_check(chip);

	if (chip->vbus_enabled && chip->vbus && chg_stat.charger_present) {
		rc = regulator_disable(chip->vbus);
		if (rc)
			mmi_err(chip, "Unable to disable vbus (%d)\n", rc);
		else {
			chip->vbus_enabled = false;
			mmi_info(chip, "VBUS Disable due to Charger\n");
		}
	}

	if (chip->enable_charging_limit && chip->is_factory_image)
		update_charging_limit_modes(chip, chg_stat.batt_soc);

	if (chip->charging_limit_modes == CHARGING_LIMIT_RUN)
		mmi_warn(chip, "Factory Mode/Image so Limiting Charging!!!\n");

	mmi_chrg_input_config(chip, &chg_stat);

	if (chip->max_main_psy && chip->max_flip_psy) {
		cap_err = 0;
		rc = power_supply_get_property(chip->max_main_psy,
					       POWER_SUPPLY_PROP_CAPACITY,
					       &pval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get maxim main capacity\n");
			cap_err = rc;
		} else if (pval.intval <0) {
			cap_err = -EAGAIN;			// TI FG RESET may return neg cap prior to init, so delay
		} else
			main_cap = pval.intval;

		rc = power_supply_get_property(chip->max_main_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
					&pval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get maxim main chrg full\n");
			cap_err = rc;
		} else
			main_cap_full = pval.intval;

		rc = power_supply_get_property(chip->max_flip_psy,
					       POWER_SUPPLY_PROP_CAPACITY,
					       &pval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get maxim flip capacity\n");
			cap_err = rc;
		} else if (pval.intval <0) {
			cap_err = -EAGAIN;			// TI FG RESET may return neg cap prior to init, so delay
		} else
			flip_cap = pval.intval;

		rc = power_supply_get_property(chip->max_flip_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
					&pval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get maxim flip chrg full\n");
			cap_err = rc;
		} else
			flip_cap_full = pval.intval;

		if (cap_err == -EAGAIN) {
			mmi_err(chip, "The FGs are not ready yet\n");
			hb_resch_time = HEARTBEAT_DUAL_DELAY_MS;
			goto sch_hb;
		}

		report_cap = main_cap * main_cap_full;
		report_cap += flip_cap * flip_cap_full;
		report_cap /= main_cap_full + flip_cap_full;

		if (chip->shut_batt || (report_cap < 0))
			report_cap = 0;
		else if (report_cap > 100)
			report_cap = 100;

		if (chip->suspended & WAS_SUSPENDED)
			batt_cap = -1;
		else
			batt_cap = chip->last_reported_soc;

		if ((batt_cap == -1) ||
		    ((report_cap != batt_cap) &&
		     (report_cap <= (batt_cap + MONOTONIC_SOC)) &&
		     (report_cap >= (batt_cap - MONOTONIC_SOC)))) {
			mmi_info(chip, "Updating Reported Capacity to %d, main_cap= %d, flip_cap= %d, main_cap_full= %d, flip_cap_full= %d\n",
				report_cap, main_cap, flip_cap, main_cap_full, flip_cap_full);
			chip->last_reported_soc = report_cap;
		}else if ((batt_cap < 100) && (report_cap > (batt_cap + MONOTONIC_SOC))) {
			chip->last_reported_soc++;
			mmi_info(chip, "Alter Up Reported Capacity to %d target %d\n",
				chip->last_reported_soc, report_cap);
		}else if ((batt_cap > 0) && (report_cap < (batt_cap - MONOTONIC_SOC))) {
			chip->last_reported_soc--;
			mmi_info(chip, "Alter Down Reported Capacity to %d target %d\n",
				chip->last_reported_soc, report_cap);
		}

		/* Age calculation */
		rc = power_supply_get_property(chip->max_main_psy,
					       POWER_SUPPLY_PROP_CHARGE_FULL,
					       &pval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get maxim main charge full\n");
			cap_err = rc;
		} else
			main_cap = pval.intval;

		rc = power_supply_get_property(chip->max_flip_psy,
					       POWER_SUPPLY_PROP_CHARGE_FULL,
					       &pval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get maxim flip charge full\n");
			cap_err = rc;
		} else
			flip_cap = pval.intval;

		main_age = ((main_cap / 10) / (main_cap_full / 1000));
		flip_age = ((flip_cap / 10) / (flip_cap_full / 1000));

		/* Block age output for now until FG can be vetted */
		if (cap_err == 0)
			chip->age = 100;

		mmi_dbg(chip, "Age %d, Main Age %d, Flip Age %d\n",
			 chip->age, main_age, flip_age);

		prev_chrg_step = chip->sm_param[BASE_BATT].pres_chrg_step;

		/* Dual Step and Thermal Charging */
		hb_resch_time = mmi_dual_charge_control(chip, &chg_stat);

		/* Check to see if Voltage is shutdown */
		if (chip->shut_batt)
			chip->last_reported_soc = 0;

		pres_chrg_step = chip->sm_param[BASE_BATT].pres_chrg_step;

		if ((prev_chrg_step == STEP_NONE) &&
		    ((pres_chrg_step == STEP_MAX) ||
		     (pres_chrg_step == STEP_NORM)))
			chip->soc_cycles_start = chip->last_reported_soc;
		else if ((pres_chrg_step == STEP_NONE) &&
			 ((prev_chrg_step == STEP_MAX) ||
			  (prev_chrg_step == STEP_NORM)) &&
			 (chip->last_reported_soc > chip->soc_cycles_start)) {

			chip->cycles += (chip->last_reported_soc -
					 chip->soc_cycles_start);
			chip->soc_cycles_start = EMPTY_CYCLES;

		} else if ((pres_chrg_step == STEP_FULL) &&
			   ((prev_chrg_step == STEP_MAX) ||
			    (prev_chrg_step == STEP_NORM)) &&
			   (100 > chip->soc_cycles_start)) {

			chip->cycles += (100 - chip->soc_cycles_start);
			chip->soc_cycles_start = EMPTY_CYCLES;
		}

	} else if (!chip->factory_mode) {
		cap_err = 0;
		rc = smb_mmi_read_iio_chan(chip,
					       SMB5_QG_CHARGE_FULL,
					       &pval.intval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get charge full\n");
			cap_err = rc;
		} else
			main_cap = pval.intval;

		rc = smb_mmi_read_iio_chan(chip,
					SMB5_QG_CHARGE_FULL_DESIGN,
					&pval.intval);
		if (rc < 0) {
			mmi_err(chip, "Couldn't get charge full design\n");
			cap_err = rc;
		} else
			main_cap_full = pval.intval;

		if (cap_err == 0)
			chip->age = ((main_cap / 10) / (main_cap_full / 1000));

		mmi_dbg(chip, "Age %d\n", chip->age);

		/* Fall here for Basic Step and Thermal Charging */
		mmi_basic_charge_sm(chip, &chg_stat);
	}

	mmi_dbg(chip, "SMBMMI: batt_mv %d, usb_mv %d, prev_usb_mv %d batt_ma %d\n",
		 chg_stat.batt_mv, chg_stat.usb_mv,
		 prev_vbus_mv, chg_stat.batt_ma);

	rc = smblib_get_usb_suspend(chip, &usb_suspend);
	if (rc < 0) {
		usb_suspend = 0;
		mmi_err(chip, "Couldn't get USB suspend rc = %d\n", rc);
	}
	if (!usb_suspend &&
	    (abs(chg_stat.usb_mv - chg_stat.batt_mv) < REV_BST_BULK_DROP) &&
	    ((chg_stat.usb_mv*1000) > TWO_VOLT)) {
		if (((chg_stat.usb_mv < REV_BST_THRESH) &&
		    ((prev_vbus_mv - REV_BST_DROP) > chg_stat.usb_mv)) ||
		    (chg_stat.batt_ma > REV_BST_MA)) {
			mmi_err(chip, "Reverse Boosted: Clear, USB Suspend. usb_mv: %d prev_vbus_mv: %d batt_ma: %d \n", chg_stat.usb_mv, prev_vbus_mv, chg_stat.batt_ma);
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
			mmi_err(chip, "Reverse Boosted: USB %d mV PUSB %d mV\n",
				   chg_stat.usb_mv, prev_vbus_mv);
		}
	}
	prev_vbus_mv = chg_stat.usb_mv;
	chip->suspended = 0;

	if (chip->factory_mode ||
	    (chip->is_factory_image && chip->enable_factory_poweroff)) {
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

		mmi_dbg(chip, "Factory Kill check pc %d, usb %d, susp %d\n",
			 pc_online, pval.intval, usb_suspend);
		if (pc_online ||
		    pval.intval ||
		    (usb_suspend && ((chg_stat.usb_mv*1000) > TWO_VOLT))) {
			mmi_dbg(chip, "Factory Kill Armed\n");
			chip->factory_kill_armed = true;
		} else if (chip->factory_kill_armed && !factory_kill_disable) {
			mmi_warn(chip, "Factory kill power off\n");
			orderly_poweroff(true);
		} else
			chip->factory_kill_armed = false;
	}

sch_hb:
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));

	if (!chg_stat.charger_present)
		smb_mmi_awake_vote(chip, false);

	chrg_rate_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (!chrg_rate_string) {
		mmi_err(chip, "Failed to Get Uevent Mem\n");
		envp[0] = NULL;
	} else {
		scnprintf(chrg_rate_string, CHG_SHOW_MAX_SIZE,
			  "POWER_SUPPLY_CHARGE_RATE=%s",
			  charge_rate[chip->charger_rate]);
		envp[0] = chrg_rate_string;
		envp[1] = NULL;
	}

	if (chip->batt_psy) {
		smb_mmi_power_supply_changed(chip->batt_psy, envp);
	} else if (chip->qcom_psy) {
		smb_mmi_power_supply_changed(chip->qcom_psy, envp);
	}

	kfree(chrg_rate_string);

}

static int mmi_psy_notifier_call(struct notifier_block *nb, unsigned long val,
				 void *v)
{
	struct smb_mmi_charger *chip = container_of(nb,
				struct smb_mmi_charger, mmi_psy_notifier);
	struct power_supply *psy = v;
	static bool first_boot = true;

	if (!chip) {
		pr_err("SMBMMI: called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (psy &&
	    ((strcmp(psy->desc->name, "usb") == 0) ||
		(strcmp(psy->desc->name, "wireless") == 0) ||
		(strcmp(psy->desc->name, "dc") == 0))) {

		mmi_info(chip, "Psy notifier call %s\n", psy->desc->name);
		if(first_boot && chip->factory_mode)
		{
			//In factory mode for the first heartbeat make sure to wait 10 second until boot is complete
			cancel_delayed_work(&chip->heartbeat_work);
			schedule_delayed_work(&chip->heartbeat_work,
					      msecs_to_jiffies(10000));
			first_boot = false;
		}
		else
		{
			cancel_delayed_work(&chip->heartbeat_work);
			schedule_delayed_work(&chip->heartbeat_work,
					      msecs_to_jiffies(0));
			first_boot = false;
		}
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
		pr_err("SMBMMI: called before chip valid!\n");
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
				mmi_info(chg, "Wait for VBUS to decay\n");
			}

			mmi_info(chg, "VBUS UV wait 1 sec!\n");
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
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= smb_mmi_get_property,
	.set_property	= smb_mmi_set_property,
	.properties	= smb_mmi_battery_props,
	.num_properties	= ARRAY_SIZE(smb_mmi_battery_props),
};

static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
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
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->cycles / 100;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chip->max_main_psy && chip->max_flip_psy)
			val->intval = chip->sm_param[MAIN_BATT].batt_health;
		else
			val->intval = chip->sm_param[BASE_BATT].batt_health;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = get_effective_result(chip->fcc_votable);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (chip->smb_version == PMI632) {
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
	case POWER_SUPPLY_PROP_TEMP:
		if (chip->max_main_psy && chip->max_flip_psy) {
			union power_supply_propval main_psy_val;
			union power_supply_propval flip_psy_val;

			/* Get the highest temp between main psy and flip psy */
			rc = power_supply_get_property(chip->max_main_psy, psp, &main_psy_val);
			if (rc >= 0) {
				rc = power_supply_get_property(chip->max_flip_psy, psp, &flip_psy_val);
				if (rc >= 0) {
					if (main_psy_val.intval > flip_psy_val.intval)
						val->intval = main_psy_val.intval;
					else
						val->intval = flip_psy_val.intval;
					break;
				}
			}
		}
		/* Fall through */
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
		chip->cycles += val->intval * 100;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval < 0) {
			vote(chip->fcc_votable, MMI_VOTER, false, 0);
		} else
			vote(chip->fcc_votable, MMI_VOTER, true, val->intval);
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
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
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
		mmi_err(chg, "mmi dtree info. missing\n");
		return -ENODEV;
	}

	chip = &chg->sm_param[BASE_BATT];
	if (of_find_property(node, "qcom,mmi-temp-zones", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			mmi_err(chg, "DT error wrong mmi temp zones\n");
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
			mmi_err(chg, "Couldn't read mmi temp zones rc = %d\n", rc);
			return rc;
		}

		mmi_info(chg, "mmi temp zones: Num: %d\n", chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			mmi_info(chg, "mmi temp zones: Zone %d, Temp %d C, " \
				"Step Volt %d mV, Full Rate %d mA, " \
				"Taper Rate %d mA\n", i,
				chip->temp_zones[i].temp_c,
				chip->temp_zones[i].norm_mv,
				chip->temp_zones[i].fcc_max_ma,
				chip->temp_zones[i].fcc_norm_ma);
		}
		chip->pres_temp_zone = ZONE_NONE;
		chip->pres_chrg_step = STEP_NONE;
	}

	if (of_find_property(node, "qcom,mmi-ffc-zones", &byte_len)) {
               if ((byte_len / sizeof(struct mmi_ffc_zone) != chip->num_temp_zones)
                      || ((byte_len / sizeof(u32)) % 3)) {
                       mmi_err(chg, "DT error wrong mmi ffc zones\n");
                       return -ENODEV;
               }

               chip->ffc_zones = (struct mmi_ffc_zone *)
                       devm_kzalloc(chg->dev, byte_len, GFP_KERNEL);

              if (chip->ffc_zones == NULL)
                       return -ENOMEM;

               rc = of_property_read_u32_array(node,
                               "qcom,mmi-ffc-zones",
                               (u32 *)chip->ffc_zones,
                               byte_len / sizeof(u32));
               if (rc < 0) {
                       mmi_err(chg, "Couldn't read mmi ffc zones rc = %d\n", rc);
                       return rc;
              }

               for (i = 0; i < chip->num_temp_zones; i++) {
                       mmi_info(chg, "FFC:Zone %d,Volt %d,Ich %d,Iqg %d", i,
                                chip->ffc_zones[i].ffc_max_mv,
                                chip->ffc_zones[i].ffc_chg_iterm,
                                chip->ffc_zones[i].ffc_qg_iterm);
               }
       } else
               chip->ffc_zones = NULL;

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

	rc = of_property_read_u32(node, "qcom,dc-icl-ma",
				  &chg->dc_cl_ma);
	if (rc)
		chg->dc_cl_ma = -EINVAL;

	mmi_charger_power_support(chg);

	rc = of_property_read_u32(node, "qcom,inc-hvdcp-cnt",
				  &chg->inc_hvdcp_cnt);
	if (rc)
		chg->inc_hvdcp_cnt = HVDCP_PULSE_COUNT_MAX;

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
		mmi_err(chg, "mmi dtree info. missing\n");
		return -ENODEV;
	}

	chip = &chg->sm_param[MAIN_BATT];
	if (of_find_property(node, "qcom,mmi-temp-zones-main", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			mmi_err(chg, "DT error wrong mmi temp zones\n");
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
			mmi_err(chg, "Couldn't read mmi temp zones rc = %d\n", rc);
			return rc;
		}

		mmi_info(chg, "mmi temp zones main: Num: %d\n", chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			mmi_info(chg, "mmi temp zones: Zone %d, Temp %d C, " \
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
			mmi_err(chg, "DT error wrong mmi temp zones\n");
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
			mmi_err(chg, "Couldn't read mmi temp zones rc = %d\n", rc);
			return rc;
		}

		mmi_info(chg, "mmi temp zones main: Num: %d\n", chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			mmi_info(chg, "mmi temp zones: Zone %d, Temp %d C, " \
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
	int subtype = (u8)of_device_get_match_data(chip->dev);

	switch (subtype) {
	case PM8150B:
		chip->smb_version = PM8150B;
		chip->name = "pm8150b_charger";
		chip->param = smb5_pm8150b_params;
		break;
	case PM7250B:
		chip->smb_version = PM7250B;
		chip->name = "pm7250b_charger";
		chip->param = smb5_pm8150b_params;
		break;
	case PMI632:
		chip->smb_version = PMI632;
		chip->name = "pmi632_charger";
		chip->param = smb5_pmi632_params;
		break;
	default:
		pr_err("SMBMMI: PMIC subtype %d not supported\n",
				subtype);
		return -EINVAL;
	}

	chip->chg_freq.freq_5V			= 600;
	chip->chg_freq.freq_6V_8V		= 800;
	chip->chg_freq.freq_9V			= 1050;
	chip->chg_freq.freq_12V         = 1200;

	pr_err("SMBMMI: PMIC %d is %s\n", chip->smb_version, chip->name);

	return 0;
}

static ssize_t charge_rate_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	if (!this_chip) {
		pr_err("SMBMMI: mmi_chip is not initialized\n");
		return 0;
	}

	mmi_chrg_rate_check(this_chip);
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n",
			 charge_rate[this_chip->charger_rate]);
}
static DEVICE_ATTR(charge_rate, S_IRUGO, charge_rate_show, NULL);

static ssize_t age_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	if (!this_chip) {
		pr_err("SMBMMI: mmi_chip is not initialized\n");
		return 0;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->age);
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
		pr_err("SMBMMI: Couldn't create charger debugfs rc=%ld\n",
			(long)dfs_root);
		return;
	}

	file = debugfs_create_file("register_dump",
			    S_IRUSR | S_IRGRP | S_IROTH,
			    dfs_root, chip, &register_dump_debugfs_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("SMBMMI: Couldn't create register_dump file rc=%ld\n",
			(long)file);
}
#else
static void smb_mmi_create_debugfs(struct smb_mmi_charger *chip)
{}
#endif

static int mmi_smbcharger_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct smb_mmi_charger *chip = iio_priv(indio_dev);
	int rc = 0;

	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		chip->cp_active = !!val1;
		break;
	default:
		pr_err("Unsupported mmi_charger IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int mmi_smbcharger_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct smb_mmi_charger *chip = iio_priv(indio_dev);
	int rc = 0;

	*val1 = 0;

	switch (chan->channel) {
	case PSY_IIO_CP_ENABLE:
		*val1 = chip->cp_active;
		break;
	default:
		pr_err("Unsupported mmi_charger IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("Couldn't read IIO channel %d, rc = %d\n",
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int mmi_smbcharger_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct smb_mmi_charger *chip = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = chip->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(mmi_smbcharger_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info mmi_smbcharger_iio_info = {
	.read_raw	= mmi_smbcharger_iio_read_raw,
	.write_raw	= mmi_smbcharger_iio_write_raw,
	.of_xlate	= mmi_smbcharger_iio_of_xlate,
};

static int smb_mmi_init_iio_psy(struct smb_mmi_charger *chip,
				struct platform_device *pdev)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int mmi_smbcharger_num_iio_channels = ARRAY_SIZE(mmi_smbcharger_iio_psy_channels);
	int rc, i;

	chip->iio_chan = devm_kcalloc(chip->dev, mmi_smbcharger_num_iio_channels,
				sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan)
		return -ENOMEM;

	chip->int_iio_chans = devm_kcalloc(chip->dev,
				mmi_smbcharger_num_iio_channels,
				sizeof(*chip->int_iio_chans),
				GFP_KERNEL);
	if (!chip->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &mmi_smbcharger_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->name = "mmi-smbcharger-iio";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = mmi_smbcharger_num_iio_channels;

	for (i = 0; i < mmi_smbcharger_num_iio_channels; i++) {
		chip->int_iio_chans[i].indio_dev = indio_dev;
		chan = &chip->iio_chan[i];
		chip->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = mmi_smbcharger_iio_psy_channels[i].channel_num;
		chan->type = mmi_smbcharger_iio_psy_channels[i].type;
		chan->datasheet_name =
			mmi_smbcharger_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			mmi_smbcharger_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			mmi_smbcharger_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc) {
		pr_err("Failed to register mmi-smbcharger-iio IIO device, rc=%d\n", rc);
		return rc;
	}

	chip->ext_iio_chans = devm_kcalloc(chip->dev,
				ARRAY_SIZE(smb_mmi_ext_iio_chan_name),
				sizeof(*chip->ext_iio_chans),
				GFP_KERNEL);
	if (!chip->ext_iio_chans)
		return -ENOMEM;

	return rc;
}

static int smb_mmi_probe(struct platform_device *pdev)
{
	struct smb_mmi_charger *chip;
	struct iio_dev *indio_dev;
	int rc = 0;
	union power_supply_propval val;
	struct power_supply_config psy_cfg = {};
	const char *max_main_name, *max_flip_name;
	union power_supply_propval pval;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	if (!chip) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for mmi_smbcharger_iio\n");
		return -ENOMEM;
	}
	chip->indio_dev = indio_dev;

	chip->dev = &pdev->dev;
	psy_cfg.drv_data = chip;
	platform_set_drvdata(pdev, chip);
	chip->suspended = 0;
	chip->awake = false;
	this_chip = chip;
	device_init_wakeup(chip->dev, true);
	chip->hb_startup_cnt = HEARTBEAT_OCP_SETTLE_CNT;
	chip->ocp_flag = true;

	smb_mmi_chg_config_init(chip);

	chip->debug_enabled = &debug_enabled;
	chip->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, "charger", 0);
	if (!chip->ipc_log)
		mmi_err(chip, "Failed to create SMBMMI IPC log\n");
	else
		mmi_info(chip, "IPC logging is enabled for SMBMMI\n");

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		mmi_err(chip, "Parent regmap is missing\n");
		return -EINVAL;
	}

	smb_mmi_init_iio_psy(chip, pdev);

	parse_mmi_dt(chip);

	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_heartbeat_work);
	INIT_DELAYED_WORK(&chip->weakcharger_work, mmi_weakcharger_work);

	chip->mmi_psy = devm_power_supply_register(chip->dev, &mmi_psy_desc,
						   &psy_cfg);
	if (IS_ERR(chip->mmi_psy)) {
		mmi_err(chip, "failed: mmi power supply register\n");
		return PTR_ERR(chip->mmi_psy);
	}

	chip->age = 100;
	chip->cycles = 0;
	chip->shut_batt = false;
	chip->soc_cycles_start = EMPTY_CYCLES;
	chip->last_reported_soc = -1;
	chip->last_reported_status = -1;
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

		rc = sysfs_create_group(&chip->batt_psy->dev.kobj,
					&power_supply_mmi_attr_group);
		if (rc)
			mmi_err(chip, "Failed: attr create\n");
	} else {
		chip->qcom_psy = power_supply_get_by_name("battery");
		chip->batt_psy = NULL;
		if (chip->qcom_psy) {
			rc = sysfs_create_group(&chip->qcom_psy->dev.kobj,
						&power_supply_mmi_attr_group);
			if (rc)
				mmi_err(chip, "Failed: attr create\n");
		} else {
			mmi_err(chip, "Failed: SMB MMI probed failed, wait SMB5 prob\n");
			return -EPROBE_DEFER;
		}
	}

	chip->bms_psy = power_supply_get_by_name("bms");
	chip->usb_psy = power_supply_get_by_name("usb");
	chip->pc_port_psy = power_supply_get_by_name("pc_port");
	chip->dc_psy = power_supply_get_by_name("dc");
	chip->wls_psy = power_supply_get_by_name("wireless");
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
	chip->dc_suspend_votable = find_votable("DC_SUSPEND");
	if (IS_ERR(chip->dc_suspend_votable))
		chip->dc_suspend_votable = NULL;

	if (chip->smb_version == PM8150B) {
		if (smblib_masked_write_mmi(chip, LEGACY_CABLE_CFG_REG,
					    0xFF, 0))
			mmi_err(chip, "Could not set Legacy Cable CFG\n");
	}

	if (chip->smb_version == PM8150B ||
	    chip->smb_version == PM7250B ||
	    chip->smb_version == PMI632) {
		mmi_err(chip, "DISABLE all QCOM JEITA\n");
		/* Ensure SW JEITA is DISABLED */
		pval.intval = 0;
		smb_mmi_write_iio_chan(chip,
					  SMB5_SW_JEITA_ENABLED, pval.intval);
		/* Ensure HW JEITA is DISABLED */
		if (smblib_masked_write_mmi(chip, PM8150B_JEITA_EN_CFG_REG,
					    0xFF, 0x00))
			mmi_err(chip, "Could not disable JEITA CFG\n");
	}

	if ((chip->smb_version == PM8150B) ||
	    (chip->smb_version == PM7250B) ||
	    (chip->smb_version == PMI632))
		if (smblib_masked_write_mmi(chip, USBIN_ADAPTER_ALLOW_CFG_REG,
					    USBIN_ADAPTER_ALLOW_MASK,
					    USBIN_ADAPTER_ALLOW_5V_TO_9V))
			mmi_err(chip, "Could not set USB Adapter CFG\n");

	/*
	 * Only 5V is allowed for QC2.0 and QC3.0, so set pulse count max
	 * accordingly to limit voltage increment.
	 */
	if (chip->hvdcp_power_max) {
		rc = smblib_masked_write_mmi(chip, HVDCP_PULSE_COUNT_MAX_REG,
					    HVDCP_PULSE_COUNT_MAX_QC2_MASK |
					    HVDCP_PULSE_COUNT_MAX_QC3_MASK,
					    chip->inc_hvdcp_cnt);
		if (rc < 0)
			mmi_err(chip, "Could not set HVDCP pulse count max\n");
	}

	chip->factory_mode = mmi_factory_check(MMI_FACTORY_MODE);
	chip->is_factory_image = mmi_factory_check(MMI_FACTORY_BUILD);
	chip->charging_limit_modes = CHARGING_LIMIT_UNKNOWN;

	if (chip->dc_cl_ma >= 0) {
		rc = smblib_set_charge_param(chip, &chip->param.dc_icl,
					chip->dc_cl_ma * 1000);
		if (rc)
			mmi_err(chip, "Failed to set DC ICL %d\n",
				chip->dc_cl_ma);
	}

	/* Workaround for some cables that collapse on boot */
	if (!chip->factory_mode) {
		dev_err(chip->dev, "SMBMMI: Suspending USB for 50 ms to clear\n");
		smblib_set_usb_suspend(chip, true);
		msleep(50);
		smblib_set_usb_suspend(chip, false);
	}

	if (chip->factory_mode &&
		(chip->smb_version == PMI632)) {
		rc = smblib_masked_write_mmi(chip, USBIN_INT_EN_CLR,
					    0xFF, USBIN_OV_EN_CLR);
		if (rc) {
			mmi_err(chip, "Could Not disable usbin ov irq\n");
		}
	}

	rc = device_create_file(chip->dev,
				&dev_attr_force_demo_mode);
	if (rc) {
		mmi_err(chip, "Couldn't create force_demo_mode\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_force_max_chrg_temp);
	if (rc) {
		mmi_err(chip, "Couldn't create force_max_chrg_temp\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_factory_image_mode);
	if (rc)
		mmi_err(chip, "Couldn't create factory_image_mode\n");

	rc = device_create_file(chip->dev,
				&dev_attr_factory_charge_upper);
	if (rc)
		mmi_err(chip, "Couldn't create factory_charge_upper\n");

	rc = device_create_file(chip->dev,
				&dev_attr_force_hvdcp_power_max);
	if (rc)
		mmi_err(chip, "Couldn't create force_hvdcp_power_max\n");

	rc = device_create_file(chip->dev,
				&dev_attr_force_pd_power_max);
	if (rc)
		mmi_err(chip, "Couldn't create force_pd_power_max\n");

	/* Register the notifier for the psy updates*/
	chip->mmi_psy_notifier.notifier_call = mmi_psy_notifier_call;
	rc = power_supply_reg_notifier(&chip->mmi_psy_notifier);
	if (rc)
		mmi_err(chip, "Failed to reg notifier: %d\n", rc);

	if (chip->factory_mode) {
		mmi_info(chip, "Entering Factory Mode SMB!\n");
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

		chip->smb_reboot.notifier_call = smbchg_reboot;
		chip->smb_reboot.next = NULL;
		chip->smb_reboot.priority = 1;
		rc = register_reboot_notifier(&chip->smb_reboot);
		if (rc)
			mmi_err(chip, "Register for reboot failed\n");
		rc = power_supply_get_property(chip->pc_port_psy,
					       POWER_SUPPLY_PROP_ONLINE,
					       &val);
		if (rc >= 0 && val.intval) {
			mmi_info(chip, "Factory Kill Armed\n");
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

		/* Some Cables need a more forced approach */
		rc = smblib_set_charge_param(chip, &chip->param.usb_icl,
					     3000000);
		if (rc < 0)
			mmi_err(chip,
				"Factory Couldn't set usb icl = 3000 rc=%d\n",
				(int)rc);

		rc = device_create_file(chip->dev,
					&dev_attr_smblib_mmi_address);
		if (rc) {
			mmi_err(chip,
				   "Couldn't create smblib_mmi_address\n");
		}

		rc = device_create_file(chip->dev,
					&dev_attr_smblib_mmi_data);
		if (rc) {
			mmi_err(chip,
				   "Couldn't create smblib_mmi_data\n");
		}

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
	cancel_delayed_work(&chip->heartbeat_work);
	if(chip->factory_mode)
	{
		//delay heartbeat in factory mode until after boot
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(10000));
	}
	else
	{
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	mmi_info(chip, "QPNP SMB MMI probed successfully!\n");

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
	if (chip->pc_port_psy)
		power_supply_put(chip->pc_port_psy);
	if (chip->dc_psy)
		power_supply_put(chip->dc_psy);
	if (chip->max_main_psy)
		power_supply_put(chip->max_main_psy);
	if (chip->max_flip_psy)
		power_supply_put(chip->max_flip_psy);

	return;
}

#ifdef CONFIG_PM_SLEEP
static int smb_mmi_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct smb_mmi_charger *chip = platform_get_drvdata(pdev);

	chip->suspended &= ~WAS_SUSPENDED;
	chip->suspended |= IS_SUSPENDED;

	return 0;
}

static int smb_mmi_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct smb_mmi_charger *chip = platform_get_drvdata(pdev);

	chip->suspended &= ~IS_SUSPENDED;
	chip->suspended |= WAS_SUSPENDED;

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
	{
		.compatible = "qcom,mmi-pm8150-smb5",
		.data = (void *)PM8150B,
	},
	{
		.compatible = "qcom,mmi-pm7250b-smb5",
		.data = (void *)PM7250B,
	},
	{
		.compatible = "qcom,mmi-pm6150-smb5",
		.data = (void *)PM6150,
	},
	{
		.compatible = "qcom,mmi-pmi632-smb5",
		.data = (void *)PMI632,
	},
	{ },
};

static struct platform_driver smb_mmi_driver = {
	.driver		= {
		.name		= "qcom,mmi-smbcharger-iio",
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
