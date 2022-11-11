/* SPDX-License-Identifier: GPL-2.0-only */
// sgm4154x Charger Driver
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.sg-micro.com

#ifndef _SGM4154x_CHARGER_H
#define _SGM4154x_CHARGER_H

#include <linux/i2c.h>

#include "mmi_charger.h"

#define SGM4154x_MANUFACTURER	"Texas Instruments"

//#define __SGM41541_CHIP_ID__
//#define __SGM41542_CHIP_ID__
//#define __SGM41516_CHIP_ID__
//#define __SGM41516D_CHIP_ID__

#ifdef __SGM41541_CHIP_ID__
#define SGM4154x_NAME		"sgm41541"
#endif

#ifdef __SGM41542_CHIP_ID__
#define SGM4154x_NAME		"sgm41542"
#endif

#ifdef __SGM41516_CHIP_ID__
#define SGM4154x_NAME		"sgm41516"
#endif

#ifdef __SGM41516D_CHIP_ID__
#define SGM4154x_NAME		"sgm41516D"
#endif

/*define register*/
#define SGM4154x_CHRG_CTRL_0	0x00
#define SGM4154x_CHRG_CTRL_1	0x01
#define SGM4154x_CHRG_CTRL_2	0x02
#define SGM4154x_CHRG_CTRL_3	0x03
#define SGM4154x_CHRG_CTRL_4	0x04
#define SGM4154x_CHRG_CTRL_5	0x05
#define SGM4154x_CHRG_CTRL_6	0x06
#define SGM4154x_CHRG_CTRL_7	0x07
#define SGM4154x_CHRG_STAT	    0x08
#define SGM4154x_CHRG_FAULT	    0x09
#define SGM4154x_CHRG_CTRL_a	0x0a
#define SGM4154x_CHRG_CTRL_b	0x0b
#define SGM4154x_CHRG_CTRL_c	0x0c
#define SGM4154x_CHRG_CTRL_d	0x0d
#define SGM4154x_INPUT_DET   	0x0e
#define SGM4154x_CHRG_CTRL_f	0x0f

/*rerun apsd*/
#define SGM4154x_IINDET_EN_MASK		GENMASK(7, 7)
#define SGM4154x_IINDET_EN				BIT(7)

/* charge status flags  */
#define SGM4154x_IINDPM_INT_MASK		GENMASK(0, 0)
#define SGM4154x_IINDPM_INT_DISABLE	BIT(0)
#define SGM4154x_VINDPM_INT_MASK		GENMASK(1, 1)
#define SGM4154x_VINDPM_INT_DISABLE	BIT(1)
#define SGM4154x_CHRG_EN		BIT(4)
#define SGM4154x_HIZ_EN		    BIT(7)
#define SGM4154x_TERM_EN		BIT(7)
#define SGM4154x_VAC_OVP_MASK	GENMASK(7, 6)
#define SGM4154x_DPDM_ONGOING   BIT(7)
#define SGM4154x_VBUS_GOOD      BIT(7)

#define SGM4154X_BOOST_LIM_MASK	GENMASK(7, 7)
#define SGM4154x_OTG_MASK		GENMASK(5, 4)
#define SGM4154x_OTG_EN		    BIT(5)

/* Part ID  */
#define SGM4154x_PN_MASK	    GENMASK(6, 3)
#define SGM4154x_PN_41541_ID    (BIT(6)| BIT(5))
#define SGM4154x_PN_41516_ID    (BIT(6)| BIT(5))
#define SGM4154x_PN_41542_ID    (BIT(6)| BIT(5)| BIT(3))
#define SGM4154x_PN_41516D_ID   (BIT(6)| BIT(5)| BIT(3))

/* WDT TIMER SET  */
#define SGM4154x_WDT_TIMER_MASK        GENMASK(5, 4)
#define SGM4154x_WDT_TIMER_DISABLE     0
#define SGM4154x_WDT_TIMER_40S         BIT(4)
#define SGM4154x_WDT_TIMER_80S         BIT(5)
#define SGM4154x_WDT_TIMER_160S        (BIT(4)| BIT(5))

#define SGM4154x_WDT_RST_MASK          BIT(6)

/*En termniataion*/
#define SGM4154x_EN_TERM_MASK          GENMASK(7,7)
#define SGM4154x_EN_TERM_DISABLE          0
#define SGM4154x_EN_TERM_ENABLE          BIT(7)

/* SAFETY TIMER SET  */
#define SGM4154x_SAFETY_TIMER_MASK     GENMASK(3, 3)
#define SGM4154x_SAFETY_TIMER_DISABLE     0
#define SGM4154x_SAFETY_TIMER_EN       BIT(3)
#define SGM4154x_SAFETY_TIMER_5H         0
#define SGM4154x_SAFETY_TIMER_10H      BIT(2)

/* recharge voltage  */
#define SGM4154x_VRECHARGE              BIT(0)
#define SGM4154x_VRECHRG_STEP_mV		100
#define SGM4154x_VRECHRG_OFFSET_mV		100

/* charge status  */
#define SGM4154x_VSYS_STAT		BIT(0)
#define SGM4154x_THERM_STAT		BIT(1)
#define SGM4154x_PG_STAT		BIT(2)
#define SGM4154x_CHG_STAT_MASK	GENMASK(4, 3)
#define SGM4154x_PRECHRG		BIT(3)
#define SGM4154x_FAST_CHRG	    BIT(4)
#define SGM4154x_TERM_CHRG	    (BIT(3)| BIT(4))

/* charge type  */
#define SGM4154x_VBUS_STAT_MASK	GENMASK(7, 5)
#define SGM4154x_NOT_CHRGING	0
#define SGM4154x_USB_SDP		BIT(5)
#define SGM4154x_USB_CDP		BIT(6)
#define SGM4154x_USB_DCP		(BIT(5) | BIT(6))
#define SGM4154x_UNKNOWN	    (BIT(7) | BIT(5))
#define SGM4154x_NON_STANDARD	(BIT(7) | BIT(6))
#define SGM4154x_OTG_MODE	    (BIT(7) | BIT(6) | BIT(5))

#define SGM4154x_WLS_TYPE	    0xFF

/* TEMP Status  */
#define SGM4154x_TEMP_MASK	    GENMASK(2, 0)
#define SGM4154x_TEMP_NORMAL	BIT(0)
#define SGM4154x_TEMP_WARM	    BIT(1)
#define SGM4154x_TEMP_COOL	    (BIT(0) | BIT(1))
#define SGM4154x_TEMP_COLD	    (BIT(0) | BIT(3))
#define SGM4154x_TEMP_HOT	    (BIT(2) | BIT(3))

/* precharge current  */
#define SGM4154x_PRECHRG_CUR_MASK		GENMASK(7, 4)
#define SGM4154x_PRECHRG_CURRENT_STEP_uA		60000
#define SGM4154x_PRECHRG_I_MIN_uA		60000
#define SGM4154x_PRECHRG_I_MAX_uA		780000
#define SGM4154x_PRECHRG_I_DEF_uA		180000

/* termination current  */
#define SGM4154x_TERMCHRG_CUR_MASK		GENMASK(3, 0)
#define SGM4154x_TERMCHRG_CURRENT_STEP_uA	60000
#define SGM4154x_TERMCHRG_I_MIN_uA		60000
#define SGM4154x_TERMCHRG_I_MAX_uA		960000
#define SGM4154x_TERMCHRG_I_DEF_uA		180000

/* charge current  */
#define SGM4154x_ICHRG_CUR_MASK		GENMASK(5, 0)
#define SGM4154x_ICHRG_CURRENT_STEP_uA		60000
#define SGM4154x_ICHRG_I_MIN_uA			0
#define SGM4154x_ICHRG_I_MAX_uA			3780000
#define SGM4154x_ICHRG_I_DEF_uA			2040000

/* charge voltage  */
#define SGM4154x_VREG_V_MASK		GENMASK(7, 3)
#define SGM4154x_VREG_V_MAX_uV	    4624000
#define SGM4154x_VREG_V_MIN_uV	    3856000
#define SGM4154x_VREG_V_DEF_uV	    4208000
#define SGM4154x_VREG_V_STEP_uV	    32000

/* VREG Fine Tuning  */
#define SGM4154x_VREG_FT_MASK	     GENMASK(7, 6)
#define SGM4154x_VREG_FT_UP_8mV	     BIT(6)
#define SGM4154x_VREG_FT_DN_8mV	     BIT(7)
#define SGM4154x_VREG_FT_DN_16mV	 (BIT(7) | BIT(6))

/* iindpm current  */
#define SGM4154x_IINDPM_I_MASK		GENMASK(4, 0)
#define SGM4154x_IINDPM_I_MIN_uA	100000
#define SGM4154x_IINDPM_I_MAX_uA	3800000
#define SGM4154x_IINDPM_STEP_uA	    100000
#define SGM4154x_IINDPM_DEF_uA	    2400000

/* vindpm voltage  */
#define SGM4154x_VINDPM_V_MASK      GENMASK(3, 0)
#define SGM4154x_VINDPM_V_MIN_uV    3900000
#define SGM4154x_VINDPM_V_MAX_uV    12000000
#define SGM4154x_VINDPM_STEP_uV     100000
#define SGM4154x_VINDPM_DEF_uV	    4500000
#define SGM4154x_VINDPM_OS_MASK     GENMASK(1, 0)

/* DP DM SEL  */
#define SGM4154x_DP_VSEL_MASK       GENMASK(4, 3)
#define SGM4154x_DM_VSEL_MASK       GENMASK(2, 1)

/* PUMPX SET  */
#define SGM4154x_EN_PUMPX           BIT(7)
#define SGM4154x_PUMPX_UP           BIT(6)
#define SGM4154x_PUMPX_DN           BIT(5)

/* customer define jeita paramter */
#define SGM4154x_JEITA_ENABLE_MASK GENMASK(0, 0)
#define SGM4154x_JEITA_ENABLE		BIT(0)
#define SGM4154x_JEITA_DISABLE		0
#define JEITA_TEMP_ABOVE_T4_CV	0
#define JEITA_TEMP_T3_TO_T4_CV	4100000
#define JEITA_TEMP_T2_TO_T3_CV	4350000
#define JEITA_TEMP_T1_TO_T2_CV	4350000
#define JEITA_TEMP_T0_TO_T1_CV	0
#define JEITA_TEMP_BELOW_T0_CV	0

#define JEITA_TEMP_ABOVE_T4_CC_CURRENT	0
#define JEITA_TEMP_T3_TO_T4_CC_CURRENT	1000000
#define JEITA_TEMP_T2_TO_T3_CC_CURRENT	2400000
#define JEITA_TEMP_T1_TO_T2_CC_CURRENT	2000000
#define JEITA_TEMP_T0_TO_T1_CC_CURRENT	0
#define JEITA_TEMP_BELOW_T0_CC_CURRENT	0

#define TEMP_T4_THRES  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRES  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRES  20
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRES  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRES  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRES 0

struct sgm4154x_init_data {
	u32 ichg;	/* charge current		*/
	u32 ilim;	/* input current		*/
	u32 wlim;	/* wireless input current	*/
	u32 vreg;	/* regulation voltage		*/
	u32 iterm;	/* termination current		*/
	u32 iprechg;	/* precharge current		*/
	u32 vlim;	/* minimum system voltage limit */
	u32 max_ichg;
	u32 max_vreg;
	u32 vrechg;
	bool charger_disabled;
};

struct sgm4154x_state {
	bool vsys_stat;
	bool therm_stat;
	bool online;
	u8 chrg_stat;
	u8 vbus_status;

	bool chrg_en;
	bool hiz_en;
	bool term_en;
	bool vbus_gd;
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 ntc_fault;

	u32 ibus_limit;
	u32 vbus_adc;
	u32 ibus_adc;
};

struct sgm4154x_jeita {
	int jeita_temp_above_t4_cv;
	int jeita_temp_t3_to_t4_cv;
	int jeita_temp_t2_to_t3_cv;
	int jeita_temp_t1_to_t2_cv;
	int jeita_temp_t0_to_t1_cv;
	int jeita_temp_below_t0_cv;
	int jeita_temp_above_t4_cc_current;
	int jeita_temp_t3_to_t4_cc_current;
	int jeita_temp_t2_to_t3_cc_current;
	int jeita_temp_t1_to_t2_cc_current;
	int jeita_temp_below_t0_cc_current;
	int temp_t4_thres;
	int temp_t4_thres_minus_x_degree;
	int temp_t3_thres;
	int temp_t3_thres_minus_x_degree;
	int temp_t2_thres;
	int temp_t2_thres_plus_x_degree;
	int temp_t1_thres;
	int temp_t1_thres_plus_x_degree;
	int temp_t0_thres;
	int temp_t0_thres_plus_x_degree;
	int temp_neg_10_thres;
};

struct sgm_mmi_charger {
	struct sgm4154x_device		*sgm;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_info		chg_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_constraint	constraint;
	struct mmi_charger_driver	*driver;
	u32				chrg_taper_cnt;
	const char			*fg_psy_name;
	struct power_supply		*fg_psy;
	int ichg_polority;

	int paired_ichg;
	int paired_ichg_table_len;
	int *paired_ichg_table;
	int paired_load;
	int paired_load_thres_len;
	int *paired_load_thres;
	int paired_esr;
	int batt_esr;
	int batt_protect_type;
	struct mmi_battery_info		paired_batt_info;
};

struct sgm4154x_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *wls_psy;
	struct power_supply *usb;
	struct power_supply *ac;
	struct mutex lock;
	struct mutex i2c_rw_lock;

	struct usb_phy *usb2_phy;
	struct usb_phy *usb3_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];
	int device_id;

	struct sgm4154x_init_data init_data;
	struct sgm4154x_state state;
	u32 watchdog_timer;
	#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
	struct charger_device *chg_dev;
	#endif
	struct regulator_dev *otg_rdev;

	struct delayed_work charge_detect_delayed_work;
	struct delayed_work charge_monitor_work;
	struct notifier_block pm_nb;
	bool sgm4154x_suspend_flag;

	struct wakeup_source *charger_wakelock;
	bool enable_sw_jeita;
	struct sgm4154x_jeita data;
	bool enable_hw_jeita;

	struct dentry *debug_root;

	u32 *thermal_levels;
	u32 thermal_fcc_ua;
	int curr_thermal_level;
	int num_thermal_levels;

	u32 ichg;	/* charge current		*/
	u32 ilim;	/* input current		*/
	u32 vreg;	/* regulation voltage		*/
	int user_ichg; /* user charge current		*/
	int user_ilim; /* user input current		*/
	int user_chg_en; /* user charging enable	*/
	int user_chg_susp; /* user charger suspend	*/

	bool batt_present;
	bool otg_enabled;
	bool irq_enabled;
	u32 batt_protect_duration;
	int vbat_ovp_threshold;
	int ibat_ocp_threshold;
	int ibat_occp_threshold;
	int high_load_en_gpio;
	int low_load_en_gpio;
	bool high_load_active_low;
	bool low_load_active_low;
	bool update_pending;

	int chg_en_gpio;
	bool use_ext_usb_psy;
	bool use_ext_wls_psy;
	struct regulator *vdd_i2c_vreg;
	struct notifier_block psy_nb;
	struct sgm_mmi_charger *mmi_charger;
};

#endif /* _SGM4154x_CHARGER_H */
