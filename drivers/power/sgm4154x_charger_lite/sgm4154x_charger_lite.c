// SPDX-License-Identifier: GPL-2.0
// SGM4154x driver version 2021-08-05-002
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.sg-micro.com

#define pr_fmt(fmt)     "sgm4154x-charger: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/time64.h>

#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "sgm4154x_charger_lite.h"

#define DEF_VBAT_OVP_MV (4600)
#define DEF_IBAT_OCP_MA (1500)
#define DEF_IBAT_OCCP_MA (1500)
#define DEF_PROTECT_DURATION_MS (60000)
#define DEF_USB_CURRENT_MA (500000)

static bool paired_vbat_panic_enabled = false;
module_param(paired_vbat_panic_enabled, bool, 0644);
MODULE_PARM_DESC(paired_vbat_panic_enabled,
	"Enable panic on abnormal paired vbat delta");

static struct power_supply_desc sgm4154x_power_supply_desc;
#ifdef SGM_BASE
static struct reg_default sgm4154x_reg_defs[] = {
	{SGM4154x_CHRG_CTRL_0, 0x0a},
	{SGM4154x_CHRG_CTRL_1, 0x1a},
	{SGM4154x_CHRG_CTRL_2, 0x88},
	{SGM4154x_CHRG_CTRL_3, 0x22},
	{SGM4154x_CHRG_CTRL_4, 0x58},
	{SGM4154x_CHRG_CTRL_5, 0x9f},
	{SGM4154x_CHRG_CTRL_6, 0xe6},
	{SGM4154x_CHRG_CTRL_7, 0x4c},
	{SGM4154x_CHRG_STAT,   0x00},
	{SGM4154x_CHRG_FAULT,  0x00},
	{SGM4154x_CHRG_CTRL_a, 0x00},//
	{SGM4154x_CHRG_CTRL_b, 0x64},
	{SGM4154x_CHRG_CTRL_c, 0x75},
	{SGM4154x_CHRG_CTRL_d, 0x00},
	{SGM4154x_INPUT_DET,   0x00},
	{SGM4154x_CHRG_CTRL_f, 0x00},
};
#endif

/* SGM4154x REG06 BOOST_LIM[5:4], uV */
static const unsigned int BOOST_VOLT_LIMIT[] = {
	4850000, 5000000, 5150000, 5300000
};
 /* SGM4154x REG02 BOOST_LIM[7:7], uA */
#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif

enum SGM4154x_VREG_FT {
	VREG_FT_DISABLE,
	VREG_FT_UP_8mV,
	VREG_FT_DN_8mV,
	VREG_FT_DN_16mV,
};

enum SGM4154x_VINDPM_OS {
	VINDPM_OS_3900mV,
	VINDPM_OS_5900mV,
	VINDPM_OS_7500mV,
	VINDPM_OS_10500mV,
};

enum {
	PAIRED_LOAD_OFF,
	PAIRED_LOAD_LOW,
	PAIRED_LOAD_HIGH,
};

enum {
	BATT_PROTECT_TYPE_NONE,
	BATT_PROTECT_TYPE_OVP,
	BATT_PROTECT_TYPE_OCP,
	BATT_PROTECT_TYPE_OCCP,
};

static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,		/* Standard Downstream Port */
	POWER_SUPPLY_USB_TYPE_DCP,		/* Dedicated Charging Port */
	POWER_SUPPLY_USB_TYPE_CDP,		/* Charging Downstream Port */
	POWER_SUPPLY_USB_TYPE_ACA,		/* Accessory Charger Adapters */
	POWER_SUPPLY_USB_TYPE_C,		/* Type C Port */
	POWER_SUPPLY_USB_TYPE_PD,		/* Power Delivery Port */
	POWER_SUPPLY_USB_TYPE_PD_DRP,		/* PD Dual Role Port */
	POWER_SUPPLY_USB_TYPE_PD_PPS,		/* PD Programmable Power Supply */
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,	/* Apple Charging Method */
};

static int sgm4154x_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct sgm4154x_device *sgm =
			container_of(nb, struct sgm4154x_device, usb_nb);

	sgm->usb_event = val;

	queue_work(system_power_efficient_wq, &sgm->usb_work);

	return NOTIFY_OK;
}

static void sgm4154x_usb_work(struct work_struct *data)
{
	struct sgm4154x_device *sgm =
			container_of(data, struct sgm4154x_device, usb_work);

	switch (sgm->usb_event) {
	case USB_EVENT_ID:
		break;

	case USB_EVENT_NONE:
		power_supply_changed(sgm->charger);
		break;
	}

	return;

	dev_err(sgm->dev, "Error switching to charger mode.\n");
}

#ifdef SGM_BASE
static int sgm4154x_get_term_curr(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val;
	int offset = SGM4154x_TERMCHRG_I_MIN_uA;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val &= SGM4154x_TERMCHRG_CUR_MASK;
	reg_val = reg_val * SGM4154x_TERMCHRG_CURRENT_STEP_uA + offset;
	return reg_val;
}

static int sgm4154x_get_prechrg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val;
	int offset = SGM4154x_PRECHRG_I_MIN_uA;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val&SGM4154x_PRECHRG_CUR_MASK)>>4;
	reg_val = reg_val * SGM4154x_PRECHRG_CURRENT_STEP_uA + offset;
	return reg_val;
}
#endif

static int sgm4154x_get_ichg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	int ichg;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_2, &ichg);
	if (ret)
		return ret;

	ichg &= SGM4154x_ICHRG_CUR_MASK;

	return ichg * SGM4154x_ICHRG_CURRENT_STEP_uA;
}

static int sgm4154x_set_term_curr(struct sgm4154x_device *sgm, int term_current)
{
	int reg_val;
	int offset = SGM4154x_TERMCHRG_I_MIN_uA;

	if (term_current < SGM4154x_TERMCHRG_I_MIN_uA)
		term_current = SGM4154x_TERMCHRG_I_MIN_uA;
	else if (term_current > SGM4154x_TERMCHRG_I_MAX_uA)
		term_current = SGM4154x_TERMCHRG_I_MAX_uA;

	reg_val = (term_current - offset) / SGM4154x_TERMCHRG_CURRENT_STEP_uA;

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_3,
				  SGM4154x_TERMCHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_prechrg_curr(struct sgm4154x_device *sgm, int pre_current)
{
	int reg_val;
	int offset = SGM4154x_PRECHRG_I_MIN_uA;

	if (pre_current < SGM4154x_PRECHRG_I_MIN_uA)
		pre_current = SGM4154x_PRECHRG_I_MIN_uA;
	else if (pre_current > SGM4154x_PRECHRG_I_MAX_uA)
		pre_current = SGM4154x_PRECHRG_I_MAX_uA;

	reg_val = (pre_current - offset) / SGM4154x_PRECHRG_CURRENT_STEP_uA;
	reg_val = reg_val << 4;
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_3,
				  SGM4154x_PRECHRG_CUR_MASK, reg_val);
}



static int sgm4154x_set_ichrg_curr(struct sgm4154x_device *sgm, int chrg_curr)
{
	int ret;
	int reg_val;

	if (chrg_curr < SGM4154x_ICHRG_I_MIN_uA)
		chrg_curr = SGM4154x_ICHRG_I_MIN_uA;
	else if ( chrg_curr > sgm->init_data.max_ichg)
		chrg_curr = sgm->init_data.max_ichg;

	reg_val = sgm->thermal_fcc_ua;
	chrg_curr = min(chrg_curr, reg_val);
	if (sgm->mmi_charger) {
		reg_val = sgm->mmi_charger->paired_ichg;
		chrg_curr = min(chrg_curr, reg_val);
	}

	if (sgm->user_ichg >= 0 && sgm->user_ichg != chrg_curr) {
		chrg_curr = sgm->user_ichg;
		pr_info("User request override ichg = %duA\n", chrg_curr);
	}

	sgm->ichg = chrg_curr;
	pr_info("set ichg = %duA\n", chrg_curr);
	reg_val = chrg_curr / SGM4154x_ICHRG_CURRENT_STEP_uA;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2,
				  SGM4154x_ICHRG_CUR_MASK, reg_val);

	return ret;
}


static int sgm4154x_set_thermal_mitigation(struct sgm4154x_device *sgm, int val)
{
	int ret;
	u32 prev_fcc_ua;

	if (!sgm->num_thermal_levels)
		return 0;

	if (sgm->num_thermal_levels < 0) {
		pr_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

	if (val < 0 || val > sgm->num_thermal_levels) {
		pr_err("Invalid thermal level: %d\n", val);
		return -EINVAL;
	}

	prev_fcc_ua = sgm->thermal_fcc_ua;
	sgm->thermal_fcc_ua = sgm->thermal_levels[val];
	ret = sgm4154x_set_ichrg_curr(sgm, sgm->thermal_levels[val]);
	if (ret) {
		pr_err("Failed to set thermal mitigation val=%d, ret=%d\n",
				sgm->thermal_levels[val], ret);
		sgm->thermal_fcc_ua = prev_fcc_ua;
	} else {
		sgm->curr_thermal_level = val;
	}
	pr_info("thermal level: %d, thermal fcc: %d\n", sgm->curr_thermal_level, sgm->thermal_fcc_ua);

	return ret;
}

static int sgm4154x_vreg_fine_tuning(struct sgm4154x_device *sgm, enum SGM4154x_VREG_FT ft);
static int sgm4154x_set_chrg_volt(struct sgm4154x_device *sgm, int chrg_volt)
{
	int ret;
	int reg_val;
	enum SGM4154x_VREG_FT ft = VREG_FT_DISABLE;

	if (chrg_volt < SGM4154x_VREG_V_MIN_uV)
		chrg_volt = SGM4154x_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;

	sgm->vreg = chrg_volt;

	reg_val = (chrg_volt-SGM4154x_VREG_V_MIN_uV) / SGM4154x_VREG_V_STEP_uV;

	switch(chrg_volt) {
	case 4480000:
	case 4450000:
		reg_val++;
		ft = VREG_FT_DN_16mV;
		break;
	case 4200000:
		reg_val++;
		ft = VREG_FT_DN_8mV;
		break;
	default:
		break;
	}

	ret = sgm4154x_vreg_fine_tuning(sgm, ft);
	if (ret) {
		pr_err("can't set vreg fine tunning ret=%d\n", ret);
		return ret;
	}

	reg_val = reg_val<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VREG_V_MASK, reg_val);

	return ret;
}

#ifdef SGM_BASE
static int sgm4154x_get_chrg_volt(struct sgm4154x_device *sgm)
{
	int ret;
	int vreg_val, chrg_volt;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;

	vreg_val = (vreg_val & SGM4154x_VREG_V_MASK)>>3;

	if (15 == vreg_val)
		chrg_volt = 4352000; //default
	else if (vreg_val < 25)
		chrg_volt = vreg_val*SGM4154x_VREG_V_STEP_uV + SGM4154x_VREG_V_MIN_uV;

	return chrg_volt;
}

#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
static int sgm4154x_enable_qc20_hvdcp_9v(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;

	/*dp and dm connected,dp 0.6V dm 0.6V*/
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6V
	if (ret)
		return ret;

	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6V
	if (ret)
		return ret;
	mdelay(1000);

	dm_val = 0x2;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	mdelay(1);
	/* dp 3.3v and dm 0.6v out 9V */
	dp_val = SGM4154x_DP_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 3.3v
	if (ret)
		return ret;
	//mdelay(1250);

	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6v

	return ret;
}

static int sgm4154x_enable_qc20_hvdcp_12v(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;

	/*dp and dm connected,dp 0.6V dm 0.6V*/
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6V
	if (ret)
		return ret;

	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6V
	if (ret)
		return ret;
	mdelay(1000);

	dm_val = 0x2;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	mdelay(1);
	/* dp 0.6v and dm 0.6v out 12V */
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;
	//mdelay(1250);

	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6v

	return ret;
}

/* step 1. entry QC3.0 mode
   step 2. up or down 200mv
   step 3. retry step 2 */
static int sgm4154x_enable_qc30_hvdcp(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;

	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;

	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6V
	if (ret)
		return ret;
	mdelay(1000);

	dm_val = 0x2;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	mdelay(1);

	dm_val = SGM4154x_DM_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 3.3v

	return ret;
}

// Must enter 3.0 mode to call ,otherwise cannot step correctly.
static int sgm4154x_qc30_step_up_vbus(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val;

	/*  dm 3.3v to dm 0.6v  step up 200mV when IC is QC3.0 mode*/
	dp_val = SGM4154x_DP_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 3.3v
	if (ret)
		return ret;

	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;

	udelay(100);
	return ret;
}
// Must enter 3.0 mode to call ,otherwise cannot step correctly.
static int sgm4154x_qc30_step_down_vbus(struct sgm4154x_device *sgm)
{
	int ret;
	int dm_val;

	/* dp 0.6v and dm 0.6v step down 200mV when IC is QC3.0 mode*/
	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6V
	if (ret)
		return ret;

	dm_val = SGM4154x_DM_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 3.3v
	udelay(100);

	return ret;
}
#endif
#endif

// fine tuning termination voltage,to Improve accuracy
static int sgm4154x_vreg_fine_tuning(struct sgm4154x_device *sgm,enum SGM4154x_VREG_FT ft)
{
	int ret;
	int reg_val;

	switch(ft) {
		case VREG_FT_DISABLE:
			reg_val = 0;
			break;

		case VREG_FT_UP_8mV:
			reg_val = SGM4154x_VREG_FT_UP_8mV;
			break;

		case VREG_FT_DN_8mV:
			reg_val = SGM4154x_VREG_FT_DN_8mV;
			break;

		case VREG_FT_DN_16mV:
			reg_val = SGM4154x_VREG_FT_DN_16mV;
			break;

		default:
			reg_val = 0;
			break;
	}
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_f,
				  SGM4154x_VREG_FT_MASK, reg_val);
	pr_info("reg_val:%d\n", reg_val);

	return ret;
}

static int sgm4154x_get_vindpm_offset_os(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_f, &reg_val);
	if (ret)
		return ret;

	reg_val = reg_val & SGM4154x_VINDPM_OS_MASK;

	return reg_val;
}

#ifdef SGM_BASE
static int sgm4154x_set_vindpm_offset_os(struct sgm4154x_device *sgm,enum SGM4154x_VINDPM_OS offset_os)
{
	int ret;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_f,
				  SGM4154x_VINDPM_OS_MASK, offset_os);

	if (ret){
		pr_err("fail\n");
		return ret;
	}

	return ret;
}
#endif

static int sgm4154x_set_input_volt_lim(struct sgm4154x_device *sgm, int vindpm)
{
	int ret;
	int offset = 0;
	int vlim;
	int temp;

	if (vindpm < SGM4154x_VINDPM_V_MIN_uV ||
	    vindpm > SGM4154x_VINDPM_V_MAX_uV)
		return -EINVAL;

	temp = sgm4154x_get_vindpm_offset_os(sgm);
	if (0 == temp)
		offset = 3900000; //uv
	else if (1 == temp)
		offset = 5900000; //uv
	else if (2 == temp)
		offset = 7500000; //uv
	else if (3 == temp)
		offset = 10500000; //uv

	vlim = (vindpm - offset) / SGM4154x_VINDPM_STEP_uV;

	vlim &= 0x0F;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VINDPM_V_MASK, vlim);

	return ret;
}

static int sgm4154x_get_input_volt_lim(struct sgm4154x_device *sgm)
{
	int ret;
	int offset = 0;
	int vlim;
	int temp;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_6, &vlim);
	if (ret)
		return ret;

	temp = sgm4154x_get_vindpm_offset_os(sgm);
	if (0 == temp)
		offset = 3900000; //uv
	else if (1 == temp)
		offset = 5900000;
	else if (2 == temp)
		offset = 7500000;
	else if (3 == temp)
		offset = 10500000;

	vlim = offset + (vlim & 0x0F) * SGM4154x_VINDPM_STEP_uV;
	return vlim;
}

static int sgm4154x_set_input_curr_lim(struct sgm4154x_device *sgm, int iindpm)
{
	int ret;
	int reg_val = 0;

	if (iindpm > sgm->init_data.ilim)
		iindpm = sgm->init_data.ilim;

	if (sgm->user_ilim >= 0 && sgm->user_ilim != iindpm) {
		iindpm = sgm->user_ilim;
		pr_info("User request override ilim = %duA\n", iindpm);
	}

	sgm->ilim = iindpm;

	if (iindpm < SGM4154x_IINDPM_I_MIN_uA)
		reg_val = 0;
	else if (iindpm >= SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1F;
	else if (iindpm >= SGM4154x_IINDPM_I_MIN_uA && iindpm <= 3100000)//default
		reg_val = (iindpm-SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
	else if (iindpm > 3100000 && iindpm < SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1E;

	return ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, reg_val);
}

static int sgm4154x_get_input_curr_lim(struct sgm4154x_device *sgm)
{
	int ret;
	int ilim;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &ilim);
	if (ret)
		return ret;
	if (SGM4154x_IINDPM_I_MASK == (ilim & SGM4154x_IINDPM_I_MASK))
		return SGM4154x_IINDPM_I_MAX_uA;

	ilim = (ilim & SGM4154x_IINDPM_I_MASK)*SGM4154x_IINDPM_STEP_uA + SGM4154x_IINDPM_I_MIN_uA;

	return ilim;
}

#ifdef SGM_BASE
static int sgm4154x_set_watchdog_timer(struct sgm4154x_device *sgm, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = SGM4154x_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = SGM4154x_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = SGM4154x_WDT_TIMER_80S;
	else
		reg_val = SGM4154x_WDT_TIMER_160S;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_5,
				SGM4154x_WDT_TIMER_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_wdt_rst(struct sgm4154x_device *sgm, bool is_rst)
{
	int val = 0;

	if (is_ rst)
		val = SGM4154x_WDT_RST_MASK;
	else
		val = 0;
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1,
				  SGM4154x_WDT_RST_MASK, val);
}
#endif

static int is_wls_online(struct sgm4154x_device *sgm)
{
	int rc;
	union power_supply_propval val;

	if (!sgm->use_ext_wls_psy || !sgm->wls_psy)
		return 0;

	rc = power_supply_get_property(sgm->wls_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		pr_err("Error wls online rc = %d\n", rc);
		return 0;
	}

	pr_debug("wireless online is %d", val.intval);
	return val.intval;
}

static int sgm4154x_get_state(struct sgm4154x_device *sgm,
			     struct sgm4154x_state *state)
{
	int chrg_stat;
	int fault;
	int chrg_param_0,chrg_param_1,chrg_param_2,chrg_param_3;
	int ret;
	union power_supply_propval val = {0};

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
		if (ret){
			pr_err("read SGM4154x_CHRG_STAT fail\n");
			return ret;
		}
	}

	if (sgm->use_ext_usb_psy && sgm->usb) {
		ret = power_supply_get_property(sgm->usb,
				POWER_SUPPLY_PROP_USB_TYPE, &val);
		switch (val.intval) {
		case POWER_SUPPLY_USB_TYPE_UNKNOWN:
			state->chrg_type = 0;
			break;
		case POWER_SUPPLY_USB_TYPE_SDP:
			state->chrg_type = SGM4154x_USB_SDP;
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			state->chrg_type = SGM4154x_USB_CDP;
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
			state->chrg_type = SGM4154x_USB_DCP;
			break;
		default:
			state->chrg_type = SGM4154x_UNKNOWN;
			break;
		}
		ret = power_supply_get_property(sgm->usb,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		if (!ret)
			state->vbus_adc = val.intval;
		ret = power_supply_get_property(sgm->usb,
				POWER_SUPPLY_PROP_CURRENT_NOW, &val);
		if (!ret)
			state->ibus_adc = val.intval;
		ret = power_supply_get_property(sgm->usb,
				POWER_SUPPLY_PROP_ONLINE, &val);
		if (!ret)
			state->online = val.intval;
		ret = power_supply_get_property(sgm->usb,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
		if (!ret)
			state->ibus_limit = val.intval;
		state->chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	} else if (sgm->use_ext_usb_psy) {
		state->chrg_type = 0;
		state->chrg_stat = 0;
		state->online = 0;
		state->ibus_limit = 0;
	} else {
		state->chrg_type = chrg_stat & SGM4154x_VBUS_STAT_MASK;
		state->chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
		state->online = !!(chrg_stat & SGM4154x_PG_STAT);
	};

	if (!state->online && is_wls_online(sgm)) {
		state->online = 1;
		state->chrg_type = SGM4154x_WLS_TYPE;
	}

	state->therm_stat = !!(chrg_stat & SGM4154x_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM4154x_VSYS_STAT);

	pr_debug("chrg_stat =%d chrg_type =%d online = %d\n", state->chrg_stat, state->chrg_type, state->online);

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_FAULT, &fault);
	if (ret){
		pr_err("read SGM4154x_CHRG_FAULT fail\n");
		return ret;
	}
	state->chrg_fault = fault;
	state->ntc_fault = fault & SGM4154x_TEMP_MASK;
	state->health = state->ntc_fault;
	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &chrg_param_0);
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_0 fail\n");
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM4154x_HIZ_EN);

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_5, &chrg_param_1);
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_5 fail\n");
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM4154x_TERM_EN);

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &chrg_param_2);
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_a fail\n");
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM4154x_VBUS_GOOD);

	ret = regmap_read(sgm->regmap, SGM4154x_INPUT_DET, &chrg_param_3);
	if (ret){
		pr_err("read SGM4154x_INPT_DET fail\n");
		return ret;
	}
	state->detect_done = !!(chrg_param_3 & SGM4154x_DPDM_ONGOING);

	state->input_ready = state->online && state->vbus_gd && state->detect_done;
	return 0;
}

static int sgm4154x_set_hiz_en(struct sgm4154x_device *sgm, bool hiz_en)
{
	int reg_val;

	if (sgm->user_chg_susp >= 0 && sgm->user_chg_susp != hiz_en) {
		hiz_en = !!sgm->user_chg_susp;
		pr_info("User request override hiz_en = %d\n", hiz_en);
	}

	dev_notice(sgm->dev, "%s:%d", __func__, hiz_en);
	reg_val = hiz_en ? SGM4154x_HIZ_EN : 0;

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_HIZ_EN, reg_val);
}

static bool sgm4154x_is_enabled_charging(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int status = 0;
	bool enabled = false;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_1, &status);
	if(ret)
		return false;

	enabled = (status & SGM4154x_CHRG_EN) ? true:false;

	if (gpio_is_valid(sgm->chg_en_gpio) &&
	    gpio_get_value(sgm->chg_en_gpio))
		enabled = false;

	return enabled;
}

int sgm4154x_enable_charger(struct sgm4154x_device *sgm)
{
	int ret;

	if (sgm->user_chg_en == 0) {
		pr_info("Skip enable charging for user request override\n");
		return 0;
	}

	if (sgm->state.input_ready == 0) {
		pr_info("Skip enable charging before input ready\n");
		return 0;
	}

	pr_info("sgm4154x_enable_charger\n");
	if (gpio_is_valid(sgm->chg_en_gpio)) {
		gpio_set_value(sgm->chg_en_gpio, 0);
	}
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1,
				SGM4154x_CHRG_EN,
				SGM4154x_CHRG_EN);

	return ret;
}

int sgm4154x_disable_charger(struct sgm4154x_device *sgm)
{
	int ret;

	if (sgm->user_chg_en > 0) {
		pr_info("Skip disable charging for user request override\n");
		return 0;
	}

	pr_info("sgm4154x_disable_charger\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1,
				SGM4154x_CHRG_EN,
				0);
	return ret;
}

int sgm4154x_disable_watchdog(struct sgm4154x_device *sgm)
{
	int ret;
	pr_info("sgm4154x_disable_watchdog\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_5, SGM4154x_WDT_TIMER_MASK,
                     SGM4154x_WDT_TIMER_DISABLE);
	return ret;
}

int sgm4154x_enable_hw_jeita(struct sgm4154x_device *sgm, bool en)
{
	int rc = 0;

	rc = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d, SGM4154x_JEITA_ENABLE_MASK,
                     en ? SGM4154x_JEITA_ENABLE : SGM4154x_JEITA_DISABLE);

	pr_info("%s hw jeita %s\n",
		en ? "enable" : "disable",
		rc ? "failed" : "success");

	return rc;
}

int sgm4154x_disable_vindpm_int_pulse(struct sgm4154x_device *sgm)
{
	int ret;
	pr_info("sgm4154x_disable_vindpm_pulse\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_a, SGM4154x_VINDPM_INT_MASK,
                     SGM4154x_VINDPM_INT_DISABLE);
	return ret;
}

int sgm4154x_disable_iindpm_int_pulse(struct sgm4154x_device *sgm)
{
	int ret;
	pr_info("sgm4154x_disable_iindpm_pulse\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_a, SGM4154x_IINDPM_INT_MASK,
                     SGM4154x_IINDPM_INT_DISABLE);
	return ret;
}

#ifdef SGM_BASE
float sgm4154x_get_charger_output_power(struct sgm4154x_device *sgm)
{
	int ret;
	int i = 0x1F;
	int j = 0;
	int vlim;
	int ilim;
	int temp;
	int offset;
	int output_volt;
	int output_curr;
	float o_i,o_v;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_6, &vlim); //read default setting to save
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_6 fail\n");
		return ret;
	}

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &ilim); //read default setting to save
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_0 fail\n");
		return ret;
	}

	regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VINDPM_V_MASK, 0);
	while(i--){

		regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, i);
		mdelay(50);
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &temp);
		if (ret){
			pr_err("read SGM4154x_CHRG_CTRL_a fail\n");
			return ret;
		}
		if (1 == (!!(temp&0x20))){
			output_curr = 100 + i*100; //mA
			if (0x1F == i)
				output_curr = 3800;	      //mA
		}
	}

	regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, SGM4154x_IINDPM_I_MASK);
	for(j = 0;j <= 0xF;j ++)
	{
		regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VINDPM_V_MASK, j);

		mdelay(10);
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &temp);
		if (ret){
			pr_err("read SGM4154x_CHRG_CTRL_a fail\n");
			return ret;
		}

		if (1 == (!!(temp&0x40))){

			temp = sgm4154x_get_vindpm_offset_os(sgm);
			if (0 == temp)
				offset = 3900;  //mv
			else if (1 == temp)
				offset = 5900;  //mv
			else if (2 == temp)
				offset = 7500;  //mv
			else if (3 == temp)
				offset = 10500; //mv
			output_volt = offset + j*100; //mv
		}
	}
	o_i = (float)output_curr/1000;
	o_v = (float)output_volt/1000;
    return o_i * o_v;
}
#endif

static int sgm4154x_set_vac_ovp(struct sgm4154x_device *sgm)
{
	int reg_val;

	dev_notice(sgm->dev, "%s", __func__);
	reg_val = 0xFF & SGM4154x_VAC_OVP_MASK;

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VAC_OVP_MASK, reg_val);
}

static int sgm4154x_set_recharge_volt(struct sgm4154x_device *sgm, int recharge_volt)
{
	int reg_val;
	dev_notice(sgm->dev, "%s:%d", __func__, recharge_volt);
	reg_val = (recharge_volt - SGM4154x_VRECHRG_OFFSET_mV) / SGM4154x_VRECHRG_STEP_mV;

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VRECHARGE, reg_val);
}

#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
static int get_charger_type(struct sgm4154x_device * sgm)
{
	enum power_supply_usb_type usb_type;

	switch(sgm->state.chrg_type) {
		case SGM4154x_USB_SDP:
			usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			break;

		case SGM4154x_USB_CDP:
			usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			break;

		case SGM4154x_USB_DCP:
			usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			break;

		case SGM4154x_NON_STANDARD:
			usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;

		default:
			usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
	}

	if (sgm->use_ext_usb_psy && sgm->usb) {
		int ret;
		union power_supply_propval val = {0};
		ret = power_supply_get_property(sgm->usb,
				POWER_SUPPLY_PROP_USB_TYPE, &val);
		if (!ret) {
			usb_type = val.intval;
		}
	}

	pr_debug("usb_type:%d\n", usb_type);
	return usb_type;
}
#endif
static int sgm4154x_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return true;
	default:
		return false;
	}
}
static int sgm4154x_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0) {
			sgm->update_pending = true;
			sgm->user_ilim = -EINVAL;
			ret = 0;
			pr_info("Clear user input limit\n");
		} else {
			sgm->user_ilim = val->intval;
			ret = sgm4154x_set_input_curr_lim(sgm, val->intval);
			pr_info("Set user input limit: %duA\n", sgm->user_ilim);
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_set_input_volt_lim(sgm, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval < 0) {
			sgm->update_pending = true;
			sgm->user_ichg = -EINVAL;
			ret = 0;
			pr_info("Clear user charging current limit\n");
		} else {
			sgm->user_ichg = val->intval;
			ret = sgm4154x_set_ichrg_curr(sgm, val->intval);
			pr_info("Set user charging current limit: %duA\n", sgm->user_ichg);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = sgm4154x_set_thermal_mitigation(sgm, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int sgm4154x_get_charging_status(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int chrg_stat;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret) {
		pr_err("read SGM4154x_CHRG_STAT fail\n");
		return ret;
	}

	chrg_stat &= SGM4154x_CHG_STAT_MASK;

	return chrg_stat;
}

static int sgm4154x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	struct sgm4154x_state state;
	int chrg_status = 0;
	int ret = 0;

	mutex_lock(&sgm->lock);
	//ret = sgm4154x_get_state(sgm, &state);
	state = sgm->state;
	mutex_unlock(&sgm->lock);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		chrg_status = sgm4154x_get_charging_status(sgm);
		if (!state.chrg_type || (state.chrg_type == SGM4154x_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!chrg_status || (chrg_status == SGM4154x_TERM_CHRG))
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		chrg_status = sgm4154x_get_charging_status(sgm);
		switch (chrg_status) {
		case SGM4154x_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case SGM4154x_TERM_CHRG:
		case SGM4154x_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SGM4154x_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SGM4154x_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state.vbus_gd;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = sgm4154x_power_supply_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = get_charger_type(sgm);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.chrg_fault & 0xF8)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		switch (state.health) {
		case SGM4154x_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
			break;
		case SGM4154x_TEMP_WARM:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case SGM4154x_TEMP_COOL:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case SGM4154x_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbus_adc;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibus_adc;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_get_input_volt_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		ret = 0;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = sgm->curr_thermal_level;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = sgm->num_thermal_levels;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm4154x_get_input_curr_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sgm4154x_get_ichg_curr(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static bool sgm4154x_state_changed(struct sgm4154x_device *sgm,
				  struct sgm4154x_state *new_state)
{
	struct sgm4154x_state old_state;

	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	return (old_state.chrg_stat != new_state->chrg_stat ||
		old_state.chrg_fault != new_state->chrg_fault ||
		old_state.online != new_state->online ||
		old_state.health != new_state->health ||
		old_state.chrg_type != new_state->chrg_type ||
		old_state.vbus_gd != new_state->vbus_gd ||
		old_state.ibus_limit != new_state->ibus_limit ||
		old_state.detect_done != new_state->detect_done
		);
}

static void sgm4154x_dump_register(struct sgm4154x_device * sgm)
{
	int i = 0;
	u32 reg = 0;

	for(i=0; i<=SGM4154x_CHRG_CTRL_f; i++) {
		regmap_read(sgm->regmap, i, &reg);
		pr_info("REG[0x%x]=0x%x\n", i, reg);
	}
}

#if 0
static bool sgm4154x_dpdm_detect_is_done(struct sgm4154x_device * sgm)
{
	int chrg_stat;
	int ret;

	ret = regmap_read(sgm->regmap, SGM4154x_INPUT_DET, &chrg_stat);
	if(ret) {
		dev_err(sgm->dev, "Check DPDM detecte error\n");
	}

	return (chrg_stat&SGM4154x_DPDM_ONGOING)?true:false;
}
#endif

static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	//static u8 last_chg_method = 0;
	bool state_changed = false;
	struct sgm4154x_state state;
	struct sgm4154x_device *sgm = container_of(work,
					struct sgm4154x_device,
					charge_monitor_work.work);

	ret = sgm4154x_get_state(sgm, &state);
	state_changed = sgm4154x_state_changed(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if (state_changed) {
		if (!sgm->state.chrg_type)
			pr_info("No charger is present\n");
		sgm4154x_dump_register(sgm);
		cancel_delayed_work(&sgm->charge_detect_delayed_work);
		schedule_delayed_work(&sgm->charge_detect_delayed_work, 0);
		power_supply_changed(sgm->charger);
	}

	schedule_delayed_work(&sgm->charge_monitor_work, 10*HZ);
}

static int sgm4154x_mmi_charger_init(struct sgm_mmi_charger *chg);
static void charger_detect_work_func(struct work_struct *work)
{
	int ret;
	//static int charge_type_old = 0;
	int curr_in_limit = 0;
	static int prev_in_limit = 0;
	bool state_changed = false;
	struct sgm4154x_state state;
	struct sgm4154x_device *sgm = container_of(work,
					struct sgm4154x_device,
					charge_detect_delayed_work.work);

	if (!sgm->charger_wakelock->active)
		__pm_stay_awake(sgm->charger_wakelock);

	if (sgm->use_ext_usb_psy) {
		if (!sgm->usb) {
			const char *usb_psy_name = NULL;
			ret = device_property_read_string(sgm->dev,
						"mmi,ext-usb-psy-name",
						&usb_psy_name);
			if (!ret && usb_psy_name)
				sgm->usb = power_supply_get_by_name(usb_psy_name);
			if (!sgm->usb) {
				pr_info("No USB power supply found, redetecting...\n");
				cancel_delayed_work(&sgm->charge_detect_delayed_work);
				schedule_delayed_work(&sgm->charge_detect_delayed_work,
							msecs_to_jiffies(1000));
				goto err;
			} else {
				pr_info("USB power supply is found\n");
			}
		}
	}

	if (sgm->use_ext_wls_psy) {
		if (!sgm->wls_psy) {
			const char *wls_psy_name = NULL;
			ret = device_property_read_string(sgm->dev,
						"mmi,ext-wls-psy-name",
						&wls_psy_name);
			if (!ret && wls_psy_name)
				sgm->wls_psy = power_supply_get_by_name(wls_psy_name);
			if (!sgm->wls_psy) {
				pr_info("No Wireless power supply found, redetecting...\n");
				cancel_delayed_work(&sgm->charge_detect_delayed_work);
				schedule_delayed_work(&sgm->charge_detect_delayed_work,
							msecs_to_jiffies(1000));
				goto err;
			} else {
				pr_info("WLS power supply is found\n");
			}
		}
	}

	if (sgm->mmi_charger &&
	    (!sgm->mmi_charger->fg_psy || !sgm->mmi_charger->driver)) {
		sgm4154x_mmi_charger_init(sgm->mmi_charger);
	}

	ret = sgm4154x_get_state(sgm, &state);
	state_changed = sgm4154x_state_changed(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if(!sgm->state.vbus_gd) {
		if (sgm4154x_is_enabled_charging(sgm))
			sgm4154x_disable_charger(sgm);
		dev_err(sgm->dev, "Vbus not present, disable charge\n");
		goto err;
	}
	if(!state.online)
	{
		dev_err(sgm->dev, "Vbus not online\n");
		goto err;
	}
	if(!state.detect_done) {
		dev_err(sgm->dev, "DPDM detecte not done\n");
		goto err;
	}
#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
	switch(sgm->state.chrg_type) {
		case SGM4154x_USB_SDP:
			pr_err("SGM4154x charger type: SDP\n");
			curr_in_limit = 500000;
			break;

		case SGM4154x_USB_CDP:
			pr_err("SGM4154x charger type: CDP\n");
			curr_in_limit = 1500000;
			break;

		case SGM4154x_USB_DCP:
			pr_err("SGM4154x charger type: DCP\n");
			curr_in_limit = 1500000;
			break;

		case SGM4154x_UNKNOWN:
			pr_err("SGM4154x charger type: UNKNOWN\n");
			curr_in_limit = 500000;
			break;
		case SGM4154x_NON_STANDARD:
			pr_err("SGM4154x charger type: NON_STANDARD\n");
			curr_in_limit = 500000;
			break;

		case SGM4154x_OTG_MODE:
			pr_err("SGM4154x OTG mode do nothing\n");
			goto err;

		default:
			if (is_wls_online(sgm)) {
				pr_err("SGM4154x charger type: Wireless\n");
				sgm->state.chrg_type = SGM4154x_WLS_TYPE;
				curr_in_limit = sgm->init_data.wlim;
				break;
			} else {
				pr_err("SGM4154x charger type: default\n");
				//curr_in_limit = 500000;
				//break;
				return;
			}
	}

	//Enable charge IC
	if (!sgm->mmi_charger) {
		sgm4154x_enable_charger(sgm);
	}

	if ((!sgm->use_ext_usb_psy || !sgm->usb) &&
	    prev_in_limit != curr_in_limit) {
		dev_info(sgm->dev, "Update: curr_in_limit = %d\n",
					curr_in_limit);
		sgm4154x_set_input_curr_lim(sgm, curr_in_limit);
		prev_in_limit = curr_in_limit;
	}
#endif
	if (state_changed) {
		sgm4154x_dump_register(sgm);
		power_supply_changed(sgm->charger);
	}
	return;
err:
	//release wakelock
	if (state_changed)
		power_supply_changed(sgm->charger);
	dev_info(sgm->dev, "Relax wakelock\n");
	__pm_relax(sgm->charger_wakelock);
	return;
}

static irqreturn_t sgm4154x_irq_handler_thread(int irq, void *private)
{
	struct sgm4154x_device *sgm = private;

	//lock wakelock
	pr_info("entry irq handler\n");
	sgm->update_pending = true;

	#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
	cancel_delayed_work(&sgm->charge_detect_delayed_work);
	schedule_delayed_work(&sgm->charge_detect_delayed_work, 100);
	//power_supply_changed(sgm->charger);
	#endif
	return IRQ_HANDLED;
}

static enum power_supply_property sgm4154x_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_PRESENT
};

static char *sgm4154x_charger_supplied_to[] = {
	"battery",
};

static struct power_supply_desc sgm4154x_power_supply_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.usb_types = sgm4154x_usb_type,
	.num_usb_types = ARRAY_SIZE(sgm4154x_usb_type),
	.properties = sgm4154x_power_supply_props,
	.num_properties = ARRAY_SIZE(sgm4154x_power_supply_props),
	.get_property = sgm4154x_charger_get_property,
	.set_property = sgm4154x_charger_set_property,
	.property_is_writeable = sgm4154x_property_is_writeable,
};

#ifdef SGM_BASE
static bool sgm4154x_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SGM4154x_CHRG_CTRL_0...SGM4154x_CHRG_CTRL_f:
		return true;
	default:
		return false;
	}
}
#endif

static const struct regmap_config sgm4154x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SGM4154x_CHRG_CTRL_f,
#ifdef SGM_BASE
	.reg_defaults	= sgm4154x_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(sgm4154x_reg_defs),
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sgm4154x_is_volatile_reg,
#endif
};

static int sgm4154x_power_supply_init(struct sgm4154x_device *sgm,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = sgm,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = sgm4154x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm4154x_charger_supplied_to);

	sgm->charger = devm_power_supply_register(sgm->dev,
						 &sgm4154x_power_supply_desc,
						 &psy_cfg);
	if (IS_ERR(sgm->charger))
		return PTR_ERR(sgm->charger);

	return 0;
}

static int sgm4154x_hw_init(struct sgm4154x_device *sgm)
{
	int ret = 0;

	ret = sgm4154x_set_ichrg_curr(sgm, sgm->init_data.ichg);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_prechrg_curr(sgm, sgm->init_data.iprechg);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_chrg_volt(sgm, sgm->init_data.vreg);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_term_curr(sgm, sgm->init_data.iterm);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_input_volt_lim(sgm, sgm->init_data.vlim);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_input_curr_lim(sgm, sgm->init_data.ilim);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_vac_ovp(sgm);//14V
	if (ret)
		goto err_out;

	ret = sgm4154x_set_recharge_volt(sgm, sgm->init_data.vrechg);//100~200mv
	if (ret)
		goto err_out;

	ret = sgm4154x_enable_hw_jeita(sgm, sgm->enable_hw_jeita);
	if (ret)
		goto err_out;

	ret = sgm4154x_disable_watchdog(sgm);
	if (ret)
		goto err_out;

	ret = sgm4154x_disable_vindpm_int_pulse(sgm);
	if (ret)
		goto err_out;

	ret = sgm4154x_disable_iindpm_int_pulse(sgm);
	if (ret)
		goto err_out;

	if (sgm->init_data.charger_disabled) {
		ret = sgm4154x_disable_charger(sgm);
		ret |= sgm4154x_set_hiz_en(sgm, true);
	} else {
		ret |= sgm4154x_set_hiz_en(sgm, false);
		ret = sgm4154x_enable_charger(sgm);
	}
	if (ret)
		goto err_out;

	dev_notice(sgm->dev, "ichrg:%d vchrg:%d iprechrg:%d iterm:%d"
		" ilim:%d vlim:%d vrechg:%d chg_dis:%d hw_jeita_en:%d\n",
		sgm->init_data.ichg, sgm->init_data.vreg,
		sgm->init_data.iprechg, sgm->init_data.iterm,
		sgm->init_data.ilim, sgm->init_data.vlim,
		sgm->init_data.vrechg, sgm->init_data.charger_disabled,
		sgm->enable_hw_jeita);

	return 0;

err_out:
	return ret;
}

static int sgm4154x_parse_dt(struct sgm4154x_device *sgm)
{
	int ret;
	//u32 val = 0;
	int irq_gpio = 0, irqn = 0;
	enum of_gpio_flags flags;
	const char *usb_psy_name = NULL;
	const char *wls_psy_name = NULL;
	int i, len;
	u32 prev, val;
	struct device_node *node = sgm->dev->of_node;

	#if 0
	ret = device_property_read_u32(sgm->dev, "watchdog-timer",
				       &sgm->watchdog_timer);
	if (ret)
		sgm->watchdog_timer = SGM4154x_WATCHDOG_DIS;

	if (sgm->watchdog_timer > SGM4154x_WATCHDOG_MAX ||
	    sgm->watchdog_timer < SGM4154x_WATCHDOG_DIS)
		return -EINVAL;
	#endif
	ret = device_property_read_u32(sgm->dev,
				       "mmi,batt-protect-duration",
				       &sgm->batt_protect_duration);
	if (ret)
		sgm->batt_protect_duration = DEF_PROTECT_DURATION_MS;

	ret = device_property_read_u32(sgm->dev,
				       "mmi,vbat-ovp-threshold",
				       &sgm->vbat_ovp_threshold);
	if (ret)
		sgm->vbat_ovp_threshold = DEF_VBAT_OVP_MV;

	ret = device_property_read_u32(sgm->dev,
				       "mmi,ibat-ocp-threshold",
				       &sgm->ibat_ocp_threshold);
	if (ret)
		sgm->ibat_ocp_threshold = DEF_IBAT_OCP_MA;

	ret = device_property_read_u32(sgm->dev,
				       "mmi,ibat-occp-threshold",
				       &sgm->ibat_occp_threshold);
	if (ret)
		sgm->ibat_occp_threshold = DEF_IBAT_OCCP_MA;

	sgm->high_load_en_gpio = of_get_named_gpio_flags(sgm->dev->of_node,
					"mmi,high-load-en-gpio", 0, &flags);
	if (gpio_is_valid(sgm->high_load_en_gpio)) {
		ret = gpio_request(sgm->high_load_en_gpio,
					"high-load-en-gpio");
		if (ret) {
			dev_err(sgm->dev, "%s: %d gpio request failed\n",
					__func__, sgm->high_load_en_gpio);
			sgm->high_load_en_gpio = -EINVAL;
		} else {
			sgm->high_load_active_low = !!(flags & OF_GPIO_ACTIVE_LOW);
			/* Disable batt high load switch by default */
			gpio_direction_output(sgm->high_load_en_gpio,
					sgm->high_load_active_low);
		}
	}

	sgm->low_load_en_gpio = of_get_named_gpio_flags(sgm->dev->of_node,
					"mmi,low-load-en-gpio", 0, &flags);
	if (gpio_is_valid(sgm->low_load_en_gpio)) {
		ret = gpio_request(sgm->low_load_en_gpio,
					"low-load-en-gpio");
		if (ret) {
			dev_err(sgm->dev, "%s: %d gpio request failed\n",
					__func__, sgm->low_load_en_gpio);
			sgm->low_load_en_gpio = -EINVAL;
		} else {
			sgm->low_load_active_low = !!(flags & OF_GPIO_ACTIVE_LOW);
			/* Disable batt low load switch by default */
			gpio_direction_output(sgm->low_load_en_gpio,
					sgm->low_load_active_low);
		}
	}

	ret = device_property_read_string(sgm->dev,
					"mmi,ext-usb-psy-name",
					&usb_psy_name);
	if (!ret && usb_psy_name)
		sgm->use_ext_usb_psy = true;
	else
		sgm->use_ext_usb_psy = false;

	ret = device_property_read_string(sgm->dev,
					"mmi,ext-wls-psy-name",
					&wls_psy_name);
	if (!ret && wls_psy_name)
		sgm->use_ext_wls_psy = true;
	else
		sgm->use_ext_wls_psy = false;

	sgm->vdd_i2c_vreg = devm_regulator_get_optional(sgm->dev, "vdd-i2c");

	sgm->enable_hw_jeita = of_property_read_bool(sgm->dev->of_node,
					"enable_hw_jeita");

	sgm->init_data.charger_disabled = of_property_read_bool(sgm->dev->of_node,
					"init-charger-disabled");

	ret = device_property_read_u32(sgm->dev,
				       "vrechg-millivolt",
				       &sgm->init_data.vrechg);
	if (ret)
		sgm->init_data.vrechg = 200;

	if (sgm->init_data.vrechg > SGM4154x_VRECHRG_OFFSET_mV)
		sgm->init_data.vrechg = 200;
	else
		sgm->init_data.vrechg = 100;

	ret = device_property_read_u32(sgm->dev,
				       "iprechg-microamp",
				       &sgm->init_data.iprechg);
	if (ret)
		sgm->init_data.iprechg = SGM4154x_PRECHRG_I_DEF_uA;

	if (sgm->init_data.iprechg > SGM4154x_PRECHRG_I_MAX_uA ||
	    sgm->init_data.iprechg < SGM4154x_PRECHRG_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "iterm-microamp",
				       &sgm->init_data.iterm);
	if (ret)
		sgm->init_data.iterm = SGM4154x_TERMCHRG_I_DEF_uA;

	if (sgm->init_data.iterm > SGM4154x_TERMCHRG_I_MAX_uA ||
	    sgm->init_data.iterm < SGM4154x_TERMCHRG_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "ichg-max-microamp",
				       &sgm->init_data.max_ichg);
	if (ret)
		sgm->init_data.max_ichg = SGM4154x_ICHRG_I_MAX_uA;

	if (sgm->init_data.max_ichg > SGM4154x_ICHRG_I_MAX_uA ||
	    sgm->init_data.max_ichg < SGM4154x_ICHRG_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "vchg-max-microvolt",
				       &sgm->init_data.max_vreg);
	if (ret)
		sgm->init_data.max_vreg = SGM4154x_VREG_V_MAX_uV;

	if (sgm->init_data.max_vreg > SGM4154x_VREG_V_MAX_uV ||
	    sgm->init_data.max_vreg < SGM4154x_VREG_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "ichg-microamp",
				       &sgm->init_data.ichg);
	if (ret)
		sgm->init_data.ichg = SGM4154x_ICHRG_I_DEF_uA;

	if (sgm->init_data.ichg > SGM4154x_ICHRG_I_MAX_uA ||
	    sgm->init_data.ichg < SGM4154x_ICHRG_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "vchg-microvolt",
				       &sgm->init_data.vreg);
	if (ret)
		sgm->init_data.vreg = SGM4154x_VREG_V_DEF_uV;

	if (sgm->init_data.vreg > SGM4154x_VREG_V_MAX_uV ||
	    sgm->init_data.vreg < SGM4154x_VREG_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "input-voltage-limit-microvolt",
				       &sgm->init_data.vlim);
	if (ret)
		sgm->init_data.vlim = SGM4154x_VINDPM_DEF_uV;

	if (sgm->init_data.vlim > SGM4154x_VINDPM_V_MAX_uV ||
	    sgm->init_data.vlim < SGM4154x_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "input-current-limit-microamp",
				       &sgm->init_data.ilim);
	if (ret)
		sgm->init_data.ilim = SGM4154x_IINDPM_DEF_uA;

	if (sgm->init_data.ilim > SGM4154x_IINDPM_I_MAX_uA ||
	    sgm->init_data.ilim < SGM4154x_IINDPM_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "wls-current-limit-microamp",
				       &sgm->init_data.wlim);
	if (ret)
		sgm->init_data.wlim = SGM4154x_IINDPM_I_MIN_uA;

	pr_info("wireless input current limit: %duA\n", sgm->init_data.wlim);

	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}
	ret = gpio_request(irq_gpio, "sgm4154x irq pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	sgm->client->irq = irqn;

	sgm->chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (gpio_is_valid(sgm->chg_en_gpio))
	{
		ret = gpio_request(sgm->chg_en_gpio, "sgm4154x chg en pin");
		if (ret) {
			dev_err(sgm->dev, "%s: %d gpio request failed\n",
						__func__,
						sgm->chg_en_gpio);
			sgm->chg_en_gpio = -EINVAL;
		} else {
			gpio_direction_output(sgm->chg_en_gpio,
					sgm->init_data.charger_disabled);
		}
	}
#ifdef SGM_BASE
	/* sw jeita */
	sgm->enable_sw_jeita = of_property_read_bool(sgm->dev->of_node, "enable_sw_jeita");

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_above_t4_cv", &val) >= 0)
		sgm->data.jeita_temp_above_t4_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_ABOVE_T4_CV:%d\n",JEITA_TEMP_ABOVE_T4_CV);
		sgm->data.jeita_temp_above_t4_cv = JEITA_TEMP_ABOVE_T4_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t3_to_t4_cv", &val) >= 0)
		sgm->data.jeita_temp_t3_to_t4_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T3_TO_T4_CV:%d\n",JEITA_TEMP_T3_TO_T4_CV);
		sgm->data.jeita_temp_t3_to_t4_cv = JEITA_TEMP_T3_TO_T4_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t2_to_t3_cv", &val) >= 0)
		sgm->data.jeita_temp_t2_to_t3_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T2_TO_T3_CV:%d\n",JEITA_TEMP_T2_TO_T3_CV);
		sgm->data.jeita_temp_t2_to_t3_cv = JEITA_TEMP_T2_TO_T3_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t1_to_t2_cv", &val) >= 0)
		sgm->data.jeita_temp_t1_to_t2_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T1_TO_T2_CV:%d\n",JEITA_TEMP_T1_TO_T2_CV);
		sgm->data.jeita_temp_t1_to_t2_cv = JEITA_TEMP_T1_TO_T2_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t0_to_t1_cv", &val) >= 0)
		sgm->data.jeita_temp_t0_to_t1_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T0_TO_T1_CV:%d\n",JEITA_TEMP_T0_TO_T1_CV);
		sgm->data.jeita_temp_t0_to_t1_cv = JEITA_TEMP_T0_TO_T1_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_below_t0_cv", &val) >= 0)
		sgm->data.jeita_temp_below_t0_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_BELOW_T0_CV:%d\n",JEITA_TEMP_BELOW_T0_CV);
		sgm->data.jeita_temp_below_t0_cv = JEITA_TEMP_BELOW_T0_CV;
	}
	pr_info("enable_sw_jeita = %d,CV1 = %d,CV2 = %d,CV3 = %d,CV4 = %d,CV5 = %d,CV6 = %d\n",
			sgm->enable_sw_jeita,sgm->data.jeita_temp_above_t4_cv,sgm->data.jeita_temp_t3_to_t4_cv,
			sgm->data.jeita_temp_t2_to_t3_cv,sgm->data.jeita_temp_t1_to_t2_cv,
			sgm->data.jeita_temp_t0_to_t1_cv,sgm->data.jeita_temp_below_t0_cv);

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_above_t4_cc_current", &val) >= 0)
		sgm->data.jeita_temp_above_t4_cc_current = val;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t3_to_t4_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t3_to_t4_cc_current = val;
	else
		sgm->data.jeita_temp_t3_to_t4_cc_current = JEITA_TEMP_T3_TO_T4_CC_CURRENT;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t2_to_t3_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t2_to_t3_cc_current = val;
	else
		sgm->data.jeita_temp_t2_to_t3_cc_current = JEITA_TEMP_T2_TO_T3_CC_CURRENT;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t1_to_t2_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t1_to_t2_cc_current = val;
	else
		sgm->data.jeita_temp_t1_to_t2_cc_current = JEITA_TEMP_T1_TO_T2_CC_CURRENT;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_below_t0_cc_current", &val) >= 0)
		sgm->data.jeita_temp_below_t0_cc_current = val;
	else
		sgm->data.jeita_temp_below_t0_cc_current = JEITA_TEMP_BELOW_T0_CC_CURRENT;

	pr_err("CC1 = %d,CC2 = %d,CC3 = %d,CC4 = %d,CC5 = %d\n",
			sgm->data.jeita_temp_above_t4_cc_current,sgm->data.jeita_temp_t3_to_t4_cc_current,
			sgm->data.jeita_temp_t2_to_t3_cc_current,sgm->data.jeita_temp_t1_to_t2_cc_current,
			sgm->data.jeita_temp_below_t0_cc_current);

	if (of_property_read_u32(sgm->dev->of_node, "temp_t4_thres", &val) >= 0)
		sgm->data.temp_t4_thres = val;
	else {
		dev_err(sgm->dev, "use default TEMP_T4_THRES:%d\n",TEMP_T4_THRES);
		sgm->data.temp_t4_thres = TEMP_T4_THRES;
	}
	if (of_property_read_u32(sgm->dev->of_node, "temp_t4_thres_minus_x_degree", &val) >= 0)
		sgm->data.temp_t4_thres_minus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T4_THRES_MINUS_X_DEGREE:%d\n",TEMP_T4_THRES_MINUS_X_DEGREE);
		sgm->data.temp_t4_thres_minus_x_degree = TEMP_T4_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t3_thres", &val) >= 0)
		sgm->data.temp_t3_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T3_THRES:%d\n",TEMP_T3_THRES);
		sgm->data.temp_t3_thres = TEMP_T3_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t3_thres_minus_x_degree", &val) >= 0)
		sgm->data.temp_t3_thres_minus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T3_THRES_MINUS_X_DEGREE:%d\n",TEMP_T3_THRES_MINUS_X_DEGREE);
		sgm->data.temp_t3_thres_minus_x_degree = TEMP_T3_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t2_thres", &val) >= 0)
		sgm->data.temp_t2_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T2_THRES:%d\n",TEMP_T2_THRES);
		sgm->data.temp_t2_thres = TEMP_T2_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t2_thres_plus_x_degree", &val) >= 0)
		sgm->data.temp_t2_thres_plus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T2_THRES_PLUS_X_DEGREE:%d\n",TEMP_T2_THRES_PLUS_X_DEGREE);
		sgm->data.temp_t2_thres_plus_x_degree = TEMP_T2_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t1_thres", &val) >= 0)
		sgm->data.temp_t1_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T1_THRES:%d\n",TEMP_T1_THRES);
		sgm->data.temp_t1_thres = TEMP_T1_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t1_thres_plus_x_degree", &val) >= 0)
		sgm->data.temp_t1_thres_plus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T1_THRES_PLUS_X_DEGREE:%d\n",TEMP_T1_THRES_PLUS_X_DEGREE);
		sgm->data.temp_t1_thres_plus_x_degree = TEMP_T1_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t0_thres", &val) >= 0)
		sgm->data.temp_t0_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T0_THRES:%d\n",TEMP_T0_THRES);
		sgm->data.temp_t0_thres = TEMP_T0_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t0_thres_plus_x_degree", &val) >= 0)
		sgm->data.temp_t0_thres_plus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T0_THRES_PLUS_X_DEGREE:%d\n",TEMP_T0_THRES_PLUS_X_DEGREE);
		sgm->data.temp_t0_thres_plus_x_degree = TEMP_T0_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_neg_10_thres", &val) >= 0)
		sgm->data.temp_neg_10_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_NEG_10_THRES:%d\n",TEMP_NEG_10_THRES);
		sgm->data.temp_neg_10_thres = TEMP_NEG_10_THRES;
	}
	pr_info("%s,thres4 = %d,thres3 = %d,thres2 = %d,thres1 = %d,thres0 = %d\n",
			sgm->data.temp_t4_thres,sgm->data.temp_t3_thres,
			sgm->data.temp_t2_thres,sgm->data.temp_t1_thres,
			sgm->data.temp_t0_thres);
#endif
	ret = of_property_count_elems_of_size(node, "mmi,thermal-mitigation",
							sizeof(u32));
	if (ret <= 0) {
		return 0;
	}

	len = ret;
	prev = sgm->init_data.max_ichg;
	for (i = 0; i < len; i++) {
		ret = of_property_read_u32_index(node,
					"mmi,thermal-mitigation",
					i, &val);
		if (ret < 0) {
			pr_err("failed to get thermal-mitigation[%d], ret=%d\n", i, ret);
			return ret;
		}
		pr_info("thermal-mitigation[%d], val=%d, prev=%d\n", i, val, prev);
		if (val > prev) {
			pr_err("Thermal levels should be in descending order\n");
			sgm->num_thermal_levels = -EINVAL;
			return 0;
		}
		prev = val;
	}

	sgm->thermal_levels = devm_kcalloc(sgm->dev, len + 1,
					sizeof(*sgm->thermal_levels),
					GFP_KERNEL);
	if (!sgm->thermal_levels)
		return -ENOMEM;

	sgm->thermal_levels[0] = sgm->init_data.max_ichg;
	ret = of_property_read_u32_array(node, "mmi,thermal-mitigation",
						&sgm->thermal_levels[1], len);
	if (ret < 0) {
		pr_err("Error in reading mmi,thermal-mitigation, rc=%d\n", ret);
		return ret;
	}
	sgm->num_thermal_levels = len;
	sgm->thermal_fcc_ua = sgm->init_data.max_ichg;

	return 0;
}

static int sgm4154x_set_otg_voltage(struct sgm4154x_device *sgm, int uv)
{
	int ret = 0;
	int reg_val = -1;
	int i = 0;
	while(i<4){
		if (uv == BOOST_VOLT_LIMIT[i]){
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;
	reg_val = reg_val << 4;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_OTG_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_otg_current(struct sgm4154x_device *sgm, int ua)
{
	int ret = 0;

	if (ua == BOOST_CURRENT_LIMIT[0]){
		ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2, SGM4154x_OTG_EN,
                     0);
	}
	else if (ua == BOOST_CURRENT_LIMIT[1]){
		ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2, SGM4154x_OTG_EN,
                     BIT(7));
	}
	return ret;
}

static int sgm4154x_enable_vbus(struct regulator_dev *rdev)
{
	struct sgm4154x_device *sgm = rdev_get_drvdata(rdev);
	int ret = 0;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
                     SGM4154x_OTG_EN);
	return ret;
}

static int sgm4154x_disable_vbus(struct regulator_dev *rdev)
{
	struct sgm4154x_device *sgm = rdev_get_drvdata(rdev);
	int ret = 0;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
                     0);

	return ret;
}

static int sgm4154x_is_enabled_vbus(struct regulator_dev *rdev)
{
	struct sgm4154x_device *sgm = rdev_get_drvdata(rdev);
	int temp = 0;
	int ret = 0;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_1, &temp);
	return (temp&SGM4154x_OTG_EN)? 1 : 0;
}

static struct regulator_ops sgm4154x_vbus_ops = {
	.enable = sgm4154x_enable_vbus,
	.disable = sgm4154x_disable_vbus,
	.is_enabled = sgm4154x_is_enabled_vbus,
};

static struct regulator_desc sgm4154x_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &sgm4154x_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int sgm4154x_vbus_regulator_register(struct sgm4154x_device *sgm)
{
	struct regulator_config config = {};
	int ret = 0;
	/* otg regulator */
	config.dev = sgm->dev;
	config.driver_data = sgm;
	sgm->otg_rdev = devm_regulator_register(sgm->dev,
						&sgm4154x_otg_rdesc, &config);
	sgm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	if (IS_ERR(sgm->otg_rdev)) {
		ret = PTR_ERR(sgm->otg_rdev);
		pr_info("register otg regulator failed (%d)\n", ret);
	}
	return ret;
}

static int sgm4154x_suspend_notifier(struct notifier_block *nb,
                unsigned long event,
                void *dummy)
{
	struct sgm4154x_device *sgm = container_of(nb, struct sgm4154x_device, pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		pr_err("sgm4154x PM_SUSPEND \n");
		cancel_delayed_work_sync(&sgm->charge_monitor_work);
		sgm->sgm4154x_suspend_flag = 1;
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		pr_err("sgm4154x PM_RESUME \n");
		schedule_delayed_work(&sgm->charge_monitor_work, 0);
		sgm->sgm4154x_suspend_flag = 0;
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

static int sgm4154x_hw_chipid_detect(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int val = 0;

	ret = regmap_read(sgm->regmap,SGM4154x_CHRG_CTRL_b,&val);
	if (ret < 0)
	{
		pr_info("read SGM4154x_CHRG_CTRL_b fail\n");
		return ret;
	}

	pr_info("Reg[0x0B]=0x%x\n", val);

	return val;
}

#define EMPTY_BATTERY_VBAT 3200
static int sgm4154x_charger_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
	int rc;
	struct sgm_mmi_charger *chg = data;
	union power_supply_propval val = {0};

	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!rc)
		chg->batt_info.batt_mv = val.intval / 1000;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!rc)
		chg->batt_info.batt_ma = val.intval / 1000 * chg->ichg_polority;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!rc)
		chg->batt_info.batt_soc = val.intval;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!rc)
		chg->batt_info.batt_temp = val.intval / 10;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!rc)
		chg->batt_info.batt_status = val.intval;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL, &val);
	if (!rc)
		chg->batt_info.batt_full_uah = val.intval;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &val);
	if (!rc)
		chg->batt_info.batt_design_uah = val.intval;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER, &val);
	if (!rc)
		chg->batt_info.batt_chg_counter = val.intval;

	if (chg->chg_cfg.full_charged)
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;

	if (!chg->sgm->state.online) {
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));
	if (chg->paired_batt_info.batt_soc == 0 &&
	    chg->paired_batt_info.batt_mv < EMPTY_BATTERY_VBAT) {
		batt_info->batt_soc = 0;
		pr_warn("Force %s to empty from %d for empty paired battery\n",
					chg->fg_psy->desc->name,
					chg->batt_info.batt_soc);
	}

        return rc;
}

static int sgm4154x_charger_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
	int ret;
	struct sgm_mmi_charger *chg = data;
	struct sgm4154x_state state = chg->sgm->state;

	sgm4154x_get_state(chg->sgm, &state);

	chg->chg_info.chrg_mv = state.vbus_adc / 1000;
	chg->chg_info.chrg_ma = state.ibus_adc / 1000;
	chg->chg_info.chrg_type = get_charger_type(chg->sgm);
	if (chg->chg_info.chrg_type == POWER_SUPPLY_USB_TYPE_UNKNOWN &&
			is_wls_online(chg->sgm)) {
		chg->chg_info.chrg_type = SGM4154x_WLS_TYPE;
	}

	chg->chg_info.chrg_present = state.online;
	chg->chg_info.vbus_present = state.vbus_gd;

	if (state.ibus_limit > 0) {
		ret = sgm4154x_get_input_curr_lim(chg->sgm);
		if (ret < 0) {
			chg->chg_info.chrg_pmax_mw = 0;
		} else {
			chg->chg_info.chrg_pmax_mw = (ret / 1000) *
					(state.vbus_adc / 1000) / 1000;
		}
	} else {
		chg->chg_info.chrg_pmax_mw = 0;
	}

        memcpy(chg_info, &chg->chg_info, sizeof(struct mmi_charger_info));

        return 0;
}

static int sgm4154x_charger_config_charge(void *data, struct mmi_charger_cfg *config)
{
	int rc;
	u32 value;
	bool chg_en;
	static bool prev_chg_en = false;
	bool cfg_changed = false;
	struct sgm_mmi_charger *chg = data;
	struct sgm4154x_state state = chg->sgm->state;
	struct timespec64 now;
	static struct timespec64 start = {0};
	uint32_t elapsed_ms;
	bool high_load_en;
	bool low_load_en;
	static int prev_paired_load = PAIRED_LOAD_OFF;
	int curr_in_limit = chg->sgm->ilim;

	/* Monitor vbat and ibat for OVP and OCP */
	if (chg->batt_info.batt_mv >= chg->sgm->vbat_ovp_threshold) {
		chg->paired_load = PAIRED_LOAD_HIGH;
		chg->batt_protect_type = BATT_PROTECT_TYPE_OVP;
		ktime_get_real_ts64(&start);
		pr_err("ERROR: vbat OVP is triggered, batt_mv=%dmV, thre=%d\n",
				chg->batt_info.batt_mv,
				chg->sgm->vbat_ovp_threshold);
	} else if (chg->batt_info.batt_ma >= chg->sgm->ibat_occp_threshold) {
		chg->paired_load = PAIRED_LOAD_HIGH;
		chg->batt_protect_type = BATT_PROTECT_TYPE_OCCP;
		ktime_get_real_ts64(&start);
		pr_err("ERROR: ibat OCCP is triggered, batt_ma=%dmA, thre=%d\n",
				chg->batt_info.batt_ma,
				chg->sgm->ibat_occp_threshold);
	} else if (chg->batt_info.batt_ma <= -chg->sgm->ibat_ocp_threshold) {
		chg->paired_load = PAIRED_LOAD_LOW;
		chg->batt_protect_type = BATT_PROTECT_TYPE_OCP;
		ktime_get_real_ts64(&start);
		pr_err("ERROR: ibat OCP is triggered, batt_ma=%dmA, thre=%d\n",
				chg->batt_info.batt_ma,
				chg->sgm->ibat_ocp_threshold);
	} else if (chg->batt_protect_type != BATT_PROTECT_TYPE_NONE) {
		ktime_get_real_ts64(&now);
		elapsed_ms = (now.tv_sec - start.tv_sec) * 1000;
		elapsed_ms += (now.tv_nsec - start.tv_nsec) / 1000000;
		if (elapsed_ms < chg->sgm->batt_protect_duration) {
			if (!chg->chg_info.chrg_present &&
			    chg->batt_protect_type == BATT_PROTECT_TYPE_OCP) {
				chg->paired_load = PAIRED_LOAD_OFF;
			} else if (chg->chg_info.chrg_present &&
			  (chg->batt_protect_type == BATT_PROTECT_TYPE_OVP ||
			   chg->batt_protect_type == BATT_PROTECT_TYPE_OCCP)) {
				chg->paired_load = PAIRED_LOAD_HIGH;
				sgm4154x_set_hiz_en(chg->sgm, true);
			} else {
				chg->batt_protect_type = BATT_PROTECT_TYPE_NONE;
			}
		} else {
			chg->batt_protect_type = BATT_PROTECT_TYPE_NONE;
		}
		if (chg->batt_protect_type != BATT_PROTECT_TYPE_NONE) {
			pr_warn("keep batt protection:%d, elapsed=%dms\n",
					chg->batt_protect_type, elapsed_ms);
		} else {
			pr_warn("recover battery from protection\n");
		}
	}

	switch (chg->paired_load) {
	case PAIRED_LOAD_LOW:
		high_load_en = false;
		low_load_en = true;
		break;
	case PAIRED_LOAD_HIGH:
		high_load_en = true;
		low_load_en = true;
		break;
	case PAIRED_LOAD_OFF:
	default:
		high_load_en = false;
		low_load_en = false;
	}
	if (chg->paired_load != prev_paired_load) {
		if (gpio_is_valid(chg->sgm->high_load_en_gpio)) {
			gpio_set_value(chg->sgm->high_load_en_gpio,
				high_load_en ^ chg->sgm->high_load_active_low);
		}
		if (gpio_is_valid(chg->sgm->low_load_en_gpio)) {
			gpio_set_value(chg->sgm->low_load_en_gpio,
				low_load_en ^ chg->sgm->low_load_active_low);
		}
		prev_paired_load = chg->paired_load;
	}

	if (!chg->sgm->batt_present || chg->sgm->otg_enabled ||
	    chg->batt_protect_type == BATT_PROTECT_TYPE_OVP ||
	    chg->batt_protect_type == BATT_PROTECT_TYPE_OCCP) {
		if (chg->sgm->client->irq && chg->sgm->irq_enabled) {
			cfg_changed = true;
			disable_irq_wake(chg->sgm->client->irq);
			disable_irq(chg->sgm->client->irq);
			chg->sgm->irq_enabled = false;
			pr_warn("irq is disabled, bpd=%d, otg=%d, protect_type=%d\n",
				chg->sgm->batt_present, chg->sgm->otg_enabled,
				chg->batt_protect_type);
		}
		config->charging_disable = true;
		config->charger_suspend = true;
		chg->chg_cfg.charger_suspend = false;
	} else {
		if (!chg->sgm->irq_enabled && chg->sgm->client->irq) {
			cfg_changed = true;
			enable_irq_wake(chg->sgm->client->irq);
			enable_irq(chg->sgm->client->irq);
			chg->sgm->irq_enabled = true;
			pr_warn("irq is enabled, bpd=%d, otg=%d, protect_type=%d\n",
				chg->sgm->batt_present, chg->sgm->otg_enabled,
				chg->batt_protect_type);
		}
	}

	/* force to update charger configuration for user input or chip reset */
	if (chg->sgm->update_pending && state.input_ready) {
		chg->sgm->update_pending = false;
		chg->sgm->ilim = UINT_MAX;
		chg->chg_cfg.target_fv = -EINVAL;
		chg->chg_cfg.target_fcc = -EINVAL;
		chg->chg_cfg.chrg_iterm = -EINVAL;
		chg->chg_cfg.charging_disable = !config->charging_disable;
		chg->chg_cfg.charger_suspend = !config->charger_suspend;
	}

	/* configure the charger if changed */
	if (!config->charging_disable &&
	    !config->charger_suspend &&
	    config->target_fv != chg->chg_cfg.target_fv) {
		value = config->target_fv * 1000;
		rc = sgm4154x_set_chrg_volt(chg->sgm, value);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.target_fv = config->target_fv;
		}
	}
	if (!config->charging_disable &&
	    !config->charger_suspend &&
	    config->target_fcc != chg->chg_cfg.target_fcc) {
		value = config->target_fcc * 1000;
		rc = sgm4154x_set_ichrg_curr(chg->sgm, value);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.target_fcc = config->target_fcc;
		}
	}
	if (config->charger_suspend != chg->chg_cfg.charger_suspend) {
		rc = sgm4154x_set_hiz_en(chg->sgm, config->charger_suspend);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.charger_suspend = config->charger_suspend;
		}
	}

	sgm4154x_get_state(chg->sgm, &state);

	if (chg->chg_info.chrg_present && !state.hiz_en &&
	    chg->sgm->use_ext_usb_psy && chg->sgm->usb &&
		chg->chg_info.chrg_type != SGM4154x_WLS_TYPE) {
		if (state.ibus_limit < DEF_USB_CURRENT_MA &&
		    state.ibus_adc < DEF_USB_CURRENT_MA) {
			curr_in_limit = DEF_USB_CURRENT_MA - state.ibus_adc;
		} else if (state.ibus_limit > state.ibus_adc) {
			curr_in_limit = state.ibus_limit - state.ibus_adc;
		} else {
			curr_in_limit = 0;
		}
		if (curr_in_limit > chg->sgm->init_data.ilim)
			curr_in_limit = chg->sgm->init_data.ilim;
	} else if (chg->chg_info.chrg_present && chg->chg_info.chrg_type == SGM4154x_WLS_TYPE) {
			curr_in_limit = chg->sgm->init_data.wlim;
			pr_debug("wireless is on, set current limit to %d", curr_in_limit);
	} else {
			curr_in_limit = 0;
	}

	if (chg->sgm->ilim != curr_in_limit) {
		sgm4154x_set_input_curr_lim(chg->sgm, curr_in_limit);
		pr_info("ibus_limit=%d, ibus_adc=%d, curr=%d\n",
				state.ibus_limit, state.ibus_adc, curr_in_limit);
	}

	if (config->charging_disable != chg->chg_cfg.charging_disable) {
		value = config->charging_disable;
		if (!value)
			rc = sgm4154x_enable_charger(chg->sgm);
		else
			rc = sgm4154x_disable_charger(chg->sgm);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.charging_disable = config->charging_disable;
		}
	}

	if (config->taper_kickoff != chg->chg_cfg.taper_kickoff) {
		chg->chg_cfg.taper_kickoff = config->taper_kickoff;
		chg->chrg_taper_cnt = 0;
	}

	if (config->full_charged != chg->chg_cfg.full_charged) {
		chg->chg_cfg.full_charged = config->full_charged;
	}

	if (!config->charging_disable &&
	    !config->charger_suspend &&
	    config->chrg_iterm != chg->chg_cfg.chrg_iterm) {
		value = config->chrg_iterm * 1000;
		rc = sgm4154x_set_term_curr(chg->sgm, value);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.chrg_iterm = config->chrg_iterm;
		}
	}

	if (!config->charging_disable &&
	    !config->charger_suspend &&
	    config->fg_iterm != chg->chg_cfg.fg_iterm) {
		if (chg->fg_psy) {
			union power_supply_propval val = {0};
			val.intval = config->fg_iterm * 1000;
			rc = power_supply_set_property(chg->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
				&val);
			if (!rc) {
				cfg_changed = true;
				chg->chg_cfg.fg_iterm = config->fg_iterm;
			}
		} else {
			chg->chg_cfg.fg_iterm = config->fg_iterm;
		}
	}

	if (!config->charging_disable &&
	    !config->charger_suspend &&
	    config->charging_reset != chg->chg_cfg.charging_reset) {
		if (config->charging_reset) {
			sgm4154x_disable_charger(chg->sgm);
			chg->chg_cfg.target_fv = -EINVAL;
			chg->chg_cfg.target_fcc = -EINVAL;
			chg->chg_cfg.charging_disable = true;
			pr_info("Battery charging reset triggered\n");
		}
		cfg_changed = true;
		chg->chg_cfg.charging_reset = config->charging_reset;
	}

	chg_en = sgm4154x_is_enabled_charging(chg->sgm);
	if (!cfg_changed &&
	    chg->chg_info.chrg_present && !state.hiz_en &&
	    chg_en && prev_chg_en && chg->sgm->ichg > 0 &&
	    ((chg->batt_info.batt_mv + 50) * 1000) <= chg->sgm->vreg &&
	    state.chrg_stat != SGM4154x_FAST_CHRG &&
	    state.chrg_stat != SGM4154x_PRECHRG) {
		sgm4154x_disable_charger(chg->sgm);
		chg->chg_cfg.target_fv = -EINVAL;
		chg->chg_cfg.target_fcc = -EINVAL;
		chg->chg_cfg.charging_disable = true;
		pr_info("Battery charging reconfigure triggered\n");
	}
	prev_chg_en = chg_en;

	if (cfg_changed) {
		cancel_delayed_work(&chg->sgm->charge_monitor_work);
		schedule_delayed_work(&chg->sgm->charge_monitor_work,
						msecs_to_jiffies(200));
	}

	pr_info("cpd:%d, bpd:%d, otg:%d, protect:%d, load:%d, paired_ichg:%duA\n",
			chg->chg_info.chrg_present, chg->sgm->batt_present,
			chg->sgm->otg_enabled, chg->batt_protect_type,
			chg->paired_load, chg->paired_ichg);
	pr_info("chg_en:%d, ichg:%d, vchg:%d, ilim:%d, chg_st:0x%x, therm:%d, suspend:%d\n",
			chg_en, chg->sgm->ichg, chg->sgm->vreg, chg->sgm->ilim,
			state.chrg_stat, state.therm_stat, state.hiz_en);

	return 0;
}

#define TAPER_COUNT 3
static bool sgm4154x_charger_is_charge_tapered(void *data, int tapered_ma)
{
	bool is_tapered = false;
	struct sgm_mmi_charger *chg = data;

	if (abs(chg->batt_info.batt_ma) <= tapered_ma) {
		if (chg->chrg_taper_cnt >= TAPER_COUNT) {
			is_tapered = true;
			chg->chrg_taper_cnt = 0;
		} else
			chg->chrg_taper_cnt++;
	} else
		chg->chrg_taper_cnt = 0;

	return is_tapered;
}

static bool sgm4154x_charger_is_charge_halt(void *data)
{
	int rc = 0;
	int chrg_stat = 0;
	bool chrg_halt = false;
	struct sgm_mmi_charger *chg = data;

	rc = regmap_read(chg->sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
	if (rc){
		pr_err("read SGM4154x_CHRG_STAT fail, rc=%d\n", rc);
		return chrg_halt;
	}

	chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	switch(chrg_stat) {
	case SGM4154x_NOT_CHRGING:
	case SGM4154x_TERM_CHRG:
		chrg_halt = true;
		break;
	case SGM4154x_PRECHRG:
	case SGM4154x_FAST_CHRG:
	default:
		break;
	}

	if (chg->batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
	    chg->batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		chrg_halt = true;

	return chrg_halt;
}

static void sgm4154x_charger_set_constraint(void *data,
                        struct mmi_charger_constraint *constraint)
{
	struct sgm_mmi_charger *chg = data;

	if (constraint->demo_mode != chg->constraint.demo_mode) {
		chg->constraint.demo_mode = constraint->demo_mode;
	}

	if (constraint->factory_version != chg->constraint.factory_version) {
		chg->constraint.factory_version = constraint->factory_version;
	}

	if (constraint->factory_mode != chg->constraint.factory_mode) {
		chg->constraint.factory_mode = constraint->factory_mode;
	}

	if (constraint->dcp_pmax != chg->constraint.dcp_pmax) {
		chg->constraint.dcp_pmax = constraint->dcp_pmax;
	}

	if (constraint->hvdcp_pmax != chg->constraint.hvdcp_pmax) {
		chg->constraint.hvdcp_pmax = constraint->hvdcp_pmax;
	}

	if (constraint->pd_pmax != chg->constraint.pd_pmax) {
		chg->constraint.pd_pmax = constraint->pd_pmax;
	}

	if (constraint->wls_pmax != chg->constraint.wls_pmax) {
		chg->constraint.wls_pmax = constraint->wls_pmax;
	}
}

/* Battery presence detection threshold on battery temperature */
#define BPD_TEMP_THRE (-30)
#define PANIC_DEB_TIME_MAX (60000)
#define OTG_VBUS_MIN_MV (4600)
#define OTG_VBUS_MAX_MV (5600)
static void sgm4154x_paired_battery_notify(void *data,
			struct mmi_battery_info *batt_info)
{
	int i;
	int rc;
	int batt_ocv;
	int paired_ocv;
	int delta_ocv;
	int delta_soc;
	bool batt_present;
	union power_supply_propval val = {0};
	struct sgm_mmi_charger *chg = data;
	int paired_ichg = chg->paired_ichg;
	int paired_load = chg->paired_load;
	struct timespec64 now = {0};
	static struct timespec64 start = {0};
	uint32_t elapsed_ms;

	if (!batt_info || batt_info->batt_mv <= 0) {
		pr_warn("Invalid paired battery info\n");
		return;
	}

	if (!chg->chg_info.chrg_present &&
	    chg->chg_info.chrg_ma == 0 &&
	    chg->chg_info.chrg_mv > OTG_VBUS_MIN_MV &&
	    chg->chg_info.chrg_mv < OTG_VBUS_MAX_MV &&
	    (chg->batt_info.batt_ma < 0 || batt_info->batt_ma < 0)) {
		chg->sgm->otg_enabled = true;
	} else {
		chg->sgm->otg_enabled = false;
	}

	memcpy(&chg->paired_batt_info, batt_info, sizeof(struct mmi_battery_info));
	batt_present = (chg->batt_info.batt_temp >= BPD_TEMP_THRE)? true : false;
	if (batt_present) {
		rc = power_supply_get_property(chg->fg_psy,
				POWER_SUPPLY_PROP_PRESENT, &val);
		if (!rc && !val.intval)
			batt_present = false;
	}
	if (!batt_present) {
		chg->paired_ichg = 0;
		chg->paired_load = PAIRED_LOAD_OFF;
		return;
	} else if (!chg->sgm->batt_present) {
		chg->paired_ichg = chg->sgm->init_data.max_ichg;
		chg->sgm->batt_present = batt_present;
		pr_info("Battery present!\n");
	}

	batt_ocv = chg->batt_info.batt_mv;
	batt_ocv -= (chg->batt_info.batt_ma * chg->batt_esr) / 1000;
	paired_ocv = batt_info->batt_mv;
	paired_ocv -= (batt_info->batt_ma * chg->paired_esr) / 1000;

	delta_ocv = batt_ocv - paired_ocv;
	delta_soc = chg->batt_info.batt_soc - batt_info->batt_soc;

	if (chg->paired_ichg_table && chg->paired_ichg_table_len) {
		paired_ichg = 0;
		for (i = 0; i < chg->paired_ichg_table_len; i++) {
			if (delta_ocv <= chg->paired_ichg_table[i * 2]) {
				paired_ichg = chg->paired_ichg_table[i * 2 + 1];
				break;
			}
		}
		paired_ichg *= 1000;
		if (!chg->constraint.factory_mode &&
		    chg->paired_ichg != paired_ichg) {
			chg->paired_ichg = paired_ichg;
			chg->chg_cfg.target_fcc = -EINVAL;
		}
	}

	if (chg->paired_load_thres && chg->paired_load_thres_len) {
		paired_load = PAIRED_LOAD_HIGH;
		for (i = 0; i < chg->paired_load_thres_len; i++) {
			if (delta_ocv > chg->paired_load_thres[i]) {
				paired_load = i;
				break;
			}
		}
		if (paired_load == PAIRED_LOAD_OFF) {
			if (start.tv_sec == 0 && start.tv_nsec == 0) {
				ktime_get_real_ts64(&start);
				elapsed_ms = 0;
			} else {
				ktime_get_real_ts64(&now);
				if (now.tv_sec <= start.tv_sec)
					start = now;
				elapsed_ms = (now.tv_sec - start.tv_sec) * 1000;
				elapsed_ms += (now.tv_nsec - start.tv_nsec) / 1000000;
			}
			if (paired_vbat_panic_enabled &&
			    elapsed_ms >= PANIC_DEB_TIME_MAX) {
				panic("ERROR: delta_ocv=%dmV, delta_soc=%d",
						delta_ocv, delta_soc);
			} else if (elapsed_ms >= PANIC_DEB_TIME_MAX) {
				pr_err("ERROR: delta_ocv=%dmV, delta_soc=%d",
						delta_ocv, delta_soc);
			}
		} else {
			start.tv_sec = 0;
			start.tv_nsec = 0;
		}
		chg->paired_load = paired_load;
	}

	 /* Turn off load switch to stop charging by paired battery */
	if (delta_ocv < 0 ||
	    (chg->chg_info.chrg_present &&
	     chg->chg_info.chrg_ma > 0 &&
	     batt_info->batt_ma >= 0)) {
		chg->paired_load = PAIRED_LOAD_OFF;
	}

	pr_info("paired_ocv:%dmV, batt_ocv:%dmV, delta_ocv:%dmV, delta_soc:%d\n",
				paired_ocv, batt_ocv, delta_ocv, delta_soc);
}

static int sgm4154x_mmi_charger_init(struct sgm_mmi_charger *chg)
{
	int rc;
	int i;
	int byte_len;
	const char *df_sn = NULL;
	struct mmi_charger_driver *driver;

	if (!chg->fg_psy && !chg->fg_psy_name) {
		chg->fg_psy_name = "fg_battery";
		device_property_read_string(chg->sgm->dev,
					"mmi,fg-psy-name",
					&chg->fg_psy_name);
	}

	if (!chg->fg_psy) {
		chg->fg_psy = power_supply_get_by_name(chg->fg_psy_name);
		if (!chg->fg_psy) {
			pr_err("No %s power supply found\n", chg->fg_psy_name);
			return -ENODEV;
		}
		pr_info("%s power supply is found\n", chg->fg_psy_name);
	}

	if (chg->driver) {
		pr_info("sgm4154x_mmi_charger has already registered\n");
		return 0;
	}

	rc = device_property_read_string(chg->sgm->dev,
					"mmi,df-serialnum",
					&df_sn);
	if (!rc && df_sn) {
		pr_info("Default Serial Number %s\n", df_sn);
	} else {
		pr_err("No Default Serial Number defined\n");
		df_sn = "unknown-sn";
	}
	strcpy(chg->batt_info.batt_sn, df_sn);
	chg->paired_batt_info.batt_soc = -EINVAL;
	chg->chg_cfg.target_fcc = chg->sgm->init_data.ichg / 1000;
	chg->chg_cfg.charging_disable = chg->sgm->init_data.charger_disabled;
	chg->chg_cfg.charger_suspend = chg->sgm->init_data.charger_disabled;

	if (of_property_read_bool(chg->sgm->dev->of_node, "mmi,ichg-invert-polority"))
		chg->ichg_polority = -1;
	else
		chg->ichg_polority = 1;
	chg->paired_ichg = chg->sgm->init_data.max_ichg;
	chg->paired_load = PAIRED_LOAD_OFF;
	if (!chg->paired_ichg_table &&
	    of_find_property(chg->sgm->dev->of_node, "mmi,paired-ichg-table", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 2) {
			pr_err("DT error wrong paired-ichg-table\n");
			return -ENODEV;
		}

		chg->paired_ichg_table = (int *)devm_kzalloc(chg->sgm->dev, byte_len, GFP_KERNEL);
		if (chg->paired_ichg_table == NULL)
			return -ENOMEM;

		chg->paired_ichg_table_len = byte_len / sizeof(u32) / 2;
		rc = of_property_read_u32_array(chg->sgm->dev->of_node,
				"mmi,paired-ichg-table",
				(u32 *)chg->paired_ichg_table,
				byte_len / sizeof(u32));
		if (rc < 0) {
			pr_err("Couldn't read paired-ichg-table rc = %d\n", rc);
			devm_kfree(chg->sgm->dev, chg->paired_ichg_table);
			chg->paired_ichg_table = NULL;
			return rc;
		}

		pr_info("paired-ichg-table: len: %d\n", chg->paired_ichg_table_len);
		for (i = 0; i < chg->paired_ichg_table_len; i++) {
			pr_info("paired-ichg-table[%d]: delta_soc %d, fcc %dmA\n", i,
				chg->paired_ichg_table[i * 2],
				chg->paired_ichg_table[i * 2 + 1]);
		}
	}

	if (!chg->paired_load_thres &&
	    of_find_property(chg->sgm->dev->of_node, "mmi,paired-load-thres", &byte_len)) {
		chg->paired_load_thres = (int *)devm_kzalloc(chg->sgm->dev, byte_len, GFP_KERNEL);
		if (chg->paired_load_thres == NULL)
			return -ENOMEM;

		chg->paired_load_thres_len = byte_len / sizeof(u32);
		rc = of_property_read_u32_array(chg->sgm->dev->of_node,
				"mmi,paired-load-thres",
				(u32 *)chg->paired_load_thres,
				byte_len / sizeof(u32));
		if (rc < 0) {
			pr_err("Couldn't read paired-load-thres rc = %d\n", rc);
			devm_kfree(chg->sgm->dev, chg->paired_load_thres);
			chg->paired_load_thres = NULL;
			return rc;
		}

		pr_info("paired-load-thres: len: %d\n", chg->paired_load_thres_len);
		for (i = 0; i < chg->paired_load_thres_len; i++) {
			pr_info("paired-load-thres[%d]: %d\n",
				i, chg->paired_load_thres[i]);
		}
	}

	rc = of_property_read_u32(chg->sgm->dev->of_node,
				"mmi,paired-esr",
				&chg->paired_esr);
	if (rc)
		chg->paired_esr = 60;
	pr_info("paired battery ESR: %d\n", chg->paired_esr);

	rc = of_property_read_u32(chg->sgm->dev->of_node,
				"mmi,batt-esr",
				&chg->batt_esr);
	if (rc)
		chg->batt_esr = 40;
	pr_info("battery ESR: %d\n", chg->batt_esr);

	driver = devm_kzalloc(chg->sgm->dev,
				sizeof(struct mmi_charger_driver),
				GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	/* init driver */
	driver->name = SGM4154x_NAME;
	driver->dev = chg->sgm->dev;
	driver->data = chg;
	driver->get_batt_info = sgm4154x_charger_get_batt_info;
	driver->get_chg_info = sgm4154x_charger_get_chg_info;
	driver->config_charge = sgm4154x_charger_config_charge;
	driver->is_charge_tapered = sgm4154x_charger_is_charge_tapered;
	driver->is_charge_halt = sgm4154x_charger_is_charge_halt;
	driver->set_constraint = sgm4154x_charger_set_constraint;
	driver->notify_paired_battery = sgm4154x_paired_battery_notify;
	chg->driver = driver;

	/* register driver to mmi charger */
	rc = mmi_register_charger_driver(driver);
	if (rc) {
		pr_err("sgm4154x_mmi_charger init failed, rc=%d\n", rc);
	} else {
		pr_info("sgm4154x_mmi_charger init successfully\n");
	}

	return rc;
}

static void sgm4154x_mmi_charger_deinit(struct sgm_mmi_charger *chg)
{
	int rc;

	if (!chg->driver) {
		pr_info("sgm4154x_mmi_charger has not inited yet\n");
		return;
	}

	/* unregister driver from mmi charger */
	rc = mmi_unregister_charger_driver(chg->driver);
	if (rc) {
		pr_err("sgm4154x_mmi_charger deinit failed, rc=%d\n", rc);
	} else {
		devm_kfree(chg->sgm->dev, chg->driver);
		chg->driver = NULL;
	}

	if (chg->fg_psy) {
		power_supply_put(chg->fg_psy);
		chg->fg_psy = NULL;
	}
}

static int sgm4154x_psy_notifier_call(struct notifier_block *nb,
				unsigned long val, void *v)
{
	struct sgm4154x_device *sgm = container_of(nb,
				struct sgm4154x_device, psy_nb);
	struct power_supply *psy = v;
	struct sgm_mmi_charger *chg = NULL;

	if (!sgm) {
		pr_err("called before sgm valid!\n");
		return NOTIFY_DONE;
	}

	if (!psy || val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	chg = sgm->mmi_charger;
	if ((sgm->usb && !strcmp(psy->desc->name, sgm->usb->desc->name)) ||
	    (chg && chg->fg_psy_name && !strcmp(psy->desc->name, chg->fg_psy_name)) ||
		(sgm->wls_psy && !strcmp(psy->desc->name, sgm->wls_psy->desc->name))) {
		cancel_delayed_work(&sgm->charge_detect_delayed_work);
		schedule_delayed_work(&sgm->charge_detect_delayed_work,
						msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

static int sgm4154x_show_registers(struct seq_file *m, void *data)
{
	int i;
	int ret;
	u32 reg = 0;
	struct sgm4154x_device *sgm = m->private;

	for (i = 0; i <= SGM4154x_CHRG_CTRL_f; i++) {
		ret = regmap_read(sgm->regmap, i, &reg);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%02X\n", i, reg);
	}

	return 0;
}

static int sgm4154x_reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct sgm4154x_device *sgm = inode->i_private;

	return single_open(file, sgm4154x_show_registers, sgm);
}

static const struct file_operations sgm4154x_reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= sgm4154x_reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct sgm4154x_device *sgm)
{
	sgm->debug_root = debugfs_create_dir("sgm4154x", NULL);
	if (!sgm->debug_root) {
		pr_err("Failed to create debug dir\n");
		return;
	}

	debugfs_create_file("registers", S_IFREG | S_IRUGO,
				sgm->debug_root, sgm,
				&sgm4154x_reg_debugfs_ops);
}

static ssize_t chg_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int enable;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 0, &enable);
	if (ret) {
		pr_err("Invalid chg_en value, ret = %d\n", ret);
		return ret;
	}

	if (enable < 0) {
		sgm->update_pending = true;
		sgm->user_chg_en = -EINVAL;
		pr_info("Clear user enable charging setting\n");
	} else {
		sgm->user_chg_en = !!enable;
		pr_info("Set user enable charging: %d\n", sgm->user_chg_en);
		if (sgm->user_chg_en)
			sgm4154x_enable_charger(sgm);
		else
			sgm4154x_disable_charger(sgm);
	}
	cancel_delayed_work(&sgm->charge_monitor_work);
	schedule_delayed_work(&sgm->charge_monitor_work,
					msecs_to_jiffies(200));

	return count;
}

static ssize_t chg_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	bool enable;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	enable = sgm4154x_is_enabled_charging(sgm);

	return sprintf(buf, "%d\n", enable);
}
DEVICE_ATTR_RW(chg_en);

static ssize_t high_load_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	bool enable;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtobool(buf, &enable);
	if (ret) {
		pr_err("Invalid high_load_en value, ret = %d\n", ret);
		return ret;
	}

	if (gpio_is_valid(sgm->high_load_en_gpio)) {
		gpio_set_value(sgm->high_load_en_gpio,
				enable ^ sgm->high_load_active_low);
		cancel_delayed_work(&sgm->charge_monitor_work);
		schedule_delayed_work(&sgm->charge_monitor_work,
						msecs_to_jiffies(200));
	}

	return count;
}

static ssize_t high_load_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	bool enable;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	if (gpio_is_valid(sgm->high_load_en_gpio))
		enable = sgm->high_load_active_low
			^ gpio_get_value(sgm->high_load_en_gpio);
	else
		enable = false;

	return sprintf(buf, "%d\n", enable);
}
DEVICE_ATTR_RW(high_load_en);

static ssize_t low_load_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	bool enable;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtobool(buf, &enable);
	if (ret) {
		pr_err("Invalid low_load_en value, ret = %d\n", ret);
		return ret;
	}

	if (gpio_is_valid(sgm->low_load_en_gpio)) {
		gpio_set_value(sgm->low_load_en_gpio,
				enable ^ sgm->low_load_active_low);
		cancel_delayed_work(&sgm->charge_monitor_work);
		schedule_delayed_work(&sgm->charge_monitor_work,
						msecs_to_jiffies(200));
	}

	return count;
}

static ssize_t low_load_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	bool enable;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	if (gpio_is_valid(sgm->low_load_en_gpio))
		enable = gpio_get_value(sgm->low_load_en_gpio)
			^ sgm->low_load_active_low;
	else
		enable = false;

	return sprintf(buf, "%d\n", enable);
}
DEVICE_ATTR_RW(low_load_en);

static ssize_t charger_suspend_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int chg_suspend;
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 0, &chg_suspend);
	if (ret) {
		pr_err("Invalid chg_suspend value, ret = %d\n", ret);
		return ret;
	}

	if (chg_suspend < 0) {
		sgm->update_pending = true;
		sgm->user_chg_susp = -EINVAL;
		pr_info("Clear user suspend charger setting\n");
	} else {
		sgm->user_chg_susp = !!chg_suspend;
		pr_info("Set user suspend charger: %d\n", sgm->user_chg_susp);
		sgm4154x_set_hiz_en(sgm, sgm->user_chg_susp);
	}
	cancel_delayed_work(&sgm->charge_monitor_work);
	schedule_delayed_work(&sgm->charge_monitor_work,
					msecs_to_jiffies(200));

	return count;
}

static ssize_t charger_suspend_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(dev);

	if (!sgm) {
		pr_err("sgm chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sgm->state.hiz_en);
}
DEVICE_ATTR_RW(charger_suspend);

static int sgm4154x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sgm4154x_device *sgm;
	int ret;
	int otg_notify;
	char *name = NULL;

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->client = client;
	sgm->dev = dev;
	sgm->user_ilim = -EINVAL;
	sgm->user_ichg = -EINVAL;
	sgm->user_chg_en = -EINVAL;
	sgm->user_chg_susp = -EINVAL;

	mutex_init(&sgm->lock);

	strncpy(sgm->model_name, id->name, I2C_NAME_SIZE);

	sgm->regmap = devm_regmap_init_i2c(client, &sgm4154x_regmap_config);
	if (IS_ERR(sgm->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(sgm->regmap);
	}

	i2c_set_clientdata(client, sgm);

	// Customer customization
	ret = sgm4154x_parse_dt(sgm);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		return ret;
	}

	if (IS_ERR_OR_NULL(sgm->vdd_i2c_vreg)) {
		dev_err(dev, "Could not get vdd-i2c power regulator\n");
	} else {
		ret = regulator_enable(sgm->vdd_i2c_vreg);
		if (ret)
			dev_err(dev, "Could not enable vdd-i2c power regulator\n");
	}

	ret = sgm4154x_hw_chipid_detect(sgm);
	if ((ret & SGM4154x_PN_MASK) != SGM4154x_PN_41542_ID){
		pr_info("device not found !!!\n");
		return ret;
	}

	name = devm_kasprintf(sgm->dev, GFP_KERNEL, "%s",
		"sgm4154x suspend wakelock");
	sgm->charger_wakelock =
		wakeup_source_register(NULL, name);

	/* OTG reporting */
	sgm->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(sgm->usb2_phy)) {
		INIT_WORK(&sgm->usb_work, sgm4154x_usb_work);
		sgm->usb_nb.notifier_call = sgm4154x_usb_notifier;
		otg_notify = usb_register_notifier(sgm->usb2_phy, &sgm->usb_nb);
	}

	sgm->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	if (!IS_ERR_OR_NULL(sgm->usb3_phy)) {
		INIT_WORK(&sgm->usb_work, sgm4154x_usb_work);
		sgm->usb_nb.notifier_call = sgm4154x_usb_notifier;
		otg_notify = usb_register_notifier(sgm->usb3_phy, &sgm->usb_nb);
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sgm4154x_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), sgm);
		if (ret)
			goto error_out;
		enable_irq_wake(client->irq);
		sgm->irq_enabled = true;
	}

	INIT_DELAYED_WORK(&sgm->charge_detect_delayed_work, charger_detect_work_func);
	INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);
	sgm->pm_nb.notifier_call = sgm4154x_suspend_notifier;
	register_pm_notifier(&sgm->pm_nb);
	sgm->psy_nb.notifier_call = sgm4154x_psy_notifier_call;
	ret = power_supply_reg_notifier(&sgm->psy_nb);
	if (ret)
		pr_err("Failed to reg power supply notifier: %d\n", ret);

	ret = sgm4154x_power_supply_init(sgm, dev);
	if (ret) {
		dev_err(dev, "Failed to register power supply, ret=%d\n", ret);
		goto error_out;
	}

	ret = sgm4154x_hw_init(sgm);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto error_out;
	}

	//OTG setting
	sgm4154x_set_otg_voltage(sgm, 5000000); //5V
	sgm4154x_set_otg_current(sgm, 1200000); //1.2A

	ret = sgm4154x_vbus_regulator_register(sgm);

	schedule_delayed_work(&sgm->charge_monitor_work,100);

	sgm->mmi_charger = devm_kzalloc(dev, sizeof(*sgm->mmi_charger), GFP_KERNEL);
	if (sgm->mmi_charger) {
		sgm->mmi_charger->sgm = sgm;
		sgm4154x_mmi_charger_init(sgm->mmi_charger);
	}

	ret = device_create_file(sgm->dev, &dev_attr_charger_suspend);
	if (ret)
		pr_err("Couldn't create charger_suspend\n");

	ret = device_create_file(sgm->dev, &dev_attr_chg_en);
	if (ret)
		pr_err("Couldn't create chg_en\n");

	ret = device_create_file(sgm->dev, &dev_attr_high_load_en);
	if (ret)
		pr_err("Couldn't create high_load_en\n");

	ret = device_create_file(sgm->dev, &dev_attr_low_load_en);
	if (ret)
		pr_err("Couldn't create low_load_en\n");

	create_debugfs_entry(sgm);

	dev_info(dev, "%s probe successfully.\n", SGM4154x_NAME);
	return 0;
error_out:
	if (!IS_ERR_OR_NULL(sgm->usb2_phy))
		usb_unregister_notifier(sgm->usb2_phy, &sgm->usb_nb);

	if (!IS_ERR_OR_NULL(sgm->usb3_phy))
		usb_unregister_notifier(sgm->usb3_phy, &sgm->usb_nb);
	return ret;
}

static int sgm4154x_charger_remove(struct i2c_client *client)
{
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);

	device_remove_file(sgm->dev, &dev_attr_high_load_en);
	device_remove_file(sgm->dev, &dev_attr_low_load_en);
	device_remove_file(sgm->dev, &dev_attr_chg_en);
	device_remove_file(sgm->dev, &dev_attr_charger_suspend);
	debugfs_remove_recursive(sgm->debug_root);

	cancel_delayed_work_sync(&sgm->charge_monitor_work);

	if (sgm->mmi_charger)
		sgm4154x_mmi_charger_deinit(sgm->mmi_charger);
	if (sgm->usb) {
		power_supply_put(sgm->usb);
		sgm->usb = NULL;
	}

	if (!IS_ERR_OR_NULL(sgm->usb2_phy))
		usb_unregister_notifier(sgm->usb2_phy, &sgm->usb_nb);

	if (!IS_ERR_OR_NULL(sgm->usb3_phy))
		usb_unregister_notifier(sgm->usb3_phy, &sgm->usb_nb);

	power_supply_unreg_notifier(&sgm->psy_nb);
	regulator_unregister(sgm->otg_rdev);

	power_supply_unregister(sgm->charger);

	if (!IS_ERR_OR_NULL(sgm->vdd_i2c_vreg)) {
		if (regulator_is_enabled(sgm->vdd_i2c_vreg))
			regulator_disable(sgm->vdd_i2c_vreg);
		devm_regulator_put(sgm->vdd_i2c_vreg);
	}

	if (gpio_is_valid(sgm->high_load_en_gpio))
		gpio_free(sgm->high_load_en_gpio);
	if (gpio_is_valid(sgm->low_load_en_gpio))
		gpio_free(sgm->low_load_en_gpio);
	if (gpio_is_valid(sgm->chg_en_gpio))
		gpio_free(sgm->chg_en_gpio);

	mutex_destroy(&sgm->lock);

	return 0;
}

static void sgm4154x_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);
	ret = sgm4154x_disable_charger(sgm);
	if (ret) {
		pr_err("Failed to disable charger, ret = %d\n", ret);
	}
	pr_info("sgm4154x_charger_shutdown\n");
}

static const struct i2c_device_id sgm4154x_i2c_ids[] = {
	{ "sgm41541", 0 },
	{ "sgm41542", 0 },
	{ "sgm41516", 0 },
	{ "sgm41516D", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm4154x_i2c_ids);

static const struct of_device_id sgm4154x_of_match[] = {
	{ .compatible = "sgm,sgm41541" },
	{ .compatible = "sgm,sgm41542" },
	{ .compatible = "sgm,sgm41516" },
	{ .compatible = "sgm,sgm41516D" },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm4154x_of_match);

static struct i2c_driver sgm4154x_driver = {
	.driver = {
		.name = "sgm4154x-charger",
		.of_match_table = sgm4154x_of_match,
	},
	.probe = sgm4154x_probe,
	.remove = sgm4154x_charger_remove,
	.shutdown = sgm4154x_charger_shutdown,
	.id_table = sgm4154x_i2c_ids,
};
module_i2c_driver(sgm4154x_driver);

MODULE_AUTHOR(" qhq <Allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("sgm4154x charger driver");
MODULE_LICENSE("GPL v2");
