// SPDX-License-Identifier: GPL-2.0
// SGM4154x driver version 2021-08-05-002
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.sg-micro.com

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

#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "sgm4154x_charger.h"
#include <linux/mmi_discrete_charger_class.h>
#include <linux/seq_file.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>


static struct power_supply_desc sgm4154x_power_supply_desc;

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

static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
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
/*mahj:for build
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
*/
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

	reg_val = chrg_curr / SGM4154x_ICHRG_CURRENT_STEP_uA;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2,
				  SGM4154x_ICHRG_CUR_MASK, reg_val);

	return ret;
}

// fine tuning termination voltage,to Improve accuracy
static int sgm4154x_vreg_fine_tuning(struct sgm4154x_device *sgm, enum SGM4154x_VREG_FT ft)
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
	pr_info("%s reg_val:%d\n",__func__,reg_val);

	return ret;
}

static int sgm4154x_set_chrg_volt(struct sgm4154x_device *sgm, int chrg_volt)
{
	int ret;
	int reg_val;
	enum SGM4154x_VREG_FT ft = VREG_FT_DISABLE;

	if (chrg_volt < SGM4154x_VREG_V_MIN_uV)
		chrg_volt = SGM4154x_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;


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
		pr_err("%s can't set vreg fine tunning ret=%d\n", __func__, ret);
		return ret;
	}

	reg_val = reg_val<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VREG_V_MASK, reg_val);

	return ret;
}
/*mahj:for build
static int sgm4154x_get_chrg_volt(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int vreg_val = 0;
	int chrg_volt = 0;

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
*/
//#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
static int sgm4154x_read_usbin_voltage_chan(struct sgm4154x_device *sgm, int *val)
{
	int rc;

	if (!sgm->iio.usbin_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(sgm->iio.usbin_v_chan, val);
	if (rc < 0) {
		dev_err(sgm->dev, "Couldn't read USBIN channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int sgm4154x_get_usb_voltage_now(struct sgm4154x_device *sgm, int *val)
{
	int rc;
	int raw_date;

	rc = sgm4154x_read_usbin_voltage_chan(sgm, &raw_date);
	if (rc < 0) {
		dev_err(sgm->dev, "Couldn't read USBIN over vadc rc=%d\n", rc);
		return rc;
	}

	*val = raw_date * 3;

	return 0;
}

static int sgm4154x_adjust_qc20_hvdcp_5v(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;

	/* dp 0.6v and dm 0v out 5V */
	dm_val = 0x1<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0v
	if (ret)
		return ret;

	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	return ret;
}

static int sgm4154x_detected_qc20_hvdcp(struct sgm4154x_device *sgm, int *charger_type)
{
	int ret;
	int dp_val, dm_val;
	int vbus_voltage;

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

	msleep(1300);

	dm_val = 0x1<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	if (ret)
		return ret;

	mdelay(500);

	/* dp 3.3v and dm 0.6v out 9V */
	dp_val = SGM4154x_DP_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 3.3v
	if (ret)
		return ret;

	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6v
	if (ret)
		return ret;

	msleep(200);//need tunning

	sgm4154x_get_usb_voltage_now(sgm, &vbus_voltage);
	dev_info(sgm->dev, "vbus voltage now = %d\n", vbus_voltage);

	if (vbus_voltage > MMI_HVDCP2_VOLTAGE_STANDARD) {
		dev_info(sgm->dev, "QC20 charger detected\n");
		*charger_type = POWER_SUPPLY_TYPE_USB_HVDCP;
	} else {
		dev_info(sgm->dev, "charger type is not HVDCP\n");
	}

	ret = sgm4154x_adjust_qc20_hvdcp_5v(sgm);
	if (ret) {
		dev_err(sgm->dev, "Cann't adjust qc20 hvdcp 5V\n");
	}

	msleep(300);//need tunning

	sgm4154x_get_usb_voltage_now(sgm, &vbus_voltage);
	dev_info(sgm->dev, "vbus voltage now = %d after qc20 detected\n", vbus_voltage);

	return ret;
}

#if 0 //mahj:for build
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
#endif

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

	udelay(2500);
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;

	udelay(2500);
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

	udelay(2500);
	dm_val = SGM4154x_DM_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 3.3v
	udelay(2500);

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
/*mahj:for build
static int sgm4154x_set_vindpm_offset_os(struct sgm4154x_device *sgm,enum SGM4154x_VINDPM_OS offset_os)
{
	int ret;


	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_f,
				  SGM4154x_VINDPM_OS_MASK, offset_os);

	if (ret){
		pr_err("%s fail\n",__func__);
		return ret;
	}

	return ret;
}
*/
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

	if (iindpm < SGM4154x_IINDPM_I_MIN_uA)
		reg_val = 0;
	else if (iindpm >= SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1F;
	else if (iindpm >= SGM4154x_IINDPM_I_MIN_uA && iindpm <= 3100000)//default
		reg_val = (iindpm-SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
	else if (iindpm > 3100000 && iindpm < SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1E;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, reg_val);
	return ret;
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

static int sgm4154x_set_icl(struct charger_device *chg_dev, u32 uA)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_info("%s set icl curr = %d\n", __func__, uA);

	sgm->input_current_cache = uA;

	if (sgm->mmi_is_qc3_authen) {
		dev_info(sgm->dev, "hvdcp detecting now\n");
		return 0;
	}

	return sgm4154x_set_input_curr_lim(sgm, uA);
}

static int sgm4154x_get_icl(struct charger_device *chg_dev, u32 *uA)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	*uA = sgm4154x_get_input_curr_lim(sgm);

	pr_info("%s get icl curr = %d\n", __func__, *uA);

	return 0;
}

/*mahj:for build
static int sgm4154x_set_watchdog_timer(struct sgm4154x_device *sgm, int time)
{
	int ret;
	u8 reg_val = 0;

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

	if (is_rst)
		val = SGM4154x_WDT_RST_MASK;
	else
		val = 0;
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1,
				  SGM4154x_WDT_RST_MASK, val);
}
*/
static int sgm4154x_get_state(struct sgm4154x_device *sgm,
			     struct sgm4154x_state *state)
{
	int chrg_stat;
	int fault;
	int chrg_param_0,chrg_param_1,chrg_param_2;
	int ret;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
		if (ret){
			pr_err("%s read SGM4154x_CHRG_STAT fail\n",__func__);
			return ret;
		}
	}

	state->chrg_type = chrg_stat & SGM4154x_VBUS_STAT_MASK;
	state->chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	state->online = !!(chrg_stat & SGM4154x_PG_STAT);
	state->therm_stat = !!(chrg_stat & SGM4154x_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM4154x_VSYS_STAT);

	pr_err("%s chrg_stat =%d,chrg_type =%d online = %d\n",__func__,state->chrg_stat,state->chrg_type,state->online);


	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_FAULT, &fault);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_FAULT fail\n",__func__);
		return ret;
	}
	state->chrg_fault = fault;
	state->ntc_fault = fault & SGM4154x_TEMP_MASK;
	state->health = state->ntc_fault;
	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &chrg_param_0);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_0 fail\n",__func__);
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM4154x_HIZ_EN);

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_5, &chrg_param_1);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_5 fail\n",__func__);
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM4154x_TERM_EN);

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &chrg_param_2);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM4154x_VBUS_GOOD);

	return 0;
}

int sgm4154x_enable_charger(struct sgm4154x_device *sgm)
{
    int ret;
    printk("sgm4154x_enable_charger\n");
    ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_CHRG_EN,
                     SGM4154x_CHRG_EN);

    return ret;
}

int sgm4154x_disable_charger(struct sgm4154x_device *sgm)
{
    int ret;
    printk("sgm4154x_disable_charger\n");
    ret =
        regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_CHRG_EN,
                     0);
    return ret;
}

int sgm4154x_disable_watchdog(struct sgm4154x_device *sgm)
{
	int ret;
	printk("sgm4154x_disable_watchdog\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_5, SGM4154x_WDT_TIMER_MASK,
                     SGM4154x_WDT_TIMER_DISABLE);
	return ret;
}

int sgm4154x_enable_hw_jeita(struct charger_device *chg_dev, bool en)
{
	int rc = 0;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	rc = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d, SGM4154x_JEITA_ENABLE_MASK,
                     en ? SGM4154x_JEITA_ENABLE : SGM4154x_JEITA_DISABLE);

	pr_info("%s, %s hw jeita %s\n", __func__,
		en ? "enable" : "disable",
		rc ? "failed" : "success");

	return rc;
}

int sgm4154x_disable_vindpm_int_pulse(struct sgm4154x_device *sgm)
{
	int ret;
	printk("sgm4154x_disable_vindpm_pulse\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_a, SGM4154x_VINDPM_INT_MASK,
                     SGM4154x_VINDPM_INT_DISABLE);
	return ret;
}

int sgm4154x_disable_iindpm_int_pulse(struct sgm4154x_device *sgm)
{
	int ret;
	printk("sgm4154x_disable_iindpm_pulse\n");
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_a, SGM4154x_IINDPM_INT_MASK,
                     SGM4154x_IINDPM_INT_DISABLE);
	return ret;
}

/*mahj:for build
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
		pr_err("%s read SGM4154x_CHRG_CTRL_6 fail\n",__func__);
		return ret;
	}

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &ilim); //read default setting to save
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_0 fail\n",__func__);
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
			pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
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
			pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
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
*/
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

	if (ua >= BOOST_CURRENT_LIMIT[1]) {
		ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2,
			SGM4154X_BOOST_LIM_MASK, BIT(7));
	} else {
		ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2,
			SGM4154X_BOOST_LIM_MASK, 0);
	}
	return ret;
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
	pr_err("%s usb_type:%d\n",__func__,usb_type);
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
		ret = sgm4154x_set_icl(sgm->chg_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_set_input_volt_lim(sgm, val->intval);
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
		pr_err("%s read SGM4154x_CHRG_STAT fail\n",__func__);
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
		sgm4154x_get_usb_voltage_now(sgm, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//val->intval = state.ibus_adc;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_get_input_volt_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		ret = 0;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm4154x_get_input_curr_lim(sgm);
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


#if 0
static bool sgm4154x_state_changed(struct sgm4154x_device *sgm,
				  struct sgm4154x_state *new_state)
{
	struct sgm4154x_state old_state;

	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	return (old_state.chrg_status != new_state->chrg_status ||
		old_state.chrg_fault != new_state->chrg_fault	||
		old_state.online != new_state->online		||
		old_state.health != new_state->health	||
		old_state.chrg_type != new_state->chrg_type
		);
}
#endif
static void sgm4154x_dump_register(struct sgm4154x_device * sgm)
{
	int i = 0;
	u32 reg = 0;

	for(i=0; i<=SGM4154x_CHRG_CTRL_f; i++) {
		regmap_read(sgm->regmap, i, &reg);
		pr_err("%s REG[0x%x]=0x%x\n", __func__, i, reg);
	}
}

int sgm4154x_get_usb_present(struct sgm4154x_device *sgm)
{
	int ret;
	u32 stat;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &stat);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
		return ret;
	}
	sgm->state.vbus_gd = !!(stat & SGM4154x_VBUS_GOOD);
	return 0;
}

static int sgm4154x_request_dpdm(struct sgm4154x_device *sgm, bool enable)
{
	int rc = 0;

		/* fetch the DPDM regulator */
	if (!sgm->dpdm_reg && of_get_property(sgm->dev->of_node,
				"dpdm-supply", NULL)) {
		sgm->dpdm_reg = devm_regulator_get(sgm->dev, "dpdm");
		if (IS_ERR(sgm->dpdm_reg)) {
			rc = PTR_ERR(sgm->dpdm_reg);
			dev_err(sgm->dev, "Couldn't get dpdm regulator rc=%d\n", rc);
			sgm->dpdm_reg = NULL;
			return rc;
		}
	}

	mutex_lock(&sgm->dpdm_lock);
	if (enable) {
		if (sgm->dpdm_reg && !sgm->dpdm_enabled) {
			dev_err(sgm->dev, "enabling DPDM regulator\n");
			rc = regulator_enable(sgm->dpdm_reg);
			if (rc < 0)
				dev_err(sgm->dev,
					"Couldn't enable dpdm regulator rc=%d\n",
					rc);
			else
				sgm->dpdm_enabled = true;
		}
	} else {
		if (sgm->dpdm_reg && sgm->dpdm_enabled) {
			dev_err(sgm->dev, "disabling DPDM regulator\n");
			rc = regulator_disable(sgm->dpdm_reg);
			if (rc < 0)
				dev_err(sgm->dev,
					"Couldn't disable dpdm regulator rc=%d\n",
					rc);
			else
				sgm->dpdm_enabled = false;
		}
	}
	mutex_unlock(&sgm->dpdm_lock);

	return rc;
}

void sgm4154x_rerun_apsd(struct sgm4154x_device * sgm)
{
	int rc;

	dev_info(sgm->dev, "re-running APSD\n");

	rc = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_7, SGM4154x_IINDET_EN_MASK,
                     SGM4154x_IINDET_EN);
	if (rc < 0)
		dev_err(sgm->dev, "Couldn't re-run APSD rc=%d\n", rc);
	else
		sgm->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
}

static bool sgm4154x_is_rerun_apsd_done(struct sgm4154x_device * sgm)
{
	int rc = 0;
	int val = 0;
	bool result = false;
	rc = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_7, &val);
	if (rc)
		return false;

	result = !(val & SGM4154x_IINDET_EN_MASK);
	pr_info("%s:rerun apsd %s", __func__, result ? "done" : "not complete");
	return result;
}

static void sgm4154x_rerun_apsd_work_func(struct work_struct *work)
{
	struct sgm4154x_device * sgm = NULL;
	int rc = 0;
	bool apsd_done = false;
	int check_count = 0;

	sgm = container_of(work, struct sgm4154x_device, rerun_apsd_work);
	if(sgm == NULL) {
		pr_err("Cann't get sgm4154x_device\n");
		return;
	}

	rc = sgm4154x_get_usb_present(sgm);
	if (rc)
		return;

	if (!sgm->state.vbus_gd)
		return;

	rc = sgm4154x_request_dpdm(sgm, true);
	if (rc < 0)
		dev_err(sgm->dev, "Couldn't to enable DPDM rc=%d\n", rc);

	sgm->typec_apsd_rerun_done = true;
	sgm4154x_rerun_apsd(sgm);

	while(check_count < 10) {
		apsd_done = sgm4154x_is_rerun_apsd_done(sgm);
		if (apsd_done) {
			break;
		}

		msleep(100);
		check_count ++;
	}

	schedule_work(&sgm->charge_detect_work);
}

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


static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	struct sgm4154x_device * sgm = NULL;
	struct delayed_work *charge_monitor_work = NULL;
	//static u8 last_chg_method = 0;
	struct sgm4154x_state state;

	charge_monitor_work = container_of(work, struct delayed_work, work);
	if(charge_monitor_work == NULL) {
		pr_err("Cann't get charge_monitor_work\n");
		return ;
	}
	sgm = container_of(charge_monitor_work, struct sgm4154x_device, charge_monitor_work);
	if(sgm == NULL) {
		pr_err("Cann't get sgm \n");
		return ;
	}

	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if (!sgm->state.chrg_type) {
		pr_err("%s not present vbus_status \n",__func__);
		goto OUT;
	}

	sgm4154x_dump_register(sgm);
	pr_err("%s\n",__func__);
OUT:
	schedule_delayed_work(&sgm->charge_monitor_work, 10*HZ);
}

static void sgm4154x_vbus_remove(struct sgm4154x_device * sgm)
{
	dev_err(sgm->dev, "Vbus removed, disable charge\n");

	sgm->pulse_cnt = 0;
	sgm->typec_apsd_rerun_done = false;
	sgm->chg_dev->noti.apsd_done = false;
	sgm->chg_dev->noti.hvdcp_done = false;
	sgm->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	sgm4154x_request_dpdm(sgm, false);
}

static int sgm4154x_detected_qc30_hvdcp(struct sgm4154x_device *sgm, int *charger_type)
{
	int ret = 0;
	int dp_val, dm_val;
	int i=0, vbus_voltage;

	/* dp 0.6v and dm 3.3v entry QC3.0 mode */
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;

	dm_val = 0x3<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 3.3v
	if (ret)
		return ret;

	mdelay(100);

	for (i = 0; i < 16; i++) {
		ret = sgm4154x_qc30_step_up_vbus(sgm);
		if (ret)
			dev_err(sgm->dev, "%s qc30 step up vbus error\n", __func__);
	}

	mdelay(100);//need tunning

	sgm4154x_get_usb_voltage_now(sgm, &vbus_voltage);
	dev_info(sgm->dev, "%s vbus voltage now = %d in detected qc30\n", __func__,vbus_voltage);

	if (vbus_voltage > MMI_HVDCP3_VOLTAGE_STANDARD) {
		*charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3;
		dev_info(sgm->dev, "%s QC3.0 charger detected\n", __func__);
	}

	for (i = 0; i < 16; i++) {
		ret = sgm4154x_qc30_step_down_vbus(sgm);
		if (ret)
			dev_err(sgm->dev, "%s qc30 step down vbus error\n", __func__);
	}

	sgm4154x_get_usb_voltage_now(sgm, &vbus_voltage);
	dev_info(sgm->dev, "%s vbus voltage now = %d after detected qc30\n", __func__,vbus_voltage);

	return ret;
}

static int mmi_hvdcp_detect_kthread(void *param)
{
	struct sgm4154x_device * sgm = param;
	int ret;
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	struct sched_param sch_param = {.sched_priority = MAX_RT_PRIO-1};

	sched_setscheduler(current, SCHED_FIFO, &sch_param);

	do {

		wait_event_interruptible(sgm->mmi_qc3_wait_que, sgm->mmi_qc3_trig_flag || kthread_should_stop());
		if (kthread_should_stop())
			break;

		down(&sgm->sem_dpdm);
		dev_info(sgm->dev, "mmi_hvdcp_detect_kthread begin\n");
		sgm->mmi_qc3_trig_flag = false;
		sgm->mmi_is_qc3_authen = true;
		sgm4154x_set_input_curr_lim(sgm, MMI_HVDCP_DETECT_ICL_LIMIT);

		//do qc2.0 detected
		ret = sgm4154x_detected_qc20_hvdcp(sgm, &charger_type);
		if (ret) {
			dev_err(sgm->dev, "Cann't detected qc20 hvdcp\n");
			goto out;
		}

		if (charger_type != POWER_SUPPLY_TYPE_USB_HVDCP)
			goto out;

		//do qc3.0 detected
		ret = sgm4154x_detected_qc30_hvdcp(sgm, &charger_type);
		if (ret) {
			dev_err(sgm->dev, "Cann't detected qc30 hvdcp\n");
		}

		sgm4154x_get_usb_present(sgm);
		if (!sgm->state.vbus_gd)
			goto out;

		sgm->real_charger_type = charger_type;
		//notify charging policy to update charger type
		sgm->chg_dev->noti.hvdcp_done = true;
		charger_dev_notify(sgm->chg_dev);

	out:
		sgm4154x_set_input_curr_lim(sgm, sgm->input_current_cache);
		sgm->mmi_is_qc3_authen = false;
		up(&sgm->sem_dpdm);
		dev_info(sgm->dev, "mmi_hvdcp_detect_kthread end\n");
	}while(!kthread_should_stop());

	dev_dbg(sgm->dev, "qc3 kthread stop\n");
	return 0;
}

static void mmi_start_hvdcp_detect(struct sgm4154x_device *sgm)
{

	if (sgm->mmi_qc3_support
		&& sgm->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		dev_info(sgm->dev, "start hvdcp detect\n");
		sgm->mmi_qc3_trig_flag = true;
		wake_up_interruptible(&sgm->mmi_qc3_wait_que);
	}
}

static void charger_detect_work_func(struct work_struct *work)
{
	struct sgm4154x_device * sgm = NULL;
	struct sgm4154x_state state;
	int ret;

	sgm = container_of(work, struct sgm4154x_device, charge_detect_work);
	if(sgm == NULL) {
		pr_err("Cann't get sgm4154x_device\n");
		goto err;
	}

	if (!sgm->charger_wakelock->active)
		__pm_stay_awake(sgm->charger_wakelock);

	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if(!sgm->state.vbus_gd) {
		sgm4154x_vbus_remove(sgm);
		goto vbus_remove;
	}

	if (sgm->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		dev_err(sgm->dev, "BC1.2 have already detected\n");
		return;
	}

	if(!state.online)
	{
		sgm4154x_request_dpdm(sgm, true);
		dev_err(sgm->dev, "BC1.2 detecte not done\n");
		goto err;
	}

	if(!sgm4154x_dpdm_detect_is_done(sgm)) {
		dev_err(sgm->dev, "DPDM detecte not done, disable charge\n");
		goto err;
	}

#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
	if (((sgm->state.chrg_type == SGM4154x_USB_SDP) ||
		(sgm->state.chrg_type == SGM4154x_NON_STANDARD) ||
		(sgm->state.chrg_type == SGM4154x_UNKNOWN))
		&& (!sgm->typec_apsd_rerun_done)) {
		down(&sgm->sem_dpdm);
		dev_err(sgm->dev, "rerun apsd for 0x%x\n", sgm->state.chrg_type);
		schedule_work(&sgm->rerun_apsd_work);
		up(&sgm->sem_dpdm);
		goto err;
	}

	switch(sgm->state.chrg_type) {
		case SGM4154x_USB_SDP:
			pr_err("SGM4154x charger type: SDP\n");
			sgm->real_charger_type = POWER_SUPPLY_TYPE_USB;
			break;

		case SGM4154x_USB_CDP:
			pr_err("SGM4154x charger type: CDP\n");
			sgm->real_charger_type = POWER_SUPPLY_TYPE_USB_CDP;
			break;

		case SGM4154x_USB_DCP:
			pr_err("SGM4154x charger type: DCP\n");
			sgm->real_charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			mmi_start_hvdcp_detect(sgm);
			break;

		case SGM4154x_UNKNOWN:
		case SGM4154x_NON_STANDARD:
			pr_err("SGM4154x charger type: UNKNOWN\n");
			sgm->real_charger_type = POWER_SUPPLY_TYPE_USB_FLOAT;
			break;

		case SGM4154x_OTG_MODE:
			pr_err("SGM4154x OTG mode do nothing\n");
			goto err;

		default:
			pr_err("SGM4154x charger type: default\n");
			return;
	}

#endif

	sgm4154x_dump_register(sgm);
	//notify charging policy to update charger type
	sgm->chg_dev->noti.apsd_done = true;
	charger_dev_notify(sgm->chg_dev);
	return;

vbus_remove:
	charger_dev_notify(sgm->chg_dev);

err:
	//release wakelock
	dev_err(sgm->dev, "Relax wakelock\n");
	__pm_relax(sgm->charger_wakelock);
	return;
}

static irqreturn_t sgm4154x_irq_handler_thread(int irq, void *private)
{
	struct sgm4154x_device *sgm = private;

	//lock wakelock
	pr_err("%s entry\n",__func__);
	#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)
	schedule_work(&sgm->charge_detect_work);
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
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
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

static const struct regmap_config sgm4154x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SGM4154x_CHRG_CTRL_f,
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
		return -EINVAL;

	return 0;
}

static int sgm4154x_hw_init(struct sgm4154x_device *sgm)
{
	int ret = 0;
	struct power_supply_battery_info bat_info = { };

	bat_info.constant_charge_current_max_ua =
			SGM4154x_ICHRG_I_DEF_uA;

	bat_info.constant_charge_voltage_max_uv =
			SGM4154x_VREG_V_DEF_uV;

	bat_info.precharge_current_ua =
			SGM4154x_PRECHRG_I_DEF_uA;

	bat_info.charge_term_current_ua =
			SGM4154x_TERMCHRG_I_DEF_uA;

	sgm->init_data.max_ichg =
			SGM4154x_ICHRG_I_MAX_uA;

	sgm->init_data.max_vreg =
			SGM4154x_VREG_V_MAX_uV;

	//OTG setting
	sgm4154x_set_otg_voltage(sgm, 5000000); //5V
	sgm4154x_set_otg_current(sgm, 2000000); //2A

	ret = sgm4154x_set_ichrg_curr(sgm,
				bat_info.constant_charge_current_max_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_prechrg_curr(sgm, bat_info.precharge_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_chrg_volt(sgm,
				bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_term_curr(sgm, bat_info.charge_term_current_ua);
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

	ret = sgm4154x_set_recharge_volt(sgm, 200);//100~200mv
	if (ret)
		goto err_out;

	ret = sgm4154x_enable_hw_jeita(sgm->chg_dev, false);
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

	ret = sgm4154x_enable_charger(sgm);
	if (ret)
		goto err_out;

	dev_notice(sgm->dev, "ichrg_curr:%d prechrg_curr:%d chrg_vol:%d"
		" term_curr:%d input_curr_lim:%d",
		bat_info.constant_charge_current_max_ua,
		bat_info.precharge_current_ua,
		bat_info.constant_charge_voltage_max_uv,
		bat_info.charge_term_current_ua,
		sgm->init_data.ilim);

	return 0;

err_out:
	return ret;
}

static int sgm4154x_parse_dt(struct sgm4154x_device *sgm)
{
	int ret;
	u32 val = 0;
	int irq_gpio = 0, irqn = 0;
	int chg_en_gpio = 0;

	if (of_property_read_string(sgm->dev->of_node, "charger_name",
			&sgm->chg_dev_name) < 0) {
		sgm->chg_dev_name = "master_chg";
		pr_warn("no charger name\n");
	}

	sgm->mmi_qc3_support = of_property_read_bool(sgm->dev->of_node, "mmi,qc3-support");

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

	chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (gpio_is_valid(chg_en_gpio))
	{
		ret = gpio_request(chg_en_gpio, "sgm4154x chg en pin");
		if (ret) {
			dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, chg_en_gpio);
			return ret;
		}
		gpio_direction_output(chg_en_gpio,0);//default enable charge
	}
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
	pr_err("%s,enable_sw_jeita = %d,CV1 = %d,CV2 = %d,CV3 = %d,CV4 = %d,CV5 = %d,CV6 = %d\n",__func__,
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

	pr_err("%s,CC1 = %d,CC2 = %d,CC3 = %d,CC4 = %d,CC5 = %d\n",__func__,
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
	pr_err("%s,thres4 = %d,thres3 = %d,thres2 = %d,thres1 = %d,thres0 = %d\n",__func__,
			sgm->data.temp_t4_thres,sgm->data.temp_t3_thres,
			sgm->data.temp_t2_thres,sgm->data.temp_t1_thres,
			sgm->data.temp_t0_thres);
	return 0;
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
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
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
		pr_info("[%s] read SGM4154x_CHRG_CTRL_b fail\n", __func__);
		return ret;
	}

	pr_info("[%s] Reg[0x0B]=0x%x\n", __func__,val);

	return val;
}

static int sgm4154x_get_iio_channel(struct sgm4154x_device *sgm, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(sgm->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(sgm->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			dev_err(sgm->dev, "%s channel unavailable, %d\n",
							propname, rc);
		*chan = NULL;
	}

	return rc;
}

static int sgm4154x_parse_dt_adc_channels(struct sgm4154x_device *sgm)
{
	int rc = 0;

	rc = sgm4154x_get_iio_channel(sgm, "gpio3_div3",
					&sgm->iio.usbin_v_chan);
	if (rc < 0)
		return rc;

	return 0;
}

static int sgm4154x_enable_otg(struct charger_device *chg_dev, bool enable)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	int rc = 0;

	if (enable)
		rc = sgm4154x_enable_vbus(sgm->otg_rdev);
	else
		rc = sgm4154x_disable_vbus(sgm->otg_rdev);

	pr_info("%s, %s otg %s\n", __func__,
		enable ? "enable" : "disable",
		rc ? "failed" : "success");

	return rc;
}

static int sgm4154x_enable_charging(struct charger_device *chg_dev, bool enable)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	int rc = 0;

	rc = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_CHRG_EN,
                 enable ? SGM4154x_CHRG_EN : 0);

	pr_info("%s, %s charging %s\n", __func__,
		enable ? "enable" : "disable",
		rc ? "failed" : "success");

	return rc;
}

static int sgm4154x_set_charging_current(struct charger_device *chg_dev, u32 uA)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	int rc = 0;

	rc = sgm4154x_set_ichrg_curr(sgm, uA);

	pr_info("%s set charging curr = %d, %s\n", __func__, uA, rc ? "failed" : "success");

	return rc;
}

static int sgm4154x_set_charging_voltage(struct charger_device *chg_dev, u32 uV)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	int rc = 0;

	rc = sgm4154x_set_chrg_volt(sgm, uV);

	pr_info("%s set charging volt = %d, %s\n", __func__, uV, rc ? "failed" : "success");

	return rc;
}

static int sgm4154x_is_charging_halted(struct charger_device *chg_dev, bool *en)
{
	int rc = 0;
	int chrg_stat = 0;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	rc = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
	if (rc){
		pr_err("%s read SGM4154x_CHRG_STAT fail\n",__func__);
		return rc;
	}

	chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	switch(chrg_stat) {
	case SGM4154x_NOT_CHRGING:
	case SGM4154x_TERM_CHRG:
		*en = true;
		break;
	case SGM4154x_PRECHRG:
	case SGM4154x_FAST_CHRG:
		*en = false;
		break;
	default:
		break;
	}

	return rc;
}

static int sgm4154x_set_boost_current_limit(struct charger_device *chg_dev, u32 uA)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	int rc = 0;

	rc = sgm4154x_set_otg_current(sgm, uA);

	pr_info("%s set boost current limit = %d, %s\n", __func__, uA, rc ? "failed" : "success");

	return rc;
}

static int sgm4154x_get_pulse_cnt(struct charger_device *chg_dev, int *count)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	*count = sgm->pulse_cnt;

	return 0;
}

static int sgm4154x_set_dp_dm(struct charger_device *chg_dev, int val)
{
	int rc = 0;
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	switch(val) {
	case MMI_POWER_SUPPLY_DP_DM_DP_PULSE:
		sgm->pulse_cnt++;
		rc = sgm4154x_qc30_step_up_vbus(sgm);
		if (rc) {
			dev_err(sgm->dev, "Couldn't increase pulse count rc=%d\n",
				rc);
			sgm->pulse_cnt--;
		}
		dev_dbg(sgm->dev,"DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, sgm->pulse_cnt);
		break;
	case MMI_POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = sgm4154x_qc30_step_down_vbus(sgm);
		if (!rc && sgm->pulse_cnt)
			sgm->pulse_cnt--;
		dev_dbg(sgm->dev, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, sgm->pulse_cnt);
		break;
	default:
		break;
	}

	return rc;
}

static int sgm4154x_get_real_charger_type(struct charger_device *chg_dev, int *chg_type)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);

	*chg_type = sgm->real_charger_type;

	return 0;
}

static int sgm4154x_dump_registers(struct charger_device *chg_dev, struct seq_file *m)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	int i = 0;
	u32 reg = 0;
	int rc = 0;

	for(i=0; i<=SGM4154x_CHRG_CTRL_f; i++) {
		rc = regmap_read(sgm->regmap, i, &reg);
		if (rc)
			continue;
		seq_printf(m, "%s REG[0x%x]=0x%x\n", __func__, i, reg);
	}

	return rc;
}

static const struct charger_properties sgm4154x_chg_props = {
	.alias_name = "sgm4154x",
};

static struct charger_ops sgm4154x_chg_ops = {
	.get_pulse_cnt = sgm4154x_get_pulse_cnt,
	.set_dp_dm = sgm4154x_set_dp_dm,
	.get_real_charger_type = sgm4154x_get_real_charger_type,
	.set_input_current = sgm4154x_set_icl,
	.get_input_current = sgm4154x_get_icl,
	.enable_hw_jeita = sgm4154x_enable_hw_jeita,
	.enable_otg = sgm4154x_enable_otg,
	.set_boost_current_limit = sgm4154x_set_boost_current_limit,
	.enable_charging = sgm4154x_enable_charging,
	.set_charging_current = sgm4154x_set_charging_current,
	.set_constant_voltage = sgm4154x_set_charging_voltage,
	.is_charge_halted = sgm4154x_is_charging_halted,

	.dump_registers = sgm4154x_dump_registers,

};

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

	mutex_init(&sgm->lock);
	mutex_init(&sgm->dpdm_lock);

	strncpy(sgm->model_name, id->name, I2C_NAME_SIZE);

	sgm->regmap = devm_regmap_init_i2c(client, &sgm4154x_regmap_config);
	if (IS_ERR(sgm->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(sgm->regmap);
	}

	i2c_set_clientdata(client, sgm);

	ret = sgm4154x_hw_chipid_detect(sgm);
	if ((ret & SGM4154x_PN_MASK) != SGM4154x_PN_41542_ID){
		pr_info("[%s] device not found !!!\n", __func__);
		return ret;
	}

	sema_init(&sgm->sem_dpdm, 1);

	// Customer customization
	ret = sgm4154x_parse_dt(sgm);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		return ret;
	}

	ret = sgm4154x_parse_dt_adc_channels(sgm);
	if (ret) {
		dev_err(dev, "Failed to get adc channels%d\n", ret);
		return ret;
	}

	sgm->chg_dev = charger_device_register(sgm->chg_dev_name,
					      &client->dev, sgm,
					      &sgm4154x_chg_ops,
					      &sgm4154x_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}

	ret = sgm4154x_hw_init(sgm);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
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

	INIT_WORK(&sgm->charge_detect_work, charger_detect_work_func);
	INIT_WORK(&sgm->rerun_apsd_work, sgm4154x_rerun_apsd_work_func);
	INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);

	if (sgm->mmi_qc3_support) {
		sgm->mmi_qc3_authen_task = kthread_create(mmi_hvdcp_detect_kthread, sgm, "mmi_qc3_authen");
		if (IS_ERR(sgm->mmi_qc3_authen_task)) {
			ret = PTR_ERR(sgm->mmi_qc3_authen_task);
			dev_err(dev, "Failed to create mmi_qc3_authen_task ret = %d\n", ret);
			goto error_out;
		}
		init_waitqueue_head(&sgm->mmi_qc3_wait_que);
		wake_up_process(sgm->mmi_qc3_authen_task);
	}

	//rerun apsd and trigger charger detect when boot with charger
	schedule_work(&sgm->rerun_apsd_work);

	sgm->pm_nb.notifier_call = sgm4154x_suspend_notifier;
	register_pm_notifier(&sgm->pm_nb);

	ret = sgm4154x_power_supply_init(sgm, dev);
	if (ret) {
		dev_err(dev, "Failed to register power supply\n");
		goto error_out;
	}

	ret = sgm4154x_vbus_regulator_register(sgm);

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sgm4154x_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), sgm);
		if (ret)
			goto error_out;
		enable_irq_wake(client->irq);
	}

	schedule_delayed_work(&sgm->charge_monitor_work,100);

	dev_info(dev, "SGM4154x prob successfully.\n");
	return ret;
error_out:
	if (sgm->mmi_qc3_support && sgm->mmi_qc3_authen_task)
		kthread_stop(sgm->mmi_qc3_authen_task);

	if (!IS_ERR_OR_NULL(sgm->usb2_phy))
		usb_unregister_notifier(sgm->usb2_phy, &sgm->usb_nb);

	if (!IS_ERR_OR_NULL(sgm->usb3_phy))
		usb_unregister_notifier(sgm->usb3_phy, &sgm->usb_nb);
	return ret;
}

static int sgm4154x_charger_remove(struct i2c_client *client)
{
	struct sgm4154x_device *sgm= i2c_get_clientdata(client);

	if (sgm->mmi_qc3_support && sgm->mmi_qc3_authen_task)
		kthread_stop(sgm->mmi_qc3_authen_task);

	cancel_delayed_work_sync(&sgm->charge_monitor_work);

	regulator_unregister(sgm->otg_rdev);

	power_supply_unregister(sgm->charger);

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
