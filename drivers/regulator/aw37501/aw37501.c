/*
* aw37501.c
*
* Version: v0.1.2
*
* Copyright (c) 2021 AWINIC Technology CO., LTD
*
* Author: Alex <shiqiang@awinic.com>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include "aw37501.h"

#define AW37501_DRIVER_VERSION	"V0.1.2"

static int aw37501_write(struct aw37501_power *aw37501, u8 reg, u8 val)
{
	int ret = -1;
	unsigned char cnt = 0;

	pr_info("%s: reg[%x] = %x\n", __func__, reg, val);
	while (cnt < AW_I2C_RETRIES) {
		ret =
		i2c_smbus_write_byte_data(aw37501->client, reg, val);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int aw37501_read(struct aw37501_power *aw37501, u8 reg, u8 *val)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw37501->client, reg);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			*val = ret;
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}
	return ret;
}

static int aw37501_soft_reset(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_OPEN_CMD);
	if (ret < 0)
		return ret;

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = reg_val | AW37501_RESET;
	ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
	if (ret < 0)
		return ret;

	ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_CLOSE_CMD);
	if (ret < 0)
		return ret;

	return 0;
}

static int aw37501_disn_enable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = reg_val | AW37501_DISN_ENABLE;
	ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
	if (ret < 0)
		return ret;
	return 0;
}

static int aw37501_disn_disable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = reg_val & AW37501_DISN_DISABLE;
	ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
	if (ret < 0)
		return ret;
	return 0;
}
static int aw37501_disp_enable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = reg_val | AW37501_DISP_ENABLE;
	ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
	if (ret < 0)
		return ret;
	return 0;
}

static int aw37501_disp_disable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = reg_val & AW37501_DISP_DISABLE;
	ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
	if (ret < 0)
		return ret;
	return 0;
}

static int aw37501_offset_set(struct aw37501_power *aw37501)
{
	u8 reg_val;
	u8 set_val;
	int ret;

	pr_info("%s\n", __func__);

	if (aw37501->offset == AW_OFFSET_0MA) {
		set_val = AW_OFFSET_0MA_VAL;
	} else if (aw37501->offset == AW_OFFSET_50MA) {
		set_val = AW_OFFSET_50MA_VAL;
	} else if (aw37501->offset == AW_OFFSET_100MA) {
		set_val = AW_OFFSET_100MA_VAL;
	} else if (aw37501->offset == AW_OFFSET_150MA) {
		set_val = AW_OFFSET_150MA_VAL;
	} else {
		pr_err("%s, offset input val error.\n", __func__);
		return -EINVAL;
	}

	ret = aw37501_read(aw37501, AW37501_REG_CTRL, &reg_val);
	if (ret < 0) {
		pr_info("%s, aw37501_read error...\n", __func__);
		return ret;
	}

	reg_val = (reg_val & AW37501_OFFSET_CLEAR) | set_val;
	ret = aw37501_write(aw37501, AW37501_REG_CTRL, reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_write failed\n", __func__);
		return ret;
	}
	return ret;
}

static int aw37501_offset_read(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_CTRL, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}
	pr_info("%s val = %#x\n", __func__, reg_val);
	reg_val = (reg_val & AW37501_OFFSET_GET);
	if (reg_val == AW_OFFSET_0MA_VAL) {
		aw37501->offset = AW_OFFSET_0MA;
	} else if (reg_val == AW_OFFSET_50MA_VAL) {
		aw37501->offset = AW_OFFSET_50MA;
	} else if (reg_val == AW_OFFSET_100MA_VAL) {
		aw37501->offset = AW_OFFSET_100MA;
	} else if (reg_val == AW_OFFSET_150MA_VAL) {
		aw37501->offset = AW_OFFSET_150MA;
	} else {
		pr_err("%s, no match offset data val\n", __func__);
		return ret;
	}
	return 0;
}

static int aw37501_enn_enable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret = 0;

	if (aw37501->bool_gpio_ctrl == true) {
		ret = pinctrl_select_state(aw37501->pinctrl.pinctrl,
					     aw37501->pinctrl.aw_enn_high);
		if (ret < 0) {
			pr_info("%s pinctrl_select_state failed for aw_enn_high\n",
				__func__);
			return ret;
		}
	} else {
		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_OPEN_CMD);
		if (ret < 0)
			return ret;

		ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
		if (ret < 0)
			return ret;

		reg_val = reg_val | AW37501_ENN_ENABLE;
		ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
		if (ret < 0)
			return ret;

		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_CLOSE_CMD);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int aw37501_enn_disable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret = 0;

	if (aw37501->bool_gpio_ctrl == true) {
		ret = pinctrl_select_state(aw37501->pinctrl.pinctrl,
					     aw37501->pinctrl.aw_enn_low);
		if (ret < 0) {
			pr_info("%s pinctrl_select_state failed for aw_enn_low\n",
				__func__);
			return ret;
		}
	} else {
		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_OPEN_CMD);
		if (ret < 0)
			return ret;

		ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
		if (ret < 0)
			return ret;

		reg_val = reg_val & AW37501_ENN_DISABLE;
		ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
		if (ret < 0)
			return ret;

		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_CLOSE_CMD);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int aw37501_enp_enable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret = 0;

	if (aw37501->bool_gpio_ctrl == true) {
		ret = pinctrl_select_state(aw37501->pinctrl.pinctrl,
					     aw37501->pinctrl.aw_enp_high);
		if (ret < 0) {
			pr_info("%s pinctrl_select_state failed for aw_enp_high\n",
				__func__);
			return ret;
		}
	} else {
		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_OPEN_CMD);
		if (ret < 0)
			return ret;

		ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
		if (ret < 0)
			return ret;

		reg_val = reg_val | AW37501_ENP_ENABLE;
		pr_info("%s write reg_val = %#x\n", __func__, reg_val);

		ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
		if (ret < 0)
			return ret;

		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_CLOSE_CMD);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int aw37501_enp_disable(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret = 0;

	if (aw37501->bool_gpio_ctrl == true) {
		ret = pinctrl_select_state(aw37501->pinctrl.pinctrl,
					     aw37501->pinctrl.aw_enp_low);
		if (ret < 0) {
			pr_info("%s pinctrl_select_state failed for aw_enp_low\n",
				__func__);
			return ret;
		}
	} else {
		ret = aw37501_write(aw37501, AW37501_REG_PRO, AW37501_OPEN_CMD);
		if (ret < 0)
			return ret;

		ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
		if (ret < 0)
			return ret;

		reg_val = reg_val & AW37501_ENP_DISABLE;
		ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
		if (ret < 0)
			return ret;

		ret = aw37501_write(aw37501, AW37501_REG_PRO,
				    AW37501_CLOSE_CMD);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int aw37501_current_limit_set(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = (reg_val & AW37501_CURRENT_CLEAR) | aw37501->power_mode;
	ret = aw37501_write(aw37501, AW37501_REG_APPS, reg_val);
	if (ret < 0)
		return ret;
	return ret;
}

static int aw37501_current_limit_get(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0)
		return ret;
	aw37501->read_power_mode = reg_val;

	return 0;
}


static int aw37501_voltage_outp_set(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s out_val = %#x\n", __func__, aw37501->outp);

	ret = aw37501_read(aw37501, AW37501_REG_VOUTP, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}

	reg_val = (reg_val & AW37501_OUT_CLEAR) | aw37501->outp;
	ret = aw37501_write(aw37501, AW37501_REG_VOUTP, reg_val);
	if (ret < 0)
		return ret;
	return 0;
}

static int aw37501_voltage_outn_set(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s out_val = %#x\n", __func__, aw37501->outn);

	ret = aw37501_read(aw37501, AW37501_REG_VOUTN, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}

	reg_val = (reg_val & AW37501_OUT_CLEAR) | aw37501->outn;
	ret = aw37501_write(aw37501, AW37501_REG_VOUTN, reg_val);
	if (ret < 0)
		return ret;

	return 0;
}
static int aw37501_voltage_outp_get(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_VOUTP, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}
	aw37501->read_outp = reg_val;

	return 0;
}

static int aw37501_voltage_outn_get(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_VOUTN, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}
	aw37501->read_outn = reg_val;
	return 0;
}

static int aw37501_ldo_current_set(struct aw37501_power *aw37501)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_CTRL, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}
	if (aw37501->limit == MAX_LOO_CURRENT)
		reg_val = reg_val & MAX_LOO_CURRENT_VAL;
	else if (aw37501->limit == MIN_LOD_CURRENT)
		reg_val = reg_val & MIN_LOD_CURRENT_VAL;

	ret = aw37501_write(aw37501, AW37501_REG_CTRL, reg_val);
	if (ret < 0)
		return ret;

	return ret;
}

static int aw37501_status_read(struct aw37501_power *aw37501, u8 *val)
{
	u8 reg_val;
	int ret;

	pr_info("%s\n", __func__);

	ret = aw37501_read(aw37501, AW37501_REG_APPS, &reg_val);
	if (ret < 0) {
		pr_err("%s, aw37501_read error...\n", __func__);
		return ret;
	}
	*val = reg_val;

	return 0;
}

/* ************************* regulator driver ************************* */
static int aw37501_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	pr_info("%s enter test\n", __func__);

	if (selector >= rdev->desc->n_voltages ||
	    selector < rdev->desc->linear_min_sel)
		return -EINVAL;
	if (strcmp(rdev->desc->name, "outp")) {
		pr_info("%s enter outp\n", __func__);
		return rdev->desc->min_uV + (rdev->desc->uV_step * selector);
	} else if (strcmp(rdev->desc->name, "outn")) {
		pr_info("%s enter outn\n", __func__);
		return rdev->desc->min_uV + (rdev->desc->uV_step * selector);
	} else {
		return -ENODEV;
	}
	return 0;
}

static int
aw37501_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);
	int ret = 0;

	pr_info("%s enter selector = %#x\n", __func__, selector);

	if (strcmp(rdev->desc->name, "outp")) {
		aw37501->outp = selector;
		pr_info("%s enter set outp = %#x\n", __func__, aw37501->outp);

		ret = aw37501_voltage_outp_set(aw37501);
		if (ret < 0) {
			pr_err("%s, aw37501_voltage_outp_set error\n",
				__func__);
			return ret;
		}
	} else if (strcmp(rdev->desc->name, "outn")) {
		aw37501->outn = selector;
		pr_info("%s enter set outn = %#x\n", __func__, aw37501->outn);

		ret = aw37501_voltage_outn_set(aw37501);
		if (ret < 0) {
			pr_err("%s, aw37501_voltage_outn_set error\n",
				__func__);
			return ret;
		}
	} else {
		pr_err("%s, regulator outp/outn set failed\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static int aw37501_get_voltage_sel(struct regulator_dev *rdev)
{
	int i = 0;
	int ret = 0;
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);

	pr_info("%s enter test\n", __func__);

	if (!strcmp(rdev->desc->name, "outp")) {
		ret = aw37501_voltage_outp_get(aw37501);
		if (ret < 0) {
			return ret;
			pr_err("%s, aw37501_voltage_outp_get error\n",
				__func__);
		}
		pr_info("%s get outp = %#x\n", __func__, aw37501->read_outp);

		for (i = 0; i < ARRAY_SIZE(aw_vout_regval); i++) {
			if (aw37501->read_outp == aw_vout_regval[i]) {
				pr_info("%s, aw_vout_map_outp[%d] = %d\n",
					__func__, i, aw_vout_map_outp[i]);
				return aw_vout_map_outp[i];
			}
		}
	} else if (!strcmp(rdev->desc->name, "outn")) {
		ret = aw37501_voltage_outn_get(aw37501);
		if (ret < 0) {
			return ret;
			pr_err("%s, aw37501_voltage_outn_get error\n",
				__func__);
		}
		pr_info("%s  get outn = %#x\n", __func__, aw37501->read_outn);

		for (i = 0; i < ARRAY_SIZE(aw_vout_regval); i++) {
			if (aw37501->read_outn == aw_vout_regval[i]) {
				pr_info("%s, aw_vout_map_outn[%d] = %d\n",
					__func__, i, aw_vout_map_outn[i]);
				return aw_vout_map_outn[i];
			}
		}

	} else {
		pr_err("%s, regulator dev error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int aw37501_set_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);
	int ret = 0;

	pr_info("%s enter min = %d, max = %d\n", __func__, min_uA, max_uA);

	if (min_uA < AW_CURRENT_40MA)
		aw37501->power_mode = AW_CURRENT_40MA_VAL;
	else if (min_uA < AW_CURRENT_80MA)
		aw37501->power_mode = AW_CURRENT_80MA_VAL;
	else if (min_uA < AW_CURRENT_100MA)
		aw37501->power_mode = AW_CURRENT_100MA_VAL;
	else
		return -EINVAL;

	ret = aw37501_current_limit_set(aw37501);
	if (ret < 0) {
		pr_err("%s,aw37501_current_limit_set failed\n", __func__);
		return ret;
	}

	return ret;
}

static int aw37501_get_current_limit(struct regulator_dev *rdev)
{
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);
	int ret = 0;

	pr_info("%s enter\n", __func__);

	ret = aw37501_current_limit_get(aw37501);
	if (ret < 0) {
		pr_err("%s, aw37501_current_limit_get failed\n", __func__);
		return ret;
	}
	if ((aw37501->read_power_mode & AW37501_CURRENT_GET) == AW_CURRENT_40MA_VAL)
		ret = AW_CURRENT_40MA;
	else if ((aw37501->read_power_mode & AW37501_CURRENT_GET) == AW_CURRENT_80MA_VAL)
		ret = AW_CURRENT_80MA;
	else if ((aw37501->read_power_mode & AW37501_CURRENT_GET) == AW_CURRENT_100MA_VAL)
		ret = AW_CURRENT_100MA;
	else {
		pr_err("%s, no matched data val\n", __func__);
		return ret;
	}

	pr_info("%s ret = %d\n", __func__, ret);
	return ret;
}

static int aw37501_enable(struct regulator_dev *rdev)
{
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);
	int ret = 0;

	pr_info("%s enter\n", __func__);

	if (!strcmp(rdev->desc->name, "outp")) {
		ret = aw37501_enp_enable(aw37501);
		if (ret < 0) {
			pr_err("%s, aw37501_enp_enable failed\n", __func__);
			return ret;
		}
	} else if (!strcmp(rdev->desc->name, "outn")) {
		ret = aw37501_enn_enable(aw37501);
		if (ret < 0) {
			pr_err("%s, aw37501_enn_enable failed\n", __func__);
			return ret;
		}
	} else {
		pr_err("%s, regulator enable error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int aw37501_disable(struct regulator_dev *rdev)
{
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);
	int ret = 0;

	pr_info("%s enter\n", __func__);

	if (!strcmp(rdev->desc->name, "outp")) {
		ret = aw37501_enp_disable(aw37501);
		if (ret < 0) {
			pr_err("%s, aw37501_enp_disable failed\n", __func__);
			return ret;
		}
	} else if (!strcmp(rdev->desc->name, "outn")) {
		ret = aw37501_enn_disable(aw37501);
		if (ret < 0) {
			pr_err("%s, aw37501_enn_disable failed\n", __func__);
			return ret;
		}
	} else {
		pr_err("%s, regulator disable error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int aw37501_is_enabled(struct regulator_dev *rdev)
{
	struct aw37501_power *aw37501 = rdev_get_drvdata(rdev);
	u8 val = 0;
	int ret = -1;

	if (aw37501->bool_gpio_ctrl == true) {
		if (!strcmp(rdev->desc->name, "outp")) {
			val = gpio_get_value(aw37501->enp_gpio);
			return val;
		} else if (!strcmp(rdev->desc->name, "outn")) {
			val = gpio_get_value(aw37501->enn_gpio);
			return val;
		}
	} else {
		ret = aw37501_status_read(aw37501, &val);
		if (ret < 0)
			return ret;
		if (!strcmp(rdev->desc->name, "outp")) {
			val = (val & (1 << 3)) >> 3;
			return val;
		} else if (!strcmp(rdev->desc->name, "outn")) {
			val = (val & (1 << 4)) >> 4;
			return val;
		}
	}
	return ret;
}

static struct regulator_ops aw37501_outp_ops = {
	.list_voltage = aw37501_list_voltage,
	.set_voltage_sel = aw37501_set_voltage_sel,
	.get_voltage_sel = aw37501_get_voltage_sel,
	.set_current_limit = aw37501_set_current_limit,
	.get_current_limit = aw37501_get_current_limit,
	.enable = aw37501_enable,
	.disable = aw37501_disable,
	.is_enabled = aw37501_is_enabled,
};

static struct regulator_ops aw37501_outn_ops = {
	.list_voltage = aw37501_list_voltage,
	.set_voltage_sel = aw37501_set_voltage_sel,
	.get_voltage_sel = aw37501_get_voltage_sel,
	.set_current_limit = aw37501_set_current_limit,
	.get_current_limit = aw37501_get_current_limit,
	.enable = aw37501_enable,
	.disable = aw37501_disable,
	.is_enabled = aw37501_is_enabled,
};

static struct regulator_desc aw37501_desc[] = {
	{
		.name = "outp",
		.id = 0,
		.n_voltages = N_VOLTAGE,
		.min_uV = MIN_VOLTAGE,
		.uV_step = UV_STEP,
		.linear_min_sel = 0,
		.ops = &aw37501_outp_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "outn",
		.id = 1,
		.n_voltages = N_VOLTAGE,
		.min_uV = MIN_VOLTAGE,
		.uV_step = UV_STEP,
		.linear_min_sel = 0,
		.ops = &aw37501_outn_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int aw37501_pinctrl_init(struct aw37501_power *aw37501)
{
	struct aw37501_pinctrl *pinctrl = &aw37501->pinctrl;

	pinctrl->pinctrl = devm_pinctrl_get(aw37501->dev);
	if (IS_ERR_OR_NULL(pinctrl->pinctrl)) {
		pr_err("%s:No pinctrl found\n", __func__);
		pinctrl->pinctrl = NULL;
		return 0;
	}
	/* for enn */
	pinctrl->aw_enn_default = pinctrl_lookup_state(pinctrl->pinctrl,
						       "aw_enn_default");
	if (IS_ERR_OR_NULL(pinctrl->aw_enn_default)) {
		pr_err("%s: Failed get pinctrl state:enn default state\n",
			__func__);
		goto exit_pinctrl_init;
	}

	pinctrl->aw_enn_high = pinctrl_lookup_state(pinctrl->pinctrl,
						    "aw_enn_output_high");
	if (IS_ERR_OR_NULL(pinctrl->aw_enn_high)) {
		pr_err("%s: Failed get pinctrl state:enn_high\n", __func__);
		goto exit_pinctrl_init;
	}

	pinctrl->aw_enn_low = pinctrl_lookup_state(pinctrl->pinctrl,
						   "aw_enn_output_low");
	if (IS_ERR_OR_NULL(pinctrl->aw_enn_low)) {
		pr_err("%s: Failed get pinctrl state:enn_low\n", __func__);
		goto exit_pinctrl_init;
	}
	/* for enp */
	pinctrl->aw_enp_default = pinctrl_lookup_state(pinctrl->pinctrl,
						       "aw_enp_default");
	if (IS_ERR_OR_NULL(pinctrl->aw_enp_default)) {
		pr_err("%s: Failed get pinctrl state:enp default state\n",
			__func__);
		goto exit_pinctrl_init;
	}

	pinctrl->aw_enp_high = pinctrl_lookup_state(pinctrl->pinctrl,
						    "aw_enp_output_high");
	if (IS_ERR_OR_NULL(pinctrl->aw_enp_high)) {
		pr_err("%s: Failed get pinctrl state:enp_high\n", __func__);
		goto exit_pinctrl_init;
	}

	pinctrl->aw_enp_low = pinctrl_lookup_state(pinctrl->pinctrl,
						    "aw_enp_output_low");
	if (IS_ERR_OR_NULL(pinctrl->aw_enp_low)) {
		pr_err("%s: Failed get pinctrl state:enp_low\n", __func__);
		goto exit_pinctrl_init;
	}
	pr_info("%s: Success init pinctrl\n", __func__);
	return 0;
 exit_pinctrl_init:
	devm_pinctrl_put(pinctrl->pinctrl);
	return -ENODEV;
}

static void aw37501_pinctrl_deinit(struct aw37501_power *aw37501)
{
	if (aw37501->pinctrl.pinctrl)
		devm_pinctrl_put(aw37501->pinctrl.pinctrl);
}

/***************************sys attribute*********************************/
static ssize_t aw37501_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);

	unsigned char i = 0, reg_val = 0;
	ssize_t len = 0;
	int ret = 0;

	pr_info("%s\n", __func__);

	for (i = 0; i < AW37501_REG_MAX; i++) {
		if (!(aw37501_reg_access[i] & REG_RD_ACCESS))
		continue;
		ret = aw37501_read(aw37501, i, &reg_val);
		if (ret < 0) {
			pr_err("aw37501_read failed,ret = %d\n", ret);
			break;
		}
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x\n",
				i, reg_val);
	}

	return len;
}

static ssize_t aw37501_reg_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw37501_write(aw37501, (unsigned char)databuf[0],
				       (unsigned char)databuf[1]);
	}

	return len;
}

static ssize_t aw37501_power_outp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw37501->regulators[0]
		     ->desc->ops->get_voltage_sel(aw37501->regulators[0]);
	if (ret < 0) {
		pr_err("get_voltage_sel failed,ret = %d\n", ret);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "outp : %d\n", ret);

	return len;
}

static ssize_t aw37501_power_outp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int databuf[1] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] >= MIN_VOLTAGE_VAL && databuf[0] <= MAX_VOLTAGE_VAL) {
			ret =
			aw37501->regulators[1]->desc->ops->set_voltage_sel(aw37501->regulators[1],
			(unsigned char)databuf[0]);
			if (ret < 0) {
				dev_err(dev, "%s set_voltage_sel error\n", __func__);
				return ret;
			}
		}
	}

	return len;
}

static ssize_t aw37501_power_outn_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw37501->regulators[1]
		     ->desc->ops->get_voltage_sel(aw37501->regulators[1]);
	if (ret < 0 && ret > -MIN_VOLTAGE) {
		dev_err(dev, "%s get_voltage_sel failed\n", __func__);
		return ret;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "outn : %d\n", ret);

	return len;
}

static ssize_t aw37501_power_outn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int databuf[1] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] >= MIN_VOLTAGE_VAL && databuf[0] <= MAX_VOLTAGE_VAL) {
			ret = aw37501->regulators[1]->desc->ops->set_voltage_sel(aw37501->regulators[0],
			(unsigned char)databuf[0]);
			if (ret < 0) {
				dev_err(dev, "%s set_voltage_sel error\n",
					__func__);
				return ret;
			}
		}
	}

	return len;
}

static ssize_t aw37501_offset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw37501_offset_read(aw37501);
	if (ret < 0) {
		dev_err(dev, "%s aw37501_offset_read error\n", __func__);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "offset: %d\n",
			aw37501->offset);

	return len;
}

static ssize_t aw37501_offset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int val = 0;
	int rc = 0;

	pr_info("%s\n", __func__);

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	aw37501->offset = val;
	rc = aw37501_offset_set(aw37501);
	if (rc < 0) {
		dev_err(dev, "%s aw37501_offset_set error\n", __func__);
		return rc;
	}

	return len;
}

static ssize_t aw37501_enn_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = aw37501->regulators[1]
		     ->desc->ops->is_enabled(aw37501->regulators[1]);
	if (ret < 0) {
		dev_err(dev, "%s is_enabled error\n", __func__);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "outn status: %d\n", ret);

	return len;
}

static ssize_t aw37501_enn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	pr_info("%s\n", __func__);

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}

	if (val == 1) {
		ret = aw37501->regulators[1]
			     ->desc->ops->enable(aw37501->regulators[1]);
		if (ret < 0) {
			dev_err(dev, "%s enable error\n", __func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw37501->regulators[1]
			     ->desc->ops->disable(aw37501->regulators[1]);
		if (ret < 0) {
			dev_err(dev, "%s disable error\n", __func__);
			return ret;
		}
	} else {
		dev_err(dev, "%s input val error\n", __func__);
		return -EINVAL;
	}
	return len;
}

static ssize_t aw37501_enp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	ssize_t len = 0;
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = aw37501->regulators[0]
		     ->desc->ops->is_enabled(aw37501->regulators[0]);
	if (ret < 0) {
		dev_err(dev, "%s is_enabled error\n", __func__);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "outp status: %d\n", ret);

	return len;
}

static ssize_t aw37501_enp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	pr_info("%s\n", __func__);

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	pr_info("%s val = %d\n", __func__, val);

	if (val == 1) {
		ret = aw37501->regulators[0]
			     ->desc->ops->enable(aw37501->regulators[0]);
		if (ret < 0) {
			dev_err(dev, "%s enable error\n", __func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw37501->regulators[0]
			     ->desc->ops->disable(aw37501->regulators[0]);
		if (ret < 0) {
			dev_err(dev, "%s disable error\n", __func__);
			return ret;
		}
	} else {
		dev_err(dev, "%s input val error\n", __func__);
		return -EINVAL;
	}
	return len;
}

static ssize_t aw37501_current_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw37501->regulators[0]
		     ->desc->ops->get_current_limit(aw37501->regulators[0]);
	if (ret < 0) {
		dev_err(dev, "%s get_current_limit error\n", __func__);
		return ret;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "modes: %d\n", ret);

	return len;
}

static ssize_t aw37501_current_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int databuf[2] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		ret = aw37501->regulators[1]->desc->ops->
				set_current_limit(aw37501->regulators[1],
						  databuf[0], databuf[1]);
		if (ret < 0) {
			dev_err(dev, "%s set_current_limit error\n", __func__);
			return ret;
		}
	}

	return len;
}

static ssize_t aw37501_disn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val > 0) {
		ret = aw37501_disn_enable(aw37501);
		if (ret < 0) {
			dev_err(dev, "%s aw37501_disn_enable error\n",
				__func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw37501_disn_disable(aw37501);
		if (ret < 0) {
			dev_err(dev, "%s aw37501_disn_disable error\n",
				__func__);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return len;
}

static ssize_t aw37501_disp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val > 0) {
		ret = aw37501_disp_enable(aw37501);
		if (ret < 0) {
			dev_err(dev, "%s aw37501_disp_enable error\n",
				__func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw37501_disp_disable(aw37501);
		if (ret < 0) {
			dev_err(dev, "%s aw37501_disp_disable error\n",
				__func__);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return len;
}

static ssize_t aw37501_listoutp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val < 0 || val >= N_VOLTAGE) {
		dev_err(dev, "%s,error n_voltage = %d\n", __func__, val);
		return -EINVAL;
	}

	ret =
	aw37501->regulators[1]->desc->ops->list_voltage(aw37501->regulators[1],
							val);
	if (ret < 0) {
		dev_err(dev, "%s list_voltage error\n", __func__);
		return ret;
	}
	pr_info("%s,voltage = %d\n", __func__, ret);

	return len;
}

static ssize_t aw37501_listoutn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val < 0 || val >= N_VOLTAGE)
		return -EINVAL;

	ret =
	aw37501->regulators[0]->desc->ops->list_voltage(aw37501->regulators[0],
							val);
	if (ret < 0) {
		dev_err(dev, "%s list_voltage error\n", __func__);
		return ret;
	}
	pr_info("%s,voltage = %d\n", __func__, ret);

	return len;
}

static ssize_t aw37501_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw37501_power *aw37501 = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}

	if (val == 1) {
		ret = aw37501_soft_reset(aw37501);
		if (ret < 0) {
			dev_err(dev, "%s aw37501_soft_reset error\n", __func__);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return len;
}

static DEVICE_ATTR(reg, 0664, aw37501_reg_show, aw37501_reg_store);
static DEVICE_ATTR(outp, 0664, aw37501_power_outp_show,
			       aw37501_power_outp_store);
static DEVICE_ATTR(outn, 0664, aw37501_power_outn_show,
			       aw37501_power_outn_store);
static DEVICE_ATTR(offset, 0664, aw37501_offset_show, aw37501_offset_store);
static DEVICE_ATTR(enn, 0664, aw37501_enn_show, aw37501_enn_store);
static DEVICE_ATTR(enp, 0664, aw37501_enp_show, aw37501_enp_store);
static DEVICE_ATTR(current_mode, 0664, aw37501_current_mode_show,
				       aw37501_current_mode_store);
static DEVICE_ATTR(disn, 0664, NULL, aw37501_disn_store);
static DEVICE_ATTR(disp, 0664, NULL, aw37501_disp_store);
static DEVICE_ATTR(list_outp, 0664, NULL, aw37501_listoutp_store);
static DEVICE_ATTR(list_outn, 0664, NULL, aw37501_listoutn_store);
static DEVICE_ATTR(reset, 0664, NULL, aw37501_reset_store);


static struct attribute *aw37501_power_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_outp.attr,
	&dev_attr_outn.attr,
	&dev_attr_offset.attr,
	&dev_attr_enn.attr,
	&dev_attr_enp.attr,
	&dev_attr_current_mode.attr,
	&dev_attr_disn.attr,
	&dev_attr_disp.attr,
	&dev_attr_list_outp.attr,
	&dev_attr_list_outn.attr,
	&dev_attr_reset.attr,
	NULL,
};

static struct attribute_group aw37501_power_attr_group = {
	.attrs = aw37501_power_attributes
};

/* **********************************driver********************************** */
static int aw37501_power_init(struct aw37501_power *aw37501)
{
	int ret = 0;

	pr_info("%s\n", __func__);


	if(aw37501->chip_id == AW37501_CHIP_ID) {
		/* VREG offset voltage configure */
		ret = aw37501_offset_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_offset_set error\n",
				__func__);
			return ret;
		}

		/* Current mode */
		ret = aw37501_current_limit_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_current_limit_set error\n",
				__func__);
			return ret;
		}

		/* LDO output current limit value configure*/
		ret = aw37501_ldo_current_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_ldo_current_set error\n",
				__func__);
			return ret;
		}

		/* Output voltage of LDO */
		ret = aw37501_voltage_outp_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_voltage_outp_set error\n",
				__func__);
			return ret;
		}

		/* Output voltage of CHARGE PUMP */
		ret = aw37501_voltage_outn_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_voltage_outn_set error\n",
				__func__);
			return ret;
		}
	} else {

		/* Current mode */
		ret = aw37501_current_limit_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_current_limit_set error\n",
				__func__);
			return ret;
		}

		/* Output voltage of LDO */
		ret = aw37501_voltage_outp_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_voltage_outp_set error\n",
				__func__);
			return ret;
		}

		/* Output voltage of CHARGE PUMP */
		ret = aw37501_voltage_outn_set(aw37501);
		if (ret < 0) {
			dev_err(aw37501->dev, "%s aw37501_voltage_outn_set error\n",
				__func__);
			return ret;
		}
	}
	return 0;
}

static int
aw37501_gpio_init(struct aw37501_power *aw37501, struct i2c_client *i2c)
{
	int ret;

	pr_info("%s enter\n", __func__);
	ret = aw37501_pinctrl_init(aw37501);
	if (ret < 0) {
		pr_err("%s: Failed get wanted pinctrl state\n", __func__);
		return ret;
	}
	ret = pinctrl_select_state(aw37501->pinctrl.pinctrl,
					     aw37501->pinctrl.aw_enn_high);
	if (ret < 0) {
		pr_err("%s: pinctrl_select_state failed for aw_enn_high\n",
			__func__);
		return ret;
	}
	ret = pinctrl_select_state(aw37501->pinctrl.pinctrl,
					     aw37501->pinctrl.aw_enp_high);
	if (ret < 0) {
		pr_err("%s: pinctrl_select_state failed for aw_enp_high\n",
			__func__);
		return ret;
	}
	return 0;
}

static int
aw37501_parse_dt(struct i2c_client *i2c, struct aw37501_power *aw37501,
		 struct device_node *np)
{
	int rc = 0;
	struct device *dev = &i2c->dev;

	pr_info("%s enter\n", __func__);


	aw37501->bool_regulator_fixed = of_property_read_bool(np, "bool_regulator_fixed");
	dev_info(dev, "%s: bool_regulator_fixed %d.\n",
			__func__, aw37501->bool_regulator_fixed);

	aw37501->bool_gpio_ctrl = of_property_read_bool(np, "bool_gpio_ctrl");

	if (aw37501->bool_gpio_ctrl == true) {
		aw37501->enn_gpio = of_get_named_gpio(np, "enn-gpio", 0);
		if (gpio_is_valid(aw37501->enn_gpio)) {
			dev_info(dev, "%s: enn gpio provided ok.\n", __func__);
		} else {
			dev_err(dev, "%s: no enn gpio provided.\n", __func__);
			return -EIO;
		}

		aw37501->enp_gpio = of_get_named_gpio(np, "enp-gpio", 0);
		if (gpio_is_valid(aw37501->enp_gpio)) {
			dev_info(dev, "%s: enp gpio provided ok.\n", __func__);
		} else {
			dev_err(dev, "%s: no enp gpio provided.\n", __func__);
			return -EIO;
		}

		rc = aw37501_gpio_init(aw37501, i2c);
		if (rc < 0) {
			dev_err(&aw37501->client->dev,
				"aw37501_gpio_init failed, rc = %d\n", rc);
			return rc;
		}

		pr_info("%s: ctrl by gpio\n", __func__);

	} else {
		pr_info("%s: ctrl by IIC\n", __func__);
	}

	rc = of_property_read_u32(np, "outp", &aw37501->outp);
	if (rc < 0) {
		dev_err(&aw37501->client->dev,
			"Failure reading outp, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,outp = %#x\n",  __func__, aw37501->outp);
	}

	rc = of_property_read_u32(np, "outn", &aw37501->outn);
	if (rc < 0) {
		dev_err(&aw37501->client->dev,
			"Failure reading outn, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,outn = %#x\n",  __func__, aw37501->outn);
	}

	rc = of_property_read_u32(np, "power_mode", &aw37501->power_mode);
	if (rc < 0) {
		dev_err(&aw37501->client->dev,
			"Failure reading mode, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,power mode = %#x\n", __func__, aw37501->power_mode);
	}

	rc = of_property_read_u32(np, "offset", &aw37501->offset);
	if (rc < 0) {
		dev_err(&aw37501->client->dev,
			"Failure reading offset, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,power offset = %d\n",  __func__, aw37501->offset);
	}

	rc = of_property_read_u32(np, "limit", &aw37501->limit);
	if (rc < 0) {
		dev_err(&aw37501->client->dev,
			"Failure reading limit, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,power limit = %d\n", __func__, aw37501->limit);
	}

	return 0;
read_err:
	if (aw37501->bool_gpio_ctrl == true)
		aw37501_pinctrl_deinit(aw37501);
	return rc;
}

static int aw37501_read_chipid(struct aw37501_power *aw37501)
{
	int ret;
	u8 val;

	pr_info("%s enter\n", __func__);
	ret = aw37501_read(aw37501, AW37501_REG_CTRL, &val);
	if (ret < 0) {
		pr_err("aw37501_read failed,ret = %d\n", ret);
		return ret;
	}
	val = val & AW37501_GET_CHIP_ID;
	aw37501->chip_id = val;
	if (val == AW37501_CHIP_ID)
		pr_info("%s,awinic IC, chip id = 0x%02x\n", __func__, val);
	else {
		pr_info("%s,not awinic IC, chip id = 0x%02x\n", __func__, val);
		return ret;
	}

	return 0;
}

static int aw37501_power_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	int ret = 0;
	int i;
	struct aw37501_power *aw37501 = NULL;
	struct device_node *node = i2c->dev.of_node;

	pr_info("%s\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw37501 = devm_kzalloc(&i2c->dev,
			(sizeof(struct aw37501_power)), GFP_KERNEL);
	if (!aw37501)
		return -ENOMEM;

	aw37501->dev = &i2c->dev;
	aw37501->client = i2c;

	ret = aw37501_read_chipid(aw37501);
	if (ret < 0)
		goto chip_error;

	if (node) {
		ret = aw37501_parse_dt(i2c, aw37501, node);
		if (ret < 0)
			goto chip_error;
	} else {
		pr_info("%s, failed device node if null!\n", __func__);
		goto chip_error;
	}
	i2c_set_clientdata(i2c, aw37501);

	if (aw37501->bool_regulator_fixed != true) {
		for (i = 0; i < ARRAY_SIZE(aw37501_desc); i++) {

			struct regulator_desc *desc = &aw37501_desc[i];
			struct regulator_init_data *init = NULL;
			struct regulator_config config = { };

			init = of_get_regulator_init_data(&i2c->dev, node, desc);

			/* set never disable */
			init->constraints.always_on = 1;

			if (init == NULL) {
				dev_err(&i2c->dev, " %s Failed to get init_data %s\n",
					__func__, desc->name);
				of_node_put(node);
				goto regulator_error;
			}
			config.dev = &i2c->dev;
			config.init_data = init;
			config.driver_data = aw37501;
			config.of_node = i2c->dev.of_node;

			pr_info("register regulator %s\n", desc->name);
			aw37501->regulators[i] = devm_regulator_register(&i2c->dev,
									 desc, &config);
			if (IS_ERR(aw37501->regulators[i])) {
				ret = PTR_ERR(aw37501->regulators[i]);
				dev_err(&i2c->dev, "failed to register regulator %s:%d\n",
					desc->name, ret);
				goto regulator_error;
			}
		}
	}

	i2c_set_clientdata(i2c, aw37501);

	ret = sysfs_create_group(&aw37501->dev->kobj,
				&aw37501_power_attr_group);
	if (ret < 0) {
		dev_err(&aw37501->client->dev, "power sysfs ret: %d\n", ret);
		goto regulator_error;
	}

	ret = aw37501_power_init(aw37501);
	if (ret < 0) {
		dev_err(&aw37501->client->dev, "%s,aw37501_power_init failed\n",
			__func__);
		goto init_error;
	}

	return 0;
init_error:
	sysfs_remove_group(&aw37501->dev->kobj,
					&aw37501_power_attr_group);
regulator_error:
	while (--i >= 0)
		regulator_unregister(aw37501->regulators[i]);
chip_error:
	devm_kfree(&i2c->dev, aw37501);
	aw37501 = NULL;
	return ret;
}

static int aw37501_power_remove(struct i2c_client *i2c)
{
	struct aw37501_power *aw37501 = i2c_get_clientdata(i2c);
	int i = 2;

	pr_info("%s\n", __func__);
	sysfs_remove_group(&aw37501->dev->kobj,
					&aw37501_power_attr_group);
	while (--i >= 0)
		devm_regulator_unregister(aw37501->dev, aw37501->regulators[i]);

	if (aw37501->bool_gpio_ctrl == true)
		aw37501_pinctrl_deinit(aw37501);

	devm_kfree(&i2c->dev, aw37501);
	aw37501 = NULL;

	return 0;
}

static const struct i2c_device_id aw37501_power_id[] = {
	{"aw37501_led", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, aw37501_power_id);

static const struct of_device_id aw37501_match_table[] = {
	{.compatible = "awinic,aw37501",},
	{ },
};

static struct i2c_driver aw37501_power_driver = {
	.driver = {
		.name = "aw37501",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw37501_match_table),
	},
	.probe = aw37501_power_probe,
	.remove = aw37501_power_remove,
	.id_table = aw37501_power_id,
};

static int __init aw37501_module_init(void)
{
	pr_info("%s: driver version: %s\n", __func__, AW37501_DRIVER_VERSION);
	return i2c_add_driver(&aw37501_power_driver);
}
module_init(aw37501_module_init);

static void __exit aw37501_module_exit(void)
{
	i2c_del_driver(&aw37501_power_driver);
}
module_exit(aw37501_module_exit);

MODULE_AUTHOR("<shiqiang@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW37501 Power driver");
MODULE_LICENSE("GPL v2");

