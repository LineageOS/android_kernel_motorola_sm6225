/*
 * Copyright (c) 2022 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define pr_fmt(fmt)     "sc760x-charger: %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/regmap.h>

#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/time64.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#include "sc760x_charger_mmi.h"


static const char* sc760x_irq_name[] = {
    [SC760X_MASTER] = "sc760x_master_irq",
    [SC760X_SLAVE] = "sc760x_slave_irq",
};

static int sc760x_role_data[] = {
    [SC760X_MASTER] = SC760X_MASTER,
    [SC760X_SLAVE] = SC760X_SLAVE,
};

static struct sc760x_cfg_e default_cfg = {
    .bat_chg_lim_disable = 0,
    .bat_chg_lim = 39,
    .pow_lim_disable = 0,
    .ilim_disable = 0,
    .load_switch_disable = 0,
    .lp_mode_enable = 0,
    .itrichg = 3,
    .iprechg = 2,
    .vfc_chg = 2,
    .chg_ovp_disable = 0,
    .chg_ovp = 0,
    .bat_ovp_disable = 0,
    .bat_ovp = 10,
    .chg_ocp_disable = 0,
    .chg_ocp = 2,
    .dsg_ocp_disable = 0,
    .dsg_ocp = 2,
    .tdie_flt_disable = 0,
    .tdie_alm_disable = 0,
    .tdie_alm = 9,
};

static const int sc760x_adc_m[] =
    {3125, 125, 125, 3125, 5};

static const int sc760x_adc_l[] =
    {1000, 100, 100, 10000, 10};

//REGISTER
static const struct reg_field sc760x_reg_fields[] = {
    /*reg00*/
    [DEVICE_REV] = REG_FIELD(0x00, 4, 7),
    [DEVICE_ID] = REG_FIELD(0x00, 0, 3),
    /*reg01*/
    [IBAT_CHG_LIM] = REG_FIELD(0x01, 0, 7),
    /*reg02*/
    [POW_LIM_DIS] = REG_FIELD(0x02, 7, 7),
    [VBALANCE_DIS] = REG_FIELD(0x02, 5, 5),
    [POW_LIM] = REG_FIELD(0x02, 0, 3),
    /*reg03*/
    [ILIM_DIS] = REG_FIELD(0x03, 7, 7),
    [IBAT_CHG_LIM_DIS] = REG_FIELD(0x03, 6, 6),
    [BAT_DET_DIS] = REG_FIELD(0x03, 5, 5),
    [VDIFF_CHECK_DIS] = REG_FIELD(0x03, 4, 4),
    [LS_OFF] = REG_FIELD(0x03, 3, 3),
    [SHIP_EN] = REG_FIELD(0x03, 0, 2),
    /*reg04*/
    [REG_RST] = REG_FIELD(0x04, 7, 7),
    [EN_LOWPOWER] = REG_FIELD(0x04, 6, 6),
    [VDIFF_OPEN_TH] = REG_FIELD(0x04, 4, 5),
    [AUTO_BSM_DIS] = REG_FIELD(0x04, 3, 3),
    [AUTO_BSM_TH] = REG_FIELD(0x04, 2, 2),
    [SHIP_WT] = REG_FIELD(0x04, 0, 0),
    /*reg05*/
    [ITRICHG] = REG_FIELD(0x05, 5, 7),
    [VPRE_CHG] = REG_FIELD(0x05, 0, 2),
    /*reg06*/
    [IPRECHG] = REG_FIELD(0x06, 4, 7),
    [VFC_CHG] = REG_FIELD(0x06, 0, 3),
    /*reg07*/
    [CHG_OVP_DIS] = REG_FIELD(0x07, 7, 7),
    [CHG_OVP] = REG_FIELD(0x07, 6, 6),
    /*reg08*/
    [BAT_OVP_DIS] = REG_FIELD(0x08, 7, 7),
    [BAT_OVP] = REG_FIELD(0x08, 2, 6),
    /*reg09*/
    [CHG_OCP_DIS] = REG_FIELD(0x09, 7, 7),
    [CHG_OCP] = REG_FIELD(0x09, 4, 6),
    /*reg0A*/
    [DSG_OCP_DIS] = REG_FIELD(0x0A, 7, 7),
    [DSG_OCP] = REG_FIELD(0x0A, 4, 6),
    /*reg0B*/
    [TDIE_FLT_DIS] = REG_FIELD(0x0B, 7, 7),
    [TDIE_FLT] = REG_FIELD(0x0B, 0, 3),
    /*reg0C*/
    [TDIE_ALRM_DIS] = REG_FIELD(0x0C, 7, 7),
    [TDIE_ALRM] = REG_FIELD(0x0C, 0, 3),
    /*reg0D*/
    [CHG_OVP_DEG] = REG_FIELD(0x0D, 6, 7),
    [BAT_OVP_DEG] = REG_FIELD(0x0D, 4, 5),
    [CHG_OCP_DEG] = REG_FIELD(0x0D, 2, 3),
    [DSG_OCP_DEG] = REG_FIELD(0x0D, 0, 1),
    /*reg0E*/
    [AUTO_BSM_DEG] = REG_FIELD(0x0E, 6, 7),
    /*reg0F*/
    [WORK_MODE] = REG_FIELD(0x0F, 5, 7),
    /*reg15*/
    [ADC_EN] = REG_FIELD(0x15, 7, 7),
    [ADC_RATE] = REG_FIELD(0x15, 6, 6),
    [ADC_FREEZE] = REG_FIELD(0x15, 5, 5),
    [ADC_DONE_MASK] = REG_FIELD(0x15, 0, 0),
};

static const struct regmap_config sc760x_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
};

#ifndef MIN_VAL
   #define  MIN_VAL( x, y ) ( ((x) < (y)) ? (x) : (y) )
#endif

__attribute__((unused)) static int sc760x_set_ibat_dis(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_load_switch(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_lowpower_mode(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static int sc760x_set_itrickle(struct sc760x_chip *sc, int curr);
__attribute__((unused)) static int sc760x_set_iprechg(struct sc760x_chip *sc, int curr);
__attribute__((unused)) static int sc760x_set_vfcchg(struct sc760x_chip *sc, int volt);
__attribute__((unused)) static int sc760x_set_batovp(struct sc760x_chip *sc, int volt);
__attribute__((unused)) static int sc760x_set_adc_enable(struct sc760x_chip *sc, bool en);
__attribute__((unused)) static void sc760x_mmi_charger_deinit(struct sc760x_mmi_charger *chg);
__attribute__((unused)) static int sc760x_set_power_limit_dis(struct sc760x_chip *sc, bool en);

static int sc760x_mmi_charger_init(struct sc760x_mmi_charger *chg);
static int sc760x_get_state(struct sc760x_chip *sc,
			     struct sc760x_state *state);
static int sc760x_init_device(struct sc760x_chip *sc);
static struct power_supply_desc sc760x_power_supply_desc;

/*********************************************************/

static int sc760x_enable_chip(struct sc760x_chip *sc, bool en)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *state;
	const char *pinctrl_name;
	int ret;

	pinctrl = pinctrl_get(sc->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(sc->dev, "Failed to get pinctrl\n");
		return -EINVAL;
	}

	if (en)
		pinctrl_name = "sc760x_enable";
	else
		pinctrl_name = "sc760x_disable";


	state = pinctrl_lookup_state(pinctrl, pinctrl_name);
	if (IS_ERR_OR_NULL(state)) {
		dev_err(sc->dev, "fail to get %s state\n", pinctrl_name);
		ret = -ENODEV;
		goto put_pinctrl;
	}

	ret = pinctrl_select_state(pinctrl, state);
	if (ret) {
		dev_err(sc->dev, "fail to set %s state\n", pinctrl_name);
		ret = -EINVAL;
		goto put_pinctrl;
	}

	if (en)
	    msleep(2000);
	sc->sc760x_enable = en;

	if (sc->sc760x_enable) {

		if (!sc->irq_enabled) {
			enable_irq_wake(sc->irq);
			enable_irq(sc->irq);
			sc->irq_enabled = true;
		}
		ret = sc760x_init_device(sc);
		if (ret < 0) {
		    pr_info("init device failed(%d)\n", ret);
		}
	} else {
		if (sc->irq_enabled) {
			disable_irq_wake(sc->irq);
			disable_irq(sc->irq);
			sc->irq_enabled = false;
		}
	}
	dev_err(sc->dev, "success to set %s state, sleep 2s\n", pinctrl_name);

	ret = 0;
put_pinctrl:
	pinctrl_put(pinctrl);

	return ret;
}

static int sc760x_field_read(struct sc760x_chip *sc,
                enum sc760x_fields field_id, int *val)
{
    int ret;
    if (!sc->sc760x_enable)
        return -1;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc760x read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc760x_field_write(struct sc760x_chip *sc,
                enum sc760x_fields field_id, int val)
{
    int ret;
    if (!sc->sc760x_enable)
        return -1;

    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc760x read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc760x_read_block(struct sc760x_chip *sc,
                int reg, uint8_t *val, int len)
{
    int ret;
    if (!sc->sc760x_enable)
        return -1;

    ret = regmap_bulk_read(sc->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(sc->dev, "sc760x read %02x block failed %d\n", reg, ret);
    }

    return ret;
}

/*******************************************************/
static int sc760x_reg_reset(struct sc760x_chip *sc)
{
    return sc760x_field_write(sc, REG_RST, 1);
}

static int sc760x_set_auto_bsm_dis(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, AUTO_BSM_DIS, !!en);
}

static int sc760x_set_ibat_limit(struct sc760x_chip *sc, int curr)
{
    int user_val = INT_MAX;

    if (sc->user_ichg >= 0) {
        user_val = sc->user_ichg;
    }

    if (sc->user_ilim >= 0) {
        user_val = MIN_VAL(user_val, sc->user_ilim);
    }

    if (user_val != INT_MAX) {
        curr = user_val/1000;
        dev_info(sc->dev, "%s : User request overide ibat = %dmA\n", __func__, curr);
    }

    dev_info(sc->dev, "%s : %dmA\n", __func__, curr);

    return sc760x_field_write(sc, IBAT_CHG_LIM,
            (curr - IBAT_CHG_LIM_BASE) / IBAT_CHG_LIM_LSB);
}

static int sc760x_set_ibat_dis(struct sc760x_chip *sc, bool en)
{
    dev_info(sc->dev, "%s : ibat_chrg %d\n", __func__, en);
    return sc760x_field_write(sc, IBAT_CHG_LIM_DIS, !!en);
}

static int sc760x_get_ibat_limit(struct sc760x_chip *sc, int *curr)
{
    int data = 0, ret = 0;
    ret = sc760x_field_read(sc, IBAT_CHG_LIM, &data);
    *curr = data *50 + 50;
    return ret;
}

static int sc760x_set_power_limit_dis(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, POW_LIM_DIS, !!en);
}

static int sc760x_set_load_switch(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, LS_OFF, !!en);
}

static int sc760x_get_load_switch(struct sc760x_chip *sc, int *ls_en)
{
    int ret = 0;
    ret = sc760x_field_read(sc, LS_OFF, ls_en);
    return ret;
}

static int sc760x_set_lowpower_mode(struct sc760x_chip *sc, bool en)
{
    return sc760x_field_write(sc, EN_LOWPOWER, !!en);
}

static int sc760x_set_itrickle(struct sc760x_chip *sc, int curr)
{
    dev_info(sc->dev, "%s : %dmA\n", __func__, curr);

    return sc760x_field_write(sc, ITRICHG,
            (curr * 1000 - ITRICHG_BASE) / ITRICHG_LSB);
}

static int sc760x_set_iprechg(struct sc760x_chip *sc, int curr)
{
    dev_info(sc->dev, "%s : %dmA\n", __func__, curr);

    return sc760x_field_write(sc, IPRECHG,
            (curr - IPRECHG_BASE) / IPRECHG_LSB);
}

static int sc760x_set_vfcchg(struct sc760x_chip *sc, int volt)
{
    dev_info(sc->dev, "%s : %dmV\n", __func__, volt);

    return sc760x_field_write(sc, VFC_CHG,
            (volt - VFC_CHG_BASE) / VFC_CHG_LSB);
}

static int sc760x_set_batovp(struct sc760x_chip *sc, int volt)
{
    dev_info(sc->dev, "%s : %dmV\n", __func__, volt);

    return sc760x_field_write(sc, BAT_OVP,
            (volt - BAT_OVP_BASE) / BAT_OVP_LSB);
}

static int sc760x_get_work_mode(struct sc760x_chip *sc, int *mode)
{
    return sc760x_field_read(sc, WORK_MODE, mode);
}

static int sc760x_set_adc_enable(struct sc760x_chip *sc, bool en)
{
    dev_err(sc->dev, "%s set adc %d\n", __func__, en);
    return sc760x_field_write(sc, ADC_EN, !!en);
}

static int sc760x_set_adc_done_mask(struct sc760x_chip *sc, bool en)
{
    dev_info(sc->dev, "%s : set adc done mask %d\n", __func__, en);
    return sc760x_field_write(sc, ADC_DONE_MASK, !!en);
}

static int sc760x_get_adc(struct sc760x_chip *sc,
            ADC_CH channel, int *result)
{
    int reg = 0x17 + channel * 2;
    u8 val[2] = {0};
    int ret;

    ret = sc760x_read_block(sc, reg, val, 2);
    if (ret) {
        return ret;
    }

    *result = (val[1] | (val[0] << 8)) *
                sc760x_adc_m[channel] / sc760x_adc_l[channel];

    return ret;
}

static int sc760x_dump_reg(struct sc760x_chip *sc)
{
    int ret = 0, i = 0, val = 0, desc = 0;
    char buf[1024];
    if (!sc->sc760x_enable)
        return -1;

    for (i = 0; i <= 0x15; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        if (!ret) {
            desc +=
                sprintf(buf + desc, "[0x%02x]:0x%02x, ", i, val);
        }
    }

    dev_err(sc->dev, "Reg %s\n ", buf);
    return ret;
}

static irqreturn_t sc760x_irq_handler(int irq, void *data)
{
    struct sc760x_chip *sc = data;

    dev_dbg(sc->dev, "%s\n", __func__);
    //sc760x_dump_reg(sc);
    return IRQ_HANDLED;
}

static ssize_t sc760x_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    u8 addr;
    int val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc7603");
    for (addr = 0x0; addr <= 0x20; addr++) {
        ret = regmap_read(sc->regmap, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc760x_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= 0x20)
        regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, sc760x_show_registers, sc760x_store_register);

static ssize_t sc760x_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long enable;
	struct sc760x_chip *sc = dev_get_drvdata(dev);


	r = kstrtoul(buf, 0, &enable);
	if (r) {
		pr_err("Invalid tx_mode = %lu\n", enable);
		return -EINVAL;
	}

	if (enable < 0) {
		sc->user_gpio_en = -EINVAL;
		pr_info("Clear user gpio en setting\n");
	} else {
		sc->user_gpio_en = !!enable;
		pr_info("Set user gpio en: %d\n", sc->user_gpio_en);
		sc760x_enable_chip(sc, enable);
		sc->sc760x_enable = enable;
	}

	return r ? r : count;
}

static ssize_t sc760x_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", sc->sc760x_enable);
}

static DEVICE_ATTR(sc760x_enable, S_IRUGO|S_IWUSR, sc760x_enable_show, sc760x_enable_store);

static ssize_t charger_suspend_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int chg_suspend;
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 0, &chg_suspend);
	if (ret) {
		pr_err("Invalid chg_suspend value, ret = %d\n", ret);
		return ret;
	}

	if (chg_suspend < 0) {
		sc->user_chg_susp = -EINVAL;
		pr_info("Clear user suspend charger setting\n");
	} else {
		sc->user_chg_susp = !!chg_suspend;
		pr_info("Set user suspend charger: %d\n", sc->user_chg_susp);
		sc760x_set_load_switch(sc, sc->user_chg_susp);
	}
	cancel_delayed_work(&sc->charge_monitor_work);
	schedule_delayed_work(&sc->charge_monitor_work,
					msecs_to_jiffies(200));

	return count;
}

static ssize_t charger_suspend_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sc->user_chg_susp);
}
DEVICE_ATTR(charger_suspend, S_IRUGO|S_IWUSR, charger_suspend_show, charger_suspend_store);

static bool sc760x_is_enabled_charging(struct sc760x_chip *sc)
{
	int ret = 0, ls_off = 0;
	bool enabled = false;

	ret = sc760x_get_load_switch(sc, &ls_off);
	if(ret)
		return false;

	enabled = ls_off ? false : true;
	return enabled;
}

int sc760x_enable_charger(struct sc760x_chip *sc)
{
	int ret;

	if (sc->user_chg_en == 0) {
		pr_info("Skip enable charging for user request override\n");
		return 0;
	}

	if (sc->user_chg_susp > 0) {
		pr_info("Skip enable charging for user chg suspend\n");
		return 0;
	}

	ret = sc760x_set_load_switch(sc, false);
	pr_info("sc760x_enable_charger\n");
	return ret;
}

int sc760x_disable_charger(struct sc760x_chip *sc)
{
	int ret;

	if (sc->user_chg_en > 0) {
		pr_info("Skip disable charging for user request override\n");
		return 0;
	}

	ret = sc760x_set_load_switch(sc, true);

	pr_info("sc760x_disable_charger\n");
	return ret;
}

static ssize_t chg_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int enable;
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 0, &enable);
	if (ret) {
		pr_err("Invalid chg_en value, ret = %d\n", ret);
		return ret;
	}

	if (enable < 0) {
		sc->user_chg_en = -EINVAL;
		pr_info("Clear user enable charging setting\n");
	} else {
		sc->user_chg_en = !!enable;
		pr_info("Set user enable charging: %d\n", sc->user_chg_en);
		if (sc->user_chg_en)
			sc760x_enable_charger(sc);
		else
			sc760x_disable_charger(sc);
	}
	cancel_delayed_work(&sc->charge_monitor_work);
	schedule_delayed_work(&sc->charge_monitor_work,
					msecs_to_jiffies(200));

	return count;
}

static ssize_t chg_en_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	bool enable;
	struct sc760x_chip *sc = dev_get_drvdata(dev);

	if (!sc) {
		pr_err("sc chip not valid\n");
		return -ENODEV;
	}

	enable = sc760x_is_enabled_charging(sc);

	return sprintf(buf, "%d\n", enable);
}
DEVICE_ATTR(chg_en , S_IRUGO|S_IWUSR, chg_en_show, chg_en_store);

static void sc760x_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
    device_create_file(dev, &dev_attr_sc760x_enable);
    device_create_file(dev, &dev_attr_charger_suspend);
    device_create_file(dev, &dev_attr_chg_en);
}

static void sc760x_remove_device_node(struct device *dev)
{
    device_remove_file(dev, &dev_attr_registers);
    device_remove_file(dev, &dev_attr_sc760x_enable);
    device_remove_file(dev, &dev_attr_charger_suspend);
    device_remove_file(dev, &dev_attr_chg_en);
}

static int sc760x_set_thermal_mitigation(struct sc760x_chip *sc, int val)
{

	if (!sc->num_thermal_levels)
		return 0;

	if (sc->num_thermal_levels < 0) {
		pr_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

	if (val < 0 || val > sc->num_thermal_levels) {
		pr_err("Invalid thermal level: %d\n", val);
		return -EINVAL;
	}

	sc->thermal_fcc_ua = sc->thermal_levels[val];
	sc->curr_thermal_level = val;

	pr_info("thermal level: %d, thermal fcc: %d\n", sc->curr_thermal_level, sc->thermal_fcc_ua);

	return 0;
}


static int get_charger_type(struct sc760x_chip * sc)
{
	enum power_supply_usb_type usb_type =  POWER_SUPPLY_USB_TYPE_UNKNOWN;

	if (sc->use_ext_usb_psy && sc->usb_psy) {
		int ret = 0;
		union power_supply_propval val = {0};
		ret = power_supply_get_property(sc->usb_psy,
				POWER_SUPPLY_PROP_USB_TYPE, &val);
		if (!ret) {
			usb_type = val.intval;
		}
	}
	pr_debug("usb_type:%d\n", usb_type);
	return usb_type;
}

static enum power_supply_property sc760x_power_supply_props[] = {
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


static enum power_supply_usb_type sc760x_usb_type[] = {
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

static char *sc760x_charger_supplied_to[] = {
	"battery",
};

static int sc760x_property_is_writeable(struct power_supply *psy,
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

static int sc760x_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct sc760x_chip *sc = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0) {
			sc->user_ilim = -EINVAL;
			ret = 0;
			pr_info("Clear user input limit\n");
		} else {
			sc->user_ilim = val->intval;
			ret = sc760x_set_ibat_limit(sc, val->intval / 1000);
			pr_info("Set user input limit: %duA\n", sc->user_ilim);
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval < 0) {
			sc->user_ichg = -EINVAL;
			ret = 0;
			pr_info("Clear user charging current limit\n");
		} else {
			sc->user_ichg = val->intval;
			ret = sc760x_set_ibat_limit(sc, val->intval / 1000);
			pr_info("Set user charging current limit: %duA\n", sc->user_ichg);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = sc760x_set_thermal_mitigation(sc, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int sc760x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sc760x_chip *sc = power_supply_get_drvdata(psy);
	struct sc760x_state state;
	int chrg_status = 0;
	int ret = 0;

	mutex_lock(&sc->lock);
	ret = sc760x_get_state(sc, &state);
	mutex_unlock(&sc->lock);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (state.chrg_stat == CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_stat == DISCHARGING)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_stat == NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;	
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		sc760x_get_work_mode(sc, &chrg_status);
		switch (chrg_status) {
		case WORK_TRICKLE_CHARGE:
		case WORK_PRE_CHARGE:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case WORK_FULLY_ON:
		case WORK_CURRENT_REGULATION:
		case WORK_POWER_REGULATION:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case WORK_FULLY_OFF:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SC760X_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SC760X_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state.usb_online;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = sc760x_power_supply_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = get_charger_type(sc);
		break;

	case POWER_SUPPLY_PROP_HEALTH:

		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vchg_adc;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibat_adc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = sc->curr_thermal_level;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = sc->num_thermal_levels;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc760x_get_ibat_limit(sc, &val->intval);
		val->intval = val->intval * 1000;
		if (ret < 0)
			return ret;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc760x_get_ibat_limit(sc, &val->intval);
		val->intval = val->intval * 1000;
		if (ret < 0)
			return ret;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct power_supply_desc sc760x_power_supply_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.usb_types = sc760x_usb_type,
	.num_usb_types = ARRAY_SIZE(sc760x_usb_type),
	.properties = sc760x_power_supply_props,
	.num_properties = ARRAY_SIZE(sc760x_power_supply_props),
	.get_property = sc760x_charger_get_property,
	.set_property = sc760x_charger_set_property,
	.property_is_writeable = sc760x_property_is_writeable,
};

static int sc760x_power_supply_init(struct sc760x_chip *sc,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = sc,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = sc760x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sc760x_charger_supplied_to);

        dev_err(dev, "regiser master power supply\n");
	sc->charger_psy = devm_power_supply_register(sc->dev,
						 &sc760x_power_supply_desc,
						 &psy_cfg);

	if (IS_ERR(sc->charger_psy))
		return PTR_ERR(sc->charger_psy);

	return 0;
}

static int sc760x_parse_dt(struct sc760x_chip *sc, struct device *dev)
{
    int i, len, ret;
    u32 prev, val;
    const char *usb_psy_name = NULL;
    const char *wls_psy_name = NULL;

    struct device_node *node = dev->of_node;

    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc760x,bat-chg-lim-disable", &(sc->cfg->bat_chg_lim_disable)},
        {"sc,sc760x,bat-chg-lim", &(sc->cfg->bat_chg_lim)},
        {"sc,sc760x,pow-lim-disable", &(sc->cfg->pow_lim_disable)},
        {"sc,sc760x,ilim-disable", &(sc->cfg->ilim_disable)},
        {"sc,sc760x,load-switch-disable", &(sc->cfg->load_switch_disable)},
        {"sc,sc760x,low-power-mode-enable", &(sc->cfg->lp_mode_enable)},
        {"sc,sc760x,itrichg", &(sc->cfg->itrichg)},
        {"sc,sc760x,iprechg", &(sc->cfg->iprechg)},
        {"sc,sc760x,vfc-chg", &(sc->cfg->vfc_chg)},
        {"sc,sc760x,chg-ovp-disable", &(sc->cfg->chg_ovp_disable)},
        {"sc,sc760x,chg-ovp", &(sc->cfg->chg_ovp)},
        {"sc,sc760x,bat-ovp-disable", &(sc->cfg->bat_ovp_disable)},
        {"sc,sc760x,bat-ovp", &(sc->cfg->bat_ovp)},
        {"sc,sc760x,chg-ocp-disable", &(sc->cfg->chg_ocp_disable)},
        {"sc,sc760x,chg-ocp", &(sc->cfg->chg_ocp)},
        {"sc,sc760x,dsg-ocp-disable", &(sc->cfg->dsg_ocp_disable)},
        {"sc,sc760x,dsg-ocp", &(sc->cfg->dsg_ocp)},
        {"sc,sc760x,tdie-flt-disable", &(sc->cfg->tdie_flt_disable)},
        {"sc,sc760x,tdie-alm-disable", &(sc->cfg->tdie_alm_disable)},
        {"sc,sc760x,tdie-alm", &(sc->cfg->tdie_alm)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(node, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "can not read %s \n", props[i].name);
            continue;
        }
    }

	ret = device_property_read_string(sc->dev,
					"mmi,ext-usb-psy-name",
					&usb_psy_name);
	if (!ret && usb_psy_name)
		sc->use_ext_usb_psy = true;
	else
		sc->use_ext_usb_psy = false;

	ret = device_property_read_string(sc->dev,
					"mmi,ext-wls-psy-name",
					&wls_psy_name);
	if (!ret && wls_psy_name)
		sc->use_ext_wls_psy = true;
	else
		sc->use_ext_wls_psy = false;


	sc->init_data.charger_disabled = of_property_read_bool(node,
					"init-charger-disabled");

	ret = device_property_read_u32(sc->dev,
				       "iterm-microamp",
				       &sc->init_data.iterm);
	if (ret)
		sc->init_data.iterm = SC760x_TERMCHRG_I_DEF_uA;

	ret = device_property_read_u32(sc->dev,
				       "ichg-max-microamp",
				       &sc->init_data.max_ichg);
	if (ret)
		sc->init_data.max_ichg = SC760x_ICHRG_I_MAX_uA;

	ret = device_property_read_u32(sc->dev,
				       "vchg-max-microvolt",
				       &sc->init_data.max_vreg);
	if (ret)
		sc->init_data.max_vreg = SC760x_VREG_V_MAX_uV;

	ret = device_property_read_u32(sc->dev,
				       "ichg-microamp",
				       &sc->init_data.ichg);
	if (ret)
		sc->init_data.ichg = SC760x_ICHRG_I_DEF_uA;

	ret = of_property_count_elems_of_size(node, "mmi,thermal-mitigation",
							sizeof(u32));
	if (ret <= 0) {
		return 0;
	}

	len = ret;
	prev = sc->init_data.max_ichg;
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
			sc->num_thermal_levels = -EINVAL;
			return 0;
		}
		prev = val;
	}

	sc->thermal_levels = devm_kcalloc(sc->dev, len + 1,
					sizeof(*sc->thermal_levels),
					GFP_KERNEL);
	if (!sc->thermal_levels)
		return -ENOMEM;

	sc->thermal_levels[0] = sc->init_data.max_ichg;
	ret = of_property_read_u32_array(node, "mmi,thermal-mitigation",
						&sc->thermal_levels[1], len);
	if (ret < 0) {
		pr_err("Error in reading mmi,thermal-mitigation, rc=%d\n", ret);
		return ret;
	}
	sc->num_thermal_levels = len;
	sc->thermal_fcc_ua = sc->init_data.max_ichg;

	ret = device_property_read_u32(sc->dev,
				       "mmi,thermal-ratio",
				       &sc->thermal_ratio);
	if (ret)
		sc->thermal_ratio = 1;

    return ret;
}

static int sc760x_init_device(struct sc760x_chip *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc760x_fields field_id;
        int conv_data;
    } props[] = {
        {IBAT_CHG_LIM_DIS, sc->cfg->bat_chg_lim_disable},
        {IBAT_CHG_LIM, sc->cfg->bat_chg_lim},
        {POW_LIM_DIS, sc->cfg->pow_lim_disable},
        {ILIM_DIS, sc->cfg->ilim_disable},
        {LS_OFF, sc->cfg->load_switch_disable},
        {EN_LOWPOWER, sc->cfg->lp_mode_enable},
        {ITRICHG, sc->cfg->itrichg},
        {IPRECHG, sc->cfg->iprechg},
        {VFC_CHG, sc->cfg->vfc_chg},
        {CHG_OVP_DIS, sc->cfg->chg_ovp_disable},
        {CHG_OVP, sc->cfg->chg_ovp},
        {BAT_OVP_DIS, sc->cfg->bat_ovp_disable},
        {BAT_OVP, sc->cfg->bat_ovp},
        {CHG_OCP_DIS, sc->cfg->chg_ocp_disable},
        {CHG_OCP, sc->cfg->chg_ocp},
        {DSG_OCP_DIS, sc->cfg->dsg_ocp_disable},
        {DSG_OCP, sc->cfg->dsg_ocp},
        {TDIE_FLT_DIS, sc->cfg->tdie_flt_disable},
        {TDIE_ALRM_DIS, sc->cfg->tdie_alm_disable},
        {TDIE_ALRM, sc->cfg->tdie_alm},
    };

    ret = sc760x_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = sc760x_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    ret = sc760x_set_adc_enable(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
    }

    ret = sc760x_set_adc_done_mask(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to set adc mask (%d)\n", __func__, ret);
    }

    return sc760x_dump_reg(sc);
}

static int sc760x_register_interrupt(struct sc760x_chip *sc, struct i2c_client *client)
{
    int ret = 0;

    ret = devm_request_threaded_irq(sc->dev, client->irq, NULL,
                    sc760x_irq_handler,
                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                    sc760x_irq_name[sc->role], sc);
    if (ret < 0) {
        dev_err(sc->dev, "request thread irq failed:%d\n", ret);
        return ret;
    }

    disable_irq_wake(client->irq);
    disable_irq(client->irq);
    sc->irq_enabled = false;
    sc->irq = client->irq;
    dev_err(sc->dev, "request thread irq success\n");
    return 0;
}

static int is_wls_online(struct sc760x_chip *sc)
{
	int rc;
	union power_supply_propval val;

	if (!sc->use_ext_wls_psy || !sc->wls_psy)
		return 0;

	rc = power_supply_get_property(sc->wls_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		pr_err("Error wls online rc = %d\n", rc);
		return 0;
	}

	pr_debug("wireless online is %d", val.intval);
	return val.intval;
}

static int is_usb_online(struct sc760x_chip *sc)
{
	int rc;
	union power_supply_propval val;

	if (!sc->use_ext_usb_psy || !sc->usb_psy)
		return 0;

	rc = power_supply_get_property(sc->usb_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		pr_err("Error usb online rc = %d\n", rc);
		return 0;
	}

	pr_debug("usb online is %d", val.intval);
	return val.intval;
}

static int sc760x_get_state(struct sc760x_chip *sc,
			     struct sc760x_state *state)
{
	int ret = 0, chrg_status = 0;
	union power_supply_propval val = {0};

	if (sc->use_ext_usb_psy && sc->usb_psy) {

		ret = power_supply_get_property(sc->usb_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		if (!ret)
			state->vbus_adc = val.intval;
		ret = power_supply_get_property(sc->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &val);
		if (!ret)
			state->ibus_adc = val.intval;
		ret = power_supply_get_property(sc->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &val);
		if (!ret) {
			state->usb_online = val.intval;
			state->online = val.intval;
		}
		ret = power_supply_get_property(sc->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
		if (!ret)
			state->ibus_limit = val.intval;

		ret = power_supply_get_property(sc->usb_psy,
				POWER_SUPPLY_PROP_USB_TYPE, &val);
		if (!ret)
			state->chrg_type = val.intval;
	} else if (sc->use_ext_usb_psy) {
		state->chrg_stat = DISCHARGING;
		state->online = 0;
		state->ibus_limit = 0;
		state->chrg_type = 0;
	}

	if (!state->online && is_wls_online(sc)) {
		state->online = 1;
		state->chrg_type = 3;
		state->chrg_stat = CHARGING;
	}

	sc760x_get_adc(sc, ADC_IBAT, &state->ibat_adc);
	sc760x_get_work_mode(sc, &chrg_status);

	if (state->ibat_adc > 10 && chrg_status)
		state->chrg_stat = CHARGING;
	else if (state->online)
		state->chrg_stat = NOT_CHARGING;
	else
		state->chrg_stat = DISCHARGING;

	sc760x_get_adc(sc, ADC_IBAT, &state->ibat_adc);
	sc760x_get_adc(sc, ADC_VBAT, &state->vbat_adc);
	sc760x_get_adc(sc, ADC_VCHG, &state->vchg_adc);
	sc760x_get_adc(sc, ADC_TBAT, &state->tbat_adc);
	sc760x_get_adc(sc, ADC_TDIE, &state->tdie_adc);
	state->ibat_adc = state->ibat_adc * 1000;
	state->vbat_adc = state->vbat_adc * 1000;
	state->vchg_adc = state->vchg_adc * 1000;
	return 0;
}

static bool sc760x_state_changed(struct sc760x_chip *sc,
				  struct sc760x_state *new_state)
{
	struct sc760x_state old_state;

	mutex_lock(&sc->lock);
	old_state = sc->state;
	mutex_unlock(&sc->lock);

	return (old_state.chrg_stat != new_state->chrg_stat ||
		old_state.online != new_state->online ||
		old_state.chrg_type != new_state->chrg_type ||
		old_state.ibus_limit != new_state->ibus_limit
		);
}

static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	bool state_changed = false;
	struct sc760x_state state;
	struct sc760x_chip *sc = container_of(work,
					struct sc760x_chip,
					charge_monitor_work.work);

	ret = sc760x_get_state(sc, &state);
	state_changed = sc760x_state_changed(sc, &state);
	mutex_lock(&sc->lock);
	sc->state = state;
	mutex_unlock(&sc->lock);

	if (state_changed) {
		if (!sc->state.chrg_type)
			pr_info("No charger is present\n");
		sc760x_dump_reg(sc);
		schedule_delayed_work(&sc->charge_detect_delayed_work, 0);
		power_supply_changed(sc->charger_psy);
	}

	schedule_delayed_work(&sc->charge_monitor_work, 10*HZ);
}

static void charger_detect_work_func(struct work_struct *work)
{
	int ret;
	bool state_changed = false;
	struct sc760x_state state;
	struct sc760x_chip *sc = container_of(work,
					struct sc760x_chip,
					charge_detect_delayed_work.work);

	if (!sc->charger_wakelock->active)
		__pm_stay_awake(sc->charger_wakelock);

	if (sc->use_ext_usb_psy) {
		if (!sc->usb_psy) {
			const char *usb_psy_name = NULL;
			ret = device_property_read_string(sc->dev,
						"mmi,ext-usb-psy-name",
						&usb_psy_name);
			if (!ret && usb_psy_name)
				sc->usb_psy = power_supply_get_by_name(usb_psy_name);
			if (!sc->usb_psy) {
				pr_info("No USB power supply found, redetecting...\n");
				cancel_delayed_work(&sc->charge_detect_delayed_work);
				schedule_delayed_work(&sc->charge_detect_delayed_work,
							msecs_to_jiffies(1000));
			       goto err;
			} else {
				pr_info("USB power supply is found\n");
			}
		}
	}

	if (sc->use_ext_wls_psy) {
		if (!sc->wls_psy) {
			const char *wls_psy_name = NULL;
			ret = device_property_read_string(sc->dev,
						"mmi,ext-wls-psy-name",
						&wls_psy_name);
			if (!ret && wls_psy_name)
				sc->wls_psy = power_supply_get_by_name(wls_psy_name);
			if (!sc->wls_psy) {
				pr_info("No Wireless power supply found, redetecting...\n");
				cancel_delayed_work(&sc->charge_detect_delayed_work);
				schedule_delayed_work(&sc->charge_detect_delayed_work,
							msecs_to_jiffies(1000));
			       goto err;
			} else {
				pr_info("WLS power supply is found\n");
			}
		}
	}

	if (sc->mmi_charger &&
	    (!sc->mmi_charger->fg_psy || !sc->mmi_charger->driver)) {
		sc760x_mmi_charger_init(sc->mmi_charger);
	}

	ret = sc760x_get_state(sc, &state);
	state_changed = sc760x_state_changed(sc, &state);
	mutex_lock(&sc->lock);
	sc->state = state;
	mutex_unlock(&sc->lock);

	if(!state.online)
	{
		dev_err(sc->dev, "Vbus not online\n");
		goto err;
	}

	if (state_changed) {
		sc760x_dump_reg(sc);
		power_supply_changed(sc->charger_psy);
	}
	return;
err:
	//release wakelock
	if (state_changed)
		power_supply_changed(sc->charger_psy);
	dev_info(sc->dev, "Relax wakelock\n");
	__pm_relax(sc->charger_wakelock);
	return;
}


static int sc760x_psy_notifier_call(struct notifier_block *nb,
				unsigned long val, void *v)
{
	struct sc760x_chip *sc = container_of(nb,
				struct sc760x_chip, psy_nb);
	struct power_supply *psy = v;
	struct sc760x_mmi_charger *chg = NULL;

	if (!sc) {
		pr_err("called before sc760x valid!\n");
		return NOTIFY_DONE;
	}

	pr_err("psy notifier call , val %d\n", (int)val);
	if (!psy || val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	chg = sc->mmi_charger;
	if ((sc->usb_psy && !strcmp(psy->desc->name, sc->usb_psy->desc->name)) ||
	    (chg && chg->fg_psy_name && !strcmp(psy->desc->name, chg->fg_psy_name))) {
		cancel_delayed_work(&sc->charge_detect_delayed_work);
		schedule_delayed_work(&sc->charge_detect_delayed_work,
						msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

#define EMPTY_BATTERY_VBAT 3200
static int sc760x_charger_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
	int rc;
	struct sc760x_mmi_charger *chg = data;
	union power_supply_propval val = {0};

	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!rc)
		chg->batt_info.batt_mv = val.intval / 1000;
	rc = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!rc)
		chg->batt_info.batt_ma = val.intval / 1000;
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

	if (!chg->sc->state.online) {
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

static int sc760x_charger_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
	int ret = 0, work_mode = 0;
	struct sc760x_mmi_charger *chg = data;
	struct sc760x_state state = chg->sc->state;

	ret = sc760x_get_state(chg->sc, &state);
	sc760x_get_work_mode(chg->sc, &work_mode);
	chg->chg_info.chrg_mv = state.vbus_adc / 1000;
	chg->chg_info.chrg_ma = state.ibus_adc / 1000;
	chg->chg_info.chrg_type = get_charger_type(chg->sc);
	if (chg->chg_info.chrg_type == POWER_SUPPLY_USB_TYPE_UNKNOWN &&
			is_wls_online(chg->sc)) {
		chg->chg_info.chrg_type = 0xFF;
	}

	chg->chg_info.chrg_present = state.online;
	chg->chg_info.vbus_present = state.online;
	chg->chg_info.chrg_pmax_mw = 0;

	memcpy(chg_info, &chg->chg_info, sizeof(struct mmi_charger_info));

	pr_info("sc760x_chg_info : gpio_en %d, workmode %d, chrg_stat =%d, usb_online = %d, wls_online %d, vbus = %d, ibus = %d, ibus_limit = %d, ibat = %d, vbat = %d, vchg = %d, tbat = %d\n",
		chg->sc->sc760x_enable, work_mode, state.chrg_stat, state.usb_online, state.wls_online, state.vbus_adc, state.ibus_adc, state.ibus_limit, state.ibat_adc, state.vbat_adc, state.vchg_adc,
		state.tbat_adc);
	sc760x_dump_reg(chg->sc);
	return 0;
}

static int sc760x_charger_config_charge(void *data, struct mmi_charger_cfg *config)
{
	int rc = 0, sc760x_ibat_limit_set = 0, ibat_limit_vote = 0;
	u32 value;
	bool chg_en;
	bool cfg_changed = false;
	
	struct sc760x_mmi_charger *chg = data;
	struct sc760x_state state = chg->sc->state;

	sc760x_get_state(chg->sc, &state);
	/* Monitor vbat and ibat for OVP and OCP */
/*
	if (chg->batt_info.batt_mv >= chg->sc->vbat_ovp_threshold) {
		pr_err("ERROR: vbat OVP is triggered, batt_mv=%dmV, thre=%d\n",
				chg->batt_info.batt_mv,
				chg->sc->vbat_ovp_threshold);
	}
*/

	rc = sc760x_get_ibat_limit(chg->sc, &sc760x_ibat_limit_set);
	if (rc < 0)
		sc760x_ibat_limit_set = ibat_limit_vote;

	if (!config->charging_disable &&
	    !config->charger_suspend &&
	    config->target_fcc != chg->chg_cfg.target_fcc) {
		cfg_changed = true;
		chg->chg_cfg.target_fcc = config->target_fcc;
	}

	/* configure the charger if changed */
	if (config->target_fv != chg->chg_cfg.target_fv) {
		cfg_changed = true;
		chg->chg_cfg.target_fv = config->target_fv;
	}

	if (chg->chg_cfg.target_fv > 0 && (state.vbat_adc > chg->chg_cfg.target_fv * 1000)) {
		sc760x_ibat_limit_set -= IBAT_CHG_LIM_BASE;
		sc760x_set_ibat_limit(chg->sc, sc760x_ibat_limit_set);
		pr_info("update new sc760x_ibat_limit_set %d, state.vbat_adc %d > chg->chg_cfg.target_fv %d\n",
			sc760x_ibat_limit_set, state.vbat_adc, chg->chg_cfg.target_fv * 1000);
	} else {

		ibat_limit_vote = MIN_VAL(chg->chg_cfg.target_fcc, chg->sc->thermal_fcc_ua / 1000);

#ifdef THERMAL_RATIO_CONTROL
		ibat_limit_vote = MIN_VAL(chg->chg_cfg.target_fcc, chg->paired_batt_info.batt_fcc_ma / chg->sc->thermal_ratio);
		chg->sc->thermal_fcc_ua = (chg->paired_batt_info.batt_fcc_ma / chg->sc->thermal_ratio) * 1000;
#endif

		if (sc760x_ibat_limit_set > (ibat_limit_vote + IBAT_CHG_LIM_BASE)) {
			sc760x_ibat_limit_set -= IBAT_CHG_LIM_BASE;
			sc760x_set_ibat_limit(chg->sc, sc760x_ibat_limit_set);
			pr_info("Devide to decrease ichg, update new sc760x_ibat_limit_set %d\n",
				sc760x_ibat_limit_set);
		} else if (sc760x_ibat_limit_set < (ibat_limit_vote - IBAT_CHG_LIM_BASE)) {
			sc760x_ibat_limit_set += IBAT_CHG_LIM_BASE;
			sc760x_set_ibat_limit(chg->sc, sc760x_ibat_limit_set);
			pr_info("Devide to increase ichg, update new sc760x_ibat_limit_set %d\n",
				sc760x_ibat_limit_set);
		}
	}

	if (config->charger_suspend != chg->chg_cfg.charger_suspend) {
		rc = sc760x_set_load_switch(chg->sc, config->charger_suspend);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.charger_suspend = config->charger_suspend;
		}
	}

	chg_en = sc760x_is_enabled_charging(chg->sc);
	if (chg->sc->user_chg_en >= 0 && chg_en != !!chg->sc->user_chg_en) {
		if (!!chg->sc->user_chg_en)
			rc = sc760x_enable_charger(chg->sc);
		else
			rc = sc760x_disable_charger(chg->sc);
		if (!rc) {
			cfg_changed = true;
			chg_en = !!chg->sc->user_chg_en;
		}
	}

	if (config->charging_disable != chg->chg_cfg.charging_disable ||
	    (chg_en == config->charging_disable && chg->sc->user_chg_en < 0)) {
		value = config->charging_disable;
		if (!value)
			rc = sc760x_enable_charger(chg->sc);
		else
			rc = sc760x_disable_charger(chg->sc);
		if (!rc) {
			cfg_changed = true;
			chg->chg_cfg.charging_disable = config->charging_disable;
			chg_en = !value;
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
			chg->chg_cfg.target_fv = -EINVAL;
			chg->chg_cfg.target_fcc = -EINVAL;
			chg->chg_cfg.charging_disable = true;
			pr_info("Battery charging reset triggered\n");
		}
		cfg_changed = true;
		chg->chg_cfg.charging_reset = config->charging_reset;
	}

	if (cfg_changed) {
		cancel_delayed_work(&chg->sc->charge_monitor_work);
		schedule_delayed_work(&chg->sc->charge_monitor_work,
						msecs_to_jiffies(200));
	}

	if (!is_usb_online(chg->sc) &&
		!is_wls_online(chg->sc)) {

	    rc = sc760x_set_auto_bsm_dis(chg->sc, false);
	    if (rc < 0) {
	        pr_err("Failed to enable audo bsm(%d)\n", rc);
	    }
	} else {

	    rc = sc760x_set_auto_bsm_dis(chg->sc, true);
	    if (rc < 0) {
	        pr_err("Failed to disable audo bsm(%d)\n", rc);
	    }
	}

	pr_info("chg_en:%d, online %d, chg_st:%d\n",
			chg_en, state.online, state.chrg_stat);
	sc760x_get_ibat_limit(chg->sc, &sc760x_ibat_limit_set);
	pr_info("sc760x_ibat_limit_set %d, ibat_limit_vote %d, target_fcc %d, target_fv %d, thermal_fcc_ua %d\n",
			sc760x_ibat_limit_set, ibat_limit_vote, chg->chg_cfg.target_fcc, chg->chg_cfg.target_fv, chg->sc->thermal_fcc_ua);
	return 0;
}

#define TAPER_COUNT 3
static bool sc760x_charger_is_charge_tapered(void *data, int tapered_ma)
{
	bool is_tapered = false;
	struct sc760x_mmi_charger *chg = data;

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

static bool sc760x_charger_is_charge_halt(void *data)
{
//	int rc = 0;
//	int chrg_stat = 0;
	bool chrg_halt = false;
	struct sc760x_mmi_charger *chg = data;

	if (chg->batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
	    chg->batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		chrg_halt = true;

	return chrg_halt;
}

static void sc760x_charger_set_constraint(void *data,
                        struct mmi_charger_constraint *constraint)
{
	struct sc760x_mmi_charger *chg = data;

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

#define DUAL_VBATT_DELTA_MV 200
static void sc760x_paired_battery_notify(void *data,
			struct mmi_battery_info *batt_info)
{
	int partner_fg_vbatt = 0, fg_vbatt = 0;
	struct sc760x_mmi_charger *chg = data;

	if (!chg) {
		pr_err("sc mmi_charger not valid\n");
		return;
	}

	if (!chg->sc) {
		pr_err("sc chip not valid\n");
		return;
	}

	memcpy(&chg->paired_batt_info, batt_info, sizeof(struct mmi_battery_info));
	if (chg->sc->sc760x_enable) {
		pr_info("sc760x has been power on\n");
		return;
	}

	pr_info("parnter battery : batt_mv %d, batt_ma %d, batt_soc %d,"
		" batt_temp %d, batt_status %d, batt_sn %s, batt_fv_mv %d,"
		" batt_fcc_ma %d\n",
		batt_info->batt_mv,
		batt_info->batt_ma,
		batt_info->batt_soc,
		batt_info->batt_temp,
		batt_info->batt_status,
		batt_info->batt_sn,
		batt_info->batt_fv_mv,
		batt_info->batt_fcc_ma);

	partner_fg_vbatt = batt_info->batt_mv;
	fg_vbatt = chg->batt_info.batt_mv;

	pr_info("sc760x checking, partner_fg_vbatt %d, fg_vbatt %d\n", partner_fg_vbatt, fg_vbatt);
	if (!is_usb_online(chg->sc) &&
		!is_wls_online(chg->sc) &&
		(DUAL_VBATT_DELTA_MV < (partner_fg_vbatt - fg_vbatt))) {
		pr_err("partner_fg_vbatt > fg_vbatt, and delta value > %d, Does not turn on sc760x\n", DUAL_VBATT_DELTA_MV);
		return;
	} else {
		if (chg->sc->user_gpio_en < 0)
			sc760x_enable_chip(chg->sc, true);
		return;
	}

    return;
}

static int sc760x_mmi_charger_init(struct sc760x_mmi_charger *chg)
{
	int rc;
	const char *df_sn = NULL;
	struct mmi_charger_driver *driver;

	if (!chg->fg_psy && !chg->fg_psy_name) {
		chg->fg_psy_name = "fg_battery";
		device_property_read_string(chg->sc->dev,
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
		pr_info("sc760x_mmi_charger has already registered\n");
		return 0;
	}

	rc = device_property_read_string(chg->sc->dev,
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
	chg->chg_cfg.target_fcc = chg->sc->init_data.ichg / 1000;
	chg->chg_cfg.charging_disable = chg->sc->init_data.charger_disabled;
	chg->chg_cfg.charger_suspend = chg->sc->init_data.charger_disabled;

	driver = devm_kzalloc(chg->sc->dev,
				sizeof(struct mmi_charger_driver),
				GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	/* init driver */
	driver->name = SC760X_NAME;
	driver->dev = chg->sc->dev;
	driver->data = chg;
	driver->get_batt_info = sc760x_charger_get_batt_info;
	driver->get_chg_info = sc760x_charger_get_chg_info;
	driver->config_charge = sc760x_charger_config_charge;
	driver->is_charge_tapered = sc760x_charger_is_charge_tapered;
	driver->is_charge_halt = sc760x_charger_is_charge_halt;
	driver->set_constraint = sc760x_charger_set_constraint;
	driver->notify_paired_battery = sc760x_paired_battery_notify;
	chg->driver = driver;

	/* register driver to mmi charger */
	rc = mmi_register_charger_driver(driver);
	if (rc) {
		pr_err("sc760x_mmi_charger init failed, rc=%d\n", rc);
	} else {
		pr_info("sc760x_mmi_charger init successfully\n");
	}

	return rc;
}

static void sc760x_mmi_charger_deinit(struct sc760x_mmi_charger *chg)
{
	int rc;

	if (!chg->driver) {
		pr_info("sc760x_mmi_charger has not inited yet\n");
		return;
	}

	/* unregister driver from mmi charger */
	rc = mmi_unregister_charger_driver(chg->driver);
	if (rc) {
		pr_err("sc760x_mmi_charger deinit failed, rc=%d\n", rc);
	} else {
		devm_kfree(chg->sc->dev, chg->driver);
		chg->driver = NULL;
	}

	if (chg->fg_psy) {
		power_supply_put(chg->fg_psy);
		chg->fg_psy = NULL;
	}
}

static struct of_device_id sc760x_charger_match_table[] = {
    {   .compatible = "southchip,sc7603_master",
        .data = &sc760x_role_data[SC760X_MASTER], },
    {   .compatible = "southchip,sc7603_slave",
        .data = &sc760x_role_data[SC760X_SLAVE], },
    { },
};
MODULE_DEVICE_TABLE(of, sc760x_charger_match_table);

static int sc760x_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc760x_chip *sc;
    struct device *dev = &client->dev;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    char *name = NULL;
    int ret = 0, i =0;

    pr_err("%s (%s)\n", __func__, SC760X_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc760x_chip), GFP_KERNEL);
    if (!sc)
        return -ENOMEM;

    sc->dev = dev;
    sc->client = client;
    mutex_init(&sc->lock);

    sc->user_ilim = -EINVAL;
    sc->user_ichg = -EINVAL;
    sc->user_chg_en = -EINVAL;
    sc->user_chg_susp = -EINVAL;
    sc->user_gpio_en = -EINVAL;
    sc->sc760x_enable = 0;


    sc->regmap = devm_regmap_init_i2c(client,
                            &sc760x_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev, "Failed to initialize regmap\n");
        return -EINVAL;
    }

    for (i = 0; i < ARRAY_SIZE(sc760x_reg_fields); i++) {
        const struct reg_field *reg_fields = sc760x_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev, "cannot allocate regmap field\n");
            return PTR_ERR(sc->rmap_fields[i]);
        }
    }

    i2c_set_clientdata(client, sc);
    sc760x_create_device_node(&(client->dev));

    match = of_match_node(sc760x_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev, "device tree match not found!\n");
        goto err_get_match;
    }

    sc->role = *(int *)match->data;

    dev_err(sc->dev, "sc760x[%s] probe running!!!\n",
            sc->role == SC760X_MASTER ? "master" : "slave");

    sc->cfg = &default_cfg;
    ret = sc760x_parse_dt(sc, &client->dev);
    if (ret < 0) {
        dev_err(sc->dev, "%s parse dt failed(%d)\n", __func__, ret);
        goto err_parse_dt;
    }

    ret = sc760x_register_interrupt(sc, client);
    if (ret < 0) {
        dev_err(sc->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

    name = devm_kasprintf(sc->dev, GFP_KERNEL, "%s",
		"sc760x suspend wakelock");
    sc->charger_wakelock =
		wakeup_source_register(NULL, name);

    ret = sc760x_power_supply_init(sc, dev);
    if (ret) {
        dev_err(dev, "Failed to register power supply, ret=%d\n", ret);
        goto err_init_device;
    }

    INIT_DELAYED_WORK(&sc->charge_detect_delayed_work, charger_detect_work_func);
    INIT_DELAYED_WORK(&sc->charge_monitor_work, charger_monitor_work_func);
    sc->psy_nb.notifier_call = sc760x_psy_notifier_call;
    ret = power_supply_reg_notifier(&sc->psy_nb);
    if (ret)
        pr_err("Failed to reg power supply notifier: %d\n", ret);

    sc->mmi_charger = devm_kzalloc(dev, sizeof(*sc->mmi_charger), GFP_KERNEL);
    if (sc->mmi_charger) {
        sc->mmi_charger->sc = sc;
        sc760x_mmi_charger_init(sc->mmi_charger);

    schedule_delayed_work(&sc->charge_detect_delayed_work,
						msecs_to_jiffies(0));
    }

    dev_err(sc->dev, "sc760x[%s] probe successfully!!!\n",
            sc->role == SC760X_MASTER ? "master" : "slave");
    return 0;

err_register_irq:
err_init_device:
err_parse_dt:
err_get_match:
    dev_err(sc->dev, "sc760x probe failed!\n");
    devm_kfree(sc->dev, sc);
    return ret;
}


static int sc760x_charger_remove(struct i2c_client *client)
{
    struct sc760x_chip *sc = i2c_get_clientdata(client);
    sc760x_remove_device_node(&(client->dev));

    cancel_delayed_work_sync(&sc->charge_monitor_work);

    if (sc->mmi_charger)
        sc760x_mmi_charger_deinit(sc->mmi_charger);

    power_supply_unreg_notifier(&sc->psy_nb);
    power_supply_unregister(sc->charger_psy);

    mutex_destroy(&sc->lock);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sc760x_suspend(struct device *dev)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    int ret = 0;
    dev_info(sc->dev, "Suspend successfully!");
    if (device_may_wakeup(dev))
        enable_irq_wake(sc->irq);
    disable_irq(sc->irq);
    ret = sc760x_set_adc_enable(sc, false);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
    }

    return 0;
}
static int sc760x_resume(struct device *dev)
{
    struct sc760x_chip *sc = dev_get_drvdata(dev);
    int ret = 0;
    dev_info(sc->dev, "Resume successfully!");
    if (device_may_wakeup(dev))
        disable_irq_wake(sc->irq);
    enable_irq(sc->irq);
    ret = sc760x_set_adc_enable(sc, true);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
    }

    return 0;
}

static const struct dev_pm_ops sc760x_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(sc760x_suspend, sc760x_resume)
};
#endif

static void sc760x_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sc760x_chip *sc = i2c_get_clientdata(client);
	ret = sc760x_set_adc_enable(sc, false);
	if (ret < 0) {
		dev_err(sc->dev, "%s Failed to enable adc(%d)\n", __func__, ret);
	}

	if (ret) {
		pr_err("Failed to disable charger, ret = %d\n", ret);
	}
	pr_info("sc760x_charger_shutdown\n");
}

static const struct i2c_device_id sc760x_i2c_ids[] = {
	{ "sc7603_master", SC760X_MASTER },
	{ "sc7603_slave", SC760X_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc760x_i2c_ids);

static struct i2c_driver sc760x_charger_driver = {
    .driver     = {
        .name   = "sc760x-charger",
        .owner  = THIS_MODULE,
        .of_match_table = sc760x_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc760x_pm,
#endif
    },
    .probe      = sc760x_charger_probe,
    .remove     = sc760x_charger_remove,
    .shutdown = sc760x_charger_shutdown,
    .id_table = sc760x_i2c_ids,
};

module_i2c_driver(sc760x_charger_driver);

MODULE_DESCRIPTION("SC SC760X Driver");
MODULE_LICENSE("GPL v2");

