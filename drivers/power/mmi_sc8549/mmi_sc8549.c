/*
 * SC8549 battery charging driver
 *
 * Copyright (C) 2021 South Chip *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"[sc8549] %s: " fmt, __func__

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
#include "mmi_sc8549.h"
#include <linux/gpio.h>

enum {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

/*below used for comm with other module*/
#define	BAT_OVP_FAULT_SHIFT			8
#define	BAT_OCP_FAULT_SHIFT			9
#define	BUS_OVP_FAULT_SHIFT			10
#define	BUS_OCP_FAULT_SHIFT			11
#define	SS_TIMEOUT_FAULT_SHIFT			16
#define	TS_SHUT_FAULT_SHIFT			17

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	SS_TIMEOUT_FAULT_MASK	(1 << SS_TIMEOUT_FAULT_SHIFT)
#define	TS_SHUT_FAULT_MASK		(1 << TS_SHUT_FAULT_SHIFT)

#define sc_err(fmt, ...)								\
do {											\
	printk(KERN_ERR "[CP]:%s:" \
		fmt,  __func__, ##__VA_ARGS__);\
} while(0);

#define sc_info(fmt, ...)								\
do {											\
	printk(KERN_INFO "[CP]:%s:" \
		fmt,  __func__, ##__VA_ARGS__);\
} while(0);

#define sc_debug(fmt, ...)								\
do {											\
	printk(KERN_DEBUG "[CP]:%s:" \
		fmt,  __func__, ##__VA_ARGS__);\
} while(0);


struct sc8549_cfg {
	bool bat_ovp_disable;
	bool bat_ocp_disable;

	int bat_ovp_th;
	int bat_ocp_th;

	bool bus_ocp_disable;

	int bus_ovp_th;
	int bus_ocp_th;

	int ac_ovp_th;

	int sense_r_mohm;
};

struct sc8549 {
	struct device *dev;
	struct i2c_client *client;

	int device_id;
	int part_no;
	int revision;
	int int_gpio;
	int gpio_irq;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;
	struct regmap *regmap;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	int irq_counts;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled;	/* Register bit status */

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int die_temp;

	/* alarm/fault status */
	bool ac_ovp_fault;
	bool drop_ovp_fault;
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;
	bool ss_timeout_fault;
	bool ts_shut_fault;

	bool vbat_reg;
	bool ibat_reg;

	int  prev_fault;
	int  prev_fault2;


	int chg_ma;
	int chg_mv;

	int charge_state;

	struct sc8549_cfg *cfg;

	int skip_writes;
	int skip_reads;

	struct sc8549_platform_data *platform_data;

	struct delayed_work monitor_work;

	struct dentry *debug_root;

	struct power_supply *fc2_psy;
	struct pinctrl *irq_pinctrl;
};

static int __sc8549_i2c_read(struct i2c_client * client,u8 reg,void *buff,size_t size)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len  = 1,
			.buf  = &reg,
		},
		{
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.len  = size,
			.buf  = buff,
		}
	};

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));


	if(ret < 0){
		sc_err("i2c read fail: can't read from reg 0x%02X\n",reg);
		return -EIO;
	}

	return ret;
}

static int __sc8549_i2c_write(struct i2c_client * client,u8 reg,void *buff,size_t size)
{
	int ret;
       u8 buffer[5];
	struct i2c_msg msg;

	buffer[0] = reg;
	memcpy(&(buffer[1]), buff, size);
  	msg.addr = client->addr,
	msg.flags = client->flags,
	msg .len  = size + 1,
	msg.buf  = buffer,

	ret = i2c_transfer(client->adapter, &msg, 1);

	if(ret < 0){
		sc_err("i2c write fail: can't write from reg 0x%02X\n",reg);
		return -EIO;
	}

	return ret;
}
/************************************************************************/
static int __sc8549_read_byte(struct sc8549 *sc, u8 reg, u8 *data)
{
	s32 ret;

	ret = __sc8549_i2c_read(sc->client, reg, data, 1);
	if (ret < 0) {
		sc_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	return 0;
}

static int __sc8549_write_byte(struct sc8549 *sc, int reg, u8 val)
{
	s32 ret;

	ret = __sc8549_i2c_write(sc->client, reg, &val, 1);
	if (ret < 0) {
		sc_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int __sc8549_read_word(struct sc8549 *sc, u8 reg, u16 *data)
{
	s32 ret;

	ret = __sc8549_i2c_read(sc->client, reg, data, 2);
	if (ret < 0) {
		sc_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	return 0;
}

static int sc8549_read_byte(struct sc8549 *sc, u8 reg, u8 *data)
{
	int ret;

	if (sc->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8549_read_byte(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	return ret;
}

static int sc8549_write_byte(struct sc8549 *sc, u8 reg, u8 data)
{
	int ret;

	if (sc->skip_writes)
		return 0;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8549_write_byte(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	return ret;
}

static int sc8549_read_word(struct sc8549 *sc, u8 reg, u16 *data)
{
	int ret;

	if (sc->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8549_read_word(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	return ret;
}

static int sc8549_update_bits(struct sc8549*sc, u8 reg,
				    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (sc->skip_reads || sc->skip_writes)
		return 0;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8549_read_byte(sc, reg, &tmp);
	if (ret) {
		sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8549_write_byte(sc, reg, tmp);
	if (ret)
		sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}

/*********************************************************************/

static int sc8549_enable_charge(struct sc8549 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8549_CHG_ENABLE;
	else
		val = SC8549_CHG_DISABLE;

	val <<= SC8549_CHG_EN_SHIFT;

	sc_err("sc8549 charger %s\n", enable == false ? "disable" : "enable");
	ret = sc8549_update_bits(sc, SC8549_REG_07,
				SC8549_CHG_EN_MASK, val);

	return ret;
}

static int sc8549_check_charge_enabled(struct sc8549 *sc, bool *enabled)
{
	int ret;
	u8 val;

	ret = sc8549_read_byte(sc, SC8549_REG_07, &val);
	sc_info(">>>reg [0x0c] = 0x%02x\n", val);
	if (!ret)
		*enabled = !!(val & SC8549_CHG_EN_MASK);
	return ret;
}

static int sc8549_set_wdt(struct sc8549 *sc, int ms)
{
	int ret;
	u8 val;

	if (ms == 0)
		val = SC8549_WATCHDOG_DISABLE;
	if (ms == 200)
		val = SC8549_WATCHDOG_0P2S;
	else if (ms == 500)
		val = SC8549_WATCHDOG_0P5S;
	else if (ms == 1000)
		val = SC8549_WATCHDOG_1S;
	else if (ms == 5000)
		val = SC8549_WATCHDOG_5S;
	else
		val = SC8549_WATCHDOG_30S;

	val <<= SC8549_WATCHDOG_TIMEOUT_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_09,
				SC8549_WATCHDOG_TIMEOUT_MASK, val);
	return ret;
}

static int sc8549_enable_batovp(struct sc8549 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8549_BAT_OVP_ENABLE;
	else
		val = SC8549_BAT_OVP_DISABLE;

	val <<= SC8549_BAT_OVP_DIS_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_00,
				SC8549_BAT_OVP_DIS_MASK, val);
	return ret;
}

static int sc8549_set_batovp_th(struct sc8549 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8549_BAT_OVP_BASE)
		threshold = SC8549_BAT_OVP_BASE;

	val = (threshold - SC8549_BAT_OVP_BASE) / SC8549_BAT_OVP_LSB;

	val <<= SC8549_BAT_OVP_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_00,
				SC8549_BAT_OVP_MASK, val);
	return ret;
}

static int sc8549_enable_batocp(struct sc8549 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8549_BAT_OCP_ENABLE;
	else
		val = SC8549_BAT_OCP_DISABLE;

	val <<= SC8549_BAT_OCP_DIS_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_01,
				SC8549_BAT_OCP_DIS_MASK, val);
	return ret;
}

static int sc8549_set_batocp_th(struct sc8549 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8549_BAT_OCP_BASE)
		threshold = SC8549_BAT_OCP_BASE;

	val = (threshold - SC8549_BAT_OCP_BASE) / SC8549_BAT_OCP_LSB;

	val <<= SC8549_BAT_OCP_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_01,
				SC8549_BAT_OCP_MASK, val);
	return ret;
}

static int sc8549_set_busovp_th(struct sc8549 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8549_BUS_OVP_BASE)
		threshold = SC8549_BUS_OVP_BASE;

	val = (threshold - SC8549_BUS_OVP_BASE) / SC8549_BUS_OVP_LSB;

	val <<= SC8549_BUS_OVP_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_04,
				SC8549_BUS_OVP_MASK, val);
	return ret;
}

static int sc8549_enable_busocp(struct sc8549 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8549_BUS_OCP_ENABLE;
	else
		val = SC8549_BUS_OCP_DISABLE;

	val <<= SC8549_BUS_OCP_DIS_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_05,
				SC8549_BUS_OCP_DIS_MASK, val);
	return ret;
}

static int sc8549_set_busocp_th(struct sc8549 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8549_BUS_OCP_BASE)
		threshold = SC8549_BUS_OCP_BASE;

	val = (threshold - SC8549_BUS_OCP_BASE) / SC8549_BUS_OCP_LSB;

	val <<= SC8549_BUS_OCP_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_05,
				SC8549_BUS_OCP_MASK, val);
	return ret;
}

static int sc8549_set_acovp_th(struct sc8549 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8549_AC_OVP_BASE)
		threshold = SC8549_AC_OVP_BASE;

	if (threshold == SC8549_AC_OVP_6P5V)
		val = 0x07;
	else
		val = (threshold - SC8549_AC_OVP_BASE) /  SC8549_AC_OVP_LSB;

	val <<= SC8549_AC_OVP_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_02,
				SC8549_AC_OVP_MASK, val);

	return ret;

}

static int sc8549_set_vdrop_th(struct sc8549 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold == 300)
		val = SC8549_VDROP_THRESHOLD_300MV;
	else
		val = SC8549_VDROP_THRESHOLD_400MV;

	val <<= SC8549_VDROP_THRESHOLD_SET_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_03,
				SC8549_VDROP_THRESHOLD_SET_MASK,
				val);

	return ret;
}

static int sc8549_set_vdrop_deglitch(struct sc8549 *sc, int us)
{
	int ret;
	u8 val;

	if (us == 8)
		val = SC8549_VDROP_DEGLITCH_8US;
	else
		val = SC8549_VDROP_DEGLITCH_5MS;

	val <<= SC8549_VDROP_DEGLITCH_SET_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_03,
				SC8549_VDROP_DEGLITCH_SET_MASK,
				val);
	return ret;
}

static int sc8549_enable_adc(struct sc8549 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8549_ADC_ENABLE;
	else
		val = SC8549_ADC_DISABLE;

	val <<= SC8549_ADC_EN_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_11,
				SC8549_ADC_EN_MASK, val);
	return ret;
}

static int sc8549_set_adc_scanrate(struct sc8549 *sc, bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = SC8549_ADC_RATE_ONESHOT;
	else
		val = SC8549_ADC_RATE_CONTINOUS;

	val <<= SC8549_ADC_RATE_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_11,
				SC8549_ADC_EN_MASK, val);
	return ret;
}

#define ADC_REG_BASE SC8549_REG_13
static int sc8549_get_adc_data(struct sc8549 *sc, int channel,  int *result)
{
	int ret;
	u16 val;
	s16 t;
	int tmp;

	if(channel >= ADC_MAX_NUM) return 0;

	ret = sc8549_read_word(sc, ADC_REG_BASE + (channel << 1), &val);

	if (ret < 0)
		return ret;

	t = val & 0xFF;
	t <<= 8;
	t |= (val >> 8) & 0xFF;

    switch (channel) {
        case ADC_IBUS:
            tmp = t * SC8549_IBUS_ADC_LSB_MUL;
			t = tmp / SC8549_IBUS_ADC_LSB_DIV;
        break;
        case ADC_VBUS:
			tmp = t * SC8549_VBUS_ADC_LSB_MUL;
			t = tmp / SC8549_VBUS_ADC_LSB_DIV;
        break;
        case ADC_VAC:
            t = t * SC8549_VAC_ADC_LSB;
        break;
        case ADC_VOUT:
			tmp = t * SC8549_VOUT_ADC_LSB_MUL;
			t = tmp / SC8549_VOUT_ADC_LSB_DIV;
        break;
        case ADC_VBAT:
		tmp = t * SC8549_VBAT_ADC_LSB_MUL;
			t = tmp / SC8549_VBAT_ADC_LSB_DIV;
        break;
        case ADC_TDIE:
		tmp = t * SC8549_TDIE_ADC_LSB_MUL;
			t = tmp / SC8549_TDIE_ADC_LSB_DIV;
        break;
        default:
            t = 0;
        break;
	}

	*result = t;

	return 0;
}

static int sc8549_set_adc_scan(struct sc8549 *sc, int channel, bool enable)
{
	int ret;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

    switch (channel) {
        case ADC_IBUS:
            shift = SC8549_IBUS_ADC_DIS_SHIFT;
            mask = SC8549_IBUS_ADC_DIS_MASK;
        break;
        case ADC_VBUS:
            shift = SC8549_VBUS_ADC_DIS_SHIFT;
            mask = SC8549_VBUS_ADC_DIS_MASK;
        break;
        case ADC_VAC:
            shift = SC8549_VAC_ADC_DIS_SHIFT;
            mask = SC8549_VAC_ADC_DIS_MASK;
        break;
        case ADC_VOUT:
            shift = SC8549_VOUT_ADC_DIS_SHIFT;
            mask = SC8549_VOUT_ADC_DIS_MASK;
        break;
        case ADC_VBAT:
            shift = SC8549_VBAT_ADC_DIS_SHIFT;
            mask = SC8549_VBAT_ADC_DIS_MASK;
        break;
        case ADC_TDIE:
            shift = SC8549_TDIE_ADC_DIS_SHIFT;
            mask = SC8549_TDIE_ADC_DIS_MASK;
        break;
        default:
            return -EINVAL;
        break;
	}

	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;

	ret = sc8549_update_bits(sc, SC8549_REG_15, mask, val);

	return ret;
}


static int sc8549_set_sense_resistor(struct sc8549 *sc, int r_mohm)
{
	int ret;
	u8 val;

	if (r_mohm == 2)
		val = SC8549_SET_IBAT_SNS_RES_2MHM;
	else if (r_mohm == 5)
		val = SC8549_SET_IBAT_SNS_RES_5MHM;
	else
		return -EINVAL;

	val <<= SC8549_SET_IBAT_SNS_RES_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_08,
				SC8549_SET_IBAT_SNS_RES_MASK,
				val);
	return ret;
}

static int sc8549_set_ss_timeout(struct sc8549 *sc, int timeout)
{
	int ret;
	u8 val;

	switch (timeout) {
	case 0:
		val = SC8549_SS_TIMEOUT_DISABLE;
		break;
	case 40:
		val = SC8549_SS_TIMEOUT_40MS;
		break;
	case 80:
		val = SC8549_SS_TIMEOUT_80MS;
		break;
	case 320:
		val = SC8549_SS_TIMEOUT_320MS;
		break;
	case 1280:
		val = SC8549_SS_TIMEOUT_1280MS;
		break;
	case 5120:
		val = SC8549_SS_TIMEOUT_5120MS;
		break;
	case 20480:
		val = SC8549_SS_TIMEOUT_20480MS;
		break;
	default:
		val = SC8549_SS_TIMEOUT_81920MS;
		break;
	}

	val <<= SC8549_SS_TIMEOUT_SET_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_08,
				SC8549_SS_TIMEOUT_SET_MASK,
				val);

	return ret;
}

static int sc8549_set_ibat_reg_th(struct sc8549 *sc, int th_ma)
{
	int ret;
	u8 val;

	if (th_ma == 200)
		val = SC8549_IBAT_REG_200MA;
	else if (th_ma == 300)
		val = SC8549_IBAT_REG_300MA;
	else if (th_ma == 400)
		val = SC8549_IBAT_REG_400MA;
	else if (th_ma == 500)
		val = SC8549_IBAT_REG_500MA;
	else
		val = SC8549_IBAT_REG_500MA;

	val <<= SC8549_IBAT_REG_SHIFT;
	ret = sc8549_update_bits(sc, SC8549_REG_0B,
				SC8549_IBAT_REG_MASK,
				val);

	return ret;

}

static int sc8549_set_vbat_reg_th(struct sc8549 *sc, int th_mv)
{
	int ret;
	u8 val;

	if (th_mv == 50)
		val = SC8549_VBAT_REG_50MV;
	else if (th_mv == 100)
		val = SC8549_VBAT_REG_100MV;
	else if (th_mv == 150)
		val = SC8549_VBAT_REG_150MV;
	else
		val = SC8549_VBAT_REG_200MV;

	val <<= SC8549_VBAT_REG_SHIFT;

	ret = sc8549_update_bits(sc, SC8549_REG_0A,
				SC8549_VBAT_REG_MASK,
				val);

	return ret;
}

static int sc8549_register_reset(struct sc8549 *sc)
{
    int ret;
    u8 val;

    val = SC8549_REG_RST_ENABLE;

    val <<= SC8549_REG_RST_SHIFT;

    ret = sc8549_update_bits(sc, SC8549_REG_07,
				SC8549_REG_RST_MASK,
				val);
    return ret;
}



static int sc8549_detect_device(struct sc8549 *sc)
{
	int ret;
	u8 data;

	ret = sc8549_read_byte(sc, SC8549_REG_36, &data);
	if (ret == 0) {
		sc->part_no = data;
		sc_info("detect device PN:%x, ID:%x, REV:%x \n!",
				sc->part_no, sc->device_id, sc->revision);

		if (sc->part_no != SC8549_DEVICE_ID)
			return -EINVAL;
	}

	return ret;
}

static int sc8549_parse_dt(struct sc8549 *sc, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;

	sc->cfg = devm_kzalloc(dev, sizeof(struct sc8549_cfg),
					GFP_KERNEL);

	if (!sc->cfg)
		return -ENOMEM;

	sc->cfg->bat_ovp_disable = of_property_read_bool(np,
			"sc,sc8549,bat-ovp-disable");
	sc->cfg->bat_ocp_disable = of_property_read_bool(np,
			"sc,sc8549,bat-ocp-disable");
	sc->cfg->bus_ocp_disable = of_property_read_bool(np,
			"sc,sc8549,bus-ocp-disable");

	ret = of_property_read_u32(np, "sc,sc8549,bat-ovp-threshold",
			&sc->cfg->bat_ovp_th);
	if (ret) {
		sc_err("failed to read bat-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8549,bat-ocp-threshold",
			&sc->cfg->bat_ocp_th);
	if (ret) {
		sc_err("failed to read bat-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8549,bus-ovp-threshold",
			&sc->cfg->bus_ovp_th);
	if (ret) {
		sc_err("failed to read bus-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8549,bus-ocp-threshold",
			&sc->cfg->bus_ocp_th);
	if (ret) {
		sc_err("failed to read bus-ocp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8549,ac-ovp-threshold",
			&sc->cfg->ac_ovp_th);
	if (ret) {
		sc_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8549,sense-resistor-mohm",
			&sc->cfg->sense_r_mohm);
	if (ret) {
		sc_err("failed to read sense-resistor-mohm\n");
		return ret;
	}

	return 0;
}


static int sc8549_init_protection(struct sc8549 *sc)
{
	int ret;

	ret = sc8549_enable_batovp(sc, !sc->cfg->bat_ovp_disable);
	sc_info("%s bat ovp %s\n",
		sc->cfg->bat_ovp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8549_enable_batocp(sc, !sc->cfg->bat_ocp_disable);
	sc_info("%s bat ocp %s\n",
		sc->cfg->bat_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8549_enable_busocp(sc, !sc->cfg->bus_ocp_disable);
	sc_info("%s bus ocp %s\n",
		sc->cfg->bus_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8549_set_batovp_th(sc, sc->cfg->bat_ovp_th);
	sc_info("set bat ovp th %d %s\n", sc->cfg->bat_ovp_th,
		!ret ? "successfully" : "failed");

	ret = sc8549_set_batocp_th(sc, sc->cfg->bat_ocp_th);
	sc_info("set bat ocp threshold %d %s\n", sc->cfg->bat_ocp_th,
		!ret ? "successfully" : "failed");

	ret = sc8549_set_busovp_th(sc, sc->cfg->bus_ovp_th);
	sc_info("set bus ovp threshold %d %s\n", sc->cfg->bus_ovp_th,
		!ret ? "successfully" : "failed");

	ret = sc8549_set_busocp_th(sc, sc->cfg->bus_ocp_th);
	sc_info("set bus ocp threshold %d %s\n", sc->cfg->bus_ocp_th,
		!ret ? "successfully" : "failed");

	ret = sc8549_set_acovp_th(sc, sc->cfg->ac_ovp_th);
	sc_info("set ac ovp threshold %d %s\n", sc->cfg->ac_ovp_th,
		!ret ? "successfully" : "failed");

	return 0;
}

static int sc8549_init_int_src(struct sc8549 *sc)
{
	int ret;

	ret = sc8549_update_bits(sc, SC8549_REG_11,
				SC8549_ADC_DONE_MASK_MASK,
				1);

	return 0;
}

static int sc8549_init_adc(struct sc8549 *sc)
{
	sc8549_set_adc_scanrate(sc, false);
	sc8549_set_adc_scan(sc, ADC_IBUS, true);
	sc8549_set_adc_scan(sc, ADC_VBUS, true);
	sc8549_set_adc_scan(sc, ADC_VOUT, false);
	sc8549_set_adc_scan(sc, ADC_VBAT, true);
	sc8549_set_adc_scan(sc, ADC_IBAT, true);
	sc8549_set_adc_scan(sc, ADC_TDIE, true);
	sc8549_set_adc_scan(sc, ADC_VAC, true);

	sc8549_enable_adc(sc, true);

	return 0;
}

static int sc8549_init_regulation(struct sc8549 *sc)
{
	sc8549_set_ibat_reg_th(sc, 300);
	sc8549_set_vbat_reg_th(sc, 100);

	sc8549_set_vdrop_deglitch(sc, 5000);
	sc8549_set_vdrop_th(sc, 400);

	return 0;
}

static int sc8549_init_device(struct sc8549 *sc)
{
	sc8549_register_reset(sc);

	sc8549_set_wdt(sc, 0);

	sc8549_set_ss_timeout(sc, 0);
	sc8549_set_sense_resistor(sc, sc->cfg->sense_r_mohm);

	sc8549_init_protection(sc);

	sc8549_init_adc(sc);
	sc8549_init_int_src(sc);

	sc8549_init_regulation(sc);

	return 0;
}


static int sc8549_set_present(struct sc8549 *sc, bool present)
{
	sc->usb_present = present;

	if (present)
		sc8549_init_device(sc);
	return 0;
}

static int sc8549_get_vbus_present(struct sc8549 *sc)
{
	int ret;
	u8 val;

	ret= sc8549_read_byte(sc, SC8549_REG_0E, &val);
	if (ret) {
		sc_err("get vbus present faile \n");
		return ret;
	}
	sc->vbus_present = !!(val & SC8549_ADAPTER_INSERT_STAT_MASK);

	return ret;
}
static ssize_t sc8549_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8549 *sc = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8549");
	for (addr = 0x0; addr <= 0x36; addr++) {
		ret = sc8549_read_byte(sc, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
					"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sc8549_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sc8549 *sc = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x36)
		sc8549_write_byte(sc, (unsigned char)reg, (unsigned char)val);

	return count;
}



static DEVICE_ATTR(registers, 0660, sc8549_show_registers, sc8549_store_register);

static struct attribute *sc8549_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group sc8549_attr_group = {
	.attrs = sc8549_attributes,
};

static enum power_supply_property sc8549_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_BOOT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static void sc8549_check_fault_status(struct sc8549 *sc);

static int sc8549_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sc8549 *sc  = power_supply_get_drvdata(psy);
	int ret;
	int result;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_BOOT:
		sc8549_check_charge_enabled(sc, &sc->charge_enabled);
		val->intval = sc->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = sc8549_get_vbus_present(sc);
		if (!ret)
		    val->intval = sc->vbus_present;
		else
		    val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8549_get_adc_data(sc, ADC_VBAT, &result);
		if (!ret)
			sc->vbat_volt = result;

		val->intval = sc->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8549_get_adc_data(sc, ADC_IBAT, &result);
		if (!ret)
			sc->ibat_curr = result;

		val->intval = sc->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc8549_get_adc_data(sc, ADC_VBUS, &result);
		if (!ret)
			sc->vbus_volt = result;

		val->intval = sc->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc8549_get_adc_data(sc, ADC_IBUS, &result);
		if (!ret)
			sc->ibus_curr = result;

		val->intval = sc->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		sc8549_check_fault_status(sc);
		val->intval = (sc->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
			| (sc->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
			| (sc->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
			| (sc->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
			| (sc->ss_timeout_fault << SS_TIMEOUT_FAULT_SHIFT)
			| (sc->ts_shut_fault << TS_SHUT_FAULT_SHIFT);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sc8549_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct sc8549 *sc = power_supply_get_drvdata(psy);
	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_BOOT:
		sc8549_enable_charge(sc, val->intval);
		sc8549_check_charge_enabled(sc, &sc->charge_enabled);
		sc_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s, %d\n",
				val->intval ? "enable" : "disable", sc->charge_enabled);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		sc8549_set_present(sc, !!val->intval);
		sc_info("set present :%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		sc->bat_ovp_fault = false;
		sc->bat_ocp_fault = false;
		sc->bus_ovp_fault = false;
		sc->bus_ocp_fault = false;
		sc->drop_ovp_fault = false;
		sc->ac_ovp_fault = false;
		sc->ss_timeout_fault = false;
		sc->ts_shut_fault = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sc8549_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_BOOT:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}


static const struct power_supply_desc sc8549_standalone_psy_desc = {
	.name = "cp-standalone",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sc8549_charger_props,
	.num_properties = ARRAY_SIZE(sc8549_charger_props),
	.get_property = sc8549_charger_get_property,
	.set_property = sc8549_charger_set_property,
	.property_is_writeable = sc8549_charger_is_writeable,
};

static int sc8549_psy_register(struct sc8549 *sc)
{
	struct power_supply_config sc8549_cfg = {};


	sc8549_cfg.drv_data = sc;
	sc8549_cfg.of_node = sc->dev->of_node;

	sc->fc2_psy = devm_power_supply_register(sc->dev,
					&sc8549_standalone_psy_desc, &sc8549_cfg);
	if (IS_ERR(sc->fc2_psy)) {
		pr_err("Couldn't register sc8549 power supply\n");
		return PTR_ERR(sc->fc2_psy);
	}

	pr_info("power supply register sc8549 successfully\n");
	return 0;
}

static void sc8549_check_fault_status(struct sc8549 *sc)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;

	mutex_lock(&sc->data_lock);

	ret = sc8549_read_byte(sc, SC8549_REG_02, &stat);
	if (!ret && (stat & (SC8549_AC_OVP_STAT_MASK | SC8549_AC_OVP_FLAG_MASK))) {
		sc_err("VAC_STAT = 0x%02X\n", stat);
		if (stat & SC8549_AC_OVP_FLAG_MASK)
			sc->ac_ovp_fault = !!(stat & SC8549_AC_OVP_FLAG_MASK);
	}

	ret = sc8549_read_byte(sc, SC8549_REG_03, &stat);
	if (!ret && (stat & (SC8549_VDROP_OVP_STAT_MASK | SC8549_VDROP_OVP_FLAG_MASK))) {
		sc_err("VDROP_STAT = 0x%02X\n", stat);
		if (stat & SC8549_VDROP_OVP_FLAG_MASK)
			sc->drop_ovp_fault = !!(stat & SC8549_VDROP_OVP_FLAG_MASK);
	}

	ret = sc8549_read_byte(sc, SC8549_REG_06, &stat);
	if (!ret && stat) {
		sc_err("REG06_STAT = 0x%02X\n", stat);
		if (stat & SC8549_TSHUT_FLAG_MASK)
			sc->ts_shut_fault = !!(stat & SC8549_TSHUT_FLAG_MASK);
		if (stat & SC8549_SS_TIMEOUT_FLAG_MASK)
			sc->ss_timeout_fault = !!(stat & SC8549_SS_TIMEOUT_FLAG_MASK);
	}

	ret = sc8549_read_byte(sc, SC8549_REG_0F, &flag);
	if (!ret && flag)
		sc_err("FAULT_FLAG = 0x%02X\n", flag);

	if (!ret && flag != sc->prev_fault) {
		sc->prev_fault = flag;
		sc->bat_ovp_fault = !!(flag & SC8549_VBAT_OVP_FLAG_MASK);
		sc->bat_ocp_fault = !!(flag & SC8549_IBAT_OCP_FLAG_MASK);
		sc->bus_ovp_fault = !!(flag & SC8549_VBUS_OVP_FLAG_MASK);
		sc->bus_ocp_fault = !!(flag & SC8549_IBUS_OCP_FLAG_MASK);
	}

	mutex_unlock(&sc->data_lock);
}


static irqreturn_t sc8549_charger_interrupt(int irq, void *dev_id)
{
	struct sc8549 *sc = dev_id;
	sc_err("------------sc8549 interrupt--------------\n");
	mutex_lock(&sc->irq_complete);
	sc->irq_waiting = true;
	if (!sc->resume_completed) {
		dev_dbg(sc->dev, "IRQ triggered before device-resume\n");
		if (!sc->irq_disabled) {
			disable_irq_nosync(irq);
			sc->irq_disabled = true;
		}
		mutex_unlock(&sc->irq_complete);
		return IRQ_HANDLED;
	}
	sc->irq_waiting = false;
	if (sc->irq_counts > INT_MAX -1)
		sc->irq_counts = 0;
	else
		sc->irq_counts++;

	sc8549_check_fault_status(sc);

	mutex_unlock(&sc->irq_complete);

	sc_err("report fault flag =0x%02X\n",sc->prev_fault);
	return IRQ_HANDLED;
}

static void determine_initial_status(struct sc8549 *sc)
{
	if (sc->client->irq)
		sc8549_charger_interrupt(sc->client->irq, sc);
}

#if defined(CONFIG_DEBUG_FS)
static int show_registers(struct seq_file *m, void *data)
{
	struct sc8549 *sc = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x36; addr++) {
		if (addr <= 0x24 || addr == 0x36) {
			ret = sc8549_read_byte(sc, addr, &val);
			if (!ret)
				seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
		}
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct sc8549 *sc = inode->i_private;

	return single_open(file, show_registers, sc);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct sc8549 *sc)
{
	sc->debug_root = debugfs_create_dir("sc8549-standalone", NULL);

	if (!sc->debug_root)
		sc_err("Failed to create debug dir\n");

	if (sc->debug_root) {
		debugfs_create_file("registers",
					S_IFREG | S_IRUGO,
					sc->debug_root, sc, &reg_debugfs_ops);

		debugfs_create_x32("skip_reads",
					S_IFREG | S_IWUSR | S_IRUGO,
					sc->debug_root,
					&(sc->skip_reads));
		debugfs_create_x32("skip_writes",
					S_IFREG | S_IWUSR | S_IRUGO,
					sc->debug_root,
					&(sc->skip_writes));
	}
}
#else
static void create_debugfs_entry(struct sc8549 *sc)
{}
#endif
static struct of_device_id sc8549_charger_match_table[] = {
	{
		.compatible = "sc,sc8549-standalone"
	},
	{},
};
MODULE_DEVICE_TABLE(of, sc8549_charger_match_table);


static int sc8549_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct sc8549 *sc;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int ret;
	printk("-----------sc8549---------\n");

	sc = devm_kzalloc(&client->dev, sizeof(struct sc8549), GFP_KERNEL);
	if (!sc) {
		sc_err("Out of memory\n");
		return -ENOMEM;
	}
	sc->dev = &client->dev;

	sc->client = client;
	i2c_set_clientdata(client, sc);
	mutex_init(&sc->i2c_rw_lock);
	mutex_init(&sc->data_lock);
	mutex_init(&sc->charging_disable_lock);
	mutex_init(&sc->irq_complete);

	sc->resume_completed = true;
	sc->irq_waiting = false;

	ret = sc8549_detect_device(sc);
	if (ret) {
		sc_err("No sc8549 device found!\n");
		return -ENODEV;
	}

	match = of_match_node(sc8549_charger_match_table, node);
	if (match == NULL) {
		sc_err("device tree match not found!\n");
		return -ENODEV;
	}

	ret = sc8549_parse_dt(sc, &client->dev);
	if (ret)
		return -EIO;

	ret = sc8549_init_device(sc);
	if (ret) {
		sc_err("Failed to init device\n");
		return ret;
	}

	ret = sc8549_psy_register(sc);
	if (ret)
		return ret;

	sc->irq_pinctrl =
		pinctrl_get_select(sc->dev, "sc8549_int_default");
	if (!sc->irq_pinctrl) {
		sc_err("Couldn't set pinctrl sc8549_int_default\n");
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, sc8549_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"sc8549 charger irq", sc);
		if (ret < 0) {
			sc_err("request irq for irq=%d failed, ret =%d\n",
							client->irq, ret);
			return ret;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(sc->dev, 1);
	create_debugfs_entry(sc);

	ret = sysfs_create_group(&sc->dev->kobj, &sc8549_attr_group);
	if (ret) {
		sc_err("failed to register sysfs. err: %d\n", ret);
		return ret;
	}

	determine_initial_status(sc);

	sc_info("sc8549 probe successfully, Part Num:%d\n!",
				sc->part_no);
	return 0;
}


static inline bool is_device_suspended(struct sc8549 *sc)
{
	return !sc->resume_completed;
}

static int sc8549_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8549 *sc = i2c_get_clientdata(client);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = false;
	mutex_unlock(&sc->irq_complete);
	sc_err("Suspend successfully!");

	return 0;
}

static int sc8549_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8549 *sc = i2c_get_clientdata(client);

	if (sc->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int sc8549_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8549 *sc = i2c_get_clientdata(client);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = true;
	if (sc->irq_waiting) {
		sc->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&sc->irq_complete);
		sc8549_charger_interrupt(client->irq, sc);
	} else {
		mutex_unlock(&sc->irq_complete);
	}

	power_supply_changed(sc->fc2_psy);
	sc_err("Resume successfully!");

	return 0;
}
static int sc8549_charger_remove(struct i2c_client *client)
{
	struct sc8549 *sc = i2c_get_clientdata(client);

	sc8549_enable_adc(sc, false);

	mutex_destroy(&sc->charging_disable_lock);
	mutex_destroy(&sc->data_lock);
	mutex_destroy(&sc->i2c_rw_lock);
	mutex_destroy(&sc->irq_complete);
#if defined(CONFIG_DEBUG_FS)
	debugfs_remove_recursive(sc->debug_root);
#endif
	sysfs_remove_group(&sc->dev->kobj, &sc8549_attr_group);

	return 0;
}

static void sc8549_charger_shutdown(struct i2c_client *client)
{
	struct sc8549 *sc = i2c_get_clientdata(client);
	sc8549_enable_adc(sc, false);
	pr_info("Shutdown Successfully\n");
}

static const struct dev_pm_ops sc8549_pm_ops = {
	.resume		= sc8549_resume,
	.suspend_noirq = sc8549_suspend_noirq,
	.suspend	= sc8549_suspend,
};

static const struct i2c_device_id sc8549_charger_id[] = {
	{"sc8549-standalone", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sc8549_charger_id);

static struct i2c_driver sc8549_charger_driver = {
	.driver		= {
		.name	= "sc8549-charger",
		.owner	= THIS_MODULE,
		.of_match_table = sc8549_charger_match_table,
		.pm	= &sc8549_pm_ops,
	},
	.id_table	= sc8549_charger_id,

	.probe		= sc8549_charger_probe,
	.remove		= sc8549_charger_remove,
	.shutdown	= sc8549_charger_shutdown,
};

module_i2c_driver(sc8549_charger_driver);

MODULE_DESCRIPTION("SC SC8549 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Southchip");

