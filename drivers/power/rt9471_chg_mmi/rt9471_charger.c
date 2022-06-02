/*
 * Copyright (C) 2021 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/reboot.h>
#include <linux/mmi_discrete_charger_class.h>
#include <linux/seq_file.h>
#include <uapi/linux/sched/types.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/phy.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>

#ifdef CONFIG_RT_REGMAP
#include <linux/regmap.h>
#endif /* CONFIG_RT_REGMAP */


#include "rt9471_charger.h"
#define RT9471_DRV_VERSION	"1.0.17_MTK"

#define RT9471_STATUS_PLUGIN			0x0001
#define RT9471_STATUS_PG				0x0002
#define RT9471_STATUS_CHARGE_ENABLE		0x0004
#define RT9471_STATUS_FAULT				0x0008
#define RT9471_STATUS_EXIST				0x0100

#define MMI_HVDCP2_VOLTAGE_STANDARD		8000000
#define MMI_HVDCP3_VOLTAGE_STANDARD		7500000
#define MMI_HVDCP_DETECT_ICL_LIMIT		500000

enum rt9471_stat_idx {
	RT9471_STATIDX_STAT0 = 0,
	RT9471_STATIDX_STAT1,
	RT9471_STATIDX_STAT2,
	RT9471_STATIDX_STAT3,
	RT9471_STATIDX_MAX,
};

enum rt9471_irq_idx {
	RT9471_IRQIDX_IRQ0 = 0,
	RT9471_IRQIDX_IRQ1,
	RT9471_IRQIDX_IRQ2,
	RT9471_IRQIDX_IRQ3,
	RT9471_IRQIDX_MAX,
};

enum rt9471_ic_stat {
	RT9471_ICSTAT_SLEEP = 0,
	RT9471_ICSTAT_VBUSRDY,
	RT9471_ICSTAT_TRICKLECHG,
	RT9471_ICSTAT_PRECHG,
	RT9471_ICSTAT_FASTCHG,
	RT9471_ICSTAT_IEOC,
	RT9471_ICSTAT_BGCHG,
	RT9471_ICSTAT_CHGDONE,
	RT9471_ICSTAT_CHGFAULT,
	RT9471_ICSTAT_OTG = 15,
	RT9471_ICSTAT_MAX,
};

static const char * const rt9471_ic_stat_names[RT9471_ICSTAT_MAX] = {
	"hz/sleep", "ready", "trickle-charge", "pre-charge",
	"fast-charge", "ieoc-charge", "background-charge",
	"done", "fault", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "OTG",
};

enum rt9471_mivr_track {
	RT9471_MIVRTRACK_REG = 0,
	RT9471_MIVRTRACK_VBAT_200MV,
	RT9471_MIVRTRACK_VBAT_250MV,
	RT9471_MIVRTRACK_VBAT_300MV,
	RT9471_MIVRTRACK_MAX,
};

enum rt9471_port_stat {
	RT9471_PORTSTAT_NOINFO = 0,
	RT9471_PORTSTAT_APPLE_10W = 8,
	RT9471_PORTSTAT_SAMSUNG_10W,
	RT9471_PORTSTAT_APPLE_5W,
	RT9471_PORTSTAT_APPLE_12W,
	RT9471_PORTSTAT_NSDP,
	RT9471_PORTSTAT_SDP,
	RT9471_PORTSTAT_CDP,
	RT9471_PORTSTAT_DCP,
	RT9471_PORTSTAT_MAX,
};

enum rt9471_usbsw_state {
	RT9471_USBSW_CHG = 0,
	RT9471_USBSW_USB,
};

enum rt9471_hz_user {
	RT9471_HZU_PP,
	RT9471_HZU_BC12,
	RT9471_HZU_OTG,
	RT9471_HZU_VBUS_GD,
	RT9471_HZU_MAX,
};

static const char * const rt9471_hz_user_names[RT9471_HZU_MAX] = {
	"PP", "BC12", "OTG", "VBUS_GD",
};

struct rt9471_desc {
	const char *chg_name;
	const char *rm_name;
	u8 rm_dev_addr;
	u32 vac_ovp;
	u32 mivr;
	u32 aicr;
	u32 cv;
	u32 ichg;
	u32 ieoc;
	u32 safe_tmr;
	u32 wdt;
	u32 mivr_track;
	bool en_safe_tmr;
	bool en_te;
	bool en_jeita;
	bool ceb_invert;
	bool dis_i2c_tout;
	bool en_qon_rst;
	bool auto_aicr;
};

/* These default values will be applied if there's no property in dts */
static struct rt9471_desc rt9471_default_desc = {
	.chg_name = "primary_chg",
	.rm_name = "rt9471",
	.rm_dev_addr = RT9471_DEVICE_ADDR,
	.vac_ovp = 6500000,
	.mivr = 4500000,
	.aicr = 500000,
	.cv = 4200000,
	.ichg = 2000000,
	.ieoc = 200000,
	.safe_tmr = 10,
	.wdt = 40,
	.mivr_track = RT9471_MIVRTRACK_REG,
	.en_safe_tmr = true,
	.en_te = true,
	.en_jeita = true,
	.ceb_invert = false,
	.dis_i2c_tout = false,
	.en_qon_rst = true,
	.auto_aicr = true,
};

static const u8 rt9471_irq_maskall[RT9471_IRQIDX_MAX] = {
	0xFF, 0xFF, 0xFF, 0xFF,
};

static const u32 rt9471_vac_ovp[] = {
	5800000, 6500000, 10900000, 14000000,
};

static const u32 rt9471_wdt[] = {
	0, 40, 80, 160,
};

static const u32 rt9471_otgcc[] = {
	500000, 1200000,
};

static const u8 rt9471_val_en_hidden_mode[] = {
	0x69, 0x96,
};

static enum power_supply_usb_type rt9471_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
};

struct rt9471_state {
	bool online;
	bool vbus_gd;
};

struct rt9471_iio {
	struct iio_channel	*usbin_v_chan;
};

struct rt9471_chip {
	struct i2c_client *client;
	struct device *dev;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct mutex io_lock;
	struct mutex hidden_mode_lock;
	struct mutex hz_lock;
	int hidden_mode_cnt;
	u8 dev_id;
	u8 dev_rev;
	u8 chip_rev;
	struct rt9471_desc *desc;
	u32 intr_gpio;
	u32 ceb_gpio;
	int irq;
	u8 irq_mask[RT9471_IRQIDX_MAX];
	bool chg_done_once;
	struct wakeup_source *buck_dwork_ws;
	struct delayed_work buck_dwork;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *rm_dev;
	struct rt_regmap_properties *rm_prop;
#endif /* CONFIG_RT_REGMAP */
	bool enter_shipping_mode;
	struct completion aicc_done;
	struct completion pe_done;
	bool is_primary;
	bool hz_users[RT9471_HZU_MAX];

	int real_charger_type;
	const char *mmi_chg_dev_name;
	struct power_supply *charger;
	struct regulator *dpdm_reg;
	struct mutex regulator_lock;
	bool dpdm_enabled;
	u32 input_current_cache;
	struct	delayed_work monitor_work;
	struct work_struct charge_detect_work;
	unsigned int status;
	struct rt9471_state state;
	struct rt9471_iio iio;

	bool mmi_qc3_support;
	wait_queue_head_t	mmi_qc3_wait_que;
	bool mmi_qc3_trig_flag;
	struct task_struct	*mmi_qc3_authen_task;
	bool mmi_is_qc3_authen;

#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	struct iio_channel	**ext_iio_chans;
#endif
};

static struct power_supply_desc rt9471_power_supply_desc;
static int rt9471_set_aicr(struct charger_device *chg_dev, u32 uA);

static const u8 rt9471_reg_addr[] = {
	RT9471_REG_OTGCFG,
	RT9471_REG_TOP,
	RT9471_REG_FUNCTION,
	RT9471_REG_IBUS,
	RT9471_REG_VBUS,
	RT9471_REG_PRECHG,
	RT9471_REG_REGU,
	RT9471_REG_VCHG,
	RT9471_REG_ICHG,
	RT9471_REG_CHGTIMER,
	RT9471_REG_EOC,
	RT9471_REG_INFO,
	RT9471_REG_JEITA,
	RT9471_REG_PUMPEXP,
	RT9471_REG_DPDMDET,
	RT9471_REG_STATUS,
	RT9471_REG_STAT0,
	RT9471_REG_STAT1,
	RT9471_REG_STAT2,
	RT9471_REG_STAT3,
	/* Skip IRQs to prevent reading clear while dumping registers */
	RT9471_REG_MASK0,
	RT9471_REG_MASK1,
	RT9471_REG_MASK2,
	RT9471_REG_MASK3,
};

static int rt9471_read_device(void *client, u32 addr, int len, void *dst)
{
	int ret = 0;

	ret = i2c_smbus_read_i2c_block_data(client, addr, len, dst);

	return (ret < 0) ? ret : 0;
}

static int rt9471_write_device(void *client, u32 addr, int len,
			       const void *src)
{
	return i2c_smbus_write_i2c_block_data(client, addr, len, src);
}

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(RT9471_REG_OTGCFG, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_TOP, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_FUNCTION, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_IBUS, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_VBUS, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_PRECHG, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_REGU, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_VCHG, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_ICHG, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_CHGTIMER, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_EOC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_INFO, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_JEITA, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_PUMPEXP, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_DPDMDET, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_STATUS, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_STAT0, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_STAT1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_STAT2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_STAT3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_IRQ0, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_IRQ1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_IRQ2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_IRQ3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_MASK0, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_MASK1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_MASK2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9471_REG_MASK3, 1, RT_VOLATILE, {});

static const rt_register_map_t rt9471_rm_map[] = {
	RT_REG(RT9471_REG_OTGCFG),
	RT_REG(RT9471_REG_TOP),
	RT_REG(RT9471_REG_FUNCTION),
	RT_REG(RT9471_REG_IBUS),
	RT_REG(RT9471_REG_VBUS),
	RT_REG(RT9471_REG_PRECHG),
	RT_REG(RT9471_REG_REGU),
	RT_REG(RT9471_REG_VCHG),
	RT_REG(RT9471_REG_ICHG),
	RT_REG(RT9471_REG_CHGTIMER),
	RT_REG(RT9471_REG_EOC),
	RT_REG(RT9471_REG_INFO),
	RT_REG(RT9471_REG_JEITA),
	RT_REG(RT9471_REG_PUMPEXP),
	RT_REG(RT9471_REG_DPDMDET),
	RT_REG(RT9471_REG_STATUS),
	RT_REG(RT9471_REG_STAT0),
	RT_REG(RT9471_REG_STAT1),
	RT_REG(RT9471_REG_STAT2),
	RT_REG(RT9471_REG_STAT3),
	RT_REG(RT9471_REG_IRQ0),
	RT_REG(RT9471_REG_IRQ1),
	RT_REG(RT9471_REG_IRQ2),
	RT_REG(RT9471_REG_IRQ3),
	RT_REG(RT9471_REG_MASK0),
	RT_REG(RT9471_REG_MASK1),
	RT_REG(RT9471_REG_MASK2),
	RT_REG(RT9471_REG_MASK3),
};

static struct rt_regmap_fops rt9471_rm_fops = {
	.read_device = rt9471_read_device,
	.write_device = rt9471_write_device,
};

static int rt9471_register_rt_regmap(struct rt9471_chip *chip)
{
	struct rt_regmap_properties *prop = NULL;

	dev_info(chip->dev, "%s\n", __func__);

	prop = devm_kzalloc(chip->dev, sizeof(*prop), GFP_KERNEL);
	if (!prop)
		return -ENOMEM;

	prop->name = chip->desc->rm_name;
	prop->aliases = chip->desc->rm_name;
	prop->register_num = ARRAY_SIZE(rt9471_rm_map);
	prop->rm = rt9471_rm_map;
	prop->rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE |
			       RT_IO_PASS_THROUGH | RT_DBG_SPECIAL;
	prop->io_log_en = 0;

	chip->rm_prop = prop;
	chip->rm_dev = rt_regmap_device_register_ex(chip->rm_prop,
						    &rt9471_rm_fops, chip->dev,
						    chip->client,
						    chip->desc->rm_dev_addr,
						    chip);
	if (!chip->rm_dev) {
		dev_notice(chip->dev, "%s fail\n", __func__);
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_RT_REGMAP */

static inline int __rt9471_i2c_read_byte(struct rt9471_chip *chip, u8 cmd,
					 u8 *data)
{
	int ret = 0;
	u8 regval = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->rm_dev, cmd, 1, &regval);
#else
	ret = rt9471_read_device(chip->client, cmd, 1, &regval);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X fail(%d)\n",
				      __func__, cmd, ret);
	else {
		dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
				   __func__, cmd, regval);
		*data = regval;
	}

	return ret;
}

static int rt9471_i2c_read_byte(struct rt9471_chip *chip, u8 cmd, u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_read_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __rt9471_i2c_write_byte(struct rt9471_chip *chip, u8 cmd,
					  u8 data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(chip->rm_dev, cmd, 1, &data);
#else
	ret = rt9471_write_device(chip->client, cmd, 1, &data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X = 0x%02X fail(%d)\n",
				      __func__, cmd, data, ret);
	else
		dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
				   __func__, cmd, data);

	return ret;
}

static int rt9471_i2c_write_byte(struct rt9471_chip *chip, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_write_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __rt9471_i2c_block_read(struct rt9471_chip *chip, u8 cmd,
					  u32 len, u8 *data)
{
	int ret = 0, i = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->rm_dev, cmd, len, data);
#else
	ret = rt9471_read_device(chip->client, cmd, len, data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X..reg0x%02X fail(%d)\n",
				      __func__, cmd, cmd + len - 1, ret);
	else
		for (i = 0; i <= len - 1; i++)
			dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
					   __func__, cmd + i, data[i]);

	return ret;
}

static int rt9471_i2c_block_read(struct rt9471_chip *chip, u8 cmd, u32 len,
				 u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_block_read(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __rt9471_i2c_block_write(struct rt9471_chip *chip, u8 cmd,
					   u32 len, const u8 *data)
{
	int ret = 0, i = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(chip->rm_dev, cmd, len, data);
#else
	ret = rt9471_write_device(chip->client, cmd, len, data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0) {
		dev_notice(chip->dev, "%s fail(%d)\n", __func__, ret);
		for (i = 0; i <= len - 1; i++)
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
					      __func__, cmd + i, data[i]);
	} else
		for (i = 0; i <= len - 1; i++)
			dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
					   __func__, cmd + i, data[i]);

	return ret;
}

static int rt9471_i2c_block_write(struct rt9471_chip *chip, u8 cmd, u32 len,
				  const u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_block_write(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int rt9471_i2c_test_bit(struct rt9471_chip *chip, u8 cmd, u8 shift,
			       bool *is_one)
{
	int ret = 0;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	regval &= 1 << shift;
	*is_one = (regval ? true : false);

	return ret;
}

static int rt9471_i2c_update_bits(struct rt9471_chip *chip, u8 cmd, u8 data,
				  u8 mask)
{
	int ret = 0;
	u8 regval = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0)
		goto out;

	regval &= ~mask;
	regval |= (data & mask);

	ret = __rt9471_i2c_write_byte(chip, cmd, regval);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int rt9471_set_bit(struct rt9471_chip *chip, u8 cmd, u8 mask)
{
	return rt9471_i2c_update_bits(chip, cmd, mask, mask);
}

static inline int rt9471_clr_bit(struct rt9471_chip *chip, u8 cmd, u8 mask)
{
	return rt9471_i2c_update_bits(chip, cmd, 0x00, mask);
}

static inline u8 rt9471_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if (target < min)
		return 0;

	if (target >= max)
		target = max;

	return (target - min) / step;
}

static inline u8 rt9471_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
					    u32 target)
{
	u32 i = 0;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static inline u32 rt9471_closest_value(u32 min, u32 max, u32 step, u8 regval)
{
	u32 val = 0;

	val = min + regval * step;
	if (val > max)
		val = max;

	return val;
}

static int rt9471_enable_hidden_mode(struct rt9471_chip *chip, bool en)
{
	int ret = 0;

	mutex_lock(&chip->hidden_mode_lock);

	if (en) {
		if (chip->hidden_mode_cnt == 0) {
			ret = rt9471_i2c_block_write(chip, RT9471_REG_PASSCODE1,
				ARRAY_SIZE(rt9471_val_en_hidden_mode),
				rt9471_val_en_hidden_mode);
			if (ret < 0)
				goto err;
		}
		chip->hidden_mode_cnt++;
	} else {
		if (chip->hidden_mode_cnt == 1) { /* last one */
			ret = rt9471_i2c_write_byte(chip, RT9471_REG_PASSCODE1,
						    0x00);
			if (ret < 0)
				goto err;
		}
		chip->hidden_mode_cnt--;
	}
	dev_info(chip->dev, "%s en = %d, cnt = %d\n",
			    __func__, en, chip->hidden_mode_cnt);
	goto out;

err:
	dev_notice(chip->dev, "%s en = %d fail(%d)\n", __func__, en, ret);
out:
	mutex_unlock(&chip->hidden_mode_lock);
	return ret;
}

static int __rt9471_get_ic_stat(struct rt9471_chip *chip,
				enum rt9471_ic_stat *stat)
{
	int ret = 0;
	u8 regval = 0;

#ifdef RT9471_I2C_NO_MUTEX
	ret = __rt9471_i2c_read_byte(chip, RT9471_REG_STATUS, &regval);
#else
	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STATUS, &regval);
#endif
	if (ret < 0)
		return ret;
	*stat = (regval & RT9471_ICSTAT_MASK) >> RT9471_ICSTAT_SHIFT;

	return ret;
}

static int __rt9471_get_mivr(struct rt9471_chip *chip, u32 *mivr)
{
	int ret = 0;
	u8 regval = 0;

#ifdef RT9471_I2C_NO_MUTEX
	ret = __rt9471_i2c_read_byte(chip, RT9471_REG_VBUS, &regval);
#else
	ret = rt9471_i2c_read_byte(chip, RT9471_REG_VBUS, &regval);
#endif
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_MIVR_MASK) >> RT9471_MIVR_SHIFT;
	*mivr = rt9471_closest_value(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
				     RT9471_MIVR_STEP, regval);

	return ret;
}

static int __rt9471_get_aicr(struct rt9471_chip *chip, u32 *aicr)
{
	int ret = 0;
	u8 regval = 0;

#ifdef RT9471_I2C_NO_MUTEX
	ret = __rt9471_i2c_read_byte(chip, RT9471_REG_IBUS, &regval);
#else
	ret = rt9471_i2c_read_byte(chip, RT9471_REG_IBUS, &regval);
#endif
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_AICR_MASK) >> RT9471_AICR_SHIFT;
	*aicr = rt9471_closest_value(RT9471_AICR_MIN, RT9471_AICR_MAX,
				     RT9471_AICR_STEP, regval);
	if (*aicr > RT9471_AICR_MIN && *aicr < RT9471_AICR_MAX)
		*aicr -= RT9471_AICR_STEP;

	return ret;
}

static int __rt9471_get_cv(struct rt9471_chip *chip, u32 *cv)
{
	int ret = 0;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_VCHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_CV_MASK) >> RT9471_CV_SHIFT;
	*cv = rt9471_closest_value(RT9471_CV_MIN, RT9471_CV_MAX, RT9471_CV_STEP,
				   regval);

	return ret;
}

static int __rt9471_get_ichg(struct rt9471_chip *chip, u32 *ichg)
{
	int ret = 0;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_ICHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_ICHG_MASK) >> RT9471_ICHG_SHIFT;
	*ichg = rt9471_closest_value(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
				     RT9471_ICHG_STEP, regval);

	return ret;
}

static int __rt9471_get_ieoc(struct rt9471_chip *chip, u32 *ieoc)
{
	int ret = 0;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_EOC, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_IEOC_MASK) >> RT9471_IEOC_SHIFT;
	*ieoc = rt9471_closest_value(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
				     RT9471_IEOC_STEP, regval);

	return ret;
}

static inline int __rt9471_is_hz_enabled(struct rt9471_chip *chip, bool *en)
{
	if (chip->is_primary)
		return rt9471_i2c_test_bit(chip, RT9471_REG_FUNCTION,
					   RT9471_HZ_SHIFT, en);
	else
		return rt9471_i2c_test_bit(chip, RT9471_REG_HIDDEN_2,
					   RT9471_FORCE_HZ_SHIFT, en);
}

static inline int __rt9471_is_chg_enabled(struct rt9471_chip *chip, bool *en)
{
	return rt9471_i2c_test_bit(chip, RT9471_REG_FUNCTION,
				   RT9471_CHG_EN_SHIFT, en);
}

static int __rt9471_enable_shipmode(struct rt9471_chip *chip)
{
	const u8 mask = RT9471_BATFETDIS_MASK | RT9471_HZ_MASK;

	dev_info(chip->dev, "%s\n", __func__);

	return rt9471_i2c_update_bits(chip, RT9471_REG_FUNCTION, mask, mask);
}

static int __rt9471_enable_safe_tmr(struct rt9471_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_CHGTIMER, RT9471_SAFETMR_EN_MASK);
}

static int __rt9471_eoc_rst(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return rt9471_set_bit(chip, RT9471_REG_EOC, RT9471_EOC_RST_MASK);
}

static int __rt9471_enable_te(struct rt9471_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_EOC, RT9471_TE_MASK);
}

static int __rt9471_enable_jeita(struct rt9471_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_JEITA, RT9471_JEITA_EN_MASK);
}

static int __rt9471_disable_i2c_tout(struct rt9471_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_TOP, RT9471_DISI2CTO_MASK);
}

static int __rt9471_enable_qon_rst(struct rt9471_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_TOP, RT9471_QONRST_MASK);
}

static int __rt9471_enable_autoaicr(struct rt9471_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_IBUS, RT9471_AUTOAICR_MASK);
}

static int __rt9471_enable_hz(struct rt9471_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (chip->is_primary) {
		ret = (en ? rt9471_set_bit : rt9471_clr_bit)
			(chip, RT9471_REG_FUNCTION, RT9471_HZ_MASK);
	} else {
		ret = rt9471_enable_hidden_mode(chip, true);
		if (ret < 0)
			goto out;

		ret = (en ? rt9471_set_bit : rt9471_clr_bit)
			(chip, RT9471_REG_HIDDEN_2, RT9471_FORCE_HZ_MASK);

		rt9471_enable_hidden_mode(chip, false);
	}
out:
	return ret;
}

static int rt9471_enable_hz(struct rt9471_chip *chip, bool en, u32 user)
{
	int ret = 0, i = 0;

	if (user >= RT9471_HZU_MAX)
		return -EINVAL;

	dev_info(chip->dev, "%s en = %d, user = %s\n",
			    __func__, en, rt9471_hz_user_names[user]);

	mutex_lock(&chip->hz_lock);
	chip->hz_users[user] = en;
	for (i = 0, en = true; i < RT9471_HZU_MAX; i++)
		en &= chip->hz_users[i];
	ret = __rt9471_enable_hz(chip, en);
	mutex_unlock(&chip->hz_lock);

	return ret;
}

static int __rt9471_enable_otg(struct rt9471_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = rt9471_enable_hz(chip, !en, RT9471_HZU_OTG);
	if (ret < 0)
		dev_notice(chip->dev, "%s %s hz fail(%d)\n",
				      __func__, en ? "dis" : "en", ret);

	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_FUNCTION, RT9471_OTG_EN_MASK);
}

static int __rt9471_set_otgcc(struct rt9471_chip *chip, u32 cc)
{
	dev_info(chip->dev, "%s cc = %d\n", __func__, cc);
	return (cc <= rt9471_otgcc[0] ? rt9471_clr_bit : rt9471_set_bit)
		(chip, RT9471_REG_OTGCFG, RT9471_OTGCC_MASK);
}

static int __rt9471_enable_chg(struct rt9471_chip *chip, bool en)
{
	int ret = 0;
	struct rt9471_desc *desc = chip->desc;

	dev_info(chip->dev, "%s en = %d, chip_rev = %d\n",
			    __func__, en, chip->chip_rev);

	if (chip->ceb_gpio != U32_MAX)
		gpio_set_value(chip->ceb_gpio, desc->ceb_invert ? en : !en);

	ret = (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_FUNCTION, RT9471_CHG_EN_MASK);
	if (ret >= 0 && chip->chip_rev <= 4)
		mod_delayed_work(system_wq, &chip->buck_dwork,
				 msecs_to_jiffies(100));

	return ret;
}

static int __rt9471_set_vac_ovp(struct rt9471_chip *chip, u32 vac_ovp)
{
	u8 regval = 0;

	regval = rt9471_closest_reg_via_tbl(rt9471_vac_ovp,
					    ARRAY_SIZE(rt9471_vac_ovp),
					    vac_ovp);

	dev_info(chip->dev, "%s vac_ovp = %d(0x%02X)\n",
			    __func__, vac_ovp, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
				      regval << RT9471_VAC_OVP_SHIFT,
				      RT9471_VAC_OVP_MASK);
}

static int __rt9471_set_mivr(struct rt9471_chip *chip, u32 mivr)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
				    RT9471_MIVR_STEP, mivr);

	dev_info(chip->dev, "%s mivr = %d(0x%02X)\n", __func__, mivr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
				      regval << RT9471_MIVR_SHIFT,
				      RT9471_MIVR_MASK);
}

static int __rt9471_set_aicr(struct rt9471_chip *chip, u32 aicr)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_AICR_MIN, RT9471_AICR_MAX,
				    RT9471_AICR_STEP, aicr);
	/* 0 & 1 are both 50mA */
	if (aicr < RT9471_AICR_MAX)
		regval += 1;

	dev_info(chip->dev, "%s aicr = %d(0x%02X)\n", __func__, aicr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_IBUS,
				      regval << RT9471_AICR_SHIFT,
				      RT9471_AICR_MASK);
}

static int __rt9471_set_cv(struct rt9471_chip *chip, u32 cv)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_CV_MIN, RT9471_CV_MAX,
				    RT9471_CV_STEP, cv);

	dev_info(chip->dev, "%s cv = %d(0x%02X)\n", __func__, cv, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VCHG,
				      regval << RT9471_CV_SHIFT,
				      RT9471_CV_MASK);
}

static int __rt9471_set_ichg(struct rt9471_chip *chip, u32 ichg)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
				    RT9471_ICHG_STEP, ichg);

	dev_info(chip->dev, "%s ichg = %d(0x%02X)\n", __func__, ichg, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_ICHG,
				      regval << RT9471_ICHG_SHIFT,
				      RT9471_ICHG_MASK);
}

static int __rt9471_set_ieoc(struct rt9471_chip *chip, u32 ieoc)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
				    RT9471_IEOC_STEP, ieoc);

	dev_info(chip->dev, "%s ieoc = %d(0x%02X)\n", __func__, ieoc, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_EOC,
				      regval << RT9471_IEOC_SHIFT,
				      RT9471_IEOC_MASK);
}

static int __rt9471_reset_eoc_state(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);

	return rt9471_set_bit(chip, RT9471_REG_EOC, RT9471_EOC_RST_MASK);
}

static int __rt9471_set_safe_tmr(struct rt9471_chip *chip, u32 hr)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_SAFETMR_MIN, RT9471_SAFETMR_MAX,
				    RT9471_SAFETMR_STEP, hr);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, hr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_CHGTIMER,
				      regval << RT9471_SAFETMR_SHIFT,
				      RT9471_SAFETMR_MASK);
}

static int __rt9471_set_wdt(struct rt9471_chip *chip, u32 sec)
{
	u8 regval = 0;

	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= 40 && sec > 0)
		sec = 40;
	regval = rt9471_closest_reg_via_tbl(rt9471_wdt, ARRAY_SIZE(rt9471_wdt),
					    sec);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, sec, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_TOP,
				      regval << RT9471_WDT_SHIFT,
				      RT9471_WDT_MASK);
}

static int __rt9471_set_mivrtrack(struct rt9471_chip *chip, u32 mivr_track)
{
	if (mivr_track >= RT9471_MIVRTRACK_MAX)
		mivr_track = RT9471_MIVRTRACK_VBAT_300MV;

	dev_info(chip->dev, "%s mivrtrack = %d\n", __func__, mivr_track);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
				      mivr_track << RT9471_MIVRTRACK_SHIFT,
				      RT9471_MIVRTRACK_MASK);
}

static int __rt9471_kick_wdt(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return rt9471_set_bit(chip, RT9471_REG_TOP, RT9471_WDTCNTRST_MASK);
}

static inline int rt9471_toggle_bc12(struct rt9471_chip *chip)
{
	int ret = 0;
	u8 regval = 0, bc12_dis[2] = {0}, bc12_en[2] = {0};
	struct i2c_client *client = chip->client;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = bc12_dis,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = bc12_en,
		},
	};

	mutex_lock(&chip->io_lock);

	ret = i2c_smbus_read_i2c_block_data(client, RT9471_REG_DPDMDET,
					    1, &regval);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read reg fail(%d)\n", __func__, ret);
		goto out;
	}

	/* bc12 disable and then enable */
	bc12_dis[0] = bc12_en[0] = RT9471_REG_DPDMDET;
	bc12_dis[1] = regval & ~RT9471_BC12_EN_MASK;
	bc12_en[1] = regval | RT9471_BC12_EN_MASK;
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_notice(chip->dev, "%s bc12 dis/en fail(%d)\n",
				      __func__, ret);
#ifdef CONFIG_RT_REGMAP
	rt_regmap_cache_reload(chip->rm_dev);
#endif /* CONFIG_RT_REGMAP */
out:
	mutex_unlock(&chip->io_lock);
	return ret < 0 ? ret : 0;
}

static int __rt9471_enable_bc12(struct rt9471_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = rt9471_enable_hz(chip, !en, RT9471_HZU_BC12);
	if (ret < 0)
		dev_notice(chip->dev, "%s %s hz fail(%d)\n",
				      __func__, en ? "dis" : "en", ret);

	if (en)
		return rt9471_toggle_bc12(chip);
	else
		return rt9471_clr_bit(chip, RT9471_REG_DPDMDET,
				      RT9471_BC12_EN_MASK);
}

static int __rt9471_dump_registers(struct rt9471_chip *chip)
{
	int ret = 0, i = 0;
	u32 mivr = 0, aicr = 0, cv = 0, ichg = 0, ieoc = 0;
	bool chg_en = 0;
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_SLEEP;
	u8 stats[RT9471_STATIDX_MAX] = {0}, regval = 0, hidden_2 = 0;

	ret = __rt9471_kick_wdt(chip);

	ret = __rt9471_get_mivr(chip, &mivr);
	ret = __rt9471_get_aicr(chip, &aicr);
	ret = __rt9471_get_cv(chip, &cv);
	ret = __rt9471_get_ichg(chip, &ichg);
	ret = __rt9471_get_ieoc(chip, &ieoc);
	ret = __rt9471_is_chg_enabled(chip, &chg_en);
	ret = __rt9471_get_ic_stat(chip, &ic_stat);
	ret = rt9471_i2c_block_read(chip, RT9471_REG_STAT0, RT9471_STATIDX_MAX,
				    stats);
	ret = rt9471_i2c_read_byte(chip, RT9471_REG_HIDDEN_2, &hidden_2);

	if (ic_stat == RT9471_ICSTAT_CHGFAULT) {
		for (i = 0; i < ARRAY_SIZE(rt9471_reg_addr); i++) {
			ret = rt9471_i2c_read_byte(chip, rt9471_reg_addr[i],
						   &regval);
			if (ret < 0)
				continue;
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
					      __func__, rt9471_reg_addr[i],
					      regval);
		}
	}

	dev_info(chip->dev, "%s MIVR = %dmV, AICR = %dmA\n",
		 __func__, mivr / 1000, aicr / 1000);

	dev_info(chip->dev, "%s CV = %dmV, ICHG = %dmA, IEOC = %dmA\n",
		 __func__, cv / 1000, ichg / 1000, ieoc / 1000);

	dev_info(chip->dev, "%s CHG_EN = %d, IC_STAT = %s\n",
		 __func__, chg_en, rt9471_ic_stat_names[ic_stat]);

	dev_info(chip->dev, "%s STAT0 = 0x%02X, STAT1 = 0x%02X\n", __func__,
		 stats[RT9471_STATIDX_STAT0], stats[RT9471_STATIDX_STAT1]);

	dev_info(chip->dev, "%s STAT2 = 0x%02X, STAT3 = 0x%02X\n", __func__,
		 stats[RT9471_STATIDX_STAT2], stats[RT9471_STATIDX_STAT3]);

	dev_info(chip->dev, "%s HIDDEN_2 = 0x%02X\n", __func__, hidden_2);

	return 0;
}

static void rt9471_buck_dwork_handler(struct work_struct *work)
{
	int ret = 0, i = 0;
	struct rt9471_chip *chip =
		container_of(work, struct rt9471_chip, buck_dwork.work);
	bool chg_rdy = false, chg_done = false, sys_min = false;
	u8 regval = 0;
	u8 reg_addrs[] = {RT9471_REG_BUCK_HDEN4, RT9471_REG_BUCK_HDEN1,
			  RT9471_REG_BUCK_HDEN2, RT9471_REG_BUCK_HDEN4,
			  RT9471_REG_BUCK_HDEN2, RT9471_REG_BUCK_HDEN1};
	u8 reg_vals[] = {0x77, 0x2F, 0xA2, 0x71, 0x22, 0x2D};

	dev_info(chip->dev, "%s\n", __func__);

	__pm_stay_awake(chip->buck_dwork_ws);

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STAT0, &regval);
	if (ret < 0)
		goto out;
	chg_rdy = (regval & RT9471_ST_CHGRDY_MASK ? true : false);
	chg_done = (regval & RT9471_ST_CHGDONE_MASK ? true : false);
	dev_info(chip->dev, "%s chg_rdy = %d\n", __func__, chg_rdy);
	dev_info(chip->dev, "%s chg_done = %d, chg_done_once = %d\n",
			    __func__, chg_done, chip->chg_done_once);
	if (!chg_rdy)
		goto out;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT2,
				  RT9471_ST_SYSMIN_SHIFT, &sys_min);
	if (ret < 0)
		goto out;
	dev_info(chip->dev, "%s sys_min = %d\n", __func__, sys_min);
	/* Should not enter CV tracking in sys_min */
	if (sys_min)
		reg_vals[1] = 0x2D;

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		goto out;

	for (i = 0; i < ARRAY_SIZE(reg_addrs); i++) {
		ret = rt9471_i2c_write_byte(chip, reg_addrs[i], reg_vals[i]);
		if (ret < 0)
			dev_notice(chip->dev,
				   "%s reg0x%02X = 0x%02X fail(%d)\n",
				   __func__, reg_addrs[i], reg_vals[i], ret);
		if (i == 1)
			udelay(1000);
	}

	rt9471_enable_hidden_mode(chip, false);

	if (chg_done && !chip->chg_done_once) {
		chip->chg_done_once = true;
		mod_delayed_work(system_wq, &chip->buck_dwork,
				 msecs_to_jiffies(100));
	}
out:
	__pm_relax(chip->buck_dwork_ws);
}

static bool rt9471_is_vbus_ready_for_chg(struct rt9471_chip *chip)
{
	int ret = 0;
	bool chg_rdy = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_CHGRDY_SHIFT, &chg_rdy);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_info(chip->dev, "%s chg_rdy = %d, chip_rev = %d\n",
			    __func__, chg_rdy, chip->chip_rev);

	return chg_rdy;
}

static bool rt9471_is_vbus_gd(struct rt9471_chip *chip)
{
	int ret = 0;
	bool vbus_gd = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_VBUSGD_SHIFT, &vbus_gd);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_dbg(chip->dev, "%s vbus_gd = %d\n", __func__, vbus_gd);

	return vbus_gd;
}

static int rt9471_detach_irq_handler(struct rt9471_chip *chip)
{
	bool vbus_gd = rt9471_is_vbus_gd(chip);

	dev_info(chip->dev, "%s vbus_gd = %d\n", __func__, vbus_gd);
	if (vbus_gd)
		goto out;

	complete(&chip->aicc_done);
	complete(&chip->pe_done);
out:
	return 0;
}

static int rt9471_rechg_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static void rt9471_bc12_done_handler(struct rt9471_chip *chip)
{

}

static int rt9471_bc12_done_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	rt9471_bc12_done_handler(chip);
	return 0;
}

static int rt9471_chg_done_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool chg_done = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_CHGDONE_SHIFT, &chg_done);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_info(chip->dev, "%s chg_done = %d, chip_rev = %d\n",
			    __func__, chg_done, chip->chip_rev);
	if (!chg_done || chip->chip_rev > 4)
		goto out;

	cancel_delayed_work_sync(&chip->buck_dwork);
	chip->chg_done_once = false;
	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));
out:
	return 0;
}

static int rt9471_bg_chg_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool bg_chg = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_BGCHG_SHIFT, &bg_chg);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_info(chip->dev, "%s bg_chg = %d\n", __func__, bg_chg);

	return 0;
}

static int rt9471_ieoc_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool ieoc = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_IEOC_SHIFT, &ieoc);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_info(chip->dev, "%s ieoc = %d\n", __func__, ieoc);
	if (!ieoc)
		goto out;

out:
	return 0;
}

static int rt9471_chg_rdy_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool chg_rdy = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_CHGRDY_SHIFT, &chg_rdy);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_info(chip->dev, "%s chg_rdy = %d, chip_rev = %d\n",
			    __func__, chg_rdy, chip->chip_rev);
	if (!chg_rdy || chip->chip_rev > 4)
		goto out;

	if (chip->chip_rev <= 3)
		rt9471_bc12_done_handler(chip);
	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));
out:
	return 0;
}

static int rt9471_vbus_gd_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool vbus_gd = false;

	ret = rt9471_enable_hz(chip, true, RT9471_HZU_VBUS_GD);
	if (ret < 0)
		dev_notice(chip->dev, "%s en hz fail(%d)\n", __func__, ret);

	vbus_gd = rt9471_is_vbus_gd(chip);
	dev_info(chip->dev, "%s vbus_gd = %d\n", __func__, vbus_gd);
	if (!vbus_gd)
		goto out;

out:
	return 0;
}

static int rt9471_chg_batov_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_sysov_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_tout_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_busuv_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_threg_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_aicr_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_mivr_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_sys_short_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_sys_min_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_aicc_done_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	complete(&chip->aicc_done);
	return 0;
}

static int rt9471_pe_done_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	complete(&chip->pe_done);
	return 0;
}

static int rt9471_jeita_cold_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_cool_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_warm_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_hot_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_otg_fault_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_otg_lbp_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_otg_cc_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_wdt_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return __rt9471_kick_wdt(chip);
}

static int rt9471_vac_ov_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool vac_ov = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT3, RT9471_ST_VACOV_SHIFT,
				  &vac_ov);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_info(chip->dev, "%s vac_ov = %d\n", __func__, vac_ov);

	return 0;
}

static int rt9471_otp_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

struct irq_mapping_tbl {
	const char *name;
	int (*hdlr)(struct rt9471_chip *chip);
	int num;
};

#define RT9471_IRQ_MAPPING(_name, _num) \
	{.name = #_name, .hdlr = rt9471_##_name##_irq_handler, .num = _num}

static const struct irq_mapping_tbl rt9471_irq_mapping_tbl[] = {
	RT9471_IRQ_MAPPING(wdt, 29),
	RT9471_IRQ_MAPPING(vbus_gd, 7),
	RT9471_IRQ_MAPPING(chg_rdy, 6),
	RT9471_IRQ_MAPPING(bc12_done, 0),
	RT9471_IRQ_MAPPING(detach, 1),
	RT9471_IRQ_MAPPING(rechg, 2),
	RT9471_IRQ_MAPPING(chg_done, 3),
	RT9471_IRQ_MAPPING(bg_chg, 4),
	RT9471_IRQ_MAPPING(ieoc, 5),
	RT9471_IRQ_MAPPING(chg_batov, 9),
	RT9471_IRQ_MAPPING(chg_sysov, 10),
	RT9471_IRQ_MAPPING(chg_tout, 11),
	RT9471_IRQ_MAPPING(chg_busuv, 12),
	RT9471_IRQ_MAPPING(chg_threg, 13),
	RT9471_IRQ_MAPPING(chg_aicr, 14),
	RT9471_IRQ_MAPPING(chg_mivr, 15),
	RT9471_IRQ_MAPPING(sys_short, 16),
	RT9471_IRQ_MAPPING(sys_min, 17),
	RT9471_IRQ_MAPPING(aicc_done, 18),
	RT9471_IRQ_MAPPING(pe_done, 19),
	RT9471_IRQ_MAPPING(jeita_cold, 20),
	RT9471_IRQ_MAPPING(jeita_cool, 21),
	RT9471_IRQ_MAPPING(jeita_warm, 22),
	RT9471_IRQ_MAPPING(jeita_hot, 23),
	RT9471_IRQ_MAPPING(otg_fault, 24),
	RT9471_IRQ_MAPPING(otg_lbp, 25),
	RT9471_IRQ_MAPPING(otg_cc, 26),
	RT9471_IRQ_MAPPING(vac_ov, 30),
	RT9471_IRQ_MAPPING(otp, 31),
};

static irqreturn_t rt9471_irq_handler(int irq, void *data)
{
	int ret = 0, i = 0, irqnum = 0, irqbit = 0;
	u8 evt[RT9471_IRQIDX_MAX] = {0};
	u8 mask[RT9471_IRQIDX_MAX] = {0};
	struct rt9471_chip *chip = (struct rt9471_chip *)data;

	dev_info(chip->dev, "%s\n", __func__);

	pm_stay_awake(chip->dev);

	schedule_work(&chip->charge_detect_work);

	ret = rt9471_i2c_block_read(chip, RT9471_REG_IRQ0, RT9471_IRQIDX_MAX,
				    evt);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read evt fail(%d)\n", __func__, ret);
		goto out;
	}

	ret = rt9471_i2c_block_read(chip, RT9471_REG_MASK0, RT9471_IRQIDX_MAX,
				    mask);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read mask fail(%d)\n", __func__, ret);
		goto out;
	}

	for (i = 0; i < RT9471_IRQIDX_MAX; i++)
		evt[i] &= ~mask[i];
	for (i = 0; i < ARRAY_SIZE(rt9471_irq_mapping_tbl); i++) {
		irqnum = rt9471_irq_mapping_tbl[i].num / 8;
		if (irqnum >= RT9471_IRQIDX_MAX)
			continue;
		irqbit = rt9471_irq_mapping_tbl[i].num % 8;
		if (evt[irqnum] & (1 << irqbit))
			rt9471_irq_mapping_tbl[i].hdlr(chip);
	}
out:
	pm_relax(chip->dev);
	return IRQ_HANDLED;
}

static int rt9471_register_irq(struct rt9471_chip *chip)
{
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = devm_gpio_request_one(chip->dev, chip->intr_gpio, GPIOF_DIR_IN,
			devm_kasprintf(chip->dev, GFP_KERNEL,
			"rt9471_intr_gpio.%s", dev_name(chip->dev)));
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	chip->irq = gpio_to_irq(chip->intr_gpio);
	if (chip->irq < 0) {
		dev_notice(chip->dev, "%s gpio2irq fail(%d)\n",
				      __func__, chip->irq);
		return chip->irq;
	}
	dev_info(chip->dev, "%s irq = %d\n", __func__, chip->irq);

	/* Request threaded IRQ */
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					rt9471_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					devm_kasprintf(chip->dev, GFP_KERNEL,
					"rt9471_irq.%s", dev_name(chip->dev)),
					chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s request threaded irq fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	device_init_wakeup(chip->dev, true);

	return ret;
}

static int rt9471_init_irq(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return rt9471_i2c_block_write(chip, RT9471_REG_MASK0,
				      ARRAY_SIZE(chip->irq_mask),
				      chip->irq_mask);
}

static inline int rt9471_get_irq_number(struct rt9471_chip *chip,
					const char *name)
{
	int i = 0;

	if (!name) {
		dev_notice(chip->dev, "%s null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rt9471_irq_mapping_tbl); i++) {
		if (!strcmp(name, rt9471_irq_mapping_tbl[i].name))
			return rt9471_irq_mapping_tbl[i].num;
	}

	return -EINVAL;
}

static inline const char *rt9471_get_irq_name(int irqnum)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(rt9471_irq_mapping_tbl); i++) {
		if (rt9471_irq_mapping_tbl[i].num == irqnum)
			return rt9471_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void rt9471_irq_mask(struct rt9471_chip *chip, int irqnum)
{
	dev_dbg(chip->dev, "%s irq(%d, %s)\n", __func__, irqnum,
		rt9471_get_irq_name(irqnum));
	chip->irq_mask[irqnum / 8] |= (1 << (irqnum % 8));
}

static inline void rt9471_irq_unmask(struct rt9471_chip *chip, int irqnum)
{
	dev_info(chip->dev, "%s irq(%d, %s)\n", __func__, irqnum,
		 rt9471_get_irq_name(irqnum));
	chip->irq_mask[irqnum / 8] &= ~(1 << (irqnum % 8));
}

static int rt9471_parse_dt(struct rt9471_chip *chip)
{
	int ret = 0, irqcnt = 0, irqnum = 0;
	struct device_node *parent_np = chip->dev->of_node, *np = NULL;
	struct rt9471_desc *desc = NULL;
	const char *name = NULL;

	dev_info(chip->dev, "%s\n", __func__);

	chip->desc = &rt9471_default_desc;

	if (!parent_np) {
		dev_notice(chip->dev, "%s no device node\n", __func__);
		return -EINVAL;
	}
	np = of_get_child_by_name(parent_np, "rt9471");
	if (!np) {
		dev_info(chip->dev, "%s no rt9471 device node\n", __func__);
		np = parent_np;
	}

	desc = devm_kmemdup(chip->dev, &rt9471_default_desc,
			    sizeof(rt9471_default_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	chip->desc = desc;

	ret = of_property_read_string(np, "mmi,chg_dev_name", &chip->mmi_chg_dev_name);
	if (ret < 0) {
		chip->mmi_chg_dev_name = "master_chg";
		dev_info(chip->dev, "%s no chg_name(%d)\n", __func__, ret);
	}

	chip->mmi_qc3_support = of_property_read_bool(chip->dev->of_node, "mmi,qc3-support");

	ret = of_property_read_string(np, "chg_name", &desc->chg_name);
	if (ret < 0)
		dev_info(chip->dev, "%s no chg_name(%d)\n", __func__, ret);

	ret = of_property_read_string(np, "chg_alias_name",
				      &chip->chg_props.alias_name);
	if (ret < 0) {
		dev_info(chip->dev, "%s no chg_alias_name(%d)\n",
				    __func__, ret);
		chip->chg_props.alias_name = "rt9471_chg";
	}
	dev_info(chip->dev, "%s name = %s, alias name = %s\n", __func__,
			    desc->chg_name, chip->chg_props.alias_name);

	if (strcmp(desc->chg_name, "primary_chg") == 0)
		chip->is_primary = true;

#if !defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND)
	ret = of_get_named_gpio(parent_np, "rt,intr_gpio", 0);
	if (ret < 0) {
		dev_notice(chip->dev, "%s no rt,intr_gpio(%d)\n",
				      __func__, ret);
		return ret;
	} else
		chip->intr_gpio = ret;

	ret = of_get_named_gpio(parent_np, "rt,ceb_gpio", 0);
	if (ret < 0) {
		dev_info(chip->dev, "%s no rt,ceb_gpio(%d)\n",
				    __func__, ret);
		chip->ceb_gpio = U32_MAX;
	} else
		chip->ceb_gpio = ret;
#else
	ret = of_property_read_u32(parent_np, "rt,intr_gpio_num",
				   &chip->intr_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s no rt,intr_gpio_num(%d)\n",
				      __func__, ret);
		return ret;
	}

	ret = of_property_read_u32(parent_np, "rt,ceb_gpio_num",
				   &chip->ceb_gpio);
	if (ret < 0) {
		dev_info(chip->dev, "%s no rt,ceb_gpio_num(%d)\n",
				    __func__, ret);
		chip->ceb_gpio = U32_MAX;
	}
#endif
	dev_info(chip->dev, "%s intr_gpio = %u, ceb_gpio = %u\n",
			    __func__, chip->intr_gpio, chip->ceb_gpio);

	if (chip->ceb_gpio != U32_MAX) {
		ret = devm_gpio_request_one(
				chip->dev, chip->ceb_gpio, GPIOF_DIR_OUT,
				devm_kasprintf(chip->dev, GFP_KERNEL,
				"rt9471_ceb_gpio.%s", dev_name(chip->dev)));
		if (ret < 0) {
			dev_notice(chip->dev, "%s gpio request fail(%d)\n",
					      __func__, ret);
			return ret;
		}
	}

	/* Register map */
	ret = of_property_read_u8(np, "rm-dev-addr", &desc->rm_dev_addr);
	if (ret < 0)
		dev_info(chip->dev, "%s no rm-dev-addr(%d)\n", __func__, ret);
	ret = of_property_read_string(np, "rm-name", &desc->rm_name);
	if (ret < 0)
		dev_info(chip->dev, "%s no rm-name(%d)\n", __func__, ret);

	/* Charger parameter */
	ret = of_property_read_u32(np, "vac_ovp", &desc->vac_ovp);
	if (ret < 0)
		dev_info(chip->dev, "%s no vac_ovp(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "mivr", &desc->mivr);
	if (ret < 0)
		dev_info(chip->dev, "%s no mivr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "aicr", &desc->aicr);
	if (ret < 0)
		dev_info(chip->dev, "%s no aicr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "cv", &desc->cv);
	if (ret < 0)
		dev_info(chip->dev, "%s no cv(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "ichg", &desc->ichg);
	if (ret < 0)
		dev_info(chip->dev, "%s no ichg(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "ieoc", &desc->ieoc) < 0;
	if (ret < 0)
		dev_info(chip->dev, "%s no ieoc(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "safe_tmr", &desc->safe_tmr);
	if (ret < 0)
		dev_info(chip->dev, "%s no safe_tmr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "wdt", &desc->wdt);
	if (ret < 0)
		dev_info(chip->dev, "%s no wdt(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "mivr_track", &desc->mivr_track);
	if (ret < 0)
		dev_info(chip->dev, "%s no mivr_track(%d)\n", __func__, ret);
	if (desc->mivr_track >= RT9471_MIVRTRACK_MAX)
		desc->mivr_track = RT9471_MIVRTRACK_VBAT_300MV;

	desc->en_safe_tmr = of_property_read_bool(np, "en_safe_tmr");
	desc->en_te = of_property_read_bool(np, "en_te");
	desc->en_jeita = of_property_read_bool(np, "en_jeita");
	desc->ceb_invert = of_property_read_bool(np, "ceb_invert");
	desc->dis_i2c_tout = of_property_read_bool(np, "dis_i2c_tout");
	desc->en_qon_rst = of_property_read_bool(np, "en_qon_rst");
	desc->auto_aicr = of_property_read_bool(np, "auto_aicr");

	memcpy(chip->irq_mask, rt9471_irq_maskall, RT9471_IRQIDX_MAX);
	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
						    irqcnt, &name);
		if (ret < 0)
			break;
		irqcnt++;
		irqnum = rt9471_get_irq_number(chip, name);
		if (irqnum >= 0)
			rt9471_irq_unmask(chip, irqnum);
	}

	return 0;
}

static int rt9471_check_chg(struct rt9471_chip *chip)
{
	int ret = 0;
	u8 regval = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STAT0, &regval);
	if (ret < 0)
		return ret;

	if (regval & RT9471_ST_VBUSGD_MASK)
		rt9471_vbus_gd_irq_handler(chip);
	if (regval & RT9471_ST_CHGDONE_MASK)
		rt9471_chg_done_irq_handler(chip);
	else if (regval & RT9471_ST_CHGRDY_MASK)
		rt9471_chg_rdy_irq_handler(chip);

	return ret;
}

static int rt9471_sw_workaround(struct rt9471_chip *chip)
{
	int ret = 0;
	u8 regval = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		return ret;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_HIDDEN_0, &regval);
	if (ret < 0)
		goto out;

	chip->chip_rev = (regval & RT9471_CHIP_REV_MASK) >>
			 RT9471_CHIP_REV_SHIFT;
	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	/* OTG load transient improvement */
	if (chip->chip_rev <= 3)
		ret = rt9471_i2c_update_bits(chip, RT9471_REG_OTG_HDEN2, 0x10,
					     RT9471_REG_OTG_RES_COMP_MASK);

out:
	rt9471_enable_hidden_mode(chip, false);
	return ret;
}

static int rt9471_init_setting(struct rt9471_chip *chip)
{
	int ret = 0;
	struct rt9471_desc *desc = chip->desc;
	u8 evt[RT9471_IRQIDX_MAX] = {0};

	dev_info(chip->dev, "%s\n", __func__);

	/* Mask all IRQs */
	ret = rt9471_i2c_block_write(chip, RT9471_REG_MASK0,
				     ARRAY_SIZE(rt9471_irq_maskall),
				     rt9471_irq_maskall);
	if (ret < 0)
		dev_notice(chip->dev, "%s mask irq fail(%d)\n", __func__, ret);

	/* Clear all IRQs */
	ret = rt9471_i2c_block_read(chip, RT9471_REG_IRQ0, RT9471_IRQIDX_MAX,
				    evt);
	if (ret < 0)
		dev_notice(chip->dev, "%s clear irq fail(%d)\n", __func__, ret);

	ret = __rt9471_set_vac_ovp(chip, desc->vac_ovp);
	if (ret < 0)
		dev_notice(chip->dev, "%s set vac ovp fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_set_mivr(chip, desc->mivr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivr fail(%d)\n", __func__, ret);

	ret = __rt9471_set_aicr(chip, desc->aicr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set aicr fail(%d)\n", __func__, ret);

	ret = __rt9471_set_cv(chip, desc->cv);
	if (ret < 0)
		dev_notice(chip->dev, "%s set cv fail(%d)\n", __func__, ret);

	ret = __rt9471_set_ichg(chip, desc->ichg);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ichg fail(%d)\n", __func__, ret);

	ret = __rt9471_set_ieoc(chip, desc->ieoc);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ieoc fail(%d)\n", __func__, ret);

	ret = __rt9471_reset_eoc_state(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s reset eoc state fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_set_safe_tmr(chip, desc->safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_set_mivrtrack(chip, desc->mivr_track);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivrtrack fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_safe_tmr(chip, desc->en_safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s en safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_te(chip, desc->en_te);
	if (ret < 0)
		dev_notice(chip->dev, "%s en te fail(%d)\n", __func__, ret);

	ret = __rt9471_enable_jeita(chip, desc->en_jeita);
	if (ret < 0)
		dev_notice(chip->dev, "%s en jeita fail(%d)\n", __func__, ret);

	ret = __rt9471_disable_i2c_tout(chip, desc->dis_i2c_tout);
	if (ret < 0)
		dev_notice(chip->dev, "%s dis i2c tout fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_qon_rst(chip, desc->en_qon_rst);
	if (ret < 0)
		dev_notice(chip->dev, "%s en qon rst fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_autoaicr(chip, desc->auto_aicr);
	if (ret < 0)
		dev_notice(chip->dev, "%s en autoaicr fail(%d)\n",
				      __func__, ret);

	ret = rt9471_sw_workaround(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s sw workaround fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_bc12(chip, false);
	if (ret < 0)
		dev_notice(chip->dev, "%s dis bc12 fail(%d)\n", __func__, ret);

	/*
	 * Customization for MTK platform
	 * Primary charger: HZ controlled by sink vbus with TCPC enabled,
	 *		    CHG_EN controlled by charging algorithm
	 * Secondary charger: HZ=0 and CHG_EN=1 at needed,
	 *		      e.x.: PE10, PE20, etc...
	 */
	if (!chip->is_primary) {
		ret = rt9471_enable_hz(chip, true, RT9471_HZU_PP);
		if (ret < 0)
			dev_notice(chip->dev, "%s en hz fail(%d)\n",
					      __func__, ret);
		ret = __rt9471_enable_chg(chip, false);
		if (ret < 0)
			dev_notice(chip->dev, "%s dis chg fail(%d)\n",
					      __func__, ret);
		chip->enter_shipping_mode = true;
	}

	return 0;
}

static int rt9471_reset_register(struct rt9471_chip *chip)
{
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);

	mutex_lock(&chip->hz_lock);
	chip->hz_users[RT9471_HZU_PP] = false;
	chip->hz_users[RT9471_HZU_BC12] = false;
	chip->hz_users[RT9471_HZU_OTG] = true;
	chip->hz_users[RT9471_HZU_VBUS_GD] = true;
	ret = rt9471_set_bit(chip, RT9471_REG_INFO, RT9471_REGRST_MASK);
	mutex_unlock(&chip->hz_lock);
	if (ret < 0)
		return ret;
#ifdef CONFIG_RT_REGMAP
	rt_regmap_cache_reload(chip->rm_dev);
#endif /* CONFIG_RT_REGMAP */
	ret = __rt9471_enable_autoaicr(chip, false);
	if (ret < 0)
		return ret;

	return __rt9471_set_wdt(chip, 0);
}

static bool rt9471_check_devinfo(struct rt9471_chip *chip)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, RT9471_REG_INFO);
	if (ret < 0) {
		dev_notice(chip->dev, "%s get devinfo fail(%d)\n",
				      __func__, ret);
		return false;
	}
	chip->dev_id = (ret & RT9471_DEVID_MASK) >> RT9471_DEVID_SHIFT;
	switch (chip->dev_id) {
	case RT9470_DEVID:
	case RT9470D_DEVID:
	case RT9471_DEVID:
	case RT9471D_DEVID:
		break;
	default:
		dev_notice(chip->dev, "%s incorrect devid 0x%02X\n",
				      __func__, chip->dev_id);
		return false;
	}
	chip->dev_rev = (ret & RT9471_DEVREV_MASK) >> RT9471_DEVREV_SHIFT;
	dev_info(chip->dev, "%s id = 0x%02X, rev = 0x%02X\n",
			    __func__, chip->dev_id, chip->dev_rev);

	return true;
}

static int get_charger_type(struct rt9471_chip *chip)
{
	enum power_supply_usb_type usb_type;

	switch(chip->real_charger_type) {
		case POWER_SUPPLY_TYPE_USB:
			usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			break;

		case POWER_SUPPLY_TYPE_USB_CDP:
			usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			break;

		case POWER_SUPPLY_TYPE_USB_DCP:
		case POWER_SUPPLY_TYPE_USB_HVDCP:
		case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
			usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			break;

		default:
			usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
	}
	pr_err("%s usb_type:%d\n",__func__,usb_type);

	return usb_type;
}

static int rt9471_read_usbin_voltage_chan(struct rt9471_chip *chip, int *val)
{
	int rc;

	if (!chip->iio.usbin_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chip->iio.usbin_v_chan, val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBIN channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int rt9471_get_usb_voltage_now(struct rt9471_chip *chip, int *val)
{
	int rc;
	int raw_date;

	rc = rt9471_read_usbin_voltage_chan(chip, &raw_date);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBIN over vadc rc=%d\n", rc);
		return rc;
	}

	*val = raw_date * 1000;

	return 0;
}

static int rt9471_get_iio_channel(struct rt9471_chip *chip, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chip->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chip->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			dev_err(chip->dev, "%s channel unavailable, %d\n",
							propname, rc);
		*chan = NULL;
	}

	return rc;
}

static int rt9471_parse_dt_adc_channels(struct rt9471_chip *chip)
{
	int rc = 0;

	rc = rt9471_get_iio_channel(chip, "read_usbin_voltage",
					&chip->iio.usbin_v_chan);

	if (rc < 0)
		return rc;

	return 0;
}

static int rt9471_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct rt9471_chip *chip = power_supply_get_drvdata(psy);
	int ret = 0;
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_SLEEP;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		__rt9471_get_ic_stat(chip, &ic_stat);
		if (ic_stat == RT9471_ICSTAT_TRICKLECHG ||
			ic_stat == RT9471_ICSTAT_PRECHG ||
			ic_stat == RT9471_ICSTAT_FASTCHG ||
			ic_stat == RT9471_ICSTAT_IEOC ||
			ic_stat == RT9471_ICSTAT_BGCHG) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ic_stat == RT9471_ICSTAT_CHGFAULT ||
					ic_stat == RT9471_ICSTAT_VBUSRDY) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else if (ic_stat == RT9471_ICSTAT_SLEEP ||
					ic_stat == RT9471_ICSTAT_OTG){
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else if (ic_stat == RT9471_ICSTAT_CHGDONE){
			val->intval = POWER_SUPPLY_STATUS_FULL;
		} else {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		__rt9471_get_ic_stat(chip, &ic_stat);
		switch(ic_stat) {
			case RT9471_ICSTAT_TRICKLECHG:
			case RT9471_ICSTAT_PRECHG:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
				break;
			case RT9471_ICSTAT_FASTCHG:
			case RT9471_ICSTAT_IEOC:
			case RT9471_ICSTAT_BGCHG:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
				break;
			case RT9471_ICSTAT_SLEEP:
			case RT9471_ICSTAT_VBUSRDY:
			case RT9471_ICSTAT_CHGDONE:
			case RT9471_ICSTAT_CHGFAULT:
			case RT9471_ICSTAT_OTG:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
				break;
			default:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "richtek";
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		switch (chip->dev_id) {
			case RT9470_DEVID:
				val->strval = "RT9470";
				break;
			case RT9470D_DEVID:
				val->strval = "RT9470D";
				break;
			case RT9471_DEVID:
				val->strval = "RT9471";
				break;
			case RT9471D_DEVID:
				val->strval = "RT9471D";
				break;
			default:
				val->strval = "UNKNOWN";
		}
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->state.online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->state.vbus_gd;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = rt9471_power_supply_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = get_charger_type(chip);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rt9471_get_usb_voltage_now(chip, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//val->intval = state.ibus_adc;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		__rt9471_get_mivr(chip, &val->intval);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		__rt9471_get_aicr(chip, &val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int rt9471_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct rt9471_chip *chip = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = rt9471_set_aicr(chip->chg_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = __rt9471_set_mivr(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int rt9471_property_is_writeable(struct power_supply *psy,
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

static enum power_supply_property rt9471_power_supply_props[] = {
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

static char *rt9471_charger_supplied_to[] = {
	"battery",
};

static struct power_supply_desc rt9471_power_supply_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.usb_types = rt9471_usb_type,
	.num_usb_types = ARRAY_SIZE(rt9471_usb_type),
	.properties = rt9471_power_supply_props,
	.num_properties = ARRAY_SIZE(rt9471_power_supply_props),
	.get_property = rt9471_charger_get_property,
	.set_property = rt9471_charger_set_property,
	.property_is_writeable = rt9471_property_is_writeable,
};

static int rt9471_power_supply_init(struct rt9471_chip *chip, struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = chip,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = rt9471_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(rt9471_charger_supplied_to);

	chip->charger = devm_power_supply_register(chip->dev,
						 &rt9471_power_supply_desc,
						 &psy_cfg);
	if (IS_ERR(chip->charger))
		return -EINVAL;

	return 0;
}

static int rt9471_enable_charging(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = __rt9471_enable_chg(chip, en);
	if (ret < 0)
		dev_notice(chip->dev, "%s en chg fail(%d)\n", __func__, ret);

	return ret;
}

static int rt9471_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_is_chg_enabled(chip, en);
}

static int rt9471_get_aicr(struct charger_device *chg_dev, u32 *uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_get_aicr(chip, uA);
}

static int rt9471_enable_hw_jeita(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_enable_jeita(chip, en);
}

static int rt9471_set_aicr(struct charger_device *chg_dev, u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s uA = %d\n", __func__, uA);

	chip->input_current_cache = uA;

	if (chip->mmi_is_qc3_authen) {
		dev_info(chip->dev, "hvdcp detecting now\n");
		return 0;
	}

	return __rt9471_set_aicr(chip, uA);
}

static int rt9471_set_cv(struct charger_device *chg_dev, u32 uV)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_set_cv(chip, uV);
}

static int rt9471_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_set_ichg(chip, uA);
}

static int rt9471_enable_te(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;

	if (en) {
        	ret = __rt9471_eoc_rst(chip);
		if (ret < 0) {
			dev_notice(chip->dev, "%s __rt9471_eoc_rst fail(%d)\n", __func__, ret);
			return ret;
		}

		ret = __rt9471_enable_te(chip, en);
		if (ret < 0) {
			dev_notice(chip->dev, "%s __rt9471_enable_te fail(%d)\n", __func__, ret);
			return ret;
                }
        } else {
        	ret = __rt9471_enable_te(chip, en);
		if (ret < 0) {
			dev_notice(chip->dev, "%s __rt9471_enable_te fail(%d)\n", __func__, ret);
			return ret;
                }

		ret = __rt9471_eoc_rst(chip);
		if (ret < 0) {
			dev_notice(chip->dev, "%s __rt9471_eoc_rst fail(%d)\n", __func__, ret);
			return ret;
		}
        }

	return ret;
}

static int rt9471_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en) {
		ret = __rt9471_set_wdt(chip, chip->desc->wdt);
		if (ret < 0) {
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
			return ret;
		}
	}

	ret = __rt9471_enable_otg(chip, en);
	if (ret < 0) {
		dev_notice(chip->dev, "%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}

	if (!en) {
		ret = __rt9471_set_wdt(chip, 0);
		if (ret < 0)
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
	}

	return ret;
}

#if 0
static int rt9471_enable_discharge(struct charger_device *chg_dev, bool en)	//disable otg
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		return ret;

	ret = (en ? rt9471_set_bit : rt9471_clr_bit)(chip,
		RT9471_REG_TOP_HDEN, RT9471_FORCE_EN_VBUS_SINK_MASK);
	if (ret < 0)
		dev_notice(chip->dev, "%s en = %d fail(%d)\n",
				      __func__, en, ret);

	rt9471_enable_hidden_mode(chip, false);

	return ret;
}
#endif

static int rt9471_set_boost_current_limit(struct charger_device *chg_dev,
					  u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_set_otgcc(chip, uA);
}

static int rt9471_dump_registers(struct charger_device *chg_dev, struct seq_file *m)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	u8 addr;
	u8 val;
	int ret;

	for(addr = 0x0; addr < 0x14; addr++) {
		ret = rt9471_i2c_read_byte(chip, addr, &val);
		if (ret)
			continue;
		seq_printf(m, "%s Reg[0x%.2x] = 0x%.2x\n", __func__, addr, val);

	}

	return ret;
}

static int rt9471_request_dpdm(struct rt9471_chip *chip, bool enable)
{
	int rc = 0;

	mutex_lock(&chip->regulator_lock);
		/* fetch the DPDM regulator */
	if (!chip->dpdm_reg && of_get_property(chip->dev->of_node,
				"dpdm-supply", NULL)) {
		chip->dpdm_reg = devm_regulator_get(chip->dev, "dpdm");
		if (IS_ERR(chip->dpdm_reg)) {
			rc = PTR_ERR(chip->dpdm_reg);
			dev_err(chip->dev, "Couldn't get dpdm regulator rc=%d\n", rc);
			chip->dpdm_reg = NULL;
			mutex_unlock(&chip->regulator_lock);
			return rc;
		}
	}

	if (enable) {
		if (chip->dpdm_reg && !chip->dpdm_enabled) {
			dev_err(chip->dev, "enabling DPDM regulator\n");
			rc = regulator_enable(chip->dpdm_reg);
			if (rc < 0)
				dev_err(chip->dev,
					"Couldn't enable dpdm regulator rc=%d\n",
					rc);
			else
				chip->dpdm_enabled = true;
		}
	} else {
		if (chip->dpdm_reg && chip->dpdm_enabled) {
			dev_err(chip->dev, "disabling DPDM regulator\n");
			rc = regulator_disable(chip->dpdm_reg);
			if (rc < 0)
				dev_err(chip->dev,
					"Couldn't disable dpdm regulator rc=%d\n",
					rc);
			else
				chip->dpdm_enabled = false;
		}
	}
	mutex_unlock(&chip->regulator_lock);

	return rc;
}

#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
bool is_chan_valid(struct rt9471_chip *chip,
		enum mmi_qc3p_ext_iio_channels chan)
{
	int rc;

	if (IS_ERR(chip->ext_iio_chans[chan]))
		return false;

	if (!chip->ext_iio_chans[chan]) {
		chip->ext_iio_chans[chan] = iio_channel_get(chip->dev,
					mmi_qc3p_ext_iio_chan_name[chan]);
		if (IS_ERR(chip->ext_iio_chans[chan])) {
			rc = PTR_ERR(chip->ext_iio_chans[chan]);
			if (rc == -EPROBE_DEFER)
				chip->ext_iio_chans[chan] = NULL;

			dev_info(chip->dev, "Failed to get IIO channel %s, rc=%d\n",
				mmi_qc3p_ext_iio_chan_name[chan], rc);
			return false;
		}
	}

	return true;
}

int mmi_charger_read_iio_chan(struct rt9471_chip *chip,
	enum mmi_qc3p_ext_iio_channels chan, int *val)
{
	int rc;

	if (is_chan_valid(chip, chan)) {
		rc = iio_read_channel_processed(
				chip->ext_iio_chans[chan], val);
		return (rc < 0) ? rc : 0;
	}

	return -EINVAL;
}

int mmi_charger_write_iio_chan(struct rt9471_chip *chip,
	enum mmi_qc3p_ext_iio_channels chan, int val)
{
	if (is_chan_valid(chip, chan))
		return iio_write_channel_raw(chip->ext_iio_chans[chan],
						val);

	return -EINVAL;
}

static int mmi_init_iio_psy(struct rt9471_chip *chip,
				struct device *dev)
{
	chip->ext_iio_chans = devm_kcalloc(chip->dev,
				ARRAY_SIZE(mmi_qc3p_ext_iio_chan_name),
				sizeof(*chip->ext_iio_chans),
				GFP_KERNEL);
	if (!chip->ext_iio_chans)
		return -ENOMEM;

	return 0;
}

int qc3p_start_detection(struct rt9471_chip *chip)
{
	int ret = 0;
	ret = mmi_charger_write_iio_chan(chip, SMB5_QC3P_START_DETECT, true);
	if(ret )
		dev_err(chip->dev, "Cann't write SMB5_QC3P_START_DETECT IIO\n");

	dev_info(chip->dev, "write SMB5_QC3P_START_DETECT IIO success\n");
	return 0;
}

bool qc3p_detection_done(struct rt9471_chip *chip)
{
	int ret = 0;
	int val = 0;
	int delay_count =0;

	do {
		ret = mmi_charger_read_iio_chan(chip, SMB5_QC3P_DETECTION_READY, &val);
		if(ret )
			dev_err(chip->dev, "Cann't read SMB5_QC3P_DETECTION_READY IIO\n");

		dev_info(chip->dev, "read SMB5_QC3P_DETECTION_READY IIO :%d\n",val);

		msleep(100);
		delay_count ++;
	}while(val == false && delay_count <= 35);

	dev_info(chip->dev, "read SMB5_QC3P_DETECTION_READY IIO :%d\n",val);
	return val;
}

int qc3p_read_charger_type(struct rt9471_chip *chip)
{
	int ret = 0;
	int val = 0;

	ret = mmi_charger_read_iio_chan(chip, SMB5_USB_REAL_TYPE, &val);
	if(ret )
		dev_err(chip->dev, "Cann't read SMB5_USB_REAL_TYPE IIO\n");

	dev_info(chip->dev, "read SMB5_USB_REAL_TYPE IIO :%d\n",val);
	return val;
}

bool qc3p_update_policy(struct rt9471_chip *chip)
{
	int ret = 0;
	int val = 0;

	ret = mmi_charger_write_iio_chan(chip, SMB5_QC3P_START_POLICY, val);
	if(ret )
		dev_err(chip->dev, "Cann't write SMB5_QC3P_START_POLICY IIO\n");

	dev_info(chip->dev, "write SMB5_QC3P_START_POLICY IIO :%d\n",val);
	return val;
}

int bc12_start_detection(struct rt9471_chip *chip)
{
	int ret = 0;
	ret = mmi_charger_write_iio_chan(chip, SMB5_BC12_START_DETECT, true);
	if(ret )
		dev_err(chip->dev, "Cann't write SMB5_BC12_START_DETECT IIO\n");

	dev_info(chip->dev, "write SMB5_BC12_START_DETECT IIO success\n");
	return 0;
}

bool bc12_detection_done(struct rt9471_chip *chip)
{
	int ret = 0;
	int val = 0;
	int delay_count =0;

	do {
		ret = mmi_charger_read_iio_chan(chip, SMB5_BC12_DETECTION_READY, &val);
		if(ret )
			dev_err(chip->dev, "Cann't read SMB5_BC12_DETECTION_READY IIO\n");

		dev_info(chip->dev, "read SMB5_BC12_DETECTION_READY IIO :%d\n",val);
		if (val)
			break;

		msleep(50);
		delay_count ++;
	}while(val == false && delay_count <= 25);

	dev_info(chip->dev, "read SMB5_BC12_DETECTION_READY IIO :%d\n",val);
	return val;
}

int bc12_read_charger_type(struct rt9471_chip *chip)
{
	int ret = 0;
	int val = 0;

	ret = mmi_charger_read_iio_chan(chip, SMB5_USB_REAL_TYPE, &val);
	if(ret )
		dev_err(chip->dev, "Cann't read SMB5_USB_REAL_TYPE IIO\n");

	dev_info(chip->dev, "read SMB5_USB_REAL_TYPE IIO :%d\n",val);
	return val;
}
#endif

static void rt9471_monitor_workfunc(struct work_struct *work)
{
	struct rt9471_chip *chip = container_of(work, struct rt9471_chip, monitor_work.work);

	dev_info(chip->dev, "%s\n", __func__);

	__rt9471_dump_registers(chip);

	schedule_delayed_work(&chip->monitor_work, 10*HZ);
}

#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
static int mmi_hvdcp_detect_kthread(void *param)
{
	struct rt9471_chip *chip = param;
	int ret;
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	struct sched_param sch_param = {.sched_priority = MAX_RT_PRIO-1};

	sched_setscheduler(current, SCHED_FIFO, &sch_param);

	do {

		wait_event_interruptible(chip->mmi_qc3_wait_que, chip->mmi_qc3_trig_flag || kthread_should_stop());
		if (kthread_should_stop())
			break;

		dev_info(chip->dev, "mmi_hvdcp_detect_kthread begin\n");
		chip->mmi_qc3_trig_flag = false;
		chip->mmi_is_qc3_authen = true;
		__rt9471_set_aicr(chip, MMI_HVDCP_DETECT_ICL_LIMIT);

		rt9471_enable_te(chip->chg_dev, true);

		qc3p_start_detection(chip);
		qc3p_detection_done(chip);
		ret = qc3p_read_charger_type(chip);

		switch(ret) {
			case WT_CHG_TYPE_QC2:
				charger_type = POWER_SUPPLY_TYPE_USB_HVDCP;
				dev_info(chip->dev, "HDVCP 2.0 detected\n");
				break;
			case WT_CHG_TYPE_QC3:
			case WT_CHG_TYPE_QC3P_18W:
				charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3;
				dev_info(chip->dev, "HDVCP 3.0 or qc3p 18W detected\n");
				break;
			case WT_CHG_TYPE_QC3P_27W:
				dev_err(chip->dev, "QC3P 27W have been detected !\n");
				charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3P5;
#ifdef CONFIG_MMI_QC3P_TURBO_CHARGER
				qc3p_update_policy(chip);
#endif
				break;
			default:
				dev_info(chip->dev, "No HDVCP detected\n");
				goto out;
		}

		if (!rt9471_is_vbus_gd(chip))
			goto out;

		chip->real_charger_type = charger_type;
		//notify charging policy to update charger type
		chip->chg_dev->noti.hvdcp_done = true;
		charger_dev_notify(chip->chg_dev);

out:
		__rt9471_set_aicr(chip, chip->input_current_cache);
		chip->mmi_is_qc3_authen = false;
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		dev_info(chip->dev, "mmi_hvdcp_detect_kthread end\n");
	}while(!kthread_should_stop());

	dev_dbg(chip->dev, "qc3 kthread stop\n");
	return 0;
}

static void mmi_start_hvdcp_detect(struct rt9471_chip *chip)
{

	if (chip->mmi_qc3_support
		&& chip->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		dev_info(chip->dev, "start hvdcp detect\n");
		chip->mmi_qc3_trig_flag = true;
		wake_up_interruptible(&chip->mmi_qc3_wait_que);
	}
}
#endif

static void rt9471_plug_in_func(struct rt9471_chip *chip)
{
#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	int ret;

	rt9471_request_dpdm(chip, true);
	bc12_start_detection(chip);
	bc12_detection_done(chip);
	ret = bc12_read_charger_type(chip);

	switch(ret)	{
		case WT_CHG_TYPE_FC:
		case WT_CHG_TYPE_OCP:
			dev_err(chip->dev, "float type have been detected !\n");
			chip->real_charger_type = POWER_SUPPLY_TYPE_USB_FLOAT;
			break;
		case WT_CHG_TYPE_SDP:
			dev_err(chip->dev, "SDP have been detected !\n");
			chip->real_charger_type = POWER_SUPPLY_TYPE_USB;
			break;
		case WT_CHG_TYPE_CDP:
			dev_err(chip->dev, "CDP have been detected !\n");
			chip->real_charger_type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case WT_CHG_TYPE_DCP:
		case WT_CHG_TYPE_QC2:
		case WT_CHG_TYPE_QC3:
		case WT_CHG_TYPE_QC3P_18W:
		case WT_CHG_TYPE_QC3P_27W:
		case Z350_CHG_TYPE_HVDCP:
			dev_err(chip->dev, "DCP have been detected !\n");
			chip->real_charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			mmi_start_hvdcp_detect(chip);
			break;
		default:
			pr_err("bc12: no charge type detected\n");
			break;
	}
#endif
	schedule_delayed_work(&chip->monitor_work, 0);

	chip->chg_dev->noti.apsd_done = true;
	charger_dev_notify(chip->chg_dev);
}

static void rt9471_plug_out_func(struct rt9471_chip *chip)
{
	cancel_delayed_work_sync(&chip->monitor_work);
#ifdef CONFIG_MMI_QC3P_TURBO_CHARGER
	rt9471_enable_te(chip->chg_dev, true);
	qc3p_update_policy(chip);
#endif
	chip->chg_dev->noti.apsd_done = false;
	chip->chg_dev->noti.hvdcp_done = false;
	chip->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	rt9471_request_dpdm(chip, false);
	charger_dev_notify(chip->chg_dev);
}

static void charger_detect_work_func(struct work_struct *work)
{
	struct rt9471_chip *chip = container_of(work, struct rt9471_chip, charge_detect_work);
	bool vbus_gd = false;
	bool chg_ready = false;

	vbus_gd = rt9471_is_vbus_gd(chip);
	chg_ready =  rt9471_is_vbus_ready_for_chg(chip);
	dev_info(chip->dev, "%s: vbus_gd:%d, chg_ready:%d\n", __func__, vbus_gd, chg_ready);
	chip->state.online = chg_ready;
	chip->state.vbus_gd = vbus_gd;

	if ((!vbus_gd) && (chip->status & RT9471_STATUS_PLUGIN)) {
		dev_info(chip->dev, "%s:adapter removed\n", __func__);
		chip->status &= ~RT9471_STATUS_PLUGIN;
		rt9471_plug_out_func(chip);
	} else if (chg_ready && !(chip->status & RT9471_STATUS_PLUGIN)) {
		dev_info(chip->dev, "%s:adapter plugged in\n", __func__);
		chip->status |= RT9471_STATUS_PLUGIN;
		rt9471_plug_in_func(chip);
	}
}

static int rt9471_get_real_charger_type(struct charger_device *chg_dev, int *chg_type)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	*chg_type = chip->real_charger_type;

	return 0;
}

static int rt9471_is_charging_halted(struct charger_device *chg_dev, bool *en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_SLEEP;
	int ret = 0;

	ret = __rt9471_get_ic_stat(chip, &ic_stat);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "%s ic_stat = %d\n", __func__, ic_stat);

	switch(ic_stat) {
		case RT9471_ICSTAT_IEOC:
		case RT9471_ICSTAT_BGCHG:
		case RT9471_ICSTAT_CHGDONE:
			*en = true;
			break;

		case RT9471_ICSTAT_TRICKLECHG:
		case RT9471_ICSTAT_PRECHG:
		case RT9471_ICSTAT_FASTCHG:
			*en = false;
			break;

		default:
			break;
	}

	return ret;
}

static struct charger_ops rt9471_chg_ops = {
	//.get_pulse_cnt = mmi_get_pulse_cnt,
	//.set_dp_dm = mmi_set_dp_dm,
	.get_real_charger_type = rt9471_get_real_charger_type,
	.set_input_current = rt9471_set_aicr,
	.get_input_current = rt9471_get_aicr,
	.enable_hw_jeita = rt9471_enable_hw_jeita,
	.enable_otg = rt9471_enable_otg,
	.set_boost_current_limit = rt9471_set_boost_current_limit,
	.enable_charging = rt9471_enable_charging,
	.set_charging_current = rt9471_set_ichg,
	.set_constant_voltage = rt9471_set_cv,
	.is_charge_halted = rt9471_is_charging_halted,

	.dump_registers = rt9471_dump_registers,
	.is_enabled_charging = rt9471_is_charging_enabled,
	.enable_termination = rt9471_enable_te,
};

static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0, tmp = 0;
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &tmp);
	if (ret < 0) {
		dev_notice(dev, "%s parsing number fail(%d)\n", __func__, ret);
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	chip->enter_shipping_mode = true;
	/*
	 * Use kernel_halt() instead of kernel_power_off() to prevent
	 * the system from booting again while cable still plugged-in.
	 */
	kernel_halt();

	return count;
}

static const DEVICE_ATTR_WO(shipping_mode);

static int rt9471_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct rt9471_chip *chip = NULL;

	dev_info(&client->dev, "%s (%s)\n", __func__, RT9471_DRV_VERSION);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	mutex_init(&chip->hidden_mode_lock);
	mutex_init(&chip->hz_lock);
	chip->hidden_mode_cnt = 0;
	chip->chg_done_once = false;
	chip->buck_dwork_ws =
		wakeup_source_register(chip->dev,
				       devm_kasprintf(chip->dev, GFP_KERNEL,
				       "rt9471_buck_dwork_ws.%s",
				       dev_name(chip->dev)));
	INIT_DELAYED_WORK(&chip->buck_dwork, rt9471_buck_dwork_handler);
	chip->enter_shipping_mode = false;
	init_completion(&chip->aicc_done);
	init_completion(&chip->pe_done);
	chip->is_primary = false;

	if (!rt9471_check_devinfo(chip)) {
		ret = -ENODEV;
		goto err_nodev;
	}

	ret = rt9471_parse_dt(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s parse dt fail(%d)\n", __func__, ret);

#ifdef CONFIG_RT_REGMAP
	ret = rt9471_register_rt_regmap(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register rt regmap fail(%d)\n",
				      __func__, ret);
		goto err_register_rm;
	}
#endif /* CONFIG_RT_REGMAP */

	ret = rt9471_reset_register(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s reset register fail(%d)\n",
				      __func__, ret);

	ret = rt9471_init_setting(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s init fail(%d)\n", __func__, ret);
		goto err_init;
	}

	i2c_set_clientdata(client, chip);

	chip->chg_dev = charger_device_register(chip->mmi_chg_dev_name,
			chip->dev, chip, &rt9471_chg_ops, &chip->chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		dev_notice(chip->dev, "%s register chg dev fail(%d)\n",
				      __func__, ret);
		goto err_register_chg_dev;
	}

	ret = rt9471_register_irq(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register irq fail(%d)\n",
				      __func__, ret);
		goto err_register_irq;
	}

	INIT_DELAYED_WORK(&chip->monitor_work, rt9471_monitor_workfunc);
	INIT_WORK(&chip->charge_detect_work, charger_detect_work_func);

#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	if (chip->mmi_qc3_support) {
		chip->mmi_qc3_authen_task = kthread_create(mmi_hvdcp_detect_kthread, chip, "mmi_qc3_authen");
		if (IS_ERR(chip->mmi_qc3_authen_task)) {
			ret = PTR_ERR(chip->mmi_qc3_authen_task);
			dev_err(chip->dev, "Failed to create mmi_qc3_authen_task ret = %d\n", ret);
			goto err_check_chg;
		}
		init_waitqueue_head(&chip->mmi_qc3_wait_que);
		wake_up_process(chip->mmi_qc3_authen_task);
	}
#endif

	ret = rt9471_power_supply_init(chip, chip->dev);
	if (ret) {
		dev_notice(chip->dev, "Failed to register power supply\n");
		goto err_check_chg;
	}

	ret = rt9471_parse_dt_adc_channels(chip);
	if (ret) {
		dev_err(chip->dev, "Failed to get adc channels%d\n", ret);
		goto err_check_chg;
	}

#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	ret = mmi_init_iio_psy(chip, chip->dev);
	if (ret) {
		dev_notice(chip->dev, "mmi iio psy init failed\n");
	}
#endif

	ret = rt9471_check_chg(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s check chg(%d)\n", __func__, ret);
		goto err_check_chg;
	}

	ret = rt9471_init_irq(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s init irq fail(%d)\n", __func__, ret);
		goto err_init_irq;
	}

	ret = device_create_file(chip->dev, &dev_attr_shipping_mode);
	if (ret < 0) {
		dev_notice(chip->dev, "%s create file fail(%d)\n",
				      __func__, ret);
		goto err_create_file;
	}

	schedule_work(&chip->charge_detect_work);

	__rt9471_dump_registers(chip);
	dev_info(chip->dev, "%s successfully\n", __func__);
	return 0;

err_create_file:
	disable_irq(chip->irq);
err_init_irq:
err_check_chg:
err_register_irq:
#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	if (chip->mmi_qc3_support && chip->mmi_qc3_authen_task)
		kthread_stop(chip->mmi_qc3_authen_task);
#endif
	charger_device_unregister(chip->chg_dev);
err_register_chg_dev:
	cancel_delayed_work_sync(&chip->buck_dwork);
err_init:
	rt9471_reset_register(chip);
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->rm_dev);
err_register_rm:
#endif /* CONFIG_RT_REGMAP */
err_nodev:
	mutex_destroy(&chip->io_lock);
	mutex_destroy(&chip->hidden_mode_lock);
	mutex_destroy(&chip->hz_lock);
	wakeup_source_unregister(chip->buck_dwork_ws);

	return ret;
}

static int rt9471_remove(struct i2c_client *client)
{
	struct rt9471_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);
#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	if (chip->mmi_qc3_support && chip->mmi_qc3_authen_task)
		kthread_stop(chip->mmi_qc3_authen_task);
#endif
	cancel_delayed_work_sync(&chip->monitor_work);
	device_remove_file(chip->dev, &dev_attr_shipping_mode);
	disable_irq(chip->irq);
	charger_device_unregister(chip->chg_dev);
	cancel_delayed_work_sync(&chip->buck_dwork);
	rt9471_reset_register(chip);
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->rm_dev);
#endif /* CONFIG_RT_REGMAP */
	mutex_destroy(&chip->io_lock);
	mutex_destroy(&chip->hidden_mode_lock);
	mutex_destroy(&chip->hz_lock);
	wakeup_source_unregister(chip->buck_dwork_ws);

	return 0;
}

static void rt9471_shutdown(struct i2c_client *client)
{
	struct rt9471_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);

	disable_irq(chip->irq);
	charger_device_unregister(chip->chg_dev);
	cancel_delayed_work_sync(&chip->buck_dwork);
	rt9471_reset_register(chip);
	if (chip->enter_shipping_mode)
		__rt9471_enable_shipmode(chip);
}

static int rt9471_suspend(struct device *dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	disable_irq(chip->irq);

	return 0;
}

static int rt9471_resume(struct device *dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	enable_irq(chip->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rt9471_pm_ops, rt9471_suspend, rt9471_resume);

static const struct of_device_id rt9471_of_device_id[] = {
	{ .compatible = "richtek,rt9470", },
	{ .compatible = "richtek,rt9471", },
	{ .compatible = "richtek,swchg", },
	{ },
};
MODULE_DEVICE_TABLE(of, rt9471_of_device_id);

static const struct i2c_device_id rt9471_i2c_device_id[] = {
	{ "rt9470", 0 },
	{ "rt9471", 1 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rt9471_i2c_device_id);

static struct i2c_driver rt9471_i2c_driver = {
	.driver = {
		.name = "rt9471",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt9471_of_device_id),
		.pm = &rt9471_pm_ops,
	},
	.probe = rt9471_probe,
	.remove = rt9471_remove,
	.shutdown = rt9471_shutdown,
	.id_table = rt9471_i2c_device_id,
};
module_i2c_driver(rt9471_i2c_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("RT9471 Charger Driver");
MODULE_VERSION(RT9471_DRV_VERSION);
