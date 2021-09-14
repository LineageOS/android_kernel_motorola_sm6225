// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Richtek RT9467 Charger Driver
 *
 * Copyright (C) 2021 Richtek Technology Corp.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#ifdef CONFIG_RT_REGMAP
#include <rt-regmap.h>
#endif
#include "rt9467_charger.h"

#define I2C_ACCESS_MAX_RETRY	5
#define RT9467_DRV_VERSION	"1.0.0_G"

/* ======================= */
/* RT9467 Parameter        */
/* ======================= */

static const u32 rt9467_boost_oc_threshold[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000,
}; /* uA */

static const u32 rt9467_safety_timer[] = {
	4, 6, 8, 10, 12, 14, 16, 20,
}; /* hour */

enum rt9467_irq_idx {
	RT9467_IRQIDX_CHG_STATC = 0,
	RT9467_IRQIDX_CHG_FAULT,
	RT9467_IRQIDX_TS_STATC,
	RT9467_IRQIDX_CHG_IRQ1,
	RT9467_IRQIDX_CHG_IRQ2,
	RT9467_IRQIDX_CHG_IRQ3,
	RT9467_IRQIDX_DPDM_IRQ,
	RT9467_IRQIDX_MAX,
};

enum rt9467_irq_stat {
	RT9467_IRQSTAT_CHG_STATC = 0,
	RT9467_IRQSTAT_CHG_FAULT,
	RT9467_IRQSTAT_TS_STATC,
	RT9467_IRQSTAT_MAX,
};

enum rt9467_chg_type {
	RT9467_CHG_TYPE_NOVBUS = 0,
	RT9467_CHG_TYPE_UNDER_GOING,
	RT9467_CHG_TYPE_SDP,
	RT9467_CHG_TYPE_SDPNSTD,
	RT9467_CHG_TYPE_DCP,
	RT9467_CHG_TYPE_CDP,
	RT9467_CHG_TYPE_MAX,
};

enum rt9467_usbsw_state {
	RT9467_USBSW_CHG = 0,
	RT9467_USBSW_USB,
};

static const u8 rt9467_irq_maskall[RT9467_IRQIDX_MAX] = {
	0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

struct irq_mapping_tbl {
	const char *name;
	const int id;
};

#define RT9467_IRQ_MAPPING(_name, _id) {.name = #_name, .id = _id}
static const struct irq_mapping_tbl rt9467_irq_mapping_tbl[] = {
	RT9467_IRQ_MAPPING(chg_treg, 4),
	RT9467_IRQ_MAPPING(chg_aicr, 5),
	RT9467_IRQ_MAPPING(chg_mivr, 6),
	RT9467_IRQ_MAPPING(pwr_rdy, 7),
	RT9467_IRQ_MAPPING(chg_vsysuv, 12),
	RT9467_IRQ_MAPPING(chg_vsysov, 13),
	RT9467_IRQ_MAPPING(chg_vbatov, 14),
	RT9467_IRQ_MAPPING(chg_vbusov, 15),
	RT9467_IRQ_MAPPING(ts_batcold, 20),
	RT9467_IRQ_MAPPING(ts_batcool, 21),
	RT9467_IRQ_MAPPING(ts_batwarm, 22),
	RT9467_IRQ_MAPPING(ts_bathot, 23),
	RT9467_IRQ_MAPPING(ts_statci, 24),
	RT9467_IRQ_MAPPING(chg_faulti, 25),
	RT9467_IRQ_MAPPING(chg_statci, 26),
	RT9467_IRQ_MAPPING(chg_tmri, 27),
	RT9467_IRQ_MAPPING(chg_batabsi, 28),
	RT9467_IRQ_MAPPING(chg_adpbadi, 29),
	RT9467_IRQ_MAPPING(chg_rvpi, 30),
	RT9467_IRQ_MAPPING(otpi, 31),
	RT9467_IRQ_MAPPING(chg_aiclmeasi, 32),
	RT9467_IRQ_MAPPING(chg_ichgmeasi, 33),
	RT9467_IRQ_MAPPING(chgdet_donei, 34),
	RT9467_IRQ_MAPPING(wdtmri, 35),
	RT9467_IRQ_MAPPING(ssfinishi, 36),
	RT9467_IRQ_MAPPING(chg_rechgi, 37),
	RT9467_IRQ_MAPPING(chg_termi, 38),
	RT9467_IRQ_MAPPING(chg_ieoci, 39),
	RT9467_IRQ_MAPPING(adc_donei, 40),
	RT9467_IRQ_MAPPING(pumpx_donei, 41),
	RT9467_IRQ_MAPPING(bst_batuvi, 45),
	RT9467_IRQ_MAPPING(bst_midovi, 46),
	RT9467_IRQ_MAPPING(bst_olpi, 47),
	RT9467_IRQ_MAPPING(attachi, 48),
	RT9467_IRQ_MAPPING(detachi, 49),
	RT9467_IRQ_MAPPING(chgdeti, 54),
	RT9467_IRQ_MAPPING(dcdti, 55),
};

enum rt9467_charging_status {
	RT9467_CHG_STATUS_READY = 0,
	RT9467_CHG_STATUS_PROGRESS,
	RT9467_CHG_STATUS_DONE,
	RT9467_CHG_STATUS_FAULT,
	RT9467_CHG_STATUS_MAX,
};

static const char *rt9467_chg_status_name[RT9467_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};

static const u8 rt9467_val_en_hidden_mode[] = {
	0x49, 0x32, 0xB6, 0x27, 0x48, 0x18, 0x03, 0xE2,
};

enum rt9467_iin_limit_sel {
	RT9467_IINLMTSEL_3_2A = 0,
	RT9467_IINLMTSEL_CHG_TYP,
	RT9467_IINLMTSEL_AICR,
	RT9467_IINLMTSEL_LOWER_LEVEL, /* lower of above three */
};

enum rt9467_adc_sel {
	RT9467_ADC_VBUS_DIV5 = 1,
	RT9467_ADC_VBUS_DIV2,
	RT9467_ADC_VSYS,
	RT9467_ADC_VBAT,
	RT9467_ADC_TS_BAT = 6,
	RT9467_ADC_IBUS = 8,
	RT9467_ADC_IBAT,
	RT9467_ADC_REGN = 11,
	RT9467_ADC_TEMP_JC,
	RT9467_ADC_MAX,
};

/*
 * Unit for each ADC parameter
 * 0 stands for reserved
 * For TS_BAT, the real unit is 0.25.
 * Here we use 25, please remember to divide 100 while showing the value
 */
static const int rt9467_adc_unit[RT9467_ADC_MAX] = {
	0,
	RT9467_ADC_UNIT_VBUS_DIV5,
	RT9467_ADC_UNIT_VBUS_DIV2,
	RT9467_ADC_UNIT_VSYS,
	RT9467_ADC_UNIT_VBAT,
	0,
	RT9467_ADC_UNIT_TS_BAT,
	0,
	RT9467_ADC_UNIT_IBUS,
	RT9467_ADC_UNIT_IBAT,
	0,
	RT9467_ADC_UNIT_REGN,
	RT9467_ADC_UNIT_TEMP_JC,
};

static const int rt9467_adc_offset[RT9467_ADC_MAX] = {
	0,
	RT9467_ADC_OFFSET_VBUS_DIV5,
	RT9467_ADC_OFFSET_VBUS_DIV2,
	RT9467_ADC_OFFSET_VSYS,
	RT9467_ADC_OFFSET_VBAT,
	0,
	RT9467_ADC_OFFSET_TS_BAT,
	0,
	RT9467_ADC_OFFSET_IBUS,
	RT9467_ADC_OFFSET_IBAT,
	0,
	RT9467_ADC_OFFSET_REGN,
	RT9467_ADC_OFFSET_TEMP_JC,
};

struct rt9467_desc {
	u32 ichg;	/* uA */
	u32 aicr;	/* uA */
	u32 mivr;	/* uV */
	u32 cv;		/* uV */
	u32 ieoc;	/* uA */
	u32 safety_timer;	/* hour */
	u32 ircmp_resistor;	/* uohm */
	u32 ircmp_vclamp;	/* uV */
	bool en_te;
	bool en_wdt;
	bool en_irq_pulse;
	bool en_jeita;
	bool en_chgdet;
	int regmap_represent_slave_addr;
	const char *regmap_name;
	const char *chg_dev_name;
};

/* These default values will be applied if there's no property in dts */
static struct rt9467_desc rt9467_default_desc = {
	.ichg = 2000000,	/* uA */
	.aicr = 500000,		/* uA */
	.mivr = 4400000,	/* uV */
	.cv = 4350000,		/* uA */
	.ieoc = 250000,		/* uA */
	.safety_timer = 12,
	.ircmp_resistor = 25000,	/* uohm */
	.ircmp_vclamp = 32000,		/* uV */
	.en_te = true,
	.en_wdt = true,
	.en_irq_pulse = false,
	.en_jeita = false,
	.en_chgdet = true,
	.regmap_represent_slave_addr = RT9467_SLAVE_ADDR,
	.regmap_name = "rt9467",
	.chg_dev_name = "primary_chg",
};

struct rt9467_info {
	struct i2c_client *client;
	struct mutex i2c_access_lock;
	struct mutex adc_access_lock;
	struct mutex irq_access_lock;
	struct mutex aicr_access_lock;
	struct mutex ichg_access_lock;
	struct mutex hidden_mode_lock;
	struct mutex bc12_access_lock;
	struct mutex ieoc_lock;
	struct mutex tchg_lock;
	struct mutex pp_lock;
	struct device *dev;
	struct rt9467_info *info;
	struct rt9467_desc *desc;
	struct power_supply *chg_psy;
	struct power_supply_desc chg_psy_desc;
	wait_queue_head_t wait_queue;
	int irq;
	u32 power_limit;
	u8 chip_rev;
	u8 irq_flag[RT9467_IRQIDX_MAX];
	u8 irq_stat[RT9467_IRQSTAT_MAX];
	u8 irq_mask[RT9467_IRQIDX_MAX];
	u32 hidden_mode_cnt;
	bool pp_en;
	bool bc12_en;
	bool pwr_rdy;
	u32 mivr;
	u32 ieoc;
	u32 ichg;
	bool ieoc_wkard;
	struct work_struct init_work;
	struct delayed_work kick_wdt_dwork;
	atomic_t bc12_sdp_cnt;
	atomic_t bc12_wkard;
	int tchg;

#if IS_ENABLED(CONFIG_TCPC_CLASS)
	atomic_t tcpc_usb_connected;
#else
	struct work_struct chgdet_work;
#endif /* CONFIG_TCPC_CLASS */

#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *regmap_dev;
	struct rt_regmap_properties *regmap_prop;
#endif /* CONFIG_RT_REGMAP */
};

/* ======================= */
/* Register Address        */
/* ======================= */

static const unsigned char rt9467_reg_addr[] = {
	RT9467_REG_CORE_CTRL0,
	RT9467_REG_CHG_CTRL1,
	RT9467_REG_CHG_CTRL2,
	RT9467_REG_CHG_CTRL3,
	RT9467_REG_CHG_CTRL4,
	RT9467_REG_CHG_CTRL5,
	RT9467_REG_CHG_CTRL6,
	RT9467_REG_CHG_CTRL7,
	RT9467_REG_CHG_CTRL8,
	RT9467_REG_CHG_CTRL9,
	RT9467_REG_CHG_CTRL10,
	RT9467_REG_CHG_CTRL11,
	RT9467_REG_CHG_CTRL12,
	RT9467_REG_CHG_CTRL13,
	RT9467_REG_CHG_CTRL14,
	RT9467_REG_CHG_CTRL15,
	RT9467_REG_CHG_CTRL16,
	RT9467_REG_CHG_ADC,
	RT9467_REG_CHG_DPDM1,
	RT9467_REG_CHG_DPDM2,
	RT9467_REG_CHG_DPDM3,
	RT9467_REG_CHG_CTRL19,
	RT9467_REG_CHG_CTRL17,
	RT9467_REG_CHG_CTRL18,
	RT9467_REG_DEVICE_ID,
	RT9467_REG_CHG_STAT,
	RT9467_REG_CHG_NTC,
	RT9467_REG_ADC_DATA_H,
	RT9467_REG_ADC_DATA_L,
	RT9467_REG_ADC_DATA_TUNE_H,
	RT9467_REG_ADC_DATA_TUNE_L,
	RT9467_REG_ADC_DATA_ORG_H,
	RT9467_REG_ADC_DATA_ORG_L,
	RT9467_REG_CHG_STATC,
	RT9467_REG_CHG_FAULT,
	RT9467_REG_TS_STATC,
	/* Skip IRQ evt to prevent reading clear while dumping registers */
	RT9467_REG_CHG_STATC_CTRL,
	RT9467_REG_CHG_FAULT_CTRL,
	RT9467_REG_TS_STATC_CTRL,
	RT9467_REG_CHG_IRQ1_CTRL,
	RT9467_REG_CHG_IRQ2_CTRL,
	RT9467_REG_CHG_IRQ3_CTRL,
	RT9467_REG_DPDM_IRQ_CTRL,
};

/* ========= */
/* RT Regmap */
/* ========= */

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(RT9467_REG_CORE_CTRL0, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL4, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL5, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL8, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL9, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL10, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL11, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL12, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL13, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL14, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL15, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL16, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_ADC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_DPDM1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_DPDM2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_DPDM3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL19, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL17, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_CTRL18, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL4, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL8, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL9, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_HIDDEN_CTRL15, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_DEVICE_ID, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_STAT, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_NTC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_ADC_DATA_H, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_ADC_DATA_L, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_ADC_DATA_TUNE_H, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_ADC_DATA_TUNE_L, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_ADC_DATA_ORG_H, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_ADC_DATA_ORG_L, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_STATC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_FAULT, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_TS_STATC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_IRQ1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_IRQ2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_IRQ3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_DPDM_IRQ, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_STATC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_FAULT_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_TS_STATC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_IRQ1_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_IRQ2_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_CHG_IRQ3_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9467_REG_DPDM_IRQ_CTRL, 1, RT_VOLATILE, {});

static const rt_register_map_t rt9467_regmap_map[] = {
	RT_REG(RT9467_REG_CORE_CTRL0),
	RT_REG(RT9467_REG_CHG_CTRL1),
	RT_REG(RT9467_REG_CHG_CTRL2),
	RT_REG(RT9467_REG_CHG_CTRL3),
	RT_REG(RT9467_REG_CHG_CTRL4),
	RT_REG(RT9467_REG_CHG_CTRL5),
	RT_REG(RT9467_REG_CHG_CTRL6),
	RT_REG(RT9467_REG_CHG_CTRL7),
	RT_REG(RT9467_REG_CHG_CTRL8),
	RT_REG(RT9467_REG_CHG_CTRL9),
	RT_REG(RT9467_REG_CHG_CTRL10),
	RT_REG(RT9467_REG_CHG_CTRL11),
	RT_REG(RT9467_REG_CHG_CTRL12),
	RT_REG(RT9467_REG_CHG_CTRL13),
	RT_REG(RT9467_REG_CHG_CTRL14),
	RT_REG(RT9467_REG_CHG_CTRL15),
	RT_REG(RT9467_REG_CHG_CTRL16),
	RT_REG(RT9467_REG_CHG_ADC),
	RT_REG(RT9467_REG_CHG_DPDM1),
	RT_REG(RT9467_REG_CHG_DPDM2),
	RT_REG(RT9467_REG_CHG_DPDM3),
	RT_REG(RT9467_REG_CHG_CTRL19),
	RT_REG(RT9467_REG_CHG_CTRL17),
	RT_REG(RT9467_REG_CHG_CTRL18),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL1),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL2),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL4),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL6),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL7),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL8),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL9),
	RT_REG(RT9467_REG_CHG_HIDDEN_CTRL15),
	RT_REG(RT9467_REG_DEVICE_ID),
	RT_REG(RT9467_REG_CHG_STAT),
	RT_REG(RT9467_REG_CHG_NTC),
	RT_REG(RT9467_REG_ADC_DATA_H),
	RT_REG(RT9467_REG_ADC_DATA_L),
	RT_REG(RT9467_REG_ADC_DATA_TUNE_H),
	RT_REG(RT9467_REG_ADC_DATA_TUNE_L),
	RT_REG(RT9467_REG_ADC_DATA_ORG_H),
	RT_REG(RT9467_REG_ADC_DATA_ORG_L),
	RT_REG(RT9467_REG_CHG_STATC),
	RT_REG(RT9467_REG_CHG_FAULT),
	RT_REG(RT9467_REG_TS_STATC),
	RT_REG(RT9467_REG_CHG_IRQ1),
	RT_REG(RT9467_REG_CHG_IRQ2),
	RT_REG(RT9467_REG_CHG_IRQ3),
	RT_REG(RT9467_REG_DPDM_IRQ),
	RT_REG(RT9467_REG_CHG_STATC_CTRL),
	RT_REG(RT9467_REG_CHG_FAULT_CTRL),
	RT_REG(RT9467_REG_TS_STATC_CTRL),
	RT_REG(RT9467_REG_CHG_IRQ1_CTRL),
	RT_REG(RT9467_REG_CHG_IRQ2_CTRL),
	RT_REG(RT9467_REG_CHG_IRQ3_CTRL),
	RT_REG(RT9467_REG_DPDM_IRQ_CTRL),
};
#endif /* CONFIG_RT_REGMAP */

/* ========================= */
/* I2C operations            */
/* ========================= */

static int rt9467_device_read(void *client, u32 addr, int leng, void *dst)
{
	struct i2c_client *i2c = (struct i2c_client *)client;

	return i2c_smbus_read_i2c_block_data(i2c, addr, leng, dst);
}

static int rt9467_device_write(void *client, u32 addr, int leng,
	const void *src)
{
	struct i2c_client *i2c = (struct i2c_client *)client;

	return i2c_smbus_write_i2c_block_data(i2c, addr, leng, src);
}

#ifdef CONFIG_RT_REGMAP
static struct rt_regmap_fops rt9467_regmap_fops = {
	.read_device = rt9467_device_read,
	.write_device = rt9467_device_write,
};

static int rt9467_register_rt_regmap(struct rt9467_info *info)
{
	int ret = 0;
	struct i2c_client *client = info->client;
	struct rt_regmap_properties *prop = NULL;

	dev_info(info->dev, "%s\n", __func__);

	prop = devm_kzalloc(&client->dev, sizeof(struct rt_regmap_properties),
		GFP_KERNEL);
	if (!prop)
		return -ENOMEM;

	prop->name = info->desc->regmap_name;
	prop->aliases = info->desc->regmap_name;
	prop->register_num = ARRAY_SIZE(rt9467_regmap_map);
	prop->rm = rt9467_regmap_map;
	prop->rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE |
		RT_IO_PASS_THROUGH;
	prop->io_log_en = 0;

	info->regmap_prop = prop;
	info->regmap_dev = rt_regmap_device_register_ex(info->regmap_prop,
		&rt9467_regmap_fops, &client->dev, client,
		info->desc->regmap_represent_slave_addr, info);
	if (!info->regmap_dev) {
		dev_notice(info->dev, "%s: register regmap dev fail\n",
			__func__);
		return -EIO;
	}

	return ret;
}
#endif /* CONFIG_RT_REGMAP */

static inline int __rt9467_i2c_write_byte(struct rt9467_info *info, u8 cmd,
	u8 data)
{
	int ret = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_write(info->regmap_dev, cmd, 1, &data);
#else
		ret = rt9467_device_write(info->client, cmd, 1, &data);
#endif
		retry++;
		if (ret < 0)
			udelay(10);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0)
		dev_notice(info->dev, "%s: I2CW[0x%02X] = 0x%02X fail\n",
			__func__, cmd, data);
	else
		dev_dbg(info->dev, "%s: I2CW[0x%02X] = 0x%02X\n", __func__,
			cmd, data);

	return ret;
}

static int rt9467_i2c_write_byte(struct rt9467_info *info, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9467_i2c_write_byte(info, cmd, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int __rt9467_i2c_read_byte(struct rt9467_info *info, u8 cmd)
{
	int ret = 0, ret_val = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_read(info->regmap_dev, cmd, 1, &ret_val);
#else
		ret = rt9467_device_read(info->client, cmd, 1, &ret_val);
#endif
		retry++;
		if (ret < 0)
			udelay(10);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0) {
		dev_notice(info->dev, "%s: I2CR[0x%02X] fail\n", __func__, cmd);
		return ret;
	}

	ret_val = ret_val & 0xFF;

	dev_dbg(info->dev, "%s: I2CR[0x%02X] = 0x%02X\n", __func__, cmd,
		ret_val);

	return ret_val;
}

static int rt9467_i2c_read_byte(struct rt9467_info *info, u8 cmd)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9467_i2c_read_byte(info, cmd);
	mutex_unlock(&info->i2c_access_lock);

	if (ret < 0)
		return ret;

	return (ret & 0xFF);
}

static inline int __rt9467_i2c_block_write(struct rt9467_info *info, u8 cmd,
	u32 leng, const u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(info->regmap_dev, cmd, leng, data);
#else
	ret = rt9467_device_write(info->client, cmd, leng, data);
#endif

	return ret;
}


static int rt9467_i2c_block_write(struct rt9467_info *info, u8 cmd, u32 leng,
	const u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9467_i2c_block_write(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int __rt9467_i2c_block_read(struct rt9467_info *info, u8 cmd,
	u32 leng, u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(info->regmap_dev, cmd, leng, data);
#else
	ret = rt9467_device_read(info->client, cmd, leng, data);
#endif

	return ret;
}


static int rt9467_i2c_block_read(struct rt9467_info *info, u8 cmd, u32 leng,
	u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9467_i2c_block_read(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}


static int rt9467_i2c_test_bit(struct rt9467_info *info, u8 cmd, u8 shift,
	bool *is_one)
{
	int ret = 0;
	u8 data = 0;

	ret = rt9467_i2c_read_byte(info, cmd);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data = ret & (1 << shift);
	*is_one = (data == 0 ? false : true);

	return ret;
}

static int rt9467_i2c_update_bits(struct rt9467_info *info, u8 cmd, u8 data,
	u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9467_i2c_read_byte(info, cmd);
	if (ret < 0) {
		mutex_unlock(&info->i2c_access_lock);
		return ret;
	}

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);

	ret = __rt9467_i2c_write_byte(info, cmd, reg_data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int rt9467_set_bit(struct rt9467_info *info, u8 reg, u8 mask)
{
	return rt9467_i2c_update_bits(info, reg, mask, mask);
}

static inline int rt9467_clr_bit(struct rt9467_info *info, u8 reg, u8 mask)
{
	return rt9467_i2c_update_bits(info, reg, 0x00, mask);
}

/* ================== */
/* Internal Functions */
/* ================== */
static int rt9467_get_mivr(struct rt9467_info *info, u32 *mivr);
static int rt9467_get_aicr(struct rt9467_info *info, u32 *aicr);
static int rt9467_get_ichg(struct rt9467_info *info, u32 *ichg);
static int rt9467_kick_wdt(struct rt9467_info *info);
static int rt9467_get_ieoc(struct rt9467_info *info, u32 *ieoc);
static int rt9467_enable_hidden_mode(struct rt9467_info *info, bool en);

static inline void rt9467_irq_set_flag(struct rt9467_info *info, u8 *irq,
	u8 mask)
{
	mutex_lock(&info->irq_access_lock);
	*irq |= mask;
	mutex_unlock(&info->irq_access_lock);
}

static inline void rt9467_irq_clr_flag(struct rt9467_info *info, u8 *irq,
	u8 mask)
{
	mutex_lock(&info->irq_access_lock);
	*irq &= ~mask;
	mutex_unlock(&info->irq_access_lock);
}

static inline const char *rt9467_get_irq_name(struct rt9467_info *info,
	int irqnum)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(rt9467_irq_mapping_tbl); i++) {
		if (rt9467_irq_mapping_tbl[i].id == irqnum)
			return rt9467_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void rt9467_irq_mask(struct rt9467_info *info, int irqnum)
{
	dev_dbg(info->dev, "%s: irq = %d, %s\n", __func__, irqnum,
		rt9467_get_irq_name(info, irqnum));
	info->irq_mask[irqnum / 8] |= (1 << (irqnum % 8));
}

static inline void rt9467_irq_unmask(struct rt9467_info *info, int irqnum)
{
	dev_dbg(info->dev, "%s: irq = %d, %s\n", __func__, irqnum,
		rt9467_get_irq_name(info, irqnum));
	info->irq_mask[irqnum / 8] &= ~(1 << (irqnum % 8));
}

static inline u8 rt9467_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	/* Greater than maximum supported value, use maximum one */
	if (target >= max)
		return (max - min) / step;

	return (target - min) / step;
}

static inline u8 rt9467_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
	u32 target)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return tbl_size - 1;
}

static inline u32 rt9467_closest_value(u32 min, u32 max, u32 step, u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static int rt9467_get_adc(struct rt9467_info *info,
	enum rt9467_adc_sel adc_sel, int *adc_val)
{
	int ret = 0, i = 0;
	const int max_wait_times = 6;
	u8 adc_data[6] = {0};
	u32 aicr = 0, ichg = 0;
	bool adc_start = false;

	mutex_lock(&info->adc_access_lock);

	rt9467_enable_hidden_mode(info, true);

	/* Select ADC to desired channel */
	ret = rt9467_i2c_update_bits(info, RT9467_REG_CHG_ADC,
		adc_sel << RT9467_SHIFT_ADC_IN_SEL, RT9467_MASK_ADC_IN_SEL);
	if (ret < 0) {
		dev_notice(info->dev, "%s: select ch to %d fail(%d)\n",
			__func__, adc_sel, ret);
		goto out;
	}

	/* Workaround for IBUS & IBAT */
	if (adc_sel == RT9467_ADC_IBUS) {
		mutex_lock(&info->aicr_access_lock);
		ret = rt9467_get_aicr(info, &aicr);
		if (ret < 0) {
			dev_notice(info->dev, "%s: get aicr fail\n", __func__);
			goto out_unlock_all;
		}
	} else if (adc_sel == RT9467_ADC_IBAT) {
		mutex_lock(&info->ichg_access_lock);
		ret = rt9467_get_ichg(info, &ichg);
		if (ret < 0) {
			dev_notice(info->dev, "%s: get ichg fail\n", __func__);
			goto out_unlock_all;
		}
	}

	/* Start ADC conversation */
	ret = rt9467_set_bit(info, RT9467_REG_CHG_ADC, RT9467_MASK_ADC_START);
	if (ret < 0) {
		dev_notice(info->dev, "%s: start con fail(%d), sel = %d\n",
			__func__, ret, adc_sel);
		goto out_unlock_all;
	}

	for (i = 0; i < max_wait_times; i++) {
		msleep(35);
		ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_ADC,
			RT9467_SHIFT_ADC_START, &adc_start);
		if (ret >= 0 && !adc_start)
			break;
	}
	if (i == max_wait_times) {
		dev_notice(info->dev, "%s: wait con fail(%d), sel = %d\n",
			__func__, ret, adc_sel);
		ret = -EINVAL;
		goto out_unlock_all;
	}

	mdelay(1);

	/* Read ADC data high/low byte */
	ret = rt9467_i2c_block_read(info, RT9467_REG_ADC_DATA_H, 6, adc_data);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read ADC data fail\n", __func__);
		goto out_unlock_all;
	}
	dev_dbg(info->dev,
		"%s: adc_tune = (0x%02X, 0x%02X), adc_org = (0x%02X, 0x%02X)\n",
		__func__, adc_data[2], adc_data[3], adc_data[4], adc_data[5]);

	/* Calculate ADC value */
	*adc_val = ((adc_data[0] << 8) + adc_data[1]) * rt9467_adc_unit[adc_sel]
		+ rt9467_adc_offset[adc_sel];

	dev_dbg(info->dev,
		"%s: adc_sel = %d, adc_h = 0x%02X, adc_l = 0x%02X, val = %d\n",
		__func__, adc_sel, adc_data[0], adc_data[1], *adc_val);

	ret = 0;

out_unlock_all:
	/* Coefficient of IBUS & IBAT */
	if (adc_sel == RT9467_ADC_IBUS) {
		if (aicr < 400000) /* 400mA */
			*adc_val = *adc_val * 67 / 100;
		mutex_unlock(&info->aicr_access_lock);
	} else if (adc_sel == RT9467_ADC_IBAT) {
		if (ichg >= 100000 && ichg <= 450000) /* 100~450mA */
			*adc_val = *adc_val * 57 / 100;
		else if (ichg >= 500000 && ichg <= 850000) /* 500~850mA */
			*adc_val = *adc_val * 63 / 100;
		mutex_unlock(&info->ichg_access_lock);
	}

out:
	rt9467_enable_hidden_mode(info, false);
	mutex_unlock(&info->adc_access_lock);
	return ret;
}

static inline int __rt9467_enable_chgdet_flow(struct rt9467_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_DPDM1, en ? RT9467_MASK_USBCHGEN :
		 RT9467_MASK_USBCHGEN | RT9467_MASK_HVDCP_EN);
	if (ret >= 0)
		info->bc12_en = en;

	return ret;
}

static int rt9467_enable_chgdet_flow(struct rt9467_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	mutex_lock(&info->bc12_access_lock);
	ret = __rt9467_enable_chgdet_flow(info, en);
	mutex_unlock(&info->bc12_access_lock);

	return ret;
}

static inline int rt9467_enable_ilim(struct rt9467_info *info, bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL3, RT9467_MASK_ILIM_EN);
}

static inline int rt9467_toggle_chgdet_flow(struct rt9467_info *info)
{
	int ret = 0;
	u8 data = 0, usbd_off[2] = {0}, usbd_on[2] = {0};
	struct i2c_client *client = info->client;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = usbd_off,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = usbd_on,
		},
	};

	/* read data */
	ret = i2c_smbus_read_i2c_block_data(client, RT9467_REG_CHG_DPDM1,
		1, &data);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read usbd fail\n", __func__);
		goto out;
	}

	/* usbd off and then on */
	usbd_off[0] = usbd_on[0] = RT9467_REG_CHG_DPDM1;
	usbd_off[1] = data & ~RT9467_MASK_USBCHGEN;
	usbd_on[1] = data | RT9467_MASK_USBCHGEN;
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_notice(info->dev, "%s: usbd off/on fail(%d)\n",
				      __func__, ret);
out:

	return ret < 0 ? ret : 0;
}

static int rt9467_bc12_sdp_workaround(struct rt9467_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	mutex_lock(&info->i2c_access_lock);

	ret = rt9467_toggle_chgdet_flow(info);
	if (ret < 0)
		goto err;

	mdelay(10);

	ret = rt9467_toggle_chgdet_flow(info);
	if (ret < 0)
		goto err;

	goto out;
err:
	dev_notice(info->dev, "%s: fail\n", __func__);
out:
	mutex_unlock(&info->i2c_access_lock);
	return ret;
}

static int __rt9467_chgdet_handler(struct rt9467_info *info)
{
	int ret = 0;
	bool pwr_rdy = false, inform_psy = true;
	bool hvdcp = false, hvdcp_en = false;
	u8 usb_status = 0;

	dev_info(info->dev, "%s\n", __func__);

	/* disabled by user, do nothing */
	if (!info->desc->en_chgdet) {
		dev_info(info->dev, "%s: bc12 is disabled by dts\n", __func__);
		return 0;
	}

#if IS_ENABLED(CONFIG_TCPC_CLASS)
	pwr_rdy = atomic_read(&info->tcpc_usb_connected);
#else
	/* check power ready */
	ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_STATC,
		RT9467_SHIFT_PWR_RDY, &pwr_rdy);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read pwr rdy state fail\n",
			__func__);
		return ret;
	}
#endif

	/* no change in pwr_rdy state */
	if (info->pwr_rdy == pwr_rdy &&
		atomic_read(&info->bc12_wkard) == 0) {
		dev_info(info->dev, "%s: pwr_rdy(%d) state is the same\n",
			__func__, pwr_rdy);
		inform_psy = false;
		goto out;
	}
	info->pwr_rdy = pwr_rdy;

	/* plug out */
	if (!pwr_rdy) {
		info->chg_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		atomic_set(&info->bc12_sdp_cnt, 0);
		goto out;
	}

	/* plug in */
	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_DPDM2);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read type fail\n", __func__);
		return ret;
	}
	usb_status = (ret & RT9467_MASK_USB_STATUS) >> RT9467_SHIFT_USB_STATUS;

	switch (usb_status) {
	case RT9467_CHG_TYPE_NOVBUS:
		dev_info(info->dev, "%s: No VBUS\n", __func__);
		return 0;
	case RT9467_CHG_TYPE_UNDER_GOING:
		dev_info(info->dev, "%s: under going...\n", __func__);
		return 0;
	case RT9467_CHG_TYPE_SDP:
		info->chg_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case RT9467_CHG_TYPE_SDPNSTD:
		info->chg_psy_desc.type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	case RT9467_CHG_TYPE_DCP:
		ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_DPDM1);
		if (ret < 0)
			dev_notice(info->dev, "%s: check HVDCP fail(%d)\n",
					      __func__, ret);
		hvdcp_en = !!(ret & RT9467_MASK_HVDCP_EN);
		hvdcp = !!(ret & RT9467_MASK_HVDCP);
		info->chg_psy_desc.type = hvdcp ? POWER_SUPPLY_TYPE_USB_HVDCP :
						  POWER_SUPPLY_TYPE_USB_DCP;
		if (!hvdcp && !hvdcp_en) {
			ret = rt9467_set_bit(info, RT9467_REG_CHG_DPDM1,
					     RT9467_MASK_HVDCP_EN);
			if (ret < 0) {
				dev_notice(info->dev,
					   "%s: HVDCP EN fail(%d)\n",
					   __func__, ret);
				break;
			}
			ret = rt9467_toggle_chgdet_flow(info);
			if (ret < 0) {
				dev_notice(info->dev,
					   "%s: toggle chgdet flow fail(%d)\n",
					   __func__, ret);
				break;
			}
			atomic_set(&info->bc12_wkard, 1);
		}
		break;
	case RT9467_CHG_TYPE_CDP:
		info->chg_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	default:
		info->chg_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}

	/* BC12 workaround (NONSTD -> STP) */
	if (atomic_read(&info->bc12_sdp_cnt) < 2 &&
		info->chg_psy_desc.type == POWER_SUPPLY_TYPE_USB) {
		ret = rt9467_bc12_sdp_workaround(info);
		/* Workaround success, wait for next event */
		if (ret >= 0) {
			atomic_inc(&info->bc12_sdp_cnt);
			atomic_set(&info->bc12_wkard, 1);
			return 0;
		}
		goto out;
	}
out:
	atomic_set(&info->bc12_wkard, 0);

	if (info->chg_psy_desc.type != POWER_SUPPLY_TYPE_USB_DCP) {
		/* turn off USB charger detection */
		ret = __rt9467_enable_chgdet_flow(info, false);
		if (ret < 0)
			dev_notice(info->dev, "%s: disable chgdet fail\n",
					      __func__);
	}

	if (inform_psy)
		power_supply_changed(info->chg_psy);

	return ret;
}

static int rt9467_chgdet_handler(struct rt9467_info *info)
{
	int ret = 0;

	mutex_lock(&info->bc12_access_lock);
	ret = __rt9467_chgdet_handler(info);
	mutex_unlock(&info->bc12_access_lock);

	return ret;
}

static int rt9467_set_aicl_vth(struct rt9467_info *info, u32 aicl_vth)
{
	u8 reg_aicl_vth = 0;

	reg_aicl_vth = rt9467_closest_reg(RT9467_AICL_VTH_MIN,
		RT9467_AICL_VTH_MAX, RT9467_AICL_VTH_STEP, aicl_vth);

	dev_info(info->dev, "%s: vth = %d(0x%02X)\n", __func__, aicl_vth,
		reg_aicl_vth);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL14,
		reg_aicl_vth << RT9467_SHIFT_AICL_VTH, RT9467_MASK_AICL_VTH);
}

static int __rt9467_set_aicr(struct rt9467_info *info, u32 aicr)
{
	u8 reg_aicr = 0;

	reg_aicr = rt9467_closest_reg(RT9467_AICR_MIN, RT9467_AICR_MAX,
		RT9467_AICR_STEP, aicr);

	dev_info(info->dev, "%s: aicr = %d(0x%02X)\n", __func__, aicr,
		reg_aicr);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL3,
		reg_aicr << RT9467_SHIFT_AICR, RT9467_MASK_AICR);
}

static int __rt9467_run_aicl(struct rt9467_info *info)
{
	int ret = 0;
	u32 mivr = 0, aicl_vth = 0, aicr = 0;
	bool mivr_act = false;

	/* Check whether MIVR loop is active */
	ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_STATC,
		RT9467_SHIFT_CHG_MIVR, &mivr_act);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read mivr stat fail\n", __func__);
		goto out;
	}

	if (!mivr_act) {
		dev_info(info->dev, "%s: mivr loop is not active\n", __func__);
		ret = -EAGAIN;
		goto out;
	}

	ret = rt9467_get_mivr(info, &mivr);
	if (ret < 0)
		goto out;

	/* Check if there's a suitable AICL_VTH */
	aicl_vth = mivr + 200000;
	if (aicl_vth > RT9467_AICL_VTH_MAX) {
		dev_notice(info->dev, "%s: no suitable vth, vth = %d\n",
			__func__, aicl_vth);
		ret = -EINVAL;
		goto out;
	}

	ret = rt9467_set_aicl_vth(info, aicl_vth);
	if (ret < 0)
		goto out;

	/* Clear AICL measurement IRQ */
	rt9467_irq_clr_flag(info, &info->irq_flag[RT9467_IRQIDX_CHG_IRQ2],
		RT9467_MASK_CHG_AICLMEASI);

	mutex_lock(&info->aicr_access_lock);

	ret = rt9467_set_bit(info, RT9467_REG_CHG_CTRL14,
		RT9467_MASK_AICL_MEAS);
	if (ret < 0)
		goto unlock_out;

	ret = wait_event_interruptible_timeout(info->wait_queue,
		info->irq_flag[RT9467_IRQIDX_CHG_IRQ2] &
		RT9467_MASK_CHG_AICLMEASI,
		msecs_to_jiffies(3500));
	if (ret <= 0) {
		dev_notice(info->dev, "%s: wait AICL time out\n", __func__);
		ret = -EIO;
		goto unlock_out;
	}

	ret = rt9467_get_aicr(info, &aicr);
	if (ret < 0)
		goto unlock_out;

	dev_info(info->dev, "%s: OK, aicr upper bound = %dmA\n", __func__,
		 aicr / 1000);
	info->power_limit = aicl_vth / 1000000 * aicr;
	dev_info(info->dev, "%s: power limit = %dmW\n", __func__,
		 info->power_limit / 1000);

unlock_out:
	mutex_unlock(&info->aicr_access_lock);
out:
	return ret;
}

#if !IS_ENABLED(CONFIG_TCPC_CLASS)
static void rt9467_chgdet_work_handler(struct work_struct *work)
{
	int ret = 0;
	bool pwr_rdy = false;
	struct rt9467_info *info = (struct rt9467_info *)container_of(work,
		struct rt9467_info, chgdet_work);

	/* Check power ready */
	ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_STATC,
		RT9467_SHIFT_PWR_RDY, &pwr_rdy);

	if (!pwr_rdy)
		return;

	/* Enable USB charger type detection */
	ret = rt9467_enable_chgdet_flow(info, true);
	if (ret < 0)
		dev_notice(info->dev, "%s: enable chgdet fail\n", __func__);

}
#endif /* CONFIG_TCPC_CLASS */


/* Prevent back boost */
static int rt9467_toggle_cfo(struct rt9467_info *info)
{
	int ret = 0;
	u8 data = 0;

	dev_info(info->dev, "%s\n", __func__);
	mutex_lock(&info->i2c_access_lock);
	ret = rt9467_device_read(info->client, RT9467_REG_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		dev_notice(info->dev, "%s read cfo fail(%d)\n", __func__, ret);
		goto out;
	}

	/* CFO off */
	data &= ~RT9467_MASK_CFO_EN;
	ret = rt9467_device_write(info->client, RT9467_REG_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		dev_notice(info->dev, "%s cfo off fail(%d)\n", __func__, ret);
		goto out;
	}

	/* CFO on */
	data |= RT9467_MASK_CFO_EN;
	ret = rt9467_device_write(info->client, RT9467_REG_CHG_CTRL2, 1, &data);
	if (ret < 0)
		dev_notice(info->dev, "%s cfo on fail(%d)\n", __func__, ret);

out:
	mutex_unlock(&info->i2c_access_lock);
	return ret;
}

/* IRQ handlers */
static int rt9467_pwr_rdy_irq_handler(struct rt9467_info *info)
{
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	int ret = 0;
	bool pwr_rdy = false;
#endif /* CONFIG_TCPC_CLASS */

	dev_notice(info->dev, "%s\n", __func__);

#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_STATC,
		RT9467_SHIFT_PWR_RDY, &pwr_rdy);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read pwr rdy fail\n", __func__);
		goto out;
	}

	if (!pwr_rdy) {
		dev_info(info->dev, "%s: pwr rdy = 0\n", __func__);
		goto out;
	}

	ret = rt9467_enable_chgdet_flow(info, true);
	if (ret < 0)
		dev_notice(info->dev, "%s: en chgdet fail(%d)\n", __func__,
			ret);

out:
#endif /* CONFIG_TCPC_CLASS */

	return 0;
}

static int rt9467_chg_mivr_irq_handler(struct rt9467_info *info)
{
	int ret = 0;
	bool mivr_act = false;
	int adc_ibus = 0;

	dev_notice(info->dev, "%s\n", __func__);

	/* Check whether MIVR loop is active */
	ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_STATC,
		RT9467_SHIFT_CHG_MIVR, &mivr_act);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read mivr stat fail\n", __func__);
		goto out;
	}

	if (!mivr_act) {
		dev_info(info->dev, "%s: mivr loop is not active\n", __func__);
		goto out;
	}

	/* Check IBUS ADC */
	ret = rt9467_get_adc(info, RT9467_ADC_IBUS, &adc_ibus);
	if (ret < 0) {
		dev_notice(info->dev, "%s: get ibus fail\n", __func__);
		return ret;
	}
	if (adc_ibus < 100000) { /* 100mA */
		ret = rt9467_toggle_cfo(info);
		return ret;
	}
out:
	return 0;
}

static int rt9467_chg_aicr_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_treg_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_vsysuv_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_vsysov_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_vbatov_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_vbusov_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_ts_bat_cold_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_ts_bat_cool_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_ts_bat_warm_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_ts_bat_hot_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_ts_statci_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_faulti_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_statci_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_tmri_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_batabsi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_adpbadi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_rvpi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_otpi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_aiclmeasi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	rt9467_irq_set_flag(info, &info->irq_flag[RT9467_IRQIDX_CHG_IRQ2],
		RT9467_MASK_CHG_AICLMEASI);
	wake_up_interruptible(&info->wait_queue);
	return 0;
}

static int rt9467_chg_ichgmeasi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chgdet_donei_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_wdtmri_irq_handler(struct rt9467_info *info)
{
	int ret = 0;

	dev_notice(info->dev, "%s\n", __func__);
	ret = rt9467_kick_wdt(info);
	if (ret < 0)
		dev_notice(info->dev, "%s: kick wdt fail\n", __func__);

	return ret;
}

static int rt9467_ssfinishi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_rechgi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_termi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_chg_ieoci_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_adc_donei_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_pumpx_donei_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_bst_batuvi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_bst_midovi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_bst_olpi_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_attachi_irq_handler(struct rt9467_info *info)
{
	int ret = 0;

	dev_notice(info->dev, "%s\n", __func__);

	/* check bc12_en state */
	mutex_lock(&info->bc12_access_lock);
	if (!info->bc12_en) {
		dev_notice(info->dev, "%s: bc12 disabled, ignore irq\n",
			__func__);
		goto out;
	}
	ret = __rt9467_chgdet_handler(info);
out:
	mutex_unlock(&info->bc12_access_lock);
	return ret;
}

static int rt9467_detachi_irq_handler(struct rt9467_info *info)
{
	int ret = 0;

	dev_notice(info->dev, "%s\n", __func__);
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	ret = rt9467_chgdet_handler(info);
#endif /* CONFIG_TCPC_CLASS */
	return ret;
}

static int rt9467_chgdeti_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9467_dcdti_irq_handler(struct rt9467_info *info)
{
	dev_notice(info->dev, "%s\n", __func__);
	return 0;
}

typedef int (*rt9467_irq_fptr)(struct rt9467_info *);
static rt9467_irq_fptr rt9467_irq_handler_tbl[56] = {
	NULL,
	NULL,
	NULL,
	NULL,
	rt9467_chg_treg_irq_handler,
	rt9467_chg_aicr_irq_handler,
	rt9467_chg_mivr_irq_handler,
	rt9467_pwr_rdy_irq_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	rt9467_chg_vsysuv_irq_handler,
	rt9467_chg_vsysov_irq_handler,
	rt9467_chg_vbatov_irq_handler,
	rt9467_chg_vbusov_irq_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	rt9467_ts_bat_cold_irq_handler,
	rt9467_ts_bat_cool_irq_handler,
	rt9467_ts_bat_warm_irq_handler,
	rt9467_ts_bat_hot_irq_handler,
	rt9467_ts_statci_irq_handler,
	rt9467_chg_faulti_irq_handler,
	rt9467_chg_statci_irq_handler,
	rt9467_chg_tmri_irq_handler,
	rt9467_chg_batabsi_irq_handler,
	rt9467_chg_adpbadi_irq_handler,
	rt9467_chg_rvpi_irq_handler,
	rt9467_chg_otpi_irq_handler,
	rt9467_chg_aiclmeasi_irq_handler,
	rt9467_chg_ichgmeasi_irq_handler,
	rt9467_chgdet_donei_irq_handler,
	rt9467_wdtmri_irq_handler,
	rt9467_ssfinishi_irq_handler,
	rt9467_chg_rechgi_irq_handler,
	rt9467_chg_termi_irq_handler,
	rt9467_chg_ieoci_irq_handler,
	rt9467_adc_donei_irq_handler,
	rt9467_pumpx_donei_irq_handler,
	NULL,
	NULL,
	NULL,
	rt9467_bst_batuvi_irq_handler,
	rt9467_bst_midovi_irq_handler,
	rt9467_bst_olpi_irq_handler,
	rt9467_attachi_irq_handler,
	rt9467_detachi_irq_handler,
	NULL,
	NULL,
	NULL,
	rt9467_chgdeti_irq_handler,
	rt9467_dcdti_irq_handler,
};

static irqreturn_t rt9467_irq_handler(int irq, void *data)
{
	int ret = 0, i = 0, j = 0;
	struct rt9467_info *info = data;
	u8 evt[RT9467_IRQIDX_MAX] = {0};
	u8 mask[RT9467_IRQIDX_MAX] = {0};
	u8 stat[RT9467_IRQSTAT_MAX] = {0};
	u8 usb_status_old = 0, usb_status_new = 0;

	dev_info(info->dev, "%s\n", __func__);

	/* Read DPDM status before reading evts */
	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_DPDM2);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read type fail\n", __func__);
		goto err_read_irq;
	}
	usb_status_old = (ret & RT9467_MASK_USB_STATUS) >>
		RT9467_SHIFT_USB_STATUS;

	/* Read event and skip CHG_IRQ3 */
	ret = rt9467_i2c_block_read(info, RT9467_REG_CHG_IRQ1, 2, &evt[3]);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read evt1 fail(%d)\n", __func__,
			ret);
		goto err_read_irq;
	}

	ret = rt9467_i2c_block_read(info, RT9467_REG_DPDM_IRQ, 1, &evt[6]);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read evt2 fail(%d)\n", __func__,
			ret);
		goto err_read_irq;
	}

	ret = rt9467_i2c_block_read(info, RT9467_REG_CHG_STATC, 3, evt);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read stat fail(%d)\n", __func__,
			ret);
		goto err_read_irq;
	}

	/* Read DPDM status after reading evts */
	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_DPDM2);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read type fail\n", __func__);
		goto err_read_irq;
	}
	usb_status_new = (ret & RT9467_MASK_USB_STATUS) >>
		RT9467_SHIFT_USB_STATUS;

	/* Read mask */
	ret = rt9467_i2c_block_read(info, RT9467_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(mask), mask);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read mask fail(%d)\n", __func__,
			ret);
		goto err_read_irq;
	}

	/* Detach */
	if (usb_status_old != RT9467_CHG_TYPE_NOVBUS &&
		usb_status_new == RT9467_CHG_TYPE_NOVBUS)
		evt[RT9467_IRQIDX_DPDM_IRQ] |= 0x02;

	/* Attach */
	if (usb_status_new >= RT9467_CHG_TYPE_SDP &&
		usb_status_new <= RT9467_CHG_TYPE_CDP &&
		usb_status_old != usb_status_new)
		evt[RT9467_IRQIDX_DPDM_IRQ] |= 0x01;

	/* Store/Update stat */
	memcpy(stat, info->irq_stat, RT9467_IRQSTAT_MAX);

	for (i = 0; i < RT9467_IRQIDX_MAX; i++) {
		evt[i] &= ~mask[i];
		if (i < RT9467_IRQSTAT_MAX) {
			info->irq_stat[i] = evt[i];
			evt[i] ^= stat[i];
		}
		for (j = 0; j < 8; j++) {
			if (!(evt[i] & (1 << j)))
				continue;
			if (rt9467_irq_handler_tbl[i * 8 + j])
				rt9467_irq_handler_tbl[i * 8 + j](info);
		}
	}

err_read_irq:
	return IRQ_HANDLED;
}

static int rt9467_irq_register(struct rt9467_info *info)
{
	int ret = 0, len = 0;
	char *name = NULL;

	dev_info(info->dev, "%s\n", __func__);

	info->irq = info->client->irq;
	dev_info(info->dev, "%s: irq = %d\n", __func__, info->irq);

	/* Request threaded IRQ */
	name = devm_kzalloc(info->dev, len + 5, GFP_KERNEL);
	snprintf(name, len + 5, "%s_irq", info->desc->chg_dev_name);
	ret = devm_request_threaded_irq(info->dev, info->irq, NULL,
		rt9467_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT, name,
		info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: request thread irq fail\n",
			__func__);
		return ret;
	}
	device_init_wakeup(info->dev, true);

	return 0;
}

static inline int rt9467_maskall_irq(struct rt9467_info *info)
{
	dev_info(info->dev, "%s\n", __func__);
	return rt9467_i2c_block_write(info, RT9467_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(rt9467_irq_maskall), rt9467_irq_maskall);
}

static inline int rt9467_irq_init(struct rt9467_info *info)
{
	dev_info(info->dev, "%s\n", __func__);
	return rt9467_i2c_block_write(info, RT9467_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(info->irq_mask), info->irq_mask);
}

static bool rt9467_is_hw_exist(struct rt9467_info *info)
{
	int ret = 0;
	u8 vendor_id = 0, chip_rev = 0;

	ret = i2c_smbus_read_byte_data(info->client, RT9467_REG_DEVICE_ID);
	if (ret < 0)
		return false;

	vendor_id = ret & 0xF0;
	chip_rev = ret & 0x0F;
	if (vendor_id != RT9467_VENDOR_ID) {
		dev_notice(info->dev, "%s: vendor id is incorrect (0x%02X)\n",
			__func__, vendor_id);
		return false;
	}

	dev_info(info->dev, "%s: 0x%02X\n", __func__, chip_rev);
	info->chip_rev = chip_rev;

	return true;
}

static int rt9467_set_safety_timer(struct rt9467_info *info, u32 hr)
{
	u8 reg_st = 0;

	reg_st = rt9467_closest_reg_via_tbl(rt9467_safety_timer,
		ARRAY_SIZE(rt9467_safety_timer), hr);

	dev_info(info->dev, "%s: time = %d(0x%02X)\n", __func__, hr, reg_st);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL12,
		reg_st << RT9467_SHIFT_WT_FC, RT9467_MASK_WT_FC);
}

static inline int rt9467_enable_wdt(struct rt9467_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	ret = (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL13, RT9467_MASK_WDT_EN);
	if (ret < 0)
		return ret;

	if (en)
		schedule_delayed_work(&info->kick_wdt_dwork,
				      msecs_to_jiffies(30000));
	else
		cancel_delayed_work_sync(&info->kick_wdt_dwork);

	return 0;
}

static inline int rt9467_select_input_current_limit(struct rt9467_info *info,
	enum rt9467_iin_limit_sel sel)
{
	dev_info(info->dev, "%s: sel = %d\n", __func__, sel);
	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL2,
		sel << RT9467_SHIFT_IINLMTSEL, RT9467_MASK_IINLMTSEL);
}

static int rt9467_enable_hidden_mode(struct rt9467_info *info, bool en)
{
	int ret = 0;

	mutex_lock(&info->hidden_mode_lock);

	if (en) {
		if (info->hidden_mode_cnt == 0) {
			ret = rt9467_i2c_block_write(info, 0x70,
				ARRAY_SIZE(rt9467_val_en_hidden_mode),
				rt9467_val_en_hidden_mode);
			if (ret < 0)
				goto err;
		}
		info->hidden_mode_cnt++;
	} else {
		if (info->hidden_mode_cnt == 1) /* last one */
			ret = rt9467_i2c_write_byte(info, 0x70, 0x00);
		info->hidden_mode_cnt--;
		if (ret < 0)
			goto err;
	}
	dev_dbg(info->dev, "%s: en = %d\n", __func__, en);
	goto out;

err:
	dev_notice(info->dev, "%s: en = %d fail(%d)\n", __func__, en, ret);
out:
	mutex_unlock(&info->hidden_mode_lock);
	return ret;
}

static int rt9467_sw_workaround(struct rt9467_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	rt9467_enable_hidden_mode(info, true);

	/* Modify UG driver */
	ret = rt9467_i2c_update_bits(info, RT9467_REG_CHG_HIDDEN_CTRL4, 0xC0,
		0xF0);
	if (ret < 0)
		dev_notice(info->dev, "%s: set UG driver fail\n", __func__);

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_HIDDEN_CTRL4);
	dev_info(info->dev, "%s: reg0x23 = 0x%02X\n", __func__, ret);

	/* Disable TS auto sensing */
	ret = rt9467_clr_bit(info, RT9467_REG_CHG_HIDDEN_CTRL15, 0x01);
	if (ret < 0)
		goto out;

	/* Only revision <= E1 needs the following workaround */
	if (info->chip_rev > RT9467_CHIP_REV_E1)
		goto out;

	/* ICC: modify sensing node, make it more accurate */
	ret = rt9467_i2c_write_byte(info, RT9467_REG_CHG_HIDDEN_CTRL8, 0x00);
	if (ret < 0)
		goto out;

	/* DIMIN level */
	ret = rt9467_i2c_write_byte(info, RT9467_REG_CHG_HIDDEN_CTRL9, 0x86);

out:
	rt9467_enable_hidden_mode(info, false);
	return ret;
}

static inline int rt9467_enable_hz(struct rt9467_info *info, bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL1, RT9467_MASK_HZ_EN);
}

/* Reset all registers' value to default */
static int rt9467_reset_chip(struct rt9467_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	/* disable hz before reset chip */
	ret = rt9467_enable_hz(info, false);
	if (ret < 0) {
		dev_notice(info->dev, "%s: disable hz fail\n", __func__);
		return ret;
	}

	return rt9467_set_bit(info, RT9467_REG_CORE_CTRL0, RT9467_MASK_RST);
}

static inline int rt9467_enable_te(struct rt9467_info *info, bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL2, RT9467_MASK_TE_EN);
}

static inline int rt9467_enable_safety_timer(struct rt9467_info *info,
	bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL12, RT9467_MASK_TMR_EN);
}

static int __rt9467_set_ieoc(struct rt9467_info *info, u32 ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	/* IEOC workaround */
	if (info->ieoc_wkard)
		ieoc += 100000; /* 100mA */

	reg_ieoc = rt9467_closest_reg(RT9467_IEOC_MIN, RT9467_IEOC_MAX,
		RT9467_IEOC_STEP, ieoc);

	dev_info(info->dev, "%s: ieoc = %d(0x%02X)\n", __func__, ieoc,
		reg_ieoc);

	ret = rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL9,
		reg_ieoc << RT9467_SHIFT_IEOC, RT9467_MASK_IEOC);

	/* Store IEOC */
	return rt9467_get_ieoc(info, &info->ieoc);
}

static int rt9467_get_mivr(struct rt9467_info *info, u32 *mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_CTRL6);
	if (ret < 0)
		return ret;
	reg_mivr = ((ret & RT9467_MASK_MIVR) >> RT9467_SHIFT_MIVR) & 0xFF;

	*mivr = rt9467_closest_value(RT9467_MIVR_MIN, RT9467_MIVR_MAX,
		RT9467_MIVR_STEP, reg_mivr);

	return ret;
}

static int __rt9467_set_mivr(struct rt9467_info *info, u32 mivr)
{
	u8 reg_mivr = 0;

	reg_mivr = rt9467_closest_reg(RT9467_MIVR_MIN, RT9467_MIVR_MAX,
		RT9467_MIVR_STEP, mivr);

	dev_info(info->dev, "%s: mivr = %d(0x%02X)\n", __func__, mivr,
		reg_mivr);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL6,
		reg_mivr << RT9467_SHIFT_MIVR, RT9467_MASK_MIVR);
}

static inline int rt9467_enable_jeita(struct rt9467_info *info, bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL16, RT9467_MASK_JEITA_EN);
}


static int rt9467_get_charging_status(struct rt9467_info *info,
	enum rt9467_charging_status *chg_stat)
{
	int ret = 0;

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_STAT);
	if (ret < 0)
		return ret;

	*chg_stat = (ret & RT9467_MASK_CHG_STAT) >> RT9467_SHIFT_CHG_STAT;

	return ret;
}

static int rt9467_get_ieoc(struct rt9467_info *info, u32 *ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_CTRL9);
	if (ret < 0)
		return ret;

	reg_ieoc = (ret & RT9467_MASK_IEOC) >> RT9467_SHIFT_IEOC;
	*ieoc = rt9467_closest_value(RT9467_IEOC_MIN, RT9467_IEOC_MAX,
		RT9467_IEOC_STEP, reg_ieoc);

	return ret;
}

static inline int rt9467_is_charging_enable(struct rt9467_info *info, bool *en)
{
	return rt9467_i2c_test_bit(info, RT9467_REG_CHG_CTRL2,
		RT9467_SHIFT_CHG_EN, en);
}

static inline int rt9467_is_otg_enable(struct rt9467_info *info, bool *en)
{
	return rt9467_i2c_test_bit(info, RT9467_REG_CHG_CTRL1,
		RT9467_SHIFT_OPA_MODE, en);
}

static int rt9467_get_ichg(struct rt9467_info *info, u32 *ichg)
{
	int ret = 0;
	u8 reg_ichg = 0;

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_CTRL7);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & RT9467_MASK_ICHG) >> RT9467_SHIFT_ICHG;
	*ichg = rt9467_closest_value(RT9467_ICHG_MIN, RT9467_ICHG_MAX,
		RT9467_ICHG_STEP, reg_ichg);

	return ret;
}

static inline int rt9467_ichg_workaround(struct rt9467_info *info, u32 uA)
{
	int ret = 0;

	/* Vsys short protection */
	rt9467_enable_hidden_mode(info, true);

	if (info->ichg >= 900000 && uA < 900000)
		ret = rt9467_i2c_update_bits(info, RT9467_REG_CHG_HIDDEN_CTRL7,
			0x00, 0x60);
	else if (uA >= 900000 && info->ichg < 900000)
		ret = rt9467_i2c_update_bits(info, RT9467_REG_CHG_HIDDEN_CTRL7,
			0x40, 0x60);

	rt9467_enable_hidden_mode(info, false);
	return ret;
}

static int __rt9467_set_ichg(struct rt9467_info *info, u32 ichg)
{
	int ret = 0;
	u8 reg_ichg = 0;

	ichg = (ichg < 500000) ? 500000 : ichg;
	rt9467_ichg_workaround(info, ichg);

	reg_ichg = rt9467_closest_reg(RT9467_ICHG_MIN, RT9467_ICHG_MAX,
		RT9467_ICHG_STEP, ichg);

	dev_info(info->dev, "%s: ichg = %d(0x%02X)\n", __func__, ichg,
		reg_ichg);

	ret = rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL7,
		reg_ichg << RT9467_SHIFT_ICHG, RT9467_MASK_ICHG);
	if (ret < 0)
		return ret;

	/* Store Ichg setting */
	rt9467_get_ichg(info, &info->ichg);

	/* Workaround to make IEOC accurate */
	if (ichg < 900000 && !info->ieoc_wkard) { /* 900mA */
		ret = __rt9467_set_ieoc(info, info->ieoc + 100000);
		info->ieoc_wkard = true;
	} else if (ichg >= 900000 && info->ieoc_wkard) {
		info->ieoc_wkard = false;
		ret = __rt9467_set_ieoc(info, info->ieoc - 100000);
	}

	return ret;
}

static int rt9467_set_cv(struct rt9467_info *info, u32 cv)
{
	u8 reg_cv = 0;

	reg_cv = rt9467_closest_reg(RT9467_CV_MIN, RT9467_CV_MAX,
		RT9467_CV_STEP, cv);

	dev_info(info->dev, "%s: cv = %d(0x%02X)\n", __func__, cv, reg_cv);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL4,
		reg_cv << RT9467_SHIFT_CV, RT9467_MASK_CV);
}

static int rt9467_set_ircmp_resistor(struct rt9467_info *info, u32 uohm)
{
	u8 reg_resistor = 0;

	reg_resistor = rt9467_closest_reg(RT9467_IRCMP_RES_MIN,
		RT9467_IRCMP_RES_MAX, RT9467_IRCMP_RES_STEP, uohm);

	dev_info(info->dev, "%s: resistor = %d(0x%02X)\n", __func__, uohm,
		reg_resistor);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL18,
		reg_resistor << RT9467_SHIFT_IRCMP_RES, RT9467_MASK_IRCMP_RES);
}

static int rt9467_set_ircmp_vclamp(struct rt9467_info *info, u32 uV)
{
	u8 reg_vclamp = 0;

	reg_vclamp = rt9467_closest_reg(RT9467_IRCMP_VCLAMP_MIN,
		RT9467_IRCMP_VCLAMP_MAX, RT9467_IRCMP_VCLAMP_STEP, uV);

	dev_info(info->dev, "%s: vclamp = %d(0x%02X)\n", __func__, uV,
		reg_vclamp);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL18,
		reg_vclamp << RT9467_SHIFT_IRCMP_VCLAMP,
		RT9467_MASK_IRCMP_VCLAMP);
}

static inline int rt9467_enable_irq_pulse(struct rt9467_info *info, bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL1, RT9467_MASK_IRQ_PULSE);
}

static inline int rt9467_get_irq_number(struct rt9467_info *info,
	const char *name)
{
	int i = 0;

	if (!name) {
		dev_notice(info->dev, "%s: null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rt9467_irq_mapping_tbl); i++) {
		if (!strcmp(name, rt9467_irq_mapping_tbl[i].name))
			return rt9467_irq_mapping_tbl[i].id;
	}

	return -EINVAL;
}

static int rt9467_parse_dt(struct rt9467_info *info, struct device *dev)
{
	int ret = 0, irq_cnt = 0;
	struct rt9467_desc *desc = NULL;
	struct device_node *np = dev->of_node;
	const char *name = NULL;
	int irqnum = 0;

	dev_info(info->dev, "%s\n", __func__);

	if (!np) {
		dev_notice(info->dev, "%s: no device node\n", __func__);
		return -EINVAL;
	}

	info->desc = &rt9467_default_desc;

	desc = devm_kzalloc(dev, sizeof(struct rt9467_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &rt9467_default_desc, sizeof(struct rt9467_desc));

	if (of_property_read_string(np, "charger_name",
		&desc->chg_dev_name) < 0)
		dev_notice(info->dev, "%s: no charger name\n", __func__);

	if (of_property_read_u32(np, "regmap_represent_slave_addr",
		&desc->regmap_represent_slave_addr) < 0)
		dev_notice(info->dev, "%s: no regmap slave addr\n", __func__);

	if (of_property_read_string(np, "regmap_name",
		&(desc->regmap_name)) < 0)
		dev_notice(info->dev, "%s: no regmap name\n", __func__);

	if (of_property_read_u32(np, "ichg", &desc->ichg) < 0)
		dev_notice(info->dev, "%s: no ichg\n", __func__);

	if (of_property_read_u32(np, "aicr", &desc->aicr) < 0)
		dev_notice(info->dev, "%s: no aicr\n", __func__);

	if (of_property_read_u32(np, "mivr", &desc->mivr) < 0)
		dev_notice(info->dev, "%s: no mivr\n", __func__);

	if (of_property_read_u32(np, "cv", &desc->cv) < 0)
		dev_notice(info->dev, "%s: no cv\n", __func__);

	if (of_property_read_u32(np, "ieoc", &desc->ieoc) < 0)
		dev_notice(info->dev, "%s: no ieoc\n", __func__);

	if (of_property_read_u32(np, "safety_timer", &desc->safety_timer) < 0)
		dev_notice(info->dev, "%s: no safety timer\n", __func__);

	if (of_property_read_u32(np, "ircmp_resistor",
		&desc->ircmp_resistor) < 0)
		dev_notice(info->dev, "%s: no ircmp resistor\n", __func__);

	if (of_property_read_u32(np, "ircmp_vclamp", &desc->ircmp_vclamp) < 0)
		dev_notice(info->dev, "%s: no ircmp vclamp\n", __func__);

	desc->en_te = of_property_read_bool(np, "en_te");
	desc->en_wdt = of_property_read_bool(np, "en_wdt");
	desc->en_irq_pulse = of_property_read_bool(np, "en_irq_pulse");
	desc->en_jeita = of_property_read_bool(np, "en_jeita");
	desc->en_chgdet = of_property_read_bool(np, "en_chgdet");

	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
			irq_cnt, &name);
		if (ret < 0)
			break;
		irq_cnt++;
		irqnum = rt9467_get_irq_number(info, name);
		if (irqnum >= 0)
			rt9467_irq_unmask(info, irqnum);
	}

	info->desc = desc;

	return 0;
}

/* =========================================================== */
/* Released interfaces                                         */
/* =========================================================== */

static int rt9467_enable_charging(struct rt9467_info *info, bool en)
{
	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	return (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL2, RT9467_MASK_CHG_EN);
}

static int rt9467_set_boost_current_limit(struct rt9467_info *info,
	u32 current_limit)
{
	u8 reg_ilimit = 0;

	reg_ilimit = rt9467_closest_reg_via_tbl(rt9467_boost_oc_threshold,
		ARRAY_SIZE(rt9467_boost_oc_threshold), current_limit);

	dev_info(info->dev, "%s: boost ilimit = %d(0x%02X)\n", __func__,
		current_limit, reg_ilimit);

	return rt9467_i2c_update_bits(info, RT9467_REG_CHG_CTRL10,
		reg_ilimit << RT9467_SHIFT_BOOST_OC, RT9467_MASK_BOOST_OC);
}

static int rt9467_enable_otg(struct rt9467_info *info, bool en)
{
	int ret = 0;
	bool en_otg = false;
	u8 hidden_val = en ? 0x00 : 0x0F;
	u8 lg_slew_rate = en ? 0xCC : 0xC3;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	rt9467_enable_hidden_mode(info, true);

	/* Set OTG_OC to 1.8A */
	ret = rt9467_set_boost_current_limit(info, 1800000);
	if (ret < 0) {
		dev_notice(info->dev, "%s: set current limit fail\n", __func__);
		return ret;
	}

	/*
	 * Workaround : slow Low side mos Gate driver slew rate
	 * for decline VBUS noise
	 * reg[0x23] = 0xCC after entering OTG mode
	 * reg[0x23] = 0xC3 after leaving OTG mode
	 */
	ret = rt9467_i2c_write_byte(info, RT9467_REG_CHG_HIDDEN_CTRL4,
		lg_slew_rate);
	if (ret < 0) {
		dev_notice(info->dev,
			"%s: set Low side mos Gate drive speed fail(%d)\n",
			__func__, ret);
		goto out;
	}

	/* Switch OPA mode */
	ret = (en ? rt9467_set_bit : rt9467_clr_bit)
		(info, RT9467_REG_CHG_CTRL1, RT9467_MASK_OPA_MODE);

	msleep(20);

	if (en) {
		ret = rt9467_i2c_test_bit(info, RT9467_REG_CHG_CTRL1,
			RT9467_SHIFT_OPA_MODE, &en_otg);
		if (ret < 0 || !en_otg) {
			dev_notice(info->dev, "%s: otg fail(%d)\n", __func__,
				ret);
			goto err_en_otg;
		}
	}

	/*
	 * Workaround reg[0x25] = 0x00 after entering OTG mode
	 * reg[0x25] = 0x0F after leaving OTG mode
	 */
	ret = rt9467_i2c_write_byte(info, RT9467_REG_CHG_HIDDEN_CTRL6,
		hidden_val);
	if (ret < 0)
		dev_notice(info->dev, "%s: workaroud fail(%d)\n", __func__,
			ret);

	goto out;

err_en_otg:
	/* Recover Low side mos Gate slew rate */
	ret = rt9467_i2c_write_byte(info, RT9467_REG_CHG_HIDDEN_CTRL4, 0x73);
	if (ret < 0)
		dev_notice(info->dev,
			"%s: recover Low side mos Gate drive speed fail(%d)\n",
			__func__, ret);
	ret = -EIO;
out:
	rt9467_enable_hidden_mode(info, false);
	return ret;
}

static int rt9467_enable_discharge(struct rt9467_info *info, bool en)
{
	int ret = 0, i = 0;
	const int check_dischg_max = 3;
	bool is_dischg = true;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = rt9467_enable_hidden_mode(info, true);
	if (ret < 0)
		return ret;

	/* Set bit2 of reg[0x21] to 1 to enable discharging */
	ret = (en ? rt9467_set_bit : rt9467_clr_bit)(info,
		RT9467_REG_CHG_HIDDEN_CTRL2, 0x04);
	if (ret < 0) {
		dev_notice(info->dev, "%s: en = %d, fail\n", __func__, en);
		return ret;
	}

	if (!en) {
		for (i = 0; i < check_dischg_max; i++) {
			ret = rt9467_i2c_test_bit(info,
				RT9467_REG_CHG_HIDDEN_CTRL2, 2, &is_dischg);
			if (ret >= 0 && !is_dischg)
				break;
			/* Disable discharging */
			ret = rt9467_clr_bit(info, RT9467_REG_CHG_HIDDEN_CTRL2,
				0x04);
		}
		if (i == check_dischg_max)
			dev_notice(info->dev, "%s: disable dischg fail(%d)\n",
				__func__, ret);
	}

	rt9467_enable_hidden_mode(info, false);
	return ret;
}

static int rt9467_enable_power_path(struct rt9467_info *info, bool en)
{
	int ret = 0;

	mutex_lock(&info->pp_lock);

	dev_info(info->dev, "%s: en = %d, pp_en = %d\n",
			    __func__, en, info->pp_en);
	if (en == info->pp_en)
		goto out;

	ret = __rt9467_set_mivr(info, en ? info->mivr : RT9467_MIVR_MAX);
	if (ret >= 0)
		info->pp_en = en;
out:
	mutex_unlock(&info->pp_lock);

	return ret;
}

#if IS_ENABLED(CONFIG_TCPC_CLASS)
static int rt9467_set_chg_type(struct rt9467_info *info,
			       enum power_supply_type chg_type)
{
	int ret = 0;
	bool en = chg_type != POWER_SUPPLY_TYPE_UNKNOWN;

	if (!info->desc->en_chgdet) {
		dev_info(info->dev, "%s: bc12 is disabled by dts\n", __func__);
		return 0;
	}

	dev_info(info->dev, "%s: chg_type = %d\n", __func__, chg_type);

	atomic_set(&info->tcpc_usb_connected, en);

	mutex_lock(&info->bc12_access_lock);
	info->pwr_rdy = en;
	info->chg_psy_desc.type = chg_type;
	ret = __rt9467_enable_chgdet_flow(info, false);
	if (ret < 0)
		dev_notice(info->dev, "%s: disable chgdet fail\n", __func__);
	mutex_unlock(&info->bc12_access_lock);
	power_supply_changed(info->chg_psy);

	return 0;
}

static int rt9467_enable_chg_type_det(struct rt9467_info *info, bool en)
{
	int ret = 0;

	if (!info->desc->en_chgdet) {
		dev_info(info->dev, "%s: bc12 is disabled by dts\n", __func__);
		return 0;
	}

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	atomic_set(&info->tcpc_usb_connected, en);

	/* TypeC detach */
	if (!en) {
		ret = rt9467_chgdet_handler(info);
		return ret;
	}

	/* plug in, make usb switch to RT9467 */
	ret = rt9467_enable_chgdet_flow(info, true);
	if (ret < 0)
		dev_notice(info->dev, "%s: en chgdet fail(%d)\n", __func__,
			ret);

	return ret;
}
#endif /* CONFIG_TCPC_CLASS */

static int rt9467_set_ichg(struct rt9467_info *info, u32 ichg)
{
	int ret = 0;

	mutex_lock(&info->ichg_access_lock);
	mutex_lock(&info->ieoc_lock);
	ret = __rt9467_set_ichg(info, ichg);
	mutex_unlock(&info->ieoc_lock);
	mutex_unlock(&info->ichg_access_lock);

	return ret;
}

static int rt9467_set_ieoc(struct rt9467_info *info, u32 ieoc)
{
	int ret = 0;

	mutex_lock(&info->ichg_access_lock);
	mutex_lock(&info->ieoc_lock);
	ret = __rt9467_set_ieoc(info, ieoc);
	mutex_unlock(&info->ieoc_lock);
	mutex_unlock(&info->ichg_access_lock);

	return ret;
}

static int rt9467_set_aicr(struct rt9467_info *info, u32 aicr)
{
	int ret = 0;

	mutex_lock(&info->aicr_access_lock);
	ret = __rt9467_set_aicr(info, aicr);
	mutex_unlock(&info->aicr_access_lock);

	return ret;
}

static int rt9467_set_mivr(struct rt9467_info *info, u32 mivr)
{
	int ret = 0;

	mutex_lock(&info->pp_lock);

	if (!info->pp_en) {
		dev_notice(info->dev, "%s: power path is disabled\n", __func__);
		goto out;
	}

	ret = __rt9467_set_mivr(info, mivr);
out:
	if (ret >= 0)
		info->mivr = mivr;
	mutex_unlock(&info->pp_lock);

	return ret;
}

static int rt9467_get_aicr(struct rt9467_info *info, u32 *aicr)
{
	int ret = 0;
	u8 reg_aicr = 0;

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_CTRL3);
	if (ret < 0)
		return ret;

	reg_aicr = (ret & RT9467_MASK_AICR) >> RT9467_SHIFT_AICR;
	*aicr = rt9467_closest_value(RT9467_AICR_MIN, RT9467_AICR_MAX,
		RT9467_AICR_STEP, reg_aicr);

	return ret;
}

static int rt9467_get_cv(struct rt9467_info *info, u32 *cv)
{
	int ret = 0;
	u8 reg_cv = 0;

	ret = rt9467_i2c_read_byte(info, RT9467_REG_CHG_CTRL4);
	if (ret < 0)
		return ret;

	reg_cv = (ret & RT9467_MASK_CV) >> RT9467_SHIFT_CV;
	*cv = rt9467_closest_value(RT9467_CV_MIN, RT9467_CV_MAX,
		RT9467_CV_STEP, reg_cv);

	return ret;
}

static int rt9467_get_tchg(struct rt9467_info *info, int *tchg)
{
	int ret = 0, adc_temp = 0;
	u32 retry_cnt = 3;

	/* Get value from ADC */
	ret = rt9467_get_adc(info, RT9467_ADC_TEMP_JC, &adc_temp);
	if (ret < 0)
		return ret;

	/* Check unusual temperature */
	while (adc_temp >= 120 && retry_cnt > 0) {
		dev_notice(info->dev,
			   "%s: [WARNING] t = %d\n", __func__, adc_temp);
		rt9467_get_adc(info, RT9467_ADC_VBAT, &adc_temp);
		ret = rt9467_get_adc(info, RT9467_ADC_TEMP_JC, &adc_temp);
		retry_cnt--;
	}
	if (ret < 0)
		return ret;

	mutex_lock(&info->tchg_lock);
	/* Use previous one to prevent system from rebooting */
	if (adc_temp >= 120)
		adc_temp = info->tchg;
	else
		info->tchg = adc_temp;
	mutex_unlock(&info->tchg_lock);

	*tchg = adc_temp;

	dev_info(info->dev, "%s: temperature = %d\n", __func__, adc_temp);
	return ret;
}

static int rt9467_kick_wdt(struct rt9467_info *info)
{
	enum rt9467_charging_status chg_status = RT9467_CHG_STATUS_READY;

	/* Any I2C communication can reset watchdog timer */
	return rt9467_get_charging_status(info, &chg_status);
}

/*
 * This function is used in shutdown function
 * Use i2c smbus directly
 */
static int rt9467_sw_reset(struct rt9467_info *info)
{
	int ret = 0;
	u8 evt[RT9467_IRQIDX_MAX] = {0};

	/* Register 0x01 ~ 0x10 */
	u8 reg_data[] = {
		0x10, 0x03, 0x23, 0x3C, 0x67, 0x0B, 0x4C, 0xA1,
		0x3C, 0x58, 0x2C, 0x02, 0x52, 0x05, 0x00, 0x10
	};

	dev_info(info->dev, "%s\n", __func__);

	/* Mask all irq */
	mutex_lock(&info->i2c_access_lock);
	ret = rt9467_device_write(info->client, RT9467_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(rt9467_irq_maskall), rt9467_irq_maskall);
	if (ret < 0)
		dev_notice(info->dev, "%s: mask all irq fail\n", __func__);

	/* Read all irq */
	ret = rt9467_device_read(info->client, RT9467_REG_CHG_STATC, 5, evt);
	if (ret < 0)
		dev_notice(info->dev, "%s: read evt1 fail(%d)\n", __func__,
			ret);

	ret = rt9467_device_read(info->client, RT9467_REG_DPDM_IRQ, 1, &evt[6]);
	if (ret < 0)
		dev_notice(info->dev, "%s: read evt2 fail(%d)\n", __func__,
			ret);

	/* Reset necessary registers */
	ret = rt9467_device_write(info->client, RT9467_REG_CHG_CTRL1,
		ARRAY_SIZE(reg_data), reg_data);
	if (ret < 0)
		dev_notice(info->dev, "%s: reset registers fail\n", __func__);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static int rt9467_init_setting(struct rt9467_info *info)
{
	int ret = 0;
	struct rt9467_desc *desc = info->desc;
	u8 evt[RT9467_IRQIDX_MAX] = {0};

	dev_info(info->dev, "%s\n", __func__);

	/* disable USB charger type detection before reset IRQ */
	ret = rt9467_enable_chgdet_flow(info, false);
	if (ret < 0) {
		dev_notice(info->dev, "%s: disable chgdet fail\n", __func__);
		goto err;
	}

	ret = rt9467_set_bit(info, RT9467_REG_CHG_DPDM6,
			     RT9467_MASK_TIMEOUT_BCDONE);
	if (ret < 0) {
		dev_notice(info->dev,
			   "%s: extend HVDCP detection time fail(%d)\n",
			   __func__, ret);
		goto err;
	}

	/* mask all irq */
	ret = rt9467_maskall_irq(info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: mask all irq fail\n", __func__);
		goto err;
	}

	/* clear event */
	ret = rt9467_i2c_block_read(info, RT9467_REG_CHG_STATC, ARRAY_SIZE(evt),
		evt);
	if (ret < 0) {
		dev_notice(info->dev, "%s: clr evt fail(%d)\n", __func__, ret);
		goto err;
	}

	ret = __rt9467_set_ichg(info, desc->ichg);
	if (ret < 0)
		dev_notice(info->dev, "%s: set ichg fail\n", __func__);

	ret = __rt9467_set_aicr(info, desc->aicr);
	if (ret < 0)
		dev_notice(info->dev, "%s: set aicr fail\n", __func__);

	ret = rt9467_set_mivr(info, desc->mivr);
	if (ret < 0)
		dev_notice(info->dev, "%s: set mivr fail\n", __func__);

	ret = rt9467_set_cv(info, desc->cv);
	if (ret < 0)
		dev_notice(info->dev, "%s: set cv fail\n", __func__);

	ret = __rt9467_set_ieoc(info, desc->ieoc);
	if (ret < 0)
		dev_notice(info->dev, "%s: set ieoc fail\n", __func__);

	ret = rt9467_enable_te(info, desc->en_te);
	if (ret < 0)
		dev_notice(info->dev, "%s: set te fail\n", __func__);

	ret = rt9467_set_safety_timer(info, desc->safety_timer);
	if (ret < 0)
		dev_notice(info->dev, "%s: set fast timer fail\n", __func__);

	ret = rt9467_enable_safety_timer(info, true);
	if (ret < 0)
		dev_notice(info->dev, "%s: enable chg timer fail\n", __func__);

	ret = rt9467_enable_wdt(info, desc->en_wdt);
	if (ret < 0)
		dev_notice(info->dev, "%s: set wdt fail\n", __func__);

	ret = rt9467_enable_jeita(info, desc->en_jeita);
	if (ret < 0)
		dev_notice(info->dev, "%s: disable jeita fail\n", __func__);

	ret = rt9467_enable_irq_pulse(info, desc->en_irq_pulse);
	if (ret < 0)
		dev_notice(info->dev, "%s: set irq pulse fail\n", __func__);

	/* set ircomp according to BIF */
	ret = rt9467_set_ircmp_resistor(info, desc->ircmp_resistor);
	if (ret < 0)
		dev_notice(info->dev, "%s: set ircmp resistor fail\n",
			__func__);

	ret = rt9467_set_ircmp_vclamp(info, desc->ircmp_vclamp);
	if (ret < 0)
		dev_notice(info->dev, "%s: set ircmp clamp fail\n", __func__);

	ret = rt9467_sw_workaround(info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: workaround fail\n", __func__);
		return ret;
	}
err:
	return ret;
}

static int rt9467_run_aicl(struct rt9467_info *info, u32 *uW)
{
	int ret = 0;

	ret = __rt9467_run_aicl(info);
	if (ret >= 0)
		*uW = info->power_limit;

	return ret;
}

static int rt9467_dump_register(struct rt9467_info *info)
{
	int i = 0, ret = 0;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0, cv = 0;
	bool chg_en = 0;
	int adc_vsys = 0, adc_vbat = 0, adc_ibat = 0;
	int adc_ibus = 0, adc_vbus = 0;
	enum rt9467_charging_status chg_status = RT9467_CHG_STATUS_READY;
	u8 chg_stat = 0, chg_ctrl[2] = {0};

	ret = rt9467_get_ichg(info, &ichg);
	ret = rt9467_get_aicr(info, &aicr);
	ret = rt9467_get_mivr(info, &mivr);
	ret = rt9467_is_charging_enable(info, &chg_en);
	ret = rt9467_get_ieoc(info, &ieoc);
	ret = rt9467_get_cv(info, &cv);
	ret = rt9467_get_charging_status(info, &chg_status);
	ret = rt9467_get_adc(info, RT9467_ADC_VSYS, &adc_vsys);
	ret = rt9467_get_adc(info, RT9467_ADC_VBAT, &adc_vbat);
	ret = rt9467_get_adc(info, RT9467_ADC_IBAT, &adc_ibat);
	ret = rt9467_get_adc(info, RT9467_ADC_IBUS, &adc_ibus);
	ret = rt9467_get_adc(info, RT9467_ADC_VBUS_DIV5, &adc_vbus);
	chg_stat = rt9467_i2c_read_byte(info, RT9467_REG_CHG_STATC);
	ret = rt9467_i2c_block_read(info, RT9467_REG_CHG_CTRL1, 2, chg_ctrl);

	/* Charging fault, dump all registers' value */
	if (chg_status == RT9467_CHG_STATUS_FAULT) {
		for (i = 0; i < ARRAY_SIZE(rt9467_reg_addr); i++)
			ret = rt9467_i2c_read_byte(info, rt9467_reg_addr[i]);
	}

	dev_info(info->dev,
	"%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA, CV = %dmV\n",
	__func__, ichg / 1000, aicr / 1000, mivr / 1000,
	ieoc / 1000, cv / 1000);

	dev_info(info->dev,
	"%s: VSYS = %dmV, VBAT = %dmV, IBAT = %dmA, IBUS = %dmA, VBUS = %dmV\n",
	__func__, adc_vsys / 1000, adc_vbat / 1000, adc_ibat / 1000,
	adc_ibus / 1000, adc_vbus / 1000);

	dev_info(info->dev,
		"%s: CHG_EN = %d, CHG_STATUS = %s, CHG_STAT = 0x%02X\n",
		__func__, chg_en, rt9467_chg_status_name[chg_status], chg_stat);

	dev_info(info->dev, "%s: CHG_CTRL1 = 0x%02X, CHG_CTRL2 = 0x%02X\n",
		__func__, chg_ctrl[0], chg_ctrl[1]);

	return 0;
}

static enum power_supply_property rt9467_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_INPUT_POWER_LIMIT,
};

static int rt9467_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	int ret = 0;
	struct rt9467_info *info = power_supply_get_drvdata(psy);
	bool en = false;
	enum rt9467_charging_status chg_status = RT9467_CHG_STATUS_READY;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->pwr_rdy;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = info->chg_psy_desc.type;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!info->pwr_rdy) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		ret = rt9467_get_charging_status(info, &chg_status);
		if (ret < 0) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		switch (chg_status) {
		case RT9467_CHG_STATUS_READY:
		case RT9467_CHG_STATUS_FAULT:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case RT9467_CHG_STATUS_PROGRESS:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case RT9467_CHG_STATUS_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = rt9467_get_tchg(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = rt9467_get_mivr(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = rt9467_get_aicr(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = rt9467_get_cv(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = rt9467_get_ichg(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = rt9467_get_ieoc(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = rt9467_is_otg_enable(info, &en);
		if (ret < 0) {
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
			break;
		}
		if (en) {
			val->intval = POWER_SUPPLY_CHARGE_TYPE_CUSTOM;
			break;
		}
		ret = rt9467_is_charging_enable(info, &en);
		if (ret < 0) {
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
			break;
		}
		val->intval = en ? POWER_SUPPLY_CHARGE_TYPE_STANDARD :
				   POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_INPUT_POWER_LIMIT:
		ret = rt9467_run_aicl(info, &val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rt9467_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	int ret = 0;
	struct rt9467_info *info = power_supply_get_drvdata(psy);

	switch (psp) {
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	case POWER_SUPPLY_PROP_ONLINE:
		ret = rt9467_enable_chg_type_det(info, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		ret = rt9467_set_chg_type(info, val->intval);
		break;
#endif /* CONFIG_TCPC_CLASS */
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = rt9467_set_mivr(info, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = rt9467_set_aicr(info, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = rt9467_set_cv(info, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = rt9467_set_ichg(info, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = rt9467_set_ieoc(info, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (val->intval) {
		case POWER_SUPPLY_CHARGE_TYPE_CUSTOM:
			ret = rt9467_enable_otg(info, true);
			break;
		case POWER_SUPPLY_CHARGE_TYPE_STANDARD:
			ret = rt9467_enable_otg(info, false);
			if (ret < 0)
				break;
			ret = rt9467_enable_charging(info, true);
			break;
		default:
			ret = rt9467_enable_otg(info, false);
			if (ret < 0)
				break;
			ret = rt9467_enable_charging(info, false);
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		ret = rt9467_enable_discharge(info, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_POWER_LIMIT:
		ret = rt9467_enable_power_path(info, !!val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rt9467_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_TYPE:
#endif /* CONFIG_TCPC_CLASS */
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
	case POWER_SUPPLY_PROP_INPUT_POWER_LIMIT:
		return true;
	default:
		return false;
	}
}

static struct power_supply_desc rt9467_chg_psy_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = rt9467_properties,
	.num_properties = ARRAY_SIZE(rt9467_properties),
	.get_property = rt9467_get_property,
	.set_property = rt9467_set_property,
	.property_is_writeable = rt9467_property_is_writeable,
};

static void rt9467_init_setting_work_handler(struct work_struct *work)
{
	int ret = 0, retry_cnt = 0;
	struct rt9467_info *info = container_of(work,
		struct rt9467_info, init_work);

	do {
		/* Select IINLMTSEL to use AICR */
		ret = rt9467_select_input_current_limit(info,
			RT9467_IINLMTSEL_AICR);
		if (ret < 0) {
			dev_notice(info->dev, "%s: sel ilmtsel fail\n",
				__func__);
			retry_cnt++;
		}
	} while (retry_cnt < 5 && ret < 0);

	msleep(150);

	retry_cnt = 0;
	do {
		/* Disable hardware ILIM */
		ret = rt9467_enable_ilim(info, false);
		if (ret < 0) {
			dev_notice(info->dev, "%s: disable ilim fail\n",
				__func__);
			retry_cnt++;
		}
	} while (retry_cnt < 5 && ret < 0);

	rt9467_dump_register(info);

	/* Schedule work for microB's BC1.2 */
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	if (info->desc->en_chgdet)
		schedule_work(&info->chgdet_work);
#endif /* CONFIG_TCPC_CLASS */
}

static void rt9467_kick_wdt_dwork_handler(struct work_struct *work)
{
	int ret = 0;
	struct rt9467_info *info = container_of(work,
		struct rt9467_info, kick_wdt_dwork.work);

	ret = rt9467_wdtmri_irq_handler(info);
	schedule_delayed_work(&info->kick_wdt_dwork,
			      msecs_to_jiffies(ret < 0 ? 1000 : 30000));
}

/* ========================= */
/* I2C driver function       */
/* ========================= */

static int rt9467_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct rt9467_info *info = NULL;
	struct power_supply_config psy_cfg = {};

	pr_info("%s(%s)\n", __func__, RT9467_DRV_VERSION);

	info = devm_kzalloc(&client->dev, sizeof(struct rt9467_info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	mutex_init(&info->i2c_access_lock);
	mutex_init(&info->adc_access_lock);
	mutex_init(&info->irq_access_lock);
	mutex_init(&info->aicr_access_lock);
	mutex_init(&info->ichg_access_lock);
	mutex_init(&info->hidden_mode_lock);
	mutex_init(&info->bc12_access_lock);
	mutex_init(&info->ieoc_lock);
	mutex_init(&info->tchg_lock);
	mutex_init(&info->pp_lock);
	atomic_set(&info->bc12_sdp_cnt, 0);
	atomic_set(&info->bc12_wkard, 0);
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	atomic_set(&info->tcpc_usb_connected, 0);
#endif /* CONFIG_TCPC_CLASS */

	info->client = client;
	info->dev = &client->dev;
	info->power_limit = 0;
	info->hidden_mode_cnt = 0;
	info->pp_en = true;
	info->bc12_en = true;
	info->ieoc_wkard = false;
	info->ieoc = 250000; /* register default value 250mA */
	info->ichg = 2000000; /* register default value 2000mA */
	info->tchg = 25;
	memcpy(info->irq_mask, rt9467_irq_maskall, RT9467_IRQIDX_MAX);

	/* Init wait queue head */
	init_waitqueue_head(&info->wait_queue);

	INIT_WORK(&info->init_work, rt9467_init_setting_work_handler);
	INIT_DELAYED_WORK(&info->kick_wdt_dwork,
			  rt9467_kick_wdt_dwork_handler);
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	INIT_WORK(&info->chgdet_work, rt9467_chgdet_work_handler);
#endif /* CONFIG_TCPC_CLASS */

	/* Is HW exist */
	if (!rt9467_is_hw_exist(info)) {
		dev_notice(info->dev, "%s: no rt9467 exists\n", __func__);
		ret = -ENODEV;
		goto err_no_dev;
	}
	i2c_set_clientdata(client, info);

	ret = rt9467_parse_dt(info, &client->dev);
	if (ret < 0) {
		dev_notice(info->dev, "%s: parse dt fail\n", __func__);
		goto err_parse_dt;
	}

#ifdef CONFIG_RT_REGMAP
	ret = rt9467_register_rt_regmap(info);
	if (ret < 0)
		goto err_register_regmap;
#endif /* CONFIG_RT_REGMAP */

	ret = rt9467_reset_chip(info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: reset chip fail\n", __func__);
		goto err_reset_chip;
	}

	ret = rt9467_init_setting(info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: init setting fail\n", __func__);
		goto err_init_setting;
	}

	memcpy(&info->chg_psy_desc, &rt9467_chg_psy_desc,
	       sizeof(info->chg_psy_desc));
	psy_cfg.of_node = info->dev->of_node;
	psy_cfg.drv_data = info;
	info->chg_psy = devm_power_supply_register(info->dev,
						   &info->chg_psy_desc,
						   &psy_cfg);
	if (IS_ERR(info->chg_psy)) {
		dev_notice(info->dev, "register chg psy fail\n");
		ret = PTR_ERR(info->chg_psy);
		goto err_register_chg_psy;
	}
	info->chg_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	ret = rt9467_irq_register(info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: irq register fail\n", __func__);
		goto err_irq_register;
	}

	ret = rt9467_irq_init(info);
	if (ret < 0) {
		dev_notice(info->dev, "%s: irq init fail\n", __func__);
		goto err_irq_init;
	}

	schedule_work(&info->init_work);
	dev_info(info->dev, "%s: successfully\n", __func__);
	return ret;

err_irq_init:
err_irq_register:
err_register_chg_psy:
err_init_setting:
err_reset_chip:
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(info->regmap_dev);
err_register_regmap:
#endif
err_parse_dt:
err_no_dev:
	mutex_destroy(&info->i2c_access_lock);
	mutex_destroy(&info->adc_access_lock);
	mutex_destroy(&info->irq_access_lock);
	mutex_destroy(&info->aicr_access_lock);
	mutex_destroy(&info->ichg_access_lock);
	mutex_destroy(&info->hidden_mode_lock);
	mutex_destroy(&info->bc12_access_lock);
	mutex_destroy(&info->ieoc_lock);
	mutex_destroy(&info->tchg_lock);
	mutex_destroy(&info->pp_lock);
	return ret;
}


static int rt9467_remove(struct i2c_client *client)
{
	int ret = 0;
	struct rt9467_info *info = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);

	if (info) {
#ifdef CONFIG_RT_REGMAP
		rt_regmap_device_unregister(info->regmap_dev);
#endif
		mutex_destroy(&info->i2c_access_lock);
		mutex_destroy(&info->adc_access_lock);
		mutex_destroy(&info->irq_access_lock);
		mutex_destroy(&info->aicr_access_lock);
		mutex_destroy(&info->ichg_access_lock);
		mutex_destroy(&info->hidden_mode_lock);
		mutex_destroy(&info->bc12_access_lock);
		mutex_destroy(&info->ieoc_lock);
		mutex_destroy(&info->tchg_lock);
		mutex_destroy(&info->pp_lock);
	}

	return ret;
}

static void rt9467_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct rt9467_info *info = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);
	if (info) {
		devm_free_irq(&client->dev, client->irq, info);

		ret = rt9467_sw_reset(info);
		if (ret < 0)
			pr_notice("%s: sw reset fail\n", __func__);
	}
}

static int rt9467_suspend(struct device *dev)
{
	struct rt9467_info *info = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(info->irq);
	disable_irq(info->irq);

	return 0;
}

static int rt9467_resume(struct device *dev)
{
	struct rt9467_info *info = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	enable_irq(info->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(info->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rt9467_pm_ops, rt9467_suspend, rt9467_resume);

static const struct i2c_device_id rt9467_i2c_id[] = {
	{"rt9467", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, rt9467_i2c_id);

static const struct of_device_id rt9467_of_match[] = {
	{ .compatible = "richtek,rt9467", },
	{},
};
MODULE_DEVICE_TABLE(of, rt9467_of_match);

static struct i2c_driver rt9467_i2c_driver = {
	.driver = {
		.name = "rt9467",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt9467_of_match),
		.pm = &rt9467_pm_ops,
	},
	.probe = rt9467_probe,
	.remove = rt9467_remove,
	.shutdown = rt9467_shutdown,
	.id_table = rt9467_i2c_id,
};

module_i2c_driver(rt9467_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lucas Tsai <lucas_tsai@richtek.com>");
MODULE_DESCRIPTION("RT9467 Charger Driver");
MODULE_VERSION(RT9467_DRV_VERSION);

/*
 * Release Note
 * 1.0.0
 * (1) Porting from 1.0.19_MTK
 * (2) Add HVDCP detection
 */
