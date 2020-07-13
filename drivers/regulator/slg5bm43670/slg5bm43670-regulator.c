// SPDX-License-Identifier: GPL-2.0+
/*
 * SLG51000 High PSRR, Multi-Output Regulators
 * Copyright (C) 2020  Dialog Semiconductor
 *
 * Author: Eric Jeong, Dialog Semiconductor
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>
#include "slg5bm43670-regulator.h"

#define SLG51000_SCTL_EVT               7
#define SLG51000_MAX_EVT_REGISTER       8

enum slg51000_regulators {
	SLG51000_REGULATOR_LDO1 = 0,
	SLG51000_REGULATOR_LDO2,
	SLG51000_REGULATOR_LDO3,
	SLG51000_REGULATOR_LDO4,
	SLG51000_REGULATOR_LDO5,
	SLG51000_REGULATOR_LDO6,
	SLG51000_REGULATOR_LDO7,
	SLG51000_MAX_REGULATORS,
};

struct slg51000 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc *rdesc[SLG51000_MAX_REGULATORS];
	struct regulator_dev *rdev[SLG51000_MAX_REGULATORS];
	int chip_irq;
	int chip_cs_pin;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
};

struct slg51000_evt_sta {
	unsigned int ereg;
	unsigned int sreg;
};

static const struct slg51000_evt_sta es_reg[SLG51000_MAX_EVT_REGISTER] = {
	{SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS},
	{SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS},
	{SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS},
	{SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS},
	{SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS},
	{SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS},
	{SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS},
	{SLG51000_SYSCTL_EVENT, SLG51000_SYSCTL_STATUS},
};

static const struct regmap_range slg51000_writeable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_MATRIX_CONF_A,
			 SLG51000_SYSCTL_MATRIX_CONF_A),
	regmap_reg_range(0x1119, 0x111D),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO1_CONF),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_16, SLG51000_MUXARRAY_INPUT_SEL_16),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO1_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO1_IRQ_MASK, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO2_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO2_IRQ_MASK, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO3_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO3_IRQ_MASK, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO4_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO4_IRQ_MASK, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO5_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO5_IRQ_MASK, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO6_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO6_IRQ_MASK, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO7_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO7_IRQ_MASK, SLG51000_LDO7_IRQ_MASK),
};

static const struct regmap_range slg51000_readable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_PATN_ID_B0,
			 SLG51000_SYSCTL_PATN_ID_B2),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_A,
			 SLG51000_SYSCTL_SYS_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_D,
			 SLG51000_SYSCTL_MATRIX_CONF_B),
	regmap_reg_range(SLG51000_SYSCTL_REFGEN_CONF_C,
			 SLG51000_SYSCTL_UVLO_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_IRQ_MASK),
	regmap_reg_range(0x1119, 0x111D),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_0,
			 SLG51000_LUTARRAY_LUT_VAL_11),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_0,
			 SLG51000_MUXARRAY_INPUT_SEL_63),
	regmap_reg_range(SLG51000_PWRSEQ_RESOURCE_EN_0,
			 SLG51000_PWRSEQ_INPUT_SENSE_CONF_B),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO1_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO1_TRIM2, SLG51000_LDO1_TRIM2),
	regmap_reg_range(SLG51000_LDO1_VSEL_ACTUAL, SLG51000_LDO1_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO2_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO2_TRIM2, SLG51000_LDO2_TRIM2),
	regmap_reg_range(SLG51000_LDO2_VSEL_ACTUAL, SLG51000_LDO2_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO3_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO3_TRIM2, SLG51000_LDO3_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO4_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO4_TRIM2, SLG51000_LDO4_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO5_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO5_TRIM2, SLG51000_LDO5_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO6_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO6_TRIM2, SLG51000_LDO6_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_VSEL_RANGE_MASK_MIN,
			 SLG51000_LDO7_VSEL_RANGE_MASK_MAX),
	regmap_reg_range(SLG51000_LDO7_TRIM2, SLG51000_LDO7_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_LOCK_GLOBAL_LOCK_CTRL1,
			 SLG51000_LOCK_GLOBAL_LOCK_CTRL1),
};

static const struct regmap_range slg51000_volatile_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_STATUS),
	regmap_reg_range(SLG51000_IO_GPIO_STATUS, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS),
};

static const struct regmap_access_table slg51000_writeable_table = {
	.yes_ranges	= slg51000_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_writeable_ranges),
};

static const struct regmap_access_table slg51000_readable_table = {
	.yes_ranges	= slg51000_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_readable_ranges),
};

static const struct regmap_access_table slg51000_volatile_table = {
	.yes_ranges	= slg51000_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_volatile_ranges),
};

static const struct regmap_config slg51000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x8000,
	.wr_table = &slg51000_writeable_table,
	.rd_table = &slg51000_readable_table,
	.volatile_table = &slg51000_volatile_table,
};

static const struct regulator_ops slg51000_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_ops slg51000_switch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int slg51000_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	int ena_gpio;

	ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (gpio_is_valid(ena_gpio))
		config->ena_gpiod = gpio_to_desc(ena_gpio);

	return 0;
}

#define SLG51000_REGL_DESC(_id, _name, _s_name, _min, _step)       \
	[SLG51000_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,                            \
		.id = SLG51000_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = slg51000_of_parse_cb,               \
		.ops = &slg51000_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = SLG51000_VSEL_MASK,                   \
		.vsel_reg = SLG51000_##_id##_VSEL,                 \
		.enable_reg = SLG51000_SYSCTL_MATRIX_CONF_A,       \
		.enable_mask = BIT(SLG51000_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc regls_desc[SLG51000_MAX_REGULATORS] = {
	SLG51000_REGL_DESC(LDO1, ldo1, NULL,   2200000,  5000),
	SLG51000_REGL_DESC(LDO2, ldo2, NULL,   2200000,  5000),
	SLG51000_REGL_DESC(LDO3, ldo3, "vin3", 1200000, 10000),
	SLG51000_REGL_DESC(LDO4, ldo4, "vin4", 1200000, 10000),
	SLG51000_REGL_DESC(LDO5, ldo5, "vin5",  400000,  5000),
	SLG51000_REGL_DESC(LDO6, ldo6, "vin6",  400000,  5000),
	SLG51000_REGL_DESC(LDO7, ldo7, "vin7", 1200000, 10000),
};

//=======================================
// Added by Aaron Wang on Dec 27th 2019
// Call this function to upgrade the firmware
// to the revision.
// CS must be high and wait 10ms
// Just a reference, need to fully re program this part.
//=======================================
static int upgrade_firmware(struct slg51000 *chip)
{
	unsigned int val;

	if (regmap_write(chip->regmap, 0x111A, 0x45) < 0)
		dev_err(chip->dev, "regmap_write val 0x45 to reg 0x111A fail...");

	if (regmap_write(chip->regmap, 0x111B, 0x53) < 0)
		dev_err(chip->dev, "regmap_write val 0x53 to reg 0x111B fail...");

	if (regmap_write(chip->regmap, 0x111C, 0x54) < 0)
		dev_err(chip->dev, "regmap_write val 0x54 to reg 0x111C fail...");

	if (regmap_write(chip->regmap, 0x111D, 0x4D) < 0)
		dev_err(chip->dev, "regmap_write val 0x4D to reg 0x111D fail...");

	/* If the 3rd bit is set to 1, then SLG51000 device is in test mode. */
	if (regmap_read(chip->regmap, 0x1119, &val) < 0)
		dev_err(chip->dev, "regmap_read reg 0x1119 fail...");

	dev_info(chip->dev, "Is test mode: (0x%X)%s\n", val, (val & 0x4) ? "TRUE" : "FALSE");

	if (regmap_read(chip->regmap, SLG51000_IO_GPIO1_CONF, &val) < 0)
		dev_err(chip->dev, "regmap_read reg SLG51000_IO_GPIO1_CONF fail...");

	dev_info(chip->dev, "GPIO1 Value: 0x%X\n", val);

	val |= (SLG51000_GPIO_DIR_MASK | SLG51000_GPIO_BYP_MASK);
	if (regmap_write(chip->regmap, SLG51000_IO_GPIO1_CONF, val) < 0)
		dev_err(chip->dev, "regmap_write val 0x%x to reg SLG51000_IO_GPIO1_CONF fail...", val);

	/* To make sure it has been written successfully */
	if (regmap_read(chip->regmap, SLG51000_IO_GPIO1_CONF, &val) < 0)
		dev_err(chip->dev, "regmap_read reg 0x1500 fail...");

	dev_info(chip->dev, "GPIO1 Config value: 0x%X\n", val);

	if (regmap_write(chip->regmap, SLG51000_MUXARRAY_INPUT_SEL_16, 0x2A) < 0)
		dev_err(chip->dev, "regmap_write val 0x%x to reg SLG51000_MUXARRAY_INPUT_SEL_16 fail...", val);

	if (regmap_read(chip->regmap, SLG51000_MUXARRAY_INPUT_SEL_16, &val) < 0)
		dev_err(chip->dev, "regmap_read reg SLG51000_MUXARRAY_INPUT_SEL_16 fail...");

	dev_info(chip->dev, "MUXARRAY_INPUT_SEL_16: 0x%X\n", val);

	if (regmap_write(chip->regmap, 0x1119, 0x0) < 0)
		dev_err(chip->dev, "regmap_write val 0x0 to reg 0x1119 fail...");

	if (regmap_read(chip->regmap, 0x1119, &val) < 0)
		dev_err(chip->dev, "regmap_read reg 0x1119 fail...");

	if (val) {
		if (regmap_write(chip->regmap, 0x1119, 0x00) < 0)
			dev_err(chip->dev, "regmap_write val 0x00 to reg 0x1119 fail...");
	}

	return 1;       // always return 1. But should add false or true return later.
}

static int slg51000_regulator_init(struct slg51000 *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	unsigned int reg, val;
	u8 vsel_range[2];
	int id, ret = 0;
	const unsigned int min_regs[SLG51000_MAX_REGULATORS] = {
		SLG51000_LDO1_VSEL_RANGE_MASK_MIN,
		SLG51000_LDO2_VSEL_RANGE_MASK_MIN,
		SLG51000_LDO3_VSEL_RANGE_MASK_MIN,
		SLG51000_LDO4_VSEL_RANGE_MASK_MIN,
		SLG51000_LDO5_VSEL_RANGE_MASK_MIN,
		SLG51000_LDO6_VSEL_RANGE_MASK_MIN,
		SLG51000_LDO7_VSEL_RANGE_MASK_MIN,
	};

	upgrade_firmware(chip);          // upgrade the firmware during initial

	for (id = 0; id < SLG51000_MAX_REGULATORS; id++) {
		chip->rdesc[id] = &regls_desc[id];
		rdesc = chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, min_regs[id],
				       vsel_range, 2);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the MIN register\n");
			return ret;
		}

		switch (id) {
		case SLG51000_REGULATOR_LDO5:
		case SLG51000_REGULATOR_LDO6:
			if (id == SLG51000_REGULATOR_LDO5)
				reg = SLG51000_LDO5_TRIM2;
			else
				reg = SLG51000_LDO6_TRIM2;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read LDO mode register\n");
				return ret;
			}

			if (val & SLG51000_SEL_BYP_MODE_MASK) {
				rdesc->ops = &slg51000_switch_ops;
				rdesc->n_voltages = 0;
				rdesc->min_uV = 0;
				rdesc->uV_step = 0;
				rdesc->linear_min_sel = 0;
				break;
			}
			/* Fall through - to the check below.*/

		default:
			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			rdesc->min_uV = rdesc->min_uV
					+ (vsel_range[0] * rdesc->uV_step);
			break;
		}

		chip->rdev[id] = devm_regulator_register(chip->dev, rdesc,
							 &config);
		if (IS_ERR(chip->rdev[id])) {
			ret = PTR_ERR(chip->rdev[id]);
			dev_err(chip->dev,
				"Failed to register regulator(%s):%d\n",
				chip->rdesc[id]->name, ret);
			return ret;
		}
	}

	return 0;
}

static irqreturn_t slg51000_irq_handler(int irq, void *data)
{
	struct slg51000 *chip = data;
	struct regmap *regmap = chip->regmap;
	enum { R0 = 0, R1, R2, REG_MAX };
	u8 evt[SLG51000_MAX_EVT_REGISTER][REG_MAX];
	int ret, i, handled = IRQ_NONE;

	/* Read event[R0], status[R1] and mask[R2] register */
	for (i = 0; i < SLG51000_MAX_EVT_REGISTER; i++) {
		ret = regmap_bulk_read(regmap, es_reg[i].ereg, evt[i], REG_MAX);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read event registers(%d)\n", ret);
			return IRQ_NONE;
		}
	}

	for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
		if (!(evt[i][R2] & SLG51000_IRQ_ILIM_FLAG_MASK) &&
		    (evt[i][R0] & SLG51000_EVT_ILIM_FLAG_MASK)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
			regulator_lock(chip->rdev[i]);
#else
			mutex_lock(&chip->rdev[i]->mutex);
#endif
			regulator_notifier_call_chain(chip->rdev[i],
						REGULATOR_EVENT_OVER_CURRENT,
						NULL);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
			regulator_unlock(chip->rdev[i]);
#else
			mutex_unlock(&chip->rdev[i]->mutex);
#endif
			if (evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK)
				dev_warn(chip->dev,
					 "Over-current limit(ldo%d)\n", i + 1);
			handled = IRQ_HANDLED;
		}
	}

	if (!(evt[SLG51000_SCTL_EVT][R2] & SLG51000_IRQ_HIGH_TEMP_WARN_MASK) &&
	    (evt[SLG51000_SCTL_EVT][R0] & SLG51000_EVT_HIGH_TEMP_WARN_MASK)) {
		for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
			if (!(evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK) &&
			    (evt[i][R1] & SLG51000_STA_VOUT_OK_FLAG_MASK)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
				regulator_lock(chip->rdev[i]);
#else
				mutex_lock(&chip->rdev[i]->mutex);
#endif
				regulator_notifier_call_chain(chip->rdev[i],
						REGULATOR_EVENT_OVER_TEMP,
						NULL);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
				regulator_unlock(chip->rdev[i]);
#else
				mutex_unlock(&chip->rdev[i]->mutex);
#endif
			}
		}
		handled = IRQ_HANDLED;
		if (evt[SLG51000_SCTL_EVT][R1] &
		    SLG51000_STA_HIGH_TEMP_WARN_MASK)
			dev_warn(chip->dev, "High temperature warning!\n");
	}

	return handled;
}

static void slg51000_clear_fault_log(struct slg51000 *chip)
{
	unsigned int val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, SLG51000_SYSCTL_FAULT_LOG1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read Fault log register\n");
		return;
	}

	dev_info(chip->dev, "SLG51000_SYSCTL_FAULT_LOG1(0x1115)'s val = 0x%X\n", val);

	if (val & SLG51000_FLT_OVER_TEMP_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_OVER_TEMP\n");
	if (val & SLG51000_FLT_POWER_SEQ_CRASH_REQ_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POWER_SEQ_CRASH_REQ\n");
	if (val & SLG51000_FLT_RST_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_RST\n");
	if (val & SLG51000_FLT_POR_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POR\n");
}

static int slg51000_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct slg51000 *chip;
	int error, cs_gpio, ret;

	chip = devm_kzalloc(dev, sizeof(struct slg51000), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	cs_gpio = of_get_named_gpio(dev->of_node, "dlg,cs-gpios", 0);
	if (cs_gpio > 0) {
		if (!gpio_is_valid(cs_gpio)) {
			dev_err(dev, "Invalid chip select pin\n");
			return -EPERM;
		}

		ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_HIGH,
					    "slg51000_cs_pin");
		if (ret) {
			dev_err(dev, "GPIO(%d) request failed(%d)\n",
				cs_gpio, ret);
			return ret;
		}

		chip->chip_cs_pin = cs_gpio;
	}

	chip->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(chip->pinctrl)) {
		chip->gpio_state_default = pinctrl_lookup_state(chip->pinctrl, "pmic_default");
	} else
		dev_err(dev, "Getting pinctrl handle failed\n");

	if (!IS_ERR_OR_NULL(chip->gpio_state_default)) {
		pinctrl_select_state(chip->pinctrl, chip->gpio_state_default);
		dev_err(dev, "Setting pinctrl handle successfully\n");
	}

	mdelay(10); //must delay 10ms, wait camera pmic stable.

	i2c_set_clientdata(client, chip);
	chip->chip_irq = client->irq;
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &slg51000_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	ret = slg51000_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	slg51000_clear_fault_log(chip);

	if (chip->chip_irq) {
		ret = devm_request_threaded_irq(dev, chip->chip_irq, NULL,
						slg51000_irq_handler,
						(IRQF_TRIGGER_LOW |
						IRQF_ONESHOT),
						"slg51000-irq", chip);
		if (ret != 0) {
			dev_err(dev, "Failed to request IRQ: %d\n",
				chip->chip_irq);
			return ret;
		}
	} else {
		dev_warn(dev, "No IRQ configured\n");
	}

	return ret;
}

static int slg51000_i2c_remove(struct i2c_client *client)
{
	struct slg51000 *chip = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0;

	if (chip->chip_cs_pin > 0) {
		desc = gpio_to_desc(chip->chip_cs_pin);
		ret = gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
	}

	return ret;
}

static const struct i2c_device_id slg51000_i2c_id[] = {
	{"slg51000", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, slg51000_i2c_id);

static struct i2c_driver slg51000_regulator_driver = {
	.driver = {
		.name = "slg51000-regulator",
	},
	.probe = slg51000_i2c_probe,
	.remove = slg51000_i2c_remove,
	.id_table = slg51000_i2c_id,
};

module_i2c_driver(slg51000_regulator_driver);

MODULE_AUTHOR("Eric Jeong <eric.jeong.opensource@diasemi.com>");
MODULE_DESCRIPTION("SLG51000 regulator driver");
MODULE_LICENSE("GPL");
