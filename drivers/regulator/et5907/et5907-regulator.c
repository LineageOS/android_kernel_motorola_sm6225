// SPDX-License-Identifier: GPL-2.0+
/*
 * et5907, Multi-Output Regulators
 * Copyright (C) 2019  Motorola Mobility LLC,
 *
 * Author: ChengLong, Motorola Mobility LLC,
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
#include "et5907-regulator.h"

enum slg51000_regulators {
	ET5907_REGULATOR_LDO1 = 0,
	ET5907_REGULATOR_LDO2,
	ET5907_REGULATOR_LDO3,
	ET5907_REGULATOR_LDO4,
	ET5907_REGULATOR_LDO5,
	ET5907_REGULATOR_LDO6,
	ET5907_REGULATOR_LDO7,
	ET5907_MAX_REGULATORS,
};

struct et5907 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc *rdesc[ET5907_MAX_REGULATORS];
	struct regulator_dev *rdev[ET5907_MAX_REGULATORS];
	// int chip_irq;
	int chip_cs_pin;
};

struct et5907_evt_sta {
	unsigned int sreg;
};

static const struct et5907_evt_sta et5907_status_reg = { ET5907_LDO_EN };

static const struct regmap_range et5907_writeable_ranges[] = {
	regmap_reg_range(ET5907_CURRENT_LIMITSEL, ET5907_SEQ_STATUS),
		/* Do not let useless register writeable */
	// regmap_reg_range(ET5907_CURRENT_LIMITSEL, ET5907_CURRENT_LIMITSEL),
	// regmap_reg_range(ET5907_LDO1_VOUT, ET5907_LDO7_VOUT),
	// regmap_reg_range(ET5907_LDO_EN, ET5907_LDO_EN),
};

static const struct regmap_range et5907_readable_ranges[] = {
	regmap_reg_range(ET5907_CHIP_REV, ET5907_SEQ_STATUS),
};

static const struct regmap_range et5907_volatile_ranges[] = {
	regmap_reg_range(ET5907_CURRENT_LIMITSEL, ET5907_SEQ_STATUS),
};

static const struct regmap_access_table et5907_writeable_table = {
	.yes_ranges	= et5907_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(et5907_writeable_ranges),
};

static const struct regmap_access_table et5907_readable_table = {
	.yes_ranges	= et5907_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(et5907_readable_ranges),
};

static const struct regmap_access_table et5907_volatile_table = {
	.yes_ranges	= et5907_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(et5907_volatile_ranges),
};

static const struct regmap_config et5907_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x0F,
	.wr_table = &et5907_writeable_table,
	.rd_table = &et5907_readable_table,
	.volatile_table = &et5907_volatile_table,
};

static int et5907_get_error_flags(struct regulator_dev *rdev, unsigned int *flags)
{
	struct et5907 *chip = rdev_get_drvdata(rdev);
	uint8_t reg_dump[ET5907_REG_NUM];
	uint8_t reg_idx;
	unsigned int val = 0;

	dev_err(chip->dev, "************ start dump et5907 register ************\n");
	dev_err(chip->dev, "register 0x00:      chip version\n");
	dev_err(chip->dev, "register 0x01:      LDO CL\n");
	dev_err(chip->dev, "register 0x03~0x09: LDO1~LDO7 OUT Voltage\n");
	dev_err(chip->dev, "register 0x0e:      Bit[6:0] LDO7~LDO1 EN\n");

	for (reg_idx = 0; reg_idx < ET5907_REG_NUM; reg_idx++) {
		regmap_read(chip->regmap, reg_idx, &val);
		reg_dump[reg_idx] = val;
		dev_err(chip->dev, "Reg[0x%02x] = 0x%x", reg_idx, reg_dump[reg_idx]);
	}
	dev_err(chip->dev, "************ end dump et5907 register ************\n");

	if (flags != NULL) {
		*flags = 0;
	}

	return 0;
}

static int et5907_get_status(struct regulator_dev *rdev)
{
	struct et5907 *chip = rdev_get_drvdata(rdev);
	int ret, id = rdev_get_id(rdev);
	unsigned int status = 0;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read enable register(%d)\n",
			ret);
		return ret;
	}

	if (!ret)
		return REGULATOR_STATUS_OFF;

	et5907_get_error_flags(rdev, NULL);

	ret = regmap_read(chip->regmap, et5907_status_reg.sreg, &status);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read status register(%d)\n",
			ret);
		return ret;
	}
	if (status & (0x01ul << id)) {
		return REGULATOR_STATUS_ON;
	} else {
		return REGULATOR_STATUS_OFF;
	}
}

static const struct regulator_ops et5907_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_status = et5907_get_status,
	.get_error_flags = et5907_get_error_flags,
};

static int et5907_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	int ena_gpio;

	ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (gpio_is_valid(ena_gpio))
		config->ena_gpiod = gpio_to_desc(ena_gpio);

	return 0;
}

#define ET5907_REGL_DESC(_id, _name, _s_name, _min, _step)       \
	[ET5907_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,                            \
		.id = ET5907_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = et5907_of_parse_cb,               \
		.ops = &et5907_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = ET5907_VSEL_MASK,               \
		.vsel_reg = ET5907_##_id##_VSEL,                 \
		.enable_reg = ET5907_LDO_EN,       \
		.enable_mask = BIT(ET5907_REGULATOR_##_id),     \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc et5907_regls_desc[ET5907_MAX_REGULATORS] = {
	ET5907_REGL_DESC(LDO1, ldo1, "vin1", 600000,  6000),
	ET5907_REGL_DESC(LDO2, ldo2, "vin1", 600000,  6000),
	ET5907_REGL_DESC(LDO3, ldo3, "vin2", 1200000, 10000),
	ET5907_REGL_DESC(LDO4, ldo4, "vin2", 1200000, 10000),
	ET5907_REGL_DESC(LDO5, ldo5, "vin2", 1200000, 10000),
	ET5907_REGL_DESC(LDO6, ldo6, "vin2", 1200000, 10000),
	ET5907_REGL_DESC(LDO7, ldo7, "vin2", 1200000, 10000),
};

static int et5907_regulator_init(struct et5907 *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	u8 vsel_range[1];
	int id, ret = 0;

	const unsigned int ldo_regs[ET5907_MAX_REGULATORS] = {
		ET5907_LDO1_VOUT,
		ET5907_LDO2_VOUT,
		ET5907_LDO3_VOUT,
		ET5907_LDO4_VOUT,
		ET5907_LDO5_VOUT,
		ET5907_LDO6_VOUT,
		ET5907_LDO7_VOUT,
	};

	const unsigned int initial_voltage[ET5907_MAX_REGULATORS] = {
		0x4B,//LDO1 1.05V
		0x64,//LDO2 1.2V
		0xa0,//LDO3 2.8V
		0xa0,//LDO4 2.8V
		0xa0,//LDO5 2.8V
		0xa0,//LDO6 2.8V
		0x3C,//LDO7 1.8V
	};

	/*Disable all ldo output by default*/
	ret = regmap_write(chip->regmap, ET5907_LDO_EN, 0);
	if (ret < 0) {
		dev_err(chip->dev,
			"Disable all LDO output failed!!!\n");
		return ret;
	}

	for (id = 0; id < ET5907_MAX_REGULATORS; id++) {
		chip->rdesc[id] = &et5907_regls_desc[id];
		rdesc = chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, ldo_regs[id],
				       vsel_range, 1);
		pr_err("et5907_regulator_init: LDO%d, default value:0x%x", (id+1), vsel_range[0]);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the ldo register\n");
			return ret;
		}

		ret = regmap_write(chip->regmap, ldo_regs[id], initial_voltage[id]);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to write inital voltage register\n");
			return ret;
		}
		pr_err("et5907_regulator_init: LDO%d, initial value:0x%x", (id+1), initial_voltage[id]);
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

static int et5907_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct et5907 *chip;
	int error, cs_gpio, ret;
	int chip_id = 0;

	chip = devm_kzalloc(dev, sizeof(struct et5907), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "et5907_i2c_probe Memory error...\n");
		return -ENOMEM;
	}
	dev_info(chip->dev, "et5907_i2c_probe Enter...\n");

	cs_gpio = of_get_named_gpio(dev->of_node, "semi,cs-gpios", 0);
	if (cs_gpio > 0) {
		if (!gpio_is_valid(cs_gpio)) {
			dev_err(dev, "Invalid chip select pin\n");
			return -EPERM;
		}

		ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_HIGH,
					    "et5907_cs_pin");
		if (ret) {
			dev_err(dev, "GPIO(%d) request failed(%d)\n",
				cs_gpio, ret);
			return ret;
		}

		chip->chip_cs_pin = cs_gpio;
	}

	dev_info(chip->dev, "et5907_i2c_probe cs_gpio:%d...\n", cs_gpio);
	mdelay(10);

	i2c_set_clientdata(client, chip);
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &et5907_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	ret = regmap_bulk_read(chip->regmap, ET5907_CHIP_ID, &chip_id, 1);
	if (ret < 0) {
		dev_err(dev, "I2C read failed: %d\n", ret);
		return ret;
	}
	dev_info(chip->dev, "et5907 chip id: %02x\n", chip_id);

	ret = et5907_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}
	dev_info(chip->dev, "et5907_i2c_probe Exit...\n");

	return ret;
}

static int et5907_i2c_remove(struct i2c_client *client)
{
	struct et5907 *chip = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0;

	if (chip->chip_cs_pin > 0) {
		desc = gpio_to_desc(chip->chip_cs_pin);
		ret = gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
	}

	return ret;
}

static const struct i2c_device_id et5907_i2c_id[] = {
	{"et5907", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, et5907_i2c_id);

static struct i2c_driver et5907_regulator_driver = {
	.driver = {
		.name = "et5907-regulator",
	},
	.probe = et5907_i2c_probe,
	.remove = et5907_i2c_remove,
	.id_table = et5907_i2c_id,
};

module_i2c_driver(et5907_regulator_driver);

MODULE_AUTHOR("ChengLong <chengl1@motorola.com>");
MODULE_DESCRIPTION("et5907 regulator driver");
MODULE_LICENSE("GPL");

