// SPDX-License-Identifier: GPL-2.0+
/*
 * wl2866d, Multi-Output Regulators
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
#include "wl2866d-regulator.h"

enum slg51000_regulators {
	WL2866D_REGULATOR_LDO1 = 0,
	WL2866D_REGULATOR_LDO2,
	WL2866D_REGULATOR_LDO3,
	WL2866D_REGULATOR_LDO4,
	WL2866D_MAX_REGULATORS,
};

struct wl2866d {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc *rdesc[WL2866D_MAX_REGULATORS];
	struct regulator_dev *rdev[WL2866D_MAX_REGULATORS];
	int chip_cs_pin;
	int init_value;
};

struct wl2866d_evt_sta {
	unsigned int sreg;
};

static const struct wl2866d_evt_sta wl2866d_status_reg = { WL2866D_LDO_EN };

static const struct regmap_range wl2866d_writeable_ranges[] = {
      /* Do not let useless register writeable */
	regmap_reg_range(WL2866D_CURRENT_LIMITSEL, WL2866D_SEQ_STATUS),
};

static const struct regmap_range wl2866d_readable_ranges[] = {
	regmap_reg_range(WL2866D_CHIP_REV, WL2866D_SEQ_STATUS),
};

static const struct regmap_range wl2866d_volatile_ranges[] = {
	regmap_reg_range(WL2866D_CURRENT_LIMITSEL, WL2866D_SEQ_STATUS),
};

static const struct regmap_access_table wl2866d_writeable_table = {
	.yes_ranges	= wl2866d_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(wl2866d_writeable_ranges),
};

static const struct regmap_access_table wl2866d_readable_table = {
	.yes_ranges	= wl2866d_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(wl2866d_readable_ranges),
};

static const struct regmap_access_table wl2866d_volatile_table = {
	.yes_ranges	= wl2866d_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(wl2866d_volatile_ranges),
};

static const struct regmap_config wl2866d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x0F,
	.wr_table = &wl2866d_writeable_table,
	.rd_table = &wl2866d_readable_table,
	.volatile_table = &wl2866d_volatile_table,
};

static int wl2866d_get_current_limit(struct regulator_dev *rdev)
{
	struct wl2866d *chip = rdev_get_drvdata(rdev);
	uint8_t reg_dump[WL2866D_REG_NUM];
	uint8_t reg_idx;
	unsigned int val = 0;

	dev_err(chip->dev, "************ start dump wl2866d register ************\n");
	dev_err(chip->dev, "register name =%s \n",rdev->desc->name);
	dev_err(chip->dev, "register 0x00:      chip version\n");
	dev_err(chip->dev, "register 0x01:      LDO CL\n");
	dev_err(chip->dev, "register 0x03~0x06: LDO1~LDO4 OUT Voltage\n");
	dev_err(chip->dev, "register 0x0e:      Bit[3:0] LDO4~LDO1 EN\n");

	for (reg_idx = 0; reg_idx < WL2866D_REG_NUM; reg_idx++) {
		regmap_read(chip->regmap, reg_idx, &val);
		reg_dump[reg_idx] = val;
		dev_err(chip->dev, "Reg[0x%02x] = 0x%x", reg_idx, reg_dump[reg_idx]);
	}
	dev_err(chip->dev, "************ end dump wl2866D  register ************\n");

	return 0;
}

static int wl2866d_get_status(struct regulator_dev * rdev)
{
	struct wl2866d *chip = rdev_get_drvdata(rdev);
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

	wl2866d_get_current_limit(rdev);

	ret = regmap_read(chip->regmap, wl2866d_status_reg.sreg, &status);
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

static const struct regulator_ops wl2866d_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_status = wl2866d_get_status,
	.get_current_limit = wl2866d_get_current_limit,
};

static int wl2866d_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	int ena_gpio;

	ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (gpio_is_valid(ena_gpio))
		config->ena_gpiod = gpio_to_desc(ena_gpio);

	return 0;
}

#define WL2866D_REGL_DESC(_id, _name, _s_name, _min, _step)       \
	[WL2866D_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,                            \
		.id = WL2866D_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = wl2866d_of_parse_cb,               \
		.ops = &wl2866d_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = WL2866D_VSEL_MASK,                   \
		.vsel_reg = WL2866D_##_id##_VSEL,                 \
		.enable_reg = WL2866D_LDO_EN,       \
		.enable_mask = BIT(WL2866D_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc wl2866d_regls_desc[WL2866D_MAX_REGULATORS] = {
	WL2866D_REGL_DESC(LDO1, ldo1, "vin1", 600000, 6000),
	WL2866D_REGL_DESC(LDO2, ldo2, "vin1", 600000, 6000),
	WL2866D_REGL_DESC(LDO3, ldo3, "vin2", 1200000, 12500),
	WL2866D_REGL_DESC(LDO4, ldo4, "vin2", 1200000, 12500),
};

static int wl2866d_regulator_init(struct wl2866d *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	u8 vsel_range[1];
	int id, ret = 0;
	const unsigned int ldo_regs[WL2866D_MAX_REGULATORS] = {
		WL2866D_LDO1_VOUT,
		WL2866D_LDO2_VOUT,
		WL2866D_LDO3_VOUT,
		WL2866D_LDO4_VOUT,
	};

	const unsigned int initial_voltage[WL2866D_MAX_REGULATORS] = {
		0x4b,//LDO1  DVDD 1.05V
		0x64,//LDO2 DVDD 1.2V
		0x80,//LDO3 AVDD 2.8V
		0x80,//LDO4 AVDD 2.8V
	};

	/*Disable all ldo output by default*/
	ret = regmap_write(chip->regmap, WL2866D_LDO_EN, chip->init_value);
	if (ret < 0) {
		dev_err(chip->dev,
			"Disable all LDO output failed!!!\n");
		return ret;
	}

	for (id = 0; id < WL2866D_MAX_REGULATORS; id++) {
		chip->rdesc[id] = &wl2866d_regls_desc[id];
		rdesc = chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, ldo_regs[id],
				       vsel_range, 1);
		pr_err("wl2866d_regulator_init: LDO%d, default value:0x%x", (id+1), vsel_range[0]);
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
		pr_err("wl2866d_regulator_init: LDO%d, initial value:0x%x", (id+1), initial_voltage[id]);

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

static int wl2866d_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct wl2866d *chip;
	int error, cs_gpio, ret, i, value;

	/* Set all register to initial value when probe driver to avoid register value was modified.
	*/
	const unsigned int initial_register[5][2] = {
		{WL2866D_CURRENT_LIMITSEL, 	0x40},
		{WL2866D_DISCHARGE_RESISTORS, 	0x00},
		{WL2866D_LDO1_LDO2_SEQ, 	0x00},
		{WL2866D_LDO3_LDO4_SEQ, 	0x00},
		{WL2866D_SEQ_STATUS, 		0x00},
	};
	chip = devm_kzalloc(dev, sizeof(struct wl2866d), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "wl2866d_i2c_probe Memory error...\n");
		return -ENOMEM;
	}

	dev_info(chip->dev, "wl2866d_i2c_probe Enter...\n");

	cs_gpio = of_get_named_gpio(dev->of_node, "semi,cs-gpios", 0);
	if (cs_gpio > 0) {
		if (!gpio_is_valid(cs_gpio)) {
			dev_err(dev, "Invalid chip select pin\n");
			return -EPERM;
		}

		ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_LOW,
					    "wl2866d_cs_pin");
		if (ret) {
			dev_err(dev, "GPIO(%d) request failed(%d)\n",
				cs_gpio, ret);
			return ret;
		}

		chip->chip_cs_pin = cs_gpio;
	}

	dev_info(chip->dev, "wl2866d_i2c_probe cs_gpio:%d...\n", cs_gpio);

	if (of_property_read_u32(dev->of_node, "semi,init-value", &value) < 0) {
		dev_info(chip->dev, "wl2866d_i2c_probe no init_value, use default 0x0\n");
		value = 0x0;
	}
	chip->init_value = value;
	dev_info(chip->dev, "wl2866d_i2c_probe init_value:%d...\n", value);

	mdelay(10);

	i2c_set_clientdata(client, chip);
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &wl2866d_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}


	for (i = 0; i < 5; i++) {
		ret = regmap_write(chip->regmap, initial_register[i][0], initial_register[i][1]);
		if (ret < 0) {
			dev_err(chip->dev,"Failed to write register: 0x%x, value: 0x%x \n",
				initial_register[i][0], initial_register[i][1]);
		}

		dev_err(chip->dev,"Success to write register: 0x%x, value: 0x%x \n",
			initial_register[i][0], initial_register[i][1]);
	}

	ret = wl2866d_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	wl2866d_get_current_limit(chip->rdev[0]);

	dev_info(chip->dev, "wl2866d_i2c_probe Exit...\n");

	return ret;
}

static int wl2866d_i2c_remove(struct i2c_client *client)
{
	struct wl2866d *chip = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0;

	if (chip->chip_cs_pin > 0) {
		desc = gpio_to_desc(chip->chip_cs_pin);
		ret = gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
	}

	return ret;
}

static const struct i2c_device_id wl2866d_i2c_id[] = {
	{"wl2866d", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wl2866d_i2c_id);

static struct i2c_driver wl2866d_regulator_driver = {
	.driver = {
		.name = "wl2866d-regulator",
	},
	.probe = wl2866d_i2c_probe,
	.remove = wl2866d_i2c_remove,
	.id_table = wl2866d_i2c_id,
};

module_i2c_driver(wl2866d_regulator_driver);

MODULE_AUTHOR("ChengLong <chengl1@motorola.com>");
MODULE_DESCRIPTION("WL2866D regulator driver");
MODULE_LICENSE("GPL");
