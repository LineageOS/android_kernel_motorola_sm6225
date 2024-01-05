/*
 * TI LMU (Lighting Management Unit) Core Driver
 *
 * Copyright 2017 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include "ti_lm3697.h"
#include "ti_lm3697_regulator.h"
#include "ti-lmu.h"

struct ti_lmu_data {
	struct mfd_cell *cells;
	int num_cells;
	unsigned int max_register;
};

#define LM3697_I2C_ADDR    0x36

static int ti_lmu_enable_hw(struct ti_lmu *lmu, enum ti_lmu_id id)
{
	int ret;

	if (gpio_is_valid(lmu->en_gpio)) {
		ret = devm_gpio_request_one(lmu->dev, lmu->en_gpio,
					    GPIOF_OUT_INIT_HIGH, "lmu_hwen");
		if (ret) {
			dev_err(lmu->dev, "Can not request enable GPIO: %d\n",
				ret);
			return ret;
		}
	}

	/* Delay about 1ms after HW enable pin control */
	usleep_range(1000, 1500);

	return 0;
}

static void ti_lmu_disable_hw(struct ti_lmu *lmu)
{
	if (gpio_is_valid(lmu->en_gpio))
		gpio_set_value(lmu->en_gpio, 0);
}


static struct mfd_cell lm3697_devices[] = {
	{
		.name          = "ti-lmu-backlight",
		.id            = LM3697,
		.of_compatible = "ti,lm3697-backlight",
	},
	/* Monitoring driver for open/short circuit detection */
	{
		.name          = "ti-lmu-fault-monitor",
		.id            = LM3697,
		.of_compatible = "ti,lm3697-fault-monitor",
	},
};

#define TI_LMU_DATA(chip, max_reg)		\
static const struct ti_lmu_data chip##_data =	\
{						\
	.cells = chip##_devices,		\
	.num_cells = ARRAY_SIZE(chip##_devices),\
	.max_register = max_reg,		\
}						\

TI_LMU_DATA(lm3697, LM3697_MAX_REG);

static const struct of_device_id ti_lmu_of_match[] = {
	{ .compatible = "ti,lm3697", .data = &lm3697_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ti_lmu_of_match);

static int ti_lmu_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct device *dev = &cl->dev;
	const struct of_device_id *match;
	const struct ti_lmu_data *data;
	struct regmap_config regmap_cfg;
	struct ti_lmu *lmu;
	int ret;

	if (cl->addr != LM3697_I2C_ADDR)
		cl->addr = LM3697_I2C_ADDR;

	match = of_match_device(ti_lmu_of_match, dev);
	if (!match)
		return -ENODEV;
	/*
	 * Get device specific data from of_match table.
	 * This data is defined by using TI_LMU_DATA() macro.
	 */
	data = (struct ti_lmu_data *)match->data;

	lmu = devm_kzalloc(dev, sizeof(*lmu), GFP_KERNEL);
	if (!lmu)
		return -ENOMEM;

	lmu->dev = &cl->dev;

	/* Setup regmap */
	memset(&regmap_cfg, 0, sizeof(struct regmap_config));
	regmap_cfg.reg_bits = 8;
	regmap_cfg.val_bits = 8;
	regmap_cfg.name = id->name;
	regmap_cfg.max_register = data->max_register;

	lmu->regmap = devm_regmap_init_i2c(cl, &regmap_cfg);
	if (IS_ERR(lmu->regmap))
		return PTR_ERR(lmu->regmap);

	/* HW enable pin control and additional power up sequence if required */
	lmu->en_gpio = of_get_named_gpio(dev->of_node, "enable-gpios", 0);
	ret = ti_lmu_enable_hw(lmu, id->driver_data);
	if (ret)
		return ret;

	/*
	 * Fault circuit(open/short) can be detected by ti-lmu-fault-monitor.
	 * After fault detection is done, some devices should re-initialize
	 * configuration. The notifier enables such kind of handling.
	 */
	BLOCKING_INIT_NOTIFIER_HEAD(&lmu->notifier);

	i2c_set_clientdata(cl, lmu);

	mfd_add_devices(lmu->dev, 0, data->cells,
			       data->num_cells, NULL, 0, NULL);

	ti_lmu_backlight_device_init();

	return ret;
}

static int ti_lmu_remove(struct i2c_client *cl)
{
	struct ti_lmu *lmu = i2c_get_clientdata(cl);

	ti_lmu_disable_hw(lmu);
	mfd_remove_devices(lmu->dev);
	return 0;
}

static const struct i2c_device_id ti_lmu_ids[] = {
	{ "lm3697", LM3697 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ti_lmu_ids);

static struct i2c_driver ti_lmu_driver = {
	.probe = ti_lmu_probe,
	.remove = ti_lmu_remove,
	.driver = {
		.name = "ti-lmu",
		.of_match_table = ti_lmu_of_match,
	},
	.id_table = ti_lmu_ids,
};

module_i2c_driver(ti_lmu_driver);

MODULE_DESCRIPTION("TI LMU MFD Core Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
