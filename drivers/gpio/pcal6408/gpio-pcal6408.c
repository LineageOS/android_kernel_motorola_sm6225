/*
 * Copyright (c) 2020 Motorola Mobility, LLC.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/of_gpio.h>

#define PCAL6408_DBG(msg, ...) \
	pr_debug("pcal6048: [%s] "msg, __func__, ##__VA_ARGS__)
#define PCAL6408_ERR(msg, ...) \
	pr_err("pcal6048: [%s] "msg, __func__, ##__VA_ARGS__)
#define PCAL6408_LOG(msg, ...) \
	pr_info("pcal6048: [%s] "msg, __func__, ##__VA_ARGS__)

#define PCAL6408_ADDR_INPUT		0x00
#define PCAL6408_ADDR_OUTPUT	0x01
#define PCAL6408_ADDR_CONFIG	0x03
#define PCAL6408_ADDR_PULL_EN	0x43
#define PCAL6408_ADDR_PULL_CFG	0x44

#define TCA6418_ADDR_INPUT		0x14
#define TCA6418_ADDR_OUTPUT		0x17
#define TCA6418_ADDR_CONFIG		0x23
#define TCA6418_ADDR_PULL_CFG	0x2C

#define BIAS_PULL_UP	0
#define BIAS_PULL_DOWN	1
#define BIAS_NO_PULL	2

struct pcal6048_dev;

struct pcal6048_config {
	char *name;
	int nr_gpio;
	int dir_out_val;
	int direction;
	int output;
	int input;
	int (*get_bit)(int);
	int (*set_pulldown)(int, struct pcal6048_dev *);
	int (*set_pullup)(int, struct pcal6048_dev *);
	int (*set_nopull)(int, struct pcal6048_dev *);
	int (*get_pull)(int, struct pcal6048_dev *);
};

struct pcal6048_dev {
	const char			*name;
	struct i2c_client	*client;
	struct device		*dev;
	struct regmap		*regmap;
	struct gpio_chip	gpio_chip;
	const struct pcal6048_config *conf;
	struct pinctrl_desc ctrldesc;
	struct pinctrl_dev *pctl_dev;
	struct pinctrl_gpio_range grange;
	int	reset_gpio;
};

static int pcal6048_set_reg(struct pcal6048_dev *chip, int gpio, int reg);
static int pcal6048_clear_reg(struct pcal6048_dev *chip, int gpio, int reg);
static int pcal6048_read_reg(struct pcal6048_dev *chip, int gpio, int reg);

static int pcal6408_get_bit(int gpio)
{
	return gpio;
}

static int pcal6408_set_pulldown(int gpio, struct pcal6048_dev *chip)
{
	if (pcal6048_clear_reg(chip, gpio, PCAL6408_ADDR_PULL_CFG)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	if (pcal6048_set_reg(chip, gpio, PCAL6408_ADDR_PULL_EN)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	return 0;
}

static int pcal6408_set_pullup(int gpio, struct pcal6048_dev *chip)
{
	if (pcal6048_set_reg(chip, gpio, PCAL6408_ADDR_PULL_CFG)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	if (pcal6048_set_reg(chip, gpio, PCAL6408_ADDR_PULL_EN)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	return 0;
}

static int pcal6408_set_nopull(int gpio, struct pcal6048_dev *chip)
{
	if (pcal6048_clear_reg(chip, gpio, PCAL6408_ADDR_PULL_EN)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	return 0;
}

static int pcal6408_get_pull(int gpio, struct pcal6048_dev *chip)
{
	if (!pcal6048_read_reg(chip, gpio, PCAL6408_ADDR_PULL_EN))
		return BIAS_NO_PULL;
	else if (pcal6048_read_reg(chip, gpio, PCAL6408_ADDR_PULL_CFG))
		return BIAS_PULL_UP;
	else
		return BIAS_PULL_DOWN;
}

static const struct pcal6048_config pcal6408a_conf = {
	.name = "pcal6408a_conf",
	.nr_gpio = 8,
	.set_pulldown = pcal6408_set_pulldown,
	.set_pullup = pcal6408_set_pullup,
	.set_nopull = pcal6408_set_nopull,
	.get_pull = pcal6408_get_pull,
	.get_bit = pcal6408_get_bit,
	.dir_out_val = 0, /* Set 0 to direction for output */
	.direction = PCAL6408_ADDR_CONFIG,
	.output = PCAL6408_ADDR_OUTPUT,
	.input = PCAL6408_ADDR_INPUT,
};

static int tca6418_get_bit(int gpio)
{
	/* Bitmapping is strange, gpio7 is bit 0 of first addr,
	* but gpio8 is bit 0 of second addr, gpio 16 is bit 0
	* of third addr
	*/
	if (gpio <= 7)
		return (7 - gpio);
	else if (gpio > 7 && gpio <= 15)
		return gpio - 8;
	else
		return gpio - 16;
}

static int tca6418_set_pulldown(int gpio, struct pcal6048_dev *chip)
{
	if (pcal6048_clear_reg(chip, gpio, TCA6418_ADDR_PULL_CFG)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	return 0;
}

static int tca6418_set_nopull(int gpio, struct pcal6048_dev *chip)
{
	if (pcal6048_set_reg(chip, gpio, TCA6418_ADDR_PULL_CFG)) {
		PCAL6408_ERR("Failed to set pulldown for gpio %d\n", gpio);
		return 1;
	}

	return 0;
}

static int tca6418_get_pull(int gpio, struct pcal6048_dev *chip)
{
	if (!pcal6048_read_reg(chip, gpio, TCA6418_ADDR_PULL_CFG))
		return BIAS_PULL_DOWN;
	else
		return BIAS_NO_PULL;
}

static const struct pcal6048_config tca6418e_conf = {
	.name = "tca6418e_conf",
	.nr_gpio = 18,
	.set_pulldown = tca6418_set_pulldown,
	.set_pullup = NULL,
	.set_nopull = tca6418_set_nopull,
	.get_pull = tca6418_get_pull,
	.get_bit = tca6418_get_bit,
	.dir_out_val = 1, /* Set 1 to direction for output */
	.direction = TCA6418_ADDR_CONFIG,
	.output = TCA6418_ADDR_OUTPUT,
	.input = TCA6418_ADDR_INPUT,
};

static int pcal6048_read_reg(struct pcal6048_dev *chip, int gpio, int reg)
{
	int bit_num = chip->conf->get_bit(gpio);
	uint8_t bit = BIT(bit_num);
	int reg_off = (gpio / 8);
	int val;
	int ret = -1;

	PCAL6408_DBG("%s (gpio %d), read bit %d from 0x%x\n",
		chip->conf->name, gpio, bit_num, reg + reg_off);

	ret = regmap_read(chip->regmap, reg + reg_off, &val);
	if (ret < 0)
		return 0;

	return !!(val & bit);
}

static int pcal6048_set_reg(struct pcal6048_dev *chip, int gpio, int reg)
{
	int bit_num = chip->conf->get_bit(gpio);
	uint8_t bit = BIT(bit_num);
	int reg_off = (gpio / 8);

	PCAL6408_DBG("%s (gpio %d), set bit %d in 0x%x\n",
		chip->conf->name, gpio, bit_num, reg + reg_off);

	return regmap_write_bits(chip->regmap, reg + reg_off, bit, bit);
}

static int pcal6048_clear_reg(struct pcal6048_dev *chip, int gpio, int reg)
{
	int bit_num = chip->conf->get_bit(gpio);
	uint8_t bit = BIT(bit_num);
	int reg_off = (gpio / 8);

	PCAL6408_DBG("%s (gpio %d), clear bit %d in 0x%x\n",
		chip->conf->name, gpio, bit_num, reg + reg_off);

	return regmap_write_bits(chip->regmap, reg + reg_off, bit, 0);
}

static int pcal6048_direction_in(struct gpio_chip *gc, unsigned offset)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);
	int ret;

	if (!chip->conf->dir_out_val)
		ret = pcal6048_set_reg(chip, offset, chip->conf->direction);
	else
		ret = pcal6048_clear_reg(chip, offset, chip->conf->direction);

	return ret;
}

static void pcal6048_set_gpio(struct gpio_chip *gc, unsigned offset, int value)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);

	if (value)
		pcal6048_set_reg(chip, offset, chip->conf->output);
	else
		pcal6048_clear_reg(chip, offset, chip->conf->output);
}

static int pcal6048_direction_out(struct gpio_chip *gc, unsigned offset, int value)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);
	int ret;

	ret = pcal6048_set_reg(chip, offset, chip->conf->output);
	if (ret)
		return ret;

	if (chip->conf->dir_out_val)
		ret = pcal6048_set_reg(chip, offset, chip->conf->direction);
	else
		ret = pcal6048_clear_reg(chip, offset, chip->conf->direction);

	pcal6048_set_gpio(gc, offset, value);

	return ret;
}

/* 0 is out, 1 is in */
static int pcal6048_get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);
	int val = pcal6048_read_reg(chip, offset, chip->conf->direction);

	if (val == chip->conf->dir_out_val)
		return 0;
	else;
		return 1;
}

static int pcal6048_get_gpio(struct gpio_chip *gc, unsigned offset)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);

	/* TODO do I need to read twice to make sure it is cleared? (TCA6418e pg 12) */
	return pcal6048_read_reg(chip, offset, chip->conf->input);
}

static int pcal6048_set_config(struct gpio_chip *gc, unsigned int offset, unsigned long config)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);
	unsigned arg = pinconf_to_config_argument(config);
	unsigned param = pinconf_to_config_param(config);

	PCAL6408_DBG("Set config for gpio %d!\n", offset);

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		if (chip->conf->set_pullup)
			return chip->conf->set_pullup(offset, chip);
		else
			return -ENOTSUPP;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (chip->conf->set_pulldown)
			return chip->conf->set_pulldown(offset, chip);
		else
			return -ENOTSUPP;
	case PIN_CONFIG_BIAS_DISABLE:
		if (chip->conf->set_nopull)
			return chip->conf->set_nopull(offset, chip);
		else
			return -ENOTSUPP;
	case PIN_CONFIG_INPUT_ENABLE:
		return pcal6048_direction_in(gc, offset);
	case PIN_CONFIG_OUTPUT_ENABLE:
		return pcal6048_direction_out(gc, offset, 1);
	case PIN_CONFIG_OUTPUT:
		return pcal6048_direction_out(gc, offset, !!arg);
	default:
		return -ENOTSUPP;
	}
}

static void pcal6048_dbg_show(struct seq_file *s, struct gpio_chip *gc)
{
	struct pcal6048_dev *chip = gpiochip_get_data(gc);
	unsigned gpio = gc->base;
	unsigned i;
	int dir = 0;
	int pull = 0;

	for (i = 0; i < gc->ngpio; i++, gpio++) {
		seq_printf(s, " gpio%d:", i);

		dir = pcal6048_get_direction(gc, i);
		/* Input is 1 */
		if (dir)
			seq_printf(s, " in %d", pcal6048_get_gpio(gc, i));
		else
			seq_printf(s, " out (set to %d)",
				pcal6048_read_reg(chip, i, chip->conf->output));

		if (chip->conf->get_pull) {
			pull = chip->conf->get_pull(i, chip);
			switch(pull) {
			case BIAS_NO_PULL:
				seq_printf(s, " bias-no-pull");
				break;
			case BIAS_PULL_DOWN:
				seq_printf(s, " bias-pull-down");
				break;
			case BIAS_PULL_UP:
				seq_printf(s, " bias-pull-up");
				break;
			default:
				break;
			}
		}

		seq_puts(s, "\n");
	}
}

static void pcal6048_setup_gpio_chip(struct pcal6048_dev *chip)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->label = chip->conf->name;
	gc->base = -1;
	gc->ngpio = chip->conf->nr_gpio;
	gc->parent = chip->dev;
	gc->owner = THIS_MODULE;
	gc->direction_input	= pcal6048_direction_in;
	gc->direction_output = pcal6048_direction_out;
	gc->get_direction = pcal6048_get_direction;
	gc->get	= pcal6048_get_gpio;
	gc->set	= pcal6048_set_gpio;
	gc->set_config = pcal6048_set_config;
	gc->dbg_show = pcal6048_dbg_show;
	gc->can_sleep = true;
}

static int pcal6048_pinconf_get(struct pinctrl_dev *pctldev,
			unsigned pin, unsigned long *config)
{
	return -ENOTSUPP;
}

static int pcal6048_pinconf_set(struct pinctrl_dev *pctldev,
			unsigned int pin, unsigned long *configs,
			unsigned int num_configs)
{
	struct pcal6048_dev *chip = pinctrl_dev_get_drvdata(pctldev);
	int i;

	for (i=0; i < num_configs; i++) {
		chip->gpio_chip.set_config(&chip->gpio_chip, pin, configs[i]);
	}

	return 0;
}

static int pcal6048_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct pcal6048_dev *chip = pinctrl_dev_get_drvdata(pctldev);

	return chip->conf->nr_gpio;
}

static const char *pcal6048_get_group_name(struct pinctrl_dev *pctldev,
		unsigned selector)
{
	struct pcal6048_dev *chip = pinctrl_dev_get_drvdata(pctldev);

	return chip->conf->name;
}

static const struct pinctrl_ops pcal6048_pctl_ops = {
	.get_groups_count = pcal6048_get_groups_count,
	.get_group_name = pcal6048_get_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
};

static const struct pinconf_ops pcal6048_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = pcal6048_pinconf_get,
	.pin_config_set = pcal6048_pinconf_set,
};

static int pcal6048_register_pinctrl(struct pcal6048_dev *chip)
{
	struct pinctrl_desc *ctrldesc = &chip->ctrldesc;
	struct pinctrl_pin_desc *pindesc, *pdesc;
	int pin;

	ctrldesc->name = chip->conf->name;
	ctrldesc->owner = THIS_MODULE;
	ctrldesc->confops = &pcal6048_pinconf_ops;
	ctrldesc->pctlops = &pcal6048_pctl_ops;

	pindesc = devm_kcalloc(chip->dev,
			       chip->conf->nr_gpio, sizeof(*pindesc),
			       GFP_KERNEL);

	ctrldesc->pins = pindesc;
	ctrldesc->npins = chip->conf->nr_gpio;

	pdesc = pindesc;
	for (pin = 0; pin < chip->conf->nr_gpio; pin++) {
		pdesc->number = pin;
		pdesc->name = kasprintf(GFP_KERNEL, "gpio%d", pin);
		pdesc++;
	}

	chip->pctl_dev = devm_pinctrl_register(chip->dev, ctrldesc, chip);

	if (IS_ERR(chip->pctl_dev)) {
		PCAL6408_ERR("Failed to register pinctrl\n");
		return PTR_ERR(chip->pctl_dev);
	}

	chip->grange.base = chip->gpio_chip.base;
	chip->grange.npins = chip->conf->nr_gpio;
	chip->grange.gc = &chip->gpio_chip;

	pinctrl_add_gpio_range(chip->pctl_dev, &chip->grange);

	return 0;
}

static const struct regmap_config pcal6048_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static const struct of_device_id pcal6048_match_table[] = {
	{ .compatible = "ti,tca6418e", .data = &tca6418e_conf },
	{ .compatible = "nxp,pcal6408a", .data = &pcal6408a_conf },
	{ },
};

static int pcal6048_config_reset(struct pcal6048_dev *chip)
{
	int err = 0;

	chip->reset_gpio = of_get_gpio(chip->dev->of_node, 0);
	if (gpio_is_valid(chip->reset_gpio)) {
		err = devm_gpio_request(chip->dev, chip->reset_gpio, "pcal6048_reset");
		if (err) {
			PCAL6408_ERR("Reset gpio request failed\n");
			return err;
		} else {
			/* Active low, so set it high  */
			err = gpio_direction_output(chip->reset_gpio, 1);
			if (err) {
				PCAL6408_ERR("Reset gpio set failed\n");
				return err;
			}
		}
	} else
		PCAL6408_LOG("No reset gpio set in dtb\n");

	return 0;
}

static int pcal6048_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc = 0;
	struct pcal6048_dev *chip;
	const struct of_device_id *match;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;
	chip->name = "pcal6048";

	chip->regmap = regmap_init_i2c(client, &pcal6048_regmap_config);
	if (IS_ERR(chip->regmap)) {
		PCAL6408_ERR("Couldn't initialize register regmap rc = %ld\n",
				PTR_ERR(chip->regmap));
		rc = PTR_ERR(chip->regmap);
		goto free_mem;
	}

	match = of_match_device(pcal6048_match_table, chip->dev);
	if (!match || !match->data) {
		PCAL6408_ERR("Missing config data!\n");
		goto free_mem;
	}

	chip->conf = match->data;
	PCAL6408_DBG("Using config %s\n", chip->conf->name);

	i2c_set_clientdata(client, chip);
	dev_set_drvdata(chip->dev, chip);

	pcal6048_setup_gpio_chip(chip);
	pcal6048_config_reset(chip);

	rc = devm_gpiochip_add_data(chip->dev, &chip->gpio_chip, chip);
	if (rc) {
		PCAL6408_ERR("Couldn't add gpio chip rc=%d\n", rc);
		goto free_mem;
	}

	pcal6048_register_pinctrl(chip);

	return 0;

free_mem:
	devm_kfree(chip->dev, chip);
	return rc;
}

static int pcal6048_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver pcal6048_driver = {
	.driver = {
		.name = "pcal6048",
		.owner = THIS_MODULE,
		.of_match_table = pcal6048_match_table,
	},
	.probe = pcal6048_probe,
	.remove = pcal6048_remove,
};

module_i2c_driver(pcal6048_driver);

MODULE_DESCRIPTION("pcal6048");
MODULE_LICENSE("GPL v2");
