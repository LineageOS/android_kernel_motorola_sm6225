/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>

#define PMIC_REG_CONFIG 		0x01
#define PMIC_REG_VOUTFLOORSET		0x02
#define PMIC_REG_VOUTROOFSET 		0x03
#define PMIC_REG_ILIMSET 		0x04
#define PMIC_REG_STATUS 		0x05
#define PMIC_REG_E2PROMCTRL 		0xFF

#define VOUT_SPPORT_MIN 		0
#define VOUT_SPPORT_MAX		0x1F
#define VOUT_SPPORT_MID		0x0F
#define DEFAULT_VOUTL_VALUE 	0x06
#define DEFAULT_VOUTH_VALUE 	0x0A

#define PINCTRL_STATE_ACTIVE "tps61280a_active"
#define PINCTRL_STATE_SUSPEND "tps61280a_suspend"
#define RETRY_COUNT 3
struct tps61280a_platform_data {
	int gpio_en;
	int gpio_vsel;
	int gpio_rst;
	struct pinctrl *tps_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct i2c_client *i2c_client;
	uint8_t xbuf[4];
};

static struct tps61280a_platform_data *gpdata;

static int tps61280a_i2c_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		pr_err("%s: i2c_read_block retry over %d\n", __func__, toRetry);
		return -EIO;
	}

	return 0;
}

static int tps61280a_i2c_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		pr_err("%s: i2c_write_block retry over %d\n", __func__, toRetry);
		return -EIO;
	}

	return 0;
}

static unsigned char tps61280a_read_register(uint8_t adress)
{
	int ret = 0;
	unsigned char buffer;
	struct i2c_client *i2c =gpdata->i2c_client;

	ret =  tps61280a_i2c_read(i2c, adress, &buffer, 1, RETRY_COUNT);
	if (ret) {
		pr_err("%s failed\n", __func__);
	}
	return buffer;
}

static int tps61280a_write_register(uint8_t adress, uint8_t value)
{
	int ret = 0;
	unsigned char buffer[2];
	struct i2c_client *i2c =gpdata->i2c_client;

	buffer[0] = adress;
	buffer[1] = value;
	ret = tps61280a_i2c_write(i2c, buffer , 2, RETRY_COUNT);
	if (ret) {
		pr_err("%s failed\n", __func__);
	}
	return ret;
}

static int tps61280a_vout_setting(int vfloor, int vroof)
{
	int ret = 0;
	int fvout = (vfloor - 2850) / 50, rvout = (vroof - 2850) / 50;

	pr_info("%s vfloor = %d fvout = 0x%x\n", __func__, vfloor , fvout);
	pr_info("%s vroot = %d rvout = 0x%x\n", __func__, vroof, rvout);

	fvout = max(fvout, VOUT_SPPORT_MIN);
	fvout = min(fvout, VOUT_SPPORT_MAX);

	rvout = max(rvout, VOUT_SPPORT_MIN);
	rvout = min(rvout, VOUT_SPPORT_MAX);

	ret = tps61280a_write_register(PMIC_REG_VOUTFLOORSET, fvout);
	if (ret) {
		pr_err("%s FLOORSET failed\n", __func__);
	}
	ret = 0 ;
	ret = tps61280a_write_register(PMIC_REG_VOUTROOFSET, rvout);
	if (ret) {
		pr_err("%s ROOFSET failed\n", __func__);
	}

	return ret;
}

static void tps61280a_dump(void)
{
	int ret = 0 , i;
	for (i=1;i<=5;i++)
	{
		ret = tps61280a_read_register(i);
		pr_err("%s reg = 0x%x data = 0x%x\n", __func__,i ,ret);
	}
	ret = tps61280a_read_register(PMIC_REG_E2PROMCTRL);
	pr_info("%s reg = 0xff data = 0x%x\n", __func__,ret);

}


static int config_tps61280a_enable_setting(void)
{
	int ret = 0;

	ret = tps61280a_write_register(PMIC_REG_CONFIG, 0x09);
	if (ret) {
		pr_err("%s failed\n", __func__);
	}
	ret = 0;
	ret = tps61280a_write_register(PMIC_REG_ILIMSET, 0x0f);
	if (ret) {
		pr_err("%s failed\n", __func__);
	}

	return ret;
}

/*static int tps61280a_regulator_configure(struct i2c_client *client, struct tps61280a_platform_data *pdata, bool regulator_on)
{
	int ret = 0;

	if (regulator_on) {
		pdata->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			ret = PTR_ERR(pdata->vcc_i2c);
			pr_err("regulator get failed rc=%d\n", ret);
		}
		if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
			ret = regulator_set_voltage(pdata->vcc_i2c, 1800000, 1800000);
			if (ret) {
				pr_err("regulator set_vtg failed rc=%d\n", ret);
			}
		}

		ret = regulator_enable(pdata->vcc_i2c);
	} else {
		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
			regulator_set_voltage(pdata->vcc_i2c, 0, 1800000);

		regulator_put(pdata->vcc_i2c);
	}

	return ret;
}
*/
static int tps61280a_gpio_power_config(struct tps61280a_platform_data *pdata)
{
	int error = 0;

	if (!pdata) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	if (gpio_is_valid(pdata->gpio_en)) {
		error = gpio_request(pdata->gpio_en, "tps61280a_en_gpio");
		if (error) {
			pr_err("unable to request gpio = %d\n", pdata->gpio_en);
			return error;
		}

		error = gpio_direction_output(pdata->gpio_en, 1);
		if (error) {
			pr_err("unable to set direction for gpio = %d\n", pdata->gpio_en);
			return error;
		}
	}

	if (gpio_is_valid(pdata->gpio_vsel)) {
		error = gpio_request(pdata->gpio_vsel, "tps61280a_vsel_gpio");
		if (error) {
			pr_err("unable to request gpio = %d\n", pdata->gpio_vsel);
			return error;
		}

		error = gpio_direction_output(pdata->gpio_vsel, 1);
		if (error) {
			pr_err("unable to set direction for gpio = %d\n", pdata->gpio_vsel);
			return error;
		}
	}

	/* Add for RT4803 EN/nBYP timing */
	udelay(60);

	if (gpio_is_valid(pdata->gpio_rst)) {
		error = gpio_request(pdata->gpio_rst, "tps61280a_rst_gpio");
		if (error) {
			pr_err("unable to request gpio = %d\n", pdata->gpio_rst);
			return error;
		}

		error = gpio_direction_output(pdata->gpio_rst, 1);
		if (error) {
			pr_err("unable to set direction for gpio = %d\n", pdata->gpio_rst);
			return error;
		}
	}
	tps61280a_dump();
	//msleep(5);
	config_tps61280a_enable_setting();
	//msleep(5);
	//tps61280a_dump();

	return error;
}

static int tps61280a_parse_dt(struct device *dev, struct tps61280a_platform_data *pdata)
{
	if (!dev || !dev->of_node || !pdata) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	pdata->gpio_en= of_get_named_gpio(dev->of_node, "tps61280a,en-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_en)) {
		pr_err("DT:en-gpio value is not valid\n");
	}

	pdata->gpio_vsel = of_get_named_gpio(dev->of_node, "tps61280a,vsel-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_vsel)) {
		pr_err("DT:vsel-gpio value is not valid\n");
	}

	pdata->gpio_rst = of_get_named_gpio(dev->of_node, "tps61280a,rst-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_rst)) {
		pr_err("DT:rst-gpio value is not valid\n");
	}

	pr_info("DT: gpio_en=%d, gpio_vsel=%d, gpio_rst =%d\n",
		pdata->gpio_en, pdata->gpio_vsel, pdata->gpio_rst);
	return 0;
}

static int tps61280a_pinctrl_init(struct i2c_client *client, struct tps61280a_platform_data *pdata)
{
	if (!pdata) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	pdata->tps_pinctrl = devm_pinctrl_get(&(client->dev));
	if (!pdata->tps_pinctrl) {
		dev_err(&(client->dev), "Target does not use pinctrl\n");
		return -EINVAL;
	}

	pdata->pinctrl_state_active = pinctrl_lookup_state(pdata->tps_pinctrl, PINCTRL_STATE_ACTIVE);
	if (!pdata->pinctrl_state_active) {
		dev_err(&(client->dev), "Can not lookup %s pinstate\n", PINCTRL_STATE_ACTIVE);
		return -EINVAL;
	}

	pdata->pinctrl_state_suspend = pinctrl_lookup_state(pdata->tps_pinctrl, PINCTRL_STATE_SUSPEND);
	if (!pdata->pinctrl_state_suspend) {
		dev_err(&(client->dev), "Can not lookup %s pinstate\n", PINCTRL_STATE_SUSPEND);
		return -EINVAL;
	}

	pinctrl_select_state(pdata->tps_pinctrl, pdata->pinctrl_state_active);
	return 0;
}

static int tps61280a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int vout_floor = 3500, vout_roof = 4200;
	struct tps61280a_platform_data *pdata = NULL;

	if (!client) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	pdata = devm_kzalloc(&(client->dev), sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		pr_err("%s: allocate tps61280a_platform_data failed\n", __func__);
		goto err_alloc_platform_data;
	}

	ret = tps61280a_parse_dt(&(client->dev), pdata);
	if (ret) {
		ret = -EINVAL;
		pr_err("%s: failed to parse DT\n", __func__);
		goto err_dt_parse;
	}

	pdata->i2c_client = client;
	gpdata = pdata;
	ret = tps61280a_pinctrl_init(client, pdata);
	if (ret)
	{
		pr_err("%s: tps61280a_pinctrl_init failed\n", __func__);
		goto err_pinctrl_init;
	}

	//tps61280a_regulator_configure(client, pdata, true);

	ret = tps61280a_gpio_power_config(pdata);
	if (ret)
	{
		pr_err("%s: tps61280a_gpio_power_config failed\n", __func__);
		goto err_gpio_power;
	}

	ret = tps61280a_vout_setting(vout_floor, vout_roof);
	if (ret)
	{
		pr_err("%s:  failed\n", __func__);
		goto err_config_init;
	}

	tps61280a_dump();
	pr_info("%s end\n", __func__);
	return 0;

err_config_init:
err_gpio_power:
err_pinctrl_init:
	gpdata = NULL;
err_dt_parse:
	devm_kfree(&(client->dev), pdata);
err_alloc_platform_data:
	return ret;
}

static int tps61280a_remove(struct i2c_client *client)
{
	struct tps61280a_platform_data *tps = i2c_get_clientdata(client);

	gpdata = NULL;
	devm_kfree(&(client->dev), tps);

	return 0;
}

static struct of_device_id tps61280a_match_table[] = {
	{.compatible = "ti,tps61280a" },
	{},
};

static struct i2c_device_id tps61280a_id[] = {
	{ "ti,tps61280a", 0},
	{}
};

static struct i2c_driver tps61280a_driver = {
	.driver = {
		.name = "ti,tps61280a",
		.owner = THIS_MODULE,
		.of_match_table = tps61280a_match_table,
	},
	.probe = tps61280a_probe,
	.remove = tps61280a_remove,
	.id_table = tps61280a_id,
};

static int __init tps61280a_init(void)
{
	return i2c_add_driver(&tps61280a_driver);
}

static void __exit tps61280a_exit(void)
{
	i2c_del_driver(&tps61280a_driver);
}

subsys_initcall(tps61280a_init);
module_exit(tps61280a_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("tps61280a driver");

