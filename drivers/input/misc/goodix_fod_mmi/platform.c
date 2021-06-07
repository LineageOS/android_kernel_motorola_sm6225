/*
 * platform indepent driver interface
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;
	gf_dev->power_enabled = 0;
	gf_dev->pwr_supply = NULL;
	if(of_property_read_bool(np,"rgltr-ctrl-support")) {
		gf_dev->rgltr_ctrl_support = 1;
	} else {
		gf_dev->rgltr_ctrl_support = 0;
		pr_err("goodix: No regulator control parameter defined\n");
	}
	if (gf_dev->rgltr_ctrl_support) {
		gf_dev->pwr_supply = regulator_get(dev, "fp,vdd");
		if (IS_ERR_OR_NULL(gf_dev->pwr_supply)) {
			gf_dev->pwr_supply = NULL;
			gf_dev->rgltr_ctrl_support = 0;
			pr_warn("goodix Unable to get fp,vdd");
		} else {
			rc = of_property_read_u32_array(np, "fp,voltage-range", gf_dev->pwr_voltage_range, 2);
			if (rc) {
				gf_dev->pwr_voltage_range[0] = -1;
				gf_dev->pwr_voltage_range[1] = -1;
			}
			if (regulator_count_voltages(gf_dev->pwr_supply) > 0) {
				if((gf_dev->pwr_voltage_range[0] >0) && (gf_dev->pwr_voltage_range[1] > 0))
					rc = regulator_set_voltage(gf_dev->pwr_supply, gf_dev->pwr_voltage_range[0], gf_dev->pwr_voltage_range[1]);
				if (rc) {
					pr_warn("goodix: %s : set vdd regulator voltage failed %d \n", __func__, rc);
				}
			}
		}
	}

	gf_dev->pwr_gpio = of_get_named_gpio(np, "fp-gpio-ven", 0);
	if (gf_dev->pwr_gpio < 0) {
		pr_warn("goodix: failed to get pwr gpio!\n");
	} else {
		if (gpio_is_valid(gf_dev->pwr_gpio)) {
			rc = devm_gpio_request(dev, gf_dev->pwr_gpio, "goodix_pwr");
			if (rc) {
				pr_err("goodix: failed to request pwr gpio, rc = %d\n", rc);
				goto err_pwr;
			}
			gpio_direction_output(gf_dev->pwr_gpio, 1);
		}
	}
	gf_power_on(gf_dev);
	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("goodix: failed to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("goodix: failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("goodix: failed to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("goodix: failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	return rc;
err_irq:
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		devm_gpio_free(dev, gf_dev->reset_gpio);
		gf_dev->reset_gpio = -1;
	}
err_reset:
	gf_power_off(gf_dev);
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		devm_gpio_free(dev, gf_dev->pwr_gpio);
		gf_dev->pwr_gpio = -1;
		pr_info("goodix: remove pwr_gpio success\n");
	}
err_pwr:
	if (!IS_ERR_OR_NULL(gf_dev->pwr_supply))
	{
		pr_info(" %s goodix:  devm_regulator_put \n", __func__);
		regulator_put(gf_dev->pwr_supply);
		gf_dev->pwr_supply= NULL;
	}
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	struct device *dev = &gf_dev->spi->dev;
	pr_info("goodix:  %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		devm_gpio_free(dev, gf_dev->irq_gpio);
		gf_dev->irq_gpio = -1;
		pr_info("goodix: remove irq_gpio success\n");
	}
	if(gf_dev->rgltr_ctrl_support ||gpio_is_valid(gf_dev->pwr_gpio)){
		gf_power_off(gf_dev);
	}
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		devm_gpio_free(dev, gf_dev->pwr_gpio);
		gf_dev->pwr_gpio = -1;
		pr_info("goodix: remove pwr_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		devm_gpio_free(dev, gf_dev->reset_gpio);
		gf_dev->reset_gpio = -1;
		pr_info("goodix: remove reset_gpio success\n");
	}

	if (gf_dev->rgltr_ctrl_support && !IS_ERR_OR_NULL(gf_dev->pwr_supply))
	{
		pr_info(" goodix: %s : devm_regulator_put \n", __func__);
		regulator_put(gf_dev->pwr_supply);
		gf_dev->pwr_supply= NULL;
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;
	if (!gf_dev->power_enabled) {
		if(gf_dev->rgltr_ctrl_support && !IS_ERR_OR_NULL(gf_dev->pwr_supply)) {
			rc = regulator_enable(gf_dev->pwr_supply);
			pr_warn("goodix:  %s : enable  pwr_supply return %d \n", __func__, rc);
		}
		if (gpio_is_valid(gf_dev->pwr_gpio)) {
			gpio_direction_output(gf_dev->pwr_gpio, 1);
			pr_warn(" goodix: %s : set  pwr_gpio:%d  1\n", __func__, gf_dev->pwr_gpio);
		}
		gf_dev->power_enabled = 1;
		if(gf_dev->rgltr_ctrl_support ||gpio_is_valid(gf_dev->pwr_gpio)){
			usleep_range(11000,12000);
		}
	}
	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	if (gf_dev->power_enabled) {
		if (gf_dev->rgltr_ctrl_support  && !IS_ERR_OR_NULL(gf_dev->pwr_supply)) {
			rc = regulator_disable(gf_dev->pwr_supply);
			pr_warn(" goodix: %s : disable  pwr_supply return %d \n", __func__, rc);
		}
		if (gpio_is_valid(gf_dev->pwr_gpio)) {
			gpio_direction_output(gf_dev->pwr_gpio, 0);
			pr_warn(" goodix: %s : set  pwr_gpio:%d 0 \n", __func__,gf_dev->pwr_gpio);
		}
		gf_dev->power_enabled = 0;
	}
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	}
	pr_warn(" goodix: %s : gf_hw_reset \n", __func__);
	gpio_direction_output(gf_dev->reset_gpio, 0);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

