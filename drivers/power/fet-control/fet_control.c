/*
 * Copyright (c) 2021 Motorola Mobility, LLC.
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
#define pr_fmt(fmt)	"fet_control-[%s]: " fmt, __func__
#include <linux/module.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

static struct fet_control_data {
	struct device *dev;
	struct power_supply	*flip_batt_psy;
	struct power_supply *main_batt_psy;
	struct power_supply *usb_psy;
	struct	notifier_block ps_notif;
	struct	work_struct update;
	bool	init_done;
	int battplus_en_gpio;
	int balance_en_n_gpio;
} fet_control_data;

int battplus_state = -1;
int balance_state = -1;

static int get_ps_int_prop(struct power_supply *psy, enum power_supply_property prop)
{
	int rc;
	union power_supply_propval val;

	if (!psy)
		return -1;

	rc = power_supply_get_property(psy,
			prop, &val);
	if (rc < 0) {
		pr_err("Error getting prop %d rc = %d\n", prop, rc);
		return -1;
	}

	return val.intval;
}

static int set_battplus_state(const char *val, const struct kernel_param *kp)
{
	int rc;
	long mode;

	if (!fet_control_data.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;

	gpio_set_value(fet_control_data.battplus_en_gpio, !!mode);
	udelay(500);

	if (gpio_get_value(fet_control_data.battplus_en_gpio) != !!mode) {
		pr_err("FAIL Set flip_battplus gpio[%d], wrote:%d, read:%d\n",
				fet_control_data.battplus_en_gpio,
				!!mode,
				gpio_get_value(fet_control_data.battplus_en_gpio));
		return -EINVAL;
	} else {
		battplus_state = !!mode;
		pr_info("Set using flip_battplus gpio[%d], VAL:[%d]\n",
				fet_control_data.battplus_en_gpio,
				gpio_get_value(fet_control_data.battplus_en_gpio));
	}

	return 0;
}

static int set_balance_state(const char *val, const struct kernel_param *kp)
{
	int rc;
	long mode;

	if (!fet_control_data.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;

	gpio_set_value(fet_control_data.balance_en_n_gpio, !!mode);
	udelay(500);

	if (gpio_get_value(fet_control_data.balance_en_n_gpio) != !!mode) {
		pr_err("FAIL set flip_balance gpio[%d], wrote:%d, read:%d\n",
				fet_control_data.balance_en_n_gpio,
				!!mode,
				gpio_get_value(fet_control_data.balance_en_n_gpio));
		return -EINVAL;
	} else {
		balance_state = !!mode;
		pr_info("Set using flip_bal_en_n gpio[%d], VAL:[%d]\n",
				fet_control_data.balance_en_n_gpio,
				gpio_get_value(fet_control_data.balance_en_n_gpio));
	}

	return 0;
}

static void update_work(struct work_struct *w)
{
	struct fet_control_data *data = container_of(w, struct fet_control_data, update);

	//unused for now
	(void)data;
	(void)get_ps_int_prop;
}

static int ps_notify_callback(struct notifier_block *nb,
		unsigned long event, void *p)
{
	struct fet_control_data *data = container_of(nb, struct fet_control_data, ps_notif);
	struct power_supply *psy = p;

	//unused for now
	(void)data;

	if (event == PSY_EVENT_PROP_CHANGED &&
			psy && psy->desc->get_property && psy->desc->name &&
			!strncmp(psy->desc->name, "battery", sizeof("battery"))) {
		//schedule_work(&data->update);
	}

	return 0;
}

static struct kernel_param_ops battplus_ops =
{
	.set = &set_battplus_state,
	.get = param_get_int,
};

static struct kernel_param_ops balance_ops =
{
	.set = &set_balance_state,
	.get = param_get_int,
};

module_param_cb(battplus_state,
    &battplus_ops,
    &battplus_state,
    S_IRUGO | S_IWUSR
);

module_param_cb(balance_state,
    &balance_ops,
    &balance_state,
    S_IRUGO | S_IWUSR
);

static int parse_dt(struct device_node *node)
{
	int rc = 0;
	const char *flip_batt_psy_name, *main_batt_psy_name;

	rc = of_property_read_string(node, "mmi,flip-batt-psy", &flip_batt_psy_name);
	rc |= of_property_read_string(node, "mmi,main-batt-psy", &main_batt_psy_name);
	if (!rc && flip_batt_psy_name && main_batt_psy_name) {
		fet_control_data.flip_batt_psy = power_supply_get_by_name(flip_batt_psy_name);
		fet_control_data.main_batt_psy = power_supply_get_by_name(main_batt_psy_name);
		if (!fet_control_data.flip_batt_psy || !fet_control_data.main_batt_psy) {
			pr_debug("Could not get flip batt psy and main batt psy, maybe we are early - defer.");
			return -EPROBE_DEFER;
		}
	}

	fet_control_data.battplus_en_gpio = of_get_named_gpio(node,
			"mmi,flip_battplus_en_gpio", 0);
	if (!gpio_is_valid(fet_control_data.battplus_en_gpio)) {
		pr_err("battplus_en_gpio is not valid!\n");
		return -ENODEV;
	}

	fet_control_data.balance_en_n_gpio = of_get_named_gpio(node,
			"mmi,flip_balance_en_n_gpio", 0);
	if (!gpio_is_valid(fet_control_data.balance_en_n_gpio)) {
		pr_err("balance_en_n_gpio is not valid!\n");
		return -ENODEV;
	}

	return rc;
}

static int fet_control_probe(struct platform_device *pdev)
{
	int rc = 1;

	fet_control_data.init_done = 0;
	fet_control_data.dev = &pdev->dev;

	rc = parse_dt(fet_control_data.dev->of_node);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			pr_err("Failed to parse device tree\n");
		goto fail;
	}

	fet_control_data.usb_psy = power_supply_get_by_name("usb");
	if (!fet_control_data.usb_psy) {
		pr_debug("Could not get usb psy, maybe we are early - defer.");
		return -EPROBE_DEFER;
	}

	rc = gpio_request(fet_control_data.battplus_en_gpio, "mmi,flip_battplus_en_gpio");
	if (rc) {
		pr_err("Failed request battplus_en_gpio\n");
		goto fail;
	}

	rc = gpio_request(fet_control_data.balance_en_n_gpio, "mmi,flip_balance_en_n_gpio");
	if (rc) {
		pr_err("Failed request balance_en_n_gpio\n");
		goto fail;
	}

	pr_info("Flip_battplus_en_GPIO: [%d]\n", fet_control_data.battplus_en_gpio);
	pr_info("Flip_balance_en_n_GPIO: [%d]\n",  fet_control_data.balance_en_n_gpio);

	/* Set default Flip Battery Path FETs: battplus path disable*/
	rc = gpio_direction_output(fet_control_data.battplus_en_gpio, 0);	//Path Disabled
	if (rc) {
		pr_err("Unable to set DIR flip_battplus_en [%d]\n", fet_control_data.battplus_en_gpio);
		goto fail;
	}
	battplus_state = 0;

	/* Set Flip Rbalance path enable */
	rc = gpio_direction_output(fet_control_data.balance_en_n_gpio, 0);	// Path Enabled
	if (rc) {
		pr_err("Unable to set DIR/VAL Flip_balance_en_n [%d]\n", fet_control_data.balance_en_n_gpio);
		goto fail;
	}
	balance_state = 0;

	pr_info("Flip_battplus_en SET GPIO:[%d], VAL:[%d]\n",
				fet_control_data.battplus_en_gpio,
				gpio_get_value(fet_control_data.battplus_en_gpio) );
	pr_info("Flip_balance_en_n SET GPIO:[%d], VAL:[%d]\n",
				fet_control_data.balance_en_n_gpio,
				gpio_get_value(fet_control_data.balance_en_n_gpio) );

	// TBD notify or run every X seconds?
	// Notify on plug/unplug?
	fet_control_data.ps_notif.notifier_call = ps_notify_callback;
	if (power_supply_reg_notifier(&fet_control_data.ps_notif)) {
		pr_err("Failed to register notifier\n");
		goto fail;
	}

	// Work to update the fet state.  Maybe delayed work that runs on heartbeat?
	INIT_WORK(&fet_control_data.update, update_work);
	schedule_work(&fet_control_data.update);

	fet_control_data.init_done = true;
	return 0;
fail:
	return rc;
}

static int fet_control_remove(struct platform_device *pdev)
{
	if (fet_control_data.init_done) {
		cancel_work_sync(&fet_control_data.update);
		power_supply_unreg_notifier(&fet_control_data.ps_notif);
	}
	return 0;
}


static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,fet-control", },
	{ },
};

static struct platform_driver fet_control_driver = {
	.driver		= {
		.name		= "mmi,fet-control",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe		= fet_control_probe,
	.remove		= fet_control_remove,
};
module_platform_driver(fet_control_driver);

MODULE_DESCRIPTION("Motorola Flip FET Control");
MODULE_LICENSE("GPL v2");
