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

#define RBALANCE_VDIFF_MV	100
#define GPIO_SET_DELAY 		50
#define CHRG_FULLCURR_EN	1
#define CHRG_FULLCURR_DIS	0
#define HBDLY_DISCHARGE_MS	30000
#define HBDLY_CHARGE_MS		6000

static struct fet_control_data {
	struct device *dev;
	struct power_supply	*flip_batt_psy;
	struct power_supply *main_batt_psy;
	struct power_supply *usb_psy;
	struct notifier_block ps_notif;
	struct delayed_work update;
	bool init_done;
	bool ps_is_present;
	int flip_chg_curr_max;
	int main_chg_curr_max;
	int battplus_en_gpio;
	int balance_en_n_gpio;
	int chrg_fullcurr_en_gpio;
} fet_control_data;

int battplus_state = -1;
int balance_state = -1;
int fullcurr_state = -1;

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

static int update_state_gpio(int gpio, int val)
{
	int readval;

	gpio_set_value(gpio, val);
	udelay(GPIO_SET_DELAY);

	readval = gpio_get_value(gpio);

	if (readval != val) {
		pr_err("FAIL set fet-ctrl gpio[%d], wrote:%d, read:%d\n",
				gpio,
				val,
				readval);
		return -EINVAL;
	}

	return 0;
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

	rc = update_state_gpio(fet_control_data.battplus_en_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fet_control_data.battplus_en_gpio);
			return rc;
	} else {
		battplus_state = !!mode;
		pr_debug("Set using flip_battplus gpio[%d], VAL:[%d]\n",
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

	rc = update_state_gpio(fet_control_data.balance_en_n_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fet_control_data.balance_en_n_gpio);
			return rc;
	} else {
		balance_state = !!mode;
		pr_debug("Set using flip_balance_enn gpio[%d], VAL:[%d]\n",
				fet_control_data.balance_en_n_gpio,
				gpio_get_value(fet_control_data.balance_en_n_gpio));
	}

	return 0;
}

static int set_fullcurr_state(const char *val, const struct kernel_param *kp)
{
	int rc;
	long mode;

	if (!fet_control_data.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;


	rc = update_state_gpio(fet_control_data.chrg_fullcurr_en_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fet_control_data.chrg_fullcurr_en_gpio);
			return rc;
	} else {
		fullcurr_state = !!mode;
		pr_debug("Set using Chrg fullcurr_en gpio[%d], VAL:[%d]\n",
				fet_control_data.chrg_fullcurr_en_gpio,
				gpio_get_value(fet_control_data.chrg_fullcurr_en_gpio));
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

static struct kernel_param_ops fullcurr_ops =
{
	.set = &set_fullcurr_state,
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

module_param_cb(fullcurr_state,
    &fullcurr_ops,
    &fullcurr_state,
    S_IRUGO | S_IWUSR
);

static void update_work(struct work_struct *work)
{
	struct fet_control_data *data = container_of(work, struct fet_control_data, update.work);
	int flip_curr, main_curr, flip_mv, main_mv;
	int flip_dischg = 1;
	int main_dischg = 1;
	int hb_sched_time;

	flip_mv = get_ps_int_prop(data->flip_batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	flip_mv /= 1000;

	main_mv = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	main_mv /= 1000;

	flip_curr = get_ps_int_prop(data->flip_batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW);
	flip_curr /= 1000;
	if (flip_curr < 0) {
		pr_debug("Charging Flip\n");
		flip_dischg = -1;
	}

	/* Main current sysfs already in mA */
	main_curr = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW);
	if (main_curr < 0) {
		pr_debug("Charging Main\n");
		main_dischg = -1;
	}

	/* Max Allowed Charge Curr= 1C */
	data->flip_chg_curr_max = get_ps_int_prop(data->flip_batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
	data->flip_chg_curr_max /= 1000;
	if (data->flip_chg_curr_max < 0) {
		pr_err("Failed to get Flip Max chrg curr\n");
		return;
	}

	data->main_chg_curr_max = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
	data->main_chg_curr_max /= 1000;
	if (data->main_chg_curr_max < 0) {
		pr_err("Failed to get Main Max chrg curr\n");
		return;
	}

	if (flip_dischg * flip_curr > data->flip_chg_curr_max  ||
		main_dischg * main_curr > data->main_chg_curr_max) {
		fullcurr_state = CHRG_FULLCURR_DIS;
	} else {
		fullcurr_state = CHRG_FULLCURR_EN;
	}

	pr_info("Flip_curr:%d, flip_Ichg_MAX:%d, fullcurr state:%d\n",
		flip_curr, data->flip_chg_curr_max, fullcurr_state);
	pr_info("Main_curr:%d, main_Ichg_MAX:%d\n",
		main_curr, data->main_chg_curr_max);
	pr_info("Flip-Vbatt:%d, Main-Vbatt: %d\n", flip_mv, main_mv);

	/* FET paths set in Batt discharge state */
	if (flip_dischg ==1 && main_dischg ==1) {
		hb_sched_time = HBDLY_DISCHARGE_MS;
		/* Leave Balance dflt-en & toggle in/out parallel low-Z battplus fet */
		balance_state = 0;
		if ((main_mv - flip_mv) < RBALANCE_VDIFF_MV) {
			battplus_state = 1;
			pr_debug("DISCHG-battplus-EN: %d\n", battplus_state);
		} else {
			battplus_state = 0;
			pr_debug("DISCHG-battplus-DIS: %d\n", battplus_state);
		}
	} else {
		hb_sched_time = HBDLY_CHARGE_MS;
		battplus_state = 0;
		balance_state = 1;
	}

	/* Update GPIO controls */
	update_state_gpio(fet_control_data.battplus_en_gpio, battplus_state);
	update_state_gpio(fet_control_data.balance_en_n_gpio, balance_state);
	update_state_gpio(fet_control_data.chrg_fullcurr_en_gpio, fullcurr_state);

	schedule_delayed_work(&data->update, msecs_to_jiffies(hb_sched_time));
}

static int ps_notify_callback(struct notifier_block *nb,
		unsigned long event, void *p)
{
	struct fet_control_data *data = container_of(nb, struct fet_control_data, ps_notif);
	struct power_supply *psy = p;
	union power_supply_propval pval = {0};
	int retval;
	bool present_ps;

	if ((event == PSY_EVENT_PROP_CHANGED) &&
		psy && psy->desc->get_property && psy->desc->name &&
		!strncmp(psy->desc->name, "usb", sizeof("usb")) && data) {
		pr_info("psy notif: event = %lu\n", event);

		retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
						&pval);
		if (retval) {
			pr_err("%s psy get property failed, ERR: %d\n", psy->desc->name, retval);
			return retval;
		}
		present_ps = (pval.intval) ? true : false;
		pr_info("%s is %s\n", psy->desc->name,
				(present_ps) ? "present" : "not present");

		if (event == PSY_EVENT_PROP_CHANGED) {
			if (data->ps_is_present == present_ps) {
				pr_info("ps present state unchanged\n");
				return 0;
			}
		}
		data->ps_is_present = present_ps;

		cancel_delayed_work(&data->update);
		schedule_delayed_work(&data->update, msecs_to_jiffies(1));
	}

	return 0;
}

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

	fet_control_data.chrg_fullcurr_en_gpio = of_get_named_gpio(node,
			"mmi,chrg-fullcurr-en-gpio", 0);
	if (!gpio_is_valid(fet_control_data.chrg_fullcurr_en_gpio)) {
		pr_err("chrg-fullcurr-en-gpio is not valid!\n");
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

	rc = gpio_request(fet_control_data.chrg_fullcurr_en_gpio, "mmi,chrg-fullcurr-en-gpio");
	if (rc) {
		pr_err("Failed request chrg-fullcurr-en-gpio\n");
		goto fail;
	}

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

	/* Set charger fullcurr enable */
	rc = gpio_direction_output(fet_control_data.chrg_fullcurr_en_gpio, 0);	// ChrgFullCurr dflt Disable
	if (rc) {
		pr_err("Unable to set DIR/VAL chrg_fullcurr_en_gpio [%d]\n", fet_control_data.chrg_fullcurr_en_gpio);
		goto fail;
	}
	fullcurr_state = 0;

	pr_info("Flip_battplus_en Init GPIO:[%d], VAL:[%d]\n",
				fet_control_data.battplus_en_gpio,
				gpio_get_value(fet_control_data.battplus_en_gpio) );
	pr_info("Flip_balance_en_n Init GPIO:[%d], VAL:[%d]\n",
				fet_control_data.balance_en_n_gpio,
				gpio_get_value(fet_control_data.balance_en_n_gpio) );
	pr_info("chrg_fullcurr Init GPIO:[%d], VAL:[%d]\n",
				fet_control_data.chrg_fullcurr_en_gpio,
				gpio_get_value(fet_control_data.chrg_fullcurr_en_gpio) );


	// Notify on plug/unplug
	fet_control_data.ps_notif.notifier_call = ps_notify_callback;
	if (power_supply_reg_notifier(&fet_control_data.ps_notif)) {
		pr_err("Failed to register notifier\n");
		goto fail;
	}

	// Work to update the fet & charge current states.
	INIT_DELAYED_WORK(&fet_control_data.update, update_work);
	schedule_delayed_work(&fet_control_data.update, 1);

	fet_control_data.init_done = true;
	return 0;
fail:
	return rc;
}

static int fet_control_remove(struct platform_device *pdev)
{

	if (fet_control_data.init_done) {
		cancel_delayed_work(&fet_control_data.update);
		power_supply_unreg_notifier(&fet_control_data.ps_notif);
		gpio_free(fet_control_data.battplus_en_gpio);
		gpio_free(fet_control_data.balance_en_n_gpio);
		gpio_free(fet_control_data.chrg_fullcurr_en_gpio);
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
