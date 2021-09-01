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
#define MAIN_MV_MID_LO		3600
#define MAIN_MV_MID_HI		3800
#define MAIN_CURR_SWITCH	250
#define GPIO_SET_DELAY 		50
#define CHRG_FULLCURR_EN	1
#define HBDLY_DISCHARGE_MS	60000
#define HBDLY_CHARGE_MS		6000

static struct fet_control_data {
	struct device *dev;
	struct power_supply	*flip_batt_psy;
	struct power_supply *main_batt_psy;
	struct delayed_work update;
	bool init_done;
	bool ps_is_present;
	int flip_chg_curr_max;
	int main_chg_curr_max;
	int battplus_en_gpio;
	int balance_en_n_gpio;
	int chrg_fullcurr_en_gpio;
} fetControlData;

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

	if (!fetControlData.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;

	rc = update_state_gpio(fetControlData.battplus_en_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fetControlData.battplus_en_gpio);
			return rc;
	} else {
		battplus_state = !!mode;
		pr_debug("Set using flip_battplus gpio[%d], VAL:[%d]\n",
				fetControlData.battplus_en_gpio,
				gpio_get_value(fetControlData.battplus_en_gpio));
	}

	return 0;
}

static int set_balance_state(const char *val, const struct kernel_param *kp)
{
	int rc;
	long mode;

	if (!fetControlData.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;

	rc = update_state_gpio(fetControlData.balance_en_n_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fetControlData.balance_en_n_gpio);
			return rc;
	} else {
		balance_state = !!mode;
		pr_debug("Set using flip_balance_enn gpio[%d], VAL:[%d]\n",
				fetControlData.balance_en_n_gpio,
				gpio_get_value(fetControlData.balance_en_n_gpio));
	}

	return 0;
}

static int set_fullcurr_state(const char *val, const struct kernel_param *kp)
{
	int rc;
	long mode;

	if (!fetControlData.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;


	rc = update_state_gpio(fetControlData.chrg_fullcurr_en_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fetControlData.chrg_fullcurr_en_gpio);
			return rc;
	} else {
		fullcurr_state = !!mode;
		pr_debug("Set using Chrg fullcurr_en gpio[%d], VAL:[%d]\n",
				fetControlData.chrg_fullcurr_en_gpio,
				gpio_get_value(fetControlData.chrg_fullcurr_en_gpio));
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
	int main_curr, main_mv;
	int main_dischg = 1;
	int hb_sched_time = HBDLY_DISCHARGE_MS;

	fullcurr_state = CHRG_FULLCURR_EN;

	/*flip_mv = get_ps_int_prop(data->flip_batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	flip_mv /= 1000;*/

	main_mv = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	main_mv /= 1000;

	/* Main current sysfs already in mA */
	main_curr = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW);
	if (main_curr < 0) {
		pr_info("Charging Main\n");
		main_dischg = -1;
	}

	data->main_chg_curr_max = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
	data->main_chg_curr_max /= 1000;
	if (data->main_chg_curr_max < 0) {
		pr_err("Failed to get Main Max chrg curr\n");
		return;
	}

	/*pr_info("Flip_curr:%d, flip_Ichg_MAX:%d, fullcurr state:%d\n",
		flip_curr, data->flip_chg_curr_max, fullcurr_state);*/
	pr_info("Main_curr:%d, main_Ichg_MAX:%d\n",
		main_curr, data->main_chg_curr_max);
	pr_info("Flip-Vbatt: --, Main-Vbatt: %d\n", main_mv);

	/* FET paths set in Batt discharge state */
	if (main_dischg == 1) {
		/* Leave Balance dflt-en & toggle in/out parallel low-Z battplus fet */
		balance_state = 0;
		/* if ((main_mv - flip_mv) < RBALANCE_VDIFF_MV) { */
		/* MMI_STOPSHIP power: Temproary W/A for EVB, without ADC Input
		 * Switch battplus_en Low-Z path at MAIN-Mid-Vbatt*/
		if ( (main_mv >= MAIN_MV_MID_LO && main_mv <= MAIN_MV_MID_HI) &&
					(main_curr/1000 < MAIN_CURR_SWITCH) ) {
			battplus_state = 1;
			pr_info("battplus-EN: %d\n", battplus_state);
		} else {
			battplus_state = 0;
			pr_info("battplus-DIS: %d\n", battplus_state);
			schedule_delayed_work(&data->update, msecs_to_jiffies(hb_sched_time));
		}
	}

	/* Update GPIO controls */
	update_state_gpio(fetControlData.battplus_en_gpio, battplus_state);
	update_state_gpio(fetControlData.balance_en_n_gpio, balance_state);
	update_state_gpio(fetControlData.chrg_fullcurr_en_gpio, fullcurr_state);
}

static int parse_dt(struct device_node *node)
{
	int rc = 0;
	const char *main_batt_psy_name;

	rc |= of_property_read_string(node, "mmi,main-batt-psy", &main_batt_psy_name);
	if (!rc && main_batt_psy_name) {
		fetControlData.main_batt_psy = power_supply_get_by_name(main_batt_psy_name);
		if (!fetControlData.main_batt_psy) {
			pr_debug("Could not get flip batt psy and main batt psy, maybe we are early - defer.");
			return -EPROBE_DEFER;
		}
	}

	fetControlData.battplus_en_gpio = of_get_named_gpio(node,
			"mmi,flip_battplus_en_gpio", 0);
	if (!gpio_is_valid(fetControlData.battplus_en_gpio)) {
		pr_err("battplus_en_gpio is not valid!\n");
		return -ENODEV;
	}

	fetControlData.balance_en_n_gpio = of_get_named_gpio(node,
			"mmi,flip_balance_en_n_gpio", 0);
	if (!gpio_is_valid(fetControlData.balance_en_n_gpio)) {
		pr_err("balance_en_n_gpio is not valid!\n");
		return -ENODEV;
	}

	fetControlData.chrg_fullcurr_en_gpio = of_get_named_gpio(node,
			"mmi,chrg-fullcurr-en-gpio", 0);
	if (!gpio_is_valid(fetControlData.chrg_fullcurr_en_gpio)) {
		pr_err("chrg-fullcurr-en-gpio is not valid!\n");
		return -ENODEV;
	}

	return rc;
}

static int fet_control_probe(struct platform_device *pdev)
{
	int rc = 1;

	fetControlData.init_done = 0;
	fetControlData.dev = &pdev->dev;

	rc = parse_dt(fetControlData.dev->of_node);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			pr_err("Failed to parse device tree\n");
		goto fail;
	}

	rc = gpio_request(fetControlData.battplus_en_gpio, "mmi,flip_battplus_en_gpio");
	if (rc) {
		pr_err("Failed request battplus_en_gpio\n");
		goto fail;
	}

	rc = gpio_request(fetControlData.balance_en_n_gpio, "mmi,flip_balance_en_n_gpio");
	if (rc) {
		pr_err("Failed request balance_en_n_gpio\n");
		goto fail;
	}

	rc = gpio_request(fetControlData.chrg_fullcurr_en_gpio, "mmi,chrg-fullcurr-en-gpio");
	if (rc) {
		pr_err("Failed request chrg-fullcurr-en-gpio\n");
		goto fail;
	}

	/* Set default Flip Battery Path FETs: battplus path disable*/
	rc = gpio_direction_output(fetControlData.battplus_en_gpio, 0);	//Path Disabled
	if (rc) {
		pr_err("Unable to set DIR flip_battplus_en [%d]\n", fetControlData.battplus_en_gpio);
		goto fail;
	}
	battplus_state = 0;

	/* Set Flip Rbalance path enable */
	rc = gpio_direction_output(fetControlData.balance_en_n_gpio, 0);	// Path Enabled
	if (rc) {
		pr_err("Unable to set DIR/VAL Flip_balance_en_n [%d]\n", fetControlData.balance_en_n_gpio);
		goto fail;
	}
	balance_state = 0;

	/* Set charger fullcurr enable */
	rc = gpio_direction_output(fetControlData.chrg_fullcurr_en_gpio, 1);	// ChrgFullCurr dflt Enable
	if (rc) {
		pr_err("Unable to set DIR/VAL chrg_fullcurr_en_gpio [%d]\n", fetControlData.chrg_fullcurr_en_gpio);
		goto fail;
	}
	fullcurr_state = CHRG_FULLCURR_EN;

	pr_info("Flip_battplus_en Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.battplus_en_gpio,
				gpio_get_value(fetControlData.battplus_en_gpio) );
	pr_info("Flip_balance_en_n Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.balance_en_n_gpio,
				gpio_get_value(fetControlData.balance_en_n_gpio) );
	pr_info("chrg_fullcurr Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.chrg_fullcurr_en_gpio,
				gpio_get_value(fetControlData.chrg_fullcurr_en_gpio) );

	// Work to update the fet & charge current states.
	INIT_DELAYED_WORK(&fetControlData.update, update_work);
	schedule_delayed_work(&fetControlData.update, 10);

	fetControlData.init_done = true;
	return 0;
fail:
	return rc;
}

static int fet_control_remove(struct platform_device *pdev)
{

	if (fetControlData.init_done) {
		cancel_delayed_work(&fetControlData.update);
		gpio_free(fetControlData.battplus_en_gpio);
		gpio_free(fetControlData.balance_en_n_gpio);
		gpio_free(fetControlData.chrg_fullcurr_en_gpio);
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
