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
#include <linux/iio/consumer.h>

#define RBALANCE_VDIFF_MV	200
#define GPIO_SET_DELAY 		50
#define CHRG_FULLCURR_EN	1
#define CHRG_FULLCURR_DIS	0
#define HBDLY_DISCHARGE_SEC	30
#define HBDLY_CHARGE_SEC	6

static struct fet_control_data {
	struct device *dev;
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
	int vbus_ocp_fault_n_gpio;
	struct iio_channel *vbatt2_sns_chan;
} fetControlData;

int battplus_state = -1;
int balance_state = -1;
int fullcurr_state = -1;
int ocp_fault_n_state = -1;

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

static int set_ocp_fault_n_state(const char *val, const struct kernel_param *kp)
{
	int rc;
	long mode;

	if (!fetControlData.init_done)
		return -ENODEV;

	rc = kstrtol(val, 0, &mode);
	if (rc)
		return rc;

	rc = update_state_gpio(fetControlData.vbus_ocp_fault_n_gpio, !!mode);
	if (rc) {
			pr_err("Update fet-ctl gpio [%d] failed\n", fetControlData.vbus_ocp_fault_n_gpio);
			return rc;
	} else {
		ocp_fault_n_state = !!mode;
		pr_debug("Set vbus_ocp_fault_n gpio[%d], VAL:[%d]\n",
				fetControlData.vbus_ocp_fault_n_gpio,
				gpio_get_value(fetControlData.vbus_ocp_fault_n_gpio));
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

static struct kernel_param_ops ocpfault_ops =
{
	.set = &set_ocp_fault_n_state,
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

module_param_cb(ocp_fault_n_state,
    &ocpfault_ops,
    &ocp_fault_n_state,
    S_IRUGO | S_IWUSR
);

static int mmi_is_factory(void)
{
	const char *bootargs_str = NULL;
	char *kvpair = NULL;
	char *value = NULL;
	int ret = -1;
	struct device_node *n = of_find_node_by_path("/chosen");
	size_t bootargs_str_len = 0;

	if (n == NULL)
		goto ret;

	if (of_property_read_string(n, "mmi,bootconfig", &bootargs_str) != 0)
		goto putnode;

	bootargs_str_len = strlen(bootargs_str);
	kvpair = strnstr(bootargs_str, "androidboot.mode=", strlen(bootargs_str));
	if (kvpair) { // found key
		value = strchr(kvpair, '=') + 1;
		if (!strncmp(value, "mot-factory", 11))
			ret = 1;
		else
			ret = 0;
	}

putnode:
	of_node_put(n);
ret:
	return ret;
}

static void update_work(struct work_struct *work)
{
	struct fet_control_data *data = container_of(work, struct fet_control_data, update.work);
	int main_curr, main_mv, flip_mv = 0, usbtype;
	int main_chg = 1, rc = 0;
	int hb_sched_time = HBDLY_DISCHARGE_SEC;
	int vbatt2_sns_uv = -EINVAL;
	const char * usb_types[] = {"UNKNOWN","SDP","DCP","CDP","ACA","Type-C","PD","PD_DRP","PD_PPS","BRICKID"};

	fullcurr_state = CHRG_FULLCURR_DIS;

	usbtype = get_ps_int_prop(data->usb_psy,
		POWER_SUPPLY_PROP_USB_TYPE);
	pr_info("USB-TYPE found:%d [%s]\n", usbtype,usb_types[usbtype] );

	/* For no Charger connected & battfet already closed, nothing more to be done */
	if (mmi_is_factory()) {
		pr_err("Factory Mode- Cancel Work.. \n");
		cancel_delayed_work(&data->update);
		return;
	} else if (usbtype == POWER_SUPPLY_USB_TYPE_UNKNOWN && battplus_state) {
		pr_err("BATTFET Closed & Not Charging- Cancel Work..\n");
		cancel_delayed_work(&data->update);
		return;
	}

	/* VMain in mV */
	main_mv = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	main_mv /= 1000;

	/* Main current in uA */
	main_curr = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW);
	if (main_curr > 0 || usbtype != POWER_SUPPLY_USB_TYPE_UNKNOWN)
		pr_info("Charging Main\n");
	else
		main_chg = -1;	// Discharging Main

	data->main_chg_curr_max = get_ps_int_prop(data->main_batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
	if (data->main_chg_curr_max < 0) {
		pr_err("Failed to get Main Max chrg curr\n");
		return;
	}

	ocp_fault_n_state = gpio_get_value(fetControlData.vbus_ocp_fault_n_gpio);
	pr_info("ocp_fault_n GPIO:[%d], VAL:[%d]\n",
			fetControlData.vbus_ocp_fault_n_gpio, ocp_fault_n_state);

	/* Leave FULLCURR_DIS until charger is connected */
	if (usbtype != POWER_SUPPLY_USB_TYPE_UNKNOWN) {
		if ( (main_chg * main_curr > data->main_chg_curr_max / 2) ||
				!ocp_fault_n_state ) {
			fullcurr_state = CHRG_FULLCURR_DIS;
			pr_err("WARNING! Main-OC or OCP-TRIP: Main_curr:%d, ocp_fault_n:%d\n",
					main_curr, ocp_fault_n_state);
		} else {
			fullcurr_state = CHRG_FULLCURR_EN;
		}
	}

	/* Read the ADC pm8350b_vbatt2_sns div3, scale *3 and to mV */
	rc = iio_read_channel_processed(fetControlData.vbatt2_sns_chan, &vbatt2_sns_uv);
	if (rc < 0) {
		pr_err("Error Reading pm8350b_vbatt2_sns_voltage- rc:%d\n", rc);
		return;
	} else {
		pr_info("Read pm8350b_vbatt2_sns_voltage- VAL-uV:%d\n", vbatt2_sns_uv);
		flip_mv = (3 * vbatt2_sns_uv) / 1000;
	}

	pr_info("Main_curr:%d, main_Ichg_MAX:%d\n",
		main_curr, data->main_chg_curr_max);
	pr_err("Flip-Vbatt: %d, Main-Vbatt: %d\n", flip_mv, main_mv);

	/* FET BattFET paths */
	/* Leave Balance dflt-en & toggle in/out parallel low-Z battplus fet */
	balance_state = 0;
	if ( abs(main_mv - flip_mv) < RBALANCE_VDIFF_MV) {
		battplus_state = 1;
		pr_info("battplus-EN: %d\n", battplus_state);
	} else {
		battplus_state = 0;
		pr_info("battplus-DIS: %d\n", battplus_state);
	}

	if (main_chg == 1)
		hb_sched_time = HBDLY_CHARGE_SEC;

	/* Update GPIO controls */
	update_state_gpio(fetControlData.battplus_en_gpio, battplus_state);
	update_state_gpio(fetControlData.balance_en_n_gpio, balance_state);
	update_state_gpio(fetControlData.chrg_fullcurr_en_gpio, fullcurr_state);

	schedule_delayed_work(&data->update, hb_sched_time * HZ);
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
		schedule_delayed_work(&data->update, 3 * HZ);
	}

	return 0;
}

static int parse_dt(struct device_node *node)
{
	int rc = 0;
	const char *main_batt_psy_name;

	rc |= of_property_read_string(node, "mmi,main-batt-psy", &main_batt_psy_name);
	if (!rc && main_batt_psy_name) {
		fetControlData.main_batt_psy = power_supply_get_by_name(main_batt_psy_name);
		if (!fetControlData.main_batt_psy) {
			pr_debug("Could not get main batt psy, maybe we are early - defer.");
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

	fetControlData.vbus_ocp_fault_n_gpio = of_get_named_gpio(node,
			"mmi,vbus-ocp-fault-n-gpio", 0);
	if (!gpio_is_valid(fetControlData.vbus_ocp_fault_n_gpio)) {
		pr_err("ocp-fault-gpio is not valid!\n");
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

	fetControlData.usb_psy = power_supply_get_by_name("usb");
	if (!fetControlData.usb_psy) {
		pr_debug("Could not get usb psy, maybe we are early - defer.");
		return -EPROBE_DEFER;
	}

	/* Get the ADC device instance pm8350b_vbatt2_sns, gpio3_pu0_div3 */
	fetControlData.vbatt2_sns_chan = iio_channel_get(fetControlData.dev,"pm8350b_vbatt2_sns");
	if (!fetControlData.vbatt2_sns_chan) {
		pr_err("Error Getting pm8350b_vbatt2_sns_voltage channel- rc:%d\n",rc);
		return -EINVAL;
	} else {
		pr_info("Found pm8350b_vbatt2_sns voltage channel\n");
	}

	rc = gpio_request(fetControlData.battplus_en_gpio,
					"mmi,flip_battplus_en_gpio");
	if (rc) {
		pr_err("Failed request battplus_en_gpio\n");
		goto fail;
	}

	rc = gpio_request(fetControlData.balance_en_n_gpio,
					"mmi,flip_balance_en_n_gpio");
	if (rc) {
		pr_err("Failed request balance_en_n_gpio\n");
		goto fail;
	}

	rc = gpio_request(fetControlData.chrg_fullcurr_en_gpio,
					"mmi,chrg-fullcurr-en-gpio");
	if (rc) {
		pr_err("Failed request chrg-fullcurr-en-gpio\n");
		goto fail;
	}

	rc = gpio_request(fetControlData.vbus_ocp_fault_n_gpio,
					"mmi,vbus-ocp-fault-n-gpio");
	if (rc) {
		pr_err("Failed request vbus-ocp-fault-n-gpio\n");
		goto fail;
	}

	/* Set default Flip Battery Path FETs: battplus path disable*/
	rc = gpio_direction_output(fetControlData.battplus_en_gpio, 0);//Path Disabled
	if (rc) {
		pr_err("Unable to set DIR flip_battplus_en [%d]\n",
				fetControlData.battplus_en_gpio);
		goto fail;
	}
	battplus_state = 0;

	/* Set Flip Rbalance path enable */
	rc = gpio_direction_output(fetControlData.balance_en_n_gpio, 0);// Path Enabled
	if (rc) {
		pr_err("Unable to set DIR/VAL Flip_balance_en_n [%d]\n",
				fetControlData.balance_en_n_gpio);
		goto fail;
	}
	balance_state = 0;

	/* Set charger fullcurr enable */
	rc = gpio_direction_output(fetControlData.chrg_fullcurr_en_gpio, 0);// ChrgFullCurr dflt DISABLE
	if (rc) {
		pr_err("Unable to set DIR/VAL chrg_fullcurr_en_gpio [%d]\n",
				fetControlData.chrg_fullcurr_en_gpio);
		goto fail;
	}
	fullcurr_state = CHRG_FULLCURR_DIS;

	/* Set OCP_FAULT input */
	rc = gpio_direction_input(fetControlData.vbus_ocp_fault_n_gpio);
	if (rc) {
		pr_err("Unable to set DIR vbus_ocp_fault_n [%d]\n",
				fetControlData.vbus_ocp_fault_n_gpio);
		goto fail;
	}

	pr_info("Flip_battplus_en Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.battplus_en_gpio,
				gpio_get_value(fetControlData.battplus_en_gpio) );
	pr_info("Flip_balance_en_n Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.balance_en_n_gpio,
				gpio_get_value(fetControlData.balance_en_n_gpio) );
	pr_info("chrg_fullcurr Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.chrg_fullcurr_en_gpio,
				gpio_get_value(fetControlData.chrg_fullcurr_en_gpio) );
	pr_info("ocp_fault_n Init GPIO:[%d], VAL:[%d]\n",
				fetControlData.vbus_ocp_fault_n_gpio,
				gpio_get_value(fetControlData.vbus_ocp_fault_n_gpio) );

	// Notify on plug/unplug
	fetControlData.ps_notif.notifier_call = ps_notify_callback;
	if (power_supply_reg_notifier(&fetControlData.ps_notif)) {
		pr_err("Failed to register notifier\n");
		goto fail;
	}

	// Work to update the fet & charge current states.
	INIT_DELAYED_WORK(&fetControlData.update, update_work);
	schedule_delayed_work(&fetControlData.update, HBDLY_DISCHARGE_SEC * HZ);

	fetControlData.init_done = true;
	return 0;
fail:
	return rc;
}

static int fet_control_remove(struct platform_device *pdev)
{

	if (fetControlData.init_done) {
		cancel_delayed_work(&fetControlData.update);
		power_supply_unreg_notifier(&fetControlData.ps_notif);
		gpio_free(fetControlData.battplus_en_gpio);
		gpio_free(fetControlData.balance_en_n_gpio);
		gpio_free(fetControlData.chrg_fullcurr_en_gpio);
		gpio_free(fetControlData.vbus_ocp_fault_n_gpio);
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
