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
#define pr_fmt(fmt)	"qpnp_adap_chg-[%s]: " fmt, __func__
#include <linux/module.h>
#include <linux/version.h>
#ifdef USE_MMI_CHARGER
#include "mmi_charger.h"
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 61))
#include <linux/mmi-pmic-voter.h>
#else
#include <linux/pmic-voter.h>
#endif
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/moduleparam.h>

#ifdef USE_MMI_CHARGER
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 61))
#define vote(votable, client_str, enabled, val) \
	mmi_vote(votable, client_str, enabled, val)

#define find_votable(name) mmi_find_votable(name)
#endif

static struct adap_chg_data {
	struct power_supply	*batt_psy;
	struct power_supply *dc_psy;
	struct power_supply *usb_psy;
#ifdef USE_MMI_CHARGER
#else
	struct votable	*usb_icl_votable;
	struct votable	*dc_suspend_votable;
	struct votable	*fcc_votable;
	struct votable	*chg_dis_votable;
#endif
	int	batt_capacity;
	struct	notifier_block ps_notif;
	struct	work_struct update;
	bool	init_success;
	bool	charging_suspended;
	bool	charging_stopped;
	struct delayed_work	reinit_work;
} adap_chg_data;

int upper_limit = -1;
int lower_limit = -1;
int blocking = 0;

#define ADAPTIVE_CHARGING_VOTER	"ADAPTIVE_CHARGING_VOTER"

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

/* Stop charging, no charging icon */
static void suspend_charging(bool on)
{
	if(!adap_chg_data.charging_suspended && on) {
#ifdef USE_MMI_CHARGER
		mmi_vote_charger_suspend(ADAPTIVE_CHARGING_VOTER, on);
#else
		vote(adap_chg_data.usb_icl_votable, ADAPTIVE_CHARGING_VOTER, true, 0);
		vote(adap_chg_data.dc_suspend_votable, ADAPTIVE_CHARGING_VOTER, true, 0);
#endif
		adap_chg_data.charging_suspended = true;
		pr_info("Set suspended on\n");
	}

	if(adap_chg_data.charging_suspended && !on) {
#ifdef USE_MMI_CHARGER
		mmi_vote_charger_suspend(ADAPTIVE_CHARGING_VOTER, on);
#else
		vote(adap_chg_data.usb_icl_votable, ADAPTIVE_CHARGING_VOTER, false, 0);
		vote(adap_chg_data.dc_suspend_votable, ADAPTIVE_CHARGING_VOTER, false, 0);
#endif
		adap_chg_data.charging_suspended = false;
		pr_info("Set suspended off");
	}
}

/* Limit charging to 0 mA, but keep charging icon, mimic what happens in charge_full state */
static void stop_charging(bool on)
{
	if(!adap_chg_data.charging_stopped && on) {
#ifdef USE_MMI_CHARGER
		mmi_vote_charging_disable(ADAPTIVE_CHARGING_VOTER, on);
#else
		vote(adap_chg_data.fcc_votable, ADAPTIVE_CHARGING_VOTER, true, 0);
		vote(adap_chg_data.chg_dis_votable, ADAPTIVE_CHARGING_VOTER, true, 0);
#endif
		adap_chg_data.charging_stopped = true;
		pr_info("Set stop charging on\n");
	}

	if(adap_chg_data.charging_stopped && !on) {
#ifdef USE_MMI_CHARGER
		mmi_vote_charging_disable(ADAPTIVE_CHARGING_VOTER, on);
#else
		vote(adap_chg_data.fcc_votable, ADAPTIVE_CHARGING_VOTER, false, 0);
		vote(adap_chg_data.chg_dis_votable, ADAPTIVE_CHARGING_VOTER, false, 0);
#endif
		adap_chg_data.charging_stopped = false;
		pr_info("Set stop charging off\n");
	}
}

static void update(struct adap_chg_data *data)
{
	data->batt_capacity = get_ps_int_prop(data->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY);
	if (data->batt_capacity < 0) {
		pr_err("Failed to get battery capacity\n");
		return;
	}

	pr_info("Batt cap is %d, upper limit is %d, lower limit is %d\n",
		data->batt_capacity, upper_limit, lower_limit);

	if (upper_limit != -1) {
		/* If no lower limit is defined, we are in Auto Mode */
		if (lower_limit == -1) {
#ifdef ADAPTIVE_TOLERANCE_OPTIMIZATION
			if (data->batt_capacity > (upper_limit + 2)) {
#else
                        if (data->batt_capacity > (upper_limit + 1)) {
#endif
				suspend_charging(true);
				stop_charging(false);
			} else if (data->batt_capacity == (upper_limit + 1)) {
				suspend_charging(false);
				stop_charging(true);
			} else if (data->batt_capacity < upper_limit) {
				suspend_charging(false);
				stop_charging(false);
			}
		/* Manual mode */
		} else {
			if (data->batt_capacity >= upper_limit) {
				suspend_charging(true);
				stop_charging(false);
			} else if (data->batt_capacity <= lower_limit) {
				suspend_charging(false);
				stop_charging(false);
			}
		}
	/* Disable */
	} else {
		suspend_charging(false);
		stop_charging(false);
	}
}

static void update_work(struct work_struct *w)
{
	struct adap_chg_data *data = container_of(w, struct adap_chg_data, update);
	int batt_capacity;

	batt_capacity = get_ps_int_prop(data->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY);
	if (batt_capacity < 0) {
		pr_err("Failed to get battery capacity\n");
		return;
	}

	if (batt_capacity != data->batt_capacity)
		update(data);
}

static int set_upper_limit(const char *val, const struct kernel_param *kp)
{
	int rc;
	long new_limit;

	if (!adap_chg_data.init_success)
		return -ENODEV;

	rc = kstrtol(val, 0, &new_limit);
	if (rc)
		return rc;

	/* If both lower is off then turn it off */
	if (lower_limit == -1 && new_limit == -1)
		upper_limit = new_limit;
	/* Else check that we are valid (between 0 and 100 and bigger than lower) */
	else if (new_limit > 0 && new_limit <= 100 && new_limit > lower_limit)
		upper_limit = new_limit;
	/* Else force to 100 since lower is set */
	else
		upper_limit = 100;

	pr_info("set upper limit to %d", upper_limit);
	update(&adap_chg_data);

	return 0;
}

static int set_lower_limit(const char *val, const struct kernel_param *kp)
{
	int rc;
	long new_limit;

	if (!adap_chg_data.init_success)
		return -ENODEV;

	rc = kstrtol(val, 0, &new_limit);
	if (rc)
		return rc;

	if (new_limit >= -1 && new_limit < upper_limit)
		lower_limit = new_limit;
	else
		lower_limit = -1;

	pr_info("set lower limit to %d", lower_limit);
	update(&adap_chg_data);

	return 0;
}

static int set_blocking(const char *val, const struct kernel_param *kp)
{
	return -EINVAL;
}

static int get_blocking(char *buffer, const struct kernel_param *kp)
{
	bool dc_connected = false;
	bool usb_connected = false;

	if (adap_chg_data.init_success) {
#ifdef USE_MMI_CHARGER
		usb_connected = (get_ps_int_prop(adap_chg_data.usb_psy,
			POWER_SUPPLY_PROP_ONLINE) > 0 ? true : false);

		dc_connected = (get_ps_int_prop(adap_chg_data.dc_psy,
			POWER_SUPPLY_PROP_ONLINE) > 0 ? true : false);
#else
		usb_connected = (get_ps_int_prop(adap_chg_data.usb_psy,
			POWER_SUPPLY_PROP_PRESENT) > 0 ? true : false);

		dc_connected = (get_ps_int_prop(adap_chg_data.dc_psy,
			POWER_SUPPLY_PROP_PRESENT) > 0 ? true : false);
#endif
		blocking = ((adap_chg_data.charging_suspended || adap_chg_data.charging_stopped) &&
			(dc_connected || usb_connected));
	} else
		blocking = 0;

	return scnprintf(buffer, PAGE_SIZE, "%d", blocking);
}

static int ps_notify_callback(struct notifier_block *nb,
		unsigned long event, void *p)
{
	struct adap_chg_data *data = container_of(nb, struct adap_chg_data, ps_notif);
	struct power_supply *psy = p;

	if (event == PSY_EVENT_PROP_CHANGED &&
			psy && psy->desc->get_property && psy->desc->name &&
			!strncmp(psy->desc->name, "battery", sizeof("battery"))) {
		if (upper_limit != -1)
			schedule_work(&data->update);
	}

	return 0;
}

static struct kernel_param_ops upper_limit_ops =
{
	.set = &set_upper_limit,
	.get = param_get_int,
};

static struct kernel_param_ops lower_limit_ops =
{
	.set = &set_lower_limit,
	.get = param_get_int,
};

static struct kernel_param_ops blocking_ops =
{
	.set = &set_blocking,
	.get = &get_blocking,
};

module_param_cb(upper_limit,
    &upper_limit_ops,
    &upper_limit,
    S_IRUGO | S_IWUSR
);

module_param_cb(lower_limit,
    &lower_limit_ops,
    &lower_limit,
    S_IRUGO | S_IWUSR
);

module_param_cb(blocking,
    &blocking_ops,
    &blocking,
    S_IRUGO
);

#define ADAP_INIT_SUCCESS 0
#define ADAP_INIT_FAILED 1
#define ADAP_INIT_RERUN 2
static int qpnp_adap_chg_init_work(void)
{
	upper_limit = -1;
	lower_limit = -1;
	adap_chg_data.init_success = false;
	adap_chg_data.charging_suspended = false;
	adap_chg_data.charging_stopped = false;

	adap_chg_data.batt_psy = power_supply_get_by_name("battery");
	if (!adap_chg_data.batt_psy) {
		pr_err("Failed to get battery power supply\n");
		goto psy_fail;
	}

	adap_chg_data.usb_psy = power_supply_get_by_name("usb");
	if (!adap_chg_data.usb_psy) {
		pr_err("Failed to get usb power supply\n");
		goto psy_fail;
	}

#ifdef USE_MMI_CHARGER
	adap_chg_data.dc_psy = power_supply_get_by_name("wireless");
	if (!adap_chg_data.dc_psy)
		pr_info("No wireless power supply found\n");
#else
	adap_chg_data.dc_psy = power_supply_get_by_name("dc");
	if (!adap_chg_data.dc_psy)
		pr_info("No dc power supply found\n");
#endif

	adap_chg_data.batt_capacity = get_ps_int_prop(adap_chg_data.batt_psy,
		POWER_SUPPLY_PROP_CAPACITY);
	if (adap_chg_data.batt_capacity < 0) {
		pr_err("Failed to get initial battery capacity\n");
		goto fail;
	}

#ifdef USE_MMI_CHARGER
#else
	adap_chg_data.usb_icl_votable = find_votable("USB_ICL");
	if (IS_ERR(adap_chg_data.usb_icl_votable)) {
		pr_err("Failed to get USB_ICL votable\n");
		goto fail;
	}

	adap_chg_data.dc_suspend_votable = find_votable("DC_SUSPEND");
	if (IS_ERR(adap_chg_data.dc_suspend_votable)) {
		pr_err("Failed to get DC_SUSPEND votable\n");
		goto fail;
	}

	adap_chg_data.fcc_votable = find_votable("FCC");
	if (IS_ERR(adap_chg_data.fcc_votable)) {
		pr_err("Failed to get FCC votable\n");
		goto fail;
	}

	adap_chg_data.chg_dis_votable = find_votable("CHG_DISABLE");
	if (IS_ERR(adap_chg_data.chg_dis_votable)) {
		pr_err("Failed to get CHG_DISABLE votable\n");
		goto fail;
	}
#endif

	adap_chg_data.ps_notif.notifier_call = ps_notify_callback;
	if (power_supply_reg_notifier(&adap_chg_data.ps_notif)) {
		pr_err("Failed to register notifier\n");
		goto fail;
	}

	INIT_WORK(&adap_chg_data.update, update_work);
	schedule_work(&adap_chg_data.update);

	adap_chg_data.init_success = true;
	return ADAP_INIT_SUCCESS;
fail:
	return ADAP_INIT_FAILED;
psy_fail:
	return ADAP_INIT_RERUN;

}

static void adap_reinit_work(struct work_struct *work)
{
	int ret = ADAP_INIT_FAILED;
	ret = qpnp_adap_chg_init_work();

	if (ret == ADAP_INIT_RERUN) {
		cancel_delayed_work(&adap_chg_data.reinit_work);
		schedule_delayed_work(&adap_chg_data.reinit_work,
					      msecs_to_jiffies(5000));
	}
}

static int __init qpnp_adap_chg_init(void)
{
	int ret = ADAP_INIT_FAILED;

	ret = qpnp_adap_chg_init_work();

	if (ret == ADAP_INIT_RERUN) {
		INIT_DELAYED_WORK(&adap_chg_data.reinit_work, adap_reinit_work);
		schedule_delayed_work(&adap_chg_data.reinit_work,
					      msecs_to_jiffies(5000));
		return ADAP_INIT_SUCCESS;
	} else
		return ret;
}

static void qpnp_adap_chg_exit(void)
{
	if(adap_chg_data.init_success) {
		cancel_work_sync(&adap_chg_data.update);
		power_supply_unreg_notifier(&adap_chg_data.ps_notif);
	}

	stop_charging(false);
	suspend_charging(false);
}

module_init(qpnp_adap_chg_init);
module_exit(qpnp_adap_chg_exit);

MODULE_DESCRIPTION("Motorola QPNP adaptive charging");
MODULE_LICENSE("GPL v2");
