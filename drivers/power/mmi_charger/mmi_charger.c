/*
 * Copyright (C) 2020 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/mmi_wake_lock.h>
#include <soc/qcom/mmi_boot_info.h>
#include <linux/time64.h>

#include "mmi_charger.h"

#define HYST_STEP_MV 50
#define DEMO_MODE_HYS_SOC 5
#define WARM_TEMP 45
#define COOL_TEMP 0

#define BATT_PAIR_ID_BITS 16
#define BATT_PAIR_ID_MASK ((1 << BATT_PAIR_ID_BITS) - 1)

#define HEARTBEAT_DELAY_MS 60000
#define HEARTBEAT_FACTORY_MS 1000
#define HEARTBEAT_DISCHARGE_MS 100000
#define HEARTBEAT_WAKEUP_INTRVAL_NS 70000000000

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for mmi charger driver");

static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);
static int suspend_wakeups;
module_param(suspend_wakeups, int, 0644);

static bool shutdown_triggered = false;

static struct mmi_charger_chip *this_chip = NULL;

enum {
	MMI_POWER_SUPPLY_CHARGE_RATE_NONE = 0,
	MMI_POWER_SUPPLY_CHARGE_RATE_NORMAL,
	MMI_POWER_SUPPLY_CHARGE_RATE_WEAK,
	MMI_POWER_SUPPLY_CHARGE_RATE_TURBO,
	MMI_POWER_SUPPLY_CHARGE_RATE_TURBO_30W,
	MMI_POWER_SUPPLY_CHARGE_RATE_HYPER,
};

enum {
	NOTIFY_EVENT_TYPE_CHG_RATE = 0,
	NOTIFY_EVENT_TYPE_LPD_PRESENT,
	NOTIFY_EVENT_TYPE_VBUS_PRESENT,
	NOTIFY_EVENT_TYPE_POWER_WATT,
};

static char *charge_rate[] = {
	"None", "Normal", "Weak", "Turbo", "Turbo_30W", "Hyper"
};

#define MAX_NUM_TEMP_ZONE 10
enum mmi_temp_zones {
	ZONE_FIRST = 0,
	/* states 0-9 are reserved for zones */
	ZONE_LAST = MAX_NUM_TEMP_ZONE + ZONE_FIRST - 1,
	ZONE_HOT,
	ZONE_COLD,
	ZONE_NONE,
};

struct mmi_temp_zone {
        int temp_c;
        int norm_mv;
        int fcc_max_ma;
        int fcc_norm_ma;
};

enum mmi_chrg_step {
	STEP_MAX,
	STEP_NORM,
	STEP_FULL,
	STEP_FLOAT,
	STEP_DEMO,
	STEP_STOP,
	STEP_NONE,
};

static char *stepchg_str[] = {
	[STEP_MAX]		= "MAX",
	[STEP_NORM]		= "NORMAL",
	[STEP_FULL]		= "FULL",
	[STEP_FLOAT]		= "FLOAT",
	[STEP_DEMO]		= "DEMO",
	[STEP_STOP]		= "STOP",
	[STEP_NONE]		= "NONE",
};

struct mmi_ffc_zone {
	int temp;
	int fv;
	int chrg_iterm;
	int fg_iterm;
};

enum charging_limit_modes {
	CHARGING_LIMIT_OFF,
	CHARGING_LIMIT_RUN,
	CHARGING_LIMIT_UNKNOWN,
};

struct mmi_battery_pack {
	int health;
	int status;
	int age;
	int cycles;
	int pending;
	int reference;
	int charger_rate;
	int soc_cycles_start;
	int paired_id;
	struct mmi_battery_pack *paired_batt;
	struct mmi_battery_info *info;
	char sn[MMI_BATT_SN_LEN];
	struct list_head list;
};

struct mmi_charger_profile {
	//for not FFC battery profile
	int noffc_fg_iterm;
	int noffc_chrg_iterm;
	int noffc_max_fv_mv;

	int shutdown_empty_vbat_mv;

	int fg_iterm;
        int chrg_iterm;
        int max_fv_mv;
	int max_fcc_ma;
        int vfloat_comp_mv;
        int demo_fv_mv;
        int num_temp_zones;
        struct mmi_temp_zone *temp_zones;
	int num_ffc_zones;
	struct mmi_ffc_zone *ffc_zones;
};

struct mmi_charger_status {
	int demo_full_soc;
	bool demo_chrg_suspend;
	struct mmi_temp_zone *temp_zone;
	enum mmi_temp_zones pres_temp_zone;
	enum mmi_chrg_step pres_chrg_step;
	enum charging_limit_modes charging_limit_modes;
};

struct mmi_charger {
	struct mmi_charger_profile profile;
	struct mmi_charger_status status;
	struct mmi_charger_driver *driver;
	struct mmi_battery_info batt_info;
	struct mmi_charger_info chg_info;
	struct mmi_charger_cfg cfg;
	struct mmi_charger_constraint constraint;
	struct mmi_battery_pack *battery;
	struct list_head list;
};

#define MMI_VOTE_NUM_MAX 32
struct mmi_vote {
	const char *name;
	int votes[MMI_VOTE_NUM_MAX];
	const char *voters[MMI_VOTE_NUM_MAX];
};

#define IS_SUSPENDED BIT(0)
#define WAS_SUSPENDED BIT(1)

struct mmi_charger_chip {
	char			*name;
	struct device		*dev;
	struct list_head	charger_list;
	struct mutex		charger_lock;
	struct list_head	battery_list;
	struct mutex		battery_lock;

	struct power_supply	*mmi_psy;
	struct power_supply	*batt_psy;
	char			*batt_uenvp[2];
	int			combo_cycles;
	int			combo_soc;
	int			combo_age;
	int			combo_voltage_mv;
	int			combo_current_ma;
	int			combo_status;
	int			combo_health;
	int			combo_temp;
	int			combo_charge_counter;
	int			charge_full;
	int			charge_full_design;
	int			init_cycles;
	int			max_charger_rate;
	bool			vbus_present;
	bool			lpd_present;
	int			power_watt;

	int			suspended;
	int			demo_mode;
	bool			factory_mode;
	bool			factory_version;
	bool			factory_kill_armed;
	bool			force_charger_disabled;
	bool			force_charging_enabled;

	int			dcp_pmax;
	int			hvdcp_pmax;
	int			pd_pmax;
	int			wls_pmax;
	int			max_chrg_temp;
	bool			enable_charging_limit;
        bool                    enable_factory_poweroff;
	bool			start_factory_kill_disabled;
	int			upper_limit_capacity;
	int			lower_limit_capacity;

	struct wakeup_source	*mmi_hb_wake_source;
	struct alarm		heartbeat_alarm;
	int			heartbeat_interval;
	struct notifier_block	mmi_reboot;
	struct notifier_block	mmi_psy_notifier;
	struct delayed_work	heartbeat_work;

	bool			*debug_enabled;
	void			*ipc_log;

	struct mmi_vote		suspend_charger_vote;
	struct mmi_vote		disable_charging_vote;
	uint32_t		factory_kill_debounce_ms;

	bool			empty_vbat_shutdown_triggered;

	int			heartbeat_dischg_ms;
	uint32_t		ibat_calc_alignment_time;
};

static int mmi_vote(struct mmi_vote *vote, const char *voter,
				bool enabled, int value)
{
	int i;
	int empty = -EINVAL;
	struct mmi_charger_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	if (!voter) {
		mmi_err(chip, "%s: Invalid voter\n", vote->name);
		return -EINVAL;
	}

	for (i = 0; i < MMI_VOTE_NUM_MAX; i++) {
		if (vote->voters[i] &&
		    !strcmp(vote->voters[i], voter)) {
			break;
		} else if (empty < 0 && !vote->voters[i]) {
			empty = i;
		}
	}

	if (i < MMI_VOTE_NUM_MAX) {
		if (enabled) {
			vote->voters[i] = voter;
			vote->votes[i] = value;
		} else {
			vote->voters[i] = NULL;
			vote->votes[i] = 0;
		}
	} else {
		if (!enabled) {
			return 0;
		} else if (empty < 0) {
			mmi_err(this_chip, "No entry found\n");
			return -ENOENT;
		} else {
			vote->voters[empty] = voter;
			vote->votes[empty] = value;
		}
	}

	mmi_info(chip, "%s:%s voter: en:%d, val:%d\n",
			vote->name, voter, enabled, value);

	return 0;
}

static int mmi_get_effective_voter(struct mmi_vote *vote)
{
	int i;
	int win_voter = -EINVAL;
	int win_vote = INT_MAX;

	for (i = 0; i < MMI_VOTE_NUM_MAX; i++) {
		if (vote->voters[i] &&
		    vote->votes[i] < win_vote) {
			win_voter = i;
			win_vote = vote->votes[i];
		}
	}

	return win_voter;
}

static void mmi_notify_charger_event(struct mmi_charger_chip *chip, int type);
static ssize_t state_sync_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(this_chip, "Invalid state_sync value = %lu\n", mode);
		return -EINVAL;
	}

	if (mode) {
		mutex_lock(&this_chip->charger_lock);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_CHG_RATE);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_LPD_PRESENT);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_VBUS_PRESENT);
		mmi_notify_charger_event(this_chip,
					NOTIFY_EVENT_TYPE_POWER_WATT);
		mutex_unlock(&this_chip->charger_lock);
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
					msecs_to_jiffies(0));
		mmi_info(this_chip, "charger state sync received\n");
	}

	return count;
}
static DEVICE_ATTR(state_sync, 0200, NULL, state_sync_store);

#define CHARGER_POWER_5W 5000
#define CHARGER_POWER_7P5W 7500
#define CHARGER_POWER_10W 10000
#define CHARGER_POWER_15W 15000
#define CHARGER_POWER_18W 18000
#define CHARGER_POWER_20W 20000
#define CHARGER_POWER_MAX 100000
static ssize_t dcp_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid dcp pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->dcp_pmax != pmax &&
	    (pmax >= CHARGER_POWER_5W && pmax <= CHARGER_POWER_10W)) {
		this_chip->dcp_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t dcp_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->dcp_pmax);
}

static DEVICE_ATTR(dcp_pmax, 0644,
		dcp_pmax_show,
		dcp_pmax_store);

static ssize_t hvdcp_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid hvdcp pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->hvdcp_pmax != pmax &&
	    (pmax >= CHARGER_POWER_7P5W && pmax <= CHARGER_POWER_MAX)) {
		this_chip->hvdcp_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t hvdcp_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->hvdcp_pmax);
}

static DEVICE_ATTR(hvdcp_pmax, 0644,
		hvdcp_pmax_show,
		hvdcp_pmax_store);

static ssize_t pd_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid pd pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->pd_pmax != pmax &&
	    (pmax >= CHARGER_POWER_15W && pmax <= CHARGER_POWER_MAX)) {
		this_chip->pd_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t pd_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->pd_pmax);
}

static DEVICE_ATTR(pd_pmax, 0644,
		pd_pmax_show,
		pd_pmax_store);

static ssize_t wls_pmax_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmax;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmax);
	if (r) {
		mmi_err(this_chip, "Invalid wireless pmax value = %lu\n", pmax);
		return -EINVAL;
	}

	if (this_chip->wls_pmax != pmax &&
	    (pmax >= CHARGER_POWER_5W && pmax <= CHARGER_POWER_MAX)) {
		this_chip->wls_pmax = pmax;
		cancel_delayed_work(&this_chip->heartbeat_work);
		schedule_delayed_work(&this_chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return r ? r : count;
}

static ssize_t wls_pmax_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->wls_pmax);
}

static DEVICE_ATTR(wls_pmax, 0644,
		wls_pmax_show,
		wls_pmax_store);

static ssize_t factory_image_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(this_chip, "Invalid factory image mode value = %lu\n", mode);
		return -EINVAL;
	}

	this_chip->factory_version = (mode) ? true : false;

	return r ? r : count;
}

static ssize_t factory_image_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	state = (this_chip->factory_version) ? 1 : 0;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_image_mode, 0644,
		factory_image_mode_show,
		factory_image_mode_store);

static ssize_t factory_charge_upper_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	state = this_chip->upper_limit_capacity;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_charge_upper, 0444,
		factory_charge_upper_show,
		NULL);

#define MMI_CHIP_MODE_LOWER_LIMIT 35
#define MMI_CHIP_MODE_UPPER_LIMIT 80
static ssize_t force_demo_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(this_chip, "Invalid demo mode value = %lu\n", mode);
		return -EINVAL;
	}

	if ((mode >= MMI_CHIP_MODE_LOWER_LIMIT) &&
	    (mode <= MMI_CHIP_MODE_UPPER_LIMIT))
		this_chip->demo_mode = mode;
	else
		this_chip->demo_mode = MMI_CHIP_MODE_LOWER_LIMIT;

	return r ? r : count;
}

static ssize_t force_demo_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	state = this_chip->demo_mode;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_demo_mode, 0644,
		force_demo_mode_show,
		force_demo_mode_store);

#define MIN_TEMP_C -20
#define MAX_TEMP_C 60
#define MIN_MAX_TEMP_C 47
#define HYSTERESIS_DEGC 2
static ssize_t force_max_chrg_temp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(this_chip, "Invalid max temp value = %lu\n", mode);
		return -EINVAL;
	}

	if ((mode >= MIN_MAX_TEMP_C) && (mode <= MAX_TEMP_C))
		this_chip->max_chrg_temp = mode;
	else
		this_chip->max_chrg_temp = MAX_TEMP_C;

	return r ? r : count;
}

static ssize_t force_max_chrg_temp_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	state = this_chip->max_chrg_temp;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_max_chrg_temp, 0644,
		force_max_chrg_temp_show,
		force_max_chrg_temp_store);

static ssize_t charge_rate_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n",
			 charge_rate[this_chip->max_charger_rate]);
}
static DEVICE_ATTR(charge_rate, S_IRUGO, charge_rate_show, NULL);

static ssize_t age_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	if (!this_chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", this_chip->combo_age);
}
static DEVICE_ATTR(age, S_IRUGO, age_show, NULL);

static ssize_t force_charger_suspend_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int state;

	if (!this_chip) {
		pr_err("mmi_charger: chip not valid\n");
		return -ENODEV;
	}

	state = this_chip->force_charger_disabled;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static ssize_t force_charger_suspend_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("mmi_charger: Invalid charger suspend value = %lu\n", mode);
		return -EINVAL;
	}

	this_chip->force_charger_disabled = (mode) ? true : false;
	cancel_delayed_work(&this_chip->heartbeat_work);
	schedule_delayed_work(&this_chip->heartbeat_work, msecs_to_jiffies(0));
	mmi_info(this_chip, "%s force_charger_disabled\n", (mode)? "set" : "clear");

	return count;
}

static DEVICE_ATTR(force_charger_suspend, 0644,
		force_charger_suspend_show,
		force_charger_suspend_store);

static ssize_t force_charging_enable_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int state;

	if (!this_chip) {
		pr_err("mmi_charger: chip not valid\n");
		return -ENODEV;
	}

	state = this_chip->force_charging_enabled;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static ssize_t force_charging_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("mmi_charger: Invalid charger enable value = %lu\n", mode);
		return -EINVAL;
	}

	this_chip->force_charging_enabled = (mode) ? true : false;
	cancel_delayed_work(&this_chip->heartbeat_work);
	schedule_delayed_work(&this_chip->heartbeat_work, msecs_to_jiffies(0));
	mmi_info(this_chip, "%s force_charging_enabled\n", (mode)? "set" : "clear");

	return count;
}

static DEVICE_ATTR(force_charging_enable, 0644,
		force_charging_enable_show,
		force_charging_enable_store);

static ssize_t force_charging_disable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	if (!this_chip) {
		pr_err("mmi_charger: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("mmi_charger: Invalid charger disable value = %lu\n", mode);
		return -EINVAL;
	}

	mmi_vote_charging_disable("MMI_USER", !!mode);
	cancel_delayed_work(&this_chip->heartbeat_work);
	schedule_delayed_work(&this_chip->heartbeat_work, msecs_to_jiffies(0));
	mmi_info(this_chip, "%s force_charging_disable\n", (mode)? "set" : "clear");

	return count;
}

static DEVICE_ATTR(force_charging_disable, 0200,
		NULL,
		force_charging_disable_store);

static struct attribute * mmi_g[] = {
	&dev_attr_charge_rate.attr,
	&dev_attr_age.attr,
	NULL,
};

static const struct attribute_group power_supply_mmi_attr_group = {
	.attrs = mmi_g,
};

static void mmi_battery_supply_init(struct mmi_charger_chip *chip)
{
	int rc;

	if (chip->batt_psy)
		return;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		mmi_err(chip, "No battery supply found\n");
		return;
	}

	rc = sysfs_create_group(&chip->batt_psy->dev.kobj,
				&power_supply_mmi_attr_group);
	if (rc)
		mmi_err(chip, "failed: attr create\n");

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_force_demo_mode);
	if (rc) {
		mmi_err(chip, "couldn't create force_demo_mode\n");
	}

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_force_max_chrg_temp);
	if (rc) {
		mmi_err(chip, "couldn't create force_max_chrg_temp\n");
	}

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_factory_image_mode);
	if (rc) {
		mmi_err(chip, "couldn't create factory_image_mode\n");
	}

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_factory_charge_upper);
	if (rc)
		mmi_err(chip, "couldn't create factory_charge_upper\n");

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_force_charger_suspend);
	if (rc)
		mmi_err(chip, "couldn't create force_charger_suspend\n");

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_force_charging_enable);
	if (rc)
		mmi_err(chip, "couldn't create force_charging_enable\n");

	rc = device_create_file(chip->batt_psy->dev.parent,
				&dev_attr_force_charging_disable);
	if (rc)
		mmi_err(chip, "couldn't create force_charging_disable\n");

	mmi_info(chip, "battery supply is initialized\n");
}

static int mmi_get_charger_profile(struct mmi_charger_chip *chip,
				struct mmi_charger *charger)
{
	int rc;
	int i;
	int byte_len;
	struct device_node *node;

	if (!charger->driver || !charger->driver->dev) {
		mmi_err(chip, "mmi charger driver is invalid\n");
		return -EINVAL;
	}

	node = charger->driver->dev->of_node;
	rc = of_property_read_u32(node, "mmi,shutdown-empty-vbat-mv",
				  &charger->profile.shutdown_empty_vbat_mv);
	if (rc)
		charger->profile.shutdown_empty_vbat_mv = -EINVAL;

	rc = of_property_read_u32(node, "mmi,chrg-iterm-ma",
				  &charger->profile.chrg_iterm);
	if (rc)
		charger->profile.chrg_iterm = 300;

	charger->profile.noffc_chrg_iterm = charger->profile.chrg_iterm;

	rc = of_property_read_u32(node, "mmi,fg-iterm-ma",
				  &charger->profile.fg_iterm);
	if (rc)
		charger->profile.fg_iterm = charger->profile.chrg_iterm + 50;

	charger->profile.noffc_fg_iterm = charger->profile.fg_iterm;

	rc = of_property_read_u32(node, "mmi,vfloat-comp-uv",
				  &charger->profile.vfloat_comp_mv);
	if (rc)
		charger->profile.vfloat_comp_mv = 0;
	charger->profile.vfloat_comp_mv /= 1000;

	rc = of_property_read_u32(node, "mmi,max-fv-mv",
				  &charger->profile.max_fv_mv);
	if (rc)
		charger->profile.max_fv_mv = 4400;

	charger->profile.noffc_max_fv_mv = charger->profile.max_fv_mv;

	rc = of_property_read_u32(node, "mmi,max-fcc-ma",
				  &charger->profile.max_fcc_ma);
	if (rc)
		charger->profile.max_fcc_ma = 4400;

	rc = of_property_read_u32(node, "mmi,demo-fv-mv",
				  &charger->profile.demo_fv_mv);
	if (rc)
		charger->profile.demo_fv_mv = 4000;

	if (of_find_property(node, "mmi,mmi-temp-zones", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			mmi_err(chip, "[C:%s]: DT error wrong mmi temp zones\n",
					charger->driver->name);
			return -ENODEV;
		}

		charger->profile.temp_zones = (struct mmi_temp_zone *)
			devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);

		if (charger->profile.temp_zones == NULL)
			return -ENOMEM;

		charger->profile.num_temp_zones =
			byte_len / sizeof(struct mmi_temp_zone);

		rc = of_property_read_u32_array(node,
				"mmi,mmi-temp-zones",
				(u32 *)charger->profile.temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_err(chip, "[C:%s]: Couldn't read mmi temp zones rc = %d\n",
					charger->driver->name, rc);
			devm_kfree(chip->dev, charger->profile.temp_zones);
			charger->profile.temp_zones = NULL;
			return rc;
		}

		mmi_info(chip, "[C:%s]: mmi temp zones: Num: %d\n",
				charger->driver->name,
				charger->profile.num_temp_zones);
		for (i = 0; i < charger->profile.num_temp_zones; i++) {
			mmi_info(chip, "[C:%s]: mmi temp zones: Zone %d, Temp %d C, " \
				"Step Volt %d mV, Full Rate %d mA, " \
				"Taper Rate %d mA\n", charger->driver->name, i,
				charger->profile.temp_zones[i].temp_c,
				charger->profile.temp_zones[i].norm_mv,
				charger->profile.temp_zones[i].fcc_max_ma,
				charger->profile.temp_zones[i].fcc_norm_ma);
		}
	}

	if (of_find_property(node, "mmi,mmi-ffc-zones", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 4) {
			mmi_err(chip, "[C:%s]: DT error wrong mmi ffc zones\n",
					charger->driver->name);
			return -ENODEV;
		}

		charger->profile.ffc_zones = (struct mmi_ffc_zone *)
			devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);

		if (charger->profile.ffc_zones == NULL)
			return -ENOMEM;

		charger->profile.num_ffc_zones =
			byte_len / sizeof(struct mmi_ffc_zone);

		rc = of_property_read_u32_array(node,
				"mmi,mmi-ffc-zones",
				(u32 *)charger->profile.ffc_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_err(chip, "[C:%s]: Couldn't read mmi ffc zones rc = %d\n",
					charger->driver->name, rc);
			devm_kfree(chip->dev, charger->profile.ffc_zones);
			charger->profile.ffc_zones = NULL;
			return rc;
		}

		mmi_info(chip, "[C:%s]: mmi ffc zones: Num: %d\n",
				charger->driver->name,
				charger->profile.num_ffc_zones);
		for (i = 0; i < charger->profile.num_ffc_zones; i++) {
			mmi_info(chip, "[C:%s]: mmi ffc zones: Zone %d, Temp %d C, " \
				"FV %d mV, Chg Iterm %d mA, Fg Iterm %d mA\n",
				charger->driver->name, i,
				charger->profile.ffc_zones[i].temp,
				charger->profile.ffc_zones[i].fv,
				charger->profile.ffc_zones[i].chrg_iterm,
				charger->profile.ffc_zones[i].fg_iterm);
		}
	}

	return 0;
}

#define TURBO_CHRG_FFC_THRSH_MW 25000
static void mmi_update_charger_profile(struct mmi_charger_chip *chip,
			       struct mmi_charger *charger)
{
	int i = 0;
	int temp;
	int num_zones;
	struct mmi_ffc_zone *zones;
	struct mmi_charger_info *chg_info = &charger->chg_info;

	if (!chip) {
		pr_err("called before chg valid!\n");
		return;
	}

	if (!(chg_info->chrg_pmax_mw > TURBO_CHRG_FFC_THRSH_MW)) {
		charger->profile.max_fv_mv = charger->profile.noffc_max_fv_mv;
		charger->profile.fg_iterm = charger->profile.noffc_fg_iterm;
		charger->profile.chrg_iterm = charger->profile.noffc_chrg_iterm;
		return;
	}

	temp = charger->batt_info.batt_temp;
	num_zones = charger->profile.num_ffc_zones;
	zones = charger->profile.ffc_zones;
	while (i < num_zones && temp > zones[i++].temp);
	zones = i > 0? &zones[i - 1] : NULL;
	if (zones) {
		charger->profile.max_fv_mv = zones->fv;
		charger->profile.fg_iterm = zones->fg_iterm;
		charger->profile.chrg_iterm = zones->chrg_iterm;
		mmi_dbg(chip, "[C:%s]: Max FV:%d, FG Iterm:%d, Chrg Iterm:%d\n",
			charger->driver->name,
			charger->profile.max_fv_mv,
			charger->profile.fg_iterm,
			charger->profile.chrg_iterm);
	}
}

static int mmi_find_colder_temp_zone(int pres_zone, int vbat,
				struct mmi_temp_zone *zones,
				int num_zones)
{
	int i;
	int colder_zone;
	int target_zone;

	if (pres_zone == ZONE_HOT)
		colder_zone = num_zones - 1;
	else if (pres_zone == ZONE_COLD ||
 	  zones[pres_zone].temp_c == zones[ZONE_FIRST].temp_c)
		return ZONE_COLD;
	else {
		for (i = pres_zone - 1; i >= ZONE_FIRST; i--) {
			if (zones[pres_zone].temp_c > zones[i].temp_c) {
				colder_zone = i;
				break;
			}
		}
		if (i < 0)
			return ZONE_COLD;
	}

	target_zone = colder_zone;
	for (i = ZONE_FIRST; i < colder_zone; i++) {
		if (zones[colder_zone].temp_c == zones[i].temp_c) {
			target_zone = i;
			if (vbat < zones[i].norm_mv)
				break;
		}
	}

	return target_zone;
}

static int mmi_find_hotter_temp_zone(int pres_zone, int vbat,
				struct mmi_temp_zone *zones,
				int num_zones)
{
	int i;
	int hotter_zone;
	int target_zone;

	if (pres_zone == ZONE_COLD)
		hotter_zone = ZONE_FIRST;
	else if (pres_zone == ZONE_HOT ||
	    zones[pres_zone].temp_c == zones[num_zones - 1].temp_c)
		return ZONE_HOT;
	else {
		for (i = pres_zone + 1; i < num_zones; i++) {
			if (zones[pres_zone].temp_c < zones[i].temp_c) {
				hotter_zone = i;
				break;
			}
		}
		if (i >= num_zones)
			return ZONE_HOT;
	}

	target_zone = hotter_zone;
	for (i = hotter_zone; i < num_zones; i++) {
		if (zones[hotter_zone].temp_c == zones[i].temp_c) {
			target_zone = i;
			if (vbat < zones[i].norm_mv)
				break;
		}
	}
	return target_zone;
}

static int mmi_refresh_temp_zone(int pres_zone, int vbat,
				struct mmi_temp_zone *zones,
				int num_zones)
{
	int i;
	int target_zone;

	if (pres_zone == ZONE_COLD || pres_zone == ZONE_HOT)
		return pres_zone;

	target_zone = pres_zone;
	for (i = ZONE_FIRST; i < num_zones; i++) {
		if (zones[pres_zone].temp_c == zones[i].temp_c) {
			target_zone = i;
			if (vbat < zones[i].norm_mv)
				break;
		}
	}
	return target_zone;
}

static void mmi_get_temp_zone(struct mmi_charger_chip *chip,
			       struct mmi_charger *charger)
{
	int i;
	int temp_c;
	int vbat_mv;
	int max_temp;
	int prev_zone, num_zones;
	int hotter_zone, colder_zone;
	struct mmi_temp_zone *zones;
	int hotter_t, hotter_fcc;
	int colder_t, colder_fcc;

	if (!chip) {
		pr_err("called before chg valid!\n");
		return;
	}

	temp_c = charger->batt_info.batt_temp;
	vbat_mv = charger->batt_info.batt_mv;
	prev_zone = charger->status.pres_temp_zone;
	num_zones = charger->profile.num_temp_zones;
	if (!charger->profile.temp_zones) {
		zones = NULL;
		num_zones = 0;
		if (chip->max_chrg_temp >= MIN_MAX_TEMP_C)
			max_temp = chip->max_chrg_temp;
		else
			max_temp = MAX_TEMP_C;
	} else {
		zones = charger->profile.temp_zones;
		if (chip->max_chrg_temp >= MIN_MAX_TEMP_C)
			max_temp = chip->max_chrg_temp;
		else
			max_temp = zones[num_zones - 1].temp_c;
	}

	if (prev_zone == ZONE_NONE && zones) {
		for (i = num_zones - 1; i >= 0; i--) {
			if (temp_c >= zones[i].temp_c) {
				charger->status.pres_temp_zone =
					mmi_find_hotter_temp_zone(i,
							vbat_mv,
							zones,
							num_zones);
				goto exit;
			}
		}
		if (temp_c < MIN_TEMP_C)
			charger->status.pres_temp_zone = ZONE_COLD;
		else
			charger->status.pres_temp_zone =
					mmi_find_hotter_temp_zone(ZONE_COLD,
							vbat_mv,
							zones,
							num_zones);
		goto exit;
	}

	if (prev_zone == ZONE_COLD) {
		if (temp_c >= MIN_TEMP_C + HYSTERESIS_DEGC) {
			if (!num_zones)
				charger->status.pres_temp_zone = ZONE_FIRST;
			else
				charger->status.pres_temp_zone =
					mmi_find_hotter_temp_zone(prev_zone,
							vbat_mv,
							zones,
							num_zones);
		}
	} else if (prev_zone == ZONE_HOT) {
		if (temp_c <=  max_temp - HYSTERESIS_DEGC) {
			if (!num_zones)
				charger->status.pres_temp_zone = ZONE_FIRST;
			else
				charger->status.pres_temp_zone =
					mmi_find_colder_temp_zone(prev_zone,
							vbat_mv,
							zones,
							num_zones);
		}
	} else if (zones) {
		hotter_zone = mmi_find_hotter_temp_zone(prev_zone,
						vbat_mv,
						zones,
						num_zones);
		colder_zone = mmi_find_colder_temp_zone(prev_zone,
						vbat_mv,
						zones,
						num_zones);
		if (hotter_zone == ZONE_HOT) {
			hotter_fcc = 0;
			hotter_t = zones[prev_zone].temp_c;
		} else {
			hotter_fcc = zones[hotter_zone].fcc_max_ma;
			hotter_t = zones[prev_zone].temp_c;
		}

		if (colder_zone == ZONE_COLD) {
			colder_fcc = 0;
			colder_t = MIN_TEMP_C;
		} else {
			colder_fcc = zones[colder_zone].fcc_max_ma;
			colder_t = zones[colder_zone].temp_c;
		}

		if (zones[prev_zone].fcc_max_ma < hotter_fcc)
			hotter_t += HYSTERESIS_DEGC;

		if (zones[prev_zone].fcc_max_ma < colder_fcc)
			colder_t -= HYSTERESIS_DEGC;

		if (temp_c < MIN_TEMP_C)
			charger->status.pres_temp_zone = ZONE_COLD;
		else if (temp_c >= max_temp)
			charger->status.pres_temp_zone = ZONE_HOT;
		else if (temp_c >= hotter_t)
			charger->status.pres_temp_zone = hotter_zone;
		else if (temp_c < colder_t)
			charger->status.pres_temp_zone = colder_zone;
		else
			charger->status.pres_temp_zone =
					mmi_refresh_temp_zone(prev_zone,
							vbat_mv,
							zones,
							num_zones);
	} else {
		if (temp_c < MIN_TEMP_C)
			charger->status.pres_temp_zone = ZONE_COLD;
		else if (temp_c >= max_temp)
			charger->status.pres_temp_zone = ZONE_HOT;
		else
			charger->status.pres_temp_zone = ZONE_FIRST;
	}

exit:
	if (!zones ||
	    charger->status.pres_temp_zone == ZONE_COLD ||
	    charger->status.pres_temp_zone == ZONE_HOT) {
		charger->status.temp_zone = NULL;
	} else {
		charger->status.temp_zone = &zones[charger->status.pres_temp_zone];
	}
	if (prev_zone != charger->status.pres_temp_zone) {
		mmi_info(chip, "[C:%s]: temp zone switch %x -> %x\n",
			charger->driver->name,
			prev_zone,
			charger->status.pres_temp_zone);
	}
}

static void mmi_update_paired_battery(
				struct mmi_charger_chip *chip,
				struct mmi_battery_pack *battery)
{
	struct mmi_battery_pack *batt = NULL;

	if (!battery->paired_id) {
		battery->paired_batt = NULL;
		return;
	}

	list_for_each_entry(batt, &chip->battery_list, list) {
		if ((batt->paired_id >> BATT_PAIR_ID_BITS) ==
			(battery->paired_id & BATT_PAIR_ID_MASK)) {
			battery->paired_batt = batt;
		}
		if ((battery->paired_id >> BATT_PAIR_ID_BITS) ==
			(batt->paired_id & BATT_PAIR_ID_MASK)) {
			batt->paired_batt = battery;
		}
	}
}

static void mmi_notify_paired_battery(struct mmi_charger *charger)
{
	if (!charger->battery->paired_batt ||
	    !charger->driver->notify_paired_battery)
		return;

	charger->driver->notify_paired_battery(charger->driver->data,
			charger->battery->paired_batt->info);
}

static int mmi_get_cur_thermal_level(struct mmi_charger_chip *chip, int *val)
{
	union power_supply_propval prop;
	int ret;
	if (!chip->batt_psy) {
		mmi_err(chip, "No battery supply found\n");
		return -ENODEV;
	}

	ret = power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &prop);
	if (!ret)
		*val = prop.intval;

	return ret;
}

static void mmi_get_charger_info(struct mmi_charger_chip *chip,
				struct mmi_charger *charger)
{
	struct mmi_battery_info *batt_info = &charger->batt_info;
	struct mmi_charger_info *chg_info = &charger->chg_info;
	int thermal_level = 0;

	mmi_get_cur_thermal_level(chip, &thermal_level);
	charger->driver->get_batt_info(charger->driver->data, batt_info);
	charger->driver->get_chg_info(charger->driver->data, chg_info);
	mmi_info(chip, "[C:%s]: batt_mv %d, batt_ma %d, batt_soc %d,"
		" batt_temp %d, batt_status %d, batt_sn %s, batt_fv_mv %d,"
		" batt_fcc_ma %d\n",
		charger->driver->name,
		batt_info->batt_mv,
		batt_info->batt_ma,
		batt_info->batt_soc,
		batt_info->batt_temp,
		batt_info->batt_status,
		batt_info->batt_sn,
		batt_info->batt_fv_mv,
		batt_info->batt_fcc_ma);
	mmi_info(chip, "[C:%s]: chrg_present %d, chrg_type %d, chrg_pmax_mw %d,"
		" chrg_mv %d, chrg_ma %d, chrg_otg_enabled %d, thermal_level %d\n",
		charger->driver->name,
		chg_info->chrg_present,
		chg_info->chrg_type,
		chg_info->chrg_pmax_mw,
		chg_info->chrg_mv,
		chg_info->chrg_ma,
		chg_info->chrg_otg_enabled,
		thermal_level);

}

static void mmi_update_charger_status(struct mmi_charger_chip *chip,
				struct mmi_charger *charger)
{
	bool voltage_full;
	int stop_recharge_hyst;
	enum charging_limit_modes charging_limit_modes;
	struct mmi_charger_profile *profile = &charger->profile;
	struct mmi_charger_status *status = &charger->status;
	struct mmi_battery_info *batt_info = &charger->batt_info;
	struct mmi_charger_info *chg_info = &charger->chg_info;
	struct mmi_charger_cfg *cfg = &charger->cfg;

	if (profile->shutdown_empty_vbat_mv > 0 &&
	    profile->shutdown_empty_vbat_mv >= batt_info->batt_mv) {
		mmi_err(chip, "[C:%s]: trigger shutdown, vbat=%d, empty_vbat=%d\n",
				charger->driver->name,
				batt_info->batt_mv,
				profile->shutdown_empty_vbat_mv);
		chip->empty_vbat_shutdown_triggered = true;
	}

	if (chip->enable_charging_limit && chip->factory_version) {
		charging_limit_modes = status->charging_limit_modes;
		if ((charging_limit_modes != CHARGING_LIMIT_RUN)
		    && (batt_info->batt_soc >= chip->upper_limit_capacity))
			charging_limit_modes = CHARGING_LIMIT_RUN;
		else if ((charging_limit_modes != CHARGING_LIMIT_OFF)
			   && (batt_info->batt_soc <= chip->lower_limit_capacity))
			charging_limit_modes = CHARGING_LIMIT_OFF;

		if (charging_limit_modes != charger->status.charging_limit_modes) {
			status->charging_limit_modes = charging_limit_modes;
			if (status->charging_limit_modes == CHARGING_LIMIT_RUN)
				mmi_warn(chip, "Factory Mode/Image so Limiting Charging!!!\n");
		}
	}

	mmi_get_temp_zone(chip, charger);
	if (!chg_info->chrg_present) {
		status->pres_chrg_step = STEP_NONE;
	} else if ((status->pres_temp_zone == ZONE_HOT) ||
		   (status->pres_temp_zone == ZONE_COLD) ||
		   (status->charging_limit_modes == CHARGING_LIMIT_RUN)) {
		status->pres_chrg_step = STEP_STOP;
	} else if (chip->demo_mode) { /* Demo Mode */
		status->pres_chrg_step = STEP_DEMO;
		voltage_full = ((status->demo_chrg_suspend == false) &&
		    ((batt_info->batt_mv + HYST_STEP_MV) >= profile->demo_fv_mv) &&
		    charger->driver->is_charge_tapered(charger->driver->data, profile->chrg_iterm));

		if ((status->demo_chrg_suspend == false) &&
		    ((batt_info->batt_soc >= chip->demo_mode) || voltage_full)) {
			status->demo_full_soc = batt_info->batt_soc;
			status->demo_chrg_suspend = true;
		} else if (status->demo_chrg_suspend == true &&
		    (batt_info->batt_soc <= (status->demo_full_soc - DEMO_MODE_HYS_SOC))) {
			status->demo_chrg_suspend = false;
			cfg->taper_kickoff = true;
		}
	} else if (!status->temp_zone) {
		status->pres_chrg_step = STEP_MAX;
		/* Skip for empty temperature zone */
	} else if (status->pres_chrg_step == STEP_NONE) {
		if (status->temp_zone->norm_mv &&
		    ((batt_info->batt_mv + HYST_STEP_MV) >= status->temp_zone->norm_mv)) {
			if (status->temp_zone->fcc_norm_ma)
				status->pres_chrg_step = STEP_NORM;
			else
				status->pres_chrg_step = STEP_STOP;
		} else
			status->pres_chrg_step = STEP_MAX;
	} else if (status->pres_chrg_step == STEP_STOP) {
		if (batt_info->batt_temp > COOL_TEMP)
			stop_recharge_hyst = 2 * HYST_STEP_MV;
		else
			stop_recharge_hyst = 5 * HYST_STEP_MV;
		if (status->temp_zone->norm_mv &&
			((batt_info->batt_mv + stop_recharge_hyst) >= status->temp_zone->norm_mv)) {
			if (status->temp_zone->fcc_norm_ma)
				status->pres_chrg_step = STEP_NORM;
			else
				status->pres_chrg_step = STEP_STOP;
		} else
			status->pres_chrg_step = STEP_MAX;
	} else if (status->pres_chrg_step == STEP_MAX) {
		if (!status->temp_zone->norm_mv) {
			/* No Step in this Zone */
			cfg->taper_kickoff = true;
			if ((batt_info->batt_mv + HYST_STEP_MV) >= profile->max_fv_mv)
				status->pres_chrg_step = STEP_NORM;
			else
				status->pres_chrg_step = STEP_MAX;
		} else if ((batt_info->batt_mv + HYST_STEP_MV) < status->temp_zone->norm_mv) {
			cfg->taper_kickoff = true;
			status->pres_chrg_step = STEP_MAX;
		} else if (!status->temp_zone->fcc_norm_ma)
			status->pres_chrg_step = STEP_FLOAT;
		else if (charger->driver->is_charge_tapered(charger->driver->data, status->temp_zone->fcc_norm_ma)) {
			cfg->taper_kickoff = true;
			if (charger->driver->is_charge_halt &&
			    charger->driver->is_charge_halt(charger->driver->data)) {
				cfg->charging_reset = true;
				mmi_warn(chip, "[C:%s]: Charge Halt..Toggle\n",
							charger->driver->name);
			}
			status->pres_chrg_step = STEP_NORM;
		}
	} else if (status->pres_chrg_step == STEP_NORM) {
		if (!status->temp_zone->fcc_norm_ma)
			status->pres_chrg_step = STEP_FLOAT;
		else if ((batt_info->batt_soc < 100) ||
			 (batt_info->batt_mv + HYST_STEP_MV) < profile->max_fv_mv) {
			cfg->taper_kickoff = true;
			status->pres_chrg_step = STEP_NORM;
		} else if (charger->driver->is_charge_tapered(charger->driver->data, profile->chrg_iterm)) {
				status->pres_chrg_step = STEP_FULL;
		}
	} else if (status->pres_chrg_step == STEP_FULL) {
#ifdef CONFIG_MMI_RECHARGER_HAWAO_MODE
		if ((batt_info->batt_soc <= 98) ||
			batt_info->batt_mv < (profile->max_fv_mv - 100 * 2))
#else
		if ((batt_info->batt_soc <= 99) ||
			batt_info->batt_mv < (profile->max_fv_mv - HYST_STEP_MV * 2))
#endif
		{
			cfg->taper_kickoff = true;
			status->pres_chrg_step = STEP_NORM;
		}
	} else if (status->pres_chrg_step == STEP_FLOAT) {
		if ((status->temp_zone->fcc_norm_ma) ||
		    ((batt_info->batt_mv + HYST_STEP_MV) < status->temp_zone->norm_mv))
			status->pres_chrg_step = STEP_MAX;
		else if (charger->driver->is_charge_tapered(charger->driver->data, profile->chrg_iterm))
			status->pres_chrg_step = STEP_STOP;
	}

	charger->battery->pending++;

	mmi_info(chip, "[C:%s]: StepChg: %s, TempZone: %d, LimitMode: %d, DemoSuspend: %d\n",
		charger->driver->name,
		stepchg_str[(int)status->pres_chrg_step],
		status->pres_temp_zone,
		status->charging_limit_modes,
		status->demo_chrg_suspend);
}

static void mmi_reset_charger_configure(struct mmi_charger_chip *chip,
				struct mmi_charger *charger)
{
	charger->cfg.target_fv = charger->profile.max_fv_mv;
	charger->cfg.target_fcc = -EINVAL;
	charger->cfg.chrg_iterm = charger->profile.chrg_iterm;
	charger->cfg.fg_iterm = charger->profile.fg_iterm;
	charger->cfg.taper_kickoff = false;
	charger->cfg.charging_reset = false;
	charger->cfg.full_charged = false;
	charger->cfg.charging_disable = false;
	charger->cfg.charger_suspend = false;

	charger->constraint.demo_mode = chip->demo_mode;
	charger->constraint.factory_version = chip->factory_version;
	charger->constraint.factory_mode = chip->factory_mode;
	charger->constraint.dcp_pmax = chip->dcp_pmax;
	charger->constraint.hvdcp_pmax = chip->hvdcp_pmax;
	charger->constraint.pd_pmax = chip->pd_pmax;
	charger->constraint.wls_pmax = chip->wls_pmax;
}

static void mmi_configure_charger(struct mmi_charger_chip *chip,
				struct mmi_charger *charger)
{
	struct mmi_charger_profile *profile = &charger->profile;
	struct mmi_charger_status *status = &charger->status;
	struct mmi_charger_cfg *cfg = &charger->cfg;

	switch (status->pres_chrg_step) {
	case STEP_FLOAT:
	case STEP_MAX:
		if (!status->temp_zone || !status->temp_zone->norm_mv)
			cfg->target_fv = profile->max_fv_mv + profile->vfloat_comp_mv;
		else
			cfg->target_fv = status->temp_zone->norm_mv + profile->vfloat_comp_mv;
		if (!status->temp_zone)
			cfg->target_fcc = profile->max_fcc_ma;
		else
			cfg->target_fcc = status->temp_zone->fcc_max_ma;
		break;
	case STEP_FULL:
		cfg->target_fv = profile->max_fv_mv;
		cfg->target_fcc = -EINVAL;
		cfg->full_charged = true;
		break;
	case STEP_NORM:
		cfg->target_fv = profile->max_fv_mv + profile->vfloat_comp_mv;
		if (!status->temp_zone)
			cfg->target_fcc = profile->max_fcc_ma;
		else
			cfg->target_fcc = status->temp_zone->fcc_norm_ma;
		break;
	case STEP_NONE:
		cfg->target_fv = profile->max_fv_mv;
		if (!status->temp_zone)
			cfg->target_fcc = profile->max_fcc_ma;
		else
			cfg->target_fcc = status->temp_zone->fcc_norm_ma;
		break;
	case STEP_STOP:
		cfg->target_fv = profile->max_fv_mv;
		cfg->target_fcc = -EINVAL;
		break;
	case STEP_DEMO:
		cfg->target_fv = profile->demo_fv_mv;
		if (!status->temp_zone)
			cfg->target_fcc = profile->max_fcc_ma;
		else
			cfg->target_fcc = status->temp_zone->fcc_norm_ma;
		break;
	default:
		cfg->target_fv = profile->max_fv_mv;
		if (!status->temp_zone)
			cfg->target_fcc = profile->max_fcc_ma;
		else
			cfg->target_fcc = status->temp_zone->fcc_norm_ma;
		break;
	}

	if (cfg->target_fcc < 0 ||
	    mmi_get_effective_voter(&chip->disable_charging_vote) >= 0)
		cfg->charging_disable = true;
	else
		cfg->charging_disable = false;

	if (chip->factory_mode) {
		cfg->target_fv = 4400;
		cfg->target_fcc = 3000;
		cfg->charging_disable = true;
		cfg->taper_kickoff = false;
		cfg->charging_reset = false;
	}

	if (chip->force_charger_disabled ||
	    status->demo_chrg_suspend ||
	    mmi_get_effective_voter(&chip->suspend_charger_vote) >= 0)
		cfg->charger_suspend = true;

	if (chip->force_charging_enabled) {
		cfg->charging_disable = false;
	}

	mmi_notify_paired_battery(charger);
	charger->driver->set_constraint(charger->driver->data, &charger->constraint);
	charger->driver->config_charge(charger->driver->data, cfg);

	mmi_info(chip, "[C:%s]: FV=%d, FCC=%d, CDIS=%d,"
		" CSUS=%d, CRES=%d, CFULL=%d\n",
		charger->driver->name,
		cfg->target_fv,
		cfg->target_fcc,
		cfg->charging_disable,
		cfg->charger_suspend,
		cfg->charging_reset,
		cfg->full_charged);
}

static void mmi_notify_charger_event(struct mmi_charger_chip *chip, int type)
{
	char *event_string = NULL;

	if (!chip->batt_psy) {
		mmi_battery_supply_init(chip);
		if (!chip->batt_psy)
			return;
	}

	event_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (event_string) {
		switch (type) {
		case NOTIFY_EVENT_TYPE_CHG_RATE:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_CHARGE_RATE=%s",
				charge_rate[chip->max_charger_rate]);
			break;
		case NOTIFY_EVENT_TYPE_LPD_PRESENT:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_LPD_PRESENT=%s",
				chip->lpd_present? "true" : "false");
			break;
		case NOTIFY_EVENT_TYPE_VBUS_PRESENT:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_VBUS_PRESENT=%s",
				chip->vbus_present? "true" : "false");
			break;
		case NOTIFY_EVENT_TYPE_POWER_WATT:
			scnprintf(event_string, CHG_SHOW_MAX_SIZE,
				"POWER_SUPPLY_POWER_WATT=%d",
				chip->power_watt / 1000);
			break;
		default:
			mmi_err(chip, "Invalid notify event type %d\n", type);
			kfree(event_string);
			return;
		}

		if (chip->batt_uenvp[0])
			kfree(chip->batt_uenvp[0]);
		chip->batt_uenvp[0] = event_string;
		chip->batt_uenvp[1] = NULL;
		kobject_uevent_env(&chip->batt_psy->dev.kobj,
					KOBJ_CHANGE,
					chip->batt_uenvp);
	}
}

#define WEAK_CHRG_THRSH_MW 2500
#define TURBO_CHRG_THRSH_MW 12500
#define TURBO_CHRG_30W_THRSH_MW 25000
#define HYPER_CHRG_THRSH_MW 40000
int mmi_get_battery_charger_rate(struct mmi_charger *charger)
{
	int rate;
	struct mmi_charger_info *info;

	info = &charger->chg_info;
	if (!info->chrg_present)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;
	else if (info->chrg_pmax_mw < WEAK_CHRG_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_WEAK;
	else if (info->chrg_pmax_mw < TURBO_CHRG_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_NORMAL;
	else if (info->chrg_pmax_mw < TURBO_CHRG_30W_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_TURBO;
	else if (info->chrg_pmax_mw < HYPER_CHRG_THRSH_MW)
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_TURBO_30W;
	else
		rate = MMI_POWER_SUPPLY_CHARGE_RATE_HYPER;

	return rate;
}

static int mmi_get_battery_health(struct mmi_charger *charger)
{
	int batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	struct mmi_charger_status *status = &charger->status;
	struct mmi_battery_info *batt_info = &charger->batt_info;

	if (status->pres_temp_zone == ZONE_HOT) {
		batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (status->pres_temp_zone == ZONE_COLD) {
		batt_health = POWER_SUPPLY_HEALTH_COLD;
	} else if (batt_info->batt_temp >= WARM_TEMP) {
		if (status->pres_chrg_step == STEP_STOP)
			batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (batt_info->batt_temp <= COOL_TEMP) {
		if (status->pres_chrg_step == STEP_STOP)
			batt_health = POWER_SUPPLY_HEALTH_COLD;
		else
			batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else
		batt_health = POWER_SUPPLY_HEALTH_GOOD;

	return batt_health;
}

static int mmi_combine_battery_soc(struct mmi_charger_chip *chip)
{
	int soc = 0;
	int remain = 0, full = 0;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		remain += battery->info->batt_soc * battery->info->batt_full_uah;
		full += battery->info->batt_full_uah;
	}

	if (full > 0 && remain >= 0) {
		soc = remain / full;
		if (soc > 100)
			soc = 100;
	} else
		soc = -1;

	return soc;
}

static int mmi_combine_battery_status(struct mmi_charger_chip *chip)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		if (battery->status == POWER_SUPPLY_STATUS_CHARGING) {
			status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		}
		if (battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
		    status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			continue;
		}
		if (battery->status == POWER_SUPPLY_STATUS_DISCHARGING ||
		    status == POWER_SUPPLY_STATUS_DISCHARGING) {
			status = POWER_SUPPLY_STATUS_DISCHARGING;
			continue;
		}
		status = battery->status;
	}

	return status;
}

static int mmi_combine_battery_cycles(struct mmi_charger_chip *chip)
{
	int cycles = 0;
	int full = 0;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		cycles += battery->cycles * battery->info->batt_full_uah;
		full += battery->info->batt_full_uah;
	}

	if (full > 0 && cycles >= 0)
		cycles /= full;
	else
		cycles = -1;

	return cycles;
}

static int mmi_combine_battery_age(struct mmi_charger_chip *chip)
{
	int age = 100;
	int full = 0;
	int full_design = 0;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		full += battery->info->batt_full_uah;
		full_design += battery->info->batt_design_uah;
	}
	chip->charge_full = full;
	chip->charge_full_design = full_design;

	if (full_design > 0 && full > 0) {
		age = full * 100 / full_design;
		if (age > 100)
			age = 100;
	} else
		age = -1;

	return age;
}

static int mmi_combine_battery_current(struct mmi_charger_chip *chip)
{
	int current_ma = 0;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		current_ma += battery->info->batt_ma;
	}

	return current_ma;
}

static int mmi_combine_charge_counter(struct mmi_charger_chip *chip)
{
	int counter = 0;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		counter += battery->info->batt_chg_counter;
	}

	return counter;
}

static int mmi_combine_battery_voltage(struct mmi_charger_chip *chip)
{
	int voltage_mv = 0;
	int count = 0;
	struct mmi_battery_pack *battery = NULL;

	list_for_each_entry(battery, &chip->battery_list, list) {
		voltage_mv += battery->info->batt_mv;
		count++;
	}

	if (count)
		voltage_mv /= count;

	return voltage_mv;
}

static void mmi_update_battery_status(struct mmi_charger_chip *chip)
{
	int soc;
	int age;
	int status;
	int cycles;
	int voltage_mv;
	int current_ma;
	int charge_counter;
	bool mmi_changed = false;
	int batt_temp;
	int batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	int charger_rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;
	int max_charger_rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;
	struct mmi_charger *charger = NULL;
	struct mmi_battery_pack *battery = NULL;
	struct mmi_battery_info *batt_info = NULL;
	bool vbus_present = false;
	bool lpd_present = false;
	int power_watt = 0;

	mutex_lock(&chip->battery_lock);
	list_for_each_entry(charger, &chip->charger_list, list) {
		battery = charger->battery;
		batt_info = &charger->batt_info;

		/* Do it only for the first charger */
		if (battery->pending == battery->reference) {
			/* init for going through all chargers */
			battery->status = POWER_SUPPLY_STATUS_DISCHARGING;
			battery->charger_rate = MMI_POWER_SUPPLY_CHARGE_RATE_NONE;

			/* update only at the first charger */
			battery->health = mmi_get_battery_health(charger);
			if (batt_info->batt_design_uah > 0)
				battery->age = batt_info->batt_full_uah * 100 \
					/ batt_info->batt_design_uah;
			if (battery->soc_cycles_start < batt_info->batt_soc)
				battery->cycles += \
				batt_info->batt_soc - battery->soc_cycles_start;
			battery->soc_cycles_start = batt_info->batt_soc;
			battery->info = batt_info;
		}

		/* update charger rate */
		charger_rate = mmi_get_battery_charger_rate(charger);
		if (battery->charger_rate < charger_rate)
			battery->charger_rate = charger_rate;

		if (charger->chg_info.chrg_pmax_mw > power_watt)
			power_watt = charger->chg_info.chrg_pmax_mw;

		/* update charging status */
		if (batt_info->batt_status == POWER_SUPPLY_STATUS_FULL ||
		    battery->status == POWER_SUPPLY_STATUS_FULL ||
		    charger->status.pres_chrg_step == STEP_FULL)
			battery->status = POWER_SUPPLY_STATUS_FULL;
		else if (batt_info->batt_status == POWER_SUPPLY_STATUS_CHARGING ||
		    battery->status == POWER_SUPPLY_STATUS_CHARGING)
			battery->status = POWER_SUPPLY_STATUS_CHARGING;
		else if (batt_info->batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
		    battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			battery->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			battery->status = POWER_SUPPLY_STATUS_DISCHARGING;

		battery->pending--;
		/* Do it only for the last charger */
		if (battery->pending <= 0) {
			if (max_charger_rate < battery->charger_rate)
				max_charger_rate = battery->charger_rate;
			battery->pending = 0;
			mmi_dbg(chip, "[B:%s]: health:%d, status:%d,"
				" age:%d, cycles:%d, rate:%s\n",
			battery->sn,
			battery->health,
			battery->status,
			battery->age,
			battery->cycles,
			charge_rate[battery->charger_rate]);
		}
		/* update vbus and liquid present detection status */
		if (!vbus_present && charger->chg_info.vbus_present)
			vbus_present = true;
		if (!lpd_present && charger->chg_info.lpd_present)
			lpd_present = true;
	}

	soc = mmi_combine_battery_soc(chip);
	status = mmi_combine_battery_status(chip);
	cycles = mmi_combine_battery_cycles(chip);
	age = mmi_combine_battery_age(chip);
	voltage_mv = mmi_combine_battery_voltage(chip);
	current_ma = mmi_combine_battery_current(chip);
	charge_counter = mmi_combine_charge_counter(chip);
	if (soc >= 0 && chip->combo_soc != soc) {
		mmi_changed = true;
		chip->combo_soc = soc;
	}
	if (chip->combo_status != status) {
		mmi_changed = true;
		chip->combo_status = status;
	}
	if (cycles >= 0 &&
	   (chip->combo_cycles / 100) != (cycles / 100)) {
		mmi_changed = true;
		chip->combo_cycles = cycles;
	}
	if (age > 0 && chip->combo_age != age) {
		mmi_changed = true;
		chip->combo_age = age;
	}
	if (voltage_mv > 0 && chip->combo_voltage_mv != voltage_mv) {
		chip->combo_voltage_mv = voltage_mv;
	}
	if (chip->combo_current_ma != current_ma) {
		chip->combo_current_ma = current_ma;
	}
	if (chip->combo_charge_counter != charge_counter) {
		chip->combo_charge_counter = charge_counter;
	}
	if (chip->max_charger_rate != max_charger_rate) {
		mmi_changed = true;
		chip->max_charger_rate = max_charger_rate;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_CHG_RATE);
		mmi_err(chip, "%s charger is detected\n",
			charge_rate[chip->max_charger_rate]);
	}

	if (chip->lpd_present != lpd_present) {
		mmi_changed = true;
		chip->lpd_present = lpd_present;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_LPD_PRESENT);
		mmi_info(chip, "lpd is %s\n",
			lpd_present? "present" : "absent");
	}

	if (chip->vbus_present != vbus_present) {
		mmi_changed = true;
		chip->vbus_present = vbus_present;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_VBUS_PRESENT);
		mmi_info(chip, "vbus is %s\n",
			vbus_present? "present" : "absent");
	}

	if ((chip->power_watt / 1000) != (power_watt / 1000)) {
		mmi_changed = true;
		chip->power_watt = power_watt;
		mmi_notify_charger_event(chip, NOTIFY_EVENT_TYPE_POWER_WATT);
		mmi_info(chip, "charger power is %d mW\n", power_watt);
	}

	list_for_each_entry(battery, &chip->battery_list, list) {
		if (batt_health == POWER_SUPPLY_HEALTH_UNKNOWN) {
			batt_health = battery->health;
			batt_temp = battery->info->batt_temp;
			continue;
		}
		if (battery->info->batt_temp > batt_temp) {
			batt_temp = battery->info->batt_temp;
			batt_health = battery->health;
		}
	}
	if (chip->combo_health != batt_health ||
	    chip->combo_temp != batt_temp) {
		mmi_changed = true;
		chip->combo_health = batt_health;
		chip->combo_temp = batt_temp;
	}

	if (mmi_changed) {
		power_supply_changed(chip->mmi_psy);
		mmi_info(chip, "Combo status: soc:%d, status:%d, temp:%d,"
			" health:%d, age:%d, cycles:%d, voltage:%d, current:%d,"
			" counter:%d, rate:%s, lpd:%d, vbus:%d\n",
			chip->combo_soc,
			chip->combo_status,
			chip->combo_temp,
			chip->combo_health,
			chip->combo_age,
			chip->combo_cycles,
			chip->combo_voltage_mv,
			chip->combo_current_ma,
			chip->combo_charge_counter,
			charge_rate[chip->max_charger_rate],
			chip->lpd_present,
			chip->vbus_present);
	}

	mutex_unlock(&chip->battery_lock);
}

static void mmi_charger_heartbeat_work(struct work_struct *work)
{
	int hb_resch_time;
	struct mmi_charger *charger = NULL;
	struct mmi_charger_chip *chip = container_of(work,
						struct mmi_charger_chip,
						heartbeat_work.work);
	struct timespec64 now;
	static struct timespec64 start;
	uint32_t elapsed_ms;

	/* Have not been resumed so wait another 100 ms */
	if (chip->suspended & IS_SUSPENDED) {
		mmi_err(chip, "HB running before Resume\n");
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));
		return;
	}

	mmi_dbg(chip, "Heartbeat!\n");

	pm_stay_awake(chip->dev);
	alarm_cancel(&chip->heartbeat_alarm);

	mutex_lock(&chip->charger_lock);
	list_for_each_entry(charger, &chip->charger_list, list) {
		mmi_get_charger_info(chip, charger);
		mmi_update_charger_profile(chip, charger);
		mmi_reset_charger_configure(chip, charger);
		mmi_update_charger_status(chip, charger);
	}
	list_for_each_entry(charger, &chip->charger_list, list) {
		mmi_configure_charger(chip, charger);
	}
	mmi_update_battery_status(chip);
	mutex_unlock(&chip->charger_lock);

	mmi_dbg(chip, "DemoMode:%d, FactoryVersion:%d, FactoryMode:%d,"
		" dcp_pmax:%d, hvdcp_pmax:%d, pd_pmax:%d, wls_pmax:%d\n",
		chip->demo_mode,
		chip->factory_version,
		chip->factory_mode,
		chip->dcp_pmax,
		chip->hvdcp_pmax,
		chip->pd_pmax,
		chip->wls_pmax);

	if (chip->factory_mode ||
	    (chip->factory_version && chip->enable_factory_poweroff)) {
		if (chip->max_charger_rate > MMI_POWER_SUPPLY_CHARGE_RATE_NONE) {
			mmi_dbg(chip, "Factory Kill Armed\n");
			chip->factory_kill_armed = true;
			ktime_get_real_ts64(&start);
		} else if (chip->factory_kill_armed && !factory_kill_disable) {
			ktime_get_real_ts64(&now);
			elapsed_ms = (now.tv_sec - start.tv_sec) * 1000;
			elapsed_ms += (now.tv_nsec - start.tv_nsec) / 1000000;
			if (elapsed_ms < chip->factory_kill_debounce_ms) {
				mmi_err(chip, "Factory kill debounce elapsed_ms:%d\n",
					elapsed_ms);
			} else if(!shutdown_triggered) {
				mmi_err(chip, "Factory kill power off\n");
				shutdown_triggered = true;
#if (KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE) || defined(MMI_GKI_API_ALLOWANCE)
				orderly_poweroff(true);
#else
				kernel_power_off();
#endif
			}
		} else {
			chip->factory_kill_armed = false;
		}
	}

	if (chip->empty_vbat_shutdown_triggered && !shutdown_triggered) {
		mmi_err(chip, "shutdown for empty battery voltage\n");
		shutdown_triggered = true;
#if (KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE) || defined(MMI_GKI_API_ALLOWANCE)
		orderly_poweroff(true);
#else
		kernel_power_off();
#endif
	}

	chip->suspended = 0;

	if (chip->factory_mode)
		hb_resch_time = HEARTBEAT_FACTORY_MS;
	else if (chip->max_charger_rate != MMI_POWER_SUPPLY_CHARGE_RATE_NONE
		 && chip->combo_status != POWER_SUPPLY_STATUS_FULL)
		hb_resch_time = chip->heartbeat_interval;
	else
		hb_resch_time = chip->heartbeat_dischg_ms;
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));
	if (suspend_wakeups ||
	    chip->max_charger_rate != MMI_POWER_SUPPLY_CHARGE_RATE_NONE)
		alarm_start_relative(&chip->heartbeat_alarm,
				     ns_to_ktime(HEARTBEAT_WAKEUP_INTRVAL_NS));

	if (chip->max_charger_rate == MMI_POWER_SUPPLY_CHARGE_RATE_NONE)
		pm_relax(chip->dev);

	PM_RELAX(chip->mmi_hb_wake_source);
}

const char *mmi_get_battery_serialnumber(void)
{
        struct device_node *np = of_find_node_by_path("/chosen");
        const char *battsn_buf;
        int retval;

        battsn_buf = NULL;

        if (np)
                retval = of_property_read_string(np, "mmi,battid",
                                                 &battsn_buf);
        else
                return NULL;

        if ((retval == -EINVAL) || !battsn_buf) {
                pr_err("Battsn unused\n");
                of_node_put(np);
                return NULL;

        } else
                pr_err("Battsn = %s\n", battsn_buf);

        of_node_put(np);

        return battsn_buf;
}
EXPORT_SYMBOL(mmi_get_battery_serialnumber);

bool mmi_is_factory_mode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory_mode = false;
	const char *bootargs = NULL;
	char *bootmode = NULL;
	char *end = NULL;

	if ((this_chip && this_chip->factory_mode) ||
	    !strncmp(bi_bootmode(), "mot-factory", 11))
		return true;

	if (!np)
		return factory_mode;

	if (!of_property_read_string(np, "bootargs", &bootargs)) {
		bootmode = strstr(bootargs, "androidboot.mode=");
		if (bootmode) {
			end = strpbrk(bootmode, " ");
			bootmode = strpbrk(bootmode, "=");
		}
		if (bootmode &&
		    end > bootmode &&
		    strnstr(bootmode, "factory", end - bootmode)) {
				factory_mode = true;
		}
	}
	of_node_put(np);

	return factory_mode;
}
EXPORT_SYMBOL(mmi_is_factory_mode);

bool mmi_is_factory_version(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory_version = false;
	const char *bootargs = NULL;
	char *bootloader = NULL;
	char *end = NULL;

	if (this_chip && this_chip->factory_version)
		return true;

	if (!np)
		return factory_version;

	if (!of_property_read_string(np, "bootargs", &bootargs)) {
		bootloader = strstr(bootargs, "androidboot.bootloader=");
		if (bootloader) {
			end = strpbrk(bootloader, " ");
			bootloader = strpbrk(bootloader, "=");
		}
		if (bootloader &&
		    end > bootloader &&
		    strnstr(bootloader, "factory", end - bootloader)) {
				factory_version = true;
		}
	}
	of_node_put(np);

	return factory_version;
}
EXPORT_SYMBOL(mmi_is_factory_version);

void mmi_get_charger_configure(struct mmi_charger_driver *driver)
{
	struct mmi_charger *charger = NULL;
	struct mmi_charger_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return;
	}

	mutex_lock(&chip->charger_lock);
	list_for_each_entry(charger, &chip->charger_list, list) {
		if (charger->driver == driver) {
			mmi_get_charger_info(chip, charger);
			mmi_update_charger_profile(chip, charger);
			mmi_reset_charger_configure(chip, charger);
			mmi_update_charger_status(chip, charger);
			mmi_configure_charger(chip, charger);
			break;
		}
	}
	mmi_update_battery_status(chip);
	mutex_unlock(&chip->charger_lock);
}
EXPORT_SYMBOL(mmi_get_charger_configure);

int mmi_vote_charging_disable(const char *voter, bool enable)
{
	struct mmi_charger_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -EINVAL;
	}

	return mmi_vote(&chip->disable_charging_vote, voter, enable, 0);
}
EXPORT_SYMBOL(mmi_vote_charging_disable);

int mmi_vote_charger_suspend(const char *voter, bool enable)
{
	struct mmi_charger_chip *chip = this_chip;

	if (!chip) {
		pr_err("mmi_charger: chip is invalid\n");
		return -EINVAL;
	}

	return mmi_vote(&chip->suspend_charger_vote, voter, enable, 0);
}
EXPORT_SYMBOL(mmi_vote_charger_suspend);

int mmi_register_charger_driver(struct mmi_charger_driver *driver)
{
	int ret = 0;
	struct mmi_charger_chip *chip = this_chip;
	struct mmi_charger *charger = NULL;
	struct mmi_battery_pack *battery = NULL;
	struct mmi_battery_info batt_info;
	struct mmi_charger_info chg_info;

	if (!chip) {
		pr_err("%s: No mmi charger chip\n", __func__);
		return -ENODEV;
	}

	if (!driver) {
		mmi_err(chip, "Invalid mmi charger driver\n");
		return -EINVAL;
	}

	if (!driver->get_batt_info ||
	    !driver->get_chg_info ||
	    !driver->config_charge ||
	    !driver->is_charge_tapered ||
	    !driver->set_constraint) {
		mmi_err(chip, "[C:%s]: mmi charger function is empty\n",
			driver->name);
		return -EINVAL;
	}

	memset(&batt_info, 0, sizeof(batt_info));
	memset(&chg_info, 0, sizeof(chg_info));
	driver->get_batt_info(driver->data, &batt_info);
	driver->get_chg_info(driver->data, &chg_info);
	mutex_lock(&chip->charger_lock);
	list_for_each_entry(charger, &chip->charger_list, list) {
		if (charger->driver == driver) {
			ret = -EEXIST;
			mmi_err(chip,
			"[C:%s]: driver has already registered\n",
			driver->name);
			goto exit;
		}
		if (!battery &&
		    !strcmp(charger->battery->sn, batt_info.batt_sn))
			battery = charger->battery;
	}

	charger = devm_kzalloc(chip->dev, sizeof(struct mmi_charger),
					GFP_KERNEL);
	if (!charger) {
		ret = -ENOMEM;
		goto exit;
	}

	if (!battery) {
		battery = devm_kzalloc(chip->dev, sizeof(struct mmi_battery_pack),
					GFP_KERNEL);
		if (!battery) {
			devm_kfree(chip->dev, charger);
			ret = -ENOMEM;
			goto exit;
		}
		battery->reference++;
		battery->age = 100;
		battery->cycles = 0;
		battery->soc_cycles_start = 100;
		memcpy(battery->sn, batt_info.batt_sn, MMI_BATT_SN_LEN);
		of_property_read_u32(driver->dev->of_node, "mmi,paired-id",
					&battery->paired_id);
		mmi_update_paired_battery(chip, battery);

		mutex_lock(&chip->battery_lock);
		list_add_tail(&battery->list, &chip->battery_list);
		mutex_unlock(&chip->battery_lock);
	} else {
		mutex_lock(&chip->battery_lock);
		battery->reference++;
		mutex_unlock(&chip->battery_lock);
	}

	memset(&charger->status, 0, sizeof(struct mmi_charger_status));
	charger->status.pres_temp_zone = ZONE_NONE;
	charger->status.pres_chrg_step = STEP_NONE;
	charger->status.demo_full_soc = 100;
	charger->status.charging_limit_modes = CHARGING_LIMIT_UNKNOWN;
	memcpy(&charger->batt_info, &batt_info, sizeof(batt_info));
	memcpy(&charger->chg_info, &chg_info, sizeof(chg_info));
	charger->driver = driver;
	charger->battery = battery;
	charger->battery->info = &charger->batt_info;
	mmi_get_charger_profile(chip, charger);
	list_add_tail(&charger->list, &chip->charger_list);

	if (chip->batt_psy) {
		mmi_info(chip, "[C:%s] register charger succesfully, Throw out BATT_PSY change to update battery info\n", driver->name);
		power_supply_changed(chip->batt_psy);
	}
exit:
	mutex_unlock(&chip->charger_lock);

	return ret;
}
EXPORT_SYMBOL(mmi_register_charger_driver);

int mmi_unregister_charger_driver(struct mmi_charger_driver *driver)
{
	int ret = 0;
	struct mmi_charger_chip *chip = this_chip;
	struct mmi_charger *charger = NULL, *tmp = NULL;

	if (!chip) {
		pr_err("%s: No mmi charger chip\n", __func__);
		return -ENODEV;
	}

	if (!driver) {
		mmi_err(chip, "Invalid mmi charger driver\n");
		return -EINVAL;
	}

	mutex_lock(&chip->charger_lock);
	list_for_each_entry(tmp, &chip->charger_list, list) {
		if (tmp->driver == driver) {
			charger = tmp;
			break;
		}
	}

	if (charger) {
		mutex_lock(&chip->battery_lock);
		charger->battery->reference--;
		if (charger->battery->reference <= 0) {
			list_del(&charger->battery->list);
			devm_kfree(chip->dev, charger->battery);
		}
		mutex_unlock(&chip->battery_lock);

		list_del(&charger->list);
		devm_kfree(chip->dev, charger);
	} else {
		mmi_err(chip, "[C:%s]: driver has not registered yet\n",
			driver->name);
		ret = -ENOENT;
	}

	mutex_unlock(&chip->charger_lock);
	return ret;
}
EXPORT_SYMBOL(mmi_unregister_charger_driver);

static enum alarmtimer_restart mmi_heartbeat_alarm_cb(struct alarm *alarm,
						      ktime_t now)
{
	struct mmi_charger_chip *chip = container_of(alarm,
						    struct mmi_charger_chip,
						    heartbeat_alarm);

	mmi_info(chip, "HB alarm fired\n");

	PM_STAY_AWAKE(chip->mmi_hb_wake_source);
	cancel_delayed_work(&chip->heartbeat_work);
	/* Delay by 500 ms to allow devices to resume. */
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(500));

	return ALARMTIMER_NORESTART;
}

static int mmi_psy_notifier_call(struct notifier_block *nb, unsigned long val,
				 void *v)
{
	struct mmi_charger_chip *chip = container_of(nb,
				struct mmi_charger_chip, mmi_psy_notifier);
	struct power_supply *psy = v;

	if (!chip) {
		pr_err("called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (psy &&
	    ((strcmp(psy->desc->name, "battery") == 0) ||
	    (strcmp(psy->desc->name, "usb") == 0) ||
	    (strcmp(psy->desc->name, "wireless") == 0))) {
		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(0));
	}

	return NOTIFY_OK;
}

static int mmi_charger_reboot(struct notifier_block *nb,
			 unsigned long event, void *unused)
{
	struct mmi_charger_chip *chip = container_of(nb, struct mmi_charger_chip,
						mmi_reboot);

	if (!chip) {
		pr_err("called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (!chip->factory_mode)
		return NOTIFY_DONE;

	switch (event) {
	case SYS_POWER_OFF:
		factory_kill_disable = true;
		chip->force_charger_disabled = true;
		schedule_delayed_work(&chip->heartbeat_work, msecs_to_jiffies(0));
		while (chip->max_charger_rate != MMI_POWER_SUPPLY_CHARGE_RATE_NONE &&
			shutdown_triggered && !chip->empty_vbat_shutdown_triggered) {
			mmi_info(chip, "Wait for charger removal\n");
			msleep(100);
		}
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static enum power_supply_property mmi_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int mmi_get_prop(struct power_supply *psy,
			 enum power_supply_property psp,
			 union power_supply_propval *val)
{
	struct mmi_charger_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;
	uint32_t elapsed_ms;
	struct timespec64 now;
	static struct timespec64 start = {0};

	if (psp == POWER_SUPPLY_PROP_CURRENT_NOW &&
	    chip->ibat_calc_alignment_time != UINT_MAX) {
		ktime_get_real_ts64(&now);
		if (now.tv_sec >= start.tv_sec) {
			elapsed_ms = (now.tv_sec - start.tv_sec) * 1000;
			elapsed_ms += (now.tv_nsec - start.tv_nsec) / 1000000;
		} else {
			elapsed_ms = 0;
			start = now;
		}
		if (elapsed_ms >= chip->ibat_calc_alignment_time) {
			cancel_delayed_work(&chip->heartbeat_work);
			schedule_delayed_work(&chip->heartbeat_work,
					msecs_to_jiffies(0));
			start = now;
		}
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->combo_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->combo_health;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->combo_temp * 10;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->combo_soc;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->init_cycles + chip->combo_cycles / 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->charge_full_design;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = chip->charge_full;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->combo_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chip->combo_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = chip->combo_charge_counter;
		break;
	default:
		val->intval = -EINVAL;
		break;
	}

	return rc;
}

static int mmi_set_prop(struct power_supply *psy,
			 enum power_supply_property prop,
			 const union power_supply_propval *val)
{
	int rc = 0;
	struct mmi_charger_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		chip->init_cycles = val->intval;
		break;
	default:
		break;
	}
	return rc;
}

static int mmi_prop_is_writeable(struct power_supply *psy,
				  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc mmi_psy_desc = {
	.name		= "mmi_battery",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= mmi_get_prop,
	.set_property	= mmi_set_prop,
	.property_is_writeable = mmi_prop_is_writeable,
	.properties	= mmi_props,
	.num_properties	= ARRAY_SIZE(mmi_props),
};

static int mmi_parse_dt(struct mmi_charger_chip *chip)
{
	int rc = 0;
	const char *charger_ability = NULL;
	struct device_node *node = chip->dev->of_node;

	chip->enable_charging_limit =
		of_property_read_bool(node, "mmi,enable-charging-limit");

        chip->enable_factory_poweroff =
                of_property_read_bool(node, "mmi,enable-factory-poweroff");

	chip->start_factory_kill_disabled =
			of_property_read_bool(node, "mmi,start-factory-kill-disabled");

	rc = of_property_read_u32(node, "mmi,factory-kill-debounce-ms",
				  &chip->factory_kill_debounce_ms);
	if (rc)
		chip->factory_kill_debounce_ms = 0;

	rc = of_property_read_u32(node, "mmi,upper-limit-capacity",
				  &chip->upper_limit_capacity);
	if (rc)
		chip->upper_limit_capacity = 100;

	rc = of_property_read_u32(node, "mmi,lower-limit-capacity",
				  &chip->lower_limit_capacity);
	if (rc)
		chip->lower_limit_capacity = 0;

	rc = of_property_read_u32(node, "mmi,heartbeat-interval",
				  &chip->heartbeat_interval);
	if (rc)
		chip->heartbeat_interval = HEARTBEAT_DELAY_MS;

	rc = of_property_read_u32(node, "mmi,dcp-power-max",
				  &chip->dcp_pmax);
	if (rc)
		chip->dcp_pmax = CHARGER_POWER_7P5W;

	rc = of_property_read_u32(node, "mmi,hvdcp-power-max",
				  &chip->hvdcp_pmax);
	if (rc)
		chip->hvdcp_pmax = CHARGER_POWER_15W;

	rc = of_property_read_u32(node, "mmi,pd-power-max",
				  &chip->pd_pmax);
	if (rc)
		chip->pd_pmax = CHARGER_POWER_18W;

	rc = of_property_read_u32(node, "mmi,wls-power-max",
				  &chip->wls_pmax);
	if (rc)
		chip->wls_pmax = CHARGER_POWER_10W;

	rc = of_property_read_u32(node, "mmi,heartbeat-discharger-ms",
				  &chip->heartbeat_dischg_ms);
	if (rc)
		chip->heartbeat_dischg_ms = HEARTBEAT_DISCHARGE_MS;

	mmi_warn(chip, "mmi,heartbeat dischg ms %d\n", chip->heartbeat_dischg_ms);

	rc = of_property_read_u32(node, "mmi,ibat-calc-alignment-time",
				  &chip->ibat_calc_alignment_time);
	if (rc)
		chip->ibat_calc_alignment_time = UINT_MAX;

	node = of_find_node_by_path("/chosen");

	if (!node)
		return 0;

	rc = of_property_read_string(node, "mmi,charger", &charger_ability);
	if ((rc == -EINVAL) || !charger_ability) {
		mmi_warn(chip, "mmi,charger is unused\n");
	} else {
		mmi_info(chip, "QC charger ability = %s\n", charger_ability);
		if (strstr(charger_ability, "15W"))
			chip->hvdcp_pmax = CHARGER_POWER_15W;
		else if (strstr(charger_ability, "18W"))
			chip->hvdcp_pmax = CHARGER_POWER_18W;
		else if (strstr(charger_ability, "20W"))
			chip->hvdcp_pmax = CHARGER_POWER_20W;
	}

	charger_ability = NULL;
	rc = of_property_read_string(node, "mmi,usb_dcp", &charger_ability);
	if ((rc == -EINVAL) || !charger_ability) {
		mmi_warn(chip, "mmi,usb_dcp is unused\n");
	} else {
		mmi_info(chip, "DCP charger ability = %s\n", charger_ability);
		if (strstr(charger_ability, "1.5A"))
			chip->dcp_pmax = CHARGER_POWER_7P5W;
		else if (strstr(charger_ability, "2A"))
			chip->dcp_pmax = CHARGER_POWER_10W;
	}

	of_node_put(node);

	return 0;
}

static int mmi_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct mmi_charger_chip *chip;
	struct power_supply_config psy_cfg = {};

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->name = "mmi_charger";
	chip->dev = &pdev->dev;
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;
	chip->suspended = 0;
	chip->combo_status = 0;
	chip->combo_age = 100;
	chip->combo_voltage_mv = 0;
	chip->combo_current_ma = 0;
	chip->combo_cycles = 0;
	chip->combo_soc = 100;
	chip->combo_charge_counter = 0;
	chip->charge_full = 0;
	chip->charge_full_design = 0;
	chip->init_cycles = 0;
	chip->factory_version = mmi_is_factory_version();
	chip->factory_mode = mmi_is_factory_mode();
	chip->disable_charging_vote.name = "disable_charging";
	chip->suspend_charger_vote.name = "suspend_charger";
	platform_set_drvdata(pdev, chip);
	device_init_wakeup(chip->dev, true);

	chip->debug_enabled = &debug_enabled;
	chip->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chip->ipc_log)
		mmi_err(chip, "Failed to create mmi charger IPC log\n");
	else
		mmi_info(chip, "IPC logging is enabled for mmi charger\n");

	rc = mmi_parse_dt(chip);
	if (rc) {
		mmi_err(chip, "Failed to parse device tree\n");
		rc = -EINVAL;
		goto exit;
	}

	this_chip = chip;
	mutex_init(&chip->charger_lock);
	mutex_init(&chip->battery_lock);
	INIT_LIST_HEAD(&chip->charger_list);
	INIT_LIST_HEAD(&chip->battery_list);
	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_charger_heartbeat_work);
	PM_WAKEUP_REGISTER(chip->dev, chip->mmi_hb_wake_source, "mmi_hb_wake");
	alarm_init(&chip->heartbeat_alarm, ALARM_BOOTTIME,
		   mmi_heartbeat_alarm_cb);

	chip->mmi_psy = devm_power_supply_register(chip->dev,
						&mmi_psy_desc,
						&psy_cfg);
	if (IS_ERR(chip->mmi_psy)) {
		mmi_err(chip, "failed: mmi power supply register\n");
		rc = PTR_ERR(chip->mmi_psy);
		goto exit;
	}

	mmi_battery_supply_init(chip);

	rc = device_create_file(chip->dev,
				&dev_attr_state_sync);
	if (rc) {
		mmi_err(chip, "couldn't create state_sync\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_dcp_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create dcp_pmax\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_hvdcp_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create hvdcp_pmax\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_pd_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create pd_pmax\n");
	}

	rc = device_create_file(chip->dev,
				&dev_attr_wls_pmax);
	if (rc) {
		mmi_err(chip, "couldn't create wls_pmax\n");
	}

	/* Register the notifier for the psy updates*/
	chip->mmi_psy_notifier.notifier_call = mmi_psy_notifier_call;
	rc = power_supply_reg_notifier(&chip->mmi_psy_notifier);
	if (rc)
		mmi_err(chip, "Failed to reg notifier: %d\n", rc);

	if (chip->factory_mode) {
		mmi_info(chip, "Entering Factory Mode!\n");
		chip->mmi_reboot.notifier_call = mmi_charger_reboot;
		chip->mmi_reboot.next = NULL;
		chip->mmi_reboot.priority = 1;
		rc = register_reboot_notifier(&chip->mmi_reboot);
		if (rc)
			mmi_err(chip, "Register for reboot failed\n");
	}

	if (chip->start_factory_kill_disabled)
		factory_kill_disable = 1;

	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(0));

	mmi_info(chip, "MMI charger probed successfully!\n");
	return 0;
exit:
	ipc_log_context_destroy(chip->ipc_log);

	return rc;
}

static int mmi_charger_remove(struct platform_device *pdev)
{
	struct mmi_charger_chip *chip = platform_get_drvdata(pdev);

	if (!list_empty(&chip->charger_list)) {
		return -EBUSY;
	}

	cancel_delayed_work(&chip->heartbeat_work);

	if (chip->factory_mode)
		unregister_reboot_notifier(&chip->mmi_reboot);
	power_supply_unreg_notifier(&chip->mmi_psy_notifier);
	device_remove_file(chip->dev, &dev_attr_state_sync);
	device_remove_file(chip->dev, &dev_attr_dcp_pmax);
	device_remove_file(chip->dev, &dev_attr_hvdcp_pmax);
	device_remove_file(chip->dev, &dev_attr_pd_pmax);
	device_remove_file(chip->dev, &dev_attr_wls_pmax);
	if (chip->batt_psy) {
		if (chip->batt_uenvp[0]) {
			kfree(chip->batt_uenvp[0]);
			chip->batt_uenvp[0] = NULL;
		}
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_force_demo_mode);
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_force_max_chrg_temp);
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_factory_image_mode);
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_factory_charge_upper);
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_force_charger_suspend);
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_force_charging_enable);
		device_remove_file(chip->batt_psy->dev.parent,
					&dev_attr_force_charging_disable);
		sysfs_remove_group(&chip->batt_psy->dev.kobj,
					&power_supply_mmi_attr_group);
		power_supply_put(chip->batt_psy);
	}
	PM_WAKEUP_UNREGISTER(chip->mmi_hb_wake_source);
	ipc_log_context_destroy(chip->ipc_log);

	return 0;
}

static void mmi_charger_shutdown(struct platform_device *pdev)
{
	struct mmi_charger_chip *chip = platform_get_drvdata(pdev);

	mmi_info(chip, "MMI charger shutdown\n");

	return;
}

#ifdef CONFIG_PM_SLEEP
static int mmi_charger_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_charger_chip *chip = platform_get_drvdata(pdev);

	chip->suspended &= ~WAS_SUSPENDED;
	chip->suspended |= IS_SUSPENDED;

	return 0;
}

static int mmi_charger_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_charger_chip *chip = platform_get_drvdata(pdev);

	chip->suspended &= ~IS_SUSPENDED;
	chip->suspended |= WAS_SUSPENDED;

	return 0;
}
#else
#define mmi_charger_suspend NULL
#define mmi_charger_resume NULL
#endif

static const struct dev_pm_ops mmi_charger_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmi_charger_suspend, mmi_charger_resume)
};

static const struct of_device_id match_table[] = {
	{ .compatible = "mmi,mmi-charger", },
	{ },
};

static struct platform_driver mmi_charger_driver = {
	.driver		= {
		.name		= "mmi,mmi-charger",
		.owner		= THIS_MODULE,
		.pm		= &mmi_charger_dev_pm_ops,
		.of_match_table	= match_table,
	},
	.probe		= mmi_charger_probe,
	.remove		= mmi_charger_remove,
	.shutdown	= mmi_charger_shutdown,
};
module_platform_driver(mmi_charger_driver);

MODULE_DESCRIPTION("MMI Charger Driver");
MODULE_LICENSE("GPL v2");
