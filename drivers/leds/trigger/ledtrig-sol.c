/*
 * Description: Sign-of-life LED trigger
 *
 * Copyright (c) 2021 Lenovo, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/leds.h>
#include <linux/power_supply.h>
#include <linux/device/class.h>
#include <linux/delay.h>

#define CHARGING_DELAY_ON_DEFAULT  (700)
#define CHARGING_DELAY_OFF_DEFAULT (700)
#define LOW_CAP_DELAY_ON_DEFAULT   (3800)
#define LOW_CAP_DELAY_OFF_DEFAULT  (200)

#define LOW_CAP_THRESHOLD_DEFAULT (15)
#define BATT_CAP_THRESHOLD_HALF   (50)
#define BATT_CAP_THRESHOLD_HIGH   (75)
#define BATT_CAP_THRESHOLD_FULL   (100)

struct sol_trig_data {
	struct led_classdev *led_cdev;
	struct notifier_block nb;
	unsigned long charging_delay_on;
	unsigned long charging_delay_off;
	unsigned long low_cap_delay_on;
	unsigned long low_cap_delay_off;
	int low_cap_threshold;
	bool mmi_battery_psy_present;
	bool battery_psy_present;
};

static ssize_t charging_delay_on_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);

	return sprintf(buf, "%lu\n", sol_data->charging_delay_on);
}

static ssize_t charging_delay_on_store(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	sol_data->charging_delay_on = val;

	return size;
}

static ssize_t charging_delay_off_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);

	return sprintf(buf, "%lu\n", sol_data->charging_delay_off);
}

static ssize_t charging_delay_off_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	sol_data->charging_delay_off = val;

	return size;
}

static ssize_t low_cap_delay_on_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);

	return sprintf(buf, "%lu\n", sol_data->low_cap_delay_on);
}

static ssize_t low_cap_delay_on_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	sol_data->low_cap_delay_on = val;

	return size;
}

static ssize_t low_cap_delay_off_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);

	return sprintf(buf, "%lu\n", sol_data->low_cap_delay_off);
}

static ssize_t low_cap_delay_off_store(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);
	unsigned long val;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	sol_data->low_cap_delay_off = val;

	return size;
}

static ssize_t low_cap_threshold_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);

	return sprintf(buf, "%d\n", sol_data->low_cap_threshold);
}

static ssize_t low_cap_threshold_store(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev  *led_cdev = led_trigger_get_led(dev);
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);
	long val;
	ssize_t ret;

	ret = kstrtol(buf, 0, &val);
	if (ret)
		return ret;

	if (val > 100)          /* Max capacity is 100% */
		return -EINVAL;

	sol_data->low_cap_threshold = val;

	return size;
}

static DEVICE_ATTR(charging_delay_on,      0644, charging_delay_on_show,  charging_delay_on_store);
static DEVICE_ATTR(charging_delay_off,     0644, charging_delay_off_show, charging_delay_off_store);
static DEVICE_ATTR(low_capacity_delay_on,  0644, low_cap_delay_on_show,   low_cap_delay_on_store);
static DEVICE_ATTR(low_capacity_delay_off, 0644, low_cap_delay_off_show,  low_cap_delay_off_store);
static DEVICE_ATTR(low_capacity_threshold, 0644, low_cap_threshold_show,  low_cap_threshold_store);

static struct attribute *sol_trig_attrs[] = {
	&dev_attr_charging_delay_on.attr,
	&dev_attr_charging_delay_off.attr,
	&dev_attr_low_capacity_delay_on.attr,
	&dev_attr_low_capacity_delay_off.attr,
	&dev_attr_low_capacity_threshold.attr,
	NULL
};
ATTRIBUTE_GROUPS(sol_trig);

static int sol_trig_activate(struct led_classdev *led_cdev);
static void sol_trig_deactivate(struct led_classdev *led_cdev);

static struct led_trigger sol_led_trigger = {
	.name       = "sign-of-life",
	.activate   = sol_trig_activate,
	.deactivate = sol_trig_deactivate,
	.groups     = sol_trig_groups,
};

static int is_multi_led_enabled () {
	return 0x01;
}


/* API to set sol led colour, brightness etc based on battery status */
static int sol_led_update_status(const char *led_name, bool blink, unsigned int brightness, struct sol_trig_data *sol_data)
{

	struct led_classdev  *led_cdev = sol_data->led_cdev;

	dev_dbg(led_cdev->dev, "Entry sol_led_update_status name:%s, blink = %d, brightness = %u\n",led_name,blink,brightness);
	if(!strcmp(led_name, "charging_full") && !strcmp(led_cdev->name, "charging_full")) {
		dev_dbg(led_cdev->dev, "POWER_STATUS_FULL\n");
		led_trigger_event(&sol_led_trigger, LED_OFF);
		msleep(10); /* MMI_TODO <kernel>: set delay to sync led-trigger with core-led calls */
		led_set_brightness(led_cdev, brightness);
		if(blink){
			led_blink_set(led_cdev, &sol_data->charging_delay_on, &sol_data->charging_delay_off);
			dev_dbg(led_cdev->dev, "SETTING DONE FOR charging full blink delay_on=%lu,delay_off=%lu\n",sol_data->charging_delay_on, sol_data->charging_delay_off);
		}
	}

	if(!strcmp(led_name, "charging") && !strcmp(led_cdev->name, "charging")) {
		dev_dbg(led_cdev->dev, "POWER_STATUS_CHARGING\n");
		led_trigger_event(&sol_led_trigger, LED_OFF);
		msleep(10); /* set delay to sync led-trigger with core-led calls */
		led_set_brightness(led_cdev, brightness);
		if(blink){
			led_blink_set(led_cdev, &sol_data->charging_delay_on, &sol_data->charging_delay_off);
			dev_dbg(led_cdev->dev, "SETTING DONE FOR charging blink delay_on=%lu,delay_off=%lu\n",sol_data->charging_delay_on, sol_data->charging_delay_off);
		}
	}

	if(!strcmp(led_name, "charging_low") && !strcmp(led_cdev->name, "charging_low")) {
		dev_dbg(led_cdev->dev, "POWER_STATUS_LOW\n");
		led_trigger_event(&sol_led_trigger, LED_OFF);
		msleep(10); /* set delay to sync led-trigger with core-led calls */
		led_set_brightness(led_cdev, brightness);
		if(blink){
			led_blink_set(led_cdev, &sol_data->low_cap_delay_on, &sol_data->low_cap_delay_off);
			dev_dbg(led_cdev->dev, "SETTING DONE FOR low cap blink delay_on= %lu,delay_off=%lu\n",sol_data->low_cap_delay_on,sol_data->low_cap_delay_off);
		}
	}
	return 0;
}

static int sol_trig_notifier_fn(struct notifier_block *nb, unsigned long action, void *data)
{
	union power_supply_propval status;
	union power_supply_propval capacity;
	struct sol_trig_data *sol_data = container_of(nb, struct sol_trig_data, nb);
	struct led_classdev  *led_cdev = sol_data->led_cdev;
	struct power_supply  *psy = data;

	/* We want to key the SOL LED off "mmi_battery" if possible. If mmi_battery is
	 * unavailable, try the "battery" PSY. If neither of those are available, just
	 * key off any battery PSY available.
	 */
	if (sol_data->mmi_battery_psy_present)
	{
		if (strcmp(psy->desc->name, "mmi_battery"))
			return NOTIFY_OK;
	}
	else if (sol_data->battery_psy_present)
	{
		if (strcmp(psy->desc->name, "battery"))
			return NOTIFY_OK;
	}
	else if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
	{
		return NOTIFY_OK;
	}

	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &status)) {
		dev_err(led_cdev->dev, "Failed to get PSY status.\n");
		return NOTIFY_OK;
	}
	dev_dbg(led_cdev->dev, "%s: PSY status: %d\n", __func__, status.intval);

	switch (status.intval) {
	case POWER_SUPPLY_STATUS_UNKNOWN: /* Fall-through */
	case POWER_SUPPLY_STATUS_FULL:
		if(is_multi_led_enabled())
			sol_led_update_status("charging_full", false, LED_FULL, sol_data);
		else
			led_set_brightness(led_cdev, LED_FULL);
		break;

	case POWER_SUPPLY_STATUS_CHARGING:
		if(is_multi_led_enabled()) {

		    if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &capacity))
		        dev_err(led_cdev->dev, "%s: Battery capacity: %d\n", __func__, capacity.intval);

		    /* charging_low led blink: Charging capacity <= 15% */
			if (capacity.intval <= sol_data->low_cap_threshold)
			    sol_led_update_status("charging_low", true, LED_FULL, sol_data);

		    /* charging led blink: Charging capacity >15% and <75% */
		    if (capacity.intval > sol_data->low_cap_threshold && capacity.intval < BATT_CAP_THRESHOLD_HIGH)
			    sol_led_update_status("charging", true, LED_FULL, sol_data);

		    /* charging_full led blink: Charging capacity >=75 and < 100% */
		    if (capacity.intval >= BATT_CAP_THRESHOLD_HIGH && capacity.intval <= BATT_CAP_THRESHOLD_FULL)
			    sol_led_update_status("charging_full", true, LED_FULL, sol_data);

		} else
			led_blink_set(led_cdev, &sol_data->charging_delay_on, &sol_data->charging_delay_off);
		break;

	case POWER_SUPPLY_STATUS_DISCHARGING:  /* Fall-through */
	case POWER_SUPPLY_STATUS_NOT_CHARGING: /* Fall-through */
	default:
		/*
		 * If PSY capacity is equal-to or below a specified threshold, use a special low-capacity blink pattern.
		 * Otherwise, set LED to full brightness by default.
		 */
		if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &capacity))
		    dev_dbg(led_cdev->dev, "%s: Battery capacity: %d\n", __func__, capacity.intval);

		if(is_multi_led_enabled()) {

		    /* charging_low led blink: Charging capacity <= 15% */
			if (capacity.intval <= sol_data->low_cap_threshold)
			    sol_led_update_status("charging_low", true, LED_FULL, sol_data);

		    /* charging led solid: Charging capacity >15% and <75% */
		    if (capacity.intval > sol_data->low_cap_threshold && capacity.intval < BATT_CAP_THRESHOLD_HIGH)
			    sol_led_update_status("charging", false, LED_HALF, sol_data);

		    /* charging_full led solid: Charging capacity >=75 and < 100% */
		    if (capacity.intval >= BATT_CAP_THRESHOLD_HIGH && capacity.intval <= BATT_CAP_THRESHOLD_FULL)
			    sol_led_update_status("charging_full", false, LED_HALF, sol_data);
		}
		else
			if(capacity.intval <= sol_data->low_cap_threshold)
				led_blink_set(led_cdev, &sol_data->low_cap_delay_on, &sol_data->low_cap_delay_off);
			else
				led_set_brightness(led_cdev, LED_FULL);
		break;
	}

	return NOTIFY_OK;
}

static int sol_trig_identify_psy(struct device *dev, void *data)
{
	struct sol_trig_data *sol_data = data;
	struct power_supply *psy = dev_get_drvdata(dev);

	if (!strcmp(psy->desc->name, "mmi_battery"))
		sol_data->mmi_battery_psy_present = true;

	if (!strcmp(psy->desc->name, "battery"))
		sol_data->battery_psy_present = true;

	return 0;
}

static int sol_trig_activate(struct led_classdev *led_cdev)
{
	int ret;
	struct sol_trig_data *sol_data;

	sol_data = kzalloc(sizeof(*sol_data), GFP_KERNEL);
	if (!sol_data)
		return -ENOMEM;

	sol_data->nb.notifier_call = sol_trig_notifier_fn;
	ret = power_supply_reg_notifier(&sol_data->nb);
	if (ret) {
		dev_err(led_cdev->dev, "Failed to register notifier: %d\n", ret);
		goto error;
	}

	sol_data->led_cdev = led_cdev;

	sol_data->charging_delay_on  = CHARGING_DELAY_ON_DEFAULT;
	sol_data->charging_delay_off = CHARGING_DELAY_OFF_DEFAULT;
	sol_data->low_cap_delay_on   = LOW_CAP_DELAY_ON_DEFAULT;
	sol_data->low_cap_delay_off  = LOW_CAP_DELAY_OFF_DEFAULT;
	sol_data->low_cap_threshold  = LOW_CAP_THRESHOLD_DEFAULT;
	sol_data->mmi_battery_psy_present = false;
	sol_data->battery_psy_present     = false;

	class_for_each_device(power_supply_class, NULL, sol_data, sol_trig_identify_psy);
	dev_err(led_cdev->dev, " %s, Initial brightness = %d\n",sol_data->led_cdev->name, sol_data->led_cdev->brightness);

	led_set_trigger_data(led_cdev, sol_data);

	led_set_brightness(led_cdev, LED_FULL); /* Set default LED state */

	return 0;

error:
	kfree(sol_data);
	return ret;
}

static void sol_trig_deactivate(struct led_classdev *led_cdev)
{
	struct sol_trig_data *sol_data = led_get_trigger_data(led_cdev);

	power_supply_unreg_notifier(&sol_data->nb);

	kfree(sol_data);

	return;
}

static int __init sol_trig_init(void)
{
	return led_trigger_register(&sol_led_trigger);
}

static void __exit sol_trig_exit(void)
{
	led_trigger_unregister(&sol_led_trigger);
}

module_init(sol_trig_init);
module_exit(sol_trig_exit);

MODULE_AUTHOR("Andy White <awhite6@motorola.com>");
MODULE_DESCRIPTION("Sign-of-life LED trigger");
MODULE_LICENSE("GPL");
