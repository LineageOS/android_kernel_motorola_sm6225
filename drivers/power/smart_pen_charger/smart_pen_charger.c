/*
 * Copyright (C) 2021 Motorola Mobility LLC
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

#define pr_fmt(fmt)     "SMART_PEN_CHG: %s: " fmt, __func__

#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include "qti_glink_charger.h"

#define MAC_ADDR_LEN 6
#define MAX_PSY_NAME_LEN 16
#define CHG_SHOW_MAX_SIZE 50

static bool simulator_enabled;
module_param(simulator_enabled, bool, 0600);
MODULE_PARM_DESC(simulator_enabled, "Enable Pen Simulator");

enum pen_notify_event {
	NOTIFY_EVENT_PEN_ID,
	NOTIFY_EVENT_PEN_STATUS,
	NOTIFY_EVENT_PEN_SOC,
	NOTIFY_EVENT_PEN_MAC,
	NOTIFY_EVENT_PEN_ERROR,
};

enum pen_status {
	PEN_STAT_DETACHED,
	PEN_STAT_ATTACHED,
	PEN_STAT_READY,
	PEN_STAT_CHARGING,
	PEN_STAT_DISCHARGING,
	PEN_STAT_CHARGE_FULL,
	PEN_STAT_CHARGE_DISABLED,
	PEN_STAT_MAX,
};

enum pen_error {
	PEN_OK,
	PEN_ERR_UNKNOWN,
	PEN_ERR_NOT_PLUGIN,
	PEN_ERR_TRANSTER_FAILED,
	PEN_ERR_OVERTEMP,
	PEN_ERR_OVERVOLT,
	PEN_ERR_OVERCURR,
	PEN_ERR_UNDERVOLT,
	PEN_ERR_MAX,
};

static char *pen_status_maps[] = {
	"detached",
	"attached",
	"ready",
	"charging",
	"discharging",
	"charge_full",
	"charge_disabled",
};

struct pen_mac {
	u32 addr[MAC_ADDR_LEN];
};

struct pen_charger_data {
	u32 id;
	u32 soc;
	int32_t error;
	enum pen_status status;
	struct pen_mac mac;
};

struct pen_charger {
	struct device		*dev;
	struct pen_charger_data	pen_data;
	struct pen_charger_data simulator_data;
	struct power_supply	*pen_psy;
	char			*pen_uenvp[2];
	char			pen_psy_name[MAX_PSY_NAME_LEN];
	struct notifier_block   pen_nb;
	struct notifier_block   pen_psy_nb;
};

static struct pen_charger *this_chip = NULL;
static void pen_charger_handle_event(struct pen_charger *chg, int event);

static ssize_t pen_control_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	u32 cmd;
	unsigned long r;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &cmd);
	if (r) {
		pr_err("Invalid pen control cmd = 0x%x\n", cmd);
		return -EINVAL;
	}

	pr_info("Send pen cmd = 0x%x\n", cmd);
	rc = qti_charger_set_property(OEM_PROP_PEN_CTRL,
				&cmd, sizeof(cmd));
	if (rc) {
		pr_err("Failed to set control cmd=0x%x, rc=%d\n", cmd, rc);
		return rc;
	}

	return count;
}
static DEVICE_ATTR_WO(pen_control);

static ssize_t pen_id_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned int id;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &id);
	if (r) {
		pr_err("Invalid pen id = 0x%x\n", id);
		return -EINVAL;
	}

	if (chg->simulator_data.id != id && id != 0) {
		pr_info("Pen id changed by user 0x%x -> 0x%x\n",
				chg->simulator_data.id, id);
		chg->simulator_data.id = id;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_ID);
	}

	return count;
}

static ssize_t pen_id_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int rc;
	u32 id;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%x\n",
					chg->simulator_data.id);

	rc = qti_charger_get_property(OEM_PROP_PEN_ID,
				&id, sizeof(id));
	if (rc) {
		pr_err("Failed to read pen id, rc=%d\n", rc);
		return rc;
	}

	if (chg->pen_data.id != id) {
		pr_info("Pen id updated 0x%x -> 0x%x\n", chg->pen_data.id, id);
		chg->pen_data.id = id;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%x\n", chg->pen_data.id);
}
static DEVICE_ATTR(pen_id, 0664, pen_id_show, pen_id_store);

static ssize_t pen_soc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned int soc;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &soc);
	if (r) {
		pr_err("Invalid pen soc = %d\n", soc);
		return -EINVAL;
	}

	if (soc > 0 && soc <= 100 && chg->simulator_data.soc != soc) {
		pr_info("Pen soc changed by user %d -> %d\n",
					chg->simulator_data.soc, soc);
		chg->simulator_data.soc = soc;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_SOC);
	}

	return count;
}

static ssize_t pen_soc_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int rc;
	u32 soc;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->simulator_data.soc);

	rc = qti_charger_get_property(OEM_PROP_PEN_SOC,
				&soc, sizeof(soc));
	if (rc) {
		pr_err("Failed to read pen soc, rc=%d\n", rc);
		return rc;
	}

	if (soc >= 0 && soc <= 100 && chg->pen_data.soc != soc) {
		pr_info("Pen soc updated %d -> %d\n", chg->pen_data.soc, soc);
		chg->pen_data.soc = soc;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->pen_data.soc);
}
static DEVICE_ATTR(pen_soc, 0664, pen_soc_show, pen_soc_store);

static ssize_t pen_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int i;
	u32 status = PEN_STAT_MAX;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	for (i = 0; i < PEN_STAT_MAX; i++) {
		if (strstr(buf, pen_status_maps[i]) == buf) {
			status = i;
			break;
		}
	}

	if (status >= PEN_STAT_MAX) {
		pr_err("Invalid pen status = %d\n", status);
		return -EINVAL;
	}

	if (chg->simulator_data.status != status) {
		pr_info("Pen status changed by user %s -> %s\n",
				pen_status_maps[chg->simulator_data.status],
				pen_status_maps[status]);
		chg->simulator_data.status = status;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_STATUS);
	}

	return count;
}

static ssize_t pen_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%s\n",
				pen_status_maps[chg->simulator_data.status]);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->pen_data.status);
}
static DEVICE_ATTR(pen_status, 0664, pen_status_show, pen_status_store);

static ssize_t pen_mac_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	struct pen_mac mac;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	rc = sscanf(buf, "%2x:%2x:%2x:%2x:%2x:%2x",
				&mac.addr[0], &mac.addr[1], &mac.addr[2],
				&mac.addr[3], &mac.addr[4], &mac.addr[5]);
	if (rc != MAC_ADDR_LEN) {
		pr_err("Invalid pen mac, rc = %d\n", rc);
		return -EINVAL;
	}

	if (memcmp(&chg->simulator_data.mac, &mac, sizeof(mac))) {
		pr_info("Pen mac changed by user, mac=%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
		memcpy(&chg->simulator_data.mac, &mac, sizeof(mac));
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_MAC);
	}

	return count;
}

static ssize_t pen_mac_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int i;
	int rc;
	struct pen_mac mac;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled) {
		memcpy(&mac, &chg->simulator_data.mac, sizeof(mac));
		return scnprintf(buf, CHG_SHOW_MAX_SIZE,
				"%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
	}

	rc = qti_charger_get_property(OEM_PROP_PEN_MAC,
				&mac, sizeof(mac));
	if (rc) {
		pr_err("Failed to read pen mac, rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < MAC_ADDR_LEN; i++)
		mac.addr[i] &= 0xFF;

	if (memcmp(&chg->pen_data.mac, &mac, sizeof(mac))) {
		pr_info("Pen mac updated, mac=%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
		memcpy(&chg->pen_data.mac, &mac, sizeof(mac));
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%2x:%2x:%2x:%2x:%2x:%2x\n",
				mac.addr[0], mac.addr[1], mac.addr[2],
				mac.addr[3], mac.addr[4], mac.addr[5]);
}
static DEVICE_ATTR(pen_mac, 0664, pen_mac_show, pen_mac_store);

static ssize_t pen_error_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned int error;
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtouint(buf, 0, &error);
	if (r) {
		pr_err("Invalid pen error = %d\n", error);
		return -EINVAL;
	}

	if (chg->simulator_data.error != error) {
		pr_info("Pen error changed by user %d -> %d\n",
					chg->simulator_data.error, error);
		chg->simulator_data.error = error;
		if (simulator_enabled)
			pen_charger_handle_event(chg, NOTIFY_EVENT_PEN_ERROR);
	}

	return count;
}

static ssize_t pen_error_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct pen_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	if (simulator_enabled)
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->simulator_data.error);
	else
		return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n",
					chg->pen_data.error);
}
static DEVICE_ATTR(pen_error, 0664, pen_error_show, pen_error_store);

static void pen_charger_psy_init(struct pen_charger *chg)
{
	int rc;

	if (chg->pen_psy)
		return;

	chg->pen_psy = power_supply_get_by_name(chg->pen_psy_name);
	if (!chg->pen_psy) {
		pr_err("No pen power supply found\n");
		return;
	}
	pr_info("Pen power supply %s is found\n", chg->pen_psy_name);

	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_control);
        if (rc)
		pr_err("couldn't create pen control\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_id);
        if (rc)
		pr_err("couldn't create pen id\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_soc);
        if (rc)
		pr_err("couldn't create pen soc\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_status);
        if (rc)
		pr_err("couldn't create pen status\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_mac);
        if (rc)
		pr_err("couldn't create pen mac\n");
	rc = device_create_file(chg->pen_psy->dev.parent,
				&dev_attr_pen_error);
        if (rc)
		pr_err("couldn't create pen error\n");
}

static void pen_charger_handle_event(struct pen_charger *chg, int event)
{
	char *event_string = NULL;
	struct pen_charger_data *data = NULL;

	event_string = kmalloc(CHG_SHOW_MAX_SIZE, GFP_KERNEL);
	if (!event_string)
		return;

	if (!simulator_enabled)
		data = &chg->pen_data;
	else
		data = &chg->simulator_data;

	switch (event) {
	case NOTIFY_EVENT_PEN_ID:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_ID=0x%x",
			data->id);
		break;
	case NOTIFY_EVENT_PEN_STATUS:
		if (data->status == PEN_STAT_DETACHED) {
			pr_warn("Pen has been deteched, clean pen data!\n");
			memset(data, 0, sizeof(struct pen_charger_data));
		}
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_STATUS=%s",
			pen_status_maps[data->status]);
		chg->pen_data.status = data->status;
		if (chg->pen_psy)
			sysfs_notify(&chg->pen_psy->dev.parent->kobj, NULL, "pen_status");
		break;
	case NOTIFY_EVENT_PEN_SOC:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_SOC=%d",
			data->soc);
		if (chg->pen_psy)
			sysfs_notify(&chg->pen_psy->dev.parent->kobj, NULL, "pen_soc");
		break;
	case NOTIFY_EVENT_PEN_MAC:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_MAC=%2x:%2x:%2x:%2x:%2x:%2x",
			data->mac.addr[0], data->mac.addr[1],
			data->mac.addr[2], data->mac.addr[3],
			data->mac.addr[4], data->mac.addr[5]);
		break;
	case NOTIFY_EVENT_PEN_ERROR:
		scnprintf(event_string, CHG_SHOW_MAX_SIZE,
			"POWER_SUPPLY_PEN_ERROR=%d",
			data->error);
		chg->pen_data.error = data->error;
		if (chg->pen_psy)
			sysfs_notify(&chg->pen_psy->dev.parent->kobj, NULL, "pen_error");
		break;
	default:
		pr_err("Invalid notify event %d\n", event);
		kfree(event_string);
		return;
	}

	if (!chg->pen_psy) {
		kfree(event_string);
		pr_warn("Pen power supply is unavailable\n");
		return;
	}

	if (chg->pen_uenvp[0])
		kfree(chg->pen_uenvp[0]);
	chg->pen_uenvp[0] = event_string;
	chg->pen_uenvp[1] = NULL;
	kobject_uevent_env(&chg->pen_psy->dev.kobj,
			KOBJ_CHANGE,
			chg->pen_uenvp);
	pr_info("Send pen event: %s\n", event_string);
}

static int pen_charger_notify_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int i;
	bool pen_changed = 0;
	struct qti_charger_notify_data *notify_data = data;
	struct pen_charger *chg = container_of(nb, struct pen_charger, pen_nb);

	if (notify_data->receiver != OEM_NOTIFY_RECEIVER_PEN_CHG) {
		pr_err("Skip mis-matched receiver: %#x\n", notify_data->receiver);
		return 0;
	}

        switch (event) {
        case NOTIFY_EVENT_PEN_ID:
	/* PEN ID update */
		pen_changed = 1;
		chg->pen_data.id = notify_data->data[0];
                break;
        case NOTIFY_EVENT_PEN_STATUS:
	/*
	 * PEN status changes: attached, ready, charging,
	 * discharging, charged, detached
	 */
		pen_changed = 1;
		chg->pen_data.status = notify_data->data[0];
		break;
        case NOTIFY_EVENT_PEN_SOC:
	/* PEN status of charge changes */
		pen_changed = 1;
		chg->pen_data.soc = notify_data->data[0];
                break;
        case NOTIFY_EVENT_PEN_MAC:
	/* PEN MAC update */
		pen_changed = 1;
		for (i = 0; i < MAC_ADDR_LEN; i++)
			chg->pen_data.mac.addr[i] = notify_data->data[i] & 0xFF;
                break;
        case NOTIFY_EVENT_PEN_ERROR:
	/* PEN occurs error */
		pen_changed = 1;
		chg->pen_data.error = notify_data->data[0];
                break;
        default:
		pr_err("Unknown pen event: %#lx\n", event);
                break;
        }

	if (pen_changed) {
		pr_err("pen event: %#lx\n", event);
		power_supply_changed(chg->pen_psy);
		pen_charger_handle_event(chg, (int)event);
	}

        return 0;
}

static int pen_charger_psy_notify_callback(struct notifier_block *nb,
				unsigned long val, void *v)
{
	struct pen_charger *chg = container_of(nb,
				struct pen_charger, pen_psy_nb);
	struct power_supply *psy = v;

	if (!chg) {
		pr_err("called before pen charger valid!\n");
		return NOTIFY_DONE;
	}

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (psy &&
	    !chg->pen_psy &&
	    !strcmp(psy->desc->name, chg->pen_psy_name)) {
		pen_charger_psy_init(chg);
	}

        return NOTIFY_OK;
}

static int pen_charger_parse_dt(struct pen_charger *chg)
{
	int rc;
	const char *psy_name = NULL;
	struct device_node *node = chg->dev->of_node;

	rc = of_property_read_string(node, "mmi,pen-psy-name", &psy_name);
	if (rc || !psy_name)
		psy_name = "wireless";
	strncpy(chg->pen_psy_name, psy_name, MAX_PSY_NAME_LEN);

	return 0;
}

static int pen_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg;
	int rc;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	memset(&chg->pen_data, 0, sizeof(struct pen_charger_data));
	memset(&chg->simulator_data, 0, sizeof(struct pen_charger_data));
	chg->pen_nb.notifier_call = pen_charger_notify_callback;
	rc = qti_charger_register_notifier(&chg->pen_nb);
	if (rc) {
		pr_err("Failed to register notifier, rc=%d\n", rc);
		return rc;
	}
	chg->pen_psy_nb.notifier_call = pen_charger_psy_notify_callback;
	rc = power_supply_reg_notifier(&chg->pen_psy_nb);
	if (rc) {
		pr_err("Failed to register psy notifier, rc=%d\n", rc);
		return rc;
	}

	rc = pen_charger_parse_dt(chg);
	if (rc) {
		pr_err("Failed to parse devicetree, rc=%d\n", rc);
		return rc;
	}

	this_chip = chg;
	pen_charger_psy_init(chg);

	return 0;
}

static int pen_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_charger *chg= dev_get_drvdata(dev);

	if (chg->pen_psy) {
		if (chg->pen_uenvp[0]) {
			kfree(chg->pen_uenvp[0]);
			chg->pen_uenvp[0] = NULL;
		}
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_control);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_id);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_soc);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_status);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_mac);
		device_remove_file(chg->pen_psy->dev.parent, &dev_attr_pen_error);
		power_supply_put(chg->pen_psy);
	}

	power_supply_unreg_notifier(&chg->pen_psy_nb);
	qti_charger_unregister_notifier(&chg->pen_nb);

	return 0;
}

static const struct of_device_id pen_charger_match_table[] = {
	{.compatible = "mmi,pen-charger"},
	{},
};

static struct platform_driver pen_charger_driver = {
	.driver	= {
		.name = "pen_charger",
		.of_match_table = pen_charger_match_table,
	},
	.probe	= pen_charger_probe,
	.remove	= pen_charger_remove,
};

module_platform_driver(pen_charger_driver);

MODULE_DESCRIPTION("Smart Pen Charger Driver");
MODULE_LICENSE("GPL v2");
