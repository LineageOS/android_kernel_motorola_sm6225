/*
 * Copyright (C) 2022 Motorola Mobility LLC
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

#define pr_fmt(fmt)     "LPD_MITIGATE: %s: " fmt, __func__

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
#include <linux/mmi_relay.h>

#include "qti_glink_charger.h"

#define LPD_SHOW_MAX_SIZE 128

struct lpd_info {
	int lpd_present;
	int lpd_rsbu1;
	int lpd_rsbu2;
};

/*
 * Lpd Mitigation Mode
 */
enum {
	LPD_MITIGATE_DRP,
	LPD_MITIGATE_SNK,
	LPD_MITIGATE_SRC,
	LPD_MITIGATE_DISABLE,
	LPD_MITIGATE_INVALID,
};

enum {
	LPD_STATUS_OFFLINE = 0,
	LPD_STATUS_ONLINE = 1,
};

struct lpd_mitigate_device {
	struct device *dev;
	u32 lpd_status;
	u32 tud_status;
	u32 lpd_mitigate_mode;
	u32 mitigate_mode_cfg;
	u32 def_mitigate_mode_cfg;
	struct work_struct lpd_mitigate_work;
	struct notifier_block lpd_mitigate_notify;
	struct notifier_block lpd_psy_notifier;
	struct power_supply *batt_psy;
	char *uenvp[2];
};

static void lpd_notify_uevent(struct lpd_mitigate_device *lpd_device, int event)
{
	char uevent_buf[LPD_SHOW_MAX_SIZE];

	if (!lpd_device->batt_psy) {
		lpd_device->batt_psy = power_supply_get_by_name("battery");
		if (!lpd_device->batt_psy) {
			pr_err("No battery supply found\n");
			return;
		}
	}

	memset(uevent_buf, '\0', LPD_SHOW_MAX_SIZE);
	if (event == NOTIFY_EVENT_LPD_STATUS) {
		scnprintf(uevent_buf, LPD_SHOW_MAX_SIZE,
			"POWER_SUPPLY_LPD_PRESENT=%s",
			lpd_device->lpd_status? "true" : "false");
	} else if (event == NOTIFY_EVENT_TUD_STATUS) {
		scnprintf(uevent_buf, LPD_SHOW_MAX_SIZE,
			"POWER_SUPPLY_TUD_PRESENT=%s",
			lpd_device->tud_status? "true" : "false");
	}
	lpd_device->uenvp[0] = uevent_buf;
	lpd_device->uenvp[1] = NULL;
	kobject_uevent_env(&lpd_device->batt_psy->dev.kobj,
		KOBJ_CHANGE, lpd_device->uenvp);
}

static int lpd_mitigate_notify_cb(struct notifier_block *self,
				unsigned long event, void *p)
{
	struct lpd_mitigate_device *lpd_device = container_of(self,
			struct lpd_mitigate_device, lpd_mitigate_notify);

	if (lpd_device && event == NOTIFY_EVENT_TUD_STATUS) {
		u32 tud_status = *(int *)p;
		pr_info("TUD status notify: %d\n", tud_status);
		if (tud_status > 0)
			tud_status = LPD_STATUS_ONLINE;
		else
			tud_status = LPD_STATUS_OFFLINE;
		if (tud_status != lpd_device->tud_status) {
			pr_info("TUD status transit: %d -> %d\n",
					lpd_device->tud_status, tud_status);
			lpd_device->tud_status = tud_status;
			schedule_work(&lpd_device->lpd_mitigate_work);
		}
	}

	return 0;
}

static void lpd_mitigate_work(struct work_struct *work)
{
	int rc;
	struct lpd_info lpd_info = {0};
	u32 lpd_status;
	u32 lpd_mitigate_mode;
	static u32 tud_status = INT_MAX;
	struct lpd_mitigate_device *lpd_device = container_of(work,
			struct lpd_mitigate_device, lpd_mitigate_work);

	if (!lpd_device) {
		pr_err("Invalid lpd device\n");
		return;
	}

	if (tud_status != lpd_device->tud_status ||
	    lpd_device->tud_status == INT_MAX) {
		if (lpd_device->tud_status == INT_MAX)
			lpd_device->tud_status = LPD_STATUS_OFFLINE;
		tud_status = lpd_device->tud_status;
		lpd_notify_uevent(lpd_device, NOTIFY_EVENT_TUD_STATUS);
	}

	rc = qti_charger_get_property(OEM_PROP_LPD_INFO,
				&lpd_info, sizeof(lpd_info));
	if (rc) {
		pr_err("Failed to read lpd info, rc=%d\n", rc);
		return;
	}

	lpd_status = lpd_device->lpd_status;
	if (lpd_info.lpd_present &&
	    lpd_device->lpd_status != LPD_STATUS_ONLINE) {
		lpd_status = LPD_STATUS_ONLINE;
	} else if (!lpd_info.lpd_present &&
	    lpd_device->lpd_status != LPD_STATUS_OFFLINE) {
		lpd_status = LPD_STATUS_OFFLINE;
	}
	if (lpd_status != lpd_device->lpd_status) {
		pr_info("LPD status transit: %d -> %d\n",
				lpd_device->lpd_status, lpd_status);
		lpd_device->lpd_status = lpd_status;
		relay_notifier_fire(BLOCKING, LPD,
				NOTIFY_EVENT_LPD_STATUS,
				(void *)&lpd_status);
		lpd_notify_uevent(lpd_device, NOTIFY_EVENT_LPD_STATUS);
	}

	lpd_mitigate_mode = lpd_device->lpd_mitigate_mode;
	if (lpd_device->lpd_status == LPD_STATUS_ONLINE &&
	    lpd_device->tud_status == LPD_STATUS_ONLINE &&
	    lpd_device->lpd_mitigate_mode != lpd_device->mitigate_mode_cfg) {
		lpd_mitigate_mode = lpd_device->mitigate_mode_cfg;
	} else if (lpd_device->lpd_status == LPD_STATUS_OFFLINE &&
	    lpd_device->lpd_mitigate_mode != lpd_device->def_mitigate_mode_cfg) {
		lpd_mitigate_mode = lpd_device->def_mitigate_mode_cfg;
	} else if (lpd_device->lpd_mitigate_mode >= LPD_MITIGATE_INVALID) {
		lpd_mitigate_mode = lpd_device->def_mitigate_mode_cfg;
	}
	if (lpd_mitigate_mode != lpd_device->lpd_mitigate_mode) {
		rc = qti_charger_set_property(OEM_PROP_LPD_MITIGATE_MODE,
				&lpd_mitigate_mode,
				sizeof(lpd_mitigate_mode));
		if (rc) {
			pr_err("Failed to set lpd mitigate mode, rc=%d\n", rc);
		} else {
			pr_info("LPD mitigate mode transit: %d -> %d\n",
					lpd_device->lpd_mitigate_mode,
					lpd_mitigate_mode);
			lpd_device->lpd_mitigate_mode = lpd_mitigate_mode;
		}
	}
}

static int lpd_psy_notifier_call(struct notifier_block *nb, unsigned long val,
                                 void *v)
{
	struct lpd_mitigate_device *lpd_device = container_of(nb,
                                struct lpd_mitigate_device, lpd_psy_notifier);
        struct power_supply *psy = v;

        if (!lpd_device) {
                pr_err("called before lpd_device valid!\n");
                return NOTIFY_DONE;
        }

        if (val != PSY_EVENT_PROP_CHANGED)
                return NOTIFY_OK;

        if (psy && strcmp(psy->desc->name, "usb") == 0) {
		schedule_work(&lpd_device->lpd_mitigate_work);
        }

        return NOTIFY_OK;
}

static ssize_t def_mitigate_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct lpd_mitigate_device *lpd_device = dev_get_drvdata(dev);

	if (!lpd_device) {
		pr_err("lpd device not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid mode = %lu\n", mode);
		return -EINVAL;
	}

	if (mode >= LPD_MITIGATE_INVALID)
		lpd_device->def_mitigate_mode_cfg = LPD_MITIGATE_INVALID;
	else
		lpd_device->def_mitigate_mode_cfg = mode;

	schedule_work(&lpd_device->lpd_mitigate_work);

	return count;
}

static ssize_t def_mitigate_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct lpd_mitigate_device *lpd_device = dev_get_drvdata(dev);

	if (!lpd_device) {
		pr_err("lpd device not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, LPD_SHOW_MAX_SIZE, "%d\n",
		lpd_device->def_mitigate_mode_cfg);
}
DEVICE_ATTR_RW(def_mitigate_mode);

static ssize_t mitigate_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct lpd_mitigate_device *lpd_device = dev_get_drvdata(dev);

	if (!lpd_device) {
		pr_err("lpd device not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid mode = %lu\n", mode);
		return -EINVAL;
	}

	if (mode >= LPD_MITIGATE_INVALID)
		lpd_device->mitigate_mode_cfg = LPD_MITIGATE_INVALID;
	else
		lpd_device->mitigate_mode_cfg = mode;

	schedule_work(&lpd_device->lpd_mitigate_work);

	return count;
}

static ssize_t mitigate_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct lpd_mitigate_device *lpd_device = dev_get_drvdata(dev);

	if (!lpd_device) {
		pr_err("lpd device not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, LPD_SHOW_MAX_SIZE, "%d\n",
		lpd_device->mitigate_mode_cfg);
}
DEVICE_ATTR_RW(mitigate_mode);

static int lpd_mitigate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lpd_mitigate_device *lpd_device;
	int rc;

	lpd_device = devm_kzalloc(dev, sizeof(*lpd_device), GFP_KERNEL);
	if (!lpd_device)
		return -ENOMEM;

	INIT_WORK(&lpd_device->lpd_mitigate_work, lpd_mitigate_work);
	platform_set_drvdata(pdev, lpd_device);
	lpd_device->dev = dev;
	lpd_device->lpd_status = INT_MAX;
	lpd_device->tud_status = INT_MAX;
	lpd_device->lpd_mitigate_mode = INT_MAX;

	rc = of_property_read_u32(dev->of_node,
				"mmi,default-mitigate-mode-cfg",
				&lpd_device->def_mitigate_mode_cfg);
	if (rc || lpd_device->def_mitigate_mode_cfg > LPD_MITIGATE_INVALID)
		lpd_device->mitigate_mode_cfg = LPD_MITIGATE_SNK;

	rc = of_property_read_u32(dev->of_node,
				"mmi,mitigate-mode-cfg",
				&lpd_device->mitigate_mode_cfg);
	if (rc || lpd_device->mitigate_mode_cfg > LPD_MITIGATE_INVALID)
		lpd_device->mitigate_mode_cfg = LPD_MITIGATE_DISABLE;

	lpd_device->lpd_mitigate_notify.notifier_call = lpd_mitigate_notify_cb;
	/*register a blocking notification to receive LPD events*/
	rc = relay_register_action(BLOCKING, LPD, &lpd_device->lpd_mitigate_notify);
	if (rc < 0) {
		pr_err("Failed to register tud_notifier: %d\n", rc);
		return rc;
	}

        lpd_device->lpd_psy_notifier.notifier_call = lpd_psy_notifier_call;
	rc = power_supply_reg_notifier(&lpd_device->lpd_psy_notifier);
	if (rc) {
		pr_err("Failed to register lpd_psy_notifier: %d\n", rc);
		return rc;
	}

	rc = device_create_file(lpd_device->dev, &dev_attr_def_mitigate_mode);
        if (rc) {
		pr_err("Couldn't create default mitigate mode\n");
		return rc;
	}
	rc = device_create_file(lpd_device->dev, &dev_attr_mitigate_mode);
        if (rc) {
		pr_err("Couldn't create mitigate mode\n");
		return rc;
	}

	pr_info("lpd mitigate driver probe complete\n");
	return 0;
}

static int lpd_mitigate_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lpd_mitigate_device *lpd_device = dev_get_drvdata(dev);

	cancel_work_sync(&lpd_device->lpd_mitigate_work);
	device_remove_file(lpd_device->dev, &dev_attr_mitigate_mode);
	device_remove_file(lpd_device->dev, &dev_attr_def_mitigate_mode);
	power_supply_unreg_notifier(&lpd_device->lpd_psy_notifier);
	relay_unregister_action(BLOCKING, LPD, &lpd_device->lpd_mitigate_notify);
	if (lpd_device->batt_psy) {
		power_supply_put(lpd_device->batt_psy);
		lpd_device->batt_psy = NULL;
	}

	return 0;
}

static const struct of_device_id lpd_mitigate_match_table[] = {
	{.compatible = "mmi,lpd-mitigate"},
	{},
};

static struct platform_driver lpd_mitigate_driver = {
	.driver	= {
		.name = "mmi_lpd_mitigate",
		.of_match_table = lpd_mitigate_match_table,
	},
	.probe	= lpd_mitigate_probe,
	.remove	= lpd_mitigate_remove,
};

module_platform_driver(lpd_mitigate_driver);

MODULE_DESCRIPTION("MMI LPD Mitigate Driver");
MODULE_LICENSE("GPL v2");
