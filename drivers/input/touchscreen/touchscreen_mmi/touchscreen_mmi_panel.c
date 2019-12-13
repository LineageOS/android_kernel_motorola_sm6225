/*
 * Copyright (C) 2019 Motorola Mobility LLC
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/touchscreen_mmi.h>
#include <linux/of.h>

static ssize_t ts_mmi_panel_supplier_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s\n", touch_cdev->panel_supplier);
}
static DEVICE_ATTR(panel_supplier, S_IRUGO, ts_mmi_panel_supplier_show, NULL);

static struct attribute *sysfs_panel_attrs[] = {
	&dev_attr_panel_supplier.attr,
	NULL,
};

static const struct attribute_group sysfs_panel_group = {
        .attrs = sysfs_panel_attrs,
};

int ts_mmi_parse_dt(struct ts_mmi_dev *touch_cdev,
	struct device_node *of_node)
{
	struct device_node *chosen;
	struct ts_mmi_dev_pdata *ppdata = &touch_cdev->pdata;

	dev_info(DEV_TS, "%s: Start parse touchscreen mmi device tree.\n", __func__);
	if (!of_property_read_string(of_node, "mmi,class-entry-name", &ppdata->class_entry_name))
		dev_info(DEV_TS, "%s: class-entry-name property %s\n",
				__func__, ppdata->class_entry_name);

	if (!of_property_read_string(of_node, "mmi,bound-display", &ppdata->bound_display))
		dev_info(DEV_TS, "%s: bound-display property %s\n",
				__func__, ppdata->bound_display);

	if (!of_property_read_u32(of_node, "mmi,control-dsi", &ppdata->ctrl_dsi))
		dev_info(DEV_TS, "%s: ctrl-dsi property %d\n",
				__func__, ppdata->ctrl_dsi);

	if (of_property_read_bool(of_node, "mmi,usb-charger-detection")) {
		dev_info(DEV_TS, "%s: using usb detection\n", __func__);
		ppdata->usb_detection = true;
	}

	if (!of_property_read_u32(of_node, "mmi,reset-on-resume", &ppdata->reset))
		dev_info(DEV_TS, "%s: using %u reset on resume\n",
				__func__, ppdata->reset);

	if (of_property_read_bool(of_node, "mmi,power-off-suspend")) {
		dev_info(DEV_TS, "%s: using power off in suspend\n", __func__);
		ppdata->power_off_suspend = true;
		if (ppdata->reset) {
			ppdata->reset = 0;
			dev_info(DEV_TS, "%s: unset reset on resume!!!\n", __func__);
		}
	}

	if (of_property_read_bool(of_node, "mmi,enable-gestures")) {
		dev_info(DEV_TS, "%s: using enable gestures\n", __func__);
		ppdata->gestures_enabled = true;
	}

	if (of_property_read_bool(of_node, "mmi,refresh-rate-update")) {
		dev_info(DEV_TS, "%s: using refresh rate update\n", __func__);
		ppdata->update_refresh_rate = true;
	}

	chosen = of_find_node_by_name(NULL, "chosen");
	if (chosen) {
		struct device_node *child;
		struct device_node *np;
		const char *supplier;
		char *s, *d;
		int rc;
		u64 panel_ver, panel_id;

		rc = of_property_read_string(chosen, "mmi,panel_name",
					(const char **)&supplier);
		if (rc) {
			dev_err(DEV_TS, "%s: cannot read mmi,panel_name %d\n",
					__func__, rc);
			goto done;
		}
		dev_info(DEV_TS, "%s: mmi,panel_name %s\n", __func__, supplier);

		s = (char *)supplier;
		/* skip dsi_ part */
		if (!strncmp(supplier, "dsi_", 4))
			s += 4;
		d = touch_cdev->panel_supplier;
		while (*s != '_') *d++ = *s++;

		rc = of_property_read_u64(chosen, "mmi,panel_id", &panel_id);
		if (rc) {
			dev_err(DEV_TS, "%s: cannot read mmi,panel_id %d\n", __func__, rc);
			goto done;
		}
		rc = of_property_read_u64(chosen, "mmi,panel_ver", &panel_ver);
		if (rc) {
			dev_err(DEV_TS, "%s: cannot read mmi,panel_ver %d\n", __func__, rc);
			goto done;
		}
		of_node_put(chosen);
		dev_dbg(DEV_TS, "%s: [%s] id=%llx ver=%llx\n",
					__func__, chosen->name, panel_id, panel_ver);

		np = of_find_node_by_name(of_node, "mmi,panel-mappings");
		if (!np) {
			dev_err(DEV_TS, "%s: cannot read mmi,panel-mappings\n", __func__);
			goto done;
		}

		for_each_child_of_node(np, child) {
			u64 id, ver;

			rc = of_property_read_u64(child, "panel-id", &id);
			if (rc) {
				dev_err(DEV_TS, "%s: cannot read panel-id %d\n", __func__, rc);
				goto done;
			}
			rc = of_property_read_u64(child, "panel-ver", &ver);
			if (rc) {
				dev_err(DEV_TS, "%s: cannot read panel-ver %d\n", __func__, rc);
				goto done;
			}

			dev_dbg(DEV_TS, "%s: [%s] id=%llx ver=%llx\n",
					__func__, child->name, id, ver);
			of_node_put(child);

			if (id == panel_id && ver == panel_ver) {
				strlcpy(touch_cdev->panel_supplier, child->name,
					sizeof(touch_cdev->panel_supplier));
				break;
			}
		}

		of_node_put(np);
		dev_info(DEV_TS, "%s: panel-supplier %s\n",
				__func__, touch_cdev->panel_supplier);
	}

done:
	return 0;
}

int ts_mmi_panel_register(struct ts_mmi_dev *touch_cdev) {
	int ret = 0;

	if (!touch_cdev->class_dev) {
		return -ENODEV;
	}

	dev_info(DEV_TS, "%s: Start panel init.\n", __func__);
	ret = sysfs_create_group(&touch_cdev->class_dev->kobj, &sysfs_panel_group);
	if (ret)
		goto PANEL_DEVICE_ATTR_CREATE_FAILED;

	dev_info(DEV_TS, "%s: Panel init OK.\n", __func__);
	return 0;

PANEL_DEVICE_ATTR_CREATE_FAILED:
	return ret;
}

void ts_mmi_panel_unregister(struct ts_mmi_dev *touch_cdev) {
	if(touch_cdev->class_dev == NULL) {
		dev_err(DEV_MMI, "%s:touch_cdev->class_dev == NULL", __func__);
	} else {
		sysfs_remove_group(&touch_cdev->class_dev->kobj, &sysfs_panel_group);
		dev_info(DEV_MMI, "%s:panel_unregister finish", __func__);
	}
}
