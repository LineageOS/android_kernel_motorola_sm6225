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
#include <linux/slab.h>

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

#define PRIM_PANEL_NAME	"mmi,panel_name"
#define PRIM_PANEL_VER	"mmi,panel_ver"
#define PRIM_PANEL_ID	"mmi,panel_id"
#define SEC_PANEL_NAME	PRIM_PANEL_NAME"_s"
#define SEC_PANEL_VER	PRIM_PANEL_VER"_s"
#define SEC_PANEL_ID	PRIM_PANEL_ID"_s"

int ts_mmi_parse_dt(struct ts_mmi_dev *touch_cdev,
	struct device_node *of_node)
{
	struct device_node *chosen;
	struct ts_mmi_dev_pdata *ppdata = &touch_cdev->pdata;
	u32 coords[2];

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

	if (of_property_read_bool(of_node, "mmi,fw-load-on-resume")) {
		dev_info(DEV_TS, "%s: load firmware on resume\n", __func__);
		ppdata->fw_load_resume = true;
	}

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

	if (of_property_read_bool(of_node, "mmi,enable-palm")) {
		dev_info(DEV_TS, "%s: using enable palm\n", __func__);
		ppdata->palm_enabled = true;
	}

	if (of_property_read_bool(of_node, "mmi,refresh-rate-update")) {
		dev_info(DEV_TS, "%s: using refresh rate update\n", __func__);
		ppdata->update_refresh_rate = true;
	}

	if (of_property_read_bool(of_node, "mmi,suppression-control")) {
		dev_info(DEV_TS, "%s: using suppression\n", __func__);
		ppdata->suppression_ctrl = true;
	}

	if (of_property_read_bool(of_node, "mmi,pill-region-control")) {
		dev_info(DEV_TS, "%s: using pill region\n", __func__);
		ppdata->pill_region_ctrl = true;
	}

	if (of_property_read_bool(of_node, "mmi,hold-distance-control")) {
		dev_info(DEV_TS, "%s: using hold distance\n", __func__);
		ppdata->hold_distance_ctrl = true;
	}

	if (of_property_read_bool(of_node, "mmi,gs-distance-control")) {
		dev_info(DEV_TS, "%s: using gs distance\n", __func__);
		ppdata->gs_distance_ctrl = true;
	}

	if (of_property_read_bool(of_node, "mmi,hold-grip-control")) {
		dev_info(DEV_TS, "%s: using hold grip\n", __func__);
		ppdata->hold_grip_ctrl = true;
	}

	if (of_property_read_bool(of_node, "mmi,poison-slot-control")) {
		dev_info(DEV_TS, "%s: using poison slot\n", __func__);
		ppdata->poison_slot_ctrl = true;
	}

	if (of_property_read_bool(of_node, "mmi,fps_detection")) {
		dev_info(DEV_TS, "%s: using fps detection\n", __func__);
		ppdata->fps_detection = true;
	}

	if (!of_property_read_u32_array(of_node, "mmi,max_coords", coords, 2)) {
		dev_info(DEV_TS, "%s: get max_coords property\n", __func__);
		ppdata->max_x = coords[0] - 1;
		ppdata->max_y = coords[1] - 1;
	}

	chosen = of_find_node_by_name(NULL, "chosen");
	if (chosen) {
		struct device_node *child;
		struct device_node *np;
		const char *panel_name_prop;
		const char *panel_ver_prop;
		const char *panel_id_prop;
		const char *supplier;
		char *s, *d;
		int rc;
		u64 panel_ver, panel_id;

		if (!ppdata->ctrl_dsi) {
			panel_name_prop = PRIM_PANEL_NAME;
			panel_ver_prop = PRIM_PANEL_VER;
			panel_id_prop = PRIM_PANEL_ID;
		} else {
			panel_name_prop = SEC_PANEL_NAME;
			panel_ver_prop = SEC_PANEL_VER;
			panel_id_prop = SEC_PANEL_ID;
		}

		rc = of_property_read_string(chosen, panel_name_prop,
					(const char **)&supplier);
		if (rc) {
			dev_err(DEV_TS, "%s: cannot read %s %d\n",
					__func__, panel_name_prop, rc);
			goto done;
		}
		dev_info(DEV_TS, "%s: %s %s\n",
					__func__, panel_name_prop, supplier);
		s = (char *)supplier;
		/* skip dsi_ part */
		if (!strncmp(supplier, "dsi_", 4))
			s += 4;
		d = touch_cdev->panel_supplier;
		while (*s != '_') *d++ = *s++;

		rc = of_property_read_u64(chosen, panel_id_prop, &panel_id);
		if (rc) {
			dev_err(DEV_TS, "%s: cannot read %s %d\n",
					__func__, panel_id_prop, rc);
			goto done;
		}
		rc = of_property_read_u64(chosen, panel_ver_prop, &panel_ver);
		if (rc) {
			dev_err(DEV_TS, "%s: cannot read %s %d\n",
					__func__, panel_ver_prop, rc);
			goto done;
		}
		of_node_put(chosen);
		dev_dbg(DEV_TS, "%s: [%s] id=%llx ver=%llx\n",
					__func__, chosen->name, panel_id, panel_ver);

		np = of_find_node_by_name(of_node, "mmi,panel-mappings");
		if (!np) {
			dev_info(DEV_TS, "%s: no panel mappings\n", __func__);
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

#if defined(CONFIG_DRM_PANEL_NOTIFICATIONS) || defined (CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS)
struct drm_panel *active_panel;

static int ts_mmi_check_dt(struct device_node *np)
{

	int i = 0;
	int count = 0;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0) {
		pr_err("%s: find drm_panel count(%d) fail", __func__, count);
		return -ENODEV;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			pr_info("%s: find drm_panel successfully", __func__);
			active_panel = panel;
			return 0;
		}
	}
	pr_err("%s: No find drm_panel", __func__);
	return -ENODEV;
}

static int ts_mmi_check_default_tp(struct device_node *dt, const char *prop)
{
	const char **active_tp = NULL;
	int count, tmp, score = 0;
	const char *active;
	int ret, i;

	count = of_property_count_strings(dt->parent, prop);
	if (count <= 0)
		return -ENODEV;

	active_tp = kcalloc(count, sizeof(char *),  GFP_KERNEL);
	if (!active_tp) {
		pr_err("%s: alloc active_tp failed", __func__);
		return -ENOMEM;
	}

	ret = of_property_read_string_array(dt->parent, prop,
			active_tp, count);
	if (ret < 0) {
		pr_err("%s: fail to read %s (%d)\n", __func__, prop, ret);
		ret = -ENODEV;
		goto out;
	}

	for (i = 0; i < count; i++) {
		active = active_tp[i];
		if (active != NULL) {
			tmp = of_device_is_compatible(dt, active);
			if (tmp > 0)
				score++;
		}
	}

	if (score <= 0) {
		pr_err("%s: Not panel match this driver\n", __func__);
		ret = -ENODEV;
		goto out;
	}
	ret = 0;
out:
	kfree(active_tp);
	return ret;
}

int ts_mmi_check_drm_panel(struct device_node *of_node)
{
	int ret;

	ret = ts_mmi_check_dt(of_node);
	if (ret) {
		pr_err( "%s: parse drm-panel fail\n", __func__);
		if (!ts_mmi_check_default_tp(of_node, "qcom,mmi-touch-active"))
			ret = -EPROBE_DEFER;
		else
			ret = -ENODEV;
	}
	return ret;
}
#endif

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
