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
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>
#include <linux/touchscreen_mmi.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/slab.h>

#define FMT_STRING	"%s"
#define FMT_INTEGER	"%d"

#define TOUCH_MMI_SHOW(name, fmt) \
static ssize_t name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
{ \
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev); \
	int ret = 0; \
	if (!touch_cdev || !touch_cdev->mdata->get_##name) { \
		dev_err(dev, "get_%s: invalid pointer\n", #name); \
		return (ssize_t)0; \
	} \
	ret = touch_cdev->mdata->get_##name(DEV_TS, &touch_cdev->name); \
	if (ret < 0) { \
		dev_err(dev, "get_%s: return error %d\n", #name, ret); \
		return (ssize_t)0; \
	} \
	return (ssize_t)scnprintf(buf, PAGE_SIZE, fmt, touch_cdev->name); \
}
#define TOUCH_MMI_STORE(name, fmt) \
static ssize_t name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
{ \
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev); \
	unsigned long value = 0; \
	int err = 0; \
	err = kstrtoul(buf, 10, &value); \
	if (err < 0) { \
		dev_err(dev, "%s: Failed to convert value\n", #name); \
		return -EINVAL; \
	} \
	if (!touch_cdev || !touch_cdev->mdata->name) { \
		dev_err(dev, "%s: invalid pointer\n", #name); \
		return -EINVAL; \
	} \
	touch_cdev->mdata->name(DEV_TS, value); \
	return size; \
} \

#define TOUCH_MMI_GET_ATTR_RO(name, fmt) \
TOUCH_MMI_SHOW(name, fmt) \
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL)

#define TOUCH_MMI_GET_ATTR_WO(name) \
TOUCH_MMI_STORE(name, fmt) \
static DEVICE_ATTR(name, (S_IWUSR | S_IWGRP), NULL, name##_store)

#define TOUCH_MMI_GET_ATTR_RW(name, fmt) \
TOUCH_MMI_SHOW(name, fmt) \
TOUCH_MMI_STORE(name, fmt) \
static DEVICE_ATTR(name, (S_IWUSR | S_IWGRP | S_IRUGO), name##_show, name##_store)

static struct class *touchscreens_class;

DECLARE_RWSEM(touchscreens_list_lock);
LIST_HEAD(touchscreens_list);


TOUCH_MMI_GET_ATTR_RO(vendor, FMT_STRING);
TOUCH_MMI_GET_ATTR_RO(productinfo, FMT_STRING);
TOUCH_MMI_GET_ATTR_RO(build_id, FMT_STRING);
TOUCH_MMI_GET_ATTR_RO(config_id, FMT_STRING);
TOUCH_MMI_GET_ATTR_RO(poweron, FMT_INTEGER);
TOUCH_MMI_GET_ATTR_RO(flashprog, FMT_INTEGER);
TOUCH_MMI_GET_ATTR_RO(irq_status, FMT_INTEGER);
TOUCH_MMI_GET_ATTR_RW(drv_irq, FMT_INTEGER);
TOUCH_MMI_GET_ATTR_WO(reset);

static ssize_t path_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	ssize_t blen;
	const char *path;

	if (!touch_cdev) {
		dev_err(dev, "touchscreen device pointer is NULL\n");
		return (ssize_t)0;
	}
	path = kobject_get_path(&DEV_MMI->kobj, GFP_KERNEL);
	blen = scnprintf(buf, PAGE_SIZE, "%s", path ? path : "na");
	kfree(path);
	return blen;
}
static DEVICE_ATTR(path, S_IRUGO, path_show, NULL);


static ssize_t ts_mmi_buildid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);

	build_id_show(dev, attr, buf);
	config_id_show(dev, attr, buf);
	return scnprintf(buf, PAGE_SIZE, "%s-%s\n",
		touch_cdev->build_id,
		touch_cdev->config_id);
}
static DEVICE_ATTR(buildid, S_IRUGO, ts_mmi_buildid_show, NULL);

static ssize_t ts_mmi_ic_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);


	productinfo_show(dev, attr, buf);
	build_id_show(dev, attr, buf);
	config_id_show(dev, attr, buf);
	return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%s\n%s%s\n",
			"Product ID: ", touch_cdev->productinfo,
			"Build ID: ", touch_cdev->build_id,
			"Config ID: ", touch_cdev->config_id);
}
static DEVICE_ATTR(ic_ver, S_IRUGO, ts_mmi_ic_ver_show, NULL);

static ssize_t ts_mmi_hw_irqstat_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);

	irq_status_show(dev, attr, buf);
	switch (touch_cdev->irq_status) {
		case 0:
			return scnprintf(buf, PAGE_SIZE, "Low\n");
		case 1:
			return scnprintf(buf, PAGE_SIZE, "High\n");
		default:
			dev_err(DEV_TS, "%s: Failed to get irq state\n",
					__func__);
			return scnprintf(buf, PAGE_SIZE, "Unknown\n");
	}
}
static DEVICE_ATTR(hw_irqstat, S_IRUGO, ts_mmi_hw_irqstat_show, NULL);

static ssize_t ts_mmi_forcereflash_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	unsigned long value = 0;
	int err = 0;
	err = kstrtoul(buf, 10, &value);
	if (err < 0) {
		dev_err(dev, "forcereflash: Failed to convert value\n");
		return -EINVAL;
	}
	touch_cdev->forcereflash = value;
	return size;
}
static DEVICE_ATTR(forcereflash, (S_IWUSR | S_IWGRP), NULL, ts_mmi_forcereflash_store);


static ssize_t ts_mmi_doreflash_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	char fw_path[TS_MMI_MAX_FW_PATH];
	char template[TS_MMI_MAX_FW_PATH];
	int err;

	if (size > TS_MMI_MAX_FW_PATH) {
		dev_err(dev, "%s: FW filename is too long\n", __func__);
		return -EINVAL;
	}

	if (!touch_cdev->forcereflash) {
		touch_cdev->mdata->get_vendor(DEV_TS, &touch_cdev->vendor);
		if (strncmp(buf, touch_cdev->vendor,
			strnlen(touch_cdev->vendor, TS_MMI_MAX_VENDOR_LEN))) {
			dev_err(dev,
				"%s: FW does not belong to %s\n",
				__func__, touch_cdev->vendor);
			return -EINVAL;
		}

		touch_cdev->mdata->get_productinfo(DEV_TS, &touch_cdev->productinfo);
		snprintf(template, sizeof(template), "-%s-", touch_cdev->productinfo);
		if (!strnstr(buf + strnlen(touch_cdev->vendor, TS_MMI_MAX_VENDOR_LEN),
			template, size)) {
			dev_err(dev, "%s: FW does not belong to %s\n",
				__func__, touch_cdev->productinfo);
			return -EINVAL;
		}
	}

	strlcpy(fw_path, buf, size);
	dev_dbg(dev, "%s: FW filename: %s\n", __func__, fw_path);
	if (!touch_cdev->mdata->firmware_update) {
		dev_err(dev,
			"%s: firmware_update method not exist.\n", __func__);
		return -EINVAL;
	}
	err = touch_cdev->mdata->firmware_update(DEV_TS, fw_path);
	if (err < 0) {
		dev_err(dev, "%s: firmware_update failed %d.\n", __func__, err);
		return -EINVAL;
	}

	dev_info(dev, "%s: update fw from %s, return %d\n",
		__func__, fw_path, err);

	return size;
}
static DEVICE_ATTR(doreflash, (S_IWUSR | S_IWGRP), NULL, ts_mmi_doreflash_store);

static struct attribute *sysfs_class_attrs[] = {
	&dev_attr_path.attr,
	&dev_attr_vendor.attr,
	&dev_attr_productinfo.attr,
	&dev_attr_buildid.attr,
	&dev_attr_build_id.attr,
	&dev_attr_config_id.attr,
	&dev_attr_poweron.attr,
	&dev_attr_flashprog.attr,
	&dev_attr_irq_status.attr,
	&dev_attr_hw_irqstat.attr,
	&dev_attr_ic_ver.attr,
	&dev_attr_drv_irq.attr,
	&dev_attr_reset.attr,
	&dev_attr_forcereflash.attr,
	&dev_attr_doreflash.attr,
	NULL,
};

static const struct attribute_group sysfs_class_group = {
        .attrs = sysfs_class_attrs,
};

static int ts_mmi_get_vendor_info(
	struct ts_mmi_dev *touch_cdev) {
	struct ts_mmi_methods *mdata = touch_cdev->mdata;

	if (mdata->get_productinfo)
		mdata->get_productinfo(DEV_TS, &touch_cdev->productinfo);
	if (mdata->get_vendor)
		mdata->get_vendor(DEV_TS, &touch_cdev->vendor);
	if (mdata->get_bus_type)
		mdata->get_bus_type(DEV_TS, &touch_cdev->bus_type);
	return 0;
}

/**
 * ts_mmi_dev_register - register a new object of ts_mmi_dev class.
 * @parent: The device to register.
 * @touch_cdev: the ts_mmi_dev structure for this device.
 */
int ts_mmi_dev_register(struct device *parent,
	struct ts_mmi_methods *mdata)
{
	struct ts_mmi_dev *touch_cdev;
	int ret = 0;
	const char *class_fname= "primary";

	if (!touchscreens_class || !parent)
		return -ENODEV;

	touch_cdev = devm_kzalloc(parent, sizeof(struct ts_mmi_dev),
				 GFP_KERNEL);
	if (!touch_cdev)
		return -ENOMEM;

	dev_info(parent, "%s: new device\n", __func__);
	DEV_TS = parent;
	touch_cdev->mdata = mdata;

	ret = ts_mmi_parse_dt(touch_cdev, DEV_TS->of_node);
	if (ret < 0) {
		dev_err(DEV_TS, "%s: init panel failed. %d\n", __func__, ret);
		goto PANEL_PARSE_DT_FAILED;
	}

	ts_mmi_get_vendor_info(touch_cdev);

	ret = input_get_new_minor(-1, 1, true);
	if (ret < 0) {
		dev_info(DEV_TS, "%s: get minor number failed. %d\n",
			__func__, ret);
		goto GET_NEW_MINOT_FAILED;
	}
	touch_cdev->class_dev_minor = ret;

	if (touch_cdev->pdata.class_entry_name)
		class_fname = touch_cdev->pdata.class_entry_name;
	else if (touch_cdev->mdata->get_class_entry_name) {
		touch_cdev->mdata->get_class_entry_name(DEV_TS,
			&touch_cdev->class_entry_name);
		class_fname = touch_cdev->class_entry_name;
	} else if (touch_cdev->productinfo[0] != 0)
		class_fname = touch_cdev->productinfo;
	dev_info(DEV_TS, "class entry name %s\n", class_fname);

	DEV_MMI = device_create(touchscreens_class,
		parent, MKDEV(INPUT_MAJOR, touch_cdev->class_dev_minor),
		touch_cdev, "%s", class_fname);
	if (IS_ERR(DEV_MMI)) {
		ret = PTR_ERR(DEV_MMI);
		goto CLASS_DEVICE_CREATE_FAILED;
	}

	down_write(&touchscreens_list_lock);
	list_add_tail(&touch_cdev->node, &touchscreens_list);
	up_write(&touchscreens_list_lock);

	ret = sysfs_create_group(&DEV_MMI->kobj, &sysfs_class_group);
	if (ret)
		goto CLASS_DEVICE_ATTR_CREATE_FAILED;

	ret = ts_mmi_panel_register(touch_cdev);
	if (ret < 0) {
		dev_err(DEV_TS, "%s: Register panel failed. %d\n",
			__func__, ret);
		goto PANEL_INIT_FAILED;
	}

	dev_info(DEV_TS, "Registered touchscreen device: %s.\n", class_fname);

	return 0;

PANEL_INIT_FAILED:
	sysfs_remove_group(&DEV_MMI->kobj, &sysfs_class_group);
CLASS_DEVICE_ATTR_CREATE_FAILED:
	down_write(&touchscreens_list_lock);
	list_del(&touch_cdev->node);
	up_write(&touchscreens_list_lock);
	device_unregister(DEV_MMI);
CLASS_DEVICE_CREATE_FAILED:
	DEV_MMI = NULL;
	input_free_minor(touch_cdev->class_dev_minor);
	touch_cdev->class_dev_minor = 0;
GET_NEW_MINOT_FAILED:
	ts_mmi_panel_unregister(touch_cdev);
PANEL_PARSE_DT_FAILED:
	devm_kfree(parent, touch_cdev);
	return ret;
}
EXPORT_SYMBOL(ts_mmi_dev_register);

/**
 * ts_mmi_dev_unregister - unregister a object of touchscreens class.
 * @touch_cdev: the touchscreen device to unregister
 * Unregister a previously registered via ts_mmi_dev_register object.
 */
void ts_mmi_dev_unregister(struct device *parent)
{
	struct ts_mmi_dev *touch_cdev;

	if (!touchscreens_class)
		return;

	down_write(&touchscreens_list_lock);
	list_for_each_entry(touch_cdev, &touchscreens_list, node) {
		if (DEV_TS == parent) {
			list_del(&touch_cdev->node);
			break;
		}
	}
	up_write(&touchscreens_list_lock);

	if (DEV_TS != parent) {
		dev_err(parent, "%s: device not registered before.\n", __func__);
		return;
	}
	ts_mmi_panel_unregister(touch_cdev);
	dev_info(DEV_TS, "%s: delete device\n", __func__);

	dev_set_drvdata(DEV_TS, NULL);
	sysfs_remove_group(&DEV_MMI->kobj, &sysfs_class_group);
	device_unregister(DEV_MMI);
	DEV_MMI = NULL;
	input_free_minor(touch_cdev->class_dev_minor);
	touch_cdev->class_dev_minor = 0;
	devm_kfree(parent, touch_cdev);
}
EXPORT_SYMBOL(ts_mmi_dev_unregister);

static int __init touchscreens_init(void)
{
	int error = 0;
	pr_info("touchscreen_class: moto touchscreen class init!\n");
	if (touchscreens_class) {
		pr_info("touchscreen_class: moto touchscreen already exist!\n");
		return 0;
	}
	touchscreens_class = class_create(THIS_MODULE, "touchscreen");
	if (IS_ERR(touchscreens_class)) {
		error = PTR_ERR(touchscreens_class);
		touchscreens_class = NULL;
		pr_err("touchscreen_class: touchscreen class init fail!\n");
		return error;
	}
	pr_info("touchscreen_class: moto touchscreen class init success!\n");
	return 0;
}

static void __exit touchscreens_exit(void)
{
	if (touchscreens_class)
		class_destroy(touchscreens_class);
}

subsys_initcall(touchscreens_init);
module_exit(touchscreens_exit);
MODULE_LICENSE("GPL v2");
