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
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>

#define FMT_STRING	"%s"
#define FMT_INTEGER	"%d"
#define FMT_HEX_INTEGER	"0x%02x"

#define TOUCH_MMI_SHOW(name, chk_tp_status, fmt) \
static ssize_t name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
{ \
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev); \
	int ret = 0; \
	if (!touch_cdev) { \
		dev_err(dev, "get_%s: invalid pointer\n", #name); \
		return (ssize_t)0; \
	} \
	mutex_lock(&touch_cdev->extif_mutex); \
	if (!chk_tp_status || is_touch_active) { \
		TRY_TO_GET(name, &touch_cdev->name); \
		if (ret < 0) { \
			dev_err(dev, "get_%s: return error %d\n", #name, ret); \
			ret = 0; \
			goto TOUCH_MMI_SHOW_OUT; \
		} \
	} else \
		dev_dbg(dev, "get_%s: read from cache data.\n", #name); \
	ret = scnprintf(buf, PAGE_SIZE, fmt, touch_cdev->name); \
TOUCH_MMI_SHOW_OUT: \
	mutex_unlock(&touch_cdev->extif_mutex); \
	return (ssize_t)ret; \
}
#define TOUCH_MMI_STORE(name, chk_tp_status, fmt) \
static ssize_t name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
{ \
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev); \
	unsigned long value = 0; \
	int ret = 0; \
	ret = kstrtoul(buf, 0, &value); \
	if (ret < 0) { \
		dev_err(dev, "%s: Failed to convert value\n", #name); \
		return -EINVAL; \
	} \
	if (!touch_cdev) { \
		dev_err(dev, "%s: invalid pointer\n", #name); \
		return -EINVAL; \
	} \
	mutex_lock(&touch_cdev->extif_mutex); \
	mutex_lock(&touch_cdev->method_mutex); \
	touch_cdev->name = value; \
	if (!chk_tp_status || is_touch_active) { \
		_TRY_TO_CALL(name, touch_cdev->name); \
		if (ret < 0) { \
			dev_err(dev, "%s: return error %d\n", #name, ret); \
			goto TOUCH_MMI_STORE_OUT; \
		} \
	}  else \
		dev_dbg(dev, "%s: write to cache data.\n", #name); \
TOUCH_MMI_STORE_OUT: \
	mutex_unlock(&touch_cdev->method_mutex); \
	mutex_unlock(&touch_cdev->extif_mutex); \
	return size; \
} \

#define TOUCH_MMI_GET_ATTR_RO(name, fmt) \
TOUCH_MMI_SHOW(name, false, fmt) \
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL)

#define TOUCH_MMI_GET_ATTR_WO(name) \
TOUCH_MMI_STORE(name, false, fmt) \
static DEVICE_ATTR(name, (S_IWUSR | S_IWGRP), NULL, name##_store)

#define TOUCH_MMI_GET_ATTR_RW(name, fmt) \
TOUCH_MMI_SHOW(name, true, fmt) \
TOUCH_MMI_STORE(name, true, fmt) \
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
TOUCH_MMI_GET_ATTR_RW(suppression, FMT_HEX_INTEGER);
TOUCH_MMI_GET_ATTR_RW(hold_grip, FMT_HEX_INTEGER);
TOUCH_MMI_GET_ATTR_RW(hold_distance, FMT_HEX_INTEGER);
TOUCH_MMI_GET_ATTR_RW(gs_distance, FMT_HEX_INTEGER);
#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
TOUCH_MMI_GET_ATTR_RW(flash_mode, FMT_INTEGER);
#endif
#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
TOUCH_MMI_GET_ATTR_RW(poison_timeout, FMT_HEX_INTEGER);
TOUCH_MMI_GET_ATTR_RW(poison_distance, FMT_HEX_INTEGER);
TOUCH_MMI_GET_ATTR_RW(poison_trigger_distance, FMT_HEX_INTEGER);
#endif
TOUCH_MMI_GET_ATTR_WO(reset);
TOUCH_MMI_GET_ATTR_WO(pinctrl);
TOUCH_MMI_GET_ATTR_WO(refresh_rate);
TOUCH_MMI_GET_ATTR_WO(charger_mode);
TOUCH_MMI_GET_ATTR_WO(update_baseline);

static char *ts_mmi_kobject_get_path(struct kobject *kobj, gfp_t gfp_mask)
{
	char *path;
	int len = 1;
	struct kobject *parent = kobj;

	do {
		if (parent->name == NULL) {
			len = 0;
			break;
		}
		len += strlen(parent->name) + 1;
		parent = parent->parent;
	} while (parent);

	if (len == 0)
		return NULL;

	path = kzalloc(len, gfp_mask);
	if (!path)
		return NULL;

	--len;
	for (parent = kobj; parent; parent = parent->parent) {
		int cur = strlen(parent->name);
		len -= cur;
		memcpy(path + len, parent->name, cur);
		*(path + --len) = '/';
	}
	pr_debug("kobject: '%s' (%p): %s: path = '%s'\n", kobj->name,
		kobj, __func__, path);

	return path;
}

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

	path = ts_mmi_kobject_get_path(&DEV_MMI->kobj, GFP_KERNEL);

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
	int ret = 0;

	if (size > TS_MMI_MAX_FW_PATH) {
		dev_err(dev, "%s: FW filename is too long\n", __func__);
		return -EINVAL;
	}

	if (!touch_cdev->forcereflash) {
		TRY_TO_GET(vendor, &touch_cdev->vendor);
		if (strncmp(buf, touch_cdev->vendor,
			strnlen(touch_cdev->vendor, TS_MMI_MAX_VENDOR_LEN))) {
			dev_err(dev,
				"%s: FW does not belong to %s\n",
				__func__, touch_cdev->vendor);
			return -EINVAL;
		}

		TRY_TO_GET(productinfo, &touch_cdev->productinfo);
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

	TRY_TO_CALL(firmware_update, fw_path);
	if (ret < 0) {
		dev_err(dev, "%s: firmware_update failed %d.\n", __func__, ret);
		return -EINVAL;
	}

	dev_info(dev, "%s: update fw from %s, return %d\n",
		__func__, fw_path, ret);

	return size;
}
static DEVICE_ATTR(doreflash, (S_IWUSR | S_IWGRP), NULL, ts_mmi_doreflash_store);

static ssize_t pwr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret = 0;
	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to convert value\n", __func__);
		return -EINVAL;
	}
	if (!touch_cdev) {
		dev_err(dev, "%s: invalid pointer\n", __func__);
		return -EINVAL;
	}
	TRY_TO_CALL(power, value);
	return size;
}
static DEVICE_ATTR(pwr, (S_IWUSR | S_IWGRP), NULL, pwr_store);

#ifdef CONFIG_BOARD_USES_DOUBLE_TAP_CTRL
/*
 * gesture value used to indicate which gesture mode type is enabled
 */
static ssize_t gesture_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%02x\n", touch_cdev->pdata.supported_gesture_type);
}
static ssize_t gesture_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	unsigned int value = 0;
	int err = 0;

	mutex_lock(&touch_cdev->extif_mutex);
	err = sscanf(buf, "%d", &value);
	if (err < 0) {
		dev_err(dev, "forcereflash: Failed to convert value\n");
		mutex_unlock(&touch_cdev->extif_mutex);
		return -EINVAL;
	}
	switch (value) {
		case 0x10:
			dev_info(dev, "%s: zero tap disable\n", __func__);
			touch_cdev->gesture_mode_type &= 0xFE;
			break;
		case 0x11:
			dev_info(dev, "%s: zero tap enable\n", __func__);
			touch_cdev->gesture_mode_type |= 0x01;
			break;
		case 0x20:
			dev_info(dev, "%s: single tap disable\n", __func__);
			touch_cdev->gesture_mode_type &= 0xFD;
			break;
		case 0x21:
			dev_info(dev, "%s: single tap enable\n", __func__);
			touch_cdev->gesture_mode_type |= 0x02;
			break;
		case 0x30:
			dev_info(dev, "%s: double tap disable\n", __func__);
			touch_cdev->gesture_mode_type &= 0xFB;
			break;
		case 0x31:
			dev_info(dev, "%s: double tap enable\n", __func__);
			touch_cdev->gesture_mode_type |= 0x04;
			break;
		default:
			dev_info(dev, "%s: unsupport gesture mode type\n", __func__);
			;
	}
	mutex_unlock(&touch_cdev->extif_mutex);
	dev_info(dev, "%s: gesture_mode_type = 0x%02x \n", __func__, touch_cdev->gesture_mode_type);

	return size;
}
static DEVICE_ATTR(gesture, (S_IWUSR | S_IWGRP | S_IRUGO), gesture_show, gesture_store);
#endif

/*
 * Show liquid detection function status and detection result
 */
static ssize_t liquid_detection_ctl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "function status: %02x, detection result: %02x\n",
		touch_cdev->lpd_state, touch_cdev->liquid_status);
}

/*
 * Enable/disable liquid detection function for debug
 */
static ssize_t liquid_detection_ctl_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	unsigned int value = 0;
	int err = 0;

	err = sscanf(buf, "%d", &value);
	if (err < 0) {
		dev_err(dev, "liquid_detection_ctl: Failed to convert value\n");
		return -EINVAL;
	}

	if (value == 2) {
		dev_info(dev, "liquid_detection_ctl: Not support liquid detecion\n");
		touch_cdev->pdata.support_liquid_detection = 0;
		return size;
	} else if (value == 3) {
		dev_info(dev, "liquid_detection_ctl: Support liquid detecion\n");
		touch_cdev->pdata.support_liquid_detection = 1;
		return size;
	}

	mutex_lock(&touch_cdev->extif_mutex);
	if (value != touch_cdev->lpd_state) {
		touch_cdev->lpd_state = !!value;
		kfifo_put(&touch_cdev->cmd_pipe, TS_MMI_DO_LIQUID_DETECTION);
		schedule_delayed_work(&touch_cdev->work, 0);
		dev_info(DEV_MMI, "%s: LPD state is %d\n", __func__, touch_cdev->lpd_state);
	}
	mutex_unlock(&touch_cdev->extif_mutex);

	return size;
}
static DEVICE_ATTR(liquid_detection_ctl, (S_IWUSR | S_IWGRP | S_IRUGO),
	liquid_detection_ctl_show, liquid_detection_ctl_store);

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
#ifdef TS_MMI_TOUCH_MULTIWAY_UPDATE_FW
	&dev_attr_flash_mode.attr,
#endif
	&dev_attr_pwr.attr,
	&dev_attr_pinctrl.attr,
	&dev_attr_refresh_rate.attr,
	&dev_attr_charger_mode.attr,
	&dev_attr_update_baseline.attr,
#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
	&dev_attr_poison_timeout.attr,
	&dev_attr_poison_distance.attr,
	&dev_attr_poison_trigger_distance.attr,
#endif
#ifdef CONFIG_BOARD_USES_DOUBLE_TAP_CTRL
	&dev_attr_gesture.attr,
#endif
	&dev_attr_liquid_detection_ctl.attr,
	NULL,
};

static const struct attribute_group sysfs_class_group = {
        .attrs = sysfs_class_attrs,
};

/*
 * pill_region input value is much different between others sys entry
 * override default functions
 */
static ssize_t pill_region_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	int ret = 0;

	if (!touch_cdev) {
		dev_err(dev, "get_pill_region: invalid pointer\n");
		return (ssize_t)0;
	}
	mutex_lock(&touch_cdev->extif_mutex);
	if (is_touch_active) {
		TRY_TO_GET(pill_region, &touch_cdev->pill_region);
		if (ret < 0) {
			dev_err(dev, "get_pill_region: return error %d\n", ret);
			ret = 0;
			goto PILL_REGION_SHOW_OUT;
		}
	} else
		dev_dbg(dev, "get_pill_region: read from cache data.\n");

	ret = scnprintf(buf, PAGE_SIZE, "0x%02x 0x%x 0x%x",
		touch_cdev->pill_region[0], touch_cdev->pill_region[1], touch_cdev->pill_region[2]);
PILL_REGION_SHOW_OUT:
	mutex_unlock(&touch_cdev->extif_mutex);
	return (ssize_t)ret;

}
static ssize_t pill_region_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	unsigned int args[TS_MMI_PILL_REGION_REQ_ARGS_NUM] = {0};
	int ret = 0;
	int i = TS_MMI_PILL_REGION_REQ_ARGS_NUM;

	ret = sscanf(buf, "0x%x 0x%x 0x%x", &args[0], &args[1], &args[2]);
	if (ret < TS_MMI_PILL_REGION_REQ_ARGS_NUM) {
		dev_err(dev, "pill_region: Failed to convert value\n");
		return -EINVAL;
	}

	if (!touch_cdev) {
		dev_err(dev, "pill_region: invalid pointer\n");
		return -EINVAL;
	}
	mutex_lock(&touch_cdev->extif_mutex);
	mutex_lock(&touch_cdev->method_mutex);
	while (i--)
		touch_cdev->pill_region[i] = args[i];
	if (is_touch_active)
		_TRY_TO_CALL(pill_region, touch_cdev->pill_region);
	else
		dev_dbg(dev, "pill_region: write to cache data.\n");
	mutex_unlock(&touch_cdev->method_mutex);
	mutex_unlock(&touch_cdev->extif_mutex);
	return size;
}
static DEVICE_ATTR(pill_region, (S_IWUSR | S_IWGRP | S_IRUGO), pill_region_show, pill_region_store);

/*
 * active_region input requires 4 parameters
 */
static ssize_t active_region_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	int ret = 0;

	if (!touch_cdev) {
		dev_err(dev, "get_active_region: invalid pointer\n");
		return (ssize_t)0;
	}
	mutex_lock(&touch_cdev->extif_mutex);
	if (is_touch_active) {
		TRY_TO_GET(active_region, &touch_cdev->active_region);
		if (ret < 0) {
			dev_err(dev, "get_active_region: return error %d\n", ret);
			ret = 0;
			goto ACTIVE_REGION_SHOW_OUT;
		}
	} else
		dev_dbg(dev, "get_active_region: read from cache data.\n");

	ret = scnprintf(buf, PAGE_SIZE, "%u %u %u %u",
		touch_cdev->active_region[0], touch_cdev->active_region[1],
		touch_cdev->active_region[2], touch_cdev->active_region[3]);
ACTIVE_REGION_SHOW_OUT:
	mutex_unlock(&touch_cdev->extif_mutex);

	return (ssize_t)ret;

}
static ssize_t active_region_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct ts_mmi_dev *touch_cdev = dev_get_drvdata(dev);
	unsigned int args[TS_MMI_ACTIVE_REGION_REQ_ARGS_NUM] = {0};
	int ret = 0;
	int i = TS_MMI_ACTIVE_REGION_REQ_ARGS_NUM;

	ret = sscanf(buf, "%u %u %u %u", &args[0], &args[1], &args[2], &args[3]);
	if (ret < TS_MMI_ACTIVE_REGION_REQ_ARGS_NUM) {
		dev_err(dev, "active_region: Failed to convert value\n");
		return -EINVAL;
	}

	if (!touch_cdev) {
		dev_err(dev, "active_region: invalid pointer\n");
		return -EINVAL;
	}
	mutex_lock(&touch_cdev->extif_mutex);
	mutex_lock(&touch_cdev->method_mutex);
	while (i--)
		touch_cdev->active_region[i] = args[i];
	if (is_touch_active)
		_TRY_TO_CALL(active_region, touch_cdev->active_region);
	else
		dev_dbg(dev, "active_region: write to cache data.\n");
	mutex_unlock(&touch_cdev->method_mutex);
	mutex_unlock(&touch_cdev->extif_mutex);

	return size;
}
static DEVICE_ATTR(active_region, (S_IWUSR | S_IWGRP | S_IRUGO), active_region_show, active_region_store);

static int ts_mmi_sysfs_create_edge_entries(struct ts_mmi_dev *touch_cdev, bool create) {
	int ret = 0;

	if (create) {
		/*initialize value*/
		if (touch_cdev->pdata.active_region_ctrl) {
			TRY_TO_GET(active_region, &touch_cdev->active_region);
			if (ret < 0)
				dev_err(DEV_TS, "%s: failed to read active_region info (%d)\n",
						__func__, ret);
			ret = sysfs_create_file(&DEV_MMI->kobj, &dev_attr_active_region.attr);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to create active_region entry (%d)\n",
						__func__, ret);
				goto CREATE_SUPPRESSION_FAILED;
			}
		}
		if (touch_cdev->pdata.suppression_ctrl) {
			TRY_TO_GET(suppression, &touch_cdev->suppression);
			if (ret < 0)
				dev_err(DEV_TS, "%s: failed to read suppression info (%d)\n",
						__func__, ret);
			ret = sysfs_create_file(&DEV_MMI->kobj, &dev_attr_suppression.attr);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to create suppression entry (%d)\n",
						__func__, ret);
				goto CREATE_SUPPRESSION_FAILED;
			}
		}
		if (touch_cdev->pdata.pill_region_ctrl) {
			TRY_TO_GET(pill_region, &touch_cdev->pill_region);
			if (ret < 0)
				dev_err(DEV_TS, "%s: failed to read pill_region info (%d)\n",
						__func__, ret);
			ret = sysfs_create_file(&DEV_MMI->kobj, &dev_attr_pill_region.attr);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to create pill_region entry (%d)\n",
						__func__, ret);
				goto CREATE_PILL_REGION_FAILED;
			}
		}
		if (touch_cdev->pdata.hold_distance_ctrl) {
			TRY_TO_GET(hold_distance, &touch_cdev->hold_distance);
			if (ret < 0)
				dev_err(DEV_TS, "%s: failed to read hold_distance info (%d)\n",
						__func__, ret);
			ret = sysfs_create_file(&DEV_MMI->kobj, &dev_attr_hold_distance.attr);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to create hold_distance entry (%d)\n",
						__func__, ret);
				goto CREATE_HOLD_DISTANCE_FAILED;
			}
		}
		if (touch_cdev->pdata.gs_distance_ctrl) {
			TRY_TO_GET(gs_distance, &touch_cdev->gs_distance);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to read gs_distance info (%d)\n",
						__func__, ret);
				/* Set default gs distance value */
				touch_cdev->gs_distance = 0x1E;
			}

			ret = sysfs_create_file(&DEV_MMI->kobj, &dev_attr_gs_distance.attr);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to create gs_distance entry (%d)\n",
						__func__, ret);
				goto CREATE_GS_DISTANCE_FAILED;
			}
		}
		if (touch_cdev->pdata.hold_grip_ctrl) {
			TRY_TO_GET(hold_grip, &touch_cdev->hold_grip);
			if (ret < 0)
				dev_err(DEV_TS, "%s: failed to read hold_grip info (%d)\n",
						__func__, ret);
			ret = sysfs_create_file(&DEV_MMI->kobj, &dev_attr_hold_grip.attr);
			if (ret < 0) {
				dev_err(DEV_TS, "%s: failed to create hold_grip entry (%d)\n",
						__func__, ret);
				goto CREATE_HOLD_GRIP_FAILED;
			}
		}
#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
		if (touch_cdev->pdata.poison_slot_ctrl) {
			touch_cdev->poison_distance = TOUCHSCREEN_MMI_DEFAULT_POISON_DISTANCE;
			touch_cdev->poison_trigger_distance = TOUCHSCREEN_MMI_DEFAULT_POISON_TRIGGER_DISTANCE;
			touch_cdev->poison_timeout = TOUCHSCREEN_MMI_DEFAULT_POISON_TIMEOUT_MS;
		}
#endif
		return 0;
	}

	if (touch_cdev->pdata.hold_grip_ctrl)
		sysfs_remove_file(&DEV_MMI->kobj, &dev_attr_hold_grip.attr);
CREATE_HOLD_GRIP_FAILED:
	if (touch_cdev->pdata.gs_distance_ctrl)
		sysfs_remove_file(&DEV_MMI->kobj, &dev_attr_gs_distance.attr);
CREATE_GS_DISTANCE_FAILED:
	if (touch_cdev->pdata.hold_distance_ctrl)
		sysfs_remove_file(&DEV_MMI->kobj, &dev_attr_hold_distance.attr);
CREATE_HOLD_DISTANCE_FAILED:
	if (touch_cdev->pdata.pill_region_ctrl)
		sysfs_remove_file(&DEV_MMI->kobj, &dev_attr_pill_region.attr);
CREATE_PILL_REGION_FAILED:
	if (touch_cdev->pdata.suppression_ctrl)
		sysfs_remove_file(&DEV_MMI->kobj, &dev_attr_suppression.attr);
CREATE_SUPPRESSION_FAILED:
	return ret;
}

static int ts_mmi_get_vendor_info(
	struct ts_mmi_dev *touch_cdev) {
	int ret = 0;

	TRY_TO_GET(productinfo, &touch_cdev->productinfo);
	TRY_TO_GET(vendor, &touch_cdev->vendor);
	TRY_TO_GET(bus_type, &touch_cdev->bus_type);

	return 0;
}

static struct ts_mmi_dev *ts_mmi_dev_to_cdev(struct device *parent)
{
	struct ts_mmi_dev *touch_cdev = NULL;

	down_write(&touchscreens_list_lock);
	list_for_each_entry(touch_cdev, &touchscreens_list, node) {
		if(DEV_TS == parent)
			break;
	}
	up_write(&touchscreens_list_lock);
	return touch_cdev;
}

static int get_class_fname_handler(struct device *parent, const char **pfname)
{
	struct ts_mmi_dev *touch_cdev = ts_mmi_dev_to_cdev(parent);
	if (!touch_cdev)
		return -ENODEV;
	*pfname = dev_name(touch_cdev->class_dev);
	return 0;
}

static int get_supplier_handler(struct device *parent, const char **psname)
{
	struct ts_mmi_dev *touch_cdev = ts_mmi_dev_to_cdev(parent);
	if (!touch_cdev)
		return -ENODEV;
	*psname = touch_cdev->panel_supplier;
	return 0;
}

static int get_gesture_type_handler(struct device *parent, unsigned char *gesture_type)
{
	struct ts_mmi_dev *touch_cdev = ts_mmi_dev_to_cdev(parent);
	if (!touch_cdev)
		return -ENODEV;
	*gesture_type = touch_cdev->gesture_mode_type;
	return 0;
}

static int report_liquid_detection_status_handler(struct device *parent, int status)
{
	struct ts_mmi_dev *touch_cdev = ts_mmi_dev_to_cdev(parent);
	if (!touch_cdev)
		return -ENODEV;

	dev_info(DEV_TS, "%s: notify liquid detection status: %d\n", __func__, status);
	touch_cdev->liquid_status = status;
	if (touch_cdev->is_lpd_registered)
		relay_notifier_fire(BLOCKING, LPD, NOTIFY_EVENT_TUD_STATUS,
			(void *)&status);
	else
		dev_err(DEV_TS, "%s: lpd_notifier does not register\n", __func__);
	return 0;
}

static int ts_mmi_default_pinctrl(struct device *parent, int on)
{
	struct ts_mmi_dev *touch_cdev = ts_mmi_dev_to_cdev(parent);

	if (!touch_cdev)
		return -ENODEV;

	dev_info(DEV_TS, "%s: %s\n", __func__, on ? "ACTIVE" : "SUSPEND");

	if (on) {
		if (!IS_ERR_OR_NULL(touch_cdev->pinctrl_on_state))
			pinctrl_select_state(touch_cdev->pinctrl_node,
				touch_cdev->pinctrl_on_state);
	} else {
		if (!IS_ERR_OR_NULL(touch_cdev->pinctrl_off_state))
			pinctrl_select_state(touch_cdev->pinctrl_node,
				touch_cdev->pinctrl_off_state);
	}

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
	mutex_init(&touch_cdev->extif_mutex);
	mutex_init(&touch_cdev->method_mutex);

	ret = ts_mmi_parse_dt(touch_cdev, DEV_TS->of_node);
	if (ret < 0) {
		dev_err(DEV_TS, "%s: init panel failed. %d\n", __func__, ret);
		goto PANEL_PARSE_DT_FAILED;
	}

	touch_cdev->pinctrl_node = devm_pinctrl_get(DEV_TS);
	if (IS_ERR_OR_NULL(touch_cdev->pinctrl_node)) {
		dev_info(DEV_TS, "%s: No pinctrl defined\n", __func__);
	} else if (!touch_cdev->mdata->pinctrl) {
		touch_cdev->pinctrl_on_state = pinctrl_lookup_state(touch_cdev->pinctrl_node, "ts_mmi_on_state");
		touch_cdev->pinctrl_off_state = pinctrl_lookup_state(touch_cdev->pinctrl_node, "ts_mmi_off_state");
		if (!IS_ERR_OR_NULL(touch_cdev->pinctrl_on_state) ||
			!IS_ERR_OR_NULL(touch_cdev->pinctrl_off_state)) {
			dev_info(DEV_TS, "%s: No pinctrl method add default\n", __func__);
			touch_cdev->mdata->pinctrl = ts_mmi_default_pinctrl;
			if (!IS_ERR_OR_NULL(touch_cdev->pinctrl_on_state))
				pinctrl_select_state(touch_cdev->pinctrl_node,
					touch_cdev->pinctrl_on_state);
		}
	}

	ts_mmi_get_vendor_info(touch_cdev);

	ret = alloc_chrdev_region(&touch_cdev->class_dev_no, 0, 1, class_fname);
	if (ret < 0) {
		dev_info(DEV_TS, "%s: get device number failed. %d\n",
			__func__, ret);
		goto GET_NEW_MINOT_FAILED;
	}

	if (touch_cdev->pdata.class_entry_name)
		class_fname = touch_cdev->pdata.class_entry_name;
	else if (touch_cdev->mdata->get_class_entry_name) {
		touch_cdev->mdata->get_class_entry_name(DEV_TS,
			&touch_cdev->class_entry_name);
		class_fname = touch_cdev->class_entry_name;
	}
	dev_info(DEV_TS, "class entry name %s\n", class_fname);

	DEV_MMI = device_create(touchscreens_class,
		parent, touch_cdev->class_dev_no,
		touch_cdev, "%s", class_fname);
	if (IS_ERR(DEV_MMI)) {
		ret = PTR_ERR(DEV_MMI);
		goto CLASS_DEVICE_CREATE_FAILED;
	}
	touch_cdev->mdata->exports.get_class_fname = get_class_fname_handler;
	touch_cdev->mdata->exports.get_supplier = get_supplier_handler;
	touch_cdev->mdata->exports.get_gesture_type = get_gesture_type_handler;
	touch_cdev->mdata->exports.report_liquid_detection_status = report_liquid_detection_status_handler;
	touch_cdev->mdata->exports.kobj_notify = &DEV_MMI->kobj;

	down_write(&touchscreens_list_lock);
	list_add_tail(&touch_cdev->node, &touchscreens_list);
	up_write(&touchscreens_list_lock);

	ret = sysfs_create_group(&DEV_MMI->kobj, &sysfs_class_group);
	if (ret)
		goto CLASS_DEVICE_ATTR_CREATE_FAILED;

	if (touch_cdev->mdata->extend_attribute_group) {
		touch_cdev->mdata->extend_attribute_group(DEV_TS,
			&touch_cdev->extern_group);
		if (touch_cdev->extern_group) {
			ret = sysfs_create_group(&DEV_MMI->kobj,
				touch_cdev->extern_group);
			if (ret)
				goto CLASS_DEVICE_EXT_ATTR_CREATE_FAILED;
		}
	}

	ret = ts_mmi_sysfs_create_edge_entries(touch_cdev, true);
	if (ret < 0) {
		dev_err(DEV_TS, "%s: Create edge sys entries failed. %d\n",
			__func__, ret);
		goto CLASS_DEVICE_EDGE_ATTR_CREATE_FAILED;
	}

	ret = ts_mmi_panel_register(touch_cdev);
	if (ret < 0) {
		dev_err(DEV_TS, "%s: Register panel failed. %d\n",
			__func__, ret);
		goto PANEL_INIT_FAILED;
	}

	ret = ts_mmi_notifiers_register(touch_cdev);
	if (ret < 0) {
		dev_err(DEV_TS, "%s: Register notifiers failed. %d\n",
			__func__, ret);
		goto NOTIFIER_INIT_FAILED;
	}

	if (touch_cdev->pdata.gestures_enabled) {
		ret = ts_mmi_gesture_init(touch_cdev);
		if (ret < 0) {
			dev_err(DEV_TS, "%s: Register gesture failed. %d\n",
				__func__, ret);
			goto GESTURE_INIT_FAILED;
		}
	}

	if (touch_cdev->pdata.cli_gestures_enabled) {
		ret = ts_mmi_cli_gesture_init(touch_cdev);
		if (ret < 0) {
			dev_err(DEV_TS, "%s: Register CLI gesture failed. %d\n",
				__func__, ret);
			goto GESTURE_INIT_FAILED;
		}
	}

	if (touch_cdev->pdata.palm_enabled) {
		ret = ts_mmi_palm_init(touch_cdev);
		if (ret < 0) {
			dev_err(DEV_TS, "%s: Register plam failed. %d\n",
				__func__, ret);
			goto GESTURE_INIT_FAILED;
		}
	}

	dev_info(DEV_TS, "Registered touchscreen device: %s.\n", class_fname);

	return 0;

GESTURE_INIT_FAILED:
	ts_mmi_notifiers_unregister(touch_cdev);
NOTIFIER_INIT_FAILED:
	ts_mmi_panel_unregister(touch_cdev);
PANEL_INIT_FAILED:
	ts_mmi_sysfs_create_edge_entries(touch_cdev, false);
CLASS_DEVICE_EDGE_ATTR_CREATE_FAILED:
	if (touch_cdev->extern_group)
		sysfs_remove_group(&DEV_MMI->kobj, touch_cdev->extern_group);
CLASS_DEVICE_EXT_ATTR_CREATE_FAILED:
	sysfs_remove_group(&DEV_MMI->kobj, &sysfs_class_group);
CLASS_DEVICE_ATTR_CREATE_FAILED:
	down_write(&touchscreens_list_lock);
	list_del(&touch_cdev->node);
	up_write(&touchscreens_list_lock);
	device_unregister(DEV_MMI);
CLASS_DEVICE_CREATE_FAILED:
	DEV_MMI = NULL;
	unregister_chrdev_region(touch_cdev->class_dev_no, 1);
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
	if (touch_cdev->pdata.gestures_enabled)
		ts_mmi_gesture_remove(touch_cdev);
	if (touch_cdev->pdata.cli_gestures_enabled)
		ts_mmi_cli_gesture_remove(touch_cdev);
	if (touch_cdev->pdata.palm_enabled)
		ts_mmi_palm_remove(touch_cdev);
	ts_mmi_notifiers_unregister(touch_cdev);
	ts_mmi_panel_unregister(touch_cdev);
	dev_info(DEV_TS, "%s: delete device\n", __func__);

	dev_set_drvdata(DEV_TS, NULL);
	ts_mmi_sysfs_create_edge_entries(touch_cdev, false);
	if (touch_cdev->extern_group)
		sysfs_remove_group(&DEV_MMI->kobj, touch_cdev->extern_group);
	sysfs_remove_group(&DEV_MMI->kobj, &sysfs_class_group);
	device_unregister(DEV_MMI);
	DEV_MMI = NULL;
	unregister_chrdev_region(touch_cdev->class_dev_no, 1);
	devm_kfree(parent, touch_cdev);
}
EXPORT_SYMBOL(ts_mmi_dev_unregister);

bool ts_mmi_is_panel_match(const char *panel_node, char *touch_ic_name)
{
	struct device_node *of_chosen;
	const char *panel_name;

	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen == NULL)
		of_chosen = of_find_node_by_path("/chosen@0");

	if (of_chosen == NULL)
		return false;

	if (of_property_read_string(of_chosen, panel_node, &panel_name)) {
		pr_err("%s: Failed to get %s", __func__, panel_node);
		return false;
	}

	if (strstr(panel_name, touch_ic_name))
			return true;

	return false;
}
EXPORT_SYMBOL(ts_mmi_is_panel_match);

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
