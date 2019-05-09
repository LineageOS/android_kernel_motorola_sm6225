/*
 * Copyright (C) 2018 Motorola Mobility LLC
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>

#include "sec_mmi.h"

/* MMI specific sysfs entries and API */
#define DEV_MMI (&data->i2c_client->dev)
#define DEV_TS  (&ts->client->dev)

static inline ssize_t sec_mmi_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sec_mmi_forcereflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_doreflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static inline ssize_t sec_mmi_show_error(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(forcereflash, (S_IWUSR | S_IWGRP), NULL, sec_mmi_forcereflash_store);
static DEVICE_ATTR(doreflash, (S_IWUSR | S_IWGRP), NULL, sec_mmi_doreflash_store);
static DEVICE_ATTR(drv_irq, (S_IWUSR | S_IWGRP | S_IRUGO), sec_mmi_drv_irq_show, sec_mmi_drv_irq_store);
static DEVICE_ATTR(hw_irqstat, S_IRUGO, sec_mmi_hw_irqstat_show, NULL);
static DEVICE_ATTR(ic_ver, S_IRUGO, sec_mmi_ic_ver_show, NULL);
static DEVICE_ATTR(poweron, S_IRUGO, sec_mmi_poweron_show, NULL);

static struct attribute *mmi_attributes[] = {
	&dev_attr_sec_mmi_forcereflash.attr,
	&dev_attr_sec_mmi_fdoreflash.attr,
	&dev_attr_sec_mmi_drv_irq.attr,
	&dev_attr_sec_mmi_hw_irqstat.attr,
	&dev_attr_sec_mmi_ic_ver.attr,
	&dev_attr_sec_mmi_poweron.attr,
	NULL,
};

static struct attribute_group mmi_attr_group = {
	.attrs = mmi_attributes,
};

/* Attribute: path (RO) */
static ssize_t path_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sec_mmi_data *data = dev_get_drvdata(dev);
	ssize_t blen;
	const char *path;

	if (!data) {
		dev_err(DEV_MMI "data pointer is NULL\n");
		return (ssize_t)0;
	}
	path = kobject_get_path(&data->i2c_client->dev.kobj, GFP_KERNEL);
	blen = scnprintf(buf, PAGE_SIZE, "%s", path ? path : "na");
	kfree(path);
	return blen;
}

/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "samsung");
}

static struct device_attribute class_attributes[] = {
	__ATTR_RO(path),
	__ATTR_RO(vendor),
	__ATTR_NULL
};

static int sec_mmi_touchscreen_class(
	struct sec_mmi_data *data, bool create)
{
	struct device_attribute *attrs = touchscreen_attributes;
	int i, error = 0;
	const char *class_fname = "primary";
	static struct class *touchscreen_class;
	static int minor;

	if (create) {
		minor = input_get_new_minor(data->i2c_client->addr,
						1, false);
		if (minor < 0)
			minor = input_get_new_minor(TSDEV_MINOR_BASE,
					TSDEV_MINOR_MAX, true);
		dev_info(DEV_MMI, "assigned minor %d\n", minor);

		if (!touchscreen_class) {
			touchscreen_class = class_create(THIS_MODULE, "touchscreen");
			if (IS_ERR(touchscreen_class)) {
				error = PTR_ERR(touchscreen_class);
				touchscreen_class = NULL;
				return error;
			}
		}

		if (data->class_entry_name)
			class_fname = data->class_entry_name;

		data->ts_class_dev = device_create(touchscreen_class,
				NULL, MKDEV(INPUT_MAJOR, minor), data, class_fname);
		if (IS_ERR(data->ts_class_dev)) {
			error = PTR_ERR(data->ts_class_dev);
			data->ts_class_dev = NULL;
			return error;
		}

		dev_set_drvdata(data->ts_class_dev, data);

		for (i = 0; class_attrs[i].attr.name != NULL; ++i) {
			error = device_create_file(data->ts_class_dev, &class_attrs[i]);
			if (error)
				break;
		}

		if (error)
			goto device_destroy;
	} else {
		if (!touchscreen_class || !data->ts_class_dev)
			return -ENODEV;

		dev_set_drvdata(rmi4_data->ts_class_dev, NULL);

		for (i = 0; class_attrs[i].attr.name != NULL; ++i)
			device_remove_file(data->ts_class_dev, &class_attrs[i]);

		device_unregister(data->ts_class_dev);
		data->ts_class_dev = NULL;
	}

	return 0;

device_destroy:
	for (--i; i >= 0; --i)
		device_remove_file(data->ts_class_dev, &class_attrs[i]);
	device_destroy(touchscreen_class, MKDEV(INPUT_MAJOR, minor));
	data->ts_class_dev = NULL;
	class_unregister(touchscreen_class);
	dev_err(DEV_MMI, "%s: error creating touchscreen class attrs\n", __func__);

	return -ENODEV;
}

static int sec_mmi_panel_cb(struct notifier_block *nb,
		unsigned long event, void *evd)
{
	struct msm_drm_notifier *evdata = evd;
	struct sec_mmi_data *data =
		container_of(nb, struct sec_mmi_data, panel_nb);

	if (!evdata || (evdata->id != data->ctrl_dsi)) {
		dev_dbg(DEV_MMI, "%s: drm notification: id(%d) != ctrl_dsi(%d)\n",
				__func__, evdata->id, data->ctrl_dsi);
		return 0;
	}

	if ((event == MSM_DRM_EARLY_EVENT_BLANK || event == MSM_DRM_EVENT_BLANK) &&
		 evdata && evdata->data && data) {
		int *blank = evdata->data;

		dev_dbg(DEV_MMI, "%s: drm notification: event = %lu blank = %d\n",
				__func__, event, *blank);
		/* entering suspend upon early blank event */
		/* to ensure shared power supply is still on */
		/* for in-cell design touch solutions */
		if (event == MSM_DRM_EARLY_EVENT_BLANK) {
			if (*blank != MSM_DRM_BLANK_POWERDOWN)
				return 0;
			synaptics_dsx_display_off(&data->i2c_client->dev);
		} else if (*blank == MSM_DRM_BLANK_UNBLANK) {
			synaptics_dsx_display_on(&data->i2c_client->dev);
		}
	}

	return 0;
}

static int sec_mmi_register_notifier(
	struct sec_mmi_data *data, bool enable)
{
	int rc;

	if (enable) {
		data->panel_nb.notifier_call = sec_mmi_panel_cb;
		rc = msm_drm_register_client(&data->panel_nb);
	} else
		rc = msm_drm_unregister_client(&data->panel_nb);

	if (!rc)
		dev_dbg(DEV_MMI, "%s: %sregistered drm notifier\n",
				__func__, enable ? "" : "un");
	else
		dev_err(DEV_MMI, "%s: unable to %sregister drm notifier\n",
				__func__, enable ? "" : "un");

	return rc;
}

static ssize_t sec_mmi_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			ts->power_status != SEC_TS_STATE_POWER_OFF);
}

static ssize_t sec_mmi_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%x\n%s%x\n",
			"Product ID: ", rmi->product_id_string,
			"Build ID: ", build_id,
			"Config ID: ", config_id);
}

static ssize_t sec_mmi_doreflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int result = -1;

	if (size > SEC_TS_MAX_FW_PATH) {
		dev_err(&rmi4_data->i2c_client->dev,
			"%s: FW filename is too long\n",
			__func__);
		return -EINVAL;
	}

	disable_irq(ts->client->irq);
	scnprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", buf);
	input_info(true, DEV_TS, "%s: initial bl update %s\n", __func__, fw_path);

	/* Loading Firmware------------------------------------------ */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		input_err(true, DEV_TS, "%s: bt is not available\n", __func__);
		goto err_request_fw;
	}
	input_info(true, DEV_TS, "%s: request bt done! size = %d\n", __func__, (int)fw_entry->size);

	result = sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 1, false, 0);

err_request_fw:
	release_firmware(fw_entry);
	enable_irq(ts->client->irq);

	return result;
}

static ssize_t sec_ts_regreadsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->device_mutex);

	lv1cmd = buf[0];
	lv1_readsize = ((unsigned int)buf[4] << 24) |
			((unsigned int)buf[3] << 16) | ((unsigned int) buf[2] << 8) | ((unsigned int)buf[1] << 0);
	lv1_readoffset = 0;
	lv1_readremain = 0;

	mutex_unlock(&ts->device_mutex);

	return size;
}

static ssize_t sec_ts_enter_recovery_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int ret;
	unsigned long on;

	ret = kstrtoul(buf, 10, &on);
	if (ret != 0) {
		input_err(true, &ts->client->dev, "%s: failed to read:%d\n",
					__func__, ret);
		return -EINVAL;
	}

	if (on == 1) {
		disable_irq(ts->client->irq);
		gpio_free(pdata->irq_gpio);

		input_info(true, &ts->client->dev, "%s: gpio free\n", __func__);
		if (gpio_is_valid(pdata->irq_gpio)) {
			ret = gpio_request_one(pdata->irq_gpio, GPIOF_OUT_INIT_LOW, "sec,tsp_int");
			input_info(true, &ts->client->dev, "%s: gpio request one\n", __func__);
			if (ret < 0)
				input_err(true, &ts->client->dev, "%s: Unable to request tsp_int [%d]: %d\n", __func__, pdata->irq_gpio, ret);
		} else {
			input_err(true, &ts->client->dev, "%s: Failed to get irq gpio\n", __func__);
			return -EINVAL;
		}

		pdata->power(ts, false);
		sec_ts_delay(100);
		pdata->power(ts, true);
	} else {
		gpio_free(pdata->irq_gpio);

		if (gpio_is_valid(pdata->irq_gpio)) {
			ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN, "sec,tsp_int");
			if (ret) {
				input_err(true, &ts->client->dev, "%s: Unable to request tsp_int [%d]\n", __func__, pdata->irq_gpio);
				return -EINVAL;
			}
		} else {
			input_err(true, &ts->client->dev, "%s: Failed to get irq gpio\n", __func__);
			return -EINVAL;
		}

		pdata->power(ts, false);
		sec_ts_delay(500);
		pdata->power(ts, true);
		sec_ts_delay(500);

		/* AFE Calibration */
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CALIBRATION_AMBIENT, NULL, 0);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: fail to write AFE_CAL\n", __func__);

		sec_ts_delay(1000);
		enable_irq(ts->client->irq);
	}

	sec_ts_read_information(ts);

	return size;
}

static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	input_err(true, &ts->client->dev, "%s: read only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	input_err(true, &ts->client->dev, "%s: write only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

int sec_ts_raw_device_init(struct sec_ts_data *ts)
{
	int ret;

	struct class *sec_class = NULL;

	sec_class = class_create(THIS_MODULE, "sec");
	ret = IS_ERR_OR_NULL(sec_class);
	if (ret) {
		input_err(true, &ts->client->dev, "%s: fail - class_create\n", __func__);
		return;
	}

	ts->dev = device_create(sec_class, NULL, 0, ts, "sec_ts");

	ret = IS_ERR(ts->dev);
	if (ret) {
		input_err(true, &ts->client->dev, "%s: fail - device_create\n", __func__);
		return ret;
	}

	ret = sysfs_create_group(&ts->dev->kobj, &cmd_attr_group);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail - sysfs_create_group\n", __func__);
		goto err_sysfs;
	}

	return ret;
err_sysfs:
	input_err(true, &ts->client->dev, "%s: fail\n", __func__);
	return ret;
}
