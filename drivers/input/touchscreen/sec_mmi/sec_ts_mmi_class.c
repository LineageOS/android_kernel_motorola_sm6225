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
#include <soc/qcom/mmi_boot_info.h>
#include <linux/touchscreen_mmi.h>

#include "sec_ts.h"
#include "sec_mmi.h"

#define GET_TS_DATA(dev) { \
	ts = dev_get_drvdata(dev); \
	if (!ts) { \
		dev_err(dev, "%s: Failed to get drv data\n", __func__); \
		return -ENODEV; \
	} \
}

static ssize_t sec_mmi_suppression_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_suppression_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_pill_region_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_pill_region_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_hold_distance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_hold_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_data_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_gs_distance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_gs_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_hold_grip_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_mmi_hold_grip_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_mmi_mutual_range_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(address, (S_IWUSR | S_IWGRP), NULL, sec_mmi_address_store);
static DEVICE_ATTR(size, (S_IWUSR | S_IWGRP), NULL, sec_mmi_size_store);
static DEVICE_ATTR(write, (S_IWUSR | S_IWGRP), NULL, sec_mmi_write_store);
static DEVICE_ATTR(data, S_IRUGO, sec_mmi_data_show, NULL);
static DEVICE_ATTR(mutual_range, S_IRUGO, sec_mmi_mutual_range_show, NULL);
static DEVICE_ATTR(suppression, (S_IRUGO | S_IWUSR | S_IWGRP),
		sec_mmi_suppression_show, sec_mmi_suppression_store);
static DEVICE_ATTR(pill_region, (S_IRUGO | S_IWUSR | S_IWGRP),
		sec_mmi_pill_region_show, sec_mmi_pill_region_store);
static DEVICE_ATTR(hold_distance, (S_IRUGO | S_IWUSR | S_IWGRP),
		sec_mmi_hold_distance_show, sec_mmi_hold_distance_store);
static DEVICE_ATTR(gs_distance, (S_IRUGO | S_IWUSR | S_IWGRP),
		sec_mmi_gs_distance_show, sec_mmi_gs_distance_store);
static DEVICE_ATTR(hold_grip, (S_IRUGO | S_IWUSR | S_IWGRP),
		sec_mmi_hold_grip_show, sec_mmi_hold_grip_store);

#define MAX_ATTRS_ENTRIES 10
#define ADD_ATTR(name) { \
	if (idx < MAX_ATTRS_ENTRIES)  { \
		dev_info(dev, "%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
		ext_attributes[idx] = &dev_attr_##name.attr; \
		idx++; \
	} else { \
		dev_err(dev, "%s: cannot add attribute '%s'\n", __func__, #name); \
	} \
}

static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

static int sec_mmi_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	struct sec_ts_data *ts;
	int idx = 0;

	GET_TS_DATA(dev);

	if (ts->plat_data->suppression_ctrl)
		ADD_ATTR(suppression);

	if (ts->plat_data->pill_region_ctrl)
		ADD_ATTR(pill_region);

	if (ts->plat_data->hold_distance_ctrl)
		ADD_ATTR(hold_distance);

	if (ts->plat_data->gs_distance_ctrl)
		ADD_ATTR(gs_distance);

	if (ts->plat_data->hold_grip_ctrl)
		ADD_ATTR(hold_grip);

	if (strncmp(bi_bootmode(), "mot-factory", strlen("mot-factory")) == 0) {
		ADD_ATTR(address);
		ADD_ATTR(size);
		ADD_ATTR(data);
		ADD_ATTR(write);
		ADD_ATTR(mutual_range);
	}

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

void sec_mmi_gesture_handler(void *data) {
	struct sec_ts_gesture_status *gs =
		(struct sec_ts_gesture_status *)data;

	if (gs->eid != SEC_TS_GESTURE_EVENT) {
		pr_info("%s: invalid gesture ID\n", __func__);
		return;
	}

	pr_info("%s: GESTURE %x %x %x %x %x %x %x %x\n", __func__,
		gs->eid | (gs->stype << 4) | (gs->sf << 6),
		gs->gesture_id,
		gs->gesture_data_1,
		gs->gesture_data_2,
		gs->gesture_data_3,
		gs->gesture_data_4,
		gs->reserved_1,
		gs->left_event_5_0 | (gs->reserved_2 << 6));

	switch (gs->gesture_id) {
	case 1:
		pr_info("%s: single tap\n", __func__);
			break;
	case 2:
		pr_info("%s: zero tap; x=%x, y=%x, w=%x, p=%x\n", __func__,
			gs->gesture_data_1+((gs->gesture_data_3 & 0x0f) << 8),
			gs->gesture_data_2+((gs->gesture_data_3 & 0xf0) << 4),
			gs->gesture_data_4,
			gs->reserved_1);
			break;
	default:
		pr_info("%s: unknown id=%x\n", __func__, gs->gesture_id);
	}
}

static void sec_mmi_ic_reset(struct sec_ts_data *ts, int mode)
{
	if (!gpio_is_valid(ts->plat_data->rst_gpio) ||
		!gpio_get_value(ts->plat_data->rst_gpio))
		return;

	PM_STAY_AWAKE(ts->wakelock);
	mutex_lock(&ts->modechange);
	/* disable irq to ensure getting boot complete */
	sec_ts_irq_enable(ts, false);

	if (mode == 1) {
		gpio_set_value(ts->plat_data->rst_gpio, 0);
		usleep_range(10, 10);
		gpio_set_value(ts->plat_data->rst_gpio, 1);
		dev_dbg(&ts->client->dev, "%s: reset line toggled\n", __func__);
	} else {
		int ret;

		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
		if (ret < 0)
			dev_err(&ts->client->dev, "%s: error sending sw reset cmd\n", __func__);
	}

	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	sec_ts_irq_enable(ts, true);

	mutex_unlock(&ts->modechange);
	PM_RELAX(ts->wakelock);

	dev_dbg(&ts->client->dev, "%s: hw reset done\n", __func__);
}

static void sec_mmi_enable_touch(struct sec_ts_data *ts)
{
	int ret;
	unsigned char buffer[8];

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, buffer, sizeof(buffer));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: failed to read fw version (%d)\n",
			__func__, ret);
	} else {
		if (buffer[0] == 0x17) {
			ts->device_id[3] = 0x7C;
			input_info(true, &ts->client->dev,
				"%s: set device_id[3] to 0x7C\n", __func__);
		}
	}
	sec_ts_integrity_check(ts);
	sec_ts_sense_on(ts);
	dev_dbg(&ts->client->dev, "%s: touch sensing ready\n", __func__);
}

static ssize_t sec_mmi_suppression_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts;
	unsigned char buffer;
	int error;
	unsigned long value;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	error = kstrtoul(buf, 0, &value);
	if (error)
		return -EINVAL;

	buffer = (unsigned char)value;
	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)buffer);

	if (ts->suppression_data == buffer) {
		dev_dbg(dev, "%s: value is same,so not write.\n", __func__);
		return size;
	}
	ts->suppression_data = buffer;
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		return size;
	}

	error = ts->sec_ts_i2c_write(ts, SEC_TS_GRIP_SUPPRESSION_INFO, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to write suppression info (%d)\n",
				__func__, error);

	error = ts->sec_ts_i2c_read(ts, SEC_TS_GRIP_SUPPRESSION_INFO, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to read suppression info (%d)\n",
				__func__, error);
	else
		dev_dbg(dev, "%s: suppression info 0x%02x\n", __func__, (unsigned int)buffer);

	return size;
}

static ssize_t sec_mmi_suppression_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts;
	unsigned char buffer;
	ssize_t blen = 0;
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)(ts->suppression_data));
		return blen;
	}
	error = ts->sec_ts_i2c_read(ts, SEC_TS_GRIP_SUPPRESSION_INFO, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to read suppression info (%d)\n",
				__func__, error);
	else {
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)buffer);
	}

	return blen;
}

int sec_mmi_sysfs_notify(struct sec_ts_data *ts, unsigned char state)
{
	if (state != 0xff)
		ts->hold_grip_data = state;

	sysfs_notify(ts->imports->kobj_notify, NULL, "hold_grip");
	dev_dbg(&ts->client->dev, "%s: triggered HAL with 0x%02x\n", __func__, state);

	return 0;
}

static ssize_t sec_mmi_hold_grip_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	sec_mmi_sysfs_notify(ts, 0xff);

	return size;
}

static ssize_t sec_mmi_hold_grip_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	return scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)(ts->hold_grip_data));
}

#define REQ_ARGS_NUM 3

static ssize_t sec_mmi_pill_region_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts;
	unsigned int args[3] = {0};
	unsigned char buffer[5];
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	error = sscanf(buf, "0x%x 0x%x 0x%x", &args[0], &args[1], &args[2]);
	if (error < REQ_ARGS_NUM)
		return -EINVAL;

	buffer[0] = (unsigned char)args[0];
	buffer[1] = (unsigned char)((args[1] >> 8) & 0xff);
	buffer[2] = (unsigned char)(args[1] & 0xff);
	buffer[3] = (unsigned char)((args[2] >> 8) & 0xff);
	buffer[4] = (unsigned char)(args[2] & 0xff);
	dev_dbg(dev, "%s: program pill region 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
		(unsigned int)buffer[0], (unsigned int)buffer[1],
		(unsigned int)buffer[2], (unsigned int)buffer[3],
		(unsigned int)buffer[4]);

	if (!memcmp(buffer, ts->pill_region_data, sizeof(buffer))) {
		dev_dbg(dev, "%s: value is same,so not write.\n", __func__);
		return size;
	}

	memcpy(ts->pill_region_data, buffer, sizeof(ts->pill_region_data));
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		return size;
	}

	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_PILL_REGION, buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to write pill region (%d)\n",
				__func__, error);

	error = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_PILL_REGION, buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to read pill region (%d)\n",
				__func__, error);
	else {
		dev_dbg(dev, "%s: pill region 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
		(unsigned int)buffer[0], (unsigned int)buffer[1],
		(unsigned int)buffer[2], (unsigned int)buffer[3],
		(unsigned int)buffer[4]);
	}

	return size;
}

static ssize_t sec_mmi_pill_region_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts;
	unsigned char buffer[5] = {0};
	ssize_t blen = 0;
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		memcpy(buffer, ts->pill_region_data, sizeof(buffer));
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x 0x%x 0x%x", (unsigned int)buffer[0],
			(unsigned int)buffer[2] | (buffer[1] << 8),
			(unsigned int)buffer[4] | (buffer[3] << 8));
		return blen;
	}
	error = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_PILL_REGION, buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to read pill region (%d)\n",
				__func__, error);
	else {
		dev_dbg(dev, "%s: pill region 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
			(unsigned int)buffer[0], (unsigned int)buffer[1],
			(unsigned int)buffer[2], (unsigned int)buffer[3],
			(unsigned int)buffer[4]);

		blen += scnprintf(buf, PAGE_SIZE, "0x%02x 0x%x 0x%x", (unsigned int)buffer[0],
			(unsigned int)buffer[2] | (buffer[1] << 8),
			(unsigned int)buffer[4] | (buffer[3] << 8));
	}

	return blen;
}

static ssize_t sec_mmi_hold_distance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts;
	unsigned char buffer;
	unsigned long value;
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	error = kstrtoul(buf, 0, &value);
	if (error)
		return -EINVAL;

	buffer = (unsigned char)value;
	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)buffer);

	if (ts->hold_distance_data == buffer) {
		dev_dbg(dev, "%s: value is same,so not write.\n", __func__);
		return size;
	}
	ts->hold_distance_data = buffer;
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		return size;
	}

	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_HOLD_DISTANCE, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to write hold distance (%d)\n",
				__func__, error);

	return size;
}

static ssize_t sec_mmi_hold_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts;
	unsigned char buffer;
	ssize_t blen = 0;
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)(ts->hold_distance_data));
		return blen;
	}

	error = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_HOLD_DISTANCE, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to read hold distance info (%d)\n",
				__func__, error);
	else {
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)buffer);
	}

	return blen;
}

static ssize_t sec_mmi_gs_distance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts;
	unsigned char buffer;
	unsigned long value;
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	error = kstrtoul(buf, 0, &value);
	if (error)
		return -EINVAL;

	buffer = (unsigned char)value;
	dev_dbg(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)buffer);

	if (ts->gs_distance_data == buffer) {
		dev_dbg(dev, "%s: value is same,so not write.\n", __func__);
		return size;
	}
	ts->gs_distance_data = buffer;
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		return size;
	}

	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_GRIP_SUPP_AREA, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to write grip suppression distance (%d)\n",
				__func__, error);

	return size;
}

static ssize_t sec_mmi_gs_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts;
	unsigned char buffer;
	ssize_t blen = 0;
	int error;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_dbg(dev, "%s: power off state\n", __func__);
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)(ts->gs_distance_data));
		return blen;
	}

	error = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_GRIP_SUPP_AREA, &buffer, sizeof(buffer));
	if (error < 0)
		dev_err(dev, "%s: failed to read grip suppression distance info (%d)\n",
				__func__, error);
	else {
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)buffer);
	}

	return blen;
}

static ssize_t sec_mmi_mutual_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char ostate = STATE_MANAGE_OFF;
	unsigned char tmode[2] = { TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH };
	int i, error;
	short p, min = 1, max = -1;
	struct sec_ts_data *ts;
	unsigned int readbytes;
	unsigned char *pRead;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_TS_DATA(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF)
		goto not_ready;

	readbytes = ts->rx_count * ts->rx_count * 2;
	pRead = kzalloc(readbytes, GFP_KERNEL);
	if (!pRead)
		return -ENOMEM;

	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, &ostate, sizeof(ostate));
	if (error < 0) {
		dev_err(dev, "%s: cannot set manage state: %d\n", __func__, error);
		goto free_and_leave;
	}
	sec_ts_delay(20);

	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CHG_SYSMODE, tmode, sizeof(tmode));
	if (error < 0) {
		dev_err(dev, "%s: cannot set sysmode: %d\n", __func__, error);
		goto restore_mode;
	}
	sec_ts_delay(20);

	ostate = TYPE_AMBIENT_DATA;
	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &ostate, sizeof(ostate));
	if (error < 0) {
		dev_err(dev, "%s: cannot set rawdata type: %d\n", __func__, error);
		goto restore_mode;
	}
	sec_ts_delay(50);

	error = ts->sec_ts_i2c_read(ts, SEC_TS_READ_TOUCH_RAWDATA, pRead, readbytes);
	if (error < 0) {
		dev_err(dev, "%s: cannot set rawdata type: %d\n", __func__, error);
		goto release_afe;
	}
	sec_ts_delay(50);

	for (i = 0; i < readbytes; i+=2) {
		p = pRead[i + 1] + (pRead[i] << 8);
		if (!i) {
			min = max = p;
			continue;
		}
		if (p < min)
			min = p;
		else if (p > max)
			max = p;
	}

release_afe:
	ostate = TYPE_INVALID_DATA;
	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &ostate, sizeof(ostate));
	if (error < 0)
		dev_err(dev, "%s: cannot restore rawdata type: %d\n", __func__, error);
restore_mode:
	ostate = STATE_MANAGE_ON;
	error = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, &ostate, sizeof(ostate));
	if (error < 0)
		dev_err(dev, "%s: cannot restore manage state: %d\n", __func__, error);
	sec_ts_delay(20);
free_and_leave:
	kfree(pRead);
not_ready:
	return scnprintf(buf, PAGE_SIZE, "%d,%d", min, max);
}

#define MAX_DATA_SZ	1024
static u8 reg_address;
static int data_size;

static ssize_t sec_mmi_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int error;
	long value;

	error = kstrtol(buf, 0, &value);
	if (error || value > MAX_DATA_SZ)
		return -EINVAL;

	reg_address = (u8)value;
	dev_info(dev, "%s: read address 0x%02X\n", __func__, reg_address);

	return size;
}

static ssize_t sec_mmi_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int error;
	long value;

	error = kstrtol(buf, 0, &value);
	if (error)
		return -EINVAL;

	data_size = (unsigned int)value;
	dev_info(dev, "%s: read size %u\n", __func__, data_size);

	return size;
}

static ssize_t sec_mmi_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	char ascii[3] = {0};
	char *sptr, *eptr;
	size_t data_sz = size;
	u8 hex[MAX_DATA_SZ];
	int byte, error;
	long value;

	dev = MMI_DEV_TO_TS_DEV(dev);

	if (*(buf + size - 1) == '\n')
		data_sz--;

	if ((data_sz%2 != 0) || (data_sz/2 > MAX_DATA_SZ)) {
		dev_err(dev, "%s: odd input\n", __func__);
		return -EINVAL;
	}

	sptr = (char *)buf;
	eptr = (char *)buf + data_sz;
	pr_debug("%s: data to write: ", __func__);
	for (byte = 0; sptr < eptr; sptr += 2, byte++) {
		memcpy(ascii, sptr, 2);
		error = kstrtol(ascii, 16, &value);
		if (error)
			break;
		hex[byte] = (u8)value;
		pr_cont("0x%02x ", hex[byte]);
	}
	pr_cont("; total=%d\n", byte);

	if (error) {
		dev_err(dev, "%s: input conversion failed\n", __func__);
		return -EINVAL;
	}

	error = sec_ts_reg_store(dev, attr, hex, byte);
	if (error != byte) {
		dev_err(dev, "%s: write error\n", __func__);
		return -EIO;
	}

	dev_info(dev, "%s: written %d bytes to address 0x%02X\n",
			__func__, byte, hex[0]);
	return size;
}

static ssize_t sec_mmi_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 buffer[MAX_DATA_SZ];
	ssize_t length, blen = 0;
	int i;

	dev = MMI_DEV_TO_TS_DEV(dev);

	/* init parameters required for sec_ts_regread_show() */
	sec_ts_lv1_params(reg_address, data_size);
	length = sec_ts_regread_show(dev, attr, buffer);
	if (length != data_size)
		return sprintf(buf,
			"error %zu reading %u bytes from reg addr=0x%02X\n",
			length, data_size, reg_address);

	dev_info(dev, "%s: read %u bytes from reg addr=0x%02X\n",
			__func__, data_size, reg_address);
	for (i = 0; i < length; i++)
		blen += scnprintf(buf + blen, PAGE_SIZE - blen, "%02X ", buffer[i]);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "\n");

	return blen;
}


static int sec_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "samsung");
}

static int sec_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%c%c%c%02x",
		tolower(ts->device_id[0]), tolower(ts->device_id[1]),
		ts->device_id[2], ts->device_id[3]);
}


static inline void bdc2ui(unsigned int *dest, unsigned char *src, size_t size)
{
	int i, shift = 0;
	*dest = 0;
	for (i = 0; i < size; i++, shift += 8)
		*dest += src[i] << (24 - shift);
}

static int sec_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned char buffer[8];
	unsigned int build_id = 0;
	int ret;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(dev, "%s: failed to read fw version (%d)\n",
				__func__, ret);
	else {
		if (buffer[0] == 0x17)
			ts->device_id[3] = 0x7C;
		bdc2ui(&build_id, buffer, sizeof(build_id));
		dev_dbg(dev, "%s: IMAGE VER: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%08x", build_id);
}

static int sec_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned char buffer[8];
	unsigned int config_id = 0;
	int ret;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_PARA_VERSION, buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(dev, "%s: failed to read fw version (%d)\n",
				__func__, ret);
	else {
		bdc2ui(&config_id, buffer+4, sizeof(config_id));
		dev_dbg(dev, "%s: CONFIG VER: " \
				"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
				__func__, buffer[0], buffer[1], buffer[2], buffer[3],
				buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%08x", config_id);
}

static int sec_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}

static int sec_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = gpio_get_value(ts->plat_data->irq_gpio);

	return 0;
}

static int sec_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = ts->irq_enabled ? 1 : 0;

	return 0;
}

static int sec_mmi_methods_get_poweron(struct device *dev, void *idata) {

	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = (ts->power_status != SEC_TS_STATE_POWER_OFF) ? 1 : 0;

	return 0;
}

static void sec_mmi_print_version(struct sec_ts_data *ts)
{
	char prod_info[TS_MMI_MAX_INFO_LEN];
	char build_id[TS_MMI_MAX_ID_LEN];
	char config_id[TS_MMI_MAX_ID_LEN];

	sec_mmi_methods_get_productinfo(&ts->client->dev, prod_info);
	if (ts->fw_invalid == true) {
		dev_info(&ts->client->dev, "Product %s in bootloader mode\n",
			prod_info);
		return;
	}

	sec_mmi_methods_get_build_id(&ts->client->dev, build_id);
	sec_mmi_methods_get_config_id(&ts->client->dev, config_id);
	dev_info(&ts->client->dev, "Product %s, firmware id: %s, config_id: %s\n",
		prod_info, build_id, config_id);
}

static int sec_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	TO_INT(idata) = ts->fw_invalid ? 1 : 0;

	return 0;
}


static int sec_mmi_methods_drv_irq(struct device *dev, int state) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	switch (state) {
	case 0: /* Disable irq */
		sec_ts_irq_enable(ts, false);
			break;
	case 1: /* Enable irq */
		sec_ts_irq_enable(ts, true);
			break;
	default:
			dev_err(dev, "%s: invalid value\n", __func__);
			return -EINVAL;
	}
	return 0;

}

static int sec_mmi_methods_reset(struct device *dev, int type) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	sec_mmi_ic_reset(ts, type);

	return 0;
}

static int sec_mmi_methods_power(struct device *dev, int on) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	if (on)
		ts->power_status = SEC_TS_STATE_POWER_ON;
	else
		ts->power_status = SEC_TS_STATE_POWER_OFF;

	return sec_ts_power((void *)ts, on == TS_MMI_POWER_ON);
}

static int sec_mmi_methods_refresh_rate(struct device *dev, int freq) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned char f = freq & 0xff;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	dev_info(dev, "%s: writing refresh rate %dhz\n", __func__, freq);
	ret = ts->sec_ts_i2c_write(ts, 0x4C, &f, 1);
	if (ret < 0) {
		dev_err(dev, "%s: failed refresh_rate (%d)\n", __func__, ret);
		return ret;
	}
	return 0;
}

static int sec_mmi_update_baseline(struct device *dev, int mode) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned char d = (mode == TS_MMI_UPDATE_BASELINE_ON) ? 0x01 : 0x00;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	dev_info(dev, "%s: update baseline %s\n", __func__,
		(mode == TS_MMI_UPDATE_BASELINE_ON) ? "enable" : "disable");
	ret = ts->sec_ts_i2c_write(ts, 0x5f, &d, 1);
	if (ret < 0) {
		dev_err(dev, "%s: failed set baseline update (%d)\n", __func__, ret);
		return ret;
	}
	return 0;
}

static int sec_mmi_methods_pinctrl(struct device *dev, int on) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	return sec_ts_pinctrl_configure(ts, on == TS_MMI_PINCTL_ON);
}

static int sec_mmi_firmware_update(struct device *dev, char *fwname) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	int result = -EFAULT;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	/* Loading Firmware------------------------------------------ */
	if (request_firmware(&fw_entry, fwname, &ts->client->dev) != 0) {
		dev_err(dev, "%s: fw not available\n", __func__);
		goto err_request_fw;
	}
	dev_info(dev, "%s: update fw from %s, size = %d\n",
			__func__, fwname, (int)fw_entry->size);

	mutex_lock(&ts->modechange);
	sec_ts_irq_enable(ts, false);
	PM_STAY_AWAKE(ts->wakelock);

	result = sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size,
				0, false, 0); /* do not run calibration!!! */
	if (ts->fw_invalid == false)
		sec_mmi_enable_touch(ts);

	sec_mmi_print_version(ts);

	mutex_unlock(&ts->modechange);
	sec_ts_irq_enable(ts, true);
	PM_RELAX(ts->wakelock);

err_request_fw:
	release_firmware(fw_entry);

	return result;
}

static int sec_mmi_firmware_erase(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_err(dev, "%s: Power off state\n", __func__);
		return -EIO;
	}

	mutex_lock(&ts->modechange);
	sec_ts_irq_enable(ts, false);
	PM_STAY_AWAKE(ts->wakelock);

	ts->fw_invalid = true;

	mutex_unlock(&ts->modechange);
	PM_RELAX(ts->wakelock);

	return 0;
}

static int sec_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	switch (to) {
	case TS_MMI_PM_DEEPSLEEP:
		return sec_ts_set_lowpowermode(ts, TO_SLEEP_MODE);
	case TS_MMI_PM_GESTURE:
		return sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
	case TS_MMI_PM_ACTIVE:
		return sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
	default:
		dev_warn(dev, "panel mode %d is invalid.\n", to);
		return -EINVAL;
	}
}

static int sec_mmi_wait_for_ready(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	if (!ts->plat_data->regulator_boot_on)
		sec_ts_delay(70);

	return sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
}

static int sec_mmi_pre_suspend(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned long start_wait_jiffies = jiffies;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	do {
		if (!ts->wakelock->active)
			break;
		usleep_range(1000, 1000);
	} while (1);

	if ((jiffies - start_wait_jiffies))
		dev_info(dev, "%s: entering suspend delayed for %ums\n",
			__func__, jiffies_to_msecs(jiffies - start_wait_jiffies));

	return 0;
}

static int sec_mmi_post_suspend(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	return 0;
}

static int sec_mmi_post_resume(struct device *dev) {
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned char buffer = 0;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	dev_dbg(dev, "%s: sending sense_on...\n", __func__);
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		dev_err(dev, "%s: failed sense_on (%d)\n", __func__, ret);

	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	if (ts->plat_data->suppression_ctrl) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_GRIP_SUPPRESSION_INFO,
						&ts->suppression_data, sizeof(ts->suppression_data));
		if (ret < 0)
			dev_err(dev, "%s: failed to write suppression info (%d)\n",
					__func__, ret);
	}

	if(ts->plat_data->pill_region_ctrl) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_PILL_REGION,
						ts->pill_region_data, sizeof(ts->pill_region_data));
		if (ret < 0)
			dev_err(dev, "%s: failed to write pill region (%d)\n",
					__func__, ret);
	}

	if(ts->plat_data->hold_distance_ctrl) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_HOLD_DISTANCE,
						&ts->hold_distance_data, sizeof(ts->hold_distance_data));
		if (ret < 0)
			dev_err(dev, "%s: failed to write hold distance (%d)\n",
					__func__, ret);
	}

	if(ts->plat_data->gs_distance_ctrl) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_GRIP_SUPP_AREA,
						&ts->gs_distance_data, sizeof(ts->gs_distance_data));
		if (ret < 0)
			dev_err(dev, "%s: failed to write grip supp distance (%d)\n",
					__func__, ret);
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS,
					&buffer, sizeof(buffer));
	if (ret < 0)
		dev_err(dev, "%s: failed to read boot status (%d)\n", __func__, ret);

	ts->fw_invalid = buffer == SEC_TS_STATUS_BOOT_MODE;

	return 0;
}

static int sec_mmi_charger_mode(struct device *dev, int mode)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned char cval = 0;
	unsigned char bitval = 1;

	if (!ts) {
		dev_err(dev, "Failed to get driver data");
		return -ENODEV;
	}

	/* plugged in USB should change value to 2 */
	if (mode)
		bitval++;

	ret = ts->sec_ts_i2c_read(ts, SET_TS_CMD_SET_CHARGER_MODE,
					&cval, sizeof(cval));
	if (ret < 0) {
		dev_err(dev, "%s: failed to read charger mode\n", __func__);
	}

	if (cval != bitval)
		sec_ts_set_charger(ts, mode);
	else
		dev_dbg(dev, "%s: charger mode already %d\n", __func__, cval);
	return 0;
}

static struct ts_mmi_methods sec_ts_mmi_methods = {
	.get_vendor = sec_mmi_methods_get_vendor,
	.get_productinfo = sec_mmi_methods_get_productinfo,
	.get_build_id = sec_mmi_methods_get_build_id,
	.get_config_id = sec_mmi_methods_get_config_id,
	.get_bus_type = sec_mmi_methods_get_bus_type,
	.get_irq_status = sec_mmi_methods_get_irq_status,
	.get_drv_irq = sec_mmi_methods_get_drv_irq,
	.get_poweron = sec_mmi_methods_get_poweron,
	.get_flashprog = sec_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  sec_mmi_methods_reset,
	.drv_irq = sec_mmi_methods_drv_irq,
	.power = sec_mmi_methods_power,
	.pinctrl = sec_mmi_methods_pinctrl,
	.refresh_rate = sec_mmi_methods_refresh_rate,
	.charger_mode = sec_mmi_charger_mode,
	.update_baseline = sec_mmi_update_baseline,
	/* Firmware */
	.firmware_update = sec_mmi_firmware_update,
	.firmware_erase = sec_mmi_firmware_erase,
	/* vendor specific attribute group */
	.extend_attribute_group = sec_mmi_extend_attribute_group,
	/* PM callback */
	.panel_state = sec_mmi_panel_state,
	.wait_for_ready = sec_mmi_wait_for_ready,
	.post_resume = sec_mmi_post_resume,
	.pre_suspend = sec_mmi_pre_suspend,
	.post_suspend = sec_mmi_post_suspend,
};

static int sec_mmi_class_fname(struct sec_ts_data *ts)
{
	int ret;
	char buffer[128] = "samsung_mmi";
	const char* fname = NULL;
	static char *input_dev_name;

	if (ts->imports && ts->imports->get_class_fname) {
		ret = ts->imports->get_class_fname(&ts->client->dev, &fname);
		if (!ret)
			scnprintf(buffer, sizeof(buffer), "samsung_mmi.%s", fname);
	}

	input_dev_name = kstrdup(buffer, GFP_KERNEL);
	ts->input_dev->name = input_dev_name;
	return 0;
}

int sec_mmi_data_init(struct sec_ts_data *ts, bool enable)
{
	int ret;

	if (enable) {
		ret = ts_mmi_dev_register(&ts->client->dev, &sec_ts_mmi_methods);
		if (ret) {
			dev_err(&ts->client->dev, "Failed to register ts mmi\n");
			return ret;
		}

		/* initialize class imported methods */
		ts->imports = &sec_ts_mmi_methods.exports;

		if (ts->fw_invalid == false) {
			sec_mmi_enable_touch(ts);
		} else /* stuck in BL mode, update productinfo to report 'se77c' */
			ts->device_id[3] = 0x7C;

		sec_mmi_print_version(ts);
		sec_mmi_class_fname(ts);

		/*initialize value*/
		if (ts->plat_data->suppression_ctrl) {
			ret = ts->sec_ts_i2c_read(ts, SEC_TS_GRIP_SUPPRESSION_INFO,
							&ts->suppression_data, sizeof(ts->suppression_data));
			if (ret < 0)
				dev_err(&ts->client->dev, "%s: failed to write suppression info (%d)\n",
						__func__, ret);
		}

		if(ts->plat_data->pill_region_ctrl) {
			ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_PILL_REGION,
							ts->pill_region_data, sizeof(ts->pill_region_data));
			if (ret < 0)
				dev_err(&ts->client->dev, "%s: failed to write pill region (%d)\n",
						__func__, ret);
		}

		if(ts->plat_data->hold_distance_ctrl) {
			ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_HOLD_DISTANCE,
							&ts->hold_distance_data, sizeof(ts->hold_distance_data));
			if (ret < 0)
				dev_err(&ts->client->dev, "%s: failed to write hold distance (%d)\n",
						__func__, ret);
		}

		if(ts->plat_data->gs_distance_ctrl) {
			ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_GRIP_SUPP_AREA,
							&ts->gs_distance_data, sizeof(ts->gs_distance_data));
			if (ret < 0)
				dev_err(&ts->client->dev, "%s: failed to write grip supp distance (%d)\n",
						__func__, ret);

			if (ts->gs_distance_data == 0xFF || ret < 0) {
				dev_err(&ts->client->dev, "%s: previous firmware dose not support gs-distance, set default value\n",
                        __func__);
				ts->gs_distance_data = 0x1E;
			}
		}

		if (ts->plat_data->hold_grip_ctrl)
			ts->hold_grip_data = SEC_HOLD_GRIP_NOGRIP;
	} else
		ts_mmi_dev_unregister(&ts->client->dev);

	return 0;
}
