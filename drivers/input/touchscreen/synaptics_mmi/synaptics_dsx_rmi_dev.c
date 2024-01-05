/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

#include "synaptics_dsx_i2c.h"

#define CHAR_DEVICE_NAME "rmi"
#define DEVICE_CLASS_NAME "rmidev"
#define DEV_NUMBER 1
#define MAX_INSTANCE 5
#define REG_ADDR_LIMIT 0xFFFF

static char *rmi_cdev_node(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);
	return kasprintf(GFP_KERNEL, "rmi/%s", dev_name(dev));
}

static struct class rmi_device_class = {
	.owner = THIS_MODULE,
	.name = DEVICE_CLASS_NAME,
	.devnode = rmi_cdev_node,
};

static int rmidev_major_num;

static ssize_t rmidev_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t rmidev_sysfs_data_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t rmidev_sysfs_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_release_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_length_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_attn_state_show(struct device *dev,
		struct device_attribute *attr, char *buf);

struct rmidev_handle {
	dev_t dev_no;
	unsigned short address;
	unsigned int length;
	struct device dev;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_exp_fn_ptr *fn_ptr;
	struct completion remove_complete;
	void *data;
};

struct rmidev_data {
	int ref_count;
	struct cdev main_dev;
	struct class *device_class;
	struct mutex file_mutex;
	struct rmidev_handle *rmi_dev;
	struct temp_buffer data_buf;
};

static const struct bin_attribute attr_data = {
	.attr = {
		.name = "data",
		.mode = (S_IRUGO | S_IWUSR | S_IWGRP),
	},
	.size = 0,
	.read = rmidev_sysfs_data_show,
	.write = rmidev_sysfs_data_store,
};

static const struct device_attribute attrs[] = {
	__ATTR(open, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_open_store),
	__ATTR(release, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_release_store),
	__ATTR(address, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_address_store),
	__ATTR(length, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_length_store),
	__ATTR(attn_state, S_IRUSR | S_IRGRP,
			rmidev_sysfs_attn_state_show,
			NULL),
};

static ssize_t rmidev_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct rmidev_data *dev_data = data_file->private_data;
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	int retval;
	unsigned int data_length = rmidev->length;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	if (data_length > (REG_ADDR_LIMIT - rmidev->address))
		data_length = REG_ADDR_LIMIT - rmidev->address;

	if (count < data_length) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Not enough space (%zu bytes) in buffer\n",
				__func__, count);
		return -EINVAL;
	}

	if (data_length) {
		retval = rmidev->fn_ptr->read(rmi4_data,
				rmidev->address,
				(unsigned char *)buf,
				data_length);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to read data\n",
					__func__);
			return retval;
		}
	} else {
		return -EINVAL;
	}

	return data_length;
}

static ssize_t rmidev_sysfs_data_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct rmidev_data *dev_data = data_file->private_data;
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	int retval;
	unsigned int data_length = rmidev->length;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	if (data_length > (REG_ADDR_LIMIT - rmidev->address))
		data_length = REG_ADDR_LIMIT - rmidev->address;

	if (data_length) {
		retval = rmidev->fn_ptr->write(rmidev->rmi4_data,
				rmidev->address,
				(unsigned char *)buf,
				data_length);
		if (retval < 0) {
			dev_err(&rmidev->rmi4_data->i2c_client->dev,
					"%s: Failed to write data\n",
					__func__);
			return retval;
		}
	} else {
		return -EINVAL;
	}

	return count;
}

static ssize_t rmidev_sysfs_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct rmidev_data *dev_data =
					dev_get_drvdata(dev);
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	unsigned int input;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input != 1)
		return -EINVAL;

	rmidev->fn_ptr->enable(rmi4_data, false);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Attention interrupt disabled\n",
			__func__);

	return count;
}

static ssize_t rmidev_sysfs_release_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct rmidev_data *dev_data =
					dev_get_drvdata(dev);
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	unsigned int input;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input != 1)
		return -EINVAL;

	rmidev->fn_ptr->enable(rmi4_data, true);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Attention interrupt enabled\n",
			__func__);

	return count;
}

static ssize_t rmidev_sysfs_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct rmidev_data *dev_data =
					dev_get_drvdata(dev);
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	unsigned int input;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input > REG_ADDR_LIMIT)
		return -EINVAL;

	rmidev->address = (unsigned short)input;

	return count;
}

static ssize_t rmidev_sysfs_length_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct rmidev_data *dev_data =
					dev_get_drvdata(dev);
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	unsigned int input;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input > REG_ADDR_LIMIT)
		return -EINVAL;

	rmidev->length = input;

	return count;
}

static ssize_t rmidev_sysfs_attn_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rmidev_data *dev_data =
					dev_get_drvdata(dev);
	struct rmidev_handle *rmidev = dev_data->rmi_dev;
	struct synaptics_rmi4_data *rmi4_data = rmidev->rmi4_data;
	int attn_state;
	const struct synaptics_dsx_platform_data *platform_data =
			&rmi4_data->board;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: dev_data %p, rmidev %p, rmi4_data %p\n",
			__func__, dev_data, rmidev, rmi4_data);

	attn_state = gpio_get_value(platform_data->irq_gpio);

	return snprintf(buf, PAGE_SIZE, "%u\n", attn_state);
}

/*
 * rmidev_llseek - used to set up register address
 *
 * @filp: file structure for seek
 * @off: offset
 *   if whence == SEEK_SET,
 *     high 16 bits: page address
 *     low 16 bits: register address
 *   if whence == SEEK_CUR,
 *     offset from current position
 *   if whence == SEEK_END,
 *     offset from end position (0xFFFF)
 * @whence: SEEK_SET, SEEK_CUR, or SEEK_END
 */
static loff_t rmidev_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	struct rmidev_data *dev_data = filp->private_data;
	struct rmidev_handle *rmidev;
	struct synaptics_rmi4_data *rmi4_data;

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	rmidev = dev_data->rmi_dev;
	rmi4_data = rmidev->rmi4_data;
	mutex_lock(&(dev_data->file_mutex));

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		newpos = REG_ADDR_LIMIT + off;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > REG_ADDR_LIMIT) {
		dev_err(&rmidev->rmi4_data->i2c_client->dev,
				"%s: New position 0x%04x is invalid\n",
				__func__, (unsigned int)newpos);
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: current position set %lld\n",
			__func__, newpos);
clean_up:
	mutex_unlock(&(dev_data->file_mutex));

	return newpos;
}

/*
 * rmidev_read: - use to read data from rmi device
 *
 * @filp: file structure for read
 * @buf: user space buffer pointer
 * @count: number of bytes to read
 * @f_pos: offset (starting register address)
 */
static ssize_t rmidev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval;
	struct rmidev_data *dev_data = filp->private_data;
	struct temp_buffer *tb;
	struct rmidev_handle *rmidev;
	struct synaptics_rmi4_data *rmi4_data;

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	rmidev = dev_data->rmi_dev;
	rmi4_data = rmidev->rmi4_data;
	mutex_lock(&(dev_data->file_mutex));

	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0) {
		retval = 0;
		goto clean_up;
	}

	if (*f_pos > REG_ADDR_LIMIT) {
		retval = -EFAULT;
		goto clean_up;
	}

	tb = &dev_data->data_buf;

	if (tb->buf_size < count && alloc_buffer(tb, count) != 0) {
		retval = -ENOMEM;
		goto clean_up;
        }

	retval = rmidev->fn_ptr->read(rmidev->rmi4_data,
			*f_pos,
			tb->buf,
			count);
	if (retval < 0)
		goto clean_up;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: read %zu bytes at offset %lld\n",
			__func__, count, *f_pos);

	if (copy_to_user(buf, tb->buf, count))
		retval = -EFAULT;
	else
		*f_pos += retval;

clean_up:
	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_write: - used to write data to rmi device
 *
 * @filep: file structure for write
 * @buf: user space buffer pointer
 * @count: number of bytes to write
 * @f_pos: offset (starting register address)
 */
static ssize_t rmidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval;
	struct rmidev_data *dev_data = filp->private_data;
	struct temp_buffer *tb;
	struct rmidev_handle *rmidev;
	struct synaptics_rmi4_data *rmi4_data;

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	rmidev = dev_data->rmi_dev;
	rmi4_data = rmidev->rmi4_data;
	mutex_lock(&(dev_data->file_mutex));

	if (*f_pos > REG_ADDR_LIMIT) {
		retval = -EFAULT;
		goto clean_up;
	}

	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0) {
		retval = 0;
		goto clean_up;
	}

	tb = &dev_data->data_buf;

	if (tb->buf_size < count && alloc_buffer(tb, count) != 0) {
		retval = -ENOMEM;
		goto clean_up;
	}

	if (copy_from_user(tb->buf, buf, count)) {
		retval = -EFAULT;
		goto clean_up;
	}

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: write %zu bytes at offset %lld\n",
			__func__, count, *f_pos);

	retval = rmidev->fn_ptr->write(rmidev->rmi4_data,
			*f_pos,
			tb->buf,
			count);
	if (retval >= 0)
		*f_pos += retval;

clean_up:
	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_open: enable access to rmi device
 * @inp: inode struture
 * @filp: file structure
 */
static int rmidev_open(struct inode *inp, struct file *filp)
{
	int retval = 0;
	struct rmidev_data *dev_data =
			container_of(inp->i_cdev, struct rmidev_data, main_dev);
	struct rmidev_handle *rmidev;

	if (!dev_data)
		return -EACCES;

	rmidev = dev_data->rmi_dev;
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: device data %p\n",
			__func__, dev_data);

	filp->private_data = dev_data;
	mutex_lock(&(dev_data->file_mutex));

	rmidev->fn_ptr->enable(rmidev->rmi4_data, false);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: Attention interrupt disabled\n",
			__func__);

	if (dev_data->ref_count < 1)
		dev_data->ref_count++;
	else
		retval = -EACCES;

	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_release: - release access to rmi device
 * @inp: inode structure
 * @filp: file structure
 */
static int rmidev_release(struct inode *inp, struct file *filp)
{
	struct rmidev_data *dev_data =
			container_of(inp->i_cdev, struct rmidev_data, main_dev);
	struct rmidev_handle *rmidev;

	if (!dev_data)
		return -EACCES;

	rmidev = dev_data->rmi_dev;

	mutex_lock(&(dev_data->file_mutex));

	dev_data->ref_count--;
	if (dev_data->ref_count < 0)
		dev_data->ref_count = 0;

	rmidev->fn_ptr->enable(rmidev->rmi4_data, true);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: Attention interrupt enabled\n",
			__func__);

	mutex_unlock(&(dev_data->file_mutex));

	return 0;
}

static const struct file_operations rmidev_fops = {
	.owner = THIS_MODULE,
	.llseek = rmidev_llseek,
	.read = rmidev_read,
	.write = rmidev_write,
	.open = rmidev_open,
	.release = rmidev_release,
};

static void rmidev_device_cleanup(struct rmidev_data *dev_data)
{
	if (dev_data) {
		dev_t devno;
		struct rmidev_handle *rmidev = dev_data->rmi_dev;

		devno = dev_data->main_dev.dev;

		if (dev_data->device_class)
			device_destroy(dev_data->device_class, devno);

		cdev_del(&dev_data->main_dev);

		dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
				"%s: rmidev device removed\n",
				__func__);
	}

	return;
}

static void rmidev_registered_release(struct device *dev)
{
	struct rmidev_handle *rmidev =
			container_of(dev, struct rmidev_handle, dev);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: rmidev release function called\n",
			__func__);
}

static int rmidev_init_device(struct synaptics_rmi4_data *rmi4_data)
{
	struct rmidev_handle *rmidev =
			(struct rmidev_handle *)rmi4_data->rmidev_data;
	int retval;
	dev_t dev_no;
	unsigned char attr_count;
	struct rmidev_data *dev_data;
	struct device *device_ptr;

	pr_debug("%s: enter rmi4_data %p\n", __func__, rmi4_data);

	rmidev->fn_ptr =  kzalloc(sizeof(*(rmidev->fn_ptr)), GFP_KERNEL);
	if (!rmidev) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fn_ptr\n",
				__func__);
		retval = -ENOMEM;
		goto err_fn_ptr;
	}

	rmidev->fn_ptr->read = rmi4_data->i2c_read;
	rmidev->fn_ptr->write = rmi4_data->i2c_write;
	rmidev->fn_ptr->enable = rmi4_data->irq_enable;
	rmidev->rmi4_data = rmi4_data;

	if (!rmidev_major_num) {
		retval = alloc_chrdev_region(&dev_no, rmi4_data->instance,
					MAX_INSTANCE, CHAR_DEVICE_NAME);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to allocate char device region %d\n",
				__func__, retval);
			goto err_device_region;
		}

		rmidev_major_num = MAJOR(dev_no);
		dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Major number of rmidev = %d\n",
			__func__, rmidev_major_num);
	} else
		dev_no = MKDEV(rmidev_major_num, rmi4_data->instance);

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for dev_data\n",
				__func__);
		retval = -ENOMEM;
		goto err_dev_data;
	}

	mutex_init(&dev_data->file_mutex);
	dev_data->rmi_dev = rmidev;
	dev_data->device_class = &rmi_device_class;
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Device data %p\n", __func__, dev_data);
	rmidev->data = dev_data;

	cdev_init(&dev_data->main_dev, &rmidev_fops);

	retval = cdev_add(&dev_data->main_dev, dev_no, 1);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to add rmi char device\n",
				__func__);
		goto err_char_device;
	}

	device_ptr = device_create(dev_data->device_class, NULL, dev_no,
			NULL, CHAR_DEVICE_NAME"%d", MINOR(dev_no));
	if (IS_ERR(device_ptr)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create rmi char device\n",
				__func__);
		retval = -ENODEV;
		goto err_char_device;
	}

	retval = gpio_export(rmi4_data->board.irq_gpio, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to export attention gpio\n",
				__func__);
	} else {
		retval = gpio_export_link(&(rmi4_data->i2c_client->dev),
				"attn", rmi4_data->board.irq_gpio);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s Failed to create gpio symlink\n",
					__func__);
		} else {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Exported attention gpio %d\n",
					__func__, rmi4_data->board.irq_gpio);
		}
	}

	rmidev->dev.class = &rmi_device_class;
	rmidev->dev.parent = &rmi4_data->i2c_client->dev;
	rmidev->dev.release = rmidev_registered_release;

	dev_set_name(&rmidev->dev, DEVICE_CLASS_NAME "%d", rmi4_data->instance);
	dev_set_drvdata(&rmidev->dev, dev_data);

	retval = device_register(&rmidev->dev);
	if (retval) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to register device\n",
				__func__);
		goto err_sysfs_dir;
	}

	retval = device_create_bin_file(&rmidev->dev, &attr_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto err_sysfs_bin;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = device_create_file(&rmidev->dev, &attrs[attr_count]);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto err_sysfs_attrs;
		}
	}

	return 0;

err_sysfs_attrs:
	for (attr_count--; attr_count >= 0; attr_count--)
		device_remove_file(&rmidev->dev, &attrs[attr_count]);

	device_remove_bin_file(&rmidev->dev, &attr_data);

err_sysfs_bin:
	device_unregister(&rmidev->dev);

err_sysfs_dir:
err_char_device:
	rmidev_device_cleanup(dev_data);
	kfree(dev_data);

err_dev_data:
	unregister_chrdev_region(dev_no, MAX_INSTANCE);

err_device_region:
	kfree(rmidev->fn_ptr);

err_fn_ptr:
	return retval;
}

static void rmidev_remove_device(struct synaptics_rmi4_data *rmi4_data)
{
	struct rmidev_handle *rmidev =
			(struct rmidev_handle *)rmi4_data->rmidev_data;
	unsigned char attr_count;
	struct rmidev_data *dev_data;

	pr_debug("%s: rmi4_data %p\n", __func__, rmi4_data);

	if (!rmidev)
		return;

	gpio_unexport(rmi4_data->board.irq_gpio);

	sysfs_remove_link(&rmi4_data->i2c_client->dev.kobj, "attn");

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		device_remove_file(&rmidev->dev, &attrs[attr_count]);

	device_remove_bin_file(&rmidev->dev, &attr_data);

	device_unregister(&rmidev->dev);

	dev_data = rmidev->data;
	if (dev_data) {
		rmidev_device_cleanup(dev_data);
		kfree(dev_data);
	}

	unregister_chrdev_region(rmidev->dev_no, 1);

	kfree(rmidev->fn_ptr);

	complete(&rmidev->remove_complete);

	return;
}

static int __init rmidev_module_init(void)
{
	int err;
	struct rmidev_handle *rmidev;
	struct synaptics_rmi4_data *next = NULL;

	err = class_register(&rmi_device_class);
	if (err) {
		pr_err("%s: couldn't register rmi device class\n", __func__);
		return err;
	}

	while ((next = synaptics_driver_getdata(next)) != NULL) {
		rmidev = kzalloc(sizeof(*rmidev), GFP_KERNEL);
		if (!rmidev) {
			dev_err(&next->i2c_client->dev,
					"%s: cannot allocate rmidev for rmi4_data %p\n",
					__func__, next);
			continue;
		}
		dev_dbg(&next->i2c_client->dev,
				"%s: new rmidev %p\n", __func__, rmidev);
		next->rmidev_data = (void *)rmidev;
		synaptics_rmi4_new_function(next, RMI_DEV, true,
				rmidev_init_device,
				rmidev_remove_device,
				NULL, NULL, IC_MODE_UI);
	}

	return 0;
}

static void __exit rmidev_module_exit(void)
{
	struct rmidev_handle *rmidev;
	struct synaptics_rmi4_data *next = NULL;

	while ((next = synaptics_driver_getdata(next)) != NULL) {
		rmidev = (struct rmidev_handle *)next->rmidev_data;
		dev_dbg(&next->i2c_client->dev,
				"%s: removing rmidev %p\n", __func__, rmidev);
		init_completion(&rmidev->remove_complete);
		synaptics_rmi4_new_function(next, RMI_DEV, false,
				rmidev_init_device,
				rmidev_remove_device,
				NULL, NULL, IC_MODE_UI);
		wait_for_completion(&rmidev->remove_complete);
		kfree(rmidev);
		next->rmidev_data = NULL;
	}

	unregister_chrdev_region(MKDEV(rmidev_major_num, 1), MAX_INSTANCE);
	class_unregister(&rmi_device_class);

	return;
}

module_init(rmidev_module_init);
module_exit(rmidev_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX RMI Dev Module");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SYNAPTICS_DSX_DRIVER_VERSION);
