/* linux/drivers/input/misc/ldc2114-debugfs.c
 *
 * LDC2114 debugfs driver. Allows user space access to raw data.
 * Copyright (C) 2017 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#if defined(CONFIG_DEBUG_FS)

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "ldc2114.h"

#define DRVNAME "ldc2114_raw"

static int buffer_init(struct buffer *buffer, size_t capacity)
{
	int i;

	buffer->data = kzalloc(capacity * sizeof(item_t *), GFP_KERNEL);
	if (buffer->data == NULL)
		return -ENOMEM;
	for (i = 0; i < capacity; i++) {
		buffer->data[i] = kzalloc(sizeof(item_t), GFP_KERNEL);
		if (buffer->data[i] == NULL)
			return -ENOMEM;
	}
	buffer->capacity = capacity;
	buffer->opened = 0UL;
	atomic_set(&buffer->start, 0);
	atomic_set(&buffer->end, 0);
	atomic_set(&buffer->active, 0);
	mutex_init(&buffer->mutex);
	init_waitqueue_head(&buffer->wait);

	return 0;
}

static void inline buffer_destroy(struct buffer *buffer)
{
	int i;

	mutex_lock(&buffer->mutex);
	buffer->capacity = 0;
	atomic_set(&buffer->start, 0);
	atomic_set(&buffer->end, 0);
	atomic_set(&buffer->active, 0);
	/* wait for remaining msgs */
	wait_event_interruptible(buffer->wait,
			!atomic_read(&buffer->active));
	for (i = 0; i < buffer->capacity; i++)
		kfree(buffer->data[i]);
	mutex_unlock(&buffer->mutex);
	kfree(buffer->data);
}

static void buffer_push(struct buffer *buffer,
				void *data, size_t length)
{
	struct ldc2114_drv *context =
				container_of(buffer, struct ldc2114_drv, ds);
	int cactive, cstart, cend;

	mutex_lock(&buffer->mutex);

	cactive = atomic_read(&buffer->active);
	cstart = atomic_read(&buffer->start);
	cend = atomic_read(&buffer->end);

	dev_dbg(context->dev, "%s: writing at pos %d\n", __func__, cend);
	memcpy(&buffer->data[cend], data, length);

	cend = (cend + 1) % buffer->capacity;
	atomic_set(&buffer->end, cend);

	if (cactive < buffer->capacity) {
		cactive++;
		atomic_set(&buffer->active, cactive);
	} else {
		cstart = (cstart + 1) % buffer->capacity;
		atomic_set(&buffer->start, cstart);
	}

	dev_dbg(context->dev, "%s: next write=%d, read=%d\n",
		__func__, cend, cstart);

	mutex_unlock(&buffer->mutex);

	wake_up_interruptible(&buffer->wait);
}

static size_t buffer_pop(struct buffer *buffer, item_t *item)
{
	struct ldc2114_drv *context =
				container_of(buffer, struct ldc2114_drv, ds);
	int cactive, cstart;
	size_t count = 0;

	if (wait_event_interruptible(buffer->wait,
		atomic_read(&buffer->active)))
		return -ERESTARTSYS;

	mutex_lock(&buffer->mutex);

	cactive = atomic_read(&buffer->active);
	cstart = atomic_read(&buffer->start);

	memcpy(item, &buffer->data[cstart], sizeof(item_t));
	dev_dbg(context->dev, "%s: reading from pos %d\n",
			__func__, cstart);

	count += sizeof(item_t);

	cactive--;
	cstart = (cstart + 1) % buffer->capacity;
	dev_dbg(context->dev, "%s: next read pos %d\n",
			__func__, cstart);

	atomic_set(&buffer->start, cstart);
	atomic_set(&buffer->active, cactive);

	mutex_unlock(&buffer->mutex);

	return count;
}

/*
 * External function called by polling work
 */
int ldc2114_buffer(struct ldc2114_data *ldc, int type, ...)
{
	struct ldc2114_drv *cxt = dev_get_drvdata(ldc->dev);
	va_list vp;
	item_t data;
	int rc = 0;

	if (!(cxt && test_bit(0, &cxt->ds.opened)))
		return -ENODEV;

	va_start(vp, type);
	data.type = type;
	switch (type) {
		case LDC2114_EV_DATA:
			data.de = va_arg(vp, struct ldc2114_raw_ext);
				break;
		default:
				rc = -EINVAL;
				goto leave_now;
	}
	buffer_push(&cxt->ds, &data, sizeof(data));

leave_now:
	va_end(vp);

	return rc;
}

static int inline comp2_12b(struct ldc2114_16bit *data)
{
	int result;
	/* 12bits 2's compliment data */
	if (data->msb & 0x8)
		result = (data->lsb | (data->msb << 8) | 0xFFFFF000);
	else
		result = (data->lsb | (data->msb << 8));

	return result;
}

static ssize_t ldc2114_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct ldc2114_drv *context = file->private_data;
	int i, ret;
	item_t data;
	char out[128];

	ret = buffer_pop(&context->ds, &data);
	if (ret < 0)
		return ret;

	switch(data.type) {
		case LDC2114_EV_DATA:
			count = snprintf(out, sizeof(out), "I:%d",
						(data.de.status & LDC2114_REG_STATUS_OUT) != 0);
			for (i = 0; i < MAX_KEYS; i++) {
				count += snprintf(out+count, sizeof(out)-count, " [%d]:%d/%d",
						i, (data.de.data.out & (1 << i)) != 0,
						comp2_12b(&data.de.data.values[i]));
			}
			count += snprintf(out+count, sizeof(out)-count, "\n");
				break;
	}

	if (copy_to_user(buf, out, count))
		count = -EFAULT;

	return count;
}

static int ldc2114_open(struct inode *inode, struct file *file)
{
	struct ldc2114_drv *cxt = inode->i_private;
	int ret;

	if (!cxt)
		return -ENODEV;

	file->private_data = cxt;

	ret = buffer_init(&cxt->ds, BUFFER_SIZE);
	if (ret)
		return -ENOMEM;

	if (test_and_set_bit_lock(0, &cxt->ds.opened)) {
		buffer_destroy(&cxt->ds);
		return -EBUSY;
	}

	dev_dbg(cxt->dev, "%s: allocated circular buffer\n", __func__);
	blocking_notifier_call_chain(&cxt->nhead, cxt->ds.opened, NULL);

	return nonseekable_open(inode, file);
}

static int ldc2114_release(struct inode *inode, struct file *file)
{
	struct ldc2114_drv *context = file->private_data;

	clear_bit_unlock(0, &context->ds.opened);
	dev_dbg(context->dev, "%s: opend flag %lu\n", __func__, context->ds.opened);
	blocking_notifier_call_chain(&context->nhead, context->ds.opened, NULL);
	buffer_destroy(&context->ds);

	dev_dbg(context->dev, "%s: destroyed circular buffer\n", __func__);

	return 0;
}

const struct file_operations ldc2114_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = ldc2114_open,
	.read = ldc2114_read,
	.release = ldc2114_release,
	.llseek = no_llseek,
};

int ldc2114_register_client(struct ldc2114_data *ldc, struct notifier_block *nb)
{
	struct ldc2114_drv *cxt = dev_get_drvdata(ldc->dev);

	dev_dbg(ldc->dev, "cxt %p\n", cxt);
	if (!cxt)
		return -ENODEV;
	dev_dbg(ldc->dev, "%s: registering notifier\n", __func__);

	return blocking_notifier_chain_register(&cxt->nhead, nb);
}

static char *INSTANCE(char *string, struct ldc2114_data *ldc)
{
	char instance[64];
	snprintf(instance, sizeof(instance), "%s.%d",
			string, ldc->instance);
	return kstrdup(instance, GFP_KERNEL);
}

static struct dentry *root_dir;

int ldc2114_debugfs_init(struct ldc2114_data *ldc)
{
	struct ldc2114_drv *cxt;
	int rc = 0;

	if (!root_dir)
		root_dir = debugfs_create_dir(DRVNAME, NULL);

	if (!root_dir)
		return -ENODEV;

	cxt = kzalloc(sizeof(*cxt), GFP_KERNEL);
	if (!cxt) {
		rc = -ENOMEM;
		goto remove_dbg_root;
	}

	cxt->name = INSTANCE(DRVNAME, ldc);
	cxt->dev = ldc->dev;
	dev_set_drvdata(ldc->dev, cxt);

	cxt->dentry = debugfs_create_file(cxt->name,
					0444, root_dir, cxt, &ldc2114_debugfs_fops);
	dev_dbg(ldc->dev, "%s: debugfs file '%s' created\n", __func__, cxt->name);

 	BLOCKING_INIT_NOTIFIER_HEAD(&cxt->nhead);

	return 0;

remove_dbg_root:
	debugfs_remove(root_dir);

	return rc;
}

void ldc2114_debugfs_remove(struct ldc2114_data *ldc)
{
	struct ldc2114_drv *cxt = dev_get_drvdata(ldc->dev);

	if (!cxt)
		return;

	debugfs_remove(cxt->dentry);
	kfree(cxt);
	dev_set_drvdata(ldc->dev, NULL);
}
#endif
