/* linux/drivers/input/misc/ldc2114-cdev.c
 *
 * LDC2114 chardev driver. Allows user space access to raw data.
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

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#define DRVNAME "ldc2114_raw"
#define BUFFER_SIZE 256

typedef struct data {
	int d[4];
} item_t;

struct buffer {
	struct mutex mutex;
	int capacity;
	unsigned long opened;
	atomic_t start;
	atomic_t end;
	atomic_t active;
	item_t **data;
	wait_queue_head_t wait;
};

struct ldc2114_drv {
	struct class *class;
	struct cdev cdev;
	struct platform_device *pdev;
	dev_t devid;
	int major;
	struct buffer ds;
	struct blocking_notifier_head nhead;
};

static struct ldc2114_drv *cxt;

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

	for (i = 0; i < buffer->capacity; i++)
		kfree(buffer->data[i]);
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

	dev_dbg(&context->pdev->dev, "%s: writing at pos %d\n", __func__, cend);
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

	dev_dbg(&context->pdev->dev, "%s: next write=%d, read=%d\n",
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
	dev_dbg(&context->pdev->dev, "%s: reading from pos %d\n",
			__func__, cstart);

	count += sizeof(item_t);

	cactive--;
	cstart = (cstart + 1) % buffer->capacity;
	dev_dbg(&context->pdev->dev, "%s: next read pos %d\n",
			__func__, cstart);

	atomic_set(&buffer->start, cstart);
	atomic_set(&buffer->active, cactive);

	mutex_unlock(&buffer->mutex);

	return count;
}

/*
 * External function called by polling work
 */
int ldc2114_buffer(int d0, int d1, int d2, int d3)
{
	item_t data;
	int ret = 0;

	if (cxt && test_bit(0, &cxt->ds.opened)) {
		data.d[0] = d0;
		data.d[1] = d1;
		data.d[2] = d2;
		data.d[3] = d3;
		buffer_push(&cxt->ds, &data, sizeof(data));
	} else
		ret = -ENODEV;

	return ret;
}

static ssize_t ldc2114_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct ldc2114_drv *context = file->private_data;
	int ret;
	item_t data;
	char out[128];

	ret = buffer_pop(&context->ds, &data);
	if (ret < 0)
		return ret;

	count = snprintf(out, sizeof(out), "%5d %5d %5d %5d\n",
				data.d[0], data.d[1], data.d[2], data.d[3]);

	if (copy_to_user(buf, out, count))
		count = -EFAULT;

	return count;
}

static int ldc2114_open(struct inode *inode, struct file *file)
{
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

	blocking_notifier_call_chain(&cxt->nhead, cxt->ds.opened, NULL);
	dev_dbg(&cxt->pdev->dev, "allocated circular buffer\n");

	return nonseekable_open(inode, file);
}

static int ldc2114_release(struct inode *inode, struct file *file)
{
	struct ldc2114_drv *context = file->private_data;

	__clear_bit_unlock(0, &context->ds.opened);
	blocking_notifier_call_chain(&context->nhead, context->ds.opened, NULL);
	buffer_destroy(&context->ds);

	dev_dbg(&cxt->pdev->dev, "destroyed circular buffer\n");

	return 0;
}

const struct file_operations ldc2114_cdev_fops = {
	.owner = THIS_MODULE,
	.open = ldc2114_open,
	.read = ldc2114_read,
	.release = ldc2114_release,
	.llseek = no_llseek,
};

int ldc2114_register_client(struct notifier_block *nb)
{
	if (!cxt)
		return -ENODEV;

	return blocking_notifier_chain_register(&cxt->nhead, nb);
}

int ldc2114_cdev_init(void)
{
	struct device *cd;
	int rc;

	cxt = kzalloc(sizeof(*cxt), GFP_KERNEL);
	if (!cxt)
		return -ENOMEM;

	cxt->pdev = platform_device_alloc(DRVNAME, 0);
	if (!cxt->pdev)
		return -ENOMEM;

	rc = platform_device_add(cxt->pdev);
	if (rc)
		goto undo_malloc;

	rc = register_chrdev(0, DRVNAME, &ldc2114_cdev_fops);
	if (rc < 0) {
		dev_err(&cxt->pdev->dev, "error register chardev: %d\n", rc);
		goto undo_platform_device_add;
	}

	cxt->major = rc;

	cxt->class = class_create(THIS_MODULE, DRVNAME "-dev");
	if (IS_ERR(cxt->class)) {
		rc = PTR_ERR(cxt->class);
		dev_err(&cxt->pdev->dev, "error chrdev: %d\n", rc);
		goto undo_register_chrdev;
	}

	cd = device_create(cxt->class, NULL, MKDEV(cxt->major, 0), NULL, DRVNAME);
	if (IS_ERR(cd)) {
		rc = PTR_ERR(cd);
		dev_err(&cxt->pdev->dev, "error chrdev create: %d\n", rc);
		goto undo_class_create;
	}

 	BLOCKING_INIT_NOTIFIER_HEAD(&cxt->nhead);

	return 0;

undo_class_create:
	class_destroy(cxt->class);
undo_register_chrdev:
	unregister_chrdev(cxt->major, DRVNAME);
undo_platform_device_add:
	platform_device_del(cxt->pdev);
undo_malloc:
	platform_device_put(cxt->pdev);
	kfree(cxt);

	return rc;
}

void ldc2114_cdev_remove(void)
{
	if (!cxt)
		return;

	device_destroy(cxt->class, MKDEV(cxt->major, 0));
	class_destroy(cxt->class);
	unregister_chrdev(cxt->major, DRVNAME);
	platform_device_unregister(cxt->pdev);
	kfree(cxt);
}

