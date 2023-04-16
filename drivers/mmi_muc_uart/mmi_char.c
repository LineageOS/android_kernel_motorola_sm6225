/* Copyright (c) 2018, Motorola Mobility LLC. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "mmi_char.h"
#include "muc_uart.h"

#define NUM_MINORS 1

static struct platform_device *parent_pdev;
static struct cdev mmi_char_cdev;
static struct class *mmi_char_class;
static dev_t mmi_char_dev_num;
wait_queue_head_t mmi_char_read_wq;
static int mmi_char_major;
static int mmi_char_max_write_size;
static atomic_t mmi_char_open_excl;

static int mmi_char_open(struct inode *inode, struct file *file)
{
	if (atomic_xchg(&mmi_char_open_excl, 1))
		return -EBUSY;

	return 0;
}

static int mmi_char_release(struct inode *inode, struct file *file)
{
	WARN_ON(!atomic_xchg(&mmi_char_open_excl, 0));

	return 0;
}

static ssize_t mmi_char_write(struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	if(count > mmi_char_max_write_size)
		return -ENOSPC;

	return muc_uart_pb_write(parent_pdev, buf, count);
}

static ssize_t mmi_char_read(struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int ret;

	if (!(file->f_flags & O_NONBLOCK)) {
		ret = wait_event_interruptible(mmi_char_read_wq,
			muc_uart_pb_get_q_size(parent_pdev));
		if(ret < 0)
			goto exit;
	}

	/* Count gets updated to bytes read, ret will
	 * be 0 if ok, and negative on an error
	 */
	ret = muc_uart_pb_get_q(parent_pdev, buf, &count);
	if(ret >= 0)
		ret = count;

exit:
	return ret;
}

static unsigned int mmi_char_poll(struct file *file,
	struct poll_table_struct *pll_table)
{
	int ret = 0;

	poll_wait(file, &mmi_char_read_wq, pll_table);
	if(muc_uart_pb_get_q_size(parent_pdev))
		ret |= POLLIN;

	return ret;
}

static const struct file_operations mmi_char_fops = {
	.write		= mmi_char_write,
	.read		= mmi_char_read,
	.open		= mmi_char_open,
	.llseek		= noop_llseek,
	.release	= mmi_char_release,
	.poll		= mmi_char_poll,
};

int mmi_char_init(int max_write_size, struct platform_device *pdev)
{
	int ret;
	mmi_char_max_write_size = max_write_size;
	parent_pdev = pdev;

	ret = alloc_chrdev_region(&mmi_char_dev_num, 0, NUM_MINORS, "muc");
	if (ret < 0)
		goto err_dev;

	mmi_char_major = MAJOR(mmi_char_dev_num);
	mmi_char_dev_num = MKDEV(mmi_char_major, 0);
	mmi_char_class = class_create(THIS_MODULE, "mmi_char_class");

	cdev_init(&mmi_char_cdev, &mmi_char_fops);
	ret = cdev_add(&mmi_char_cdev, mmi_char_dev_num, 1);
	if(ret)
		goto err_cdev;

	if(!device_create(mmi_char_class, NULL,
		mmi_char_dev_num, NULL, "muc%d", 0))
		goto err_cdev;

	atomic_set(&mmi_char_open_excl, 0);

	init_waitqueue_head(&mmi_char_read_wq);
	muc_uart_pb_register_wq(pdev, &mmi_char_read_wq);

	return 0;
err_cdev:
	unregister_chrdev_region(mmi_char_dev_num, NUM_MINORS);
err_dev:
	return 1;
}

void mmi_char_exit(void)
{
	device_destroy(mmi_char_class, mmi_char_dev_num);
	unregister_chrdev_region(mmi_char_dev_num, NUM_MINORS);
	class_destroy(mmi_char_class);
}
