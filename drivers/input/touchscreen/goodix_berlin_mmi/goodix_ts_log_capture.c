/*
 * Copyright (C) 2023 Motorola Mobility LLC
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
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "goodix_ts_core.h"

#define LOG_FIFO_SIZE 8172
#define GOODIX_LOG_DEVICE_NAME "tp_tools"

struct goodix_ts_log {
	struct kfifo        fifo;
	wait_queue_head_t   wq;
	struct miscdevice   miscdev;
};

static struct goodix_ts_log ts_log_dev;

/* Save a byte to a FIFO and discard the oldest byte if FIFO is full */
void put_fifo_with_discard(char *log_buf, int len)
{
	int ret = 0;
	if (!kfifo_initialized(&ts_log_dev.fifo))
		return;
	if (kfifo_is_full(&ts_log_dev.fifo)) {
		kfifo_skip(&ts_log_dev.fifo);
		ts_info("Save a byte to a FIFO and discard the oldest byte if FIFO is full");
	}

	ret = kfifo_in(&ts_log_dev.fifo, log_buf, len);

	wake_up_interruptible(&ts_log_dev.wq);
}

void clear_kfifo(void)
{
	if (kfifo_len(&ts_log_dev.fifo) != 0) {
		kfifo_reset(&ts_log_dev.fifo);
	}
}

static __poll_t log_file_poll(struct file *file,
                    struct poll_table_struct *pt)
{
	struct goodix_ts_log *log = file->private_data;

	poll_wait(file, &log->wq, pt);
	return !kfifo_is_empty(&log->fifo) ? (POLLPRI|POLLIN) : 0;
}

static ssize_t log_file_read(struct file *file, char __user *buffer,
                size_t count, loff_t *ppos)
{
	struct goodix_ts_log *log = file->private_data;
	unsigned int copied;
	int ret = 0;

	if (kfifo_is_empty(&log->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(log->wq,
			!kfifo_is_empty(&log->fifo));
		if (ret == -ERESTARTSYS)
			return -EINTR;
	}
	ret = kfifo_to_user(&log->fifo, buffer, count, &copied);
	if (ret)
		return ret;

	return copied;
}

static int goodix_log_device_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ts_log_dev;
	ts_info("success open log device");

	return 0;
}

static const struct file_operations log_device_fops = {
	.owner  = THIS_MODULE,
	.open    = goodix_log_device_open,
	.read   = log_file_read,
	.poll   = log_file_poll,
	.llseek = noop_llseek,
};

int goodix_log_capture_register_misc(struct goodix_ts_core *cd)
{
	int rc = 0;

	init_waitqueue_head(&ts_log_dev.wq);
	/* Create FIFO datastructure */
	rc = kfifo_alloc(&ts_log_dev.fifo,
		LOG_FIFO_SIZE, GFP_KERNEL);
	if (rc)
		return rc;

	mutex_init(&cd->frame_log_lock);

	ts_log_dev.miscdev.minor = MISC_DYNAMIC_MINOR;
	ts_log_dev.miscdev.name = GOODIX_LOG_DEVICE_NAME;
	ts_log_dev.miscdev.fops = &log_device_fops;
	rc = misc_register(&ts_log_dev.miscdev);
	if (rc)
		return rc;
	return 0;
}

int goodix_log_capture_unregister_misc(struct goodix_ts_core *cd)
{
	kfifo_free(&ts_log_dev.fifo);
	misc_deregister(&ts_log_dev.miscdev);
	mutex_destroy(&cd->frame_log_lock);
	return 0;
}
