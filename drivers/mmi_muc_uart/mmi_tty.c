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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include "mmi_tty.h"

/* TODO only 1 device for non-debug version. */
#define MMI_TTY_MINOR_COUNT 2
#define MMI_TTY_DRIVER_NAME "tty_muc"
#define MMI_TTY_DEVICE_NAME "ttymuc"

static DEFINE_SPINLOCK(mmi_tty_lock);

static struct tty_driver *mmi_tty_driver;
static dev_t mmi_tty_dev_no;

struct mmi_tty_data {
	struct platform_device *pdev;
	struct tty_port port;
	struct tty_struct *tty;
	int open_count;
};

static struct mmi_tty_data mmi_tty[MMI_TTY_MINOR_COUNT];

static int (*mmi_tty_rx_cb)(struct platform_device *pdev,
	int idx,
	uint8_t *payload,
	size_t len);
static int mmi_max_write_size;

static int mmi_tty_open(struct tty_struct *tty, struct file *f)
{
	int n = tty->index;
	struct mmi_tty_data *tty_data;
	unsigned long flags;

	if (n < 0 || n >= MMI_TTY_MINOR_COUNT)
		return -ENODEV;

	tty_data = mmi_tty + n;

	if (tty_data->open_count >= 1)
		return -EBUSY;

	spin_lock_irqsave(&mmi_tty_lock, flags);

	tty_data->tty = tty;
	tty->driver_data = tty_data;
	tty_data->open_count++;

	spin_unlock_irqrestore(&mmi_tty_lock, flags);

	return 0;
}

static void mmi_tty_close(struct tty_struct *tty, struct file *f)
{
	struct mmi_tty_data *tty_data = tty->driver_data;
	unsigned long flags;

	if (tty_data == NULL)
		return;

	spin_lock_irqsave(&mmi_tty_lock, flags);
	tty_data->open_count--;

	if (tty_data->open_count == 0) {
		tty_data->tty = NULL;
		tty->driver_data = NULL;
	}

	spin_unlock_irqrestore(&mmi_tty_lock, flags);
}

static int mmi_tty_write(struct tty_struct *tty,
				const unsigned char *buf, int len)
{
	struct mmi_tty_data *tty_data = tty->driver_data;
	int ret;

	/* If multiple users are trying to open/close/write to
	 * the same device the kernel panics sometimes.
	 */
	if (!tty_data)
		return -ENODEV;

	pr_debug("muc_uart tty: Writing %d bytes.\n", len);
	ret = (*mmi_tty_rx_cb)(tty_data->pdev,
		tty->index,
		(uint8_t *)buf,
		(size_t)len);
	pr_debug("muc_uart tty: Done with ret %d.\n", ret);
	return ret;
}

static int mmi_tty_write_room(struct tty_struct *tty)
{
	return mmi_max_write_size;
}

static int mmi_tty_chars_in_buffer(struct tty_struct *tty)
{
	/* Data is always written when available, this should be changed if
	 * write to userspace is changed to delayed work
	 */
	return 0;
}

static const struct tty_operations mmi_tty_ops = {
	.open = mmi_tty_open,
	.close = mmi_tty_close,
	.write = mmi_tty_write,
	.write_room = mmi_tty_write_room,
	.chars_in_buffer = mmi_tty_chars_in_buffer,
};

/* Write a message to userspace */
int mmi_tty_push_to_us(int idx, uint8_t *buf, size_t sz)
{
	struct mmi_tty_data *tty_data;
	unsigned char *tty_buf;
	int tty_allocated;
	unsigned long flags;

	if (idx < 0 || idx >= MMI_TTY_MINOR_COUNT)
		return -ENODEV;

	tty_data = &(mmi_tty[idx]);

	spin_lock_irqsave(&mmi_tty_lock, flags);

	if (tty_data->tty == NULL) {
		spin_unlock_irqrestore(&mmi_tty_lock, flags);
		return -EIO;
	}

	tty_allocated = tty_prepare_flip_string(&tty_data->port,
						&tty_buf, sz);

	if (tty_allocated < sz) {
		spin_unlock_irqrestore(&mmi_tty_lock, flags);
		return -ENOMEM;
	}

	memcpy(tty_buf, buf, sz);
	tty_flip_buffer_push(&tty_data->port);
	pr_debug("muc_uart tty: Pushed %zd bytes.\n", sz);
	spin_unlock_irqrestore(&mmi_tty_lock, flags);

	return 0;
}

int mmi_tty_init(int (*tty_rx_cb)(struct platform_device *pdev,
	int idx, uint8_t *payload, size_t len),
	int max_write_size,
	struct platform_device *pdev)
{
	int result;
	int i;

	mmi_tty_rx_cb = tty_rx_cb;
	mmi_max_write_size = max_write_size;

	mmi_tty_driver = alloc_tty_driver(MMI_TTY_MINOR_COUNT);
	if (mmi_tty_driver == NULL)
		return -ENOMEM;

	mmi_tty_driver->owner = THIS_MODULE;
	mmi_tty_driver->driver_name = MMI_TTY_DRIVER_NAME;
	mmi_tty_driver->name = MMI_TTY_DEVICE_NAME;
	mmi_tty_driver->minor_start = 0;
	mmi_tty_driver->type = TTY_DRIVER_TYPE_SYSTEM;
	mmi_tty_driver->subtype = SYSTEM_TYPE_TTY;
	mmi_tty_driver->init_termios = tty_std_termios;
	mmi_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD;
	mmi_tty_driver->init_termios.c_iflag = IGNBRK;
	mmi_tty_driver->init_termios.c_oflag = 0;
	mmi_tty_driver->init_termios.c_lflag = 0;
	mmi_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;

	tty_set_operations(mmi_tty_driver, &mmi_tty_ops);

	result = tty_register_driver(mmi_tty_driver);
	if (result) {
		pr_err("mmi_tty Failed to register mmi_tty driver with major num :%d, res: %d.\n",
			MAJOR(mmi_tty_dev_no),
			result);
		put_tty_driver(mmi_tty_driver);
		mmi_tty_driver = NULL;
		return result;
	}

	for (i = 0; i < MMI_TTY_MINOR_COUNT; i++) {
		tty_port_init(&mmi_tty[i].port);
		tty_port_register_device(&mmi_tty[i].port, mmi_tty_driver,
				i, NULL);
		mmi_tty[i].pdev = pdev;
	}

	pr_info("mmi_tty TTY Registered OK\n");

	return 0;
}

void mmi_tty_exit(void)
{
	int i;

	if (mmi_tty_driver) {
		for (i = MMI_TTY_MINOR_COUNT - 1; i >= 0; i--)
			tty_unregister_device(mmi_tty_driver, i);
		tty_unregister_driver(mmi_tty_driver);
		put_tty_driver(mmi_tty_driver);
	}
}
