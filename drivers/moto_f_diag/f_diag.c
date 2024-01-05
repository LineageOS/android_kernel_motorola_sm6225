/* f_diag.c
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2017, The Linux Foundation. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 * Copyright (c) 2018, Motorola Mobility LLC.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/usb/usbdiag.h>

#include "tty_diag.h"

struct usb_diag_ch *usb_diag_open(const char *name, void *priv,
		void (*notify)(void *, unsigned int, struct diag_request *))
{
	return tty_diag_channel_open(name, priv, notify);
}
EXPORT_SYMBOL(usb_diag_open);

void usb_diag_close(struct usb_diag_ch *ch)
{
	return tty_diag_channel_close(ch);
}
EXPORT_SYMBOL(usb_diag_close);

int usb_diag_alloc_req(struct usb_diag_ch *ch, int n_write, int n_read)
{
	return 0;
}
EXPORT_SYMBOL(usb_diag_alloc_req);
#define DWC3_MAX_REQUEST_SIZE (16 * 1024 * 1024)

int usb_diag_request_size(struct usb_diag_ch *ch)
{
	return DWC3_MAX_REQUEST_SIZE;
}
EXPORT_SYMBOL(usb_diag_request_size);

int usb_diag_read(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	return tty_diag_channel_read(ch, d_req);
}
EXPORT_SYMBOL(usb_diag_read);

int usb_diag_write(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	return tty_diag_channel_write(ch, d_req);
}
EXPORT_SYMBOL(usb_diag_write);

static int __init diag_init(void)
{
	return diag_tty_init();
}

static void __exit diag_exit(void)
{
	diag_tty_exit();
}

module_init(diag_init);
module_exit(diag_exit);

MODULE_DESCRIPTION("TTY diag function driver");
MODULE_LICENSE("GPL v2");
