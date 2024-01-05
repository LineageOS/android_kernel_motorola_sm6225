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

#ifndef _DRIVERS_TTY_DIAG_H_
#define _DRIVERS_TTY_DIAG_H_

#include <linux/usb/usbdiag.h>

int diag_tty_init(void);
void diag_tty_exit(void);
struct usb_diag_ch *tty_diag_channel_open(const char *name, void *priv,
		void (*notify)(void *, unsigned, struct diag_request *));
void tty_diag_channel_close(struct usb_diag_ch *diag_ch);
int tty_diag_channel_read(struct usb_diag_ch *diag_ch,
				struct diag_request *d_req);
int tty_diag_channel_write(struct usb_diag_ch *diag_ch,
				struct diag_request *d_req);

#endif /* _DRIVERS_TTY_DIAG_H_ */
