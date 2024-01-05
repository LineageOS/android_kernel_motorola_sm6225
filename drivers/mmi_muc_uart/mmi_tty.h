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

#ifndef _DRIVERS_TTY_MMI_H_
#define _DRIVERS_TTY_MMI_H_

#define MMI_TTY_DEFAULT_IDX 0
#define MMI_TTY_DEBUG_IDX 1

int mmi_tty_init(int (*tty_rx_cb)(struct platform_device *pdev,
	int idx, uint8_t *payload, size_t len),
	int max_write_size,
	struct platform_device *pdev);
void mmi_tty_exit(void);
int mmi_tty_push_to_us(int idx, uint8_t *buf, size_t sz);

#endif /* _DRIVERS_TTY_MMI_H_ */
