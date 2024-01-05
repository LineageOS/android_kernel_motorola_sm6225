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
#ifndef _DRIVERS_MUC_UART_H_
#define _DRIVERS_MUC_UART_H_

int muc_uart_pb_get_q_size(struct platform_device *pdev);
int muc_uart_pb_get_q(struct platform_device *pdev,
	char __user *buf,
	size_t *count);
int muc_uart_pb_register_wq(struct platform_device *pdev,
	wait_queue_head_t *read_wq);
int muc_uart_pb_write(struct platform_device *pdev,
	const char __user *buf,
	size_t count);

#endif /* _DRIVERS_MUC_UART_H_ */
