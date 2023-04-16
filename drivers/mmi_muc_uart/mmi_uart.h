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
#ifndef _DRIVERS_MMI_UART_H_
#define _DRIVERS_MMI_UART_H_

#define UART_MAX_MSG_SIZE (2048)

int mmi_uart_open(void *uart_data);
int mmi_uart_close(void *uart_data);
int mmi_uart_send(void *uart_data, uint8_t *buf, size_t len);

/* Lock the UART while setting the baud or pm. */
int mmi_uart_set_tx_busy(void *uart_data);
void mmi_uart_clear_tx_busy(void *uart_data);
int mmi_uart_do_pm(void *uart_data, bool on);
int mmi_uart_get_baud(void *uart_data);
int mmi_uart_set_baud(void *uart_data, uint32_t baud);

/* sysfs err reporting */
void mmi_uart_report_rx_len_err(void *uart_data);
void mmi_uart_report_rx_crc_err(void *uart_data);
void mmi_uart_report_rx_mag_err(void *uart_data);
void mmi_uart_report_tx_err(void *uart_data);

/* Driver Initializations */
int mmi_uart_init(size_t (*uart_rx_cb)(struct platform_device *pdev,
					uint8_t *payload, size_t len),
				    int (*uart_register_cb)(struct platform_device *pdev,
					void *uart_data),
					void *(*uart_get_data)(struct device *dev),
					struct platform_device *pdev);
int mmi_uart_exit(void *uart_data);

#endif /* _DRIVERS_MMI_UART_H_ */
