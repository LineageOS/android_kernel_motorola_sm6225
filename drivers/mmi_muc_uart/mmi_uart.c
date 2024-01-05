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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/sysfs.h>
#include "mmi_uart.h"

#define DRIVERNAME	"mmi_uart"
#define N_MMI_UART	26

static DEFINE_SPINLOCK(tx_lock);

struct mmi_uart_err_stats {
	uint32_t tx_failure;
	uint32_t rx_crc;
	uint32_t rx_timeout;
	uint32_t rx_abort;
	uint32_t rx_len;
	uint32_t rx_mag;
};

struct mmi_uart_data {
	struct platform_device *pdev;
	struct tty_struct *tty;
	char rx_data[UART_MAX_MSG_SIZE];
	size_t rx_len;
	unsigned long last_rx;
	struct mmi_uart_err_stats stats;
	const char *tty_name;
	speed_t default_baud;
	bool tx_busy;
	bool tx_ready;
};

#define MMI_UART_SEGMENT_TIMEOUT 500 /* msec */

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Found in tty_io.c */
extern struct mutex tty_mutex;

static size_t (*mmi_uart_rx_cb)(struct platform_device *pdev,
	uint8_t *payload,
	size_t len);
static int (*mmi_uart_register_cb)(struct platform_device *pdev,
	void *uart_data);
static void *(*mmi_uart_get_data)(struct device *dev);

static ssize_t uart_stats_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mmi_uart_data *mud =
		(struct mmi_uart_data *)mmi_uart_get_data(dev);

	return scnprintf(buf, PAGE_SIZE,
			 "tx err:%d, rx crc:%d, rx timeout:%d, rx abort:%d, "
			 "rx len:%d, rx mag:%d\n",
			 mud->stats.tx_failure, mud->stats.rx_crc,
			 mud->stats.rx_timeout, mud->stats.rx_abort,
			 mud->stats.rx_len, mud->stats.rx_mag);
}

/* /sys/devices/soc:mod_uart/uart_stats */
static DEVICE_ATTR_RO(uart_stats);

static struct attribute *uart_attrs[] = {
	&dev_attr_uart_stats.attr,
	NULL,
};

ATTRIBUTE_GROUPS(uart);

void mmi_uart_report_rx_len_err(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;

	mud->stats.rx_len++;
}

void mmi_uart_report_rx_crc_err(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;

	mud->stats.rx_crc++;
}

void mmi_uart_report_rx_mag_err(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;

	mud->stats.rx_mag++;
}

void mmi_uart_report_tx_err(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;

	mud->stats.tx_failure++;
}

int mmi_uart_set_tx_busy(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&tx_lock, flags);
	if (mud->tx_busy)
		ret = 1;
	else {
		mud->tx_busy = 1;
		ret = 0;
	}
	spin_unlock_irqrestore(&tx_lock, flags);
	return ret;
}

void mmi_uart_clear_tx_busy(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	unsigned long flags;

	spin_lock_irqsave(&tx_lock, flags);
	mud->tx_busy = 0;
	spin_unlock_irqrestore(&tx_lock, flags);
}

static int mmi_uart_get_ready(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	unsigned long flags;
	int res;

	spin_lock_irqsave(&tx_lock, flags);
	res = mud->tx_ready;
	spin_unlock_irqrestore(&tx_lock, flags);

	return res;
}

static void mmi_uart_set_ready(void *uart_data, bool ready)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	unsigned long flags;

	spin_lock_irqsave(&tx_lock, flags);
	mud->tx_ready = ready;
	spin_unlock_irqrestore(&tx_lock, flags);
}

/* Only call with tx busy lock */
int mmi_uart_send(void *uart_data, uint8_t *buf, size_t len)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	struct device *dev = &mud->pdev->dev;
	int ret;

	print_hex_dump_debug("muc_uart tx: ", DUMP_PREFIX_OFFSET,
		16, 1, buf, len, true);

	if (!mmi_uart_get_ready(uart_data) || !mud->tty) {
		dev_err(dev, "%s: no tty\n", __func__);
		return -ENODEV;
	}

	ret = mud->tty->ops->write(mud->tty, buf, len);
	if (ret != len) {
		dev_err(dev, "%s: Failed to send message\n", __func__);
		goto send_err;
	}

	return 0;

send_err:
	mud->stats.tx_failure++;
	return -EIO;
}

int mmi_uart_get_baud(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	speed_t speed;

	if (!mud || !mud->tty) {
		pr_err("%s: no tty\n", __func__);
		return -ENODEV;
	}

	down_read(&mud->tty->termios_rwsem);
	speed = mud->tty->termios.c_ispeed;
	up_read(&mud->tty->termios_rwsem);

	return (int)speed;
}

static int config_tty(struct mmi_uart_data *mud, speed_t speed,
		      struct tty_struct *tty)
{
	struct ktermios kt;

	if (!speed)
		speed = mud->default_baud;
	dev_info(&mud->pdev->dev, "%s: speed=%d\n", __func__, speed);

	/* Use one stop bit (CSTOPB not set) and no parity (PARENB not set) */
	kt.c_cflag = 0;
	kt.c_cflag |= CLOCAL;  /* Ignore modem control lines */
	kt.c_cflag |= CREAD;   /* Enable receiver */
	kt.c_cflag |= CS8;     /* Character size */
	kt.c_cflag |= CRTSCTS; /* Enable RTS/CTS (hardware) flow control */
	kt.c_iflag = 0;
	kt.c_iflag |= IGNBRK;  /* Ignore BREAK condition on input */
	kt.c_iflag |= IGNPAR;  /* Ignore framing errors and parity errors. */
	kt.c_oflag = 0;
	kt.c_lflag = 0;

	/* Save off the previous c_line so we don't overwrite it */
	kt.c_line = tty->termios.c_line;

	tty_termios_encode_baud_rate(&kt, speed, speed);

	return tty_set_termios(tty, &kt);
}

int mmi_uart_set_baud(void *uart_data, uint32_t baud)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	int ret = -ENODEV;

	if (mud && mud->tty)
		ret = config_tty(mud, (speed_t)baud, mud->tty);

	return ret;
}

/**
 * Find the tty driver for a given tty name.
 *
 * @name: name string to match
 * @line: pointer to resulting tty line nr
 *
 * Originally copied from tty_find_polling_driver, with polling check removed.
 */
static struct tty_driver *find_tty_driver(char *name, int *line)
{
	struct tty_driver *p, *res = NULL;
	unsigned long tty_line = 0;
	int len;
	char *str, *stp;

	for (str = name; *str; str++)
		if ((*str >= '0' && *str <= '9') || *str == ',')
			break;
	if (!*str)
		return NULL;

	len = str - name;
	if (kstrtoul(str, 10, &tty_line))
		return NULL;

	mutex_lock(&tty_mutex);
	/* Search through the tty devices to look for a match */
	list_for_each_entry(p, &tty_drivers, tty_drivers) {
		if (strncmp(name, p->name, len) != 0)
			continue;
		stp = str;
		if (*stp == ',')
			stp++;
		if (*stp == '\0')
			stp = NULL;

		if (tty_line >= 0 && tty_line < p->num) {
			res = tty_driver_kref_get(p);
			*line = tty_line;
			break;
		}
	}
	mutex_unlock(&tty_mutex);

	return res;
}

int mmi_uart_do_pm(void *uart_data, bool on)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;

	if (!mud || !mud->tty) {
		pr_err("%s: no tty\n", __func__);
		return -ENODEV;
	}

	return mud->tty->ops->ioctl(mud->tty, on ? TIOCPMGET : TIOCPMPUT, 0);
}

int mmi_uart_open(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	struct tty_driver *driver;
	int tty_line = 0;
	int ret;
	struct tty_struct *tty_tmp;

	if (mud->tty) {
		dev_warn(&mud->pdev->dev, "%s: already open\n", __func__);
		return -EEXIST;
	}

	dev_dbg(&mud->pdev->dev, "%s: opening uart\n", __func__);

	/* Find the driver for the specified tty */
	driver = find_tty_driver((char *)mud->tty_name, &tty_line);
	if (!driver || !driver->ttys) {
		dev_err(&mud->pdev->dev, "%s: Did not find tty driver\n",
			__func__);
		ret = -ENODEV;
		goto open_fail;
	}

	/* Use existing tty if present */
	tty_tmp = driver->ttys[tty_line];
	if (!tty_tmp)
		tty_tmp = tty_init_dev(driver, tty_line);

	tty_driver_kref_put(driver);

	if (IS_ERR(tty_tmp)) {
		ret = PTR_ERR(tty_tmp);
		goto open_fail;
	}

	ret = tty_tmp->ops->open(tty_tmp, NULL);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Failed to open tty\n", __func__);
		goto release_tty;
	}

	ret = config_tty(mud, 0 /* default speed */, tty_tmp);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Failed to config tty\n",
			__func__);
		goto close_tty_locked;
	}

	/*
	 * Before the line discipline can be set, the tty must be unlocked.
	 * If this is not done, the kernel will deadlock.
	 */
	tty_unlock(tty_tmp);

	/* Reset sysfs stat entries */
	memset((void *)&mud->stats, 0, sizeof(struct mmi_uart_err_stats));

	/* Set device to ready before we set up the ldisc, we don't want
	 * to not be ready if we start processing rx messages.
	 */
	mud->tty = tty_tmp;
	mmi_uart_set_ready(uart_data, true);

	/*
	 * Set the line discipline to the local ldisc. This will allow this
	 * driver to know when new data is received without having to poll.
	 */
	ret = tty_set_ldisc(tty_tmp, N_MMI_UART);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Failed to set ldisc\n", __func__);
		mmi_uart_set_ready(uart_data, false);
		goto close_tty_unlocked;
	}
	tty_tmp->disc_data = mud;

	dev_info(&mud->pdev->dev, "%s: uart init done\n", __func__);

	return 0;

close_tty_unlocked:
	/* TTY must be locked to close the connection */
	tty_lock(tty_tmp);

close_tty_locked:
	tty_tmp->ops->close(tty_tmp, NULL);
	tty_unlock(tty_tmp);

release_tty:
	/* Release the TTY, which requires the mutex to be held */
	mutex_lock(&tty_mutex);
	tty_release_struct(tty_tmp, tty_tmp->index);
	mutex_unlock(&tty_mutex);

open_fail:

	return ret;
}

int mmi_uart_close(void *uart_data)
{
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;
	int ret = 0;

	dev_dbg(&mud->pdev->dev, "%s: closing uart\n", __func__);

	if (mmi_uart_set_tx_busy(uart_data))
		return -EBUSY;

	if (!mmi_uart_get_ready(uart_data) || !mud->tty) {
		dev_warn(&mud->pdev->dev, "%s: already closed\n", __func__);
		return -ENODEV;
	}

	dev_dbg(&mud->pdev->dev, "%s: really closing\n", __func__);

	if (tty_set_ldisc(mud->tty, N_TTY))
		dev_err(&mud->pdev->dev, "%s: Failed to set ldisc\n", __func__);

	/* TTY must be locked to close the connection */
	tty_lock(mud->tty);
	mud->tty->ops->close(mud->tty, NULL);
	tty_unlock(mud->tty);

	/* Release the TTY, which requires the mutex to be held */
	mutex_lock(&tty_mutex);
	tty_release_struct(mud->tty, mud->tty->index);
	mutex_unlock(&tty_mutex);
	mud->tty = NULL;

	mmi_uart_set_ready(uart_data, false);
	mmi_uart_clear_tx_busy(uart_data);

	return ret;
}

static int mmi_uart_consume_segment(struct mmi_uart_data *mud)
{
	size_t segment_size;

	segment_size = (*mmi_uart_rx_cb)(mud->pdev, mud->rx_data, mud->rx_len);
	if (segment_size) {
		mud->rx_len -= segment_size;
		if (mud->rx_len) {
			memmove(mud->rx_data, mud->rx_data + segment_size,
				mud->rx_len);
			/* more data to consume */
			return 1;
		}
	}

	return 0;
}

static int n_mmi_uart_receive_buf2(struct tty_struct *tty,
				    const unsigned char *cp,
				    char *fp, int count)
{
	struct mmi_uart_data *mud = tty->disc_data;
	struct device *dev = &mud->pdev->dev;
	int to_be_consumed = count;

	print_hex_dump_debug("muc_uart rx raw: ", DUMP_PREFIX_OFFSET,
		16, 1, cp, count, true);

	/*
	 * Try to clean up garbage/incomplete chars received
	 * for some reason.
	 */
	if (mud->rx_len &&
	    (jiffies_to_msecs(jiffies - mud->last_rx) >
	     MMI_UART_SEGMENT_TIMEOUT)) {
		mud->stats.rx_timeout++;
		print_hex_dump_debug("muc_uart rx raw (timeout): ",
			DUMP_PREFIX_OFFSET,
			16, 1, mud->rx_data, mud->rx_len, true);
		dev_err(dev, "%s: RX Buffer cleaned up\n", __func__);
		mud->rx_len = 0;
	}
	mud->last_rx = jiffies;

	while (to_be_consumed > 0) {
		int copy_size;

		if (mud->rx_len == UART_MAX_MSG_SIZE) {
			/*
			 * buffer is already left full, not consumed.
			 * Something is wrong. Need to reset the buffer
			 * to proceed
			 */
			mud->stats.rx_abort++;
			print_hex_dump_debug("muc_uart rx raw (overflow): ",
				DUMP_PREFIX_OFFSET, 16, 1,
				mud->rx_data, mud->rx_len, true);
			dev_err(dev, "%s: RX buffer overflow\n", __func__);
			mud->rx_len = 0;
		}

		copy_size = MIN(to_be_consumed,
				UART_MAX_MSG_SIZE - mud->rx_len);

		memcpy(&mud->rx_data[mud->rx_len], cp, copy_size);
		mud->rx_len += copy_size;

		do {} while (mmi_uart_consume_segment(mud));

		to_be_consumed -= copy_size;
	}

	return count;
}

static struct tty_ldisc_ops mmi_uart_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= DRIVERNAME,
	.receive_buf2	= n_mmi_uart_receive_buf2,
};

int mmi_uart_init(size_t (*uart_rx_cb)(struct platform_device *pdev,
					uint8_t *payload, size_t len),
				    int (*uart_register_cb)(struct platform_device *pdev,
					void *uart_data),
					void *(*uart_get_data)(struct device *dev),
					struct platform_device *pdev)
{
	struct mmi_uart_data *mud;
	speed_t speed;
	int ret;
	struct device_node *np = pdev->dev.of_node;

	mmi_uart_rx_cb = uart_rx_cb;
	mmi_uart_register_cb = uart_register_cb;
	mmi_uart_get_data = uart_get_data;

	ret = tty_register_ldisc(N_MMI_UART, &mmi_uart_ldisc);
	if (ret) {
		pr_err("%s: ldisc registration failed: %d\n", __func__, ret);
		return ret;
	}

	mud = devm_kzalloc(&pdev->dev, sizeof(*mud), GFP_KERNEL);
	if (!mud)
		return -ENOMEM;

	ret = of_property_read_u32(np, "mmi,tty_speed", &speed);
	if (ret) {
		dev_err(&pdev->dev, "%s: TTY speed not populated\n", __func__);
		return ret;
	}
	mud->default_baud = speed;
	mud->pdev = pdev;

	/* Retrieve the name of the tty from the device tree */
	ret = of_property_read_string(np, "mmi,tty", &mud->tty_name);
	if (ret) {
		dev_err(&pdev->dev, "%s: TTY name not populated\n", __func__);
		return ret;
	}
	dev_info(&pdev->dev, "%s: Using %s\n", __func__, mud->tty_name);

	ret = sysfs_create_groups(&pdev->dev.kobj, uart_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		return ret;
	}

	if ((*mmi_uart_register_cb)(pdev, mud)) {
		ret = -EPROBE_DEFER;
		goto remove_sysfs;
	}

	return 0;

remove_sysfs:
	sysfs_remove_groups(&pdev->dev.kobj, uart_groups);

	return ret;
}

int mmi_uart_exit(void *uart_data)
{
	int ret;
	struct mmi_uart_data *mud = (struct mmi_uart_data *)uart_data;

	sysfs_remove_groups(&mud->pdev->dev.kobj, uart_groups);
	mmi_uart_do_pm(mud, false);
	mmi_uart_close(mud);

	ret = tty_unregister_ldisc(N_MMI_UART);
	if (ret < 0)
		pr_err("%s: ldisc unregistration failed: %d\n", __func__, ret);

	return 0;
}
