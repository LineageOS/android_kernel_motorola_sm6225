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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/crc16.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/pm.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <soc/qcom/mmi_boot_info.h>
#include "mmi_uart.h"
#include "mmi_tty.h"
#include "muc_protocol.h"

#define DRIVERNAME	"muc_uart"

/* Go into low power mode on a sleep request by the muc.  Side effects are that
 * we might return -EAGAIN on a transmit, and will be slower to wake up
 * when the muc sends the next message
 */
#define MUC_UART_DISABLE_ON_SLEEP 1

/* Initiate a sleep request after a timeout if the muc hasn't requested
 * it already
 */
#define MUC_UART_SEND_SLEEP_ON_IDLE 1
#define MUC_UART_IDLE_TIME 5000 /* ms */

struct write_data {
	struct list_head list;
	uint16_t cmd;
	uint8_t *payload;
	size_t payload_length;
};

struct mod_muc_data_t {
	struct platform_device *pdev;
	void *uart_data;
	int wake_out_gpio;
	int wake_in_gpio;
	int wake_irq;
	int mod_attached_gpio;
	int mod_attached_irq;
	int uart_pm_state;
	atomic_t suspend_ok;
	atomic_t waiting_for_ack;
	struct timer_list idle_timer;
	struct timer_list ack_timer;
	atomic_t mod_attached;
	struct workqueue_struct *write_wq;
	struct write_data *write_data_q;
	int write_q_size;
	atomic_t write_credits;
	struct delayed_work wake_work;
	struct delayed_work sleep_work;
	struct delayed_work idle_work;
	struct delayed_work attach_work;
	struct delayed_work write_work;
};

/* How long until we move on without an ack */
#define MUC_UART_ACK_TIME 500
#define WRITE_CREDIT_INIT 100
static DEFINE_SPINLOCK(write_q_lock);

/* Share tx between acks and send queue */
DEFINE_MUTEX(tx_lock);

static char *param_bootmode = "NA";
module_param(param_bootmode, charp, S_IRUSR);
MODULE_PARM_DESC(param_bootmode, "ro.bootmode");

static ktime_t time_send_start;
static ktime_t time_send_end;
static ktime_t time_uart_start;
static ktime_t time_uart_end;
static ktime_t time_ack_rxed;
static ktime_t time_rxed;
static ktime_t time_next_send;

static void print_times(void)
{
	/* TTY to ACK */
    printk("mmi_uart time: TTY to ACK: %lld\n",
		(long long)ktime_to_ns(ktime_sub(time_ack_rxed,
		time_send_start)));
	/* UART TX */
	printk("mmi_uart time: UART Tx: %lld\n",
		(long long)ktime_to_ns(ktime_sub(time_uart_end,
		time_uart_start)));
	/* TIME to Rx */
	printk("mmi_uart time: Time to Rx: %lld\n",
		(long long)ktime_to_ns(ktime_sub(time_rxed,
		time_uart_end)));
	/* TIME FOR ACK */
	printk("mmi_uart time: Time for ACK: %lld\n",
		(long long)ktime_to_ns(ktime_sub(time_ack_rxed,
		time_uart_end)));
}

#if MUC_UART_SEND_SLEEP_ON_IDLE
static inline void reset_idle_timer(struct mod_muc_data_t *mm_data)
{
	cancel_delayed_work(&mm_data->idle_work);
	if (mm_data)
		mod_timer(&mm_data->idle_timer,
			jiffies + msecs_to_jiffies(MUC_UART_IDLE_TIME));
}
#else
static inline void reset_idle_timer(struct mod_muc_data_t *mm_data) {}
#endif

static inline void set_wait_for_ack(struct mod_muc_data_t *mm_data,
	int cmd)
{
	if (mm_data)
		if (!(cmd & MSG_ACK_MASK || cmd & MSG_NACK_MASK)) {
			atomic_set(&mm_data->waiting_for_ack, 1);
			mod_timer(&mm_data->ack_timer, jiffies +
				msecs_to_jiffies(MUC_UART_ACK_TIME));
		}
}

static inline void clear_wait_for_ack(struct mod_muc_data_t *mm_data)
{
	if (mm_data) {
		pr_info("muc_uart processed an ack\n");
		time_ack_rxed = ktime_get();
		print_times();
		del_timer(&mm_data->ack_timer);
		atomic_set(&mm_data->waiting_for_ack, 0);
		atomic_add_unless(&mm_data->write_credits, 1, 100);
		pr_info("muc_uart clear_wait_for_ack, num credits: %d\n",
			atomic_read(&mm_data->write_credits));
		cancel_delayed_work_sync(&mm_data->write_work);
		queue_delayed_work(mm_data->write_wq,
				&mm_data->write_work,
				msecs_to_jiffies(0));
	}
}

/* 1 if the mod is attached to the phone, 0 otherwise */
static ssize_t mod_attached_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int gpio_state = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);

	if (mm_data)
		gpio_state = atomic_read(&mm_data->mod_attached);
	return snprintf(buf, 4, "%u\n", gpio_state);
}

/* /sys/devices/soc:mod_uart/mod_attached */
DEVICE_ATTR_RO(mod_attached);

static struct attribute *muc_uart_attrs[] = {
	&dev_attr_mod_attached.attr,
	NULL
};

ATTRIBUTE_GROUPS(muc_uart);

static void muc_uart_attach_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, attach_work.work);
	struct kobj_uevent_env *env;
	int gpio_state;

	if (mm_data) {
		gpio_state = gpio_get_value(mm_data->mod_attached_gpio);
		atomic_set(&mm_data->mod_attached, gpio_state);

		env = kzalloc(sizeof(*env), GFP_KERNEL);
		if (!env)
			return;

		if (gpio_state)
			add_uevent_var(env, "MOD_EVENT=ATTACHED");
		else
			add_uevent_var(env, "MOD_EVENT=DETACHED");

		kobject_uevent_env(&mm_data->pdev->dev.kobj,
			KOBJ_CHANGE,
			env->envp);
		kfree(env);
	}
}

static irqreturn_t muc_uart_attach_irq_handler(int irq, void *data)
{
	struct mod_muc_data_t *mm_data = (struct mod_muc_data_t *)data;

	cancel_delayed_work(&mm_data->attach_work);
	schedule_delayed_work(&mm_data->attach_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static int muc_uart_send(struct mod_muc_data_t *mm_data,
	uint16_t cmd,
	uint8_t *payload,
	size_t payload_length)
{
	size_t pkt_size;
	uint16_t calc_crc;
	uint8_t *pkt;
	struct mmi_uart_hdr_t *hdr;
	int ret;

	if (!mm_data)
		return -ENODEV;

	if (payload_length > UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE) {
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -E2BIG;
	}

	/* If we are waiting for an ack */
	if (atomic_read(&mm_data->waiting_for_ack)) {
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -EALREADY;
	}

	/* If someone else is sending */
	if (mmi_uart_set_tx_busy(mm_data->uart_data)) {
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -EBUSY;
	}

	/* If powered off, try to wake. */
	if (!mm_data->uart_pm_state) {
		if (in_interrupt()) {
			mmi_uart_clear_tx_busy(mm_data->uart_data);
			schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(0));
			mmi_uart_report_tx_err(mm_data->uart_data);
			return -EAGAIN;
		} else {
			mmi_uart_do_pm(mm_data->uart_data, true);
			mm_data->uart_pm_state = 1;
		}
	}

	/* Rising edge to wake muc */
	gpio_set_value(mm_data->wake_out_gpio, 0);
	gpio_set_value(mm_data->wake_out_gpio, 1);

	reset_idle_timer(mm_data);

	pkt_size = sizeof(struct mmi_uart_hdr_t) +
		payload_length + sizeof(calc_crc);
	pkt = kmalloc(pkt_size, GFP_ATOMIC);
	if (!pkt) {
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -ENOMEM;
	}

	/* Populate the packet */
	hdr = (struct mmi_uart_hdr_t *)pkt;
	hdr->payload_length = cpu_to_le16(payload_length);
	hdr->magic =  cpu_to_le16(MSG_MAGIC);
	hdr->cmd =  cpu_to_le16(cmd);
	memcpy(pkt + sizeof(*hdr), payload, payload_length);
	calc_crc = crc16(0, pkt, pkt_size - sizeof(calc_crc));
	*(uint16_t *)(pkt + pkt_size - sizeof(calc_crc)) =
		cpu_to_le16(calc_crc);

	pr_info("muc_uart sending message\n");
	time_uart_start = ktime_get();
	ret = mmi_uart_send(mm_data->uart_data, pkt, pkt_size);
	kfree(pkt);
	time_uart_end = ktime_get();
	pr_info("muc_uart message sent\n");

	mmi_uart_clear_tx_busy(mm_data->uart_data);

	if (ret == 0) {
		set_wait_for_ack(mm_data, cmd);
		return payload_length;
	}

	time_send_end = ktime_get();

	return ret;
}

/* Any message that needs an ack goes on the queue */
static int muc_uart_queue_send(struct mod_muc_data_t *mm_data,
	uint16_t cmd,
	uint8_t *payload,
	size_t payload_len)
{
	struct write_data *write_data;
	unsigned long flags;

	if (payload_len > UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE)
		return -E2BIG;

	if (atomic_dec_if_positive(&mm_data->write_credits) < 0)
		pr_err("muc_uart_queue_send: out of credits! "
			"queue size is: %d\n", mm_data->write_q_size);

	write_data = kmalloc(sizeof(*write_data), GFP_ATOMIC);
	if (WARN_ON(!write_data)) {
		atomic_inc(&mm_data->write_credits);
		return -ENOMEM;
	}

	write_data->payload = kmalloc(payload_len, GFP_ATOMIC);
	if (WARN_ON(!write_data->payload)) {
		atomic_inc(&mm_data->write_credits);
		kfree(write_data);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&write_data->list);
	write_data->payload_length = payload_len;
	write_data->cmd = cmd;
	memcpy(write_data->payload, payload, payload_len);

	spin_lock_irqsave(&write_q_lock, flags);
	if (!mm_data->write_data_q)
		mm_data->write_data_q = write_data;
	else
		list_add_tail(&write_data->list,
			&mm_data->write_data_q->list);
	mm_data->write_q_size++;
	spin_unlock_irqrestore(&write_q_lock, flags);

	pr_info("muc_uart added message to queue\n");

	if (!atomic_read(&mm_data->waiting_for_ack)) {
		cancel_delayed_work(&mm_data->write_work);
		queue_delayed_work(mm_data->write_wq,
					&mm_data->write_work,
					msecs_to_jiffies(0));
	}

	return payload_len;
}

static void muc_uart_send_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, write_work.work);
	struct write_data *write_data = NULL;
	unsigned long flags;
	int ret;

	time_next_send = ktime_get();

	pr_info("muc_uart_send_work, num credits: %d\n",
		atomic_read(&mm_data->write_credits));

	if (atomic_read(&mm_data->waiting_for_ack))
		return;

	spin_lock_irqsave(&write_q_lock, flags);
	if (mm_data->write_data_q) {
		write_data = mm_data->write_data_q;
		if (list_is_last(&write_data->list,
			&mm_data->write_data_q->list))
			mm_data->write_data_q = NULL;
		else {
			mm_data->write_data_q =
				list_next_entry(write_data, list);
			list_del(&write_data->list);
		}
		mm_data->write_q_size--;
	}
	spin_unlock_irqrestore(&write_q_lock, flags);

	if (!write_data) {
		pr_info("muc_uart muc_uart_send_work, q empty!\n");
		return;
	}

	mutex_lock(&tx_lock);
	ret = muc_uart_send(mm_data,
		write_data->cmd,
		write_data->payload,
		write_data->payload_length);
	mutex_unlock(&tx_lock);

	if (ret < 0) {
		pr_err("muc_uart_send_work failed to send: %d\n", ret);
		/* Put back on top of queue to try later*/
		INIT_LIST_HEAD(&write_data->list);
		spin_lock_irqsave(&write_q_lock, flags);
		if (mm_data->write_data_q)
			list_add_tail(&write_data->list,
				&mm_data->write_data_q->list);
		mm_data->write_data_q = write_data;
		mm_data->write_q_size++;
		spin_unlock_irqrestore(&write_q_lock, flags);

		cancel_delayed_work(&mm_data->write_work);
		queue_delayed_work(mm_data->write_wq,
				&mm_data->write_work,
				msecs_to_jiffies(100));
	} else {
		kfree(write_data->payload);
		kfree(write_data);
	}
}

static int muc_uart_send_bootmode(struct mod_muc_data_t *mm_data)
{
	struct boot_mode_t bootmode;

	memset(&bootmode, 0x00, sizeof(struct boot_mode_t));

	bootmode.hwid = bi_hwrev();

	if(strncmp(param_bootmode, "mot-factory", strlen("mot-factory")) == 0) {
		bootmode.boot_mode = FACTORY;
	} else if(strncmp(param_bootmode, "qcom", strlen("qcom")) == 0) {
		bootmode.boot_mode = QCOM;
	} else if(strncmp(param_bootmode, "bp-tools", strlen("bp-tools")) == 0) {
		bootmode.boot_mode = BP_TOOLS;
	} else {
		/* TODO defaults to normal.  Should we have unknown/other? */
		bootmode.boot_mode = NORMAL;
	}

	pr_info("muc_uart_send_bootmode sending AP bootmode %u\n",
		bootmode.boot_mode);
	pr_info("muc_uart_send_bootmode sending AP hwid 0x%x\n",
		bootmode.hwid);

	return muc_uart_queue_send(mm_data,
		BOOT_MODE,
		(uint8_t *)&bootmode,
		sizeof(struct boot_mode_t));
}

static void muc_uart_set_power_control(struct power_control_t *pwrctl)
{
	/* TODO handle power control */
}

static int muc_uart_tty_rx_cb(struct platform_device *pdev,
	int idx,
	uint8_t *payload,
	size_t payload_length)
{
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	pr_info("muc_uart_send_packetbus: send from %d msg size %zd.\n",
		idx,
		payload_length);

	/* TODO idx 1 is for debug */
	if (idx == MMI_TTY_DEBUG_IDX) {
		if (!mmi_tty_push_to_us(MMI_TTY_DEFAULT_IDX,
			payload, payload_length))
			return payload_length;
		else
			return -EAGAIN;
	} else {
		mmi_tty_push_to_us(MMI_TTY_DEBUG_IDX,
			payload, payload_length);
		time_send_start = ktime_get();
		return muc_uart_queue_send(mm_data,
			PACKETBUS_PROT_MSG,
			payload,
			payload_length);
	}
}

static void muc_uart_handle_message(struct mod_muc_data_t *mm_data,
	struct mmi_uart_hdr_t *hdr,
	uint8_t *payload,
	size_t payload_len)
{
	pr_info("muc_uart_handle_message: got msg of type %x.\n", hdr->cmd);
	switch (hdr->cmd) {
		case UART_SLEEP_REQ: {
			/* TODO always ack for now. */
			muc_uart_send(mm_data, UART_SLEEP_ACK, NULL, 0);
			schedule_delayed_work(&mm_data->sleep_work, msecs_to_jiffies(0));
			break;
		}
		case UART_SLEEP_ACK: {
			clear_wait_for_ack(mm_data);
			schedule_delayed_work(&mm_data->sleep_work, msecs_to_jiffies(0));
			break;
		}
		case UART_SLEEP_NACK: {
			clear_wait_for_ack(mm_data);
			reset_idle_timer(mm_data);
			break;
		}
		case PACKETBUS_PROT_MSG: {
			if (!mmi_tty_push_to_us(MMI_TTY_DEFAULT_IDX,
				payload, payload_len)) {
				muc_uart_send(mm_data,
					PACKETBUS_PROT_MSG|MSG_ACK_MASK,
					NULL, 0);
			} else {
				pr_err("muc_uart_handle_message: Couldn't pass packet to us!");
				muc_uart_send(mm_data,
					PACKETBUS_PROT_MSG|MSG_NACK_MASK,
					NULL, 0);
			}
			break;
		}
		case PACKETBUS_PROT_MSG | MSG_ACK_MASK: {
			clear_wait_for_ack(mm_data);
			break;
		}
		case PACKETBUS_PROT_MSG | MSG_NACK_MASK: {
			clear_wait_for_ack(mm_data);
			break;
		}
		/* TODO power status will be sent, not received... probably? */
		/*case POWER_STATUS: {
			muc_uart_send_power_status();
			break;
		}*/
		case POWER_CONTROL: {
			/* TODO nack if size is wrong ? */
			if(payload_len == sizeof(struct power_control_t))
				muc_uart_set_power_control(
					(struct power_control_t *)payload);
			muc_uart_send(mm_data, hdr->cmd|MSG_ACK_MASK, NULL, 0);
			break;
		}
		case BOOT_MODE: {
			if(muc_uart_send_bootmode(mm_data) >= 0)
				muc_uart_send(mm_data, BOOT_MODE|MSG_ACK_MASK, NULL, 0);
			else
				muc_uart_send(mm_data, BOOT_MODE|MSG_NACK_MASK, NULL, 0);
			break;
		}
		default: {
			if (hdr->cmd & MSG_ACK_MASK)
				clear_wait_for_ack(mm_data);
			else if (hdr->cmd & MSG_NACK_MASK)
				clear_wait_for_ack(mm_data);
			else {
				pr_err("muc_uart_handle_message: Unhandled type %d!",
					hdr->cmd);
				muc_uart_send(mm_data, hdr->cmd|MSG_NACK_MASK, NULL, 0);
			}
		}
	}
}

static size_t muc_uart_rx_cb(struct platform_device *pdev,
	uint8_t *data,
	size_t len)
{
	size_t content_size;
	size_t segment_size;
	struct mmi_uart_hdr_t *hdr;
	uint16_t calc_crc;
	uint16_t rcvd_crc;
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	pr_info("muc_uart received message\n");
	time_rxed = ktime_get();

	reset_idle_timer(mm_data);

	/* Need at least a header to start */
	if (len < sizeof(*hdr))
		return 0;

	hdr = (struct mmi_uart_hdr_t *)data;
	segment_size = le16_to_cpu(hdr->payload_length)
		+ sizeof(*hdr) + sizeof(calc_crc);

	/* Validate the payload size */
	if (le16_to_cpu(hdr->payload_length) >
		UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE) {
		pr_err("%s: invalid len %zd\n", __func__, segment_size);
		mmi_uart_report_rx_len_err(mm_data->uart_data);
		return len;
	}

	/* Verify the magic number */
	if (le16_to_cpu(hdr->magic) != MSG_MAGIC) {
		pr_err("%s: invalid magic %x\n",
			__func__, le16_to_cpu(hdr->magic));
		mmi_uart_report_rx_mag_err(mm_data->uart_data);
		return len;
	}

	/* Need the whole message to continue */
	if (len < segment_size)
		return 0;

	content_size = segment_size - sizeof(calc_crc);
	rcvd_crc = *(uint16_t *)(data + content_size);
	calc_crc = crc16(0, data, content_size);

	/* Verify the CRC */
	if (le16_to_cpu(rcvd_crc) != calc_crc) {
		mmi_uart_report_rx_crc_err(mm_data->uart_data);
		print_hex_dump_debug("muc_uart rx (CRC error): ",
			DUMP_PREFIX_OFFSET,
			16, 1, data, content_size, true);
		pr_err("%s: CRC mismatch, received: 0x%x, "
			"calculated: 0x%x\n", __func__,
			le16_to_cpu(rcvd_crc), calc_crc);
	} else {
		pr_info("muc_uart rx: cmd=%x, magic=%x, "
			"payload_length=%x, len=%zd\n",
			hdr->cmd, hdr->magic, hdr->payload_length,
			content_size);

		print_hex_dump_debug("muc_uart rx: ", DUMP_PREFIX_OFFSET, 16, 1,
			data, content_size, true);

		/* TODO it still might be possible for a message to go
		 * out of the queue before we handle the response to a
		 * request.
		 */
		mutex_lock(&tx_lock);
		muc_uart_handle_message(mm_data,
			hdr,
			data + sizeof(*hdr),
			content_size - sizeof(*hdr));
		mutex_unlock(&tx_lock);
	}

	return segment_size;
}

/* TODO call from power management suspend. */
static void muc_uart_sleep_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, sleep_work.work);

	atomic_set(&mm_data->suspend_ok, 1);
	del_timer(&mm_data->idle_timer);
	cancel_delayed_work_sync(&mm_data->idle_work);

	if (!mm_data || !mm_data->uart_data ||
		mmi_uart_set_tx_busy(mm_data->uart_data))
		schedule_delayed_work(&mm_data->sleep_work, msecs_to_jiffies(500));
	else {
		pr_info("muc_uart_sleep_work: off.\n");
#if MUC_UART_DISABLE_ON_SLEEP
		if (mm_data->uart_pm_state) {
			mmi_uart_do_pm(mm_data->uart_data, false);
			mm_data->uart_pm_state = 0;
		}
#endif

		if (gpio_get_value(mm_data->wake_out_gpio))
			gpio_set_value(mm_data->wake_out_gpio, 0);

		mmi_uart_clear_tx_busy(mm_data->uart_data);
	}
}

/* TODO call from power management wake. */
static void muc_uart_wake_work(struct work_struct *w)
{
	static bool booted = false;
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, wake_work.work);

	disable_irq(mm_data->wake_irq);
	cancel_delayed_work_sync(&mm_data->sleep_work);
	atomic_set(&mm_data->suspend_ok, 0);

	reset_idle_timer(mm_data);

	if (!mm_data || !mm_data->uart_data ||
		mmi_uart_set_tx_busy(mm_data->uart_data))
		schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(500));
	else {
		pr_info("muc_uart_wake_work: on.\n");
		if (!mm_data->uart_pm_state) {
			mmi_uart_do_pm(mm_data->uart_data, true);
			mm_data->uart_pm_state = 1;
		}

		/* Rising edge to wake muc */
		gpio_set_value(mm_data->wake_out_gpio, 0);
		gpio_set_value(mm_data->wake_out_gpio, 1);

		enable_irq(mm_data->wake_irq);
		mmi_uart_clear_tx_busy(mm_data->uart_data);

		/* Send boot mode on initial wake */
		if (!booted) {
			if (muc_uart_send_bootmode(mm_data) >= 0);
				booted = true;
		}
	}
}

static irqreturn_t muc_uart_wake_irq_handler(int irq, void *data)
{
	struct mod_muc_data_t *mm_data = (struct mod_muc_data_t *)data;

	cancel_delayed_work(&mm_data->wake_work);
	schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static void muc_uart_idle_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, idle_work.work);

	if (muc_uart_queue_send(mm_data, UART_SLEEP_REQ, NULL, 0) < 0)
		schedule_delayed_work(&mm_data->idle_work, msecs_to_jiffies(1000));

	reset_idle_timer(mm_data);
}

static void muc_uart_idle_cb(unsigned long timer_data)
{
	struct mod_muc_data_t *mm_data =
		(struct mod_muc_data_t *)timer_data;

	schedule_delayed_work(&mm_data->idle_work, msecs_to_jiffies(0));
}

static void muc_uart_ack_cb(unsigned long timer_data)
{
	struct mod_muc_data_t *mm_data =
		(struct mod_muc_data_t *)timer_data;

	/* TODO what should I do if I don't get an ack in time? */
	if (mm_data) {
		atomic_set(&mm_data->waiting_for_ack, 0);
		cancel_delayed_work(&mm_data->write_work);
		queue_delayed_work(mm_data->write_wq,
				&mm_data->write_work,
				msecs_to_jiffies(0));
	}
}

static int muc_uart_register_cb(struct platform_device *pdev,
	void *uart_data)
{
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);

	if (!mm_data)
		return 1;

	pr_info("muc_uart_register_cb: register cb\n");

	mm_data->uart_data = uart_data;
	mmi_uart_open(uart_data);
	schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(0));

	pr_info("muc_uart_register_cb: register ok\n");

	return 0;
}

static void *muc_uart_get_uart_data(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);
	return mm_data->uart_data;
}

static int muc_uart_config_gpio(struct device *dev,
		int gpio, char *name, int dir_out, int out_val)
{
	int error;

	error = devm_gpio_request(dev, gpio, name);
	if (error) {
		dev_err(dev, "unable to request gpio [%d]\n", gpio);
		goto f_end;
	}
	if (dir_out == 2)
		goto f_end;
	if (dir_out == 1)
		error = gpio_direction_output(gpio, out_val);
	else
		error = gpio_direction_input(gpio);
	if (error) {
		dev_err(dev, "unable to set direction for gpio [%d]\n", gpio);
		goto f_end;
	}
f_end:
	return error;
}

static int muc_uart_probe(struct platform_device *pdev)
{
	struct device_node *np = (&pdev->dev)->of_node;
	struct mod_muc_data_t *mm_data;

	mm_data = devm_kzalloc(&pdev->dev, sizeof(*mm_data), GFP_KERNEL);
	if (!mm_data)
		return -ENOMEM;
	mm_data->pdev = pdev;
	platform_set_drvdata(pdev, mm_data);

	pr_info("muc_uart_probe: probe start\n");

	INIT_DELAYED_WORK(&mm_data->wake_work, muc_uart_wake_work);
	INIT_DELAYED_WORK(&mm_data->sleep_work, muc_uart_sleep_work);
	INIT_DELAYED_WORK(&mm_data->idle_work, muc_uart_idle_work);
	INIT_DELAYED_WORK(&mm_data->attach_work, muc_uart_attach_work);
	INIT_DELAYED_WORK(&mm_data->write_work, muc_uart_send_work);

	/* Create workqueue for the write work */
	mm_data->write_wq = alloc_workqueue("muc_uart_write", WQ_UNBOUND, 1);
	if (!mm_data->write_wq) {
		dev_err(&pdev->dev, "Could not create workqueue\n");
		goto err;
	}

	atomic_set(&mm_data->write_credits, WRITE_CREDIT_INIT);

	setup_timer(&mm_data->idle_timer,
		muc_uart_idle_cb,
		(unsigned long)mm_data);
	setup_timer(&mm_data->ack_timer,
		muc_uart_ack_cb,
		(unsigned long)mm_data);

	mm_data->wake_out_gpio = of_get_gpio(np, 0);
	if (mm_data->wake_out_gpio >= 0) {
		if (muc_uart_config_gpio(&pdev->dev,
			mm_data->wake_out_gpio, "muc_wake", 1, 1))
			goto err1;
	} else
		goto err1;

	mm_data->wake_in_gpio = of_get_gpio(np, 1);
	if (mm_data->wake_in_gpio >= 0)
		mm_data->wake_irq =
			gpio_to_irq(mm_data->wake_in_gpio);
	else
		goto err2;

	pr_info("muc_uart_probe: set gpio %d to irq %d.\n",
		mm_data->wake_in_gpio,
		mm_data->wake_irq);

	if (devm_request_irq(&pdev->dev,
		mm_data->wake_irq,
		muc_uart_wake_irq_handler,
		IRQF_TRIGGER_RISING,
		pdev->dev.driver->name,
		mm_data))
		goto err2;

	mm_data->mod_attached_gpio = of_get_gpio(np, 2);
	if (mm_data->mod_attached_gpio >= 0)
		mm_data->mod_attached_irq =
			gpio_to_irq(mm_data->mod_attached_gpio);
	else
		goto err2;

	if (devm_request_irq(&pdev->dev,
		mm_data->mod_attached_irq,
		muc_uart_attach_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		pdev->dev.driver->name,
		mm_data))
		goto err2;

	if (sysfs_create_groups(&pdev->dev.kobj, muc_uart_groups)) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		goto err2;
	}

	if (mmi_uart_init(muc_uart_rx_cb,
		muc_uart_register_cb,
		muc_uart_get_uart_data,
		pdev))
		goto err2;

	if (mmi_tty_init(muc_uart_tty_rx_cb,
		(UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE),
		pdev))
		goto err3;

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	schedule_delayed_work(&mm_data->attach_work, msecs_to_jiffies(0));

	pr_info("muc_uart_probe: probe done\n");

	return 0;
err3:
	mmi_uart_exit(mm_data->uart_data);
err2:
	gpio_free(mm_data->wake_out_gpio);
err1:
	destroy_workqueue(mm_data->write_wq);
err:
	return 1;
}

static int muc_uart_remove(struct platform_device *pdev)
{
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);

	if (gpio_is_valid(mm_data->wake_in_gpio))
		gpio_free(mm_data->wake_in_gpio);

	if (gpio_is_valid(mm_data->wake_out_gpio))
		gpio_free(mm_data->wake_out_gpio);

	if (gpio_is_valid(mm_data->mod_attached_gpio))
		gpio_free(mm_data->mod_attached_gpio);

	del_timer(&mm_data->idle_timer);
	del_timer(&mm_data->ack_timer);
	cancel_delayed_work_sync(&mm_data->wake_work);
	cancel_delayed_work_sync(&mm_data->sleep_work);
	cancel_delayed_work_sync(&mm_data->idle_work);
	cancel_delayed_work_sync(&mm_data->attach_work);
	cancel_delayed_work_sync(&mm_data->write_work);
	destroy_workqueue(mm_data->write_wq);
	mmi_uart_exit(mm_data->uart_data);
	mmi_tty_exit();
	sysfs_remove_groups(&pdev->dev.kobj, muc_uart_groups);
	kobject_uevent(&pdev->dev.kobj, KOBJ_REMOVE);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

/* TODO define suspend functions */
static const struct dev_pm_ops muc_uart_dev_pm_ops = {
	.runtime_suspend = NULL,
	.runtime_resume = NULL,
	.runtime_idle = NULL,
	.suspend_noirq = NULL,
	.resume_noirq = NULL
};

static const struct of_device_id muc_uart_match_table[] = {
	{ .compatible = "mmi,muc-uart"},
	{}
};

static struct platform_driver muc_uart_driver = {
	.probe	= muc_uart_probe,
	.remove = muc_uart_remove,
	.driver = {
		.name = DRIVERNAME,
		.pm = &muc_uart_dev_pm_ops,
		.of_match_table = muc_uart_match_table,
	},
};

int muc_uart_init(void)
{
	pr_info("muc_uart_init, bootmode: %s\n", param_bootmode);
	return platform_driver_register(&muc_uart_driver);
}

void muc_uart_exit(void)
{
	platform_driver_unregister(&muc_uart_driver);
}

module_init(muc_uart_init);
module_exit(muc_uart_exit);

MODULE_DESCRIPTION("MMI MuC UART Driver");
MODULE_LICENSE("GPL v2");
