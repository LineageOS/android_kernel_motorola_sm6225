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
#include <linux/uaccess.h>
#include <linux/ipc_logging.h>
#include <linux/firmware.h>
#include <soc/qcom/mmi_boot_info.h>
#include "muc_uart.h"
#include "mmi_uart.h"
#include "mmi_char.h"
#include "muc_protocol.h"

#ifdef CONFIG_IPC_LOGGING
void *muc_ipc_log;
#define MUC_IPC_LOG_PAGES 50
#define MUC_DBG(msg, ...)					\
	do {								\
		if (muc_ipc_log) {	\
			ipc_log_string(muc_ipc_log,			\
				"[%s] " msg, __func__, ##__VA_ARGS__);	\
		}							\
	} while (0)
#define MUC_ERR(msg, ...) \
	do {								\
		MUC_DBG(msg, ##__VA_ARGS__); \
		pr_err("muc_uart: [%s] "msg, __func__, ##__VA_ARGS__); \
	} while (0)
#define MUC_LOG(msg, ...) \
	do {								\
		MUC_DBG(msg, ##__VA_ARGS__); \
		pr_info("muc_uart: [%s] "msg, __func__, ##__VA_ARGS__); \
	} while (0)
#else
#define MUC_DBG(msg, ...) \
	pr_info("muc_uart: [%s] "msg, __func__, ##__VA_ARGS__)
#define MUC_ERR(msg, ...) \
	pr_err("muc_uart: [%s] "msg, __func__, ##__VA_ARGS__)
#define MUC_LOG(msg, ...) \
	pr_info("muc_uart: [%s] "msg, __func__, ##__VA_ARGS__)
#endif

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
#define MUC_UART_IDLE_TIME 1000 /* ms */

/* When a sleep request on occurs either to or from the muc, sending
 * is blocked.  If the sleep request never finishes for some reason
 * we will be stuck forever.  Use this value for a timer to clear
 * the request eventually to try to maybe recover or panic.
 */
#define MUC_UART_SLEEP_REQ_TIMEOUT 500 /* ms */

/* How many times to try to send a message before giving up */
#define MUC_UART_RETRY_COUNT 100

/* Max number of chars we will copy from the fw vers sent by the muc */
#define MUC_FW_VERS_MAX_SIZE 32

/* Should match EXT_USB_CHECK_TIME in dwc3-msm.c */
#define EXT_USB_CHECK_TIME 3000

struct write_data {
	struct list_head list;
	uint16_t cmd;
	uint8_t *payload;
	size_t payload_length;
	uint8_t retries;
};

struct read_data {
	struct list_head list;
	size_t payload_length;
	uint8_t *payload;
};

struct mod_muc_data_t {
	struct platform_device *pdev;
	void *uart_data;
	int wake_out_gpio;
	int wake_in_gpio;
	int wake_irq;
	int muc_1_gpio;
	int muc_2_gpio;
	int mod_attached_gpio;
	int mod_attached_irq;
	int uart_pm_state;
	atomic_t suspend_ok;
	atomic_t waiting_for_ack;
	struct timer_list idle_timer;
	struct timer_list ack_timer;
	struct timer_list sleep_req_timer;
	atomic_t mod_attached;
	atomic_t mod_online;
	struct workqueue_struct *write_wq;
	struct write_data *write_data_q;
	int write_q_size;
	struct read_data *read_data_q;
	int read_q_size;
	wait_queue_head_t *read_wq;
	atomic_t write_credits;
	struct delayed_work wake_work;
	struct delayed_work sleep_work;
	struct delayed_work idle_work;
	struct delayed_work attach_work;
	struct delayed_work write_work;
	struct delayed_work connect_work;
	struct work_struct ps_notify_work;
	struct power_supply *phone_psy;
	struct power_supply *batt_psy;
	struct power_supply *bms_psy;
	struct power_supply *usb_psy;
	struct power_supply *mmi_psy;
	struct power_supply *dc_psy;
	atomic_t sleep_req_pending;
	struct notifier_block ps_nb;
	char muc_fw_vers[MUC_FW_VERS_MAX_SIZE];
};

/* How long until we move on without an ack */
#define MUC_UART_ACK_TIME 2000
#define WRITE_CREDIT_INIT 100
static DEFINE_SPINLOCK(write_q_lock);
static DEFINE_SPINLOCK(read_q_lock);

/* Share tx between acks and send queue */
DEFINE_MUTEX(tx_lock);

static char *param_guid = "";
module_param(param_guid, charp, S_IRUSR);
MODULE_PARM_DESC(param_guid, "ro.mot.build.guid");

static char *param_vers = "";
module_param(param_vers, charp, S_IRUSR);
MODULE_PARM_DESC(param_vers, "ro.build.fingerprint");

static char *param_mucfw = "";
module_param(param_mucfw, charp, S_IRUSR);
MODULE_PARM_DESC(param_mucfw, "ro.mot.build.version.mod.nuttx");

static int muc_uart_config_gpio(struct device *dev,
		int gpio, char *name, int dir_out, int out_val);

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
		if (!(cmd & MSG_ACK_MASK ||
			cmd & MSG_NACK_MASK ||
			cmd == UART_SLEEP_REQ ||
			cmd == UART_SLEEP_REJ)) {
			atomic_set(&mm_data->waiting_for_ack, 1);
			mod_timer(&mm_data->ack_timer, jiffies +
				msecs_to_jiffies(MUC_UART_ACK_TIME));
		}
}

static inline void clear_wait_for_ack(struct mod_muc_data_t *mm_data)
{
	if (mm_data) {
		del_timer(&mm_data->ack_timer);
		atomic_set(&mm_data->waiting_for_ack, 0);
		atomic_add_unless(&mm_data->write_credits, 1, 100);
		MUC_DBG("got an ack, num credits: %d\n",
			atomic_read(&mm_data->write_credits));
		cancel_delayed_work(&mm_data->write_work);
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

static ssize_t muc_fw_vers_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);

	if (mm_data && mm_data->muc_fw_vers[0] != '\0')
		return snprintf(buf, strlen(mm_data->muc_fw_vers) + 1,
			"%s", mm_data->muc_fw_vers);

	return 0;
}

/* /sys/devices/soc:mod_uart/mod_attached */
DEVICE_ATTR_RO(mod_attached);
DEVICE_ATTR_RO(muc_fw_vers);

static struct attribute *muc_uart_attrs[] = {
	&dev_attr_mod_attached.attr,
	&dev_attr_muc_fw_vers.attr,
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

		if (atomic_read(&mm_data->mod_attached) == gpio_state)
			return;

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

		/* also make sure to notify the drivers of the attach/detach */
		power_supply_changed(mm_data->phone_psy);
	}
}

static irqreturn_t muc_uart_attach_irq_handler(int irq, void *data)
{
	struct mod_muc_data_t *mm_data = (struct mod_muc_data_t *)data;

	cancel_delayed_work(&mm_data->attach_work);
	schedule_delayed_work(&mm_data->attach_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static void muc_uart_connect_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, connect_work.work);
	struct kobj_uevent_env *env;

	if (mm_data) {
		env = kzalloc(sizeof(*env), GFP_KERNEL);
		if (!env)
			return;

		if (atomic_read(&mm_data->mod_online))
			add_uevent_var(env, "MOD_EVENT=ONLINE");
		else
			add_uevent_var(env, "MOD_EVENT=OFFLINE");

		kobject_uevent_env(&mm_data->pdev->dev.kobj,
			KOBJ_CHANGE,
			env->envp);
		kfree(env);
	}
}

static inline void set_sleep_req_pending(struct mod_muc_data_t *mm_data)
{
	atomic_set(&mm_data->sleep_req_pending, 1);
	mod_timer(&mm_data->sleep_req_timer, jiffies +
		msecs_to_jiffies(MUC_UART_SLEEP_REQ_TIMEOUT));
}

static inline void clear_sleep_req_pending(struct mod_muc_data_t *mm_data)
{
	del_timer(&mm_data->sleep_req_timer);
	atomic_set(&mm_data->sleep_req_pending, 0);
}

static int muc_uart_wake_muc(struct mod_muc_data_t *mm_data)
{
	gpio_direction_output(mm_data->wake_out_gpio, 0);

	/* Only toggle the line if the muc isn't awake */
	if (gpio_get_value(mm_data->wake_in_gpio)) {
		gpio_set_value(mm_data->wake_out_gpio, 1);
		mdelay(1);
		gpio_set_value(mm_data->wake_out_gpio, 0);
		mdelay(1);
		return 1;
	}

	/* Return 0 if the muc was already awake */
	return 0;
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

	/* If we are waiting on a sleep req */
	if (atomic_read(&mm_data->sleep_req_pending)) {
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -EALREADY;
	}

	/* If sending a sleep req, block further sends until
	 * we get a rej or go to sleep
	 */
	if (cmd == UART_SLEEP_REQ)
		set_sleep_req_pending(mm_data);

	/* If someone else is sending */
	if (mmi_uart_set_tx_busy(mm_data->uart_data)) {
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -EBUSY;
	}

	/* If we are in low power mode that needs to change
	 * else just try to wake the muc
	 */
	if (!mm_data->uart_pm_state) {
		cancel_delayed_work(&mm_data->wake_work);
		schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(0));
		mmi_uart_clear_tx_busy(mm_data->uart_data);
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -EAGAIN;
	} else if (muc_uart_wake_muc(mm_data)) {
		mmi_uart_clear_tx_busy(mm_data->uart_data);
		mmi_uart_report_tx_err(mm_data->uart_data);
		return -EAGAIN;
	}

	reset_idle_timer(mm_data);

	pkt_size = sizeof(struct mmi_uart_hdr_t) +
		payload_length + sizeof(calc_crc);
	pkt = kmalloc(pkt_size, GFP_ATOMIC);
	if (!pkt) {
		mmi_uart_clear_tx_busy(mm_data->uart_data);
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

	ret = mmi_uart_send(mm_data->uart_data, pkt, pkt_size);
	kfree(pkt);

	mmi_uart_clear_tx_busy(mm_data->uart_data);

	if (ret == 0) {
		set_wait_for_ack(mm_data, cmd);
		return payload_length;
	} else {
		/* Just in case we failed to send and we were
		 * going to sleep
		 */
		clear_sleep_req_pending(mm_data);
	}

	return ret;
}

/* Any message that needs an ack goes on the queue */
static int __muc_uart_queue_send(struct mod_muc_data_t *mm_data,
	uint16_t cmd,
	uint8_t *payload,
	size_t payload_len,
	bool payload_persistant)
{
	struct write_data *write_data;
	unsigned long flags;

	if (payload_len > UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE)
		return -E2BIG;

	/* TODO this should probably panic */
	if (atomic_dec_if_positive(&mm_data->write_credits) < 0)
		MUC_ERR("out of credits! "
			"queue size is: %d\n", mm_data->write_q_size);

	write_data = kmalloc(sizeof(*write_data), GFP_ATOMIC);
	if (WARN_ON(!write_data)) {
		atomic_inc(&mm_data->write_credits);
		return -ENOMEM;
	}

	if(!payload_persistant) {
		write_data->payload = kmalloc(payload_len, GFP_ATOMIC);
		/* TODO this should probably panic */
		if (WARN_ON(!write_data->payload)) {
			atomic_inc(&mm_data->write_credits);
			kfree(write_data);
			return -ENOMEM;
		}
		memcpy(write_data->payload, payload, payload_len);
	} else
		write_data->payload = payload;

	INIT_LIST_HEAD(&write_data->list);
	write_data->payload_length = payload_len;
	write_data->cmd = cmd;
	write_data->retries = MUC_UART_RETRY_COUNT;

	spin_lock_irqsave(&write_q_lock, flags);
	if (!mm_data->write_data_q)
		mm_data->write_data_q = write_data;
	else
		list_add_tail(&write_data->list,
			&mm_data->write_data_q->list);
	mm_data->write_q_size++;
	spin_unlock_irqrestore(&write_q_lock, flags);

	if (!atomic_read(&mm_data->waiting_for_ack)) {
		cancel_delayed_work(&mm_data->write_work);
		queue_delayed_work(mm_data->write_wq,
					&mm_data->write_work,
					msecs_to_jiffies(0));
	}

	return payload_len;
}

static int muc_uart_queue_send(struct mod_muc_data_t *mm_data,
	uint16_t cmd,
	uint8_t *payload,
	size_t payload_len)
{
	return __muc_uart_queue_send(mm_data, cmd, payload,
		payload_len, false);
}

static void muc_uart_send_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, write_work.work);
	struct write_data *write_data = NULL;
	unsigned long flags;
	int ret;

	if (atomic_read(&mm_data->waiting_for_ack)) {
		MUC_DBG("waiting for an ack...");
		return;
	}

	MUC_DBG("num credits remaining: %d\n",
		atomic_read(&mm_data->write_credits));

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
		return;
	}

	/* Wake the muc up here if it isn't already awake
	 * Send will wake it too, but hopefully this will
	 * reduce the chances it returns an error.
	 *
	 * Also wake up the uart if it is in low power mode.
	 */
	mmi_uart_set_tx_busy(mm_data->uart_data);
	if (!mm_data->uart_pm_state) {
		mmi_uart_do_pm(mm_data->uart_data, true);
		mm_data->uart_pm_state = 1;
	}
	mmi_uart_clear_tx_busy(mm_data->uart_data);
	muc_uart_wake_muc(mm_data);

	mutex_lock(&tx_lock);
	ret = muc_uart_send(mm_data,
		write_data->cmd,
		write_data->payload,
		write_data->payload_length);
	mutex_unlock(&tx_lock);

	if (ret < 0) {
		MUC_DBG("failed to send: %d\n", ret);
		if (write_data->retries) {
			write_data->retries--;
			if (!write_data->retries) {
				MUC_ERR("failed to send message %d\n", ret);
				atomic_add_unless(&mm_data->write_credits, 1, 100);
				goto destroy_msg;
			}
		}

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
				msecs_to_jiffies(10));
	} else
		goto destroy_msg;

	return;
destroy_msg:
	kfree(write_data->payload);
	kfree(write_data);
}

static int muc_uart_send_bootmode(struct mod_muc_data_t *mm_data)
{
	struct boot_mode_t bootmode;
	uint32_t mucfw_major;
	uint32_t mucfw_minor;

	memset(&bootmode, 0x00, sizeof(struct boot_mode_t));

	bootmode.hwid = bi_hwrev();

	strncpy(bootmode.ap_guid,
		param_guid,
		sizeof(bootmode.ap_guid));
	strncpy(bootmode.ap_fw_ver_str,
		param_vers,
		sizeof(bootmode.ap_fw_ver_str));

	/* Format mucfw vers from "MAJOR.MINOR_DESCRIPTION"
	 * to uint32_t in format MAJOR<<16|MINOR
	 */
	if (sscanf(param_mucfw, "%d.%d%*s", &mucfw_major, &mucfw_minor) != 2) {
		mucfw_major = 0;
		mucfw_minor = 0;
	}

	bootmode.muc_fw_vers = (mucfw_major << 16) | mucfw_minor;

	if (strncmp(bi_bootmode(), "mot-factory", strlen("mot-factory")) == 0) {
		bootmode.boot_mode = FACTORY;
	} else if (strncmp(bi_bootmode(), "qcom", strlen("qcom")) == 0) {
		bootmode.boot_mode = QCOM;
	} else if (strncmp(bi_bootmode(), "bp-tools", strlen("bp-tools")) == 0) {
		bootmode.boot_mode = BP_TOOLS;
	} else if (strncmp(bi_bootmode(), "recovery", strlen("recovery")) == 0) {
		bootmode.boot_mode = RECOVERY;
	} else {
		/* TODO defaults to normal.  Should we have unknown/other? */
		bootmode.boot_mode = NORMAL;
	}

	MUC_DBG("sending AP bootmode %u\n",
		bootmode.boot_mode);
	MUC_DBG("sending AP hwid 0x%x\n",
		bootmode.hwid);
	MUC_DBG("(%zd) sending AP guid %s\n",
		sizeof(bootmode.ap_guid), bootmode.ap_guid);
	MUC_DBG("(%zd) sending AP vers %s\n",
		sizeof(bootmode.ap_fw_ver_str), bootmode.ap_fw_ver_str);
	MUC_DBG("sending MUC fw vers 0x%08x\n",
		bootmode.muc_fw_vers);

	return muc_uart_queue_send(mm_data,
		BOOT_MODE,
		(uint8_t *)&bootmode,
		sizeof(struct boot_mode_t));
}

#define VBUS_PRESENT_UV 2000000
static int muc_uart_send_power_status(struct mod_muc_data_t *mm_data)
{
	struct power_status_t pstatus;
	union power_supply_propval pval = {0};

	if(!mm_data->batt_psy ||
		!mm_data->bms_psy ||
		!mm_data->usb_psy ||
		!mm_data->dc_psy)
		return -1;

	memset(&pstatus, 0x00, sizeof(struct power_status_t));

	if (!power_supply_get_property(mm_data->usb_psy,
		POWER_SUPPLY_PROP_PRESENT, &pval))
		pstatus.charger = pval.intval ? 1 : 0;

	if (pstatus.charger &&
	    !power_supply_get_property(mm_data->usb_psy,
		POWER_SUPPLY_PROP_TYPEC_MODE, &pval) &&
		(pval.intval == POWER_SUPPLY_TYPEC_NONE))
		pstatus.charger = 0;

	if (!power_supply_get_property(mm_data->batt_psy,
		POWER_SUPPLY_PROP_TEMP, &pval))
		/* Div by 10 to convert from deciDegC to DegC */
		pstatus.battery_temp = (int16_t)(pval.intval / 10);

	if (!power_supply_get_property(mm_data->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY, &pval))
		pstatus.battery_capacity =
			pval.intval > 0 ? (uint16_t)pval.intval : 0;

	if (!power_supply_get_property(mm_data->bms_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL, &pval))
		/* Div by 1000 to convert from uah to mah */
		pstatus.battery_max_capacity =
			pval.intval > 0 ? (uint16_t)(pval.intval / 1000) : 0;

	if (!power_supply_get_property(mm_data->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval))
		pstatus.battery_voltage =
			pval.intval > 0 ? (uint32_t)pval.intval : 0;

	if (!power_supply_get_property(mm_data->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &pval))
		pstatus.battery_current = (int32_t)pval.intval;

	if (!power_supply_get_property(mm_data->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval))
		pstatus.battery_max_voltage =
			pval.intval > 0 ? (uint32_t)pval.intval : 0;

	if (!power_supply_get_property(mm_data->batt_psy,
		POWER_SUPPLY_PROP_STATUS, &pval))
		pstatus.charging =
			(pval.intval == POWER_SUPPLY_STATUS_CHARGING);

	if (!power_supply_get_property(mm_data->usb_psy,
		POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval))
		pstatus.mod_input_voltage =
			pval.intval > 0 ? (uint32_t)pval.intval : 0;
	if (!power_supply_get_property(mm_data->usb_psy,
		POWER_SUPPLY_PROP_HW_CURRENT_MAX, &pval))
		pstatus.mod_input_current =
			pval.intval > 0 ? (uint32_t)pval.intval : 0;

	if (!power_supply_get_property(mm_data->dc_psy,
		POWER_SUPPLY_PROP_PRESENT, &pval))
		pstatus.dc_in = pval.intval ? 1 : 0;

	if (!power_supply_get_property(mm_data->usb_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval))
		pstatus.vbus = (pval.intval >= VBUS_PRESENT_UV) ? 1 : 0;

	/* TODO */
	/* pstatus.reverse_boost;
	 * pstatus.mod_output_current;
	 * pstatus.mod_input_current;
	 */

	pr_debug("muc_uart reverse_boost %d\n",
		 pstatus.reverse_boost);
	pr_debug("muc_uart charging %d\n",
		 pstatus.charging);
	pr_debug("muc_uart charger %d\n",
		 pstatus.charger);
	pr_debug("muc_uart battery_temp %hd degC\n",
		 pstatus.battery_temp);
	pr_debug("muc_uart battery_voltage %u uV\n",
		 pstatus.battery_voltage);
	pr_debug("muc_uart battery_current %d uA\n",
		 pstatus.battery_current);
	pr_debug("muc_uart battery_max_voltage %u uV\n",
		 pstatus.battery_max_voltage);
	pr_debug("muc_uart battery_capacity %hu percent\n",
		 pstatus.battery_capacity);
	pr_debug("muc_uart battery_max_capacity %hu mAhr\n",
		 pstatus.battery_max_capacity);
	pr_debug("muc_uart mod_output_voltage %u uV\n",
		 pstatus.mod_output_voltage);
	pr_debug("muc_uart mod_input_voltage %u uV\n",
		 pstatus.mod_input_voltage);
	pr_debug("muc_uart mod_output_current %u uA\n",
		 pstatus.mod_output_current);
	pr_debug("muc_uart mod_input_current %u uA\n",
		 pstatus.mod_input_current);

	return muc_uart_queue_send(mm_data,
		POWER_STATUS,
		(uint8_t *)&pstatus,
		sizeof(struct power_status_t));
}

static const char *pctrl_names[] = {
	"VBUS_IN",
	"VBUS_IN_SPLIT", 
	"VBUS_OUT",
	"DC_IN",
};

static void muc_uart_set_power_control(struct mod_muc_data_t *mm_data,
				       struct power_control_t *pwrctl)
{
	union power_supply_propval pval = {0};

	if (!mm_data || !pwrctl || !mm_data->mmi_psy) {
		MUC_ERR("No pwrctrl\n");
		return;
	}

	MUC_DBG("flow %s, voltage %u, current %u\n",
		pctrl_names[pwrctl->flow],
		pwrctl->voltage_uv,
		pwrctl->current_ua);

	if (pwrctl->flow == VBUS_OUT) {
		pval.intval = 1;
		power_supply_set_property(mm_data->mmi_psy,
					  POWER_SUPPLY_PROP_USB_OTG,
					  &pval);
	} else {
		pval.intval = 0;
		power_supply_set_property(mm_data->mmi_psy,
					  POWER_SUPPLY_PROP_USB_OTG,
					  &pval);
	}

	if ((pwrctl->flow == VBUS_IN) || (pwrctl->flow == VBUS_IN_SPLIT)) {
		pval.intval = pwrctl->current_ua;
		power_supply_set_property(mm_data->mmi_psy,
					  POWER_SUPPLY_PROP_CURRENT_MAX,
					  &pval);
	}
}

static void muc_uart_ps_notifier_work(struct work_struct *w)
{
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, ps_notify_work);

	muc_uart_send_power_status(mm_data);
}

static int muc_uart_ps_notifier_cb(struct notifier_block *nb,
			       unsigned long event,
			       void *v)
{
	struct mod_muc_data_t *mm_data = container_of(nb,
		struct mod_muc_data_t, ps_nb);
	struct power_supply *psy = v;

	if (!psy ||
		event != PSY_EVENT_PROP_CHANGED ||
		(strcmp(psy->desc->name, "usb") != 0 &&
		strcmp(psy->desc->name, "bms") != 0 &&
		strcmp(psy->desc->name, "battery") != 0))
		return NOTIFY_OK;

	schedule_work(&mm_data->ps_notify_work);

	return NOTIFY_OK;
}

int muc_uart_pb_register_wq(struct platform_device *pdev,
	wait_queue_head_t *read_wq)
{
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	/* Only allow the first one who registered */
	if(mm_data->read_wq)
		return 1;

	mm_data->read_wq = read_wq;
	return 0;
}

int muc_uart_pb_get_q(struct platform_device *pdev,
	char __user *buf,
	size_t *count)
{
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	unsigned long flags;
	struct read_data *pb_msg = NULL;

	spin_lock_irqsave(&read_q_lock, flags);
	if (mm_data->read_data_q) {
		pb_msg = mm_data->read_data_q;
		if(list_is_last(&pb_msg->list,
			&mm_data->read_data_q->list))
			mm_data->read_data_q = NULL;
		else {
			mm_data->read_data_q =
				list_next_entry(pb_msg, list);
			list_del(&pb_msg->list);
		}
		mm_data->read_q_size--;
	}
	spin_unlock_irqrestore(&read_q_lock, flags);

	if(!pb_msg)
		return -ENOMSG;

	if(pb_msg->payload_length > *count)
		return -ENOSPC;

	if(copy_to_user(buf, pb_msg->payload, pb_msg->payload_length))
		return -EFAULT;

	*count = pb_msg->payload_length;

	kfree(pb_msg->payload);
	kfree(pb_msg);

	return 0;
}

int muc_uart_pb_get_q_size(struct platform_device *pdev)
{
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	return mm_data->read_q_size;
}

static int muc_uart_pb_put_q(struct mod_muc_data_t *mm_data,
	size_t payload_length,
	uint8_t *payload)
{
	unsigned long flags;
	struct read_data *pb_msg;

	if(!payload_length)
		return 1;

	/* TODO should this list have a size limit?
	 * otherwise a crash of the consumer could
	 * potentially crash the kernel ...
	 */

	/* Set up the message */
	pb_msg = kmalloc(sizeof(struct read_data), GFP_KERNEL);
	if (!pb_msg)
		return -ENOMEM;

	pb_msg->payload = kmalloc(payload_length, GFP_KERNEL);
	if (!pb_msg->payload) {
		kfree(pb_msg);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&pb_msg->list);
	pb_msg->payload_length = payload_length;
	memcpy(pb_msg->payload, payload, payload_length);

	spin_lock_irqsave(&read_q_lock, flags);
	if (!mm_data->read_data_q)
		mm_data->read_data_q = pb_msg;
	else
		list_add_tail(&pb_msg->list,
			&mm_data->read_data_q->list);
	mm_data->read_q_size++;
	spin_unlock_irqrestore(&read_q_lock, flags);

	/* Wake up any process waiting on a message */
	if(mm_data->read_wq)
		wake_up(mm_data->read_wq);

	return 0;
}

static int muc_uart_send_firmware(struct mod_muc_data_t *mm_data)
{
	struct mmi_uart_fw_t *fw_pkt;
	const struct firmware *fw = NULL;
	int fw_data_remaining;
	int fw_data_offset = 0;
	uint16_t fw_pkt_sn = 1;
	uint16_t fw_pkt_len;
	uint16_t fw_payload_len;

	MUC_LOG("Updating muc firmware\n");

	if (request_firmware(&fw, MUC_FIRMWARE_NAME, &mm_data->pdev->dev)) {
		MUC_ERR("Unable to locate firmware %s!\n", MUC_FIRMWARE_NAME);
		goto err;
	}

	MUC_LOG("Firmware size is %zd bytes\n", fw->size);
	MUC_LOG("Queuing %zd messages for upload\n",
		(fw->size % MUC_FW_PAYLOAD_SIZE) > 0 ?
		(fw->size / MUC_FW_PAYLOAD_SIZE) + 2 :
		(fw->size / MUC_FW_PAYLOAD_SIZE) + 1);

	fw_pkt = kmalloc(sizeof(struct mmi_uart_fw_t) +
		MUC_FW_PAYLOAD_SIZE, GFP_KERNEL);
	if (!fw_pkt) {
		MUC_ERR("Unable allocate firmware packet\n");
		goto err_mem;
	}

	fw_pkt->port_id = PACKETBUS_PORT_MUC_FW;
	fw_data_remaining = fw->size;

	while (fw_data_remaining > 0) {
		if (fw_data_remaining >= MUC_FW_PAYLOAD_SIZE)
			fw_payload_len = MUC_FW_PAYLOAD_SIZE;
		else
			fw_payload_len = fw_data_remaining;

		memcpy(fw_pkt->payload,
			fw->data + fw_data_offset,
			fw_payload_len);

		fw_pkt_len = sizeof(struct mmi_uart_fw_t) + fw_payload_len;
		fw_pkt->sn = fw_pkt_sn++;

		if (muc_uart_queue_send(mm_data,
			PACKETBUS_PROT_MSG,
			(uint8_t *)fw_pkt,
			fw_pkt_len) != fw_pkt_len)
			goto err_pkt;

		fw_data_offset += fw_payload_len;
		fw_data_remaining -= fw_payload_len;
	}

	/* Last packet empty payload to indicate EOF */
	fw_pkt_len = sizeof(struct mmi_uart_fw_t);
	fw_pkt->sn = fw_pkt_sn;

	if (muc_uart_queue_send(mm_data,
		PACKETBUS_PROT_MSG,
		(uint8_t *)fw_pkt,
		fw_pkt_len) != fw_pkt_len)
		goto err_pkt;

	kfree(fw_pkt);
	release_firmware(fw);

	return 0;
err_pkt:
	kfree(fw_pkt);
err_mem:
	release_firmware(fw);
err:
	return 1;
}

static int muc_uart_check_update(struct mod_muc_data_t *mm_data,
	size_t payload_len,
	uint8_t *payload)
{
	struct mmi_uart_pb_hdr_t *pb_hdr;

	/* Check if this message was for us instead of userspace */
	if (payload_len < sizeof(struct mmi_uart_pb_hdr_t))
		return 1;

	pb_hdr = (struct mmi_uart_pb_hdr_t *)payload;

	if (pb_hdr->port_id != PACKETBUS_PORT_MUC_FW)
		return 1;

	if (pb_hdr->cmd != BOLT_MSG_GET_MUC_FW)
		return 1;

	if (muc_uart_send_firmware(mm_data))
		MUC_ERR("Failed to send firmware to muc\n");

	return 0;
}

int muc_uart_pb_write(struct platform_device *pdev,
	const char __user *buf,
	size_t count)
{
	uint8_t *payload;
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	/* Payload is freed after send work is done */
	payload = kmalloc(count, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;

	if (copy_from_user(payload, buf, count))
		return -EFAULT;

	return __muc_uart_queue_send(mm_data, PACKETBUS_PROT_MSG,
		payload,count, true);
}

static int muc_uart_set_gpio(struct device *dev, int value)
{
	/* payload: 1 byte, output value */
	pr_info("muc_uart_set_gpio out value:%d\n", value);

	muc_uart_config_gpio(dev, MOD_ATTACH_GPIO, "mod_attach", 1, value);
	gpio_set_value(MOD_ATTACH_GPIO, value);

	gpio_set_value(MUC_GPIO_1, value);

	muc_uart_config_gpio(dev, FORCE_USB_BOOT_GPIO, "force_usb_boot", 1, value);
	gpio_set_value(FORCE_USB_BOOT_GPIO, value);

	return 0;
}

static int muc_uart_handle_usb_ctrl(struct mod_muc_data_t *mm_data,
	struct usb_control_t *usb_ctrl_pkt)
{
	if (!mm_data)
		return 1;

	if (usb_ctrl_pkt->cmd != USB_SECONDARY_DISABLE &&
		usb_ctrl_pkt->cmd != USB_SECONDARY_ENABLE)
		return 1;

	atomic_set(&mm_data->mod_online, usb_ctrl_pkt->cmd);
	power_supply_changed(mm_data->phone_psy);

	return 0;
}

static void muc_uart_handle_message(struct mod_muc_data_t *mm_data,
	struct mmi_uart_hdr_t *hdr,
	uint8_t *payload,
	size_t payload_len)
{
	unsigned long flags;

	MUC_DBG("rxed msg of type 0x%x.\n", hdr->cmd);
	switch (hdr->cmd) {
		case UART_SLEEP_REQ: {
			spin_lock_irqsave(&write_q_lock, flags);
			if (mm_data->write_data_q)
				muc_uart_send(mm_data,
					UART_SLEEP_REJ,
					NULL, 0);
			else {
				set_sleep_req_pending(mm_data);
				schedule_delayed_work(&mm_data->sleep_work, msecs_to_jiffies(0));
			}
			spin_unlock_irqrestore(&write_q_lock, flags);
			break;
		}
		case UART_SLEEP_REJ: {
			clear_sleep_req_pending(mm_data);
			reset_idle_timer(mm_data);
			break;
		}
		case PACKETBUS_PROT_MSG: {
			if (!muc_uart_check_update(mm_data,
				payload_len,
				payload) ||
				!muc_uart_pb_put_q(mm_data,
				payload_len,
				payload)) {
				muc_uart_send(mm_data,
					PACKETBUS_PROT_MSG|MSG_ACK_MASK,
					NULL, 0);
			} else {
				MUC_ERR("Couldn't add packet to pb q!\n");
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
		case POWER_STATUS: {
			if (muc_uart_send_power_status(mm_data) >= 0)
				muc_uart_send(mm_data, POWER_STATUS|MSG_ACK_MASK, NULL, 0);
			else
				muc_uart_send(mm_data, POWER_STATUS|MSG_NACK_MASK, NULL, 0);
			break;
		}
		case POWER_CONTROL: {
			/* TODO nack if size is wrong ? */
			if (payload_len == sizeof(struct power_control_t))
				muc_uart_set_power_control(mm_data,
					(struct power_control_t *)payload);
			muc_uart_send(mm_data, hdr->cmd|MSG_ACK_MASK, NULL, 0);
			break;
		}
		case BOOT_MODE: {
			if (muc_uart_send_bootmode(mm_data) >= 0)
				muc_uart_send(mm_data, BOOT_MODE|MSG_ACK_MASK, NULL, 0);
			else
				muc_uart_send(mm_data, BOOT_MODE|MSG_NACK_MASK, NULL, 0);
			break;
		}
		case MUC_FW_VERSION: {
			if (payload_len) {
				muc_uart_send(mm_data, MUC_FW_VERSION|MSG_ACK_MASK, NULL, 0);
				MUC_LOG("muc_fw_ver: %.*s\n",
					(int)payload_len,
					(char *)payload);
				memset(mm_data->muc_fw_vers, 0, MUC_FW_VERS_MAX_SIZE);
				/* Last value of muc_fw_vers should be null char */
				memcpy(mm_data->muc_fw_vers, payload,
					payload_len < (MUC_FW_VERS_MAX_SIZE - 1) ?
					payload_len : (MUC_FW_VERS_MAX_SIZE - 1));
			} else
				muc_uart_send(mm_data, MUC_FW_VERSION|MSG_NACK_MASK, NULL, 0);
			break;
		}
		case MUC_SET_GPIO: {
			if (payload_len != 1) {
				pr_info("muc_set_gpio invalid payload_len: %d\n", (int)payload_len);
				muc_uart_send(mm_data, MUC_SET_GPIO|MSG_NACK_MASK, NULL, 0);
				break;
			}

			if (muc_uart_set_gpio(&mm_data->pdev->dev, (int)payload[0]) == 0) {
				muc_uart_send(mm_data, MUC_SET_GPIO|MSG_ACK_MASK, NULL, 0);
			} else {
				muc_uart_send(mm_data, MUC_SET_GPIO|MSG_NACK_MASK, NULL, 0);
			}
			break;
		}
		case USB_CONTROL: {
			/* TODO nack if size is wrong ? */
			if (payload_len == sizeof(struct usb_control_t))
				muc_uart_handle_usb_ctrl(mm_data,
					(struct usb_control_t *)payload);
			muc_uart_send(mm_data, hdr->cmd|MSG_ACK_MASK, NULL, 0);
			break;
		}
		case MUC_DEBUG_MSG: {
			if (payload_len) {
				/* Ensure null terminated */
				payload[payload_len - 1] = '\0';
				MUC_LOG("muc_dbg_msg: %s\n", (char *)payload);
			}
			muc_uart_send(mm_data, hdr->cmd|MSG_ACK_MASK, NULL, 0);
			break;
		}
		default: {
			if (hdr->cmd & MSG_ACK_MASK)
				clear_wait_for_ack(mm_data);
			else if (hdr->cmd & MSG_NACK_MASK)
				clear_wait_for_ack(mm_data);
			else {
				MUC_ERR("Unhandled type %d\n", hdr->cmd);
				muc_uart_send(mm_data, hdr->cmd|MSG_NACK_MASK, NULL, 0);
			}
		}
	}
}

static size_t muc_uart_rx_cb(struct platform_device *pdev,
	uint8_t *data,
	size_t len)
{
	int i;
	size_t content_size;
	size_t segment_size;
	struct mmi_uart_hdr_t *hdr;
	uint16_t calc_crc;
	uint16_t rcvd_crc;
	struct mod_muc_data_t *mm_data =
		platform_get_drvdata(pdev);

	reset_idle_timer(mm_data);

	/* Need at least a header to start */
	if (len < sizeof(*hdr))
		return 0;

	hdr = (struct mmi_uart_hdr_t *)data;

	/* Verify the magic number */
	if (le16_to_cpu(hdr->magic) != MSG_MAGIC) {
		MUC_ERR("invalid magic %x\n",
			le16_to_cpu(hdr->magic));
		mmi_uart_report_rx_mag_err(mm_data->uart_data);

		/* Look through the rest of the data we have,
		 * and try to find a magic number, then throw out
		 * the bytes in front of it by returning the count
		 * before it.  This function will be re-entered
		 * and the magic number should now be in front.
		 */
		for(i = 1; i < (len - 1); i++) {
			if (le16_to_cpu(*(uint16_t *)(data + i)) == MSG_MAGIC) {
				MUC_LOG("Found valid data at offset %d\n", i);
				return i;
			}
		}

		return i;
	}

	segment_size = le16_to_cpu(hdr->payload_length)
		+ sizeof(*hdr) + sizeof(calc_crc);

	/* Validate the payload size */
	if (le16_to_cpu(hdr->payload_length) >
		UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE) {
		MUC_ERR("invalid len %zd\n", segment_size);
		mmi_uart_report_rx_len_err(mm_data->uart_data);
		return sizeof(*hdr);
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
		MUC_ERR("CRC mismatch, received: 0x%x, "
			"calculated: 0x%x\n", le16_to_cpu(rcvd_crc), calc_crc);
	} else {
		MUC_DBG("cmd=%x, magic=%x, "
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
		MUC_DBG("off.\n");
#if MUC_UART_DISABLE_ON_SLEEP
		if (mm_data->uart_pm_state) {
			mmi_uart_do_pm(mm_data->uart_data, false);
			mm_data->uart_pm_state = 0;
		}
#endif
		/* Set to high Z / float */
		gpio_direction_input(mm_data->wake_out_gpio);
		clear_sleep_req_pending(mm_data);
		mmi_uart_clear_tx_busy(mm_data->uart_data);
	}
}

/* TODO call from power management wake. */
static void muc_uart_wake_work(struct work_struct *w)
{
	static bool booted = false;
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, wake_work.work);

	cancel_delayed_work_sync(&mm_data->sleep_work);
	atomic_set(&mm_data->suspend_ok, 0);

	reset_idle_timer(mm_data);

	if (!mm_data || !mm_data->uart_data ||
		mmi_uart_set_tx_busy(mm_data->uart_data))
		schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(500));
	else {
		MUC_DBG("on.\n");

		if (!mm_data->uart_pm_state) {
			mmi_uart_do_pm(mm_data->uart_data, true);
			mm_data->uart_pm_state = 1;
		}

		muc_uart_wake_muc(mm_data);
		mmi_uart_clear_tx_busy(mm_data->uart_data);

		/* Send boot mode on initial wake */
		if (!booted) {
			if (muc_uart_send_bootmode(mm_data) >= 0);
				booted = true;
		/* Else if there was something in the queue when
		 * we went to sleep start sending it
		 */
		} else {
			cancel_delayed_work(&mm_data->write_work);
			queue_delayed_work(mm_data->write_wq,
					&mm_data->write_work,
					msecs_to_jiffies(0));
		}
	}
}

static irqreturn_t muc_uart_wake_irq_handler(int irq, void *data)
{
	struct mod_muc_data_t *mm_data = (struct mod_muc_data_t *)data;

	/* 0->1 irq is to sleep, 1->0 irq is to wake */
	if (gpio_get_value(mm_data->wake_in_gpio)) {
		if (atomic_read(&mm_data->sleep_req_pending)) {
			cancel_delayed_work(&mm_data->sleep_work);
			schedule_delayed_work(&mm_data->sleep_work,
				msecs_to_jiffies(0));
		}
	} else {
		cancel_delayed_work(&mm_data->wake_work);
		schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(0));
	}

	return IRQ_HANDLED;
}

static void muc_uart_idle_work(struct work_struct *w)
{
	unsigned long flags;
	struct mod_muc_data_t *mm_data =
		container_of(w, struct mod_muc_data_t, idle_work.work);

	if (!atomic_read(&mm_data->sleep_req_pending)) {
		mutex_lock(&tx_lock);
		spin_lock_irqsave(&write_q_lock, flags);
		if(!mm_data->write_data_q &&
			muc_uart_send(mm_data, UART_SLEEP_REQ, NULL, 0) < 0)
			schedule_delayed_work(&mm_data->idle_work,
				msecs_to_jiffies(1000));
		spin_unlock_irqrestore(&write_q_lock, flags);
		mutex_unlock(&tx_lock);
	}

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

static void muc_uart_sleep_req_cb(unsigned long timer_data)
{
	struct mod_muc_data_t *mm_data =
		(struct mod_muc_data_t *)timer_data;

	/* TODO this should probably panic */
	if (mm_data) {
		if(atomic_read(&mm_data->sleep_req_pending))
			atomic_set(&mm_data->sleep_req_pending, 0);
	}
}

static int muc_uart_register_cb(struct platform_device *pdev,
	void *uart_data)
{
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);

	if (!mm_data)
		return 1;

	mm_data->uart_data = uart_data;
	mmi_uart_open(uart_data);
	schedule_delayed_work(&mm_data->wake_work, msecs_to_jiffies(0));

	MUC_LOG("register ok\n");

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

static enum power_supply_property muc_uart_ps_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int muc_uart_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct mod_muc_data_t *mm_data = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = atomic_read(&mm_data->mod_attached);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&mm_data->mod_online);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int muc_uart_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct mod_muc_data_t *mm_data = power_supply_get_drvdata(psy);
	struct usb_control_t usbctrl_pkt;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE: {
		schedule_delayed_work(&mm_data->connect_work,
			msecs_to_jiffies(EXT_USB_CHECK_TIME));

		usbctrl_pkt.cmd = val->intval;
		muc_uart_queue_send(mm_data,
			USB_CONTROL,
			(uint8_t *)&usbctrl_pkt,
			sizeof(struct usb_control_t));
		break;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct power_supply_desc phone_psy_desc = {
	.name		= "phone",
	.type		= POWER_SUPPLY_TYPE_MAIN,
	.get_property	= muc_uart_get_property,
	.set_property   = muc_uart_set_property,
	.properties	= muc_uart_ps_props,
	.num_properties	= ARRAY_SIZE(muc_uart_ps_props),
};

static int muc_uart_probe(struct platform_device *pdev)
{
	struct device_node *np = (&pdev->dev)->of_node;
	struct mod_muc_data_t *mm_data;
	struct power_supply_config psy_cfg = {};

#ifdef CONFIG_IPC_LOGGING
	muc_ipc_log = ipc_log_context_create(MUC_IPC_LOG_PAGES, "muc_uart", 0);
	if (!muc_ipc_log)
		MUC_ERR("Failed to create IPC logging context\n");
#endif

	MUC_LOG("probe start\n");

	mm_data = devm_kzalloc(&pdev->dev, sizeof(*mm_data), GFP_KERNEL);
	if (!mm_data)
		return -ENOMEM;
	mm_data->pdev = pdev;
	platform_set_drvdata(pdev, mm_data);
	psy_cfg.drv_data = mm_data;

	INIT_DELAYED_WORK(&mm_data->wake_work, muc_uart_wake_work);
	INIT_DELAYED_WORK(&mm_data->sleep_work, muc_uart_sleep_work);
	INIT_DELAYED_WORK(&mm_data->idle_work, muc_uart_idle_work);
	INIT_DELAYED_WORK(&mm_data->attach_work, muc_uart_attach_work);
	INIT_DELAYED_WORK(&mm_data->write_work, muc_uart_send_work);
	INIT_DELAYED_WORK(&mm_data->connect_work, muc_uart_connect_work);
	INIT_WORK(&mm_data->ps_notify_work, muc_uart_ps_notifier_work);

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
	setup_timer(&mm_data->sleep_req_timer,
		muc_uart_sleep_req_cb,
		(unsigned long)mm_data);

	if (sysfs_create_groups(&pdev->dev.kobj, muc_uart_groups)) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		goto err_dest_wq;
	}

	if (mmi_uart_init(muc_uart_rx_cb,
		muc_uart_register_cb,
		muc_uart_get_uart_data,
		pdev))
		goto err_dest_groups;

	mm_data->wake_out_gpio = of_get_gpio(np, 0);
	mm_data->wake_in_gpio = of_get_gpio(np, 1);
	mm_data->mod_attached_gpio = of_get_gpio(np, 2);
	mm_data->muc_1_gpio = of_get_gpio(np, 3);
	mm_data->muc_2_gpio = of_get_gpio(np, 4);

	if (mm_data->wake_out_gpio < 0 ||
		mm_data->wake_in_gpio < 0 ||
		mm_data->mod_attached_gpio < 0 ||
		mm_data->muc_1_gpio < 0 ||
		mm_data->muc_2_gpio < 0)
		goto err_exit_uart;

	mm_data->wake_irq =
		gpio_to_irq(mm_data->wake_in_gpio);
	mm_data->mod_attached_irq =
		gpio_to_irq(mm_data->mod_attached_gpio);

	/* Let wake irq take us out of suspend */
	if (irq_set_irq_wake(mm_data->wake_irq, 1))
		MUC_ERR("Failed to set wake irq as wake source");
	/* Let attach irq take us out of suspend */
	if (irq_set_irq_wake(mm_data->mod_attached_irq, 1))
		MUC_ERR("Failed to set attach irq as wake source");

	if (devm_request_irq(&pdev->dev,
		mm_data->wake_irq,
		muc_uart_wake_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		pdev->dev.driver->name,
		mm_data))
		goto err_exit_uart;

	if (devm_request_irq(&pdev->dev,
		mm_data->mod_attached_irq,
		muc_uart_attach_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		pdev->dev.driver->name,
		mm_data))
		goto err_exit_uart;

	if (muc_uart_config_gpio(&pdev->dev,
			mm_data->wake_out_gpio, "muc_wake", 1, 1))
		goto err_exit_uart;

	if (muc_uart_config_gpio(&pdev->dev,
			mm_data->muc_1_gpio, "muc_1", 1, 0))
		goto err_free_wake_out;

	if (muc_uart_config_gpio(&pdev->dev,
			mm_data->muc_2_gpio, "muc_2", 1, 1))
		goto err_free_muc1;

	if (mmi_char_init(UART_MAX_MSG_SIZE - MSG_META_DATA_SIZE, pdev))
		goto err_free_muc2;

	mm_data->phone_psy = devm_power_supply_register(&pdev->dev,
							&phone_psy_desc,
							&psy_cfg);
	if (IS_ERR(mm_data->phone_psy)) {
		MUC_ERR("failed: phone power supply register\n");
		goto err_exit_char;
	}

	mm_data->batt_psy = power_supply_get_by_name("battery");
	mm_data->bms_psy = power_supply_get_by_name("bms");
	mm_data->usb_psy = power_supply_get_by_name("usb");
	mm_data->mmi_psy = power_supply_get_by_name("mmi_battery");
	mm_data->dc_psy = power_supply_get_by_name("dc");

	mm_data->ps_nb.notifier_call = muc_uart_ps_notifier_cb;
	if (power_supply_reg_notifier(&mm_data->ps_nb)) {
		MUC_ERR("couldn't register ps notifier\n");
		goto err_put_ps;
	}

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	schedule_delayed_work(&mm_data->attach_work, msecs_to_jiffies(0));

	MUC_LOG("probe done\n");

	return 0;

err_put_ps:
	power_supply_put(mm_data->batt_psy);
	power_supply_put(mm_data->bms_psy);
	power_supply_put(mm_data->usb_psy);
	power_supply_put(mm_data->mmi_psy);
	power_supply_put(mm_data->dc_psy);
err_exit_char:
	mmi_char_exit();
err_free_muc2:
	gpio_free(mm_data->muc_2_gpio);
err_free_muc1:
	gpio_free(mm_data->muc_1_gpio);
err_free_wake_out:
	gpio_free(mm_data->wake_out_gpio);
err_exit_uart:
	mmi_uart_exit(mm_data->uart_data);
err_dest_groups:
	sysfs_remove_groups(&pdev->dev.kobj, muc_uart_groups);
err_dest_wq:
	del_timer(&mm_data->idle_timer);
	del_timer(&mm_data->ack_timer);
	del_timer(&mm_data->sleep_req_timer);
	destroy_workqueue(mm_data->write_wq);
err:
	return 1;
}

static int muc_uart_remove(struct platform_device *pdev)
{
	struct mod_muc_data_t *mm_data = platform_get_drvdata(pdev);

	irq_set_irq_wake(mm_data->wake_irq, 0);
	irq_set_irq_wake(mm_data->mod_attached_irq, 0);

	if (gpio_is_valid(mm_data->wake_in_gpio))
		gpio_free(mm_data->wake_in_gpio);

	if (gpio_is_valid(mm_data->wake_out_gpio))
		gpio_free(mm_data->wake_out_gpio);

	if (gpio_is_valid(mm_data->mod_attached_gpio))
		gpio_free(mm_data->mod_attached_gpio);

	power_supply_put(mm_data->batt_psy);
	power_supply_put(mm_data->bms_psy);
	power_supply_put(mm_data->usb_psy);
	power_supply_put(mm_data->mmi_psy);
	power_supply_put(mm_data->dc_psy);
	del_timer(&mm_data->idle_timer);
	del_timer(&mm_data->ack_timer);
	del_timer(&mm_data->sleep_req_timer);
	cancel_delayed_work_sync(&mm_data->wake_work);
	cancel_delayed_work_sync(&mm_data->sleep_work);
	cancel_delayed_work_sync(&mm_data->idle_work);
	cancel_delayed_work_sync(&mm_data->attach_work);
	cancel_delayed_work_sync(&mm_data->write_work);
	cancel_work_sync(&mm_data->ps_notify_work);
	destroy_workqueue(mm_data->write_wq);
	mmi_uart_exit(mm_data->uart_data);
	mmi_char_exit();
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
	MUC_LOG("guid: %s\n", param_guid);
	MUC_LOG("vers: %s\n", param_vers);
	MUC_LOG("mucfw: %s\n", param_mucfw);
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
