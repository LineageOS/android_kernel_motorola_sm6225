// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 **** Copyright (C), 2022, Shanghai awinic technology Co.,Ltd.
												all rights reserved. ***********
 *******************************************************************************
 * File Name     : tcpc_aw35616.c
 * Author        : awinic
 * Date          : 2022-12-16
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/version.h>
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#include <linux/sched/types.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/sched/rt.h>
#include <uapi/linux/sched/types.h>

//#include "inc/tcpci_timer.h"
//#include "inc/tcpci_typec.h"
//#include "inc/pd_dbg_info.h"
#include "inc/tcpci.h"
#include "inc/tcpc_aw35616.h"
#include <linux/usb/typec.h>

#define AW35616_DRIVER_VERSION	"V0.3.0"

#define AW35616_IRQ_WAKE_TIME	(1000) /* ms */

const unsigned char aw35616_reg_access[AW35616_REG_MAX] = {
	[AW35616_REG_DEV_ID] = REG_RD_ACCESS,
	[AW35616_REG_CTR] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW35616_REG_INT] = REG_RD_ACCESS,
	[AW35616_REG_STATUS] = REG_RD_ACCESS,
	[AW35616_REG_STATUS1] = REG_RD_ACCESS,
	[AW35616_REG_RSTN] = REG_WR_ACCESS,
	[AW35616_REG_USB_VID0] = REG_RD_ACCESS,
	[AW35616_REG_USB_VID1] = REG_RD_ACCESS,
	[AW35616_REG_CTR2] = REG_RD_ACCESS|REG_WR_ACCESS
};

/******************************************************
 *
 * aw35616 i2c write/read
 *
 ******************************************************/
static int aw35616_i2c_write(struct aw35616_chip *chip,
		u8 reg_addr, u8 reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	__pm_stay_awake(chip->for_i2c_wake_lock);

	while (cnt++ < AW35616_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(chip->client, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__,
					cnt, ret);
		} else {
			break;
		}
		usleep_range(1000, 2000);
	}

	__pm_relax(chip->for_i2c_wake_lock);

	return ret;
}

static int aw35616_i2c_read(struct aw35616_chip *chip,
		u8 reg_addr, u8 *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	__pm_stay_awake(chip->for_i2c_wake_lock);

	while (cnt++ < AW35616_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(chip->client, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__,
					cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		usleep_range(1000, 2000);
	}

	__pm_relax(chip->for_i2c_wake_lock);

	return ret;
}

static int aw35616_i2c_read_bits(struct aw35616_chip *chip,
		u8 reg_addr, u8 *buf, u8 len)
{
	int ret = 0;
	unsigned char *rdbuf = NULL;
	unsigned char cnt = 0;
	struct i2c_msg msgs[] = {
		{
			.addr	 = chip->client->addr,
			.flags  = 0,
			.len	 = 1,
			.buf	 = &reg_addr,
		},
		{
			.addr	 = chip->client->addr,
			.flags  = I2C_M_RD,
			.len	 = len,
		},
	};

	__pm_stay_awake(chip->for_i2c_wake_lock);

	if (chip->client == NULL) {
		pr_err("msg %s i2c client is NULL\n", __func__);
		return -ENODEV;
	}

	rdbuf = kmalloc(len, GFP_KERNEL);
	if (rdbuf == NULL)
		return -ENOMEM;

	msgs[1].buf = rdbuf;

	while (cnt < AW35616_I2C_RETRIES) {
		ret = i2c_transfer(chip->client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret < 0)
			pr_err("msg %s i2c read error: %d\n", __func__, ret);
		else
			break;
		cnt++;
		usleep_range(1000, 2000);
	}

	if (buf != NULL)
		memcpy(buf, rdbuf, len);

	kfree(rdbuf);

	__pm_relax(chip->for_i2c_wake_lock);

	return ret;
}

static int aw35616_remote_rp_level(struct tcpc_device *tcpc)
{
	struct aw35616_chip *chip = tcpc_get_dev_data(tcpc);
	if (chip->tcpc->typec_usb_sink_curr == 1)
		return TYPEC_CC_VOLT_SNK_DFT;
	else if (chip->tcpc->typec_usb_sink_curr == 2)
		return TYPEC_CC_VOLT_SNK_1_5;
	else if (chip->tcpc->typec_usb_sink_curr == 3)
		return TYPEC_CC_VOLT_SNK_3_0;
	else
		return TYPEC_CC_VOLT_SNK_DFT;
}

static int first_check_flag;
static void aw35616_first_check_typec_work(struct work_struct *work)
{
	struct aw35616_chip *chip = container_of(work,
			struct aw35616_chip, first_check_typec_work.work);
	int i = 0;
	u8 reg_data[4];

	tcpci_lock_typec(chip->tcpc);
	first_check_flag = 1;
	/* get interrupt */
	aw35616_i2c_read(chip, AW35616_REG_INT, &chip->reg.ints.byte);
	AW_LOG("int_sts[0x%02x]\n", chip->reg.ints.byte);
	if (chip->reg.ints.intb_flag == NO_INTB) {
		aw35616_i2c_read_bits(chip, AW35616_REG_CTR, reg_data, 4);
		for (i = 1; i < 5; i++)
			AW_LOG("reg:0x%02x=0x%02x\n", i, reg_data[i - 1]);

		tcpci_unlock_typec(chip->tcpc);
		return;
	}

	if (chip->reg.ints.intb_flag == ATTACHED) {
		aw35616_i2c_read_bits(chip, AW35616_REG_STATUS, chip->reg.status.byte, 2);
		AW_LOG("plug_st = %d snk_det_rp_dbg = %d\n",
				chip->reg.status.plug_st, chip->reg.status.snk_det_rp_dbg);
		if (chip->reg.status.plug_ori > 0)
			chip->tcpc->typec_polarity = chip->reg.status.plug_ori - 1;
		AW_LOG("plug_ori = %d typec_polarity = %d\n",chip->reg.status.plug_ori,chip->tcpc->typec_polarity);
		switch (chip->reg.status.plug_st) {
		case STANDBY:
			AW_LOG("plug status not connected\n");
			break;
		case SINK:
			chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_1_5;
			if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
				chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
				tcpci_source_vbus(chip->tcpc,
					TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 1000);
				tcpci_notify_typec_state(chip->tcpc);
				chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
			}
			break;
		case SOURCE:
			if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
				chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
				AW_LOG("snk_cur_md:%d\n",chip->reg.status.snk_cur_md);
				chip->tcpc->typec_usb_sink_curr = chip->reg.status.snk_cur_md;
				chip->tcpc->typec_remote_rp_level = aw35616_remote_rp_level(chip->tcpc);
				tcpci_notify_typec_state(chip->tcpc);
				chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
			}
			break;
		case AUD_ACC:
			if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_AUDIO) {
				chip->tcpc->typec_attach_new = TYPEC_ATTACHED_AUDIO;
				tcpci_notify_typec_state(chip->tcpc);
				chip->tcpc->typec_attach_old = TYPEC_ATTACHED_AUDIO;
			}
			break;
		case DUG_ACC:
			if (chip->reg.status.snk_det_rp_dbg) {
				AW_LOG("plug int Rp-Rp\n");
				if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
					chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
					tcpci_notify_typec_state(chip->tcpc);
					chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
				}
			} else {
				AW_LOG("plug int Rd-Rd\n");
				chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_1_5;
				if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
					chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
					tcpci_source_vbus(chip->tcpc,
							TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 1000);
					tcpci_notify_typec_state(chip->tcpc);
					chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
				}
			}
			break;
		default:
			AW_LOG("plug status unknown = %d\n", chip->reg.status.plug_st);
			break;
		}
	} else if (chip->reg.ints.intb_flag == DETACHED) {
		if ((chip->reg.status.plug_st == SINK) ||
			(chip->reg.status.plug_st == SOURCE) ||
			(chip->reg.status.plug_st == DUG_ACC) ||
			(chip->reg.status.plug_st == AUD_ACC)) {
			chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
			tcpci_notify_typec_state(chip->tcpc);
			if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SRC) {
				tcpci_source_vbus(chip->tcpc,
						TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
			}
			chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
		} else {
			AW_LOG("unknown plug out\n");
		}
	}

	tcpci_unlock_typec(chip->tcpc);
}

static irqreturn_t aw35616_irq_work_handler(int irq, void *data)
{
	struct aw35616_chip *chip = data;

	int i = 0;
	u8 reg_data[4];
	u8 reg_data_20;

	if (first_check_flag == 0)
		return IRQ_HANDLED;

	tcpci_lock_typec(chip->tcpc);
	/* get interrupt */
	aw35616_i2c_read(chip, AW35616_REG_INT, &chip->reg.ints.byte);
	AW_LOG("int_sts[0x%02x]\n", chip->reg.ints.byte);
	reg_data_20 = 0x0;
	if (chip->reg.ints.intb_flag == NO_INTB) {
		aw35616_i2c_read_bits(chip, AW35616_REG_CTR, reg_data, 4);
		for (i = 1; i < 5; i++)
			AW_LOG("reg:0x%02x=0x%02x\n", i, reg_data[i - 1]);
		aw35616_i2c_read(chip,AW35616_REG_TYPEC_STATUS,&reg_data_20);
		AW_LOG("reg:0x20=0x%02x\n",reg_data_20);
		tcpci_unlock_typec(chip->tcpc);
		return IRQ_HANDLED;
	}

	if (chip->reg.ints.intb_flag == ATTACHED) {
		aw35616_i2c_read_bits(chip, AW35616_REG_STATUS, chip->reg.status.byte, 2);
		AW_LOG("plug_st = %d snk_det_rp_dbg = %d\n",
				chip->reg.status.plug_st, chip->reg.status.snk_det_rp_dbg);
		if(chip->reg.status.plug_ori > 0)
			chip->tcpc->typec_polarity = chip->reg.status.plug_ori -1;
		AW_LOG("plug_ori = %d, typec_polarity = %d", chip->reg.status.plug_ori, chip->tcpc->typec_polarity);
		switch (chip->reg.status.plug_st) {
		case STANDBY:
			AW_LOG("plug status not connected\n");
			break;
		case SINK:
			chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_1_5;
			if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
				chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
				tcpci_source_vbus(chip->tcpc,
					TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 1000);
				tcpci_notify_typec_state(chip->tcpc);
				chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
			}
			break;
		case SOURCE:
			if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
				chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
				AW_LOG("snk_cur_md:%d\n",chip->reg.status.snk_cur_md);
				chip->tcpc->typec_usb_sink_curr = chip->reg.status.snk_cur_md;
				chip->tcpc->typec_remote_rp_level = aw35616_remote_rp_level(chip->tcpc);
				tcpci_notify_typec_state(chip->tcpc);
				chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
			}
			break;
		case AUD_ACC:
			if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_AUDIO) {
				chip->tcpc->typec_attach_new = TYPEC_ATTACHED_AUDIO;
				tcpci_notify_typec_state(chip->tcpc);
				chip->tcpc->typec_attach_old = TYPEC_ATTACHED_AUDIO;
			}
			break;
		case DUG_ACC:
			if (chip->reg.status.snk_det_rp_dbg) {
				AW_LOG("plug int Rp-Rp\n");
				if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
					chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
					tcpci_notify_typec_state(chip->tcpc);
					chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
				}
			} else {
				AW_LOG("plug int Rd-Rd\n");
				chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_1_5;
				if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
					chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
					tcpci_source_vbus(chip->tcpc,
							TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 1000);
					tcpci_notify_typec_state(chip->tcpc);
					chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
				}
			}
			break;
		default:
			AW_LOG("plug status unknown = %d\n", chip->reg.status.plug_st);
			break;
		}
	} else if (chip->reg.ints.intb_flag == DETACHED) {
		if ((chip->reg.status.plug_st == SINK) ||
			(chip->reg.status.plug_st == SOURCE) ||
			(chip->reg.status.plug_st == DUG_ACC) ||
			(chip->reg.status.plug_st == AUD_ACC)) {
			chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
			tcpci_notify_typec_state(chip->tcpc);
			if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SRC) {
				tcpci_source_vbus(chip->tcpc,
						TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
			}
			chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
		} else {
			AW_LOG("unknown plug out\n");
		}
	}

	aw35616_i2c_read_bits(chip,AW35616_REG_CTR,reg_data,4);
	for (i = 1;i < 5;i++)
		AW_LOG("reg:0x%02x=0x%02x\n",i,reg_data[i -1]);
	aw35616_i2c_read(chip,AW35616_REG_TYPEC_STATUS,&reg_data_20);
	AW_LOG("reg:0x20=0x%02x\n",reg_data_20);

	tcpci_unlock_typec(chip->tcpc);

	return IRQ_HANDLED;
}

static int aw35616_init_alert(struct tcpc_device *tcpc)
{
	struct aw35616_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	char *name;
	int len;

	len = strlen(chip->tcpc_desc->name);
	name = devm_kzalloc(chip->dev, len+5, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, PAGE_SIZE, "%s-IRQ", chip->tcpc_desc->name);

	pr_err("%s name = %s, gpio = %d\n", __func__,
				chip->tcpc_desc->name, chip->irq_gpio);

	ret = devm_gpio_request(chip->dev, chip->irq_gpio, name);
	if (ret < 0) {
		pr_err("Error: failed to request GPIO%d (ret = %d)\n",
		chip->irq_gpio, ret);
		goto init_alert_err;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		pr_err("Error: failed to set GPIO%d as input pin(ret = %d)\n",
		chip->irq_gpio, ret);
		goto init_alert_err;
	}

	chip->irq = gpio_to_irq(chip->irq_gpio);
	if (chip->irq <= 0) {
		pr_err("%s gpio to irq fail, chip->irq(%d)\n",
						__func__, chip->irq);
		goto init_alert_err;
	}

	pr_err("%s : IRQ number = %d\n", __func__, chip->irq);

	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
			aw35616_irq_work_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			name, chip);

	if(ret < 0){
		pr_err("Error: failed to request irq%d(gpio = %d, ret=%d)\n",
			chip->irq, chip->irq_gpio, ret);
		goto init_alert_err;
	}


	device_init_wakeup(chip->dev, true);
	return 0;
init_alert_err:
	return -EINVAL;
}

int aw35616_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	struct aw35616_chip *chip = tcpc_get_dev_data(tcpc);

	AW_LOG("enter\n");

		/* set toggle cycle time */
	chip->reg.ctr2.tog_save_md = chip->toggle_cycle;
	aw35616_i2c_write(chip, AW35616_REG_CTR2, chip->reg.ctr2.byte);

	/* set power role mode */
	switch (chip->tcpc_desc->role_def) {
	case 1: /* set only sink */
		chip->reg.ctr.wkmd = SNK;
		chip->reg.ctr.try_md = NO_TRY;
		break;
	case 2: /* set only source */
		chip->reg.ctr.wkmd = SRC;
		chip->reg.ctr.try_md = NO_TRY;
		break;
	case 3: /* set only drp */
		chip->reg.ctr.wkmd = DRP;
		chip->reg.ctr.try_md = NO_TRY;
		break;
	case 4: /* set try src */
		chip->reg.ctr.wkmd = DRP;
		chip->reg.ctr.try_md = TRY_SRC;
		break;
	case 5: /* set try snk */
		chip->reg.ctr.wkmd = DRP;
		chip->reg.ctr.try_md = TRY_SNK;
		break;
	default:
		break;
	}

	/* set source current */
	switch (chip->tcpc_desc->rp_lvl) {
	case TYPEC_CC_RP_DFT: /* RP Default */
		chip->reg.ctr.src_cur_md = SRC_DEF;
		break;
	case TYPEC_CC_RP_1_5: /* RP 1.5V */
		chip->reg.ctr.src_cur_md = SRC_1_5A;
		break;
	case TYPEC_CC_RP_3_0: /* RP 3.0V */
		chip->reg.ctr.src_cur_md = SRC_3_0A;
		break;
	default:
		break;
	}

	/* set acc mode */
	chip->reg.ctr.accdis = chip->acc_support;
	aw35616_i2c_write(chip, AW35616_REG_CTR, chip->reg.ctr.byte);
	aw35616_i2c_write(chip, AW35616_reg_val1, AW35616_reg_val2);
	aw35616_i2c_write(chip, AW35616_reg_val3, AW35616_reg_val4);
	aw35616_i2c_write(chip, AW35616_reg_val1, AW35616_reg_val5);

	/* The first detection interrupt can be completed. delay 150ms */
	msleep(150);

	schedule_delayed_work(&chip->first_check_typec_work, msecs_to_jiffies(1000));

	return 0;
}

int aw35616_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35616_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35616_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_get_power_status(
		struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35616_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{

	AW_LOG("enter\n");
	return 0;
}

static int aw35616_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	AW_LOG("enter\n");
	return 0;
}

int aw35616_typec_cc_orientation(void)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_set_role(struct tcpc_device *tcpc, int mode);
static int aw35616_set_cc(struct tcpc_device *tcpc, int pull)
{
	int ret;
	uint8_t mode = 0;

	AW_LOG("enter\n");

	if (pull == TYPEC_CC_RP)
		mode = REVERSE_CHG_SOURCE;
	else if (pull == TYPEC_CC_RD)
		mode = REVERSE_CHG_SINK;
	else if (pull == TYPEC_CC_DRP)
		mode = REVERSE_CHG_DRP;

	ret = aw35616_set_role(tcpc, mode);
	if (ret < 0)
		pr_err("%s: set mode fail!\n", __func__);

	return 0;
}

static int aw35616_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_set_vconn(struct tcpc_device *tcpc, int enable)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_tcpc_deinit(struct tcpc_device *tcpc_dev)
{
	AW_LOG("enter\n");

#ifdef CONFIG_TCPC_SHUTDOWN_CC_DETACH
	aw35616_set_cc(tcpc_dev, TYPEC_CC_RD);
#endif

	return 0;
}

static int aw35616_set_role(struct tcpc_device *tcpc, int mode)
{
	AW_LOG("dhx--set role  end %d\n", mode);
	return 0;
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int aw35616_set_msg_header(struct tcpc_device *tcpc,
					uint8_t power_role, uint8_t data_role)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_protocol_reset(struct tcpc_device *tcpc_dev)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_get_message(struct tcpc_device *tcpc, uint32_t *payload,
			uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_transmit(struct tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	AW_LOG("enter\n");
	return 0;
}

static int aw35616_set_bist_carrier_mode(
	struct tcpc_device *tcpc, uint8_t pattern)
{
	AW_LOG("enter\n");
	return 0;
}
#endif	/* CONFIG_USB_POWER_DELIVERY */

static struct tcpc_ops aw35616_tcpc_ops = {
	.init = aw35616_tcpc_init,
	.alert_status_clear = aw35616_alert_status_clear,
	.fault_status_clear = aw35616_fault_status_clear,
	.get_alert_mask = aw35616_get_alert_mask,
	.get_alert_status = aw35616_get_alert_status,
	.get_power_status = aw35616_get_power_status,
	.get_fault_status = aw35616_get_fault_status,
	.get_cc = aw35616_get_cc,
	.set_cc = aw35616_set_cc,
	//.set_role = aw35616_set_role,
	//.get_mode = aw35616_tcpc_get_mode,
	.set_polarity = aw35616_set_polarity,
	.set_low_rp_duty = aw35616_set_low_rp_duty,
	.set_vconn = aw35616_set_vconn,
	.deinit = aw35616_tcpc_deinit,

#ifdef CONFIG_USB_POWER_DELIVERY
	.set_msg_header = aw35616_set_msg_header,
	.set_rx_enable = aw35616_set_rx_enable,
	.protocol_reset = aw35616_protocol_reset,
	.get_message = aw35616_get_message,
	.transmit = aw35616_transmit,
	.set_bist_test_mode = aw35616_set_bist_test_mode,
	.set_bist_carrier_mode = aw35616_set_bist_carrier_mode,
#endif	/* CONFIG_USB_POWER_DELIVERY */
};


static int aw35616_parse_dt(struct aw35616_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -EINVAL;

	AW_LOG("enter\n");

	np = of_find_node_by_name(NULL, "usb_type_c_aw35616");
	if (!np) {
		pr_err("%s find node type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	ret = of_get_named_gpio(np, "aw35616,irq-gpio", 0);
	if (ret < 0) {
		pr_err("%s no intr_gpio info\n", __func__);
		return ret;
	}
	chip->irq_gpio = ret;

	return ret;
}

static int aw35616_tcpcdev_init(struct aw35616_chip *chip, struct device *dev)
{
	struct tcpc_desc *desc;
	struct device_node *np;
	u32 val, len;

	const char *name = "default";

	np = of_find_node_by_name(NULL, "usb_type_c_aw35616");
	if (!np) {
		pr_err("%s find node aw35616 fail\n", __func__);
		return -ENODEV;
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	if (of_property_read_u32(np, "aw35616_acc_support", &val) >= 0) {
		chip->acc_support = (bool)val;
		AW_LOG("aw35616_acc_support = %d\n", chip->acc_support);
	} else {
		dev_info(dev, "use default val\n");
		chip->acc_support = false;
	}
	if (of_property_read_u32(np, "aw35616-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
		AW_LOG("aw35616_role_def = %d\n", desc->role_def);
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(
		np, "aw35616-tcpc,notifier_supply_num", &val) >= 0) {
		if (val < 0)
			desc->notifier_supply_num = 0;
		else
			desc->notifier_supply_num = val;
	} else
		desc->notifier_supply_num = 0;

	if (of_property_read_u32(np, "aw35616,rp_level", &val) >= 0) {
		switch (val) {
		case 0: /* RP Default */
			desc->rp_lvl = TYPEC_CC_RP_DFT;
			break;
		case 1: /* RP 1.5V */
			desc->rp_lvl = TYPEC_CC_RP_1_5;
			break;
		case 2: /* RP 3.0V */
			desc->rp_lvl = TYPEC_CC_RP_3_0;
			break;
		default:
			break;
		}
		AW_LOG("aw35616_rp_level = %d\n", desc->rp_lvl);
	}

	if (of_property_read_u32(np, "aw35616_toggle_cycle", &val) >= 0) {
		chip->toggle_cycle = (uint8_t)val;
		AW_LOG("aw35616_toggle_cycle = %d\n", chip->toggle_cycle);
	} else {
		dev_info(dev, "use default val\n");
		chip->toggle_cycle = 0;
	}
	of_property_read_string(np, "aw35616-tcpc,name", (char const **)&name);

	len = strlen(name);
	desc->name = kzalloc(len+1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;

	strscpy((char *)desc->name, name, len+1);

	chip->tcpc_desc = desc;

	chip->tcpc = tcpc_device_register(dev,
			desc, &aw35616_tcpc_ops, chip);
	if (IS_ERR(chip->tcpc))
		return -EINVAL;

	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;

	return 0;
}

static inline bool aw35616_check_revision(struct i2c_client *client)
{
	int rc = 0;
	int i = 0;

	for (i = 0; i <= AW35616_CHECK_RETEY; i++) {
		/* Read chip id */
		rc = i2c_smbus_read_byte_data(client, AW35616_REG_DEV_ID);
		if (rc >= 0) {
			if ((rc & 0x7) == AW35616_VENDOR_ID)
				AW_LOG("Device check passed chip id = 0x%x\n", rc);
			return true;
		}

		AW_LOG("ERROR: Could not communicate with device over i2c!\n");
		usleep_range(1000, 2000);
	}

	return false;
}

static void aw35616_init_reg(struct aw35616_chip *chip)
{
	chip->reg.rstn.sft_rstn = 1;
	aw35616_i2c_write(chip, AW35616_REG_RSTN, chip->reg.rstn.byte);
	chip->reg.rstn.sft_rstn = 0;
	/* Detect between gpio and i2c modes */
	msleep(30);
	aw35616_i2c_read(chip, AW35616_REG_DEV_ID, &chip->reg.dev_id.byte);
	aw35616_i2c_read(chip, AW35616_REG_CTR, &chip->reg.ctr.byte);
	aw35616_i2c_read_bits(chip, AW35616_REG_STATUS, chip->reg.status.byte, 2);
	aw35616_i2c_read_bits(chip, AW35616_REG_USB_VID0, chip->reg.vid.byte, 2);
	aw35616_i2c_read(chip, AW35616_REG_CTR2, &chip->reg.ctr2.byte);
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t regdump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw35616_chip *chip = i2c_get_clientdata(client);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW35616_REG_MAX; i++) {
		if (!(aw35616_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw35616_i2c_read(chip, i, &reg_val);
		AW_LOG("i = %d reg_val = %x\n", i, reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n",
				i, reg_val);
	}

	return len;
}

static ssize_t regdump_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw35616_chip *chip = i2c_get_clientdata(client);
	u32 databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw35616_i2c_write(chip, (u8)databuf[0], (u8)databuf[1]);

	return count;
}
DEVICE_ATTR_RW(regdump);

static ssize_t typec_polarity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw35616_chip *chip = i2c_get_clientdata(client);

	aw35616_i2c_read(chip, AW35616_REG_STATUS, &chip->reg.status.byte[0]);
	AW_LOG("reg_val[0x04] = %x\n", chip->reg.status.byte[0]);

	return sprintf(buf, "%s\n",
			(chip->reg.status.plug_ori == CC1) ? "CC1" :
			(chip->reg.status.plug_ori == CC2) ? "CC2" :
			(chip->reg.status.plug_ori == CC1_CC2) ? "CC1&&CC2" : "None");
}
DEVICE_ATTR_RO(typec_polarity);

static int aw35616_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct aw35616_chip *chip;
	int ret;
	bool chip_ret;

	AW_LOG("enter\n");
	if (i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA))
		AW_LOG("I2C functionality : OK...\n");
	else
		pr_err("I2C functionality check : failuare...\n");

	chip_ret = aw35616_check_revision(client);
	if (chip_ret == false) {
		pr_err("aw35616 init fail\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	aw35616_parse_dt(chip, &client->dev);

	chip->dev = &client->dev;
	chip->client = client;

	sema_init(&chip->suspend_lock, 1);
	i2c_set_clientdata(client, chip);

	INIT_DELAYED_WORK(&chip->first_check_typec_work,
			aw35616_first_check_typec_work);

	schedule_delayed_work(&chip->first_check_typec_work, msecs_to_jiffies(3000));

	chip->for_irq_wake_lock =
		wakeup_source_register(chip->dev, "aw35616_irq_wakelock");
	chip->for_i2c_wake_lock =
		wakeup_source_register(chip->dev, "aw35616_i2c_wakelock");

	aw35616_init_reg(chip);

	ret = aw35616_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "aw35616 tcpc dev init fail\n");
		goto err_tcpc_reg;
	}

	ret = aw35616_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("aw35616 init alert fail\n");
		goto err_irq_init;
	}

	ret = device_create_file(&client->dev, &dev_attr_regdump);
	if (ret < 0) {
		pr_err("failed to create regdump\n");
		ret = -ENODEV;
		goto err_irq_init;
	}


	ret = device_create_file(&client->dev, &dev_attr_typec_polarity);
	if (ret < 0) {
		pr_err("failed to create typec_polarity\n");
		ret = -ENODEV;
		goto err_create_fregdump_file;
	}

	AW_LOG("probe OK!\n");
	return 0;

err_create_fregdump_file:
	device_remove_file(&client->dev, &dev_attr_regdump);
err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
err_tcpc_reg:
	wakeup_source_unregister(chip->for_i2c_wake_lock);
	wakeup_source_unregister(chip->for_irq_wake_lock);
	return ret;
}

static int aw35616_i2c_remove(struct i2c_client *client)
{
	struct aw35616_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		cancel_delayed_work_sync(&chip->first_check_typec_work);
		tcpc_device_unregister(chip->dev, chip->tcpc);
		device_remove_file(&client->dev, &dev_attr_regdump);
		device_remove_file(&client->dev, &dev_attr_typec_polarity);
		wakeup_source_unregister(chip->for_i2c_wake_lock);
		wakeup_source_unregister(chip->for_irq_wake_lock);
	}

	return 0;
}

#ifdef CONFIG_PM
static int aw35616_i2c_suspend(struct device *dev)
{
	struct aw35616_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (device_may_wakeup(chip->dev))
			enable_irq_wake(chip->irq);
		disable_irq(chip->irq);
	}

	return 0;
}

static int aw35616_i2c_resume(struct device *dev)
{
	struct aw35616_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		enable_irq(chip->irq);
		if (device_may_wakeup(chip->dev))
			disable_irq_wake(chip->irq);
	}

	return 0;
}

static void aw35616_shutdown(struct i2c_client *client)
{
	struct aw35616_chip *chip = i2c_get_clientdata(client);

	/* Please reset IC here */
	chip->reg.rstn.sft_rstn = 0x1;
	aw35616_i2c_write(chip, AW35616_REG_RSTN, chip->reg.rstn.byte);
	if (chip != NULL) {
		if (chip->irq)
			disable_irq(chip->irq);
	}
}

#ifdef CONFIG_PM_RUNTIME
static int aw35616_pm_suspend_runtime(struct device *device)
{
	AW_LOG("pm_runtime: suspending...\n");
	return 0;
}

static int aw35616_pm_resume_runtime(struct device *device)
{
	AW_LOG("pm_runtime: resuming...\n");
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */


static const struct dev_pm_ops aw35616_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
			aw35616_i2c_suspend,
			aw35616_i2c_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(
			aw35616_pm_suspend_runtime,
			aw35616_pm_resume_runtime,
			NULL
	)
#endif /* CONFIG_PM_RUNTIME */
};

#define aw35616_PM_OPS (&aw35616_pm_ops)
#else
#define aw35616_PM_OPS (NULL)
#endif /* CONFIG_PM */

static const struct i2c_device_id aw35616_id_table[] = {
	{"aw35616", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, aw35616_id_table);

static const struct of_device_id aw35616_match_table[] = {
	{.compatible = "awinic,usb_type_c_aw35616",},
	{},
};

static struct i2c_driver aw35616_driver = {
	.driver = {
		.name = "usb_type_c_aw35616",
		.owner = THIS_MODULE,
		.of_match_table = aw35616_match_table,
		.pm = aw35616_PM_OPS,
	},
	.probe = aw35616_i2c_probe,
	.remove = aw35616_i2c_remove,
	.shutdown = aw35616_shutdown,
	.id_table = aw35616_id_table,
};

static int __init aw35616_init(void)
{
	AW_LOG("start driver init\n");
	AW_LOG("aw35616 driver version %s\n", AW35616_DRIVER_VERSION);

	return i2c_add_driver(&aw35616_driver);
}
subsys_initcall(aw35616_init);

static void __exit aw35616_exit(void)
{
	i2c_del_driver(&aw35616_driver);
}
module_exit(aw35616_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AWINIC");
MODULE_DESCRIPTION("aw35616 TCPC Driver");
MODULE_VERSION(AW35616_DRIVER_VERSION);


