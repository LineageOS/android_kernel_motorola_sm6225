/*
 * File: haptic_nv.c
 *
 * Author: <chelvming@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/vmalloc.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "haptic_nv.h"
#include "haptic_nv_reg.h"
#include "haptic_nv_dts.h"

#define HAPTIC_NV_DRIVER_VERSION	"v1.9.0"

static char aw_ram_name[][AW_NAME_MAX] = {
	{"aw8624_haptic.bin"},
	{"aw8622x_haptic.bin"},
	{"aw86214_haptic.bin"},
	{"aw8623x_haptic.bin"},
};

static char aw_rtp_name[][AW_NAME_MAX] = {
	{"haptic_nv_rtp_osc_24K_5s.bin"},
	{"haptic_nv_rtp.bin"},
	{"haptic_nv_rtp_lighthouse.bin"},
	{"haptic_nv_rtp_silk.bin"},
};

struct transient_trig_data {
	int activate;
	int state;
	int restore_state;
	unsigned long duration;
	struct timer_list timer;
	struct led_classdev *led_cdev;
};

/*********************************************************
 *
 * I2C Read/Write
 *
 *********************************************************/
int haptic_nv_i2c_reads(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			uint8_t *buf, uint32_t len)
{
	int ret = -1;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = aw_haptic->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = aw_haptic->i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
			},
	};

	ret = i2c_transfer(aw_haptic->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_err("transfer failed. reg_addr=0x%02X", reg_addr);
		return ret;
	} else if (ret != AW_I2C_READ_MSG_NUM) {
		aw_err("transfer failed(size error). reg_addr=0x%02X",
		       reg_addr);
		return -ENXIO;
	}

	return ret;
}

int haptic_nv_i2c_writes(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			 uint8_t *buf, uint32_t len)
{
	uint8_t *data = NULL;
	int ret = -1;

	data = kmalloc(len + 1, GFP_KERNEL);
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(aw_haptic->i2c, data, len + 1);
	if (ret < 0)
		aw_err("i2c master send 0x%02X err", reg_addr);
	kfree(data);
	return ret;
}

int haptic_nv_i2c_write_bits(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			     uint32_t mask, uint8_t reg_data)
{
	uint8_t reg_val = 0;
	int ret = -1;

	ret = haptic_nv_i2c_reads(aw_haptic, reg_addr, &reg_val,
				  AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c read error, ret=%d", ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = haptic_nv_i2c_writes(aw_haptic, reg_addr, &reg_val,
				   AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c write error, ret=%d", ret);
		return ret;
	}
	return 0;
}

ssize_t haptic_nv_read_reg_array(struct aw_haptic *aw_haptic, char *buf,
				 ssize_t len, uint8_t head_reg_addr,
				 uint8_t tail_reg_addr)
{
	int reg_num = 0;
	int i = 0;
	uint8_t reg_array[AW_REG_MAX] = {0};

	reg_num = tail_reg_addr - head_reg_addr + 1;
	haptic_nv_i2c_reads(aw_haptic, head_reg_addr, reg_array, reg_num);
	for (i = 0 ; i < reg_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02X=0x%02X\n",
				head_reg_addr + i, reg_array[i]);
	}
	return len;
}

static void pm_qos_enable(struct aw_haptic *aw_haptic, bool enable)
{
#ifdef KERNEL_OVER_5_10
	if (enable) {
		if (!cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_add_request(&aw_haptic->aw_pm_qos_req_vb, CPU_LATENCY_QOC_VALUE);
	} else {
		if (cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#else
	if (enable) {
		if (!pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_add_request(&aw_haptic->aw_pm_qos_req_vb,
					   PM_QOS_CPU_DMA_LATENCY,
					   AW_PM_QOS_VALUE_VB);
	} else {
		if (pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#endif
}

#ifdef AW_CHECK_RAM_DATA
static int parse_ram_data(struct aw_haptic *aw_haptic, uint32_t len,
		   uint8_t *cont_data, uint8_t *ram_data)
{
	int i = 0;

	aw_info("enter");
	for (i = 0; i < len; i++) {
		if (ram_data[i] != cont_data[i]) {
			aw_err("check ramdata error, addr=0x%04X, ram_data=0x%02X, file_data=0x%02X",
				i, ram_data[i], cont_data[i]);
			return -ERANGE;
		}
	}
	return 0;
}

static int check_ram_data(struct aw_haptic *aw_haptic,
			  struct aw_haptic_container *aw_fw)
{
	int i = 0;
	int len = 0;
	int ret = 0;
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};

	aw_haptic->func->set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < aw_fw->len) {
		if ((aw_fw->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = aw_fw->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		aw_haptic->func->get_ram_data(aw_haptic, ram_data, len);
		ret = parse_ram_data(aw_haptic, len, &aw_fw->data[i], ram_data);
		if (ret < 0)
			break;
		i += len;
	}
	return ret;
}
#endif

static void write_ram_data(struct aw_haptic *aw_haptic,
			   struct aw_haptic_container *aw_fw)
{
	int i = 0;
	int len = 0;

	aw_haptic->func->set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < aw_fw->len) {
		if ((aw_fw->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = aw_fw->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		aw_haptic->func->set_ram_data(aw_haptic, &aw_fw->data[i], len);
		i += len;
	}
}

static int parse_dt_gpio(struct aw_haptic *aw_haptic, struct device *dev,
			 struct device_node *np)
{
	aw_haptic->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw_haptic->reset_gpio < 0) {
		aw_err("no reset gpio provide");
		return -EPERM;
	}
	aw_info("reset gpio provide ok %d", aw_haptic->reset_gpio);

	aw_haptic->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw_haptic->irq_gpio < 0) {
		aw_err("no irq gpio provided.");
		aw_haptic->is_used_irq_pin = false;
	} else {
		aw_info("irq gpio provide ok irq = %d.", aw_haptic->irq_gpio);
		aw_haptic->is_used_irq_pin = true;
	}

	return 0;
}

static void parse_dts_i2c_addr(struct aw_haptic *aw_haptic, struct device *dev,
			 struct device_node *np)
{
	uint32_t val = 0;

	val = of_property_read_u32(np, "aw862xx_i2c_addr",
				   &aw_haptic->aw862xx_i2c_addr);
	if (val)
		aw_err("configure aw862xx_i2c_addr error");
	else
		aw_info("configure aw862xx_i2c_addr ok");
}

static void hw_reset(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	if (aw_haptic && gpio_is_valid(aw_haptic->reset_gpio)) {
		gpio_set_value_cansleep(aw_haptic->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(aw_haptic->reset_gpio, 1);
		usleep_range(8000, 8500);
	} else {
		aw_err("failed");
	}
}

static void sw_reset(struct aw_haptic *aw_haptic)
{
	uint8_t reset = AW_BIT_RESET;

	aw_info("enter");
	haptic_nv_i2c_writes(aw_haptic, AW_REG_ID, &reset, AW_I2C_BYTE_ONE);
	usleep_range(2000, 2500);
}

static int read_chipid(struct aw_haptic *aw_haptic, uint8_t *reg_val,
		       uint8_t type)
{
	int cnt = 0;
	int ret = -1;

	while (cnt < AW_I2C_RETRIES) {
		ret = haptic_nv_i2c_reads(aw_haptic, AW_REG_CHIPIDH, reg_val,
					  AW_I2C_BYTE_ONE);
		if (*reg_val == AW8623X_CHIP_ID_H)
			ret = haptic_nv_i2c_reads(aw_haptic, AW_REG_CHIPIDL,
						  reg_val, AW_I2C_BYTE_ONE);
		else
			ret = haptic_nv_i2c_reads(aw_haptic, AW_REG_ID, reg_val,
						  AW_I2C_BYTE_ONE);
		if (ret < 0) {
			if (type == AW_FIRST_TRY)
				aw_info("reading chip id");
			else if (type == AW_LAST_TRY)
				aw_err("i2c_read cnt=%d error=%d", cnt, ret);
			else
				aw_err("type is error");
		} else {
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int parse_chipid(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint8_t reg[2] = { 0 };
	uint8_t cnt = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		/* hardware reset */
		hw_reset(aw_haptic);

		ret = read_chipid(aw_haptic, &reg[0], AW_FIRST_TRY);
		if (ret < 0) {
			aw_haptic->i2c->addr = (u16)aw_haptic->aw862xx_i2c_addr;
			aw_info("try to replace i2c addr [(0x%02X)] to read chip id again",
				aw_haptic->i2c->addr);
			ret = read_chipid(aw_haptic, &reg[0], AW_LAST_TRY);
			if (ret < 0)
				break;
		}

		switch (reg[0]) {
#ifdef AW8624_DRIVER
		case AW8624_CHIP_ID:
			aw_haptic->name = AW8624;
			aw_info("detected aw8624.");
			return 0;
#endif

#ifdef AW8622X_DRIVER
		case AW8622X_CHIP_ID:
			haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_EFRD9,
					    &reg[1], AW_I2C_BYTE_ONE);
			if ((reg[1] & 0x41) == AW86223_EF_ID) {
				aw_haptic->name = AW86223;
				aw_info("aw86223 detected");
				return 0;
			}
			if ((reg[1] & 0x41) == AW86224_EF_ID) {
				aw_haptic->name = AW86224;
				aw_info("aw86224 or aw86225 detected");
				return 0;
			}
			aw_info("unsupported ef_id = (0x%02X)", reg[1]);
			break;
#endif

#ifdef AW86214_DRIVER
		case AW86214_CHIP_ID:
			haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_EFRD9,
					    &reg[1], AW_I2C_BYTE_ONE);
			if ((reg[1] & 0x41) == AW86214_EF_ID) {
				aw_haptic->name = AW86214;
				aw_info("aw86214 detected");
				return 0;
			}
			aw_info("unsupported ef_id = (0x%02X)", reg[1]);
			break;
#endif

#ifdef AW8623X_DRIVER
		case AW86233_CHIP_ID_L:
			aw_haptic->name = AW86233;
			aw_info("aw86233 detected");
			return 0;
		case AW86234_CHIP_ID_L:
			aw_haptic->name = AW86234;
			aw_info("aw86234 detected");
			return 0;
		case AW86235_CHIP_ID_L:
			aw_haptic->name = AW86235;
			aw_info("aw86235 detected");
			return 0;
#endif

		default:
			aw_info("unsupport device revision (0x%02X)", reg[0]);
			break;
		}
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}
	return -EINVAL;
}

static int chip_private_init(struct aw_haptic *aw_haptic)
{
	switch (aw_haptic->name) {
#ifdef AW8624_DRIVER
	case AW8624:
		aw_haptic->is_supported_rtp = true;
		aw_haptic->is_supported_trig = true;
		aw_haptic->dts_name_array = aw8624_dts_name;
		aw_haptic->dts_name_len = ARRAY_SIZE(aw8624_dts_name);
		aw_haptic->ram_file_num = AW8624_RAM_FILE;
		break;
#endif

#ifdef AW8622X_DRIVER
	case AW86223:
	case AW86224:
	case AW86225:
		aw_haptic->is_supported_rtp = true;
		aw_haptic->is_supported_trig = true;
		aw_haptic->dts_name_array = aw8622x_dts_name;
		aw_haptic->dts_name_len = ARRAY_SIZE(aw8622x_dts_name);
		aw_haptic->ram_file_num = AW8622X_RAM_FILE;
		break;
#endif

#ifdef AW86214_DRIVER
	case AW86214:
		aw_haptic->is_supported_rtp = false;
		aw_haptic->is_supported_trig = false;
		aw_haptic->dts_name_array = aw86214_dts_name;
		aw_haptic->dts_name_len = ARRAY_SIZE(aw86214_dts_name);
		aw_haptic->ram_file_num = AW86214_RAM_FILE;
		break;
#endif

#ifdef AW8623X_DRIVER
	case AW86233:
	case AW86234:
	case AW86235:
		aw_haptic->is_supported_rtp = true;
		aw_haptic->is_supported_trig = true;
		aw_haptic->dts_name_array = aw8623x_dts_name;
		aw_haptic->dts_name_len = ARRAY_SIZE(aw8623x_dts_name);
		aw_haptic->ram_file_num = AW8623X_RAM_FILE;
		break;
#endif

	default:
		aw_info("unexpected chip!");
		return -EINVAL;
	}
	return 0;
}

static int func_ptr_init(struct aw_haptic *aw_haptic, struct device *dev)
{
	switch (aw_haptic->name) {
#ifdef AW8624_DRIVER
	case AW8624:
		aw_haptic->func = &aw8624_func_list;
		break;
#endif

#ifdef AW8622X_DRIVER
	case AW86223:
	case AW86224:
	case AW86225:
		aw_haptic->func = &aw862xx_func_list;
		break;
#endif

#ifdef AW86214_DRIVER
	case AW86214:
		aw_haptic->func = &aw862xx_func_list;
		break;
#endif

#ifdef AW8623X_DRIVER
	case AW86233:
	case AW86234:
	case AW86235:
		aw_haptic->func = &aw8623x_func_list;
		break;
#endif

	default:
		aw_info("unexpected chip!");
		return -EINVAL;
	}
	return 0;
}

static int write_rtp_data(struct aw_haptic *aw_haptic)
{
	uint32_t buf_len = 0;
	int ret = -ERANGE;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	if (!aw_rtp) {
		aw_info("aw_rtp is null, break!");
		return ret;
	}

	if (aw_haptic->rtp_cnt < (aw_haptic->ram.base_addr)) {
		if ((aw_rtp->len - aw_haptic->rtp_cnt) <
		    (aw_haptic->ram.base_addr)) {
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		} else {
			if (aw_haptic->rtp_loop)
				buf_len = aw_haptic->ram.base_addr >> 2;
			else
				buf_len = aw_haptic->ram.base_addr;
		}
	} else if ((aw_rtp->len - aw_haptic->rtp_cnt) <
		   (aw_haptic->ram.base_addr >> 2)) {
		buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
	} else {
		buf_len = aw_haptic->ram.base_addr >> 2;
	}

#ifdef AW_ENABLE_RTP_PRINT_LOG
	aw_info("buf_len = %d", buf_len);
#endif

	aw_haptic->func->set_rtp_data(aw_haptic,
				      &aw_rtp->data[aw_haptic->rtp_cnt],
				      buf_len);
	aw_haptic->rtp_cnt += buf_len;
	return 0;

}

static int judge_rtp_load_end(struct aw_haptic *aw_haptic)
{
	uint8_t glb_st = 0;
	int ret = -ERANGE;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;
	struct transient_trig_data *transient_data = led_get_trigger_data(&aw_haptic->vib_dev);

	glb_st = aw_haptic->func->get_glb_state(aw_haptic);

	if ((aw_haptic->rtp_cnt == aw_rtp->len) ||
	    ((glb_st & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY)) {
		if (aw_haptic->rtp_cnt != aw_rtp->len) {
			aw_haptic->rtp_cnt = 0;
			aw_haptic->rtp_init = false;
			if (transient_data) {
				transient_data->activate = 0;
				del_timer(&transient_data->timer);
			}
			aw_err("rtp play suspend!");
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
			ret = 0;
		} else if (aw_haptic->rtp_file_num >= 1 && aw_haptic->rtp_file_num <= 50) {
			aw_haptic->rtp_cnt = 0;
			aw_haptic->rtp_init = true;
			aw_haptic->rtp_loop = true;
			pm_qos_enable(aw_haptic, true);
			aw_info("rtp update loop!");
		} else {
			aw_haptic->rtp_cnt = 0;
			aw_haptic->rtp_init = false;
			if (transient_data) {
				transient_data->activate = 0;
				del_timer(&transient_data->timer);
			}
			aw_info("rtp update complete!,cnt=%d",
				aw_haptic->rtp_cnt);
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
			ret = 0;
		}
	}

	return ret;
}

static void rtp_play(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	while ((!aw_haptic->func->rtp_get_fifo_afs(aw_haptic))
	       && (aw_haptic->play_mode == AW_RTP_MODE)) {

#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_info("rtp cnt = %d", aw_haptic->rtp_cnt);
#endif
		ret = write_rtp_data(aw_haptic);
		if (ret != 0)
			break;
		ret = judge_rtp_load_end(aw_haptic);
		if (ret == 0)
			break;
	}
}

static irqreturn_t irq_handle(int irq, void *data)
{
	int irq_state = 0;
	struct aw_haptic *aw_haptic = data;
	struct transient_trig_data *transient_data = led_get_trigger_data(&aw_haptic->vib_dev);

	aw_info("enter");

	do {
		irq_state = aw_haptic->func->get_irq_state(aw_haptic);

		if (aw_haptic->is_supported_rtp == false)
		return IRQ_HANDLED;

		if (irq_state == AW_IRQ_ALMOST_EMPTY) {
			if (aw_haptic->rtp_init) {
				mutex_lock(&aw_haptic->rtp_lock);
				rtp_play(aw_haptic);
				mutex_unlock(&aw_haptic->rtp_lock);
			} else {
				aw_info("rtp_init: %d", aw_haptic->rtp_init);
				if (transient_data) {
					transient_data->activate = 0;
					del_timer(&transient_data->timer);
				}
			}
		}

		if (aw_haptic->play_mode != AW_RTP_MODE)
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
	} while (irq_state);

	aw_info("exit");
	return IRQ_HANDLED;
}

static int irq_config(struct device *dev, struct aw_haptic *aw_haptic)
{
	int ret = -1;
	int irq_flags = 0;

	aw_info("enter");
	if (gpio_is_valid(aw_haptic->irq_gpio) &&
	    !(aw_haptic->flags & AW_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw_haptic->func->interrupt_setup(aw_haptic);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(dev,
					       gpio_to_irq(aw_haptic->irq_gpio),
					       NULL, irq_handle, irq_flags,
					       "aw_haptic", aw_haptic);
		if (ret != 0) {
			aw_err("failed to request IRQ %d: %d",
			       gpio_to_irq(aw_haptic->irq_gpio), ret);
			return ret;
		}
	} else {
		aw_info("skipping IRQ registration");
		/* disable feature support if gpio was invalid */
		aw_haptic->flags |= AW_FLAG_SKIP_INTERRUPTS;
	}

	return 0;
}

static int get_ram_num(struct aw_haptic *aw_haptic)
{
	uint32_t first_wave_addr = 0;

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init faild, ram_num = 0!");
		return -EPERM;
	}

	mutex_lock(&aw_haptic->lock);
	/* RAMINIT Enable */
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_first_wave_addr(aw_haptic, &first_wave_addr);
	aw_haptic->ram.ram_num =
			(first_wave_addr - aw_haptic->ram.base_addr - 1) / 4;
	aw_info("first wave addr = 0x%04X, ram_num = %d",
		first_wave_addr, aw_haptic->ram.ram_num);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int container_update(struct aw_haptic *aw_haptic,
			    struct aw_haptic_container *aw_fw)
{
	uint32_t shift = 0;
	int ret = 0;

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->ram_init(aw_haptic, true);
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr = (uint32_t)((aw_fw->data[0 + shift] << 8) |
					      (aw_fw->data[1 + shift]));
	aw_info("base_addr = %d", aw_haptic->ram.base_addr);
	if (!aw_haptic->ram.base_addr) {
		aw_haptic->is_supported_rtp = false;
		aw_info("no set FIFO space for rtp play!");
	}
	aw_haptic->func->set_base_addr(aw_haptic);
	aw_haptic->func->set_fifo_addr(aw_haptic);
	aw_haptic->func->get_fifo_addr(aw_haptic);
	write_ram_data(aw_haptic, aw_fw);

#ifdef AW_CHECK_RAM_DATA
	ret = check_ram_data(aw_haptic, aw_fw);
	if (ret)
		aw_err("ram data check sum error");
	else
		aw_info("ram data check sum pass");
#endif

	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	aw_info("exit");
	return ret;
}

static void ram_load(const struct firmware *cont, void *context)
{
	uint16_t check_sum = 0;
	int i = 0;
	int ret = 0;
	struct aw_haptic *aw_haptic = context;
	struct aw_haptic_container *aw_fw;
	uint32_t ram_file_num = aw_haptic->ram_file_num;

#ifdef AW_READ_BIN_FLEXBALLY
	static uint8_t load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif

	if (!cont) {
		aw_err("failed to read %s", aw_ram_name[ram_file_num]);
		release_firmware(cont);

#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw_haptic->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			aw_info("start hrtimer:load_cont%d", load_cont);
		}
#endif

		return;
	}

	aw_info("loaded %s - size: %zu", aw_ram_name[ram_file_num],
		cont ? cont->size : 0);

	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum != (uint16_t)((cont->data[0] << 8) | (cont->data[1]))) {
		aw_err("check sum err: check_sum=0x%04X", check_sum);
		release_firmware(cont);
		return;
	}
	aw_info("check sum pass.");
	aw_haptic->ram.check_sum = check_sum;

	/* aw ram update */
	aw_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw_fw) {
		release_firmware(cont);
		aw_err("Error allocating memory");
		return;
	}
	aw_fw->len = cont->size;
	memcpy(aw_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = container_update(aw_haptic, aw_fw);
	if (ret) {
		aw_err("ram firmware update failed!");
	} else {
		aw_haptic->ram_init = true;
		aw_haptic->ram.len = aw_fw->len - aw_haptic->ram.ram_shift;

		mutex_lock(&aw_haptic->lock);
		if (aw_haptic->is_supported_trig)
			aw_haptic->func->trig_init(aw_haptic);
		mutex_unlock(&aw_haptic->lock);

		aw_info("ram firmware update complete!");
		get_ram_num(aw_haptic);
	}
	kfree(aw_fw);
	aw_fw = NULL;

}

static int ram_update(struct aw_haptic *aw_haptic)
{
	aw_haptic->ram_init = false;
	aw_haptic->rtp_init = false;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw_ram_name[aw_haptic->ram_file_num],
				       aw_haptic->dev, GFP_KERNEL,
				       aw_haptic, ram_load);
}

static void ram_play(struct aw_haptic *aw_haptic)
{
	aw_haptic->func->play_mode(aw_haptic, aw_haptic->activate_mode);
	aw_haptic->func->haptic_start(aw_haptic);
}

#ifdef AW_DURATION_DECIDE_WAVEFORM
static int analyse_duration_range(struct aw_haptic *aw_haptic)
{
	int i = 0;
	int ret = 0;
	int len = 0;
	int *duration_time = NULL;

	aw_info("enter");
	len = ARRAY_SIZE(aw_haptic->info.duration_time);
	duration_time = aw_haptic->info.duration_time;
	if (len < 2) {
		aw_err("duration time range error");
		return -ERANGE;
	}

	for (i = (len - 1); i > 0; i--) {
		if (duration_time[i] > duration_time[i-1])
			continue;
		else
			break;

	}

	if (i > 0) {
		aw_err("duration time range error");
		ret = -ERANGE;
	}

	return ret;
}

static int ram_config(struct aw_haptic *aw_haptic, int duration)
{
	uint8_t wavseq = 0;
	uint8_t wavloop = 0;
	int ret = 0;

	aw_info("enter");
	ret = analyse_duration_range(aw_haptic);
	if (ret < 0)
		return ret;

	if ((duration > 0) && (duration < aw_haptic->info.duration_time[0])) {
		wavseq = 3;
		aw_haptic->activate_mode = AW_RAM_MODE;
	} else if ((duration >= aw_haptic->info.duration_time[0]) &&
		(duration < aw_haptic->info.duration_time[1])) {
		wavseq = 2;
		aw_haptic->activate_mode = AW_RAM_MODE;
	} else if ((duration >= aw_haptic->info.duration_time[1]) &&
		(duration < aw_haptic->info.duration_time[2])) {
		wavseq = 1;
		aw_haptic->activate_mode = AW_RAM_MODE;
	} else if (duration >= aw_haptic->info.duration_time[2]) {
		wavseq = 4;
		wavloop = 15;
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
	} else {
		aw_err("duration time error, duration= %d", duration);
		aw_haptic->activate_mode = AW_NULL;
		return -ERANGE;
	}
	aw_info("duration %d, select index %d", aw_haptic->duration, wavseq);
	aw_haptic->func->set_wav_seq(aw_haptic, 0, wavseq);
	aw_haptic->func->set_wav_loop(aw_haptic, 0, wavloop);
	aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
	aw_haptic->func->set_wav_loop(aw_haptic, 1, 0);
	return 0;
}
#endif

static void upload_lra(struct aw_haptic *aw_haptic, uint8_t flag)
{
	uint8_t reg_val;

	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		reg_val = 0x00;
		break;
	case AW_F0_CALI_LRA:
		aw_info("write f0_cali_data to trim_lra = 0x%02X",
			 aw_haptic->f0_cali_data);
		reg_val = (uint8_t)aw_haptic->f0_cali_data;
		break;
	case AW_OSC_CALI_LRA:
		aw_info("write osc_cali_data to trim_lra = 0x%02X",
			aw_haptic->osc_cali_data);
		reg_val = (uint8_t)aw_haptic->osc_cali_data;
		break;
	default:
		aw_err("flag is error");
		reg_val = 0x00;
		break;
	}
	aw_haptic->func->set_trim_lra(aw_haptic, reg_val);
}

static int judge_within_cali_range(struct aw_haptic *aw_haptic)
{
	uint32_t f0_cali_min = 0;
	uint32_t f0_cali_max = 0;

	aw_info("enter");
	f0_cali_min = aw_haptic->info.f0_pre *
			(100 - aw_haptic->info.f0_cali_percent) / 100;
	f0_cali_max = aw_haptic->info.f0_pre *
			(100 + aw_haptic->info.f0_cali_percent) / 100;

	aw_info("f0_pre = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d",
		aw_haptic->info.f0_pre, f0_cali_min, f0_cali_max,
		aw_haptic->f0);

	if (aw_haptic->f0 < f0_cali_min) {
		aw_err("lra f0 is too small, lra_f0 = %d!", aw_haptic->f0);
		aw_haptic->f0_cali_data = AW_F0_CALI_DATA_MIN;
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		return -ERANGE;
	}

	if (aw_haptic->f0 > f0_cali_max) {
		aw_err("lra f0 is too large, lra_f0 = %d!", aw_haptic->f0);
		aw_haptic->f0_cali_data = AW_F0_CALI_DATA_MAX;
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		return -ERANGE;
	}
	return 0;
}

static int f0_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");
	upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->get_f0(aw_haptic)) {
		aw_err("get f0 failed");
	} else {
		/* max and min limit */
		ret = judge_within_cali_range(aw_haptic);
		if (ret < 0)
			return -ERANGE;
		/* calculate cali step */
		aw_haptic->func->calculate_cali_data(aw_haptic);
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	aw_haptic->func->play_stop(aw_haptic);
	return ret;
}

static void ram_vbat_comp(struct aw_haptic *aw_haptic, bool flag)
{
	int temp_gain = 0;

	aw_info("enter");
	if (flag) {
		if (aw_haptic->ram_vbat_comp == AW_RAM_VBAT_COMP_ENABLE) {
			aw_haptic->func->get_vbat(aw_haptic);
			temp_gain = aw_haptic->gain * AW_VBAT_REFER /
				    aw_haptic->vbat;
			if (temp_gain >
			    (AW_DEFAULT_GAIN * AW_VBAT_REFER / AW_VBAT_MIN)) {
				temp_gain = AW_DEFAULT_GAIN * AW_VBAT_REFER /
					    AW_VBAT_MIN;
				aw_info("gain limit=%d", temp_gain);
			}
			aw_haptic->func->set_gain(aw_haptic, temp_gain);
			aw_info("ram vbat comp open, set_gain 0x%02X",
				temp_gain);
		} else {
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
			aw_info("ram vbat comp close, set_gain 0x%02X",
				aw_haptic->gain);
		}
	} else {
		aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		aw_info("ram vbat comp close, set_gain 0x%02X",
			aw_haptic->gain);
	}
}

static int osc_calculation_time(struct aw_haptic *aw_haptic)
{
	uint32_t base_addr = aw_haptic->ram.base_addr;
	uint32_t buf_len = 0;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	if (!aw_haptic->func->rtp_get_fifo_afs(aw_haptic)) {
		aw_dbg("haptic_rtp_get_fifo_afi, rtp_cnt= %d",
			aw_haptic->rtp_cnt);

		mutex_lock(&aw_haptic->rtp_lock);
		pm_qos_enable(aw_haptic, true);
		if ((aw_rtp->len - aw_haptic->rtp_cnt) < (base_addr >> 2))
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		else
			buf_len = (base_addr >> 2);

		if (aw_haptic->rtp_cnt != aw_rtp->len) {
			if (aw_haptic->timeval_flags == 1) {
				aw_haptic->kstart = ktime_get();
				aw_haptic->timeval_flags = 0;
			}
			aw_haptic->func->set_rtp_data(aw_haptic,
				    &aw_rtp->data[aw_haptic->rtp_cnt], buf_len);
			aw_haptic->rtp_cnt += buf_len;
		}
		pm_qos_enable(aw_haptic, false);
		mutex_unlock(&aw_haptic->rtp_lock);

	}

	if (aw_haptic->func->get_osc_status(aw_haptic)) {
		aw_haptic->kend = ktime_get();
		aw_info("osc trim playback done aw_haptic->rtp_cnt= %d",
			aw_haptic->rtp_cnt);
		return 0;
	}
	aw_haptic->kend = ktime_get();
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend,
						aw_haptic->kstart));
	if (aw_haptic->microsecond > AW_OSC_CALI_MAX_LENGTH) {
		aw_info("osc trim time out! aw_haptic->rtp_cnt %d",
			aw_haptic->rtp_cnt);
		return 0;
	}
	return -ERANGE;
}

static int rtp_fw_load(struct aw_haptic *aw_haptic, uint32_t file_num)
{
	int ret = 0;
	const struct firmware *rtp_file;

	ret = request_firmware(&rtp_file, aw_rtp_name[file_num],
			       aw_haptic->dev);
	if (ret < 0) {
		aw_err("failed to read %s", aw_rtp_name[file_num]);
		return ret;
	}

	vfree(aw_haptic->aw_rtp);
	aw_haptic->aw_rtp = NULL;
	aw_haptic->aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_haptic->aw_rtp) {
		release_firmware(rtp_file);
		aw_err("error allocating memory");
		return -ENOMEM;
	}
	aw_haptic->aw_rtp->len = rtp_file->size;
	aw_haptic->rtp_len = rtp_file->size;
	aw_info("rtp file:[%s] size = %dbytes", aw_rtp_name[file_num],
		aw_haptic->aw_rtp->len);
	memcpy(aw_haptic->aw_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	return 0;
}

static int rtp_osc_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");
	/*aw_haptic add stop,for irq interrupt during calibrate */
	aw_haptic->func->play_stop(aw_haptic);

	mutex_lock(&aw_haptic->rtp_lock);
	ret = rtp_fw_load(aw_haptic, 0);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->rtp_lock);
		return ret;
	}
	mutex_unlock(&aw_haptic->rtp_lock);

	aw_haptic->rtp_init = false;
	aw_haptic->rtp_cnt = 0;
	aw_haptic->timeval_flags = 1;

	ram_vbat_comp(aw_haptic, false);
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	disable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->func->haptic_start(aw_haptic);
	while (1) {
		ret = osc_calculation_time(aw_haptic);
		if (!ret)
			break;
	}
	enable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend,
						       aw_haptic->kstart));
	/*calibration osc */
	aw_info("microsecond: %llu", aw_haptic->microsecond);
	return 0;
}

static void rtp_trim_lra_cali(struct aw_haptic *aw_haptic)
{
	uint32_t lra_trim_code = 0;
	/*0.1 percent below no need to calibrate */
	uint32_t osc_cali_threshold = 10;
	uint32_t code = 0;
	unsigned long real_time = 0;
	unsigned long theory_time = 0;

	upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (rtp_osc_cali(aw_haptic))
		return;
	real_time = aw_haptic->microsecond;
	theory_time = aw_haptic->func->get_theory_time(aw_haptic);
	if (theory_time == real_time) {
		aw_info("theory_time == real_time: %ld, no need to calibrate!",
			real_time);
		return;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) >
		    (theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(real_time - theory_time) > (theory_time/50), can't calibrate!");
			return;
		}

		if ((real_time - theory_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %ld, theory_time: %ld, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		code = ((real_time - theory_time) * 4) / (theory_time / 1000);
		code = ((code % 10 < 5) ? 0 : 1) + code / 10;
		code = 32 + code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) >
		    (theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(theory_time - real_time) > (theory_time / 50), can't calibrate!");
			return;
		}
		if ((theory_time - real_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %ld, theory_time: %ld, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		code = ((theory_time - real_time) * 4) / (theory_time / 1000);
		code = ((code % 10 < 5) ? 0 : 1) + code / 10;
		code = 32 - code;
	}
	if (code > 31)
		lra_trim_code = code - 32;
	else
		lra_trim_code = code + 32;

	aw_haptic->osc_cali_data = lra_trim_code;
	upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_info("real_time: %ld, theory_time: %ld", real_time, theory_time);
	aw_info("code: %d, trim_lra: 0x%02X", code, lra_trim_code);
}

static int parse_awrw_data(struct aw_haptic *aw_haptic, const char *buf)
{
	uint8_t i = 0;
	uint8_t reg_num = aw_haptic->i2c_info.reg_num;
	const char *temp_buf = NULL;
	char data_buf[AWRW_CMD_UNIT] = { 0 };
	uint32_t value = 0;
	int ret = 0;
	int len = strlen(buf) - (AWRW_CMD_UNIT * 3);

	if (len < 0) {
		aw_err("parse data error");
		return -ERANGE;
	}
	temp_buf = &buf[AWRW_CMD_UNIT * 3];
	for (i = 0; i < reg_num; i++) {
		if (((i + 1) * AWRW_CMD_UNIT) > len) {
			aw_err("parse data error");
			return -ERANGE;
		}
		memcpy(data_buf, &temp_buf[i * AWRW_CMD_UNIT], 4);
		data_buf[4] = '\0';
		ret = kstrtouint(data_buf, 0, &value);
		if (ret < 0) {
			aw_err("kstrtouint error");
			return ret;
		}
		aw_haptic->i2c_info.reg_data[i] = (uint8_t)value;
	}
	return 0;
}

static int haptic_audio_ctr_list_insert(struct aw_haptic *aw_haptic,
					struct aw_haptic_ctr *haptic_ctr)
{
	struct aw_haptic_ctr *p_new = NULL;
	struct aw_haptic_audio *haptic_audio = &aw_haptic->haptic_audio;

	aw_info("enter");
	p_new = kzalloc(sizeof(struct aw_haptic_ctr), GFP_KERNEL);
	if (p_new == NULL)
		return -ENOMEM;

	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;
	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));
	aw_info("exit");
	return 0;
}

static void haptic_audio_ctrl_list_clr(struct aw_haptic_audio *haptic_audio)
{
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
		p_ctr = NULL;
	}
}

static void haptic_audio_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
}

static void haptic_audio_off(struct aw_haptic *aw_haptic)
{
	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, 0x80);
	aw_haptic->func->play_stop(aw_haptic);
	haptic_audio_ctrl_list_clr(&aw_haptic->haptic_audio);
	mutex_unlock(&aw_haptic->lock);
}

static void haptic_ctr_list_config(struct aw_haptic *aw_haptic,
				   struct aw_haptic_ctr *p_ctr,
				   struct aw_haptic_ctr *p_ctr_bak,
				   struct aw_haptic_audio *haptic_audio)
{
	uint32_t list_input_cnt = 0;
	uint32_t list_output_cnt = 0;
	uint32_t list_diff_cnt = 0;
	uint32_t list_del_cnt = 0;

	list_for_each_entry_safe(p_ctr, p_ctr_bak,
			&(haptic_audio->ctr_list), list) {
		list_input_cnt =  p_ctr->cnt;
		break;
	}
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
			&(haptic_audio->ctr_list), list) {
		list_output_cnt =  p_ctr->cnt;
		break;
	}

	if (list_input_cnt > list_output_cnt)
		list_diff_cnt = list_input_cnt - list_output_cnt;

	if (list_input_cnt < list_output_cnt)
		list_diff_cnt = 32 + list_input_cnt - list_output_cnt;

	if (list_diff_cnt > 2) {
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
				&(haptic_audio->ctr_list), list) {
			if ((p_ctr->play == 0) &&
			    (AW_CMD_ENABLE == (AW_CMD_ENABLE & p_ctr->cmd))) {
				list_del(&p_ctr->list);
				kfree(p_ctr);
				p_ctr = NULL;
				list_del_cnt++;
			}
			if (list_del_cnt == list_diff_cnt)
				break;
		}
	}
}

static void parse_haptic_audio_data(struct aw_haptic *aw_haptic)
{
	bool ctr_list_flag = false;
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;
	struct aw_haptic_audio *haptic_audio = &(aw_haptic->haptic_audio);

	memset(&(haptic_audio->ctr), 0, sizeof(struct aw_haptic_ctr));
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		ctr_list_flag = true;
		break;
	}

	if (ctr_list_flag == false)
		aw_dbg("ctr list empty");
	if (ctr_list_flag == true)
		haptic_ctr_list_config(aw_haptic, p_ctr, p_ctr_bak,
				       haptic_audio);

	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
		&(haptic_audio->ctr_list), list) {
		haptic_audio->ctr.cnt = p_ctr->cnt;
		haptic_audio->ctr.cmd = p_ctr->cmd;
		haptic_audio->ctr.play = p_ctr->play;
		haptic_audio->ctr.wavseq = p_ctr->wavseq;
		haptic_audio->ctr.loop = p_ctr->loop;
		haptic_audio->ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		p_ctr = NULL;
		break;
	}

	if (haptic_audio->ctr.play) {
		aw_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d",
			haptic_audio->ctr.cnt, haptic_audio->ctr.cmd,
			haptic_audio->ctr.play, haptic_audio->ctr.wavseq,
			haptic_audio->ctr.loop, haptic_audio->ctr.gain);
	}

}

static void haptic_audio_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   haptic_audio.work);

	int rtp_is_going_on = 0;

	aw_dbg("enter");

	mutex_lock(&aw_haptic->haptic_audio.lock);
	parse_haptic_audio_data(aw_haptic);
	/* rtp mode jump */
	rtp_is_going_on = aw_haptic->func->judge_rtp_going(aw_haptic);
	if (rtp_is_going_on) {
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		return;
	}
	mutex_unlock(&aw_haptic->haptic_audio.lock);

	if ((AW_CMD_HAPTIC & aw_haptic->haptic_audio.ctr.cmd) ==
	     AW_CMD_ENABLE) {
		if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_ENABLE) {
			aw_info("haptic audio play start");

			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			aw_haptic->func->play_mode(aw_haptic, AW_RAM_MODE);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00,
					aw_haptic->haptic_audio.ctr.wavseq);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00,
					aw_haptic->haptic_audio.ctr.loop);
			aw_haptic->func->set_gain(aw_haptic,
					aw_haptic->haptic_audio.ctr.gain);
			aw_haptic->func->haptic_start(aw_haptic);
			mutex_unlock(&aw_haptic->lock);

		} else if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_STOP) {

			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);

		} else if (aw_haptic->haptic_audio.ctr.play == AW_PLAY_GAIN) {

			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->set_gain(aw_haptic,
					aw_haptic->haptic_audio.ctr.gain);
			mutex_unlock(&aw_haptic->lock);
		}
	}
}

static enum hrtimer_restart haptic_audio_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic,
						   haptic_audio.timer);
	int time = aw_haptic->haptic_audio.timer_val;

	schedule_work(&aw_haptic->haptic_audio.work);
	hrtimer_start(&aw_haptic->haptic_audio.timer,
		      ktime_set(time / 1000000, (time % 1000000) * 1000),
		      HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->state);
}

static ssize_t state_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	return count;
}

static ssize_t duration_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t duration_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw_haptic->duration = val;
	aw_info("duration=%d", aw_haptic->duration);
	return count;
}

static ssize_t activate_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t activate_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int rc = 0;
	uint32_t val = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	if (val != 0 && val != 1)
		return count;

	aw_info("value=%d", val);

	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->state = val;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	mutex_unlock(&aw_haptic->lock);

	schedule_work(&aw_haptic->vibrator_work);
	return count;
}

static ssize_t activate_mode_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->activate_mode);
}

static ssize_t activate_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t index_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	aw_haptic->func->get_wav_seq(aw_haptic, 1);
	aw_haptic->index = aw_haptic->seq[0];
	return snprintf(buf, PAGE_SIZE, "index = %d\n", aw_haptic->index);
}

static ssize_t index_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;

	if (val > aw_haptic->ram.ram_num) {
		aw_err("input value out of range!");
		return count;
	}

	aw_info("value=%d", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->index = val;
	aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t gain_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint8_t gain = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_gain(aw_haptic, &gain);
	mutex_unlock(&aw_haptic->lock);

	return snprintf(buf, PAGE_SIZE, "gain = 0x%02X\n", gain);
}

static ssize_t gain_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw_info("value=0x%02X", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gain = val;
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t seq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	uint8_t i = 0;
	size_t count = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, AW_SEQUENCER_SIZE);
	for (i = 0; i < AW_SEQUENCER_SIZE; i++)
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d = %d\n", i + 1, aw_haptic->seq[i]);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t seq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%X %X", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= AW_SEQUENCER_SIZE ||
		    databuf[1] > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		aw_info("seq%d=0x%02X", databuf[0], databuf[1]);

		mutex_lock(&aw_haptic->lock);
		aw_haptic->seq[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_seq(aw_haptic, (uint8_t)databuf[0],
					     aw_haptic->seq[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t loop_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	uint8_t i = 0;
	size_t count = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	uint8_t reg_val[AW_SEQUENCER_LOOP_SIZE] = {0};
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_loop(aw_haptic, reg_val);
	for (i = 0; i < AW_SEQUENCER_LOOP_SIZE; i++) {
		aw_haptic->loop[i * 2 + 0] = (reg_val[i] >> 4) & 0x0F;
		aw_haptic->loop[i * 2 + 1] = (reg_val[i] >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02X\n", i * 2 + 1,
				  aw_haptic->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02X\n", i * 2 + 2,
				  aw_haptic->loop[i * 2 + 1]);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t loop_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_info("seq%d loop=0x%02X", databuf[0], databuf[1]);

		mutex_lock(&aw_haptic->lock);
		aw_haptic->loop[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_loop(aw_haptic, (uint8_t)databuf[0],
					      aw_haptic->loop[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	len = aw_haptic->func->get_reg(aw_haptic, len, buf);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	uint8_t val = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%X %X", &databuf[0], &databuf[1]) == 2) {
		val = (uint8_t)databuf[1];

		mutex_lock(&aw_haptic->lock);
		haptic_nv_i2c_writes(aw_haptic, (uint8_t)databuf[0], &val,
				     AW_I2C_BYTE_ONE);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t ram_update_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	int i = 0;
	int j = 0;
	int size = 0;
	ssize_t len = 0;
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
	uint8_t buffer[AW_RAMDATA_SHOW_LINE_BUFFER_SZIE] = {0};
	uint8_t unit[AW_RAMDATA_SHOW_UINT_SIZE] = {0};

	mutex_lock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "Please cat log\n");
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_ram_addr(aw_haptic);
	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;

		aw_haptic->func->get_ram_data(aw_haptic, ram_data, size);
		for (j = 0; j < size; j++) {
			sprintf(unit, "0x%02X ", ram_data[j]);
			strcat(buffer, unit);
			if ((j + 1) % AW_RAMDATA_SHOW_COLUMN == 0 || j == size - 1) {
				aw_info("[ram_data]:%s", buffer);
				memset(buffer, 0,
				       AW_RAMDATA_SHOW_LINE_BUFFER_SZIE);
			}
		}
		i += size;
	}
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t ram_update_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
		ram_update(aw_haptic);
	return count;
}

static ssize_t f0_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->get_f0(aw_haptic);
	upload_lra(aw_haptic, AW_F0_CALI_LRA);
	mutex_unlock(&aw_haptic->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", aw_haptic->f0);
	return len;
}

static ssize_t f0_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	return count;
}


static ssize_t cali_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", aw_haptic->f0);
	return len;
}

static ssize_t cali_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {

		mutex_lock(&aw_haptic->lock);
		f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}


static ssize_t f0_save_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 0x%02X\n",
			aw_haptic->f0_cali_data);

	return len;
}

static ssize_t f0_save_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;
	aw_haptic->f0_cali_data = val;
	return count;
}

static ssize_t cont_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->read_cont_f0(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", aw_haptic->cont_f0);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t cont_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (val)
		aw_haptic->func->cont_config(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	return count;
}


static ssize_t vbat_monitor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_vbat(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_monitor = %d\n",
			aw_haptic->vbat);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t vbat_monitor_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	return count;
}


static ssize_t lra_resistance_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_lra_resistance(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %d\n",
			aw_haptic->lra);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t lra_resistance_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	return count;
}

static ssize_t prctmode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;
	uint8_t reg_val = 0;

	mutex_lock(&aw_haptic->lock);
	reg_val = aw_haptic->func->get_prctmode(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode = %d\n", reg_val);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t prctmode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };
	uint32_t prtime = 0;
	uint32_t prlvl = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		prtime = databuf[0];
		prlvl = databuf[1];

		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->protect_config(aw_haptic, prtime, prlvl);
		mutex_unlock(&aw_haptic->lock);

	}
	return count;
}

static ssize_t ram_vbat_comp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"ram_vbat_comp = %d\n",
			aw_haptic->ram_vbat_comp);

	return len;
}

static ssize_t ram_vbat_comp_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	if (val)
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	else
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw_haptic->lock);

	return count;
}


static ssize_t ram_num_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	get_ram_num(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"ram_num = %d\n", aw_haptic->ram.ram_num);
	return len;
}


static ssize_t awrw_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint8_t reg_num = aw_haptic->i2c_info.reg_num;
	uint8_t flag = aw_haptic->i2c_info.flag;
	uint8_t *reg_data = aw_haptic->i2c_info.reg_data;
	uint8_t i = 0;
	ssize_t len = 0;

	if (!reg_num) {
		aw_err("awrw parameter error");
		return len;
	}
	if (flag == AW_READ) {
		for (i = 0; i < reg_num; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"0x%02X,", reg_data[i]);
		}

		len += snprintf(buf + len - 1, PAGE_SIZE - len, "\n");
	}

	return len;
}

static ssize_t awrw_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	uint32_t datatype[3] = { 0 };
	int ret = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x %x", &datatype[0], &datatype[1],
				    &datatype[2]) == 3) {
		if (!datatype[1]) {
			aw_err("awrw parameter error");
			return count;
		}
		aw_haptic->i2c_info.flag = (uint8_t)datatype[0];
		aw_haptic->i2c_info.reg_num = (uint8_t)datatype[1];
		aw_haptic->i2c_info.first_addr = (uint8_t)datatype[2];

		mutex_lock(&aw_haptic->lock);
		if (aw_haptic->i2c_info.flag == AW_WRITE) {
			ret = parse_awrw_data(aw_haptic, buf);
			if (ret < 0) {
				mutex_unlock(&aw_haptic->lock);
				return count;
			}
			haptic_nv_i2c_writes(aw_haptic,
					     aw_haptic->i2c_info.first_addr,
					     aw_haptic->i2c_info.reg_data,
					     aw_haptic->i2c_info.reg_num);
		} else if (aw_haptic->i2c_info.flag == AW_READ) {
			haptic_nv_i2c_reads(aw_haptic,
					    aw_haptic->i2c_info.first_addr,
					    aw_haptic->i2c_info.reg_data,
					    aw_haptic->i2c_info.reg_num);
		} else {
			aw_err("flag is error");
		}
		mutex_unlock(&aw_haptic->lock);

	} else
		aw_err("missing number of parameters");

	return count;
}

static ssize_t gun_type_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", aw_haptic->gun_type);
}

static ssize_t gun_type_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gun_type = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t bullet_nr_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", aw_haptic->bullet_nr);
}

static ssize_t bullet_nr_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t haptic_audio_time_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.delay_val=%dus\n",
			aw_haptic->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.timer_val=%dus\n",
			aw_haptic->haptic_audio.timer_val);
	return len;
}

static ssize_t haptic_audio_time_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0 };

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->haptic_audio.delay_val = databuf[0];
		aw_haptic->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static ssize_t haptic_audio_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n",
			aw_haptic->haptic_audio.ctr.cnt);
	return len;
}

static int haptic_audio_config(struct aw_haptic *aw_haptic, uint32_t databuf[])
{
	struct aw_haptic_ctr *hap_ctr = NULL;
	int delay = aw_haptic->haptic_audio.delay_val;

	aw_info("enter");
	hap_ctr = kzalloc(sizeof(struct aw_haptic_ctr), GFP_KERNEL);
	if (hap_ctr == NULL)
		return -ENOMEM;

	mutex_lock(&aw_haptic->haptic_audio.lock);
	hap_ctr->cnt = (uint8_t)databuf[0];
	hap_ctr->cmd = (uint8_t)databuf[1];
	hap_ctr->play = (uint8_t)databuf[2];
	hap_ctr->wavseq = (uint8_t)databuf[3];
	hap_ctr->loop = (uint8_t)databuf[4];
	hap_ctr->gain = (uint8_t)databuf[5];

	haptic_audio_ctr_list_insert(aw_haptic, hap_ctr);

	if (hap_ctr->cmd == 0xff) {
		aw_info("haptic_audio stop");
		if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
			aw_info("cancel haptic_audio_timer");
			hrtimer_cancel(&aw_haptic->haptic_audio.timer);
			aw_haptic->haptic_audio.ctr.cnt = 0;
			haptic_audio_off(aw_haptic);
		}
	} else {
		if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
			/* */
		} else {
			aw_info("start haptic_audio_timer");
			haptic_audio_init(aw_haptic);
			hrtimer_start(&aw_haptic->haptic_audio.timer,
				      ktime_set(delay / 1000000,
						(delay % 1000000) * 1000),
						HRTIMER_MODE_REL);
		}
	}
	kfree(hap_ctr);
	mutex_unlock(&aw_haptic->haptic_audio.lock);
	return 0;
}

static ssize_t haptic_audio_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[6] = { 0 };
	int rtp_is_going_on = 0;

	rtp_is_going_on = aw_haptic->func->judge_rtp_going(aw_haptic);

	if (rtp_is_going_on) {
		aw_info("RTP is runing, stop audio haptic");
		return count;
	}

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}

	if (sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5]) == 6) {
		if (databuf[2]) {
			aw_info("cnt=%d, cmd=%d, play=%d",
				databuf[0], databuf[1], databuf[2]);
			aw_info("wavseq=%d, loop=%d, gain=%d",
				databuf[3], databuf[4], databuf[5]);
			haptic_audio_config(aw_haptic, databuf);
		}
	}
	return count;
}

static DEVICE_ATTR(aw_state, 0664, state_show, state_store);
static DEVICE_ATTR(reg, 0664, reg_show, reg_store);
static DEVICE_ATTR(aw_duration, 0664, duration_show, duration_store);
static DEVICE_ATTR(aw_activate, 0644, activate_show, activate_store);
static DEVICE_ATTR(activate_mode, 0644, activate_mode_show,
		   activate_mode_store);
static DEVICE_ATTR(index, 0644, index_show, index_store);
static DEVICE_ATTR(gain, 0644, gain_show, gain_store);
static DEVICE_ATTR(seq, 0644, seq_show, seq_store);
static DEVICE_ATTR(loop, 0644, loop_show, loop_store);
static DEVICE_ATTR(ram_update, 0644, ram_update_show, ram_update_store);
static DEVICE_ATTR(f0, 0644, f0_show, f0_store);
static DEVICE_ATTR(cali, 0644, cali_show, cali_store);
static DEVICE_ATTR(f0_save, 0644, f0_save_show, f0_save_store);
static DEVICE_ATTR(cont, 0644, cont_show, cont_store);
static DEVICE_ATTR(vbat_monitor, 0644, vbat_monitor_show, vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, 0644, lra_resistance_show,
		   lra_resistance_store);
static DEVICE_ATTR(prctmode, 0644, prctmode_show, prctmode_store);
static DEVICE_ATTR(ram_vbat_comp, 0644, ram_vbat_comp_show,
		   ram_vbat_comp_store);
static DEVICE_ATTR(ram_num, 0664, ram_num_show, NULL);
static DEVICE_ATTR(awrw, 0664, awrw_show, awrw_store);
static DEVICE_ATTR(gun_type, 0664, gun_type_show, gun_type_store);
static DEVICE_ATTR(bullet_nr, 0664, bullet_nr_show, bullet_nr_store);
static DEVICE_ATTR(haptic_audio_time, 0664, haptic_audio_time_show,
		   haptic_audio_time_store);
static DEVICE_ATTR(haptic_audio, 0664, haptic_audio_show, haptic_audio_store);
static struct attribute *vibrator_attributes[] = {
	&dev_attr_aw_state.attr,
	&dev_attr_aw_duration.attr,
	&dev_attr_aw_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_reg.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_f0_save.attr,
	&dev_attr_cont.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_ram_num.attr,
	&dev_attr_awrw.attr,
	&dev_attr_gun_type.attr,
	&dev_attr_bullet_nr.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_haptic_audio.attr,
	NULL
};

static struct attribute_group vibrator_attribute_group = {
	.attrs = vibrator_attributes
};

static ssize_t rtp_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %d",
			aw_haptic->rtp_cnt);
	return len;
}

static ssize_t rtp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	uint32_t rtp_num_max = sizeof(aw_rtp_name) / AW_NAME_MAX;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_err("kstrtouint fail");
		return count;
	}

	if (!aw_haptic->ram.base_addr) {
		aw_err("no set FIFO space for rtp play!");
		return count;
	}

	mutex_lock(&aw_haptic->lock);
	if ((val > 0) && (val < rtp_num_max)) {
		aw_haptic->state = 1;
		aw_haptic->rtp_file_num = val;
	} else if (val == 0) {
		aw_haptic->state = 0;
	} else {
		aw_haptic->state = 0;
		aw_err("input number error:%d", val);
	}
	mutex_unlock(&aw_haptic->lock);

	schedule_work(&aw_haptic->rtp_work);
	return count;
}

static ssize_t osc_cali_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_cali_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	if (val == 1) {
		rtp_trim_lra_cali(aw_haptic);
	} else if (val == 2) {
		upload_lra(aw_haptic, AW_OSC_CALI_LRA);
		rtp_osc_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t osc_save_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_save_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return count;
	aw_haptic->osc_cali_data = val;
	return count;
}

static DEVICE_ATTR(rtp, 0664, rtp_show, rtp_store);
static DEVICE_ATTR(osc_cali, 0664, osc_cali_show, osc_cali_store);
static DEVICE_ATTR(osc_save, 0664, osc_save_show, osc_save_store);

static struct attribute *rtp_attributes[] = {
	&dev_attr_rtp.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_osc_save.attr,
	NULL
};

static struct attribute_group rtp_attribute_group = {
	.attrs = rtp_attributes
};

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic,
						   timer);
	aw_info("enter");
	aw_haptic->state = 0;
	schedule_work(&aw_haptic->vibrator_work);
	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   vibrator_work);

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
#ifdef AW_DURATION_DECIDE_WAVEFORM
		if (ram_config(aw_haptic, aw_haptic->duration) < 0) {
			mutex_unlock(&aw_haptic->lock);
			return;
		}
#endif
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			ram_play(aw_haptic);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			ram_play(aw_haptic);
#if 0
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
#endif
		} else if (aw_haptic->activate_mode == AW_CONT_MODE) {
			aw_haptic->func->cont_config(aw_haptic);
#if 0
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
#endif
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int wait_enter_rtp_mode(struct aw_haptic *aw_haptic, int cnt)
{
	bool rtp_work_flag = false;
	uint8_t ret = 0;

	while (cnt) {
		ret = aw_haptic->func->judge_rtp_going(aw_haptic);
		if (ret) {
			rtp_work_flag = true;
			aw_info("RTP_GO!");
			break;
		}
		cnt--;
		aw_info("wait for RTP_GO, glb_state=0x%02X", ret);
		usleep_range(AW_RTP_DELAY_MIN, AW_RTP_DELAY_MAX);
	}
	if (!rtp_work_flag) {
		aw_haptic->func->play_stop(aw_haptic);
		aw_err("failed to enter RTP_GO status!");
		return -ERANGE;
	}
	return 0;
}

static void rtp_work_routine(struct work_struct *work)
{
	int ret = 0;
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   rtp_work);

	aw_info("enter");

	if (aw_haptic->is_supported_rtp == false)
		return;

	mutex_lock(&aw_haptic->lock);
	if (hrtimer_active(&aw_haptic->timer))
		hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->irq_clear(aw_haptic);
	if (!aw_haptic->state) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);

	/* fw loaded */
	mutex_lock(&aw_haptic->rtp_lock);
	ret = rtp_fw_load(aw_haptic, aw_haptic->rtp_file_num);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
	mutex_unlock(&aw_haptic->rtp_lock);

	/* rtp config */
	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_init = true;
	upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	aw_haptic->func->haptic_start(aw_haptic);
	usleep_range(AW_RTP_DELAY_MIN, AW_RTP_DELAY_MAX);
	ret = wait_enter_rtp_mode(aw_haptic, 200);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	aw_haptic->rtp_cnt = 0;
	rtp_play(aw_haptic);
	if (aw_haptic->play_mode == AW_RTP_MODE)
		aw_haptic->func->set_rtp_aei(aw_haptic, true);
	pm_qos_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->rtp_lock);
}

#ifdef TIMED_OUTPUT
static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw_haptic *aw_haptic = container_of(dev, struct aw_haptic,
						   vib_dev);

	if (hrtimer_active(&aw_haptic->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw_haptic->timer);

		return ktime_to_ms(r);
	}
	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw_haptic *aw_haptic = container_of(dev, struct aw_haptic,
						   vib_dev);

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return;
	}

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (value > 0) {
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		ram_vbat_comp(aw_haptic, false);
		aw_haptic->func->play_mode(aw_haptic, AW_RAM_MODE);
		aw_haptic->func->haptic_start(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	aw_info("exit");
}
#else
static enum led_brightness brightness_get(struct led_classdev *cdev)
{
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return aw_haptic->amplitude;
}

static void brightness_set(struct led_classdev *cdev, enum led_brightness level)
{
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	struct transient_trig_data *transient_data = led_get_trigger_data(cdev);
	uint64_t duration = 0;

	aw_info("enter");
	if (!transient_data)
		return;
	del_timer(&transient_data->timer);
	if (level == 0) {
		aw_haptic->state = 0;
		pm_qos_enable(aw_haptic, false);
		schedule_work(&aw_haptic->vibrator_work);
		return;
	}
	if (!aw_haptic->ram_init) {
		transient_data->activate = 0;
		aw_err("ram init failed, not allow to play!");
		return;
	}
	duration = transient_data->duration;
	aw_info("duration is %llu", duration);
	if (duration == 0) {
		transient_data->activate = 0;
		aw_info("duration is 0, no work");
		return;
	} else if (duration > 100 && duration <= 120) {
		if (duration - 100 > aw_haptic->ram.ram_num) {
			transient_data->activate = 0;
			aw_err("waveform out of range!");
			return;
		}
		mutex_lock(&aw_haptic->lock);
		aw_haptic->state = level;
		aw_haptic->activate_mode = AW_RAM_MODE;
		aw_haptic->func->set_wav_seq(aw_haptic, 0, duration - 100);
		aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
		aw_haptic->func->set_wav_loop(aw_haptic, 0, 0);
		mutex_unlock(&aw_haptic->lock);
		schedule_work(&aw_haptic->vibrator_work);
	} else if (duration > 5000000 && duration <= 5000050) {
		if ((duration - 5000000) >= (sizeof(aw_rtp_name) / AW_NAME_MAX)) {
			transient_data->activate = 0;
			aw_err("rtp file out of range!");
			return;
		}
		mutex_lock(&aw_haptic->lock);
		aw_haptic->state = level;
		aw_haptic->rtp_file_num = duration - 5000000;
		aw_info("aw_rtp_name[%d]: %s", aw_haptic->rtp_file_num, aw_rtp_name[aw_haptic->rtp_file_num]);
		mutex_unlock(&aw_haptic->lock);
		schedule_work(&aw_haptic->rtp_work);
	} else {
		mutex_lock(&aw_haptic->lock);
		aw_haptic->state = level;
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
		aw_haptic->func->set_wav_seq(aw_haptic, 0, 4);
		aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
		aw_haptic->func->set_wav_loop(aw_haptic, 0, 0x0F);
		mutex_unlock(&aw_haptic->lock);
		schedule_work(&aw_haptic->vibrator_work);
	}
}
#endif

static int vibrator_init(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");

#ifdef TIMED_OUTPUT
	aw_info("TIMED_OUT FRAMEWORK!");
	aw_haptic->vib_dev.name = "vibrator";
	aw_haptic->vib_dev.get_time = vibrator_get_time;
	aw_haptic->vib_dev.enable = vibrator_enable;

	ret = timed_output_dev_register(&(aw_haptic->vib_dev));
	if (ret < 0) {
		aw_err("fail to create timed output dev");
		return ret;
	}
#else
	aw_info("loaded in leds_cdev framework!");
#ifdef KERNEL_OVER_5_10
	aw_haptic->vib_dev.name = "vibrator";
#else
	aw_haptic->vib_dev.name = "vibrator";
#endif
	aw_haptic->vib_dev.brightness_get = brightness_get;
	aw_haptic->vib_dev.brightness_set = brightness_set;
	ret = devm_led_classdev_register(&aw_haptic->i2c->dev,
					 &aw_haptic->vib_dev);
	if (ret < 0) {
		aw_err("fail to create led dev");
		return ret;
	}
#endif

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error creating sysfs common attr files");
		return ret;
	}
	if (aw_haptic->is_supported_rtp && aw_haptic->is_used_irq_pin) {
		ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &rtp_attribute_group);
		if (ret < 0) {
			aw_err("error creating sysfs rtp attr files");
			return ret;
		}
	}
	hrtimer_init(&aw_haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->timer.function = vibrator_timer_func;
	INIT_WORK(&aw_haptic->vibrator_work, vibrator_work_routine);
	INIT_WORK(&aw_haptic->rtp_work, rtp_work_routine);
	mutex_init(&aw_haptic->lock);
	mutex_init(&aw_haptic->rtp_lock);

	return 0;
}

static void haptic_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw_haptic->haptic_audio.delay_val = 1;
	aw_haptic->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(aw_haptic->haptic_audio.ctr_list));
	hrtimer_init(&aw_haptic->haptic_audio.timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	aw_haptic->haptic_audio.timer.function = haptic_audio_timer_func;
	INIT_WORK(&aw_haptic->haptic_audio.work, haptic_audio_work_routine);
	mutex_init(&aw_haptic->haptic_audio.lock);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->activate_mode = aw_haptic->info.mode;
	aw_haptic->f0_pre = aw_haptic->info.f0_pre;
	aw_haptic->gun_type = AW_GUN_TYPE_DEF_VAL;
	aw_haptic->bullet_nr = AW_BULLET_NR_DEF_VAL;
	aw_haptic->func->play_mode(aw_haptic, AW_STANDBY_MODE);
	aw_haptic->func->misc_para_init(aw_haptic);
	aw_haptic->func->offset_cali(aw_haptic);
	aw_haptic->func->vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
	aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;

	if (aw_haptic->info.lk_f0_cali) {
		aw_haptic->f0_cali_data = aw_haptic->info.lk_f0_cali;
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
	} else {
		f0_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	aw_info("exit");
}

static void ram_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   ram_work.work);

	ram_update(aw_haptic);
}

static void ram_work_init(struct aw_haptic *aw_haptic)
{
	int ram_timer_val = AW_RAM_WORK_DELAY_INTERVAL;

	INIT_DELAYED_WORK(&aw_haptic->ram_work, ram_work_routine);
	schedule_delayed_work(&aw_haptic->ram_work,
			      msecs_to_jiffies(ram_timer_val));
}

static int get_dts_data(struct aw_haptic *aw_haptic, uint8_t *dts_name,
			struct aw_haptic_dts_data *dts_data,
			struct device_node *np)
{
	int ret = 0;
	uint32_t *dts_val = NULL;
	bool *dts_bool = NULL;
	uint8_t type = dts_data->type;

	if (dts_name == NULL)
		return -ERANGE;
	switch (type) {
	case AW_DTS_DATA_U32:
		dts_val = (uint32_t *)(dts_data->dts_val);
		ret = of_property_read_u32(np, dts_name, dts_val);
		if (ret != 0)
			aw_info("%s not found", dts_name);
		break;
	case AW_DTS_DATA_ARRAY:
		dts_val = (uint32_t *)(dts_data->dts_val);
		ret = of_property_read_u32_array(np, dts_name, dts_val,
						 dts_data->len);
		if (ret != 0)
			aw_info("%s not found", dts_name);
		break;
	case AW_DTS_DATA_BOOL:
		dts_bool = (bool *)(dts_data->dts_val);
		*dts_bool = of_property_read_bool(np, dts_name);
		break;
	default:
		break;
	}
	return 0;
}

static void parse_dts(struct aw_haptic *aw_haptic, struct device_node *np)
{
	int i = 0;
	struct aw_haptic_dts_info *info = &(aw_haptic->info);
	struct aw_haptic_dts_data *dts_data = NULL;
	struct aw_haptic_dts_name *dts_name_array =  aw_haptic->dts_name_array;

	dts_data = devm_kzalloc(aw_haptic->dev,
				sizeof(struct aw_haptic_dts_data),
				GFP_KERNEL);
	if (dts_data == NULL)
		return;
	for (i = 0; i < aw_haptic->dts_name_len; i++) {
		switch (dts_name_array[i].flag) {
		case AW_LK_F0_CALI:
			dts_data->dts_val = &(info->lk_f0_cali);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_MODE:
			dts_data->dts_val = &(info->mode);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_F0_PRE:
			dts_data->dts_val = &(info->f0_pre);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_F0_CALI_PERCENT:
			dts_data->dts_val = &(info->f0_cali_percent);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_DRV1_LVL:
			dts_data->dts_val = &(info->cont_drv1_lvl);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_DRV2_LVL:
			dts_data->dts_val = &(info->cont_drv2_lvl);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_LRA_VRMS:
			dts_data->dts_val = &(info->lra_vrms);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_DRV1_TIME:
			dts_data->dts_val = &(info->cont_drv1_time);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_DRV2_TIME:
			dts_data->dts_val = &(info->cont_drv2_time);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_TD:
			dts_data->dts_val = &(info->cont_td);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_ZC_THR:
			dts_data->dts_val = &(info->cont_zc_thr);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_NUM_BRK:
			dts_data->dts_val = &(info->cont_num_brk);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_WAIT_NUM:
			dts_data->dts_val = &(info->cont_wait_num);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_BRK_GAIN:
			dts_data->dts_val = &(info->cont_brk_gain);
			dts_data->len = 1;
			break;
		case AW_CONT_BEMF_SET:
			dts_data->dts_val = &(info->cont_bemf_set);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_D2S_GAIN:
			dts_data->dts_val = &(info->d2s_gain);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_BRK_TIME:
			dts_data->dts_val = &(info->cont_brk_time);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_TRACK_MARGIN:
			dts_data->dts_val = &(info->cont_track_margin);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_CONT_TSET:
			dts_data->dts_val = &(info->cont_tset);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_F0_COEFF:
			dts_data->dts_val = &(info->f0_coeff);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_U32;
			break;
		case AW_SINE_ARRAY:
			dts_data->dts_val = info->sine_array;
			dts_data->len = ARRAY_SIZE(info->sine_array);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_SW_BRAKE:
			dts_data->dts_val = info->sw_brake;
			dts_data->len = ARRAY_SIZE(info->sw_brake);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_DURATION_TIME:
			dts_data->dts_val = info->duration_time;
			dts_data->len = ARRAY_SIZE(info->duration_time);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_TRIG_CONFIG:
			dts_data->dts_val = info->trig_cfg;
			if (aw_haptic->name == AW8624)
				dts_data->len = 5;
			else
				dts_data->len = ARRAY_SIZE(info->trig_cfg);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_BRAKE_CONT_CONFIG:
			dts_data->dts_val = info->cont_brake;
			dts_data->len = ARRAY_SIZE(info->cont_brake);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_F0_TRACE_PARAM:
			dts_data->dts_val = info->f0_trace_parameter;
			dts_data->len = ARRAY_SIZE(info->f0_trace_parameter);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_BEMF_CONFIG:
			dts_data->dts_val = info->bemf_config;
			dts_data->len = ARRAY_SIZE(info->bemf_config);
			dts_data->type = AW_DTS_DATA_ARRAY;
			break;
		case AW_ENABLE_AUTO_BRK:
			dts_data->dts_val = &(info->is_enabled_auto_brk);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_BOOL;
			break;
		case AW_ENABLE_TRACK_EN:
			dts_data->dts_val = &(info->is_enabled_track_en);
			dts_data->len = 1;
			dts_data->type = AW_DTS_DATA_BOOL;
			break;
		default:
			dts_data->type = AW_DTS_NULL;
			break;
		}
		if (dts_data->type)
			get_dts_data(aw_haptic, dts_name_array[i].name,
				     dts_data, np);
	}

	devm_kfree(aw_haptic->dev, dts_data);
	dts_data = NULL;
}

#ifdef AW_INPUT_FRAMEWORK
static void input_gain_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   input_gain_work);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);
}

static void input_vib_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   input_vib_work);

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
		upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00,
						     aw_haptic->index);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
			ram_play(aw_haptic);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			aw_haptic->func->set_repeat_seq(aw_haptic,
							aw_haptic->index);
			ram_play(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int input_upload_effect(struct input_dev *dev, struct ff_effect *effect,
			       struct ff_effect *old)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);
	short wav_id = 0;
	int wav_id_max = 0;
	int ret = 0;

	mutex_lock(&aw_haptic->lock);
	switch (effect->type) {
	case FF_CONSTANT:
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
		aw_haptic->duration = effect->replay.length;
		aw_haptic->index = aw_haptic->ram.ram_num;
		aw_info("waveform id = %d", aw_haptic->index);
		break;
	case FF_PERIODIC:
		ret = copy_from_user(&wav_id, effect->u.periodic.custom_data,
				     sizeof(short));
		if (ret) {
			aw_err("copy from user error %d!!", ret);
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		aw_info("waveform id = %d", wav_id);
		wav_id_max = sizeof(aw_rtp_name) / sizeof(*aw_rtp_name) +
			     aw_haptic->ram.ram_num - 1;
		if (wav_id > 0 && wav_id < aw_haptic->ram.ram_num) {
			aw_haptic->activate_mode = AW_RAM_MODE;
			aw_haptic->index = wav_id;
		} else if (wav_id > aw_haptic->ram.ram_num && wav_id <= wav_id_max) {
			aw_haptic->activate_mode = AW_RTP_MODE;
			aw_haptic->rtp_file_num = wav_id - aw_haptic->ram.ram_num;
		} else {
			aw_haptic->activate_mode = AW_STANDBY_MODE;
			aw_err("waveform id is error");
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		break;
	default:
		aw_err("Unsupported effect type: %d", effect->type);
		break;
	}
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int input_playback(struct input_dev *dev, int effect_id, int val)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (val > 0)
		aw_haptic->state = 1;
	if (val <= 0)
		aw_haptic->state = 0;
	switch (aw_haptic->activate_mode) {
	case AW_RAM_MODE:
		schedule_work(&aw_haptic->input_vib_work);
		break;
	case AW_RAM_LOOP_MODE:
		schedule_work(&aw_haptic->input_vib_work);
		break;
	case AW_RTP_MODE:
		schedule_work(&aw_haptic->rtp_work);
		break;
	default:
		aw_err("Unsupported mode: %d", aw_haptic->activate_mode);
		break;
	}

	return 0;
}

static int input_erase(struct input_dev *dev, int effect_id)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	aw_haptic->duration = 0;
	return 0;
}

static void input_set_gain(struct input_dev *dev, uint16_t gain)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (gain > 0x7fff)
		gain = 0x7fff;
	aw_haptic->gain = gain * 0x80 / 0x7fff;
	schedule_work(&aw_haptic->input_gain_work);
	aw_info("aw_haptic->gain = 0x%02x", aw_haptic->gain);
}

static int input_framework_init(struct aw_haptic *aw_haptic)
{
	struct input_dev *input_dev;
	int ret = 0;

	input_dev = devm_input_allocate_device(aw_haptic->dev);
	if (input_dev == NULL)
		return -ENOMEM;
	input_dev->name = "aw-haptic-hv";
	input_set_drvdata(input_dev, aw_haptic);
	aw_haptic->input_dev = input_dev;
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_PERIODIC);
	input_set_capability(input_dev, EV_FF, FF_CUSTOM);
	ret = input_ff_create(input_dev, AW_EFFECT_NUMBER);
	if (ret < 0) {
		aw_err("create input FF device failed, rc=%d\n", ret);
		return ret;
	}
	input_dev->ff->upload = input_upload_effect;
	input_dev->ff->playback = input_playback;
	input_dev->ff->erase = input_erase;
	input_dev->ff->set_gain = input_set_gain;
	INIT_WORK(&aw_haptic->input_gain_work, input_gain_work_routine);
	INIT_WORK(&aw_haptic->input_vib_work, input_vib_work_routine);
	ret = input_register_device(input_dev);
	if (ret < 0) {
		aw_err("register input device failed, rc=%d\n", ret);
		input_ff_destroy(aw_haptic->input_dev);
		return ret;
	}
	return ret;
}
#endif

static int aw_i2c_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw_haptic *aw_haptic;
	struct device_node *np = i2c->dev.of_node;

	aw_info("enter");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		aw_err("check_functionality failed");
		return -EIO;
	}

	aw_haptic = devm_kzalloc(&i2c->dev, sizeof(struct aw_haptic),
				 GFP_KERNEL);
	if (aw_haptic == NULL)
		return -ENOMEM;

	aw_haptic->dev = &i2c->dev;
	aw_haptic->i2c = i2c;

	i2c_set_clientdata(i2c, aw_haptic);
#ifdef AW_INPUT_FRAMEWORK
	ret = input_framework_init(aw_haptic);
	if (ret < 0)
		goto err_input_config;
#endif
	/* aw_haptic rst & int */
	if (np) {
		ret = parse_dt_gpio(aw_haptic, &i2c->dev, np);
		if (ret) {
			aw_err("failed to parse gpio");
			goto err_parse_dt;
		}
	} else {
		aw_haptic->reset_gpio = -1;
		aw_haptic->irq_gpio = -1;
	}

	if (gpio_is_valid(aw_haptic->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "awinic_rst");
		if (ret) {
			aw_err("rst request failed");
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw_haptic->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->irq_gpio,
					    GPIOF_DIR_IN, "awinic_int");
		if (ret) {
			aw_err("int request failed");
			goto err_irq_gpio_request;
		}
	}

	parse_dts_i2c_addr(aw_haptic, &i2c->dev, np);
	/* aw_haptic chip id */
	ret = parse_chipid(aw_haptic);
	if (ret < 0) {
		aw_err("read_chipid failed ret=%d", ret);
		goto err_id;
	}

	sw_reset(aw_haptic);

	ret = chip_private_init(aw_haptic);
	if (ret < 0) {
		aw_err("chip_private_init failed ret=%d", ret);
		goto err_chip_private_init;
	}

	ret = func_ptr_init(aw_haptic, &i2c->dev);
	if (ret < 0) {
		aw_err("ctrl_init failed ret=%d", ret);
		goto err_ctrl_init;
	}

	ret = aw_haptic->func->check_qualify(aw_haptic);
	if (ret < 0) {
		aw_err("qualify check failed ret=%d", ret);
		goto err_ctrl_init;
	}

	parse_dts(aw_haptic, np);
	ret = irq_config(&i2c->dev, aw_haptic);
	if (ret != 0) {
		aw_err("irq_config failed ret=%d", ret);
		goto err_irq_config;
	}

	vibrator_init(aw_haptic);
	haptic_init(aw_haptic);
	aw_haptic->func->creat_node(aw_haptic);
	ram_work_init(aw_haptic);
	dev_set_drvdata(&i2c->dev, aw_haptic);
	aw_info("probe completed successfully!");
	return 0;

err_input_config:
err_id:
err_ctrl_init:
err_chip_private_init:
err_irq_config:
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);

err_irq_gpio_request:
	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);

err_parse_dt:
err_reset_gpio_request:
	devm_kfree(&i2c->dev, aw_haptic);
	aw_haptic = NULL;
	return ret;
}

static int aw_i2c_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_info("enter");
	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->haptic_audio.work);
	hrtimer_cancel(&aw_haptic->haptic_audio.timer);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->haptic_audio.lock);
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&aw_haptic->vib_dev);
#endif
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);

	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);

	return 0;
}

static int aw_i2c_suspend(struct device *dev)
{
	int ret = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	aw_info("enter");

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static int aw_i2c_resume(struct device *dev)
{
	aw_info("enter");
	return 0;
}

static SIMPLE_DEV_PM_OPS(aw_pm_ops, aw_i2c_suspend, aw_i2c_resume);

static const struct i2c_device_id aw_i2c_id[] = {
	{AW_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw_i2c_id);

static const struct of_device_id aw_dt_match[] = {
	{.compatible = "awinic,haptic_nv"},
	{},
};

static struct i2c_driver aw_i2c_driver = {
	.driver = {
		   .name = AW_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw_pm_ops,
#endif
		   },
	.probe = aw_i2c_probe,
	.remove = aw_i2c_remove,
	.id_table = aw_i2c_id,
};

static int __init aw_i2c_init(void)
{
	int ret = 0;

	aw_info("aw_haptic driver version %s", HAPTIC_NV_DRIVER_VERSION);
	ret = i2c_add_driver(&aw_i2c_driver);
	if (ret) {
		aw_err("fail to add aw_haptic device into i2c");
		return ret;
	}
	return 0;
}
module_init(aw_i2c_init);

static void __exit aw_i2c_exit(void)
{
	i2c_del_driver(&aw_i2c_driver);
}
module_exit(aw_i2c_exit);

MODULE_DESCRIPTION("AWINIC Haptic Driver");
MODULE_LICENSE("GPL v2");
