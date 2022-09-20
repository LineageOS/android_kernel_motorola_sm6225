/*
 * aw8695.c   aw8695 haptic module
 *
 * Version: v1.3.6
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
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
#include <asm/uaccess.h>
#include <linux/pm_qos.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/mmi_kernel_common.h>
#include "aw8695.h"
#include "aw8695_reg.h"
#include "aw8695_config.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8695_I2C_NAME "aw8695_haptic"
#define AW8695_HAPTIC_NAME "aw8695_haptic"

#define AW8695_VERSION "v1.3.6"


#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW8695_MAX_DSP_START_TRY_COUNT    10



#define AW8695_MAX_FIRMWARE_LOAD_CNT 20
#define AW8695_SEQ_NO_RTP_BASE 102
#define AW8695_SEQ_NO_RTP_REPEAT 100
#define AW8695_SEQ_NO_RTP_STOP 120000

#define AW8695_REPEAT_RTP_PLAYING

#define PM_QOS_VALUE_VB 400
struct pm_qos_request pm_qos_req_vb;

#ifdef CONFIG_AF_NOISE_ELIMINATION
static bool is_af_enabled = false;
#endif
/******************************************************
 *
 * variable
 *
 ******************************************************/
#define AW8695_RTP_NAME_MAX        64
static char *aw8695_ram_name = "aw8695_haptic.bin";
static char aw8695_rtp_name[][AW8695_RTP_NAME_MAX] = {
	{"aw8695_rtp.bin"},
	{"aw8695_rtp_Argo_Navis.bin"},
	{"aw8695_rtp_Attentive.bin"},
	{"aw8695_rtp_Awake.bin"},
	{"aw8695_rtp_Bird_Loop.bin"},
	{"aw8695_rtp_Brilliant_Times.bin"},
	{"aw8695_rtp_Chimey_Phone.bin"},
	{"aw8695_rtp_Complex.bin"},
	{"aw8695_rtp_Crazy_Dream.bin"},
	{"aw8695_rtp_Curve_Ball_Blend.bin"},
	{"aw8695_rtp_Digital_Phone.bin"},
	{"aw8695_rtp_Electrovision.bin"},
	{"aw8695_rtp_Ether_Shake.bin"},
	{"aw8695_rtp_Fateful_Words.bin"},
	{"aw8695_rtp_Flutey_Phone.bin"},
	{"aw8695_rtp_Future_Funk.bin"},
	{"aw8695_rtp_Future_Hi_Tech.bin"},
	{"aw8695_rtp_Girtab.bin"},
	{"aw8695_rtp_Hello.bin"},
	{"aw8695_rtp_Hexagon.bin"},
	{"aw8695_rtp_Hydra.bin"},
	{"aw8695_rtp_Insert_Coin.bin"},
	{"aw8695_rtp_Jumping_Dots.bin"},
	{"aw8695_rtp_Keys.bin"},
	{"aw8695_rtp_Loopy.bin"},
	{"aw8695_rtp_Loopy_Lounge.bin"},
	{"aw8695_rtp_Modular.bin"},
	{"aw8695_rtp_Momentum.bin"},
	{"aw8695_rtp_Morning.bin"},
	{"aw8695_rtp_Moto.bin"},
	{"aw8695_rtp_Natural.bin"},
	{"aw8695_rtp_New_Player.bin"},
	{"aw8695_rtp_Onward.bin"},
	{"aw8695_rtp_Organ_Dub.bin"},
	{"aw8695_rtp_Overclocked.bin"},
	{"aw8695_rtp_Pegasus.bin"},
	{"aw8695_rtp_Pyxis.bin"},
	{"aw8695_rtp_Regrade.bin"},
	{"aw8695_rtp_Scarabaeus.bin"},
	{"aw8695_rtp_Sceptrum.bin"},
	{"aw8695_rtp_Simple.bin"},
	{"aw8695_rtp_Solarium.bin"},
	{"aw8695_rtp_Sparse.bin"},
	{"aw8695_rtp_Terrabytes.bin"},
	{"aw8695_rtp_Zero_Hour.bin"},
	{"aw8695_rtp_Play.bin"},
	{"aw8695_rtp_TJINGLE.bin"},
	{"aw8695_rtp_Verizon_Airwaves.bin"},
	{"aw8695_rtp_City_Lights.bin"},
	{"aw8695_rtp_Firefly.bin"},
	{"aw8695_rtp_Now_or_Never.bin"},
	{"aw8695_rtp_Moto_Retro.bin"},
	{"aw8695_rtp_Moto_Original.bin"},
};

struct aw8695_container *aw8695_rtp;
struct aw8695 *g_aw8695;

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8695_interrupt_clear(struct aw8695 *aw8695);
static int aw8695_haptic_trig_enable_config(struct aw8695 *aw8695);
static void aw8695_vibrate(struct aw8695 *aw8695, int value);
#ifdef CONFIG_AF_NOISE_ELIMINATION
extern int mot_actuator_on_vibrate_start(void);
extern int mot_actuator_on_vibrate_stop(void);
#endif

/******************************************************
*
* aw8695 i2c write/read
*
******************************************************/
static int aw8695_i2c_write(struct aw8695 *aw8695,
			    unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw8695->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			break;
		}
		cnt ++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw8695_i2c_read(struct aw8695 *aw8695,
			   unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8695->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt ++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw8695_i2c_write_bits(struct aw8695 *aw8695,
				 unsigned char reg_addr, unsigned int mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw8695_i2c_read(aw8695, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw8695_i2c_write(aw8695, reg_addr, reg_val);

	return 0;
}

static int aw8695_i2c_writes(struct aw8695 *aw8695,
			     unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return  -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw8695->i2c, data, len + 1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
	}

	kfree(data);

	return ret;
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw8695_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw8695 *aw8695 = context;
	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8695_rtp_name[aw8695->rtp_file_num]);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw8695_rtp_name[aw8695->rtp_file_num],
		cont ? cont->size : 0);

	/* aw8695 rtp update */
	aw8695_rtp = vzalloc(cont->size + sizeof(int));
	if (!aw8695_rtp) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8695_rtp->len = cont->size;
	pr_info("%s: rtp size = %d\n", __func__, aw8695_rtp->len);
	memcpy(aw8695_rtp->data, cont->data, cont->size);
	release_firmware(cont);

	aw8695->rtp_init = 1;
	pr_info("%s: rtp update complete\n", __func__);
}

static int aw8695_rtp_update(struct aw8695 *aw8695)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8695_rtp_name[aw8695->rtp_file_num], aw8695->dev, GFP_KERNEL,
				       aw8695, aw8695_rtp_loaded);
}


static void aw8695_container_update(struct aw8695 *aw8695,
				    struct aw8695_container *aw8695_cont)
{
	int i = 0;
	unsigned int shift = 0;

	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8695->lock);

	aw8695->ram.baseaddr_shift = 2;
	aw8695->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_EN);

	/* base addr */
	shift = aw8695->ram.baseaddr_shift;
	aw8695->ram.base_addr = (unsigned int)((aw8695_cont->data[0 + shift] << 8) |
					       (aw8695_cont->data[1 + shift]));
	pr_info("%s: base_addr=0x%4x\n", __func__, aw8695->ram.base_addr);

	aw8695_i2c_write(aw8695, AW8695_REG_BASE_ADDRH, aw8695_cont->data[0 + shift]);
	aw8695_i2c_write(aw8695, AW8695_REG_BASE_ADDRL, aw8695_cont->data[1 + shift]);

	aw8695_i2c_write(aw8695, AW8695_REG_FIFO_AEH,
			 (unsigned char)((aw8695->ram.base_addr >> 2) >> 8));
	aw8695_i2c_write(aw8695, AW8695_REG_FIFO_AEL,
			 (unsigned char)((aw8695->ram.base_addr >> 2) & 0x00FF));
	aw8695_i2c_write(aw8695, AW8695_REG_FIFO_AFH,
			 (unsigned char)((aw8695->ram.base_addr - (aw8695->ram.base_addr >> 2)) >> 8));
	aw8695_i2c_write(aw8695, AW8695_REG_FIFO_AFL,
			 (unsigned char)((aw8695->ram.base_addr - (aw8695->ram.base_addr >> 2)) & 0x00FF));

	/* ram */
	shift = aw8695->ram.baseaddr_shift;
	aw8695_i2c_write(aw8695, AW8695_REG_RAMADDRH, aw8695_cont->data[0 + shift]);
	aw8695_i2c_write(aw8695, AW8695_REG_RAMADDRL, aw8695_cont->data[1 + shift]);
	shift = aw8695->ram.ram_shift;
	for (i = shift; i < aw8695_cont->len; i++) {
		aw8695_i2c_write(aw8695, AW8695_REG_RAMDATA, aw8695_cont->data[i]);
	}

#if 0
	/* ram check */
	shift = aw8695->ram.baseaddr_shift;
	aw8695_i2c_write(aw8695, AW8695_REG_RAMADDRH, aw8695_cont->data[0 + shift]);
	aw8695_i2c_write(aw8695, AW8695_REG_RAMADDRL, aw8695_cont->data[1 + shift]);
	shift = aw8695->ram.ram_shift;
	for (i = shift; i < aw8695_cont->len; i++) {
		aw8695_i2c_read(aw8695, AW8695_REG_RAMDATA, &reg_val);
		if (reg_val != aw8695_cont->data[i]) {
			pr_err("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
			       __func__, i, aw8695_cont->data[i], reg_val);
			return;
		}
	}
#endif

	/* RAMINIT Disable */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_OFF);

	mutex_unlock(&aw8695->lock);

	pr_info("%s exit\n", __func__);
}


static void aw8695_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw8695 *aw8695 = context;
	struct aw8695_container *aw8695_fw;
	int i = 0;
	unsigned short check_sum = 0;

	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8695_ram_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw8695_ram_name,
		cont ? cont->size : 0);
	/*
	    for(i=0; i<cont->size; i++) {
	        pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
	    }
	*/

	/* check sum */
	for (i = 2; i < cont->size; i++) {
		check_sum += cont->data[i];
	}
	if (check_sum != (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
		return;
	} else {
		pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw8695->ram.check_sum = check_sum;
	}

	/* aw8695 ram update */
	aw8695_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw8695_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8695_fw->len = cont->size;
	memcpy(aw8695_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw8695_container_update(aw8695, aw8695_fw);

	aw8695->ram.len = aw8695_fw->len;

	kfree(aw8695_fw);

	aw8695->ram_init = 1;
	pr_info("%s: fw update complete\n", __func__);

	aw8695_haptic_trig_enable_config(aw8695);

	aw8695_rtp_update(aw8695);
}

static int aw8695_ram_update(struct aw8695 *aw8695)
{
	aw8695->ram_init = 0;
	aw8695->rtp_init = 0;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8695_ram_name, aw8695->dev, GFP_KERNEL,
				       aw8695, aw8695_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw8695_ram_work_routine(struct work_struct *work)
{
	struct aw8695 *aw8695 = container_of(work, struct aw8695, ram_work.work);

	pr_info("%s enter\n", __func__);

	aw8695_ram_update(aw8695);

}
#endif

static int aw8695_ram_init(struct aw8695 *aw8695)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;
	INIT_DELAYED_WORK(&aw8695->ram_work, aw8695_ram_work_routine);
	schedule_delayed_work(&aw8695->ram_work, msecs_to_jiffies(ram_timer_val));
#else
	aw8695_ram_update(aw8695);
#endif
	return 0;
}



/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw8695_haptic_softreset(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	aw8695_i2c_write(aw8695, AW8695_REG_ID, 0xAA);
	msleep(1);
	return 0;
}

static int aw8695_haptic_active(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_WORK_MODE_MASK, AW8695_BIT_SYSCTRL_ACTIVE);
	aw8695_interrupt_clear(aw8695);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
			      AW8695_BIT_SYSINTM_UVLO_MASK, AW8695_BIT_SYSINTM_UVLO_EN);
	return 0;
}

static int aw8695_haptic_play_mode(struct aw8695 *aw8695, unsigned char play_mode)
{
	pr_debug("%s enter\n", __func__);

	switch (play_mode) {
	case AW8695_HAPTIC_STANDBY_MODE:
		aw8695->play_mode = AW8695_HAPTIC_STANDBY_MODE;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
				      AW8695_BIT_SYSINTM_UVLO_MASK, AW8695_BIT_SYSINTM_UVLO_OFF);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_WORK_MODE_MASK, AW8695_BIT_SYSCTRL_STANDBY);
		break;
	case AW8695_HAPTIC_RAM_MODE:
		aw8695->play_mode = AW8695_HAPTIC_RAM_MODE;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_PLAY_MODE_MASK, AW8695_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8695_haptic_active(aw8695);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
					      AW8695_BIT_SYSCTRL_BST_MODE_MASK & AW8695_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8695_BIT_SYSCTRL_BST_MODE_BOOST | AW8695_BIT_SYSCTRL_STANDBY);
			aw8695_haptic_active(aw8695);
		} else {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
					      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		msleep(3);
		break;
	case AW8695_HAPTIC_RAM_LOOP_MODE:
		aw8695->play_mode = AW8695_HAPTIC_RAM_LOOP_MODE;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_PLAY_MODE_MASK, AW8695_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8695_haptic_active(aw8695);
		break;
	case AW8695_HAPTIC_RTP_MODE:
		aw8695->play_mode = AW8695_HAPTIC_RTP_MODE;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_PLAY_MODE_MASK, AW8695_BIT_SYSCTRL_PLAY_MODE_RTP);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RTP_DISABLE);
		}
		aw8695_haptic_active(aw8695);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RTP_ENABLE);
			aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
					      AW8695_BIT_SYSCTRL_BST_MODE_MASK & AW8695_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8695_BIT_SYSCTRL_BST_MODE_BOOST | AW8695_BIT_SYSCTRL_STANDBY);
			aw8695_haptic_active(aw8695);
		} else {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
					      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		msleep(3);
		break;
	case AW8695_HAPTIC_TRIG_MODE:
		aw8695->play_mode = AW8695_HAPTIC_TRIG_MODE;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_PLAY_MODE_MASK, AW8695_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8695_haptic_active(aw8695);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
					      AW8695_BIT_SYSCTRL_BST_MODE_MASK & AW8695_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8695_BIT_SYSCTRL_BST_MODE_BOOST | AW8695_BIT_SYSCTRL_STANDBY);
			aw8695_haptic_active(aw8695);
		} else {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
					      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		msleep(3);
		break;
	case AW8695_HAPTIC_CONT_MODE:
		aw8695->play_mode = AW8695_HAPTIC_CONT_MODE;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_PLAY_MODE_MASK, AW8695_BIT_SYSCTRL_PLAY_MODE_CONT);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
				      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8695->auto_boost) {
			aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
					      AW8695_BIT_BST_AUTO_BST_RAM_MASK, AW8695_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8695_haptic_active(aw8695);
		break;
	default:
		dev_err(aw8695->dev, "%s: play mode %d err",
			__func__, play_mode);
		break;
	}
	return 0;
}

static int aw8695_haptic_play_go(struct aw8695 *aw8695, bool flag)
{
	pr_debug("%s enter\n", __func__);

	if (!flag) {
		GET_TIME_OF_DAY(&aw8695->current_time);
		aw8695->interval_us = (aw8695->current_time.tv_sec - aw8695->pre_enter_time.tv_sec) * 1000000
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
			+ (aw8695->current_time.tv_nsec - aw8695->pre_enter_time.tv_nsec) / 1000;
#else
			+ (aw8695->current_time.tv_usec - aw8695->pre_enter_time.tv_usec);
#endif

		if (aw8695->interval_us < 2000) {
			pr_info("aw8695->interval_us t=%u\n", aw8695->interval_us);
			mdelay(2);
		}
	}
	if(flag == true) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_GO,
			AW8695_BIT_GO_MASK, AW8695_BIT_GO_ENABLE);
		GET_TIME_OF_DAY(&aw8695->pre_enter_time);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_GO,
			AW8695_BIT_GO_MASK, AW8695_BIT_GO_DISABLE);
	}
	return 0;
}

static int aw8695_haptic_stop_delay(struct aw8695 *aw8695)
{
	unsigned char reg_val = 0;
	unsigned int cnt = 100;

	while (cnt--) {
		aw8695_i2c_read(aw8695, AW8695_REG_GLB_STATE, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			return 0;
		}
		msleep(2);
		pr_debug("%s wait for standby, reg glb_state=0x%02x\n",
			 __func__, reg_val);
	}
	pr_err("%s do not enter standby automatically\n", __func__);

	return 0;
}

static int aw8695_haptic_stop(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	aw8695_haptic_play_go(aw8695, false);
	aw8695_haptic_stop_delay(aw8695);
	aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_STANDBY_MODE);

#ifdef CONFIG_AF_NOISE_ELIMINATION
	if ((aw8695->haptic_mode == HAPTIC_LONG) || (is_af_enabled == true)) {
		pr_info("%s: %d: mot_actuator_on_vibrate_stop, duration=%d, haptic_mode=%d, play_mode=%hhu \n", __func__,__LINE__
			,aw8695->duration,aw8695->haptic_mode,aw8695->play_mode);
		mot_actuator_on_vibrate_stop();
		is_af_enabled = false;
	}
#endif

	return 0;
}

static int aw8695_haptic_start(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	aw8695_haptic_play_go(aw8695, true);

	return 0;
}

static int aw8695_haptic_set_wav_seq(struct aw8695 *aw8695,
				     unsigned char wav, unsigned char seq)
{
	aw8695_i2c_write(aw8695, AW8695_REG_WAVSEQ1 + wav,
			 seq);
	return 0;
}

static int aw8695_haptic_set_wav_loop(struct aw8695 *aw8695,
				      unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_WAVLOOP1 + (wav / 2),
				      AW8695_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw8695_i2c_write_bits(aw8695, AW8695_REG_WAVLOOP1 + (wav / 2),
				      AW8695_BIT_WAVLOOP_SEQN_MASK, tmp);
	}

	return 0;
}
/*
static int aw8695_haptic_set_main_loop(struct aw8695 *aw8695,
        unsigned char loop)
{
    aw8695_i2c_write(aw8695, AW8695_REG_MAIN_LOOP, loop);
    return 0;
}
*/

static int aw8695_haptic_set_repeat_wav_seq(struct aw8695 *aw8695, unsigned char seq)
{
	aw8695_haptic_set_wav_seq(aw8695, 0x00, seq);
	aw8695_haptic_set_wav_loop(aw8695, 0x00, AW8695_BIT_WAVLOOP_INIFINITELY);

	return 0;
}


static int aw8695_haptic_set_bst_vol(struct aw8695 *aw8695, unsigned char bst_vol)
{
	if (bst_vol & 0xe0) {
		bst_vol = 0x1f;
	}
	aw8695_i2c_write_bits(aw8695, AW8695_REG_BSTDBG4,
			      AW8695_BIT_BSTDBG4_BSTVOL_MASK, (bst_vol << 1));
	return 0;
}

static int aw8695_haptic_set_bst_peak_cur(struct aw8695 *aw8695, unsigned char peak_cur)
{
	peak_cur &= AW8695_BSTCFG_PEAKCUR_LIMIT;
	aw8695_i2c_write_bits(aw8695, AW8695_REG_BSTCFG,
			      AW8695_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
	return 0;
}

static int aw8695_haptic_set_gain(struct aw8695 *aw8695, unsigned char gain)
{
	aw8695_i2c_write(aw8695, AW8695_REG_DATDBG, gain);
	return 0;
}

static int aw8695_haptic_set_pwm(struct aw8695 *aw8695, unsigned char mode)
{
	switch (mode) {
	case AW8695_PWM_48K:
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PWMDBG,
				      AW8695_BIT_PWMDBG_PWM_MODE_MASK, AW8695_BIT_PWMDBG_PWM_48K);
		break;
	case AW8695_PWM_24K:
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PWMDBG,
				      AW8695_BIT_PWMDBG_PWM_MODE_MASK, AW8695_BIT_PWMDBG_PWM_24K);
		break;
	case AW8695_PWM_12K:
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PWMDBG,
				      AW8695_BIT_PWMDBG_PWM_MODE_MASK, AW8695_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw8695_haptic_play_wav_seq(struct aw8695 *aw8695, unsigned char flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag) {
		aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_RAM_MODE);
		aw8695_haptic_start(aw8695);
	}
	return 0;
}

static int aw8695_haptic_play_repeat_seq(struct aw8695 *aw8695, unsigned char flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag) {
		aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_RAM_LOOP_MODE);
		aw8695_haptic_start(aw8695);
	}

	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw8695_haptic_swicth_motorprotect_config(struct aw8695 *aw8695, unsigned char addr, unsigned char val)
{
	pr_debug("%s enter\n", __func__);
	if (addr == 1) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_DETCTRL,
				      AW8695_BIT_DETCTRL_PROTECT_MASK, AW8695_BIT_DETCTRL_PROTECT_SHUTDOWN);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PWMPRC,
				      AW8695_BIT_PWMPRC_PRC_MASK, AW8695_BIT_PWMPRC_PRC_ENABLE);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PRLVL,
				      AW8695_BIT_PRLVL_PR_MASK, AW8695_BIT_PRLVL_PR_ENABLE);
	} else if (addr == 0) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_DETCTRL,
				      AW8695_BIT_DETCTRL_PROTECT_MASK,  AW8695_BIT_DETCTRL_PROTECT_NO_ACTION);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PWMPRC,
				      AW8695_BIT_PWMPRC_PRC_MASK, AW8695_BIT_PWMPRC_PRC_DISABLE);
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PRLVL,
				      AW8695_BIT_PRLVL_PR_MASK, AW8695_BIT_PRLVL_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PWMPRC,
				      AW8695_BIT_PWMPRC_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PRLVL,
				      AW8695_BIT_PRLVL_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_PRTIME,
				      AW8695_BIT_PRTIME_PRTIME_MASK, val);
	} else {
		/*nothing to do;*/
	}
	return 0;
}

/*****************************************************
 *
 * os calibration
 *
 *****************************************************/
static int aw8695_haptic_os_calibration(struct aw8695 *aw8695)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;
	pr_debug("%s enter\n", __func__);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DETCTRL,
			      AW8695_BIT_DETCTRL_DIAG_GO_MASK, AW8695_BIT_DETCTRL_DIAG_GO_ENABLE);
	while (1) {
		aw8695_i2c_read(aw8695, AW8695_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_OFF);
	return 0;
}
/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw8695_haptic_trig_param_init(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	aw8695->trig[0].enable = AW8695_TRG1_DISABLE;
	aw8695->trig[0].default_level = AW8695_TRG1_DEFAULT_LEVEL;
	aw8695->trig[0].dual_edge = AW8695_TRG1_DUAL_EDGE;
	aw8695->trig[0].frist_seq = AW8695_TRG1_FIRST_EDGE_SEQ;
	aw8695->trig[0].second_seq = AW8695_TRG1_SECOND_EDGE_SEQ;

	aw8695->trig[1].enable = AW8695_TRG2_DISABLE;
	aw8695->trig[1].default_level = AW8695_TRG2_DEFAULT_LEVEL;
	aw8695->trig[1].dual_edge = AW8695_TRG2_DUAL_EDGE;
	aw8695->trig[1].frist_seq = AW8695_TRG2_FIRST_EDGE_SEQ;
	aw8695->trig[1].second_seq = AW8695_TRG2_SECOND_EDGE_SEQ;

	aw8695->trig[2].enable = AW8695_TRG3_DISABLE;
	aw8695->trig[2].default_level = AW8695_TRG3_DEFAULT_LEVEL;
	aw8695->trig[2].dual_edge = AW8695_TRG3_DUAL_EDGE;
	aw8695->trig[2].frist_seq = AW8695_TRG3_FIRST_EDGE_SEQ;
	aw8695->trig[2].second_seq = AW8695_TRG3_SECOND_EDGE_SEQ;

	return 0;
}

static int aw8695_haptic_trig_param_config(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	if (aw8695->trig[0].default_level) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG1_POLAR_MASK, AW8695_BIT_TRGCFG1_TRG1_POLAR_NEG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG1_POLAR_MASK, AW8695_BIT_TRGCFG1_TRG1_POLAR_POS);
	}
	if (aw8695->trig[1].default_level) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG2_POLAR_MASK, AW8695_BIT_TRGCFG1_TRG2_POLAR_NEG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG2_POLAR_MASK, AW8695_BIT_TRGCFG1_TRG2_POLAR_POS);
	}
	if (aw8695->trig[2].default_level) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG3_POLAR_MASK, AW8695_BIT_TRGCFG1_TRG3_POLAR_NEG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG3_POLAR_MASK, AW8695_BIT_TRGCFG1_TRG3_POLAR_POS);
	}

	if (aw8695->trig[0].dual_edge) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG1_EDGE_MASK, AW8695_BIT_TRGCFG1_TRG1_EDGE_POS_NEG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG1_EDGE_MASK, AW8695_BIT_TRGCFG1_TRG1_EDGE_POS);
	}
	if (aw8695->trig[1].dual_edge) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG2_EDGE_MASK, AW8695_BIT_TRGCFG1_TRG2_EDGE_POS_NEG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG2_EDGE_MASK, AW8695_BIT_TRGCFG1_TRG2_EDGE_POS);
	}
	if (aw8695->trig[2].dual_edge) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG3_EDGE_MASK, AW8695_BIT_TRGCFG1_TRG3_EDGE_POS_NEG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG1,
				      AW8695_BIT_TRGCFG1_TRG3_EDGE_MASK, AW8695_BIT_TRGCFG1_TRG3_EDGE_POS);
	}

	if (aw8695->trig[0].frist_seq) {
		aw8695_i2c_write(aw8695, AW8695_REG_TRG1_WAV_P, aw8695->trig[0].frist_seq);
	}
	if (aw8695->trig[0].second_seq && aw8695->trig[0].dual_edge) {
		aw8695_i2c_write(aw8695, AW8695_REG_TRG1_WAV_N, aw8695->trig[0].second_seq);
	}
	if (aw8695->trig[1].frist_seq) {
		aw8695_i2c_write(aw8695, AW8695_REG_TRG2_WAV_P, aw8695->trig[1].frist_seq);
	}
	if (aw8695->trig[1].second_seq && aw8695->trig[1].dual_edge) {
		aw8695_i2c_write(aw8695, AW8695_REG_TRG2_WAV_N, aw8695->trig[1].second_seq);
	}
	if (aw8695->trig[2].frist_seq) {
		aw8695_i2c_write(aw8695, AW8695_REG_TRG3_WAV_P, aw8695->trig[1].frist_seq);
	}
	if (aw8695->trig[2].second_seq && aw8695->trig[2].dual_edge) {
		aw8695_i2c_write(aw8695, AW8695_REG_TRG3_WAV_N, aw8695->trig[1].second_seq);
	}

	return 0;
}

static int aw8695_haptic_trig_enable_config(struct aw8695 *aw8695)
{
	pr_debug("%s enter\n", __func__);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG2,
			      AW8695_BIT_TRGCFG2_TRG1_ENABLE_MASK, aw8695->trig[0].enable);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG2,
			      AW8695_BIT_TRGCFG2_TRG2_ENABLE_MASK, aw8695->trig[1].enable);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_TRG_CFG2,
			      AW8695_BIT_TRGCFG2_TRG3_ENABLE_MASK, aw8695->trig[2].enable);

	return 0;
}


static int aw8695_haptic_auto_boost_config(struct aw8695 *aw8695, unsigned char flag)
{
	aw8695->auto_boost = flag;
	if (flag) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
				      AW8695_BIT_BST_AUTO_BST_AUTOSW_MASK, AW8695_BIT_BST_AUTO_BST_AUTOMATIC_BOOST);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_BST_AUTO,
				      AW8695_BIT_BST_AUTO_BST_AUTOSW_MASK, AW8695_BIT_BST_AUTO_BST_MANUAL_BOOST);
	}
	return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw8695_haptic_cont_vbat_mode(struct aw8695 *aw8695, unsigned char flag)
{
	if (flag == AW8695_HAPTIC_CONT_VBAT_HW_COMP_MODE) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_ADCTEST,
				      AW8695_BIT_ADCTEST_VBAT_MODE_MASK, AW8695_BIT_ADCTEST_VBAT_HW_COMP);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_ADCTEST,
				      AW8695_BIT_ADCTEST_VBAT_MODE_MASK, AW8695_BIT_ADCTEST_VBAT_SW_COMP);
	}
	return 0;
}

static int aw8695_haptic_get_vbat(struct aw8695 *aw8695)
{
	unsigned char reg_val = 0;
	unsigned int cont = 2000;

	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DETCTRL,
			      AW8695_BIT_DETCTRL_VBAT_GO_MASK, AW8695_BIT_DETCTRL_VABT_GO_ENABLE);

	while (1) {
		aw8695_i2c_read(aw8695, AW8695_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
		cont--;
	}

	aw8695_i2c_read(aw8695, AW8695_REG_VBATDET, &reg_val);
	aw8695->vbat = 6100 * reg_val / 256;
	if (aw8695->vbat > AW8695_VBAT_MAX) {
		aw8695->vbat = AW8695_VBAT_MAX;
		pr_debug("%s vbat max limit = %dmV\n", __func__, aw8695->vbat);
	}
	if (aw8695->vbat < AW8695_VBAT_MIN) {
		aw8695->vbat = AW8695_VBAT_MIN;
		pr_debug("%s vbat min limit = %dmV\n", __func__, aw8695->vbat);
	}

	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_OFF);

	return 0;
}

static int aw8695_haptic_ram_vbat_comp(struct aw8695 *aw8695, bool flag)
{
	int temp_gain = 0;
	int aw8695_gain = 0;

	if (aw8695->debugfs_debug)
		aw8695_gain = aw8695->gain_debug;
	else
		aw8695_gain = aw8695->gain;
	if (flag) {
		if (aw8695->ram_vbat_comp == AW8695_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw8695_haptic_get_vbat(aw8695);
			temp_gain = aw8695_gain * AW8695_VBAT_REFER / aw8695->vbat;
			if (temp_gain > (128 * AW8695_VBAT_REFER / AW8695_VBAT_MIN)) {
				temp_gain = 128 * AW8695_VBAT_REFER / AW8695_VBAT_MIN;
				pr_debug("%s gain limit=%d\n", __func__, temp_gain);
			}
			aw8695_haptic_set_gain(aw8695, temp_gain);
		} else {
			aw8695_haptic_set_gain(aw8695, aw8695_gain);
		}
	} else {
		aw8695_haptic_set_gain(aw8695, aw8695_gain);
	}

	return 0;
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw8695_haptic_set_f0_preset(struct aw8695 *aw8695)
{
	unsigned int f0_reg = 0;

	pr_debug("%s enter\n", __func__);

	f0_reg = 1000000000 / (aw8695->f0_pre * AW8695_HAPTIC_F0_COEFF);
	aw8695_i2c_write(aw8695, AW8695_REG_F_PRE_H, (unsigned char)((f0_reg >> 8) & 0xff));
	aw8695_i2c_write(aw8695, AW8695_REG_F_PRE_L, (unsigned char)((f0_reg >> 0) & 0xff));

	return 0;
}

static int aw8695_haptic_read_f0(struct aw8695 *aw8695)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_debug("%s enter\n", __func__);

	ret = aw8695_i2c_read(aw8695, AW8695_REG_F_LRA_F0_H, &reg_val);
	f0_reg = (reg_val << 8);
	ret = aw8695_i2c_read(aw8695, AW8695_REG_F_LRA_F0_L, &reg_val);
	f0_reg |= (reg_val << 0);
	f0_tmp = 1000000000 / (f0_reg * AW8695_HAPTIC_F0_COEFF);
	aw8695->f0 = (unsigned int)f0_tmp;
	pr_info("%s f0=%d\n", __func__, aw8695->f0);

	return 0;
}

static int aw8695_haptic_read_cont_f0(struct aw8695 *aw8695)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_debug("%s enter\n", __func__);

	ret = aw8695_i2c_read(aw8695, AW8695_REG_F_LRA_CONT_H, &reg_val);
	f0_reg = (reg_val << 8);
	ret = aw8695_i2c_read(aw8695, AW8695_REG_F_LRA_CONT_L, &reg_val);
	f0_reg |= (reg_val << 0);
	f0_tmp = 1000000000 / (f0_reg * AW8695_HAPTIC_F0_COEFF);
	aw8695->cont_f0 = (unsigned int)f0_tmp;
	pr_info("%s f0=%d\n", __func__, aw8695->cont_f0);

	return 0;
}

static int aw8695_haptic_read_beme(struct aw8695 *aw8695)
{
	int ret = 0;
	unsigned char reg_val = 0;

	ret = aw8695_i2c_read(aw8695, AW8695_REG_WAIT_VOL_MP, &reg_val);
	aw8695->max_pos_beme = (reg_val << 0);
	ret = aw8695_i2c_read(aw8695, AW8695_REG_WAIT_VOL_MN, &reg_val);
	aw8695->max_neg_beme = (reg_val << 0);

	pr_info("%s max_pos_beme=%d\n", __func__, aw8695->max_pos_beme);
	pr_info("%s max_neg_beme=%d\n", __func__, aw8695->max_neg_beme);

	return 0;
}



/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw8695_haptic_set_rtp_aei(struct aw8695 *aw8695, bool flag)
{
	if (flag) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
				      AW8695_BIT_SYSINTM_FF_AE_MASK, AW8695_BIT_SYSINTM_FF_AE_EN);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
				      AW8695_BIT_SYSINTM_FF_AE_MASK, AW8695_BIT_SYSINTM_FF_AE_OFF);
	}
}
/*
static void aw8695_haptic_set_rtp_afi(struct aw8695 *aw8695, bool flag)
{
    if(flag) {
        aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
                AW8695_BIT_SYSINTM_FF_AF_MASK, AW8695_BIT_SYSINTM_FF_AF_EN);
    } else {
        aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
                AW8695_BIT_SYSINTM_FF_AF_MASK, AW8695_BIT_SYSINTM_FF_AF_OFF);
    }
}
*/
/*
static unsigned char aw8695_haptic_rtp_get_fifo_aei(struct aw8695 *aw8695)
{
    unsigned char ret;
    unsigned char reg_val;

    aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
    reg_val &= AW8695_BIT_SYSINT_FF_AEI;
    ret = reg_val>>4;

    return ret;
}
*/

static unsigned char aw8695_haptic_rtp_get_fifo_afi(struct aw8695 *aw8695)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
	reg_val &= AW8695_BIT_SYSINT_FF_AFI;
	ret = reg_val >> 3;

	return ret;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static int aw8695_haptic_rtp_init(struct aw8695 *aw8695)
{
	unsigned int buf_len = 0;
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);

	aw8695->rtp_cnt = 0;

	while ((!aw8695_haptic_rtp_get_fifo_afi(aw8695)) &&
	       (aw8695->play_mode == AW8695_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw8695->rtp_cnt);
		if (!aw8695_rtp) {
			pr_info("%s:aw8695_rtp is null break\n", __func__);
			break;
		}

		if ((aw8695->rtp_cnt < aw8695->ram.base_addr)) {
			if((aw8695_rtp->len-aw8695->rtp_cnt) < (aw8695->ram.base_addr)){
				buf_len = aw8695_rtp->len-aw8695->rtp_cnt;
			} else {
				buf_len = (aw8695->ram.base_addr);
			}
		} else if ((aw8695_rtp->len - aw8695->rtp_cnt) < (aw8695->ram.base_addr >> 2)) {
			buf_len = aw8695_rtp->len - aw8695->rtp_cnt;
		} else {
			buf_len = (aw8695->ram.base_addr >> 2);
		}
		aw8695_i2c_writes(aw8695, AW8695_REG_RTP_DATA,
				  &aw8695_rtp->data[aw8695->rtp_cnt], buf_len);
		aw8695->rtp_cnt += buf_len;

		aw8695_i2c_read(aw8695, AW8695_REG_GLB_STATE, &reg_val);
		if ((aw8695->rtp_cnt == aw8695_rtp->len) || ((reg_val & 0x0f) == 0x00)) {
			pr_info("%s: rtp update complete\n", __func__);
			aw8695->rtp_cnt = 0;
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (aw8695->play_mode == AW8695_HAPTIC_RTP_MODE) {
		aw8695_haptic_set_rtp_aei(aw8695, true);
	}
	pm_qos_remove_request(&pm_qos_req_vb);
	pr_info("%s exit\n", __func__);

	return 0;
}

static void aw8695_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	struct aw8695 *aw8695 = container_of(work, struct aw8695, rtp_work);

	pr_info("%s enter\n", __func__);

	/* fw loaded */
	ret = request_firmware(&rtp_file,
			       aw8695_rtp_name[aw8695->rtp_file_num],
			       aw8695->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw8695_rtp_name[aw8695->rtp_file_num]);
		return ;
	}
	aw8695->rtp_init = 0;
	vfree(aw8695_rtp);
	aw8695_rtp = vzalloc(rtp_file->size + sizeof(int));
	if (!aw8695_rtp) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
	aw8695_rtp->len = rtp_file->size;
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw8695_rtp_name[aw8695->rtp_file_num], aw8695_rtp->len);
	memcpy(aw8695_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);

	mutex_lock(&aw8695->lock);

	aw8695->rtp_init = 1;

	/* gain */
	aw8695_haptic_ram_vbat_comp(aw8695, false);

	/* rtp mode config */
	aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_RTP_MODE);

	/* haptic start */
	aw8695_haptic_start(aw8695);

	mutex_unlock(&aw8695->lock);

	aw8695_haptic_rtp_init(aw8695);
}


/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static enum hrtimer_restart aw8695_haptic_audio_timer_func(struct hrtimer *timer)
{
	struct aw8695 *aw8695 = container_of(timer, struct aw8695, haptic_audio.timer);

	pr_debug("%s enter\n", __func__);
	schedule_work(&aw8695->haptic_audio.work);

	hrtimer_start(&aw8695->haptic_audio.timer,
		      ktime_set(aw8695->haptic_audio.timer_val / 1000000,
				(aw8695->haptic_audio.timer_val % 1000000) * 1000),
		      HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void aw8695_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw8695 *aw8695 = container_of(work, struct aw8695, haptic_audio.work);

	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8695->haptic_audio.lock);
	memcpy(&aw8695->haptic_audio.ctr,
	       &aw8695->haptic_audio.data[aw8695->haptic_audio.cnt],
	       sizeof(struct haptic_ctr));
	pr_debug("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
		 __func__,
		 aw8695->haptic_audio.cnt,
		 aw8695->haptic_audio.ctr.cmd,
		 aw8695->haptic_audio.ctr.play,
		 aw8695->haptic_audio.ctr.wavseq,
		 aw8695->haptic_audio.ctr.loop,
		 aw8695->haptic_audio.ctr.gain);
	mutex_unlock(&aw8695->haptic_audio.lock);
	if (aw8695->haptic_audio.ctr.cmd == 0x01) {
		mutex_lock(&aw8695->lock);
		aw8695_haptic_stop(aw8695);
		mutex_unlock(&aw8695->lock);
		if (aw8695->haptic_audio.ctr.play == 0x01) {
			pr_info("%s: haptic_audio_play_start\n", __func__);
			mutex_lock(&aw8695->lock);
			aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_RAM_MODE);

			aw8695_haptic_set_wav_seq(aw8695, 0x00,
						  aw8695->haptic_audio.ctr.wavseq);
			aw8695_haptic_set_wav_seq(aw8695, 0x01, 0x00);

			aw8695_haptic_set_wav_loop(aw8695, 0x00,
						   aw8695->haptic_audio.ctr.loop);

			aw8695_haptic_set_gain(aw8695,
					       aw8695->haptic_audio.ctr.gain);

			aw8695_haptic_start(aw8695);
			mutex_unlock(&aw8695->lock);
		}
		mutex_lock(&aw8695->haptic_audio.lock);
		memset(&aw8695->haptic_audio.data[aw8695->haptic_audio.cnt],
		       0, sizeof(struct haptic_ctr));
		mutex_unlock(&aw8695->haptic_audio.lock);
	}

	mutex_lock(&aw8695->haptic_audio.lock);
	aw8695->haptic_audio.cnt ++;
	if (aw8695->haptic_audio.data[aw8695->haptic_audio.cnt].cmd == 0) {
		aw8695->haptic_audio.cnt = 0;
		pr_debug("%s: haptic play buffer restart\n", __func__);
	}
	mutex_unlock(&aw8695->haptic_audio.lock);

}


/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw8695_haptic_cont(struct aw8695 *aw8695)
{
	pr_info("%s enter\n", __func__);

	/* work mode */
	aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_CONT_MODE);

	/* preset f0 */
	aw8695->f0_pre = aw8695->f0;
	aw8695_haptic_set_f0_preset(aw8695);

	/* lpf */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DATCTRL,
			      AW8695_BIT_DATCTRL_FC_MASK, AW8695_BIT_DATCTRL_FC_1000HZ);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DATCTRL,
			      AW8695_BIT_DATCTRL_LPF_ENABLE_MASK, AW8695_BIT_DATCTRL_LPF_ENABLE);

	/* cont config */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_ZC_DETEC_MASK, AW8695_BIT_CONT_CTRL_ZC_DETEC_ENABLE);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_WAIT_PERIOD_MASK, AW8695_BIT_CONT_CTRL_WAIT_1PERIOD);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_MODE_MASK, AW8695_BIT_CONT_CTRL_BY_GO_SIGNAL);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8695_CONT_PLAYBACK_MODE);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_F0_DETECT_MASK, AW8695_BIT_CONT_CTRL_F0_DETECT_DISABLE);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_O2C_MASK, AW8695_BIT_CONT_CTRL_O2C_DISABLE);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_AUTO_BRK_MASK, AW8695_BIT_CONT_CTRL_AUTO_BRK_ENABLE);

	/* TD time */
	aw8695_i2c_write(aw8695, AW8695_REG_TD_H, (unsigned char)(aw8695->cont_td >> 8));
	aw8695_i2c_write(aw8695, AW8695_REG_TD_L, (unsigned char)(aw8695->cont_td >> 0));
	aw8695_i2c_write(aw8695, AW8695_REG_TSET, 0x12);

	/* zero cross */
	aw8695_i2c_write(aw8695, AW8695_REG_ZC_THRSH_H, (unsigned char)(aw8695->cont_zc_thr >> 8));
	aw8695_i2c_write(aw8695, AW8695_REG_ZC_THRSH_L, (unsigned char)(aw8695->cont_zc_thr >> 0));

	/* bemf */
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHH_H, 0x10);
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHH_L, 0x08);
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHL_H, 0x03);
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHL_L, 0xf8);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_BEMF_NUM,
			      AW8695_BIT_BEMF_NUM_BRK_MASK, aw8695->cont_num_brk);
	aw8695_i2c_write(aw8695, AW8695_REG_TIME_NZC, 0x23);  /* 35*171us=5.985ms */

	/* f0 driver level */
	aw8695_i2c_write(aw8695, AW8695_REG_DRV_LVL, aw8695->cont_drv_lvl);
	aw8695_i2c_write(aw8695, AW8695_REG_DRV_LVL_OV, aw8695->cont_drv_lvl_ov);

	/* cont play go */
	aw8695_haptic_play_go(aw8695, true);

	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw8695_haptic_get_f0(struct aw8695 *aw8695)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char f0_pre_num = 0;
	unsigned char f0_wait_num = 0;
	unsigned char f0_repeat_num = 0;
	unsigned char f0_trace_num = 0;
	unsigned int t_f0_ms = 0;
	unsigned int t_f0_trace_ms = 0;
	unsigned int f0_cali_cnt = 50;

	pr_info("%s enter\n", __func__);

	aw8695->f0 = aw8695->f0_pre;

	/* f0 calibrate work mode */
	aw8695_haptic_stop(aw8695);
	aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_CONT_MODE);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8695_BIT_CONT_CTRL_OPEN_PLAYBACK);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_F0_DETECT_MASK, AW8695_BIT_CONT_CTRL_F0_DETECT_ENABLE);

	/* LPF */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DATCTRL,
			      AW8695_BIT_DATCTRL_FC_MASK, AW8695_BIT_DATCTRL_FC_1000HZ);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DATCTRL,
			      AW8695_BIT_DATCTRL_LPF_ENABLE_MASK, AW8695_BIT_DATCTRL_LPF_ENABLE);

	/* LRA OSC Source */
	if (aw8695->f0_cali_flag == AW8695_HAPTIC_CALI_F0) {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_ANACTRL,
				      AW8695_BIT_ANACTRL_LRA_SRC_MASK, AW8695_BIT_ANACTRL_LRA_SRC_REG);
	} else {
		aw8695_i2c_write_bits(aw8695, AW8695_REG_ANACTRL,
				      AW8695_BIT_ANACTRL_LRA_SRC_MASK, AW8695_BIT_ANACTRL_LRA_SRC_EFUSE);
	}

	/* preset f0 */
	aw8695_haptic_set_f0_preset(aw8695);

	/* beme config */
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHH_H, 0x10);
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHH_L, 0x08);
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHL_H, 0x03);
	aw8695_i2c_write(aw8695, AW8695_REG_BEMF_VTHL_L, 0xf8);

	/* f0 driver level */
	aw8695_i2c_write(aw8695, AW8695_REG_DRV_LVL, aw8695->cont_drv_lvl);

	/* f0 trace parameter */
	f0_pre_num = 0x05;
	f0_wait_num = 0x02;
	f0_repeat_num = 0x0d;
	f0_trace_num = 0x0f;
	aw8695_i2c_write(aw8695, AW8695_REG_NUM_F0_1, (f0_pre_num << 4) | (f0_wait_num << 0));
	aw8695_i2c_write(aw8695, AW8695_REG_NUM_F0_2, (f0_repeat_num << 0));
	aw8695_i2c_write(aw8695, AW8695_REG_NUM_F0_3, (f0_trace_num << 0));

	/* clear aw8695 interrupt */
	ret = aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);

#ifdef CONFIG_AF_NOISE_ELIMINATION
	pr_info("%s: %d: mot_actuator_on_vibrate_start, duration=%d, haptic_mode=%d, play_mode=%hhu \n", __func__,__LINE__
		,aw8695->duration,aw8695->haptic_mode,aw8695->play_mode);
	mot_actuator_on_vibrate_start();
#endif

	/* play go and start f0 calibration */
	aw8695_haptic_play_go(aw8695, true);

	/* f0 trace time */
	t_f0_ms = 1000 * 10 / aw8695->f0_pre;
	t_f0_trace_ms = t_f0_ms * (f0_pre_num + f0_wait_num + (f0_trace_num + f0_wait_num) * (f0_repeat_num - 1));
	msleep(t_f0_trace_ms);

	for (i = 0; i < f0_cali_cnt; i++) {
		ret = aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
		/* f0 calibrate done */
		if (reg_val & 0x01) {
			aw8695_haptic_read_f0(aw8695);
			aw8695_haptic_read_beme(aw8695);
			break;
		}
		msleep(10);
		pr_info("%s f0 cali sleep 10ms\n", __func__);
	}

	if (i == f0_cali_cnt) {
		ret = -1;
	} else {
		ret = 0;
	}

	/* restore default config */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8695_CONT_PLAYBACK_MODE);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_CONT_CTRL,
			      AW8695_BIT_CONT_CTRL_F0_DETECT_MASK, AW8695_BIT_CONT_CTRL_F0_DETECT_DISABLE);

	return ret;
}

static int aw8695_haptic_f0_calibration(struct aw8695 *aw8695)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	int f0_dft_step = 0;

	pr_info("%s enter\n", __func__);

	aw8695->f0_cali_flag = AW8695_HAPTIC_CALI_F0;

	if (aw8695_haptic_get_f0(aw8695)) {
		pr_err("%s get f0 error, user defafult f0\n", __func__);
	} else {
		/* max and min limit */
		f0_limit = aw8695->f0;
		if (aw8695->f0 * 100 < aw8695->f0_pre * (100 - aw8695->f0_cali_percen)) {
			f0_limit = aw8695->f0_pre * (100 - aw8695->f0_cali_percen) / 100;
		}
		if (aw8695->f0 * 100 > aw8695->f0_pre * (100 + aw8695->f0_cali_percen)) {
			f0_limit = aw8695->f0_pre * (100 + aw8695->f0_cali_percen) / 100;
		}

		/* calculate cali step */
		f0_cali_step = 10000 * ((int)f0_limit - (int)aw8695->f0_pre) / ((int)f0_limit * 25);
		pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);

		/* get default cali step */
		aw8695_i2c_read(aw8695, AW8695_REG_TRIM_LRA, &reg_val);
		if (reg_val & 0x20) {
			f0_dft_step = reg_val - 0x40;
		} else {
			f0_dft_step = reg_val;
		}
		pr_debug("%s f0_dft_step=%d\n", __func__, f0_dft_step);

		/* get new cali step */
		f0_cali_step += f0_dft_step;
		pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);

		if (f0_cali_step > 31) {
			f0_cali_step = 31;
		} else if (f0_cali_step < -32) {
			f0_cali_step = -32;
		}
		f0_cali_lra = (char)f0_cali_step;
		pr_debug("%s f0_cali_lra=%d\n", __func__, f0_cali_lra);

		/* get cali step complement code*/
		if (f0_cali_lra < 0) {
			f0_cali_lra += 0x40;
		}
		pr_debug("%s reg f0_cali_lra=%d\n", __func__, f0_cali_lra);

		/* update cali step */
		aw8695_i2c_write(aw8695, AW8695_REG_TRIM_LRA, (char)f0_cali_lra);
		aw8695_i2c_read(aw8695, AW8695_REG_TRIM_LRA, &reg_val);
		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val);
	}

	/* restore default work mode */
	aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_STANDBY_MODE);
	aw8695->play_mode = AW8695_HAPTIC_RAM_MODE;
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_PLAY_MODE_MASK, AW8695_BIT_SYSCTRL_PLAY_MODE_RAM);
	aw8695_haptic_stop(aw8695);

#ifdef CONFIG_AF_NOISE_ELIMINATION
	pr_info("%s: %d: mot_actuator_on_vibrate_stop, duration=%d, haptic_mode=%d, play_mode=%hhu \n", __func__,__LINE__
		,aw8695->duration,aw8695->haptic_mode,aw8695->play_mode);
	mot_actuator_on_vibrate_stop();
#endif

	return ret;
}

/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw8695_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_aw8695;

	return 0;
}

static int aw8695_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw8695_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aw8695 *aw8695 = (struct aw8695 *)file->private_data;

	int ret = 0;

	dev_info(aw8695->dev, "%s: cmd=0x%x, arg=0x%lx\n",
		 __func__, cmd, arg);

	mutex_lock(&aw8695->lock);

	if (_IOC_TYPE(cmd) != AW8695_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw8695->dev, "%s: cmd magic err\n",
			__func__);
		return -EINVAL;
	}

	switch (cmd) {
	default:
		dev_err(aw8695->dev, "%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&aw8695->lock);

	return ret;
}

static ssize_t aw8695_file_read(struct file *filp, char *buff, size_t len, loff_t *offset)
{
	struct aw8695 *aw8695 = (struct aw8695 *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw8695->lock);

	dev_info(aw8695->dev, "%s: len=%zu\n", __func__, len);

	switch (aw8695->fileops.cmd) {
	case AW8695_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw8695_i2c_read(aw8695, aw8695->fileops.reg + i, &reg_val);
				pbuff[i] = reg_val;
			}
			for (i = 0; i < len; i++) {
				dev_info(aw8695->dev, "%s: pbuff[%d]=0x%02x\n",
					 __func__, i, pbuff[i]);
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw8695->dev, "%s: copy to user fail\n", __func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8695->dev, "%s: alloc memory fail\n", __func__);
		}
		break;
	default:
		dev_err(aw8695->dev, "%s, unknown cmd %d \n", __func__, aw8695->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8695->lock);


	return len;
}

static ssize_t aw8695_file_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	struct aw8695 *aw8695 = (struct aw8695 *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw8695->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	ret = copy_from_user(pbuff, buff, len);
	if (ret) {
		dev_err(aw8695->dev, "%s: copy from user fail\n", __func__);
		return len;
	}

	for (i = 0; i < len; i++) {
		dev_info(aw8695->dev, "%s: pbuff[%d]=0x%02x\n",
			 __func__, i, pbuff[i]);
	}

	mutex_lock(&aw8695->lock);

	aw8695->fileops.cmd = pbuff[0];

	switch (aw8695->fileops.cmd) {
	case AW8695_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw8695->fileops.reg = pbuff[1];
		} else {
			dev_err(aw8695->dev, "%s: read cmd len %zu err\n", __func__, len);
		}
		break;
	case AW8695_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			for (i = 0; i < len - 2; i++) {
				dev_info(aw8695->dev, "%s: write reg0x%02x=0x%02x\n",
					 __func__, pbuff[1] + i, pbuff[i + 2]);
				aw8695_i2c_write(aw8695, pbuff[1] + i, pbuff[2 + i]);
			}
		} else {
			dev_err(aw8695->dev, "%s: write cmd len %zu err\n", __func__, len);
		}
		break;
	default:
		dev_err(aw8695->dev, "%s, unknown cmd %d \n", __func__, aw8695->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8695->lock);

	if (pbuff != NULL) {
		kfree(pbuff);
	}
	return len;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8695_file_read,
	.write = aw8695_file_write,
	.unlocked_ioctl = aw8695_file_unlocked_ioctl,
	.open = aw8695_file_open,
	.release = aw8695_file_release,
};

static struct miscdevice aw8695_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8695_HAPTIC_NAME,
	.fops = &fops,
};

static int aw8695_haptic_init(struct aw8695 *aw8695)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	ret = misc_register(&aw8695_haptic_misc);
	if (ret) {
		dev_err(aw8695->dev,  "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* haptic audio */
	aw8695->haptic_audio.delay_val = 20833;
	aw8695->haptic_audio.timer_val = 20833;

	hrtimer_init(&aw8695->haptic_audio.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8695->haptic_audio.timer.function = aw8695_haptic_audio_timer_func;
	INIT_WORK(&aw8695->haptic_audio.work, aw8695_haptic_audio_work_routine);

	mutex_init(&aw8695->haptic_audio.lock);


	/* haptic init */
	mutex_lock(&aw8695->lock);

	aw8695->activate_mode = AW8695_HAPTIC_ACTIVATE_RAM_MODE;

	ret = aw8695_i2c_read(aw8695, AW8695_REG_WAVSEQ1, &reg_val);
	aw8695->index = reg_val & 0x7F;
	ret = aw8695_i2c_read(aw8695, AW8695_REG_DATDBG, &reg_val);
	aw8695->gain = reg_val & 0xFF;
	ret = aw8695_i2c_read(aw8695, AW8695_REG_BSTDBG4, &reg_val);
	aw8695->vmax = (reg_val >> 1) & 0x1F;
	for (i = 0; i < AW8695_SEQUENCER_SIZE; i++) {
		ret = aw8695_i2c_read(aw8695, AW8695_REG_WAVSEQ1 + i, &reg_val);
		aw8695->seq[i] = reg_val;
	}

	aw8695_haptic_play_mode(aw8695, AW8695_HAPTIC_STANDBY_MODE);

	aw8695_haptic_set_pwm(aw8695, AW8695_PWM_24K);

	aw8695_i2c_write(aw8695, AW8695_REG_BSTDBG1, 0x30);
	aw8695_i2c_write(aw8695, AW8695_REG_BSTDBG2, 0xeb);
	aw8695_i2c_write(aw8695, AW8695_REG_BSTDBG3, 0xd4);
	aw8695_i2c_write(aw8695, AW8695_REG_TSET, 0x12);
	aw8695_i2c_write(aw8695, AW8695_REG_R_SPARE, 0x68);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_ANADBG,
			      AW8695_BIT_ANADBG_IOC_MASK, AW8695_BIT_ANADBG_IOC_4P65A);

	aw8695_haptic_set_bst_peak_cur(aw8695, AW8695_DEFAULT_PEAKCUR);

	aw8695_haptic_swicth_motorprotect_config(aw8695, 0x00, 0x00);

	aw8695_haptic_auto_boost_config(aw8695, false);

	aw8695_haptic_trig_param_init(aw8695);
	aw8695_haptic_trig_param_config(aw8695);

	aw8695_haptic_os_calibration(aw8695);

	aw8695_haptic_cont_vbat_mode(aw8695,
				     AW8695_HAPTIC_CONT_VBAT_HW_COMP_MODE);
	aw8695->ram_vbat_comp = AW8695_HAPTIC_RAM_VBAT_COMP_ENABLE;

	mutex_unlock(&aw8695->lock);


	/* f0 calibration */
	mutex_lock(&aw8695->lock);
	aw8695_haptic_f0_calibration(aw8695);
	mutex_unlock(&aw8695->lock);

	return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/

static void aw8695_rtp_play(struct aw8695 *aw8695, int value)
{
	aw8695_haptic_stop(aw8695);
	aw8695_haptic_set_rtp_aei(aw8695, false);
	aw8695_interrupt_clear(aw8695);
	if (value < (sizeof(aw8695_rtp_name) / AW8695_RTP_NAME_MAX)) {
		aw8695->rtp_file_num = value;
		if (value) {
			schedule_work(&aw8695->rtp_work);
		}
	} else {
		pr_err("%s: rtp_file_num 0x%02x over max value \n", __func__, aw8695->rtp_file_num);
	}
}

static void aw8695_haptic_context(struct aw8695 *aw8695, enum aw8695_haptic_mode cmd)
{
	int t_top = 0;
	if (!gpio_is_valid(aw8695->haptic_context_gpio)) {
		pr_debug("%s haptic context gpio is invalid \n", __func__);
		return;
	}

	t_top = gpio_get_value(aw8695->haptic_context_gpio);
	if (t_top) {
		switch (cmd) {
		case HAPTIC_RTP:
			aw8695->gain = 0x80;
			break;
		case HAPTIC_SHORT:
			aw8695->gain = 0x80;
			break;
		case HAPTIC_LONG:
			aw8695->gain = aw8695->long_gain_reduced;
			break;
		default:
			break;
		}
	}
}

static void aw8695_vibrate(struct aw8695 *aw8695, int value)
{
	int seq = 0;
	mutex_lock(&aw8695->lock);

#ifdef AW8695_REPEAT_RTP_PLAYING
	if (aw8695->haptic_mode == HAPTIC_RTP_LOOP) {
		aw8695->haptic_mode = HAPTIC_RTP;
		hrtimer_cancel(&aw8695->timer);
	}
#endif
	aw8695_haptic_stop(aw8695);
	if (aw8695->index == 0x02)
		PM_RELAX(aw8695->ws);

	seq = aw8695->seq[0];
	pr_info("%s: value=%d, seq=%d, index=%x\n", __FUNCTION__, value, seq, aw8695->index);

	if (value > 0 || seq > 2) {

		if (seq >= AW8695_SEQ_NO_RTP_BASE) {
			aw8695->haptic_mode = HAPTIC_RTP;
			aw8695->gain = 0x80;
#ifdef AW8695_REPEAT_RTP_PLAYING
			if (seq >= AW8695_SEQ_NO_RTP_BASE + AW8695_SEQ_NO_RTP_REPEAT) {
				aw8695->haptic_mode = HAPTIC_RTP_LOOP;
				seq -= AW8695_SEQ_NO_RTP_REPEAT;

				value = AW8695_SEQ_NO_RTP_STOP;
				PM_WAKEUP_EVENT(aw8695->ws, value + 100);
				/* run ms timer */
				hrtimer_cancel(&aw8695->timer);
				hrtimer_start(&aw8695->timer,
					ktime_set(value / 1000, (value % 1000) * 1000000),
					HRTIMER_MODE_REL);
			}
#endif
		} else if (value < 100 || seq > 2) {
			aw8695->haptic_mode = HAPTIC_SHORT;
			aw8695->gain = 0x80;
		} else {
			aw8695->haptic_mode = HAPTIC_LONG;
			aw8695->gain = aw8695->long_gain_normal;
		}
		if(!aw8695->factory_mode)
			aw8695_haptic_context(aw8695,aw8695->haptic_mode);
		if (aw8695->debugfs_debug)
			aw8695_haptic_set_gain(aw8695, aw8695->gain_debug);
		else
			aw8695_haptic_set_gain(aw8695, aw8695->gain);

		switch (aw8695->haptic_mode) {

		case HAPTIC_RTP:
		case HAPTIC_RTP_LOOP:
			aw8695_rtp_play(aw8695, seq - AW8695_SEQ_NO_RTP_BASE);
			break;
		case HAPTIC_SHORT:
			if (aw8695->seq[0] == 0)
				aw8695->seq[0] = 0x01;
			aw8695->index = 0x01;
#ifdef CONFIG_AW8965_VIBRATOR_SHORT_WAV_ENABLE
			/*If duration < 100ms, use four waveforms corresponding to weakest, weak, medium, strong*/
			if(value < 12){
				aw8695_haptic_set_bst_vol(aw8695, 0x2);
				aw8695_haptic_set_wav_seq(aw8695, 0x00, 4);
			}
			else if(value < 25) {
				aw8695_haptic_set_bst_vol(aw8695, 0x1e);
				aw8695_haptic_set_wav_seq(aw8695, 0x00, 4);
			}
			else if(value < 38) {
				aw8695_haptic_set_bst_vol(aw8695, 0x1);
				aw8695_haptic_set_wav_seq(aw8695, 0x00, 3);
			}
			else if(value < 100) {
				aw8695_haptic_set_bst_vol(aw8695, 0x1f);
				aw8695_haptic_set_wav_seq(aw8695, 0x00, 1);
			}
#else
			aw8695_haptic_set_wav_seq(aw8695, 0x00, aw8695->seq[0]);
#endif
			aw8695_haptic_set_wav_loop(aw8695, 0x00, 0x00);
			aw8695_haptic_ram_vbat_comp(aw8695, false);
			aw8695_haptic_play_wav_seq(aw8695, 0x01);
			break;
		case HAPTIC_LONG:
#ifdef CONFIG_AF_NOISE_ELIMINATION
			pr_info("%s: %d: mot_actuator_on_vibrate_start, duration=%d, haptic_mode=%d, play_mode=%hhu \n", __func__,__LINE__
				,aw8695->duration,aw8695->haptic_mode,aw8695->play_mode);
			is_af_enabled = true;
			mot_actuator_on_vibrate_start();
#endif
			aw8695->duration = value;
			/* wav index config */
			aw8695->index = 0x02;
			aw8695_haptic_set_repeat_wav_seq(aw8695, aw8695->index);
			PM_WAKEUP_EVENT(aw8695->ws, value + 100);
			/* run ms timer */
			hrtimer_cancel(&aw8695->timer);
			aw8695->state = 0x01;
			if (aw8695->state)
			{
				hrtimer_start(&aw8695->timer,
					ktime_set(aw8695->duration / 1000, (value % 1000) * 1000000),
					HRTIMER_MODE_REL);
			}
			schedule_work(&aw8695->vibrator_work);
			break;
		default:
			break;
		}

		/* Restore to default short waveform */
		if (seq > 2)
			aw8695->seq[0] = 0;
	}

    mutex_unlock(&aw8695->lock);
}
#ifdef TIMED_OUTPUT
static int aw8695_vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw8695 *aw8695 = container_of(dev, struct aw8695, to_dev);

	if (hrtimer_active(&aw8695->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw8695->timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void aw8695_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw8695 *aw8695 = container_of(dev, struct aw8695, to_dev);

	mutex_lock(&aw8695->lock);

	pr_debug("%s enter\n", __func__);

	aw8695_haptic_stop(aw8695);

	if (value > 0) {
		aw8695_haptic_ram_vbat_comp(aw8695, false);
		aw8695_haptic_play_wav_seq(aw8695, value);
	}

	mutex_unlock(&aw8695->lock);

	pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness aw8695_haptic_brightness_get(struct led_classdev *cdev)
{
	struct aw8695 *aw8695 =
		container_of(cdev, struct aw8695, cdev);

	return aw8695->amplitude;
}

static void aw8695_haptic_brightness_set(struct led_classdev *cdev,
		enum led_brightness level)
{
	struct aw8695 *aw8695 =
		container_of(cdev, struct aw8695, cdev);

	aw8695->amplitude = level;

	mutex_lock(&aw8695->lock);

	aw8695_haptic_stop(aw8695);
	if (aw8695->amplitude > 0) {
		aw8695_haptic_ram_vbat_comp(aw8695, false);
		aw8695_haptic_play_wav_seq(aw8695, aw8695->amplitude);
	}

	mutex_unlock(&aw8695->lock);

}
#endif

static ssize_t aw8695_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8695->state);
}

static ssize_t aw8695_state_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8695_duration_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw8695->timer)) {
		time_rem = hrtimer_get_remaining(&aw8695->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw8695_duration_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8695->duration = val;

	return count;
}

static ssize_t aw8695_activate_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8695->state);
}

static ssize_t aw8695_activate_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif

	unsigned int val = 0;
	int rc = 0;;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	pr_info("%s: value=%d\n", __FUNCTION__, val);
	aw8695->state = val;
	if (aw8695->state)
		aw8695_vibrate(aw8695, aw8695->duration);
	else
		aw8695_vibrate(aw8695, 0);

	return count;
}

static ssize_t aw8695_activate_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "activate_mode=%d\n", aw8695->activate_mode);
}

static ssize_t aw8695_activate_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8695->lock);
	aw8695->activate_mode = val;
	mutex_unlock(&aw8695->lock);
	return count;
}

static ssize_t aw8695_index_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned char reg_val = 0;
	aw8695_i2c_read(aw8695, AW8695_REG_WAVSEQ1, &reg_val);
	aw8695->index = reg_val;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8695->index);
}

static ssize_t aw8695_index_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw8695->lock);
	aw8695->index = val;
	aw8695_haptic_set_repeat_wav_seq(aw8695, aw8695->index);
	mutex_unlock(&aw8695->lock);
	return count;
}

static ssize_t aw8695_vmax_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8695->vmax);
}

static ssize_t aw8695_vmax_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw8695->lock);
	aw8695->vmax = val;
	aw8695_haptic_set_bst_vol(aw8695, aw8695->vmax);
	mutex_unlock(&aw8695->lock);
	return count;
}

static ssize_t aw8695_gain_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8695->gain_debug);
}

static ssize_t aw8695_gain_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw8695->lock);
	if (val > 0)
		aw8695->debugfs_debug = true;
	else
		aw8695->debugfs_debug = false;
	aw8695->gain_debug = val;
	aw8695_haptic_set_gain(aw8695, aw8695->gain_debug);
	mutex_unlock(&aw8695->lock);
	return count;
}

static ssize_t aw8695_seq_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8695_SEQUENCER_SIZE; i++) {
		aw8695_i2c_read(aw8695, AW8695_REG_WAVSEQ1 + i, &reg_val);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d: 0x%02x\n", i + 1, reg_val);
		aw8695->seq[i] |= reg_val;
	}
	return count;
}

static ssize_t aw8695_seq_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif

	unsigned int i = 0;
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_info("%s: value=%x\n", __FUNCTION__, val);

	mutex_lock(&aw8695->lock);
	for(i=0; i<4; i++) {
		aw8695->seq[i] = (val>>((AW8695_WAV_SEQ_SIZE-i-1)*8)) & 0xFF;
		aw8695_haptic_set_wav_seq(aw8695, i, aw8695->seq[i]);
	}
	mutex_unlock(&aw8695->lock);

	return count;
}

static ssize_t aw8695_loop_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8695_SEQUENCER_LOOP_SIZE; i++) {
		aw8695_i2c_read(aw8695, AW8695_REG_WAVLOOP1 + i, &reg_val);
		aw8695->loop[i * 2 + 0] = (reg_val >> 4) & 0x0F;
		aw8695->loop[i * 2 + 1] = (reg_val >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1, aw8695->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 2, aw8695->loop[i * 2 + 1]);
	}
	return count;
}

static ssize_t aw8695_loop_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_debug("%s: seq%d loop=0x%x\n", __FUNCTION__, databuf[0], databuf[1]);
		mutex_lock(&aw8695->lock);
		aw8695->loop[databuf[0]] = (unsigned char)databuf[1];
		aw8695_haptic_set_wav_loop(aw8695, (unsigned char)databuf[0],
					   aw8695->loop[databuf[0]]);
		mutex_unlock(&aw8695->lock);
	}

	return count;
}

static ssize_t aw8695_reg_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW8695_REG_MAX; i ++) {
		if (!(aw8695_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw8695_i2c_read(aw8695, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}

static ssize_t aw8695_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8695_i2c_write(aw8695, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8695_rtp_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "rtp play: %d\n", aw8695->rtp_cnt);

	return len;
}

static ssize_t aw8695_rtp_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw8695_rtp_play(aw8695, val);

	return count;
}

static ssize_t aw8695_ram_update_show(struct device *dev, struct device_attribute *attr,
				      char *buf)
{

	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "sram update mode\n");
	return len;
}

static ssize_t aw8695_ram_update_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		aw8695_ram_update(aw8695);
	}
	return count;
}

static ssize_t aw8695_f0_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8695->lock);
	aw8695->f0_cali_flag = AW8695_HAPTIC_LRA_F0;
	aw8695_haptic_get_f0(aw8695);
	mutex_unlock(&aw8695->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 lra f0 = %d\n", aw8695->f0);
	return len;
}

static ssize_t aw8695_f0_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{

	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t aw8695_cali_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	mutex_lock(&aw8695->lock);
	aw8695->f0_cali_flag = AW8695_HAPTIC_CALI_F0;
	aw8695_haptic_get_f0(aw8695);
	mutex_unlock(&aw8695->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cali f0 = %d\n", aw8695->f0);
	return len;
}

static ssize_t aw8695_cali_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		mutex_lock(&aw8695->lock);
		aw8695_haptic_f0_calibration(aw8695);
		mutex_unlock(&aw8695->lock);
	}
	return count;
}

static ssize_t aw8695_cont_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	aw8695_haptic_read_cont_f0(aw8695);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cont f0 = %d\n", aw8695->cont_f0);
	return len;
}

static ssize_t aw8695_cont_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw8695_haptic_stop(aw8695);
	if (val) {
		aw8695_haptic_cont(aw8695);
	}
	return count;
}


static ssize_t aw8695_cont_td_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cont delay time = 0x%04x\n", aw8695->cont_td);
	return len;
}

static ssize_t aw8695_cont_td_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[1] = {0};
	if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw8695->cont_td = databuf[0];
		aw8695_i2c_write(aw8695, AW8695_REG_TD_H, (unsigned char)(databuf[0] >> 8));
		aw8695_i2c_write(aw8695, AW8695_REG_TD_L, (unsigned char)(databuf[0] >> 0));
	}
	return count;
}

static ssize_t aw8695_cont_drv_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cont drv level = %d\n", aw8695->cont_drv_lvl);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cont drv level overdrive= %d\n", aw8695->cont_drv_lvl_ov);
	return len;
}

static ssize_t aw8695_cont_drv_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[2] = {0, 0};
	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw8695->cont_drv_lvl = databuf[0];
		aw8695_i2c_write(aw8695, AW8695_REG_DRV_LVL, aw8695->cont_drv_lvl);
		aw8695->cont_drv_lvl_ov = databuf[1];
		aw8695_i2c_write(aw8695, AW8695_REG_DRV_LVL_OV, aw8695->cont_drv_lvl_ov);
	}
	return count;
}

static ssize_t aw8695_cont_num_brk_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cont break num = %d\n", aw8695->cont_num_brk);
	return len;
}

static ssize_t aw8695_cont_num_brk_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[1] = {0};
	if (1 == sscanf(buf, "%d", &databuf[0])) {
		aw8695->cont_num_brk = databuf[0];
		if (aw8695->cont_num_brk > 7) {
			aw8695->cont_num_brk = 7;
		}
		aw8695_i2c_write_bits(aw8695, AW8695_REG_BEMF_NUM,
				      AW8695_BIT_BEMF_NUM_BRK_MASK, aw8695->cont_num_brk);
	}
	return count;
}

static ssize_t aw8695_cont_zc_thr_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695 cont zero cross thr = 0x%04x\n", aw8695->cont_zc_thr);
	return len;
}

static ssize_t aw8695_cont_zc_thr_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[1] = {0};
	if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw8695->cont_zc_thr = databuf[0];
		aw8695_i2c_write(aw8695, AW8695_REG_ZC_THRSH_H, (unsigned char)(databuf[0] >> 8));
		aw8695_i2c_write(aw8695, AW8695_REG_ZC_THRSH_L, (unsigned char)(databuf[0] >> 0));
	}
	return count;
}

static ssize_t aw8695_vbat_monitor_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8695->lock);
	aw8695_haptic_stop(aw8695);
	aw8695_haptic_get_vbat(aw8695);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat=%dmV\n", aw8695->vbat);
	mutex_unlock(&aw8695->lock);

	return len;
}

static ssize_t aw8695_vbat_monitor_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8695_lra_resistance_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	mutex_lock(&aw8695->lock);
	aw8695_haptic_stop(aw8695);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BYPASS);


	aw8695_i2c_write_bits(aw8695, AW8695_REG_ANACTRL,
			      AW8695_BIT_ANACTRL_HD_PD_MASK, AW8695_BIT_ANACTRL_HD_HZ_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_D2SCFG,
			      AW8695_BIT_D2SCFG_CLK_ADC_MASK, AW8695_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_DETCTRL,
			      AW8695_BIT_DETCTRL_RL_OS_MASK, AW8695_BIT_DETCTRL_RL_DETECT);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DETCTRL,
			      AW8695_BIT_DETCTRL_DIAG_GO_MASK, AW8695_BIT_DETCTRL_DIAG_GO_ENABLE);
	msleep(3);
	aw8695_i2c_read(aw8695, AW8695_REG_RLDET, &reg_val);
	aw8695->lra = 298 * reg_val;
	len += snprintf(buf + len, PAGE_SIZE - len, "r_lra=%dmohm\n", aw8695->lra);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_ANACTRL,
			      AW8695_BIT_ANACTRL_HD_PD_MASK, AW8695_BIT_ANACTRL_HD_PD_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_D2SCFG,
			      AW8695_BIT_D2SCFG_CLK_ADC_MASK, AW8695_BIT_D2SCFG_CLK_ASC_6MHZ);

	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_OFF);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_BST_MODE_MASK, AW8695_BIT_SYSCTRL_BST_MODE_BOOST);
	mutex_unlock(&aw8695->lock);

	return len;
}

static ssize_t aw8695_lra_resistance_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8695_auto_boost_show(struct device *dev, struct device_attribute *attr,
				      char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "auto_boost=%d\n", aw8695->auto_boost);

	return len;
}


static ssize_t aw8695_auto_boost_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8695->lock);
	aw8695_haptic_stop(aw8695);
	aw8695_haptic_auto_boost_config(aw8695, val);
	mutex_unlock(&aw8695->lock);

	return count;
}

static ssize_t aw8695_prctmode_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	aw8695_i2c_read(aw8695, AW8695_REG_RLDET, &reg_val);

	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode=%d\n", reg_val & 0x20);
	return len;
}


static ssize_t aw8695_prctmode_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[2] = {0, 0};
	unsigned int addr = 0;
	unsigned int val = 0;
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		addr = databuf[0];
		val = databuf[1];
		mutex_lock(&aw8695->lock);
		aw8695_haptic_swicth_motorprotect_config(aw8695, addr, val);
		mutex_unlock(&aw8695->lock);
	}
	return count;
}

static ssize_t aw8695_trig_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	for (i = 0; i < AW8695_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: enable=%d, default_level=%d, dual_edge=%d, frist_seq=%d, second_seq=%d\n",
				i + 1, aw8695->trig[i].enable, aw8695->trig[i].default_level, aw8695->trig[i].dual_edge,
				aw8695->trig[i].frist_seq, aw8695->trig[i].second_seq);
	}

	return len;
}

static ssize_t aw8695_trig_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[6] = {0};
	if (sscanf(buf, "%d %d %d %d %d %d",
		   &databuf[0], &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
		pr_debug("%s: %d, %d, %d, %d, %d, %d\n", __func__,
			 databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
		if (databuf[0] > 3) {
			databuf[0] = 3;
		}
		if (databuf[0] > 0) {
			databuf[0] -= 1;
		}
		aw8695->trig[databuf[0]].enable = databuf[1];
		aw8695->trig[databuf[0]].default_level = databuf[2];
		aw8695->trig[databuf[0]].dual_edge = databuf[3];
		aw8695->trig[databuf[0]].frist_seq = databuf[4];
		aw8695->trig[databuf[0]].second_seq = databuf[5];
		mutex_lock(&aw8695->lock);
		aw8695_haptic_trig_param_config(aw8695);
		aw8695_haptic_trig_enable_config(aw8695);
		mutex_unlock(&aw8695->lock);
	}
	return count;
}

static ssize_t aw8695_ram_vbat_comp_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp=%d\n", aw8695->ram_vbat_comp);

	return len;
}


static ssize_t aw8695_ram_vbat_comp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8695->lock);
	if (val) {
		aw8695->ram_vbat_comp = AW8695_HAPTIC_RAM_VBAT_COMP_ENABLE;
	} else {
		aw8695->ram_vbat_comp = AW8695_HAPTIC_RAM_VBAT_COMP_DISABLE;
	}
	mutex_unlock(&aw8695->lock);

	return count;
}

static ssize_t aw8695_haptic_audio_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", aw8695->haptic_audio.cnt);
	return len;
}

static ssize_t aw8695_haptic_audio_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[6] = {0};

	if (6 == sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1], &databuf[2],
			&databuf[3], &databuf[4], &databuf[5])) {
		pr_debug("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
			 __func__, databuf[0], databuf[1], databuf[2], databuf[3],
			 databuf[4], databuf[5]);
		mutex_lock(&aw8695->haptic_audio.lock);
		aw8695->haptic_audio.data[(unsigned char)databuf[0]].cmd = (unsigned char)databuf[1];
		aw8695->haptic_audio.data[(unsigned char)databuf[0]].play = (unsigned char)databuf[2];
		aw8695->haptic_audio.data[(unsigned char)databuf[0]].wavseq = (unsigned char)databuf[3];
		aw8695->haptic_audio.data[(unsigned char)databuf[0]].loop = (unsigned char)databuf[4];
		aw8695->haptic_audio.data[(unsigned char)databuf[0]].gain = (unsigned char)databuf[5];
		mutex_unlock(&aw8695->haptic_audio.lock);

		if (aw8695->haptic_audio.data[aw8695->haptic_audio.cnt].cmd == 0xff) {
			pr_info("%s: haptic_audio stop\n", __func__);
			if (hrtimer_active(&aw8695->haptic_audio.timer)) {
				pr_info("%s: cancel haptic_audio_timer\n", __func__);
				hrtimer_cancel(&aw8695->haptic_audio.timer);
				aw8695->haptic_audio.cnt = 0;
				aw8695_haptic_set_gain(aw8695, 0x80);
			}
		} else {
			if (hrtimer_active(&aw8695->haptic_audio.timer)) {
			} else {
				pr_info("%s: start haptic_audio_timer\n", __func__);
				hrtimer_start(&aw8695->haptic_audio.timer,
					      ktime_set(aw8695->haptic_audio.delay_val / 1000000,
							(aw8695->haptic_audio.delay_val % 1000000) * 1000),
					      HRTIMER_MODE_REL);
			}
		}
	}
	return count;
}


static ssize_t aw8695_haptic_audio_time_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "haptic_audio.delay_val=%dus\n", aw8695->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len, "haptic_audio.timer_val=%dus\n", aw8695->haptic_audio.timer_val);
	return len;
}

static ssize_t aw8695_haptic_audio_time_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(to_dev, struct aw8695, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695 *aw8695 = container_of(cdev, struct aw8695, cdev);
#endif
	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw8695->haptic_audio.delay_val = databuf[0];
		aw8695->haptic_audio.timer_val = databuf[1];
	}
	return count;
}


static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw8695_state_show, aw8695_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw8695_duration_show, aw8695_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw8695_activate_show, aw8695_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw8695_activate_mode_show, aw8695_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw8695_index_show, aw8695_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw8695_vmax_show, aw8695_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw8695_gain_show, aw8695_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw8695_seq_show, aw8695_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw8695_loop_show, aw8695_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw8695_reg_show, aw8695_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw8695_rtp_show, aw8695_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw8695_ram_update_show, aw8695_ram_update_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, aw8695_f0_show, aw8695_f0_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw8695_cali_show, aw8695_cali_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw8695_cont_show, aw8695_cont_store);
static DEVICE_ATTR(cont_td, S_IWUSR | S_IRUGO, aw8695_cont_td_show, aw8695_cont_td_store);
static DEVICE_ATTR(cont_drv, S_IWUSR | S_IRUGO, aw8695_cont_drv_show, aw8695_cont_drv_store);
static DEVICE_ATTR(cont_num_brk, S_IWUSR | S_IRUGO, aw8695_cont_num_brk_show, aw8695_cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, S_IWUSR | S_IRUGO, aw8695_cont_zc_thr_show, aw8695_cont_zc_thr_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw8695_vbat_monitor_show, aw8695_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO, aw8695_lra_resistance_show, aw8695_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw8695_auto_boost_show, aw8695_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw8695_prctmode_show, aw8695_prctmode_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8695_trig_show, aw8695_trig_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, aw8695_ram_vbat_comp_show, aw8695_ram_vbat_comp_store);
static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw8695_haptic_audio_show, aw8695_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO, aw8695_haptic_audio_time_show, aw8695_haptic_audio_time_store);


static struct attribute *aw8695_vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_register.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_cont.attr,
	&dev_attr_cont_td.attr,
	&dev_attr_cont_drv.attr,
	&dev_attr_cont_num_brk.attr,
	&dev_attr_cont_zc_thr.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_trig.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	NULL
};

static struct attribute_group aw8695_vibrator_attribute_group = {
	.attrs = aw8695_vibrator_attributes
};

static enum hrtimer_restart aw8695_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8695 *aw8695 = container_of(timer, struct aw8695, timer);

	pr_debug("%s enter\n", __func__);
	aw8695->state = 0;
	schedule_work(&aw8695->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw8695_vibrator_work_routine(struct work_struct *work)
{
	struct aw8695 *aw8695 = container_of(work, struct aw8695, vibrator_work);

	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8695->lock);

#ifdef AW8695_REPEAT_RTP_PLAYING
	if (aw8695->haptic_mode == HAPTIC_RTP_LOOP)
		aw8695->haptic_mode = HAPTIC_RTP;
#endif
	aw8695_haptic_stop(aw8695);
	if (aw8695->state) {
		if (aw8695->activate_mode == AW8695_HAPTIC_ACTIVATE_RAM_MODE) {
			aw8695_haptic_ram_vbat_comp(aw8695, true);
			aw8695_haptic_play_repeat_seq(aw8695, true);
		} else if (aw8695->activate_mode == AW8695_HAPTIC_ACTIVATE_CONT_MODE) {
			aw8695_haptic_cont(aw8695);
		} else {
		}
	}
	mutex_unlock(&aw8695->lock);
}

static int aw8695_vibrator_init(struct aw8695 *aw8695)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

#ifdef TIMED_OUTPUT
	aw8695->to_dev.name = "vibrator";
	aw8695->to_dev.get_time = aw8695_vibrator_get_time;
	aw8695->to_dev.enable = aw8695_vibrator_enable;

	ret = timed_output_dev_register(&(aw8695->to_dev));
	if (ret < 0) {
		dev_err(aw8695->dev, "%s: fail to create timed output dev\n",
			__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw8695->to_dev.dev->kobj, &aw8695_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8695->dev, "%s error creating sysfs attr files\n", __func__);
		return ret;
	}
#else
	aw8695->cdev.name = "vibrator";
	aw8695->cdev.brightness_get = aw8695_haptic_brightness_get;
	aw8695->cdev.brightness_set = aw8695_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8695->i2c->dev, &aw8695->cdev);
	if (ret < 0) {
		dev_err(aw8695->dev, "%s: fail to create led dev\n",
			__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw8695->cdev.dev->kobj, &aw8695_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8695->dev, "%s error creating sysfs attr files\n", __func__);
		return ret;
	}
#endif
	hrtimer_init(&aw8695->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8695->timer.function = aw8695_vibrator_timer_func;
	INIT_WORK(&aw8695->vibrator_work, aw8695_vibrator_work_routine);

	INIT_WORK(&aw8695->rtp_work, aw8695_rtp_work_routine);

	PM_WAKEUP_REGISTER(aw8695->dev, aw8695->ws, "vibrator");
	if (!aw8695->ws)
		return -ENOMEM;

	mutex_init(&aw8695->lock);

	return 0;
}




/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw8695_interrupt_clear(struct aw8695 *aw8695)
{
	unsigned char reg_val = 0;
	pr_debug("%s enter\n", __func__);
	aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
	pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw8695_interrupt_setup(struct aw8695 *aw8695)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	/* edge int mode */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_DBGCTRL,
			      AW8695_BIT_DBGCTRL_INT_MODE_MASK, AW8695_BIT_DBGCTRL_INT_MODE_EDGE);

	/* int enable */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
			      AW8695_BIT_SYSINTM_BSTERR_MASK, AW8695_BIT_SYSINTM_BSTERR_OFF);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
			      AW8695_BIT_SYSINTM_OV_MASK, AW8695_BIT_SYSINTM_OV_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
			      AW8695_BIT_SYSINTM_UVLO_MASK, AW8695_BIT_SYSINTM_UVLO_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
			      AW8695_BIT_SYSINTM_OCD_MASK, AW8695_BIT_SYSINTM_OCD_EN);
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSINTM,
			      AW8695_BIT_SYSINTM_OT_MASK, AW8695_BIT_SYSINTM_OT_EN);
}

static irqreturn_t aw8695_irq(int irq, void *data)
{
	struct aw8695 *aw8695 = data;
	unsigned char reg_val = 0;
	unsigned char dbg_val = 0;
	unsigned int buf_len = 0;
	unsigned char glb_state = 0;

	pr_debug("%s enter\n", __func__);

	aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	aw8695_i2c_read(aw8695, AW8695_REG_DBGSTAT, &dbg_val);
	pr_info("%s: reg DBGSTAT=0x%x\n", __func__, dbg_val);

	if (reg_val & AW8695_BIT_SYSINT_OVI) {
		pr_err("%s chip ov int error\n", __func__);
	}
	if (reg_val & AW8695_BIT_SYSINT_UVLI) {
		pr_err("%s chip uvlo int error\n", __func__);
	}
	if (reg_val & AW8695_BIT_SYSINT_OCDI) {
		pr_err("%s chip over current int error\n", __func__);
	}
	if (reg_val & AW8695_BIT_SYSINT_OTI) {
		pr_err("%s chip over temperature int error\n", __func__);
	}
	if (reg_val & AW8695_BIT_SYSINT_DONEI) {
		pr_info("%s chip playback done\n", __func__);
	}

	if (reg_val & AW8695_BIT_SYSINT_FF_AEI) {
		pr_debug("%s: aw8695 rtp fifo almost empty int\n", __func__);
		if (aw8695->rtp_init) {
			while ((!aw8695_haptic_rtp_get_fifo_afi(aw8695)) &&
			       (aw8695->play_mode == AW8695_HAPTIC_RTP_MODE)) {
				pr_info("%s: aw8695 rtp mode fifo update, cnt=%d\n",
					__func__, aw8695->rtp_cnt);
				if ((aw8695_rtp->len - aw8695->rtp_cnt) < (aw8695->ram.base_addr >> 2)) {
					buf_len = aw8695_rtp->len - aw8695->rtp_cnt;
				} else {
					buf_len = (aw8695->ram.base_addr >> 2);
				}
				aw8695_i2c_writes(aw8695, AW8695_REG_RTP_DATA,
						  &aw8695_rtp->data[aw8695->rtp_cnt], buf_len);
				aw8695->rtp_cnt += buf_len;
				aw8695_i2c_read(aw8695, AW8695_REG_GLB_STATE, &glb_state);
				if ((aw8695->rtp_cnt == aw8695_rtp->len) || ((glb_state & 0x0f) == 0x00)) {
					pr_info("%s: rtp update complete\n", __func__);
					aw8695_haptic_set_rtp_aei(aw8695, false);
					aw8695->rtp_cnt = 0;
					aw8695->rtp_init = 0;
#ifdef AW8695_REPEAT_RTP_PLAYING
					if (aw8695->haptic_mode == HAPTIC_RTP_LOOP)
						aw8695_rtp_play(aw8695, aw8695->rtp_file_num);
#endif
					break;
				}
			}
		} else {
			pr_err("%s: aw8695 rtp init = %d, init error\n", __func__, aw8695->rtp_init);
		}
	}

	if (reg_val & AW8695_BIT_SYSINT_FF_AFI) {
		pr_debug("%s: aw8695 rtp mode fifo full empty\n", __func__);
	}

	if (aw8695->play_mode != AW8695_HAPTIC_RTP_MODE) {
		aw8695_haptic_set_rtp_aei(aw8695, false);
	}

	aw8695_i2c_read(aw8695, AW8695_REG_SYSINT, &reg_val);
	pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	aw8695_i2c_read(aw8695, AW8695_REG_SYSST, &reg_val);
	pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8695_parse_dt(struct device *dev, struct aw8695 *aw8695,
			   struct device_node *np)
{
	int rc;

	aw8695->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8695->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		return -1;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw8695->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8695->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

	aw8695->haptic_context_gpio = of_get_named_gpio(np, "haptic-context-gpio", 0);
	if (aw8695->haptic_context_gpio < 0) {
		dev_err(dev, "%s: no haptic context gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: haptic context gpio provided ok.\n", __func__);
	}

	aw8695->factorymode_reduce =  of_property_read_bool(np, "factorymode-reduce");
	if (aw8695->factorymode_reduce) {
		dev_err(dev, "%s: reduce on factory mode.\n", __func__);
	} else {
		dev_info(dev, "%s:normal on factory mode.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_f0_pre", &aw8695->f0_pre);
	if (rc) {
		aw8695->f0_pre = 2050; /* 205HZ as default */
		dev_err(dev, "%s: no f0_pre provided.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_f0_cali_percen", &aw8695->f0_cali_percen);
	if (rc) {
		aw8695->f0_cali_percen = 7; /* 7 as default */
		dev_err(dev, "%s: no f0_cali_percen provided.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_cont_drv_lvl", &aw8695->cont_drv_lvl);
	if (rc) {
		aw8695->cont_drv_lvl = 100; /* 100 as default */
		dev_err(dev, "%s: no cont_drv_lvl provided.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_cont_drv_lvl_ov", &aw8695->cont_drv_lvl_ov);
	if (rc) {
		aw8695->cont_drv_lvl_ov = 120; /* 120 as default */
		dev_err(dev, "%s: no cont_drv_lvl_ov provided.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_cont_td", &aw8695->cont_td);
	if (rc) {
		aw8695->cont_td = 0x006c; /* 0x006c as default */
		dev_err(dev, "%s: no cont_td provided.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_cont_zc_thr", &aw8695->cont_zc_thr);
	if (rc) {
		aw8695->cont_zc_thr = 0x0ff1; /* 0x0ff1 as default */
		dev_err(dev, "%s: no cont_zc_thr provided.\n", __func__);
	}

	rc = of_property_read_u32(np, "vib_cont_num_brk", &aw8695->cont_num_brk);
	if (rc) {
		aw8695->cont_num_brk = 3; /* 3 as default */
		dev_err(dev, "%s: no cont_zc_thr provided.\n", __func__);
	}

	rc = of_property_read_s32(np, "long-gain-normal", &aw8695->long_gain_normal);
	if (rc) {
		aw8695->long_gain_normal = 0x80;
		dev_err(dev, "%s: no normal gain value for long vibrating provided.\n", __func__);
	}
	dev_info(dev, "%s: normal gain value for long vibrating is 0x%02x.\n", __func__, aw8695->long_gain_normal);

	rc = of_property_read_s32(np, "long-gain-reduced", &aw8695->long_gain_reduced);
	if (rc) {
		aw8695->long_gain_reduced = 0x80;
		dev_err(dev, "%s: no reduced gain value for long vibrating provided.\n", __func__);
	}
	dev_info(dev, "%s: reduced gain value for long vibrating is 0x%02x.\n", __func__, aw8695->long_gain_reduced);
	return 0;
}

static int aw8695_hw_reset(struct aw8695 *aw8695)
{
	pr_info("%s enter\n", __func__);

	if (aw8695 && gpio_is_valid(aw8695->reset_gpio)) {
		gpio_set_value_cansleep(aw8695->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw8695->reset_gpio, 1);
		msleep(2);
	} else {
		dev_err(aw8695->dev, "%s:  failed\n", __func__);
	}
	return 0;
}


/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw8695_read_chipid(struct aw8695 *aw8695)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		/* hardware reset */
		aw8695_hw_reset(aw8695);

		ret = aw8695_i2c_read(aw8695, AW8695_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw8695->dev, "%s: failed to read register AW8695_REG_ID: %d\n", __func__, ret);
		}
		switch (reg) {
		case AW8695_CHIPID1:
			pr_info("%s aw8695 detected\n", __func__);
			aw8695->chipid = AW8695_CHIPID1;
			/*
			 *aw8695->flags |= AW8695_FLAG_SKIP_INTERRUPTS;
			 */
			aw8695_haptic_softreset(aw8695);
			return 0;
		case AW8695_CHIPID2:
			pr_info("%s aw8697 detected\n", __func__);
			aw8695->chipid = AW8695_CHIPID2;
			aw8695_haptic_softreset(aw8695);
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n", __func__, reg);
			break;
		}
		cnt ++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8695_i2c_reg_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8695 *aw8695 = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8695_i2c_write(aw8695, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8695_i2c_reg_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct aw8695 *aw8695 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW8695_REG_MAX; i ++) {
		if (!(aw8695_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw8695_i2c_read(aw8695, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}
static ssize_t aw8695_i2c_ram_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8695 *aw8695 = dev_get_drvdata(dev);

	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw8695_ram_update(aw8695);
		}
	}

	return count;
}

static ssize_t aw8695_i2c_ram_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct aw8695 *aw8695 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	aw8695_haptic_stop(aw8695);
	/* RAMINIT Enable */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_EN);

	aw8695_i2c_write(aw8695, AW8695_REG_RAMADDRH, (unsigned char)(aw8695->ram.base_addr >> 8));
	aw8695_i2c_write(aw8695, AW8695_REG_RAMADDRL, (unsigned char)(aw8695->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8695_haptic_ram:\n");
	for (i = 0; i < aw8695->ram.len; i++) {
		aw8695_i2c_read(aw8695, AW8695_REG_RAMDATA, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RAMINIT Disable */
	aw8695_i2c_write_bits(aw8695, AW8695_REG_SYSCTRL,
			      AW8695_BIT_SYSCTRL_RAMINIT_MASK, AW8695_BIT_SYSCTRL_RAMINIT_OFF);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8695_i2c_reg_show, aw8695_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8695_i2c_ram_show, aw8695_i2c_ram_store);

static struct attribute *aw8695_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8695_attribute_group = {
	.attrs = aw8695_attributes
};

static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;
	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");
	of_node_put(np);
	return factory;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw8695_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw8695 *aw8695;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw8695 = devm_kzalloc(&i2c->dev, sizeof(struct aw8695), GFP_KERNEL);
	if (aw8695 == NULL)
		return -ENOMEM;

	aw8695->dev = &i2c->dev;
	aw8695->i2c = i2c;

	i2c_set_clientdata(i2c, aw8695);

	/* aw8695 rst & int */
	if (np) {
		ret = aw8695_parse_dt(&i2c->dev, aw8695, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
			goto err_parse_dt;
		}
	} else {
		aw8695->reset_gpio = -1;
		aw8695->irq_gpio = -1;
	}

	if (gpio_is_valid(aw8695->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8695->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw8695_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw8695->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8695->irq_gpio,
					    GPIOF_DIR_IN, "aw8695_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n", __func__);
			goto err_irq_gpio_request;
		}
	}

	/* aw8695 chip id */
	ret = aw8695_read_chipid(aw8695);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8695_read_chipid failed ret=%d\n", __func__, ret);
		goto err_id;
	}

	/* aw8695 irq */
	if (gpio_is_valid(aw8695->irq_gpio) &&
	    !(aw8695->flags & AW8695_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw8695_interrupt_setup(aw8695);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw8695->irq_gpio),
						NULL, aw8695_irq, irq_flags,
						"aw8695", aw8695);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw8695->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw8695->flags |= AW8695_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw8695);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw8695_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
		goto err_sysfs;
	}

	g_aw8695 = aw8695;

	if (aw8695->factorymode_reduce)
		aw8695->factory_mode = false;
	else
		aw8695->factory_mode = mmi_factory_check();

	aw8695_vibrator_init(aw8695);

	aw8695_haptic_init(aw8695);

	aw8695_ram_init(aw8695);

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw8695->irq_gpio), aw8695);
err_irq:
err_id:
	if (gpio_is_valid(aw8695->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8695->irq_gpio);
err_irq_gpio_request:
	if (gpio_is_valid(aw8695->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8695->reset_gpio);
err_reset_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw8695);
	aw8695 = NULL;
	return ret;
}

static int aw8695_i2c_remove(struct i2c_client *i2c)
{
	struct aw8695 *aw8695 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	sysfs_remove_group(&i2c->dev.kobj, &aw8695_attribute_group);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw8695->irq_gpio), aw8695);

	if (gpio_is_valid(aw8695->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8695->irq_gpio);
	if (gpio_is_valid(aw8695->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8695->reset_gpio);

	PM_WAKEUP_UNREGISTER(aw8695->ws);

	devm_kfree(&i2c->dev, aw8695);
	aw8695 = NULL;

	return 0;
}

static const struct i2c_device_id aw8695_i2c_id[] = {
	{ AW8695_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8695_i2c_id);

static struct of_device_id aw8695_dt_match[] = {
	{ .compatible = "awinic,aw8695_haptic" },
	{ },
};

static struct i2c_driver aw8695_i2c_driver = {
	.driver = {
		.name = AW8695_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw8695_dt_match),
	},
	.probe = aw8695_i2c_probe,
	.remove = aw8695_i2c_remove,
	.id_table = aw8695_i2c_id,
};


static int __init aw8695_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8695 driver version %s\n", AW8695_VERSION);

	ret = i2c_add_driver(&aw8695_i2c_driver);
	if (ret) {
		pr_err("fail to add aw8695 device into i2c\n");
		return ret;
	}

	return 0;
}

module_init(aw8695_i2c_init);


static void __exit aw8695_i2c_exit(void)
{
	i2c_del_driver(&aw8695_i2c_driver);
}
module_exit(aw8695_i2c_exit);


MODULE_DESCRIPTION("AW8695 Haptic Driver");
MODULE_LICENSE("GPL v2");
