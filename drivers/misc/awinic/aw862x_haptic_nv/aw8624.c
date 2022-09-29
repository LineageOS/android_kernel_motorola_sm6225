/*
 * aw8624.c
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * Author: <chelvming@awinic.com>
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
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include "haptic_nv.h"
#include "haptic_nv_reg.h"
static int aw8624_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t glb_st = 0;
	int ret = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);
	if (reg_val & AW8624_BIT_SYSINT_OCDI)
		aw_err("chip over current int error");
	if (reg_val & AW8624_BIT_SYSINT_OTI)
		aw_err("chip over temperature int error");
	if (reg_val & AW8624_BIT_SYSINT_DONEI)
		aw_info("chip playback done");
	if (reg_val & AW8624_BIT_SYSINT_UVLI) {
		aw_err("chip uvlo int error");
		haptic_nv_i2c_reads(aw_haptic, AW8624_REG_GLB_STATE, &glb_st,
				    AW_I2C_BYTE_ONE);
		if (glb_st == 0) {
			haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSINTM,
						 AW8624_BIT_SYSINTM_UVLO_MASK,
						 AW8624_BIT_SYSINTM_UVLO_OFF);
		}
	}
	if (reg_val & AW8624_BIT_SYSINT_FF_AFI)
		aw_info("rtp mode fifo full");
	if (reg_val & AW8624_BIT_SYSINT_FF_AEI) {
		aw_info("rtp fifo almost empty");
		ret = AW_IRQ_ALMOST_EMPTY;
	}
	return ret;
}

static void aw8624_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%X", reg_val);

	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DBGCTRL,
				 AW8624_BIT_DBGCTRL_INT_MODE_MASK,
				 AW8624_BIT_DBGCTRL_INT_MODE_EDGE);

	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSINTM,
				 (AW8624_BIT_SYSINTM_OV_MASK &
				  AW8624_BIT_SYSINTM_UVLO_MASK &
				  AW8624_BIT_SYSINTM_FF_AE_MASK &
				  AW8624_BIT_SYSINTM_FF_AF_MASK &
				  AW8624_BIT_SYSINTM_OCD_MASK &
				  AW8624_BIT_SYSINTM_OT_MASK &
				  AW8624_BIT_SYSINTM_DONE_MASK),
				 (AW8624_BIT_SYSINTM_OV_OFF |
				  AW8624_BIT_SYSINTM_UVLO_EN |
				  AW8624_BIT_SYSINTM_FF_AE_OFF |
				  AW8624_BIT_SYSINTM_FF_AF_OFF |
				  AW8624_BIT_SYSINTM_OCD_EN |
				  AW8624_BIT_SYSINTM_OT_EN |
				  AW8624_BIT_SYSINTM_DONE_OFF));
}

static void aw8624_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
			  uint32_t len)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_RTP_DATA, data, len);
}

static uint8_t aw8624_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_GLB_STATE, &state,
			    AW_I2C_BYTE_ONE);
	return state;
}

static uint8_t aw8624_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8624_BIT_SYSST_FF_AFS;
	reg_val = reg_val >> 3;
	return reg_val;
}

static void aw8624_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSINTM,
					 AW8624_BIT_SYSINTM_FF_AE_MASK,
					 AW8624_BIT_SYSINTM_FF_AE_EN);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSINTM,
					 AW8624_BIT_SYSINTM_FF_AE_MASK,
					 AW8624_BIT_SYSINTM_FF_AE_OFF);
	}
}

static int aw8624_haptic_play_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	if (aw_haptic->play_mode == AW_CONT_MODE)
		reg_val = (uint8_t)(aw_haptic->info.sw_brake[0]);
	else
		reg_val = (uint8_t)(aw_haptic->info.sw_brake[1]);

	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_SW_BRAKE, &reg_val,
			     AW_I2C_BYTE_ONE);
	return 0;
}

static void aw8624_interrupt_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%2X", reg_val);
}

static void aw8624_haptic_active(struct aw_haptic *aw_haptic)
{
	aw8624_haptic_play_init(aw_haptic);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
		    AW8624_BIT_SYSCTRL_WORK_MODE_MASK,
		    AW8624_BIT_SYSCTRL_ACTIVE);
	aw8624_interrupt_clear(aw_haptic);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSINTM,
				 AW8624_BIT_SYSINTM_UVLO_MASK,
		   AW8624_BIT_SYSINTM_UVLO_EN);
}

static void aw8624_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSINTM,
					 AW8624_BIT_SYSINTM_UVLO_MASK,
					 AW8624_BIT_SYSINTM_UVLO_OFF);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_WORK_MODE_MASK,
					 AW8624_BIT_SYSCTRL_STANDBY);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8624_haptic_active(aw_haptic);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8624_haptic_active(aw_haptic);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_RTP);
		aw8624_haptic_active(aw_haptic);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8624_haptic_active(aw_haptic);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
					 AW8624_BIT_SYSCTRL_PLAY_MODE_CONT);
		aw8624_haptic_active(aw_haptic);
		break;
	default:
		aw_err("play mode %d error", play_mode);
		break;
	}
}

static int aw8624_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t val = 0;

	aw_info("enter, flag = %d", flag);
	if (!flag) {
		aw_haptic->current_t = ktime_get();
		aw_haptic->interval_us = ktime_to_us(ktime_sub(aw_haptic->current_t,
						     aw_haptic->pre_enter_t));
		if (aw_haptic->interval_us < 2000) {
			aw_info("interval_us=%d", aw_haptic->interval_us);
			usleep_range(AW_PLAY_DELAY_MIN, AW_PLAY_DELAY_MAX);
		}
	}
	if (flag == true) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_GO,
					 AW8624_BIT_GO_MASK,
					 AW8624_BIT_GO_ENABLE);
		aw_haptic->pre_enter_t = ktime_get();
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_GO,
					 AW8624_BIT_GO_MASK,
					 AW8624_BIT_GO_DISABLE);
	}
	val = aw8624_get_glb_state(aw_haptic);
	aw_info("reg:0x%02X=0x%02X", AW8624_REG_GLB_STATE, val);
	return 0;
}

static int aw8624_stop_delay(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int cnt = 100;

	while (cnt--) {
		haptic_nv_i2c_reads(aw_haptic, AW8624_REG_GLB_STATE, &reg_val,
				    AW_I2C_BYTE_ONE);
		if ((reg_val & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY) {
			aw_info("enter standby,reg glb_state=0x%02X", reg_val);
			return 0;
		}
		usleep_range(AW_STOP_DELAY_MIN, AW_STOP_DELAY_MAX);
		aw_info("wait for standby,reg glb_state=0x%02X", reg_val);
	}
	aw_err("do not enter standby automatically");
	return 0;
}

static void aw8624_play_stop(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw8624_play_go(aw_haptic, false);
	aw8624_stop_delay(aw_haptic);
	aw8624_play_mode(aw_haptic, AW_STANDBY_MODE);
}

static void aw8624_haptic_start(struct aw_haptic *aw_haptic)
{
	aw8624_haptic_active(aw_haptic);
	aw8624_play_go(aw_haptic, true);
}

static void aw8624_raminit(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_RAMINIT_MASK,
					 AW8624_BIT_SYSCTRL_RAMINIT_EN);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_SYSCTRL,
					 AW8624_BIT_SYSCTRL_RAMINIT_MASK,
					 AW8624_BIT_SYSCTRL_RAMINIT_OFF);
	}
}

static void aw8624_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw8624_play_stop(aw_haptic);
	/*step 1:EN_RAMINIT*/
	aw8624_raminit(aw_haptic, true);
	/*step 2 :launch power supply testing */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DETCTRL,
				 AW8624_BIT_DETCTRL_VBAT_GO_MASK,
				 AW8624_BIT_DETCTRL_VABT_GO_ENABLE);
	usleep_range(AW_VBAT_DELAY_MIN, AW_VBAT_DELAY_MAX);
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_VBATDET, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->vbat = AW8624_VBAT_FORMULA(reg_val);
	aw_info("get_vbat=%dmV, vbat_code=0x%02X", aw_haptic->vbat, reg_val);
	/*step 3: return val*/
	aw8624_raminit(aw_haptic, false);
}

static void aw8624_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_DATDBG, &gain,
			     AW_I2C_BYTE_ONE);
}

static void aw8624_set_trim_lra(struct aw_haptic *aw_haptic, uint8_t val)
{
	uint8_t reg_val = 0;

	reg_val = val & AW8624_BIT_TRIM_LRA;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TRIM_LRA, &reg_val,
			     AW_I2C_BYTE_ONE);
}

static int  aw8624_set_f0_preset(struct aw_haptic *aw_haptic, uint32_t f0_pre)
{
	uint32_t f0_reg = 0;
	uint8_t reg_array[2] = {0};

	if (!f0_pre || !aw_haptic->info.f0_coeff) {
		aw_err("f0_pre or f0_coeff is error, f0_pre=%d, f0_coeff=%d",
			f0_pre, aw_haptic->info.f0_coeff);
		return -ERANGE;
	}
	f0_reg = 1000000000 / (f0_pre * aw_haptic->info.f0_coeff);
	reg_array[0] = (uint8_t)((f0_reg >> 8) & 0xff);
	reg_array[1] = (uint8_t)((f0_reg >> 0) & 0xff);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_F_PRE_H, reg_array,
			     AW_I2C_BYTE_TWO);
	return 0;
}

static void aw8624_cont_config(struct aw_haptic *aw_haptic)
{
	uint8_t brake0_level = 0;
	uint8_t time_nzc = 0;
	uint8_t en_brake1 = 0;
	uint8_t brake1_level = 0;
	uint8_t en_brake2 = 0;
	uint8_t brake2_level = 0;
	uint8_t brake2_num = 0;
	uint8_t brake1_num = 0;
	uint8_t brake0_num = 0;
	uint8_t val[4] = {0};
	int ret = 0;

	aw_info("enter");
	aw8624_haptic_active(aw_haptic);
	aw8624_play_mode(aw_haptic, AW_CONT_MODE);
	/* preset f0 */
	if (aw_haptic->f0 <= 0)
		ret = aw8624_set_f0_preset(aw_haptic, aw_haptic->info.f0_pre);
	else
		ret = aw8624_set_f0_preset(aw_haptic, aw_haptic->f0);
	if (ret < 0)
		return;
	/* lpf */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DATCTRL,
				 (AW8624_BIT_DATCTRL_FC_MASK &
				  AW8624_BIT_DATCTRL_LPF_ENABLE_MASK),
				 (AW8624_BIT_DATCTRL_FC_1000HZ |
				  AW8624_BIT_DATCTRL_LPF_ENABLE));
	/* brake */
	en_brake1 = aw_haptic->info.cont_brake[0];
	en_brake2 = aw_haptic->info.cont_brake[1];
	brake0_level = aw_haptic->info.cont_brake[2];
	brake1_level = aw_haptic->info.cont_brake[3];
	brake2_level = aw_haptic->info.cont_brake[4];
	brake0_num = aw_haptic->info.cont_brake[5];
	brake1_num = aw_haptic->info.cont_brake[6];
	brake2_num = aw_haptic->info.cont_brake[7];
	val[0] = brake0_level << 0;
	val[1] = (en_brake1 << 7)|(brake1_level << 0);
	val[2] = (en_brake2 << 7)|(brake2_level << 0);
	val[3] = (brake2_num << 6) | (brake1_num << 3) | (brake0_num << 0);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_BRAKE0_CTRL, val,
			     AW_I2C_BYTE_FOUR);
	/* cont config */
	val[0] = AW8624_BIT_CONT_CTRL_ZC_DETEC_ENABLE |
		 AW8624_BIT_CONT_CTRL_WAIT_1PERIOD |
		 AW8624_BIT_CONT_CTRL_BY_GO_SIGNAL |
		 AW8624_BIT_CONT_CTRL_CLOSE_PLAYBACK |
		 AW8624_BIT_CONT_CTRL_F0_DETECT_DISABLE |
		 AW8624_BIT_CONT_CTRL_O2C_DISABLE;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_CONT_CTRL, val,
			     AW_I2C_BYTE_ONE);
	/* TD time */
	val[0] = (uint8_t)(aw_haptic->info.cont_td>>8);
	val[1] = (uint8_t)(aw_haptic->info.cont_td>>0);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TD_H, val,
			     AW_I2C_BYTE_TWO);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_BEMF_NUM,
				 AW8624_BIT_BEMF_NUM_BRK_MASK,
				 aw_haptic->info.cont_num_brk);
	time_nzc = AW8624_BIT_TIME_NZC_DEF_VAL;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TIME_NZC, &time_nzc,
			     AW_I2C_BYTE_ONE);
	/* f0 driver level */
	val[0] = aw_haptic->info.cont_drv1_lvl;
	val[1] = aw_haptic->info.cont_drv2_lvl;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_DRV_LVL, val,
			     AW_I2C_BYTE_TWO);
	aw8624_play_go(aw_haptic, true);
}

static ssize_t aw8624_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			      char *buf)
{
	len = haptic_nv_read_reg_array(aw_haptic, buf, len, AW8624_REG_ID,
				       AW8624_REG_RTP_DATA - 1);
	if (!len)
		return len;
	len = haptic_nv_read_reg_array(aw_haptic, buf, len,
				       AW8624_REG_RTP_DATA + 1,
				       AW8624_REG_RAMDATA - 1);
	if (!len)
		return len;
	len = haptic_nv_read_reg_array(aw_haptic, buf, len,
				       AW8624_REG_RAMDATA + 1,
				       AW8624_REG_NUM_F0_3);
	return len;
}

static void aw8624_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_PWMDBG,
					 AW8624_BIT_PWMDBG_PWM_MODE_MASK,
					 AW8624_BIT_PWMDBG_PWM_48K);
		break;
	case AW_PWM_24K:
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_PWMDBG,
					 AW8624_BIT_PWMDBG_PWM_MODE_MASK,
					 AW8624_BIT_PWMDBG_PWM_24K);
		break;
	case AW_PWM_12K:
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_PWMDBG,
					 AW8624_BIT_PWMDBG_PWM_MODE_MASK,
					 AW8624_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
}

static void aw8624_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				  uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_PWMPRC,
				 AW8624_BIT_PWMPRC_PRC_EN_MASK,
				 AW8624_BIT_PWMPRC_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW8624_BIT_PRLVL_PR_ENABLE |
			  (prlvl & (~AW8624_BIT_PRLVL_PRLVL_MASK));
		haptic_nv_i2c_writes(aw_haptic, AW8624_REG_PRLVL, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_nv_i2c_writes(aw_haptic, AW8624_REG_PRTIME, &prtime,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_PRLVL,
					 AW8624_BIT_PRLVL_PR_EN_MASK,
					 AW8624_BIT_PRLVL_PR_DISABLE);
	}
}

static void aw8624_misc_para_init(struct aw_haptic *aw_haptic)
{
	int ret = 0;
	uint8_t reg_val = 0;
	uint8_t reg_flag = 0;
	uint8_t reg_array[8] = {0};

	/* Get chipid_flag */
	ret = haptic_nv_i2c_reads(aw_haptic, AW8624_REG_EF_RDATAH, &reg_flag,
				  AW_I2C_BYTE_ONE);
	if ((ret >= 0) && ((reg_flag & 0x1) == 1))
		aw_haptic->chipid_flag = 1;
	else
		aw_err("to read register AW8624_REG_EF_RDATAH: %d", ret);
	/* Get seq and gain */
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_DATDBG, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val & 0xFF;
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_WAVSEQ1, reg_array,
			    AW_SEQUENCER_SIZE);
	aw_haptic->index = reg_array[0] & 0x7F;
	memcpy(aw_haptic->seq, reg_array, AW_SEQUENCER_SIZE);
	/* r_spare */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_R_SPARE,
				 AW8624_BIT_R_SPARE_MASK,
				 AW8624_BIT_R_SPARE_ENABLE);
	/* LRA trim source select register */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_ANACTRL,
				 AW8624_BIT_ANACTRL_LRA_SRC_MASK,
				 AW8624_BIT_ANACTRL_LRA_SRC_REG);
	/* brake */
	reg_val = (uint8_t)aw_haptic->info.sw_brake[0];
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_SW_BRAKE, &reg_val,
			     AW_I2C_BYTE_ONE);
	reg_val = 0x00;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_THRS_BRA_END, &reg_val,
			     AW_I2C_BYTE_ONE);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_WAVECTRL,
				 AW8624_BIT_WAVECTRL_NUM_OV_DRIVER_MASK,
				 AW8624_BIT_WAVECTRL_NUM_OV_DRIVER);
	/* zero cross */
	reg_array[0] = (uint8_t)(aw_haptic->info.cont_zc_thr >> 8);
	reg_array[1] = (uint8_t)(aw_haptic->info.cont_zc_thr >> 0);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_ZC_THRSH_H, reg_array,
			     AW_I2C_BYTE_TWO);
	/* cont_tset */
	reg_val = (uint8_t)aw_haptic->info.cont_tset;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TSET, &reg_val,
			     AW_I2C_BYTE_ONE);
	/* bemf */
	reg_array[0] = (uint8_t)(aw_haptic->info.bemf_config[0]);
	reg_array[1] = (uint8_t)(aw_haptic->info.bemf_config[1]);
	reg_array[2] = (uint8_t)(aw_haptic->info.bemf_config[2]);
	reg_array[3] = (uint8_t)(aw_haptic->info.bemf_config[3]);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_BEMF_VTHH_H, reg_array,
			     AW_I2C_BYTE_FOUR);
	aw8624_set_pwm(aw_haptic, AW_PWM_24K);
	aw8624_protect_config(aw_haptic, AW8624_REG_PRTIME_DEFAULT_VALUE,
			      AW8624_BIT_PRLVL_PRLVL_DEFAULT_VALUE);
}

static void aw8624_offset_cali(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int cont = 2000;

	aw_info("enter");

	aw8624_raminit(aw_haptic, true);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DETCTRL,
				 AW8624_BIT_DETCTRL_DIAG_GO_MASK,
				 AW8624_BIT_DETCTRL_DIAG_GO_ENABLE);
	while (cont) {
		haptic_nv_i2c_reads(aw_haptic, AW8624_REG_DETCTRL, &reg_val,
			    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	if (cont == 0)
		aw_err("calibration offset failed!");
	aw8624_raminit(aw_haptic, false);
}

static void aw8624_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_ADCTEST,
					 AW8624_BIT_DETCTRL_VBAT_MODE_MASK,
					 AW8624_BIT_DETCTRL_VBAT_HW_COMP);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_ADCTEST,
					 AW8624_BIT_DETCTRL_VBAT_MODE_MASK,
					 AW8624_BIT_DETCTRL_VBAT_SW_COMP);
	}
}

static void aw8624_calculate_cali_data(struct aw_haptic *aw_haptic)
{
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	uint8_t chipid_flag = aw_haptic->chipid_flag;

	f0_cali_step = 100000 * ((int)aw_haptic->f0 -
				 (int)aw_haptic->info.f0_pre) /
				((int)aw_haptic->f0 * AW8624_F0_CALI_ACCURACY);
	if (f0_cali_step >= 0) {
		if (f0_cali_step % 10 >= 5) {
			f0_cali_step = f0_cali_step / 10 + 1 +
					(chipid_flag == 1 ? 32 : 16);
		} else {
			f0_cali_step = f0_cali_step / 10 +
					(chipid_flag == 1 ? 32 : 16);
		}
	} else {
		if (f0_cali_step % 10 <= -5) {
			f0_cali_step = (chipid_flag == 1 ? 32 : 16) +
					(f0_cali_step / 10 - 1);
		} else {
			f0_cali_step = (chipid_flag == 1 ? 32 : 16) +
					f0_cali_step / 10;
		}
	}
	if (chipid_flag == 1) {
		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;
	} else {
		if (f0_cali_step < 16 ||
		    (f0_cali_step > 31 && f0_cali_step < 48)) {
			f0_cali_lra = (char)f0_cali_step + 16;
		} else {
			f0_cali_lra = (char)f0_cali_step - 16;
		}
	}
	if ((chipid_flag != 1) &&
	    (aw_haptic->f0 <= aw_haptic->info.f0_pre * 97 / 100))
		f0_cali_lra = 16;
	aw_haptic->f0_cali_data = (int)f0_cali_lra;
	aw_info("f0_cali_data=0x%02X", aw_haptic->f0_cali_data);
}

static int aw8624_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t f0_reg = 0;
	uint64_t f0_tmp = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_F_LRA_F0_H, val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (val[0] << 8) | val[1];
	if (!f0_reg || !aw_haptic->info.f0_coeff) {
		aw_haptic->f0 = 0;
		aw_info("get f0 failed with the value becoming 0!");
		return -ERANGE;
	}
	f0_tmp = AW8624_F0_FORMULA(f0_reg, aw_haptic->info.f0_coeff);
	aw_haptic->f0 = (uint32_t)f0_tmp;
	aw_info("f0=%d", aw_haptic->f0);
	return 0;
}

static void aw8624_read_beme(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_WAIT_VOL_MP, val,
			    AW_I2C_BYTE_TWO);
	aw_haptic->max_pos_beme = val[0];
	aw_haptic->max_neg_beme = val[1];
	aw_info("max_pos_beme=%d", aw_haptic->max_pos_beme);
	aw_info("max_neg_beme=%d", aw_haptic->max_neg_beme);
}

static int aw8624_get_f0(struct aw_haptic *aw_haptic)
{
	bool get_f0_flag = false;
	uint8_t val[3] = {0};
	uint8_t f0_pre_num = 0;
	uint8_t f0_wait_num = 0;
	uint8_t f0_repeat_num = 0;
	uint8_t f0_trace_num = 0;
	uint32_t t_f0_ms = 0;
	uint32_t t_f0_trace_ms = 0;
	int ret = 0;
	int cnt = 50;

	aw_info("enter");
	/* f0 calibrate work mode */
	aw8624_play_stop(aw_haptic);
	aw8624_play_mode(aw_haptic, AW_CONT_MODE);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_CONT_CTRL,
				 (AW8624_BIT_CONT_CTRL_EN_CLOSE_MASK &
				  AW8624_BIT_CONT_CTRL_F0_DETECT_MASK),
				 (AW8624_BIT_CONT_CTRL_OPEN_PLAYBACK |
				  AW8624_BIT_CONT_CTRL_F0_DETECT_ENABLE));
	/* LPF */
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DATCTRL,
				 (AW8624_BIT_DATCTRL_FC_MASK &
				  AW8624_BIT_DATCTRL_LPF_ENABLE_MASK),
				 (AW8624_BIT_DATCTRL_FC_1000HZ |
				  AW8624_BIT_DATCTRL_LPF_ENABLE));
	/* preset f0 */
	ret = aw8624_set_f0_preset(aw_haptic, aw_haptic->f0_pre);
	if (ret < 0)
		return -ERANGE;
	/* f0 driver level */
	val[0] = (uint8_t)(aw_haptic->info.cont_drv1_lvl);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_DRV_LVL, &val[0],
			     AW_I2C_BYTE_ONE);
	/* f0 trace parameter */
	if (!aw_haptic->f0_pre) {
		aw_info("fail to get t_f0_ms");
		return 0;
	}
	f0_pre_num = aw_haptic->info.f0_trace_parameter[0];
	f0_wait_num = aw_haptic->info.f0_trace_parameter[1];
	f0_repeat_num = aw_haptic->info.f0_trace_parameter[2];
	f0_trace_num = aw_haptic->info.f0_trace_parameter[3];
	val[0] = (f0_pre_num << 4)|(f0_wait_num << 0);
	val[1] = f0_repeat_num << 0;
	val[2] = f0_trace_num << 0;
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_NUM_F0_1, val,
			     AW_I2C_BYTE_THREE);
	/* clear aw8624 interrupt */
	ret = haptic_nv_i2c_reads(aw_haptic, AW8624_REG_SYSINT, &val[0],
				  AW_I2C_BYTE_ONE);
	/* play go and start f0 calibration */
	aw8624_play_go(aw_haptic, true);
	/* f0 trace time */
	t_f0_ms = 1000*10 / aw_haptic->f0_pre;
	t_f0_trace_ms = t_f0_ms * (f0_pre_num + f0_wait_num +
		       (f0_trace_num + f0_wait_num) * (f0_repeat_num - 1));
	aw_info("t_f0_trace_ms = %dms", t_f0_trace_ms);
	usleep_range(t_f0_trace_ms * 1000, t_f0_trace_ms * 1000 + 500);
	while (cnt) {
		haptic_nv_i2c_reads(aw_haptic, AW8624_REG_GLB_STATE, &val[0],
				    AW_I2C_BYTE_ONE);
		if (val[0] == AW_BIT_STATE_STANDBY) {
			get_f0_flag = true;
			aw_info("entered standby! glb_state=0x%02X", val[0]);
			break;
		}
		cnt--;
		aw_dbg("waitting for standby,glb_state=0x%02X", val[0]);
		usleep_range(AW_F0_DELAY_MIN, AW_F0_DELAY_MAX);
	}
	if (get_f0_flag) {
		ret = aw8624_read_f0(aw_haptic);
		if (ret < 0)
			aw_err("read lra f0 is failed");
		aw8624_read_beme(aw_haptic);
	} else {
		ret = -ERANGE;
		aw_err("enter standby mode failed, stop reading f0!");
	}

	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_CONT_CTRL,
				 (AW8624_BIT_CONT_CTRL_EN_CLOSE_MASK &
				  AW8624_BIT_CONT_CTRL_F0_DETECT_MASK),
				 (AW8624_BIT_CONT_CTRL_CLOSE_PLAYBACK |
				  AW8624_BIT_CONT_CTRL_F0_DETECT_DISABLE));

	return ret;
}

#ifdef AW8624_MUL_GET_F0
static int aw8624_multiple_get_f0(struct aw_haptic *aw_haptic)
{
	int i = 0;
	int ret = 0;
	int f0_max = aw_haptic->info.f0_pre + AW8624_MUL_GET_F0_RANGE;
	int f0_min = aw_haptic->info.f0_pre - AW8624_MUL_GET_F0_RANGE;

	aw_haptic->f0_pre = aw_haptic->info.f0_pre;
	for (i = 0; i < AW8624_MUL_GET_F0_NUM; i++) {
		aw_info("f0_pre=%d", aw_haptic->f0_pre);
		ret = aw8624_get_f0(aw_haptic);
		if (ret)
			return ret;
		if (aw_haptic->f0 >= f0_max || aw_haptic->f0 <= f0_min)
			break;
		aw_haptic->f0_pre = aw_haptic->f0;
		usleep_range(4000, 4500);
	}
	return 0;
}
#endif

static void aw8624_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_WAVSEQ1, aw_haptic->seq, len);
}

static void aw8624_set_wav_seq(struct aw_haptic *aw_haptic,
			       uint8_t wav, uint8_t seq)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_WAVSEQ1 + wav, &seq,
			     AW_I2C_BYTE_ONE);
}

static void aw8624_set_wav_loop(struct aw_haptic *aw_haptic,
				uint8_t wav, uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_nv_i2c_write_bits(aw_haptic,
					 AW8624_REG_WAVLOOP1 + (wav / 2),
					 AW8624_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_nv_i2c_write_bits(aw_haptic,
					 AW8624_REG_WAVLOOP1 + (wav / 2),
					 AW8624_BIT_WAVLOOP_SEQN_MASK, tmp);
	}

}

static void aw8624_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw8624_set_wav_seq(aw_haptic, 0x00, seq);
	aw8624_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw8624_set_wav_loop(aw_haptic, 0x00, AW8624_BIT_WAVLOOP_INIFINITELY);
}

static void aw8624_get_gain(struct aw_haptic *aw_haptic, uint8_t *gain)
{
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_DATDBG, gain, AW_I2C_BYTE_ONE);
}

static void aw8624_get_wav_loop(struct aw_haptic *aw_haptic, uint8_t *val)
{
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_WAVLOOP1, val,
			    AW_SEQUENCER_LOOP_SIZE);
}

static void aw8624_get_ram_data(struct aw_haptic *aw_haptic, uint8_t *ram_data,
				 int size)
{
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_RAMDATA, ram_data, size);
}

static void aw8624_read_cont_f0(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t f0_reg = 0;
	uint64_t f0_tmp = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_F_LRA_CONT_H, val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (val[0] << 8) | val[1];

	if (!f0_reg) {
		aw_haptic->cont_f0 = 0;
		aw_info("failed to reading cont f0 with 0");
		return;
	}

	f0_tmp = AW8624_F0_FORMULA(f0_reg, aw_haptic->info.f0_coeff);
	aw_haptic->cont_f0 = (uint32_t)f0_tmp;
	aw_info("cont_f0=%d", aw_haptic->cont_f0);
}

static void aw8624_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t anactrl = 0;
	uint8_t d2scfg = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_ANACTRL, &anactrl,
			    AW_I2C_BYTE_ONE);
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_D2SCFG, &d2scfg,
			    AW_I2C_BYTE_ONE);
	aw8624_play_stop(aw_haptic);
	aw8624_raminit(aw_haptic, true);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_ANACTRL,
				 AW8624_BIT_ANACTRL_EN_IO_PD1_MASK,
				 AW8624_BIT_ANACTRL_EN_IO_PD1_HIGH);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_D2SCFG,
				 AW8624_BIT_D2SCFG_CLK_ADC_MASK,
				 AW8624_BIT_D2SCFG_CLK_ASC_1P5MHZ);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DETCTRL,
				 (AW8624_BIT_DETCTRL_RL_OS_MASK &
				  AW8624_BIT_DETCTRL_DIAG_GO_MASK),
				 (AW8624_BIT_DETCTRL_RL_DETECT |
				  AW8624_BIT_DETCTRL_DIAG_GO_ENABLE));
	usleep_range(AW_RL_DELAY_MIN, AW_RL_DELAY_MAX);
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_RLDET, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->lra = AW8624_RL_FORMULA(reg_val);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_D2SCFG, &d2scfg,
			     AW_I2C_BYTE_ONE);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_ANACTRL, &anactrl,
			     AW_I2C_BYTE_ONE);
	aw8624_raminit(aw_haptic, false);
}

static uint8_t aw8624_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_PRLVL, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static uint8_t aw8624_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw8624_get_glb_state(aw_haptic);
	if (glb_state == AW_BIT_STATE_RTP_GO) {
		rtp_state = 1;  /*is going on */
		aw_info("rtp_routine_on");
	}
	return rtp_state;
}

static uint64_t aw8624_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_PWMDBG, &reg_val,
			    AW_I2C_BYTE_ONE);
	fre_val = (reg_val & 0x006f) >> 5;
	if (fre_val == AW8624_CLK_12K)
		theory_time = (aw_haptic->rtp_len / 12000) * 1000000;	/*12K*/
	else if (fre_val == AW8624_CLK_24K)
		theory_time = (aw_haptic->rtp_len / 24000) * 1000000;	/*24K*/
	else
		theory_time = (aw_haptic->rtp_len / 48000) * 1000000;	/*48K*/
	aw_info("microsecond:%llu  theory_time = %llu",
		aw_haptic->microsecond, theory_time);
	return theory_time;
}

static uint8_t aw8624_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_DBGSTAT, &state,
			    AW_I2C_BYTE_ONE);
	state &= AW8624_BIT_SYSINT_DONEI;
	return state;
}

static int aw8624_check_qualify(struct aw_haptic *aw_haptic)
{
	return 0;
}

static void aw8624_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("SYSINT=0x%02X", reg_val);
}

static void aw8624_set_base_addr(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	aw_info("enter");
	val[0] = (uint8_t)AW_SET_BASEADDR_H(base_addr);
	val[1] = (uint8_t)AW_SET_BASEADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_BASE_ADDRH, val,
			     AW_I2C_BYTE_TWO);
}

static void aw8624_set_fifo_addr(struct aw_haptic *aw_haptic)
{
	uint8_t val[4] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	aw_info("enter");
	val[0] = (uint8_t)AW8624_SET_AEADDR_H(base_addr);
	val[1] = (uint8_t)AW8624_SET_AEADDR_L(base_addr);
	val[2] = (uint8_t)AW8624_SET_AFADDR_H(base_addr);
	val[3] = (uint8_t)AW8624_SET_AFADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_FIFO_AEH, val,
			     AW_I2C_BYTE_FOUR);
}

static void aw8624_get_fifo_addr(struct aw_haptic *aw_haptic)
{
	uint8_t val[4] = {0};

	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_FIFO_AEH, val,
			    AW_I2C_BYTE_FOUR);
	aw_info("almost_empty_threshold = %d, almost_full_threshold = %d",
		(uint16_t)((val[0] << 8) | val[1]),
		(uint16_t)((val[2] << 8) | val[3]));
}

static void aw8624_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	aw_info("enter");
	val[0] = (uint8_t)AW_SET_RAMADDR_H(base_addr);
	val[1] = (uint8_t)AW_SET_RAMADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_RAMADDRH, val,
			     AW_I2C_BYTE_TWO);
}

static void aw8624_set_ram_data(struct aw_haptic *aw_haptic, uint8_t *data,
				int len)
{
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_RAMDATA, data, len);
}

static void aw8624_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint32_t *first_wave_addr)
{
	uint8_t val[3] = {0};

	aw_info("enter");
	aw8624_set_ram_addr(aw_haptic);
	haptic_nv_i2c_reads(aw_haptic, AW8624_REG_RAMDATA, val,
			    AW_I2C_BYTE_THREE);
	*first_wave_addr = (val[1] << 8 | val[2]);
}

static void aw8624_haptic_select_pin(struct aw_haptic *aw_haptic, uint8_t pin)
{
	if (pin == AW_TRIG1) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DBGCTRL,
					 AW8624_BIT_DBGCTRL_INTN_TRG_SEL_MASK,
					 AW8624_BIT_DBGCTRL_TRG_SEL_ENABLE);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_TRG_CFG2,
					 AW8624_BIT_TRGCFG2_TRG1_ENABLE_MASK,
					 AW8624_BIT_TRGCFG2_TRG1_ENABLE);
		aw_info("select TRIG1 pin");
	} else if (pin == AW_IRQ) {
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_DBGCTRL,
					 AW8624_BIT_DBGCTRL_INTN_TRG_SEL_MASK,
					 AW8624_BIT_DBGCTRL_INTN_SEL_ENABLE);
		haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_TRG_CFG2,
					 AW8624_BIT_TRGCFG2_TRG1_ENABLE_MASK,
					 AW8624_BIT_TRGCFG2_TRG1_DISABLE);
		aw_info("select INIT pin");
	} else
		aw_err("There is no such option");
}

static void aw8624_haptic_trig1_param_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw_haptic->trig[0].enable = aw_haptic->info.trig_cfg[0];
	aw_haptic->trig[0].trig_edge = aw_haptic->info.trig_cfg[1];
	aw_haptic->trig[0].trig_polar = aw_haptic->info.trig_cfg[2];
	aw_haptic->trig[0].pos_sequence = aw_haptic->info.trig_cfg[3];
	aw_haptic->trig[0].neg_sequence = aw_haptic->info.trig_cfg[4];
}

static void aw8624_haptic_trig1_param_config(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	if (!(aw_haptic->trig[0].enable) || aw_haptic->is_used_irq_pin) {
		aw8624_haptic_trig1_param_init(aw_haptic);
		aw8624_haptic_select_pin(aw_haptic, AW_IRQ);
		return;
	}
	aw8624_haptic_select_pin(aw_haptic, AW_TRIG1);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_TRG_CFG1,
				 AW8624_BIT_TRGCFG1_TRG1_EDGE_MASK,
				 aw_haptic->trig[0].trig_edge);
	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_TRG_CFG1,
				 AW8624_BIT_TRGCFG1_TRG1_POLAR_MASK,
				 aw_haptic->trig[0].trig_polar << 1);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TRG1_SEQP,
			     &aw_haptic->trig[0].pos_sequence, AW_I2C_BYTE_ONE);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TRG1_SEQN,
			     &aw_haptic->trig[0].neg_sequence, AW_I2C_BYTE_ONE);
}

static void aw8624_haptic_set_trig1(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw8624_haptic_trig1_param_init(aw_haptic);
	aw8624_haptic_trig1_param_config(aw_haptic);
}

static void aw8624_trig_init(struct aw_haptic *aw_haptic)
{
	if (aw_haptic->is_used_irq_pin) {
		aw8624_haptic_select_pin(aw_haptic, AW_IRQ);
		return;
	}
	aw_info("enter");
	aw8624_haptic_set_trig1(aw_haptic);
}


static ssize_t cont_td_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf+len, PAGE_SIZE-len,
			"cont_delay_time = 0x%04X\n",
			aw_haptic->info.cont_td);
	return len;
}

static ssize_t cont_td_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	int err, val;
	uint8_t reg_array[2] = {0};

	err = kstrtoint(buf, 16, &val);
	if (err != 0) {
		aw_err("format not match!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->info.cont_td = val;
	reg_array[0] = (uint8_t)(val >> 8);
	reg_array[1] = (uint8_t)(val >> 0);
	haptic_nv_i2c_writes(aw_haptic, AW8624_REG_TD_H, reg_array,
			     AW_I2C_BYTE_TWO);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t cont_drv_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf+len, PAGE_SIZE-len, "cont drv level = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"cont drv level overdrive= 0x%02X\n",
			aw_haptic->info.cont_drv2_lvl);
	return len;
}

static ssize_t cont_drv_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	uint8_t reg_array[2] = {0};
	uint32_t databuf[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	mutex_lock(&aw_haptic->lock);
	if (sscanf(buf, "%X %X", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		reg_array[0] = aw_haptic->info.cont_drv1_lvl;
		reg_array[1] = aw_haptic->info.cont_drv2_lvl;
		haptic_nv_i2c_writes(aw_haptic, AW8624_REG_DRV_LVL, reg_array,
				     AW_I2C_BYTE_TWO);
	}
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t cont_num_brk_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf+len, PAGE_SIZE-len,
			"cont_brk_num = 0x%02x\n",
			aw_haptic->info.cont_num_brk);
	return len;
}

static ssize_t cont_num_brk_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err, val;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	err = kstrtoint(buf, 16, &val);
	if (err != 0) {
		aw_err("format not match!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->info.cont_num_brk = val;
	if (aw_haptic->info.cont_num_brk > 7)
		aw_haptic->info.cont_num_brk = 7;

	haptic_nv_i2c_write_bits(aw_haptic, AW8624_REG_BEMF_NUM,
				 AW8624_BIT_BEMF_NUM_BRK_MASK,
				 aw_haptic->info.cont_num_brk);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t cont_zc_thr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
			"cont_zero_cross_thr = 0x%04X\n",
			aw_haptic->info.cont_zc_thr);
	return len;
}

static ssize_t cont_zc_thr_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	uint8_t reg_array[2] = {0};
	int err, val;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	err = kstrtoint(buf, 0, &val);
	if (err != 0) {
		aw_err("format not match!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->info.cont_num_brk = val;
	aw_info("val=%d", val);
	if (val > 0) {
		reg_array[0] = (uint8_t)(val >> 8);
		reg_array[1] = (uint8_t)(val >> 0);
		haptic_nv_i2c_writes(aw_haptic, AW8624_REG_ZC_THRSH_H,
				     reg_array, AW_I2C_BYTE_TWO);
	}
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t trig_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"trig: trig_enable=%d, trig_edge=%d\n",
			aw_haptic->trig[0].enable,
			aw_haptic->trig[0].trig_edge);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"trig_polar=%d, pos_sequence=%d, neg_sequence=%d\n",
			aw_haptic->trig[0].trig_polar,
			aw_haptic->trig[0].pos_sequence,
			aw_haptic->trig[0].neg_sequence);
	return len;

}

static ssize_t trig_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	uint32_t databuf[5] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	if (sscanf(buf, "%d %d %d %d %d",
		&databuf[0], &databuf[1],
		&databuf[2], &databuf[3], &databuf[4]) == 5) {
		if (databuf[0] > 1)
			databuf[0] = 1;
		if (databuf[0] < 0)
			databuf[0] = 0;
		if (databuf[1] > 1)
			databuf[0] = 1;
		if (databuf[1] < 0)
			databuf[0] = 0;
		if (databuf[2] > 1)
			databuf[0] = 1;
		if (databuf[2] < 0)
			databuf[0] = 0;
		if (databuf[3] > aw_haptic->ram.ram_num ||
		    databuf[4] > aw_haptic->ram.ram_num) {
			aw_err("input seq value out of range!");
			return count;
		}
		aw_haptic->trig[0].enable = databuf[0];
		aw_haptic->trig[0].trig_edge = databuf[1];
		aw_haptic->trig[0].trig_polar = databuf[2];
		aw_haptic->trig[0].pos_sequence = databuf[3];
		aw_haptic->trig[0].neg_sequence = databuf[4];
		mutex_lock(&aw_haptic->lock);
		aw8624_haptic_trig1_param_config(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	} else
		aw_err("please input five parameters");
	return count;
}

static DEVICE_ATTR(cont_td, 0664, cont_td_show, cont_td_store);
static DEVICE_ATTR(cont_drv, 0664, cont_drv_show, cont_drv_store);
static DEVICE_ATTR(cont_num_brk, 0664, cont_num_brk_show, cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, 0664, cont_zc_thr_show, cont_zc_thr_store);
static DEVICE_ATTR(trig, 0664, trig_show, trig_store);
static struct attribute *aw8624_vibrator_attributes[] = {
	&dev_attr_cont_td.attr,
	&dev_attr_cont_drv.attr,
	&dev_attr_cont_num_brk.attr,
	&dev_attr_cont_zc_thr.attr,
	&dev_attr_trig.attr,
	NULL
};

static struct attribute_group aw8624_vibrator_attribute_group = {
	.attrs = aw8624_vibrator_attributes
};

static void aw8624_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw8624_vibrator_attribute_group);
	if (ret < 0)
		aw_err("error create aw862xx sysfs attr files");
}

struct aw_haptic_func aw8624_func_list = {
	.ram_init = aw8624_raminit,
	.trig_init = aw8624_trig_init,
	.play_mode = aw8624_play_mode,
	.play_stop = aw8624_play_stop,
	.irq_clear = aw8624_irq_clear,
	.creat_node = aw8624_creat_node,
	.cont_config = aw8624_cont_config,
	.offset_cali = aw8624_offset_cali,
	.haptic_start = aw8624_haptic_start,
	.read_cont_f0 = aw8624_read_cont_f0,
	.check_qualify = aw8624_check_qualify,
	.judge_rtp_going = aw8624_judge_rtp_going,
	.protect_config = aw8624_protect_config,
	.misc_para_init = aw8624_misc_para_init,
	.interrupt_setup = aw8624_interrupt_setup,
	.rtp_get_fifo_afs = aw8624_rtp_get_fifo_afs,
	.vbat_mode_config = aw8624_vbat_mode_config,
	.calculate_cali_data = aw8624_calculate_cali_data,
	.set_gain = aw8624_set_gain,
	.get_gain = aw8624_get_gain,
	.set_wav_seq = aw8624_set_wav_seq,
	.get_wav_seq = aw8624_get_wav_seq,
	.set_wav_loop = aw8624_set_wav_loop,
	.get_wav_loop = aw8624_get_wav_loop,
	.set_ram_data = aw8624_set_ram_data,
	.get_ram_data = aw8624_get_ram_data,
	.set_fifo_addr = aw8624_set_fifo_addr,
	.get_fifo_addr = aw8624_get_fifo_addr,
	.set_rtp_aei = aw8624_set_rtp_aei,
	.set_rtp_data = aw8624_set_rtp_data,
	.set_ram_addr = aw8624_set_ram_addr,
	.set_trim_lra = aw8624_set_trim_lra,
	.set_base_addr = aw8624_set_base_addr,
	.set_repeat_seq = aw8624_set_repeat_seq,
#ifdef AW8624_MUL_GET_F0
	.get_f0 = aw8624_multiple_get_f0,
#else
	.get_f0 = aw8624_get_f0,
#endif
	.get_reg = aw8624_get_reg,
	.get_vbat = aw8624_get_vbat,
	.get_prctmode = aw8624_get_prctmode,
	.get_irq_state = aw8624_get_irq_state,
	.get_glb_state = aw8624_get_glb_state,
	.get_osc_status = aw8624_get_osc_status,
	.get_theory_time = aw8624_get_theory_time,
	.get_lra_resistance = aw8624_get_lra_resistance,
	.get_first_wave_addr = aw8624_get_first_wave_addr,
};
