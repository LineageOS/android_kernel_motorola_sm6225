/*
 * aw862xx.c
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

static void aw862xx_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
	/* edge int mode */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL7,
		   (AW862XX_BIT_SYSCTRL7_INT_MODE_MASK &
		    AW862XX_BIT_SYSCTRL7_INT_EDGE_MODE_MASK),
		   (AW862XX_BIT_SYSCTRL7_INT_MODE_EDGE |
		    AW862XX_BIT_SYSCTRL7_INT_EDGE_MODE_POS));
	/* int enable */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSINTM,
		   (AW862XX_BIT_SYSINTM_UVLM_MASK &
		    AW862XX_BIT_SYSINTM_FF_AEM_MASK &
		    AW862XX_BIT_SYSINTM_FF_AFM_MASK &
		    AW862XX_BIT_SYSINTM_OCDM_MASK &
		    AW862XX_BIT_SYSINTM_OTM_MASK &
		    AW862XX_BIT_SYSINTM_DONEM_MASK),
		    (AW862XX_BIT_SYSINTM_UVLM_ON |
		     AW862XX_BIT_SYSINTM_FF_AEM_OFF |
		     AW862XX_BIT_SYSINTM_FF_AFM_OFF |
		     AW862XX_BIT_SYSINTM_OCDM_ON |
		     AW862XX_BIT_SYSINTM_OTM_ON |
		     AW862XX_BIT_SYSINTM_DONEM_OFF));
}

static void aw862xx_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
			  uint32_t len)
{
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_RTPDATA, data, len);
}

static uint8_t aw862xx_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_GLBRD5, &state,
			    AW_I2C_BYTE_ONE);
	return state;
}

static int aw862xx_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);
	if (reg_val & AW862XX_BIT_SYSINT_UVLI)
		aw_err("chip uvlo int error");
	if (reg_val & AW862XX_BIT_SYSINT_OCDI)
		aw_err("chip over current int error");
	if (reg_val & AW862XX_BIT_SYSINT_OTI)
		aw_err("chip over temperature int error");
	if (reg_val & AW862XX_BIT_SYSINT_DONEI)
		aw_info("chip playback done");
	if (reg_val & AW862XX_BIT_SYSINT_FF_AFI)
		aw_info("rtp mode fifo almost full!");
	if (reg_val & AW862XX_BIT_SYSINT_FF_AEI) {
		aw_info("rtp fifo almost empty");
		ret = AW_IRQ_ALMOST_EMPTY;
	}
	return ret;
}

static uint8_t aw862xx_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW862XX_BIT_SYSST_FF_AFS;
	reg_val = reg_val >> 3;
	return reg_val;
}

static void aw862xx_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSINTM,
					 AW862XX_BIT_SYSINTM_FF_AEM_MASK,
					 AW862XX_BIT_SYSINTM_FF_AEM_ON);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSINTM,
					 AW862XX_BIT_SYSINTM_FF_AEM_MASK,
					 AW862XX_BIT_SYSINTM_FF_AEM_OFF);
	}
}

#ifdef AW862XX_COMMON
static void aw862xx_sram_size(struct aw_haptic *aw_haptic, uint8_t size_flag)
{
	switch (size_flag) {
	case AW862XX_HAPTIC_SRAM_1K:
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_2K_DIS);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
		break;
	case AW862XX_HAPTIC_SRAM_2K:
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_1K_DIS);
		break;
	case AW862XX_HAPTIC_SRAM_3K:
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_1K_MASK,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_1K_EN);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_2K_MASK,
					 AW862XX_BIT_RTPCFG1_SRAM_SIZE_2K_EN);
		break;
	default:
		aw_err("size_flag is error");
		break;
	}
}

static void aw862xx_auto_brk_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW862XX_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW862XX_BIT_PLAYCFG3_BRK_DISABLE);
	}
}
#endif

#ifdef AW86214_DRIVER
static void aw86214_brk_en(struct aw_haptic *aw_haptic, bool is_enable)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	if (is_enable == false)
		reg_val = 0x00;
	else
		reg_val = (uint8_t)aw_haptic->info.cont_brk_time;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG10, &reg_val,
			     AW_I2C_BYTE_ONE);
}

static int aw86214_play_mode_config(struct aw_haptic *aw_haptic,
				    uint8_t play_mode)
{
	int ret = 0;

	aw_info("enter");
	switch (play_mode) {
	case AW_RAM_MODE:
	case AW_RAM_LOOP_MODE:
		aw86214_brk_en(aw_haptic, false);
		break;
	case AW_CONT_MODE:
		aw86214_brk_en(aw_haptic, true);
		break;
	case AW_RTP_MODE:
	case AW_TRIG_MODE:
		ret = -ERANGE;
		break;
	default:
		ret = -ERANGE;
		break;
	}
	return ret;
}

static void aw86214_config(struct aw_haptic *aw_haptic)
{
	uint8_t val = 0;

	aw_info("enter");
	aw862xx_sram_size(aw_haptic, AW862XX_HAPTIC_SRAM_1K);
	aw862xx_auto_brk_config(aw_haptic, true);
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG10, &val,
			     AW_I2C_BYTE_ONE);
}
#endif

#ifdef AW8622X_DRIVER
static void aw8622x_config(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	aw862xx_sram_size(aw_haptic, AW862XX_HAPTIC_SRAM_3K);
	aw862xx_auto_brk_config(aw_haptic, aw_haptic->info.is_enabled_auto_brk);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_TRGCFG8,
				 AW862XX_BIT_TRGCFG8_TRG_TRIG1_MODE_MASK,
				 AW862XX_BIT_TRGCFG8_TRIG1);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_ANACFG8,
				 AW862XX_BIT_ANACFG8_TRTF_CTRL_HDRV_MASK,
				 AW862XX_BIT_ANACFG8_TRTF_CTRL_HDRV);
	if (aw_haptic->info.cont_brk_time) {
		reg_val = (uint8_t)aw_haptic->info.cont_brk_time;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG10,
				     &reg_val, AW_I2C_BYTE_ONE);
	} else {
		aw_err("dts_info.cont_brk_time=0");
	}
	if (aw_haptic->info.cont_brk_gain) {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG5,
					 AW862XX_BIT_CONTCFG5_BRK_GAIN_MASK,
					 aw_haptic->info.cont_brk_gain);
	} else {
		aw_err("dts_info.cont_brk_gain=0");
	}

}
#endif

static void aw862xx_play_stop(struct aw_haptic *aw_haptic)
{
	bool force_flag = true;
	uint8_t val = 0;
	int cnt = 40;

	aw_info("enter");
	aw_haptic->play_mode = AW_STANDBY_MODE;
	val = AW862XX_BIT_PLAYCFG4_STOP_ON;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_PLAYCFG4, &val,
			     AW_I2C_BYTE_ONE);
	while (cnt) {
		haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_GLBRD5, &val,
				    AW_I2C_BYTE_ONE);
		if ((val & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY) {
			force_flag = false;
			aw_info("entered standby! glb_state=0x%02X", val);
			break;
		}
		cnt--;
		aw_info("wait for standby, glb_state=0x%02X", val);
		usleep_range(AW_STOP_DELAY_MIN, AW_STOP_DELAY_MAX);
	}
	if (force_flag) {
		aw_err("force to enter standby mode!");
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_STANDBY_MASK,
					 AW862XX_BIT_SYSCTRL2_STANDBY_ON);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_STANDBY_MASK,
					 AW862XX_BIT_SYSCTRL2_STANDBY_OFF);
	}
}

static void aw862xx_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW862XX_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW_PWM_24K:
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW862XX_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW_PWM_12K:
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW862XX_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
}

static void aw862xx_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
#ifdef AW86214_DRIVER
	if (aw_haptic->name == AW86214) {
		if (aw86214_play_mode_config(aw_haptic, play_mode))
			return;
	}
#endif

	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		aw862xx_play_stop(aw_haptic);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		aw862xx_set_pwm(aw_haptic, AW_PWM_12K);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		aw862xx_set_pwm(aw_haptic, AW_PWM_12K);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_CONT);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		aw862xx_set_pwm(aw_haptic, AW_PWM_24K);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_RTP);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		aw862xx_set_pwm(aw_haptic, AW_PWM_12K);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW862XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	default:
		aw_err("play mode %d error", play_mode);
		break;
	}
}

static void aw862xx_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSINT, &val,
			    AW_I2C_BYTE_ONE);
	aw_info("SYSINT=0x%02X", val);
}

static void aw862xx_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t val = 0;

	aw_info("enter");
	if (flag == true) {
		val = AW862XX_BIT_PLAYCFG4_GO_ON;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_PLAYCFG4, &val,
				     AW_I2C_BYTE_ONE);
		usleep_range(AW_PLAY_DELAY_MIN, AW_PLAY_DELAY_MAX);
	} else {
		val = AW862XX_BIT_PLAYCFG4_STOP_ON;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_PLAYCFG4, &val,
				     AW_I2C_BYTE_ONE);
	}
	val = aw862xx_get_glb_state(aw_haptic);
	aw_info("reg:0x%02X=0x%02X", AW862XX_REG_GLBRD5, val);
}

static void aw862xx_haptic_start(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	aw862xx_play_go(aw_haptic, true);
}

static void aw862xx_raminit(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL1,
					 AW862XX_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW862XX_BIT_SYSCTRL1_RAMINIT_ON);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL1,
					 AW862XX_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW862XX_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static void aw862xx_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t vbat_code = 0;

	aw_info("enter");
	aw862xx_play_stop(aw_haptic);
	aw862xx_raminit(aw_haptic, true);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_DETCFG2,
				 AW862XX_BIT_DETCFG2_VBAT_GO_MASK,
				 AW862XX_BIT_DETCFG2_VABT_GO_ON);
	usleep_range(AW_VBAT_DELAY_MIN, AW_VBAT_DELAY_MAX);
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_DET_VBAT, &reg_val,
			    AW_I2C_BYTE_ONE);
	vbat_code = (vbat_code | reg_val) << 2;
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_DET_LO, &reg_val,
			    AW_I2C_BYTE_ONE);
	vbat_code = vbat_code | ((reg_val & AW862XX_BIT_DET_LO_VBAT) >> 4);
	aw_haptic->vbat = AW862XX_VBAT_FORMULA(vbat_code);
	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_info("vbat max limit = %dmV", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_info("vbat min limit = %dmV", aw_haptic->vbat);
	}
	aw_info("vbat=%dmV, vbat_code=0x%02X", aw_haptic->vbat, vbat_code);
	aw862xx_raminit(aw_haptic, false);
}

static void aw862xx_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_PLAYCFG2, &gain,
			     AW_I2C_BYTE_ONE);
}

static void aw862xx_set_trim_lra(struct aw_haptic *aw_haptic, uint8_t val)
{
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_TRIMCFG3,
				 AW862XX_BIT_TRIMCFG3_TRIM_LRA_MASK, val);
}

static void aw862xx_cont_config(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_info("enter");
	/* work mode */
	aw862xx_play_mode(aw_haptic, AW_CONT_MODE);
	/* cont config */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG6,
				 (AW862XX_BIT_CONTCFG6_TRACK_EN_MASK &
				 AW862XX_BIT_CONTCFG6_DRV1_LVL_MASK),
				 ((aw_haptic->info.is_enabled_track_en << 7) |
				 (uint8_t)aw_haptic->info.cont_drv1_lvl));
	reg_val = (uint8_t)aw_haptic->info.cont_drv2_lvl;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG7, &reg_val,
			     AW_I2C_BYTE_ONE);
	/* DRV2_TIME */
	reg_val = 0xFF;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG9, &reg_val,
			     AW_I2C_BYTE_ONE);
	/* cont play go */
	aw862xx_play_go(aw_haptic, true);
}

static ssize_t aw862xx_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			       char *buf)
{
	len = haptic_nv_read_reg_array(aw_haptic, buf, len, AW862XX_REG_ID,
				       AW862XX_REG_RTPDATA - 1);
	if (!len)
		return len;
	len = haptic_nv_read_reg_array(aw_haptic, buf, len,
				       AW862XX_REG_RTPDATA + 1,
				       AW862XX_REG_RAMDATA - 1);
	if (!len)
		return len;
	len = haptic_nv_read_reg_array(aw_haptic, buf, len,
				       AW862XX_REG_RAMDATA + 1,
				       AW862XX_REG_ANACFG8);
	return len;
}

static void aw862xx_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				   uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PWMCFG1,
				 AW862XX_BIT_PWMCFG1_PRC_EN_MASK,
				 AW862XX_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW862XX_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW862XX_BIT_PWMCFG3_PRLVL_MASK));
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_PWMCFG3, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_PWMCFG4, &prtime,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PWMCFG3,
					 AW862XX_BIT_PWMCFG3_PR_EN_MASK,
					 AW862XX_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw862xx_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t val = 0;
	uint8_t array[8] = {0};

	aw_info("enter");
	/* Get seq and gain */
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_WAVCFG1, &val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->index = val & AW862XX_BIT_WAVCFG_SEQ;
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_PLAYCFG2, &val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->gain = val;
	aw_info("gain=0x%02X, index=0x%02X", aw_haptic->gain, aw_haptic->index);
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_WAVCFG1, array,
			    AW_SEQUENCER_SIZE);
	memcpy(aw_haptic->seq, array, AW_SEQUENCER_SIZE);
	/* GAIN_BYPASS config */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL7,
				 AW862XX_BIT_SYSCTRL7_GAIN_BYPASS_MASK,
				 AW862XX_BIT_SYSCTRL7_GAIN_CHANGEABLE);
	/* SIN_H */
	array[0] = (uint8_t)aw_haptic->info.sine_array[0];
	/* SIN_L */
	array[1] = (uint8_t)aw_haptic->info.sine_array[1];
	/* COS_H */
	array[2] = (uint8_t)aw_haptic->info.sine_array[2];
	/* COS_L */
	array[3] = (uint8_t)aw_haptic->info.sine_array[3];
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_SYSCTRL3, array,
			     AW_I2C_BYTE_FOUR);
	if (!aw_haptic->info.d2s_gain) {
		aw_err("dts_info.d2s_gain = 0!");
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL7,
					 AW862XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
					 aw_haptic->info.d2s_gain);
	}
	if (aw_haptic->info.cont_tset && aw_haptic->info.cont_bemf_set) {
		array[0] = (uint8_t)aw_haptic->info.cont_tset;
		array[1] = (uint8_t)aw_haptic->info.cont_bemf_set;
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG13,
					 (AW862XX_BIT_CONTCFG13_TSET_MASK &
					 AW862XX_BIT_CONTCFG13_BEME_SET_MASK),
					 ((array[0] << 4) | array[1]));
	} else {
		aw_err("dts_info.cont_tset = 0x%02X",
			aw_haptic->info.cont_tset);
		aw_err("dts_info.cont_bemf_set = 0x%02X",
			aw_haptic->info.cont_bemf_set);
	}
	aw_haptic->info.cont_drv2_lvl = AW862XX_DRV2_LVL_FORMULA(
			      aw_haptic->info.f0_pre, aw_haptic->info.lra_vrms);
	aw_info("lra_vrms=%d, cont_drv2_lvl=0x%02X", aw_haptic->info.lra_vrms,
		aw_haptic->info.cont_drv2_lvl);
	if (aw_haptic->info.cont_drv2_lvl > AW862XX_DRV2_LVL_MAX) {
		aw_err("cont_drv2_lvl[0x%02X] is error, restore max vale[0x%02X]",
		       aw_haptic->info.cont_drv2_lvl, AW862XX_DRV2_LVL_MAX);
		aw_haptic->info.cont_drv2_lvl = AW862XX_DRV2_LVL_MAX;
	}

	switch (aw_haptic->name) {
#ifdef AW86214_DRIVER
	case AW86214:
		aw86214_config(aw_haptic);
		break;
#endif

#ifdef AW8622X_DRIVER
	case AW86223:
	case AW86224:
	case AW86225:
		aw8622x_config(aw_haptic);
		break;
#endif
	default:
		break;
	}
	aw862xx_set_pwm(aw_haptic, AW_PWM_12K);
	aw862xx_protect_config(aw_haptic, AW862XX_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW862XX_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
}

static void aw862xx_offset_cali(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int cont = 2000;

	aw_info("enter");
	aw862xx_raminit(aw_haptic, true);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_DETCFG2,
				 AW862XX_BIT_DETCFG2_DIAG_GO_MASK,
				 AW862XX_BIT_DETCFG2_DIAG_GO_ON);
	while (cont) {
		haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_DETCFG2, &reg_val,
				    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	if (cont == 0)
		aw_err("calibration offset failed!");
	aw862xx_raminit(aw_haptic, false);
}

static void aw862xx_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	uint8_t val = 0;

	aw_info("enter");
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		val = AW862XX_BIT_GLBCFG2_START_DLY_250US;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_GLBCFG2, &val,
				     AW_I2C_BYTE_ONE);
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL1,
					 AW862XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
					 AW862XX_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL1,
					 AW862XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
					 AW862XX_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
}

static void aw862xx_calculate_cali_data(struct aw_haptic *aw_haptic)
{
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

	f0_cali_step = 100000 * ((int)aw_haptic->f0 -
				 (int)aw_haptic->info.f0_pre) /
				((int)aw_haptic->f0 * AW862XX_F0_CALI_ACCURACY);
	aw_info("f0_cali_step=%d", f0_cali_step);
	if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
		if (f0_cali_step % 10 >= 5)
			f0_cali_step = 32 + (f0_cali_step / 10 + 1);
		else
			f0_cali_step = 32 + f0_cali_step / 10;
	} else {	/* f0_cali_step < 0 */
		if (f0_cali_step % 10 <= -5)
			f0_cali_step = 32 + (f0_cali_step / 10 - 1);
		else
			f0_cali_step = 32 + f0_cali_step / 10;
	}
	if (f0_cali_step > 31)
		f0_cali_lra = (char)f0_cali_step - 32;
	else
		f0_cali_lra = (char)f0_cali_step + 32;
	/* update cali step */
	aw_haptic->f0_cali_data = (int)f0_cali_lra;
	aw_info("f0_cali_data=0x%02X", aw_haptic->f0_cali_data);
}

static void aw862xx_read_cont_f0(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t f0_reg = 0;
	uint64_t f0_tmp = 0;

	aw_info("enter");
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_CONTRD16, val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (f0_reg | val[0]) << 8;
	f0_reg |= (val[1] << 0);
	if (!f0_reg) {
		aw_err("didn't get cont f0 because f0_reg value is 0!");
		aw_haptic->cont_f0 = 0;
		return;
	}
	f0_tmp = AW862XX_F0_FORMULA(f0_reg);
	aw_haptic->cont_f0 = (uint32_t)f0_tmp;
	aw_info("cont_f0=%d", aw_haptic->cont_f0);
}

static int aw862xx_read_lra_f0(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t f0_reg = 0;
	uint64_t f0_tmp = 0;

	aw_info("enter");
	/* F_LRA_F0 */
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_CONTRD14, val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (f0_reg | val[0]) << 8;
	f0_reg |= (val[1] << 0);
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("didn't get lra f0 because f0_reg value is 0!");
		return -ERANGE;
	}
	f0_tmp = AW862XX_F0_FORMULA(f0_reg);
	aw_haptic->f0 = (uint32_t)f0_tmp;
	aw_info("lra_f0=%d", aw_haptic->f0);
	return 0;
}

static int aw862xx_get_f0(struct aw_haptic *aw_haptic)
{
	bool get_f0_flag = false;
	uint8_t brk_en_temp = 0;
	uint8_t val[3] = {0};
	int cnt = 200;
	int ret = 0;

	aw_info("enter");
	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw862xx_play_stop(aw_haptic);
	/* f0 calibrate work mode */
	aw862xx_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG1,
				 AW862XX_BIT_CONTCFG1_EN_F0_DET_MASK,
				 AW862XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG6,
				 AW862XX_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto brake */
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_PLAYCFG3, &val[0],
		    AW_I2C_BYTE_ONE);
	brk_en_temp = AW862XX_BIT_PLAYCFG3_BRK & val[0];
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
				 AW862XX_BIT_PLAYCFG3_BRK_EN_MASK,
				 AW862XX_BIT_PLAYCFG3_BRK_ENABLE);
	/* f0 driver level */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG6,
				 AW862XX_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	val[0] = (uint8_t)aw_haptic->info.cont_drv2_lvl;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG7, val,
			     AW_I2C_BYTE_ONE);
	val[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
	val[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG8, val,
			     AW_I2C_BYTE_TWO);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_err("dts_info.cont_track_margin = 0!");
	} else {
		val[0] = (uint8_t)aw_haptic->info.cont_track_margin;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG11, &val[0],
				     AW_I2C_BYTE_ONE);
	}
	/* DRV_WIDTH */
	if (aw_haptic->info.f0_pre)
		val[0] = AW862XX_DRV_WIDTH_FORMULA(aw_haptic->info.f0_pre,
						   aw_haptic->info.cont_track_margin,
						   aw_haptic->info.cont_brk_gain);
	else
		val[0] = AW862XX_BIT_DRV_WIDTH_DEF_VAL;
	aw_info("cont_drv_width=0x%02X", val[0]);
	if (val[0] < AW862XX_DRV_WIDTH_MIN || val[0] > AW862XX_DRV_WIDTH_MAX) {
		aw_err("cont_drv_width[0x%02X] is error, restore default vale[0x%02X]",
			val[0], AW862XX_BIT_DRV_WIDTH_DEF_VAL);
		val[0] = AW862XX_BIT_DRV_WIDTH_DEF_VAL;
	}
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG3, &val[0],
		    AW_I2C_BYTE_ONE);
	/* cont play go */
	aw862xx_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	while (cnt) {
		haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_GLBRD5, &val[0],
				    AW_I2C_BYTE_ONE);
		if ((val[0] & AW_BIT_GLBRD_STATE_MASK) ==
		    AW_BIT_STATE_STANDBY) {
			get_f0_flag = true;
			aw_info("entered standby! glb_state=0x%02X", val[0]);
			break;
		}
		cnt--;
		aw_dbg("waitting for standby,glb_state=0x%02X", val[0]);
		usleep_range(AW_F0_DELAY_MIN, AW_F0_DELAY_MAX);
	}
	if (get_f0_flag) {
		ret = aw862xx_read_lra_f0(aw_haptic);
		if (ret < 0)
			aw_err("read lra f0 is failed");
		aw862xx_read_cont_f0(aw_haptic);
	} else {
		ret = -ERANGE;
		aw_err("enter standby mode failed, stop reading f0!");
	}
	/* disable f0 detect */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG1,
				 AW862XX_BIT_CONTCFG1_EN_F0_DET_MASK,
				 AW862XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_PLAYCFG3,
				 AW862XX_BIT_PLAYCFG3_BRK_EN_MASK, brk_en_temp);
	return ret;
}

static void aw862xx_set_base_addr(struct aw_haptic *aw_haptic)
{
	uint32_t base_addr = 0;
	uint8_t val = 0;

	aw_info("enter");
	base_addr = aw_haptic->ram.base_addr;
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_RTPCFG1,
				 AW862XX_BIT_RTPCFG1_ADDRH_MASK,
				 (uint8_t)AW_SET_BASEADDR_H(base_addr));
	val = (uint8_t)AW_SET_BASEADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_RTPCFG2, &val,
			     AW_I2C_BYTE_ONE);
}

static void aw862xx_set_fifo_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ae_addr_h = 0;
	uint8_t af_addr_h = 0;
	uint8_t val[3] = {0};

	aw_info("enter");
	ae_addr_h = (uint8_t)AW862XX_SET_AEADDR_H(aw_haptic->ram.base_addr);
	af_addr_h = (uint8_t)AW862XX_SET_AFADDR_H(aw_haptic->ram.base_addr);
	val[0] = ae_addr_h | af_addr_h;
	val[1] = (uint8_t)AW862XX_SET_AEADDR_L(aw_haptic->ram.base_addr);
	val[2] = (uint8_t)AW862XX_SET_AFADDR_L(aw_haptic->ram.base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_RTPCFG3, val,
			     AW_I2C_BYTE_THREE);
}

static void aw862xx_get_fifo_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ae_addr_h = 0;
	uint8_t af_addr_h = 0;
	uint8_t ae_addr_l = 0;
	uint8_t af_addr_l = 0;
	uint8_t val[3] = {0};

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_RTPCFG3, val,
			    AW_I2C_BYTE_THREE);
	ae_addr_h = ((val[0]) & AW862XX_BIT_RTPCFG3_FIFO_AEH) >> 4;
	ae_addr_l = val[1];
	af_addr_h = ((val[0]) & AW862XX_BIT_RTPCFG3_FIFO_AFH);
	af_addr_l = val[2];
	aw_info("almost_empty_threshold = %d,almost_full_threshold = %d",
		(uint16_t)((ae_addr_h << 8) | ae_addr_l),
		(uint16_t)((af_addr_h << 8) | af_addr_l));
}

static void aw862xx_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t val[2] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	aw_info("enter");
	val[0] = (uint8_t)AW_SET_RAMADDR_H(base_addr);
	val[1] = (uint8_t)AW_SET_RAMADDR_L(base_addr);
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_RAMADDRH, val,
			     AW_I2C_BYTE_TWO);
}

static void aw862xx_set_ram_data(struct aw_haptic *aw_haptic, uint8_t *data,
				 int len)
{
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_RAMDATA, data, len);
}

static void aw862xx_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint32_t *first_wave_addr)
{
	uint8_t val[3] = {0};

	aw_info("enter");
	aw862xx_set_ram_addr(aw_haptic);
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_RAMDATA, val,
			    AW_I2C_BYTE_THREE);
	*first_wave_addr = (val[1] << 8 | val[2]);
}

#ifdef AW8622X_DRIVER
static void aw862xx_haptic_select_pin(struct aw_haptic *aw_haptic, uint8_t pin)
{
	if (pin == AW_TRIG1) {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_INTN_PIN_MASK,
					 AW862XX_BIT_SYSCTRL2_TRIG1);
		aw_info("select TRIG1 pin");
	} else if (pin == AW_IRQ) {
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
					 AW862XX_BIT_SYSCTRL2_INTN_PIN_MASK,
					 AW862XX_BIT_SYSCTRL2_INTN);
		aw_info("select INIT pin");
	} else
		aw_err("There is no such option");
}

static void aw862xx_haptic_trig_param_init(struct aw_haptic *aw_haptic,
					   uint8_t pin)
{
	struct aw_haptic_dts_info info = aw_haptic->info;

	switch (pin) {
	case AW_TRIG1:
		aw_haptic->trig[0].trig_level = info.trig_cfg[0];
		aw_haptic->trig[0].trig_polar = info.trig_cfg[1];
		aw_haptic->trig[0].pos_enable = info.trig_cfg[2];
		aw_haptic->trig[0].pos_sequence = info.trig_cfg[3];
		aw_haptic->trig[0].neg_enable = info.trig_cfg[4];
		aw_haptic->trig[0].neg_sequence = info.trig_cfg[5];
		aw_haptic->trig[0].trig_brk = info.trig_cfg[6];
		break;
	case AW_TRIG2:
		aw_haptic->trig[1].trig_level = info.trig_cfg[7 + 0];
		aw_haptic->trig[1].trig_polar = info.trig_cfg[7 + 1];
		aw_haptic->trig[1].pos_enable = info.trig_cfg[7 + 2];
		aw_haptic->trig[1].pos_sequence = info.trig_cfg[7 + 3];
		aw_haptic->trig[1].neg_enable = info.trig_cfg[7 + 4];
		aw_haptic->trig[1].neg_sequence = info.trig_cfg[7 + 5];
		aw_haptic->trig[1].trig_brk = info.trig_cfg[7 + 6];
		break;
	case AW_TRIG3:
		aw_haptic->trig[2].trig_level = info.trig_cfg[14 + 0];
		aw_haptic->trig[2].trig_polar = info.trig_cfg[14 + 1];
		aw_haptic->trig[2].pos_enable = info.trig_cfg[14 + 2];
		aw_haptic->trig[2].pos_sequence = info.trig_cfg[14 + 3];
		aw_haptic->trig[2].neg_enable = info.trig_cfg[14 + 4];
		aw_haptic->trig[2].neg_sequence = info.trig_cfg[14 + 5];
		aw_haptic->trig[2].trig_brk = info.trig_cfg[14 + 6];
		break;
	default:
		break;
	}
}

static int aw862xx_haptic_trig_param_config(struct aw_haptic *aw_haptic,
					    uint8_t pin)
{
	uint8_t trig_polar_lev_brk = 0x00;
	uint8_t trig_pos_seq = 0x00;
	uint8_t trig_neg_seq = 0x00;

	if ((aw_haptic->name == AW86224 || aw_haptic->name == AW86225) &&
	   aw_haptic->is_used_irq_pin) {
		aw862xx_haptic_trig_param_init(aw_haptic, AW_TRIG1);
		aw862xx_haptic_select_pin(aw_haptic, AW_IRQ);
		return -ERANGE;
	}
	switch (pin) {
	case AW_TRIG1:
		if (aw_haptic->name == AW86224 || aw_haptic->name == AW86225)
			aw862xx_haptic_select_pin(aw_haptic, AW_TRIG1);
		trig_polar_lev_brk = aw_haptic->trig[0].trig_polar << 2 |
				     aw_haptic->trig[0].trig_level << 1 |
				     aw_haptic->trig[0].trig_brk;
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_TRGCFG7,
					 AW862XX_BIT_TRGCFG7_TRG1_POR_LEV_BRK_MASK,
					 trig_polar_lev_brk << 5);
		trig_pos_seq = aw_haptic->trig[0].pos_enable << 7 |
			       aw_haptic->trig[0].pos_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_TRGCFG1,
				     &trig_pos_seq, AW_I2C_BYTE_ONE);
		trig_neg_seq = aw_haptic->trig[0].neg_enable << 7 |
			       aw_haptic->trig[0].neg_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_TRGCFG4,
				     &trig_neg_seq, AW_I2C_BYTE_ONE);
		aw_info("trig1 config ok!");
		break;
	case AW_TRIG2:
		trig_polar_lev_brk = aw_haptic->trig[1].trig_polar << 2 |
				     aw_haptic->trig[1].trig_level << 1 |
				     aw_haptic->trig[1].trig_brk;
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_TRGCFG7,
					 AW862XX_BIT_TRGCFG7_TRG2_POR_LEV_BRK_MASK,
					 trig_polar_lev_brk << 1);
		trig_pos_seq = aw_haptic->trig[1].pos_enable << 7 |
			       aw_haptic->trig[1].pos_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_TRGCFG2,
				     &trig_pos_seq, AW_I2C_BYTE_ONE);
		trig_neg_seq = aw_haptic->trig[1].neg_enable << 7 |
			       aw_haptic->trig[1].neg_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_TRGCFG5,
				     &trig_neg_seq, AW_I2C_BYTE_ONE);
		aw_info("trig2 config ok!");
		break;
	case AW_TRIG3:
		trig_polar_lev_brk = aw_haptic->trig[2].trig_polar << 2 |
				     aw_haptic->trig[2].trig_level << 1 |
				     aw_haptic->trig[2].trig_brk;
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_TRGCFG8,
					 AW862XX_BIT_TRGCFG8_TRG3_POR_LEV_BRK_MASK,
					 trig_polar_lev_brk << 5);
		trig_pos_seq = aw_haptic->trig[2].pos_enable << 7 |
			       aw_haptic->trig[2].pos_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_TRGCFG3,
				     &trig_pos_seq, AW_I2C_BYTE_ONE);
		trig_neg_seq = aw_haptic->trig[2].neg_enable << 7 |
			       aw_haptic->trig[2].neg_sequence;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_TRGCFG6,
				     &trig_neg_seq, AW_I2C_BYTE_ONE);
		aw_info("trig3 config ok!");
		break;
	default:
		break;
	}
	return 0;
}

static void aw862xx_set_trig(struct aw_haptic *aw_haptic, uint8_t pin)
{
	aw_info("enter");
	aw862xx_haptic_trig_param_init(aw_haptic, pin);
	aw862xx_haptic_trig_param_config(aw_haptic, pin);
}
#endif

static void aw862xx_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
#ifdef AW8622X_DRIVER
	switch (aw_haptic->name) {
	case AW86223:
		aw862xx_set_trig(aw_haptic, AW_TRIG1);
		aw862xx_set_trig(aw_haptic, AW_TRIG2);
		aw862xx_set_trig(aw_haptic, AW_TRIG3);
		break;
	case AW86224:
	case AW86225:
		aw862xx_set_trig(aw_haptic, AW_TRIG1);
		break;
	default:
		break;
	}
#endif
}

static void aw862xx_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_WAVCFG1,
			    aw_haptic->seq, len);
}

static void aw862xx_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav,
				uint8_t seq)
{
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_WAVCFG1 + wav, &seq,
			     AW_I2C_BYTE_ONE);
}

static void aw862xx_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav,
				 uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_nv_i2c_write_bits(aw_haptic,
					 AW862XX_REG_WAVCFG9 + (wav / 2),
					 AW862XX_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_nv_i2c_write_bits(aw_haptic,
					 AW862XX_REG_WAVCFG9 + (wav / 2),
					 AW862XX_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw862xx_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw862xx_set_wav_seq(aw_haptic, 0x00, seq);
	aw862xx_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw862xx_set_wav_loop(aw_haptic, 0x00, AW862XX_BIT_WAVLOOP_INIFINITELY);
}

static void aw862xx_get_gain(struct aw_haptic *aw_haptic, uint8_t *gain)
{
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_PLAYCFG2, gain,
			    AW_I2C_BYTE_ONE);
}

static void aw862xx_get_wav_loop(struct aw_haptic *aw_haptic, uint8_t *val)
{
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_WAVCFG9, val,
			    AW_SEQUENCER_LOOP_SIZE);
}

static void aw862xx_get_ram_data(struct aw_haptic *aw_haptic, uint8_t *ram_data,
				 int size)
{
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_RAMDATA, ram_data, size);
}


static int aw862xx_select_d2s_gain(uint8_t reg)
{
	int d2s_gain = 0;

	switch (reg) {
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_1:
		d2s_gain = 1;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_2:
		d2s_gain = 2;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_4:
		d2s_gain = 4;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_5:
		d2s_gain = 5;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_8:
		d2s_gain = 8;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_10:
		d2s_gain = 10;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_20:
		d2s_gain = 20;
		break;
	case AW862XX_BIT_SYSCTRL7_D2S_GAIN_40:
		d2s_gain = 40;
		break;
	default:
		d2s_gain = -1;
		break;
	}
	return d2s_gain;
}

static void aw862xx_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t d2s_gain_temp = 0;
	uint32_t lra_code = 0;
	uint32_t lra = 0;
	int d2s_gain = 0;

	aw862xx_play_stop(aw_haptic);
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSCTRL7, &reg_val,
			    AW_I2C_BYTE_ONE);
	d2s_gain_temp = AW862XX_BIT_SYSCTRL7_GAIN & reg_val;
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL7,
				 AW862XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);
	d2s_gain = aw862xx_select_d2s_gain(aw_haptic->info.d2s_gain);
	if (d2s_gain < 0) {
		aw_err("d2s_gain is error");
		return;
	}
	aw862xx_raminit(aw_haptic, true);
	/* enter standby mode */
	aw862xx_play_stop(aw_haptic);
	usleep_range(AW_STOP_DELAY_MIN, AW_STOP_DELAY_MAX);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL2,
				 AW862XX_BIT_SYSCTRL2_STANDBY_MASK,
				 AW862XX_BIT_SYSCTRL2_STANDBY_OFF);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_DETCFG1,
				 AW862XX_BIT_DETCFG1_RL_OS_MASK,
				 AW862XX_BIT_DETCFG1_RL);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_DETCFG2,
				 AW862XX_BIT_DETCFG2_DIAG_GO_MASK,
				 AW862XX_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(AW_RL_DELAY_MIN, AW_RL_DELAY_MAX);
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_DET_RL, &reg_val,
			    AW_I2C_BYTE_ONE);
	lra_code = (lra_code | reg_val) << 2;
	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_DET_LO, &reg_val,
			    AW_I2C_BYTE_ONE);
	lra_code = lra_code | (reg_val & AW862XX_BIT_DET_LO_RL);
	/* 2num */
	lra = AW862XX_RL_FORMULA(lra_code, d2s_gain);
	/* Keep up with aw8624 driver */
	aw_haptic->lra = lra * 10;
	aw862xx_raminit(aw_haptic, false);
	haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_SYSCTRL7,
				 AW862XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
				 d2s_gain_temp);
}

static uint8_t aw862xx_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_PWMCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static uint8_t aw862xx_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw862xx_get_glb_state(aw_haptic);
	if (glb_state == AW_BIT_STATE_RTP_GO) {
		rtp_state = 1;  /*is going on */
		aw_info("rtp_routine_on");
	}
	return rtp_state;
}

static uint64_t aw862xx_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSCTRL2, &reg_val,
			    AW_I2C_BYTE_ONE);
	fre_val = reg_val & AW862XX_BIT_SYSCTRL2_RATE;
	if (fre_val == AW862XX_BIT_SYSCTRL2_RATE_48K)
		theory_time = (aw_haptic->rtp_len / 48000) * 1000000;	/*48K*/
	else if (fre_val == AW862XX_BIT_SYSCTRL2_RATE_24K)
		theory_time = (aw_haptic->rtp_len / 24000) * 1000000;	/*24K*/
	else
		theory_time = (aw_haptic->rtp_len / 12000) * 1000000;	/*12K*/
	aw_info("microsecond:%llu  theory_time = %llu",
		aw_haptic->microsecond, theory_time);
	return theory_time;
}

static uint8_t aw862xx_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_SYSST2, &state,
			    AW_I2C_BYTE_ONE);
	state &= AW862XX_BIT_SYSST2_FF_EMPTY;
	return state;
}

static int aw862xx_check_qualify(struct aw_haptic *aw_haptic)
{
	uint8_t reg = 0;
	int ret = 0;

	aw_info("enter");
	ret = haptic_nv_i2c_reads(aw_haptic, AW862XX_REG_EFRD9, &reg,
				  AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;
	if ((reg & 0x80) == 0x80)
		return 0;
	aw_err("register 0x64 error: 0x%02X", reg);
	return -ERANGE;
}

static ssize_t cont_wait_num_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_wait_num = 0x%02X\n",
			aw_haptic->info.cont_wait_num);
	return len;
}

static ssize_t cont_wait_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint8_t reg_val = 0;
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
	aw_haptic->info.cont_wait_num = val;
	reg_val = aw_haptic->info.cont_wait_num;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG4, &reg_val,
			     AW_I2C_BYTE_ONE);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t cont_drv_lvl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl,
			aw_haptic->info.cont_drv2_lvl);
	return len;
}

static ssize_t cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint8_t reg_val = 0;
	uint32_t databuf[2] = { 0, 0 };

	mutex_lock(&aw_haptic->lock);
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		haptic_nv_i2c_write_bits(aw_haptic, AW862XX_REG_CONTCFG6,
					 AW862XX_BIT_CONTCFG6_DRV1_LVL_MASK,
					 aw_haptic->info.cont_drv1_lvl);
		reg_val = (uint8_t)aw_haptic->info.cont_drv2_lvl;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG7, &reg_val,
				     AW_I2C_BYTE_ONE);
	}
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t cont_drv_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw_haptic->info.cont_drv1_time,
			aw_haptic->info.cont_drv2_time);
	return len;
}

static ssize_t cont_drv_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint8_t reg_val[2] = {0};
	uint32_t databuf[2] = { 0, 0 };

	mutex_lock(&aw_haptic->lock);
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_time = databuf[0];
		aw_haptic->info.cont_drv2_time = databuf[1];
		reg_val[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
		reg_val[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
		haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG8, reg_val,
				     AW_I2C_BYTE_TWO);
	}
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static DEVICE_ATTR(cont_wait_num, 0664, cont_wait_num_show,
		   cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, 0664, cont_drv_lvl_show, cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, 0664, cont_drv_time_show,
		   cont_drv_time_store);

static struct attribute *aw862xx_vibrator_attributes[] = {
	&dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	NULL
};

static struct attribute_group aw862xx_vibrator_attribute_group = {
	.attrs = aw862xx_vibrator_attributes
};

#ifdef AW8622X_DRIVER
static ssize_t cont_brk_time_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw_haptic->info.cont_brk_time);
	return len;
}

static ssize_t cont_brk_time_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint8_t reg_val = 0;
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
	aw_haptic->info.cont_brk_time = val;
	reg_val = aw_haptic->info.cont_brk_time;
	haptic_nv_i2c_writes(aw_haptic, AW862XX_REG_CONTCFG10, &reg_val,
			     AW_I2C_BYTE_ONE);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t trig_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int i = 0;
	ssize_t len = 0;
	uint8_t trig_num = 3;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (aw_haptic->name == AW86224 || aw_haptic->name == AW86225)
		trig_num = 1;
	for (i = 0; i < trig_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: trig_level=%d, trig_polar=%d",
				i + 1,
				aw_haptic->trig[i].trig_level,
				aw_haptic->trig[i].trig_polar);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"pos_enable=%d, pos_sequence=%d,",
				aw_haptic->trig[i].pos_enable,
				aw_haptic->trig[i].pos_sequence);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"neg_enable=%d, neg_sequence=%d trig_brk=%d\n",
				aw_haptic->trig[i].neg_enable,
				aw_haptic->trig[i].neg_sequence,
				aw_haptic->trig[i].trig_brk);
	}
	return len;
}

static ssize_t trig_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[9] = { 0 };

	if (sscanf(buf, "%d %d %d %d %d %d %d %d",
		&databuf[0], &databuf[1], &databuf[2],
		&databuf[3], &databuf[4], &databuf[5],
		&databuf[6], &databuf[7]) == 8) {
		aw_info("%d, %d, %d, %d, %d, %d, %d, %d",
			databuf[0], databuf[1], databuf[2], databuf[3],
			databuf[4], databuf[5], databuf[6], databuf[7]);
		if ((aw_haptic->name == AW86224 ||
		     aw_haptic->name == AW86225) && (databuf[0])) {
			aw_err("input seq value out of range!");
			return count;
		}
		if (databuf[0] < 0 || databuf[0] > 2) {
			aw_err("input seq value out of range!");
			return count;
		}
		if (!aw_haptic->ram_init) {
			aw_err("ram init failed, not allow to play!");
			return count;
		}
		if (databuf[4] > aw_haptic->ram.ram_num ||
		    databuf[6] > aw_haptic->ram.ram_num) {
			aw_err("input seq value out of range!");
			return count;
		}
		aw_haptic->trig[databuf[0]].trig_level = databuf[1];
		aw_haptic->trig[databuf[0]].trig_polar = databuf[2];
		aw_haptic->trig[databuf[0]].pos_enable = databuf[3];
		aw_haptic->trig[databuf[0]].pos_sequence = databuf[4];
		aw_haptic->trig[databuf[0]].neg_enable = databuf[5];
		aw_haptic->trig[databuf[0]].neg_sequence = databuf[6];
		aw_haptic->trig[databuf[0]].trig_brk = databuf[7];
		mutex_lock(&aw_haptic->lock);
		aw862xx_haptic_trig_param_config(aw_haptic, databuf[0]);
		mutex_unlock(&aw_haptic->lock);
	} else
		aw_err("please input eight parameters");
	return count;
}

static DEVICE_ATTR(cont_brk_time, 0664, cont_brk_time_show,
		   cont_brk_time_store);
static DEVICE_ATTR(trig, 0664, trig_show, trig_store);

static struct attribute *aw8622x_vibrator_attributes[] = {
	&dev_attr_cont_brk_time.attr,
	&dev_attr_trig.attr,
	NULL
};

static struct attribute_group aw8622x_vibrator_attribute_group = {
	.attrs = aw8622x_vibrator_attributes
};
#endif

static void aw862xx_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw862xx_vibrator_attribute_group);
	if (ret < 0)
		aw_err("error create aw862xx sysfs attr files");
#ifdef AW8622X_DRIVER
	if (aw_haptic->name == AW86223 || aw_haptic->name == AW86224 ||
	    aw_haptic->name == AW86225) {
		ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw8622x_vibrator_attribute_group);
		if (ret < 0)
			aw_err("error create aw8622x sysfs attr files");
	}
#endif
}

struct aw_haptic_func aw862xx_func_list = {
	.ram_init = aw862xx_raminit,
	.trig_init = aw862xx_trig_init,
	.play_mode = aw862xx_play_mode,
	.play_stop = aw862xx_play_stop,
	.irq_clear = aw862xx_irq_clear,
	.creat_node = aw862xx_creat_node,
	.cont_config = aw862xx_cont_config,
	.offset_cali = aw862xx_offset_cali,
	.haptic_start = aw862xx_haptic_start,
	.read_cont_f0 = aw862xx_read_cont_f0,
	.check_qualify = aw862xx_check_qualify,
	.judge_rtp_going = aw862xx_judge_rtp_going,
	.protect_config = aw862xx_protect_config,
	.misc_para_init = aw862xx_misc_para_init,
	.interrupt_setup = aw862xx_interrupt_setup,
	.rtp_get_fifo_afs = aw862xx_rtp_get_fifo_afs,
	.vbat_mode_config = aw862xx_vbat_mode_config,
	.calculate_cali_data = aw862xx_calculate_cali_data,
	.set_gain = aw862xx_set_gain,
	.get_gain = aw862xx_get_gain,
	.set_wav_seq = aw862xx_set_wav_seq,
	.get_wav_seq = aw862xx_get_wav_seq,
	.set_wav_loop = aw862xx_set_wav_loop,
	.get_wav_loop = aw862xx_get_wav_loop,
	.set_ram_data = aw862xx_set_ram_data,
	.get_ram_data = aw862xx_get_ram_data,
	.set_fifo_addr = aw862xx_set_fifo_addr,
	.get_fifo_addr = aw862xx_get_fifo_addr,
	.set_rtp_aei = aw862xx_set_rtp_aei,
	.set_rtp_data = aw862xx_set_rtp_data,
	.set_ram_addr = aw862xx_set_ram_addr,
	.set_trim_lra = aw862xx_set_trim_lra,
	.set_base_addr = aw862xx_set_base_addr,
	.set_repeat_seq = aw862xx_set_repeat_seq,
	.get_f0 = aw862xx_get_f0,
	.get_reg = aw862xx_get_reg,
	.get_vbat = aw862xx_get_vbat,
	.get_prctmode = aw862xx_get_prctmode,
	.get_irq_state = aw862xx_get_irq_state,
	.get_glb_state = aw862xx_get_glb_state,
	.get_osc_status = aw862xx_get_osc_status,
	.get_theory_time = aw862xx_get_theory_time,
	.get_lra_resistance = aw862xx_get_lra_resistance,
	.get_first_wave_addr = aw862xx_get_first_wave_addr,
};
