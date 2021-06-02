/*
 * TI LMU (Lighting Management Unit) Backlight Device Data
 *
 * Copyright 2016 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include "ti_lm3697.h"
#include "ti_lm3697_regulator.h"
#include "ti-lmu.h"

/* LM3697 */
static u32 lm3697_init_regs[] = {
	//LM3697_INIT_RAMP_SELECT,
	LM3697_REG_HVLED_OUTPUT_CFG,
	LM3697_REG_BOOST_CFG,
	LM3697_REG_PWM_CFG,
	LM3697_REG_BRT_B_LSB,
	LM3697_REG_BRT_B_MSB,
	LM3697_REG_ENABLE,
};

static u32 lm3697_channel_regs[]  = {
	LM3697_CHANNEL_1,
	LM3697_CHANNEL_2,
	LM3697_CHANNEL_3,
};

static u32 lm3697_mode_regs[] = {
	LM3697_MODE_PWM_B,
};

static u32 lm3697_ramp_regs[] = {
	LM3697_RAMPUP,
	LM3697_RAMPDN,
};

static u8 lm3697_enable_reg = LM3697_REG_ENABLE;

static u8 lm3697_brightness_msb_regs[] = {
	LM3697_REG_BRT_B_MSB,
};

static u8 lm3697_brightness_lsb_regs[] = {
	LM3697_REG_BRT_B_LSB,
};

static const struct ti_lmu_bl_reg lm3697_reg_info = {
	.init		 = lm3697_init_regs,
	.num_init	 = ARRAY_SIZE(lm3697_init_regs),
	.channel	 = lm3697_channel_regs,
	.mode		 = lm3697_mode_regs,
	.ramp		 = lm3697_ramp_regs,
	.ramp_reg_offset = 1, /* For LM3697_REG_BL1_RAMPUP/DN */
	.enable		 = &lm3697_enable_reg,
	.brightness_msb	 = lm3697_brightness_msb_regs,
	.brightness_lsb	 = lm3697_brightness_lsb_regs,
};

static int common_ramp_table[] = {
	   2, 250, 500, 1000, 2000, 4000, 8000, 16000,
};

struct ti_lmu_bl_cfg lmu_bl_cfg[LMU_MAX_ID] = {
	{
		.reginfo		= &lm3697_reg_info,
		.num_channels		= LM3697_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_11BIT,
		.pwm_action		= UPDATE_PWM_AND_BRT_REGISTER,
		.ramp_table		= common_ramp_table,
		.size_ramp		= ARRAY_SIZE(common_ramp_table),
		.fault_monitor_used	= true,
	},
};
EXPORT_SYMBOL_GPL(lmu_bl_cfg);
