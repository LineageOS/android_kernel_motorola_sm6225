/*
 * TI LMU (Lighting Management Unit) Backlight Common Driver
 *
 * Copyright 2016 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TI_LMU_BACKLIGHT_H__
#define __TI_LMU_BACKLIGHT_H__

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include "ti_lm3697_regulator.h"
#include "ti-lmu.h"
/**
 * LMU backlight register data
 *	value[23:16] | mask[15:8] | address[7:0]
 */
#define LMU_BL_REG(addr, mask, value)				\
	((value << 16) | (mask << 8) | addr)

#define LMU_BL_GET_ADDR(x)	(x & 0xFF)
#define LMU_BL_GET_MASK(x)	((x >> 8) & 0xFF)
#define LMU_BL_GET_VAL(x)	((x >> 16) & 0xFF)

#define LM3697_INIT_RAMP_SELECT						\
	LMU_BL_REG(LM3697_REG_RAMP_CONF, LM3697_RAMP_MASK, LM3697_RAMP_EACH)
#define LM3697_CHANNEL_1						\
	LMU_BL_REG(LM3697_REG_HVLED_OUTPUT_CFG, LM3697_HVLED1_CFG_MASK,	\
	LM3697_HVLED1_CFG_SHIFT)
#define LM3697_CHANNEL_2						\
	LMU_BL_REG(LM3697_REG_HVLED_OUTPUT_CFG, LM3697_HVLED2_CFG_MASK,	\
	LM3697_HVLED2_CFG_SHIFT)
#define LM3697_CHANNEL_3						\
	LMU_BL_REG(LM3697_REG_HVLED_OUTPUT_CFG, LM3697_HVLED3_CFG_MASK,	\
	LM3697_HVLED3_CFG_SHIFT)
#define LM3697_MODE_PWM_A						\
	LMU_BL_REG(LM3697_REG_PWM_CFG, LM3697_PWM_A_MASK, LM3697_PWM_A_MASK)
#define LM3697_MODE_PWM_B						\
	LMU_BL_REG(LM3697_REG_PWM_CFG, LM3697_PWM_B_MASK, LM3697_PWM_B_MASK)
#define LM3697_RAMPUP							\
	LMU_BL_REG(LM3697_REG_BL0_RAMP, LM3697_RAMPUP_MASK, LM3697_RAMPUP_SHIFT)
#define LM3697_RAMPDN							\
	LMU_BL_REG(LM3697_REG_BL0_RAMP, LM3697_RAMPDN_MASK, LM3697_RAMPDN_SHIFT)

#define LM3697_MAX_CHANNELS		3

#define MAX_BRIGHTNESS_8BIT		255
#define MAX_BRIGHTNESS_11BIT		2047

enum ti_lmu_bl_ctrl_mode {
	BL_REGISTER_BASED,
	BL_PWM_BASED,
};

enum ti_lmu_bl_pwm_action {
	/* Update PWM duty, no brightness register update is required */
	UPDATE_PWM_ONLY,
	/* Update not only duty but also brightness register */
	UPDATE_PWM_AND_BRT_REGISTER,
	/* Update max value in brightness registers */
	UPDATE_MAX_BRT,
};

enum ti_lmu_bl_ramp_mode {
	BL_RAMP_UP,
	BL_RAMP_DOWN,
};

struct ti_lmu_bl;
struct ti_lmu_bl_chip;

/**
 * struct ti_lmu_bl_reg
 *
 * @init:		Device initialization registers
 * @num_init:		Numbers of initialization registers
 * @channel:		Backlight channel configuration registers
 * @mode:		Brightness control mode registers
 * @ramp:		Ramp registers for lighting effect
 * @ramp_reg_offset:	Ramp register offset.
 *			Only used for multiple ramp registers.
 * @enable:		Enable control register address
 * @enable_offset:	Enable control register bit offset
 * @enable_usec:	Delay time for updating enable register.
 *			Unit is microsecond.
 * @brightness_msb:	Brightness MSB(Upper 8 bits) registers.
 *			Concatenated with LSB in 11 bit dimming mode.
 *			In 8 bit dimming, only MSB is used.
 * @brightness_lsb:	Brightness LSB(Lower 3 bits) registers.
 *			Only valid in 11 bit dimming mode.
 */
struct ti_lmu_bl_reg {
	u32 *init;
	int num_init;
	u32 *channel;
	u32 *mode;
	u32 *ramp;
	int ramp_reg_offset;
	u8 *enable;
	u8 enable_offset;
	unsigned long enable_usec;
	u8 *brightness_msb;
	u8 *brightness_lsb;
};

/**
 * struct ti_lmu_bl_cfg
 *
 * @reginfo:		Device register configuration
 * @num_channels:	Number of backlight channels
 * @single_bank_used:	[Optional] Set true if one bank controls multiple channels.
 *			Only used for LM36274.
 * @max_brightness:	Max brightness value of backlight device
 * @pwm_action:		How to control brightness registers in PWM mode
 * @ramp_table:		[Optional] Ramp time table for lighting effect.
 *			It's used for searching approximate register index.
 * @size_ramp:		[Optional] Size of ramp table
 * @fault_monitor_used:	[Optional] Set true if the device needs to handle
 *			LMU fault monitor event.
 *
 * This structure is used for device specific data configuration.
 */
struct ti_lmu_bl_cfg {
	const struct ti_lmu_bl_reg *reginfo;
	int num_channels;
	bool single_bank_used;
	int max_brightness;
	enum ti_lmu_bl_pwm_action pwm_action;
	int *ramp_table;
	int size_ramp;
	bool fault_monitor_used;
};

/**
 * struct ti_lmu_bl_chip
 *
 * @dev:		Parent device pointer
 * @lmu:		LMU structure.
 *			Used for register R/W access and notification.
 * @cfg:		Device configuration data
 * @lmu_bl:		Multiple backlight channels
 * @num_backlights:	Number of backlight channels
 * @nb:			Notifier block for handling LMU fault monitor event
 *
 * One backlight chip can have multiple backlight channels, 'ti_lmu_bl'.
 */
struct ti_lmu_bl_chip {
	struct device *dev;
	struct ti_lmu *lmu;
	const struct ti_lmu_bl_cfg *cfg;
	struct ti_lmu_bl *lmu_bl;
	int num_backlights;
	struct notifier_block nb;
};

/**
 * struct ti_lmu_bl
 *
 * @chip:		Pointer to parent backlight device
 * @bl_dev:		Backlight subsystem device structure
 * @bank_id:		Backlight bank ID
 * @name:		Backlight channel name
 * @mode:		Backlight control mode
 * @led_sources:	Backlight output channel configuration.
 *			Bit mask is set on parsing DT.
 * @default_brightness:	[Optional] Initial brightness value
 * @ramp_up_msec:	[Optional] Ramp up time
 * @ramp_down_msec:	[Optional] Ramp down time
 * @pwm_period:		[Optional] PWM period
 * @pwm:		[Optional] PWM subsystem structure
 *
 * Each backlight device has its own channel configuration.
 * For chip control, parent chip data structure is used.
 */
struct ti_lmu_bl {
	struct ti_lmu_bl_chip *chip;
	struct backlight_device *bl_dev;

	int bank_id;
	const char *name;
	enum ti_lmu_bl_ctrl_mode mode;
	unsigned long led_sources;

	unsigned int default_brightness;

	/* Used for lighting effect */
	unsigned int ramp_up_msec;
	unsigned int ramp_down_msec;

	/* Only valid in PWM mode */
	unsigned int pwm_period;

	/* current mode */
	unsigned int led_current_mode;

	/* Boost OVP */
	unsigned int led_boost_ovp;
	/* Boost frequency */
	unsigned int led_boost_freq;

	/* Map type */
	unsigned int map_type;

	/* Align boost current to AW chip */
	unsigned int led_current_align;

	struct pwm_device *pwm;
};

enum backlight_hbm_mode {
	HBM_MODE_UN_SET,
	HBM_MODE_DEFAULT = 1,
	HBM_MODE_LEVEL1,	//CURRENT = HBM_MODE_DEFAULT*112.5%
	HBM_MODE_LEVEL2,	//CURRENT = HBM_MODE_DEFAULT*125%
	HBM_MODE_LEVEL3,	//CURRENT = HBM_MODE_DEFAULT*107%
	HBM_MODE_LEVEL4,	//CURRENT = HBM_MODE_DEFAULT*111%
	HBM_MODE_LEVEL_MAX
};

enum backlight_boost_ovp {
	BOOST_OVP_16V,
	BOOST_OVP_24V,
	BOOST_OVP_32V,
	BOOST_OVP_40V
};

enum backlight_boost_freq {
	BOOST_FREQ_500K,
	BOOST_FREQ_1M
};

enum backlight_map_type {
	EXPONENTIAL_TYPE,
	LINEAR_TYPE
};

enum backlight_exp_current_align {
	ALIGN_NONE,
	ALIGN_AW99703
};

extern struct ti_lmu_bl_cfg lmu_bl_cfg[LMU_MAX_ID];
extern int ti_lmu_backlight_device_init(void);
extern void ti_lmu_backlight_exit(void);
#endif
