/*
 * TI LMU (Lighting Management Unit) Device Register Map
 *
 * Copyright 2017 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MFD_TI_LMU_REGISTER_H__
#define __MFD_TI_LMU_REGISTER_H__

#include <linux/bitops.h>

/* LM3697 */
#define LM3697_REG_HVLED_OUTPUT_CFG		0x10
#define LM3697_HVLED1_CFG_MASK			BIT(0)
#define LM3697_HVLED2_CFG_MASK			BIT(1)
#define LM3697_HVLED3_CFG_MASK			BIT(2)
#define LM3697_HVLED1_CFG_SHIFT			0
#define LM3697_HVLED2_CFG_SHIFT			1
#define LM3697_HVLED3_CFG_SHIFT			2

#define LM3697_REG_BL0_RAMP			0x11
#define LM3697_REG_BL1_RAMP			0x12
#define LM3697_RAMPUP_MASK			0xF0
#define LM3697_RAMPUP_SHIFT			4
#define LM3697_RAMPDN_MASK			0x0F
#define LM3697_RAMPDN_SHIFT			0

#define LM3697_REG_RAMP_CONF			0x14
#define LM3697_RAMP_MASK			0x0F
#define LM3697_RAMP_EACH			0x05

#define LM3697_REG_BOOST_CFG		0x1A

#define LM3697_REG_PWM_CFG			0x1C
#define LM3697_PWM_A_MASK			BIT(0)
#define LM3697_PWM_B_MASK			BIT(1)

#define LM3697_REG_IMAX_A			0x17
#define LM3697_REG_IMAX_B			0x18

#define LM3697_REG_FEEDBACK_ENABLE		0x19

#define LM3697_REG_BRT_A_LSB			0x20
#define LM3697_REG_BRT_A_MSB			0x21
#define LM3697_REG_BRT_B_LSB			0x22
#define LM3697_REG_BRT_B_MSB			0x23

#define LM3697_REG_ENABLE			0x24

#define LM3697_REG_OPEN_FAULT_STATUS		0xB0

#define LM3697_REG_SHORT_FAULT_STATUS		0xB2

#define LM3697_REG_MONITOR_ENABLE		0xB4

#define LM3697_MAX_REG				0xB4
#endif
