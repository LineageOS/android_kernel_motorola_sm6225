// SPDX-License-Identifier: GPL-2.0+
/*
 * wl2868c, Multi-Output Regulators
 * Copyright (C) 2019  Motorola Mobility LLC,
 *
 * Author: ChengLong, Motorola Mobility LLC,
 */

#ifndef __ET5907_REGISTERS_H__
#define __ET5907_REGISTERS_H__

/* Registers */
#define ET5907_REG_NUM (ET5907_SEQ_STATUS - ET5907_CHIP_REV + 1)

#define ET5907_CHIP_ID 0x00
#define ET5907_CHIP_REV 0x01
#define ET5907_CURRENT_LIMITSEL 0x02
#define ET5907_LDO_EN 0x03
#define ET5907_LDO1_VOUT 0x04
#define ET5907_LDO2_VOUT 0x05
#define ET5907_LDO3_VOUT 0x06
#define ET5907_LDO4_VOUT 0x07
#define ET5907_LDO5_VOUT 0x08
#define ET5907_LDO6_VOUT 0x09
#define ET5907_LDO7_VOUT 0x0a
#define ET5907_SEQ_STATUS 0x0f

#define ET5907_LDO_OFFSET 0x01

/* ET5907_LDO1_VSEL ~ ET5907_LDO7_VSEL =
 * 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a
 */
#define  ET5907_LDO1_VSEL                      ET5907_LDO1_VOUT
#define  ET5907_LDO2_VSEL                      ET5907_LDO2_VOUT
#define  ET5907_LDO3_VSEL                      ET5907_LDO3_VOUT
#define  ET5907_LDO4_VSEL                      ET5907_LDO4_VOUT
#define  ET5907_LDO5_VSEL                      ET5907_LDO5_VOUT
#define  ET5907_LDO6_VSEL                      ET5907_LDO6_VOUT
#define  ET5907_LDO7_VSEL                      ET5907_LDO7_VOUT

#define  ET5907_VSEL_SHIFT                     0
#define  ET5907_VSEL_MASK                      (0xff << 0)

#endif /* __ET5907_REGISTERS_H__ */

