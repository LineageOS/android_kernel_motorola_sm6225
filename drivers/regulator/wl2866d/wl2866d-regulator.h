// SPDX-License-Identifier: GPL-2.0+
/*
 * WL2866D, Multi-Output Regulators
 * Copyright (C) 2019  Motorola Mobility LLC,
 *
 * Author: ChengLong, Motorola Mobility LLC,
 */

#ifndef __WL2866D_REGISTERS_H__
#define __WL2866D_REGISTERS_H__

/* Registers */
#define WL2866D_REG_NUM (WL2866D_SEQ_STATUS-WL2866D_CHIP_REV+1)

#define WL2866D_CHIP_REV 0x00
#define WL2866D_CURRENT_LIMITSEL 0x01
#define WL2866D_DISCHARGE_RESISTORS 0x02
#define WL2866D_LDO1_VOUT 0x03
#define WL2866D_LDO2_VOUT 0x04
#define WL2866D_LDO3_VOUT 0x05
#define WL2866D_LDO4_VOUT 0x06
#define WL2866D_LDO1_LDO2_SEQ 0x0a
#define WL2866D_LDO3_LDO4_SEQ 0x0b
#define WL2866D_LDO_EN 0x0e
#define WL2866D_SEQ_STATUS 0x0f


/* WL2866D_LDO1_VSEL ~ WL2866D_LDO4_VSEL =
 * 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
 */
#define  WL2866D_LDO1_VSEL                      WL2866D_LDO1_VOUT
#define  WL2866D_LDO2_VSEL                      WL2866D_LDO2_VOUT
#define  WL2866D_LDO3_VSEL                      WL2866D_LDO3_VOUT
#define  WL2866D_LDO4_VSEL                      WL2866D_LDO4_VOUT


#define  WL2866D_VSEL_SHIFT                     0
#define  WL2866D_VSEL_MASK                      (0xff << 0)

#endif /* __WL2866D_REGISTERS_H__ */
