// SPDX-License-Identifier: GPL-2.0+
/*
 * wl2868c, Multi-Output Regulators
 * Copyright (C) 2019  Motorola Mobility LLC,
 *
 * Author: ChengLong, Motorola Mobility LLC,
 */

#ifndef __wl2868c_REGISTERS_H__
#define __wl2868c_REGISTERS_H__

/* Registers */
#define wl2868c_REG_NUM (wl2868c_SEQ_STATUS - wl2868c_CHIP_REV + 1)

#define wl2868c_CHIP_REV 0x00
#define wl2868c_CURRENT_LIMITSEL 0x01
#define wl2868c_DISCHARGE_RESISTORS 0x02
#define wl2868c_LDO1_VOUT 0x03
#define wl2868c_LDO2_VOUT 0x04
#define wl2868c_LDO3_VOUT 0x05
#define wl2868c_LDO4_VOUT 0x06
#define wl2868c_LDO5_VOUT 0x07
#define wl2868c_LDO6_VOUT 0x08
#define wl2868c_LDO7_VOUT 0x09
#define wl2868c_LDO1_LDO2_SEQ 0x0a
#define wl2868c_LDO3_LDO4_SEQ 0x0b
#define wl2868c_LDO5_LDO6_SEQ 0x0c
#define wl2868c_LDO7_SEQ 0x0d
#define wl2868c_LDO_EN 0x0e
#define wl2868c_SEQ_STATUS 0x0f


/* wl2868c_LDO1_VSEL ~ wl2868c_LDO7_VSEL =
 * 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
 */
#define  wl2868c_LDO1_VSEL                      wl2868c_LDO1_VOUT
#define  wl2868c_LDO2_VSEL                      wl2868c_LDO2_VOUT
#define  wl2868c_LDO3_VSEL                      wl2868c_LDO3_VOUT
#define  wl2868c_LDO4_VSEL                      wl2868c_LDO4_VOUT
#define  wl2868c_LDO5_VSEL                      wl2868c_LDO5_VOUT
#define  wl2868c_LDO6_VSEL                      wl2868c_LDO6_VOUT
#define  wl2868c_LDO7_VSEL                      wl2868c_LDO7_VOUT

#define  wl2868c_VSEL_SHIFT                     0
#define  wl2868c_VSEL_MASK                      (0xff << 0)

#endif /* __wl2868c_REGISTERS_H__ */
