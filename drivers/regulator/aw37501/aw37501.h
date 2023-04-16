/*
 * Copyright (c) aw37501, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_AW37501_LED_H__
#define __LINUX_AW37501_LED_H__

#define AW37501_REG_VOUTP	0x00
#define AW37501_REG_VOUTN	0x01
#define AW37501_REG_APPS	0x03
#define AW37501_REG_CTRL	0x04
#define AW37501_REG_PRO		0x21

#define AW37501_REG_MAX		0x2f
#define AW37501_OPEN_CMD	0x4C
#define AW37501_CLOSE_CMD	0x00

#define AW37501_CHIP_ID		0x01
#define AW37501_GET_CHIP_ID	0x03

#define AW37501_OUT_CLEAR	0xE0

/*register read/write access*/
#define REG_NONE_ACCESS		0
#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)

#define AW37501_RESET		(1 << 5)

#define AW37501_DISN_ENABLE	(1 << 0)
#define AW37501_DISN_DISABLE	(~(1 << 0))
#define AW37501_DISP_ENABLE	(1 << 1)
#define AW37501_DISP_DISABLE	(~(1 << 1))

#define AW37501_ENN_ENABLE	(1 << 4)
#define AW37501_ENN_DISABLE	(~(1 << 4))
#define AW37501_ENP_ENABLE	(1 << 3)
#define AW37501_ENP_DISABLE	(~(1 << 3))
#define AW37501_ENP_IS_ENABLE	(1 << 3)
#define AW37501_ENN_IS_ENABLE	(~(1 << 3))

#define AW_I2C_RETRIES		5

#define AW_OFFSET_0MA		0
#define AW_OFFSET_50MA		50
#define AW_OFFSET_100MA		100
#define AW_OFFSET_150MA		150

#define AW37501_OFFSET_CLEAR	(~(3 << 6))
#define AW37501_OFFSET_GET	(3 << 6)

#define AW_OFFSET_0MA_VAL	0
#define AW_OFFSET_50MA_VAL	(1 << 6)
#define AW_OFFSET_100MA_VAL	(1 << 7)
#define AW_OFFSET_150MA_VAL	(3 << 6)


#define AW_CURRENT_40MA		40000
#define AW_CURRENT_80MA		80000
#define AW_CURRENT_100MA	100000

#define AW37501_CURRENT_GET	(3 << 6)
#define AW37501_CURRENT_CLEAR	(~(3 << 6))
#define AW_CURRENT_40MA_VAL	0
#define AW_CURRENT_80MA_VAL	(1 << 6)
#define AW_CURRENT_100MA_VAL	(1 << 7)

#define MIN_VOLTAGE		4000000
#define MAX_VOLTAGE		6000000
#define MIN_VOLTAGE_VAL		0x00
#define MAX_VOLTAGE_VAL		0x14

#define N_VOLTAGE		21
#define UV_STEP			100000

#define MAX_LOO_CURRENT		370
#define MIN_LOD_CURRENT		270
#define MAX_LOO_CURRENT_VAL	(~(1 << 4))
#define MIN_LOD_CURRENT_VAL	(1 << 4)

struct aw37501_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *aw_enn_default;
	struct pinctrl_state *aw_enn_high;
	struct pinctrl_state *aw_enn_low;
	struct pinctrl_state *aw_enp_default;
	struct pinctrl_state *aw_enp_high;
	struct pinctrl_state *aw_enp_low;
};

struct aw37501_power {
	struct i2c_client *client;
	struct device *dev;
	struct mutex lock;
	struct aw37501_pinctrl pinctrl;
	struct regulator_init_data *init_data;
	struct regulator_dev *regulators[2];

	int enn_gpio;
	int enp_gpio;
	int offset;
	int limit;
	int power_mode;
	int read_power_mode;
	int outp;
	int outn;
	int read_outp;
	int read_outn;
	bool bool_gpio_ctrl;
	bool bool_regulator_fixed;

	u8 chip_id;
};

const unsigned char aw37501_reg_access[AW37501_REG_MAX] = {
	[AW37501_REG_VOUTP] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW37501_REG_VOUTN] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW37501_REG_APPS]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW37501_REG_CTRL]  = REG_RD_ACCESS|REG_WR_ACCESS,
};

static const unsigned int aw_vout_map_outp[] = {
	4000000, 4100000, 4200000, 4300000, 4400000,
	4500000, 4600000, 4700000, 4800000, 4900000,
	5000000, 5100000, 5200000, 5300000, 5400000,
	5500000, 5600000, 5700000, 5800000, 5900000,
	6000000,
};
static const int aw_vout_map_outn[] = {
	-4000000, -4100000, -4200000, -4300000, -4400000,
	-4500000, -4600000, -4700000, -4800000, -4900000,
	-5000000, -5100000, -5200000, -5300000, -5400000,
	-5500000, -5600000, -5700000, -5800000, -5900000,
	-6000000,
};

static const unsigned int aw_vout_regval[] = {
	0x00, 0x01, 0x02, 0x03, 0x04,
	0x05, 0x06, 0x07, 0x08, 0x09,
	0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
	0x0f, 0x10, 0x11, 0x12, 0x13,
	0x14,
};

#endif
