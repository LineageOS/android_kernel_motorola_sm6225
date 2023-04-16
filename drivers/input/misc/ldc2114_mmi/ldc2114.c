/*
 * LDC2114 Metal Touch Inductance-to-Digital Converter
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/memory.h>
#include <linux/input.h>
#include <linux/reboot.h>

#include "ldc2114.h"

#define LDC2114_DRIVER_NAME		"ldc2114"

#define LDC2114_EV_DATA 1
#define LDC2114_EV_IRQ 2
#define LDC2114_EV_PRESS 3

#define LDC2114_SHORT_DELAY	100
#define LDC2114_LONG_DELAY	(LDC2114_SHORT_DELAY * 5)
#define LDC2114_MAX_RETRIES	10

#define CONFIG_LDC2114_MAX_FAILURES 100

#define WAIT_USEC_50	(50 * USEC_PER_MSEC)
#define WAIT_USEC_60	(60 * USEC_PER_MSEC)
#define WAIT_USEC_LOW	(LDC2114_SHORT_DELAY * USEC_PER_MSEC)
#define WAIT_USEC_HIGH	(WAIT_USEC_LOW + (WAIT_USEC_LOW >> 3))

#define irq_to_gpio(irq) ((irq) - gpio_to_irq(0))

/* LDC2114 registers */
#define LDC2114_STATUS			0x00
#define LDC2114_OUT			0x01
#define LDC2114_DATA0_LSB 		0x02
#define LDC2114_DATA0_MSB		0x03
#define LDC2114_DATA1_LSB		0x04
#define LDC2114_DATA1_MSB		0x05
#define LDC2114_DATA2_LSB		0x06
#define LDC2114_DATA2_MSB		0x07
#define LDC2114_DATA3_LSB		0x08
#define LDC2114_DATA3_MSB		0x09
#define LDC2114_RESET			0x0A
#define LDC2114_EN			0x0C
#define LDC2114_NP_SCAN_RATE		0x0D
#define LDC2114_GAIN0			0x0E
#define LDC2114_LP_SCAN_RATE		0x0F
#define LDC2114_GAIN1			0x10
#define LDC2114_INTPOL			0x11
#define LDC2114_GAIN2			0x12
#define LDC2114_LP_BASE_INC		0x13
#define LDC2114_GAIN3			0x14
#define LDC2114_NP_BASE_INC		0x15
#define LDC2114_MAXWIN			0x16
#define LDC2114_LC_DIVIDER		0x17
#define LDC2114_HYST			0x18
#define LDC2114_TWIST			0x19
#define LDC2114_COMMON_DEFORM		0x1A
#define LDC2114_OPOL			0x1C
#define LDC2114_CNTSC			0x1E
#define LDC2114_SENSOR0_CONFIG		0x20
#define LDC2114_SENSOR1_CONFIG		0x22
#define LDC2114_SENSOR2_CONFIG		0x24
#define LDC2114_FTF0			0x25
#define LDC2114_SENSOR3_CONFIG		0x26
#define LDC2114_FTF1_2			0x28
#define LDC2114_FTF3			0x2B
#define LDC2114_MANUFACTURER_ID_LSB	0xFC
#define LDC2114_MANUFACTURER_ID_MSB	0xFD
#define LDC2114_DEVICE_ID_LSB		0xFE
#define LDC2114_DEVICE_ID_MSB 		0xFF

static int drv_instance_counter;

static const struct reg_field ldc2114_reg_fields[] = {
	[F_STATUS_TIMEOUT]          = REG_FIELD(LDC2114_STATUS,         1, 1),
	[F_STATUS_LC_WD]            = REG_FIELD(LDC2114_STATUS,         2, 2),
	[F_STATUS_FSM_WD]           = REG_FIELD(LDC2114_STATUS,         3, 3),
	[F_STATUS_MAXOUT]           = REG_FIELD(LDC2114_STATUS,         4, 4),
	[F_STATUS_RDY_TO_WRITE]     = REG_FIELD(LDC2114_STATUS,         5, 5),
	[F_STATUS_CHIP_READY]       = REG_FIELD(LDC2114_STATUS,         6, 6),
	[F_STATUS_OUT_STATUS]       = REG_FIELD(LDC2114_STATUS,         7, 7),
	[F_OUT_OUT0]                = REG_FIELD(LDC2114_OUT,            0, 0),
	[F_OUT_OUT1]                = REG_FIELD(LDC2114_OUT,            1, 1),
	[F_OUT_OUT2]                = REG_FIELD(LDC2114_OUT,            2, 2),
	[F_OUT_OUT3]                = REG_FIELD(LDC2114_OUT,            3, 3),
	[F_RESET_STATE_RESET]       = REG_FIELD(LDC2114_RESET,          0, 0),
	[F_RESET_FULL_RESET]        = REG_FIELD(LDC2114_RESET,          4, 4),
	[F_EN_EN0]                  = REG_FIELD(LDC2114_EN,             0, 0),
	[F_EN_EN1]                  = REG_FIELD(LDC2114_EN,             1, 1),
	[F_EN_EN2]                  = REG_FIELD(LDC2114_EN,             2, 2),
	[F_EN_EN3]                  = REG_FIELD(LDC2114_EN,             3, 3),
	[F_EN_LPEN0]                = REG_FIELD(LDC2114_EN,             4, 4),
	[F_EN_LPEN1]                = REG_FIELD(LDC2114_EN,             5, 5),
	[F_EN_LPEN2]                = REG_FIELD(LDC2114_EN,             6, 6),
	[F_EN_LPEN3]                = REG_FIELD(LDC2114_EN,             7, 7),
	[F_NP_SCAN_RATE_NPSR]       = REG_FIELD(LDC2114_NP_SCAN_RATE,   0, 1),
	[F_GAIN0_GAIN0]             = REG_FIELD(LDC2114_GAIN0,          0, 5),
	[F_LP_SCAN_RATE_LPSR]       = REG_FIELD(LDC2114_LP_SCAN_RATE,   0, 1),
	[F_GAIN1_GAIN1]             = REG_FIELD(LDC2114_GAIN1,          0, 5),
	[F_INTPOL_INTPOL]           = REG_FIELD(LDC2114_INTPOL,         2, 2),
	[F_GAIN2_GAIN2]             = REG_FIELD(LDC2114_GAIN2,          0, 5),
	[F_LP_BASE_INC_LPBI]        = REG_FIELD(LDC2114_LP_BASE_INC,    0, 2),
	[F_GAIN3_GAIN3]             = REG_FIELD(LDC2114_GAIN3,          0, 5),
	[F_NP_BASE_INC_NPBI]        = REG_FIELD(LDC2114_NP_BASE_INC,    0, 2),
	[F_MAXWIN_MAXWIN0]          = REG_FIELD(LDC2114_MAXWIN,         0, 0),
	[F_MAXWIN_MAXWIN1]          = REG_FIELD(LDC2114_MAXWIN,         1, 1),
	[F_MAXWIN_MAXWIN2]          = REG_FIELD(LDC2114_MAXWIN,         2, 2),
	[F_MAXWIN_MAXWIN3]          = REG_FIELD(LDC2114_MAXWIN,         3, 3),
	[F_LC_DIVIDER_LCDIV]        = REG_FIELD(LDC2114_LC_DIVIDER,     0, 2),
	[F_HYST_HYST]               = REG_FIELD(LDC2114_HYST,           0, 3),
	[F_TWIST_ANTITWST]          = REG_FIELD(LDC2114_TWIST,          0, 2),
	[F_COMMON_DEFORM_ANTIDFRM0] = REG_FIELD(LDC2114_COMMON_DEFORM,  0, 0),
	[F_COMMON_DEFORM_ANTIDFRM1] = REG_FIELD(LDC2114_COMMON_DEFORM,  1, 1),
	[F_COMMON_DEFORM_ANTIDFRM2] = REG_FIELD(LDC2114_COMMON_DEFORM,  2, 2),
	[F_COMMON_DEFORM_ANTIDFRM3] = REG_FIELD(LDC2114_COMMON_DEFORM,  3, 3),
	[F_COMMON_DEFORM_ANTICM0]   = REG_FIELD(LDC2114_COMMON_DEFORM,  4, 4),
	[F_COMMON_DEFORM_ANTICM1]   = REG_FIELD(LDC2114_COMMON_DEFORM,  5, 5),
	[F_COMMON_DEFORM_ANTICM2]   = REG_FIELD(LDC2114_COMMON_DEFORM,  6, 6),
	[F_COMMON_DEFORM_ANTICM3]   = REG_FIELD(LDC2114_COMMON_DEFORM,  7, 7),
	[F_OPOL_OPOL0]              = REG_FIELD(LDC2114_OPOL,           4, 4),
	[F_OPOL_OPOL1]              = REG_FIELD(LDC2114_OPOL,           5, 5),
	[F_OPOL_OPOL2]              = REG_FIELD(LDC2114_OPOL,           6, 6),
	[F_OPOL_OPOL3]              = REG_FIELD(LDC2114_OPOL,           7, 7),
	[F_CNTSC_CNTSC0]            = REG_FIELD(LDC2114_CNTSC,          0, 1),
	[F_CNTSC_CNTSC1]            = REG_FIELD(LDC2114_CNTSC,          2, 3),
	[F_CNTSC_CNTSC2]            = REG_FIELD(LDC2114_CNTSC,          4, 5),
	[F_CNTSC_CNTSC3]            = REG_FIELD(LDC2114_CNTSC,          6, 7),
	[F_SENSOR0_CONFIG_SENCYC0]  = REG_FIELD(LDC2114_SENSOR0_CONFIG, 0, 4),
	[F_SENSOR0_CONFIG_FREQ0]    = REG_FIELD(LDC2114_SENSOR0_CONFIG, 5, 6),
	[F_SENSOR0_CONFIG_RP0]      = REG_FIELD(LDC2114_SENSOR0_CONFIG, 7, 7),
	[F_SENSOR1_CONFIG_SENCYC1]  = REG_FIELD(LDC2114_SENSOR1_CONFIG, 0, 4),
	[F_SENSOR1_CONFIG_FREQ1]    = REG_FIELD(LDC2114_SENSOR1_CONFIG, 5, 6),
	[F_SENSOR1_CONFIG_RP1]      = REG_FIELD(LDC2114_SENSOR1_CONFIG, 7, 7),
	[F_SENSOR2_CONFIG_SENCYC2]  = REG_FIELD(LDC2114_SENSOR2_CONFIG, 0, 4),
	[F_SENSOR2_CONFIG_FREQ2]    = REG_FIELD(LDC2114_SENSOR2_CONFIG, 5, 6),
	[F_SENSOR2_CONFIG_RP2]      = REG_FIELD(LDC2114_SENSOR2_CONFIG, 7, 7),
	[F_FTF0_FTF0]               = REG_FIELD(LDC2114_FTF0,           1, 2),
	[F_SENSOR3_CONFIG_SENCYC3]  = REG_FIELD(LDC2114_SENSOR3_CONFIG, 0, 4),
	[F_SENSOR3_CONFIG_FREQ3]    = REG_FIELD(LDC2114_SENSOR3_CONFIG, 5, 6),
	[F_SENSOR3_CONFIG_RP3]      = REG_FIELD(LDC2114_SENSOR3_CONFIG, 7, 7),
	[F_FTF1_2_FTF1]             = REG_FIELD(LDC2114_FTF1_2,         4, 5),
	[F_FTF1_2_FTF2]             = REG_FIELD(LDC2114_FTF1_2,         6, 7),
	[F_FTF3_FTF3]               = REG_FIELD(LDC2114_FTF3,           0, 1),
};

struct ldc2114_attr {
	struct device_attribute dev_attr;
	unsigned int field;
};

#define to_ldc2114_attr(_dev_attr) \
	container_of(_dev_attr, struct ldc2114_attr, dev_attr)

#define LDC2114_ATTR(_name, _field) \
	struct ldc2114_attr ldc2114_attr_##_name = { \
		.dev_attr = __ATTR(_name, (S_IRUGO | S_IWUSR), \
				   ldc2114_show_reg, \
				   ldc2114_store_reg), \
		.field = _field, \
	}

static ssize_t ldc2114_store_reg(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ldc2114_data *ldc = i2c_get_clientdata(client);
	struct ldc2114_attr *ldc2114_attr = to_ldc2114_attr(attr);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_field_write(ldc->fields[ldc2114_attr->field], val);
	if (ret)
		return ret;

	return count;
}

static ssize_t ldc2114_show_reg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ldc2114_data *ldc = i2c_get_clientdata(client);
	struct ldc2114_attr *ldc2114_attr = to_ldc2114_attr(attr);
	unsigned int val;
	int ret;

	ret = regmap_field_read(ldc->fields[ldc2114_attr->field], &val);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static LDC2114_ATTR(timout, F_STATUS_TIMEOUT);
static LDC2114_ATTR(lc_wd, F_STATUS_LC_WD);
static LDC2114_ATTR(fsm_wd, F_STATUS_FSM_WD);
static LDC2114_ATTR(maxout, F_STATUS_MAXOUT);
static LDC2114_ATTR(rdy_to_write, F_STATUS_RDY_TO_WRITE);
static LDC2114_ATTR(chip_ready, F_STATUS_CHIP_READY);
static LDC2114_ATTR(out_status, F_STATUS_OUT_STATUS);
static LDC2114_ATTR(out0, F_OUT_OUT0);
static LDC2114_ATTR(out1, F_OUT_OUT1);
static LDC2114_ATTR(out2, F_OUT_OUT2);
static LDC2114_ATTR(out3, F_OUT_OUT3);
static LDC2114_ATTR(state_reset, F_RESET_STATE_RESET);
static LDC2114_ATTR(full_reset, F_RESET_FULL_RESET);
static LDC2114_ATTR(en0, F_EN_EN0);
static LDC2114_ATTR(en1, F_EN_EN1);
static LDC2114_ATTR(en2, F_EN_EN2);
static LDC2114_ATTR(en3, F_EN_EN3);
static LDC2114_ATTR(lpen0, F_EN_LPEN0);
static LDC2114_ATTR(lpen1, F_EN_LPEN1);
static LDC2114_ATTR(lpen2, F_EN_LPEN2);
static LDC2114_ATTR(lpen3, F_EN_LPEN3);
static LDC2114_ATTR(npsr, F_NP_SCAN_RATE_NPSR);
static LDC2114_ATTR(lpsr, F_LP_SCAN_RATE_LPSR);
static LDC2114_ATTR(gain0, F_GAIN0_GAIN0);
static LDC2114_ATTR(gain1, F_GAIN1_GAIN1);
static LDC2114_ATTR(gain2, F_GAIN2_GAIN2);
static LDC2114_ATTR(gain3, F_GAIN3_GAIN3);
static LDC2114_ATTR(intpol, F_INTPOL_INTPOL);
static LDC2114_ATTR(lpbi, F_LP_BASE_INC_LPBI);
static LDC2114_ATTR(npbi, F_NP_BASE_INC_NPBI);
static LDC2114_ATTR(maxwin0, F_MAXWIN_MAXWIN0);
static LDC2114_ATTR(maxwin1, F_MAXWIN_MAXWIN1);
static LDC2114_ATTR(maxwin2, F_MAXWIN_MAXWIN2);
static LDC2114_ATTR(maxwin3, F_MAXWIN_MAXWIN3);
static LDC2114_ATTR(lcdiv, F_LC_DIVIDER_LCDIV);
static LDC2114_ATTR(hyst, F_HYST_HYST);
static LDC2114_ATTR(antitwst, F_TWIST_ANTITWST);
static LDC2114_ATTR(antidfrm0, F_COMMON_DEFORM_ANTIDFRM0);
static LDC2114_ATTR(antidfrm1, F_COMMON_DEFORM_ANTIDFRM1);
static LDC2114_ATTR(antidfrm2, F_COMMON_DEFORM_ANTIDFRM2);
static LDC2114_ATTR(antidfrm3, F_COMMON_DEFORM_ANTIDFRM3);
static LDC2114_ATTR(anticm0, F_COMMON_DEFORM_ANTICM0);
static LDC2114_ATTR(anticm1, F_COMMON_DEFORM_ANTICM1);
static LDC2114_ATTR(anticm2, F_COMMON_DEFORM_ANTICM2);
static LDC2114_ATTR(anticm3, F_COMMON_DEFORM_ANTICM3);
static LDC2114_ATTR(opol0, F_OPOL_OPOL0);
static LDC2114_ATTR(opol1, F_OPOL_OPOL1);
static LDC2114_ATTR(opol2, F_OPOL_OPOL2);
static LDC2114_ATTR(opol3, F_OPOL_OPOL3);
static LDC2114_ATTR(cntsc0, F_CNTSC_CNTSC0);
static LDC2114_ATTR(cntsc1, F_CNTSC_CNTSC1);
static LDC2114_ATTR(cntsc2, F_CNTSC_CNTSC2);
static LDC2114_ATTR(cntsc3, F_CNTSC_CNTSC3);
static LDC2114_ATTR(sencyc0, F_SENSOR0_CONFIG_SENCYC0);
static LDC2114_ATTR(freq0, F_SENSOR0_CONFIG_FREQ0);
static LDC2114_ATTR(rp0, F_SENSOR0_CONFIG_RP0);
static LDC2114_ATTR(sencyc1, F_SENSOR1_CONFIG_SENCYC1);
static LDC2114_ATTR(freq1, F_SENSOR1_CONFIG_FREQ1);
static LDC2114_ATTR(rp1, F_SENSOR1_CONFIG_RP1);
static LDC2114_ATTR(sencyc2, F_SENSOR2_CONFIG_SENCYC2);
static LDC2114_ATTR(freq2, F_SENSOR2_CONFIG_FREQ2);
static LDC2114_ATTR(rp2, F_SENSOR2_CONFIG_RP2);
static LDC2114_ATTR(sencyc3, F_SENSOR3_CONFIG_SENCYC3);
static LDC2114_ATTR(freq3, F_SENSOR3_CONFIG_FREQ3);
static LDC2114_ATTR(rp3, F_SENSOR3_CONFIG_RP3);
static LDC2114_ATTR(ftf0, F_FTF0_FTF0);
static LDC2114_ATTR(ftf1, F_FTF1_2_FTF1);
static LDC2114_ATTR(ftf2, F_FTF1_2_FTF2);
static LDC2114_ATTR(ftf3, F_FTF3_FTF3);

static ssize_t ldc2114_verno_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ldc2114_data *ldc = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", ldc->verno);
}

static struct ldc2114_attr ldc2114_attr_verno = {
	.dev_attr = __ATTR(verno, S_IRUGO, ldc2114_verno_show, NULL),
	.field = 0,
};

static struct attribute *ldc2114_attributes[] = {
	&ldc2114_attr_timout.dev_attr.attr,
	&ldc2114_attr_lc_wd.dev_attr.attr,
	&ldc2114_attr_fsm_wd.dev_attr.attr,
	&ldc2114_attr_maxout.dev_attr.attr,
	&ldc2114_attr_rdy_to_write.dev_attr.attr,
	&ldc2114_attr_chip_ready.dev_attr.attr,
	&ldc2114_attr_out_status.dev_attr.attr,
	&ldc2114_attr_out0.dev_attr.attr,
	&ldc2114_attr_out1.dev_attr.attr,
	&ldc2114_attr_out2.dev_attr.attr,
	&ldc2114_attr_out3.dev_attr.attr,
	&ldc2114_attr_state_reset.dev_attr.attr,
	&ldc2114_attr_full_reset.dev_attr.attr,
	&ldc2114_attr_en0.dev_attr.attr,
	&ldc2114_attr_en1.dev_attr.attr,
	&ldc2114_attr_en2.dev_attr.attr,
	&ldc2114_attr_en3.dev_attr.attr,
	&ldc2114_attr_lpen0.dev_attr.attr,
	&ldc2114_attr_lpen1.dev_attr.attr,
	&ldc2114_attr_lpen2.dev_attr.attr,
	&ldc2114_attr_lpen3.dev_attr.attr,
	&ldc2114_attr_npsr.dev_attr.attr,
	&ldc2114_attr_lpsr.dev_attr.attr,
	&ldc2114_attr_gain0.dev_attr.attr,
	&ldc2114_attr_gain1.dev_attr.attr,
	&ldc2114_attr_gain2.dev_attr.attr,
	&ldc2114_attr_gain3.dev_attr.attr,
	&ldc2114_attr_intpol.dev_attr.attr,
	&ldc2114_attr_lpbi.dev_attr.attr,
	&ldc2114_attr_npbi.dev_attr.attr,
	&ldc2114_attr_maxwin0.dev_attr.attr,
	&ldc2114_attr_maxwin1.dev_attr.attr,
	&ldc2114_attr_maxwin2.dev_attr.attr,
	&ldc2114_attr_maxwin3.dev_attr.attr,
	&ldc2114_attr_lcdiv.dev_attr.attr,
	&ldc2114_attr_hyst.dev_attr.attr,
	&ldc2114_attr_antitwst.dev_attr.attr,
	&ldc2114_attr_antidfrm0.dev_attr.attr,
	&ldc2114_attr_antidfrm1.dev_attr.attr,
	&ldc2114_attr_antidfrm2.dev_attr.attr,
	&ldc2114_attr_antidfrm3.dev_attr.attr,
	&ldc2114_attr_anticm0.dev_attr.attr,
	&ldc2114_attr_anticm1.dev_attr.attr,
	&ldc2114_attr_anticm2.dev_attr.attr,
	&ldc2114_attr_anticm3.dev_attr.attr,
	&ldc2114_attr_opol0.dev_attr.attr,
	&ldc2114_attr_opol1.dev_attr.attr,
	&ldc2114_attr_opol2.dev_attr.attr,
	&ldc2114_attr_opol3.dev_attr.attr,
	&ldc2114_attr_cntsc0.dev_attr.attr,
	&ldc2114_attr_cntsc1.dev_attr.attr,
	&ldc2114_attr_cntsc2.dev_attr.attr,
	&ldc2114_attr_cntsc3.dev_attr.attr,
	&ldc2114_attr_sencyc0.dev_attr.attr,
	&ldc2114_attr_freq0.dev_attr.attr,
	&ldc2114_attr_rp0.dev_attr.attr,
	&ldc2114_attr_sencyc1.dev_attr.attr,
	&ldc2114_attr_freq1.dev_attr.attr,
	&ldc2114_attr_rp1.dev_attr.attr,
	&ldc2114_attr_sencyc2.dev_attr.attr,
	&ldc2114_attr_freq2.dev_attr.attr,
	&ldc2114_attr_rp2.dev_attr.attr,
	&ldc2114_attr_sencyc3.dev_attr.attr,
	&ldc2114_attr_freq3.dev_attr.attr,
	&ldc2114_attr_rp3.dev_attr.attr,
	&ldc2114_attr_ftf0.dev_attr.attr,
	&ldc2114_attr_ftf1.dev_attr.attr,
	&ldc2114_attr_ftf2.dev_attr.attr,
	&ldc2114_attr_ftf3.dev_attr.attr,
	&ldc2114_attr_verno.dev_attr.attr,
	NULL
};

static const struct attribute_group ldc2114_attr_group = {
	.attrs = ldc2114_attributes,
};

static int ldc2114_i2c_write_regmap(void *context,
				const void *data, size_t count)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg xfer[2];
	uint8_t buffer[2];
	int ret;

	buffer[0] = LDC2114_RESET;
	buffer[1] = BIT(0);
	ret = i2c_master_send(client, buffer, 2);
	if (ret < 0)
		return ret;
	else if (ret != 2)
		return -EIO;

	do {
		xfer[0].addr = client->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		buffer[0] = LDC2114_STATUS;
		xfer[0].buf = (void *)&buffer[0];

		xfer[1].addr = client->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = 1;
		xfer[1].buf = (void *)&buffer[1];

		ret = i2c_transfer(client->adapter, xfer, 2);
		if (ret < 0)
			return ret;
		else if (ret != 2)
			return -EIO;
	} while(!(buffer[1] & BIT(5)));

	ret = i2c_master_send(client, data, count);
	if (ret < 0)
		return ret;
	else if (ret != count)
		return -EIO;

	buffer[0] = LDC2114_RESET;
	buffer[1] = 0;
	ret = i2c_master_send(client, buffer, 2);
	if (ret < 0)
		return ret;
	else if (ret != 2)
		return -EIO;

	return 0;
}

#define I2C_TRANSFER(adapter, msgs, count) \
			ret = i2c_transfer(adapter, msgs, count); \
			if (ret == count) \
				ret = 0

static int ldc2114_i2c_read_regmap(void *context,
				const void *reg, size_t reg_size,
				void *val, size_t val_size)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct i2c_msg xfer[2];
	int ret;

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = reg_size;
	xfer[0].buf = (void *)reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = val_size;
	xfer[1].buf = val;

	I2C_TRANSFER(i2c->adapter, xfer, ARRAY_SIZE(xfer));

	return ret;
}

static int ldc2114_write_reg8(void *context, uint8_t regaddr, uint8_t value)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
	uint8_t wbuf[] = { regaddr, value };
	struct i2c_msg msgs[1];
	int ret;

	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(wbuf);
	msgs[0].buf = wbuf;

	I2C_TRANSFER(i2c->adapter, msgs, ARRAY_SIZE(msgs));

	return ret;
}

static int ldc2114_read_reg8(void *context, uint8_t regaddr, uint8_t *value)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
    struct i2c_msg msgs[2];
	int ret;

	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(regaddr);
	msgs[0].buf = &regaddr;

	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(*value);
	msgs[1].buf = (uint8_t *)value;

	I2C_TRANSFER(i2c->adapter, msgs, ARRAY_SIZE(msgs));

    return ret;
}

static int ldc2114_read_bulk(void *context,
				uint8_t regaddr, void *data, size_t count)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(regaddr);
	msgs[0].buf = &regaddr;

	msgs[1].addr = i2c->addr;
	msgs[1].flags  = I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (uint8_t *)data;

	I2C_TRANSFER(i2c->adapter, msgs, ARRAY_SIZE(msgs));

	return ret;
}

static int inline ldc2114_write_bulk(void *context,
				uint8_t regaddr, void *data, size_t count)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);

	return i2c_master_send(i2c, data, count);
}

static int ldc2114_reset(void *context, uint8_t reset_val, uint8_t status_bit)
{
	struct device *dev = context;
	int rc, loops = 0;
	uint8_t status;

	rc = ldc2114_write_reg8(context, LDC2114_RESET, reset_val);
	if (rc)
		dev_err(dev, "error writing reset command 0x%02x\n", reset_val);
	while (1) {
		loops++;
		rc = ldc2114_read_reg8(context, LDC2114_STATUS, &status);
		if (rc)
			dev_err(dev, "error reading status\n");
		else
			dev_dbg(dev, "status=0x%02x\n", status);

		if (status & status_bit) {
			dev_info(dev, "reset 0x%02x complete: loops=%d\n", reset_val, loops);
			break;
		}

		if (loops > CONFIG_LDC2114_MAX_FAILURES) {
			dev_err(dev, "reset 0x%02x failed\n", reset_val);
			return -EIO;
		}
		usleep_range(WAIT_USEC_50, WAIT_USEC_60);
	}

	return 0;
}

/*
 * We cannot use regmap-i2c generic bus implementation here as the LDC2114
 * has a special handshake process to write registers.
 */
static struct regmap_bus regmap_ldc2114_bus = {
	.write = ldc2114_i2c_write_regmap,
	.read = ldc2114_i2c_read_regmap,
};

static const struct regmap_range ldc2114_yes_ranges[] = {
	regmap_reg_range(LDC2114_STATUS, LDC2114_DATA3_MSB),
};

static const struct regmap_access_table ldc2114_volatile_table = {
	.yes_ranges = ldc2114_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(ldc2114_yes_ranges),
};

static const struct regmap_config ldc2114_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = LDC2114_DEVICE_ID_MSB,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &ldc2114_volatile_table,
};

static int ldc2114_input_device(struct ldc2114_data *ldc)
{
	int i, ret;

	ldc->input = devm_input_allocate_device(ldc->dev);
	if (!ldc->input) {
		dev_err(ldc->dev, "unable to allocate input device\n");
		return -ENOMEM;
	}

	set_bit(EV_KEY, ldc->input->evbit);
	for (i = 0; i < MAX_KEYS; i++) {
		set_bit(ldc->button_map[i], ldc->input->keybit);
		input_set_capability(ldc->input, EV_KEY, ldc->button_map[i]);
	}

	ldc->input->name = LDC2114_DRIVER_NAME "_keys";
	ldc->input->phys = LDC2114_DRIVER_NAME "_keys/input0";

	ret = input_register_device(ldc->input);
	if(ret < 0)
		dev_err(ldc->dev, "error registering input %s\n", ldc->input->name);

	return ret;
}

#if defined(LDC2114_POLL_DEBUG) || defined(LDC2114_CFG_DEBUG)
static void ldc2114_toggle_gpio(struct ldc2114_data *ldc,
						int gpio)
{
	int value = gpio_get_value(gpio);
	dev_dbg(ldc->dev, "toggle gpio %d from state %d\n", gpio, value);
	gpio_set_value(gpio, !value);
#else
static void inline ldc2114_toggle_gpio(struct ldc2114_data *ldc,
						int gpio)
{
	gpio_set_value(gpio, !gpio_get_value(gpio));
#endif
}

static void ldc2114_toggle(struct ldc2114_data *ldc,
						int gpio, int times)
{
	int i = 0;

	if (!times || times < 0)
		times = 1;

	while(1) {
		ldc2114_toggle_gpio(ldc, gpio);
		if (i++ == times)
			break;
		usleep_range(WAIT_USEC_LOW, WAIT_USEC_HIGH);
	}
}

static int ldc2114_poll(struct ldc2114_data *ldc, uint8_t *out)
{
	int ret;
	struct ldc2114_raw_ext de;
	static unsigned long executed;

	if (ldc->failures >= CONFIG_LDC2114_MAX_FAILURES)
		return -EFAULT;

	ret = ldc2114_read_bulk(ldc->dev, LDC2114_STATUS, &de, sizeof(de));
	if (out)
		*out = de.data.out;
	if (ret) {
		ldc->failures++;

		if (ldc->failures == CONFIG_LDC2114_MAX_FAILURES) {
			dev_err(ldc->dev, "Max failures, disabling polling\n");
			return -EIO;
		}
	} else {
		ldc->failures = 0;
		ret = ldc2114_buffer(ldc, LDC2114_EV_DATA, de);
		if (ret)
			dev_err(ldc->dev, "buffer is not ready\n");
	}

	if (!(executed++%10))
		dev_info(ldc->dev, "polled %lu\n", executed);

	return 0;
}

#ifdef LDC2114_POLL_DEBUG
#define NANO_SEC	1000000000
#define SEC_TO_MSEC	1000
#define NANO_TO_MSEC	1000000

static inline unsigned long long timediff_ms(struct timespec start,
						struct timespec end)
{
	struct timespec temp;

	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = NANO_SEC + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return (temp.tv_sec * SEC_TO_MSEC) + (temp.tv_nsec / NANO_TO_MSEC);
}
#endif

static void ldc2114_polling_work(struct work_struct *work)
{
	struct delayed_work *dw =
		container_of(work, struct delayed_work, work);
	struct ldc2114_data *ldc =
		container_of(dw, struct ldc2114_data, polling_work);
	int ret;

	ret = ldc2114_poll(ldc, NULL);

	if (atomic_read(&ldc->poll_work_running))
		schedule_delayed_work(&ldc->polling_work,
					msecs_to_jiffies(ldc->poll_interval));
}

#ifdef LDC2114_POLL_DEBUG
static ktime_t start_polling_tm;
static int polling_counter;
#endif

static void ldc2114_input_report(struct ldc2114_data *ldc,
		int output_bits)
{
	int s, i, status;

	for (i = 0, s = 0; i < MAX_KEYS; i++) {
		status = (output_bits >> i) & 1;
		if (status == ldc->buttons[i])
			continue;
		ldc->buttons[i] = status;
		if (ldc->btn_enabled) {
			input_report_key(ldc->input, ldc->button_map[i], status);
			s++;
			dev_dbg(ldc->dev, "sent key code = %d(%d)\n",
					ldc->button_map[i], status);
		}
	}

	if (s) { /* only send sync if there were keys reported */
		input_sync(ldc->input);
		dev_dbg(ldc->dev, "sent SYNC\n");
	}

}

static void ldc2114_irq_work(struct work_struct *work)
{
	struct delayed_work *dw =
		container_of(work, struct delayed_work, work);
	struct ldc2114_data *ldc =
		container_of(dw, struct ldc2114_data, irq_work);
	struct ldc2114_raw data;
	uint8_t status_reg;

	while (1) {
		int ret;

		down(&ldc->semaphore);

		ret = ldc2114_read_reg8(ldc->dev, LDC2114_STATUS, &status_reg);
		if (status_reg & ~(LDC2114_REG_STATUS_CHIP_READY))
			dev_info(ldc->dev, "STATUS bits 0x%x\n", status_reg);

		if (status_reg & LDC2114_REG_STATUS_OUT) {
			ret = ldc2114_read_bulk(ldc->dev,
									LDC2114_OUT, &data, sizeof(data));
			if (data.out != ldc->output_bits) {
				ldc->output_bits = data.out;
				dev_dbg(ldc->dev, "OUTPUT bits 0x%x\n", ldc->output_bits);
			}

			ldc2114_input_report(ldc, ldc->output_bits);
		}

		if (atomic_read(&ldc->irq_work_running)) {
#ifdef LDC2114_POLL_DEBUG
			polling_counter++;
#endif
			usleep_range(WAIT_USEC_LOW, WAIT_USEC_HIGH);
			up(&ldc->semaphore);
		} else {
#ifdef LDC2114_POLL_DEBUG
			dev_info(ldc->dev, "%s: polled %d times within %llums\n",
					__func__, polling_counter,
					timediff_ms(ktime_to_timespec(start_polling_tm),
								ktime_to_timespec(ktime_get())));
			polling_counter = 0;
#endif
			/* TODO: make sure we never miss release :) */
			/* Work around missed release */
			if (ldc->btn_enabled)
				ldc2114_input_report(ldc, 0);

			memset(ldc->buttons, 0, sizeof(ldc->buttons));
		}
	}
}

static irqreturn_t ldc2114_irq(int irq, void *data)
{
	struct ldc2114_data *ldc = data;
	bool stop_irq_work = atomic_read(&ldc->irq_work_running) == 1;
	uint8_t status;
	int ret;

	ret = ldc2114_read_reg8(ldc->dev, LDC2114_STATUS, &status);
	dev_dbg(ldc->dev, "IRQ triggered; status 0x%x\n", status);

	/* check OUT ready bit */
	if (status & LDC2114_REG_STATUS_OUT) {
		dev_dbg(ldc->dev, "starting work...\n");
		atomic_set(&ldc->irq_work_running, 1);
		up(&ldc->semaphore);
#ifdef LDC2114_POLL_DEBUG
		start_polling_tm = ktime_get();
#endif
	} else if (stop_irq_work) {
		dev_dbg(ldc->dev, "stopping work...\n");
		atomic_set(&ldc->irq_work_running, 0);
	} else
		return IRQ_HANDLED;

	if (gpio_is_valid(ldc->signal_gpio))
		ldc2114_toggle_gpio(ldc, ldc->signal_gpio);

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id ldc2114_of_match[] = {
	{ .compatible = "ti,ldc2114", },
	{},
};

MODULE_DEVICE_TABLE(of, ldc2114_of_match);

static struct ldc2114_config *read_config_data(struct ldc2114_data *ldc,
				struct device_node *parent, const char *suffix)
{
	struct ldc2114_config *config = NULL;
	struct device_node *node;
	u32 length = 0;
	const char *data;
	char node_name[64];
	int npairs, i;

	scnprintf(node_name, 63, "config-%s", suffix);
	node = of_find_node_by_name(parent, node_name);
	if (!node)
		return NULL;

	data = of_get_property(node, "config-data", &length);
	if (!data) {
#ifdef LDC2114_CFG_DEBUG
		dev_info(ldc->dev, "(config-%s) prop config-data not found\n", suffix);
#endif
		goto out;
	}

	npairs = length / 2;
#ifdef LDC2114_CFG_DEBUG
	dev_info(ldc->dev, "(config-%s) array size %d\n", suffix, npairs);
#endif
	config = devm_kzalloc(ldc->dev, sizeof(struct ldc2114_config) +
				sizeof(struct ldc2114_cfg_data) * npairs, GFP_KERNEL);
	if (!config)
		goto out;

	for (i = 0; i < npairs; i++) {
		config->data[i].address = (uint8_t)*data++;
		config->data[i].value = (uint8_t)*data++;
#ifdef LDC2114_CFG_DEBUG
		dev_info(ldc->dev, "[%d] addr=0x%02x, val=0x%02x\n",
				i, config->data[i].address, config->data[i].value);
#endif
	}

	config->size = npairs;

out:
	of_node_put(node);

	return config;
}

static int ldc2114_of_init(struct i2c_client *client)
{
	int ret;
	struct ldc2114_data *ldc = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *config_np;

	ret = of_get_gpio(np, 0);
	if (ret < 0) {
		dev_err(ldc->dev, "failed to get lwpm-gpio: %d\n", ret);
		return ret;
	}

	ldc->lpwm_gpio = ret;
	ret = devm_gpio_request(ldc->dev,
					ldc->lpwm_gpio, LDC2114_DRIVER_NAME "_lpwm");
	if (ret) {
		dev_err(ldc->dev, "failed to request lpwm-gpio %d\n", ldc->lpwm_gpio);
		return ret;
	}

	ret = of_get_gpio(np, 1);
	if (ret < 0)
		ldc->signal_gpio = -EINVAL;
	else {
		ldc->signal_gpio = ret;
		dev_info(ldc->dev, "using gpio %d as a signal\n", ldc->signal_gpio);

		ret = devm_gpio_request(ldc->dev,
						ldc->signal_gpio, LDC2114_DRIVER_NAME "_signal");
		if (ret) {
			dev_err(ldc->dev, "failed to request signal-gpio %d\n", ldc->signal_gpio);
			return ret;
		}

		/* start high */
		gpio_direction_output(ldc->signal_gpio, 1);
	}

	/* Init LWPM gpio into LOW, since this is default power off state */
	/* Then toggle its state as part of apply configuration procedure */
	gpio_direction_output(ldc->lpwm_gpio, 0);

	ret = of_irq_get(np, 0);
	if (ret < 0) {
		dev_err(ldc->dev, "failed to get irq: %d\n", ret);
		return ret;
	}

	ldc->irq = ret;
	ldc->intb_gpio = irq_to_gpio(ldc->irq);
	ret = devm_gpio_request(ldc->dev,
					ldc->intb_gpio, LDC2114_DRIVER_NAME "_irq");
	if (ret) {
		dev_err(ldc->dev, "failed to request gpio %d\n", ldc->intb_gpio);
		return ret;
	}

	ret = of_property_read_u32_array(np, "ldc2114,button-map",
						ldc->button_map, 4);
	if (!ret) {
		ldc->btn_enabled = true;
		dev_info(ldc->dev, "has keymap\n");
	}

	if (of_property_read_bool(np, "ldc2114,do-polling")) {
		ldc->data_polling = true;
		dev_info(ldc->dev,"supports data polling\n");
	}

	config_np = of_find_node_by_name(np, "configs");
	if (!config_np) {
		dev_info(ldc->dev, "does not support configs\n");
		return 0;
	}

	of_property_read_u32(config_np, "config-ver", &ldc->verno);
	dev_info(ldc->dev, "dt config Rev.%u\n", ldc->verno);

	ldc->config.normal = read_config_data(ldc, config_np, "normal");
	if (ldc->config.normal)
		dev_info(ldc->dev, "has normal config\n");

	ldc->config.lpm = read_config_data(ldc, config_np, "lpm");
	if (ldc->config.lpm)
		dev_info(ldc->dev, "has lpm config\n");

	of_node_put(config_np);

	return 0;
}
#else
static inline void ldc2114_of_init(struct i2c_client *client)
{
}
#endif

static void inline ldc2114_config_init(struct ldc2114_data *ldc,
				struct ldc2114_config *cfg, int delay_ms, bool resched)
{
	dev_dbg(ldc->dev, "config_init entered\n");

	ldc->config.active = cfg;
	ldc->config.delay_ms = delay_ms;
	ldc->config.attempts = 0;
	ldc->config.toggled = false;
	ldc->config.reschedulable = resched;
	reinit_completion(&ldc->config.done);
}

#define EXITIF_ERR(cmd) { \
			ret = cmd; \
			if (ret) { \
				dev_dbg(ldc->dev, "ret=%d; exiting...\n", ret); \
				return ret; \
			} \
		}

#define JUMPIF_ERR(cmd) { \
			ret = cmd; \
			if (ret) \
				goto err_check; \
		}

static void ldc2114_config_work(struct work_struct *work)
{
	struct delayed_work *dw =
		container_of(work, struct delayed_work, work);
	struct ldc2114_data *ldc =
		container_of(dw, struct ldc2114_data, config_work);
	bool partial_fail = false;
	bool is_ready = ldc->config.active && ldc->config.active->size;
	int i, ret;

	dev_dbg(ldc->dev, "config_work entered; is_ready=%d\n", is_ready);

	if (!ldc->not_connected &&
		ldc->config.reschedulable &&
		(gpio_get_value(ldc->intb_gpio) == 0))
		goto reschedule_me;

	if (!ldc->config.toggled) {
		/* now it's time to toggle LWPM pin, but only toggle it once */
		ldc2114_toggle_gpio(ldc, ldc->lpwm_gpio);
		ldc->config.toggled = true;
		dev_info(ldc->dev, "LWPM toggled\n");

		if (gpio_is_valid(ldc->signal_gpio))
			ldc2114_toggle(ldc, ldc->signal_gpio, 2);
	}

	if (!is_ready) {
		dev_dbg(ldc->dev, "nothing to apply\n");
		goto done;
	}

	JUMPIF_ERR(ldc2114_reset(ldc->dev, LDC2114_REG_RESET_FULL,
					LDC2114_REG_STATUS_CHIP_READY));
	JUMPIF_ERR(ldc2114_reset(ldc->dev, LDC2114_REG_RESET_CONFIG_MODE,
					LDC2114_REG_STATUS_RDY_TO_WRITE));

	for (i = 0; i < ldc->config.active->size; i++) {
		ret = ldc2114_write_reg8(ldc->dev,
					ldc->config.active->data[i].address,
					ldc->config.active->data[i].value);
		if (ret) {
			partial_fail = true;
			dev_err(ldc->dev, "config failed at i=%d\n", i);
			break;
		}
	}

	JUMPIF_ERR(ldc2114_reset(ldc->dev, LDC2114_REG_RESET_NONE,
					LDC2114_REG_STATUS_CHIP_READY));
err_check:

	if (ret || partial_fail) {
		/* In unlikely case when all LDC2114_MAX_RETRIES failed, */
		/* we just bail out and return an error. LPWM gpio will  */
		/* be toggled anyway. And then we either fail probe or   */
		/* leave IC in unknown state. Not much we can do here    */
		if (++ldc->config.attempts > LDC2114_MAX_RETRIES) {
			dev_err(ldc->dev, "bailed out on applying config\n");
			goto done;
		}
	} else {
		dev_info(ldc->dev, "config Rev.%u applied\n", ldc->verno);
		goto done;
	}

reschedule_me:
	queue_delayed_work(ldc->wq, &ldc->config_work,
			msecs_to_jiffies(ldc->config.delay_ms));
#ifdef LDC2114_CFG_DEBUG
	dev_info(ldc->dev, "re-scheduled\n");
#endif
	return;

done:
	complete(&ldc->config.done);
	ldc->config.active = NULL;
#ifdef LDC2114_CFG_DEBUG
	if (1) {
		uint8_t value;

		ldc2114_read_reg8(ldc->dev, LDC2114_STATUS, &value);
		dev_info(ldc->dev, "done: INTB %d, STATUS 0x%02x\n",
				gpio_get_value(ldc->intb_gpio), value);
	}
#endif

	if (gpio_is_valid(ldc->signal_gpio))
		ldc2114_toggle(ldc, ldc->signal_gpio, 2);
}

static int ldc2114_initialize(struct ldc2114_data *ldc)
{
	struct ldc2114_16bit version;
	uint8_t value;
	int ret;

error_workaround:

	EXITIF_ERR(ldc2114_read_reg8(ldc->dev, LDC2114_STATUS, &value));
	dev_info(ldc->dev, "STATUS 0x%02x\n", value);

	/* check connection issues */
	if (value & LDC2114_REG_STATUS_LC_WD) {
		ldc->not_connected = true;
		dev_info(ldc->dev, "sensor disconnected\n");
		if (ldc->btn_enabled) {
			ldc->btn_enabled = false;
			dev_info(ldc->dev, "disabling buttons\n");
		}
	} else {
		/* reading OUT helps with issues when INTB gets stuck due */
		/* to internal issues between LDC2114 and Silego parts */
		uint8_t out;

		ldc2114_poll(ldc, &out);
#ifdef LDC2114_CFG_DEBUG
		dev_info(ldc->dev, "OUT 0x%02x, INTB %d\n",
				out, gpio_get_value(ldc->intb_gpio));
#endif
		/* some errors require STATUS register to be read twice, */
		/* thus do STATUS read again here if errors detected */
		if (value & LDC2114_REG_STATUS_ERROR_OTHERS) {
			dev_info(ldc->dev, "error condition detected; retrying...\n");
			goto error_workaround;
		}
	}

	EXITIF_ERR(ldc2114_read_bulk(ldc->dev, LDC2114_DEVICE_ID_LSB,
				&version, sizeof(version)));
	dev_info(ldc->dev, "TI LDC%d 0x%x\n",
				2114 - version.lsb * 2,
				(version.lsb | (version.msb << 8)));

	EXITIF_ERR(ldc2114_read_reg8(ldc->dev, LDC2114_INTPOL, &value));
	ldc->intb_polarity = value & BIT(3) ? 1 : 0;

	dev_info(ldc->dev, "INTB active %d\n", ldc->intb_polarity);
	dev_info(ldc->dev, "IRQ %d (gpio%d)\n",
				ldc->irq, irq_to_gpio(ldc->irq));

	ldc2114_config_init(ldc, ldc->config.normal, LDC2114_LONG_DELAY, true);

	queue_delayed_work(ldc->wq, &ldc->config_work, 0);

	return 0;
}

static int ldc2114_poll_enable_cb(struct notifier_block *n,
				unsigned long val, void *data)
{
	struct ldc2114_data *ldc =
		container_of(n, struct ldc2114_data, poll_nb);
	int state = test_bit(0, &val) ? 1 : 0;

	dev_info(ldc->dev, "polling flag %d; state=%d\n",
			ldc->data_polling, state);

	if (!ldc->data_polling)
		return 0;

	atomic_set(&ldc->poll_work_running, state);

	if (state)
		schedule_work(&ldc->polling_work.work);

	return 0;
}

#include <linux/ratelimit.h>

static DEFINE_RATELIMIT_STATE(ldc2114_rl, HZ, 5);

static int ldc2114_reboot_cb(struct notifier_block *nb,
					unsigned long event, void *unused)
{
	struct ldc2114_data *ldc =
		container_of(nb, struct ldc2114_data, reboot_nb);
	static int runaway;

	if (runaway)
		goto out;

	runaway++;

	ldc2114_config_init(ldc, ldc->config.lpm, LDC2114_SHORT_DELAY, false);

	/* Stop polling work if it's running */
	if (ldc->data_polling)
		cancel_delayed_work_sync(&ldc->polling_work);

	if (ldc->irq_enabled) {
		/* wait until active touch is cleared */
		while (atomic_read(&ldc->irq_work_running) == 1) {

			if (!__ratelimit(&ldc2114_rl))
				dev_warn(ldc->dev, "irq still busy\n");

			usleep_range(WAIT_USEC_LOW, WAIT_USEC_HIGH);
		}

		disable_irq(ldc->irq);

		if (gpio_is_valid(ldc->signal_gpio))
			ldc2114_toggle(ldc, ldc->signal_gpio, 2);
	}

	ldc2114_config_work(&ldc->config_work.work);
	/* set low at the end */
	if (gpio_is_valid(ldc->signal_gpio))
		gpio_set_value(ldc->signal_gpio, 0);
out:
	return NOTIFY_DONE;
}

static int ldc2114_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ldc2114_data *ldc;
	struct pinctrl *pinctrl;
	int i, ret;

	if (!i2c_check_functionality(
			client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBus byte data not supported\n");
		return -EIO;
	}

	ldc = devm_kzalloc(&client->dev, sizeof(*ldc), GFP_KERNEL);
	if (!ldc)
		return -ENOMEM;

	i2c_set_clientdata(client, ldc);
	ldc->dev = &client->dev;
	ldc->irq = client->irq;
	ldc->data_polling = true;

	if (client->dev.of_node) {
		ret = ldc2114_of_init(client);
		if (ret < 0)
			return -EINVAL;
	}

	ldc->regmap = devm_regmap_init(ldc->dev, &regmap_ldc2114_bus,
				       ldc->dev, &ldc2114_regmap_config);
	if (IS_ERR(ldc->regmap)) {
		dev_err(ldc->dev, "Unable to allocate register map\n");
		return PTR_ERR(ldc->regmap);
	}

	for (i = 0; i < F_MAX_FIELDS; i++) {
		ldc->fields[i] = devm_regmap_field_alloc(ldc->dev,
								ldc->regmap, ldc2114_reg_fields[i]);
		if (IS_ERR(ldc->fields[i])) {
			dev_err(ldc->dev, "Unable to allocate regmap fields\n");
			return PTR_ERR(ldc->fields[i]);
		}
	}

	pinctrl = devm_pinctrl_get_select_default(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(ldc->dev, "pinctrl failed err: %ld\n", PTR_ERR(pinctrl));
		return PTR_ERR(pinctrl);
	}

	ldc->wq = create_singlethread_workqueue("ldc2114_workqueue");
	init_completion(&ldc->config.done);
	INIT_DELAYED_WORK(&ldc->config_work, ldc2114_config_work);

	ret = ldc2114_initialize(ldc);
	if (ret) {
		dev_err(ldc->dev, "Failed to init: %d\n", ret);
		return ret;
	}

	ldc->instance = ++drv_instance_counter;
	sema_init(&ldc->semaphore, 0);

	if (!ldc->not_connected) {
		/*
		 * Even though we setup edge triggered irq handler, genirq still
		 * checks for ONESHOT safety if no primary handler provided
		*/
		ret = devm_request_threaded_irq(ldc->dev, ldc->irq, NULL, ldc2114_irq,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					LDC2114_DRIVER_NAME, ldc);
		if (ret) {
			dev_err(ldc->dev, "Failed to register irq handler: %d\n", ret);
			return ret;
		}

		ldc->irq_enabled = true;
		INIT_DELAYED_WORK(&ldc->irq_work, ldc2114_irq_work);
		queue_delayed_work(ldc->wq, &ldc->irq_work,
					msecs_to_jiffies(LDC2114_SHORT_DELAY));
	}

	if (ldc->btn_enabled)
		ldc2114_input_device(ldc);

	ret = ldc2114_debugfs_init(ldc);
	if (ret < 0) {
		ldc->data_polling = false;
		if (ret != -ENODEV)
			dev_warn(ldc->dev, "Error registering debugfs: %d\n", ret);
	} else {
		ldc->poll_interval = 250;
		INIT_DELAYED_WORK(&ldc->polling_work, ldc2114_polling_work);

		ldc->poll_nb.notifier_call = ldc2114_poll_enable_cb;
		ret = ldc2114_register_client(ldc, &ldc->poll_nb);
		if (ret < 0)
			dev_warn(ldc->dev, "Unable to register notifier: %d\n", ret);
	}

	ret = sysfs_create_group(&ldc->dev->kobj, &ldc2114_attr_group);
	if (ret)
		dev_err(ldc->dev, "Unable to create sysfs group: %d\n", ret);

	ldc->reboot_nb.notifier_call = ldc2114_reboot_cb;
	ldc->reboot_nb.next = NULL;
	ldc->reboot_nb.priority = 1;
	ret = register_reboot_notifier(&ldc->reboot_nb);
	if (ret)
		dev_err(ldc->dev, "register for reboot failed: %d\n", ret);

	return 0;
}

static int ldc2114_remove(struct i2c_client *client)
{
	struct ldc2114_data *ldc = i2c_get_clientdata(client);

	if (ldc->irq_enabled) {
		disable_irq(ldc->irq);
		cancel_delayed_work_sync(&ldc->irq_work);
	}

	if (ldc->data_polling)
		cancel_delayed_work_sync(&ldc->polling_work);

	if (ldc->input)
		input_unregister_device(ldc->input);

	ldc2114_debugfs_remove(ldc);

	sysfs_remove_group(&client->dev.kobj, &ldc2114_attr_group);

	unregister_reboot_notifier(&ldc->reboot_nb);

	return 0;
}

static const struct i2c_device_id ldc2114_ids[] = {
	{ "ldc2114", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ldc2114_ids);

static struct i2c_driver ldc2114_i2c_driver = {
	.driver = {
		.name = LDC2114_DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ldc2114_of_match),
#endif
	},
	.probe = ldc2114_probe,
	.remove = ldc2114_remove,
	.id_table = ldc2114_ids,
};

static int __init ldc2114_init(void)
{
	return i2c_add_driver(&ldc2114_i2c_driver);
}

static void __exit ldc2114_exit(void)
{
	i2c_del_driver(&ldc2114_i2c_driver);
}

module_init(ldc2114_init);
module_exit(ldc2114_exit);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI LDC2114 Metal Touch Inductance-to-Digital Converter");
MODULE_LICENSE("GPL v2");
