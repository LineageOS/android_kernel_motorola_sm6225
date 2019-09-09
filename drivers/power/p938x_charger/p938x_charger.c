/*
 * Copyright (c) 2019 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/string.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/limits.h>

#define CHIP_ID_REG			0x0000
#define HW_VER_REG			0x0002
#define CUST_ID_REG			0x0003
#define MTP_FW_VER_REG		0x0004
#define MTP_FW_DATE_REG		0x0008
#define E2PROM_FW_VER_REG	0x001c
#define DEV_STATUS_REG		0x0034
#define IRQ_STATUS_REG		0x0036
#define IRQ_ENABLE_REG		0x0038
#define IRQ_CLEAR_REG		0x003a

#define VOUT_SET_REG		0x003c
#define ILIMIT_SET_REG		0x003d

#define VRECT_READ_REG		0x0040
#define VOUT_READ_REG		0x0042
#define IOUT_READ_REG		0x0044
#define OPT_FREQ_REG		0x0048
#define DIE_TEMP_REG		0x0066

#define SYS_MODE_REG		0x004c
#define CMD_2_REG		0x004e

#define ST_TX_FOD_FAULT	BIT(15)
#define ST_TX_CONFLICT	BIT(14)
#define ST_RX_CONN		BIT(13)
#define ST_ADT_ERR		BIT(11)
#define ST_ADT_RCV		BIT(9)
#define ST_ADT_SENT		BIT(8)
#define ST_VOUT_ON		BIT(7)
#define ST_VRECT_ON		BIT(6)
#define ST_MODE_CHANGE	BIT(5)
#define ST_OVER_TEMP	BIT(2)
#define ST_OVER_VOLT	BIT(1)
#define ST_OVER_CURR	BIT(0)

#define SYS_MODE_RAMCODE	BIT(6)
#define SYS_MODE_EXTENDED	BIT(3)
#define SYS_MODE_TXMODE	BIT(2)
#define SYS_MODE_WPCMODE	BIT(0)

#define CMD_2_RENEGOTIATE	BIT(7)
#define CMD_2_SWITCH_RAM	BIT(6)
#define CMD_2_CLR_IRQ		BIT(5)
#define CMD_2_SEND_CSP		BIT(4)
#define CMD_2_SEND_EPT		BIT(3)
#define CMD_2_CFG_TABLE		BIT(2)
#define CMD_2_TOGGLE_LDO	BIT(1)
#define CMD_2_SEND_RX_DATA	BIT(0)

#define WAIT_FOR_AUTH_MS 1000
#define WAIT_FOR_RCVD_TIMEOUT_MS 1000
#define HEARTBEAT_INTERVAL_MS 60000

#define MAX_VOUT_MV_DEFAULT 5000
#define MAX_IOUT_MA_DEFAULT 500
#define MAX_VOUT_MV_NORM 5000
#define MAX_IOUT_MA_NORM 1000
#define MAX_VOUT_MV_FAST 9000
#define MAX_IOUT_MA_FAST 1100

#define WLS_SHOW_MAX_SIZE 32

#define p938x_err(chip, fmt, ...)		\
	pr_err("%s: %s: " fmt, chip->name,	\
		__func__, ##__VA_ARGS__)	\

#define p938x_dbg(chip, reason, fmt, ...)			\
	do {							\
		if (*chip->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

static char mtp_downloader[] = {
0x00, 0x04, 0x00, 0x20, 0xE7, 0x00, 0x00, 0x00,
0x41, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xFE, 0xE7, 0x00, 0x00, 0xF0, 0xB5, 0x42, 0x49,
0x00, 0x20, 0x0A, 0x88, 0x05, 0x46, 0x93, 0x06,
0x00, 0xD5, 0x04, 0x20, 0xD2, 0x06, 0x07, 0xD5,
0x8A, 0x78, 0x0B, 0x79, 0x1A, 0x43, 0x92, 0x07,
0x02, 0xD1, 0x20, 0x22, 0x10, 0x43, 0x01, 0x25,
0x3A, 0x4B, 0x5A, 0x22, 0x1A, 0x74, 0x39, 0x4A,
0x20, 0x3A, 0x10, 0x72, 0x02, 0x20, 0x40, 0x1C,
0x20, 0x28, 0xFC, 0xD3, 0xFF, 0x20, 0x36, 0x4B,
0x01, 0x30, 0x98, 0x81, 0x48, 0x88, 0xDC, 0x13,
0x04, 0x19, 0x00, 0x2D, 0x09, 0xD0, 0x00, 0x20,
0x03, 0xE0, 0x45, 0x18, 0xAD, 0x68, 0x25, 0x50,
0x00, 0x1D, 0x8D, 0x88, 0x85, 0x42, 0xF8, 0xD8,
0x08, 0xE0, 0x00, 0x20, 0x03, 0xE0, 0x45, 0x18,
0x2D, 0x7A, 0x25, 0x54, 0x40, 0x1C, 0x8D, 0x88,
0x85, 0x42, 0xF8, 0xD8, 0x00, 0x20, 0x10, 0x72,
0x28, 0x48, 0x98, 0x81, 0x02, 0x20, 0x00, 0x23,
0x1A, 0x46, 0x0B, 0xE0, 0x57, 0x18, 0x3E, 0x7A,
0xA5, 0x5C, 0xAE, 0x42, 0x05, 0xD0, 0x3D, 0x72,
0x00, 0x2B, 0x00, 0xD1, 0x4A, 0x80, 0x04, 0x20,
0x5B, 0x1C, 0x52, 0x1C, 0x8D, 0x88, 0x95, 0x42,
0xF0, 0xD8, 0x8B, 0x80, 0xF0, 0xBD, 0x1F, 0x49,
0x1D, 0x48, 0x08, 0x60, 0x1A, 0x48, 0x08, 0x25,
0x40, 0x38, 0x85, 0x83, 0x5A, 0x21, 0x01, 0x70,
0x01, 0x21, 0x01, 0x71, 0x05, 0x21, 0x01, 0x72,
0x19, 0x49, 0x81, 0x81, 0x12, 0x4F, 0x00, 0x20,
0x38, 0x80, 0xFF, 0x20, 0x40, 0x1E, 0xFD, 0xD1,
0x02, 0x26, 0x38, 0x78, 0x3C, 0x46, 0xC0, 0x07,
0xFB, 0xD0, 0x60, 0x88, 0xA2, 0x88, 0x10, 0x18,
0x81, 0xB2, 0x00, 0x20, 0x04, 0xE0, 0x03, 0x19,
0x1B, 0x7A, 0x59, 0x18, 0x89, 0xB2, 0x40, 0x1C,
0x82, 0x42, 0xF8, 0xD8, 0xE0, 0x88, 0x88, 0x42,
0x01, 0xD0, 0x3D, 0x80, 0xE9, 0xE7, 0x00, 0x2A,
0x03, 0xD0, 0xFF, 0xF7, 0x7F, 0xFF, 0x20, 0x80,
0xE3, 0xE7, 0x3E, 0x80, 0xE1, 0xE7, 0x00, 0x00,
0x00, 0x04, 0x00, 0x20, 0x40, 0x5C, 0x00, 0x40,
0x40, 0x30, 0x00, 0x40, 0xFF, 0x01, 0x00, 0x00,
0xFF, 0x0F, 0x00, 0x00, 0x80, 0xE1, 0x00, 0xE0,
0x04, 0x0E, 0x00, 0x00 };

enum {
	PROGRAM_FW_NONE,
	PROGRAM_FW_PENDING,
	PROGRAM_FW_SUCCESS,
	PROGRAM_FW_FAIL,
};

enum print_reason {
	PR_INTERRUPT    = BIT(0),
	PR_MISC         = BIT(2),
	PR_MOTO         = BIT(7),
	PR_FWPROG       = BIT(6)
};

static int __debug_mask = PR_MOTO | PR_INTERRUPT;
module_param_named(
	debug_mask, __debug_mask, int, S_IRUSR | S_IWUSR
);

struct battery_st {
	int status;
	int capacity;
	int voltage_now;
	int voltage_max;
	int current_now;
	int current_max;
	int charge_type;
	int temp;
};

struct p938x_charger {
	const char		*name;
	int			*debug_mask;
	struct i2c_client	*client;
	struct device		*dev;
	struct regmap		*regmap;

	u32			chg_efficiency;
	u32			chg_limit_soc;
	u32			chg_idle_cl;
	struct regulator	*vdd_i2c_vreg;
	struct gpio		wchg_int_n;
	struct gpio		wchg_en_n;
	struct gpio		wchg_det;
	struct gpio		wchg_sleep;
	struct gpio		wchg_boost;
	int			wchg_det_irq;
	struct pinctrl		*pinctrl_irq;
	const char		*pinctrl_name;

	u16			irq_en;
	u16			irq_stat;
	u16			stat;
	u16			mode;
	u16			rx_vout_max;
	u16			rx_iout_max;
	u16			wls_allowed_vmax;
	u16			wls_allowed_imax;
	bool			wired_connected;
	bool			charging_disabled;
	int			temp_level;
	int			thermal_levels;
	struct battery_st	batt_st;

	struct work_struct	chg_det_work;
	struct delayed_work	heartbeat_work;

	u32			peek_poke_address;
	struct dentry		*debug_root;

	struct power_supply	*consumer_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*main_psy;
	struct power_supply     *wls_psy;
	struct power_supply_desc	wls_psy_d;

	char			fw_name[NAME_MAX];
	int			program_fw_stat;

	atomic_t tx_attached;
	atomic_t boost_enabled;
};

static char *pm_wls_supplied_to[] = {
	"wireless",
};

static char *pm_wls_supplied_from[] = {
	"battery",
	"usb",
	"dc",
};

static inline int p938x_is_chip_on(struct p938x_charger *chip)
{
	return (atomic_read(&chip->boost_enabled) ||
		atomic_read(&chip->tx_attached));
}

static inline int p938x_is_tx_connected(struct p938x_charger *chip)
{
	return (p938x_is_chip_on(chip) && (chip->stat & ST_VRECT_ON));
}

static inline int p938x_is_ldo_on(struct p938x_charger *chip)
{
	return (p938x_is_chip_on(chip) && (chip->stat & ST_VOUT_ON));
}

static int p938x_read_reg(struct p938x_charger *chip, u16 reg, u8 *val)
{
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, reg, &temp);
	if (rc < 0)
		p938x_err(chip, "Failed to read reg=0x%x, rc=%d\n", reg, rc);
	else
		*val = (u8)temp;

	return rc;
}

static int p938x_write_reg(struct p938x_charger *chip, u16 reg, u8 val)
{
	int rc;

	rc = regmap_write(chip->regmap, reg, val);
	if (rc < 0)
		p938x_err(chip, "Failed to write reg=0x%x, rc=%d\n", reg, rc);

	return rc;
}

static void p938x_write_reg_ignore_err(struct p938x_charger *chip, u16 reg, u8 val)
{
	regmap_write(chip->regmap, reg, val);
}

static int p938x_read_buffer(struct p938x_charger *chip, u16 reg,
		u8 *buf, u32 size)
{
	int rc;

	rc = regmap_bulk_read(chip->regmap, reg, buf, size);
	if (rc < 0)
		p938x_err(chip, "Failed to read buffer on reg=0x%x, rc=%d\n",
					reg, rc);

	return rc;
}

static int p938x_write_buffer(struct p938x_charger *chip, u16 reg,
		u8 *buf, u32 size)
{
	int rc = 0;

	while (size--) {
		rc = regmap_write(chip->regmap, reg++, *buf++);
		if (rc < 0)
			break;
	}

	if (rc < 0)
		p938x_err(chip, "Failed to write buffer on reg=0x%x, rc=%d\n",
					reg, rc);

	return rc;
}

static void p938x_reset(struct p938x_charger *chip)
{
	p938x_write_reg_ignore_err(chip, 0x3040, 0x80);
}

static int p938x_get_rx_vout(struct p938x_charger *chip)
{
	int rc;
	u16 volt;

	rc = p938x_read_buffer(chip, VOUT_READ_REG, (u8 *)&volt, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read rx voltage, rc = %d\n", rc);
		return rc;
	}

	return volt;
}

static int p938x_set_rx_vout(struct p938x_charger *chip, u16 mv)
{
	int rc;
	u16 rx_vout_max = mv;

	if (mv < 3500)
		mv = 3500;
	else if (mv > 12500)
		mv = 12500;

	mv = mv / 100;
	rc = p938x_write_reg(chip, VOUT_SET_REG, (u8)mv);
	if (rc < 0)
		p938x_err(chip, "Failed to set rx voltage, rc = %d\n", rc);
	else
		chip->rx_vout_max = rx_vout_max;

	return rc;
}

static int p938x_get_rx_iout(struct p938x_charger *chip)
{
	int rc;
	u16 ma;

	rc = p938x_read_buffer(chip, IOUT_READ_REG, (u8 *)&ma, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read rx current, rc = %d\n", rc);
		return rc;
	}

	return ma;
}

static int p938x_get_rx_ocl(struct p938x_charger *chip)
{
	int rc;
	u16 ma;

	rc = p938x_read_buffer(chip, ILIMIT_SET_REG, (u8 *)&ma, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read rx current, rc = %d\n", rc);
		return rc;
	}

	return (ma & 0xf) * 100 + 100;
}

static int p938x_enable_charging(struct p938x_charger *chip, bool on)
{
	int rc = 0;

	rc = p938x_write_reg(chip, CMD_2_REG, CMD_2_TOGGLE_LDO);
	if (rc < 0)
		p938x_err(chip, "Failed to %s RX ldo, rc = %d\n",
			on ? "enable":"disable", rc);
	else {
		p938x_dbg(chip, PR_MOTO, "RX ldo is %s\n",
			on ? "enabled":"disabled");
	}

	return rc;
}

// TODO
/*static int p938x_request_rx_power(struct p938x_charger *chip,
		int vout, int iout)
{
	int rc = 0;
	u16 vmax, imax;

	if (vout < 0 && iout < 0) {
		p938x_err(chip, "Invalid request power parameters\n");
		return -EINVAL;
	}

	p938x_get_tx_capability(chip, &vmax, &imax);
	if (vout >= 0) {
		if (vmax < vout)
			return -EINVAL;

		rc = p938x_set_fastchg_voltage(chip, vout);
		if (rc == -EOPNOTSUPP && vout < MAX_VOUT_MV_FAST)
			rc = p938x_set_rx_vout(chip, vout);
		if (rc < 0) {
			p938x_err(chip, "Failed to req vout=%d, rc=%d\n",
						vout, rc);
			return rc;
		}
	}

	if (chip->rx_vout_max <= MAX_VOUT_MV_NORM &&
	    (vmax > MAX_VOUT_MV_NORM ||
	    imax > MAX_IOUT_MA_NORM))
		imax = MAX_IOUT_MA_NORM;

	if (chip->rx_iout_max > imax)
		chip->rx_iout_max = imax;

	if (iout >= 0) {
		if (imax < iout)
			return -EINVAL;

		chip->rx_iout_max = iout;
	}

	p938x_dbg(chip, PR_MOTO, "Request RX power, V=%dmV, I=%dmA\n",
				chip->rx_vout_max, chip->rx_iout_max);

	return rc;
}*/

static int p938x_update_supplies_status(struct p938x_charger *chip)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chip->usb_psy)
		chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		pr_debug("USB psy not found\n");
		return -EINVAL;
	}
	rc = power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read USB present prop, rc=%d\n", rc);
		return rc;
	} else if (!prop.intval) {
		rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_MODE, &prop);
		if (rc) {
			p938x_err(chip,
				"Couldn't read USB mode prop, rc=%d\n", rc);
			return rc;
		}
	}
	chip->wired_connected = !!prop.intval;

	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		pr_debug("Battery psy not found\n");
		return -EINVAL;
	}
	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_STATUS, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt status prop, rc=%d\n", rc);
		return rc;
	}
	chip->batt_st.status = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt capacity prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.capacity = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_TEMP, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt temp prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.temp = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt voltage now prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.voltage_now = prop.intval / 1000;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt current now prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.current_now = prop.intval / 1000;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt charge type prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.charge_type = prop.intval;

	if (!chip->main_psy)
		chip->main_psy = power_supply_get_by_name("main");
	if (!chip->main_psy) {
		pr_debug("Main psy not found\n");
		return -EINVAL;
	}
	rc = power_supply_get_property(chip->main_psy,
			POWER_SUPPLY_PROP_VOLTAGE_MAX, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt voltage max prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.voltage_max = prop.intval / 1000;

	rc = power_supply_get_property(chip->main_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt current max prop, rc=%d\n",
					rc);
		return rc;
	}
	chip->batt_st.current_max = prop.intval / 1000;

	return 0;
}

static void p938x_handle_wls_insertion(struct p938x_charger *chip)
{
	chip->rx_vout_max = MAX_VOUT_MV_DEFAULT;
	chip->rx_iout_max = 0;

	// TODO hardcoded for testing
	p938x_set_rx_vout(chip, 5000);
	p938x_enable_charging(chip, 1);

	power_supply_changed(chip->wls_psy);
	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
			msecs_to_jiffies(0));
}

static void p938x_handle_wls_removal(struct p938x_charger *chip)
{
	chip->irq_en = 0;
	chip->irq_stat = 0;
	chip->stat = 0;
	chip->rx_vout_max = 0;
	chip->rx_iout_max = 0;
	chip->temp_level = 0;
	chip->thermal_levels = 0;
	memset(&chip->batt_st, 0, sizeof(struct battery_st));

	power_supply_changed(chip->wls_psy);
	cancel_delayed_work(&chip->heartbeat_work);
}

static void p938x_clear_irq(struct p938x_charger *chip, u16 mask)
{
	p938x_dbg(chip, PR_MOTO, "IRQ Clear 0x%02X\n", mask);
	p938x_write_buffer(chip, IRQ_CLEAR_REG, (u8 *)&mask, 2);
	p938x_write_reg(chip, CMD_2_REG, CMD_2_CLR_IRQ);
}

static void p938x_chg_det_work(struct work_struct *work)
{
	struct p938x_charger *chip = container_of(work,
				struct p938x_charger, chg_det_work);

	if (!p938x_is_tx_connected(chip)) {
		p938x_handle_wls_removal(chip);
		p938x_dbg(chip, PR_MOTO, "Wireless charger is removed\n");
	} else if (p938x_is_tx_connected(chip)) {
		p938x_handle_wls_insertion(chip);
		p938x_dbg(chip, PR_MOTO, "Wireless charger is inserted\n");
	}
}

// TODO
static void p938x_heartbeat_work(struct work_struct *work)
{
	struct p938x_charger *chip = container_of(work,
				struct p938x_charger, heartbeat_work.work);

	if (p938x_is_tx_connected(chip)) {
		int rc;
		/*u16 vmax, imax;
		u16 target_vout, target_iout;*/

		rc = p938x_update_supplies_status(chip);
		if (rc < 0) {
			p938x_err(chip,	"Update supplies status, rc=%d\n", rc);
			schedule_delayed_work(&chip->heartbeat_work,
				msecs_to_jiffies(HEARTBEAT_INTERVAL_MS));
			return;
		}

		/* p938x_get_tx_capability(chip, &vmax, &imax);
		if (chip->wired_connected || chip->charging_disabled) {
			target_vout = MAX_VOUT_MV_DEFAULT;
			target_iout = 0;
		} else if (chip->batt_st.capacity > chip->chg_limit_soc) {
			target_vout = MAX_VOUT_MV_DEFAULT;
			if (chip->batt_st.status == POWER_SUPPLY_STATUS_FULL)
				target_iout = chip->chg_idle_cl;
			else if (chip->batt_st.capacity == 100)
				target_iout = MAX_IOUT_MA_DEFAULT / 2;
			else
				target_iout = MAX_IOUT_MA_DEFAULT;
		} else {
			int batt_pwr_max;
			int chg_pwr_max;
			int type = chip->batt_st.charge_type;

			if (type == POWER_SUPPLY_CHARGE_TYPE_TAPER &&
			    chip->batt_st.current_now < 0) {
				batt_pwr_max = chip->batt_st.voltage_max *
						(-chip->batt_st.current_now);
				chg_pwr_max = batt_pwr_max * 100 /
						chip->chg_efficiency;
			} else if (type == POWER_SUPPLY_CHARGE_TYPE_FAST ||
			    type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE) {
				batt_pwr_max = chip->batt_st.voltage_max *
						chip->batt_st.current_max;
				chg_pwr_max = batt_pwr_max * 100 /
						chip->chg_efficiency;
			} else
				chg_pwr_max = chip->rx_vout_max *
						chip->rx_iout_max;

			chg_pwr_max = min(chg_pwr_max, (vmax * imax));
			target_vout = vmax;
			target_iout = chg_pwr_max / target_vout;
			if (target_iout < chip->chg_idle_cl)
				target_iout = chip->chg_idle_cl;
		}

		target_vout = min(target_vout, chip->wls_allowed_vmax);
		target_iout = min(target_iout, chip->wls_allowed_imax);
		if (target_vout != chip->rx_vout_max ||
		    target_iout != chip->rx_iout_max) {
			rc = p938x_request_rx_power(chip,
						target_vout,
						target_iout);
			if (rc < 0)
				p938x_err(chip, "Request power, rc=%d\n", rc);
			else
				power_supply_changed(chip->wls_psy);
		} else if (!chip->consumer_psy) {
			chip->consumer_psy =
				power_supply_get_by_name(pm_wls_supplied_to[0]);
			power_supply_changed(chip->wls_psy);
		}*/

		schedule_delayed_work(&chip->heartbeat_work,
			msecs_to_jiffies(HEARTBEAT_INTERVAL_MS));
	}
}

static int p938x_init_gpio(struct p938x_charger *chip,
	struct gpio *p_gpio, int dir_out, int out_val)
{
	int rc;

	rc = gpio_request_one(p_gpio->gpio, p_gpio->flags, p_gpio->label);
	if (rc) {
		p938x_err(chip, "Failed to request gpio %d\n", p_gpio->gpio);
		return rc;
	}

	if (dir_out)
		rc = gpio_direction_output(p_gpio->gpio, out_val);
	else
		rc = gpio_direction_input(p_gpio->gpio);

	if (rc) {
		p938x_err(chip, "Failed to set gpio direction for gpio %d\n",
			p_gpio->gpio);
		return rc;
	}

	return 0;
}

static int p938x_hw_init(struct p938x_charger *chip)
{
	int rc;

	if (regulator_count_voltages(chip->vdd_i2c_vreg) > 0) {
		rc = regulator_set_voltage(chip->vdd_i2c_vreg,
					1800000, 1800000);
		if (rc) {
			p938x_err(chip, "Failed to set vreg voltage, rc=%d\n",
						rc);
			return rc;
		}
	}
	rc = regulator_enable(chip->vdd_i2c_vreg);
	if (rc) {
		p938x_err(chip, "Failed to enable vdd vreg, rc=%d\n", rc);
		return rc;
	}

	if (chip->pinctrl_name) {
		chip->pinctrl_irq = pinctrl_get_select(chip->dev,
						chip->pinctrl_name);
		if (IS_ERR(chip->pinctrl_irq)) {
			p938x_err(chip,
				"Couldn't get/set %s pinctrl state rc=%ld\n",
				chip->pinctrl_name,
				PTR_ERR(chip->pinctrl_irq));
			rc = PTR_ERR(chip->pinctrl_irq);
			goto disable_vreg;
		}
	}

	if (p938x_init_gpio(chip, &chip->wchg_int_n, 0, 0))
		goto disable_vreg;

	if (p938x_init_gpio(chip, &chip->wchg_det, 0, 0))
		goto disable_vreg;

	if (p938x_init_gpio(chip, &chip->wchg_en_n, 1, 0))
		goto disable_vreg;

	if (p938x_init_gpio(chip, &chip->wchg_boost, 1, 0))
		goto disable_vreg;

	if (p938x_init_gpio(chip, &chip->wchg_sleep, 1, 0))
		goto disable_vreg;

	chip->wchg_det_irq = gpio_to_irq(chip->wchg_det.gpio);

disable_vreg:
	regulator_disable(chip->vdd_i2c_vreg);

	return rc;
}

static inline void p938x_set_boost(struct p938x_charger *chip, int val)
{
	gpio_set_value(chip->wchg_boost.gpio, !!val);
	atomic_set(&chip->boost_enabled, !!val);
}

static inline int p938x_get_boost(struct p938x_charger *chip)
{
	return gpio_get_value(chip->wchg_boost.gpio);
}

static int p938x_program_mtp_downloader(struct p938x_charger *chip)
{
	int i;
	int rc = 0;

	/*
	 * Transfer 9382 boot loader code "MTPBootloader" to 9382 SRAM
	 * - Setup 9382 registers before transferring the boot loader code
	 * - Transfer the boot loader code to 9382 SRAM
	 * - Reset 9382 => 9382 M0 runs the boot loader
	 */
	rc = p938x_write_reg(chip, 0x3000, 0x5a);
	if (rc < 0) {
		p938x_err(chip, "Failed to write 0x3000(5a), rc=%d\n", rc);
		return rc;
	}

	msleep(10);

	rc = p938x_write_reg(chip, 0x3040, 0x10);
	if (rc < 0) {
		p938x_err(chip, "Failed to write 0x3040(10), rc=%d\n", rc);
		return rc;
	}

	msleep(10);

	/* Write MTP downloader data */
	for (i = 0; i < sizeof(mtp_downloader); i++) {
		rc = p938x_write_reg(chip, 0x1c00 + i, mtp_downloader[i]);
		if (rc < 0) {
			p938x_err(chip, "Failed to program MTP downloader\n");
			return rc;
		}
	}

	rc = p938x_write_reg(chip, 0x400, 0x00);
	if (rc < 0) {
		p938x_err(chip, "Failed to write 0x400(0), rc=%d\n", rc);
		return rc;
	}

	rc = p938x_write_reg(chip, 0x3048, 0x80);
	if (rc < 0) {
		p938x_err(chip, "Failed to write 0x3048(80), rc=%d\n", rc);
		return rc;
	}

	/* ignoreNAK */
	p938x_reset(chip);
	msleep(100);

	return rc;
}

static inline int align_16(u32 val)
{
	return ((val + 15) / 16 ) * 16;
}

#define WLS_LOG_BUF_SIZE 1024
static void p938x_print_packet(struct p938x_charger *chip, u16 addr, char *buf, int len)
{
	int i;
	char log_buf[WLS_LOG_BUF_SIZE];
	int offset = 0;

	for (i=0; i<len; i++) {
		offset += scnprintf(log_buf + offset, WLS_LOG_BUF_SIZE, "%02x ", buf[i]);
		if ((i + 1) % 8 == 0) {
			p938x_dbg(chip, PR_FWPROG, "%s\n", log_buf);
			offset = 0;
		}
	}
	if (i % 8)
		p938x_dbg(chip, PR_FWPROG, "%s\n", log_buf);
}

#define FW_MTP_CHECK_ST_RETRY_CNT 30
#define FW_MTP_PACK_HEADER_LEN 8
#define FW_MTP_PADDING 8
#define FW_MTP_PACK_DATA_LEN 128
#define FW_MTP_PACK_SIZE (FW_MTP_PACK_HEADER_LEN + FW_MTP_PACK_DATA_LEN + FW_MTP_PADDING)
static int p938x_program_mtp_package(struct p938x_charger *chip,
			const u8 *src, u16 addr, u32 size)
{
	int i;
	int rc;
	u16 check_sum = addr;
	u16 code_len = FW_MTP_PACK_DATA_LEN;
	u32 packet_len;
	int retry_cnt = 0;
	char buf[FW_MTP_PACK_SIZE];

	memset(buf, 0, FW_MTP_PACK_SIZE);

	/* Check if we don't have the full 128 bytes left */
	if ((size - addr) < FW_MTP_PACK_DATA_LEN)
		code_len = size - addr;

	/*(1) Copy the bytes of the MTP image data to the packet data buffer */
	memcpy(buf + FW_MTP_PACK_HEADER_LEN, src, code_len);

	/* Now that we copied it, include padded bits */
	code_len = align_16(code_len);

	/* Packet length 16 byte aligned */
	packet_len = align_16(code_len + FW_MTP_PACK_HEADER_LEN);

	/*
	 *(2) Calculate the packet checksum of the 128-byte data,
	 *	StartAddr, and CodeLength
	 */
	for (i = code_len - 1; i >= 0; i--)
		check_sum += buf[i + FW_MTP_PACK_HEADER_LEN];
	check_sum += code_len;

	/*(3) Fill up StartAddr, CodeLength, CheckSum of the current packet.*/
	memcpy(buf + 2, &addr, 2);
	memcpy(buf + 4, &code_len, 2);
	memcpy(buf + 6, &check_sum, 2);

	p938x_dbg(chip, PR_FWPROG, "Writing MTP to 0x%04x (%d;%d)\n",
		addr, code_len, packet_len);
	p938x_print_packet(chip, addr, buf, packet_len);

	/* Send the current packet to SRAM via I2C
	 * read status is guaranteed to be != 1 at this point
	 */
	for (i = 0; i < packet_len; i++) {
		rc = p938x_write_reg(chip, 0x400 + i, buf[i]);
		if (rc < 0) {
			p938x_err(chip, "ERROR: on writing to MTP buffer\n");
			return rc;
		}
	}

	/* Write 0x11 to the Status in the SRAM. This informs the 9382 to
	 * start programming the new packet
	 * from SRAM to OTP memory
	 */
	rc = p938x_write_reg(chip, 0x400, 0x1);
	if (rc < 0) {
		p938x_err(chip, "ERROR: on MTP buffer validation\n");
		return rc;
	}

	/* Wait for 9382 bootloader to complete programming the current
	 * packet image data from SRAM to the OTP.
	 * The boot loader will update the Status in the SRAM as follows:
	 * Status:
	 * "0" - reset value (from AP)
	 * "1" - buffer validated / busy (from AP)
	 * "2" - finish "OK" (from the boot loader)
	 * "4" - programming error (from the boot loader)
	 * "8" - wrong check sum (from the boot loader)
	 * "16"- programming not possible (try to write "0" to bit location
	 * already programmed to "1")
	 *       (from the boot loader)
	 * DateTime startT = DateTime.Now;
	 */
	do {
		msleep(20);
		p938x_read_reg(chip, 0x400, buf);
		if (retry_cnt++ > FW_MTP_CHECK_ST_RETRY_CNT) {
			p938x_err(chip, "Status timed out");
			break;
		}
	} while (buf[0] == 1);

	if (buf[0] != 2) {
		p938x_err(chip,
			"ERROR: Programming MTP buffer status:%02x at:%d\n",
			buf[0], addr);
		return -EAGAIN;
	}

	return 0;
}

static int p938x_program_mtp(struct p938x_charger *chip, const u8 *src, u32 size)
{
	int i;
	int rc;

	/* Program MTP image data to 9382 memory */
	for (i = 0; i < size; i += FW_MTP_PACK_DATA_LEN) {
		rc = p938x_program_mtp_package(chip, src, i, size);
		if (rc < 0) {
			p938x_err(chip, "Program MTP failed at 0x%04x\n", i);
			return rc;
		}
		src += FW_MTP_PACK_DATA_LEN;
	}

	/*
	 * Need to reset or power cycle 9382 to run the OTP code
	*/
	rc = p938x_write_reg(chip, 0x3000, 0x5a);
	if (rc < 0) {
		p938x_err(chip, "Failed to write 0x3000(5a), rc=%d\n", rc);
		return rc;
	}

	rc = p938x_write_reg(chip, 0x3048, 0x00);
	if (rc < 0) {
		p938x_err(chip, "Failed to write 0x3048(00), rc=%d\n", rc);
		return rc;
	}

	/* ignoreNAK */
	p938x_reset(chip);
	msleep(100);

	return rc;
}

static int p938x_program_fw(struct p938x_charger *chip)
{
	int rc = 0;
	const struct firmware *fw = NULL;

	if (chip->program_fw_stat == PROGRAM_FW_PENDING) {
		p938x_err(chip, "Programming FW is ongoing\n");
		return -EBUSY;
	}
	chip->program_fw_stat = PROGRAM_FW_PENDING;

	rc = request_firmware(&fw, chip->fw_name, chip->dev);
	if (rc < 0) {
		p938x_err(chip, "Request firmware (%s) failed, rc=%d\n",
			chip->fw_name, rc);
		chip->program_fw_stat = PROGRAM_FW_FAIL;
		return rc;
	}

	/* Turn on the boost so i2c works */
	p938x_set_boost(chip, 1);

	/* Brief delay for ic init */
	msleep(100);

	p938x_dbg(chip, PR_MOTO, "Loading FW programmer...\n");

	rc = p938x_program_mtp_downloader(chip);
	if (rc < 0) {
		p938x_err(chip, "Programming MTP downloader failed, rc=%d\n",
			rc);
		chip->program_fw_stat = PROGRAM_FW_FAIL;
		goto release_fw;
	}

	p938x_dbg(chip, PR_MOTO, "Starting FW programming...\n");

	rc = p938x_program_mtp(chip, fw->data, fw->size);
	if (rc < 0) {
		p938x_err(chip, "Programming MTP failed, rc=%d\n", rc);
		chip->program_fw_stat = PROGRAM_FW_FAIL;
		goto release_fw;
	}

	chip->program_fw_stat = PROGRAM_FW_SUCCESS;
	p938x_dbg(chip, PR_MOTO, "Programming FW success\n");

release_fw:
	release_firmware(fw);
	p938x_set_boost(chip, 0);
	return rc;
}

static int p938x_get_status(struct p938x_charger *chip)
{
	int rc;

	chip->irq_en = 0;
	rc = p938x_read_buffer(chip, IRQ_ENABLE_REG, (u8 *)&chip->irq_en, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to get irq enable, rc=%d\n", rc);
		return rc;
	}

	// TODO spec says to read these two together

	chip->stat = 0;
	rc = p938x_read_buffer(chip, DEV_STATUS_REG, (u8 *)&chip->stat, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to get status, rc=%d\n", rc);
		return rc;
	}

	chip->irq_stat = 0;
	rc = p938x_read_buffer(chip, IRQ_STATUS_REG, (u8 *)&chip->irq_stat, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to get irq status, rc=%d\n", rc);
		return rc;
	}

	chip->mode = 0;
	rc = p938x_read_buffer(chip, SYS_MODE_REG, (u8 *)&chip->mode, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to get mode status, rc=%d\n", rc);
		return rc;
	}

	p938x_dbg(chip, PR_MOTO, "IRQ_ENABLE=0x%04x, IRQ_ST=0x%04x, STATUS=0x%04x, MODE=0x%02x\n",
		chip->irq_en, chip->irq_stat, chip->stat, chip->mode);

	return 0;
}

static void p938x_check_status(struct p938x_charger *chip)
{
	u16 status = chip->stat;
	u16 irq_status = chip->irq_stat;

	if(status & ST_TX_FOD_FAULT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_TX_FOD_FAULT\n");
	if(status & ST_TX_CONFLICT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_TX_CONFLICT\n");
	if(status & ST_RX_CONN)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_RX_CONN\n");
	if(status & ST_ADT_ERR)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_ADT_ERR\n");
	if(status & ST_ADT_RCV)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_ADT_RCV\n");
	if(status & ST_ADT_SENT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_ADT_SENT\n");
	if(status & ST_VOUT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_VOUT_ON\n");
	if(status & ST_VRECT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_VRECT_ON\n");
	if(status & ST_MODE_CHANGE)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_MODE_CHANGE\n");
	if(status & ST_OVER_TEMP)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_OVER_TEMP\n");
	if(status & ST_OVER_VOLT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_OVER_VOLT\n");
	if(status & ST_OVER_CURR)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_OVER_CURR\n");

	if(irq_status & ST_TX_FOD_FAULT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_TX_FOD_FAULT\n");
	if(irq_status & ST_TX_CONFLICT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_TX_CONFLICT\n");
	if(irq_status & ST_RX_CONN)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_RX_CONN\n");
	if(irq_status & ST_ADT_ERR)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_ADT_ERR\n");
	if(irq_status & ST_ADT_RCV)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_ADT_RCV\n");
	if(irq_status & ST_ADT_SENT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_ADT_SENT\n");
	if(irq_status & ST_VOUT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_VOUT_ON\n");
	if(irq_status & ST_VRECT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_VRECT_ON\n");
	if(irq_status & ST_MODE_CHANGE)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_MODE_CHANGE\n");
	if(irq_status & ST_OVER_TEMP)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_OVER_TEMP\n");
	if(irq_status & ST_OVER_VOLT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_OVER_VOLT\n");
	if(irq_status & ST_OVER_CURR)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_OVER_CURR\n");
}

static void p938x_check_system_mode(struct p938x_charger *chip)
{
	u16 mode = chip->mode;

	if(mode & SYS_MODE_RAMCODE)
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_RAMCODE\n");
	if(mode & SYS_MODE_EXTENDED)
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_EXTENDED\n");
	if(mode & SYS_MODE_TXMODE)
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_TXMODE\n");
	if(mode & SYS_MODE_WPCMODE)
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_WPCMODE\n");
}

static irqreturn_t p938x_irq_handler(int irq, void *dev_id)
{
	struct p938x_charger *chip = dev_id;

	if (!chip) {
		p938x_err(chip, "Invalid chip\n");
		return IRQ_HANDLED;
	}

	if (chip->program_fw_stat == PROGRAM_FW_PENDING) {
		p938x_err(chip, "Skip irq for FW programming\n");
		return IRQ_HANDLED;
	}

	if(!p938x_get_status(chip)) {
		p938x_check_status(chip);
		p938x_check_system_mode(chip);
	}

	// TODO should this be in both places
	if(chip->irq_stat & ST_VRECT_ON)
		schedule_work(&chip->chg_det_work);

	p938x_clear_irq(chip, chip->irq_stat);

	return IRQ_HANDLED;
}

static irqreturn_t p938x_det_irq_handler(int irq, void *dev_id)
{
	struct p938x_charger *chip = dev_id;
	int tx_detected = gpio_get_value(chip->wchg_det.gpio);

	atomic_set(&chip->tx_attached, tx_detected);

	// TODO should this be in both places
	// TODO clean up on detach
	schedule_work(&chip->chg_det_work);

	return IRQ_HANDLED;
}

static ssize_t boost_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long value;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &value);
	if (r) {
		p938x_err(chip, "Invalid boost value = %ld\n", value);
		return -EINVAL;
	}

	p938x_set_boost(chip, value);

	return r ? r : count;
}

static ssize_t boost_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n",
		p938x_get_boost(chip));
}

static ssize_t chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	u8 data[2];

	p938x_read_buffer(chip, CHIP_ID_REG, data, 2);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%02x%02x\n",
		data[1], data[0]);
}

static ssize_t fw_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	u8 data[12];

	p938x_read_buffer(chip, MTP_FW_DATE_REG, data, 12);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%s\n", data);
}

static ssize_t fw_name_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	if(count && strlen(buf)) {
		strncpy(chip->fw_name, buf, NAME_MAX);
		if(chip->fw_name[strlen(chip->fw_name)-1] == '\n')
			chip->fw_name[strlen(chip->fw_name)-1] = '\0';
	}

	return count;
}

static ssize_t program_fw_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n", chip->program_fw_stat);
}

static ssize_t program_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned long r;
	unsigned long flash;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &flash);
	if (r) {
		p938x_err(chip, "Invalid flash value = %ld\n", flash);
		return -EINVAL;
	}

	if (flash) {
		rc = p938x_program_fw(chip);
		if (rc < 0)
			return rc;
	}
	return r ? r : count;
}

static ssize_t rx_vout_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned long r;
	unsigned long vout;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &vout);
	if (r) {
		p938x_err(chip, "Invalid vout value = %lu\n", vout);
		return -EINVAL;
	}

	rc = p938x_set_rx_vout(chip, (u16)vout);
	if (rc < 0)
		return rc;

	power_supply_changed(chip->wls_psy);
	return r ? r : count;
}

static ssize_t rx_vout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int vout;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	vout = p938x_get_rx_vout(chip);
	if (vout < 0)
		return vout;

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n", vout);
}

static ssize_t rx_iout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int iout;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	iout = p938x_get_rx_iout(chip);
	if (iout < 0)
		return iout;

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n", iout);
}

static ssize_t rx_ocl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ocl;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	ocl = p938x_get_rx_ocl(chip);
	if (ocl < 0)
		return ocl;

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n", ocl);
}

static DEVICE_ATTR(boost, S_IRUGO|S_IWUSR, boost_show, boost_store);
static DEVICE_ATTR(chip_id, S_IRUGO, chip_id_show, NULL);
static DEVICE_ATTR(fw_ver, S_IRUGO, fw_ver_show, NULL);
static DEVICE_ATTR(fw_name, S_IWUSR, NULL, fw_name_store);
static DEVICE_ATTR(program_fw_stat, S_IRUGO, program_fw_stat_show, NULL);
static DEVICE_ATTR(program_fw, S_IWUSR, NULL, program_fw_store);
static DEVICE_ATTR(rx_vout, S_IRUGO|S_IWUSR, rx_vout_show, rx_vout_store);
static DEVICE_ATTR(rx_iout, S_IRUGO, rx_iout_show, NULL);
static DEVICE_ATTR(rx_ocl, S_IRUGO, rx_ocl_show, NULL);

static struct attribute *p938x_attrs[] = {
	&dev_attr_boost.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_fw_name.attr,
	&dev_attr_program_fw_stat.attr,
	&dev_attr_program_fw.attr,
	&dev_attr_rx_vout.attr,
	&dev_attr_rx_iout.attr,
	&dev_attr_rx_ocl.attr,
	NULL
};

ATTRIBUTE_GROUPS(p938x);

static int show_dump_regs(struct seq_file *m, void *data)
{
	struct p938x_charger *chip = m->private;
	u8 buf[12];

	p938x_read_buffer(chip, CHIP_ID_REG, buf, 2);
	seq_printf(m, "CHIP_ID: 0x%02x%02x\n", buf[1], buf[0]);
	p938x_read_reg(chip, HW_VER_REG, &buf[0]);
	seq_printf(m, "HW_VER: 0x%02x\n", buf[0]);
	p938x_read_reg(chip, CUST_ID_REG, &buf[0]);
	seq_printf(m, "CUST_ID: 0x%02x\n", buf[0]);
	p938x_read_buffer(chip, MTP_FW_VER_REG, buf, 4);
	seq_printf(m, "MTP_FW_VER: 0x%02x%02x:0x%02x%02x\n",
			buf[0], buf[1], buf[2], buf[3]);
	p938x_read_buffer(chip, MTP_FW_DATE_REG, buf, 12);
	seq_printf(m, "MTP_FW_DATE: %s", buf);
	p938x_read_buffer(chip, E2PROM_FW_VER_REG, buf, 4);
	seq_printf(m, "E2PROM_FW_VER: 0x%02x%02x:0x%02x%02x\n",
			buf[0], buf[1], buf[2], buf[3]);
	p938x_read_buffer(chip, DEV_STATUS_REG, buf, 2);
	seq_printf(m, "DEV STATUS: 0x%02x%02x\n", buf[1], buf[0]);
	p938x_read_buffer(chip, IRQ_STATUS_REG, buf, 2);
	seq_printf(m, "IRQ STATUS: 0x%02x%02x\n", buf[1], buf[0]);
	p938x_read_buffer(chip, IRQ_ENABLE_REG, buf, 2);
	seq_printf(m, "IRQ ENABLE: 0x%02x%02x\n", buf[1], buf[0]);
	p938x_read_buffer(chip, IRQ_CLEAR_REG, buf, 2);
	seq_printf(m, "IRQ CLEAR: 0x%02x%02x\n", buf[1], buf[0]);
	p938x_read_reg(chip, CMD_2_REG, &buf[0]);
	seq_printf(m, "CMD_REG: 0x%02x\n", buf[0]);
	p938x_read_buffer(chip, VOUT_READ_REG, buf, 2);
	seq_printf(m, "VOUT: %dmV\n", (buf[1] << 8) | buf[0]);
	p938x_read_reg(chip, VOUT_SET_REG, &buf[0]);
	seq_printf(m, "VOUT SET: %dmV\n", buf[0] * 100);
	p938x_read_buffer(chip, VRECT_READ_REG, buf, 2);
	seq_printf(m, "VRECT: %dmV\n", (buf[1] << 8) | buf[0]);
	p938x_read_buffer(chip, IOUT_READ_REG, buf, 2);
	seq_printf(m, "IOUT: %dmA\n", (buf[1] << 8) | buf[0]);
	p938x_read_buffer(chip, DIE_TEMP_REG, buf, 2);
	seq_printf(m, "DIE TEMP: %d C\n",
		((((buf[1]&0x0F) << 8)|buf[0]) - 1350) * 83 / 444 - 273);
	p938x_read_buffer(chip, OPT_FREQ_REG, buf, 2);
	seq_printf(m, "OPT FREQ: %dkHz\n", (buf[1] << 8) | buf[0]);
	p938x_read_buffer(chip, ILIMIT_SET_REG, buf, 2);
	seq_printf(m, "ILIMIT_SET: %dmA\n", buf[0] * 100 + 100);
	p938x_read_reg(chip, SYS_MODE_REG, &buf[0]);
	seq_printf(m, "SYS_MODE: 0x%02x\n", buf[0]);

	return 0;
}

static int dump_regs_debugfs_open(struct inode *inode, struct file *file)
{
	struct p938x_charger *chip = inode->i_private;

	return single_open(file, show_dump_regs, chip);
}

static const struct file_operations dump_regs_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_regs_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct p938x_charger *chip = data;
	int rc;
	u8 temp;

	rc = p938x_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc) {
		p938x_err(chip, "Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct p938x_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = p938x_write_reg(chip, chip->peek_poke_address, temp);
	if (rc) {
		p938x_err(chip, "Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static void create_debugfs_entries(struct p938x_charger *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("p938x", NULL);
	if (!chip->debug_root) {
		p938x_err(chip, "Couldn't create debug dir\n");
		return;
	}

	ent = debugfs_create_file("dump_regs", S_IFREG | S_IRUGO,
				  chip->debug_root, chip,
				  &dump_regs_debugfs_ops);
	if (!ent)
		p938x_err(chip, "Couldn't create dump_regs debug file\n");

	ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
				  chip->debug_root,
				  &(chip->peek_poke_address));
	if (!ent)
		p938x_err(chip, "Couldn't create address debug file\n");

	ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
				  chip->debug_root, chip,
				  &poke_poke_debug_ops);
	if (!ent)
		p938x_err(chip, "Couldn't create data debug file\n");
}

static int p938x_parse_gpio(struct device_node *node, struct gpio *gpio, int idx)
{
	gpio->gpio = of_get_gpio_flags(node, idx,
				(enum of_gpio_flags *)&gpio->flags);
	of_property_read_string_index(node, "gpio-names", idx,
					&gpio->label);

	if (!gpio_is_valid(gpio->gpio))
		return 1;

	return 0;
}

static int p938x_parse_dt(struct p938x_charger *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	const char *fw_name;

	if (!node) {
		p938x_err(chip, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "idt,chg-efficiency",
					&chip->chg_efficiency);
	if (rc)
		chip->chg_efficiency = 85;

	rc = of_property_read_u32(node, "idt,chg-limit-soc",
					&chip->chg_limit_soc);
	if (rc)
		chip->chg_limit_soc = 95;

	rc = of_property_read_u32(node, "idt,chg-idle-cl",
					&chip->chg_idle_cl);
	if (rc)
		chip->chg_idle_cl = 50;

	if (of_find_property(node, "vdd-i2c-supply", NULL)) {
		chip->vdd_i2c_vreg = devm_regulator_get(chip->dev, "vdd-i2c");
		if (IS_ERR(chip->vdd_i2c_vreg))
			return PTR_ERR(chip->vdd_i2c_vreg);
	}

	if (of_gpio_count(node) < 0) {
		p938x_err(chip, "No GPIOS defined.\n");
		return -EINVAL;
	}

	if (p938x_parse_gpio(node, &chip->wchg_int_n, 0)) {
		p938x_err(chip, "Invalid gpio wchg_int_n=%d\n",
			chip->wchg_int_n.gpio);
		return -EINVAL;
	}

	if (p938x_parse_gpio(node, &chip->wchg_en_n, 1)) {
		p938x_err(chip, "Invalid gpio wchg_en_n=%d\n",
			chip->wchg_int_n.gpio);
		return -EINVAL;
	}

	if (p938x_parse_gpio(node, &chip->wchg_det, 2)) {
		p938x_err(chip, "Invalid gpio wchg_det=%d\n",
			chip->wchg_int_n.gpio);
		return -EINVAL;
	}

	if (p938x_parse_gpio(node, &chip->wchg_sleep, 3)) {
		p938x_err(chip, "Invalid gpio wchg_sleep=%d\n",
			chip->wchg_int_n.gpio);
		return -EINVAL;
	}

	if (p938x_parse_gpio(node, &chip->wchg_boost, 4)) {
		p938x_err(chip, "Invalid gpio wchg_boost=%d\n",
			chip->wchg_int_n.gpio);
		return -EINVAL;
	}

	chip->pinctrl_name = of_get_property(chip->dev->of_node,
						"pinctrl-names", NULL);

	fw_name = of_get_property(chip->dev->of_node,"fw-name", NULL);
	if(fw_name)
		strncpy(chip->fw_name, fw_name, NAME_MAX);

	return 0;
}

static enum power_supply_property p938x_wls_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
};

static int p938x_wls_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct p938x_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = p938x_is_tx_connected(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = p938x_is_ldo_on(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (p938x_is_chip_on(chip)) {
			rc = p938x_get_rx_iout(chip);
			if (rc >= 0)
				val->intval = rc * 1000;
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->rx_iout_max * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (p938x_is_chip_on(chip)) {
			rc = p938x_get_rx_vout(chip);
			if (rc >= 0)
				val->intval = rc * 1000;
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chip->rx_vout_max * 1000;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = POWER_SUPPLY_TYPE_WIRELESS;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !chip->charging_disabled;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->temp_level;
		break;
	default:
		return -EINVAL;
	}

	if (rc < 0) {
		p938x_dbg(chip, PR_MISC, "Couldn't get prop %d rc = %d\n",
					psp, rc);
		return rc;
	}

	return 0;
}

static int p938x_wls_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct p938x_charger *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if ((val->intval / 1000) > MAX_IOUT_MA_FAST)
			return -EINVAL;
		chip->wls_allowed_imax = val->intval / 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if ((val->intval / 1000) > MAX_VOUT_MV_FAST)
			return -EINVAL;
		chip->wls_allowed_vmax = val->intval / 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->charging_disabled = !val->intval;
		if (p938x_is_tx_connected(chip))
			rc = p938x_enable_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		if (chip->thermal_levels <= 0)
			break;
		if (val->intval > chip->thermal_levels)
			chip->temp_level = chip->thermal_levels - 1;
		else
			chip->temp_level = val->intval;
		break;
	default:
		return -EINVAL;
	}

	power_supply_changed(chip->wls_psy);
	return rc;
}

static int p938x_wls_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}

static void p938x_external_power_changed(struct power_supply *psy)
{
	// TODO
	/*struct p938x_charger *chip = power_supply_get_drvdata(psy);

	if (p938x_is_tx_connected(chip)) {
		schedule_work(&chip->chg_det_work);
		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				msecs_to_jiffies(0));
	}*/
}

static const struct regmap_config p938x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static int p938x_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct p938x_charger *chip;
	struct power_supply_config wls_psy_cfg = {};

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;
	chip->name = "WLS";
	chip->debug_mask = &__debug_mask;
	INIT_WORK(&chip->chg_det_work, p938x_chg_det_work);
	INIT_DELAYED_WORK(&chip->heartbeat_work, p938x_heartbeat_work);
	device_init_wakeup(chip->dev, true);

	chip->regmap = regmap_init_i2c(client, &p938x_regmap_config);
	if (IS_ERR(chip->regmap)) {
		pr_err("Couldn't initialize register regmap rc = %ld\n",
				PTR_ERR(chip->regmap));
		rc = PTR_ERR(chip->regmap);
		goto free_mem;
	}

	rc = p938x_parse_dt(chip);
	if (rc) {
		p938x_err(chip, "Couldn't parse DT nodes rc=%d\n", rc);
		goto free_mem;
	}

	i2c_set_clientdata(client, chip);
	dev_set_drvdata(chip->dev, chip);

	chip->wls_psy_d.name		= "idt_wireless";
	chip->wls_psy_d.type		= POWER_SUPPLY_TYPE_WIRELESS;
	chip->wls_psy_d.get_property	= p938x_wls_get_prop;
	chip->wls_psy_d.set_property	= p938x_wls_set_prop;
	chip->wls_psy_d.property_is_writeable =
					p938x_wls_prop_is_writeable;
	chip->wls_psy_d.properties	= p938x_wls_props;
	chip->wls_psy_d.num_properties	=
				ARRAY_SIZE(p938x_wls_props);
	chip->wls_psy_d.external_power_changed =
					p938x_external_power_changed;

	wls_psy_cfg.drv_data = chip;
	wls_psy_cfg.supplied_to = pm_wls_supplied_to;
	wls_psy_cfg.num_supplicants = ARRAY_SIZE(pm_wls_supplied_to);
	chip->wls_psy = power_supply_register(chip->dev,
			&chip->wls_psy_d,
			&wls_psy_cfg);
	if (IS_ERR(chip->wls_psy)) {
		p938x_err(chip, "Couldn't register wls psy rc=%ld\n",
				PTR_ERR(chip->wls_psy));
		rc = PTR_ERR(chip->wls_psy);
		goto free_mem;
	}
	chip->wls_psy->supplied_from = pm_wls_supplied_from;
	chip->wls_psy->num_supplies = ARRAY_SIZE(pm_wls_supplied_from);

	chip->irq_en = 0;
	chip->irq_stat = 0;
	chip->stat = 0;
	chip->rx_vout_max = 0;
	chip->rx_iout_max = 0;
	chip->wls_allowed_vmax = MAX_VOUT_MV_FAST;
	chip->wls_allowed_imax = MAX_IOUT_MA_FAST;
	chip->temp_level = 0;
	chip->thermal_levels = 0;
	memset(&chip->batt_st, 0, sizeof(struct battery_st));

	rc = p938x_hw_init(chip);
	if (rc < 0) {
		p938x_err(chip, "Failed to init hw, rc=%d\n", rc);
		goto free_psy;
	}

	rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			p938x_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"p938x_irq", chip);
	if (rc) {
		p938x_err(chip, "Failed irq=%d request rc = %d\n",
				client->irq, rc);
		goto free_psy;
	}

	rc = devm_request_threaded_irq(&client->dev, chip->wchg_det_irq, NULL,
			p938x_det_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"p938x_det_irq", chip);
	if (rc) {
		p938x_err(chip, "Failed irq=%d request rc = %d\n",
				chip->wchg_det_irq, rc);
		goto free_psy;
	}

	enable_irq_wake(chip->wchg_det_irq);

	create_debugfs_entries(chip);

	if (sysfs_create_groups(&chip->dev->kobj, p938x_groups))
		p938x_err(chip, "Failed to create sysfs attributes\n");

	/* Reset the chip to in case we inserted the module with a transmitter attached
	 * in order to force the right irqs to run
	 */
	p938x_reset(chip);

	pr_info("p938x wireless receiver initialized successfully\n");

	return 0;

free_psy:
	power_supply_unregister(chip->wls_psy);
free_mem:
	devm_kfree(chip->dev, chip);
	return rc;
}

static int p938x_charger_remove(struct i2c_client *client)
{
	struct p938x_charger *chip = i2c_get_clientdata(client);

	sysfs_remove_groups(&chip->dev->kobj, p938x_groups);
	cancel_delayed_work_sync(&chip->heartbeat_work);
	debugfs_remove_recursive(chip->debug_root);

	return 0;
}

static void p938x_shutdown(struct i2c_client *client)
{
	struct p938x_charger *chip = i2c_get_clientdata(client);

	p938x_dbg(chip, PR_MOTO, "Wireless charger shutdown\n");
}

static int p938x_suspend(struct device *dev)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	if (regulator_is_enabled(chip->vdd_i2c_vreg))
		return regulator_disable(chip->vdd_i2c_vreg);

	return 0;
}

static int p938x_resume(struct device *dev)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	if (!regulator_is_enabled(chip->vdd_i2c_vreg))
		return regulator_enable(chip->vdd_i2c_vreg);

	return 0;
}

static const struct dev_pm_ops p938x_pm_ops = {
	.suspend	= p938x_suspend,
	.resume		= p938x_resume,
};

static const struct of_device_id p938x_match_table[] = {
	{ .compatible = "idt,p938x-charger",},
	{ },
};

static const struct i2c_device_id p938x_charger_id[] = {
	{"p938x-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, p938x_charger_id);

static struct i2c_driver p938x_charger_driver = {
	.driver		= {
		.name		= "p938x-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= p938x_match_table,
		.pm		= &p938x_pm_ops,
	},
	.probe		= p938x_charger_probe,
	.remove		= p938x_charger_remove,
	.id_table	= p938x_charger_id,
	.shutdown	= p938x_shutdown,
};

module_i2c_driver(p938x_charger_driver);

MODULE_DESCRIPTION("p938x charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:p938x-charger");
