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
#include <linux/thermal.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/mmi_wake_lock.h>

#define CHIP_ID_REG			0x0000
#define HW_VER_REG			0x0002
#define CUST_ID_REG			0x0003
#define MTP_FW_MAJ_VER_REG	0x0004
#define MTP_FW_MIN_VER_REG	0x0006
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

#define SYS_MODE_REG		0x004a
#define SYS_CMD_REG		0x004e

#define EPP_QFACTOR_REG	0x0083
#define EPP_TX_GUARANTEED	0x0084
#define EPP_TX_POTENTIAL	0x0085

#define EPT_REASON_REG	0x00BE
#define FOD_CFG_REG		0x0070
#define TX_FOD_CFG_REG		0x00AC
#define SYS_TM_MODE_REG	0x0069
#define ST_TM_MODE_EN 3
#define ST_TM_MODE_DIS 0

#define ST_TX_CONFLICT	BIT(14)
#define ST_RX_CONN		BIT(13)
#define ST_EPT_TYPE_INT	BIT(12)
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
#define SYS_MODE_TX	        BIT(2)
#define SYS_MODE_WPCMODE	BIT(0)

#define CMD_TX_RM_POWER_TOGGLE	BIT(10)
#define CMD_RX_RENEGOTIATE	BIT(7)
#define CMD_RX_SWITCH_RAM	BIT(6)
#define CMD_RX_CLR_IRQ		BIT(5)
#define CMD_RX_SEND_CSP		BIT(4)
#define CMD_RX_SEND_EPT		BIT(3)
#define CMD_RX_CFG_TABLE	BIT(2)
#define CMD_RX_TOGGLE_LDO	BIT(1)
#define CMD_RX_SEND_RX_DATA	BIT(0)

#define EPT_POCP BIT(15)
#define EPT_OTP  BIT(14)
#define EPT_FOD  BIT(13)
#define EPT_LVP  BIT(12)
#define EPT_OVP  BIT(11)
#define EPT_OCP  BIT(10)
#define EPT_CMD  BIT(0)

#define WAIT_FOR_AUTH_MS 1000
#define WAIT_FOR_RCVD_TIMEOUT_MS 1000
#define HEARTBEAT_INTERVAL_MS 3000
#define HEARTBEAT_REPORT_INTERVAL 60000
#define TXMODEWORK_INTERVAL_MS 2000

#define BPP_MAX_VOUT 5000
#define BPP_MAX_IOUT 1600
#define BPP_MAX_IDC  1000
#define EPP_MAX_VOUT 12000
#define EPP_MAX_IOUT 1600
#define EPP_MAX_IDC  1250

#define MIN_VOUT_SET 5000
#define MAX_VOUT_SET 12000
#define MIN_IOUT_SET 500
#define MAX_IOUT_SET 3000

#define FOD_MAX_LEN 16
#define TX_FOD_WRITE_LEN 6
#define TX_FOD_REG_LEN 13

#define MAX_EPT_FOD_COUNT 3
#define MAX_EPT_POCP_COUNT 3
#define MAX_TX_MODE_RECOVER 10

#define WLS_SHOW_MAX_SIZE 32

#define MIN_CHIP_VERS 0x9380
#define MAX_CHIP_VERS 0x9389
#define CHIP_VENDOR "idt"

#define IDC_THERMAL_RESTORE_TIME 300000 /* 5 minutes */
#define IDC_THERMAL_BACKOFF_MA 100
#define IDC_THERMAL_MIN_CURRENT 400

#define FAST_CHARGE_MW 15000

#define DETACH_ON_READ_FAILURE

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

enum {
	TX_MODE_OVERHEAT = -1,
	TX_MODE_NOT_CONNECTED = 0,
	TX_MODE_POWER_SHARE = 2,
};

enum print_reason {
	PR_INTERRUPT    = BIT(0),
	PR_IMPORTANT	= BIT(1),
	PR_MISC         = BIT(2),
	PR_MOTO         = BIT(7),
	PR_FWPROG       = BIT(6)
};

static int __debug_mask = PR_IMPORTANT;
module_param_named(
	debug_mask, __debug_mask, int, S_IRUSR | S_IWUSR
);

struct p938x_limits {
	u32 tx_capability_mw;
	u32 idc_max_ma;
};

struct p938x_charger {
	const char		*name;
	int			*debug_mask;
	struct i2c_client	*client;
	struct device		*dev;
	struct regmap		*regmap;

	struct regulator	*vdd_i2c_vreg;
	struct gpio		wchg_int_n;
	struct gpio		wchg_en_n;
	struct gpio		wchg_det;
	struct gpio		wchg_sleep;
	struct gpio		wchg_boost;
	int			wchg_det_irq;
	struct pinctrl		*pinctrl_irq;
	const char		*pinctrl_name;
	long long unsigned error_cnt;

	u16 wls_vout_max;
	u16 wls_iout_max;
	int wls_idc_max;

	u16 stat;
	u16 irq_stat;

	struct delayed_work	heartbeat_work;
	struct delayed_work	tx_mode_work;
	struct delayed_work	removal_work;
	struct delayed_work	restore_idc_work;
	int heartbeat_count;

	u32			peek_poke_address;
	struct dentry		*debug_root;

	struct power_supply	*usb_psy;
	struct power_supply *wls_psy;
	struct power_supply *dc_psy;
	struct power_supply *batt_psy;

	bool	batt_hot;

	char			fw_name[NAME_MAX];
	int			program_fw_stat;

	unsigned long flags;

	struct wakeup_source *wls_wake_source;
	bool	epp_mode;

	u32 bpp_current_limit;
	struct p938x_limits *epp_current_limits;
	int epp_limit_num;
	u32 bpp_voltage;
	u32 epp_voltage;

	u8	fod_array_bpp[FOD_MAX_LEN];
	u8	fod_array_epp[FOD_MAX_LEN];
	u8	fod_array_tx[TX_FOD_WRITE_LEN];
	int	fod_array_bpp_len;
	int	fod_array_epp_len;
	int	fod_array_tx_len;
	bool 	use_q_factor;
	u8	q_factor;
	u8	ept_fod_count;
	u8 	ept_pocp_count;
	u8	tx_mode_recover_cnt;

	struct thermal_cooling_device *tcd;
	struct mutex disconnect_lock;
	struct mutex txmode_lock;
};

#define WLS_FLAG_BOOST_ENABLED 0
#define WLS_FLAG_TX_ATTACHED   1
#define WLS_FLAG_KEEP_AWAKE    2
#define WLS_FLAG_TX_MODE_EN    3
#define WLS_FLAG_USB_CONNECTED 4
#define WLS_FLAG_USB_KEEP_ON   5
#define WLS_FLAG_OVERHEAT      6

/* Send our notications to the battery */
static char *pm_wls_supplied_to[] = {
	"battery",
};

/* Get notifications from supplies */
static char *pm_wls_supplied_from[] = {
	"usb",
	"battery",
};

static inline int p938x_is_chip_on(struct p938x_charger *chip)
{
	return (test_bit(WLS_FLAG_BOOST_ENABLED, &chip->flags) ||
		test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags));
}

static inline int p938x_is_tx_connected(struct p938x_charger *chip)
{
	return test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags);
}

static inline int p938x_is_ldo_on(struct p938x_charger *chip)
{
	return (p938x_is_chip_on(chip) && (chip->stat & ST_VOUT_ON)
		&& gpio_get_value(chip->wchg_det.gpio));
}

static int p938x_read_reg(struct p938x_charger *chip, u16 reg, u8 *val)
{
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, reg, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

static int p938x_write_reg(struct p938x_charger *chip, u16 reg, u8 val)
{
	return regmap_write(chip->regmap, reg, val);
}

static void p938x_reset(struct p938x_charger *chip)
{
	p938x_write_reg(chip, 0x3040, 0x80);
	msleep(100);
}

static void p938x_pm_set_awake(struct p938x_charger *chip, int awake)
{
	if(!test_bit(WLS_FLAG_KEEP_AWAKE, &chip->flags) && awake) {
		PM_STAY_AWAKE(chip->wls_wake_source);
		set_bit(WLS_FLAG_KEEP_AWAKE, &chip->flags);
	} else if(test_bit(WLS_FLAG_KEEP_AWAKE, &chip->flags) && !awake) {
		PM_RELAX(chip->wls_wake_source);
		clear_bit(WLS_FLAG_KEEP_AWAKE, &chip->flags);
	}
}

static int p938x_get_rx_vrect(struct p938x_charger *chip);
static int p938x_set_dc_en_override(struct p938x_charger *chip, int en);

static void p938x_handle_wls_removal(struct p938x_charger *chip)
{
	if (!mutex_trylock(&chip->disconnect_lock)) {
		cancel_delayed_work(&chip->removal_work);
		schedule_delayed_work(&chip->removal_work, msecs_to_jiffies(500));
		return;
	}

	if(test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags)) {
		/* Make sure we aren't actually connect */
		if (gpio_get_value(chip->wchg_det.gpio))
			goto unlock;

		clear_bit(WLS_FLAG_TX_ATTACHED, &chip->flags);
		power_supply_changed(chip->wls_psy);
		cancel_delayed_work(&chip->heartbeat_work);

		chip->stat = 0;
		chip->irq_stat = 0;
		chip->heartbeat_count = 0;

		p938x_set_dc_en_override(chip, 0);

		if (!test_bit(WLS_FLAG_BOOST_ENABLED, &chip->flags))
			p938x_pm_set_awake(chip, 0);

		p938x_dbg(chip, PR_IMPORTANT, "Wireless charger is removed\n");
	}

unlock:
	mutex_unlock(&chip->disconnect_lock);
}

static void p938x_removal_work(struct work_struct *work)
{
	struct p938x_charger *chip = container_of(work,
				struct p938x_charger, removal_work.work);

	p938x_handle_wls_removal(chip);
}

static int p938x_read_buffer(struct p938x_charger *chip, u16 reg,
		u8 *buf, u32 size)
{
	int rc;

	rc = regmap_bulk_read(chip->regmap, reg, buf, size);
#ifdef DETACH_ON_READ_FAILURE
	if(rc)
		p938x_handle_wls_removal(chip);
#endif

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

	return rc;
}

static int p938x_get_rx_vout(struct p938x_charger *chip)
{
	int rc;
	u16 volt;

	if(!p938x_is_tx_connected(chip))
		return 0;

	rc = p938x_read_buffer(chip, VOUT_READ_REG, (u8 *)&volt, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read rx voltage, rc = %d\n", rc);
		return rc;
	}

	return volt;
}

static int p938x_get_rx_vrect(struct p938x_charger *chip)
{
	int rc;
	u16 volt;

	if(!p938x_is_tx_connected(chip))
		return 0;

	rc = p938x_read_buffer(chip, VRECT_READ_REG, (u8 *)&volt, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read rx voltage, rc = %d\n", rc);
		return rc;
	}

	return volt;
}

static int p938x_get_rx_vout_set(struct p938x_charger *chip)
{
	int rc;
	u8 volt;

	if(!p938x_is_chip_on(chip))
		return 0;

	rc = p938x_read_buffer(chip, VOUT_SET_REG, &volt, 1);
	if (rc < 0) {
		p938x_err(chip, "Failed to read rx voltage, rc = %d\n", rc);
		return rc;
	}

	return (volt * 100);
}

static int p938x_set_rx_vout(struct p938x_charger *chip, u16 mv)
{
	int rc;

	if(!p938x_is_chip_on(chip))
		return 1;

	if (mv < MIN_VOUT_SET)
		mv = MIN_VOUT_SET;
	else if (mv > MAX_VOUT_SET)
		mv = MAX_VOUT_SET;

	rc = p938x_write_reg(chip, VOUT_SET_REG, (u8)(mv / 100));
	if (rc < 0)
		p938x_err(chip, "Failed to set rx voltage, rc = %d\n", rc);
	else
		p938x_dbg(chip, PR_MOTO, "Set VOUT to %u mV\n", mv);

	return rc;
}

static int p938x_set_rx_ocl(struct p938x_charger *chip, u16 ma)
{
	int rc;

	if(!p938x_is_chip_on(chip))
		return 1;

	if (ma < MIN_IOUT_SET)
		ma = MIN_IOUT_SET;
	else if (ma > MAX_IOUT_SET)
		ma = MAX_IOUT_SET;

	rc = p938x_write_reg(chip, ILIMIT_SET_REG, (u8)((ma - 100) / 100));
	if (rc < 0)
		p938x_err(chip, "Failed to set rx current, rc = %d\n", rc);
	else
		p938x_dbg(chip, PR_MOTO, "Set ILIMIT to %u mA\n", ma);

	return rc;
}

static int p938x_get_rx_iout(struct p938x_charger *chip)
{
	int rc;
	u16 ma;

	if(!p938x_is_chip_on(chip))
		return 0;

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

	if(!p938x_is_chip_on(chip))
		return 0;

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

	if(!p938x_is_tx_connected(chip))
		return 1;

	rc = p938x_write_reg(chip, SYS_CMD_REG, CMD_RX_TOGGLE_LDO);
	if (rc < 0)
		p938x_err(chip, "Failed to %s RX ldo, rc = %d\n",
			on ? "enable":"disable", rc);
	else {
		p938x_dbg(chip, PR_MOTO, "RX ldo is %s\n",
			on ? "enabled":"disabled");
	}

	return rc;
}

static inline void p938x_set_boost(struct p938x_charger *chip, int val)
{
	/* Assume if we turned the boost on we want to stay awake */
	gpio_set_value(chip->wchg_boost.gpio, !!val);

	if(val) {
		set_bit(WLS_FLAG_BOOST_ENABLED, &chip->flags);
		p938x_pm_set_awake(chip, 1);
		gpio_set_value(chip->wchg_en_n.gpio, 0);
	} else {
		clear_bit(WLS_FLAG_BOOST_ENABLED, &chip->flags);
		if (!p938x_is_chip_on(chip))
			p938x_pm_set_awake(chip, 0);
	}
}

static inline int p938x_get_boost(struct p938x_charger *chip)
{
	return gpio_get_value(chip->wchg_boost.gpio);
}

static void p938x_toggle_power(struct p938x_charger *chip, int extra_delay_ms)
{
	p938x_set_boost(chip, 0);
	msleep(100 + extra_delay_ms);
	p938x_set_boost(chip, 1);
	msleep(100);
}

static int p938x_get_dc_now_icl_ma(struct p938x_charger *chip, unsigned int *idc);
static int p938x_set_dc_now_icl_ma(struct p938x_charger *chip, unsigned int idc);

static int p938x_update_supplies_status(struct p938x_charger *chip)
{
	int rc;
	union power_supply_propval prop = {0,};
	int wired_connection;
	unsigned int dc_current_now;
	unsigned int dc_current_new;

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
	}

	wired_connection = !!prop.intval;

	/* TODO For now disable wireless charging when a usb cable is connected */
	if (wired_connection) {
		set_bit(WLS_FLAG_USB_CONNECTED, &chip->flags);
		if(test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags) &&
				!test_bit(WLS_FLAG_USB_KEEP_ON, &chip->flags)) {
			gpio_set_value(chip->wchg_en_n.gpio, 1);
			/* Turn off boost too just in case */
			p938x_set_boost(chip, 0);
			p938x_dbg(chip, PR_MOTO, "usb and tx connected, disabled wls\n");
		}
	}
	else {
		clear_bit(WLS_FLAG_USB_CONNECTED, &chip->flags);
		gpio_set_value(chip->wchg_en_n.gpio, 0);
	}

	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		pr_debug("batt psy not found\n");
		return -EINVAL;
	}
	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_HOT_TEMP, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read batt temp zone prop, rc=%d\n", rc);
		return rc;
	}

	if (!chip->batt_hot && prop.intval) {
		/* If the battery is too hot, lower the current
		 * setting to try to keep us in optimal temp zone
		 */
		cancel_delayed_work(&chip->restore_idc_work);
		chip->batt_hot = 1;

		rc = p938x_get_dc_now_icl_ma(chip, &dc_current_now);
		if (rc) {
			p938x_err(chip, "Couldn't read dc current, rc=%d\n", rc);
			return rc;
		}

		dc_current_new = dc_current_now - IDC_THERMAL_BACKOFF_MA;
		if (dc_current_new < IDC_THERMAL_MIN_CURRENT)
			dc_current_new = IDC_THERMAL_MIN_CURRENT;

		p938x_set_dc_now_icl_ma(chip, dc_current_new);
		p938x_dbg(chip, PR_IMPORTANT, "Reduced dc current to %u mA due to hot battery\n",
			dc_current_new);
	} else if (chip->batt_hot && !prop.intval) {
		/* Set work to restore original current settings */
		chip->batt_hot = 0;
		cancel_delayed_work(&chip->restore_idc_work);
		schedule_delayed_work(&chip->restore_idc_work,
			msecs_to_jiffies(IDC_THERMAL_RESTORE_TIME));
	}

	return 0;
}

static void p938x_clear_irq(struct p938x_charger *chip, u16 mask)
{
	p938x_dbg(chip, PR_MOTO, "IRQ Clear 0x%02X\n", mask);
	p938x_write_buffer(chip, IRQ_CLEAR_REG, (u8 *)&mask, 2);
	p938x_write_reg(chip, SYS_CMD_REG, CMD_RX_CLR_IRQ);
}

static void p938x_heartbeat_work(struct work_struct *work)
{
	struct p938x_charger *chip = container_of(work,
				struct p938x_charger, heartbeat_work.work);
	int vout = 0;
	int iout = 0;
	int vout_set = 0;
	int iout_limit = 0;
	int vrect = 0;

	if (p938x_is_tx_connected(chip)) {
		/* Check if we disconnected */
		vrect = p938x_get_rx_vrect(chip);
		if (vrect <= 0) {
			p938x_err(chip, "Detected wireless charger is not attached.\n");
			p938x_handle_wls_removal(chip);
			return;
		}

		if (!p938x_is_ldo_on(chip)) {
			p938x_dbg(chip, PR_MOTO, "LDO is not on yet\n");
			/* This is where we will trigger it after the first VRECT_ON, so
			 * first indication to system we have a power supply attached is VRECT_ON
			 * + initial delay to hb
			 */
			power_supply_changed(chip->wls_psy);
			chip->heartbeat_count = 0;
			goto schedule_hb;
		}

		vout = p938x_get_rx_vout(chip);
		vout_set = p938x_get_rx_vout_set(chip);
		iout = p938x_get_rx_iout(chip);
		iout_limit = p938x_get_rx_ocl(chip);

		if (!chip->heartbeat_count) {
			p938x_dbg(chip, PR_IMPORTANT,
				"mode=%s vout=%d vout_set=%d vrect=%d iout=%d ocl=%d\n",
				chip->epp_mode ? "epp" : "bpp",
				vout, vout_set, vrect, iout, iout_limit);
		}

		chip->heartbeat_count++;
		if (HEARTBEAT_INTERVAL_MS * chip->heartbeat_count >=
				HEARTBEAT_REPORT_INTERVAL)
			chip->heartbeat_count = 0;

schedule_hb:
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

static int p938x_set_dc_psp_prop(struct p938x_charger *chip,
	enum power_supply_property psp,
	union power_supply_propval val)
{
	int rc = 1;

	if (chip->dc_psy) {
		rc = power_supply_set_property(chip->dc_psy, psp, &val);
		if (rc < 0) {
			p938x_err(chip, "Couldn't set dc prop %d, rc=%d\n", psp, rc);
		}
	}

	return rc;
}

static int p938x_set_dc_suspend(struct p938x_charger *chip, int en)
{
	union power_supply_propval val;

	val.intval = en;

	return p938x_set_dc_psp_prop(chip,
		POWER_SUPPLY_PROP_INPUT_SUSPEND,
		val);
}

static int p938x_set_dc_en_override(struct p938x_charger *chip, int en)
{
	union power_supply_propval val;

	val.intval = en;

	return p938x_set_dc_psp_prop(chip,
		POWER_SUPPLY_PROP_PIN_ENABLED,
		val);
}

static int p938x_set_dc_max_icl_ma(struct p938x_charger *chip, unsigned int idc)
{
	union power_supply_propval val;

	p938x_dbg(chip, PR_MOTO, "Setting IDC max to %u mA\n", idc);

	idc *= 1000; /* Convert to uA */
	val.intval = idc;

	return p938x_set_dc_psp_prop(chip,
		POWER_SUPPLY_PROP_CURRENT_MAX,
		val);
}

static int p938x_set_dc_now_icl_ma(struct p938x_charger *chip, unsigned int idc)
{
	union power_supply_propval val;

	p938x_dbg(chip, PR_MOTO, "Setting IDC to %u mA\n", idc);

	idc *= 1000; /* Convert to uA */
	val.intval = idc;

	return p938x_set_dc_psp_prop(chip,
		POWER_SUPPLY_PROP_CURRENT_NOW,
		val);
}

static int p938x_get_dc_now_icl_ma(struct p938x_charger *chip, unsigned int *idc)
{
	union power_supply_propval prop = {0,};
	int rc = 1;

	if (chip->dc_psy) {
		rc = power_supply_get_property(chip->dc_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		if (rc) {
			p938x_err(chip, "Couldn't read batt temp zone prop, rc=%d\n", rc);
			return rc;
		}

		*idc = prop.intval / 1000;
	}

	return rc;
}

static int p938x_set_dc_aicl_rerun(struct p938x_charger *chip)
{
	union power_supply_propval val;

	/* unused */
	val.intval = 1;

	return p938x_set_dc_psp_prop(chip,
		POWER_SUPPLY_PROP_RERUN_AICL,
		val);
}

static void p938x_restore_idc_work(struct work_struct *work)
{
	struct p938x_charger *chip = container_of(work,
			struct p938x_charger, restore_idc_work.work);

	p938x_dbg(chip, PR_IMPORTANT, "Running AICL to get back to max dc current\n");
	p938x_set_dc_aicl_rerun(chip);
}

static int p938x_program_tx_fod(struct p938x_charger *chip,
	u8 *array, int array_len)
{
	int rc;

	if (array_len <= 0 || array_len > TX_FOD_WRITE_LEN) {
		p938x_dbg(chip, PR_MOTO, "FOD length is not valid: %d\n", array_len);
		return 1;
	}

	rc = p938x_write_buffer(chip, TX_FOD_CFG_REG,
			array, array_len);
	if (rc < 0) {
		p938x_err(chip, "Failed to program fod: %d\n", rc);
		return 1;
	}

	return 0;
}

static int p938x_program_fod(struct p938x_charger *chip,
	u8 *array, int array_len)
{
	int rc;

	if (array_len <= 0 || array_len > FOD_MAX_LEN) {
		p938x_dbg(chip, PR_MOTO, "FOD length is not valid: %d\n", array_len);
		return 1;
	}

	rc = p938x_write_buffer(chip, FOD_CFG_REG,
			array, array_len);
	if (rc < 0) {
		p938x_err(chip, "Failed to program fod: %d\n", rc);
		return 1;
	}

	return 0;
}

static void p938x_configure_tx_mode(struct p938x_charger *chip)
{
	u16 buf = 0;
	int rc;

	p938x_dbg(chip, PR_MOTO, "Configuring tx mode\n");

	/* Configure the TX FOD parameters */
	p938x_program_tx_fod(chip, chip->fod_array_tx,
		chip->fod_array_tx_len);

	/* Set to disable on tx error */
	rc = p938x_read_buffer(chip, SYS_CMD_REG, (u8 *)&buf, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read 0x%04x, rc=%d\n",
			SYS_CMD_REG, rc);
		buf = 0;
	}
	buf = buf | CMD_TX_RM_POWER_TOGGLE;
	p938x_write_buffer(chip, SYS_CMD_REG, (u8 *)&buf, 2);
}

static int p938x_disable_tx_mode(struct p938x_charger *chip)
{
	int rc;
	u16 buf = 0;

	if (!test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags)) {
		p938x_dbg(chip, PR_MOTO, "Tx mode already disabled\n");
		return 1;
	}

	/* Make sure it won't accidently get re-enabled, there is a small
	 * window where there is a chance if we don't cancel this
	 */
	cancel_delayed_work_sync(&chip->tx_mode_work);

	rc = p938x_write_reg(chip, SYS_TM_MODE_REG, ST_TM_MODE_DIS);
	if (rc < 0)
		p938x_err(chip, "Failed to write 0x%04x(%d), rc=%d\n",
			SYS_TM_MODE_REG, ST_TM_MODE_EN, rc);

	rc = p938x_read_buffer(chip, SYS_CMD_REG, (u8 *)&buf, 2);
	if (rc < 0) {
		p938x_err(chip, "Failed to read 0x%04x, rc=%d\n",
			SYS_CMD_REG, rc);
		buf = 0;
	}

	buf = buf & ~ CMD_TX_RM_POWER_TOGGLE;
	p938x_write_buffer(chip, SYS_CMD_REG, (u8 *)&buf, 2);
	if (rc < 0)
		p938x_err(chip, "Failed to write 0x%04x, rc=%d\n",
			SYS_CMD_REG, rc);

	p938x_set_boost(chip, 0);
	p938x_set_dc_en_override(chip, 0);
	p938x_set_dc_suspend(chip, 0);

	clear_bit(WLS_FLAG_TX_MODE_EN, &chip->flags);
	sysfs_notify(&chip->dev->kobj, NULL, "tx_mode");

	return 0;
}

static inline void p938x_set_tx_mode(struct p938x_charger *chip, int val)
{
	int rc;

	if (chip->program_fw_stat == PROGRAM_FW_PENDING) {
		p938x_err(chip, "Tx mode request rejected, fw programming.\n");
		return;
	}

	mutex_lock(&chip->txmode_lock);

	if (test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags)) {
		p938x_err(chip, "Tx mode request rejected, charger is attached.\n");
		goto unlock;
	}

	if (val) {
		if (test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags)) {
			p938x_dbg(chip, PR_MOTO, "Tx mode already enabled\n");
			goto unlock;
		}

		if (test_bit(WLS_FLAG_OVERHEAT, &chip->flags)) {
			p938x_dbg(chip, PR_IMPORTANT, "Device too hot to enable tx mode\n");
			goto unlock;
		}

		/* Force dc in off so system doesn't see charger attached */
		/* TODO are both dc_en and dc suspend needed? */
		p938x_set_dc_suspend(chip, 1);
		p938x_set_dc_en_override(chip, 1);

		/* Power on and wait for boot */
		if(!p938x_is_chip_on(chip)) {
			p938x_set_boost(chip, 1);
			msleep(100);
		}

		rc = p938x_write_reg(chip, SYS_TM_MODE_REG, ST_TM_MODE_EN);
		if (rc < 0) {
			p938x_err(chip, "Failed to write 0x%04x(%d), rc=%d\n",
				SYS_TM_MODE_REG, ST_TM_MODE_EN, rc);
			p938x_set_boost(chip, 0);
			p938x_set_dc_en_override(chip, 0);
			p938x_set_dc_suspend(chip, 0);
		} else {
			p938x_dbg(chip, PR_MOTO, "tx mode enabled OK\n");
			set_bit(WLS_FLAG_TX_MODE_EN, &chip->flags);
			sysfs_notify(&chip->dev->kobj, NULL, "tx_mode");
			chip->ept_fod_count = 0;
			chip->ept_pocp_count = 0;
			chip->tx_mode_recover_cnt = 0;
			cancel_delayed_work(&chip->tx_mode_work);
			schedule_delayed_work(&chip->tx_mode_work,
				msecs_to_jiffies(TXMODEWORK_INTERVAL_MS));
		}
	} else {
		if (!p938x_disable_tx_mode(chip))
			p938x_dbg(chip, PR_MOTO, "tx mode disabled\n");
	}

unlock:
	mutex_unlock(&chip->txmode_lock);
}

static int p938x_tcd_get_max_state(struct thermal_cooling_device *tcd,
	unsigned long *state)
{
	*state = 1;

	return 0;
}

static int p938x_tcd_get_cur_state(struct thermal_cooling_device *tcd,
	unsigned long *state)
{
	struct p938x_charger *chip = tcd->devdata;

	*state = test_bit(WLS_FLAG_OVERHEAT, &chip->flags);

	return 0;
}

static int p938x_tcd_set_cur_state(struct thermal_cooling_device *tcd,
	unsigned long state)
{
	struct p938x_charger *chip = tcd->devdata;

	if (state && !test_bit(WLS_FLAG_OVERHEAT, &chip->flags)) {
		p938x_dbg(chip, PR_IMPORTANT, "Wireless charger overtemp\n");
		set_bit(WLS_FLAG_OVERHEAT, &chip->flags);
		p938x_set_tx_mode(chip, 0);
		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
	} else if (!state && test_bit(WLS_FLAG_OVERHEAT, &chip->flags)) {
		p938x_dbg(chip, PR_IMPORTANT, "Wireless charger temp OK\n");
		clear_bit(WLS_FLAG_OVERHEAT, &chip->flags);
		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
	}

	return 0;
}

static const struct thermal_cooling_device_ops p938x_tcd_ops = {
	.get_max_state = p938x_tcd_get_max_state,
	.get_cur_state = p938x_tcd_get_cur_state,
	.set_cur_state = p938x_tcd_set_cur_state,
};

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

	return rc;
}

static int p938x_program_fw(struct p938x_charger *chip)
{
	int rc = 0;
	const struct firmware *fw = NULL;

	if (!strlen(chip->fw_name)) {
		p938x_err(chip, "No FW name given\n");
		return -EINVAL;
	}

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

	mutex_lock(&chip->disconnect_lock);
	mutex_lock(&chip->txmode_lock);

	/* Make sure tx mode is off or we'll fail */
	p938x_disable_tx_mode(chip);
	msleep(100);

	/* Turn on the boost so i2c works */
	p938x_set_boost(chip, 1);

	/* Brief delay for ic init */
	msleep(100);

	p938x_dbg(chip, PR_FWPROG, "Loading FW programmer...\n");

	rc = p938x_program_mtp_downloader(chip);
	if (rc < 0) {
		p938x_err(chip, "Programming MTP downloader failed, rc=%d\n",
			rc);
		chip->program_fw_stat = PROGRAM_FW_FAIL;
		goto release_fw;
	}

	p938x_dbg(chip, PR_FWPROG, "Starting FW programming...\n");

	rc = p938x_program_mtp(chip, fw->data, fw->size);
	if (rc < 0) {
		p938x_err(chip, "Programming MTP failed, rc=%d\n", rc);
		chip->program_fw_stat = PROGRAM_FW_FAIL;
		goto release_fw;
	}

	chip->program_fw_stat = PROGRAM_FW_SUCCESS;
	p938x_dbg(chip, PR_IMPORTANT, "Programming FW success\n");

release_fw:
	mutex_unlock(&chip->txmode_lock);
	mutex_unlock(&chip->disconnect_lock);
	release_firmware(fw);
	p938x_set_boost(chip, 0);
	return rc;
}

static int p938x_get_tx_capability(struct p938x_charger *chip, int *guaranteed,
	int *potential)
{
	u8 tmp;

	if (!p938x_read_reg(chip, EPP_TX_POTENTIAL, &tmp))
		*potential = 500 * tmp;
	else
		return 1;

	if (!p938x_read_reg(chip, EPP_TX_GUARANTEED, &tmp))
		*guaranteed = 500 * tmp;
	else
		return 1;

	return 0;
}

static void p938x_check_ept_reason(struct p938x_charger *chip)
{
	u16 ept_reason = 0;
	int rc;

	/* Set to disable on tx error */
	rc = p938x_read_buffer(chip, EPT_REASON_REG, (u8 *)&ept_reason, 2);
	if (rc < 0) {
		p938x_err(chip,
			"Failed to read 0x%04x. Shutting off tx mode, rc=%d\n",
			EPT_REASON_REG, rc);
		p938x_set_tx_mode(chip, 0);
	}

	cancel_delayed_work_sync(&chip->tx_mode_work);

	if (ept_reason & EPT_POCP) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_POCP\n");
		chip->ept_pocp_count++;
		if(chip->ept_pocp_count == MAX_EPT_POCP_COUNT) {
			p938x_set_tx_mode(chip, 0);
			p938x_err(chip, "EPT_POCP triggered %d times. Disabled tx mode.\n",
				MAX_EPT_POCP_COUNT);
		} else
			p938x_toggle_power(chip, 1000);
	}

	if (ept_reason & EPT_OTP) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_OTP\n");
		p938x_set_tx_mode(chip, 0);
		p938x_err(chip, "EPT_OTP triggered. Disabled tx mode.\n");
	}

	if (ept_reason & EPT_FOD) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_FOD\n");
		chip->ept_fod_count++;
		if(chip->ept_fod_count == MAX_EPT_FOD_COUNT) {
			p938x_set_tx_mode(chip, 0);
			p938x_err(chip, "EPT_FOD triggered %d times. Disabled tx mode.\n",
				MAX_EPT_FOD_COUNT);
		} else
			p938x_toggle_power(chip, 5000);
	}

	if (ept_reason & EPT_LVP) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_LVP\n");
		p938x_toggle_power(chip, 1000);
	}

	if (ept_reason & EPT_OVP) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_OVP\n");
		p938x_toggle_power(chip, 1000);
	}

	if (ept_reason & EPT_OCP) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_OCP\n");
		p938x_toggle_power(chip, 2000);
	}

	if (ept_reason & EPT_CMD) {
		p938x_dbg(chip, PR_INTERRUPT, "EPT: EPT_CMD\n");
		p938x_toggle_power(chip, 1000);
	}

	schedule_delayed_work(&chip->tx_mode_work,
		msecs_to_jiffies(0));
}

static int p938x_check_system_mode(struct p938x_charger *chip)
{
	int rc;
	int i;
	u8 mode = 0;
	u8 tx_mode = 0;
	int tx_guaranteed;
	int tx_potential;
	struct p938x_limits epp_limit;

	rc = p938x_read_buffer(chip, SYS_MODE_REG, &mode, 1);
	if (rc < 0)
		return rc;

	rc = p938x_read_reg(chip, SYS_TM_MODE_REG, &tx_mode);
	if (rc < 0)
		return rc;

	p938x_dbg(chip, PR_MOTO, "MODE=0x%02x\n", mode);

	if (mode & SYS_MODE_RAMCODE)
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_RAMCODE\n");
	if (mode & SYS_MODE_TX) {
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_TXMODE\n");
		p938x_configure_tx_mode(chip);
	}
	if (mode & SYS_MODE_WPCMODE) {
		p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_WPCMODE\n");
		if (mode & SYS_MODE_EXTENDED) {
			p938x_dbg(chip, PR_INTERRUPT, "MODE: SYS_MODE_EXTENDED\n");
			chip->epp_mode = true;

			if (chip->epp_limit_num &&
					!p938x_get_tx_capability(chip, &tx_guaranteed, &tx_potential)) {
				p938x_dbg(chip, PR_INTERRUPT, "EPP GUARANTEED %dmW, POTENTIAL %dmW\n",
					tx_guaranteed, tx_potential);

				/* Default value is the lowest setting, start at the highest setting
				 * and go backwards, if we find one more appropriate use that one
				 * instead
				 */
				epp_limit = chip->epp_current_limits[0];
				for(i=chip->epp_limit_num-1; i >= 0; i--) {
					if(tx_guaranteed >= chip->epp_current_limits[i].tx_capability_mw) {
						epp_limit = chip->epp_current_limits[i];
						break;
					}
				}

				p938x_dbg(chip, PR_INTERRUPT, "EPP LIMIT is %d mW -> %d mA\n",
					epp_limit.tx_capability_mw, epp_limit.idc_max_ma);

				p938x_set_dc_max_icl_ma(chip, epp_limit.idc_max_ma);
			} else
				p938x_set_dc_max_icl_ma(chip, EPP_MAX_IDC);

			p938x_set_rx_vout(chip, chip->epp_voltage);
			p938x_set_rx_ocl(chip, EPP_MAX_IOUT);
			p938x_program_fod(chip, chip->fod_array_epp,
				chip->fod_array_epp_len);
		} else {
			chip->epp_mode = false;
			p938x_set_dc_max_icl_ma(chip, chip->bpp_current_limit);
			p938x_set_rx_vout(chip, chip->bpp_voltage);
			p938x_set_rx_ocl(chip, BPP_MAX_IOUT);
			p938x_program_fod(chip, chip->fod_array_bpp,
				chip->fod_array_bpp_len);
		}

		/* Override if set */
		if(chip->wls_vout_max)
			p938x_set_rx_vout(chip, chip->wls_vout_max);
		if(chip->wls_iout_max)
			p938x_set_rx_ocl(chip, chip->wls_iout_max);
		if(chip->wls_idc_max)
			p938x_set_dc_max_icl_ma(chip, chip->wls_idc_max);
	}

	return 0;
}

static int p938x_get_status(struct p938x_charger *chip, u16 *stat, u16 *irq_stat)
{
	int rc;
	u16 irq_en;
	u32 stat_and_irq_stat;

	rc = p938x_read_buffer(chip, IRQ_ENABLE_REG, (u8 *)&irq_en, 2);
	if (rc < 0)
		return rc;

	/* Read both DEV_STATUS_REG and IRQ_STATUS_REG together */
	rc = p938x_read_buffer(chip, DEV_STATUS_REG, (u8 *)&stat_and_irq_stat, 4);
	if (rc < 0)
		return rc;

	p938x_dbg(chip, PR_MOTO, "IRQ_ENABLE=0x%04x, IRQ_ST=0x%04x, STATUS=0x%04x\n",
		irq_en, ((u16 *)(&stat_and_irq_stat))[1], ((u16 *)(&stat_and_irq_stat))[0]);

	if(stat)
		*stat = ((u16 *)(&stat_and_irq_stat))[0];
	if(irq_stat)
		*irq_stat = ((u16 *)(&stat_and_irq_stat))[1];

	return rc;
}

static void p938x_debug_status(struct p938x_charger *chip, u16 status)
{
	if (status & ST_TX_CONFLICT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_TX_CONFLICT\n");
	if (status & ST_RX_CONN)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_RX_CONN\n");
	if (status & ST_EPT_TYPE_INT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_EPT_TYPE_INT\n");
	if (status & ST_ADT_ERR)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_ADT_ERR\n");
	if (status & ST_ADT_RCV)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_ADT_RCV\n");
	if (status & ST_ADT_SENT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_ADT_SENT\n");
	if (status & ST_VOUT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_VOUT_ON\n");
	if (status & ST_VRECT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_VRECT_ON\n");
	if (status & ST_MODE_CHANGE)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_MODE_CHANGE\n");
	if (status & ST_OVER_TEMP)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_OVER_TEMP\n");
	if (status & ST_OVER_VOLT)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_OVER_VOLT\n");
	if (status & ST_OVER_CURR)
		p938x_dbg(chip, PR_INTERRUPT, "STATUS: ST_OVER_CURR\n");
}

static void p938x_debug_irq(struct p938x_charger *chip, u16 irq_status)
{
	if (irq_status & ST_TX_CONFLICT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_TX_CONFLICT\n");
	if (irq_status & ST_RX_CONN)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_RX_CONN\n");
	if (irq_status & ST_EPT_TYPE_INT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_EPT_TYPE_INT\n");
	if (irq_status & ST_ADT_ERR)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_ADT_ERR\n");
	if (irq_status & ST_ADT_RCV)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_ADT_RCV\n");
	if (irq_status & ST_ADT_SENT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_ADT_SENT\n");
	if (irq_status  & ST_VOUT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_VOUT_ON\n");
	if (irq_status & ST_VRECT_ON)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_VRECT_ON\n");
	if (irq_status & ST_MODE_CHANGE)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_MODE_CHANGE\n");
	if (irq_status & ST_OVER_TEMP)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_OVER_TEMP\n");
	if (irq_status & ST_OVER_VOLT)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_OVER_VOLT\n");
	if (irq_status & ST_OVER_CURR)
		p938x_dbg(chip, PR_INTERRUPT, "IRQ: ST_OVER_CURR\n");
}

static int p938x_check_status(struct p938x_charger *chip)
{
	u16 status;
	u16 irq_status;

	if (p938x_get_status(chip, &status, &irq_status)) {
		p938x_dbg(chip, PR_MOTO, "Could not read status registers");
		return 1;
	}

	chip->stat = status;
	chip->irq_stat = irq_status;

	p938x_debug_status(chip, status);
	p938x_debug_irq(chip, irq_status);

	if (irq_status & status & ST_TX_CONFLICT) {
		p938x_set_tx_mode(chip, 0);
		p938x_dbg(chip, PR_IMPORTANT, "Tx mode conflict. Disabled tx mode\n");
	}

	if ((irq_status & ST_EPT_TYPE_INT) &&
			test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags))
		p938x_check_ept_reason(chip);

	if (test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags)) {
		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");
	}

	if ((irq_status & status & ST_VRECT_ON) ||
		(!test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags) && (status & ST_VRECT_ON))) {
		if (test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags)) {
			p938x_dbg(chip, PR_IMPORTANT, "Connected to tx pad. Disabling tx mode\n");
			p938x_set_tx_mode(chip, 0);
		}

		if (chip->use_q_factor)
			p938x_write_reg(chip, EPP_QFACTOR_REG, chip->q_factor);

		set_bit(WLS_FLAG_TX_ATTACHED, &chip->flags);
		/* Call p938x_pm_set_awake before update supplies status.  Update supplies
		 * status will turn it back off if we have a cable attached
		 */
		p938x_pm_set_awake(chip, 1);
		/* Update usb status incase we powered on with it connected */
		p938x_update_supplies_status(chip);

		cancel_delayed_work(&chip->heartbeat_work);
		/* The power_supply_changed call is in the heartbeat.  This delay is how long
		 * we wait for a disconnect before changing the state to charging.  Without this
		 * we might report we are charging even if the connection is not stable enough
		 * to charge
		 */
		schedule_delayed_work(&chip->heartbeat_work,
				msecs_to_jiffies(1500));
		p938x_dbg(chip, PR_IMPORTANT, "Wireless charger is inserted\n");
	}

	if (irq_status & status & ST_VOUT_ON) {
		p938x_dbg(chip, PR_IMPORTANT, "Wireless charger ldo is on\n");
		p938x_check_system_mode(chip);
		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				msecs_to_jiffies(0));
		power_supply_changed(chip->wls_psy);
	}

	if ((irq_status & ST_MODE_CHANGE) &&
			test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags)) {
		p938x_dbg(chip, PR_IMPORTANT, "Wireless mode changed\n");
		if(chip->tx_mode_recover_cnt >= MAX_TX_MODE_RECOVER) {
			p938x_err(chip, "Tx mode recovered too many times. Turning off.\n");
			p938x_set_tx_mode(chip, 0);
		} else
			p938x_check_system_mode(chip);
	}

	p938x_clear_irq(chip, irq_status);

	return 0;
}

/* Sometimes the tx mode turns off on its own... make sure the user doesn't
 * notice
 */

static void p938x_tx_mode_work(struct work_struct *work)
{
	u8 buf;
	int rc;
	struct p938x_charger *chip = container_of(work,
				struct p938x_charger, tx_mode_work.work);

	if (test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags)) {
		rc = p938x_read_reg(chip, SYS_TM_MODE_REG, &buf);
		if (rc >= 0) {
			if (buf != ST_TM_MODE_EN) {
				p938x_dbg(chip, PR_INTERRUPT,
					"Tx mode is off and it should be on, turning back on.\n");
				chip->tx_mode_recover_cnt++;

				rc = p938x_write_reg(chip, SYS_TM_MODE_REG,
										ST_TM_MODE_EN);
				if (rc < 0) {
					p938x_err(chip, "Failed to write 0x%04x(%d), rc=%d\n",
						SYS_TM_MODE_REG, ST_TM_MODE_EN, rc);
				}
			}
		} else {
			/* If we are in tx mode and we can't read the register, something is wrong
			 * so force a reset
			 */
			p938x_err(chip, "Something went wrong, resetting IC...");
			p938x_toggle_power(chip, 0);
		}

		sysfs_notify(&chip->dev->kobj, NULL, "rx_connected");

		schedule_delayed_work(&chip->tx_mode_work,
			msecs_to_jiffies(TXMODEWORK_INTERVAL_MS));
	}
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

	mutex_lock(&chip->disconnect_lock);

	if (p938x_check_status(chip)) {
		p938x_err(chip,
			"Error checking status. Check charging pad alignment\n");
		chip->error_cnt++;
		sysfs_notify(&chip->dev->kobj, NULL, "error_cnt");
	}

	mutex_unlock(&chip->disconnect_lock);

	return IRQ_HANDLED;
}

static irqreturn_t p938x_det_irq_handler(int irq, void *dev_id)
{
	struct p938x_charger *chip = dev_id;
	int tx_detected = gpio_get_value(chip->wchg_det.gpio);

	if (tx_detected) {
		p938x_dbg(chip, PR_IMPORTANT, "Detected an attach event.\n");
		/* Force dcin-en on */
		p938x_set_dc_en_override(chip, 2);
	} else {
		p938x_dbg(chip, PR_IMPORTANT, "Detected a detach event.\n");
		p938x_handle_wls_removal(chip);
	}

	return IRQ_HANDLED;
}

/* Only allow usb_keep_on and boost in userdebug builds */
#ifdef P938X_USER_DEBUG
static ssize_t usb_keep_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long value;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &value);
	if (r) {
		p938x_err(chip, "Invalid usb keep on value = %ld\n", value);
		return -EINVAL;
	}

	if (value) {
		set_bit(WLS_FLAG_USB_KEEP_ON, &chip->flags);
		if (test_bit(WLS_FLAG_USB_CONNECTED, &chip->flags))
			gpio_set_value(chip->wchg_en_n.gpio, 0);
	} else {
		clear_bit(WLS_FLAG_USB_KEEP_ON, &chip->flags);
		if (test_bit(WLS_FLAG_USB_CONNECTED, &chip->flags) &&
				test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags))
			gpio_set_value(chip->wchg_en_n.gpio, 1);
	}

	return r ? r : count;
}

static ssize_t usb_keep_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n",
		test_bit(WLS_FLAG_USB_KEEP_ON, &chip->flags));
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

static ssize_t force_idc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long value;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &value);
	if (r) {
		p938x_err(chip, "Invalid idc value = %ld\n", value);
		return -EINVAL;
	}

	/* Store to persist change */
	chip->wls_idc_max = value;

	p938x_set_dc_now_icl_ma(chip, value);
	p938x_set_dc_max_icl_ma(chip, value);

	return r ? r : count;
}

static ssize_t force_idc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;
	union power_supply_propval prop = {0,};
	struct p938x_charger *chip = dev_get_drvdata(dev);

	rc = power_supply_get_property(chip->dc_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc) {
		p938x_err(chip, "Couldn't read IDC prop, rc=%d\n", rc);
		return scnprintf(buf, WLS_SHOW_MAX_SIZE, "Unknown\n");
	}

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n",
		prop.intval / 1000);
}

static int fod_store(struct p938x_charger *chip, const char *buf,
	u8 *fod_array, u32 *fod_array_len)
{
	int i = 0, ret = 0, sum = 0;
	char *buffer;
	unsigned int temp;
	u8 fod_array_temp[FOD_MAX_LEN];

	buffer = (char *)buf;

	for (i = 0; i < FOD_MAX_LEN; i++) {
		ret = sscanf((const char *)buffer, "%x,%s", &temp, buffer);
		fod_array_temp[i] = temp;
		sum++;
		if (ret != 2)
			break;
	}

	if (sum > 0 && sum <= FOD_MAX_LEN) {
		memcpy(fod_array, fod_array_temp, sum);
		*fod_array_len = sum;
		p938x_program_fod(chip, fod_array, *fod_array_len);
	}

	return sum;
}

static ssize_t fod_epp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	fod_store(chip, buf, chip->fod_array_epp,
		&chip->fod_array_epp_len);

	return count;
}

static ssize_t fod_bpp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	fod_store(chip, buf, chip->fod_array_bpp,
		&chip->fod_array_bpp_len);

	return count;
}

static ssize_t fod_tx_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	int i = 0, ret = 0, sum = 0;
	char *buffer;
	unsigned int temp;
	u8 fod_array_temp[TX_FOD_WRITE_LEN];

	buffer = (char *)buf;

	for (i = 0; i < TX_FOD_WRITE_LEN; i++) {
		ret = sscanf((const char *)buffer, "%x,%s", &temp, buffer);
		fod_array_temp[i] = temp;
		sum++;
		if (ret != 2)
			break;
	}

	if (sum > 0 && sum <= TX_FOD_WRITE_LEN) {
		memcpy(chip->fod_array_tx, fod_array_temp, sum);
		chip->fod_array_tx_len = sum;
		p938x_program_tx_fod(chip, chip->fod_array_tx, chip->fod_array_tx_len);
	}

	return count;
}

static ssize_t fod_epp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	for (i = 0; i < chip->fod_array_epp_len; i++) {
		count += scnprintf(buf+count, WLS_SHOW_MAX_SIZE,
				"0x%02x ", chip->fod_array_epp[i]);
		if (i == chip->fod_array_epp_len - 1)
			count += scnprintf(buf+count, WLS_SHOW_MAX_SIZE, "\n");
	}

	return count;
}

static ssize_t fod_bpp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	for (i = 0; i < chip->fod_array_bpp_len; i++) {
		count += scnprintf(buf+count, WLS_SHOW_MAX_SIZE,
				"0x%02x ", chip->fod_array_bpp[i]);
		if (i == chip->fod_array_bpp_len - 1)
			count += scnprintf(buf+count, WLS_SHOW_MAX_SIZE, "\n");
	}

	return count;
}

static ssize_t fod_tx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	for (i = 0; i < chip->fod_array_tx_len; i++) {
		count += scnprintf(buf+count, WLS_SHOW_MAX_SIZE,
				"0x%02x ", chip->fod_array_tx[i]);
		if (i == chip->fod_array_tx_len - 1)
			count += scnprintf(buf+count, WLS_SHOW_MAX_SIZE, "\n");
	}

	return count;
}

static ssize_t q_factor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long value;
	struct p938x_charger *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &value);
	if (r) {
		p938x_err(chip, "Invalid q factor value = %ld\n", value);
		return -EINVAL;
	}

	chip->q_factor = value;
	chip->use_q_factor = true;

	return r ? r : count;
}

static ssize_t q_factor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	if (chip->use_q_factor)
		return scnprintf(buf, WLS_SHOW_MAX_SIZE, "0x%02X\n",
			chip->q_factor);
	else
		return scnprintf(buf, WLS_SHOW_MAX_SIZE,
			"Using default\n");
}
#endif

static ssize_t tx_mode_store(struct device *dev,
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

	p938x_set_tx_mode(chip, value);

	return r ? r : count;
}

static ssize_t tx_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n",
		test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags));
}

static ssize_t chip_id_max_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "0x%04x\n",
		MAX_CHIP_VERS);
}

static ssize_t chip_id_min_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "0x%04x\n",
		MIN_CHIP_VERS);
}

static ssize_t chip_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%s\n",
		CHIP_VENDOR);
}

static ssize_t chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	int turn_off = 0;
	u8 data[2];

	if(!p938x_is_chip_on(chip)) {
		turn_off = 1;
		p938x_set_boost(chip, 1);
		msleep(100);
	}

	p938x_read_buffer(chip, CHIP_ID_REG, data, 2);

	if(turn_off)
		p938x_set_boost(chip, 0);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "0x%02x%02x\n",
		data[1], data[0]);
}

static ssize_t fw_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	int turn_off = 0;
	u16 minor_vers = 0x00;
	u16 major_vers = 0x00;

	if(!p938x_is_chip_on(chip)) {
		turn_off = 1;
		p938x_set_boost(chip, 1);
		msleep(100);
	}

	p938x_read_buffer(chip, MTP_FW_MAJ_VER_REG, (u8 *)&major_vers, 2);
	p938x_read_buffer(chip, MTP_FW_MIN_VER_REG, (u8 *)&minor_vers, 2);

	if(turn_off)
		p938x_set_boost(chip, 0);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%04x%04x\n",
		major_vers, minor_vers);
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

static ssize_t tx_capability_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	int tx_potential;
	int tx_guaranteed;

	if (!test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags))
		return scnprintf(buf, WLS_SHOW_MAX_SIZE, "No charger attached\n");

	if (!p938x_is_ldo_on(chip))
		return scnprintf(buf, WLS_SHOW_MAX_SIZE, "Unknown\n");

	if (chip->epp_mode) {
		if (!p938x_get_tx_capability(chip, &tx_guaranteed, &tx_potential))
			return scnprintf(buf, WLS_SHOW_MAX_SIZE, "EPP %dmW (Potential %dmW)\n",
				tx_guaranteed, tx_potential);
		else
			return scnprintf(buf, WLS_SHOW_MAX_SIZE, "EPP Unknown\n");
	} else
		return scnprintf(buf, WLS_SHOW_MAX_SIZE, "BPP\n");
}

static ssize_t rx_connected_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);
	int rx_connected;

	if (test_bit(WLS_FLAG_OVERHEAT, &chip->flags))
		rx_connected = TX_MODE_OVERHEAT;
	else if (test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags) &&
			(chip->stat & ST_RX_CONN) &&
			p938x_get_rx_iout(chip))
		rx_connected = TX_MODE_POWER_SHARE;
	else
		rx_connected = TX_MODE_NOT_CONNECTED;

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%d\n",
		rx_connected);
}

static ssize_t error_cnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct p938x_charger *chip = dev_get_drvdata(dev);

	return scnprintf(buf, WLS_SHOW_MAX_SIZE, "%llu\n",
		chip->error_cnt);
}

#ifdef P938X_USER_DEBUG
static DEVICE_ATTR(usb_keep_on, S_IRUGO|S_IWUSR, usb_keep_on_show, usb_keep_on_store);
static DEVICE_ATTR(boost, S_IRUGO|S_IWUSR, boost_show, boost_store);
static DEVICE_ATTR(force_idc, S_IRUGO|S_IWUSR, force_idc_show, force_idc_store);
static DEVICE_ATTR(fod_epp, S_IRUGO|S_IWUSR, fod_epp_show, fod_epp_store);
static DEVICE_ATTR(fod_bpp, S_IRUGO|S_IWUSR, fod_bpp_show, fod_bpp_store);
static DEVICE_ATTR(fod_tx, S_IRUGO|S_IWUSR, fod_tx_show, fod_tx_store);
static DEVICE_ATTR(q_factor, S_IRUGO|S_IWUSR, q_factor_show, q_factor_store);
#endif
static DEVICE_ATTR(tx_mode, S_IRUGO|S_IWUSR, tx_mode_show, tx_mode_store);
static DEVICE_ATTR(chip_id, S_IRUGO, chip_id_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, chip_vendor_show, NULL);
static DEVICE_ATTR(chip_id_max, S_IRUGO, chip_id_max_show, NULL);
static DEVICE_ATTR(chip_id_min, S_IRUGO, chip_id_min_show, NULL);
static DEVICE_ATTR(fw_ver, S_IRUGO, fw_ver_show, NULL);
static DEVICE_ATTR(fw_name, S_IWUSR, NULL, fw_name_store);
static DEVICE_ATTR(program_fw_stat, S_IRUGO, program_fw_stat_show, NULL);
static DEVICE_ATTR(program_fw, S_IWUSR, NULL, program_fw_store);
static DEVICE_ATTR(tx_capability, S_IRUGO, tx_capability_show, NULL);
static DEVICE_ATTR(rx_connected, S_IRUGO, rx_connected_show, NULL);
static DEVICE_ATTR(error_cnt, S_IRUGO, error_cnt_show, NULL);

static struct attribute *p938x_attrs[] = {
#ifdef P938X_USER_DEBUG
	&dev_attr_usb_keep_on.attr,
	&dev_attr_boost.attr,
	&dev_attr_force_idc.attr,
	&dev_attr_fod_epp.attr,
	&dev_attr_fod_bpp.attr,
	&dev_attr_fod_tx.attr,
	&dev_attr_q_factor.attr,
#endif
	&dev_attr_tx_mode.attr,
	&dev_attr_chip_id_max.attr,
	&dev_attr_chip_id_min.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_vendor.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_fw_name.attr,
	&dev_attr_program_fw_stat.attr,
	&dev_attr_program_fw.attr,
	&dev_attr_tx_capability.attr,
	&dev_attr_rx_connected.attr,
	&dev_attr_error_cnt.attr,
	NULL
};

ATTRIBUTE_GROUPS(p938x);

static void show_dump_flags(struct seq_file *m,
	struct p938x_charger *chip)
{
	seq_printf(m, "WLS_FLAG_BOOST_ENABLED: %d\n",
		test_bit(WLS_FLAG_BOOST_ENABLED, &chip->flags));
	seq_printf(m, "WLS_FLAG_KEEP_AWAKE: %d\n",
		test_bit(WLS_FLAG_KEEP_AWAKE, &chip->flags));
	seq_printf(m, "WLS_FLAG_TX_ATTACHED: %d\n",
		test_bit(WLS_FLAG_TX_ATTACHED, &chip->flags));
	seq_printf(m, "WLS_FLAG_TX_MODE_EN: %d\n",
		test_bit(WLS_FLAG_TX_MODE_EN, &chip->flags));
	seq_printf(m, "WLS_FLAG_USB_CONNECTED: %d\n",
		test_bit(WLS_FLAG_USB_CONNECTED, &chip->flags));
	seq_printf(m, "WLS_FLAG_USB_KEEP_ON: %d\n",
		test_bit(WLS_FLAG_USB_KEEP_ON, &chip->flags));
	seq_printf(m, "WLS_FLAG_OVERHEAT: %d\n",
		test_bit(WLS_FLAG_OVERHEAT, &chip->flags));
}

static int show_dump_regs(struct seq_file *m, void *data)
{
	struct p938x_charger *chip = m->private;
	u8 buf[16];

	p938x_read_buffer(chip, CHIP_ID_REG, buf, 2);
	seq_printf(m, "CHIP_ID: 0x%02x%02x\n", buf[1], buf[0]);
	p938x_read_reg(chip, HW_VER_REG, &buf[0]);
	seq_printf(m, "HW_VER: 0x%02x\n", buf[0]);
	p938x_read_reg(chip, CUST_ID_REG, &buf[0]);
	seq_printf(m, "CUST_ID: 0x%02x\n", buf[0]);
	p938x_read_buffer(chip, MTP_FW_MAJ_VER_REG, buf, 4);
	seq_printf(m, "MTP_FW_VER: 0x%02x%02x:0x%02x%02x\n",
			buf[0], buf[1], buf[2], buf[3]);
	p938x_read_buffer(chip, MTP_FW_DATE_REG, buf, 12);
	seq_printf(m, "MTP_FW_DATE: %s\n", buf);
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
	p938x_read_reg(chip, SYS_CMD_REG, &buf[0]);
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
	/* TODO conv is NOT right. */
	seq_printf(m, "DIE TEMP: %d %dC\n",((buf[1] << 8)| buf[0]),
		(((buf[1] << 8)| buf[0]) - 1350) * 83 / 444 - 273);
	p938x_read_buffer(chip, OPT_FREQ_REG, buf, 2);
	seq_printf(m, "OPT FREQ: %dkHz\n", (buf[1] << 8) | buf[0]);
	p938x_read_buffer(chip, ILIMIT_SET_REG, buf, 1);
	seq_printf(m, "ILIMIT_SET: %dmA\n", buf[0] * 100 + 100);
	p938x_read_reg(chip, SYS_MODE_REG, &buf[0]);
	seq_printf(m, "SYS_MODE: 0x%02x\n", buf[0]);
	p938x_read_reg(chip, SYS_TM_MODE_REG, &buf[0]);
	seq_printf(m, "TX_MODE: %d\n", buf[0]);
	p938x_read_reg(chip, EPP_TX_GUARANTEED, &buf[0]);
	seq_printf(m, "EPP_TX_GUARANTEED: 0x%02x\n", buf[0]);
	p938x_read_reg(chip, EPP_TX_POTENTIAL, &buf[0]);
	seq_printf(m, "EPP_TX_POTENTIAL: 0x%02x\n", buf[0]);
	p938x_read_reg(chip, EPP_QFACTOR_REG, &buf[0]);
	seq_printf(m, "EPP_QFACTOR_REG: 0x%02x\n", buf[0]);
	p938x_read_buffer(chip, FOD_CFG_REG, buf, FOD_MAX_LEN);
	seq_printf(m, "FOD_CFG_REG: %02X%02X %02X%02X "
		"%02X%02X %02X%02X "
		"%02X%02X %02X%02X "
		"%02X%02X %02X%02X\n",
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11],
		buf[12], buf[13], buf[14], buf[15]);
	p938x_read_buffer(chip, TX_FOD_CFG_REG, buf, TX_FOD_REG_LEN);
	seq_printf(m, "TX_FOD_CFG_REG: %02X%02X %02X%02X "
		"%02X%02X %02X%02X "
		"%02X%02X %02X "
		"%02X%02X\n",
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11],
		buf[12]);

	show_dump_flags(m, chip);

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
	struct device_node *node = chip->dev->of_node;
	int rc;
	unsigned tmp;
	int byte_len;
	int i;

	if (!node) {
		p938x_err(chip, "device tree info. missing\n");
		return -EINVAL;
	}

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

	chip->fod_array_bpp_len =
		of_property_count_u8_elems(node,
					"fod-array-bpp-val");

	chip->fod_array_epp_len =
		of_property_count_u8_elems(node,
					"fod-array-epp-val");

	chip->fod_array_tx_len =
		of_property_count_u8_elems(node,
					"fod-array-tx-val");

	of_property_read_u8_array(chip->dev->of_node, "fod-array-bpp-val",
					chip->fod_array_bpp,
					chip->fod_array_bpp_len);
	of_property_read_u8_array(chip->dev->of_node, "fod-array-epp-val",
					chip->fod_array_epp,
					chip->fod_array_epp_len);
	of_property_read_u8_array(chip->dev->of_node, "fod-array-tx-val",
					chip->fod_array_tx,
					chip->fod_array_tx_len);

	rc = of_property_read_u32(node, "epp-q-factor",
				  &tmp);
	if (!rc) {
		chip->use_q_factor = true;
		chip->q_factor = tmp;
	}

	if (of_find_property(node, "epp-current-table", &byte_len)) {
		chip->epp_current_limits = (struct p938x_limits *)
			devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);

		if (chip->epp_current_limits == NULL)
			return -ENOMEM;

		rc = of_property_read_u32_array(node,
				"epp-current-table",
				(u32 *)chip->epp_current_limits,
				byte_len / sizeof(u32));
		if (rc < 0) {
			p938x_err(chip, "Couldn't read epp current limits rc = %d\n", rc);
			return rc;
		}

		chip->epp_limit_num =
			byte_len / sizeof(struct p938x_limits);

		p938x_dbg(chip, PR_IMPORTANT, "epp current limits: Num: %d\n", chip->epp_limit_num);
		for (i = 0; i < chip->epp_limit_num; i++) {
			p938x_dbg(chip, PR_IMPORTANT, "epp current limit: %dW, %dmA",
				chip->epp_current_limits[i].tx_capability_mw / 1000,
				chip->epp_current_limits[i].idc_max_ma);
		}
	}

	rc = of_property_read_u32(node, "epp-voltage", &chip->epp_voltage);
	if (rc)
		chip->epp_voltage = EPP_MAX_VOUT;

	rc = of_property_read_u32(node, "bpp-current", &chip->bpp_current_limit);
	if (rc)
		chip->bpp_current_limit = BPP_MAX_IDC;

	rc = of_property_read_u32(node, "bpp-voltage", &chip->bpp_voltage);
	if (rc)
		chip->bpp_voltage = BPP_MAX_VOUT;

	return 0;
}

static enum power_supply_property p938x_wls_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static int p938x_wls_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct p938x_charger *chip = power_supply_get_drvdata(psy);
	int tx_guaranteed;
	int tx_potential;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = p938x_is_ldo_on(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = p938x_is_ldo_on(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = p938x_get_rx_iout(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = p938x_get_rx_ocl(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = p938x_get_rx_vout(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = p938x_get_rx_vout_set(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = POWER_SUPPLY_TYPE_WIRELESS;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = p938x_is_ldo_on(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (p938x_is_ldo_on(chip)) {
			if (chip->epp_mode &&
					!p938x_get_tx_capability(chip, &tx_guaranteed, &tx_potential) &&
					tx_guaranteed >= FAST_CHARGE_MW)
				val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			else
				val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		} else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		return -EINVAL;
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
		chip->wls_iout_max = val->intval / 1000;
		rc = p938x_set_rx_ocl(chip, chip->wls_iout_max);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		chip->wls_vout_max = val->intval / 1000;
		rc = p938x_set_rx_vout(chip, chip->wls_vout_max);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = p938x_enable_charging(chip, val->intval);
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
#ifdef P938X_USER_DEBUG
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
#endif
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}

/* Called when something we are "supplied_from" reports a change */
static void p938x_external_power_changed(struct power_supply *psy)
{
	struct p938x_charger *chip = power_supply_get_drvdata(psy);

	p938x_update_supplies_status(chip);

	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
		msecs_to_jiffies(0));
}

static const struct power_supply_desc wls_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.get_property = p938x_wls_get_prop,
	.set_property = p938x_wls_set_prop,
	.property_is_writeable = p938x_wls_prop_is_writeable,
	.properties = p938x_wls_props,
	.num_properties = ARRAY_SIZE(p938x_wls_props),
	.external_power_changed = p938x_external_power_changed,
};

static int p938x_register_power_supply(struct p938x_charger *chip)
{
	struct power_supply_config wls_psy_cfg = {};
	int rc = 0;

	wls_psy_cfg.drv_data = chip;
	wls_psy_cfg.supplied_to = pm_wls_supplied_to;
	wls_psy_cfg.num_supplicants = ARRAY_SIZE(pm_wls_supplied_to);

	chip->wls_psy = power_supply_register(chip->dev,
			&wls_psy_desc,
			&wls_psy_cfg);

	if (IS_ERR(chip->wls_psy)) {
		p938x_err(chip, "Couldn't register wls psy rc=%ld\n",
				PTR_ERR(chip->wls_psy));
		rc = PTR_ERR(chip->wls_psy);
	} else {
		chip->wls_psy->supplied_from = pm_wls_supplied_from;
		chip->wls_psy->num_supplies = ARRAY_SIZE(pm_wls_supplied_from);
	}

	return rc;
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

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;
	chip->name = "WLS";
	chip->debug_mask = &__debug_mask;
	INIT_DELAYED_WORK(&chip->heartbeat_work, p938x_heartbeat_work);
	INIT_DELAYED_WORK(&chip->tx_mode_work, p938x_tx_mode_work);
	INIT_DELAYED_WORK(&chip->removal_work, p938x_removal_work);
	INIT_DELAYED_WORK(&chip->restore_idc_work, p938x_restore_idc_work);
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

	rc = p938x_register_power_supply(chip);
	if (rc) {
		p938x_err(chip, "Couldn't register power supply rc=%d\n", rc);
		goto free_mem;
	}

	rc = p938x_hw_init(chip);
	if (rc < 0) {
		p938x_err(chip, "Failed to init hw, rc=%d\n", rc);
		goto free_psy;
	}

	chip->dc_psy = power_supply_get_by_name("dc");
	if (!chip->dc_psy) {
		p938x_err(chip, "Couldn't get dc psy\n");
		goto free_psy;
	}

	mutex_init(&chip->disconnect_lock);
	mutex_init(&chip->txmode_lock);

	PM_WAKEUP_REGISTER(chip->dev, chip->wls_wake_source, "p938x wireless charger");

	if (!chip->wls_wake_source) {
		p938x_err(chip, "failed to allocate wakeup source\n");
		rc = -ENOMEM;
		goto free_psy;
	}

	/* In case we are already powered on */
	p938x_check_status(chip);
	p938x_check_system_mode(chip);

	/* This IRQ handler is the primary one, and detects when a wireless charger
	 * is attached
	 */
	rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			p938x_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"p938x_irq", chip);
	if (rc) {
		p938x_err(chip, "Failed irq=%d request rc = %d\n",
				client->irq, rc);
		goto free_psy;
	}

	enable_irq_wake(client->irq);

	/* This IRQ handler is for detachment only.  The chip is powered off when
	 * the transmitter is removed, so we need to rely on a separate IRQ to
	 * handle that event
	 */
	rc = devm_request_threaded_irq(&client->dev, chip->wchg_det_irq, NULL,
			p938x_det_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"p938x_det_irq", chip);
	if (rc) {
		p938x_err(chip, "Failed irq=%d request rc = %d\n",
				chip->wchg_det_irq, rc);
		goto free_psy;
	}

	/* Force dc-en on if we boot attached, see p938x_det_irq_handler */
	if (gpio_get_value(chip->wchg_det.gpio)) {
		p938x_dbg(chip, PR_IMPORTANT, "Detected attached at probe.\n");
		p938x_set_dc_en_override(chip, 2);
		/* Without a dc-in interrupt, the qcom driver will not set the current
		 * so re-run aicl to get the correct max current
		 */
		p938x_set_dc_aicl_rerun(chip);
	}

	/* Register thermal zone cooling device */
	chip->tcd = thermal_of_cooling_device_register(dev_of_node(chip->dev),
		"p938x_charger", chip, &p938x_tcd_ops);

	create_debugfs_entries(chip);
	if (sysfs_create_groups(&chip->dev->kobj, p938x_groups))
		p938x_err(chip, "Failed to create sysfs attributes\n");

	pr_info("p938x wireless receiver initialized successfully\n");

	power_supply_changed(chip->wls_psy);

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

	PM_WAKEUP_UNREGISTER(chip->wls_wake_source);
	sysfs_remove_groups(&chip->dev->kobj, p938x_groups);
	cancel_delayed_work_sync(&chip->heartbeat_work);
	cancel_delayed_work_sync(&chip->tx_mode_work);
	debugfs_remove_recursive(chip->debug_root);
	if(chip->tcd)
		thermal_cooling_device_unregister(chip->tcd);

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
