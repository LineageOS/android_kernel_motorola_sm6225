/*
 * Copyright (C) 2020 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MMI_CHARGER_H__
#define __MMI_CHARGER_H__

#include <linux/ipc_logging.h>
#include <linux/debugfs.h>

#define mmi_err(chg, fmt, ...)			\
	do {						\
		pr_err("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"E %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_warn(chg, fmt, ...)			\
	do {						\
		pr_warn("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"W %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_info(chg, fmt, ...)			\
	do {						\
		pr_info("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
		"I %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define mmi_dbg(chg, fmt, ...)			\
	do {							\
		if (*chg->debug_enabled)		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		ipc_log_string(chg->ipc_log,		\
			"D %s: %s: " fmt, chg->name, __func__, ##__VA_ARGS__); \
	} while (0)

#define MMI_LOG_PAGES (50)
#define MMI_LOG_DIR "mmi_charger"
#define MMI_BATT_SN_LEN 16
#define CHG_SHOW_MAX_SIZE 50

struct mmi_battery_info {
	int batt_mv;
	int batt_ma;
	int batt_soc;
	int batt_temp;
	int batt_status;
	int batt_full_uah;
	int batt_design_uah;
	char batt_sn[MMI_BATT_SN_LEN];
};

struct mmi_charger_info {
	int chrg_mv;
	int chrg_ma;
	int chrg_type;
	int chrg_pmax_mw;
	int chrg_present;
};

struct mmi_charger_cfg {
	int target_fcc;
	int target_fv;
	int fg_iterm;
	int chrg_iterm;
	bool full_charged;
	bool charging_reset;
	bool taper_kickoff;
	bool charging_disable;
	bool charger_suspend;
};

struct mmi_charger_constraint {
	int demo_mode;
	bool factory_mode;
	bool factory_version;
	int dcp_pmax;
	int hvdcp_pmax;
	int pd_pmax;
	int wls_pmax;
};

struct mmi_charger_driver {
	const char *name;
	struct device *dev;
	int (*get_batt_info)(void *data, struct mmi_battery_info *batt_info);
	int (*get_chg_info)(void *data, struct mmi_charger_info *chg_info);
	int (*config_charge)(void *data, struct mmi_charger_cfg *config);
	bool (*is_charge_tapered)(void *data, int tapered_ma);
	bool (*is_charge_halt)(void *data);
	void (*set_constraint)(void *data, struct mmi_charger_constraint *constraint);
	void *data;
};

bool mmi_is_factory_mode(void);
bool mmi_is_factory_version(void);
const char *mmi_get_battery_serialnumber(void);
void mmi_get_charger_configure(struct mmi_charger_driver *driver);
int mmi_register_charger_driver(struct mmi_charger_driver *driver);
int mmi_unregister_charger_driver(struct mmi_charger_driver *driver);

#endif
