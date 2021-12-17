/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MMI_DISCRETE_CHARGER_CLASS_H
#define MMI_DISCRETE_CHARGER_CLASS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>

struct charger_properties {
	const char *alias_name;
};

/* Data of notifier from charger device */
struct chgdev_notify {
	bool apsd_done;
	bool hvdcp_done;
};

struct charger_device {
	struct charger_properties props;
	struct chgdev_notify noti;
	const struct charger_ops *ops;
	struct mutex ops_lock;
	struct device dev;
	struct srcu_notifier_head evt_nh;
	void	*driver_data;
	bool is_polling_mode;
};

struct charger_ops {

	int (*set_input_voltage_limit)(struct charger_device *dev, u32 uV);

	int (*set_dp_dm)(struct charger_device *dev, int val);
	int (*get_pulse_cnt)(struct charger_device *dev, int *count);

	int (*get_real_charger_type)(struct charger_device *dev, int *charger_type);

	/* enable/disable charger */
	int (*enable_charging)(struct charger_device *dev, bool en);
	int (*is_enabled_charging)(struct charger_device *dev, bool *en);

	/* get/set charging current*/
	int (*get_charging_current)(struct charger_device *dev, u32 *uA);
	int (*set_charging_current)(struct charger_device *dev, u32 uA);
	int (*get_min_charging_current)(struct charger_device *dev, u32 *uA);

	/* set cv */
	int (*set_constant_voltage)(struct charger_device *dev, u32 uV);
	int (*get_constant_voltage)(struct charger_device *dev, u32 *uV);

	/* get/set input_current */
	int (*get_input_current)(struct charger_device *dev, u32 *uA);
	int (*set_input_current)(struct charger_device *dev, u32 uA);
	int (*get_min_input_current)(struct charger_device *dev, u32 *uA);

	/* enable/disable usb suspend */
	int (*is_usb_suspend)(struct charger_device *dev, bool *suspend_en);
	int (*set_usb_suspend)(struct charger_device *dev, bool suspend_en);

	int (*enable_hw_jeita)(struct charger_device *dev, bool en);

	/*OTG*/
	int (*enable_otg)(struct charger_device *dev, bool en);
	int (*set_boost_current_limit)(struct charger_device *dev, u32 uA);

	int (*rerun_aicl)(struct charger_device *dev);
	int (*is_charge_halted)(struct charger_device *dev, bool *en);

	int (*dump_registers)(struct charger_device *dev, struct seq_file *m);
	int (*enable_termination)(struct charger_device *dev, bool en);
};

#define to_charger_device(obj) container_of(obj, struct charger_device, dev)

static inline void *charger_dev_get_drvdata(
	const struct charger_device *charger_dev)
{
	return charger_dev->driver_data;
}

static inline void charger_dev_set_drvdata(
	struct charger_device *charger_dev, void *data)
{
	charger_dev->driver_data = data;
}

extern int charger_dev_is_charge_halted(struct charger_device *chg_dev, bool *en);

extern int charger_dev_set_constant_voltage(struct charger_device *chg_dev, u32 uV);

extern int charger_dev_set_charging_current(struct charger_device *chg_dev, u32 uA);

extern int charger_dev_get_charging_current(struct charger_device *chg_dev, u32 *uA);

extern int charger_dev_enable_charging(struct charger_device *chg_dev, bool en);

extern int charger_dev_enable_termination(struct charger_device *chg_dev, bool en);

extern int charger_dev_is_enabled_charging(struct charger_device *chg_dev, bool *en);

extern int charger_dev_dump_registers(struct charger_device *chg_dev, struct seq_file *m);

extern int charger_dev_rerun_aicl(struct charger_device *chg_dev);

extern int charger_dev_enable_otg(struct charger_device *chg_dev, bool en);

extern int charger_dev_set_boost_current_limit(struct charger_device *chg_dev, u32 uA);

extern int charger_dev_enable_hw_jeita(struct charger_device *chg_dev, bool enable);

extern int charger_dev_is_usb_suspend(struct charger_device *chg_dev, bool *suspend_en);

extern int charger_dev_set_usb_suspend(struct charger_device *chg_dev, bool suspend_en);

extern int charger_dev_get_input_current(struct charger_device *charger_dev, u32 *uA);

extern int charger_dev_set_input_current(struct charger_device *chg_dev, u32 uA);

extern int charger_dev_set_input_voltage(struct charger_device *chg_dev, u32 uV);

extern int charger_dev_get_pulse_cnt(struct charger_device *chg_dev, int *count);

extern int charger_dev_set_dp_dm(struct charger_device *chg_dev, int val);

extern int charger_dev_get_real_charger_type(struct charger_device *chg_dev, int *charger_type);

extern int register_charger_device_notifier(
	struct charger_device *charger_dev,
			      struct notifier_block *nb);

extern int unregister_charger_device_notifier(
	struct charger_device *charger_dev,
				struct notifier_block *nb);

extern int charger_dev_notify(struct charger_device *charger_dev);

extern struct charger_device *charger_device_register(
	const char *name,
	struct device *parent, void *devdata, const struct charger_ops *ops,
	const struct charger_properties *props);

extern void charger_device_unregister(
	struct charger_device *charger_dev);

extern struct charger_device *get_charger_by_name(
	const char *name);

#endif
