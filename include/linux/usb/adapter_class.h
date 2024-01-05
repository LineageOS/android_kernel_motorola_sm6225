/* Copyright (c)  2020, 2021 The Linux Foundation. All rights reserved.
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

#ifndef LINUX_POWER_ADAPTER_CLASS_H
#define LINUX_POWER_ADAPTER_CLASS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>


#define ADAPTER_CAP_MAX_NR 10

struct adapter_power_cap {
	uint8_t selected_cap_idx;
	uint8_t nr;
	uint8_t pdp;
	uint8_t pwr_limit[ADAPTER_CAP_MAX_NR];
	int max_mv[ADAPTER_CAP_MAX_NR];
	int min_mv[ADAPTER_CAP_MAX_NR];
	int ma[ADAPTER_CAP_MAX_NR];
	int maxwatt[ADAPTER_CAP_MAX_NR];
	int minwatt[ADAPTER_CAP_MAX_NR];
	uint8_t type[ADAPTER_CAP_MAX_NR];
	int info[ADAPTER_CAP_MAX_NR];
};

enum adapter_type {
	MMI_PD_ADAPTER,
};

enum adapter_event {
	MMI_PD_CONNECT_NONE,
	MMI_PD_CONNECT_HARD_RESET,
	MMI_PD_CONNECT_PE_READY_SNK,
	MMI_PD_CONNECT_PE_READY_SNK_PD30,
	MMI_PD_CONNECT_PE_READY_SNK_APDO,
	MMI_PD_CONNECT_TYPEC_ONLY_SNK,
	MMI_TYPEC_WD_STATUS,
	MMI_TYPEC_HRESET_STATUS,
};

enum adapter_property {
	TYPEC_RP_LEVEL,
	PD_TYPE,
};

enum adapter_cap_type {
	/*start pd30 apdo*/
	MMI_PD_APDO_START,
	/*end pd30 apdo*/
	MMI_PD_APDO_END,
	/*pd30 apdo*/
	MMI_PD_APDO,
	/*pd20 fixed pdo*/
	MMI_PD_FIXED,
	/*all pdo: pd20&pd30*/
	MMI_PD_ALL,
	MMI_CAP_TYPE_UNKNOWN,
};

enum adapter_return_value {
	MMI_ADAPTER_OK = 0,
	MMI_ADAPTER_NOT_SUPPORT,
	MMI_ADAPTER_TIMEOUT,
	MMI_ADAPTER_REJECT,
	MMI_ADAPTER_ERROR,
	MMI_ADAPTER_ADJUST,
};


struct adapter_status {
	int temperature;
	bool ocp;
	bool otp;
	bool ovp;
};

struct adapter_properties {
	const char *alias_name;
};

struct adapter_device {
	struct adapter_properties props;
	const struct adapter_ops *ops;
	struct mutex ops_lock;
	struct device dev;
	struct srcu_notifier_head evt_nh;
	void	*driver_data;

};

struct adapter_ops {
	int (*suspend)(struct adapter_device *dev, pm_message_t state);
	int (*resume)(struct adapter_device *dev);
	int (*get_property)(struct adapter_device *dev,
		enum adapter_property pro);
	int (*get_status)(struct adapter_device *dev,
		struct adapter_status *sta);
	int (*set_cap)(struct adapter_device *dev, enum adapter_cap_type type,
		int pdo_idx, int mV, int mA);
	int (*get_cap)(struct adapter_device *dev, enum adapter_cap_type type,
		struct adapter_power_cap *cap);
	int (*get_output)(struct adapter_device *dev, int *mV, int *mA);

};

static inline void *adapter_dev_get_drvdata(
	const struct adapter_device *adapter_dev)
{
	return adapter_dev->driver_data;
}

static inline void adapter_dev_set_drvdata(
	struct adapter_device *adapter_dev, void *data)
{
	adapter_dev->driver_data = data;
}

extern struct adapter_device *adapter_device_register(
	const char *name,
	struct device *parent, void *devdata, const struct adapter_ops *ops,
	const struct adapter_properties *props);
extern void adapter_device_unregister(
	struct adapter_device *adapter_dev);
extern int register_adapter_device_notifier(struct adapter_device *adapter_dev,
				struct notifier_block *nb);
extern int unregister_adapter_device_notifier(
				struct adapter_device *adapter_dev,
				struct notifier_block *nb);
extern struct adapter_device *get_adapter_by_name(
	const char *name);

#define to_adapter_device(obj) container_of(obj, struct adapter_device, dev)

extern int adapter_dev_get_property(struct adapter_device *adapter_dev,
	enum adapter_property sta);
extern int adapter_dev_get_status(struct adapter_device *adapter_dev,
	struct adapter_status *sta);
extern int adapter_dev_get_output(struct adapter_device *adapter_dev,
	int *mV, int *mA);
extern int adapter_dev_set_cap(struct adapter_device *adapter_dev,
	enum adapter_cap_type type, int pdo_idx,
	int mV, int mA);
extern int adapter_dev_get_cap(struct adapter_device *adapter_dev,
	enum adapter_cap_type type,
	struct adapter_power_cap *cap);


#endif /*LINUX_POWER_ADAPTER_CLASS_H*/

