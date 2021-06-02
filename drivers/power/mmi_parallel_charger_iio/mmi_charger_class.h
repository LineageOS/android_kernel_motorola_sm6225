/*
 * Copyright (c) 2018 Motorola Mobility, LLC.
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
 #ifndef __MMI_CHRG_CLASS_H_
 #define __MMI_CHRG_CLASS_H_
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>

#define chrg_dev_info(chip, fmt, ...)		\
	pr_info("%s: %s: " fmt, chip->name,	\
		__func__, ##__VA_ARGS__)	\

struct mmi_charger_info {
	bool vbus_pres;
	int vbatt_volt;
	int vbus_volt;
	int ibatt_curr;
	int ibus_curr;
	int batt_temp;
};

#define MMI_BAT_OVP_ALARM_BIT 0
#define MMI_BAT_OCP_ALARM_BIT 1
#define MMI_BAT_UCP_ALARM_BIT 2
#define MMI_BUS_OVP_ALARM_BIT 3
#define MMI_BUS_OCP_ALARM_BIT 4
#define MMI_BUS_UCP_ALARM_BIT 5
#define MMI_BAT_OVP_FAULT_BIT 6
#define MMI_BAT_OCP_FAULT_BIT 7
#define MMI_BAT_UCP_FAULT_BIT 8
#define MMI_BUS_OVP_FAULT_BIT 9
#define MMI_BUS_OCP_FAULT_BIT 10
#define MMI_BUS_UCP_FAULT_BIT 11
#define MMI_CONV_OCP_FAULT_BIT 12
#define MMI_THERM_ALARM_BIT 13
#define MMI_THERM_FAULT_BIT  14

struct mmi_charger_error_info {
	int chrg_err_type;
	int bus_ucp_err_cnt;
	int bus_ocp_err_cnt;
};

struct mmi_charger_device {
	const char *name;	/*charger device name*/
	struct power_supply	*chrg_psy; /*power supply handle*/
	const struct mmi_charger_ops	*ops; /*operation function*/
	struct mutex	ops_lock;
	struct device	dev;
	struct mmi_charger_info	charger_data;
	struct mmi_charger_error_info charger_error;
	int *debug_mask;
	int input_curr_setted; /*save input current*/
	int charging_curr_limited; /*charger current limitation*/
	int charging_curr_min; /*The minimum charging current supported by this device*/
	bool	charger_enabled; /*is this charger device enabled*/
	bool	charger_limited; /*is the charging current of this device limited*/
};

struct mmi_charger_ops {
	int (*enable)(struct mmi_charger_device *chrg, bool en);
	int (*is_enabled)(struct mmi_charger_device *chrg, bool *en);
	int (*get_charging_current)(struct mmi_charger_device *chrg, u32 *uA);
	int (*set_charging_current)(struct mmi_charger_device *chrg, u32 uA);
	int (*get_input_current_settled)(struct mmi_charger_device *chrg, u32 *uA);
	int (*get_vbus)(struct mmi_charger_device *chrg, u32 *mv);
	int (*get_input_current)(struct mmi_charger_device *chrg, u32 *uA);
	int (*set_input_current)(struct mmi_charger_device *chrg, u32 uA);
	int (*update_charger_status)(struct mmi_charger_device *chrg);
	int (*update_charger_error)(struct mmi_charger_device *chrg);
	int (*clear_charger_error)(struct mmi_charger_device *chrg);
};


#define to_mmi_charger_device(obj) container_of(obj, struct mmi_charger_device, dev)
#define mmi_chrg_name(x) (dev_name(&(x)->dev))

extern struct mmi_charger_device *get_charger_by_name(const char *name);
extern int is_charger_exist(const char *name);
extern int mmi_charger_class_init(void);
extern void mmi_charger_class_exit(void);
extern int mmi_enable_charging(struct mmi_charger_device *chrg, bool en);
extern int mmi_is_charging_enabled(struct mmi_charger_device *chrg, bool *en);
extern int mmi_get_charing_current(struct mmi_charger_device *chrg, u32 *uA);
extern int mmi_set_charing_current(struct mmi_charger_device *chrg, u32 uA);
extern int mmi_get_input_current_settled(struct mmi_charger_device *chrg, u32 *uA);
extern int mmi_get_input_current(struct mmi_charger_device *chrg, u32 *uA);
extern int mmi_set_input_current(struct mmi_charger_device *chrg, u32 uA);
extern int mmi_update_charger_status(struct mmi_charger_device *chrg);
extern int mmi_update_charger_error(struct mmi_charger_device *chrg);
extern int mmi_clear_charger_error(struct mmi_charger_device *chrg);
extern int mmi_get_vbus(struct mmi_charger_device *chrg, u32 *mv);
extern struct mmi_charger_ops sc8549_charger_ops;
extern struct mmi_charger_ops qpnp_pmic_charger_ops;
extern struct mmi_charger_device *mmi_charger_device_register(const char *name,
		const char *psy_name,struct device *parent, void *devdata,
		const struct mmi_charger_ops *ops);
extern void mmi_charger_device_unregister(struct mmi_charger_device* charger_dev);
#endif
