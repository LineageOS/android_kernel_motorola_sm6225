/*
 * Copyright (C) 2019 Motorola Mobility LLC
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

#ifndef __LINUX_TOUCHSCREEN_MMI_H_
#define __LINUX_TOUCHSCREEN_MMI_H_

#include <linux/list.h>
#include <linux/device.h>

#if defined(CONFIG_PANEL_NOTIFICATIONS)
#include <linux/panel_notifier.h>
#endif

#define TS_MMI_MAX_FW_PATH		64
#define TS_MMI_MAX_ID_LEN		16
#define TS_MMI_MAX_VENDOR_LEN		16
#define TS_MMI_MAX_INFO_LEN		16
#define TS_MMI_MAX_CLASS_NAME_LEN	16
#define TS_MMI_MAX_PANEL_LEN		16

struct touch_event_data {
	int type;			/* TS_TOUCH, TS_RELEASE */
	int x, y, w, p, m;	/* X, Y, area, pressure and major */
};

enum ts_mmi_pm_mode {
	TS_MMI_PM_DEEPSLEEP = 0,
	TS_MMI_PM_GESTURE,
	TS_MMI_PM_ACTIVE,
};

#define TS_MMI_RESET_SOFT	0
#define TS_MMI_RESET_HARD	1
#define TS_MMI_POWER_OFF	0
#define TS_MMI_POWER_ON		1
#define TS_MMI_PINCTL_OFF	0
#define TS_MMI_PINCTL_ON	1
#define TS_MMI_IRQ_OFF		0
#define TS_MMI_IRQ_ON		1

/**
 * struct touchscreen_mmi_methods - hold vendor provided functions
 *
 * @convert_data:		convert internal touch data to common format
 * @get_vendor:			returns vendor name
 * @get_product_id:		returns product ID string
 * @get_config_id:		returns configuration ID string
 * @get_irq_status:		return current IRQ pin status
 * @reset:				performs touch IC reset (hard or soft)
 * @irq:				enable/disable IRQ handling
 * @firmware_update:	performs firmware update from provided file
 * @firmware_erase:		performs chip erasure
 */
 struct ts_mmi_methods {
	int	(*convert_data)(struct device *dev, struct touch_event_data *data, int max);
	/* GET methods */
	int	(*get_vendor)(struct device *dev, void *cdata);
	int	(*get_productinfo)(struct device *dev, void *cdata);
	int	(*get_build_id)(struct device *dev, void *cdata);
	int	(*get_config_id)(struct device *dev, void *cdata);
	int	(*get_class_entry_name)(struct device *dev, void *cdata);
	int	(*get_bus_type)(struct device *dev, void *idata);
	int	(*get_irq_status)(struct device *dev, void *idata);
	int	(*get_drv_irq)(struct device *dev, void *idata);
	int	(*get_poweron)(struct device *dev, void *idata);
	int	(*get_flashprog)(struct device *dev, void *idata);
	/* SET methods */
	int	(*reset)(struct device *dev, int type);
	int	(*drv_irq)(struct device *dev, int state);
	int	(*power)(struct device *dev, int on);
	int	(*pinctrl)(struct device *dev, int on);
	int	(*refresh_rate)(struct device *dev, int freq);
	/* Firmware */
	int	(*firmware_update)(struct device *dev, char *fwname);
	int	(*firmware_erase)(struct device *dev);
	/* vendor specific attribute group */
	int	(*extend_attribute_group)(struct device *dev, struct attribute_group **group);
	/* PM callback */
	int	(*panel_state)(struct device *dev, enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to);
	int	(*wait_for_ready)(struct device *dev);
	int	(*pre_resume)(struct device *dev);
	int	(*post_resume)(struct device *dev);
	int	(*pre_suspend)(struct device *dev);
	int	(*post_suspend)(struct device *dev);
};

#define TO_CHARP(dp)	((char*)(dp))
#define TO_INT(dp)	(*(int*)(dp))

#define TOUCHSCREEN_MMI_BUS_TYPE_I2C	0
#define TOUCHSCREEN_MMI_BUS_TYPE_SPI	1

struct ts_mmi_dev_pdata {
	bool		power_off_suspend;
	bool		usb_detection;
	bool		update_refresh_rate;
	bool		gestures_enabled;
	int 		ctrl_dsi;
	int		reset;
	const char	*class_entry_name;
	const char 	*bound_display;
};

/**
 * struct touchscreen_mmi_dev - hold touch general parameters and APIs
 *
 * @handle:			Handle that identifies this sensors.
 * @class_dev		Touchscreen class device.
 * @class_dev_minor	Minor number of touchscreen class device.
 * @node:			List for the all the touch drivers.
 * @bus_type:		IC bus type.
 *			0	I2C
 *			1	SPI
 * @mdata:			Vendor specific methods
 * @class_entry_name:
 *
 */
struct ts_mmi_dev {
	/*
	 * internally used
	 */
	struct device		*dev;
	struct device		*class_dev;
	int 			class_dev_minor;
	int			forcereflash;
	struct ts_mmi_dev_pdata	pdata;
	struct notifier_block	panel_nb;
	atomic_t		touch_stopped;
	enum ts_mmi_pm_mode	pm_mode;
	struct delayed_work	resume_work;

	struct notifier_block	freq_nb;
	unsigned char		refresh_rate;

	struct work_struct	ps_notify_work;
	struct notifier_block	ps_notif;
	bool			ps_is_present;

	/*
	 * sys entey variable
	 */
	char			vendor[TS_MMI_MAX_VENDOR_LEN];
	char			productinfo[TS_MMI_MAX_INFO_LEN];
	char			build_id[TS_MMI_MAX_ID_LEN];
	char			config_id[TS_MMI_MAX_ID_LEN];
	char			class_entry_name[TS_MMI_MAX_CLASS_NAME_LEN];
	char			panel_supplier[TS_MMI_MAX_PANEL_LEN];
	int			bus_type;
	int			irq_status;
	int			drv_irq;
	int			poweron;
	int			flashprog;
	struct attribute_group	*extern_group;
	struct list_head	node;
	/*
	 * vendor provided
	 */
	struct ts_mmi_methods *mdata;
};

#define DEV_MMI (touch_cdev->class_dev)
#define DEV_TS  (touch_cdev->dev)
#define MMI_DEV_TO_TS_DEV(cdev) (((struct ts_mmi_dev *)dev_get_drvdata(dev))->dev)
#define TRY_TO_CALL(_method, ...) \
do { \
	if (touch_cdev->mdata->_method) { \
		touch_cdev->mdata->_method(DEV_TS, ##__VA_ARGS__); \
	} \
} while (0)


extern int ts_mmi_notifiers_register(struct ts_mmi_dev *touch_cdev);
extern void ts_mmi_notifiers_unregister(struct ts_mmi_dev *touch_cdev);
extern int ts_mmi_panel_register(struct ts_mmi_dev *touch_cdev);
extern void ts_mmi_panel_unregister(struct ts_mmi_dev *touch_cdev);
extern int ts_mmi_dev_register(struct device *parent,
			struct ts_mmi_methods *mdata);
extern void ts_mmi_dev_unregister(struct device *parent);
extern int ts_mmi_parse_dt(struct ts_mmi_dev *touch_cdev, struct device_node *of_node);

#endif		/* __LINUX_TOUCHSCREEN_MMI_H_ */
