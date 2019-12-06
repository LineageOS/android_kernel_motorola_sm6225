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
	/* Firmware */
	int	(*firmware_update)(struct device *dev, char *fwname);
	int	(*firmware_erase)(struct device *dev);
};

#define TO_CHARP(dp)	((char*)(dp))
#define TO_INT(dp)	(*(int*)(dp))

#define TOUCHSCREEN_MMI_BUS_TYPE_I2C	0
#define TOUCHSCREEN_MMI_BUS_TYPE_SPI	1

struct ts_mmi_dev_pdata {
	bool		power_off_suspend;
	bool		usb_detection;
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
	struct list_head	node;
	/*
	 * vendor provided
	 */
	struct ts_mmi_methods *mdata;
};

#define DEV_MMI (touch_cdev->class_dev)
#define DEV_TS  (touch_cdev->dev)

extern int ts_mmi_panel_register(struct ts_mmi_dev *touch_cdev);
extern void ts_mmi_panel_unregister(struct ts_mmi_dev *touch_cdev);
extern int ts_mmi_dev_register(struct device *parent,
			struct ts_mmi_methods *mdata);
extern void ts_mmi_dev_unregister(struct device *parent);
extern int ts_mmi_parse_dt(struct ts_mmi_dev *touch_cdev, struct device_node *of_node);

#endif		/* __LINUX_TOUCHSCREEN_MMI_H_ */
