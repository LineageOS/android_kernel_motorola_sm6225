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
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/mmi_kernel_common.h>

#if defined(CONFIG_PANEL_NOTIFICATIONS)

#include <linux/panel_notifier.h>

#define REGISTER_PANEL_NOTIFIER {\
	touch_cdev->panel_nb.notifier_call = ts_mmi_panel_cb; \
	ret = panel_register_notifier(&touch_cdev->panel_nb); \
}

#define UNREGISTER_PANEL_NOTIFIER {\
	panel_unregister_notifier(&touch_cdev->panel_nb); \
}

#define GET_CONTROL_DSI_INDEX { \
	if (evd) \
		idx = *(int *)evd; \
}

#define EVENT_PRE_DISPLAY_OFF \
	(event == PANEL_EVENT_PRE_DISPLAY_OFF)

#define EVENT_DISPLAY_OFF \
	(event == PANEL_EVENT_DISPLAY_OFF)

#define EVENT_PRE_DISPLAY_ON \
	(event == PANEL_EVENT_PRE_DISPLAY_ON)

#define EVENT_DISPLAY_ON \
	(event == PANEL_EVENT_DISPLAY_ON)

#else /* CONFIG_PANEL_NOTIFICATIONS */
#if defined(CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS)
#include <linux/soc/qcom/panel_event_notifier.h>

#define REGISTER_PANEL_NOTIFIER { \
	void *cookie = NULL; \
	struct ts_mmi_dev_pdata *ppdata = &touch_cdev->pdata; \
	if (!ppdata->ctrl_dsi) { \
		cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY, \
			PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH, touch_cdev->active_panel, \
			&ts_mmi_panel_cb, touch_cdev); \
	} else { \
		cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_SECONDARY, \
			PANEL_EVENT_NOTIFIER_CLIENT_SECONDARY_TOUCH, touch_cdev->active_panel, \
			&ts_mmi_panel_cb, touch_cdev); \
	} \
	if (!cookie) \
		ret = -1; \
	else \
		touch_cdev->notifier_cookie = cookie; \
}

#define UNREGISTER_PANEL_NOTIFIER { \
	panel_event_notifier_unregister(touch_cdev->notifier_cookie); \
}

#define EVENT_PRE_DISPLAY_OFF \
	((event == DRM_PANEL_EVENT_BLANK) && \
	 (evdata.early_trigger))

#define EVENT_DISPLAY_OFF \
	((event == DRM_PANEL_EVENT_BLANK) && \
	 (!evdata.early_trigger))

#define EVENT_PRE_DISPLAY_ON \
	((event == DRM_PANEL_EVENT_UNBLANK) && \
	 (evdata.early_trigger))

#define EVENT_DISPLAY_ON \
	((event == DRM_PANEL_EVENT_UNBLANK) && \
	 (!evdata.early_trigger))

#else /* CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS */
#if defined(CONFIG_DRM_PANEL_NOTIFICATIONS)
#include <drm/drm_panel.h>
#define REGISTER_PANEL_NOTIFIER {\
	touch_cdev->panel_nb.notifier_call = ts_mmi_panel_cb; \
	ret = drm_panel_notifier_register(touch_cdev->active_panel, &touch_cdev->panel_nb); \
}

#define UNREGISTER_PANEL_NOTIFIER {\
	drm_panel_notifier_unregister(touch_cdev->active_panel, &touch_cdev->panel_nb);\
}

#define GET_CONTROL_DSI_INDEX \
int *blank; \
struct drm_panel_notifier *evdata = evd; \
{ \
	if (!(evdata && evdata->data)) { \
		dev_dbg(DEV_MMI, "%s: invalid evdata\n", __func__); \
		return 0; \
	} \
	idx = 0;\
	blank = (int *)evdata->data; \
	dev_dbg(DEV_MMI, "%s: drm notification: event = %lu, blank = %d\n", \
			__func__, event, *blank); \
}

#define EVENT_PRE_DISPLAY_OFF \
	((event == DRM_PANEL_EARLY_EVENT_BLANK) && \
	 (*blank == DRM_PANEL_BLANK_POWERDOWN))

#define EVENT_DISPLAY_OFF \
	((event == DRM_PANEL_EVENT_BLANK) && \
	 (*blank == DRM_PANEL_BLANK_POWERDOWN))

#define EVENT_PRE_DISPLAY_ON \
	((event == DRM_PANEL_EARLY_EVENT_BLANK) && \
	 (*blank == DRM_PANEL_BLANK_UNBLANK))

#define EVENT_DISPLAY_ON \
	((event == DRM_PANEL_EVENT_BLANK) && \
	 (*blank == DRM_PANEL_BLANK_UNBLANK))

#else /* CONFIG_DRM_PANEL_NOTIFICATIONS */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,0)
#if defined(CONFIG_DRM_MSM)

#include <linux/msm_drm_notify.h>

#define REGISTER_PANEL_NOTIFIER {\
	touch_cdev->panel_nb.notifier_call = ts_mmi_panel_cb; \
	ret = msm_drm_register_client(&touch_cdev->panel_nb); \
}

#define UNREGISTER_PANEL_NOTIFIER {\
	msm_drm_unregister_client(&touch_cdev->panel_nb); \
}

#define GET_CONTROL_DSI_INDEX \
int *blank; \
struct msm_drm_notifier *evdata = evd; \
{ \
	if (!(evdata && evdata->data)) { \
		dev_dbg(DEV_MMI, "%s: invalid evdata\n", __func__); \
		return 0; \
	} \
	idx = evdata->id; \
	blank = (int *)evdata->data; \
	dev_dbg(DEV_MMI, "%s: drm notification: event = %lu, blank = %d\n", \
			__func__, event, *blank); \
}

#define EVENT_PRE_DISPLAY_OFF \
	((event == MSM_DRM_EARLY_EVENT_BLANK) && \
	 (*blank == MSM_DRM_BLANK_POWERDOWN))

#define EVENT_DISPLAY_OFF \
	((event == MSM_DRM_EVENT_BLANK) && \
	 (*blank == MSM_DRM_BLANK_POWERDOWN))

#define EVENT_PRE_DISPLAY_ON \
	((event == MSM_DRM_EARLY_EVENT_BLANK) && \
	 (*blank == MSM_DRM_BLANK_UNBLANK))

#define EVENT_DISPLAY_ON \
	((event == MSM_DRM_EVENT_BLANK) && \
	 (*blank == MSM_DRM_BLANK_UNBLANK))

/* enable internal config option to refernece in the code */
#define CONFIG_MSM_DRM_NOTIFICATIONS

#else /* CONFIG_DRM_MSM */

#warning That isn't supposed to happen!!!
#define register_panel_notifier(...) ret
#define unregister_panel_notifier(...)

#endif /* CONFIG_DRM_MSM */
#else /* LINUX_VERSION_CODE */

#warning Panel notifier undefined!!!
#define register_panel_notifier(...) ret
#define unregister_panel_notifier(...)

#endif /* LINUX_VERSION_CODE */
#endif /* CONFIG_DRM_PANEL_NOTIFICATIONS */
#endif /* CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS */
#endif /* CONFIG_PANEL_NOTIFICATIONS */


#if defined (TS_MMI_TOUCH_GESTURE_POISON_EVENT)
#ifndef TS_MMI_TOUCH_EDGE_GESTURE
#define TS_MMI_TOUCH_EDGE_GESTURE
#endif
#endif


#define NANO_SEC	1000000000
#define SEC_TO_MSEC	1000
#define NANO_TO_MSEC	1000000

static inline unsigned long long timediff_ms(
		struct TIME_SPEC start, struct TIME_SPEC end)
{
	struct TIME_SPEC temp;

	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = NANO_SEC + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return (temp.tv_sec * SEC_TO_MSEC) + (temp.tv_nsec / NANO_TO_MSEC);
}

#define TS_MMI_MAX_POINT_NUM		10
#define TS_MMI_MAX_FW_PATH		64
#define TS_MMI_MAX_FULL_FW_PATH		128
#define TS_MMI_MAX_ID_LEN		16
#define TS_MMI_MAX_VENDOR_LEN		16
#define TS_MMI_MAX_INFO_LEN		16
#define TS_MMI_MAX_CLASS_NAME_LEN	16
#define TS_MMI_MAX_PANEL_LEN		16
#define TS_MMI_PILL_REGION_REQ_ARGS_NUM	3
#define TS_MMI_FW_PARAM_PATH	"/data/vendor/param/touch/"

enum touch_event_mode {
	TS_COORDINATE_ACTION_NONE = 0,
	TS_COORDINATE_ACTION_PRESS,
	TS_COORDINATE_ACTION_RELEASE,
	TS_COORDINATE_ACTION_MOVE
};

enum TS_FW_UPGRADE_MODE {
	FW_DEFAULT_MODE = 0,
	FW_PARAM_MODE,
	FW_SDCARD_MODE,
};

struct touch_event_data {
	bool skip_report;
	enum touch_event_mode type;		/* TS_TOUCH, TS_RELEASE */
	int id;             			/* Finger id */
	int x, y, w, p;        			/* X, Y, Area, Pressure */
	int major, minor;   			/* Major, Minor */
};

struct gesture_event_data {
	 union {
		int evcode;
		struct touch_event_data evdata;
	};
};

/**
 * struct touchscreen_mmi_class_methods - export class methods to vendor
 *
 * @report_gesture:    report gesture event
 */
struct ts_mmi_class_methods {
	int     (*report_gesture)(struct gesture_event_data *gev);
	int     (*report_palm)(bool value);
	int     (*get_class_fname)(struct device *dev , const char **fname);
	int     (*get_supplier)(struct device *dev , const char **sname);
	int     (*report_touch_event)(struct touch_event_data *tev, struct input_dev *input_dev);
	struct kobject *kobj_notify;
};

enum ts_mmi_pm_mode {
	TS_MMI_PM_DEEPSLEEP = 0,
	TS_MMI_PM_GESTURE,
	TS_MMI_PM_ACTIVE,
};

enum ts_mmi_panel_event {
	TS_MMI_EVENT_PRE_DISPLAY_OFF,
	TS_MMI_EVENT_PRE_DISPLAY_ON,
	TS_MMI_EVENT_DISPLAY_OFF,
	TS_MMI_EVENT_DISPLAY_ON,
	TS_MMI_EVENT_DISPLAY_ON_PREPARE,
	TS_MMI_EVENT_UNKNOWN
};

#define TS_MMI_RESET_SOFT	0
#define TS_MMI_RESET_HARD	1
#define TS_MMI_POWER_OFF	0
#define TS_MMI_POWER_ON		1
#define TS_MMI_PINCTL_OFF	0
#define TS_MMI_PINCTL_ON	1
#define TS_MMI_IRQ_OFF		0
#define TS_MMI_IRQ_ON		1
#define TS_MMI_UPDATE_BASELINE_OFF	0
#define TS_MMI_UPDATE_BASELINE_ON	1

#define TS_MMI_DISABLE_SUPPRESSION_NONE	0
#define TS_MMI_DISABLE_SUPPRESSION_ALL		1
#define TS_MMI_DISABLE_SUPPRESSION_LEFT	2
#define TS_MMI_DISABLE_SUPPRESSION_RIGHT	3
#define TOUCHSCREEN_MMI_DEFAULT_GS_DISTANCE	0x1E
#define TOUCHSCREEN_MMI_DEFAULT_POISON_TIMEOUT_MS	800
#define TOUCHSCREEN_MMI_DEFAULT_POISON_TRIGGER_DISTANCE	120
#define TOUCHSCREEN_MMI_DEFAULT_POISON_DISTANCE	25

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
	int	(*get_suppression)(struct device *dev, void *idata);
	int	(*get_hold_grip)(struct device *dev, void *idata);
	int	(*get_flash_mode)(struct device *dev, void *idata);
	int	(*get_pill_region)(struct device *dev, void *uiadata);
	int	(*get_hold_distance)(struct device *dev, void *idata);
	int	(*get_gs_distance)(struct device *dev, void *idata);
	int	(*get_poison_timeout)(struct device *dev, void *idata);
	int	(*get_poison_distance)(struct device *dev, void *idata);
	int	(*get_poison_trigger_distance)(struct device *dev, void *idata);
	/* SET methods */
	int	(*reset)(struct device *dev, int type);
	int	(*drv_irq)(struct device *dev, int state);
	int	(*power)(struct device *dev, int on);
	int	(*pinctrl)(struct device *dev, int on);
	int	(*refresh_rate)(struct device *dev, int freq);
	int	(*charger_mode)(struct device *dev, int mode);
	int	(*palm_set_enable)(struct device *dev, unsigned int enable);
	int	(*suppression)(struct device *dev, int state);
	int	(*hold_grip)(struct device *dev, int state);
	int	(*flash_mode)(struct device *dev, int state);
	int	(*pill_region)(struct device *dev, int *region_array);
	int	(*hold_distance)(struct device *dev, int dis);
	int	(*gs_distance)(struct device *dev, int dis);
	int	(*poison_timeout)(struct device *dev, int timeout);
	int	(*poison_distance)(struct device *dev, int dis);
	int	(*poison_trigger_distance)(struct device *dev, int dis);
	int	(*update_baseline)(struct device *dev, int enable);
	int	(*update_fod_mode)(struct device *dev, int enable);
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
	/*
	 * class exported methods
	 */
	struct ts_mmi_class_methods exports;
};

#define TO_CHARP(dp)	((char*)(dp))
#define TO_INT(dp)	(*(int*)(dp))

#define TOUCHSCREEN_MMI_BUS_TYPE_I2C	0
#define TOUCHSCREEN_MMI_BUS_TYPE_SPI	1

struct ts_mmi_dev_pdata {
	bool		power_off_suspend;
	bool		fps_detection;
	bool		fod_detection;
	bool		usb_detection;
	bool		update_refresh_rate;
	bool		gestures_enabled;
	bool		palm_enabled;
	bool		fw_load_resume;
	bool		suppression_ctrl;
	bool		pill_region_ctrl;
	bool		hold_distance_ctrl;
	bool		gs_distance_ctrl;
	bool		hold_grip_ctrl;
	bool		poison_slot_ctrl;
	int		max_x;
	int		max_y;
	int		fod_x;
	int		fod_y;
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
	dev_t			class_dev_no;
	int			forcereflash;
	int			panel_status;
	struct ts_mmi_dev_pdata	pdata;
#if defined(CONFIG_DRM_PANEL_NOTIFICATIONS) || defined (CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS)
	struct drm_panel *active_panel;
#endif
#ifdef CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS
	void *notifier_cookie;
#else
	struct notifier_block	panel_nb;
#endif
	struct mutex		extif_mutex;
	struct mutex		method_mutex;
	struct pinctrl		*pinctrl_node;
	struct pinctrl_state		*pinctrl_on_state;
	struct pinctrl_state		*pinctrl_off_state;

	atomic_t		touch_stopped;
	enum ts_mmi_pm_mode	pm_mode;

	atomic_t		resume_should_stop;
	struct delayed_work	work;
	struct kfifo		cmd_pipe;

	struct notifier_block	freq_nb;
	unsigned char		refresh_rate;

	struct work_struct	ps_notify_work;
	struct notifier_block	ps_notif;
	bool			ps_is_present;

	struct notifier_block	fps_notif;
	bool is_fps_registered;	/* FPS notif registration might be delayed */
	bool fps_state;
	bool delay_baseline_update;

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
	int			suppression;
	unsigned int		pill_region[TS_MMI_PILL_REGION_REQ_ARGS_NUM];
	int			hold_distance;
	int			gs_distance;
	int			hold_grip;
	int			flash_mode;
	int			poison_timeout;
	int			poison_distance;
	int			poison_trigger_distance;
	int			charger_mode;
	int			reset;
	int			pinctrl;
	int			update_baseline;
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
/* call this after hold method mutex */
#define _TRY_TO_CALL(_method, ...) \
do { \
	if (touch_cdev->mdata->_method) { \
		ret = touch_cdev->mdata->_method(DEV_TS, ##__VA_ARGS__); \
	} \
} while (0)
#define TRY_TO_CALL(_method, ...) \
do { \
	ret = ret; \
	mutex_lock(&touch_cdev->method_mutex); \
	_TRY_TO_CALL(_method, ##__VA_ARGS__); \
	mutex_unlock(&touch_cdev->method_mutex); \
} while (0)
#define TRY_TO_GET(_method, ...) \
do { \
	ret = ret; \
	if (touch_cdev->mdata->get_##_method) { \
		mutex_lock(&touch_cdev->method_mutex); \
		ret = touch_cdev->mdata->get_##_method(DEV_TS, ##__VA_ARGS__); \
		mutex_unlock(&touch_cdev->method_mutex); \
	} \
} while (0)
#define is_touch_stopped	(atomic_read(&touch_cdev->touch_stopped) == 1)
#define is_touch_active		(touch_cdev->pm_mode == TS_MMI_PM_ACTIVE)

extern int ts_mmi_notifiers_register(struct ts_mmi_dev *touch_cdev);
extern void ts_mmi_notifiers_unregister(struct ts_mmi_dev *touch_cdev);
extern int ts_mmi_panel_register(struct ts_mmi_dev *touch_cdev);
extern void ts_mmi_panel_unregister(struct ts_mmi_dev *touch_cdev);
extern int ts_mmi_dev_register(struct device *parent,
			struct ts_mmi_methods *mdata);
extern void ts_mmi_dev_unregister(struct device *parent);
extern int ts_mmi_parse_dt(struct ts_mmi_dev *touch_cdev, struct device_node *of_node);
extern int ts_mmi_gesture_init(struct ts_mmi_dev *data);
extern int ts_mmi_gesture_remove(struct ts_mmi_dev *data);
extern int ts_mmi_palm_init(struct ts_mmi_dev *data);
extern int ts_mmi_palm_remove(struct ts_mmi_dev *data);
#ifdef TS_MMI_TOUCH_EDGE_GESTURE
extern int ts_mmi_gesture_suspend(struct ts_mmi_dev *touch_cdev);
#endif
#if defined (CONFIG_DRM_PANEL_NOTIFICATIONS) || defined (CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS)
int ts_mmi_check_drm_panel(struct ts_mmi_dev* touch_cdev, struct device_node *of_node);
#endif
extern bool ts_mmi_is_panel_match(const char *panel_node, char *touch_ic_name);

/*sensor*/
extern bool ts_mmi_is_sensor_enable(void);

extern const char *mmi_bl_bootmode(void);
#endif		/* __LINUX_TOUCHSCREEN_MMI_H_ */
