#ifndef _SEC_MMI_H_
#define _SEC_MMI_H_

#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#include <linux/version.h>
#include <linux/mmi_wake_lock.h>

#if defined(CONFIG_PANEL_NOTIFICATIONS)
#include <linux/panel_notifier.h>
#endif
#if defined(CONFIG_DRM)
#include <linux/msm_drm_notify.h>
extern bool dsi_display_is_panel_enable(int id, int *probe_status, char **pname);
#endif

#define TSDEV_MINOR_BASE 128
#define TSDEV_MINOR_MAX 32

struct sec_ts_data;

enum sec_mmi_task {
	MMI_TASK_INIT = 0,
	MMI_TASK_SET_RATE,
};

struct sec_mmi_data {
	struct sec_ts_data *ts_ptr;
	struct i2c_client *i2c_client;
	struct device *ts_class_dev;
	struct notifier_block panel_nb;

	bool update_refresh_rate;
	struct notifier_block freq_nb;
	unsigned char refresh_rate;
	enum sec_mmi_task task;

	struct delayed_work resume_work;
	struct delayed_work work;

	atomic_t touch_stopped;

	bool gpio_config;
	bool gestures_enabled;
	bool power_off_suspend;
	unsigned int reset;
	bool usb_detection;
	struct work_struct ps_notify_work;
	struct notifier_block ps_notif;
	bool ps_is_present;

	bool force_calibration;
	int ctrl_dsi;
	unsigned int build_id;
	unsigned int config_id;
	char product_id[10];
	char panel_supplier[16];

	char *image_name;
	const char *class_entry_name;
	const char *bound_display;

	void (*reset_func)(struct sec_mmi_data *data, int mode);
};

int sec_mmi_data_init(struct sec_ts_data *ts, bool enable);

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
int sec_mmi_sysfs_notify(struct sec_ts_data *ts, unsigned char state);
#else
#define DEV_MMI (&data->i2c_client->dev)
#define DEV_TS  (&ts->client->dev)
static int inline sec_mmi_sysfs_notify(struct sec_ts_data *ts, unsigned char state) {
	return -ENOSYS;
}
#endif

extern struct class *get_touchscreen_class_ptr(void);
extern void set_touchscreen_class_ptr(struct class *ptr);

#endif
