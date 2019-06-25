#ifndef _SEC_MMI_H_
#define _SEC_MMI_H_

#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/notifier.h>

#if defined(CONFIG_DRM)
#include <linux/msm_drm_notify.h>
extern bool dsi_display_is_panel_enable(int id, int *probe_status, char **pname);
#endif

#define TSDEV_MINOR_BASE 128
#define TSDEV_MINOR_MAX 32

struct sec_ts_data;

struct sec_mmi_data {
	struct sec_ts_data *ts_ptr;
	struct i2c_client *i2c_client;
	struct device *ts_class_dev;
	struct notifier_block panel_nb;
	struct delayed_work resume_work;
	struct delayed_work detection_work;

	int ctrl_dsi;
	unsigned int build_id;
	unsigned int config_id;
	char product_id[10];

	char *image_name;
	const char *class_entry_name;
	const char *bound_display;

	void (*hard_reset)(struct sec_mmi_data *data);
};

int sec_mmi_data_init(struct sec_ts_data *ts, bool enable);

#define DEV_MMI (&data->i2c_client->dev)
#define DEV_TS  (&ts->client->dev)

extern struct class *get_touchscreen_class_ptr(void);
extern void set_touchscreen_class_ptr(struct class *ptr);

#endif
