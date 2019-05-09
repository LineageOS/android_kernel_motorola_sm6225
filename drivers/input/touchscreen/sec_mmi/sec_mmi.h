#ifndef _SEC_MMI_H_
#define _SEC_MMI_H_

#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/notifier.h>
#include <linux/msm_drm_notify.h>

#define TSDEV_MINOR_BASE 128
#define TSDEV_MINOR_MAX 32

struct sec_mmi_data {
	struct i2c_client *i2c_client;
	struct device *ts_class_dev;
	struct notifier_block panel_nb;
	struct work_struct resume_work;

	int ctrl_dsi;

	char *image_name;
	const char *class_entry_name;
	const char *bound_display;
};

#endif
