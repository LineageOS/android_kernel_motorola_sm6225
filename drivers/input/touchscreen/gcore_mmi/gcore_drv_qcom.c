/*
 * GalaxyCore touchscreen driver
 *
 * Copyright (C) 2021 GalaxyCore Incorporated
 *
 * Copyright (C) 2021 Neo Chen <neo_chen@gcoreinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "gcore_drv_common.h"
#include <linux/interrupt.h>

#ifdef CONFIG_DRM
#include <drm/drm_panel.h>
#endif

#if defined(CONFIG_ENABLE_GESTURE_WAKEUP) && defined(CONFIG_GESTURE_SPECIAL_INT)

int gcore_enable_irq_wake(struct gcore_dev *gdev)
{
	GTP_DEBUG("enable irq wake");

	if (gdev->ges_irq) {
		enable_irq_wake(gdev->ges_irq);
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL(gcore_enable_irq_wake);

void gcore_ges_irq_enable(struct gcore_dev *gdev)
{
	unsigned long flags;

	GTP_DEBUG("enable ges irq");

	spin_lock_irqsave(&gdev->irq_flag_lock, flags);

	if (gdev->ges_irq_en == false) {
		gdev->ges_irq_en = true;
		spin_unlock_irqrestore(&gdev->irq_flag_lock, flags);
		enable_irq(gdev->ges_irq);
	} else if (gdev->ges_irq_en == true) {
		spin_unlock_irqrestore(&gdev->irq_flag_lock, flags);
		GTP_ERROR("gesture Eint already enabled!");
	} else {
		spin_unlock_irqrestore(&gdev->irq_flag_lock, flags);
		GTP_ERROR("Invalid irq_flag %d!", gdev->irq_flag);
	}
	/*GTP_DEBUG("Enable irq_flag=%d",g_touch.irq_flag);*/

}

void gcore_ges_irq_disable(struct gcore_dev *gdev)
{
	unsigned long flags;

	GTP_DEBUG("disable ges irq");

	spin_lock_irqsave(&gdev->irq_flag_lock, flags);

	if (gdev->ges_irq_en == false) {
		spin_unlock_irqrestore(&gdev->irq_flag_lock, flags);
		GTP_ERROR("gesture Eint already disable!");
		return;
	}

	gdev->ges_irq_en = false;

	spin_unlock_irqrestore(&gdev->irq_flag_lock, flags);

	disable_irq_nosync(gdev->ges_irq);
}
EXPORT_SYMBOL(gcore_ges_irq_disable);
#endif

void gcore_suspend(void)
{
	//struct gcore_dev *gdev = fn_data.gdev;

	GTP_DEBUG("enter gcore suspend");

#if defined(CONFIG_ENABLE_GESTURE_WAKEUP) && defined(CONFIG_GESTURE_SPECIAL_INT)
	struct gcore_dev *gdev = fn_data.gdev;
	if (fn_data.gdev->gesture_wakeup_en) {
		gcore_enable_irq_wake(gdev);

		gcore_irq_disable(gdev);

		//sprd_pin_set(&gdev->bus_device->dev, "gpio_144_slp");

		gcore_ges_irq_enable(gdev);
	}
#endif

	fn_data.gdev->tp_suspend = true;

	msleep(20);

}

void gcore_resume(void)
{
	//struct gcore_dev *gdev = fn_data.gdev;

	GTP_DEBUG("enter gcore resume");

#if defined(CONFIG_ENABLE_GESTURE_WAKEUP) && defined(CONFIG_GESTURE_SPECIAL_INT)
	struct gcore_dev *gdev = fn_data.gdev;
	if (gdev->gesture_wakeup_en) {
		GTP_DEBUG("disable irq wake");

		gcore_ges_irq_disable(gdev);

		//sprd_pin_set(&gdev->bus_device->dev, "gpio_144");

		gcore_irq_enable(gdev);
	} else {
		queue_delayed_work(fn_data.gdev->fwu_workqueue, &fn_data.gdev->fwu_work, \
				msecs_to_jiffies(100));
	}
#else
	//queue_delayed_work(fn_data.gdev->fwu_workqueue, &fn_data.gdev->fwu_work, msecs_to_jiffies(1000));
	gcore_request_firmware_update_work(NULL);
#endif

	gcore_touch_release_all_point(fn_data.gdev->input_device);

	fn_data.gdev->tp_suspend = false;

}


#ifdef CONFIG_DRM
int gcore_ts_drm_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	unsigned int blank;

	struct drm_panel_notifier *evdata = data;

	if (!evdata)
		return 0;

	blank = *(int *) (evdata->data);
	GTP_DEBUG("event = %lu, blank = %d", event, blank);

	if (!(event == DRM_PANEL_EARLY_EVENT_BLANK || event == DRM_PANEL_EVENT_BLANK)) {
		GTP_DEBUG("event(%lu) do not need process\n", event);
		return 0;
	}

	switch (blank) {
	case DRM_PANEL_BLANK_POWERDOWN:
		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			gcore_suspend();
		}
		break;

	case DRM_PANEL_BLANK_UNBLANK:
		if (event == DRM_PANEL_EVENT_BLANK) {
			gcore_resume();
		}
		break;

	default:
		break;
	}
	return 0;
}
#endif

static int __init touch_driver_init(void)
{
	GTP_DEBUG("touch driver init.");

	if (gcore_touch_bus_init()) {
		GTP_ERROR("bus init fail!");
		return -1;
	}

	return 0;
}

/* should never be called */
static void __exit touch_driver_exit(void)
{
	gcore_touch_bus_exit();
}



module_init(touch_driver_init);
module_exit(touch_driver_exit);
#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
MODULE_AUTHOR("GalaxyCore, Inc.");
MODULE_DESCRIPTION("GalaxyCore Touch Main Mudule");
MODULE_LICENSE("GPL");
