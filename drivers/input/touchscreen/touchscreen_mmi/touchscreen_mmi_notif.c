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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/touchscreen_mmi.h>

#if defined(CONFIG_PANEL_NOTIFICATIONS)
#define register_panel_notifier panel_register_notifier
#define unregister_panel_notifier panel_unregister_notifier
#else
#define register_panel_notifier(...) ret
#define unregister_panel_notifier(...)
#endif

#if defined(CONFIG_DRM_DYNAMIC_REFRESH_RATE)
extern struct blocking_notifier_head dsi_freq_head;
#define register_dynamic_refresh_rate_notifier(_nb) \
	blocking_notifier_chain_register(&dsi_freq_head, _nb)
#define unregister_dynamic_refresh_rate_notifier(_nb) \
	blocking_notifier_chain_unregister(&dsi_freq_head, _nb)
#else
#define register_dynamic_refresh_rate_notifier(...) ret
#define unregister_dynamic_refresh_rate_notifier(...)
#endif

#if defined(CONFIG_PANEL_NOTIFICATIONS)
static int ts_mmi_panel_off(struct ts_mmi_dev *touch_cdev) {
	if (atomic_cmpxchg(&touch_cdev->touch_stopped, 0, 1) == 1)
		return 0;

	TRY_TO_CALL(pre_suspend);

	if (touch_cdev->pdata.power_off_suspend)
		/* IC power is off. IRQ pin status is floated. So disable IRQ. */
		TRY_TO_CALL(drv_irq, TS_MMI_IRQ_OFF);
	else {
		if (touch_cdev->pdata.gestures_enabled)
			TRY_TO_CALL(panel_state, touch_cdev->pm_mode, TS_MMI_PM_GESTURE);
		else
			TRY_TO_CALL(panel_state, touch_cdev->pm_mode, TS_MMI_PM_DEEPSLEEP);
	}
	cancel_delayed_work_sync(&touch_cdev->resume_work);

	TRY_TO_CALL(post_suspend);

	dev_info(DEV_MMI, "%s: done\n", __func__);

	return 0;
}

static int inline ts_mmi_panel_on(struct ts_mmi_dev *touch_cdev) {
	/* schedule_delayed_work returns true if work has been scheduled */
	/* and false otherwise, thus return 0 on success to comply POSIX */
	return schedule_delayed_work(&touch_cdev->resume_work, 0) == false;
}

#endif

static int ts_mmi_panel_cb(struct notifier_block *nb,
		unsigned long event, void *evd)
{
	struct ts_mmi_dev *touch_cdev =
		container_of(nb, struct ts_mmi_dev, panel_nb);

	if (!touch_cdev)
		return 0;

	/* entering suspend upon early blank event */
	/* to ensure shared power supply is still on */
	/* for in-cell design touch solutions */
	switch (event) {
#if defined(CONFIG_PANEL_NOTIFICATIONS)
	case PANEL_EVENT_PRE_DISPLAY_OFF:
	/* put in reset first */
		if (touch_cdev->pdata.power_off_suspend &&
			touch_cdev->mdata->pinctrl)
			touch_cdev->mdata->pinctrl(DEV_TS, TS_MMI_PINCTL_OFF);
		ts_mmi_panel_off(touch_cdev);
		break;

	case PANEL_EVENT_DISPLAY_OFF:
		if (touch_cdev->pdata.power_off_suspend &&
			touch_cdev->mdata->power) {
			/* then proceed with de-powering */
			touch_cdev->mdata->power(DEV_TS, TS_MMI_POWER_OFF);
			dev_dbg(DEV_MMI, "%s: touch powered off\n", __func__);
		}
		break;

	case PANEL_EVENT_PRE_DISPLAY_ON:
		if (touch_cdev->pdata.power_off_suspend &&
			touch_cdev->mdata->power) {
			/* powering on early */
			touch_cdev->mdata->power(DEV_TS, TS_MMI_POWER_ON);
			dev_dbg(DEV_MMI, "%s: touch powered on\n", __func__);
		} else if (touch_cdev->pdata.reset &&
			touch_cdev->mdata->reset) {
			/* Power is not off in previous suspend.
			 * But need reset IC in resume.
			 */
			dev_dbg(DEV_MMI, "%s: resetting...\n", __func__);
			touch_cdev->mdata->reset(DEV_TS, TS_MMI_RESET_HARD);
		}
		break;

	case PANEL_EVENT_DISPLAY_ON:
		/* out of reset to allow wait for boot complete */
		if (touch_cdev->pdata.power_off_suspend &&
			touch_cdev->mdata->pinctrl)
			touch_cdev->mdata->pinctrl(DEV_TS, TS_MMI_PINCTL_ON);
		ts_mmi_panel_on(touch_cdev);
		break;
#endif
	default:/* use DEV_TS here to avoid unused variable */
		dev_dbg(DEV_TS, "%s: function not implemented\n", __func__);
		break;
	}

	return 0;
}

static void ts_mmi_queued_resume(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct ts_mmi_dev *touch_cdev =
		container_of(dw, struct ts_mmi_dev, resume_work);

	bool wait4_boot_complete = true;

	if (atomic_cmpxchg(&touch_cdev->touch_stopped, 1, 0) == 0)
		return;

	TRY_TO_CALL(pre_resume);
	if (touch_cdev->pdata.power_off_suspend) {
		/* power turn on in PANEL_EVENT_PRE_DISPLAY_ON.
		 * IC need some time to boot up.
		 * Check IC is ready or not.
		 */
		TRY_TO_CALL(wait_for_ready);
	} else if (!touch_cdev->pdata.reset) {
		/* IC power is not down in suspend.
		 * IC also do not need reset in resume.
		 * So IC RAM is not lost, just change IC working mode to normal mode.
		 */
		TRY_TO_CALL(panel_state, touch_cdev->pm_mode, TS_MMI_PM_ACTIVE);
		wait4_boot_complete = false;
	}

	if (wait4_boot_complete) {
		/* IC is just power on or reseted.
		 * Need setup post resume work.
		 */
		if (touch_cdev->pdata.power_off_suspend)
			TRY_TO_CALL(drv_irq, TS_MMI_IRQ_ON);
	}

	TRY_TO_CALL(post_resume);

	touch_cdev->pm_mode = TS_MMI_PM_ACTIVE;

	dev_info(DEV_MMI, "%s: done\n", __func__);
}

static int ts_mmi_refresh_rate_cb(struct notifier_block *nb,
		unsigned long refresh_rate, void *ptr)
{
	struct ts_mmi_dev *touch_cdev =
		container_of(nb, struct ts_mmi_dev, freq_nb);
	bool do_calibration = false;

	if (!touch_cdev)
		return 0;

	if (!touch_cdev->refresh_rate ||
		(touch_cdev->refresh_rate != (refresh_rate & 0xFF))) {
		touch_cdev->refresh_rate = refresh_rate & 0xFF;
		do_calibration = true;
	}

	if (do_calibration)
		TRY_TO_CALL(refresh_rate, (int)touch_cdev->refresh_rate);

	return 0;
}

int ts_mmi_notifiers_register(struct ts_mmi_dev *touch_cdev) {
	int ret = 0;

	if (!touch_cdev->class_dev) {
		return -ENODEV;
	}

	dev_info(DEV_TS, "%s: Start notifiers init.\n", __func__);

	INIT_DELAYED_WORK(&touch_cdev->resume_work, ts_mmi_queued_resume);
	touch_cdev->panel_nb.notifier_call = ts_mmi_panel_cb;
	ret = register_panel_notifier(&touch_cdev->panel_nb);
	if (ret)
		goto PANEL_NOTIF_REGISTER_FAILED;
	if (touch_cdev->pdata.update_refresh_rate) {
		touch_cdev->freq_nb.notifier_call = ts_mmi_refresh_rate_cb;
		ret = register_dynamic_refresh_rate_notifier(&touch_cdev->freq_nb);
		if (ret)
			goto FREQ_NOTIF_REGISTER_FAILED;
	}

	dev_info(DEV_TS, "%s: Notifiers init OK.\n", __func__);
	return 0;

FREQ_NOTIF_REGISTER_FAILED:
	unregister_panel_notifier(&touch_cdev->panel_nb);
PANEL_NOTIF_REGISTER_FAILED:
	cancel_delayed_work(&touch_cdev->resume_work);
	return ret;
}

void ts_mmi_notifiers_unregister(struct ts_mmi_dev *touch_cdev) {
	if(touch_cdev->class_dev == NULL) {
		dev_err(DEV_MMI, "%s:touch_cdev->class_dev == NULL", __func__);
	} else {
		if (touch_cdev->pdata.update_refresh_rate) {
			unregister_dynamic_refresh_rate_notifier(&touch_cdev->freq_nb);
		}
		unregister_panel_notifier(&touch_cdev->panel_nb);
		cancel_delayed_work(&touch_cdev->resume_work);
		dev_info(DEV_MMI, "%s:notifiers_unregister finish", __func__);
	}
}
