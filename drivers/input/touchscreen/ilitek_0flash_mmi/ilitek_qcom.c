/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"

#ifdef ILI_CONFIG_PANEL_NOTIFICATIONS
#define register_panel_notifier panel_register_notifier
#define unregister_panel_notifier panel_unregister_notifier
enum touch_state {
	TOUCH_DEEP_SLEEP_STATE = 0,
	TOUCH_LOW_POWER_STATE,
};
#else
#define register_panel_notifier(...) rc
#define unregister_panel_notifier(...) rc
#endif

#ifdef ILI_SENSOR_EN
static struct sensors_classdev __maybe_unused sensors_touch_cdev = {

	.name = "dt-gesture",
	.vendor = "ilitek",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_DOUBLE_TAP,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "1",
	.min_delay = 0,
	.max_delay = 0,
	/* WAKE_UP & SPECIAL_REPORT */
	.flags = 1 | 6,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

#ifdef ILI_CONFIG_PANEL_NOTIFICATIONS
static int ili_panel_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME	"tchip,ilitek"

void ili_tp_reset(void)
{
	ILI_INFO("edge delay = %d\n", ilits->rst_edge_delay);

	/* Need accurate power sequence, do not change it to msleep */
	gpio_direction_output(ilits->tp_rst, 1);
	mdelay(1);
	gpio_set_value(ilits->tp_rst, 0);
	mdelay(5);
	gpio_set_value(ilits->tp_rst, 1);
	mdelay(ilits->rst_edge_delay);
}

void ili_input_register(void)
{
	ILI_INFO();

	ilits->input = input_allocate_device();
	if (ERR_ALLOC_MEM(ilits->input)) {
		ILI_ERR("Failed to allocate touch input device\n");
		input_free_device(ilits->input);
		return;
	}

	ilits->input->name = ilits->hwif->name;
	ilits->input->phys = ilits->phys;
	ilits->input->dev.parent = ilits->dev;
	ilits->input->id.bustype = ilits->hwif->bus_type;

	/* set the supported event type for input device */
	set_bit(EV_ABS, ilits->input->evbit);
	set_bit(EV_SYN, ilits->input->evbit);
	set_bit(EV_KEY, ilits->input->evbit);
	set_bit(BTN_TOUCH, ilits->input->keybit);
	set_bit(BTN_TOOL_FINGER, ilits->input->keybit);
	set_bit(INPUT_PROP_DIRECT, ilits->input->propbit);

	input_set_abs_params(ilits->input, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, ilits->panel_wid - 1, 0, 0);
	input_set_abs_params(ilits->input, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, ilits->panel_hei - 1, 0, 0);
	input_set_abs_params(ilits->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ilits->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	if (MT_PRESSURE)
		input_set_abs_params(ilits->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (MT_B_TYPE) {
#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
		input_mt_init_slots(ilits->input, MAX_TOUCH_NUM, INPUT_MT_DIRECT);
#else
		input_mt_init_slots(ilits->input, MAX_TOUCH_NUM);
#endif /* LINUX_VERSION_CODE */
	} else {
		input_set_abs_params(ilits->input, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM, 0, 0);
	}

	/* Gesture keys register */
	input_set_capability(ilits->input, EV_KEY, KEY_POWER);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_O);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_E);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_M);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_W);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_S);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_V);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_C);
	input_set_capability(ilits->input, EV_KEY, KEY_GESTURE_F);

	__set_bit(KEY_GESTURE_POWER, ilits->input->keybit);
	__set_bit(KEY_GESTURE_UP, ilits->input->keybit);
	__set_bit(KEY_GESTURE_DOWN, ilits->input->keybit);
	__set_bit(KEY_GESTURE_LEFT, ilits->input->keybit);
	__set_bit(KEY_GESTURE_RIGHT, ilits->input->keybit);
	__set_bit(KEY_GESTURE_O, ilits->input->keybit);
	__set_bit(KEY_GESTURE_E, ilits->input->keybit);
	__set_bit(KEY_GESTURE_M, ilits->input->keybit);
	__set_bit(KEY_GESTURE_W, ilits->input->keybit);
	__set_bit(KEY_GESTURE_S, ilits->input->keybit);
	__set_bit(KEY_GESTURE_V, ilits->input->keybit);
	__set_bit(KEY_GESTURE_Z, ilits->input->keybit);
	__set_bit(KEY_GESTURE_C, ilits->input->keybit);
	__set_bit(KEY_GESTURE_F, ilits->input->keybit);

	/* register the input device to input sub-system */
	if (input_register_device(ilits->input) < 0) {
		ILI_ERR("Failed to register touch input device\n");
		input_unregister_device(ilits->input);
		input_free_device(ilits->input);
	}
}

#if REGULATOR_POWER
void ili_plat_regulator_power_on(bool status)
{
	ILI_INFO("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (ilits->vdd) {
			if (regulator_enable(ilits->vdd) < 0)
				ILI_ERR("regulator_enable VDD fail\n");
		}
		if (ilits->vcc) {
			if (regulator_enable(ilits->vcc) < 0)
				ILI_ERR("regulator_enable VCC fail\n");
		}
	} else {
		if (ilits->vdd) {
			if (regulator_disable(ilits->vdd) < 0)
				ILI_ERR("regulator_enable VDD fail\n");
		}
		if (ilits->vcc) {
			if (regulator_disable(ilits->vcc) < 0)
				ILI_ERR("regulator_enable VCC fail\n");
		}
	}
	atomic_set(&ilits->ice_stat, DISABLE);
	mdelay(5);
}

static void ilitek_plat_regulator_power_init(void)
{
	const char *vdd_name = "vdd";
	const char *vcc_name = "vcc";

	ilits->vdd = regulator_get(ilits->dev, vdd_name);
	if (ERR_ALLOC_MEM(ilits->vdd)) {
		ILI_ERR("regulator_get VDD fail\n");
		ilits->vdd = NULL;
	}
	if (regulator_set_voltage(ilits->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
		ILI_ERR("Failed to set VDD %d\n", VDD_VOLTAGE);

	ilits->vcc = regulator_get(ilits->dev, vcc_name);
	if (ERR_ALLOC_MEM(ilits->vcc)) {
		ILI_ERR("regulator_get VCC fail.\n");
		ilits->vcc = NULL;
	}
	if (regulator_set_voltage(ilits->vcc, VCC_VOLTAGE, VCC_VOLTAGE) < 0)
		ILI_ERR("Failed to set VCC %d\n", VCC_VOLTAGE);

	ili_plat_regulator_power_on(true);
}
#endif

void ili_irq_disable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&ilits->irq_spin, flag);

	if (atomic_read(&ilits->irq_stat) == DISABLE)
		goto out;

	if (!ilits->irq_num) {
		ILI_ERR("gpio_to_irq (%d) is incorrect\n", ilits->irq_num);
		goto out;
	}

	disable_irq_nosync(ilits->irq_num);
	atomic_set(&ilits->irq_stat, DISABLE);
	ILI_DBG("Disable irq success\n");

out:
	spin_unlock_irqrestore(&ilits->irq_spin, flag);
}

void ili_irq_enable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&ilits->irq_spin, flag);

	if (atomic_read(&ilits->irq_stat) == ENABLE)
		goto out;

	if (!ilits->irq_num) {
		ILI_ERR("gpio_to_irq (%d) is incorrect\n", ilits->irq_num);
		goto out;
	}

	enable_irq(ilits->irq_num);
	atomic_set(&ilits->irq_stat, ENABLE);
	ILI_DBG("Enable irq success\n");

out:
	spin_unlock_irqrestore(&ilits->irq_spin, flag);
}

static irqreturn_t ilitek_plat_isr_top_half(int irq, void *dev_id)
{
	if (irq != ilits->irq_num) {
		ILI_ERR("Incorrect irq number (%d)\n", irq);
		return IRQ_NONE;
	}

	if (atomic_read(&ilits->cmd_int_check) == ENABLE) {
		atomic_set(&ilits->cmd_int_check, DISABLE);
		ILI_DBG("CMD INT detected, ignore\n");
		wake_up(&(ilits->inq));
		return IRQ_HANDLED;
	}

	if (ilits->prox_near) {
		ILI_INFO("Proximity event, ignore interrupt!\n");
		return IRQ_HANDLED;
	}

	ILI_DBG("report: %d, rst: %d, fw: %d, switch: %d, mp: %d, sleep: %d, esd: %d\n",
			ilits->report,
			atomic_read(&ilits->tp_reset),
			atomic_read(&ilits->fw_stat),
			atomic_read(&ilits->tp_sw_mode),
			atomic_read(&ilits->mp_stat),
			atomic_read(&ilits->tp_sleep),
			atomic_read(&ilits->esd_stat));

	if (!ilits->report || atomic_read(&ilits->tp_reset) ||
		atomic_read(&ilits->fw_stat) || atomic_read(&ilits->tp_sw_mode) ||
		atomic_read(&ilits->mp_stat) || atomic_read(&ilits->tp_sleep) ||
		atomic_read(&ilits->esd_stat)) {
			ILI_DBG("ignore interrupt !\n");
			return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	if (mutex_is_locked(&ilits->touch_mutex)) {
		ILI_DBG("touch is locked, ignore\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&ilits->touch_mutex);
	ili_report_handler();
	mutex_unlock(&ilits->touch_mutex);
	return IRQ_HANDLED;
}

void ili_irq_unregister(void)
{
	devm_free_irq(ilits->dev, ilits->irq_num, NULL);
}

int ili_irq_register(int type)
{
	int ret = 0;
	static bool get_irq_pin;

	atomic_set(&ilits->irq_stat, DISABLE);

	if (get_irq_pin == false) {
		ilits->irq_num  = gpio_to_irq(ilits->tp_int);
		get_irq_pin = true;
	}

	ILI_INFO("ilits->irq_num = %d\n", ilits->irq_num);

	ret = devm_request_threaded_irq(ilits->dev, ilits->irq_num,
				   ilitek_plat_isr_top_half,
				   ilitek_plat_isr_bottom_half,
				   type | IRQF_ONESHOT, "ilitek", NULL);

	if (type == IRQF_TRIGGER_FALLING)
		ILI_INFO("IRQ TYPE = IRQF_TRIGGER_FALLING\n");
	if (type == IRQF_TRIGGER_RISING)
		ILI_INFO("IRQ TYPE = IRQF_TRIGGER_RISING\n");

	if (ret != 0)
		ILI_ERR("Failed to register irq handler, irq = %d, ret = %d\n", ilits->irq_num, ret);

	atomic_set(&ilits->irq_stat, ENABLE);

	return ret;
}

#if defined(CONFIG_FB)
static int ilitek_plat_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ILI_INFO("Notifier's event = %ld\n", event);

	/*
	 *	FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *	FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data) {
		blank = evdata->data;
		switch (*blank) {
#ifdef CONFIG_DRM_MSM
		case MSM_DRM_BLANK_POWERDOWN:
#else
		case FB_BLANK_POWERDOWN:
#endif
#if CONFIG_PLAT_SPRD
		case DRM_MODE_DPMS_OFF:
#endif /* CONFIG_PLAT_SPRD */
			if (TP_SUSPEND_PRIO) {
#ifdef CONFIG_DRM_MSM
				if (event != MSM_DRM_EARLY_EVENT_BLANK)
#else
				if (event != FB_EARLY_EVENT_BLANK)
#endif
					return NOTIFY_DONE;
			} else {
#ifdef CONFIG_DRM_MSM
				if (event != MSM_DRM_EVENT_BLANK)
#else
				if (event != FB_EVENT_BLANK)
#endif
					return NOTIFY_DONE;
			}
			if (ili_sleep_handler(TP_SUSPEND) < 0)
				ILI_ERR("TP suspend failed\n");
			break;
#ifdef CONFIG_DRM_MSM
		case MSM_DRM_BLANK_UNBLANK:
		case MSM_DRM_BLANK_NORMAL:
#else
		case FB_BLANK_UNBLANK:
		case FB_BLANK_NORMAL:
#endif

#if CONFIG_PLAT_SPRD
		case DRM_MODE_DPMS_ON:
#endif /* CONFIG_PLAT_SPRD */

#ifdef CONFIG_DRM_MSM
			if (event == MSM_DRM_EVENT_BLANK)
#else
			if (event == FB_EVENT_BLANK)
#endif
			{
				if (ili_sleep_handler(TP_RESUME) < 0)
					ILI_ERR("TP resume failed\n");

			}
			break;
		default:
			ILI_ERR("Unknown event, blank = %d\n", *blank);
			break;
		}
	}
	return NOTIFY_OK;
}
#endif
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;

int ili_drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        ILI_ERR("find drm_panel count(%d) fail", count);
        return 0;
    }

    ILI_INFO("find drm_panel count(%d) ", count);
    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        ILI_INFO("node%p", node);
        panel = of_drm_find_panel(node);
        ILI_INFO("panel%p ", panel);

        of_node_put(node);
        if (!IS_ERR(panel)) {
            ILI_INFO("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

	ILI_ERR("no find drm_panel");
    return -ENODEV;
}
int ili_check_default_tp(struct device_node *dt, const char *prop)
{
	const char *active_tp;
	const char *compatible;
	char *start;
	int ret;

	ret = of_property_read_string(dt->parent, prop, &active_tp);
	if (ret) {
		ILI_ERR(" %s:fail to read %s %d\n", __func__, prop, ret);
		return -ENODEV;
	}
    ILI_INFO("active_tp %s", active_tp);
	ret = of_property_read_string(dt, "compatible", &compatible);
	if (ret < 0) {
		ILI_ERR(" %s:fail to read %s %d\n", __func__, "compatible", ret);
		return -ENODEV;
	}
    ILI_INFO("compatible %s", compatible);
	start = strnstr(active_tp, compatible, strlen(active_tp));
	if (start == NULL) {
		ILI_ERR(" %s:no match compatible, %s, %s\n",
			__func__, compatible, active_tp);
		ret = -ENODEV;
	}

	return ret;
}

#ifndef ILI_CONFIG_PANEL_NOTIFICATIONS
static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;

    if (!evdata) {
        ILI_ERR("evdata is null");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
          || (event == DRM_PANEL_EVENT_BLANK))) {
        ILI_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    ILI_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case DRM_PANEL_BLANK_UNBLANK:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            	ILI_INFO("resume: event = %lu, not care\n", event);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
		ILI_INFO("suspend: event = %lu, TP_RESUME\n", event);
		if (ili_sleep_handler(TP_RESUME) < 0)
			ILI_ERR("TP resume failed\n");
        }
        break;
    case DRM_PANEL_BLANK_POWERDOWN:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
		ILI_INFO("suspend: event = %lu, TP_SUSPEND\n", event);
			if (ili_sleep_handler(TP_SUSPEND) < 0)
				ILI_ERR("TP suspend failed\n");
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            	ILI_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        ILI_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#endif
#endif
#if defined(CONFIG_DRM_MSM)
static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct msm_drm_notifier *evdata = data;
    int *blank = NULL;
    if (!evdata) {
        ILI_ERR("evdata is null");
        return 0;
    }

    if (!((event == MSM_DRM_EARLY_EVENT_BLANK )
          || (event == MSM_DRM_EVENT_BLANK))) {
        ILI_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    ILI_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case MSM_DRM_BLANK_UNBLANK:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            ILI_INFO("resume: event = %lu, not care\n", event);
        } else if (MSM_DRM_EVENT_BLANK == event) {
			if (ili_sleep_handler(TP_RESUME) < 0)
				ILI_ERR("TP resume failed\n");
        }
        break;
    case MSM_DRM_BLANK_POWERDOWN:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
 			if (ili_sleep_handler(TP_SUSPEND) < 0)
				ILI_ERR("TP suspend failed\n");
        } else if (MSM_DRM_EVENT_BLANK == event) {
            ILI_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        ILI_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#endif
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ilitek_plat_early_suspend(struct early_suspend *h)
{
	if (ili_sleep_handler(TP_SUSPEND) < 0)
		ILI_ERR("TP suspend failed\n");
}

static void ilitek_plat_late_resume(struct early_suspend *h)
{
	if (ili_sleep_handler(TP_RESUME) < 0)
		ILI_ERR("TP resume failed\n");
}
#endif

#ifdef ILI_SENSOR_EN
static int ili_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	ILI_INFO("Gesture set enable %d!", enable);
	mutex_lock(&ilits->state_mutex);
	if (enable == 1) {
		ilits->should_enable_gesture = true;
	} else if (enable == 0) {
		ilits->should_enable_gesture = false;
	} else {
		ILI_INFO("unknown enable symbol\n");
	}
	mutex_unlock(&ilits->state_mutex);
	return 0;
}

static int ili_sensor_init(struct ilitek_ts_data *data)
{
	struct ili_sensor_platform_data *sensor_pdata;
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		ILI_ERR("Failed to allocate device");
		goto exit;
	}

	sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
			sizeof(struct ili_sensor_platform_data),
			GFP_KERNEL);
	if (!sensor_pdata) {
		ILI_ERR("Failed to allocate memory");
		goto free_sensor_pdata;
	}
	data->sensor_pdata = sensor_pdata;

	if (data->report_gesture_key) {
		__set_bit(EV_KEY, sensor_input_dev->evbit);
		__set_bit(KEY_F1, sensor_input_dev->keybit);
	} else {
		__set_bit(EV_ABS, sensor_input_dev->evbit);
		input_set_abs_params(sensor_input_dev, ABS_DISTANCE,
				0, REPORT_MAX_COUNT, 0, 0);
	}
	__set_bit(EV_SYN, sensor_input_dev->evbit);

	sensor_input_dev->name = "double-tap";
	data->sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		ILI_ERR("Unable to register device, err=%d", err);
		goto free_sensor_input_dev;
	}

	sensor_pdata->ps_cdev = sensors_touch_cdev;
	sensor_pdata->ps_cdev.sensors_enable = ili_sensor_set_enable;
	sensor_pdata->data = data;

	err = sensors_classdev_register(&sensor_input_dev->dev,
				&sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(data->sensor_pdata->input_sensor_dev);
free_sensor_input_dev:
	input_free_device(data->sensor_pdata->input_sensor_dev);
free_sensor_pdata:
	devm_kfree(&sensor_input_dev->dev, sensor_pdata);
	data->sensor_pdata = NULL;
exit:
	return 1;
}

int ili_sensor_remove(struct ilitek_ts_data *data)
{
	sensors_classdev_unregister(&data->sensor_pdata->ps_cdev);
	input_unregister_device(data->sensor_pdata->input_sensor_dev);
	devm_kfree(&data->sensor_pdata->input_sensor_dev->dev,
		data->sensor_pdata);
	data->sensor_pdata = NULL;
	data->wakeable = false;
	data->should_enable_gesture = false;
	return 0;
}
#endif

#ifdef ILI_CONFIG_PANEL_NOTIFICATIONS
static int ili_panel_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
#ifdef ILI_SENSOR_EN
	struct ilitek_ts_data *ts =
		container_of(self, struct ilitek_ts_data, panel_notif);
#endif

	switch (event) {
	case PANEL_EVENT_PRE_DISPLAY_OFF:
			ILI_INFO("event=%lu\n", event);
			if (ili_sleep_handler(TP_SUSPEND) < 0)
				ILI_ERR("TP suspend failed\n");
#ifdef ILI_SENSOR_EN
			if (ts->should_enable_gesture) {
				ILI_INFO("double tap gesture suspend\n");
				touch_set_state(TOUCH_LOW_POWER_STATE, TOUCH_PANEL_IDX_PRIMARY);
			} else {
				touch_set_state(TOUCH_DEEP_SLEEP_STATE, TOUCH_PANEL_IDX_PRIMARY);
			}
#endif
			break;

	case PANEL_EVENT_DISPLAY_ON:
			ILI_INFO("event=%lu\n", event);
			if (ili_sleep_handler(TP_RESUME) < 0)
				ILI_ERR("TP resume failed\n");
			break;
	case PANEL_EVENT_DISPLAY_ON_PREPARE:
			ILI_INFO("event=%lu\n", event);
#if RESUME_BY_DDI
			ili_resume_by_ddi();
#endif
			break;
	default:	/* use DEV_TS here to avoid unused variable */
			ILI_INFO("%s: function not implemented event %lu\n", __func__, event);
			break;
	}

	return 0;
}

void ilitek_panel_notifier_unregister(void)
{
	if (register_panel_notifier(&ilits->panel_notif)) {
		ILI_ERR("unregister panel_notifier failed\n");
	}
}
#endif

static void ilitek_plat_sleep_init(void)
{
	int ret;
#if defined(CONFIG_FB)
	ILI_INFO("Init notifier_fb struct\n");
	ilits->notifier_fb.notifier_call = ilitek_plat_notifier_fb;
	#if CONFIG_PLAT_SPRD
		if (adf_register_client(&ilits->notifier_fb))
			ILI_ERR("Unable to register notifier_fb\n");
	#else
		if (fb_register_client(&ilits->notifier_fb))
			ILI_ERR("Unable to register notifier_fb\n");
	#endif /* CONFIG_PLAT_SPRD */
#endif
#ifdef ILI_CONFIG_PANEL_NOTIFICATIONS
	ilits->panel_notif.notifier_call = ili_panel_notifier_callback;
	ret = register_panel_notifier(&ilits->panel_notif);
	if(ret) {
		ILI_ERR("register panel_notifier failed. ret=%d\n", ret);
	}
#else
#if defined(CONFIG_DRM)
    	ilits->notifier_fb.notifier_call = drm_notifier_callback;
	#if defined(CONFIG_DRM_PANEL)
	if (active_panel) {
		ret = drm_panel_notifier_register(active_panel,&ilits->notifier_fb);
		if (ret)
		ILI_ERR("[DRM]drm_panel_notifier_register fail: %d\n", ret);
	}
	#endif
	#if defined(CONFIG_DRM_MSM)
	ret = msm_drm_register_client(&ilits->notifier_fb);
	if (ret) {
		ILI_ERR("[DRM]Unable to register fb_notifier: %d\n", ret);
	}
	#endif
#endif/* CONFIG_DRM */
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	ILI_ERR("Init eqarly_suspend struct\n");
	ilits->early_suspend.suspend = ilitek_plat_early_suspend;
	ilits->early_suspend.resume = ilitek_plat_late_resume;
	ilits->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&ilits->early_suspend);
#endif
}

static int ilitek_plat_gpio_register(void)
{
	int ret = 0;
	u32 flag;
	struct device_node *dev_node = ilits->dev->of_node;
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
	int err;
	ret = ili_drm_check_dt(dev_node);
	if (ret) {
            ILI_ERR("parse drm-panel fail");
            if(!ili_check_default_tp(dev_node,"tchip,ilitek"))
            {
                ILI_ERR("EPROBE_DEFER");
		err=-EPROBE_DEFER;
            }else{
                ILI_ERR("ENODEV");
                err= -ENODEV;
            }
            return err;
        }
#endif
#endif
#ifdef ILI_SENSOR_EN
	if (of_property_read_bool(dev_node, "ilitek,report_gesture_key")) {
		ILI_INFO("ilitek,report_gesture_key set");
		ilits->report_gesture_key = 1;
	} else {
		ilits->report_gesture_key = 0;
	}
#endif
	ilits->charger_detection_enable = of_property_read_bool(dev_node, "ilitek,usb_charger");
	if (!ilits->charger_detection_enable) {
		ILI_ERR("error reading ilitek,usb_charger. ilits->charger_detection_enable = %d\n",
			ilits->charger_detection_enable);
	} else {
		ILI_INFO("ilitek,usb_charger = %d\n", ilits->charger_detection_enable);
	}

	ilits->tp_int = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	ilits->tp_rst = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);

	ILI_INFO("TP INT: %d\n", ilits->tp_int);
	ILI_INFO("TP RESET: %d\n", ilits->tp_rst);

	if (!gpio_is_valid(ilits->tp_int)) {
		ILI_ERR("Invalid INT gpio: %d\n", ilits->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(ilits->tp_rst)) {
		ILI_ERR("Invalid RESET gpio: %d\n", ilits->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(ilits->tp_int, "TP_INT");
	if (ret < 0) {
		ILI_ERR("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(ilits->tp_int);
		ret = gpio_request(ilits->tp_int, "TP_INT");
		if (ret < 0) {
			ILI_ERR("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(ilits->tp_rst, "TP_RESET");
	if (ret < 0) {
		ILI_ERR("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(ilits->tp_rst);
		ret = gpio_request(ilits->tp_rst, "TP_RESET");
		if (ret < 0) {
			ILI_ERR("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

out:
	gpio_direction_input(ilits->tp_int);
	return ret;
}

static int ilitek_charger_notifier_callback(struct notifier_block *nb,
								unsigned long val, void *v)
{
	int ret = 0;
	struct power_supply *psy = NULL;
	union power_supply_propval prop;

	if(ilits->fw_update_stat != FW_UPDATE_PASS)
		return 0;

	psy= power_supply_get_by_name("usb");
	if (!psy) {
		ILI_ERR("Couldn't get usbpsy\n");
		return -EINVAL;
	}
	if (!strcmp(psy->desc->name, "usb")) {
		if (psy && val == POWER_SUPPLY_PROP_STATUS) {
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &prop);
			if (ret < 0) {
				ILI_ERR("Couldn't get POWER_SUPPLY_PROP_ONLINE rc=%d\n", ret);
				return ret;
			} else {
				if(prop.intval != ilits->usb_plug_status) {
					ILI_INFO("usb_plug_status =%d\n", prop.intval);
					ilits->usb_plug_status = prop.intval;
					if(ilits->charger_detection_enable && !ilits->tp_suspend &&
						(ilits->charger_notify_wq != NULL))
						queue_work(ilits->charger_notify_wq, &ilits->charger_notify_work);
				}
			}
		}
	}
	return 0;
}

static void ilitek_charger_notify_work(struct work_struct *work)
{
	int ret = 0;

	if (NULL == work) {
		ILI_ERR("%s:  parameter work are null!\n", __func__);
		return;
	}
	ILI_INFO("enter ilitek_charger_notify_work\n");

	mutex_lock(&ilits->touch_mutex);
	ret = ili_ic_func_ctrl("plug", !ilits->usb_plug_status);// plug in
	if(ret<0) {
		ILI_ERR("Write plug in failed\n");
	}
	mutex_unlock(&ilits->touch_mutex);
}

void ilitek_plat_charger_init(void)
{
	int ret = 0;
	struct power_supply *psy = NULL;
	union power_supply_propval prop;

	ilits->charger_notify_wq = create_singlethread_workqueue("ili_charger_wq");
	if (!ilits->charger_notify_wq) {
		ILI_ERR("allocate charger_notify_wq failed\n");
		return;
	}
	INIT_WORK(&ilits->charger_notify_work, ilitek_charger_notify_work);
	ilits->charger_notif.notifier_call = ilitek_charger_notifier_callback;
	ret = power_supply_reg_notifier(&ilits->charger_notif);
	if (ret < 0)
		ILI_ERR("power_supply_reg_notifier failed\n");

	/* if power supply supplier registered brfore TP
	* ps_notify_callback will not receive PSY_EVENT_PROP_ADDED
	* event, and will cause miss to set TP into charger state.
	* So check PS state in probe.
	*/
	psy = power_supply_get_by_name("usb");
	if (psy) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &prop);
		if (ret < 0) {
			ILI_ERR("Couldn't get POWER_SUPPLY_PROP_ONLINE rc=%d\n", ret);
		} else {
			ilits->usb_plug_status = prop.intval;
			ILI_INFO("boot check usb_plug_status = %d\n", prop.intval);
		}
	}
}

static int ilitek_plat_probe(void)
{
#ifdef ILI_SENSOR_EN
	static bool initialized_sensor;
#endif
	ILI_INFO("platform probe\n");

#if REGULATOR_POWER
	ilitek_plat_regulator_power_init();
#endif

	if (ilitek_plat_gpio_register() < 0)
		ILI_ERR("Register gpio failed\n");

	if (ilits->charger_detection_enable)
		ilitek_plat_charger_init();

	if (ili_tddi_init() < 0) {
		ILI_ERR("ILITEK Driver probe failed\n");
		if(gpio_is_valid(ilits->tp_int))
			gpio_free(ilits->tp_int);
		if(gpio_is_valid(ilits->tp_rst))
			gpio_free(ilits->tp_rst);
		if (ilits->charger_notif.notifier_call)
			power_supply_unreg_notifier(&ilits->charger_notif);
		if (ilits->charger_notify_wq)
			destroy_workqueue(ilits->charger_notify_wq);
		return -ENODEV;
	}

	ili_irq_register(ilits->irq_tirgger_type);
	ilitek_plat_sleep_init();
	ilits->pm_suspend = false;
	init_completion(&ilits->pm_completion);
#ifdef ILI_SENSOR_EN
	mutex_init(&ilits->state_mutex);
	//unknown screen state
	ilits->screen_state = SCREEN_UNKNOWN;
	if (!initialized_sensor) {
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_init(&(ilits->gesture_wakelock), WAKE_LOCK_SUSPEND, "dt-wake-lock");
#else
		PM_WAKEUP_REGISTER(ilits->dev, ilits->gesture_wakelock, "dt-wake-lock");
		if (!ilits->gesture_wakelock) {
			ILI_ERR("ILITEK Driver failed to load. wakeup_source_init failed.");
			return -ENOMEM;
		}
#endif
		if (!ili_sensor_init(ilits))
			initialized_sensor = true;
	}
#endif
	ILI_INFO("ILITEK Driver loaded successfully!");
	return 0;
}

static int ilitek_tp_pm_suspend(struct device *dev)
{
	ILI_INFO("CALL BACK TP PM SUSPEND");
	ilits->pm_suspend = true;
	reinit_completion(&ilits->pm_completion);
	return 0;
}

static int ilitek_tp_pm_resume(struct device *dev)
{
	ILI_INFO("CALL BACK TP PM RESUME");
	ilits->pm_suspend = false;
	complete(&ilits->pm_completion);
	return 0;
}

static int ilitek_plat_remove(void)
{
	ILI_INFO("remove plat dev\n");
#ifndef CONFIG_HAS_WAKELOCK
	if(ilits->gesture_wakelock)
		PM_WAKEUP_UNREGISTER(ilits->gesture_wakelock);
#endif
	ili_dev_remove();
	return 0;
}

static const struct dev_pm_ops tp_pm_ops = {
	.suspend = ilitek_tp_pm_suspend,
	.resume = ilitek_tp_pm_resume,
};

static const struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
	.bus_type = TDDI_INTERFACE,
	.plat_type = TP_PLAT_QCOM,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
	.of_match_table = of_match_ptr(tp_match_table),
	.plat_probe = ilitek_plat_probe,
	.plat_remove = ilitek_plat_remove,
	.pm = &tp_pm_ops,
};

static int __init ilitek_plat_dev_init(void)
{
	ILI_INFO("ILITEK TP driver init for QCOM\n");
	if (ili_dev_init(&hwif) < 0) {
		ILI_ERR("Failed to register i2c/spi bus driver\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ILI_INFO("remove plat dev\n");
	ili_dev_remove();
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
