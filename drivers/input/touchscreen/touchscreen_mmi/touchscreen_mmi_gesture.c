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
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/touchscreen_mmi.h>
#include <linux/sensors.h>

#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
/* Declare depended feature */
#ifndef TS_MMI_TOUCH_GESTURE_LOG_EVENT
#define TS_MMI_TOUCH_GESTURE_LOG_EVENT
#endif

#ifndef TS_MMI_TOUCH_GESTURE_REPORT_TOUCH_EVENT
#define TS_MMI_TOUCH_GESTURE_REPORT_TOUCH_EVENT
#endif

#ifndef TS_MMI_TOUCH_GESTURE_SUPPRESSION
#define TS_MMI_TOUCH_GESTURE_SUPPRESSION
#endif

#ifndef TS_MMI_TOUCH_GESTURE_GS_DISTANCE
#define TS_MMI_TOUCH_GESTURE_GS_DISTANCE
#endif
#endif /* TS_MMI_TOUCH_GESTURE_POISON_EVENT */

#ifdef TS_MMI_TOUCH_GESTURE_SUPPRESSION
/* Declare depended feature */
#ifndef TS_MMI_TOUCH_GESTURE_GS_DISTANCE
#define TS_MMI_TOUCH_GESTURE_GS_DISTANCE
#endif
#endif /* TS_MMI_TOUCH_GESTURE_SUPPRESSION */


struct ts_mmi_sensor_platform_data {
	struct input_dev *input_sensor_dev;
	struct sensors_classdev ps_cdev;
	int sensor_opened;
	char sensor_data; /* 0 near, 1 far */
	struct ts_mmi_dev *touch_cdev;
};

#ifdef TS_MMI_TOUCH_GESTURE_LOG_EVENT
struct touch_event_with_time_data {
	struct touch_event_data event;
	ktime_t time;
};
#endif

#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
struct touch_event_slot_poison_data {
	struct touch_event_with_time_data center;
	bool is_slot_poisoned;
};
#endif

struct ts_mmi_touch_events_data {
	struct ts_mmi_dev *touch_cdev;
#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
	/* Touch slot is poisoned, do not report this slot point until slot release or IC resumed. */
	struct touch_event_slot_poison_data poison_events[TS_MMI_MAX_POINT_NUM];
#endif
};

static struct ts_mmi_sensor_platform_data *sensor_pdata;
static struct ts_mmi_sensor_platform_data *palm_sensor_pdata;
static struct ts_mmi_touch_events_data *events_data;
static struct ts_mmi_sensor_platform_data *cli_sensor_pdata;

#ifdef TS_MMI_TOUCH_GESTURE_LOG_EVENT
static inline void ts_mmi_touch_log_event(struct touch_event_with_time_data *dst, const struct touch_event_data *src)
{
	memcpy(&dst->event, src, sizeof(struct touch_event_data));
	dst->time = ktime_get();
}

static inline void ts_mmi_touch_clear_event(struct touch_event_with_time_data *dst)
{
	memset(dst, 0, sizeof(struct touch_event_with_time_data));
}
#endif /* TS_MMI_TOUCH_GESTURE_LOG_EVENT */

#ifdef TS_MMI_TOUCH_GESTURE_REPORT_TOUCH_EVENT
static inline void ts_mmi_touch_event_report_event_release(int index, struct input_dev *input_dev)
{
	input_mt_slot(input_dev, index);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
}
#endif

#ifdef TS_MMI_TOUCH_GESTURE_GS_DISTANCE
static inline bool ts_mmi_touch_can_handle_gs_distance(void)
{
	struct ts_mmi_dev *touch_cdev = events_data->touch_cdev;

	return (touch_cdev->pdata.gs_distance_ctrl);
}

static inline int ts_mmi_touch_event_get_gs_distance(struct ts_mmi_dev *touch_cdev)
{
	return (ts_mmi_touch_can_handle_gs_distance()) ? touch_cdev->gs_distance : TOUCHSCREEN_MMI_DEFAULT_GS_DISTANCE;
}
#endif /* TS_MMI_TOUCH_GESTURE_GS_DISTANCE */

#ifdef TS_MMI_TOUCH_GESTURE_SUPPRESSION
static inline bool ts_mmi_touch_can_handle_suppression(void)
{
	struct ts_mmi_dev *touch_cdev = events_data->touch_cdev;

	return ((touch_cdev->pdata.suppression_ctrl) &&
		(touch_cdev->pdata.gs_distance_ctrl) &&
		(touch_cdev->pdata.max_x > 0) &&
		(touch_cdev->pdata.max_y > 0));
}

static inline bool ts_mmi_touch_event_is_in_suppression(struct touch_event_data *tev, struct ts_mmi_dev *touch_cdev)
{
	int gs_distance = ts_mmi_touch_event_get_gs_distance(touch_cdev);

	if (touch_cdev->suppression == TS_MMI_DISABLE_SUPPRESSION_ALL)
		return false;
	if (touch_cdev->suppression == TS_MMI_DISABLE_SUPPRESSION_LEFT)
		return (tev->x > (touch_cdev->pdata.max_x - gs_distance)) ? true : false;
	if (touch_cdev->suppression == TS_MMI_DISABLE_SUPPRESSION_RIGHT)
		return (tev->x < gs_distance) ? true : false;

	return ((tev->x < gs_distance) || (tev->x > (touch_cdev->pdata.max_x - gs_distance))) ? true : false;
}
#endif /* TS_MMI_TOUCH_GESTURE_SUPPRESSION */

#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
static inline bool ts_mmi_touch_event_rescue_slot(int slot_id)
{
	if (events_data->poison_events[slot_id].is_slot_poisoned) {
		ts_mmi_touch_clear_event(&events_data->poison_events[slot_id].center);
		events_data->poison_events[slot_id].is_slot_poisoned = false;
		return true;
	}
	return false;
}

static inline bool ts_mmi_touch_can_handle_poison_slot(void)
{
	struct ts_mmi_dev *touch_cdev = events_data->touch_cdev;

	return ((touch_cdev->pdata.suppression_ctrl) &&
		(touch_cdev->pdata.gs_distance_ctrl) &&
		(touch_cdev->pdata.poison_slot_ctrl) &&
		(touch_cdev->pdata.max_x > 0) &&
		(touch_cdev->pdata.max_y > 0));
}

static inline bool ts_mmi_touch_event_is_in_poison_trigger(struct touch_event_data *tev, struct ts_mmi_dev *touch_cdev)
{
	int trigger_distance = touch_cdev->poison_trigger_distance;

	if (touch_cdev->suppression == TS_MMI_DISABLE_SUPPRESSION_ALL)
		return false;
	if (touch_cdev->suppression == TS_MMI_DISABLE_SUPPRESSION_LEFT)
		return (tev->x > (touch_cdev->pdata.max_x - trigger_distance)) ? true : false;
	if (touch_cdev->suppression == TS_MMI_DISABLE_SUPPRESSION_RIGHT)
		return (tev->x < trigger_distance) ? true : false;

	return ((tev->x < trigger_distance) || (tev->x > (touch_cdev->pdata.max_x - trigger_distance))) ? true : false;
}

static inline bool need_handle_poison_event(int id)
{
	return ((events_data->poison_events[id].center.event.type == TS_COORDINATE_ACTION_PRESS) ||
		(events_data->poison_events[id].center.event.type == TS_COORDINATE_ACTION_MOVE));
}

static inline bool is_far_from_poison_center(struct touch_event_data *tev)
{
	int delta_y = abs(tev->y - events_data->poison_events[tev->id].center.event.y);
	return (delta_y > events_data->touch_cdev->poison_distance) ? true : false;
}

static inline bool is_stack_at_poison_center(struct touch_event_data *tev)
{
	if (events_data->poison_events[tev->id].center.time) {
		unsigned long long duration = timediff_ms(
			ktime_to_timespec(events_data->poison_events[tev->id].center.time),
			ktime_to_timespec(ktime_get()));

		return (duration > events_data->touch_cdev->poison_timeout) ? true : false;
	}

	return false;
}

static inline void update_poison_center(struct touch_event_data *tev)
{
	return ts_mmi_touch_log_event(&events_data->poison_events[tev->id].center, tev);
}
#endif /* TS_MMI_TOUCH_GESTURE_POISON_EVENT */

static int ts_mmi_gesture_handler(struct gesture_event_data *gev)
{
	int key_code;
	bool need2report = true;
	struct ts_mmi_dev *touch_cdev = sensor_pdata->touch_cdev;

	switch (gev->evcode) {
	case 1:
		key_code = KEY_F1;
		pr_info("%s: single tap\n", __func__);
			break;
	case 2:
		key_code = KEY_F2;
		if(gev->evdata.x == 0)
			gev->evdata.x = touch_cdev->pdata.fod_x ;
		if(gev->evdata.y== 0)
			gev->evdata.y = touch_cdev->pdata.fod_y;
		input_report_abs(sensor_pdata->input_sensor_dev, ABS_X, gev->evdata.x);
		input_report_abs(sensor_pdata->input_sensor_dev, ABS_Y, gev->evdata.y);
		pr_info("%s: zero tap; x=%x, y=%x\n", __func__, gev->evdata.x, gev->evdata.y);
		break;
	case 3:
		key_code = KEY_F3;
		pr_info("%s: zero tap up\n", __func__);
		break;
	case 4:
		key_code = KEY_F4;
		pr_info("%s: double tap\n", __func__);
		break;
	default:
		need2report = false;
		pr_info("%s: unknown id=%x\n", __func__, gev->evcode);
	}

	if (!need2report)
		return 1;

	input_report_key(sensor_pdata->input_sensor_dev, key_code, 1);
	input_sync(sensor_pdata->input_sensor_dev);
	input_report_key(sensor_pdata->input_sensor_dev, key_code, 0);
	input_sync(sensor_pdata->input_sensor_dev);

	return 0;
}

static int ts_mmi_palm_handler(bool value)
{
	if (!palm_sensor_pdata->input_sensor_dev)
		return 0;

	if (value) {
		input_report_abs(palm_sensor_pdata->input_sensor_dev,
				ABS_DISTANCE, 1);
		pr_info("%s: palm report 1\n", __func__);
	} else {
	input_report_abs(palm_sensor_pdata->input_sensor_dev,
				ABS_DISTANCE, 0);
		pr_info("%s: palm report 0\n", __func__);
	}
	input_sync(palm_sensor_pdata->input_sensor_dev);
	return 0;
}


#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
static int ts_mmi_touch_event_poison_slot_handler(struct touch_event_data *tev,  struct input_dev *input_dev)
{
	struct ts_mmi_dev *touch_cdev = events_data->touch_cdev;

	if (tev->type == TS_COORDINATE_ACTION_PRESS) {
		if (ts_mmi_touch_event_is_in_poison_trigger(tev, touch_cdev)) {
			update_poison_center(tev);
		}
	} else if (tev->type == TS_COORDINATE_ACTION_RELEASE) {
		if (need_handle_poison_event(tev->id)) {
			if (events_data->poison_events[tev->id].is_slot_poisoned) {
				if (ts_mmi_touch_event_rescue_slot(tev->id))
					pr_info("%s: Poison slot(%d) rescued\n", __func__, tev->id);
				tev->skip_report = true;
			}
		}
	} else if (tev->type == TS_COORDINATE_ACTION_MOVE) {
		if (need_handle_poison_event(tev->id)) {
			if (events_data->poison_events[tev->id].is_slot_poisoned) {
				if (ts_mmi_touch_event_is_in_poison_trigger(tev, touch_cdev)) {
					dev_dbg(DEV_TS, "%s: slot(%d) is poisoned\n", __func__, tev->id);
					tev->skip_report = true;
					tev->type = TS_COORDINATE_ACTION_NONE;
				} else {
					events_data->poison_events[tev->id].is_slot_poisoned = false;
					pr_info("%s: Poison slot(%d) temperary rescued because outof trigger area\n", __func__, tev->id);
				}
			} else if (is_far_from_poison_center(tev) && ts_mmi_touch_event_is_in_suppression(tev, touch_cdev)) {
				update_poison_center(tev);
			} else if (is_stack_at_poison_center(tev) && ts_mmi_touch_event_is_in_suppression(tev, touch_cdev)) {
				dev_info(DEV_TS, "%s: slot(%d) is stacked near poison center. Mark this slot poisoned\n", __func__, tev->id);
				ts_mmi_touch_event_report_event_release(tev->id, input_dev);
				events_data->poison_events[tev->id].is_slot_poisoned = true;
				tev->skip_report = true;
				tev->type = TS_COORDINATE_ACTION_NONE;
			}
		}
	}
	return 0;
}
#endif /* TS_MMI_TOUCH_GESTURE_POISON_EVENT */

#ifdef TS_MMI_TOUCH_EDGE_GESTURE
static int ts_mmi_touch_event_edge_handler(struct touch_event_data *tev,  struct input_dev *input_dev)
{
#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
	if (ts_mmi_touch_can_handle_poison_slot())
		ts_mmi_touch_event_poison_slot_handler(tev, input_dev);
#endif

	return 0;
}
#endif

static int ts_mmi_touch_event_handler(struct touch_event_data *tev,  struct input_dev *input_dev)
{
	struct ts_mmi_dev *touch_cdev = events_data->touch_cdev;
	__maybe_unused int ret = 0;

#ifdef TS_MMI_TOUCH_EDGE_GESTURE
	ts_mmi_touch_event_edge_handler(tev, input_dev);
#endif

	switch (tev->type) {
	case TS_COORDINATE_ACTION_PRESS:
#if defined(CONFIG_TOUCHCLASS_MMI_DEBUG_INFO)
		pr_info("%s: [P]Finger %d: Down, x=%d, y=%d, major=%d, minor=%d",
				__func__, tev->id, tev->x, tev->y, tev->major, tev->minor);
#endif
	case TS_COORDINATE_ACTION_MOVE:
		break;

	case TS_COORDINATE_ACTION_RELEASE:
#if defined(CONFIG_TOUCHCLASS_MMI_DEBUG_INFO)
		pr_info("%s: [R]Finger %d: UP", __func__, tev->id);
#endif

		if (touch_cdev->pdata.fps_detection) {
			if ((touch_cdev->delay_baseline_update) &&
				mutex_trylock(&touch_cdev->method_mutex)) {
				_TRY_TO_CALL(update_baseline, TS_MMI_UPDATE_BASELINE_ON);
				touch_cdev->delay_baseline_update = false;
				mutex_unlock(&touch_cdev->method_mutex);
			}
		}
		break;
	case TS_COORDINATE_ACTION_NONE:
		break;
	default:
		pr_info("%s: unsupport type=%d\n", __func__, tev->type);

	}

	return 0;
}

bool ts_mmi_is_sensor_enable(void)
{
	if (sensor_pdata != NULL) {
		return !!sensor_pdata->sensor_opened;
	}
	else
		return false;
}

static int ts_mmi_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
#if !defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL) && \
	!defined(CONFIG_BOARD_USES_CLI_DOUBLE_TAP_CTRL)
	struct ts_mmi_sensor_platform_data *sensor_pdata = container_of(
			sensors_cdev, struct ts_mmi_sensor_platform_data, ps_cdev);
	struct ts_mmi_dev *touch_cdev = sensor_pdata->touch_cdev;

	sensor_pdata->sensor_opened = enable;
	if (enable == 1) {
		dev_info(DEV_TS, "%s: sensor ENABLE\n", __func__);
	} else if (enable == 0) {
		dev_info(DEV_TS, "%s: sensor DISABLE\n", __func__);
	} else {
		dev_err(DEV_TS, "%s: unknown enable symbol\n", __func__);
	}
#endif
	return 0;
}

static int ts_mmi_palm_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ts_mmi_sensor_platform_data *sensor_pdata = container_of(
			sensors_cdev, struct ts_mmi_sensor_platform_data, ps_cdev);
	struct ts_mmi_dev *touch_cdev = sensor_pdata->touch_cdev;
	int ret = 0;

	TRY_TO_CALL(palm_set_enable, enable);
	if (enable == 1) {
		dev_info(DEV_TS, "%s: sensor ENABLE\n", __func__);
	} else if (enable == 0) {
		dev_info(DEV_TS, "%s: sensor DISABLE\n", __func__);
	} else {
		dev_err(DEV_TS, "%s: unknown enable symbol\n", __func__);
	}
	return 0;
}

static struct sensors_classdev __maybe_unused sensors_touch_cdev = {
	.name = "dt-gesture",
	.vendor = "Motorola",
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

static struct sensors_classdev __maybe_unused cli_sensors_touch_cdev = {
	.name = "s-dt-gesture",
	.vendor = "Motorola",
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

static struct sensors_classdev __maybe_unused palm_sensors_touch_cdev = {
	.name = "palm-gesture",
	.vendor = "Motorola",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_TOUCH_PALM,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "1",
	.min_delay = 0,
	.max_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#ifdef TS_MMI_TOUCH_EDGE_GESTURE
int ts_mmi_gesture_suspend(struct ts_mmi_dev *touch_cdev)
{
#ifdef TS_MMI_TOUCH_GESTURE_POISON_EVENT
	int i;
	for (i = 0; i < TS_MMI_MAX_POINT_NUM; i++)
		if (ts_mmi_touch_event_rescue_slot(i))
			pr_info("%s: Poison slot(%d) rescued\n", __func__, i);
#endif
	return 0;
}
#endif

int ts_mmi_gesture_init(struct ts_mmi_dev *touch_cdev)
{
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		dev_err(DEV_TS, "%s: Failed to allocate device", __func__);
		goto exit;
	}

	sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
			sizeof(struct ts_mmi_sensor_platform_data), GFP_KERNEL);
	if (!sensor_pdata) {
		dev_err(DEV_TS, "%s: Failed to allocate memory", __func__);
		goto free_sensor_input_dev;
	}
	events_data = devm_kzalloc(DEV_TS,
			sizeof(struct ts_mmi_touch_events_data), GFP_KERNEL);
	if (!events_data) {
		dev_err(DEV_TS, "%s: Failed to allocate memory", __func__);
		goto free_sensor_pdata;
	}
	events_data->touch_cdev = touch_cdev;

	__set_bit(EV_KEY, sensor_input_dev->evbit);
	__set_bit(KEY_F1, sensor_input_dev->keybit);
	__set_bit(KEY_F2, sensor_input_dev->keybit);
	__set_bit(KEY_F3, sensor_input_dev->keybit);
	__set_bit(KEY_F4, sensor_input_dev->keybit);
	__set_bit(EV_ABS, sensor_input_dev->evbit);
	__set_bit(EV_SYN, sensor_input_dev->evbit);
	/* TODO: fill in real screen resolution */
	input_set_abs_params(sensor_input_dev, ABS_X, 0, 4096, 0, 0);
	input_set_abs_params(sensor_input_dev, ABS_Y, 0, 4096, 0, 0);

	sensor_input_dev->name = "double-tap";
	sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		dev_err(DEV_TS, "%s: Unable to register device, err=%d", __func__, err);
		goto free_touch_events_data;
	}

	sensor_pdata->ps_cdev = sensors_touch_cdev;
	sensor_pdata->ps_cdev.sensors_enable = ts_mmi_sensor_set_enable;
	sensor_pdata->touch_cdev = touch_cdev;

	err = sensors_classdev_register(&sensor_input_dev->dev,
				&sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

	/* export report gesture function to vendor */
	touch_cdev->mdata->exports.report_gesture = ts_mmi_gesture_handler;
	/* export report touch event function to vendor */
	touch_cdev->mdata->exports.report_touch_event = ts_mmi_touch_event_handler;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(sensor_input_dev);
	sensor_input_dev = NULL;
free_touch_events_data:
	if (events_data)
		devm_kfree(DEV_TS, events_data);
free_sensor_pdata:
	if (sensor_input_dev && sensor_pdata)
		devm_kfree(&sensor_input_dev->dev, sensor_pdata);
free_sensor_input_dev:
	if (sensor_input_dev)
		input_free_device(sensor_input_dev);
exit:
	return 1;
}

int ts_mmi_gesture_remove(struct ts_mmi_dev *touch_cdev)
{
	sensors_classdev_unregister(&sensor_pdata->ps_cdev);
	input_unregister_device(sensor_pdata->input_sensor_dev);
	devm_kfree(&sensor_pdata->input_sensor_dev->dev, sensor_pdata);
	devm_kfree(DEV_TS, events_data);
	sensor_pdata = NULL;
	events_data = NULL;

	return 0;
}

int ts_mmi_cli_gesture_init(struct ts_mmi_dev *touch_cdev)
{
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		dev_err(DEV_TS, "%s: Failed to allocate input device", __func__);
		goto exit;
	}

	cli_sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
			sizeof(struct ts_mmi_sensor_platform_data), GFP_KERNEL);
	if (!cli_sensor_pdata) {
		dev_err(DEV_TS, "%s: Failed to allocate memory", __func__);
		goto free_sensor_input_dev;
	}

	__set_bit(EV_KEY, sensor_input_dev->evbit);
	__set_bit(KEY_F1, sensor_input_dev->keybit);
	__set_bit(KEY_F2, sensor_input_dev->keybit);
	__set_bit(KEY_F3, sensor_input_dev->keybit);
	__set_bit(KEY_F4, sensor_input_dev->keybit);
	__set_bit(EV_ABS, sensor_input_dev->evbit);
	__set_bit(EV_SYN, sensor_input_dev->evbit);
	/* TODO: fill in real screen resolution */
	input_set_abs_params(sensor_input_dev, ABS_X, 0, 4096, 0, 0);
	input_set_abs_params(sensor_input_dev, ABS_Y, 0, 4096, 0, 0);

	sensor_input_dev->name = "s-double-tap";
	cli_sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		dev_err(DEV_TS, "%s: Unable to register device, err=%d", __func__, err);
		goto free_sensor_pdata;
	}

	cli_sensor_pdata->ps_cdev = cli_sensors_touch_cdev;
	cli_sensor_pdata->ps_cdev.sensors_enable = ts_mmi_sensor_set_enable;
	cli_sensor_pdata->touch_cdev = touch_cdev;

	err = sensors_classdev_register(&sensor_input_dev->dev,
				&cli_sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

	/* export report gesture function to vendor */
	touch_cdev->mdata->exports.report_gesture = ts_mmi_gesture_handler;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(sensor_input_dev);
	sensor_input_dev = NULL;
free_sensor_pdata:
	if (sensor_input_dev && cli_sensor_pdata)
		devm_kfree(&sensor_input_dev->dev, cli_sensor_pdata);
free_sensor_input_dev:
	if (sensor_input_dev)
		input_free_device(sensor_input_dev);
exit:
	return 1;
}

int ts_mmi_cli_gesture_remove(struct ts_mmi_dev *touch_cdev)
{
	sensors_classdev_unregister(&cli_sensor_pdata->ps_cdev);
	input_unregister_device(cli_sensor_pdata->input_sensor_dev);
	devm_kfree(&cli_sensor_pdata->input_sensor_dev->dev, cli_sensor_pdata);
	cli_sensor_pdata = NULL;

	return 0;
}

int ts_mmi_palm_init(struct ts_mmi_dev *touch_cdev)
{
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		dev_err(DEV_TS, "%s: Failed to allocate device", __func__);
		goto exit;
	}

	palm_sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
					sizeof(struct ts_mmi_sensor_platform_data),
					GFP_KERNEL);
	if (!palm_sensor_pdata) {
		dev_err(DEV_TS, "%s: Failed to allocate memory", __func__);
		goto free_sensor_pdata;
	}

	__set_bit(EV_ABS, sensor_input_dev->evbit);
	__set_bit(EV_SYN, sensor_input_dev->evbit);
	input_set_abs_params(sensor_input_dev, ABS_DISTANCE,
					0, 5, 0, 0);
	sensor_input_dev->name = "palm_detect";
	palm_sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		dev_err(DEV_TS, "Unable to register device, err=%d", err);
		goto free_sensor_input_dev;
	}

	palm_sensor_pdata->ps_cdev = palm_sensors_touch_cdev;
	palm_sensor_pdata->ps_cdev.sensors_enable = ts_mmi_palm_set_enable;
	palm_sensor_pdata->touch_cdev = touch_cdev;

	err = sensors_classdev_register(&sensor_input_dev->dev,
						&palm_sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

	/* export report gesture function to vendor */
	touch_cdev->mdata->exports.report_palm = ts_mmi_palm_handler;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(sensor_input_dev);
free_sensor_input_dev:
	input_free_device(sensor_input_dev);
free_sensor_pdata:
	devm_kfree(&sensor_input_dev->dev, palm_sensor_pdata);
	palm_sensor_pdata = NULL;
exit:
	return 1;
}

int ts_mmi_palm_remove(struct ts_mmi_dev *touch_cdev)
{
	sensors_classdev_unregister(&palm_sensor_pdata->ps_cdev);
	input_unregister_device(palm_sensor_pdata->input_sensor_dev);
	devm_kfree(&palm_sensor_pdata->input_sensor_dev->dev, palm_sensor_pdata);
	palm_sensor_pdata = NULL;

	return 0;
}
