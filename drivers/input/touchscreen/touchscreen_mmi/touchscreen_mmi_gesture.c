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
#include <linux/touchscreen_mmi.h>
#include <linux/sensors.h>

struct ts_mmi_sensor_platform_data {
	struct input_dev *input_sensor_dev;
	struct sensors_classdev ps_cdev;
	int sensor_opened;
	char sensor_data; /* 0 near, 1 far */
	struct ts_mmi_dev *touch_cdev;
};

static struct ts_mmi_sensor_platform_data *sensor_pdata;

static int ts_mmi_gesture_handler(struct gesture_event_data *gev)
{
	int key_code;
	bool need2report = true;

	switch (gev->evcode) {
	case 1:
		key_code = KEY_F1;
		pr_info("%s: single tap\n", __func__);
			break;
	case 2:
		key_code = KEY_F2;
		input_report_abs(sensor_pdata->input_sensor_dev, ABS_X, gev->evdata.x);
		input_report_abs(sensor_pdata->input_sensor_dev, ABS_Y, gev->evdata.y);
		pr_info("%s: zero tap; x=%x, y=%x\n", __func__, gev->evdata.x, gev->evdata.y);
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

static int ts_mmi_touch_event_handler(struct touch_event_data *tev)
{
	switch (tev->type) {
	case TS_COORDINATE_ACTION_PRESS:
#if defined(CONFIG_TOUCHCLASS_MMI_DEBUG_INFO)
		pr_info("%s: [P]Finger %d: Down, x=%d, y=%d, major=%d, minor=%d",
				__func__, tev->id, tev->x, tev->y, tev->major, tev->minor);
#endif
		break;

	case TS_COORDINATE_ACTION_RELEASE:
#if defined(CONFIG_TOUCHCLASS_MMI_DEBUG_INFO)
		pr_info("%s: [R]Finger %d: UP", __func__, tev->id);
#endif
		break;

	default:
		pr_info("%s: unsupport type=%d\n", __func__, tev->type);

	}

	return 0;
}

bool ts_mmi_is_sensor_enable(void)
{
	struct ts_mmi_dev *touch_cdev;
	if (sensor_pdata != NULL) {
		touch_cdev = sensor_pdata->touch_cdev;
		return !!sensor_pdata->sensor_opened;
	}
	else
		return false;
}

static int ts_mmi_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
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
		goto free_sensor_pdata;
	}

	__set_bit(EV_KEY, sensor_input_dev->evbit);
	__set_bit(KEY_F1, sensor_input_dev->keybit);
	__set_bit(KEY_F2, sensor_input_dev->keybit);
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
		goto free_sensor_input_dev;
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
free_sensor_input_dev:
	input_free_device(sensor_input_dev);
free_sensor_pdata:
	devm_kfree(&sensor_input_dev->dev, sensor_pdata);
exit:
	return 1;
}

int ts_mmi_gesture_remove(struct ts_mmi_dev *touch_cdev)
{
	sensors_classdev_unregister(&sensor_pdata->ps_cdev);
	input_unregister_device(sensor_pdata->input_sensor_dev);
	devm_kfree(&sensor_pdata->input_sensor_dev->dev, sensor_pdata);
	sensor_pdata = NULL;

	return 0;
}
