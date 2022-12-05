/*
 * Goodix Gesture Module
 *
 * Copyright (C) 2019 - 2020 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/input/mt.h>
#include "goodix_ts_core.h"
#include "goodix_ts_mmi.h"

/*
 * struct gesture_module - gesture module data
 * @registered: module register state
 * @sysfs_node_created: sysfs node state
 * @gesture_type: valid gesture type, each bit represent one gesture type
 * @gesture_data: store latest gesture code get from irq event
 * @gesture_ts_cmd: gesture command data
 */
struct gesture_module {
	atomic_t registered;
	rwlock_t rwlock;
	struct goodix_ts_core *ts_core;
	u8 gesture_type;
	struct goodix_ext_module module;
};

static struct gesture_module *gsx_gesture; /*allocated in gesture init module*/
static bool module_initialized;
int goodix_gesture_enable(int enable)
{
	int ret = 0;

	if (!module_initialized)
		return 0;

	if (enable) {
		if (atomic_read(&gsx_gesture->registered))
			ts_info("gesture module has been already registered");
		else
			ret = goodix_register_ext_module_no_wait(&gsx_gesture->module);
		atomic_set(&gsx_gesture->registered, 1);
	} else {
		if (!atomic_read(&gsx_gesture->registered))
			ts_info("gesture module has been already unregistered");
		else
			ret = goodix_unregister_ext_module(&gsx_gesture->module);
		atomic_set(&gsx_gesture->registered, 0);
	}

	return ret;
}
static ssize_t gsx_double_type_show(struct goodix_ext_module *module,
		char *buf)
{
	struct gesture_module *gsx = module->priv_data;
	unsigned char type = gsx->ts_core->gesture_type;

	if (!gsx)
		return -EIO;

	if (atomic_read(&gsx->registered) == 0) {
		ts_err("gesture module is not registered");
		return 0;
	}

	return sprintf(buf, "%s\n",
			(type & GESTURE_DOUBLE_TAP) ? "enable" : "disable");
}

static ssize_t gsx_double_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	struct gesture_module *gsx = module->priv_data;

	if (!gsx)
		return -EIO;

	if (atomic_read(&gsx->registered) == 0) {
		ts_err("gesture module is not registered");
		return 0;
	}

	if (buf[0] == '1') {
		ts_info("enable double tap");
		gsx->ts_core->gesture_type |= GESTURE_DOUBLE_TAP;
	} else if (buf[0] == '0') {
		ts_info("disable double tap");
		gsx->ts_core->gesture_type &= ~GESTURE_DOUBLE_TAP;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t gsx_single_type_show(struct goodix_ext_module *module,
		char *buf)
{
	struct gesture_module *gsx = module->priv_data;
	unsigned char type = gsx->ts_core->gesture_type;

	if (!gsx)
		return -EIO;

	if (atomic_read(&gsx->registered) == 0) {
		ts_err("gesture module is not registered");
		return 0;
	}

	return sprintf(buf, "%s\n",
			(type & GESTURE_SINGLE_TAP) ? "enable" : "disable");
}

static ssize_t gsx_single_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	struct gesture_module *gsx = module->priv_data;

	if (!gsx)
		return -EIO;

	if (atomic_read(&gsx->registered) == 0) {
		ts_err("gesture module is not registered");
		return 0;
	}

	if (buf[0] == '1') {
		ts_info("enable single tap");
		gsx->ts_core->gesture_type |= GESTURE_SINGLE_TAP;
	} else if (buf[0] == '0') {
		ts_info("disable single tap");
		gsx->ts_core->gesture_type &= ~GESTURE_SINGLE_TAP;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t gsx_fod_type_show(struct goodix_ext_module *module,
		char *buf)
{
	struct gesture_module *gsx = module->priv_data;
	unsigned char type = gsx->ts_core->gesture_type;

	if (!gsx)
		return -EIO;

	if (atomic_read(&gsx->registered) == 0) {
		ts_err("gesture module is not registered");
		return 0;
	}

	return sprintf(buf, "%s\n",
			(type & GESTURE_FOD_PRESS) ? "enable" : "disable");
}

static ssize_t gsx_fod_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	struct gesture_module *gsx = module->priv_data;

	if (!gsx)
		return -EIO;

	if (atomic_read(&gsx->registered) == 0) {
		ts_err("gesture module is not registered");
		return 0;
	}

	if (buf[0] == '1') {
		ts_info("enable fod");
		gsx->ts_core->gesture_type |= GESTURE_FOD_PRESS;
	} else if (buf[0] == '0') {
		ts_info("disable fod");
		gsx->ts_core->gesture_type &= ~GESTURE_FOD_PRESS;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}
#if 0
/**
 * gsx_gesture_type_show - show valid gesture type
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to output buffer
 * Returns >=0 - succeed,< 0 - failed
 */
static ssize_t gsx_gesture_type_show(struct goodix_ext_module *module,
				char *buf)
{
	int count = 0, i, ret = 0;
	unsigned char *type;

	type = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!type)
		return -ENOMEM;
	read_lock(&gsx_gesture->rwlock);
	for (i = 0; i < 256; i++) {
		if (QUERYBIT(gsx_gesture->gesture_type, i)) {
			count += scnprintf(type + count,
					   PAGE_SIZE, "%02x,", i);
		}
	}
	if (count > 0)
		ret = scnprintf(buf, PAGE_SIZE, "%s\n", type);
	read_unlock(&gsx_gesture->rwlock);

	kfree(type);
	return ret;
}

/**
 * gsx_gesture_type_store - set vailed gesture
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to valid gesture type
 * @count: length of buf
 * Returns >0 - valid gestures, < 0 - failed
 */
static ssize_t gsx_gesture_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	int i;

	if (count <= 0 || count > 256 || buf == NULL) {
		ts_err("Parameter error");
		return -EINVAL;
	}

	write_lock(&gsx_gesture->rwlock);
	memset(gsx_gesture->gesture_type, 0, GSX_GESTURE_TYPE_LEN);
	for (i = 0; i < count; i++)
		gsx_gesture->gesture_type[buf[i]/8] |= (0x1 << buf[i]%8);
	write_unlock(&gsx_gesture->rwlock);

	return count;
}
#endif
static ssize_t gsx_gesture_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&gsx_gesture->registered));
}

static ssize_t gsx_gesture_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	bool val;
	int ret;

	ret = strtobool(buf, &val);
	if (ret < 0)
		return ret;

	if (val) {
		ret = goodix_gesture_enable(1);
		return ret ? ret : count;
	} else {
		ret = goodix_gesture_enable(0);
		return ret ? ret : count;
	}
}

static ssize_t gsx_gesture_data_show(struct goodix_ext_module *module,
				char *buf)
{
	ssize_t count;

	read_lock(&gsx_gesture->rwlock);
	count = scnprintf(buf, PAGE_SIZE, "gesture type code:0x%x\n",
			gsx_gesture->gesture_type);
	read_unlock(&gsx_gesture->rwlock);

	return count;
}

const struct goodix_ext_attribute gesture_attrs[] = {
#if 0
	__EXTMOD_ATTR(type, 0666, gsx_gesture_type_show,
		gsx_gesture_type_store),
#endif
	__EXTMOD_ATTR(enable, 0666, gsx_gesture_enable_show,
		gsx_gesture_enable_store),
	__EXTMOD_ATTR(data, 0444, gsx_gesture_data_show, NULL),
	__EXTMOD_ATTR(double_en, 0666,
			gsx_double_type_show, gsx_double_type_store),
	__EXTMOD_ATTR(single_en, 0666,
			gsx_single_type_show, gsx_single_type_store),
	__EXTMOD_ATTR(fod_en, 0666,
			gsx_fod_type_show, gsx_fod_type_store)
};

static int gsx_gesture_init(struct goodix_ts_core *cd,
		struct goodix_ext_module *module)
{
	struct gesture_module *gsx = module->priv_data;

	if (!cd || !cd->hw_ops->gesture) {
		ts_err("gesture unsupported");
		return -EINVAL;
	}

	ts_info("gesture switch: ON");
	ts_debug("enable all gesture type");

	gsx->ts_core = cd;
	/*enable all gesture wakeup by default */
	gsx->ts_core->gesture_type = GESTURE_SINGLE_TAP |GESTURE_FOD_PRESS | GESTURE_DOUBLE_TAP;
	cd->zerotap_data[0] = 0;
	//default on the fod event
	cd->fod_enable = true;
	atomic_set(&gsx_gesture->registered, 1);
	return 0;
}

static int gsx_gesture_exit(struct goodix_ts_core *cd,
		struct goodix_ext_module *module)
{
	if (!cd || !cd->hw_ops->gesture) {
		ts_err("gesture unsupported");
		return -EINVAL;
	}

	ts_info("gesture switch: OFF");
	ts_debug("disable all gesture type");
	cd->gesture_type = 0;
	atomic_set(&gsx_gesture->registered, 0);
	return 0;
}

/**
 * gsx_gesture_ist - Gesture Irq handle
 * This functions is excuted when interrupt happended and
 * ic in doze mode.
 *
 * @cd: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_CANCEL_IRQEVT  stop execute
 */
static int gsx_gesture_ist(struct goodix_ts_core *cd,
	struct goodix_ext_module *module)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_ts_event gs_event = {0};
	__maybe_unused int fodx, fody, overlay_area;
	int ret;
#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
	struct gesture_event_data mmi_event;
	static  unsigned  long  start = 0;
	int fod_down_interval = 0;
	int fod_down = cd->zerotap_data[0];
	int underwater_flag = 0;
#endif
	if (atomic_read(&cd->suspended) == 0 || cd->gesture_type == 0)
		return EVT_CONTINUE;

	if (cd->board_data.gesture_wait_pm) {
		PM_WAKEUP_EVENT(cd->gesture_wakelock, 3000);
		/* Waiting for pm resume completed */
		ret = wait_event_interruptible_timeout(cd->pm_wq, atomic_read(&cd->pm_resume), msecs_to_jiffies(700));
		if (!ret) {
			ts_err("system(spi) can't finished resuming procedure.");
			return IRQ_HANDLED;
		}
	}

	ret = hw_ops->event_handler(cd, &gs_event);
	if (ret) {
		ts_err("failed get gesture data");
		goto re_send_ges_cmd;
	}

	if (!(gs_event.event_type & EVENT_GESTURE)) {
		ts_err("invalid event type: 0x%x",
			gs_event.event_type);
		goto re_send_ges_cmd;
	}
	ts_debug("got  gesture type 0x%x", gs_event.gesture_type);
#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
	if (cd->set_mode.liquid_detection) {
		underwater_flag = gs_event.gesture_report_info & GOODIX_GESTURE_UNDER_WATER;
		if ( cd->liquid_status != underwater_flag) {
			cd->liquid_status = underwater_flag;
			/* call class method */
			cd->imports->report_liquid_detection_status(cd->bus->dev, cd->liquid_status? 1:0);
			ts_info("under water flag changed to: 0x%x\n", cd->liquid_status);
			goto gesture_ist_exit;
		}
	}

	mmi_event.evcode =0;
	if ( cd->imports && cd->imports->report_gesture) {
		if(cd->gesture_type & GESTURE_SINGLE_TAP && gs_event.gesture_type == GOODIX_GESTURE_SINGLE_TAP) {
			ts_info("get SINGLE-TAP gesture");
			mmi_event.evcode =1;
			fod_down = 0;

			/* call class method */
			ret = cd->imports->report_gesture(&mmi_event);
			if (!ret)
				PM_WAKEUP_EVENT(cd->gesture_wakelock, 3000);
			goto gesture_ist_exit;
		} else if(cd->gesture_type & GESTURE_DOUBLE_TAP && gs_event.gesture_type == GOODIX_GESTURE_DOUBLE_TAP) {
			ts_info("get DOUBLE-TAP gesture");
			mmi_event.evcode =4;
			fod_down = 0;

			/* call class method */
			ret = cd->imports->report_gesture(&mmi_event);
			if (!ret)
				PM_WAKEUP_EVENT(cd->gesture_wakelock, 3000);
			goto gesture_ist_exit;
		} else if(cd->gesture_type & GESTURE_FOD_PRESS && gs_event.gesture_type == GOODIX_GESTURE_FOD_DOWN ){
			fod_down_interval = (int)jiffies_to_msecs(jiffies-start);
			fodx = le16_to_cpup((__le16 *)gs_event.gesture_data);
			fody = le16_to_cpup((__le16 *)(gs_event.gesture_data + 2));
			overlay_area = gs_event.gesture_data[4];
			//goodix firmware do not send coordinate, need mmi touch to define a vaild coordinate thru dts
			mmi_event.evcode = 2;
			mmi_event.evdata.x= 0;
			mmi_event.evdata.y= 0;

			ts_info("Get FOD-DOWN gesture:%d interval:%d",fod_down,fod_down_interval);
			if(fod_down_interval > 2000)
				fod_down = 0;
			if(fod_down_interval > 0 && fod_down_interval < 250 && fod_down) {
					goto gesture_ist_exit;
			}
			start = jiffies;
			//maximum allow send down event 7 times
			if(fod_down < 6) {
				ret = cd->imports->report_gesture(&mmi_event);
				if (!ret)
					PM_WAKEUP_EVENT(cd->gesture_wakelock, 3000);
			}
			fod_down++;
		}else if(cd->gesture_type & GESTURE_FOD_PRESS && gs_event.gesture_type == GOODIX_GESTURE_FOD_UP) {
			ts_info("Get FOD-UP gesture");
			mmi_event.evcode = 3;
			mmi_event.evdata.x= 0;
			mmi_event.evdata.y= 0;
			ret = cd->imports->report_gesture(&mmi_event);
			if (!ret)
				PM_WAKEUP_EVENT(cd->gesture_wakelock, 500);
			fod_down = 0;
		} else {
			ts_debug("not support gesture type[%02X] to wakeup, suspended =%d", gs_event.gesture_type, atomic_read(&cd->suspended));
			fod_down = 0;
		}
	}
#else
	switch (gs_event.gesture_type) {
	case GOODIX_GESTURE_SINGLE_TAP:
		if (cd->gesture_type & GESTURE_SINGLE_TAP) {
			ts_info("get SINGLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_POWER, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_POWER, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable SINGLE-TAP");
		}
		break;
	case GOODIX_GESTURE_DOUBLE_TAP:
		if (cd->gesture_type & GESTURE_DOUBLE_TAP) {
			ts_info("get DOUBLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable DOUBLE-TAP");
		}
		break;
	case GOODIX_GESTURE_FOD_DOWN:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info("get FOD-DOWN gesture");
			fodx = le16_to_cpup((__le16 *)gs_event.gesture_data);
			fody = le16_to_cpup((__le16 *)(gs_event.gesture_data + 2));
			overlay_area = gs_event.gesture_data[4];
			ts_info("fodx:%d fody:%d overlay_area:%d", fodx, fody, overlay_area);
			input_report_key(cd->input_dev, BTN_TOUCH, 1);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_X, fodx);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_Y, fody);
			input_report_abs(cd->input_dev, ABS_MT_WIDTH_MAJOR, overlay_area);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable FOD-DOWN");
		}
		break;
	case GOODIX_GESTURE_FOD_UP:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info("get FOD-UP gesture");
			fodx = le16_to_cpup((__le16 *)gs_event.gesture_data);
			fody = le16_to_cpup((__le16 *)(gs_event.gesture_data + 2));
			overlay_area = gs_event.gesture_data[4];
			input_report_key(cd->input_dev, BTN_TOUCH, 0);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev,
					MT_TOOL_FINGER, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable FOD-UP");
		}
		break;
	default:
		ts_err("not support gesture type[%02X]", gs_event.gesture_type);
		break;
	}
#endif

re_send_ges_cmd:
#if defined(PRODUCT_MIAMI)
	if (hw_ops->gesture(cd, 0x80))
#elif defined(CONFIG_BOARD_USES_DOUBLE_TAP_CTRL)
	if (goodix_ts_send_cmd(cd, ENTER_GESTURE_MODE_CMD, 6, (cd->gesture_cmd) >> 8,
			cd->gesture_cmd & 0xFF) < 0)
#else
	if (hw_ops->gesture(cd, 0))
#endif
		ts_info("warning: failed re_send gesture cmd");
gesture_ist_exit:
	if (!cd->tools_ctrl_sync)
		hw_ops->after_event_handler(cd);
	cd->zerotap_data[0] = fod_down;
	return EVT_CANCEL_IRQEVT;
}

/**
 * gsx_gesture_before_suspend - execute gesture suspend routine
 * This functions is excuted to set ic into doze mode
 *
 * @cd: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_IRQCANCLED  stop execute
 */
static int gsx_gesture_before_suspend(struct goodix_ts_core *cd,
	struct goodix_ext_module *module)
{
	int ret;
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	if (cd->gesture_type == 0)
		return EVT_CONTINUE;

	ret = hw_ops->gesture(cd, 0);
	if (ret)
		ts_err("failed enter gesture mode");
	else
		ts_info("enter gesture mode, type[0x%02X]", cd->gesture_type);

	hw_ops->irq_enable(cd, true);
	enable_irq_wake(cd->irq);

	return EVT_CANCEL_SUSPEND;
}

static int gsx_gesture_before_resume(struct goodix_ts_core *cd,
	struct goodix_ext_module *module)
{
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	if (cd->gesture_type == 0)
		return EVT_CONTINUE;
	hw_ops->irq_enable(cd, false);
	disable_irq_wake(cd->irq);
	hw_ops->reset(cd, GOODIX_NORMAL_RESET_DELAY_MS);

	return EVT_CANCEL_RESUME;
}

static struct goodix_ext_module_funcs gsx_gesture_funcs = {
	.irq_event = gsx_gesture_ist,
	.init = gsx_gesture_init,
	.exit = gsx_gesture_exit,
	.before_suspend = gsx_gesture_before_suspend,
	.before_resume = gsx_gesture_before_resume,
};

int gesture_module_init(void)
{
	int ret;
	int i;
	struct kobject *def_kobj = goodix_get_default_kobj();
	struct kobj_type *def_kobj_type = goodix_get_default_ktype();

	gsx_gesture = kzalloc(sizeof(struct gesture_module), GFP_KERNEL);
	if (!gsx_gesture)
		return -ENOMEM;

	gsx_gesture->module.funcs = &gsx_gesture_funcs;
	gsx_gesture->module.priority = EXTMOD_PRIO_GESTURE;
	gsx_gesture->module.name = "Goodix_gsx_gesture";
	gsx_gesture->module.priv_data = gsx_gesture;

	atomic_set(&gsx_gesture->registered, 0);
	rwlock_init(&gsx_gesture->rwlock);

	/* gesture sysfs init */
	ret = kobject_init_and_add(&gsx_gesture->module.kobj,
			def_kobj_type, def_kobj, "gesture");
	if (ret) {
		ts_err("failed create gesture sysfs node!");
		goto err_out;
	}

	for (i = 0; i < ARRAY_SIZE(gesture_attrs) && !ret; i++)
		ret = sysfs_create_file(&gsx_gesture->module.kobj,
				&gesture_attrs[i].attr);
	if (ret) {
		ts_err("failed create gst sysfs files");
		while (--i >= 0)
			sysfs_remove_file(&gsx_gesture->module.kobj,
					&gesture_attrs[i].attr);

		kobject_put(&gsx_gesture->module.kobj);
		goto err_out;
	}

	module_initialized = true;

	goodix_gesture_enable(1);

	ts_info("gesture module init success");

	return 0;

err_out:
	ts_err("gesture module init failed!");
	kfree(gsx_gesture);
	return ret;
}

void gesture_module_exit(void)
{
	int i;

	ts_info("gesture module exit");
	if (!module_initialized)
		return;

	goodix_gesture_enable(0);

	/* deinit sysfs */
	for (i = 0; i < ARRAY_SIZE(gesture_attrs); i++)
		sysfs_remove_file(&gsx_gesture->module.kobj,
					&gesture_attrs[i].attr);

	kobject_put(&gsx_gesture->module.kobj);
	kfree(gsx_gesture);
	module_initialized = false;
}
