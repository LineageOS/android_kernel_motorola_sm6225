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

#include <linux/touchscreen_mmi.h>
#include "goodix_ts_mmi.h"
#include "goodix_cfg_bin.h"
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif


extern int goodix_ts_unregister_notifier(struct notifier_block *nb);
extern struct goodix_module goodix_modules;

static int goodix_ts_mmi_methods_get_vendor(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "goodix");
}
static int goodix_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata) {
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_INFO_LEN, "%s", GOODIX_TP_IC_TYPE);
}
static int goodix_ts_mmi_methods_get_build_id(struct device *dev, void *cdata) {
	int ret;
	struct goodix_ts_version fw_ver;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;
	if (!ts_dev) {
		ts_err("Failed to get platform data");
		return -ENODEV;
	}
	ret = ts_dev->hw_ops->read_version(ts_dev, &fw_ver);
	if (!ret && fw_ver.valid) {
		ret = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fw_ver.vid[0]);
		ts_info("get config id:%d ", fw_ver.vid[0]);
		return ret;
	} else
		return -ENODEV;
}
/*return firmware version*/
static int goodix_ts_mmi_methods_get_config_id(struct device *dev, void *cdata) {
	int ret;
	struct goodix_ts_version fw_ver;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;
	if (!ts_dev) {
		ts_err("Failed to get platform data");
		return -ENODEV;
	}
	ret = ts_dev->hw_ops->read_version(ts_dev, &fw_ver);
	if (!ret && fw_ver.valid) {
		ret = scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%02x", fw_ver.vid[3]);
		ts_info("get build id:%d ", fw_ver.vid[3]);
		return ret;
	} else
		return -ENODEV;
}
static int goodix_ts_mmi_methods_get_bus_type(struct device *dev, void *idata) {
	TO_INT(idata) = TOUCHSCREEN_MMI_BUS_TYPE_I2C;
	return 0;
}
static int goodix_ts_mmi_methods_get_irq_status(struct device *dev, void *idata) {
	struct goodix_ts_board_data *ts_bdata;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(ts_bdata->irq_gpio);
	return 0;
}
static int goodix_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	TO_INT(idata) = atomic_read(&core_data->irq_enabled);
	return 0;
}
static int goodix_ts_mmi_methods_get_poweron(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	TO_INT(idata) = core_data->power_on;
	return 0;
}
static int goodix_ts_mmi_methods_get_flashprog(struct device *dev, void *idata) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	if (core_data->ts_mmi_info.need_reflash)
		TO_INT(idata) = 1;
	else
		TO_INT(idata) = core_data->ts_mmi_info.update_status;
	return 0;
}
static int goodix_ts_mmi_methods_drv_irq(struct device *dev, int state) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	goodix_ts_irq_enable(core_data, !(!state));
	return 0;
}
/* reset chip
 * type：can control software reset and hardware reset,
 * but GOODIX has no software reset,
 * so the type parameter is not used here.
 */
static int goodix_ts_mmi_methods_reset(struct device *dev, int type) {
	int ret = -ENODEV;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;
	if (ts_dev->hw_ops->reset)
		ret = ts_dev->hw_ops->reset(ts_dev);
	return ret;
}
static int goodix_ts_firmware_update(struct device *dev, char *fwname) {
	struct goodix_ts_board_data *ts_bdata;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_bdata = board_data(core_data);
	if (!ts_bdata) {
		ts_err("Failed to get ts board data");
		return -ENODEV;
	}

	ts_debug("HW request update fw");
	/* set firmware image name */
	if (core_data->set_fw_name)
		core_data->set_fw_name(fwname);
	else
		ts_err("fw update handler not ready");

	if (core_data->do_fw_update)
		core_data->do_fw_update(UPDATE_MODE_SRC_REQUEST |
					UPDATE_MODE_BLOCK |
					UPDATE_MODE_FORCE);
	else {
		ts_err("fw update handler not ready.");
		return -EFAULT;
	}
	core_data->ts_mmi_info.need_reflash = 0;
	return 0;
}

static int goodix_ts_mmi_methods_power(struct device *dev, int on) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	if (on == TS_MMI_POWER_ON)
		return goodix_ts_power_on(core_data);
	else if(on == TS_MMI_POWER_OFF)
		return goodix_ts_power_off(core_data);
	else {
		ts_err("Invalid power parameter %d.\n", on);
		return -EINVAL;
	}
}

#ifdef CONFIG_PINCTRL
static int goodix_ts_mmi_methods_pinctrl(struct device *dev, int on) {
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	if (on == TS_MMI_PINCTL_ON) {
		if (core_data->pinctrl)
			return pinctrl_select_state(core_data->pinctrl,
						 core_data->pin_sta_active);
	} else if(on == TS_MMI_PINCTL_OFF) {
		if (core_data->pinctrl)
			return pinctrl_select_state(core_data->pinctrl,
					core_data->pin_sta_suspend);
	} else {
		ts_err("Invalid power parameter %d.\n", on);
		return -EINVAL;
	}

	return 0;
}
#endif

static int goodix_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	ts_dev = core_data->ts_dev;

	switch (to) {
	case TS_MMI_PM_GESTURE:
	case TS_MMI_PM_DEEPSLEEP:
		/* let touch ic work in sleep mode */
		if (ts_dev && ts_dev->hw_ops->suspend)
			ts_dev->hw_ops->suspend(ts_dev);
		atomic_set(&core_data->suspended, 1);
		break;
	case TS_MMI_PM_ACTIVE:
		atomic_set(&core_data->suspended, 0);
		/* resume device */
		if (ts_dev && ts_dev->hw_ops->resume)
			ts_dev->hw_ops->resume(ts_dev);
		break;
	default:
		ts_err("Invalid power state parameter %d.\n", to);
		return -EINVAL;
	}

	return 0;
}

static int goodix_ts_mmi_pre_resume(struct device *dev) {
	struct goodix_ext_module *ext_module, *next;
	struct input_dev *input_dev;
	struct input_mt *mt;
	int i;
	int r = 0;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	input_dev = core_data->input_dev;
	mt = input_dev->mt;

	if (mt) {
		for (i = 0; i < mt->num_slots; i++) {
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER,
					false);
		}
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_mt_sync_frame(input_dev);
		input_sync(input_dev);
	}

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->before_resume)
				continue;

			r = ext_module->funcs->before_resume(core_data,
							     ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

out:
	return r;
}

static int goodix_ts_mmi_post_resume(struct device *dev) {
	struct goodix_ext_module *ext_module, *next;
	int r = 0;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(core_data,
							    ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);
	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	ts_info("try notify resume");
	goodix_ts_blocking_notify(NOTIFY_RESUME, NULL);

out:
	ts_debug("Resume end");
	return r;
}

static int goodix_ts_mmi_pre_suspend(struct device *dev) {
	struct goodix_ext_module *ext_module, *next;
	int r = 0;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_blocking_notify(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->before_suspend)
				continue;

			r = ext_module->funcs->before_suspend(core_data,
							      ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

out:
	return r;
}

static int goodix_ts_mmi_post_suspend(struct device *dev) {
	struct goodix_ext_module *ext_module, *next;
	int r = 0;
	struct goodix_ts_core *core_data;
	struct platform_device *pdev = dev_get_drvdata(dev);

	if (!pdev) {
		ts_err("Failed to get platform device");
		return -ENODEV;
	}
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}

	/* inform exteranl modules */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->after_suspend)
				continue;

			r = ext_module->funcs->after_suspend(core_data,
							     ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

out:
	ts_info("Suspend end");
	return r;
}

static struct ts_mmi_methods goodix_ts_mmi_methods = {
	.get_vendor = goodix_ts_mmi_methods_get_vendor,
	.get_productinfo = goodix_ts_mmi_methods_get_productinfo,
	.get_build_id = goodix_ts_mmi_methods_get_build_id,
	.get_config_id = goodix_ts_mmi_methods_get_config_id,
	.get_bus_type = goodix_ts_mmi_methods_get_bus_type,
	.get_irq_status = goodix_ts_mmi_methods_get_irq_status,
	.get_drv_irq = goodix_ts_mmi_methods_get_drv_irq,
	.get_poweron = goodix_ts_mmi_methods_get_poweron,
	.get_flashprog = goodix_ts_mmi_methods_get_flashprog,
	/* SET methods */
	.reset =  goodix_ts_mmi_methods_reset,
	.drv_irq = goodix_ts_mmi_methods_drv_irq,
	.power = goodix_ts_mmi_methods_power,
#ifdef CONFIG_PINCTRL
	.pinctrl = goodix_ts_mmi_methods_pinctrl,
#endif
	/* Firmware */
	.firmware_update = goodix_ts_firmware_update,
	/* PM callback */
	.panel_state = goodix_ts_mmi_panel_state,
	.pre_resume = goodix_ts_mmi_pre_resume,
	.post_resume = goodix_ts_mmi_post_resume,
	.pre_suspend = goodix_ts_mmi_pre_suspend,
	.post_suspend = goodix_ts_mmi_post_suspend,
};

static int mmi_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct goodix_ts_mmi *mmi_info = container_of(nb,
			struct goodix_ts_mmi, mmi_notifier);
	struct goodix_ts_core *core_data = container_of(mmi_info,
			struct goodix_ts_core, ts_mmi_info);
	switch (action) {
	case NOTIFY_FWUPDATE_START:
		ts_info("Firmware update start");
		core_data->ts_mmi_info.update_status = 1;
		break;
	case NOTIFY_FWUPDATE_FAILED:
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_FWUPDATE_SKIP:
		core_data->ts_mmi_info.update_status = 0;
		break;
	default:
		ts_err("Firmware state unknown");
		break;
	}
	return 0;
}

int goodix_ts_mmi_dev_register(struct platform_device *pdev) {
	int ret;
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data) {
		ts_err("Failed to get driver data");
		return -ENODEV;
	}
	core_data->ts_mmi_info.mmi_notifier.notifier_call = mmi_notifier_callback;
	goodix_ts_register_notifier(&core_data->ts_mmi_info.mmi_notifier);
	ret = ts_mmi_dev_register(core_data->ts_dev->dev, &goodix_ts_mmi_methods);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ts mmi\n");
		return ret;
	}
	return 0;
}

void goodix_ts_mmi_dev_unregister(struct platform_device *pdev) {
	struct goodix_ts_core *core_data;
	core_data = platform_get_drvdata(pdev);
	if (!core_data)
		ts_err("Failed to get driver data");
	ts_mmi_dev_unregister(&pdev->dev);
	goodix_ts_unregister_notifier(&core_data->ts_mmi_info.mmi_notifier);
}
