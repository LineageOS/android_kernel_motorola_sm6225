/*
 *Copyright (C) 2021 Motorola Mobility LLC,
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/rc-core.h>
#include <linux/platform_device.h>
#include "pm6125_flash_gpio.h"

#ifdef CONFIG_CAMERA_FLASH_PWM
#include "cam_flash_dev.h"
#include "cam_flash_soc.h"
#include "cam_flash_core.h"
#include "cam_common_util.h"
#include "cam_res_mgr_api.h"
#endif


/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct pwm_device *pwm;
#ifdef CONFIG_CAMERA_FLASH_PWM
static unsigned int flashlight_enable = CAMERA_SENSOR_FLASH_STATUS_OFF;
static unsigned int flashlight_brightness = 0;
#endif

/*****************************************************************************
 * Function
 *****************************************************************************/

void pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE s, enum camera_flash_opcode opcode, u64 flash_current){
	struct pwm_state pstate;
	int rc = 0;
	u64 real_current = 0;

	pwm_get_state(pwm, &pstate);
	PM6125_FLASH_PRINT("[pm6125_flash_gpio]Status duty_cycle = %u, period = %u\n",
					pstate.duty_cycle, pstate.period);
	pstate.period = PM6125_PWM_PERIOD;
	switch (s) {
	case PM6125_FLASH_GPIO_STATE_ACTIVE:
		switch (opcode)
		{
		case CAMERA_SENSOR_FLASH_OP_FIRELOW:
			real_current = (flash_current > FLASH_FIRE_LOW_MAXCURRENT?FLASH_FIRE_LOW_MAXCURRENT:flash_current);
			pstate.duty_cycle = PM6125_PWM_PERIOD * real_current / FLASH_FIRE_LOW_MAXCURRENT;
			break;
		case CAMERA_SENSOR_FLASH_OP_FIREHIGH:
			real_current = (flash_current > FLASH_FIRE_HIGH_MAXCURRENT?FLASH_FIRE_HIGH_MAXCURRENT:flash_current);
			pstate.duty_cycle = PM6125_PWM_PERIOD * real_current / FLASH_FIRE_HIGH_MAXCURRENT;
			break;
		default:
			pstate.duty_cycle = PM6125_PWM_PERIOD;
			break;
		}

		pstate.enabled = true;
		rc = pwm_apply_state(pwm, &pstate);

		PM6125_FLASH_PRINT("[pm6125_flash_gpio]Active duty_cycle = %u, period = %u, opcode = %u, real_current = %u\n",
						pstate.duty_cycle, pstate.period, opcode, real_current);
		break;
	case PM6125_FLASH_GPIO_STATE_SUSPEND:
		pstate.enabled = false;
		rc = pwm_apply_state(pwm, &pstate);
		PM6125_FLASH_PRINT("[pm6125_flash_gpio]Suspend");
		break;
	default:
		PM6125_FLASH_PRINT("[pm6125_flash_gpio]Failed to control PWM use a err state!\n");
	}

	if(rc < 0) {
		PM6125_FLASH_PRINT("[pm6125_flash_gpio]Apply PWM state fail, rc = %d", rc);
	}
}

#ifdef CONFIG_CAMERA_FLASH_PWM
static ssize_t flashlight_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", flashlight_enable);
}

static ssize_t flashlight_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cam_flash_ctrl *fctrl = dev_get_drvdata(dev);
	struct cam_flash_private_soc *soc_private = fctrl->soc_info.soc_private;
	unsigned long state_to_set;
	ssize_t ret = -EINVAL;

	CAM_INFO(CAM_FLASH, "Flash Enable Command: %s", buf);
	ret = kstrtoul(buf, 10, &state_to_set);
	if (ret)
		return ret;

	if (CAMERA_SENSOR_FLASH_STATUS_OFF != state_to_set) {
		// enable flash
		if (CAMERA_SENSOR_FLASH_STATUS_OFF == flashlight_enable) {
			ret = cam_res_mgr_gpio_request(
				fctrl->soc_info.dev,
				soc_private->flash_gpio_enable, 0,
				"CUSTOM_GPIO1");
			if (ret) {
				CAM_ERR(CAM_FLASH, "gpio %d request fails",
					soc_private->flash_gpio_enable);
				return ret;
			}
		}

		flashlight_enable = state_to_set;
		cam_res_mgr_gpio_set_value(
			soc_private->flash_gpio_enable,
			flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_HIGH);
		if (flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_LOW) {
			pm6125_flash_gpio_select_state(
				PM6125_FLASH_GPIO_STATE_ACTIVE,
				CAMERA_SENSOR_FLASH_OP_FIRELOW,
				FLASH_FIRE_LOW_MAXCURRENT);
			usleep_range(5000, 6000);
		}
		pm6125_flash_gpio_select_state(
			PM6125_FLASH_GPIO_STATE_ACTIVE,
			flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_HIGH ?
					  CAMERA_SENSOR_FLASH_OP_FIREHIGH :
					  CAMERA_SENSOR_FLASH_OP_FIRELOW,
			(u64)flashlight_brightness);
	}
	else {
		// disable flash
		if (CAMERA_SENSOR_FLASH_STATUS_OFF != flashlight_enable) {
			cam_res_mgr_gpio_set_value(
				soc_private->flash_gpio_enable, 0);
			cam_res_mgr_gpio_free(fctrl->soc_info.dev,
						  soc_private->flash_gpio_enable);
		}
		flashlight_enable = CAMERA_SENSOR_FLASH_STATUS_OFF;
		pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_SUSPEND,
						   CAMERA_SENSOR_FLASH_OP_OFF, 0);
	}
	return size;
}

static ssize_t flashlight_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", flashlight_brightness);
}

static ssize_t flashlight_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cam_flash_ctrl *fctrl = dev_get_drvdata(dev);
	struct cam_flash_private_soc *soc_private = fctrl->soc_info.soc_private;
	unsigned long brightness_to_set;
	ssize_t ret = -EINVAL;

	CAM_INFO(CAM_FLASH, "Flash Brightness Command: %s", buf);
	ret = kstrtoul(buf, 10, &brightness_to_set);
	if (ret)
		return ret;

	flashlight_brightness = brightness_to_set;

	switch (flashlight_enable)
	{
	case CAMERA_SENSOR_FLASH_STATUS_LOW: // CAMERA_SENSOR_FLASH_OP_FIRELOW
	case CAMERA_SENSOR_FLASH_STATUS_HIGH: // CAMERA_SENSOR_FLASH_OP_FIREHIGH

		CAM_INFO(CAM_FLASH, "Flash Mode %s: %d",
			 flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_HIGH ?
					   "High" :
					   "Low",
			 flashlight_brightness);

		// disable first
		cam_res_mgr_gpio_set_value(
			soc_private->flash_gpio_enable, 0);
		cam_res_mgr_gpio_free(fctrl->soc_info.dev,
						soc_private->flash_gpio_enable);
		pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_SUSPEND, CAMERA_SENSOR_FLASH_OP_OFF, 0);

		// get handle
		ret = cam_res_mgr_gpio_request(
			fctrl->soc_info.dev,
			soc_private->flash_gpio_enable, 0,
			"CUSTOM_GPIO1");
		if (ret) {
			CAM_ERR(CAM_FLASH, "gpio %d request fails",
				soc_private->flash_gpio_enable);
			return ret;
		}

		cam_res_mgr_gpio_set_value(
			soc_private->flash_gpio_enable,
			flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_HIGH);
		if (flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_LOW) {
			pm6125_flash_gpio_select_state(
				PM6125_FLASH_GPIO_STATE_ACTIVE,
				CAMERA_SENSOR_FLASH_OP_FIRELOW,
				FLASH_FIRE_LOW_MAXCURRENT);
			usleep_range(5000, 6000);
		}
		pm6125_flash_gpio_select_state(
			PM6125_FLASH_GPIO_STATE_ACTIVE,
			flashlight_enable == CAMERA_SENSOR_FLASH_STATUS_HIGH ?
					  CAMERA_SENSOR_FLASH_OP_FIREHIGH :
					  CAMERA_SENSOR_FLASH_OP_FIRELOW,
			(u64)flashlight_brightness);
		break;
	default:
		CAM_INFO(CAM_FLASH, "flashlight had not been enable yet.");
		break;
	}

	return size;
}
static DEVICE_ATTR_RW(flashlight_enable);
static DEVICE_ATTR_RW(flashlight_brightness);
struct device_attribute *flashlight_attributes[] = {
	&dev_attr_flashlight_enable,
	&dev_attr_flashlight_brightness,
};

int pm6125_flash_control_create_device(struct device* dev)
{
	int i, ret = 0;
	for (i = 0; i < ARRAY_SIZE(flashlight_attributes); i++) {
		ret = device_create_file(dev,
						flashlight_attributes[i]);
		if (ret) {
			dev_err(dev, "failed: sysfs file %s\n",
					flashlight_attributes[i]->attr.name);
			pm6125_flash_control_remove_device(dev);
			return ret;
		}
	}
	return 0;
}

int pm6125_flash_control_remove_device(struct device* dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(flashlight_attributes); i++) {
		device_remove_file(dev, flashlight_attributes[i]);
	}
	return 0;
}
#endif

static int pm6125_flash_pwm_probe(struct platform_device *pdev)
{
	int rc = 0;
	pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pwm))
		return PTR_ERR(pwm);
	return rc;
}

static const struct of_device_id gpio_of_match[] = {
	{ .compatible = "qualcomm,pm6125_flash_gpio", },
	{},
};

static struct platform_driver pm6125_flash_gpio_platform_driver = {
	.probe = pm6125_flash_pwm_probe,
	.driver = {
		.name = "PM6125_FLASH_GPIO_DTS",
		.of_match_table = gpio_of_match,
	},
};

static int __init pm6125_flash_gpio_init_module(void)
{
	if (platform_driver_register(&pm6125_flash_gpio_platform_driver)) {
		PM6125_FLASH_PRINT("[pm6125_flash_gpio]Failed to register pm6125_flash_gpio_platform_driver!\n");
		return -1;
	}
	return 0;
}

static void __exit pm6125_flash_gpio_exit_module(void)
{
	platform_driver_unregister(&pm6125_flash_gpio_platform_driver);
}

module_init(pm6125_flash_gpio_init_module);
module_exit(pm6125_flash_gpio_exit_module);
MODULE_AUTHOR("liyang <liyang14@huaqin.com>");
MODULE_DESCRIPTION("CONTROL PM6125 FLASH GPIO Driver");
MODULE_LICENSE("GPL");
