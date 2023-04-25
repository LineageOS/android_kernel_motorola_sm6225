// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved. */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
/*
 * Define vibration periods: default(5sec), min(50ms), max(15sec)).
 */
#define MOTO_VIB_MIN_PLAY_MS		50
#define MOTO_VIB_PLAY_MS		5000
#define MOTO_VIB_MAX_PLAY_MS		15000

#ifdef CONFIG_VIBRATOR_NOISE_CAMERA
extern int mot_actuator_on_vibrate_start(void);
extern int mot_actuator_on_vibrate_stop(void);
#endif

struct vib_ldo_chip {
	struct led_classdev	cdev;
	struct mutex		lock;
	struct hrtimer		stop_timer;
	struct work_struct	vib_work;

	int			state;
	int			en_gpio;
	u64			vib_play_ms;
	bool			vib_enabled;
};


static inline int moto_vib_ldo_enable(struct vib_ldo_chip *chip, bool enable)
{
	int ret;

#ifdef CONFIG_VIBRATOR_NOISE_CAMERA
	static int mot_actuator_started = 0;
#endif

	if (chip->vib_enabled == enable)
		return 0;

	pr_debug("moto_vib_ldo_enable enable = %d\n", enable);

	if (enable) {
		ret = gpio_direction_output(chip->en_gpio, 1);
		if (ret < 0)
			pr_err(" %s : ldo en gpio enable failed \n", __func__);
		chip->vib_enabled = true;
	}
	else  {
		ret = gpio_direction_output(chip->en_gpio, 0);
		if (ret < 0)
			pr_err(" %s : ldo en gpio disable failed \n", __func__);
		chip->vib_enabled = false;
	}

#ifdef CONFIG_VIBRATOR_NOISE_CAMERA
	if ((chip->vib_play_ms > 70) && (enable)) {
		mot_actuator_on_vibrate_start();
		mot_actuator_started = 1;
	}
	if(!(enable) && (mot_actuator_started == 1 )) {
		mot_actuator_on_vibrate_stop();
		mot_actuator_started = 0;
	}
	pr_debug("moto_vib_ldo_enable camera done\n");
#endif

	return ret;
}

static void moto_vib_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip = container_of(work, struct vib_ldo_chip,
						vib_work);
	int ret = 0;

	if (chip->state) {
		if (!chip->vib_enabled)
			ret = moto_vib_ldo_enable(chip, true);

		if (ret == 0)
			hrtimer_start(&chip->stop_timer,
				      ms_to_ktime(chip->vib_play_ms),
				      HRTIMER_MODE_REL);
	} else {
		moto_vib_ldo_enable(chip, false);
	}
}

static enum hrtimer_restart vib_stop_timer(struct hrtimer *timer)
{
	struct vib_ldo_chip *chip = container_of(timer, struct vib_ldo_chip,
					     stop_timer);

	chip->state = 0;
	schedule_work(&chip->vib_work);
	return HRTIMER_NORESTART;
}

static ssize_t moto_vib_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->vib_enabled);
}

static ssize_t moto_vib_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/* At present, nothing to do with setting state */
	return count;
}

static ssize_t moto_vib_show_duration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&chip->stop_timer)) {
		time_rem = hrtimer_get_remaining(&chip->stop_timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return scnprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t moto_vib_store_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	if (val < MOTO_VIB_MIN_PLAY_MS)
		val = MOTO_VIB_MIN_PLAY_MS;

	if (val > MOTO_VIB_MAX_PLAY_MS)
		val = MOTO_VIB_MAX_PLAY_MS;

	mutex_lock(&chip->lock);
	chip->vib_play_ms = val;
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t moto_vib_show_activate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t moto_vib_store_activate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val != 0 && val != 1)
		return count;

	mutex_lock(&chip->lock);
	hrtimer_cancel(&chip->stop_timer);
	chip->state = val;
	pr_debug("state = %d, time = %llums\n", chip->state, chip->vib_play_ms);
	mutex_unlock(&chip->lock);
	schedule_work(&chip->vib_work);

	return count;
}

static struct device_attribute moto_vib_attrs[] = {
	__ATTR(state, 0664, moto_vib_show_state, moto_vib_store_state),
	__ATTR(duration, 0664, moto_vib_show_duration, moto_vib_store_duration),
	__ATTR(activate, 0664, moto_vib_show_activate, moto_vib_store_activate),
};

static int moto_vib_hw_init(struct device *dev, struct vib_ldo_chip *chip)
{
	int ret = 0;

	chip->en_gpio = of_get_named_gpio(dev->of_node, "moto,vib-ldo-gpio", 0);

	if (!gpio_is_valid(chip->en_gpio)) {
		ret = -ENODEV;
		pr_err("moto,vib-ldo-gpio is invalid, ret=%d\n",ret);
		return ret;
	}
	else  {
		ret = gpio_request(chip->en_gpio, "vib-ldo-gpio");
		if (ret < 0) {
			pr_err(" %s : gpio_requset vib-ldo-gpio pin failed \n", __func__);
				ret = -ENODEV;
			return ret;
		}
		ret = gpio_direction_output(chip->en_gpio, 0);
		if (ret < 0) {
			pr_err(" %s : gpio set defalut status failed \n", __func__);
				ret = -ENODEV;
			goto err_en_gpio;
		}
	}
	return ret;

err_en_gpio:
	if (gpio_is_valid(chip->en_gpio) )
		gpio_free(chip->en_gpio);

	return ret;
}

/* Dummy functions for brightness */
static enum led_brightness moto_vib_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static void moto_vib_brightness_set(struct led_classdev *cdev,
			enum led_brightness level)
{
}

static int moto_vibrator_ldo_suspend(struct device *dev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);

	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	moto_vib_ldo_enable(chip, false);
	mutex_unlock(&chip->lock);

	return 0;
}
static SIMPLE_DEV_PM_OPS(moto_vibrator_ldo_pm_ops, moto_vibrator_ldo_suspend,
			NULL);

static int moto_vibrator_ldo_probe(struct platform_device *pdev)
{
	//struct device_node *of_node = pdev->dev.of_node;
	struct vib_ldo_chip *chip;
	int i, ret = 0;;
	pr_info("Vibrator LDO successfully registered: ret = %d\n",ret);

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;


	ret = moto_vib_hw_init(&pdev->dev, chip);
	if (ret < 0) {
		pr_err("couldn't init hw resource, ret=%d\n", ret);
		return ret;
	}

	chip->vib_play_ms = MOTO_VIB_PLAY_MS;
	mutex_init(&chip->lock);
	INIT_WORK(&chip->vib_work, moto_vib_work);

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = vib_stop_timer;
	dev_set_drvdata(&pdev->dev, chip);

	chip->cdev.name = "vibrator";
	chip->cdev.brightness_get = moto_vib_brightness_get;
	chip->cdev.brightness_set = moto_vib_brightness_set;
	chip->cdev.max_brightness = 100;
	ret = devm_led_classdev_register(&pdev->dev, &chip->cdev);
	if (ret < 0) {
		pr_err("Error in registering led class device, ret=%d\n", ret);
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(moto_vib_attrs); i++) {
		ret = sysfs_create_file(&chip->cdev.dev->kobj,
				&moto_vib_attrs[i].attr);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error in creating sysfs file, ret=%d\n",
				ret);
			goto sysfs_fail;
		}
	}

	pr_info("Vibrator LDO successfully registered: gpio = %d\n",chip->en_gpio);
	return 0;

sysfs_fail:
	for (--i; i >= 0; i--)
		sysfs_remove_file(&chip->cdev.dev->kobj,
				&moto_vib_attrs[i].attr);
fail:
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	if (gpio_is_valid(chip->en_gpio) )
		gpio_free(chip->en_gpio);
	return ret;
}

static int moto_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);

	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	if (gpio_is_valid(chip->en_gpio) )
		gpio_free(chip->en_gpio);
	return 0;
}

static const struct of_device_id vibrator_ldo_match_table[] = {
	{ .compatible = "moto,vibrator-ldo" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vibrator_ldo_match_table);

static struct platform_driver moto_vibrator_ldo_driver = {
	.driver	= {
		.name		= "moto-vibrator-ldo",
		.of_match_table	= vibrator_ldo_match_table,
		.pm		= &moto_vibrator_ldo_pm_ops,
	},
	.probe	= moto_vibrator_ldo_probe,
	.remove	= moto_vibrator_ldo_remove,
};
module_platform_driver(moto_vibrator_ldo_driver);

MODULE_DESCRIPTION("MOTO LDO-Vibrator driver");
MODULE_LICENSE("GPL v2");
