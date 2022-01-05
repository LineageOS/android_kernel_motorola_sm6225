// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved. */


#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

/*
 * Define vibration periods: default(5sec), min(50ms), max(15sec) and
 * overdrive(30ms).
 */
#define EXT_VIB_MIN_PLAY_MS		50
#define EXT_VIB_PLAY_MS			5000
#define EXT_VIB_MAX_PLAY_MS		15000

struct vib_ldo_chip {
	struct led_classdev	cdev;
	struct mutex		lock;
	struct hrtimer		stop_timer;
	struct work_struct	vib_work;

	int			state;
	u64			vib_play_ms;

	int			vcc_2v6_Pin;
	int			vcc_3v0_Pin;

	int 			pwr_by_gpio;
	struct regulator	 *vcc;
	u32 			regulator_voltage_max;
	u32 			regulator_voltage_min;

	/*for long/short vibrator*/
	bool                    dis_short_long;
	int                     dis_long_ms;

	u32			long_voltage_supply;
	u32			short_voltage_supply;
	u32			long_play_threshold;
	bool 			pwr_by_regulator;
};

static void ext_vib_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip = container_of(work, struct vib_ldo_chip,
						vib_work);
	int ret = 0;

	static bool regulator_en_flag = false;
	if (chip->state) {
		if (chip->pwr_by_gpio) {
			if (chip->dis_short_long) {
				pr_warn("vib in dis short and long, play ms=%lld, dis_longms=%d\n",chip->vib_play_ms, chip->dis_long_ms);
				if (chip->vib_play_ms > chip->dis_long_ms) {
					ret = gpio_direction_output(chip->vcc_2v6_Pin, chip->state);
				}
				else {
					ret = gpio_direction_output(chip->vcc_3v0_Pin, chip->state);
					ret = gpio_direction_output(chip->vcc_2v6_Pin, chip->state);
				}
			}
			else {
				ret = gpio_direction_output(chip->vcc_3v0_Pin, chip->state);
				ret = gpio_direction_output(chip->vcc_2v6_Pin, chip->state);
			}
		}
		else if (!regulator_en_flag) {
			ret = regulator_enable(chip->vcc);
			regulator_en_flag = true;
		}
		if (ret == 0)
			hrtimer_start(&chip->stop_timer,
				      ms_to_ktime(chip->vib_play_ms),
				      HRTIMER_MODE_REL);
	}
	else {
		if (chip->pwr_by_gpio) {
			gpio_direction_output(chip->vcc_2v6_Pin, chip->state);
			gpio_direction_output(chip->vcc_3v0_Pin, chip->state);
		}
		else if (regulator_en_flag) {
			regulator_en_flag = false;
			ret = regulator_disable(chip->vcc);
		}
	}
	if (chip->pwr_by_gpio)
	{
		pr_info("vib:chip->state = %d,vib_play_ms=%lld\n",chip->state,
				chip->vib_play_ms);
	}
	else
	{
		pr_info("vib:chip->state = %d,vib_play_ms=%lld vib_vol=%d\n",chip->state,
				chip->vib_play_ms, regulator_get_voltage(chip->vcc));
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

static ssize_t ext_vib_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->state);
}

static ssize_t ext_vib_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/* At present, nothing to do with setting state */
	return count;
}

static ssize_t ext_vib_show_duration(struct device *dev,
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

static ssize_t ext_vib_store_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;
	static bool long_vib_flag = true;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	if (val < EXT_VIB_MIN_PLAY_MS)
		val = EXT_VIB_MIN_PLAY_MS;

	if (val > EXT_VIB_MAX_PLAY_MS)
		val = EXT_VIB_MAX_PLAY_MS;

	if (chip->pwr_by_regulator) {
	    if((val >= chip->long_play_threshold) && long_vib_flag){
		 chip->regulator_voltage_max = chip->long_voltage_supply;
		 chip->regulator_voltage_min = chip->long_voltage_supply;
		 ret = regulator_set_voltage(chip->vcc, chip->regulator_voltage_min, chip->regulator_voltage_max);
		 if (ret)
		     pr_info("set vcc regulator voltage failed\n");
		 long_vib_flag = false;
	    }
	    if((val < chip->long_play_threshold) && (!long_vib_flag)){
		chip->regulator_voltage_max = chip->short_voltage_supply;
		chip->regulator_voltage_min = chip->short_voltage_supply;
		ret = regulator_set_voltage(chip->vcc, chip->regulator_voltage_min, chip->regulator_voltage_max);
		if (ret)
			pr_info("set vcc regulator voltage failed\n");
		long_vib_flag = true;
	    }
	}

	mutex_lock(&chip->lock);
	chip->vib_play_ms = val;
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t ext_vib_show_activate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t ext_vib_store_activate(struct device *dev,
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
	pr_info("vib: state = %d, time = %llu ms\n", chip->state, chip->vib_play_ms);
	mutex_unlock(&chip->lock);

	schedule_work(&chip->vib_work);

	return count;
}

static int vcc_2v6 = 0;
static int vcc_3v0 = 0;
static ssize_t ext_vib_show_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return scnprintf(buf, PAGE_SIZE, "vcc_2v6=%d\n", vcc_2v6);
}

static ssize_t ext_vib_store_en(struct device *dev,
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

	mutex_lock(&chip->lock);
	vcc_2v6 = val;
	if(val)
		ret = gpio_direction_output(chip->vcc_2v6_Pin, val);
	else
		ret = gpio_direction_output(chip->vcc_2v6_Pin, val);

	mutex_unlock(&chip->lock);
	return count;
}

static ssize_t ext_vib_show_v3en(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return scnprintf(buf, PAGE_SIZE, "vcc_3v0=%d\n", vcc_3v0);
}

static ssize_t ext_vib_store_v3en(struct device *dev,
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

	mutex_lock(&chip->lock);
	vcc_3v0 = val;
	if(val){
		ret = gpio_direction_output(chip->vcc_3v0_Pin, val);
		ret = gpio_direction_output(chip->vcc_2v6_Pin, val);
	}
	else{
		ret = gpio_direction_output(chip->vcc_3v0_Pin, val);
		ret = gpio_direction_output(chip->vcc_2v6_Pin, val);
	}

	mutex_unlock(&chip->lock);
	return count;
}

static ssize_t ext_vib_show_vcc21(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);

	return scnprintf(buf, PAGE_SIZE, "regulator_voltage = %d\n", chip->regulator_voltage_max);
}

static ssize_t ext_vib_store_vcc21(struct device *dev,
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

	mutex_lock(&chip->lock);
	chip->regulator_voltage_max = val;
	chip->regulator_voltage_min = val;
	ret = regulator_set_voltage(chip->vcc, chip->regulator_voltage_min,
				chip->regulator_voltage_max);
	mutex_unlock(&chip->lock);
	return count;
}

static struct device_attribute ext_vib_attrs[] = {
	__ATTR(state, 0664, ext_vib_show_state, ext_vib_store_state),
	__ATTR(duration, 0664, ext_vib_show_duration, ext_vib_store_duration),
	__ATTR(activate, 0664, ext_vib_show_activate, ext_vib_store_activate),
	__ATTR(en, 0664, ext_vib_show_en, ext_vib_store_en),
	__ATTR(v3en, 0664, ext_vib_show_v3en, ext_vib_store_v3en),
	__ATTR(vcc21, 0664, ext_vib_show_vcc21, ext_vib_store_vcc21),
};

static int ext_vib_parse_dt(struct device *dev, struct vib_ldo_chip *chip)
{
	int ret;
	u32 voltage_supply[2];
	u32 current_supply;
	u32 long_play_threshold;
	u32 voltage_supply_regulator;

	chip->pwr_by_gpio = of_property_read_bool(dev->of_node, "vib-gpio-vcc-enable");
	if (chip->pwr_by_gpio)
	{
		chip->vcc_2v6_Pin = of_get_named_gpio(dev->of_node, "vib,gpio_2v6_en", 0);
		if (!gpio_is_valid(chip->vcc_2v6_Pin))
		{
			pr_err("vib: vcc_2v6_Pin gpio is invalid \n");
			return -ENODEV;
		}
		ret = gpio_request(chip->vcc_2v6_Pin, "vcc_2v6_Pin");
		if (ret < 0)
		{
			pr_err("vib: vcc_2v6_Pin Request gpio fail ret = %d]\n", ret);
			return ret;
		}
		ret = gpio_direction_output(chip->vcc_2v6_Pin, 0);
		if(ret < 0){
			pr_err("vib:vcc_2v6_Pin gpio direction set fail ret = %d\n",ret);
			return ret;
		}
		pr_info("vib: vcc_2v6_Pin gpio num is %d \n", chip->vcc_2v6_Pin);

		chip->vcc_3v0_Pin = of_get_named_gpio(dev->of_node, "vib,gpio_3v0_en", 0);
		if (!gpio_is_valid(chip->vcc_3v0_Pin))
		{
			pr_err("vib: vcc_3v0_Pin gpio is invalid \n");
			return -ENODEV;
		}
		ret = gpio_request(chip->vcc_3v0_Pin, "vcc_3v0_Pin");
		if (ret < 0)
		{
			pr_err("vib: vcc_3v0_Pin Request gpio fail ret = %d]\n", ret);
			return ret;
		}
		ret = gpio_direction_output(chip->vcc_3v0_Pin, 0);
		if(ret < 0){
			pr_err("vib: vcc_3v0_Pin gpio direction set fail ret = %d\n",ret);
			return ret;
		}
		pr_info("vib: vcc_3v0_Pin gpio num is %d \n", chip->vcc_3v0_Pin);

		chip->dis_short_long = of_property_read_bool(dev->of_node,
                                        "qcom,vib-dis-short-long");
		if (chip->dis_short_long) {
			pr_warn("read dis_short_long true");
			ret = of_property_read_u32(dev->of_node, "qcom,vib-dis-short-long-val",
                                &chip->dis_long_ms);
                	if (ret < 0) {
                   	 	pr_err("qcom,vib-dis-short-long-val property read failed, ret=%d\n",ret);
                   	 	return ret;
               	 	}
			pr_warn("read dis_long_ms=%d\n", chip->dis_long_ms);
		}
	}
	else
	{
		chip->pwr_by_regulator = of_property_read_bool(dev->of_node,
                                          "vib,pwr-by-regulator-enable");
		if (chip->pwr_by_regulator)	 {

		    ret = of_property_read_u32_array(dev->of_node, "vib,long-vcc-voltage", &voltage_supply_regulator, 1);
		    if (ret < 0)
		    {
			    pr_info(" fail to get long vcc regulator voltage \n");
			    return -ENODEV;
		    }
		    chip->long_voltage_supply = voltage_supply_regulator;
		    ret = of_property_read_u32_array(dev->of_node, "vib,short-vcc-voltage", &voltage_supply_regulator, 1);
		    if (ret < 0)
		    {
			    pr_info(" fail to get short vcc regulator voltage \n");
			    return -ENODEV;
		    }
		    chip->short_voltage_supply = voltage_supply_regulator;
		    chip->regulator_voltage_max = chip->short_voltage_supply;
		    chip->regulator_voltage_min = chip->short_voltage_supply;

		    ret = of_property_read_u32_array(dev->of_node, "vib,long-play-threshold", &long_play_threshold, 1);
		    if (ret < 0)
		    {
			    pr_info(" fail to get long_play_threshold \n");
			    return -ENODEV;
		    }
		    chip->long_play_threshold = long_play_threshold;
		    pr_info(" vcc regulator voltage get long vib = %d, short vib = %d long_play_threshold = %d\n",
			    chip->long_voltage_supply, chip->short_voltage_supply, chip->long_play_threshold);
		} else {
		    ret = of_property_read_u32_array(dev->of_node, "vib,vcc-voltage", voltage_supply, 2);
		    if (ret < 0)
		    {
			     pr_info("%s: fail to get vcc regulator voltage \n", __func__);
			    return -ENODEV;
		    }

		    pr_info("vib: vcc regulator voltage get Max = %d, Min = %d \n",voltage_supply[1], voltage_supply[0]);
		    chip->regulator_voltage_max = voltage_supply[1];
		    chip->regulator_voltage_min = voltage_supply[0];
		}

		ret = of_property_read_u32_array(dev->of_node, "vib,vcc-current", &current_supply, 1);
		if (ret < 0) {
			pr_info("%s: fail to get vcc regulator current\n", __func__);\
			/*  300mA as default  */
			current_supply = 300000;
		}
		else
			pr_info("vib: vcc regulator current = %d\n", current_supply);

		chip->vcc = devm_regulator_get(dev, "vib_vcc");

		if (IS_ERR(chip->vcc))
		{
			ret = PTR_ERR(chip->vcc);
			pr_info("%s: get vcc regulator failed %d \n", __func__, ret);
			return ret;
		}

		if (regulator_count_voltages(chip->vcc) > 0)
		{
			ret = regulator_set_voltage(chip->vcc, chip->regulator_voltage_min, chip->regulator_voltage_max);
			if (ret)
				pr_info("%s: set vcc regulator voltage failed %d \n", __func__, ret);

			ret = regulator_set_load(chip->vcc, current_supply);
			if (ret)
				pr_info("%s: set vcc regulator current failed %d \n", __func__, ret);
		}
	}

	return ret;
}

/* Dummy functions for brightness */
static enum led_brightness ext_vib_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static void ext_vib_brightness_set(struct led_classdev *cdev,
			enum led_brightness level)
{
}

static int ext_vibrator_ldo_suspend(struct device *dev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	if (chip->pwr_by_gpio)
	{
		gpio_direction_output(chip->vcc_2v6_Pin, 0);
		gpio_direction_output(chip->vcc_3v0_Pin, 0);
	}
	else
		regulator_disable(chip->vcc);
	mutex_unlock(&chip->lock);

	return 0;
}
static SIMPLE_DEV_PM_OPS(ext_vibrator_ldo_pm_ops, ext_vibrator_ldo_suspend,
			NULL);

static int ext_vibrator_ldo_probe(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip;
	int i, ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	ret = ext_vib_parse_dt(&pdev->dev, chip);
	if (ret < 0) {
		pr_err("vib: couldn't parse device tree, ret=%d\n", ret);
		return ret;
	}

	chip->vib_play_ms = EXT_VIB_PLAY_MS;
	mutex_init(&chip->lock);
	INIT_WORK(&chip->vib_work, ext_vib_work);

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = vib_stop_timer;

	dev_set_drvdata(&pdev->dev, chip);

	chip->cdev.name = "vibrator";
	chip->cdev.brightness_get = ext_vib_brightness_get;
	chip->cdev.brightness_set = ext_vib_brightness_set;
	chip->cdev.max_brightness = 100;
	ret = devm_led_classdev_register(&pdev->dev, &chip->cdev);
	if (ret < 0) {
		pr_err("vib: Error in registering led class device, ret=%d\n", ret);
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(ext_vib_attrs); i++) {
		ret = sysfs_create_file(&chip->cdev.dev->kobj,
				&ext_vib_attrs[i].attr);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error in creating sysfs file, ret=%d\n",
				ret);
			goto sysfs_fail;
		}
	}

	pr_info("vib: Vibrator LDO successfully \n");

	return 0;

sysfs_fail:
	for (--i; i >= 0; i--)
		sysfs_remove_file(&chip->cdev.dev->kobj,
				&ext_vib_attrs[i].attr);
fail:
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	return ret;
}

static int ext_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);
  	int i = ARRAY_SIZE(ext_vib_attrs);

	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);

  	if (chip->pwr_by_gpio){
		gpio_free(chip->vcc_2v6_Pin);
		gpio_free(chip->vcc_3v0_Pin);
	}
	else
		devm_regulator_put(chip->vcc);

	for (--i; i >= 0; i--)
		sysfs_remove_file(&chip->cdev.dev->kobj,
				&ext_vib_attrs[i].attr);

	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id vibrator_ldo_match_table[] = {
	{ .compatible = "ext-vibrator-ldo" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vibrator_ldo_match_table);

static struct platform_driver ext_vibrator_ldo_driver = {
	.driver	= {
		.name		= "ext-vibrator-ldo",
		.of_match_table	= vibrator_ldo_match_table,
		.pm		= &ext_vibrator_ldo_pm_ops,
	},
	.probe	= ext_vibrator_ldo_probe,
	.remove	= ext_vibrator_ldo_remove,
};
module_platform_driver(ext_vibrator_ldo_driver);

MODULE_DESCRIPTION("EXT Vibrator-LDO driver");
MODULE_LICENSE("GPL v2");
