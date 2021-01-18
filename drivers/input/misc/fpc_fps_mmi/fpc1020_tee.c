/*
 * FPC1020 Fingerprint sensor device driver
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>

#define RESET_LOW_SLEEP_MIN_US 5000
#define RESET_LOW_SLEEP_MAX_US (RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP1_MIN_US 100
#define RESET_HIGH_SLEEP1_MAX_US (RESET_HIGH_SLEEP1_MIN_US + 100)
#define RESET_HIGH_SLEEP2_MIN_US 5000
#define RESET_HIGH_SLEEP2_MAX_US (RESET_HIGH_SLEEP2_MIN_US + 100)

struct FPS_data {
	unsigned int enabled;
	unsigned int state;
	struct blocking_notifier_head nhead;
} *fpsData;

struct FPS_data *FPS_init(struct device *dev)
{
	struct FPS_data *mdata = devm_kzalloc(dev,
			sizeof(struct FPS_data), GFP_KERNEL);
	if (mdata) {
		BLOCKING_INIT_NOTIFIER_HEAD(&mdata->nhead);
		pr_debug("%s: FPS notifier data structure init-ed\n", __func__);
	}
	return mdata;
}

int FPS_register_notifier(struct notifier_block *nb,
	unsigned long stype, bool report)
{
	int error;
	struct FPS_data *mdata = fpsData;

	if (!mdata)
		return -ENODEV;

	mdata->enabled = (unsigned int)stype;
	pr_info("%s: FPS sensor %lu notifier enabled\n", __func__, stype);

	error = blocking_notifier_chain_register(&mdata->nhead, nb);
	if (!error && report) {
		int state = mdata->state;
		/* send current FPS state on register request */
		blocking_notifier_call_chain(&mdata->nhead,
				stype, (void *)&state);
		pr_debug("%s: FPS reported state %d\n", __func__, state);
	}
	return error;
}
EXPORT_SYMBOL_GPL(FPS_register_notifier);

int FPS_unregister_notifier(struct notifier_block *nb,
		unsigned long stype)
{
	int error;
	struct FPS_data *mdata = fpsData;

	if (!mdata)
		return -ENODEV;

	error = blocking_notifier_chain_unregister(&mdata->nhead, nb);
	pr_debug("%s: FPS sensor %lu notifier unregister\n", __func__, stype);

	if (!mdata->nhead.head) {
		mdata->enabled = 0;
		pr_info("%s: FPS sensor %lu no clients\n", __func__, stype);
	}

	return error;
}
EXPORT_SYMBOL_GPL(FPS_unregister_notifier);

void FPS_notify(unsigned long stype, int state)
{
	struct FPS_data *mdata = fpsData;

	pr_debug("%s: Enter", __func__);

	if (!mdata) {
		pr_err("%s: FPS notifier not initialized yet\n", __func__);
		return;
	} else if (!mdata->enabled) {
		pr_debug("%s: !mdata->enabled", __func__);
		return;
	}

	pr_debug("%s: FPS current state %d -> (0x%x)\n", __func__,
	       mdata->state, state);

	if (mdata->state != state) {
		mdata->state = state;
		blocking_notifier_call_chain(&mdata->nhead,
					     stype, (void *)&state);
		pr_debug("%s: FPS notification sent\n", __func__);
	} else
		pr_warn("%s: mdata->state==state", __func__);
}

struct fpc1020_data {
	struct device *dev;
#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
	struct device *class_dev;
#endif
	struct platform_device *pdev;
	struct notifier_block nb;
	int irq_gpio;
	int irq_num;
	unsigned int irq_cnt;
	int rst_gpio;
};

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;
	int rc = 0;

	if (!gpio_is_valid(fpc1020->rst_gpio)) {
		dev_warn(dev, "reset pin is invalid\n");
		goto exit;
	}

	rc = gpio_direction_output(fpc1020->rst_gpio, 1);
	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	rc = gpio_direction_output(fpc1020->rst_gpio, 0);
	if (rc)
		goto exit;
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = gpio_direction_output(fpc1020->rst_gpio, 1);
	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);

exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	rc = hw_reset(fpc1020);

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

static ssize_t dev_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	int state = (*buf == '1') ? 1 : 0;

	FPS_notify(0xbeef, state);
	dev_dbg(fpc1020->dev, "%s state = %d\n", __func__, state);
	return 1;
}
static DEVICE_ATTR(dev_enable, S_IWUSR | S_IWGRP, NULL, dev_enable_set);

static ssize_t irq_get(struct device *device,
		       struct device_attribute *attribute,
		       char *buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}
static DEVICE_ATTR(irq, S_IRUSR | S_IRGRP, irq_get, NULL);

static ssize_t irq_cnt_get(struct device *device,
		       struct device_attribute *attribute,
		       char *buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);

	return scnprintf(buffer, PAGE_SIZE, "%u\n", fpc1020->irq_cnt);
}
static DEVICE_ATTR(irq_cnt, S_IRUSR, irq_cnt_get, NULL);

#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "fpc");
}
static DEVICE_ATTR_RO(vendor);

static ssize_t modalias_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "fpc1020");
}
static DEVICE_ATTR_RO(modalias);
#endif

static struct attribute *attributes[] = {
	&dev_attr_dev_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_irq_cnt.attr,
	&dev_attr_hw_reset.attr,
#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
	&dev_attr_vendor.attr,
	&dev_attr_modalias.attr,
#endif
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
static const struct attribute_group *attribute_groups[] = {
	&attribute_group,
	NULL
};
#endif

#define MAX_UP_TIME (1 * MSEC_PER_SEC)

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	pm_wakeup_event(fpc1020->dev, MAX_UP_TIME);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	fpc1020->irq_cnt++;
#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
	sysfs_notify(&fpc1020->class_dev->kobj, NULL, dev_attr_irq.attr.name);
#else
	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);
#endif
	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc;

	*gpio = of_get_named_gpio(np, label, 0);
	if (!gpio_is_valid(*gpio)) {
		dev_err(dev, "gpio %s is invalid\n", label);
		return -EINVAL;
	}
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);
	return 0;
}

#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
#define MAX_INSTANCE	5
#define MAJOR_BASE	32
static int fpc1020_create_sysfs(struct fpc1020_data *fpc1020, bool create) {
	struct device *dev = fpc1020->dev;
	static struct class *fingerprint_class;
	static dev_t dev_no;
	int rc = 0;

	if (create) {
		rc = alloc_chrdev_region(&dev_no, MAJOR_BASE, MAX_INSTANCE, "fpc");
		if (rc < 0) {
			dev_err(dev, "%s alloc fingerprint class device MAJOR failed.\n", __func__);
			goto ALLOC_REGION;
		}
		if (!fingerprint_class) {
			fingerprint_class = class_create(THIS_MODULE, "fingerprint");
			if (IS_ERR(fingerprint_class)) {
				dev_err(dev, "%s create fingerprint class failed.\n", __func__);
				rc = PTR_ERR(fingerprint_class);
				fingerprint_class = NULL;
				goto CLASS_CREATE_ERR;
			}
		}
		fpc1020->class_dev = device_create_with_groups(fingerprint_class, NULL,
				MAJOR(dev_no), fpc1020, attribute_groups, "fpc1020");
		if (IS_ERR(fpc1020->class_dev)) {
			dev_err(dev, "%s create fingerprint class device failed.\n", __func__);
			rc = PTR_ERR(fpc1020->class_dev);
			fpc1020->class_dev = NULL;
			goto DEVICE_CREATE_ERR;
		}
		return 0;
	}

	device_destroy(fingerprint_class, MAJOR(dev_no));
	fpc1020->class_dev = NULL;
DEVICE_CREATE_ERR:
	class_destroy(fingerprint_class);
	fingerprint_class = NULL;
CLASS_CREATE_ERR:
	unregister_chrdev_region(dev_no, 1);
ALLOC_REGION:
	return rc;
}
#endif

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	int irqf;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		rc = -ENOMEM;
		goto exit;
	}

	fpsData = FPS_init(dev);

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);
	fpc1020->pdev = pdev;

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "irq",
			&fpc1020->irq_gpio);
	gpio_direction_input(fpc1020->irq_gpio);
	if (rc)
		goto exit;

	rc = fpc1020_request_named_gpio(fpc1020, "rst",
			&fpc1020->rst_gpio);
	if (rc)
		fpc1020->rst_gpio = -EINVAL;

	if (gpio_is_valid(fpc1020->rst_gpio)) {
		usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);
		rc = gpio_direction_output(fpc1020->rst_gpio, 1);
		if (rc) {
			dev_err(dev, "cannot set reset pin direction\n");
			goto exit;
		}
	}

	rc = device_init_wakeup(fpc1020->dev, true);
	if (rc)
		goto exit;

#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
	rc = fpc1020_create_sysfs(fpc1020, true);
#else
	rc = sysfs_create_group(&dev->kobj, &attribute_group);
#endif
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	fpc1020->irq_cnt = 0;
	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;

	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto irq_exit;
	}
	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	dev_info(dev, "%s: ok\n", __func__);

	return 0;

irq_exit:
#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
	fpc1020_create_sysfs(fpc1020, false);
#else
	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
#endif
exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(&pdev->dev);

	disable_irq(gpio_to_irq(fpc1020->irq_gpio));
#ifdef CONFIG_INPUT_MISC_FPC1020_SAVE_TO_CLASS_DEVICE
	fpc1020_create_sysfs(fpc1020, false);
#else
	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
#endif

	device_init_wakeup(fpc1020->dev, false);
	devm_free_irq(fpc1020->dev, gpio_to_irq(fpc1020->irq_gpio),fpc1020);

	if (gpio_is_valid(fpc1020->irq_gpio))
		devm_gpio_free(fpc1020->dev, fpc1020->irq_gpio);
	if (gpio_is_valid(fpc1020->rst_gpio))
		devm_gpio_free(fpc1020->dev, fpc1020->rst_gpio);

	dev_info(&pdev->dev, "%s\n", __func__);
	return 0;
}

static int fpc1020_suspend(struct device *dev)
{
	return 0;
}

static int fpc1020_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops fpc1020_pm_ops = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume,
};

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
#if defined(CONFIG_PM)
		.pm = &fpc1020_pm_ops,
#endif
	},
	.probe		= fpc1020_probe,
	.remove		= fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = platform_driver_register(&fpc1020_driver);

	if (!rc)
		pr_debug("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);
	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
