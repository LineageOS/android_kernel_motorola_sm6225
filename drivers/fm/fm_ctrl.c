/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>


#define DRIVER_VERSION "0.0.1"

struct fm_ctrl_drvdata {
	struct device	*dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pstate_default;
	struct pinctrl_state *pstate_active;
	struct pinctrl_state *pstate_suspend;
	bool   factory_mode;
};


static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

static ssize_t device_name_read(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fm_ctrl_drvdata *data = dev_get_drvdata(dev);

	if (!data) {
		pr_err("fm_ctrl drvdata is NULL\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "moto fm control intf\n");
}


static ssize_t elna_en_read(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct fm_ctrl_drvdata *data = dev_get_drvdata(dev);

	if (!data) {
		pr_err("fm_ctrl drvdata is NULL\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "fm_ctrl: not support!\n");
}

static ssize_t elna_en_write(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct fm_ctrl_drvdata *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int res = 0;

	if (!data) {
		pr_err("fm_ctrl drvdata is NULL\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &res);
	if(ret) {
		pr_err("fm_ctrl failed to get data, set as default!\n");
	}
	if(1 == res) {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_active);
	}
	else {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_suspend);
	}
	if(ret) {
		pr_err("fm_ctrl failed to set pinctrl!\n");
	}
	else {
		pr_info("fm_ctrl set elan=%u\n", res);
	}

	return count;
}


static DEVICE_ATTR(device_name, 0444, device_name_read, NULL);
static DEVICE_ATTR(elna_en, 0644, elna_en_read, elna_en_write);

static struct attribute *fm_ctrl_sysfs_attrs[] = {
	&dev_attr_device_name.attr,
	&dev_attr_elna_en.attr,
	NULL,
};

static struct attribute_group fm_ctrl_sysfs_attr_grp = {
	.attrs = fm_ctrl_sysfs_attrs,
};


static int fm_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fm_ctrl_drvdata *drvdata;
	int ret = 0;

	dev_dbg(&pdev->dev, "%s begin\n", __func__);
	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	/* Get pinctrl if target uses pinctrl */
	drvdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(drvdata->pinctrl)) {
		ret = PTR_ERR(drvdata->pinctrl);
		pr_err("%s: Pincontrol DT property returned %X\n", __func__, ret);
		return ret;
	}

	drvdata->pstate_default = pinctrl_lookup_state(drvdata->pinctrl,
		"default");
	if (IS_ERR_OR_NULL(drvdata->pstate_default)) {
		ret = PTR_ERR(drvdata->pstate_default);
		pr_err("Can not lookup default pinstate %d\n", ret);
		return -ENOENT;
	}
	drvdata->pstate_active = pinctrl_lookup_state(drvdata->pinctrl,
		"elna_active");
	if (IS_ERR_OR_NULL(drvdata->pstate_active)) {
		ret = PTR_ERR(drvdata->pstate_active);
		pr_err("Can not lookup active pinstate %d\n", ret);
		return ret;
	}
	drvdata->pstate_suspend = pinctrl_lookup_state(drvdata->pinctrl,
		"elna_suspend");
	if (IS_ERR_OR_NULL(drvdata->pstate_suspend)) {
		ret = PTR_ERR(drvdata->pstate_suspend);
		pr_err("Can not lookup suspend pinstate %d\n", ret);
		return ret;
	}
	drvdata->factory_mode = mmi_factory_check();
	if(drvdata->factory_mode) {
		ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_active);
		if(ret) {
			pr_err("fm_ctrl failed to set pinctrl @factory mode!\n");
		}
		else {
			pr_info("fm_ctrl enable elan @ factory mode\n");
		}
	}
	else {
		ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_default);
		if(ret) {
			pr_err("fm_ctrl failed to set pinctrl as default!\n");
		}
		else {
			pr_info("fm_ctrl disable elan as default.\n");
		}
	}

	ret = sysfs_create_group(&dev->kobj, &fm_ctrl_sysfs_attr_grp);
	if (ret) {
		pr_err("%s: sysfs group creation failed %d\n", __func__, ret);
		return ret;
	}

	device_init_wakeup(&pdev->dev, 1);
	dev_info(&pdev->dev, "probe: All success !\n");

	return ret;
}

static int fm_ctrl_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);
	return 0;
}


static const struct of_device_id fm_ctrl_match[] = {
	{ .compatible = "moto,fmctrl" },
	{}
};

static struct platform_driver fm_ctrl_plat_driver = {
	.probe = fm_ctrl_probe,
	.remove = fm_ctrl_remove,
	.driver = {
		.name = "fm_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = fm_ctrl_match,
	},
};

module_platform_driver(fm_ctrl_plat_driver);


MODULE_AUTHOR("Motorola Mobiity");
MODULE_DESCRIPTION("FMRadio control interface driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
