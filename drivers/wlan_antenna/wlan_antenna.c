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

struct wlan_antenna_drvdata {
	struct device	*dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pstate_active;
	struct pinctrl_state *pstate_suspend;
};

static ssize_t wlan_antenna_en_read(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct wlan_antenna_drvdata *data = dev_get_drvdata(dev);

	if (!data) {
		pr_err("wlan_antenna drvdata is NULL\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "wlan_antenna: not support!\n");
}

static ssize_t wlan_antenna_en_write(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct wlan_antenna_drvdata *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int res = 0;

	if (!data) {
		pr_err("wlan_antenna drvdata is NULL\n");
		return -EINVAL;
	}

	ret = kstrtouint(buf, 0, &res);
	if(ret) {
		pr_err("wlan_antenna failed to get data, set as default!\n");
	}

	if(1 == res) {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_active);
	} else {
		ret = pinctrl_select_state(data->pinctrl, data->pstate_suspend);
	}

	if(ret) {
		pr_err("wlan_antenna failed to set pinctrl!\n");
	} else {
		pr_info("wlan_antenna success set pinctrl\n");
	}

	return count;
}

static DEVICE_ATTR(wlan_antenna_en, 0644, wlan_antenna_en_read, wlan_antenna_en_write);

static struct attribute *wlan_antenna_sysfs_attrs[] = {
	&dev_attr_wlan_antenna_en.attr,
	NULL,
};

static struct attribute_group wlan_antenna_sysfs_attr_grp = {
	.attrs = wlan_antenna_sysfs_attrs,
};

static int wlan_antenna_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wlan_antenna_drvdata *drvdata;
	int ret = 0;

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

	drvdata->pstate_active = pinctrl_lookup_state(drvdata->pinctrl, "antenna_active");
	if (IS_ERR_OR_NULL(drvdata->pstate_active)) {
		ret = PTR_ERR(drvdata->pstate_active);
		pr_err("Can not lookup antenna_active pinstate %d\n", ret);
		return ret;
	}

	drvdata->pstate_suspend = pinctrl_lookup_state(drvdata->pinctrl, "antenna_suspend");
	if (IS_ERR_OR_NULL(drvdata->pstate_suspend)) {
		ret = PTR_ERR(drvdata->pstate_suspend);
		pr_err("Can not lookup antenna_suspend pinstate %d\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(drvdata->pinctrl, drvdata->pstate_suspend);
	if(ret) {
		pr_err("wlan_antenna failed to set pinctrl pstate_suspend!\n");
	} else {
		pr_info("wlan_antenna enable pstate_suspend\n");
	}

	ret = sysfs_create_group(&dev->kobj, &wlan_antenna_sysfs_attr_grp);
	if (ret) {
		pr_err("%s: sysfs group creation failed %d\n", __func__, ret);
		return ret;
	}
	dev_info(&pdev->dev, "probe: All success !\n");

	return ret;
}

static int wlan_antenna_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id wlan_antenna_match[] = {
	{ .compatible = "moto,wlan_antenna" },
	{}
};

static struct platform_driver wlan_antenna_plat_driver = {
	.probe = wlan_antenna_probe,
	.remove = wlan_antenna_remove,
	.driver = {
		.name = "wlan_antenna",
		.owner = THIS_MODULE,
		.of_match_table = wlan_antenna_match,
	},
};

static int wlan_antenna_init(void)
{
	return platform_driver_register(&wlan_antenna_plat_driver);
}

static void wlan_antenna_exit(void)
{
	platform_driver_unregister(&wlan_antenna_plat_driver);
}

module_init(wlan_antenna_init);
module_exit(wlan_antenna_exit);

MODULE_AUTHOR("Motorola Mobiity");
MODULE_DESCRIPTION("wlan antenna control interface driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
