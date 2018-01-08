/*
 * Copyright (C) 2013 - 2015 Motorola Mobility LLC
 * Copyright (C) 2018 Motorola Mobility LLC
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
#include <soc/qcom/socinfo.h>
#include "mmi_info.h"

static ssize_t version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "%u.%u\n",
			SOCINFO_VERSION_MAJOR(socinfo_get_version()),
			SOCINFO_VERSION_MINOR(socinfo_get_version()));
}

SYSFS_SIMPLE_SHOW(id, socinfo_get_id(), "%u", 32)
SYSFS_SIMPLE_SHOW(raw_id, socinfo_get_raw_id(), "%u", 32)
SYSFS_SIMPLE_SHOW(raw_version, socinfo_get_raw_version(), "%u", 32)
SYSFS_SIMPLE_SHOW(platform_type, socinfo_get_platform_type(), "%u", 32)
SYSFS_SIMPLE_SHOW(platform_version, socinfo_get_platform_version(), "%u", 32)
SYSFS_SIMPLE_SHOW(platform_subtype, socinfo_get_platform_subtype(), "%u", 32)

SYSFS_ATTRIBUTE(version, 0444)
SYSFS_ATTRIBUTE(id, 0444)
SYSFS_ATTRIBUTE(raw_id, 0444)
SYSFS_ATTRIBUTE(raw_version, 0444)
SYSFS_ATTRIBUTE(platform_type, 0444)
SYSFS_ATTRIBUTE(platform_version, 0444)
SYSFS_ATTRIBUTE(platform_subtype, 0444)

static struct attribute *soc_info_properties_attrs[] = {
	&id_attr.attr,
	&version_attr.attr,
	&raw_id_attr.attr,
	&raw_version_attr.attr,
	&platform_type_attr.attr,
	&platform_version_attr.attr,
	&platform_subtype_attr.attr,
	NULL
};

static struct attribute_group soc_info_properties_attr_group = {
	.attrs = soc_info_properties_attrs,
};

static struct kobject *soc_info_properties_kobj;

int mmi_soc_info_init(void)
{
	int ret = 0;
	int status = 0;

	soc_info_properties_kobj = kobject_create_and_add("soc", NULL);
	if (soc_info_properties_kobj)
		status = sysfs_create_group(soc_info_properties_kobj,
				&soc_info_properties_attr_group);

	if (!soc_info_properties_kobj || status) {
		pr_err("%s: failed to create /sys/soc\n", __func__);
		ret = 1;
		goto err;
	}

err:
	return ret;
}

void mmi_soc_info_exit(void)
{
	sysfs_remove_group(soc_info_properties_kobj,
			&soc_info_properties_attr_group);
	kobject_del(soc_info_properties_kobj);
}
