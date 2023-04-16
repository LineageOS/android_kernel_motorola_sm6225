/*
 * Copyright (C) 2012 Motorola Mobility LLC
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
#include <linux/version.h>
#include <linux/mmi_annotate.h>
#include "mmi_info.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
#include <linux/soc/qcom/smem.h>
/* Match to
 * vendor/qcom/nonhlos/boot_images/QcomPkg/SDMPkg/Include/smem_type.h
 */
#define SMEM_SDRAM_INFO 135
#else
#include <soc/qcom/smsm.h>
#define SMEM_SDRAM_INFO SMEM_ID_VENDOR1
#endif

static struct {
	unsigned int mr5;
	unsigned int mr6;
	unsigned int mr7;
	unsigned int mr8;
	unsigned int ramsize;
} *smem_ddr_info;

struct smem_ddr_info_v2 {
	unsigned int mr5;
	unsigned int mr6;
	unsigned int mr7;
	unsigned int mr8;
	unsigned int ramsize;
	unsigned int type;
};

static char sysfsram_type_name[20] = "unknown";
static char sysfsram_vendor_name[20] = "unknown";
static uint32_t sysfsram_ramsize;

static ssize_t mr_register_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	uint32_t val = 0;
	const char *name = attr->attr.name;

	if (smem_ddr_info != NULL &&
		strnlen(name, 4) == 3 && name[0] == 'm' && name[1] == 'r') {
		switch (name[2]) {
		case '5':
			val = smem_ddr_info->mr5;
			break;
		case '6':
			val = smem_ddr_info->mr6;
			break;
		case '7':
			val = smem_ddr_info->mr7;
			break;
		case '8':
			val = smem_ddr_info->mr8;
			break;
		}
	}

	return snprintf(buf, 8, "0x%02x\n", val);
}

static ssize_t info_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 60, "%s:%s:%uMB\n",
			sysfsram_vendor_name,
			sysfsram_type_name,
			sysfsram_ramsize);
}

SYSFS_SIMPLE_SHOW(size, sysfsram_ramsize, "%u", 12)
SYSFS_SIMPLE_SHOW(type, sysfsram_type_name, "%s", 20)

SYSFS_ATTRIBUTE(size, 0444)
SYSFS_ATTRIBUTE(type, 0444)
SYSFS_ATTRIBUTE(info, 0444)

static struct kobj_attribute mr5_register_attr =
	__ATTR(mr5, 0444, mr_register_show, NULL);

static struct kobj_attribute mr6_register_attr =
	__ATTR(mr6, 0444, mr_register_show, NULL);

static struct kobj_attribute mr7_register_attr =
	__ATTR(mr7, 0444, mr_register_show, NULL);

static struct kobj_attribute mr8_register_attr =
	__ATTR(mr8, 0444, mr_register_show, NULL);

static struct attribute *ram_info_properties_attrs[] = {
	&mr5_register_attr.attr,
	&mr6_register_attr.attr,
	&mr7_register_attr.attr,
	&mr8_register_attr.attr,
	&size_attr.attr,
	&type_attr.attr,
	&info_attr.attr,
	NULL
};

static struct attribute_group ram_info_properties_attr_group = {
	.attrs = ram_info_properties_attrs,
};

static struct kobject *ram_info_properties_kobj;

static void parse_ddr_type(void)
{
	uint32_t tid;
	static const char *types[] = {
		"S4 SDRAM",
		"S2 SDRAM",
		"N NVM",
		"Reserved"
	};

	/* identify type */
	tid = smem_ddr_info->mr8 & 0x03;
	if (tid < ARRAY_SIZE(types)) {
		snprintf(sysfsram_type_name, sizeof(sysfsram_type_name),
			"%s", types[tid]);
	}
}

static void parse_ddr_type_v2(struct smem_ddr_info_v2 *ddr_info)
{
	uint32_t tid;
	static const char *types[] = {
		"No DDR",
		"LPDDR1",
		"LPDDR2",
		"PCDDR2",
		"PCDDR3",
		"LPDDR3",
		"LPDDR4",
		"LPDDR4X",
		"LPDDR5",
		"LPDDR5X"
	};

	/* identify type */
	tid = ddr_info->type;
	if (tid < ARRAY_SIZE(types)) {
		snprintf(sysfsram_type_name, sizeof(sysfsram_type_name),
			"%s", types[tid]);
	}
}

int mmi_ram_info_init(void)
{
	int status = 0;
	uint8_t is_v2 = 0;
	uint32_t vid;
	const char *vname = "unknown";
	static const char *vendors[] = {
		"unknown",
		"Samsung",
		"Qimonda",
		"Elpida",
		"Etron",
		"Nanya",
		"Hynix",
		"Mosel",
		"Winbond",
		"ESMT",
		"unknown",
		"Spansion",
		"SST",
		"ZMOS",
		"Intel"
	};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
	ssize_t ddr_info_size = 0;
	smem_ddr_info = qcom_smem_get(QCOM_SMEM_HOST_ANY,
		SMEM_SDRAM_INFO,
		&ddr_info_size);

	if(smem_ddr_info) {
		if(ddr_info_size < sizeof(*smem_ddr_info)) {
			pr_err("%s: Invalid SMEM size, cannot get RAM info\n", __func__);
			goto err;
		}

		/* v2 has an extra field */
		if(ddr_info_size == sizeof(struct smem_ddr_info_v2))
			is_v2 = 1;
	}
#else
	smem_ddr_info = smem_alloc(SMEM_SDRAM_INFO, sizeof(*smem_ddr_info), 0,
			SMEM_ANY_HOST_FLAG);
#endif
	if (smem_ddr_info == NULL) {
		pr_err("%s: failed to access RAM info in SMEM\n", __func__);
		goto err;
	}

	/* identify vendor */
	vid = smem_ddr_info->mr5 & 0xFF;
	if (vid < ARRAY_SIZE(vendors))
		vname = vendors[vid];
	else if (vid == 0xFE)
		vname = "Numonyx";
	else if (vid == 0xFF)
		vname = "Micron";

	snprintf(sysfsram_vendor_name, sizeof(sysfsram_vendor_name),
		"%s", vname);

	if(is_v2)
		parse_ddr_type_v2((struct smem_ddr_info_v2 *)smem_ddr_info);
	else
		parse_ddr_type();

	/* extract size */
	sysfsram_ramsize = smem_ddr_info->ramsize;

	mmi_annotate_persist(
		"RAM: %s, %s, %u MB, MR5:0x%02X, MR6:0x%02X, "
		"MR7:0x%02X, MR8:0x%02X\n",
		vname, sysfsram_type_name, smem_ddr_info->ramsize,
		smem_ddr_info->mr5, smem_ddr_info->mr6,
		smem_ddr_info->mr7, smem_ddr_info->mr8);

	/* create sysfs object */
	ram_info_properties_kobj = kobject_create_and_add("ram", NULL);

	if (ram_info_properties_kobj)
		status = sysfs_create_group(ram_info_properties_kobj,
			&ram_info_properties_attr_group);

	if (!ram_info_properties_kobj || status) {
		pr_err("%s: failed to create /sys/ram\n", __func__);
		goto err;
	}

	return 0;
err:
	return 1;
}

void mmi_ram_info_exit(void)
{
	sysfs_remove_group(ram_info_properties_kobj,
			&ram_info_properties_attr_group);
	kobject_del(ram_info_properties_kobj);
}
