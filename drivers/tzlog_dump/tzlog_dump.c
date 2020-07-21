/* Copyright (c) 2011-2015,2017 The Linux Foundation. All rights reserved.
 * Copyright (C) 2018 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/msm_ion.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ctype.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/cma.h>
#include <soc/qcom/memory_dump.h>
#include <soc/qcom/mmi_boot_info.h>
#include <linux/mmi_annotate.h>
#include <linux/version.h>

/* Check memory_dump.h to verify this is not going over the max or
 * conflicting with another entry. Also must match BL
 */
#define MSM_DUMP_DATA_TZ_LOG 0x11F

#define TZBSP_DIAG_SIZE 0x3000
#define TZBSP_MAGIC_NUMBER 0x747A6461
#define TZBSP_VERSION 0x90001
#define TZBSP_MAX_CPU_COUNT 0x08
#define TZBSP_DIAG_NUM_OF_VMID 16
#define TZBSP_DIAG_VMID_DESC_LEN 7
#define TZBSP_DIAG_INT_NUM  64
#define TZBSP_MAX_INT_DESC 16
#define TZBSP_AES_256_ENCRYPTED_KEY_SIZE 256
#define TZBSP_NONCE_LEN 12
#define TZBSP_TAG_LEN 16

struct tzdbg_vmid_t {
	uint8_t vmid;
	uint8_t desc[TZBSP_DIAG_VMID_DESC_LEN];
};

struct tzdbg_boot_info_t {
	uint32_t wb_entry_cnt;
	uint32_t wb_exit_cnt;
	uint32_t pc_entry_cnt;
	uint32_t pc_exit_cnt;
	uint32_t psci_entry_cnt;
	uint32_t psci_exit_cnt;
	uint64_t warm_jmp_addr;
	uint32_t warm_jmp_instr;
};

struct tzdbg_reset_info_t {
	uint32_t reset_type;
	uint32_t reset_cnt;
};

struct tzdbg_int_t {
	uint16_t int_info;
	uint8_t avail;
	uint8_t spare;
	uint32_t int_num;
	uint8_t int_desc[TZBSP_MAX_INT_DESC];
	uint64_t int_count[TZBSP_MAX_CPU_COUNT];
};

struct tzbsp_diag_wakeup_info_t {
	uint32_t HPPIR;
	uint32_t AHPPIR;
};

struct tzdbg_log_pos_t {
	uint16_t wrap;
	uint16_t offset;
};

struct tzdbg_log_t {
	struct tzdbg_log_pos_t	log_pos;
	uint8_t	log_buf[];
};

struct tzdbg_t {
	uint32_t magic_num;
	uint32_t version;
	uint32_t cpu_count;
	uint32_t vmid_info_off;
	uint32_t boot_info_off;
	uint32_t reset_info_off;
	uint32_t int_info_off;
	uint32_t ring_off;
	uint32_t ring_len;
	uint32_t wakeup_info_off;
	struct tzdbg_vmid_t vmid_info[TZBSP_DIAG_NUM_OF_VMID];
	struct tzdbg_boot_info_t  boot_info[TZBSP_MAX_CPU_COUNT];
	struct tzdbg_reset_info_t reset_info[TZBSP_MAX_CPU_COUNT];
	uint32_t num_interrupts;
	struct tzdbg_int_t  int_info[TZBSP_DIAG_INT_NUM];
	struct tzbsp_diag_wakeup_info_t  wakeup_info[TZBSP_MAX_CPU_COUNT];
	uint8_t key[TZBSP_AES_256_ENCRYPTED_KEY_SIZE];
	uint8_t nonce[TZBSP_NONCE_LEN];
	uint8_t tag[TZBSP_TAG_LEN];
	struct tzdbg_log_t ring_buffer;
};

struct tz_dump_platform_data {
	phys_addr_t	mem_address;
	size_t		mem_size;
};

static struct tzdbg_t *tzdbg_data;

#define MSMDBG(fmt, args...) mmi_annotate(fmt, ##args)

#define MSMWDTD_IFWDOG(fmt, args...) do { \
	if (bi_powerup_reason() == PU_REASON_WDOG_AP_RESET) \
		MSMDBG(fmt, ##args); \
} while (0)

static void tzlog_dump_show_boot_info(void)
{
	int cpu;
	int power_collapsed;
	struct tzdbg_boot_info_t *ptr;

	ptr = (struct tzdbg_boot_info_t *)
			((u8 *)tzdbg_data + tzdbg_data->boot_info_off);

	MSMWDTD_IFWDOG("\n--- TZ Power Collapse Counters\n");
	MSMWDTD_IFWDOG("     | WarmEntry : WarmExit : TermEntry :");
	MSMWDTD_IFWDOG(" TermExit : PsciEntry : PsciExit : JumpAddr |\n");
	for (cpu = 0; cpu < tzdbg_data->cpu_count; cpu++) {
		power_collapsed = ptr->wb_entry_cnt +
				ptr->pc_exit_cnt - ptr->pc_entry_cnt;
		if (cpu)
			power_collapsed--;
		MSMWDTD_IFWDOG("CPU%d |  %8x : %8x : %8x : %8x : %8x : %8x :      "
			"%llx | %sPC\n",
			cpu,
			ptr->wb_entry_cnt,
			ptr->wb_exit_cnt,
			ptr->pc_entry_cnt,
			ptr->pc_exit_cnt,
			ptr->psci_entry_cnt,
			ptr->psci_exit_cnt,
			ptr->warm_jmp_addr,
			power_collapsed ? "IN-" : "NOT-");
		ptr++;
	}
}

static void tzlog_dump_show_log(void)
{
	struct tzdbg_log_t *log_ptr;
	const char *log_buf, *p, *start;

	log_buf = (const char *)tzdbg_data + tzdbg_data->ring_off;
	log_ptr = (struct tzdbg_log_t *)(log_buf -
				offsetof(struct tzdbg_log_t, log_buf));

	if (log_ptr->log_pos.offset >= tzdbg_data->ring_len)
		return;
	MSMWDTD_IFWDOG("--- TZ Log start ---\n");
	if (log_ptr->log_pos.wrap) {
		for (start = log_buf + log_ptr->log_pos.offset, p = start;
				p < (log_buf + tzdbg_data->ring_len); p++) {
			if (isprint(*p))
				MSMWDTD_IFWDOG("%c", *p);
			else if ((p > start) && isprint(*(p-1)))
				MSMWDTD_IFWDOG("\n");
		}
	}
	for (start = log_buf, p = start;
			p < (log_buf + log_ptr->log_pos.offset); p++) {
		if (isprint(*p))
			MSMWDTD_IFWDOG("%c", *p);
		else if ((p > start) && isprint(*(p-1)))
			MSMWDTD_IFWDOG("\n");
	}
	MSMWDTD_IFWDOG("\n--- TZ Log end ---\n");
}

static int tzlog_dump_annotate(void)
{
	if (!tzdbg_data ||
		tzdbg_data->magic_num != TZBSP_MAGIC_NUMBER) {
		MSMWDTD_IFWDOG("No valid backup\n");
		return 0;
	}

	tzlog_dump_show_boot_info();
	tzlog_dump_show_log();

	return 0;
}

static void tzlog_dump_table_register(struct device *dev,
	struct tz_dump_platform_data *pdata)
{
	int err = 0;
	struct msm_dump_entry dump_entry;
	struct msm_dump_data *tz_dump_data;

	tz_dump_data = kzalloc(sizeof(struct msm_dump_data),
					GFP_KERNEL);

	if (!tz_dump_data) {
		dev_err(dev, "Cannot alloc dump data structure.\n");
		goto err;
	}

	tz_dump_data->addr = pdata->mem_address;
	tz_dump_data->len = pdata->mem_size;
	dump_entry.id = MSM_DUMP_DATA_TZ_LOG;
	dump_entry.addr = virt_to_phys(tz_dump_data);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
	err = msm_dump_data_register(MSM_DUMP_TABLE_APPS, &dump_entry);
#else
	//walkaround the compile error in GKI
#ifdef CONFIG_QGKI
	err = msm_dump_data_register(MSM_DUMP_TABLE_APPS, &dump_entry);
#endif
#endif
	if (err) {
		dev_err(dev, "Registering dump data failed.\n");
		kfree(tz_dump_data);
	}
err:
	return;
}

static int tzlog_dump_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tz_dump_platform_data *pdata;
	struct resource res;
	struct device_node *node;
	int err = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		err = -ENOMEM;
		goto err;
	}

	/* Get reserved memory region from Device-tree */
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(dev, "No %s specified\n", "memory-region");
		goto err;
	}

	err = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (err) {
		dev_err(dev, "No memory address assigned to the region\n");
		goto err;
	}

	pdata->mem_size = resource_size(&res);
	pdata->mem_address = res.start;

	dev_info(dev, "tzlog_dump_size %zx", pdata->mem_size);
	dev_info(dev, "tzlog_dump_addr %lx", (unsigned long)pdata->mem_address);

	if (pdata->mem_size < TZBSP_DIAG_SIZE) {
		dev_err(dev, "Mem reserve too small %zx/%xu\n",
				pdata->mem_size, TZBSP_DIAG_SIZE);
		err = -ENOMEM;
		goto err;
	}

	tzlog_dump_table_register(dev, pdata);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
	tzdbg_data = ioremap_wc(pdata->mem_address, pdata->mem_size);
#else
	tzdbg_data = dma_remap(dev, NULL, pdata->mem_address,
					pdata->mem_size, 0);
#endif

	if (!tzdbg_data) {
		dev_err(dev, "Cannot remap buffer %pa size %zx\n",
				&pdata->mem_address, pdata->mem_size);
		err = -ENOMEM;
		goto err;
	}

	tzlog_dump_annotate();
	memset_io(tzdbg_data, 0, pdata->mem_size);
err:
	return err;
}

static int tzlog_dump_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id tzlog_dump_match[] = {
	{ .compatible = "mmi,tzlog-dump" },
	{}
};

static struct platform_driver tzlog_dump_driver = {
	.probe		= tzlog_dump_probe,
	.remove		= tzlog_dump_remove,
	.driver		= {
		.name = "tzlog_dump",
		.of_match_table = tzlog_dump_match,
	},
};

static int __init tzlog_dump_init(void)
{
	return platform_driver_register(&tzlog_dump_driver);
}

static void __exit tzlog_dump_exit(void)
{
	platform_driver_unregister(&tzlog_dump_driver);
}

module_init(tzlog_dump_init);
module_exit(tzlog_dump_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TZ Log dump driver");
