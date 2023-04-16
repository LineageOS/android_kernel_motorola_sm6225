/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2013-2020 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _MC_PLATFORM_H_
#define _MC_PLATFORM_H_

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <linux/errno.h>
#include <linux/types.h>

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
#define USE_SHM_BRIDGE
#endif

#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
#include <linux/qcom_scm.h>
#include <soc/qcom/qseecomi.h>
#if defined USE_SHM_BRIDGE
#include <linux/qtee_shmbridge.h>
#endif
#elif KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
#include <soc/qcom/scm.h>
#include <soc/qcom/qseecomi.h>
#if defined USE_SHM_BRIDGE
#include <soc/qcom/qtee_shmbridge.h>
#endif
#endif

/*--------------- Implementation -------------- */
/* MobiCore Interrupt for Qualcomm (DT IRQ has priority if present) */
#define MC_INTR_SSIQ	280

/* Use SMC for fastcalls */
#define MC_SMC_FASTCALL

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0)
#define SCM_MOBIOS_FNID(s, c) (((((s) & 0xFF) << 8) | ((c) & 0xFF)) \
		| 0x33000000)

#define TZ_EXECUTIVE_EXT_ID_PARAM_ID \
	TZ_SYSCALL_CREATE_PARAM_ID_4( \
			TZ_SYSCALL_PARAM_TYPE_BUF_RW, \
			TZ_SYSCALL_PARAM_TYPE_VAL, \
			TZ_SYSCALL_PARAM_TYPE_BUF_RW, \
			TZ_SYSCALL_PARAM_TYPE_VAL)
#endif

/* from following file */
#define SCM_SVC_MOBICORE		250
#define SCM_CMD_MOBICORE		1

#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
extern int trustonic_smc_fastcall(void *fc_generic, size_t size);
static inline int smc_fastcall(void *fc_generic, size_t size)
{
	return trustonic_smc_fastcall(fc_generic, size);
}

#elif KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
static inline int smc_fastcall(void *fc_generic, size_t size)
{
#if !defined(USE_SHM_BRIDGE)
	if (is_scm_armv8()) {
#endif
		struct scm_desc desc = {0};
		int ret;
		static void *scm_buf;
		static phys_addr_t scm_buf_pa;

		if (!scm_buf) {
#if defined USE_SHM_BRIDGE
			static struct qtee_shm scm_shm = {0};

			ret = qtee_shmbridge_allocate_shm(PAGE_ALIGN(size),
							  &scm_shm);
			scm_buf = scm_shm.vaddr;
			scm_buf_pa = scm_shm.paddr;
#else
			scm_buf = kzalloc(PAGE_ALIGN(size), GFP_KERNEL);
			scm_buf_pa = virt_to_phys(scm_buf);
#endif
		}
		if (!scm_buf)
			return -ENOMEM;
		memcpy(scm_buf, fc_generic, size);
		dmac_flush_range(scm_buf, scm_buf + size);

		desc.arginfo = TZ_EXECUTIVE_EXT_ID_PARAM_ID;
		desc.args[0] = scm_buf_pa;
		desc.args[1] = (u32)size;
		desc.args[2] = scm_buf_pa;
		desc.args[3] = (u32)size;

		ret = scm_call2(
			SCM_MOBIOS_FNID(SCM_SVC_MOBICORE, SCM_CMD_MOBICORE),
				&desc);

		dmac_flush_range(scm_buf, scm_buf + size);

		memcpy(fc_generic, scm_buf, size);
		return ret;
#if !defined(USE_SHM_BRIDGE)
	}
	return scm_call(SCM_SVC_MOBICORE, SCM_CMD_MOBICORE,
			fc_generic, size,
			fc_generic, size);
#endif
}
#endif

/*
 * Do not start the TEE at driver init
 */
#define MC_DELAYED_TEE_START

/*
 * Perform crypto clock enable/disable
 * of clocks
 *	 "bus_clk"
 *	 "core_clk"
 *	 "iface_clk"
 */
#ifndef RSU_INTERNAL_CLOCK
#define MC_CRYPTO_CLOCK_MANAGEMENT
#endif
/*
 * This should be defined only on platforms without ICE clock e.g SDM845, SM6150, SM7225
 * #define TT_CRYPTO_NO_CLOCK_SUPPORT_FEATURE "qcom,no-clock-support"
 */
#define MC_CRYPTO_CLOCK_CORESRC_PROPNAME "qcom,ce-opp-freq"

#if defined USE_SHM_BRIDGE
#define MC_CLOCK_CORESRC_DEFAULTRATE 192000000
#else
#define MC_CLOCK_CORESRC_DEFAULTRATE 100000000
#endif

/*
 * Perform clock enable/disable for clock  "core_clk_src"
 */
#define MC_DEVICE_PROPNAME "qcom,mcd"
/*
 * Platform Node
 */
#define TT_CLOCK_DEVICE_NAME "qcom,qseecom"

/* All TZBSPv4 targets are using AARCH32_FC flag */
#define MC_AARCH32_FC

/* This should be defined only on some platforms e.g SM8150, SM6150, SM7225
 * from which the Gold cores do not support TEE interfaces
 * so that CPU_IDS should list only Silver cores.
 */
/* Enforce/restrict statically CPUs potentially running TEE
 * (Customize to match platform CPU layout... 0xF0 for big cores only for ex).
 * If not defined TEE dynamically using all platform CPUs (recommended)
 * Warning: Both PLAT_DEFAULT_TEE_AFFINITY_MASK and
 * BIG_CORE_SWITCH_AFFINITY_MASK have to be defined
 */
#define PLAT_DEFAULT_TEE_AFFINITY_MASK (0xF)
#define BIG_CORE_SWITCH_AFFINITY_MASK (0xF)

#endif /* _MC_PLATFORM_H_ */
