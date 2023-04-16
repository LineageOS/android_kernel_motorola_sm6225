/*
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
#ifndef __WATCHDOG_CPU_CTX_H
#define __WATCHDOG_CPU_CTX_H

#include <linux/thread_info.h>
#include <soc/qcom/memory_dump.h>

#define MAX_CPU_CTX_SIZE	2048
#define CPU_FORMAT_VERSION4	0x14

#define DUMP_MAGIC_NUMBER	0x42445953
#define CPUCTX_LNX_INFO_MAGIC		0x4c4e5831
#define CPUCTX_STAT_MAGIC		0x4c4e5832
#define CPUCTX_DUMP_DATA_MAGIC		0x4c4e5833
#define CPUCTX_TASK_STRUCT_MAGIC	0x4c4e5834
#define CPUCTX_THREAD_MAGIC		0x4c4e5835

#define MSM_WDOG_CTX_SIG		0x77647473
#define MSM_WDOG_CTX_REV		0x00020000

#define RET_STAGE_SHIFT			(28)
#define RET_SECTION_SHIFT		(24)
#define RET_ERR_SHIFT			(16)
#define RET_MISC_SHIFT			(12)

#define RET_TZ				(0xfu << RET_STAGE_SHIFT)
#define ERR_NONE			(RET_TZ | (0xffu << RET_ERR_SHIFT))
#define ERR_NO_SYMS			(RET_TZ | (0xe9u << RET_ERR_SHIFT))
#define ERR_TASK_INVAL		(RET_TZ | (0xeau << RET_ERR_SHIFT))

#define SC_STATUS_NS		0x01
#define SC_STATUS_WDT		0x02
#define SC_STATUS_SGI		0x04
#define SC_STATUS_WARM_BOOT	0x08
#define SC_STATUS_DBI		0x10
#define SC_STATUS_CTX_BY_TZ	0x20

#ifndef THREAD_SIZE_ORDER
#define THREAD_SIZE_ORDER	(get_order(THREAD_SIZE))
#endif

enum lnx_jobs {
	LNX_STACK,
	LNX_TASK,
	LNX_CTX_AREAS,
};

enum cpuctx_type {
	CPUCTX_LNX_INFO,
	CPUCTX_STAT,
	CPUCTX_DUMP_DATA,
	CPUCTX_TASK_STRUCT,
	CPUCTX_THREAD,
	CPUCTX_TYPES,
	CPUCTX_MAX = 20,
};

struct msm_wdog_copy {
	uint64_t from;
	uint64_t to;
	uint64_t size;
} __packed __aligned(4);

struct sysdbg_cpu64_ctxt_regs {
	uint64_t x0;
	uint64_t x1;
	uint64_t x2;
	uint64_t x3;
	uint64_t x4;
	uint64_t x5;
	uint64_t x6;
	uint64_t x7;
	uint64_t x8;
	uint64_t x9;
	uint64_t x10;
	uint64_t x11;
	uint64_t x12;
	uint64_t x13;
	uint64_t x14;
	uint64_t x15;
	uint64_t x16;
	uint64_t x17;
	uint64_t x18;
	uint64_t x19;
	uint64_t x20;
	uint64_t x21;
	uint64_t x22;
	uint64_t x23;
	uint64_t x24;
	uint64_t x25;
	uint64_t x26;
	uint64_t x27;
	uint64_t x28;
	uint64_t x29;
	uint64_t x30;
	uint64_t pc;
	uint64_t currentEL;
	uint64_t sp_el3;
	uint64_t elr_el3;
	uint64_t spsr_el3;
	uint64_t sp_el2;
	uint64_t elr_el2;
	uint64_t spsr_el2;
	uint64_t sp_el1;
	uint64_t elr_el1;
	uint64_t spsr_el1;
	uint64_t sp_el0;
	uint64_t __reserved1;
	uint64_t __reserved2;
	uint64_t __reserved3;
	uint64_t __reserved4;
};

struct sysdbg_cpu32_ctxt_regs {
	union {
		uint32_t r0;
		uint64_t x0;
	};
	union {
		uint32_t r1;
		uint64_t x1;
	};
	union {
		uint32_t r2;
		uint64_t x2;
	};
	union {
		uint32_t r3;
		uint64_t x3;
	};
	union {
		uint32_t r4;
		uint64_t x4;
	};
	union {
		uint32_t r5;
		uint64_t x5;
	};
	union {
		uint32_t r6;
		uint64_t x6;
	};
	union {
		uint32_t r7;
		uint64_t x7;
	};
	union {
		uint32_t r8;
		uint64_t x8;
	};
	union {
		uint32_t r9;
		uint64_t x9;
	};
	union {
		uint32_t r10;
		uint64_t x10;
	};
	union {
		uint32_t r11;
		uint64_t x11;
	};
	union {
		uint32_t r12;
		uint64_t x12;
	};
	union {
		uint32_t r13_usr;
		uint64_t x13;
	};
	union {
		uint32_t r14_usr;
		uint64_t x14;
	};
	union {
		uint32_t r13_hyp;
		uint64_t x15;
	};
	union {
		uint32_t r14_irq;
		uint64_t x16;
	};
	union {
		uint32_t r13_irq;
		uint64_t x17;
	};
	union {
		uint32_t r14_svc;
		uint64_t x18;
	};
	union {
		uint32_t r13_svc;
		uint64_t x19;
	};
	union {
		uint32_t r14_abt;
		uint64_t x20;
	};
	union {
		uint32_t r13_abt;
		uint64_t x21;
	};
	union {
		uint32_t r14_und;
		uint64_t x22;
	};
	union {
		uint32_t r13_und;
		uint64_t x23;
	};
	union {
		uint32_t r8_fiq;
		uint64_t x24;
	};
	union {
		uint32_t r9_fiq;
		uint64_t x25;
	};
	union {
		uint32_t r10_fiq;
		uint64_t x26;
	};
	union {
		uint32_t r11_fiq;
		uint64_t x27;
	};
	union {
		uint32_t r12_fiq;
		uint64_t x28;
	};
	union {
		uint32_t r13_fiq;
		uint64_t x29;
	};
	union {
		uint32_t r14_fiq;
		uint64_t x30;
	};
	union {
		uint32_t pc;
		uint64_t x31;
	};
	union {
		uint32_t r13_mon;
		uint64_t x32;
	};
	union {
		uint32_t r14_mon;
		uint64_t x33;
	};
	union {
		uint32_t r14_hyp;
		uint64_t x34;
	};
	union {
		uint32_t _reserved;
		uint64_t x35;
	};
	union {
		uint32_t __reserved1;
		uint64_t x36;
	};
	union {
		uint32_t __reserved2;
		uint64_t x37;
	};
	union {
		uint32_t __reserved3;
		uint64_t x38;
	};
	union {
		uint32_t __reserved4;
		uint64_t x39;
	};
};


union sysdbg_cpu_ctxt_regs_type {
	struct sysdbg_cpu32_ctxt_regs	cpu32_ctxt;
	struct sysdbg_cpu64_ctxt_regs	cpu64_ctxt;
};

struct sysdbgCPUCtxtType {
	uint32_t status[4];
	union sysdbg_cpu_ctxt_regs_type cpu_regs;
	union sysdbg_cpu_ctxt_regs_type __reserved3; /* Secure - Not used */
};

union sysdbg_cpuctx {
	struct sysdbgCPUCtxtType data;
	uint8_t space[MAX_CPU_CTX_SIZE];
};

struct msm_wdog_cpuctx_info {
	uint32_t sig;
	uint32_t rev;
	uint32_t ret;
	uint32_t size;
	uint32_t rev2;
	uint32_t syms_avail;
	uint32_t reserve2;
	uint32_t reserve3;
} __packed __aligned(4);

struct msm_wdog_cpuctx_header {
	uint32_t	type; /* cpuctx_type */
	uint32_t	magic;
	uint64_t	offset;
	uint64_t	length;
} __packed __aligned(4);

struct msm_wdog_lnx_info {
	uint32_t tsk_size;
	uint32_t ti_tsk_offset;
	uint32_t aa64;
	uint32_t lpae;
	uint64_t text_paddr;
	uint64_t pgd_paddr;
	uint32_t thread_size;
	uint32_t va_bits;
	uint32_t pa_bits;
	uint32_t page_shift;
	uint32_t aa32_pxn;
	uint32_t pad1;
	uint64_t kaslr_offset;
} __packed __aligned(4);

struct msm_wdog_cpuctx_stat {
	uint32_t ret;
	uint32_t reserved;
	uint64_t stack_va;
	struct msm_wdog_copy jobs[LNX_CTX_AREAS];
} __packed __aligned(4);

struct msm_wdog_cpuctx {
	union sysdbg_cpuctx sysdbg;
	struct msm_wdog_cpuctx_info info;
	struct msm_wdog_cpuctx_header header[CPUCTX_MAX];
	struct msm_wdog_lnx_info lnx;
	struct msm_wdog_cpuctx_stat stat;
	struct msm_dump_data cpu_data;
	struct task_struct task;
	u8 stack[THREAD_SIZE] __aligned(1024);
} __packed __aligned(4);

#define WDOG_CPUCTX_SIZE_PERCPU	(sizeof(struct msm_wdog_cpuctx))
#define WDOG_CPUCTX_SIZE	(num_present_cpus() * WDOG_CPUCTX_SIZE_PERCPU)

#define KASLR_MAGIC_NUM 0xdead4ead
#define KASLR_REGION_LEN 32

#endif /* __WATCHDOG_CPU_CTX_H */
