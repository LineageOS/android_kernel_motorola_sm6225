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
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ctype.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <asm/stacktrace.h>
#include <asm/memory.h>
#include <asm/sections.h>
#include <asm/pgtable.h>
#include <asm/cputype.h>
#include <linux/kallsyms.h>
#include <linux/io.h>
#include <linux/mm_types.h>
#include <linux/mmi_annotate.h>
#include <soc/qcom/mmi_boot_info.h>
#include "watchdog_cpu_ctx.h"

#ifndef TASK_STATE_TO_CHAR_STR
#define TASK_STATE_TO_CHAR_STR "RSDTtXZxKWPNn"
#endif

#define MSMDBG(fmt, args...) mmi_annotate(fmt, ##args)

#define MSMWDT_ERR(fmt, args...) do { \
	pr_err("WdogCtx: "fmt, ##args); \
	MSMWDTD("WdogCtx: "fmt, ##args); \
} while (0)

#define MSMWDTD_IFWDOG(fmt, args...) do { \
	if (bi_powerup_reason() == PU_REASON_WDOG_AP_RESET) \
		MSMDBG(fmt, ##args); \
} while (0)

#define MSMWDTD(fmt, args...) MSMWDTD_IFWDOG(fmt, ##args)

/* Check memory_dump.h to verify this is not going over the max or
 * conflicting with another entry. Also must match BL
 */
#define MSM_DUMP_DATA_WDOG_CPU_CTX 0x200

#define KASLR_OFFSET_PROP "qcom,msm-imem-kaslr_offset"

struct platform_data {
	phys_addr_t	mem_address;
	size_t		mem_size;
};

const struct msm_wdog_cpuctx_header mwc_header[] = {
	{
		.type	= CPUCTX_LNX_INFO,
		.magic	= CPUCTX_LNX_INFO_MAGIC,
		.offset	= offsetof(struct msm_wdog_cpuctx, lnx),
		.length	= sizeof(struct msm_wdog_lnx_info),
	},
	{
		.type	= CPUCTX_STAT,
		.magic	= CPUCTX_STAT_MAGIC,
		.offset	= offsetof(struct msm_wdog_cpuctx, stat),
		.length	= sizeof(struct msm_wdog_cpuctx_stat),
	},
	{
		.type	= CPUCTX_DUMP_DATA,
		.magic	= CPUCTX_DUMP_DATA_MAGIC,
		.offset	= offsetof(struct msm_wdog_cpuctx, cpu_data),
		.length	= sizeof(struct msm_dump_data),
	},
	{
		.type	= CPUCTX_TASK_STRUCT,
		.magic	= CPUCTX_TASK_STRUCT_MAGIC,
		.offset	= offsetof(struct msm_wdog_cpuctx, task),
		.length	= sizeof(struct task_struct),
	},
	{
		.type	= CPUCTX_THREAD,
		.magic	= CPUCTX_THREAD_MAGIC,
		.offset	= offsetof(struct msm_wdog_cpuctx, stack),
		.length	= THREAD_SIZE,
	},
};

#if defined(CONFIG_ARM64)
static int aa64_pa_max(void)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,9,0)
	int pa_range = read_cpuid(SYS_ID_AA64MMFR0_EL1) & 0xf;
#else
	int pa_range = read_cpuid(ID_AA64MMFR0_EL1) & 0xf;
#endif
	switch (pa_range) {
	case 0x0:
		return 32;
	case 0x1:
		return 36;
	case 0x2:
		return 40;
	case 0x3:
		return 42;
	case 0x4:
		return 44;
	case 0x5:
		return 48;
	default:
		pr_err("PAMax: Reserved PARange: %#x\n", pa_range);
		break;
	}
	return 32;
}

static unsigned long get_kaslr_offset(void)
{
	void __iomem *kaslr_addr;
	unsigned kaslr_magic_num;
	unsigned kaslr_offset_low = 0;
	unsigned kaslr_offset_high = 0;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, KASLR_OFFSET_PROP);
	if (!np) {
		pr_err("Unable to find %s node\n", KASLR_OFFSET_PROP);
		goto out;
	} else {
		kaslr_addr = of_iomap(np, 0);
	}

	if(!kaslr_addr)
		goto out;

	kaslr_magic_num = ioread32(kaslr_addr);
	if(kaslr_magic_num != KASLR_MAGIC_NUM)
		goto unmap;

	kaslr_offset_low = ioread32(kaslr_addr + 4);
	kaslr_offset_high = ioread32(kaslr_addr + 8);
unmap:
	iounmap(kaslr_addr);
out:
	return ((unsigned long)kaslr_offset_high << 32 |
		(unsigned long)kaslr_offset_low);
}

static void msm_wdog_ctx_lnx_arch(struct msm_wdog_lnx_info *lnx)
{
	lnx->va_bits = VA_BITS;
	lnx->pa_bits = aa64_pa_max();
	lnx->kaslr_offset = get_kaslr_offset();
}

#elif defined(CONFIG_ARM)

struct wdog_frame_struct {
	unsigned long addr;
	unsigned long stack;
};

#if defined(CONFIG_ARM_LPAE)
static int armv7_pa_max(void)
{
	int cached_mem_size = (read_cpuid_ext(CPUID_EXT_MMFR3) >> 24) & 0xf;

	switch (cached_mem_size) {
	case 0x0:
		return 32;
	case 0x1:
		return 36;
	case 0x2:
		return 40;
	default:
		pr_err("PAMax: Unknown mem size: %#x\n", cached_mem_size);
		break;
	}
	return 32;
}
#else
static int armv7_pa_max(void)
{
	int supersection = (read_cpuid_ext(CPUID_EXT_MMFR3) >> 28) & 0xf;

	if ((supersection == 0x0) && IS_ENABLED(CONFIG_PHYS_ADDR_T_64BIT))
		return 40;
	else
		return 32;
}
#endif /* CONFIG_ARM_LPAE */

static void msm_wdog_ctx_lnx_arch(struct msm_wdog_lnx_info *lnx)
{
	lnx->va_bits = 32;
	lnx->pa_bits = armv7_pa_max();
	lnx->aa32_pxn = ((read_cpuid_ext(CPUID_EXT_MMFR0) & 0xf) >= 0x4);
	lnx->kaslr_offset = 0;
}
#endif

static void msm_wdog_ctx_lnx(struct msm_wdog_lnx_info *lnx)
{
	/* Since this is a module we cannot link to these symbols
	 * directly, so use kallsyms. kallsyms is needed to find the
	 * names of the stacktrace symbols anyways.
	 */
	unsigned long _text_addr = kallsyms_lookup_name("_text");
	/* This is a way to get the address of swapper_pg_dir which
	 * is not available via kallsyms_lookup_name directly
	 */
	unsigned long init_mm_addr = kallsyms_lookup_name("init_mm");
	struct mm_struct *init_mm = NULL;
	if(init_mm_addr) {
		init_mm = (struct mm_struct *)init_mm_addr;
	}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,9,0) || IS_ENABLED(CONFIG_ARM)
	lnx->ti_tsk_offset = offsetof(struct thread_info, task);
#else
	lnx->ti_tsk_offset = 0;
#endif
	lnx->tsk_size = sizeof(struct task_struct);
	lnx->aa64 = IS_ENABLED(CONFIG_ARM64);
	lnx->lpae = IS_ENABLED(CONFIG_ARM_LPAE);
	lnx->text_paddr = _text_addr ? virt_to_phys((void *)_text_addr) : 0;
	lnx->pgd_paddr = init_mm ? virt_to_phys(init_mm->pgd) : 0;
	lnx->page_shift = PAGE_SHIFT;
	lnx->thread_size = THREAD_SIZE;
	msm_wdog_ctx_lnx_arch(lnx);
}

static void msm_wdog_ctx_header_init(struct msm_wdog_cpuctx *ctxi)
{
	struct msm_wdog_cpuctx_header *header = &ctxi->header[0];

	memset_io(header, 0, sizeof(*header));
	memcpy(header, mwc_header, sizeof(mwc_header));
}

static void msm_wdog_ctx_reset(struct msm_wdog_cpuctx *ctx, size_t ctx_size)
{
	struct msm_wdog_cpuctx *ctxi;
	int cpu;

	memset_io(ctx, 0, ctx_size);
	for_each_cpu(cpu, cpu_present_mask) {
		ctxi = &ctx[cpu];
		ctxi->info.sig = MSM_WDOG_CTX_SIG;
		ctxi->info.rev = MSM_WDOG_CTX_REV;
		ctxi->info.size = WDOG_CPUCTX_SIZE_PERCPU;
#if IS_ENABLED(CONFIG_KALLSYMS)
		ctxi->info.syms_avail = 1;
#else
		ctxi->info.syms_avail = 0;
#endif
		msm_wdog_ctx_header_init(ctxi);
		msm_wdog_ctx_lnx(&ctxi->lnx);

		if(!ctxi->lnx.text_paddr ||
			!ctxi->lnx.pgd_paddr ||
			(IS_ENABLED(CONFIG_ARM64) && !ctxi->lnx.kaslr_offset))
			ctxi->info.syms_avail = 0;
	}
}

#if defined(CONFIG_ARM64)
static inline int virt_is_valid(unsigned long addr)
{
	return (addr >= VA_START);
}
#else
static inline int virt_is_valid(unsigned long addr)
{
	return virt_addr_valid(addr);
}
#endif

static int msm_wdog_cpu_regs_version_unknown(uint32_t version)
{
	if (version != CPU_FORMAT_VERSION4)
		return 1;
	return 0;
}

static int msm_wdog_ctx_header_check(struct msm_wdog_cpuctx *ctxi)
{
	struct msm_wdog_cpuctx_header *header = &ctxi->header[0];
	return memcmp(header, mwc_header, sizeof(mwc_header));
}

static void msm_wdt_show_raw_mem(unsigned long addr, int nbytes,
			unsigned long old_addr, const char *label)
{
	int	i, j;
	int	nlines;
	unsigned long *p;

	MSMWDTD("%s: %#lx: ", label, old_addr);
	if (!virt_is_valid(old_addr)) {
		MSMWDTD("is not valid kernel address.\n");
		return;
	}
	MSMWDTD("\n");

	/*
	 * round address down to unsigned long aligned boundary
	 * and always dump a multiple of 32 bytes
	 */
	p = (unsigned long *)(addr & ~(sizeof(unsigned long) - 1));
	nbytes += (addr & (sizeof(unsigned long) - 1));
	nlines = (nbytes + 31) / 32;

	for (i = 0; i < nlines; i++) {
		/*
		 * just display low 16 bits of address to keep
		 * each line of the dump < 80 characters
		 */
		MSMWDTD("%04lx ", (unsigned long)old_addr & 0xffff);
		for (j = 0; j < (32 / sizeof(unsigned long)); j++) {
			if (sizeof(unsigned long) == 4)
				MSMWDTD(" %08lx", *p);
			else
				MSMWDTD(" %016lx", *p);
			++p;
			old_addr += sizeof(*p);
		}
		MSMWDTD("\n");
	}
}

static void msm_wdog_show_sc_status(uint32_t sc_status)
{
	if (sc_status & SC_STATUS_DBI)
		MSMWDTD("SDI: Secure watchdog bite. ");
	if (sc_status & SC_STATUS_CTX_BY_TZ)
		MSMWDTD("TZ: Non-secure watchdog bite. ");
	MSMWDTD("%sS ", (sc_status & SC_STATUS_NS) ?  "N" : " ");
	if (sc_status & SC_STATUS_WDT)
		MSMWDTD("WDT ");
	if (sc_status & SC_STATUS_SGI)
		MSMWDTD("SGI ");
	if (sc_status & SC_STATUS_WARM_BOOT)
		MSMWDTD("WARM_BOOT ");
	MSMWDTD("\n");
}

static void msm_wdt_show_thread_saved_pc(struct task_struct *p,
				struct thread_info *ti);
static void msm_wdt_show_task(struct task_struct *p, struct thread_info *ti)
{
	unsigned state;
	const char *stat_nam = TASK_STATE_TO_CHAR_STR;

	state = p->state ? __ffs(p->state) + 1 : 0;
	MSMWDTD("%-15.15s %c", p->comm,
		state < strlen(stat_nam) - 1 ? stat_nam[state] : '?');
	if (state == TASK_RUNNING)
		MSMWDTD(" running  ");
	else
		msm_wdt_show_thread_saved_pc(p, ti);
	MSMWDTD("pid %6d tgid %6d 0x%08lx\n", task_pid_nr(p), task_tgid_nr(p),
			(unsigned long)ti->flags);
}

#if defined(CONFIG_ARM64)
static void msm_wdt_show_thread_saved_pc(struct task_struct *p,
				struct thread_info *ti)
{
	MSMWDTD(" %016lx ", (unsigned long)p->thread.cpu_context.pc);
}

static int msm_wdt_unwind_frame_aa64(struct stackframe *frame,
				unsigned long stack,
				unsigned long *sp)
{
	unsigned long high, low;
	unsigned long fp = frame->fp;

	low  = *sp;
	high = ALIGN(low, THREAD_SIZE) - 0x20;

	if (fp < low || fp > high || fp & 0xf)
		return -EINVAL;

	*sp = (unsigned long)(fp + 0x10);
	frame->fp = (*(unsigned long *)(fp) & (THREAD_SIZE - 1)) + stack;
	frame->pc = *(unsigned long *)(fp + 8);

	return 0;
}

static void msm_wdt_show_regs(struct sysdbgCPUCtxtType *sysdbg_ctx)
{
	struct sysdbg_cpu64_ctxt_regs *regs = &sysdbg_ctx->cpu_regs.cpu64_ctxt;
	uint64_t *gp_reg;
	int i;

	MSMWDTD("PC is at %016llx\n", regs->pc);
	MSMWDTD("sp : %016llx ", regs->sp_el1);
	MSMWDTD("x30: %016llx\n", regs->x30);
	gp_reg = (uint64_t *)regs;
	for (i = 29; i >= 0; i--) {
		MSMWDTD("x%-2d: %016llx ", i, gp_reg[i]);
		if (i % 2 == 0)
			MSMWDTD("\n");
	}
	MSMWDTD("currentEL : %016llx\n", regs->currentEL);
	MSMWDTD("EL3: elr : %016llx spsr : %016llx sp : %016llx\n",
			regs->elr_el3, regs->spsr_el3, regs->sp_el3);
	MSMWDTD("EL2: elr : %016llx spsr : %016llx sp : %016llx\n",
			regs->elr_el2, regs->spsr_el2, regs->sp_el2);
	MSMWDTD("EL1: elr : %016llx spsr : %016llx sp : %016llx\n",
			regs->elr_el1, regs->spsr_el1, regs->sp_el1);
	MSMWDTD("EL0: sp : %016llx\n", regs->sp_el0);
}

static void msm_wdt_unwind(struct sysdbgCPUCtxtType *sysdbg_ctx,
			unsigned long addr, unsigned long kaslr_offset,
			unsigned long stack)
{
	struct stackframe frame;
	unsigned long sp;
	int offset;
	char sym_buf[KSYM_NAME_LEN];
	struct sysdbg_cpu64_ctxt_regs *regs = &sysdbg_ctx->cpu_regs.cpu64_ctxt;
	unsigned long cur_kaslr_offset = get_kaslr_offset();

	if (!virt_is_valid(addr)) {
		MSMWDTD("%016lx is not valid kernel address.\n", addr);
		return;
	}

	if ((regs->sp_el1 & ~(THREAD_SIZE - 1)) == addr) {
		frame.fp = (regs->x29 & (THREAD_SIZE - 1)) + stack;
		sp = (regs->sp_el1 & (THREAD_SIZE - 1)) + stack;
		frame.pc = regs->pc;
	}
	else {
		MSMWDTD("Unexpected stack address\n");
		return;
	}

	msm_wdt_show_raw_mem(stack, 96, addr, "thread_info");
	offset = (sp - stack - 128) & ~(128 - 1);
	if (offset < 0) {
		MSMWDT_ERR("Unexpected offset: %d, sp_el1: 0x%llx\n", offset, regs->sp_el1);
		return;
	}
	msm_wdt_show_raw_mem(stack + offset, THREAD_SIZE - offset,
			addr + offset, "stack");

	sprint_symbol(sym_buf,
		regs->pc - kaslr_offset + cur_kaslr_offset);
	MSMWDTD("PC: %s <%016llx>\n", sym_buf, regs->pc);
	sprint_symbol(sym_buf,
		regs->x30 - kaslr_offset + cur_kaslr_offset);
	MSMWDTD("LR: %s <%016llx>\n", sym_buf, regs->x30);

	while (1) {
		int urc;
		unsigned long where = frame.pc;

		/* Symbols are located at a random offset...subtract
		 * the old one and add the current one
		 */
		sprint_symbol(sym_buf,
			frame.pc - kaslr_offset + cur_kaslr_offset);
		MSMWDTD("[<%016lx>] %s\n", where, sym_buf);

		urc = msm_wdt_unwind_frame_aa64(&frame, stack, &sp);
		if (urc < 0)
			break;
	}
}

#elif defined(CONFIG_ARM)

static int __in_irqentry_text(unsigned long ptr)
{
	unsigned long _irqentry_text_start_addr = kallsyms_lookup_name("__irqentry_text_start");
	unsigned long __irqentry_text_end_addr = kallsyms_lookup_name("__irqentry_text_end");
	return ptr >= (unsigned long)&_irqentry_text_start_addr &&
	       ptr < (unsigned long)&__irqentry_text_end_addr;
}

static int in_exception_text(unsigned long ptr)
{
	unsigned long _exception_text_start_addr = kallsyms_lookup_name("__exception_text_start");
	unsigned long _exception_text_end_addr = kallsyms_lookup_name("__exception_text_end");
	int in;
	in = ptr >= (unsigned long)&_exception_text_start_addr &&
	     ptr < (unsigned long)&_exception_text_end_addr;

	return in ? : __in_irqentry_text(ptr);
}

static int msm_wdt_stackframe(struct stackframe *frame, void *data)
{
	unsigned long where = frame->pc;
	struct wdog_frame_struct * frame_data = (struct wdog_frame_struct *)data;

	MSMWDTD("[<%08lx>] (%pS) from [<%08lx>] (%pS)\n", where,
		(void *)where, frame->pc, (void *)frame->pc);
	if (in_exception_text(where)) {
		struct pt_regs *excep_regs =
				(struct pt_regs *)(frame->sp);
		if ((excep_regs->ARM_sp & ~(THREAD_SIZE - 1)) == frame_data->addr) {
			excep_regs->ARM_sp -= frame_data->addr;
			excep_regs->ARM_sp += frame_data->stack;
		}
	}
	return 0;
}

static void msm_wdt_show_thread_saved_pc(struct task_struct *p,
				struct thread_info *ti)
{
	MSMWDTD(" %08lx ", (unsigned long)ti->cpu_context.pc);
}

static void msm_wdt_show_regs(struct sysdbgCPUCtxtType *sysdbg_ctx)
{
	struct sysdbg_cpu32_ctxt_regs *regs = &sysdbg_ctx->cpu_regs.cpu32_ctxt;
	unsigned long _stext_addr = kallsyms_lookup_name("_stext");
	unsigned long _etext_addr = kallsyms_lookup_name("_etext");

	if ((regs->pc >= _stext_addr) &&
				(regs->pc <= _etext_addr))
		MSMWDTD("PC is at %pS <%08x>\n",
			(void *)(unsigned long)regs->pc, (uint32_t)regs->pc);
	else
		MSMWDTD("PC is at %08x\n", (uint32_t)regs->pc);
	MSMWDTD("\tr12: %08x  r11: %08x  r10: %08x  r9 : %08x  r8 : %08x\n",
			regs->r12, regs->r11, regs->r10, regs->r9, regs->r8);
	MSMWDTD("\tr7 : %08x  r6 : %08x  r5 : %08x  r4 : %08x\n",
			regs->r7, regs->r6, regs->r5, regs->r4);
	MSMWDTD("\tr3 : %08x  r2 : %08x  r1 : %08x  r0 : %08x\n",
			regs->r3, regs->r2, regs->r1, regs->r0);
	MSMWDTD("USR:\tr13: %08x  r14: %08x\n", regs->r13_usr, regs->r14_usr);
	MSMWDTD("HYP:\tr13: %08x  r14: %08x\n", regs->r13_hyp, regs->r14_hyp);
	MSMWDTD("IRQ:\tr13: %08x  r14: %08x\n", regs->r13_irq, regs->r14_irq);
	MSMWDTD("SVC:\tr13: %08x  r14: %08x\n", regs->r13_svc, regs->r14_svc);
	MSMWDTD("ABT:\tr13: %08x  r14: %08x\n", regs->r13_abt, regs->r14_abt);
	MSMWDTD("UND:\tr13: %08x  r14: %08x\n", regs->r13_und, regs->r14_und);
	MSMWDTD("MON:\tr13: %08x  r14: %08x\n", regs->r13_mon, regs->r14_mon);
	MSMWDTD("FIQ:\tr13: %08x  r14: %08x\n", regs->r13_fiq, regs->r14_fiq);
	MSMWDTD("\tr12: %08x  r11: %08x  r10: %08x  r9 : %08x  r8 : %08x\n",
			regs->r12_fiq, regs->r11_fiq, regs->r10_fiq,
			regs->r9_fiq, regs->r8_fiq);
}

static void msm_wdt_unwind(struct sysdbgCPUCtxtType *sysdbg_ctx,
			unsigned long addr, unsigned long kaslr_offset,
			unsigned long stack)
{
	struct stackframe frame;
	int offset;
	struct wdog_frame_struct frame_data;
	struct sysdbg_cpu32_ctxt_regs *regs = &sysdbg_ctx->cpu_regs.cpu32_ctxt;

	if (!virt_is_valid(addr)) {
		MSMWDTD("%08lx is not valid kernel address.\n", addr);
		return;
	}

	frame_data.addr = addr;
	frame_data.stack = stack;

	if ((regs->r13_svc & ~(THREAD_SIZE - 1)) == addr) {
		frame.fp = (regs->r11 & (THREAD_SIZE - 1)) + stack;
		frame.sp = (regs->r13_svc & (THREAD_SIZE - 1)) + stack;
		frame.lr = regs->r14_svc;
		frame.pc = regs->pc;
	} else {
		struct thread_info *ti = (struct thread_info *)stack;

		frame.fp = ti->cpu_context.fp - addr + stack;
		frame.sp = ti->cpu_context.sp - addr + stack;
		frame.lr = 0;
		frame.pc = ti->cpu_context.pc;
	}
	msm_wdt_show_raw_mem(stack, 96, addr, "thread_info");
	offset = (frame.sp - stack - 128) & ~(128 - 1);
	if (offset < 0) {
		MSMWDT_ERR("Unexpected offset: %d, r13_svc: 0x%x\n", offset, regs->r13_svc);
		return;
	}
	msm_wdt_show_raw_mem(stack + offset, THREAD_SIZE - offset,
			addr + offset, "stack");
	walk_stackframe(&frame, &msm_wdt_stackframe, &frame_data);
}
#endif

static void msm_wdog_ctx_print(struct msm_wdog_cpuctx *ctx,
				phys_addr_t paddr, size_t ctx_size)
{
	struct msm_wdog_cpuctx *ctxi;
	struct msm_dump_data *cpu_data;
	struct msm_wdog_cpuctx_info *info;
	struct msm_wdog_cpuctx_stat *stat;
	cpumask_t cpus, cpus_nodump, cpus_regs, cpus_dd;
	unsigned long stack_tmp = 0;
	int cpu;

	cpumask_clear(&cpus);
	for_each_cpu(cpu, cpu_present_mask) {
		ctxi = &ctx[cpu];
		cpu_data = &ctxi->cpu_data;
		if (msm_wdog_ctx_header_check(ctxi)) {
			MSMWDTD_IFWDOG("CPU%d: ctx header invalid\n", cpu);
			continue;
		}

		info = &ctxi->info;
		if ((info->sig != MSM_WDOG_CTX_SIG) ||
				(info->rev2 != MSM_WDOG_CTX_REV) ||
				(info->rev != MSM_WDOG_CTX_REV) ||
				(info->size != WDOG_CPUCTX_SIZE_PERCPU) ||
				(info->ret != ERR_NONE)) {
			MSMWDTD_IFWDOG("CPU%d: sig %x rev %x/%x sz %x ret %x\n",
					cpu, info->sig, (unsigned)info->rev,
					info->rev2, info->size, info->ret);
			continue;
		}
		cpumask_set_cpu(cpu, &cpus);
	}

	if (cpumask_empty(&cpus))
		return;

	cpumask_clear(&cpus_nodump);
	cpumask_clear(&cpus_regs);
	cpumask_clear(&cpus_dd);

	for_each_cpu(cpu, &cpus) {
		uint32_t *status;
		ctxi = &ctx[cpu];
		cpu_data = &ctxi->cpu_data;
		stat = &ctxi->stat;

		if (!cpu_data->magic && !cpu_data->version &&
				!ctxi->sysdbg.data.status[0]) {
			MSMWDTD_IFWDOG("CPU%d: No Dump!\n", cpu);
			cpumask_set_cpu(cpu, &cpus_nodump);
			continue;
		}
		if (cpu_data->magic != DUMP_MAGIC_NUMBER) {
			MSMWDTD_IFWDOG("CPU%d: dump magic mismatch %x/%x\n",
				cpu, cpu_data->magic, DUMP_MAGIC_NUMBER);
			continue;
		}
		if (msm_wdog_cpu_regs_version_unknown(cpu_data->version)) {
			MSMWDTD_IFWDOG("CPU%d: unknown version %d\n",
				cpu, cpu_data->version);
			continue;
		}
		cpumask_set_cpu(cpu, &cpus_regs);
		status = &ctxi->sysdbg.data.status[0];
		MSMWDTD("CPU%d: %x %x ", cpu, status[0], status[1]);
		msm_wdog_show_sc_status(status[1]);
		if (stat->ret == ERR_NONE || stat->ret == ERR_TASK_INVAL)
			cpumask_set_cpu(cpu, &cpus_dd);
	}

	if (cpumask_equal(&cpus_nodump, cpu_present_mask)) {
		MSMWDTD_IFWDOG("Might be Secure Watchdog Bite!\n");
		return;
	}
	if (cpumask_empty(&cpus_regs))
		return;
	MSMWDTD("\n");
	for_each_cpu(cpu, &cpus_regs) {
		struct msm_wdog_copy *job;

		ctxi = &ctx[cpu];
		stat = &ctxi->stat;
		MSMWDTD("CPU%d: ret %x", cpu, stat->ret);
		if (stat->stack_va) {
			MSMWDTD(" stack %lx ", (unsigned long)stat->stack_va);
			job = &stat->jobs[LNX_STACK];
			MSMWDTD("%lx -> %lx (%lx) ", (unsigned long)job->from,
					(unsigned long)job->to,
					(unsigned long)job->size);
			job = &stat->jobs[LNX_TASK];
			MSMWDTD("%lx -> %lx (%lx)", (unsigned long)job->from,
					(unsigned long)job->to,
					(unsigned long)job->size);
		}
		MSMWDTD("\n");
	}
	for_each_cpu(cpu, &cpus_regs) {
		ctxi = &ctx[cpu];
		MSMWDTD("\nCPU%d\n", cpu);
		msm_wdt_show_regs(&ctxi->sysdbg.data);
	}
	for_each_cpu(cpu, &cpus_dd) {
		unsigned long data;

		ctxi = &ctx[cpu];
		stat = &ctxi->stat;
		if (!IS_ALIGNED((unsigned long)ctxi->stack, THREAD_SIZE)
					&& !stack_tmp) {
			stack_tmp = __get_free_pages(GFP_KERNEL,
					THREAD_SIZE_ORDER);
			if (!stack_tmp)
				MSMWDT_ERR("Alloc temp stack failed.\n");
		}
		if (stack_tmp) {
			memcpy_fromio((void *)stack_tmp, ctxi->stack,
					THREAD_SIZE);
			data = stack_tmp;
		} else {
			data = (unsigned long)ctxi->stack;
		}

		MSMWDTD("\nCPU%d\n", cpu);
		if(stat->ret != ERR_TASK_INVAL) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,9,0) || !IS_ENABLED(CONFIG_THREAD_INFO_IN_TASK)
			msm_wdt_show_task(&ctxi->task,
				(struct thread_info *)ctxi->stack);
#else
			msm_wdt_show_task(&ctxi->task,
				&ctxi->task.thread_info);
#endif
		}
		msm_wdt_unwind(&ctxi->sysdbg.data, ctxi->stat.stack_va,
			ctx->lnx.kaslr_offset, data);
	}
	MSMWDTD("\n");
	if (stack_tmp)
		free_pages(stack_tmp, THREAD_SIZE_ORDER);
}

static void watchdog_cpu_ctx_table_register(struct device *dev,
	struct platform_data *pdata)
{
	int err = 0;
	struct msm_dump_entry dump_entry;
	struct msm_dump_data *ctx_dump_data;

	ctx_dump_data = kzalloc(sizeof(struct msm_dump_data),
					GFP_KERNEL);

	if (!ctx_dump_data) {
		dev_err(dev, "Cannot alloc dump data structure.\n");
		goto err;
	}

	ctx_dump_data->addr = pdata->mem_address;
	ctx_dump_data->len = pdata->mem_size;
	dump_entry.id = MSM_DUMP_DATA_WDOG_CPU_CTX;
	dump_entry.addr = virt_to_phys(ctx_dump_data);
	err = msm_dump_data_register(MSM_DUMP_TABLE_APPS, &dump_entry);
	if (err) {
		dev_err(dev, "Registering dump data failed.\n");
		kfree(ctx_dump_data);
	}
err:
	return;
}

static int watchdog_cpu_ctx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_data *pdata;
	struct resource res;
	struct device_node *node;
	struct msm_wdog_cpuctx *ctx_vaddr;
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

	dev_info(dev, "size %zx", pdata->mem_size);
	dev_info(dev, "addr %lx", (unsigned long)pdata->mem_address);
	dev_info(dev, "size per cpu is %lx", (unsigned long)WDOG_CPUCTX_SIZE_PERCPU);

	if (pdata->mem_size < WDOG_CPUCTX_SIZE) {
		dev_err(dev, "Mem size too small %zx/%lu\n",
				pdata->mem_size, (unsigned long)WDOG_CPUCTX_SIZE);
		err = -ENOMEM;
		goto err;
	}

	watchdog_cpu_ctx_table_register(dev, pdata);

	ctx_vaddr = dma_remap(dev, NULL, pdata->mem_address,
					pdata->mem_size, 0);
	if (!ctx_vaddr) {
		dev_err(dev, "Cannot remap buffer %pa size %zx\n",
				&pdata->mem_address, pdata->mem_size);
		err = -ENOMEM;
		goto err;
	}

	msm_wdog_ctx_print(ctx_vaddr, pdata->mem_address, pdata->mem_size);
	msm_wdog_ctx_reset(ctx_vaddr, pdata->mem_size);

err:
	return err;
}

static int watchdog_cpu_ctx_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id watchdog_cpu_ctx_match[] = {
	{ .compatible = "mmi,watchdog_cpu_ctx" },
	{}
};

static struct platform_driver watchdog_cpu_ctx_driver = {
	.probe		= watchdog_cpu_ctx_probe,
	.remove		= watchdog_cpu_ctx_remove,
	.driver		= {
		.name = "watchdog_cpu_ctx",
		.of_match_table = watchdog_cpu_ctx_match,
	},
};

static int watchdog_cpu_ctx_init(void)
{
	return platform_driver_register(&watchdog_cpu_ctx_driver);
}

static void watchdog_cpu_ctx_exit(void)
{
	platform_driver_unregister(&watchdog_cpu_ctx_driver);
}

module_init(watchdog_cpu_ctx_init);
module_exit(watchdog_cpu_ctx_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Watchdog cpu ctx");
