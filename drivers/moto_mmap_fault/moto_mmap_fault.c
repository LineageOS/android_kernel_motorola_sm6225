/*
 * Copyright (C) 2023 Motorola Mobility LLC
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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>       // struct mm_struct, apply_to_page_range()
#include <linux/kallsyms.h> // kallsyms_lookup_name()
#include <linux/vmalloc.h>  // vm_unmap_aliases()
#include <trace/hooks/mm.h>
#include <linux/kprobes.h>
#include <linux/version.h>

typedef struct page *(*page_cache_get_page_funct_t)(struct address_space *mapping, pgoff_t offset, int fgp_flags, gfp_t gfp_mask);
static page_cache_get_page_funct_t page_cache_get_page_func;
typedef unsigned long (*kallsyms_lookup_name_func_t)(const char *name);
static kallsyms_lookup_name_func_t kallsyms_lookup_name_func;
static int max_ra_pages = 8;
module_param(max_ra_pages, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_ra_pages, "Max read ahead pages");
static void __nocfi filemap_fault_get_page(void *p, struct vm_fault *vmf, struct page **page_out, bool *retry)
{
    struct file *file = vmf->vma->vm_file;
    pgoff_t offset = vmf->pgoff;
    struct page *page = NULL;
    struct file_ra_state *ra = NULL;
    struct address_space *mapping = NULL;
    unsigned int mmap_miss;
    unsigned int old_ra_pages = 0;

    if (!file)
        return;

    ra = &file->f_ra;
    if(!ra)
        return;

    mapping = file->f_mapping;
    if (page_cache_get_page_func)
        page = page_cache_get_page_func(mapping, offset, 0, 0);

    if (likely(page) && !(vmf->flags & FAULT_FLAG_TRIED)) {
        put_page(page);
    } else if (!page) {
        mmap_miss = READ_ONCE(ra->mmap_miss);
        if ((vmf->vma->vm_flags & VM_RAND_READ) ||
            (!ra->ra_pages) ||
            (vmf->vma->vm_flags & VM_SEQ_READ) ||
            mmap_miss > 100) {
            return;
        } else {
            old_ra_pages = ra->ra_pages;
            if (ra->ra_pages > max_ra_pages) {
                ra->ra_pages = max_ra_pages; // reduce the read ahead limit to 8 pages
                vmf->android_oem_data1[0] = old_ra_pages;
                vmf->android_oem_data1[1] = max_ra_pages;
            }
            return;
        }
    } else {
        put_page(page);
    }

    return;
}

static void __nocfi filemap_fault_cache_page(void *p, struct vm_fault *vmf, struct page *page)
{
    struct file *file = vmf->vma->vm_file;
    struct file_ra_state *ra = NULL;

    if (!file)
        return;

    ra = &file->f_ra;
    if(!ra)
        return;

    if ((ra->ra_pages == max_ra_pages) && (vmf->android_oem_data1[0] != 0) && (vmf->android_oem_data1[1] != 0)) {
        ra->ra_pages = (unsigned long)vmf->android_oem_data1[0]; //restore the old ra_pages
        vmf->android_oem_data1[0] = 0;
        vmf->android_oem_data1[1] = 0;
    }
}

static int __nocfi __init moto_mmap_fault_init(void)
{
    int ret = 0;
    struct kprobe kp = {
        .symbol_name = "kallsyms_lookup_name",
    };

    ret = register_kprobe(&kp);
    if (ret < 0) {
        pr_err("Failed to register kprobe, %d\n", ret);
        return ret;
    }
    unregister_kprobe(&kp);
    kallsyms_lookup_name_func = (kallsyms_lookup_name_func_t)kp.addr;
    if (!kallsyms_lookup_name_func) {
        pr_err("Failed to get kallsyms_lookup_name address\n");
        return -ENXIO;
    }

    page_cache_get_page_func = (void *)kallsyms_lookup_name_func("pagecache_get_page");
    if (!page_cache_get_page_func) {
        pr_err("Failed to get required function\n");
        return -ENXIO;
    }
    ret = register_trace_android_vh_filemap_fault_get_page(filemap_fault_get_page, NULL) ?:
     register_trace_android_vh_filemap_fault_cache_page(filemap_fault_cache_page, NULL);

    if (ret != 0)
        return -ENXIO;
    else
        return 0;
}
static void __nocfi __exit moto_mmap_fault_exit(void)
{
    unregister_trace_android_vh_filemap_fault_get_page(filemap_fault_get_page, NULL);
    unregister_trace_android_vh_filemap_fault_cache_page(filemap_fault_cache_page, NULL);
}

module_init(moto_mmap_fault_init);
module_exit(moto_mmap_fault_exit);
MODULE_DESCRIPTION("Motorola vendor mmap fault driver");
MODULE_LICENSE("GPL v2");
