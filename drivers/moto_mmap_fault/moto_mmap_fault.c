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
#include <trace/hooks/mm.h>
#include <linux/pagemap.h>

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
	page = find_get_page(mapping, offset);

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

	if ((ra->ra_pages == max_ra_pages) && (vmf->android_oem_data1[0] != 0)
			&& (vmf->android_oem_data1[1] == max_ra_pages)) {
			ra->ra_pages = (unsigned int)vmf->android_oem_data1[0]; //restore the old ra_pages
			vmf->android_oem_data1[0] = 0;
			vmf->android_oem_data1[1] = 0;
	}
}

static int __nocfi __init moto_mmap_fault_init(void)
{
	int ret = 0;
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
