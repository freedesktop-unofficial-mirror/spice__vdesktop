/*
 * Copyright (C) 2008-2009 Red Hat, Inc.
 * Authors:
 *	Andrea Arcangeli
 *	Izik Eidus
 */

//#define KPROBES_ENABLE_SWAP
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/kallsyms.h>
#ifdef KPROBES_ENABLE_SWAP
#include <linux/pagemap.h>
#include <linux/rmap.h>
#endif
#include "wp_notifier.h"

static int pre_do_wp_page(struct kprobe *p,
                         struct pt_regs *regs)
{
       struct mm_struct *mm;
       unsigned long address;

       /*
        * kprobes runs with irq disabled and preempt disabled but we
        * need irq enabled to flush the smp tlb with IPIs while
        * tearing down sptes.
        */
       local_irq_enable();

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
       mm = (struct mm_struct *) regs->rdi;
       address = (unsigned long) regs->rdx;
#else
       mm = (struct mm_struct *) regs->di;
       address = (unsigned long) regs->dx;
#endif
       kvm_wp_notifier(mm, address);

       local_irq_disable();

       return 0;
}

#ifdef KPROBES_ENABLE_SWAP

static unsigned long vma_address(struct page *page, struct vm_area_struct *vma)
{
	pgoff_t pgoff = page->index << (PAGE_CACHE_SHIFT - PAGE_SHIFT);
	unsigned long address;

	address = vma->vm_start + ((pgoff - vma->vm_pgoff) << PAGE_SHIFT);
	if (unlikely(address < vma->vm_start || address >= vma->vm_end)) {
		/* page should be within @vma mapping range */
		return -EFAULT;
	}
	return address;
}

static struct anon_vma *__page_get_anon_vma(struct page *page)
{
	struct anon_vma *anon_vma;
	unsigned long anon_mapping;

	anon_mapping = (unsigned long) page->mapping;
	if (!(anon_mapping & PAGE_MAPPING_ANON))
		goto out;
	if (!page_mapped(page))
		goto out;

	anon_vma = (struct anon_vma *) (anon_mapping - PAGE_MAPPING_ANON);
	return anon_vma;
out:
	return NULL;
}

static int pre_page_remove_rmap(struct kprobe *p, struct pt_regs *regs)
{
	struct mm_struct *mm;
	unsigned long address;
	struct page *page;
	struct vm_area_struct *vma;
	struct anon_vma *anon_vma;

	/*
	 * kprobes runs with irq disabled and preempt disabled but we
	 * need irq enabled to flush the smp tlb with IPIs while
	 * tearing down sptes.
	 */
	local_irq_enable();

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
	page = (struct page *) regs->rdi;
#else
	page = (struct page *) regs->di;
#endif
	/*
	 * We care just about try_to_unmap here (other cases for ksm wo_wp_page
	 * notifier will handle) therefore page must be lock. (make life easier
	 * for filebacked mapping as well)
	 */
	if (!PageLocked(page))
		goto out;

	/*
	 * Ok, every caller for page_rmap_remove that hold page lock, hold
	 * anom_vma->lock as well, so no need to take it.
	 */
	if (PageAnon(page)) {
		anon_vma = __page_get_anon_vma(page);
		if (!anon_vma)
			goto out;

		list_for_each_entry(vma, &anon_vma->head, anon_vma_node) {
			mm = vma->vm_mm;
			if (!(vma->vm_flags & VM_NONLINEAR)) {
				address = vma_address(page, vma);
				if (address != -EFAULT)
					kvm_wp_notifier(mm, address);
			}
		}
        }
out:
	local_irq_disable();

	return 0;
}

static struct kprobe swap_not_kprobe;
#endif

static struct kprobe wp_not_kprobe;

int init_wp_notifier(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) && defined(CONFIG_KALLSYMS) && !defined(RHEL_RELEASE_CODE)
       wp_not_kprobe.addr = (kprobe_opcode_t *)kallsyms_lookup_name("do_wp_page");
       if (!wp_not_kprobe.addr) {
               printk(KERN_WARNING "do_wp_page not found");
               return 1;
       }
#else
       wp_not_kprobe.symbol_name = "do_wp_page";
#endif
       wp_not_kprobe.pre_handler = pre_do_wp_page;

       if (register_kprobe(&wp_not_kprobe)) {
               printk(KERN_WARNING "cant register kprobe for do_wp_page");
               return 1;
       }

#ifdef KPROBES_ENABLE_SWAP
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) && defined(CONFIG_KALLSYMS) && !defined(RHEL_RELEASE_CODE)
       swap_not_kprobe.addr = (kprobe_opcode_t *)kallsyms_lookup_name("page_remove_rmap");
       if (!swap_not_kprobe.addr) {
               printk(KERN_WARNING "page_remove_rmap not found");
               return 1;
       }
#else
       swap_not_kprobe.symbol_name = "page_remove_rmap";
#endif
       swap_not_kprobe.pre_handler = pre_page_remove_rmap;

       if (register_kprobe(&swap_not_kprobe)) {
               printk(KERN_WARNING "cant register kprobe for do_wp_page");
               return 1;
       }
#endif

       return 0;
}

void exit_wp_notifier(void)
{
       unregister_kprobe(&wp_not_kprobe);
#ifdef KPROBES_ENABLE_SWAP
       unregister_kprobe(&swap_not_kprobe);
#endif
}
