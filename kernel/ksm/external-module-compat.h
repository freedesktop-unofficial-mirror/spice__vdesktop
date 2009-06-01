
/*
 * Compatibility header for building as an external module.
 */

/*
 * Avoid picking up the kernel's kvm.h in case we have a newer one.
 */

#include <linux/compiler.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/cpu.h>
#include <linux/list.h>
#include <asm/processor.h>
#include <linux/hrtimer.h>
#include <asm/bitops.h>
#include <linux/mm.h>
#include <linux/rmap.h>
#include <asm/tlbflush.h>
#include <linux/module.h>
#include <asm/cacheflush.h>
#include <asm-generic/pgtable.h>

/*
 * 2.6.16 does not have GFP_NOWAIT
 */

#include <linux/gfp.h>

void kvm_ksm_set_pte(struct mm_struct *mm, unsigned long address, pte_t pte);
int kvm_ksm_spte_count(struct mm_struct *mm,
		       unsigned long address);

#define list_first_entry(ptr, type, member) \
        list_entry((ptr)->next, type, member)

static struct anon_vma *page_lock_anon_vma(struct page *page)
{
	struct anon_vma *anon_vma;
	unsigned long anon_mapping;

	rcu_read_lock();
	anon_mapping = (unsigned long) page->mapping;
	if (!(anon_mapping & PAGE_MAPPING_ANON))
		goto out;
	if (!page_mapped(page))
		goto out;

	anon_vma = (struct anon_vma *) (anon_mapping - PAGE_MAPPING_ANON);
	spin_lock(&anon_vma->lock);
	return anon_vma;
out:
	rcu_read_unlock();
	return NULL;
}

static void page_unlock_anon_vma(struct anon_vma *anon_vma)
{
	spin_unlock(&anon_vma->lock);
	rcu_read_unlock();
}

/*
 * At what user virtual address is page expected in @vma?
 * Returns virtual address or -EFAULT if page's index/offset is not
 * within the range mapped the @vma.
 */
static inline unsigned long
vma_address(struct page *page, struct vm_area_struct *vma)
{
	pgoff_t pgoff = page->index;
	unsigned long address;

	address = vma->vm_start + ((pgoff - vma->vm_pgoff) << PAGE_SHIFT);
	if (unlikely(address < vma->vm_start || address >= vma->vm_end)) {
		/* page should be within @vma mapping range */
		return -EFAULT;
	}
	return address;
}

/*
 * At what user virtual address is page expected in vma? checking that the
 * page matches the vma: currently only used on anon pages, by unuse_vma;
 */
unsigned long page_address_in_vma(struct page *page, struct vm_area_struct *vma)
{
	if (PageAnon(page)) {
		if ((void *)vma->anon_vma !=
		    (void *)page->mapping - PAGE_MAPPING_ANON)
			return -EFAULT;
	} else if (page->mapping && !(vma->vm_flags & VM_NONLINEAR)) {
		if (!vma->vm_file ||
		    vma->vm_file->f_mapping != page->mapping)
			return -EFAULT;
	} else
		return -EFAULT;
	return vma_address(page, vma);
}

/*
 * Check that @page is mapped at @address into @mm.
 *
 * On success returns with pte mapped and locked.
 */
pte_t *page_check_address(struct page *page, struct mm_struct *mm,
			  unsigned long address, spinlock_t **ptlp)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	spinlock_t *ptl;

	pgd = pgd_offset(mm, address);
	if (!pgd_present(*pgd))
		return NULL;

	pud = pud_offset(pgd, address);
	if (!pud_present(*pud))
		return NULL;

	pmd = pmd_offset(pud, address);
	if (!pmd_present(*pmd))
		return NULL;

	pte = pte_offset_map(pmd, address);
	/* Make a quick check before getting the lock */
	if (!pte_present(*pte)) {
		pte_unmap(pte);
		return NULL;
	}

	ptl = pte_lockptr(mm, pmd);
	spin_lock(ptl);
	if (pte_present(*pte) && page_to_pfn(page) == pte_pfn(*pte)) {
		*ptlp = ptl;
		return pte;
	}
	pte_unmap_unlock(pte, ptl);
	return NULL;
}

void page_remove_rmap_old(struct page *page, struct vm_area_struct *vma)
{
	if (atomic_add_negative(-1, &page->_mapcount)) {
		if (unlikely(page_mapcount(page) < 0)) {
			printk (KERN_EMERG "Eeek! page_mapcount(page) went negative! (%d)\n", page_mapcount(page));
			printk (KERN_EMERG "  page pfn = %lx\n", page_to_pfn(page));
			printk (KERN_EMERG "  page->flags = %lx\n", page->flags);
			printk (KERN_EMERG "  page->count = %x\n", page_count(page));
			printk (KERN_EMERG "  page->mapping = %p\n", page->mapping);
			BUG();
		}

		/*
		 * It would be tidy to reset the PageAnon mapping here,
		 * but that might overwrite a racing page_add_anon_rmap
		 * which increments mapcount after us but sets mapping
		 * before us: so leave the reset to free_hot_cold_page,
		 * and remember that it's only reliable while mapped.
		 * Leaving it set also helps swapoff to reinstate ptes
		 * faster for those pages still in swapcache.
		 */
		__dec_zone_page_state(page,
				PageAnon(page) ? NR_ANON_PAGES : NR_FILE_MAPPED);
	}
}

void page_add_file_rmap_old(struct page *page)
{
	if (atomic_inc_and_test(&page->_mapcount))
		__inc_zone_page_state(page, NR_FILE_MAPPED);
}

static int page_wrprotect_one(struct page *page, struct vm_area_struct *vma,
			      int *odirect_sync, int count_offset)
{
	struct mm_struct *mm = vma->vm_mm;
	unsigned long address;
	pte_t *pte;
	spinlock_t *ptl;
	int ret = 0;

	address = vma_address(page, vma);
	if (address == -EFAULT)
		goto out;

	pte = page_check_address(page, mm, address, &ptl);
	if (!pte)
		goto out;

	if (pte_write(*pte)) {
		pte_t entry;


		flush_cache_page(vma, address, pte_pfn(*pte));
		/*
		 * Ok, so after ptep_clear_flush will get called the pte will
		 * be not present, so gup-fast will become gup-slow and will
		 * block on the pte_lock, now, the fact that ptep_clear_flush
		 * will notify all the cpu, is a way to sync it with knowing
		 * that by the time it return gup-fast is not running in the
		 * middle, beacuse gup-fast run with irq_disabled.
		 */
		entry = ptep_clear_flush(vma, address, pte);

		/*
		 * this is needed here to balance the mapcount of the page
		 */
		count_offset += kvm_ksm_spte_count(mm, address);
		
		/*
		 * Check that no O_DIRECT or similar I/O is in progress on the
		 * page
		 */
		if ((page_mapcount(page) + count_offset) != page_count(page)) {
			*odirect_sync = 0;
			set_pte_at(mm, address, pte, entry);
			goto out_unlock;
		}

		entry = pte_wrprotect(entry);
		set_pte_at(mm, address, pte, entry);
		BUG_ON(pte_write(entry));
		kvm_ksm_set_pte(mm, address, entry);
	}
	ret = 1;

out_unlock:
	pte_unmap_unlock(pte, ptl);
out:
	return ret;
}

static int page_wrprotect_anon(struct page *page, int *odirect_sync,
			       int count_offset)
{
	struct vm_area_struct *vma;
	struct anon_vma *anon_vma;
	int ret = 0;

	anon_vma = page_lock_anon_vma(page);
	if (!anon_vma)
		return ret;

	/*
	 * If the page is inside the swap cache, its _count number was
	 * increased by one, therefore we have to increase count_offset by one.
	 */
	if (PageSwapCache(page))
		count_offset++;

	list_for_each_entry(vma, &anon_vma->head, anon_vma_node)
		ret += page_wrprotect_one(page, vma, odirect_sync,
					  count_offset);

	page_unlock_anon_vma(anon_vma);

	return ret;
}

/**
 * page_wrprotect - set all ptes pointing to a page as readonly
 * @page:         the page to set as readonly
 * @odirect_sync: boolean value that is set to 0 when some of the ptes were not
 *                marked as readonly beacuse page_wrprotect_one() was not able
 *                to mark this ptes as readonly without opening window to a race
 *                with odirect
 * @count_offset: number of times page_wrprotect() caller had called get_page()
 *                on the page
 *
 * returns the number of ptes which were marked as readonly.
 * (ptes that were readonly before this function was called are counted as well)
 */
int page_wrprotect(struct page *page, int *odirect_sync, int count_offset)
{
	int ret = 0;

	/*
	 * Page lock is needed for anon pages for the PageSwapCache check,
	 * and for page_mapping for filebacked pages
	 */
	BUG_ON(!PageLocked(page));

	*odirect_sync = 1;
	if (PageAnon(page))
		ret = page_wrprotect_anon(page, odirect_sync, count_offset);

	return ret;
}

/**
 * replace_page - replace page in vma with new page
 * @vma:      vma that hold the pte oldpage is pointed by.
 * @oldpage:  the page we are replacing with newpage
 * @newpage:  the page we replace oldpage with
 * @orig_pte: the original value of the pte
 * @prot: page protection bits
 *
 * Returns 0 on success, -EFAULT on failure.
 *
 * Note: @newpage must not be an anonymous page because replace_page() does
 * not change the mapping of @newpage to have the same values as @oldpage.
 * @newpage can be mapped in several vmas at different offsets (page->index).
 */
int replace_page(struct vm_area_struct *vma, struct page *oldpage,
		 struct page *newpage, pte_t orig_pte, pgprot_t prot)
{
	struct mm_struct *mm = vma->vm_mm;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep;
	pte_t new_pte;
	spinlock_t *ptl;
	unsigned long addr;
	int ret;

	BUG_ON(PageAnon(newpage));

	ret = -EFAULT;
	addr = page_address_in_vma(oldpage, vma);
	if (addr == -EFAULT)
		goto out;

	pgd = pgd_offset(mm, addr);
	if (!pgd_present(*pgd))
		goto out;

	pud = pud_offset(pgd, addr);
	if (!pud_present(*pud))
		goto out;

	pmd = pmd_offset(pud, addr);
	if (!pmd_present(*pmd))
		goto out;

	ptep = pte_offset_map_lock(mm, pmd, addr, &ptl);
	if (!ptep)
		goto out;

	if (!pte_same(*ptep, orig_pte)) {
		pte_unmap_unlock(ptep, ptl);
		goto out;
	}

	ret = 0;
	get_page(newpage);
	page_add_file_rmap_old(newpage);

	flush_cache_page(vma, addr, pte_pfn(*ptep));
	ptep_clear_flush(vma, addr, ptep);
	new_pte = mk_pte(newpage, prot);
	set_pte_at(mm, addr, ptep, new_pte);
	update_mmu_cache(vma, addr, new_pte);
	BUG_ON(pte_write(new_pte));
	kvm_ksm_set_pte(mm, addr, new_pte);

	page_remove_rmap_old(oldpage, vma);
	if (PageAnon(oldpage)) {
		dec_mm_counter(mm, anon_rss);
		inc_mm_counter(mm, file_rss);
	}
	put_page(oldpage);

	pte_unmap_unlock(ptep, ptl);
out:
	return ret;
}


#include <linux/smp.h>

/* HRTIMER_MODE_ABS started life with a different name */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#define HRTIMER_MODE_ABS HRTIMER_ABS
#endif

/* __mmdrop() is not exported before 2.6.25 */
#include <linux/sched.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)

#define mmdrop(x) do { (void)(x); } while (0)

#endif
