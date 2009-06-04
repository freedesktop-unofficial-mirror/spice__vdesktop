/*
 * Memory merging driver for Linux
 *
 * This module enables dynamic sharing of identical pages found in different
 * memory areas, even if they are not shared by fork()
 *
 * Copyright (C) 2008 Red Hat, Inc.
 * Authors:
 *	Izik Eidus
 *	Andrea Arcangeli
 *	Chris Wright
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/file.h>
#include <linux/mman.h>
#include <linux/sched.h>
#include <linux/rwsem.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/rmap.h>
#include <linux/spinlock.h>
#include <linux/jhash.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/scatterlist.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/rbtree.h>
#include <linux/anon_inodes.h>

#include <asm/tlbflush.h>

#include "ksm.h"
#include "wp_notifier.h"
#include "external-module-compat.h"

#define KSM_MINOR 234

MODULE_AUTHOR("Red Hat, Inc.");
MODULE_LICENSE("GPL");

static int rmap_hash_size;
module_param(rmap_hash_size, int, 0);
MODULE_PARM_DESC(rmap_hash_size, "Hash table size for the reverse mapping");

/*
 * ksm_mem_slot - hold information for an userspace scanning range
 * (the scanning for this region will be from addr untill addr +
 *  npages * PAGE_SIZE inside mm)
 */
struct ksm_mem_slot {
	struct list_head link;
	struct list_head sma_link;
	struct mm_struct *mm;
	unsigned long addr;	/* the begining of the virtual address */
	int npages;		/* number of pages to share */
};

/*
 * ksm_sma - shared memory area, each process have its own sma that contain the
 * information about the slots that it own
 */
struct ksm_sma {
	struct list_head sma_slots;
};

/**
 * struct ksm_scan - cursor for scanning
 * @slot_index: the current slot we are scanning
 * @page_index: the page inside the sma that is currently being scanned
 *
 * ksm uses it to know what are the next pages it need to scan
 */
struct ksm_scan {
	struct ksm_mem_slot *slot_index;
	unsigned long page_index;
};

/*
 * Few notes about ksm scanning progress (make it easier to understand the
 * structures below):
 *
 * In order to reduce excessive scanning, pages are sorted into the hash
 * table, page_hash.  After a page is inserted into the hash table, its
 * contents may have changed.  In this case, ksm must remove the page from
 * the hash table and potentially rehash it.  Ksm uses a reverse mapping,
 * rmap_hash, to efficiently manage this.
 */

struct rmap_item;

/*
 * tree_item - object of the write protected pages tree
 */
struct tree_item {
	struct rb_node node;
	struct rmap_item *rmap_item;
};

/*
 * rmap_item - object of the rmap_hash hash table
 * (it is holding the previous hash value (oldindex),
 *  pointer into the page_hash_item, and pointer into the tree_item)
 */
struct rmap_item {
	struct hlist_node link;
	struct mm_struct *mm;
	unsigned long address;
	unsigned int oldchecksum; /* old checksum value */
	unsigned char stable_tree; // 1 stable_tree 0 unstable tree
	struct tree_item *tree_item;
	struct rmap_item *next;
	struct rmap_item *prev;
};

/*
 * slots is linked list that hold all the memory regions that were registred
 * to be scanned.
 */
static LIST_HEAD(slots);
static DECLARE_RWSEM(slots_lock);

struct rb_root root_stable_tree = RB_ROOT;
struct rb_root root_unstable_tree = RB_ROOT;

static int nrmaps_hash;
/* rmap_hash hash table */
static struct hlist_head *rmap_hash;

static struct kmem_cache *tree_item_cache;
static struct kmem_cache *rmap_item_cache;

static int kthread_sleep; /* sleep time of the kernel thread */
static int kthread_pages_to_scan; /* npages to scan for the kernel thread */
static struct ksm_scan kthread_ksm_scan;
static int ksmd_flags;
static struct task_struct *kthread;
static DECLARE_WAIT_QUEUE_HEAD(kthread_wait);
static DECLARE_RWSEM(kthread_lock);

static int ksm_slab_init(void)
{
	int ret = -ENOMEM;

	tree_item_cache = KMEM_CACHE(tree_item, 0);
	if (!tree_item_cache)
		goto out;

	rmap_item_cache = KMEM_CACHE(rmap_item, 0);
	if (!rmap_item_cache)
		goto out_free;

	return 0;

out_free:
	kmem_cache_destroy(tree_item_cache);
out:
	return ret;
}

static void ksm_slab_free(void)
{
	kmem_cache_destroy(rmap_item_cache);
	kmem_cache_destroy(tree_item_cache);
}

static inline struct tree_item *alloc_tree_item(void)
{
	return kmem_cache_zalloc(tree_item_cache, GFP_KERNEL);
}

static void free_tree_item(struct tree_item *tree_item)
{
	kmem_cache_free(tree_item_cache, tree_item);
}

static inline struct rmap_item *alloc_rmap_item(void)
{
	return kmem_cache_zalloc(rmap_item_cache, GFP_KERNEL);
}

static inline void free_rmap_item(struct rmap_item *rmap_item)
{
	kmem_cache_free(rmap_item_cache, rmap_item);
}

/*
 * PageKsm - this type of pages are the write protected pages that ksm map
 * into multiple vmas (this is the "shared page")
 * this page was allocated using alloc_page(), every pte that pointing to it
 * is always write protected (therefore its data content cant ever be changed)
 * and this page cant be swapped.
 */
static inline int PageKsm(struct page *page)
{
	return !PageAnon(page);
}

static int rmap_hash_init(void)
{
	if (!rmap_hash_size) {
		struct sysinfo sinfo;

		si_meminfo(&sinfo);
		rmap_hash_size = sinfo.totalram / 10;
	}
	nrmaps_hash = rmap_hash_size;
	rmap_hash = vmalloc(nrmaps_hash * sizeof(struct hlist_head));
	if (!rmap_hash)
		return -ENOMEM;
	memset(rmap_hash, 0, nrmaps_hash * sizeof(struct hlist_head));
	return 0;
}

static void rmap_hash_free(void)
{
	int i;
	struct hlist_head *bucket;
	struct hlist_node *node, *n;
	struct rmap_item *rmap_item;

	for (i = 0; i < nrmaps_hash; ++i) {
		bucket = &rmap_hash[i];
		hlist_for_each_entry_safe(rmap_item, node, n, bucket, link) {
			hlist_del(&rmap_item->link);
			free_rmap_item(rmap_item);
		}
	}
	vfree(rmap_hash);
}

static inline u32 calc_checksum(struct page *page)
{
	u32 checksum;
	void *addr = kmap_atomic(page, KM_USER0);
	checksum = jhash2(addr, PAGE_SIZE / 4, 17);
	kunmap_atomic(addr, KM_USER0);
	return checksum;
}

static struct rmap_item *get_rmap_item(struct mm_struct *mm, unsigned long addr)
{
	struct rmap_item *rmap_item;
	struct hlist_head *bucket;
	struct hlist_node *node;

	bucket = &rmap_hash[addr % nrmaps_hash];
	hlist_for_each_entry(rmap_item, node, bucket, link) {
		if (mm == rmap_item->mm && rmap_item->address == addr) {
			return rmap_item;
		}
	}
	return NULL;
}

static void remove_rmap_item_from_tree(struct rmap_item *rmap_item)
{
	struct tree_item *tree_item;

	tree_item = rmap_item->tree_item;
	rmap_item->tree_item = NULL;

	if (rmap_item->stable_tree) {
		if (rmap_item->prev) {
			BUG_ON(rmap_item->prev->next != rmap_item);
			rmap_item->prev->next = rmap_item->next;
		}
		if (rmap_item->next) {
			BUG_ON(rmap_item->next->prev != rmap_item);
			rmap_item->next->prev = rmap_item->prev;
		}
	}

	if (tree_item) {
		if (rmap_item->stable_tree) {
	 		if (!rmap_item->next && !rmap_item->prev) {
				rb_erase(&tree_item->node, &root_stable_tree);
				free_tree_item(tree_item);
			} else if (!rmap_item->prev) {
				BUG_ON(tree_item->rmap_item != rmap_item);
				tree_item->rmap_item = rmap_item->next;
			} else
				BUG_ON(tree_item->rmap_item == rmap_item);
		} else if (!rmap_item->stable_tree)
			free_tree_item(tree_item);
	}

	hlist_del(&rmap_item->link);
	free_rmap_item(rmap_item);
}

static void remove_page_from_tree(struct mm_struct *mm,
				  unsigned long addr)
{
	struct rmap_item *rmap_item;

	rmap_item = get_rmap_item(mm, addr);
	if (!rmap_item)
		return;
	remove_rmap_item_from_tree(rmap_item);
	return;
}

static int ksm_sma_ioctl_register_memory_region(struct ksm_sma *ksm_sma,
						struct ksm_memory_region *mem)
{
	struct ksm_mem_slot *slot;
	int ret = -EPERM;

	slot = kzalloc(sizeof(struct ksm_mem_slot), GFP_KERNEL);
	if (!slot) {
		ret = -ENOMEM;
		goto out;
	}

	slot->mm = get_task_mm(current);
	if (!slot->mm)
		goto out_free;
	slot->addr = mem->addr;
	slot->npages = mem->npages;

	down_write(&slots_lock);

	list_add_tail(&slot->link, &slots);
	list_add_tail(&slot->sma_link, &ksm_sma->sma_slots);

	up_write(&slots_lock);
	return 0;

out_free:
	kfree(slot);
out:
	return ret;
}

static void remove_mm_from_hash_and_tree(struct mm_struct *mm)
{
	struct ksm_mem_slot *slot;
	int pages_count;

	list_for_each_entry(slot, &slots, link)
		if (slot->mm == mm)
			break;
	BUG_ON(!slot);

	root_unstable_tree = RB_ROOT;
	for (pages_count = 0; pages_count < slot->npages; ++pages_count)
		remove_page_from_tree(mm, slot->addr +
				      pages_count * PAGE_SIZE);
	list_del(&slot->link);
}

static int ksm_sma_ioctl_remove_memory_region(struct ksm_sma *ksm_sma)
{
	struct ksm_mem_slot *slot, *node;

	down_write(&slots_lock);
	list_for_each_entry_safe(slot, node, &ksm_sma->sma_slots, sma_link) {
		remove_mm_from_hash_and_tree(slot->mm);
		mmput(slot->mm);
		list_del(&slot->sma_link);
		kfree(slot);
	}
	up_write(&slots_lock);
	return 0;
}

static int ksm_sma_release(struct inode *inode, struct file *filp)
{
	struct ksm_sma *ksm_sma = filp->private_data;
	int r;

	r = ksm_sma_ioctl_remove_memory_region(ksm_sma);
	kfree(ksm_sma);
	return r;
}

static long ksm_sma_ioctl(struct file *filp,
			  unsigned int ioctl, unsigned long arg)
{
	struct ksm_sma *sma = filp->private_data;
	void __user *argp = (void __user *)arg;
	int r = EINVAL;

	switch (ioctl) {
	case KSM_REGISTER_MEMORY_REGION: {
		struct ksm_memory_region ksm_memory_region;

		r = -EFAULT;
		if (copy_from_user(&ksm_memory_region, argp,
				   sizeof(ksm_memory_region)))
			goto out;
		r = ksm_sma_ioctl_register_memory_region(sma,
							 &ksm_memory_region);
		break;
	}
	case KSM_REMOVE_MEMORY_REGION:
		r = ksm_sma_ioctl_remove_memory_region(sma);
		break;
	}

out:
	return r;
}

static unsigned long addr_in_vma(struct vm_area_struct *vma, struct page *page)
{
	pgoff_t pgoff = page->index << (PAGE_CACHE_SHIFT - PAGE_SHIFT);
	unsigned long addr;

	addr = vma->vm_start + ((pgoff - vma->vm_pgoff) << PAGE_SHIFT);
	if (unlikely(addr < vma->vm_start || addr >= vma->vm_end))
		return -EFAULT;
	return addr;
}

static pte_t *get_pte(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep = NULL;

	pgd = pgd_offset(mm, addr);
	if (!pgd_present(*pgd))
		goto out;

	pud = pud_offset(pgd, addr);
	if (!pud_present(*pud))
		goto out;

	pmd = pmd_offset(pud, addr);
	if (!pmd_present(*pmd))
		goto out;

	ptep = pte_offset_map(pmd, addr);
out:
	return ptep;
}

static int is_present_pte(struct mm_struct *mm, unsigned long addr)
{
	pte_t *ptep;
	int r;

	ptep = get_pte(mm, addr);
	if (!ptep)
		return 0;

	r = pte_present(*ptep);
	pte_unmap(ptep);

	return r;
}

static int memcmp_pages(struct page *page1, struct page *page2)
{
	char *addr1, *addr2;
	int r;

	addr1 = kmap_atomic(page1, KM_USER0);
	addr2 = kmap_atomic(page2, KM_USER1);
	r = memcmp(addr1, addr2, PAGE_SIZE);
	kunmap_atomic(addr1, KM_USER0);
	kunmap_atomic(addr2, KM_USER1);
	return r;
}

/* pages_identical
 * return 1 if identical, 0 otherwise.
 */
static inline int pages_identical(struct page *page1, struct page *page2)
{
	return !memcmp_pages(page1, page2);
}

/*
 * try_to_merge_one_page - take two pages and merge them into one
 * note:
 * oldpage should be anon page while newpage should be file mapped page
 *
 * this function return 0 if the pages were merged, 1 otherwise.
 */
static int try_to_merge_one_page(struct mm_struct *mm,
				 struct vm_area_struct *vma,
				 struct page *oldpage,
				 struct page *newpage,
				 pgprot_t newprot)
{
	int ret = 1;
	int odirect_sync;
	unsigned long page_addr_in_vma;
	pte_t orig_pte, *orig_ptep;

	if (!PageAnon(oldpage))
		goto out;

	get_page(newpage);
	get_page(oldpage);

	page_addr_in_vma = addr_in_vma(vma, oldpage);
	if (page_addr_in_vma == -EFAULT)
		goto out_putpage;

	orig_ptep = get_pte(mm, page_addr_in_vma);
	if (!orig_ptep)
		goto out_putpage;
	orig_pte = *orig_ptep;
	pte_unmap(orig_ptep);
	if (!pte_present(orig_pte))
		goto out_putpage;
	if (page_to_pfn(oldpage) != pte_pfn(orig_pte))
		goto out_putpage;
	/*
	 * we need the page lock to read a stable PageSwapCache in
	 * page_wrprotect()
	 */
	if (TestSetPageLocked(oldpage))
		goto out_putpage;
	/*
	 * page_wrprotect check if the page is swapped or in swap cache,
	 * in the future we might want to run here if_present_pte and then
	 * swap_free
	 */
	if (!page_wrprotect(oldpage, &odirect_sync, 2)) {
		unlock_page(oldpage);
		goto out_putpage;
	}
	unlock_page(oldpage);
	if (!odirect_sync)
		goto out_putpage;

	orig_pte = pte_wrprotect(orig_pte);

	if (pages_identical(oldpage, newpage))
		ret = replace_page(vma, oldpage, newpage, orig_pte, newprot);

out_putpage:
	put_page(oldpage);
	put_page(newpage);
out:
	return ret;
}

/*
 * try_to_merge_two_pages_alloc - take two identical pages and prepare them
 * to be merged into one page.
 *
 * this function return 0 if we successfully mapped two identical pages into one
 * page, 1 otherwise.
 * (note this function will allocate a new kernel page, if one of the pages
 * is already shared page (KsmPage), then try_to_merge_two_pages_noalloc()
 * should be called.)
 */

static int try_to_merge_two_pages_alloc(struct mm_struct *mm1,
					struct page *page1,
					struct mm_struct *mm2,
					struct page *page2,
					unsigned long addr1,
					unsigned long addr2)
{
	struct vm_area_struct *vma;
	pgprot_t prot;
	int ret = 1;
	struct page *kpage;


	kpage = alloc_page(GFP_HIGHUSER);
	if (!kpage)
		return ret;
	down_read(&mm1->mmap_sem);
	vma = find_vma(mm1, addr1);
	if (!vma) {
		put_page(kpage);
		up_read(&mm1->mmap_sem);
		return ret;
	}
	prot = vma->vm_page_prot;
	pgprot_val(prot) &= ~_PAGE_RW;

	copy_user_highpage(kpage, page1, addr1);
	ret = try_to_merge_one_page(mm1, vma, page1, kpage, prot);
	up_read(&mm1->mmap_sem);

	if (!ret) {
		down_read(&mm2->mmap_sem);
		vma = find_vma(mm2, addr2);
		if (!vma) {
			put_page(kpage);
			ret = 1;
			up_read(&mm2->mmap_sem);
			return ret;
		}

		prot = vma->vm_page_prot;
		pgprot_val(prot) &= ~_PAGE_RW;

		ret = try_to_merge_one_page(mm2, vma, page2, kpage,
					    prot);
		up_read(&mm2->mmap_sem);
		/*
		 * If the secoend try_to_merge_one_page call was failed,
		 * we are in situation where we have Ksm page that have
		 * just one pte pointing to it, in this case we break
		 * it.
		 */
		if (ret) {
			struct page *tmppage[1];
			down_read(&mm1->mmap_sem);
			if (get_user_pages(current, mm1, addr1, 1, 1, 0,
					   tmppage, NULL) == 1) {
				put_page(tmppage[0]);
			}
			up_read(&mm1->mmap_sem);
		}
	}

	put_page(kpage);
	return ret;
}

/*
 * try_to_merge_two_pages_noalloc - the same astry_to_merge_two_pages_alloc,
 * but no new kernel page is allocated (page2 should be KsmPage)
 */
static int try_to_merge_two_pages_noalloc(struct mm_struct *mm1,
					  struct page *page1,
					  struct page *page2,
					  unsigned long addr1)
{
	struct vm_area_struct *vma;
	pgprot_t prot;
	int ret = 1;

	/*
	 * If page2 is shared, we can just make the pte of mm1(page1) point to
	 * page2.
	 */
	BUG_ON(!PageKsm(page2));
	down_read(&mm1->mmap_sem);
	vma = find_vma(mm1, addr1);
	if (!vma) {
		up_read(&mm1->mmap_sem);
		return ret;
	}
	prot = vma->vm_page_prot;
	pgprot_val(prot) &= ~_PAGE_RW;
	ret = try_to_merge_one_page(mm1, vma, page1, page2, prot);
	up_read(&mm1->mmap_sem);

	return ret;
}


static int is_zapped_item(struct rmap_item *rmap_item,
			  struct page **page)
{
	int ret = 0;
	struct vm_area_struct *vma;

	cond_resched();
	down_read(&rmap_item->mm->mmap_sem);
	if (is_present_pte(rmap_item->mm, rmap_item->address)) {
		vma = find_vma(rmap_item->mm, rmap_item->address);
		if (vma && !vma->vm_file) {
			BUG_ON(vma->vm_flags & VM_SHARED);
			ret = get_user_pages(current, rmap_item->mm,
					     rmap_item->address,
					     1, 0, 0, page, NULL);
		}
	}
	up_read(&rmap_item->mm->mmap_sem);

	if (ret != 1)
		return 1;

	if (unlikely(!PageKsm(page[0]))) {
		put_page(page[0]);
		return 1;
	}
	return 0;
}

static struct rmap_item *stable_tree_search(struct page *page,
					    struct page **page2,
					    struct rmap_item *rmap_item)
{
	struct rb_node *node = root_stable_tree.rb_node;
	struct tree_item *tree_item;
	struct rmap_item *found_rmap_item, *next_rmap_item;

	while (node) {
		int ret;

		tree_item = rb_entry(node, struct tree_item, node);
		found_rmap_item = tree_item->rmap_item;
		while (found_rmap_item) {
			BUG_ON(!found_rmap_item->stable_tree);
			BUG_ON(!found_rmap_item->tree_item);
			if (!rmap_item ||
			     !(found_rmap_item->mm == rmap_item->mm &&
			      found_rmap_item->address == rmap_item->address)) {
				if (!is_zapped_item(found_rmap_item, page2))
					break;
				next_rmap_item = found_rmap_item->next;
				remove_rmap_item_from_tree(found_rmap_item);
				found_rmap_item = next_rmap_item;
			} else
				found_rmap_item = found_rmap_item->next;
		}
		if (!found_rmap_item)
			goto out_didnt_find;

		/*
		 * We can trust the value of the memcmp as we know the pages
		 * are write protected.
		 */
		ret = memcmp_pages(page, page2[0]);

		if (ret < 0) {
			put_page(page2[0]);
			node = node->rb_left;
		}
		else if (ret > 0) {
			put_page(page2[0]);
			node = node->rb_right;
		}
		else
			goto out_found;
	}
out_didnt_find:
	found_rmap_item = NULL;
out_found:
	return found_rmap_item;
}

static int stable_tree_insert(struct page *page,
			      struct tree_item *new_tree_item,
			      struct rmap_item *rmap_item)
{
	struct rb_node **new = &(root_stable_tree.rb_node);
	struct rb_node *parent = NULL;
	struct tree_item *tree_item;
	struct page *page2[1];

	while (*new) {
		int ret;
		struct rmap_item *insert_rmap_item, *next_rmap_item;

		tree_item = rb_entry(*new, struct tree_item, node);
		BUG_ON(!tree_item);
		BUG_ON(!tree_item->rmap_item);

		insert_rmap_item = tree_item->rmap_item;
		while (insert_rmap_item) {
			BUG_ON(!insert_rmap_item->stable_tree);
			BUG_ON(!insert_rmap_item->tree_item);
			if (!rmap_item ||
			    !(insert_rmap_item->mm == rmap_item->mm &&
			     insert_rmap_item->address == rmap_item->address)) {
				if (!is_zapped_item(insert_rmap_item, page2))
					break;
				next_rmap_item = insert_rmap_item->next;
				remove_rmap_item_from_tree(insert_rmap_item);
				insert_rmap_item = next_rmap_item;
			} else
				insert_rmap_item = insert_rmap_item->next;
		}
		if (!insert_rmap_item)
			return 1;

		ret = memcmp_pages(page, page2[0]);

		parent = *new;
		if (ret < 0) {
			put_page(page2[0]);
			new = &((*new)->rb_left);
		}
		else if (ret > 0) {
			put_page(page2[0]);
			new = &((*new)->rb_right);
		}
		else {
			/*
			 * It isnt a bug when we are here,
			 * beacuse after we release the stable_tree_lock
			 * someone else could have merge identical page to the
			 * tree.
			 */
			return 1;
		}
	}

	rb_link_node(&new_tree_item->node, parent, new);
	rb_insert_color(&new_tree_item->node, &root_stable_tree);
	rmap_item->stable_tree = 1;
	rmap_item->tree_item = new_tree_item;

	return 0;
}

static struct tree_item *unstable_tree_search_insert(struct page *page,
					struct page **page2,
					struct rmap_item *page_rmap_item)
{
	struct rb_node **new = &(root_unstable_tree.rb_node);
	struct rb_node *parent = NULL;
	struct tree_item *tree_item;
	struct tree_item *new_tree_item;
	struct rmap_item *rmap_item;
	unsigned int checksum;

	while (*new) {
		int ret;

		tree_item = rb_entry(*new, struct tree_item, node);
		BUG_ON(!tree_item);
		rmap_item = tree_item->rmap_item;
		BUG_ON(!rmap_item);

		down_read(&rmap_item->mm->mmap_sem);
		/*
		 * We dont want to swap in pages
		 */
		if (!is_present_pte(rmap_item->mm, rmap_item->address)) {
			up_read(&rmap_item->mm->mmap_sem);
			return NULL;
		}

		ret = get_user_pages(current, rmap_item->mm, rmap_item->address,
				     1, 0, 0, page2, NULL);
		up_read(&rmap_item->mm->mmap_sem);
		if (ret != 1)
			return NULL;

		ret = memcmp_pages(page, page2[0]);

		parent = *new;
		if (ret < 0) {
			put_page(page2[0]);
			new = &((*new)->rb_left);
		}
		else if (ret > 0) {
			put_page(page2[0]);
			new = &((*new)->rb_right);
		} else
			return tree_item;
	}

	if (!page_rmap_item)
		return NULL;

	checksum = calc_checksum(page);
	if (page_rmap_item->oldchecksum != checksum) {
		page_rmap_item->oldchecksum = checksum;
		return NULL;
	}

	new_tree_item = alloc_tree_item();
	if (!new_tree_item)
		return NULL;

	page_rmap_item->tree_item = new_tree_item;
	page_rmap_item->stable_tree = 0;
	new_tree_item->rmap_item = page_rmap_item;
	rb_link_node(&new_tree_item->node, parent, new);
	rb_insert_color(&new_tree_item->node, &root_unstable_tree);

	return NULL;
}

/*
 * update_stable_tree - check if the page inside the tree got zapped,
 * and if it got zapped, kick it from the tree.
 */
int update_tree(struct rmap_item *rmap_item, int *wait)
{
	if (!rmap_item->stable_tree) {
		/*
		 * If the rmap_item is !stable_tree and in addition
		 * it have tree_item != NULL, it mean this rmap_item
		 * was inside the unstable tree, therefore we have to free
		 * the tree_item from it (beacuse the unstable tree was already
		 * flushed by the time we are here).
		 */
		if (rmap_item->tree_item) {
			free_tree_item(rmap_item->tree_item);
			rmap_item->tree_item = NULL;
			return 0;
		}
		return 0;
	}

	/* If we are here it mean the rmap_item was zapped, beacuse the
	 * rmap_item was pointing into the stable_tree and there all the pages
	 * should be KsmPages, so it shouldnt have came to here in the first
	 * place. (cmp_and_merge_page() shouldnt have been called)
	 */
	remove_rmap_item_from_tree(rmap_item);
	*wait = 1;
	return 1;
}

static struct rmap_item *create_new_rmap_item(struct mm_struct *mm,
			 		      unsigned long addr,
					      unsigned int checksum)
{
	struct rmap_item *rmap_item;
	struct hlist_head *bucket;

	rmap_item = alloc_rmap_item();
	if (!rmap_item)
		return NULL;

	rmap_item->mm = mm;
	rmap_item->address = addr;
	rmap_item->oldchecksum = checksum;
	rmap_item->stable_tree = 0;
	rmap_item->tree_item = NULL;

	bucket = &rmap_hash[addr % nrmaps_hash];
	hlist_add_head(&rmap_item->link, bucket);

	return rmap_item;
}

/*
 * cmp_and_merge_page - take a page computes its hash value and check if there
 * is similar hash value to different page,
 * in case we find that there is similar hash to different page we call to
 * try_to_merge_two_pages().
 */
static int cmp_and_merge_page(struct ksm_scan *ksm_scan, struct page *page)
{
	struct page *page2[1];
	struct ksm_mem_slot *slot;
	struct tree_item *tree_item;
	struct rmap_item *rmap_item;
	struct rmap_item *tree_rmap_item;
	unsigned int checksum;
	unsigned long addr;
	int wait = 0;

	slot = ksm_scan->slot_index;
	addr = slot->addr + ksm_scan->page_index * PAGE_SIZE;
	rmap_item = get_rmap_item(slot->mm, addr);
	if (rmap_item) {
		if (update_tree(rmap_item, &wait))
			rmap_item = NULL;
	}

	tree_rmap_item = stable_tree_search(page, page2, rmap_item);
	if (tree_rmap_item) {
		int ret;

		BUG_ON(!tree_rmap_item->tree_item);
		ret = try_to_merge_two_pages_noalloc(slot->mm, page, page2[0],
						     addr);
		put_page(page2[0]);
		if (!ret) {
			if (!rmap_item)
				rmap_item = create_new_rmap_item(slot->mm,
								 addr, 0);
			if (!rmap_item)
				return !ret;


			rmap_item->next = tree_rmap_item->next;
			rmap_item->prev = tree_rmap_item;

			if (tree_rmap_item->next)
				tree_rmap_item->next->prev = rmap_item;

			tree_rmap_item->next = rmap_item;

			rmap_item->stable_tree = 1;
			rmap_item->tree_item = tree_rmap_item->tree_item;
		}
		return !ret;
	}

	tree_item = unstable_tree_search_insert(page, page2, rmap_item);
	if (tree_item) {
		int ret;
		struct rmap_item *tmp_rmap_item;

		tmp_rmap_item = tree_item->rmap_item;
		BUG_ON(!tmp_rmap_item);
		ret = try_to_merge_two_pages_alloc(slot->mm, page,
						   tmp_rmap_item->mm,
						   page2[0], addr,
						   tmp_rmap_item->address);
		if (!ret) {
			rb_erase(&tree_item->node, &root_unstable_tree);
			if (!stable_tree_insert(page2[0],
						tree_item, tmp_rmap_item)) {
				if (rmap_item) {
					rmap_item->stable_tree = 1;
					rmap_item->next = tmp_rmap_item->next;
					rmap_item->prev = tmp_rmap_item;
					if (tmp_rmap_item->next)
						tmp_rmap_item->next->prev =
								      rmap_item;
					tmp_rmap_item->next = rmap_item;
					rmap_item->tree_item =
						       tmp_rmap_item->tree_item;
				}
			}
		}
		put_page(page2[0]);
		return !ret;
	}
	if (!wait && !rmap_item) {
		checksum = calc_checksum(page);
		create_new_rmap_item(slot->mm, addr, checksum);
		return 0;
	}
	return 0;
}

/* return -EAGAIN - no slots registered, nothing to be done */
static int scan_get_next_index(struct ksm_scan *ksm_scan, int nscan)
{
	struct ksm_mem_slot *slot;

	if (list_empty(&slots))
		return -EAGAIN;

	slot = ksm_scan->slot_index;

	/* Are there pages left in this slot to scan? */
	if ((slot->npages - ksm_scan->page_index - nscan) > 0) {
		ksm_scan->page_index += nscan;
		return 0;
	}

	list_for_each_entry_from(slot, &slots, link) {
		if (slot == ksm_scan->slot_index)
			continue;
		ksm_scan->page_index = 0;
		ksm_scan->slot_index = slot;
		return 0;
	}

	/* look like we finished scanning the whole memory, starting again */
	root_unstable_tree = RB_ROOT;
	ksm_scan->page_index = 0;
	ksm_scan->slot_index = list_first_entry(&slots,
						struct ksm_mem_slot, link);
	return 0;
}

/*
 * update slot_index - make sure ksm_scan will point to vaild data,
 * it is possible that by the time we are here the data that ksm_scan was
 * pointed to was released so we have to call this function every time after
 * taking the slots_lock
 */
static void scan_update_old_index(struct ksm_scan *ksm_scan)
{
	struct ksm_mem_slot *slot;

	if (list_empty(&slots))
		return;

	list_for_each_entry(slot, &slots, link) {
		if (ksm_scan->slot_index == slot)
			return;
	}

	ksm_scan->slot_index = list_first_entry(&slots,
						struct ksm_mem_slot, link);
	ksm_scan->page_index = 0;
}

/**
 * ksm_scan_start - the ksm scanner main worker function.
 * @ksm_scan -    the scanner.
 * @scan_npages - number of pages we are want to scan before we return from this
 * @function.
 *
 * (this function can be called from the kernel thread scanner, or from 
 *  userspace ioctl context scanner)
 *
 *  The function return -EAGAIN in case there are not slots to scan.
 */
static int ksm_scan_start(struct ksm_scan *ksm_scan, int scan_npages)
{
	struct ksm_mem_slot *slot;
	struct page *page[1];
	int val;
	int ret = 0;

	down_read(&slots_lock);

	scan_update_old_index(ksm_scan);

	while (scan_npages > 0) {
		ret = scan_get_next_index(ksm_scan, 1);
		if (ret)
			goto out;

		slot = ksm_scan->slot_index;

		cond_resched();

		/*
		 * If the page is swapped out or in swap cache, we don't want to
		 * scan it (it is just for performance).
		 */
		down_read(&slot->mm->mmap_sem);
		if (is_present_pte(slot->mm, slot->addr +
				   ksm_scan->page_index * PAGE_SIZE)) {
			val = get_user_pages(current, slot->mm, slot->addr +
					     ksm_scan->page_index * PAGE_SIZE ,
					      1, 0, 0, page, NULL);
			up_read(&slot->mm->mmap_sem);
			if (val == 1) {
				if (!PageKsm(page[0]))
					cmp_and_merge_page(ksm_scan, page[0]);
				put_page(page[0]);
			}
		} else {
			up_read(&slot->mm->mmap_sem);
		}
		scan_npages--;
	}
	scan_get_next_index(ksm_scan, 1);
out:
	up_read(&slots_lock);
	return ret;
}

/*
 * no multithreaded ksm for ovirt
 */
/*static int ksm_scan_ioctl_start(struct ksm_scan *ksm_scan,
				struct ksm_user_scan *scan)
{
	if (!(scan->flags & ksm_control_flags_run))
		return 0;

	return ksm_scan_start(ksm_scan, scan->pages_to_scan);
}*/

static int ksm_scan_release(struct inode *inode, struct file *filp)
{
	struct ksm_scan *ksm_scan = filp->private_data;

	kfree(ksm_scan);
	return 0;
}

static long ksm_scan_ioctl(struct file *filp,
			   unsigned int ioctl, unsigned long arg)
{
//	struct ksm_scan *ksm_scan = filp->private_data;
	//void __user *argp = (void __user *)arg;
	int r = EINVAL;

	switch (ioctl) {
	/*
	 * i didnt implemented the locking yet, and in ovirt we dont run
	 * multi-threaded ksm.
	 */
	/*case KSM_SCAN: {
		struct ksm_user_scan scan;

		r = -EFAULT;
		if (copy_from_user(&scan, argp,
				   sizeof(struct ksm_user_scan)))
			break;

		r = ksm_scan_ioctl_start(ksm_scan, &scan);
	}*/
	}
	return r;
}

static struct file_operations ksm_sma_fops = {
	.release        = ksm_sma_release,
	.unlocked_ioctl = ksm_sma_ioctl,
	.compat_ioctl   = ksm_sma_ioctl,
};

static int ksm_dev_ioctl_create_shared_memory_area(void)
{
	int fd = -1;
	struct ksm_sma *ksm_sma;

	ksm_sma = kmalloc(sizeof(struct ksm_sma), GFP_KERNEL);
	if (!ksm_sma)
		goto out;

	INIT_LIST_HEAD(&ksm_sma->sma_slots);

	fd = anon_inode_getfd("ksm-sma", &ksm_sma_fops, ksm_sma, 0);
	if (fd < 0)
		goto out_free;

	return fd;
out_free:
	kfree(ksm_sma);
out:
	return fd;
}

static struct file_operations ksm_scan_fops = {
	.release        = ksm_scan_release,
	.unlocked_ioctl = ksm_scan_ioctl,
	.compat_ioctl   = ksm_scan_ioctl,
};

static struct ksm_scan *ksm_scan_create(void)
{
	return kzalloc(sizeof(struct ksm_scan), GFP_KERNEL);
}

static int ksm_dev_ioctl_create_scan(void)
{
	int fd = -ENOMEM;
	struct ksm_scan *ksm_scan;

	ksm_scan = ksm_scan_create();
	if (!ksm_scan)
		goto out;

	fd = anon_inode_getfd("ksm-scan", &ksm_scan_fops, ksm_scan, 0);
	if (fd < 0)
		goto out_free;
	return fd;

out_free:
	kfree(ksm_scan);
out:
	return fd;
}

/*
 * ksm_dev_ioctl_start_stop_kthread - control the kernel thread scanning running
 * speed.
 * This function allow us to control on the time the kernel thread will sleep
 * how many pages it will scan between sleep and sleep, and how many pages it
 * will maximum merge between sleep and sleep.
 */
static int ksm_dev_ioctl_start_stop_kthread(struct ksm_kthread_info *info)
{
	int rc = 0;

	down_write(&kthread_lock);

	if (info->flags & ksm_control_flags_run) {
		if (!info->pages_to_scan) {
			rc = EPERM;
			up_write(&kthread_lock);
			goto out;
		}
	}

	kthread_sleep = info->sleep;
	kthread_pages_to_scan = info->pages_to_scan;
	ksmd_flags = info->flags;

	up_write(&kthread_lock);

	if (ksmd_flags & ksm_control_flags_run)
		wake_up_interruptible(&kthread_wait);

out:
	return rc;
}

/*
 * ksm_dev_ioctl_get_info_kthread - write into info the scanning information
 * of the ksm kernel thread
 */
static void ksm_dev_ioctl_get_info_kthread(struct ksm_kthread_info *info)
{
	down_read(&kthread_lock);

	info->sleep = kthread_sleep;
	info->pages_to_scan = kthread_pages_to_scan;
	info->flags = ksmd_flags;

	up_read(&kthread_lock);
}

static long ksm_dev_ioctl(struct file *filp,
			  unsigned int ioctl, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long r = -EINVAL;

	switch (ioctl) {
	case KSM_GET_API_VERSION:
		r = KSM_API_VERSION;
		break;
	case KSM_CREATE_SHARED_MEMORY_AREA:
		r = ksm_dev_ioctl_create_shared_memory_area();
		break;
	case KSM_CREATE_SCAN:
		r = ksm_dev_ioctl_create_scan();
		break;
	case KSM_START_STOP_KTHREAD: {
		struct ksm_kthread_info info;

		r = -EFAULT;
		if (copy_from_user(&info, argp,
				   sizeof(struct ksm_kthread_info)))
			break;

		r = ksm_dev_ioctl_start_stop_kthread(&info);
		break;
		}
	case KSM_GET_INFO_KTHREAD: {
		struct ksm_kthread_info info;

		ksm_dev_ioctl_get_info_kthread(&info);
		r = -EFAULT;
		if (copy_to_user(argp, &info,
				 sizeof(struct ksm_kthread_info)))
			break;
		r = 0;
		break;
	}
	default:
		break;
	}
	return r;
}

static struct file_operations ksm_chardev_ops = {
	.unlocked_ioctl = ksm_dev_ioctl,
	.compat_ioctl   = ksm_dev_ioctl,
	.owner          = THIS_MODULE,
};

static struct miscdevice ksm_dev = {
	KSM_MINOR,
	"ksm",
	&ksm_chardev_ops,
};

int kthread_ksm_scan_thread(void *nothing)
{
	while (!kthread_should_stop()) {
		if (ksmd_flags & ksm_control_flags_run) {
			down_read(&kthread_lock);
			ksm_scan_start(&kthread_ksm_scan,
				       kthread_pages_to_scan);
			up_read(&kthread_lock);
			schedule_timeout_interruptible(
					usecs_to_jiffies(kthread_sleep));
		} else
			wait_event_interruptible(kthread_wait,
					ksmd_flags & ksm_control_flags_run ||
					kthread_should_stop());
	}
	return 0;
}

static int __init ksm_init(void)
{
	int r;

	r = ksm_slab_init();
	if (r)
		goto out;

	r = rmap_hash_init();
	if (r)
		goto out_free1;

	kthread = kthread_run(kthread_ksm_scan_thread, NULL, "kksmd");
	if (IS_ERR(kthread)) {
		printk(KERN_ERR "ksm: creating kthread failed\n");
		r = PTR_ERR(kthread);
		goto out_free2;
	}

	r = init_wp_notifier();
	if (r)
		goto out_free3;

	r = misc_register(&ksm_dev);
	if (r) {
		printk(KERN_ERR "ksm: misc device register failed\n");
		goto out_free4;
	}

	printk(KERN_WARNING "ksm loaded\n");
	return 0;

out_free4:
	exit_wp_notifier();
out_free3:
	kthread_stop(kthread);
out_free2:
	rmap_hash_free();
out_free1:
	ksm_slab_free();
out:
	return r;
}

static void __exit ksm_exit(void)
{
	misc_deregister(&ksm_dev);
	exit_wp_notifier();
	ksmd_flags = ksm_control_flags_run;
	kthread_stop(kthread);
	rmap_hash_free();
	ksm_slab_free();
}

module_init(ksm_init)
module_exit(ksm_exit)
