/*
 *  linux/drivers/video/fb_defio.c
 *
 *  Copyright (C) 2006 Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/rmap.h>
#include <linux/pagemap.h>
#include <linux/bitops.h>

static struct page *fb_deferred_io_page(struct fb_info *info, unsigned long offs)
{
	void *screen_base = (void __force *) info->screen_base;
	struct page *page;

	if (is_vmalloc_addr(screen_base + offs))
		page = vmalloc_to_page(screen_base + offs);
	else
		page = pfn_to_page((info->fix.smem_start + offs) >> PAGE_SHIFT);

	return page;
}

/* This is to find and return the fb pages */
static int fb_deferred_io_fault(struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	unsigned long offset;
	struct page *page;
	struct fb_info *info = vma->vm_private_data;

	offset = vmf->pgoff << PAGE_SHIFT;
	if (offset >= info->fix.smem_len)
		return VM_FAULT_SIGBUS;

	page = fb_deferred_io_page(info, offset);
	if (!page)
		return VM_FAULT_SIGBUS;

	get_page(page);

	if (vma->vm_file)
		page->mapping = vma->vm_file->f_mapping;
	else
		printk(KERN_ERR "no mapping available\n");

	BUG_ON(!page->mapping);
	page->index = vmf->pgoff;

	vmf->page = page;
	return 0;
}

int fb_deferred_io_fsync(struct file *file, struct dentry *dentry, int datasync)
{
	struct fb_info *info = file->private_data;

	/* Skip if deferred io is complied-in but disabled on this fbdev */
	if (!info->fbdefio)
		return 0;

	/* Kill off the delayed work */
	cancel_rearming_delayed_work(&info->deferred_work);

	/* Run it immediately */
	return schedule_delayed_work(&info->deferred_work, 0);
}
EXPORT_SYMBOL_GPL(fb_deferred_io_fsync);

/*
 * Set page to mark it dirty in our pagemap
 */
static void fb_defio_set_pagemap(struct fb_deferred_io *fbdefio,
					struct page *page)
{
	/* protect against the workqueue changing the pagemap */
	mutex_lock(&fbdefio->lock);
	set_bit(page->index, fbdefio->pagemap);
	mutex_unlock(&fbdefio->lock);
}

/* vm_ops->page_mkwrite handler */
static int fb_deferred_io_mkwrite(struct vm_area_struct *vma,
				  struct vm_fault *vmf)
{
	struct page *page = vmf->page;
	struct fb_info *info = vma->vm_private_data;
	struct fb_deferred_io *fbdefio = info->fbdefio;

	/*
	 * This is a callback we get when userspace first tries to
	 * write to the page. We schedule a workqueue. That workqueue
	 * will eventually mkclean the touched pages and execute the
	 * deferred framebuffer IO. Then, if userspace touches a page
	 * again, we repeat the same scheme
	 */

	fb_defio_set_pagemap(fbdefio, page);

	/* need this page to be protected until it is marked dirty */
	lock_page(page);

	/* Come back after delay to process the deferred IO */
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);

	return VM_FAULT_LOCKED;
}

static struct vm_operations_struct fb_deferred_io_vm_ops = {
	.fault		= fb_deferred_io_fault,
	.page_mkwrite	= fb_deferred_io_mkwrite,
};

static int fb_deferred_io_set_page_dirty(struct page *page)
{
	if (!PageDirty(page))
		SetPageDirty(page);
	return 0;
}

static const struct address_space_operations fb_deferred_io_aops = {
	.set_page_dirty = fb_deferred_io_set_page_dirty,
};

static int fb_deferred_io_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_ops = &fb_deferred_io_vm_ops;
	vma->vm_flags |= ( VM_IO | VM_RESERVED | VM_DONTEXPAND );
	vma->vm_private_data = info;
	return 0;
}

/* workqueue callback */
static void fb_deferred_io_work(struct work_struct *work)
{
	struct fb_info *info = container_of(work, struct fb_info,
						deferred_work.work);
	struct page *cur;
	struct fb_deferred_io *fbdefio = info->fbdefio;
	int i;

	/* Here we mkclean the pages, then do all deferred IO */
	mutex_lock(&fbdefio->lock);

	/* walk the pagemap to find written pages and then clean them */
	for (i = find_first_bit(fbdefio->pagemap, fbdefio->pagecount);
		i < fbdefio->pagecount;
		i = find_next_bit(fbdefio->pagemap, fbdefio->pagecount, i),
		i++) {
		cur = fb_deferred_io_page(info, i * PAGE_SIZE);
		lock_page(cur);
		page_mkclean(cur);
		unlock_page(cur);
	}

	/* now do driver's callback to pass them the pagemap */
	fbdefio->deferred_io(info);

	/* Clear the pagemap */
	bitmap_zero(fbdefio->pagemap, fbdefio->pagecount);
	mutex_unlock(&fbdefio->lock);
}

void fb_deferred_io_init(struct fb_info *info)
{
	struct fb_deferred_io *fbdefio = info->fbdefio;
	int pmap_bytes;

	BUG_ON(!fbdefio);
	mutex_init(&fbdefio->lock);
	info->fbops->fb_mmap = fb_deferred_io_mmap;
	INIT_DELAYED_WORK(&info->deferred_work, fb_deferred_io_work);
	fbdefio->pagecount = DIV_ROUND_UP(info->fix.smem_len, PAGE_SIZE);
	pmap_bytes = DIV_ROUND_UP(fbdefio->pagecount, sizeof(u32));
	fbdefio->pagemap = kzalloc(pmap_bytes, GFP_KERNEL);
	if (fbdefio->delay == 0) /* set a default of 1 s */
		fbdefio->delay = HZ;
}
EXPORT_SYMBOL_GPL(fb_deferred_io_init);

void fb_deferred_io_open(struct fb_info *info,
			 struct inode *inode,
			 struct file *file)
{
	file->f_mapping->a_ops = &fb_deferred_io_aops;
}
EXPORT_SYMBOL_GPL(fb_deferred_io_open);

void fb_deferred_io_cleanup(struct fb_info *info)
{
	struct fb_deferred_io *fbdefio = info->fbdefio;
	struct page *page;
	int i;

	BUG_ON(!fbdefio);
	cancel_delayed_work(&info->deferred_work);
	flush_scheduled_work();

	/* clear out the mapping that we setup */
	for (i = 0 ; i < info->fix.smem_len; i += PAGE_SIZE) {
		page = fb_deferred_io_page(info, i);
		lock_page(page);
		page->mapping = NULL;
		unlock_page(page);
	}


	info->fbops->fb_mmap = NULL;
	mutex_destroy(&fbdefio->lock);
	kfree(fbdefio->pagemap);
}
EXPORT_SYMBOL_GPL(fb_deferred_io_cleanup);

ssize_t fb_defio_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	ssize_t wr_count;
	unsigned long i;
	struct fb_deferred_io *fbdefio;
	struct page *page;
	unsigned long orig_p = *ppos;

	/*
	 * fb_sys_write can return err but have completed some portion
	 * of a write. In such a scenario, we just leave that data since
	 * it is incomplete and its a fb so a subsequent write should
	 * clear things up.
	 */
	wr_count = fb_sys_write(info, buf, count, ppos);
	if (wr_count <= 0)
		return wr_count;

	/* if not using defio, we're done here. */
	fbdefio = info->fbdefio;
	if (!fbdefio)
		return wr_count;

	/* now we must list the pages fb_sys_write has written. */
	for (i = orig_p ; i < (orig_p + wr_count) ; i += PAGE_SIZE) {

		/* get the right page */
		page = fb_deferred_io_page(info, i);

		/*
		 * It is ok if user mappings haven't been setup, the only
		 * requirement is that index is setup for the user. The
		 * reasoning is that, if there is a user mapping, we are
		 * already in good shape, if there isn't than mkclean
		 * will just fail but this should have no ill impact on us
		 * or the client driver.
		 */
		if (!page->index)
			page->index = i >> PAGE_SHIFT;

		/* set it in our pagemap */
		fb_defio_set_pagemap(fbdefio, page);
	}

	/* now we schedule our deferred work */
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);

	return wr_count;
}
EXPORT_SYMBOL_GPL(fb_defio_write);

MODULE_LICENSE("GPL");
