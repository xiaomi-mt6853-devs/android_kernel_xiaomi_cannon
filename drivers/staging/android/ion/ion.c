/*
 *
 * drivers/staging/android/ion/ion.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#endif
#if IS_ENABLED(CONFIG_PROC_FS)
#include <linux/proc_fs.h>
#endif
#include <linux/dma-buf.h>
#include <linux/idr.h>
#include <linux/sched/task.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include "ion.h"
#include "ion_priv.h"
#include "compat_ion.h"
#include "aee.h"
#include "mtk/ion_profile.h"
#include "mtk/mtk_ion.h"
#include "mtk/ion_drv_priv.h"

#ifdef CONFIG_MTK_IOMMU_V2
#include <mach/pseudo_m4u.h>
#endif

static atomic_long_t total_heap_bytes;

struct ion_dmabuf_list {
	struct list_head head;
	struct mutex lock; /* dmabuf lock */
};

static struct ion_dmabuf_list dmabuf_list;

atomic64_t page_sz_cnt = ATOMIC64_INIT(0);

/* To support  ion_dmabuf_dbg_show, we also need to porting
 * dma-buf.c . Ref: Change 3104500
 */
void ion_dmabuf_init(void)
{
	mutex_init(&dmabuf_list.lock);
	INIT_LIST_HEAD(&dmabuf_list.head);
}

void ion_dmabuf_dbg_show(struct seq_file *s)
{
	struct dma_buf *dmabuf;

	mutex_lock(&dmabuf_list.lock);

	if (s) {
		seq_puts(s, "\ndmabuf get/put count for orphaned buffer\n");
		seq_printf(s, "%18.s %8.s %4.s\n",
			   "buffer", "size", "ref");
	} else {
		pr_info("\ndmabuf get/put count for orphaned buffer\n");
		pr_info("%18.s %8.s %3.s\n",
			"buffer", "size", "ref");
	}
	list_for_each_entry(dmabuf, &dmabuf_list.head, node) {
		if (atomic_read(&dmabuf->ref_dbg) > 0) {
			struct ion_buffer *buffer = dmabuf->priv;

			if (buffer->handle_count)
				continue;
			if (s)
				seq_printf(s, "0x%p %8zu %3d\n",
					   buffer, buffer->size,
					atomic_read(&dmabuf->ref_dbg));
			else
				pr_info("0x%p %8zu %3d\n",
					buffer, buffer->size,
					atomic_read(&dmabuf->ref_dbg));
		}
	}

	mutex_unlock(&dmabuf_list.lock);
}

void ion_client_buf_add(struct ion_heap *heap, struct ion_client *client,
			size_t size)
{
#ifdef ION_RECORD_TOTAL_SIZE_SUPPORT
	u64 total_size;

	client->hnd_cnt++;
	if (heap->type == ION_HEAP_TYPE_MULTIMEDIA_SEC)
		total_size =
		atomic64_add_return(size, &client->total_size[SECURE_HEAP]);
	else if (heap->type == ION_HEAP_TYPE_SYSTEM)
		total_size =
		atomic64_add_return(size, &client->total_size[SYSTEM_HEAP]);
	else
		total_size =
		atomic64_add_return(size, &client->total_size[NORMAL_HEAP]);

	if (total_size >= client->threshold_size) {
		if (client->task) {
			char task_comm[TASK_COMM_LEN];

			get_task_comm(task_comm, client->task);
			IONMSG(
			       "warn: client:%s(%s) memory exceed threshold:%llu, heap:%u, pid:%d\n",
				task_comm, (*client->dbg_name) ?
				client->dbg_name : client->name,
				client->threshold_size,
				heap->id, client->pid);
		} else {
			IONMSG(
			       "warn: client:%s(%s) memory exceed threshold:%llu, heap:%u, pid:%d\n",
				client->name, "from_kernel",
				client->threshold_size,
				heap->id, client->pid);
		}
		client->threshold_size += CLIENT_THRESHOLD_SIZE_INC;
	}
#endif
}

void ion_client_buf_sub(struct ion_heap *heap, struct ion_client *client,
			size_t size)
{
#ifdef ION_RECORD_TOTAL_SIZE_SUPPORT
	long long total_size;

	client->hnd_cnt--;
	if (heap->type == ION_HEAP_TYPE_MULTIMEDIA_SEC) {
		total_size =
		atomic64_sub_return(size, &client->total_size[SECURE_HEAP]);
		if (total_size < 0) {
			IONMSG(
			       "heap_id:%u underflow!, total_now[%lld--%ld]\n",
			heap->id, total_size,
			atomic64_read(&client->total_size[SECURE_HEAP]));
			atomic64_set(&client->total_size[SECURE_HEAP], 0);
		}
	} else if (heap->type == ION_HEAP_TYPE_SYSTEM) {
		total_size =
		atomic64_sub_return(size, &client->total_size[SYSTEM_HEAP]);
		if (total_size < 0) {
			IONMSG(
			       "heap_id:%u underflow!, total_now[%lld--%ld]\n",
			heap->id, total_size,
			atomic64_read(&client->total_size[SYSTEM_HEAP]));
			atomic64_set(&client->total_size[SYSTEM_HEAP], 0);
		}
	} else {
		total_size =
		atomic64_sub_return(size, &client->total_size[NORMAL_HEAP]);
		if (total_size < 0) {
			IONMSG(
			       "heap_id:%u underflow!, total_now[%lld--%ld]\n",
			heap->id, total_size,
			atomic64_read(&client->total_size[NORMAL_HEAP]));
			atomic64_set(&client->total_size[NORMAL_HEAP], 0);
		}
	}

	if (client->threshold_size > CLIENT_THRESHOLD_SIZE &&
	    (total_size <=
	    (client->threshold_size - CLIENT_THRESHOLD_SIZE_DEC))) {
		client->threshold_size -= CLIENT_THRESHOLD_SIZE_INC;
		if (client->threshold_size < CLIENT_THRESHOLD_SIZE)
			client->threshold_size = CLIENT_THRESHOLD_SIZE;
	}
#endif
}

u64 ion_client_buf_dump(struct ion_heap *heap, struct ion_client *client)
{
#ifdef ION_RECORD_TOTAL_SIZE_SUPPORT
	if (heap->type == ION_HEAP_TYPE_MULTIMEDIA_SEC)
		return (u64)(atomic64_read(&client->total_size[SECURE_HEAP]));
	else if (heap->type == ION_HEAP_TYPE_SYSTEM)
		return (u64)(atomic64_read(&client->total_size[SYSTEM_HEAP]));
	else
		return (u64)(atomic64_read(&client->total_size[NORMAL_HEAP]));
#else
	return 0;
#endif
}

bool ion_buffer_fault_user_mappings(struct ion_buffer *buffer)
{
	return (buffer->flags & ION_FLAG_CACHED) &&
		!(buffer->flags & ION_FLAG_CACHED_NEEDS_SYNC);
}

bool ion_buffer_cached(struct ion_buffer *buffer)
{
	return !!(buffer->flags & ION_FLAG_CACHED);
}

static inline struct page *ion_buffer_page(struct page *page)
{
	return (struct page *)((unsigned long)page & ~(1UL));
}

static inline bool ion_buffer_page_is_dirty(struct page *page)
{
	return !!((unsigned long)page & 1UL);
}

static inline void ion_buffer_page_dirty(struct page **page)
{
	*page = (struct page *)((unsigned long)(*page) | 1UL);
}

static inline void ion_buffer_page_clean(struct page **page)
{
	*page = (struct page *)((unsigned long)(*page) & ~(1UL));
}

/* this function should only be called while dev->lock is held */
static void ion_buffer_add(struct ion_device *dev,
			   struct ion_buffer *buffer)
{
	struct rb_node **p = &dev->buffers.rb_node;
	struct rb_node *parent = NULL;
	struct ion_buffer *entry;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_buffer, node);

		if (buffer < entry) {
			p = &(*p)->rb_left;
		} else if (buffer > entry) {
			p = &(*p)->rb_right;
		} else {
			pr_err("%s: buffer already found.", __func__);
			BUG();
		}
	}

	rb_link_node(&buffer->node, parent, p);
	rb_insert_color(&buffer->node, &dev->buffers);
}

/* this function should only be called while dev->lock is held */
static struct ion_buffer *ion_buffer_create(struct ion_heap *heap,
					    struct ion_device *dev,
					    unsigned long len,
					    unsigned long align,
					    unsigned long flags)
{
	struct ion_buffer *buffer;
	struct sg_table *table;
	struct scatterlist *sg;
	int i, ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		IONMSG("%s kzalloc failed, buffer is null.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	buffer->heap = heap;
	buffer->flags = flags;

	/* log task pid for debug +by k.zhang */
	{
		struct task_struct *task;

		task = current->group_leader;
		get_task_comm(buffer->task_comm, task);
		buffer->pid = task_pid_nr(task);
		buffer->tid = task_pid_nr(current);
		get_task_comm(buffer->thread_comm, current);
		buffer->timestamp = sched_clock();
	}

	kref_init(&buffer->ref);

	ret = heap->ops->allocate(heap, buffer, len, align, flags);

	if (ret) {
		if (!(heap->flags & ION_HEAP_FLAG_DEFER_FREE))
			goto err2;

		ion_heap_freelist_drain(heap, 0);
		ret = heap->ops->allocate(heap, buffer, len, align,
					  flags);
		if (ret)
			goto err2;
	}

	if (!buffer->sg_table) {
		WARN_ONCE(1, "This heap needs to set the sgtable");
		ret = -EINVAL;
		goto err1;
	}

	table = buffer->sg_table;
	buffer->dev = dev;
	buffer->size = len;

	if (ion_buffer_fault_user_mappings(buffer)) {
		int num_pages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
		struct scatterlist *sg;
		int i, j, k = 0;

		buffer->pages = vmalloc(sizeof(struct page *) * num_pages);
		if (!buffer->pages) {
			IONMSG("%s vamlloc failed pages is null.\n", __func__);
			ret = -ENOMEM;
			goto err1;
		}

		for_each_sg(table->sgl, sg, table->nents, i) {
			struct page *page = sg_page(sg);

			for (j = 0; j < sg->length / PAGE_SIZE; j++)
				buffer->pages[k++] = page++;
		}
	}

	buffer->dev = dev;
	buffer->size = len;
	INIT_LIST_HEAD(&buffer->vmas);
#ifdef MTK_ION_DMABUF_SUPPORT
	INIT_LIST_HEAD(&buffer->attachments);
#endif
	mutex_init(&buffer->lock);
	if (heap->type == ION_HEAP_TYPE_MULTIMEDIA_SEC)
		goto exit;
	/*
	 * this will set up dma addresses for the sglist -- it is not
	 * technically correct as per the dma api -- a specific
	 * device isn't really taking ownership here.  However, in practice on
	 * our systems the only dma_address space is physical addresses.
	 * Additionally, we can't afford the overhead of invalidating every
	 * allocation via dma_map_sg. The implicit contract here is that
	 * memory coming from the heaps is ready for dma, ie if it has a
	 * cached mapping that mapping has been invalidated
	 */
	for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents, i) {
		sg_dma_address(sg) = sg_phys(sg);
		sg_dma_len(sg) = sg->length;
	}
exit:
	mutex_lock(&dev->buffer_lock);
	ion_buffer_add(dev, buffer);
	mutex_unlock(&dev->buffer_lock);
	atomic_long_add(len, &total_heap_bytes);
	atomic_long_add(len, &heap->total_allocated);
	return buffer;

err1:
	heap->ops->free(buffer);
err2:
	kfree(buffer);
	return ERR_PTR(ret);
}

void ion_buffer_destroy(struct ion_buffer *buffer)
{
	if (buffer->kmap_cnt > 0) {
		WARN_ON(1);
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
	}

	atomic_long_sub(buffer->size, &buffer->heap->total_allocated);
	buffer->heap->ops->free(buffer);
	vfree(buffer->pages);
	kfree(buffer);
}

static void _ion_buffer_destroy(struct kref *kref)
{
	struct ion_buffer *buffer = container_of(kref, struct ion_buffer, ref);
	struct ion_heap *heap = buffer->heap;
	struct ion_device *dev = buffer->dev;

	mutex_lock(&dev->buffer_lock);
	rb_erase(&buffer->node, &dev->buffers);
	mutex_unlock(&dev->buffer_lock);
	atomic_long_sub(buffer->size, &total_heap_bytes);

	if (unlikely(buffer->flags & ION_FLAG_FREE_WITHOUT_DEFER))
		ion_buffer_destroy(buffer);
	else if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_freelist_add(heap, buffer);
	else
		ion_buffer_destroy(buffer);
}

static void ion_buffer_get(struct ion_buffer *buffer)
{
	kref_get(&buffer->ref);
}

static int ion_buffer_put(struct ion_buffer *buffer)
{
	return kref_put(&buffer->ref, _ion_buffer_destroy);
}

static void ion_buffer_add_to_handle(struct ion_buffer *buffer)
{
	mutex_lock(&buffer->lock);
	buffer->handle_count++;
	mutex_unlock(&buffer->lock);
}

static void ion_buffer_remove_from_handle(struct ion_buffer *buffer)
{
	/*
	 * when a buffer is removed from a handle, if it is not in
	 * any other handles, copy the taskcomm and the pid of the
	 * process it's being removed from into the buffer.  At this
	 * point there will be no way to track what processes this buffer is
	 * being used by, it only exists as a dma_buf file descriptor.
	 * The taskcomm and pid can provide a debug hint as to where this fd
	 * is in the system
	 */
	mutex_lock(&buffer->lock);
	buffer->handle_count--;
	WARN_ON(buffer->handle_count < 0);
	if (!buffer->handle_count) {
		struct task_struct *task;

		task = current->group_leader;
		get_task_comm(buffer->task_comm, task);
		buffer->pid = task_pid_nr(task);
	}
	mutex_unlock(&buffer->lock);
}

static struct ion_handle *ion_handle_create(struct ion_client *client,
					    struct ion_buffer *buffer)
{
	struct ion_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle) {
		IONMSG("%s kzalloc failed handle is null.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	kref_init(&handle->ref);
	RB_CLEAR_NODE(&handle->node);
	handle->client = client;
	ion_buffer_get(buffer);
	ion_buffer_add_to_handle(buffer);
	handle->buffer = buffer;

	return handle;
}

static void ion_handle_kmap_put(struct ion_handle *);

static void ion_handle_destroy(struct kref *kref)
{
	struct ion_handle *handle = container_of(kref, struct ion_handle, ref);
	struct ion_client *client = handle->client;
	struct ion_buffer *buffer = handle->buffer;

	mutex_lock(&buffer->lock);
	while (handle->kmap_cnt)
		ion_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);

	idr_remove(&client->idr, handle->id);
	if (!RB_EMPTY_NODE(&handle->node))
		rb_erase(&handle->node, &client->handles);

	ion_client_buf_sub(buffer->heap, client, buffer->size);
	ion_buffer_remove_from_handle(buffer);
	ion_buffer_put(buffer);

	handle->buffer = NULL;
	handle->client = NULL;

	kfree(handle);
}

static void ion_handle_get(struct ion_handle *handle)
{
	kref_get(&handle->ref);
}

static struct ion_handle *ion_handle_get_check_overflow(
							struct ion_handle *handle)
{
	if (atomic_read(&handle->ref.refcount.refs) + 1 == 0)
		return ERR_PTR(-EOVERFLOW);
	ion_handle_get(handle);
	return handle;
}

int ion_handle_put_nolock(struct ion_handle *handle)
{
	return kref_put(&handle->ref, ion_handle_destroy);
}

int ion_handle_put(struct ion_handle *handle)
{
	struct ion_client *client = handle->client;
	int ret;

	mutex_lock(&client->lock);
	ret = ion_handle_put_nolock(handle);
	mutex_unlock(&client->lock);

	return ret;
}

/* Must hold the client lock */
static void user_ion_handle_get(struct ion_handle *handle)
{
	if (handle->user_ref_count++ == 0)
		kref_get(&handle->ref);
}

/* Must hold the client lock */
static struct ion_handle *user_ion_handle_get_check_overflow(struct ion_handle *handle)
{
	if (handle->user_ref_count + 1 == 0)
		return ERR_PTR(-EOVERFLOW);
	user_ion_handle_get(handle);
	return handle;
}

/* passes a kref to the user ref count.
 * We know we're holding a kref to the object before and
 * after this call, so no need to reverify handle.
 */
struct ion_handle *pass_to_user(struct ion_handle *handle)
{
	struct ion_client *client = handle->client;
	struct ion_handle *ret;

	mutex_lock(&client->lock);
	ret = user_ion_handle_get_check_overflow(handle);
	ion_handle_put_nolock(handle);
	mutex_unlock(&client->lock);
	return ret;
}

/* Must hold the client lock */
static int user_ion_handle_put_nolock(struct ion_handle *handle)
{
	int ret = 0;

	if (--handle->user_ref_count == 0)
		ret = ion_handle_put_nolock(handle);
	return ret;
}

static struct ion_handle *ion_handle_lookup(struct ion_client *client,
					    struct ion_buffer *buffer)
{
	struct rb_node *n = client->handles.rb_node;

	while (n) {
		struct ion_handle *entry = rb_entry(n, struct ion_handle, node);

		if (buffer < entry->buffer)
			n = n->rb_left;
		else if (buffer > entry->buffer)
			n = n->rb_right;
		else
			return entry;
	}
	return ERR_PTR(-EINVAL);
}

struct ion_handle *ion_handle_get_by_id_nolock(struct ion_client *client,
					       int id)
{
	struct ion_handle *handle;

	handle = idr_find(&client->idr, id);
	if (handle)
		return ion_handle_get_check_overflow(handle);
	else
		IONDBG("%s: can't get handle by id:%d\n", __func__, id);
	return  ERR_PTR(-EINVAL);
}


static bool ion_handle_validate(struct ion_client *client,
				struct ion_handle *handle)
{
	WARN_ON(!mutex_is_locked(&client->lock));
	return idr_find(&client->idr, handle->id) == handle;
}

static int ion_handle_add(struct ion_client *client, struct ion_handle *handle)
{
	int id;
	struct rb_node **p = &client->handles.rb_node;
	struct rb_node *parent = NULL;
	struct ion_handle *entry;

	id = idr_alloc(&client->idr, handle, 1, 0, GFP_KERNEL);
	if (id < 0) {
		IONMSG("%s idr_alloc failed id = %d.\n", __func__, id);
		return id;
	}

	handle->id = id;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_handle, node);

		if (handle->buffer < entry->buffer)
			p = &(*p)->rb_left;
		else if (handle->buffer > entry->buffer)
			p = &(*p)->rb_right;
		else
			WARN(1, "%s: buffer already found.", __func__);
	}

	rb_link_node(&handle->node, parent, p);
	rb_insert_color(&handle->node, &client->handles);

	return 0;
}

struct ion_handle *__ion_alloc(struct ion_client *client, size_t len,
			       size_t align, unsigned int heap_id_mask,
			       unsigned int flags, bool grab_handle)
{
	struct ion_handle *handle;
	struct ion_device *dev = client->dev;
	struct ion_buffer *buffer = NULL;
	struct ion_heap *heap;
	int ret;
	unsigned long long start, end;
	unsigned int heap_mask = ~0;
	unsigned int alloc_err_heap = 0;

	pr_debug("%s: len %zu align %zu heap_id_mask %u flags %x\n", __func__,
		 len, align, heap_id_mask, flags);

	/* For some case(C2 audio decoder), it can not set heap id in AOSP,
	 * so mtk ion will set this heap to ion_mm_heap.
	 */
	if (heap_id_mask == heap_mask)
		heap_id_mask = ION_HEAP_MULTIMEDIA_MASK;

	/*
	 * traverse the list of heaps available in this system in priority
	 * order.  If the heap type is supported by the client, and matches the
	 * request of the caller allocate from it.  Repeat until allocate has
	 * succeeded or all heaps have been tried
	 */
	len = PAGE_ALIGN(len);

	if (!len) {
		IONMSG("%s len cannot be zero.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_ALLOC], MMPROFILE_FLAG_START,
			 (unsigned long)client, len);
	start = sched_clock();

	down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		/* if the caller didn't specify this heap id */
		if (!((1 << heap->id) & heap_id_mask))
			continue;
		buffer = ion_buffer_create(heap, dev, len, align, flags);
		if (!IS_ERR(buffer))
			break;
		if (IS_ERR(buffer))
			alloc_err_heap |= (1 << heap->id);
	}
	up_read(&dev->lock);

	if (!buffer) {
		IONMSG("%s buffer is null.\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	if (IS_ERR(buffer)) {
		IONMSG("%s error 0x%lx, heap_mask: 0x%x size:%zu align:%zu.\n",
		       __func__, (unsigned long)buffer,
		       heap_id_mask, len, align);
		IONMSG("error heap:0x%x\n", alloc_err_heap);
		plist_for_each_entry(heap, &dev->heaps, node) {
			if (!((1 << heap->id) & alloc_err_heap))
				continue;
			if (IS_ERR_OR_NULL(heap->debug_show))
				continue;
			IONMSG("heap[%s][%d] show:\n", heap->name, heap->id);
			heap->debug_show(heap, NULL, NULL);
		}
		return ERR_CAST(buffer);
	}

	handle = ion_handle_create(client, buffer);

	/*
	 * ion_buffer_create will create a buffer with a ref_cnt of 1,
	 * and ion_handle_create will take a second reference, drop one here
	 */
	ion_buffer_put(buffer);

	if (IS_ERR(handle)) {
		IONMSG("%s handle is error 0x%p.\n", __func__, handle);
		return handle;
	}

	mutex_lock(&client->lock);
	ret = ion_handle_add(client, handle);
	ion_client_buf_add(heap, client, len);
	if (!ret && grab_handle)
		ion_handle_get(handle);
	mutex_unlock(&client->lock);
	end = sched_clock();
	if (ret) {
		ion_handle_put(handle);
		handle = ERR_PTR(ret);
		IONMSG("%s ion handle add failed %d.\n", __func__, ret);
	} else {
		handle->dbg.user_ts = end;
		do_div(handle->dbg.user_ts, 1000000);
		memcpy(buffer->alloc_dbg, client->dbg_name, ION_MM_DBG_NAME_LEN);
	}

	if (end - start > 100000000ULL) {/* unit is ns */
		IONMSG("warn: ion alloc buffer size: %zu time: %lld ns\n",
		       buffer->size, end - start);
	}
	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_ALLOC], MMPROFILE_FLAG_END,
			 (unsigned long)client, (unsigned long)handle);

#ifdef CONFIG_MTK_ION
#ifdef ION_HISTORY_RECORD
	ion_history_count_kick(true, len);
#endif
#endif

	return handle;
}

struct ion_handle *ion_alloc(struct ion_client *client, size_t len,
			     size_t align, unsigned int heap_id_mask,
			     unsigned int flags)
{
	return __ion_alloc(client, len, align, heap_id_mask, flags, false);
}
EXPORT_SYMBOL(ion_alloc);

void user_ion_free_nolock(struct ion_client *client, struct ion_handle *handle)
{
	bool valid_handle;

	WARN_ON(client != handle->client);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to free.\n", __func__);
		return;
	}
	if (handle->user_ref_count <= 0) {
		WARN(1, "%s: User does not have access!\n", __func__);
		return;
	}
	user_ion_handle_put_nolock(handle);
}

void ion_free_nolock(struct ion_client *client,
		     struct ion_handle *handle)
{
	WARN_ON(client != handle->client);

	if (!ion_handle_validate(client, handle)) {
		WARN(1, "%s: invalid handle passed to free.\n", __func__);
		return;
	}
	ion_handle_put_nolock(handle);
#ifdef CONFIG_MTK_ION
#ifdef ION_HISTORY_RECORD
	ion_history_count_kick(false, 0);
#endif
#endif
}

void ion_free(struct ion_client *client, struct ion_handle *handle)
{
	WARN_ON(client != handle->client);

	mutex_lock(&client->lock);
	ion_free_nolock(client, handle);
	mutex_unlock(&client->lock);
	mmprofile_log_ex(ion_mmp_events[PROFILE_FREE], MMPROFILE_FLAG_PULSE,
			 (unsigned long)client, (unsigned long)handle);
}
EXPORT_SYMBOL(ion_free);

static void *ion_buffer_kmap_get(struct ion_buffer *buffer)
{
	void *vaddr;

	if (buffer->kmap_cnt + 1 == 0) {
		IONMSG("%s overflow\n", __func__);
		return ERR_PTR(-EOVERFLOW);
	}

	if (buffer->kmap_cnt) {
		if (buffer->kmap_cnt == INT_MAX)
			return ERR_PTR(-EOVERFLOW);

		buffer->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = buffer->heap->ops->map_kernel(buffer->heap, buffer);
	if (!vaddr) {
		WARN_ONCE(1,
			  "heap->ops->map_kernel should return ERR_PTR on error");
		return ERR_PTR(-EINVAL);
	}
	if (IS_ERR(vaddr)) {
		IONMSG("%s map kernel is failed addr = 0x%p.\n",
		       __func__, vaddr);
		return vaddr;
	}
	buffer->vaddr = vaddr;
	buffer->kmap_cnt++;
	return vaddr;
}

static void *ion_handle_kmap_get(struct ion_handle *handle)
{
	struct ion_buffer *buffer = handle->buffer;
	void *vaddr;

	if (handle->kmap_cnt + 1 == 0) {
		IONMSG("%s overflow\n", __func__);
		return ERR_PTR(-EOVERFLOW);
	}

	if (handle->kmap_cnt) {
		handle->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = ion_buffer_kmap_get(buffer);
	if (IS_ERR(vaddr)) {
		IONMSG("%s vadd is error 0x%p.\n", __func__, vaddr);
		return vaddr;
	}
	handle->kmap_cnt++;
	return vaddr;
}

static void ion_buffer_kmap_put(struct ion_buffer *buffer)
{
	buffer->kmap_cnt--;
	if (!buffer->kmap_cnt) {
		/*avoid camelcase, will modify in a letter*/
		mmprofile_log_ex(ion_mmp_events[PROFILE_UNMAP_KERNEL],
				 MMPROFILE_FLAG_START,
				 buffer->size, buffer->kmap_cnt);
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
		mmprofile_log_ex(ion_mmp_events[PROFILE_UNMAP_KERNEL],
				 MMPROFILE_FLAG_END,
				 buffer->size, buffer->kmap_cnt);
		buffer->vaddr = NULL;
	}
}

static void ion_handle_kmap_put(struct ion_handle *handle)
{
	struct ion_buffer *buffer = handle->buffer;

	if (!handle->kmap_cnt) {
		WARN(1, "%s: Double unmap detected! bailing...\n", __func__);
		return;
	}
	handle->kmap_cnt--;
	if (!handle->kmap_cnt)
		ion_buffer_kmap_put(buffer);
}

void *ion_map_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_handle *handle_ret;
	struct ion_buffer *buffer;
	void *vaddr;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_kernel.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}

	buffer = handle->buffer;

	if (!handle->buffer->heap->ops->map_kernel) {
		pr_err("%s: map_kernel is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-ENODEV);
	}

	handle_ret = ion_handle_get_check_overflow(handle);
	if (IS_ERR(handle_ret)) {
		mutex_unlock(&client->lock);
		WARN(1, "%s: handle get fail %ld.\n", __func__,
		     PTR_ERR(handle_ret));
		return handle_ret;
	}
	mutex_lock(&buffer->lock);
	vaddr = ion_handle_kmap_get(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
	return vaddr;
}
EXPORT_SYMBOL(ion_map_kernel);

void ion_unmap_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	ion_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);

	ion_handle_put_nolock(handle);
	mutex_unlock(&client->lock);
}
EXPORT_SYMBOL(ion_unmap_kernel);

static int ion_client_validate(struct ion_device *dev,
			       struct ion_client *client)
{
	struct rb_node *n;

	for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
		struct ion_client *valid_client = rb_entry(n,
							   struct ion_client,
							   node);

		if (client == valid_client)
			return 1;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_DEBUG_FS) || IS_ENABLED(CONFIG_PROC_FS)
static int ion_debug_client_show(struct seq_file *s, void *unused)
{
	struct ion_client *client = s->private;
	struct ion_device *dev = g_ion_device;
	struct rb_node *n;
	/*size_t sizes[ION_NUM_HEAP_IDS] = {0};*/
	/*const char *names[ION_NUM_HEAP_IDS] = {NULL};*/
	size_t *sizes;
	const char **names;
	int i;

	sizes = kcalloc(ION_NUM_HEAP_IDS, sizeof(size_t), GFP_ATOMIC);

	if (!sizes)
		return -ENOMEM;

	names = kcalloc(ION_NUM_HEAP_IDS, sizeof(char *), GFP_ATOMIC);

	if (!names) {
		kfree(sizes);
		return -ENOMEM;
	}

	if (!down_read_trylock(&dev->lock)) {
		IONMSG("%s get lock fail\n", __func__);
		kfree(sizes);
		kfree(names);
		return 0;
	}
	if (!ion_client_validate(dev, client)) {
		seq_printf(s, "ion_client 0x%pK dead, can't dump its buffers\n",
			   client);
		IONMSG("%s:client invalid\n", __func__);
		up_read(&dev->lock);
		kfree(sizes);
		kfree(names);
		return 0;
	}

	seq_printf(s, "%16.s %8.s %8.s %8.s %8.s %8.s\n",
		   "heap_name", "pid", "size",
		   "handle_count", "handle", "buffer");

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);
		struct ion_buffer *buffer = handle->buffer;
		unsigned int id = buffer->heap->id;

		if (!names[id])
			names[id] = buffer->heap->name;
		sizes[id] += buffer->size;

		seq_printf(s, "%16.s %3d %8zu %3d %pK %pK.\n", buffer->heap->name,
			   client->pid, buffer->size,
			   buffer->handle_count, handle, buffer);
	}
	mutex_unlock(&client->lock);
	up_read(&dev->lock);

	seq_puts(s, "----------------------------------------------------\n");

	seq_printf(s, "%16.16s: %16.16s\n", "heap_name", "size_in_bytes");
	for (i = 0; i < ION_NUM_HEAP_IDS; i++) {
		if (!names[i])
			continue;
		seq_printf(s, "%16.16s: %16zu\n", names[i], sizes[i]);
	}

	kfree(sizes);
	kfree(names);

	return 0;
}

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int ion_debug_client_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_client_show, inode->i_private);
}

static const struct file_operations debug_client_fops = {
	.open = ion_debug_client_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

#if IS_ENABLED(CONFIG_PROC_FS)
static int ion_proc_client_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_client_show, PDE_DATA(inode));
}

static const struct file_operations proc_client_fops = {
	.open = ion_proc_client_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif
#endif

static int ion_get_client_serial(const struct rb_root *root,
				 const unsigned char *name)
{
	int serial = -1;
	struct rb_node *node;

	for (node = rb_first(root); node; node = rb_next(node)) {
		struct ion_client *client = rb_entry(node, struct ion_client,
						     node);

		if (strcmp(client->name, name))
			continue;
		serial = max(serial, client->display_serial);
	}
	return serial + 1;
}

struct ion_client *ion_client_create(struct ion_device *dev,
				     const char *name)
{
	struct ion_client *client;
	struct task_struct *task;
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct ion_client *entry;
	pid_t pid;

	if (!name) {
		pr_err("%s: Name cannot be null\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	pid = task_pid_nr(current->group_leader);
	/*
	 * don't bother to store task struct for kernel threads,
	 * they can't be killed anyway
	 */
	if (current->group_leader->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		goto err_put_task_struct;

	client->threshold_size = CLIENT_THRESHOLD_SIZE;
	client->dev = dev;
	client->handles = RB_ROOT;
	idr_init(&client->idr);
	mutex_init(&client->lock);
	client->task = task;
	client->pid = pid;
	client->name = kstrdup(name, GFP_KERNEL);
	if (!client->name)
		goto err_free_client;

	down_write(&dev->lock);
	client->display_serial = ion_get_client_serial(&dev->clients, name);
	client->display_name = kasprintf(
		GFP_KERNEL, "%s-%d", name, client->display_serial);
	if (!client->display_name) {
		up_write(&dev->lock);
		goto err_free_client_name;
	}
	p = &dev->clients.rb_node;
	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_client, node);

		if (client < entry)
			p = &(*p)->rb_left;
		else if (client > entry)
			p = &(*p)->rb_right;
	}
	rb_link_node(&client->node, parent, p);
	rb_insert_color(&client->node, &dev->clients);

#if IS_ENABLED(CONFIG_PROC_FS)
	client->proc_root = proc_create_data(client->display_name,
					     S_IFREG | 0664,
					     dev->clients_proc_root,
					     &proc_client_fops,
					     client);
	if (!client->proc_root)
		IONMSG(
		       "Failed to create client procfs at /proc/ion/clients/%s\n",
		       client->display_name);
#endif

	up_write(&dev->lock);

	return client;

err_free_client_name:
	kfree(client->name);
err_free_client:
	kfree(client);
err_put_task_struct:
	if (task)
		put_task_struct(current->group_leader);
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL(ion_client_create);

void ion_client_destroy(struct ion_client *client)
{
	struct ion_device *dev = client->dev;
	struct rb_node *n;
	struct task_struct *task = current->group_leader;
	char task_comm[TASK_COMM_LEN];
	pid_t pid;
	unsigned long long time_s, time_e_lock, time_e_unlock;

	pr_debug("%s: %d\n", __func__, __LINE__);
	get_task_comm(task_comm, task);
	pid = task_pid_nr(task);
	time_s = sched_clock();
	while ((n = rb_first(&client->handles))) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);

		mutex_lock(&client->lock);
		IONMSG(
		       "%s:hdl=%p,buf=%p,sz=%zu,ref=%d,kmp=%d, client=%s,disp=%s,dbg=%s\n",
		       __func__, handle, handle->buffer,
		       handle->buffer->size,
		       atomic_read(&handle->buffer->ref.refcount.refs),
		       handle->buffer->kmap_cnt,
		       client->name ? client->name : NULL,
		       client->display_name ? client->display_name : NULL,
		       client->dbg_name);
		ion_handle_destroy(&handle->ref);
		mutex_unlock(&client->lock);
	}

	idr_destroy(&client->idr);

	time_e_unlock = sched_clock();
	if ((time_e_unlock - time_s) > 50000000) // 50ms
		IONMSG("%s unlock warnning, time:%llu, task:%s (%d)\n",
		       __func__, (time_e_unlock - time_s),
		       task_comm, pid);

	down_write(&dev->lock);
	if (client->task)
		put_task_struct(client->task);
	rb_erase(&client->node, &dev->clients);
#if IS_ENABLED(CONFIG_DEBUG_FS)
	debugfs_remove_recursive(client->debug_root);
#endif
#if IS_ENABLED(CONFIG_PROC_FS)
	proc_remove(client->proc_root);
#endif
	up_write(&dev->lock);

	time_e_lock = sched_clock();
	if ((time_e_lock - time_s) > 100000000) // 100ms
		IONMSG("%s warnning, time:%llu, task:%s (%d)\n",
		       __func__, (time_e_lock - time_s),
		       task_comm, pid);

	kfree(client->display_name);
	kfree(client->name);
	kfree(client);
}
EXPORT_SYMBOL(ion_client_destroy);

static void ion_buffer_sync_for_device(struct ion_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction direction);

int clone_sg_table(const struct sg_table *source, struct sg_table *dest)
{
	int i = 0;
	struct scatterlist *s_sg, *d_sg;

	if (source->nents != dest->nents) {
		IONMSG("%s, %d, source nents:%d, dest nents:%d\n",
		       __func__, __LINE__,
		       source->nents, dest->nents);
		return -1;
	}

	d_sg = dest->sgl;
	for_each_sg(source->sgl, s_sg, source->nents, i) {
		memcpy(d_sg, s_sg, sizeof(*s_sg));
		d_sg = sg_next(d_sg);
	}

	return 0;
}

#ifdef MTK_ION_DMABUF_SUPPORT
static int ion_iommu_heap_type(struct ion_buffer *buffer)
{
	if (!buffer) {
		WARN_ON(1);
		return 0;
	}

	if (buffer->heap->type == (int)ION_HEAP_TYPE_FB ||
	    buffer->heap->type == (int)ION_HEAP_TYPE_MULTIMEDIA ||
	    buffer->heap->type == (int)ION_HEAP_TYPE_MULTIMEDIA_SEC) {
		return 1;
	}
	return 0;
}

static struct sg_table *dup_sg_table(struct sg_table *table)
{
	struct sg_table *new_table;
	int ret, i;
	struct scatterlist *sg, *new_sg;

	if (!table) {
		IONMSG("%s invalid table\n", __func__);
		return ERR_PTR(-EFAULT);
	}

	new_table = kzalloc(sizeof(*new_table), GFP_KERNEL);
	if (!new_table)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(new_table, table->nents, GFP_KERNEL);
	if (ret) {
		kfree(new_table);
		return ERR_PTR(-ENOMEM);
	}

	new_sg = new_table->sgl;
	for_each_sg(table->sgl, sg, table->nents, i) {
		memcpy(new_sg, sg, sizeof(*sg));
		//sg->dma_address = 0;
		new_sg = sg_next(new_sg);
	}

	return new_table;
}

static void free_duped_table(struct sg_table *table)
{
	if (!table) {
		IONMSG("%s invalid table\n", __func__);
		return;
	}

	sg_free_table(table);
	kfree(table);
}

struct ion_dma_buf_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
};

static int ion_dma_buf_attach(struct dma_buf *dmabuf, struct device *dev,
			      struct dma_buf_attachment *attachment)
{
	struct ion_dma_buf_attachment *a;
	struct sg_table *table = NULL;
	struct ion_buffer *buffer;

	if (!attachment) {
		IONMSG("%s invalid attachment\n", __func__);
		return -ENODEV;
	}
	if (!dmabuf || !dmabuf->priv) {
		IONMSG("%s invalid dmabuf\n", __func__);
		return -ENODEV;
	}

	buffer = dmabuf->priv;

	if (!ion_iommu_heap_type(buffer) &&
	    buffer->heap->type != (int)ION_HEAP_TYPE_SYSTEM) {
		IONMSG("%s not dma buf device\n", __func__);
		return 0;
	}

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

#ifdef CONFIG_MTK_IOMMU_V2
	if (buffer->sg_table_orig)
		table = dup_sg_table(buffer->sg_table_orig);
#endif
	if (!table)
		table = dup_sg_table(buffer->sg_table);
	if (IS_ERR(table)) {
		kfree(a);
		return -ENOMEM;
	}

	a->table = table;
	a->dev = dev;
	INIT_LIST_HEAD(&a->list);

	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

    //pr_notice("%s, %d\n", __func__, __LINE__);
	return 0;
}

static void ion_dma_buf_detatch(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attachment)
{
	struct ion_dma_buf_attachment *a;
	struct ion_buffer *buffer;

	if (!attachment ||
	    !attachment->priv ||
	    !attachment->dmabuf) {
		IONMSG("%s invalid attachment\n", __func__);
		return;
	}
	a = attachment->priv;

	if (!dmabuf || !dmabuf->priv) {
		IONMSG("%s invalid dmabuf\n", __func__);
		return;
	}
	buffer = dmabuf->priv;

	if (!ion_iommu_heap_type(buffer) &&
	    buffer->heap->type != (int)ION_HEAP_TYPE_SYSTEM) {
		IONMSG("%s not dma buf device\n", __func__);
		return;
	}

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	mutex_unlock(&buffer->lock);
	free_duped_table(a->table);

    //pr_notice("%s, %d\n", __func__, __LINE__);
	kfree(a);
}

static struct sg_table *ion_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct ion_dma_buf_attachment *a;
	struct dma_buf *dmabuf;
	struct sg_table *table;
	struct ion_buffer *buffer;
	ion_phys_addr_t addr = 0x0;
	size_t len = 0;
	int ret = 0, retry = 0;

	if (!attachment ||
	    !attachment->priv ||
	    !attachment->dmabuf)
		return ERR_PTR(-ENODEV);
	a = attachment->priv;
	dmabuf = attachment->dmabuf;

	if (!dmabuf->priv)
		return ERR_PTR(-ENODEV);
	buffer = dmabuf->priv;

	if (!ion_iommu_heap_type(buffer) &&
	    buffer->heap->type != (int)ION_HEAP_TYPE_SYSTEM) {
		pr_debug("%s not iommu device\n", __func__);
		ion_buffer_sync_for_device(buffer,
					   attachment->dev,
					   direction);
		table = buffer->sg_table;
	} else if (buffer->heap->type ==
		(int)ION_HEAP_TYPE_MULTIMEDIA_SEC ||
		buffer->heap->type == (int)ION_HEAP_TYPE_SYSTEM) {
		pr_debug("%s heap type:%d\n", __func__,
			 buffer->heap->type);
		table = a->table;
		if (clone_sg_table(buffer->sg_table, table))
			return ERR_PTR(-EINVAL);
	} else {
retry:
		mutex_lock(&buffer->lock);
		if (buffer->heap->ops->dma_buf_config) {
			ret = buffer->heap->ops->dma_buf_config(
						      buffer, attachment->dev);
			if (ret) {
#if defined(ION_NOT_SUPPORT_RETRY)
				if (ret == M4U_PORT_GPU) {
					struct ion_heap *heap = buffer->heap;

					if (heap->ops->get_table) {
						heap->ops->get_table(buffer,
								a->table);
					} else {
						IONMSG(
						       "error, heap:%u get_table not support!\n",
						       heap->id);
					}
					mutex_unlock(&buffer->lock);

					dma_sync_sg_for_device(attachment->dev,
							       a->table->sgl,
							       a->table->nents,
							       direction);

					return a->table;
				}
				mutex_unlock(&buffer->lock);
#else
				mutex_unlock(&buffer->lock);
				if (ret == -ION_ERROR_CONFIG_CONFLICT &&
				    retry++ < 50) {
					IONMSG("%s, corrupt, retry%d...\n",
					       __func__, retry);
					usleep_range(20, 40);
					goto retry;
				}
#endif
				IONMSG("%s, failed at dmabuf process, ret:%d\n",
				       __func__, ret);
				return ERR_PTR(ret);
			}
		}

		ret = buffer->heap->ops->phys(buffer->heap,
					      buffer,
					      &addr,
					      &len);

		if (ret) {
			mutex_unlock(&buffer->lock);
			IONMSG("%s, failed at get phys, ret:%d\n",
			       __func__, ret);
			if (ret == -EDOM)
				aee_kernel_warning_api(__FILE__, __LINE__,
						       DB_OPT_DEFAULT |
						       DB_OPT_NATIVE_BACKTRACE,
						       "ion phys failed",
						       "dump user backtrace");
			return ERR_PTR(ret);
		}
		table = a->table;
		if (clone_sg_table(buffer->sg_table, table)) {
			mutex_unlock(&buffer->lock);
			return ERR_PTR(-EINVAL);
		}
		mutex_unlock(&buffer->lock);
	}

	return table;
}
#else
static struct sg_table *ion_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct dma_buf *dmabuf = attachment->dmabuf;
	struct ion_buffer *buffer = dmabuf->priv;

	ion_buffer_sync_for_device(buffer, attachment->dev, direction);
	return buffer->sg_table;
}

#endif

static void ion_unmap_dma_buf(struct dma_buf_attachment *attachment,
			      struct sg_table *table,
			      enum dma_data_direction direction)
{
	/*
	 * do nothing, since the ion buffer may be shared for multiple users
	 * the attachment will be freed at detach time
	 * the iova will be unmapped at buffer destroy time
	 */
}

void ion_pages_sync_for_device(struct device *dev, struct page *page,
			       size_t size, enum dma_data_direction dir)
{
	struct scatterlist sg;

	if (!page) {
		IONMSG("%s fail, page is NULL\n", __func__);
		return;
	}
	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	/*
	 * This is not correct - sg_dma_address need a dma_addr_t that is valid
	 * for the targeted device, but this works on the currently targeted
	 * hardware.
	 */
	sg_dma_address(&sg) = page_to_phys(page);
	dma_sync_sg_for_device(dev, &sg, 1, dir);
}

struct ion_vma_list {
	struct list_head list;
	struct vm_area_struct *vma;
};

static void ion_buffer_sync_for_device(struct ion_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction dir)
{
	struct ion_vma_list *vma_list;
	int pages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	int i;

	pr_debug("%s: syncing for device %s\n", __func__,
		 dev ? dev_name(dev) : "null");

	if (!ion_buffer_fault_user_mappings(buffer))
		return;

	mutex_lock(&buffer->lock);
	for (i = 0; i < pages; i++) {
		struct page *page = buffer->pages[i];

		if (ion_buffer_page_is_dirty(page))
			ion_pages_sync_for_device(dev, ion_buffer_page(page),
						  PAGE_SIZE, dir);

		ion_buffer_page_clean(buffer->pages + i);
	}
	list_for_each_entry(vma_list, &buffer->vmas, list) {
		struct vm_area_struct *vma = vma_list->vma;

		zap_page_range(vma, vma->vm_start, vma->vm_end - vma->vm_start);
	}
	mutex_unlock(&buffer->lock);
}

static int ion_vm_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct dma_buf *dmabuf = vma->vm_private_data;
	struct ion_buffer *buffer =  dmabuf->priv;
	unsigned long pfn;
	int ret;

	if (!buffer || !buffer->pages) {
		WARN_ON(1);
		return VM_FAULT_ERROR;
	}

	mutex_lock(&buffer->lock);
	ion_buffer_page_dirty(buffer->pages + vmf->pgoff);
	if (!buffer->pages[vmf->pgoff]) {
		mutex_unlock(&buffer->lock);
		WARN_ON(1);
		return VM_FAULT_ERROR;
	}

	pfn = page_to_pfn(ion_buffer_page(buffer->pages[vmf->pgoff]));
	ret = vm_insert_pfn(vma, (unsigned long)vmf->address, pfn);
	mutex_unlock(&buffer->lock);
	if (ret) {
		IONMSG("%s vm insert pfn fail,vma =0x%p,addr =0x%lx,pfn =%lu\n",
		       __func__, vma, vmf->address, pfn);
		return VM_FAULT_ERROR;
	}

	return VM_FAULT_NOPAGE;
}

static void ion_vm_open(struct vm_area_struct *vma)
{
	struct dma_buf *dmabuf = vma->vm_private_data;
	struct ion_buffer *buffer =  dmabuf->priv;
	struct ion_vma_list *vma_list;

	vma_list = kmalloc(sizeof(*vma_list), GFP_KERNEL);
	if (!vma_list) {
		IONMSG("%s kmalloc failed, vma_list is null.\n", __func__);
		return;
	}
	vma_list->vma = vma;
	mutex_lock(&buffer->lock);
	list_add(&vma_list->list, &buffer->vmas);
	mutex_unlock(&buffer->lock);
	pr_debug("%s: adding %pK\n", __func__, vma);
}

static void ion_vm_close(struct vm_area_struct *vma)
{
	struct dma_buf *dmabuf = vma->vm_private_data;
	struct ion_buffer *buffer =  dmabuf->priv;
	struct ion_vma_list *vma_list, *tmp;

	pr_debug("%s\n", __func__);
	mutex_lock(&buffer->lock);
	list_for_each_entry_safe(vma_list, tmp, &buffer->vmas, list) {
		if (vma_list->vma != vma)
			continue;
		list_del(&vma_list->list);
		kfree(vma_list);
		pr_debug("%s: deleting %pK\n", __func__, vma);
		break;
	}
	mutex_unlock(&buffer->lock);
}

static const struct vm_operations_struct ion_vma_ops = {
	.open = ion_vm_open,
	.close = ion_vm_close,
	.fault = ion_vm_fault,
};

static int ion_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = dmabuf->priv;
	int ret = 0;

	mmprofile_log_ex(ion_mmp_events[PROFILE_MAP_USER],
			 MMPROFILE_FLAG_START, buffer->size, vma->vm_start);

	if (!buffer->heap->ops->map_user) {
		pr_err("%s: heap does not define a method for map to native\n",
		       __func__);
		return -EINVAL;
	}

	vma->vm_private_data = dmabuf;
	if (ion_buffer_fault_user_mappings(buffer)) {
		vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND |
							VM_DONTDUMP;
		vma->vm_ops = &ion_vma_ops;
		ion_vm_open(vma);
		return 0;
	}

	if (!(buffer->flags & ION_FLAG_CACHED))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	mutex_lock(&buffer->lock);
	/* now map it to userspace */
	ret = buffer->heap->ops->map_user(buffer->heap, buffer, vma);
	mutex_unlock(&buffer->lock);

	if (ret)
		pr_err("%s: failure mapping buffer to userspace\n",
		       __func__);
	mmprofile_log_ex(ion_mmp_events[PROFILE_MAP_USER],
			 MMPROFILE_FLAG_END, buffer->size, vma->vm_start);

	return ret;
}

static void ion_dma_buf_release(struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer = dmabuf->priv;

	ion_buffer_put(buffer);
	mutex_lock(&dmabuf_list.lock);
	list_del(&dmabuf->node);
	mutex_unlock(&dmabuf_list.lock);
}

static void *ion_dma_buf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
	struct ion_buffer *buffer = dmabuf->priv;
	void *vaddr;

	if (!buffer->heap->ops->map_kernel) {
		pr_err("%s: map kernel is not implemented by this heap.\n",
		       __func__);
		return ERR_PTR(-ENOTTY);
	}
	mutex_lock(&buffer->lock);
	vaddr = ion_buffer_kmap_get(buffer);
	mutex_unlock(&buffer->lock);

	if (IS_ERR(vaddr))
		return vaddr;

	return vaddr + offset * PAGE_SIZE;
}

static void ion_dma_buf_kunmap(struct dma_buf *dmabuf, unsigned long offset,
			       void *ptr)
{
	struct ion_buffer *buffer = dmabuf->priv;

	if (buffer->heap->ops->map_kernel) {
		mutex_lock(&buffer->lock);
		ion_buffer_kmap_put(buffer);
		mutex_unlock(&buffer->lock);
	}
}

#ifdef MTK_ION_DMABUF_SUPPORT
static int ion_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	struct ion_dma_buf_attachment *a;

	if (ion_iommu_heap_type(buffer) ||
	    buffer->heap->type == (int)ION_HEAP_TYPE_SYSTEM) {
		IONDBG("%s iommu device, to cache sync\n", __func__);

		mutex_lock(&buffer->lock);
		list_for_each_entry(a, &buffer->attachments, list) {
			dma_sync_sg_for_cpu(a->dev,
					    a->table->sgl,
					    a->table->nents,
					    direction);
		}
		mutex_unlock(&buffer->lock);
	}

	return 0;// PTR_ERR_OR_ZERO(vaddr);
}

static int ion_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				      enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	struct ion_dma_buf_attachment *a;

	if (ion_iommu_heap_type(buffer) ||
	    buffer->heap->type == (int)ION_HEAP_TYPE_SYSTEM) {
		IONDBG("%s iommu device, to cache sync\n", __func__);

		mutex_lock(&buffer->lock);
		list_for_each_entry(a, &buffer->attachments, list) {
			dma_sync_sg_for_device(a->dev,
					       a->table->sgl,
					       a->table->nents,
					       direction);
		}
		mutex_unlock(&buffer->lock);
	}

	return 0;
}
#else
static int ion_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					enum dma_data_direction direction)
{
	return 0;
}

static int ion_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				      enum dma_data_direction direction)
{
	return 0;
}
#endif

static struct dma_buf_ops dma_buf_ops = {
	.map_dma_buf = ion_map_dma_buf,
	.unmap_dma_buf = ion_unmap_dma_buf,
	.mmap = ion_mmap,
	.release = ion_dma_buf_release,
#ifdef MTK_ION_DMABUF_SUPPORT
	.attach = ion_dma_buf_attach,
	.detach = ion_dma_buf_detatch,
#endif
	.begin_cpu_access = ion_dma_buf_begin_cpu_access,
	.end_cpu_access = ion_dma_buf_end_cpu_access,
	.map_atomic = ion_dma_buf_kmap,
	.unmap_atomic = ion_dma_buf_kunmap,
	.map = ion_dma_buf_kmap,
	.unmap = ion_dma_buf_kunmap,
};

static struct dma_buf *__ion_share_dma_buf(struct ion_client *client,
					   struct ion_handle *handle,
					   bool lock_client)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct ion_buffer *buffer;
	struct dma_buf *dmabuf;
	bool valid_handle;

	if (lock_client)
		mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to share.\n", __func__);
		if (lock_client)
			mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buffer = handle->buffer;
	ion_buffer_get(buffer);
	if (lock_client)
		mutex_unlock(&client->lock);

	exp_info.ops = &dma_buf_ops;
	exp_info.size = buffer->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buffer;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dma buf export failed dmabuf is error 0x%p.\n",
		       __func__, dmabuf);
		ion_buffer_put(buffer);
		return dmabuf;
	}

	mutex_lock(&dmabuf_list.lock);
	list_add(&dmabuf->node, &dmabuf_list.head);
	mutex_unlock(&dmabuf_list.lock);

	return dmabuf;
}

struct dma_buf *ion_share_dma_buf(struct ion_client *client,
				  struct ion_handle *handle)
{
	return __ion_share_dma_buf(client, handle, true);
}

EXPORT_SYMBOL(ion_share_dma_buf);

static int __ion_share_dma_buf_fd(struct ion_client *client,
				  struct ion_handle *handle, bool lock_client)
{
	struct dma_buf *dmabuf;
	int fd;

	mmprofile_log_ex(ion_mmp_events[PROFILE_SHARE], MMPROFILE_FLAG_START,
			 (unsigned long)client, (unsigned long)handle);
	dmabuf = __ion_share_dma_buf(client, handle, lock_client);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dmabuf is err 0x%lx.\n",
		       __func__, (unsigned long)dmabuf);
		return PTR_ERR(dmabuf);
	}

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	mmprofile_log_ex(ion_mmp_events[PROFILE_SHARE],
			 MMPROFILE_FLAG_END, (unsigned long)client, fd);
	if (fd < 0) {
		IONMSG("%s dma_buf_fd failed %d.\n", __func__, fd);
		dma_buf_put(dmabuf);
	}
	handle->dbg.fd = fd;
	return fd;
}

int ion_share_dma_buf_fd(struct ion_client *client, struct ion_handle *handle)
{
	return __ion_share_dma_buf_fd(client, handle, true);
}

EXPORT_SYMBOL(ion_share_dma_buf_fd);

int ion_share_dma_buf_fd_nolock(struct ion_client *client,
				struct ion_handle *handle)
{
	return __ion_share_dma_buf_fd(client, handle, false);
}

struct ion_handle *ion_import_dma_buf(struct ion_client *client,
				      struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer;
	struct ion_handle *handle;
	int ret;

	/* if this memory came from ion */

	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not import dmabuf from another exporter\n",
		       __func__);
		return ERR_PTR(-EINVAL);
	}
	buffer = dmabuf->priv;

	mutex_lock(&client->lock);
	/* if a handle exists for this buffer just take a reference to it */
	handle = ion_handle_lookup(client, buffer);
	if (!IS_ERR(handle)) {
		handle = ion_handle_get_check_overflow(handle);
		mutex_unlock(&client->lock);
		goto end;
	}

	handle = ion_handle_create(client, buffer);
	if (IS_ERR(handle)) {
		mutex_unlock(&client->lock);
		IONMSG("%s handle is error 0x%lx.\n",
		       __func__, (unsigned long)handle);
		goto end;
	}

	ret = ion_handle_add(client, handle);
	ion_client_buf_add(buffer->heap, client, buffer->size);
	mutex_unlock(&client->lock);
	if (ret) {
		ion_handle_put(handle);
		handle = ERR_PTR(ret);
		IONMSG("ion_import: ion_handle_add fail %d\n", ret);
	}

end:
	return handle;
}
EXPORT_SYMBOL(ion_import_dma_buf);

struct ion_handle *ion_import_dma_buf_fd(struct ion_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct ion_handle *handle;

	mmprofile_log_ex(ion_mmp_events[PROFILE_IMPORT], MMPROFILE_FLAG_START,
			 (unsigned long)client, fd);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dma_buf_get fail fd=%d ret=0x%lx\n",
		       __func__, fd, (unsigned long)dmabuf);
		return ERR_CAST(dmabuf);
	}

	handle = ion_import_dma_buf(client, dmabuf);
	dma_buf_put(dmabuf);

	mmprofile_log_ex(ion_mmp_events[PROFILE_IMPORT], MMPROFILE_FLAG_END,
			 (unsigned long)client,
			 (unsigned long)handle);
	if (!IS_ERR(handle)) {
		handle->dbg.fd = fd;
		handle->dbg.user_ts = sched_clock();
		do_div(handle->dbg.user_ts, 1000000);
	}
	return handle;
}
EXPORT_SYMBOL(ion_import_dma_buf_fd);

int ion_sync_for_device(struct ion_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		IONMSG("%s dma_buf_get failed dmabuf is err %d, 0x%lx.\n",
		       __func__, fd, (unsigned long)dmabuf);
		return PTR_ERR(dmabuf);
	}

	/* if this memory came from ion */
	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not sync dmabuf from another exporter\n",
		       __func__);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	buffer = dmabuf->priv;

#ifdef CONFIG_MTK_ION
#ifdef CONFIG_MTK_IOMMU_V2
	if (ion_iommu_heap_type(buffer) &&
	    buffer->heap->type != (int)ION_HEAP_TYPE_MULTIMEDIA_SEC)
		dma_sync_sg_for_device(g_iommu_device,
				       buffer->sg_table->sgl,
				       buffer->sg_table->nents,
				       DMA_BIDIRECTIONAL);
	else if (buffer->heap->type != (int)ION_HEAP_TYPE_FB  &&
		 buffer->heap->type != (int)ION_HEAP_TYPE_MULTIMEDIA_SEC)
		dma_sync_sg_for_device(g_ion_device->dev.this_device,
				       buffer->sg_table->sgl,
				       buffer->sg_table->nents,
				       DMA_BIDIRECTIONAL);
	else
		IONMSG("%s: can not support heap type(%d) id (%d) to sync\n",
		       __func__, buffer->heap->type, buffer->heap->id);
#else
	if (buffer->heap->type != (int)ION_HEAP_TYPE_FB &&
	    buffer->heap->type != (int)ION_HEAP_TYPE_MULTIMEDIA_SEC)
		dma_sync_sg_for_device(g_ion_device->dev.this_device,
				       buffer->sg_table->sgl,
				       buffer->sg_table->nents,
				       DMA_BIDIRECTIONAL);
	else
		IONMSG("%s: can not support heap type(%d) id (%d) to sync\n",
		       __func__, buffer->heap->type, buffer->heap->id);
#endif
#endif

	dma_buf_put(dmabuf);
	return 0;
}

int ion_query_heaps(struct ion_client *client, struct ion_heap_query *query)
{
	struct ion_device *dev = client->dev;
	struct ion_heap_data __user *buffer = u64_to_user_ptr(query->heaps);
	int ret = -EINVAL, cnt = 0, max_cnt;
	struct ion_heap *heap;
	struct ion_heap_data hdata;

	memset(&hdata, 0, sizeof(hdata));

	down_read(&dev->lock);
	if (!buffer) {
		query->cnt = dev->heap_cnt;
		ret = 0;
		goto out;
	}

	if (query->cnt <= 0)
		goto out;

	max_cnt = query->cnt;

	plist_for_each_entry(heap, &dev->heaps, node) {
		strncpy(hdata.name, heap->name, MAX_HEAP_NAME);
		hdata.name[sizeof(hdata.name) - 1] = '\0';
		hdata.type = heap->type;
		hdata.heap_id = heap->id;

		if (copy_to_user(&buffer[cnt], &hdata, sizeof(hdata))) {
			ret = -EFAULT;
			goto out;
		}

		cnt++;
		if (cnt >= max_cnt)
			break;
	}

	query->cnt = cnt;
out:
	up_read(&dev->lock);
	return ret;
}

static int ion_release(struct inode *inode, struct file *file)
{
	struct ion_client *client = file->private_data;

	pr_debug("%s: %d\n", __func__, __LINE__);
	ion_client_destroy(client);
	return 0;
}

static int ion_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ion_device *dev = container_of(miscdev, struct ion_device, dev);
	struct ion_client *client;
	char debug_name[64];
	unsigned long long start, end;

	pr_debug("%s: %d\n", __func__, __LINE__);
	snprintf(debug_name, 64, "%u", task_pid_nr(current->group_leader));
	start = sched_clock();
	client = ion_client_create(dev, debug_name);
	if (IS_ERR(client)) {
		IONMSG("%s ion client create failed 0x%lx.\n",
		       __func__, (unsigned long)client);
		return PTR_ERR(client);
	}
	file->private_data = client;
	end = sched_clock();

	if (end - start > 10000000ULL) {/* unit is ns */
		IONMSG("warn: ion open time: %lld ns\n", end - start);
	}

	return 0;
}

static const struct file_operations ion_fops = {
	.owner          = THIS_MODULE,
	.open           = ion_open,
	.release        = ion_release,
	.unlocked_ioctl = ion_ioctl,
	.compat_ioctl   = compat_ion_ioctl,
};

#if IS_ENABLED(CONFIG_DEBUG_FS) || IS_ENABLED(CONFIG_PROC_FS)
static size_t ion_debug_heap_total(struct ion_client *client,
					unsigned int id)
{
	size_t size = 0;
	struct rb_node *n;
	unsigned int heapid;
	enum ion_heap_type type;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle =
			rb_entry(n, struct ion_handle, node);
		heapid = handle->buffer->heap->id;
		type = handle->buffer->heap->type;
		if (heapid == id ||
			/* for exception dump ion_mm_heap info */
		    (id == ION_HEAP_TYPE_MULTIMEDIA &&
		     (type == ION_HEAP_TYPE_SYSTEM ||
		      type == ION_HEAP_TYPE_MULTIMEDIA ||
		      type == ION_HEAP_TYPE_MULTIMEDIA_SEC))) {
			client->dbg_hnd_cnt++;
			size += handle->buffer->size;
		}
	}
	mutex_unlock(&client->lock);
	return size;
}

static int ion_debug_heap_show(struct seq_file *s, void *unused)
{
	struct ion_heap *heap = s->private;
	struct ion_device *dev = heap->dev;
	struct rb_node *n;
	struct ion_heap *cam_heap = NULL;
	size_t total_size = 0;
	size_t camera_total_size = 0;
	size_t total_orphaned_size = 0;
	unsigned long long current_ts = 0;
	unsigned int cam_id = ION_HEAP_TYPE_MULTIMEDIA_FOR_CAMERA;

	seq_printf(s, "total sz[%llu]\n",
		   (unsigned long long)(4096 * atomic64_read(&page_sz_cnt)));
	seq_printf(s, "%16.s(%16.s) %16.s %16.s %16.s %16.s\n",
		   "client", "dbg_name", "pid",
		   "size(cnt)--size(cnt)", "address", "threshold");
	seq_puts(s, "----------------------------------------------------\n");

	down_read(&dev->lock);

	current_ts = sched_clock();
	do_div(current_ts, 1000000);
	seq_printf(s, "time 1 %lld ms\n", current_ts);

	for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
		struct ion_client *client = rb_entry(n, struct ion_client,
						     node);
		size_t size = ion_debug_heap_total(client, heap->id);
		u64 total = ion_client_buf_dump(heap, client);

		if (!size)
			continue;
		if (client->task) {
			char task_comm[TASK_COMM_LEN];

			get_task_comm(task_comm, client->task);
			seq_printf(s, "%16.s(%16.s) %16u %16zu(%d)--%llu(%d) 0x%p %llu\n",
				   task_comm,
				   (*client->dbg_name) ?
				   client->dbg_name :
				   client->name,
				   client->pid, size, client->dbg_hnd_cnt,
				   total, client->hnd_cnt, client,
				   client->threshold_size);
		} else {
			seq_printf(s, "%16.s(%16.s) %16u %16zu(%d)--%llu(%d) 0x%p %llu\n",
				   client->name, "from_kernel",
				   client->pid, size, client->dbg_hnd_cnt,
				   total, client->hnd_cnt, client,
				   client->threshold_size);
		}
		client->dbg_hnd_cnt = 0;
	}
	up_read(&dev->lock);

	seq_puts(s, "----------------------------------------------------\n");
	seq_puts(s, "orphaned allocation (info is from last known client):\n");
	mutex_lock(&dev->buffer_lock);

	current_ts = sched_clock();
	do_div(current_ts, 1000000);
	seq_printf(s, "time 2 %lld ms\n", current_ts);

	for (n = rb_first(&dev->buffers); n; n = rb_next(n)) {
		struct ion_buffer *buffer = rb_entry(n, struct ion_buffer,
						     node);

		if (buffer->heap->id != heap->id) {
			if (heap->id == ION_HEAP_TYPE_MULTIMEDIA &&
			    buffer->heap->id == cam_id) {
				cam_heap = buffer->heap;
				camera_total_size += buffer->size;
			} else {
				continue;
			}
		}
		total_size += buffer->size;
		if (!buffer->handle_count) {
			seq_printf(s, "0x%p %16.s %16u %16zu %d %d\n",
				   buffer,
				   buffer->task_comm, buffer->pid,
				   buffer->size, buffer->kmap_cnt,
				   atomic_read(
					&buffer->ref.refcount.refs));
			total_orphaned_size += buffer->size;
		}
	}
	mutex_unlock(&dev->buffer_lock);
	seq_puts(s, "----------------------------------------------------\n");
	seq_printf(s, "%16.s %16zu\n", "total orphaned",
		   total_orphaned_size);
	seq_printf(s, "%16.s %16zu\n", "total ", total_size);
	if (heap->id == ION_HEAP_TYPE_MULTIMEDIA)
		seq_printf(s, "%16.s %16zu\n", "cam total",
			   camera_total_size);
	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		seq_printf(s, "%16.s %u %16zu\n", "defer free heap_id",
			   heap->id, heap->free_list_size);
	if (cam_heap) {
		if (cam_heap->flags & ION_HEAP_FLAG_DEFER_FREE)
			seq_printf(s, "%16.s %u %16zu\n",
				   "cam heap deferred free heap_id",
				   cam_heap->id, cam_heap->free_list_size);
	}
	seq_puts(s, "----------------------------------------------------\n");

	if (heap->debug_show)
		heap->debug_show(heap, s, unused);

	return 0;
}

static int debug_shrink_set(void *data, u64 val)
{
	struct ion_heap *heap = data;
	struct shrink_control sc;
	int objs;

	sc.gfp_mask = GFP_HIGHUSER;
	sc.nr_to_scan = val;

	if (!val) {
		objs = heap->shrinker.count_objects(&heap->shrinker, &sc);
		sc.nr_to_scan = objs;
	}

	heap->shrinker.scan_objects(&heap->shrinker, &sc);
	return 0;
}

static int debug_shrink_get(void *data, u64 *val)
{
	struct ion_heap *heap = data;
	struct shrink_control sc;
	int objs;

	sc.gfp_mask = GFP_HIGHUSER;
	sc.nr_to_scan = 0;

	objs = heap->shrinker.count_objects(&heap->shrinker, &sc);
	*val = objs;
	return 0;
}

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int ion_debug_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_heap_show, inode->i_private);
}

static const struct file_operations debug_heap_fops = {
	.open = ion_debug_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

DEFINE_SIMPLE_ATTRIBUTE(debug_shrink_fops, debug_shrink_get,
			debug_shrink_set, "%llu\n");
#endif

#if IS_ENABLED(CONFIG_PROC_FS)
static int ion_proc_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_heap_show, PDE_DATA(inode));
}

static const struct file_operations proc_heap_fops = {
	.open = ion_proc_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int proc_shrink_fops_open(struct inode *inode, struct file *file)
{
	struct inode local_inode = *inode;

	local_inode.i_private = PDE_DATA(inode);

	return simple_attr_open(&local_inode, file, debug_shrink_get,
				debug_shrink_set, "%llu\n");
}

static const struct file_operations proc_shrink_fops = {
	.owner	 = THIS_MODULE,
	.open	 = proc_shrink_fops_open,
	.release = simple_attr_release,
	.read	 = simple_attr_read,
	.write	 = simple_attr_write,
	.llseek	 = generic_file_llseek,
};
#endif
#endif

void ion_device_add_heap(struct ion_device *dev, struct ion_heap *heap)
{
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry *debug_file;
#endif
#if IS_ENABLED(CONFIG_PROC_FS)
	struct proc_dir_entry *proc_file;
#endif

	if (!heap->ops->allocate || !heap->ops->free)
		pr_err("%s: can not add heap with invalid ops struct.\n",
		       __func__);

	spin_lock_init(&heap->free_lock);
	heap->free_list_size = 0;

	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_init_deferred_free(heap);

	if ((heap->flags & ION_HEAP_FLAG_DEFER_FREE) || heap->ops->shrink)
		ion_heap_init_shrinker(heap);

	heap->dev = dev;
	down_write(&dev->lock);
	/*
	 * use negative heap->id to reverse the priority -- when traversing
	 * the list later attempt higher id numbers first
	 */
	plist_node_init(&heap->node, -heap->id);
	plist_add(&heap->node, &dev->heaps);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	debug_file = debugfs_create_file(heap->name, 0664,
					 dev->heaps_debug_root, heap,
					 &debug_heap_fops);

	if (!debug_file) {
		char buf[256], *path;

		path = dentry_path(dev->heaps_debug_root, buf, 256);
		pr_err("Failed to create heap debugfs at %s/%s\n",
		       path, heap->name);
	}

	if (heap->shrinker.count_objects && heap->shrinker.scan_objects) {
		char debug_name[64];

		snprintf(debug_name, 64, "%s_shrink", heap->name);
		debug_file = debugfs_create_file(
			debug_name, 0644, dev->heaps_debug_root, heap,
			&debug_shrink_fops);
		if (!debug_file) {
			char buf[256], *path;

			path = dentry_path(dev->heaps_debug_root, buf, 256);
			pr_err("Failed to create heap shrinker debugfs at %s/%s\n",
			       path, debug_name);
		}
	}
#endif

#if IS_ENABLED(CONFIG_PROC_FS)
	proc_file = proc_create_data(heap->name,
				     S_IFREG | 0644,
				     dev->heaps_proc_root,
				     &proc_heap_fops,
				     heap);

	if (!proc_file)
		IONMSG("Failed to create heap procfs at /proc/ion/heaps/%s\n",
		       heap->name);

	if (heap->shrinker.count_objects && heap->shrinker.scan_objects) {
		char debug_name[64];
		int name_length = 0;

		name_length = snprintf(debug_name, 64, "%s_shrink", heap->name);
		if (name_length <= 0)
			IONMSG("%s set debug name error, heap %s\n",
			       __func__, heap->name);
		proc_file = proc_create_data(debug_name,
					     S_IFREG | 0644,
					     dev->heaps_proc_root,
					     &proc_shrink_fops,
					     heap);
		if (!proc_file)
			IONMSG(
			       "Failed to create heap shrinker procfs at /proc/ion/heaps/%s\n",
			       debug_name);
	}
#endif

	dev->heap_cnt++;
	up_write(&dev->lock);
}
EXPORT_SYMBOL(ion_device_add_heap);

static ssize_t
total_heaps_kb_show(struct kobject *kobj, struct kobj_attribute *attr,
		    char *buf)
{
	u64 size_in_bytes = atomic_long_read(&total_heap_bytes);

	return sprintf(buf, "%llu\n", div_u64(size_in_bytes, 1024));
}

static ssize_t
total_pools_kb_show(struct kobject *kobj, struct kobj_attribute *attr,
		    char *buf)
{
	u64 size_in_bytes = ion_page_pool_nr_pages() * PAGE_SIZE;

	return sprintf(buf, "%llu\n", div_u64(size_in_bytes, 1024));
}

static struct kobj_attribute total_heaps_kb_attr =
	__ATTR_RO(total_heaps_kb);

static struct kobj_attribute total_pools_kb_attr =
	__ATTR_RO(total_pools_kb);

static struct attribute *ion_device_attrs[] = {
	&total_heaps_kb_attr.attr,
	&total_pools_kb_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(ion_device);

static int ion_init_sysfs(void)
{
	struct kobject *ion_kobj;
	int ret;

	ion_kobj = kobject_create_and_add("ion", kernel_kobj);
	if (!ion_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(ion_kobj, ion_device_groups);
	if (ret) {
		kobject_put(ion_kobj);
		return ret;
	}

	return 0;
}

struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg))
{
	struct ion_device *idev;
	int ret;

	idev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return ERR_PTR(-ENOMEM);

	idev->dev.minor = MISC_DYNAMIC_MINOR;
	idev->dev.name = "ion";
	idev->dev.fops = &ion_fops;
	idev->dev.parent = NULL;
	ret = misc_register(&idev->dev);
	if (ret) {
		pr_err("ion: failed to register misc device.\n");
		goto err_reg;
	}

	ret = ion_init_sysfs();
	if (ret) {
		pr_err("ion: failed to add sysfs attributes.\n");
		goto err_sysfs;
	}

#if IS_ENABLED(CONFIG_DEBUG_FS)
	idev->debug_root = debugfs_create_dir("ion", NULL);
	if (!idev->debug_root) {
		pr_err("ion: failed to create debugfs root directory.\n");
		goto debugfs_done;
	}
	idev->heaps_debug_root = debugfs_create_dir("heaps", idev->debug_root);
	if (!idev->heaps_debug_root) {
		pr_err("ion: failed to create debugfs heaps directory.\n");
		goto debugfs_done;
	}
	idev->clients_debug_root = debugfs_create_dir(
		"clients", idev->debug_root);
	if (!idev->clients_debug_root)
		pr_err("ion: failed to create debugfs clients directory.\n");

debugfs_done:
#endif

#if IS_ENABLED(CONFIG_PROC_FS)
	idev->proc_root = proc_mkdir("ion", NULL);
	if (!idev->proc_root) {
		IONMSG("ion: failed to create procfs root directory.\n");
		goto procfs_done;
	}
	idev->heaps_proc_root = proc_mkdir("heaps", idev->proc_root);
	if (!idev->heaps_proc_root) {
		IONMSG("ion: failed to create procfs heaps directory.\n");
		goto procfs_done;
	}
	idev->clients_proc_root = proc_mkdir("clients", idev->proc_root);
	if (!idev->clients_proc_root)
		IONMSG("ion: failed to create procfs clients directory.\n");

procfs_done:
#endif
	idev->custom_ioctl = custom_ioctl;
	idev->buffers = RB_ROOT;
	mutex_init(&idev->buffer_lock);
	init_rwsem(&idev->lock);
	plist_head_init(&idev->heaps);
	idev->clients = RB_ROOT;
	return idev;

err_sysfs:
	misc_deregister(&idev->dev);
err_reg:
	kfree(idev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(ion_device_create);

void ion_device_destroy(struct ion_device *dev)
{
	misc_deregister(&dev->dev);
#if IS_ENABLED(CONFIG_DEBUG_FS)
	debugfs_remove_recursive(dev->debug_root);
#endif
#if IS_ENABLED(CONFIG_PROC_FS)
	proc_remove(dev->proc_root);
#endif
	/* XXX need to free the heaps and clients ? */
	kfree(dev);
}
EXPORT_SYMBOL(ion_device_destroy);

int ion_phys(struct ion_client *client, struct ion_handle *handle,
	     ion_phys_addr_t *addr, size_t *len)
{
	struct ion_buffer *buffer;
	int ret;

	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_GET_PHYS], MMPROFILE_FLAG_START,
			 (unsigned long)client, (unsigned long)handle);
	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		mutex_unlock(&client->lock);
		IONMSG("ion: %s invalid handle pass to phys.\n", __func__);
		return -EINVAL;
	}

	buffer = handle->buffer;

	if (!buffer->heap->ops->phys) {
		IONMSG(
			"%s is not impl by heap (name=%s, type=%d).\n",
		       __func__, buffer->heap->name, buffer->heap->type);
		mutex_unlock(&client->lock);
		return -ENODEV;
	}
	mutex_unlock(&client->lock);
	mutex_lock(&buffer->lock);
	ret = buffer->heap->ops->phys(buffer->heap, buffer, addr, len);
	mutex_unlock(&buffer->lock);
	if (ret == -EDOM)
		aee_kernel_warning_api(__FILE__, __LINE__,
				       DB_OPT_DEFAULT |
				       DB_OPT_NATIVE_BACKTRACE,
				       "ion phys failed",
				       "dump user backtrace");
	/*avoid camelcase, will modify in a letter*/
	mmprofile_log_ex(ion_mmp_events[PROFILE_GET_PHYS], MMPROFILE_FLAG_END,
			 buffer->size, (unsigned long)*addr);
	return ret;
}
EXPORT_SYMBOL(ion_phys);

/* ===================================== */
/* helper functions */
/* ===================================== */
struct ion_buffer *ion_handle_buffer(struct ion_handle *handle)
{
	return handle->buffer;
}

struct ion_handle *ion_drv_get_handle(struct ion_client *client,
				      int user_handle,
				      struct ion_handle *kernel_handle,
				      int from_kernel)
{
	struct ion_handle *handle;

	if (from_kernel) {
		handle = kernel_handle;

		if (IS_ERR_OR_NULL(handle)) {
			IONMSG("%s handle invalid, handle = 0x%lx.\n",
			       __func__,
			       (unsigned long)handle);
			return ERR_PTR(-EINVAL);
		}

		mutex_lock(&client->lock);
		if (!ion_handle_validate(client, handle)) {
			IONMSG("%s handle invalid, handle=0x%lx\n",
			       __func__,
			       (unsigned long)handle);
			mutex_unlock(&client->lock);
			return ERR_PTR(-EINVAL);
		}
		ion_handle_get(handle);
		mutex_unlock(&client->lock);
	} else {
		mutex_lock(&client->lock);
		handle = ion_handle_get_by_id_nolock(client, user_handle);
		if (IS_ERR_OR_NULL(handle)) {
			IONMSG("%s handle invalid, handle_id=%d\n",
			       __func__,
			       user_handle);
			mutex_unlock(&client->lock);
			return ERR_PTR(-EINVAL);
		}
		mutex_unlock(&client->lock);
	}
	return handle;
}

int ion_drv_put_kernel_handle(void *kernel_handle)
{
	return ion_handle_put(kernel_handle);
}

struct ion_heap *ion_drv_get_heap(struct ion_device *dev,
				  int heap_id,
				  int need_lock)
{
	struct ion_heap *_heap, *heap = NULL, *tmp;

	if (need_lock) {
		if (!down_read_trylock(&dev->lock)) {
			IONMSG("%s get lock fail\n", __func__);
			return NULL;
		}
	}

	plist_for_each_entry_safe(_heap, tmp, &dev->heaps, node) {
		if (_heap->id == heap_id) {
			heap = _heap;
			break;
		}
	}

	if (need_lock)
		up_read(&dev->lock);

	return heap;
}

struct ion_buffer *ion_drv_file_to_buffer(struct file *file)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer = NULL;
	const char *pathname = NULL;

	if (!file)
		goto file2buf_exit;
	if (!(file->f_path.dentry))
		goto file2buf_exit;

	pathname = file->f_path.dentry->d_name.name;
	if (!pathname)
		goto file2buf_exit;

	if (strstr(pathname, "dmabuf")) {
		dmabuf = file->private_data;
		if (!dmabuf) {
			IONMSG("%s warnning, dmabuf is NULL\n", __func__);
			goto file2buf_exit;
		}
		if (dmabuf->ops == &dma_buf_ops)
			buffer = dmabuf->priv;
	}

file2buf_exit:
	if (buffer)
		return buffer;
	else
		return ERR_PTR(-EINVAL);
}

/* ===================================== */
