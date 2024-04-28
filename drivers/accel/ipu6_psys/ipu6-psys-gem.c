// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2024 Intel Corporation

#include <linux/dma-buf.h>
#include <linux/highmem.h>
#include <linux/module.h>

#include <drm/drm_device.h>
#include <drm/drm_gem.h>

#include "ipu6-psys-gem.h"
#include "ipu6-psys.h"

struct ipu6_psys_kbuffer *ipu6_psys_bo_alloc(struct ipu6_psys *psys,
					     struct ipu6_psys_buffer *buf)
{
	struct drm_gem_shmem_object *shmem;
	struct ipu6_psys_kbuffer *kbuf;

	shmem = drm_gem_shmem_create(&psys->drm_dev, buf->len);
	if (IS_ERR(shmem))
		return ERR_CAST(shmem);

	kbuf = to_psys_kbuf(&shmem->base);
	kbuf->len = PAGE_ALIGN(buf->len);
	kbuf->flags = buf->flags;

	drm_gem_object_put(&shmem->base);

	return kbuf;
}

int ipu6_psys_mapbuf_locked(struct drm_file *file, struct ipu6_psys *psys,
			    struct ipu6_psys_map *map)
{
	struct device *dev = &psys->adev->auxdev.dev;
	struct dma_buf_attachment *db_attach;
	struct ipu6_psys_kbuffer *kbuf;
	struct drm_gem_object *obj;
	struct dma_buf *dbuf;
	struct sg_table *sgt;
	struct iosys_map dmap;
	int ret;

	dev_dbg(dev, "psys mapbuf handle %u\n", map->handle);
	obj = drm_gem_object_lookup(file, map->handle);
	if (!obj)
		return -EINVAL;

	kbuf = to_psys_kbuf(obj);
	if (!kbuf) {
		ret = -EINVAL;
		goto map_fail_obj_put;
	}

	dbuf = dma_buf_get(map->fd);
	if (IS_ERR(dbuf)) {
		ret = -EINVAL;
		goto map_fail_obj_put;
	}

	db_attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(db_attach)) {
		ret = PTR_ERR(db_attach);
		dev_dbg(dev, "dma buf attach failed\n");
		goto map_fail_put;
	}

	sgt = dma_buf_map_attachment_unlocked(db_attach,
					      DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto map_fail_detach;
	}

	ret = dma_buf_vmap_unlocked(dbuf, &dmap);
	if (ret) {
		dev_dbg(dev, "dma buf vmap failed\n");
		goto map_fail_unmap;
	}

	kbuf->dbuf = dbuf;
	kbuf->db_attach = db_attach;
	kbuf->sgt = sgt;
	kbuf->vaddr = dmap.vaddr;
	kbuf->dma_addr = sg_dma_address(sgt->sgl);
	kbuf->dma_dir = DMA_BIDIRECTIONAL;
	kbuf->valid = true;

	drm_gem_object_put(obj);

	return 0;

map_fail_unmap:
	dma_buf_unmap_attachment_unlocked(db_attach, sgt, DMA_BIDIRECTIONAL);
map_fail_detach:
	dma_buf_detach(dbuf, db_attach);
map_fail_put:
	dma_buf_put(dbuf);
map_fail_obj_put:
	drm_gem_object_put(obj);

	return ret;
}

int ipu6_psys_unmapbuf_locked(struct drm_file *file, struct ipu6_psys *psys,
			      struct ipu6_psys_map *map)
{
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_kbuffer *kbuf;
	struct drm_gem_object *obj;

	dev_dbg(dev, "psys unmapbuf handle %u\n", map->handle);

	obj = drm_gem_object_lookup(file, map->handle);
	if (!obj)
		return -EINVAL;

	kbuf = to_psys_kbuf(obj);
	if (!kbuf)
		return -EINVAL;

	if (kbuf->vaddr) {
		struct iosys_map dmap;

		iosys_map_set_vaddr(&dmap, kbuf->vaddr);
		dma_buf_vunmap_unlocked(kbuf->dbuf, &dmap);
	}

	if (kbuf->sgt)
		dma_buf_unmap_attachment_unlocked(kbuf->db_attach, kbuf->sgt,
						  kbuf->dma_dir);
	if (kbuf->db_attach)
		dma_buf_detach(kbuf->dbuf, kbuf->db_attach);

	dma_buf_put(kbuf->dbuf);

	kbuf->db_attach = NULL;
	kbuf->dbuf = NULL;
	kbuf->sgt = NULL;
	kbuf->valid = false;

	return 0;
}

static int ipu6_psys_dmabuf_attach(struct dma_buf *dbuf,
				   struct dma_buf_attachment *attach)
{
	struct ipu6_psys_kbuffer *kbuf = dbuf->priv;
	struct ipu6_psys_dma_buf_attach *ipu6_attach;
	struct sg_table *sgt;
	int ret;

	ipu6_attach = kzalloc(sizeof(*ipu6_attach), GFP_KERNEL);
	if (!ipu6_attach)
		return -ENOMEM;

	ret = drm_gem_shmem_pin(&kbuf->shmem_bo);
	if (ret)
		return ret;

	sgt = drm_gem_shmem_get_pages_sgt(&kbuf->shmem_bo);
	if (IS_ERR(sgt)) {
		kfree(ipu6_attach);
		return PTR_ERR(sgt);
	}

	ipu6_attach->sgt = sgt;
	ipu6_attach->len = kbuf->len;
	attach->priv = ipu6_attach;

	return 0;
}

static void ipu6_psys_dmabuf_detach(struct dma_buf *dbuf,
				    struct dma_buf_attachment *attach)
{
	struct ipu6_psys_dma_buf_attach *ipu6_attach = attach->priv;
	struct ipu6_psys_kbuffer *kbuf = dbuf->priv;

	drm_gem_shmem_put_pages(&kbuf->shmem_bo);
	drm_gem_shmem_unpin(&kbuf->shmem_bo);

	kfree(ipu6_attach);
	attach->priv = NULL;
}

static struct sg_table *ipu6_psys_dmabuf_map(struct dma_buf_attachment *attach,
					     enum dma_data_direction dir)
{
	struct ipu6_psys_dma_buf_attach *ipu6_attach = attach->priv;
	unsigned long attrs;
	int ret;

	attrs = DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sgtable(attach->dev, ipu6_attach->sgt, dir, attrs);
	if (ret < 0) {
		dev_dbg(attach->dev, "buf map failed\n");

		return ERR_PTR(-EIO);
	}

	dma_sync_sg_for_device(attach->dev, ipu6_attach->sgt->sgl,
			       ipu6_attach->sgt->orig_nents, DMA_BIDIRECTIONAL);

	return ipu6_attach->sgt;
}

static void ipu6_psys_dmabuf_unmap(struct dma_buf_attachment *attach,
				   struct sg_table *sgt,
				   enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
}

static int ipu6_psys_dmabuf_mmap(struct dma_buf *dbuf,
				 struct vm_area_struct *vma)
{
	return -ENOTTY;
}

static void ipu6_psys_dmabuf_release(struct dma_buf *dbuf)
{
	struct ipu6_psys_kbuffer *kbuf = dbuf->priv;

	if (!kbuf)
		return;

	if (kbuf->db_attach) {
		dev_dbg(kbuf->db_attach->dev,
			"releasing buffer %d\n", kbuf->fd);
		drm_gem_shmem_unpin(&kbuf->shmem_bo);
	}

	kfree(kbuf);
}

static int ipu6_psys_dmabuf_begin_cpu_access(struct dma_buf *dbuf,
					     enum dma_data_direction dir)
{
	return -ENOTTY;
}

static int ipu6_psys_dmabuf_vmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct ipu6_psys_kbuffer *kbuf = dbuf->priv;
	struct dma_buf_attachment *attach;
	struct ipu6_psys_dma_buf_attach *ipu6_attach;
	int ret;

	if (list_empty(&dbuf->attachments))
		return -EINVAL;

	attach = list_last_entry(&dbuf->attachments,
				 struct dma_buf_attachment, node);
	ipu6_attach = attach->priv;

	if (!ipu6_attach || !ipu6_attach->sgt)
		return -EINVAL;

	dma_resv_lock(kbuf->shmem_bo.base.resv, NULL);
	ret = drm_gem_shmem_vmap(&kbuf->shmem_bo, map);
	dma_resv_unlock(kbuf->shmem_bo.base.resv);

	return ret;
}

static void ipu6_psys_dmabuf_vunmap(struct dma_buf *dbuf,
				    struct iosys_map *map)
{
	struct ipu6_psys_kbuffer *kbuf = dbuf->priv;
	struct dma_buf_attachment *attach;
	struct ipu6_psys_dma_buf_attach *ipu6_attach;

	if (WARN_ON(list_empty(&dbuf->attachments)))
		return;

	attach = list_last_entry(&dbuf->attachments,
				 struct dma_buf_attachment, node);
	ipu6_attach = attach->priv;

	if (WARN_ON(!ipu6_attach))
		return;

	dma_resv_lock(kbuf->shmem_bo.base.resv, NULL);
	drm_gem_shmem_vunmap(&kbuf->shmem_bo, map);
	dma_resv_unlock(kbuf->shmem_bo.base.resv);
}

static const struct dma_buf_ops ipu6_psys_dmabuf_ops = {
	.attach = ipu6_psys_dmabuf_attach,
	.detach = ipu6_psys_dmabuf_detach,
	.map_dma_buf = ipu6_psys_dmabuf_map,
	.unmap_dma_buf = ipu6_psys_dmabuf_unmap,
	.release = ipu6_psys_dmabuf_release,
	.begin_cpu_access = ipu6_psys_dmabuf_begin_cpu_access,
	.mmap = ipu6_psys_dmabuf_mmap,
	.vmap = ipu6_psys_dmabuf_vmap,
	.vunmap = ipu6_psys_dmabuf_vunmap,
};

static struct dma_buf *ipu6_psys_gem_prime_export(struct drm_gem_object *obj,
						  int flags)
{
	struct ipu6_psys_kbuffer *kbuf = to_psys_kbuf(obj);
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &ipu6_psys_dmabuf_ops;
	exp_info.size = obj->size;
	exp_info.flags = flags;
	exp_info.priv = kbuf;
	exp_info.resv = obj->resv;

	return drm_gem_dmabuf_export(obj->dev, &exp_info);
}

static void ipu6_psys_free_object(struct drm_gem_object *obj)
{
	struct ipu6_psys_kbuffer *kbuf = to_psys_kbuf(obj);

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, NULL);

	mutex_destroy(&kbuf->lock);
	drm_gem_shmem_free(&kbuf->shmem_bo);
	kfree(kbuf);
}

static const struct drm_gem_object_funcs ipu6_psys_gem_obj_funcs = {
	.free = ipu6_psys_free_object,
	.print_info = drm_gem_shmem_object_print_info,
	.export = ipu6_psys_gem_prime_export,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

struct drm_gem_object *ipu6_psys_gem_create_object(struct drm_device *drm_dev,
						   size_t size)
{
	struct ipu6_psys_kbuffer *kbuf;

	if (!size || PAGE_ALIGNED(size))
		return ERR_PTR(-EINVAL);

	kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
	if (!kbuf)
		return NULL;

	kbuf->shmem_bo.base.funcs = &ipu6_psys_gem_obj_funcs;
	kbuf->shmem_bo.pages_mark_dirty_on_put = true;

	INIT_LIST_HEAD(&kbuf->node);
	mutex_init(&kbuf->lock);

	return &kbuf->shmem_bo.base;
}

struct drm_gem_object *ipu6_psys_gem_prime_import(struct drm_device *drm_dev,
						  struct dma_buf *dma_buf)
{
	return drm_gem_prime_import(drm_dev, dma_buf);
}

struct drm_gem_object *
ipu6_psys_gem_prime_import_sg_table(struct drm_device *drm_dev,
				    struct dma_buf_attachment *attach,
				    struct sg_table *sgt)
{
	struct ipu6_psys *psys = to_ipu6_psys(drm_dev);
	struct device *dma_dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_kbuffer *kbuf;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_shmem_prime_import_sg_table(drm_dev, attach, sgt);
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	kbuf = to_psys_kbuf(obj);
	if (IS_ERR(kbuf))
		return ERR_CAST(obj);

	ret = dma_map_sgtable(dma_dev, sgt, DMA_BIDIRECTIONAL,
			      DMA_ATTR_SKIP_CPU_SYNC);
	if (ret < 0)
		return ERR_PTR(ret);

	dma_sync_sg_for_device(dma_dev, sgt->sgl,
			       sgt->orig_nents, DMA_BIDIRECTIONAL);

	kbuf->dma_addr = sg_dma_address(sgt->sgl);

	return obj;
}
