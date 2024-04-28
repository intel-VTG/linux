/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2024 Intel Corporation
 */
#ifndef __IPU6_PSYS_GEM_H__
#define __IPU6_PSYS_GEM_H__

#include <linux/dma-buf.h>

#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_mm.h>

#include <uapi/drm/ipu6_psys_accel.h>
#include "ipu6-psys.h"

struct ipu6_psys_kbuffer {
	u64 len;
	u32 flags;
	int fd;
	u32 handle;
	struct drm_gem_shmem_object shmem_bo;
	struct mutex lock; /* protect kbuf */
	struct list_head node;
	dma_addr_t dma_addr;
	enum dma_data_direction dma_dir;
	struct sg_table *sgt;
	struct dma_buf_attachment *db_attach;
	struct dma_buf *dbuf;
	bool valid;	/* true when buffer is ready */
	void *vaddr;
};

struct ipu6_psys_dma_buf_attach {
	struct device *dev;
	u64 len;
	struct sg_table *sgt;
};

static inline struct ipu6_psys_kbuffer *to_psys_kbuf(struct drm_gem_object *obj)
{
	return container_of(obj, struct ipu6_psys_kbuffer, shmem_bo.base);
}

struct drm_gem_object *ipu6_psys_gem_create_object(struct drm_device *drm_dev,
						   size_t size);
struct drm_gem_object *ipu6_psys_gem_prime_import(struct drm_device *drm_dev,
						  struct dma_buf *dma_buf);
struct ipu6_psys_kbuffer *ipu6_psys_bo_alloc(struct ipu6_psys *psys,
					     struct ipu6_psys_buffer *buf);
int ipu6_psys_mapbuf_locked(struct drm_file *file, struct ipu6_psys *psys,
			    struct ipu6_psys_map *map);
int ipu6_psys_unmapbuf_locked(struct drm_file *file, struct ipu6_psys *psys,
			      struct ipu6_psys_map *map);
struct drm_gem_object *
ipu6_psys_gem_prime_import_sg_table(struct drm_device *dev,
				    struct dma_buf_attachment *attach,
				    struct sg_table *sgt);
#endif /* __IPU6_PSYS_GEM_H__ */
