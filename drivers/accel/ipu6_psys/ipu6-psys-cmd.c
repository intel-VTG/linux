// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2024 Intel Corporation

#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_mm.h>
#include <drm/drm_utils.h>

#include <linux/atomic.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/dma-resv.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/ww_mutex.h>

#include <uapi/drm/ipu6_psys_accel.h>

#include <media/ipu6/ipu6.h>
#include "ipu6-psys.h"
#include "ipu6-psys-cmd.h"

struct ipu6_psys_fence {
	struct dma_fence base;
	spinlock_t lock; /* protect fence */
	struct ipu6_psys *psys;
};

static struct ipu6_psys_kcmd *
ipu6_psys_ppg_get_kcmd(struct ipu6_psys_ppg *kppg,
		       enum ipu6_psys_cmd_state state)
{
	struct ipu6_psys_kcmd *kcmd;

	if (list_empty(&kppg->kcmds_new))
		return NULL;

	list_for_each_entry(kcmd, &kppg->kcmds_new, list) {
		if (kcmd->state == state)
			return kcmd;
	}

	return NULL;
}

static struct ipu6_psys_buffer_set *__get_buf_set(struct ipu6_psys_fh *fh,
						  size_t buf_set_size)
{
	struct device *dev = &fh->psys->adev->auxdev.dev;
	struct ipu6_psys_buffer_set *kbuf_set;
	struct ipu6_psys_cmdq *cmdq = &fh->cmdq;

	mutex_lock(&cmdq->bs_mutex);
	list_for_each_entry(kbuf_set, &cmdq->buf_sets, list) {
		if (!kbuf_set->buf_set_size &&
		    kbuf_set->size >= buf_set_size) {
			kbuf_set->buf_set_size = buf_set_size;
			mutex_unlock(&cmdq->bs_mutex);
			return kbuf_set;
		}
	}

	mutex_unlock(&cmdq->bs_mutex);

	kbuf_set = kzalloc(sizeof(*kbuf_set), GFP_KERNEL);
	if (!kbuf_set)
		return NULL;

	kbuf_set->vaddr = dma_alloc_attrs(dev, buf_set_size,
					  &kbuf_set->dma_addr, GFP_KERNEL, 0);
	if (!kbuf_set->vaddr) {
		kfree(kbuf_set);
		return NULL;
	}

	kbuf_set->buf_set_size = buf_set_size;
	kbuf_set->size = buf_set_size;
	mutex_lock(&cmdq->bs_mutex);
	list_add(&kbuf_set->list, &cmdq->buf_sets);
	mutex_unlock(&cmdq->bs_mutex);

	return kbuf_set;
}

static struct ipu6_psys_buffer_set *
ipu6_psys_create_buffer_set(struct ipu6_psys_kcmd *kcmd,
			    struct ipu6_psys_ppg *kppg)
{
	struct ipu6_psys_fh *fh = kcmd->fh;
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_buffer_set *kbuf_set;
	size_t buf_set_size;
	u32 *keb;

	buf_set_size = ipu6_psys_fw_ppg_get_buffer_set_size(kcmd);

	kbuf_set = __get_buf_set(fh, buf_set_size);
	if (!kbuf_set) {
		dev_err(dev, "failed to create buffer set\n");
		return NULL;
	}

	kbuf_set->buf_set =
		ipu6_psys_fw_ppg_create_buffer_set(kcmd, kbuf_set->vaddr, 0);

	ipu6_psys_fw_ppg_buffer_set_vaddress(kbuf_set->buf_set,
					     kbuf_set->dma_addr);
	keb = kcmd->kernel_enable_bitmap;
	ipu6_psys_fw_ppg_buffer_set_set_keb(kbuf_set->buf_set, keb);

	return kbuf_set;
}

static int ipu6_psys_ppg_get_bufset(struct ipu6_psys_kcmd *kcmd,
				    struct ipu6_psys_ppg *kppg)
{
	struct ipu6_psys *psys = kppg->fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_buffer_set *kbuf_set;
	unsigned int i;
	int ret;

	kbuf_set = ipu6_psys_create_buffer_set(kcmd, kppg);
	if (!kbuf_set) {
		ret = -EINVAL;
		goto error;
	}
	kcmd->kbuf_set = kbuf_set;
	kbuf_set->kcmd = kcmd;

	for (i = 0; i < kcmd->nbuffers; i++) {
		struct ipu6_psys_fw_terminal *terminal;
		u32 buffer;

		terminal = ipu6_psys_fw_pg_get_terminal(kcmd, i);
		if (!terminal)
			continue;

		buffer = (u32)kcmd->kbufs[i]->dma_addr +
			kcmd->buffers[i].data_offset;

		ret = ipu6_psys_fw_ppg_set_buffer_set(kcmd, terminal, i,
						      buffer);
		if (ret) {
			dev_err(dev, "Unable to set bufset\n");
			goto error;
		}
	}

	return 0;

error:
	dev_err(dev, "failed to get buffer set\n");
	return ret;
}

static struct ipu6_psys_ppg *
ipu6_psys_identify_kppg(struct ipu6_psys_kcmd *kcmd)
{
	struct ipu6_psys_cmdq *cmdq = &kcmd->fh->cmdq;
	struct ipu6_psys_ppg *kppg, *tmp;

	mutex_lock(&kcmd->fh->mutex);
	if (list_empty(&cmdq->ppgs))
		goto not_found;

	list_for_each_entry_safe(kppg, tmp, &cmdq->ppgs, list) {
		if (ipu6_psys_fw_pg_get_token(kcmd)
		    != kppg->token)
			continue;

		mutex_unlock(&kcmd->fh->mutex);
		return kppg;
	}

not_found:
	mutex_unlock(&kcmd->fh->mutex);
	return NULL;
}

static struct ipu6_psys_pg *ipu6_psys_get_pg_buf(struct ipu6_psys *psys,
						 size_t pg_size)
{
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_pg *kpg;
	unsigned long flags;

	spin_lock_irqsave(&psys->pgs_lock, flags);
	list_for_each_entry(kpg, &psys->pgs, list) {
		if (!kpg->pg_size && kpg->size >= pg_size) {
			kpg->pg_size = pg_size;
			spin_unlock_irqrestore(&psys->pgs_lock, flags);
			return kpg;
		}
	}
	spin_unlock_irqrestore(&psys->pgs_lock, flags);

	kpg = kzalloc(sizeof(*kpg), GFP_KERNEL);
	if (!kpg)
		return NULL;

	kpg->pg = dma_alloc_attrs(dev, pg_size,  &kpg->pg_dma_addr,
				  GFP_KERNEL, 0);
	if (!kpg->pg) {
		kfree(kpg);
		return NULL;
	}

	kpg->pg_size = pg_size;
	kpg->size = pg_size;
	spin_lock_irqsave(&psys->pgs_lock, flags);
	list_add(&kpg->list, &psys->pgs);
	spin_unlock_irqrestore(&psys->pgs_lock, flags);

	return kpg;
}

static inline
struct ipu6_psys_fence *to_ipu6_psys_fence(struct dma_fence *fence)
{
	return container_of(fence, struct ipu6_psys_fence, base);
}

static const char *ipu6_psys_fence_get_driver_name(struct dma_fence *fence)
{
	return "intel-ipu6-psys";
}

static const char *ipu6_psys_fence_get_timeline_name(struct dma_fence *fence)
{
	struct ipu6_psys_fence *ipu6_psys_fence = to_ipu6_psys_fence(fence);

	return dev_name(ipu6_psys_fence->psys->drm_dev.dev);
}

static const struct dma_fence_ops ipu6_psys_fence_ops = {
	.get_driver_name = ipu6_psys_fence_get_driver_name,
	.get_timeline_name = ipu6_psys_fence_get_timeline_name,
};

static struct dma_fence *ipu6_psys_fence_create(struct ipu6_psys *psys)
{
	struct ipu6_psys_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return NULL;

	fence->psys = psys;
	spin_lock_init(&fence->lock);
	dma_fence_init(&fence->base, &ipu6_psys_fence_ops, &fence->lock,
		       dma_fence_context_alloc(1), 1);

	return &fence->base;
}

static int ipu6_psys_prepare_kcmd(struct drm_file *file,
				  struct ipu6_psys_cmd *cmd,
				  struct ipu6_psys_kcmd *kcmd)
{
	struct ipu6_psys_fh *fh = file->driver_priv;
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_kbuffer *kpgbuf;
	struct ww_acquire_ctx ctx;
	unsigned int i;
	int ret;

	if (!cmd->pg_manifest_size)
		return -EINVAL;

	kcmd->state = KCMD_STATE_PPG_NEW;
	kcmd->fh = fh;
	INIT_LIST_HEAD(&kcmd->list);
	kcmd->complete_fence = ipu6_psys_fence_create(psys);

	mutex_lock(&fh->mutex);
	mutex_unlock(&fh->mutex);

	kcmd->pg_user = kpgbuf->vaddr;
	kcmd->kpg = ipu6_psys_get_pg_buf(psys, kpgbuf->len);
	if (!kcmd->kpg) {
		ret = -ENOMEM;
		goto free;
	}

	memcpy(kcmd->kpg->pg, kcmd->pg_user, kcmd->kpg->pg_size);

	kcmd->pg_manifest = kzalloc(cmd->pg_manifest_size, GFP_KERNEL);
	if (!kcmd->pg_manifest) {
		ret = -ENOMEM;
		goto free;
	}

	ret = copy_from_user(kcmd->pg_manifest, cmd->pg_manifest,
			     cmd->pg_manifest_size);
	if (ret) {
		ret = -EINVAL;
		goto free;
	}

	kcmd->pg_manifest_size = cmd->pg_manifest_size;
	kcmd->user_token = cmd->user_token;
	kcmd->issue_id = cmd->issue_id;
	kcmd->priority = cmd->priority;

	memcpy(kcmd->kernel_enable_bitmap, cmd->kernel_enable_bitmap,
	       sizeof(cmd->kernel_enable_bitmap));

	kcmd->nbuffers = ipu6_psys_fw_pg_get_terminal_count(kcmd);
	if (kcmd->nbuffers > IPU6_MAX_PSYS_CMD_BUFFERS)
		return -EINVAL;

	kcmd->buffers = kcalloc(kcmd->nbuffers, sizeof(*kcmd->buffers),
				GFP_KERNEL);
	if (!kcmd->buffers) {
		ret = -ENOMEM;
		goto free;
	}

	kcmd->kbufs = kcalloc(kcmd->nbuffers, sizeof(kcmd->kbufs[0]),
			      GFP_KERNEL);
	if (!kcmd->kbufs) {
		ret = -ENOMEM;
		goto free;
	}

	kcmd->handles = kcalloc(kcmd->nbuffers, sizeof(*kcmd->handles),
				GFP_KERNEL);
	if (!kcmd->handles) {
		ret = -ENOMEM;
		goto free;
	}

	ret = copy_from_user(kcmd->buffers, cmd->buffers,
			     kcmd->nbuffers * sizeof(*kcmd->buffers));
	if (ret)
		goto free;

	ret = copy_from_user(kcmd->handles, (void __user *)cmd->handles,
			     kcmd->nbuffers * sizeof(*kcmd->handles));
	if (ret)
		goto free;

	for (i = 0; i < kcmd->nbuffers; i++) {
		struct ipu6_psys_fw_terminal *terminal;

		terminal = ipu6_psys_fw_pg_get_terminal(kcmd, i);
		if (!terminal)
			continue;

		kcmd->kbufs[i] = kpgbuf;
		dma_sync_sg_for_device(dev, kcmd->kbufs[i]->sgt->sgl,
				       kcmd->kbufs[i]->sgt->orig_nents,
				       DMA_BIDIRECTIONAL);
	}

	drm_gem_objects_lookup(file, (void __user *)cmd->handles,
			       kcmd->nbuffers, &kcmd->bos);

	if (dma_resv_test_signaled(kcmd->kbufs[i]->shmem_bo.base.resv,
				   DMA_RESV_USAGE_READ)) {
		dev_warn(dev, "Buffer is already locked.\n");
		ret = -EBUSY;
		goto free;
	}

	ret = drm_gem_lock_reservations(kcmd->bos, kcmd->nbuffers, &ctx);
	if (ret) {
		ret = -EINVAL;
		dev_err(dev, "Failed to lock reservations: %d\n", ret);
		goto free;
	}

	for (i = 0; i < kcmd->nbuffers; i++) {
		ret = dma_resv_reserve_fences(kcmd->bos[i]->resv, 1);
		if (ret) {
			ret = -EINVAL;
			dev_err(dev, "Failed to reserve fences: %d\n", ret);
			goto unlock;
		}
	}

	for (i = 0; i < kcmd->nbuffers; i++)
		dma_resv_add_fence(kcmd->bos[i]->resv, kcmd->complete_fence,
				   DMA_RESV_USAGE_READ);

	if (kcmd->state != KCMD_STATE_PPG_START)
		kcmd->state = KCMD_STATE_PPG_ENQUEUE;

	return 0;

unlock:
	drm_gem_unlock_reservations(kcmd->bos, kcmd->nbuffers, &ctx);
free:
	ipu6_psys_kcmd_free(kcmd);

	dev_dbg(dev, "failed to copy cmd\n");

	return ret;
}

static int ipu6_psys_kcmd_start(struct ipu6_psys *psys,
				struct ipu6_psys_kcmd *kcmd)
{
	struct device *dev = &psys->adev->auxdev.dev;
	int ret;

	ipu6_psys_fw_pg_start(kcmd);
	ret = ipu6_psys_fw_pg_disown(kcmd);
	if (ret) {
		dev_err(dev, "failed to start kcmd!\n");
		return ret;
	}

	return 0;
}

static void ipu6_psys_kcmd_complete(struct ipu6_psys_ppg *kppg,
				    struct ipu6_psys_kcmd *kcmd, int error)
{
	unsigned int i;

	list_move_tail(&kcmd->list, &kppg->kcmds_complete);

	for (i = 0; i < kcmd->nbuffers; i++)
		drm_gem_object_put(&kcmd->kbufs[i]->shmem_bo.base);

	kcmd->state = KCMD_STATE_PPG_COMPLETE;
	dma_fence_signal(kcmd->complete_fence);
}

static int ipu6_psys_ppg_start(struct ipu6_psys_ppg *kppg)
{
	struct ipu6_psys *psys = kppg->fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_kcmd *kcmd;
	unsigned int i;
	int ret;

	kcmd = ipu6_psys_ppg_get_kcmd(kppg, KCMD_STATE_PPG_START);
	if (!kcmd) {
		dev_err(dev, "failed to find start kcmd!\n");
		return -EINVAL;
	}

	dev_dbg(dev, "start ppg id %d, addr 0x%p\n",
		ipu6_psys_fw_pg_get_id(kcmd), kppg);

	kppg->state = PPG_STATE_STARTING;
	for (i = 0; i < kcmd->nbuffers; i++) {
		struct ipu6_psys_fw_terminal *terminal;

		terminal = ipu6_psys_fw_pg_get_terminal(kcmd, i);
		if (!terminal)
			continue;

		ret = ipu6_psys_fw_terminal_set(terminal, i, kcmd, 0,
						kcmd->buffers[i].len);
		if (ret) {
			dev_err(dev, "Unable to set terminal\n");
			return ret;
		}
	}

	ipu6_psys_fw_pg_submit(kcmd);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "failed to power on psys\n");
		goto error;
	}

	ret = ipu6_psys_kcmd_start(psys, kcmd);
	if (ret) {
		ipu6_psys_kcmd_complete(kppg, kcmd, -EIO);
		goto error;
	}

	dev_dbg(dev, "s_change:%s: %p %d -> %d\n",
		__func__, kppg, kppg->state, PPG_STATE_STARTED);
	kppg->state = PPG_STATE_STARTED;
	ipu6_psys_kcmd_complete(kppg, kcmd, 0);

	return 0;

error:
	pm_runtime_put_noidle(dev);
	dev_err(dev, "failed to start ppg\n");

	return ret;
}

static int ipu6_psys_ppg_stop(struct ipu6_psys_ppg *kppg)
{
	struct ipu6_psys *psys = kppg->fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_kcmd kcmd_temp;
	struct ipu6_psys_kcmd *kcmd;
	int ppg_id, ret = 0;

	kcmd  = ipu6_psys_ppg_get_kcmd(kppg, KCMD_STATE_PPG_STOP);
	if (kcmd) {
		list_move_tail(&kcmd->list, &kppg->kcmds_running);
	} else {
		kcmd_temp.kpg = kppg->kpg;
		kcmd_temp.fh = kppg->fh;
		kcmd = &kcmd_temp;
	}

	ppg_id = ipu6_psys_fw_pg_get_id(kcmd);
	kppg->state = PPG_STATE_STOPPING;
	ipu6_psys_fw_pg_abort(kcmd);

	dev_dbg(dev, "stop ppg(%d, addr 0x%p)\n", ppg_id, kppg);

	return ret;
}

static bool ipu6_psys_ppg_enqueue_bufsets(struct ipu6_psys_ppg *kppg)
{
	struct ipu6_psys_kcmd *kcmd, *kcmd0;
	struct ipu6_psys *psys = kppg->fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	bool resume = false;

	mutex_lock(&kppg->mutex);
	if (!(kppg->state & (PPG_STATE_STARTED | PPG_STATE_RUNNING))) {
		mutex_unlock(&kppg->mutex);
		return false;
	}

	if (!list_empty(&kppg->kcmds_new)) {
		mutex_unlock(&kppg->mutex);
		return false;
	}

	list_for_each_entry_safe(kcmd, kcmd0,
				 &kppg->kcmds_new, list) {
		int ret;

		if (kcmd->state != KCMD_STATE_PPG_ENQUEUE) {
			resume = true;
			break;
		}

		ret = ipu6_psys_fw_ppg_enqueue_bufs(kcmd);
		if (ret) {
			dev_err(dev,
				"kppg 0x%p fail to qbufset %d",
				kppg, ret);
			break;
		}
		list_move_tail(&kcmd->list,
			       &kppg->kcmds_running);
		dev_dbg(dev, "kppg %d %p queue kcmd 0x%p fh 0x%p\n",
			ipu6_psys_fw_pg_get_id(kcmd), kppg, kcmd, kcmd->fh);
	}

	mutex_unlock(&kppg->mutex);
	return resume;
}

static void ipu6_psys_update_ppg_state_by_kcmd(struct ipu6_psys *psys,
					       struct ipu6_psys_ppg *kppg,
					       struct ipu6_psys_kcmd *kcmd)
{
	struct device *dev = &psys->adev->auxdev.dev;

	if (kppg->state == PPG_STATE_STARTED) {
		if (kcmd->state == KCMD_STATE_PPG_START)
			ipu6_psys_kcmd_complete(kppg, kcmd, 0);
		else if (kcmd->state == KCMD_STATE_PPG_STOP)
			kppg->state = PPG_STATE_STOP;
	} else if (kppg->state == PPG_STATE_STOPPED) {
		if (kcmd->state == KCMD_STATE_PPG_START) {
			kppg->state = PPG_STATE_START;
		} else if (kcmd->state == KCMD_STATE_PPG_STOP) {
			ipu6_psys_kcmd_complete(kppg, kcmd, 0);
		} else if (kcmd->state == KCMD_STATE_PPG_ENQUEUE) {
			dev_err(dev, "ppg %p stopped!\n", kppg);
			ipu6_psys_kcmd_complete(kppg, kcmd, -EIO);
		}
	}
}

static void ipu6_psys_ppg_complete(struct ipu6_psys *psys,
				   struct ipu6_psys_ppg *kppg)
{
	struct device *dev = &psys->adev->auxdev.dev;
	u8 queue_id;

	if (!psys || !kppg)
		return;

	mutex_lock(&kppg->mutex);
	if (kppg->state == PPG_STATE_STOPPING) {
		struct ipu6_psys_kcmd tmp_kcmd = {
			.kpg = kppg->kpg,
		};

		kppg->state = PPG_STATE_STOPPED;
		queue_id = ipu6_psys_fw_ppg_get_base_queue_id(&tmp_kcmd);
		ipu6_psys_fw_put_cmd_queue(psys, queue_id);
		pm_runtime_put(dev);
	}

	if (kppg->state == PPG_STATE_STARTED) {
		kppg->state = PPG_STATE_RUNNING;
		atomic_set(&psys->wakeup, 1);
		wake_up_interruptible(&psys->psys_wq);
	}

	mutex_unlock(&kppg->mutex);
}

static int ipu6_psys_kcmd_to_cmdq_start(struct ipu6_psys_kcmd *kcmd)
{
	struct ipu6_psys_fh *fh = kcmd->fh;
	struct ipu6_psys_cmdq *cmdq = &fh->cmdq;
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_ppg *kppg;
	int queue_id;

	kppg = kzalloc(sizeof(*kppg), GFP_KERNEL);
	if (!kppg)
		return -ENOMEM;

	kppg->fh = fh;
	kppg->kpg = kcmd->kpg;
	kppg->state = PPG_STATE_START;
	kppg->priority = kcmd->priority;
	INIT_LIST_HEAD(&kppg->list);

	mutex_init(&kppg->mutex);
	INIT_LIST_HEAD(&kppg->kcmds_new);
	INIT_LIST_HEAD(&kppg->kcmds_running);
	INIT_LIST_HEAD(&kppg->kcmds_complete);

	kppg->manifest = kzalloc(kcmd->pg_manifest_size, GFP_KERNEL);
	if (!kppg->manifest) {
		kfree(kppg);
		return -ENOMEM;
	}
	memcpy(kppg->manifest, kcmd->pg_manifest,
	       kcmd->pg_manifest_size);

	queue_id = ipu6_psys_fw_get_cmd_queue(psys);
	if (queue_id == -ENOSPC) {
		dev_err(dev, "no available queue\n");
		kfree(kppg->manifest);
		kfree(kppg);
		mutex_unlock(&psys->mutex);
		return -ENOMEM;
	}

	kppg->token = (u64)kcmd->kpg;
	ipu6_psys_fw_ppg_set_base_queue_id(kcmd, queue_id);
	ipu6_psys_fw_pg_set_token(kcmd, kppg->token);
	ipu6_psys_fw_pg_set_ipu6_vaddress(kcmd, kcmd->kpg->pg_dma_addr);
	memcpy(kcmd->pg_user, kcmd->kpg->pg, kcmd->kpg->pg_size);

	mutex_lock(&fh->mutex);
	list_add_tail(&kppg->list, &cmdq->ppgs);
	mutex_unlock(&fh->mutex);

	mutex_lock(&kppg->mutex);
	list_add(&kcmd->list, &kppg->kcmds_new);
	mutex_unlock(&kppg->mutex);

	dev_dbg(dev, "START ppg(%d, 0x%p) kcmd 0x%p, queue %d\n",
		ipu6_psys_fw_pg_get_id(kcmd), kppg, kcmd, queue_id);

	atomic_set(&psys->wakeup, 1);
	wake_up_interruptible(&psys->psys_wq);

	return 0;
}

static int ipu6_psys_kcmd_to_cmdq(struct ipu6_psys_kcmd *kcmd)
{
	struct ipu6_psys_fh *fh = kcmd->fh;
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_ppg *kppg;
	unsigned long flags;
	u8 id;

	if (kcmd->state == KCMD_STATE_PPG_START)
		return ipu6_psys_kcmd_to_cmdq_start(kcmd);

	kppg = ipu6_psys_identify_kppg(kcmd);
	if (!kppg) {
		dev_err(dev, "token not match\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&psys->pgs_lock, flags);
	kcmd->kpg->pg_size = 0;
	spin_unlock_irqrestore(&psys->pgs_lock, flags);
	kcmd->kpg = kppg->kpg;

	if (kcmd->state == KCMD_STATE_PPG_STOP) {
		mutex_lock(&kppg->mutex);
		if (kppg->state == PPG_STATE_STOPPED) {
			id = ipu6_psys_fw_ppg_get_base_queue_id(kcmd);
			ipu6_psys_fw_put_cmd_queue(psys, id);
			ipu6_psys_kcmd_complete(kppg, kcmd, 0);
			pm_runtime_put(dev);
			mutex_unlock(&kppg->mutex);
			return 0;
		}

		list_add(&kcmd->list, &kppg->kcmds_new);
		mutex_unlock(&kppg->mutex);
	} else {
		int ret;

		ret = ipu6_psys_ppg_get_bufset(kcmd, kppg);
		if (ret)
			return ret;

		mutex_lock(&kppg->mutex);
		list_add_tail(&kcmd->list, &kppg->kcmds_new);
		mutex_unlock(&kppg->mutex);
	}

	atomic_set(&psys->wakeup, 1);
	wake_up_interruptible(&psys->psys_wq);

	return 0;
}

int ipu6_psys_submit_kcmd(struct drm_file *file, struct ipu6_psys_cmd *cmd)
{
	struct ipu6_psys_fh *fh = file->driver_priv;
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_kcmd *kcmd;
	size_t pg_size;
	int ret;

	kcmd = kzalloc(sizeof(*kcmd), GFP_KERNEL);
	if (!kcmd)
		return -ENOMEM;

	ret = ipu6_psys_prepare_kcmd(file, cmd, kcmd);
	if (ret)
		return -EINVAL;

	pg_size = ipu6_psys_fw_pg_get_size(kcmd);
	if (pg_size > kcmd->kpg->pg_size) {
		dev_dbg(dev, "pg size mismatch %lu %lu\n", pg_size,
			kcmd->kpg->pg_size);
		ret = -EINVAL;
		goto error;
	}

	if (ipu6_psys_fw_pg_get_protocol(kcmd) !=
	    IPU6_FW_PSYS_PROCESS_GROUP_PROTOCOL_PPG) {
		dev_err(dev, "legacy interface is not supported.\n");
		ret = -EINVAL;
		goto error;
	}

	ret = ipu6_psys_kcmd_to_cmdq(kcmd);
	if (ret)
		goto error;

	return 0;

error:
	ipu6_psys_kcmd_free(kcmd);

	return ret;
}

struct ipu6_psys_kcmd *ipu6_get_completed_kcmd(struct ipu6_psys_fh *fh)
{
	struct ipu6_psys_cmdq *cmdq = &fh->cmdq;
	struct device *dev = &fh->psys->adev->auxdev.dev;
	struct ipu6_psys_kcmd *kcmd;
	struct ipu6_psys_ppg *kppg;

	mutex_lock(&fh->mutex);
	if (list_empty(&cmdq->ppgs)) {
		mutex_unlock(&fh->mutex);
		return NULL;
	}

	list_for_each_entry(kppg, &cmdq->ppgs, list) {
		mutex_lock(&kppg->mutex);
		if (list_empty(&kppg->kcmds_complete)) {
			mutex_unlock(&kppg->mutex);
			continue;
		}

		kcmd = list_first_entry(&kppg->kcmds_complete,
					struct ipu6_psys_kcmd, list);
		mutex_unlock(&fh->mutex);
		mutex_unlock(&kppg->mutex);
		dev_dbg(dev, "get completed kcmd 0x%p\n", kcmd);
		return kcmd;
	}
	mutex_unlock(&fh->mutex);

	return NULL;
}

void ipu6_psys_kcmd_free(struct ipu6_psys_kcmd *kcmd)
{
	struct ipu6_psys_ppg *kppg;
	struct ipu6_psys_cmdq *cmdq;

	if (!kcmd)
		return;

	kppg = ipu6_psys_identify_kppg(kcmd);
	cmdq = &kcmd->fh->cmdq;

	if (kcmd->kbuf_set) {
		mutex_lock(&cmdq->bs_mutex);
		kcmd->kbuf_set->buf_set_size = 0;
		mutex_unlock(&cmdq->bs_mutex);
		kcmd->kbuf_set = NULL;
	}

	if (kppg) {
		mutex_lock(&kppg->mutex);
		if (!list_empty(&kcmd->list))
			list_del(&kcmd->list);
		mutex_unlock(&kppg->mutex);
	}

	dma_fence_put(kcmd->complete_fence);
	kfree(kcmd->pg_manifest);
	kfree(kcmd->kbufs);
	kfree(kcmd->buffers);
	kfree(kcmd);
}

void ipu6_psys_handle_events(struct ipu6_psys *psys)
{
	struct ipu6_psys_kcmd *kcmd;
	struct ipu6_psys_fw_event event;
	struct ipu6_psys_ppg *kppg;
	u16 status;
	int res;

	do {
		memset(&event, 0, sizeof(event));
		if (!ipu6_psys_fw_rcv_event(psys, &event))
			break;

		if (!event.context_handle)
			break;

		status = event.status;
		res = (status == IPU6_PSYS_EVENT_CMD_COMPLETE ||
		       status == IPU6_PSYS_EVENT_FRAGMENT_COMPLETE) ?
			0 : -EIO;
		ipu6_psys_ppg_complete(psys, kppg);

		mutex_lock(&kppg->mutex);
		ipu6_psys_kcmd_complete(kppg, kcmd, res);
		mutex_unlock(&kppg->mutex);
	} while (1);
}

int ipu6_psys_wait_kcmd(struct drm_file *file, u32 handle, u32 timeout_us)
{
	struct drm_gem_object *obj;
	u64 timeout;
	int ret;

	timeout = drm_timeout_abs_to_jiffies(timeout_us);
	obj = drm_gem_object_lookup(file, handle);
	if (!obj)
		return -EINVAL;

	ret = dma_resv_wait_timeout(obj->resv, DMA_RESV_USAGE_READ, true,
				    timeout);
	if (ret == 0)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = 0;

	drm_gem_object_put(obj);

	return 0;
}

void ipu6_psys_cmd_run(struct ipu6_psys *psys)
{
	struct ipu6_psys_cmdq *cmdq;
	struct ipu6_psys_kcmd *kcmd;
	struct ipu6_psys_ppg *kppg, *tmp;
	struct ipu6_psys_fh *fh;

	list_for_each_entry(fh, &psys->fhs, list) {
		mutex_lock(&fh->mutex);
		cmdq = &fh->cmdq;
		if (list_empty(&cmdq->ppgs)) {
			mutex_unlock(&fh->mutex);
			continue;
		}

		list_for_each_entry_safe(kppg, tmp, &cmdq->ppgs, list) {
			mutex_lock(&kppg->mutex);
			if (list_empty(&kppg->kcmds_new)) {
				mutex_unlock(&kppg->mutex);
				continue;
			};

			kcmd = list_first_entry(&kppg->kcmds_new,
						struct ipu6_psys_kcmd, list);
			ipu6_psys_update_ppg_state_by_kcmd(psys, kppg, kcmd);

			if (kppg->state & PPG_STATE_STOP)
				ipu6_psys_ppg_stop(kppg);

			if (kppg->state == PPG_STATE_START)
				ipu6_psys_ppg_start(kppg);

			ipu6_psys_ppg_enqueue_bufsets(kppg);

			mutex_unlock(&kppg->mutex);
		}
		mutex_unlock(&fh->mutex);
	}
}
