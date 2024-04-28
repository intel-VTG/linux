// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2024 Intel Corporation

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <drm/drm_accel.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_prime.h>
#include <drm/drm_utils.h>

#include <media/ipu6/ipu6.h>
#include <media/ipu6/ipu6-bus.h>
#include <media/ipu6/ipu6-buttress.h>
#include <media/ipu6/ipu6-cpd.h>
#include <media/ipu6/ipu6-fw-com.h>
#include <media/ipu6/ipu6-mmu.h>
#include <media/ipu6/ipu6-platform-regs.h>

#include <uapi/drm/ipu6_psys_accel.h>

#include "ipu6-psys.h"
#include "ipu6-psys-cmd.h"
#include "ipu6-psys-fw.h"
#include "ipu6-psys-gem.h"

#define BUTTRESS_FW_PARAMS_PSYS_OFFSET	7
#define IPU6_PSYS_NUM_DEVICES		4

#define PKG_DIR_ENT_LEN_FOR_PSYS	2
#define PKG_DIR_SIZE_MASK_FOR_PSYS	GENMASK(23, 0)

enum ipu6_version ipu6_ver;

static u32 ipu6_cpd_pkg_dir_get_address(const u64 *pkg_dir, int pkg_dir_idx)
{
	return pkg_dir[++pkg_dir_idx * PKG_DIR_ENT_LEN_FOR_PSYS];
}

static u32 ipu6_cpd_pkg_dir_get_num_entries(const u64 *pkg_dir)
{
	return pkg_dir[1];
}

static u32 ipu6_cpd_pkg_dir_get_size(const u64 *pkg_dir, int pkg_dir_idx)
{
	return pkg_dir[++pkg_dir_idx * PKG_DIR_ENT_LEN_FOR_PSYS + 1] &
		PKG_DIR_SIZE_MASK_FOR_PSYS;
}

#define PKG_DIR_ID_SHIFT		48
#define PKG_DIR_ID_MASK			0x7f

static u32 ipu6_cpd_pkg_dir_get_type(const u64 *pkg_dir, int pkg_dir_idx)
{
	return pkg_dir[++pkg_dir_idx * PKG_DIR_ENT_LEN_FOR_PSYS + 1] >>
		PKG_DIR_ID_SHIFT & PKG_DIR_ID_MASK;
}

static void ipu6_set_sp_info_bits(void __iomem *base)
{
	unsigned int i;

	writel(IPU6_INFO_REQUEST_DESTINATION_IOSF,
	       base + IPU6_REG_PSYS_INFO_SEG_0_CONFIG_ICACHE_MASTER);

	for (i = 0; i < 4; i++)
		writel(IPU6_INFO_REQUEST_DESTINATION_IOSF,
		       base + IPU6_REG_PSYS_INFO_SEG_CMEM_MASTER(i));
	for (i = 0; i < 4; i++)
		writel(IPU6_INFO_REQUEST_DESTINATION_IOSF,
		       base + IPU6_REG_PSYS_INFO_SEG_XMEM_MASTER(i));
}

static void ipu6_psys_subdomains_power(struct ipu6_psys *psys, bool on)
{
	struct device *dev = &psys->adev->auxdev.dev;
	void __iomem *reg;
	int ret;
	u32 val;

	dev_dbg(dev, "power %s psys sub-domains", on ? "UP" : "DOWN");
	reg = psys->adev->isp->base + IPU6_PSYS_SUBDOMAINS_POWER_REQ;
	writel(on ? IPU6_PSYS_SUBDOMAINS_POWER_MASK : 0, reg);

	reg =  psys->adev->isp->base + IPU6_PSYS_SUBDOMAINS_POWER_STATUS;
	ret = readl_poll_timeout(reg, val, !(val & BIT(31)), 200, 50000);
	if (ret)
		dev_warn(dev, "Psys sub-domains %s req timeout with 0x%x",
			 on ? "UP" : "DOWN", val);
}

static void ipu6_psys_setup_hw(struct ipu6_psys *psys)
{
	void __iomem *base = psys->pdata->base;
	void __iomem *spc_regs_base =
		base + psys->pdata->ipdata->hw_variant.spc_offset;
	void __iomem *psys_iommu0_ctrl;
	u32 irqs;
	const u8 r3 = IPU6_DEVICE_AB_GROUP1_TARGET_ID_R3_SPC_STATUS_REG;
	const u8 r4 = IPU6_DEVICE_AB_GROUP1_TARGET_ID_R4_SPC_MASTER_BASE_ADDR;
	const u8 r5 = IPU6_DEVICE_AB_GROUP1_TARGET_ID_R5_SPC_PC_STALL;

	if (!psys->adev->isp->secure_mode) {
		/* configure access blocker for non-secure mode */
		writel(NCI_AB_ACCESS_MODE_RW,
		       base + IPU6_REG_DMA_TOP_AB_GROUP1_BASE_ADDR +
		       IPU6_REG_DMA_TOP_AB_RING_ACCESS_OFFSET(r3));
		writel(NCI_AB_ACCESS_MODE_RW,
		       base + IPU6_REG_DMA_TOP_AB_GROUP1_BASE_ADDR +
		       IPU6_REG_DMA_TOP_AB_RING_ACCESS_OFFSET(r4));
		writel(NCI_AB_ACCESS_MODE_RW,
		       base + IPU6_REG_DMA_TOP_AB_GROUP1_BASE_ADDR +
		       IPU6_REG_DMA_TOP_AB_RING_ACCESS_OFFSET(r5));
	}
	psys_iommu0_ctrl = base +
		psys->pdata->ipdata->hw_variant.mmu_hw[0].offset +
		IPU6_MMU_INFO_OFFSET;
	writel(IPU6_INFO_REQUEST_DESTINATION_IOSF, psys_iommu0_ctrl);

	ipu6_set_sp_info_bits(spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);
	ipu6_set_sp_info_bits(spc_regs_base + IPU6_PSYS_REG_SPP0_STATUS_CTRL);

	/* Enable FW interrupt */
	writel(0, base + IPU6_REG_PSYS_GPDEV_FWIRQ(0));
	irqs = IPU6_PSYS_GPDEV_IRQ_FWIRQ(IPU6_PSYS_GPDEV_FWIRQ0);
	writel(irqs, base + IPU6_REG_PSYS_GPDEV_IRQ_EDGE);
	writel(irqs, base + IPU6_REG_PSYS_GPDEV_IRQ_LEVEL_NOT_PULSE);
	writel(0xffffffff, base + IPU6_REG_PSYS_GPDEV_IRQ_CLEAR);
	writel(irqs, base + IPU6_REG_PSYS_GPDEV_IRQ_MASK);
	writel(irqs, base + IPU6_REG_PSYS_GPDEV_IRQ_ENABLE);
}

static int ipu6_psys_fh_init(struct ipu6_psys_fh *fh)
{
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_buffer_set *kbuf_set, *kbuf_set_tmp;
	struct ipu6_psys_cmdq *cmdq = &fh->cmdq;
	int i;

	mutex_init(&cmdq->bs_mutex);
	INIT_LIST_HEAD(&cmdq->buf_sets);
	INIT_LIST_HEAD(&cmdq->ppgs);

	for (i = 0; i < IPU6_PSYS_BUF_SET_POOL_SIZE; i++) {
		kbuf_set = kzalloc(sizeof(*kbuf_set), GFP_KERNEL);
		if (!kbuf_set)
			goto out_free_buf_sets;
		kbuf_set->vaddr = dma_alloc_attrs(dev,
						  IPU6_PSYS_BUF_SET_MAX_SIZE,
						  &kbuf_set->dma_addr,
						  GFP_KERNEL, 0);
		if (!kbuf_set->vaddr) {
			kfree(kbuf_set);
			goto out_free_buf_sets;
		}
		kbuf_set->size = IPU6_PSYS_BUF_SET_MAX_SIZE;
		list_add(&kbuf_set->list, &cmdq->buf_sets);
	}

	return 0;

out_free_buf_sets:
	list_for_each_entry_safe(kbuf_set, kbuf_set_tmp,
				 &cmdq->buf_sets, list) {
		dma_free_attrs(dev, kbuf_set->size, kbuf_set->vaddr,
			       kbuf_set->dma_addr, 0);
		list_del(&kbuf_set->list);
		kfree(kbuf_set);
	}
	mutex_destroy(&cmdq->bs_mutex);

	return -ENOMEM;
}

static int ipu6_psys_fh_deinit(struct ipu6_psys_fh *fh)
{
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_psys_ppg *kppg, *kppg0;
	struct ipu6_psys_kcmd *kcmd, *kcmd0;
	struct ipu6_psys_buffer_set *kbuf_set, *kbuf_set0;
	struct ipu6_psys_cmdq *cmdq = &fh->cmdq;

	mutex_lock(&fh->mutex);
	if (!list_empty(&cmdq->ppgs)) {
		list_for_each_entry_safe(kppg, kppg0, &cmdq->ppgs, list) {
			unsigned long flags;

			mutex_lock(&kppg->mutex);
			list_del(&kppg->list);
			mutex_unlock(&kppg->mutex);

			list_for_each_entry_safe(kcmd, kcmd0,
						 &kppg->kcmds_new, list) {
				kcmd->pg_user = NULL;
				mutex_unlock(&fh->mutex);
				ipu6_psys_kcmd_free(kcmd);
				mutex_lock(&fh->mutex);
			}

			list_for_each_entry_safe(kcmd, kcmd0,
						 &kppg->kcmds_running,
						 list) {
				kcmd->pg_user = NULL;
				mutex_unlock(&fh->mutex);
				ipu6_psys_kcmd_free(kcmd);
				mutex_lock(&fh->mutex);
			}

			list_for_each_entry_safe(kcmd, kcmd0,
						 &kppg->kcmds_complete,
						 list) {
				kcmd->pg_user = NULL;
				mutex_unlock(&fh->mutex);
				ipu6_psys_kcmd_free(kcmd);
				mutex_lock(&fh->mutex);
			}

			spin_lock_irqsave(&psys->pgs_lock, flags);
			kppg->kpg->pg_size = 0;
			spin_unlock_irqrestore(&psys->pgs_lock, flags);

			mutex_destroy(&kppg->mutex);
			kfree(kppg->manifest);
			kfree(kppg);
		}
	}
	mutex_unlock(&fh->mutex);

	mutex_lock(&cmdq->bs_mutex);
	list_for_each_entry_safe(kbuf_set, kbuf_set0, &cmdq->buf_sets, list) {
		dma_free_attrs(dev, kbuf_set->size, kbuf_set->vaddr,
			       kbuf_set->dma_addr, 0);
		list_del(&kbuf_set->list);
		kfree(kbuf_set);
	}
	mutex_unlock(&cmdq->bs_mutex);
	mutex_destroy(&cmdq->bs_mutex);

	return 0;
}

static int ipu6_psys_open(struct drm_device *drm_dev, struct drm_file *file)
{
	struct ipu6_psys *psys = to_ipu6_psys(drm_dev);
	struct ipu6_psys_fh *fh;
	int idx;
	int ret;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh) {
		ret = -ENOMEM;
		goto exit;
	}

	fh->psys = psys;

	mutex_init(&fh->mutex);
	INIT_LIST_HEAD(&fh->bufmap);

	ret = ipu6_psys_fh_init(fh);
	if (ret)
		goto open_failed;

	mutex_lock(&psys->mutex);
	list_add_tail(&fh->list, &psys->fhs);
	mutex_unlock(&psys->mutex);

	file->driver_priv = fh;

	drm_info(drm_dev, "%s\n", __func__);

	return 0;

open_failed:
	mutex_destroy(&fh->mutex);
	kfree(fh);
exit:
	drm_dev_exit(idx);

	return ret;
}

static void ipu6_psys_release(struct drm_device *drm_dev,
			      struct drm_file *file)
{
	struct ipu6_psys *psys = to_ipu6_psys(drm_dev);
	struct ipu6_psys_fh *fh = file->driver_priv;

	mutex_lock(&psys->mutex);
	list_del(&fh->list);
	mutex_unlock(&psys->mutex);

	ipu6_psys_fh_deinit(fh);
	mutex_destroy(&fh->mutex);
	kfree(fh);
}

static int ipu6_psys_get_manifest(struct drm_device *drm_dev, void *data,
				  struct drm_file *file)
{
	struct ipu6_psys_fh *fh = file->driver_priv;
	struct ipu6_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_bus_device *adev = psys->adev;
	struct ipu6_device *isp = adev->isp;
	struct ipu6_cpd_client_pkg_hdr *client_pkg;
	struct ipu6_psys_manifest *manifest = data;
	u32 entries;
	void *host_fw_data;
	dma_addr_t dma_fw_data;
	u32 client_pkg_offset;

	host_fw_data = (void *)isp->cpd_fw->data;
	dma_fw_data = sg_dma_address(adev->fw_sgt.sgl);
	entries = ipu6_cpd_pkg_dir_get_num_entries(adev->pkg_dir);
	if (!manifest || manifest->index > entries - 1) {
		dev_err(dev, "invalid argument\n");
		return -EINVAL;
	}

	if (!ipu6_cpd_pkg_dir_get_size(adev->pkg_dir, manifest->index) ||
	    ipu6_cpd_pkg_dir_get_type(adev->pkg_dir, manifest->index) <
	    IPU6_CPD_PKG_DIR_CLIENT_PG_TYPE)
		return -ENOENT;

	client_pkg_offset = ipu6_cpd_pkg_dir_get_address(adev->pkg_dir,
							 manifest->index);
	client_pkg_offset -= dma_fw_data;
	client_pkg = host_fw_data + client_pkg_offset;
	manifest->size = client_pkg->pg_manifest_size;

	if (!manifest->manifest)
		return 0;

	if (copy_to_user(manifest->manifest,
			 (uint8_t *)client_pkg + client_pkg->pg_manifest_offs,
			 manifest->size)) {
		return -EFAULT;
	}

	return 0;
}

static int ipu6_psys_create_bo(struct drm_device *drm_dev, void *data,
			       struct drm_file *file)
{
	struct ipu6_psys_fh *fh = file->driver_priv;
	struct ipu6_psys *psys = fh->psys;
	struct ipu6_psys_buffer *buf = data;
	struct ipu6_psys_kbuffer *kbuf;
	int idx;
	int ret;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	kbuf = ipu6_psys_bo_alloc(psys, buf);
	if (!kbuf) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = drm_gem_handle_create(file, &kbuf->shmem_bo.base, &buf->handle);
	if (!ret)
		list_add(&kbuf->node, &fh->bufmap);

exit:
	drm_dev_exit(idx);

	return ret;
}

static int ipu6_psys_get_bo_info(struct drm_device *drm_dev, void *data,
				 struct drm_file *file)
{
	int idx;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	drm_dev_exit(idx);

	return 0;
}

static int ipu6_psys_mapbuf(struct drm_device *drm_dev, void *data,
			    struct drm_file *file)
{
	struct ipu6_psys_map *map = data;
	struct ipu6_psys *psys = to_ipu6_psys(drm_dev);
	struct ipu6_psys_fh *fh = file->driver_priv;
	int idx;
	int ret;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	mutex_lock(&fh->mutex);
	ret = ipu6_psys_mapbuf_locked(file, psys, map);
	mutex_unlock(&fh->mutex);

	drm_dev_exit(idx);
	return ret;
}

static int ipu6_psys_unmapbuf(struct drm_device *drm_dev, void *data,
			      struct drm_file *file)
{
	struct ipu6_psys_map *map = data;
	struct ipu6_psys *psys = to_ipu6_psys(drm_dev);
	struct ipu6_psys_fh *fh = file->driver_priv;
	int idx;
	int ret;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	mutex_lock(&fh->mutex);
	ret = ipu6_psys_unmapbuf_locked(file, psys, map);
	mutex_unlock(&fh->mutex);

	drm_dev_exit(idx);
	return ret;
}

static int ipu6_psys_submit_cmd(struct drm_device *drm_dev, void *data,
				struct drm_file *file)
{
	struct ipu6_psys_cmd *cmd = data;
	int idx;
	int ret;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	ret = ipu6_psys_submit_kcmd(file, cmd);
	drm_dev_exit(idx);
	return ret;
}

static int ipu6_psys_wait_event(struct drm_device *drm_dev, void *data,
				struct drm_file *file)
{
	struct ipu6_psys_wait *wait = data;
	int idx;
	int ret;

	if (!drm_dev_enter(drm_dev, &idx))
		return -ENODEV;

	ret = ipu6_psys_wait_kcmd(file, wait->handle, wait->timeout_us);
	drm_dev_exit(idx);

	return ret;
}

static const struct drm_ioctl_desc ipu6_psys_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(IPU6_PS_GET_MANIFEST, ipu6_psys_get_manifest, 0),
	DRM_IOCTL_DEF_DRV(IPU6_PS_CREATE_BO, ipu6_psys_create_bo, 0),
	DRM_IOCTL_DEF_DRV(IPU6_PS_GET_BO_INFO, ipu6_psys_get_bo_info, 0),
	DRM_IOCTL_DEF_DRV(IPU6_PS_BO_MAP, ipu6_psys_mapbuf, 0),
	DRM_IOCTL_DEF_DRV(IPU6_PS_BO_UNMAP, ipu6_psys_unmapbuf, 0),
	DRM_IOCTL_DEF_DRV(IPU6_PS_SUBMIT_CMD, ipu6_psys_submit_cmd, 0),
	DRM_IOCTL_DEF_DRV(IPU6_PS_BO_WAIT, ipu6_psys_wait_event, 0),
};

static const struct file_operations ipu6_psys_fops = {
	DRM_ACCEL_FOPS,
	.owner = THIS_MODULE,
};

static const struct drm_driver ipu6_psys_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_COMPUTE_ACCEL,

	.open = ipu6_psys_open,
	.postclose = ipu6_psys_release,

	.gem_create_object = ipu6_psys_gem_create_object,
	.gem_prime_import = ipu6_psys_gem_prime_import,
	.gem_prime_import_sg_table = ipu6_psys_gem_prime_import_sg_table,

	.ioctls = ipu6_psys_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(ipu6_psys_drm_ioctls),
	.fops = &ipu6_psys_fops,

	.name = IPU6_PSYS_DRM_DRV_NAME,
	.desc = "Driver for Intel IPU6 processing system",
	.date = "20240415",
	.major = 1,
	.minor = 0,
};

static int psys_runtime_pm_resume(struct device *dev)
{
	struct ipu6_bus_device *adev = to_ipu6_bus_device(dev);
	struct ipu6_psys *psys = ipu6_bus_get_drvdata(adev);
	unsigned long flags;
	int ret;

	if (!psys)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
	if (psys->ready) {
		spin_unlock_irqrestore(&psys->ready_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	ret = ipu6_mmu_hw_init(adev->mmu);
	if (ret)
		return ret;

	if (!ipu6_buttress_auth_done(adev->isp)) {
		dev_dbg(dev, "fw not yet authenticated, skipping\n");
		return 0;
	}

	ipu6_psys_setup_hw(psys);

	ipu6_psys_subdomains_power(psys, 1);
	ipu6_configure_spc(adev->isp,
			   &psys->pdata->ipdata->hw_variant,
			   IPU6_CPD_PKG_DIR_PSYS_SERVER_IDX,
			   psys->pdata->base, adev->pkg_dir,
			   adev->pkg_dir_dma_addr);

	ret = ipu6_psys_fw_open(psys);
	if (ret) {
		dev_err(dev, "Failed to open abi.\n");
		return ret;
	}

	spin_lock_irqsave(&psys->ready_lock, flags);
	psys->ready = 1;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	return 0;
}

static int psys_runtime_pm_suspend(struct device *dev)
{
	struct ipu6_bus_device *adev = to_ipu6_bus_device(dev);
	struct ipu6_psys *psys = ipu6_bus_get_drvdata(adev);
	unsigned long flags;
	int ret;

	if (!psys)
		return 0;

	if (!psys->ready)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
	psys->ready = 0;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	ret = ipu6_psys_fw_close(psys);
	if (ret)
		dev_err(dev, "Device close failure: %d\n", ret);

	ipu6_psys_subdomains_power(psys, 0);

	ipu6_mmu_hw_cleanup(adev->mmu);

	return 0;
}

static int psys_resume(struct device *dev)
{
	return 0;
}

static int psys_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops psys_pm_ops = {
	.runtime_suspend = psys_runtime_pm_suspend,
	.runtime_resume = psys_runtime_pm_resume,
	.suspend = psys_suspend,
	.resume = psys_resume,
};

static int ipu6_psys_cmd_thread(void *ptr)
{
	struct ipu6_psys *psys = ptr;
	int wake;
	int ret;

	while (!kthread_should_stop()) {
		wake = atomic_read(&psys->wakeup);
		ret = wait_event_interruptible(psys->psys_wq,
					       kthread_should_stop() || wake);
		if (ret)
			return ret;

		if (!wake)
			continue;

		mutex_lock(&psys->mutex);
		atomic_set(&psys->wakeup, 0);
		ipu6_psys_cmd_run(psys);
		mutex_unlock(&psys->mutex);
	}

	return 0;
}

static void start_sp(struct ipu6_bus_device *adev)
{
	struct ipu6_psys *psys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = psys->pdata->base +
		psys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = 0;

	val |= IPU6_PSYS_SPC_STATUS_START |
		IPU6_PSYS_SPC_STATUS_RUN |
		IPU6_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;
	val |= psys->icache_prefetch_sp ?
		IPU6_PSYS_SPC_STATUS_ICACHE_PREFETCH : 0;
	writel(val, spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);
}

static int query_sp(struct ipu6_bus_device *adev)
{
	struct ipu6_psys *psys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = psys->pdata->base +
		psys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = readl(spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);

	val &= IPU6_PSYS_SPC_STATUS_READY | IPU6_PSYS_SPC_STATUS_START;

	return val == IPU6_PSYS_SPC_STATUS_READY;
}

static int ipu6_psys_fw_init(struct ipu6_psys *psys)
{
	struct ipu6_fw_syscom_queue_config *queue_cfg;
	struct device *dev = &psys->adev->auxdev.dev;
	unsigned int size;
	struct ipu6_fw_syscom_queue_config fw_psys_event_queue_cfg[] = {
		{
			IPU6_FW_PSYS_EVENT_QUEUE_SIZE,
			sizeof(struct ipu6_psys_fw_event)
		}
	};
	struct ipu6_psys_fw_srv_init server_init = {
		.ddr_pkg_dir_address = 0,
		.host_ddr_pkg_dir = NULL,
		.pkg_dir_size = 0,
		.icache_prefetch_sp = psys->icache_prefetch_sp,
		.icache_prefetch_isp = psys->icache_prefetch_isp,
	};
	struct ipu6_fw_com_cfg fwcom = {
		.num_output_queues = IPU6_FW_PSYS_N_PSYS_EVENT_QUEUE_ID,
		.output = fw_psys_event_queue_cfg,
		.specific_addr = &server_init,
		.specific_size = sizeof(server_init),
		.cell_start = start_sp,
		.cell_ready = query_sp,
		.buttress_boot_offset = BUTTRESS_FW_PARAMS_PSYS_OFFSET,
	};
	int i;

	size = IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	if (ipu6_ver == IPU6_VER_6 || ipu6_ver == IPU6_VER_6EP ||
	    ipu6_ver == IPU6_VER_6EP_MTL)
		size = IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID;

	queue_cfg = devm_kzalloc(dev, sizeof(*queue_cfg) * size,
				 GFP_KERNEL);
	if (!queue_cfg)
		return -ENOMEM;

	for (i = 0; i < size; i++) {
		queue_cfg[i].queue_size = IPU6_FW_PSYS_CMD_QUEUE_SIZE;
		queue_cfg[i].token_size = sizeof(struct ipu6_psys_fw_cmd);
	}

	fwcom.input = queue_cfg;
	fwcom.num_input_queues = size;
	fwcom.dmem_addr = psys->pdata->ipdata->hw_variant.dmem_offset;

	psys->fwcom = ipu6_fw_com_prepare(&fwcom, psys->adev,
					  psys->pdata->base);
	if (!psys->fwcom) {
		dev_err(dev, "psys fw com prepare failed\n");
		return -EIO;
	}

	return 0;
}

static int ipu6_psys_probe(struct auxiliary_device *auxdev,
			   const struct auxiliary_device_id *auxdev_id)
{
	struct ipu6_bus_device *adev = auxdev_to_adev(auxdev);
	struct device *dev = &auxdev->dev;
	struct ipu6_psys_pg *kpg, *kpg0;
	struct ipu6_psys *psys;
	int i, ret = -E2BIG;

	if (!adev->isp->bus_ready_to_probe)
		return -EPROBE_DEFER;

	if (!adev->pkg_dir)
		return -EPROBE_DEFER;

	ipu6_ver = adev->isp->hw_ver;
	ret = ipu6_mmu_hw_init(adev->mmu);
	if (ret)
		return ret;

	adev->auxdrv_data =
		(const struct ipu6_auxdrv_data *)auxdev_id->driver_data;
	adev->auxdrv = to_auxiliary_drv(dev->driver);

	psys = devm_drm_dev_alloc(dev, &ipu6_psys_drm_driver, struct ipu6_psys,
				  drm_dev);
	if (IS_ERR(psys)) {
		ret = PTR_ERR(psys);
		goto out_cleanup;
	}

	psys->adev = adev;
	psys->pdata = adev->pdata;
	psys->icache_prefetch_sp = 0;

	spin_lock_init(&psys->ready_lock);
	spin_lock_init(&psys->pgs_lock);
	spin_lock_init(&psys->queues_lock);
	psys->ready = 0;
	psys->timeout = IPU6_PSYS_CMD_TIMEOUT_MS;

	mutex_init(&psys->mutex);
	INIT_LIST_HEAD(&psys->fhs);
	INIT_LIST_HEAD(&psys->pgs);

	init_waitqueue_head(&psys->psys_wq);
	atomic_set(&psys->wakeup, 0);

	psys->sched_cmd_thread = kthread_run(ipu6_psys_cmd_thread, psys,
					     "psys_cmd_thread");
	if (IS_ERR(psys->sched_cmd_thread)) {
		psys->sched_cmd_thread = NULL;
		mutex_destroy(&psys->mutex);
		goto out_cleanup;
	}

	if (ipu6_ver == IPU6_VER_6SE)
		bitmap_zero(psys->cmd_queues,
			    IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID);
	else
		bitmap_zero(psys->cmd_queues,
			    IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID);

	for (i = 0; i < IPU6_PSYS_PG_POOL_SIZE; i++) {
		kpg = kzalloc(sizeof(*kpg), GFP_KERNEL);
		if (!kpg)
			goto out_free_pgs;
		kpg->pg = dma_alloc_attrs(dev, IPU6_PSYS_PG_MAX_SIZE,
					  &kpg->pg_dma_addr,
					  GFP_KERNEL, 0);
		if (!kpg->pg) {
			kfree(kpg);
			goto out_free_pgs;
		}
		kpg->size = IPU6_PSYS_PG_MAX_SIZE;
		list_add(&kpg->list, &psys->pgs);
	}

	dev_info(dev, "pkg_dir entry count:%d\n",
		 ipu6_cpd_pkg_dir_get_num_entries(adev->pkg_dir));

	ret = ipu6_psys_fw_init(psys);
	if (ret) {
		dev_err(dev, "FW init failed(%d)\n", ret);
		goto out_free_pgs;
	}

	ret = drm_dev_register(&psys->drm_dev, 0);
	if (ret < 0) {
		dev_err(dev, "psys register drm device failed\n");
		goto out_release_fw_com;
	}

	dev_set_drvdata(dev, psys);

	ipu6_mmu_hw_cleanup(adev->mmu);

	return 0;

out_release_fw_com:
	ipu6_fw_com_release(psys->fwcom, 1);
out_free_pgs:
	list_for_each_entry_safe(kpg, kpg0, &psys->pgs, list) {
		dma_free_attrs(dev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
		kfree(kpg);
	}

	mutex_destroy(&psys->mutex);
	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}
out_cleanup:
	ipu6_mmu_hw_cleanup(adev->mmu);

	return ret;
}

static void ipu6_psys_remove(struct auxiliary_device *auxdev)
{
	struct device *dev = &auxdev->dev;
	struct ipu6_psys *psys = dev_get_drvdata(&auxdev->dev);
	struct ipu6_psys_pg *kpg, *kpg0;

	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}

	list_for_each_entry_safe(kpg, kpg0, &psys->pgs, list) {
		dma_free_attrs(dev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
		kfree(kpg);
	}

	if (psys->fwcom && ipu6_fw_com_release(psys->fwcom, 1))
		dev_err(dev, "fw com release failed.\n");

	mutex_destroy(&psys->mutex);
	drm_dev_unplug(&psys->drm_dev);
}

static irqreturn_t psys_isr_threaded(struct ipu6_bus_device *adev)
{
	struct ipu6_psys *psys = ipu6_bus_get_drvdata(adev);
	struct device *dev = &psys->adev->auxdev.dev;
	void __iomem *base = psys->pdata->base;
	u32 status;
	int r;

	mutex_lock(&psys->mutex);
	r = pm_runtime_get_if_in_use(dev);
	if (!r || WARN_ON_ONCE(r < 0)) {
		mutex_unlock(&psys->mutex);
		return IRQ_NONE;
	}

	status = readl(base + IPU6_REG_PSYS_GPDEV_IRQ_STATUS);
	writel(status, base + IPU6_REG_PSYS_GPDEV_IRQ_CLEAR);

	if (status & IPU6_PSYS_GPDEV_IRQ_FWIRQ(IPU6_PSYS_GPDEV_FWIRQ0)) {
		writel(0, base + IPU6_REG_PSYS_GPDEV_FWIRQ(0));
		ipu6_psys_handle_events(psys);
	}

	pm_runtime_put(dev);
	mutex_unlock(&psys->mutex);

	return status ? IRQ_HANDLED : IRQ_NONE;
}

static const struct ipu6_auxdrv_data ipu6_psys_auxdrv_data = {
	.isr_threaded = psys_isr_threaded,
	.wake_isr_thread = true,
};

static const struct auxiliary_device_id ipu6_psys_id_table[] = {
	{
		.name = "intel_ipu6.psys",
		.driver_data = (kernel_ulong_t)&ipu6_psys_auxdrv_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, ipu6_psys_id_table);

static struct auxiliary_driver ipu6_psys_aux_driver = {
	.name = IPU6_PSYS_NAME,
	.probe = ipu6_psys_probe,
	.remove = ipu6_psys_remove,
	.id_table = ipu6_psys_id_table,
	.driver = {
		.pm = &psys_pm_ops,
	},
};
module_auxiliary_driver(ipu6_psys_aux_driver);

MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU processing system driver");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_IMPORT_NS(INTEL_IPU6);
