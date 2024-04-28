/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 Intel Corporation
 */

#ifndef IPU6_PSYS_H
#define IPU6_PSYS_H

#include <drm/drm_accel.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_prime.h>

#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <media/ipu6/ipu6.h>
#include <media/ipu6/ipu6-bus.h>
#include "ipu6-psys-cmd.h"
#include "ipu6-psys-fw.h"
#include "ipu6-psys-gem.h"

#define IPU6_PSYS_DRM_DRV_NAME "intel_ipu6_psys"

#define IPU6_PSYS_CMD_TIMEOUT_MS	2000
#define IPU6_PSYS_OPEN_TIMEOUT_US	   50
#define IPU6_PSYS_OPEN_RETRY (10000 / IPU6_PSYS_OPEN_TIMEOUT_US)

#define IPU6_PSYS_PG_POOL_SIZE 16
#define IPU6_PSYS_PG_MAX_SIZE 8192
#define IPU6_MAX_PSYS_CMD_BUFFERS 32
#define IPU6_PSYS_EVENT_CMD_COMPLETE IPU6_FW_PSYS_EVENT_TYPE_SUCCESS
#define IPU6_PSYS_EVENT_FRAGMENT_COMPLETE IPU6_FW_PSYS_EVENT_TYPE_SUCCESS
#define IPU6_PSYS_CLOSE_TIMEOUT_US   50
#define IPU6_PSYS_CLOSE_TIMEOUT (100000 / IPU6_PSYS_CLOSE_TIMEOUT_US)

#define IPU6_REG_PSYS_INFO_SEG_CMEM_MASTER(i)	(0x2c + ((i) * 12))
#define IPU6_REG_PSYS_INFO_SEG_XMEM_MASTER(i)	(0x5c + ((i) * 12))
/*
 * psys subdomains power request regs
 */
enum ipu_device_buttress_psys_domain_pos {
	IPU6_PSYS_SUBDOMAIN_ISA		 = 0,
	IPU6_PSYS_SUBDOMAIN_PSA		 = 1,
	IPU6_PSYS_SUBDOMAIN_BB		 = 2,
	IPU6_PSYS_SUBDOMAIN_IDSP1	 = 3, /* only in IPU6M */
	IPU6_PSYS_SUBDOMAIN_IDSP2	 = 4, /* only in IPU6M */
};

#define IPU6_PSYS_SUBDOMAINS_POWER_MASK (BIT(IPU6_PSYS_SUBDOMAIN_ISA) | \
					 BIT(IPU6_PSYS_SUBDOMAIN_PSA) | \
					 BIT(IPU6_PSYS_SUBDOMAIN_BB))
#define IPU6_PSYS_SUBDOMAINS_POWER_REQ		0xa0
#define IPU6_PSYS_SUBDOMAINS_POWER_STATUS	0xa4

extern enum ipu6_version ipu6_ver;
struct task_struct;

struct ipu6_psys {
	struct drm_device drm_dev;
	struct mutex mutex; /* protect psys */

	bool icache_prefetch_sp;
	bool icache_prefetch_isp;
	int ready;
	spinlock_t ready_lock; /* psys power */
	spinlock_t pgs_lock; /* psys ppgs */

	struct list_head fhs;
	struct list_head pgs;

	struct ipu6_psys_pdata *pdata;
	struct ipu6_bus_device *adev;
	DECLARE_BITMAP(cmd_queues, 32);
	spinlock_t queues_lock; /* psys fw cmd queues */
	struct task_struct *sched_cmd_thread;
	wait_queue_head_t psys_wq;
	atomic_t wakeup;

	const struct firmware *fw;
	struct sg_table fw_sgt;
	u64 *pkg_dir;
	dma_addr_t pkg_dir_dma_addr;
	unsigned int pkg_dir_size;

	unsigned long timeout;
	void *fwcom;
};

struct ipu6_psys_cmdq {
	struct list_head ppgs;
	struct mutex bs_mutex; /* protect the bufsets list */
	struct list_head buf_sets;
};

struct ipu6_psys_fh {
	struct ipu6_psys *psys;
	struct mutex mutex;	/* Protects bufmap & kcmds fields */
	struct list_head list;
	struct list_head bufmap;
	struct ipu6_psys_cmdq cmdq;
};

struct ipu6_psys_pg {
	struct ipu6_psys_fw_process_group *pg;
	size_t size;
	size_t pg_size;
	dma_addr_t pg_dma_addr;
	struct list_head list;
};

static inline struct ipu6_psys *to_ipu6_psys(struct drm_device *dev)
{
	return container_of(dev, struct ipu6_psys, drm_dev);
}

#endif /* IPU6_PSYS_H */
