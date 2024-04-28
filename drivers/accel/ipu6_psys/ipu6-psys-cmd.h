/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 Intel Corporation
 */

#ifndef IPU6_PSYS_CMD_H
#define IPU6_PSYS_CMD_H

#include <drm/drm_file.h>
#include <linux/dma-fence.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/timer_types.h>
#include <linux/types.h>

#include <uapi/drm/ipu6_psys_accel.h>

#include "ipu6-psys.h"
#include "ipu6-psys-fw.h"

#define IPU6_PSYS_BUF_SET_POOL_SIZE 8
#define IPU6_PSYS_BUF_SET_MAX_SIZE 1024

struct ipu6_psys_fw_buffer_set;

enum SCHED_LIST {
	SCHED_START_LIST = 2,
	SCHED_STOP_LIST,
};

enum ipu6_psys_cmd_state {
	KCMD_STATE_PPG_NEW,
	KCMD_STATE_PPG_START,
	KCMD_STATE_PPG_ENQUEUE,
	KCMD_STATE_PPG_STOP,
	KCMD_STATE_PPG_COMPLETE
};

enum ipu6_psys_ppg_state {
	PPG_STATE_START = (1 << 0),
	PPG_STATE_STARTING = (1 << 1),
	PPG_STATE_STARTED = (1 << 2),
	PPG_STATE_RUNNING = (1 << 3),
	PPG_STATE_SUSPEND = (1 << 4),
	PPG_STATE_SUSPENDING = (1 << 5),
	PPG_STATE_SUSPENDED = (1 << 6),
	PPG_STATE_RESUME = (1 << 7),
	PPG_STATE_RESUMING = (1 << 8),
	PPG_STATE_RESUMED = (1 << 9),
	PPG_STATE_STOP = (1 << 10),
	PPG_STATE_STOPPING = (1 << 11),
	PPG_STATE_STOPPED = (1 << 12),
};

struct ipu6_psys_ppg {
	struct ipu6_psys_pg *kpg;
	struct ipu6_psys_fh *fh;
	struct list_head list;
	u64 token;
	void *manifest;
	struct mutex mutex;     /* Protects kcmd and ppg state field */
	struct list_head kcmds_new;
	struct list_head kcmds_running;
	struct list_head kcmds_complete;
	enum ipu6_psys_ppg_state state;
	u32 priority;
};

struct ipu6_psys_kcmd {
	struct ipu6_psys_fh *fh;
	struct list_head list;
	struct ipu6_psys_buffer_set *kbuf_set;
	enum ipu6_psys_cmd_state state;
	void *pg_manifest;
	size_t pg_manifest_size;
	u32 *handles;
	struct ipu6_psys_kbuffer **kbufs;
	struct ipu6_psys_buffer *buffers;
	struct drm_gem_object **bos;
	size_t nbuffers;
	struct dma_fence *complete_fence;
	struct ipu6_psys_fw_process_group *pg_user;
	struct ipu6_psys_pg *kpg;
	u64 user_token;
	u64 issue_id;
	u32 priority;
	u32 kernel_enable_bitmap[4];
	u32 terminal_enable_bitmap[4];
	u32 routing_enable_bitmap[4];
	u32 rbm[5];
	struct timer_list watchdog;
};

struct ipu6_psys_buffer_set {
	struct list_head list;
	struct ipu6_psys_fw_buffer_set *buf_set;
	size_t size;
	size_t buf_set_size;
	dma_addr_t dma_addr;
	void *vaddr;
	struct ipu6_psys_kcmd *kcmd;
};

int ipu6_psys_submit_kcmd(struct drm_file *file, struct ipu6_psys_cmd *cmd);
int ipu6_psys_kcmd_new(struct ipu6_psys_cmd *cmd, struct ipu6_psys_fh *fh);
void ipu6_psys_kcmd_free(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_submit_kcmd(struct drm_file *file, struct ipu6_psys_cmd *cmd);
int ipu6_psys_wait_kcmd(struct drm_file *file, u32 handle, u32 timeout_us);
void ipu6_psys_handle_events(struct ipu6_psys *psys);
struct ipu6_psys_kcmd *ipu6_get_completed_kcmd(struct ipu6_psys_fh *fh);
void ipu6_psys_cmd_run(struct ipu6_psys *psys);
#endif /* IPU6_PSYS_CMD_H */
