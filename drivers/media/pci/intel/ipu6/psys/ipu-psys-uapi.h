/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Copyright (C) 2013 - 2020 Intel Corporation */

#ifndef _IPU_PSYS_UAPI_H
#define _IPU_PSYS_UAPI_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

struct ipu_psys_capability {
	u32 version;
	u8 driver[20];
	u32 pg_count;
	u8 dev_model[32];
	u32 reserved[17];
} __packed;

struct ipu_psys_event {
	u32 type;		/* IPU_PSYS_EVENT_TYPE_ */
	u64 user_token;
	u64 issue_id;
	u32 buffer_idx;
	u32 error;
	s32 reserved[2];
} __packed;

#define IPU_PSYS_EVENT_TYPE_CMD_COMPLETE	1
#define IPU_PSYS_EVENT_TYPE_BUFFER_COMPLETE	2

/**
 * struct ipu_psys_buffer - for input/output terminals
 * @len:	total allocated size @ base address
 * @userptr:	user pointer
 * @fd:		DMA-BUF handle
 * @data_offset:offset to valid data
 * @bytes_used:	amount of valid data including offset
 * @flags:	flags
 */
struct ipu_psys_buffer {
	u64 len;
	union {
		int fd;
		void __user *userptr;
		u64 reserved;
	} base;
	u32 data_offset;
	u32 bytes_used;
	u32 flags;
	u32 reserved[2];
} __packed;

#define IPU_BUFFER_FLAG_INPUT		BIT(0)
#define IPU_BUFFER_FLAG_OUTPUT		BIT(1)
#define IPU_BUFFER_FLAG_MAPPED		BIT(2)
#define IPU_BUFFER_FLAG_NO_FLUSH	BIT(3)
#define IPU_BUFFER_FLAG_DMA_HANDLE	BIT(4)
#define IPU_BUFFER_FLAG_USERPTR		BIT(5)

#define	IPU_PSYS_CMD_PRIORITY_HIGH	0
#define	IPU_PSYS_CMD_PRIORITY_MED	1
#define	IPU_PSYS_CMD_PRIORITY_LOW	2
#define	IPU_PSYS_CMD_PRIORITY_NUM	3

/**
 * struct ipu_psys_command - processing command
 * @issue_id:		unique id for the command set by user
 * @user_token:		token of the command
 * @priority:		priority of the command
 * @pg_manifest:	userspace pointer to program group manifest
 * @buffers:		userspace pointers to array of psys dma buf structs
 * @pg:			process group DMA-BUF handle
 * @pg_manifest_size:	size of program group manifest
 * @bufcount:		number of buffers in buffers array
 * @min_psys_freq:	minimum psys frequency in MHz used for this cmd
 * @frame_counter:      counter of current frame synced between isys and psys
 * @kernel_enable_bitmap:       enable bits for each individual kernel
 * @terminal_enable_bitmap:     enable bits for each individual terminals
 * @routing_enable_bitmap:      enable bits for each individual routing
 * @rbm:                        enable bits for routing
 *
 * Specifies a processing command with input and output buffers.
 */
struct ipu_psys_command {
	u64 issue_id;
	u64 user_token;
	u32 priority;
	void __user *pg_manifest;
	struct ipu_psys_buffer __user *buffers;
	int pg;
	u32 pg_manifest_size;
	u32 bufcount;
	u32 min_psys_freq;
	u32 frame_counter;
	u32 kernel_enable_bitmap[4];
	u32 terminal_enable_bitmap[4];
	u32 routing_enable_bitmap[4];
	u32 rbm[5];
	u32 reserved[2];
} __packed;

struct ipu_psys_manifest {
	u32 index;
	u32 size;
	void __user *manifest;
	u32 reserved[5];
} __packed;

#define IPU_IOC_QUERYCAP _IOR('A', 1, struct ipu_psys_capability)
#define IPU_IOC_MAPBUF _IOWR('A', 2, int)
#define IPU_IOC_UNMAPBUF _IOWR('A', 3, int)
#define IPU_IOC_GETBUF _IOWR('A', 4, struct ipu_psys_buffer)
#define IPU_IOC_PUTBUF _IOWR('A', 5, struct ipu_psys_buffer)
#define IPU_IOC_QCMD _IOWR('A', 6, struct ipu_psys_command)
#define IPU_IOC_DQEVENT _IOWR('A', 7, struct ipu_psys_event)
#define IPU_IOC_CMD_CANCEL _IOWR('A', 8, struct ipu_psys_command)
#define IPU_IOC_GET_MANIFEST _IOWR('A', 9, struct ipu_psys_manifest)

#endif /* _IPU_PSYS_UAPI_H */
