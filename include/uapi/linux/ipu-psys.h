/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Copyright (C) 2013 - 2020 Intel Corporation */

#ifndef _UAPI_IPU_PSYS_H
#define _UAPI_IPU_PSYS_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

struct ipu_psys_capability {
	__u32 version;
	__u8 driver[20];
	__u32 pg_count;
	__u8 dev_model[32];
	__u32 reserved[17];
} __attribute__ ((packed));

struct ipu_psys_event {
	__u32 type;		/* IPU_PSYS_EVENT_TYPE_ */
	__u64 user_token;
	__u64 issue_id;
	__u32 buffer_idx;
	__u32 error;
	__u32 reserved[2];
} __attribute__ ((packed));

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
	__u64 len;
	union {
		int fd;
		__u64 userptr;
		__u64 reserved;
	} base;
	__u32 data_offset;
	__u32 bytes_used;
	__u32 flags;
	__u32 reserved[2];
} __attribute__ ((packed));

#define IPU_BUFFER_FLAG_INPUT	(1 << 0)
#define IPU_BUFFER_FLAG_OUTPUT	(1 << 1)
#define IPU_BUFFER_FLAG_MAPPED	(1 << 2)
#define IPU_BUFFER_FLAG_NO_FLUSH	(1 << 3)
#define IPU_BUFFER_FLAG_DMA_HANDLE	(1 << 4)
#define IPU_BUFFER_FLAG_USERPTR	(1 << 5)

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
	__u64 issue_id;
	__u64 user_token;
	__u32 priority;
	void __user *pg_manifest;
	struct ipu_psys_buffer __user *buffers;
	int pg;
	__u32 pg_manifest_size;
	__u32 bufcount;
	__u32 min_psys_freq;
	__u32 frame_counter;
	__u32 kernel_enable_bitmap[4];
	__u32 terminal_enable_bitmap[4];
	__u32 routing_enable_bitmap[4];
	__u32 rbm[5];
	__u32 reserved[2];
} __attribute__ ((packed));

struct ipu_psys_manifest {
	__u32 index;
	__u32 size;
	void __user *manifest;
	__u32 reserved[5];
} __attribute__ ((packed));

#define IPU_IOC_QUERYCAP _IOR('A', 1, struct ipu_psys_capability)
#define IPU_IOC_MAPBUF _IOWR('A', 2, int)
#define IPU_IOC_UNMAPBUF _IOWR('A', 3, int)
#define IPU_IOC_GETBUF _IOWR('A', 4, struct ipu_psys_buffer)
#define IPU_IOC_PUTBUF _IOWR('A', 5, struct ipu_psys_buffer)
#define IPU_IOC_QCMD _IOWR('A', 6, struct ipu_psys_command)
#define IPU_IOC_DQEVENT _IOWR('A', 7, struct ipu_psys_event)
#define IPU_IOC_CMD_CANCEL _IOWR('A', 8, struct ipu_psys_command)
#define IPU_IOC_GET_MANIFEST _IOWR('A', 9, struct ipu_psys_manifest)

#endif /* _UAPI_IPU_PSYS_H */
