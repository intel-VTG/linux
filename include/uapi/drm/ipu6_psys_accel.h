/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (C) 2020-2023 Intel Corporation
 */

#ifndef __IPU6_PSYS_ACCEL_H__
#define __IPU6_PSYS_ACCEL_H__

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/**
 * struct ipu6_psys_buffer - for input/output terminals
 * @len:	total allocated size @ base address
 * @userptr:	user pointer
 * @fd:		DMA-BUF handle
 * @data_offset:offset to valid data
 * @bytes_used:	amount of valid data including offset
 * @flags:	flags
 */
struct ipu6_psys_buffer {
	__u64 len;
	__s32 fd;
	__u32 handle;
	__u32 data_offset;
	__u32 bytes_used;
	__u32 flags;
	__u32 reserved[2];
} __attribute__ ((packed));

struct ipu6_psys_buffer_info {
};

#define IPU6_BUFFER_FLAG_INPUT	(1 << 0)
#define IPU6_BUFFER_FLAG_OUTPUT	(1 << 1)
#define IPU6_BUFFER_FLAG_MAPPED	(1 << 2)
#define IPU6_BUFFER_FLAG_NO_FLUSH	(1 << 3)
#define IPU6_BUFFER_FLAG_DMA_HANDLE	(1 << 4)
#define IPU6_BUFFER_FLAG_USERPTR	(1 << 5)

/**
 * struct ipu6_psys_cmd - processing command
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
struct ipu6_psys_cmd {
	__u64 issue_id;
	__u64 user_token;
	__u32 priority;
	void __user *pg_manifest;
	void __user *handles; /* buffer handles array ptr */
	struct ipu6_psys_buffer __user *buffers;
	int pg;
	__u32 pg_manifest_size;
	__u32 min_psys_freq;
	__u32 frame_counter;
	__u32 kernel_enable_bitmap[4];
	__u32 terminal_enable_bitmap[4];
	__u32 routing_enable_bitmap[4];
	__u32 rbm[5];
	__u32 reserved[2];
} __attribute__ ((packed));

struct ipu6_psys_wait {
	__u32 handle;
	__u32 timeout_us;
} __attribute__ ((packed));

struct ipu6_psys_manifest {
	__u32 index;
	__u32 size;
	void __user *manifest;
	__u32 reserved[5];
} __attribute__ ((packed));

struct ipu6_psys_map {
	__u32 fd;
	__u32 handle;
	__u32 flags;
} __attribute__ ((packed));

#define	DRM_IPU6_PS_GET_MANIFEST		0x0
#define	DRM_IPU6_PS_CREATE_BO			0x1
#define	DRM_IPU6_PS_GET_BO_INFO			0x2
#define	DRM_IPU6_PS_BO_MAP			0x3
#define	DRM_IPU6_PS_BO_UNMAP			0x4
#define	DRM_IPU6_PS_SUBMIT_CMD			0x5
#define	DRM_IPU6_PS_BO_WAIT			0x6

#define DRM_IOCTL_IPU6_PS_GET_MANIFEST				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_GET_MANIFEST,	\
		 struct ipu6_psys_manifest)

#define DRM_IOCTL_IPU6_PS_CREATE_BO				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_CREATE_BO,	\
		 struct ipu6_psys_buffer)

#define DRM_IOCTL_IPU6_PS_GET_BO_INFO				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_GET_BO_INFO,	\
		 struct ipu6_psys_buffer_info)

#define DRM_IOCTL_IPU6_PS_BO_MAP				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_BO_MAP,	\
		 struct ipu6_psys_map)

#define DRM_IOCTL_IPU6_PS_BO_UNMAP				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_BO_UNMAP,	\
		 struct ipu6_psys_map)

#define DRM_IOCTL_IPU6_PS_SUBMIT_CMD				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_SUBMIT_CMD,		\
		 struct ipu6_psys_cmd)

#define DRM_IOCTL_IPU6_PS_BO_WAIT				\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_IPU6_PS_BO_WAIT,	\
		 struct ipu6_psys_buffer)
#endif /* __IPU6_PSYS_ACCEL_H__ */
