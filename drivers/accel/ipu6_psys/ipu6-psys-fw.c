// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2024 Intel Corporation

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <media/ipu6/ipu6-fw-com.h>
#include <uapi/drm/ipu6_psys_accel.h>

#include "ipu6-psys-cmd.h"
#include "ipu6-psys-fw.h"
#include "ipu6-psys.h"

void ipu6_psys_fw_pg_start(struct ipu6_psys_kcmd *kcmd)
{
	kcmd->kpg->pg->state = IPU6_FW_PSYS_PROCESS_GROUP_STARTED;
}

int ipu6_psys_fw_pg_disown(struct ipu6_psys_kcmd *kcmd)
{
	struct ipu6_psys_fw_cmd *psys_cmd;
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, 0);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		return -ENODATA;
	}
	psys_cmd->command = IPU6_FW_PSYS_PROCESS_GROUP_CMD_START;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kpg->pg->ipu6_virtual_address;
	ipu6_send_put_token(kcmd->fh->psys->fwcom, 0);

	return 0;
}

int ipu6_psys_fw_pg_abort(struct ipu6_psys_kcmd *kcmd)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu6_psys_fw_cmd *psys_cmd;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, 0);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		return -ENODATA;
	}
	psys_cmd->command = IPU6_FW_PSYS_PROCESS_GROUP_CMD_STOP;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kpg->pg->ipu6_virtual_address;
	ipu6_send_put_token(kcmd->fh->psys->fwcom, 0);

	return 0;
}

void ipu6_psys_fw_pg_submit(struct ipu6_psys_kcmd *kcmd)
{
	kcmd->kpg->pg->state = IPU6_FW_PSYS_PROCESS_GROUP_BLOCKED;
}

int ipu6_psys_fw_rcv_event(struct ipu6_psys *psys,
			   struct ipu6_psys_fw_event *event)
{
	void *rcv;

	rcv = ipu6_recv_get_token(psys->fwcom, 0);
	if (!rcv)
		return 0;

	memcpy(event, rcv, sizeof(*event));
	ipu6_recv_put_token(psys->fwcom, 0);
	return 1;
}

int ipu6_psys_fw_terminal_set(struct ipu6_psys_fw_terminal *terminal,
			      int terminal_idx,
			      struct ipu6_psys_kcmd *kcmd,
			      u32 buffer, unsigned int size)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	u32 type;
	u32 buffer_state;

	type = terminal->terminal_type;

	switch (type) {
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PROGRAM:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PROGRAM_CONTROL_INIT:
		buffer_state = IPU6_FW_PSYS_BUFFER_UNDEFINED;
		break;
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_STREAM:
	case IPU6_FW_PSYS_TERMINAL_TYPE_DATA_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_STATE_IN:
		buffer_state = IPU6_FW_PSYS_BUFFER_FULL;
		break;
	case IPU6_FW_PSYS_TERMINAL_TYPE_DATA_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_STATE_OUT:
		buffer_state = IPU6_FW_PSYS_BUFFER_EMPTY;
		break;
	default:
		dev_err(dev, "unknown terminal type: 0x%x\n", type);
		return -EAGAIN;
	}

	if (type == IPU6_FW_PSYS_TERMINAL_TYPE_DATA_IN ||
	    type == IPU6_FW_PSYS_TERMINAL_TYPE_DATA_OUT) {
		struct ipu6_psys_fw_data_terminal *dterminal =
			(struct ipu6_psys_fw_data_terminal *)terminal;
		dterminal->connection_type = IPU6_FW_PSYS_CONNECTION_MEMORY;
		dterminal->frame.data_bytes = size;
		if (!ipu6_psys_fw_pg_get_protocol(kcmd))
			dterminal->frame.data = buffer;
		else
			dterminal->frame.data_index = terminal_idx;
		dterminal->frame.buffer_state = buffer_state;
	} else {
		struct ipu6_psys_fw_param_terminal *pterminal =
			(struct ipu6_psys_fw_param_terminal *)terminal;
		if (!ipu6_psys_fw_pg_get_protocol(kcmd))
			pterminal->param_payload.buffer = buffer;
		else
			pterminal->param_payload.terminal_index = terminal_idx;
	}

	return 0;
}

int ipu6_psys_fw_pg_get_id(struct ipu6_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->ID;
}

int ipu6_psys_fw_pg_get_terminal_count(struct ipu6_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->terminal_count;
}

int ipu6_psys_fw_pg_get_size(struct ipu6_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->size;
}

void ipu6_psys_fw_pg_set_ipu6_vaddress(struct ipu6_psys_kcmd *kcmd,
				       dma_addr_t vaddress)
{
	kcmd->kpg->pg->ipu6_virtual_address = vaddress;
}

struct ipu6_psys_fw_terminal *ipu6_psys_fw_pg_get_terminal(struct ipu6_psys_kcmd
							   *kcmd, int index)
{
	struct ipu6_psys_fw_terminal *terminal;
	u16 *terminal_offset_table;

	terminal_offset_table =
		(uint16_t *)((char *)kcmd->kpg->pg +
			     kcmd->kpg->pg->terminals_offset);
	terminal = (struct ipu6_psys_fw_terminal *)
		((char *)kcmd->kpg->pg + terminal_offset_table[index]);
	return terminal;
}

void ipu6_psys_fw_pg_set_token(struct ipu6_psys_kcmd *kcmd, u64 token)
{
	kcmd->kpg->pg->token = (u64)token;
}

u64 ipu6_psys_fw_pg_get_token(struct ipu6_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->token;
}

int ipu6_psys_fw_pg_get_protocol(struct ipu6_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->protocol_version;
}

int ipu6_psys_fw_ppg_set_buffer_set(struct ipu6_psys_kcmd *kcmd,
				    struct ipu6_psys_fw_terminal *terminal,
				    int terminal_idx, u32 buffer)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu6_psys_fw_buffer_set *buf_set = kcmd->kbuf_set->buf_set;
	u32 buffer_state;
	u32 *buffer_ptr;
	u32 type;

	type = terminal->terminal_type;

	switch (type) {
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PROGRAM:
	case IPU6_FW_PSYS_TERMINAL_TYPE_PROGRAM_CONTROL_INIT:
		buffer_state = IPU6_FW_PSYS_BUFFER_UNDEFINED;
		break;
	case IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_STREAM:
	case IPU6_FW_PSYS_TERMINAL_TYPE_DATA_IN:
	case IPU6_FW_PSYS_TERMINAL_TYPE_STATE_IN:
		buffer_state = IPU6_FW_PSYS_BUFFER_FULL;
		break;
	case IPU6_FW_PSYS_TERMINAL_TYPE_DATA_OUT:
	case IPU6_FW_PSYS_TERMINAL_TYPE_STATE_OUT:
		buffer_state = IPU6_FW_PSYS_BUFFER_EMPTY;
		break;
	default:
		dev_err(dev, "unknown terminal type: 0x%x\n", type);
		return -EAGAIN;
	}

	buffer_ptr = (u32 *)((char *)buf_set + sizeof(*buf_set) +
			     terminal_idx * sizeof(*buffer_ptr));

	*buffer_ptr = buffer;

	if (type == IPU6_FW_PSYS_TERMINAL_TYPE_DATA_IN ||
	    type == IPU6_FW_PSYS_TERMINAL_TYPE_DATA_OUT) {
		struct ipu6_psys_fw_data_terminal *dterminal =
			(struct ipu6_psys_fw_data_terminal *)terminal;
		dterminal->frame.buffer_state = buffer_state;
	}

	return 0;
}

size_t ipu6_psys_fw_ppg_get_buffer_set_size(struct ipu6_psys_kcmd *kcmd)
{
	return (sizeof(struct ipu6_psys_fw_buffer_set) +
		kcmd->kpg->pg->terminal_count * sizeof(u32));
}

void
ipu6_psys_fw_ppg_buffer_set_vaddress(struct ipu6_psys_fw_buffer_set *buf_set,
				     u32 vaddress)
{
	buf_set->ipu6_virtual_address = vaddress;
}

int ipu6_psys_fw_ppg_buffer_set_set_keb(struct ipu6_psys_fw_buffer_set *buf_set,
					u32 *kernel_enable_bitmap)
{
	memcpy(buf_set->kernel_enable_bitmap, (u8 *)kernel_enable_bitmap,
	       sizeof(buf_set->kernel_enable_bitmap));
	return 0;
}

struct ipu6_psys_fw_buffer_set *
ipu6_psys_fw_ppg_create_buffer_set(struct ipu6_psys_kcmd *kcmd,
				   void *vaddr, u32 frame_counter)
{
	struct ipu6_psys_fw_buffer_set *buffer_set = NULL;
	unsigned int i;

	buffer_set = (struct ipu6_psys_fw_buffer_set *)vaddr;

	/*
	 * Set base struct members
	 */
	buffer_set->ipu6_virtual_address = 0;
	buffer_set->process_group_handle = kcmd->kpg->pg->ipu6_virtual_address;
	buffer_set->frame_counter = frame_counter;
	buffer_set->terminal_count = kcmd->kpg->pg->terminal_count;

	/*
	 * Initialize adjacent buffer addresses
	 */
	for (i = 0; i < buffer_set->terminal_count; i++) {
		u32 *buffer =
			(u32 *)((char *)buffer_set +
				sizeof(*buffer_set) + sizeof(u32) * i);

		*buffer = 0;
	}

	return buffer_set;
}

int ipu6_psys_fw_ppg_enqueue_bufs(struct ipu6_psys_kcmd *kcmd)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu6_psys_fw_cmd *psys_cmd;
	unsigned int queue_id;
	int ret = 0;
	unsigned int size;

	if (ipu6_ver == IPU6_VER_6SE)
		size = IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	else
		size = IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	queue_id = kcmd->kpg->pg->base_queue_id;

	if (queue_id >= size)
		return -EINVAL;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, queue_id);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		return -ENODATA;
	}

	psys_cmd->command = IPU6_FW_PSYS_PROCESS_GROUP_CMD_RUN;
	psys_cmd->msg = 0;
	psys_cmd->context_handle =
		kcmd->kbuf_set->buf_set->ipu6_virtual_address;

	ipu6_send_put_token(kcmd->fh->psys->fwcom, queue_id);

	return ret;
}

u8 ipu6_psys_fw_ppg_get_base_queue_id(struct ipu6_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->base_queue_id;
}

void ipu6_psys_fw_ppg_set_base_queue_id(struct ipu6_psys_kcmd *kcmd,
					u8 queue_id)
{
	kcmd->kpg->pg->base_queue_id = queue_id;
}

int ipu6_psys_fw_get_cmd_queue(struct ipu6_psys *psys)
{
	unsigned long p;
	int size, start;

	size = IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	start = IPU6_FW_PSYS_CMD_QUEUE_PPG0_COMMAND_ID;

	if (ipu6_ver == IPU6_VER_6SE) {
		size = IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
		start = IPU6SE_FW_PSYS_CMD_QUEUE_PPG0_COMMAND_ID;
	}

	spin_lock(&psys->queues_lock);
	p = bitmap_find_next_zero_area(psys->cmd_queues, size, start, 1, 0);
	if (p >= size) {
		spin_unlock(&psys->queues_lock);
		return -ENOSPC;
	}
	bitmap_set(psys->cmd_queues, p, 1);
	spin_unlock(&psys->queues_lock);

	return p;
}

void ipu6_psys_fw_put_cmd_queue(struct ipu6_psys *psys, u8 queue_id)
{
	spin_lock(&psys->queues_lock);
	bitmap_clear(psys->cmd_queues, queue_id, 1);
	spin_unlock(&psys->queues_lock);
}

int ipu6_psys_fw_open(struct ipu6_psys *psys)
{
	struct device *dev = &psys->adev->auxdev.dev;
	int retry = IPU6_PSYS_OPEN_RETRY, ret;

	ret = ipu6_fw_com_open(psys->fwcom);
	if (ret) {
		dev_err(dev, "fw com open failed.\n");
		return ret;
	}

	do {
		usleep_range(IPU6_PSYS_OPEN_TIMEOUT_US,
			     IPU6_PSYS_OPEN_TIMEOUT_US + 10);
		ret = ipu6_fw_com_ready(psys->fwcom);
		if (ret) {
			dev_dbg(dev, "psys port open ready!\n");
			break;
		}
		retry--;
	} while (retry > 0);

	if (!retry) {
		dev_err(dev, "psys port open ready failed\n");
		ipu6_fw_com_close(psys->fwcom);
		return -EIO;
	}

	return 0;
}

int ipu6_psys_fw_close(struct ipu6_psys *psys)
{
	struct device *dev = &psys->adev->auxdev.dev;
	int ret;

	ret = ipu6_fw_com_close(psys->fwcom);
	if (ret) {
		dev_err(dev, "fw com close failed.\n");
		return ret;
	}
	return ret;
}
