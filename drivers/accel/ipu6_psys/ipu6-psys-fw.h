/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 Intel Corporation
 */

#ifndef IPU6_PSYS_FW_H
#define IPU6_PSYS_FW_H

#define IPU6_FW_PSYS_CMD_QUEUE_SIZE	0x20
#define IPU6_FW_PSYS_EVENT_QUEUE_SIZE	0x40

#define IPU6_FW_PSYS_CMD_BITS		64
#define IPU6_FW_PSYS_EVENT_BITS		128

enum {
	IPU6_FW_PSYS_CMD_QUEUE_COMMAND_ID = 0,
	IPU6_FW_PSYS_CMD_QUEUE_DEVICE_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG0_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG1_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG2_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG3_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG4_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG5_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG6_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG7_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG8_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG9_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG10_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG11_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG12_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG13_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG14_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG15_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG16_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG17_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG18_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG19_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG20_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG21_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG22_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG23_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG24_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG25_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG26_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG27_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG28_COMMAND_ID,
	IPU6_FW_PSYS_CMD_QUEUE_PPG29_COMMAND_ID,
	IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID
};

enum {
	IPU6_FW_PSYS_TRANSFER_VMEM0_TYPE_ID = 0,
	IPU6_FW_PSYS_TRANSFER_VMEM1_TYPE_ID,
	IPU6_FW_PSYS_LB_VMEM_TYPE_ID,
	IPU6_FW_PSYS_DMEM_TYPE_ID,
	IPU6_FW_PSYS_VMEM_TYPE_ID,
	IPU6_FW_PSYS_BAMEM_TYPE_ID,
	IPU6_FW_PSYS_PMEM_TYPE_ID,
	IPU6_FW_PSYS_N_MEM_TYPE_ID
};

enum ipu6_mem_id {
	IPU6_FW_PSYS_VMEM0_ID = 0,	/* ISP0 VMEM */
	IPU6_FW_PSYS_TRANSFER_VMEM0_ID,	/* TRANSFER VMEM 0 */
	IPU6_FW_PSYS_TRANSFER_VMEM1_ID,	/* TRANSFER VMEM 1 */
	IPU6_FW_PSYS_LB_VMEM_ID,	/* LB VMEM */
	IPU6_FW_PSYS_BAMEM0_ID,	/* ISP0 BAMEM */
	IPU6_FW_PSYS_DMEM0_ID,	/* SPC0 Dmem */
	IPU6_FW_PSYS_DMEM1_ID,	/* SPP0 Dmem */
	IPU6_FW_PSYS_DMEM2_ID,	/* SPP1 Dmem */
	IPU6_FW_PSYS_DMEM3_ID,	/* ISP0 Dmem */
	IPU6_FW_PSYS_PMEM0_ID,	/* ISP0 PMEM */
	IPU6_FW_PSYS_N_MEM_ID
};

enum {
	IPU6_FW_PSYS_DEV_CHN_DMA_EXT0_ID = 0,
	IPU6_FW_PSYS_DEV_CHN_DMA_EXT1_READ_ID,
	IPU6_FW_PSYS_DEV_CHN_DMA_EXT1_WRITE_ID,
	IPU6_FW_PSYS_DEV_CHN_DMA_INTERNAL_ID,
	IPU6_FW_PSYS_DEV_CHN_DMA_ISA_ID,
	IPU6_FW_PSYS_N_DEV_CHN_ID
};

enum {
	IPU6_FW_PSYS_SP_CTRL_TYPE_ID = 0,
	IPU6_FW_PSYS_SP_SERVER_TYPE_ID,
	IPU6_FW_PSYS_VP_TYPE_ID,
	IPU6_FW_PSYS_ACC_PSA_TYPE_ID,
	IPU6_FW_PSYS_ACC_ISA_TYPE_ID,
	IPU6_FW_PSYS_ACC_OSA_TYPE_ID,
	IPU6_FW_PSYS_GDC_TYPE_ID,
	IPU6_FW_PSYS_TNR_TYPE_ID,
	IPU6_FW_PSYS_N_CELL_TYPE_ID
};

enum {
	IPU6_FW_PSYS_SP0_ID = 0,
	IPU6_FW_PSYS_VP0_ID,
	IPU6_FW_PSYS_PSA_ACC_BNLM_ID,
	IPU6_FW_PSYS_PSA_ACC_DM_ID,
	IPU6_FW_PSYS_PSA_ACC_ACM_ID,
	IPU6_FW_PSYS_PSA_ACC_GTC_YUV1_ID,
	IPU6_FW_PSYS_BB_ACC_OFS_PIN_MAIN_ID,
	IPU6_FW_PSYS_BB_ACC_OFS_PIN_DISPLAY_ID,
	IPU6_FW_PSYS_BB_ACC_OFS_PIN_PP_ID,
	IPU6_FW_PSYS_PSA_ACC_GAMMASTAR_ID,
	IPU6_FW_PSYS_PSA_ACC_GLTM_ID,
	IPU6_FW_PSYS_PSA_ACC_XNR_ID,
	IPU6_FW_PSYS_PSA_VCSC_ID,	/* VCSC */
	IPU6_FW_PSYS_ISA_ICA_ID,
	IPU6_FW_PSYS_ISA_LSC_ID,
	IPU6_FW_PSYS_ISA_DPC_ID,
	IPU6_FW_PSYS_ISA_SIS_A_ID,
	IPU6_FW_PSYS_ISA_SIS_B_ID,
	IPU6_FW_PSYS_ISA_B2B_ID,
	IPU6_FW_PSYS_ISA_B2R_R2I_SIE_ID,
	IPU6_FW_PSYS_ISA_R2I_DS_A_ID,
	IPU6_FW_PSYS_ISA_R2I_DS_B_ID,
	IPU6_FW_PSYS_ISA_AWB_ID,
	IPU6_FW_PSYS_ISA_AE_ID,
	IPU6_FW_PSYS_ISA_AF_ID,
	IPU6_FW_PSYS_ISA_DOL_ID,
	IPU6_FW_PSYS_ISA_ICA_MEDIUM_ID,
	IPU6_FW_PSYS_ISA_X2B_MD_ID,
	IPU6_FW_PSYS_ISA_X2B_SVE_RGBIR_ID,
	IPU6_FW_PSYS_ISA_PAF_ID,
	IPU6_FW_PSYS_BB_ACC_GDC0_ID,
	IPU6_FW_PSYS_BB_ACC_TNR_ID,
	IPU6_FW_PSYS_N_CELL_ID
};

enum {
	IPU6_FW_PSYS_DEV_DFM_BB_FULL_PORT_ID = 0,
	IPU6_FW_PSYS_DEV_DFM_BB_EMPTY_PORT_ID,
	IPU6_FW_PSYS_DEV_DFM_ISL_FULL_PORT_ID,
	IPU6_FW_PSYS_DEV_DFM_ISL_EMPTY_PORT_ID,
	IPU6_FW_PSYS_DEV_DFM_LB_FULL_PORT_ID,
	IPU6_FW_PSYS_DEV_DFM_LB_EMPTY_PORT_ID,
};

enum {
	IPU6SE_FW_PSYS_CMD_QUEUE_COMMAND_ID = 0,
	IPU6SE_FW_PSYS_CMD_QUEUE_DEVICE_ID,
	IPU6SE_FW_PSYS_CMD_QUEUE_PPG0_COMMAND_ID,
	IPU6SE_FW_PSYS_CMD_QUEUE_PPG1_COMMAND_ID,
	IPU6SE_FW_PSYS_CMD_QUEUE_PPG2_COMMAND_ID,
	IPU6SE_FW_PSYS_CMD_QUEUE_PPG3_COMMAND_ID,
	IPU6SE_FW_PSYS_CMD_QUEUE_PPG4_COMMAND_ID,
	IPU6SE_FW_PSYS_CMD_QUEUE_PPG5_COMMAND_ID,
	IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID
};

enum {
	IPU6SE_FW_PSYS_TRANSFER_VMEM0_TYPE_ID = 0,
	IPU6SE_FW_PSYS_LB_VMEM_TYPE_ID,
	IPU6SE_FW_PSYS_DMEM_TYPE_ID,
	IPU6SE_FW_PSYS_VMEM_TYPE_ID,
	IPU6SE_FW_PSYS_BAMEM_TYPE_ID,
	IPU6SE_FW_PSYS_PMEM_TYPE_ID,
	IPU6SE_FW_PSYS_N_MEM_TYPE_ID
};

enum ipu6se_mem_id {
	IPU6SE_FW_PSYS_TRANSFER_VMEM0_ID = 0,	/* TRANSFER VMEM 0 */
	IPU6SE_FW_PSYS_LB_VMEM_ID,	/* LB VMEM */
	IPU6SE_FW_PSYS_DMEM0_ID,	/* SPC0 Dmem */
	IPU6SE_FW_PSYS_DMEM1_ID,	/* SPP0 Dmem */
	IPU6SE_FW_PSYS_N_MEM_ID
};

enum {
	IPU6SE_FW_PSYS_DEV_CHN_DMA_EXT0_ID = 0,
	IPU6SE_FW_PSYS_DEV_CHN_DMA_EXT1_READ_ID,
	IPU6SE_FW_PSYS_DEV_CHN_DMA_EXT1_WRITE_ID,
	IPU6SE_FW_PSYS_DEV_CHN_DMA_ISA_ID,
	IPU6SE_FW_PSYS_N_DEV_CHN_ID
};

enum {
	IPU6SE_FW_PSYS_SP_CTRL_TYPE_ID = 0,
	IPU6SE_FW_PSYS_SP_SERVER_TYPE_ID,
	IPU6SE_FW_PSYS_ACC_ISA_TYPE_ID,
	IPU6SE_FW_PSYS_N_CELL_TYPE_ID
};

enum {
	IPU6SE_FW_PSYS_SP0_ID = 0,
	IPU6SE_FW_PSYS_ISA_ICA_ID,
	IPU6SE_FW_PSYS_ISA_LSC_ID,
	IPU6SE_FW_PSYS_ISA_DPC_ID,
	IPU6SE_FW_PSYS_ISA_B2B_ID,
	IPU6SE_FW_PSYS_ISA_BNLM_ID,
	IPU6SE_FW_PSYS_ISA_DM_ID,
	IPU6SE_FW_PSYS_ISA_R2I_SIE_ID,
	IPU6SE_FW_PSYS_ISA_R2I_DS_A_ID,
	IPU6SE_FW_PSYS_ISA_R2I_DS_B_ID,
	IPU6SE_FW_PSYS_ISA_AWB_ID,
	IPU6SE_FW_PSYS_ISA_AE_ID,
	IPU6SE_FW_PSYS_ISA_AF_ID,
	IPU6SE_FW_PSYS_ISA_PAF_ID,
	IPU6SE_FW_PSYS_N_CELL_ID
};

enum {
	IPU6SE_FW_PSYS_DEV_DFM_ISL_FULL_PORT_ID = 0,
	IPU6SE_FW_PSYS_DEV_DFM_ISL_EMPTY_PORT_ID,
};

enum {
	IPU6EP_FW_PSYS_SP0_ID = 0,
	IPU6EP_FW_PSYS_VP0_ID,
	IPU6EP_FW_PSYS_PSA_ACC_BNLM_ID,
	IPU6EP_FW_PSYS_PSA_ACC_DM_ID,
	IPU6EP_FW_PSYS_PSA_ACC_ACM_ID,
	IPU6EP_FW_PSYS_PSA_ACC_GTC_YUV1_ID,
	IPU6EP_FW_PSYS_BB_ACC_OFS_PIN_MAIN_ID,
	IPU6EP_FW_PSYS_BB_ACC_OFS_PIN_DISPLAY_ID,
	IPU6EP_FW_PSYS_BB_ACC_OFS_PIN_PP_ID,
	IPU6EP_FW_PSYS_PSA_ACC_GAMMASTAR_ID,
	IPU6EP_FW_PSYS_PSA_ACC_GLTM_ID,
	IPU6EP_FW_PSYS_PSA_ACC_XNR_ID,
	IPU6EP_FW_PSYS_PSA_VCSC_ID,	/* VCSC */
	IPU6EP_FW_PSYS_ISA_ICA_ID,
	IPU6EP_FW_PSYS_ISA_LSC_ID,
	IPU6EP_FW_PSYS_ISA_DPC_ID,
	IPU6EP_FW_PSYS_ISA_SIS_A_ID,
	IPU6EP_FW_PSYS_ISA_SIS_B_ID,
	IPU6EP_FW_PSYS_ISA_B2B_ID,
	IPU6EP_FW_PSYS_ISA_B2R_R2I_SIE_ID,
	IPU6EP_FW_PSYS_ISA_R2I_DS_A_ID,
	IPU6EP_FW_PSYS_ISA_AWB_ID,
	IPU6EP_FW_PSYS_ISA_AE_ID,
	IPU6EP_FW_PSYS_ISA_AF_ID,
	IPU6EP_FW_PSYS_ISA_X2B_MD_ID,
	IPU6EP_FW_PSYS_ISA_X2B_SVE_RGBIR_ID,
	IPU6EP_FW_PSYS_ISA_PAF_ID,
	IPU6EP_FW_PSYS_BB_ACC_GDC0_ID,
	IPU6EP_FW_PSYS_BB_ACC_TNR_ID,
	IPU6EP_FW_PSYS_N_CELL_ID
};

enum {
	IPU6_FW_PSYS_EVENT_TYPE_SUCCESS = 0,
	IPU6_FW_PSYS_EVENT_TYPE_UNKNOWN_ERROR = 1,
	IPU6_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_NOT_FOUND = 2,
	IPU6_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_TOO_BIG = 3,
	IPU6_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_DDR_TRANS_ERR = 4,
	IPU6_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_NULL_PKG_DIR_ADDR = 5,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_LOAD_FRAME_ERR = 6,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_LOAD_FRAGMENT_ERR = 7,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_PROCESS_COUNT_ZERO = 8,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_PROCESS_INIT_ERR = 9,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_ABORT = 10,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_NULL = 11,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_VALIDATION_ERR = 12,
	IPU6_FW_PSYS_EVENT_TYPE_PROC_GRP_INVALID_FRAME = 13
};

enum {
	IPU6_FW_PSYS_EVENT_QUEUE_MAIN_ID,
	IPU6_FW_PSYS_N_PSYS_EVENT_QUEUE_ID
};

enum {
	IPU6_FW_PSYS_PROCESS_GROUP_ERROR = 0,
	IPU6_FW_PSYS_PROCESS_GROUP_CREATED,
	IPU6_FW_PSYS_PROCESS_GROUP_READY,
	IPU6_FW_PSYS_PROCESS_GROUP_BLOCKED,
	IPU6_FW_PSYS_PROCESS_GROUP_STARTED,
	IPU6_FW_PSYS_PROCESS_GROUP_RUNNING,
	IPU6_FW_PSYS_PROCESS_GROUP_STALLED,
	IPU6_FW_PSYS_PROCESS_GROUP_STOPPED,
	IPU6_FW_PSYS_N_PROCESS_GROUP_STATES
};

enum {
	IPU6_FW_PSYS_CONNECTION_MEMORY = 0,
	IPU6_FW_PSYS_CONNECTION_MEMORY_STREAM,
	IPU6_FW_PSYS_CONNECTION_STREAM,
	IPU6_FW_PSYS_N_CONNECTION_TYPES
};

enum {
	IPU6_FW_PSYS_BUFFER_NULL = 0,
	IPU6_FW_PSYS_BUFFER_UNDEFINED,
	IPU6_FW_PSYS_BUFFER_EMPTY,
	IPU6_FW_PSYS_BUFFER_NONEMPTY,
	IPU6_FW_PSYS_BUFFER_FULL,
	IPU6_FW_PSYS_N_BUFFER_STATES
};

enum {
	IPU6_FW_PSYS_TERMINAL_TYPE_DATA_IN = 0,
	IPU6_FW_PSYS_TERMINAL_TYPE_DATA_OUT,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_STREAM,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_IN,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_OUT,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_IN,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_OUT,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_IN,
	IPU6_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_OUT,
	IPU6_FW_PSYS_TERMINAL_TYPE_STATE_IN,
	IPU6_FW_PSYS_TERMINAL_TYPE_STATE_OUT,
	IPU6_FW_PSYS_TERMINAL_TYPE_PROGRAM,
	IPU6_FW_PSYS_TERMINAL_TYPE_PROGRAM_CONTROL_INIT,
	IPU6_FW_PSYS_N_TERMINAL_TYPES
};

enum {
	IPU6_FW_PSYS_COL_DIMENSION = 0,
	IPU6_FW_PSYS_ROW_DIMENSION = 1,
	IPU6_FW_PSYS_N_DATA_DIMENSION = 2
};

enum {
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_NOP = 0,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_SUBMIT,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_ATTACH,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_DETACH,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_START,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_DISOWN,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_RUN,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_STOP,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_SUSPEND,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_RESUME,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_ABORT,
	IPU6_FW_PSYS_PROCESS_GROUP_CMD_RESET,
	IPU6_FW_PSYS_N_PROCESS_GROUP_CMDS
};

enum {
	IPU6_FW_PSYS_PROCESS_GROUP_PROTOCOL_LEGACY = 0,
	IPU6_FW_PSYS_PROCESS_GROUP_PROTOCOL_PPG,
	IPU6_FW_PSYS_PROCESS_GROUP_N_PROTOCOLS
};

/* Excluding PMEM */
#define IPU6SE_FW_PSYS_N_DATA_MEM_TYPE_ID (IPU6SE_FW_PSYS_N_MEM_TYPE_ID - 1)
#define IPU6SE_FW_PSYS_N_DEV_DFM_ID			\
	(IPU6SE_FW_PSYS_DEV_DFM_ISL_EMPTY_PORT_ID + 1)
#define IPU6SE_FW_PSYS_VMEM0_MAX_SIZE				0x0800
#define IPU6SE_FW_PSYS_TRANSFER_VMEM0_MAX_SIZE			0x0800
#define IPU6SE_FW_PSYS_LB_VMEM_MAX_SIZE				0x0400
#define IPU6SE_FW_PSYS_DMEM0_MAX_SIZE				0x4000
#define IPU6SE_FW_PSYS_DMEM1_MAX_SIZE				0x1000

#define IPU6SE_FW_PSYS_DEV_CHN_DMA_EXT0_MAX_SIZE		22
#define IPU6SE_FW_PSYS_DEV_CHN_DMA_EXT1_READ_MAX_SIZE		22
#define IPU6SE_FW_PSYS_DEV_CHN_DMA_EXT1_WRITE_MAX_SIZE		22
#define IPU6SE_FW_PSYS_DEV_CHN_DMA_IPFD_MAX_SIZE		0
#define IPU6SE_FW_PSYS_DEV_CHN_DMA_ISA_MAX_SIZE			2

#define IPU6SE_FW_PSYS_DEV_DFM_ISL_FULL_PORT_ID_MAX_SIZE	32
#define IPU6SE_FW_PSYS_DEV_DFM_LB_FULL_PORT_ID_MAX_SIZE		32
#define IPU6SE_FW_PSYS_DEV_DFM_ISL_EMPTY_PORT_ID_MAX_SIZE	32
#define IPU6SE_FW_PSYS_DEV_DFM_LB_EMPTY_PORT_ID_MAX_SIZE	32
#define	IPU6SE_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_EXT_STRUCT	1

#define	IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_EXT_STRUCT	0
#define IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROGRAM_MANIFEST	0
#define	IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_STRUCT		0
#define	IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_GROUP_STRUCT	2
#define	IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROGRAM_MANIFEST_EXT	2
#define IPU6_FW_PSYS_N_PADDING_UINT8_IN_TERMINAL_STRUCT		5
#define IPU6_FW_PSYS_N_PADDING_UINT8_IN_PARAM_TERMINAL_STRUCT	6
#define	IPU6_FW_PSYS_N_PADDING_UINT8_IN_DATA_TERMINAL_STRUCT	3
#define	IPU6_FW_PSYS_N_PADDING_UINT8_IN_FRAME_DESC_STRUCT	3
#define IPU6_FW_PSYS_N_FRAME_PLANES				6
#define IPU6_FW_PSYS_N_PADDING_UINT8_IN_FRAME_STRUCT		4
#define IPU6_FW_PSYS_N_PADDING_UINT8_IN_BUFFER_SET_STRUCT	1

#define IPU6_FW_PSYS_MAX_INPUT_DEC_RESOURCES			4
#define IPU6_FW_PSYS_MAX_OUTPUT_DEC_RESOURCES			4
#define IPU6_FW_PSYS_PROCESS_MAX_CELLS				1
#define IPU6_FW_PSYS_KERNEL_BITMAP_NOF_ELEMS			4
#define IPU6_FW_PSYS_RBM_NOF_ELEMS				5
#define IPU6_FW_PSYS_KBM_NOF_ELEMS				4

/* Excluding PMEM */
#define IPU6_FW_PSYS_N_DATA_MEM_TYPE_ID	(IPU6_FW_PSYS_N_MEM_TYPE_ID - 1)
#define IPU6_FW_PSYS_N_DEV_DFM_ID			\
	(IPU6_FW_PSYS_DEV_DFM_LB_EMPTY_PORT_ID + 1)

#define IPU6_FW_PSYS_VMEM0_MAX_SIZE				0x0800
/* Transfer VMEM0 words, ref HAS Transfer*/
#define IPU6_FW_PSYS_TRANSFER_VMEM0_MAX_SIZE			0x0800
/* Transfer VMEM1 words, ref HAS Transfer*/
#define IPU6_FW_PSYS_TRANSFER_VMEM1_MAX_SIZE			0x0800
#define IPU6_FW_PSYS_LB_VMEM_MAX_SIZE				0x0400
#define IPU6_FW_PSYS_BAMEM0_MAX_SIZE				0x0800
#define IPU6_FW_PSYS_DMEM0_MAX_SIZE				0x4000
#define IPU6_FW_PSYS_DMEM1_MAX_SIZE				0x1000
#define IPU6_FW_PSYS_DMEM2_MAX_SIZE				0x1000
#define IPU6_FW_PSYS_DMEM3_MAX_SIZE				0x1000
#define IPU6_FW_PSYS_PMEM0_MAX_SIZE				0x0500

#define IPU6_FW_PSYS_DEV_CHN_DMA_EXT0_MAX_SIZE			30
#define IPU6_FW_PSYS_DEV_CHN_GDC_MAX_SIZE			0
#define IPU6_FW_PSYS_DEV_CHN_DMA_EXT1_READ_MAX_SIZE		30
#define IPU6_FW_PSYS_DEV_CHN_DMA_EXT1_WRITE_MAX_SIZE		43
#define IPU6_FW_PSYS_DEV_CHN_DMA_INTERNAL_MAX_SIZE		8
#define IPU6_FW_PSYS_DEV_CHN_DMA_IPFD_MAX_SIZE			0
#define IPU6_FW_PSYS_DEV_CHN_DMA_ISA_MAX_SIZE			2

#define IPU6_FW_PSYS_DEV_DFM_BB_FULL_PORT_ID_MAX_SIZE		32
#define IPU6_FW_PSYS_DEV_DFM_ISL_FULL_PORT_ID_MAX_SIZE		32
#define IPU6_FW_PSYS_DEV_DFM_LB_FULL_PORT_ID_MAX_SIZE		32
#define IPU6_FW_PSYS_DEV_DFM_BB_EMPTY_PORT_ID_MAX_SIZE		32
#define IPU6_FW_PSYS_DEV_DFM_ISL_EMPTY_PORT_ID_MAX_SIZE		32
#define IPU6_FW_PSYS_DEV_DFM_LB_EMPTY_PORT_ID_MAX_SIZE		32

struct ipu6_psys_fw_program_manifest_ext {
	u32 dfm_port_bitmap[IPU6_FW_PSYS_N_DEV_DFM_ID];
	u32 dfm_active_port_bitmap[IPU6_FW_PSYS_N_DEV_DFM_ID];
	u16 ext_mem_size[IPU6_FW_PSYS_N_DATA_MEM_TYPE_ID];
	u16 ext_mem_offset[IPU6_FW_PSYS_N_DATA_MEM_TYPE_ID];
	u16 dev_chn_size[IPU6_FW_PSYS_N_DEV_CHN_ID];
	u16 dev_chn_offset[IPU6_FW_PSYS_N_DEV_CHN_ID];
	u8 is_dfm_relocatable[IPU6_FW_PSYS_N_DEV_DFM_ID];
	u8 dec_resources_input[IPU6_FW_PSYS_MAX_INPUT_DEC_RESOURCES];
	u8 dec_resources_input_terminal[IPU6_FW_PSYS_MAX_INPUT_DEC_RESOURCES];
	u8 dec_resources_output[IPU6_FW_PSYS_MAX_OUTPUT_DEC_RESOURCES];
	u8 dec_resources_output_terminal[IPU6_FW_PSYS_MAX_OUTPUT_DEC_RESOURCES];
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROGRAM_MANIFEST_EXT];
};

struct ipu6_psys_fw_process_ext {
	u32 dfm_port_bitmap[IPU6_FW_PSYS_N_DEV_DFM_ID];
	u32 dfm_active_port_bitmap[IPU6_FW_PSYS_N_DEV_DFM_ID];
	u16 ext_mem_offset[IPU6_FW_PSYS_N_DATA_MEM_TYPE_ID];
	u16 dev_chn_offset[IPU6_FW_PSYS_N_DEV_CHN_ID];
	u8 ext_mem_id[IPU6_FW_PSYS_N_DATA_MEM_TYPE_ID];
};

struct ipu6_psys_fw_process {
	s16 parent_offset;
	u8 size;
	u8 cell_dependencies_offset;
	u8 terminal_dependencies_offset;
	u8 process_extension_offset;
	u8 ID;
	u8 program_idx;
	u8 state;
	u8 cells[IPU6_FW_PSYS_PROCESS_MAX_CELLS];
	u8 cell_dependency_count;
	u8 terminal_dependency_count;
};

struct ipu6_psys_fw_program_manifest {
	u32 kernel_bitmap[IPU6_FW_PSYS_KERNEL_BITMAP_NOF_ELEMS];
	s16 parent_offset;
	u8  program_dependency_offset;
	u8  terminal_dependency_offset;
	u8  size;
	u8  program_extension_offset;
	u8 program_type;
	u8 ID;
	u8 cells[IPU6_FW_PSYS_PROCESS_MAX_CELLS];
	u8 cell_type_id;
	u8 program_dependency_count;
	u8 terminal_dependency_count;
};

struct ipu6_psys_fw_process_group {
	u64 token;
	u64 private_token;
	u32 routing_bitmap[IPU6_FW_PSYS_RBM_NOF_ELEMS];
	u32 kernel_bitmap[IPU6_FW_PSYS_KBM_NOF_ELEMS];
	u32 size;
	u32 psys_server_init_cycles;
	u32 pg_load_start_ts;
	u32 pg_load_cycles;
	u32 pg_init_cycles;
	u32 pg_processing_cycles;
	u32 pg_next_frame_init_cycles;
	u32 pg_complete_cycles;
	u32 ID;
	u32 state;
	u32 ipu6_virtual_address;
	u32 resource_bitmap;
	u16 fragment_count;
	u16 fragment_state;
	u16 fragment_limit;
	u16 processes_offset;
	u16 terminals_offset;
	u8 process_count;
	u8 terminal_count;
	u8 subgraph_count;
	u8 protocol_version;
	u8 base_queue_id;
	u8 num_queues;
	u8 mask_irq;
	u8 error_handling_enable;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_GROUP_STRUCT];
} __packed;

struct ipu6_psys_fw_srv_init {
	void *host_ddr_pkg_dir;
	u32 ddr_pkg_dir_address;
	u32 pkg_dir_size;

	u32 icache_prefetch_sp;
	u32 icache_prefetch_isp;
};

struct ipu6_psys_fw_cmd {
	u16 command;
	u16 msg;
	u32 context_handle;
} __packed;

struct ipu6_psys_fw_event {
	u16 status;
	u16 command;
	u32 context_handle;
	u64 token;
} __packed;

struct ipu6_psys_fw_terminal {
	u32 terminal_type;
	s16 parent_offset;
	u16 size;
	u16 tm_index;
	u8 ID;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_TERMINAL_STRUCT];
};

struct ipu6_psys_fw_param_payload {
	u64 host_buffer;
	u32 buffer;
	u32 terminal_index;
};

struct ipu6_psys_fw_param_terminal {
	struct ipu6_psys_fw_terminal base;
	struct ipu6_psys_fw_param_payload param_payload;
	u16 param_section_desc_offset;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_PARAM_TERMINAL_STRUCT];
};

struct ipu6_psys_fw_frame {
	u32 buffer_state;
	u32 access_type;
	u32 pointer_state;
	u32 access_scope;
	u32 data;
	u32 data_index;
	u32 data_bytes;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_FRAME_STRUCT];
};

struct ipu6_psys_fw_frame_descriptor {
	u32 frame_format_type;
	u32 plane_count;
	u32 plane_offsets[IPU6_FW_PSYS_N_FRAME_PLANES];
	u32 stride[1];
	u32 ts_offsets[IPU6_FW_PSYS_N_FRAME_PLANES];
	u16 dimension[2];
	u16 size;
	u8 bpp;
	u8 bpe;
	u8 is_compressed;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_FRAME_DESC_STRUCT];
};

struct ipu6_psys_fw_stream {
	u64 dummy;
};

struct ipu6_psys_fw_data_terminal {
	struct ipu6_psys_fw_terminal base;
	struct ipu6_psys_fw_frame_descriptor frame_descriptor;
	struct ipu6_psys_fw_frame frame;
	struct ipu6_psys_fw_stream stream;
	u32 reserved;
	u32 connection_type;
	u16 fragment_descriptor_offset;
	u8 kernel_id;
	u8 subgraph_id;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_DATA_TERMINAL_STRUCT];
};

struct ipu6_psys_fw_buffer_set {
	u64 token;
	u32 kernel_enable_bitmap[IPU6_FW_PSYS_KERNEL_BITMAP_NOF_ELEMS];
	u32 terminal_enable_bitmap[IPU6_FW_PSYS_KERNEL_BITMAP_NOF_ELEMS];
	u32 routing_enable_bitmap[IPU6_FW_PSYS_KERNEL_BITMAP_NOF_ELEMS];
	u32 rbm[IPU6_FW_PSYS_RBM_NOF_ELEMS];
	u32 ipu6_virtual_address;
	u32 process_group_handle;
	u16 terminal_count;
	u8 frame_counter;
	u8 padding[IPU6_FW_PSYS_N_PADDING_UINT8_IN_BUFFER_SET_STRUCT];
};

struct ipu6_psys_fw_pgm {
	u32 kernel_bitmap[IPU6_FW_PSYS_KERNEL_BITMAP_NOF_ELEMS];
	u32 ID;
	u16 program_manifest_offset;
	u16 terminal_manifest_offset;
	u16 private_data_offset;
	u16 rbm_manifest_offset;
	u16 size;
	u8 alignment;
	u8 kernel_count;
	u8 program_count;
	u8 terminal_count;
	u8 subgraph_count;
	u8 reserved[5];
};

struct ipu6_fw_generic_program_manifest {
	u16 *dev_chn_size;
	u16 *dev_chn_offset;
	u16 *ext_mem_size;
	u16 *ext_mem_offset;
	u8 cell_id;
	u8 cells[IPU6_FW_PSYS_PROCESS_MAX_CELLS];
	u8 cell_type_id;
	u8 *is_dfm_relocatable;
	u32 *dfm_port_bitmap;
	u32 *dfm_active_port_bitmap;
};

struct ipu6_fw_resource_defs {
	u32 num_cells;
	u32 num_cells_type;
	const u8 *cells;
	u32 num_dev_channels;
	const u16 *dev_channels;

	u32 num_ext_mem_types;
	u32 num_ext_mem_ids;
	const u16 *ext_mem_ids;

	u32 num_dfm_ids;
	const u16 *dfms;

	u32 cell_mem_row;
	const u8 *cell_mem;
};

struct ipu6_psys_kcmd;
struct ipu6_psys;
void ipu6_psys_fw_pg_start(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_pg_disown(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_pg_abort(struct ipu6_psys_kcmd *kcmd);
void ipu6_psys_fw_pg_submit(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_rcv_event(struct ipu6_psys *psys,
			   struct ipu6_psys_fw_event *event);
int ipu6_psys_fw_terminal_set(struct ipu6_psys_fw_terminal *terminal,
			      int terminal_idx,
			      struct ipu6_psys_kcmd *kcmd,
			      u32 buffer, unsigned int size);
int ipu6_psys_fw_pg_get_id(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_pg_get_terminal_count(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_pg_get_size(struct ipu6_psys_kcmd *kcmd);
void ipu6_psys_fw_pg_set_ipu6_vaddress(struct ipu6_psys_kcmd *kcmd,
				       dma_addr_t vaddress);
struct ipu6_psys_fw_terminal *
ipu6_psys_fw_pg_get_terminal(struct ipu6_psys_kcmd *kcmd, int index);
void ipu6_psys_fw_pg_set_token(struct ipu6_psys_kcmd *kcmd, u64 token);
u64 ipu6_psys_fw_pg_get_token(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_ppg_set_buffer_set(struct ipu6_psys_kcmd *kcmd,
				    struct ipu6_psys_fw_terminal *terminal,
				    int terminal_idx, u32 buffer);
size_t ipu6_psys_fw_ppg_get_buffer_set_size(struct ipu6_psys_kcmd *kcmd);
void
ipu6_psys_fw_ppg_buffer_set_vaddress(struct ipu6_psys_fw_buffer_set *buf_set,
				     u32 vaddress);
int ipu6_psys_fw_ppg_buffer_set_set_keb(struct ipu6_psys_fw_buffer_set *buf_set,
					u32 *kernel_enable_bitmap);
struct ipu6_psys_fw_buffer_set *
ipu6_psys_fw_ppg_create_buffer_set(struct ipu6_psys_kcmd *kcmd,
				   void *kaddr, u32 frame_counter);
int ipu6_psys_fw_ppg_enqueue_bufs(struct ipu6_psys_kcmd *kcmd);
u8 ipu6_psys_fw_ppg_get_base_queue_id(struct ipu6_psys_kcmd *kcmd);
void ipu6_psys_fw_ppg_set_base_queue_id(struct ipu6_psys_kcmd *kcmd,
					u8 queue_id);
int ipu6_psys_fw_pg_get_protocol(struct ipu6_psys_kcmd *kcmd);
int ipu6_psys_fw_open(struct ipu6_psys *psys);
int ipu6_psys_fw_close(struct ipu6_psys *psys);
int ipu6_psys_fw_get_cmd_queue(struct ipu6_psys *psys);
void ipu6_psys_fw_put_cmd_queue(struct ipu6_psys *psys, u8 queue_id);
#endif /* IPU6_PSYS_FW_H */
