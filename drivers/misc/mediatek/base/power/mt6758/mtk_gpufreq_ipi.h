/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#define IPI_GPU_DVFS_MBOX_DATA_SIZE 2

enum {
	IPI_GPU_DVFS_INIT_OPPTAB,
	IPI_GPU_DVFS_INIT,
	IPI_GPU_DVFS_OPPIDX_INFO,
	IPI_GPU_DVFS_STATUS_OP,
	IPI_GPU_DVFS_TARGET_FREQ_IDX,
	IPI_GPU_DVFS_SET_FREQ,
	IPI_GPU_DVFS_SET_FV,
	IPI_GPU_DVFS_AP_LIMITED_FACTOR,
	IPI_GPU_DVFS_TEST,
	IPI_GPU_DVFS_POW_STATUS_UPDATE,
	IPI_GPU_DVFS_LOW_BATT_VOLT_CB,
	IPI_GPU_DVFS_OC_CB,
	IPI_GPU_DVFS_SET_PMIC_LOW_POWER_MODE,
	NR_GPU_DVFS_IPI,
};

enum {
	QUERY_FREQ,
	QUERY_VOLT,
	QUERY_VGPU_VSRAM,
	QUERY_POWER,
};

enum {
	LIMIED_FACTOR_PBM,
	LIMIED_FACTOR_THERMAL,
};

enum {
	QUERY_CUR_OPPIDX,
	QUERY_CUR_LIMITED_IDX,
	QUERY_VOLT_ENABLED,
	QUERY_LMITED_FACTOR,
	QUERY_CUR_FREQ,
	QUERY_CUR_VOLT,
	SET_VOLT_SWITCH,
	SET_IGNORE_LMITED_FACTOR,
	SET_UPBOUND_FREQ,
	SET_THROTTLE_POWER,
	SET_GPU_LOADING,
	SET_VGPU,
	SET_PLL,
};

enum {
	SUB_CMD_MOD_FREQ,
	SUB_CMD_FIX_FREQ,
	SUB_CMD_REL_FREQ,
};


enum {
	FIX_NO,
	FIX_NORMAL,
	FIX_PTPOD,
	FIX_MAX,
};

#ifdef MTK_SSPM
typedef struct gdvfs_data {
	unsigned int cmd;
	union {
		struct {
			unsigned int arg[IPI_GPU_DVFS_MBOX_DATA_SIZE];
		} set_fv;
	} u;
} gdvfs_data_t;
#endif

/*
 *  SSPM -> AP definition
 */
enum {
	IPI_GPU_DVFS_SSPM_KICK_PBM,
	IPI_GPU_DVFS_CUR_FREQ_UPDATE,
	NR_GPU_DVFS_IPI_SSPM,
};

#define oSPEED_DEV_ID_GPU_DVFS IPI_ID_GPU_DVFS
