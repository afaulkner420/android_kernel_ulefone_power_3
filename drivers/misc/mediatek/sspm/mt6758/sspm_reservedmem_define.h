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
#ifndef _SSPM_RESERVEDMEM_DEFINE_H_
#define _SSPM_RESERVEDMEM_DEFINE_H_
#include <sspm_reservedmem.h>

enum {
	SSPM_MEM_ID = 0,
	PTPOD_MEM_ID,
	GPU_DVFS_MEM_ID,
	UPD_MEM_ID,
	/* debug only */
	/* MET_MEM_ID, */
	NUMS_MEM_ID,
};

#ifdef _SSPM_INTERNAL_
/* The total size of sspm_reserve_mblock should less equal than reserve-memory-sspm_share of device tree */
static struct sspm_reserve_mblock sspm_reserve_mblock[NUMS_MEM_ID] = {
	{
		.num = SSPM_MEM_ID,
		.size = 0x100400,
	},
	{
		.num = PTPOD_MEM_ID,
		.size = 0x3000,
	},
	{
		.num = GPU_DVFS_MEM_ID,
		.size = 0xA000,
	},
	{
		.num = UPD_MEM_ID,
		.size = 0x1800,
	},
	/*{ */
	/*	.num = MET_MEM_ID, */
	/*	.size = 0x400000, */
	/*}, */
};
#endif
#endif
