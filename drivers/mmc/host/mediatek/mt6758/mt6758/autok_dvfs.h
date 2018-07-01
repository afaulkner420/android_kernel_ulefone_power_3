/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _AUTOK_DVFS_H_
#define _AUTOK_DVFS_H_
#include "msdc_cust.h"
#include "autok.h"

#if !defined(FPGA_PLATFORM) \
	&& !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS) \
		&& !defined(CONFIG_MTK_MSDC_MIGRATION_BYPASS)
#include <mtk_vcorefs_manager.h>
#include <mtk_spm_vcore_dvfs.h>

enum AUTOK_VCORE {
	AUTOK_VCORE_LEVEL0 = 0,
	AUTOK_VCORE_LEVEL1,
	AUTOK_VCORE_MERGE,
	AUTOK_VCORE_NUM,
};

#else
enum dvfs_opp {
	OPP_UNREQ = -1,
	OPP_0 = 0,
	OPP_1,
	OPP_2,
	OPP_3,
	NUM_OPP,
};

#define is_vcorefs_can_work() -1
#define vcorefs_request_dvfs_opp(a, b)	0
#define vcorefs_get_hw_opp() OPP_0
#define spm_msdc_dvfs_setting(a, b)

#define AUTOK_VCORE_LEVEL0	0
#define AUTOK_VCORE_LEVEL1	0
#define AUTOK_VCORE_MERGE	1
#define AUTOK_VCORE_NUM		2
#endif

#define SDIO_DVFS_TIMEOUT       (HZ/100 * 5)    /* 10ms x5 */
/* Enable later@Peter */
/* #define SDIO_HW_DVFS_CONDITIONAL */

/**********************************************************
 * Function Declaration                                    *
 *********************************************************
 */
extern int sdio_autok_res_exist(struct msdc_host *host);
extern int sdio_autok_res_apply(struct msdc_host *host, int vcore);
extern int sdio_autok_res_save(struct msdc_host *host, int vcore, u8 *res);
extern void sdio_autok_wait_dvfs_ready(void);
extern int emmc_execute_dvfs_autok(struct msdc_host *host, u32 opcode);
extern int sd_execute_dvfs_autok(struct msdc_host *host, u32 opcode);
extern void sdio_execute_dvfs_autok(struct msdc_host *host);

extern int autok_res_check(u8 *res_h, u8 *res_l);
extern void sdio_set_hw_dvfs(int vcore, int done, struct msdc_host *host);
extern void sdio_dvfs_reg_restore(struct msdc_host *host);
extern void msdc_dump_autok(struct msdc_host *host);

#endif /* _AUTOK_DVFS_H_ */

