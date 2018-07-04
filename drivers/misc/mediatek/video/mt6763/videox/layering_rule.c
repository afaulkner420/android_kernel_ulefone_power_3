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

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>

#include "mtk_dramc.h"
#include "layering_rule.h"

static struct layering_rule_ops l_rule_ops;
static struct layering_rule_info_t l_rule_info;

#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
int emi_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{16, 16, 16, 24},
	/* HRT_BOUND_TYPE_LP3 */
	{14, 14, 14, 14},
	/* HRT_BOUND_TYPE_LP4_1CH */
	{14, 14, 14, 14},
	/* HRT_BOUND_TYPE_LP4_HYBRID */
	{16, 16, 16, 24},
	/* HRT_BOUND_TYPE_LP3_HD */
	{30, 30, 30, 30},
	/* HRT_BOUND_TYPE_LP4_HD */
	{36, 36, 36, 54},
};
#else
int emi_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{14, 14, 14, 22},
	/* HRT_BOUND_TYPE_LP3 */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP4_1CH */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP4_HYBRID */
	{14, 14, 14, 22},
	/* HRT_BOUND_TYPE_LP3_HD */
	{28, 28, 28, 28},
	/* HRT_BOUND_TYPE_LP4_HD */
	{34, 34, 34, 52},
};
#endif


int larb_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP3 */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP4_1CH */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP4_HYBRID */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP3_HD */
	{12, 12, 12, 12},
	/* HRT_BOUND_TYPE_LP4_HD */
	{12, 12, 12, 12},
};

/**
 * The layer mapping table define ovl layer dispatch rule for both
 * primary and secondary display.Each table has 16 elements which
 * represent the layer mapping rule by the number of input layers.
 */
#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
static int layer_mapping_table[HRT_TB_NUM][TOTAL_OVL_LAYER_NUM] = {
	/* HRT_TB_TYPE_GENERAL */
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003003F,
	0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
};

#else
static int layer_mapping_table[HRT_TB_NUM][TOTAL_OVL_LAYER_NUM] = {
	/* HRT_TB_TYPE_GENERAL */
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003001F,
	0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
};
#endif
/**
 * The larb mapping table represent the relation between LARB and OVL.
 */
static int larb_mapping_table[HRT_TB_NUM] = {
	0x00010010,
};

/**
 * The OVL mapping table is used to get the OVL index of correcponding layer.
 * The bit value 1 means the position of the last layer in OVL engine.
 */
#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
static int ovl_mapping_table[HRT_TB_NUM] = {
	0x00020028,
};
#else
static int ovl_mapping_table[HRT_TB_NUM] = {
	0x00020018,
};
#endif
#define GET_SYS_STATE(sys_state) ((l_rule_info.hrt_sys_state >> sys_state) & 0x1)

static void layering_rule_senario_decision(struct disp_layer_info *disp_info)
{
	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_START, l_rule_info.disp_path,
		l_rule_info.layer_tb_idx | (l_rule_info.bound_tb_idx << 16));

	if (GET_SYS_STATE(DISP_HRT_MULTI_TUI_ON)) {
		l_rule_info.disp_path = HRT_PATH_GENERAL;
		/* layer_tb_idx = HRT_TB_TYPE_MULTI_WINDOW_TUI;*/
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_GENERAL;
	} else {
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_GENERAL;
		l_rule_info.disp_path = HRT_PATH_GENERAL;
	}

	l_rule_info.primary_fps = 60;
	if (get_ddr_type() == TYPE_LPDDR3) {
		if (primary_display_get_width() < 800) {
			/* HD or HD+ */
			l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP3_HD;
		} else {
			l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP3;
		}

	} else {
		/* LPDDR4, LPDDR4X */
		if (primary_display_get_width() < 800) {
			if (get_emi_ch_num() == 2)
				l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP4_HD;
			else
				l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP3_HD;
		} else {
			if (get_emi_ch_num() == 2)
				l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP4;
			else
				l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP4_1CH;
		}
	}

	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_END, l_rule_info.disp_path,
		l_rule_info.layer_tb_idx | (l_rule_info.bound_tb_idx << 16));
}

static bool filter_by_hw_limitation(struct disp_layer_info *disp_info)
{
	bool flag = false;
	unsigned int i = 0;
	struct layer_config *info;
	unsigned int disp_idx = 0;
#if 0
	/* ovl only support 1 yuv layer */
	for (disp_idx = 0 ; disp_idx < 2 ; disp_idx++) {
		for (i = 0; i < disp_info->layer_num[disp_idx]; i++) {
			info = &(disp_info->input_config[disp_idx][i]);
			if (is_gles_layer(disp_info, disp_idx, i))
				continue;
			if (is_yuv(info->src_fmt)) {
				if (flag) {
					/* push to GPU */
					if (disp_info->gles_head[disp_idx] == -1 || i < disp_info->gles_head[disp_idx])
						disp_info->gles_head[disp_idx] = i;
					if (disp_info->gles_tail[disp_idx] == -1 || i > disp_info->gles_tail[disp_idx])
						disp_info->gles_tail[disp_idx] = i;
				} else {
					flag = true;
				}
			}
		}
	}
#else
	unsigned int layer_cnt = 0;

	disp_idx = 1;
	for (i = 0; i < disp_info->layer_num[disp_idx]; i++) {
		info = &(disp_info->input_config[disp_idx][i]);
		if (is_gles_layer(disp_info, disp_idx, i))
			continue;

		layer_cnt++;
		if (layer_cnt > SECONDARY_OVL_LAYER_NUM) {
			/* push to GPU */
			if (disp_info->gles_head[disp_idx] == -1 || i < disp_info->gles_head[disp_idx])
				disp_info->gles_head[disp_idx] = i;
			if (disp_info->gles_tail[disp_idx] == -1 || i > disp_info->gles_tail[disp_idx])
				disp_info->gles_tail[disp_idx] = i;

			flag = true;
		}
	}
#endif
	return flag;
}


static int get_hrt_bound(int is_larb, int hrt_level)
{
	if (is_larb)
		return larb_bound_table[l_rule_info.bound_tb_idx][hrt_level];
	else
		return emi_bound_table[l_rule_info.bound_tb_idx][hrt_level];
}

static int *get_bound_table(enum DISP_HW_MAPPING_TB_TYPE tb_type)
{
	switch (tb_type) {
	case DISP_HW_EMI_BOUND_TB:
		return emi_bound_table[l_rule_info.bound_tb_idx];
	case DISP_HW_LARB_BOUND_TB:
		return larb_bound_table[l_rule_info.bound_tb_idx];
	default:
		return NULL;
	}
}

static int get_mapping_table(enum DISP_HW_MAPPING_TB_TYPE tb_type, int param)
{
	switch (tb_type) {
	case DISP_HW_OVL_TB:
		return ovl_mapping_table[l_rule_info.layer_tb_idx];
	case DISP_HW_LARB_TB:
		return larb_mapping_table[l_rule_info.layer_tb_idx];
	case DISP_HW_LAYER_TB:
		if (param < MAX_PHY_OVL_CNT && param >= 0)
			return layer_mapping_table[l_rule_info.layer_tb_idx][param];
		else
			return -1;
	default:
		return -1;
	}
}

void layering_rule_init(void)
{
	l_rule_info.primary_fps = 60;
	register_layering_rule_ops(&l_rule_ops, &l_rule_info);
}

static struct layering_rule_ops l_rule_ops = {
	.scenario_decision = layering_rule_senario_decision,
	.get_bound_table = get_bound_table,
	.get_hrt_bound = get_hrt_bound,
	.get_mapping_table = get_mapping_table,
	.rollback_to_gpu_by_hw_limitation = filter_by_hw_limitation,
};

