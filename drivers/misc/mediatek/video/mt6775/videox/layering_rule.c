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

#include "layering_rule.h"

static struct layering_rule_ops l_rule_ops;
static struct layering_rule_info_t l_rule_info;

/**
 * The layer mapping table define ovl layer dispatch rule for both
 * primary and secondary display.Each table has 16 elements which
 * represent the layer mapping rule by the number of input layers.
 */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT

int emi_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{250, 450, 750},
	/* HRT_BOUND_TYPE_LP4_FHD_PLUS */
	{150, 350, 550},
};

int larb_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{1200, 1200, 1200},
	/* HRT_BOUND_TYPE_LP4_FHD_PLUS */
	{1200, 1200, 1200},
};

static int layer_mapping_table[HRT_TB_NUM][TOTAL_OVL_LAYER_NUM] = {
	/* HRT_TB_TYPE_GENERAL */
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003001F,
	 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
	/* HRT_PATH_2_OVL_ONLY */
	{0x00010001, 0x00030011, 0x00030013, 0x00030017, 0x0003001F, 0x0003001F,
	 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
	 /* HRT_PATH_2_RPO */
	{0x00010001, 0x00030005, 0x0003000D, 0x0003001D, 0x0003001D, 0x0003001D,
	 0x0003001D, 0x0003001D, 0x0003001D, 0x0003001D},
	/* HRT_TB_TYPE_GENERAL */
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003001F,
	 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
};

#else

int emi_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{300, 500, 800},
	/* HRT_BOUND_TYPE_LP4_FHD_PLUS */
	{200, 400, 600},
};

int larb_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	/* HRT_BOUND_TYPE_LP4 */
	{1200, 1200, 1200},
	/* HRT_BOUND_TYPE_LP4_FHD_PLUS */
	{1200, 1200, 1200},
};


static int layer_mapping_table[HRT_TB_NUM][TOTAL_OVL_LAYER_NUM] = {
	/* HRT_TB_TYPE_GENERAL */
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003003F,
	0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
	/* HRT_PATH_2_OVL_ONLY */
	{0x00010001, 0x00030011, 0x00030031, 0x00030033, 0x00030037, 0x0003003F,
	0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
	/* HRT_PATH_2_OVL_RPO */
	{0x00010001, 0x00030005, 0x0003000D, 0x0003001D, 0x0003003D, 0x0003003D,
	0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D},
	/* HRT_PATH_OVL2_TO_OVL4 */
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003003F,
	0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
};
#endif
/**
 * The larb mapping table represent the relation between LARB and OVL.
 */
static int larb_mapping_table[HRT_TB_NUM] = {
	0x00010010, 0x00010010, 0x00010001, 0x00010001,
};

/**
 * The OVL mapping table is used to get the OVL index of correcponding layer.
 * The bit value 1 means the position of the last layer in OVL engine.
 */
static int ovl_mapping_table[HRT_TB_NUM] = {
	0x00020028, 0x00020028, 0x00020022, 0x00020022,
};

#define GET_SYS_STATE(sys_state) ((l_rule_info.hrt_sys_state >> sys_state) & 0x1)

static void layering_rule_senario_decision(struct disp_layer_info *disp_info)
{
	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_START, l_rule_info.disp_path,
		l_rule_info.layer_tb_idx | (l_rule_info.bound_tb_idx << 16));

	if (l_rule_info.disp_path == HRT_PATH_UNKNOWN) {
		l_rule_info.disp_path = HRT_PATH_GENERAL;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_GENERAL;
	} else if (l_rule_info.disp_path == HRT_PATH_2_OVL_ONLY) {
		l_rule_info.disp_path = HRT_PATH_2_OVL_ONLY;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_2_OVL_ONLY;
	} else if (l_rule_info.disp_path == HRT_PATH_2_OVL_RPO) {
		l_rule_info.disp_path = HRT_PATH_2_OVL_RPO;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_2_OVL_RPO;
	} else if (l_rule_info.disp_path == HRT_PATH_OVL2_TO_OVL4) {
		l_rule_info.disp_path = HRT_PATH_OVL2_TO_OVL4;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_OVL2_TO_OVL4;
	}

	l_rule_info.primary_fps = 60;

	if (primary_display_get_height() > 2160)
		l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP4_FHD_PLUS;
	else
		l_rule_info.bound_tb_idx = HRT_BOUND_TYPE_LP4;

	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_END, l_rule_info.disp_path,
		l_rule_info.layer_tb_idx | (l_rule_info.bound_tb_idx << 16));
}

static void _filter_one_yuv_ovl_only_layer(struct disp_layer_info *disp_info, int ovl_only_layer_cnt)
{
	unsigned int i, disp_idx = 0;
	bool is_yuv_occupied = false;
	struct layer_config *p_layer_info;

	/* ovl only support 1 yuv layer */
	for (disp_idx = 0 ; disp_idx < 2 ; disp_idx++) {
		for (i = 0; i < disp_info->layer_num[disp_idx]; i++) {
			p_layer_info = &(disp_info->input_config[disp_idx][i]);
			if (is_gles_layer(disp_info, disp_idx, i))
				continue;
			if (!is_yuv(p_layer_info->src_fmt))
				continue;

			if (is_yuv_occupied ||
				(!has_layer_cap(p_layer_info, LAYERING_OVL_ONLY) && ovl_only_layer_cnt)) {
				/* push to GPU */
				if (disp_info->gles_head[disp_idx] == -1 || i < disp_info->gles_head[disp_idx])
					disp_info->gles_head[disp_idx] = i;
				if (disp_info->gles_tail[disp_idx] == -1 || i > disp_info->gles_tail[disp_idx])
					disp_info->gles_tail[disp_idx] = i;
			} else {
				is_yuv_occupied = true;
			}
		}
	}
}

static void _filter_2_ovl_only_layer(struct disp_layer_info *disp_info, int max_ovl_only_layer_idx)
{
	if (max_ovl_only_layer_idx != disp_info->layer_num[HRT_PRIMARY]) {
		if (disp_info->gles_head[HRT_PRIMARY] == -1 ||
			max_ovl_only_layer_idx + 1 < disp_info->gles_head[HRT_PRIMARY])
			disp_info->gles_head[HRT_PRIMARY] = max_ovl_only_layer_idx + 1;
		disp_info->gles_tail[HRT_PRIMARY] = disp_info->layer_num[HRT_PRIMARY] - 1;
	}
	l_rule_info.disp_path = HRT_PATH_2_OVL_ONLY;
}

static bool filter_by_hw_limitation(struct disp_layer_info *disp_info)
{
	unsigned int disp_idx = 0, i = 0, ovl_only_layer_cnt = 0;
	unsigned int max_ovl_only_layer_idx = 0;
	bool is_yuv_occupied = false;
	struct layer_config *p_layer_info;

	if (disp_helper_get_option(DISP_OPT_RPO))
		return false;

	for (i = 0; i < disp_info->layer_num[HRT_PRIMARY]; i++) {
		p_layer_info = &(disp_info->input_config[disp_idx][i]);
		if (has_layer_cap(p_layer_info, LAYERING_OVL_ONLY)) {
			ovl_only_layer_cnt++;
			max_ovl_only_layer_idx = i;
		}
	}

	if (ovl_only_layer_cnt < 2)
		_filter_one_yuv_ovl_only_layer(disp_info, ovl_only_layer_cnt);
	else if (ovl_only_layer_cnt == 2)
		_filter_2_ovl_only_layer(disp_info, max_ovl_only_layer_idx);
	else
		DISPERR("Not support the number of OVL_ONLY layer over 2, cnt:%d\n",
			ovl_only_layer_cnt);

	return is_yuv_occupied;
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
/* --------------rpo start-------------*/
static bool is_RPO(struct disp_layer_info *disp_info)
{
	struct layer_config *c = NULL;

	if (!disp_rsz_sram_get_sta() || !disp_info->layer_num[0])
		return false;

	c = &disp_info->input_config[HRT_PRIMARY][0];

	if (is_gles_layer(disp_info, HRT_PRIMARY, 0))
		return false;

	if ((c->src_height == 0) || (c->src_width == 0))
		return false;

	if (c->src_width == c->dst_width && c->src_height == c->dst_height)
		return false;

	if (c->src_width > c->dst_width || c->src_height > c->dst_height)
		return false;

	if (unlikely((c->src_height < 5) || (c->src_width < 5) ||
		((c->src_height << 7) < c->dst_height) ||
		((c->src_width << 7) < c->dst_width))) {
		DISPERR("input rsz size not support!\n");
		return false;
	}
	/* set rsz flag */
	c->layer_caps |= DISP_RSZ_LAYER;

	return true;
}

static bool gles_layer_adjustment_resize(struct disp_layer_info *disp_info)
{
	int gpu_rsz_idx = 0;
	bool ret = false;

	l_rule_info.scale_rate = -1;
	l_rule_info.disp_path = HRT_PATH_OVL2_TO_OVL4;

	if (is_ext_path(disp_info))
		rollback_all_resize_layer_to_GPU(disp_info, HRT_SECONDARY);

	if (is_RPO(disp_info)) {
		l_rule_info.scale_rate = 0;
		l_rule_info.disp_path = HRT_PATH_2_OVL_RPO;
		gpu_rsz_idx = 1;
		ret = true;
	}

	rollback_resize_layer_to_GPU_range(disp_info, HRT_PRIMARY,
		gpu_rsz_idx, disp_info->layer_num[HRT_PRIMARY] - 1);

#ifdef HRT_DEBUG_LEVEL1
	DISPMSG("[%s] scale_id =%d\n", __func__, l_rule_info.scale_rate);
#endif

	return ret;
}
static void lr_gpu_change_rsz_info(void)
{
}

static bool lr_unset_disp_rsz_attr(struct disp_layer_info *disp_info,
		     int disp_idx)
{
	struct layer_config *lc = &disp_info->input_config[disp_idx][0];

	if (l_rule_info.disp_path == HRT_PATH_2_OVL_RPO &&
	    has_layer_cap(lc, MDP_RSZ_LAYER) &&
	    has_layer_cap(lc, DISP_RSZ_LAYER)) {
		lc->layer_caps &= ~DISP_RSZ_LAYER;
		l_rule_info.disp_path = HRT_PATH_OVL2_TO_OVL4;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_OVL2_TO_OVL4;
		return true;
	}
	return false;
}


/*--------------rpo end--------------*/
void layering_rule_init(void)
{
	l_rule_info.primary_fps = 60;
	register_layering_rule_ops(&l_rule_ops, &l_rule_info);
}

static struct layering_rule_ops l_rule_ops = {
	.scenario_decision = layering_rule_senario_decision,
	.resizing_rule = gles_layer_adjustment_resize,
	.get_bound_table = get_bound_table,
	.get_hrt_bound = get_hrt_bound,
	.get_mapping_table = get_mapping_table,
	.rollback_to_gpu_by_hw_limitation = filter_by_hw_limitation,
	.rsz_by_gpu_info_change = lr_gpu_change_rsz_info,
	.unset_disp_rsz_attr = lr_unset_disp_rsz_attr,
};

