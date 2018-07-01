/*
 * Copyright (C) 2017 MediaTek Inc.
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

/**
 * @file	mtk_gpufreq_core
 * @brief   Driver for GPU-DVFS
 */

/**
 * ===============================================
 * SECTION : Include files
 * ===============================================
 */

#include "mtk_gpufreq_core.h"

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mtk_vcorefs_manager.h>
#ifdef MT_GPUFREQ_OPP_STRESS_TEST
#include <linux/random.h>
#endif				/* ifdef MT_GPUFREQ_OPP_STRESS_TEST */

#include "mt-plat/upmu_common.h"
#include "mt-plat/sync_write.h"
#include "mt-plat/mtk_pmic_wrap.h"
#include "mach/mtk_fhreg.h"
#include "mach/mtk_freqhopping.h"
#include "mtk_freqhopping_drv.h"
#include "mach/mtk_thermal.h"
#include "mach/upmu_sw.h"
#include "mach/upmu_hw.h"
#include "mach/mtk_pbm.h"
#include "mt6775_clkmgr.h"
#include "mtk_dramc.h"
#ifdef MT_GPUFREQ_STATIC_PWR_READY2USE
#include "mtk_common_static_power.h"
#endif				/* ifdef MT_GPUFREQ_STATIC_PWR_READY2USE */
#include "mtk_gpufreq.h"
#include "mtk_devinfo.h"

/**
 * ===============================================
 * SECTION : Local functions declaration
 * ===============================================
 */
static int __mt_gpufreq_pdrv_probe(struct platform_device *pdev);
static void __mt_gpufreq_set(unsigned int freq_old, unsigned int freq_new,
			     unsigned int volt_old, unsigned int volt_new);
static void __mt_gpufreq_set_fixed_volt(int fixed_volt);
static void __mt_gpufreq_set_fixed_freq(int fixed_freq);
static void __mt_gpufreq_buck_set_PWM_mode(unsigned int mode);
static unsigned int __mt_gpufreq_get_cur_gpu_volt(void);
static unsigned int __mt_gpufreq_get_cur_sram_core_volt(void);
static unsigned int __mt_gpufreq_get_cur_freq(void);
/* static int __mt_gpufreq_get_opp_idx_by_volt(unsigned int volt); */
static unsigned int __mt_gpufreq_get_limited_freq_by_power(unsigned int limited_power);
static enum g_post_div_order_enum __mt_gpufreq_get_post_div_order(unsigned int freq,
								  unsigned int efuse);
static void __mt_gpufreq_kick_pbm(int enable);
static void __mt_gpufreq_clock_switch(unsigned int freq_new);
static void __mt_gpufreq_volt_switch(unsigned int volt_old, unsigned int volt_new);
static void __mt_gpufreq_batt_oc_protect(unsigned int limited_idx);
static void __mt_gpufreq_batt_percent_protect(unsigned int limited_index);
static void __mt_gpufreq_low_batt_protect(unsigned int limited_index);
static void __mt_update_gpufreqs_power_table(void);
static void __mt_gpufreq_update_max_limited_idx(void);
static unsigned int __mt_gpufreq_calculate_dds(unsigned int freq_khz,
					       enum g_post_div_order_enum post_div_order);
static void __mt_gpufreq_setup_opp_power_table(int num);


/**
 * ===============================================
 * SECTION : Local variables definition
 * ===============================================
 */

static struct mt_gpufreq_power_table_info *g_power_table;
static struct g_opp_table_info *g_opp_table;
static struct g_opp_table_info *g_opp_table_default;
static struct g_pmic_info *g_pmic;
static struct g_clk_info *g_clk;
#ifdef GPUFREQ_SUPPORT_CT
/* CT0 opp table */
static struct g_opp_table_info g_opp_table_e1_0[] = {
	GPUOP(GPU_DVFS_FREQ0, GPU_DVFS_VOLT0_CT, 0),
	GPUOP(GPU_DVFS_FREQ1, GPU_DVFS_VOLT1_CT0, 1),
	GPUOP(GPU_DVFS_FREQ2, GPU_DVFS_VOLT2_CT0, 2),
	GPUOP(GPU_DVFS_FREQ3, GPU_DVFS_VOLT3_CT0, 3),
	GPUOP(GPU_DVFS_FREQ4, GPU_DVFS_VOLT4_CT0, 4),
	GPUOP(GPU_DVFS_FREQ5, GPU_DVFS_VOLT5_CT0, 5),
	GPUOP(GPU_DVFS_FREQ6, GPU_DVFS_VOLT6_CT0, 6),
	GPUOP(GPU_DVFS_FREQ7, GPU_DVFS_VOLT7_CT0, 7),
	GPUOP(GPU_DVFS_FREQ8, GPU_DVFS_VOLT8_CT0, 8),
	GPUOP(GPU_DVFS_FREQ9, GPU_DVFS_VOLT9_CT0, 9),
	GPUOP(GPU_DVFS_FREQ10, GPU_DVFS_VOLT10_CT0, 10),
	GPUOP(GPU_DVFS_FREQ11, GPU_DVFS_VOLT11_CT0, 11),
	GPUOP(GPU_DVFS_FREQ12, GPU_DVFS_VOLT12_CT0, 12),
	GPUOP(GPU_DVFS_FREQ13, GPU_DVFS_VOLT13_CT0, 13),
	GPUOP(GPU_DVFS_FREQ14, GPU_DVFS_VOLT14_CT0, 14),
	GPUOP(GPU_DVFS_FREQ15, GPU_DVFS_VOLT15_CT0, 15),
};
/* CT1 opp table */
static struct g_opp_table_info g_opp_table_e1_1[] = {
	GPUOP(GPU_DVFS_FREQ0, GPU_DVFS_VOLT0_CT, 0),
	GPUOP(GPU_DVFS_FREQ1, GPU_DVFS_VOLT1_CT1, 1),
	GPUOP(GPU_DVFS_FREQ2, GPU_DVFS_VOLT2_CT1, 2),
	GPUOP(GPU_DVFS_FREQ3, GPU_DVFS_VOLT3_CT1, 3),
	GPUOP(GPU_DVFS_FREQ4, GPU_DVFS_VOLT4_CT1, 4),
	GPUOP(GPU_DVFS_FREQ5, GPU_DVFS_VOLT5_CT1, 5),
	GPUOP(GPU_DVFS_FREQ6, GPU_DVFS_VOLT6_CT1, 6),
	GPUOP(GPU_DVFS_FREQ7, GPU_DVFS_VOLT7_CT1, 7),
	GPUOP(GPU_DVFS_FREQ8, GPU_DVFS_VOLT8_CT1, 8),
	GPUOP(GPU_DVFS_FREQ9, GPU_DVFS_VOLT9_CT1, 9),
	GPUOP(GPU_DVFS_FREQ10, GPU_DVFS_VOLT10_CT1, 10),
	GPUOP(GPU_DVFS_FREQ11, GPU_DVFS_VOLT11_CT1, 11),
	GPUOP(GPU_DVFS_FREQ12, GPU_DVFS_VOLT12_CT1, 12),
	GPUOP(GPU_DVFS_FREQ13, GPU_DVFS_VOLT13_CT1, 13),
	GPUOP(GPU_DVFS_FREQ14, GPU_DVFS_VOLT14_CT1, 14),
	GPUOP(GPU_DVFS_FREQ15, GPU_DVFS_VOLT15_CT1, 15),
};
/* CT2 opp table */
static struct g_opp_table_info g_opp_table_e1_2[] = {
	GPUOP(GPU_DVFS_FREQ0, GPU_DVFS_VOLT0_CT, 0),
	GPUOP(GPU_DVFS_FREQ1, GPU_DVFS_VOLT1_CT2, 1),
	GPUOP(GPU_DVFS_FREQ2, GPU_DVFS_VOLT2_CT2, 2),
	GPUOP(GPU_DVFS_FREQ3, GPU_DVFS_VOLT3_CT2, 3),
	GPUOP(GPU_DVFS_FREQ4, GPU_DVFS_VOLT4_CT2, 4),
	GPUOP(GPU_DVFS_FREQ5, GPU_DVFS_VOLT5_CT2, 5),
	GPUOP(GPU_DVFS_FREQ6, GPU_DVFS_VOLT6_CT2, 6),
	GPUOP(GPU_DVFS_FREQ7, GPU_DVFS_VOLT7_CT2, 7),
	GPUOP(GPU_DVFS_FREQ8, GPU_DVFS_VOLT8_CT2, 8),
	GPUOP(GPU_DVFS_FREQ9, GPU_DVFS_VOLT9_CT2, 9),
	GPUOP(GPU_DVFS_FREQ10, GPU_DVFS_VOLT10_CT2, 10),
	GPUOP(GPU_DVFS_FREQ11, GPU_DVFS_VOLT11_CT2, 11),
	GPUOP(GPU_DVFS_FREQ12, GPU_DVFS_VOLT12_CT2, 12),
	GPUOP(GPU_DVFS_FREQ13, GPU_DVFS_VOLT13_CT2, 13),
	GPUOP(GPU_DVFS_FREQ14, GPU_DVFS_VOLT14_CT2, 14),
	GPUOP(GPU_DVFS_FREQ15, GPU_DVFS_VOLT15_CT2, 15),
};
/* CT3 opp table */
static struct g_opp_table_info g_opp_table_e1_3[] = {
	GPUOP(GPU_DVFS_FREQ0, GPU_DVFS_VOLT0_CT, 0),
	GPUOP(GPU_DVFS_FREQ1, GPU_DVFS_VOLT1_CT3, 1),
	GPUOP(GPU_DVFS_FREQ2, GPU_DVFS_VOLT2_CT3, 2),
	GPUOP(GPU_DVFS_FREQ3, GPU_DVFS_VOLT3_CT3, 3),
	GPUOP(GPU_DVFS_FREQ4, GPU_DVFS_VOLT4_CT3, 4),
	GPUOP(GPU_DVFS_FREQ5, GPU_DVFS_VOLT5_CT3, 5),
	GPUOP(GPU_DVFS_FREQ6, GPU_DVFS_VOLT6_CT3, 6),
	GPUOP(GPU_DVFS_FREQ7, GPU_DVFS_VOLT7_CT3, 7),
	GPUOP(GPU_DVFS_FREQ8, GPU_DVFS_VOLT8_CT3, 8),
	GPUOP(GPU_DVFS_FREQ9, GPU_DVFS_VOLT9_CT3, 9),
	GPUOP(GPU_DVFS_FREQ10, GPU_DVFS_VOLT10_CT3, 10),
	GPUOP(GPU_DVFS_FREQ11, GPU_DVFS_VOLT11_CT3, 11),
	GPUOP(GPU_DVFS_FREQ12, GPU_DVFS_VOLT12_CT3, 12),
	GPUOP(GPU_DVFS_FREQ13, GPU_DVFS_VOLT13_CT3, 13),
	GPUOP(GPU_DVFS_FREQ14, GPU_DVFS_VOLT14_CT3, 14),
	GPUOP(GPU_DVFS_FREQ15, GPU_DVFS_VOLT15_CT3, 15),
};
#else
/* classic opp table */
static struct g_opp_table_info g_opp_table_e1_4[] = {
	GPUOP(GPU_DVFS_FREQ0, GPU_DVFS_VOLT0, 0),
	GPUOP(GPU_DVFS_FREQ1, GPU_DVFS_VOLT1, 1),
	GPUOP(GPU_DVFS_FREQ2, GPU_DVFS_VOLT2, 2),
	GPUOP(GPU_DVFS_FREQ3, GPU_DVFS_VOLT3, 3),
	GPUOP(GPU_DVFS_FREQ4, GPU_DVFS_VOLT4, 4),
	GPUOP(GPU_DVFS_FREQ5, GPU_DVFS_VOLT5, 5),
	GPUOP(GPU_DVFS_FREQ6, GPU_DVFS_VOLT6, 6),
	GPUOP(GPU_DVFS_FREQ7, GPU_DVFS_VOLT7, 7),
	GPUOP(GPU_DVFS_FREQ8, GPU_DVFS_VOLT8, 8),
	GPUOP(GPU_DVFS_FREQ9, GPU_DVFS_VOLT9, 9),
	GPUOP(GPU_DVFS_FREQ10, GPU_DVFS_VOLT10, 10),
	GPUOP(GPU_DVFS_FREQ11, GPU_DVFS_VOLT11, 11),
	GPUOP(GPU_DVFS_FREQ12, GPU_DVFS_VOLT12, 12),
	GPUOP(GPU_DVFS_FREQ13, GPU_DVFS_VOLT13, 13),
	GPUOP(GPU_DVFS_FREQ14, GPU_DVFS_VOLT14, 14),
	GPUOP(GPU_DVFS_FREQ15, GPU_DVFS_VOLT15, 15),
};
#endif

static const struct of_device_id g_gpufreq_of_match[] = {
	{.compatible = "mediatek,mt6775-gpufreq",},
	{},
};

static struct platform_driver g_gpufreq_pdrv = {
	.probe = __mt_gpufreq_pdrv_probe,
	.remove = NULL,
	.driver = {
		   .name = "gpufreq",
		   .owner = THIS_MODULE,
		   .of_match_table = g_gpufreq_of_match,
		   },
};

static bool g_bParking;
static bool g_debug;
static bool g_DVFS_is_ready;
static bool g_DVFS_is_paused_by_ptpod;
static bool g_LPM_state;
static bool g_volt_enable_state;
static bool g_keep_opp_freq_state;
static bool g_fixed_freq_volt_state;
static bool g_pbm_limited_ignore_state;
static bool g_thermal_protect_limited_ignore_state;
static unsigned int g_device_id;
/* static unsigned int g_extra_buck_exist; */
static unsigned int g_opp_idx_num;
static unsigned int g_cur_opp_freq;
static unsigned int g_cur_opp_volt;
static unsigned int g_cur_opp_idx;
static unsigned int g_cur_opp_cond_idx;
static unsigned int g_keep_opp_freq;
static unsigned int g_keep_opp_freq_idx;
static unsigned int g_fixed_freq;
static unsigned int g_fixed_volt;
static unsigned int g_max_limited_idx;
static unsigned int g_pbm_limited_power;
static unsigned int g_thermal_protect_power;
static unsigned int g_vgpu_sfchg_rrate;
static unsigned int g_vgpu_sfchg_frate;
static unsigned int g_DVFS_off_by_ptpod_idx;
#ifdef MT_GPUFREQ_BATT_OC_PROTECT
static bool g_batt_oc_limited_ignore_state;
static unsigned int g_batt_oc_level;
static unsigned int g_batt_oc_limited_idx;
static unsigned int g_batt_oc_limited_idx_lvl_0;
static unsigned int g_batt_oc_limited_idx_lvl_1;
#endif				/* ifdef MT_GPUFREQ_BATT_OC_PROTECT */
#ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT
static bool g_batt_percent_limited_ignore_state;
static unsigned int g_batt_percent_level;
static unsigned int g_batt_percent_limited_idx;
static unsigned int g_batt_percent_limited_idx_lvl_0;
static unsigned int g_batt_percent_limited_idx_lvl_1;
#endif				/* ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT */
#ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT
static bool g_low_batt_limited_ignore_state;
static unsigned int g_low_battery_level;
static unsigned int g_low_batt_limited_idx;
static unsigned int g_low_batt_limited_idx_lvl_0;
static unsigned int g_low_batt_limited_idx_lvl_2;
#endif				/* ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT */
static DEFINE_MUTEX(mt_gpufreq_lock);
static DEFINE_MUTEX(mt_gpufreq_power_lock);
static unsigned int g_limited_idx_array[NUMBER_OF_LIMITED_IDX] = { 0 };
static bool g_limited_ignore_array[NUMBER_OF_LIMITED_IDX] = { false };

static void __iomem *g_apmixed_base;
phys_addr_t gpu_fdvfs_virt_addr;	/* for GED, legacy ?! */

/**
 * ===============================================
 * SECTION : API definition
 * ===============================================
 */

/*
 * API : handle frequency change request
 */
static int enable_gpufreq_switch = 1;
module_param(enable_gpufreq_switch, int, S_IRUGO|S_IWUSR);
unsigned int mt_gpufreq_target(unsigned int idx)
{
	unsigned int target_freq;
	unsigned int target_volt;
	unsigned int target_idx;
	unsigned int target_cond_idx;

	if (!enable_gpufreq_switch) {
		gpufreq_pr_debug("@%s: enable_gpufreq_switch is not ready\n", __func__);
		return -1;
	}

	mutex_lock(&mt_gpufreq_lock);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		mutex_unlock(&mt_gpufreq_lock);
		return -1;
	}

	if (!g_volt_enable_state) {
		gpufreq_pr_debug("@%s: voltage is not enabled\n", __func__);
		mutex_unlock(&mt_gpufreq_lock);
		return -1;
	}
#ifdef MT_GPUFREQ_OPP_STRESS_TEST
	get_random_bytes(&idx, sizeof(idx));
	idx = idx % g_opp_idx_num;
	gpufreq_pr_debug("@%s: OPP stress test index: %d\n", __func__, idx);
#endif				/* ifdef MT_GPUFREQ_OPP_STRESS_TEST */

	if (idx > (g_opp_idx_num - 1)) {
		gpufreq_pr_debug("@%s: OPP index (%d) is out of range\n", __func__, idx);
		mutex_unlock(&mt_gpufreq_lock);
		return -1;
	}

	/* look up for the target OPP table */
	target_freq = g_opp_table[idx].gpufreq_khz;
	target_volt = g_opp_table[idx].gpufreq_volt;
	target_idx = g_opp_table[idx].gpufreq_idx;
	target_cond_idx = idx;

	gpufreq_pr_debug("@%s: receive freq: %d, index: %d\n", __func__, target_freq, target_idx);

	/* OPP freq is limited by Thermal/Power/PBM */
	if (g_max_limited_idx != 0) {
		if (target_freq > g_opp_table[g_max_limited_idx].gpufreq_khz) {
			target_freq = g_opp_table[g_max_limited_idx].gpufreq_khz;
			target_volt = g_opp_table[g_max_limited_idx].gpufreq_volt;
			target_idx = g_opp_table[g_max_limited_idx].gpufreq_idx;
			target_cond_idx = g_max_limited_idx;
			gpufreq_pr_debug
			    ("@%s: OPP freq is limited by Thermal/Power/PBM, g_max_limited_idx = %d\n",
			     __func__, target_cond_idx);
		}
	}

	/* If /proc command keep OPP freq */
	if (g_keep_opp_freq_state) {
		target_freq = g_opp_table[g_keep_opp_freq_idx].gpufreq_khz;
		target_volt = g_opp_table[g_keep_opp_freq_idx].gpufreq_volt;
		target_idx = g_opp_table[g_keep_opp_freq_idx].gpufreq_idx;
		target_cond_idx = g_keep_opp_freq_idx;
		gpufreq_pr_debug("@%s: keep OPP freq, freq = %d, volt = %d, idx = %d\n",
			    __func__, target_freq, target_volt, target_cond_idx);
	}

	/* If /proc command fix the freq and volt */
	if (g_fixed_freq_volt_state) {
		target_freq = g_fixed_freq;
		target_volt = g_fixed_volt;
		target_idx = 0;
		target_cond_idx = 0;
		gpufreq_pr_debug("@%s: fixed both freq and volt, freq = %d, volt = %d\n",
			    __func__, target_freq, target_volt);
	}

	/* keep at max freq when PTPOD is initializing */
	if (g_DVFS_is_paused_by_ptpod) {
		target_freq = g_opp_table[g_DVFS_off_by_ptpod_idx].gpufreq_khz;
		target_volt = GPU_DVFS_PTPOD_DISABLE_VOLT;
		target_idx = g_opp_table[g_DVFS_off_by_ptpod_idx].gpufreq_idx;
		target_cond_idx = g_DVFS_off_by_ptpod_idx;
		gpufreq_pr_debug("@%s: PTPOD disable DVFS, g_DVFS_off_by_ptpod_idx = %d\n",
			    __func__, target_cond_idx);
	}

	/* target freq == current freq && target volt == current volt, skip it */
	if (g_cur_opp_freq == target_freq && g_cur_opp_volt == target_volt) {
		gpufreq_pr_debug("@%s: Freq: %d ---> %d (skipped)\n", __func__, g_cur_opp_freq,
			    target_freq);
		mutex_unlock(&mt_gpufreq_lock);
		return 0;
	}
#ifdef MT_GPUFREQ_AEE_RR_REC
	aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() | (1 << GPU_DVFS_IS_DOING_DVFS));
	aee_rr_rec_gpu_dvfs_oppidx(target_cond_idx);
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

	/* set to the target frequency and voltage */
	__mt_gpufreq_set(g_cur_opp_freq, target_freq, g_cur_opp_volt, target_volt);

	g_cur_opp_idx = target_idx;
	g_cur_opp_cond_idx = target_cond_idx;

#ifdef MT_GPUFREQ_AEE_RR_REC
	aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() & ~(1 << GPU_DVFS_IS_DOING_DVFS));
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

	mutex_unlock(&mt_gpufreq_lock);

	return 0;
}

/*
 * enable Clock Gating
 */
void mt_gpufreq_enable_CG(void)
{
	if (clk_prepare_enable(g_clk->subsys_mfg_cg))
		gpufreq_pr_err("@%s: failed when enable subsys-mfg-cg\n", __func__);

	gpufreq_pr_debug("@%s: enable CG done\n", __func__);
}

/*
 * disable Clock Gating
 */
void mt_gpufreq_disable_CG(void)
{
	clk_disable_unprepare(g_clk->subsys_mfg_cg);

	gpufreq_pr_debug("@%s: disable CG done\n", __func__);
}

/*
 * enable MTCMOS
 */
void mt_gpufreq_enable_MTCMOS(void)
{
	if (clk_prepare_enable(g_clk->mtcmos_mfg_async))
		gpufreq_pr_err("@%s: failed when enable mtcmos-mfg-async\n", __func__);

	if (clk_prepare_enable(g_clk->mtcmos_mfg))
		gpufreq_pr_err("@%s: failed when enable mtcmos-mfg\n", __func__);

	if (clk_prepare_enable(g_clk->mtcmos_mfg_core0))
		gpufreq_pr_err("@%s: failed when enable mtcmos-mfg-core0\n", __func__);

	if (clk_prepare_enable(g_clk->mtcmos_mfg_core1))
		gpufreq_pr_err("@%s: failed when enable mtcmos-mfg-core1\n", __func__);

	if (clk_prepare_enable(g_clk->mtcmos_mfg_core2))
		gpufreq_pr_err("@%s: failed when enable mtcmos-mfg-core2\n", __func__);

	if (clk_prepare_enable(g_clk->mtcmos_mfg_core3))
		gpufreq_pr_err("@%s: failed when enable mtcmos-mfg-core3\n", __func__);

	gpufreq_pr_debug("@%s: enable MTCMOS done\n", __func__);
}

/*
 * disable MTCMOS
 */
void mt_gpufreq_disable_MTCMOS(void)
{
	clk_disable_unprepare(g_clk->mtcmos_mfg_core3);
	clk_disable_unprepare(g_clk->mtcmos_mfg_core2);
	clk_disable_unprepare(g_clk->mtcmos_mfg_core1);
	clk_disable_unprepare(g_clk->mtcmos_mfg_core0);
	clk_disable_unprepare(g_clk->mtcmos_mfg);
	clk_disable_unprepare(g_clk->mtcmos_mfg_async);

	gpufreq_pr_debug("@%s: disable MTCMOS done\n", __func__);
}

/*
 * API : GPU voltage on/off setting
 * 0 : off
 * 1 : on
 */
unsigned int mt_gpufreq_voltage_enable_set(unsigned int enable)
{
	int ret = 0;

	mutex_lock(&mt_gpufreq_lock);

	gpufreq_pr_debug("@%s: begin, enable = %d, g_volt_enable_state = %d\n",
		    __func__, enable, g_volt_enable_state);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		ret = -1;
		goto SET_EXIT;
	}

	if (g_DVFS_is_paused_by_ptpod) {
		if (enable == BUCK_OFF) {
			gpufreq_pr_debug("@%s: DVFS is paused by PTPOD\n", __func__);
			ret = -1;
			goto SET_EXIT;
		}
	}

	if (enable == BUCK_ON) {
		ret = regulator_enable(g_pmic->reg_vgpu);
		if (ret) {
			gpufreq_pr_err("@%s: enable VGPU failed, ret = %d\n", __func__, ret);
			goto SET_EXIT;
		}
		udelay(VGPU_ENABLE_TIME_US);
	} else {
		ret = regulator_disable(g_pmic->reg_vgpu);
		if (ret) {
			gpufreq_pr_err("@%s: disable VGPU failed, ret = %d\n", __func__, ret);
			goto SET_EXIT;
		}
	}

	if (regulator_is_enabled(g_pmic->reg_vgpu) > 0) {
		gpufreq_pr_debug("@%s: VGPU is on\n", __func__);
		g_volt_enable_state = true;
		__mt_gpufreq_kick_pbm(1);
	} else if (regulator_is_enabled(g_pmic->reg_vgpu) == 0) {
		gpufreq_pr_debug("@%s: VGPU is off\n", __func__);
		g_volt_enable_state = false;
		__mt_gpufreq_kick_pbm(0);
	}
#ifdef MT_GPUFREQ_AEE_RR_REC
	if (enable == BUCK_ON)
		aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() |
					   (1 << GPU_DVFS_IS_VGPU_ENABLED));
	else
		aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() &
					   ~(1 << GPU_DVFS_IS_VGPU_ENABLED));
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

SET_EXIT:

	gpufreq_pr_debug("@%s: end, enable = %d, g_volt_enable_state = %d\n",
		    __func__, enable, g_volt_enable_state);

	mutex_unlock(&mt_gpufreq_lock);

	return ret;
}

/*
 * API : set Low Power Mode (LPM)
 * 0 : leave LPM
 * 1 : enter LPM
 */
unsigned int mt_gpufreq_voltage_lpm_set(unsigned int enable_LPM)
{
	int ret = 0;

	mutex_lock(&mt_gpufreq_lock);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		ret = -1;
		goto SET_LPM_EXIT;
	}

	if (g_DVFS_is_paused_by_ptpod) {
		gpufreq_pr_debug("@%s: DVFS is paused by ptpod\n", __func__);
		ret = -1;
		goto SET_LPM_EXIT;
	}

	if (enable_LPM) {
		if (!g_LPM_state) {
			__mt_gpufreq_volt_switch(g_cur_opp_volt, PMIC_VGPU_LPM_VOLT);
			g_LPM_state = true;
		}
		goto SET_LPM_EXIT;
	} else {
		if (g_LPM_state) {
			__mt_gpufreq_volt_switch(PMIC_VGPU_LPM_VOLT, g_cur_opp_volt);
			g_LPM_state = false;
		}
		goto SET_LPM_EXIT;
	}

SET_LPM_EXIT:

	gpufreq_pr_debug("@%s: enable_LPM = %d, g_LPM_state = %d\n", __func__, enable_LPM, g_LPM_state);

	mutex_unlock(&mt_gpufreq_lock);

	return ret;
}

/*
 * API : enable DVFS for PTPOD initializing
 */
void mt_gpufreq_enable_by_ptpod(void)
{
	/* Set GPU Buck to leave PWM mode */
	__mt_gpufreq_buck_set_PWM_mode(0);

	/* Freerun GPU DVFS */
	g_DVFS_is_paused_by_ptpod = false;

	/* Turn off GPU MTCMOS */
	mt_gpufreq_disable_MTCMOS();

	/* Turn off GPU PMIC Buck */
	/* No buck on/off flow for stability since GPU share VSRAM_CORE with others */
	/* mt_gpufreq_voltage_enable_set(0); */

	gpufreq_pr_debug("@%s: DVFS is enabled by ptpod\n", __func__);
}

/*
 * API : disable DVFS for PTPOD initializing
 */
void mt_gpufreq_disable_by_ptpod(void)
{
	int i = 0;
	int target_idx = 0;

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return;
	}

	/* Turn on GPU PMIC Buck */
	/* No buck on/off flow for stability since GPU share VSRAM_CORE with others */
	/* mt_gpufreq_voltage_enable_set(1); */

	/* Turn on GPU MTCMOS */
	mt_gpufreq_enable_MTCMOS();

	/* Pause GPU DVFS */
	g_DVFS_is_paused_by_ptpod = true;

	/* Fix GPU @ 0.8V */
	for (i = 0; i < g_opp_idx_num; i++) {
		if (g_opp_table_default[i].gpufreq_volt <= GPU_DVFS_PTPOD_DISABLE_VOLT) {
			target_idx = i;
			break;
		}
	}
	g_DVFS_off_by_ptpod_idx = (unsigned int)target_idx;
	mt_gpufreq_target(target_idx);

	/* Set GPU Buck to enter PWM mode */
	__mt_gpufreq_buck_set_PWM_mode(1);

	gpufreq_pr_debug("@%s: DVFS is disabled by ptpod\n", __func__);
}

/*
 * API : update OPP and switch back to default voltage setting
 */
void mt_gpufreq_restore_default_volt(void)
{
	int i;

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return;
	}

	mutex_lock(&mt_gpufreq_lock);

	gpufreq_pr_debug("@%s: restore OPP table to default voltage\n", __func__);

	for (i = 0; i < g_opp_idx_num; i++) {
		g_opp_table[i].gpufreq_volt = g_opp_table_default[i].gpufreq_volt;
		gpufreq_pr_debug("@%s: g_opp_table[%d].gpufreq_volt = %x\n",
			    __func__, i, g_opp_table[i].gpufreq_volt);
	}

	__mt_gpufreq_volt_switch(g_cur_opp_volt, g_opp_table[g_cur_opp_cond_idx].gpufreq_volt);

	g_cur_opp_volt = g_opp_table[g_cur_opp_cond_idx].gpufreq_volt;

	mutex_unlock(&mt_gpufreq_lock);
}

/*
 * API : update OPP and set voltage because PTPOD modified voltage table by PMIC wrapper
 */
unsigned int mt_gpufreq_update_volt(unsigned int pmic_volt[], unsigned int array_size)
{
	int i;

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return -1;
	}

	mutex_lock(&mt_gpufreq_lock);

	gpufreq_pr_debug("@%s: update OPP table to given voltage\n", __func__);

	for (i = 0; i < array_size; i++) {
		g_opp_table[i].gpufreq_volt = pmic_volt[i];
		gpufreq_pr_debug("@%s: g_opp_table[%d].gpufreq_volt = %d\n",
			    __func__, i, g_opp_table[i].gpufreq_volt);
	}
	/* Cannon GPU PIC: handling corner tightening chips */
	if (g_opp_table[0].gpufreq_volt > 80000) {
		/* set VSRAM_CORE to 881250 uV when highest VGPU volt is larger than 800000 uv */
		regulator_set_voltage(g_pmic->reg_vsram_core, 881250, 881250 + 125);
	}

	__mt_gpufreq_volt_switch(g_cur_opp_volt, g_opp_table[g_cur_opp_cond_idx].gpufreq_volt);

	g_cur_opp_volt = g_opp_table[g_cur_opp_cond_idx].gpufreq_volt;

	mutex_unlock(&mt_gpufreq_lock);

	return 0;
}

/* API : get OPP table index number */
unsigned int mt_gpufreq_get_dvfs_table_num(void)
{
	return g_opp_idx_num;
}

/* API : get frequency via OPP table index */
unsigned int mt_gpufreq_get_freq_by_idx(unsigned int idx)
{
	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return -1;
	}

	if (idx < g_opp_idx_num) {
		gpufreq_pr_debug("@%s: idx = %d, freq = %d\n", __func__, idx,
			    g_opp_table[idx].gpufreq_khz);
		return g_opp_table[idx].gpufreq_khz;
	}
	gpufreq_pr_debug("@%s: not found, idx = %d\n", __func__, idx);
	return 0;
}

/* API : get voltage via OPP table index */
unsigned int mt_gpufreq_get_volt_by_idx(unsigned int idx)
{
	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return -1;
	}

	if (idx < g_opp_idx_num) {
		gpufreq_pr_debug("@%s: idx = %d, volt = %d\n", __func__, idx,
			    g_opp_table[idx].gpufreq_volt);
		return g_opp_table[idx].gpufreq_volt;
	}
	gpufreq_pr_debug("@%s: not found, idx = %d\n", __func__, idx);
	return 0;
}

/* API : get max power on power table */
unsigned int mt_gpufreq_get_max_power(void)
{
	return (!g_power_table) ? 0 : g_power_table[0].gpufreq_power;
}

/* API : get min power on power table */
unsigned int mt_gpufreq_get_min_power(void)
{
	return (!g_power_table) ? 0 : g_power_table[g_opp_idx_num - 1].gpufreq_power;
}

/* API : get static leakage power */
unsigned int mt_gpufreq_get_leakage_mw(void)
{
	int temp = 0;
#ifdef MT_GPUFREQ_STATIC_PWR_READY2USE
	unsigned int cur_vcore = __mt_gpufreq_get_cur_gpu_volt() / 100;
	int leak_power;
#endif				/* ifdef MT_GPUFREQ_STATIC_PWR_READY2USE */

#ifdef CONFIG_THERMAL
	temp = get_immediate_gpu_wrap() / 1000;
#else
	temp = 40;
#endif				/* ifdef CONFIG_THERMAL */

#ifdef MT_GPUFREQ_STATIC_PWR_READY2USE
	leak_power = mt_spower_get_leakage(MTK_SPOWER_GPU, cur_vcore, temp);
	if (g_volt_enable_state && leak_power > 0)
		return leak_power;
	else
		return 0;
#else
	return 130;
#endif				/* ifdef MT_GPUFREQ_STATIC_PWR_READY2USE */
}

/*
 * API : get current Thermal/Power/PBM limited OPP table index
 */
unsigned int mt_gpufreq_get_thermal_limit_index(void)
{
	gpufreq_pr_debug("@%s: current GPU Thermal/Power/PBM limit index is %d\n",
		    __func__, g_max_limited_idx);
	return g_max_limited_idx;
}

/*
 * API : get current Thermal/Power/PBM limited OPP table frequency
 */
unsigned int mt_gpufreq_get_thermal_limit_freq(void)
{
	gpufreq_pr_debug("@%s: current GPU thermal limit freq is %d MHz\n",
		    __func__, g_opp_table[g_max_limited_idx].gpufreq_khz / 1000);
	return g_opp_table[g_max_limited_idx].gpufreq_khz;
}

/*
 * API : get current OPP table conditional index
 */
unsigned int mt_gpufreq_get_cur_freq_index(void)
{
	gpufreq_pr_debug("@%s: current OPP table conditional index is %d\n", __func__,
		    g_cur_opp_cond_idx);
	return g_cur_opp_cond_idx;
}

/*
 * API : get current OPP table frequency
 */
unsigned int mt_gpufreq_get_cur_freq(void)
{
	gpufreq_pr_debug("@%s: current frequency is %d MHz\n", __func__, g_cur_opp_freq / 1000);
	return g_cur_opp_freq;
}
EXPORT_SYMBOL(mt_gpufreq_get_cur_freq);

/*
 * API : get current voltage
 */
unsigned int mt_gpufreq_get_cur_volt(void)
{
	return __mt_gpufreq_get_cur_gpu_volt();
}

/* API : get Thermal/Power/PBM limited OPP table index */
int mt_gpufreq_get_cur_ceiling_idx(void)
{
	return (int)mt_gpufreq_get_thermal_limit_index();
}

#ifdef MT_GPUFREQ_BATT_OC_PROTECT
/*
 * API : Over Currents(OC) Callback
 */
void mt_gpufreq_batt_oc_callback(BATTERY_OC_LEVEL battery_oc_level)
{
	gpufreq_pr_debug("@%s: battery_oc_level = %d\n", __func__, battery_oc_level);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return;
	}

	if (g_batt_oc_limited_ignore_state) {
		gpufreq_pr_debug("@%s: ignore Over Currents(OC) protection\n", __func__);
		return;
	}

	g_batt_oc_level = battery_oc_level;

	if (battery_oc_level == BATTERY_OC_LEVEL_1) {
		if (g_batt_oc_limited_idx != g_batt_oc_limited_idx_lvl_1) {
			g_batt_oc_limited_idx = g_batt_oc_limited_idx_lvl_1;
			__mt_gpufreq_batt_oc_protect(g_batt_oc_limited_idx_lvl_1);	/* Limit */
		}
	} else {
		if (g_batt_oc_limited_idx != g_batt_oc_limited_idx_lvl_0) {
			g_batt_oc_limited_idx = g_batt_oc_limited_idx_lvl_0;
			__mt_gpufreq_batt_oc_protect(g_batt_oc_limited_idx_lvl_0);	/* Unlimit */
		}
	}
}
#endif				/* ifdef MT_GPUFREQ_BATT_OC_PROTECT */

#ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT
/*
 * API : Battery Percentage Callback
 */
void mt_gpufreq_batt_percent_callback(BATTERY_PERCENT_LEVEL battery_percent_level)
{
	gpufreq_pr_debug("@%s: battery_percent_level = %d\n", __func__, battery_percent_level);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return;
	}

	if (g_batt_percent_limited_ignore_state) {
		gpufreq_pr_debug("@%s: ignore Battery Percentage protection\n", __func__);
		return;
	}

	g_batt_percent_level = battery_percent_level;

	/* BATTERY_PERCENT_LEVEL_1: <= 15%, BATTERY_PERCENT_LEVEL_0: >15% */
	if (battery_percent_level == BATTERY_PERCENT_LEVEL_1) {
		if (g_batt_percent_limited_idx != g_batt_percent_limited_idx_lvl_1) {
			g_batt_percent_limited_idx = g_batt_percent_limited_idx_lvl_1;
			__mt_gpufreq_batt_percent_protect(g_batt_percent_limited_idx_lvl_1);
		}
	} else {
		if (g_batt_percent_limited_idx != g_batt_percent_limited_idx_lvl_0) {
			g_batt_percent_limited_idx = g_batt_percent_limited_idx_lvl_0;
			__mt_gpufreq_batt_percent_protect(g_batt_percent_limited_idx_lvl_0);	/* Unlimit */
		}
	}
}
#endif				/* ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT */

#ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT
/*
 * API : Low Battery Volume Callback
 */
void mt_gpufreq_low_batt_callback(LOW_BATTERY_LEVEL low_battery_level)
{
	gpufreq_pr_debug("@%s: low_battery_level = %d\n", __func__, low_battery_level);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return;
	}

	if (g_low_batt_limited_ignore_state) {
		gpufreq_pr_debug("@%s: ignore Low Battery Volume protection\n", __func__);
		return;
	}

	g_low_battery_level = low_battery_level;

	/*
	 * 3.25V HW issue int and is_low_battery = 1
	 * 3.00V HW issue int and is_low_battery = 2
	 * 3.50V HW issue int and is_low_battery = 0
	 */

	if (low_battery_level == LOW_BATTERY_LEVEL_2) {
		if (g_low_batt_limited_idx != g_low_batt_limited_idx_lvl_2) {
			g_low_batt_limited_idx = g_low_batt_limited_idx_lvl_2;
			__mt_gpufreq_low_batt_protect(g_low_batt_limited_idx_lvl_2);
		}
	} else {
		if (g_low_batt_limited_idx != g_low_batt_limited_idx_lvl_0) {
			g_low_batt_limited_idx = g_low_batt_limited_idx_lvl_0;
			__mt_gpufreq_low_batt_protect(g_low_batt_limited_idx_lvl_0);	/* Unlimit */
		}
	}
}
#endif				/* ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT */

/*
 * API : set limited OPP table index for Thermal protection
 */
void mt_gpufreq_thermal_protect(unsigned int limited_power)
{
	int i = -1;
	unsigned int limited_freq;

	mutex_lock(&mt_gpufreq_power_lock);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		mutex_unlock(&mt_gpufreq_power_lock);
		return;
	}

	if (g_thermal_protect_limited_ignore_state) {
		gpufreq_pr_debug("@%s: ignore Thermal protection\n", __func__);
		mutex_unlock(&mt_gpufreq_power_lock);
		return;
	}

	if (limited_power == g_thermal_protect_power) {
		gpufreq_pr_debug("@%s: limited_power(%d mW) not changed, skip it\n", __func__,
			    limited_power);
		mutex_unlock(&mt_gpufreq_power_lock);
		return;
	}

	g_thermal_protect_power = limited_power;

#ifdef MT_GPUFREQ_DYNAMIC_POWER_TABLE_UPDATE
	__mt_update_gpufreqs_power_table();
#endif				/* ifdef MT_GPUFREQ_DYNAMIC_POWER_TABLE_UPDATE */

	gpufreq_pr_debug("@%s: limited power = %d\n", __func__, limited_power);

	if (limited_power == 0) {
		g_limited_idx_array[IDX_THERMAL_PROTECT_LIMITED] = 0;
		__mt_gpufreq_update_max_limited_idx();
	} else {
		limited_freq = __mt_gpufreq_get_limited_freq_by_power(limited_power);
		for (i = 0; i < g_opp_idx_num; i++) {
			if (g_opp_table[i].gpufreq_khz <= limited_freq) {
				g_limited_idx_array[IDX_THERMAL_PROTECT_LIMITED] = i;
				__mt_gpufreq_update_max_limited_idx();
				break;
			}
		}
	}

	gpufreq_pr_debug("@%s: limited power index = %d\n", __func__, i);

	mutex_unlock(&mt_gpufreq_power_lock);
}

/* API : set limited OPP table index by PBM */
void mt_gpufreq_set_power_limit_by_pbm(unsigned int limited_power)
{
	int i = -1;
	unsigned int limited_freq;

	mutex_lock(&mt_gpufreq_power_lock);

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		mutex_unlock(&mt_gpufreq_power_lock);
		return;
	}

	if (g_pbm_limited_ignore_state) {
		gpufreq_pr_debug("@%s: ignore PBM Power limited\n", __func__);
		mutex_unlock(&mt_gpufreq_power_lock);
		return;
	}

	if (limited_power == g_pbm_limited_power) {
		gpufreq_pr_debug("@%s: limited_power(%d mW) not changed, skip it\n",
			     __func__, limited_power);
		mutex_unlock(&mt_gpufreq_power_lock);
		return;
	}

	g_pbm_limited_power = limited_power;

	gpufreq_pr_debug("@%s: limited_power = %d\n", __func__, limited_power);

	if (limited_power == 0) {
		g_limited_idx_array[IDX_PBM_LIMITED] = 0;
		__mt_gpufreq_update_max_limited_idx();
	} else {
		limited_freq = __mt_gpufreq_get_limited_freq_by_power(limited_power);
		for (i = 0; i < g_opp_idx_num; i++) {
			if (g_opp_table[i].gpufreq_khz <= limited_freq) {
				g_limited_idx_array[IDX_PBM_LIMITED] = i;
				__mt_gpufreq_update_max_limited_idx();
				break;
			}
		}
	}

	gpufreq_pr_debug("@%s: limited power index = %d\n", __func__, i);

	mutex_unlock(&mt_gpufreq_power_lock);
}

/*
 * API : set GPU loading for SSPM
 */
void mt_gpufreq_set_loading(unsigned int gpu_loading)
{
	/* legacy */
}

/*
 * API : register GPU power limited notifiction callback
 */
void mt_gpufreq_power_limit_notify_registerCB(gpufreq_power_limit_notify pCB)
{
	/* legacy */
}

/**
 * ===============================================
 * SECTION : PROCFS interface for debugging
 * ===============================================
 */

#ifdef CONFIG_PROC_FS
/*
 * PROCFS : show OPP table
 */
static int mt_gpufreq_opp_dump_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < g_opp_idx_num; i++) {
		seq_printf(m, "[%d] ", i);
		seq_printf(m, "freq = %d, ", g_opp_table[i].gpufreq_khz);
		seq_printf(m, "volt = %d\n", g_opp_table[i].gpufreq_volt);
	}

	return 0;
}

/*
 * PROCFS : show OPP power table
 */
static int mt_gpufreq_power_dump_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < g_opp_idx_num; i++) {
		seq_printf(m, "[%d] ", i);
		seq_printf(m, "freq = %d, ", g_power_table[i].gpufreq_khz);
		seq_printf(m, "volt = %d, ", g_power_table[i].gpufreq_volt);
		seq_printf(m, "power = %d\n", g_power_table[i].gpufreq_power);
	}

	return 0;
}

/*
 * PROCFS : show important variables for debugging
 */
static int mt_gpufreq_var_dump_proc_show(struct seq_file *m, void *v)
{
	int i;
	unsigned int gpu_loading = 0;

	mtk_get_gpu_loading(&gpu_loading);

	seq_printf(m, "g_cur_opp_idx = %d, g_cur_opp_cond_idx = %d\n",
		   g_cur_opp_idx, g_cur_opp_cond_idx);
	seq_printf(m, "g_cur_opp_freq = %d, g_cur_opp_volt = %d\n", g_cur_opp_freq, g_cur_opp_volt);
	seq_printf(m, "real freq = %d, real gpu_volt = %d, real_sram_core_volt = %d\n",
		   __mt_gpufreq_get_cur_freq(), __mt_gpufreq_get_cur_gpu_volt(),
		   __mt_gpufreq_get_cur_sram_core_volt());
	seq_printf(m, "clock freq = %d\n", mt_get_ckgen_freq(10));
	seq_printf(m, "g_device_id = %d\n", g_device_id);
	seq_printf(m, "g_LPM_state = %d\n", g_LPM_state);
	seq_printf(m, "g_volt_enable_state = %d\n", g_volt_enable_state);
	seq_printf(m, "g_DVFS_off_by_ptpod_idx = %d\n", g_DVFS_off_by_ptpod_idx);
	seq_printf(m, "g_max_limited_idx = %d\n", g_max_limited_idx);
	seq_printf(m, "gpu_loading = %d\n", gpu_loading);

	for (i = 0; i < NUMBER_OF_LIMITED_IDX; i++)
		seq_printf(m, "g_limited_idx_array[%d] = %d\n", i, g_limited_idx_array[i]);
	seq_printf(m, "leakage_power(mW) = %u\n", mt_gpufreq_get_leakage_mw());

	return 0;
}

/*
 * PROCFS : show current debugging state
 */
static int mt_gpufreq_debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "GPU-DVFS debugging is %s (0x%1x)\n", g_debug ? "enabled" : "disabled",
		   g_debug);
	return 0;
}

/*
 * PROCFS : debugging message setting
 * 0 : hide
 * 1 : show
 */
static ssize_t mt_gpufreq_debug_proc_write(struct file *file, const char __user *buffer,
					   size_t count, loff_t *data)
{
	char buf[32];
	int len = 0;
	int value = 0;
	int ret = 0;

	ret = -EFAULT;
	len = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);

	if (copy_from_user(buf, buffer, len))
		goto out;

	buf[len] = '\0';

	if (!kstrtoint(buf, 0, &value)) {
		if (!value || !(value - 1)) {
			ret = 0;
			g_debug = value;
		}
	}

out:
	return (ret < 0) ? ret : count;
}

/*
 * PROCFS : show Thermal/Power/PBM limited ignore state
 * 0 : consider
 * 1 : ignore
 */
static int mt_gpufreq_power_limited_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "GPU-DVFS power limited state ....\n");
	seq_printf(m, "g_batt_oc_limited_ignore_state = %d\n", g_batt_oc_limited_ignore_state);
	seq_printf(m, "g_batt_percent_limited_ignore_state = %d\n",
		   g_batt_percent_limited_ignore_state);
	seq_printf(m, "g_low_batt_limited_ignore_state = %d\n", g_low_batt_limited_ignore_state);
	seq_printf(m, "g_thermal_protect_limited_ignore_state = %d\n",
		   g_thermal_protect_limited_ignore_state);
	seq_printf(m, "g_pbm_limited_ignore_state = %d\n", g_pbm_limited_ignore_state);
	return 0;
}

/*
 * PROCFS : ignore state or power value setting for Thermal/Power/PBM limit
 */
static ssize_t mt_gpufreq_power_limited_proc_write(struct file *file,
						   const char __user *buffer, size_t count,
						   loff_t *data)
{
	char buf[64];
	int len = 0;
	int ret = 0;
	int i;
	int size;
	unsigned int value = 0;
	static const char * const array[] = {
#ifdef MT_GPUFREQ_BATT_OC_PROTECT
		"ignore_batt_oc",
#endif				/* ifdef MT_GPUFREQ_BATT_OC_PROTECT */
#ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT
		"ignore_batt_percent",
#endif				/* ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT */
#ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT
		"ignore_low_batt",
#endif				/* ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT */
		"ignore_thermal_protect",
		"ignore_pbm_limited",
		"pbm_limited_power",
		"thermal_protect_power",
	};

	ret = -EFAULT;
	len = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);

	if (copy_from_user(buf, buffer, len))
		goto out;

	buf[len] = '\0';

	size = ARRAY_SIZE(array);

	for (i = 0; i < size; i++) {
		if (strncmp(array[i], buf, MIN(strlen(array[i]), count)) == 0) {
			char cond_buf[64];

			snprintf(cond_buf, sizeof(cond_buf), "%s %%u", array[i]);
			if (sscanf(buf, cond_buf, &value) == 1) {
				ret = 0;
				if (strncmp(array[i], "pbm_limited_power", strlen(array[i])) == 0) {
					mt_gpufreq_set_power_limit_by_pbm(value);
				} else
				    if (strncmp(array[i], "thermal_protect_power", strlen(array[i]))
					== 0) {
					mt_gpufreq_thermal_protect(value);
				}
#ifdef MT_GPUFREQ_BATT_OC_PROTECT
				else if (strncmp(array[i], "ignore_batt_oc", strlen(array[i])) == 0) {
					if (!value || !(value - 1)) {
						g_batt_oc_limited_ignore_state =
						    (value) ? true : false;
						g_limited_ignore_array[IDX_BATT_OC_LIMITED] =
						    g_batt_oc_limited_ignore_state;
					}
				}
#endif				/* ifdef MT_GPUFREQ_BATT_OC_PROTECT */
#ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT
				else if (strncmp(array[i], "ignore_batt_percent", strlen(array[i]))
					 == 0) {
					if (!value || !(value - 1)) {
						g_batt_percent_limited_ignore_state =
						    (value) ? true : false;
						g_limited_ignore_array[IDX_BATT_PERCENT_LIMITED] =
						    g_batt_percent_limited_ignore_state;
					}
				}
#endif				/* ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT */
#ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT
				else if (strncmp(array[i], "ignore_low_batt", strlen(array[i])) ==
					 0) {
					if (!value || !(value - 1)) {
						g_low_batt_limited_ignore_state =
						    (value) ? true : false;
						g_limited_ignore_array[IDX_LOW_BATT_LIMITED] =
						    g_low_batt_limited_ignore_state;
					}
				}
#endif				/* ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT */
				else if (strncmp
					 (array[i], "ignore_thermal_protect",
					  strlen(array[i])) == 0) {
					if (!value || !(value - 1)) {
						g_thermal_protect_limited_ignore_state =
						    (value) ? true : false;
						g_limited_ignore_array[IDX_THERMAL_PROTECT_LIMITED]
						    = g_thermal_protect_limited_ignore_state;
					}
				} else if (strncmp(array[i], "ignore_pbm_limited", strlen(array[i]))
					   == 0) {
					if (!value || !(value - 1)) {
						g_pbm_limited_ignore_state = (value) ? true : false;
						g_limited_ignore_array[IDX_PBM_LIMITED] =
						    g_pbm_limited_ignore_state;
					}
				}
				break;
			}
		}
	}

out:
	return (ret < 0) ? ret : count;
}

/*
 * PROCFS : show current keeping OPP frequency state
 */
static int mt_gpufreq_opp_freq_proc_show(struct seq_file *m, void *v)
{
	if (g_keep_opp_freq_state) {
		seq_printf(m, "[%d] ", g_keep_opp_freq_idx);
		seq_printf(m, "freq = %d, ", g_opp_table[g_keep_opp_freq_idx].gpufreq_khz);
		seq_printf(m, "volt = %d\n", g_opp_table[g_keep_opp_freq_idx].gpufreq_volt);
	} else
		seq_puts(m, "Keeping OPP frequency is disabled\n");

	return 0;
}

/*
 * PROCFS : keeping OPP frequency setting
 * 0 : free run
 * 1 : keep OPP frequency
 */
static ssize_t mt_gpufreq_opp_freq_proc_write(struct file *file,
					      const char __user *buffer, size_t count,
					      loff_t *data)
{
	char buf[32];
	int len = 0;
	int value = 0;
	int i = 0;
	int ret = 0;

	ret = -EFAULT;
	len = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);

	if (copy_from_user(buf, buffer, len))
		goto out;

	buf[len] = '\0';

	if (kstrtoint(buf, 0, &value) == 0) {
		if (value == 0) {
			g_keep_opp_freq_state = false;
		} else {
			for (i = 0; i < g_opp_idx_num; i++) {
				if (value == g_opp_table[i].gpufreq_khz) {
					ret = 0;
					g_keep_opp_freq_idx = i;
					g_keep_opp_freq_state = true;
					g_keep_opp_freq = value;
					mt_gpufreq_voltage_enable_set(1);
					mt_gpufreq_target(i);
					break;
				}
			}
		}
	}

out:
	return (ret < 0) ? ret : count;
}

/*
 * PROCFS : show current GPU voltage enable state
 */
static int mt_gpufreq_volt_enable_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "GPU voltage enable = %d\n", g_volt_enable_state);
	return 0;
}

/*
 * PROCFS : GPU voltage enable state setting
 * 0 : disable
 * 1 : enable
 */
static ssize_t mt_gpufreq_volt_enable_proc_write(struct file *file,
						 const char __user *buffer, size_t count,
						 loff_t *data)
{
	char buf[32];
	int len = 0;
	int value = 0;
	int ret = 0;

	ret = -EFAULT;
	len = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);

	if (copy_from_user(buf, buffer, len))
		goto out;

	buf[len] = '\0';

	if (!kstrtoint(buf, 0, &value)) {
		if (!value || !(value - 1)) {
			ret = 0;
			mt_gpufreq_voltage_enable_set(value);
		}
	}

out:
	return (ret < 0) ? ret : count;
}

/*
 * PROCFS : show current fixed freq & volt state
 */
static int mt_gpufreq_fixed_freq_volt_proc_show(struct seq_file *m, void *v)
{
	if (g_fixed_freq_volt_state) {
		seq_puts(m, "GPU-DVFS fixed freq & volt is enabled\n");
		seq_printf(m, "g_fixed_freq = %d\n", g_fixed_freq);
		seq_printf(m, "g_fixed_volt = %d\n", g_fixed_volt);
	} else
		seq_puts(m, "GPU-DVFS fixed freq & volt is disabled\n");

	return 0;
}

/*
 * PROCFS : fixed freq & volt state setting
 */
static ssize_t mt_gpufreq_fixed_freq_volt_proc_write(struct file *file,
						     const char __user *buffer, size_t count,
						     loff_t *data)
{
	char buf[32];
	int len = 0;
	int ret = 0;
	int fixed_freq = 0;
	int fixed_volt = 0;

	ret = -EFAULT;
	len = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);

	if (copy_from_user(buf, buffer, len))
		goto out;

	buf[len] = '\0';

	if (sscanf(buf, "%d %d", &fixed_freq, &fixed_volt) == 2) {
		ret = 0;
		if ((fixed_freq == 0) && (fixed_volt == 0)) {
			g_fixed_freq_volt_state = false;
			g_fixed_freq = 0;
			g_fixed_volt = 0;
		} else {
			g_cur_opp_freq = __mt_gpufreq_get_cur_freq();
			if (fixed_freq > g_cur_opp_freq) {
				__mt_gpufreq_set_fixed_volt(fixed_volt);
				__mt_gpufreq_set_fixed_freq(fixed_freq);
			} else {
				__mt_gpufreq_set_fixed_freq(fixed_freq);
				__mt_gpufreq_set_fixed_volt(fixed_volt);
			}
			g_fixed_freq_volt_state = true;
		}
	}

out:
	return (ret < 0) ? ret : count;
}

/*
 * PROCFS : initialization
 */
PROC_FOPS_RW(gpufreq_debug);
PROC_FOPS_RW(gpufreq_power_limited);
PROC_FOPS_RO(gpufreq_opp_dump);
PROC_FOPS_RO(gpufreq_power_dump);
PROC_FOPS_RW(gpufreq_opp_freq);
PROC_FOPS_RO(gpufreq_var_dump);
PROC_FOPS_RW(gpufreq_volt_enable);
PROC_FOPS_RW(gpufreq_fixed_freq_volt);
static int __mt_gpufreq_create_procfs(void)
{
	struct proc_dir_entry *dir = NULL;
	int i;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(gpufreq_debug),
		PROC_ENTRY(gpufreq_power_limited),
		PROC_ENTRY(gpufreq_opp_dump),
		PROC_ENTRY(gpufreq_power_dump),
		PROC_ENTRY(gpufreq_opp_freq),
		PROC_ENTRY(gpufreq_var_dump),
		PROC_ENTRY(gpufreq_volt_enable),
		PROC_ENTRY(gpufreq_fixed_freq_volt),
	};

	dir = proc_mkdir("gpufreq", NULL);
	if (!dir) {
		gpufreq_pr_err("@%s: fail to create /proc/gpufreq\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create
		    (entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, dir, entries[i].fops))
			gpufreq_pr_err("@%s: create /proc/gpufreq/%s failed\n", __func__,
				    entries[i].name);
	}

	return 0;
}
#endif				/* ifdef CONFIG_PROC_FS */

/**
 * ===============================================
 * SECTION : Local functions definition
 * ===============================================
 */

/*
 * frequency ramp up/down handler
 * - frequency ramp up need to wait voltage settle
 * - frequency ramp down do not need to wait voltage settle
 */
static void __mt_gpufreq_set(unsigned int freq_old, unsigned int freq_new,
			     unsigned int volt_old, unsigned int volt_new)
{
	gpufreq_pr_debug("@%s: freq: %d ---> %d, volt: %d ---> %d\n",
		    __func__, freq_old, freq_new, volt_old, volt_new);

	if (freq_new > freq_old) {
		__mt_gpufreq_volt_switch(volt_old, volt_new);
		__mt_gpufreq_clock_switch(freq_new);
	} else {
		__mt_gpufreq_clock_switch(freq_new);
		__mt_gpufreq_volt_switch(volt_old, volt_new);
	}

	gpufreq_pr_debug("@%s: real_freq = %d, real_volt = %d\n",
		    __func__, mt_get_ckgen_freq(10), __mt_gpufreq_get_cur_gpu_volt());

	g_cur_opp_freq = freq_new;
	g_cur_opp_volt = volt_new;

	__mt_gpufreq_kick_pbm(1);
}

/*
 * switch clock(frequency) via PLL
 */
#ifndef GPU_ALWAYS_ON
static void gpu_dvfs_switch_to_univpll(bool on)
{
	clk_prepare_enable(g_clk->clk_mux);

	if (on) {
		clk_set_parent(g_clk->clk_mux, g_clk->clk_sub_parent);
	} else {
		clk_set_parent(g_clk->clk_mux, g_clk->clk_main_parent);
	}
	clk_disable_unprepare(g_clk->clk_mux);
}
#endif

static void __mt_gpufreq_clock_switch(unsigned int freq_new)
{
	enum g_post_div_order_enum post_div_order;
	unsigned int mfgpll;
	unsigned int cur_volt;
	unsigned int cur_freq;
	unsigned int dds;

	cur_volt = __mt_gpufreq_get_cur_gpu_volt();
	cur_freq = __mt_gpufreq_get_cur_freq();

	/*      [GPUPLL_CON1]
	 *              31              : MMPLL_SDM_PCW_CHG
	 *              26-24   : GPUPLL_POSDIV
	 *              000             : /1
	 *              001             : /2
	 *              010             : /4
	 *              011             : /8
	 *              100             : /16
	 *              21-0    : MMPLL_SDM_PCW
	 */
	post_div_order = __mt_gpufreq_get_post_div_order(freq_new, 0);
	dds = __mt_gpufreq_calculate_dds(freq_new, post_div_order);

	gpufreq_pr_debug("@%s: request GPU dds = 0x%x, cur_volt = %d, cur_freq = %d\n",
		    __func__, dds, cur_volt, cur_freq);

	mfgpll = DRV_Reg32(GPUPLL_CON1);
	gpufreq_pr_debug("@%s: begin, freq = %d, GPUPLL_CON1 = 0x%x\n", __func__, freq_new, mfgpll);

#ifndef GPU_ALWAYS_ON
	if (g_bParking) {
#else
	if (1) {
#endif
		gpufreq_pr_debug("@%s: switch to univ pll\n", __func__);
		/* Step1. Select to clk26m 26MHz */
#ifndef GPU_ALWAYS_ON
		gpu_dvfs_switch_to_univpll(true);
#endif

		DRV_WriteReg32(GPUPLL_CON1,
			       (0x80000000) | (post_div_order << POST_DIV_SHIFT) | dds);
		/* DRV_WriteReg32(GPUPLL_CON0, 0xFC000181); */
		udelay(20);

		gpufreq_pr_debug("@%s: switch back to gpu pll\n", __func__);
		/* Step3. Select back to gpupll_ck */
#ifndef GPU_ALWAYS_ON
		gpu_dvfs_switch_to_univpll(false);
#endif
		g_bParking = false;
	} else {
		/* Modify gpupll_ck */
		mt_dfs_general_pll(FH_PLL6, dds);
	}

	mfgpll = DRV_Reg32(GPUPLL_CON1);
	gpufreq_pr_debug("@%s: end, freq = %d, GPUPLL_CON1 = 0x%x\n", __func__, freq_new, mfgpll);
}

/*
 * switch voltage and vGPU via PMIC
 */
static void __mt_gpufreq_volt_switch(unsigned int volt_old, unsigned int volt_new)
{
	unsigned int max_diff;
	unsigned int delay_unit_us;
	unsigned int sfchg_rate_vgpu;

	volt_new = VOLT_NORMALISATION(volt_new);

	gpufreq_pr_debug("@%s: volt_new = %d, volt_old = %d\n", __func__, volt_new, volt_old);

	if (volt_new > volt_old) {
		/* rising rate */
		sfchg_rate_vgpu = g_vgpu_sfchg_rrate;

		/* VGPU */
		regulator_set_voltage(g_pmic->reg_vgpu, volt_new * 10, (PMIC_MAX_VGPU * 10) + 125);
		max_diff = volt_new - volt_old;
		delay_unit_us = (max_diff / DELAY_FACTOR) * 120 / 100 + 1;
		gpufreq_pr_debug("@%s: udelay us(%d) = delay_unit_us(%d) * sfchg_rate_vgpu(%d)\n",
			    __func__, delay_unit_us * sfchg_rate_vgpu, delay_unit_us,
			    sfchg_rate_vgpu);
		if (volt_old == PMIC_VGPU_LPM_VOLT)
			delay_unit_us += 20;
		udelay(delay_unit_us * sfchg_rate_vgpu);
	} else {
		/* falling rate */
		sfchg_rate_vgpu = g_vgpu_sfchg_frate;

		/* VGPU */
		regulator_set_voltage(g_pmic->reg_vgpu, volt_new * 10, (PMIC_MAX_VGPU * 10) + 125);
		max_diff = volt_old - volt_new;
		delay_unit_us = (max_diff / DELAY_FACTOR) * 120 / 100 + 1;
		gpufreq_pr_debug("@%s: udelay us(%d) = delay_unit_us(%d) * sfchg_rate_vgpu(%d)\n",
			    __func__, delay_unit_us * sfchg_rate_vgpu, delay_unit_us,
			    sfchg_rate_vgpu);
		udelay(delay_unit_us * sfchg_rate_vgpu);
	}
}

/*
 * set PWM mode for each buck
 */
static void __mt_gpufreq_buck_set_PWM_mode(unsigned int mode)
{
	/* vgpu_pmic_set_mode(mode) ?! */
	gpufreq_pr_debug("@%s: GPU_BUCK set PWM mode, mode = %d\n", __func__, mode);
}

/*
 * set fixed frequency for PROCFS: fixed_freq_volt
 */
static void __mt_gpufreq_set_fixed_freq(int fixed_freq)
{
	gpufreq_pr_debug("@%s: before, g_fixed_freq = %d, g_fixed_volt = %d\n",
		    __func__, g_fixed_freq, g_fixed_volt);
	g_fixed_freq = fixed_freq;
	g_fixed_volt = g_cur_opp_volt;
	mt_gpufreq_voltage_enable_set(1);
	gpufreq_pr_debug("@%s: now, g_fixed_freq = %d, g_fixed_volt = %d\n",
		    __func__, g_fixed_freq, g_fixed_volt);
	__mt_gpufreq_clock_switch(g_fixed_freq);
	g_cur_opp_freq = g_fixed_freq;
}

/*
 * set fixed voltage for PROCFS: fixed_freq_volt
 */
static void __mt_gpufreq_set_fixed_volt(int fixed_volt)
{
	gpufreq_pr_debug("@%s: before, g_fixed_freq = %d, g_fixed_volt = %d\n",
		    __func__, g_fixed_freq, g_fixed_volt);
	g_fixed_freq = g_cur_opp_freq;
	g_fixed_volt = fixed_volt;
	mt_gpufreq_voltage_enable_set(1);
	gpufreq_pr_debug("@%s: now, g_fixed_freq = %d, g_fixed_volt = %d\n",
		    __func__, g_fixed_freq, g_fixed_volt);
	__mt_gpufreq_volt_switch(g_cur_opp_volt, g_fixed_volt);
	g_cur_opp_volt = g_fixed_volt;
}

/*
 * dds calculation for clock switching
 */
static unsigned int __mt_gpufreq_calculate_dds(unsigned int freq_khz,
					       enum g_post_div_order_enum post_div_order)
{
	unsigned int dds = 0;

	gpufreq_pr_debug("@%s: request freq = %d, div_order = %d\n", __func__, freq_khz, post_div_order);

	/* Below freq range is Austin @ mt6775 only: dds formula is consistent with Elbrus */
	if ((freq_khz >= GPU_DVFS_FREQ15) && (freq_khz <= GPU_DVFS_FREQ0)) {
		dds = (((freq_khz / TO_MHz_HEAD * (1 << post_div_order)) << DDS_SHIFT)
		       / GPUPLL_FIN + ROUNDING_VALUE) / TO_MHz_TAIL;
	} else {
		gpufreq_pr_err("@%s: out of range, freq_khz = %d\n", __func__, freq_khz);
	}

	return dds;
}

/* power calculation for power table */
static void __mt_gpufreq_calculate_power(unsigned int idx, unsigned int freq,
					 unsigned int volt, unsigned int temp)
{
	unsigned int p_total = 0;
	unsigned int p_dynamic = 0;
	unsigned int ref_freq = 0;
	unsigned int ref_volt = 0;
	int p_leakage = 0;

	p_dynamic = GPU_ACT_REF_POWER;
	ref_freq = GPU_ACT_REF_FREQ;
	ref_volt = GPU_ACT_REF_VOLT;

	p_dynamic = p_dynamic *
	    ((freq * 100) / ref_freq) *
	    ((volt * 100) / ref_volt) * ((volt * 100) / ref_volt) / (100 * 100 * 100);

#ifdef MT_GPUFREQ_STATIC_PWR_READY2USE
	p_leakage = mt_spower_get_leakage(MTK_SPOWER_GPU, (volt / 100), temp);
	if (!g_volt_enable_state || p_leakage < 0)
		p_leakage = 0;
#else
	p_leakage = 71;
#endif				/* ifdef MT_GPUFREQ_STATIC_PWR_READY2USE */

	p_total = p_dynamic + p_leakage;

	gpufreq_pr_debug("@%s: idx = %d, p_dynamic = %d, p_leakage = %d, p_total = %d, temp = %d\n",
		    __func__, idx, p_dynamic, p_leakage, p_total, temp);

	g_power_table[idx].gpufreq_power = p_total;
}

/*
 * VGPU slew rate calculation
 * false : falling rate
 * true : rising rate
 */
static unsigned int __calculate_vgpu_sfchg_rate(bool isRising)
{
	/* unsigned int sfchg_rate_reg; */
	unsigned int sfchg_rate_vgpu = 1;

	/* [MT6763] RG_LDO_VGPU_SFCHG_RRATE and RG_LDO_VGPU_SFCHG_FRATE
	 *      4:      0.19us
	 *      8:      0.34us
	 *      11:     0.46us
	 *      17:     0.69us
	 *      23:     0.92us
	 *      25:     1us
	 */
	/*
	 *  if (isRising)
	 *  pmic_read_interface(MT6356_LDO_VGPU_CFG0, &sfchg_rate_reg,
	 *  PMIC_RG_LDO_VGPU_SFCHG_RRATE_MASK, PMIC_RG_LDO_VGPU_SFCHG_RRATE_SHIFT);
	 *  else
	 *  pmic_read_interface(MT6356_LDO_VGPU_CFG0, &sfchg_rate_reg,
	 *  PMIC_RG_LDO_VGPU_SFCHG_FRATE_MASK, PMIC_RG_LDO_VGPU_SFCHG_FRATE_SHIFT);
	 *
	 *  gpufreq_pr_debug("@%s: isRising = %d, sfchg_rate_vgpu = %d, sfchg_rate_reg = 0x%x\n",
	 *  __func__, isRising, sfchg_rate_vgpu, sfchg_rate_reg);
	 */
	return sfchg_rate_vgpu;
}

/*
 * get post divider value
 * - VCO needs proper post divider value to get corresponding dds value to adjust PLL value.
 * - e.g: In Vinson, VCO range is 2.0GHz - 4.0GHz, required frequency is 900MHz, so post
 * divider could be 2(X), 4(3600/4), 8(X), 16(X).
 * - It may have special requiremt by DE in different efuse value
 * - e.g: In Olympus, efuse value(3'b001), VCO range is 1.5GHz - 3.8GHz, required frequency
 * range is 375MHz - 900MHz, It can only use post divider 4, no post divider 2.
 */
static enum g_post_div_order_enum __mt_gpufreq_get_post_div_order(unsigned int freq,
								  unsigned int efuse)
{
	/* [Cannon]VCO range: 1.5G - 3.8GHz by div 1/2/4/8/16
	 *
	 *      PLL range: 125MHz - 3.8GHz
	 *      Version(eFuse Value)                    Info     POSTDIV                   FOUT
	 *      Free Version(efuse 3'b000)       No limit  1/2/4/8/16  125MHz - 3800MHz
	 */
	static enum g_post_div_order_enum current_order = POST_DIV8;
	enum g_post_div_order_enum post_div_order = POST_DIV8;

	if (freq > POST_DIV_8_MAX_FREQ)
		post_div_order = POST_DIV4;
	else if (freq < POST_DIV_8_MIN_FREQ)
		post_div_order = POST_DIV16;

	if (current_order != post_div_order) {
		g_bParking = true;
		current_order = post_div_order;
	}

	gpufreq_pr_debug("@%s: freq = %d, post_div_order = %d\n", __func__, freq, post_div_order);
	return post_div_order;
}

/*
 * get current frequency (KHZ)
 */
static unsigned int __mt_gpufreq_get_cur_freq(void)
{
	unsigned long mfgpll = 0;
	unsigned int post_div_order = 0;
	unsigned int freq_khz = 0;
	unsigned long dds;

	mfgpll = DRV_Reg32(GPUPLL_CON1);
	dds = mfgpll & (0x3FFFFF);

	post_div_order = (mfgpll & (0x7 << POST_DIV_SHIFT)) >> POST_DIV_SHIFT;

	freq_khz = (((dds * TO_MHz_TAIL + ROUNDING_VALUE) * GPUPLL_FIN) >> DDS_SHIFT)
	    / (1 << post_div_order) * TO_MHz_HEAD;

	gpufreq_pr_debug("@%s: mfgpll = 0x%lx, freq = %d KHz, div_order = %d\n",
		    __func__, mfgpll, freq_khz, post_div_order);

	return freq_khz;
}

/*
 * get current voltage (mV * 100)
 */
static unsigned int __mt_gpufreq_get_cur_sram_core_volt(void)
{
	unsigned int volt = 0;

	/* WARRNING: regulator_get_voltage prints uV */
	volt = regulator_get_voltage(g_pmic->reg_vsram_core) / 10;

	gpufreq_pr_debug("@%s: volt = %d\n", __func__, volt);

	return volt;
}

static unsigned int __mt_gpufreq_get_cur_gpu_volt(void)
{
	unsigned int volt = 0;

	/* WARRNING: regulator_get_voltage prints uV */
	volt = regulator_get_voltage(g_pmic->reg_vgpu) / 10;

	gpufreq_pr_debug("@%s: volt = %d\n", __func__, volt);

	return volt;
}

/*
 * get OPP table index by voltage (mV * 100)
 */
/* static int __mt_gpufreq_get_opp_idx_by_volt(unsigned int volt) */
int __mt_gpufreq_get_opp_idx_by_volt(unsigned int volt)
{
	int i = g_opp_idx_num - 1;

	while (i >= 0) {
	if (g_opp_table[i--].gpufreq_volt >= volt)
		goto EXIT;
	}

EXIT:
	return i+1;
}

/*
 * get limited frequency by limited power (mW)
 */
static unsigned int __mt_gpufreq_get_limited_freq_by_power(unsigned int limited_power)
{
	int i;
	unsigned int limited_freq;

	limited_freq = g_power_table[g_opp_idx_num - 1].gpufreq_khz;

	for (i = 0; i < g_opp_idx_num; i++) {
		if (g_power_table[i].gpufreq_power <= limited_power) {
			limited_freq = g_power_table[i].gpufreq_khz;
			break;
		}
	}

	gpufreq_pr_debug("@%s: limited_freq = %d\n", __func__, limited_freq);

	return limited_freq;
}

#ifdef MT_GPUFREQ_DYNAMIC_POWER_TABLE_UPDATE
/* update OPP power table */
static void __mt_update_gpufreqs_power_table(void)
{
	int i;
	int temp = 0;
	unsigned int freq = 0;
	unsigned int volt = 0;

	if (!g_DVFS_is_ready) {
		gpufreq_pr_debug("@%s: DVFS is not ready\n", __func__);
		return;
	}
#ifdef CONFIG_THERMAL
	temp = get_immediate_gpu_wrap() / 1000;
#else
	temp = 40;
#endif				/* ifdef CONFIG_THERMAL */

	gpufreq_pr_debug("@%s: temp = %d\n", __func__, temp);

	mutex_lock(&mt_gpufreq_lock);

	if ((temp >= -20) && (temp <= 125)) {
		for (i = 0; i < g_opp_idx_num; i++) {
			freq = g_power_table[i].gpufreq_khz;
			volt = g_power_table[i].gpufreq_volt;

			__mt_gpufreq_calculate_power(i, freq, volt, temp);

			gpufreq_pr_debug("@%s: update g_power_table[%d].gpufreq_khz = %d\n",
				    __func__, i, g_power_table[i].gpufreq_khz);
			gpufreq_pr_debug("@%s: update g_power_table[%d].gpufreq_volt = %d\n",
				    __func__, i, g_power_table[i].gpufreq_volt);
			gpufreq_pr_debug("@%s: update g_power_table[%d].gpufreq_power = %d\n",
				    __func__, i, g_power_table[i].gpufreq_power);
		}
	} else {
		gpufreq_pr_err("@%s: temp < -20 or temp > 125, NOT update power table!\n", __func__);
	}

	mutex_unlock(&mt_gpufreq_lock);
}
#endif				/* ifdef MT_GPUFREQ_DYNAMIC_POWER_TABLE_UPDATE */

/* update OPP limited index for Thermal/Power/PBM protection */
static void __mt_gpufreq_update_max_limited_idx(void)
{
	int i = 0;
	unsigned int limited_idx = 0;

	/* Check lowest frequency index in all limitation */
	for (i = 0; i < NUMBER_OF_LIMITED_IDX; i++) {
		if (g_limited_idx_array[i] > limited_idx) {
			if (!g_limited_ignore_array[i])
				limited_idx = g_limited_idx_array[i];
		}
	}

	g_max_limited_idx = limited_idx;

	gpufreq_pr_debug("@%s: g_max_limited_idx = %d\n", __func__, g_max_limited_idx);
}

#ifdef MT_GPUFREQ_BATT_OC_PROTECT
/*
 * limit OPP index for Over Currents (OC) protection
 */
static void __mt_gpufreq_batt_oc_protect(unsigned int limited_idx)
{
	mutex_lock(&mt_gpufreq_power_lock);

	gpufreq_pr_debug("@%s: limited_idx = %d\n", __func__, limited_idx);

	g_limited_idx_array[IDX_BATT_OC_LIMITED] = limited_idx;
	__mt_gpufreq_update_max_limited_idx();

	mutex_unlock(&mt_gpufreq_power_lock);
}
#endif				/* ifdef MT_GPUFREQ_BATT_OC_PROTECT */

#ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT
/*
 * limit OPP index for Battery Percentage protection
 */
static void __mt_gpufreq_batt_percent_protect(unsigned int limited_index)
{
	mutex_lock(&mt_gpufreq_power_lock);

	gpufreq_pr_debug("@%s: limited_index = %d\n", __func__, limited_index);

	g_limited_idx_array[IDX_BATT_PERCENT_LIMITED] = limited_index;
	__mt_gpufreq_update_max_limited_idx();

	mutex_unlock(&mt_gpufreq_power_lock);
}
#endif				/* ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT */

#ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT
/*
 * limit OPP index for Low Battery Volume protection
 */
static void __mt_gpufreq_low_batt_protect(unsigned int limited_index)
{
	mutex_lock(&mt_gpufreq_power_lock);

	gpufreq_pr_debug("@%s: limited_index = %d\n", __func__, limited_index);

	g_limited_idx_array[IDX_LOW_BATT_LIMITED] = limited_index;
	__mt_gpufreq_update_max_limited_idx();

	mutex_unlock(&mt_gpufreq_power_lock);
}
#endif				/* ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT */

/*
 * kick Power Budget Manager(PBM) when OPP changed
 */
static void __mt_gpufreq_kick_pbm(int enable)
{
	unsigned int power;
	unsigned int cur_freq;
	unsigned int cur_volt;
	unsigned int found = 0;
	int tmp_idx = -1;
	int i;

	cur_freq = __mt_gpufreq_get_cur_freq();
	cur_volt = __mt_gpufreq_get_cur_gpu_volt();

	if (enable) {
		for (i = 0; i < g_opp_idx_num; i++) {
			if (g_power_table[i].gpufreq_khz == cur_freq) {
				/* record idx since current voltage may not in DVFS table */
				tmp_idx = i;

				if (g_power_table[i].gpufreq_volt == cur_volt) {
					power = g_power_table[i].gpufreq_power;
					found = 1;
#ifdef MT_GPUFERQ_PBM_SUPPORT
					kicker_pbm_by_gpu(true, power, cur_volt / 100);
#endif
					gpufreq_pr_debug("@%s: request GPU power = %d, cur_volt = %d, cur_freq = %d\n"
						, __func__, power, cur_volt / 100, cur_freq);
					return;
				}
			}
		}

		if (!found) {
			gpufreq_pr_debug("@%s: tmp_idx = %d\n", __func__, tmp_idx);
			if (tmp_idx != -1 && tmp_idx < g_opp_idx_num) {
				/* use freq to found corresponding power budget */
				power = g_power_table[tmp_idx].gpufreq_power;
#ifdef MT_GPUFERQ_PBM_SUPPORT
				kicker_pbm_by_gpu(true, power, cur_volt / 100);
#endif
				gpufreq_pr_debug
				    ("@%s: request GPU power = %d, cur_volt = %d, cur_freq = %d\n",
				     __func__, power, cur_volt / 100, cur_freq);
			} else {
				gpufreq_pr_err("@%s: Cannot found request power in power table", __func__);
				gpufreq_pr_err(", cur_freq = %d KHz, cur_volt = %d mV\n"
					, cur_freq, cur_volt / 100);
			}
		}
	} else {
#ifdef MT_GPUFERQ_PBM_SUPPORT
		kicker_pbm_by_gpu(false, 0, cur_volt / 100);
#endif
	}
}

/*
 * (default) OPP table initialization
 */
static void __mt_gpufreq_setup_opp_table(struct g_opp_table_info *freqs, int num)
{
	int i = 0;

	g_opp_table = kzalloc((num) * sizeof(*freqs), GFP_KERNEL);
	g_opp_table_default = kzalloc((num) * sizeof(*freqs), GFP_KERNEL);

	if (g_opp_table == NULL || g_opp_table_default == NULL)
		return;

	for (i = 0; i < num; i++) {
		g_opp_table[i].gpufreq_khz = freqs[i].gpufreq_khz;
		g_opp_table[i].gpufreq_volt = freqs[i].gpufreq_volt;
		g_opp_table[i].gpufreq_idx = freqs[i].gpufreq_idx;

		g_opp_table_default[i].gpufreq_khz = freqs[i].gpufreq_khz;
		g_opp_table_default[i].gpufreq_volt = freqs[i].gpufreq_volt;
		g_opp_table_default[i].gpufreq_idx = freqs[i].gpufreq_idx;

		gpufreq_pr_debug("@%s: freqs[%d].gpufreq_khz = %u\n", __func__, i, freqs[i].gpufreq_khz);
		gpufreq_pr_debug("@%s: freqs[%d].gpufreq_volt = %u\n", __func__, i,
			    freqs[i].gpufreq_volt);
		gpufreq_pr_debug("@%s: freqs[%d].gpufreq_idx = %u\n", __func__, i, freqs[i].gpufreq_idx);
	}

	g_opp_idx_num = num;
	g_max_limited_idx = 0;

	__mt_gpufreq_setup_opp_power_table(num);
}

/*
 * OPP power table initialization
 */
static void __mt_gpufreq_setup_opp_power_table(int num)
{
	int i = 0;
	int temp = 0;

	g_power_table = kzalloc((num) * sizeof(struct mt_gpufreq_power_table_info), GFP_KERNEL);

	if (g_power_table == NULL)
		return;

#ifdef CONFIG_THERMAL
	temp = get_immediate_gpu_wrap() / 1000;
#else
	temp = 40;
#endif				/* ifdef CONFIG_THERMAL */

	gpufreq_pr_debug("@%s: temp = %d\n", __func__, temp);

	if ((temp < -20) || (temp > 125)) {
		gpufreq_pr_debug("@%s: temp < -20 or temp > 125!\n", __func__);
		temp = 65;
	}

	for (i = 0; i < num; i++) {
		g_power_table[i].gpufreq_khz = g_opp_table[i].gpufreq_khz;
		g_power_table[i].gpufreq_volt = g_opp_table[i].gpufreq_volt;

		__mt_gpufreq_calculate_power(i, g_power_table[i].gpufreq_khz,
					     g_power_table[i].gpufreq_volt, temp);

		gpufreq_pr_debug("@%s: g_power_table[%d].gpufreq_khz = %u\n",
			    __func__, i, g_power_table[i].gpufreq_khz);
		gpufreq_pr_debug("@%s: g_power_table[%d].gpufreq_volt = %u\n",
			    __func__, i, g_power_table[i].gpufreq_volt);
		gpufreq_pr_debug("@%s: g_power_table[%d].gpufreq_power = %u\n",
			    __func__, i, g_power_table[i].gpufreq_power);
	}

#ifdef CONFIG_THERMAL
	mtk_gpufreq_register(g_power_table, num);
#endif				/* ifdef CONFIG_THERMAL */
}

/*
 *Set default OPP index at driver probe function
 */
static void __mt_gpufreq_set_initial(void)
{
	unsigned int cur_volt = 0;
	unsigned int cur_freq = 0;

	mutex_lock(&mt_gpufreq_lock);

#ifdef MT_GPUFREQ_AEE_RR_REC
	aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() | (1 << GPU_DVFS_IS_DOING_DVFS));
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

	/* default OPP index */
	g_cur_opp_cond_idx = 0;

	gpufreq_pr_debug("@%s: initial opp index = %d\n", __func__, g_cur_opp_cond_idx);

	cur_volt = __mt_gpufreq_get_cur_gpu_volt();
	cur_freq = __mt_gpufreq_get_cur_freq();

	__mt_gpufreq_set(cur_freq, g_opp_table[g_cur_opp_cond_idx].gpufreq_khz,
			 cur_volt, g_opp_table[g_cur_opp_cond_idx].gpufreq_volt);

	g_cur_opp_freq = g_opp_table[g_cur_opp_cond_idx].gpufreq_khz;
	g_cur_opp_volt = g_opp_table[g_cur_opp_cond_idx].gpufreq_volt;
	g_cur_opp_idx = g_opp_table[g_cur_opp_cond_idx].gpufreq_idx;
	g_cur_opp_cond_idx = g_cur_opp_idx;

#ifdef MT_GPUFREQ_AEE_RR_REC
	aee_rr_rec_gpu_dvfs_oppidx(g_cur_opp_cond_idx);
	aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() & ~(1 << GPU_DVFS_IS_DOING_DVFS));
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

	mutex_unlock(&mt_gpufreq_lock);
}

/*
 * gpufreq driver probe
 */
static int __mt_gpufreq_pdrv_probe(struct platform_device *pdev)
{
	struct device_node *apmixed_node;
	struct device_node *node;
	int low_batt_limited_freq_lvl_2;
	int batt_oc_limited_freq_lvl_1;
	int ret;
	int i;
	u32 efuse_for_CT;

	gpufreq_pr_debug("@%s: gpufreq driver probe\n", __func__);

	g_debug = 0;
	gpufreq_pr_debug("@%s: clock freq = %d\n", __func__, mt_get_ckgen_freq(10));

	node = of_find_matching_node(NULL, g_gpufreq_of_match);
	if (!node)
		gpufreq_pr_err("@%s: find GPU node failed\n", __func__);

	g_clk = kzalloc(sizeof(struct g_clk_info), GFP_KERNEL);
	if (g_clk == NULL)
		return -ENOMEM;

	g_clk->clk_mux = devm_clk_get(&pdev->dev, "clk_mux");
	if (IS_ERR(g_clk->clk_mux)) {
		gpufreq_pr_err("@%s: cannot get clk_mux\n", __func__);
		return PTR_ERR(g_clk->clk_mux);
	}

	g_clk->clk_main_parent = devm_clk_get(&pdev->dev, "clk_main_parent");
	if (IS_ERR(g_clk->clk_main_parent)) {
		gpufreq_pr_err("@%s: cannot get clk_main_parent\n", __func__);
		return PTR_ERR(g_clk->clk_main_parent);
	}

	g_clk->clk_sub_parent = devm_clk_get(&pdev->dev, "clk_sub_parent");
	if (IS_ERR(g_clk->clk_sub_parent)) {
		gpufreq_pr_err("@%s: cannot get clk_sub_parent\n", __func__);
		return PTR_ERR(g_clk->clk_sub_parent);
	}

	g_clk->mtcmos_mfg_async = devm_clk_get(&pdev->dev, "mtcmos-mfg-async");
	if (IS_ERR(g_clk->mtcmos_mfg_async)) {
		gpufreq_pr_err("@%s: cannot get mtcmos-mfg-async\n", __func__);
		return PTR_ERR(g_clk->mtcmos_mfg_async);
	}

	g_clk->mtcmos_mfg = devm_clk_get(&pdev->dev, "mtcmos-mfg");
	if (IS_ERR(g_clk->mtcmos_mfg)) {
		gpufreq_pr_err("@%s: cannot get mtcmos-mfg\n", __func__);
		return PTR_ERR(g_clk->mtcmos_mfg);
	}

	g_clk->mtcmos_mfg_core0 = devm_clk_get(&pdev->dev, "mtcmos-mfg-core0");
	if (IS_ERR(g_clk->mtcmos_mfg_core0)) {
		gpufreq_pr_err("@%s: cannot get mtcmos-mfg-core0\n", __func__);
		return PTR_ERR(g_clk->mtcmos_mfg_core0);
	}

	g_clk->mtcmos_mfg_core1 = devm_clk_get(&pdev->dev, "mtcmos-mfg-core1");
	if (IS_ERR(g_clk->mtcmos_mfg_core1)) {
		gpufreq_pr_err("@%s: cannot get mtcmos-mfg-core1\n", __func__);
		return PTR_ERR(g_clk->mtcmos_mfg_core1);
	}

	g_clk->mtcmos_mfg_core2 = devm_clk_get(&pdev->dev, "mtcmos-mfg-core2");
	if (IS_ERR(g_clk->mtcmos_mfg_core2)) {
		gpufreq_pr_err("@%s: cannot get mtcmos-mfg-core2\n", __func__);
		return PTR_ERR(g_clk->mtcmos_mfg_core2);
	}

	g_clk->mtcmos_mfg_core3 = devm_clk_get(&pdev->dev, "mtcmos-mfg-core3");
	if (IS_ERR(g_clk->mtcmos_mfg_core3)) {
		gpufreq_pr_err("@%s: cannot get mtcmos-mfg-core3\n", __func__);
		return PTR_ERR(g_clk->mtcmos_mfg_core3);
	}

	g_clk->subsys_mfg_cg = devm_clk_get(&pdev->dev, "subsys-mfg-cg");
	if (IS_ERR(g_clk->subsys_mfg_cg)) {
		gpufreq_pr_err("@%s: cannot get subsys-mfg-cg\n", __func__);
		return PTR_ERR(g_clk->subsys_mfg_cg);
	}

	gpufreq_pr_debug("@%s: clk_mux is at 0x%p\n", __func__, g_clk->clk_mux);
	gpufreq_pr_debug("@%s: clk_main_parent is at 0x%p\n", __func__, g_clk->clk_main_parent);
	gpufreq_pr_debug("@%s: clk_sub_parent is at 0x%p\n", __func__, g_clk->clk_sub_parent);
	gpufreq_pr_debug("@%s: mtcmos-mfg-async is at 0x%p\n", __func__, g_clk->mtcmos_mfg_async);
	gpufreq_pr_debug("@%s: mtcmos-mfg is at 0x%p\n", __func__, g_clk->mtcmos_mfg);
	gpufreq_pr_debug("@%s: mtcmos-mfg-core0 is at 0x%p\n", __func__, g_clk->mtcmos_mfg_core0);
	gpufreq_pr_debug("@%s: mtcmos-mfg-core1 is at 0x%p\n", __func__, g_clk->mtcmos_mfg_core1);
	gpufreq_pr_debug("@%s: mtcmos-mfg-core2 is at 0x%p\n", __func__, g_clk->mtcmos_mfg_core2);
	gpufreq_pr_debug("@%s: mtcmos-mfg-core3 is at 0x%p\n", __func__, g_clk->mtcmos_mfg_core3);
	gpufreq_pr_debug("@%s: subsys-mfg-cg is at 0x%p\n", __func__, g_clk->subsys_mfg_cg);


	/* gpufreq_pr_debug("@%s: GPU is%s extra buck\n", __func__, g_extra_buck_exist ? "" : " not"); */

	/* alloc PMIC regulator */
	g_pmic = kzalloc(sizeof(struct g_pmic_info), GFP_KERNEL);
	if (g_pmic == NULL)
		return -ENOMEM;

	g_pmic->reg_vgpu = regulator_get(&pdev->dev, "vgpu");
	if (IS_ERR(g_pmic->reg_vgpu)) {
		gpufreq_pr_err("@%s: cannot get VGPU\n", __func__);
		return PTR_ERR(g_pmic->reg_vgpu);
	}
	g_pmic->reg_vsram_core = regulator_get(&pdev->dev, "vsram_core");
	if (IS_ERR(g_pmic->reg_vsram_core)) {
		gpufreq_pr_debug("@%s: cannot get VSRAM_CORE\n", __func__);
		return PTR_ERR(g_pmic->reg_vsram_core);
	}
#ifdef MT_GPUFREQ_STATIC_PWR_READY2USE
	/* Initial leackage power usage */
	mt_spower_init();
#endif				/* ifdef MT_GPUFREQ_STATIC_PWR_READY2USE */

#ifdef MT_GPUFREQ_AEE_RR_REC
	/* Initial VGPU debugging ptr */
	aee_rr_rec_gpu_dvfs_vgpu(0xFF);
	aee_rr_rec_gpu_dvfs_oppidx(0xFF);
	aee_rr_rec_gpu_dvfs_status(0xFC);
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

	/* setup OPP table */
	gpufreq_pr_debug("@%s: setup OPP table\n", __func__);

	efuse_for_CT = (get_devinfo_with_index(50) & 0x50000000) >> 28;
#ifdef GPUFREQ_SUPPORT_CT
	if (efuse_for_CT == 0x5)
		__mt_gpufreq_setup_opp_table(g_opp_table_e1_0, ARRAY_SIZE(g_opp_table_e1_0));
	else if (efuse_for_CT == 0x4)
		__mt_gpufreq_setup_opp_table(g_opp_table_e1_1, ARRAY_SIZE(g_opp_table_e1_1));
	else if (efuse_for_CT == 0x1)
		__mt_gpufreq_setup_opp_table(g_opp_table_e1_2, ARRAY_SIZE(g_opp_table_e1_2));
	else /* efuse_for_CT == 0x0 */
		__mt_gpufreq_setup_opp_table(g_opp_table_e1_3, ARRAY_SIZE(g_opp_table_e1_3));
	gpufreq_pr_debug("@%s: efuse: %u\n", __func__, efuse_for_CT);
#else
	__mt_gpufreq_setup_opp_table(g_opp_table_e1_4, ARRAY_SIZE(g_opp_table_e1_4));
#endif

	/* setup PMIC init value */
	g_vgpu_sfchg_rrate = __calculate_vgpu_sfchg_rate(true);
	g_vgpu_sfchg_frate = __calculate_vgpu_sfchg_rate(false);

	/* set VGPU */
	regulator_set_voltage(g_pmic->reg_vgpu, PMIC_MAX_VGPU * 10, PMIC_MAX_VGPU * 10 + 125);

	/* Enable VGPU */
	if (!regulator_is_enabled(g_pmic->reg_vgpu)) {
		ret = regulator_enable(g_pmic->reg_vgpu);
		if (ret) {
			gpufreq_pr_err("@%s: enable VGPU failed, ret = %d\n", __func__, ret);
			return ret;
		}
	}
	udelay(VGPU_ENABLE_TIME_US);

#ifdef MT_GPUFREQ_AEE_RR_REC
	aee_rr_rec_gpu_dvfs_status(aee_rr_curr_gpu_dvfs_status() | (1 << GPU_DVFS_IS_VGPU_ENABLED));
#endif				/* ifdef MT_GPUFREQ_AEE_RR_REC */

	gpufreq_pr_debug("@%s: VGPU is enabled = %d (%d mV)\n", __func__,
		     regulator_is_enabled(g_pmic->reg_vgpu),
		     regulator_get_voltage(g_pmic->reg_vgpu) / 1000);

	g_volt_enable_state = true;
	gpufreq_pr_debug("@%s: power init done\n", __func__);

	/* Init APMIXED base address */
	apmixed_node = of_find_compatible_node(NULL, NULL, "mediatek,apmixedsys");
	g_apmixed_base = of_iomap(apmixed_node, 0);
	if (!g_apmixed_base) {
		gpufreq_pr_err("Error, APMIXED iomap failed");
		return -ENOENT;
	}

	/* setup initial frequency */
	__mt_gpufreq_set_initial();
	gpufreq_pr_debug("@%s: current freq = %d KHz, current volt = %d mV\n",
		     __func__, __mt_gpufreq_get_cur_freq(), __mt_gpufreq_get_cur_gpu_volt() / 100);
	gpufreq_pr_debug("@%s: g_cur_opp_freq = %d, g_cur_opp_volt = %d\n",
		     __func__, g_cur_opp_freq, g_cur_opp_volt);
	gpufreq_pr_debug("@%s: g_cur_opp_idx = %d, g_cur_opp_cond_idx = %d\n",
		     __func__, g_cur_opp_idx, g_cur_opp_cond_idx);
	g_DVFS_is_ready = true;

#ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT
	low_batt_limited_freq_lvl_2 = MT_GPUFREQ_LOW_BATT_VOLT_LIMIT_FREQ_2;

	for (i = 0; i < g_opp_idx_num; i++) {
		if (g_opp_table[i].gpufreq_khz == low_batt_limited_freq_lvl_2) {
			g_low_batt_limited_idx_lvl_2 = i;
			break;
		}
	}
	register_low_battery_notify(&mt_gpufreq_low_batt_callback, LOW_BATTERY_PRIO_GPU);
#endif				/* ifdef MT_GPUFREQ_LOW_BATT_VOLT_PROTECT */

#ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT
	for (i = 0; i < g_opp_idx_num; i++) {
		if (g_opp_table[i].gpufreq_khz == MT_GPUFREQ_BATT_PERCENT_LIMIT_FREQ) {
			g_batt_percent_limited_idx_lvl_1 = i;
			break;
		}
	}
	register_battery_percent_notify(&mt_gpufreq_batt_percent_callback,
					BATTERY_PERCENT_PRIO_GPU);
#endif				/* ifdef MT_GPUFREQ_BATT_PERCENT_PROTECT */

#ifdef MT_GPUFREQ_BATT_OC_PROTECT
	batt_oc_limited_freq_lvl_1 = MT_GPUFREQ_BATT_OC_LIMIT_FREQ;

	for (i = 0; i < g_opp_idx_num; i++) {
		if (g_opp_table[i].gpufreq_khz == batt_oc_limited_freq_lvl_1) {
			g_batt_oc_limited_idx_lvl_1 = i;
			break;
		}
	}
	register_battery_oc_notify(&mt_gpufreq_batt_oc_callback, BATTERY_OC_PRIO_GPU);
#endif				/* ifdef MT_GPUFREQ_BATT_OC_PROTECT */
	return 0;
}

/*
 * register the gpufreq driver
 */
static int __init __mt_gpufreq_init(void)
{
	int ret = 0;

	gpufreq_pr_debug("@%s: start to initialize gpufreq driver\n", __func__);

#ifdef CONFIG_PROC_FS
	if (__mt_gpufreq_create_procfs())
		goto out;
#endif				/* ifdef CONFIG_PROC_FS */

	/* register platform driver */
	ret = platform_driver_register(&g_gpufreq_pdrv);
	if (ret)
		gpufreq_pr_err("@%s: fail to register gpufreq driver\n", __func__);


out:
	return ret;
}

/*
 * unregister the gpufreq driver
 */
static void __exit __mt_gpufreq_exit(void)
{
	platform_driver_unregister(&g_gpufreq_pdrv);
}

module_init(__mt_gpufreq_init);
module_exit(__mt_gpufreq_exit);

MODULE_DEVICE_TABLE(of, g_gpufreq_of_match);
MODULE_DESCRIPTION("MediaTek GPU-DVFS driver");
MODULE_LICENSE("GPL");
