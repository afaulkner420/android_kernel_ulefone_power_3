/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/ctype.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/pm_qos.h>

#include <m4u.h>
#include <ion.h>
#include <mtk/ion_drv.h>
#include <mtk/mtk_ion.h>
#include <smi_debug.h>

#ifndef MTK_VPU_FPGA_PORTING
#include <mmdvfs_mgr.h>
/*#include <mtk_pmic_info.h>*/
#endif
#include <mtk_vcorefs_manager.h>

#ifdef MTK_VPU_DVT
#define VPU_TRACE_ENABLED
#endif
/* #define BYPASS_M4U_DBG */

#include "vpu_hw.h"
#include "vpu_reg.h"
#include "vpu_cmn.h"
#include "vpu_algo.h"
#include "vpu_dbg.h"

/* #define ENABLE_PMQOS */
#ifdef ENABLE_PMQOS
#include "helio-dvfsrc-opp.h"
#endif

#include "vpu_dvfs.h"

/* opp, mW */
struct VPU_OPP_INFO vpu_power_table[VPU_OPP_NUM] = {
	{VPU_OPP_0, 336 * 4},
	{VPU_OPP_1, 250 * 4},
	{VPU_OPP_2, 221 * 4},
	{VPU_OPP_3, 208 * 4},
	{VPU_OPP_4, 140 * 4},
	{VPU_OPP_5, 120 * 4},
	{VPU_OPP_6, 114 * 4},
	{VPU_OPP_7, 84 * 4},
};


#define CMD_WAIT_TIME_MS    (3 * 1000)
#define OPP_WAIT_TIME_MS    (300)
#define PWR_KEEP_TIME_MS    (500)
#define IOMMU_VA_START      (0x7DA00000)
#define IOMMU_VA_END        (0x82600000)
#define POWER_ON_MAGIC		(2)
#define OPPTYPE_VCORE		(0)
#define OPPTYPE_DSPFREQ		(1)


/* ion & m4u */
static m4u_client_t *m4u_client;
static struct ion_client *ion_client;
static struct vpu_device *vpu_dev;
static wait_queue_head_t cmd_wait;

struct vup_service_info {
	struct task_struct *vpu_service_task;
	struct sg_table sg_reset_vector;
	struct sg_table sg_main_program;
	struct sg_table sg_algo_binary_data;
	struct sg_table sg_iram_data;
	uint64_t vpu_base;
	uint64_t bin_base;
	uint64_t iram_data_mva;
	uint64_t algo_data_mva;
	vpu_id_t current_algo;
	int thread_variable;
	struct mutex cmd_mutex;
	bool is_cmd_done;
	struct mutex state_mutex;
	enum VpuCoreState state;
	struct vpu_shared_memory *work_buf; /* working buffer */
	struct vpu_shared_memory *exec_kernel_lib; /* execution kernel library */
};
static struct vup_service_info vpu_service_cores[MTK_VPU_CORE];
struct vpu_shared_memory *core_shared_data; /* shared data for all cores */


#ifndef MTK_VPU_FPGA_PORTING

/* clock */
static struct clk *clk_top_dsp_sel;
static struct clk *clk_top_dsp1_sel;
static struct clk *clk_top_dsp2_sel;
static struct clk *clk_top_ipu_if_sel;
static struct clk *clk_ipu_core0_jtag_cg;
static struct clk *clk_ipu_core0_axi_m_cg;
static struct clk *clk_ipu_core0_ipu_cg;
static struct clk *clk_ipu_core1_jtag_cg;
static struct clk *clk_ipu_core1_axi_m_cg;
static struct clk *clk_ipu_core1_ipu_cg;
static struct clk *clk_ipu_adl_cabgen;
static struct clk *clk_ipu_conn_dap_rx_cg;
static struct clk *clk_ipu_conn_apb2axi_cg;
static struct clk *clk_ipu_conn_apb2ahb_cg;
static struct clk *clk_ipu_conn_ipu_cab1to2;
static struct clk *clk_ipu_conn_ipu1_cab1to2;
static struct clk *clk_ipu_conn_ipu2_cab1to2;
static struct clk *clk_ipu_conn_cab3to3;
static struct clk *clk_ipu_conn_cab2to1;
static struct clk *clk_ipu_conn_cab3to1_slice;
static struct clk *clk_ipu_conn_ipu_cg;
static struct clk *clk_ipu_conn_ahb_cg;
static struct clk *clk_ipu_conn_axi_cg;
static struct clk *clk_ipu_conn_isp_cg;
static struct clk *clk_ipu_conn_cam_adl_cg;
static struct clk *clk_ipu_conn_img_adl_cg;
static struct clk *clk_top_mmpll_d6;
static struct clk *clk_top_mmpll_d7;
static struct clk *clk_top_univpll_d3;
static struct clk *clk_top_syspll_d3;
static struct clk *clk_top_univpll_d2_d2;
static struct clk *clk_top_syspll_d2_d2;
static struct clk *clk_top_univpll_d3_d2;
static struct clk *clk_top_syspll_d3_d2;

/* mtcmos */
static struct clk *mtcmos_dis;
static struct clk *mtcmos_vpu_top;
/*static struct clk *mtcmos_vpu_core0_dormant;*/
static struct clk *mtcmos_vpu_core0_shutdown;
/*static struct clk *mtcmos_vpu_core1_dormant;*/
static struct clk *mtcmos_vpu_core1_shutdown;

/* smi */
static struct clk *clk_mmsys_gals_ipu2mm;
static struct clk *clk_mmsys_gals_ipu12mm;
static struct clk *clk_mmsys_gals_comm0;
static struct clk *clk_mmsys_gals_comm1;
static struct clk *clk_mmsys_smi_common;
#endif

/* workqueue */
struct my_struct_t {
	int core;
	struct delayed_work my_work;
};
static struct workqueue_struct *wq;
static void vpu_power_counter_routine(struct work_struct *);
static struct my_struct_t power_counter_work[MTK_VPU_CORE];
/*static DECLARE_DELAYED_WORK(power_counter_work, vpu_power_counter_routine);*/

/* power */
static struct mutex power_mutex[MTK_VPU_CORE];
static bool is_power_on[MTK_VPU_CORE];
static bool is_power_debug_lock;
static struct mutex power_counter_mutex[MTK_VPU_CORE];
static int power_counter[MTK_VPU_CORE];
static struct mutex opp_mutex;
static bool force_change_vcore_opp[MTK_VPU_CORE];
static bool force_change_dsp_freq[MTK_VPU_CORE];
static wait_queue_head_t waitq_change_vcore;
static wait_queue_head_t waitq_do_core_executing;
static uint8_t max_vcore_opp;
static uint8_t max_dsp_freq;


/* dvfs */
static struct vpu_dvfs_opps opps;
#ifdef ENABLE_PMQOS
static struct pm_qos_request vpu_qos_bw_request;
static struct pm_qos_request vpu_qos_vcore_request;
#endif

/* jtag */
static bool is_jtag_enabled;

/* direct link */
static bool is_locked;
static struct mutex lock_mutex;
static wait_queue_head_t lock_wait;

/* isr handler */
static irqreturn_t vpu0_isr_handler(int irq, void *dev_id);
static irqreturn_t vpu1_isr_handler(int irq, void *dev_id);

typedef irqreturn_t (*ISR_CB)(int, void *);
struct ISR_TABLE {
	ISR_CB          isr_fp;
	unsigned int    int_number;
	char            device_name[16];
};
const struct ISR_TABLE VPU_ISR_CB_TBL[MTK_VPU_CORE] = {
	{vpu0_isr_handler,     0,  "ipu1"}, /* Must be the same name with that in device node. */
	{vpu1_isr_handler,     0,  "ipu2"}
};


static inline void lock_command(int core)
{
	mutex_lock(&(vpu_service_cores[core].cmd_mutex));
	vpu_service_cores[core].is_cmd_done = false;
}

static inline int wait_command(int core)
{
	return (wait_event_interruptible_timeout(
				cmd_wait, vpu_service_cores[core].is_cmd_done, msecs_to_jiffies(CMD_WAIT_TIME_MS)) > 0)
			? 0 : -ETIMEDOUT;
}

static inline void unlock_command(int core)
{
	mutex_unlock(&(vpu_service_cores[core].cmd_mutex));
}

static inline int Map_DSP_Freq_Table(int freq_opp)
{
	int freq_value = 0;

	switch (freq_opp) {
	case 0:
	default:
		freq_value = 525;
		break;
	case 1:
		freq_value = 450;
		break;
	case 2:
		freq_value = 416;
		break;
	case 3:
		freq_value = 364;
		break;
	case 4:
		freq_value = 312;
		break;
	case 5:
		freq_value = 273;
		break;
	case 6:
		freq_value = 208;
		break;
	case 7:
		freq_value = 182;
		break;
	}

	return freq_value;
}

/*******************************************************************************
* Add MET ftrace event for power profilling.
********************************************************************************/
#if defined(VPU_MET_READY)
void MET_Events_Trace(bool enter, int core, int algo_id)
{
	int vcore_opp = 0;
	int dsp_freq = 0, ipu_if_freq = 0, dsp1_freq = 0, dsp2_freq = 0;

	if (enter) {
		/* only read for debug purpose*/
		/*mutex_lock(&opp_mutex);*/
		vcore_opp = opps.vcore.index;
		dsp_freq = Map_DSP_Freq_Table(opps.dsp.index);
		ipu_if_freq = Map_DSP_Freq_Table(opps.ipu_if.index);
		dsp1_freq = Map_DSP_Freq_Table(opps.dspcore[0].index);
		dsp2_freq = Map_DSP_Freq_Table(opps.dspcore[1].index);
		/*mutex_unlock(&opp_mutex);*/
		vpu_met_event_enter(core, algo_id, vcore_opp, dsp_freq, ipu_if_freq, dsp1_freq, dsp2_freq);
	} else {
		vpu_met_event_leave(core, algo_id);
	}
}
#endif

static inline bool vpu_other_core_idle_check(int core)
{
	int i = 0;
	bool idle = true;

	LOG_DBG("vpu vpu_other_core_idle_check+\n");
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (i == core) {
			continue;
		} else {
			LOG_DBG("vpu test %d/%d/%d\n", core, i, vpu_service_cores[i].state);
			mutex_lock(&(vpu_service_cores[i].state_mutex));
			switch (vpu_service_cores[i].state) {
			case VCT_SHUTDOWN:
			case VCT_BOOTUP:
			case VCT_IDLE:
				break;
			case VCT_EXECUTING:
			case VCT_NONE:
			case VCT_VCORE_CHG:
				idle = false;
				mutex_unlock(&(vpu_service_cores[i].state_mutex));
				goto out;
				/*break;*/
			}
			mutex_unlock(&(vpu_service_cores[i].state_mutex));
		}
	}
	LOG_DBG("vpu core_idle_check %d, %d/%d\n", idle, vpu_service_cores[0].state, vpu_service_cores[1].state);
out:
	return idle;
}

static inline int wait_to_do_change_vcore_opp(int core)
{
	int ret = 0;
	int retry = 0;

	if (g_func_mask & VFM_SKIP_WAIT_VCORE) {
		if (g_vpu_log_level > 3)
			LOG_INF("[vpu_%d_0x%x] wait change vcore return now\n", core, g_func_mask);
		return ret;
	}
#if 0
	return (wait_event_interruptible_timeout(
			waitq_change_vcore, vpu_other_core_idle_check(core), msecs_to_jiffies(OPP_WAIT_TIME_MS)) > 0)
			? 0 : -ETIMEDOUT;
#else
	do {
		ret = wait_event_interruptible_timeout(waitq_change_vcore,
			vpu_other_core_idle_check(core), msecs_to_jiffies(OPP_WAIT_TIME_MS));

		/* ret == -ERESTARTSYS, if signal interrupt */
		if (ret < 0) {
			retry += 1;
			ret = -EINTR;
			LOG_ERR("[vpu_%d/%d] interrupt by signal, while wait to change vcore, ret=%d\n",
				core, retry, ret);
		} else {
			/* normal case */
			if (ret > 0) {
				ret = 0;
			} else {
				ret = -ETIMEDOUT;
				LOG_ERR("[vpu_%d] wait_to_do_change_vcore_opp timeout, ret=%d\n", core, ret);
			}
			break;
		}
	} while (retry < 3);

	return ret;
#endif
}

static inline bool vpu_opp_change_idle_check(int core)
{
	int i = 0;
	bool idle = true;

	#if 0
	mutex_lock(&opp_mutex);
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (i == core) {
			continue;
		} else {
			if (force_change_vcore_opp[i]) {
				idle = false;
				break;
			}
		}
	}
	mutex_unlock(&opp_mutex);
	LOG_DBG("opp_idle_check %d, %d/%d\n", idle, force_change_vcore_opp[0], force_change_vcore_opp[1]);
	#else
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (i == core) {
			continue;
		} else {
			LOG_DBG("vpu test %d/%d/%d\n", core, i, vpu_service_cores[i].state);
			mutex_lock(&(vpu_service_cores[i].state_mutex));
			switch (vpu_service_cores[i].state) {
			case VCT_SHUTDOWN:
			case VCT_BOOTUP:
			case VCT_IDLE:
			case VCT_EXECUTING:
			case VCT_NONE:
				break;
			case VCT_VCORE_CHG:
				idle = false;
				mutex_unlock(&(vpu_service_cores[i].state_mutex));
				goto out;
				/*break;*/
			}
			mutex_unlock(&(vpu_service_cores[i].state_mutex));
		}
	}
	#endif
out:
	return idle;
}

static inline int wait_to_do_vpu_running(int core)
{
	int ret = 0;
	int retry = 0;
#if 0
	return (wait_event_interruptible_timeout(
				waitq_do_core_executing, vpu_opp_change_idle_check(),
				msecs_to_jiffies(OPP_WAIT_TIME_MS)) > 0)
			? 0 : -ETIMEDOUT;
#else
	do {
		ret = wait_event_interruptible_timeout(waitq_do_core_executing,
			vpu_opp_change_idle_check(core), msecs_to_jiffies(OPP_WAIT_TIME_MS));

		/* ret == -ERESTARTSYS, if signal interrupt */
		if (ret < 0) {
			retry += 1;
			ret = -EINTR;
			LOG_ERR("[vpu_%d/%d] interrupt by signal, while wait to do vpu running, ret=%d\n",
				core, retry, ret);
		} else {
			/* normal case */
			if (ret > 0) {
				ret = 0;
			} else {
				ret = -ETIMEDOUT;
				LOG_ERR("[vpu_%d] wait_to_do_vpu_running timeout, ret=%d\n", core, ret);
			}
			break;
		}
	} while (retry < 3);

	return ret;
#endif
}

static int vpu_set_clock_source(struct clk *clk, uint8_t step)
{
	struct clk *clk_src;

	LOG_DBG("vpu scc(%d)", step);
	/* set dsp frequency - 0:525MHZ, 1:450HMz, 2:416MHZ, 3:364HMz*/
	/* set dsp frequency - 4:312MHZ, 5:273HMz, 6:208MHZ, 7:182HMz*/
	switch (step) {
	case 0:
		clk_src = clk_top_mmpll_d6;
		break;
	case 1:
		clk_src = clk_top_mmpll_d7;
		break;
	case 2:
		clk_src = clk_top_univpll_d3;
		break;
	case 3:
		clk_src = clk_top_syspll_d3;
		break;
	case 4:
		clk_src = clk_top_univpll_d2_d2;
		break;
	case 5:
		clk_src = clk_top_syspll_d2_d2;
		break;
	case 6:
		clk_src = clk_top_univpll_d3_d2;
		break;
	case 7:
		clk_src = clk_top_syspll_d3_d2;
		break;
	default:
		LOG_ERR("wrong freq step(%d)", step);
		return -EINVAL;
	}

	return clk_set_parent(clk, clk_src);
}

static int vpu_get_hw_vcore_opp(int core)
{
	int opp_value = 0;
	int get_vcore_value = 0;

	get_vcore_value = (int)vcorefs_get_curr_vcore();
	if (get_vcore_value >= 800000)
		opp_value = 0;
	else
		opp_value = 1;

	LOG_DBG("[vpu_%d] vcore(%d -> %d)\n", core, get_vcore_value, opp_value);

	return opp_value;
}


/* expected range, vcore_index: 0~1 */
/* expected range, freq_index: 0~7 */
static void vpu_opp_check(int core, uint8_t vcore_index, uint8_t freq_index)
{
	int i = 0;
	bool freq_check = false;
	int log_freq = 0, log_max_freq = 0;
	int get_vcore_opp = 0;

	if (is_power_debug_lock) {
		force_change_vcore_opp[core] = false;
		force_change_dsp_freq[core] = false;
		goto out;
	}

	log_freq = Map_DSP_Freq_Table(freq_index);
	LOG_DBG("opp_check + (%d/%d/%d), ori vcore(%d)", core, vcore_index, freq_index, opps.vcore.index);

	mutex_lock(&opp_mutex);
	log_max_freq = Map_DSP_Freq_Table(max_dsp_freq);
	/* vcore opp */
	get_vcore_opp = vpu_get_hw_vcore_opp(core);
	if ((vcore_index == 0xFF) || (vcore_index == get_vcore_opp)) {
		LOG_DBG("no need, vcore opp(%d), hw vore opp(%d)\n", vcore_index, get_vcore_opp);
		force_change_vcore_opp[core] = false;
	} else {
		if (vcore_index < max_vcore_opp) {
			LOG_INF("vpu bound vcore opp(%d) to %d", vcore_index, max_vcore_opp);
			vcore_index = max_vcore_opp;
		}

		if (vcore_index >= opps.count) {
			LOG_ERR("wrong vcore opp(%d), max(%d)", vcore_index, opps.count - 1);
		} else if (vcore_index != opps.vcore.index) {
			opps.vcore.index = vcore_index;
			/*opps.index = vcore_index;*/
			force_change_vcore_opp[core] = true;
			freq_check = true;
		}
	}

	/* dsp freq opp */
	if (freq_index == 0xFF) {
		LOG_DBG("no request, freq opp(%d)", freq_index);
		force_change_dsp_freq[core] = false;
	} else {
		if (freq_index < max_dsp_freq) {
			LOG_INF("vpu bound dsp freq(%dMHz) to %dMHz", log_freq, log_max_freq);
			freq_index = max_dsp_freq;
		}

		if ((opps.dspcore[core].index != freq_index) || (freq_check)) {
			opps.dspcore[core].index = freq_index;
			if (opps.vcore.index > 0 && opps.dspcore[core].index < 4) {
				/* adjust 0~3 to 4~7 for real table if needed*/
				opps.dspcore[core].index = opps.dspcore[core].index + 4;
			}
			opps.dsp.index = 7;
			opps.ipu_if.index = 7;
			for (i = 0 ; i < MTK_VPU_CORE ; i++) {
				LOG_DBG("opp_check opps.dspcore[%d].index(%d -> %d)", core,
					opps.dspcore[core].index, opps.dsp.index);
				/* interface should be the max freq of vpu cores */
				if (opps.dspcore[i].index < opps.dsp.index) {
					opps.dsp.index = opps.dspcore[i].index;
					opps.ipu_if.index = opps.dspcore[i].index;
				}
			}
			force_change_dsp_freq[core] = true;
		}
	}
	mutex_unlock(&opp_mutex);
out:
	LOG_INF("opp_check(%d) - locked(%d), [vcore(%d/%d) %d.%d.%d.%d), check(%d/%d/%d)",
		core, is_power_debug_lock,
		opps.vcore.index, get_vcore_opp, opps.dsp.index, opps.dspcore[0].index,
		opps.dspcore[1].index, opps.ipu_if.index, freq_check,
		force_change_vcore_opp[core], force_change_dsp_freq[core]);
}

static bool vpu_change_opp(int core, int type)
{
	int ret;

	switch (type) {
	/* vcore opp */
	case OPPTYPE_VCORE:
		LOG_INF("[vpu_%d] wait for changing vcore opp", core);
		ret = wait_to_do_change_vcore_opp(core);
		CHECK_RET("[vpu_%d] ..timeout to wait_to_do_change_vcore_opp, ret=%d\n", core, ret);
		LOG_DBG("[vpu_%d] to do vcore opp change", core);
		mutex_lock(&(vpu_service_cores[core].state_mutex));
		vpu_service_cores[core].state = VCT_VCORE_CHG;
		mutex_unlock(&(vpu_service_cores[core].state_mutex));
		mutex_lock(&opp_mutex);
		vpu_trace_begin("vcore:request");
		#ifdef ENABLE_PMQOS
		switch (opps.vcore.index) {
		case 0:
			pm_qos_update_request(&vpu_qos_vcore_request, VCORE_OPP_0);
			break;
		case 1:
		default:
			pm_qos_update_request(&vpu_qos_vcore_request, VCORE_OPP_1);
			break;
		}
		#else
		ret = mmdvfs_set_fine_step(MMDVFS_SCEN_VPU_KERNEL, opps.vcore.index);
		#endif
		vpu_trace_end();
		mutex_lock(&(vpu_service_cores[core].state_mutex));
		vpu_service_cores[core].state = VCT_BOOTUP;
		mutex_unlock(&(vpu_service_cores[core].state_mutex));
		CHECK_RET("[vpu_%d]fail to request vcore, step=%d\n", core, opps.vcore.index);
		LOG_INF("[vpu_%d] cgopp vcore=%d, ddr=%d\n", core, vcorefs_get_curr_vcore(), vcorefs_get_curr_ddr());
		force_change_vcore_opp[core] = false;
		mutex_unlock(&opp_mutex);
		wake_up_interruptible(&waitq_do_core_executing);
		break;
	/* dsp freq opp */
	case OPPTYPE_DSPFREQ:
		mutex_lock(&opp_mutex);
		LOG_INF("[vpu_%d] vpu_change_opp setclksrc(%d/%d/%d/%d)\n", core, opps.dsp.index,
			opps.dspcore[0].index, opps.dspcore[1].index, opps.ipu_if.index);

		ret = vpu_set_clock_source(clk_top_dsp_sel, opps.dsp.index);
		CHECK_RET("[vpu_%d]fail to set dsp freq, step=%d, ret=%d\n", core, opps.dsp.index, ret);

		ret = vpu_set_clock_source(clk_top_dsp1_sel, opps.dspcore[core].index);
		CHECK_RET("[vpu_%d]fail to set dsp_%d freq, step=%d, ret=%d\n",
			core, core, opps.dspcore[core].index, ret);

		ret = vpu_set_clock_source(clk_top_ipu_if_sel, opps.ipu_if.index);
		CHECK_RET("[vpu_%d]fail to set ipu_if freq, step=%d, ret=%d\n", core, opps.ipu_if.index, ret);

		force_change_dsp_freq[core] = false;
		mutex_unlock(&opp_mutex);
		break;
	default:
		LOG_INF("unexpected type(%d)", type);
		break;
	}

out:
	return true;
}

int32_t vpu_thermal_en_throttle_cb(uint8_t vcore_opp, uint8_t vpu_opp)
{
	int i = 0;
	int ret = 0;
	int vcore_opp_index = 0;
	int vpu_freq_index = 0;

	#if 0
	if ((int)vpu_opp < 4) {
		vcore_opp_index = 0;
		vpu_freq_index = vpu_opp;
	} else {
		vcore_opp_index = 1;
		vpu_freq_index = vpu_opp;
	}
	#else
	if (vpu_opp < VPU_MAX_NUM_OPPS) {
		vcore_opp_index = opps.vcore.opp_map[vpu_opp];
		vpu_freq_index = opps.dsp.opp_map[vpu_opp];
	} else {
		LOG_ERR("vpu_thermal_en wrong opp(%d)\n", vpu_opp);
		return -1;
	}
	#endif
	LOG_INF("vpu_thermal_en_throttle_cb, opp(%d)->(%d/%d)\n",
		vpu_opp, vcore_opp_index, vpu_freq_index);

	for (i = 0 ; i < MTK_VPU_CORE ; i++)
		vpu_opp_check(i, vcore_opp_index, vpu_freq_index);

	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (force_change_dsp_freq[i]) {
			/* force change freq while running */
			switch (vpu_freq_index) {
			case 0:
			default:
				LOG_INF("thermal force change dsp freq to 525MHz\n");
				break;
			case 1:
				LOG_INF("thermal force change dsp freq to 450MHz\n");
				break;
			case 2:
				LOG_INF("thermal force change dsp freq to 416MHz\n");
				break;
			case 3:
				LOG_INF("thermal force change dsp freq to 364MHz\n");
				break;
			case 4:
				LOG_INF("thermal force change dsp freq to 312MHz\n");
				break;
			case 5:
				LOG_INF("thermal force change dsp freq to 273MHz\n");
				break;
			case 6:
				LOG_INF("thermal force change dsp freq to 208MHz\n");
				break;
			case 7:
				LOG_INF("thermal force change dsp freq to 182MHz\n");
				break;
			}
			mutex_lock(&opp_mutex);
			max_dsp_freq = vpu_freq_index;
			mutex_unlock(&opp_mutex);
			vpu_change_opp(i, OPPTYPE_DSPFREQ);
		} else if (force_change_vcore_opp[i]) {
			/* vcore change should wait */
			LOG_INF("thermal force change vcore opp to %d\n", vcore_opp_index);
			mutex_lock(&opp_mutex);
			max_vcore_opp = vcore_opp_index;
			mutex_unlock(&opp_mutex);
			/* vcore only need to change one time from thermal request*/
			if (i == 0)
				vpu_change_opp(i, OPPTYPE_VCORE);
		}
	}

	return ret;
}

int32_t vpu_thermal_dis_throttle_cb(void)
{
	int ret = 0;

	LOG_INF("vpu_thermal_dis_throttle_cb +\n");
	mutex_lock(&opp_mutex);
	max_vcore_opp = 0;
	max_dsp_freq = 0;
	mutex_unlock(&opp_mutex);
	LOG_INF("vpu_thermal_dis_throttle_cb -\n");

	return ret;
}

#ifdef MTK_VPU_FPGA_PORTING
#define vpu_prepare_regulator_and_clock(...)   0
#define vpu_enable_regulator_and_clock(...)    0
#define vpu_disable_regulator_and_clock(...)   0
#define vpu_unprepare_regulator_and_clock(...)
#else
static int vpu_prepare_regulator_and_clock(struct device *pdev)
{
	int ret = 0;

#define PREPARE_VPU_MTCMOS(clk) \
	{ \
		clk = devm_clk_get(pdev, #clk); \
		if (IS_ERR(clk)) { \
			ret = -ENOENT; \
			LOG_ERR("can not find mtcmos: %s\n", #clk); \
		} \
	}

	PREPARE_VPU_MTCMOS(mtcmos_dis);
	PREPARE_VPU_MTCMOS(mtcmos_vpu_top);
	PREPARE_VPU_MTCMOS(mtcmos_vpu_core0_shutdown);
	PREPARE_VPU_MTCMOS(mtcmos_vpu_core1_shutdown);
#undef PREPARE_VPU_MTCMOS

#define PREPARE_VPU_CLK(clk) \
	{ \
		clk = devm_clk_get(pdev, #clk); \
		if (IS_ERR(clk)) { \
			ret = -ENOENT; \
			LOG_ERR("can not find clock: %s\n", #clk); \
		} else if (clk_prepare(clk)) { \
			ret = -EBADE; \
			LOG_ERR("fail to prepare clock: %s\n", #clk); \
		} \
	}

	PREPARE_VPU_CLK(clk_mmsys_gals_ipu2mm);
	PREPARE_VPU_CLK(clk_mmsys_gals_ipu12mm);
	PREPARE_VPU_CLK(clk_mmsys_gals_comm0);
	PREPARE_VPU_CLK(clk_mmsys_gals_comm1);
	PREPARE_VPU_CLK(clk_mmsys_smi_common);
	PREPARE_VPU_CLK(clk_ipu_adl_cabgen);
	PREPARE_VPU_CLK(clk_ipu_conn_dap_rx_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_apb2axi_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_apb2ahb_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_ipu_cab1to2);
	PREPARE_VPU_CLK(clk_ipu_conn_ipu1_cab1to2);
	PREPARE_VPU_CLK(clk_ipu_conn_ipu2_cab1to2);
	PREPARE_VPU_CLK(clk_ipu_conn_cab3to3);
	PREPARE_VPU_CLK(clk_ipu_conn_cab2to1);
	PREPARE_VPU_CLK(clk_ipu_conn_cab3to1_slice);
	PREPARE_VPU_CLK(clk_ipu_conn_ipu_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_ahb_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_axi_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_isp_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_cam_adl_cg);
	PREPARE_VPU_CLK(clk_ipu_conn_img_adl_cg);
	PREPARE_VPU_CLK(clk_ipu_core0_jtag_cg);
	PREPARE_VPU_CLK(clk_ipu_core0_axi_m_cg);
	PREPARE_VPU_CLK(clk_ipu_core0_ipu_cg);
	PREPARE_VPU_CLK(clk_ipu_core1_jtag_cg);
	PREPARE_VPU_CLK(clk_ipu_core1_axi_m_cg);
	PREPARE_VPU_CLK(clk_ipu_core1_ipu_cg);
	PREPARE_VPU_CLK(clk_top_dsp_sel);
	PREPARE_VPU_CLK(clk_top_dsp1_sel);
	PREPARE_VPU_CLK(clk_top_dsp2_sel);
	PREPARE_VPU_CLK(clk_top_ipu_if_sel);
	PREPARE_VPU_CLK(clk_top_mmpll_d6);
	PREPARE_VPU_CLK(clk_top_mmpll_d7);
	PREPARE_VPU_CLK(clk_top_univpll_d3);
	PREPARE_VPU_CLK(clk_top_syspll_d3);
	PREPARE_VPU_CLK(clk_top_univpll_d2_d2);
	PREPARE_VPU_CLK(clk_top_syspll_d2_d2);
	PREPARE_VPU_CLK(clk_top_univpll_d3_d2);
	PREPARE_VPU_CLK(clk_top_syspll_d3_d2);
#undef PREPARE_VPU_CLK

	return ret;
}
static int vpu_enable_regulator_and_clock(int core)
{
	int ret = 0;
	int get_vcore_opp = 0;
	bool adjust_vcore = false;

	LOG_INF("[vpu_%d] en_rc + (%d)\n", core, is_power_debug_lock);
	vpu_trace_begin("vpu_enable_regulator_and_clock");

	if (is_power_debug_lock)
		goto clk_on;

	get_vcore_opp = vpu_get_hw_vcore_opp(core);
	if (opps.vcore.index != get_vcore_opp)
		adjust_vcore = true;

	vpu_trace_begin("vcore:request");
	if (adjust_vcore) {
		LOG_DBG("[vpu_%d] en_rc wait for changing vcore opp", core);
		ret = wait_to_do_change_vcore_opp(core);
		if (ret) {
			/* skip change vcore in these time */
			LOG_WRN("[vpu_%d] .timeout to wait_to_do_change_vcore_opp(%d/%d), ret=%d\n",
				core, opps.vcore.index, get_vcore_opp, ret);
			ret = 0;
			goto clk_on;
		}
		LOG_DBG("[vpu_%d] en_rc to do vcore opp change", core);
#ifdef ENABLE_PMQOS
		switch (opps.vcore.index) {
		case 0:
			pm_qos_update_request(&vpu_qos_vcore_request, VCORE_OPP_0);
			break;
		case 1:
		default:
			pm_qos_update_request(&vpu_qos_vcore_request, VCORE_OPP_1);
			break;
		}
#else
		ret = mmdvfs_set_fine_step(MMDVFS_SCEN_VPU_KERNEL, opps.vcore.index);
#endif
	}
	vpu_trace_end();
	CHECK_RET("[vpu_%d]fail to request vcore, step=%d\n", core, opps.vcore.index);
	LOG_INF("[vpu_%d] adjust(%d,%d) result vcore=%d, ddr=%d\n", core,
		adjust_vcore, opps.vcore.index, vcorefs_get_curr_vcore(), vcorefs_get_curr_ddr());
	LOG_DBG("[vpu_%d] en_rc setmmdvfs(%d) done\n", core, opps.vcore.index);

clk_on:
#define ENABLE_VPU_MTCMOS(clk) \
	{ \
		if (clk != NULL) { \
			if (clk_prepare_enable(clk)) \
				LOG_ERR("fail to prepare and enable mtcmos: %s\n", #clk); \
		} else { \
			LOG_WRN("mtcmos not existed: %s\n", #clk); \
		} \
	}

#define ENABLE_VPU_CLK(clk) \
	{ \
		if (clk != NULL) { \
			if (clk_enable(clk)) \
				LOG_ERR("fail to enable clock: %s\n", #clk); \
		} else { \
			LOG_WRN("clk not existed: %s\n", #clk); \
		} \
	}

	vpu_trace_begin("clock:enable_source");
	ENABLE_VPU_CLK(clk_top_dsp_sel);
	ENABLE_VPU_CLK(clk_top_ipu_if_sel);
	ENABLE_VPU_CLK(clk_top_dsp1_sel);
	ENABLE_VPU_CLK(clk_top_dsp2_sel);
	vpu_trace_end();

	vpu_trace_begin("mtcmos:enable");
	ENABLE_VPU_MTCMOS(mtcmos_dis);
	ENABLE_VPU_MTCMOS(mtcmos_vpu_top);
	ENABLE_VPU_MTCMOS(mtcmos_vpu_core0_shutdown);
	ENABLE_VPU_MTCMOS(mtcmos_vpu_core1_shutdown);
	vpu_trace_end();

	vpu_trace_begin("clock:enable");
	ENABLE_VPU_CLK(clk_mmsys_gals_ipu2mm);
	ENABLE_VPU_CLK(clk_mmsys_gals_ipu12mm);
	ENABLE_VPU_CLK(clk_mmsys_gals_comm0);
	ENABLE_VPU_CLK(clk_mmsys_gals_comm1);
	ENABLE_VPU_CLK(clk_mmsys_smi_common);
	ENABLE_VPU_CLK(clk_ipu_adl_cabgen);
	ENABLE_VPU_CLK(clk_ipu_conn_dap_rx_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_apb2axi_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_apb2ahb_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_ipu_cab1to2);
	ENABLE_VPU_CLK(clk_ipu_conn_ipu1_cab1to2);
	ENABLE_VPU_CLK(clk_ipu_conn_ipu2_cab1to2);
	ENABLE_VPU_CLK(clk_ipu_conn_cab3to3);
	ENABLE_VPU_CLK(clk_ipu_conn_cab2to1);
	ENABLE_VPU_CLK(clk_ipu_conn_cab3to1_slice);
	ENABLE_VPU_CLK(clk_ipu_conn_ipu_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_ahb_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_axi_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_isp_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_cam_adl_cg);
	ENABLE_VPU_CLK(clk_ipu_conn_img_adl_cg);
	switch (core) {
	case 0:
	default:
		ENABLE_VPU_CLK(clk_ipu_core0_jtag_cg);
		ENABLE_VPU_CLK(clk_ipu_core0_axi_m_cg);
		ENABLE_VPU_CLK(clk_ipu_core0_ipu_cg);
		break;
	case 1:
		ENABLE_VPU_CLK(clk_ipu_core1_jtag_cg);
		ENABLE_VPU_CLK(clk_ipu_core1_axi_m_cg);
		ENABLE_VPU_CLK(clk_ipu_core1_ipu_cg);
		break;
	}
	vpu_trace_end();

#undef ENABLE_VPU_MTCMOS
#undef ENABLE_VPU_CLK
#if 1
	LOG_INF("[vpu_%d] en_rc setclksrc(%d/%d/%d/%d)\n", core, opps.dsp.index,
		opps.dspcore[0].index, opps.dspcore[1].index, opps.ipu_if.index);

	ret = vpu_set_clock_source(clk_top_dsp_sel, opps.dsp.index);
	CHECK_RET("[vpu_%d]fail to set dsp freq, step=%d, ret=%d\n", core, opps.dsp.index, ret);

	ret = vpu_set_clock_source(clk_top_dsp1_sel, opps.dspcore[0].index);
	CHECK_RET("[vpu_%d]fail to set dsp0 freq, step=%d, ret=%d\n", core, opps.dspcore[0].index, ret);

	ret = vpu_set_clock_source(clk_top_dsp2_sel, opps.dspcore[1].index);
	CHECK_RET("[vpu_%d]fail to set dsp1 freq, step=%d, ret=%d\n", core, opps.dspcore[1].index, ret);

	ret = vpu_set_clock_source(clk_top_ipu_if_sel, opps.ipu_if.index);
	CHECK_RET("[vpu_%d]fail to set ipu_if freq, step=%d, ret=%d\n", core, opps.ipu_if.index, ret);

out:
#endif
	vpu_trace_end();
	is_power_on[core] = true;
	force_change_vcore_opp[core] = false;
	force_change_dsp_freq[core] = false;
	LOG_DBG("[vpu_%d] en_rc -\n", core);
	return ret;
}

static unsigned int vpu_read_smi_bus_debug(int core)
{
	unsigned int smi_bus_value = 0x0;
	unsigned int smi_bus_vpu_value = 0x0;

#ifdef MTK_VPU_SMI_DEBUG_ON
	if ((int)(vpu_dev->smi_common_base) != 0) {
		switch (core) {
		case 0:
		default:
			smi_bus_value = vpu_read_reg32(vpu_dev->smi_common_base, 0x414);
			break;
		case 1:
			smi_bus_value = vpu_read_reg32(vpu_dev->smi_common_base, 0x418);
			break;
		}
		smi_bus_vpu_value = (smi_bus_value & 0x007FE000) >> 13;
	} else {
		LOG_INF("[vpu_%d] null smi_common_base\n", core);
	}
#endif
	LOG_INF("[vpu_%d] read_smi_bus (0x%x/0x%x)\n", core, smi_bus_value, smi_bus_vpu_value);

	return smi_bus_vpu_value;
}

static int vpu_disable_regulator_and_clock(int core)
{
	int ret = 0;

	unsigned int smi_bus_vpu_value = 0x0;

	/* check there is un-finished transaction in bus before turning off vpu power */
#ifdef MTK_VPU_SMI_DEBUG_ON
	smi_bus_vpu_value = vpu_read_smi_bus_debug(core);
	LOG_INF("[vpu_%d] dis_rc 1 (0x%x)\n", core, smi_bus_vpu_value);
	if ((int)smi_bus_vpu_value != 0) {
		mdelay(1);
		smi_bus_vpu_value = vpu_read_smi_bus_debug(core);
		LOG_INF("[vpu_%d] dis_rc again (0x%x)\n", core, smi_bus_vpu_value);
		if ((int)smi_bus_vpu_value != 0) {
			smi_debug_bus_hanging_detect_ext2(0x1ff, 1, 0, 1);
			vpu_aee_warn("VPU SMI CHECK", "core_%d fail to check smi, value=%d\n", core,
				smi_bus_vpu_value);
		}
	}
#else
	LOG_INF("[vpu_%d] dis_rc + (0x%x)\n", core, smi_bus_vpu_value);
#endif

#define DISABLE_VPU_CLK(clk) \
	{ \
		if (clk != NULL) { \
			clk_disable(clk); \
		} else { \
			LOG_WRN("clk not existed: %s\n", #clk); \
		} \
	}

	switch (core) {
	case 0:
	default:
		DISABLE_VPU_CLK(clk_ipu_core0_jtag_cg);
		DISABLE_VPU_CLK(clk_ipu_core0_axi_m_cg);
		DISABLE_VPU_CLK(clk_ipu_core0_ipu_cg);
		break;
	case 1:
		DISABLE_VPU_CLK(clk_ipu_core1_jtag_cg);
		DISABLE_VPU_CLK(clk_ipu_core1_axi_m_cg);
		DISABLE_VPU_CLK(clk_ipu_core1_ipu_cg);
		break;
	}
	DISABLE_VPU_CLK(clk_ipu_adl_cabgen);
	DISABLE_VPU_CLK(clk_ipu_conn_dap_rx_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_apb2axi_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_apb2ahb_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_ipu_cab1to2);
	DISABLE_VPU_CLK(clk_ipu_conn_ipu1_cab1to2);
	DISABLE_VPU_CLK(clk_ipu_conn_ipu2_cab1to2);
	DISABLE_VPU_CLK(clk_ipu_conn_cab3to3);
	DISABLE_VPU_CLK(clk_ipu_conn_cab2to1);
	DISABLE_VPU_CLK(clk_ipu_conn_cab3to1_slice);
	DISABLE_VPU_CLK(clk_ipu_conn_ipu_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_ahb_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_axi_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_isp_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_cam_adl_cg);
	DISABLE_VPU_CLK(clk_ipu_conn_img_adl_cg);
	DISABLE_VPU_CLK(clk_mmsys_gals_ipu2mm);
	DISABLE_VPU_CLK(clk_mmsys_gals_ipu12mm);
	DISABLE_VPU_CLK(clk_mmsys_gals_comm0);
	DISABLE_VPU_CLK(clk_mmsys_gals_comm1);
	DISABLE_VPU_CLK(clk_mmsys_smi_common);
	LOG_DBG("[vpu_%d] dis_rc flag4\n", core);

#define DISABLE_VPU_MTCMOS(clk) \
	{ \
		if (clk != NULL) { \
			clk_disable_unprepare(clk); \
		} else { \
			LOG_WRN("mtcmos not existed: %s\n", #clk); \
		} \
	}

	DISABLE_VPU_MTCMOS(mtcmos_vpu_core1_shutdown);
	DISABLE_VPU_MTCMOS(mtcmos_vpu_core0_shutdown);
	DISABLE_VPU_MTCMOS(mtcmos_vpu_top);
	DISABLE_VPU_MTCMOS(mtcmos_dis);

	DISABLE_VPU_CLK(clk_top_dsp_sel);
	DISABLE_VPU_CLK(clk_top_ipu_if_sel);
	DISABLE_VPU_CLK(clk_top_dsp1_sel);
	DISABLE_VPU_CLK(clk_top_dsp2_sel);

#undef DISABLE_VPU_MTCMOS
#undef DISABLE_VPU_CLK
#ifdef ENABLE_PMQOS
	pm_qos_update_request(&vpu_qos_vcore_request, VCORE_OPP_UNREQ);
#else
	ret = mmdvfs_set_fine_step(MMDVFS_SCEN_VPU_KERNEL, MMDVFS_FINE_STEP_UNREQUEST);
#endif
	CHECK_RET("[vpu_%d]fail to unrequest vcore!\n", core);
	LOG_DBG("[vpu_%d] disable result vcore=%d, ddr=%d\n", core, vcorefs_get_curr_vcore(), vcorefs_get_curr_ddr());
out:
	is_power_on[core] = false;
	LOG_INF("[vpu_%d] dis_rc -\n", core);
	return ret;

}

static void vpu_unprepare_regulator_and_clock(void)
{

#define UNPREPARE_VPU_CLK(clk) \
	{ \
		if (clk != NULL) { \
			clk_unprepare(clk); \
			clk = NULL; \
		} \
	}
	UNPREPARE_VPU_CLK(clk_ipu_core0_jtag_cg);
	UNPREPARE_VPU_CLK(clk_ipu_core0_axi_m_cg);
	UNPREPARE_VPU_CLK(clk_ipu_core0_ipu_cg);
	UNPREPARE_VPU_CLK(clk_ipu_core1_jtag_cg);
	UNPREPARE_VPU_CLK(clk_ipu_core1_axi_m_cg);
	UNPREPARE_VPU_CLK(clk_ipu_core1_ipu_cg);
	UNPREPARE_VPU_CLK(clk_ipu_adl_cabgen);
	UNPREPARE_VPU_CLK(clk_ipu_conn_dap_rx_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_apb2axi_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_apb2ahb_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_ipu_cab1to2);
	UNPREPARE_VPU_CLK(clk_ipu_conn_ipu1_cab1to2);
	UNPREPARE_VPU_CLK(clk_ipu_conn_ipu2_cab1to2);
	UNPREPARE_VPU_CLK(clk_ipu_conn_cab3to3);
	UNPREPARE_VPU_CLK(clk_ipu_conn_cab2to1);
	UNPREPARE_VPU_CLK(clk_ipu_conn_cab3to1_slice);
	UNPREPARE_VPU_CLK(clk_ipu_conn_ipu_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_ahb_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_axi_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_isp_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_cam_adl_cg);
	UNPREPARE_VPU_CLK(clk_ipu_conn_img_adl_cg);
	UNPREPARE_VPU_CLK(clk_mmsys_gals_ipu2mm);
	UNPREPARE_VPU_CLK(clk_mmsys_gals_ipu12mm);
	UNPREPARE_VPU_CLK(clk_mmsys_gals_comm0);
	UNPREPARE_VPU_CLK(clk_mmsys_gals_comm1);
	UNPREPARE_VPU_CLK(clk_mmsys_smi_common);
	UNPREPARE_VPU_CLK(clk_top_dsp_sel);
	UNPREPARE_VPU_CLK(clk_top_dsp1_sel);
	UNPREPARE_VPU_CLK(clk_top_dsp2_sel);
	UNPREPARE_VPU_CLK(clk_top_ipu_if_sel);
	UNPREPARE_VPU_CLK(clk_top_mmpll_d6);
	UNPREPARE_VPU_CLK(clk_top_mmpll_d7);
	UNPREPARE_VPU_CLK(clk_top_univpll_d3);
	UNPREPARE_VPU_CLK(clk_top_syspll_d3);
	UNPREPARE_VPU_CLK(clk_top_univpll_d2_d2);
	UNPREPARE_VPU_CLK(clk_top_syspll_d2_d2);
	UNPREPARE_VPU_CLK(clk_top_univpll_d3_d2);
	UNPREPARE_VPU_CLK(clk_top_syspll_d3_d2);
#undef UNPREPARE_VPU_CLK

}
#endif

irqreturn_t vpu0_isr_handler(int irq, void *dev_id)
{
	LOG_DBG("vpu 0 received a interrupt\n");
	vpu_service_cores[0].is_cmd_done = true;
	wake_up_interruptible(&cmd_wait);
	vpu_write_field(0, FLD_APMCU_INT, 1);                   /* clear int */

	return IRQ_HANDLED;
}
irqreturn_t vpu1_isr_handler(int irq, void *dev_id)
{
	LOG_DBG("vpu 1 received a interrupt\n");
	vpu_service_cores[1].is_cmd_done = true;
	wake_up_interruptible(&cmd_wait);
	vpu_write_field(1, FLD_APMCU_INT, 1);                   /* clear int */

	return IRQ_HANDLED;
}

static bool service_pool_is_empty(int core)
{
	bool is_empty = true;

	mutex_lock(&vpu_dev->servicepool_mutex[core]);
	if (!list_empty(&vpu_dev->servicepool_list[core]))
		is_empty = false;

	mutex_unlock(&vpu_dev->servicepool_mutex[core]);

	return is_empty;
}
static bool common_pool_is_empty(void)
{
	bool is_empty = true;

	mutex_lock(&vpu_dev->commonpool_mutex);
	if (!list_empty(&vpu_dev->commonpool_list))
		is_empty = false;

	mutex_unlock(&vpu_dev->commonpool_mutex);

	return is_empty;
}

static int vpu_service_routine(void *arg)
{
	struct vpu_user *user = NULL;
	struct vpu_request *req = NULL;
	/*struct vpu_algo *algo = NULL;*/
	uint8_t vcore_opp_index = 0xFF;
	uint8_t dsp_freq_index = 0xFF;
	struct vpu_user *user_in_list = NULL;
	struct list_head *head = NULL;
	int *d = (int *)arg;
	int service_core = (*d);
	bool get = false;

	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	for (; !kthread_should_stop();) {
		/* wait for requests if there is no one in user's queue */
		add_wait_queue(&vpu_dev->req_wait, &wait);
		while (1) {
			if ((!service_pool_is_empty(service_core)) || (!common_pool_is_empty()))
				break;
			wait_woken(&wait, TASK_INTERRUPTIBLE, MAX_SCHEDULE_TIMEOUT);
		}
		remove_wait_queue(&vpu_dev->req_wait, &wait);

		/* this thread will be stopped if start direct link */
		/* todo, no need in this currently */
		/*wait_event_interruptible(lock_wait, !is_locked);*/

		/* consume the user's queue */
		req = NULL;
		user_in_list = NULL;
		head = NULL;
		get = false;

		mutex_lock(&vpu_dev->servicepool_mutex[service_core]);
		if (!(list_empty(&vpu_dev->servicepool_list[service_core]))) {
			req = vlist_node_of(vpu_dev->servicepool_list[service_core].next, struct vpu_request);
			list_del_init(vlist_link(req, struct vpu_request));
			vpu_dev->servicepool_list_size[service_core] -= 1;
			LOG_DBG("[vpu] flag - : selfpool(%d)_size(%d)\n", service_core,
				vpu_dev->servicepool_list_size[service_core]);
			mutex_unlock(&vpu_dev->servicepool_mutex[service_core]);
			LOG_DBG("[vpu] flag - 2: get selfpool\n");
		} else {
			mutex_unlock(&vpu_dev->servicepool_mutex[service_core]);

			mutex_lock(&vpu_dev->commonpool_mutex);
			if (!(list_empty(&vpu_dev->commonpool_list))) {
				req = vlist_node_of(vpu_dev->commonpool_list.next, struct vpu_request);
				list_del_init(vlist_link(req, struct vpu_request));
				vpu_dev->commonpool_list_size -= 1;
				LOG_DBG("[vpu] flag - : common pool_size(%d)\n", vpu_dev->commonpool_list_size);
				LOG_DBG("[vpu] flag - 3: get common pool\n");
			}
			mutex_unlock(&vpu_dev->commonpool_mutex);
		}
		/* suppose that req is null would not happen */
		/* due to we check service_pool_is_empty and common_pool_is_empty */
		if (req != NULL) {
			LOG_DBG("[vpu] service core index...: %d/%d", service_core, (*d));
			user = (struct vpu_user *)req->user_id;
			LOG_DBG("[vpu_%d] user...0x%lx/0x%lx/0x%lx/0x%lx\n", service_core,
				(unsigned long)user, (unsigned long)&user,
				(unsigned long)req->user_id, (unsigned long)&(req->user_id));
			mutex_lock(&vpu_dev->user_mutex);
			/* check to avoid user had been removed from list, and kernel vpu thread still do the request */
			list_for_each(head, &vpu_dev->user_list)
			{
				user_in_list = vlist_node_of(head, struct vpu_user);
				LOG_DBG("[vpu_%d] user->id = 0x%lx, 0x%lx\n", service_core,
					(unsigned long)(user_in_list->id), (unsigned long)(user));
				if ((unsigned long)(user_in_list->id) == (unsigned long)(user)) {
					get = true;
					LOG_DBG("[vpu_%d] get_0x%lx = true\n", service_core, (unsigned long)(user));
					break;
				}
			}
			if (!get) {
				mutex_unlock(&vpu_dev->user_mutex);
				LOG_WRN("[vpu_%d] get request that the original user(0x%lx) is deleted\n",
					service_core, (unsigned long)(user));
				continue;
			}

			user->running = true; /* for flush request from queue, DL usage */
			/* unlock for avoiding long time locking */
			mutex_unlock(&vpu_dev->user_mutex);
			#if 1
			if (req->power_param.opp_step == 0xFF) {
				vcore_opp_index = 0xFF;
				dsp_freq_index = 0xFF;
			} else {
				vcore_opp_index = opps.vcore.opp_map[req->power_param.opp_step];
				dsp_freq_index = opps.dspcore[service_core].opp_map[req->power_param.opp_step];
			}
			#else
			vcore_opp_index = req->power_param.opp_step;
			dsp_freq_index = req->power_param.freq_step;
			#endif
			LOG_DBG("[vpu_%d] run, opp(%d/%d/%d)\n", service_core, req->power_param.opp_step,
				vcore_opp_index, dsp_freq_index);
			vpu_opp_check(service_core, vcore_opp_index, dsp_freq_index);
			LOG_INF("[vpu_%d<-0x%x]run,algoid(0x%lx_%d,%d->%d),op(%d,%d/%d->%d,%d->%d),f(0x%x),%d/%d/%d\n",
				service_core, req->requested_core,
				(unsigned long)req->request_id, req->frame_magic,
				vpu_service_cores[service_core].current_algo, (int)(req->algo_id[service_core]),
				req->power_param.opp_step, req->power_param.freq_step,
				vcore_opp_index, opps.vcore.index,
				dsp_freq_index, opps.dspcore[service_core].index, g_func_mask,
				vpu_dev->servicepool_list_size[service_core],
				vpu_dev->commonpool_list_size, is_locked);
			#if 0
			/*  prevent the worker shutdown vpu first, and current enque use the same algo_id */
			/*ret = wait_to_do_vpu_running(service_core);*/
			/*CHECK_RET("[vpu_%d] fail to wait_to_do_vpu_running!, ret = %d\n", service_core, ret);*/
			/*mutex_lock(&(vpu_service_cores[service_core].state_mutex));*/
			/*vpu_service_cores[service_core].state = VCT_PWRON;*/
			if (req->algo_id[service_core] != vpu_service_cores[service_core].current_algo) {
				/*mutex_unlock(&(vpu_service_cores[service_core].state_mutex));*/

				if (vpu_find_algo_by_id(service_core, req->algo_id[service_core], &algo)) {
					req->status = VPU_REQ_STATUS_INVALID;
						LOG_ERR("can not find the algo, id=%d\n", req->algo_id[service_core]);
						goto out;
				}
				if (vpu_hw_load_algo(service_core, algo)) {
					LOG_ERR("load algo failed, while enque\n");
					req->status = VPU_REQ_STATUS_FAILURE;
					goto out;
				}
			} else {
				/*mutex_unlock(&(vpu_service_cores[service_core].state_mutex));*/
			}
			LOG_DBG("[vpu] flag - 4: hw_enque_request\n");
			vpu_hw_enque_request(service_core, req);
			#else
			if (vpu_hw_processing_request(service_core, req)) {
				LOG_ERR("[vpu_%d] hw_processing_request failed, while enque\n", service_core);
				req->status = VPU_REQ_STATUS_FAILURE;
				goto out;
			}
			#endif
			LOG_DBG("[vpu] flag - 5: hw enque_request done\n");
		} else {
			/* consider that only one req in common pool and all services get pass through */
			/* do nothing if the service do not get the request */
			LOG_WRN("[vpu_%d] get null request, %d/%d/%d\n", service_core,
				vpu_dev->servicepool_list_size[service_core],
				vpu_dev->commonpool_list_size, is_locked);
			continue;
		}
out:
		/* if req is null, we should not do anything of following codes */
		mutex_lock(&(vpu_service_cores[service_core].state_mutex));
		vpu_service_cores[service_core].state = VCT_IDLE;
		mutex_unlock(&(vpu_service_cores[service_core].state_mutex));
		mutex_lock(&vpu_dev->user_mutex);
		LOG_DBG("[vpu] flag - 5.5 : ....\n");
		/* check to avoid user had been removed from list, and kernel vpu thread finish the task */
		get = false;
		head = NULL;
		user_in_list = NULL;
		list_for_each(head, &vpu_dev->user_list)
		{
			user_in_list = vlist_node_of(head, struct vpu_user);
			if ((unsigned long)(user_in_list->id) == (unsigned long)(user)) {
				get = true;
				break;
			}
		}
		if (get) {
			mutex_lock(&user->data_mutex);
			LOG_DBG("[vpu] flag - 6: add to deque list\n");
			req->occupied_core = (0x1 << service_core);
			list_add_tail(vlist_link(req, struct vpu_request), &user->deque_list);
			LOG_INF("[vpu_%d, 0x%x -> 0x%x] algo_id(%d_%d) add to deque list done\n",
				service_core, req->requested_core, req->occupied_core,
				(int)(req->algo_id[service_core]), req->frame_magic);
			user->running = false;
			mutex_unlock(&user->data_mutex);
			wake_up_interruptible_all(&user->deque_wait);
		} else {
			LOG_WRN("[vpu_%d]done request that the original user(0x%lx) is deleted\n",
				service_core, (unsigned long)(user));
		}
		mutex_unlock(&vpu_dev->user_mutex);
		wake_up_interruptible(&waitq_change_vcore);
		/* leave loop of round-robin */
		if (is_locked)
			break;
		/* release cpu for another operations */
		usleep_range(1, 10);
	}
	return 0;
}


#ifndef MTK_VPU_EMULATOR

static int vpu_map_mva_of_bin(int core, uint64_t bin_pa)
{
	int ret = 0;

#ifndef BYPASS_M4U_DBG
	uint32_t mva_reset_vector;
	uint32_t mva_main_program;
	uint32_t mva_algo_binary_data;
	uint32_t mva_iram_data;
	uint64_t binpa_reset_vector;
	uint64_t binpa_main_program;
	uint64_t binpa_iram_data;
	struct sg_table *sg;
	const uint64_t size_algos = VPU_SIZE_ALGO_AREA + VPU_SIZE_ALGO_AREA + VPU_SIZE_ALGO_AREA;

	LOG_DBG("vpu_map_mva_of_bin, bin_pa(0x%lx)\n", (unsigned long)bin_pa);

	switch (core) {
	case 0:
	default:
		mva_reset_vector = VPU_MVA_RESET_VECTOR;
		mva_main_program = VPU_MVA_MAIN_PROGRAM;
		binpa_reset_vector = bin_pa;
		binpa_main_program = bin_pa + VPU_OFFSET_MAIN_PROGRAM;
		binpa_iram_data = bin_pa + VPU_OFFSET_MAIN_PROGRAM_IMEM;
		break;
	case 1:
		mva_reset_vector = VPU2_MVA_RESET_VECTOR;
		mva_main_program = VPU2_MVA_MAIN_PROGRAM;
		binpa_reset_vector = bin_pa + VPU_DDR_SHIFT_RESET_VECTOR;
		binpa_main_program = binpa_reset_vector + VPU_OFFSET_MAIN_PROGRAM;
		binpa_iram_data = bin_pa + VPU_OFFSET_MAIN_PROGRAM_IMEM + VPU_DDR_SHIFT_IRAM_DATA;
		break;
	}
	LOG_DBG("vpu_map_mva_of_bin(core:%d), pa resvec/mainpro(0x%lx/0x%lx)\n", core,
		(unsigned long)binpa_reset_vector, (unsigned long)binpa_main_program);

	/* 1. map reset vector */
	sg = &(vpu_service_cores[core].sg_reset_vector);
	ret = sg_alloc_table(sg, 1, GFP_KERNEL);
	CHECK_RET("[vpu_%d]fail to allocate sg table[reset]!\n", core);
	LOG_DBG("vpu...sg_alloc_table ok\n");

	sg_dma_address(sg->sgl) = binpa_reset_vector;
	LOG_DBG("vpu...sg_dma_address ok, bin_pa(0x%x)\n", (unsigned int)binpa_reset_vector);
	sg_dma_len(sg->sgl) = VPU_SIZE_RESET_VECTOR;
	LOG_DBG("vpu...sg_dma_len ok, VPU_SIZE_RESET_VECTOR(0x%x)\n", VPU_SIZE_RESET_VECTOR);
	ret = m4u_alloc_mva(m4u_client, VPU_PORT_OF_IOMMU,
			0, sg,
			VPU_SIZE_RESET_VECTOR,
			M4U_PROT_READ | M4U_PROT_WRITE,
			M4U_FLAGS_START_FROM/*M4U_FLAGS_FIX_MVA*/, &mva_reset_vector);
	CHECK_RET("[vpu_%d]fail to allocate mva of reset vecter!\n", core);
	LOG_DBG("vpu...m4u_alloc_mva ok\n");

	/* 2. map main program */
	sg = &(vpu_service_cores[core].sg_main_program);
	ret = sg_alloc_table(sg, 1, GFP_KERNEL);
	CHECK_RET("[vpu_%d]fail to allocate sg table[main]!\n", core);
	LOG_DBG("vpu...sg_alloc_table main_program ok\n");

	sg_dma_address(sg->sgl) = binpa_main_program;
	sg_dma_len(sg->sgl) = VPU_SIZE_MAIN_PROGRAM;
	ret = m4u_alloc_mva(m4u_client, VPU_PORT_OF_IOMMU,
			0, sg,
			VPU_SIZE_MAIN_PROGRAM,
			M4U_PROT_READ | M4U_PROT_WRITE,
			M4U_FLAGS_START_FROM/*M4U_FLAGS_FIX_MVA*/, &mva_main_program);
	if (ret) {
		LOG_ERR("fail to allocate mva of main program!\n");
		m4u_dealloc_mva(m4u_client, VPU_PORT_OF_IOMMU, mva_main_program);
		goto out;
	}
	LOG_DBG("vpu...m4u_alloc_mva main_program ok, (0x%x/0x%x)\n",
		(unsigned int)(binpa_main_program), (unsigned int)VPU_SIZE_MAIN_PROGRAM);

	/* 3. map all algo binary data(src addr for dps to copy) */
	/* no need reserved mva, use SG_READY*/
	if (core == 0) {
		sg = &(vpu_service_cores[core].sg_algo_binary_data);
		ret = sg_alloc_table(sg, 1, GFP_KERNEL);
		CHECK_RET("[vpu_%d]fail to allocate sg table[reset]!\n", core);
		LOG_DBG("vpu...sg_alloc_table algo_data ok, mva_algo_binary_data = 0x%x\n",
			mva_algo_binary_data);

		sg_dma_address(sg->sgl) = bin_pa + VPU_OFFSET_ALGO_AREA;
		sg_dma_len(sg->sgl) = size_algos;
		ret = m4u_alloc_mva(m4u_client, VPU_PORT_OF_IOMMU,
				0, sg,
				size_algos,
				M4U_PROT_READ | M4U_PROT_WRITE,
				M4U_FLAGS_SG_READY, &mva_algo_binary_data);
		CHECK_RET("[vpu_%d]fail to allocate mva of reset vecter!\n", core);
		vpu_service_cores[core].algo_data_mva = mva_algo_binary_data;
		LOG_DBG("a vpu va_algo_data pa: 0x%x\n", (unsigned int)(bin_pa + VPU_OFFSET_ALGO_AREA));
		LOG_DBG("a vpu va_algo_data: 0x%x/0x%x, size: 0x%x\n", mva_algo_binary_data,
			(unsigned int)(vpu_service_cores[core].algo_data_mva), (unsigned int)size_algos);
	} else {
		vpu_service_cores[core].algo_data_mva = vpu_service_cores[0].algo_data_mva;
	}

	/* 4. map main program iram data */
	/* no need reserved mva, use SG_READY*/
	sg = &(vpu_service_cores[core].sg_iram_data);
	ret = sg_alloc_table(sg, 1, GFP_KERNEL);
	CHECK_RET("[vpu_%d]fail to allocate sg table[reset]!\n", core);
	LOG_DBG("vpu...sg_alloc_table iram_data ok, mva_iram_data = 0x%x\n", mva_iram_data);
	LOG_DBG("a vpu iram pa: 0x%lx\n", (unsigned long)(binpa_iram_data));
	sg_dma_address(sg->sgl) = binpa_iram_data;
	sg_dma_len(sg->sgl) = VPU_SIZE_MAIN_PROGRAM_IMEM;
	ret = m4u_alloc_mva(m4u_client, VPU_PORT_OF_IOMMU,
			0, sg,
			VPU_SIZE_MAIN_PROGRAM_IMEM,
			M4U_PROT_READ | M4U_PROT_WRITE,
			M4U_FLAGS_SG_READY, &mva_iram_data);
	CHECK_RET("[vpu_%d]fail to allocate mva of iram data!\n", core);
	vpu_service_cores[core].iram_data_mva = (uint64_t)(mva_iram_data);
	LOG_DBG("a vpu va_iram_data: 0x%x, iram_data_mva: 0x%lx\n",
		mva_iram_data, (unsigned long)(vpu_service_cores[core].iram_data_mva));

out:
#endif
	return ret;
}
#endif

static int vpu_get_power(int core)
{
	int ret = 0;

	LOG_DBG("[vpu_%d/%d] gp +\n", core, power_counter[core]);
	mutex_lock(&power_counter_mutex[core]);
	power_counter[core]++;
	ret = vpu_boot_up(core);
	mutex_unlock(&power_counter_mutex[core]);
	LOG_DBG("[vpu_%d/%d] gp + 2\n", core, power_counter[core]);
	if (ret == POWER_ON_MAGIC) {
		mutex_lock(&opp_mutex);
		if (force_change_vcore_opp[core]) {
			mutex_unlock(&opp_mutex);
			/* vcore change should wait */
			LOG_DBG("vpu_%d force change vcore opp", core);
			vpu_change_opp(core, OPPTYPE_VCORE);
		} else {
			mutex_unlock(&opp_mutex);
		}

		mutex_lock(&opp_mutex);
		if (force_change_dsp_freq[core]) {
			mutex_unlock(&opp_mutex);
			/* force change freq while running */
			LOG_DBG("vpu_%d force change dsp freq", core);
			vpu_change_opp(core, OPPTYPE_DSPFREQ);
		} else {
			mutex_unlock(&opp_mutex);
		}
	}
	LOG_DBG("[vpu_%d/%d] gp -\n", core, power_counter[core]);

	if (ret == POWER_ON_MAGIC)
		return 0;
	else
		return ret;
}

static void vpu_put_power(int core, enum VpuPowerOnType type)
{
	LOG_DBG("[vpu_%d/%d] pp +\n", core, power_counter[core]);
	mutex_lock(&power_counter_mutex[core]);
	if (--power_counter[core] == 0) {
		switch (type) {
		case VPT_PRE_ON:
			LOG_DBG("[vpu_%d] VPT_PRE_ON\n", core);
			mod_delayed_work(wq, &(power_counter_work[core].my_work),
				msecs_to_jiffies(10 * PWR_KEEP_TIME_MS));
			break;
		case VPT_ENQUE_ON:
		default:
			LOG_DBG("[vpu_%d] VPT_ENQUE_ON\n", core);
		mod_delayed_work(wq, &(power_counter_work[core].my_work), msecs_to_jiffies(PWR_KEEP_TIME_MS));
		}
	}
	mutex_unlock(&power_counter_mutex[core]);
	LOG_DBG("[vpu_%d/%d] pp -\n", core, power_counter[core]);
}

int vpu_set_power(struct vpu_user *user, struct vpu_power *power)
{
	int ret = 0;
	uint8_t vcore_opp_index = 0xFF;
	uint8_t dsp_freq_index = 0xFF;
	int i = 0, core = -1;

	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		/*LOG_DBG("debug i(%d), (0x1 << i) (0x%x)", i, (0x1 << i));*/
		if (power->core == (0x1 << i)) {
			core = i;
			break;
		}
	}

	if (core >= MTK_VPU_CORE || core < 0) {
		LOG_ERR("wrong core index (0x%x/%d/%d)", power->core, core, MTK_VPU_CORE);
		ret = -1;
		return ret;
	}

	LOG_INF("[vpu_%d] set power opp:%d, pid=%d, tid=%d\n",
			core, power->opp_step,
			user->open_pid, user->open_tgid);

	if (power->opp_step == 0xFF) {
		vcore_opp_index = 0xFF;
		dsp_freq_index = 0xFF;
	} else {
		if (power->opp_step < VPU_MAX_NUM_OPPS && power->opp_step >= 0) {
			vcore_opp_index = opps.vcore.opp_map[power->opp_step];
			dsp_freq_index = opps.dspcore[core].opp_map[power->opp_step];
		} else {
			LOG_ERR("wrong opp step (%d)", power->opp_step);
			ret = -1;
			return ret;
		}
	}
	vpu_opp_check(core, vcore_opp_index, dsp_freq_index);
	user->power_opp = power->opp_step;

	ret = vpu_get_power(core);
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	vpu_service_cores[core].state = VCT_IDLE;
	mutex_unlock(&(vpu_service_cores[core].state_mutex));

	/* to avoid power leakage, power on/off need be paired */
	vpu_put_power(core, VPT_PRE_ON);
	LOG_INF("[vpu_%d] vpu_set_power -\n", core);
	return ret;
}

static void vpu_power_counter_routine(struct work_struct *work)
{
	int core = 0;
	struct my_struct_t *my_work_core = container_of(work, struct my_struct_t, my_work.work);

	core = my_work_core->core;
	LOG_INF("vpu_%d counterR +", core);

	mutex_lock(&power_counter_mutex[core]);
	if (power_counter[core] == 0)
		vpu_shut_down(core);
	else
		LOG_DBG("vpu_%d no need this time.\n", core);
	mutex_unlock(&power_counter_mutex[core]);

	LOG_INF("vpu_%d counterR -", core);
}

int vpu_init_hw(int core, struct vpu_device *device)
{
	int ret, i;
	int param;

	struct vpu_shared_memory_param mem_param;

	LOG_INF("[vpu] core : %d\n", core);

	vpu_service_cores[core].vpu_base = device->vpu_base[core];
	vpu_service_cores[core].bin_base = device->bin_base;

#ifdef MTK_VPU_EMULATOR
	vpu_request_emulator_irq(device->irq_num[core], vpu_isr_handler);
#else
	m4u_client = m4u_create_client();
	ion_client = ion_client_create(g_ion_device, "vpu");

	ret = vpu_map_mva_of_bin(core, device->bin_pa);
	CHECK_RET("[vpu_%d]fail to map binary data!\n", core);

	ret = request_irq(device->irq_num[core], (irq_handler_t)VPU_ISR_CB_TBL[core].isr_fp,
		device->irq_trig_level, (const char *)VPU_ISR_CB_TBL[core].device_name, NULL);
	CHECK_RET("[vpu_%d]fail to request vpu irq!\n", core);
#endif


	if (core == 0) {
		init_waitqueue_head(&cmd_wait);
		mutex_init(&lock_mutex);
		init_waitqueue_head(&lock_wait);
		mutex_init(&opp_mutex);
		init_waitqueue_head(&waitq_change_vcore);
		init_waitqueue_head(&waitq_do_core_executing);
		is_locked = false;
		max_vcore_opp = 0;
		max_dsp_freq = 0;
		vpu_dev = device;
		is_power_debug_lock = false;

		for (i = 0 ; i < MTK_VPU_CORE ; i++) {
			mutex_init(&(power_mutex[i]));
			mutex_init(&(power_counter_mutex[i]));
			power_counter[i] = 0;
			power_counter_work[i].core = i;
			is_power_on[i] = false;
			force_change_vcore_opp[i] = false;
			force_change_dsp_freq[i] = false;
			INIT_DELAYED_WORK(&(power_counter_work[i].my_work), vpu_power_counter_routine);

			vpu_service_cores[i].vpu_service_task =
				(struct task_struct *)kmalloc(sizeof(struct task_struct), GFP_KERNEL);
			if (vpu_service_cores[i].vpu_service_task != NULL) {
				param = i;
				vpu_service_cores[i].thread_variable = i;
				if (i == 0) {
					vpu_service_cores[i].vpu_service_task =
						kthread_create(vpu_service_routine,
						&(vpu_service_cores[i].thread_variable), "vpu0");
				} else {
					vpu_service_cores[i].vpu_service_task =
						kthread_create(vpu_service_routine,
						&(vpu_service_cores[i].thread_variable), "vpu1");
				}
				if (IS_ERR(vpu_service_cores[i].vpu_service_task)) {
					ret = PTR_ERR(vpu_service_cores[i].vpu_service_task);
					goto out;
				}
			} else {
				LOG_ERR("allocate enque task(%d) fail", i);
				goto out;
			}
			wake_up_process(vpu_service_cores[i].vpu_service_task);

			mutex_init(&(vpu_service_cores[i].cmd_mutex));
			vpu_service_cores[i].is_cmd_done = false;
			mutex_init(&(vpu_service_cores[i].state_mutex));
			vpu_service_cores[i].state = VCT_SHUTDOWN;

			/* working buffer */
			/* no need in reserved region */
			mem_param.require_pa = true;
			mem_param.require_va = true;
			mem_param.size = VPU_SIZE_WORK_BUF;
			mem_param.fixed_addr = 0;
			ret = vpu_alloc_shared_memory(&(vpu_service_cores[i].work_buf), &mem_param);
			LOG_INF("core(%d):work_buf va (0x%lx),pa(0x%x)",
				i, (unsigned long)(vpu_service_cores[i].work_buf->va),
				vpu_service_cores[i].work_buf->pa);
			CHECK_RET("core(%d):fail to allocate working buffer!\n", i);

			/* execution kernel library */
			/* need in reserved region, set end as the end of reserved address, m4u use start-from */
			mem_param.require_pa = true;
			mem_param.require_va = true;
			mem_param.size = VPU_SIZE_ALGO_KERNEL_LIB;
			switch (i) {
			case 0:
			default:
				mem_param.fixed_addr = VPU_MVA_KERNEL_LIB;
				break;
			case 1:
				mem_param.fixed_addr = VPU2_MVA_KERNEL_LIB;
				break;
			}

			ret = vpu_alloc_shared_memory(&(vpu_service_cores[i].exec_kernel_lib), &mem_param);
			LOG_INF("core(%d):kernel_lib va (0x%lx),pa(0x%x)",
				i, (unsigned long)(vpu_service_cores[i].exec_kernel_lib->va),
				vpu_service_cores[i].exec_kernel_lib->pa);
			CHECK_RET("core(%d):fail to allocate kernel_lib buffer!\n", i);

		}
		/* multi-core shared  */
		/* need in reserved region, set end as the end of reserved address, m4u use start-from */
		mem_param.require_pa = true;
		mem_param.require_va = true;
		mem_param.size = VPU_SIZE_SHARED_DATA;
		mem_param.fixed_addr = VPU_MVA_SHARED_DATA;
		ret = vpu_alloc_shared_memory(&(core_shared_data), &mem_param);
		LOG_DBG("shared_data va (0x%lx),pa(0x%x)",
			(unsigned long)(core_shared_data->va), core_shared_data->pa);
		CHECK_RET("fail to allocate working buffer!\n");


		wq = create_workqueue("vpu_wq");
		g_func_mask = 0x0;


		/* define the steps and OPP */
#define DEFINE_VPU_STEP(step, m, v0, v1, v2, v3, v4, v5, v6, v7) \
			{ \
				opps.step.index = m - 1; \
				opps.step.count = m; \
				opps.step.values[0] = v0; \
				opps.step.values[1] = v1; \
				opps.step.values[2] = v2; \
				opps.step.values[3] = v3; \
				opps.step.values[4] = v4; \
				opps.step.values[5] = v5; \
				opps.step.values[6] = v6; \
				opps.step.values[7] = v7; \
			}


#define DEFINE_VPU_OPP(i, v0, v1, v2, v3, v4) \
		{ \
			opps.vcore.opp_map[i]  = v0; \
			opps.dsp.opp_map[i]    = v1; \
			opps.dspcore[0].opp_map[i]   = v2; \
			opps.dspcore[1].opp_map[i]   = v3; \
			opps.ipu_if.opp_map[i] = v4; \
		}


		DEFINE_VPU_STEP(vcore,  2, 800000, 725000, 0, 0, 0, 0, 0, 0);
		DEFINE_VPU_STEP(dsp,    8, 525000, 450000, 416000, 364000, 312000, 273000, 208000, 182000);
		DEFINE_VPU_STEP(dspcore[0],   8, 525000, 450000, 416000, 364000, 312000, 273000, 208000, 182000);
		DEFINE_VPU_STEP(dspcore[1],   8, 525000, 450000, 416000, 364000, 312000, 273000, 208000, 182000);
		DEFINE_VPU_STEP(ipu_if, 8, 525000, 450000, 416000, 364000, 312000, 273000, 208000, 182000);

		/* default freq */
		DEFINE_VPU_OPP(0, 0, 0, 0, 0, 0);
		DEFINE_VPU_OPP(1, 0, 1, 1, 1, 1);
		DEFINE_VPU_OPP(2, 0, 2, 2, 2, 2);
		DEFINE_VPU_OPP(3, 0, 3, 3, 3, 3);
		DEFINE_VPU_OPP(4, 1, 4, 4, 4, 4);
		DEFINE_VPU_OPP(5, 1, 5, 5, 5, 5);
		DEFINE_VPU_OPP(6, 1, 6, 6, 6, 6);
		DEFINE_VPU_OPP(7, 1, 7, 7, 7, 7);

		/* default low opp */
		opps.count = 8;
		opps.index = 4; /* user space usage*/
		opps.vcore.index = 1;
		opps.dsp.index = 4;
		opps.dspcore[0].index = 4;
		opps.dspcore[1].index = 4;
		opps.ipu_if.index = 4;

#undef DEFINE_VPU_OPP
#undef DEFINE_VPU_STEP

		ret = vpu_prepare_regulator_and_clock(vpu_dev->dev[core]);
		CHECK_RET("[vpu_%d]fail to prepare regulator or clock!\n", core);

		/* pmqos  */
		#ifdef ENABLE_PMQOS
		pm_qos_add_request(&vpu_qos_bw_request, PM_QOS_MM_MEMORY_BANDWIDTH, PM_QOS_DEFAULT_VALUE);
		pm_qos_add_request(&vpu_qos_vcore_request, PM_QOS_VCORE_OPP, PM_QOS_VCORE_OPP_DEFAULT_VALUE);
		#endif

	}
	return 0;

out:

	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		if (vpu_service_cores[i].vpu_service_task != NULL) {
			kfree(vpu_service_cores[i].vpu_service_task);
			vpu_service_cores[i].vpu_service_task = NULL;
		}

		if (vpu_service_cores[i].work_buf)
			vpu_free_shared_memory(vpu_service_cores[i].work_buf);
	}

	return ret;
}

int vpu_uninit_hw(void)
{
	int i;

	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		cancel_delayed_work(&(power_counter_work[i].my_work));

		if (vpu_service_cores[i].vpu_service_task != NULL) {
			kthread_stop(vpu_service_cores[i].vpu_service_task);
			kfree(vpu_service_cores[i].vpu_service_task);
			vpu_service_cores[i].vpu_service_task = NULL;
		}

		if (vpu_service_cores[i].work_buf) {
			vpu_free_shared_memory(vpu_service_cores[i].work_buf);
			vpu_service_cores[i].work_buf = NULL;
		}
	}
	/* pmqos  */
	#ifdef ENABLE_PMQOS
	pm_qos_remove_request(&vpu_qos_bw_request);
	pm_qos_remove_request(&vpu_qos_vcore_request);
	#endif

	vpu_unprepare_regulator_and_clock();

	if (m4u_client) {
		m4u_destroy_client(m4u_client);
		m4u_client = NULL;
	}

	if (ion_client) {
		ion_client_destroy(ion_client);
		ion_client = NULL;
	}

	if (wq) {
		flush_workqueue(wq);
		destroy_workqueue(wq);
		wq = NULL;
	}

	return 0;
}


static int vpu_check_precond(int core)
{
	uint32_t status;
	size_t i;

	/* wait 1 seconds, if not ready or busy */
	for (i = 0; i < 50; i++) {
		status = vpu_read_field(core, FLD_XTENSA_INFO00);
		switch (status) {
		case VPU_STATE_READY:
		case VPU_STATE_IDLE:
		case VPU_STATE_ERROR:
			return 0;
		case VPU_STATE_NOT_READY:
		case VPU_STATE_BUSY:
			msleep(20);
			break;
		case VPU_STATE_TERMINATED:
			return -EBADFD;
		}
	}
	LOG_ERR("core(%d) still busy(%d) after wait 1 second.\n", core, status);
	return -EBUSY;
}

static int vpu_check_postcond(int core)
{
	uint32_t status = vpu_read_field(core, FLD_XTENSA_INFO00);

	LOG_DBG("vpu_check_postcond (0x%x)", status);

	switch (status) {
	case VPU_STATE_READY:
	case VPU_STATE_IDLE:
		return 0;
	case VPU_STATE_NOT_READY:
	case VPU_STATE_BUSY:
		return -EIO;
	default:
		return -EINVAL;
	}
}

int vpu_hw_enable_jtag(bool enabled)
{
	int ret;
	int TEMP_CORE = 0;

	vpu_get_power(TEMP_CORE);
#if 0
	ret = mt_set_gpio_mode(GPIO14 | 0x80000000, enabled ? GPIO_MODE_05 : GPIO_MODE_01);
	ret |= mt_set_gpio_mode(GPIO15 | 0x80000000, enabled ? GPIO_MODE_05 : GPIO_MODE_01);
	ret |= mt_set_gpio_mode(GPIO16 | 0x80000000, enabled ? GPIO_MODE_05 : GPIO_MODE_01);
	ret |= mt_set_gpio_mode(GPIO17 | 0x80000000, enabled ? GPIO_MODE_05 : GPIO_MODE_01);
	ret |= mt_set_gpio_mode(GPIO18 | 0x80000000, enabled ? GPIO_MODE_05 : GPIO_MODE_01);
#else
	ret = 0;
#endif

	CHECK_RET("fail to config gpio-jtag2!\n");

	vpu_write_field(TEMP_CORE, FLD_SPNIDEN, enabled);
	vpu_write_field(TEMP_CORE, FLD_SPIDEN, enabled);
	vpu_write_field(TEMP_CORE, FLD_NIDEN, enabled);
	vpu_write_field(TEMP_CORE, FLD_DBG_EN, enabled);

out:
	vpu_put_power(TEMP_CORE, VPT_ENQUE_ON);
	return ret;
}

int vpu_hw_boot_sequence(int core)
{
	int ret;
	uint64_t ptr_ctrl;
	uint64_t ptr_reset;
	uint64_t ptr_axi_0;
	uint64_t ptr_axi_1;
	unsigned int reg_value = 0;

	vpu_trace_begin("vpu_hw_boot_sequence");
	LOG_INF("boot-seq core(%d)", core);
	LOG_DBG("CTRL(0x%x)", vpu_read_reg32(vpu_service_cores[core].vpu_base, CTRL_BASE_OFFSET + 0x110));
	LOG_DBG("XTENSA_INT(0x%x)", vpu_read_reg32(vpu_service_cores[core].vpu_base, CTRL_BASE_OFFSET + 0x114));
	LOG_DBG("CTL_XTENSA_INT(0x%x)", vpu_read_reg32(vpu_service_cores[core].vpu_base, CTRL_BASE_OFFSET + 0x118));
	LOG_DBG("CTL_XTENSA_INT_CLR(0x%x)", vpu_read_reg32(vpu_service_cores[core].vpu_base, CTRL_BASE_OFFSET + 0x11C));

	lock_command(core);
	ptr_ctrl = vpu_service_cores[core].vpu_base + g_vpu_reg_descs[REG_CTRL].offset;
	ptr_reset = vpu_service_cores[core].vpu_base + g_vpu_reg_descs[REG_SW_RST].offset;
	ptr_axi_0 = vpu_service_cores[core].vpu_base + g_vpu_reg_descs[REG_AXI_DEFAULT0].offset;
	ptr_axi_1 = vpu_service_cores[core].vpu_base + g_vpu_reg_descs[REG_AXI_DEFAULT1].offset;

	/* 1. write register */
	/* set specific address for reset vector in external boot */
	reg_value = vpu_read_field(core, FLD_CORE_XTENSA_ALTRESETVEC);
	LOG_DBG("vpu bf ALTRESETVEC (0x%x), RV(0x%x)\n",
		reg_value, VPU_MVA_RESET_VECTOR);
	switch (core) {
	case 0:
	default:
		vpu_write_field(core, FLD_CORE_XTENSA_ALTRESETVEC, VPU_MVA_RESET_VECTOR);
		break;
	case 1:
		vpu_write_field(core, FLD_CORE_XTENSA_ALTRESETVEC, VPU2_MVA_RESET_VECTOR);
		break;
	}
	reg_value = vpu_read_field(core, FLD_CORE_XTENSA_ALTRESETVEC);
	LOG_DBG("vpu af ALTRESETVEC (0x%x), RV(0x%x)\n",
		reg_value, VPU_MVA_RESET_VECTOR);

	VPU_SET_BIT(ptr_ctrl, 31);      /* csr_p_debug_enable */
	VPU_SET_BIT(ptr_ctrl, 26);      /* debug interface cock gated enable */
	VPU_SET_BIT(ptr_ctrl, 19);      /* force to boot based on XTENSA_ALTRESETVEC */
	VPU_SET_BIT(ptr_ctrl, 23);      /* RUN_STALL pull up */
	VPU_SET_BIT(ptr_ctrl, 17);      /* pif gated enable */
	#if 0
	VPU_SET_BIT(ptr_ctrl, 29);     /* SHARE_SRAM_CONFIG: default: imem 196K, icache 0K*/
	VPU_CLR_BIT(ptr_ctrl, 28);
	VPU_CLR_BIT(ptr_ctrl, 27);
	#else
	/* SHARE_SRAM_CONFIG: default: imem 64, icache 128K*/
	VPU_CLR_BIT(ptr_ctrl, 29);
	VPU_CLR_BIT(ptr_ctrl, 28);
	VPU_CLR_BIT(ptr_ctrl, 27);
	#endif
	VPU_SET_BIT(ptr_reset, 12);     /* OCD_HALT_ON_RST pull up */
	ndelay(27);                     /* wait for 27ns */

	VPU_CLR_BIT(ptr_reset, 12);     /* OCD_HALT_ON_RST pull down */
	VPU_SET_BIT(ptr_reset, 4);      /* B_RST pull up */
	VPU_SET_BIT(ptr_reset, 8);      /* D_RST pull up */
	ndelay(27);                     /* wait for 27ns */

	VPU_CLR_BIT(ptr_reset, 4);      /* B_RST pull down */
	VPU_CLR_BIT(ptr_reset, 8);      /* D_RST pull down */
	ndelay(27);                     /* wait for 27ns */

	VPU_CLR_BIT(ptr_ctrl, 17);      /* pif gated disable, to prevent unknown propagate to BUS */
#ifndef BYPASS_M4U_DBG
	VPU_SET_BIT(ptr_axi_0, 22);		/* AXI Request via M4U */
	VPU_SET_BIT(ptr_axi_0, 27);
	VPU_SET_BIT(ptr_axi_1, 4);       /* AXI Request via M4U */
	VPU_SET_BIT(ptr_axi_1, 9);
#endif
	/* default set pre-ultra instead of ultra */
	VPU_SET_BIT(ptr_axi_0, 28);

	if (g_vpu_log_level > 1)
		LOG_INF("[vpu_%d] REG_AXI_DEFAULT0(0x%x)\n", core,
			vpu_read_reg32(vpu_service_cores[core].vpu_base, CTRL_BASE_OFFSET + 0x13C));

	LOG_DBG("[vpu_%d] REG_AXI_DEFAULT1(0x%x)\n", core,
		vpu_read_reg32(vpu_service_cores[core].vpu_base, CTRL_BASE_OFFSET + 0x140));

	/* 2. trigger to run */
	LOG_DBG("vpu dsp:running (%d/0x%x)", core, vpu_read_field(core, FLD_SRAM_CONFIGURE));
	vpu_trace_begin("dsp:running");

	VPU_CLR_BIT(ptr_ctrl, 23);      /* RUN_STALL pull down */


	/* 3. wait until done */
	ret = wait_command(core);
	VPU_SET_BIT(ptr_ctrl, 23);      /* RUN_STALL pull up to avoid fake cmd */
	vpu_trace_end();
	if (ret) {
		LOG_ERR("[vpu_%d] boot-up timeout , status(%d/%d)\n", core,
			vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
		vpu_dump_register(NULL);
		vpu_dump_debug_stack(core, DEBUG_STACK_SIZE);
		vpu_aee("VPU Timeout", "timeout to external boot\n");
		goto out;
	}

	/* 4. check the result of boot sequence */
	ret = vpu_check_postcond(core);
	CHECK_RET("[vpu_%d]fail to boot vpu!\n", core);

out:
	unlock_command(core);
	vpu_trace_end();
	LOG_INF("vpu_hw_boot_sequence -");
	return ret;
}

int vpu_hw_set_debug(int core)
{
	int ret;
	struct timespec now;

	LOG_DBG("vpu_hw_set_debug (%d)+", core);
	vpu_trace_begin("vpu_hw_set_debug");

	lock_command(core);

	/* 1. set debug */
	getnstimeofday(&now);
	vpu_write_field(core, FLD_XTENSA_INFO01, VPU_CMD_SET_DEBUG);
	vpu_write_field(core, FLD_XTENSA_INFO19, vpu_service_cores[core].iram_data_mva);
	vpu_write_field(core, FLD_XTENSA_INFO21, vpu_service_cores[core].work_buf->pa + VPU_OFFSET_LOG);
	vpu_write_field(core, FLD_XTENSA_INFO22, VPU_SIZE_LOG_BUF);
	vpu_write_field(core, FLD_XTENSA_INFO23, now.tv_sec * 1000000 + now.tv_nsec / 1000);
	LOG_DBG("work_buf->pa + VPU_OFFSET_LOG (0x%lx)",
		(unsigned long)(vpu_service_cores[core].work_buf->pa + VPU_OFFSET_LOG));
	LOG_DBG("vpu_set ok, running");

	/* 2. trigger interrupt */
	vpu_trace_begin("dsp:running");
	vpu_write_field(core, FLD_RUN_STALL, 0);      /* RUN_STALL pull down*/
	vpu_write_field(core, FLD_CTL_INT, 1);
	LOG_DBG("debug timestamp: %.2lu:%.2lu:%.2lu:%.6lu\n", (now.tv_sec / 3600) % (24),
			(now.tv_sec / 60) % (60), now.tv_sec % 60, now.tv_nsec / 1000);
	/* 3. wait until done */
	ret = wait_command(core);
	vpu_write_field(core, FLD_RUN_STALL, 1);      /* RUN_STALL pull up to avoid fake cmd */
	vpu_trace_end();
	if (ret) {
		LOG_ERR("[vpu_%d] set-debug timeout , status(%d/%d)\n", core,
			vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
		vpu_dump_mesg(NULL);
		vpu_dump_register(NULL);
	}
	CHECK_RET("[vpu_%d]timeout of set debug\n", core);

	/* 4. check the result */
	ret = vpu_check_postcond(core);
	CHECK_RET("[vpu_%d]fail to set debug!\n", core);

out:
	unlock_command(core);
	vpu_trace_end();
	LOG_DBG("vpu_hw_set_debug -");
	return ret;
}

int vpu_get_name_of_algo(int core, int id, char **name)
{
	int i;
	int tmp = id;
	struct vpu_image_header *header;

	header = (struct vpu_image_header *) ((uintptr_t)vpu_service_cores[core].bin_base + (VPU_OFFSET_IMAGE_HEADERS));
	for (i = 0; i < VPU_NUMS_IMAGE_HEADER; i++) {
		if (tmp > header[i].algo_info_count) {
			tmp -= header[i].algo_info_count;
			continue;
		}

		*name = header[i].algo_infos[tmp - 1].name;
		return 0;
	}

	*name = NULL;
	LOG_ERR("algo is not existed, id=%d\n", id);
	return -ENOENT;
}

int vpu_get_entry_of_algo(int core, char *name, int *id, unsigned int *mva, int *length)
{
	int i, j;
	int s = 1;
	unsigned int coreMagicNum;
	struct vpu_algo_info *algo_info;
	struct vpu_image_header *header;

	LOG_DBG("[vpu] vpu_get_entry_of_algo +\n");
	/* coreMagicNum = ( 0x60 | (0x01 << core) ); */
	/* ignore vpu version */
	coreMagicNum = (0x01 << core);

	header = (struct vpu_image_header *) ((uintptr_t)vpu_service_cores[core].bin_base + (VPU_OFFSET_IMAGE_HEADERS));
	for (i = 0; i < VPU_NUMS_IMAGE_HEADER; i++) {
		for (j = 0; j < header[i].algo_info_count; j++) {
			algo_info = &header[i].algo_infos[j];
			LOG_INF("algo name: %s/%s, core info:0x%x, input core:%d, magicNum: 0x%x, 0x%x\n",
				name, algo_info->name, (unsigned int)(algo_info->vpu_core),
				core, (unsigned int)coreMagicNum,
				algo_info->vpu_core & coreMagicNum);
			/* CHRISTODO */
			if ((strcmp(name, algo_info->name) == 0) &&
				(algo_info->vpu_core & coreMagicNum)) {
				LOG_DBG("algo_info->offset(0x%x)/0x%x",
					algo_info->offset, (unsigned int)(vpu_service_cores[core].algo_data_mva));
				*mva = algo_info->offset - VPU_OFFSET_ALGO_AREA + vpu_service_cores[core].algo_data_mva;
				LOG_DBG("*mva(0x%x/0x%lx), s(%d)", *mva, (unsigned long)(*mva), s);
				*length = algo_info->length;
				*id = s;
				return 0;
			}
			s++;
		}
	}

	*id = 0;
	LOG_ERR("algo is not existed, name=%s\n", name);
	return -ENOENT;
};


int vpu_ext_be_busy(void)
{
	int ret;
	/* CHRISTODO */
	int TEMP_CORE = 0;

	lock_command(TEMP_CORE);

	/* 1. write register */
	vpu_write_field(TEMP_CORE, FLD_XTENSA_INFO01, VPU_CMD_EXT_BUSY);            /* command: be busy */
	/* 2. trigger interrupt */

	vpu_write_field(TEMP_CORE, FLD_CTL_INT, 1);

	/* 3. wait until done */
	ret = wait_command(TEMP_CORE);

	unlock_command(TEMP_CORE);
	return ret;
}

int vpu_debug_func_core_state(int core, enum VpuCoreState state)
{
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	vpu_service_cores[core].state = state;
	mutex_unlock(&(vpu_service_cores[core].state_mutex));
	return 0;
}

int vpu_boot_up(int core)
{
	int ret = 0;

	LOG_DBG("[vpu_%d] boot_up +\n", core);
	mutex_lock(&power_mutex[core]);
	LOG_DBG("[vpu_%d] is_power_on(%d)\n", core, is_power_on[core]);
	if (is_power_on[core]) {
		mutex_unlock(&power_mutex[core]);
		mutex_lock(&(vpu_service_cores[core].state_mutex));
		vpu_service_cores[core].state = VCT_BOOTUP;
		mutex_unlock(&(vpu_service_cores[core].state_mutex));
		wake_up_interruptible(&waitq_change_vcore);
		return POWER_ON_MAGIC;
	}
	LOG_DBG("[vpu_%d] boot_up flag2\n", core);

	vpu_trace_begin("vpu_boot_up");
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	vpu_service_cores[core].state = VCT_BOOTUP;
	mutex_unlock(&(vpu_service_cores[core].state_mutex));
	wake_up_interruptible(&waitq_change_vcore);

	ret = vpu_enable_regulator_and_clock(core);
	CHECK_RET("[vpu_%d]fail to enable regulator or clock\n", core);

	ret = vpu_hw_boot_sequence(core);
	CHECK_RET("[vpu_%d]fail to do boot sequence\n", core);
	LOG_DBG("[vpu_%d] vpu_hw_boot_sequence done\n", core);

	ret = vpu_hw_set_debug(core);
	CHECK_RET("[vpu_%d]fail to set debug\n", core);
	LOG_DBG("[vpu_%d] vpu_hw_set_debug done\n", core);

#ifdef MET_POLLING_MODE
	ret = vpu_profile_state_set(core, 1);
	CHECK_RET("[vpu_%d] fail to vpu_profile_state_set 1\n", core);
#endif

out:
#if 0 /* control on/off outside the via get_power/put_power */
	if (ret) {
		ret = vpu_disable_regulator_and_clock(core);
		CHECK_RET("[vpu_%d]fail to disable regulator and clock\n", core);
	}
#endif
	vpu_trace_end();
	mutex_unlock(&power_mutex[core]);
	return ret;
}

int vpu_shut_down(int core)
{
	int ret = 0;
	/*int i = 0;*/
	/*bool shut_down = true;*/

	LOG_DBG("[vpu_%d] shutdown +\n", core);
	mutex_lock(&power_mutex[core]);
	if (!is_power_on[core]) {
		mutex_unlock(&power_mutex[core]);
		return 0;
	}

	#if 0
	for (i = 0 ; i < MTK_VPU_CORE ; i++) {
		mutex_lock(&(vpu_service_cores[i].state_mutex));
		switch (vpu_service_cores[i].state) {
		case VCT_SHUTDOWN:
		case VCT_IDLE:
		case VCT_NONE:
			vpu_service_cores[core].current_algo = 0;
			vpu_service_cores[core].state = VCT_SHUTDOWN;
			break;
		case VCT_BOOTUP:
		case VCT_EXECUTING:
			mutex_unlock(&(vpu_service_cores[i].state_mutex));
			goto out;
			/*break;*/
		}
		mutex_unlock(&(vpu_service_cores[i].state_mutex));
	}
	#else
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	switch (vpu_service_cores[core].state) {
	case VCT_SHUTDOWN:
	case VCT_IDLE:
	case VCT_NONE:
		vpu_service_cores[core].current_algo = 0;
		vpu_service_cores[core].state = VCT_SHUTDOWN;
		mutex_unlock(&(vpu_service_cores[core].state_mutex));
		break;
	case VCT_BOOTUP:
	case VCT_EXECUTING:
	case VCT_VCORE_CHG:
		mutex_unlock(&(vpu_service_cores[core].state_mutex));
		goto out;
		/*break;*/
	}
	#endif

#ifdef MET_POLLING_MODE
	ret = vpu_profile_state_set(core, 0);
	CHECK_RET("[vpu_%d] fail to vpu_profile_state_set 0\n", core);
#endif

	vpu_trace_begin("vpu_shut_down");
	ret = vpu_disable_regulator_and_clock(core);
	CHECK_RET("[vpu_%d]fail to disable regulator and clock\n", core);
	wake_up_interruptible(&waitq_change_vcore);
out:
	vpu_trace_end();
	mutex_unlock(&power_mutex[core]);
	LOG_DBG("[vpu_%d] shutdown -\n", core);
	return ret;
}

int vpu_hw_load_algo(int core, struct vpu_algo *algo)
{
	int ret;

	LOG_DBG("[vpu_%d] vpu_hw_load_algo +\n", core);
	/* no need to reload algo if have same loaded algo*/
	if (vpu_service_cores[core].current_algo == algo->id[core])
		return 0;

	vpu_trace_begin("vpu_hw_load_algo(%d)", algo->id[core]);
	ret = vpu_get_power(core);
	CHECK_RET("[vpu_%d]fail to get power!\n", core);
	LOG_DBG("[vpu_%d] vpu_get_power done\n", core);

	ret = wait_to_do_vpu_running(core);
	CHECK_RET("[vpu_%d]load_algo fail to wait_to_do_vpu_running!, ret = %d\n", core, ret);
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	vpu_service_cores[core].state = VCT_EXECUTING;
	mutex_unlock(&(vpu_service_cores[core].state_mutex));

	lock_command(core);
	LOG_DBG("start to load algo\n");

	ret = vpu_check_precond(core);
	CHECK_RET("[vpu_%d]have wrong status before do loader!\n", core);
	LOG_DBG("[vpu_%d] vpu_check_precond done\n", core);

	LOG_DBG("[vpu_%d] algo ptr/length (0x%lx/0x%x)\n", core,
		(unsigned long)algo->bin_ptr, algo->bin_length);
	/* 1. write register */
	vpu_write_field(core, FLD_XTENSA_INFO01, VPU_CMD_DO_LOADER);           /* command: d2d */
	vpu_write_field(core, FLD_XTENSA_INFO12, algo->bin_ptr);              /* binary data's address */
	vpu_write_field(core, FLD_XTENSA_INFO13, algo->bin_length);           /* binary data's length */
	vpu_write_field(core, FLD_XTENSA_INFO15, opps.dsp.values[opps.dsp.index]);
	vpu_write_field(core, FLD_XTENSA_INFO16, opps.ipu_if.values[opps.ipu_if.index]);

	/* 2. trigger interrupt */
	vpu_trace_begin("dsp:running");
	LOG_DBG("[vpu_%d] dsp:running\n", core);
	vpu_write_field(core, FLD_RUN_STALL, 0);      /* RUN_STALL down */
	vpu_write_field(core, FLD_CTL_INT, 1);

	/* 3. wait until done */
	ret = wait_command(core);
	LOG_DBG("[vpu_%d] algo done\n", core);
	vpu_write_field(core, FLD_RUN_STALL, 1);      /* RUN_STALL pull up to avoid fake cmd */
	vpu_trace_end();
	if (ret) {
		LOG_ERR("[vpu_%d] load_algo timeout , status(%d/%d)\n", core,
			vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
		vpu_dump_mesg(NULL);
		vpu_dump_register(NULL);
		vpu_dump_debug_stack(core, DEBUG_STACK_SIZE);
		vpu_dump_code_segment(core, DEBUG_CODE_SEG_SIZE);
		vpu_dump_algo_segment(core, algo->id[core], 0x0);
		vpu_aee("VPU Timeout", "core_%d timeout to do loader, algo_id=%d\n", core,
			vpu_service_cores[core].current_algo);
		goto out;
	}

	/* 4. update the id of loaded algo */
	vpu_service_cores[core].current_algo = algo->id[core];

out:
	unlock_command(core);
	vpu_put_power(core, VPT_ENQUE_ON);
	vpu_trace_end();
	LOG_DBG("[vpu] vpu_hw_load_algo -\n");
	return ret;
}

int vpu_hw_enque_request(int core, struct vpu_request *request)
{
	int ret;

	LOG_DBG("[vpu_%d/%d] eq + ", core, request->algo_id[core]);

	vpu_trace_begin("vpu_hw_enque_request(%d)", request->algo_id[core]);
	ret = vpu_get_power(core);
	CHECK_RET("[vpu_%d]fail to get power!\n", core);

	ret = wait_to_do_vpu_running(core);
	CHECK_RET("[vpu_%d] enq fail to wait_to_do_vpu_running!, ret = %d\n", core, ret);
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	vpu_service_cores[core].state = VCT_EXECUTING;
	mutex_unlock(&(vpu_service_cores[core].state_mutex));

	lock_command(core);
	LOG_DBG("start to enque request\n");

	ret = vpu_check_precond(core);
	if (ret) {
		request->status = VPU_REQ_STATUS_BUSY;
		LOG_ERR("error state before enque request!\n");
		goto out;
	}

	memcpy((void *) (uintptr_t)vpu_service_cores[core].work_buf->va, request->buffers,
			sizeof(struct vpu_buffer) * request->buffer_count);

	if (g_vpu_log_level > 4)
		vpu_dump_buffer_mva(request);
	LOG_INF("[vpu_%d] start d2d, id/frm (%d/%d)\n", core,
		request->algo_id[core], request->frame_magic);
	/* 1. write register */
	/* command: d2d */
	vpu_write_field(core, FLD_XTENSA_INFO01, VPU_CMD_DO_D2D);
	/* buffer count */
	vpu_write_field(core, FLD_XTENSA_INFO12, request->buffer_count);
	/* pointer to array of struct vpu_buffer */
	vpu_write_field(core, FLD_XTENSA_INFO13, vpu_service_cores[core].work_buf->pa);
	/* pointer to property buffer */
	vpu_write_field(core, FLD_XTENSA_INFO14, request->sett_ptr);
	/* size of property buffer */
	vpu_write_field(core, FLD_XTENSA_INFO15, request->sett_length);

	/* 2. trigger interrupt */
	#ifdef ENABLE_PMQOS
	/* pmqos, 10880 Mbytes per second */
	pm_qos_update_request(&vpu_qos_bw_request, request->power_param.bw);/*max 10880*/
	#endif
	vpu_trace_begin("dsp:running");
	LOG_DBG("[vpu] vpu_hw_enque_request running... ");
	#if defined(VPU_MET_READY)
	MET_Events_Trace(1, core, request->algo_id[core]);
	#endif
	vpu_write_field(core, FLD_RUN_STALL, 0);      /* RUN_STALL pull down */
	vpu_write_field(core, FLD_CTL_INT, 1);

	/* 3. wait until done */
	ret = wait_command(core);
	LOG_DBG("[vpu_%d] end d2d\n", core);
	vpu_write_field(core, FLD_RUN_STALL, 1);      /* RUN_STALL pull up to avoid fake cmd */
	#ifdef ENABLE_PMQOS
	/* pmqos, release request after d2d done */
	pm_qos_update_request(&vpu_qos_bw_request, PM_QOS_DEFAULT_VALUE);
	#endif
	vpu_trace_end();
	#if defined(VPU_MET_READY)
	MET_Events_Trace(0, core, request->algo_id[core]);
	#endif
	if (ret) {
		request->status = VPU_REQ_STATUS_TIMEOUT;
		LOG_ERR("[vpu_%d] hw_enque_request timeout , status(%d/%d)\n", core,
			vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
		vpu_dump_buffer_mva(request);
		vpu_dump_mesg(NULL);
		vpu_dump_register(NULL);
		vpu_dump_debug_stack(core, DEBUG_STACK_SIZE);
		vpu_dump_code_segment(core, DEBUG_CODE_SEG_SIZE);
		vpu_aee("VPU Timeout", "core_%d timeout to do d2d, algo_id=%d\n", core,
			vpu_service_cores[core].current_algo);
		goto out;
	}

	request->status = (vpu_check_postcond(core)) ? VPU_REQ_STATUS_FAILURE : VPU_REQ_STATUS_SUCCESS;

out:
	unlock_command(core);
	vpu_put_power(core, VPT_ENQUE_ON);
	vpu_trace_end();
	LOG_DBG("[vpu] vpu_hw_enque_request - (%d)", request->status);
	return ret;

}

/* do whole processing for enque request, including check algo, load algo, run d2d. */
/* minimize timing gap between each step for a single eqneu request and minimize the risk of timing issue */
int vpu_hw_processing_request(int core, struct vpu_request *request)
{
	int ret;
	struct vpu_algo *algo = NULL;
	bool need_reload = false;

	if (g_vpu_log_level > 2)
		LOG_INF("[vpu_%d/%d] pr + ", core, request->algo_id[core]);

	/* step1, enable clocks and boot-up if needed */
	ret = vpu_get_power(core);
	if (ret) {
		LOG_ERR("[vpu_%d] fail to get power!\n", core);
		goto out2;
	}
	LOG_DBG("[vpu_%d] vpu_get_power done\n", core);

	/* step2. check algo */
	if (request->algo_id[core] != vpu_service_cores[core].current_algo) {
		ret = vpu_find_algo_by_id(core, request->algo_id[core], &algo);
		need_reload = true;
		if (ret) {
			request->status = VPU_REQ_STATUS_INVALID;
			LOG_ERR("[vpu_%d] pr can not find the algo, id=%d\n",
				core, request->algo_id[core]);
			goto out2;
		}
	}

	/* step3. do processing, algo loader and d2d*/
	ret = wait_to_do_vpu_running(core);
	CHECK_RET("[vpu_%d]pr load_algo fail to wait_to_do_vpu_running!, ret = %d\n", core, ret);
	mutex_lock(&(vpu_service_cores[core].state_mutex));
	vpu_service_cores[core].state = VCT_EXECUTING;
	mutex_unlock(&(vpu_service_cores[core].state_mutex));
	/* algo loader if needed */
	if (need_reload) {
		vpu_trace_begin("[vpu_%d] hw_load_algo(%d)", core, algo->id[core]);
		lock_command(core);
		LOG_DBG("start to load algo\n");

		ret = vpu_check_precond(core);
		CHECK_RET("[vpu_%d]have wrong status before do loader!\n", core);
		LOG_DBG("[vpu_%d] vpu_check_precond done\n", core);

		if (g_vpu_log_level > 2)
			LOG_INF("[vpu_%d] algo ptr/length (0x%lx/0x%x)\n", core,
				(unsigned long)algo->bin_ptr, algo->bin_length);

		/* 1. write register */
		vpu_write_field(core, FLD_XTENSA_INFO01, VPU_CMD_DO_LOADER);           /* command: d2d */
		vpu_write_field(core, FLD_XTENSA_INFO12, algo->bin_ptr);              /* binary data's address */
		vpu_write_field(core, FLD_XTENSA_INFO13, algo->bin_length);           /* binary data's length */
		vpu_write_field(core, FLD_XTENSA_INFO15, opps.dsp.values[opps.dsp.index]);
		vpu_write_field(core, FLD_XTENSA_INFO16, opps.ipu_if.values[opps.ipu_if.index]);

		/* 2. trigger interrupt */
		vpu_trace_begin("[vpu_%d] dsp:load_algo running", core);
		LOG_DBG("[vpu_%d] dsp:load_algo running\n", core);
		vpu_write_field(core, FLD_RUN_STALL, 0);      /* RUN_STALL down */
		vpu_write_field(core, FLD_CTL_INT, 1);

		/* 3. wait until done */
		ret = wait_command(core);
		if (g_vpu_log_level > 2)
			LOG_INF("[vpu_%d] algo done\n", core);
		vpu_write_field(core, FLD_RUN_STALL, 1);      /* RUN_STALL pull up to avoid fake cmd */
		vpu_trace_end();
		if (ret) {
			LOG_ERR("[vpu_%d] pr_load_algo timeout , status(%d/%d)\n", core,
				vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
			vpu_dump_mesg(NULL);
			vpu_dump_register(NULL);
			vpu_dump_debug_stack(core, DEBUG_STACK_SIZE);
			vpu_dump_code_segment(core, DEBUG_CODE_SEG_SIZE);
			vpu_dump_algo_segment(core, request->algo_id[core], 0x0);
			vpu_aee("VPU Timeout", "core_%d timeout to do loader, algo_id=%d\n", core,
				vpu_service_cores[core].current_algo);
			goto out;
		}

		/* 4. update the id of loaded algo */
		vpu_service_cores[core].current_algo = algo->id[core];
		unlock_command(core);
		vpu_trace_end();
	}

	/* d2d operation */
	vpu_trace_begin("[vpu_%d] hw_processing_request(%d)", core, request->algo_id[core]);
	LOG_DBG("start to enque request\n");
	lock_command(core);
	ret = vpu_check_precond(core);
	if (ret) {
		request->status = VPU_REQ_STATUS_BUSY;
		LOG_ERR("error state before enque request!\n");
		goto out;
	}

	memcpy((void *) (uintptr_t)vpu_service_cores[core].work_buf->va, request->buffers,
			sizeof(struct vpu_buffer) * request->buffer_count);

	if (g_vpu_log_level > 4)
		vpu_dump_buffer_mva(request);
	LOG_INF("[vpu_%d] start d2d, id/frm (%d/%d), bw(%d), algo(%d->%d, %d)\n", core,
		request->algo_id[core], request->frame_magic,
		request->power_param.bw,
		vpu_service_cores[core].current_algo, request->algo_id[core], need_reload);
	/* 1. write register */
	/* command: d2d */
	vpu_write_field(core, FLD_XTENSA_INFO01, VPU_CMD_DO_D2D);
	/* buffer count */
	vpu_write_field(core, FLD_XTENSA_INFO12, request->buffer_count);
	/* pointer to array of struct vpu_buffer */
	vpu_write_field(core, FLD_XTENSA_INFO13, vpu_service_cores[core].work_buf->pa);
	/* pointer to property buffer */
	vpu_write_field(core, FLD_XTENSA_INFO14, request->sett_ptr);
	/* size of property buffer */
	vpu_write_field(core, FLD_XTENSA_INFO15, request->sett_length);

	/* 2. trigger interrupt */
	#ifdef ENABLE_PMQOS
	/* pmqos, 10880 Mbytes per second */
	pm_qos_update_request(&vpu_qos_bw_request, request->power_param.bw);/*max 10880*/
	#endif
	vpu_trace_begin("[vpu_%d] dsp:d2d running", core);
	LOG_DBG("[vpu_%d] d2d running...\n", core);
	#if defined(VPU_MET_READY)
	MET_Events_Trace(1, core, request->algo_id[core]);
	#endif
	vpu_write_field(core, FLD_RUN_STALL, 0);      /* RUN_STALL pull down */
	vpu_write_field(core, FLD_CTL_INT, 1);

	/* 3. wait until done */
	ret = wait_command(core);
	if (g_vpu_log_level > 1)
		LOG_INF("[vpu_%d] end d2d\n", core);
	vpu_write_field(core, FLD_RUN_STALL, 1);      /* RUN_STALL pull up to avoid fake cmd */
	#ifdef ENABLE_PMQOS
	/* pmqos, release request after d2d done */
	pm_qos_update_request(&vpu_qos_bw_request, PM_QOS_DEFAULT_VALUE);
	#endif
	vpu_trace_end();
	#if defined(VPU_MET_READY)
	MET_Events_Trace(0, core, request->algo_id[core]);
	#endif
	LOG_DBG("[vpu_%d] pr hw_d2d test , status(%d/%d)\n", core,
			vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
	if (ret) {
		request->status = VPU_REQ_STATUS_TIMEOUT;
		LOG_ERR("[vpu_%d] pr hw_d2d timeout , status(%d/%d)\n", core,
			vpu_read_field(core, FLD_XTENSA_INFO00), vpu_service_cores[core].is_cmd_done);
		vpu_dump_buffer_mva(request);
		vpu_dump_mesg(NULL);
		vpu_dump_register(NULL);
		vpu_dump_debug_stack(core, DEBUG_STACK_SIZE);
		vpu_dump_code_segment(core, DEBUG_CODE_SEG_SIZE);
		vpu_dump_algo_segment(core, request->algo_id[core], 0x0);
		vpu_aee("VPU Timeout", "core_%d timeout to do d2d, algo_id=%d\n", core,
			vpu_service_cores[core].current_algo);
		goto out;
	}

	request->status = (vpu_check_postcond(core)) ? VPU_REQ_STATUS_FAILURE : VPU_REQ_STATUS_SUCCESS;

out:
	unlock_command(core);
	vpu_trace_end();
out2:
	vpu_put_power(core, VPT_ENQUE_ON);
	LOG_DBG("[vpu] vpu_hw_processing_request - (%d)", request->status);
	return ret;

}


int vpu_hw_get_algo_info(int core, struct vpu_algo *algo)
{
	int ret = 0;
	int port_count = 0;
	int info_desc_count = 0;
	int sett_desc_count = 0;
	unsigned int ofs_ports, ofs_info, ofs_info_descs, ofs_sett_descs;
	int i;

	vpu_trace_begin("vpu_hw_get_algo_info(%d)", algo->id[core]);
	ret = vpu_get_power(core);
	CHECK_RET("[vpu_%d]fail to get power!\n", core);

	lock_command(core);
	LOG_DBG("start to get algo, algo_id=%d\n", algo->id[core]);

	ret = vpu_check_precond(core);
	CHECK_RET("[vpu_%d]have wrong status before get algo!\n", core);

	ofs_ports = 0;
	ofs_info = sizeof(((struct vpu_algo *)0)->ports);
	ofs_info_descs = ofs_info + algo->info_length;
	ofs_sett_descs = ofs_info_descs + sizeof(((struct vpu_algo *)0)->info_descs);
	LOG_INF("[vpu_%d] vpu_hw_get_algo_info check precond done\n", core);

	/* 1. write register */
	vpu_write_field(core, FLD_XTENSA_INFO01, VPU_CMD_GET_ALGO);   /* command: get algo */
	vpu_write_field(core, FLD_XTENSA_INFO06, vpu_service_cores[core].work_buf->pa + ofs_ports);
	vpu_write_field(core, FLD_XTENSA_INFO07, vpu_service_cores[core].work_buf->pa + ofs_info);
	vpu_write_field(core, FLD_XTENSA_INFO08, algo->info_length);
	vpu_write_field(core, FLD_XTENSA_INFO10, vpu_service_cores[core].work_buf->pa + ofs_info_descs);
	vpu_write_field(core, FLD_XTENSA_INFO12, vpu_service_cores[core].work_buf->pa + ofs_sett_descs);

	/* 2. trigger interrupt */
	vpu_trace_begin("dsp:running");
	LOG_DBG("[vpu] vpu_hw_get_algo_info running...\n");
	vpu_write_field(core, FLD_RUN_STALL, 0);      /* RUN_STALL pull down */
	vpu_write_field(core, FLD_CTL_INT, 1);

	/* 3. wait until done */
	ret = wait_command(core);
	LOG_INF("[vpu_%d] VPU_CMD_GET_ALGO done\n", core);
	vpu_trace_end();
	if (ret) {
		vpu_dump_mesg(NULL);
		vpu_dump_register(NULL);
		vpu_dump_debug_stack(core, DEBUG_STACK_SIZE);
		vpu_dump_code_segment(core, DEBUG_CODE_SEG_SIZE);
		vpu_aee("VPU Timeout", "core_%d timeout to get algo, algo_id=%d\n", core,
			vpu_service_cores[core].current_algo);
		goto out;
	}
	vpu_write_field(core, FLD_RUN_STALL, 1);      /* RUN_STALL pull up to avoid fake cmd */

	/* 4. get the return value */
	port_count = vpu_read_field(core, FLD_XTENSA_INFO05);
	info_desc_count = vpu_read_field(core, FLD_XTENSA_INFO09);
	sett_desc_count = vpu_read_field(core, FLD_XTENSA_INFO11);
	algo->port_count = port_count;
	algo->info_desc_count = info_desc_count;
	algo->sett_desc_count = sett_desc_count;

	LOG_DBG("end of get algo, port_count=%d, info_desc_count=%d, sett_desc_count=%d\n",
		port_count, info_desc_count, sett_desc_count);

	/* 5. write back data from working buffer */
	memcpy((void *)(uintptr_t)algo->ports, (void *)((uintptr_t)vpu_service_cores[core].work_buf->va + ofs_ports),
			sizeof(struct vpu_port) * port_count);

	for (i = 0 ; i < algo->port_count ; i++) {
		LOG_DBG("port %d.. id=%d, name=%s, dir=%d, usage=%d\n",
			i, algo->ports[i].id, algo->ports[i].name, algo->ports[i].dir, algo->ports[i].usage);
	}

	memcpy((void *)(uintptr_t)algo->info_ptr, (void *)((uintptr_t)vpu_service_cores[core].work_buf->va + ofs_info),
			algo->info_length);
	memcpy((void *)(uintptr_t)algo->info_descs,
			(void *)((uintptr_t)vpu_service_cores[core].work_buf->va + ofs_info_descs),
			sizeof(struct vpu_prop_desc) * info_desc_count);
	memcpy((void *)(uintptr_t)algo->sett_descs,
			(void *)((uintptr_t)vpu_service_cores[core].work_buf->va + ofs_sett_descs),
			sizeof(struct vpu_prop_desc) * sett_desc_count);

	LOG_DBG("end of get algo 2, port_count=%d, info_desc_count=%d, sett_desc_count=%d\n",
		algo->port_count, algo->info_desc_count, algo->sett_desc_count);

out:
	unlock_command(core);
	vpu_put_power(core, VPT_ENQUE_ON);
	vpu_trace_end();
	LOG_DBG("[vpu] vpu_hw_get_algo_info -\n");
	return ret;
}

void vpu_hw_lock(struct vpu_user *user)
{
	/* CHRISTODO */
	int TEMP_CORE = 0;

	if (user->locked)
		LOG_ERR("double locking bug, pid=%d, tid=%d\n",
				user->open_pid, user->open_tgid);
	else {
		mutex_lock(&lock_mutex);
		is_locked = true;
		user->locked = true;
		vpu_get_power(TEMP_CORE);
	}
}

void vpu_hw_unlock(struct vpu_user *user)
{
	/* CHRISTODO */
	int TEMP_CORE = 0;

	if (user->locked) {
		vpu_put_power(TEMP_CORE, VPT_ENQUE_ON);
		is_locked = false;
		user->locked = false;
		wake_up_interruptible(&lock_wait);
		mutex_unlock(&lock_mutex);
	} else
		LOG_ERR("should not unlock while unlocked, pid=%d, tid=%d\n",
				user->open_pid, user->open_tgid);
}

int vpu_alloc_shared_memory(struct vpu_shared_memory **shmem, struct vpu_shared_memory_param *param)
{
	int ret = 0;

	/* CHRISTODO */
	struct ion_mm_data mm_data;
	struct ion_sys_data sys_data;
	struct ion_handle *handle = NULL;

	*shmem = kzalloc(sizeof(struct vpu_shared_memory), GFP_KERNEL);
	ret = (*shmem == NULL);
	CHECK_RET("fail to kzalloc 'struct memory'!\n");

	handle = ion_alloc(ion_client, param->size, 0, ION_HEAP_MULTIMEDIA_MASK, 0);
	ret = (handle == NULL) ? -ENOMEM : 0;
	CHECK_RET("fail to alloc ion buffer, ret=%d\n", ret);
	(*shmem)->handle = (void *) handle;

	mm_data.mm_cmd = ION_MM_CONFIG_BUFFER_EXT;
	mm_data.config_buffer_param.kernel_handle = handle;
	mm_data.config_buffer_param.module_id = VPU_PORT_OF_IOMMU;
	mm_data.config_buffer_param.security = 0;
	mm_data.config_buffer_param.coherent = 1;
	if (param->fixed_addr) {
		mm_data.config_buffer_param.reserve_iova_start = param->fixed_addr;
		mm_data.config_buffer_param.reserve_iova_end = IOMMU_VA_END; /*param->fixed_addr;*/
	} else {
		/* CHRISTODO, need revise starting address for working buffer*/
		mm_data.config_buffer_param.reserve_iova_start = 0x60000000; /*IOMMU_VA_START*/
		mm_data.config_buffer_param.reserve_iova_end = IOMMU_VA_END;
	}
	ret = ion_kernel_ioctl(ion_client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data);
	CHECK_RET("fail to config ion buffer, ret=%d\n", ret);

	/* map pa */
	LOG_DBG("vpu param->require_pa(%d)\n", param->require_pa);
	if (param->require_pa) {
		sys_data.sys_cmd = ION_SYS_GET_PHYS;
		sys_data.get_phys_param.kernel_handle = handle;
		sys_data.get_phys_param.phy_addr = VPU_PORT_OF_IOMMU << 24 | ION_FLAG_GET_FIXED_PHYS;
		sys_data.get_phys_param.len = ION_FLAG_GET_FIXED_PHYS;
		ret = ion_kernel_ioctl(ion_client, ION_CMD_SYSTEM, (unsigned long)&sys_data);
		CHECK_RET("fail to get ion phys, ret=%d\n", ret);
		(*shmem)->pa = sys_data.get_phys_param.phy_addr;
		(*shmem)->length = sys_data.get_phys_param.len;
	}

	/* map va */
	if (param->require_va) {
		(*shmem)->va = (uint64_t)(uintptr_t)ion_map_kernel(ion_client, handle);
		ret = ((*shmem)->va) ? 0 : -ENOMEM;
		CHECK_RET("fail to map va of buffer!\n");
	}

	return 0;

out:
	if (handle)
		ion_free(ion_client, handle);

	if (*shmem) {
		kfree(*shmem);
		*shmem = NULL;
	}

	return ret;
}

void vpu_free_shared_memory(struct vpu_shared_memory *shmem)
{
	struct ion_handle *handle;

	if (shmem == NULL)
		return;

	handle = (struct ion_handle *) shmem->handle;
	if (handle) {
		ion_unmap_kernel(ion_client, handle);
		ion_free(ion_client, handle);
	}

	kfree(shmem);
}

int vpu_dump_buffer_mva(struct vpu_request *request)
{
	struct vpu_buffer *buf;
	struct vpu_plane *plane;
	int i, j;

	LOG_DBG("dump request - setting: 0x%x, length: %d\n",
			(uint32_t) request->sett_ptr, request->sett_length);

	for (i = 0; i < request->buffer_count; i++) {
		buf = &request->buffers[i];
		LOG_DBG("  buffer[%d] - port: %d, size: %dx%d, format: %d\n",
				i, buf->port_id, buf->width, buf->height, buf->format);

		for (j = 0; j < buf->plane_count; j++) {
			plane = &buf->planes[j];
			LOG_DBG("	 plane[%d] - ptr: 0x%x, length: %d, stride: %d\n",
					j, (uint32_t) plane->ptr, plane->length, plane->stride);
		}
	}

	return 0;

}

int vpu_dump_register(struct seq_file *s)
{
	int i, j;
	bool first_row_of_field;
	struct vpu_reg_desc *reg;
	struct vpu_reg_field_desc *field;
	int TEMP_CORE = 0;

#define LINE_BAR "  +---------------+-------+---+---+-------------------------+----------+\n"

	for (TEMP_CORE = 0; TEMP_CORE < MTK_VPU_CORE; TEMP_CORE++) {
		vpu_print_seq(s, "  |Core: %-62d|\n", TEMP_CORE);
		vpu_print_seq(s, LINE_BAR);
		vpu_print_seq(s, "  |%-15s|%-7s|%-3s|%-3s|%-25s|%-10s|\n",
					  "Register", "Offset", "MSB", "LSB", "Field", "Value");
		vpu_print_seq(s, LINE_BAR);


		for (i = 0; i < VPU_NUM_REGS; i++) {
			reg = &g_vpu_reg_descs[i];
#ifndef MTK_VPU_DVT
			if (reg->reg < REG_DEBUG_INFO00)
				continue;
#endif
			first_row_of_field = true;

			for (j = 0; j < VPU_NUM_REG_FIELDS; j++) {
				field = &g_vpu_reg_field_descs[j];
				if (reg->reg != field->reg)
					continue;

				if (first_row_of_field) {
					first_row_of_field = false;
					vpu_print_seq(s, "  |%-15s|0x%-5.5x|%-3d|%-3d|%-25s|0x%-8.8x|\n",
								  reg->name,
								  reg->offset,
								  field->msb,
								  field->lsb,
								  field->name,
								  vpu_read_field(TEMP_CORE, (enum vpu_reg_field) j));
				} else {
					vpu_print_seq(s, "  |%-15s|%-7s|%-3d|%-3d|%-25s|0x%-8.8x|\n",
								  "", "",
								  field->msb,
								  field->lsb,
								  field->name,
								  vpu_read_field(TEMP_CORE, (enum vpu_reg_field) j));
				}
			}
			vpu_print_seq(s, LINE_BAR);
		}
	}
#undef LINE_BAR

	return 0;
}

int vpu_dump_image_file(struct seq_file *s)
{
	int i, j, id = 1;
	struct vpu_algo_info *algo_info;
	struct vpu_image_header *header;
	int core = 0;

#define LINE_BAR "  +------+-----+--------------------------------+--------+-----------+----------+\n"
	vpu_print_seq(s, LINE_BAR);
	vpu_print_seq(s, "  |%-6s|%-5s|%-32s|%-8s|%-11s|%-10s|\n",
				  "Header", "Id", "Name", "MagicNum", "MVA", "Length");
	vpu_print_seq(s, LINE_BAR);

	header = (struct vpu_image_header *) ((uintptr_t)vpu_service_cores[core].bin_base + (VPU_OFFSET_IMAGE_HEADERS));
	for (i = 0; i < VPU_NUMS_IMAGE_HEADER; i++) {
		#if 0
		header = (struct vpu_image_header *) ((uintptr_t)vpu_service_cores[core].bin_base +
			(VPU_OFFSET_IMAGE_HEADERS) + sizeof(struct vpu_image_header) * i);
		#endif
		for (j = 0; j < header[i].algo_info_count; j++) {
			algo_info = &header[i].algo_infos[j];

			vpu_print_seq(s, "  |%-6d|%-5d|%-32s|0x%-6lx|0x%-9lx|0x%-8x|\n",
						  (i + 1),
						  id,
						  algo_info->name,
						  (unsigned long)(algo_info->vpu_core),
						  algo_info->offset - VPU_OFFSET_ALGO_AREA +
						  (uintptr_t)vpu_service_cores[core].algo_data_mva,
						  algo_info->length);
			id++;
		}
	}

	vpu_print_seq(s, LINE_BAR);
#undef LINE_BAR

/* #ifdef MTK_VPU_DUMP_BINARY */
#if 0
	{
		uint32_t dump_1k_size = (0x00000400);
		unsigned char *ptr = NULL;

		vpu_print_seq(s, "Reset Vector Data:\n");
		ptr = (unsigned char *) vpu_service_cores[core].bin_base + VPU_OFFSET_RESET_VECTOR;
		for (i = 0; i < dump_1k_size / 2; i++, ptr++) {
			if (i % 16 == 0)
				vpu_print_seq(s, "\n%07X0h: ", i / 16);

			vpu_print_seq(s, "%02X ", *ptr);
		}
		vpu_print_seq(s, "\n");
		vpu_print_seq(s, "\n");
		vpu_print_seq(s, "Main Program Data:\n");
		ptr = (unsigned char *) vpu_service_cores[core].bin_base + VPU_OFFSET_MAIN_PROGRAM;
		for (i = 0; i < dump_1k_size; i++, ptr++) {
			if (i % 16 == 0)
				vpu_print_seq(s, "\n%07X0h: ", i / 16);

			vpu_print_seq(s, "%02X ", *ptr);
		}
		vpu_print_seq(s, "\n");
	}
#endif

	return 0;
}

void vpu_dump_debug_stack(int core, int size)
{
	int i = 0;
	unsigned int vpu_domain_addr = 0x0;

	vpu_domain_addr = ((DEBUG_STACK_BASE_OFFSET & 0x000fffff) | 0x7FF00000);
	LOG_ERR("==============vpu_dump_debug_stack, core_%d==============\n", core);
	LOG_WRN("==============0x%x/0x%x/0x%x==============\n", vpu_domain_addr,
		DEBUG_STACK_BASE_OFFSET, DEBUG_STACK_SIZE);
	for (i = 0 ; i < (int)size / 4 ; i = i + 4) {
		#if 1
		LOG_WRN("%08X %08X %08X %08X %08X\n", vpu_domain_addr,
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i)),
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i + 4)),
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i + 8)),
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i + 12)));
		#else
		LOG_WRN("%X %X %X %X %X , 0x%X/0x%X/0x%X/0x%X\n", vpu_domain_addr,
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i)),
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i + 4)),
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i + 8)),
			vpu_read_reg32(vpu_service_cores[core].vpu_base,
			DEBUG_STACK_BASE_OFFSET + (4 * i + 12)),
			DEBUG_STACK_BASE_OFFSET + (4 * i),
			DEBUG_STACK_BASE_OFFSET + (4 * i + 4),
			DEBUG_STACK_BASE_OFFSET + (4 * i + 8),
			DEBUG_STACK_BASE_OFFSET + (4 * i + 12));
		#endif
		vpu_domain_addr += (4 * 4);
	}
}

void vpu_dump_code_segment(int core, int size)
{
	int i = 0;
	unsigned long addr = 0x0;
	unsigned int dump_addr = 0x0;
	unsigned int value_1, value_2, value_3, value_4;

	switch (core) {
	case 0:
	default:
		dump_addr = VPU_MVA_KERNEL_LIB;
		break;
	case 1:
		dump_addr = VPU2_MVA_KERNEL_LIB;
		break;
	}
	addr = (unsigned long)(vpu_service_cores[core].exec_kernel_lib->va);
	LOG_ERR("==============vpu_dump_code_segment, core_%d==============\n", core);
	LOG_WRN("==============0x%lx/0x%x/0x%x/0x%x==============\n", addr, dump_addr,
		size, DEBUG_CODE_SEG_SIZE);
	for (i = 0 ; i < (int)size / 4 ; i = i + 4) {
		value_1 = (unsigned int)(*((unsigned long *)((uintptr_t)addr + (4 * i))));
		value_2 = (unsigned int)(*((unsigned long *)((uintptr_t)addr + (4 * i + 4))));
		value_3 = (unsigned int)(*((unsigned long *)((uintptr_t)addr + (4 * i + 8))));
		value_4 = (unsigned int)(*((unsigned long *)((uintptr_t)addr + (4 * i + 12))));
		LOG_WRN("%08X %08X %08X %08X %08X\n", dump_addr,
			value_1, value_2, value_3, value_4);
		dump_addr += (4 * 4);
	}
}

void vpu_dump_algo_segment(int core, int algo_id, int size)
{
/* we do not mapping out va(bin_ptr is mva) for algo bin file currently */
#if 0
	unsigned int addr = 0x0;
	unsigned int length = 0x0;
	int i = 0;
	struct vpu_algo *algo = NULL;
	int ret = 0;

	LOG_ERR("==============vpu_dump_algo_segment, core_%d, id_%d==============\n",
		core, algo_id);
	ret = vpu_find_algo_by_id(core, algo_id, &algo);
	if (ret) {
		LOG_ERR("vpu_dump_algo_segment can not find the algo, core=%d, id=%d\n",
				core, algo_id);
	} else {
		addr = (unsigned int)algo->bin_ptr;
		length = (unsigned int)algo->bin_length;

		LOG_WRN("==============0x%x/0x%x/0x%x==============\n", addr, length, size);
		for (i = 0 ; i < (int)length / 4 ; i = i + 4) {
			LOG_WRN("%X %X %X %X %X\n", addr,
				(unsigned int)(*((unsigned int *)((uintptr_t)addr + (4 * i)))),
				(unsigned int)(*((unsigned int *)((uintptr_t)addr + (4 * i + 4)))),
				(unsigned int)(*((unsigned int *)((uintptr_t)addr + (4 * i + 8)))),
				(unsigned int)(*((unsigned int *)((uintptr_t)addr + (4 * i + 12)))));
			addr += (4 * 4);
		}
	}
#endif
}

int vpu_dump_mesg(struct seq_file *s)
{
	char *ptr = NULL;
	char *log_head = NULL;
	char *log_buf;
	char *log_a_pos = NULL;
	int core_index = 0;
	bool jump_out = false;

	for (core_index = 0 ; core_index < MTK_VPU_CORE; core_index++) {
		log_buf = (char *) ((uintptr_t)vpu_service_cores[core_index].work_buf->va + VPU_OFFSET_LOG);
	if (g_vpu_log_level > 8) {
		int i = 0;
		int line_pos = 0;
		char line_buffer[16 + 1] = {0};

		ptr = log_buf;
		vpu_print_seq(s, "VPU_%d Log Buffer:\n", core_index);
		for (i = 0; i < VPU_SIZE_LOG_BUF; i++, ptr++) {
			line_pos = i % 16;
			if (line_pos == 0)
				vpu_print_seq(s, "\n%07X0h: ", i / 16);

			line_buffer[line_pos] = isascii(*ptr) && isprint(*ptr) ? *ptr : '.';
			vpu_print_seq(s, "%02X ", *ptr);
			if (line_pos == 15)
				vpu_print_seq(s, " %s", line_buffer);

		}
		vpu_print_seq(s, "\n\n");
	}

	ptr = log_buf;
	log_head = log_buf;

	/* set the last byte to '\0' */
	#if 0
	*(ptr + VPU_SIZE_LOG_BUF - 1) = '\0';

	/* skip the header part */
	ptr += VPU_SIZE_LOG_HEADER;
	log_head = strchr(ptr, '\0') + 1;

		vpu_print_seq(s, "=== VPU_%d Log Buffer ===\n", core_index);
	vpu_print_seq(s, "vpu: print dsp log\n%s%s", log_head, ptr);
	#else
	vpu_print_seq(s, "=== VPU_%d Log Buffer ===\n", core_index);
	vpu_print_seq(s, "vpu: print dsp log (0x%x):\n", (unsigned int)(uintptr_t)log_buf);

    /* in case total log < VPU_SIZE_LOG_SHIFT and there's '\0' */
	*(log_head + VPU_SIZE_LOG_BUF - 1) = '\0';
	vpu_print_seq(s, "%s", ptr+VPU_SIZE_LOG_HEADER);

	ptr += VPU_SIZE_LOG_HEADER;
	log_head = ptr;

	jump_out = false;
	*(log_head + (VPU_SIZE_LOG_BUF - VPU_SIZE_LOG_HEADER) - 1) = '\n';
	do {
		if ((ptr + VPU_SIZE_LOG_SHIFT) >= (log_head + (VPU_SIZE_LOG_BUF - VPU_SIZE_LOG_HEADER))) {
			*(log_head + (VPU_SIZE_LOG_BUF - VPU_SIZE_LOG_HEADER) - 1) = '\0'; /* last part of log buffer */
			jump_out = true;
		} else {
			log_a_pos = strchr(ptr + VPU_SIZE_LOG_SHIFT, '\n');
			if (log_a_pos == NULL)
				break;
			*log_a_pos = '\0';
		}
		vpu_print_seq(s, "%s\n", ptr);
		ptr = log_a_pos + 1;

		/* incase log_a_pos is at end of string */
		if (ptr >= log_head + (VPU_SIZE_LOG_BUF - VPU_SIZE_LOG_HEADER))
			break;
	} while (!jump_out);

	#endif
	}
	return 0;
}

int vpu_dump_opp_table(struct seq_file *s)
{
	int i;

#define LINE_BAR "  +-----+----------+----------+------------+-----------+-----------+\n"
	vpu_print_seq(s, LINE_BAR);
	vpu_print_seq(s, "  |%-5s|%-10s|%-10s|%-12s|%-11s|%-11s|\n",
				  "OPP", "VCORE(uV)", "DSP(KHz)", "IPU_IF(KHz)", "DSP1(KHz)", "DSP2(KHz)");
	vpu_print_seq(s, LINE_BAR);

	for (i = 0; i < opps.count; i++) {
		vpu_print_seq(s, "  |%-5d|[%d]%-7d|[%d]%-7d|[%d]%-9d|[%d]%-8d|[%d]%-8d|\n",
				i,
				opps.vcore.opp_map[i],  opps.vcore.values[opps.vcore.opp_map[i]],
				opps.dsp.opp_map[i],    opps.dsp.values[opps.dsp.opp_map[i]],
				opps.ipu_if.opp_map[i], opps.ipu_if.values[opps.ipu_if.opp_map[i]],
				opps.dspcore[0].opp_map[i], opps.dspcore[0].values[opps.dspcore[0].opp_map[i]],
				opps.dspcore[1].opp_map[i], opps.dspcore[1].values[opps.dspcore[1].opp_map[i]]);
	}

	vpu_print_seq(s, LINE_BAR);
#undef LINE_BAR

	return 0;
}

int vpu_dump_power(struct seq_file *s)
{
	int hw_vcore = 0, vcore_opp = 0;

	hw_vcore = vcorefs_get_curr_vcore();
	if (hw_vcore >= 800000)
		vcore_opp = 0;
	else
		vcore_opp = 1;

	vpu_print_seq(s, "dvfs_debug(rw): vore[%d/%d], dsp[%d], ipu_if[%d], dsp1[%d], dsp2[%d]\n",
			opps.vcore.index,
			vcore_opp,
			opps.dsp.index,
			opps.ipu_if.index,
			opps.dspcore[0].index,
			opps.dspcore[1].index);
	vpu_print_seq(s, "is_power_debug_lock(rw): %d\n", is_power_debug_lock);

	return 0;
}

int vpu_dump_vpu(struct seq_file *s)
{
	int core;

#define LINE_BAR "  +-------------+------+-------+-------+-------+-------+\n"
	vpu_print_seq(s, LINE_BAR);
	vpu_print_seq(s, "  |%-12s|%-34s|\n",
			"Queue#", "Waiting");
	vpu_print_seq(s, LINE_BAR);

	mutex_lock(&vpu_dev->commonpool_mutex);
	vpu_print_seq(s, "  |%-12s|%-34d|\n",
			      "Common",
			      vpu_dev->commonpool_list_size);
	mutex_unlock(&vpu_dev->commonpool_mutex);

	for (core = 0 ; core < MTK_VPU_CORE; core++) {
		mutex_lock(&vpu_dev->servicepool_mutex[core]);
		vpu_print_seq(s, "  |Core %-7d|%-34d|\n",
				      core,
				      vpu_dev->servicepool_list_size[core]);
		mutex_unlock(&vpu_dev->servicepool_mutex[core]);
	}
	vpu_print_seq(s, "\n");

#undef LINE_BAR

	return 0;
}

int vpu_set_power_parameter(uint8_t param, int argc, int *args)
{
	int ret = 0;

	switch (param) {
	case VPU_POWER_PARAM_FIX_OPP:
		ret = (argc == 1) ? 0 : -EINVAL;
		CHECK_RET("invalid argument, expected:1, received:%d\n", argc);

		switch (args[0]) {
		case 0:
			is_power_debug_lock = false;
			break;
		case 1:
			is_power_debug_lock = true;
			break;
		default:
			CHECK_RET("invalid argument, received:%d\n", (int)(args[0]));
			ret = -EINVAL;
			goto out;
		}
		break;
	case VPU_POWER_PARAM_DVFS_DEBUG:
		ret = (argc == 1) ? 0 : -EINVAL;
		CHECK_RET("invalid argument, expected:1, received:%d\n", argc);

		ret = args[0] >= opps.count;
		CHECK_RET("opp step(%d) is out-of-bound, count:%d\n", (int)(args[0]), opps.count);

		opps.vcore.index = opps.vcore.opp_map[args[0]];
		opps.dsp.index = opps.dsp.opp_map[args[0]];
		opps.ipu_if.index = opps.ipu_if.opp_map[args[0]];
		opps.dspcore[0].index = opps.dspcore[0].opp_map[args[0]];
		opps.dspcore[1].index = opps.dspcore[1].opp_map[args[0]];

		is_power_debug_lock = true;

		break;
	case VPU_POWER_PARAM_JTAG:
		ret = (argc == 1) ? 0 : -EINVAL;
		CHECK_RET("invalid argument, expected:1, received:%d\n", argc);

		is_jtag_enabled = args[0];
		ret = vpu_hw_enable_jtag(is_jtag_enabled);

		break;
	case VPU_POWER_PARAM_LOCK:
		ret = (argc == 1) ? 0 : -EINVAL;
		CHECK_RET("invalid argument, expected:1, received:%d\n", argc);

		is_power_debug_lock = args[0];

		break;
	default:
		LOG_ERR("unsupport the power parameter:%d\n", param);
		break;
	}

out:
	return ret;
}
