/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <mt-plat/aee.h>
#include <linux/debugfs.h>
#include <linux/sort.h>
#include <linux/string.h>


#include <fpsgo_common.h>
#include <trace/events/fpsgo.h>
#include <mach/mtk_ppm_api.h>

#include <mtk_vcorefs_governor.h>
#include <mtk_vcorefs_manager.h>

#include "eas_controller.h"

#include "fbt_error.h"
#include "fbt_base.h"
#include "fbt_usedext.h"
#include "fbt_notifier.h"
#include "fbt_cpu.h"
#include "../fstb/fstb.h"
#include "xgf.h"

#define Target_fps_30_NS 33333333
#define Target_fps_60_NS 16666666
#define GED_VSYNC_MISS_QUANTUM_NS 16666666
#define TIME_8MS  8000000
#define TIME_5MS  5000000
#define TIME_3MS  3000000
#define TIME_1MS  1000000
#define WINDOW 20


#define FBTCPU_SEC_DIVIDER 1000000000
#define TARGET_UNLIMITED_FPS 60
#define RESET_TOLERENCE 2

static int bhr;
static int bhr_opp;
static int target_cluster;
static int cluster_freq_bound;
static int cluster_rescue_bound;
static int migrate_bound;
static int vsync_percent;
static int rescue_opp_f;
static int rescue_opp_c;
static int rescue_percent;
static int vsync_period;
static int deqtime_bound;
static int variance;
static int floor_bound;
static int kmin;
static int floor_opp;

module_param(bhr, int, S_IRUGO|S_IWUSR);
module_param(bhr_opp, int, S_IRUGO|S_IWUSR);
module_param(target_cluster, int, S_IRUGO|S_IWUSR);
module_param(cluster_freq_bound, int, S_IRUGO|S_IWUSR);
module_param(migrate_bound, int, S_IRUGO|S_IWUSR);
module_param(vsync_percent, int, S_IRUGO|S_IWUSR);
module_param(rescue_opp_f, int, S_IRUGO|S_IWUSR);
module_param(rescue_opp_c, int, S_IRUGO|S_IWUSR);
module_param(rescue_percent, int, S_IRUGO|S_IWUSR);
module_param(vsync_period, int, S_IRUGO|S_IWUSR);
module_param(deqtime_bound, int, S_IRUGO|S_IWUSR);
module_param(variance, int, S_IRUGO|S_IWUSR);
module_param(floor_bound, int, S_IRUGO|S_IWUSR);
module_param(kmin, int, S_IRUGO|S_IWUSR);
module_param(floor_opp, int, S_IRUGO|S_IWUSR);
module_param(cluster_rescue_bound, int, S_IRUGO|S_IWUSR);

static unsigned long long base_blc;
static unsigned int base_freq;

static unsigned int vsync_distance;
static unsigned int rescue_distance;

static atomic_t loading_cur;

static unsigned last_obv;
static unsigned long long last_cb_ts;

static unsigned clus_obv[2];
static int dequeuebuffer;
static unsigned long long vsync_time;
static unsigned long long deqend_time;
static unsigned long long asfc_time;
static unsigned long long asfc_last_fps;

static int history_blc_count;

static DEFINE_SPINLOCK(xgf_slock);
static DEFINE_MUTEX(xgf_mlock);

static int power_ll[16], power_l[16];

static unsigned int _gdfrc_fps_limit;
static unsigned int _gdfrc_fps_limit_ex;
static int _gdfrc_cpu_target;

static struct fbt_proc proc;

static unsigned long long q2q_time;
static unsigned long long deq_time;
static int sf_check;
static int sf_bound;
static int sf_bound_max;
static int sf_bound_min;
static int last_fps;
static int middle_enable;

static int fbt_enable;
static int game_mode;
static int game_mode_hint;

static struct fbt_frame_info frame_info[WINDOW];
static unsigned long long floor;
static int floor_count;
static int reset_floor_bound;
static int f_iter;

static int vag_fps;

static int lpp_max_cap;
static int lpp_fps;
static int lpp_max_freq_LL;
static int lpp_max_freq_L;

void fbt_cpu_vag_set_fps(unsigned int fps)
{
	unsigned long flags;

	spin_lock_irqsave(&xgf_slock, flags);
	vag_fps = fps;
	spin_unlock_irqrestore(&xgf_slock, flags);
	fpsgo_systrace_c_fbt_gm(-100, vag_fps, "vag_fps");
}

void fbt_cpu_lpp_set_fps(unsigned int fps)
{
	unsigned long flags;

	spin_lock_irqsave(&xgf_slock, flags);
	lpp_fps = fps;
	spin_unlock_irqrestore(&xgf_slock, flags);
	fpsgo_systrace_c_fbt_gm(-100, lpp_fps, "lpp_fps");
}

void fbt_cpu_lppcap_update(unsigned int llpcap)
{
	unsigned long flags;

	spin_lock_irqsave(&xgf_slock, flags);

	if (llpcap)
		lpp_max_cap = (int)llpcap;
	else
		lpp_max_cap = 100;

	if (lpp_max_cap < 100) {
		int lpp_max_freq;
		int i;

		lpp_max_freq = (min((int)(lpp_max_cap), 100) * power_l[0]) / 100;

		for (i = 15; i > 0; i--) {
			if (power_ll[i] >= lpp_max_freq)
				break;
		}
		lpp_max_freq_LL = power_ll[min((i+1), 15)];

		for (i = 15; i > 0; i--) {
			if (power_l[i] >= lpp_max_freq)
				break;
		}
		lpp_max_freq_L = power_l[min((i+1), 15)];

	} else {
		lpp_max_freq_LL = power_ll[0];
		lpp_max_freq_L = power_l[0];
	}
	spin_unlock_irqrestore(&xgf_slock, flags);
}

int fbt_cpu_set_bhr(int new_bhr)
{
	unsigned long flags;

	if (new_bhr < 0 || new_bhr > 100)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	bhr = new_bhr;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_bhr_opp(int new_opp)
{
	unsigned long flags;

	if (new_opp < 0)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	bhr_opp = new_opp;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_rescue_opp_c(int new_opp)
{
	unsigned long flags;

	if (new_opp < 0)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	rescue_opp_c = new_opp;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_rescue_opp_f(int new_opp)
{
	unsigned long flags;

	if (new_opp < 0)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	rescue_opp_f = new_opp;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_rescue_percent(int percent)
{
	unsigned long flags;

	if (percent < 0 || percent > 100)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	rescue_percent = percent;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_variance(int var)
{
	unsigned long flags;

	if (var < 0 || var > 100)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	variance = var;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_floor_bound(int bound)
{
	unsigned long flags;

	if (bound < 0 || bound > WINDOW)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	floor_bound = bound;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_floor_kmin(int k)
{
	unsigned long flags;

	if (k < 0 || k > WINDOW)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	kmin = k;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

int fbt_cpu_set_floor_opp(int new_opp)
{
	unsigned long flags;

	if (new_opp < 0)
		return -EINVAL;

	spin_lock_irqsave(&xgf_slock, flags);
	floor_opp = new_opp;
	spin_unlock_irqrestore(&xgf_slock, flags);

	return 0;
}

void xgf_setting_exit(void)
{
	struct ppm_limit_data pld[2];
	unsigned long flags;

	/* clear boost data & loading*/
	spin_lock_irqsave(&xgf_slock, flags);
	atomic_set(&loading_cur, 0);
	last_cb_ts = ged_get_time();
	target_cluster = 2;
	history_blc_count = 0;
	middle_enable = 1;
	floor = 0;
	floor_count = 0;
	reset_floor_bound = 0;
	f_iter = 0;
	q2q_time = 0;
	deq_time = 0;
	sf_check = 0;
	last_fps = 0;
	sf_bound = sf_bound_min;
	memset(frame_info, 0, sizeof(frame_info));
	spin_unlock_irqrestore(&xgf_slock, flags);
	fpsgo_systrace_c_fbt_gm(-100, target_cluster, "target_cluster");
	fpsgo_systrace_c_fbt_gm(-100, history_blc_count, "history_blc_count");
	fpsgo_systrace_c_fbt_gm(-100, atomic_read(&loading_cur), "loading");

	update_eas_boost_value(EAS_KIR_FBC, CGROUP_TA, 0);
	set_idle_prefer(0); /*todo sync?*/
	set_cpuset(-1); /*todo sync?*/
	/*vcorefs_request_dvfs_opp(KIR_FBT, -1);*/

	pld[0].min = -1;
	pld[1].min = -1;
	pld[0].max = -1;
	pld[1].max = -1;
	update_userlimit_cpu_freq(EAS_KIR_FBC, 2, &pld[0]);
}

int switch_fbt_game(int enable)
{
	mutex_lock(&xgf_mlock);
	fbt_enable = enable;
	game_mode = game_mode_hint && fbt_enable;

	if (!fbt_enable && game_mode_hint) {
		xgf_setting_exit();
		ppm_game_mode_change_cb(0);
		xgf_game_mode_exit(!fbt_enable);
	}

	fpsgo_systrace_c_fbt_gm(-100, fbt_enable, "fbt_enable");
	mutex_unlock(&xgf_mlock);
	return 0;
}

void fbt_cpu_set_game_hint_cb(int is_game_mode)
{
	FBT_LOGE("%s game mode", is_game_mode ? "enter" : "exit");

	mutex_lock(&xgf_mlock);
	game_mode_hint = is_game_mode;
	game_mode = game_mode_hint && fbt_enable;

	ppm_game_mode_change_cb(game_mode);
	if (game_mode)
		set_idle_prefer(1);
	else
		xgf_setting_exit();
	mutex_unlock(&xgf_mlock);

}
EXPORT_SYMBOL(fbt_cpu_set_game_hint_cb);

unsigned long long fbt_get_new_base_blc(void)
{
	struct ppm_limit_data pld[2];
	unsigned long long blc_wt;
	int blc_freq, i;
	int opp_L, opp_LL = 0;
	unsigned long flags;
	int move_cluster = 0;

	spin_lock_irqsave(&xgf_slock, flags);

	blc_wt = base_blc;
	blc_freq = base_freq;

	for (i = 15; i > 0; i--)
		if (power_l[i] >= blc_freq)
			break;

	opp_L = i;

	for (i = 15; i > 0; i--)
		if (power_ll[i] >= blc_freq)
			break;

	opp_LL = i;

	if (target_cluster) {
		blc_freq = power_l[max((opp_L - rescue_opp_f), 0)];
		blc_freq = min(blc_freq, lpp_max_freq_L);
		pld[1].max = power_l[max((opp_L - rescue_opp_c - bhr_opp), 0)];
		pld[0].max = power_ll[opp_LL];
	} else {
		blc_freq = power_ll[max((opp_LL - rescue_opp_f), 0)];
		blc_freq = min(blc_freq, lpp_max_freq_LL);
		if (blc_freq > cluster_rescue_bound) {
			target_cluster = 1;
			history_blc_count = 0;
			move_cluster = 1;
			pld[1].max = power_l[max((opp_L - rescue_opp_c), 0)];
			pld[0].max = power_ll[opp_LL];
		} else {
			pld[0].max = power_ll[max((opp_LL - rescue_opp_c), 0)];
			pld[1].max = power_l[opp_L];
		}
	}
	pld[0].min = -1;
	pld[1].min = -1;
	blc_wt = (unsigned long long)(blc_freq * 100) / power_l[0];
	pld[1].max = min(pld[1].max, lpp_max_freq_L);
	pld[0].max = min(pld[0].max, lpp_max_freq_LL);
	fpsgo_systrace_c_log(blc_wt, "perf idx");
	fpsgo_systrace_c_log(target_cluster, "target_cluster");
	spin_unlock_irqrestore(&xgf_slock, flags);

	if (move_cluster)
		set_cpuset(target_cluster);
	update_userlimit_cpu_freq(EAS_KIR_FBC, 2, pld);

	fpsgo_systrace_c_fbt_gm(-100, pld[0].max, "cluster0 limit freq");
	fpsgo_systrace_c_fbt_gm(-100, pld[1].max, "cluster1 limit freq");
	fpsgo_systrace_c_fbt_gm(-100, lpp_max_freq_L, "lpp_max_freq_L");
	fpsgo_systrace_c_fbt_gm(-100, lpp_max_freq_LL, "lpp_max_freq_LL");

	return blc_wt;
}


static void fbt_do_jerk(struct work_struct *work)
{
	struct fbt_jerk *jerk;

	if (target_cluster == 2)
		return;

	jerk = container_of(work, struct fbt_jerk, work);

	if (jerk->id == proc.active_jerk_id) {
		unsigned long long blc_wt;

		blc_wt = fbt_get_new_base_blc();
		if (target_cluster == 1)
			update_eas_boost_value(EAS_KIR_FBC, CGROUP_TA, (blc_wt - 1) + 3200);
		else if (target_cluster == 0)
			update_eas_boost_value(EAS_KIR_FBC, CGROUP_TA, (blc_wt - 1) + 3100);
		xgf_trace("boost jerk %d proc.id %d", jerk->id, proc.active_jerk_id);
	} else
		xgf_trace("skip jerk %d proc.id %d", jerk->id, proc.active_jerk_id);
}

static enum hrtimer_restart fbt_jerk_tfn(struct hrtimer *timer)
{
	struct fbt_jerk *jerk;

	jerk = container_of(timer, struct fbt_jerk, timer);
	schedule_work(&jerk->work);
	return HRTIMER_NORESTART;
}

static inline void fbt_init_jerk(struct fbt_jerk *jerk, int id)
{
	jerk->id = id;

	hrtimer_init(&jerk->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	jerk->timer.function = &fbt_jerk_tfn;
	INIT_WORK(&jerk->work, fbt_do_jerk);
}

void xgf_ged_vsync(void)
{
	if (game_mode) {
		unsigned long flags;

		spin_lock_irqsave(&xgf_slock, flags);
		vsync_time = ged_get_time();
		xgf_trace("vsync_time=%llu", vsync_time);
		spin_unlock_irqrestore(&xgf_slock, flags);
	}
}

void xgf_get_deqend_time(void)
{
	unsigned long flags;

	spin_lock_irqsave(&xgf_slock, flags);
	deqend_time = ged_get_time();
	xgf_trace("deqend_time=%llu", deqend_time);
	spin_unlock_irqrestore(&xgf_slock, flags);
}

void __xgf_cpufreq_cb_normal(int cid, unsigned long freq, int ob)
{
	unsigned curr_obv = 0UL;
	unsigned long long curr_cb_ts;

	curr_cb_ts = ged_get_time();

	curr_obv = (unsigned)(freq * 100) / power_l[0];

	clus_obv[cid] = curr_obv;
	fpsgo_systrace_c_log(curr_obv, "curr_obv[%d]", cid);

	if (last_obv == base_blc) {
		last_obv = clus_obv[target_cluster];
		last_cb_ts = curr_cb_ts;
		return;
	}

	if (curr_cb_ts > last_cb_ts) {
		unsigned dur  = (unsigned)((curr_cb_ts - last_cb_ts) / (unsigned)NSEC_PER_USEC);
		signed diff = last_obv - base_blc;

		xgf_trace("base=%d obv=%u->%u time=%llu->%llu load=%d->%d", base_blc,
			  last_obv, clus_obv[target_cluster],
			  last_cb_ts, curr_cb_ts, loading_cur,
			  atomic_add_return((int)(dur * diff), &loading_cur));
		fpsgo_systrace_c_fbt_gm(-100, atomic_read(&loading_cur), "loading");
	}

	last_obv = clus_obv[target_cluster];
	xgf_trace("keep last_obv=%d ", last_obv);
	last_cb_ts = curr_cb_ts;
}


void __xgf_cpufreq_cb_flush(void)
{
	unsigned long long curr_cb_ts;

	curr_cb_ts = ged_get_time();

	if (last_obv == base_blc) {
		last_obv = 0UL;
		last_cb_ts = curr_cb_ts;
		return;
	}

	if (curr_cb_ts > last_cb_ts) {
		unsigned dur  = (unsigned)((curr_cb_ts - last_cb_ts) / (unsigned)NSEC_PER_USEC);
		unsigned diff = last_obv - base_blc;

		xgf_trace("base=%d obv=%u->%u time=%llu->%llu load=%d", base_blc,
			  last_obv, clus_obv[target_cluster],
			  last_cb_ts, curr_cb_ts,
			  atomic_add_return((int)(dur * diff), &loading_cur));
		fpsgo_systrace_c_fbt_gm(-100, atomic_read(&loading_cur), "loading");
	}

	last_obv = 0UL;
	last_cb_ts = curr_cb_ts;
}

void __xgf_cpufreq_cb_skip(int cid, unsigned long freq, int ob)
{
	unsigned curr_obv = 0UL;
	unsigned long long curr_cb_ts;

	curr_cb_ts = ged_get_time();

	curr_obv = (unsigned)(freq * 100) / power_l[0];

	clus_obv[cid] = curr_obv;
	xgf_trace("skip last_obv=%d ", last_obv);
	fpsgo_systrace_c_log(curr_obv, "curr_obv[%d]", cid);

	last_obv = 0UL;
	last_cb_ts = curr_cb_ts;
}

void xgf_cpufreq_cb(int cid, unsigned long freq)
{
	unsigned long flags;
	signed int boost_min;
	int over_boost;

	if (!game_mode)
		return;

	spin_lock_irqsave(&xgf_slock, flags);

	if (target_cluster == 2) {
		last_cb_ts = ged_get_time();
		clus_obv[cid] = (unsigned)(freq * 100) / power_l[0];
		spin_unlock_irqrestore(&xgf_slock, flags);
		return;
	}

	if (cid != target_cluster) {
		spin_unlock_irqrestore(&xgf_slock, flags);
		return;
	}

	boost_min = min_boost_freq[cid];
	over_boost = (freq > base_freq) ? 1 : 0;
	xgf_trace("over_boost=%d min_boost_freq[%d]=%d freq=%d base_freq=%d",
		over_boost, cid, min_boost_freq[cid], freq, base_freq);

	if (dequeuebuffer == 0)
		__xgf_cpufreq_cb_normal(cid, freq, over_boost);
	else
		__xgf_cpufreq_cb_skip(cid, freq, over_boost);
	spin_unlock_irqrestore(&xgf_slock, flags);
}

void xgf_dequeuebuffer(unsigned long arg)
{
	unsigned long flags;

	if (!game_mode)
		return;

	spin_lock_irqsave(&xgf_slock, flags);

	if (target_cluster == 2) {
		dequeuebuffer = arg;
		spin_unlock_irqrestore(&xgf_slock, flags);
		return;
	}
	if (arg)
		__xgf_cpufreq_cb_flush();
	else {
		xgf_trace("dequeue obv=%u->%u time=%llu->%llu",
			  last_obv, clus_obv[target_cluster],
			  last_cb_ts, ged_get_time());
		last_obv = clus_obv[target_cluster];
		last_cb_ts = ged_get_time();
	}
	dequeuebuffer = arg;

	xgf_trace("dequeuebuffer=%d", dequeuebuffer);
	fpsgo_systrace_c_fbt_gm(-100, last_obv, "deq_obv");
	spin_unlock_irqrestore(&xgf_slock, flags);

	fpsgo_systrace_c_fbt_gm(-100, arg, "dequeueBuffer");
}
EXPORT_SYMBOL(xgf_dequeuebuffer);

void update_pwd_tbl(void)
{
	int i;

	for (i = 0; i < 16; i++)
		power_ll[i] = mt_cpufreq_get_freq_by_idx(0, i);

	for (i = 0; i < 16; i++)
		power_l[i] = mt_cpufreq_get_freq_by_idx(1, i);
}

static inline long long llabs(long long val)
{
	return (val < 0) ? -val : val;
}

static inline int fbt_is_self_control(unsigned long long t_cpu_target,
		unsigned long long deqtime_thr)
{
	long long diff;

	diff = (long long)t_cpu_target - (long long)q2q_time;
	if (llabs(diff) <= (long long)TIME_1MS)
		return 0;

	if (deq_time > deqtime_thr)
		return 0;

	return 1;
}

void fbt_check_self_control(unsigned long long t_cpu_target)
{
	unsigned long long thr;

	thr = t_cpu_target >> 4;

	if (middle_enable) {
		if (!fbt_is_self_control(t_cpu_target, thr))
			sf_check = min(sf_bound_min, ++sf_check);
		else
			sf_check = 0;
		if (sf_check == sf_bound_min)
			middle_enable ^= 1;
		fpsgo_systrace_c_fbt_gm(-100, sf_bound_min, "sf_bound");

	} else {
		if (fbt_is_self_control(t_cpu_target, thr))
			sf_check = min(sf_bound, ++sf_check);
		else
			sf_check = 0;
		if (sf_check == sf_bound)
			middle_enable ^= 1;
		fpsgo_systrace_c_fbt_gm(-100, sf_bound, "sf_bound");
	}
	fpsgo_systrace_c_fbt_gm(-100, sf_check, "sf_check");
	fpsgo_systrace_c_fbt_gm(-100, middle_enable, "middle_enable");

}

long long xgf_middle_vsync_check(long long t_cpu_target, unsigned long long t_cpu_slptime)
{
	unsigned long long next_vsync;
	unsigned long long queue_end;
	int diff, i;

	queue_end = ged_get_time();
	next_vsync = vsync_time;
	for (i = 1; i < 5; i++) {
		if (next_vsync > queue_end)
			break;
		next_vsync = vsync_time + vsync_period * i;
	}
	diff = next_vsync - queue_end;
	xgf_trace("next_vsync=%llu vsync_time=%llu queue_end=%llu diff=%llu t_cpu_target=%llu",
	next_vsync / (long long)NSEC_PER_USEC, vsync_time / (long long)NSEC_PER_USEC,
	queue_end / (long long)NSEC_PER_USEC, diff, t_cpu_target / (long long)NSEC_PER_USEC);
	if (diff < vsync_distance)
		t_cpu_target = t_cpu_target - TIME_1MS;

	return t_cpu_target;
}

static int cmpint(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

void fbt_cpu_floor_check(long long t_cpu_cur, long long loading,
		unsigned int target_fps, long long t_cpu_target)
{
	int pre_iter = 0;
	int next_iter = 0;

	pre_iter = (f_iter - 1 + WINDOW) % WINDOW;
	next_iter = (f_iter + 1 + WINDOW) % WINDOW;

	if (target_fps > 50)
		frame_info[f_iter].target_fps = 60;
	else if (target_fps > 40)
		frame_info[f_iter].target_fps = 45;
	else
		frame_info[f_iter].target_fps = 30;

	if (!(frame_info[pre_iter].target_fps)) {
		frame_info[f_iter].mips = base_blc * t_cpu_cur + loading;
		xgf_trace("fbt_cpu floor first frame frame_info[%d].mips=%d q2q_time=%llu",
				f_iter, frame_info[f_iter].mips, frame_info[f_iter].q2q_time);
		f_iter++;
		f_iter = f_iter % WINDOW;
		return;
	}

	if (frame_info[f_iter].target_fps == frame_info[pre_iter].target_fps) {
		unsigned long long mips_diff;
		long long frame_time;
		unsigned long long frame_bound;

		frame_info[f_iter].mips = base_blc * t_cpu_cur + loading;
		mips_diff =
			(abs(frame_info[pre_iter].mips - frame_info[f_iter].mips) * 100)
				/ frame_info[f_iter].mips;
		mips_diff = min(mips_diff, 100ULL);
		mips_diff = max(mips_diff, 1ULL);
		xgf_trace("fbt_cpu floor frame_info[%d].mips=%llu frame_info[%d].mips=%llu mips_diff=%llu",
				f_iter, frame_info[f_iter].mips, pre_iter,
				frame_info[pre_iter].mips, mips_diff);
		frame_time =
			(frame_info[pre_iter].q2q_time + frame_info[f_iter].q2q_time)
				/ (long long)NSEC_PER_USEC;
		xgf_trace("fbt_cpu floor frame_info[%d].q2q_time=%llu frame_info[%d].q2q_time=%llu",
			f_iter, frame_info[f_iter].q2q_time, pre_iter,
			frame_info[pre_iter].q2q_time);

		frame_info[f_iter].mips_diff = mips_diff;
		frame_info[f_iter].frame_time = frame_time;

		frame_bound = 5 * t_cpu_target / 2;

		fpsgo_systrace_c_fbt_gm(-100, frame_bound, "frame_bound");

		if (mips_diff > variance && frame_time > frame_bound)
			frame_info[f_iter].count = 1;
		else
			frame_info[f_iter].count = 0;

		floor_count = floor_count + frame_info[f_iter].count -	frame_info[next_iter].count;

		xgf_trace("fbt_cpu floor frame_info[%d].count=%d frame_info[%d].count=%d",
			f_iter, frame_info[f_iter].count, next_iter,
			frame_info[next_iter].count);

		fpsgo_systrace_c_fbt_gm(-100, mips_diff, "mips_diff");
		fpsgo_systrace_c_fbt_gm(-100, frame_time, "frame_time");
		fpsgo_systrace_c_fbt_gm(-100, frame_info[f_iter].count, "count");
		fpsgo_systrace_c_fbt_gm(-100, floor_count, "floor_count");

		if (floor_count >= floor_bound) {
			int i;
			int array[WINDOW];

			for (i = 0; i < WINDOW; i++)
				array[i] = (int)frame_info[i].mips_diff;
			sort(array, WINDOW, sizeof(int), cmpint, NULL);
			floor = array[min(kmin - 1, 0)];
		}

		/*reset floor check*/

		if (floor > 0) {
			if (floor_count == 0) {
				int reset_bound;

				reset_bound = 5 * frame_info[f_iter].target_fps;
				reset_floor_bound++;
				reset_floor_bound = min(reset_floor_bound, reset_bound);

				if (reset_floor_bound == reset_bound) {
					floor = 0;
					reset_floor_bound = 0;
				}
			} else if (floor_count > 2) {
				reset_floor_bound = 0;
			}
		}
		fpsgo_systrace_c_fbt_gm(-100, reset_floor_bound, "reset_floor_bound");

		f_iter++;
		f_iter = f_iter % WINDOW;
	} else {
		/*reset frame time info*/
		memset(frame_info, 0, sizeof(frame_info));
		floor_count = 0;
		f_iter = 0;
	}

}

void xgf_set_cpuset_and_limit(unsigned long long blc_wt)
{
	struct ppm_limit_data pld[2];
	unsigned long flags;
	int i;
	int tgt_opp, tgt_freq, floor_freq = 0;
	int mbhr, mbhr_opp;
	int floor_freq_L, floor_freq_LL = 0;
	int cluster;
	int move_cluster = 0;
	unsigned long long new_blc;

	pld[0].min = -1;
	pld[1].min = -1;

	fpsgo_systrace_c_log(floor, "floor");

	new_blc = (blc_wt * (floor + 100)) / 100;
	new_blc = min(new_blc, 100ULL);
	new_blc = max(new_blc, 1ULL);

	tgt_freq = (min((int)(blc_wt), 100) * power_l[0]) / 100;
	floor_freq = (min((int)(new_blc), 100) * power_l[0]) / 100;

	/* LL cluster sorting*/
	for (i = 15; i > 0; i--) {
		if (power_ll[i] >= tgt_freq)
			break;
	}
	tgt_opp  = i;
	floor_freq_LL = power_ll[i];

	if (floor > 1 && target_cluster == 0) {
		int k, opp;

		for (k = 15; k > 0; k--) {
			if (power_ll[k] >= floor_freq)
				break;
		}
		if (power_ll[k] == floor_freq_LL)
			opp = max((int)(k - floor_opp), 0);
		else
			opp = k;

		xgf_trace("fbt_cpu floor_freq_LL=%d power_ll[%d]=%d floor_freq=%d new_blc=%d",
			 floor_freq_LL, opp, power_ll[opp], floor_freq, new_blc);
		floor_freq_LL = power_ll[opp];
		tgt_opp = opp;
		bhr_opp = floor_opp;
	}

	mbhr_opp = max((int)(tgt_opp - bhr_opp), 0);
	mbhr = (floor_freq_LL * 100 / power_l[0]) + bhr;
	mbhr = (min((int)(mbhr), 100) * power_l[0]) / 100;
	for (i = 15; i > 0; i--) {
		if (power_ll[i] >= mbhr)
			break;
	}
	mbhr = power_ll[i];

	pld[0].max = max(mbhr, power_ll[mbhr_opp]);

	xgf_trace("max=%d tar=[%d]=%d opp=[%d]=%d bhr=%d",
			 pld[0].max, tgt_opp, tgt_freq,
			 mbhr_opp, power_ll[mbhr_opp], mbhr);
	/* LL cluster sorting done */

	/* L cluster sorting  */
	for (i = 15; i > 0; i--) {
		if (power_l[i] >= tgt_freq)
			break;
	}
	tgt_opp  = i;
	floor_freq_L = power_l[i];

	if (floor > 1 && target_cluster == 1) {
		int k, opp;

		for (k = 15; k > 0; k--) {
			if (power_l[k] >= floor_freq)
				break;
		}
		if (power_l[k] == floor_freq_L)
			opp = max((int)(k - floor_opp), 0);
		else
			opp = k;
		xgf_trace("fbt_cpu floor_freq_L=%d power_l[%d]=%d floor_freq=%d",
			 floor_freq_L, opp, power_l[opp], floor_freq);
		floor_freq_L = power_l[opp];
		tgt_opp = opp;
		bhr_opp = floor_opp;
	}

	mbhr_opp = max((int)(tgt_opp - bhr_opp), 0);

	mbhr = (floor_freq_L * 100 / power_l[0]) + bhr;
	mbhr = (min((int)(mbhr), 100) * power_l[0]) / 100;
	for (i = 15; i > 0; i--) {
		if (power_l[i] >= mbhr)
			break;
	}

	mbhr = power_l[i];

	pld[1].max = max(mbhr, power_l[mbhr_opp]);

	xgf_trace("max=%d tar=[%d]=%d opp=[%d]=%d bhr=%d",
			 pld[1].max, tgt_opp, tgt_freq,
			 mbhr_opp, power_l[mbhr_opp], mbhr);

	cluster = pld[1].max > cluster_freq_bound ? 1 : 0;

	spin_lock_irqsave(&xgf_slock, flags);
	if (target_cluster == 2) {
		target_cluster = cluster;
		move_cluster = 1;
	}
	if (cluster == target_cluster) {
		history_blc_count--;
		history_blc_count = max(history_blc_count, 0);
	} else {
		history_blc_count++;
		history_blc_count = min(history_blc_count, migrate_bound);
		if (history_blc_count == migrate_bound) {
			target_cluster = cluster;
			move_cluster = 1;
			history_blc_count = 0;
		}
	}
	floor_freq_LL = min(floor_freq_LL, lpp_max_freq_LL);
	floor_freq_L = min(floor_freq_L, lpp_max_freq_L);
	tgt_freq = target_cluster ? floor_freq_L : floor_freq_LL;
	base_blc = (unsigned)(tgt_freq * 100) / power_l[0];
	base_freq = tgt_freq;
	fpsgo_systrace_c_log(target_cluster, "target_cluster");
	fpsgo_systrace_c_log(base_blc, "perf idx");
	spin_unlock_irqrestore(&xgf_slock, flags);

	fpsgo_systrace_c_fbt_gm(-100, history_blc_count, "history_blc_count");
	fpsgo_systrace_c_fbt_gm(-100, tgt_freq, "tgt_freq");

	if (move_cluster) {
		set_cpuset(target_cluster);
#if 0
		if (target_cluster)
			vcorefs_request_dvfs_opp(KIR_FBT, 2);
		else
			vcorefs_request_dvfs_opp(KIR_FBT, -1);
#endif
	}
	/*set cpuset and cluster freq*/
	if (target_cluster == 1) { /* floor_freq > LL max freq, set L root cluster*/
		update_eas_boost_value(EAS_KIR_FBC, CGROUP_TA, (base_blc - 1) + 3200);
		pld[0].max = floor_freq_LL;
	} else if (target_cluster == 0) {
		update_eas_boost_value(EAS_KIR_FBC, CGROUP_TA, (base_blc - 1) + 3100);
		pld[1].max = floor_freq_L;
	}

	pld[1].max = min(pld[1].max, lpp_max_freq_L);
	pld[0].max = min(pld[0].max, lpp_max_freq_LL);

	update_userlimit_cpu_freq(EAS_KIR_FBC, 2, pld);
	fpsgo_systrace_c_fbt_gm(-100, pld[0].max, "cluster0 limit freq");
	fpsgo_systrace_c_fbt_gm(-100, pld[1].max, "cluster1 limit freq");
	fpsgo_systrace_c_fbt_gm(-100, lpp_max_freq_L, "lpp_max_freq_L");
	fpsgo_systrace_c_fbt_gm(-100, lpp_max_freq_LL, "lpp_max_freq_LL");
}

unsigned int xgf_get_userlimit_freq(void)
{
	int L_max, LL_max;
	unsigned long long max_freq;
	unsigned int limited_cap;

	L_max = mt_ppm_userlimit_freq_limit_by_others(1);
	LL_max = mt_ppm_userlimit_freq_limit_by_others(0);
	max_freq = max(power_l[L_max], power_ll[LL_max]);

	limited_cap = (max_freq * 100UL) / power_l[0];
	xgf_trace("max_freq=%d limited_cap=%d", max_freq, limited_cap);
	return limited_cap;
}

unsigned long long fbt_get_t2wnt(long long t_cpu_target)
{
	unsigned long long next_vsync, queue_end, queue_start;
	unsigned long long t2wnt;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&xgf_slock, flags);

	queue_start = ged_get_time();
	queue_end = queue_start + t_cpu_target;
	next_vsync = vsync_time;
	for (i = 1; i < 6; i++) {
		if (next_vsync > queue_end)
			break;
		next_vsync = vsync_time + vsync_period * i;
	}
	t2wnt = next_vsync - (unsigned long long)rescue_distance - queue_start;
	xgf_trace("t2wnt=%llu next_vsync=%llu queue_end=%llu",
		t2wnt / (long long)NSEC_PER_USEC,
		next_vsync / (long long)NSEC_PER_USEC, queue_end / (long long)NSEC_PER_USEC);

	spin_unlock_irqrestore(&xgf_slock, flags);

	return t2wnt;
}

void fbt_cpu_boost_policy(
	long long t_cpu_cur,
	long long t_cpu_target,
	unsigned long long t_cpu_slptime,
	unsigned int target_fps)
{
	unsigned long long blc_wt = 0;
	unsigned long long target_time = 0;
	long long aa;
	unsigned long flags;
	unsigned int limited_cap;
	long long t1, t2, t_sleep;
	u64 t2wnt;
	struct hrtimer *timer;
	char reset_asfc = 0;

	if (!game_mode)
		return;

	xgf_trace("fbt_cpu_boost_policy");
	xgf_dequeuebuffer(0);

	fpsgo_systrace_c_fbt_gm(-100, target_fps, "fps before using dfrc");
	target_fps = _gdfrc_fps_limit;
	t_cpu_target = _gdfrc_cpu_target;

	spin_lock_irqsave(&xgf_slock, flags);

	target_time = t_cpu_target;
	if (target_fps != TARGET_UNLIMITED_FPS)
		target_time = (unsigned long long)(FBTCPU_SEC_DIVIDER / (target_fps + RESET_TOLERENCE));

	xgf_trace("update target_time=%d t_cpu_target=%d target_fps=%d",
		target_time, t_cpu_target, target_fps);

	vsync_period = t_cpu_target;
	rescue_distance = (t_cpu_target * rescue_percent) / 100;
	vsync_distance = (t_cpu_target * vsync_percent) / 100;
	xgf_trace("update vsync_period=%d rescue_distance=%d vsync_distance=%d",
		vsync_period, rescue_distance, vsync_distance);

	if (lpp_fps || vag_fps) {
		if (lpp_fps)
			target_fps = lpp_fps;
		else
			target_fps = vag_fps;
		target_time = (unsigned long long)(FBTCPU_SEC_DIVIDER / target_fps);
		middle_enable = 0;
	} else
		fbt_check_self_control(target_time);

	if (asfc_last_fps == target_fps && asfc_time
		&& last_fps == TARGET_UNLIMITED_FPS) {
		unsigned long long asfc_tmp;

		asfc_tmp = ged_get_time();
		if (asfc_tmp - asfc_time < 300 * TIME_1MS)
			sf_bound = min(sf_bound_max, (sf_bound + sf_bound_min));
		xgf_trace("asfc_tmp=%llu asfc_time=%llu diff=%llu",
			asfc_tmp, asfc_time, (asfc_tmp - asfc_time));
		xgf_trace("last_fps=%d", last_fps);
		fpsgo_systrace_c_fbt_gm(-100, sf_bound, "sf_bound");
		fpsgo_systrace_c_fbt_gm(-100, asfc_last_fps, "asfc_last_fps");
	}

	last_fps = target_fps;

	if (middle_enable) {
		target_time = xgf_middle_vsync_check(target_time, t_cpu_slptime);
		if (target_fps == 30) {
			unsigned long long deqstar_time;

			deqstar_time = deqend_time - deq_time;
			xgf_trace("deqend_time=%llu vsync_time=%llu deq_time=%d deqstar_time=%llu",
					deqend_time, vsync_time, deq_time, deqstar_time);
			if (deqend_time > vsync_time && deqstar_time < vsync_time
				&& deq_time > deqtime_bound) {
				xgf_trace("fbt_reset_asfc");
				asfc_time = ged_get_time();
				asfc_last_fps = target_fps;
				reset_asfc = 1;
			}
		}
	}

	aa = (long long)atomic_read(&loading_cur);
	t1 = t_cpu_cur / (long long)NSEC_PER_USEC;
	t2 = target_time / (long long)NSEC_PER_USEC;
	t_sleep = (t_cpu_slptime + deq_time) / (long long)NSEC_PER_USEC;
	if (t1 > (2 * t2) || (base_blc * t1 + aa) < 0) {
		blc_wt = base_blc;
		aa = 0;
	} else if (t_sleep) {
		long long new_aa;

		new_aa = (aa * (t1 - t_sleep) / t1);
		xgf_trace("new_aa = %d aa = %d", new_aa, aa);
		aa = new_aa;
		blc_wt = (base_blc * t1 + new_aa) / t2;
	} else
		blc_wt = (base_blc * t1 + aa) / t2;

	xgf_trace("perf_index=%d base=%d aa=%d t_cpu_cur=%d target=%d sleep=%d",
		blc_wt, base_blc, aa, t1, t2, t_sleep);

	if (!lpp_fps)
		fbt_cpu_floor_check(t1, aa, target_fps, t2);

	atomic_set(&loading_cur, 0);
	fpsgo_systrace_c_fbt_gm(-100, atomic_read(&loading_cur), "loading");
	spin_unlock_irqrestore(&xgf_slock, flags);

	if (middle_enable && reset_asfc)
		fbt_reset_asfc(0);

	blc_wt = min(blc_wt, 100ULL);
	blc_wt = max(blc_wt, 1ULL);

	fpsgo_systrace_c_fbt_gm(-100, t_cpu_cur, "t_cpu_cur");
	fpsgo_systrace_c_fbt_gm(-100, target_time, "target_time");
	fpsgo_systrace_c_log(blc_wt, "perf idx");

	blc_wt = min_t(unsigned long long, blc_wt, lpp_max_cap);

	fpsgo_systrace_c_fbt_gm(-100, blc_wt, "perf idx");

	xgf_set_cpuset_and_limit(blc_wt);

	/*disable rescue frame for vag*/
	if (!lpp_fps || !vag_fps) {
		t2wnt = (u64) fbt_get_t2wnt(target_time);
		proc.active_jerk_id ^= 1;
		timer = &proc.jerks[proc.active_jerk_id].timer;
		hrtimer_start(timer, ns_to_ktime(t2wnt), HRTIMER_MODE_REL);
	}

	limited_cap = xgf_get_userlimit_freq();
	fpsgo_systrace_c_log(limited_cap, "limited_cap");
	fbt_notifier_push_cpu_capability(base_blc, limited_cap, target_fps);
}

void fbt_save_deq_q2q_info(unsigned long long deqtime, unsigned long long q2qtime)
{
	deq_time = deqtime;
	/* update and than refer to these data in one thread, lockless */
	q2q_time = q2qtime;

	frame_info[f_iter].q2q_time = q2qtime;
	xgf_trace("update deq_time=%llu q2qtime=%llu",
		deq_time, q2q_time);
}

int fbt_cpu_push_frame_time(pid_t pid, unsigned long long last_ts,
		unsigned long long curr_ts,
		unsigned long long *p_frmtime,
		unsigned long long *p_slptime)
{
	unsigned long long q2qtime;
	unsigned long long deqtime, slptime, blank_time;

	if (!game_mode)
		return 0;

	if (curr_ts < last_ts)
		return -EINVAL;

	q2qtime = curr_ts - last_ts;

	xgf_query_blank_time(pid, &deqtime, &slptime);
	blank_time = deqtime + slptime;

	if (q2qtime < blank_time || *p_frmtime < blank_time)
		return -EINVAL;

	*p_frmtime -= blank_time;
	*p_slptime = slptime;

	fpsgo_systrace_c_log(q2qtime, "get frame q2q time");
	fpsgo_systrace_c_log(slptime, "get frame sleep time");

	fbt_save_deq_q2q_info(deqtime, q2qtime);

	fbt_notifier_push_cpu_frame_time(q2qtime, *p_frmtime);
	return 0;
}

void fbt_cpu_cpu_boost_check_01(
	int gx_game_mode,
	int gx_force_cpu_boost,
	int enable_cpu_boost,
	int ismainhead)
{
	fpsgo_systrace_c_log(gx_game_mode, "gx_game_mode");
	fpsgo_systrace_c_fbt_gm(-100, gx_force_cpu_boost, "gx_force_cpu_boost");
	fpsgo_systrace_c_fbt_gm(-100, enable_cpu_boost, "enable_cpu_boost");
	fpsgo_systrace_c_fbt_gm(-100, ismainhead, "psHead == main_head");
}

static int fbt_cpu_dfrc_ntf_cb(unsigned int fps_limit)
{
	unsigned long flags;

	_gdfrc_fps_limit_ex = _gdfrc_fps_limit;
	_gdfrc_fps_limit = fps_limit;

	/* clear middle_enable once fps changes */
	spin_lock_irqsave(&xgf_slock, flags);
	if (_gdfrc_fps_limit_ex != _gdfrc_fps_limit) {
		middle_enable = 0;
		fpsgo_systrace_c_fbt_gm(-100, 0, "middle_enable");
	}
	spin_unlock_irqrestore(&xgf_slock, flags);

	_gdfrc_cpu_target = FBTCPU_SEC_DIVIDER/fps_limit;

	return 0;
}

#define FBT_DEBUGFS_ENTRY(name) \
static int fbt_##name##_open(struct inode *i, struct file *file) \
{ \
	return single_open(file, fbt_##name##_show, i->i_private); \
} \
\
static const struct file_operations fbt_##name##_fops = { \
	.owner = THIS_MODULE, \
	.open = fbt_##name##_open, \
	.read = seq_read, \
	.write = fbt_##name##_write, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

static int fbt_switch_fbt_show(struct seq_file *m, void *unused)
{
	SEQ_printf(m, "fbt_enable:%d\n", fbt_enable);
	return 0;
}

static ssize_t fbt_switch_fbt_write(struct file *flip,
			const char *ubuf, size_t cnt, loff_t *data)
{
	int val;
	int ret;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &val);
	if (ret)
		return ret;

	switch_fbt_game(val);

	return cnt;
}

FBT_DEBUGFS_ENTRY(switch_fbt);


static int fbt_set_vag_fps_show(struct seq_file *m, void *unused)
{
	SEQ_printf(m, "vag_fps:%d\n", vag_fps);
	return 0;
}

static ssize_t fbt_set_vag_fps_write(struct file *flip,
			const char *ubuf, size_t cnt, loff_t *data)
{
	int val;
	int ret;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &val);
	if (ret)
		return ret;

	fbt_cpu_vag_set_fps(val);

	return cnt;
}

FBT_DEBUGFS_ENTRY(set_vag_fps);

static int fbt_lpp_max_cap_show(struct seq_file *m, void *unused)
{
	SEQ_printf(m, "lpp_max_cap:%d\n", lpp_max_cap);
	return 0;
}

static ssize_t fbt_lpp_max_cap_write(struct file *flip,
			const char *ubuf, size_t cnt, loff_t *data)
{
	int val;
	int ret;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &val);
	if (ret)
		return ret;

	fbt_cpu_lppcap_update(val);

	return cnt;
}

FBT_DEBUGFS_ENTRY(lpp_max_cap);

static int fbt_lpp_fps_show(struct seq_file *m, void *unused)
{
	SEQ_printf(m, "lpp_fps:%d\n", lpp_fps);
	return 0;
}

static ssize_t fbt_lpp_fps_write(struct file *flip,
			const char *ubuf, size_t cnt, loff_t *data)
{
	int val;
	int ret;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &val);
	if (ret)
		return ret;

	fbt_cpu_lpp_set_fps(val);

	return cnt;
}

FBT_DEBUGFS_ENTRY(lpp_fps);


void fbt_cpu_exit(void)
{

}

int __init fbt_cpu_init(void)
{
	struct proc_dir_entry *pe;

	bhr = 5;
	bhr_opp = 1;
	target_cluster = 2;
	cluster_freq_bound = 1287000;
	cluster_rescue_bound = 1287000;
	migrate_bound = 10;
	vsync_percent = 50;
	rescue_opp_c = 15;
	rescue_opp_f = 5;
	rescue_percent = 33;
	vsync_period = GED_VSYNC_MISS_QUANTUM_NS;
	sf_bound_max = 10;
	sf_bound_min = 3;
	sf_bound = sf_bound_min;
	deqtime_bound = TIME_3MS;
	fbt_enable = 1;
	variance = 40;
	floor_bound = 3;
	floor = 0;
	kmin = 10;
	floor_opp = 2;
	_gdfrc_fps_limit    = TARGET_UNLIMITED_FPS;
	_gdfrc_fps_limit_ex = TARGET_UNLIMITED_FPS;

	/* GED */
	ged_kpi_set_game_hint_value_fp_fbt = fbt_notifier_push_game_mode;
	ged_kpi_cpu_boost_fp_fbt = fbt_cpu_boost_policy;

	ged_kpi_push_game_frame_time_fp_fbt = fbt_cpu_push_frame_time;
	ged_kpi_push_app_self_fc_fp_fbt = xgf_self_ctrl_enable;

	ged_kpi_cpu_boost_check_01 = fbt_cpu_cpu_boost_check_01;

	fbt_notifier_dfrc_fp_fbt_cpu = fbt_cpu_dfrc_ntf_cb;

	/* */
	cpufreq_notifier_fp = xgf_cpufreq_cb;
	ged_vsync_notifier_fp = xgf_ged_vsync;
	update_pwd_tbl();

	lpp_max_freq_L = power_l[0];
	lpp_max_freq_LL = power_ll[0];
	lpp_max_cap = 100;

	/*llp*/
	update_lppcap = fbt_cpu_lppcap_update;
	set_fps = fbt_cpu_lpp_set_fps;

	fbt_init_jerk(&proc.jerks[0], 0);
	fbt_init_jerk(&proc.jerks[1], 1);

	if (!proc_mkdir("fbt_cpu", NULL))
		return -1;

	pe = proc_create("fbt_cpu/switch_fbt", 0660, NULL, &fbt_switch_fbt_fops);
	if (!pe)
		return -ENOMEM;

	pe = proc_create("fbt_cpu/set_vag_fps", 0660, NULL, &fbt_set_vag_fps_fops);
	if (!pe)
		return -ENOMEM;

	pe = proc_create("fbt_cpu/lpp_max_cap", 0660, NULL, &fbt_lpp_max_cap_fops);
	if (!pe)
		return -ENOMEM;

	pe = proc_create("fbt_cpu/lpp_fps", 0660, NULL, &fbt_lpp_fps_fops);
	if (!pe)
		return -ENOMEM;

	return 0;
}
