/*
 * Copyright (C) 2016-2017 MediaTek Inc.
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

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/string.h>

#ifdef CONFIG_MTK_FPSGO_FBT_GAME
#include <fpsgo_common.h>
#include <mtk_dramc.h>
#include <mtk_vcorefs_governor.h>
#include <mtk_vcorefs_manager.h>
#include "eas_controller_usedext.h"
#endif

#include <linux/platform_device.h>
#include "eas_controller.h"

#define TAG "[Boost Controller]"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

static struct mutex boost_eas;
#ifdef CONFIG_SCHED_TUNE
static int current_boost_value[NR_CGROUP];
#endif
static int boost_value[NR_CGROUP][EAS_MAX_KIR];
static int debug_boost_value[NR_CGROUP];
static int debug;

#ifdef CONFIG_MTK_FPSGO_FBT_GAME
/*for CPI monitor*/
/*
*LL: 806MHz(1600->2400), 1.6G(2400->3200)
*L: 1G(1600->2400), 1.4~1.5G(2400->3200)
*/
static long long power_ll[16], power_l[16];
static long long cpi_ll_boost_threshold[2], cpi_l_boost_threshold[2];
static int cpi_thres = 250, vcore_high = 1;
static int vcore;
static int ddr_type;

static void update_pwd_tbl(void)
{
	int i;
	long long max_freq;

	ddr_type = get_ddr_type();

	for (i = 0; i < 16; i++)
		power_ll[i] = mt_cpufreq_get_freq_by_idx(0, i);

	for (i = 0; i < 16; i++)
		power_l[i] = mt_cpufreq_get_freq_by_idx(1, i);

	max_freq = power_l[0];

	switch (ddr_type) {
	case TYPE_LPDDR3:
		cpi_ll_boost_threshold[0] = 160000000LL / max_freq;
		cpi_ll_boost_threshold[1] = 101;
		cpi_l_boost_threshold[0] = 150000000LL / max_freq;
		cpi_l_boost_threshold[1] = 101;
		break;
	case TYPE_LPDDR4:
	case TYPE_LPDDR4X:
	default:
		cpi_ll_boost_threshold[0] = 80600000LL / max_freq;
		cpi_ll_boost_threshold[1] = 160000000LL / max_freq;
		cpi_l_boost_threshold[0] = 100000000LL / max_freq;
		cpi_l_boost_threshold[1] = 150000000LL / max_freq;
		break;
	}

	pr_debug(TAG" max_freq:%lld, ll_0:%lld, ll_1:%lld, l_0:%lld, l_1:%lld\n",
			max_freq, cpi_ll_boost_threshold[0], cpi_ll_boost_threshold[1],
			cpi_l_boost_threshold[0], cpi_l_boost_threshold[1]);
}

static void reduce_stall(int boost_value)
{
	unsigned int cpi_ll, cpi_l;
	int vcore_opp = 0;

	if (boost_value < 3000) {
		vcore_opp = -1;
	/*LL*/
	} else if (boost_value >= 3100 && boost_value <= 3199) {
		cpi_ll = ppm_get_cluster_cpi(0);
		fpsgo_systrace_c_fbt_gm(-400, cpi_ll, "cpi_ll");
		if (cpi_ll < cpi_thres)
			vcore_opp = -1;
		else if (boost_value - 3100 > cpi_ll_boost_threshold[1] && vcore_high)
			vcore_opp = 1;
		else if (boost_value - 3100 > cpi_ll_boost_threshold[0])
			vcore_opp = 2;
		else
			vcore_opp = -1;
	/*L*/
	} else if (boost_value >= 3200 && boost_value <= 3299) {
		cpi_l = ppm_get_cluster_cpi(1);
		fpsgo_systrace_c_fbt_gm(-400, cpi_l, "cpi_l");
		if (cpi_l < cpi_thres)
			vcore_opp = -1;
		else if (boost_value - 3200 > cpi_l_boost_threshold[1] && vcore_high)
			vcore_opp = 1;
		else if (boost_value - 3200 > cpi_l_boost_threshold[0])
			vcore_opp = 2;
		else
			vcore_opp = -1;
	}

	if (vcore == vcore_opp)
		return;
	vcore = vcore_opp;
	vcorefs_request_dvfs_opp(KIR_FBT, vcore);

	fpsgo_systrace_c_fbt_gm(-400, boost_value - 3100, "boost_value");
	fpsgo_systrace_c_fbt_gm(-400, vcore_opp, "vcore_opp");

}
#endif
/*************************************************************************************/
#ifdef CONFIG_SCHED_TUNE
int update_eas_boost_value(int kicker, int cgroup_idx, int value)
{
	int final_boost_value = 0, final_boost_value_1 = 0, final_boost_value_2 = -101;
	int boost_1[EAS_MAX_KIR], boost_2[EAS_MAX_KIR];
	int has_set = 0;
	int i;

	mutex_lock(&boost_eas);

	if (cgroup_idx >= NR_CGROUP) {
		mutex_unlock(&boost_eas);
		pr_debug(TAG" cgroup_idx >= NR_CGROUP, error\n");
		return -1;
	}

	boost_value[cgroup_idx][kicker] = value;

	for (i = 0; i < EAS_MAX_KIR; i++) {
		boost_1[i] = boost_value[cgroup_idx][i] / 1000;
		boost_2[i] = boost_value[cgroup_idx][i] % 1000;
	}

	for (i = 0; i < EAS_MAX_KIR; i++) {
		if (boost_value[cgroup_idx][i] != 0) {
			final_boost_value_1 = MAX(boost_1[i], final_boost_value_1);
			final_boost_value_2 = MAX(boost_2[i], final_boost_value_2);
			has_set = 1;
		}
	}

	if (has_set)
		final_boost_value = final_boost_value_1 * 1000 + final_boost_value_2;
	else
		final_boost_value = 0;

	if (final_boost_value > 4000)
		current_boost_value[cgroup_idx] = 4000;
	else if (final_boost_value < -100)
		current_boost_value[cgroup_idx] = -100;
	else
		current_boost_value[cgroup_idx] = final_boost_value;

	if (kicker == EAS_KIR_PERF)
		pr_debug(TAG"kicker:%d, boost:%d, final:%d, current:%d",
				kicker, boost_value[cgroup_idx][kicker],
				final_boost_value, current_boost_value[cgroup_idx]);


	if (!debug) {
		if (current_boost_value[cgroup_idx] >= -100 && current_boost_value[cgroup_idx] < 4000) {
			boost_write_for_perf_idx(cgroup_idx, current_boost_value[cgroup_idx]);
#ifdef CONFIG_MTK_FPSGO_FBT_GAME
				reduce_stall(current_boost_value[cgroup_idx]);
#endif
		}
	}

	mutex_unlock(&boost_eas);

	return current_boost_value[cgroup_idx];
}
#else
int update_eas_boost_value(int kicker, int cgroup_idx, int value)
{
	return -1;
}
#endif

/*************************************************************************************/
static ssize_t perfmgr_perfserv_fg_boost_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	if (data > 3000)
		data = 3000;
	else if (data < -100)
		data = -100;

	update_eas_boost_value(EAS_KIR_PERF, CGROUP_FG, data);

	return cnt;
}

static int perfmgr_perfserv_fg_boost_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", boost_value[CGROUP_FG][EAS_KIR_PERF]);

	return 0;
}

static int perfmgr_perfserv_fg_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_perfserv_fg_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_perfserv_fg_boost_fops = {
	.open = perfmgr_perfserv_fg_boost_open,
	.write = perfmgr_perfserv_fg_boost_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static int perfmgr_current_fg_boost_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_SCHED_TUNE
	seq_printf(m, "%d\n", current_boost_value[CGROUP_FG]);
#else
	seq_printf(m, "%d\n", -1);
#endif

	return 0;
}

static int perfmgr_current_fg_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_current_fg_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_current_fg_boost_fops = {
	.open = perfmgr_current_fg_boost_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static ssize_t perfmgr_debug_fg_boost_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	if (data > 3000) {
		debug_boost_value[CGROUP_FG] = 3000;
		debug = 1;
	} else if (data < -100) {
		debug_boost_value[CGROUP_FG] = -100;
		debug = 1;
	} else {
		debug_boost_value[CGROUP_FG] = data;
		debug = 1;
	}

#ifdef CONFIG_SCHED_TUNE
	if (debug)
		boost_write_for_perf_idx(CGROUP_FG, debug_boost_value[CGROUP_FG]);
#endif

	return cnt;
}

static int perfmgr_debug_fg_boost_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", debug_boost_value[CGROUP_FG]);

	return 0;
}

static int perfmgr_debug_fg_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_debug_fg_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_debug_fg_boost_fops = {
	.open = perfmgr_debug_fg_boost_open,
	.write = perfmgr_debug_fg_boost_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static ssize_t perfmgr_perfserv_bg_boost_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	if (data > 3000)
		data = 3000;
	else if (data < -100)
		data = -100;

	update_eas_boost_value(EAS_KIR_PERF, CGROUP_BG, data);

	return cnt;
}

static int perfmgr_perfserv_bg_boost_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", boost_value[CGROUP_BG][EAS_KIR_PERF]);

	return 0;
}

static int perfmgr_perfserv_bg_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_perfserv_bg_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_perfserv_bg_boost_fops = {
	.open = perfmgr_perfserv_bg_boost_open,
	.write = perfmgr_perfserv_bg_boost_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static int perfmgr_current_bg_boost_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_SCHED_TUNE
	seq_printf(m, "%d\n", current_boost_value[CGROUP_BG]);
#else
	seq_printf(m, "%d\n", -1);
#endif

	return 0;
}

static int perfmgr_current_bg_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_current_bg_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_current_bg_boost_fops = {
	.open = perfmgr_current_bg_boost_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static ssize_t perfmgr_debug_bg_boost_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	if (data > 3000) {
		debug_boost_value[CGROUP_BG] = 3000;
		debug = 1;
	} else if (data < -100) {
		debug_boost_value[CGROUP_BG] = -100;
		debug = 1;
	} else {
		debug_boost_value[CGROUP_BG] = data;
		debug = 1;
	}

#ifdef CONFIG_SCHED_TUNE
	if (debug)
		boost_write_for_perf_idx(CGROUP_BG, debug_boost_value[CGROUP_BG]);
#endif

	return cnt;
}

static int perfmgr_debug_bg_boost_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", debug_boost_value[CGROUP_BG]);

	return 0;
}

static int perfmgr_debug_bg_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_debug_bg_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_debug_bg_boost_fops = {
	.open = perfmgr_debug_bg_boost_open,
	.write = perfmgr_debug_bg_boost_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


/*************************************************************************************/
static ssize_t perfmgr_perfserv_ta_boost_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	if (data > 4000)
		data = 4000;
	else if (data < -100)
		data = -100;

	update_eas_boost_value(EAS_KIR_PERF, CGROUP_TA, data);

	return cnt;
}

static int perfmgr_perfserv_ta_boost_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", boost_value[CGROUP_TA][EAS_KIR_PERF]);

	return 0;
}

static int perfmgr_perfserv_ta_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_perfserv_ta_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_perfserv_ta_boost_fops = {
	.open = perfmgr_perfserv_ta_boost_open,
	.write = perfmgr_perfserv_ta_boost_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static int perfmgr_current_ta_boost_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_SCHED_TUNE
	seq_printf(m, "%d\n", current_boost_value[CGROUP_TA]);
#else
	seq_printf(m, "%d\n", -1);
#endif

	return 0;
}

static int perfmgr_current_ta_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_current_ta_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_current_ta_boost_fops = {
	.open = perfmgr_current_ta_boost_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static ssize_t perfmgr_debug_ta_boost_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	if (data > 3000) {
		debug_boost_value[CGROUP_TA] = 3000;
		debug = 1;
	} else if (data < -100) {
		debug_boost_value[CGROUP_TA] = -100;
		debug = 1;
	} else {
		debug_boost_value[CGROUP_TA] = data;
		debug = 1;
	}

#ifdef CONFIG_SCHED_TUNE
	if (debug)
		boost_write_for_perf_idx(CGROUP_TA, debug_boost_value[CGROUP_TA]);
#endif

	return cnt;
}

static int perfmgr_debug_ta_boost_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", debug_boost_value[CGROUP_TA]);

	return 0;
}

static int perfmgr_debug_ta_boost_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_debug_ta_boost_show, inode->i_private);
}

static const struct file_operations perfmgr_debug_ta_boost_fops = {
	.open = perfmgr_debug_ta_boost_open,
	.write = perfmgr_debug_ta_boost_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/*************************************************************************************/
#ifdef CONFIG_MTK_FPSGO_FBT_GAME
static ssize_t perfmgr_cpi_thres_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	cpi_thres = data;

	return cnt;
}

static int perfmgr_cpi_thres_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", cpi_thres);

	return 0;
}

static int perfmgr_cpi_thres_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_cpi_thres_show, inode->i_private);
}

static const struct file_operations perfmgr_cpi_thres_fops = {
	.open = perfmgr_cpi_thres_open,
	.write = perfmgr_cpi_thres_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static ssize_t perfmgr_vcore_high_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *pos)
{
	int data;
	char buf[128];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;

	if (kstrtoint(buf, 10, &data))
		return -1;

	vcore_high = data;

	return cnt;
}

static int perfmgr_vcore_high_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", vcore_high);

	return 0;
}

static int perfmgr_vcore_high_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_vcore_high_show, inode->i_private);
}

static const struct file_operations perfmgr_vcore_high_fops = {
	.open = perfmgr_vcore_high_open,
	.write = perfmgr_vcore_high_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif
/*************************************************************************************/
void perfmgr_eas_boost_init(void)
{
	int i, j;
	struct proc_dir_entry *boost_dir = NULL;

	mutex_init(&boost_eas);

	boost_dir = proc_mkdir("perfmgr/eas", NULL);
	proc_create("perfserv_fg_boost", 0644, boost_dir, &perfmgr_perfserv_fg_boost_fops);
	proc_create("current_fg_boost", 0644, boost_dir, &perfmgr_current_fg_boost_fops);
	proc_create("debug_fg_boost", 0644, boost_dir, &perfmgr_debug_fg_boost_fops);

	proc_create("perfserv_bg_boost", 0644, boost_dir, &perfmgr_perfserv_bg_boost_fops);
	proc_create("current_bg_boost", 0644, boost_dir, &perfmgr_current_bg_boost_fops);
	proc_create("debug_bg_boost", 0644, boost_dir, &perfmgr_debug_bg_boost_fops);

	proc_create("perfserv_ta_boost", 0644, boost_dir, &perfmgr_perfserv_ta_boost_fops);
	proc_create("current_ta_boost", 0644, boost_dir, &perfmgr_current_ta_boost_fops);
	proc_create("debug_ta_boost", 0644, boost_dir, &perfmgr_debug_ta_boost_fops);

#ifdef CONFIG_MTK_FPSGO_FBT_GAME
	proc_create("vcore_high", 0644, boost_dir, &perfmgr_vcore_high_fops);
	proc_create("cpi_thres", 0644, boost_dir, &perfmgr_cpi_thres_fops);
#endif

	for (i = 0; i < NR_CGROUP; i++)
		for (j = 0; j < EAS_MAX_KIR; j++)
			boost_value[i][j] = 0;

#ifdef CONFIG_MTK_FPSGO_FBT_GAME
	/*update pwr table for CPI monitor*/
	update_pwd_tbl();
#endif

}

void init_perfmgr_eas_controller(void)
{
	perfmgr_eas_boost_init();
}

