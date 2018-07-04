#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/string.h>

#include <linux/platform_device.h>
#include "legacy_controller.h"
#include <mach/mtk_ppm_api.h>

#define TAG "[boost_controller]"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

static struct mutex boost_freq;
static struct mutex boost_core;
static struct ppm_limit_data current_core[NR_PPM_CLUSTERS];
static struct ppm_limit_data current_freq[NR_PPM_CLUSTERS];
static struct ppm_limit_data core_set[PPM_MAX_KIR][NR_PPM_CLUSTERS];
static struct ppm_limit_data freq_set[PPM_MAX_KIR][NR_PPM_CLUSTERS];
static int log_enable;

#define legacy_debug(enable, fmt, x...)\
	do {\
		if (enable)\
			pr_debug(fmt, ##x);\
	} while (0)

/*************************************************************************************/
int update_userlimit_cpu_core(int kicker, int num_cluster, struct ppm_limit_data *core_limit)
{
#ifndef CONFIG_MTK_ACAO_SUPPORT
	struct ppm_limit_data final_core[NR_PPM_CLUSTERS];
	int i, j;

	mutex_lock(&boost_core);
	if (num_cluster != NR_PPM_CLUSTERS) {
		pr_crit(TAG"num_cluster : %d NR_PPM_CLUSTERS: %d, doesn't match",
			 num_cluster, NR_PPM_CLUSTERS);
		mutex_unlock(&boost_core);
		return -1;
	}

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		final_core[i].min = -1;
		final_core[i].max = -1;
	}

#if 0
	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		legacy_debug(log_enable, TAG"cluster %d, core_limit_min %d core_limit_max %d\n",
			 i, core_limit[i].min, core_limit[i].max);
	}
#endif

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		core_set[kicker][i].min = core_limit[i].min >= -1 ? core_limit[i].min : -1;
		core_set[kicker][i].max = core_limit[i].max >= -1 ? core_limit[i].max : -1;
		legacy_debug(log_enable, TAG"kicker %d cluster %d, core_set_min %d core_set_max %d\n",
			 kicker, i, core_set[kicker][i].min, core_set[kicker][i].max);
	}

	for (i = 0; i < PPM_MAX_KIR; i++) {
		for (j = 0; j < NR_PPM_CLUSTERS; j++) {
			final_core[j].min = MAX(core_set[i][j].min, final_core[j].min);
			final_core[j].max = MAX(core_set[i][j].max, final_core[j].max);
			if (final_core[j].min > final_core[j].max && final_core[j].max != -1)
				final_core[j].max = final_core[j].min;
		}
	}

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		current_core[i].min = final_core[i].min;
		current_core[i].max = final_core[i].max;
		legacy_debug(log_enable, TAG"cluster %d, current_core_min %d current_core_max %d\n",
			 i, current_core[i].min, current_core[i].max);
	}

	mt_ppm_userlimit_cpu_core(NR_PPM_CLUSTERS, final_core);

	mutex_unlock(&boost_core);
#endif

	return 0;
}

/*************************************************************************************/
int update_userlimit_cpu_freq(int kicker, int num_cluster, struct ppm_limit_data *freq_limit)
{
	struct ppm_limit_data final_freq[NR_PPM_CLUSTERS];
	int i, j;

	mutex_lock(&boost_freq);
	if (num_cluster != NR_PPM_CLUSTERS) {
		pr_crit(TAG"num_cluster : %d NR_PPM_CLUSTERS: %d, doesn't match",
			 num_cluster, NR_PPM_CLUSTERS);
		mutex_unlock(&boost_core);
		return -1;
	}

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		final_freq[i].min = -1;
		final_freq[i].max = -1;
	}

#if 0
	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		legacy_debug(log_enable, TAG"cluster %d, freq_limit_min %d freq_limit_max %d\n",
			 i, freq_limit[i].min, freq_limit[i].max);
	}
#endif


	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		freq_set[kicker][i].min = freq_limit[i].min >= -1 ? freq_limit[i].min : -1;
		freq_set[kicker][i].max = freq_limit[i].max >= -1 ? freq_limit[i].max : -1;
		legacy_debug(log_enable, TAG"kicker %d cluster %d, freq_set_min %d freq_set_max %d\n",
			 kicker, i, freq_set[kicker][i].min, freq_set[kicker][i].max);
	}

	for (i = 0; i < PPM_MAX_KIR; i++) {
		for (j = 0; j < NR_PPM_CLUSTERS; j++) {
			final_freq[j].min = MAX(freq_set[i][j].min, final_freq[j].min);
			final_freq[j].max = MAX(freq_set[i][j].max, final_freq[j].max);
			if (final_freq[j].min > final_freq[j].max && final_freq[j].max != -1)
				final_freq[j].max = final_freq[j].min;
		}
	}

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		current_freq[i].min = final_freq[i].min;
		current_freq[i].max = final_freq[i].max;
		legacy_debug(log_enable, TAG"cluster %d, freq_min %d freq_max %d\n",
				i, current_freq[i].min, current_freq[i].max);
	}

	mt_ppm_userlimit_cpu_freq(NR_PPM_CLUSTERS, final_freq);

	mutex_unlock(&boost_freq);

	return 0;
}

/*************************************************************************************/
static ssize_t perfmgr_perfserv_core_write(struct file *filp, const char __user *ubuf,
		size_t cnt, loff_t *pos)
{
	int i = 0, data;
	struct ppm_limit_data core_limit[NR_PPM_CLUSTERS];
	unsigned int arg_num = NR_PPM_CLUSTERS * 2; /* for min and max */
	char *tok, *tmp;
	char *buf = ppm_copy_from_user_for_proc(ubuf, cnt);

	tmp = buf;
	while ((tok = strsep(&tmp, " ")) != NULL) {
		if (i == arg_num) {
			pr_crit(TAG"@%s: number of arguments > %d!\n", __func__, arg_num);
			goto out;
		}

		if (kstrtoint(tok, 10, &data)) {
			pr_crit(TAG"@%s: Invalid input: %s\n", __func__, tok);
			goto out;
		} else {
			if (i % 2) /* max */
				core_limit[i/2].max = data;
			else /* min */
				core_limit[i/2].min = data;
			i++;
		}
	}

	if (i < arg_num)
		pr_crit(TAG"@%s: number of arguments < %d!\n", __func__, arg_num);
	else
		update_userlimit_cpu_core(PPM_KIR_PERF, NR_PPM_CLUSTERS, core_limit);

out:
	free_page((unsigned long)buf);
	return cnt;

}

static int perfmgr_perfserv_core_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < NR_PPM_CLUSTERS; i++)
		seq_printf(m, "cluster %d min:%d max:%d\n",
				i, core_set[PPM_KIR_PERF][i].min, core_set[PPM_KIR_PERF][i].max);

	return 0;
}

static int perfmgr_perfserv_core_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_perfserv_core_show, inode->i_private);
}

static const struct file_operations perfmgr_perfserv_core_fops = {
	.open = perfmgr_perfserv_core_open,
	.write = perfmgr_perfserv_core_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static ssize_t perfmgr_perfserv_freq_write(struct file *filp, const char __user *ubuf,
		size_t cnt, loff_t *pos)
{
	int i = 0, data;
	struct ppm_limit_data freq_limit[NR_PPM_CLUSTERS];
	unsigned int arg_num = NR_PPM_CLUSTERS * 2; /* for min and max */
	char *tok, *tmp;
	char *buf = ppm_copy_from_user_for_proc(ubuf, cnt);

	tmp = buf;
	while ((tok = strsep(&tmp, " ")) != NULL) {
		if (i == arg_num) {
			pr_crit(TAG"@%s: number of arguments > %d!\n", __func__, arg_num);
			goto out;
		}

		if (kstrtoint(tok, 10, &data)) {
			pr_crit(TAG"@%s: Invalid input: %s\n", __func__, tok);
			goto out;
		} else {
			if (i % 2) /* max */
				freq_limit[i/2].max = data;
			else /* min */
				freq_limit[i/2].min = data;
			i++;
		}
	}

	if (i < arg_num)
		pr_crit(TAG"@%s: number of arguments < %d!\n", __func__, arg_num);
	else
		update_userlimit_cpu_freq(PPM_KIR_PERF, NR_PPM_CLUSTERS, freq_limit);

out:
	free_page((unsigned long)buf);
	return cnt;

}

static int perfmgr_perfserv_freq_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < NR_PPM_CLUSTERS; i++)
		seq_printf(m, "cluster %d min:%d max:%d\n",
				i, freq_set[PPM_KIR_PERF][i].min, freq_set[PPM_KIR_PERF][i].max);

	return 0;
}

static int perfmgr_perfserv_freq_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_perfserv_freq_show, inode->i_private);
}

static const struct file_operations perfmgr_perfserv_freq_fops = {
	.open = perfmgr_perfserv_freq_open,
	.write = perfmgr_perfserv_freq_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static int perfmgr_current_core_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < NR_PPM_CLUSTERS; i++)
		seq_printf(m, "cluster %d min:%d max:%d\n",
				i, current_core[i].min, current_core[i].max);

	return 0;
}

static int perfmgr_current_core_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_current_core_show, inode->i_private);
}

static const struct file_operations perfmgr_current_core_fops = {
	.open = perfmgr_current_core_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/*************************************************************************************/
static int perfmgr_current_freq_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < NR_PPM_CLUSTERS; i++)
		seq_printf(m, "cluster %d min:%d max:%d\n",
				i, current_freq[i].min, current_freq[i].max);

	return 0;
}

static int perfmgr_current_freq_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_current_freq_show, inode->i_private);
}

static const struct file_operations perfmgr_current_freq_fops = {
	.open = perfmgr_current_freq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static ssize_t perfmgr_log_write(struct file *filp, const char __user *ubuf,
		size_t cnt, loff_t *pos)
{
	char buf[64];
	unsigned long val;
	int ret;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;
	ret = kstrtoul(buf, 10, &val);
	if (ret < 0)
		return ret;

	if (val)
		log_enable = 1;
	else
		log_enable = 0;

	return cnt;
}

static int perfmgr_log_show(struct seq_file *m, void *v)
{
		seq_printf(m, "%d\n",
				log_enable);

	return 0;
}

static int perfmgr_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmgr_log_show, inode->i_private);
}

static const struct file_operations perfmgr_log_fops = {
	.open = perfmgr_log_open,
	.write = perfmgr_log_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*************************************************************************************/
static int __init perfmgr_legacy_boost_init(void)
{
	int i, j;
	struct proc_dir_entry *boost_dir = NULL;

	mutex_init(&boost_core);
	mutex_init(&boost_freq);
	boost_dir = proc_mkdir("perfmgr/legacy", NULL);

	proc_create("perfserv_core", 0644, boost_dir, &perfmgr_perfserv_core_fops);
	proc_create("current_core", 0644, boost_dir, &perfmgr_current_core_fops);
	proc_create("perfserv_freq", 0644, boost_dir, &perfmgr_perfserv_freq_fops);
	proc_create("current_freq", 0644, boost_dir, &perfmgr_current_freq_fops);
	proc_create("perfmgr_log", 0644, boost_dir, &perfmgr_log_fops);

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		current_core[i].min = -1;
		current_core[i].max = -1;
		current_freq[i].min = -1;
		current_freq[i].max = -1;
	}

	for (i = 0; i < PPM_MAX_KIR; i++) {
		for (j = 0; j < NR_PPM_CLUSTERS; j++) {
			core_set[i][j].min = -1;
			core_set[i][j].max = -1;
			freq_set[i][j].min = -1;
			freq_set[i][j].max = -1;
		}
	}

	return 0;

}

late_initcall(perfmgr_legacy_boost_init);
