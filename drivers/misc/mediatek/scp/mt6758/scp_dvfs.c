/*
* Copyright (C) 2011-2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_secure_api.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#endif

#include <linux/uaccess.h>
#include "scp_ipi.h"
#include "scp_helper.h"
#include "scp_excep.h"
#include "scp_dvfs.h"

#ifndef CONFIG_MTK_CLKMGR
#include <linux/clk.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif

#if !defined(CONFIG_FPGA_EARLY_PORTING)
#include "mtk_pmic_info.h"
#endif

/*
 * LOG
 */
#define TAG	"[Power/scp_dvfs] "

#define scp_dvfs_err(fmt, args...)	\
	pr_err(TAG"[ERROR]"fmt, ##args)
#define scp_dvfs_warn(fmt, args...)	\
	pr_warn(TAG"[WARNING]"fmt, ##args)
#define scp_dvfs_info(fmt, args...)	\
	pr_warn(TAG""fmt, ##args)
#define scp_dvfs_dbg(fmt, args...)	\
	do {			\
		if (mt_scp_dvfs_debug)		\
			scp_dvfs_info(fmt, ##args);     \
	} while (0)
#define scp_dvfs_ver(fmt, args...)	\
	do {			\
		if (mt_scp_dvfs_debug)	\
			pr_debug(TAG""fmt, ##args);	\
	} while (0)

#define READ_REGISTER_UINT32(reg) \
	(*(volatile unsigned int * const)(reg))

#define WRITE_REGISTER_UINT32(reg, val) \
	((*(volatile unsigned int * const)(reg)) = (val))

#define INREG32(x)          READ_REGISTER_UINT32((unsigned int *)((void *)(x)))
#define OUTREG32(x, y)      WRITE_REGISTER_UINT32((unsigned int *)((void *)(x)), (unsigned int)(y))
#define SETREG32(x, y)      OUTREG32(x, INREG32(x)|(y))
#define CLRREG32(x, y)      OUTREG32(x, INREG32(x)&~(y))
#define MASKREG32(x, y, z)  OUTREG32(x, (INREG32(x)&~(y))|(z))

#define DRV_Reg32(addr)             INREG32(addr)
#define DRV_WriteReg32(addr, data)  OUTREG32(addr, data)
#define DRV_SetReg32(addr, data)    SETREG32(addr, data)
#define DRV_ClrReg32(addr, data)    CLRREG32(addr, data)

/***************************
 * Operate Point Definition
 ****************************/
static struct pinctrl *scp_pctrl; /* static pinctrl instance */

/* DTS state */
typedef enum tagDTS_GPIO_STATE {
	SCP_DTS_GPIO_STATE_DEFAULT = 0,
	SCP_DTS_VREQ_OFF,
	SCP_DTS_VREQ_ON,
	SCP_DTS_GPIO_STATE_MAX,	/* for array size */
} SCP_DTS_GPIO_STATE;

/* DTS state mapping name */
static const char *scp_state_name[SCP_DTS_GPIO_STATE_MAX] = {
	"default",
	"scp_gpio_off",
	"scp_gpio_on"
};

/* 0:SCP DVFS OFF, 1:SCP DVFS ON */
static int scp_dvfs_flag = 1;

/*
 * 0: SCP Sleep: OFF,
 * 1: SCP Sleep: ON,
 * 2: SCP Sleep: sleep without wakeup,
 * 3: SCP Sleep: force to sleep
 */
static int scp_sleep_flag = -1;

static int mt_scp_dvfs_debug = -1;
static unsigned int scp_cur_volt = -1;
static struct mt_scp_pll_t *mt_scp_pll;
static struct wake_lock scp_suspend_lock;
static int g_scp_dvfs_init_flag = -1;

static unsigned int pre_feature_req = 0xff;

static unsigned long gpio_base;
#define RG_GPIO159_MODE		(gpio_base + 0x430)
#define GPIO159_BIT			28
#define GPIO159_MASK		0x7

unsigned int scp_get_dvfs_opp(void)
{
	return scp_cur_volt;
}

short  scp_set_pmic_vcore(unsigned int cur_freq)
{
	short ret = 0;
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	unsigned short ret_vc, ret_vs;

	if (cur_freq <= CLK_OPP0) {
		ret_vc = pmic_scp_set_vcore(600000);
		ret_vs = pmic_scp_set_vsram_vcore(800000);
		scp_cur_volt = 2;
	} else if (cur_freq <= CLK_OPP1) {
		ret_vc = pmic_scp_set_vcore(700000);
		ret_vs = pmic_scp_set_vsram_vcore(800000);
		scp_cur_volt = 1;
	} else {
		ret_vc = pmic_scp_set_vcore(800000);
		ret_vs = pmic_scp_set_vsram_vcore(900000);
		scp_cur_volt = 0;
	}

	if (ret_vc != 0 || ret_vs != 0) {
		ret = -1;
		scp_dvfs_err("scp vcore / vsram setting error, (%d, %d)", ret_vc, ret_vs);
		WARN_ON(1);
	}
#endif

	return ret;
}

uint32_t scp_get_freq(void)
{
	uint32_t i;

	uint32_t sum = 0;
	uint32_t return_freq = 0;

	/*
	 * calculate scp frequence
	 */
	for (i = 0; i < NUM_FEATURE_ID; i++) {
		if (feature_table[i].enable == 1)
			sum += feature_table[i].freq;
	}
	/*
	 * calculate scp sensor frequence
	 */
	for (i = 0; i < NUM_SENSOR_TYPE; i++) {
		if (sensor_type_table[i].enable == 1)
			sum += sensor_type_table[i].freq;
	}

	/*pr_debug("[SCP] needed freq sum:%d\n",sum);*/
	if (sum <= CLK_OPP0)
		return_freq = CLK_OPP0;
	else if (sum <= CLK_OPP1)
		return_freq = CLK_OPP1;
	else
		return_freq = CLK_OPP2;

	return return_freq;
}

/* scp_request_freq
 * return :-1 means the scp request freq. error
 * return :0  means the request freq. finished
 */
int scp_request_freq(void)
{
	int value = 0;
	int timeout = 250;
	int ret = 0;
	unsigned long spin_flags;
	int is_increasing_freq = 0;

	if (scp_dvfs_flag != 1) {
		pr_info("warning: SCP DVFS is OFF\n");
		return -1;
	}

	/* because we are waiting for scp to update register:scp_current_freq
	 * use wake lock to prevent AP from entering suspend state
	 */
	wake_lock(&scp_suspend_lock);

	if (scp_current_freq != scp_expected_freq) {

		/* do DVS before DFS if increasing frequency */
		if (scp_current_freq < scp_expected_freq) {
			mt_secure_call(MTK_SIP_KERNEL_SCP_DVFS_CTRL, scp_expected_freq, 0, 0);
			is_increasing_freq = 1;
		}

		#if SCP_DVFS_USE_PLL
		/*  turn on PLL */
		scp_pll_ctrl_set(PLL_ENABLE, scp_expected_freq);
		#endif

		 do {
			ret = scp_ipi_send(IPI_DVFS_SET_FREQ, (void *)&value, sizeof(value), 0, SCP_A_ID);
			if (ret != SCP_IPI_DONE)
				pr_err("%s: SCP send IPI fail - %d\n", __func__, ret);

			mdelay(2);
			timeout -= 1; /*try 50 times, total about 100ms*/
			if (timeout <= 0) {
				pr_crit("%s: set freq fail, current(%d) != expect(%d)\n", __func__,
					scp_current_freq, scp_expected_freq);
				goto set_freq_fail;
			}

			/* read scp_current_freq again */
			spin_lock_irqsave(&scp_awake_spinlock, spin_flags);
			scp_current_freq = readl(CURRENT_FREQ_REG);
			spin_unlock_irqrestore(&scp_awake_spinlock, spin_flags);

		} while (scp_current_freq != scp_expected_freq);

		#if SCP_DVFS_USE_PLL
		/* turn off PLL */
		scp_pll_ctrl_set(PLL_DISABLE, 0);
		#endif

		/* do DVS after DFS if decreasing frequency */
		if (is_increasing_freq == 0)
			mt_secure_call(MTK_SIP_KERNEL_SCP_DVFS_CTRL, scp_expected_freq, 0, 0);

		/*  set pmic sshub_sleep_vcore_ctrl accroding to frequency */
		ret = scp_set_pmic_vcore(scp_current_freq);
		if (ret != 0) {
			pr_crit("%s: scp_set_pmic_vcore(%d) fail - %d\n", __func__, scp_current_freq, ret);
			goto fatal_error;
		}
	}

	wake_unlock(&scp_suspend_lock);
	pr_info("[SCP] set freq OK, %d == %d\n", scp_expected_freq, scp_current_freq);
	return 0;

set_freq_fail:
	mt_secure_call(MTK_SIP_KERNEL_SCP_DVFS_CTRL, scp_current_freq, 0, 0);
	scp_A_dump_regs();
	wake_unlock(&scp_suspend_lock);
	WARN_ON(1);
	return -1;

fatal_error:
	scp_A_dump_regs();
	wake_unlock(&scp_suspend_lock);
	WARN_ON(1);
	return -1;
}

void wait_scp_dvfs_init_done(void)
{
	int count = 0;

	while (g_scp_dvfs_init_flag != 1) {
		mdelay(1);
		count++;
		if (count > 3000) {
			scp_dvfs_err("SCP dvfs driver init fail\n");
			WARN_ON(1);
		}
	}
}

void scp_pll_mux_set(unsigned int pll_ctrl_flag)
{
	int ret = 0;

	scp_dvfs_info("%s(%d)\n\n", __func__, pll_ctrl_flag);

	if (pll_ctrl_flag == PLL_ENABLE) {
		ret = clk_prepare_enable(mt_scp_pll->clk_mux);
		if (ret) {
			scp_dvfs_err("scp dvfs cannot enable clk mux, %d\n", ret);
			WARN_ON(1);
		}
	} else
		clk_disable_unprepare(mt_scp_pll->clk_mux);
}

int scp_pll_ctrl_set(unsigned int pll_ctrl_flag, unsigned int pll_sel)
{
	int ret = 0;

	scp_dvfs_info("%s(%d, %d)\n", __func__, pll_ctrl_flag, pll_sel);

	if (pll_ctrl_flag == PLL_ENABLE) {
		ret = clk_prepare_enable(mt_scp_pll->clk_mux);
		if (ret) {
			scp_dvfs_err("scp dvfs cannot enable clk mux, %d\n", ret);
			WARN_ON(1);
		}

		switch (pll_sel) {
		case CLK_26M:
			ret = clk_set_parent(
					mt_scp_pll->clk_mux,
					mt_scp_pll->clk_pll0);
			break;
		case CLK_OPP0:
			ret = clk_set_parent(mt_scp_pll->clk_mux, mt_scp_pll->clk_pll0);
			break;
		case CLK_OPP1:
			ret = clk_set_parent(mt_scp_pll->clk_mux, mt_scp_pll->clk_pll5);
			break;
		case CLK_OPP2:
			ret = clk_set_parent(mt_scp_pll->clk_mux, mt_scp_pll->clk_pll2);
			break;
		default:
			break;
		}
	} else if (pll_ctrl_flag == PLL_DISABLE)
		clk_disable_unprepare(mt_scp_pll->clk_mux);

	return ret;
}

void scp_pll_ctrl_handler(int id, void *data, unsigned int len)
{
	unsigned int *pll_ctrl_flag = (unsigned int *)data;
	unsigned int *pll_sel =  (unsigned int *) (data + 1);
	int ret = 0;

	ret = scp_pll_ctrl_set(*pll_ctrl_flag, *pll_sel);
}

#ifdef CONFIG_PROC_FS
/*
 * PROC
 */

/***************************
 * show current debug status
 ****************************/
static int mt_scp_dvfs_debug_proc_show(struct seq_file *m, void *v)
{
	if (mt_scp_dvfs_debug == -1)
		seq_puts(m, "mt_scp_dvfs_debug has not been set\n");
	else
		seq_printf(m, "mt_scp_dvfs_debug = %d\n", mt_scp_dvfs_debug);

	return 0;
}

/***********************
 * enable debug message
 ************************/
static ssize_t mt_scp_dvfs_debug_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[64];
	unsigned int debug = 0;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (kstrtouint(desc, 0, &debug) == 0) {
		if (debug == 0)
			mt_scp_dvfs_debug = 0;
		else if (debug == 1)
			mt_scp_dvfs_debug = 1;
		else
			scp_dvfs_warn("bad argument %d\n", debug);
			scp_dvfs_warn("echo [0|1] > /proc/scp_dvfs/scp_dvfs_debug\n");
	} else {
		scp_dvfs_warn("invalid command!\n");
		scp_dvfs_warn("echo [0|1] > /proc/scp_dvfs/scp_dvfs_debug\n");
	}

	scp_ipi_send(IPI_DVFS_DEBUG, (void *)&mt_scp_dvfs_debug, sizeof(mt_scp_dvfs_debug), 0, SCP_A_ID);

	return count;
}

/****************************
 * show SCP state
 *****************************/
static int mt_scp_dvfs_state_proc_show(struct seq_file *m, void *v)
{
	unsigned int scp_state;

	scp_state = readl(SCP_A_SLEEP_DEBUG_REG);
	seq_printf(m, "scp status: %s\n",
		((scp_state & IN_DEBUG_IDLE) == IN_DEBUG_IDLE) ? "idle mode"
		: ((scp_state & ENTERING_SLEEP) == ENTERING_SLEEP) ? "enter sleep"
		: ((scp_state & IN_SLEEP) == IN_SLEEP) ? "sleep mode"
		: ((scp_state & ENTERING_ACTIVE) == ENTERING_ACTIVE) ? "enter active"
		: ((scp_state & IN_ACTIVE) == IN_ACTIVE) ? "active mode" : "none of state");
	return 0;
}

/****************************
 * show scp dvfs sleep
 *****************************/
static int mt_scp_dvfs_sleep_proc_show(struct seq_file *m, void *v)
{
	if (scp_sleep_flag == -1)
		seq_puts(m, "Warning: SCP sleep has not been manually configured by shell command!\n");
	else if (scp_sleep_flag == 0)
		seq_puts(m, "SCP Sleep: OFF\n");
	else if (scp_sleep_flag == 1)
		seq_puts(m, "SCP Sleep: ON\n");
	else if (scp_sleep_flag == 2)
		seq_puts(m, "SCP Sleep: sleep without wakeup\n");
	else if (scp_sleep_flag == 3)
		seq_puts(m, "SCP Sleep: force to sleep\n");
	else
		seq_puts(m, "Warning: invalid SCP Sleep configure\n");

	return 0;
}

/**********************************
 * write scp dvfs sleep
 ***********************************/
static ssize_t mt_scp_dvfs_sleep_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[64];
	unsigned int val = 0;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (kstrtouint(desc, 0, &val) == 0) {
		if (val >= 0  && val <= 3) {
			if (val != scp_sleep_flag) {
				scp_sleep_flag = val;
				scp_dvfs_warn("scp_sleep_flag = %d\n", scp_sleep_flag);
				scp_ipi_send(IPI_DVFS_SLEEP, (void *)&scp_sleep_flag, sizeof(scp_sleep_flag),
							0, SCP_A_ID);
			} else
				scp_dvfs_warn("SCP sleep setting is not changed. keep in %d\n", val);
		} else {
			scp_dvfs_warn("Warning: invalid input value %d\n", val);
			scp_dvfs_warn("sleep off:\n");
			scp_dvfs_warn("  echo 0 > /proc/scp_dvfs/scp_dvfs_sleep\n");
			scp_dvfs_warn("sleep on:\n");
			scp_dvfs_warn("  echo 1 > /proc/scp_dvfs/scp_dvfs_sleep\n");
			scp_dvfs_warn("sleep without wakeup:\n");
			scp_dvfs_warn("  echo 2 > /proc/scp_dvfs/scp_dvfs_sleep\n");
			scp_dvfs_warn("force to sleep:\n");
			scp_dvfs_warn("  echo 3 > /proc/scp_dvfs/scp_dvfs_sleep\n");
		}
	} else {
		scp_dvfs_warn("Warning: invalid input command, val=%d\n", val);
		scp_dvfs_warn("sleep off:\n");
		scp_dvfs_warn("  echo 0 > /proc/scp_dvfs/scp_dvfs_sleep\n");
		scp_dvfs_warn("sleep on:\n");
		scp_dvfs_warn("  echo 1 > /proc/scp_dvfs/scp_dvfs_sleep\n");
		scp_dvfs_warn("sleep without wakeup:\n");
		scp_dvfs_warn("  echo 2 > /proc/scp_dvfs/scp_dvfs_sleep\n");
		scp_dvfs_warn("force to sleep:\n");
		scp_dvfs_warn("  echo 3 > /proc/scp_dvfs/scp_dvfs_sleep\n");
	}

	return count;
}

/****************************
 * show scp dvfs ctrl
 *****************************/
static int mt_scp_dvfs_ctrl_proc_show(struct seq_file *m, void *v)
{
	unsigned long spin_flags;
	int i;

	spin_lock_irqsave(&scp_awake_spinlock, spin_flags);
	scp_current_freq = readl(CURRENT_FREQ_REG);
	scp_expected_freq = readl(EXPECTED_FREQ_REG);
	spin_unlock_irqrestore(&scp_awake_spinlock, spin_flags);
	seq_printf(m, "SCP DVFS: %s\n", (scp_dvfs_flag == 1)?"ON":"OFF");
	seq_printf(m, "SCP frequency: cur=%dMHz, expect=%dMHz\n", scp_current_freq, scp_expected_freq);

	for (i = 0; i < NUM_FEATURE_ID; i++)
		seq_printf(m, "feature=%d, freq=%d, enable=%d\n",
			feature_table[i].feature, feature_table[i].freq, feature_table[i].enable);

	for (i = 0; i < NUM_SENSOR_TYPE; i++)
		seq_printf(m, "sensor id=%d, freq=%d, enable=%d\n",
			sensor_type_table[i].feature, sensor_type_table[i].freq, sensor_type_table[i].enable);

	return 0;
}

/**********************************
 * write scp dvfs ctrl
 ***********************************/
static ssize_t mt_scp_dvfs_ctrl_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[64], cmd[32];
	int len = 0;
	int req;
	int n;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	n = sscanf(desc, "%31s %d", cmd, &req);
	if (n == 1 || n == 2) {
		if (!strcmp(cmd, "on")) {
			scp_dvfs_flag = 1;
			scp_dvfs_warn("SCP DVFS: ON\n");
		} else if (!strcmp(cmd, "off")) {
			scp_dvfs_flag = 0;
			scp_dvfs_warn("SCP DVFS: OFF\n");
		} else if (!strcmp(cmd, "req")) {
			if (req >= 0 && req <= 5) {
				if (pre_feature_req == 1)
					scp_deregister_feature(VCORE_TEST_FEATURE_ID);
				else if (pre_feature_req == 2)
					scp_deregister_feature(VCORE_TEST2_FEATURE_ID);
				else if (pre_feature_req == 3)
					scp_deregister_feature(VCORE_TEST3_FEATURE_ID);
				else if (pre_feature_req == 4)
					scp_deregister_feature(VCORE_TEST4_FEATURE_ID);
				else if (pre_feature_req == 5)
					scp_deregister_feature(VCORE_TEST5_FEATURE_ID);

				if (req == 1)
					scp_register_feature(VCORE_TEST_FEATURE_ID);
				else if (req == 2)
					scp_register_feature(VCORE_TEST2_FEATURE_ID);
				else if (req == 3)
					scp_register_feature(VCORE_TEST3_FEATURE_ID);
				else if (req == 4)
					scp_register_feature(VCORE_TEST4_FEATURE_ID);
				else if (req == 5)
					scp_register_feature(VCORE_TEST5_FEATURE_ID);

				pre_feature_req = req;
				scp_dvfs_warn("[SCP] set freq: %d => %d\n", scp_current_freq, scp_expected_freq);
			} else {
				scp_dvfs_warn("invalid req value %d\n", req);
				scp_dvfs_warn("echo req <0|1|2|3|4|5> > /proc/scp_dvfs/scp_dvfs_ctrl\n");
			}
		} else {
			scp_dvfs_warn("invalid command %s\n", cmd);
			scp_dvfs_warn("echo on > /proc/scp_dvfs/scp_dvfs_ctrl\n");
			scp_dvfs_warn("echo off > /proc/scp_dvfs/scp_dvfs_ctrl\n");
			scp_dvfs_warn("echo req <0|1|2|3|4|5> > /proc/scp_dvfs/scp_dvfs_ctrl\n");
		}
	} else {
		scp_dvfs_warn("invalid length %d\n", n);
		scp_dvfs_warn("echo on > /proc/scp_dvfs/scp_dvfs_ctrl\n");
		scp_dvfs_warn("echo off > /proc/scp_dvfs/scp_dvfs_ctrl\n");
		scp_dvfs_warn("echo req <0|1|2|3|4|5> > /proc/scp_dvfs/scp_dvfs_ctrl\n");
	}

	return count;
}

#define PROC_FOPS_RW(name)							\
	static int mt_ ## name ## _proc_open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, mt_ ## name ## _proc_show, PDE_DATA(inode));	\
}									\
static const struct file_operations mt_ ## name ## _proc_fops = {		\
	.owner		  = THIS_MODULE,				\
	.open		   = mt_ ## name ## _proc_open,	\
	.read		   = seq_read,					\
	.llseek		 = seq_lseek,					\
	.release		= single_release,				\
	.write		  = mt_ ## name ## _proc_write,				\
}

#define PROC_FOPS_RO(name)							\
	static int mt_ ## name ## _proc_open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, mt_ ## name ## _proc_show, PDE_DATA(inode));	\
}									\
static const struct file_operations mt_ ## name ## _proc_fops = {		\
	.owner		  = THIS_MODULE,				\
	.open		   = mt_ ## name ## _proc_open,	\
	.read		   = seq_read,					\
	.llseek		 = seq_lseek,					\
	.release		= single_release,				\
}

#define PROC_ENTRY(name)	{__stringify(name), &mt_ ## name ## _proc_fops}

PROC_FOPS_RW(scp_dvfs_debug);
PROC_FOPS_RO(scp_dvfs_state);
PROC_FOPS_RW(scp_dvfs_sleep);
PROC_FOPS_RW(scp_dvfs_ctrl);

static int mt_scp_dvfs_create_procfs(void)
{
	struct proc_dir_entry *dir = NULL;
	int i;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(scp_dvfs_debug),
		PROC_ENTRY(scp_dvfs_state),
		PROC_ENTRY(scp_dvfs_sleep),
		PROC_ENTRY(scp_dvfs_ctrl)
	};

	dir = proc_mkdir("scp_dvfs", NULL);
	if (!dir) {
		scp_dvfs_err("fail to create /proc/scp_dvfs @ %s()\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, dir, entries[i].fops))
			scp_dvfs_err("@%s: create /proc/scp_dvfs/%s failed\n", __func__, entries[i].name);
	}

	return 0;
}
#endif /* CONFIG_PROC_FS */

/* pinctrl implementation */
static long _set_state(const char *name)
{
	long ret = 0;
	struct pinctrl_state *pState = 0;

	WARN_ON(!scp_pctrl);

	pState = pinctrl_lookup_state(scp_pctrl, name);
	if (IS_ERR(pState)) {
		pr_err("lookup state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(scp_pctrl, pState);

exit:
	return ret; /* Good! */
}

static const struct of_device_id scpdvfs_of_ids[] = {
	{.compatible = "mediatek,mt6758-scpdvfs",},
	{}
};

static int mt_scp_dvfs_suspend(struct device *dev)
{
	return 0;
}

static int mt_scp_dvfs_resume(struct device *dev)
{
	return 0;
}

static int mt_scp_dvfs_pm_restore_early(struct device *dev)
{
	return 0;
}

static int mt_scp_dvfs_pdrv_probe(struct platform_device *pdev)
{
	struct device_node *node;
	unsigned int gpio_mode;

	scp_dvfs_info("%s()\n", __func__);

	node = of_find_matching_node(NULL, scpdvfs_of_ids);
	if (!node) {
		dev_err(&pdev->dev, "%s: fail to find SCPDVFS node\n", __func__);
		WARN_ON(1);
	}

	mt_scp_pll = kzalloc(sizeof(struct mt_scp_pll_t), GFP_KERNEL);
	if (mt_scp_pll == NULL)
		return -ENOMEM;

	mt_scp_pll->clk_mux = devm_clk_get(&pdev->dev, "clk_mux");
	if (IS_ERR(mt_scp_pll->clk_mux)) {
		dev_err(&pdev->dev, "cannot get clock mux\n");
		return PTR_ERR(mt_scp_pll->clk_mux);
	}

	mt_scp_pll->clk_pll0 = devm_clk_get(&pdev->dev, "clk_pll_0");
	if (IS_ERR(mt_scp_pll->clk_pll0)) {
		dev_err(&pdev->dev, "cannot get 1st clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll0);
	}
	mt_scp_pll->clk_pll1 = devm_clk_get(&pdev->dev, "clk_pll_1");
	if (IS_ERR(mt_scp_pll->clk_pll1)) {
		dev_err(&pdev->dev, "cannot get 2nd clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll1);
	}
	mt_scp_pll->clk_pll2 = devm_clk_get(&pdev->dev, "clk_pll_2");
	if (IS_ERR(mt_scp_pll->clk_pll2)) {
		dev_err(&pdev->dev, "cannot get 3rd clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll2);
	}
	mt_scp_pll->clk_pll3 = devm_clk_get(&pdev->dev, "clk_pll_3");
	if (IS_ERR(mt_scp_pll->clk_pll3)) {
		dev_err(&pdev->dev, "cannot get 4th clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll3);
	}
	mt_scp_pll->clk_pll4 = devm_clk_get(&pdev->dev, "clk_pll_4");
	if (IS_ERR(mt_scp_pll->clk_pll4)) {
		dev_err(&pdev->dev, "cannot get 5th clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll4);
	}
	mt_scp_pll->clk_pll5 = devm_clk_get(&pdev->dev, "clk_pll_5");
	if (IS_ERR(mt_scp_pll->clk_pll5)) {
		dev_err(&pdev->dev, "cannot get 6th clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll5);
	}
	mt_scp_pll->clk_pll6 = devm_clk_get(&pdev->dev, "clk_pll_6");
	if (IS_ERR(mt_scp_pll->clk_pll6)) {
		dev_err(&pdev->dev, "cannot get 7th clock parent\n");
		return PTR_ERR(mt_scp_pll->clk_pll6);
	}
	scp_pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(scp_pctrl)) {
		dev_err(&pdev->dev, "Cannot find scp pinctrl!\n");
		return PTR_ERR(scp_pctrl);
	}

	_set_state(scp_state_name[SCP_DTS_VREQ_ON]);

	/* get GPIO base address */
	node = of_find_compatible_node(NULL, NULL, "mediatek,gpio");
	if (!node) {
		pr_err("error: can't find GPIO node\n");
		WARN_ON(1);
		return 0;
	}

	gpio_base = (unsigned long)of_iomap(node, 0);
	if (!gpio_base) {
		pr_err("error: iomap fail for GPIO\n");
		WARN_ON(1);
		return 0;
	}

	/* check if v_req pin is configured correctly  */
	gpio_mode = (DRV_Reg32(RG_GPIO159_MODE)>>GPIO159_BIT)&GPIO159_MASK;
	if (gpio_mode == 1)
		scp_dvfs_info("V_REQ muxpin setting is correct\n");
	else
		pr_err("error: V_REQ muxpin setting is wrong - %d\n", gpio_mode);

	g_scp_dvfs_init_flag = 1;

	return 0;
}

/***************************************
 * this function should never be called
 ****************************************/
static int mt_scp_dvfs_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct dev_pm_ops mt_scp_dvfs_pm_ops = {
	.suspend = mt_scp_dvfs_suspend,
	.resume = mt_scp_dvfs_resume,
	.restore_early = mt_scp_dvfs_pm_restore_early,
};

struct platform_device mt_scp_dvfs_pdev = {
	.name = "mt-scpdvfs",
	.id = -1,
};

static struct platform_driver mt_scp_dvfs_pdrv = {
	.probe = mt_scp_dvfs_pdrv_probe,
	.remove = mt_scp_dvfs_pdrv_remove,
	.driver = {
		.name = "scpdvfs",
		.pm = &mt_scp_dvfs_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = scpdvfs_of_ids,
		},
};

/**********************************
 * mediatek scp dvfs initialization
 ***********************************/
void mt_pmic_sshub_init(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	unsigned int ret[6];

	ret[0] = pmic_get_register_value(PMIC_RG_BUCK_VCORE_SSHUB_ON);
	ret[1] = pmic_get_register_value(PMIC_RG_BUCK_VCORE_SSHUB_MODE);
	ret[2] = pmic_get_register_value(PMIC_RG_BUCK_VCORE_SSHUB_VOSEL);
	ret[3] = pmic_get_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_ON);
	ret[4] = pmic_get_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_MODE);
	ret[5] = pmic_get_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL);
	scp_dvfs_info("Before init vcore and vsram_core:\n");
	scp_dvfs_info("vcore: on, mode, vosel = 0x%x, 0x%x, 0x%x\n", ret[0], ret[1], ret[2]);
	scp_dvfs_info("vsram: on, mode, vosel = 0x%x, 0x%x, 0x%x\n", ret[3], ret[4], ret[5]);

	pmic_scp_set_vcore(800000);
	pmic_scp_set_vsram_vcore(900000);
	pmic_set_register_value(PMIC_RG_BUCK_VCORE_SSHUB_ON, 1);
	pmic_set_register_value(PMIC_RG_BUCK_VCORE_SSHUB_MODE, 1);
	pmic_set_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_ON, 1);
	pmic_set_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_MODE, 1);
	ret[0] = pmic_get_register_value(PMIC_RG_BUCK_VCORE_SSHUB_ON);
	ret[1] = pmic_get_register_value(PMIC_RG_BUCK_VCORE_SSHUB_MODE);
	ret[2] = pmic_get_register_value(PMIC_RG_BUCK_VCORE_SSHUB_VOSEL);
	ret[3] = pmic_get_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_ON);
	ret[4] = pmic_get_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_MODE);
	ret[5] = pmic_get_register_value(PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL);
	scp_dvfs_info("After init vcore and vsram_core:\n");
	scp_dvfs_info("vcore: on, mode, vosel = 0x%x, 0x%x, 0x%x\n", ret[0], ret[1], ret[2]);
	scp_dvfs_info("vsram: on, mode, vosel = 0x%x, 0x%x, 0x%x\n", ret[3], ret[4], ret[5]);
#endif
}

void mt_scp_dvfs_ipi_init(void)
{
	scp_ipi_registration(IPI_SCP_PLL_CTRL, scp_pll_ctrl_handler, "IPI_SCP_PLL_CTRL");
}

int __init scp_dvfs_init(void)
{
	int ret = 0;

	scp_dvfs_info("@%s\n", __func__);

#ifdef CONFIG_PROC_FS
	/* init proc */
	if (mt_scp_dvfs_create_procfs()) {
		pr_info("mt_scp_dvfs_create_procfs fail..\n");
		WARN_ON(1);
		return -1;
	}
#endif /* CONFIG_PROC_FS */

	/* register platform device/driver */
	ret = platform_device_register(&mt_scp_dvfs_pdev);
	if (ret) {
		scp_dvfs_err("fail to register scp dvfs device @ %s()\n", __func__);
		goto out;
	}

	ret = platform_driver_register(&mt_scp_dvfs_pdrv);
	if (ret) {
		scp_dvfs_err("fail to register scp dvfs driver @ %s()\n", __func__);
		platform_device_unregister(&mt_scp_dvfs_pdev);
		goto out;
	}

	wake_lock_init(&scp_suspend_lock, WAKE_LOCK_SUSPEND, "scp wakelock");

	mt_scp_dvfs_ipi_init();
	mt_pmic_sshub_init();
out:
	return ret;
}

void __exit scp_dvfs_exit(void)
{
	platform_driver_unregister(&mt_scp_dvfs_pdrv);
	platform_device_unregister(&mt_scp_dvfs_pdev);
}

