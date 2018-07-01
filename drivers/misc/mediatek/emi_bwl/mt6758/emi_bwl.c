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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#define MET_USER_EVENT_SUPPORT
/* #include <linux/met_drv.h> */

#include <mt-plat/mtk_io.h>
#include <mt-plat/sync_write.h>

#include "emi_bwl.h"
#include "emi_mbw.h"
#include "emi_elm.h"

DEFINE_SEMAPHORE(emi_bwl_sem);
static DEFINE_SPINLOCK(emi_drs_lock);

static void __iomem *CEN_EMI_BASE;
static void __iomem *CHA_EMI_BASE;
static void __iomem *CHB_EMI_BASE;
static void __iomem *EMI_MPU_BASE;
static void __iomem *EMI_ELM_DBG_BASE;
static unsigned int mpu_irq;
static unsigned int cgm_irq;
static unsigned int elm_irq;
static bool force_drs_disable;

static struct emi_info_t emi_info;

static int emi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	pr_debug("[EMI] module probe.\n");

	if (node) {
		mpu_irq = irq_of_parse_and_map(node, 0);
		cgm_irq = irq_of_parse_and_map(node, 1);
		elm_irq = irq_of_parse_and_map(node, 2);
		pr_info("[EMI] get irq of MPU(%d), GCM(%d), ELM(%d)\n",
			mpu_irq, cgm_irq, elm_irq);
	} else {
		mpu_irq = 0;
		cgm_irq = 0;
		elm_irq = 0;
	}

	force_drs_disable = false;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	CEN_EMI_BASE = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(CEN_EMI_BASE)) {
		pr_debug("[EMI] unable to map CEN_EMI_BASE\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	CHA_EMI_BASE = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(CHA_EMI_BASE)) {
		pr_debug("[EMI] unable to map CHA_EMI_BASE\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	CHB_EMI_BASE = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(CHB_EMI_BASE)) {
		pr_debug("[EMI] unable to map CHB_EMI_BASE\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	EMI_MPU_BASE = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(EMI_MPU_BASE)) {
		pr_debug("[EMI] unable to map EMI_MPU_BASE\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	EMI_ELM_DBG_BASE = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(EMI_ELM_DBG_BASE)) {
		pr_debug("[EMI] unable to map EMI_ELM_DBG_BASE\n");
		return -EINVAL;
	}

	pr_info("[EMI] get CEN_EMI_BASE @ %p\n", mt_cen_emi_base_get());
	pr_info("[EMI] get CHA_EMI_BASE @ %p\n", mt_chn_emi_base_get(0));
	pr_info("[EMI] get CHB_EMI_BASE @ %p\n", mt_chn_emi_base_get(1));
	pr_info("[EMI] get EMI_MPU_BASE @ %p\n", mt_emi_mpu_base_get());
	pr_debug("[EMI] get EMI_ELM_DBG_BASE @ %p\n", mt_emi_elm_dbg_base_get());

	ret = mtk_mem_bw_ctrl(CON_SCE_UI, ENABLE_CON_SCE);
	if (ret)
		pr_debug("[EMI/BWL] fail to set EMI bandwidth limiter\n");

#if ENABLE_MBW
	mbw_init();
#endif
#if ENABLE_ELM
	elm_init(cgm_irq);
#endif
	return 0;
}

static int emi_remove(struct platform_device *dev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id emi_of_ids[] = {
	{.compatible = "mediatek,emi",},
	{}
};
#endif

#ifdef CONFIG_PM
static int emi_suspend_noirq(struct device *dev)
{
	/* pr_info("[EMI] suspend\n"); */
	suspend_elm();

	return 0;
}

static int emi_resume_noirq(struct device *dev)
{
	/* pr_info("[EMI] resume\n"); */
	resume_elm();

	return 0;
}

static const struct dev_pm_ops emi_pm_ops = {
	.suspend_noirq = emi_suspend_noirq,
	.resume_noirq = emi_resume_noirq,
};
#define EMI_PM_OPS     (&emi_pm_ops)
#else
#define EMI_PM_OPS     NULL
#endif

static struct platform_driver emi_ctrl = {
	.probe = emi_probe,
	.remove = emi_remove,
	.driver = {
		.name = "emi_ctrl",
		.owner = THIS_MODULE,
		.pm = EMI_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = emi_of_ids,
#endif
	},
};

/* define EMI bandwiwth limiter control table */
static struct emi_bwl_ctrl ctrl_tbl[NR_CON_SCE];

/* current concurrency scenario */
static int cur_con_sce = 0x0FFFFFFF;

/* define concurrency scenario strings */
static const char * const con_sce_str[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm)\
(#con_sce),
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};

/****************** For LPDDR4-3200******************/

static const unsigned int emi_arba_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) arba,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbc_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) arbc,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbd_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) arbd,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbe_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) arbe,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbf_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) arbf,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbg_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) arbg,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};
static const unsigned int emi_conm_lpddr4_val[] = {
#define X_CON_SCE(con_sce, arba, arbc, arbd, arbe, arbf, arbg, conm) conm,
#include "con_sce_lpddr4.h"
#undef X_CON_SCE
};

/*
 * mtk_mem_bw_ctrl: set EMI bandwidth limiter for memory bandwidth control
 * @sce: concurrency scenario ID
 * @op: either ENABLE_CON_SCE or DISABLE_CON_SCE
 * Return 0 for success; return negative values for failure.
 */
int mtk_mem_bw_ctrl(int sce, int op)
{
	int i, highest;

	if (sce >= NR_CON_SCE)
		return -1;

	if (op != ENABLE_CON_SCE && op != DISABLE_CON_SCE)
		return -1;

	if (in_interrupt())
		return -1;

	down(&emi_bwl_sem);

	if (op == ENABLE_CON_SCE)
		ctrl_tbl[sce].ref_cnt++;

	else if (op == DISABLE_CON_SCE) {
		if (ctrl_tbl[sce].ref_cnt != 0)
			ctrl_tbl[sce].ref_cnt--;
	}

	/* find the scenario with the highest priority */
	highest = -1;
	for (i = 0; i < NR_CON_SCE; i++) {
		if (ctrl_tbl[i].ref_cnt != 0) {
			highest = i;
			break;
		}
	}
	if (highest == -1)
		highest = CON_SCE_UI;

	/* set new EMI bandwidth limiter value */
	if (highest != cur_con_sce) {
		if (get_dram_type()) {
			writel(emi_arba_lpddr4_val[highest], EMI_ARBA);
			writel(emi_arbc_lpddr4_val[highest], EMI_ARBC);
			writel(emi_arbd_lpddr4_val[highest], EMI_ARBD);
			writel(emi_arbe_lpddr4_val[highest], EMI_ARBE);
			writel(emi_arbf_lpddr4_val[highest], EMI_ARBF);
			writel(emi_arbg_lpddr4_val[highest], EMI_ARBG);
			mt_reg_sync_writel(emi_conm_lpddr4_val[highest], EMI_CONM);
		} else {
			pr_debug("[EMI BWL] undefined dram_type\n");
		}

		cur_con_sce = highest;
	}

	up(&emi_bwl_sem);

	return 0;
}

/*
 * con_sce_show: sysfs con_sce file show function.
 * @driver:
 * @buf:
 * Return the number of read bytes.
 */
static ssize_t con_sce_show(struct device_driver *driver, char *buf)
{
	char *ptr = buf;
	int i = 0;

	if (cur_con_sce >= NR_CON_SCE)
		ptr += sprintf(ptr, "none\n");
	else
		ptr += sprintf(ptr, "current scenario: %s\n",
		con_sce_str[cur_con_sce]);

#if 1
	ptr += sprintf(ptr, "%s\n", con_sce_str[cur_con_sce]);
	ptr += sprintf(ptr, "EMI_ARBA = 0x%x\n",  readl(IOMEM(EMI_ARBA)));
	ptr += sprintf(ptr, "EMI_ARBC = 0x%x\n",  readl(IOMEM(EMI_ARBC)));
	ptr += sprintf(ptr, "EMI_ARBD = 0x%x\n",  readl(IOMEM(EMI_ARBD)));
	ptr += sprintf(ptr, "EMI_ARBE = 0x%x\n",  readl(IOMEM(EMI_ARBE)));
	ptr += sprintf(ptr, "EMI_ARBF = 0x%x\n",  readl(IOMEM(EMI_ARBF)));
	ptr += sprintf(ptr, "EMI_ARBG = 0x%x\n",  readl(IOMEM(EMI_ARBG)));
	ptr += sprintf(ptr, "EMI_CONM = 0x%x\n",  readl(IOMEM(EMI_CONM)));
	for (i = 0; i < NR_CON_SCE; i++)
		ptr += sprintf(ptr, "%s = 0x%x\n", con_sce_str[i],
			ctrl_tbl[i].ref_cnt);

	pr_debug("[EMI BWL] EMI_ARBA = 0x%x\n", readl(IOMEM(EMI_ARBA)));
	pr_debug("[EMI BWL] EMI_ARBC = 0x%x\n", readl(IOMEM(EMI_ARBC)));
	pr_debug("[EMI BWL] EMI_ARBD = 0x%x\n", readl(IOMEM(EMI_ARBD)));
	pr_debug("[EMI BWL] EMI_ARBE = 0x%x\n", readl(IOMEM(EMI_ARBE)));
	pr_debug("[EMI BWL] EMI_ARBF = 0x%x\n", readl(IOMEM(EMI_ARBF)));
	pr_debug("[EMI BWL] EMI_ARBG = 0x%x\n", readl(IOMEM(EMI_ARBG)));
	pr_debug("[EMI BWL] EMI_CONM = 0x%x\n", readl(IOMEM(EMI_CONM)));
#endif

	return strlen(buf);

}

/*
 * con_sce_store: sysfs con_sce file store function.
 * @driver:
 * @buf:
 * @count:
 * Return the number of write bytes.
 */
static ssize_t con_sce_store(struct device_driver *driver,
const char *buf, size_t count)
{
	int i;

	for (i = 0; i < NR_CON_SCE; i++) {
		if (!strncmp(buf, con_sce_str[i], strlen(con_sce_str[i]))) {
			if (!strncmp(buf + strlen(con_sce_str[i]) + 1,
				EN_CON_SCE_STR, strlen(EN_CON_SCE_STR))) {

				mtk_mem_bw_ctrl(i, ENABLE_CON_SCE);
				pr_debug("concurrency scenario %s ON\n", con_sce_str[i]);
				break;
			} else if (!strncmp(buf + strlen(con_sce_str[i]) + 1,
				DIS_CON_SCE_STR, strlen(DIS_CON_SCE_STR))) {

				mtk_mem_bw_ctrl(i, DISABLE_CON_SCE);
				pr_debug("concurrency scenario %s OFF\n", con_sce_str[i]);
				break;
			}
		}
	}

	return count;
}

DRIVER_ATTR(concurrency_scenario, 0644, con_sce_show, con_sce_store);

static ssize_t dump_latency_ctrl_show(struct device_driver *driver, char *buf)
{
	char *ptr;

	ptr = (char *)buf;
	ptr += sprintf(ptr, "dump_latency_ctrl_show: is_dump_latency: %d\n", is_dump_latency());

#if 0
	/* test for dump_emi_latency */
	dump_emi_latency();
#endif

	return strlen(buf);
}

static ssize_t dump_latency_ctrl_store(struct device_driver *driver,
	const char *buf, size_t count)
{
	if (!strncmp(buf, "ON", strlen("ON")))
		enable_dump_latency();
	else if (!strncmp(buf, "OFF", strlen("OFF"))) {
		disable_dump_latency();
		disable_elm();
	} else
		pr_debug("Unknown dump latency command.\n");

	pr_debug("dump_latency_ctrl_store: is_dump_latency: %d\n", is_dump_latency());

	return count;
}

DRIVER_ATTR(dump_latency_ctrl, 0644, dump_latency_ctrl_show, dump_latency_ctrl_store);

static ssize_t elm_ctrl_show(struct device_driver *driver, char *buf)
{
	char *ptr;

	ptr = (char *)buf;
	ptr += sprintf(ptr, "ELM enabled: %d\n", is_elm_enabled());

	return strlen(buf);
}

static ssize_t elm_ctrl_store(struct device_driver *driver,
			      const char *buf, size_t count)
{
	if (!strncmp(buf, "ON", strlen("ON")))
		enable_elm();
	else if (!strncmp(buf, "OFF", strlen("OFF")))
		disable_elm();

	return count;
}

DRIVER_ATTR(elm_ctrl, 0644, elm_ctrl_show, elm_ctrl_store);

/*
 * emi_ctrl_init: module init function.
 */
static int __init emi_ctrl_init(void)
{
	int ret;
	int i;

	/* register EMI ctrl interface */
	ret = platform_driver_register(&emi_ctrl);
	if (ret)
		pr_debug("[EMI/BWL] fail to register emi_ctrl driver\n");

	ret = driver_create_file(&emi_ctrl.driver, &driver_attr_concurrency_scenario);
	if (ret)
		pr_debug("[EMI/BWL] fail to create emi_bwl sysfs file\n");

	ret = driver_create_file(&emi_ctrl.driver, &driver_attr_dump_latency_ctrl);
	if (ret)
		pr_debug("[EMI/MBW] fail to create dump_latency_ctrl file\n");

	ret = driver_create_file(&emi_ctrl.driver, &driver_attr_elm_ctrl);
	if (ret)
		pr_info("[EMI/ELM] fail to create elm_ctrl file\n");

	/* get EMI info from boot tags */
	if (of_chosen) {
		ret = of_property_read_u32(of_chosen, "emi_info,dram_type", &(emi_info.dram_type));
		if (ret)
			pr_debug("[EMI] fail to get dram_type\n");
		ret = of_property_read_u32(of_chosen, "emi_info,ch_num", &(emi_info.ch_num));
		if (ret)
			pr_debug("[EMI] fail to get ch_num\n");
		ret = of_property_read_u32(of_chosen, "emi_info,rk_num", &(emi_info.rk_num));
		if (ret)
			pr_debug("[EMI] fail to get rk_num\n");
		ret = of_property_read_u32_array(of_chosen, "emi_info,rank_size",
			emi_info.rank_size, MAX_RK);
		if (ret)
			pr_debug("[EMI] fail to get rank_size\n");
	}

	pr_debug("[EMI] dram_type(%d)\n", get_dram_type());
	pr_debug("[EMI] ch_num(%d)\n", get_ch_num());
	pr_debug("[EMI] rk_num(%d)\n", get_rk_num());
	for (i = 0; i < get_rk_num(); i++)
		pr_debug("[EMI] rank%d_size(0x%x)", i, get_rank_size(i));

	return 0;
}

unsigned int mt_emi_elm_irq_get(void)
{
	return elm_irq;
}

unsigned int mt_emi_cgm_irq_get(void)
{
	return cgm_irq;
}

/*
 * emi_ctrl_exit: module exit function.
 */
static void __exit emi_ctrl_exit(void)
{
}

/* EXPORT_SYMBOL(get_dram_type); */

postcore_initcall(emi_ctrl_init);
module_exit(emi_ctrl_exit);

unsigned int get_dram_type(void)
{
	return emi_info.dram_type;
}

unsigned int get_ch_num(void)
{
	return emi_info.ch_num;
}

unsigned int get_rk_num(void)
{
	if (emi_info.rk_num > MAX_RK)
		pr_debug("[EMI] rank overflow\n");

	return emi_info.rk_num;
}

unsigned int get_rank_size(unsigned int rank_index)
{
	if (rank_index < emi_info.rk_num)
		return emi_info.rank_size[rank_index];

	return 0;
}

void __iomem *mt_cen_emi_base_get(void)
{
	return CEN_EMI_BASE;
}
EXPORT_SYMBOL(mt_cen_emi_base_get);

void __iomem *mt_emi_base_get(void)
{
	return mt_cen_emi_base_get();
}
EXPORT_SYMBOL(mt_emi_base_get);

void __iomem *mt_chn_emi_base_get(int chn)
{
	switch (chn) {
	case 0:
		return CHA_EMI_BASE;
	case 1:
		return CHB_EMI_BASE;
	default:
		return NULL;
	}
}
EXPORT_SYMBOL(mt_chn_emi_base_get);

void __iomem *mt_emi_mpu_base_get(void)
{
	return EMI_MPU_BASE;
}
EXPORT_SYMBOL(mt_emi_mpu_base_get);

void __iomem *mt_emi_elm_dbg_base_get(void)
{
	return EMI_ELM_DBG_BASE;
}

unsigned int mt_emi_mpu_irq_get(void)
{
	return mpu_irq;
}

int disable_drs(unsigned char *backup)
{
	int count;
	unsigned int drs_status;
	unsigned long flags;

	if ((CHA_EMI_BASE == NULL) || (CHB_EMI_BASE == NULL)) {
		pr_debug("[EMI] can not get base to disable DRS\n");
		return -1;
	}

	spin_lock_irqsave(&emi_drs_lock, flags);
	if (force_drs_disable) {
		spin_unlock_irqrestore(&emi_drs_lock, flags);
		return -1;
	}
	force_drs_disable = true;
	spin_unlock_irqrestore(&emi_drs_lock, flags);

	*backup = (readl(IOMEM(CHA_EMI_DRS)) << 4) & 0x10;
	*backup |= (readl(IOMEM(CHB_EMI_DRS)) & 0x01);

	writel(readl(IOMEM(CHA_EMI_DRS)) & ~0x1, IOMEM(CHA_EMI_DRS));
	writel(readl(IOMEM(CHB_EMI_DRS)) & ~0x1, IOMEM(CHB_EMI_DRS));

	for (count = 100; count > 0; count--) {
		drs_status = readl(IOMEM(CHA_EMI_DRS_ST5));
		if ((drs_status == 0x10) || (drs_status == 0x40))
			continue;

		drs_status = readl(IOMEM(CHB_EMI_DRS_ST5));
		if ((drs_status != 0x10) && (drs_status != 0x40))
			break;
	}

	if (count == 0) {
		restore_drs(*backup);
		/* pr_debug("[EMI] disable DRS fail\n"); */
		return -1;
	}

	return 0;
}

void restore_drs(unsigned char enable)
{
	unsigned long flags;

	if ((CHA_EMI_BASE == NULL) || (CHB_EMI_BASE == NULL)) {
		pr_debug("[EMI] can not get base to enable DRS\n");
		goto out;
	}

	writel(readl(IOMEM(CHA_EMI_DRS)) | ((enable >> 4) & 0x1),
		IOMEM(CHA_EMI_DRS));
	writel(readl(IOMEM(CHB_EMI_DRS)) | (enable & 0x1),
		IOMEM(CHB_EMI_DRS));

out:
	spin_lock_irqsave(&emi_drs_lock, flags);
	force_drs_disable = false;
	spin_unlock_irqrestore(&emi_drs_lock, flags);
}

bool is_drs_enabled(unsigned char ch)
{
	switch (ch) {
	case 0:
		if (readl(IOMEM(CHA_EMI_DRS)) & 0x1)
			return true;
		break;
	case 1:
		if (readl(IOMEM(CHB_EMI_DRS)) & 0x1)
			return true;
		break;
	}

	return false;
}

int DRS_enable(void)
{
	unsigned char status;
	unsigned int count;

	if (is_drs_enabled(0) && is_drs_enabled(1))
		return 0;

	count = 0;
	while (disable_drs(&status)) {
		udelay(100);
		if (count > 10000) {
			pr_debug("[EMI] waiting to enable DRS\n");
			count = 0;
		} else
			count++;
	}

	restore_drs(0x11);

	return 0;
}

int DRS_disable(void)
{
	unsigned char status;
	unsigned int count;

	if (!(is_drs_enabled(0) || is_drs_enabled(1)))
		return 0;

	count = 0;
	while (disable_drs(&status)) {
		udelay(100);
		if (count > 10000) {
			pr_debug("[EMI] waiting to disable DRS\n");
			count = 0;
		} else
			count++;
	}

	restore_drs(0x00);

	return 0;
}

unsigned long long get_drs_all_self_cnt(unsigned int ch)
{
	unsigned long long cnt = 0;

	switch (ch) {
	case 0:
		cnt = (readl(IOMEM(CHA_EMI_DRS_ST4)) & 0x3fffff);
		break;
	case 1:
		cnt = (readl(IOMEM(CHB_EMI_DRS_ST4)) & 0x3fffff);
		break;
	default:
		cnt = (readl(IOMEM(CHA_EMI_DRS_ST4)) & 0x3fffff);
		pr_info("[EMI] wrong channel(%d) for CHA_EMI_DRS_ST4\n", ch);
		break;
	}

	/* unit:38.5ns , transfer to 38500 ps */
	cnt = cnt * 38500;
	pr_warn("[EMI] all self-refresh count: 0x%llx\n", cnt);

	return cnt;
}

unsigned long long get_drs_rank1_self_cnt(unsigned int ch)
{
	unsigned long long cnt = 0;

	switch (ch) {
	case 0:
		cnt = (readl(IOMEM(CHA_EMI_DRS_ST3)) & 0x3fffff);
		break;
	case 1:
		cnt = (readl(IOMEM(CHB_EMI_DRS_ST3)) & 0x3fffff);
		break;
	default:
		cnt = (readl(IOMEM(CHA_EMI_DRS_ST3)) & 0x3fffff);
		pr_debug("[EMI] wrong channel(%d) for CHA_EMI_DRS_ST3\n", ch);
		break;
	}

	/* unit:38.5ns , transfer to 38500 ps */
	cnt = cnt * 38500;
	pr_info("[EMI] rank1 self-refresh count: 0x%llx\n", cnt);

	return cnt;
}

unsigned int mask_master_disable_drs(unsigned int master)
{
	unsigned int tmp;

	tmp = (readl(IOMEM(CHA_EMI_DRS)) >> 20) & 0xf;
	master = (~master & tmp) << 20;

	tmp = readl(IOMEM(CHA_EMI_DRS)) & ~(0xf << 20);
	writel(tmp | master, IOMEM(CHA_EMI_DRS));

	tmp = readl(IOMEM(CHB_EMI_DRS)) & ~(0xf << 20);
	writel(tmp | master, IOMEM(CHB_EMI_DRS));

	return 0;
}

unsigned int unmask_master_disable_drs(unsigned int master)
{
	unsigned int tmp;

	tmp = (readl(IOMEM(CHA_EMI_DRS)) >> 20) & 0xf;
	master = (master | tmp) << 20;

	tmp = readl(IOMEM(CHA_EMI_DRS)) & ~(0xf << 20);
	writel(tmp | master, IOMEM(CHA_EMI_DRS));

	tmp = readl(IOMEM(CHB_EMI_DRS)) & ~(0xf << 20);
	writel(tmp | master, IOMEM(CHB_EMI_DRS));

	return 0;
}

unsigned long long get_drs_rank_prd(unsigned int ch)
{
	unsigned long long rank_drs_prd = 0;

	switch (ch) {
	case 0:
		rank_drs_prd = (readl(IOMEM(CHA_EMI_DRS_MON0)) >> 1) & 0x7;
		break;
	case 1:
		rank_drs_prd = (readl(IOMEM(CHB_EMI_DRS_MON0)) >> 1) & 0x7;
		break;
	default:
		rank_drs_prd = (readl(IOMEM(CHA_EMI_DRS_MON0)) >> 1) & 0x7;
		pr_debug("[EMI] wrong channel(%d) for CHN_EMI_DRS_MON0\n", ch);
		break;
	}

	/*DRS monitor period = 16ms* (RANK_DRS_PRD+1)*/
	rank_drs_prd = ((rank_drs_prd + 1) << 4);
	pr_warn("[EMI] DRS monitor period: 0x%llx ms\n", rank_drs_prd);

	return rank_drs_prd;
}
