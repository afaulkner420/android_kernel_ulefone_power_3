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

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include "ccci_config.h"
#include <linux/clk.h>
#include <mach/mtk_pbm.h>
#include <mach/emi_mpu.h>

#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
#include <mach/mt6605.h>
#endif

#include "include/pmic_api_buck.h"
#include <mt-plat/upmu_common.h>
#include <mtk_spm_sleep.h>
#include "ccci_core.h"
#include "ccci_platform.h"

#include "md_sys1_platform.h"
#include "cldma_reg.h"
#include "modem_reg_base.h"
/*#include "mtk_clkbuf_ctl.h"*/
static struct ccci_clk_node clk_table[] = {
	{ NULL,	"scp-sys-md1-main"},
	{ NULL,	"infra-cldma-ap"},
	{ NULL,	"infra-cldma-ap-hclk"},
	{ NULL,	"infra-ccif-ap"},
	{ NULL,	"infra-ccif-md"},
	{ NULL,	"infra-ap-c2k-ccif-0"},
	{ NULL,	"infra-ap-c2k-ccif-1"},
	{ NULL,	"infra-scp-md1-ccif-0"},
	{ NULL,	"infra-scp-md1-ccif-1"},
	{ NULL,	"infra-scp-c2k-ccif-0"},
	{ NULL,	"infra-scp-c2k-ccif-1"},
	{ NULL,	"infra-md2md-ccif-0"},
	{ NULL,	"infra-md2md-ccif-1"},
	{ NULL,	"infra-md2md-ccif-2"},
	{ NULL,	"infra-md2md-ccif-3"},
	{ NULL,	"infra-md2md-ccif-4"},
	{ NULL,	"infra-md2md-ccif-5"},
};
#if defined(CONFIG_PINCTRL_ELBRUS)
static struct pinctrl *mdcldma_pinctrl;
#endif

static void __iomem *md_sram_pd_psmcusys_base;

#define TAG "mcd"
void md_cldma_hw_reset(unsigned char md_id)
{
	unsigned int reg_value;

	CCCI_DEBUG_LOG(md_id, TAG, "md_cldma_hw_reset:rst cldma\n");
	/* reset cldma hw: AO Domain */
	reg_value = ccci_read32(infra_ao_base, INFRA_RST0_REG_AO);
	reg_value &= ~(CLDMA_AO_RST_MASK); /* the bits in reg is WO, */
	reg_value |= (CLDMA_AO_RST_MASK);/* so only this bit effective */
	ccci_write32(infra_ao_base, INFRA_RST0_REG_AO, reg_value);
	CCCI_BOOTUP_LOG(md_id, TAG, "md_cldma_hw_reset:clear reset\n");
	/* reset cldma clr */
	reg_value = ccci_read32(infra_ao_base, INFRA_RST1_REG_AO);
	reg_value &= ~(CLDMA_AO_RST_MASK);/* read no use, maybe a time delay */
	reg_value |= (CLDMA_AO_RST_MASK);
	ccci_write32(infra_ao_base, INFRA_RST1_REG_AO, reg_value);
	CCCI_BOOTUP_LOG(md_id, TAG, "md_cldma_hw_reset:done\n");

	/* reset cldma hw: PD Domain */
	reg_value = ccci_read32(infra_ao_base, INFRA_RST0_REG_PD);
	reg_value &= ~(CLDMA_PD_RST_MASK);
	reg_value |= (CLDMA_PD_RST_MASK);
	ccci_write32(infra_ao_base, INFRA_RST0_REG_PD, reg_value);
	CCCI_BOOTUP_LOG(md_id, TAG, "md_cldma_hw_reset:clear reset\n");
	/* reset cldma clr */
	reg_value = ccci_read32(infra_ao_base, INFRA_RST1_REG_PD);
	reg_value &= ~(CLDMA_PD_RST_MASK);
	reg_value |= (CLDMA_PD_RST_MASK);
	ccci_write32(infra_ao_base, INFRA_RST1_REG_PD, reg_value);
	CCCI_DEBUG_LOG(md_id, TAG, "md_cldma_hw_reset:done\n");
}

int md_cd_get_modem_hw_info(struct platform_device *dev_ptr, struct ccci_dev_cfg *dev_cfg, struct md_hw_info *hw_info)
{
	struct device_node *node = NULL;
	int idx = 0;
	struct cldma_hw_info *cldma_hw;

	if (dev_ptr->dev.of_node == NULL) {
		CCCI_ERROR_LOG(0, TAG, "modem OF node NULL\n");
		return -1;
	}

	memset(dev_cfg, 0, sizeof(struct ccci_dev_cfg));
	of_property_read_u32(dev_ptr->dev.of_node, "mediatek,md_id", &dev_cfg->index);
	CCCI_DEBUG_LOG(dev_cfg->index, TAG, "modem hw info get idx:%d\n", dev_cfg->index);

	cldma_hw = kzalloc(sizeof(struct cldma_hw_info), GFP_KERNEL);
	if (cldma_hw == NULL) {
		CCCI_ERROR_LOG(-1, TAG, "md_cd_get_modem_hw_info:alloc cldma hw mem fail\n");
		return -1;
	}

	memset(hw_info, 0, sizeof(struct md_hw_info));
	hw_info->hif_hw_info = cldma_hw;

	switch (dev_cfg->index) {
	case 0:		/* MD_SYS1 */
		dev_cfg->major = 0;
		dev_cfg->minor_base = 0;
		of_property_read_u32(dev_ptr->dev.of_node, "mediatek,cldma_capability", &dev_cfg->capability);

		cldma_hw->cldma_ap_ao_base = (unsigned long)of_iomap(dev_ptr->dev.of_node, 0);
		cldma_hw->cldma_md_ao_base = (unsigned long)of_iomap(dev_ptr->dev.of_node, 1);
		cldma_hw->cldma_ap_pdn_base = (unsigned long)of_iomap(dev_ptr->dev.of_node, 2);
		cldma_hw->cldma_md_pdn_base = (unsigned long)of_iomap(dev_ptr->dev.of_node, 3);
		hw_info->ap_ccif_base = (unsigned long)of_iomap(dev_ptr->dev.of_node, 4);
		hw_info->md_ccif_base = (unsigned long)of_iomap(dev_ptr->dev.of_node, 5);
		cldma_hw->cldma_irq_id = irq_of_parse_and_map(dev_ptr->dev.of_node, 0);
		hw_info->ap_ccif_irq_id = irq_of_parse_and_map(dev_ptr->dev.of_node, 1);
		hw_info->md_wdt_irq_id = irq_of_parse_and_map(dev_ptr->dev.of_node, 2);

		hw_info->md_pcore_pccif_base = ioremap_nocache(MD_PCORE_PCCIF_BASE, 0x20);
		CCCI_BOOTUP_LOG(dev_cfg->index, TAG, "pccif:%x\n", MD_PCORE_PCCIF_BASE);

		/* Device tree using none flag to register irq, sensitivity has set at "irq_of_parse_and_map" */
		cldma_hw->cldma_irq_flags = IRQF_TRIGGER_NONE;
		hw_info->ap_ccif_irq_flags = IRQF_TRIGGER_NONE;
		hw_info->md_wdt_irq_flags = IRQF_TRIGGER_NONE;
		hw_info->ap2md_bus_timeout_irq_flags = IRQF_TRIGGER_NONE;

		hw_info->sram_size = CCIF_SRAM_SIZE;
		hw_info->md_rgu_base = MD_RGU_BASE;
		hw_info->md_boot_slave_En = MD_BOOT_VECTOR_EN;
#if defined(CONFIG_PINCTRL_ELBRUS)
		mdcldma_pinctrl = devm_pinctrl_get(&dev_ptr->dev);
		if (IS_ERR(mdcldma_pinctrl)) {
			CCCI_ERROR_LOG(dev_cfg->index, TAG, "modem %d get mdcldma_pinctrl failed\n",
							dev_cfg->index + 1);
			return -1;
		}
#else
		CCCI_ERROR_LOG(dev_cfg->index, TAG, "gpio pinctrl is not ready yet, use workaround.\n");
#endif
		for (idx = 0; idx < ARRAY_SIZE(clk_table); idx++) {
			clk_table[idx].clk_ref = devm_clk_get(&dev_ptr->dev, clk_table[idx].clk_name);
			if (IS_ERR(clk_table[idx].clk_ref)) {
				CCCI_ERROR_LOG(dev_cfg->index, TAG, "md%d get %s failed\n",
								dev_cfg->index + 1, clk_table[idx].clk_name);
				clk_table[idx].clk_ref = NULL;
			}
		}
		node = of_find_compatible_node(NULL, NULL, "mediatek,apmixedsys");
		hw_info->ap_mixed_base = (unsigned long)of_iomap(node, 0);
		/* set DMA support bit range */
		if (of_property_read_u32(dev_ptr->dev.of_node, "dma_addrmask", &(hw_info->dma_addrmask))) {
			CCCI_ERROR_LOG(dev_cfg->index, TAG, "CLDMA get dma_addrmask fail failed\n");
			return -ENODEV;
		}
		/* Support >4GB SPI DMA address */
		if (dma_set_mask(&(dev_ptr->dev), DMA_BIT_MASK(hw_info->dma_addrmask))) {
			CCCI_ERROR_LOG(dev_cfg->index, TAG,
				"CLDMA dma_set_mask failed, dma_addrmask = %u\n", hw_info->dma_addrmask);
			return -ENODEV;
		}
		CCCI_ERROR_LOG(dev_cfg->index, TAG, "CLDMA get dma_addrmask done!!!!\n");
		break;
	default:
		return -1;
	}

	if (cldma_hw->cldma_ap_ao_base == 0 ||
		cldma_hw->cldma_md_ao_base == 0 ||
		cldma_hw->cldma_ap_pdn_base == 0 ||
		cldma_hw->cldma_md_pdn_base  == 0) {
		CCCI_ERROR_LOG(dev_cfg->index, TAG,
			"ap_cldma: ao_base=0x%p, pdn_base=0x%p,md_cldma: ao_base=0x%p, pdn_base=0x%p\n",
			(void *)cldma_hw->cldma_ap_ao_base, (void *)cldma_hw->cldma_ap_pdn_base,
			(void *)cldma_hw->cldma_md_ao_base, (void *)cldma_hw->cldma_md_pdn_base);
		return -1;
	}
	if (hw_info->ap_ccif_base == 0 ||
		hw_info->md_ccif_base == 0) {
		CCCI_ERROR_LOG(dev_cfg->index, TAG, "ap_ccif_base:0x%p, md_ccif_base:0x%p\n",
			(void *)hw_info->ap_ccif_base, (void *)hw_info->md_ccif_base);
		return -1;
	}
	if (cldma_hw->cldma_irq_id == 0 ||
		hw_info->ap_ccif_irq_id == 0 ||
		hw_info->md_wdt_irq_id == 0) {
		CCCI_ERROR_LOG(dev_cfg->index, TAG, "cldma_irq:%d,ccif_irq:%d,md_wdt_irq:%d\n",
			cldma_hw->cldma_irq_id, hw_info->ap_ccif_irq_id, hw_info->md_wdt_irq_id);
		return -1;
	}

	CCCI_BOOTUP_LOG(dev_cfg->index, TAG, "dev_major:%d,minor_base:%d,capability:%d\n",
							dev_cfg->major,
		     dev_cfg->minor_base, dev_cfg->capability);
	CCCI_BOOTUP_LOG(dev_cfg->index, TAG,
		     "ap_cldma: ao_base=0x%p, pdn_base=0x%p,md_cldma: ao_base=0x%p, pdn_base=0x%p\n",
		(void *)cldma_hw->cldma_ap_ao_base, (void *)cldma_hw->cldma_ap_pdn_base,
		(void *)cldma_hw->cldma_md_ao_base, (void *)cldma_hw->cldma_md_pdn_base);

	CCCI_BOOTUP_LOG(dev_cfg->index, TAG, "ap_ccif_base:0x%p, md_ccif_base:0x%p\n",
					(void *)hw_info->ap_ccif_base, (void *)hw_info->md_ccif_base);
	CCCI_BOOTUP_LOG(dev_cfg->index, TAG, "cldma_irq:%d,ccif_irq:%d,md_wdt_irq:%d\n",
		cldma_hw->cldma_irq_id, hw_info->ap_ccif_irq_id, hw_info->md_wdt_irq_id);

	if (!get_modem_is_enabled(dev_cfg->index)) {
		CCCI_ERROR_LOG(dev_cfg->index, TAG, "modem %d not enable, exit\n", dev_cfg->index + 1);
		return -1;
	}

	return 0;
}

/* md1 sys_clk_cg no need set in this API*/
void ccci_set_clk_cg(struct ccci_modem *md, unsigned int on)
{
	int idx = 0;
	int ret = 0;

	CCCI_NORMAL_LOG(md->index, TAG, "%s: on=%d\n", __func__, on);
	for (idx = 1; idx < ARRAY_SIZE(clk_table); idx++) {
		if (clk_table[idx].clk_ref == NULL)
			continue;
		if (on) {
			ret = clk_prepare_enable(clk_table[idx].clk_ref);
			if (ret)
				CCCI_ERROR_LOG(md->index, TAG, "%s: on=%d,ret=%d\n", __func__, on, ret);
		} else
			clk_disable_unprepare(clk_table[idx].clk_ref);
	}
}

int md_cd_io_remap_md_side_register(struct ccci_modem *md)
{
	struct md_pll_reg *md_reg;
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;

	md_info->md_boot_slave_En = ioremap_nocache(md->hw_info->md_boot_slave_En, 0x4);
	md_info->md_rgu_base = ioremap_nocache(md->hw_info->md_rgu_base, 0x300);
	md_info->l1_rgu_base = ioremap_nocache(md->hw_info->l1_rgu_base, 0x40);
	md_info->md_global_con0 = ioremap_nocache(MD_GLOBAL_CON0, 0x4);


	md_reg = kzalloc(sizeof(struct md_pll_reg), GFP_KERNEL);
	if (md_reg == NULL) {
		CCCI_ERROR_LOG(-1, TAG, "cldma_sw_init:alloc md reg map mem fail\n");
		return -1;
	}
	/*needed by bootup flow start*/
	md_reg->md_top_Pll = ioremap_nocache(MDTOP_PLLMIXED_BASE, MDTOP_PLLMIXED_LENGTH);
	md_reg->md_top_clkSW = ioremap_nocache(MDTOP_CLKSW_BASE, MDTOP_CLKSW_LENGTH);
	md_reg->md_usb_int_mux = ioremap_nocache(MD_USB_INTERRUPT_MUX_BASE, MD_USB_INTERRUPT_MUX_LENGTH);
	/*needed by bootup flow end*/

	/*just for dump begin*/
	md_reg->md_peri_misc = ioremap_nocache(MD_PERI_MISC_BASE, MD_PERI_MISC_LEN);
	md_reg->md_L1_a0 = ioremap_nocache(MDL1A0_BASE, MDL1A0_LEN);

	md_reg->md_sys_clk = ioremap_nocache(MDSYS_CLKCTL_BASE, MDSYS_CLKCTL_LEN);
	md_reg->md_dbg_sys = ioremap_nocache(MD1_OPEN_DEBUG_APB_CLK, 4);
	md_reg->md_pc_monc0 = ioremap_nocache(MD1_PC_MONITOR_BASEC0, MD1_PC_MONITOR_C_LEN0);
	md_reg->md_pc_monc1 = ioremap_nocache(MD1_PC_MONITOR_BASEC1, MD1_PC_MONITOR_C_LEN1);
	md_reg->md_pc_monc2 = ioremap_nocache(MD1_PC_MONITOR_BASEC2, MD1_PC_MONITOR_C_LEN2);
	md_reg->md_pc_monc3 = ioremap_nocache(MD1_PC_MONITOR_BASEC3, MD1_PC_MONITOR_C_LEN3);
	md_reg->md_pc_monc4 = ioremap_nocache(MD1_PC_MONITOR_BASEC4, MD1_PC_MONITOR_C_LEN4);

	md_reg->md_pc_mon0 = ioremap_nocache(MD1_PC_MONITOR_BASE0_1, MD1_PC_MONITOR_LEN0_1);
	md_reg->md_pc_mon1 = ioremap_nocache(MD1_PC_MONITOR_BASE1_1, MD1_PC_MONITOR_LEN1_1);
	md_reg->md_pc_mon2 = ioremap_nocache(MD1_PC_MONITOR_BASE2_1, MD1_PC_MONITOR_LEN2_1);
	md_reg->md_pc_mon3 = ioremap_nocache(MD1_PC_MONITOR_BASE3_1, MD1_PC_MONITOR_LEN3_1);

	md_reg->md_clkSW = ioremap_nocache(MD_CLKSW_BASE, MD_CLKSW_LENGTH);
	md_reg->md_clkSw0 = md_reg->md_clkSW;
	md_reg->md_clkSw1 = ioremap_nocache(MD1_CLKSW_BASE1, MD1_CLKSW_LEN1);
	md_reg->md_clkSw2 = ioremap_nocache(MD1_CLKSW_BASE2, MD1_CLKSW_LEN2);
	md_reg->md_clkSw3 = ioremap_nocache(MD1_CLKSW_BASE3, MD1_CLKSW_LEN3);
	md_reg->md_pll_mixed_0 = ioremap_nocache(MD1_PLLMIXED_BASE0, MD1_PLLMIXED_LEN0);
	md_reg->md_pll_mixed_1 = ioremap_nocache(MD1_PLLMIXED_BASE1, MD1_PLLMIXED_LEN1);
	md_reg->md_pll_mixed_2 = ioremap_nocache(MD1_PLLMIXED_BASE2, MD1_PLLMIXED_LEN1);
	md_reg->md_pll_mixed_3 = ioremap_nocache(MD1_PLLMIXED_BASE3, MD1_PLLMIXED_LEN3);
	md_reg->md_pll_mixed_4 = ioremap_nocache(MD1_PLLMIXED_BASE4, MD1_PLLMIXED_LEN4);
	md_reg->md_pll_mixed_5 = ioremap_nocache(MD1_PLLMIXED_BASE5, MD1_PLLMIXED_LEN5);
	md_reg->md_pll_mixed_6 = ioremap_nocache(MD1_PLLMIXED_BASE6, MD1_PLLMIXED_LEN6);
	md_reg->md_pll_mixed_7 = ioremap_nocache(MD1_PLLMIXED_BASE7, MD1_PLLMIXED_LEN7);
	md_reg->md_pll_mixed_8 = ioremap_nocache(MD1_PLLMIXED_BASE8, MD1_PLLMIXED_LEN8);
	md_reg->md_pll_mixed_9 = ioremap_nocache(MD1_PLLMIXED_BASE9, MD1_PLLMIXED_LEN9);
	md_reg->md_pll_mixed_A = ioremap_nocache(MD1_PLLMIXED_BASEA, MD1_PLLMIXED_LENA);
	md_reg->md_clk_ctl_0 = ioremap_nocache(MD1_CLKCTL_BASE0, MD1_CLKCTL_LEN0);
	md_reg->md_clk_ctl_1 = ioremap_nocache(MD1_CLKCTL_BASE1, MD1_CLKCTL_LEN1);
	md_reg->md_clk_ctl_2 = ioremap_nocache(MD1_CLKCTL_BASE2, MD1_CLKCTL_LEN2);
	md_reg->md_clk_ctl_3 = ioremap_nocache(MD1_CLKCTL_BASE3, MD1_CLKCTL_LEN3);
	md_reg->md_global_con_0 = ioremap_nocache(MD1_GLOBALCON_BASE0, MD1_GLOBALCON_LEN0);
	md_reg->md_global_con_1 = ioremap_nocache(MD1_GLOBALCON_BASE1, MD1_GLOBALCON_LEN1);
	md_reg->md_global_con_2 = ioremap_nocache(MD1_GLOBALCON_BASE2, MD1_GLOBALCON_LEN2);
	md_reg->md_global_con_3 = ioremap_nocache(MD1_GLOBALCON_BASE3, MD1_GLOBALCON_LEN3);
	md_reg->md_global_con_4 = ioremap_nocache(MD1_GLOBALCON_BASE4, MD1_GLOBALCON_LEN4);
	md_reg->md_global_con_5 = ioremap_nocache(MD1_GLOBALCON_BASE5, MD1_GLOBALCON_LEN5);
	md_reg->md_bus_reg_0 = ioremap_nocache(MD1_BUS_REG_BASE0, MD1_BUS_REG_LEN0);
	md_reg->md_bus_reg_1 = ioremap_nocache(MD1_BUS_REG_BASE1, MD1_BUS_REG_LEN1);
	md_reg->md_bus_reg_2 = ioremap_nocache(MD1_BUS_REG_BASE2, MD1_BUS_REG_LEN2);
	md_reg->md_bus_reg_3 = ioremap_nocache(MD1_BUS_REG_BASE3, MD1_BUS_REG_LEN3);
	md_reg->md_bus_reg_4 = ioremap_nocache(MD1_BUS_REG_BASE4, MD1_BUS_REG_LEN4);
	md_reg->md_bus_rec_0 = ioremap_nocache(MD1_MCU_MM_BUSREC_BASE0, MD1_MCU_MM_BUSREC_LEN0);
	md_reg->md_bus_rec_1 = ioremap_nocache(MD1_MCU_MO_BUSREC_BASE0, MD1_MCU_MO_BUSREC_LEN0);
	md_reg->md_bus_rec_2 = ioremap_nocache(MD1_INFRA_BUSREC_BASE0, MD1_INFRA_BUSREC_LEN0);
	md_reg->md_ect_0 = ioremap_nocache(MD1_ECT_REG_BASE0, MD1_ECT_REG_LEN0);
	md_reg->md_ect_1 = ioremap_nocache(MD1_ECT_REG_BASE1, MD1_ECT_REG_LEN1);
	md_reg->md_ect_2 = ioremap_nocache(MD1_ECT_REG_BASE2, MD1_ECT_REG_LEN2);
	md_reg->md_ect_3 = ioremap_nocache(MD1_ECT_REG_BASE3, MD1_ECT_REG_LEN3);
	md_reg->md_topsm_reg = ioremap_nocache(MD1_TOPSM_REG_BASE0, MD1_TOPSM_REG_LEN0);
	md_reg->md_rgu_reg_0 = ioremap_nocache(MD1_RGU_REG_BASE0, MD1_RGU_REG_LEN0);
	md_reg->md_rgu_reg_1 = ioremap_nocache(MD1_RGU_REG_BASE1, MD1_RGU_REG_LEN1);
	md_info->md_ost_status = ioremap_nocache(MD_OST_STATUS_BASE, MD_OST_STATUS_LENGTH);
	md_reg->md_ost_status = md_info->md_ost_status;

	md_reg->md_boot_stats0 = ioremap_nocache(MD1_CFG_BOOT_STATS0, 4);
	md_reg->md_boot_stats1 = ioremap_nocache(MD1_CFG_BOOT_STATS1, 4);
	/*just for dump end*/

	md_info->md_pll_base = md_reg;

	md_sram_pd_psmcusys_base = ioremap_nocache(MD_SRAM_PD_PSMCUSYS_SRAM_BASE, MD_SRAM_PD_PSMCUSYS_SRAM_LEN);

#ifdef MD_PEER_WAKEUP
	md_info->md_peer_wakeup = ioremap_nocache(MD_PEER_WAKEUP, 0x4);
#endif
	return 0;
}

void md_cd_lock_cldma_clock_src(int locked)
{
	/* spm_ap_mdsrc_req(locked); */
}

void md_cd_lock_modem_clock_src(int locked)
{
#ifdef SPM_API_READY
	spm_ap_mdsrc_req(locked);
#endif
}

void md_cd_dump_md_bootup_status(struct ccci_modem *md)
{
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;
	struct md_pll_reg *md_reg = md_info->md_pll_base;

	/*To avoid AP/MD interface delay, dump 3 times, and buy-in the 3rd dump value.*/

	cldma_read32(md_reg->md_boot_stats0, 0);	/* dummy read */
	cldma_read32(md_reg->md_boot_stats0, 0);	/* dummy read */
	CCCI_NOTICE_LOG(md->index, TAG, "md_boot_stats0:0x%X\n", cldma_read32(md_reg->md_boot_stats0, 0));

	cldma_read32(md_reg->md_boot_stats1, 0);	/* dummy read */
	cldma_read32(md_reg->md_boot_stats1, 0);	/* dummy read */
	CCCI_NOTICE_LOG(md->index, TAG, "md_boot_stats1:0x%X\n", cldma_read32(md_reg->md_boot_stats1, 0));
}

static int dump_emi_last_bm(struct ccci_modem *md)
{
	u32 buf_len = 1024;
	char *buf = NULL;

	buf = kzalloc(buf_len, GFP_ATOMIC);
	if (!buf) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "alloc memory failed for emi last bm\n");
		return -1;
	}

	dump_last_bm(buf, buf_len);
	CCCI_MEM_LOG_TAG(md->index, TAG, "Dump EMI last bm\n");
	ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, buf, buf_len);

	kfree(buf);

	return 0;
}

void md_cd_dump_debug_register(struct ccci_modem *md)
{
	/* MD no need dump because of bus hang happened - open for debug */
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;
	struct md_pll_reg *md_reg = md_info->md_pll_base;
	struct ccci_per_md *per_md_data = &md->per_md_data;

	/*dump_emi_latency();*/
	dump_emi_last_bm(md);

	/*CCCI_MEM_LOG(md->index, TAG, "Dump subsys_if_on\n");
	*subsys_if_on(); TODO: ask source clock for API ready, or close it
	*/

	md_cd_lock_modem_clock_src(1);

	/* 1. pre-action */
	if (per_md_data->md_dbg_dump_flag & (MD_DBG_DUMP_ALL & ~(1 << MD_DBG_DUMP_SMEM))) {
		ccci_write32(md_reg->md_dbg_sys, 0x00, 0x1);
		udelay(1000);
	} else {
		md_cd_lock_modem_clock_src(0);
		return;
	}
	/* 1. PC Monitor */
	if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_PCMON)) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD PC monitor\n");
		CCCI_MEM_LOG_TAG(md->index, TAG, "core0: [0]0x%X, [1]0x%X, [2]0x%x\n",
			MD1_PC_MONITOR_BASE0, MD1_PC_MONITOR_BASE1, (MD1_PC_MONITOR_BASE1 + 4));
		/* Stop all PCMon */
		ccci_write32(md_reg->md_pc_monc0, 0, 0x2222); /* stop MD PCMon */
		CCCI_MEM_LOG_TAG(md->index, TAG, "common: [0]0x%X, [1]0x%X, [2]0x%x, [3]0x%x, [4]0x%x\n",
			MD1_PC_MONITOR_BASEC0, MD1_PC_MONITOR_BASEC1, MD1_PC_MONITOR_BASEC2,
			MD1_PC_MONITOR_BASEC3, MD1_PC_MONITOR_BASEC4);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_monc0, MD1_PC_MONITOR_C_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_monc1, MD1_PC_MONITOR_C_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_monc2, MD1_PC_MONITOR_C_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_monc3, MD1_PC_MONITOR_C_LEN3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_monc4, MD1_PC_MONITOR_C_LEN4);

		CCCI_MEM_LOG_TAG(md->index, TAG, "core0/1/2/3: [0]0x%X, [1]0x%X, [2]0x%x, [3]0x%x\n",
			MD1_PC_MONITOR_BASE0_1, MD1_PC_MONITOR_BASE1_1,
			MD1_PC_MONITOR_BASE2_1, MD1_PC_MONITOR_BASE3_1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_mon0, 0x400);/* core0 */
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_mon1, 0x400);/* core1 */
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_mon2, 0x400);/* core2 */
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pc_mon3, 0x400);/* core3 */
		/* Resume PCMon */
		ccci_write32(md_reg->md_pc_monc0, 0, 0x1111);
	}
	/* 2. dump PLL */
	if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_PLL)) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD PLL\n");
		/* MD CLKSW */
		CCCI_MEM_LOG_TAG(md->index, TAG, "CLKSW: [0]0x%X, [1]0x%X, [2]0x%X, [3]0x%X\n",
			MD1_CLKSW_BASE0, MD1_CLKSW_BASE1, MD1_CLKSW_BASE2, MD1_CLKSW_BASE3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clkSW, MD1_CLKSW_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clkSw1, MD1_CLKSW_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clkSw2, MD1_CLKSW_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clkSw3, MD1_CLKSW_LEN3);
		/* MD PLLMIXED */
		CCCI_MEM_LOG_TAG(md->index, TAG,
		"PLLMIXED:[0]0x%X,[1]0x%X,[2]0x%X,[3]0x%X,[4]0x%X,[5]0x%X,[6]0x%X,[7]0x%X,[8]0x%X,[9]0x%X,[10]0x%X\n",
			MD1_PLLMIXED_BASE0, MD1_PLLMIXED_BASE1, MD1_PLLMIXED_BASE2, MD1_PLLMIXED_BASE3,
			MD1_PLLMIXED_BASE4, MD1_PLLMIXED_BASE5, MD1_PLLMIXED_BASE6, MD1_PLLMIXED_BASE7,
			MD1_PLLMIXED_BASE8, MD1_PLLMIXED_BASE9, MD1_PLLMIXED_BASEA);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_0, MD1_PLLMIXED_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_1, MD1_PLLMIXED_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_2, MD1_PLLMIXED_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_3, MD1_PLLMIXED_LEN3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_4, MD1_PLLMIXED_LEN4);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_5, MD1_PLLMIXED_LEN5);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_6, MD1_PLLMIXED_LEN6);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_7, MD1_PLLMIXED_LEN7);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_8, MD1_PLLMIXED_LEN8);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_9, MD1_PLLMIXED_LEN9);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_pll_mixed_A, MD1_PLLMIXED_LENA);
		/* MD CLKCTL */
		CCCI_MEM_LOG_TAG(md->index, TAG, "CLKCTL: [0]0x%X, [1]0x%X, [2]0x%X, [3]0x%X\n",
			MD1_CLKCTL_BASE0, MD1_CLKCTL_BASE1, MD1_CLKCTL_BASE2, MD1_CLKCTL_BASE3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clk_ctl_0, MD1_CLKCTL_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clk_ctl_1, MD1_CLKCTL_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clk_ctl_2, MD1_CLKCTL_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_clk_ctl_3, MD1_CLKCTL_LEN3);
		/* MD GLOBAL CON */
		CCCI_MEM_LOG_TAG(md->index, TAG,
		"GLOBAL CON: [0]0x%X, [1]0x%X, [2]0x%X, [3]0x%X, [4]0x%X, [5]0x%X\n",
			MD1_GLOBALCON_BASE0, MD1_GLOBALCON_BASE1, MD1_GLOBALCON_BASE2,
			MD1_GLOBALCON_BASE3, MD1_GLOBALCON_BASE4, MD1_GLOBALCON_BASE5);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_global_con_0, MD1_GLOBALCON_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_global_con_1, MD1_GLOBALCON_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_global_con_2, MD1_GLOBALCON_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_global_con_3, MD1_GLOBALCON_LEN3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_global_con_4, MD1_GLOBALCON_LEN4);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_global_con_5, MD1_GLOBALCON_LEN5);
	}
	/* 3. Bus status */
	if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_BUS)) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD Bus status: [0]0x%X, [1]0x%X, [2]0x%X, [3]0x%X, [4]0x%X\n",
			MD1_BUS_REG_BASE0, MD1_BUS_REG_BASE1, MD1_BUS_REG_BASE2,
			MD1_BUS_REG_BASE3, MD1_BUS_REG_BASE4);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_reg_0, MD1_BUS_REG_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_reg_1, MD1_BUS_REG_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_reg_2, MD1_BUS_REG_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_reg_3, MD1_BUS_REG_LEN3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_reg_4, MD1_BUS_REG_LEN4);
	}

	/* 4. Bus REC */
	if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_BUSREC)) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD Bus REC: [0]0x%X, [1]0x%X, [2]0x%X\n",
			MD1_MCU_MM_BUSREC_BASE0, MD1_MCU_MO_BUSREC_BASE0, MD1_INFRA_BUSREC_BASE0);
		ccci_write32(md_reg->md_bus_rec_0, 4, 0x1);/* pre-action: stop busrec */
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_rec_0, MD1_MCU_MM_BUSREC_LEN0);
		ccci_write32(md_reg->md_bus_rec_0, 0x4, 0x3);/* post-action: restart busrec */
		ccci_write32(md_reg->md_bus_rec_1, 4, 0x1);/* pre-action: stop busrec */
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_rec_1, MD1_MCU_MO_BUSREC_LEN0);
		ccci_write32(md_reg->md_bus_rec_1, 0x4, 0x3);/* post-action: restart busrec */
		ccci_write32(md_reg->md_bus_rec_2, 4, 0x1);/* pre-action: stop busrec */
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_bus_rec_2, MD1_INFRA_BUSREC_LEN0);
		ccci_write32(md_reg->md_bus_rec_2, 0x4, 0x3);/* post-action: restart busrec */
	}
	/* 5. ECT */
	if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_ECT)) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD ECT: [0]0x%X, [1]0x%X, [2]0x%X, [3]0x%X\n",
			MD1_ECT_REG_BASE0, MD1_ECT_REG_BASE1, MD1_ECT_REG_BASE2, MD1_ECT_REG_BASE3);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_ect_0, MD1_ECT_REG_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_ect_1, MD1_ECT_REG_LEN1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_ect_2, MD1_ECT_REG_LEN2);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_ect_3, MD1_ECT_REG_LEN3);
	}
#if 1 /* close register dump for HWT */
	/* 6. TOPSM */
	/* if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_TOPSM)) { */
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD TOPSM status: 0x%X\n", MD1_TOPSM_REG_BASE0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_topsm_reg, MD1_TOPSM_REG_LEN0);
	/*} */

	/* 7. MD RGU */
	/* if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_MDRGU)) { */
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD RGU: [0]0x%X, [1]0x%X\n",
			MD1_RGU_REG_BASE0, MD1_RGU_REG_BASE1);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_rgu_reg_0, MD1_RGU_REG_LEN0);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_rgu_reg_1, MD1_RGU_REG_LEN1);
	/*} */
	/* 8 OST */
	/* if (per_md_data->md_dbg_dump_flag & (1 << MD_DBG_DUMP_OST)) { */
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD OST status: 0x%X\n", MD_OST_STATUS_BASE);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP, md_reg->md_ost_status, MD_OST_STATUS_LENGTH);
	/*} */
#endif

	md_cd_lock_modem_clock_src(0);

}

int md_cd_pccif_send(struct ccci_modem *md, int channel_id)
{
	int busy = 0;
	struct md_hw_info *hw_info = md->hw_info;

	md_cd_lock_modem_clock_src(1);

	busy = cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_BUSY);
	if (busy & (1 << channel_id)) {
		md_cd_lock_modem_clock_src(0);
		return -1;
	}
	cldma_write32(hw_info->md_pcore_pccif_base, APCCIF_BUSY, 1 << channel_id);
	cldma_write32(hw_info->md_pcore_pccif_base, APCCIF_TCHNUM, channel_id);

	md_cd_lock_modem_clock_src(0);
	return 0;
}

void md_cd_dump_pccif_reg(struct ccci_modem *md)
{
	struct md_hw_info *hw_info = md->hw_info;

	md_cd_lock_modem_clock_src(1);

	CCCI_MEM_LOG_TAG(md->index, TAG, "AP_CON(%p)=%x\n", hw_info->md_pcore_pccif_base + APCCIF_CON,
		     cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_CON));
	CCCI_MEM_LOG_TAG(md->index, TAG, "AP_BUSY(%p)=%x\n", hw_info->md_pcore_pccif_base + APCCIF_BUSY,
		     cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_BUSY));
	CCCI_MEM_LOG_TAG(md->index, TAG, "AP_START(%p)=%x\n", hw_info->md_pcore_pccif_base + APCCIF_START,
		     cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_START));
	CCCI_MEM_LOG_TAG(md->index, TAG, "AP_TCHNUM(%p)=%x\n", hw_info->md_pcore_pccif_base + APCCIF_TCHNUM,
		     cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_TCHNUM));
	CCCI_MEM_LOG_TAG(md->index, TAG, "AP_RCHNUM(%p)=%x\n", hw_info->md_pcore_pccif_base + APCCIF_RCHNUM,
		     cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_RCHNUM));
	CCCI_MEM_LOG_TAG(md->index, TAG, "AP_ACK(%p)=%x\n", hw_info->md_pcore_pccif_base + APCCIF_ACK,
		     cldma_read32(hw_info->md_pcore_pccif_base, APCCIF_ACK));

	md_cd_lock_modem_clock_src(0);
}

void md_cd_check_md_DCM(struct md_cd_ctrl *md_ctrl)
{
}

void md_cd_check_emi_state(struct ccci_modem *md, int polling)
{
}

#define ROr2W(a, b, c)  cldma_write32(a, b, (cldma_read32(a, b)|c))
#define RAnd2W(a, b, c)  cldma_write32(a, b, (cldma_read32(a, b)&c))
#define RabIsc(a, b, c) ((cldma_read32(a, b)&c) != c)

static void md1_pcore_sram_turn_on(struct ccci_modem *md)
{
	int i;
	void __iomem *base = md_sram_pd_psmcusys_base;

	for (i = 31; i >= 0; i--)
		RAnd2W(base, 0x160, ~(0x1 << i));

	for (i = 31; i >= 0; i--)
		RAnd2W(base, 0x164, ~(0x1 << i));

	for (i = 25; i >= 0; i--)
		RAnd2W(base, 0x168, ~(0x1 << i));

	for (i = 31; i >= 16; i--)
		RAnd2W(base, 0x184, ~(0x1 << i));

	for (i = 31; i >= 0; i--)
		RAnd2W(base, 0x188, ~(0x1 << i));

	for (i = 25; i >= 0; i--)
		RAnd2W(base, 0x18C, ~(0x1 << i));

	for (i = 28; i >= 23; i--)
		RAnd2W(base, 0x190, ~(0x1 << i));

	CCCI_BOOTUP_LOG(md->index, TAG, "md1_pcore_sram reg: 0x%X, 0x%X, 0x%X, 0x%X 0x%X, 0x%X, 0x%X\n",
		ccci_read32(base, 0x160), ccci_read32(base, 0x164), ccci_read32(base, 0x168),
		ccci_read32(base, 0x184), ccci_read32(base, 0x188), ccci_read32(base, 0x18C),
		ccci_read32(base, 0x190));
}

static void md1_enable_mcu_clock(struct ccci_modem *md)
{
	void __iomem *base = md_sram_pd_psmcusys_base;

	ROr2W(base, 0x50, (0x1 << 20));
	ROr2W(base, 0x54, (0x1 << 20));
	RAnd2W(base, 0x840, 0xFFFFFFFE);
	ccci_write8(base, 0x850, 0x87);
	RAnd2W(base, 0xA34, 0xFFFFFFFE);

	CCCI_BOOTUP_LOG(md->index, TAG, "md1_mcu reg: 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n",
		ccci_read32(base, 0x50), ccci_read32(base, 0x54), ccci_read32(base, 0x840),
		ccci_read32(base, 0x850), ccci_read32(base, 0xA34));
}

static void md1_pcore_sram_on(struct ccci_modem *md)
{
	CCCI_NORMAL_LOG(md->index, TAG, "md1_pcore_sram_on enter\n");

	md1_pcore_sram_turn_on(md);
	md1_enable_mcu_clock(md);

	CCCI_BOOTUP_LOG(md->index, TAG, "md1_pcore_sram_on exit\n");
}

void md1_pmic_setting_off(void)
{
#ifdef CONFIG_MTK_PMIC_NEW_ARCH
#ifdef CONFIG_MTK_PMIC_CHIP_MT6355
	vmd1_pmic_setting_off();
#endif
#endif
}

void md1_pmic_setting_on(void)
{
#ifdef CONFIG_MTK_PMIC_NEW_ARCH
#ifdef CONFIG_MTK_PMIC_CHIP_MT6355
	vmd1_pmic_setting_on();
#endif
#endif
}

/* callback for system power off*/
void ccci_power_off(void)
{
	md1_pmic_setting_on();
}

void md1_pll_init(struct ccci_modem *md)
{
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;
	struct md_pll_reg *md_pll = md_info->md_pll_base;
	void __iomem *map_addr = (void __iomem *)(md->hw_info->ap_mixed_base);

	/* Enables clock square1 low-pass filter for 26M quality */
	ROr2W(map_addr, 0x04, 0x10);
	udelay(100);

	cldma_write32(md_pll->md_top_Pll, 0x4, 0x02020E94);

	cldma_write32(md_pll->md_top_Pll, 0x30, 0x0019000D);  /* 0x20140000 */
	cldma_write32(md_pll->md_top_Pll, 0x88, 0x80229D8A);
	cldma_write32(md_pll->md_top_Pll, 0x8C, 0x00000812);
	cldma_write32(md_pll->md_top_Pll, 0x98, 0x801EEC4F);
	cldma_write32(md_pll->md_top_Pll, 0x9C, 0x00000812);
	cldma_write32(md_pll->md_top_Pll, 0x48, 0x801EC4EC);

	cldma_write32(md_pll->md_top_Pll, 0x58, 0x801BB13B);
	cldma_write32(md_pll->md_top_Pll, 0x90, 0x801FD89D);

	cldma_write32(md_pll->md_top_Pll, 0x40, 0x80229D8A);
	CCCI_BOOTUP_LOG(md->index, TAG, "pll init clock done\n");

	while (((cldma_read32(md_pll->md_top_Pll, 0xC00) >> 16) & 0x7FFF) !=
		(cldma_read32(md_pll->md_top_Pll, 0x40) >> 7))
		;

	cldma_write32(md_pll->md_top_Pll, 0x10, 0x0);
	CCCI_BOOTUP_LOG(md->index, TAG, "pll init: before 0x80\n");
	while ((cldma_read32(md_pll->md_top_clkSW, 0x80) & 0x8000) != 0x8000)
		;

	ROr2W(md_pll->md_top_clkSW, 0x24, 0x3);
	CCCI_BOOTUP_LOG(md->index, TAG, "pll init: before 0x240\n");
	while ((cldma_read32(md_pll->md_sys_clk, 0x240) & 0x40000) != 0x40000)
		;
	cldma_write32(md_pll->md_sys_clk, 0x240, ((cldma_read32(md_pll->md_sys_clk, 0x240) & 0xFFFFFF00) | 0x00000002));
	ROr2W(md_pll->md_sys_clk, 0x240, 0x100);
	CCCI_BOOTUP_LOG(md->index, TAG, "pll init: before 0x240\n");
	while ((cldma_read32(md_pll->md_sys_clk, 0x240) & 0x40000) != 0x40000)
		;

	ROr2W(md_pll->md_top_clkSW, 0x24, 0x150177FC);
	cldma_write32(md_pll->md_top_clkSW, 0x20, 0x0);

	cldma_write32(md_pll->md_top_Pll, 0x314, 0xFFFF);
	cldma_write32(md_pll->md_top_Pll, 0x318, 0xFFFF);

	/*make a record that means MD pll has been initialized.*/
	cldma_write32(md_pll->md_top_clkSW, 0xF00, 0x62920000);
	CCCI_BOOTUP_LOG(md->index, TAG, "pll init: end\n");
}


void __attribute__((weak)) kicker_pbm_by_md(enum pbm_kicker kicker, bool status)
{
}

int md_cd_soft_power_off(struct ccci_modem *md, unsigned int mode)
{
	return 0;
}

int md_cd_soft_power_on(struct ccci_modem *md, unsigned int mode)
{
	return 0;
}

int md_cd_power_on(struct ccci_modem *md)
{
	int ret = 0;
	unsigned int reg_value;
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;

	/* step 0: PMIC setting */
	md1_pmic_setting_on();

	/* steip 1: power on MD_INFRA and MODEM_TOP */
	switch (md->index) {
	case MD_SYS1:
		CCCI_BOOTUP_LOG(md->index, TAG, "enable md sys clk\n");
		ret = clk_prepare_enable(clk_table[0].clk_ref);
		CCCI_BOOTUP_LOG(md->index, TAG, "enable md sys clk done,ret = %d\n", ret);
		kicker_pbm_by_md(KR_MD1, true);
		CCCI_BOOTUP_LOG(md->index, TAG, "Call end kicker_pbm_by_md(0,true)\n");
		break;
	}
	if (ret)
		return ret;

	md1_pcore_sram_on(md);

	/* step 3: MD srcclkena setting */
	reg_value = ccci_read32(infra_ao_base, INFRA_AO_MD_SRCCLKENA);
#if defined(CONFIG_MTK_MD3_SUPPORT) &&  (CONFIG_MTK_MD3_SUPPORT > 0)
	reg_value &= ~(0xB2);	/* md1 set 0x09: bit 0/3 */
	reg_value |= 0x09;	/* C2K set |0x44: bit 2/6 */
#else
	reg_value &= ~(0xFF);
	reg_value |= 0x09;
#endif
	ccci_write32(infra_ao_base, INFRA_AO_MD_SRCCLKENA, reg_value);
	CCCI_BOOTUP_LOG(md->index, CORE, "md_cd_power_on: set md1_srcclkena bit(0x1000_0F0C)=0x%x\n",
		     ccci_read32(infra_ao_base, INFRA_AO_MD_SRCCLKENA));

#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
	/* notify NFC */
	inform_nfc_vsim_change(md->index, 1, 0);
#endif
	/* step 4: pll init */
	md1_pll_init(md);

	/* step 5: disable MD WDT */
	cldma_write32(md_info->md_rgu_base, WDT_MD_MODE, WDT_MD_MODE_KEY);

	/* step 6: switch md usb interrupt MUX to partial interrupt */
	ROr2W(md_info->md_pll_base->md_usb_int_mux, 0, 1);

	return 0;
}

int md_cd_bootup_cleanup(struct ccci_modem *md, int success)
{
	return 0;
}

int md_cd_let_md_go(struct ccci_modem *md)
{
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;

	if (MD_IN_DEBUG(md))
		return -1;
	CCCI_BOOTUP_LOG(md->index, TAG, "set MD boot slave\n");

	cldma_write32(md_info->md_boot_slave_En, 0, 1);	/* make boot vector take effect */
	return 0;
}

int md_cd_power_off(struct ccci_modem *md, unsigned int timeout)
{
	int ret = 0;

#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
	/* notify NFC */
	inform_nfc_vsim_change(md->index, 0, 0);
#endif

	/* power off MD_INFRA and MODEM_TOP */
	switch (md->index) {
	case MD_SYS1:
		clk_disable_unprepare(clk_table[0].clk_ref);
		CCCI_BOOTUP_LOG(md->index, TAG, "disble md1 clk\n");
		CCCI_BOOTUP_LOG(md->index, TAG, "Call md1_pmic_setting_off\n");
		md1_pmic_setting_off();
		kicker_pbm_by_md(KR_MD1, false);
		CCCI_BOOTUP_LOG(md->index, TAG, "Call end kicker_pbm_by_md(0,false)\n");
		break;
	}
	return ret;
}

void cldma_dump_register(struct md_cd_ctrl *md_ctrl)
{

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump AP CLDMA Tx pdn register, active=%x\n", md_ctrl->txq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_ap_pdn_base + CLDMA_AP_UL_START_ADDR_0,
		      CLDMA_AP_UL_LAST_UPDATE_ADDR_4MSB - CLDMA_AP_UL_START_ADDR_0 + 4);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump AP CLDMA Tx ao register, active=%x\n", md_ctrl->txq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_ap_ao_base + CLDMA_AP_UL_START_ADDR_BK_0,
		      CLDMA_AP_UL_CURRENT_ADDR_BK_4MSB - CLDMA_AP_UL_START_ADDR_BK_0 + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump AP CLDMA Rx pdn register, active=%x\n", md_ctrl->rxq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_ap_pdn_base + CLDMA_AP_SO_ERROR,
		      CLDMA_AP_SO_STOP_CMD - CLDMA_AP_SO_ERROR + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump AP CLDMA Rx ao register, active=%x\n", md_ctrl->rxq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_ap_ao_base + CLDMA_AP_SO_CFG,
		      CLDMA_AP_SO_NEXT_BUF_PTR_4MSB - CLDMA_AP_SO_CFG + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump AP CLDMA MISC pdn register\n");
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_ap_pdn_base + CLDMA_AP_L2TISAR0,
		      CLDMA_AP_L3TIMSR2 - CLDMA_AP_L2TISAR0 + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump AP CLDMA MISC ao register\n");
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_ap_ao_base + CLDMA_AP_L2RIMR0,
						CLDMA_AP_DUMMY - CLDMA_AP_L2RIMR0 + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump MD CLDMA Tx pdn register, active=%x\n", md_ctrl->txq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_md_pdn_base + CLDMA_AP_UL_START_ADDR_0,
		      CLDMA_AP_UL_LAST_UPDATE_ADDR_4MSB - CLDMA_AP_UL_START_ADDR_0 + 4);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump MD CLDMA Tx ao register, active=%x\n", md_ctrl->txq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_md_ao_base + CLDMA_AP_UL_START_ADDR_BK_0,
		      CLDMA_AP_UL_CURRENT_ADDR_BK_4MSB - CLDMA_AP_UL_START_ADDR_BK_0 + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump MD CLDMA Rx pdn register, active=%x\n", md_ctrl->rxq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_md_pdn_base + CLDMA_AP_SO_ERROR,
		      CLDMA_AP_SO_STOP_CMD - CLDMA_AP_SO_ERROR + 4);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump MD CLDMA Rx ao register, active=%x\n", md_ctrl->rxq_active);
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_md_ao_base + CLDMA_AP_SO_CFG,
		      CLDMA_AP_SO_NEXT_BUF_PTR_4MSB - CLDMA_AP_SO_CFG + 4);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump MD CLDMA MISC pdn register\n");
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_md_pdn_base + CLDMA_AP_L2TISAR0,
		      CLDMA_AP_L3TIMSR2 - CLDMA_AP_L2TISAR0 + 4);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "dump MD CLDMA MISC ao register\n");
	ccci_util_mem_dump(md_ctrl->md_id, CCCI_DUMP_MEM_DUMP, md_ctrl->cldma_md_ao_base + CLDMA_AP_L2RIMR0,
						CLDMA_AP_DUMMY - CLDMA_AP_L2RIMR0 + 4);

}

int ccci_modem_remove(struct platform_device *dev)
{
	return 0;
}

void ccci_modem_shutdown(struct platform_device *dev)
{
}

int ccci_modem_suspend(struct platform_device *dev, pm_message_t state)
{
	struct ccci_modem *md = (struct ccci_modem *)dev->dev.platform_data;

	CCCI_NORMAL_LOG(md->index, TAG, "ccci_modem_suspend\n");
	return 0;
}

int ccci_modem_resume(struct platform_device *dev)
{
	struct ccci_modem *md = (struct ccci_modem *)dev->dev.platform_data;

	CCCI_NORMAL_LOG(md->index, TAG, "ccci_modem_resume\n");
	return 0;
}

int ccci_modem_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	if (pdev == NULL) {
		CCCI_ERROR_LOG(MD_SYS1, TAG, "%s pdev == NULL\n", __func__);
		return -1;
	}
	return ccci_modem_suspend(pdev, PMSG_SUSPEND);
}

int ccci_modem_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	if (pdev == NULL) {
		CCCI_ERROR_LOG(MD_SYS1, TAG, "%s pdev == NULL\n", __func__);
		return -1;
	}
	return ccci_modem_resume(pdev);
}

int ccci_modem_pm_restore_noirq(struct device *device)
{
	struct ccci_modem *md = (struct ccci_modem *)device->platform_data;

	/* set flag for next md_start */
	md->per_md_data.config.setting |= MD_SETTING_RELOAD;
	md->per_md_data.config.setting |= MD_SETTING_FIRST_BOOT;
	/* restore IRQ */
#ifdef FEATURE_PM_IPO_H
	irq_set_irq_type(md_ctrl->cldma_irq_id, IRQF_TRIGGER_HIGH);
	irq_set_irq_type(md_ctrl->md_wdt_irq_id, IRQF_TRIGGER_RISING);
#endif
	return 0;
}

void ccci_modem_restore_reg(struct ccci_modem *md)
{
	struct md_cd_ctrl *md_ctrl = (struct md_cd_ctrl *)ccci_hif_get_by_id(CLDMA_HIF_ID);
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;
	enum MD_STATE md_state = ccci_fsm_get_md_state(md->index);
	int i;
	unsigned long flags;
	unsigned int val = 0;
	dma_addr_t bk_addr = 0;

	if (md_state == GATED || md_state == WAITING_TO_STOP || md_state == INVALID) {
		CCCI_NORMAL_LOG(md->index, TAG, "Resume no need reset cldma for md_state=%d\n", md_state);
		return;
	}
	cldma_write32(md_info->ap_ccif_base, APCCIF_CON, 0x01);	/* arbitration */

	cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_RESUME_CMD, CLDMA_BM_ALL_QUEUE & (1 << 0));
	cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_RESUME_CMD); /* dummy read */

	if (cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_TQSAR(0))
		|| cldma_reg_get_4msb_val(md_ctrl->cldma_ap_ao_base,
					CLDMA_AP_UL_START_ADDR_4MSB, md_ctrl->txq[0].index)) {
		CCCI_NORMAL_LOG(md->index, TAG, "Resume cldma pdn register: No need  ...\n");
	} else {
		CCCI_NORMAL_LOG(md->index, TAG, "Resume cldma pdn register ...11\n");
		spin_lock_irqsave(&md_ctrl->cldma_timeout_lock, flags);
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_HPQR, 0x00);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		/* re-config 8G mode flag for pd register*/
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CFG,
				      cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_UL_CFG) | 0x40);
#endif
		/* set start address */
		for (i = 0; i < QUEUE_LEN(md_ctrl->txq); i++) {
			if (cldma_read32(md_ctrl->cldma_ap_ao_base, CLDMA_AP_TQCPBAK(md_ctrl->txq[i].index)) == 0
				 && cldma_reg_get_4msb_val(md_ctrl->cldma_ap_ao_base,
					CLDMA_AP_UL_CURRENT_ADDR_BK_4MSB, md_ctrl->txq[i].index) == 0) {
				if (i != 7) /* Queue 7 not used currently */
					CCCI_NORMAL_LOG(md->index, TAG, "Resume CH(%d) current bak:== 0\n", i);
				cldma_reg_set_tx_start_addr(md_ctrl->cldma_ap_pdn_base,
					md_ctrl->txq[i].index,
					md_ctrl->txq[i].tr_done->gpd_addr);
				cldma_reg_set_tx_start_addr_bk(md_ctrl->cldma_ap_ao_base,
					md_ctrl->txq[i].index,
					md_ctrl->txq[i].tr_done->gpd_addr);
			} else {
				if (sizeof(dma_addr_t) > sizeof(u32)) {
					val = cldma_reg_get_4msb_val(md_ctrl->cldma_ap_ao_base,
							CLDMA_AP_UL_CURRENT_ADDR_BK_4MSB,
							md_ctrl->txq[i].index);
					/*set high bits*/
					bk_addr = val;
					bk_addr <<= 32;
				} else
					bk_addr = 0;
				/*set low bits*/
				val = cldma_read32(md_ctrl->cldma_ap_ao_base,
									CLDMA_AP_TQCPBAK(md_ctrl->txq[i].index));
				bk_addr |= val;
				cldma_reg_set_tx_start_addr(md_ctrl->cldma_ap_pdn_base,
					md_ctrl->txq[i].index, bk_addr);
				cldma_reg_set_tx_start_addr_bk(md_ctrl->cldma_ap_ao_base,
					md_ctrl->txq[i].index, bk_addr);
			}
		}
		/* wait write done*/
		wmb();
		/* start all Tx and Rx queues */
		md_ctrl->txq_started = 0;
		md_ctrl->txq_active |= CLDMA_BM_ALL_QUEUE;
		/* cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_START_CMD, CLDMA_BM_ALL_QUEUE); */
		/* cldma_read32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_SO_START_CMD); // dummy read */
		/* md_ctrl->rxq_active |= CLDMA_BM_ALL_QUEUE; */
		/* enable L2 DONE and ERROR interrupts */
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L2TIMCR0,
				CLDMA_TX_INT_DONE | CLDMA_TX_INT_QUEUE_EMPTY | CLDMA_TX_INT_ERROR);
		/* enable all L3 interrupts */
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMCR0, CLDMA_BM_INT_ALL);
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3TIMCR1, CLDMA_BM_INT_ALL);
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMCR0, CLDMA_BM_INT_ALL);
		cldma_write32(md_ctrl->cldma_ap_pdn_base, CLDMA_AP_L3RIMCR1, CLDMA_BM_INT_ALL);
		spin_unlock_irqrestore(&md_ctrl->cldma_timeout_lock, flags);
		CCCI_NORMAL_LOG(md->index, TAG, "Resume cldma pdn register done\n");
	}
}

int ccci_modem_syssuspend(void)
{
	CCCI_NORMAL_LOG(0, TAG, "ccci_modem_syssuspend\n");
	return 0;
}

void ccci_modem_sysresume(void)
{
	struct ccci_modem *md;

	CCCI_NORMAL_LOG(0, TAG, "ccci_modem_sysresume\n");
	md = ccci_md_get_modem_by_id(0);
	if (md != NULL)
		ccci_modem_restore_reg(md);
}
