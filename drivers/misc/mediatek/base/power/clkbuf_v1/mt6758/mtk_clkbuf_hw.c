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

/*
 * @file    mtk_clk_buf_hw.c
 * @brief   Driver for clock buffer control of each platform
 *
 */

#include <mtk_spm.h>
#include <mtk_clkbuf_ctl.h>
#include <mtk_clkbuf_common.h>
#include "mtk_spm_sleep.h"
#ifdef CONFIG_MTK_UFS_BOOTING
#include "ufs-mtk.h"
#endif

#ifdef CONFIG_PM
#define DEFINE_ATTR_RO(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
}

#define DEFINE_ATTR_RW(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define __ATTR_OF(_name)	(&_name##_attr.attr)
#endif /* CONFIG_PM */

static void __iomem *pwrap_base;

#define PWRAP_REG(ofs)		(pwrap_base + ofs)

/* PMICWRAP Reg */
#define DCXO_ENABLE		PWRAP_REG(0x17C)
#define DCXO_CONN_ADR0		PWRAP_REG(0x180)
#define DCXO_CONN_WDATA0	PWRAP_REG(0x184)
#define DCXO_CONN_ADR1		PWRAP_REG(0x188)
#define DCXO_CONN_WDATA1	PWRAP_REG(0x18C)
#define DCXO_NFC_ADR0		PWRAP_REG(0x190)
#define DCXO_NFC_WDATA0		PWRAP_REG(0x194)
#define DCXO_NFC_ADR1		PWRAP_REG(0x198)
#define DCXO_NFC_WDATA1		PWRAP_REG(0x19C)

#define PMIC_DCXO_CW00		MT6355_DCXO_CW00
#define PMIC_DCXO_CW00_SET	MT6355_DCXO_CW00_SET
#define PMIC_DCXO_CW00_CLR	MT6355_DCXO_CW00_CLR
#define PMIC_DCXO_CW01		MT6355_DCXO_CW01
#define PMIC_DCXO_CW02		MT6355_DCXO_CW02
#define PMIC_DCXO_CW03		MT6355_DCXO_CW03
#define PMIC_DCXO_CW04		MT6355_DCXO_CW04
#define PMIC_DCXO_CW05		MT6355_DCXO_CW05
#define PMIC_DCXO_CW06		MT6355_DCXO_CW06
#define PMIC_DCXO_CW07		MT6355_DCXO_CW07
#define PMIC_DCXO_CW08		MT6355_DCXO_CW08
#define PMIC_DCXO_CW09		MT6355_DCXO_CW09
#define PMIC_DCXO_CW10		MT6355_DCXO_CW10
#define PMIC_DCXO_CW11		MT6355_DCXO_CW11
#define PMIC_DCXO_CW11_SET	MT6355_DCXO_CW11_SET
#define PMIC_DCXO_CW11_CLR	MT6355_DCXO_CW11_CLR
#define PMIC_DCXO_CW12		MT6355_DCXO_CW12
#define PMIC_DCXO_CW13		MT6355_DCXO_CW13
#define PMIC_DCXO_CW14		MT6355_DCXO_CW14
#define PMIC_DCXO_CW15		MT6355_DCXO_CW15
#define PMIC_DCXO_CW16		MT6355_DCXO_CW16
#define PMIC_DCXO_CW17		MT6355_DCXO_CW17
#define PMIC_DCXO_CW18		MT6355_DCXO_CW18
#define PMIC_DCXO_CW19		MT6355_DCXO_CW19

#define	DCXO_CONN_ENABLE	(0x1 << 1)
#define	DCXO_NFC_ENABLE		(0x1 << 0)

#define PMIC_REG_MASK				0xFFFF
#define PMIC_REG_SHIFT				0

#define PMIC_CW00_INIT_VAL			0x4EDD
#define PMIC_CW11_INIT_VAL			0xA000

/* TODO: marked this after driver is ready */
/* #define CLKBUF_BRINGUP */

/* #define CLKBUF_AUTOK_ENABLE */

/* #define CLKBUF_CONN_SUPPORT_CTRL_FROM_I1 */

/* TODO: enable BBLPM if its function is ready (set as 1) */
static unsigned int bblpm_switch = 1;

static unsigned int pwrap_dcxo_en_flag = (DCXO_CONN_ENABLE | DCXO_NFC_ENABLE);

static unsigned int CLK_BUF1_STATUS_PMIC = CLOCK_BUFFER_HW_CONTROL,
		    CLK_BUF2_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL,
		    CLK_BUF3_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL,
		    CLK_BUF4_STATUS_PMIC = CLOCK_BUFFER_HW_CONTROL,
		    CLK_BUF5_STATUS_PMIC = CLOCK_BUFFER_DISABLE,
		    CLK_BUF6_STATUS_PMIC = CLOCK_BUFFER_DISABLE,
		    CLK_BUF7_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
static int PMIC_CLK_BUF1_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K,
	   PMIC_CLK_BUF2_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K,
	   PMIC_CLK_BUF3_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K,
	   PMIC_CLK_BUF4_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K,
	   PMIC_CLK_BUF5_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K,
	   PMIC_CLK_BUF6_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K,
	   PMIC_CLK_BUF7_DRIVING_CURR = CLK_BUF_DRIVING_CURR_AUTO_K;

#ifdef CLKBUF_AUTOK_ENABLE
static u8 clkbuf_drv_curr_auxout[CLKBUF_NUM];
static u8 xo_en_stat[CLKBUF_NUM];
#endif

#ifndef CLKBUF_BRINGUP
static enum CLK_BUF_SWCTRL_STATUS_T  pmic_clk_buf_swctrl[CLKBUF_NUM] = {
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_ENABLE
};
#else /* For Bring-up */
static enum CLK_BUF_SWCTRL_STATUS_T  pmic_clk_buf_swctrl[CLKBUF_NUM] = {
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE
};
#endif

static void pmic_clk_buf_ctrl_wcn(short on)
{
#if defined(CLKBUF_BRINGUP)
	if (on) {
		pmic_config_interface(PMIC_DCXO_CW00_SET_ADDR, 1,
				    PMIC_XO_EXTBUF2_EN_M_MASK,
				    PMIC_XO_EXTBUF2_EN_M_SHIFT);
		pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x3,
				    PMIC_XO_EXTBUF2_MODE_MASK,
				    PMIC_XO_EXTBUF2_MODE_SHIFT);

	} else {
		pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 1,
				    PMIC_XO_EXTBUF2_EN_M_MASK,
				    PMIC_XO_EXTBUF2_EN_M_SHIFT);
		pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x3,
				    PMIC_XO_EXTBUF2_MODE_MASK,
				    PMIC_XO_EXTBUF2_MODE_SHIFT);
	}
#elif defined(CLKBUF_CONN_SUPPORT_CTRL_FROM_I1)
	if (on)
		pmic_config_interface(PMIC_DCXO_CW00_SET_ADDR, 0x1,
				      PMIC_XO_EXTBUF2_EN_M_MASK,
				      PMIC_XO_EXTBUF2_EN_M_SHIFT);
	else
		pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x1,
				      PMIC_XO_EXTBUF2_EN_M_MASK,
				      PMIC_XO_EXTBUF2_EN_M_SHIFT);
#else
	if (on)
		pmic_config_interface(PMIC_RG_SRCLKEN_IN3_EN_ADDR, 1,
				    PMIC_RG_SRCLKEN_IN3_EN_MASK,
				    PMIC_RG_SRCLKEN_IN3_EN_SHIFT);
	else
		pmic_config_interface(PMIC_RG_SRCLKEN_IN3_EN_ADDR, 0,
				    PMIC_RG_SRCLKEN_IN3_EN_MASK,
				    PMIC_RG_SRCLKEN_IN3_EN_SHIFT);
#endif
}

static void pmic_clk_buf_ctrl_nfc(short on)
{
#if defined(CLKBUF_BRINGUP)
		if (on) {
			pmic_config_interface(PMIC_DCXO_CW00_SET_ADDR, 1,
					    PMIC_XO_EXTBUF3_EN_M_MASK,
					    PMIC_XO_EXTBUF3_EN_M_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x3,
					    PMIC_XO_EXTBUF3_MODE_MASK,
					    PMIC_XO_EXTBUF3_MODE_SHIFT);
		} else {
			pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 1,
					    PMIC_XO_EXTBUF3_EN_M_MASK,
					    PMIC_XO_EXTBUF3_EN_M_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x3,
					    PMIC_XO_EXTBUF3_MODE_MASK,
					    PMIC_XO_EXTBUF3_MODE_SHIFT);
		}
#else
	if (on)
		pmic_config_interface(PMIC_DCXO_CW00_SET_ADDR, 0x1,
				      PMIC_XO_EXTBUF3_EN_M_MASK,
				      PMIC_XO_EXTBUF3_EN_M_SHIFT);
	else
		pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x1,
				      PMIC_XO_EXTBUF3_EN_M_MASK,
				      PMIC_XO_EXTBUF3_EN_M_SHIFT);
#endif
}

static void pmic_clk_buf_ctrl_cel(short on)
{
#if defined(CLKBUF_BRINGUP)
		if (on) {
			pmic_config_interface(PMIC_DCXO_CW00_SET_ADDR, 1,
					    PMIC_XO_EXTBUF4_EN_M_MASK,
					    PMIC_XO_EXTBUF4_EN_M_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x3,
					    PMIC_XO_EXTBUF4_MODE_MASK,
					    PMIC_XO_EXTBUF4_MODE_SHIFT);
		} else {
			pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 1,
					    PMIC_XO_EXTBUF4_EN_M_MASK,
					    PMIC_XO_EXTBUF4_EN_M_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW00_CLR_ADDR, 0x3,
					    PMIC_XO_EXTBUF4_MODE_MASK,
					    PMIC_XO_EXTBUF4_MODE_SHIFT);
		}
#endif
}


static void pmic_clk_buf_ctrl_aud(short on)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CLKBUF_BRINGUP)
	if (on) {
		pmic_config_interface(PMIC_DCXO_CW11_SET_ADDR, 1,
				    PMIC_XO_EXTBUF6_EN_M_MASK,
				    PMIC_XO_EXTBUF6_EN_M_SHIFT);
		pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 0x3,
				    PMIC_XO_EXTBUF6_MODE_MASK,
				    PMIC_XO_EXTBUF6_MODE_SHIFT);

	} else {
		pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 1,
				    PMIC_XO_EXTBUF6_EN_M_MASK,
				    PMIC_XO_EXTBUF6_EN_M_SHIFT);
		pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 0x3,
				    PMIC_XO_EXTBUF6_MODE_MASK,
				    PMIC_XO_EXTBUF6_MODE_SHIFT);
	}
#else

	if (on)
		pmic_config_interface(PMIC_DCXO_CW11_SET_ADDR, 0x1,
				      PMIC_XO_EXTBUF6_EN_M_MASK,
				      PMIC_XO_EXTBUF6_EN_M_SHIFT);
	else
		pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 0x1,
				      PMIC_XO_EXTBUF6_EN_M_MASK,
				      PMIC_XO_EXTBUF6_EN_M_SHIFT);
#endif
#endif
}

static void pmic_clk_buf_ctrl_ext(short on)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CLKBUF_BRINGUP)
		if (on) {
			pmic_config_interface(PMIC_DCXO_CW11_SET_ADDR, 1,
					    PMIC_XO_EXTBUF7_EN_M_MASK,
					    PMIC_XO_EXTBUF7_EN_M_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 0x3,
					    PMIC_XO_EXTBUF7_MODE_MASK,
					    PMIC_XO_EXTBUF7_MODE_SHIFT);
		} else {
			pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 1,
					    PMIC_XO_EXTBUF7_EN_M_MASK,
					    PMIC_XO_EXTBUF7_EN_M_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 0x3,
					    PMIC_XO_EXTBUF7_MODE_MASK,
					    PMIC_XO_EXTBUF7_MODE_SHIFT);
		}
#else
	if (on)
		pmic_config_interface(PMIC_DCXO_CW11_SET_ADDR, 0x1,
				      PMIC_XO_EXTBUF7_EN_M_MASK,
				      PMIC_XO_EXTBUF7_EN_M_SHIFT);
	else
		pmic_config_interface(PMIC_DCXO_CW11_CLR_ADDR, 0x1,
				      PMIC_XO_EXTBUF7_EN_M_MASK,
				      PMIC_XO_EXTBUF7_EN_M_SHIFT);
#endif
#endif
}

static void pmic_clk_buf_ctrl(enum CLK_BUF_SWCTRL_STATUS_T *status)
{
	u32 pmic_cw00 = 0, pmic_cw11 = 0;

#ifndef CLKBUF_BRINGUP
	if (!is_clkbuf_initiated)
		return;
#endif
	pmic_clk_buf_ctrl_wcn(status[XO_WCN] % 2);
	pmic_clk_buf_ctrl_nfc(status[XO_NFC] % 2);
	pmic_clk_buf_ctrl_cel(status[XO_CEL] % 2);
	pmic_clk_buf_ctrl_aud(status[XO_AUD] % 2);
	pmic_clk_buf_ctrl_ext(status[XO_EXT] % 2);

	pmic_read_interface(PMIC_DCXO_CW00, &pmic_cw00,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_DCXO_CW11, &pmic_cw11,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	clk_buf_pr_info("%s DCXO_CW00=0x%x, CW11=0x%x, clk_buf_swctrl=[%u %u %u %u %u %u %u]\n",
		     __func__, pmic_cw00, pmic_cw11, status[0], status[1],
		     status[2], status[3], status[4], status[5], status[6]);
}

void clk_buf_control_bblpm(bool on)
{
#ifdef CLKBUF_USE_BBLPM
#ifndef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	u32 cw00 = 0;

	if (!is_clkbuf_initiated || !is_pmic_clkbuf)
		return;

	if (on) /* FPM -> BBLPM */
		pmic_config_interface_nolock(PMIC_DCXO_CW00_SET_ADDR, 0x1,
				      PMIC_XO_BB_LPM_EN_MASK,
				      PMIC_XO_BB_LPM_EN_SHIFT);
	else /* BBLPM -> FPM */
		pmic_config_interface_nolock(PMIC_DCXO_CW00_CLR_ADDR, 0x1,
				      PMIC_XO_BB_LPM_EN_MASK,
				      PMIC_XO_BB_LPM_EN_SHIFT);

	pmic_read_interface_nolock(PMIC_DCXO_CW00, &cw00,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);

	clk_buf_pr_dbg("%s(%u): CW00=0x%x\n", __func__, (on ? 1 : 0), cw00);
#endif
#endif
}

/*
 * Baseband Low Power Mode (BBLPM) for PMIC clkbuf
 * Condition: XO_CELL/XO_NFC/XO_WCN/XO_EXT OFF
 * Caller: deep idle, SODI2.5
 * Return: 0 if all conditions are matched & ready to enter BBLPM
 */
u32 clk_buf_bblpm_enter_cond(void)
{
	u32 bblpm_cond = 0;
#ifdef CLKBUF_USE_BBLPM
	u32 pwr_sta = 0;
#endif

#ifdef CLKBUF_USE_BBLPM
	if (!is_clkbuf_initiated || !is_pmic_clkbuf || !bblpm_switch) {
		bblpm_cond |= BBLPM_COND_SKIP;
		return bblpm_cond;
	}

	pwr_sta = clkbuf_readl(PWR_STATUS);

	if (pwr_sta & (PWR_STATUS_MD | PWR_STATUS_C2K))
		bblpm_cond |= BBLPM_COND_CEL;

	if ((pmic_clk_buf_swctrl[XO_WCN] == CLK_BUF_SW_ENABLE) ||
	    (pwr_sta & PWR_STATUS_CONN))
		bblpm_cond |= BBLPM_COND_WCN;

	if (pmic_clk_buf_swctrl[XO_NFC] == CLK_BUF_SW_ENABLE)
		bblpm_cond |= BBLPM_COND_NFC;

#ifdef CONFIG_MTK_UFS_BOOTING
	if (ufs_mtk_deepidle_hibern8_check() < 0)
		bblpm_cond |= BBLPM_COND_EXT;
#endif
#else /* !CLKBUF_USE_BBLPM */
	bblpm_cond |= BBLPM_COND_SKIP;
#endif

	if (!bblpm_cond)
		bblpm_cnt++;

	return bblpm_cond;
}

static void clk_buf_ctrl_internal(enum clk_buf_id id, bool onoff)
{
	if (!is_pmic_clkbuf)
		return;

	mutex_lock(&clk_buf_ctrl_lock);

	switch (id) {
	case CLK_BUF_CONN:
		if (onoff) {
			CLK_BUF2_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
#ifdef CLKBUF_CONN_SUPPORT_CTRL_FROM_I1
			pmic_config_interface(PMIC_DCXO_CW00_SET, 0x3,
					      PMIC_XO_EXTBUF2_MODE_MASK,
					      PMIC_XO_EXTBUF2_MODE_SHIFT);
#else
			pmic_config_interface(PMIC_DCXO_CW00_CLR, 0x3,
					      PMIC_XO_EXTBUF2_MODE_MASK,
					      PMIC_XO_EXTBUF2_MODE_SHIFT);
			pmic_config_interface(PMIC_DCXO_CW00_SET, 0x2,
					      PMIC_XO_EXTBUF2_MODE_MASK,
					      PMIC_XO_EXTBUF2_MODE_SHIFT);
#endif
			pmic_clk_buf_ctrl_wcn(1);
			pmic_clk_buf_swctrl[XO_WCN] = 1;

			pwrap_dcxo_en_flag |= DCXO_CONN_ENABLE;
			clkbuf_writel(DCXO_ENABLE, pwrap_dcxo_en_flag);
		} else {
			pwrap_dcxo_en_flag &= ~DCXO_CONN_ENABLE;
			clkbuf_writel(DCXO_ENABLE, pwrap_dcxo_en_flag);

			pmic_config_interface(PMIC_DCXO_CW00_CLR, 0x3,
					      PMIC_XO_EXTBUF2_MODE_MASK,
					      PMIC_XO_EXTBUF2_MODE_SHIFT);
			pmic_clk_buf_ctrl_wcn(0);
			pmic_clk_buf_swctrl[XO_WCN] = 0;
			CLK_BUF2_STATUS_PMIC = CLOCK_BUFFER_DISABLE;
		}
		clk_buf_pr_info("%s: id=%d, onoff=%d, DCXO_ENABLE=0x%x, pwrap_dcxo_en_flag=0x%x\n",
			     __func__, id, onoff, clkbuf_readl(DCXO_ENABLE),
			     pwrap_dcxo_en_flag);
		break;
	case CLK_BUF_NFC:
		if (onoff) {
			CLK_BUF3_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
			pmic_config_interface(PMIC_DCXO_CW00_SET, 0x3,
					      PMIC_XO_EXTBUF3_MODE_MASK,
					      PMIC_XO_EXTBUF3_MODE_SHIFT);
			pmic_clk_buf_ctrl_nfc(1);
			pmic_clk_buf_swctrl[XO_NFC] = 1;

			pwrap_dcxo_en_flag |= DCXO_NFC_ENABLE;
			clkbuf_writel(DCXO_ENABLE, pwrap_dcxo_en_flag);
		} else {
			pwrap_dcxo_en_flag &= ~DCXO_NFC_ENABLE;
			clkbuf_writel(DCXO_ENABLE, pwrap_dcxo_en_flag);

			pmic_config_interface(PMIC_DCXO_CW00_CLR, 0x3,
					      PMIC_XO_EXTBUF3_MODE_MASK,
					      PMIC_XO_EXTBUF3_MODE_SHIFT);
			pmic_clk_buf_ctrl_nfc(0);
			pmic_clk_buf_swctrl[XO_NFC] = 0;
			CLK_BUF3_STATUS_PMIC = CLOCK_BUFFER_DISABLE;
		}
		clk_buf_pr_info("%s: id=%d, onoff=%d, DCXO_ENABLE=0x%x, pwrap_dcxo_en_flag=0x%x\n",
			     __func__, id, onoff, clkbuf_readl(DCXO_ENABLE),
			     pwrap_dcxo_en_flag);
		break;
	case CLK_BUF_AUDIO:
		if (onoff) {
			CLK_BUF6_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
			pmic_clk_buf_ctrl_aud(1);
			pmic_clk_buf_swctrl[XO_AUD] = 1;
		} else {
			pmic_clk_buf_ctrl_aud(0);
			pmic_clk_buf_swctrl[XO_AUD] = 0;
			CLK_BUF6_STATUS_PMIC = CLOCK_BUFFER_DISABLE;
		}
		clk_buf_pr_info("%s: id=%d, onoff=%d\n", __func__, id, onoff);
		break;
	case CLK_BUF_UFS:
		if (onoff) {
			CLK_BUF7_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
			pmic_clk_buf_ctrl_ext(1);
			pmic_clk_buf_swctrl[XO_EXT] = 1;
		} else {
			pmic_clk_buf_ctrl_ext(0);
			pmic_clk_buf_swctrl[XO_EXT] = 0;
			CLK_BUF7_STATUS_PMIC = CLOCK_BUFFER_DISABLE;
		}
		clk_buf_pr_info("%s: id=%d, onoff=%d\n", __func__, id, onoff);
		break;
	default:
		clk_buf_pr_err("%s: id=%d isn't supported\n", __func__, id);
		break;
	}

	mutex_unlock(&clk_buf_ctrl_lock);
}

bool clk_buf_ctrl(enum clk_buf_id id, bool onoff)
{
	short ret = 0, no_lock = 0;

	if (!is_clkbuf_initiated)
		return false;

	if (!is_pmic_clkbuf)
		return false;

	clk_buf_pr_dbg("%s: id=%d, onoff=%d, clkbuf_ctrl_stat=0x%x\n", __func__,
		    id, onoff, clkbuf_ctrl_stat);

	if (preempt_count() > 0 || irqs_disabled() || system_state != SYSTEM_RUNNING || oops_in_progress)
		no_lock = 1;

	if (!no_lock)
		mutex_lock(&clk_buf_ctrl_lock);

	switch (id) {
	case CLK_BUF_BB_MD:
		if (CLK_BUF1_STATUS_PMIC != CLOCK_BUFFER_SW_CONTROL) {
			ret = -1;
			clk_buf_pr_info("%s: id=%d isn't controlled by SW\n", __func__, id);
			break;
		}
		break;
	case CLK_BUF_CONN:
		if (CLK_BUF2_STATUS_PMIC != CLOCK_BUFFER_SW_CONTROL) {
			ret = -1;
			clk_buf_pr_info("%s: id=%d isn't controlled by SW\n", __func__, id);
			break;
		}
		if (!(pwrap_dcxo_en_flag & DCXO_CONN_ENABLE)) {
			ret = -1;
			clk_buf_pr_err("%s: id=%d skip due to non co-clock for CONN\n", __func__, id);
			pmic_clk_buf_ctrl_wcn(0);
			pmic_clk_buf_swctrl[XO_WCN] = 0;
			break;
		}
		/* record the status of CONN from caller for checking BBLPM */
		pmic_clk_buf_swctrl[XO_WCN] = onoff;
		break;
	case CLK_BUF_NFC:
		if (CLK_BUF3_STATUS_PMIC != CLOCK_BUFFER_SW_CONTROL) {
			ret = -1;
			clk_buf_pr_info("%s: id=%d isn't controlled by SW\n", __func__, id);
			break;
		}
		/* record the status of NFC from caller for checking BBLPM */
		pmic_clk_buf_swctrl[XO_NFC] = onoff;
		break;
	case CLK_BUF_RF:
		if (CLK_BUF4_STATUS_PMIC != CLOCK_BUFFER_SW_CONTROL) {
			ret = -1;
			clk_buf_pr_info("%s: id=%d isn't controlled by SW\n", __func__, id);
			break;
		}
		break;
	case CLK_BUF_AUDIO:
		if (CLK_BUF6_STATUS_PMIC != CLOCK_BUFFER_SW_CONTROL) {
			ret = -1;
			clk_buf_pr_info("%s: id=%d isn't controlled by SW\n", __func__, id);
			break;
		}
		pmic_clk_buf_ctrl_aud(onoff);
		pmic_clk_buf_swctrl[XO_AUD] = onoff;
		break;
	case CLK_BUF_UFS:
		if (CLK_BUF7_STATUS_PMIC != CLOCK_BUFFER_SW_CONTROL) {
			ret = -1;
			clk_buf_pr_info("%s: id=%d isn't controlled by SW\n", __func__, id);
			break;
		}
		pmic_clk_buf_ctrl_ext(onoff);
		pmic_clk_buf_swctrl[XO_EXT] = onoff;
		break;
	default:
		ret = -1;
		clk_buf_pr_err("%s: id=%d isn't supported\n", __func__, id);
		break;
	}

	if (!no_lock)
		mutex_unlock(&clk_buf_ctrl_lock);

	if (ret)
		return false;
	else
		return true;
}
EXPORT_SYMBOL(clk_buf_ctrl);

void clk_buf_dump_dts_log(void)
{
	clk_buf_pr_info("%s: PMIC_CLK_BUF?_STATUS=%d %d %d %d %d %d %d\n", __func__,
		     CLK_BUF1_STATUS_PMIC, CLK_BUF2_STATUS_PMIC,
		     CLK_BUF3_STATUS_PMIC, CLK_BUF4_STATUS_PMIC,
		     CLK_BUF5_STATUS_PMIC, CLK_BUF6_STATUS_PMIC,
		     CLK_BUF7_STATUS_PMIC);
	clk_buf_pr_info("%s: PMIC_CLK_BUF?_DRV_CURR=%d %d %d %d %d %d %d\n", __func__,
		     PMIC_CLK_BUF1_DRIVING_CURR,
		     PMIC_CLK_BUF2_DRIVING_CURR,
		     PMIC_CLK_BUF3_DRIVING_CURR,
		     PMIC_CLK_BUF4_DRIVING_CURR,
		     PMIC_CLK_BUF5_DRIVING_CURR,
		     PMIC_CLK_BUF6_DRIVING_CURR,
		     PMIC_CLK_BUF7_DRIVING_CURR);
}

void clk_buf_dump_clkbuf_log(void)
{
	u32 pmic_cw00 = 0, pmic_cw02 = 0, pmic_cw11 = 0, pmic_cw14 = 0,
	    pmic_cw16 = 0, top_spi_con1 = 0;

	pmic_read_interface(PMIC_XO_EXTBUF1_MODE_ADDR, &pmic_cw00,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_XO_BUFLDOK_EN_ADDR, &pmic_cw02,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_XO_EXTBUF6_MODE_ADDR, &pmic_cw11,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_XO_EXTBUF2_CLKSEL_MAN_ADDR, &pmic_cw14,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_XO_EXTBUF1_ISET_M_ADDR, &pmic_cw16,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_RG_SRCLKEN_IN3_EN_ADDR, &top_spi_con1,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	clk_buf_pr_info("%s DCXO_CW00=0x%x, CW02=0x%x, CW11=0x%x, CW14=0x%x, CW16=0x%x, top_spi_con1=0x%x\n",
		     __func__, pmic_cw00, pmic_cw02, pmic_cw11, pmic_cw14,
		     pmic_cw16, top_spi_con1);
}

#ifdef CLKBUF_AUTOK_ENABLE
static u32 dcxo_dbg_read_auxout(u16 sel)
{
	u32 rg_auxout = 0;

	pmic_config_interface(PMIC_DCXO_CW18, sel,
			      PMIC_XO_STATIC_AUXOUT_SEL_MASK,
			      PMIC_XO_STATIC_AUXOUT_SEL_SHIFT);
	pmic_read_interface(PMIC_DCXO_CW19, &rg_auxout,
			    PMIC_XO_STATIC_AUXOUT_MASK,
			    PMIC_XO_STATIC_AUXOUT_SHIFT);
	clk_buf_pr_dbg("%s: sel=%d, rg_auxout=0x%x\n", __func__, sel, rg_auxout);

	return rg_auxout;
}

static bool clk_buf_is_auto_calc_ready(void)
{
	if (((dcxo_dbg_read_auxout(42) & (0x3 << 2)) >> 2) == 3)
		return true;
	else
		return false;
}

static void clk_buf_get_drv_curr(void)
{
	u32 rg_auxout = 0;

	rg_auxout = dcxo_dbg_read_auxout(5);
	clk_buf_pr_dbg("%s: sel io_dbg4: rg_auxout=0x%x\n", __func__, rg_auxout);
	clkbuf_drv_curr_auxout[XO_SOC] = (rg_auxout & (0x3 << 1)) >> 1;
	clkbuf_drv_curr_auxout[XO_WCN] = (rg_auxout & (0x3 << 7)) >> 7;

	rg_auxout = dcxo_dbg_read_auxout(6);
	clk_buf_pr_dbg("%s: sel io_dbg5: rg_auxout=0x%x\n", __func__, rg_auxout);
	clkbuf_drv_curr_auxout[XO_NFC] = (rg_auxout & (0x3 << 1)) >> 1;
	clkbuf_drv_curr_auxout[XO_CEL] = (rg_auxout & (0x3 << 7)) >> 7;

	rg_auxout = dcxo_dbg_read_auxout(7);
	clk_buf_pr_dbg("%s: sel io_dbg6: rg_auxout=0x%x\n", __func__, rg_auxout);
	clkbuf_drv_curr_auxout[XO_NONE] = (rg_auxout & (0x3 << 1)) >> 1;
	clkbuf_drv_curr_auxout[XO_AUD] = (rg_auxout & (0x3 << 7)) >> 7;
	clkbuf_drv_curr_auxout[XO_EXT] = (rg_auxout & (0x3 << 12)) >> 12;

	clk_buf_pr_info("%s: PMIC_CLK_BUF?_DRV_CURR_AUXOUT=%d %d %d %d %d %d %d\n", __func__,
		     clkbuf_drv_curr_auxout[XO_SOC],
		     clkbuf_drv_curr_auxout[XO_WCN],
		     clkbuf_drv_curr_auxout[XO_NFC],
		     clkbuf_drv_curr_auxout[XO_CEL],
		     clkbuf_drv_curr_auxout[XO_NONE],
		     clkbuf_drv_curr_auxout[XO_AUD],
		     clkbuf_drv_curr_auxout[XO_EXT]);
}

static bool clk_buf_is_auto_calc_enabled(void)
{
	u32 autok = 0;

	pmic_read_interface(PMIC_XO_BUFLDOK_EN_ADDR, &autok,
			    PMIC_XO_BUFLDOK_EN_MASK, PMIC_XO_BUFLDOK_EN_SHIFT);
	if (autok)
		return true;
	else
		return false;
}

static void clk_buf_set_auto_calc(u8 onoff)
{
	if (onoff) {
		pmic_config_interface(PMIC_XO_BUFLDOK_EN_ADDR, 0,
				    PMIC_XO_BUFLDOK_EN_MASK, PMIC_XO_BUFLDOK_EN_SHIFT);
		udelay(100);
		pmic_config_interface(PMIC_XO_BUFLDOK_EN_ADDR, 1,
				    PMIC_XO_BUFLDOK_EN_MASK, PMIC_XO_BUFLDOK_EN_SHIFT);
		mdelay(1);
	} else
		pmic_config_interface(PMIC_XO_BUFLDOK_EN_ADDR, 0,
				    PMIC_XO_BUFLDOK_EN_MASK, PMIC_XO_BUFLDOK_EN_SHIFT);
}
#endif /* CLKBUF_AUTOK_ENABLE */

static void clk_buf_set_manual_drv_curr(u32 *drv_curr_vals)
{
	u32 drv_curr_val = 0, drv_curr_mask = 0, drv_curr_shift = 0;

	drv_curr_val =
		(drv_curr_vals[XO_SOC] << PMIC_XO_EXTBUF1_ISET_M_SHIFT) |
		(drv_curr_vals[XO_WCN] << PMIC_XO_EXTBUF2_ISET_M_SHIFT) |
		(drv_curr_vals[XO_NFC] << PMIC_XO_EXTBUF3_ISET_M_SHIFT) |
		(drv_curr_vals[XO_CEL] << PMIC_XO_EXTBUF4_ISET_M_SHIFT) |
		/* (drv_curr_vals[XO_AUD] << PMIC_XO_EXTBUF5_ISET_M_SHIFT) | */
		(drv_curr_vals[XO_AUD] << PMIC_XO_EXTBUF6_ISET_M_SHIFT) |
		(drv_curr_vals[XO_EXT] << PMIC_XO_EXTBUF7_ISET_M_SHIFT);
	drv_curr_mask =
		(PMIC_XO_EXTBUF1_ISET_M_MASK << PMIC_XO_EXTBUF1_ISET_M_SHIFT) |
		(PMIC_XO_EXTBUF2_ISET_M_MASK << PMIC_XO_EXTBUF2_ISET_M_SHIFT) |
		(PMIC_XO_EXTBUF3_ISET_M_MASK << PMIC_XO_EXTBUF3_ISET_M_SHIFT) |
		(PMIC_XO_EXTBUF4_ISET_M_MASK << PMIC_XO_EXTBUF4_ISET_M_SHIFT) |
		/* (PMIC_XO_EXTBUF5_ISET_M_MASK << PMIC_XO_EXTBUF5_ISET_M_SHIFT) | */
		(PMIC_XO_EXTBUF6_ISET_M_MASK << PMIC_XO_EXTBUF6_ISET_M_SHIFT) |
		(PMIC_XO_EXTBUF7_ISET_M_MASK << PMIC_XO_EXTBUF7_ISET_M_SHIFT);
	drv_curr_shift = PMIC_XO_EXTBUF1_ISET_M_SHIFT;

	pmic_config_interface(PMIC_XO_EXTBUF1_ISET_M_ADDR, drv_curr_val,
			      drv_curr_mask, drv_curr_shift);
	clk_buf_pr_info("%s: drv_curr_val/mask/shift=0x%x %x %x\n", __func__,
		     drv_curr_val, drv_curr_mask, drv_curr_shift);
}

#ifdef CLKBUF_AUTOK_ENABLE
static void clk_buf_get_xo_en(void)
{
	u32 rg_auxout = 0;

	rg_auxout = dcxo_dbg_read_auxout(5);
	clk_buf_pr_dbg("%s: sel io_dbg4: rg_auxout=0x%x\n", __func__, rg_auxout);
	xo_en_stat[XO_SOC] = (rg_auxout & (0x1 << 0)) >> 0;
	xo_en_stat[XO_WCN] = (rg_auxout & (0x1 << 6)) >> 6;

	rg_auxout = dcxo_dbg_read_auxout(6);
	clk_buf_pr_dbg("%s: sel io_dbg5: rg_auxout=0x%x\n", __func__, rg_auxout);
	xo_en_stat[XO_NFC] = (rg_auxout & (0x1 << 0)) >> 0;
	xo_en_stat[XO_CEL] = (rg_auxout & (0x1 << 6)) >> 6;
	xo_en_stat[XO_EXT] = (rg_auxout & (0x1 << 12)) >> 12;

	rg_auxout = dcxo_dbg_read_auxout(7);
	clk_buf_pr_dbg("%s: sel io_dbg6: rg_auxout=0x%x\n", __func__, rg_auxout);
	xo_en_stat[XO_AUD] = (rg_auxout & (0x1 << 6)) >> 6;

	clk_buf_pr_info("%s: PMIC_CLK_BUF?_EN_STAT=%d %d %d %d %d %d %d\n", __func__,
		     xo_en_stat[XO_SOC],
		     xo_en_stat[XO_WCN],
		     xo_en_stat[XO_NFC],
		     xo_en_stat[XO_CEL],
		     xo_en_stat[XO_NONE],
		     xo_en_stat[XO_AUD],
		     xo_en_stat[XO_EXT]);
}
#endif /* CLKBUF_AUTOK_ENABLE */

#ifdef CONFIG_PM
static ssize_t clk_buf_ctrl_store(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	u32 clk_buf_en[CLKBUF_NUM], i;
	char cmd[32];

	if (sscanf(buf, "%31s %x %x %x %x %x %x %x", cmd, &clk_buf_en[XO_SOC],
		   &clk_buf_en[XO_WCN], &clk_buf_en[XO_NFC], &clk_buf_en[XO_CEL],
		   &clk_buf_en[XO_NONE], &clk_buf_en[XO_AUD], &clk_buf_en[XO_EXT])
	    != (CLKBUF_NUM + 1))
		return -EPERM;

	if (!strcmp(cmd, "pmic")) {
#ifndef CLKBUF_BRINGUP
		if (!is_pmic_clkbuf)
			return -EINVAL;
#endif

		mutex_lock(&clk_buf_ctrl_lock);

		for (i = 0; i < CLKBUF_NUM; i++)
			pmic_clk_buf_swctrl[i] = clk_buf_en[i];

		pmic_clk_buf_ctrl(pmic_clk_buf_swctrl);

		mutex_unlock(&clk_buf_ctrl_lock);

		return count;
	} else if (!strcmp(cmd, "pwrap")) {
		if (!is_pmic_clkbuf)
			return -EINVAL;

		mutex_lock(&clk_buf_ctrl_lock);

		for (i = 0; i < CLKBUF_NUM; i++) {
			if (i == XO_WCN) {
				if (clk_buf_en[i])
					pwrap_dcxo_en_flag |= DCXO_CONN_ENABLE;
				else
					pwrap_dcxo_en_flag &= ~DCXO_CONN_ENABLE;
			} else if (i == XO_NFC) {
				if (clk_buf_en[i])
					pwrap_dcxo_en_flag |= DCXO_NFC_ENABLE;
				else
					pwrap_dcxo_en_flag &= ~DCXO_NFC_ENABLE;
			}
		}

		clkbuf_writel(DCXO_ENABLE, pwrap_dcxo_en_flag);
		clk_buf_pr_info("%s: DCXO_ENABLE=0x%x, pwrap_dcxo_en_flag=0x%x\n", __func__,
			     clkbuf_readl(DCXO_ENABLE), pwrap_dcxo_en_flag);

		mutex_unlock(&clk_buf_ctrl_lock);

		return count;
	} else if (!strcmp(cmd, "drvcurr")) {
		if (!is_pmic_clkbuf)
			return -EINVAL;

		mutex_lock(&clk_buf_ctrl_lock);

#ifdef CLKBUF_AUTOK_ENABLE
		if (clk_buf_is_auto_calc_enabled())
			clk_buf_set_auto_calc(0);
#endif
		clk_buf_set_manual_drv_curr(clk_buf_en);

		mutex_unlock(&clk_buf_ctrl_lock);

		return count;
	} else {
		return -EINVAL;
	}
}

static ssize_t clk_buf_ctrl_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	int len = 0;
	u32 pmic_cw00 = 0, pmic_cw02 = 0, pmic_cw11 = 0, pmic_cw12 = 0, pmic_cw14 = 0,
	    pmic_cw16 = 0, pwr_sta = 0;
	u32 buf2_mode, buf3_mode, buf4_mode, buf6_mode, buf7_mode;
	u32 buf2_en_m, buf3_en_m, buf4_en_m, buf6_en_m, buf7_en_m, srclken_in3;
	int buf2_status = -1, buf3_status = -1, buf4_status = -1, buf6_status = -1, buf7_status = -1;
	int buf24_en, buf234_en, buf246_en, buf247_en;
	u32 xo_extbuf2_clksel_man, xo_extbuf4_clksel_man;
	bool srcclkena_o1, conn_srclkena, srcclkeni_o0;

#ifdef CLKBUF_AUTOK_ENABLE
	clk_buf_get_drv_curr();
	clk_buf_get_xo_en();
#endif
	len += snprintf(buf+len, PAGE_SIZE-len,
			"********** PMIC clock buffer state (%s) **********\n",
			(is_pmic_clkbuf ? "on" : "off"));
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_SOC   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF1_STATUS_PMIC, pmic_clk_buf_swctrl[XO_SOC]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_WCN   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF2_STATUS_PMIC, pmic_clk_buf_swctrl[XO_WCN]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_NFC   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF3_STATUS_PMIC, pmic_clk_buf_swctrl[XO_NFC]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_CEL   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF4_STATUS_PMIC, pmic_clk_buf_swctrl[XO_CEL]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_NONE   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF5_STATUS_PMIC, pmic_clk_buf_swctrl[XO_NONE]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_AUD    SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF6_STATUS_PMIC, pmic_clk_buf_swctrl[XO_AUD]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"XO_EXT   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
			CLK_BUF7_STATUS_PMIC, pmic_clk_buf_swctrl[XO_EXT]);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"\n********** clock buffer command help **********\n");
	len += snprintf(buf+len, PAGE_SIZE-len,
			"PMIC switch on/off: echo pmic en1 en2 en3 en4 en5 en6 en7 > /sys/power/clk_buf/clk_buf_ctrl\n");
	len += snprintf(buf+len, PAGE_SIZE-len,
			"Set drv curr(0~3): echo drvcurr v1 v2 v3 v4 v5 v6 v7 > /sys/power/clk_buf/clk_buf_ctrl\n");
	len += snprintf(buf+len, PAGE_SIZE-len,
			"\n********** clock buffer debug info **********\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "pmic_drv_curr_vals=%d %d %d %d %d %d %d\n",
			PMIC_CLK_BUF1_DRIVING_CURR, PMIC_CLK_BUF2_DRIVING_CURR,
			PMIC_CLK_BUF3_DRIVING_CURR, PMIC_CLK_BUF4_DRIVING_CURR,
			PMIC_CLK_BUF5_DRIVING_CURR, PMIC_CLK_BUF6_DRIVING_CURR,
			PMIC_CLK_BUF7_DRIVING_CURR);
#ifdef CLKBUF_AUTOK_ENABLE
	len += snprintf(buf+len, PAGE_SIZE-len, "clkbuf_drv_curr_auxout=%d %d %d %d %d %d %d\n",
		     clkbuf_drv_curr_auxout[XO_SOC],
		     clkbuf_drv_curr_auxout[XO_WCN],
		     clkbuf_drv_curr_auxout[XO_NFC],
		     clkbuf_drv_curr_auxout[XO_CEL],
		     clkbuf_drv_curr_auxout[XO_NONE],
		     clkbuf_drv_curr_auxout[XO_AUD],
		     clkbuf_drv_curr_auxout[XO_EXT]);
#endif

	len += snprintf(buf+len, PAGE_SIZE-len, "clkbuf_ctrl_stat=0x%x, pwrap_dcxo_en_flag=0x%x\n",
			clkbuf_ctrl_stat, pwrap_dcxo_en_flag);

	pmic_read_interface_nolock(PMIC_DCXO_CW00, &pmic_cw00,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface_nolock(PMIC_DCXO_CW02, &pmic_cw02,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface_nolock(PMIC_DCXO_CW11, &pmic_cw11,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface_nolock(PMIC_DCXO_CW14, &pmic_cw14,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface_nolock(PMIC_DCXO_CW16, &pmic_cw16,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	len += snprintf(buf+len, PAGE_SIZE-len, "DCXO_CW00/CW02/CW11/CW14/CW16=0x%x 0x%x 0x%x 0x%x 0x%x\n",
			pmic_cw00, pmic_cw02, pmic_cw11, pmic_cw14, pmic_cw16);
	pmic_read_interface_nolock(PMIC_RG_SRCLKEN_IN3_EN_ADDR, &srclken_in3,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	len += snprintf(buf+len, PAGE_SIZE-len, "SRCLKEN_IN3_EN(srclken_conn)=0x%x\n", srclken_in3);

	len += snprintf(buf+len, PAGE_SIZE-len, "DCXO_CONN_ADR0/WDATA0/ADR1/WDATA1=0x%x %x %x %x\n",
		     clkbuf_readl(DCXO_CONN_ADR0),
		     clkbuf_readl(DCXO_CONN_WDATA0),
		     clkbuf_readl(DCXO_CONN_ADR1),
		     clkbuf_readl(DCXO_CONN_WDATA1));
	len += snprintf(buf+len, PAGE_SIZE-len, "DCXO_NFC_ADR0/WDATA0/ADR1/WDATA1/EN=0x%x %x %x %x %x\n",
		     clkbuf_readl(DCXO_NFC_ADR0),
		     clkbuf_readl(DCXO_NFC_WDATA0),
		     clkbuf_readl(DCXO_NFC_ADR1),
		     clkbuf_readl(DCXO_NFC_WDATA1),
		     clkbuf_readl(DCXO_ENABLE));

	pwr_sta = clkbuf_readl(PWR_STATUS);

	len += snprintf(buf+len, PAGE_SIZE-len,
		"PWR_STATUS=0x%x, CONN(10)=%d MD(7,6)=0x%x\n",
		pwr_sta, (pwr_sta & PWR_STATUS_CONN) >> PWR_STATUS_CONN_SHIFT,
		(pwr_sta & (PWR_STATUS_MD | PWR_STATUS_C2K)) >> PWR_STATUS_C2K_SHIFT);

	len += snprintf(buf+len, PAGE_SIZE-len, "bblpm_cnt=%u, MD1/C2K_PWR_CON=0x%x/%x\n",
			bblpm_cnt, clkbuf_readl(MD1_PWR_CON),
			clkbuf_readl(C2K_PWR_CON));

	conn_srclkena = !!(spm_read(PCM_REG13_DATA) & R13_CONN_SRCCLKENA);
	srcclkeni_o0 = !!(spm_read(PCM_REG13_DATA) & R13_SRCCLKENI_0);
	srcclkena_o1 = !!(spm_read(PCM_REG13_DATA) & R13_MD_VRF18_REQ_0);

	pmic_read_interface(PMIC_DCXO_CW00, &pmic_cw00, PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_DCXO_CW11, &pmic_cw11, PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_DCXO_CW12, &pmic_cw12, PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_DCXO_CW14, &pmic_cw14, PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_DCXO_CW16, &pmic_cw16, PMIC_REG_MASK, PMIC_REG_SHIFT);

	buf2_mode = (pmic_cw00 >> PMIC_XO_EXTBUF2_MODE_SHIFT) & PMIC_XO_EXTBUF2_MODE_MASK;
	buf3_mode = (pmic_cw00 >> PMIC_XO_EXTBUF3_MODE_SHIFT) & PMIC_XO_EXTBUF3_MODE_MASK;
	buf4_mode = (pmic_cw00 >> PMIC_XO_EXTBUF4_MODE_SHIFT) & PMIC_XO_EXTBUF4_MODE_MASK;
	buf6_mode = (pmic_cw11 >> PMIC_XO_EXTBUF6_MODE_SHIFT) & PMIC_XO_EXTBUF6_MODE_MASK;
	buf7_mode = (pmic_cw11 >> PMIC_XO_EXTBUF7_MODE_SHIFT) & PMIC_XO_EXTBUF7_MODE_MASK;

	buf2_en_m = (pmic_cw00 >> PMIC_XO_EXTBUF2_EN_M_SHIFT) & PMIC_XO_EXTBUF2_EN_M_MASK;
	buf3_en_m = (pmic_cw00 >> PMIC_XO_EXTBUF3_EN_M_SHIFT) & PMIC_XO_EXTBUF3_EN_M_MASK;
	buf4_en_m = (pmic_cw00 >> PMIC_XO_EXTBUF4_EN_M_SHIFT) & PMIC_XO_EXTBUF4_EN_M_MASK;
	buf6_en_m = (pmic_cw11 >> PMIC_XO_EXTBUF6_EN_M_SHIFT) & PMIC_XO_EXTBUF6_EN_M_MASK;
	buf7_en_m = (pmic_cw11 >> PMIC_XO_EXTBUF7_EN_M_SHIFT) & PMIC_XO_EXTBUF7_EN_M_MASK;

	xo_extbuf4_clksel_man = (pmic_cw12 >> PMIC_XO_EXTBUF4_CLKSEL_MAN_SHIFT)
		& PMIC_XO_EXTBUF4_CLKSEL_MAN_MASK;
	xo_extbuf2_clksel_man = (pmic_cw14 >> PMIC_XO_EXTBUF2_CLKSEL_MAN_SHIFT)
		& PMIC_XO_EXTBUF2_CLKSEL_MAN_MASK;

	buf24_en = ((srcclkena_o1 & ~(xo_extbuf4_clksel_man)) || (buf4_en_m & xo_extbuf4_clksel_man)) ||
		((conn_srclkena & ~(xo_extbuf2_clksel_man)) || (buf2_en_m & xo_extbuf2_clksel_man));
	buf234_en = buf24_en | buf3_en_m;
	buf246_en = buf24_en | buf6_en_m;
	buf247_en = buf24_en | buf7_en_m;

	if (buf2_mode == 0x3)
		buf2_status = buf24_en;
	else if (buf2_mode == 0x2)
		buf2_status = conn_srclkena;
	else if (buf2_mode == 0x0)
		buf2_status = buf2_en_m;

	if (buf3_mode == 0x3)
		buf3_status = buf234_en;
	else if (buf3_mode == 0x0)
		buf3_status = buf3_en_m;

	if (buf4_mode == 0x3)
		buf4_status = buf24_en;
	else if (buf4_mode == 0x2)
		buf4_status = srcclkena_o1;
	else if (buf4_mode == 0x0)
		buf4_status = buf4_en_m;

	if (buf6_mode == 0x3)
		buf6_status = buf246_en;
	else if (buf6_mode == 0x0)
		buf6_status = buf6_en_m;

	if (buf7_mode == 0x3)
		buf7_status = buf247_en;
	else if (buf7_mode == 0x0)
		buf7_status = buf7_en_m;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"conn_srclken:%d srclkenai0:%d srclken_o1:%d,  buf2/3/4/6/7 en_m=%d/%d/%d/%d/%d\n",
		conn_srclkena, srcclkeni_o0, srcclkena_o1,
		buf2_en_m, buf3_en_m, buf4_en_m, buf6_en_m, buf7_en_m);

	len += snprintf(buf+len, PAGE_SIZE-len,
		"buf2/3/4/6/7 mode=%d/%d/%d/%d/%d, buf2/3/4/6/7 status=%d/%d/%d/%d/%d\n",
		buf2_mode, buf3_mode, buf4_mode, buf6_mode, buf7_mode,
		buf2_status, buf3_status, buf4_status, buf6_status, buf7_status);

	return len;
}

static ssize_t clk_buf_debug_store(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int debug = 0;

	if (!kstrtoint(buf, 10, &debug)) {
		if (debug == 0)
			clkbuf_debug = false;
		else if (debug == 1)
			clkbuf_debug = true;
		else if (debug == 2)
			clk_buf_ctrl(CLK_BUF_AUDIO, true);
		else if (debug == 3)
			clk_buf_ctrl(CLK_BUF_AUDIO, false);
		else if (debug == 4)
			clk_buf_ctrl(CLK_BUF_CHG, true);
		else if (debug == 5)
			clk_buf_ctrl(CLK_BUF_CHG, false);
		else if (debug == 6)
			clk_buf_ctrl(CLK_BUF_UFS, true);
		else if (debug == 7)
			clk_buf_ctrl(CLK_BUF_UFS, false);
		else if (debug == 8)
			clk_buf_ctrl(CLK_BUF_CONN, true);
		else if (debug == 9)
			clk_buf_ctrl(CLK_BUF_CONN, false);
		else if (debug == 10)
			clk_buf_ctrl_internal(CLK_BUF_CONN, false);
		else if (debug == 11)
			clk_buf_ctrl_internal(CLK_BUF_CONN, true);
		else if (debug == 12)
			clk_buf_ctrl_internal(CLK_BUF_NFC, false);
		else if (debug == 13)
			clk_buf_ctrl_internal(CLK_BUF_NFC, true);
		else if (debug == 14)
			clk_buf_ctrl_internal(CLK_BUF_UFS, false);
		else if (debug == 15)
			clk_buf_ctrl_internal(CLK_BUF_UFS, true);
		else if (debug == 16)
			bblpm_switch = 1;
		else if (debug == 17)
			bblpm_switch = 0;
		else
			clk_buf_pr_info("bad argument!! should be 0 or 1 [0: disable, 1: enable]\n");
	} else
		return -EPERM;

	return count;
}

static ssize_t clk_buf_debug_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	int len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "clkbuf_debug=%d\n", clkbuf_debug);

	return len;
}

DEFINE_ATTR_RW(clk_buf_ctrl);
DEFINE_ATTR_RW(clk_buf_debug);

static struct attribute *clk_buf_attrs[] = {
	/* for clock buffer control */
	__ATTR_OF(clk_buf_ctrl),
	__ATTR_OF(clk_buf_debug),

	/* must */
	NULL,
};

static struct attribute_group clk_buf_attr_group = {
	.name	= "clk_buf",
	.attrs	= clk_buf_attrs,
};

int clk_buf_fs_init(void)
{
	int r = 0;

	/* create /sys/power/clk_buf/xxx */
	r = sysfs_create_group(power_kobj, &clk_buf_attr_group);
	if (r)
		clk_buf_pr_err("FAILED TO CREATE /sys/power/clk_buf (%d)\n", r);

	return r;
}
#else /* !CONFIG_PM */
int clk_buf_fs_init(void)
{
	return 0;
}
#endif /* CONFIG_PM */

#if defined(CONFIG_OF)
int clk_buf_dts_map(void)
{
	struct device_node *node;
	u32 vals[CLKBUF_NUM] = {0, 0, 0, 0, 0, 0, 0};
	int ret = -1;

	node = of_find_compatible_node(NULL, NULL, "mediatek,pmic_clock_buffer");
	if (node) {
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-config",
						 vals, CLKBUF_NUM);
		if (!ret) {
			CLK_BUF1_STATUS_PMIC = vals[0];
			CLK_BUF2_STATUS_PMIC = vals[1];
			CLK_BUF3_STATUS_PMIC = vals[2];
			CLK_BUF4_STATUS_PMIC = vals[3];
			CLK_BUF5_STATUS_PMIC = vals[4];
			CLK_BUF6_STATUS_PMIC = vals[5];
			CLK_BUF7_STATUS_PMIC = vals[6];
		}
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-driving-current",
						 vals, CLKBUF_NUM);
		if (!ret) {
			PMIC_CLK_BUF1_DRIVING_CURR = vals[0];
			PMIC_CLK_BUF2_DRIVING_CURR = vals[1];
			PMIC_CLK_BUF3_DRIVING_CURR = vals[2];
			PMIC_CLK_BUF4_DRIVING_CURR = vals[3];
			PMIC_CLK_BUF5_DRIVING_CURR = vals[4];
			PMIC_CLK_BUF6_DRIVING_CURR = vals[5];
			PMIC_CLK_BUF7_DRIVING_CURR = vals[6];
		}
	} else {
		clk_buf_pr_err("%s can't find compatible node for pmic_clock_buffer\n", __func__);
		return -1;
	}
	node = of_find_compatible_node(NULL, NULL, "mediatek,pwrap");
	if (node)
		pwrap_base = of_iomap(node, 0);
	else {
		clk_buf_pr_err("%s can't find compatible node for pwrap\n",
		       __func__);
		return -1;
	}

	return 0;
}
#else /* !CONFIG_OF */
int clk_buf_dts_map(void)
{
	return 0;
}
#endif

void clk_buf_init_pmic_clkbuf(void)
{
	/* Dump registers before setting */
	clk_buf_dump_clkbuf_log();

#ifndef __KERNEL__
	/* Setup initial PMIC clock buffer setting */
	/* auto mode of driving current */
	/* clk_buf_set_auto_calc(0); */
	pmic_config_interface(PMIC_DCXO_CW00, PMIC_CW00_INIT_VAL,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_config_interface(PMIC_DCXO_CW11, PMIC_CW11_INIT_VAL,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);

	/* XO_WCN */
#ifdef CLKBUF_CONN_SUPPORT_CTRL_FROM_I1
	pmic_config_interface(PMIC_XO_EXTBUF2_CLKSEL_MAN_ADDR, 0x1,
			    PMIC_XO_EXTBUF2_CLKSEL_MAN_MASK,
			    PMIC_XO_EXTBUF2_CLKSEL_MAN_SHIFT);
#endif
#ifndef CLKBUF_CONN_SUPPORT_CTRL_FROM_I1
	/* XO_WCN: srclken_conn = 0 */
	pmic_config_interface(PMIC_RG_SRCLKEN_IN3_EN_ADDR, 0,
			    PMIC_RG_SRCLKEN_IN3_EN_MASK, PMIC_RG_SRCLKEN_IN3_EN_SHIFT);
#endif

	/* Check if the setting is ok */
	clk_buf_dump_clkbuf_log();
#endif /* #ifndef __KERNEL__ */

#ifdef CLKBUF_AUTOK_ENABLE
	if (clk_buf_is_auto_calc_ready())
		clk_buf_get_drv_curr();
#endif
}

void clk_buf_init_pmic_wrap(void)
{
#ifndef __KERNEL__
	/* Setup PMIC_WRAP setting for XO2 & XO3 */
#ifdef CLKBUF_CONN_SUPPORT_CTRL_FROM_I1
	clkbuf_writel(DCXO_CONN_ADR0, PMIC_DCXO_CW00_CLR_ADDR);
	clkbuf_writel(DCXO_CONN_WDATA0,
		      PMIC_XO_EXTBUF2_EN_M_MASK << PMIC_XO_EXTBUF2_EN_M_SHIFT);	/* bit5 = 0 */
	clkbuf_writel(DCXO_CONN_ADR1, PMIC_DCXO_CW00_SET_ADDR);
	clkbuf_writel(DCXO_CONN_WDATA1,
		      PMIC_XO_EXTBUF2_EN_M_MASK << PMIC_XO_EXTBUF2_EN_M_SHIFT);	/* bit5 = 1 */
#else
	clkbuf_writel(DCXO_CONN_ADR0, PMIC_RG_SRCLKEN_IN3_EN_ADDR);
	clkbuf_writel(DCXO_CONN_WDATA0,
		      0 << PMIC_RG_SRCLKEN_IN3_EN_SHIFT); /* bit0 = 0 */
	clkbuf_writel(DCXO_CONN_ADR1, PMIC_RG_SRCLKEN_IN3_EN_ADDR);
	clkbuf_writel(DCXO_CONN_WDATA1,
		      1 << PMIC_RG_SRCLKEN_IN3_EN_SHIFT); /* bit0 = 1 */
#endif
	clkbuf_writel(DCXO_NFC_ADR0, PMIC_DCXO_CW00_CLR_ADDR);
	clkbuf_writel(DCXO_NFC_WDATA0,
		      PMIC_XO_EXTBUF3_EN_M_MASK << PMIC_XO_EXTBUF3_EN_M_SHIFT);	/* bit8 = 0 */
	clkbuf_writel(DCXO_NFC_ADR1, PMIC_DCXO_CW00_SET_ADDR);
	clkbuf_writel(DCXO_NFC_WDATA1,
		      PMIC_XO_EXTBUF3_EN_M_MASK << PMIC_XO_EXTBUF3_EN_M_SHIFT);	/* bit8 = 1 */

	clkbuf_writel(DCXO_ENABLE, DCXO_CONN_ENABLE | DCXO_NFC_ENABLE);

	clk_buf_pr_info("%s: DCXO_CONN_ADR0/WDATA0/ADR1/WDATA1=0x%x/%x/%x/%x\n",
		     __func__, clkbuf_readl(DCXO_CONN_ADR0),
		     clkbuf_readl(DCXO_CONN_WDATA0),
		     clkbuf_readl(DCXO_CONN_ADR1),
		     clkbuf_readl(DCXO_CONN_WDATA1));
	clk_buf_pr_info("%s: DCXO_NFC_ADR0/WDATA0/ADR1/WDATA1/EN=0x%x/%x/%x/%x/%x\n",
		     __func__, clkbuf_readl(DCXO_NFC_ADR0),
		     clkbuf_readl(DCXO_NFC_WDATA0),
		     clkbuf_readl(DCXO_NFC_ADR1),
		     clkbuf_readl(DCXO_NFC_WDATA1),
		     clkbuf_readl(DCXO_ENABLE));
#endif /* #ifndef __KERNEL__ */
}

void clk_buf_init_pmic_swctrl(void)
{
	if (CLK_BUF1_STATUS_PMIC == CLOCK_BUFFER_DISABLE)
		pmic_clk_buf_swctrl[XO_SOC] = CLK_BUF_SW_DISABLE;
	else if (CLK_BUF2_STATUS_PMIC == CLOCK_BUFFER_DISABLE) {
		clk_buf_ctrl_internal(CLK_BUF_CONN, false);
		pmic_clk_buf_swctrl[XO_WCN] = CLK_BUF_SW_DISABLE;
	} else if (CLK_BUF3_STATUS_PMIC == CLOCK_BUFFER_DISABLE) {
		clk_buf_ctrl_internal(CLK_BUF_NFC, false);
		pmic_clk_buf_swctrl[XO_NFC] = CLK_BUF_SW_DISABLE;
	} else if (CLK_BUF4_STATUS_PMIC == CLOCK_BUFFER_DISABLE)
		pmic_clk_buf_swctrl[XO_CEL] = CLK_BUF_SW_DISABLE;
	else if (CLK_BUF5_STATUS_PMIC == CLOCK_BUFFER_DISABLE)
		pmic_clk_buf_swctrl[XO_NONE] = CLK_BUF_SW_DISABLE;
	else if (CLK_BUF6_STATUS_PMIC == CLOCK_BUFFER_DISABLE)
		pmic_clk_buf_swctrl[XO_AUD] = CLK_BUF_SW_DISABLE;
	else if (CLK_BUF7_STATUS_PMIC == CLOCK_BUFFER_DISABLE) {
		clk_buf_ctrl_internal(CLK_BUF_UFS, false);
		pmic_clk_buf_swctrl[XO_EXT] = CLK_BUF_SW_DISABLE;
	}
}

short is_clkbuf_bringup(void)
{
#ifdef CLKBUF_BRINGUP
	clk_buf_pr_info("%s: skipped for bring up\n", __func__);
	return 1;
#else
	return 0;
#endif
}

void clk_buf_post_init(void)
{
	/* no need to use XO_NFC if no NFC chip */
	/* clk_buf_ctrl_internal(CLK_BUF_NFC, false); */
#ifndef CONFIG_MTK_UFS_BOOTING
	/* no need to use XO_EXT if storage is emmc */
	clk_buf_ctrl_internal(CLK_BUF_UFS, false);
#endif
}

