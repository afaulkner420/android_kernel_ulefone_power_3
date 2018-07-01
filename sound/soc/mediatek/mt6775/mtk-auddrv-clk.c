/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Clk.c
 *
 * Project:
 * --------
 *   MT6759 Audio Driver clock control implement
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang (MTK02308)
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/clk.h>

#include "mtk-auddrv-common.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-afe.h"
#include "mtk-soc-digital-type.h"

#include <linux/spinlock.h>
#include <linux/delay.h>
#if defined(_MT_IDLE_HEADER) && !defined(CONFIG_FPGA_EARLY_PORTING)
#include "mtk_idle.h"
#include "mtk_clk_id.h"
#endif
#include <linux/err.h>
#include <linux/platform_device.h>

#define _MT_SPM_RESOURCE
#if defined(_MT_SPM_RESOURCE) && !defined(CONFIG_FPGA_EARLY_PORTING)
#include "mtk_spm_resource_req.h"
bool spm_resource_req(unsigned int user, unsigned int req_mask) __attribute__((weak));
#endif
/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/

static int APLL1Counter;
static int APLL2Counter;
static int Aud_APLL_DIV_APLL1_cntr;
static int Aud_APLL_DIV_APLL2_cntr;
static unsigned int MCLKFS = 128;
static unsigned int MCLKFS_HDMI = 256;

int Aud_Core_Clk_cntr;
int Aud_AFE_Clk_cntr;
int Aud_I2S_Clk_cntr;
int Aud_TDM_Clk_cntr;
int Aud_ADC_Clk_cntr;
int Aud_ADC2_Clk_cntr;
int Aud_ADC3_Clk_cntr;
int Aud_ADC_HIRES_Clk_cntr;
int Aud_ADC2_Hires_Clk_cntr;
int Aud_ANA_Clk_cntr;
#ifdef _NON_COMMON_FEATURE_READY
int Aud_HDMI_Clk_cntr;
#endif
int Aud_APLL22M_Clk_cntr;
int Aud_APLL24M_Clk_cntr;
int Aud_APLL1_Tuner_cntr;
int Aud_APLL2_Tuner_cntr;
static int Aud_EMI_cntr;

static DEFINE_SPINLOCK(auddrv_Clk_lock);

/* amp mutex lock */
static DEFINE_MUTEX(auddrv_pmic_mutex);
static DEFINE_MUTEX(audEMI_Clk_mutex);
static DEFINE_MUTEX(auddrv_clk_mutex);


/* AUDIO_APLL_DIVIDER_GROUP may vary by chip!!! */
enum AUDIO_APLL_DIVIDER_GROUP {
	AUDIO_APLL1_DIV0,
	AUDIO_APLL2_DIV0,
	AUDIO_APLL12_DIV0,
	AUDIO_APLL12_DIV1,
	AUDIO_APLL12_DIV2,
	AUDIO_APLL12_DIV3,
	AUDIO_APLL12_DIV4,
	AUDIO_APLL12_DIVB,
	AUDIO_APLL12_DIV5,
	AUDIO_APLL_DIV_NUM
};

/* mI2SAPLLDivSelect may vary by chip!!! */
static const unsigned int mI2SAPLLDivSelect[Soc_Aud_I2S_CLKDIV_NUMBER] = {
	AUDIO_APLL1_DIV0,
	AUDIO_APLL2_DIV0,
	AUDIO_APLL12_DIV0,
	AUDIO_APLL12_DIV1,
	AUDIO_APLL12_DIV2,
	AUDIO_APLL12_DIV3,
	AUDIO_APLL12_DIV4,
	AUDIO_APLL12_DIVB,
	AUDIO_APLL12_DIV5
};

enum audio_system_clock_type {
	CLOCK_AFE = 0,
	CLOCK_I2S,
	CLOCK_DAC,
	CLOCK_DAC_PREDIS,
	CLOCK_ADC,
	CLOCK_ADDA6_ADC,
	CLOCK_TML,
	CLOCK_APLL22M,
	CLOCK_APLL24M,
	CLOCK_APLL1_TUNER,
	CLOCK_APLL2_TUNER,
/*	CLOCK_TDM,*/
	CLOCK_ADC_HIRES,
	CLOCK_ADDA6_ADC_HIRES,
	CLOCK_ADC_HIRES_TML,
	CLOCK_SCP_SYS_AUD,
	CLOCK_INFRA_SYS_AUDIO,
	CLOCK_MUX_AUDIO,
	CLOCK_TOP_SYSPLL3_D4,
	CLOCK_MUX_AUDIOINTBUS,
	CLOCK_TOP_SYSPLL1_D4,
	CLOCK_TOP_SYSPLL1_D2,
	CLK_TOP_F_FAUD26M_EN,
	CLOCK_CLK26M,
	CLOCK_NUM
};

struct audio_clock_attr {
	const char *name;
	bool clk_prepare;
	bool clk_status;
	struct clk *clock;
};

static struct audio_clock_attr aud_clks[CLOCK_NUM] = {
		[CLOCK_AFE] = {"aud_afe_clk", false, false, NULL},
		[CLOCK_I2S] = {"aud_i2s_clk", false, false, NULL},			/* AudDrv_I2S_Clk_On, suspend */
		[CLOCK_DAC] = {"aud_dac_clk", false, false, NULL},			/* AudDrv_Clk_On only */
		[CLOCK_DAC_PREDIS] = {"aud_dac_predis_clk", false, false, NULL},	/* AudDrv_Clk_On only */
		[CLOCK_ADC] = {"aud_adc_clk", false, false, NULL},			/* AudDrv_ADC_Clk_On only */
		[CLOCK_ADDA6_ADC] = {"aud_adda6_adc_clk", false, false, NULL},		/* AudDrv_ADC2_Clk_On only */
		[CLOCK_TML] = {"aud_tml_clk", false, false, NULL},			/* NOT USED */
		[CLOCK_APLL22M] = {"aud_apll22m_clk", false, false, NULL},
		[CLOCK_APLL24M] = {"aud_apll24m_clk", false, false, NULL},
		[CLOCK_APLL1_TUNER] = {"aud_apll1_tuner_clk", false, false, NULL},
		[CLOCK_APLL2_TUNER] = {"aud_apll2_tuner_clk", false, false, NULL},
		[CLOCK_ADC_HIRES] = {"aud_adc_hires_clk", false, false, NULL}, /* use this clock when HIRES */
		[CLOCK_ADDA6_ADC_HIRES] = {"aud_adda6_adc_hires_clk", false, false, NULL},
		[CLOCK_ADC_HIRES_TML] = {"aud_adc_hires_tml_clk", false, false, NULL}, /* use this clock when HIRES */
		[CLOCK_SCP_SYS_AUD] = {"scp_sys_audio", false, false, NULL},
		[CLOCK_INFRA_SYS_AUDIO] = {"aud_infra_clk", false, false, NULL},
		[CLOCK_MUX_AUDIO] = {"top_mux_audio", false, false, NULL},
		[CLOCK_TOP_SYSPLL3_D4] = {"top_sys_pll3_d4", false, false, NULL},
		[CLOCK_MUX_AUDIOINTBUS] = {"top_mux_audio_int", false, false, NULL},	/* AudDrv_AUDINTBUS_Sel */
		[CLOCK_TOP_SYSPLL1_D4] = {"top_sys_pll1_d4", false, false, NULL},	/* AudDrv_AUDINTBUS_Sel */
		[CLOCK_TOP_SYSPLL1_D2] = {"top_sys_pll1_d2", false, false, NULL},
		[CLK_TOP_F_FAUD26M_EN] = {"top_sys_audio26m", false, false, NULL},
		[CLOCK_CLK26M] = {"top_clk26m_clk", false, false, NULL}
};

#if !defined(CONFIG_FPGA_EARLY_PORTING)
static int aud_clk_enable(const struct audio_clock_attr *clk)
{
	int ret;

	if (clk->clk_prepare) {
		ret = clk_enable(clk->clock);
		if (ret) {
			pr_err("%s(), clk_enable %s fail, ret %d\n",
			       __func__, clk->name, ret);
			return -EPERM;
		}
	} else {
		pr_warn("%s(), clk %s, not prepared\n", __func__, clk->name);
		return -EINVAL;
	}
	return 0;
}
#endif

int AudDrv_Clk_probe(void *dev)
{
	size_t i;
	int ret = 0;

	Aud_EMI_cntr = 0;

	pr_debug("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(aud_clks); i++) {
		aud_clks[i].clock = devm_clk_get(dev, aud_clks[i].name);
		if (IS_ERR(aud_clks[i].clock)) {
			ret = PTR_ERR(aud_clks[i].clock);
			pr_err("%s devm_clk_get %s fail %d\n", __func__, aud_clks[i].name, ret);
		} else {
			aud_clks[i].clk_status = true;
		}
	}

	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(aud_clks); i++) {
		if (i == CLOCK_SCP_SYS_AUD)	/* CLOCK_SCP_SYS_AUD is MTCMOS */
			continue;

		if (aud_clks[i].clk_status) {
			ret = clk_prepare(aud_clks[i].clock);
			if (ret) {
				pr_err("%s clk_prepare %s fail %d\n",
				       __func__, aud_clks[i].name, ret);
			} else {
				aud_clks[i].clk_prepare = true;
			}
		}
	}

	return ret;
}

void AudDrv_Clk_Deinit(void *dev)
{
	size_t i;

	pr_debug("%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(aud_clks); i++) {
		if (i == CLOCK_SCP_SYS_AUD)	/* CLOCK_SCP_SYS_AUD is MTCMOS */
			continue;

		if (aud_clks[i].clock && !IS_ERR(aud_clks[i].clock) && aud_clks[i].clk_prepare) {
			clk_unprepare(aud_clks[i].clock);
			aud_clks[i].clk_prepare = false;
		}
	}
}

void AudDrv_Clk_Global_Variable_Init(void)
{
	APLL1Counter = 0;
	APLL2Counter = 0;
	Aud_APLL_DIV_APLL1_cntr = 0;
	Aud_APLL_DIV_APLL2_cntr = 0;
	MCLKFS = 128;
	MCLKFS_HDMI = 256;
}

void AudDrv_Bus_Init(void)
{
	/* No need on 6759, system default set bit14 to 1 */
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_Clk_On / AudDrv_Clk_Off
 *
 * DESCRIPTION
 *  Enable/Disable PLL(26M clock) \ AFE clock
 *
 *****************************************************************************
*/

void AudDrv_AUDINTBUS_Sel(int parentidx)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)

	int ret = 0;

	if (!aud_clks[CLOCK_MUX_AUDIOINTBUS].clk_prepare ||
	    !aud_clks[CLOCK_TOP_SYSPLL1_D4].clk_prepare ||
	    !aud_clks[CLOCK_CLK26M].clk_prepare) {
		pr_warn("%s(), clk_prepare = false\n", __func__);
		goto EXIT;
	}

	PRINTK_AUD_CLK("+AudDrv_AUDINTBUS_Sel, parentidx = %d\n", parentidx);
	if (parentidx == 1) {
		ret = clk_set_parent(aud_clks[CLOCK_MUX_AUDIOINTBUS].clock,
				     aud_clks[CLOCK_TOP_SYSPLL1_D4].clock);
		if (ret) {
			pr_warn("%s clk_set_parent %s-%s fail %d\n",
			       __func__, aud_clks[CLOCK_MUX_AUDIOINTBUS].name,
			       aud_clks[CLOCK_TOP_SYSPLL1_D4].name, ret);
			goto EXIT;
		}
	} else if (parentidx == 0) {
		ret = clk_set_parent(aud_clks[CLOCK_MUX_AUDIOINTBUS].clock,
				     aud_clks[CLOCK_CLK26M].clock);
		if (ret) {
			pr_warn("%s clk_set_parent %s-%s fail %d\n",
			       __func__, aud_clks[CLOCK_MUX_AUDIOINTBUS].name,
			       aud_clks[CLOCK_CLK26M].name, ret);
			goto EXIT;
		}
	}
EXIT:
	PRINTK_AUD_CLK("-%s()\n", __func__);

#endif /* #if !defined(CONFIG_FPGA_EARLY_PORTING) */
}

void AudDrv_Clk_On(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	int ret = 0;

	PRINTK_AUD_CLK("+AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d\n", Aud_AFE_Clk_cntr);
	mutex_lock(&auddrv_clk_mutex);
	Aud_AFE_Clk_cntr++;
	if (Aud_AFE_Clk_cntr == 1) {
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_SCP_SYS_AUD].clk_status) {
			ret = clk_prepare_enable(aud_clks[CLOCK_SCP_SYS_AUD].clock);
			if (ret) {
				pr_err("%s [CCF]Aud clk_prepare_enable %s fail\n", __func__,
					aud_clks[CLOCK_SCP_SYS_AUD].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		}

		if (aud_clk_enable(&aud_clks[CLOCK_INFRA_SYS_AUDIO]))
			goto EXIT;

		if (aud_clk_enable(&aud_clks[CLOCK_MUX_AUDIO]))
			goto EXIT;

		ret = clk_set_parent(aud_clks[CLOCK_MUX_AUDIO].clock,
				     aud_clks[CLOCK_CLK26M].clock);
		if (ret) {
			pr_warn("%s clk_set_parent %s-%s fail %d\n",
			       __func__, aud_clks[CLOCK_MUX_AUDIO].name,
			       aud_clks[CLOCK_CLK26M].name, ret);
			AUDIO_AEE("");
			goto EXIT;
		}

		/* AUD_INTBUS */
		if (aud_clk_enable(&aud_clks[CLOCK_MUX_AUDIOINTBUS]))
			goto EXIT;
		AudDrv_AUDINTBUS_Sel(1);

		if (aud_clk_enable(&aud_clks[CLOCK_AFE]))
			goto EXIT;

		if (aud_clk_enable(&aud_clks[CLOCK_DAC]))
			goto EXIT;

		if (aud_clk_enable(&aud_clks[CLOCK_DAC_PREDIS]))
			goto EXIT;

		if (aud_clk_enable(&aud_clks[CLK_TOP_F_FAUD26M_EN]))
			goto EXIT;

		/* enable audio sys DCM for power saving */
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x1 << 29, 0x1 << 29);
#else
		SetInfraCfg(AUDIO_CG_CLR, 0x2000000, 0x2000000);
		/* bit 25=0, without 133m master and 66m slave bus clock cg gating */
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x4000, 0x06004044);
#endif
	}
EXIT:
	mutex_unlock(&auddrv_clk_mutex);
	PRINTK_AUD_CLK("-AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d\n", Aud_AFE_Clk_cntr);
#endif /* #if !defined(CONFIG_FPGA_EARLY_PORTING) */
}
EXPORT_SYMBOL(AudDrv_Clk_On);

void AudDrv_Clk_Off(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	PRINTK_AUD_CLK("+!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d\n", Aud_AFE_Clk_cntr);
	mutex_lock(&auddrv_clk_mutex);

	Aud_AFE_Clk_cntr--;
	if (Aud_AFE_Clk_cntr == 0) {
		/* Disable AFE clock */
#ifdef PM_MANAGER_API
		/* Make sure all IRQ status is cleared */
		Afe_Set_Reg(AFE_IRQ_MCU_CLR, 0xffff, 0xffff);
		Afe_Set_Reg(AFE_IRQ_MCU_CLR, 0xffff, 0xffff);

		if (aud_clks[CLK_TOP_F_FAUD26M_EN].clk_prepare)
			clk_disable(aud_clks[CLK_TOP_F_FAUD26M_EN].clock);

		if (aud_clks[CLOCK_AFE].clk_prepare)
			clk_disable(aud_clks[CLOCK_AFE].clock);

		if (aud_clks[CLOCK_DAC].clk_prepare)
			clk_disable(aud_clks[CLOCK_DAC].clock);

		if (aud_clks[CLOCK_DAC_PREDIS].clk_prepare)
			clk_disable(aud_clks[CLOCK_DAC_PREDIS].clock);

		AudDrv_AUDINTBUS_Sel(0);
		if (aud_clks[CLOCK_MUX_AUDIOINTBUS].clk_prepare)
			clk_disable(aud_clks[CLOCK_MUX_AUDIOINTBUS].clock);

		if (aud_clks[CLOCK_MUX_AUDIO].clk_prepare)
			clk_disable(aud_clks[CLOCK_MUX_AUDIO].clock);

		if (aud_clks[CLOCK_INFRA_SYS_AUDIO].clk_prepare)
			clk_disable(aud_clks[CLOCK_INFRA_SYS_AUDIO].clock);

		if (aud_clks[CLOCK_SCP_SYS_AUD].clk_status)
			clk_disable_unprepare(aud_clks[CLOCK_SCP_SYS_AUD].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x06000044, 0x06000044);
		/* bit25=1, with 133m mastesr and 66m slave bus clock cg gating */
#endif
	} else if (Aud_AFE_Clk_cntr < 0) {
		pr_warn("!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr<0 (%d)\n",
			Aud_AFE_Clk_cntr);
		AUDIO_ASSERT(true);
		Aud_AFE_Clk_cntr = 0;
	}
	mutex_unlock(&auddrv_clk_mutex);
	PRINTK_AUD_CLK("-!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d\n", Aud_AFE_Clk_cntr);
#endif /* #if !defined(CONFIG_FPGA_EARLY_PORTING) */
}
EXPORT_SYMBOL(AudDrv_Clk_Off);


/*****************************************************************************
 * FUNCTION
 *  AudDrv_ANA_Clk_On / AudDrv_ANA_Clk_Off
 *
 * DESCRIPTION
 *  Enable/Disable analog part clock
 *
 *****************************************************************************/
void AudDrv_ANA_Clk_On(void)
{
	mutex_lock(&auddrv_pmic_mutex);
	if (Aud_ANA_Clk_cntr == 0)
		PRINTK_AUD_CLK("+AudDrv_ANA_Clk_On, Aud_ANA_Clk_cntr:%d\n", Aud_ANA_Clk_cntr);

	Aud_ANA_Clk_cntr++;
	mutex_unlock(&auddrv_pmic_mutex);
	/* PRINTK_AUD_CLK("-AudDrv_ANA_Clk_Off, Aud_ANA_Clk_cntr:%d\n",Aud_ANA_Clk_cntr); */
}
EXPORT_SYMBOL(AudDrv_ANA_Clk_On);

void AudDrv_ANA_Clk_Off(void)
{
	/* PRINTK_AUD_CLK("+AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr:%d\n",  Aud_ANA_Clk_cntr); */
	mutex_lock(&auddrv_pmic_mutex);
	Aud_ANA_Clk_cntr--;
	if (Aud_ANA_Clk_cntr == 0) {
		PRINTK_AUD_CLK("+AudDrv_ANA_Clk_Off disable_clock Ana clk(%x)\n",
			       Aud_ANA_Clk_cntr);
		/* Disable ADC clock */
#ifdef PM_MANAGER_API

#else
		/* TODO:: open ADC clock.... */
#endif
	} else if (Aud_ANA_Clk_cntr < 0) {
		pr_warn("!! AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr<0 (%d)\n",
			Aud_ANA_Clk_cntr);
		AUDIO_ASSERT(true);
		Aud_ANA_Clk_cntr = 0;
	}
	mutex_unlock(&auddrv_pmic_mutex);
	/* PRINTK_AUD_CLK("-AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr:%d\n", Aud_ANA_Clk_cntr); */
}
EXPORT_SYMBOL(AudDrv_ANA_Clk_Off);

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC_Clk_On / AudDrv_ADC_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_ADC_Clk_On(void)
{
	/* PRINTK_AUDDRV("+AudDrv_ADC_Clk_On, Aud_ADC_Clk_cntr:%d\n", Aud_ADC_Clk_cntr); */
	int ret = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);

	if (Aud_ADC_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s enable_clock ADC clk(%x)\n", __func__,
			      Aud_ADC_Clk_cntr);
		/* Afe_Set_Reg(AUDIO_TOP_CON0, 0 << 24 , 1 << 24); */
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADC].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_ADC].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail", __func__, aud_clks[CLOCK_ADC].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error %s fail", __func__, aud_clks[CLOCK_ADC].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0 << 24, 1 << 24);
#endif
	}
	Aud_ADC_Clk_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_ADC_Clk_Off(void)
{
	unsigned long flags = 0;

	/* PRINTK_AUDDRV("+AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d\n", Aud_ADC_Clk_cntr); */
	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_ADC_Clk_cntr--;
	if (Aud_ADC_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s disable_clock ADC clk(%x)\n", __func__,
			      Aud_ADC_Clk_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADC].clk_prepare)
			clk_disable(aud_clks[CLOCK_ADC].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 1 << 24, 1 << 24);
#endif
	}
	if (Aud_ADC_Clk_cntr < 0) {
		PRINTK_AUDDRV("!! %s, Aud_ADC_Clk_cntr<0 (%d)\n", __func__,
			      Aud_ADC_Clk_cntr);
		Aud_ADC_Clk_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
	/* PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d\n", Aud_ADC_Clk_cntr); */
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC2_Clk_On / AudDrv_ADC2_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_ADC2_Clk_On(void)
{
	int ret = 0;
	unsigned long flags = 0;

	PRINTK_AUD_CLK("+%s %d\n", __func__, Aud_ADC2_Clk_cntr);
	spin_lock_irqsave(&auddrv_Clk_lock, flags);

	if (Aud_ADC2_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s  enable_clock ADC2 clk(%x)\n", __func__, Aud_ADC2_Clk_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADDA6_ADC].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_ADDA6_ADC].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail", __func__, aud_clks[CLOCK_ADDA6_ADC].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error Aud %s fail", __func__, aud_clks[CLOCK_ADDA6_ADC].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#else
		Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 20, 1 << 20);
#endif
	}
	Aud_ADC2_Clk_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_ADC2_Clk_Off(void)
{
	unsigned long flags = 0;

	/* PRINTK_AUDDRV("+%s %d\n", __func__,Aud_ADC2_Clk_cntr); */
	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_ADC2_Clk_cntr--;
	if (Aud_ADC2_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s disable_clock ADC2 clk(%x)\n", __func__,
			      Aud_ADC2_Clk_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADDA6_ADC].clk_prepare)
			clk_disable(aud_clks[CLOCK_ADDA6_ADC].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON1, 1 << 20, 1 << 20);
#endif
	}
	if (Aud_ADC2_Clk_cntr < 0) {
		PRINTK_AUDDRV("!! %s, Aud_ADC2_Clk_cntr<0 (%d)\n", __func__,
			      Aud_ADC2_Clk_cntr);
		Aud_ADC2_Clk_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
	/* PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d\n", Aud_ADC_Clk_cntr); */
}


/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC3_Clk_On / AudDrv_ADC3_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_ADC3_Clk_On(void)
{
	PRINTK_AUD_CLK("+%s %d\n", __func__, Aud_ADC3_Clk_cntr);
	mutex_lock(&auddrv_pmic_mutex);

	if (Aud_ADC3_Clk_cntr == 0)
		PRINTK_AUDDRV("+%s  enable_clock ADC clk(%x)\n", __func__, Aud_ADC3_Clk_cntr);

	Aud_ADC3_Clk_cntr++;
	mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_ADC3_Clk_Off(void)
{
	/* PRINTK_AUDDRV("+%s %d\n", __func__,Aud_ADC2_Clk_cntr); */
	mutex_lock(&auddrv_pmic_mutex);
	Aud_ADC3_Clk_cntr--;

	if (Aud_ADC3_Clk_cntr == 0)
		PRINTK_AUDDRV("+%s disable_clock ADC clk(%x)\n", __func__, Aud_ADC3_Clk_cntr);


	if (Aud_ADC3_Clk_cntr < 0) {
		PRINTK_AUDDRV("%s  <0 (%d)\n", __func__, Aud_ADC3_Clk_cntr);
		Aud_ADC3_Clk_cntr = 0;
	}

	mutex_unlock(&auddrv_pmic_mutex);
	/* PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d\n", Aud_ADC_Clk_cntr); */
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC_Hires_Clk_On / AudDrv_ADC_Hires_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_ADC_Hires_Clk_On(void)
{
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);

	if (Aud_ADC_HIRES_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s enable_clock ADC clk(%x)\n", __func__,
			      Aud_ADC_HIRES_Clk_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADC_HIRES].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_ADC_HIRES].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock ADC_HIRES fail", __func__);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error ADC_HIRES fail", __func__);
			AUDIO_AEE("");
			goto EXIT;
		}

		if (aud_clks[CLOCK_ADC_HIRES_TML].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_ADC_HIRES_TML].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock ADC_HIRES_TML fail", __func__);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error ADC_HIRES_TML fail", __func__);
			AUDIO_AEE("");
			goto EXIT;
		}
#else
		Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 16, 1 << 16);
		Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 17, 1 << 17);
#endif
	}
	Aud_ADC_HIRES_Clk_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_ADC_Hires_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_ADC_HIRES_Clk_cntr--;
	if (Aud_ADC_HIRES_Clk_cntr == 0) {
		PRINTK_AUDDRV("+AudDrv_ADC_Hires_Clk_Off disable_clock ADC_HIRES clk(%x)\n",
			      Aud_ADC_HIRES_Clk_cntr);
		/* Afe_Set_Reg(AUDIO_TOP_CON0, 1 << 24 , 1 << 24); */
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADC_HIRES_TML].clk_prepare)
			clk_disable(aud_clks[CLOCK_ADC_HIRES_TML].clock);

		if (aud_clks[CLOCK_ADC_HIRES].clk_prepare)
			clk_disable(aud_clks[CLOCK_ADC_HIRES].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON1, 1 << 17, 1 << 17);
		Afe_Set_Reg(AUDIO_TOP_CON1, 1 << 16, 1 << 16);
#endif
	}
	if (Aud_ADC_HIRES_Clk_cntr < 0) {
		PRINTK_AUDDRV("!! AudDrv_ADC_Hires_Clk_Off, Aud_ADC_Clk_cntr<0 (%d)\n",
			      Aud_ADC_HIRES_Clk_cntr);
		Aud_ADC_HIRES_Clk_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC2_Hires_Clk_On / AudDrv_ADC2_Hires_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_ADC2_Hires_Clk_On(void)
{
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	if (Aud_ADC2_Hires_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s enable_clock %s clk(%x)\n", __func__,
			      aud_clks[CLOCK_ADDA6_ADC_HIRES].name, Aud_ADC2_Hires_Clk_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADDA6_ADC_HIRES].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_ADDA6_ADC_HIRES].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail", __func__,
				       aud_clks[CLOCK_ADDA6_ADC_HIRES].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error %s fail", __func__,
			       aud_clks[CLOCK_ADDA6_ADC_HIRES].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#else
		Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 21, 1 << 21);
#endif
	}
	Aud_ADC2_Hires_Clk_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_ADC2_Hires_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_ADC2_Hires_Clk_cntr--;
	if (Aud_ADC2_Hires_Clk_cntr == 0) {
		PRINTK_AUDDRV("+%s disable_clock %s clk(%x)\n", __func__,
			      aud_clks[CLOCK_ADDA6_ADC_HIRES].name, Aud_ADC2_Hires_Clk_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_ADDA6_ADC_HIRES].clk_prepare)
			clk_disable(aud_clks[CLOCK_ADDA6_ADC_HIRES].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON1, 1 << 21, 1 << 21);
#endif
	}
	if (Aud_ADC2_Hires_Clk_cntr < 0) {
		PRINTK_AUDDRV("!! %s, Aud_ADC_Clk_cntr<0 (%d)\n", __func__,
			      Aud_ADC2_Hires_Clk_cntr);
		Aud_ADC2_Hires_Clk_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_APLL22M_Clk_On / AudDrv_APLL22M_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_APLL22M_Clk_On(void)
{
	int ret = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	PRINTK_AUD_CLK("+%s counter = %d\n", __func__, Aud_APLL22M_Clk_cntr);

	if (Aud_APLL22M_Clk_cntr == 0) {
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL22M].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_APLL22M].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail",
				       __func__, aud_clks[CLOCK_APLL22M].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error %s fail",
			       __func__, aud_clks[CLOCK_APLL22M].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#endif
	}
	Aud_APLL22M_Clk_cntr++;
EXIT:
	PRINTK_AUD_CLK("-%s: counter = %d\n", __func__, Aud_APLL22M_Clk_cntr);
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_APLL22M_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	PRINTK_AUD_CLK("+%s: counter = %d\n", __func__, Aud_APLL22M_Clk_cntr);

	Aud_APLL22M_Clk_cntr--;

	if (Aud_APLL22M_Clk_cntr == 0) {
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL22M].clk_prepare)
			clk_disable(aud_clks[CLOCK_APLL22M].clock);
		else {
			pr_err("%s [CCF]clk_prepare error %s fail",
			       __func__, aud_clks[CLOCK_APLL22M].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#endif
	}

EXIT:
	if (Aud_APLL22M_Clk_cntr < 0) {
		pr_warn("err, %s <0 (%d)\n", __func__,
			Aud_APLL22M_Clk_cntr);
		Aud_APLL22M_Clk_cntr = 0;
	}

	PRINTK_AUD_CLK("-%s: counter = %d\n", __func__, Aud_APLL22M_Clk_cntr);
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}


/*****************************************************************************
 * FUNCTION
  *  AudDrv_APLL24M_Clk_On / AudDrv_APLL24M_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_APLL24M_Clk_On(void)
{
	int ret = 0;
	unsigned long flags = 0;

	PRINTK_AUD_CLK("+%s counter = %d\n", __func__, Aud_APLL24M_Clk_cntr);

	spin_lock_irqsave(&auddrv_Clk_lock, flags);

	if (Aud_APLL24M_Clk_cntr == 0) {
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL24M].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_APLL24M].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail",
				       __func__, aud_clks[CLOCK_APLL24M].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error %s fail",
			       __func__, aud_clks[CLOCK_APLL24M].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#endif
	}
	Aud_APLL24M_Clk_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_APLL24M_Clk_Off(void)
{
	unsigned long flags = 0;

	PRINTK_AUD_CLK("+%s counter = %d\n", __func__, Aud_APLL24M_Clk_cntr);

	spin_lock_irqsave(&auddrv_Clk_lock, flags);

	Aud_APLL24M_Clk_cntr--;

	if (Aud_APLL24M_Clk_cntr == 0) {
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL24M].clk_prepare)
			clk_disable(aud_clks[CLOCK_APLL24M].clock);
		else {
			pr_err("%s [CCF]clk_prepare error %s fail",
			       __func__, aud_clks[CLOCK_APLL24M].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#endif
	}
EXIT:
	if (Aud_APLL24M_Clk_cntr < 0) {
		pr_warn("%s <0 (%d)\n", __func__,
			Aud_APLL24M_Clk_cntr);
		Aud_APLL24M_Clk_cntr = 0;
	}

	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_I2S_Clk_On / AudDrv_I2S_Clk_Off
  *
  * DESCRIPTION
  * Enable I2S In clock (bck)
  * This should be enabled in slave i2s mode.
  *
  *****************************************************************************/
void aud_top_con_pdn_i2s(bool _pdn)
{
	if (_pdn)
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x1 << 6, 0x1 << 6); /* power off I2S clock */
	else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x0 << 6, 0x1 << 6); /* power on I2S clock */
}

void AudDrv_I2S_Clk_On(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);

	if (Aud_I2S_Clk_cntr == 0)
		aud_top_con_pdn_i2s(false);

	Aud_I2S_Clk_cntr++;
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
EXPORT_SYMBOL(AudDrv_I2S_Clk_On);

void AudDrv_I2S_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_I2S_Clk_cntr--;
	if (Aud_I2S_Clk_cntr == 0) {
		aud_top_con_pdn_i2s(true);
	} else if (Aud_I2S_Clk_cntr < 0) {
		pr_warn("!! AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr<0 (%d)\n",
			Aud_I2S_Clk_cntr);
		AUDIO_ASSERT(true);
		Aud_I2S_Clk_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
EXPORT_SYMBOL(AudDrv_I2S_Clk_Off);

/*****************************************************************************
  * FUNCTION
  *  AudDrv_TDM_Clk_On / AudDrv_TDM_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable TDM clock
  *
  *****************************************************************************/
void aud_top_con_pdn_tdm_ck(bool _pdn)
{
	Afe_Set_Reg(AUDIO_TOP_CON0, _pdn << 20, 0x1 << 20); /* power on I2S clock */
}

void AudDrv_TDM_Clk_On(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	if (Aud_TDM_Clk_cntr == 0)
		aud_top_con_pdn_tdm_ck(false); /* enable HDMI CK */

	Aud_TDM_Clk_cntr++;
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
EXPORT_SYMBOL(AudDrv_TDM_Clk_On);

void AudDrv_TDM_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_TDM_Clk_cntr--;
	if (Aud_TDM_Clk_cntr == 0) {
		aud_top_con_pdn_tdm_ck(true); /* disable HDMI CK */
	} else if (Aud_TDM_Clk_cntr < 0) {
		pr_warn("!! %s(), Aud_TDM_Clk_cntr<0 (%d)\n",
			__func__, Aud_TDM_Clk_cntr);
		AUDIO_ASSERT(true);
		Aud_TDM_Clk_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
EXPORT_SYMBOL(AudDrv_TDM_Clk_Off);

void AudDrv_APLL1Tuner_Clk_On(void)
{
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	if (Aud_APLL1_Tuner_cntr == 0) {
		PRINTK_AUD_CLK("+%s, Aud_APLL1_Tuner_cntr:%d\n", __func__,
			       Aud_APLL1_Tuner_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL1_TUNER].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_APLL1_TUNER].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail\n",
				       __func__, aud_clks[CLOCK_APLL1_TUNER].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error %s fail\n",
			       __func__, aud_clks[CLOCK_APLL1_TUNER].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x0 << 19, 0x1 << 19);
#endif
		SetApmixedCfg(AP_PLL_CON5, 0x1, 0x1);
	}
	Aud_APLL1_Tuner_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_APLL1Tuner_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_APLL1_Tuner_cntr--;
	if (Aud_APLL1_Tuner_cntr == 0) {
		SetApmixedCfg(AP_PLL_CON5, 0x0, 0x1);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL1_TUNER].clk_prepare)
			clk_disable(aud_clks[CLOCK_APLL1_TUNER].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x1 << 19, 0x1 << 19);
#endif
	} else if (Aud_APLL1_Tuner_cntr < 0) {
		pr_warn("!! %s, Aud_APLL1_Tuner_cntr<0 (%d)\n",
			__func__, Aud_APLL1_Tuner_cntr);
		Aud_APLL1_Tuner_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}


void AudDrv_APLL2Tuner_Clk_On(void)
{
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	if (Aud_APLL2_Tuner_cntr == 0) {
		PRINTK_AUD_CLK("+%s, Aud_APLL2_Tuner_cntr:%d\n", __func__,
			       Aud_APLL2_Tuner_cntr);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL2_TUNER].clk_prepare) {
			ret = clk_enable(aud_clks[CLOCK_APLL2_TUNER].clock);
			if (ret) {
				pr_err("%s [CCF]Aud enable_clock %s fail\n",
				       __func__, aud_clks[CLOCK_APLL2_TUNER].name);
				AUDIO_AEE("");
				goto EXIT;
			}
		} else {
			pr_err("%s [CCF]clk_prepare error %s fail\n",
			       __func__, aud_clks[CLOCK_APLL2_TUNER].name);
			AUDIO_AEE("");
			goto EXIT;
		}
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x0 << 18, 0x1 << 18);
#endif
		SetApmixedCfg(AP_PLL_CON5, 0x1 << 1, 0x1 << 1);
	}
	Aud_APLL2_Tuner_cntr++;
EXIT:
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_APLL2Tuner_Clk_Off(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&auddrv_Clk_lock, flags);
	Aud_APLL2_Tuner_cntr--;
	if (Aud_APLL2_Tuner_cntr == 0) {
		SetApmixedCfg(AP_PLL_CON5, 0x0 << 1, 0x1 << 1);
#ifdef PM_MANAGER_API
		if (aud_clks[CLOCK_APLL2_TUNER].clk_prepare)
			clk_disable(aud_clks[CLOCK_APLL2_TUNER].clock);
#else
		Afe_Set_Reg(AUDIO_TOP_CON0, 0x1 << 18, 0x1 << 18);
#endif
	} else if (Aud_APLL2_Tuner_cntr < 0) {
		pr_warn("!! %s, Aud_APLL1_Tuner_cntr<0 (%d)\n",
			__func__, Aud_APLL2_Tuner_cntr);
		Aud_APLL2_Tuner_cntr = 0;
	}
	spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_HDMI_Clk_On / AudDrv_HDMI_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_HDMI_Clk_On(void)
{
#ifdef _NON_COMMON_FEATURE_READY
	PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_On, Aud_I2S_Clk_cntr:%d\n", Aud_HDMI_Clk_cntr);
	if (Aud_HDMI_Clk_cntr == 0) {
		AudDrv_ANA_Clk_On();
		AudDrv_Clk_On();
	}
	Aud_HDMI_Clk_cntr++;
#endif
}

void AudDrv_HDMI_Clk_Off(void)
{
#ifdef _NON_COMMON_FEATURE_READY
	PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_Off, Aud_I2S_Clk_cntr:%d\n",
		       Aud_HDMI_Clk_cntr);
	Aud_HDMI_Clk_cntr--;
	if (Aud_HDMI_Clk_cntr == 0) {
		AudDrv_ANA_Clk_Off();
		AudDrv_Clk_Off();
	} else if (Aud_HDMI_Clk_cntr < 0) {
		pr_warn("!! AudDrv_Linein_Clk_Off, Aud_I2S_Clk_cntr<0 (%d)\n",
			Aud_HDMI_Clk_cntr);
		AUDIO_ASSERT(true);
		Aud_HDMI_Clk_cntr = 0;
	}
	PRINTK_AUD_CLK("-AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d\n", Aud_HDMI_Clk_cntr);
#endif
}

void AudDrv_Emi_Clk_On(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	mutex_lock(&auddrv_pmic_mutex);
	if (Aud_EMI_cntr == 0) {
#ifdef _MT_SPM_RESOURCE
		/* CPU can sleep and be waked up when receiving interrupt */
		spm_resource_req(SPM_RESOURCE_USER_AUDIO,
				 SPM_RESOURCE_DRAM);
		/* SPM_RESOURCE_CPU | SPM_RESOURCE_DRAM); */
#elif defined(_MT_IDLE_HEADER)
		/* mutex is used in these api */
		disable_dpidle_by_bit(MTK_CG_AUDIO0_PDN_AFE);
		disable_soidle_by_bit(MTK_CG_AUDIO0_PDN_AFE);
		disable_mcsodi_by_bit(MTK_CG_AUDIO0_PDN_AFE);
#endif
	}
	Aud_EMI_cntr++;
	mutex_unlock(&auddrv_pmic_mutex);
#endif
}

void AudDrv_Emi_Clk_Off(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	mutex_lock(&auddrv_pmic_mutex);
	Aud_EMI_cntr--;
	if (Aud_EMI_cntr == 0) {
#ifdef _MT_SPM_RESOURCE
		spm_resource_req(SPM_RESOURCE_USER_AUDIO, 0);
#elif defined(_MT_IDLE_HEADER)
		/* mutex is used in these api */
		enable_dpidle_by_bit(MTK_CG_AUDIO0_PDN_AFE);
		enable_soidle_by_bit(MTK_CG_AUDIO0_PDN_AFE);
		enable_mcsodi_by_bit(MTK_CG_AUDIO0_PDN_AFE);
#endif
	}

	if (Aud_EMI_cntr < 0) {
		Aud_EMI_cntr = 0;
		pr_warn("Aud_EMI_cntr = %d\n", Aud_EMI_cntr);
	}
	mutex_unlock(&auddrv_pmic_mutex);
#endif
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ANC_Clk_On / AudDrv_ANC_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable ANC clock
  *
  *****************************************************************************/

void AudDrv_ANC_Clk_On(void)
{
	/* MT6759 no ANC */
}

void AudDrv_ANC_Clk_Off(void)
{
	/* MT6759 no ANC*/
}

unsigned int GetApllbySampleRate(unsigned int SampleRate)
{
	if (SampleRate == 176400 || SampleRate == 88200 || SampleRate == 44100 ||
	    SampleRate == 22050 || SampleRate == 11025)
		return Soc_Aud_APLL1;
	else
		return Soc_Aud_APLL2;
}

void SetckSel(unsigned int I2snum, unsigned int SampleRate)
{
	/* always from APLL1 */
	unsigned int ApllSource;

	if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL1)
		ApllSource = 0;
	else
		ApllSource = 1;

	switch (I2snum) {
	case Soc_Aud_I2S0:
		clksys_set_reg(CLK_AUDDIV_0, ApllSource << 8, 1 << 8);
		break;
	case Soc_Aud_I2S1:
		clksys_set_reg(CLK_AUDDIV_0, ApllSource << 9, 1 << 9);
		break;
	case Soc_Aud_I2S2:
		clksys_set_reg(CLK_AUDDIV_0, ApllSource << 10, 1 << 10);
		break;
	case Soc_Aud_I2S3:
		clksys_set_reg(CLK_AUDDIV_0, ApllSource << 11, 1 << 11);
		break;
	case Soc_Aud_I2S4:
		clksys_set_reg(CLK_AUDDIV_0, ApllSource << 12, 1 << 12);
		break;
	case Soc_Aud_I2S5:
		clksys_set_reg(CLK_AUDDIV_2, ApllSource << 20, 1 << 12);
		break;
	}
	pr_debug("%s I2snum = %d ApllSource = %d\n", __func__, I2snum, ApllSource);
}

void EnableALLbySampleRate(unsigned int SampleRate)
{
	pr_aud("%s, APLL1Counter = %d, APLL2Counter = %d, SampleRate = %d\n", __func__,
	       APLL1Counter, APLL2Counter, SampleRate);

	switch (GetApllbySampleRate(SampleRate)) {
	case Soc_Aud_APLL1:
		APLL1Counter++;
		if (APLL1Counter == 1) {
			EnableApll1(true);
			AudDrv_APLL1Tuner_Clk_On();
		}
		break;
	case Soc_Aud_APLL2:
		APLL2Counter++;
		if (APLL2Counter == 1) {
			EnableApll2(true);
			AudDrv_APLL2Tuner_Clk_On();
		}
		break;
	default:
		pr_debug("[AudioWarn] GetApllbySampleRate(%d) = %d not recognized\n",
			 SampleRate, GetApllbySampleRate(SampleRate));
		break;
	}
}

void DisableALLbySampleRate(unsigned int SampleRate)
{
	pr_aud("%s, APLL1Counter = %d, APLL2Counter = %d, SampleRate = %d\n", __func__,
	       APLL1Counter, APLL2Counter, SampleRate);

	switch (GetApllbySampleRate(SampleRate)) {
	case Soc_Aud_APLL1:
		APLL1Counter--;
		if (APLL1Counter == 0) {
			AudDrv_APLL1Tuner_Clk_Off();
			EnableApll1(false);
		} else if (APLL1Counter < 0) {
			pr_warn("%s(), APLL1Counter %d < 0\n",
				__func__,
				APLL1Counter);
			APLL1Counter = 0;
		}
		break;
	case Soc_Aud_APLL2:
		APLL2Counter--;
		if (APLL2Counter == 0) {
			AudDrv_APLL2Tuner_Clk_Off();
			EnableApll2(false);
		} else if (APLL2Counter < 0) {
			pr_warn("%s(), APLL2Counter %d < 0\n",
				__func__,
				APLL2Counter);
			APLL2Counter = 0;
		}
		break;
	default:
		pr_warn("[AudioWarn] GetApllbySampleRate(%d) = %d not recognized\n",
			SampleRate, GetApllbySampleRate(SampleRate));
		break;
	}
}

void EnableI2SDivPower(unsigned int Diveder_name, bool bEnable)
{
	pr_debug("%s bEnable = %d\n", __func__, bEnable);
	if (Diveder_name < AUDIO_APLL12_DIV5)
		clksys_set_reg(CLK_AUDDIV_0, !bEnable << Diveder_name, 1 << Diveder_name);
	else
		clksys_set_reg(CLK_AUDDIV_2, !bEnable << 16/*APLL_apll12_div5_pdn_POS*/, 1 << 16);
}

void EnableI2SCLKDiv(unsigned int I2snum, bool bEnable)
{
	pr_debug("%s mI2SAPLLDivSelect = %d, i2snum = %d\n", __func__, mI2SAPLLDivSelect[I2snum], I2snum);
	EnableI2SDivPower(mI2SAPLLDivSelect[I2snum], bEnable);
}

void EnableApll1(bool enable)
{
	pr_aud("%s enable = %d\n", __func__, enable);

	if (enable) {
		if (Aud_APLL_DIV_APLL1_cntr == 0) {
			AudDrv_APLL22M_Clk_On();
			Afe_Set_Reg(AFE_HD_ENGEN_ENABLE, 0x1 << 0, 0x1 << 0);
		}
		Aud_APLL_DIV_APLL1_cntr++;
	} else {
		Aud_APLL_DIV_APLL1_cntr--;
		if (Aud_APLL_DIV_APLL1_cntr == 0) {
			Afe_Set_Reg(AFE_HD_ENGEN_ENABLE, 0x0 << 0, 0x1 << 0);
			AudDrv_APLL22M_Clk_Off();
		}
	}
}

void EnableApll2(bool enable)
{
	pr_aud("%s enable = %d\n", __func__, enable);

	if (enable) {
		if (Aud_APLL_DIV_APLL2_cntr == 0) {
			AudDrv_APLL24M_Clk_On();
			Afe_Set_Reg(AFE_HD_ENGEN_ENABLE, 0x1 << 1, 0x1 << 1);
		}
		Aud_APLL_DIV_APLL2_cntr++;
	} else {
		Aud_APLL_DIV_APLL2_cntr--;
		if (Aud_APLL_DIV_APLL2_cntr == 0) {
			Afe_Set_Reg(AFE_HD_ENGEN_ENABLE, 0x0 << 1, 0x1 << 1);
			AudDrv_APLL24M_Clk_Off();
		}
	}
}

unsigned int SetCLkMclk(unsigned int I2snum, unsigned int SampleRate)
{
	unsigned int I2S_APll = 0;
	unsigned int I2s_ck_div = 0;

	if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL1)
		I2S_APll = APLL_44K_BASE;
	else
		I2S_APll = APLL_48K_BASE;

	SetckSel(I2snum, SampleRate);	/* set I2Sx mck source */

	switch (I2snum) {
	case Soc_Aud_I2S0:
		I2s_ck_div = (I2S_APll / MCLKFS / SampleRate) - 1;
		clksys_set_reg(CLK_AUDDIV_1, I2s_ck_div << 0, 0xff << 0);
		break;
	case Soc_Aud_I2S1:
		I2s_ck_div = (I2S_APll / MCLKFS / SampleRate) - 1;
		clksys_set_reg(CLK_AUDDIV_1, I2s_ck_div << 8, 0xff << 8);
		break;
	case Soc_Aud_I2S2:
		I2s_ck_div = (I2S_APll / MCLKFS / SampleRate) - 1;
		clksys_set_reg(CLK_AUDDIV_1, I2s_ck_div << 16, 0xff << 16);
		break;
	case Soc_Aud_I2S3:
		I2s_ck_div = (I2S_APll / MCLKFS / SampleRate) - 1;
		clksys_set_reg(CLK_AUDDIV_1, I2s_ck_div << 24, 0xff << 24);
		break;
	case Soc_Aud_I2S4:
		I2s_ck_div = (I2S_APll / MCLKFS_HDMI / SampleRate) - 1;
		clksys_set_reg(CLK_AUDDIV_2, I2s_ck_div << 0, 0xff << 0);
		break;
	case Soc_Aud_I2S5:
		I2s_ck_div = (I2S_APll / MCLKFS_HDMI / SampleRate) - 1;
		clksys_set_reg(CLK_AUDDIV_2, (I2s_ck_div & 0xf) << 28, 0xf << 0);
		clksys_set_reg(CLK_AUDDIV_3, ((I2s_ck_div >> 4) & 0xf) << 0, 0xf << 0);
		break;
	default:
		pr_warn("[AudioWarn] SetCLkMclk: I2snum = %d not recognized\n",
			I2snum);
		break;
	}

	pr_debug("%s I2snum = %d I2s_ck_div = %d I2S_APll = %d\n", __func__, I2snum, I2s_ck_div,
		 I2S_APll);

	return I2s_ck_div;
}

void SetCLkBclk(unsigned int MckDiv, unsigned int SampleRate, unsigned int Channels, unsigned int Wlength)
{
	/* BCK set only required in 6595 TDM function div4/div5 */
	unsigned int I2S_APll = 0;
	unsigned int I2S_Bclk = 0;
	unsigned int I2s_Bck_div = 0;

	pr_debug("%s MckDiv = %dv SampleRate = %d  Channels = %d Wlength = %d\n", __func__, MckDiv,
		 SampleRate, Channels, Wlength);
	MckDiv++;

	if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL1)
		I2S_APll = APLL_44K_BASE;
	else
		I2S_APll = APLL_48K_BASE;

	I2S_Bclk = SampleRate * Channels * (Wlength + 1) * 16;
	I2s_Bck_div = (I2S_APll / MckDiv) / I2S_Bclk;

	pr_debug("%s I2S_APll = %dv I2S_Bclk = %d  I2s_Bck_div = %d\n", __func__, I2S_APll,
		I2S_Bclk, I2s_Bck_div);

	if (I2s_Bck_div > 0)
		I2s_Bck_div--;

	clksys_set_reg(CLK_AUDDIV_2, I2s_Bck_div << 8, 0xff << 8);

}

void PowerDownAllI2SDiv(void)
{
	int i = 0;

	for (i = AUDIO_APLL1_DIV0; i < AUDIO_APLL_DIV_NUM; i++)
		EnableI2SDivPower(i, false);
}

#if !defined(CONFIG_FPGA_EARLY_PORTING)
/* 26M low power control */
static int mtk_idle_notify_call(struct notifier_block *nfb, unsigned long id, void *arg)
{
	switch (id) {
	case NOTIFY_DPIDLE_ENTER:
	case NOTIFY_SOIDLE_ENTER:
		/* intbus pll -> 26m */
		if (aud_clk_enable(&aud_clks[CLOCK_MUX_AUDIOINTBUS]))
			return NOTIFY_OK;

		AudDrv_AUDINTBUS_Sel(0);

		if (aud_clks[CLOCK_MUX_AUDIOINTBUS].clk_prepare)
			clk_disable(aud_clks[CLOCK_MUX_AUDIOINTBUS].clock);

		break;
	case NOTIFY_DPIDLE_LEAVE:
	case NOTIFY_SOIDLE_LEAVE:
		/* Audio Clock: 26m -> pll */
		if (aud_clk_enable(&aud_clks[CLOCK_MUX_AUDIOINTBUS]))
			return NOTIFY_OK;

		AudDrv_AUDINTBUS_Sel(1);

		if (aud_clks[CLOCK_MUX_AUDIOINTBUS].clk_prepare)
			clk_disable(aud_clks[CLOCK_MUX_AUDIOINTBUS].clock);

		break;
	case NOTIFY_SOIDLE3_ENTER:
	case NOTIFY_SOIDLE3_LEAVE:
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block mtk_idle_nfb = {
	.notifier_call = mtk_idle_notify_call,
};
#endif  /* end of  #if !defined(CONFIG_FPGA_EARLY_PORTING) */

void audio_clk_control(void)
{
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	mtk_idle_notifier_register(&mtk_idle_nfb);
#endif
}

