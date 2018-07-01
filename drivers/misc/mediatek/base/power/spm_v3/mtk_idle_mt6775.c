/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/of.h>
#include <linux/of_address.h>

#include <mtk_idle_internal.h>
#include <ddp_pwm.h>
#include <mt-plat/mtk_secure_api.h>

#if !defined(CONFIG_FPGA_EARLY_PORTING)
#include <trace/events/mtk_idle_event.h>
#endif

#define IDLE_TAG     "Power/swap"
/* TODO: fix me */
#define idle_err(fmt, args...)	pr_info(IDLE_TAG fmt, ##args)

enum subsys_id {
	SYS_MFG_ASYNC = 0,
	SYS_MFG_TOP,
	SYS_MFG_SHADER0,
	SYS_MFG_SHADER1,
	SYS_MFG_SHADER2,
	SYS_INFRA,
	SYS_AUD,
	SYS_MJC,
	SYS_MM0,
	SYS_CAM,
	SYS_IPU,
	SYS_ISP,
	SYS_VEN,
	SYS_VDE,
	NR_SYSS__,
};

/*
 * Variable Declarations
 */
void __iomem *infrasys_base;        /* INFRA_REG, INFRA_SW_CG_x_STA */
void __iomem *perisys_base;         /* PERI_REG, PERI_SW_CG_x_STA */
void __iomem *audiosys_base_in_idle;/* AUDIOSYS_REG, AUDIO_TOP_CON_x */
void __iomem *mmsys_base;           /* MM_REG, DISP_CG_CON_x */
void __iomem *camsys_base;          /* CAMSYS_REG, CAMSYS_CG_CON */
void __iomem *imgsys_base;          /* IMGSYS_REG, IMG_CG_CON */
void __iomem *mfgsys_base;          /* MFGSYS_REG, MFG_CG_CON */
void __iomem *vdecsys_base;         /* VDECSYS_REG, VDEC_x */
void __iomem *vencsys_global_base;  /* VENC_GLOBAL_REG, VENC_CE */
void __iomem *vencsys_base;         /* VENCSYS_REG, VENCSYS_CG_CON */
void __iomem *ipusys_base;          /* IPUSYS_REG, IPU_CG_CON */
void __iomem *topcksys_base;        /* TOPCKSYS_REG, CLK_CFG */

void __iomem *sleepsys_base;        /* SPM_REG */
void __iomem *apmixed_base_in_idle; /* APMIXEDSYS */

/* Idle handler on/off */
int idle_switch[NR_TYPES] = {
	0,	/* dpidle switch */
	0,	/* soidle3 switch */
	0,	/* soidle switch */
	0,	/* mcidle switch */
	0,	/* mcsodi switch */
	0,	/* slidle switch */
	1,	/* rgidle switch */
};

unsigned int idle_condition_mask[NR_TYPES][NR_GRPS] = {
	/* dpidle_condition_mask */
	[IDLE_TYPE_DP] = {
		0x00000800, /* INFRA0: */
		0x00080000, /* INFRA1: */
		0x00000000, /* INFRA2: */
		0x0FFF001F, /* PERI0 */
		0x03FF0006, /* PERI1 */
		0x00000000, /* PERI2 */
		0x00000000, /* PERI3: Check USB by spm_resource API */
		0x00000049, /* PERI4 */
		0x00000000, /* PERI5 */
		0x00000000, /* AUDIO0 */
		0x00000000, /* AUDIO1 */
		0xFFFFFFFF, /* DISP0 */
		0x00003FFF, /* DISP1 */
		0x00000000, /* DISP2 */
		0x00001FC7, /* CAM */
		0x000003FF, /* IMAGE */
		0x00000001, /* MFG */
		0x00000001, /* VDEC0 */
		0x00000001, /* VDEC1 */
		0x00000001, /* VENC0 */
		0x00000111, /* VENC1 */
		0x00000000, /* MJC is removed */
		0x00000005, /* IPU */
	},
	/* soidle3_condition_mask */
	[IDLE_TYPE_SO3] = {
		0x02000800, /* INFRA0: */
		0x00080000, /* INFRA1: */
		0x00000000, /* INFRA2: */
		0x0FFF001F, /* PERI0 */
		0x03FF0006, /* PERI1 */
		0x00000000, /* PERI2 */
		0x00000000, /* PERI3: Check USB by spm_resource API */
		0x00000041, /* PERI4 */
		0x00000000, /* PERI5 */
		0x00000000, /* AUDIO0 */
		0x00000000, /* AUDIO1 */
		0xFFFFFFFF, /* DISP0 */
		0x00003FFF, /* DISP1 */
		0x00000000, /* DISP2 */
		0x00001FC7, /* CAM */
		0x000003FF, /* IMAGE */
		0x00000001, /* MFG */
		0x00000001, /* VDEC0 */
		0x00000001, /* VDEC1 */
		0x00000001, /* VENC0 */
		0x00000111, /* VENC1 */
		0x00000000, /* MJC is removed */
		0x00000005, /* IPU */
	},
	/* soidle_condition_mask */
	[IDLE_TYPE_SO] = {
		0x00000800, /* INFRA0: */
		0x00080000, /* INFRA1: */
		0x00000000, /* INFRA2: */
		0x0FFF001F, /* PERI0 */
		0x03FF0006, /* PERI1 */
		0x00000000, /* PERI2 */
		0x00000000, /* PERI3: Check USB by spm_resource API */
		0x00000041, /* PERI4 */
		0x00000000, /* PERI5 */
		0x00000000, /* AUDIO0 */
		0x00000000, /* AUDIO1 */
		0x810FFC00, /* DISP0 */
		0x00003D7C, /* DISP1 */
		0x00000000, /* DISP2 */
		0x00001FC7, /* CAM */
		0x000003FF, /* IMAGE */
		0x00000001, /* MFG */
		0x00000001, /* VDEC0 */
		0x00000001, /* VDEC1 */
		0x00000001, /* VENC0 */
		0x00000111, /* VENC1 */
		0x00000000, /* MJC is removed */
		0x00000005, /* IPU */
	},
	/* mcsodi_condition_mask */
	[IDLE_TYPE_MCSO] = {
		0x00400800, /* INFRA0: */
		0x00080000, /* INFRA1: */
		0x00000000, /* INFRA2: */
		0x0FFF001F, /* PERI0 */
		0x03FF0006, /* PERI1 */
		0x00000000, /* PERI2 */
		0x00000000, /* PERI3: Check USB by spm_resource API */
		0x00000041, /* PERI4 */
		0x00000000, /* PERI5 */
		0x00000000, /* AUDIO0 */
		0x00000000, /* AUDIO1 */
		0x810FFC00, /* DISP0 */
		0x00003D7C, /* DISP1 */
		0x00000000, /* DISP2 */
		0x00001FC7, /* CAM */
		0x000003FF, /* IMAGE */
		0x00000001, /* MFG */
		0x00000001, /* VDEC0 */
		0x00000001, /* VDEC1 */
		0x00000001, /* VENC0 */
		0x00000111, /* VENC1 */
		0x00000000, /* MJC is removed */
		0x00000005, /* IPU */
	},
	/* mcidle_condition_mask */
	[IDLE_TYPE_MC] = {
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0},
	/* slidle_condition_mask */
	[IDLE_TYPE_SL] = {
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0},
	/* rgidle_condition_mask */
	[IDLE_TYPE_RG] = {
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0},
};

unsigned int soidle3_pll_condition_mask[NR_PLLS] = {
	0, /* UNIVPLL, will be checked sperately */
	1, /* MSDCPLL */
	0, /* MMPLL */
};

/* TODO */
unsigned int clkmux_condition_mask[NF_CLKMUX][4] = {
	/* [0]: PDN mask, [1]: PDN val, [2]: PON mask, [3]: PON val */
	[CLKMUX_MM]             = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_DDRPHY]         = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_MEM]            = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_AXI]            = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_VDEC]           = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_DISPPWM]        = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_PWM]            = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_DPI0]           = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_CAMTG2]         = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_CAMTG]          = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_MFG]            = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_VENC]           = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_UART]           = { 0x80, 0x80, 0x81, 0x00 },
	[CLKMUX_I2C]            = { 0x80, 0x80, 0x81, 0x00 },
	[CLKMUX_CAMTG4]         = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_CAMTG3]         = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_MSDC50_0]       = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_MSDC50_0_HCLK]  = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_USB30_P0]       = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_SPI]            = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_MSDC50_3_HCLK]  = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_MSDC30_3]       = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_I3C]            = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_MSDC30_1]       = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_SCP]            = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_PMICSPI]        = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_AUD_INTBUS]     = { 0x80, 0x80, 0x87, 0x00 },
	[CLKMUX_AUDIO]          = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_AUD_2]          = { 0x80, 0x80, 0x81, 0x00 },
	[CLKMUX_AUD_1]          = { 0x80, 0x80, 0x81, 0x00 },
	[CLKMUX_DSP]            = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_ATB]            = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_CAM]            = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_DFP_MFG]        = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_AUD_ENGEN2]     = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_AUD_ENGEN1]     = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_AUDIO_H]        = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_AES_UFSFD]      = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_IMG]            = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_IPU_IF]         = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_DXCC]           = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_BSI_SPI]        = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_UFS_CARD]       = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_SSPM]           = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_RSV_0]          = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_RSV_1]          = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_DFP]            = { 0x80, 0x80, 0x81, 0x00 },
	[CLKMUX_SENIF]          = { 0x80, 0x80, 0x83, 0x00 },
	[CLKMUX_RSV_2]          = { 0x00, 0x00, 0x00, 0x00 },
	[CLKMUX_DSP3]           = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_DSP2]           = { 0x80, 0x80, 0x8F, 0x00 },
	[CLKMUX_DSP1]           = { 0x80, 0x80, 0x8F, 0x00 },
};

static const char *idle_name[NR_TYPES] = {
	"dpidle",
	"soidle3",
	"soidle",
	"mcidle",
	"mcsodi",
	"slidle",
	"rgidle",
};

static const char *reason_name[NR_REASONS] = {
	"by_cpu",
	"by_clk",
	"by_tmr",
	"by_oth",
	"by_vtg",
	"by_frm",
	"by_pll",
	"by_pwm",
	"by_dcs",
	"by_ufs",
	"by_gpu",
	"by_sspm_ipi"
};

static const char *cg_group_name[NR_GRPS] = {
	"INFRA0",
	"INFRA1",
	"INFRA2",
	"PERI0",
	"PERI1",
	"PERI2",
	"PERI3",
	"PERI4",
	"PERI5",
	"AUDIO0",
	"AUDIO1",
	"DISP0",
	"DISP1",
	"DISP2",
	"CAM",
	"IMAGE",
	"MFG",
	"VDEC0",
	"VDEC1",
	"VENC0",
	"VENC1",
	"MJC",
	"IPU",
};

/*
 * Weak functions
 */
bool __attribute__((weak)) disp_pwm_is_osc(void)
{
	return false;
}

/*
 * Function Definitions
 */
const char *mtk_get_idle_name(int id)
{
	WARN_ON(INVALID_IDLE_ID(id));
	return idle_name[id];
}

const char *mtk_get_reason_name(int id)
{
	WARN_ON(INVALID_REASON_ID(id));
	return reason_name[id];
}

const char *mtk_get_cg_group_name(int id)
{
	WARN_ON(INVALID_GRP_ID(id));
	return cg_group_name[id];
}

static int sys_is_on(enum subsys_id id)
{
	u32 pwr_sta_mask[] = {
		SC_MFG_ASYNC_PWR_ACK,
		SC_MFG_TOP_PWR_ACK,
		SC_MFG_SHADER0_PWR_ACK,
		SC_MFG_SHADER1_PWR_ACK,
		SC_MFG_SHADER2_PWR_ACK,
		SC_INFRA_PWR_ACK,
		SC_AUD_PWR_ACK,
		SC_MJC_PWR_ACK,
		SC_MM0_PWR_ACK,
		SC_CAM_PWR_ACK,
		SC_IPU_PWR_ACK,
		SC_ISP_PWR_ACK,
		SC_VEN_PWR_ACK,
		SC_VDE_PWR_ACK,
	};

#if 0
	if (id >= NR_SYSS__)
		/* BUG(); */
#endif

	u32 mask = pwr_sta_mask[id];
	u32 sta = idle_readl(SPM_PWR_STATUS);
	u32 sta_s = idle_readl(SPM_PWR_STATUS_2ND);

	return (sta & mask) && (sta_s & mask);
}

static void get_all_clock_state(u32 clks[NR_GRPS])
{
	int i;

	for (i = 0; i < NR_GRPS; i++)
		clks[i] = 0;

	clks[CG_INFRA_0] = ~idle_readl(INFRA_SW_CG_0_STA);      /* INFRA0 */
	clks[CG_INFRA_1] = ~idle_readl(INFRA_SW_CG_1_STA);      /* INFRA1 */
	/* no idle condition relative to CG_INFRA_2 */
	/* clks[CG_INFRA_2] = ~idle_readl(INFRA_SW_CG_2_STA); *//* INFRA2 */

	clks[CG_PERI_0] = ~idle_readl(PERI_SW_CG_0_STA);        /* PERI0 */
	clks[CG_PERI_1] = ~idle_readl(PERI_SW_CG_1_STA);        /* PERI1 */
	/* no idle condition relative to PERI2,3,5 */
	/* clks[CG_PERI_2] = ~idle_readl(PERI_SW_CG_2_STA); */  /* PERI2 */
	/* clks[CG_PERI_3] = ~idle_readl(PERI_SW_CG_3_STA); */  /* PERI3 */
	clks[CG_PERI_4] = ~idle_readl(PERI_SW_CG_4_STA);        /* PERI4 */
	/* clks[CG_PERI_5] = ~idle_readl(PERI_SW_CG_5_STA); */  /* PERI5 */

	#if 0 /* no idle condition relative to AUDIO */
	if (sys_is_on(SYS_AUD) && (clks[CG_INFRA_0] & 0x02000000)) {
		clks[CG_AUDIO_0] = ~idle_readl(AUDIO_TOP_CON_0);    /* AUDIO */
		clks[CG_AUDIO_1] = ~idle_readl(AUDIO_TOP_CON_1);    /* AUDIO */
	}
	#endif

	if (sys_is_on(SYS_MM0)) {
		clks[CG_DISP_0] = ~idle_readl(DISP_CG_CON_0);       /* DISP0 */
		clks[CG_DISP_1] = ~idle_readl(DISP_CG_CON_1);       /* DISP1 */
		/* no idle condition relative to DISP2 */
		/* clks[CG_DISP_2] = ~idle_readl(DISP_CG_CON_2); */
	}

	if (sys_is_on(SYS_CAM))
		clks[CG_CAM] = ~idle_readl(CAMSYS_CG_CON);          /* CAM */

	if (sys_is_on(SYS_ISP))
		clks[CG_IMAGE] = ~idle_readl(IMG_CG_CON);           /* IMAGE */

	if (sys_is_on(SYS_MFG_TOP))
		clks[CG_MFG] = ~idle_readl(MFG_CG_CON);             /* MFG */

	if (sys_is_on(SYS_VDE)) {
		clks[CG_VDEC_0] = idle_readl(VDEC_CKEN_SET);        /* VDEC0 */
		clks[CG_VDEC_1] = idle_readl(VDEC_LARB1_CKEN_SET);  /* VDEC1 */
	}

	if (sys_is_on(SYS_VEN)) {
		clks[CG_VENC_0] = idle_readl(VENC_CE);              /* VENC0 */
		clks[CG_VENC_1] = idle_readl(VENCSYS_CG_CON);       /* VENC1 */
	}

	if (sys_is_on(SYS_IPU))
		clks[CG_IPU] = ~idle_readl(IPU_CG_CON);             /* IPU */
}

static int cgmon_sel = -1;
static unsigned int cgmon_sta[NR_GRPS];
static DEFINE_SPINLOCK(cgmon_spin_lock);

void mtk_idle_cg_monitor(int sel)
{
	unsigned long flags;

	spin_lock_irqsave(&cgmon_spin_lock, flags);
	cgmon_sel = sel;
	memset(cgmon_sta, 0, sizeof(cgmon_sta));
	spin_unlock_irqrestore(&cgmon_spin_lock, flags);
}

static void mtk_idle_cgmon_trace_log(
	unsigned int block_mask[NR_TYPES][NF_CG_STA_RECORD])
{
	unsigned int diff, g, n;

	if (cgmon_sel == IDLE_TYPE_DP ||
		cgmon_sel == IDLE_TYPE_SO3 ||
		cgmon_sel == IDLE_TYPE_SO) {

		for (g = 0; g < NR_GRPS; g++) {
			diff = cgmon_sta[g] ^ block_mask[cgmon_sel][g];
			if (diff) {
				cgmon_sta[g] = block_mask[cgmon_sel][g];
				#if !defined(CONFIG_FPGA_EARLY_PORTING)
				for (n = 0; n < 32; n++)
					if (diff & (1U << n))
						trace_idle_cg(g * 32 + n,
							((1 << n) & cgmon_sta[g]) ? 1 : 0);
				#endif
			}
		}
	}
}

bool mtk_idle_check_secure_cg(
	unsigned int block_mask[NR_TYPES][NF_CG_STA_RECORD])
{
	int i;
	int ret = 0;

	ret = mt_secure_call(MTK_SIP_KERNEL_CHECK_SECURE_CG, 0, 0, 0);
	if (ret)
		for (i = 0; i < NR_TYPES; i++)
			if (idle_switch[i])
				block_mask[i][CG_PERI_4] |= 0x00060000;

	return !ret;
}

bool mtk_idle_check_cg(
	unsigned int block_mask[NR_TYPES][NF_CG_STA_RECORD])
{
	bool ret = true;
	int i, j;
	unsigned int sta;
	u32 clks[NR_GRPS];

	get_all_clock_state(clks);

	sta = idle_readl(SPM_PWR_STATUS);

	for (i = 0; i < NR_TYPES; i++) {
		if (idle_switch[i]) {
			/* CG status */
			for (j = 0; j < NR_GRPS; j++) {
				block_mask[i][j] = idle_condition_mask[i][j] & clks[j];
				if (block_mask[i][j])
					block_mask[i][NR_GRPS] |= 0x2;
			}

			/* mtcmos */
			if (i == IDLE_TYPE_DP && !dpidle_by_pass_pg) {
				unsigned int flag =
					SC_MFG_TOP_PWR_ACK |
					SC_ISP_PWR_ACK |
					SC_VDE_PWR_ACK |
					SC_VEN_PWR_ACK |
					SC_MM0_PWR_ACK;

				if (sta & flag) {
					block_mask[i][NR_GRPS + 0] |= 0x4;
					block_mask[i][NR_GRPS + 1] = (sta & flag);
				}
			}
			if ((i == IDLE_TYPE_SO || i == IDLE_TYPE_SO3) &&
				!soidle_by_pass_pg) {
				unsigned int flag =
					SC_MFG_TOP_PWR_ACK |
					SC_ISP_PWR_ACK |
					SC_VDE_PWR_ACK |
					SC_VEN_PWR_ACK;

				if (sta & flag) {
					block_mask[i][NR_GRPS + 0] |= 0x4;
					block_mask[i][NR_GRPS + 1] = (sta & flag);
				}
			}
			if (i == IDLE_TYPE_MCSO && !mcsodi_by_pass_pg) {
				unsigned int flag =
					SC_MFG_TOP_PWR_ACK |
					SC_ISP_PWR_ACK |
					SC_VDE_PWR_ACK |
					SC_VEN_PWR_ACK;

				if (sta & flag) {
					block_mask[i][NR_GRPS + 0] |= 0x4;
					block_mask[i][NR_GRPS + 1] = (sta & flag);
				}
			}
			if (block_mask[i][NR_GRPS])
				ret = false;
		}
	}

	/* cg monitor: print cg change info to ftrace log */
	mtk_idle_cgmon_trace_log(block_mask);

	return ret;
}

static const char *pll_name[NR_PLLS] = {
	"UNIVPLL",
	"MSDCPLL",
	"MMPLL",
};

const char *mtk_get_pll_group_name(int id)
{
	return pll_name[id];
}

static int is_pll_on(int id)
{
	return idle_readl(APMIXEDSYS(0x240 + id * 0x10)) & 0x1;
}

bool mtk_idle_check_pll(
	unsigned int *condition_mask, unsigned int *block_mask)
{
	int i, j;

	memset(block_mask, 0, NR_PLLS * sizeof(unsigned int));

	for (i = 0; i < NR_PLLS; i++) {
		if (is_pll_on(i) & condition_mask[i]) {
			for (j = 0; j < NR_PLLS; j++)
				block_mask[j] = is_pll_on(j) & condition_mask[j];
			return false;
		}
	}

	return true;
}

static int __init get_base_from_matching_node(
	const struct of_device_id *ids, void __iomem **pbase, int idx,
	const char *cmp)
{
	struct device_node *node;

	node = of_find_matching_node(NULL, ids);
	if (!node) {
		idle_err("node '%s' not found!\n", cmp);
		/* TODO: BUG() */
	}

	*pbase = of_iomap(node, idx);
	if (!(*pbase)) {
		idle_err("node '%s' cannot iomap!\n", cmp);
		/* TODO: BUG() */
	}

	return 0;
}

static int __init get_base_from_node(
	const char *cmp, void __iomem **pbase, int idx)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, cmp);

	if (!node) {
		idle_err("node '%s' not found!\n", cmp);
		/* TODO: BUG() */
	}

	*pbase = of_iomap(node, idx);
	if (!(*pbase)) {
		idle_err("node '%s' cannot iomap!\n", cmp);
		/* TODO: BUG() */
	}

	return 0;
}

void __init iomap_init(void)
{
	static const struct of_device_id audiosys_ids[] = {
		{.compatible = "mediatek,audio"},
		{.compatible = "mediatek,mt6755-audiosys"},
		{ /* sentinel */ }
	};

	get_base_from_node("mediatek,infracfg_ao", &infrasys_base, 0);
	get_base_from_node("mediatek,pericfg", &perisys_base, 0);
	get_base_from_matching_node(audiosys_ids, &audiosys_base_in_idle, 0, "audio");
	get_base_from_node("mediatek,mmsys_config", &mmsys_base, 0);
	get_base_from_node("mediatek,camsys", &camsys_base, 0);
	get_base_from_node("mediatek,imgsys", &imgsys_base, 0);
	get_base_from_node("mediatek,g3d_config", &mfgsys_base, 0);
	get_base_from_node("mediatek,vdec_top_global_con", &vdecsys_base, 0);
	get_base_from_node("mediatek,venc", &vencsys_global_base, 0);
	get_base_from_node("mediatek,venc_global_con", &vencsys_base, 0);
	get_base_from_node("mediatek,ipu_conn", &ipusys_base, 0);
	get_base_from_node("mediatek,topckgen", &topcksys_base, 0);

	get_base_from_node("mediatek,sleep", &sleepsys_base, 0);
	get_base_from_node("mediatek,apmixedsys", &apmixed_base_in_idle, 0);
}

bool mtk_idle_disp_is_pwm_rosc(void)
{
	return disp_pwm_is_osc();
}

static void get_all_clkcfg_state(u32 clkcfgs[NF_CLK_CFG])
{
	int i;

	for (i = 0; i < NF_CLK_CFG; i++)
		clkcfgs[i] = idle_readl(CLK_CFG(i));
}

bool mtk_idle_check_clkmux(
	int idle_type, unsigned int block_mask[NR_TYPES][NF_CLK_CFG])
{
	u32 clkcfgs[NF_CLK_CFG] = {0};
	int i;
	int idx;
	int offset;
	u32 masks[4]  = {0xFF000000, 0x00FF0000, 0x0000FF00, 0x000000FF};
	int shifts[4] = {24, 16, 8, 0};
	u32 clkmux_val;

	bool result[2] = {false, false};
	bool final_result = true;

	get_all_clkcfg_state(clkcfgs);

	/* Check each clkmux setting */
	for (i = 0; i < NF_CLKMUX; i++) {
		idx        = i / 4;
		offset     = i % 4;
		clkmux_val = ((clkcfgs[idx] & masks[offset]) >> shifts[offset]);

		/* clkmux power down: PASS */
		result[0] = ((clkmux_val & clkmux_condition_mask[i][PDN_MASK])
			== clkmux_condition_mask[i][PDN_VALUE]);

		/* clkmux power up: check mux switch */
		result[1] = ((clkmux_val & clkmux_condition_mask[i][PUP_MASK])
			== clkmux_condition_mask[i][PUP_VALUE]);

		if (result[0] == false && result[1] == false) {
			final_result = false;

			block_mask[idle_type][idx] |= (clkmux_val << shifts[offset]);
		}
	}

	return final_result;
}
