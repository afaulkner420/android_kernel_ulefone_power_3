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

#ifndef __SCP_DVFS_H__
#define __SCP_DVFS_H__

/* MT6758 DVFS doesn't need PLL because it switches between ULPOSC1 and ULPOSC2 directly  */
#define SCP_DVFS_USE_PLL	0

#define PLL_ENABLE				(1)
#define PLL_DISABLE				(0)

#define DVFS_STATUS_OK			(0)
#define DVFS_STATUS_BUSY		(-1)
#define DVFS_REQUEST_SAME_CLOCK	(-2)
#define DVFS_STATUS_ERR			(-3)
#define DVFS_STATUS_TIMEOUT		(-4)
#define DVFS_CLK_ERROR			(-5)
#define DVFS_STATUS_CMD_FIX		(-6)
#define DVFS_STATUS_CMD_LIMITED	(-7)
#define DVFS_STATUS_CMD_DISABLE	(-8)

typedef enum {
	IN_DEBUG_IDLE = 1,
	ENTERING_SLEEP = 2,
	IN_SLEEP = 4,
	ENTERING_ACTIVE = 8,
	IN_ACTIVE = 16,
} scp_state_enum;

enum {
	CLK_SYS_EN_BIT = 0,
	CLK_HIGH_EN_BIT = 1,
	CLK_HIGH_CG_BIT = 2,
	CLK_SYS_IRQ_EN_BIT = 16,
	CLK_HIGH_IRQ_EN_BIT = 17,
};
/*#ifdef CONFIG_PINCTRL_MT6797*/

typedef enum  {
	CLK_26M	 = 26,
	CLK_OPP0 = 82,
	CLK_OPP1 = 250,
	CLK_OPP2 = 328,
	CLK_INVALID_OPP,
} clk_opp_enum;

typedef enum {
	CLK_DIV_1 = 0,
	CLK_DIV_2 = 1,
	CLK_DIV_4  = 2,
	CLK_DIV_8  = 3,
	CLK_DIV_UNKNOWN,
} clk_div_enum;

typedef enum  {
	SPM_VOLTAGE_800_D = 0,
	SPM_VOLTAGE_800,
	SPM_VOLTAGE_900,
	SPM_VOLTAGE_1000,
	SPM_VOLTAGE_TYPE_NUM,
} voltage_enum;

struct mt_scp_pll_t {
	struct clk *clk_mux;          /* main clock for mfg setting */
	struct clk *clk_pll0; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll1; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll2; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll3; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll4; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll5; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll6; /* substitution clock for scp transient parent setting */
	struct clk *clk_pll7; /* substitution clock for scp transient parent setting */
};

extern int scp_pll_ctrl_set(unsigned int pll_ctrl_flag, unsigned int pll_sel);
extern short  scp_set_pmic_vcore(unsigned int cur_freq);
extern unsigned int scp_get_dvfs_opp(void);
extern uint32_t scp_get_freq(void);
extern int scp_request_freq(void);
extern void scp_pll_mux_set(unsigned int pll_ctrl_flag);
extern void wait_scp_dvfs_init_done(void);
extern int __init scp_dvfs_init(void);
extern void __exit scp_dvfs_exit(void);

/* scp dvfs variable*/
extern unsigned int scp_expected_freq;
extern unsigned int scp_current_freq;
extern spinlock_t scp_awake_spinlock;

#endif  /* __SCP_DVFS_H__ */
