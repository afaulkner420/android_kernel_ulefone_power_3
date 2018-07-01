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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mt-plat/mtk_gpio.h>
#include <mach/gpio_const.h>
#include "disp_dts_gpio.h"
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/upmu_hw.h>

#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static LCM_UTIL_FUNCS lcm_util;

#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#ifndef BUILD_LK
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#endif
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
		lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#define AUXADC_LCM_VOLTAGE_CHANNEL	2
#define LCM_ID_MAX_VOLTAGE			150
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE			0
#define FRAME_WIDTH					(1080)
#define FRAME_HEIGHT 				(2160)
#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
#define LCM_BIAS_VOLTAGE			5500000  //5.5v
#endif

#ifdef BUILD_LK
#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST				(GPIO45 | 0x80000000)
#endif
#endif

#define REGFLAG_DELAY				0xFFFC
#define REGFLAG_UDELAY				0xFFFB
#define REGFLAG_END_OF_TABLE		0xFFFD
#define REGFLAG_RESET_LOW			0xFFFE
#define REGFLAG_RESET_HIGH			0xFFFF

#ifdef BUILD_LK
#define SET_RESET_PIN(v)			(mt_set_gpio_out(GPIO_LCM_RST,(v)))
#endif

extern unsigned int g_lcm_inversion;

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[120];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	//CMD2 ENABLE
	{0x00,01,{0x00}},
	{0xff,03,{0x19,0x11,0x01}},
	{0x00,01,{0x80}},
	{0xff,02,{0x19,0x11}},

	//C080H TCON RTN Setting
	{0x00,01,{0x80}},
	{0xC0,07,{0x51,0x00,0x08,0x08,0x51,0x04,0x00}},

	// C08AH Panel Scan Mode, b[2]mirror X2, b[0]mirror Y2
	{0x00,01,{0x8A}},
	{0xC0,01,{0x00}},

	//B392H Panel Mode, no swap G / 0xB392[1] = 1 , Enable SW Resolution setting
	{0x00,01,{0x92}},
	{0xB3,02,{0x18,0x06}},

	//C08BH Panel Driving Mode
	{0x00,01,{0x8B}},
	{0xC0,01,{0x88}},
	//B3B0H 1080RGBx2160
	{0x00,01,{0xB0}},
	{0xB3,04,{0x04,0x38,0x08,0x70}},
	//LTPS INITIAL CODE
	// C280H LTPS VST Setting1
	{0x00,01,{0x80}},
	{0xC2,04,{0x84,0x01,0x33,0x34}},
	//C284 VST2
	{0x00,01,{0x84}},
	{0xC2,02,{0x00,0x00}},
	// C2B0H LTPS CKV Setting1
	{0x00,01,{0xB0}},
	{0xC2,14,{0x85,0x05,0x11,0x09,0x00,0x85,0x02,0x22,0x85,0x03,0x33,0x85,0x04,0x00}},
	// C2C0H LTPS CKV Setting2
	{0x00,01,{0xC0}},
	{0xC2,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	// C2D0H LTPS CKV period
	{0x00,01,{0xD0}},
	{0xC2,05,{0x33,0x33,0x00,0x00,0xF0}},
	// C2E0H LTPS VEND Setting1
	{0x00,01,{0xE0}}, //C2E0
	{0xC2,06,{0x02,0x01,0x09,0x07,0x00,0x00}},
	// C2F0H LTPS VEND Setting1
	{0x00,01,{0xF0}}, //C2F0
	{0xC2,05,{0x80,0xFF,0x01,0x08,0x07}},
	// C290H LTPS VEND Setting1
	{0x00,01,{0x90}}, //C2F0
	{0xC2,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,01,{0xA2}},
	{0xC5,01,{0x00}},
	//CD80H map_sel
	{0x00,01,{0x80}},
	{0xCD,01,{0x01}},
	//C094H  CKH
	{0x00,01,{0x94}},
	{0xC0,07,{0x00,0x01,0x06,0x00,0x06,0x15,0x00}},
	// D800H - GVDDP GVDDN
	{0x00,01,{0x00}}, //GVDD/NGVDD=+/-5V
	{0xD8,02,{0x2B,0x2B}},
	// D900H - VCOM
	//{0x00,01,{0x00}},
	//{0xD9,02,{0xA2,0x00}},
	//E000H Gamma Separate Change
	{0x00,01,{0x00}},
	{0xE0,01,{0x00}},
	//E100H  Gamma R
	{0x00,01,{0x00}},
	{0xE1,37,{0x52,0x88,0xe7,0x2e,0x40,0x56,0x82,0xbf,0xef,0x55,0xd7,0x02,0x23,0x43,0xa9,0x64,0x85,0xf4,0x12,0x9a,0x30,0x53,0x71,0x93,0xaa,0xc2,0xca,0xfa,0x3a,0xea,0x62,0x83,0xb8,0xb5,0xff,0xdc,0x03}},
	//E300H  Gamma G
	{0x00,01,{0x00}},
	{0xE3,37,{0x52,0x88,0xe7,0x2e,0x40,0x56,0x82,0xbf,0xef,0x55,0xd7,0x02,0x23,0x43,0xa9,0x64,0x85,0xf4,0x12,0x9a,0x30,0x53,0x71,0x93,0xaa,0xc2,0xca,0xfa,0x3a,0xea,0x62,0x83,0xb8,0xb5,0xff,0xdc,0x03}},
	//E500H  Gamma B
	{0x00,01,{0x00}},
	{0xE5,37,{0x52,0x88,0xe7,0x2e,0x40,0x56,0x82,0xbf,0xef,0x55,0xd7,0x02,0x23,0x43,0xa9,0x64,0x85,0xf4,0x12,0x9a,0x30,0x53,0x71,0x93,0xaa,0xc2,0xca,0xfa,0x3a,0xea,0x62,0x83,0xb8,0xb5,0xff,0xdc,0x03}},

	//--------Down Power  Consumption-----------------
	//C590H GAP For Power Saving Setting Modify
	{0x00,01,{0x90}},
	{0xC5,01,{0x45}},
	//C591H SAP For special pattern horizontal band
	{0x00,01,{0x91}},
	{0xC5,01,{0xA0}},
	//C583H VGH=10.75V  VGH Clamp 8.8V
	{0x00,01,{0x83}},
	{0xC5,01,{0x9B}},
	//C584H VGL=-8.3V   VGL Clamp -8.3V
	{0x00,01,{0x84}},
	{0xC5,01,{0xAB}},
	//C5A0H VGHO=8.5V
	{0x00,01,{0xA0}},
	{0xC5,01,{0x99}},
	//C5A1H VGLO=-8.0V
	{0x00,01,{0xA1}},
	{0xC5,01,{0xA8}},
	//*******Initial code Fine Tune*******//
	//C390H
	{0x00,01,{0x90}},
	{0xC3,04,{0x00,0x00,0x00,0x00}},
	//C386H
	{0x00,01,{0x86}},
	{0xC3,01,{0x00}},
	//C191H timeout open
	{0x00,01,{0x91}},
	{0xC1,01,{0x0F}},
	//C480H Source v-blank output min
	{0x00,01,{0x80}},
	{0xC4,01,{0x01}},
	//C481H Chop 2line/2frame
	{0x00,01,{0x81}},
	{0xC4,01,{0x02}},
	//C5B1H Gamma Calibration control disable
	{0x00,01,{0xB1}},
	{0xC5,01,{0x08}},
	//C5B2H Gamma chop = 2line/2frame
	{0x00,01,{0xB2}},
	{0xC5,01,{0x22}},
	//C380H gnd eq
	{0x00,01,{0x80}},
	{0xC3,8,{0x00,0x00,0x00,0x22,0x22,0x00,0x22,0x22}},
	//C390H VSP_VSN EQ
	{0x00,01,{0x90}},
	{0xC3,04,{0x20,0x20,0x02,0x02}},
	//C590H sap
	{0x00,01,{0x90}},
	{0xC5,01,{0x80}},
	//C592H vdd lvdsvdd
	{0x00,01,{0x92}},
	{0xC5,01,{0x33}},
	//C181H SSC
	{0x00,01,{0x81}},
	{0xC1,03,{0xB0,0xC0,0xF0}},
	//C592H EMI improving
	{0x00,01,{0x92}},
	{0xC5,01,{0x33}},
	//C181H EMI improving
	{0x00,01,{0x81}},
	{0xC1,03,{0xB0,0xC0,0xF0}},
	//C089H
	{0x00,01,{0x89}},
	{0xC0,02,{0x10,0x14}},
	//CB90H
	{0x00,01,{0x90}},
	{0xCB,03,{0x00,0x00,0x0C}},
	//CBC0H
	{0x00,01,{0xC0}},
	{0xcb,12,{0x05,0x04,0x04,0xf4,0x00,0x00,0x04,0x00,0x04,0xf3,0x00,0x00}},
	//CBF0H
	{0x00,01,{0xF0}},
	{0xCB,04,{0xFF,0x30,0x33,0x80}},
	//F584H Vcom active region
	{0x00,01,{0x84}},
	{0xF5,01,{0x9A}},
	//Disable command2
	{0x00,01,{0x00}},
	{0xFF,03,{0xFF,0xFF,0xFF}},
	//{0x51,02,{0xD6,0x0C}}, //CABC
	//{0x53,01,{0x2C}},
	//{0x55,01,{0x01}},
	{0x35,01,{0x00}}, //TE
	
	{0x11,01,{0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x29,01,{0x00}},
	{REGFLAG_DELAY,20, {}},
};

/*static struct LCM_setting_table lcm_deep_standby_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	// Deep Standby
	{0x04, 1, {0x5A} },
	{0x05, 1, {0x5A} },
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};*/

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
		unsigned int cmd;
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE :
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type		= LCM_TYPE_DSI;

	params->width		= FRAME_WIDTH;
	params->height		= FRAME_HEIGHT;
	params->lcm_if		= LCM_INTERFACE_DSI0;
	params->lcm_cmd_if	= LCM_INTERFACE_DSI0;

	/* add for *#88* flicker test */
	params->inversion	= LCM_INVERSIONE_COLUMN;
	g_lcm_inversion		= LCM_INVERSIONE_COLUMN;

#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
	lcm_bias_vol		= LCM_BIAS_VOLTAGE;
#endif

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode	= CMD_MODE;
#else
	params->dsi.mode	= BURST_VDO_MODE;
#endif

	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;

	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability */
	/* video mode timing */
	params->dsi.PS									= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 10;
	params->dsi.vertical_frontporch					= 10;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;
	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 35;
	params->dsi.horizontal_frontporch				= 36;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	/* this value must be in MTK suggested table */
	params->dsi.PLL_CLOCK 							= 498;
	params->dsi.ssc_disable							= 0;
	params->dsi.ssc_range							= 4;
	params->dsi.HS_TRAIL							= 15;

	params->dsi.noncont_clock 						= 1;
	params->dsi.noncont_clock_period 				= 1;

	params->dsi.esd_check_enable 					= 0;
	params->dsi.customization_esd_check_enable      = 0;
	params->dsi.clk_lp_per_line_enable 				= 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 32;
#endif
}

#ifdef BUILD_LK
static void mt6370_avdd_avee_power_on(void)
{
	int ret = 0;

	/*config mt6370 register 0xB2[7:6]=0x3, that is set db_delay=4ms.*/
	ret = PMU_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));

	/* set AVDD 5.4v, (4v+28*0.05v) */
	/*ret = mt6370_write_byte(0xB3, (1 << 6) | 28);*/
	ret = PMU_REG_MASK(0xB3, 30, (0x3F << 0));//set vol is 5.5V
	if (ret < 0)
		LCM_LOGI("otm1911a----cmd=%0x--i2c write error----\n", 0xB3);
	else
		LCM_LOGI("otm1911a----cmd=%0x--i2c write success----\n", 0xB3);

	/* set AVEE */
	/*ret = mt6370_write_byte(0xB4, (1 << 6) | 28);*/
	ret = PMU_REG_MASK(0xB4, 30, (0x3F << 0));
	if (ret < 0)
		LCM_LOGI("otm1911a----cmd=%0x--i2c write error----\n", 0xB4);
	else
		LCM_LOGI("otm1911a----cmd=%0x--i2c write success----\n", 0xB4);

	/* enable AVDD & AVEE */
	/* 0x12--default value; bit3--Vneg; bit6--Vpos; */
	/*ret = mt6370_write_byte(0xB1, 0x12 | (1<<3) | (1<<6));*/
	ret = PMU_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
	if (ret < 0)
		LCM_LOGI("otm1911a----cmd=%0x--i2c write error----\n", 0xB1);
	else
		LCM_LOGI("otm1911a----cmd=%0x--i2c write success----\n", 0xB1);

	MDELAY(15);
}
#endif

static void lcm_init_power(void)
{
#ifdef BUILD_LK
	mt6370_avdd_avee_power_on();
#else
#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
	tran_display_bias_enable(LCM_BIAS_VOLTAGE); //set vol 5.5v
#else
	display_bias_enable(); //mtk default is 5.4v,so we cannot using it.
#endif
	MDELAY(15);
#endif
}

static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
	display_bias_disable();
#endif
}

static void lcm_resume_power(void)
{
#ifndef BUILD_LK
#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
	tran_display_bias_enable(LCM_BIAS_VOLTAGE); //set vol 5.5v
#else
	display_bias_enable();  //mtk default is 5.4v,so we cannot using it.
#endif
	MDELAY(15);
#endif
}

static void lcm_init(void)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_DISABLE);
	MDELAY(5);

	SET_RESET_PIN(1);
	MDELAY(6);
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_RESET_PIN(1);
#else
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(6);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(2);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
#endif
	MDELAY(11);

	/* when phone initial , config output high, enable backlight drv chip */
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
	SET_RESET_PIN(0);
	MDELAY(2);
#else
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(2);
#endif
}

static void lcm_resume(void)
{
#ifndef BUILD_LK
	MDELAY(5);

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(6);

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(2);

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(11);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}

static unsigned int lcm_compare_id(void)
{
	int lcm_vol = 0;
	int data[4] = {0, 0, 0, 0};
	int rawdata = 0;
	int res = 0;
#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
	res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawdata);
	if (res < 0) {
		LCM_LOGI("(%s) otm1911a_fhdp_dsi_vdo_djn_auo_x605 get lcm chip id vol fail\n",__func__);
		return 0;
	}
#endif
	lcm_vol = data[0] * 1000 + data[1] * 10;
	LCM_LOGI("(%s) otm1911a_fhdp_dsi_vdo_djn_auo_x605 lcm chip id adc rawdata: %d, lcm_vol: %d\n", __func__, rawdata, lcm_vol);
	if (lcm_vol < LCM_ID_MAX_VOLTAGE) {
		return 1;
	} else {
		return 0;
	}
}

LCM_DRIVER otm1911a_fhdp_dsi_vdo_djn_auo_x605_lcm_drv =
{
	.name			= "otm1911a_fhdp_dsi_vdo_djn_auo_x605",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power	    = lcm_init_power,
#ifndef BUILD_LK
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
#endif
};
