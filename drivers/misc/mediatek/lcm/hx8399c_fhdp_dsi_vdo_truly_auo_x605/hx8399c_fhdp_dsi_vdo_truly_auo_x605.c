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
#define LCM_ID_MAX_VOLTAGE			1650
#define LCM_ID_MIN_VOLTAGE			1350
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
	
	{0xB9, 03, {0xFF,0x83,0x99}},
	{0xD2, 01, {0x88}},
	{0xB1, 11, {0x02,0x04,0x72,0x92,0x01,0x32,0xAA,0x11,0x11,0x52,0x57}},
	{0xB2, 15, {0x00,0x80,0x80,0xCC,0x05,0x07,0x5A,0x11,0x10,0x10,0x00,0x1E,0x70,0x03,0xD4}},
	{0xB4, 44, {0x00,0xFF,0x59,0x59,0x01,0xAB,0x00,0x00,0x09,0x00,0x03,0x05,0x00,0x28,0x02,0x0B,0x0D,0x21,0x03,0x02,0x00,0x0C,0xA2,0x82,0x59,0x59,0x02,0xAB,0x00,0x00,0x09,0x00,0x03,0x05,0x00,0x28,0x02,0x0B,0x0D,0x02,0x00,0x0C,0xA2,0x01}},
	{0xD3, 33, {0x00,0x0C,0x03,0x03,0x00,0x00,0x10,0x10,0x00,0x00,0x03,0x00,0x03,0x00,0x08,0x78,0x08,0x78,0x00,0x00,0x00,0x00,0x00,0x24,0x02,0x05,0x05,0x03,0x00,0x00,0x00,0x05,0x40}},
	{0xD5, 32, {0x20,0x20,0x19,0x19,0x18,0x18,0x02,0x03,0x00,0x01,0x24,0x24,0x18,0x18,0x18,0x18,0x24,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2F,0x2F,0x30,0x30,0x31,0x31}},
	{0xD6, 32, {0x24,0x24,0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x24,0x24,0x18,0x18,0x18,0x18,0x20,0x20,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x2F,0x2F,0x30,0x30,0x31,0x31}},
	{0xBD, 01, {0x00}},
	{0xD8, 16, {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xBA,0xAA,0xAA,0xAA,0xBA,0xAA,0xAA}},
	{0xBD, 01, {0x01}},
	{0xD8, 16, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x82,0xEA,0xAA,0xAA,0x82,0xEA,0xAA,0xAA}},
	{0xBD, 01, {0x02}},
	{0xD8, 8, {0xFF,0xFF,0xC0,0x3F,0xFF,0xFF,0xC0,0x3F}},
	{0xBD, 01, {0x00}},
	{0xE0, 54, {0x08,0x27,0x35,0x31,0x6A,0x71,0x7C,0x74,0x79,0x80,0x85,0x88,0x8B,0x8F,0x95,0x97,0x9A,0xA2,0xA4,0xAD,0xA3,0xB3,0xB8,0x60,0x5D,0x69,0x74,0x08,0x27,0x35,0x31,0x6A,0x71,0x7C,0x74,0x79,0x80,0x85,0x88,0x8B,0x8F,0x95,0x97,0x9A,0xA2,0xA4,0xAD,0xA3,0xB3,0xB8,0x60,0x5D,0x69,0x74}},
	{0xCC, 01, {0x08}},
	
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
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 3;
	params->dsi.vertical_frontporch					= 9;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;
	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 42;
	params->dsi.horizontal_frontporch				= 20;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	/* this value must be in MTK suggested table */
	params->dsi.PLL_CLOCK 							= 490;
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
		LCM_LOGI("hx8399c----cmd=%0x--i2c write error----\n", 0xB3);
	else
		LCM_LOGI("hx8399c----cmd=%0x--i2c write success----\n", 0xB3);

	/* set AVEE */
	/*ret = mt6370_write_byte(0xB4, (1 << 6) | 28);*/
	ret = PMU_REG_MASK(0xB4, 30, (0x3F << 0));
	if (ret < 0)
		LCM_LOGI("hx8399c----cmd=%0x--i2c write error----\n", 0xB4);
	else
		LCM_LOGI("hx8399c----cmd=%0x--i2c write success----\n", 0xB4);

	/* enable AVDD & AVEE */
	/* 0x12--default value; bit3--Vneg; bit6--Vpos; */
	/*ret = mt6370_write_byte(0xB1, 0x12 | (1<<3) | (1<<6));*/
	ret = PMU_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
	if (ret < 0)
		LCM_LOGI("hx8399c----cmd=%0x--i2c write error----\n", 0xB1);
	else
		LCM_LOGI("hx8399c----cmd=%0x--i2c write success----\n", 0xB1);

	MDELAY(6);
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
	MDELAY(6);
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
	MDELAY(6);
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
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
#else
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(6);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(2);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
#endif
	MDELAY(50);

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
	MDELAY(1);

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(1);

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(50);

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
		LCM_LOGI("(%s) hx8399c_fhdp_dsi_vdo_truly_auo_x605 get lcm chip id vol fail\n",__func__);
		return 0;
	}
#endif
	lcm_vol = data[0] * 1000 + data[1] * 10;
	LCM_LOGI("(%s) hx8399c_fhdp_dsi_vdo_truly_auo_x605 lcm chip id adc rawdata: %d, lcm_vol: %d\n", __func__, rawdata, lcm_vol);
	if (lcm_vol <= LCM_ID_MAX_VOLTAGE && lcm_vol >= LCM_ID_MIN_VOLTAGE) {
		return 1;
	} else {
		return 0;
	}
}

LCM_DRIVER hx8399c_fhdp_dsi_vdo_truly_auo_x605_lcm_drv =
{
	.name			= "hx8399c_fhdp_dsi_vdo_truly_auo_x605",
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
