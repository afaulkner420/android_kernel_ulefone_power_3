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

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2160)
#define LCM_ID_NT36672_ID0 (0x00)
#define LCM_ID_NT36672_ID1 (0x66)
#define LCM_ID_NT36672_ID2 (0x00)
/* physical size in um */

#define REGFLAG_DELAY             							0xFFE
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static void lcm_init_power(void)
{
    display_bias_enable();
    MDELAY(20);
}

static void lcm_suspend_power(void)
{
    display_bias_disable();
}

static void lcm_resume_power(void)
{
    SET_RESET_PIN(0);
    display_bias_enable();
    MDELAY(20);
}
static struct LCM_setting_table lcm_initialization_setting[] =
{	
 {0xFF,1,{0x20}},
 {0xFB,1,{0x01}},
 {0x06,1,{0x9E}},
 {0x07,1,{0x94}},
 {0x0E,1,{0x35}},
 {0x0F,1,{0x24}},
 {0x68,1,{0x03}}, //digital DE
 {0x6D,1,{0x66}},
 {0x69,1,{0x99}},
 {0x89,1,{0x17}},
 {0x95,1,{0xF5}},
 {0x96,1,{0xF5}},
         
         
 {0xFF,1,{0x24}},
 {0xFB,1,{0x01}},
 {0x00,1,{0x20}},
 {0x01,1,{0x20}},
 {0x02,1,{0x20}},
 {0x03,1,{0x01}},
 {0x04,1,{0x0B}},
 {0x05,1,{0x0C}},
 {0x06,1,{0xA9}},
 {0x07,1,{0x06}},
 {0x08,1,{0x04}},
 {0x09,1,{0x20}},
 {0x0A,1,{0x0F}},
 {0x0B,1,{0x20}},
 {0x0C,1,{0x20}},
 {0x0D,1,{0x20}},
 {0x0E,1,{0x20}},
 {0x0F,1,{0x17}},
 {0x10,1,{0x15}},
 {0x11,1,{0x13}},
 {0x12,1,{0x00}},
 {0x13,1,{0x00}},
 {0x14,1,{0x20}},
 {0x15,1,{0x01}},
 {0x16,1,{0x0B}},
 {0x17,1,{0x0C}},
 {0x18,1,{0xA9}},
 {0x19,1,{0x05}},
 {0x1A,1,{0x03}},
 {0x1B,1,{0x20}},
 {0x1C,1,{0x0F}},
 {0x1D,1,{0x20}},
 {0x1E,1,{0x20}},
 {0x1F,1,{0x20}},
 {0x20,1,{0x20}},
 {0x21,1,{0x17}},
 {0x22,1,{0x15}},
 {0x23,1,{0x13}},
 {0x2F,1,{0x04}},
 {0x30,1,{0x08}},
 {0x31,1,{0x04}},
 {0x32,1,{0x08}},
 {0x33,1,{0x02}},
 {0x34,1,{0x02}},
 {0x35,1,{0x00}},
 {0x37,1,{0x02}},
 {0x38,1,{0x72}},
 {0x39,1,{0x72}},
 {0x3B,1,{0x40}},
 {0x3F,1,{0x72}},
 {0x60,1,{0x10}},
 {0x61,1,{0x00}},
 {0x68,1,{0x83}},
 {0x78,1,{0x00}},
 {0x79,1,{0x00}},
 {0x7A,1,{0x08}},
 {0x7B,1,{0x9C}},
 {0x7D,1,{0x06}},
 {0x7E,1,{0x02}},
 {0x80,1,{0x45}},
 {0x81,1,{0x06}},
 {0x8E,1,{0xF0}},
 {0x90,1,{0x00}},
 {0x92,1,{0x76}},
 {0x93,1,{0x0A}},
 {0x94,1,{0x0A}},
 {0x99,1,{0x33}},
 {0x9B,1,{0xFF}},
 {0xB3,1,{0x00}},
 {0xB4,1,{0x04}},
 {0xB5,1,{0x04}},
 {0xDC,1,{0x00}},
 {0xDD,1,{0x01}},
 {0xDE,1,{0x00}},
 {0xDF,1,{0x00}},
 {0xE0,1,{0x75}},
 {0xE9,1,{0x08}},
 {0xED,1,{0x40}},
         
         
 {0xFF,1,{0x25}},
 {0xFB,1,{0x01}},
 {0x0A,1,{0x81}},
 {0x0B,1,{0xD7}},
 {0x0C,1,{0x01}},
 {0x17,1,{0x82}},
 {0x21,1,{0x1C}},
 {0x22,1,{0x1C}},
 {0x24,1,{0x76}},
 {0x25,1,{0x76}},
 {0x5C,1,{0x25}},
 {0x5D,1,{0x80}},
 {0x5E,1,{0x80}},
 {0x5F,1,{0x22}},
 {0x65,1,{0x00}},
 {0x69,1,{0x60}},
 {0x6B,1,{0x00}},
 {0x71,1,{0x2D}},
 {0x80,1,{0x00}},
 {0x8D,1,{0x04}},
 {0xD7,1,{0x00}},
 {0xD8,1,{0x00}},
 {0xD9,1,{0x00}},
 {0xDA,1,{0x00}},
 {0xDB,1,{0x00}},
 {0xDC,1,{0x00}},
         
 {0xFF,1,{0x26}},
 {0xFB,1,{0x01}},
 {0x06,1,{0xC8}},
 {0x12,1,{0x5A}},
 {0x19,1,{0x0A}},
 {0x1A,1,{0x97}},
 {0x1D,1,{0x0A}},
 {0x1E,1,{0x1E}},
 {0x99,1,{0x20}},
 {0xFF,1,{0x27}},
 {0xFB,1,{0x01}},
 {0x13,1,{0x0E}},
 {0x16,1,{0xB0}},
 {0x17,1,{0xD0}},
         
         
 {0xFF,1,{0x10}},
 {0xFB,1,{0x01}},
 {0x51,1,{0xFF}},
 {0x53,1,{0x24}},
 {0x55,1,{0x00}},
        


{0x11,1,{0x00}},

{REGFLAG_DELAY, 120, {}},

{0x29, 0,{0x00}},
{REGFLAG_DELAY, 40, {}},//40
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] =
{
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                //MDELAY(2);
        }
    }
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
    params->physical_width = 63;
    params->physical_height = 110;
    // enable tearing-free
    //params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

  params->dsi.mode = SYNC_EVENT_VDO_MODE;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 8;/*20*/
	params->dsi.vertical_frontporch = 10;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->physical_width = 68;
    params->physical_height = 136;
  	params->dsi.PLL_CLOCK = 490;
 // 	params->dsi.ssc_disable = 1;
 //	params->dsi.cont_clock=1;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}
//static unsigned int lcm_compare_id(void);
static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
	
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	/*LCM_LOGD("lcm_suspend\n");*/

    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    //SET_RESET_PIN(0);
    MDELAY(50);
	/* SET_RESET_PIN(0); */
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static void lcm_resume(void)
{
	/*LCM_LOGD("lcm_resume\n");*/

	lcm_init();
//    lcm_compare_id();
}
static int adc_read_vol(void)
{
  int adc[1];
  int data[4] ={0,0,0,0};
  int sum = 0;
  int adc_vol=0;
  int num = 0;

  for(num=0;num<10;num++)
  {
    IMM_GetOneChannelValue(2, data, NULL);
    sum+=(data[0]*100+data[1]);
  }
  adc_vol = sum/10;

#if defined(BUILD_LK)
  printf("wujie  adc_vol is %d\n",adc_vol);
#else
  printk("wujie  adc_vol is %d\n",adc_vol);
#endif

  return (adc_vol>60) ? 0 : 1;
}
#if 1
static unsigned int lcm_compare_id(void)
{
	unsigned int id0 = 0, id1 = 0, id2 = 0;
	unsigned char buffer[2];
	unsigned int array[16];

	SET_RESET_PIN(1);
    MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(120);

    array[0] = 0x00023902;
    array[1] = 0x000020ff;
    dsi_set_cmdq(array, 2, 1);
	MDELAY(10);
	array[0] = 0x00013700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);
    MDELAY(10);
	read_reg_v2(0x3B, buffer, 1);
	id1 =(buffer[0] + adc_read_vol());
#ifdef BUILD_LK
#else
	printk("%s,NT36672_ID1=0x%08x\n", __func__, id1);
#endif
	return (id1 == 0x66) ?1:0;
}


#endif


LCM_DRIVER nt36672_hxgd60_ykl_lfhd_lcm_drv = 
{
	.name = "nt36672_hxgd60_ykl_lfhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};
