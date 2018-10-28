/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 *****************************************************************************/

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
#else
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH     (1080)
#define FRAME_HEIGHT    (2160)
#define LCM_ID			(0x023c)

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

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0xFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table
{
    unsigned cmd;
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
    {0xB0,1,{0x00}},
    {0xB3,3,{0x31,0x00,0x06}},
    {0xB4,1,{0x00}},
    {0xB6,5,{0x33,0x5B,0x81,0x12,0x00}},
    {0xB8,7,{0x57,0x3D,0x19,0x1E,0x0A,0x50,0x50}},
    {0xB9,7,{0x6F,0x3D,0x28,0x3C,0x14,0xC8,0xC8}},
    {0xBA,7,{0xB5,0x33,0x41,0x64,0x23,0xA0,0xA0}},
    {0xBB,2,{0x14,0x14}},
    {0xBC,2,{0x37,0x32}},
    {0xBD,2,{0x64,0x32}},
    {0xBE,1,{0x04}},
    {0xC0,1,{0x00}},
    {0xC1,48,{0x04,0x40,0x00,0xFF,0xFF,0xC0,0x9A,0x2B,0xE3,0xBF,
                 0xFF,0xFF,0x97,0x11,0xC2,0x28,0xFF,0xFF,0xFF,0x7D,
                 0x41,0x5C,0x63,0xE0,0xFF,0x0F,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x40,0x02,0x62,0x11,0x06,0x02,
                 0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x00}},
    {0xC2,24,{0x01,0xF8,0x70,0x08,0x64,0x04,0x0C,0x10,0x00,0x08,
                 0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,
                 0x00,0x00,0x00,0x00}},
    {0xC3,62,{0x86,0x58,0x65,0x86,0x50,0x00,0x00,0x00,0x00,0x00,
                 0x41,0xB4,0x1B,0x00,0x00,0x46,0x14,0x61,0x00,0x00,
                 0x01,0x01,0x03,0x28,0x00,0x01,0x00,0x01,0x00,0x00,
                 0x15,0x00,0x15,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x00,
                 0x28,0x00,0x5A,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x08,0x08}},
    {0xC4,20,{0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
                 0x02,0x31,0x01,0x00,0x00,0x00,0x02,0x01,0x01,0x01}},
    {0xC5,8,{0x08,0x00,0x00,0x00,0x00,0x70,0x00,0x00}},
    {0xC6,63,{0x44,0x00,0x44,0x03,0x3D,0x01,0x0E,0x01,0x02,0x01,
                 0x02,0x01,0x02,0x00,0x00,0x00,0x00,0x07,0x0E,0x04,
                 0x44,0x00,0x47,0x54,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00}},
    {0xC7,38,{0x0A,0x1D,0x2A,0x3C,0x4B,0x57,0x70,0x80,0x8D,0x99,
                 0x4C,0x58,0x67,0x7B,0x84,0x8F,0x9D,0xA8,0xB4,0x0A,
                 0x1C,0x29,0x3B,0x4A,0x56,0x6F,0x80,0x8D,0x99,0x4C,
                 0x58,0x67,0x7B,0x84,0x8F,0x9D,0xA8,0xB4}},
    {0xC8,55,{0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,
                 0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,
                 0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,
                 0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,
                 0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,
                 0x00,0x00,0x00,0xFC,0x00}},
    {0xC9,19,{0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,
                 0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00}},
    {0xCA,43,{0x1C,0xFC,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00}},
    {0xCB,27,{0xF8,0x85,0x1F,0xFA,0x0D,0x40,0x00,0x00,0x20,0x00,
                 0x11,0x50,0x00,0x0D,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xCC,2,{0x37,0x00}},
    {0xCD,38,{0x0D,0x00,0x23,0x00,0x23,0x00,0x5C,0x02,0xBF,0xBF,
                 0xE3,0xE3,0xAF,0xAF,0xE2,0xE2,0x01,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA2}},
    {0xCE,25,{0x5D,0x40,0x49,0x53,0x59,0x5E,0x63,0x68,0x6E,0x74,
                 0x7E,0x8A,0x98,0xA8,0xBB,0xD0,0xFF,0x04,0x00,0x04,
                 0x04,0x42,0x00,0x69,0x5A}},
    {0xCF,2,{0x48,0x1D}},
    {0xD0,19,{0x33,0x59,0xCF,0x31,0x01,0x10,0x10,0x10,0x19,0x19,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x6D,0x65}},
    {0xD1,1,{0x00}},
    {0xD3,21,{0x1B,0x3B,0xBB,0x77,0x77,0x77,0xBB,0xB3,0x33,0x00,
                 0x00,0x6E,0x60,0xE7,0xE7,0x33,0xBB,0xF2,0xFD,0xC6,
                 0x0B}},
    {0xD5,10,{0x03,0x00,0x00,0x02,0x2A,0x02,0x2A,0x01,0x00,0x00}},
    {0xD6,1,{0xC1}},
    {0xD7,33,{0x86,0xFF,0x03,0x05,0x41,0x24,0x80,0x1F,0xC7,0x1F,
                 0x1B,0x00,0x0C,0x07,0x20,0x00,0x00,0x00,0x00,0x00,
                 0x0C,0xF0,0x07,0x00,0x0C,0x00,0x00,0xAA,0x67,0x7E,
                 0x1D,0x06,0x00}},
    {0xD9,7,{0x00,0x00,0x14,0x3F,0x01,0x77,0x02}},
    {0xDD,4,{0x30,0x06,0x23,0x65}},
    {0xDE,4,{0x00,0x3F,0xFF,0x90}},
    {0xEA,1,{0x9F}},
    {0xEE,3,{0x41,0x51,0x00}},
    {0xF1,3,{0x00,0x00,0x00}},
    {0xB0,1,{0x03}},
    {0xB0,1,{0x00}},
    {0xD6,1,{0x01}},
    {0xB0,1,{0x03}},

    {0x11,0,{0x00}},
    {REGFLAG_DELAY, 130, {}},
    {0x29,0,{0x00}},
    {REGFLAG_DELAY, 20, {}},
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

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->physical_width = 63;
    params->physical_height = 110;
    // enable tearing-free
    params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    params->dsi.mode   = SYNC_EVENT_VDO_MODE;//SYNC_EVENT_VDO_MODE;//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                = 2; //2
    params->dsi.vertical_backporch                  = 14; //14
    params->dsi.vertical_frontporch                 = 16;  //16
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 8;//10 8
    params->dsi.horizontal_backporch				= 16;//34 32; 
    params->dsi.horizontal_frontporch				= 16;//24 43;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 460;
    params->dsi.cont_clock	= 1;
    params->dsi.ssc_disable = 1;
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
    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
    MDELAY(50);
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static void lcm_resume(void)
{
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
static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[3];
    int id=0;

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    MDELAY(10);

    read_reg_v2(0xbf, buffer, 2);
    id = (buffer[0] << 8) + buffer[1] + adc_read_vol();

#if defined(BUILD_LK)
    printf("%s,hct_td4310_dsi_vdo_fhd_auo_60_by buffer[0] = 0x%08x, id = 0x%08x\n", __func__, buffer[0], id);
#else
    printk("%s,hct_td4310_dsi_vdo_fhd_auo_60_by buffer[0] = 0x%08x, buffer[1] = 0x%08x, id = 0x%08x\n", __func__, buffer[0], buffer[1], id);
#endif
    return (0x023c == id)?1:0;
}

LCM_DRIVER hct_td4310_dsi_vdo_fhd_auo_60_by_lcm_drv =
{
    .name			= "hct_td4310_dsi_vdo_fhd_auo_60_by",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
};
