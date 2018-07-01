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
//#include "gpio_const.h"
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

//add XLLSHLSS-4 by qiang.xue 20171213 start
#include "../../../../input/touchscreen/mediatek/nt36672_x604/nt36xxx.h"
//add XLLSHLSS-4 by qiang.xue 20171213 end

#ifdef BUILD_LK
#ifndef USE_DTB_NO_DWS
#include <cust_gpio_usage.h>
#ifndef MACH_FPGA
#include <cust_i2c.h>
#endif
#endif

#ifdef USE_DTB_NO_DWS
#define I2C_MT6370_PMU_CHANNEL          5
#define I2C_MT6370_PMU_SLAVE_7_BIT_ADDR 0x34
#endif  // USE_DTB_NO_DWS
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)     pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)     pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_DEBUG(fmt, args...)  printk(fmt, ##args)
#define LCM_ERROR(fmt, args...)  printk(fmt, ##args)
#define AUXADC_LCM_VOLTAGE_CHANNEL     2
#define LCM_ID_MAX_VOLTAGE 150
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

//#define LCM_ID_NT36672 ()
extern unsigned int g_lcm_inversion;
static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
static unsigned long system_time_before = 0;
static unsigned long system_time_after = 0;
extern unsigned int is_lcm_suspend;
extern unsigned int jiffies_to_msecs(const unsigned long j);
#endif
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
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
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

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                     (1080)
#define FRAME_HEIGHT                                    (2160)

#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
#define LCM_BIAS_VOLTAGE                                5600000 //5.6v
#endif

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST                (GPIO45 | 0x80000000)
#endif


#define GPIO_CTP_RST                (GPIO2 | 0x80000000)



#ifndef GPIO_LCD_BIAS_ENP_PIN
#define GPIO_LCD_BIAS_ENP_PIN       (GPIO13 | 0x80000000)
#endif

#ifndef GPIO_LCD_BIAS_ENN_PIN
#define GPIO_LCD_BIAS_ENN_PIN       (GPIO14 | 0x80000000)
#endif

#define REGFLAG_DELAY           0xFFFC
#define REGFLAG_UDELAY          0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW       0xFFFE
#define REGFLAG_RESET_HIGH      0xFFFF


#define SET_RESET_PIN(v)        (mt_set_gpio_out(GPIO_LCM_RST,(v)))

//add XLLSHLSS-4 by qiang.xue 20171130 start
#if defined(WAKEUP_GESTURE)
extern int ctp_gesture_fun_enable;
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 end

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[120];
};

static void NT36672_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
    unsigned int data_array[16];

    data_array[0] = (0x00022902);
    data_array[1] = (0x00000000 | (para << 8) | (cmd));
    dsi_set_cmdq(data_array, 2, 1);
}

/*static void NT35695_DCS_write_1A_0P(unsigned char cmd)
{
    unsigned int data_array[16];

    data_array[0]=(0x00000500 | (cmd<<16));
    dsi_set_cmdq(data_array, 1, 1);
}*/
static void init_lcm_registers(void)
{
    unsigned int array[16];
#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    printk("%s winston_init_lcm_registers start\n",__func__);
#endif
    NT36672_DCS_write_1A_1P(0xFF,0x20);//cmd2 page 0
    NT36672_DCS_write_1A_1P(0xFB,0x01);
    NT36672_DCS_write_1A_1P(0x01,0x33);
    NT36672_DCS_write_1A_1P(0x06,0x99);
    NT36672_DCS_write_1A_1P(0x07,0x9E);
    NT36672_DCS_write_1A_1P(0x0E,0x30);
    NT36672_DCS_write_1A_1P(0x0F,0x2E);
    NT36672_DCS_write_1A_1P(0x1D,0x33);
    NT36672_DCS_write_1A_1P(0x6D,0x66);
    NT36672_DCS_write_1A_1P(0x68,0x03);
    NT36672_DCS_write_1A_1P(0x69,0x99);
//  NT36672_DCS_write_1A_1P(0x89,0x0F);  //PR0 no gamma OTP and Vcom OTP,so no need this
    NT36672_DCS_write_1A_1P(0x95,0xCD);
    NT36672_DCS_write_1A_1P(0x96,0xCD);
    NT36672_DCS_write_1A_1P(0xFF,0x24);//cmd2 page4
    NT36672_DCS_write_1A_1P(0xFB,0x01);
    NT36672_DCS_write_1A_1P(0x00,0x01);
    NT36672_DCS_write_1A_1P(0x01,0x1C);
    NT36672_DCS_write_1A_1P(0x02,0x0B);
    NT36672_DCS_write_1A_1P(0x03,0x0C);
    NT36672_DCS_write_1A_1P(0x04,0x29);
    NT36672_DCS_write_1A_1P(0x05,0x0F);
    NT36672_DCS_write_1A_1P(0x06,0x0F);
    NT36672_DCS_write_1A_1P(0x07,0x03);
    NT36672_DCS_write_1A_1P(0x08,0x05);
    NT36672_DCS_write_1A_1P(0x09,0x22);
    NT36672_DCS_write_1A_1P(0x0A,0x00);
    NT36672_DCS_write_1A_1P(0x0B,0x24);
    NT36672_DCS_write_1A_1P(0x0C,0x13);
    NT36672_DCS_write_1A_1P(0x0D,0x13);
    NT36672_DCS_write_1A_1P(0x0E,0x15);
    NT36672_DCS_write_1A_1P(0x0F,0x15);
    NT36672_DCS_write_1A_1P(0x10,0x17);
    NT36672_DCS_write_1A_1P(0x11,0x17);
    NT36672_DCS_write_1A_1P(0x12,0x01);
    NT36672_DCS_write_1A_1P(0x13,0x1C);
    NT36672_DCS_write_1A_1P(0x14,0x0B);
    NT36672_DCS_write_1A_1P(0x15,0x0C);
    NT36672_DCS_write_1A_1P(0x16,0x29);
    NT36672_DCS_write_1A_1P(0x17,0x0F);
    NT36672_DCS_write_1A_1P(0x18,0x0F);
    NT36672_DCS_write_1A_1P(0x19,0x04);
    NT36672_DCS_write_1A_1P(0x1A,0x06);
    NT36672_DCS_write_1A_1P(0x1B,0x23);
    NT36672_DCS_write_1A_1P(0x1C,0x0F);
    NT36672_DCS_write_1A_1P(0x1D,0x24);
    NT36672_DCS_write_1A_1P(0x1E,0x13);
    NT36672_DCS_write_1A_1P(0x1F,0x13);
    NT36672_DCS_write_1A_1P(0x20,0x15);
    NT36672_DCS_write_1A_1P(0x21,0x15);
    NT36672_DCS_write_1A_1P(0x22,0x17);
    NT36672_DCS_write_1A_1P(0x23,0x17);
    NT36672_DCS_write_1A_1P(0x2F,0x04);
    NT36672_DCS_write_1A_1P(0x30,0x08);
    NT36672_DCS_write_1A_1P(0x31,0x04);
    NT36672_DCS_write_1A_1P(0x32,0x08);
    NT36672_DCS_write_1A_1P(0x33,0x04);
    NT36672_DCS_write_1A_1P(0x34,0x04);
    NT36672_DCS_write_1A_1P(0x35,0x00);
    NT36672_DCS_write_1A_1P(0x37,0x09);
    NT36672_DCS_write_1A_1P(0x38,0x75);
    NT36672_DCS_write_1A_1P(0x39,0x75);
    NT36672_DCS_write_1A_1P(0x3B,0xC0);
    NT36672_DCS_write_1A_1P(0x3F,0x75);
    NT36672_DCS_write_1A_1P(0x60,0x10);
    NT36672_DCS_write_1A_1P(0x61,0x00);
    NT36672_DCS_write_1A_1P(0x68,0xC2);
    NT36672_DCS_write_1A_1P(0x78,0x80);
    NT36672_DCS_write_1A_1P(0x79,0x23);
    NT36672_DCS_write_1A_1P(0x7A,0x10);
    NT36672_DCS_write_1A_1P(0x7B,0x9B);
    NT36672_DCS_write_1A_1P(0x7C,0x80);
    NT36672_DCS_write_1A_1P(0x7D,0x06);
    NT36672_DCS_write_1A_1P(0x7E,0x02);
    NT36672_DCS_write_1A_1P(0x8E,0xF0);
    NT36672_DCS_write_1A_1P(0x92,0x76);
    NT36672_DCS_write_1A_1P(0x93,0x0A);
    NT36672_DCS_write_1A_1P(0x94,0x0A);
    NT36672_DCS_write_1A_1P(0x99,0x33);
    NT36672_DCS_write_1A_1P(0x9B,0xFF);//column inversion
    NT36672_DCS_write_1A_1P(0x9F,0x00);
    NT36672_DCS_write_1A_1P(0xA3,0x91);
    NT36672_DCS_write_1A_1P(0xB3,0x00);
    NT36672_DCS_write_1A_1P(0xB4,0x00);
    NT36672_DCS_write_1A_1P(0xB5,0x04);
    NT36672_DCS_write_1A_1P(0xDC,0x40);
    NT36672_DCS_write_1A_1P(0xDD,0x03);
    NT36672_DCS_write_1A_1P(0xDE,0x01);
    NT36672_DCS_write_1A_1P(0xDF,0x3D);
    NT36672_DCS_write_1A_1P(0xE0,0x3D);
    NT36672_DCS_write_1A_1P(0xE1,0x22);
    NT36672_DCS_write_1A_1P(0xE2,0x24);
    NT36672_DCS_write_1A_1P(0xE3,0x0A);
    NT36672_DCS_write_1A_1P(0xE4,0x0A);
    NT36672_DCS_write_1A_1P(0xE8,0x01);
    NT36672_DCS_write_1A_1P(0xE9,0x10);
    NT36672_DCS_write_1A_1P(0xED,0x40);
    NT36672_DCS_write_1A_1P(0xFF,0x25);//cmd2 page5
    NT36672_DCS_write_1A_1P(0xFB,0x01);
    NT36672_DCS_write_1A_1P(0x0A,0x81);
    NT36672_DCS_write_1A_1P(0x0B,0xCD);
    NT36672_DCS_write_1A_1P(0x0C,0x01);
    NT36672_DCS_write_1A_1P(0x17,0x82);
    NT36672_DCS_write_1A_1P(0x21,0x1B);
    NT36672_DCS_write_1A_1P(0x22,0x1B);
    NT36672_DCS_write_1A_1P(0x24,0x76);
    NT36672_DCS_write_1A_1P(0x25,0x76);
    NT36672_DCS_write_1A_1P(0x30,0x2A);
    NT36672_DCS_write_1A_1P(0x31,0x2A);
    NT36672_DCS_write_1A_1P(0x38,0x2A);
    NT36672_DCS_write_1A_1P(0x3F,0x11);
    NT36672_DCS_write_1A_1P(0x40,0x3A);
    NT36672_DCS_write_1A_1P(0x4B,0x31);
    NT36672_DCS_write_1A_1P(0x4C,0x3A);
    NT36672_DCS_write_1A_1P(0x58,0x22);
    NT36672_DCS_write_1A_1P(0x59,0x05);
    NT36672_DCS_write_1A_1P(0x5A,0x0A);
    NT36672_DCS_write_1A_1P(0x5B,0x0A);
    NT36672_DCS_write_1A_1P(0x5C,0x25);
    NT36672_DCS_write_1A_1P(0x5D,0x80);
    NT36672_DCS_write_1A_1P(0x5E,0x80);
    NT36672_DCS_write_1A_1P(0x5F,0x28);
    NT36672_DCS_write_1A_1P(0x62,0x3F);
    NT36672_DCS_write_1A_1P(0x63,0x82);
    NT36672_DCS_write_1A_1P(0x65,0x00);
    NT36672_DCS_write_1A_1P(0x66,0xDD);
    NT36672_DCS_write_1A_1P(0x6C,0x6D);
    NT36672_DCS_write_1A_1P(0x71,0x6D);
    NT36672_DCS_write_1A_1P(0x78,0x25);
    NT36672_DCS_write_1A_1P(0xC3,0x00);
    NT36672_DCS_write_1A_1P(0xFF,0x26);//cmd2 page6
    NT36672_DCS_write_1A_1P(0xFB,0x01);
    NT36672_DCS_write_1A_1P(0x06,0xC8);
    NT36672_DCS_write_1A_1P(0x12,0x5A);
    NT36672_DCS_write_1A_1P(0x19,0x09);
    NT36672_DCS_write_1A_1P(0x1A,0x84);
    NT36672_DCS_write_1A_1P(0x1C,0xFA);
    NT36672_DCS_write_1A_1P(0x1D,0x09);
    NT36672_DCS_write_1A_1P(0x1E,0x0B);
    NT36672_DCS_write_1A_1P(0x99,0x20);
    NT36672_DCS_write_1A_1P(0xFF,0x27);//cmd2 page7
    NT36672_DCS_write_1A_1P(0xFB,0x01);
    NT36672_DCS_write_1A_1P(0x13,0x08);
    NT36672_DCS_write_1A_1P(0x14,0x43);
    NT36672_DCS_write_1A_1P(0x16,0xB8);
    NT36672_DCS_write_1A_1P(0x17,0xB8);
    NT36672_DCS_write_1A_1P(0x7A,0x02);
    NT36672_DCS_write_1A_1P(0xFF,0x10);

    array[0] = 0x00110500;
    dsi_set_cmdq(array, 1, 1);
#if !defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    MDELAY(100);//120->80

    array[0] = 0x00290500;
    dsi_set_cmdq(array, 1, 1);
    MDELAY(40);
#endif
#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    printk("%s winston_init_lcm_registers end\n",__func__);
#endif
}

#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
static struct LCM_setting_table lcm_29_cmd_setting[] = {
    {0x29,0,{0x00}},
};
#endif

static struct LCM_setting_table lcm_deep_standby_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 60, {}},
    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 70, {}},
    //enter deep standby
    {0x4F, 1, {0x01}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};
//add XLLSHLSS-4 by qiang.xue 20171213 start
#if defined (WAKEUP_GESTURE)
static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 60, {}},
    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 70, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};
#endif
//add XLLSHLSS-4 by qiang.xue 20171213 end

/*add by yongjian.huang for ca8 project start*/
#ifdef CONFIG_CTP_SUSPEND_RESUME_CUSTOM
extern void CTP_suspend_resume(int bl_level, int suspend_flag);
#endif
/*add by yongjian.huang for ca8 project end*/

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

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->lcm_if = LCM_INTERFACE_DSI0;
    params->lcm_cmd_if = LCM_INTERFACE_DSI0;
        /* add for *#88* flicker test */
    params->inversion = LCM_INVERSIONE_COLUMN;
    g_lcm_inversion = LCM_INVERSIONE_COLUMN;

#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
    lcm_bias_vol = LCM_BIAS_VOLTAGE;
#endif

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    //params->dsi.mode   = BURST_VDO_MODE;
    params->dsi.mode = SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
    lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif

    /* Command mode setting */
    params->dsi.LANE_NUM                    = LCM_FOUR_LANE;

    /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format          = LCM_DSI_FORMAT_RGB888;

    /* Highly depends on LCD driver capability */

    /* video mode timing */
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.vertical_sync_active                = 8;
    params->dsi.vertical_backporch                  = 8;
    params->dsi.vertical_frontporch                 = 16;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active              = 10;
    params->dsi.horizontal_backporch                = 12;
    params->dsi.horizontal_frontporch               = 50;
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    /* this value must be in MTK suggested table */

    params->dsi.ssc_disable                         = 0;
    params->dsi.ssc_range                           = 4;
    params->dsi.HS_TRAIL                            = 15;

    params->dsi.PLL_CLOCK = 492;
    params->dsi.noncont_clock = 1;
    params->dsi.noncont_clock_period = 1;

    params->dsi.esd_check_enable = 0;
    params->dsi.customization_esd_check_enable      = 0;
    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;


}

#ifdef BUILD_LK
#define RT5081_SLAVE_ADDR_WRITE  0x7C

static int RT5081_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer)
{
    kal_uint32 ret = I2C_OK;
    kal_uint16 len;
    struct mt_i2c_t RT5081_i2c;
    *dataBuffer = addr;

    RT5081_i2c.id = I2C_MT6370_PMU_CHANNEL;
    RT5081_i2c.addr = I2C_MT6370_PMU_SLAVE_7_BIT_ADDR;
    RT5081_i2c.mode = ST_MODE;
    RT5081_i2c.speed = 100;
    len = 1;

    ret = i2c_write_read(&RT5081_i2c, dataBuffer, len, len);
    if (I2C_OK != ret)
        LCM_LOGI("%s: i2c_read  failed! ret: %d\n", __func__, ret);

    return ret;
}

static int RT5081_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;
    struct mt_i2c_t RT5081_i2c;

    write_data[0] = addr;
    write_data[1] = value;

    RT5081_i2c.id = I2C_MT6370_PMU_CHANNEL;
    RT5081_i2c.addr = I2C_MT6370_PMU_SLAVE_7_BIT_ADDR;
    RT5081_i2c.mode = ST_MODE;
    RT5081_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&RT5081_i2c, write_data, len);

    return ret_code;
}

static int RT5081_REG_MASK (kal_uint8 addr, kal_uint8 val, kal_uint8 mask)
{
    kal_uint8 RT5081_reg = 0;
    kal_uint32 ret = 0;

    ret = RT5081_read_byte(addr, &RT5081_reg);

    RT5081_reg &= ~mask;
    RT5081_reg |= val;

    ret = RT5081_write_byte(addr, RT5081_reg);

    return ret;
}

static void RT5081_avdd_avee_power_on(void)
{
    int ret = 0;
    /*config rt5081 register 0xB2[7:6]=0x3, that is set db_delay=4ms.*/
    ret = RT5081_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));

    /* set AVDD 5.4v, (4v+28*0.05v) */
    /*ret = RT5081_write_byte(0xB3, (1 << 6) | 28);*/
    ret = RT5081_REG_MASK(0xB3, 32, (0x3F << 0));//set vol is 5.6V
    if (ret < 0)
        LCM_LOGI("NT36672----tps6132----cmd=%0x--i2c write error----\n", 0xB3);
    else
        LCM_LOGI("NT36672----tps6132----cmd=%0x--i2c write success----\n", 0xB3);

    /* set AVEE */
    /*ret = RT5081_write_byte(0xB4, (1 << 6) | 28);*/
    ret = RT5081_REG_MASK(0xB4, 32, (0x3F << 0));
    if (ret < 0)
        LCM_LOGI("NT36672----tps6132----cmd=%0x--i2c write error----\n", 0xB4);
    else
        LCM_LOGI("NT36672----tps6132----cmd=%0x--i2c write success----\n", 0xB4);

    /* enable AVDD & AVEE */
    /* 0x12--default value; bit3--Vneg; bit6--Vpos; */
    /*ret = RT5081_write_byte(0xB1, 0x12 | (1<<3) | (1<<6));*/
    ret = RT5081_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
    if (ret < 0)
        LCM_LOGI("NT36672----tps6132----cmd=%0x--i2c write error----\n", 0xB1);
    else
        LCM_LOGI("NT36672----tps6132----cmd=%0x--i2c write success----\n", 0xB1);

    MDELAY(15);
}
#endif


static void lcm_init_power(void)
{
/*no need to set tp_reset,because tp_reset is attach to IO1.8v*/

    //  tpd_gpio_output(tpd_rst_gpio_number, 1);
    //  MDELAY(1);
#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
    tran_display_bias_enable(LCM_BIAS_VOLTAGE); //set vol 5.6v
#else
    display_bias_enable();  //mtk default is 5.4v,so we cannot using it.
#endif
}

static void lcm_suspend_power(void)
{
    // disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
    //tpd_gpio_output(tpd_rst_gpio_number, 0);
    //SET_RESET_PIN(0);
//add XLLSHLSS-4 by qiang.xue 20171130 start
#if defined (WAKEUP_GESTURE)
    if (!ctp_gesture_fun_enable) {
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 end
    display_bias_disable();
//add XLLSHLSS-4 by qiang.xue 20171130 start
#if defined (WAKEUP_GESTURE)
	}
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 end
}

static void lcm_resume_power(void)
{
//add XLLSHLSS-4 by qiang.xue 20171130 start
#if defined (WAKEUP_GESTURE)
    if (!ctp_gesture_fun_enable) {
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 end
    //  tpd_gpio_output(tpd_rst_gpio_number, 1);
    //  MDELAY(1);
#ifdef CONFIG_TRAN_LCM_SET_VOLTAGE
    tran_display_bias_enable(LCM_BIAS_VOLTAGE); //set vol 5.6v
#else
    display_bias_enable();  //mtk default is 5.4v,so we cannot using it.
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 start
#if defined (WAKEUP_GESTURE)
	}
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 end
}
static void lcm_init(void)
{
    MDELAY(15);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
    MDELAY(10);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
    MDELAY(15);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
    MDELAY(10);

    /* when phone initial , config output high, enable backlight drv chip */
    init_lcm_registers();
}

static void lcm_suspend(void)
{
//add XLLSHLSS-4 by qiang.xue 20171130 start
#if defined (WAKEUP_GESTURE)
    if(ctp_gesture_fun_enable){
		push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
		//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	}
    else{
#endif
//add XLLSHLSS-4 by qiang.xue 20171130 end
		push_table(lcm_deep_standby_mode_in_setting, sizeof(lcm_deep_standby_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	//add XLLSHLSS-4 by qiang.xue 20171130 start
	#if defined (WAKEUP_GESTURE)
	}
	#endif
	//add XLLSHLSS-4 by qiang.xue 20171130 end
}

static void lcm_resume(void)
{
#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    printk("%s winston_lcm_resume start\n",__func__);
#endif
    MDELAY(15);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
    MDELAY(10);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
    MDELAY(15);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
    MDELAY(10);
    init_lcm_registers();
#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    system_time_before = jiffies_to_msecs(jiffies);
    UDELAY(5000);
#endif
#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    printk("%s winston_lcm_resume end\n",__func__);
#endif
}


static unsigned int lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;
#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
        LCM_ERROR("(%s) nt36672_fhdp_dsi_vdo_tcl_auo_x604  get lcm chip id vol fail\n", __func__);
        return 0;
    }
#endif
    lcm_vol = data[0]*1000+data[1]*10;

        LCM_DEBUG("(%s) nt36672_fhdp_dsi_vdo_tcl_auo_x604 lcm chip id adc raw data:%d, lcm_vol:%d\n", __func__, rawdata, lcm_vol);

    if (lcm_vol <= LCM_ID_MAX_VOLTAGE)
        return 1;
    else
        return 0;
}

#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
static void lcm_display_on(void)
{
    //unsigned int array[2];
    unsigned int time_diff;
    is_lcm_suspend = FALSE;
    system_time_after = jiffies_to_msecs(jiffies);
    time_diff = system_time_after - system_time_before;
    LCM_DEBUG("%s system_time_before:%lu,system_time_after:%lu,time_diff:%d\n",__func__,system_time_before,system_time_after,time_diff);
    if (100 > time_diff){
        MDELAY(100 - time_diff);
    }

    //array[0] = 0x00290500;
    //dsi_set_cmdq(array, 1, 1);
    LCM_DEBUG("resume and start to set cmd 29\n");
    push_table(lcm_29_cmd_setting,sizeof(lcm_29_cmd_setting) /sizeof(struct LCM_setting_table),1);
    LCM_DEBUG("%s before cmd 29 delay\n",__func__);
    UDELAY(40000);
    LCM_DEBUG("%s after cmd 29 delay\n",__func__);
}
#endif

LCM_DRIVER nt36672_fhdp_dsi_vdo_tcl_auo_x604_lcm_drv =
{
    .name           = "nt36672_fhdp_dsi_vdo_tcl_auo_x604",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
#if defined(CONFIG_TRAN_LCM_TIME_OPT_ENABLE)
    .set_dis_on     = lcm_display_on,
#endif
};
