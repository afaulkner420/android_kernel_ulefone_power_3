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

#ifndef __TPD_H
#define __TPD_H
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <generated/autoconf.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>

/*debug macros */
#define TPD_DEBUG
#define TPD_DEBUG_CODE
/* #define TPD_DEBUG_TRACK */
#define TPD_DMESG(a, arg...) pr_info(TPD_DEVICE ": " a, ##arg)
#if defined(TPD_DEBUG)
#undef TPD_DEBUG
#define TPD_DEBUG(a, arg...) pr_info(TPD_DEVICE ": " a, ##arg)
#else
#define TPD_DEBUG(arg...)
#endif

/* register, address, configurations */
#define TPD_DEVICE            "mtk-tpd"
#define TPD_X                  0
#define TPD_Y                  1
#define TPD_Z1                 2
#define TPD_Z2                 3
#define TP_DELAY              (2*HZ/100)
#define TP_DRV_MAX_COUNT          (20)
#define TPD_WARP_CNT          (4)
#define TPD_VIRTUAL_KEY_MAX   (10)

/* various mode */
#define TPD_MODE_NORMAL        0
#define TPD_MODE_KEYPAD        1
#define TPD_MODE_SW 2
#define TPD_MODE_FAV_SW 3
#define TPD_MODE_FAV_HW 4
#define TPD_MODE_RAW_DATA 5
#undef TPD_RES_X
#undef TPD_RES_Y
extern unsigned long TPD_RES_X;
extern unsigned long TPD_RES_Y;
extern int tpd_load_status;	/* 0: failed, 1: success */
extern int tpd_mode;
extern int tpd_mode_axis;
extern int tpd_mode_min;
extern int tpd_mode_max;
extern int tpd_mode_keypad_tolerance;
extern int tpd_em_debounce_time;
extern int tpd_em_debounce_time0;
extern int tpd_em_debounce_time1;
extern int tpd_em_asamp;
extern int tpd_em_auto_time_interval;
extern int tpd_em_sample_cnt;
extern int tpd_calmat[];
extern int tpd_def_calmat[];
extern int tpd_calmat[];
extern int tpd_def_calmat[];
extern int TPD_DO_WARP;
extern int tpd_wb_start[];
extern int tpd_wb_end[];
extern int tpd_v_magnify_x;
extern int tpd_v_magnify_y;
extern unsigned int DISP_GetScreenHeight(void);
extern unsigned int DISP_GetScreenWidth(void);
extern void tpd_ldo_power_enable(bool en);
#if defined(CONFIG_MTK_S3320) || defined(CONFIG_MTK_S3320_47) || defined(CONFIG_MTK_S3320_50)
extern void synaptics_init_sysfs(void);
#endif /* CONFIG_MTK_S3320 */
extern void tpd_button_init(void);
struct tpd_device {
	struct device *tpd_dev;
	struct regulator *reg;
	struct regulator *io_reg;
	struct input_dev *dev;
	struct input_dev *kpd;
	struct timer_list timer;
	struct tasklet_struct tasklet;
	int btn_state;
};
struct tpd_key_dim_local {
	int key_x;
	int key_y;
	int key_width;
	int key_height;
};

struct tpd_filter_t {
	int enable; /*0: disable, 1: enable*/
	int pixel_density; /*XXX pixel/cm*/
	int W_W[3][4];/*filter custom setting prameters*/
	unsigned int VECLOCITY_THRESHOLD[3];/*filter speed custom settings*/
};

struct tpd_dts_info {
	int tpd_resolution[2];
	int touch_max_num;
	int use_tpd_button;
	int tpd_key_num;
	int tpd_key_local[4];
/* Vanzo:yangzhihong on: Tue, 19 Apr 2016 19:57:57 +0800
 */
    int tpd_switch_vkey;
// End of Vanzo:yangzhihong
	bool tpd_use_ext_gpio;
	int rst_ext_gpio_num;
	struct tpd_key_dim_local tpd_key_dim_local[4];
	struct tpd_filter_t touch_filter;
};
extern struct tpd_dts_info tpd_dts_data;
struct tpd_attrs {
	struct device_attribute **attr;
	int num;
};
struct tpd_driver_t {
	char *tpd_device_name;
	int (*tpd_local_init)(void);
	void (*suspend)(struct device *h);
	void (*resume)(struct device *h);
	int tpd_have_button;
	struct tpd_attrs attrs;
};


#if 1				/* #ifdef TPD_HAVE_BUTTON */
void tpd_button(unsigned int x, unsigned int y, unsigned int down);
void tpd_button_init(void);
ssize_t tpd_virtual_key(char *buf);
/* #ifndef TPD_BUTTON_HEIGHT */
/* #define TPD_BUTTON_HEIGHT TPD_RES_Y */
/* #endif */
#endif

extern int tpd_driver_add(struct tpd_driver_t *tpd_drv);
extern int tpd_driver_remove(struct tpd_driver_t *tpd_drv);
void tpd_button_setting(int keycnt, void *keys, void *keys_dim);
extern int tpd_em_spl_num;
extern int tpd_em_pressure_threshold;
extern struct tpd_device *tpd;
extern void tpd_get_dts_info(void);
#define GTP_RST_PORT    0
#define GTP_INT_PORT    1
extern void tpd_gpio_as_int(int pin);
extern void tpd_gpio_output(int pin, int level);
/* Vanzo:yuntaohe on: Mon, 11 Jan 2016 14:15:51 +0800
 */
#ifndef CONFIG_TPD_POWER_SOURCE_VIA_VGP
extern void tpd_ldo_power_enable(bool en);
#endif
// End of Vanzo:yuntaohe
extern const struct of_device_id touch_of_match[];
#ifdef TPD_DEBUG_CODE
#include "tpd_debug.h"
#endif
#ifdef TPD_DEBUG_TRACK
int DAL_Clean(void);
int DAL_Printf(const char *fmt, ...);
int LCD_LayerEnable(int id, BOOL enable);
#endif

#ifdef TPD_HAVE_CALIBRATION
#include "tpd_calibrate.h"
#endif

#include "tpd_default.h"

/* switch touch panel into different mode */
void _tpd_switch_single_mode(void);
void _tpd_switch_multiple_mode(void);
void _tpd_switch_sleep_mode(void);
void _tpd_switch_normal_mode(void);


/* Vanzo:yangzhihong on: Thu, 25 Feb 2016 20:26:13 +0800
 */
#define CFG_TPD_MAX_TOUCH_NUM   5
#if (defined(FHDPLUS) || defined(HDPLUS))
#define CFG_TPD_USE_BUTTON      0
#else
#define CFG_TPD_USE_BUTTON      1
#endif

//#if CFG_TPD_USE_BUTTON
#if (defined(WVGA) || defined(CU_WVGA) || defined(CMCC_WVGA) || defined(CMCC_LTE_WVGA))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{60,870,60,50},{180,870,60,50},{300,870,60,50},{420,870,60,50}}
#define CFG_TPD_WIDTH               480
#define CFG_TPD_HEIGHT              800

#elif (defined(FWVGA) || defined(CU_FWVGA) || defined(CMCC_FWVGA) || defined(CMCC_LTE_FWVGA))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{60,920,60,50},{180,920,60,50},{300,920,60,50},{420,920,60,50}}
#define CFG_TPD_WIDTH               480
#define CFG_TPD_HEIGHT              854

#elif (defined(QHD) || defined(CU_QHD) || defined(CMCC_QHD) || defined(CMCC_LTE_QHD))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{85,1030,60,50},{185,1030,60,50},{350,1030,60,50},{500,1030,60,50}}
#define CFG_TPD_WIDTH               540
#define CFG_TPD_HEIGHT              960

#elif (defined(HD) || defined(HD720) || defined(CU_HD720) || defined(CMCC_HD720)|| defined(CMCC_LTE_HD720))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, 0}
#define CFG_TPD_KEYS_DIM            {{90,1350,60,50},{270,1350,60,50},{430,1350,60,50},{630,1350,60,50}}
#define CFG_TPD_WIDTH               720
#define CFG_TPD_HEIGHT              1280
#elif (defined(HDPLUS) || defined(LHD720) || defined(LHD720) || defined(CU_LHD720) || defined(CMCC_LHD720)|| defined(CMCC_LTE_LHD720))

#define CFG_TPD_KEY_COUNT           0
#define CFG_TPD_KEYS                {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, 0}
#define CFG_TPD_KEYS_DIM            {{90,1550,60,50},{270,1550,60,50},{430,1550,60,50},{630,1550,60,50}}
#define CFG_TPD_WIDTH               720
#define CFG_TPD_HEIGHT              1440

#elif (defined(FHDPLUS))

#define CFG_TPD_KEY_COUNT           0
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{200,2300,100,100},{500,2300,100,100},{800,2300,100,100}}
#define CFG_TPD_WIDTH               1080
#define CFG_TPD_HEIGHT              2160
#elif (defined(FHD) || defined(CU_FHD) || defined(CMCC_FHD) || defined(CMCC_LTE_FHD))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{200,2100,100,100},{500,2100,100,100},{800,2100,100,100}}
#define CFG_TPD_WIDTH               1080
#define CFG_TPD_HEIGHT              1920

#elif (defined(HVGA))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{40,530,60,50},{120,530,60,50},{200,530,60,50},{280,530,60,50}}
#define CFG_TPD_WIDTH               320
#define CFG_TPD_HEIGHT              480

#elif (defined(LQHD))

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{50,1030,60,50},{185,1030,60,50},{350,1030,60,50},{500,1030,60,50}}
#define CFG_TPD_WIDTH               540
#define CFG_TPD_HEIGHT              960

#else

#define CFG_TPD_KEY_COUNT           4
#define CFG_TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU, 0}
#define CFG_TPD_KEYS_DIM            {{60,920,60,50},{180,920,60,50},{300,920,60,50},{420,920,60,50}}
#define CFG_TPD_WIDTH               480
#define CFG_TPD_HEIGHT              854

#endif
//#endif /*CFG_TPD_USE_BUTTON*/

// End of Vanzo:yangzhihong
#endif
