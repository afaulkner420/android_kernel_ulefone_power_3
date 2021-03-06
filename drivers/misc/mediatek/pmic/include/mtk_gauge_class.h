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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *    mtk_battery.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of the Anroid Battery service for updating the battery status
 *
 * Author:
 * -------
 * Weiching Lin
 *
 ****************************************************************************/
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/wait.h>		/* For wait queue*/
#include <linux/sched.h>	/* For wait queue*/
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/platform_device.h>	/* platform device */
#include <linux/time.h>
#include <linux/wakelock.h>

#include <linux/netlink.h>	/* netlink */
#include <linux/kernel.h>
#include <linux/socket.h>	/* netlink */
#include <linux/skbuff.h>	/* netlink */
#include <net/sock.h>		/* netlink */
#include <linux/cdev.h>		/* cdev */

#include <linux/err.h>	/* IS_ERR, PTR_ERR */
#include <linux/reboot.h>	/*kernel_power_off*/
#include <linux/proc_fs.h>

#include <linux/vmalloc.h>

#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_charger.h>
#include <mt-plat/mtk_battery.h>

#ifndef __MTK_GAUGE_CLASS_H__
#define __MTK_GAUGE_CLASS_H__


struct gauge_device;
struct gauge_ops;

enum gauge_info {
	GAUGE_2SEC_REBOOT,
	GAUGE_PL_CHARGING_STATUS,
	GAUGE_MONITER_PLCHG_STATUS,
	GAUGE_INFO_MAX
};

enum gauge_hw_version {
	GAUGE_HW_V1000 = 1000,
	GAUGE_HW_V2000 = 2000,

	GAUGE_HW_MAX
};

struct gauge_properties {
	const char *alias_name;
};

struct gauge_hw_status {
	int hw_ocv;
	int sw_ocv;
	int soc;
	int ui_soc;
	int nafg_vbat;
	bool flag_hw_ocv_unreliable;
	int is_bat_charging;
	signed int sw_car_nafg_cnt;
	signed int sw_car_nafg_dltv;
	signed int sw_car_nafg_c_dltv;
	int iavg_intr_flag;
};

struct gauge_ops {
	int (*suspend)(struct gauge_device *, pm_message_t);
	int (*resume)(struct gauge_device *);

	int (*gauge_initial)(struct gauge_device *);
	int (*gauge_read_current)(struct gauge_device *gauge_dev, bool *fg_is_charging, int *data);
	int (*gauge_get_average_current)(struct gauge_device *gauge_dev, int *data, bool *valid);
	int (*gauge_get_coulomb)(struct gauge_device *gauge_dev, int *data);
	int (*gauge_reset_hw)(struct gauge_device *);
	int (*gauge_get_hwocv)(struct gauge_device *, int *data);
	int (*gauge_set_coulomb_interrupt1_ht)(struct gauge_device *, int car);
	int (*gauge_set_coulomb_interrupt1_lt)(struct gauge_device *, int car);
	int (*gauge_get_boot_battery_plug_out_status)(struct gauge_device *gauge_dev, int *is_plugout,
		int *plutout_time);
	int (*gauge_get_ptim_current)(struct gauge_device *gauge_dev, int *ptim_current, bool *is_charging);
	int (*gauge_get_zcv_current)(struct gauge_device *gauge_dev, int *zcv_current);
	int (*gauge_get_zcv)(struct gauge_device *gauge_dev, int *zcv);
	int (*gauge_is_gauge_initialized)(struct gauge_device *gauge_dev, int *init);
	int (*gauge_set_gauge_initialized)(struct gauge_device *gauge_dev, int init);
	int (*gauge_set_battery_cycle_interrupt)(struct gauge_device *gauge_dev, int car);
	int (*gauge_reset_shutdown_time)(struct gauge_device *gauge_dev);
	int (*gauge_reset_ncar)(struct gauge_device *gauge_dev);
	int (*gauge_set_nag_zcv)(struct gauge_device *gauge_dev, int zcv);
	int (*gauge_set_nag_c_dltv)(struct gauge_device *gauge_dev, int c_dltv_mv);
	int (*gauge_enable_nag_interrupt)(struct gauge_device *gauge_dev, int en);
	int (*gauge_get_nag_cnt)(struct gauge_device *gauge_dev, int *nag_cnt);
	int (*gauge_get_nag_dltv)(struct gauge_device *gauge_dev, int *nag_dltv);
	int (*gauge_get_nag_c_dltv)(struct gauge_device *gauge_dev, int *nag_c_dltv);
	int (*gauge_get_nag_vbat)(struct gauge_device *gauge_dev, int *vbat);
	int (*gauge_enable_zcv_interrupt)(struct gauge_device *gauge_dev, int en);
	int (*gauge_set_zcv_interrupt_threshold)(struct gauge_device *gauge_dev, int threshold);
	int (*gauge_enable_battery_tmp_lt_interrupt)(struct gauge_device *gauge_dev, bool en, int threshold);
	int (*gauge_enable_battery_tmp_ht_interrupt)(struct gauge_device *gauge_dev, bool en, int threshold);
	int (*gauge_get_time)(struct gauge_device *gauge_dev, unsigned int *time);
	int (*gauge_set_time_interrupt)(struct gauge_device *gauge_dev, int threshold);
	int (*gauge_enable_time_interrupt)(struct gauge_device *gauge_dev, int threshold);
	int (*gauge_get_hw_status)(struct gauge_device *gauge_dev, struct gauge_hw_status *hw_status, int interno);
	int (*gauge_enable_bat_plugout_interrupt)(struct gauge_device *gauge_dev, int en);
	int (*gauge_enable_iavg_interrupt)(struct gauge_device *gauge_dev, bool ht_en, int ht_th,
	bool lt_en, int lt_th);
	int (*gauge_enable_vbat_low_interrupt)(struct gauge_device *gauge_dev, int en);
	int (*gauge_enable_vbat_high_interrupt)(struct gauge_device *gauge_dev, int en);
	int (*gauge_set_vbat_low_threshold)(struct gauge_device *gauge_dev, int threshold);
	int (*gauge_set_vbat_high_threshold)(struct gauge_device *gauge_dev, int threshold);
	int (*gauge_enable_car_tune_value_calibration)(struct gauge_device *gauge_dev, int init_current,
		int *car_tune_value);
	int (*gauge_set_rtc_ui_soc)(struct gauge_device *gauge_dev, int ui_soc);
	int (*gauge_get_rtc_ui_soc)(struct gauge_device *gauge_dev, int *ui_soc);
	int (*gauge_is_rtc_invalid)(struct gauge_device *gauge_dev, int *invalid);
	int (*gauge_set_reset_status)(struct gauge_device *gauge_dev, int reset);
	int (*gauge_dump)(struct gauge_device *gauge_dev, struct seq_file *m);
	int (*gauge_get_hw_version)(struct gauge_device *gauge_dev);
	int (*gauge_set_info)(struct gauge_device *gauge_dev, enum gauge_info ginfo, int value);
	int (*gauge_get_info)(struct gauge_device *gauge_dev, enum gauge_info ginfo, int *value);

};

struct gauge_hw_info_data {
	int current_1;
	int current_2;
	int current_avg;
	int current_avg_sign;
	int car;
	int ncar;
	int time;
	int iavg_lt;
	int iavg_ht;
};

struct gauge_device {
	const struct gauge_ops *ops;
	struct mutex ops_lock;
	struct charger_properties props;
	struct device dev;

	struct gauge_hw_info_data fg_hw_info;
	struct fuel_gauge_custom_data *fg_cust_data;

};

#define to_gauge_device(obj) container_of(obj, struct gauge_device, dev)

extern struct gauge_device *get_gauge_by_name(const char *name);
extern struct gauge_device *gauge_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct gauge_ops *ops,
		const struct gauge_properties *props);
extern void gauge_device_unregister(struct gauge_device *gauge_dev);
extern int gauge_dev_initial(struct gauge_device *gauge_dev);
extern int gauge_dev_get_current(struct gauge_device *gauge_dev, bool *is_charging, int *battery_current);
extern int gauge_dev_get_average_current(struct gauge_device *gauge_dev, int *iavg, bool *valid);
extern int gauge_dev_get_coulomb(struct gauge_device *gauge_dev, int *data);
extern int gauge_dev_reset_hw(struct gauge_device *gauge_dev);
extern int gauge_dev_get_hwocv(struct gauge_device *, int *data);
extern int gauge_dev_set_coulomb_interrupt1_ht(struct gauge_device *gauge_dev, int car);
extern int gauge_dev_set_coulomb_interrupt1_lt(struct gauge_device *gauge_dev, int car);
extern int gauge_dev_get_boot_battery_plug_out_status(struct gauge_device *gauge_dev,
	int *is_plugout, int *plutout_time);
extern int gauge_dev_get_ptim_current(struct gauge_device *gauge_dev,
	int *ptim_current, bool *is_charging); /* by ptim */
extern int gauge_dev_get_zcv_current(struct gauge_device *gauge_dev, int *zcv_current);
extern int gauge_dev_get_zcv(struct gauge_device *gauge_dev, int *zcv);
extern int gauge_dev_is_gauge_initialized(struct gauge_device *gauge_dev, int *init);
extern int gauge_dev_set_gauge_initialized(struct gauge_device *gauge_dev, int init);
extern int gauge_dev_set_battery_cycle_interrupt(struct gauge_device *gauge_dev, int car);
extern int gauge_dev_reset_shutdown_time(struct gauge_device *gauge_dev);
extern int gauge_dev_reset_ncar(struct gauge_device *gauge_dev);
extern int gauge_dev_set_nag_zcv(struct gauge_device *gauge_dev, int zcv);
extern int gauge_dev_set_nag_c_dltv(struct gauge_device *gauge_dev, int c_dltv_mv);
extern int gauge_dev_enable_nag_interrupt(struct gauge_device *gauge_dev, int en);
extern int gauge_dev_get_nag_cnt(struct gauge_device *gauge_dev, int *nag_cnt);
extern int gauge_dev_get_nag_dltv(struct gauge_device *gauge_dev, int *nag_dltv);
extern int gauge_dev_get_nag_c_dltv(struct gauge_device *gauge_dev, int *nag_c_dltv);
extern int gauge_dev_enable_zcv_interrupt(struct gauge_device *gauge_dev, int en);
extern int gauge_dev_set_zcv_interrupt_threshold(struct gauge_device *gauge_dev, int threshold);
extern int gauge_dev_get_time(struct gauge_device *gauge_dev, unsigned int *time);
extern int gauge_dev_enable_time_interrupt(struct gauge_device *gauge_dev, int threshold);
extern int gauge_dev_get_hw_status(struct gauge_device *gauge_dev, struct gauge_hw_status *hw, int interno);
extern int gauge_dev_enable_bat_plugout_interrupt(struct gauge_device *gauge_dev, int en);
extern int gauge_dev_enable_iavg_interrupt(struct gauge_device *gauge_dev, bool ht_en, int ht_th,
	bool lt_en, int lt_th);
extern int gauge_dev_enable_vbat_low_interrupt(struct gauge_device *gauge_dev, int en);
extern int gauge_dev_enable_vbat_high_interrupt(struct gauge_device *gauge_dev, int en);
extern int gauge_dev_set_vbat_low_threshold(struct gauge_device *gauge_dev, int threshold);
extern int gauge_dev_set_vbat_high_threshold(struct gauge_device *gauge_dev, int threshold);
extern int gauge_dev_enable_car_tune_value_calibration(struct gauge_device *gauge_dev,
	int init_current, int *car_tune_value);
extern int gauge_dev_set_rtc_ui_soc(struct gauge_device *gauge_dev, int ui_soc);
extern int gauge_dev_get_rtc_ui_soc(struct gauge_device *gauge_dev, int *ui_soc);
extern int gauge_dev_is_rtc_invalid(struct gauge_device *gauge_dev, int *invalid);
extern int gauge_dev_set_reset_status(struct gauge_device *gauge_dev, int reset);
extern int gauge_dev_get_nag_vbat(struct gauge_device *gauge_dev, int *vbat);
extern int gauge_dev_enable_battery_tmp_lt_interrupt(struct gauge_device *gauge_dev, bool en, int threshold);
extern int gauge_dev_enable_battery_tmp_ht_interrupt(struct gauge_device *gauge_dev, bool en, int threshold);
extern int gauge_dev_dump(struct gauge_device *gauge_dev, struct seq_file *m);
extern int gauge_dev_get_hw_version(struct gauge_device *gauge_dev);
extern int gauge_dev_set_info(struct gauge_device *gauge_dev, enum gauge_info ginfo, int value);
extern int gauge_dev_get_info(struct gauge_device *gauge_dev, enum gauge_info ginfo, int *value);

#endif

