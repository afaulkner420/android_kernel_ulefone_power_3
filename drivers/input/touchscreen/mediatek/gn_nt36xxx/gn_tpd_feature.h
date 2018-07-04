#ifndef _GN_TPD_FEATURE_H_
#define _GN_TPD_FEATURE_H_

/**File Description: This is a gionee TP feature driver head file for Focaltech TouchPanel
Author: hupiing Create Date:20151109

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/compat.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
//#include <mach/mt_typedefs.h>
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
#include <linux/gn_device_check.h>
#endif

struct ges_feature_info
{
	union {
		struct{
			u16 start_x;
			u16 start_y;
			u16 end_x;
			u16 end_y;
			u16 width;
			u16 height;
			u16 mid_x;
			u16 mid_y;
			u16 top_x;
			u16 top_y;
			u16 bottom_x;
			u16 bottom_y;
			u16 left_x;
			u16 left_y;
			u16 right_x;
			u16 right_y;
		};
		u16 data[16];
	};
};

struct tpd_ges_data 
{
	u8 length;
	struct ges_feature_info f_point;
};

#define TST_BIT(flag,bit)	(flag & (0x1 << bit))
#define CLR_BIT(flag,bit)	(flag &= (~(0x1 << bit)))
#define SET_BIT(flag,bit)	(flag |= (0x1 << bit))
enum {
	TYPE_E = 0,
	TYPE_V_RIGHT ,
	TYPE_M,
	TYPE_TRIANGLE,
	TYPE_2F_UP,
	TYPE_2F_DOWN,
	TYPE_W,
	TYPE_Z,
	TYPE_CIRCLE,
	TYPE_S,
	TYPE_V_LEFT,
	TYPE_V_UP ,
	TYPE_V_DOWN ,
	TYPE_2F_LEFT,
	TYPE_2F_RIGHT,
};

//Gionee <GN_BSP_CTP> <mogongfu> <20170505> add for TP gesture define begin
#define GN_GESTURE_U_UP								183
#define GN_GESTURE_U_DOWN							184
#define GN_GESTURE_U_LEFT							185
#define GN_GESTURE_U_RIGHT							186
#define GN_GESTURE_DOUBLE_TAP							187
#define GN_GESTURE_E							188
#define GN_GESTURE_M							189
#define GN_GESTURE_TRIANGLE							190
#define GN_GESTURE_2F_UP							191
#define GN_GESTURE_2F_DOWN							192
#define GN_GESTURE_W							193
#define GN_GESTURE_Z							194
#define GN_GESTURE_O							195
#define GN_GESTURE_S							196
#define GN_GESTURE_2F_LEFT							197
#define GN_GESTURE_2F_RIGHT							198
#define GN_GESTURE_1F_SWIPE							199
//Gionee <GN_BSP_CTP> <mogongfu> <20170505> add for TP gesture define end

//Gionee <GN_BSP_CTP> <mogongfu> <20161026> add for delete gesture begin
#define GN_BSP_TP_GESTURE_SUPPORT
//Gionee <GN_BSP_CTP> <mogongfu> <20161026> add for delete gesture end
//Gionee <GN_BSP_CTP> <mogongfu> <20161026> add for tp auto test begin
#define GN_GET_TP_AUTOTEST_DATA    0
//Gionee <GN_BSP_FPR> <guomingzhi> <20170223> removed for fp nav-func begin	
//#define GN_FP_HAVE_NAV_FUNC 0
//Gionee <GN_BSP_FPR> <guomingzhi> <20170223> removed for fp nav-func end	
//Gionee <GN_BSP_CTP> <mogongfu> <20161026> add for tp auto test begin

//#define FTS_REG_FW_VER						0xA6   			// FW  version 
//#define FTS_REG_VENDOR_ID					0xA8   			// TP vendor ID 

int gn_nvt_tpd_feature_init_data(struct i2c_client *client);
int gn_nvt_tpd_feature_reinit(void);
#endif
