#ifndef _TRANSSION_CUSTOM_H
#define _TRANSSION_CUSTOM_H

#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/device.h>
#include <asm/uaccess.h>

#define TranssionCustom

#define CTP_DEBUG_ON   0

#define CTP_INFO(fmt,arg...)           printk("<<-CTP-INFO->> "fmt"\n",##arg)
#define CTP_ERROR(fmt,arg...)          printk("<<-CTP-ERROR->> "fmt"\n",##arg)
#define CTP_DEBUG(fmt,arg...)          do{\
                                         if(CTP_DEBUG_ON)\
                                         printk("<<-CTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#ifdef TranssionCustom
//gesture mode
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
//static DOZE_T doze_status = DOZE_DISABLED;
#define FTS_MAGIC_NUMBER        'G'
#define NEGLECT_SIZE_MASK           (~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GESTURE_ENABLE_TOTALLY      _IO(FTS_MAGIC_NUMBER, 1)	// 1
#define GESTURE_DISABLE_TOTALLY     _IO(FTS_MAGIC_NUMBER, 2)
#define GESTURE_ENABLE_PARTLY       _IO(FTS_MAGIC_NUMBER, 3)
#define GESTURE_DISABLE_PARTLY      _IO(FTS_MAGIC_NUMBER, 4)
#define GESTURE_DATA_OBTAIN         (_IOR(FTS_MAGIC_NUMBER, 6, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_ERASE          _IO(FTS_MAGIC_NUMBER, 7)

#define GESTURE_NODE "gesture_function"
#define GESTURE_MAX_POINT_COUNT    64

#pragma pack(1)
typedef struct {
	u8 ic_msg[6];		/*from the first byte */
	u8 gestures[4];
	u8 data[3 + GESTURE_MAX_POINT_COUNT * 4 + 80];	/*80 bytes for extra data */
} st_gesture_data;
#pragma pack()

#define SETBIT(longlong, bit)   (longlong[bit/8] |=  (1 << bit%8))
#define CLEARBIT(longlong, bit) (longlong[bit/8] &=(~(1 << bit%8)))
#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

extern u8 is_all_dead(u8 * longlong, s32 size);
extern void gesture_clear_data(void);
extern s32 setgesturestatus(unsigned int cmd,unsigned long arg);
extern s32 gesture_init_node(void);
extern void gesture_deinit_node(void);


extern int SetDozeStatus(void);
extern int GestureIdTranslate(const int gesture_id);
extern int GestureHandle(void);

#define GESTURE_LF		        0xBB
#define GESTURE_RT		        0xAA
#define GESTURE_down            0xAB
#define GESTURE_up		        0xBA
#define GESTURE_DC		        0xCC
#define GESTURE_o		        0x6F
#define GESTURE_w		        0x77
#define GESTURE_m		        0x6D
#define GESTURE_e		        0x65
#define GESTURE_c		        0x63
#define GESTURE_s		        0x73
#define GESTURE_v		        0x76
#define GESTURE_z		        0x7A

#define GESTURE_C			12
#define GESTURE_W			13
#define GESTURE_V			14
#define GESTURE_DOUBLECLICK	15
#define GESTURE_Z			16
#define GESTURE_M			17
#define GESTURE_O			18
#define GESTURE_E			19
#define GESTURE_S			20
#define GESTURE_UP		21
#define GESTURE_DOWN		22
#define GESTURE_LEFT		23
#define GESTURE_RIGHT		24

#endif
#endif
