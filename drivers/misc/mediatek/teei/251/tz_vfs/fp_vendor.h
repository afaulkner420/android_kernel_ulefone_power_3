//modify XLLSHLSS-97 by haiping.nai 20171222 start
#ifndef __FP_VENDOR_H__
#define __FP_VENDOR_H__

#include <linux/types.h>

#define MAX_TA_NAME 64

// if add a new id , sould sort by alphabetic
enum {
    FP_VENDOR_BETTERLIFE,//贝特莱
    FP_VENDOR_BIOSEC,//图正
    FP_VENDOR_CDFINGER,//费恩格尔
    FP_VENDOR_CHIPONE,//集创北方
    FP_VENDOR_CHIPSAILING,//芯启航
    FP_VENDOR_EGIS,//神盾
    FP_VENDOR_ELAN,//义隆电
    FP_VENDOR_FINCHOS,//方程式
    FP_VENDOR_FOCALTECH,//墩泰
    FP_VENDOR_FPC,// FPC
    FP_VENDOR_GOODIX,//汇鼎
    FP_VENDOR_HENGZHI,//恒智
    FP_VENDOR_HOLITECH,//合力泰
    FP_VENDOR_IMAGING,//成像通
    FP_VENDOR_LEADCORETECH,//联芯
    FP_VENDOR_MICROARRAY,//迈瑞微
    FP_VENDOR_METRICS,//茂丞
    FP_VENDOR_NOVATECH,//联咏
    FP_VENDOR_MSTAR,//MSTAR
    FP_VENDOR_RISKSTORM,//箩箕
    FP_VENDOR_SILEAD,//思立微
    FP_VENDOR_SYNAPTICS,//新思
    FP_VENDOR_SUNWAVE,//信炜
    FP_VENDOR_SUNRISE,//桑莱士
    FP_VENDOR_MAX
};

void set_fp_vendor(uint8_t fp_vendor_id);
uint8_t get_fp_vendor(void);
void get_fp_ta_load_path(char* fp_ta_load_path);
/*
 * return 1 if fp driver can call spi in ree, else return 0
*/
int get_fp_spi_enable(void);

extern int fp_spi_enable;

#endif  /*__FP_VENDOR_H__*/
//modify XLLSHLSS-97 by haiping.bai 20171222 end
