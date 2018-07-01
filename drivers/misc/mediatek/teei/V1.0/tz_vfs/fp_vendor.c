//modify XLLSHLSS-97 by chengwenwu 20171221 start
#include<linux/kernel.h>
#include <linux/platform_device.h>
#include<linux/module.h>
#include<linux/types.h>
#include<linux/fs.h>
#include<linux/errno.h>
#include<linux/mm.h>
#include<linux/sched.h>
#include<linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include "TEEI.h"
#include "teei_id.h"
#include "fp_vendor.h"
#include <asm/uaccess.h>
#include "../tz_driver/include/backward_driver.h"
#include "../tz_driver/include/teei_client_main.h"

//#include "fp_vendor.h"
//#include <linux/types.h>
//#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
//#include <stdio.h>
#include <imsg_log.h>

#if 0
static char* __default_ta_load_path = "/vendor/thh/fp_server";
int fp_spi_enable = 1;
// if add a new id , sould sort by alphabetic
static char __fp_ta_load_path[FP_VENDOR_MAX][MAX_TA_NAME] = {
    "/vendor/thh/fp_server_betterlife",//FP_VENDOR_BETTERLIFE
    "/vendor/thh/fp_server_biosec",//FP_VENDOR_BIOSEC
    "/vendor/thh/fp_server_cdfinger",//FP_VENDOR_CDFINGER
    "/vendor/thh/fp_server_chipone",//FP_VENDOR_CHIPONE
    "/vendor/thh/fp_server_chipsailing",//FP_VENDOR_CHIPSAILING
    "/vendor/thh/fp_server_egis",//FP_VENDOR_FOCALTECH
    "/vendor/thh/fp_server_elan",//FP_VENDOR_ELAN
    "/vendor/thh/fp_server_finchos",//FP_VENDOR_FINCHOS
    "/vendor/thh/fp_server_focaltech",//FP_VENDOR_FOCALTECH
    "/vendor/thh/fp_server_fpc",//FP_VENDOR_FPC
    "/vendor/thh/fp_server_goodix",//FP_VENDOR_GOODIX
    "/vendor/thh/fp_server_hengzhi",//FP_VENDOR_HENGZHI
    "/vendor/thh/fp_server_holitech",//FP_VENDOR_HOLITECH
    "/vendor/thh/fp_server_imaging",//FP_VENDOR_IMAGING
    "/vendor/thh/fp_server_leadcoretech",//FP_VENDOR_LEADCORETECH
    "/vendor/thh/fp_server_metrics",//FP_VENDOR_METRICS
    "/vendor/thh/fp_server_microarray",//FP_VENDOR_MICROARRAY
    "/vendor/thh/fp_server_novatech",//FP_VENDOR_NOVATECH
    "/vendor/thh/fp_server_mstar",//FP_VENDOR_MSTAR
    "/vendor/thh/fp_server_riskstorm",//FP_VENDOR_RISKSTORM
    "/vendor/thh/fp_server_silead",//FP_VENDOR_SILEAD
    "/vendor/thh/fp_server_synaptics",//FP_VENDOR_SYNAPTICS
    "/vendor/thh/fp_server_sunwave",//FP_VENDOR_SUNWAVE
    "/vendor/thh/fp_server_sunrise",//FP_VENDOR_SUNRISE
};
#else
static char* __default_ta_load_path = "fp_server";
int fp_spi_enable = 1;
// if add a new id , sould sort by alphabetic
static char __fp_ta_load_path[FP_VENDOR_MAX][MAX_TA_NAME] = {
    "fp_server_betterlife",//FP_VENDOR_BETTERLIFE
    "fp_server_biosec",//FP_VENDOR_BIOSEC
    "fp_server_cdfinger",//FP_VENDOR_CDFINGER
    "fp_server_chipone",//FP_VENDOR_CHIPONE
    "fp_server_chipsailing",//FP_VENDOR_CHIPSAILING
    "fp_server_egis",//FP_VENDOR_FOCALTECH
    "fp_server_elan",//FP_VENDOR_ELAN
    "fp_server_finchos",//FP_VENDOR_FINCHOS
    "fp_server_focaltech",//FP_VENDOR_FOCALTECH
    "fp_server_fpc",//FP_VENDOR_FPC
    "fp_server_goodix",//FP_VENDOR_GOODIX
    "fp_server_hengzhi",//FP_VENDOR_HENGZHI
    "fp_server_holitech",//FP_VENDOR_HOLITECH
    "fp_server_imaging",//FP_VENDOR_IMAGING
    "fp_server_leadcoretech",//FP_VENDOR_LEADCORETECH
    "fp_server_metrics",//FP_VENDOR_METRICS
    "fp_server_microarray",//FP_VENDOR_MICROARRAY
    "fp_server_novatech",//FP_VENDOR_NOVATECH
    "fp_server_mstar",//FP_VENDOR_MSTAR
    "fp_server_riskstorm",//FP_VENDOR_RISKSTORM
    "fp_server_silead",//FP_VENDOR_SILEAD
    "fp_server_synaptics",//FP_VENDOR_SYNAPTICS
    "fp_server_sunwave",//FP_VENDOR_SUNWAVE
    "fp_server_sunrise",//FP_VENDOR_SUNRISE
};
#endif


static DEFINE_MUTEX(fp_vendor_lock);
static uint8_t __fp_vendor_id = 255;

void set_fp_vendor(uint8_t fp_vendor_id)
{
    // check param
    if (fp_vendor_id>FP_VENDOR_MAX) {
        IMSG_ERROR("%s:%d param error , fp_vendor_id=%d,FP_VENDOR_MAX=%d, fp_vendor_id>FP_VENDOR_MAX", __func__, __LINE__,  fp_vendor_id, FP_VENDOR_MAX);
        return;
    }

    // set only onece
    if (__fp_vendor_id==255) {
        mutex_lock(&fp_vendor_lock);
        __fp_vendor_id = fp_vendor_id;
        mutex_unlock(&fp_vendor_lock);
        IMSG_INFO("%s:%d set __fp_vendor_id=%d", __func__, __LINE__, __fp_vendor_id);
    } else {
        IMSG_WARN("%s:%d __fp_vendor_id already seted __fp_vendor_id = %d, return Directly", __func__, __LINE__, __fp_vendor_id);
    }
}

uint8_t get_fp_vendor()
{
    if (__fp_vendor_id>FP_VENDOR_MAX) {
        IMSG_ERROR("%s:%d param error , __fp_vendor_id=%d, __fp_vendor_id<0 || __fp_vendor_id>FP_VENDOR_MAX , return -1  ", __func__, __LINE__,  __fp_vendor_id);
        return -1;
    }

    IMSG_INFO("%s:%d get __fp_vendor_id=%d", __func__, __LINE__, __fp_vendor_id);

    return __fp_vendor_id;
}

// make sure fp_ta_load_path's len <= MAX_TA_NAME
void get_fp_ta_load_path(char* fp_ta_load_path)
{
    if (__fp_vendor_id>FP_VENDOR_MAX) {
        IMSG_ERROR("%s:%d param error , __fp_vendor_id=%d, __fp_vendor_id<0 || __fp_vendor_id>FP_VENDOR_MAX , return __default_ta_load_path=%s", __func__, __LINE__,  __fp_vendor_id, __default_ta_load_path);
        strcpy(fp_ta_load_path, __default_ta_load_path);
        return;
    }

    memcpy(fp_ta_load_path,__fp_ta_load_path[__fp_vendor_id],MAX_TA_NAME);
    IMSG_INFO("%s:%d fp_ta_load_path = %s", __func__, __LINE__, fp_ta_load_path);
}

int get_fp_spi_enable()
{
    if (fp_spi_enable==0) {
        IMSG_ERROR("%s:%d  ERROR fp_spi_enable==0", __func__, __LINE__);
    }

    IMSG_INFO("%s:%d fp_spi_enable==%d", __func__, __LINE__, fp_spi_enable);

    return fp_spi_enable;
}
//modify XLLSHLSS-97 by chengwenwu 20171221 end