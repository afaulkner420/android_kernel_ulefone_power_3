#ifndef _LINUX_ELAN_KTF_H
#define _LINUX_ELAN_KTF_H

#define ELAN_X_MAX      1024
#define ELAN_Y_MAX      2112

#define LCM_X_MAX       1080
#define LCM_Y_MAX		2160

#define L2500_ADDR			0x7bd0
#define EKTF2100_ADDR		0x7bd0
#define EKTF2200_ADDR		0x7bd0
#define EKTF3100_ADDR		0x7c16
#define FW_ADDR					L2500_ADDR
extern struct tpd_device *tpd;

#define ELAN_DEBUG_ON    1

#define ELAN_INFO(fmt,arg...)           printk("<<-ELAN-INFO->> "fmt"\n",##arg)
#define ELAN_ERROR(fmt,arg...)          printk("<<-ELAN-ERROR->> "fmt"\n",##arg)

#if ELAN_DEBUG_ON
#define ELAN_DEBUG(fmt,arg...)          do{\
    						printk("<<-ELAN-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
					}while(0)
#else
#define ELAN_DEBUG(fmt,arg...)
#endif

#define elan_fw_upgrade       0
#define RLK_LEATHER_MODE      0       // add for leathermode
#define ELAN_PROC_FILE     "gtp_leather_mode"

#define ELAN_KTF_NAME "EKTH3676C"

struct elan_ktf_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
        int mode_check_gpio;
	int (*power)(int on);
};

#endif /* _LINUX_ELAN_KTF_H */

