/* drivers/input/touchscreen/ektf.c - ELAN EKTF verions of driver
*
* Copyright (C) 2011 Elan Microelectronics Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* 2014/0/28: The first release, version 0x0006
*             Integrated 2 ,5 ,and 10 fingers driver code together and
*             auto-mapping resolution.
*             Please change following parameters
*                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
*                    The packet size is 18 or 24 bytes.
*                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
*                    The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
*                 3. Please enable the ELAN_BUTTON configuraton to support button.
*		  4. For ektf3k serial, Add Re-Calibration Machanism
*                    So, please enable the define of RE_CALIBRATION.
*		  5. Please enable the define of ESD_CHECK, if your firmware support
*		     "I am live" packet(0x78 0x78 0x78 0x78).
*
*
*/

/* The ELAN_PROTOCOL support normanl packet format */
#define FINGER_NUM 10
#define ELAN_PROTOCOL
//#define ELAN_BUFFER_MODE
//#define ELAN_BUTTON
//#define RE_CALIBRATION   /* Re-Calibration after system resume. */
#define ELAN_RESUME_RST
#define DEVICE_NAME "elan_ktf"
#define EKTF3K_FLASH
//#define PROTOCOL_A    /* multi-touch protocol  */
#define PROTOCOL_B    /* Default: PROTOCOL B */
#define ELAN_HID_I2C	/* for hid over i2c protocol */
//#define SENSOR_OPTION
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/input.h>

#ifdef PROTOCOL_B
#include <linux/input/mt.h>
#endif
#ifdef PROTOCOL_A
#include <linux/input.h>
#endif
#include <linux/interrupt.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include "tpd.h"
//#include "mt_boot_common.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
//#include <linux/i2c/elan_ktf.h>
#include <linux/kthread.h>
#include "elan_ktf.h"
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
//#include <linux/regulator/machine.h>
//#include <linux/regulator/krait-regulator.h>
#endif
//#define ELAN_IDEL2_CMD
#ifdef ELAN_HID_I2C
#define PACKET_SIZE		67		/* support 5 fingers elan hid packet */
#else 
#define PACKET_SIZE		55		/* support 10 fingers packet for nexus7 55 */
#endif
#define FW_POS_PRESSURE		45
#define FW_POS_WIDTH		35

#define MAX_FINGER_SIZE		255
#define MAX_FINGER_PRESSURE        4095
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54
#define RESET_PKT		0x77
#define CALIB_PKT		0x66
#define IamAlive_PKT	0x78
//#define PEN_PKT			0x71
#define PEN_PKT			0x0d
#define HELLO_PKT			0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define ELAN_HID_PKT		0x3f //0x40
#define BUFFER_PKT			0x63
#define BUFFER55_PKT		0x66
static DEFINE_MUTEX(elan_tp_updata);
//add XWQYHWSYY-24 by qiang.xue 20171207 start
static DEFINE_MUTEX(cmd_done);
//add XWQYHWSYY-24 by qiang.xue 20171207 end

//Add these Define
#define IAP_PORTION
#define PAGERETRY  30
#define IAPRESTART 5

//#define ESD_CHECK
#if defined( ESD_CHECK )
static int    have_interrupts = 0;
static struct workqueue_struct *esd_wq = NULL;
static struct delayed_work      esd_work;
static unsigned long  delay = 2*HZ;

//declare function
static void elan_touch_esd_func(struct work_struct *work);
#endif
static int elan_i2c_send_data(struct i2c_client *client, uint8_t *buf, uint8_t len);
static int elan_i2c_recv_data(struct i2c_client *client, uint8_t *buf, uint8_t len);

// For Firmware Update
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_FW_INFO  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10
#define GPIO_CTP_EINT_PIN 1
#define SYSFS_MAX_LEN 100
static unsigned int gPrint_point = 0;
static int boot_normal_flag=1;
static int debug = DEBUG_TRACE;
#define touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
		printk("[elan]:" __VA_ARGS__); \
	} while (0)

static struct workqueue_struct *elan_wq;
uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
int BC_VERSION = 0x00;
int X_RESOLUTION=1344;	// nexus7 1280 1344
int Y_RESOLUTION=2240;	// nexus7 2112 2240
int FW_ID=0x00;
int work_lock=0x00;
int power_lock=0x00;
int circuit_ver=0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr=0x10;
/*++++i2c transfer end+++++++*/
struct mutex ktf_mutex;
int button_state = 0;
unsigned int touch_irq = 0;
static int suspend_flag=0;
char final_ctp_firmware_version[16] = "0x561c";

static int pen_key_flag=0;
static int pen_power_flag=0;
static int pen_open_state = 0;
int elan_key_flag=0;
int elan_pen_xmax=4276;
int elan_pen_ymax=7656;
static int plam_flag=0;
//add XWQYHWSYY-24 by qiang.xue 20171206 start
//extern u32 hall_gpio;
//add XWQYHWSYY-24 by qiang.xue 20171206 end
#if RLK_LEATHER_MODE
static struct proc_dir_entry *elan_porc;
#endif

#ifdef IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x10, 0x20, 0x21};/*I2C devices address*/
int is_OldBootCode = 0; // 0:new 1:old
//static unsigned char firmware[52800];
unsigned char ELAN_FW_DATA_LOCAL_FILENAME[30];

int (*get_psval)(void) = NULL;
#define ELAN_FW_FILENAME  "elan_fw.ekt"

#ifdef SENSOR_OPTION
//define fw name for sensor option
#define ELAN_FW_FILENAME_TG_AUO  "elan_fw_tg_auo.ekt"
#define ELAN_FW_FILENAME_TG_INX  "elan_fw_tg_inx.ekt"
#define ELAN_FW_FILENAME_HLT_AUO  "elan_fw_hlt_auo.ekt"
#define ELAN_FW_FILENAME_HLT_INX  "elan_fw_hlt_inx.ekt"
//define fw id
#define FWID_TG_AUO 0x3026
#define FWID_TG_INX 0x3027
#define FWID_HLT_AUO 0x3028
#define FWID_HLT_INX 0x3029

int FW_ID_Check = 1;  //ok = 1, fail = 0


#endif
int path_test = 0;

//fw update in driver
/*The newest firmware, if update must be changed here*/
#ifndef SENSOR_OPTION
//static uint8_t file_fw_data[] = {
//	#include "fw_data.i"
//};
#else
static uint8_t file_fw_data[] = {
	#include "fw_data.i"
};
#if 0
static uint8_t file_fw_data_TG_AUO[] = {
	#include "fw_data_tg_auo.i"
};
static uint8_t file_fw_data_TG_INX[] = {
	#include "fw_data_tg_inx.i"
};
static uint8_t file_fw_data_HLT_AUO[] = {
	#include "fw_data_hlt_auo.i"
};
static uint8_t file_fw_data_HLT_INX[] = {
	#include "fw_data_hlt_inx.i"
};
static uint8_t *file_fw_data = file_fw_data_TG_AUO;
#endif
#endif

enum
{
    PageSize		= 132,
    ACK_Fail		= 0x00,
    ACK_OK			= 0xAA,
    ACK_REWRITE		= 0x55,
};

//int PageNum = sizeof(file_fw_data)/132; /*for ektf2xxx/3xxx serial, the page number is 249/351*/
int PageNum = 0;

enum
{
    E_FD			= -1,
};
#endif
// ADD ELAN 2016-10-8
//#define ELAN_SUPPORT_I2C_DMA

extern struct pinctrl *pinctrl1;
struct pinctrl_state *avdd_output0, *avdd_output1;
#define ELAN_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define ELAN_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)
#ifdef ELAN_SUPPORT_I2C_DMA
static u8 *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa;
#define ELAN_DMA_MAX_TRANSACTION_LENGTH  2048//255
#endif

struct elan_ktf_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct workqueue_struct *elan_wq;
    struct work_struct work;
    int intr_gpio;
    int rst_gpio;
    // Firmware Information
    int fw_ver;
    int fw_id;
    int bc_ver;
    int x_resolution;
    int y_resolution;
    // For Firmare Update
    struct miscdevice firmware;
    struct wake_lock wakelock;
};

static struct elan_ktf_ts_data *private_ts;
#ifndef ELAN_HID_I2C
static int __fw_packet_handler(struct i2c_client *client);
#else
static int __fw_packet_handler_HID(struct i2c_client *client);
#endif
static int elan_ktf_ts_calibrate(struct i2c_client *client);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
void elan_ktf_ts_hw_reset(void);
#ifndef ELAN_HID_I2C
static int __hello_packet_handler(struct i2c_client *client);
#endif
static int elan_ktf_ts_get_data(struct i2c_client *, uint8_t *, uint8_t *, size_t,  size_t);

#ifdef IAP_PORTION
static int Update_FW_One(int source);
#endif

//add by zhanghui for pen switch
static ssize_t pen_switch_status_read(struct file *file,char *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
    int ret = -1;
    char *page = NULL;
    char *ptr = NULL;
	uint8_t pen_buff[67];
	int pen_switch_status;
	uint8_t pen_option_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x53,0xc1,0x00,0x01};
    if(power_lock==0){
		page = kmalloc(PAGE_SIZE, GFP_KERNEL);

		if (!page){
			kfree(page);
			return -ENOMEM;
		}
		ptr = page;
		//add XWQYHWSYY-24 by qiang.xue 20171207 start
		mutex_lock(&cmd_done);
		//add XWQYHWSYY-24 by qiang.xue 20171207 end
		disable_irq(touch_irq);
		cancel_work_sync(&private_ts->work);
		elan_i2c_send_data(private_ts->client, pen_option_cmd, sizeof(pen_option_cmd));
		//modify XWQYHWSYY-24 by qiang.xue 20171207 start
		msleep(2);
		//modify XWQYHWSYY-24 by qiang.xue 20171207 end
		ret = elan_i2c_recv_data(private_ts->client, pen_buff, 67);
		printk("[elan]pen mode:%x,%x,%x,%x,%x,%x,%x\n",pen_buff[4],pen_buff[5],pen_buff[6],pen_buff[7],pen_buff[8],pen_buff[9],pen_buff[10]);
		enable_irq(touch_irq);
		//add XWQYHWSYY-24 by qiang.xue 20171207 start
		mutex_unlock(&cmd_done);
		//add XWQYHWSYY-24 by qiang.xue 20171207 end
		pen_switch_status = (pen_buff[6]>>1)&0x1;
		printk("[elan] pen_switch: %d",pen_switch_status);

		ptr += sprintf(ptr,"%d\n",pen_switch_status);
		len = ptr - page;

		if(*ppos >= len)
			{
			kfree(page);
			return 0;
			}

		ret = copy_to_user(buffer,(char *)page,len);
		*ppos += len;
			if(ret)
			{
			kfree(page);
			return ret;
		}
		kfree(page);
	}else{
		printk("[elan]:update firmware oning\n");
	}
	return len;
}

static int switch_pen_state(int int_state){
	uint8_t pen_option_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x53,0xc1,0x00,0x01};
	uint8_t pen_buff[67];
	uint8_t cmd_len = sizeof(pen_option_cmd);
	disable_irq(touch_irq);
	cancel_work_sync(&private_ts->work);
	elan_i2c_send_data(private_ts->client, pen_option_cmd, cmd_len);
	msleep(10);
	elan_i2c_recv_data(private_ts->client, pen_buff, 67);
	msleep(10);
	printk("[elan]pen mode:%x,%x,%x,%x,%x,%x,%x\n",pen_buff[4],pen_buff[5],pen_buff[6],pen_buff[7],pen_buff[8],pen_buff[9],pen_buff[10]);
	if(int_state == 1){
		printk("[elan]:pen mode enable\n");
		pen_option_cmd[7]=0x54;
		pen_option_cmd[9]=pen_buff[6]|0x2;
		pen_option_cmd[10]=pen_buff[7];
		if(elan_i2c_send_data(private_ts->client, pen_option_cmd, cmd_len)!=cmd_len){
			printk(" %s and int_state is %d failed open xpen\n", __func__, int_state);
			enable_irq(touch_irq);
			return -1;
		}
	}else if(int_state == 0){
		printk("[elan]:pen mode disable\n");
		pen_option_cmd[7]=0x54;
		pen_option_cmd[9]=pen_buff[6]&0xfd;
		pen_option_cmd[10]=pen_buff[7];
		if(elan_i2c_send_data(private_ts->client, pen_option_cmd, cmd_len)!=cmd_len){
			printk(" %s and int_state is %d failed open xpen\n", __func__, int_state);
			enable_irq(touch_irq);
			return -1;
		}
	}
	enable_irq(touch_irq);
	return 0;
}

static ssize_t pen_switch_status_write(struct file *filp, const char * buf, size_t len, loff_t * off)
{
        char buff[3];
		int ret = -1;
        ret = copy_from_user(buff, buf, sizeof(buff));
        sscanf(buff,"%d",&pen_open_state);
		printk("[elan]:pen enable:%d\n",pen_open_state);
		if(power_lock==0){
			//add XWQYHWSYY-24 by qiang.xue 20171207 start
			mutex_lock(&cmd_done);
			//add XWQYHWSYY-24 by qiang.xue 20171207 end
			switch_pen_state(pen_open_state);
			//add XWQYHWSYY-24 by qiang.xue 20171207 start
			mutex_unlock(&cmd_done);
			//add XWQYHWSYY-24 by qiang.xue 20171207 end
		}
		else
			printk("[elan]:update firmware oning\n");
		return len;
}

static const struct file_operations pen_switch = {
         .owner = THIS_MODULE,
         .read = pen_switch_status_read,
         .write = pen_switch_status_write,
};
//add by zhanghui for pen switch end

static int __elan_ktf_ts_poll(struct i2c_client *client)
{
    //struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
    int status = 0, retry = 10;

    do {
        status = gpio_get_value(GPIO_CTP_EINT_PIN);
        if(status==0) break;
		
        touch_debug(DEBUG_MESSAGES, "%s: status = %d\n", __func__, status);
		mdelay(50);
        retry--;
    } while (status == 1 && retry > 0);
    touch_debug(DEBUG_INFO, "[elan]%s: poll interrupt status %s\n", __func__, status == 1 ? "high" : "low");
    return (status == 0 ? 0 : -ETIMEDOUT);

    return 0;
}

static int elan_ktf_ts_poll_packet(struct i2c_client *client)
{
    //struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
    int status = 0, retry = 3;

    do {
        status = gpio_get_value(GPIO_CTP_EINT_PIN);
        if(status==0) break;

        touch_debug(DEBUG_MESSAGES, "%s: status = %d\n", __func__, status);
		mdelay(5);
        retry--;
    } while (status == 1 && retry > 0);
    touch_debug(DEBUG_INFO, "[elan]%s: poll interrupt status %s\n", __func__, status == 1 ? "high" : "low");
    return (status == 0 ? 0 : -ETIMEDOUT);

    return 0;
}

static int elan_ktf_ts_poll(struct i2c_client *client)
{
    return __elan_ktf_ts_poll(client);
}

/************************************
* Restet TP
*************************************/
void elan_ktf_ts_hw_reset()
{
	printk("[elan]:TP RESET!\n");
	ELAN_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(20);	
	ELAN_GPIO_OUTPUT(GTP_RST_PORT, 1);	
	msleep(5);
}

static int elan_i2c_recv_data(struct i2c_client *client, uint8_t *buf, uint8_t len)
{
	int rc = 0;
//	int i = 0;

#ifdef ELAN_SUPPORT_I2C_DMA	
	if(buf == NULL || gpDMABuf_va == NULL){
		printk("[elan] BUFFER is NULL!!!!!\n");
		return -1;
	}
	
	memset(buf, 0, len);
	
	rc = i2c_master_recv(client, (u8 *)(uintptr_t)gpDMABuf_pa, len);
	
	if(rc >= 0){
	    for(i = 0 ; i < len; i++){
			buf[i] = gpDMABuf_va[i];
			//printk("%02x ", buf[i]);
		}
	}
	
#else
	rc = i2c_master_recv(client, buf, len);
#endif
	
	return rc;
}

static int elan_i2c_send_data(struct i2c_client *client, uint8_t *buf, uint8_t len)
{
	int rc = 0;
//	int i = 0;
	
#ifdef ELAN_SUPPORT_I2C_DMA	
	if(buf == NULL || gpDMABuf_va == NULL){
		printk("[elan] BUFFER is NULL!!!!!\n");
		return -1;
	}
	
	for(i = 0 ; i < len; i++){
		gpDMABuf_va[i] = buf[i];
		//printk("%02x ", buf[i]);
	}
	
	rc = i2c_master_send(client,(u8 *)(uintptr_t)gpDMABuf_pa, len);	
	
#else
	rc = i2c_master_send(client, buf, len);
#endif
	return rc;
}

// For Firmware Update
int elan_iap_open(struct inode *inode, struct file *filp) {
    touch_debug(DEBUG_MESSAGES, "[elan]into elan_iap_open\n");
    if (private_ts == NULL)  touch_debug(DEBUG_ERROR,"private_ts is NULL\n");

    return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp) {
    return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp) {
    int ret;
    char *tmp;
    touch_debug(DEBUG_MESSAGES, "[elan]into elan_iap_write\n");

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
    ret = elan_i2c_send_data(private_ts->client, tmp, count);

    //if (ret != count) printk("elan i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    //return ret;
    return (ret == 1) ? count : ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp) {
    char *tmp;
    int ret;
    long rc;
    touch_debug(DEBUG_MESSAGES,"[elan]into elan_iap_read\n");

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    ret = elan_i2c_recv_data(private_ts->client, tmp, count);

    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);

    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;

}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {

    int __user *ip = (int __user *)arg;

    touch_debug(DEBUG_MESSAGES,"[elan]into elan_iap_ioctl\n");
    touch_debug(DEBUG_MESSAGES,"cmd value %x\n",cmd);

    switch (cmd) {
    case IOCTL_I2C_SLAVE:
  //      private_ts->client->addr = (int __user)arg;
//		private_ts->client->addr=((private_ts->client->addr)&I2C_MASK_FLAG)|I2C_DMA_FLAG|I2C_ENEXT_FLAG;
        break;
    case IOCTL_FW_INFO:
		__fw_packet_handler_HID(private_ts->client);
        break;
    case IOCTL_MINOR_FW_VER:
        break;
    case IOCTL_RESET:
        elan_ktf_ts_hw_reset();
        break;
    case IOCTL_IAP_MODE_LOCK:
        if(work_lock==0)
        {
            work_lock=1;
            disable_irq(touch_irq);
            cancel_work_sync(&private_ts->work);
#if defined( ESD_CHECK )
            cancel_delayed_work_sync( &esd_work );
#endif
        }
        break;
    case IOCTL_IAP_MODE_UNLOCK:
        if(work_lock==1)
        {
            work_lock=0;
            enable_irq(touch_irq);
#if defined( ESD_CHECK )  //0604
            queue_delayed_work( esd_wq, &esd_work, delay );
#endif
        }
        break;
    case IOCTL_CHECK_RECOVERY_MODE:
        return RECOVERY;
        break;
    case IOCTL_FW_VER:
		//__fw_packet_handler(private_ts->client);
        return FW_VERSION;
        break;
    case IOCTL_X_RESOLUTION:
		//__fw_packet_handler(private_ts->client);
        return X_RESOLUTION;
        break;
    case IOCTL_Y_RESOLUTION:
		//__fw_packet_handler(private_ts->client);
        return Y_RESOLUTION;
        break;
    case IOCTL_FW_ID:
		//__fw_packet_handler(private_ts->client);
        return FW_ID;
        break;
	case IOCTL_BC_VER:
		return BC_VERSION;
		break;
    case IOCTL_ROUGH_CALIBRATE:
        return elan_ktf_ts_calibrate(private_ts->client);
    case IOCTL_I2C_INT:
        put_user(gpio_get_value(GPIO_CTP_EINT_PIN), ip);
        break;
    case IOCTL_RESUME:
        //elan_ktf_ts_resume(private_ts->client);
        break;
    case IOCTL_POWER_LOCK:
        power_lock=1;
        break;
    case IOCTL_POWER_UNLOCK:
        power_lock=0;
        break;
#ifdef IAP_PORTION
    case IOCTL_GET_UPDATE_PROGREE:
        update_progree=(int __user)arg;
        break;
    case IOCTL_FW_UPDATE:
        //read_test();
        Update_FW_One(0);
        break;
#endif
    case IOCTL_CIRCUIT_CHECK:
        return circuit_ver;
        break;
    default:
        touch_debug(DEBUG_ERROR,"[elan] Un-known IOCTL Command %d\n", cmd);
        break;
    }
    return 0;
}

struct file_operations elan_touch_fops = {
    .open =         elan_iap_open,
    .write =        elan_iap_write,
    .read = 	elan_iap_read,
    .release =	elan_iap_release,
    .unlocked_ioctl=elan_iap_ioctl,
	.compat_ioctl=elan_iap_ioctl,
};



#ifdef IAP_PORTION

int EnterISPMode(struct i2c_client *client)
{
    int len = 0;
    uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50}; //{0x45, 0x49, 0x41, 0x50};
    len = i2c_master_send(private_ts->client, isp_cmd,  4);
    if (len != 4) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: EnterISPMode fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
    return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
    int len = 0;

    len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
    if (len != byte)
    {
        touch_debug(DEBUG_ERROR,"[elan] %s: read page error, read error. len=%d\r\n", __func__, len);
        return -1;
    }

    return 0;
}

int WritePage( uint8_t * szPage, int byte)
{
    int len = 0;
	mutex_lock(&elan_tp_updata);
    len = elan_i2c_send_data(private_ts->client, szPage,  byte);
    if (len != byte)
    {
        touch_debug(DEBUG_ERROR,"[elan] %s: write page error, write error. len=%d\r\n", __func__, len);
        return -1;
    }
	mutex_unlock(&elan_tp_updata);
    return 0;
}

int GetAckData(struct i2c_client *client)
{
    int rc = 0;

    uint8_t buff[67] = {0};
#ifdef ELAN_HID_I2C
    rc = elan_ktf_ts_poll(client);
    //msleep(20);
#endif
    rc = elan_i2c_recv_data(private_ts->client, buff, sizeof(buff));
    if (rc != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] %s: Read ACK Data error. rc=%d\r\n", __func__, rc);
        return -1;
    }

    touch_debug(DEBUG_MESSAGES, "[elan] %s: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",__func__,buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9],buff[10],buff[11]);
	if (buff[4] == 0xaa && buff[5] == 0xaa)
		return ACK_OK;
	else
		return -1;

}

void print_progress(int page, int ic_num, int j)
{
    int i, percent,page_tatol=351,percent_tatol;
    char str[256];
    str[0] = '\0';
    for (i=0; i<((page)/10); i++) {
        str[i] = '#';
        str[i+1] = '\0';
    }

    percent = ((100*page)/(PageNum));
    if ((page) == (PageNum))
        percent = 100;

    if ((page_tatol) == (PageNum*ic_num))
        percent_tatol = 100;

    touch_debug(DEBUG_INFO, "\r[elan]progress %s| %d %d", str, percent, page);

    if (page == (PageNum))
        touch_debug(DEBUG_INFO, "\n");

}
//add for leather_mode xuzhou.li
#if RLK_LEATHER_MODE
static int ftp_leather_mode_status = 0;

// add /proc/gtp_leather_mode
static ssize_t elan_leather_mode_write_proc(struct file *file,const char *buffer, size_t count, loff_t *ppos)
{
	int my_status = 0;
	int num = 0;
    char temp[50]; // for store special format cmd
	uint8_t enable_cover_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0xbf,0x00,0x01};
	uint8_t disable_cover_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0xbf,0x00,0x00};
	struct i2c_client *client = private_ts->client;
    if (copy_from_user(temp, buffer, sizeof(temp)))
		{
        TPD_DEBUG("copy from user fail 2");
        return -EFAULT;
    }

	num = sscanf(temp,"%d",&my_status);
	if(1 == num){
		TPD_DEBUG("[elan] Recive from user %d:%d:%d:%d\n",__LINE__,*(int*)buffer,num,my_status);
		if(1==my_status) {
			ftp_leather_mode_status = my_status;
			if ((elan_i2c_send_data(client, enable_cover_cmd, sizeof(enable_cover_cmd))) != sizeof(enable_cover_cmd)) {
				printk("[elan] i2c_master_send enable_cover_cmd failed in file\n");
				msleep(10);
				elan_i2c_send_data(client, enable_cover_cmd, sizeof(enable_cover_cmd));
			}
			else
				printk("[elan]enable cover mode suss in file\n");
		}
		else{
			ftp_leather_mode_status = my_status;
			if ((elan_i2c_send_data(client, disable_cover_cmd, sizeof(disable_cover_cmd))) != sizeof(disable_cover_cmd)) {
				printk("[elan] i2c_master_send disable_cover_cmd failed in file\n");
				msleep(10);
				elan_i2c_send_data(client, disable_cover_cmd, sizeof(disable_cover_cmd));
			}
			else
				printk("[elan]disable cover mode suss in file\n");
		}
	}
	else{
		TPD_DEBUG("[elan] Recive from user %d:%d:%d:%d\n",__LINE__,*(int*)buffer,num,my_status);
		return -EINVAL;
	}
	return count;
}

static ssize_t elan_leather_mode_read_proc(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	int len,ret = -1;
	char *page = NULL;
	char *ptr = NULL;
	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	  if (!page)
	  {
		kfree(page);
		return -ENOMEM;
	  }
	ptr = page;
	ptr += sprintf(ptr,"%d\n",ftp_leather_mode_status);
	len = ptr - page;
	  if(*ppos >= len)
	  {
		  kfree(page);
		  return 0;
	  }
	  ret = copy_to_user(buffer,(char *)page,len);
	  *ppos += len;
	  if(ret)
	  {
		kfree(page);
		return ret;
	  }
	  kfree(page);
	  return len;
}

static const struct file_operations gt_leather_mode_proc_fops = {
    .write = elan_leather_mode_write_proc,
    .read = elan_leather_mode_read_proc,
};
#endif
//add end
/*	fw_source = 0: update fw_data.i compile with driver code
	fw_source = 1: update fw at /system/etc/firmware/elan_fw.ekt
	fw_source = 2: update ekt file at /data/local/tmp/ElanFW.ekt       */
static int Update_FW_One(int fw_source)
{
#if 0
    int res = 0,ic_num = 1;
    int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
    int i = 0;
    uint8_t data;
	//struct timeval tv1, tv2;
	// for open user space file
	int pos=0;
	struct file *firmware_fp;
	mm_segment_t oldfs;

    uint8_t boot_buffer[4] = {0};
    int byte_count = 0;

    uint8_t *szBuff = NULL;
    int curIndex = 0;
    struct firmware *p_fw_entry;
    uint8_t *fw_data;
    int rc, fw_size;
    unsigned char fw_local_path[50];
	uint8_t cal_cmd[] = {0x54, 0x29, 0x00, 0x01}; 
	uint8_t flash_key[] = {0x54, 0xC0, 0xE1, 0x5A};
#ifdef SENSOR_OPTION
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	int major, minor;
	uint8_t buf_recv[4] = {0};
	int sensor_FW_ID = 0x00;
#endif
    mutex_lock(&ktf_mutex);
	touch_debug(DEBUG_INFO, "[elan] %s: Update FW\n", __func__);

IAP_RESTART:

    curIndex=0;
    data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);

    if(RECOVERY != 0x80)
    {
        touch_debug(DEBUG_MESSAGES, "[elan] Firmware upgrade normal mode !\n");
    } else
        touch_debug(DEBUG_MESSAGES, "[elan] Firmware upgrade recovery mode !\n");

	/*check where fw is coming from; 0 = fw_data.i; 1 = request_fw*/
    if(fw_source == 1) { 	//use request firmware
#ifndef SENSOR_OPTION
		touch_debug(DEBUG_INFO, "[elan] request_firmware name = %s\n",ELAN_FW_FILENAME);
        rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &private_ts->client->dev);
#else		
		touch_debug(DEBUG_ERROR, "[elan] ---request firmware.\n");
		/*sensor option start 20160622*/
		rc = elan_ktf_ts_get_data(private_ts->client, cmd_id, buf_recv, 4,4);
		if (rc < 0)
		{
			printk("Get Firmware ID error, exit %s.\n", __func__);
			return 0;
		}
		major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
		sensor_FW_ID = major << 8 | minor;
		touch_debug(DEBUG_ERROR, "[elan] %s sensor_FW_ID = 0x%4.4x\n", __func__, sensor_FW_ID);
	
		if(sensor_FW_ID == FWID_TG_AUO)
		{
			touch_debug(DEBUG_INFO, "[elan] request_firmware name = %s\n",ELAN_FW_FILENAME_TG_AUO);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_TG_AUO, &private_ts->client->dev);
			path_test = 3;
		}
		else if(sensor_FW_ID == FWID_TG_INX)
		{
			touch_debug(DEBUG_INFO, "[elan] request_firmware name = %s\n",ELAN_FW_FILENAME_TG_INX);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_TG_INX, &private_ts->client->dev);
			path_test = 4;
		}
		else if(sensor_FW_ID == FWID_HLT_AUO)
		{
			touch_debug(DEBUG_INFO, "[elan] request_firmware name = %s\n",ELAN_FW_FILENAME_HLT_AUO);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_HLT_AUO, &private_ts->client->dev);
			path_test = 5;
		}
		else if(sensor_FW_ID == FWID_HLT_INX)
		{
			touch_debug(DEBUG_INFO, "[elan] request_firmware name = %s\n",ELAN_FW_FILENAME_HLT_INX);
			rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME_HLT_INX, &private_ts->client->dev);
			path_test = 6;
		}
		else
		{
			path_test = 7;
			touch_debug(DEBUG_INFO, "[elan] %s: FW_ID_Check Not Match, Don't update fw!\n", __func__);
			mutex_unlock(&ktf_mutex);
            return -1;
		}
		/* sensor option end 20160622 */
#endif
        if (rc != 0) {
            touch_debug(DEBUG_ERROR,"[elan] rc=%d, request_firmware fail\n", rc);
			mutex_unlock(&ktf_mutex);
            return -1;
        }
        else
            printk(KERN_DEBUG "[elan] Request Firmware Size=%zu\n", p_fw_entry->size);

        fw_data = p_fw_entry->data;
        fw_size = p_fw_entry->size;
        PageNum = (fw_size/sizeof(uint8_t)/PageSize);
    }
	else if(fw_source == 2) {  //data/local/tmp
		//Note: If want fix file name, please replace <file name>
		//sprintf(ELAN_FW_DATA_LOCAL_FILENAME,"%s","<file name>");
		sprintf(fw_local_path,"%s%s","/data/local/tmp/",ELAN_FW_DATA_LOCAL_FILENAME);
		printk(KERN_DEBUG "[elan] Update Firmware from %s\n",fw_local_path);
		oldfs=get_fs();
		set_fs(KERNEL_DS);
		firmware_fp = filp_open(fw_local_path, O_RDONLY, S_IRUSR |S_IRGRP);
		if(PTR_ERR(firmware_fp) == -ENOENT) {
			touch_debug(DEBUG_ERROR, "[elan] open file error.\n");
			set_fs(oldfs);
    		mutex_unlock(&ktf_mutex);
			return -1;
		}else
			touch_debug(DEBUG_ERROR, "[elan] open file success.\n");
		PageNum=0;
		firmware_fp->f_pos = 0;
		//touch_debug(DEBUG_ERROR, "[elan]test1.\n");
		
		for(pos = 0; pos < 1000*132; pos += 132,PageNum++) {
			if(firmware_fp->f_op->read(firmware_fp, firmware + pos, 132, &firmware_fp->f_pos) != 132) {
				//touch_debug(DEBUG_ERROR, "[elan] break!!!!\n");
				break;
			}
			//touch_debug(DEBUG_ERROR, "[elan]pos = %d, PageNum = %d\n", pos, PageNum);
		}
		//touch_debug(DEBUG_ERROR, "[elan]test2.\n");
		fw_data = firmware;
		//touch_debug(DEBUG_INFO, "[elan]%s: PageNUM %d, FW_Ver %x %x, FW_ID %x %x\n",__func__,PageNum,firmware[34058],firmware[34059],firmware[34586],firmware[34587]);
		//touch_debug(DEBUG_INFO, "[elan]%s: %s, PageNUM = %d.\n",__func__, ELAN_FW_DATA_LOCAL_FILENAME, PageNum);
		touch_debug(DEBUG_INFO, "[elan]%s: %s = %d, \n",__func__, ELAN_FW_DATA_LOCAL_FILENAME, PageNum);
		set_fs(oldfs);
		filp_close(firmware_fp, NULL);
		
		//test
		//touch_debug(DEBUG_INFO, "[elan]%s: test3, return 1\n",__func__);
		//mutex_unlock(&ktf_mutex);
		//return 1;
	}
    else {		//use fw_data.i
#ifndef SENSOR_OPTION
		printk(KERN_DEBUG "[elan] Update Firmware by fw_data.i\n");
        PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
        fw_data = file_fw_data;
#else		
		if(FW_ID_Check) //FW ID is checked OK
		{
			printk(KERN_DEBUG "[elan] Update Firmware by fw_data.i\n");
			PageNum = (sizeof(file_fw_data_TG_AUO)/sizeof(uint8_t)/PageSize);
			fw_data = file_fw_data;
			path_test = path_test + 100;
		}
		else	//FW ID check fail
		{
			path_test = 8;
			touch_debug(DEBUG_ERROR, "[elan] %s: FW_ID_Check Not Match, Don't update fw.\n", __func__);
			mutex_unlock(&ktf_mutex);
			return -1;
		}
#endif
    }
    printk(KERN_DEBUG "[elan] PageNum = %d.\n", PageNum);
    /*end check firmware*/

    /*Send enter bootcode cmd*/
    elan_ktf_ts_hw_reset();
    mdelay(25);
    res = EnterISPMode(private_ts->client);	 //enter ISP mode
    //elan_ktf_ts_poll(private_ts->client);
    mdelay(100);
    /*check enter bootcode cmd*/
    res = i2c_master_recv(private_ts->client, boot_buffer, 4);   //55 aa 33 cc
    touch_debug(DEBUG_MESSAGES, "[elan] %s :%x,%x,%x,%x\n",__func__,boot_buffer[0],boot_buffer[1],boot_buffer[2],boot_buffer[3]);

    /* Send Dummy Byte	*/
    res = i2c_master_send(private_ts->client, &data,  sizeof(data));
    if(res!=sizeof(data))
    {
        touch_debug(DEBUG_ERROR, "[elan] dummy error code = %d\n",res);
    }
    else
        touch_debug(DEBUG_ERROR, "[elan] Send Dummy Byte Success!!\n");

    /* Start IAP*/
    for( iPage = 1; iPage <= PageNum; iPage++ )
    {
#if 1 // 8byte mode
        // 8 bytes
        //szBuff = fw_data + ((iPage-1) * PageSize);
        for(byte_count=1; byte_count<=17; byte_count++)
        {
            //touch_debug(DEBUG_INFO, "[elan] byte %d, curIndex = %d\n", byte_count, curIndex );
            szBuff = fw_data + curIndex;
            if(byte_count!=17)
            {
                //printk("[elan] byte %d\n",byte_count);
                //printk("curIndex =%d\n",curIndex);

                curIndex =  curIndex + 8;

                //ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
                res = WritePage(szBuff, 8);
            }
            else
            {
                //printk("[elan] byte %d\n",byte_count);
                //printk("curIndex =%d\n",curIndex);
                curIndex =  curIndex + 4;
                //ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
                res = WritePage(szBuff, 4);
            }
        } // end of for(byte_count=1;byte_count<=17;byte_count++)
		//printk("[elan] iPage = %d\n",iPage);
#endif
#if 0 // 132byte mode		
        //szBuff = fw_data + curIndex;
        //szBuff = firmware + curIndex;
        szBuff = fw_data + curIndex;
        curIndex =  curIndex + PageSize;
        res = WritePage(szBuff, PageSize);
#endif
#if 1 //if use old bootcode
        if(iPage==PageNum || iPage==1)
        {
            mdelay(600);
        }
        else
        {
            mdelay(50);
        }
#endif

        res = GetAckData(private_ts->client);

        if (ACK_OK != res)
        {
            mdelay(50);
            touch_debug(DEBUG_ERROR, "[elan] ERROR: GetAckData fail! res=%d\r\n", res);
            rewriteCnt = rewriteCnt + 1;
            if (rewriteCnt == PAGERETRY)
            {
                touch_debug(DEBUG_ERROR, "[elan] %dth page ReWrite %d times fails!\n", iPage, PAGERETRY);
                mutex_unlock(&ktf_mutex);
                return E_FD;
            }
            else
            {
                touch_debug(DEBUG_ERROR, "[elan] %d page ReWrite %d times!\n",  iPage, rewriteCnt);
                goto IAP_RESTART;
            }
        }
        else
        {
            rewriteCnt = 0;
            print_progress(iPage,ic_num,i);
            if (iPage == PageNum)
				printk("[elan] %s Firmware Update Successfully!\n", __func__);
        }
    } // end of for(iPage = 1; iPage <= PageNum; iPage++)

    elan_ktf_ts_hw_reset();   //20160114 added, to reset ic
    mdelay(150);
    RECOVERY=0;
	
#ifndef ELAN_HID_I2C	
    res = __hello_packet_handler(private_ts->client);

    if(res == 0x80)
        touch_debug(DEBUG_ERROR, "[elan] Recovery mode! res = %02x.\n", res);
    else
        touch_debug(DEBUG_ERROR, "[elan] hello_packet_handler() res = %d.\n", res);

        res = __fw_packet_handler(private_ts->client);
#else
		res = __fw_packet_handler_HID(private_ts->client);
#endif
    touch_debug(DEBUG_ERROR, "[elan] __fw_packet_handler() res = %d.\n", res);
    if (res < 0)
        touch_debug(DEBUG_ERROR, "[elan] res = %d, Get Firmware information error, maybe received by system interrupt!\n", res);
#if 1
	//Re-calibration start 20160315
	mdelay(600);
	touch_debug(DEBUG_INFO, "[elan] %s: Re-calibration start\n", __func__);

	if ((i2c_master_send(private_ts->client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) 
	{
		touch_debug(DEBUG_ERROR, "[elan] i2c_master_send flash_key failed\n");
    }

	if ((i2c_master_send(private_ts->client, cal_cmd, sizeof(cal_cmd))) != sizeof(cal_cmd)) 
	{
		touch_debug(DEBUG_ERROR, "[elan] i2c_master_send cal_cmd failed\n");
    }
	//Re calibration end
#endif	
    touch_debug(DEBUG_ERROR, "[elan] fw_update end.\n");
#ifdef SENSOR_OPTION
	touch_debug(DEBUG_ERROR, "[elan] !! path_test = %d.\n", path_test);
	path_test = 0;
#endif	
    mutex_unlock(&ktf_mutex);
    return res;   /* 0:sucessfully, 0x80: Recovery, -1: No response */
#endif
    return 0;
}

#endif
// End Firmware Update

#ifdef ELAN_HID_I2C
int SendEndCmd(struct i2c_client *client)
{
    int len = 0;
    uint8_t send_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x1A};
    len = elan_i2c_send_data(private_ts->client, send_cmd, sizeof(send_cmd));
    if (len != sizeof(send_cmd)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Send Cmd fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n", send_cmd[0], send_cmd[1], send_cmd[2], send_cmd[3], send_cmd[4], send_cmd[5]);

    return 0;
}

int HID_EnterISPMode(struct i2c_client *client)
{
    int len = 0;
    int j;
    uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a};
    uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34};
    uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10};
    uint8_t buff[67] = {0};


    len = elan_i2c_send_data(private_ts->client, flash_key,  37);
    if (len != 37) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Flash key fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

    mdelay(20);

    len = elan_i2c_send_data(private_ts->client, isp_cmd,  37);
    if (len != 37) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: EnterISPMode fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);


    mdelay(20);
    len = elan_i2c_send_data(private_ts->client, check_addr,  sizeof(check_addr));
    if (len != sizeof(check_addr)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", check_addr[7], check_addr[8], check_addr[9], check_addr[10]);

    mdelay(20);

    len=elan_i2c_recv_data(private_ts->client, buff, sizeof(buff));
    if (len != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address Read Data error. len=%d \r\n", len);
        return -1;
    }
    else {
        printk("[Check Addr]: ");
        for (j=0; j<37; j++)    //j<37
            printk("%x ", buff[j]);
        printk("\n");

    }

    return 0;
}

int HID_RecoveryISP(struct i2c_client *client)
{
    int len = 0;
    int j;
    uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a};
//	uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34};
    uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10};
    uint8_t buff[67] = {0};

    len = elan_i2c_send_data(private_ts->client, flash_key,  37);
    if (len != 37) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Flash key fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

    mdelay(40);
  
    len = elan_i2c_send_data(private_ts->client, check_addr,  sizeof(check_addr));
    if (len != sizeof(check_addr)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", check_addr[7], check_addr[8], check_addr[9], check_addr[10]);

    mdelay(20);
    len=elan_i2c_recv_data(private_ts->client, buff, sizeof(buff));
    if (len != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address Read Data error. len=%d \r\n", len);
        return -1;
    }
    else {
        printk("[elan][Check Addr]: ");
        for (j=0; j<10; j++) //j<37
            printk("%x ", buff[j]);
        printk("\n");

    }

    return 0;
}
int CheckISPstatus(struct i2c_client *client)
{
    int len = 0;
    int j;
    uint8_t checkstatus[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x18};
    uint8_t buff[67] = {0};

    len = elan_i2c_send_data(private_ts->client, checkstatus, sizeof(checkstatus));
    if (len != sizeof(checkstatus)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Flash key fail! len=%d\r\n", len);
        return -1;
    }
    else
        touch_debug(DEBUG_MESSAGES,"[elan] check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n", checkstatus[0], checkstatus[1], checkstatus[2], checkstatus[3], checkstatus[4], checkstatus[5]);

    mdelay(10);
    len=elan_i2c_recv_data(private_ts->client, buff, sizeof(buff));
    if (len != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address Read Data error. len=%d \r\n", len);
        return -1;
    }
    else {
        printk("[elan][Check status]: ");
        for (j=0; j<10; j++) //j<37
            printk("%x ", buff[j]);
        printk("\n");
        if (buff[6] == 0xa6)	return 0xa6; /* return recovery mode 0x88 */
    }

    return 0;
}
/*	fw_source = 0: update fw_data.i compile with driver code
	fw_source = 1: update fw at /system/etc/firmware/elan_fw.ekt
	fw_source = 2: update ekt file at /data/local/tmp/ElanFW.ekt       */
#if 0
static int HID_FW_Update(int fw_source)
{
    int res = 0,ic_num = 1;
    int iPage = 0; /* rewriteCnt = 0; rewriteCnt for PAGE_REWRITE */
    int j = 0; // i=0;
    int write_times = 142;
    //int write_bytes = 12;
    uint8_t data;
    //int restartCnt = 0; // For IAP_RESTART
    int byte_count;
    uint8_t *szBuff = NULL;
    //u8 write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x21, 0x00, 0x00, 0x28};
    u8 write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x21, 0x00, 0x00, 0x1c};
    u8 cmd_iap_write[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x22};
    int curIndex = 0;
    int offset = 0;
    // 0x54, 0x00, 0x12, 0x34
    int rc;//fw_size;
	// for open user space file
    int pos=0;
    struct file *firmware_fp;
    mm_segment_t oldfs;
    unsigned char fw_local_path[50];

    /* Star Request Firmware */
     uint8_t *fw_data;
     const struct firmware *p_fw_entry;
	
    mutex_lock(&ktf_mutex);
    printk("[elan] enter %s.\n",__FUNCTION__);
    work_lock=1;
    disable_irq(touch_irq);
        //cancel_work_sync(&ts->work);
    power_lock = 1;

    if(fw_source == 1) { 	//use request firmware
		touch_debug(DEBUG_INFO, "Request_firmware name = %s\n",ELAN_FW_FILENAME);
		rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &private_ts->client->dev);
		if (rc != 0) {
			touch_debug(DEBUG_ERROR,"rc=%d, Request_firmware fail\n", rc);
			mutex_unlock(&ktf_mutex);
			return -1;
		} else
			PageNum=479; /* 63228/132=479 */
			//PageNum=473;
		touch_debug(DEBUG_INFO,"Firmware Size=%zu, PageNum=%d\n", p_fw_entry->size,PageNum);
		
		//fw_data = p_fw_entry->data;
		//fw_size = p_fw_entry->size;
		
     }/* End Request Firmware */
     else if(fw_source == 2) {  //data/local/tmp
		sprintf(fw_local_path,"%s%s","/data/local/tmp/",ELAN_FW_DATA_LOCAL_FILENAME);
		printk(KERN_DEBUG "[elan] Update Firmware from %s\n",fw_local_path);
#if 1
		oldfs=get_fs();
		set_fs(KERNEL_DS);
		firmware_fp = filp_open(fw_local_path, O_RDONLY, S_IRUSR |S_IRGRP);
		if(PTR_ERR(firmware_fp) == -ENOENT) {
			touch_debug(DEBUG_ERROR, "[elan] open file error.\n");
			set_fs(oldfs);
    		mutex_unlock(&ktf_mutex);
			return -1;
		}else
			touch_debug(DEBUG_ERROR, "[elan] open file success.\n");
		PageNum=0;
		firmware_fp->f_pos = 0;
		//touch_debug(DEBUG_ERROR, "[elan]test1.\n");
		
		for(pos = 0; pos < 1000*132; (pos += 132,PageNum++)) {
			//PageNum++;
			if(firmware_fp->f_op->read(firmware_fp, firmware + pos, 132, &firmware_fp->f_pos) != 132) {
				//touch_debug(DEBUG_ERROR, "[elan] break!!!!\n");
				break;
			}
			//touch_debug(DEBUG_ERROR, "[elan]pos = %d, PageNum = %d\n", pos, PageNum);
		}
		//touch_debug(DEBUG_ERROR, "[elan]test2.\n");
		fw_data = firmware;

		touch_debug(DEBUG_INFO, "[elan]%s: %s, PageNum = %d, \n",__func__, ELAN_FW_DATA_LOCAL_FILENAME, PageNum);
		set_fs(oldfs);
		filp_close(firmware_fp, NULL);
#endif
	}
	else {		//use fw_data.i
		printk(KERN_DEBUG "[elan] Update Firmware by fw_data.i\n");
        	PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
		touch_debug(DEBUG_ERROR, "[elan] PageNum = %d.\n", PageNum);
        	fw_data = file_fw_data;
	}
		
    touch_debug(DEBUG_INFO, "[elan] %s:  ic_num=%d\n", __func__, ic_num);

    data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);

    elan_ktf_ts_hw_reset();
    msleep(200);

    res = CheckISPstatus(private_ts->client);
	printk("[elan zhanghui ] res: 0x%x\n",res);
  //  mdelay(20);
    if (res == 0xa6) { /* 0x88 recovery mode  */
        elan_ktf_ts_hw_reset();
        msleep(200);
        printk("[elan hid iap] Recovery mode\n");
        res = HID_RecoveryISP(private_ts->client);
    }
    else {
        printk("[elan hid iap] Normal mode\n");
        res = HID_EnterISPMode(private_ts->client);   //enter HID ISP mode
    }
    msleep(50);
	
    // Start HID IAP
    //for( iPage = 1; iPage <= 473; iPage +=30 )
    for( iPage = 1; iPage <= PageNum; iPage +=30 )
    {
        offset=0;
        if (iPage == 451 ) {
            //write_times = 109;
			
			write_times = 137;  //29*132=3828, 3828=136*28+20, 
			//printk("[elan]:write_times=%d\n",write_times);
            //write_bytes = 18;  //28=37-9, 9 means 04 00 23 00 03 21 x1 x2 y1, where x1,x2=offset, y1=payload
        }
        else {
            write_times = 142;  //30*132=3960, 3960=141*28+12
			//printk("[elan]:write_times!!!!!=%d\n",write_times);
            //write_bytes = 12;
        }
        mdelay(5);
        for(byte_count=1; byte_count <= write_times; byte_count++)
        {
            //mdelay(2);
            if(byte_count != write_times)	// header + data = 9+28=37 bytes
            {
				//printk("[elan]:%d\n",byte_count);
                szBuff = fw_data + curIndex;
                write_buf[8] = 28;   //payload length = 0x1c => 28
                write_buf[7] = offset & 0x00ff;
                write_buf[6] = offset >> 8;
                offset +=28;
                curIndex =  curIndex + 28;
                for (j=0; j<28; j++)
                    write_buf[j+9]=szBuff[j];
#if 0          
               if(iPage==121){//if ((iPage == 451)&& (byte_count > 130)) {
                printk("[iap] P=%d i=%d: ", iPage,byte_count);
                for (j=0; j<37; j++)
                	printk("%x ", write_buf[j]);
                printk("\n");
                } 
#endif
                res = WritePage(write_buf, 37); 
            }
			else if((iPage == 451) && (byte_count == write_times)) // the final page, header + data = 9+20=29 bytes, the rest of bytes are the previous page's data
			{
				printk("[elan] Final Page...\n");
                szBuff = fw_data + curIndex;
                write_buf[8] = 20;	//payload length = 0x14 => 20
                write_buf[7] = offset & 0x00ff;
                write_buf[6] = offset >> 8;
                curIndex =  curIndex + 20;
                for (j=0; j<20; j++)
                    write_buf[j+9]=szBuff[j];
#if 0
                printk("[hid iap_Final] P=%d i=%d times=%d: ", iPage,byte_count, write_times);
				//printk("[iap] P=%d i=%d: ", iPage,byte_count);
                for (j=0; j<37; j++)
                    printk("%x ", write_buf[j]);
                printk("\n");
#endif
                res = WritePage(write_buf, 37);
			}
            else			// last run of this 30 page, header + data = 9+12=21 bytes, the rest of bytes are the previous page's data
            {
				//printk("[elan]-----:%d\n",byte_count);
                szBuff = fw_data + curIndex;
                write_buf[8] = 12;	//payload length = 0x0c => 12
                write_buf[7] = offset & 0x00ff;
                write_buf[6] = offset >> 8;
                curIndex =  curIndex + 12;
                for (j=0; j<12; j++)
                    write_buf[j+9]=szBuff[j];
#if 0
                printk("[elan:hid iap_Last] P=%d i=%d times=%d: ", iPage,byte_count, write_times);
				//printk("[iap] P=%d i=%d: ", iPage,byte_count);
                for (j=0; j<37; j++)
                    printk("%x ", write_buf[j]);
                printk("\n");
#endif
                res = WritePage(write_buf, 37);
            }
        } // end of for(byte_count=1;byte_count<=17;byte_count++)

        msleep(50);

		/*
        printk("[elan iap write] cmd ");
        for (j=0; j<37; j++)
        	printk("%x ", cmd_iap_write[j]);
        printk("\n");
		*/

        res = WritePage(cmd_iap_write, 37);
       // mdelay(200);
		printk("[iap] iPage=%d :", iPage);
        res = GetAckData(private_ts->client);
        msleep(10);
    } // end of for(iPage = 1; iPage <= PageNum; iPage++)

    res = SendEndCmd(private_ts->client);

    msleep(200);
    elan_ktf_ts_hw_reset();
    msleep(200);
    res = elan_ktf_ts_calibrate(private_ts->client);
    msleep(100);

    touch_debug(DEBUG_INFO,"[elan] Update Firmware successfully!\n");
    power_lock = 0;
    work_lock=0;
    enable_irq(touch_irq);
    mutex_unlock(&ktf_mutex);
    return res;
}
#endif
#endif
#if elan_fw_upgrade
static int HID_FW_Update_InDriver(void *d)
{
    int res = 0,ic_num = 1;
    int iPage = 0; /* rewriteCnt = 0; rewriteCnt for PAGE_REWRITE */
    int j = 0; // i=0;
    int write_times = 142;
    //int write_bytes = 12;
    uint8_t data;
    //int restartCnt = 0; // For IAP_RESTART
    int byte_count;
    uint8_t *szBuff = NULL;
    //u8 write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x21, 0x00, 0x00, 0x28};
    u8 write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x21, 0x00, 0x00, 0x1c};
    u8 cmd_iap_write[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x22};
	//uint8_t cmd_idel[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x2c, 0x03, 0x01};
    int curIndex = 0;
    int offset = 0;
//int retry_num=0;
    uint8_t *fw_data;	
    mutex_lock(&ktf_mutex);
    printk("[elan] enter %s.\n",__FUNCTION__);
	//msleep(10*1000);
    work_lock=1;
    disable_irq(touch_irq);       
    power_lock = 1;

	printk(KERN_DEBUG "[elan] Update Firmware by fw_data.i\n");
    PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
	touch_debug(DEBUG_ERROR, "[elan] PageNum = %d.\n", PageNum);
    fw_data = file_fw_data;		
    touch_debug(DEBUG_INFO, "[elan] %s:  ic_num=%d\n", __func__, ic_num);
    data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);
//UpdataFW_Retry:
    elan_ktf_ts_hw_reset();
    msleep(200);//mdelay(200);
    res = CheckISPstatus(private_ts->client);
	printk("[elan zhanghui ] res: 0x%x\n",res);
    if (res == 0xa6) { /* 0x88 recovery mode  */
        elan_ktf_ts_hw_reset();
        mdelay(200);//mdelay(200);
        printk("[elan hid iap] Recovery mode\n");
        res = HID_RecoveryISP(private_ts->client);
    }
    else {
        printk("[elan hid iap] Normal mode\n");
        res = HID_EnterISPMode(private_ts->client);   //enter HID ISP mode
    }
    mdelay(50);	
    // Start HID IAP
    for( iPage = 1; iPage <= PageNum; iPage +=30 )
    {
        offset=0;
        if (iPage == 451 ) {
            //write_times = 109;			
			write_times = 137;  //29*132=3828, 3828=136*28+20, 
            //write_bytes = 18;  //28=37-9, 9 means 04 00 23 00 03 21 x1 x2 y1, where x1,x2=offset, y1=payload
        }
        else {
            write_times = 142;  //30*132=3960, 3960=141*28+12
            //write_bytes = 12;
        }
        mdelay(5);
        for(byte_count=1; byte_count <= write_times; byte_count++)
        {
            mdelay(1);
            if(byte_count != write_times)	// header + data = 9+28=37 bytes
            {
				//printk("[elan]:%d\n",byte_count);
                szBuff = fw_data + curIndex;
                write_buf[8] = 28;   //payload length = 0x1c => 28
                write_buf[7] = offset & 0x00ff;
                write_buf[6] = offset >> 8;
                offset +=28;
                curIndex =  curIndex + 28;
                for (j=0; j<28; j++)
                    write_buf[j+9]=szBuff[j];
#if 0          
               if(iPage==121){//if ((iPage == 451)&& (byte_count > 130)) {
                printk("[iap] P=%d i=%d: ", iPage,byte_count);
                for (j=0; j<37; j++)
                	printk("%x ", write_buf[j]);
                printk("\n");
                } 
#endif
                res = WritePage(write_buf, 37); 
            }
			else if((iPage == 451) && (byte_count == write_times)) // the final page, header + data = 9+20=29 bytes, the rest of bytes are the previous page's data
			{
				printk("[elan] Final Page...\n");
                szBuff = fw_data + curIndex;
                write_buf[8] = 20;	//payload length = 0x14 => 20
                write_buf[7] = offset & 0x00ff;
                write_buf[6] = offset >> 8;
                curIndex =  curIndex + 20;
                for (j=0; j<20; j++)
                    write_buf[j+9]=szBuff[j];
#if 0
                printk("[hid iap_Final] P=%d i=%d times=%d: ", iPage,byte_count, write_times);
				//printk("[iap] P=%d i=%d: ", iPage,byte_count);
                for (j=0; j<37; j++)
                    printk("%x ", write_buf[j]);
                printk("\n");
#endif
                res = WritePage(write_buf, 37);
			}
            else			// last run of this 30 page, header + data = 9+12=21 bytes, the rest of bytes are the previous page's data
            {
				//printk("[elan]-----:%d\n",byte_count);
                szBuff = fw_data + curIndex;
                write_buf[8] = 12;	//payload length = 0x0c => 12
                write_buf[7] = offset & 0x00ff;
                write_buf[6] = offset >> 8;
                curIndex =  curIndex + 12;
                for (j=0; j<12; j++)
                    write_buf[j+9]=szBuff[j];
#if 0
                printk("[elan:hid iap_Last] P=%d i=%d times=%d: ", iPage,byte_count, write_times);
				//printk("[iap] P=%d i=%d: ", iPage,byte_count);
                for (j=0; j<37; j++)
                    printk("%x ", write_buf[j]);
                printk("\n");
#endif
                res = WritePage(write_buf, 37);
            }
        } // end of for(byte_count=1;byte_count<=17;byte_count++)

        mdelay(200);//msleep(200);
		/*
        printk("[elan iap write] cmd ");
        for (j=0; j<37; j++)
        	printk("%x ", cmd_iap_write[j]);
        printk("\n");
		*/
        res = WritePage(cmd_iap_write, 37);
		mdelay(200);
		printk("[iap] iPage=%d :", iPage);
        res = GetAckData(private_ts->client);
		if(res<0)
		{
			return res;
#if 0
			if(retry_num<1)
			{
				touch_debug(DEBUG_INFO,"[elan] Update Firmware retry_num=%d fail!!\n",retry_num);
				retry_num++;
				//goto UpdataFW_Retry;
			}
			else
			{				
				touch_debug(DEBUG_INFO,"[elan] Update Firmware retry_num=%d !!\n",retry_num);
			}
#endif
		}
        mdelay(10);
    } // end of for(iPage = 1; iPage <= PageNum; iPage++)

    res = SendEndCmd(private_ts->client);

    msleep(200);//mdelay(200);
    elan_ktf_ts_hw_reset();
    msleep(400);//mdelay(200);
    res = elan_ktf_ts_calibrate(private_ts->client);
    msleep(100);//mdelay(100);
	__fw_packet_handler_HID(private_ts->client);
	//elan_i2c_send_data(private_ts->client,cmd_idel,37);
    touch_debug(DEBUG_INFO,"[elan] Update Firmware successfully!\n");
    power_lock = 0;
    work_lock=0;
    enable_irq(touch_irq);
	//switch_pen_state(0);
    mutex_unlock(&ktf_mutex);
	
    return res;
}
#endif

static int elan_ktf_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t w_size,  size_t r_size)
{
    int rc;

    dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

    if (buf == NULL)
        return -EINVAL;
#ifdef ELAN_SUPPORT_I2C_DMA
    if ((elan_i2c_send_data(client, cmd, w_size)) != w_size) {
        dev_err(&client->dev,
                "[elan]%s: elan_i2c_send_data failed\n", __func__);
        return -EINVAL;
    }
#else
	if ((i2c_master_send(client, cmd, w_size)) != w_size) {
        dev_err(&client->dev,
                "[elan]%s: i2c_master_send failed\n", __func__);
        return -EINVAL;
    }
#endif
    rc = elan_ktf_ts_poll_packet(client);
    if (rc < 0)
        printk("%s: poll is high\n",__func__);

    if(r_size <= 0) r_size=w_size;

    if (elan_i2c_recv_data(client, buf, r_size) != r_size)	return -EINVAL;

    return 0;
}
#ifndef ELAN_HID_I2C
static int __hello_packet_handler(struct i2c_client *client)
{
    int rc;
    uint8_t buf_recv[8] = { 0 };

    rc = elan_ktf_ts_poll(client);
    if (rc < 0) {
        printk( "[elan] %s: Int poll failed!\n", __func__);
    }

    rc = i2c_master_recv(client, buf_recv, 8);
	printk("[elan] %s: hello packet %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
	
    if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
    {
        RECOVERY=0x80;
        return RECOVERY;
    }

    /*Some Elan init don't need Re-Calibration */
#if 0
    mdelay(300);
    rc = elan_ktf_ts_poll(client);
    if (rc < 0) {
        printk( "[elan] %s: Int poll failed!\n", __func__);
    }
    rc = i2c_master_recv(client, buf_recv, 8);
    printk("[elan] %s: Try Re-Calibration packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
#endif
    return 0;
}


static int __fw_packet_handler(struct i2c_client *client)
{
    struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
    int rc;
    int major, minor;
    uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
#if 0  /* for one-layer */
    uint8_t cmd_x[]   = { 0x53, 0x60, 0x00, 0x00 };     /*Get x resolution*/
    uint8_t cmd_y[]   = { 0x53, 0x63, 0x00, 0x00 };     /*Get y resolution*/
#endif
#if 0
    uint8_t info_buff[] = { 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00 }; /*Get IC info*/
    uint8_t info_buff_resp[17] = { 0 };
#endif	
    uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
    uint8_t cmd_bc[] = {CMD_R_PKT, 0x10, 0x00, 0x01};/* Get BootCode Version*/
    //uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
    uint8_t buf_recv[4] = {0};
    // Firmware version
    rc = elan_ktf_ts_get_data(client, cmd, buf_recv, 4,4);
    if (rc < 0)
    {
        printk("Get Firmware version error\n");
    }
    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
    ts->fw_ver = major << 8 | minor;
    FW_VERSION = ts->fw_ver;
    // Firmware ID
    rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4,4);
    if (rc < 0)
    {
        printk("Get Firmware ID error\n");
    }
    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
    ts->fw_id = major << 8 | minor;
    FW_ID = ts->fw_id;
		
    // Bootcode version
    rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv, 4,4);
    if (rc < 0)
    {
        printk("Get Bootcode version error\n");
    }
    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
    ts->bc_ver = major << 8 | minor;
	BC_VERSION = ts->bc_ver;

    /*Get XY info*/
#if 0  /* for one-layer */
    /* X Resolution */
    rc = elan_ktf_ts_get_data( client, cmd_x, buf_recv, 4, 4);
    if( rc < 0 )
        return rc;
    minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
    ts->x_resolution =minor;
    X_RESOLUTION = minor;

    /* Y Resolution */
    rc = elan_ktf_ts_get_data( client, cmd_y, buf_recv, 4, 4);
    if( rc < 0 )
        return rc;
    minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
    ts->y_resolution =minor;
    Y_RESOLUTION = minor;
#endif
#if 0
    rc = elan_ktf_ts_get_data(client, info_buff, info_buff_resp, sizeof(info_buff), sizeof(info_buff_resp));
    if (rc < 0)
    {
        printk("[elan] Get XY info error\n");
    }
    ts->x_resolution = (info_buff_resp[2] + info_buff_resp[6]
                        + info_buff_resp[10] + info_buff_resp[14] - 1)*64;
//	X_RESOLUTION = ts->x_resolution;

    ts->y_resolution = (info_buff_resp[3] + info_buff_resp[7]
                        + info_buff_resp[11] + info_buff_resp[15] - 1)*64;
//	Y_RESOLUTION = ts->y_resolution;
#endif

    printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n", __func__, ts->fw_ver);
    printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n", __func__, ts->fw_id);
    printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n", __func__, ts->bc_ver);
    printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n", __func__, X_RESOLUTION, Y_RESOLUTION);

    return 0;
}

#else
#if 0
static void elan_enable_plam(struct i2c_client *client)
{
	 uint8_t plan_check_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0xc1, 0x00, 0x01};
	 uint8_t plan_check_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0xcf, 0x00, 0x01};
	 uint8_t buf_recv[67]={0};
	 int res=0;
	 elan_ktf_ts_get_data(client, plan_check_cmd, buf_recv, 37, 67);
	 printk("[elan] %s: (plam check) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
	 res=(buf_recv[7]>>3)&0x1;
	 if(res==0)
	 {
		 plan_check_cmd[7]=0x54;
		 plan_check_cmd[9]=buf_recv[6];
		 plan_check_cmd[10]=buf_recv[7]|0x8;
		 elan_i2c_send_data(client,plan_check_cmd,37);
		 msleep(5);
		 elan_ktf_ts_get_data(client, plan_check_buf, buf_recv, 37, 67);
		 printk("[elan] %s: (plam check flag) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
		 res=(buf_recv[7]>>3)&0x1;
		 if(res==0)
		 {
			  plan_check_buf[7]=0x54;
			  plan_check_buf[9]=buf_recv[6];
		      plan_check_buf[10]=buf_recv[7]|0x8;
		      elan_i2c_send_data(client,plan_check_buf,37);
		 }
	 }
	 else
	 {
		  elan_ktf_ts_get_data(client, plan_check_buf, buf_recv, 37, 67);
		  printk("[elan] %s: (plam check flag) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
		  res=(buf_recv[7]>>3)&0x1;
		  if(res==0)
		  {
			plan_check_buf[7]=0x54;
			plan_check_buf[9]=buf_recv[6];
			plan_check_buf[10]=buf_recv[7]|0x8;
			elan_i2c_send_data(client,plan_check_buf,37);
		  }
	 }
}
#endif
static int __fw_packet_handler_HID(struct i2c_client *client)
{
    struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
    int rc;
    int major, minor;
	//int addr;
	//uint8_t cmd_addr[37] = 	{0x04, 0x00, 0x23, 0x00, 0x03, 0x18};/* Get IC Addr*/
    uint8_t cmd_fw_ver[37] = 	{0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
    uint8_t cmd_id[37] = 		{0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
    uint8_t cmd_bc[37] = 		{0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0x10, 0x00, 0x01};/* Get BootCode Version*/
	//uint8_t cmd_idel[37] = 		{0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x2c, 0x03, 0x01};
	uint8_t pen_xmax[37] = 		{0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0x60, 0x00, 0x01};
	uint8_t pen_ymax[37] = 		{0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x53, 0x61, 0x00, 0x01};
    //uint8_t buf_recv[4] = {0};
	uint8_t buf_recv[67] = {0};

#if 0
	rc = elan_ktf_ts_get_data(client, cmd_addr, buf_recv, 37, 67);
    if (rc < 0)
    {
        printk("Get IC ADDR error\n");
    }
	printk("[elan] %s: (IC ADDR) %2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4]);
	addr = buf_recv[4];
#endif
    // Firmware version
    rc = elan_ktf_ts_get_data(client, cmd_fw_ver, buf_recv, 37, 67);
    if (rc < 0)
    {
        printk("Get Firmware version error\n");
    }
	printk("[elan] %s: (Firmware version) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
    major = ((buf_recv[5] & 0x0f) << 4) | ((buf_recv[6] & 0xf0) >> 4);
    minor = ((buf_recv[6] & 0x0f) << 4) | ((buf_recv[7] & 0xf0) >> 4);
    ts->fw_ver = major << 8 | minor;
    FW_VERSION = ts->fw_ver;	
    // Firmware ID
    rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 37, 67);
    if (rc < 0)
    {
        printk("Get Firmware ID error\n");
    }
	printk("[elan] %s: (Firmware ID) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
    major = ((buf_recv[5] & 0x0f) << 4) | ((buf_recv[6] & 0xf0) >> 4);
    minor = ((buf_recv[6] & 0x0f) << 4) | ((buf_recv[7] & 0xf0) >> 4);
    ts->fw_id = major << 8 | minor;
    FW_ID = ts->fw_id;

    // Bootcode version
    rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv, 37, 67);
    if (rc < 0)
    {
        printk("Get Bootcode version error\n");
    }
	printk("[elan] %s: (Bootcode version) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
    major = ((buf_recv[5] & 0x0f) << 4) | ((buf_recv[6] & 0xf0) >> 4);
    minor = ((buf_recv[6] & 0x0f) << 4) | ((buf_recv[7] & 0xf0) >> 4);
    ts->bc_ver = major << 8 | minor;
	BC_VERSION = ts->bc_ver;
	// read pen x
	rc = elan_ktf_ts_get_data(client, pen_xmax, buf_recv, 37, 67);
    if (rc < 0)
    {
        printk("Get Bootcode version error\n");
    }
	printk("[elan] %s: (Bootcode version) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
	elan_pen_xmax=(buf_recv[7]<<8)|buf_recv[6];
	//read pen y
	rc = elan_ktf_ts_get_data(client, pen_ymax, buf_recv, 37, 67);
    if (rc < 0)
    {
        printk("Get Bootcode version error\n");
    }
	printk("[elan] %s: (Bootcode version) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
	elan_pen_ymax=(buf_recv[7]<<8)|buf_recv[6];
	
	//elan_i2c_send_data(client,cmd_idel,37);

	//elan_enable_plam(client);
	//printk(KERN_INFO "[elan] %s: IC Addr: 0x%4.4x\n", __func__, addr);
    printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n", __func__, ts->fw_ver);
    printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n", __func__, ts->fw_id);
    printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n", __func__, ts->bc_ver);
    printk(KERN_INFO "[elan] %s: penx resolution: %d, pen y resolution: %d\n", __func__, elan_pen_xmax, elan_pen_ymax);

    return 0;
}
#endif

static inline int elan_ktf_pen_parse_xy(uint8_t *data,
                                        uint16_t *x, uint16_t *y, uint16_t *p)
{
    *x = *y = *p = 0;
    *x = data[5];
    *x <<= 8;
    *x |= data[4];

    *y = data[7];
    *y <<= 8;
    *y |= data[6];

    *p = data[9];
    *p <<= 8;
    *p |= data[8];

    return 0;
}

static inline int elan_ktf_ts_parse_xy(uint8_t *data,
                                       uint16_t *x, uint16_t *y)
{
    *x = *y = 0;

    *x = (data[0] & 0xf0);
    *x <<= 4;
    *x |= data[1];

    *y = (data[0] & 0x0f);
    *y <<= 8;
    *y |= data[2];

    return 0;
}

static int elan_ktf_ts_setup(struct i2c_client *client)
{
    int rc = 0;
#ifndef ELAN_HID_I2C
    rc = __hello_packet_handler(client);
#endif
    if (rc != 0x80) {
#ifndef ELAN_HID_I2C
        rc = __fw_packet_handler(client);
#else
		rc = __fw_packet_handler_HID(client);
#endif
        if (rc < 0)
            printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
        dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);
        //Check for FW_VERSION, if 0x0000 means FW update fail!
        if ( FW_VERSION == 0x00)
        {
            rc = 0x80;
            printk("[elan] FW_VERSION = 0x%4.4x, last FW update fail\n", FW_VERSION);
        }
    }
#ifdef SENSOR_OPTION
	else
	{
		printk("[elan] %s: rc = 0x%x, Recovery Mode\n", __func__, rc);
		// Ask Firmware ID
		rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4,4);
		if (rc < 0)
		{
			printk("Get Firmware ID error\n");
		}
		major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
		ts->fw_id = major << 8 | minor;
		FW_ID = ts->fw_id;
		printk(KERN_DEBUG "[elan] %s: in Recovery mode, get FW_ID = 0x%4.4x\n", __func__, FW_ID);
	}
#endif
    return rc;
}

static int elan_ktf_ts_calibrate(struct i2c_client *client) {

#ifdef ELAN_HID_I2C
    uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0xc0, 0xe1, 0x5a};
    uint8_t cal_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0x29, 0x00, 0x01};

    printk("[elan] %s: Flash Key cmd\n", __func__);
    if ((elan_i2c_send_data(client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) {
        printk("[elan] %s: i2c_master_send failed\n", __func__);
        return -EINVAL;
    }
    printk("[elan] %s: Calibration cmd: %02x, %02x, %02x, %02x\n", __func__,
             cal_cmd[7], cal_cmd[8], cal_cmd[9], cal_cmd[10]);
    if ((elan_i2c_send_data(client, cal_cmd, sizeof(cal_cmd))) != sizeof(cal_cmd)) {
        printk("[elan] %s: i2c_master_send failed\n", __func__);
        return -EINVAL;
    }

#else
    uint8_t flash_key[] = {CMD_W_PKT, 0xC0, 0xE1, 0x5A};
    uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};
    printk("[elan] %s: enter\n", __func__);
	dev_info(&client->dev, "[elan] dump flash_key: %02x, %02x, %02x, %02x\n", flash_key[0], flash_key[1], flash_key[2], flash_key[3]);
	if ((i2c_master_send(client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) {
        dev_err(&client->dev, "[elan] %s: (flash_key) i2c_master_send failed\n", __func__);
        return -EINVAL;
    }	
    dev_info(&client->dev, "[elan] dump cal_cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);
    if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
        dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
        return -EINVAL;
    }
#endif
    return 0;
}

static int elan_ktf_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{
	int rc;
	if (buf == NULL)
		return -EINVAL;
	memset(buf, 0, bytes_to_recv);
	rc = elan_i2c_recv_data(client, buf, bytes_to_recv);
	if (rc != bytes_to_recv) {
		dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
		return -1;
    }
	ELAN_DEBUG("[elan_debug_receive] recv_data  %x %x %x %x %x %x----%x,%x,%x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],buf[61],buf[62],buf[63]);

	return rc;
}

#ifdef PROTOCOL_B
/* Protocol B  */
static int mTouchStatus[FINGER_NUM] = {0};  /* finger_num=10 */
void force_release_pos(struct i2c_client *client)
{
    struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
    int i;
    for (i=0; i < FINGER_NUM; i++) {
        if (mTouchStatus[i] == 0) continue;
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
        mTouchStatus[i] = 0;
    }

    input_sync(ts->input_dev);
}

static inline int elan_ktf_hid_parse_xy(uint8_t *data,
                                        uint16_t *x, uint16_t *y)
{
    *x = *y = 0;

    *x = (data[6]);
    *x <<= 8;
    *x |= data[5];

    *y = (data[10]);
    *y <<= 8;
    *y |= data[9];

    return 0;
}

/* Protocol B  */
static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	//struct input_dev *idev = ts->input_dev;
	struct input_dev *idev = tpd->dev;
	uint16_t x =0, y =0,touch_size, pressure_size;
	uint16_t fbits=0;
	uint8_t i, num;
	uint16_t active = 0;
	uint8_t idx, btn_idx;
	int finger_num;
	int finger_id;
	int pen_hover = 0;
	int pen_down = 0;
	int pen_key=0;

//	int plam_value=0;
	//int old_key =0;
	uint16_t p = 0;
	//int report=0;

    /* for 10 fingers */
	if (buf[0] == TEN_FINGERS_PKT) {
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx=3;
		btn_idx=33;
	}
    /* for 5 fingers  */
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)) {
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
	} else {
        /* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
	//      fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
		idx=1;
		btn_idx=7;
	}

	switch (buf[0]) {
		case MTK_FINGERS_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:
		case TEN_FINGERS_PKT:

		for(i = 0; i < finger_num; i++) {
			active = fbits & 0x1;
			if(active || mTouchStatus[i]) {
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
                if(active) {
                    elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
			//printk("[elan] finger id!!!!!!=%d x=%d y=%d \n", i, x, y);
                    //y=Y_RESOLUTION -y;
           	x=x*LCM_Y_MAX/ELAN_Y_MAX;
			y=y*LCM_X_MAX/ELAN_X_MAX;
			touch_size = buf[FW_POS_WIDTH + i];
			pressure_size = buf[FW_POS_PRESSURE + i];
					
                    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch_size);
                    input_report_abs(idev, ABS_MT_PRESSURE, pressure_size);
                    input_report_abs(idev, ABS_MT_POSITION_X, x);
                    input_report_abs(idev, ABS_MT_POSITION_Y, y);
                    if(unlikely(gPrint_point))
                        touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size);
					printk("[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size);
                }
            }
            mTouchStatus[i] = active;
            fbits = fbits >> 1;
            idx += 3;
        }
        if (num == 0) {
            //printk("[elan] ALL Finger Up\n");
            input_report_key(idev, BTN_TOUCH, 0); //for all finger up
            force_release_pos(client);
        }
        input_sync(idev);
        break;
    case PEN_PKT:
		ELAN_DEBUG("[elan] pen PEN_PKT enter \n");
	
		if(buf[13]==0x1)
		{
			
			ELAN_DEBUG("[elan] pen KEY_APPSELECT down! \n");
			elan_key_flag=KEY_APPSELECT;
			input_report_key(idev, KEY_APPSELECT, 1);
			input_sync(idev);
			return;
		}
		else if(buf[13]==0x2)
		{
			
			ELAN_DEBUG("[elan]pen KEY_BACK down! \n");
			elan_key_flag=KEY_BACK;
			input_report_key(idev, KEY_BACK, 1);
			input_sync(idev);
			return;
		}
		else if((elan_key_flag!=0)&&(buf[13]==0))
		{
			
			ELAN_DEBUG("[elan] pen KEY up! \n");
			input_report_key(idev, elan_key_flag, 0);
			input_sync(idev);
			elan_key_flag=0;
			return;
		}

	      pen_hover = buf[3] & 0x1;
	      pen_down = buf[3] & 0x03;
		pen_key=(buf[3]>>2)&0x7;
		ELAN_DEBUG("[elan] pen_hover=%d pen_down=%d  pen_key=%d\n",pen_hover,pen_down,pen_key);
		if((pen_key!=0)&&(pen_key_flag==0))
		{
			 pen_key_flag=1;
			 ELAN_DEBUG("[elan]:F20 pen_KEY touch down1!!\n");
			 input_report_key(idev, KEY_F20, 1);
			 input_sync(idev);						
		}
		else if((pen_key==0x0)&&(pen_key_flag==1))
		{
			 pen_key_flag=0;
			 ELAN_DEBUG("[elan]:F20 pen_KEY touch up!!\n");
			 input_report_key(idev, KEY_F20, 0);
			 input_sync(idev);
		}

        if (pen_hover) {
            //elan_ktf_pen_parse_xy(&buf[0], &x, &y, &p);
            elan_ktf_pen_parse_xy(&buf[0], &y, &x, &p);
			x=x*LCM_X_MAX/(16*260-1); // change by yuanhui 20170612 start
			y=y*LCM_Y_MAX/(33*260-1); // change by yuanhui 20170612 end

			//x=LCM_X_MAX-x;
			//y=LCM_Y_MAX-y;
            if (pen_down == 0x01) {  /* report hover function  */
				//delete XWQYHWSYY-24 by qiang.xue 20171207 start
				input_report_key(idev, BTN_TOOL_PENCIL, 1);
				//delete XWQYHWSYY-24 by qiang.xue 20171207 end
                input_report_abs(idev, ABS_MT_PRESSURE, 0);
				input_report_abs(idev, ABS_MT_DISTANCE, 15);
            }
            else {
				input_report_key(idev, BTN_TOUCH, 1);
                input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 20);
                input_report_abs(idev, ABS_MT_PRESSURE, p);
				input_report_abs(idev, ABS_MT_DISTANCE, 1);
            }

			input_report_abs(idev, ABS_MT_TRACKING_ID, 0);
            input_report_abs(idev, ABS_MT_POSITION_X, x);
            input_report_abs(idev, ABS_MT_POSITION_Y, y);
			input_mt_sync(idev);
//add by zhanghui for factory mode touch err 20170320 start
		if(boot_normal_flag==0){
			tpd_button(x,y,1);
			TPD_EM_PRINT(x,y,x,y,0,1);
		}
//add by zhanghui for factory mode touch err 20170320 end
        }
	//	else{
	//		input_mt_sync(idev);
	//	}
        if(unlikely(gPrint_point)) {
            touch_debug(DEBUG_INFO, "[elan pen] %x %x %x %x %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
            touch_debug(DEBUG_INFO, "[elan] x=%d y=%d p=%d \n", x, y, p);
        }
            //touch_debug(DEBUG_INFO, "[elan pen] x=%d y=%d p=%d \n", x, y, p);
        if (pen_down == 0) {
				ELAN_DEBUG("[elan] ALL Finger Up\n");
				pen_power_flag=0;
				pen_key_flag=0;
				input_report_key(idev, BTN_TOUCH, 0); //for all finger up
				//delete XWQYHWSYY-24 by qiang.xue 20171207 start
				input_report_key(idev, BTN_TOOL_PENCIL, 0);
				//delete XWQYHWSYY-24 by qiang.xue 20171207 end
				input_mt_sync(idev);

		if((buf[10]==0x2)&&(pen_power_flag==0))//电量低于15%
		{
			ELAN_DEBUG("[elan]:pen power <15\n");
			pen_power_flag=1;
			input_report_key(idev, KEY_F23, 1);
			input_sync(idev);
			input_report_key(idev, KEY_F23, 0);
			input_sync(idev);
		}else if((buf[10]==0x1)&&(pen_power_flag==0))//电量低于5%
		{
			ELAN_DEBUG("[elan]:pen power <5\n");
			pen_power_flag=1;
			input_report_key(idev, KEY_F24, 1);
			input_sync(idev);
			input_report_key(idev, KEY_F24, 0);
			input_sync(idev);
		}

//add by zhanghui for factory mode touch err 20170320 start
				if(boot_normal_flag==0){
					tpd_button(0,0,0);
					TPD_EM_PRINT(0,0,0,0,0,0);
				}
//add by zhanghui for factory mode touch err 20170320 end
            //force_release_pos(client);
        }//zhanghui
        input_sync(idev);
	
        break;
	case ELAN_HID_PKT:
		//ELAN_DEBUG("[elan]:enter ELAN_HID_PKT\n");
		finger_num = buf[62];
		if (finger_num > 5)
			finger_num = 5;   /* support 5 fingers    */
		idx=3;
		num = 5;
		#if 0
		plam_value=buf[63]>>7;
		ELAN_DEBUG("[elan]:plam_flag=%d,plam_value=%d\n",plam_flag,plam_value);
		if((plam_flag==0)&&(plam_value==1))
		{
			plam_flag=1;
			ELAN_DEBUG("[elan]:is in to plam\n");
			return;
		}else if((plam_flag==1)&&(plam_value==1)){
			return;
		}

		plam_flag=0;
		#endif

        for(i = 0; i < 5; i++) {
	    //add by zhanghui for no key up err XWQYHWSYYN-408 20170418 start
		if((elan_key_flag!=0)&&(buf[63]==0)){
			ELAN_DEBUG("[elan] KEY up! \n");
			input_report_key(idev, elan_key_flag, 0);
			input_sync(idev);
			elan_key_flag=0;
		}
	    //add by zhanghui for no key up err XWQYHWSYYN-408 20170418 end

            if ((buf[idx]&0x03) == 0x00)	active = 0;   /* 0x03: finger down, 0x00 finger up  */
            else	active = 1;

            if ((buf[idx] & 0x03) == 0) num --;
            finger_id = (buf[idx] & 0xfc) >> 2;
	   // 	ELAN_DEBUG("[elan]:finger_num=%d,plam_flag=%d,active=%d,finger_id=%d\n",finger_num,plam_flag,active,finger_id);
            if(active) {
             //   elan_ktf_hid_parse_xy(&buf[idx], &x, &y);
				elan_ktf_hid_parse_xy(&buf[idx], &y, &x);
			ELAN_DEBUG("[elan] finger id----!!!=%d x=%d y=%d \n", i, y, x);
                    //y=Y_RESOLUTION -y;
				x=x*LCM_X_MAX/ELAN_X_MAX;
				y=y*LCM_Y_MAX/ELAN_Y_MAX;
				//x=LCM_X_MAX-x;
				//y=LCM_Y_MAX-y;
                //y = Y_RESOLUTION - y;
				//if(y<1910){                               //change by zhanghui for 10 pixel bottom retract 20170420
					input_report_key(idev, BTN_TOUCH, 1);
					input_report_abs(idev, ABS_MT_TRACKING_ID, finger_id);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
					input_report_abs(idev, ABS_MT_PRESSURE, 1300);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					ELAN_DEBUG("[elan hid] i=%d finger_id=%d x=%d y=%d Finger NO.=%d \n", i, finger_id, x, y, finger_num);
					input_mt_sync(idev);
				//}
//add by zhanghui for factory mode touch err 20170320 start
				if(boot_normal_flag==0)
				{
					tpd_button(x,y,1);
					TPD_EM_PRINT(x,y,x,y,finger_id,1);
				}
//add by zhanghui for factory mode touch err 20170320 end
		//report++;
            }
            mTouchStatus[i] = active;
            idx += 11;
        }

        if (buf[62] == 0){
			if(buf[63]==0x1)
			{
				ELAN_DEBUG("[elan] KEY_APPSELECT down! \n");
				elan_key_flag=KEY_APPSELECT;
				input_report_key(idev, KEY_APPSELECT, 1);
				input_sync(idev);
				return;
			}
			else if(buf[63]==0x2)
			{
				ELAN_DEBUG("[elan] KEY_BCAK down! \n");
				elan_key_flag=KEY_BACK;
				input_report_key(idev, KEY_BACK, 1);
				input_sync(idev);
				return;
			}
			else if((elan_key_flag!=0)&&(buf[63]==0))
			{
				ELAN_DEBUG("[elan] KEY up! \n");
				input_report_key(idev, elan_key_flag, 0);
				input_sync(idev);
				elan_key_flag=0;
				return;
			}
        }
		if(num==0){
			ELAN_DEBUG("[elan] Release ALL Finger\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			input_mt_sync(idev);
//add by zhanghui for factory mode touch err 20170320 start
			if(boot_normal_flag==0)
			{
				tpd_button(0,0,0);
				TPD_EM_PRINT(0,0,0,0,0,0);
			}
//add by zhanghui for factory mode touch err 20170320 end
		}
        input_sync(idev);
        break ;
    case IamAlive_PKT:
        touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3]);
        break;
    case HELLO_PKT:
        printk("[elan] %s: Report Hello packet: %x %x %x %x\n",__func__, buf[0],buf[1],buf[2],buf[3]);
        //__fw_packet_handler(private_ts->client);
        break;
    case CALIB_PKT:
        printk("[elan] %s: Report Calibration packet: %x %x %x %x\n",__func__, buf[0],buf[1],buf[2],buf[3]);
        break;
    default:
		printk("[elan] %s: unknown packet type: %x %x %x %x %x %x %x %x\n",__func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        break;
    } // end switch

    return;
}

#endif
//end #ifdef PROTOCOL_B

static void elan_tp_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf_ts_data *ts = container_of(work,struct elan_ktf_ts_data,work);
	uint8_t buf[PACKET_SIZE] = { 0 };
#if defined( ESD_CHECK )
	have_interrupts = 1;
#endif
#if 0
	if (gpio_get_value(GPIO_CTP_EINT_PIN))
	{
		printk("[elan] Detected the jitter on INT pin");
		return;
	}
#endif
	rc = elan_ktf_ts_recv_data(ts->client, buf, PACKET_SIZE);
	if (rc < 0)
	{
        	printk("[elan] Received the packet Error.\n");
        	return;
	}
	elan_ktf_ts_report_data(ts->client, buf);

}
static irqreturn_t elan_ktf_ts_irq_handler(unsigned irq, void *dev_id)
{
    //printk("[elan]:%s\n",__func__);
    queue_work(elan_wq,&private_ts->work);		
    return IRQ_HANDLED;
}

static int tpd_irq_registration(struct i2c_client *client)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	node = of_find_matching_node(node, touch_of_match);

	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		/*gpio_set_debounce(ints[0]|0x80000000, ints[1]);*/
		touch_irq = irq_of_parse_and_map(node, 0);

		ret = request_irq(touch_irq, (irq_handler_t) elan_ktf_ts_irq_handler,
			    IRQF_TRIGGER_FALLING, "touch-eint", NULL);
				
		if (ret) {
			printk("[elan] %s: request_irq %d failed,ret=%d\n", __func__,touch_irq,ret);
		}
		else
			printk("[elan]:request irq ok");

	}	
	printk("[ELAN:%s]irq:%d, debounce:%d-%d:\n", __func__, touch_irq, ints[0], ints[1]);
	return ret;
}

#if defined( ESD_CHECK )
static void elan_touch_esd_func(struct work_struct *work)
{
    touch_debug(DEBUG_INFO, "[elan esd] %s: enter.......\n", __FUNCTION__);  /* elan_dlx */

    if( (have_interrupts == 1) || (work_lock == 1) )
    {
        touch_debug(DEBUG_INFO, "[elan esd] : had interrup not need check\n");
    }
    else
    {
        /* Reset TP, if touch controller no any response  */
        elan_ktf_ts_hw_reset();
    }
    have_interrupts = 0;
    queue_delayed_work( esd_wq, &esd_work, delay );
    touch_debug(DEBUG_INFO, "[elan esd] %s: exit.......\n", __FUNCTION__ );  /* elan_dlx */
}
#endif
//add 10-18
static ssize_t store_debug_mesg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int level[SYSFS_MAX_LEN];
    sscanf(buf,"%x",&level[0]);
    debug=level[0];
    touch_debug(DEBUG_INFO, "debug level %d, size %d\n", debug, (int)count);
    return count;
}

static ssize_t show_debug_mesg(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Debug level %d \n", debug);
}

static ssize_t show_gpio_int(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Int=%d\n", gpio_get_value(GPIO_CTP_EINT_PIN));
}

static ssize_t store_gpio_int(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return 0;
}

static ssize_t show_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "by echo >reset to reset tp ic\n");
}

static ssize_t store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    elan_ktf_ts_hw_reset();
    touch_debug(DEBUG_INFO, "Reset Touch Screen Controller!\n");
    return count;
}
static ssize_t store_disable_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    work_lock=0;
    enable_irq(touch_irq);
    wake_unlock(&private_ts->wakelock);
    touch_debug(DEBUG_INFO, "Enable IRQ.\n");
    return count;
}

static ssize_t show_disable_irq(struct device *dev, struct device_attribute *attr, char *buf)
{  
    return sprintf(buf, "echo>elan_disable_irq Disable IRQ \n");
}

static ssize_t store_enable_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    work_lock=0;
    enable_irq(touch_irq);
    wake_unlock(&private_ts->wakelock);
    touch_debug(DEBUG_INFO, "Enable IRQ.\n");
    return count;
}

static ssize_t show_enable_irq(struct device *dev, struct device_attribute *attr, char *buf)
{  
    return sprintf(buf, "echo>elan_enable_irq!! enable IRQ \n");
}

static ssize_t show_fw_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "FW VER = 0x%x, FW ID = 0x%x, BC VER = 0x%x\n", FW_VERSION,FW_ID,BC_VERSION);
}

static ssize_t store_fw_info(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return 0;
}

//add by zhanghui for firmware info start
static ssize_t tpd_firmware_version_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        return sprintf(buf,"0x%x\n",FW_VERSION);
}
 
static ssize_t tpd_firmware_version_store(struct device *dev, struct device_attribute *attr,
                 const char *buf, size_t count)
{
       return count;
}
 
static DEVICE_ATTR(tp_firmware_version, 0644, tpd_firmware_version_show, tpd_firmware_version_store);
 
static ssize_t tpd_vendor_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{   
        return sprintf(buf, "%s\n","TCL_EKTH3676C_00");
}
 
static ssize_t tpd_vendor_store(struct device *dev, struct device_attribute *attr,
                 const char *buf, size_t count)
{
       return count;
}
 
static DEVICE_ATTR(tp_vendor, 0644, tpd_vendor_show, tpd_vendor_store);

/* Add by zhanghui for checking TP firmware version start */
/*     
 * Show the release CTP firmware version,
 * This will be checked in the Midtest CTP test item.
 */    
static ssize_t tp_final_fw_version_show(struct device *dev, struct device_attribute *attr,char *buf)
{      
	return sprintf(buf,"%s\n", final_ctp_firmware_version);
}      
       
/*     
 * Store the release CTP firmware version
 * used for test
 */    
static ssize_t tp_final_fw_version_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{      
	strncpy(final_ctp_firmware_version, buf, sizeof(final_ctp_firmware_version));
       return count;
}      
       
/* path:/sys/devices/platform/mtk-tpd/tp_final_firmware_version*/
static DEVICE_ATTR(tp_final_firmware_version, 0664, tp_final_fw_version_show, tp_final_fw_version_store);
/* Add by zhanghui for checking TP firmware version end */


//add by zhanghui for firmware info end

static DEVICE_ATTR(debug_mesg, 0664, show_debug_mesg, store_debug_mesg);
static DEVICE_ATTR(elan_gpio_int, 0664, show_gpio_int, store_gpio_int);
static DEVICE_ATTR(elan_reset, 0664, show_reset, store_reset);
static DEVICE_ATTR(elan_disable_irq, 0664, show_disable_irq, store_disable_irq);
static DEVICE_ATTR(elan_enable_irq, 0664, show_enable_irq, store_enable_irq);
static DEVICE_ATTR(elan_fw_info, 0664, show_fw_info, store_fw_info);
static struct device_attribute *elan_attributes[] = {
	&dev_attr_debug_mesg,
    &dev_attr_elan_gpio_int,
    &dev_attr_elan_reset,
    &dev_attr_elan_disable_irq,
	&dev_attr_elan_enable_irq,
	&dev_attr_elan_fw_info,
	//add by zhanghui 
	&dev_attr_tp_firmware_version,
	&dev_attr_tp_vendor,     
	&dev_attr_tp_final_firmware_version,

};

static int elan_ktf_ts_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf_ts_data *ts;
	//int New_FW_ID;
	//int New_FW_VER;
#if elan_fw_upgrade
	struct task_struct *fw_update_thread=NULL;
#endif

#if RLK_LEATHER_MODE
	uint8_t enable_cover_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0xbf,0x00,0x01};
	uint8_t disable_cover_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0xbf,0x00,0x00};
#endif

	ELAN_INFO("[elan] enter elan_ktf_ts_probe....\n");
	//client->addr = ((client->addr)&I2C_MASK_FLAG)|I2C_DMA_FLAG|I2C_ENEXT_FLAG;
	//client->timing=400;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "[elan] %s: allocate elan_ktf_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ktf_mutex);

	elan_wq=create_singlethread_workqueue("elan_wq");
	if(!elan_wq)
		printk("[elan]:create_singlethread_workqueue fail\n ");

	INIT_WORK(&ts->work, elan_tp_work_func);
#if 0
    ts->gpio_int=devm_gpiod_get(&client->dev,"elan-int",GPIOD_IN);
    if(IS_ERR(ts->gpio_int)){
        printk("[elan]:gpio_int is null\n");
    }
#endif
	elan_ktf_ts_hw_reset();
	msleep(200);
	fw_err = elan_ktf_ts_setup(client);
	if (fw_err < 0) {
		printk(KERN_INFO "[elan] No Elan chip inside\n");
	}
	//elan_enable_plam(client);
	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "elan-touchscreen");
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, MAX_FINGER_PRESSURE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_PRESSURE, 0, MAX_FINGER_PRESSURE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_DISTANCE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, LCM_X_MAX, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, LCM_Y_MAX, 0, 0);
	//  set_bit(KEY_F20, tpd->dev->keybit);
	__set_bit(KEY_F20, tpd->dev->keybit);
	input_set_capability(tpd->dev,EV_KEY,KEY_F20);
	input_set_capability(tpd->dev,EV_KEY,KEY_F23);
	input_set_capability(tpd->dev,EV_KEY,KEY_F24);
	set_bit(KEY_BACK, tpd->dev->keybit);
	set_bit(KEY_APPSELECT, tpd->dev->keybit);
	set_bit(KEY_POWER, tpd->dev->keybit);
	//delete XWQYHWSYY-24 by qiang.xue 20171207 start
	set_bit(BTN_TOOL_PENCIL, tpd->dev->keybit);
	//delete XWQYHWSYY-24 by qiang.xue 20171207 end
	set_bit(INPUT_PROP_POINTER,tpd->dev->propbit);

	private_ts = ts;
	ELAN_GPIO_AS_INT(GTP_INT_PORT);
	tpd_irq_registration(client);
	// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO;
	if (misc_register(&ts->firmware) < 0)
		printk("[elan]misc_register failed!!\n");
	else
		printk("[elan]misc_register finished!!\n");

	// End Firmware Update
	tpd_load_status = 1;

	/* register sysfs */
#if 0
	if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
		printk("[elan]:sysfs create group error\n");
#endif
	pen_open_state = 0;
	//switch_pen_state(pen_open_state);

#ifdef IAP_PORTION
	if(1)
	{
		ELAN_DEBUG("[elan] IAP_PORTION:\n");
		/* FW ID & FW VER*/
		/* for Nexus7 flo ekt file */
		/*
		ELAN_DEBUG(" [elan] [D1DE]=0x%02x, [D1DF]=0x%02x, [D5EA]=0x%02x, [D5EB]=0x%02x\n", file_fw_data[53726],file_fw_data[53727],file_fw_data[54762],file_fw_data[54763]);
		New_FW_ID = file_fw_data[54763]<<8  | file_fw_data[54762] ;
		New_FW_VER = file_fw_data[53727]<<8  | file_fw_data[53726] ;
		ELAN_INFO("[elan] FW_ID=0x%x, New_FW_ID=0x%x\n",  FW_ID, New_FW_ID);
		ELAN_INFO("[elan] FW_VERSION=0x%x, New_FW_VER=0x%x\n", FW_VERSION, New_FW_VER);
		*/

#if elan_fw_upgrade
		if ((New_FW_ID == FW_ID)||(FW_ID==0xffff)){  // for firmware auto-upgrade
			if ((New_FW_VER > (FW_VERSION))||(FW_VERSION==0xffff))
			{
				ELAN_INFO("[elan]:updata fw now!!!!\n");
				path_test = 1;
				fw_update_thread = kthread_run(HID_FW_Update_InDriver, NULL, "elan_update");
    				if(IS_ERR(fw_update_thread))
    				{
        				printk("[elan] failed to create kernel thread\n");
    				}
			}
			else{
				printk("[elan]:This is new FW,not need updata\n");
			}

		}else
			printk("[elan] No auto update, FW_ID is different!\n");
#endif
	}
#endif

#if RLK_LEATHER_MODE
	// Add /proc/gtp_leather_mode
	elan_porc = proc_create(ELAN_PROC_FILE, 0666, NULL, &gt_leather_mode_proc_fops);
	if (elan_porc == NULL)
	{
		printk("create_proc_entry %s failed\n", ELAN_PROC_FILE);
	}
	//add XWQYHWSYY-24 by qiang.xue 20171206 start
	//ftp_leather_mode_status = !(gpio_get_value(hall_gpio));
	//add XWQYHWSYY-24 by qiang.xue 20171206 end
	if(1==ftp_leather_mode_status) {
			if ((elan_i2c_send_data(client, enable_cover_cmd, sizeof(enable_cover_cmd))) != sizeof(enable_cover_cmd)) {
				printk("[elan] i2c_master_send enable_cover_cmd failed\n");
				msleep(10);
				elan_i2c_send_data(client, enable_cover_cmd, sizeof(enable_cover_cmd));
			}
			else
				printk("[elan]enable cover mode suss probe\n");
			//msleep(20);
		}
		else{
			if ((elan_i2c_send_data(client, disable_cover_cmd, sizeof(disable_cover_cmd))) != sizeof(disable_cover_cmd)) {
				printk("[elan] i2c_master_send disable_cover_cmd failed\n");
				msleep(10);
				elan_i2c_send_data(client, disable_cover_cmd, sizeof(disable_cover_cmd));
			}
			else
				printk("[elan]disable cover mode suss probe\n");
		}
#endif

	proc_create("pen_switch_state", 0666, NULL, &pen_switch);
//add by zhanghui for factory mode touch err 20170320 start
	//if(FACTORY_BOOT==get_boot_mode()||RECOVERY_BOOT==get_boot_mode()){
		//boot_normal_flag=0;
	//}
//add by zhanghui for factory mode touch err 20170320 end
	return 0;

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf_ts_remove(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf_ts_set_power_state(struct i2c_client *client, int state)
{
    //uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	uint8_t cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0x50, 0x00, 0x01};
#ifdef ELAN_IDEL2_CMD
	uint8_t cmd_idel2[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0x2c,0x02,0x01};
	uint8_t cmd_read[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x53,0x50,0x00,0x01};
#endif

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
	cmd[8] |= (state << 3);
	
#ifdef ELAN_IDEL2_CMD
	printk("[elan] dump cmd: %02x, %02x, %02x, %02x\n",cmd_idel2[7],cmd_idel2[8],cmd_idel2[9],cmd_idel2[10]);
	if ((elan_i2c_send_data(client, cmd_idel2, sizeof(cmd_idel2))) != sizeof(cmd_idel2)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
	if ((elan_i2c_send_data(client, cmd_read, sizeof(cmd_read))) != sizeof(cmd_read)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	
#else
	dev_dbg(&client->dev, "[elan] dump cmd: %02x, %02x, %02x, %02x\n", cmd[7], cmd[8], cmd[9], cmd[10]);
	if ((elan_i2c_send_data(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
#endif


	return 0;
}

static int elan_ktf_ts_get_power_state(struct i2c_client *client)
{
    int rc = 0;
    uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
    uint8_t buf[4], power_state;

    rc = elan_ktf_ts_get_data(client, cmd, buf, 4, 4);
    if (rc)
        return rc;

    power_state = buf[1];
    dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
    power_state = (power_state & PWR_STATE_MASK) >> 3;
    dev_dbg(&client->dev, "[elan] power state = %s\n", 
            power_state == PWR_STATE_DEEP_SLEEP ? "Deep Sleep" : "Normal/Idle");

    return power_state;
}

static void elan_release_finger(void)
{
	 struct input_dev *idev = tpd->dev;
	 input_report_key(idev, BTN_TOUCH, 0);
	 input_mt_sync(idev);
	 input_sync(idev);
	 return;
}
static void tpd_suspend(struct device *h)
{
    int rc = 0;
	printk("[elan] %s: enter; power lock:%d\n", __func__,power_lock);
	plam_flag = 0;
    if(power_lock==0)/* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
    {
       // printk(KERN_INFO "[elan] %s: enter\n", __func__);
	   //add XWQYHWSYY-24 by qiang.xue 20171207 start
		mutex_lock(&cmd_done);
		//add XWQYHWSYY-24 by qiang.xue 20171207 end
		suspend_flag=1;
        rc = elan_ktf_ts_set_power_state(private_ts->client, PWR_STATE_DEEP_SLEEP);
		//add XWQYHWSYY-24 by qiang.xue 20171207 start
		mutex_unlock(&cmd_done);
		//add XWQYHWSYY-24 by qiang.xue 20171207 end
		elan_release_finger();
    }
    //return 0;
}

static void tpd_resume(struct device *h)
{

    int rc = 0, retry = 3;
	uint8_t cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0x58, 0x00, 0x01};

#if RLK_LEATHER_MODE
	uint8_t enable_cover_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0xbf,0x00,0x01};
	uint8_t disable_cover_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x54,0xbf,0x00,0x00};
#endif

	uint8_t cmd_len = 37;
#ifdef RE_CALIBRATION
    uint8_t buf_recv[4] = { 0 };
#endif
	mdelay(100);
    if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
    {
		suspend_flag=0;
        printk(KERN_INFO "[elan] %s: enter\n", __func__);
#ifdef ELAN_RESUME_RST
        printk("[elan] %s:command to resume touch panel\n", __func__);
        //elan_ktf_ts_hw_reset();
        // change by yuanhui 20170612 start
		//add XWQYHWSYY-24 by qiang.xue 20171207 start
		mutex_lock(&cmd_done);
		//add XWQYHWSYY-24 by qiang.xue 20171207 end
        if(elan_i2c_send_data(private_ts->client,cmd,cmd_len)!=cmd_len)
		{
			elan_ktf_ts_hw_reset();
			msleep(300);
			switch_pen_state(pen_open_state);
        }
		//add XWQYHWSYY-24 by qiang.xue 20171207 start
		mutex_unlock(&cmd_done);
		//add XWQYHWSYY-24 by qiang.xue 20171207 end
        // change by yuanhui 20170612 end
#if RLK_LEATHER_MODE
		//add XWQYHWSYY-24 by qiang.xue 20171206 start
		//printk("[elan]:%d\n",!(gpio_get_value(hall_gpio)));
		//add XWQYHWSYY-24 by qiang.xue 20171206 end
		if(1==ftp_leather_mode_status) {
			msleep(10);
			if ((elan_i2c_send_data(private_ts->client, enable_cover_cmd, sizeof(enable_cover_cmd))) != sizeof(enable_cover_cmd)) {
				printk("[elan] i2c_master_send enable_cover_cmd failed in rusume\n");
				msleep(10);
				elan_i2c_send_data(private_ts->client, enable_cover_cmd, sizeof(enable_cover_cmd));
			}
			else
				printk("[elan]enable cover mode suss rusume\n");
		}
		else{
			if ((elan_i2c_send_data(private_ts->client, disable_cover_cmd, sizeof(disable_cover_cmd))) != sizeof(disable_cover_cmd)) {
				printk("[elan] i2c_master_send disable_cover_cmd failed in rusume\n");
				msleep(10);
				elan_i2c_send_data(private_ts->client, disable_cover_cmd, sizeof(disable_cover_cmd));
			}
			else
				printk("[elan]disable cover mode suss rusume\n");
		}
#endif
        return;
#endif
        do {
            rc = elan_ktf_ts_set_power_state(private_ts->client, PWR_STATE_NORMAL);
            mdelay(200);
            rc = elan_ktf_ts_get_power_state(private_ts->client);
            if (rc != PWR_STATE_NORMAL)
                printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n", __func__, rc);
            else
                break;
        } while (--retry);

    }
   // return 0;
}

struct pinctrl_state *xpen_output0,*xpen_output1;

int elan_get_gpio_info_for_xpen(void)
{
	int ret;
	printk("[elan]: mt_tpd_pinctrl+++++++++++++++++\n");
    	xpen_output0 = pinctrl_lookup_state(pinctrl1,"state_xpen_output0");
	if(IS_ERR(xpen_output0)){
		ret = PTR_ERR(xpen_output0);
		printk("[elan]:fwq Cannot find touch pinctrl state_xpen_output0!\n");
		return ret;
	}
    	xpen_output1 = pinctrl_lookup_state(pinctrl1,"state_xpen_output1");
	if(IS_ERR(xpen_output1)){
		ret = PTR_ERR(xpen_output1);
		printk("[elan]:fwq Cannot find touch pinctrl state_xpen_output1!\n");
		return ret;
	}		
	printk(" [elan]: mt_tpd_pinctrl----------\n");
	return 0;
}

#if 0
void elan_power_on_dts( int enable )
{
	printk("[elan]:tpd_gpio_output enable = %d",enable);
	if(enable)
		pinctrl_select_state(pinctrl1,xpen_output1);
	else
		pinctrl_select_state(pinctrl1,xpen_output0);
}
#endif

static const struct i2c_device_id elan_ktf_ts_id[] = {
    { ELAN_KTF_NAME, 0 },
    { }
};

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

static int elan_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, ELAN_KTF_NAME);
	return 0;
}
MODULE_DEVICE_TABLE(of, tpd_of_match);

static struct i2c_driver ektf_ts_driver = {
	.probe		= elan_ktf_ts_probe,
	.remove		= elan_ktf_ts_remove,
	.driver = {
		.name = ELAN_KTF_NAME,
		.of_match_table = tpd_of_match,
	},
	.id_table	= elan_ktf_ts_id,
	.detect = elan_i2c_detect,
    
};
static int tpd_local_init(void)
{
	//int ret;
#ifdef ELAN_SUPPORT_I2C_DMA
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va =
	    (u8 *) dma_alloc_coherent(&tpd->dev->dev, ELAN_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va)
		printk("[Error] Allocate DMA I2C Buffer failed!\n");
	memset(gpDMABuf_va, 0, ELAN_DMA_MAX_TRANSACTION_LENGTH);
#endif
#if 0
	if(elan_get_gpio_info_for_xpen()<0){
		printk("mtk_tpd:error get gpio info\n");
		return -1;
	}
#endif
    
	ELAN_DEBUG("mtk_tpd:success get gpio info\n");
	//elan_power_on_dts(1);

	
	
	ELAN_DEBUG(" Device Tree get regulator!");
	#if 0
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	ret = regulator_set_voltage(tpd->reg, 3300000, 3300000);
	if (ret) {
		printk("regulator_set_voltage(%d) failed!\n", ret);
		return -1;
	}
	ELAN_INFO("regulator_set_voltage(%d) suscess!\n", ret);
	ret = regulator_enable(tpd->reg);
    #endif
    tpd_power(1);
    xpen_tpd_power(1);
	
	if (i2c_add_driver(&ektf_ts_driver) != 0) {
		printk("unable to add i2c driver.\n");
		return -1;
	}
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
	tpd_type_cap = 1;
	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = ELAN_KTF_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
	.attrs={
		 .attr = elan_attributes,
         .num  = ARRAY_SIZE(elan_attributes),
	}
};

static int __init elan_ktf_ts_init(void)
{
	TPD_DMESG(KERN_INFO "[elan] %s driver version 0x0005: Integrated 2, 5, and 10 fingers together and auto-mapping resolution\n", __func__);
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		printk("add generic driver failed\n");
	return 0;
	// return i2c_add_driver(&ektf_ts_driver);
}

static void __exit elan_ktf_ts_exit(void)
{
	tpd_driver_remove(&tpd_device_driver);
	return;
}

module_init(elan_ktf_ts_init);
module_exit(elan_ktf_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");



