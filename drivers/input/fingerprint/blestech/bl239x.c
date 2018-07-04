/*
 * Copyright (C) 2016 BetterLife.Co.Ltd. All rights  reserved.
 *
 */

/*
 * Revision: 1.0
Revision: 2.0 modified by sunshizheng@blestech.com
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/signal.h>
#include <linux/ctype.h>
#include <linux/wakelock.h>

#include <linux/jiffies.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/delay.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/notifier.h>

#include "btlcustom.h"
#include "xz_sensor.h"

#define ARCH_MTK_BTL (1)
#define MTK_ANDROID_L (0)
#define CONFIG_PINCTRL_BTL

#if(MTK_ANDROID_L == 1)
#define BTL_USE_WORKQUEUE (1)
#endif

#if(MTK_ANDROID_L != 1)
#include <linux/fb.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#else
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#endif

#define SPI_DRV_NAME	"bl229x"

//#define ENABLE_VKEY_IRQ_CTL
#if(ARCH_MTK_BTL == 1)
#if(MTK_ANDROID_L != 1)
#include "mtk_spi_hal.h"
#include <mtk_spi.h>
#else
#include <mach/mtk_spi.h>
#endif
#endif

#ifndef IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_EDGE_RISING IRQF_TRIGGER_RISING
#endif

#ifndef IRQ_TYPE_EDGE_FALLING
#define IRQ_TYPE_EDGE_FALLING IRQF_TRIGGER_FALLING
#endif


u8 BL229X_HEIGHT = 0;//  FINGER_HEIGHT
u8 BL229X_WIDTH  = 0; // FINGER_WIDTH


u32 bl229x_image_size = 0; //(BL229X_HEIGHT * BL229X_WIDTH)
#define BL229X_SPI_CLOCK_SPEED 6*1000*1000//6*1000*1000//10*1000*1000
u32 DMA_TRANSFER_SIZE = 0; //(((bl229x_image_size / 1024) + 1)*1024)

u32 READIMAGE_BUF_SIZE = 0; //(DMA_TRANSFER_SIZE)
#define BL229X_IOCTL_MAGIC_NO			0xFC

#define BL229X_INIT				        _IO(BL229X_IOCTL_MAGIC_NO, 0)
#define BL229X_GETIMAGE			        _IOW(BL229X_IOCTL_MAGIC_NO, 1, u32)
#define BL229X_INITERRUPT_MODE		    _IOW(BL229X_IOCTL_MAGIC_NO, 2, u32)
#define BL229X_CONTRAST_ADJUST          _IOW(BL229X_IOCTL_MAGIC_NO, 3, u32)
#define BL229X_CONTRAST_ADJUST2		    _IOW(BL229X_IOCTL_MAGIC_NO, 3, u8)

#define BL229X_POWERDOWN_MODE1			_IO (BL229X_IOCTL_MAGIC_NO, 4)
#define BL229X_POWERDOWN_MODE2			_IO (BL229X_IOCTL_MAGIC_NO, 5)

#define BL229X_INTERRUPT_FLAGS1         _IOW(BL229X_IOCTL_MAGIC_NO, 4,  u32)
#define BL229X_INTERRUPT_FLAGS2         _IOW(BL229X_IOCTL_MAGIC_NO, 5,  u32)
#define BL229X_MULTIFUNCTIONAL_KEYCODE	_IOW(BL229X_IOCTL_MAGIC_NO, 6,  u32)
#define BL229X_TEST_MODE	            _IOWR(BL229X_IOCTL_MAGIC_NO, 7, u32)
#define BL229X_GET_ID	                _IOWR(BL229X_IOCTL_MAGIC_NO, 9, u32)
#define BL229X_GET_CHIP_INFO	        _IOWR(BL229X_IOCTL_MAGIC_NO, 10, u32)

#define BL229X_INIT_ARGS	            _IOWR(BL229X_IOCTL_MAGIC_NO, 11, u32)
#define BL229X_GAIN_ADJUST              _IOWR(BL229X_IOCTL_MAGIC_NO, 12, u32)
#define BL229X_ENBACKLIGHT           	_IOW(BL229X_IOCTL_MAGIC_NO, 13, u32)
#define BL229X_ISBACKLIGHT           	_IOWR(BL229X_IOCTL_MAGIC_NO, 14, u32)
#define BL229X_REPORTKEY_NEEDED         _IOWR(BL229X_IOCTL_MAGIC_NO, 15, u32)



#define CHIP_ID_LOW		                (0x83)
#define CHIP_ID_HIGH	                (0x51)
#if(MTK_ANDROID_L != 1)
#define GPIO_OUT_ZERO	                (0)
#define GPIO_OUT_ONE	                (1)
#endif

/*
#define BTL_DEBUG(fmt,arg...)          do{\
if(bl229x_log)\
printk("<btl-dbg>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
}while(0)
 */
#define BTL_DEBUG(fmt, args...)  printk("btl-dbg:%5d: <%s>" fmt "\n",  __LINE__, __func__, ##args)
//#define BTL_DEBUG(fmt, args...)

#define DRIVER_DEBUG            (0)
#define FP_DET_BY_FD            (0)
#define FP_DET_BY_NAV           (1)

#define INT_MO
#define FP_DETECT_METHOD        (FP_DET_BY_FD) //0 FD:mode, 1: NAV MODE


#define RESET_PIN_FAILED	(1)
#define SPI_PIN_FAILED		(2)
#define INT_PIN_FAILED		(3)

#define PRESS_MAX_TIMEOUT_COUNT	    (50)
#define ESD_RECOVERY_RETRY_TIMES    (3)

#define REPORT_DELAY_TIME              (15)

// sensor parameter
#define FINGER_TD_THRED_HIGH_VALUE		(0x4)
#define SENSOR_DEFAULT_GAIN_V2	       (0xB6)	//(0xB6)	//B6
/*#if	(F_CHIP_TYPE == BTL_FP_BF3182)
#define SENSOR_DEFAULT_GAIN_V3		   (0x57)//(0xB5)	
#define SENSOR_CAPTURE_GAIN_V3		   (0x57)//(0x37)	
#define SENSOR_DEFAULT_CONTRAST_V3     (0x50)//(0x8c)	
#elif (F_CHIP_TYPE == BTL_FP_BF3390) 
#define SENSOR_DEFAULT_GAIN_V3		   (0x30)//(0xB5)	
#define SENSOR_CAPTURE_GAIN_V3		   (0x30)//(0x37)	
#define SENSOR_DEFAULT_CONTRAST_V3     (0x8c)//(0x8c)	
#elif (F_CHIP_TYPE == BTL_FP_BF3290) 
#define SENSOR_DEFAULT_GAIN_V3		   (0x57)//(0xB5)	
#define SENSOR_CAPTURE_GAIN_V3		   (0x57)//(0x37)	
#define SENSOR_DEFAULT_CONTRAST_V3     (0x30)//(0x8c)	
#endif*/
u8 SENSOR_DEFAULT_GAIN_V3 = 0;
u8 SENSOR_CAPTURE_GAIN_V3 = 0;
u8 SENSOR_DEFAULT_CONTRAST_V3 = 0;

u8 REGA_FINGER_DT_INTERVAL_LOW = 0x0A;
u8 REGA_FINGER_DT_INTERVAL_HIGH = 0x0B;
u8 REGA_HOST_CMD = 0x13;
u8 REGA_RX_DACP_LOW = 0x1B;
u8 REGA_RX_DACP_HIGH = 0x1C;

#define SENSOR_DEFAULT_GAIN2_V3		   (0x48)//(0xB5)	
// AGC
#define SENSOR_DEFAULT_CONTRAST_V2     (64)

//#define SENSOR_DEFAULT_CONTRAST (0x70)
#define SENSOR_DEFAULT_DACN_V2		   (0x1)
#define SENSOR_DEFAULT_DACN_V3		   (0xff)	// DACN (N should be larger than P)

#define REPORT_KEY_DEBOUNCE_TIME    (HZ/10)

#define INIT_PARAM_SIZE              22
#define OMIT_PIXCEL	(1)

/*----------------------------------------------------------------------------*/
#if(MTK_ANDROID_L == 1)
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mtk_gpio.h>
#define	GPIO_FP_INT_PIN			    (GPIO2 | 0x80000000)//GPIO_FINGER_PRINT_EINT//(GPIO60 | 0x80000000)
#define	CUST_EINT_FP_INT_NUM		2//CUST_EINT_FINGER_NUM
#define	GPIO_FP_INT_PIN_M_EINT		GPIO_MODE_00
#define GPIO_FINGER_PRINT_RESET     (GPIO3 | 0x80000000)
extern void mt_eint_ack(unsigned int eint_num);
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
static void disable_irq_nosync(unsigned int irq)
{
	mt_eint_mask(irq);
}
static void enable_irq(unsigned int irq)
{
	mt_eint_unmask(irq);
}
static void enable_irq_wake(unsigned int irq)
{
}
static inline bool gpio_is_valid(int number)
{
	return number >= 0;
}

static inline int gpio_get_value(unsigned int gpio)
{
	return mt_get_gpio_in(gpio | 0x80000000);
}

static inline int gpio_direction_input(unsigned gpio)
{
	return 	mt_set_gpio_dir((gpio | 0x80000000),GPIO_DIR_IN);
}

//mt_set_gpio_out(GPIO_FINGER_POWER_3V3_EN_PIN, GPIO_OUT_ONE);
static inline int gpio_direction_output(unsigned gpio, int value)
{
	mt_set_gpio_dir((gpio | 0x80000000), GPIO_DIR_OUT);
	return mt_set_gpio_out((gpio | 0x80000000), value);
}

static inline int gpio_request(unsigned gpio, const char *label)
{
	return mt_set_gpio_mode((gpio | 0x80000000), GPIO_MODE_00);
}

static inline int gpio_to_irq(unsigned int gpio)
{
	return gpio;
}
#endif

typedef struct bl229x_chip_info {
	u32 chip_id_num;         //chip id number
	u8 chip_type;            //chip type a/b/c...etc
	u8 chip_driver_type;     //active or inactive
} chip_info_t;

typedef struct bl229x_chip_params {
	u8 *m_chip_name;
	u32 m_contrast;
	u32 m_gain;
	u32 m_gain_capture;
	u32 m_dacn;
} chip_params_t;

typedef enum {
	SENSOR_T_MODE_IDLE = 0,
	SENSOR_T_MODE_FP,
	SENSOR_T_MODE_CAP,
	SENSOR_T_MODE_NAV,
	SENSOR_T_MODE_INI,
	SENSOR_T_MODE_RESET,
} sensor_testmode_t;


typedef enum {
	INT_MODE_CAPTURE  = 0,
	INT_MODE_KEY      = 1,
	INT_MODE_NONE     = 2,
} mode_type;

struct bl229x_data {
	struct spi_device *spi;
	u8 *image_buf;
	u8 *imagetxcmd;
	u8 *imagerxpix;
	u32 reset_gpio;
	u32 irq_gpio;
	u32 irq_num;
	u32 power_en_gpio;
	u32 power1v8_en_gpio;
	u8  interrupt_mode; // 0: none interrupt; 1: interrupt
	u8  is_frame_ready;
	u32 contrast;
	s32 report_key;
	s32 report_delay;
	s32 reset;
	u8  need_report;
	u8  fp_detect_method;

	unsigned long report_timeout;
#if(ARCH_MTK_BTL == 1)
	struct pinctrl *pinctrl1;
	struct pinctrl_state *spi_pins_default;
	struct pinctrl_state *power_en_output0,*power_en_output1;
	struct pinctrl_state *power1v8_en_output0,*power1v8_en_output1;
	struct pinctrl_state *rst_output0,*rst_output1;
	struct pinctrl_state *int_default;
#endif
#ifdef BTL_USE_WORKQUEUE
	struct work_struct fingerprint_work; 
	struct workqueue_struct *fingerprint_workqueue;
#endif
	chip_info_t  chip_info;
	chip_params_t  chip_params;
	struct notifier_block fb_notify;
};

#if(ARCH_MTK_BTL == 1)
static struct mt_chip_conf spi_conf= {

	.setuptime = 10,
	.holdtime = 10,
	.high_time = 8, //\B4˴\A6\BE\F6\B6\A8slk\B5\C4Ƶ\C2\CA
	.low_time =  8,
	.cs_idletime = 20, //10,
	//.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,  //\CF\810\A89\AB\B8\DF\A6\CB
	.tx_mlsb = 1,

	.tx_endian = 0, //tx_endian \B1\ED\810\B65\B4\F3\B6\CB\810\8F0\810\B64
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,


};

/*----------------------------------------------------------------------------*/
#if(MTK_ANDROID_L == 1)
static struct spi_board_info spi_board_bl229x[] __initdata = {
	[0] = {
		.modalias= SPI_DRV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.max_speed_hz = BL229X_SPI_CLOCK_SPEED,
	},
};
#endif

#endif


struct fingerprintd_params_t{
	u8 def_contrast;                   //contrast's default for captruing image
	u8 def_ck_fingerup_contrast;       //contrast's value for checking if finger up
	u8 def_enroll_ck_fingerup_timeout;   
	u8 def_match_ck_fingerup_timeout;                
	u8 def_match_failed_times;         //max failed times for match
	u8 def_enroll_try_times;           //tried times for balance of wet and dry when enroll
	u8 def_match_try_times;            //tried times for balance of wet and dry when match
	u8 def_intensity_threshold;        //intensity's threshold
	u8 def_contrast_high_value;        // high threshold of contrast
	u8 def_contrast_low_value;         // low threshold of contrast
	u8 def_match_quality_score_threshold; //score's threshold of quality, when match
	u8 def_match_quality_area_threshold;  //area's threshold of quality, when match
	u8 def_enroll_quality_score_threshold;//score's threshold of quality, when enroll
	u8 def_enroll_quality_area_threshold; //area's threshold of quality, when match
	u8 def_shortkey_disable;              //short key switch 
	u8 def_far_rate;                          
	u8 def_max_samples;                   // max sample for a finger
	u8 def_debug_enable;
	u8 def_contrast_direction;
	u8 def_update;
	u8 def_step_counts;
	u8 def_algorithm_type;
	u16 g_points_threshold;
	u8 def_enhance_image;
	u8 def_gain;                   //gain default for captruing image
	u8 finger_fd_thred_high;
	u16 chip_id;
};

static struct fingerprintd_params_t fingerprintdParams = {
	.def_contrast = 0,
	.def_ck_fingerup_contrast = 0,
	.def_enroll_ck_fingerup_timeout = 100, //ms
	.def_match_ck_fingerup_timeout = 100,    //ms
	.def_match_failed_times = 5,
	.def_enroll_try_times = 3,
	.def_match_try_times = 3,
	.def_intensity_threshold = 9,
	.def_contrast_high_value = 255,
	.def_contrast_low_value = 1,
	.def_match_quality_score_threshold  = 50,
	.def_match_quality_area_threshold   = 20,
	.def_enroll_quality_score_threshold = 50,
	.def_enroll_quality_area_threshold  = 26,
	.def_shortkey_disable = 2,
	.def_far_rate = 17,
	.def_max_samples = 15,
	.def_debug_enable = DRIVER_DEBUG,
	.def_contrast_direction = 1,
	.def_update = 1,
	.def_step_counts = 4,
	.def_algorithm_type = 16,
	.g_points_threshold = 2000,
	.def_enhance_image = 0,
	.def_gain = 0,
	.finger_fd_thred_high = 0x6,
	.chip_id = 0x5383,
};

/*----------------------------------------------------------------------------*/
static DEFINE_MUTEX(handler_mutex);

#ifdef ENABLE_VKEY_IRQ_CTL
extern void do_delaywork(u8 enable,unsigned long delay);
extern void bl_disable_irqdirect(void);
extern void bl_enable_irqdirect(void);
#endif

static int (*bl229x_dev_init)(struct bl229x_data *bl229x);
static int (*bl229x_read_chipid)(struct bl229x_data *bl229x);
//driver init
static int  mt_spi_init(void);
static void mt_spi_exit(void);
static int  bl229x_probe(struct spi_device *spi);
static int  bl229x_open(struct inode *inode, struct file *file);
static ssize_t bl229x_write(struct file *file, const char *buff,size_t count, loff_t *ppos);
static ssize_t bl229x_read(struct file *file, char *buff,size_t count, loff_t *ppos);
static long    bl229x_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) ;
static int     bl229x_release(struct inode *inode, struct file *file);

//spi func and dev init spi cmd
static int  spi_send_cmd(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen);
static int  spi_read_frame(struct bl229x_data *bl229x);
static int  bl229x_dev_init_3290(struct bl229x_data *spidev);
static int  bl229x_dev_init_3182(struct bl229x_data *spidev);
static int  bl229x_dev_init_3390(struct bl229x_data *spidev);
static int  bl229x_dev_capture_init(struct bl229x_data *bl229x,u32 arg);
static int  bl229x_dev_interrupt_init(struct bl229x_data *bl229x, int navOrFd);
static int  bl229x_contrast_init(struct bl229x_data *bl229x,unsigned long arg);

//hardware init gpio and irq, dma
//static int bl229x_eint_gpio_init(struct bl229x_data *bl229x);
irqreturn_t bl229x_eint_handler(int irq,void *data);
static int bl229x_create_inputdev(void);
static int mtspi_set_dma_en(int mode);
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable);
static int bl229x_gpio_select_and_init(struct bl229x_data *bl229x);
static int bl229x_reset_for_esd(struct bl229x_data *bl229x);
static int bl229x_suspend(struct device *dev);
static int bl229x_resume(struct device *dev);
#if(MTK_ANDROID_L != 1)
static int bl229x_async_fasync(int fd,struct file *filp,int mode);
#endif
static int bl229x_set_testmode(struct bl229x_data *bl229x, u8* params, u8 size);
static int bl229x_read_chipid1(struct bl229x_data *bl229x);
static int bl229x_read_chipid2(struct bl229x_data *bl229x);
int is_connected(struct bl229x_data *bl229x);
static int hw_reset(struct bl229x_data *bl229x);
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data);

/*----------------------------------------------------------------------------*/
static atomic_t suspended;
static struct bl229x_data *g_bl229x= NULL;
static struct input_dev *bl229x_inputdev = NULL;
static struct kobject *bl229x_kobj=NULL;
static u8 bl229x_log = DRIVER_DEBUG;
static struct wake_lock fp_suspend_lock;
static atomic_t is_busy;
//Asynchronous notification struct
#if(MTK_ANDROID_L != 1)
static struct fasync_struct *async_queue;
#endif

static DECLARE_WAIT_QUEUE_HEAD(frame_waiter);
static DECLARE_WAIT_QUEUE_HEAD(waiting_spi_prepare);
/*----------------------------------------------------------------------------*/
//#if (F_CHIP_TYPE == BTL_FP_BF3290) 
#if 1
static void exchange_white_black(u8 *dst,u8 *src,int len)
{
	int i = 0;
	if(fingerprintdParams.chip_id == 0x5183)
	{
		for( ; i < len; i++) {
			*(dst + i) = 0xff & (0xff - *(src + i));
		}	
	}
}
#endif

/*----------------------------------------------------------------------------*/

void spi_io_set_mode(int enable)
{
#if(ARCH_MTK_BTL == 1)
	pinctrl_select_state(g_bl229x->pinctrl1, g_bl229x->spi_pins_default);
#endif
}

/*----------------------------------------------------------------------------*/
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;

	int *blank;


	if (evdata && evdata->data && event == FB_EVENT_BLANK )//&&

	{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			g_bl229x->need_report = 1;
		else if (*blank == FB_BLANK_POWERDOWN)
			g_bl229x->need_report = 2;
	}
	BTL_DEBUG("need report:%d", g_bl229x->need_report);
	return 0;
}

/*----------------------------------------------------------------------------*/

// \810\D45\D4\F1\B9\A4\D7\F7\D3\EB\C4\C7\D6\D6\810\8F0\810\B64
static int mtspi_set_dma_en(int mode)
{
#if(ARCH_MTK_BTL == 1)
	struct mt_chip_conf* spi_par;
	spi_par = &spi_conf;
	if (!spi_par) {
		return -1;
	}
	if (1 == mode) {
		if (spi_par->com_mod == DMA_TRANSFER) {
			return 0;
		}
		spi_par->com_mod = DMA_TRANSFER;
	} else {
		if (spi_par->com_mod == FIFO_TRANSFER) {
			return 0;
		}
		spi_par->com_mod = FIFO_TRANSFER;
	}

	spi_setup(g_bl229x->spi);
#endif
	return 0;
}

/*--------------------------- Data Transfer -----------------------------*/

#if USE_SPI1_4GB_TEST
//static dma_addr_t SpiDmaBufTx_pa;
//static dma_addr_t SpiDmaBufRx_pa;
static char *spi_tx_local_buf;
static char *spi_rx_local_buf;
/*
   static int reserve_memory_spi_fn(struct reserved_mem *rmem)
   {
   printk(" 11111name: %s, base: 0x%llx, size: 0x%llx\n", rmem->name,
   (unsigned long long)rmem->base, (unsigned long long)rmem->size);
   BUG_ON(rmem->size < 0x8000);
   SpiDmaBufTx_pa = rmem->base;
   SpiDmaBufRx_pa = rmem->base+0x4000;
   return 0;
   }
   RESERVEDMEM_OF_DECLARE(reserve_memory_test, "mediatek,spi-reserve-memory", reserve_memory_spi_fn);
 */
static int spi_setup_xfer(struct spi_transfer *xfer)
{
	//u8 tx_buffer = 0x01;
	/* map physical addr to virtual addr */
	if (NULL == spi_tx_local_buf) {
		spi_tx_local_buf = (char *)ioremap_nocache(SpiDmaBufTx_pa, 0x4000);
		if (!spi_tx_local_buf) {
			BTL_DEBUG("SPI Failed to dma_alloc_coherent()\n");
			return -ENOMEM;
		}
	}
	if (NULL == spi_rx_local_buf) {
		spi_rx_local_buf = (char *)ioremap_nocache(SpiDmaBufRx_pa, 0x4000);
		if (!spi_rx_local_buf) {
			BTL_DEBUG("SPI Failed to dma_alloc_coherent()\n");
			return -ENOMEM;
		}
	}
	//xfer->tx_buf = spi_tx_local_buf;
	//xfer->rx_buf = spi_rx_local_buf;
	//xfer->tx_dma = SpiDmaBufTx_pa;
	//xfer->rx_dma = SpiDmaBufRx_pa;
	return 0;

}
#endif

/*----------------------------------------------------------------------------*/
static int spi_send_cmd(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen)
{
	int ret=0;
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
#if USE_SPI1_4GB_TEST
		.tx_dma = SpiDmaBufTx_pa,
		.rx_dma = SpiDmaBufRx_pa,
#endif
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret= spi_sync(bl229x->spi,&m);

	return ret;
}

//-------------------------------------------------------------------------------------------------
static u8 bl229x_spi_read_reg(u8 nRegID)
{
	u8 nAddr;
	u8 data_tx[4];
	u8 data_rx[4];

	nAddr = nRegID << 1;
	nAddr &= 0x7F;

	data_tx[0] = nAddr;
	data_tx[1] = 0xff;

	spi_send_cmd(g_bl229x,data_tx,data_rx,2);
	return data_rx[1];
}


/*----------------------------------------------------------------------------*/
static u8 bl229x_spi_write_reg(u8 nRegID, u8 value)
{
	u8 nAddr;
	u8 data_tx[4];
	u8 data_rx[4];

	nAddr = nRegID << 1;
	nAddr |= 0x80;

	data_tx[0] = nAddr;
	data_tx[1] = value;

	spi_send_cmd(g_bl229x,data_tx,data_rx,2);
	return data_rx[1];
}

/*----------------------------------------------------------------------------*/
  static u8 bl229x_spi_write_reg_bit(u8 nRegID, u8 bit, u8 value)
  {
  u8 tempvalue = 0;

  tempvalue = bl229x_spi_read_reg(nRegID);
  tempvalue &= ~(1 << bit);
  tempvalue |= (value << bit);
  bl229x_spi_write_reg(nRegID,tempvalue);

  return 0;
  }
 
/*----------------------------------------------------------------------------*/
static u8 getSensorInterruptStatus(struct bl229x_data *bl229x)
{
	u8 unStatus = 0;

	unStatus = bl229x_spi_read_reg(REGA_INTR_STATUS);

	BTL_DEBUG("nStatus=%2x",unStatus);

	return unStatus;
}


/*----------------------------------------------------------------------------*/
void hexdump(const unsigned char *buf, const int num)
{
	int i;
	for(i = 0; i < num; i++) {
		printk("%02X ", buf[i]);
		if ((i+1)%8 == 0)
			printk("\n");
	}
	printk("\n");
	return;
}

/*----------------------------------------------------------------------------*/
#if USE_SPI1_4GB_TEST
static int spi_read_frame(struct bl229x_data *bl229x)
{
	u8 nAddr;
	//u8 data_tx[4];
	//u8 data_rx[4];

	BTL_DEBUG("%s++\n",__func__);
	memset(bl229x->image_buf, 0xff, READIMAGE_BUF_SIZE);
	memset(bl229x->imagetxcmd, 0xff, READIMAGE_BUF_SIZE);
	memset(bl229x->imagerxpix, 0xff, READIMAGE_BUF_SIZE);

	//spi_write_reg(REGA_HOST_CMD, MODE_FG_PRINT);
	nAddr = REGA_HOST_CMD << 1;
	nAddr |= 0x80;
	spi_tx_local_buf[0] = nAddr;
	spi_tx_local_buf[1] = MODE_FG_PRINT;
	spi_send_cmd(g_bl229x,spi_tx_local_buf,spi_rx_local_buf,2);


	nAddr = REGA_FINGER_CAP << 1;
	nAddr &= 0x7F;
	spi_tx_local_buf[0] = nAddr;//bl229x->imagetxcmd[0] = nAddr;

	mtspi_set_dma_en(1);
	spi_send_cmd(bl229x,  spi_tx_local_buf,  spi_rx_local_buf, READIMAGE_BUF_SIZE);
	if(fingerprintdParams.chip_id == 0x5183)
	{
		exchange_white_black(bl229x->image_buf,spi_rx_local_buf + OMIT_PIXCEL, DMA_TRANSFER_SIZE - OMIT_PIXCEL);
		memcpy(bl229x->image_buf+((BL229X_HEIGHT-1) * BL229X_WIDTH),bl229x->image_buf+((BL229X_HEIGHT-2) * BL229X_WIDTH),BL229X_WIDTH);
	}else
		memcpy(bl229x->image_buf,spi_rx_local_buf + OMIT_PIXCEL, DMA_TRANSFER_SIZE - OMIT_PIXCEL);

	mtspi_set_dma_en(0);

	BTL_DEBUG("%s--\n",__func__);
	return 0;
}
#else
static int spi_read_frame(struct bl229x_data *bl229x)
{
	u8 nAddr;
	u8 data_tx[4];
	u8 data_rx[4];

	BTL_DEBUG("%s++\n",__func__);
	memset(bl229x->image_buf, 0xff, READIMAGE_BUF_SIZE);
	memset(bl229x->imagetxcmd, 0xff, READIMAGE_BUF_SIZE);
	memset(bl229x->imagerxpix, 0xff, READIMAGE_BUF_SIZE);

	//spi_write_reg(REGA_HOST_CMD, MODE_FG_PRINT);
	nAddr = REGA_HOST_CMD << 1;
	nAddr |= 0x80;
	data_tx[0] = nAddr;
	data_tx[1] = MODE_FG_PRINT;
	spi_send_cmd(g_bl229x,data_tx,data_rx,2);


	nAddr = REGA_FINGER_CAP << 1;
	nAddr &= 0x7F;
	bl229x->imagetxcmd[0] = nAddr;
	mtspi_set_dma_en(1);
	spi_send_cmd(bl229x,  bl229x->imagetxcmd,  bl229x->imagerxpix, READIMAGE_BUF_SIZE);
	if	(fingerprintdParams.chip_id == 0x5183)
	{
		exchange_white_black(bl229x->image_buf,bl229x->imagerxpix + OMIT_PIXCEL, DMA_TRANSFER_SIZE - OMIT_PIXCEL);
		memcpy(bl229x->image_buf+((BL229X_HEIGHT-1) * BL229X_WIDTH),bl229x->image_buf+((BL229X_HEIGHT-2) * BL229X_WIDTH),BL229X_WIDTH);
	}
	else
		memcpy(bl229x->image_buf,bl229x->imagerxpix + OMIT_PIXCEL, DMA_TRANSFER_SIZE - OMIT_PIXCEL);

	mtspi_set_dma_en(0);

	BTL_DEBUG("%s--\n",__func__);
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int hw_reset(struct bl229x_data *bl229x)
{
	//test reset pin
	u32 pin_val= -1;
	bl229x_power_on(bl229x, 0);
	msleep(200);
	pin_val = gpio_get_value(bl229x->reset_gpio);
	if(GPIO_OUT_ZERO != pin_val)
		return -RESET_PIN_FAILED;
	BTL_DEBUG("%s rst pin_val=%d\n",__func__,pin_val);
	bl229x_power_on(bl229x, 1);
	pin_val = gpio_get_value(bl229x->reset_gpio);
	if(GPIO_OUT_ONE != pin_val)
		return -RESET_PIN_FAILED;
	BTL_DEBUG("%s rst pin_val=%d\n",__func__,pin_val);
	msleep(200);
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_key_interrupt(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "\nkey_interrupt=%d\n", g_bl229x->report_key );
}

static ssize_t bl229x_store_key_interrupt(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	g_bl229x->report_key  = simple_strtoul(buf, &next, 10);
	enable_irq(g_bl229x->irq_num);
	return size;
}

static DEVICE_ATTR(key_interrupt, 0664, bl229x_show_key_interrupt, bl229x_store_key_interrupt);

/*----------------------------------------------------------------------------*/
int g_bl229x_enbacklights = 1;
static ssize_t bl229x_show_enbacklight(struct device *ddri,struct device_attribute *attr,char *buf)
{
	//return sprintf(buf, "\nenbacklight=%d\n", g_bl229x->enbacklight);
	return sprintf(buf, "\nenbacklight=%d\n", g_bl229x_enbacklights);
}

static ssize_t bl229x_store_enbacklight(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	//g_bl229x->enbacklight = simple_strtoul(buf, &next, 10);

	g_bl229x_enbacklights = simple_strtoul(buf, &next, 10);
	BTL_DEBUG("g_bl229x_enbacklights = %d",g_bl229x_enbacklights);
	return size;
}

static DEVICE_ATTR(enbacklight, 0664, bl229x_show_enbacklight, bl229x_store_enbacklight);

EXPORT_SYMBOL(g_bl229x_enbacklights);
/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_report_delay(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "\nreport_delay=%d\n", g_bl229x->report_delay);
}

static ssize_t bl229x_store_report_delay(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	g_bl229x->report_delay = simple_strtoul(buf, &next, 10);
	return size;
}

static DEVICE_ATTR(report_delay, 0664, bl229x_show_report_delay, bl229x_store_report_delay);

/*----------------------------------------------------------------------------*/
static int  bl229x_dev_selftest(struct bl229x_data *bl229x)
{
	//\B8\B4λ\D0ź\C5
	u32 pin_val= -1;
	int chip_id = 0;

	BTL_DEBUG("[bl229x]%s:\n\n", __func__);
	//test reset pin
	hw_reset(bl229x);

	pin_val = gpio_get_value(g_bl229x->irq_gpio);
	BTL_DEBUG("%s int pin_val=%d\n",__func__,pin_val);
	if(GPIO_OUT_ONE != pin_val)
		return -INT_PIN_FAILED;

	//test spi pins
	chip_id = (*bl229x_read_chipid)(g_bl229x);
	if(chip_id < 0)
		return -SPI_PIN_FAILED;
	//---------------------------------------------

	if((chip_id == 0x5283)||(chip_id == 0x5383))
	{
		BTL_DEBUG("chip_id=%x",chip_id);
		bl229x_spi_write_reg(0x13,0x40);
		bl229x_spi_write_reg(0x13,0);
	}

	bl229x_spi_write_reg(REGA_HOST_CMD,MODE_IDLE);
	//msleep(g_bl229x->report_delay);
	msleep(100);//modify by jaston  20170628 
	pin_val = gpio_get_value(g_bl229x->irq_gpio);
	BTL_DEBUG("%s int pin_val=%d\n",__func__,pin_val);
	if(GPIO_OUT_ZERO != pin_val)
		return -INT_PIN_FAILED;

	(*bl229x_dev_init)(g_bl229x);
	bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);

	return 0;
}

static ssize_t bl229x_show_selftest(struct device *ddri,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	disable_irq_nosync(g_bl229x->irq_num);
	ret = bl229x_dev_selftest(g_bl229x);
	enable_irq(g_bl229x->irq_num);
	return sprintf(buf, "\nselftest=%d interrupt_mode_flag=%d\n", ret,g_bl229x->interrupt_mode);
}

static ssize_t bl229x_store_selftest(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(selftest, 0664, bl229x_show_selftest, bl229x_store_selftest);

#if(MTK_ANDROID_L != 1)
/*----------------------------------------------------------------------------*/
static int bl229x_async_fasync(int fd,struct file *filp,int mode)
{
	BTL_DEBUG("\n\n");
	return fasync_helper(fd,filp,mode,&async_queue);
}


/*----------------------------------------------------------------------------*/
static void bl229x_async_Report(void)
{
	//Send signal to user space,POLL_IN is enable write
	BTL_DEBUG("\n\n");
	if (async_queue) {
		BTL_DEBUG("bl229x kill_fasync\n ");
		kill_fasync(&async_queue,SIGIO,POLL_IN);
	}
}
#endif

/*----------------------------------------------------------------------------*/
//#if	(F_CHIP_TYPE == BTL_FP_BF3182)|(F_CHIP_TYPE == BTL_FP_BF3390) 
static int bl229x_read_chipid1(struct bl229x_data *bl229x)
{
	u32 id;
	u8 regValue;
	u8 version;

	BTL_DEBUG("  ++\n");
	bl229x_spi_write_reg_bit(0x10,5,0);
	regValue = bl229x_spi_read_reg(0x37);
	id = regValue << 8;
	regValue = bl229x_spi_read_reg(0x36);
	id += regValue;
	version = bl229x_spi_read_reg(0x38);

	BTL_DEBUG(" id=0x%x,version=0x%x\n",id,version);
	fingerprintdParams.chip_id = id;

	if(id == 0x5283)//3182
	{
		BL229X_WIDTH = 72;
		BL229X_HEIGHT = 128;
		SENSOR_DEFAULT_GAIN_V3 = 0x96;//0x56
		SENSOR_CAPTURE_GAIN_V3 = 0x96;//0x56
		SENSOR_DEFAULT_CONTRAST_V3 = 0x50;
		bl229x_dev_init = bl229x_dev_init_3182;
		bl229x_read_chipid = bl229x_read_chipid1;
		REGA_FINGER_DT_INTERVAL_LOW = 0x25;
		REGA_FINGER_DT_INTERVAL_HIGH = 0x26;
		REGA_HOST_CMD = 0x34;
		REGA_RX_DACP_LOW = 0x1B;
		REGA_RX_DACP_HIGH = 0x1C;
	}else if(id == 0x5383)//3390
	{
		BL229X_WIDTH = 80;
		BL229X_HEIGHT = 80;	
		SENSOR_DEFAULT_GAIN_V3 = 0x30;
		SENSOR_CAPTURE_GAIN_V3 = 0x30;
		SENSOR_DEFAULT_CONTRAST_V3 = 0xaa;//modify by jaston 0x8c ---> 0xaa
		bl229x_dev_init = bl229x_dev_init_3390;
		bl229x_read_chipid = bl229x_read_chipid1;
		REGA_FINGER_DT_INTERVAL_LOW = 0x25;
		REGA_FINGER_DT_INTERVAL_HIGH = 0x26;
		REGA_HOST_CMD = 0x34;
		REGA_RX_DACP_LOW = 0x1D;
		REGA_RX_DACP_HIGH = 0x1E;
	}
	return id;
}
//#endif
static int bl229x_read_chipid2(struct bl229x_data *bl229x)
{
	u8 val_low = 0,val_high = 0,chip_type=0xFF,driver_type=0x5A;
	int chip_id = 0;

	u8  old_value = 0;
	u8  active_ic = 0;

	BTL_DEBUG("  ++\n");

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	old_value = bl229x_spi_read_reg(REGA_VERSION_RD_EN);

	bl229x_spi_write_reg(REGA_VERSION_RD_EN, old_value|0x80);

	msleep(g_bl229x->report_delay);

	val_low = bl229x_spi_read_reg(REGA_RC_THRESHOLD_HIGH);//id reg low
	BTL_DEBUG("val_low=0x%x",val_low);
	if(CHIP_ID_LOW != val_low)
		return -SPI_PIN_FAILED;

	val_high = bl229x_spi_read_reg(REGA_FINGER_TD_THRED_LOW);//id reg high
	BTL_DEBUG("val_high=0x%x",val_high);
	if(CHIP_ID_HIGH != val_high)
		return -SPI_PIN_FAILED;

	chip_type = bl229x_spi_read_reg(REGA_FINGER_TD_THRED_HIGH);//ic type
	BTL_DEBUG("chip_type=0x%x",chip_type);

	chip_id =(val_high << 8) | (val_low & 0xff);
	BTL_DEBUG("chip_id=%x",chip_id);

	bl229x_spi_write_reg(REGA_VERSION_RD_EN, old_value);
	bl229x->chip_info.chip_id_num = chip_id;
	bl229x->chip_info.chip_type = chip_type;

	driver_type = bl229x_spi_read_reg(REGA_DRIVER_VERSION);//ic driver type
	active_ic  = driver_type &0x70;
	BTL_DEBUG("11 driver_type=0x%x,active_ic=%d\n",driver_type,active_ic);

	if (!active_ic) {
		driver_type = 0x5A;
		driver_type = bl229x_spi_read_reg(REGA_PUMP_VOLTAGE);//
		active_ic = ((driver_type& 0xF0) ==0x70)?0:0xFF;
	}
	bl229x->chip_info.chip_driver_type = active_ic;
	BTL_DEBUG("active_type=%d\n",active_ic);
	bl229x->interrupt_mode = INT_MODE_NONE;
	fingerprintdParams.chip_id = chip_id;
	if(chip_id == 0x5183)
	{
		BL229X_WIDTH = 112;
		BL229X_HEIGHT = 96;	
		SENSOR_DEFAULT_GAIN_V3 = 0x57;
		SENSOR_CAPTURE_GAIN_V3 = 0x57;
		SENSOR_DEFAULT_CONTRAST_V3 = 0x30;
		bl229x_dev_init = bl229x_dev_init_3290;
		bl229x_read_chipid = bl229x_read_chipid2;
		REGA_FINGER_DT_INTERVAL_LOW = 0x0A;
		REGA_FINGER_DT_INTERVAL_HIGH = 0x0B;
		REGA_HOST_CMD = 0x13;
		REGA_RX_DACP_LOW = 0x1B;
		REGA_RX_DACP_HIGH = 0x1C;
	}
	return chip_id;
}


/*----------------------------------------------------------------------------*/
static int bl229x_reset_for_esd(struct bl229x_data *bl229x)
{
	int recovery_count = 3;
	u8 cur_contrast = 0;
	int ret = 0;
	BTL_DEBUG("  ++\n");
	BTL_DEBUG("REGA_HOST_CMD=%x REGA_RX_DACP_LOW=%x ++",REGA_HOST_CMD,REGA_RX_DACP_LOW);
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	cur_contrast = bl229x_spi_read_reg(REGA_RX_DACP_LOW);
		BTL_DEBUG(" cur_contrast=%x ++\n",cur_contrast);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,cur_contrast);
	
	ret = bl229x_spi_read_reg(REGA_RX_DACP_LOW);
		BTL_DEBUG(" cur_contrast2=%x ++\n",ret);
	if ((ret == cur_contrast) && (cur_contrast != 0)){
		if (gpio_get_value(g_bl229x->irq_gpio) == 0){
			BTL_DEBUG("int gpio is low");
			return 1;
		}	    
	}

	BTL_DEBUG("ret: %d,count = %d\n",ret,recovery_count);

	bl229x->reset = 1;

	while((ret <= 0) && recovery_count--) {
		BTL_DEBUG("hw_reset\n");
		hw_reset(bl229x);
		ret = (*bl229x_read_chipid)(bl229x);
		BTL_DEBUG("recovey_from_esd_failed  recovery_count=%d chip_id=%x\n", recovery_count, ret);
	}
	bl229x->reset = 0;
	BTL_DEBUG("  --\n");
	return 0;
}

int bl229x_dev_inactive_driver_init(struct bl229x_data *bl229x)
{
	int i = 0;
	BTL_DEBUG("+++");
	//0. set to idle
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	// 1. FD threshold[7:0]:  0x11 00 
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_LOW, 0x00);
	// 2. FD threshold[13:8]: 0x12
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, fingerprintdParams.finger_fd_thred_high);
	// 3. 0xa
	bl229x_spi_write_reg(REGA_NAVI_FRM5_LOW, 0);
	// 4. 0xb
	bl229x_spi_write_reg(REGA_NAVI_FRM5_HIGH, 0xa);
	// 5. 0x2
	bl229x_spi_write_reg(REGA_NAVI_FRM1_LOW, 0x11);
	// 6. 
	bl229x_spi_write_reg(0x17, 0x2c);
	bl229x_spi_write_reg(REGA_NAVI_FRM6_HIGH, 0x1b);
	bl229x_spi_write_reg(0x17, 0xac);
	// 7.
	bl229x_spi_write_reg(0x2D, 0xf0);
	// 8.
	bl229x_spi_write_reg(REGA_PUMP_VOLTAGE, 0x75);
	// 9.
	bl229x_spi_write_reg(0x33, 0x92);
	bl229x_spi_write_reg(0x35, 0x52);
	// 10.0x1d--0x28
	for (i = 0x1D; i <= 0x28; i++) {
		bl229x_spi_write_reg(i, bl229x->chip_params.m_dacn);
	}
	// 11.set gain
	bl229x_spi_write_reg(REGA_GC_STAGE,bl229x->chip_params.m_gain);
	// 12.0x48
	bl229x_spi_write_reg(0x32, SENSOR_DEFAULT_GAIN2_V3);
	// 13.0x1b
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,bl229x->chip_params.m_contrast);
	BTL_DEBUG("---");
	return 0;
}


/*----------------------------------------------------------------------------*/
int  bl229x_dev_active_driver_init(struct bl229x_data *bl229x)
{
	// 1 reg from 0x1d to 0x28, write 0x38, do not effect read 0x28 int status.
	int i = 0;
	uint8_t nVal = 0;

	if (bl229x->reset)
	{
		BTL_DEBUG("chip is in reseting\n");
		//return -1;
	}

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	for (i = 0x1D; i <= 0x28; i++) {
		bl229x_spi_write_reg(i, bl229x->chip_params.m_dacn);
	}

	// 2. FD threshold[7:0]: 0x11 00
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_LOW, 0x00);
	// FD threshold[13:8]: 0x12 0x10
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, fingerprintdParams.finger_fd_thred_high);

	// 3.1 decrement FD DT interval, 0x0A - 70ms
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_LOW, 0x00);
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_HIGH, 0x18);

	//bl229x_spi_write_reg(REGA_TIME_INTERVAL_LOW, 0x00);
	//bl229x_spi_write_reg(REGA_TIME_INTERVAL_HIGH, 0x04);//less than 0x15

	// 3.2 32K clock change from 18K to 205K
	//if (bl229x->chip_info.chip_type ==3) { //c type
	bl229x_spi_write_reg(REGA_VERSION_RD_EN, 0x0F);
	//}

	// 3.4 32M fast clock change to (v3.3, x65)->50M,
	//    bl229x_spi_write_reg(0x39, 0x75);

	// 3.5 \u52a0\u5feb\u6a21\u62df\u7535\u8def\u5f00\u542f
	bl229x_spi_write_reg(REGA_NAVI_FRM1_LOW, 0x11);

	// 4 task 1: Frame num to 1 to test
	bl229x_spi_write_reg(0x17, 0x2C);	// prepare to read REGA_FRAME_NUM
	nVal = bl229x_spi_read_reg(REGA_FRAME_NUM);
	nVal &= 0xE7;	// b'11100111  
	bl229x_spi_write_reg(REGA_FRAME_NUM, nVal);


	// 5.1 image background reversing, set reg 0x0E bit[0] to 1
	nVal = bl229x_spi_read_reg(REGA_NAVI_FRM7_LOW);
	nVal &= 0xFE;
	nVal |= 0x01;
	bl229x_spi_write_reg(REGA_NAVI_FRM7_LOW, nVal);

	// 5.2 3 scan rows, close 1,3, set reg 0x2D bit[4] to 1, (pd_rx\u5bc4\u5b58\u5668)
	//BL_WriteSingleCmd(0x2D, 0xF0);

	// 6 set gain, contrast
	bl229x_spi_write_reg(0x32, 0x48);
	bl229x_spi_write_reg(REGA_GC_STAGE,bl229x->chip_params.m_gain);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,bl229x->chip_params.m_contrast);

	return 0;
}


/*
// reg used: 0x10, 0x13, 0x1b, 0x1c, 0x1d, 0x1e, 0x2b, 0x34
// R0D0 - IC revision 0, driver edition 0
int SensorInitFD_5283_R0D0(void)
{
//{ ic_stage1 reg 0x32[4:2] default 0x6d, reduce current.
//BL_WriteSingleCmd(0x32, 0x65);

//  pump voltage
//BL_WriteSingleCmd(0x3D, 0x7F);	// x75->6.5v, default: 0x79->7.5v
//BL_WriteSingleCmd(0x3D, 0x71);	// x75->6.5v, default: 0x79->7.5v
//}

// 1.0 set dacn for finger detect state
BL_WriteSingleCmd(REGA_RX_DACN, g_stSensor.stSensorCfg.nDefaultDacN);

//  set dacn for capturing state
BL_WriteSingleCmd(0x1E, g_stSensor.stSensorCfg.nDefaultDacN);

//  set dacp for finger detect state
BL_WriteSingleCmd(REGA_RX_DACP_LOW, g_stSensor.stSensorCfg.nContrastDefault);

//  set dacp for capturing state
BL_WriteSingleCmd(REGA_RX_DACP_HIGH, g_stSensor.stSensorCfg.nContrastDefault);

// 1.1 set current
//BL_WriteSingleCmd(0x32, 0x52);

// 1.2 open rx
BL_WriteSingleCmd(0x2D, 0x60);	

// 1.3 reg 0x2B[5:3], average number of frame
//BL_WriteSingleCmd(0x2B, 0x00);	// one frame
BL_WriteSingleCmd(0x2B, 0x38);		// 8 frame

// 1.4 reg 0x10 bit[3], pixel value reverse
//	bit[2] 0 - large pixel threshold,  1 - less than.
//	bit[5] 0, AGC disable
//  bit[6] (default 0), ADC reverse, 1\A1\AFb0 -> ADC\CA\E4\C8밴λȡ\B7\B4\A3\BB 1\A1\AFb1->ADC\CA\E4\C8벻ȡ\B7\B4\A3\BB
BL_WriteSingleCmd(0x10, 0x4C);

// 2. FD threshold[7:0]: 0x11 00 
BL_WriteSingleCmd(REGA_FINGER_TD_THRED_LOW, 0x00);

// FD threshold[13:8]: 0x10, 0x3FFF
BL_WriteSingleCmd(REGA_FINGER_TD_THRED_HIGH, 0x01);	// 0x05 / 0x10

// task: 3.1 set finger detect interval
BL_WriteSingleCmd(0x25, 0x00);
BL_WriteSingleCmd(0x26, 0x0A);	// 50ms @default 32K slow clock, 13ms @205K slow clock

//BL_WriteSingleCmd(REGA_FINGER_DT_INTERVAL_LOW, 0x00);
//BL_WriteSingleCmd(REGA_FINGER_DT_INTERVAL_HIGH, 0x0A);	// 50ms @default 32K slow clock, 13ms @205K slow clock
//BL_WriteSingleCmd(REGA_FINGER_DT_INTERVAL_HIGH, 0x06);	// 30ms @default 32K slow clock
//BL_WriteSingleCmd(REGA_FINGER_DT_INTERVAL_HIGH, 0x18);	// 30ms @205K slow clock

// 3.2 32K clock change from 205K
//BL_WriteSingleCmd(REGA_VERSION_RD_EN, 0x0F);

// 3.4 32M fast clock changed to (v3.3, x65)->3.1M, (v2.8, x75)->3.5M
//BL_WriteSingleCmd(0x39, 0x75);


// 5. 3 scan rows, close 1,3, set reg 0x2D bit[4] to 1, including other bits(pd_rx\BCĴ\E6\C6\F7)
//BL_WriteSingleCmd(0x2D, 0xF0);

// 7 set gain, contrast
SensorSetGainContrast(g_stSensor.stSensorCfg.nGainDefault, g_stSensor.stSensorCfg.nContrastDefault);
return 0;
}
 */
static int bl229x_dev_init_3182(struct bl229x_data *bl229x)
{
	// reg used: 0x10, 0x13, 0x1b, 0x1c, 0x1d, 0x1e, 0x2b, 0x34
	// R0D0 - IC revision 0, driver edition 0
	BTL_DEBUG("  ++\n");

	fingerprintdParams.def_contrast = bl229x->chip_params.m_contrast;
	fingerprintdParams.def_ck_fingerup_contrast = bl229x->chip_params.m_contrast;
	bl229x->chip_info.chip_driver_type= 0;

	//\C7\E5\B3\FD\B8\B4λ\D6ж\CF
	bl229x_spi_write_reg(0x13,0x40);
	bl229x_spi_write_reg(0x13,0);
	// 1.0 set dacn for finger detect state
	bl229x_spi_write_reg(REGA_RX_DACN, bl229x->chip_params.m_dacn);

	//  set dacn for capturing state
	bl229x_spi_write_reg(0x1E, bl229x->chip_params.m_dacn);

	//  set dacp for finger detect state
	bl229x_spi_write_reg(REGA_RX_DACP_LOW, bl229x->chip_params.m_contrast);

	//  set dacp for capturing state
	bl229x_spi_write_reg(REGA_RX_DACP_HIGH, fingerprintdParams.def_contrast);

	// 1.1 set current
	//bl229x_spi_write_reg(0x32, 0x52);

	// 1.2 open rx
	bl229x_spi_write_reg(0x2D, 0x60);	

	// 1.3 reg 0x2B[5:3], average number of frame
	//bl229x_spi_write_reg(0x2B, 0x00);	// one frame
	bl229x_spi_write_reg(0x2B, 0x38);		// 8 frame

	// 1.4 reg 0x10 bit[3], pixel value reverse
	//	bit[2] 0 - large pixel threshold,  1 - less than.
	//	bit[5] 0, AGC disable
	//  bit[6] (default 0), ADC reverse, 1\A1\AFb0 -> ADC\CA\E4\C8밴λȡ\B7\B4\A3\BB 1\A1\AFb1->ADC\CA\E4\C8벻ȡ\B7\B4\A3\BB
	bl229x_spi_write_reg(0x10, 0x08);

	// 2. FD threshold[7:0]: 0x11 00 
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_LOW, 0x00);

	// FD threshold[13:8]: 0x10, 0x3FFF
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, fingerprintdParams.finger_fd_thred_high);	// 0x05 / 0x10

	// task: 3.1 set finger detect interval
	bl229x_spi_write_reg(0x25, 0x00);
	bl229x_spi_write_reg(0x26, 0x0A);	// 50ms @default 32K slow clock, 13ms @205K slow clock

	//SensorSetGainContrast(g_stSensor.stSensorCfg.nGainDefault, g_stSensor.stSensorCfg.nContrastDefault);
	// 6 set gain, contrast
	bl229x_spi_write_reg(REGA_GC_STAGE,bl229x->chip_params.m_gain);
	bl229x_spi_write_reg(0x32,0x48);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,bl229x->chip_params.m_contrast);
	return 0;
}

static int bl229x_dev_init_3390(struct bl229x_data *bl229x)
{
	BTL_DEBUG("  ++\n");
	bl229x_spi_write_reg(0x13,0x40);
	bl229x_spi_write_reg(0x13,0);

	bl229x_spi_write_reg(0x2b,0x7b);
	bl229x_spi_write_reg(0x10,0x48);
	bl229x_spi_write_reg(0x11,0x0);
	bl229x_spi_write_reg(0x12,0x6);
	bl229x_spi_write_reg(0x25,0x0);
	bl229x_spi_write_reg(0x26,0xa);
	bl229x_spi_write_reg(0x1b,0xff);
	bl229x_spi_write_reg(0x1c,0xff);
	bl229x_spi_write_reg(0x20,0xff);
	bl229x_spi_write_reg(0x9,0xff);
	bl229x_spi_write_reg(0xa,0xff);
	bl229x_spi_write_reg(0xb,0xff);
	bl229x_spi_write_reg(0xc,0xff);
	bl229x_spi_write_reg(0xd,0xff);
	bl229x_spi_write_reg(0xe,0xff);
	bl229x_spi_write_reg(0xf,0xff);
	bl229x_spi_write_reg(0x1d,0x9e);
	bl229x_spi_write_reg(0x1e,0x9e);
	bl229x_spi_write_reg(0x31,0x38);
	bl229x_spi_write_reg(0x32,0x48);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int bl229x_dev_init_3290(struct bl229x_data *bl229x)
{
	static int fisrt_entor = 1;
	u8 val_low = 0,val_high = 0,chip_type=0xFF,driver_type=0x5A;
	int chip_id = 0;

	u8  old_value = 0;
	u8  active_ic = 0;

	if (bl229x->reset)
	{
		BTL_DEBUG("chip is in reseting\n");
		//return -1;
	}
	if(fisrt_entor == 1)
	{
		bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

		old_value = bl229x_spi_read_reg(REGA_VERSION_RD_EN);
		bl229x_spi_write_reg(REGA_VERSION_RD_EN, old_value|0x80);

		msleep(g_bl229x->report_delay);

		val_low = bl229x_spi_read_reg(REGA_RC_THRESHOLD_HIGH);//id reg low
		val_high = bl229x_spi_read_reg(REGA_FINGER_TD_THRED_LOW);//id reg high
		chip_type = bl229x_spi_read_reg(REGA_FINGER_TD_THRED_HIGH);//ic type

		chip_id =(val_high << 8) | (val_low & 0xff);
		bl229x_spi_write_reg(REGA_VERSION_RD_EN, old_value);

		bl229x->chip_info.chip_type = chip_type;
		bl229x->chip_info.chip_id_num = chip_id;
		BTL_DEBUG("vv chip_type=0x%x,chip_id=0x%x\n",chip_type,chip_id);


		driver_type = bl229x_spi_read_reg(REGA_DRIVER_VERSION);//ic driver type
		active_ic  = driver_type & 0x70;

		if (!active_ic) {
			driver_type = 0x5A;
			driver_type = bl229x_spi_read_reg(REGA_PUMP_VOLTAGE);//
			active_ic = ((driver_type& 0xF0) ==0x70)?0:0xFF;
		}
		BTL_DEBUG("yy driver_type=0x%x,active_ic=%d\n",driver_type,active_ic);
		bl229x->chip_info.chip_driver_type = active_ic;
	}else{
		active_ic = bl229x->chip_info.chip_driver_type;
	}
	BTL_DEBUG("chip_driver_type =%d",active_ic);
	if (active_ic) {
		if(fisrt_entor == 1)
		{
			fisrt_entor = 0;
			bl229x->chip_params.m_contrast =SENSOR_DEFAULT_CONTRAST_V2;
			bl229x->chip_params.m_gain = SENSOR_DEFAULT_GAIN_V2;
			bl229x->chip_params.m_dacn= SENSOR_DEFAULT_DACN_V2;
			fingerprintdParams.def_gain = bl229x->chip_params.m_gain;
			bl229x->chip_params.m_gain_capture = bl229x->chip_params.m_gain;//SENSOR_CAPTURE_GAIN_V3;
			fingerprintdParams.def_contrast = bl229x->chip_params.m_contrast;
			fingerprintdParams.def_ck_fingerup_contrast = bl229x->chip_params.m_contrast;
		}
		bl229x_dev_active_driver_init(bl229x);//hasn't driver ic
	} else { //
		if(fisrt_entor == 1)
		{
			fisrt_entor = 0;
			bl229x->chip_params.m_contrast = SENSOR_DEFAULT_CONTRAST_V3 ;        
			bl229x->chip_params.m_gain = SENSOR_DEFAULT_GAIN_V3;
			bl229x->chip_params.m_gain_capture = bl229x->chip_params.m_gain;//SENSOR_CAPTURE_GAIN_V3;
			fingerprintdParams.def_gain = bl229x->chip_params.m_gain;
			bl229x->chip_params.m_dacn= SENSOR_DEFAULT_DACN_V3;
			fingerprintdParams.def_contrast = bl229x->chip_params.m_contrast;
			fingerprintdParams.def_ck_fingerup_contrast = bl229x->chip_params.m_contrast;
		}

		bl229x_dev_inactive_driver_init(bl229x);//has driver ic
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int bl229x_contrast_init(struct bl229x_data *bl229x,unsigned long arg)
{

	BTL_DEBUG("  ++\n");
	bl229x->contrast = arg; //bl229x_spi_read_reg(REGA_RX_DACP_LOW);

	BTL_DEBUG("  --\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
static int bl229x_gain_init(struct bl229x_data *bl229x,unsigned long arg)
{
	u8 cur_gain = (u8) arg;

	BTL_DEBUG("  ++\n");

	if (bl229x->reset)
	{
		BTL_DEBUG("chip is in reseting\n");
		//return -1;
	}
	bl229x->interrupt_mode = INT_MODE_KEY;
	bl229x->chip_params.m_gain = cur_gain;
	fingerprintdParams.def_gain = cur_gain;
	bl229x->chip_params.m_gain_capture = cur_gain;

	BTL_DEBUG("  --\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int  bl229x_dev_interrupt_init(struct bl229x_data *bl229x, int navOrfp)
{

	u8 val = 0;
	u8 fd_dacp_reg;
	u8 fd_gain_reg = REGA_GC_STAGE;
	BTL_DEBUG("  ++\n");

	if (bl229x->reset)
	{
		BTL_DEBUG("chip is in reseting\n");
		//return -1;
	}
	if	(fingerprintdParams.chip_id == 0x5283)
		fd_dacp_reg = 0x1b;
	else if(fingerprintdParams.chip_id == 0x5383)
		fd_dacp_reg = 0x1d;
	else
		fd_dacp_reg = 0x1b;


	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	val = bl229x_spi_read_reg(REGA_HOST_CMD);
	if (val != MODE_IDLE){
		BTL_DEBUG("err0 %d\n",val);
		bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	}

	bl229x_spi_write_reg(fd_dacp_reg, bl229x->chip_params.m_contrast);
	bl229x_spi_write_reg(fd_gain_reg,bl229x->chip_params.m_gain);
	BTL_DEBUG("chip_params.m_contrast %d\n",bl229x->chip_params.m_contrast);
	val = bl229x_spi_read_reg(fd_dacp_reg);
	if (val != bl229x->chip_params.m_contrast) {
		BTL_DEBUG("err1 %d\n",val);
		bl229x_spi_write_reg(fd_dacp_reg, bl229x->chip_params.m_contrast);
	}

	bl229x->interrupt_mode = INT_MODE_KEY;

	if (navOrfp)
		bl229x_spi_write_reg(REGA_HOST_CMD, MODE_NAVI);
	else
		bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_DT);

	BTL_DEBUG("  --\n");
	return 0;
}



/*----------------------------------------------------------------------------*/
static int bl229x_dev_capture_init(struct bl229x_data *bl229x,u32 arg)
{
	u8 fd_dacp_reg;
	if	(fingerprintdParams.chip_id == 0x5283)
		fd_dacp_reg = 0x1c;
	else if(fingerprintdParams.chip_id == 0x5383)
		fd_dacp_reg = 0x1e;
	else
		fd_dacp_reg = 0x1b;

	BTL_DEBUG("  ++\n");

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	bl229x->interrupt_mode = INT_MODE_CAPTURE;
	bl229x_spi_write_reg(fd_dacp_reg, bl229x->contrast);

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_CAP);

	BTL_DEBUG("  --\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
static int bl229x_set_testmode(struct bl229x_data *bl229x, u8 * params, u8 size)
{
	int testType;
	int i;
	int workmode;

	BTL_DEBUG("  ++\n");


	for (i = 0; i < size; i++)
		BTL_DEBUG("%d\n",params[i]);


	testType = params[0];
	switch (testType){
		case 1:  // mode switch
			workmode = params[1];
			if (workmode == MODE_IDLE)
				bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
			else if (workmode == MODE_FG_DT)
				bl229x_dev_interrupt_init(bl229x,FP_DET_BY_FD);
			else if (workmode == MODE_FG_CAP)
				bl229x_dev_capture_init(bl229x,10);
			else if (workmode == MODE_NAVI)
				bl229x_dev_interrupt_init(bl229x,FP_DET_BY_NAV);
			break;
		case 2:  // init action 
			if (params[1] == 1)
				(*bl229x_dev_init)(bl229x);
			else if (params[1] == 2){
				(*bl229x_dev_init)(bl229x);
				bl229x_dev_interrupt_init(bl229x,0);
			}else if (params[1] == 2){
				(*bl229x_dev_init)(bl229x);
				bl229x_dev_interrupt_init(bl229x,1);
			}
			break;
		case 3:  // enable or disabel interrupt
			if (params[1] == 1)
				enable_irq(bl229x->irq_num);  
			else 
				disable_irq_nosync(bl229x->irq_num);
			break;
		case 4:  //reset
			if (params[1] == 1){
				hw_reset(bl229x);
				(*bl229x_dev_init)(bl229x);
			}else if (params[1] == 2){
				hw_reset(bl229x);
				(*bl229x_dev_init)(bl229x);
				bl229x_dev_interrupt_init(bl229x,0);
			}else if (params[1] == 3){
				hw_reset(bl229x);
				(*bl229x_dev_init)(bl229x);
				bl229x_dev_interrupt_init(bl229x,1);
			}
			break;
		default:
			break;

	}

	BTL_DEBUG("  --\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
int imageCheckEmpty(uint8_t* pImage, uint8_t value_threshold, int32_t counts_threshold, int size)
{
	int32_t imagepix_empty  = 0;
	int32_t imagepix_content = 0;
	int32_t i=0;

	if (counts_threshold > size)
		counts_threshold = size;

	for (i=0; i < size; i++) {
		if (pImage[i] <  value_threshold)
			imagepix_content++;
		else
			imagepix_empty++;

		if (imagepix_content >= counts_threshold)
			return 0;
	}
	BTL_DEBUG("%s,%d,%d",__func__,imagepix_empty,imagepix_content);
	return -1;
}

/*----------------------------------------------------------------------------
  p1---a1---p2
  b1---c ---b2
  p3---a2---p4
 */
struct point_data{
	int x;
	int y;
	int meanvalue;
};
struct point_data touch_area[5];
static struct point_data start_tmp;

static int touch_index = 0;
static int dircount1 = 0;
static int dircount2 = 0;
static int gDirection = 0;
static int dircount3 = 0;
static int dircount4 = 0;

#define PRESS_PIXCEL_VAL (255 - fingerprintdParams.finger_fd_thred_high*4)//(240)
#define HIGH_PRESS_PIXCEL_VAL (30)
#define EMPTY_PIXCEL_COUNT (32)//(112*10)
#define MOVING_DELTA (20)
#define MOVING_DELTA_LOW (MOVING_DELTA/2)
#define SAME_MOVING_DELTA (3)

#define W_HIGH	(4)
#define W_MIDDLE	(2)
#define W_LOW	(1)
#define FRAME_COUNT_LONG_PRESS	(2)
#define FRAME_COUNT_SAME_PRESS	(3)
static int m_is_high_press = 0;
static int m_is_positive_x = 0;
static int m_is_positive_y = 0;
static int m_is_long_press = 0;
static int m_long_press_count = 0;
static int m_press_time_by_count = 0;
static int m_same_press = 0;
static int m_dx_max = 0;
static int m_dy_max = 0;

int cal_touch_area_point(struct bl229x_data *bl229x)
{
	char *buf = bl229x->image_buf;
	int i,j;
	int sum_x = 0;
	int sum_y = 0;
	int area_count = 0;
	int index_pre = 0;
	int mMeanValue = 0;
	int pixcelvalue = 0;
	int tmp_dx = 0;
	int tmp_dy = 0;	
	int tmp_dx2 = 0;
	int tmp_dy2 = 0;	


	BTL_DEBUG("++++");
	for(i = 0;i < BL229X_HEIGHT - 1; i++)//ȥ\B5\F4\BAڱ\DF
	{
		for(j = 0;j < BL229X_WIDTH; j++)
		{
			//BTL_DEBUG("index=%d,j=%d,i=%d",touch_index,j,i);
			pixcelvalue = *(buf + i*BL229X_WIDTH + j);
			mMeanValue += pixcelvalue;
			if(pixcelvalue < PRESS_PIXCEL_VAL)
			{
				sum_x += j;
				sum_y += i;	
				area_count++;	
				//BTL_DEBUG("index=%d,j=%d,i=%d",touch_index,j,i);	
			}
		}
	}
	mMeanValue = mMeanValue/BL229X_WIDTH/BL229X_HEIGHT;
	if(area_count > 0)
	{
		touch_area[touch_index].x = sum_x/area_count;
		touch_area[touch_index].y = sum_y/area_count;
		touch_area[touch_index].meanvalue = mMeanValue;
		BTL_DEBUG("index=%d,x=%d,y=%d,area_count=%d mMeanValue=%d",touch_index,touch_area[touch_index].x,touch_area[touch_index].y,area_count,mMeanValue);

	}
	if(area_count < EMPTY_PIXCEL_COUNT)
	{
		BTL_DEBUG("up area_count=%d,m_is_high_press=%d",area_count,m_is_high_press);
		BTL_DEBUG("time_by_count=%d,long_press=%d n1=%d,n2=%d,n3=%d,n4=%d,dx=%d,dy=%d==a",m_press_time_by_count,m_long_press_count,dircount1,dircount2,dircount3,dircount4,m_dx_max,m_dy_max);
		return -1;
	}//add  by jaston 20170628 start
     else 
     
     return  0 ;
#if 0
	if(touch_index == 0)
	{
		m_is_high_press = 0;
		m_is_long_press = 0;
		m_is_positive_x = 0;
		m_is_positive_y = 0;
		m_long_press_count = 0;
		m_press_time_by_count = 1;
		m_same_press = 0;
		m_dx_max = 0;
		m_dy_max = 0;
		dircount1 = 0;
		dircount2 = 0;
		dircount3 = 0;
		dircount4 = 0;
		start_tmp.x = touch_area[touch_index].x;
		start_tmp.y = touch_area[touch_index].y;
	}	
	else
	{
		m_press_time_by_count++;
		tmp_dx = touch_area[touch_index].x - touch_area[0].x;
		tmp_dy = touch_area[touch_index].y - touch_area[0].y;
		if(tmp_dx > 0)
		{	
			if(tmp_dx > m_dx_max)
			{
				m_dx_max = tmp_dx;
				m_is_positive_x = 1;
			}
		}
		else
		{
			if(-tmp_dx > m_dx_max)
			{
				m_dx_max = -tmp_dx;
				m_is_positive_x = 0;
			}
		}
		if(tmp_dy > 0)
		{
			if(tmp_dy > m_dy_max)
			{
				m_dy_max = tmp_dy;
				m_is_positive_y = 1;
			}
		}
		else
		{
			if(-tmp_dy > m_dy_max)
			{
				m_dy_max = -tmp_dy;
				m_is_positive_y = 0;
			}
		}

		index_pre = (touch_index - 1) %5;
		if(index_pre == 0)
			index_pre = 4;
		tmp_dx2 = touch_area[touch_index].x - touch_area[index_pre].x;
		tmp_dy2 = touch_area[touch_index].y - touch_area[index_pre].y;

		if((-SAME_MOVING_DELTA < tmp_dx2)&& (tmp_dx2 < SAME_MOVING_DELTA))
		{
			if((-SAME_MOVING_DELTA < tmp_dy2)&& (tmp_dy2 < SAME_MOVING_DELTA))
			{
				if(m_same_press == 0)
				{
					m_same_press = 1;
					m_long_press_count = 0;
				}
				else if(m_same_press == 1)
				{
					m_long_press_count++;
					if(m_long_press_count > FRAME_COUNT_SAME_PRESS)
						m_is_long_press = 1;
				}
			}
			else
			{
				m_same_press = 0;
			}
		}
		else
		{
			m_same_press = 0;
		}

		BTL_DEBUG("time_by_count=%d,long_press=%d n1=%d,n2=%d,n3=%d,n4=%d==a",m_press_time_by_count,m_long_press_count,dircount1,dircount2,dircount3,dircount4);
		gDirection = 0;
		if((m_press_time_by_count >= FRAME_COUNT_LONG_PRESS)&&(!m_is_long_press))
		{
			if((m_dx_max > MOVING_DELTA)||(m_dy_max > MOVING_DELTA))
			{
				if(m_dx_max > m_dy_max){
					if(m_is_positive_x == 1){
						dircount3 += W_HIGH;
						gDirection = 3;
					}
					else{
						dircount4 += W_HIGH;
						gDirection = 4;
					}
				}
				else{
					if(m_is_positive_y == 1){
						dircount1 += W_HIGH;
						gDirection = 1;
					}
					else{
						dircount2 += W_HIGH;
						gDirection = 2;
					}
				}
			}
		}
	}
	if(touch_area[touch_index].meanvalue < HIGH_PRESS_PIXCEL_VAL)
	{
		m_is_high_press = 1;
	}
	touch_index++;
	if((touch_index%5) == 0)
	{
		touch_index = 1;
	}
#endif 
//add  by jaston 20170628 end

	BTL_DEBUG("---");
	return 0;
}

/* -------------------------------------------------------------------- */
#ifdef CONFIG_PINCTRL_BTL
static int bl229x_parse_dt_pinstates(struct device *dev,struct bl229x_data *pdata)
{
	int ret;
	struct pinctrl *pinctrl1 = pdata->pinctrl1;
	struct pinctrl_state *spi_pins_default = pdata->spi_pins_default;
	struct pinctrl_state *power_en_output0 = pdata->power_en_output0;
	struct pinctrl_state *power_en_output1 = pdata->power_en_output1;
	struct pinctrl_state *power1v8_en_output0 = pdata->power1v8_en_output0;
	struct pinctrl_state *power1v8_en_output1 = pdata->power1v8_en_output1;
	struct pinctrl_state *rst_output0 = pdata->rst_output0;
	struct pinctrl_state *rst_output1 = pdata->rst_output1;
	struct pinctrl_state *int_default = pdata->int_default;

	BTL_DEBUG("bl229x_pinctrl+++++++++++++++++\n");
	pinctrl1 = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl1!\n");
		return ret;
	}
	spi_pins_default = pinctrl_lookup_state(pinctrl1, "spi0_default");
	if (IS_ERR(pdata->spi_pins_default)) {
		ret = PTR_ERR(pdata->spi_pins_default);
		dev_err(dev, "fwq Cannot find bl229x pinctrl default %d!\n", ret);
	}
	rst_output1 = pinctrl_lookup_state(pinctrl1, "rst_output1");
	if (IS_ERR(rst_output1)) {
		ret = PTR_ERR(rst_output1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl rst_output1!\n");
	}
	rst_output0 = pinctrl_lookup_state(pinctrl1, "rst_output0");
	if (IS_ERR(rst_output0)) {
		ret = PTR_ERR(rst_output0);
		dev_err(dev, "fwq Cannot find bl229x pinctrl rst_output0!\n");
	}
	power1v8_en_output1 = pinctrl_lookup_state(pinctrl1, "power1v8_en_output1");
	if (IS_ERR(power1v8_en_output1)) {
		ret = PTR_ERR(power1v8_en_output1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl power1v8_en_output1!\n");
	}
	power1v8_en_output0 = pinctrl_lookup_state(pinctrl1, "power1v8_en_output0");
	if (IS_ERR(power1v8_en_output0)) {
		ret = PTR_ERR(power1v8_en_output0);
		dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output0!\n");
	}

	power_en_output1 = pinctrl_lookup_state(pinctrl1, "power_en_output1");
	if (IS_ERR(power_en_output1)) {
		ret = PTR_ERR(power_en_output1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output1!\n");
	}
	power_en_output0 = pinctrl_lookup_state(pinctrl1, "power_en_output0");
	if (IS_ERR(power_en_output0)) {
		ret = PTR_ERR(power_en_output0);
		dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output0!\n");
	}
	int_default = pinctrl_lookup_state(pinctrl1, "int_default");
	if (IS_ERR(int_default)) {
		ret = PTR_ERR(int_default);
		dev_err(dev, "fwq Cannot find bl229x pinctrl int_default!\n");
	}

	pdata->pinctrl1 = pinctrl1;
	pdata->spi_pins_default = spi_pins_default;
	pdata->power_en_output0 = power_en_output0;
	pdata->power_en_output1 = power_en_output1;
	pdata->power1v8_en_output0 = power1v8_en_output0;
	pdata->power1v8_en_output1 = power1v8_en_output1;
	pdata->rst_output0 = rst_output0;
	pdata->rst_output1 = rst_output1;
	pdata->int_default = int_default;

	pinctrl_select_state(pinctrl1, spi_pins_default);
	pinctrl_select_state(pinctrl1, rst_output1);
	pinctrl_select_state(pinctrl1, power_en_output1);
	pinctrl_select_state(pinctrl1, power1v8_en_output1);
	pinctrl_select_state(pinctrl1, int_default);

	BTL_DEBUG("bl229x_pinctrl----------\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
//\B5\E7\810\E96\BF\AA\B9\D8AVDD\A3\A82.6V-3.6V\A3\A9\A3\ACDVDD\A3\A81.8V\A3\A9\A3\ACIOVDD\A3\A81.8V or 2.8V\A3\A9,RST/SHUTDOWN pull high
//ESD recovery have to power off, AVDD must under control
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable)
{
	if(enable) {
		if (!IS_ERR(bl229x->rst_output1)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->rst_output1);
		}
		if (!IS_ERR(bl229x->power_en_output1)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->power_en_output1);
		}
		if (!IS_ERR(bl229x->power1v8_en_output1)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->power1v8_en_output1);
		}
	} else {
		if (!IS_ERR(bl229x->rst_output0)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->rst_output0);
		}
		if (!IS_ERR(bl229x->power_en_output0)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->power_en_output0);
		}
		if (!IS_ERR(bl229x->power1v8_en_output0)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->power1v8_en_output0);
		}
	}

	return 0;
}
#else
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable)
{
	if(enable) {
		
		gpio_direction_output(bl229x->power_en_gpio, 1);
		gpio_set_value(bl229x->power_en_gpio, 1)
			;
		gpio_direction_output(bl229x->power1v8_en_gpio, 1);
		gpio_set_value(bl229x->power1v8_en_gpio, 1);
		
		gpio_direction_output(bl229x->reset_gpio, 1);
		gpio_set_value(bl229x->reset_gpio, 1);
		
	} else {
	
		gpio_direction_output(bl229x->reset_gpio, 0);
		gpio_set_value(bl229x->reset_gpio, 0);
	
		gpio_direction_output(bl229x->power_en_gpio, 0);
		gpio_set_value(bl229x->power_en_gpio, 0);

		
		gpio_direction_output(bl229x->power1v8_en_gpio, 0);
		gpio_set_value(bl229x->power1v8_en_gpio, 0);
		
		
		
	}

	return 0;
}
#endif
#ifdef CONFIG_OF
static int bl229x_parse_dt_gpio(struct device *dev,struct bl229x_data *pdata)
{

	dev_err(dev, "bl229x_parse_dt_gpio\n");
    pdata->reset_gpio = of_get_named_gpio_flags(dev->of_node,
                        "fingerprint,rst-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->reset_gpio))
		return -EINVAL;

    pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
                      "fingerprint,touch-int-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->irq_gpio))
		return -EINVAL;
    printk("pdata->irq_gpio = %d --pdata->reset_gpio = %d \n", pdata->irq_gpio, pdata->reset_gpio);
    pdata->power_en_gpio = of_get_named_gpio_flags(dev->of_node,
                           "fingerprint,en-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->power_en_gpio))
		return -EINVAL;

	pdata->power1v8_en_gpio = of_get_named_gpio_flags(dev->of_node,
                           "fingerprint,en1v8-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->power1v8_en_gpio))
		return -EINVAL;

	dev_err(dev, "bl229x_parse_dt_gpio out\n");

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int bl229x_gpio_select_and_init(struct bl229x_data *bl229x)
{
	int error = 0;
#ifdef CONFIG_PINCTRL_BTL
	bl229x_parse_dt_pinstates(&bl229x->spi->dev, bl229x);
#endif
	bl229x_parse_dt_gpio(&bl229x->spi->dev, bl229x);

#if(MTK_ANDROID_L == 1)
	bl229x->irq_gpio = GPIO_FP_INT_PIN ;
	bl229x->reset_gpio = GPIO_FINGER_PRINT_RESET ;
	bl229x->irq_num = CUST_EINT_FP_INT_NUM ;
#else
	bl229x->irq_num = gpio_to_irq(bl229x->irq_gpio);
#endif

#if  defined(CONFIG_OF)&&!defined(CONFIG_PINCTRL_BTL)
	if (gpio_is_valid(bl229x->reset_gpio)) {
		error = gpio_request(bl229x->reset_gpio, "FINGERPRINT_RST");
		if (error) {
			dev_err(&bl229x->spi->dev, "unable to request GPIO %d\n",bl229x->reset_gpio);
		}
		gpio_direction_output(bl229x->reset_gpio, 1);
	}
	if (gpio_is_valid(bl229x->power_en_gpio)) {
		error = gpio_request(bl229x->power_en_gpio, "FINGERPRINT_3V3_EN");
		if (error) {
			dev_err(&bl229x->spi->dev, "unable to request GPIO %d\n",bl229x->power_en_gpio);
		}
		gpio_direction_output(bl229x->power_en_gpio, 1);
		dev_err(&bl229x->spi->dev, "power_en_gpio\n");
	}

	
	if (gpio_is_valid(bl229x->power1v8_en_gpio)) {
		error = gpio_request(bl229x->power1v8_en_gpio, "FINGERPRINT_1V8_EN");
		if (error) {
			dev_err(&bl229x->spi->dev, "unable to request GPIO %d\n",bl229x->power1v8_en_gpio);
		}
		gpio_direction_output(bl229x->power1v8_en_gpio, 1);
		dev_err(&bl229x->spi->dev, "power1v8_en_gpio\n");
	}

	if (gpio_is_valid(bl229x->irq_gpio)) {
		error = gpio_request(bl229x->irq_gpio, "FINGERPRINT-IRQ");
		if (error) {
			dev_err(&bl229x->spi->dev, "unable to request GPIO %d\n",bl229x->irq_gpio);
			goto err;
		}
		error = gpio_direction_input(bl229x->irq_gpio);
		if (error) {
			dev_err(&bl229x->spi->dev, "set_direction for irq gpio failed\n");
			goto err;
		}
	}
err:
#endif
	return error;
}

/*----------------------------------------------------------------------------*/

static int  bl229x_dev_reset(struct bl229x_data *bl229x)
{
	//\B8\B4λ\D0ź\C5
	u32 pin_val= -1;
	int chip_id = 0;

	BTL_DEBUG("[bl229x]%s:\n\n", __func__);

	bl229x->chip_info.chip_type = 0;
	bl229x->chip_info.chip_driver_type = 0;
	//test reset pin
	hw_reset(bl229x);
	if((fingerprintdParams.chip_id == 0x5283)||(fingerprintdParams.chip_id == 0x5383))
	{//\C7\E5\B3\FD\B8\B4λ\D6ж\CF
		bl229x_spi_write_reg(0x13,0x40);
		bl229x_spi_write_reg(0x13,0);
	}
	//test spi pins
	chip_id = (*bl229x_read_chipid)(g_bl229x);
	if(chip_id < 0)
		return -SPI_PIN_FAILED;
	//---------------------------------------------
	bl229x_spi_write_reg(REGA_HOST_CMD,MODE_IDLE);

	// bl229x_spi_write_reg(REGA_HOST_CMD,MODE_FG_CAP);
	msleep(g_bl229x->report_delay);
	pin_val = gpio_get_value(g_bl229x->irq_gpio);
	BTL_DEBUG("%s int pin_val=%d\n",__func__,pin_val);
	if(GPIO_OUT_ZERO != pin_val)
		return -INT_PIN_FAILED;
	(*bl229x_dev_init)(g_bl229x);
	bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);

	return 0;
}

static ssize_t bl229x_show_debug(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"debug=%d irq_gpio = %d,reset_gpio=%d,irq_num=%d\n",bl229x_log,g_bl229x->irq_gpio,g_bl229x->reset_gpio,g_bl229x->irq_num);
}
static ssize_t bl229x_store_debug(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	bl229x_log = simple_strtoul(buf, NULL, 16);
	BTL_DEBUG("bl229x_log=0x%x",bl229x_log);
	return size;
}
static DEVICE_ATTR(debug,0664,bl229x_show_debug,bl229x_store_debug);

static u8 spicmd_rsp[65];
static ssize_t bl229x_show_register(struct device *ddri,struct device_attribute *attr,char *buf)
{

	int count = 0;
	int buflen = 0;
	int param_size = spicmd_rsp[0];
	for(count = 0; count < param_size; count++) {
		buflen += sprintf(buf + buflen, "rsp[%d]=%x\n", count, spicmd_rsp[count+1]);
	}

	return param_size;

}
static ssize_t bl229x_store_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	int count=0;
	u8 param[64];
	u8 w = 0;
	u8 tempBuf[64];
	u8 param_size;


	memset(tempBuf,0,64);
	memcpy(tempBuf,buf,size);
	memset(spicmd_rsp,0,65);
	BTL_DEBUG("string:%s \n",tempBuf);

	w =  simple_strtoul(tempBuf, &next, 16);
	BTL_DEBUG("w:%d \n",w);
	BTL_DEBUG("next=%s \n ",next);
	if (w)
		param_size = 2;
	else 
		param_size = 1;
	BTL_DEBUG("spicmd len=%d \n",param_size);

	for (count = 0; count < param_size; count++) {
		while(!isxdigit(*next) && *next != 0)
			next++;
		if (*next == 0) break;

		param[count] = simple_strtoul(next, &next, 16);
		BTL_DEBUG("param[%d]=%x \n ",count,param[count]);
		BTL_DEBUG("next=%s \n ",next);
	}

	if (count != param_size) {
		BTL_DEBUG("format error -2  \n");
		return 1;
	}

	param[0] = param[0] << 1;
	if (w)
		param[0] |= 0x80;
	else
		param[0] &= 0x7f;

	if (param_size > 64)
		return 1;

	for(count = 0; count < param_size; count++) {
		BTL_DEBUG("param[%d]=%x\n", count,param[count]);
	}

	spicmd_rsp[0] = param_size;

	spi_send_cmd(g_bl229x, param, &spicmd_rsp[1], count);

	for(count = 0; count < param_size; count++) {
		BTL_DEBUG("rsp[%d]=%x\n", count, spicmd_rsp[count+1]);
	}

	return size;
}
static DEVICE_ATTR(register,0664,bl229x_show_register,bl229x_store_register);

/*----------------------------------------------------------------------------*/
static u8 reg_rsp[256];
static ssize_t bl229x_show_reg(struct device *ddri,struct device_attribute *attr,char *buf)
{
	int count = 0;
	int buflen = 0;
	int param_size = reg_rsp[0];
	for(count = 0; count < param_size; count++) {
		buflen += sprintf(buf + buflen, "rsp[%d]=%x\n", count, reg_rsp[count+1]);
	}

	return buflen;
}

static ssize_t bl229x_store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	int count=0;
	u8 param[256];
	u8 w = 0;
	u8 tempBuf[128];
	u8 param_size;


	memset(tempBuf,0,128); 
	memcpy(tempBuf,buf,size);
	BTL_DEBUG("string:%s \n",tempBuf);


	w =  simple_strtoul(tempBuf, &next, 16);
	BTL_DEBUG("w:%d \n",w);
	BTL_DEBUG("next=%s \n ",next);
	while(!isxdigit(*next) && *next != 0)next++;
	if (*next == 0) {
		BTL_DEBUG("format error \n");
		reg_rsp[0] = 0;
		return 1; 
	}
	param_size = simple_strtoul(next, &next, 16);
	BTL_DEBUG("reg len=%d \n",param_size);

	for (count = 0; count < param_size; count++){	
		while(!isxdigit(*next) && *next != 0) 
			next++;
		if (*next == 0) break;

		param[count] = simple_strtoul(next, &next, 16);
		BTL_DEBUG("param[%d]=%x \n ",count,param[count]);
		BTL_DEBUG("next=%s \n ",next);
	}

	if (count != param_size){
		BTL_DEBUG("format error -2  \n");
		reg_rsp[0] = 0;
		return 1; 
	}

	param[0] = param[0] << 1;
	if (w)
		param[0] |= 0x80;
	else 
		param[0] &= 0x7f;

	for(count = 0; count < param_size; count++) {
		BTL_DEBUG("param[%d]=%x\n", count,param[count]);
	}

	reg_rsp[0] = param_size;

	spi_send_cmd(g_bl229x, param, &reg_rsp[1], count);

	for(count = 0; count < param_size; count++) {
		BTL_DEBUG("rsp[%d]=%x\n", count, reg_rsp[count+1]);
	}

	return size;
}
static DEVICE_ATTR(reg, 0664, bl229x_show_reg, bl229x_store_reg);


static ssize_t bl229x_show_mode(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"idle:0,fd:2,cap:4,nav:5 current cmd reg:0x%x",bl229x_spi_read_reg(REGA_HOST_CMD));
}

static ssize_t bl229x_store_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 register_value = simple_strtoul(buf, NULL, 16);

	BTL_DEBUG("host cmd reg:%d",register_value);

	switch (register_value) {
		case MODE_IDLE:
			bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
			break;
		case MODE_NAVI:
		case MODE_FG_DT:
			bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);
			break;
		case MODE_FG_CAP:
			bl229x_dev_capture_init(g_bl229x,10);
			break;
		default:
			break;

	}
	return size;
}
static DEVICE_ATTR(mode,0664,bl229x_show_mode,bl229x_store_mode);

static ssize_t bl229x_show_image(struct device *ddri,struct device_attribute *attr,char *buf)
{
	int i;
	for(i = 0; i < DMA_TRANSFER_SIZE; i++) {
		printk("%02X ", g_bl229x->image_buf[i]);
		if ((i+1)%16 == 0)
			printk("\n");
	}
	printk("\n");
	return sprintf(buf,"look image's content by log \n");
}
static DEVICE_ATTR(image,0664,bl229x_show_image,NULL);

static ssize_t bl229x_show_init(struct device *ddri,struct device_attribute *attr,char *buf)
{
	bl229x_dev_init(g_bl229x);
	return sprintf(buf,"device is re-inited, contrast=0x%x  gain=0x%x,dacn=0x%x\n",g_bl229x->contrast, g_bl229x->chip_params.m_gain,g_bl229x->chip_params.m_dacn);
}
static DEVICE_ATTR(init,0664,bl229x_show_init,NULL);

static ssize_t bl229x_show_reset(struct device *ddri,struct device_attribute *attr,char *buf)
{
	bl229x_dev_reset(g_bl229x);
	return sprintf(buf,"driver type: %d, contrast=0x%x  gain=0x%x\n", g_bl229x->chip_info.chip_driver_type,bl229x_spi_read_reg(REGA_RX_DACP_LOW), bl229x_spi_read_reg(REGA_GC_STAGE));
}
static DEVICE_ATTR(reset,0664,bl229x_show_reset,NULL);

static ssize_t bl229x_show_dacn(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"dacn var=0x%x\n",g_bl229x->chip_params.m_dacn);
}
static ssize_t bl229x_store_dacn(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 i;
	g_bl229x->chip_params.m_dacn = simple_strtoul(buf, NULL, 16);

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	for (i = 0x1D; i <= 0x28; i++) {
		bl229x_spi_write_reg(i, g_bl229x->chip_params.m_dacn);
	}

	return size;
}
static DEVICE_ATTR(dacn,0664,bl229x_show_dacn,bl229x_store_dacn);

static ssize_t bl229x_show_fdthred(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"fdthred var=0x%x\n",fingerprintdParams.finger_fd_thred_high);
}
static ssize_t bl229x_store_fdthred(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	fingerprintdParams.finger_fd_thred_high= simple_strtoul(buf, NULL, 16);

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, fingerprintdParams.finger_fd_thred_high);	// 0x05 / 0x10

	return size;
}
static DEVICE_ATTR(fdthred,0664,bl229x_show_fdthred,bl229x_store_fdthred);

static ssize_t bl229x_show_gain(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"m_gain =0x%x,m_gain_capture=0x%x  gain_reg=0x%x\n",g_bl229x->chip_params.m_gain,g_bl229x->chip_params.m_gain_capture, bl229x_spi_read_reg(REGA_GC_STAGE));
}
static ssize_t bl229x_store_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	g_bl229x->chip_params.m_gain = simple_strtoul(buf, &next, 16);
	g_bl229x->chip_params.m_gain_capture = simple_strtoul(next+1, &next, 16);
	fingerprintdParams.def_gain = g_bl229x->chip_params.m_gain;
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	bl229x_spi_write_reg(REGA_GC_STAGE,g_bl229x->chip_params.m_gain);

	return size;
}
static DEVICE_ATTR(gain,0664,bl229x_show_gain,bl229x_store_gain);

static ssize_t bl229x_show_contrast(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"capture contrast_var=0x%x contrast_reg=0x%x int contrast=0x%x\n",g_bl229x->contrast, bl229x_spi_read_reg(REGA_RX_DACP_LOW),g_bl229x->chip_params.m_contrast);
}
static ssize_t bl229x_store_contrast(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	g_bl229x->chip_params.m_contrast  = simple_strtoul(buf, NULL, 16);
	fingerprintdParams.def_contrast = g_bl229x->chip_params.m_contrast;
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,g_bl229x->chip_params.m_contrast);

	return size;
}
static DEVICE_ATTR(contrast,0664,bl229x_show_contrast,bl229x_store_contrast);

/*----------------------------------------------------------------------------*/
static struct device_attribute *bl229x_attr_list[] = {
	&dev_attr_contrast,
	&dev_attr_gain,
	&dev_attr_dacn,
	&dev_attr_reset,
	&dev_attr_init,
	&dev_attr_mode,
	&dev_attr_register,
	&dev_attr_reg,
	&dev_attr_report_delay,
	&dev_attr_enbacklight,
	&dev_attr_selftest,
	&dev_attr_key_interrupt,    
	&dev_attr_fdthred,
	&dev_attr_image,
	&dev_attr_debug,

};
/*----------------------------------------------------------------------------*/
static void fps_create_attributes(struct device *dev)
{
	int num = (int)(sizeof(bl229x_attr_list)/sizeof(bl229x_attr_list[0]));
	for (; num > 0;)
		device_create_file(dev, bl229x_attr_list[--num]);
}

static int fps_sysfs_init(void)
{
	int ret;
	bl229x_kobj = kobject_create_and_add("bl229x_sysfs",NULL);
	if(bl229x_kobj == NULL) {
		BTL_DEBUG("%s  subsystem_register failed\n",__func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(bl229x_kobj,&dev_attr_contrast.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_gain.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_dacn.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_reset.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_init.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_mode.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_register.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_reg.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_report_delay.attr);    
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_selftest.attr);    
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_enbacklight.attr);    
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_key_interrupt.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_fdthred.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_image.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_debug.attr);

	if(ret) {
		BTL_DEBUG("%s sysfs_create_file failed\n",__func__);
	}

	kobject_uevent(bl229x_kobj, KOBJ_ADD);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int bl229x_create_inputdev(void)
{
	bl229x_inputdev = input_allocate_device();
	if (!bl229x_inputdev) {
		BTL_DEBUG("bl229x_inputdev create faile!\n");
		return -ENOMEM;
	}
	__set_bit(EV_KEY,bl229x_inputdev->evbit);
	__set_bit(KEY_F10,bl229x_inputdev->keybit);		//68
	__set_bit(KEY_F11,bl229x_inputdev->keybit);		//88
	__set_bit(KEY_F12,bl229x_inputdev->keybit);		//88
	__set_bit(KEY_CAMERA,bl229x_inputdev->keybit);	//212
	__set_bit(KEY_POWER,bl229x_inputdev->keybit);	//116
	__set_bit(KEY_PHONE,bl229x_inputdev->keybit);  //call 169
	__set_bit(KEY_BACK,bl229x_inputdev->keybit);  //call 158
	__set_bit(KEY_HOMEPAGE,bl229x_inputdev->keybit);  //call 158
	__set_bit(KEY_MENU,bl229x_inputdev->keybit);  //call 158

	__set_bit(KEY_F1,bl229x_inputdev->keybit);	//69
	__set_bit(KEY_F2,bl229x_inputdev->keybit);	//60
	__set_bit(KEY_F3,bl229x_inputdev->keybit);	//61
	__set_bit(KEY_F4,bl229x_inputdev->keybit);	//62
	__set_bit(KEY_F5,bl229x_inputdev->keybit);	//63
	__set_bit(KEY_F6,bl229x_inputdev->keybit);	//64
	__set_bit(KEY_F7,bl229x_inputdev->keybit);	//65
	__set_bit(KEY_F8,bl229x_inputdev->keybit);	//66
	__set_bit(KEY_F9,bl229x_inputdev->keybit);	//67

	__set_bit(KEY_UP,bl229x_inputdev->keybit);	//103
	__set_bit(KEY_DOWN,bl229x_inputdev->keybit);	//108
	__set_bit(KEY_LEFT,bl229x_inputdev->keybit);	//105
	__set_bit(KEY_RIGHT,bl229x_inputdev->keybit);	//106

	bl229x_inputdev->id.bustype = BUS_HOST;
	bl229x_inputdev->name = "bl229x_inputdev";
	if (input_register_device(bl229x_inputdev)) {
		printk("%s, register inputdev failed\n", __func__);
		input_free_device(bl229x_inputdev);
		return -ENOMEM;
	}

	return 0;
}

/* -------------------------------------------------------------------- */
static int is_need_lock(unsigned int cmd)
{
	int ret = 0;
	switch (cmd) {
		case BL229X_INIT:
		case BL229X_GETIMAGE:
		case BL229X_INITERRUPT_MODE:
		case BL229X_POWERDOWN_MODE1:
		case BL229X_POWERDOWN_MODE2:
		case BL229X_TEST_MODE:
		case BL229X_GET_ID:
		case BL229X_REPORTKEY_NEEDED:
			ret = 1;
			break;
		case BL229X_CONTRAST_ADJUST: 
		case BL229X_CONTRAST_ADJUST2: 
		case BL229X_GAIN_ADJUST:
		case BL229X_INTERRUPT_FLAGS1:
		case BL229X_INTERRUPT_FLAGS2:
		case BL229X_MULTIFUNCTIONAL_KEYCODE:
		case BL229X_INIT_ARGS:
		default:
			ret = 0;
			break;

	}
	BTL_DEBUG("cmd=0x%x,ret=%d\n",cmd,ret);
	return ret;
}

/*----------------------------------------------------------------------------*/
irqreturn_t bl229x_eint_handler(int irq,void *data)
{
	u8 intStatus = 0;
	int ret = 0;
	int curStatus = 0;

	BTL_DEBUG("  ++\n");

	wake_lock_timeout(&fp_suspend_lock, HZ/2);
	wait_event_interruptible_timeout(waiting_spi_prepare,!atomic_read(&suspended),msecs_to_jiffies(100));
	BTL_DEBUG("mode:%d HZ=%d\n",g_bl229x->interrupt_mode,HZ);
	BTL_DEBUG("g_bl229x->fp_detect_method:%d \n", g_bl229x->fp_detect_method);
	if (g_bl229x->fp_detect_method == FP_DET_BY_FD) 
		intStatus = 2;
	else if (g_bl229x->fp_detect_method == FP_DET_BY_NAV) 
		intStatus = 4;

	mutex_lock(&handler_mutex);
	curStatus = getSensorInterruptStatus(g_bl229x);
	if (curStatus != intStatus) {
		if(curStatus == 8)
		{
			g_bl229x->interrupt_mode = INT_MODE_NONE;
			(*bl229x_dev_init)(g_bl229x);
			bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);			 
		}
		else {
			bl229x_reset_for_esd(g_bl229x);
			(*bl229x_dev_init)(g_bl229x);
			if (g_bl229x->interrupt_mode == INT_MODE_KEY)
				bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);			 
			else
				bl229x_dev_capture_init(g_bl229x,0);
		}

		BTL_DEBUG("IC reset\n");
		mutex_unlock(&handler_mutex);
		return IRQ_HANDLED;
	}
#ifdef ENABLE_VKEY_IRQ_CTL
	do_delaywork(0, HZ/2);
#endif	

	if (g_bl229x->interrupt_mode == INT_MODE_KEY) {       
		g_bl229x->interrupt_mode =  INT_MODE_CAPTURE;
#if(MTK_ANDROID_L != 1)
		bl229x_async_Report();
#endif
		BTL_DEBUG("g_bl229x->report_key:%d, g_bl229x->need_report: %d", g_bl229x->report_key,g_bl229x->need_report);
		if (g_bl229x->report_key!=0 && g_bl229x->need_report!=0){

			BTL_DEBUG("is_busy=%x ++++\n",atomic_read(&is_busy));
			if(g_bl229x->need_report == 1){
				touch_index = 0;
				input_report_key(bl229x_inputdev,g_bl229x->report_key ,1);
				input_sync(bl229x_inputdev);
				while(ret == 0)
				{
					if(atomic_read(&is_busy) == 1) {
						break;
					} else {
						if (getSensorInterruptStatus(g_bl229x) != intStatus) {
							BTL_DEBUG("eint read frame err");
							(*bl229x_dev_init)(g_bl229x);
						}
						else{
							spi_read_frame(g_bl229x);
							g_bl229x->is_frame_ready = 1;

							ret = cal_touch_area_point(g_bl229x);
							BTL_DEBUG("isfingerup ret = %d touch_index=%d\n", ret,touch_index);									
						}
						if(ret != 0)
						{
							break;
						}
						g_bl229x->contrast = g_bl229x->chip_params.m_contrast;
						bl229x_dev_capture_init(g_bl229x,g_bl229x->contrast);
						msleep(g_bl229x->report_delay);
					}
				}
				if(gDirection == 0)
				{
					input_report_key(bl229x_inputdev,g_bl229x->report_key ,0);

					input_sync(bl229x_inputdev);
				} else if(gDirection == 1)
				{
					input_report_key(bl229x_inputdev,KEY_RIGHT ,1);

					input_sync(bl229x_inputdev);
					input_report_key(bl229x_inputdev,KEY_RIGHT ,0);
					input_sync(bl229x_inputdev);
				} else if(gDirection == 2)

				{
					input_report_key(bl229x_inputdev,KEY_LEFT ,1);
					input_sync(bl229x_inputdev);

					input_report_key(bl229x_inputdev,KEY_LEFT ,0);
					input_sync(bl229x_inputdev);

				} else if(gDirection == 3)
				{
					input_report_key(bl229x_inputdev,KEY_DOWN ,1);
					input_sync(bl229x_inputdev);

					input_report_key(bl229x_inputdev,KEY_DOWN ,0);
					input_sync(bl229x_inputdev);
				} else if(gDirection == 4)
				{

					input_report_key(bl229x_inputdev,KEY_UP ,1);
					input_sync(bl229x_inputdev);
					input_report_key(bl229x_inputdev,KEY_UP ,0);
					input_sync(bl229x_inputdev);

				}
				BTL_DEBUG("report power key %d\n",g_bl229x->report_key );
				//kobject_uevent_env(&g_bl229x->spi->dev.kobj, KOBJ_CHANGE,"UEVENT=bl229x_irq");
				if(atomic_read(&is_busy) == 0) {
					ret = bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);
				}
			}else if(g_bl229x->need_report == 2){
						BTL_DEBUG("is_busy=%x ++++\n",atomic_read(&is_busy));
				input_report_key(bl229x_inputdev,KEY_F10, 1);
        		input_sync(bl229x_inputdev);
				input_report_key(bl229x_inputdev,KEY_F10, 0);

        		input_sync(bl229x_inputdev);
			}
			BTL_DEBUG("report power key %d\n",g_bl229x->report_key );
		}
		//spi_read_frame(g_bl229x);
		//g_bl229x->is_frame_ready = 1;
		//wake_up_interruptible(&frame_waiter);
	} else {
		spi_read_frame(g_bl229x);
		BTL_DEBUG("is_frame_ready:%d\n", g_bl229x->is_frame_ready);
		if (g_bl229x->is_frame_ready == 0 )
			g_bl229x->is_frame_ready = 1;
		wake_up_interruptible(&frame_waiter);
	}
	mutex_unlock(&handler_mutex);
	BTL_DEBUG("  --\n");
	return IRQ_HANDLED;
}

#ifdef BTL_USE_WORKQUEUE
//\D6жϴ\A6\C0\ED\BA\AF\CA\FD
static void bl229x_eint_handler_l(void)
{
	disable_irq_nosync(g_bl229x->irq_num);
	BTL_DEBUG("\n");
	queue_work(g_bl229x->fingerprint_workqueue, &(g_bl229x->fingerprint_work));
}

void work_func(struct work_struct *p_work)
{
	BTL_DEBUG("  ++\n");
	bl229x_eint_handler(g_bl229x->irq_num,g_bl229x);
	enable_irq(g_bl229x->irq_num);
	BTL_DEBUG("  --\n");
}
#endif

/* -------------------------------------------------------------------- */
static long bl229x_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
	struct bl229x_data *bl229x = filp->private_data;
	int error=0;
	u32 user_regval = 0;
	u32 chipid;
	u8  dataBuf[64];
	void __user *argp = (void __user *)arg;
	u32 bl229x_enbacklight = 0;

	BTL_DEBUG("%s\n",__func__);


#if 0
	if (_IOC_DIR(cmd) & _IOC_READ)
		error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		error = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (error) {
		BTL_DEBUG("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
#endif


	if (bl229x->reset){
		BTL_DEBUG("chip is in reseting\n");
		//return -EBUSY;
	}

	if(is_need_lock(cmd))
	{
#ifdef ENABLE_VKEY_IRQ_CTL
		do_delaywork(0, HZ/10);
#endif
		if(BL229X_INITERRUPT_MODE != cmd)
			atomic_set(&is_busy, 1);
		mutex_lock(&handler_mutex);
	}
	switch (cmd) {
		case BL229X_INIT:
			BTL_DEBUG("INIT_BL229X \n");
			error= (*bl229x_dev_init)(bl229x);
			break;
		case BL229X_GETIMAGE:
			BTL_DEBUG("BL229X_GETIMAGE \n");
			bl229x_spi_write_reg(REGA_GC_STAGE, bl229x->chip_params.m_gain_capture);
			error = bl229x_dev_capture_init(bl229x,arg);
			break;
		case BL229X_CONTRAST_ADJUST: 
		case BL229X_CONTRAST_ADJUST2: 	
			BTL_DEBUG("BL229X_CONTRAST_ADJUST1 \n");
			error= bl229x_contrast_init(bl229x,arg);
			break;
		case BL229X_INITERRUPT_MODE:
		case BL229X_POWERDOWN_MODE1:
		case BL229X_POWERDOWN_MODE2:
			BTL_DEBUG("BL229X_INITERRUPT_MODE \n");
			error = bl229x_dev_interrupt_init(bl229x,bl229x->fp_detect_method);
			atomic_set(&is_busy, 0);
			break;
		case BL229X_INTERRUPT_FLAGS1:
		case BL229X_INTERRUPT_FLAGS2:
			BTL_DEBUG("BL229X_INTERRUPT_FLAGS1 \n");
			user_regval = gpio_get_value(bl229x->irq_gpio);
			if (copy_to_user((void __user*)arg, &user_regval, sizeof(user_regval)) != 0) {
				error = -EFAULT;
			}
			break;
		case BL229X_MULTIFUNCTIONAL_KEYCODE:
			g_bl229x->report_key  = (int)arg;
			break;		
		case BL229X_TEST_MODE:
			if (copy_from_user(dataBuf,(void __user*)arg,32) != 0 ){
				BTL_DEBUG("ERROR: BL229X_TEST_MODE\n");
				error = -EFAULT;
				break;
			}
			bl229x_set_testmode(bl229x,dataBuf, 32);
			break;
		case BL229X_GAIN_ADJUST:
			bl229x_gain_init(bl229x,arg);
			break;
		case BL229X_GET_ID:
			chipid =(*bl229x_read_chipid)(bl229x);
			if (copy_to_user((void __user*)arg,&chipid,sizeof(u32)*1) != 0 ){
				error = -EFAULT;
			}
			break;
		case BL229X_GET_CHIP_INFO:
			{
				u32 drver_type = bl229x->chip_info.chip_driver_type;
				BTL_DEBUG("BL229X_GET_CHIP_INFO driver_type=%d",drver_type);
				//bl229x_read_chipid(bl229x);
				if (copy_to_user((void __user*)argp,&drver_type,sizeof(u32)*1) != 0 ) {
					error = -EFAULT;
				}
			}
			break;
		case BL229X_INIT_ARGS:
			if (copy_to_user((void __user*)arg,&fingerprintdParams,sizeof(fingerprintdParams)) != 0 ){
				error = -EFAULT;
			}
			break;
		case BL229X_ENBACKLIGHT:
			BTL_DEBUG("BL229X_ENBACKLIGHT arg:%d\n", (int)arg);
			g_bl229x_enbacklights = (int)arg;
			//wake_up_interruptible(&waiter);
			break;
		case BL229X_ISBACKLIGHT:
			BTL_DEBUG("BL229X_ISBACKLIGHT\n");
			bl229x_enbacklight = g_bl229x_enbacklights;
			if (copy_to_user((void __user*)arg,&bl229x_enbacklight,sizeof(u32)*1) != 0 ){
				error = -EFAULT;
			}
			break;
		case BL229X_REPORTKEY_NEEDED:
			//if(copy_from_user(&g_bl229x->need_report,(void __user*)arg, sizeof(u32)*1)!=0)
			//  error = -EFAULT;
			g_bl229x->need_report = (u32)arg;
			BTL_DEBUG("g_bl229x->need_report = %d", g_bl229x->need_report);
			break;
		default:
			error = -ENOTTY;
			break;

	}
	if(is_need_lock(cmd))
		mutex_unlock(&handler_mutex);
	return error;

}

#ifdef CONFIG_COMPAT
static long bl229x_compat_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
	return bl229x_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define bl229x_compat_ioctl NULL
#endif /* CONFIG_COMPAT */
/*----------------------------------------------------------------------------*/
static int is_irq_enabled = 0;
static int bl229x_open(struct inode *inode, struct file *file)
{
	struct bl229x_data *bl229x = g_bl229x;
	if(is_irq_enabled == 0)
	{
		is_irq_enabled = 1;
		enable_irq(bl229x->irq_num);
	}
	BTL_DEBUG("  ++\n");
	if (g_bl229x->reset) {
		BTL_DEBUG("chip is in reseting\n");
		//return -EAGAIN;
	}

	file->private_data = bl229x;

	BTL_DEBUG("  --\n");
	return 0;
}


/* -------------------------------------------------------------------- */
//\A7\D5\B2\D9\D7\F7\A3\AC\810\F68\CE\810\9D0\AB\B8\A7\D3\BB\D0\E8\810\DB8\B6\D4\C6\E4\BD\F8\D0\D0\A7\D5\B2\D9\D7\F7 \A1\A3\B9\CA\810\F61\BD\810\E35\B5\BB\D8 \B2\D9\D7\F7
static ssize_t bl229x_write(struct file *file, const char *buff,size_t count, loff_t *ppos)
{
	return -ENOMEM;
}

/* -------------------------------------------------------------------- */
// \B6\C1\B2\D9\D7\F7
static ssize_t bl229x_read(struct file *file, char  *buff,size_t count, loff_t *ppos)
{
	int ret=0;
	int timeout;
	//struct bl229x_data *bl229x = file->private_data;
	ssize_t status = 0;

	BTL_DEBUG("  ++\n");


	BTL_DEBUG("mode:%d \n",g_bl229x->interrupt_mode);
	if (g_bl229x->reset)
	{
		BTL_DEBUG("chip is in reseting\n");
		//return -EAGAIN;
	}
	//wait_event_interruptible(frame_waiter, is_frame_ready !=0);
	timeout = wait_event_interruptible_timeout(frame_waiter, g_bl229x->is_frame_ready !=0, 50);

	BTL_DEBUG("timeout:%d, is_frame_ready : %d\n\n",timeout,g_bl229x->is_frame_ready);

	if (timeout == 0 || g_bl229x->is_frame_ready == 0) {
		BTL_DEBUG("read timeout\n\n");
		g_bl229x->is_frame_ready = 0;
		return -EFAULT;
	}

	g_bl229x->is_frame_ready = 0;
	ret = copy_to_user(buff, g_bl229x->image_buf , count); //skip
	if (ret) {
		status = -EFAULT;
	}
	//memset(g_bl229x->image_buf ,0xFF,count);
	BTL_DEBUG("status: %d \n", (int)status);
	BTL_DEBUG("  --\n");
	return status;
}


/* -------------------------------------------------------------------- */
static int bl229x_release(struct inode *inode, struct file *file)
{
	int status = 0 ;

	BTL_DEBUG("  ++\n");
#if(MTK_ANDROID_L != 1)
	bl229x_async_fasync(-1, file, 0);
#endif
	BTL_DEBUG("  --\n");
	return status;
}
/* -------------------------------------------------------------------- */

static int bl229x_suspend (struct device *dev)
{
	//struct bl229x_data *bl229x = dev_get_drvdata(dev);
	BTL_DEBUG("  ++\n");
	atomic_set(&suspended, 1);
	BTL_DEBUG("\n");
	return 0;
}

/* -------------------------------------------------------------------- */
static int bl229x_resume (struct device *dev)
{
	//struct bl229x_data *bl229x = dev_get_drvdata(dev);
	//dev_err (&bl229x->spi->dev,"[bl229x]%s\n", __func__);
	BTL_DEBUG("  ++\n");
	atomic_set(&suspended, 0);
	wake_up_interruptible(&waiting_spi_prepare);
	BTL_DEBUG("\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static const struct file_operations bl229x_fops = {
	.owner = THIS_MODULE,
	.open  = bl229x_open,
	.write = bl229x_write,
	.read  = bl229x_read,
	.release = bl229x_release,
	.unlocked_ioctl = bl229x_ioctl,
	.compat_ioctl = bl229x_compat_ioctl,
#if(MTK_ANDROID_L != 1)
	.fasync = bl229x_async_fasync,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice bl229x_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SPI_DRV_NAME,
	.fops = &bl229x_fops,
};
int is_connected(struct bl229x_data *bl229x)
{
	int cur_contrast = 100;
	int m_is_conneted = 0;
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,cur_contrast);
	BTL_DEBUG("write %d",cur_contrast);
	m_is_conneted = bl229x_spi_read_reg(REGA_RX_DACP_LOW);
	BTL_DEBUG("readback %d",m_is_conneted);
	if(cur_contrast == m_is_conneted)
	{
		BTL_DEBUG("ok %d",bl229x_spi_read_reg(REGA_RX_DACP_LOW));
		return 1;
	}
	BTL_DEBUG("failed %d",bl229x_spi_read_reg(REGA_RX_DACP_LOW));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int  bl229x_probe(struct spi_device *spi)
{
	struct bl229x_data *bl229x = NULL;
	int err = 0;

	BTL_DEBUG("  ++\n\n");
	bl229x = kzalloc(sizeof(struct bl229x_data),GFP_KERNEL);
	if (!bl229x) {
		return -ENOMEM;
	}
	g_bl229x = bl229x;

	spi_set_drvdata(spi,bl229x);
#if USE_SPI1_4GB_TEST
	spi_setup_xfer(NULL);
#endif
	bl229x->interrupt_mode = INT_MODE_KEY;

	bl229x->report_key = KEY_F11;

	bl229x->report_delay = REPORT_DELAY_TIME;

	bl229x->is_frame_ready = 0;

	bl229x->fp_detect_method = FP_DETECT_METHOD;

	bl229x->report_timeout = 0;
	bl229x->reset = 0;
	bl229x->need_report = 1;

	bl229x->spi = spi;
	bl229x->spi->bits_per_word = 8;
	bl229x->spi->mode = SPI_MODE_0;
#if(ARCH_MTK_BTL == 1)
	bl229x->spi->controller_data = (void*)&spi_conf;
#endif
	spi_setup(bl229x->spi);

	bl229x_gpio_select_and_init(bl229x);
	hw_reset(bl229x);
	//spi dma or fifo mode
	mtspi_set_dma_en(0);

   	if(bl229x_read_chipid1(bl229x) != 0x5383 && bl229x_read_chipid1(bl229x) != 0x5283)
   	{
   		BTL_DEBUG("chip is not 3390 & 3182");
		if(bl229x_read_chipid2(bl229x) != 0x5183)
		{
			BTL_DEBUG("chip is not 3290");
			goto exit_misc_device_register_failed;
		}
   	}

	bl229x_image_size = BL229X_WIDTH * BL229X_HEIGHT;
	DMA_TRANSFER_SIZE =(((bl229x_image_size / 1024) + 1)*1024);

    READIMAGE_BUF_SIZE = DMA_TRANSFER_SIZE;
	fingerprintdParams.def_contrast = SENSOR_DEFAULT_CONTRAST_V3,
	fingerprintdParams.def_ck_fingerup_contrast = SENSOR_DEFAULT_CONTRAST_V3;
	fingerprintdParams.def_gain = SENSOR_DEFAULT_GAIN_V3;

	bl229x->image_buf = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));
	bl229x->imagetxcmd = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));;
	bl229x->imagerxpix = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));;

	memset(bl229x->image_buf,0x00,get_order(READIMAGE_BUF_SIZE));
	memset(bl229x->imagetxcmd,0x00,get_order(READIMAGE_BUF_SIZE));
	memset(bl229x->imagerxpix,0x00,get_order(READIMAGE_BUF_SIZE));
	if (!bl229x->image_buf) {
		BTL_DEBUG("\n\n");
		return -ENOMEM;
	}

	err = misc_register(&bl229x_misc_device);
	if(err) {
		BTL_DEBUG("bl229x_misc_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	bl229x_create_inputdev();

	wake_lock_init(&fp_suspend_lock, WAKE_LOCK_SUSPEND, "fp_wakelock");	
	BTL_DEBUG("step-2:%d\n",spi->irq);
	atomic_set(&is_busy, 0);
	atomic_set(&suspended, 0);

	bl229x->chip_params.m_contrast =SENSOR_DEFAULT_CONTRAST_V3 ;
	bl229x->chip_params.m_gain = fingerprintdParams.def_gain;
	bl229x->chip_params.m_dacn= SENSOR_DEFAULT_DACN_V3;
	bl229x->chip_params.m_gain_capture = fingerprintdParams.def_gain;
	fingerprintdParams.finger_fd_thred_high = FINGER_TD_THRED_HIGH_VALUE;
	//  IRQF_TRIGGER_HIGH | IRQF_ONESHOT
#ifdef BTL_USE_WORKQUEUE
	mt_eint_registration(bl229x->irq_num, CUST_EINTF_TRIGGER_RISING,bl229x_eint_handler_l,0);
	INIT_WORK(&(bl229x->fingerprint_work), work_func);; 
	bl229x->fingerprint_workqueue = create_singlethread_workqueue("btl_fingerpirnt_thread");
#else
	//bl229x->irq_num = spi->irq > 0? spi->irq:gpio_to_irq(bl229x->irq_gpio);
	err = request_threaded_irq(spi->irq, NULL, bl229x_eint_handler, IRQ_TYPE_EDGE_RISING | IRQF_ONESHOT, SPI_DRV_NAME, bl229x);
#endif

	enable_irq_wake(bl229x->irq_num);
	disable_irq_nosync(bl229x->irq_num);

	//debug \B5\F7\CA\810\EA5\811\921\E3
	fps_sysfs_init();
	fps_create_attributes(&spi->dev);

	(*bl229x_dev_init)(bl229x);
	bl229x_dev_interrupt_init(bl229x,bl229x->fp_detect_method);
	bl229x->fb_notify.notifier_call = fb_notifier_callback;
	err = fb_register_client(&bl229x->fb_notify);
	if (err)
		dev_err(&bl229x->spi->dev, "Unable to register fb_notifier: %d\n",
				err);
	BTL_DEBUG("  --\n");
	return 0;

exit_misc_device_register_failed:

	free_pages((unsigned long)bl229x->imagerxpix, get_order(READIMAGE_BUF_SIZE));
	free_pages((unsigned long)bl229x->imagetxcmd, get_order(READIMAGE_BUF_SIZE));
	free_pages((unsigned long)bl229x->image_buf, get_order(READIMAGE_BUF_SIZE));
	kfree(bl229x);
	printk("%s,probe-error!!!!\n\n",__func__);
	return -1;
}

/*----------------------------------------------------------------------------*/

static struct of_device_id bl229x_match_table[] = {
	{.compatible = "mediatek,BL229X",},
	{},
};

/*----------------------------------------------------------------------------*/
static const struct dev_pm_ops bl229x_pm = {
	.suspend = bl229x_suspend,
	.resume = bl229x_resume
};


static struct spi_driver bl229x_driver = {
	.driver = {
		.name	= SPI_DRV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,

#ifdef CONFIG_OF
		.of_match_table = bl229x_match_table,
#endif
		.pm = &bl229x_pm,
	},
	.probe	= bl229x_probe,
};

/*----------------------------------------------------------------------------*/
static int  mt_spi_init(void)
{
	int ret=0;
	BTL_DEBUG("%s",__func__);
#if(MTK_ANDROID_L == 1)
	ret=spi_register_board_info(spi_board_bl229x,ARRAY_SIZE(spi_board_bl229x));
#endif
	ret=spi_register_driver(&bl229x_driver);

	return ret;
}


static void  mt_spi_exit(void)
{
	spi_unregister_driver(&bl229x_driver);
}

module_init(mt_spi_init);
module_exit(mt_spi_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("BetterLife BetterLife@blestech.com");
MODULE_DESCRIPTION("BL2x9x fingerprint sensor driver.");

