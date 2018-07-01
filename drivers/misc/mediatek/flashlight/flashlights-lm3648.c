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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/gpio.h>
//#include <platform/mt_gpio.h>
//#include <asm/arch/mt_gpio.h>
#include "flashlight-core.h"
#include "flashlight-dt.h"
#include "mt-plat/mtk_gpio.h"
#include "mt-plat/mtk_gpio_core.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef LM3648_DTNAME
#define LM3648_DTNAME "mediatek,flashlights_lm3648"
#endif
#ifndef LM3648_DTNAME_I2C
#define LM3648_DTNAME_I2C "mediatek,strobe_sub"
#endif

#define LM3648_NAME "flashlights-lm3648"

/* define registers */
#define LM3648_REG_ENABLE 			0x01
#define LM3648_REG_IVFM				0x02
#define LM3648_REG_FLASH_LEVEL_LED1 0x03
//#define LM3648_REG_FLASH_LEVEL_LED2 0x04
#define LM3648_REG_TORCH_LEVEL_LED1 0x05
//#define LM3648_REG_TORCH_LEVEL_LED2 0x06
#define LM3648_REG_BOOST_CONFIG		0x07
#define LM3648_REG_TIMING_CONFIG	0x08
#define LM3648_REG_TEMP				0x09
#define LM3648_REG_FLAG1			0x0A
#define LM3648_REG_FLAG2			0x0B
#define LM3648_REG_DEVICE_ID		0x0C


#define LM3648_LEVEL_NUM 26
#define LM3648_LEVEL_TORCH 7


#define LM3648_PINCTRL_PIN_HWEN (90 | 0x80000000)
#define LM3648_PINCTRL_PINSTATE_LOW 0
#define LM3648_PINCTRL_PINSTATE_HIGH 1
#define LM3648_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define LM3648_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *lm3648_pinctrl;
static struct pinctrl_state *lm3648_hwen_high;
static struct pinctrl_state *lm3648_hwen_low;

//#define GPIO_CAMERA_HWEN_PIN    	(90 | 0x80000000)

/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(lm3648_mutex);
static struct work_struct lm3648_work;
/* define usage count */
static int use_count;
static int m_duty;
unsigned int  rgt_sub_torch_level=0;
/* define i2c */
static struct i2c_client *LM3648_i2c_client;

/* platform data */
struct lm3648_platform_data {
	u8 torch_pin_enable;         /* 1: TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;      /* 1: TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable; /* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;       /* 1: STROBE Input disabled */
	u8 vout_mode_enable;         /* 1: Voltage Out Mode enable */
};

/* lm3648 chip data */
struct lm3648_chip_data {
	struct i2c_client *client;
	struct lm3648_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};


/******************************************************************************
 * lm3648 operations
 *****************************************************************************/

/* i2c wrapper function */
static int lm3648_flash_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct lm3648_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		printk("failed writing at 0x%02x\n", reg);

	return ret;
}


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
#if 1
static int lm3648_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	lm3648_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lm3648_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(lm3648_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	lm3648_hwen_high = pinctrl_lookup_state(lm3648_pinctrl, LM3648_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(lm3648_hwen_high)) {
		pr_err("Failed to init (%s)\n", LM3648_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(lm3648_hwen_high);
	}
	lm3648_hwen_low = pinctrl_lookup_state(lm3648_pinctrl, LM3648_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(lm3648_hwen_low)) {
		pr_err("Failed to init (%s)\n", LM3648_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(lm3648_hwen_low);
	}

	return ret;
}
#endif

static int lm3648_pinctrl_set(int pin, int state)
{
	int ret = 0;


	if (IS_ERR(lm3648_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case LM3648_PINCTRL_PIN_HWEN:
		if (state == LM3648_PINCTRL_PINSTATE_LOW && !IS_ERR(lm3648_hwen_low))
			pinctrl_select_state(lm3648_pinctrl, lm3648_hwen_low);
		else if (state == LM3648_PINCTRL_PINSTATE_HIGH && !IS_ERR(lm3648_hwen_high))
			pinctrl_select_state(lm3648_pinctrl, lm3648_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * lm3648 operations
 *****************************************************************************/
 //torch(mA) = (Code * 2.8) + 1.954(mA)     0x55:240mA
static const unsigned char lm3648_torch_level[LM3648_LEVEL_NUM] = {
	0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//flash (mA) = (Code * 23.45) + 21.8 (mA) //397,467,514,585,631,702,772,842(mA)
static const unsigned char lm3648_flash_level[LM3648_LEVEL_NUM] = {
	0x10, 0x13, 0x15, 0x18, 0x1A, 0x1D, 0x20, 0x23, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};




/* flashlight enable function */
static int lm3648_enable(void)
{
	printk("ludesuo__lm3648_enable m_duty = %d\n",m_duty);
	
	lm3648_pinctrl_set(LM3648_PINCTRL_PIN_HWEN, LM3648_PINCTRL_PINSTATE_HIGH);
	
	 if(m_duty < 1)
	{
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_TORCH_LEVEL_LED1,((lm3648_torch_level[m_duty]&0x3F)|0x80));//clean bit 7 to 0 ,then led2 not be override		
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_ENABLE,0x0B );//torch mode ,enable led1 led2
	}
	else
	{
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_FLASH_LEVEL_LED1,((lm3648_flash_level[m_duty]&0x3F)|0x80));//clean bit 7 to 0 ,then led2 not be override
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_ENABLE,0x0F );//flash mode ,enable led1 led2
	}

	return 0;
}

/* flashlight disable function */
static int lm3648_disable(void)
{
	printk("ludesuo__lm3648_disable\n");
	rgt_sub_torch_level = 0;
	lm3648_flash_write(LM3648_i2c_client,LM3648_REG_ENABLE,0x00);
	lm3648_pinctrl_set(LM3648_PINCTRL_PIN_HWEN, LM3648_PINCTRL_PINSTATE_LOW);
	return 0;
}

/* set flashlight level */
static int lm3648_set_level(int level)
{
	if (level < 0)
	{
		level = 0;
	}
	m_duty = level;
	printk("ludesuo__lm3648_set_level level = %d\n",level);
	return 0;
}

/* flashlight init */
static int lm3648_init(void)
{
	printk("ludesuo__lm3648_init\n");
	lm3648_pinctrl_set(LM3648_PINCTRL_PIN_HWEN, LM3648_PINCTRL_PINSTATE_HIGH);
	lm3648_flash_write(LM3648_i2c_client,LM3648_REG_ENABLE,0x03);
	lm3648_flash_write(LM3648_i2c_client,LM3648_REG_BOOST_CONFIG,0x09);
	lm3648_flash_write(LM3648_i2c_client,LM3648_REG_TIMING_CONFIG,0x1f);	

	return 0;
}

/* flashlight uninit */
static int lm3648_uninit(void)
{
	lm3648_disable();

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer lm3648_timer;
static unsigned int lm3648_timeout_ms;

static void lm3648_work_disable(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	lm3648_disable();
}

static enum hrtimer_restart lm3648_timer_func(struct hrtimer *timer)
{
	schedule_work(&lm3648_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int lm3648_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm3648_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm3648_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (lm3648_timeout_ms) {
				ktime = ktime_set(lm3648_timeout_ms / 1000,
						(lm3648_timeout_ms % 1000) * 1000000);
				hrtimer_start(&lm3648_timer, ktime, HRTIMER_MODE_REL);
			}
			lm3648_enable();
		} else {
			lm3648_disable();
			hrtimer_cancel(&lm3648_timer);
		}
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int lm3648_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int lm3648_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&lm3648_mutex);
	use_count--;
	if (!use_count)
		lm3648_uninit();
	lm3648_pinctrl_set(LM3648_PINCTRL_PIN_HWEN, LM3648_PINCTRL_PINSTATE_LOW);
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&lm3648_mutex);

	printk("Release: %d\n", use_count);

	return 0;
}

static int lm3648_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&lm3648_mutex);
	if (!use_count)
		lm3648_init();
	use_count++;
	mutex_unlock(&lm3648_mutex);

	printk("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t lm3648_strobe_store(struct flashlight_arg arg)
{
	lm3648_set_driver(0);
	lm3648_set_level(arg.level);
	lm3648_enable();
	msleep(arg.dur);
	lm3648_disable();
	lm3648_release();

	return 0;
}

static struct flashlight_operations lm3648_ops = {
	lm3648_open,
	lm3648_release,
	lm3648_ioctl,
	lm3648_strobe_store,
	lm3648_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int lm3648_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * lm3648_init();
	 */

	return 0;
}


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int lm3648_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3648_chip_data *chip;
	struct lm3648_platform_data *pdata = client->dev.platform_data;
	int err;

	pr_debug("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct lm3648_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_debug("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct lm3648_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	LM3648_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&lm3648_work, lm3648_work_disable);

	/* init timer */
	hrtimer_init(&lm3648_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm3648_timer.function = lm3648_timer_func;
	lm3648_timeout_ms = 100;

	/* init chip hw */
	lm3648_chip_init();

	/* register flashlight operations */
	if (flashlight_dev_register(LM3648_NAME, &lm3648_ops)) {
		pr_err("Failed to register flashlight device.\n");
		err = -EFAULT;
		goto err_free;
	}

	/* clear usage count */
	use_count = 0;
	printk("lm3648_probe done.\n");

	return 0;
err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int lm3648_i2c_remove(struct i2c_client *client)
{
	struct lm3648_chip_data *chip = i2c_get_clientdata(client);

	pr_debug("Remove start.\n");

	/* flush work queue */
	flush_work(&lm3648_work);

	/* unregister flashlight operations */
	flashlight_dev_unregister(LM3648_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id lm3648_i2c_id[] = {
	{LM3648_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id lm3648_i2c_of_match[] = {
	{.compatible = LM3648_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver lm3648_i2c_driver = {
	.driver = {
		   .name = LM3648_NAME,
#ifdef CONFIG_OF
		   .of_match_table = lm3648_i2c_of_match,
#endif
		   },
	.probe = lm3648_i2c_probe,
	.remove = lm3648_i2c_remove,
	.id_table = lm3648_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int lm3648_probe(struct platform_device *dev)
{
	pr_debug("Probe start.\n");
    lm3648_pinctrl_init(dev);
	if (i2c_add_driver(&lm3648_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	pr_debug("Probe done.\n");

	return 0;
}

static int lm3648_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&lm3648_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id lm3648_of_match[] = {
	{.compatible = LM3648_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, lm3648_of_match);
#else
static struct platform_device lm3648_platform_device[] = {
	{
		.name = LM3648_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, lm3648_platform_device);
#endif

static struct platform_driver lm3648_platform_driver = {
	.probe = lm3648_probe,
	.remove = lm3648_remove,
	.driver = {
		.name = LM3648_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lm3648_of_match,
#endif
	},
};

static int __init flashlight_lm3648_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&lm3648_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&lm3648_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_lm3648_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&lm3648_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_lm3648_init);
module_exit(flashlight_lm3648_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight LM3648 Driver");

/* add by qi.song 20170313 start */
static struct class *sub_torch_class = NULL;
static struct device *sub_torch_dev = NULL;
static struct device_attribute dev_attr_sub_torch_level;
//static int rgt_sub_torch_level = 0;

static ssize_t sub_torch_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    printk("get sub torch value is:%d \n", rgt_sub_torch_level);
	return sprintf(buf, "%u\n", rgt_sub_torch_level);
}

static ssize_t sub_torch_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    //mode :range[0~9]
    //0: close
    //1~2:torch mode
    //9: flash mode
	unsigned int  mode = 0;
	//const int torch1[2] = {25,75};  //A=(code+1)*375/128(mA)  //43   
	//const int torch2[2] = {12,33};  //A=(code+1)*375/64(mA)   //22

	sscanf(buf, "%d", &mode);
	printk("ludesuo sub_torch_level_store mode=%d\n", mode);
	lm3648_init();
	rgt_sub_torch_level = mode;

	if((mode>0)&&(mode<=2))
	{
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_TORCH_LEVEL_LED1,((lm3648_torch_level[0]&0x3F)|0x80));		
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_ENABLE,0x0B );//torch mode ,enable led1 led2
		printk("ludesuo sub_torch mode=%d\n", mode);		
	}
	else if(mode == 9)
	{
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_FLASH_LEVEL_LED1,((lm3648_flash_level[1]&0x3F)|0x80));
		lm3648_flash_write(LM3648_i2c_client, LM3648_REG_ENABLE,0x0F );
		printk("ludeuso sub_flash mode=%d\n", mode);
	}
	else
	{
		lm3648_disable();
	    printk("ludesuo sub_torch_level_store disable\n");		
	}


	return size;
}

static DEVICE_ATTR_RW(sub_torch_level);

static int __init rgt_sub_torch_level_init(void)
{
	printk("start\n");
	sub_torch_class = class_create(THIS_MODULE, "sub_torch");
	if (IS_ERR(sub_torch_class))
	{
		printk("Unable to create class, err = %d\n", (int)PTR_ERR(sub_torch_class));
		return 0;
	}
	sub_torch_dev = device_create(sub_torch_class, NULL, 0, 0,"sub_torch");
	if(NULL == sub_torch_dev)
	{
		printk("device_create fail\n");
		return 0;
	}

	device_create_file(sub_torch_dev, &dev_attr_sub_torch_level);

	printk("Done\n");
	return 0;
}

static void __exit rgt_sub_torch_level_exit(void)
{
	device_remove_file(sub_torch_dev, &dev_attr_sub_torch_level);
	device_unregister(sub_torch_dev);
	if(sub_torch_class != NULL)
	{
		class_destroy(sub_torch_class);
	}
}


module_init(rgt_sub_torch_level_init);
module_exit(rgt_sub_torch_level_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<qi.song@reallytek.com>");
MODULE_DESCRIPTION("midtest sub flashlight");
/* add by qi.song 20170313 end */