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


#include "flashlight-core.h"
#include "flashlight-dt.h"
#include "mt-plat/mtk_gpio.h"
#include "mt-plat/mtk_gpio_core.h"
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef AW3644_DTNAME_I2C
#define AW3644_DTNAME_I2C "mediatek,strobe_main"
#endif
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef AW3644_DTNAME
#define AW3644_DTNAME "mediatek,flashlights_aw3644"
#endif
/* TODO: define driver name */
#define AW3644_NAME "flashlights-aw3644"

/* define registers */
#define AW3644_REG_ENABLE 			0x01
#define AW3644_REG_IVFM				0x02
#define AW3644_REG_FLASH_LEVEL_LED1 0x03
#define AW3644_REG_FLASH_LEVEL_LED2 0x04
#define AW3644_REG_TORCH_LEVEL_LED1 0x05
#define AW3644_REG_TORCH_LEVEL_LED2 0x06
#define AW3644_REG_BOOST_CONFIG		0x07
#define AW3644_REG_TIMING_CONFIG	0x08
#define AW3644_REG_TEMP				0x09
#define AW3644_REG_FLAG1			0x0A
#define AW3644_REG_FLAG2			0x0B
#define AW3644_REG_DEVICE_ID		0x0C


#define AW3644_LEVEL_NUM 26
#define AW3644_LEVEL_TORCH 7


#define AW3644_PINCTRL_PIN_HWEN 0
#define AW3644_PINCTRL_PINSTATE_LOW 0
#define AW3644_PINCTRL_PINSTATE_HIGH 1
#define AW3644_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define AW3644_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *aw3644_pinctrl;
static struct pinctrl_state *aw3644_hwen_high;
static struct pinctrl_state *aw3644_hwen_low;

#define AW3644_CHANNEL_CH0 0
#define AW3644_CHANNEL_CH1 1 //wwsub

//#define GPIO_CAMERA_HWEN_PIN    	(35 | 0x80000000)

/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(aw3644_mutex);
static struct work_struct aw3644_work;


/* define usage count */
static int use_count;
//static int m_duty;
unsigned int  rgt_torch_level2=0;
unsigned int  rgt_sub_torch_level=0;
static int main_duty ,sub_duty;//wwsub
/* define i2c */
static struct i2c_client *AW3644_i2c_client;

/* platform data */
struct aw3644_platform_data {
	u8 torch_pin_enable;         /* 1: TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;      /* 1: TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable; /* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;       /* 1: STROBE Input disabled */
	u8 vout_mode_enable;         /* 1: Voltage Out Mode enable */
};

/* aw3644 chip data */
struct aw3644_chip_data {
	struct i2c_client *client;
	struct aw3644_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};


/******************************************************************************
 * aw3644 operations
 *****************************************************************************/

/* i2c wrapper function */
static int aw3644_flash_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw3644_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int aw3644_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	printk("RLK_camera_flash_aw3644 aw3644_pinctrl_init in\n");
	//return 1;
	/* get pinctrl */
	aw3644_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3644_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3644_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	aw3644_hwen_high = pinctrl_lookup_state(aw3644_pinctrl, AW3644_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(aw3644_hwen_high)) {
		pr_err("Failed to init (%s)\n", AW3644_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(aw3644_hwen_high);
	}
	aw3644_hwen_low = pinctrl_lookup_state(aw3644_pinctrl, AW3644_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(aw3644_hwen_low)) {
		pr_err("Failed to init (%s)\n", AW3644_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(aw3644_hwen_low);
	}
      printk("RLK_camera_flash_aw3644 aw3644_pinctrl_init out\n");
	return ret;
}

static int aw3644_pinctrl_set(int pin, int state)
{
	int ret = 0;
	printk("RLK_camera_flash_aw3644 aw3644_pinctrl_set in\n");
	//return 0;

	if (IS_ERR(aw3644_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case AW3644_PINCTRL_PIN_HWEN:
		if (state == AW3644_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3644_hwen_low))
			pinctrl_select_state(aw3644_pinctrl, aw3644_hwen_low);
		else if (state == AW3644_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3644_hwen_high))
			pinctrl_select_state(aw3644_pinctrl, aw3644_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);
       printk("RLK_camera_flash_aw3644 aw3644_pinctrl_set out\n");
	return ret;
}
//torch (mA) = (Code * 2.91) + 2.55 (mA)//125*2
static const unsigned char aw3644_torch_level[AW3644_LEVEL_NUM] = {
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//flash (mA) = (Code * 11.72) + 11.35 (mA) //200.300.400.500.600.700.750 * 2
static const unsigned char aw3644_flash_level[AW3644_LEVEL_NUM] = {
	0x10, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1d, 0x1f,
	0x21, 0x23, 0x25, 0x27, 0x29, 0x2b, 0x2d, 0x2f, 0x31, 0x33,
	0x35, 0x37, 0x39, 0x3d, 0x3f, 0x41};


static int aw3644_verity_level(void)
{
	if(main_duty > AW3644_LEVEL_NUM - 1 )
	{
		main_duty = AW3644_LEVEL_NUM - 1;
	}
	else if(main_duty < 0 )
	{
		main_duty = 0;
	}
	if(sub_duty > AW3644_LEVEL_NUM - 1 )
	{
		sub_duty = AW3644_LEVEL_NUM - 1;
	}
	else if(sub_duty < 0 )
	{
		sub_duty = 0;
	}
	return 0;
}

/* flashlight enable function */
static int aw3644_enable(int channel)//wwsub
{

	printk("RLK_camera_flash_aw3644 aw3644_enable:%d\n",channel);
	aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_HIGH);
	aw3644_verity_level();
	if (channel == AW3644_CHANNEL_CH0)
	{
	    printk("aw3644_enable main_duty=%d\n",main_duty);
	    if(main_duty == 4)
	    {
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED1,aw3644_torch_level[0]&0x7F);//clean bit 7 to 0 ,then led2 not be override
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED2,aw3644_torch_level[0]&0x7F);//bit 1~6
			msleep(3);
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE,0x0B );//torch mode ,enable led2
		    printk("aw3644_enable CH0 torch mode done\n");
	    }
	    else
	    {
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED1,aw3644_flash_level[sub_duty]&0x7F);//clean bit 7 to 0 ,then led2 not be override
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED2,aw3644_flash_level[main_duty]&0x7F);
			msleep(3);
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE,0x0F );//flash mode ,enable led2
		    printk("aw3644_enable CH0 flash mode done\n");
	    }
	}
	else if(channel == AW3644_CHANNEL_CH1)
	{
	    printk("aw3644_enable sub_duty=%d\n",sub_duty);
	    /*if(main_duty == 4)
	    {
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED1,aw3644_torch_level[0]&0x7F);//clean bit 7 to 0 ,then led2 not be override
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED2,aw3644_torch_level[0]&0x7F);//bit 1~6
			msleep(3);
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE,0x0B );//torch mode ,enable led2
		    printk("aw3644_enable CH0 torch mode done\n");
	    }
	    else
	    {
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED1,aw3644_flash_level[sub_duty]&0x7F);//clean bit 7 to 0 ,then led2 not be override
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED2,aw3644_flash_level[main_duty]&0x7F);
		    aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE,0x0F );//flash mode ,enable led2
			msleep(3);
		    printk("aw3644_enable CH0 flash mode done\n");
	    }*/
	}
	else
	{
	    pr_err("Error channel\n");
	    return -1;
	}

	return 0;
}

/* flashlight disable function */
static int aw3644_disable_main(void)
{
     aw3644_flash_write(AW3644_i2c_client,AW3644_REG_ENABLE,0x00);
     return 0;
}

static int aw3644_disable_sub(void)
{
     aw3644_flash_write(AW3644_i2c_client,AW3644_REG_ENABLE,0x00);
     return 0;
}

static int aw3644_disable(void)
{
	printk("RLK_camera_flash_aw3644__aw3644_disable\n");
	rgt_torch_level2 = 0;
	aw3644_disable_main();
	aw3644_disable_sub();
	return 0;
}

/* set flashlight level */
static int aw3644_set_level(int channel,int level)//wwsub
{
    if (channel == AW3644_CHANNEL_CH0)
    {
	    if (level < 0)
	    {
		    level = 0;
	    }
	    main_duty = level;
	    printk("RLK_camera_flash_aw3644__aw3644_set_level main_duty = %d\n",main_duty);
    }
    else if (channel == AW3644_CHANNEL_CH1)
    {
	    if (level < 0)
	    {
		    level = 0;
	    }
	    sub_duty = level;
	    printk("RLK_camera_flash_aw3644__aw3644_set_level sub_duty = %d\n",sub_duty);
    }
    else
    {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
static int aw3644_init(void)
{
	printk("RLK_camera_flash_aw3644__aw3644_init\n");
	aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_HIGH);
	aw3644_flash_write(AW3644_i2c_client,AW3644_REG_ENABLE,0x00);
	aw3644_flash_write(AW3644_i2c_client,AW3644_REG_BOOST_CONFIG,0x09);
	aw3644_flash_write(AW3644_i2c_client,AW3644_REG_TIMING_CONFIG,0x1f);

	return 0;
}

/* flashlight uninit */
static int aw3644_uninit(void)
{
	aw3644_disable();
	// aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_LOW);

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw3644_timer;
static unsigned int aw3644_timeout_ms;

static void aw3644_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	aw3644_disable();
}

static enum hrtimer_restart aw3644_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3644_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw3644_ioctl(unsigned int cmd, unsigned long arg)
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
		aw3644_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3644_set_level(channel,fl_arg->arg);//wwsub
		//aw3644_set_level(AW3644_CHANNEL_CH1,fl_arg->arg);//wwsub
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw3644_timeout_ms) {
				ktime = ktime_set(aw3644_timeout_ms / 1000,
						(aw3644_timeout_ms % 1000) * 1000000);
				hrtimer_start(&aw3644_timer, ktime, HRTIMER_MODE_REL);
			}
			aw3644_enable(channel);//wwsub
		} else {
		    /*if(0 == channel)//wwsub
		    {
		        aw3644_disable_main();
		    }
		    else
		    {
		      	aw3644_disable_sub();
		    }
			*/
			aw3644_disable_main();
			aw3644_disable_sub();
			hrtimer_cancel(&aw3644_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3644_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int aw3644_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw3644_mutex);
	use_count--;
	if (!use_count)
		aw3644_uninit();
	aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_LOW);
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw3644_mutex);

	printk("Release: %d\n", use_count);

	return 0;
}

static int aw3644_set_driver(int set)
{
    printk("aw3644_set_driver in\n");
	/* init chip and set usage count */
	mutex_lock(&aw3644_mutex);
	if (!use_count)
		aw3644_init();
	use_count++;
	mutex_unlock(&aw3644_mutex);

	printk("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw3644_strobe_store(struct flashlight_arg arg)
{
	aw3644_set_driver(0);
	aw3644_set_level(arg.channel,arg.level);//wwsub
	aw3644_enable(arg.channel);
	msleep(arg.dur);
	aw3644_disable();
	aw3644_release();

	return 0;
}

static struct flashlight_operations aw3644_ops = {
	aw3644_open,
	aw3644_release,
	aw3644_ioctl,
	aw3644_strobe_store,
	aw3644_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw3644_chip_init(struct aw3644_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw3644_init();
	 */

	return 0;
}

static int aw3644_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw3644_chip_data *chip;
	struct aw3644_platform_data *pdata = client->dev.platform_data;
	int err;

	printk("aw3644_i2c_probe start.\n");
	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}
	/* init chip private data */
	chip = kzalloc(sizeof(struct aw3644_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_debug("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct aw3644_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("Failed to allocate memory.\n");
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	AW3644_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw3644_work, aw3644_work_disable);

	/* init timer */
	hrtimer_init(&aw3644_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3644_timer.function = aw3644_timer_func;
	aw3644_timeout_ms = 100;

	/* init chip hw */
	aw3644_chip_init(chip);
	/* register flashlight operations */
	if (flashlight_dev_register(AW3644_NAME, &aw3644_ops)) {
		err = -EFAULT;
		goto err_free;
	}
	/* clear usage count */
	use_count = 0;

	printk("Probe done.\n");

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw3644_i2c_remove(struct i2c_client *client)
{
	struct aw3644_chip_data *chip = i2c_get_clientdata(client);

	printk("Remove start.\n");

	/* flush work queue */
	flush_work(&aw3644_work);

	/* unregister flashlight operations */
	flashlight_dev_unregister(AW3644_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	printk("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw3644_i2c_id[] = {
	{AW3644_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, aw3644_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id aw3644_i2c_of_match[] = {
	{.compatible = AW3644_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, aw3644_i2c_of_match);
#endif

static struct i2c_driver aw3644_i2c_driver = {
	.driver = {
		   .name = AW3644_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw3644_i2c_of_match,
#endif
		   },
	.probe = aw3644_i2c_probe,
	.remove = aw3644_i2c_remove,
	.id_table = aw3644_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
//extern  int mt_set_gpio_pull_enable(unsigned long pin, unsigned long enable);
//extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
//extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
//extern int mt_set_gpio_pull_select(unsigned long pin, unsigned long select);
// int mt_set_gpio_out(unsigned long pin, unsigned long output)


static int aw3644_probe(struct platform_device *dev)
{
	printk("aw3644_platform_probe start.\n");
	/*
	mt_set_gpio_mode(GPIO_CAMERA_HWEN_PIN,0);
	mt_set_gpio_dir(GPIO_CAMERA_HWEN_PIN,1);
	mt_set_gpio_pull_enable(GPIO_CAMERA_HWEN_PIN, true);
	mt_set_gpio_pull_select(GPIO_CAMERA_HWEN_PIN, 1);
	mt_set_gpio_out(GPIO_CAMERA_HWEN_PIN,1);
	*/
	/* init pinctrl */
	if (aw3644_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
	//	return -1;
	}

	if (i2c_add_driver(&aw3644_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	printk("aw3644_probe done.\n");

	return 0;
}

static int aw3644_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&aw3644_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3644_of_match[] = {
	{.compatible = AW3644_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3644_of_match);
#else
static struct platform_device aw3644_platform_device = {

		.name = AW3644_NAME,
		.id = 0,
		.dev = {}

};
MODULE_DEVICE_TABLE(platform, aw3644_platform_device);
#endif

static struct platform_driver aw3644_platform_driver = {
	.probe = aw3644_probe,
	.remove = aw3644_remove,
	.driver = {
		.name = AW3644_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3644_of_match,
#endif
	},
};

static int __init flashlight_aw3644_init(void)
{
	int ret;

	printk("flashlight_aw3644_initInit start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3644_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3644_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_aw3644_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&aw3644_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_aw3644_init);
module_exit(flashlight_aw3644_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight AW3644 Driver");


/**************************add by luyan.ye torch-mode and for factory&midtest flashlight***********************************/
static struct class *torch_class = NULL;
static struct device *torch_dev = NULL;
static struct device_attribute dev_attr_torch_level;

static ssize_t torch_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    printk("get torch value is:%d \n",rgt_torch_level2);
	return sprintf(buf, "%u\n", rgt_torch_level2);
}

static ssize_t torch_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
//mode :range[0~9]
//0: close
// 1~2:torch mode
//9: flash mode
	unsigned int  mode = 0;
	const int torch[2] = {0x2A,0x2A};  //torch (mA) = (Code * 2.91) + 2.55 (mA)  // 125*2

	sscanf(buf, "%d",&mode);
	printk("aw3644 torch_level_store mode=%d\n",mode);
	aw3644_init();
	rgt_torch_level2 = mode;
	if((mode>0)&&(mode<=2))
	{
		aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_HIGH);
		mdelay(3);
		//modify for IN2 flashlight by RLK_camera_flash_aw3644 20171214 start
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED1,0x00);//clean bit 7 to 0 ,then led2 not be override
		//modify for IN2 flashlight by RLK_camera_flash_aw3644 20171214 end
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED2,torch[mode-1]&0x7F);//bit 1~6
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE,0x0A );//torch mode ,enable led2
	}
	else if(mode == 9)
	{
	    //modify for IN2 flashlight by RLK_camera_flash_aw3644 20171214 start
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED1,0x00);//1000ma clean bit 7 to 0 ,then led2 not be override
		//modify for IN2 flashlight by RLK_camera_flash_aw3644 20171214 end
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED2,0x54);
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE,0x0E );//flash mode ,enable led2
	}
	else if((mode>=100)&&(mode<=163))
	{
		mode = mode - 100;
		//modify for IN2 flashlight by RLK_camera_flash_aw3644 20171214 start
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED1,0x00);
		//modify for IN2 flashlight by RLK_camera_flash_aw3644 20171214 end
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED2,(mode&0x7F));
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE, 0x0A);//torch mode ,enable led2

	}
	else
	{
		pr_debug(" close led!\n");
		aw3644_disable();
		aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_LOW);

	}

   	return size;
}

static DEVICE_ATTR_RW(torch_level);
// sys/class/torch/torch/
static int __init rgt_torch_level_init(void)
{
	pr_debug("start\n");
	//modify XLLWHLSE-4 bring up flashlight in x605 by luyan.ye 20180202 start
	torch_class = class_create(THIS_MODULE, "torch");
	//modify XLLWHLSE-4 bring up flashlight in x605 by luyan.ye 20180202 end
	if (IS_ERR(torch_class))
	{
		pr_debug("Unable to create class, err = %d\n", (int)PTR_ERR(torch_class));
		return 0;
	}
	torch_dev = device_create(torch_class, NULL, 0, 0,"torch");
	if(NULL == torch_dev)
	{
		pr_debug("device_create fail\n");
		return 0;
	}

	device_create_file(torch_dev, &dev_attr_torch_level);

	pr_debug("Done\n");
	return 0;
}

static void __exit rgt_torch_level_exit(void)
{
	device_remove_file(torch_dev, &dev_attr_torch_level);
	device_unregister(torch_dev);
	if(torch_class!=NULL)
	{
		class_destroy(torch_class);
	}
}


module_init(rgt_torch_level_init);
module_exit(rgt_torch_level_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<luyan.ye@reallytek.com>");
MODULE_DESCRIPTION("midtest flashlight");