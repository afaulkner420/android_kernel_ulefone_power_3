/*
 * NXP tfa98xx smart pa driver
 *
 * Copyright (C) 2017 TRANSSION HOLDINGS
 * 
 * Author: achang.zhang@reallytek.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
 
#include <linux/kernel.h> 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>

#define MAX_BUFFER_SIZE	512
#ifdef CONFIG_MTK_I2C_EXTENSION
static char *I2CDMABuf;
static dma_addr_t I2CDMABuf_pa;
#else
static char I2CDMABuf[MAX_BUFFER_SIZE];
#endif

struct device_data {
	struct i2c_client *client;
	struct miscdevice miscdev;
	struct mutex lock;
};

static ssize_t tfa98xx_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct device_data *data = filp->private_data;
	int ret = 0;
	
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	
	mutex_lock(&data->lock);
	
#ifdef CONFIG_MTK_I2C_EXTENSION
	data->client->addr = (data->client->addr & I2C_MASK_FLAG);
	data->client->ext_flag |= I2C_DMA_FLAG;
	
	ret = i2c_master_recv(data->client,
				(unsigned char *)(uintptr_t)I2CDMABuf_pa,count);
#else
	ret = i2c_master_recv(data->client,
				(unsigned char *)(uintptr_t)I2CDMABuf,count);
#endif

	if (ret < 0) {
		mutex_unlock(&data->lock);
		pr_err("tfa98xx_read error\n");
		return ret;
	}
	
	if (copy_to_user(buf, I2CDMABuf, ret)) {
		mutex_unlock(&data->lock);
		pr_err("failed to copy to user space\n");
		return -EFAULT;
	}
	
	mutex_unlock(&data->lock);
	
	return ret;
}

static ssize_t tfa98xx_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct device_data *data = filp->private_data;
	int ret = 0;
	
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	
	mutex_lock(&data->lock);
	
	if (copy_from_user(I2CDMABuf, buf, count)) 
	{
		pr_err("failed to copy from user space\n");
		return -EFAULT;
	}
	
#ifdef CONFIG_MTK_I2C_EXTENSION
	data->client->addr = (data->client->addr & I2C_MASK_FLAG);
	data->client->ext_flag |= I2C_DMA_FLAG;
	
	ret = i2c_master_send(data->client,
				(unsigned char *)(uintptr_t)I2CDMABuf_pa,count);
#else
	ret = i2c_master_send(data->client,
				(unsigned char *)(uintptr_t)I2CDMABuf,count);
#endif
	
	mutex_unlock(&data->lock);
	
	if (ret != count) {
		pr_err("tfa98xx_write error\n");
		return -EIO;
	}
	
	return ret;
}

static int tfa98xx_open(struct inode *nodp, struct file *filp)
{
	struct device_data *data;
	
	data = container_of(filp->private_data, struct device_data,
				  miscdev);
	filp->private_data = data;
	
	return 0;
}

static long tfa98xx_ioctl(struct file *filp, unsigned int cmd, 
								unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	default:
		{
			break;
		}
	}
	return ret;
}

static const struct file_operations tfa98xx_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = tfa98xx_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tfa98xx_ioctl,
#endif
	.open = tfa98xx_open,
	.write = tfa98xx_write,
	.read = tfa98xx_read,
};

static int pinctrl_init(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct pinctrl *pinctrl;
	struct pinctrl_state *rst_h;
	struct pinctrl_state *rst_l;
	u32 reset;/* 0:no reset pin, 1:have reset pin */
	int ret;
	
	if (!node) {
		dev_err(&client->dev, "Device_node is null\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(node, "have-reset-pin", &reset);
	if (ret) {
		dev_err(&client->dev, "Couldn't read have-reset-pin: %d\n", ret);
		return ret;
	}
	
	pr_info("have-reset-pin: %u\n",reset);
	
	if (!reset) {
		return ret;
	}
	
	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&client->dev, "Cannot find pinctrl!\n");
		ret = PTR_ERR(pinctrl);
		goto end;
	}
	
	rst_h = pinctrl_lookup_state(pinctrl, "smartpa_rst_h");
	if (IS_ERR(rst_h)) {
		dev_err(&client->dev, "pinctrl err, smartpa_rst_h!\n");
		ret = PTR_ERR(rst_h);
		goto end;
	}
	
	rst_l = pinctrl_lookup_state(pinctrl, "smartpa_rst_l");
	if (IS_ERR(rst_l)) {
		dev_err(&client->dev, "pinctrl err, smartpa_rst_l!\n");
		ret = PTR_ERR(rst_l);
		goto end;
	}
	
	//reset
	ret = pinctrl_select_state(pinctrl, rst_l);
	if (ret) {
		dev_err(&client->dev, "pinctrl_select_state smartpa_rst_l failed!\n");
		goto end;
	}
	msleep(10);
	ret = pinctrl_select_state(pinctrl, rst_h);
	if (ret) {
		dev_err(&client->dev, "pinctrl_select_state smartpa_rst_h failed!\n");
		goto end;
	}
	msleep(10);
	ret = pinctrl_select_state(pinctrl, rst_l);
	if (ret) {
		dev_err(&client->dev, "pinctrl_select_state smartpa_rst_l failed!\n");
		goto end;
	}

end:
	return ret;
}

static int tfa98xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_data *data;
	int ret;
	
	pr_info("tfa98xx i2c driver probe enter!\n");
	
	ret = pinctrl_init(client);
	if (ret) {
		dev_err(&client->dev, "pinctrl_init failed\n");
		return ret;
	}
	
	data = kzalloc(sizeof(struct device_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	
	data->client = client;
	mutex_init(&data->lock);
	
	data->miscdev.fops = &tfa98xx_fops;
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = "smartpa_i2c";
	
	ret = misc_register(&data->miscdev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to misc_register\n");
		goto err_misc_register;
	}
	
#ifdef CONFIG_MTK_I2C_EXTENSION	
	I2CDMABuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
					(dma_addr_t *) &I2CDMABuf_pa,GFP_KERNEL);
	if (!I2CDMABuf) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to dma_alloc_coherent\n");
		goto err_dam_alloc;
	}
#endif

	i2c_set_clientdata(client, data);
	
	pr_info("tfa98xx i2c driver probe ok!\n");
	
	return 0;
#ifdef CONFIG_MTK_I2C_EXTENSION		
err_dam_alloc:
	misc_deregister(&data->miscdev);
#endif
err_misc_register:
	mutex_destroy(&data->lock);
	kfree(data);
	
	return ret;
}

static int tfa98xx_remove(struct i2c_client *client)
{
	struct device_data *data = i2c_get_clientdata(client);
	
	misc_deregister(&data->miscdev);
	mutex_destroy(&data->lock);
	kfree(data);
#ifdef CONFIG_MTK_I2C_EXTENSION
	if(I2CDMABuf)
	{	
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMABuf, I2CDMABuf_pa);
		I2CDMABuf = NULL;
		I2CDMABuf_pa = 0;
	}
#endif

	return 0;
}

static const struct of_device_id smartpa_of_match[] = {
	{ .compatible = "nxp,tfa98xx", },
	{},
};
static const struct i2c_device_id tfa98xx_id[] = {
	{ "tfa98xx", 0 },
	{}
}
MODULE_DEVICE_TABLE(i2c, tfa98xx_id);
static struct i2c_driver tfa98xx_driver = {
	.driver = {
		.name	= "tfa98xx",
		.of_match_table = smartpa_of_match,
	},
	.probe		= tfa98xx_probe,
	.remove		= tfa98xx_remove,
	.id_table   = tfa98xx_id,
	
};

module_i2c_driver(tfa98xx_driver);

/* Module information */
MODULE_AUTHOR("achang.zhang@reallytek.com");
MODULE_DESCRIPTION("NXP Smart PA Tfa98xx driver");
MODULE_LICENSE("GPL v2");
