/* 
 * ICM206XX sensor driver
 * Copyright (C) 2016 Invensense, Inc.
 *
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
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h> 
 
#include <hwmsensor.h>
#include "cust_acc.h"
#include "accel.h"
#include "icm206xx_register_20608D.h"
#include "icm206xx_share_interface.h"

/*=======================================================================================*/
/* Shared I2C Primitive Functions Section				 		 */
/*=======================================================================================*/

extern struct i2c_client *icm206xx_accel_i2c_client;

static DEFINE_MUTEX(icm206xx_accel_i2c_mutex);

static int icm206xx_i2c_read_register(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int res = 0;
	struct i2c_msg msgs[2] = {{0}, {0} };

	mutex_lock(&icm206xx_accel_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		ACC_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	}
	res = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (res != 2) {
		ACC_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, res);
		res = -EIO;
	} else
		res = 0;

	mutex_unlock(&icm206xx_accel_i2c_mutex);
	return res;
}

static int icm206xx_i2c_write_register(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int idx, num;
	int res = 0;
	char buf[C_I2C_FIFO_SIZE];

	mutex_lock(&icm206xx_accel_i2c_mutex);
	if (!client) {
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		ACC_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	res = i2c_master_send(client, buf, num);
	if (res < 0) {
		ACC_ERR("send command error!!\n");
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EFAULT;
	}
	else
		res = 0;
	mutex_unlock(&icm206xx_accel_i2c_mutex);

	return res;
}

#define		ONE_PACKAGE_MAX 	4

static int icm206xx_i2c_write_memory(struct i2c_client *client, u16 mem_addr,
						u32 len, u8 const *data)
{
#if 0
	u8 bank[2];
	u8 addr[2];
	u8 buf[513];

	struct i2c_msg msgs[3];
	int res = 0;

	if (len >= (sizeof(buf) - 1))
		return -ENOMEM;

	mutex_lock(&icm206xx_accel_i2c_mutex);

	bank[0] = ICM206XX_REG_MEM_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = ICM206XX_REG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf[0] = ICM206XX_REG_MEM_R_W;
	memcpy(buf + 1, data, len);

	/* write message */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = client->addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = client->addr;
	msgs[2].flags = 0;
	msgs[2].buf = (u8 *) buf;
	msgs[2].len = len + 1;

	if (!client) {
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	}

	res = i2c_transfer(client->adapter, msgs, 3);
	if (res != 3) {
		ACC_ERR("i2c_transfer error: (%d %p %d) %d\n", client->addr, data, len, res);
		res = -EIO;
	} else
		res = 0;

	mutex_unlock(&icm206xx_accel_i2c_mutex);

	return res;
#else
	u8 buffer[ONE_PACKAGE_MAX + 1];
	int res = 0;
	int write_size;
	u8 buf[513];
	//int size = 0;
	if (len >= (sizeof(buf) - 1))
		return -ENOMEM;

	mutex_lock(&icm206xx_accel_i2c_mutex);

	if (!client) {
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	}
	while (len > 0) 
	{
		if (len > ONE_PACKAGE_MAX)
			write_size = ONE_PACKAGE_MAX;
		else
			write_size = len;
	    //bank[0] = MPUREG_BANK_SEL;
    	//bank[1] = mem_addr >> 8;
		inv_i2c_single_write(client, ICM206XX_REG_MEM_BANK_SEL, (unsigned char)(mem_addr >> 8));
    	//addr[0] = MPUREG_MEM_START_ADDR;
    	//addr[1] = mem_addr & 0xFF;
		inv_i2c_single_write(client, ICM206XX_REG_MEM_START_ADDR, (unsigned char)(mem_addr & 0xFF));
		//buf[0] = MPUREG_MEM_R_W;
	    //memcpy(buf + 1, data, length);
		buffer[0] = ICM206XX_REG_MEM_R_W;
		memcpy(buffer+1, data, write_size);
		if(i2c_master_send(client, buffer, write_size + 1) <= 0)
		{
			printk(KERN_INFO "%s: transfer failed.", __func__);
			mutex_unlock(&icm206xx_accel_i2c_mutex);
			return -EIO;
		}

		data += write_size;
		mem_addr += write_size;
		len -= write_size;
	}


	mutex_unlock(&icm206xx_accel_i2c_mutex);

	return res;	
#endif
}

static int icm206xx_read_interface(struct i2c_client *client, u8 *data, u32 len)
{
	struct i2c_msg msgs[2]= { { 0 },{ 0 } };
	int res = -1;
//	mutex_lock(&icm206xx_accel_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = I2C_M_RD;
	msgs[0].buf = data;
	msgs[0].len = len;
	res = i2c_transfer(client->adapter, msgs, 1);
	if (res != 1)
	{
		printk("icm206xx_read_interface error! lens is :%d\n",len);
//		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -1;
	}
//	mutex_unlock(&icm206xx_accel_i2c_mutex);
	return 0;
}



static int icm206xx_i2c_read_memory(struct i2c_client *client, u16 mem_addr,
						u32 len, u8 *data)
{
#if 0
	u8 bank[2];
	u8 addr[2];
	u8 buf;

	struct i2c_msg msgs[4];
	int res = 0;

	mutex_lock(&icm206xx_accel_i2c_mutex);

	bank[0] = ICM206XX_REG_MEM_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = ICM206XX_REG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf = ICM206XX_REG_MEM_R_W;

	/* write message */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = client->addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = client->addr;
	msgs[2].flags = 0;
	msgs[2].buf = &buf;
	msgs[2].len = 1;

	msgs[3].addr = client->addr;
	msgs[3].flags = I2C_M_RD;
	msgs[3].buf = data;
	msgs[3].len = len;

	if (!client) {
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	}

	res = i2c_transfer(client->adapter, msgs, 4);
	if (res != 4) {
		ACC_ERR("i2c_transfer error: (%d %p %d) %d\n", client->addr, data, len, res);
		res = -EIO;
	} else
		res = 0;

	mutex_unlock(&icm206xx_accel_i2c_mutex);

	return res;
#else
	int res = 0;
	int read_size;
	//int size = len;
	u8 buffer[ONE_PACKAGE_MAX + 1];
	mutex_lock(&icm206xx_accel_i2c_mutex);

	if (!client) {
		mutex_unlock(&icm206xx_accel_i2c_mutex);
		return -EINVAL;
	}

	while (len > 0) {
		if (len > ONE_PACKAGE_MAX)
			read_size = ONE_PACKAGE_MAX;
		else
			read_size = len;
		inv_i2c_single_write(client, ICM206XX_REG_MEM_BANK_SEL, (unsigned char)(mem_addr >> 8));
		inv_i2c_single_write(client, ICM206XX_REG_MEM_START_ADDR, (unsigned char)(mem_addr & 0xFF));
		buffer[0] = ICM206XX_REG_MEM_R_W;
		
		if(i2c_master_send(client, buffer, 1) <= 0)
		{
			printk(KERN_INFO "%s: delfino transfer failed.", __func__);
			mutex_unlock(&icm206xx_accel_i2c_mutex);	
			return -EIO;
		}
		if(icm206xx_read_interface(client, data, read_size) != 0)
		{
			printk(KERN_INFO "%s: transfer failed.", __func__);
			mutex_unlock(&icm206xx_accel_i2c_mutex);
			return -EIO;
		}

		data += read_size;
		mem_addr += read_size;
		len -= read_size;
	}	

	mutex_unlock(&icm206xx_accel_i2c_mutex);

	return res;

#endif
}

int inv_i2c_single_write(struct i2c_client *client,
                         unsigned char reg, unsigned char value)
{
	static unsigned char buffer[2];
	buffer[0] = reg;
	buffer[1] = value;
	if(i2c_master_send(client, buffer, 2) <= 0)
	{
		printk(KERN_INFO "%s: transfer failed.", __func__);
		return -EIO;
	}
//		GSE_REG_RW("6050 gsensor reg:0x%x, value: 0x%x\n", reg, value);
	return 0;
}

/*=======================================================================================*/
/* Export Symbols Section					 			 */
/* -- To make this module as an Entry module to access other INVN sensors such as Gyro	 */
/*=======================================================================================*/

int icm206xx_share_read_register(u8 addr, u8 *data, u8 len)
{
	return icm206xx_i2c_read_register(icm206xx_accel_i2c_client, addr, data, len);
}
EXPORT_SYMBOL(icm206xx_share_read_register);

int icm206xx_share_write_register(u8 addr, u8 *data, u8 len)
{
	return icm206xx_i2c_write_register(icm206xx_accel_i2c_client, addr, data, len);
}
EXPORT_SYMBOL(icm206xx_share_write_register);

int icm206xx_share_read_memory(u16 mem_addr,u32 len, u8 *data)
{
	return icm206xx_i2c_read_memory(icm206xx_accel_i2c_client, mem_addr, len, data);
}
EXPORT_SYMBOL(icm206xx_share_read_memory);

int icm206xx_share_write_memory(u16 mem_addr,u32 len, u8 const *data)
{
	return icm206xx_i2c_write_memory(icm206xx_accel_i2c_client, mem_addr, len, data);
}
EXPORT_SYMBOL(icm206xx_share_write_memory);

/*----------------------------------------------------------------------------*/

