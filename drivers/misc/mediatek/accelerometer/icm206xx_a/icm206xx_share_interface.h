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

#ifndef ICM206XX_SHARE_INTERFACE_H
#define ICM206XX_SHARE_INTERFACE_H

/*=======================================================================================*/
/* Shared I2C Primitive 								 */
/*=======================================================================================*/

#define ICM206XX_SUCCESS             		0
#define ICM206XX_ERR_BUS             		-1
#define ICM206XX_ERR_INVALID_PARAM     		-2
#define ICM206XX_ERR_STATUS          		-3
#define ICM206XX_ERR_SETUP_FAILURE   		-4

/*----------------------------------------------------------------------------*/

extern int icm206xx_share_read_register(u8 addr, u8 *data, u8 len);
extern int icm206xx_share_write_register(u8 addr, u8 *data, u8 len);
extern int icm206xx_share_read_memory(u16 mem_addr,u32 len, u8 *data);
extern int icm206xx_share_write_memory(u16 mem_addr,u32 len, u8 const *data);
extern int inv_i2c_single_write(struct i2c_client *client,unsigned char reg, unsigned char value);


/*----------------------------------------------------------------------------*/

#endif /* ICM206XX_SHARE_INTERFACE_H */

