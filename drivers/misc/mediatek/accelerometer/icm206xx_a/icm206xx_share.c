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

#include <hwmsensor.h>
#include "cust_acc.h"
#include "accel.h"
#include "icm206xx_register_20608D.h"
#include "icm206xx_share.h"
#include "icm206xx_share_interface.h"

/*=======================================================================================*/
/* Shared Vendor Specific Functions Section				 		 */
/*=======================================================================================*/

struct icm206xx_sensor_status_info {
	bool sensor_power;
	int sample_rate;
};

static struct icm206xx_sensor_status_info icm206xx_all_sensor_status_info[ICM206XX_SENSOR_TYPE_MAX];

static void icm206xx_set_sensor_power(int paramSensor, bool paramPower)
{
	icm206xx_all_sensor_status_info[paramSensor].sensor_power = paramPower;
}

static bool icm206xx_get_sensor_power(int paramSensor)
{
	return icm206xx_all_sensor_status_info[paramSensor].sensor_power;
}

static bool icm206xx_any_step_sensor_is_on(int paramSensor)
{
/* return true if any other step counter sensors are enabled except itself */
/* step counter sensors are 
	   : ICM206XX_SENSOR_TYPE_STEP_COUNTER, 
	   : ICM206XX_SENSOR_TYPE_STEP_DETECTOR, 
	   : ICM206XX_SENSOR_TYPE_SIGNIFICANT_MOTION 
*/

	bool step_sensor_power[3];
	int i;

	for (i = 0; i < 3; i++)
		step_sensor_power[i] = icm206xx_get_sensor_power(i + ICM206XX_SENSOR_TYPE_STEP_COUNTER);

	if (paramSensor >= ICM206XX_SENSOR_TYPE_STEP_COUNTER)
		step_sensor_power[paramSensor - ICM206XX_SENSOR_TYPE_STEP_COUNTER] = false;

	if ((step_sensor_power[0] == true) || (step_sensor_power[1] == true) || (step_sensor_power[2] == true))
		return true;
	else
		return false;
}

static int icm206xx_ChipSoftReset(void)
{
	u8 databuf[10];
	int res = 0;
	int i;

	memset(databuf, 0, sizeof(u8) * 10);

	// read 
	res = icm206xx_share_read_register(ICM206XX_REG_PWR_CTL, databuf, 0x1);
	if (res < 0)
	{
		ACC_ERR("read power ctl register err!\n");
		return ICM206XX_ERR_BUS;
	}

	// set device_reset bit to do soft reset
	databuf[0] |= ICM206XX_BIT_DEVICE_RESET;
	res = icm206xx_share_write_register(ICM206XX_REG_PWR_CTL, databuf, 0x1);
	if (res < 0)
	{
		ACC_ERR("write power ctl register err!\n");
		return ICM206XX_ERR_BUS;
	}

	mdelay(100);

	// sensor status rset
	for (i = 0; i < ICM206XX_SENSOR_TYPE_MAX; i++)
	{
		icm206xx_all_sensor_status_info[i].sensor_power = false;
		icm206xx_all_sensor_status_info[i].sample_rate = 0;
	}

	return ICM206XX_SUCCESS;
}

static int icm206xx_SetPowerMode(int sensor_type, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	int i;

	if(sensor_type >= ICM206XX_SENSOR_TYPE_MAX)
	{
		return ICM206XX_ERR_INVALID_PARAM;
	}

	icm206xx_all_sensor_status_info[sensor_type].sensor_power = enable;

	res = icm206xx_share_read_register(ICM206XX_REG_PWR_CTL, databuf, 1);
	if (res < 0) {
		ACC_ERR("read power ctl register err!\n");
		return ICM206XX_ERR_BUS;
	}

	databuf[0] &= ~ICM206XX_BIT_SLEEP;
	if (enable == false)
	{
		/* Check power status of all sensors */
		for(i = 0; i < ICM206XX_SENSOR_TYPE_MAX; i++)
		{
			if(icm206xx_all_sensor_status_info[i].sensor_power == true)
				break;
		}

		/* Go to sleep mode when all sensors are disabled */
		if(i == ICM206XX_SENSOR_TYPE_MAX)
		{
			databuf[0] |= ICM206XX_BIT_SLEEP;	
		}
	}

	res = icm206xx_share_write_register(ICM206XX_REG_PWR_CTL, databuf, 1);
	if (res < 0) {
		ACC_ERR("set power mode failed!\n");
		return ICM206XX_ERR_BUS;
	}

	mdelay(5);

	ACC_LOG("set power mode ok %d!\n", enable);

	return ICM206XX_SUCCESS;
}

static int icm206xx_EnableInterrupt(u8 int_type, bool enable)
{
	int res = 0;

	u8 databuf[2] = {0};

	res = icm206xx_share_read_register(ICM206XX_REG_INT_ENABLE, databuf, 1);
	if (res < 0) {
		ACC_ERR("read interrupt register err!\n");
		return ICM206XX_ERR_BUS;
	}

	if(enable == true)
		databuf[0] |= int_type;
	else
		databuf[0] &= ~int_type;

	res = icm206xx_share_write_register(ICM206XX_REG_INT_ENABLE, databuf, 1);

	if (res < 0) {
		ACC_ERR("write interrupt register err!\n");
		return ICM206XX_ERR_BUS;
	}

	return ICM206XX_SUCCESS;
}

static int icm206xx_EnableSensor(int sensor_type, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;

	if(sensor_type >= ICM206XX_SENSOR_TYPE_MAX)
	{
		return ICM206XX_ERR_INVALID_PARAM;
	}

	res = icm206xx_share_read_register(ICM206XX_REG_PWR_CTL2, databuf, 1);
	if (res < 0) {
		ACC_ERR("read power ctl2 register err!\n");
		return ICM206XX_ERR_BUS;
	}

	if(enable == true)
	{
		if (sensor_type == ICM206XX_SENSOR_TYPE_GYRO)
		{
			databuf[0] &= ~ICM206XX_BIT_GYRO_STANDBY;		
		}
		else if(sensor_type == ICM206XX_SENSOR_TYPE_ACC || 
			sensor_type == ICM206XX_SENSOR_TYPE_STEP_COUNTER ||
			sensor_type == ICM206XX_SENSOR_TYPE_STEP_DETECTOR || 
			sensor_type == ICM206XX_SENSOR_TYPE_SIGNIFICANT_MOTION)
		{
			databuf[0] &= ~ICM206XX_BIT_ACCEL_STANDBY;
	
		}
	}
	else
	{
		if (sensor_type == ICM206XX_SENSOR_TYPE_GYRO)
		{
			databuf[0] |= ICM206XX_BIT_GYRO_STANDBY;		
		}
		else if(sensor_type == ICM206XX_SENSOR_TYPE_ACC)
		{
			if (icm206xx_any_step_sensor_is_on(sensor_type) == false)
				databuf[0] |= ICM206XX_BIT_ACCEL_STANDBY;
		}
		else
		{
			if ((icm206xx_any_step_sensor_is_on(sensor_type) == false) && 
			    (icm206xx_get_sensor_power(ICM206XX_SENSOR_TYPE_ACC) == false))
				databuf[0] |= ICM206XX_BIT_ACCEL_STANDBY;

		}
	}
	
	res = icm206xx_share_write_register(ICM206XX_REG_PWR_CTL2, databuf, 1);
	if (res < 0) {
		ACC_ERR("set power ctl2 failed!\n");
		return ICM206XX_ERR_BUS;
	}

	mdelay(50);	

	return ICM206XX_SUCCESS;
}

static int icm206xx_ReadChipInfo(char *buf, int bufsize)
{
	u8 databuf[2] = {0};
	int res = 0;

	if ((NULL == buf) || (bufsize <= 30))
		return -1;

	res = icm206xx_share_read_register(ICM206XX_REG_WHO_AM_I, databuf, 1);
	if (res < 0) {
		ACC_ERR("read who_am_i register err!\n");
		return ICM206XX_ERR_BUS;
	}

	switch (databuf[0])
	{
	case ICM20608D_WHO_AM_I:
		sprintf(buf, "ICM20608D [0x%x]", databuf[0]);		
		break;
	default:
		sprintf(buf, "Unknown Sensor [0x%x]", databuf[0]);
		break;	
	}

	return ICM206XX_SUCCESS;
}

static int icm206xx_SetSampleRate(int sensor_type, u64 delay_ns, bool force_1khz)
{
	u8 databuf[2] = {0};
	int sample_rate = 0;
	int rate_div = 0;
	int res = 0;
	int i, highest_sample_rate = 0;

	/* ---------------------------------------------------- */
	/* Normal  : 200ms (200,000,000ns) 	: 5hz		*/
	/* UI 	   : 66ms (66,670,000ns)   	: 15hz		*/
	/* Game    : 20ms (20,000,000ns)  	: 50hz		*/
	/* Fastest : 5ms (5,000,000ns)    	: 200hz		*/
	/* 	Actual Delay of Accel : 10ms (100hz)		*/
	/* 	Actual Delay of Gyro : 2ms (500hz)		*/
	/* ---------------------------------------------------- */
	sample_rate = (int)(delay_ns / 1000);		// ns to us
	sample_rate = (int)(sample_rate / 1000);	// us to ms

	if(sample_rate != 0)
		sample_rate = (int)(1000 / sample_rate); 	// ns to hz

	/* sample rate: 5hz to 200hz; */
	/* when force_1khz is true, it means self test mode is running at 1khz */
	if ((sample_rate > 200) && (force_1khz == false))  
		sample_rate = 200;
	else if (sample_rate < 5)
		sample_rate = 5;	

	if(icm206xx_all_sensor_status_info[sensor_type].sample_rate == sample_rate)
		return ICM206XX_SUCCESS;

	icm206xx_all_sensor_status_info[sensor_type].sample_rate = sample_rate;

	/* Check sample rate of enabled sensors */
	for(i = 0; i < ICM206XX_SENSOR_TYPE_MAX; i++)
	{
		if(icm206xx_all_sensor_status_info[i].sensor_power == true)
		{
			if(highest_sample_rate < icm206xx_all_sensor_status_info[i].sample_rate)
				highest_sample_rate = icm206xx_all_sensor_status_info[i].sample_rate;
		}
	}

	if(highest_sample_rate == 0)
	{
		/* Every sensor is disabled */
		highest_sample_rate = icm206xx_all_sensor_status_info[sensor_type].sample_rate;
	}

	/* Change sample rate */
	rate_div = ICM206XX_INTERNAL_SAMPLE_RATE / highest_sample_rate - 1;

	databuf[0] = rate_div;
	res = icm206xx_share_write_register(ICM206XX_REG_SAMRT_DIV, databuf, 1);
	if (res < 0) {
		ACC_ERR("write sample rate register err!\n");
		return ICM206XX_ERR_BUS;
	}

	/* read sample div after written for test */
	res = icm206xx_share_read_register(ICM206XX_REG_SAMRT_DIV, databuf, 1);
	if (res < 0) {
		ACC_ERR("read accel sample rate register err!\n");
		return ICM206XX_ERR_BUS;
	}
		
	ACC_LOG("read accel sample rate: 0x%x\n", databuf[0]);
	
	return ICM206XX_SUCCESS;
}

/*=======================================================================================*/
/* Export Symbols Section					 			 */
/* -- To make this module as an Entry module to access other INVN sensors such as Gyro	 */
/*=======================================================================================*/

/*----------------------------------------------------------------------------*/
void icm206xx_share_set_sensor_power(int paramSensor, bool paramPower)
{
	icm206xx_set_sensor_power(paramSensor, paramPower);
}
EXPORT_SYMBOL(icm206xx_share_set_sensor_power);

bool icm206xx_share_get_sensor_power(int paramSensor)
{
	return icm206xx_get_sensor_power(paramSensor);
}
EXPORT_SYMBOL(icm206xx_share_get_sensor_power);

bool icm206xx_share_any_step_sensor_is_on(int paramSensor)
{	
	return icm206xx_any_step_sensor_is_on(paramSensor);
}
EXPORT_SYMBOL(icm206xx_share_any_step_sensor_is_on);

int icm206xx_share_ChipSoftReset(void)
{
	return icm206xx_ChipSoftReset();
}
EXPORT_SYMBOL(icm206xx_share_ChipSoftReset);

int icm206xx_share_SetPowerMode(int sensor_type, bool enable)
{
	return icm206xx_SetPowerMode(sensor_type, enable);
}
EXPORT_SYMBOL(icm206xx_share_SetPowerMode);

int icm206xx_share_EnableInterrupt(u8 int_type, bool enable)
{
	return icm206xx_EnableInterrupt(int_type, enable);
}
EXPORT_SYMBOL(icm206xx_share_EnableInterrupt);

int icm206xx_share_EnableSensor(int sensor_type, bool enable)
{
	return icm206xx_EnableSensor(sensor_type, enable);
}
EXPORT_SYMBOL(icm206xx_share_EnableSensor);

int icm206xx_share_ReadChipInfo(char *buf, int bufsize)
{
	return icm206xx_ReadChipInfo(buf, bufsize);
}
EXPORT_SYMBOL(icm206xx_share_ReadChipInfo);

int icm206xx_share_SetSampleRate(int sensor_type, u64 delay_ns, bool force_1khz)
{
	return icm206xx_SetSampleRate(sensor_type, delay_ns, force_1khz);
}
EXPORT_SYMBOL(icm206xx_share_SetSampleRate);
/*----------------------------------------------------------------------------*/


