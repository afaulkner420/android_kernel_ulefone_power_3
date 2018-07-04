//////////////////////////////////////////////////////////////////////////
//	Filename:	xz_sensor.h
//	Description:	Head file of xz5183 sensor, declaration for register
//			of sensor
//	Author:		Steven Zeng, created
//	Date:		Jan 21, 2016
//	Copyright:	BetterLife Inc.
//////////////////////////////////////////////////////////////////////////
#ifndef _XZ_SENSOR_H_
#define _XZ_SENSOR_H_
#include "btlcustom.h"
#define VENDOR_ID		0x5183

#define BL2390E_V1_REV0	0x51830000
#define BL2390E_V1_REV1	0x51830001
#define BL2390E_V1_REV3     0x51830003

#define DUMMY_BYTE		0x00

//========================================================================
// 1 register address
//========================================================================

#define REGA_NAVI_FRM0_LOW			0x00
#define REGA_NAVI_FRM0_HIGH			0x01
#define REGA_NAVI_FRM1_LOW			0x02
#define REGA_NAVI_FRM1_HIGH			0x03
#define REGA_NAVI_FRM2_LOW			0x04
#define REGA_NAVI_FRM2_HIGH			0x05
#define REGA_NAVI_FRM3_LOW			0x06
#define REGA_NAVI_FRM3_HIGH			0x07
#define REGA_NAVI_FRM4_LOW			0x08
#define REGA_NAVI_FRM4_HIGH			0x09
#define REGA_NAVI_FRM5_LOW			0x0A
#define REGA_NAVI_FRM5_HIGH			0x0B
#define REGA_NAVI_FRM6_LOW			0x0C
#define REGA_NAVI_FRM6_HIGH			0x0D
#define REGA_NAVI_FRM7_LOW			0x0E
#define REGA_NAVI_FRM7_HIGH			0x0F

#define REGA_ADC_OPTION				0x07
#define REGA_TIME_INTERVAL_LOW		0x08
#define REGA_TIME_INTERVAL_HIGH		0x09

#define REGA_FRAME_NUM					0x0D	// number reset row vdd
#define REGA_RC_THRESHOLD_LOW			0x0E
#define REGA_RC_THRESHOLD_MID			0x0F
#define REGA_RC_THRESHOLD_HIGH		0x10
#define REGA_FINGER_TD_THRED_LOW	0x11
#define REGA_FINGER_TD_THRED_HIGH	0x12
/*
#if	(F_CHIP_TYPE == BTL_FP_BF3182)
	#define REGA_FINGER_DT_INTERVAL_LOW		0x25
	#define REGA_FINGER_DT_INTERVAL_HIGH	0x26
	#define REGA_HOST_CMD				0x34
	#define REGA_RX_DACP_HIGH			0x1C //
	#define REGA_RX_DACP_LOW			0x1B
#elif (F_CHIP_TYPE == BTL_FP_BF3390)
	#define REGA_FINGER_DT_INTERVAL_LOW		0x25
	#define REGA_FINGER_DT_INTERVAL_HIGH	0x26
	#define REGA_HOST_CMD				0x34
	#define REGA_RX_DACP_LOW			0x1D //
	#define REGA_RX_DACP_HIGH			0x1E
#else
	#define REGA_FINGER_DT_INTERVAL_LOW		0x0A
	#define REGA_FINGER_DT_INTERVAL_HIGH	0x0B
	#define REGA_HOST_CMD				0x13
	#define REGA_RX_DACP_LOW			0x1B //
	#define REGA_RX_DACP_HIGH			0x1C
#endif
*/
#define REGA_PIXEL_MAX_DELTA		0x18
#define REGA_PIXEL_MIN_DELTA		0x19
#define REGA_PIXEL_ERR_NUM			0x1A

#define REGA_RX_DACN				0x1D

#define REGA_FINGER_CAP				0x27
#define REGA_INTR_STATUS			0x28

#define REGA_GC_STAGE				0x31 //
#define REGA_IC_STAGE				0x32

#define REGA_VERSION_RD_EN			0x3A
#define REGA_F32K_CALI_LOW			0x3B
#define REGA_F32K_CALI_HIGH			0x3C

#define REGA_PUMP_VOLTAGE			0x3D	// bit[3:1]
#define REGA_DRIVER_VERSION			0x3E	// bit[6:4]

#define REGA_F32K_CALI_EN			0x3F

//========================================================================
// 2 register defition
//========================================================================

// work mode
#define MODE_IDLE					0x00
#define MODE_RC_DT					0x01
#define MODE_FG_DT					0x02
#define MODE_FG_PRINT				0x03
#define MODE_FG_CAP					0x04
#define MODE_NAVI					0x05
#define MODE_CLK_CALI				0x06
#define MODE_PIEXL_TEST				0x07


//========================================================================
// 3 struction
//========================================================================
#define INTSTATE_FINGERCAP (1 << 1)
#define INTSTATE_NAVIGATION (1 << 2)
typedef union U_INTR_STATUS_tag 
{
	uint8_t nStatus;
	struct 
	{
		int nEOfClk32KCali : 1;		// bit 0
		int nEOfFingerCap : 1;		// bit 1
		int nEOfNavi : 1;
		int nEOfReset : 1;
		int nEOfHW_AGC : 1;
		int nReserved : 3;
	} s;
} U_INTR_STATUS, *PU_INTR_STATUS;

#define BF229X_INTERRUPT_STATUS_CLK_CALI      (1<<0)
#define BF229X_INTERRUPT_STATUS_FINGER_DETECT      (1<<1)
#define BF229X_INTERRUPT_STATUS_NAVI_DETECT      (1<<2)
#define BF229X_INTERRUPT_STATUS_REEST_DETECT      (1<<3)

#endif	//_XZ_SENSOR_H_
