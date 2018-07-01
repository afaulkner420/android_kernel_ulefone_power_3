/*****************************************************************************
 *
 * Filename:
 * ---------
 *   s5k3p3sx_otp.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   John Wei (MTK07407)
 *
 *============================================================================*/
#ifndef __CAM_CAL_H
#define __CAM_CAL_H
#include <linux/i2c.h>
unsigned int s5k3p3sx_otp_selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size);


#endif /* __CAM_CAL_H */

