#ifndef _BTL_CUSTOM_H_
#define _BTL_CUSTOM_H_

#define BTL_FP_BF3290 1
#define BTL_FP_BF3182 2
#define BTL_FP_BF3390 3

#define F_CHIP_TYPE BTL_FP_BF3290

#if	(F_CHIP_TYPE == BTL_FP_BF3182)
	#define FINGER_WIDTH	(72)
	#define FINGER_HEIGHT	(128)
#elif (F_CHIP_TYPE == BTL_FP_BF3390) 
	#define FINGER_WIDTH	(80)
	#define FINGER_HEIGHT	(80)
#else
	#define FINGER_WIDTH	(112)
	#define FINGER_HEIGHT	(96)
#endif

//for MT6797 USE
#define USE_SPI1_4GB_TEST (0)

//if save the image when enroll and match failed
#define SAVE_IMAGE (1)

//if version is ANDROID N
#define BTL_ANDROID_N (1)

//save the fingerprint template in file (not database)
#define  FINGER_DATA_FILE     1

#endif
