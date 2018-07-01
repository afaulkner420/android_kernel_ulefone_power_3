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

#ifndef __CAMERA_FDVT_H__
#define __CAMERA_FDVT_H__

#include <linux/ioctl.h>
#define FDVT_IOC_MAGIC    'N'

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

struct MT6573FDVTRegIO {
	unsigned int  *pAddr;
	unsigned int  *pData;
	unsigned int  u4Count;
};

#ifdef CONFIG_COMPAT

struct compat_MT6573FDVTRegIO {
	compat_uptr_t pAddr;
	compat_uptr_t pData;
	unsigned int  u4Count;
};


#endif


/* below is control message */
#define MT6573FDVT_INIT_SETPARA_CMD       _IO(FDVT_IOC_MAGIC, 0x00)
#define MT6573FDVTIOC_STARTFD_CMD         _IO(FDVT_IOC_MAGIC, 0x01)
#define MT6573FDVTIOC_G_WAITIRQ           _IOR(FDVT_IOC_MAGIC, 0x02, unsigned int)
#define MT6573FDVTIOC_T_SET_FDCONF_CMD    _IOW(FDVT_IOC_MAGIC, 0x03, struct MT6573FDVTRegIO)
#define MT6573FDVTIOC_G_READ_FDREG_CMD    _IOWR(FDVT_IOC_MAGIC, 0x04, struct MT6573FDVTRegIO)
#define MT6573FDVTIOC_T_SET_SDCONF_CMD    _IOW(FDVT_IOC_MAGIC, 0x05, struct MT6573FDVTRegIO)
/* #define FDVT_DESTROY_CMD                _IO(FDVT_IOC_MAGIC, 0x10) */

#define MT6573FDVTIOC_T_DUMPREG           _IO(FDVT_IOC_MAGIC, 0x80)


/*#define FDVT_SET_CMD_CMD            _IOW(FDVT_IOC_MAGIC, 0x03, unsigned int)*/
/*#define FDVT_SET_PWR_CMD            _IOW(FDVT_IOC_MAGIC, 0x04, unsigned int)*/
/*#define FDVT_SET_ISR_CMD            _IOW(FDVT_IOC_MAGIC, 0x05, unsigned int)*/
/*#define FDVT_GET_CACHECTRLADDR_CMD  _IOR(FDVT_IOC_MAGIC, 0x06, int)*/


#ifdef CONFIG_COMPAT

#define COMPAT_MT6573FDVT_INIT_SETPARA_CMD       _IO(FDVT_IOC_MAGIC, 0x00)
#define COMPAT_MT6573FDVTIOC_STARTFD_CMD         _IO(FDVT_IOC_MAGIC, 0x01)
#define COMPAT_MT6573FDVTIOC_G_WAITIRQ           _IOR(FDVT_IOC_MAGIC, 0x02, unsigned int)
#define COMPAT_MT6573FDVTIOC_T_SET_FDCONF_CMD    _IOW(FDVT_IOC_MAGIC, 0x03, struct compat_MT6573FDVTRegIO)
#define COMPAT_MT6573FDVTIOC_G_READ_FDREG_CMD    _IOWR(FDVT_IOC_MAGIC, 0x04, struct compat_MT6573FDVTRegIO)
#define COMPAT_MT6573FDVTIOC_T_SET_SDCONF_CMD    _IOW(FDVT_IOC_MAGIC, 0x05, struct compat_MT6573FDVTRegIO)
#define COMPAT_MT6573FDVTIOC_T_DUMPREG           _IO(FDVT_IOC_MAGIC, 0x80)

#endif


#endif /* __CAMERA_FDVT_H__ */


























