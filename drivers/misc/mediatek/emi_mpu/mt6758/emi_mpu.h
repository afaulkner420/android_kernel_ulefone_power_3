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

#ifndef __MT_EMI_MPU_H
#define __MT_EMI_MPU_H

#define ENABLE_AP_REGION	0

#define EMI_MPUD0_ST            (CEN_EMI_BASE + 0x160)
#define EMI_MPUD_ST(domain)     (EMI_MPUD0_ST + (4*domain)) /* violation status domain */
#define EMI_MPUD0_ST2           (CEN_EMI_BASE + 0x200)     /* violation status domain */
#define EMI_MPUD_ST2(domain)    (EMI_MPUD0_ST2 + (4*domain))/* violation status domain */
#define EMI_MPUS                (CEN_EMI_BASE + 0x01F0)    /* Memory protect unit control registers S */
#define EMI_MPUT                (CEN_EMI_BASE + 0x01F8)    /* Memory protect unit control registers S */
#define EMI_MPUT_2ND            (CEN_EMI_BASE + 0x01FC)    /* Memory protect unit control registers S */

#define EMI_WP_ADR              (CEN_EMI_BASE + 0x5E0)
#define EMI_WP_ADR_2ND          (CEN_EMI_BASE + 0x5E4)
#define EMI_WP_CTRL             (CEN_EMI_BASE + 0x5E8)
#define EMI_CHKER               (CEN_EMI_BASE + 0x5F0)
#define EMI_CHKER_TYPE          (CEN_EMI_BASE + 0x5F4)
#define EMI_CHKER_ADR           (CEN_EMI_BASE + 0x5F8)
#define EMI_CHKER_ADR_2ND       (CEN_EMI_BASE + 0x5FC)

#define EMI_MPU_CTRL                 (0x0)
#define EMI_MPU_DBG                  (0x4)
#define EMI_MPU_SA0                  (0x100)
#define EMI_MPU_EA0                  (0x200)
#define EMI_MPU_SA(region)           (EMI_MPU_SA0 + (region*4))
#define EMI_MPU_EA(region)           (EMI_MPU_EA0 + (region*4))
#define EMI_MPU_APC0                 (0x300)
#define EMI_MPU_APC(region, domain)  (EMI_MPU_APC0 + (region*4) + ((domain/8)*0x100))
#define EMI_MPU_CTRL_D0              (0x800)
#define EMI_MPU_CTRL_D(domain)       (EMI_MPU_CTRL_D0 + (domain*4))
#define EMI_RG_MASK_D0               (0x900)
#define EMI_RG_MASK_D(domain)        (EMI_RG_MASK_D0 + (domain*4))

#define NO_PROTECTION   0
#define SEC_RW          1
#define SEC_RW_NSEC_R   2
#define SEC_RW_NSEC_W   3
#define SEC_R_NSEC_R    4
#define FORBIDDEN       5
#define SEC_R_NSEC_RW   6

#define EN_MPU_STR "ON"
#define DIS_MPU_STR "OFF"

#define EN_WP_STR "ON"
#define DIS_WP_STR "OFF"

#define MAX_CHANNELS  (2)
#define MAX_RANKS     (2)

#define EMI_MPU_REGION_NUMBER	(32)
#define EMI_MPU_DOMAIN_NUMBER	(16)

enum {
	AXI_VIO_ID = 0,
	AXI_ADR_CHK_EN = 16,
	AXI_LOCK_CHK_EN = 17,
	AXI_NON_ALIGN_CHK_EN = 18,
	AXI_NON_ALIGN_CHK_MST = 20,
	AXI_VIO_CLR = 24,
	AXI_VIO_WR = 27,
	AXI_ADR_VIO = 28,
	AXI_LOCK_ISSUE = 29,
	AXI_NON_ALIGN_ISSUE = 30
};

#define EMI_WP_RANGE        0x0000003F
#define EMI_WP_AXI_ID       0x0000FFFF
#define EMI_WP_RW_MONITOR   0x000000C0
#define EMI_WP_RW_DISABLE   0x00000300
#define WP_BOTH_READ_WRITE  3

#define EMI_WP_RW_MONITOR_SHIFT  6
#define EMI_WP_RW_DISABLE_SHIFT  8
#define EMI_WP_SLVERR_SHIFT     10
#define EMI_WP_INT_SHIFT        15
#define EMI_WP_ENABLE_SHIFT     19
#define EMI_WP_VIO_CLR_SHIFT    24

enum {
	MASTER_APMCU_CH1 = 0,
	MASTER_APMCU_CH2 = 1,
	MASTER_MM_CH1 = 2,
	MASTER_MD_C2K = 3,
	MASTER_PERI = 4,
	MASTER_MDDMA = 4,
	MASTER_MM_CH2 = 5,
	MASTER_MFG_0 = 6,
	MASTER_MFG_1 = 7,
	MASTER_ALL = 8
};

/* Basic DRAM setting */
struct basic_dram_setting {
	/* Number of channels */
	unsigned channel_nr;
	/* Per-channel information */
	struct {
		/* Per-rank information */
		struct {
			/* Does this rank exist */
			bool valid_rank;
			/* Rank size - (in Gb)*/
			unsigned rank_size;
			/* Number of segments */
			unsigned segment_nr;
		} rank[MAX_RANKS];
	} channel[MAX_CHANNELS];
};

#define LOCK	1
#define UNLOCK	0

#define SET_ACCESS_PERMISSON(lock, d15, d14, d13, d12, d11, d10, d9, d8, d7, d6, d5, d4, d3, d2, d1, d0) \
((((unsigned long long) d15) << 53) | (((unsigned long long) d14) << 50) | (((unsigned long long) d13) << 47) | \
(((unsigned long long) d12) << 44) | (((unsigned long long) d11) << 41) | (((unsigned long long) d10) << 38) | \
(((unsigned long long) d9)  << 35) | (((unsigned long long) d8)  << 32) | (((unsigned long long) d7)  << 21) | \
(((unsigned long long) d6)  << 18) | (((unsigned long long) d5)  << 15) | (((unsigned long long) d4)  << 12) | \
(((unsigned long long) d3)  <<  9) | (((unsigned long long) d2)  <<  6) | (((unsigned long long) d1)  <<  3) | \
((unsigned long long) d0) | ((unsigned long long) lock << 26))

extern int emi_mpu_set_region_protection(unsigned long long start_addr,
unsigned long long end_addr, int region, unsigned long long access_permission);
#if defined(CONFIG_MTKPASR)
extern void acquire_dram_setting(struct basic_dram_setting *pasrdpd);
#endif
extern void emi_wp_get_status(void);
extern unsigned int mt_emi_mpu_irq_get(void);
extern void mt_emi_reg_base_set(void *base);
extern void *mt_emi_reg_base_get(void);
extern int emi_mpu_get_violation_port(void);
extern phys_addr_t get_max_DRAM_size(void);
extern unsigned int mt_emi_reg_read(unsigned int offset);
extern void mt_emi_reg_write(unsigned int data, unsigned int offset);
#endif  /* !__MT_EMI_MPU_H */
