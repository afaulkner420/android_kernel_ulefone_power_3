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

#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/delay.h>
#include <linux/fs.h>

#include "autok_dvfs.h"
#include "mtk_sd.h"
#include <mmc/core/sdio_ops.h>

static char const * const sdio_autok_res_path[] = {
	"/data/sdio_autok_0", "/data/sdio_autok_1",
	"/data/sdio_autok_2", "/data/sdio_autok_3",
};

#define SDIO_AUTOK_DIFF_MARGIN  3

#ifdef CONFIG_MTK_SDIO_SUPPORT
static struct file *msdc_file_open(const char *path, int flags, int rights)
{
	struct file *filp = NULL;
#ifdef SDIO_HQA
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
#endif

	return filp;
}

static int msdc_file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
	int ret = 0;
#ifdef SDIO_HQA
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, &offset);

	set_fs(oldfs);
#endif

	return ret;
}

static int msdc_file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
	int ret = 0;
#ifdef SDIO_HQA
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, &offset);

	set_fs(oldfs);
#endif

	return ret;
}
#endif

int sdio_autok_res_exist(struct msdc_host *host)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	struct file *filp = NULL;
	int i;

	for (i = 0; i < AUTOK_VCORE_NUM; i++) {
		filp = msdc_file_open(sdio_autok_res_path[i], O_RDONLY, 0644);
		if (filp == NULL) {
			pr_err("autok result not exist\n");
			return 0;
		}
		filp_close(filp, NULL);
	}
#endif
	return 1;
}

int sdio_autok_res_apply(struct msdc_host *host, int vcore)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	struct file *filp = NULL;
	size_t size;
	u8 *res;
	int ret = -1;
	int i;

	if (vcore < AUTOK_VCORE_LEVEL0 ||  vcore >= AUTOK_VCORE_NUM)
		vcore = AUTOK_VCORE_LEVEL0;

	res = host->autok_res[vcore];

	filp = msdc_file_open(sdio_autok_res_path[vcore], O_RDONLY, 0644);
	if (filp == NULL) {
		pr_err("autok result open fail\n");
		return ret;
	}

	size = msdc_file_read(filp, 0, res, TUNING_PARA_SCAN_COUNT);
	if (size == TUNING_PARA_SCAN_COUNT) {
		autok_tuning_parameter_init(host, res);

		for (i = 1; i < TUNING_PARA_SCAN_COUNT; i++)
			pr_err("autok result exist!, result[%d] = %d\n", i, res[i]);
		ret = 0;
	}

	filp_close(filp, NULL);

	return ret;
#endif
	return 0;
}

int sdio_autok_res_save(struct msdc_host *host, int vcore, u8 *res)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	struct file *filp = NULL;
	size_t size;
	int ret = -1;

	if (res == NULL)
		return ret;

	if (vcore < AUTOK_VCORE_LEVEL0 ||  vcore >= AUTOK_VCORE_NUM)
		vcore = AUTOK_VCORE_LEVEL0;

	filp = msdc_file_open(sdio_autok_res_path[vcore], O_CREAT | O_WRONLY, 0644);
	if (filp == NULL) {
		pr_err("autok result open fail\n");
		return ret;
	}

	size = msdc_file_write(filp, 0, res, TUNING_PARA_SCAN_COUNT);
	if (size == TUNING_PARA_SCAN_COUNT)
		ret = 0;
	vfs_fsync(filp, 0);

	filp_close(filp, NULL);

	return ret;
#endif
	return 0;
}

int autok_res_check(u8 *res_h, u8 *res_l)
{
	int ret = 0;
	int i;

	for (i = 0; i < TUNING_PARAM_COUNT; i++) {
		if ((i == CMD_RD_D_DLY1) || (i == DAT_RD_D_DLY1)) {
			if ((res_h[i] > res_l[i]) && (res_h[i] - res_l[i] > SDIO_AUTOK_DIFF_MARGIN))
				ret = -1;
			if ((res_l[i] > res_h[i]) && (res_l[i] - res_h[i] > SDIO_AUTOK_DIFF_MARGIN))
				ret = -1;
		} else if ((i == CMD_RD_D_DLY1_SEL) || (i == DAT_RD_D_DLY1_SEL)) {
			/* this is cover by previous check,
			 * just by pass if 0 and 1 in cmd/dat delay
			 */
		} else {
			if (res_h[i] != res_l[i])
				ret = -1;
		}
	}

#ifndef SDIO_HW_DVFS_CONDITIONAL
	ret = -1;
#endif
	pr_err("autok_res_check %d!\n", ret);

	return ret;
}
#ifdef CONFIG_MTK_SDIO_SUPPORT
static u32 sdio_reg_backup[AUTOK_VCORE_NUM * BACKUP_REG_COUNT_SDIO];
#endif

#ifdef CONFIG_MTK_SDIO_SUPPORT
u16 sdio_reg_backup_offsets_src[] = {
	OFFSET_MSDC_IOCON,
	OFFSET_MSDC_PATCH_BIT0,
	OFFSET_MSDC_PATCH_BIT1,
	OFFSET_MSDC_PATCH_BIT2,
	OFFSET_MSDC_PAD_TUNE0,
	OFFSET_MSDC_PAD_TUNE1,
	OFFSET_EMMC50_PAD_DS_TUNE,
	OFFSET_EMMC50_PAD_CMD_TUNE,
	OFFSET_EMMC50_PAD_DAT01_TUNE,
	OFFSET_EMMC50_PAD_DAT23_TUNE,
	OFFSET_EMMC50_PAD_DAT45_TUNE,
	OFFSET_EMMC50_PAD_DAT67_TUNE,
	OFFSET_EMMC50_CFG0,
	OFFSET_EMMC50_CFG1
};

u16 sdio_dvfs_reg_backup_offsets[] = {
	OFFSET_MSDC_IOCON_1,
	OFFSET_MSDC_PATCH_BIT0_1,
	OFFSET_MSDC_PATCH_BIT1_1,
	OFFSET_MSDC_PATCH_BIT2_1,
	OFFSET_MSDC_PAD_TUNE0_1,
	OFFSET_MSDC_PAD_TUNE1_1,
	OFFSET_EMMC50_PAD_DS_TUNE_1,
	OFFSET_EMMC50_PAD_CMD_TUNE_1,
	OFFSET_EMMC50_PAD_DAT01_TUNE_1,
	OFFSET_EMMC50_PAD_DAT23_TUNE_1,
	OFFSET_EMMC50_PAD_DAT45_TUNE_1,
	OFFSET_EMMC50_PAD_DAT67_TUNE_1,
	OFFSET_EMMC50_CFG0_1,
	OFFSET_EMMC50_CFG1_1
};
#endif

void sdio_dvfs_reg_restore(struct msdc_host *host)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	void __iomem *base = host->base;

	/* High Vcore */
	MSDC_WRITE32(MSDC_IOCON_1,      sdio_reg_backup[0][0]);
	MSDC_WRITE32(MSDC_PATCH_BIT0_1, sdio_reg_backup[0][1]);
	MSDC_WRITE32(MSDC_PATCH_BIT1_1, sdio_reg_backup[0][2]);
	MSDC_WRITE32(MSDC_PATCH_BIT2_1, sdio_reg_backup[0][3]);
	MSDC_WRITE32(MSDC_PAD_TUNE0_1,  sdio_reg_backup[0][4]);
	MSDC_WRITE32(MSDC_PAD_TUNE1_1,  sdio_reg_backup[0][5]);
	MSDC_WRITE32(MSDC_DAT_RDDLY0_1, sdio_reg_backup[0][6]);
	MSDC_WRITE32(MSDC_DAT_RDDLY1_1, sdio_reg_backup[0][7]);
	MSDC_WRITE32(MSDC_DAT_RDDLY2_1, sdio_reg_backup[0][8]);
	MSDC_WRITE32(MSDC_DAT_RDDLY3_1, sdio_reg_backup[0][9]);

	/* Low Vcore */
	MSDC_WRITE32(MSDC_IOCON_2,      sdio_reg_backup[1][0]);
	MSDC_WRITE32(MSDC_PATCH_BIT0_2, sdio_reg_backup[1][1]);
	MSDC_WRITE32(MSDC_PATCH_BIT1_2, sdio_reg_backup[1][2]);
	MSDC_WRITE32(MSDC_PATCH_BIT2_2, sdio_reg_backup[1][3]);
	MSDC_WRITE32(MSDC_PAD_TUNE0_2,  sdio_reg_backup[1][4]);
	MSDC_WRITE32(MSDC_PAD_TUNE1_2,  sdio_reg_backup[1][5]);
	MSDC_WRITE32(MSDC_DAT_RDDLY0_2, sdio_reg_backup[1][6]);
	MSDC_WRITE32(MSDC_DAT_RDDLY1_2, sdio_reg_backup[1][7]);
	MSDC_WRITE32(MSDC_DAT_RDDLY2_2, sdio_reg_backup[1][8]);
	MSDC_WRITE32(MSDC_DAT_RDDLY3_2, sdio_reg_backup[1][9]);

	/* Enable HW DVFS */
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_DVFS_EN, 1);
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_DVFS_HW, 1);
#endif
}

#ifdef CONFIG_MTK_SDIO_SUPPORT
static void sdio_dvfs_reg_backup(struct msdc_host *host)
{
	void __iomem *base = host->base;

	/* High Vcore */
	sdio_reg_backup[0][0] = MSDC_READ32(MSDC_IOCON_1);
	sdio_reg_backup[0][1] = MSDC_READ32(MSDC_PATCH_BIT0_1);
	sdio_reg_backup[0][2] = MSDC_READ32(MSDC_PATCH_BIT1_1);
	sdio_reg_backup[0][3] = MSDC_READ32(MSDC_PATCH_BIT2_1);
	sdio_reg_backup[0][4] = MSDC_READ32(MSDC_PAD_TUNE0_1);
	sdio_reg_backup[0][5] = MSDC_READ32(MSDC_PAD_TUNE1_1);
	sdio_reg_backup[0][6] = MSDC_READ32(MSDC_DAT_RDDLY0_1);
	sdio_reg_backup[0][7] = MSDC_READ32(MSDC_DAT_RDDLY1_1);
	sdio_reg_backup[0][8] = MSDC_READ32(MSDC_DAT_RDDLY2_1);
	sdio_reg_backup[0][9] = MSDC_READ32(MSDC_DAT_RDDLY3_1);

	/* Low Vcore */
	sdio_reg_backup[1][0] = MSDC_READ32(MSDC_IOCON_2);
	sdio_reg_backup[1][1] = MSDC_READ32(MSDC_PATCH_BIT0_2);
	sdio_reg_backup[1][2] = MSDC_READ32(MSDC_PATCH_BIT1_2);
	sdio_reg_backup[1][3] = MSDC_READ32(MSDC_PATCH_BIT2_2);
	sdio_reg_backup[1][4] = MSDC_READ32(MSDC_PAD_TUNE0_2);
	sdio_reg_backup[1][5] = MSDC_READ32(MSDC_PAD_TUNE1_2);
	sdio_reg_backup[1][6] = MSDC_READ32(MSDC_DAT_RDDLY0_2);
	sdio_reg_backup[1][7] = MSDC_READ32(MSDC_DAT_RDDLY1_2);
	sdio_reg_backup[1][8] = MSDC_READ32(MSDC_DAT_RDDLY2_2);
	sdio_reg_backup[1][9] = MSDC_READ32(MSDC_DAT_RDDLY3_2);
}
#endif

void sdio_set_hw_dvfs(int vcore, int done, struct msdc_host *host)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	void __iomem *base = host->base;

	if (vcore >= AUTOK_VCORE_LEVEL1) {
		MSDC_WRITE32(MSDC_IOCON_1,      MSDC_READ32(MSDC_IOCON));
		MSDC_WRITE32(MSDC_PATCH_BIT0_1, MSDC_READ32(MSDC_PATCH_BIT0));
		MSDC_WRITE32(MSDC_PATCH_BIT1_1, MSDC_READ32(MSDC_PATCH_BIT1));
		MSDC_WRITE32(MSDC_PATCH_BIT2_1, MSDC_READ32(MSDC_PATCH_BIT2));
		MSDC_WRITE32(MSDC_PAD_TUNE0_1,  MSDC_READ32(MSDC_PAD_TUNE0));
		MSDC_WRITE32(MSDC_PAD_TUNE1_1,  MSDC_READ32(MSDC_PAD_TUNE1));
		MSDC_WRITE32(MSDC_DAT_RDDLY0_1, MSDC_READ32(MSDC_DAT_RDDLY0));
		MSDC_WRITE32(MSDC_DAT_RDDLY1_1, MSDC_READ32(MSDC_DAT_RDDLY1));
		MSDC_WRITE32(MSDC_DAT_RDDLY2_1, MSDC_READ32(MSDC_DAT_RDDLY2));
		MSDC_WRITE32(MSDC_DAT_RDDLY3_1, MSDC_READ32(MSDC_DAT_RDDLY3));
	} else {
		MSDC_WRITE32(MSDC_IOCON_2,      MSDC_READ32(MSDC_IOCON));
		MSDC_WRITE32(MSDC_PATCH_BIT0_2, MSDC_READ32(MSDC_PATCH_BIT0));
		MSDC_WRITE32(MSDC_PATCH_BIT1_2, MSDC_READ32(MSDC_PATCH_BIT1));
		MSDC_WRITE32(MSDC_PATCH_BIT2_2, MSDC_READ32(MSDC_PATCH_BIT2));
		MSDC_WRITE32(MSDC_PAD_TUNE0_2,  MSDC_READ32(MSDC_PAD_TUNE0));
		MSDC_WRITE32(MSDC_PAD_TUNE1_2,  MSDC_READ32(MSDC_PAD_TUNE1));
		MSDC_WRITE32(MSDC_DAT_RDDLY0_2, MSDC_READ32(MSDC_DAT_RDDLY0));
		MSDC_WRITE32(MSDC_DAT_RDDLY1_2, MSDC_READ32(MSDC_DAT_RDDLY1));
		MSDC_WRITE32(MSDC_DAT_RDDLY2_2, MSDC_READ32(MSDC_DAT_RDDLY2));
		MSDC_WRITE32(MSDC_DAT_RDDLY3_2, MSDC_READ32(MSDC_DAT_RDDLY3));
	}

	if (done) {
		/* Enable HW DVFS */
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_DVFS_EN, 1);
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_DVFS_HW, 1);

		/* Backup the register, restore when resume */
		sdio_dvfs_reg_backup(host);
	}
#endif
}

/* For backward compatible, remove later */
int wait_sdio_autok_ready(void *data)
{
	return 0;
}
EXPORT_SYMBOL(wait_sdio_autok_ready);

void sdio_autok_wait_dvfs_ready(void)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	int dvfs;

	dvfs = is_vcorefs_can_work();

	/* DVFS not ready, just wait */
	while (dvfs == 0) {
		pr_err("DVFS not ready\n");
		msleep(100);
		dvfs = is_vcorefs_can_work();
	}

	if (dvfs == -1)
		pr_err("DVFS feature not enable\n");

	if (dvfs == 1)
		pr_err("DVFS ready\n");
#endif
}

int sd_execute_dvfs_autok(struct msdc_host *host, u32 opcode)
{
	int ret = 0;
	int vcore, vcore_dvfs_work;
	u8 *res;

	vcore_dvfs_work = is_vcorefs_can_work();
	if (vcore_dvfs_work == -1) {
		vcore = 0;
		pr_err("DVFS feature not enabled\n");
	} else if (vcore_dvfs_work == 0) {
		vcore = vcorefs_get_hw_opp();
		pr_err("DVFS not ready\n");
	} else if (vcore_dvfs_work == 1) {
		vcore = vcorefs_get_hw_opp();
		pr_err("DVFS ready\n");
	} else {
		vcore = 0;
		pr_err("Invalid return value from is_vcorefs_can_work()\n");
	}

	res = host->autok_res[vcore];
	pr_err("sd_execute_dvfs_autok %d\n", vcore);

	if (host->mmc->ios.timing == MMC_TIMING_UHS_SDR104 ||
	    host->mmc->ios.timing == MMC_TIMING_UHS_SDR50) {
		if (host->is_autok_done == 0) {
			pr_err("[AUTOK]SDcard autok\n");
			ret = autok_execute_tuning(host, res);
			host->is_autok_done = 1;
		} else {
			autok_init_sdr104(host);
			autok_tuning_parameter_init(host, res);
		}
	}

	return ret;
}


int emmc_execute_dvfs_autok(struct msdc_host *host, u32 opcode)
{
	int ret = 0;
	int vcore, vcore_dvfs_work;
	u8 *res;

	vcore_dvfs_work = is_vcorefs_can_work();
	if (vcore_dvfs_work == -1) {
		vcore = 0;
		pr_err("DVFS feature not enabled\n");
	} else if (vcore_dvfs_work == 0) {
		vcore = vcorefs_get_hw_opp();
		pr_err("DVFS not ready\n");
	} else if (vcore_dvfs_work == 1) {
		vcore = vcorefs_get_hw_opp();
		pr_err("DVFS ready\n");
	} else {
		vcore = 0;
		pr_err("Invalid return value from is_vcorefs_can_work()\n");
	}

	res = host->autok_res[vcore];
	pr_err("emmc_execute_dvfs_autok %d\n", vcore);

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS200) {
#ifdef MSDC_HQA
		msdc_HQA_set_voltage(host);
#endif

		if (opcode == MMC_SEND_STATUS) {
			pr_err("[AUTOK]eMMC HS200 Tune CMD only\n");
			ret = hs200_execute_tuning_cmd(host, res);
		} else {
			pr_err("[AUTOK]eMMC HS200 Tune\n");
			ret = hs200_execute_tuning(host, res);
		}
	} else if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400) {
#ifdef MSDC_HQA
		msdc_HQA_set_voltage(host);
#endif

		if (opcode == MMC_SEND_STATUS) {
			pr_err("[AUTOK]eMMC HS400 Tune CMD only\n");
			ret = hs400_execute_tuning_cmd(host, res);
		} else {
			pr_err("[AUTOK]eMMC HS400 Tune\n");
			ret = hs400_execute_tuning(host, res);
		}
	}

	return ret;
}

void sdio_execute_dvfs_autok_mode(struct msdc_host *host, bool ddr208)
{
#ifndef FPGA_PLATFORM
#ifdef CONFIG_MTK_SDIO_SUPPORT
	void __iomem *base = host->base;
	int sdio_res_exist = 0;
	int vcore;
	int i;
	int (*autok_init)(struct msdc_host *host);
	int (*autok_execute)(struct msdc_host *host, u8 *res);

	if (ddr208) {
		autok_init = autok_init_ddr208;
		autok_execute = autok_sdio30_plus_tuning;
		pr_err("[AUTOK]SDIO DDR208 Tune\n");
	} else {
		autok_init = autok_init_sdr104;
		autok_execute = autok_execute_tuning;
		pr_err("[AUTOK]SDIO SDR104 Tune\n");
	}

	if (host->is_autok_done) {
		autok_init(host);

		/* Check which vcore setting to apply */
		vcore = vcorefs_get_hw_opp();
		pr_err("[AUTOK]Apply first tune para vcore = %d\n", vcore);
		autok_tuning_parameter_init(host, host->autok_res[vcore]);

		if (host->use_hw_dvfs == 0) {
			pr_err("[AUTOK]No need change para when dvfs\n");
		} else {
			pr_err("[AUTOK]Need change para when dvfs\n");

			/* Use HW DVFS */
			msdc_dvfs_reg_restore(host);
		}

		return;
	}
	/* HQA need read autok setting from file */
	sdio_res_exist = sdio_autok_res_exist(host);

	host->dvfs_reg_backup = sdio_reg_backup;
	host->dvfs_reg_offsets = sdio_dvfs_reg_backup_offsets;
	host->dvfs_reg_offsets_src = sdio_reg_backup_offsets_src;
	host->dvfs_reg_backup_cnt = BACKUP_REG_COUNT_SDIO;

	/* Wait DFVS ready for excute autok here */
	sdio_autok_wait_dvfs_ready();

	for (i = 0; i < AUTOK_VCORE_NUM; i++) {
		if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, i) != 0)
			pr_err("vcorefs_request_dvfs_opp@LEVEL%d fail!\n", i);

		if (sdio_res_exist)
			sdio_autok_res_apply(host, i);
		else
			autok_execute(host, host->autok_res[i]);

		msdc_set_hw_dvfs(i, host);
	}

	/* Backup the register, restore when resume */
	msdc_dvfs_reg_backup(host);

	if (autok_res_check(host->autok_res[AUTOK_VCORE_LEVEL3],
			host->autok_res[AUTOK_VCORE_LEVEL0]) == 0) {
		pr_err("[AUTOK]No need change para when dvfs\n");
	} else {
		pr_err("[AUTOK]Need change para when dvfs\n");

		/* Use HW DVFS */
		host->use_hw_dvfs = 1;
		host->dvfs_id = KIR_AUTOK_SDIO;

		/* Enable HW DVFS, but setting used now is at register offset <=0x104.
		 * Setting at register offset >=0x300 will effect after SPM handshakes
		 * with MSDC.
		 */
		MSDC_WRITE32(MSDC_CFG,
			MSDC_READ32(MSDC_CFG) | (MSDC_CFG_DVFS_EN | MSDC_CFG_DVFS_HW));
	}

	/* Un-request, return 0 pass */
	if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPP_UNREQ) != 0)
		pr_err("vcorefs_request_dvfs_opp@OPP_UNREQ fail!\n");

	/* Tell DVFS can start now because AUTOK done */
	spm_msdc_dvfs_setting(host->dvfs_id, 1);

	host->is_autok_done = 1;
	complete(&host->autok_done);
#endif
#endif
}

#ifdef CONFIG_MTK_SDIO_SUPPORT
static int msdc_io_rw_direct_host(struct mmc_host *host, int write, unsigned fn,
	unsigned addr, u8 in, u8 *out)
{
	struct mmc_command cmd = {0};
	int err;

	if (!host || fn > 7)
		return -EINVAL;

	/* sanity check */
	if (addr & ~0x1FFFF)
		return -EINVAL;

	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.arg = write ? 0x80000000 : 0x00000000;
	cmd.arg |= fn << 28;
	cmd.arg |= (write && out) ? 0x08000000 : 0x00000000;
	cmd.arg |= addr << 9;
	cmd.arg |= in;
	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	if (mmc_host_is_spi(host)) {
		/* host driver already reported errors */
	} else {
		if (cmd.resp[0] & R5_ERROR)
			return -EIO;
		if (cmd.resp[0] & R5_FUNCTION_NUMBER)
			return -EINVAL;
		if (cmd.resp[0] & R5_OUT_OF_RANGE)
			return -ERANGE;
	}

	if (out) {
		if (mmc_host_is_spi(host))
			*out = (cmd.resp[0] >> 8) & 0xFF;
		else
			*out = cmd.resp[0] & 0xFF;
	}

	return 0;
}
#endif

/* #define DEVICE_RX_READ_DEBUG */
void sdio_plus_set_device_rx(struct msdc_host *host)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	struct mmc_host *mmc = host->mmc;
	void __iomem *base = host->base;
	u32 msdc_cfg;
	int retry = 3, cnt = 1000;
#ifdef DEVICE_RX_READ_DEBUG
	unsigned char data;
#endif
	int ret = 0;

	msdc_cfg = MSDC_READ32(MSDC_CFG);
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD_HS400, 0);
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, 0);
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKDIV, 5);
	msdc_retry(!(MSDC_READ32(MSDC_CFG) & MSDC_CFG_CKSTB), retry, cnt, host->id);

#ifdef DEVICE_RX_READ_DEBUG
	pr_err("%s +++++++++++++++++++++++++=\n", __func__);
	ret = msdc_io_rw_direct_host(mmc, 0, 0, 0x2, 0, &data);
	pr_err("0x2 data: %x , ret: %x\n", data, ret);
#endif
	ret = msdc_io_rw_direct_host(mmc, 1, 0, 0x2, 0x2, 0);

#ifdef DEVICE_RX_READ_DEBUG
	ret = msdc_io_rw_direct_host(mmc, 0, 0, 0x2, 0, &data);
	pr_err("0x2 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x11C, 0, &data);
	pr_err("0x11C data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x124, 0, &data);
	pr_err("0x124 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x125, 0, &data);
	pr_err("0x125 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x126, 0, &data);
	pr_err("0x126 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x127, 0, &data);
	pr_err("0x127 data: %x , ret: %x\n", data, ret);
#endif

	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x11C, 0x90, 0);

#if 0 /* Device data RX window cover by host data TX */
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x124, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x125, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x126, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x127, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x128, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x129, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x12A, 0x87, 0);
	ret = msdc_io_rw_direct_host(mmc, 1, 1, 0x12B, 0x87, 0);
#endif

#ifdef DEVICE_RX_READ_DEBUG
	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x11C, 0, &data);
	pr_err("0x11C data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x124, 0, &data);
	pr_err("0x124 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x125, 0, &data);
	pr_err("0x125 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x126, 0, &data);
	pr_err("0x126 data: %x , ret: %x\n", data, ret);

	ret = msdc_io_rw_direct_host(mmc, 0, 1, 0x127, 0, &data);
	pr_err("0x127 data: %x , ret: %x\n", data, ret);
#endif

	MSDC_WRITE32(MSDC_CFG, msdc_cfg);
	msdc_retry(!(MSDC_READ32(MSDC_CFG) & MSDC_CFG_CKSTB), retry, cnt, host->id);
#endif
}

#define SDIO_CCCR_MTK_DDR208       0xF2
#define SDIO_MTK_DDR208            0x3
#define SDIO_MTK_DDR208_SUPPORT    0x2
int sdio_plus_set_device_ddr208(struct msdc_host *host)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	struct mmc_host *mmc = host->mmc;
	static u8 autok_res104[TUNING_PARA_SCAN_COUNT];
	unsigned char data;
	int err = 0;

	if (host->is_autok_done) {
		autok_init_sdr104(host);
		autok_tuning_parameter_init(host, autok_res104);
	} else {
		autok_execute_tuning(host, autok_res104);
	}

	/* Read SDIO Device CCCR[0x00F2]
	 * Bit[1] Always 1, Support DDR208 Mode.
	 * Bit[0]
	 *        1:Enable DDR208.
	 *        0:Disable DDR208.
	 */
	err = msdc_io_rw_direct_host(mmc, 0, 0, SDIO_CCCR_MTK_DDR208, 0, &data);

	/* Re-autok sdr104 if default setting fail */
	if (err) {
		autok_execute_tuning(host, autok_res104);
		err = msdc_io_rw_direct_host(mmc, 0, 0, SDIO_CCCR_MTK_DDR208, 0, &data);
	}

	if (err) {
		pr_err("Read SDIO_CCCR_MTK_DDR208 fail\n");
		goto end;
	}
	if ((data & SDIO_MTK_DDR208_SUPPORT) == 0) {
		pr_err("Device not support SDIO_MTK_DDR208\n");
		err = -1;
		goto end;
	}

	/* Switch to DDR208 Flow :
	 * 1. First switch to DDR50 mode;
	 * 2. Then Host CMD52 Write 0x03/0x01 to CCCR[0x00F2]
	 */
	err = msdc_io_rw_direct_host(mmc, 0, 0, SDIO_CCCR_SPEED, 0, &data);
	if (err) {
		pr_err("Read SDIO_CCCR_SPEED fail\n");
		goto end;
	}

	data = (data & (~SDIO_SPEED_BSS_MASK)) | SDIO_SPEED_DDR50;
	err = msdc_io_rw_direct_host(mmc, 1, 0, SDIO_CCCR_SPEED, data, NULL);
	if (err) {
		pr_err("Set SDIO_CCCR_SPEED to DDR fail\n");
		goto end;
	}

	err = msdc_io_rw_direct_host(mmc, 1, 0, SDIO_CCCR_MTK_DDR208,
		SDIO_MTK_DDR208, NULL);
	if (err) {
		pr_err("Set SDIO_MTK_DDR208 fail\n");
		goto end;
	}

end:

	return err;
#endif
	return 0;
}

void sdio_execute_dvfs_autok(struct msdc_host *host)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	/* Set device timming for latch data */
	sdio_plus_set_device_rx(host);

	/* Not support DDR208 and only Autok SDR104 */
	if (!(host->hw->flags & MSDC_SDIO_DDR208)) {
		sdio_execute_dvfs_autok_mode(host, 0);
		return;
	}

	if (sdio_plus_set_device_ddr208(host))
		return;

	/* Set HS400 clock mode and DIV = 0 */
	msdc_clk_stable(host, 3, 0, 1);

	/* Find DDR208 timing */
	sdio_execute_dvfs_autok_mode(host, 1);
#endif
}

int emmc_autok(void)
{
	struct msdc_host *host = mtk_msdc_host[0];

	if (!host || !host->mmc) {
		pr_err("emmc card not ready\n");
		return -1;
	}

	pr_err("emmc autok\n");

	return 0;
}
EXPORT_SYMBOL(emmc_autok);

/* FIX ME: Since card can be insert at any time but this is invoked only once
 * when DVFS ready, this function is not suitable for card insert after boot
 */
int sd_autok(void)
{
	struct msdc_host *host = mtk_msdc_host[1];

	if (!host || !host->mmc) {
		pr_err("SD card not ready\n");
		return -1;
	}

	pr_err("sd autok\n");

	return 0;
}
EXPORT_SYMBOL(sd_autok);

int sdio_autok(void)
{
#ifdef CONFIG_MTK_SDIO_SUPPORT
	struct msdc_host *host = mtk_msdc_host[2];
	int timeout = 0;

	while (!host || !host->hw) {
		pr_err("SDIO host not ready\n");
		msleep(1000);
		timeout++;
		if (timeout == 20) {
			pr_err("SDIO host not exist\n");
			return -1;
		}
	}

	if (host->hw->host_function != MSDC_SDIO) {
		pr_err("SDIO device not in this host\n");
		return -1;
	}

	pr_err("sdio autok\n");

	/* Device never ready in moudle_init.
	 * Call spm_msdc_dvfs_setting if device autok done.
	 */
#if 0
	if (!wait_for_completion_timeout(&host->autok_done, 30 * HZ)) {
		pr_err("SDIO wait device autok ready timeout");
		return -1;
	}

	pr_err("sdio autok done!");
#endif
#endif
	return 0;
}
EXPORT_SYMBOL(sdio_autok);

void msdc_dump_autok(struct msdc_host *host)
{
	int i, j;
	int bit_pos, byte_pos, start;
	char buf[65];

	pr_err("[AUTOK]VER : 0x%02x%02x%02x%02x\r\n",
		host->autok_res[0][AUTOK_VER3],
		host->autok_res[0][AUTOK_VER2],
		host->autok_res[0][AUTOK_VER1],
		host->autok_res[0][AUTOK_VER0]);

	for (i = AUTOK_VCORE_LEVEL1; i >= AUTOK_VCORE_LEVEL0; i--) {
		start = CMD_SCAN_R0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]CMD Rising \t: %s\r\n", buf);

		start = CMD_SCAN_F0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]CMD Falling \t: %s\r\n", buf);

		start = DAT_SCAN_R0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]DAT Rising \t: %s\r\n", buf);

		start = DAT_SCAN_F0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]DAT Falling \t: %s\r\n", buf);
		/* cmd response use ds pin, but window is different with data pin, because cmd response is SDR. */
		start = DS_CMD_SCAN_0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]DS CMD Window \t: %s\r\n", buf);

		start = DS_DAT_SCAN_0;
		for (j = 0; j < 64; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]DS DAT Window \t: %s\r\n", buf);

		start = D_DATA_SCAN_0;
		for (j = 0; j < 32; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]Device Data RX \t: %s\r\n", buf);

		start = H_DATA_SCAN_0;
		for (j = 0; j < 32; j++) {
			bit_pos = j % 8;
			byte_pos = j / 8 + start;
			if (host->autok_res[i][byte_pos] & (1 << bit_pos))
				buf[j] = 'X';
			else
				buf[j] = 'O';
		}
		buf[j] = '\0';
		pr_err("[AUTOK]Host   Data TX \t: %s\r\n", buf);

		pr_err("[AUTOK]CMD [EDGE:%d CMD_FIFO_EDGE:%d DLY1:%d DLY2:%d]\r\n",
			host->autok_res[i][0], host->autok_res[i][1], host->autok_res[i][5], host->autok_res[i][7]);
		pr_err("[AUTOK]DAT [RDAT_EDGE:%d RD_FIFO_EDGE:%d WD_FIFO_EDGE:%d]\r\n",
			host->autok_res[i][2], host->autok_res[i][3], host->autok_res[i][4]);
		pr_err("[AUTOK]DAT [LATCH_CK:%d DLY1:%d DLY2:%d]\r\n",
			host->autok_res[i][13], host->autok_res[i][9], host->autok_res[i][11]);
		pr_err("[AUTOK]DS  [DLY1:%d DLY2:%d DLY3:%d]\r\n",
			host->autok_res[i][14], host->autok_res[i][16], host->autok_res[i][18]);
		pr_err("[AUTOK]DAT [TX SEL:%d]\r\n", host->autok_res[i][20]);
	}
}
