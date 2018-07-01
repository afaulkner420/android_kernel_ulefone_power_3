/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifdef CONFIG_MTK_HW_FDE
/* include api for call security mode */
#include "mtk_secure_api.h"
/* map from AES Spec */
enum {
	MSDC_CRYPTO_XTS_AES         = 4,
	MSDC_CRYPTO_AES_CBC_ESSIV   = 9,
	MSDC_CRYPTO_BITLOCKER       = 17,
	MSDC_CRYPTO_AES_ECB       = 0,
	MSDC_CRYPTO_AES_CBC       = 1,
	MSDC_CRYPTO_AES_CTR       = 2,
	MSDC_CRYPTO_AES_CBC_MAC   = 3,
} aes_mode;

static void msdc_enable_crypto(struct msdc_host *host)
{
	void __iomem *base = host->base;

	/* enable AES path by clear bypass bit */
	MSDC_CLR_BIT32(EMMC52_AES_SWST, EMMC52_AES_BYPASS);
}

static void msdc_disable_crypto(struct msdc_host *host)
{
	void __iomem *base = host->base;

	/* disable AES path by set bypass bit */
	MSDC_SET_BIT32(EMMC52_AES_SWST, EMMC52_AES_BYPASS);
}


static void msdc_crypto_switch_config(struct msdc_host *host, u32 block_address, u32 dir)
{
	void __iomem *base = host->base;
	u32 aes_mode_current = 0, aes_sw_reg = 0;
	u32 ctr[4] = {0};
	unsigned long polling_tmo = 0;

	/* 1. set ctr */
	aes_sw_reg = MSDC_READ32(EMMC52_AES_EN);

	if (aes_sw_reg & EMMC52_AES_SWITCH_VALID0)
		MSDC_GET_FIELD(EMMC52_AES_CFG_GP0, EMMC52_AES_MODE_0, aes_mode_current);
	else if (aes_sw_reg & EMMC52_AES_SWITCH_VALID1)
		MSDC_GET_FIELD(EMMC52_AES_CFG_GP1, EMMC52_AES_MODE_1, aes_mode_current);
	else {
		pr_notice("MSDC: EMMC52_AES_SWITCH_VALID error in msdc reg\n");
		WARN_ON(1);
		return;
	}

	switch (aes_mode_current) {
	case MSDC_CRYPTO_XTS_AES:
	case MSDC_CRYPTO_AES_CBC_ESSIV:
	case MSDC_CRYPTO_BITLOCKER:
		ctr[0] = block_address;
		break;
	case MSDC_CRYPTO_AES_ECB:
	case MSDC_CRYPTO_AES_CBC:
		break;
	case MSDC_CRYPTO_AES_CTR:
		ctr[0] = block_address;
		break;
	case MSDC_CRYPTO_AES_CBC_MAC:
		break;
	default:
		pr_notice("msdc unknown aes mode\n");
		WARN_ON(1);
		return;
	}

	if (aes_sw_reg & EMMC52_AES_SWITCH_VALID0) {
		MSDC_WRITE32(EMMC52_AES_CTR0_GP0, ctr[0]);
		MSDC_WRITE32(EMMC52_AES_CTR1_GP0, ctr[1]);
		MSDC_WRITE32(EMMC52_AES_CTR2_GP0, ctr[2]);
		MSDC_WRITE32(EMMC52_AES_CTR3_GP0, ctr[3]);
	} else {
		MSDC_WRITE32(EMMC52_AES_CTR0_GP1, ctr[0]);
		MSDC_WRITE32(EMMC52_AES_CTR1_GP1, ctr[1]);
		MSDC_WRITE32(EMMC52_AES_CTR2_GP1, ctr[2]);
		MSDC_WRITE32(EMMC52_AES_CTR3_GP1, ctr[3]);
	}

	/* 2. enable AES path */
	msdc_enable_crypto(host);

	/* 3. AES switch start (flush the configure) */
	if (dir == DMA_TO_DEVICE) {
		MSDC_SET_BIT32(EMMC52_AES_SWST, EMMC52_AES_SWITCH_START_ENC);
		polling_tmo = jiffies + POLLING_BUSY;
		while (MSDC_READ32(EMMC52_AES_SWST) & EMMC52_AES_SWITCH_START_ENC) {
			if (time_after(jiffies, polling_tmo)) {
				pr_notice("msdc%d, error: triger AES ENC timeout!\n", host->id);
				WARN_ON(1);
			}
		}
	} else{
		MSDC_SET_BIT32(EMMC52_AES_SWST, EMMC52_AES_SWITCH_START_DEC);
		polling_tmo = jiffies + POLLING_BUSY;
		while (MSDC_READ32(EMMC52_AES_SWST) & EMMC52_AES_SWITCH_START_DEC) {
			if (time_after(jiffies, polling_tmo)) {
				pr_notice("msdc%d, error: triger DEC AES DEC timeout!\n", host->id);
				WARN_ON(1);
			}
		}
	}
}

static void msdc_pre_crypto(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	struct mmc_queue_req *mq_rq;
#endif
	u32 dir = DMA_FROM_DEVICE;
	u32 blk_addr;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	struct mmc_async_req *areq;
	unsigned int task_id;
#endif

	if (host->hw == NULL || mmc->card == NULL)
		return;

	if (host->hw->host_function == MSDC_SDIO || host->hw->host_function == MSDC_SD)
	return;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* CMDQ Command */
	if (check_mmc_cmd4647(cmd->opcode)) {
		task_id = (cmd->arg >> 16) & 0x1f;
		areq = mmc->areq_que[task_id];
		mq_rq = container_of(areq, struct mmc_queue_req, mmc_active);

		if (mq_rq && mq_rq->req->bio && mq_rq->req->bio->bi_hw_fde) {
			if (!host->is_crypto_init
				|| (host->key_idx != mq_rq->req->bio->bi_key_idx)) {
			/* fde init */
			mt_secure_call(MTK_SIP_KERNEL_HW_FDE_MSDC_CTL, (1 << 3), 4, 1);
			host->is_crypto_init = true;
			host->key_idx = mq_rq->req->bio->bi_key_idx;
			}

			blk_addr = mq_rq->brq.que.arg;
			if (!mmc_card_blockaddr(mmc->card))
				blk_addr = blk_addr >> 9;

			/* Check data size with 16bytes */
			WARN_ON(host->dma.xfersz & 0xf);
			/* Check data addressw with 16bytes alignment */
			WARN_ON((host->dma.gpd_addr & 0xf) || (host->dma.bd_addr & 0xf));
			dir = cmd->data->flags & MMC_DATA_READ ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
			msdc_crypto_switch_config(host, blk_addr, dir);
		}
	}
#endif

	/* Normal Read Write Command */
	if (check_mmc_cmd1718(cmd->opcode) || check_mmc_cmd2425(cmd->opcode)) {
		if (mrq->bi_hw_fde) {
			if (!host->is_crypto_init || (host->key_idx != mrq->bi_key_idx)) {
				/* fde init */
				mt_secure_call(MTK_SIP_KERNEL_HW_FDE_MSDC_CTL, (1 << 3), 4, 1);
				host->is_crypto_init = true;
				host->key_idx = mrq->bi_key_idx;
			}

			blk_addr = cmd->arg;
			if (!mmc_card_blockaddr(mmc->card))
				blk_addr = blk_addr >> 9;

			/* Check data size with 16bytes */
			WARN_ON(host->dma.xfersz & 0xf);
			/* Check data addressw with 16bytes alignment */
			WARN_ON((host->dma.gpd_addr & 0xf) || (host->dma.bd_addr & 0xf));
			dir = cmd->data->flags & MMC_DATA_READ ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
			msdc_crypto_switch_config(host, blk_addr, dir);
		}
	}
}

static void msdc_post_crypto(struct msdc_host *host)
{
	/* disable AES path */
	msdc_disable_crypto(host);
}
#endif
