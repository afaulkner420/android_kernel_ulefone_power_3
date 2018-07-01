/**
 * The platform spi device for sunwave's fingerprint sensor.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson Leung <mailto:liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>

#include <linux/clk.h>
#include <linux/spi/spi.h>

//#include <mtk_spi.h>
#define SF_SPI_BUS_NUM 0 // TODO:
#define SF_SPI_CS__NUM 0 // TODO:

//#include <mach/mtk_clkmgr.h>
#include "mtk_spi_hal.h"
#define MODULE_NAME "sf_spi"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)

// TODO: register REE spi device spec.
/*
static struct mt_chip_conf mt_spi_chip_conf = {
    .setuptime    = 20,
    .holdtime     = 20,
    .high_time    = 25,
    .low_time     = 25,
    .cs_idletime  = 10,
    .ulthgh_thrsh =  0,
    .sample_sel   = POSEDGE,
    .cs_pol       = ACTIVE_LOW,
    .cpol         = SPI_CPOL_0,
    .cpha         = SPI_CPHA_0,
    .tx_mlsb      = SPI_MSB,
    .rx_mlsb      = SPI_MSB,
    .tx_endian    = SPI_LENDIAN,
    .rx_endian    = SPI_LENDIAN,
    .com_mod      = DMA_TRANSFER, // FIFO_TRANSFER/DMA_TRANSFER
    .pause        = PAUSE_MODE_ENABLE,
    .finish_intr  = FINISH_INTR_EN,
    .deassert     = DEASSERT_DISABLE,
    .ulthigh      = ULTRA_HIGH_DISABLE,
    .tckdly       = TICK_DLY0,
};
*/
/*
 * REE using the standard 'spidev' driver.
 * CONFIG_SPI_SPIDEV must be set in REE.
 */
/*
static struct spi_board_info spi_desc __initdata = {
    .modalias        = "spidev",
    .bus_num         = SF_SPI_BUS_NUM,
    .chip_select     = SF_SPI_CS__NUM,
    .mode            = SPI_MODE_0,
    .controller_data = &mt_spi_chip_conf,
};
*/

////////////////////////////////////////////////////////////////////////////////
#if 0
static struct spi_master *spi_ctl = NULL;
static struct spi_device *spi_dev = NULL;

int sf_spi_clock_enable(bool on)
{
    int err = 0;
    //struct clk *clk;
    //struct device *dev;

#if 0
    struct mt_spi_t *mt_spi = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    mt_spi = spi_master_get_devdata(spi_ctl);

    if (!mt_spi) {
        xprintk(KERN_ERR, "fail to get mediatek spi device.\n");
        dump_stack();
        return (-ENODEV);
    }
    dev = &mt_spi->pdev->dev;
#endif
    if (on) {
        xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

        //clk = devm_clk_get(dev, NULL);

        //mt_spi_enable_clk(mt_spi);
        //clk_enable(clk);
        clkmux_sel(MT_CLKMUX_SPI_GFMUX_SEL, MT_CG_UPLL_D12, "spi");
        enable_clock(MT_CG_SPI_SW_CG, "spi");
        xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    }
    else {
        //mt_spi_disable_clk(mt_spi);
        //clk_disable(clk);
        disable_clock(MT_CG_SPI_SW_CG, "spi");
        clkmux_sel(MT_CLKMUX_SPI_GFMUX_SEL, MT_CG_SYS_26M, "spi");
    }

    return err;
}

int __init sf_spi_platform_init(void)
{
    int err = 0;

    clkmux_sel(MT_CLKMUX_SPI_GFMUX_SEL, MT_CG_UPLL_D12, "spi");
        enable_clock(MT_CG_SPI_SW_CG, "spi");

    spi_ctl = spi_busnum_to_master(SF_SPI_BUS_NUM);

    if (!spi_ctl) {
        xprintk(KERN_ERR, "there is no spi master for bus %d.\n", SF_SPI_BUS_NUM);
        return (-ENODEV);
    }

    sf_spi_clock_enable(true);

    xprintk(KERN_INFO, "sunwave spi board info has been registered.\n");
    return err;
}

void __exit sf_spi_platform_exit(void)
{
    sf_spi_clock_enable(false);

    if (spi_dev) {
        spi_master_put(spi_dev->master);
        spi_unregister_device(spi_dev);
        /* CHECKME: will spi_unregister_device(..) free the memory of 'spi_dev'? */
        /* kfree(spi_dev); */
        spi_dev = NULL;
    }

    if (spi_ctl) {
        spi_master_put(spi_ctl);
        spi_ctl = NULL;
    }

    xprintk(KERN_INFO, "sunwave spi board info released.\n");
}
#endif
