/*
 * Copyright (C) 2016 MediaTek Inc.
 * ShuFanLee <shufan_lee@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <mt-plat/mtk_boot.h>
#ifdef CONFIG_RT_REGMAP
#include <mt-plat/rt-regmap.h>
#endif

#include "mtk_charger_intf.h"
#include "rt9466.h"
#define I2C_ACCESS_MAX_RETRY	5
#define RT9466_DRV_VERSION	"1.0.9_MTK"

/* ======================= */
/* RT9466 Parameter        */
/* ======================= */

static const u32 rt9466_boost_oc_threshold[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000,
}; /* uA */

enum rt9466_irq_idx {
	RT9466_IRQIDX_CHG_STATC = 0,
	RT9466_IRQIDX_CHG_FAULT,
	RT9466_IRQIDX_TS_STATC,
	RT9466_IRQIDX_CHG_IRQ1,
	RT9466_IRQIDX_CHG_IRQ2,
	RT9466_IRQIDX_CHG_IRQ3,
	RT9466_IRQIDX_MAX,
};

static u8 rt9466_irqmask[RT9466_IRQIDX_MAX] = {
	0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF
};

static const u8 rt9466_irq_maskall[RT9466_IRQIDX_MAX] = {
	0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF
};

struct irq_mapping_tbl {
	const char *name;
	const int id;
};

#define RT9466_IRQ_MAPPING(_name, _id) {.name = #_name, .id = _id}
static const struct irq_mapping_tbl rt9466_irq_mapping_tbl[] = {
	RT9466_IRQ_MAPPING(chg_treg, 4),
	RT9466_IRQ_MAPPING(chg_aicr, 5),
	RT9466_IRQ_MAPPING(chg_mivr, 6),
	RT9466_IRQ_MAPPING(pwr_rdy, 7),
	RT9466_IRQ_MAPPING(chg_vsysuv, 12),
	RT9466_IRQ_MAPPING(chg_vsysov, 13),
	RT9466_IRQ_MAPPING(chg_vbatov, 14),
	RT9466_IRQ_MAPPING(chg_vbusov, 15),
	RT9466_IRQ_MAPPING(ts_batcold, 20),
	RT9466_IRQ_MAPPING(ts_batcool, 21),
	RT9466_IRQ_MAPPING(ts_batwarm, 22),
	RT9466_IRQ_MAPPING(ts_bathot, 23),
	RT9466_IRQ_MAPPING(ts_statci, 24),
	RT9466_IRQ_MAPPING(chg_faulti, 25),
	RT9466_IRQ_MAPPING(chg_statci, 26),
	RT9466_IRQ_MAPPING(chg_tmri, 27),
	RT9466_IRQ_MAPPING(chg_batabsi, 28),
	RT9466_IRQ_MAPPING(chg_adpbadi, 29),
	RT9466_IRQ_MAPPING(chg_rvpi, 30),
	RT9466_IRQ_MAPPING(otpi, 31),
	RT9466_IRQ_MAPPING(chg_aiclmeasi, 32),
	RT9466_IRQ_MAPPING(chg_ichgmeasi, 33),
	RT9466_IRQ_MAPPING(wdtmri, 35),
	RT9466_IRQ_MAPPING(ssfinishi, 36),
	RT9466_IRQ_MAPPING(chg_rechgi, 37),
	RT9466_IRQ_MAPPING(chg_termi, 38),
	RT9466_IRQ_MAPPING(chg_ieoci, 39),
	RT9466_IRQ_MAPPING(adc_donei, 40),
	RT9466_IRQ_MAPPING(pumpx_donei, 41),
	RT9466_IRQ_MAPPING(bst_batuvi, 45),
	RT9466_IRQ_MAPPING(bst_midovi, 46),
	RT9466_IRQ_MAPPING(bst_olpi, 47),
};

enum rt9466_charging_status {
	RT9466_CHG_STATUS_READY = 0,
	RT9466_CHG_STATUS_PROGRESS,
	RT9466_CHG_STATUS_DONE,
	RT9466_CHG_STATUS_FAULT,
	RT9466_CHG_STATUS_MAX,
};

/* Charging status name */
static const char *rt9466_chg_status_name[RT9466_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};


static const u8 rt9466_reg_en_hidden_mode[] = {
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
};

static const u8 rt9466_val_en_hidden_mode[] = {
	0x49, 0x32, 0xB6, 0x27, 0x48, 0x18, 0x03, 0xE2,
};

enum rt9466_iin_limit_sel {
	RT9466_IIMLMTSEL_PSEL_OTG,
	RT9466_IINLMTSEL_AICR = 2,
	RT9466_IINLMTSEL_LOWER_LEVEL, /* lower of above two */
};

enum rt9466_adc_sel {
	RT9466_ADC_VBUS_DIV5 = 1,
	RT9466_ADC_VBUS_DIV2,
	RT9466_ADC_VSYS,
	RT9466_ADC_VBAT,
	RT9466_ADC_TS_BAT = 6,
	RT9466_ADC_IBUS = 8,
	RT9466_ADC_IBAT,
	RT9466_ADC_REGN = 11,
	RT9466_ADC_TEMP_JC,
	RT9466_ADC_MAX,
};

/* Unit for each ADC parameter
 * 0 stands for reserved
 * For TS_BAT, the real unit is 0.25.
 * Here we use 25, please remember to divide 100 while showing the value
 */
static const int rt9466_adc_unit[RT9466_ADC_MAX] = {
	0,
	RT9466_ADC_UNIT_VBUS_DIV5,
	RT9466_ADC_UNIT_VBUS_DIV2,
	RT9466_ADC_UNIT_VSYS,
	RT9466_ADC_UNIT_VBAT,
	0,
	RT9466_ADC_UNIT_TS_BAT,
	0,
	RT9466_ADC_UNIT_IBUS,
	RT9466_ADC_UNIT_IBAT,
	0,
	RT9466_ADC_UNIT_REGN,
	RT9466_ADC_UNIT_TEMP_JC,
};

static const int rt9466_adc_offset[RT9466_ADC_MAX] = {
	0,
	RT9466_ADC_OFFSET_VBUS_DIV5,
	RT9466_ADC_OFFSET_VBUS_DIV2,
	RT9466_ADC_OFFSET_VSYS,
	RT9466_ADC_OFFSET_VBAT,
	0,
	RT9466_ADC_OFFSET_TS_BAT,
	0,
	RT9466_ADC_OFFSET_IBUS,
	RT9466_ADC_OFFSET_IBAT,
	0,
	RT9466_ADC_OFFSET_REGN,
	RT9466_ADC_OFFSET_TEMP_JC,
};

struct rt9466_desc {
	u32 ichg;	/* uA */
	u32 aicr;	/* uA */
	u32 mivr;	/* uV */
	u32 cv;		/* uV */
	u32 ieoc;	/* uA */
	u32 safety_timer;	/* hour */
	u32 ircmp_resistor;	/* uohm */
	u32 ircmp_vclamp;	/* uV */
	bool en_te;
	bool en_wdt;
	int regmap_represent_slave_addr;
	const char *regmap_name;
	const char *chg_dev_name;
	bool ceb_invert;
};

/* These default values will be applied if there's no property in dts */
static struct rt9466_desc rt9466_default_desc = {
	.ichg = 2000000,	/* uA */
	.aicr = 500000,		/* uA */
	.mivr = 4400000,	/* uV */
	.cv = 4350000,		/* uA */
	.ieoc = 250000,		/* uA */
	.safety_timer = 12,
#ifdef CONFIG_MTK_BIF_SUPPORT
	.ircmp_resistor = 0,		/* uohm */
	.ircmp_vclamp = 0,		/* uV */
#else
	.ircmp_resistor = 25000,	/* uohm */
	.ircmp_vclamp = 32000,		/* uV */
#endif
	.en_te = true,
	.en_wdt = true,
	.regmap_represent_slave_addr = RT9466_SLAVE_ADDR,
	.regmap_name = "rt9466",
	.chg_dev_name = "primary_chg",
	.ceb_invert = false,
};

struct rt9466_info {
	struct i2c_client *client;
	struct mutex i2c_access_lock;
	struct mutex adc_access_lock;
	struct mutex adcflag_access_lock;
	struct mutex irq_access_lock;
	struct mutex aicr_access_lock;
	struct mutex ichg_access_lock;
	struct mutex pe_access_lock;
	struct mutex hidden_mode_lock;
	struct device *dev;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct rt9466_desc *desc;
	struct work_struct aicl_work;
	struct workqueue_struct *aicl_wq;
	wait_queue_head_t wait_queue;
	int irq;
	int aicr_limit;
	u32 intr_gpio;
	u32 ceb_gpio;
	u8 chip_rev;
	u8 irq_flag[RT9466_IRQIDX_MAX];
	bool adc_wait_evt;
	u32 hidden_mode_cnt;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *regmap_dev;
	struct rt_regmap_properties *regmap_prop;
#endif
};

/* ======================= */
/* Register Address        */
/* ======================= */

static const unsigned char rt9466_reg_addr[] = {
	RT9466_REG_CORE_CTRL0,
	RT9466_REG_CHG_CTRL1,
	RT9466_REG_CHG_CTRL2,
	RT9466_REG_CHG_CTRL3,
	RT9466_REG_CHG_CTRL4,
	RT9466_REG_CHG_CTRL5,
	RT9466_REG_CHG_CTRL6,
	RT9466_REG_CHG_CTRL7,
	RT9466_REG_CHG_CTRL8,
	RT9466_REG_CHG_CTRL9,
	RT9466_REG_CHG_CTRL10,
	RT9466_REG_CHG_CTRL11,
	RT9466_REG_CHG_CTRL12,
	RT9466_REG_CHG_CTRL13,
	RT9466_REG_CHG_CTRL14,
	RT9466_REG_CHG_CTRL15,
	RT9466_REG_CHG_CTRL16,
	RT9466_REG_CHG_ADC,
	RT9466_REG_CHG_CTRL19,
	RT9466_REG_CHG_CTRL17,
	RT9466_REG_CHG_CTRL18,
	RT9466_REG_DEVICE_ID,
	RT9466_REG_CHG_STAT,
	RT9466_REG_CHG_NTC,
	RT9466_REG_ADC_DATA_H,
	RT9466_REG_ADC_DATA_L,
	RT9466_REG_CHG_STATC,
	RT9466_REG_CHG_FAULT,
	RT9466_REG_TS_STATC,
	RT9466_REG_CHG_IRQ1,
	RT9466_REG_CHG_IRQ2,
	RT9466_REG_CHG_IRQ3,
	RT9466_REG_CHG_STATC_CTRL,
	RT9466_REG_CHG_FAULT_CTRL,
	RT9466_REG_TS_STATC_CTRL,
	RT9466_REG_CHG_IRQ1_CTRL,
	RT9466_REG_CHG_IRQ2_CTRL,
	RT9466_REG_CHG_IRQ3_CTRL,
};

/* ========= */
/* RT Regmap */
/* ========= */

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(RT9466_REG_CORE_CTRL0, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL4, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL5, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL8, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL9, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL10, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL11, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL12, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL13, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL14, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL15, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL16, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_ADC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL19, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL17, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL18, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_HIDDEN_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_HIDDEN_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_HIDDEN_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_HIDDEN_CTRL8, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_HIDDEN_CTRL9, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_HIDDEN_CTRL15, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_DEVICE_ID, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_STAT, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_NTC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_ADC_DATA_H, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_ADC_DATA_L, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_STATC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_FAULT, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_TS_STATC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_STATC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_FAULT_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_TS_STATC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ1_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ2_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ3_CTRL, 1, RT_VOLATILE, {});

static rt_register_map_t rt9466_regmap_map[] = {
	RT_REG(RT9466_REG_CORE_CTRL0),
	RT_REG(RT9466_REG_CHG_CTRL1),
	RT_REG(RT9466_REG_CHG_CTRL2),
	RT_REG(RT9466_REG_CHG_CTRL3),
	RT_REG(RT9466_REG_CHG_CTRL4),
	RT_REG(RT9466_REG_CHG_CTRL5),
	RT_REG(RT9466_REG_CHG_CTRL6),
	RT_REG(RT9466_REG_CHG_CTRL7),
	RT_REG(RT9466_REG_CHG_CTRL8),
	RT_REG(RT9466_REG_CHG_CTRL9),
	RT_REG(RT9466_REG_CHG_CTRL10),
	RT_REG(RT9466_REG_CHG_CTRL11),
	RT_REG(RT9466_REG_CHG_CTRL12),
	RT_REG(RT9466_REG_CHG_CTRL13),
	RT_REG(RT9466_REG_CHG_CTRL14),
	RT_REG(RT9466_REG_CHG_CTRL15),
	RT_REG(RT9466_REG_CHG_CTRL16),
	RT_REG(RT9466_REG_CHG_ADC),
	RT_REG(RT9466_REG_CHG_CTRL19),
	RT_REG(RT9466_REG_CHG_CTRL17),
	RT_REG(RT9466_REG_CHG_CTRL18),
	RT_REG(RT9466_REG_CHG_HIDDEN_CTRL2),
	RT_REG(RT9466_REG_CHG_HIDDEN_CTRL6),
	RT_REG(RT9466_REG_CHG_HIDDEN_CTRL7),
	RT_REG(RT9466_REG_CHG_HIDDEN_CTRL8),
	RT_REG(RT9466_REG_CHG_HIDDEN_CTRL9),
	RT_REG(RT9466_REG_CHG_HIDDEN_CTRL15),
	RT_REG(RT9466_REG_DEVICE_ID),
	RT_REG(RT9466_REG_CHG_STAT),
	RT_REG(RT9466_REG_CHG_NTC),
	RT_REG(RT9466_REG_ADC_DATA_H),
	RT_REG(RT9466_REG_ADC_DATA_L),
	RT_REG(RT9466_REG_CHG_STATC),
	RT_REG(RT9466_REG_CHG_FAULT),
	RT_REG(RT9466_REG_TS_STATC),
	RT_REG(RT9466_REG_CHG_IRQ1),
	RT_REG(RT9466_REG_CHG_IRQ2),
	RT_REG(RT9466_REG_CHG_IRQ3),
	RT_REG(RT9466_REG_CHG_STATC_CTRL),
	RT_REG(RT9466_REG_CHG_FAULT_CTRL),
	RT_REG(RT9466_REG_TS_STATC_CTRL),
	RT_REG(RT9466_REG_CHG_IRQ1_CTRL),
	RT_REG(RT9466_REG_CHG_IRQ2_CTRL),
	RT_REG(RT9466_REG_CHG_IRQ3_CTRL),
};
#endif /* CONFIG_RT_REGMAP */

/* ========================= */
/* I2C operations            */
/* ========================= */

static int rt9466_device_read(void *client, u32 addr, int leng, void *dst)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_read_i2c_block_data(i2c, addr, leng, dst);

	return ret;
}

static int rt9466_device_write(void *client, u32 addr, int leng,
	const void *src)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_write_i2c_block_data(i2c, addr, leng, src);

	return ret;
}

#ifdef CONFIG_RT_REGMAP
static struct rt_regmap_fops rt9466_regmap_fops = {
	.read_device = rt9466_device_read,
	.write_device = rt9466_device_write,
};

static int rt9466_register_rt_regmap(struct rt9466_info *info)
{
	int ret = 0;
	struct i2c_client *client = info->client;
	struct rt_regmap_properties *prop = NULL;

	dev_info(info->dev, "%s\n", __func__);

	prop = devm_kzalloc(&client->dev, sizeof(struct rt_regmap_properties),
		GFP_KERNEL);
	if (!prop)
		return -ENOMEM;

	prop->name = info->desc->regmap_name;
	prop->aliases = info->desc->regmap_name;
	prop->register_num = ARRAY_SIZE(rt9466_regmap_map);
	prop->rm = rt9466_regmap_map;
	prop->rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE |
		RT_IO_PASS_THROUGH;
	prop->io_log_en = 0;

	info->regmap_prop = prop;
	info->regmap_dev = rt_regmap_device_register_ex(
		info->regmap_prop,
		&rt9466_regmap_fops,
		&client->dev,
		client,
		info->desc->regmap_represent_slave_addr,
		info
	);

	if (!info->regmap_dev) {
		dev_err(info->dev, "%s: register regmap device failed\n",
			__func__);
		return -EIO;
	}

	return ret;
}
#endif /* CONFIG_RT_REGMAP */

static inline int _rt9466_i2c_write_byte(struct rt9466_info *info, u8 cmd,
	u8 data)
{
	int ret = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_write(info->regmap_dev, cmd, 1, &data);
#else
		ret = rt9466_device_write(info->client, cmd, 1, &data);
#endif
		retry++;
		if (ret < 0)
			mdelay(20);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0)
		dev_err(info->dev, "%s: I2CW[0x%02X] = 0x%02X failed\n",
			__func__, cmd, data);
	else
		dev_dbg(info->dev, "%s: I2CW[0x%02X] = 0x%02X\n", __func__,
			cmd, data);

	return ret;
}

static int rt9466_i2c_write_byte(struct rt9466_info *info, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9466_i2c_write_byte(info, cmd, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int _rt9466_i2c_read_byte(struct rt9466_info *info, u8 cmd)
{
	int ret = 0, ret_val = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_read(info->regmap_dev, cmd, 1, &ret_val);
#else
		ret = rt9466_device_read(info->client, cmd, 1, &ret_val);
#endif
		retry++;
		if (ret < 0)
			msleep(20);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0) {
		dev_err(info->dev, "%s: I2CR[0x%02X] failed\n", __func__, cmd);
		return ret;
	}

	ret_val = ret_val & 0xFF;

	dev_dbg(info->dev, "%s: I2CR[0x%02X] = 0x%02X\n", __func__, cmd,
		ret_val);

	return ret_val;
}

static int rt9466_i2c_read_byte(struct rt9466_info *info, u8 cmd)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9466_i2c_read_byte(info, cmd);
	mutex_unlock(&info->i2c_access_lock);

	if (ret < 0)
		return ret;

	return (ret & 0xFF);
}

static inline int _rt9466_i2c_block_write(struct rt9466_info *info, u8 cmd,
	u32 leng, const u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(info->regmap_dev, cmd, leng, data);
#else
	ret = rt9466_device_write(info->client, cmd, leng, data);
#endif

	return ret;
}


static int rt9466_i2c_block_write(struct rt9466_info *info, u8 cmd, u32 leng,
	const u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9466_i2c_block_write(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int _rt9466_i2c_block_read(struct rt9466_info *info, u8 cmd,
	u32 leng, u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(info->regmap_dev, cmd, leng, data);
#else
	ret = rt9466_device_read(info->client, cmd, leng, data);
#endif

	return ret;
}


static int rt9466_i2c_block_read(struct rt9466_info *info, u8 cmd, u32 leng,
	u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9466_i2c_block_read(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}


static int rt9466_i2c_test_bit(struct rt9466_info *info, u8 cmd, u8 shift,
	bool *is_one)
{
	int ret = 0;
	u8 data = 0;

	ret = rt9466_i2c_read_byte(info, cmd);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data = ret & (1 << shift);
	*is_one = (data == 0 ? false : true);

	return ret;
}

static int rt9466_i2c_update_bits(struct rt9466_info *info, u8 cmd, u8 data,
	u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _rt9466_i2c_read_byte(info, cmd);
	if (ret < 0) {
		mutex_unlock(&info->i2c_access_lock);
		return ret;
	}

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);

	ret = _rt9466_i2c_write_byte(info, cmd, reg_data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int rt9466_set_bit(struct rt9466_info *info, u8 reg, u8 mask)
{
	return rt9466_i2c_update_bits(info, reg, mask, mask);
}

static inline int rt9466_clr_bit(struct rt9466_info *info, u8 reg, u8 mask)
{
	return rt9466_i2c_update_bits(info, reg, 0x00, mask);
}

/* ================== */
/* Internal Functions */
/* ================== */
static int rt9466_get_mivr(struct rt9466_info *info, u32 *mivr);
static int rt9466_get_aicr(struct charger_device *chg_dev, u32 *aicr);
static int rt9466_get_ichg(struct charger_device *chg_dev, u32 *ichg);
static int rt9466_set_aicr(struct charger_device *chg_dev, u32 aicr);
static int rt9466_set_ichg(struct charger_device *chg_dev, u32 aicr);
static int rt9466_kick_wdt(struct charger_device *chg_dev);
static int rt9466_enable_charging(struct charger_device *chg_dev, bool en);

static inline void rt9466_irq_set_flag(struct rt9466_info *info, u8 *irq,
	u8 mask)
{
	mutex_lock(&info->irq_access_lock);
	*irq |= mask;
	mutex_unlock(&info->irq_access_lock);
}

static inline void rt9466_irq_clr_flag(struct rt9466_info *info, u8 *irq,
	u8 mask)
{
	mutex_lock(&info->irq_access_lock);
	*irq &= ~mask;
	mutex_unlock(&info->irq_access_lock);
}

static inline const char *rt9466_get_irq_name(struct rt9466_info *info,
	int irqnum)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(rt9466_irq_mapping_tbl); i++) {
		if (rt9466_irq_mapping_tbl[i].id == irqnum)
			return rt9466_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void rt9466_irq_mask(struct rt9466_info *info, int irqnum)
{
	dev_dbg(info->dev, "%s: irq = %d, %s\n", __func__, irqnum,
		rt9466_get_irq_name(info, irqnum));
	rt9466_irqmask[irqnum / 8] |= (1 << (irqnum % 8));
}

static inline void rt9466_irq_unmask(struct rt9466_info *info, int irqnum)
{
	dev_dbg(info->dev, "%s: irq = %d, %s\n", __func__, irqnum,
		rt9466_get_irq_name(info, irqnum));
	rt9466_irqmask[irqnum / 8] &= ~(1 << (irqnum % 8));
}

static u8 rt9466_find_closest_reg_value(u32 min, u32 max, u32 step, u32 num,
	u32 target)
{
	u32 i = 0, cur_val = 0, next_val = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	for (i = 0; i < num - 1; i++) {
		cur_val = min + i * step;
		next_val = cur_val + step;

		if (cur_val > max)
			cur_val = max;

		if (next_val > max)
			next_val = max;

		if (target >= cur_val && target < next_val)
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return num - 1;
}

static u8 rt9466_find_closest_reg_value_via_table(const u32 *value_table,
	u32 table_size, u32 target_value)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target_value < value_table[0])
		return 0;

	for (i = 0; i < table_size - 1; i++) {
		if (target_value >= value_table[i] &&
		    target_value < value_table[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return table_size - 1;
}

static u32 rt9466_find_closest_real_value(u32 min, u32 max, u32 step,
	u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static int rt9466_get_adc(struct rt9466_info *info,
	enum rt9466_adc_sel adc_sel, int *adc_val)
{
	int ret = 0;
	u8 adc_data[2] = {0, 0};
	u32 aicr = 0, ichg = 0;
	bool adc_start = false;

	mutex_lock(&info->adc_access_lock);

	/* Select ADC to desired channel */
	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_ADC,
		adc_sel << RT9466_SHIFT_ADC_IN_SEL,
		RT9466_MASK_ADC_IN_SEL
	);

	if (ret < 0) {
		dev_err(info->dev, "%s: select ch to %d failed, ret = %d\n",
			__func__, adc_sel, ret);
		goto out;
	}

	/* Workaround for IBUS & IBAT */
	if (adc_sel == RT9466_ADC_IBUS) {
		mutex_lock(&info->aicr_access_lock);
		ret = rt9466_get_aicr(info->chg_dev, &aicr);
		if (ret < 0) {
			dev_err(info->dev, "%s: get aicr failed\n", __func__);
			goto out_unlock_all;
		}
	} else if (adc_sel == RT9466_ADC_IBAT) {
		mutex_lock(&info->ichg_access_lock);
		ret = rt9466_get_ichg(info->chg_dev, &ichg);
		if (ret < 0) {
			dev_err(info->dev, "%s: get ichg failed\n", __func__);
			goto out_unlock_all;
		}
	}

	/* Clear adc done event */
	rt9466_irq_clr_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ3],
		RT9466_MASK_ADC_DONEI);

	/* Start ADC conversation */
	ret = rt9466_set_bit(info, RT9466_REG_CHG_ADC, RT9466_MASK_ADC_START);
	if (ret < 0) {
		dev_err(info->dev, "%s: start con failed, sel = %d, ret = %d\n",
			__func__, adc_sel, ret);
		goto out_unlock_all;
	}

	mutex_lock(&info->adcflag_access_lock);
	info->adc_wait_evt = true;
	mutex_unlock(&info->adcflag_access_lock);

	/* Wait for ADC conversation */
	ret = wait_event_interruptible_timeout(info->wait_queue,
		info->irq_flag[RT9466_IRQIDX_CHG_IRQ3] & RT9466_MASK_ADC_DONEI,
		msecs_to_jiffies(150));

	mutex_lock(&info->adcflag_access_lock);
	info->adc_wait_evt = false;
	mutex_unlock(&info->adcflag_access_lock);

	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_ADC,
		RT9466_SHIFT_ADC_START, &adc_start);
	if (ret < 0 || adc_start) {
		dev_err(info->dev, "%s: wait con failed, sel = %d, ret = %d\n",
			__func__, adc_sel, ret);
		ret = -EINVAL;
		goto out_unlock_all;
	}

	mdelay(1);

	/* Read ADC data high/low byte */
	ret = rt9466_i2c_block_read(info, RT9466_REG_ADC_DATA_H, 2, adc_data);
	if (ret < 0) {
		dev_err(info->dev, "%s: read ADC data failed\n", __func__);
		goto out_unlock_all;
	}

	/* Calculate ADC value */
	*adc_val = (adc_data[0] * 256 + adc_data[1]) * rt9466_adc_unit[adc_sel]
		+ rt9466_adc_offset[adc_sel];

	dev_dbg(info->dev,
		"%s: adc_sel = %d, adc_h = 0x%02X, adc_l = 0x%02X, val = %d\n",
		__func__, adc_sel, adc_data[0], adc_data[1], *adc_val);

out_unlock_all:
	/* Coefficient of IBUS & IBAT */
	if (adc_sel == RT9466_ADC_IBUS) {
		if (aicr < 400000) /* 400mA */
			*adc_val = *adc_val * 67 / 100;
		mutex_unlock(&info->aicr_access_lock);
	} else if (adc_sel == RT9466_ADC_IBAT) {
		if (ichg >= 100000 && ichg <= 450000) /* 100~450mA */
			*adc_val = *adc_val * 57 / 100;
		else if (ichg >= 500000 && ichg <= 850000) /* 500~850mA */
			*adc_val = *adc_val * 63 / 100;
		mutex_unlock(&info->ichg_access_lock);
	}

out:
	mutex_unlock(&info->adc_access_lock);
	return ret;
}


static int rt9466_set_aicl_vth(struct rt9466_info *info, u32 aicl_vth)
{
	int ret = 0;
	u8 reg_aicl_vth = 0;

	reg_aicl_vth = rt9466_find_closest_reg_value(RT9466_AICL_VTH_MIN,
		RT9466_AICL_VTH_MAX, RT9466_AICL_VTH_STEP, RT9466_AICL_VTH_NUM,
		aicl_vth);

	dev_info(info->dev, "%s: vth = %d(0x%02X)\n", __func__, aicl_vth, reg_aicl_vth);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL14,
		reg_aicl_vth << RT9466_SHIFT_AICL_VTH,
		RT9466_MASK_AICL_VTH
	);

	return ret;
}

static int _rt9466_set_aicr(struct rt9466_info *info, u32 aicr)
{
	int ret = 0;
	u8 reg_aicr = 0;

	if (info->aicr_limit != -1 && aicr > info->aicr_limit) {
		dev_err(info->dev, "%s: %dmA over TA's cap, can only be %dmA\n",
			__func__, aicr, info->aicr_limit);
		aicr = info->aicr_limit;
	}

	/* Find corresponding reg value */
	reg_aicr = rt9466_find_closest_reg_value(RT9466_AICR_MIN,
		RT9466_AICR_MAX, RT9466_AICR_STEP, RT9466_AICR_NUM, aicr);

	dev_info(info->dev, "%s: aicr = %d(0x%02X)\n", __func__, aicr,
		reg_aicr);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL3,
		reg_aicr << RT9466_SHIFT_AICR,
		RT9466_MASK_AICR
	);

	return ret;
}

static int rt9466_run_aicl(struct rt9466_info *info)
{
	int ret = 0;
	u32 mivr = 0, aicl_vth = 0, aicr = 0;

	ret = rt9466_get_mivr(info, &mivr);
	if (ret < 0)
		goto out;

	/* Check if there's a suitable AICL_VTH */
	aicl_vth = mivr + 200000;
	if (aicl_vth > RT9466_AICL_VTH_MAX) {
		dev_err(info->dev, "%s: no suitable vth, vth = %d\n", __func__,
			aicl_vth);
		ret = -EINVAL;
		goto out;
	}

	ret = rt9466_set_aicl_vth(info, aicl_vth);
	if (ret < 0)
		goto out;

	/* Clear AICL measurement IRQ */
	rt9466_irq_clr_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ2],
		RT9466_MASK_CHG_AICLMEASI);

	mutex_lock(&info->pe_access_lock);
	mutex_lock(&info->aicr_access_lock);

	/* Set AICR to max */
	ret = _rt9466_set_aicr(info, RT9466_AICR_MAX);
	if (ret < 0)
		goto unlock_out;

	ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL14,
		RT9466_MASK_AICL_MEAS);
	if (ret < 0)
		goto unlock_out;

	ret = wait_event_interruptible_timeout(info->wait_queue,
		info->irq_flag[RT9466_IRQIDX_CHG_IRQ2] & RT9466_MASK_CHG_AICLMEASI,
		msecs_to_jiffies(2500));
	if (ret <= 0) {
		dev_err(info->dev, "%s: wait AICL time out\n", __func__);
		ret = -EIO;
		goto unlock_out;
	}

	ret = rt9466_get_aicr(info->chg_dev, &aicr);
	if (ret < 0)
		goto unlock_out;

	info->aicr_limit = aicr;
	dev_dbg(info->dev, "%s: OK, aicr upper bound = %dmA\n", __func__,
		aicr / 1000);

unlock_out:
	mutex_unlock(&info->aicr_access_lock);
	mutex_unlock(&info->pe_access_lock);
out:
	return ret;
}

static void rt9466_aicl_work_handler(struct work_struct *work)
{
	int ret = 0;
	struct rt9466_info *info = (struct rt9466_info *)container_of(work,
		struct rt9466_info, aicl_work);
	struct charger_manager *chg_mgr = NULL;

	chg_mgr = charger_dev_get_drvdata(info->chg_dev);
	if (!chg_mgr)
		goto out;

	if (mtk_is_pe30_running(chg_mgr) || mtk_pe20_get_is_connect(chg_mgr)) {
		dev_dbg(info->dev, "%s: in PE, stop AICL\n", __func__);
		goto out;
	}

	ret = rt9466_run_aicl(info);
	if (ret < 0)
		dev_err(info->dev, "%s: run aicl failed\n", __func__);
out:
	/* Enable MIVR IRQ */
	ret = rt9466_clr_bit(info, RT9466_REG_CHG_STATC_CTRL,
		RT9466_MASK_CHG_MIVRM);
	if (ret < 0)
		dev_err(info->dev, "%s: en MIVR IRQ failed\n", __func__);
}

/* Prevent back boost */
static int rt9466_toggle_cfo(struct rt9466_info *info)
{
	int ret = 0;
	u8 data = 0;

	dev_info(info->dev, "%s\n", __func__);
	mutex_lock(&info->i2c_access_lock);
	ret = rt9466_device_read(info->client, RT9466_REG_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		dev_err(info->dev, "%s read cfo fail(%d)\n", __func__, ret);
		goto out;
	}

	/* CFO off */
	data &= ~RT9466_MASK_CFO_EN;
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		dev_err(info->dev, "%s cfo off fail(%d)\n", __func__, ret);
		goto out;
	}

	/* CFO on */
	data |= RT9466_MASK_CFO_EN;
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_CTRL2, 1, &data);
	if (ret < 0)
		dev_err(info->dev, "%s cfo on fail(%d)\n", __func__, ret);

out:
	mutex_unlock(&info->i2c_access_lock);
	return ret;
}

/* IRQ handlers */
static int rt9466_pwr_rdy_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_mivr_irq_handler(struct rt9466_info *info)
{
	int ret = 0;
	bool mivr_act = false;
	int adc_ibus = 0;

	dev_err(info->dev, "%s\n", __func__);

	/* Check whether MIVR loop is active */
	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_STATC,
		RT9466_SHIFT_CHG_MIVR, &mivr_act);
	if (ret < 0) {
		dev_err(info->dev, "%s: read mivr stat failed\n", __func__);
		goto out;
	}

	if (!mivr_act) {
		dev_info(info->dev, "%s: mivr loop is not active\n", __func__);
		goto out;
	}

	if (strcmp(info->desc->chg_dev_name, "primary_chg") == 0) {
		/* Check IBUS ADC */
		ret = rt9466_get_adc(info, RT9466_ADC_IBUS, &adc_ibus);
		if (ret < 0) {
			dev_err(info->dev, "%s: get ibus fail\n", __func__);
			return ret;
		}
		if (adc_ibus < 100000) { /* 100mA */
			ret = rt9466_toggle_cfo(info);
			return ret;
		}
	}

	/* Disable MIVR IRQ */
	ret = rt9466_set_bit(info, RT9466_REG_CHG_STATC_CTRL,
		RT9466_MASK_CHG_MIVRM);
	if (ret < 0) {
		dev_err(info->dev, "%s: disable MIVR IRQ failed\n", __func__);
		goto out;
	}

	if (!queue_work(info->aicl_wq, &info->aicl_work))
		dev_err(info->dev, "%s: queue aicl work failed\n",
			__func__);
out:
	return 0;
}

static int rt9466_chg_aicr_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_treg_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_vsysuv_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_vsysov_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_vbatov_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_vbusov_irq_handler(struct rt9466_info *info)
{
	struct chgdev_notify noti;

	dev_err(info->dev, "%s\n", __func__);
	noti.vbusov_stat = true;
	srcu_notifier_call_chain(&info->chg_dev->evt_nh,
		CHARGER_DEV_NOTIFY_VBUS_OVP, &noti);
	return 0;
}

static int rt9466_ts_bat_cold_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_ts_bat_cool_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_ts_bat_warm_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_ts_bat_hot_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_ts_statci_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_faulti_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_statci_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_tmri_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	srcu_notifier_call_chain(&info->chg_dev->evt_nh,
		CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT, NULL);
	return 0;
}

static int rt9466_chg_batabsi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_adpbadi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_rvpi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_otpi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_aiclmeasi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	rt9466_irq_set_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ2],
		RT9466_MASK_CHG_AICLMEASI);
	wake_up_interruptible(&info->wait_queue);
	return 0;
}

static int rt9466_chg_ichgmeasi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_wdtmri_irq_handler(struct rt9466_info *info)
{
	int ret = 0;

	dev_err(info->dev, "%s\n", __func__);
	ret = rt9466_kick_wdt(info->chg_dev);
	if (ret < 0)
		dev_err(info->dev, "%s: kick wdt failed\n", __func__);

	return ret;
}

static int rt9466_ssfinishi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_rechgi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	srcu_notifier_call_chain(&info->chg_dev->evt_nh,
		CHARGER_DEV_NOTIFY_RECHG, NULL);
	return 0;
}

static int rt9466_chg_termi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_chg_ieoci_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	srcu_notifier_call_chain(&info->chg_dev->evt_nh,
		CHARGER_DEV_NOTIFY_EOC, NULL);
	return 0;
}

static int rt9466_adc_donei_irq_handler(struct rt9466_info *info)
{
	int ret = 0;
	bool adc_start = false;

	dev_err(info->dev, "%s\n", __func__);

	mutex_lock(&info->adcflag_access_lock);

	/* Read ADC_START */
	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_ADC,
		RT9466_SHIFT_ADC_START, &adc_start);
	if (ret < 0) {
		dev_err(info->dev, "%s: read adc start failed\n", __func__);
		goto out;
	}

	if (!adc_start && info->adc_wait_evt) {
		rt9466_irq_set_flag(info,
			&info->irq_flag[RT9466_IRQIDX_CHG_IRQ3],
			RT9466_MASK_ADC_DONEI);
		wake_up_interruptible(&info->wait_queue);
	} else
		dev_err(info->dev, "%s: ignore irq\n", __func__);

out:
	mutex_unlock(&info->adcflag_access_lock);
	return 0;
}

static int rt9466_pumpx_donei_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	rt9466_irq_set_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ3],
		RT9466_MASK_PUMPX_DONEI);
	wake_up_interruptible(&info->wait_queue);
	return 0;
}

static int rt9466_bst_batuvi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_bst_midovi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

static int rt9466_bst_olpi_irq_handler(struct rt9466_info *info)
{
	dev_err(info->dev, "%s\n", __func__);
	return 0;
}

typedef int (*rt9466_irq_fptr)(struct rt9466_info *);
static rt9466_irq_fptr rt9466_irq_handler_tbl[48] = {
	NULL,
	NULL,
	NULL,
	NULL,
	rt9466_chg_treg_irq_handler,
	rt9466_chg_aicr_irq_handler,
	rt9466_chg_mivr_irq_handler,
	rt9466_pwr_rdy_irq_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	rt9466_chg_vsysuv_irq_handler,
	rt9466_chg_vsysov_irq_handler,
	rt9466_chg_vbatov_irq_handler,
	rt9466_chg_vbusov_irq_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	rt9466_ts_bat_cold_irq_handler,
	rt9466_ts_bat_cool_irq_handler,
	rt9466_ts_bat_warm_irq_handler,
	rt9466_ts_bat_hot_irq_handler,
	rt9466_ts_statci_irq_handler,
	rt9466_chg_faulti_irq_handler,
	rt9466_chg_statci_irq_handler,
	rt9466_chg_tmri_irq_handler,
	rt9466_chg_batabsi_irq_handler,
	rt9466_chg_adpbadi_irq_handler,
	rt9466_chg_rvpi_irq_handler,
	rt9466_chg_otpi_irq_handler,
	rt9466_chg_aiclmeasi_irq_handler,
	rt9466_chg_ichgmeasi_irq_handler,
	NULL,
	rt9466_wdtmri_irq_handler,
	rt9466_ssfinishi_irq_handler,
	rt9466_chg_rechgi_irq_handler,
	rt9466_chg_termi_irq_handler,
	rt9466_chg_ieoci_irq_handler,
	rt9466_adc_donei_irq_handler,
	rt9466_pumpx_donei_irq_handler,
	NULL,
	NULL,
	NULL,
	rt9466_bst_batuvi_irq_handler,
	rt9466_bst_midovi_irq_handler,
	rt9466_bst_olpi_irq_handler,
};

static irqreturn_t rt9466_irq_handler(int irq, void *data)
{
	int ret = 0, i = 0, j = 0;
	u8 evt[RT9466_IRQIDX_MAX] = {0};
	u8 mask[RT9466_IRQIDX_MAX] = {0};
	struct rt9466_info *info = (struct rt9466_info *)data;

	dev_info(info->dev, "%s\n", __func__);

	/* Read event */
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_STATC,
		ARRAY_SIZE(evt), evt);
	if (ret < 0) {
		dev_err(info->dev, "%s: read evt fail(%d)\n", __func__, ret);
		goto err_read_irq;
	}

	/* Read mask */
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(mask), mask);
	if (ret < 0) {
		dev_err(info->dev, "%s: read mask fail(%d)\n", __func__, ret);
		goto err_read_irq;
	}

	for (i = 0; i < RT9466_IRQIDX_MAX; i++) {
		evt[i] &= ~mask[i];
		for (j = 0; j < 8; j++) {
			if (!(evt[i] & (1 << j)))
				continue;
			if (rt9466_irq_handler_tbl[i * 8 + j])
				rt9466_irq_handler_tbl[i * 8 + j](info);
		}
	}

err_read_irq:
	return IRQ_HANDLED;
}

static int rt9466_irq_register(struct rt9466_info *info)
{
	int ret = 0, len;
	char *name = NULL;


	dev_info(info->dev, "%s\n", __func__);

	/* request gpio */
	len = strlen(info->desc->chg_dev_name);
	name = devm_kzalloc(info->dev, len + 10, GFP_KERNEL);
	sprintf(name,  "%s_irq_gpio", info->desc->chg_dev_name);
	ret = devm_gpio_request_one(info->dev, info->intr_gpio, GPIOF_IN, name);
	if (ret < 0) {
		dev_err(info->dev, "%s: gpio request fail\n", __func__);
		return ret;
	}

	ret = gpio_to_irq(info->intr_gpio);
	if (ret < 0) {
		dev_err(info->dev, "%s: irq mapping fail\n", __func__);
		goto err_to_irq;
	}
	info->irq = ret;
	dev_info(info->dev, "%s: irq = %d\n", __func__, info->irq);

	/* Request threaded IRQ */
	name = devm_kzalloc(info->dev, len + 5, GFP_KERNEL);
	sprintf(name, "%s_irq", info->desc->chg_dev_name);
	ret = devm_request_threaded_irq(info->dev, info->irq, NULL,
		rt9466_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, name,
		info);
	if (ret < 0)
		dev_err(info->dev, "%s: request thread irq failed\n", __func__);

	return 0;

err_to_irq:
	return ret;
}

static int rt9466_maskall_irq(struct rt9466_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	mutex_lock(&info->i2c_access_lock);
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(rt9466_irq_maskall), rt9466_irq_maskall);
	mutex_unlock(&info->i2c_access_lock);
	if (ret < 0)
		dev_err(info->dev, "%s: failed\n", __func__);

	return ret;
}

static int rt9466_irq_init(struct rt9466_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	ret = rt9466_i2c_block_write(info, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(rt9466_irqmask), rt9466_irqmask);
	if (ret < 0)
		dev_err(info->dev, "%s: init irq failed\n", __func__);

	return ret;
}


static bool rt9466_is_hw_exist(struct rt9466_info *info)
{
	int ret = 0;
	u8 vendor_id = 0, chip_rev = 0;

	ret = i2c_smbus_read_byte_data(info->client, RT9466_REG_DEVICE_ID);
	if (ret < 0)
		return false;

	vendor_id = ret & 0xF0;
	chip_rev = ret & 0x0F;
	if (vendor_id != RT9466_VENDOR_ID) {
		dev_err(info->dev, "%s: vendor id is incorrect (0x%02X)\n",
			__func__, vendor_id);
		return false;
	}

	dev_info(info->dev, "%s: 0x%02X\n", __func__, chip_rev);
	info->chip_rev = chip_rev;

	return true;
}

static int rt9466_set_fast_charge_timer(struct rt9466_info *info, u32 hour)
{
	int ret = 0;
	u8 reg_fct = 0;

	reg_fct = rt9466_find_closest_reg_value(RT9466_WT_FC_MIN,
		RT9466_WT_FC_MAX, RT9466_WT_FC_STEP, RT9466_WT_FC_NUM, hour);

	dev_info(info->dev, "%s: time = %d(0x%02X)\n", __func__, hour, reg_fct);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL12,
		reg_fct << RT9466_SHIFT_WT_FC,
		RT9466_MASK_WT_FC
	);

	return ret;
}

static int rt9466_enable_wdt(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL13, RT9466_MASK_WDT_EN);

	return ret;
}

/* Hardware pin current limit */
static int rt9466_enable_ilim(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL3, RT9466_MASK_ILIM_EN);

	return ret;
}

/* Select IINLMTSEL to use AICR */
static int rt9466_select_input_current_limit(struct rt9466_info *info,
	enum rt9466_iin_limit_sel sel)
{
	int ret = 0;

	dev_info(info->dev, "%s: select input current limit = %d\n",
		__func__, sel);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL2,
		sel << RT9466_SHIFT_IINLMTSEL,
		RT9466_MASK_IINLMTSEL
	);

	return ret;
}

/* Enable/Disable hidden mode */
static int rt9466_enable_hidden_mode(struct rt9466_info *info, bool en)
{
	int ret = 0;

	mutex_lock(&info->hidden_mode_lock);

	if (en) {
		if (info->hidden_mode_cnt == 0) {
			/* Enable hidden mode */
			ret = rt9466_i2c_block_write(info,
				rt9466_reg_en_hidden_mode[0],
				ARRAY_SIZE(rt9466_val_en_hidden_mode),
				rt9466_val_en_hidden_mode);
			if (ret < 0)
				goto err;
		}
		info->hidden_mode_cnt++;
	} else {
		if (info->hidden_mode_cnt == 1) /* last one */
			ret = rt9466_i2c_write_byte(info, 0x70, 0x00);
		info->hidden_mode_cnt--;
		if (ret < 0)
			goto err;
	}
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	goto out;

err:
	dev_err(info->dev, "%s: en = %d fail(%d)\n", __func__, en, ret);
out:
	mutex_unlock(&info->hidden_mode_lock);
	return ret;
}

static int rt9466_set_iprec(struct rt9466_info *info, u32 iprec)
{
	int ret = 0;
	u8 reg_iprec = 0;

	/* Find corresponding reg value */
	reg_iprec = rt9466_find_closest_reg_value(RT9466_IPREC_MIN,
		RT9466_IPREC_MAX, RT9466_IPREC_STEP, RT9466_IPREC_NUM, iprec);

	dev_info(info->dev, "%s: iprec = %d(0x%02X)\n", __func__, iprec,
		reg_iprec);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL8,
		reg_iprec << RT9466_SHIFT_IPREC,
		RT9466_MASK_IPREC
	);

	return ret;
}

/* Software workaround */
static int rt9466_sw_workaround(struct rt9466_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	/* Enter hidden mode */
	ret = rt9466_enable_hidden_mode(info, true);
	if (ret < 0)
		goto out;

	/* Set precharge current to 850mA, only do this in normal boot */
	if (info->chip_rev <= RT9466_CHIP_REV_E3) {
		if (get_boot_mode() == NORMAL_BOOT) {
			ret = rt9466_set_iprec(info, 850000);
			if (ret < 0)
				goto out;

			/* Increase Isys drop threshold to 2.5A */
			ret = rt9466_i2c_write_byte(info,
				RT9466_REG_CHG_HIDDEN_CTRL7, 0x1c);
			if (ret < 0)
				goto out;
		}

		/* Disable TS auto sensing */
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_HIDDEN_CTRL15, 0x01);
		if (ret < 0)
			goto out;

		/* Worst case delay: wait auto sensing */
		msleep(200);
	}

	/* Only revision <= E1 needs the following workaround */
	if (info->chip_rev > RT9466_CHIP_REV_E1)
		goto out;

	/* ICC: modify sensing node, make it more accurate */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL8, 0x00);
	if (ret < 0)
		goto out;

	/* DIMIN level */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL9, 0x86);

out:
	/* Exit hidden mode */
	ret = rt9466_enable_hidden_mode(info, false);
	if (ret < 0)
		dev_err(info->dev, "%s: exit hidden mode failed\n", __func__);

	return ret;
}

static int rt9466_enable_hz(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL1, RT9466_MASK_HZ_EN);

	return ret;
}

/* Reset all registers' value to default */
static int rt9466_reset_chip(struct rt9466_info *info)
{
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	/* disable hz before reset chip */
	ret = rt9466_enable_hz(info, false);
	if (ret < 0) {
		dev_err(info->dev, "%s: disable hz failed\n", __func__);
		return ret;
	}

	ret = rt9466_set_bit(info, RT9466_REG_CORE_CTRL0, RT9466_MASK_RST);

	return ret;
}

static int rt9466_enable_te(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL2, RT9466_MASK_TE_EN);

	return ret;
}

static int _rt9466_enable_safety_timer(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL12, RT9466_MASK_TMR_EN);

	return ret;
}


static int rt9466_set_ieoc(struct rt9466_info *info, u32 ieoc)
{
	int ret = 0;

	/* Find corresponding reg value */
	u8 reg_ieoc = rt9466_find_closest_reg_value(RT9466_IEOC_MIN,
		RT9466_IEOC_MAX, RT9466_IEOC_STEP, RT9466_IEOC_NUM, ieoc);

	dev_info(info->dev, "%s: ieoc = %d(0x%02X)\n", __func__, ieoc,
		reg_ieoc);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL9,
		reg_ieoc << RT9466_SHIFT_IEOC,
		RT9466_MASK_IEOC
	);

	return ret;
}

static int rt9466_get_mivr(struct rt9466_info *info, u32 *mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL6);
	if (ret < 0)
		return ret;
	reg_mivr = ((ret & RT9466_MASK_MIVR) >> RT9466_SHIFT_MIVR) & 0xFF;

	*mivr = rt9466_find_closest_real_value(RT9466_MIVR_MIN, RT9466_MIVR_MAX,
		RT9466_MIVR_STEP, reg_mivr);

	return ret;
}

static int _rt9466_set_mivr(struct rt9466_info *info, u32 mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;

	/* Find corresponding reg value */
	reg_mivr = rt9466_find_closest_reg_value(RT9466_MIVR_MIN,
		RT9466_MIVR_MAX, RT9466_MIVR_STEP, RT9466_MIVR_NUM, mivr);

	dev_info(info->dev, "%s: mivr = %d(0x%02X)\n", __func__, mivr,
		reg_mivr);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL6,
		reg_mivr << RT9466_SHIFT_MIVR,
		RT9466_MASK_MIVR
	);

	return ret;
}

static int rt9466_enable_jeita(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL16, RT9466_MASK_JEITA_EN);

	return ret;
}


static int rt9466_get_charging_status(struct rt9466_info *info,
	enum rt9466_charging_status *chg_stat)
{
	int ret = 0;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_STAT);
	if (ret < 0)
		return ret;

	*chg_stat = (ret & RT9466_MASK_CHG_STAT) >> RT9466_SHIFT_CHG_STAT;

	return ret;
}

static int rt9466_get_ieoc(struct rt9466_info *info, u32 *ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL9);
	if (ret < 0)
		return ret;

	reg_ieoc = (ret & RT9466_MASK_IEOC) >> RT9466_SHIFT_IEOC;
	*ieoc = rt9466_find_closest_real_value(RT9466_IEOC_MIN, RT9466_IEOC_MAX,
		RT9466_IEOC_STEP, reg_ieoc);

	return ret;
}

static inline int _rt9466_is_charging_enable(struct rt9466_info *info, bool *en)
{
	int ret = 0;

	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_CTRL2,
		RT9466_SHIFT_CHG_EN, en);

	return ret;
}

static int _rt9466_set_ichg(struct rt9466_info *info, u32 ichg)
{
	int ret = 0;
	u8 reg_ichg = 0;

	/* Find corresponding reg value */
	reg_ichg = rt9466_find_closest_reg_value(RT9466_ICHG_MIN,
		RT9466_ICHG_MAX, RT9466_ICHG_STEP, RT9466_ICHG_NUM, ichg);

	dev_info(info->dev, "%s: ichg = %d(0x%02X)\n", __func__, ichg,
		reg_ichg);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL7,
		reg_ichg << RT9466_SHIFT_ICHG,
		RT9466_MASK_ICHG
	);

	return ret;
}

static int _rt9466_set_cv(struct rt9466_info *info, u32 cv)
{
	int ret = 0;
	u8 reg_cv = 0;

	reg_cv = rt9466_find_closest_reg_value(RT9466_CV_MIN,
		RT9466_CV_MAX, RT9466_CV_STEP, RT9466_CV_NUM, cv);

	dev_info(info->dev, "%s: cv = %d(0x%02X)\n", __func__, cv, reg_cv);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL4,
		reg_cv << RT9466_SHIFT_CV,
		RT9466_MASK_CV
	);

	return ret;
}

static int rt9466_set_ircmp_resistor(struct rt9466_info *info, u32 uohm)
{
	int ret = 0;
	u8 reg_resistor = 0;

	reg_resistor = rt9466_find_closest_reg_value(RT9466_IRCMP_RES_MIN,
		RT9466_IRCMP_RES_MAX, RT9466_IRCMP_RES_STEP,
		RT9466_IRCMP_RES_NUM, uohm);

	dev_info(info->dev, "%s: resistor = %d(0x%02X)\n", __func__, uohm,
		reg_resistor);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL18,
		reg_resistor << RT9466_SHIFT_IRCMP_RES,
		RT9466_MASK_IRCMP_RES
	);

	return ret;
}

static int rt9466_set_ircmp_vclamp(struct rt9466_info *info, u32 uV)
{
	int ret = 0;
	u8 reg_vclamp = 0;

	reg_vclamp = rt9466_find_closest_reg_value(RT9466_IRCMP_VCLAMP_MIN,
		RT9466_IRCMP_VCLAMP_MAX, RT9466_IRCMP_VCLAMP_STEP,
		RT9466_IRCMP_VCLAMP_NUM, uV);

	dev_info(info->dev, "%s: vclamp = %d(0x%02X)\n", __func__, uV,
		reg_vclamp);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL18,
		reg_vclamp << RT9466_SHIFT_IRCMP_VCLAMP,
		RT9466_MASK_IRCMP_VCLAMP
	);

	return ret;
}

static int rt9466_enable_pump_express(struct rt9466_info *info, bool en)
{
	int ret = 0;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = rt9466_set_aicr(info->chg_dev, 800000);
	if (ret < 0)
		return ret;

	ret = rt9466_set_ichg(info->chg_dev, 2000000);
	if (ret < 0)
		return ret;

	ret = rt9466_enable_charging(info->chg_dev, true);
	if (ret < 0)
		return ret;

	rt9466_irq_clr_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ3],
		RT9466_MASK_PUMPX_DONEI);

	rt9466_enable_hidden_mode(info, true);

	ret = rt9466_clr_bit(info, RT9466_REG_CHG_HIDDEN_CTRL9, 0x80);
	if (ret < 0)
		dev_err(info->dev, "%s: disable skip mode fail\n", __func__);

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_EN);
	if (ret < 0)
		goto out;

	ret = wait_event_interruptible_timeout(info->wait_queue,
		info->irq_flag[RT9466_IRQIDX_CHG_IRQ3] & RT9466_MASK_PUMPX_DONEI,
		msecs_to_jiffies(7500));
	if (ret <= 0) {
		dev_err(info->dev, "%s: pumpx done fail(%d)\n", __func__, ret);
		ret = -EIO;
	} else
		ret = 0;

out:
	rt9466_set_bit(info, RT9466_REG_CHG_HIDDEN_CTRL9, 0x80);
	rt9466_enable_hidden_mode(info, false);
	return ret;
}

static inline int rt9466_get_irq_number(struct rt9466_info *info,
	const char *name)
{
	int i = 0;

	if (!name) {
		dev_err(info->dev, "%s: null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rt9466_irq_mapping_tbl); i++) {
		if (!strcmp(name, rt9466_irq_mapping_tbl[i].name))
			return rt9466_irq_mapping_tbl[i].id;
	}

	return -EINVAL;
}

static int rt9466_parse_dt(struct rt9466_info *info, struct device *dev)
{
	int ret = 0, irq_cnt = 0;
	struct rt9466_desc *desc = NULL;
	struct device_node *np = dev->of_node;
	const char *name = NULL;
	int irqnum = 0;

	dev_info(info->dev, "%s\n", __func__);

	if (!np) {
		dev_err(info->dev, "%s: no device node\n", __func__);
		return -EINVAL;
	}

	info->desc = &rt9466_default_desc;

	desc = devm_kzalloc(dev, sizeof(struct rt9466_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &rt9466_default_desc, sizeof(struct rt9466_desc));

	if (of_property_read_string(np, "charger_name",
		&desc->chg_dev_name) < 0)
		dev_err(info->dev, "%s: no charger name\n", __func__);

#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "rt,intr_gpio", 0);
	if (ret < 0)
		return ret;
	info->intr_gpio = ret;
	if (strcmp(desc->chg_dev_name, "secondary_chg") == 0) {
		ret = of_get_named_gpio(np, "rt,ceb_gpio", 0);
		if (ret < 0)
			return ret;
		info->ceb_gpio = ret;
	}
#else
	ret = of_property_read_u32(np, "rt,intr_gpio_num", &info->intr_gpio);
	if (ret < 0)
		return ret;
	if (strcmp(desc->chg_dev_name, "secondary_chg") == 0) {
		ret = of_property_read_u32(np, "rt,ceb_gpio_num",
			&info->ceb_gpio);
		if (ret < 0)
			return ret;
	}
#endif

	dev_info(info->dev, "%s: intr/ceb gpio = %d, %d\n", __func__,
		info->intr_gpio, info->ceb_gpio);

	/* request ceb gpio for secondary charger */
	if (strcmp(desc->chg_dev_name, "secondary_chg") == 0) {
		ret = devm_gpio_request_one(info->dev, info->ceb_gpio,
			GPIOF_DIR_OUT, "rt9466_sec_ceb_gpio");
		if (ret < 0) {
			dev_err(info->dev, "%s: ceb gpio request fail\n",
				__func__);
			return ret;
		}
	}

	if (of_property_read_u32(np, "regmap_represent_slave_addr",
		&desc->regmap_represent_slave_addr) < 0)
		dev_err(info->dev, "%s: no regmap slave addr\n", __func__);

	if (of_property_read_string(np, "regmap_name",
		&(desc->regmap_name)) < 0)
		dev_err(info->dev, "%s: no regmap name\n", __func__);

	if (of_property_read_u32(np, "ichg", &desc->ichg) < 0)
		dev_err(info->dev, "%s: no ichg\n", __func__);

	if (of_property_read_u32(np, "aicr", &desc->aicr) < 0)
		dev_err(info->dev, "%s: no aicr\n", __func__);

	if (of_property_read_u32(np, "mivr", &desc->mivr) < 0)
		dev_err(info->dev, "%s: no mivr\n", __func__);

	if (of_property_read_u32(np, "cv", &desc->cv) < 0)
		dev_err(info->dev, "%s: no cv\n", __func__);

	if (of_property_read_u32(np, "ieoc", &desc->ieoc) < 0)
		dev_err(info->dev, "%s: no ieoc\n", __func__);

	if (of_property_read_u32(np, "safety_timer", &desc->safety_timer) < 0)
		dev_err(info->dev, "%s: no safety timer\n", __func__);

	if (of_property_read_u32(np, "ircmp_resistor",
		&desc->ircmp_resistor) < 0)
		dev_err(info->dev, "%s: no ircmp resistor\n", __func__);

	if (of_property_read_u32(np, "ircmp_vclamp", &desc->ircmp_vclamp) < 0)
		dev_err(info->dev, "%s: no ircmp vclamp\n", __func__);

	desc->en_te = of_property_read_bool(np, "enable_te");
	desc->en_wdt = of_property_read_bool(np, "enable_wdt");
	desc->ceb_invert = of_property_read_bool(np, "ceb_invert");

	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
			irq_cnt, &name);
		if (ret < 0)
			break;
		irq_cnt++;
		irqnum = rt9466_get_irq_number(info, name);
		if (irqnum >= 0)
			rt9466_irq_unmask(info, irqnum);
	}

	info->desc = desc;

	return 0;
}


/* =========================================================== */
/* Released interfaces                                         */
/* =========================================================== */

static int rt9466_enable_charging(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	/* set hz/ceb pin for secondary charger */
	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0) {
		ret = rt9466_enable_hz(info, !en);
		if (ret < 0) {
			dev_err(info->dev, "%s: set hz of sec chg fail\n",
				__func__);
			return ret;
		}
		if (info->desc->ceb_invert)
			gpio_set_value(info->ceb_gpio, en);
		else
			gpio_set_value(info->ceb_gpio, !en);
	}

	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL2, RT9466_MASK_CHG_EN);

	return ret;
}

static int rt9466_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = _rt9466_enable_safety_timer(info, en);

	return ret;
}

static int rt9466_set_boost_current_limit(struct charger_device *chg_dev,
	u32 current_limit)
{
	int ret = 0;
	u8 reg_ilimit = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	reg_ilimit = rt9466_find_closest_reg_value_via_table(
		rt9466_boost_oc_threshold,
		ARRAY_SIZE(rt9466_boost_oc_threshold),
		current_limit
	);

	dev_info(info->dev, "%s: boost ilimit = %d(0x%02X)\n", __func__,
		current_limit, reg_ilimit);

	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL10,
		reg_ilimit << RT9466_SHIFT_BOOST_OC,
		RT9466_MASK_BOOST_OC
	);

	return ret;
}

static int rt9466_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	bool en_otg = false;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);
	u8 hidden_val = en ? 0x00 : 0x0F;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	/* Enable WDT */
	if (en && info->desc->en_wdt) {
		ret = rt9466_enable_wdt(info, true);
		if (ret < 0) {
			dev_err(info->dev, "%s: en wdt failed\n", __func__);
			return ret;
		}
	}

	/* Set OTG_OC to 500mA */
	ret = rt9466_set_boost_current_limit(chg_dev, 500000);
	if (ret < 0) {
		dev_err(info->dev, "%s: set current limit fail\n", __func__);
		return ret;
	}

	/* Clear soft start event */
	rt9466_irq_clr_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ2],
		RT9466_MASK_SSFINISHI);

	/* Switch OPA mode */
	ret = (en ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL1, RT9466_MASK_OPA_MODE);

	msleep(20);

	if (en) {
		ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_SHIFT_OPA_MODE, &en_otg);
		if (ret < 0 || !en_otg) {
			dev_err(info->dev, "%s: otg fail(%d)\n", __func__, ret);

			/* Disable WDT */
			ret = rt9466_enable_wdt(info, false);
			if (ret < 0)
				dev_err(info->dev, "%s: disable wdt fail\n",
					__func__);

			return -EIO;
		}
	}

	/*
	 * Woraround reg[0x25] = 0x00 after entering OTG mode
	 * reg[0x25] = 0x0F after leaving OTG mode
	 */

	/* Enter hidden mode */
	ret = rt9466_enable_hidden_mode(info, true);
	if (ret < 0)
		return ret;

	/* Woraround reg[0x25] = 0x00 after entering OTG mode */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL6,
		hidden_val);
	if (ret < 0)
		dev_err(info->dev, "%s: workaroud fail(%d)\n", __func__, ret);

	/* Exit hidden mode */
	ret = rt9466_enable_hidden_mode(info, false);

	/* Disable WDT */
	if (!en) {
		ret = rt9466_enable_wdt(info, false);
		if (ret < 0)
			dev_err(info->dev, "%s: disable wdt fail\n", __func__);
	}

	return ret;
}

static int rt9466_enable_discharge(struct charger_device *chg_dev, bool en)
{
	int ret = 0, i = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);
	const int check_dischg_max = 3;
	bool is_dischg = true;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	ret = rt9466_enable_hidden_mode(info, true);
	if (ret < 0)
		return ret;

	/* Set bit2 of reg[0x21] to 1 to enable discharging */
	ret = (en ? rt9466_set_bit : rt9466_clr_bit)(info,
		RT9466_REG_CHG_HIDDEN_CTRL2, 0x04);
	if (ret < 0) {
		dev_err(info->dev, "%s: en = %d, failed\n", __func__, en);
		return ret;
	}

	if (!en) {
		for (i = 0; i < check_dischg_max; i++) {
			ret = rt9466_i2c_test_bit(info,
				RT9466_REG_CHG_HIDDEN_CTRL2, 2, &is_dischg);
			if (!is_dischg)
				break;
			/* Disable discharging */
			ret = rt9466_clr_bit(info, RT9466_REG_CHG_HIDDEN_CTRL2,
				0x04);
		}
		if (i == check_dischg_max)
			dev_err(info->dev,
				"%s: disable dischg failed, ret = %d\n",
				__func__, ret);
	}

	ret = rt9466_enable_hidden_mode(info, false);
	return ret;
}

static int rt9466_enable_power_path(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	u32 mivr = (en ? 4500000 : RT9466_MIVR_MAX);
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	ret = _rt9466_set_mivr(info, mivr);

	return ret;
}

static int rt9466_is_power_path_enable(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	u32 mivr = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_get_mivr(info, &mivr);
	if (ret < 0)
		return ret;

	*en = ((mivr == RT9466_MIVR_MAX) ? false : true);


	return ret;
}

static int rt9466_set_ichg(struct charger_device *chg_dev, u32 ichg)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&info->ichg_access_lock);
	ret = _rt9466_set_ichg(info, ichg);
	mutex_unlock(&info->ichg_access_lock);

	return ret;
}

static int rt9466_set_aicr(struct charger_device *chg_dev, u32 aicr)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&info->aicr_access_lock);
	ret = _rt9466_set_aicr(info, aicr);
	mutex_unlock(&info->aicr_access_lock);

	return ret;
}

static int rt9466_set_mivr(struct charger_device *chg_dev, u32 mivr)
{
	int ret = 0;
	bool en = true;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_is_power_path_enable(chg_dev, &en);
	if (!en) {
		dev_info(info->dev,
			"%s: power path is disabled, op is not allowed\n",
			__func__);
		return -EINVAL;
	}

	ret = _rt9466_set_mivr(info, mivr);

	return ret;
}

static int rt9466_set_cv(struct charger_device *chg_dev, u32 cv)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = _rt9466_set_cv(info, cv);

	return ret;
}

static int rt9466_set_pep_current_pattern(struct charger_device *chg_dev,
	bool is_increase)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s: pump_up = %d\n", __func__, is_increase);

	mutex_lock(&info->pe_access_lock);

	/* Set to PE1.0 */
	ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL17,
		RT9466_MASK_PUMPX_20_10);

	/* Set Pump Up/Down */
	ret = (is_increase ? rt9466_set_bit : rt9466_clr_bit)
		(info, RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_UP_DN);

	/* Enable PumpX */
	ret = rt9466_enable_pump_express(info, true);
	mutex_unlock(&info->pe_access_lock);

	return ret;
}

static int rt9466_set_pep20_reset(struct charger_device *chg_dev)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&info->pe_access_lock);
	ret = rt9466_set_mivr(chg_dev, 4500000);
	if (ret < 0)
		goto out;

	ret = rt9466_set_ichg(chg_dev, 512000);
	if (ret < 0)
		goto out;

	ret = rt9466_set_aicr(chg_dev, 100000);
	if (ret < 0)
		goto out;

	msleep(250);

	ret = rt9466_set_aicr(chg_dev, 700000);

out:
	mutex_unlock(&info->pe_access_lock);
	return ret;
}

static int rt9466_set_pep20_current_pattern(struct charger_device *chg_dev,
	u32 uV)
{
	int ret = 0;
	u8 reg_volt = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&info->pe_access_lock);

	/* Find register value of target voltage */
	reg_volt = rt9466_find_closest_reg_value(RT9466_PEP20_VOLT_MIN,
		RT9466_PEP20_VOLT_MAX, RT9466_PEP20_VOLT_STEP,
		RT9466_PEP20_VOLT_NUM, uV);

	dev_info(info->dev, "%s: volt = %d(0x%02X)\n", __func__, uV, reg_volt);

	/* Set to PEP2.0 */
	ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL17,
		RT9466_MASK_PUMPX_20_10);
	if (ret < 0)
		goto out;

	/* Set Voltage */
	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL17,
		reg_volt << RT9466_SHIFT_PUMPX_DEC,
		RT9466_MASK_PUMPX_DEC
	);
	if (ret < 0)
		goto out;

	/* Enable PumpX */
	ret = rt9466_enable_pump_express(info, true);

out:
	mutex_unlock(&info->pe_access_lock);
	return ret;
}

static int rt9466_enable_cable_drop_comp(struct charger_device *chg_dev,
	bool en)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s: en = %d\n", __func__, en);

	mutex_lock(&info->pe_access_lock);

	/* Set to PEP2.0 */
	ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL17,
		RT9466_MASK_PUMPX_20_10);
	if (ret < 0)
		goto out;

	/* Set Voltage */
	ret = rt9466_i2c_update_bits(
		info,
		RT9466_REG_CHG_CTRL17,
		0x1F << RT9466_SHIFT_PUMPX_DEC,
		RT9466_MASK_PUMPX_DEC
	);
	if (ret < 0)
		goto out;

	/* Enable PumpX */
	ret = rt9466_enable_pump_express(info, true);

out:
	mutex_unlock(&info->pe_access_lock);
	return ret;
}

static int rt9466_get_ichg(struct charger_device *chg_dev, u32 *ichg)
{
	int ret = 0;
	u8 reg_ichg = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL7);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & RT9466_MASK_ICHG) >> RT9466_SHIFT_ICHG;
	*ichg = rt9466_find_closest_real_value(RT9466_ICHG_MIN, RT9466_ICHG_MAX,
		RT9466_ICHG_STEP, reg_ichg);

	return ret;
}

static int rt9466_get_aicr(struct charger_device *chg_dev, u32 *aicr)
{
	int ret = 0;
	u8 reg_aicr = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL3);
	if (ret < 0)
		return ret;

	reg_aicr = (ret & RT9466_MASK_AICR) >> RT9466_SHIFT_AICR;
	*aicr = rt9466_find_closest_real_value(RT9466_AICR_MIN, RT9466_AICR_MAX,
		RT9466_AICR_STEP, reg_aicr);

	return ret;
}

static int rt9466_get_cv(struct charger_device *chg_dev, u32 *cv)
{
	int ret = 0;
	u8 reg_cv = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL4);
	if (ret < 0)
		return ret;

	reg_cv = (ret & RT9466_MASK_CV) >> RT9466_SHIFT_CV;
	*cv = rt9466_find_closest_real_value(RT9466_CV_MIN, RT9466_CV_MAX,
		RT9466_CV_STEP, reg_cv);

	return ret;
}

static int rt9466_get_tchg(struct charger_device *chg_dev, int *tchg_min,
	int *tchg_max)
{
	int ret = 0, adc_temp = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	/* Get value from ADC */
	ret = rt9466_get_adc(info, RT9466_ADC_TEMP_JC, &adc_temp);
	if (ret < 0)
		return ret;

	*tchg_min = adc_temp;
	*tchg_max = adc_temp;

	dev_info(info->dev, "%s: temperature = %d\n", __func__, adc_temp);
	return ret;
}

#if 0
static int rt9466_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	int ret = 0, adc_vbus = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	/* Get value from ADC */
	ret = rt9466_get_adc(info, RT9466_ADC_VBUS_DIV2, &adc_vbus);
	if (ret < 0)
		return ret;

	*vbus = adc_vbus;

	dev_info(info->dev, "%s: vbus = %dmA\n", __func__, adc_vbus);
	return ret;
}
#endif

static int rt9466_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret = 0;
	enum rt9466_charging_status chg_stat = RT9466_CHG_STATUS_READY;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_get_charging_status(info, &chg_stat);

	/* Return is charging done or not */
	switch (chg_stat) {
	case RT9466_CHG_STATUS_READY:
	case RT9466_CHG_STATUS_PROGRESS:
	case RT9466_CHG_STATUS_FAULT:
		*done = false;
		break;
	case RT9466_CHG_STATUS_DONE:
		*done = true;
		break;
	default:
		*done = false;
		break;
	}

	return ret;
}

static int rt9466_is_safety_timer_enable(struct charger_device *chg_dev,
	bool *en)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_CTRL12,
		RT9466_SHIFT_TMR_EN, en);

	return ret;
}

static int rt9466_kick_wdt(struct charger_device *chg_dev)
{
	int ret = 0;
	enum rt9466_charging_status chg_status = RT9466_CHG_STATUS_READY;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	/* Any I2C communication can reset watchdog timer */
	ret = rt9466_get_charging_status(info, &chg_status);

	return ret;
}


static int rt9466_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	int ret = 0;
	struct charger_manager *chg_mgr = NULL;

	chg_mgr = charger_dev_get_drvdata(chg_dev);
	if (!chg_mgr)
		return -EINVAL;

	chg_mgr->pe2.profile[0].vchr = 8000000;
	chg_mgr->pe2.profile[1].vchr = 8000000;
	chg_mgr->pe2.profile[2].vchr = 8000000;
	chg_mgr->pe2.profile[3].vchr = 8500000;
	chg_mgr->pe2.profile[4].vchr = 8500000;
	chg_mgr->pe2.profile[5].vchr = 8500000;
	chg_mgr->pe2.profile[6].vchr = 9000000;
	chg_mgr->pe2.profile[7].vchr = 9000000;
	chg_mgr->pe2.profile[8].vchr = 9500000;
	chg_mgr->pe2.profile[9].vchr = 9500000;

	return ret;
}

/*
 * This function is used in shutdown function
 * Use i2c smbus directly
 */
static int rt9466_sw_reset(struct rt9466_info *info)
{
	int ret = 0;
	u8 evt[RT9466_IRQIDX_MAX] = {0};

	/* Register 0x01 ~ 0x10 */
	u8 reg_data[] = {
		0x10, 0x03, 0x23, 0x3C, 0x67, 0x0B, 0x4C, 0xA1,
		0x3C, 0x58, 0x2C, 0x02, 0x52, 0x05, 0x00, 0x10
	};

	dev_info(info->dev, "%s\n", __func__);

	/* enable HZ mode of secondary charger */
	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0)
		reg_data[0] = 0x14;

	/* Mask all irq */
	ret = rt9466_maskall_irq(info);
	if (ret < 0)
		dev_err(info->dev, "%s: mask all irq fail\n", __func__);

	mutex_lock(&info->i2c_access_lock);
	/* Read all irq */
	ret = rt9466_device_read(info->client, RT9466_REG_CHG_STATC,
		ARRAY_SIZE(evt), evt);
	if (ret < 0)
		dev_err(info->dev, "%s: read all irq fail\n", __func__);

	/* Reset necessary registers */
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_CTRL1,
		ARRAY_SIZE(reg_data), reg_data);
	if (ret < 0)
		dev_err(info->dev, "%s: reset registers fail\n", __func__);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static int rt9466_init_setting(struct rt9466_info *info)
{
	int ret = 0;
	struct rt9466_desc *desc = info->desc;
	u8 evt[RT9466_IRQIDX_MAX] = {0};

	dev_info(info->dev, "%s\n", __func__);

	/* mask all irq */
	ret = rt9466_maskall_irq(info);
	if (ret < 0) {
		dev_err(info->dev, "%s: mask all irq fail\n", __func__);
		goto err;
	}

	/* clear event */
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_STATC, ARRAY_SIZE(evt),
		evt);
	if (ret < 0) {
		dev_err(info->dev, "%s: clr evt fail(%d)\n", __func__, ret);
		goto err;
	}

	/* Select IINLMTSEL to use AICR */
	ret = rt9466_select_input_current_limit(info, RT9466_IINLMTSEL_AICR);
	if (ret < 0) {
		dev_err(info->dev, "%s: select iinlmtsel fail\n", __func__);
		goto err;
	}

	mdelay(5);

	/* Disable hardware ILIM */
	ret = rt9466_enable_ilim(info, false);
	if (ret < 0) {
		dev_err(info->dev, "%s: disable ilim fail\n", __func__);
		goto err;
	}

	ret = _rt9466_set_ichg(info, desc->ichg);
	if (ret < 0)
		dev_err(info->dev, "%s: set ichg fail\n", __func__);

	ret = _rt9466_set_aicr(info, desc->aicr);
	if (ret < 0)
		dev_err(info->dev, "%s: set aicr fail\n", __func__);

	ret = _rt9466_set_mivr(info, desc->mivr);
	if (ret < 0)
		dev_err(info->dev, "%s: set mivr fail\n", __func__);

	ret = _rt9466_set_cv(info, desc->cv);
	if (ret < 0)
		dev_err(info->dev, "%s: set cv fail\n", __func__);

	ret = rt9466_set_ieoc(info, desc->ieoc);
	if (ret < 0)
		dev_err(info->dev, "%s: set ieoc fail\n", __func__);

	ret = rt9466_enable_te(info, desc->en_te);
	if (ret < 0)
		dev_err(info->dev, "%s: set te fail\n", __func__);

	/* Set fast charge timer to 12 hours */
	ret = rt9466_set_fast_charge_timer(info, desc->safety_timer);
	if (ret < 0)
		dev_err(info->dev, "%s: set fast timer fail\n", __func__);

	ret = _rt9466_enable_safety_timer(info, true);
	if (ret < 0)
		dev_err(info->dev, "%s: enable chg timer fail\n", __func__);

	ret = rt9466_enable_wdt(info, desc->en_wdt);
	if (ret < 0)
		dev_err(info->dev, "%s: set wdt fail\n", __func__);

	ret = rt9466_enable_jeita(info, false);
	if (ret < 0)
		dev_err(info->dev, "%s: disable jeita fail\n", __func__);

	/* Set ircomp according to BIF */
	ret = rt9466_set_ircmp_resistor(info, desc->ircmp_resistor);
	if (ret < 0)
		dev_err(info->dev, "%s: set ircmp resistor fail\n", __func__);

	ret = rt9466_set_ircmp_vclamp(info, desc->ircmp_vclamp);
	if (ret < 0)
		dev_err(info->dev, "%s: set ircmp clamp fail\n", __func__);

	ret = rt9466_sw_workaround(info);
	if (ret < 0) {
		dev_err(info->dev, "%s: workaround fail\n", __func__);
		return ret;
	}

	/* Enable HZ mode of secondary charger */
	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0) {
		ret = rt9466_enable_hz(info, true);
		if (ret < 0)
			dev_err(info->dev, "%s: hz sec chg fail\n", __func__);
	}
err:
	return ret;
}

static int rt9466_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s\n", __func__);

	/* Enable WDT */
	if (info->desc->en_wdt) {
		ret = rt9466_enable_wdt(info, true);
		if (ret < 0)
			dev_err(info->dev, "%s: en wdt failed\n", __func__);
	}

	/* Enable charging */
	ret = rt9466_enable_charging(chg_dev, true);
	if (ret < 0)
		dev_err(info->dev, "%s: en charging failed\n", __func__);

	return ret;
}

static int rt9466_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s\n", __func__);

	/* Reset AICR limit */
	info->aicr_limit = -1;

	/* Disable charging */
	ret = rt9466_enable_charging(chg_dev, false);
	if (ret < 0) {
		dev_err(info->dev, "%s: disable charging failed\n", __func__);
		return ret;
	}

	/* Disable WDT */
	ret = rt9466_enable_wdt(info, false);
	if (ret < 0)
		dev_err(info->dev, "%s: disable wdt failed\n", __func__);

	/* enable HZ mode of secondary charger */
	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0) {
		ret = rt9466_enable_hz(info, true);
		if (ret < 0)
			dev_err(info->dev, "%s: en hz of sec chg fail\n",
				__func__);
	}

	return ret;
}

static int rt9466_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	return _rt9466_is_charging_enable(info, en);
}

static int rt9466_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	int ret = 0;

	*uA = rt9466_find_closest_real_value(RT9466_ICHG_MIN, RT9466_ICHG_MAX,
		RT9466_ICHG_STEP, 0);

	return ret;
}

static int rt9466_dump_register(struct charger_device *chg_dev)
{
	int i = 0, ret = 0;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0;
	bool chg_en = 0;
	int adc_vsys = 0, adc_vbat = 0, adc_ibat = 0;
	enum rt9466_charging_status chg_status = RT9466_CHG_STATUS_READY;
	struct rt9466_info *info = dev_get_drvdata(&chg_dev->dev);

	ret = rt9466_get_ichg(chg_dev, &ichg);
	ret = rt9466_get_aicr(chg_dev, &aicr);
	ret = rt9466_get_mivr(info, &mivr);
	ret = _rt9466_is_charging_enable(info, &chg_en);
	ret = rt9466_get_ieoc(info, &ieoc);
	ret = rt9466_get_charging_status(info, &chg_status);
	ret = rt9466_get_adc(info, RT9466_ADC_VSYS, &adc_vsys);
	ret = rt9466_get_adc(info, RT9466_ADC_VBAT, &adc_vbat);
	ret = rt9466_get_adc(info, RT9466_ADC_IBAT, &adc_ibat);

	/*
	 * Charging fault, dump all registers' value
	 */
	if (chg_status == RT9466_CHG_STATUS_FAULT) {
		for (i = 0; i < ARRAY_SIZE(rt9466_reg_addr); i++) {
			ret = rt9466_i2c_read_byte(info, rt9466_reg_addr[i]);
			if (ret < 0)
				return ret;
		}
	}

	dev_info(info->dev,
		"%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA\n",
		__func__, ichg / 1000, aicr / 1000, mivr / 1000, ieoc / 1000);

	dev_info(info->dev, "%s: VSYS = %dmV, VBAT = %dmV, IBAT = %dmA\n",
		__func__, adc_vsys / 1000, adc_vbat / 1000, adc_ibat / 1000);

	dev_info(info->dev, "%s: CHG_EN = %d, CHG_STATUS = %s\n",
		__func__, chg_en, rt9466_chg_status_name[chg_status]);

	return ret;
}



static struct charger_ops rt9466_chg_ops = {
	/* Normal charging */
	.plug_in = rt9466_plug_in,
	.plug_out = rt9466_plug_out,
	.dump_registers = rt9466_dump_register,
	.enable = rt9466_enable_charging,
	.is_enabled = rt9466_is_charging_enable,
	.get_charging_current = rt9466_get_ichg,
	.set_charging_current = rt9466_set_ichg,
	.get_input_current = rt9466_get_aicr,
	.set_input_current = rt9466_set_aicr,
	.get_constant_voltage = rt9466_get_cv,
	.set_constant_voltage = rt9466_set_cv,
	.kick_wdt = rt9466_kick_wdt,
	.set_mivr = rt9466_set_mivr,
	.is_charging_done = rt9466_is_charging_done,
	.get_min_charging_current = rt9466_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = rt9466_enable_safety_timer,
	.is_safety_timer_enabled = rt9466_is_safety_timer_enable,

	/* Power path */
	.enable_powerpath = rt9466_enable_power_path,
	.is_powerpath_enabled = rt9466_is_power_path_enable,

	/* OTG */
	.enable_otg = rt9466_enable_otg,
	.set_boost_current_limit = rt9466_set_boost_current_limit,
	.enable_discharge = rt9466_enable_discharge,

	/* PE+/PE+20 */
	.send_ta_current_pattern = rt9466_set_pep_current_pattern,
	.set_pe20_efficiency_table = rt9466_set_pep20_efficiency_table,
	.send_ta20_current_pattern = rt9466_set_pep20_current_pattern,
	.set_ta20_reset = rt9466_set_pep20_reset,
	.enable_cable_drop_comp = rt9466_enable_cable_drop_comp,

	/* ADC */
	.get_tchg_adc = rt9466_get_tchg,
};

/* ========================= */
/* I2C driver function       */
/* ========================= */

static int rt9466_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct rt9466_info *info = NULL;

	pr_info("%s(%s)\n", __func__, RT9466_DRV_VERSION);

	info = devm_kzalloc(&client->dev, sizeof(struct rt9466_info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	mutex_init(&info->i2c_access_lock);
	mutex_init(&info->adc_access_lock);
	mutex_init(&info->adcflag_access_lock);
	mutex_init(&info->irq_access_lock);
	mutex_init(&info->aicr_access_lock);
	mutex_init(&info->ichg_access_lock);
	mutex_init(&info->hidden_mode_lock);
	mutex_init(&info->pe_access_lock);
	info->client = client;
	info->dev = &client->dev;
	info->aicr_limit = -1;
	info->hidden_mode_cnt = 0;

	/* Init wait queue head */
	init_waitqueue_head(&info->wait_queue);

	/* Create single thread workqueue */
	info->aicl_wq = create_singlethread_workqueue("aicl_wq");
	if (!info->aicl_wq) {
		ret = -ENOMEM;
		goto err_create_wq;
	}

	/* Init AICL workqueue */
	INIT_WORK(&info->aicl_work, rt9466_aicl_work_handler);

	/* Is HW exist */
	if (!rt9466_is_hw_exist(info)) {
		dev_err(info->dev, "%s: no rt9466 exists\n", __func__);
		ret = -ENODEV;
		goto err_no_dev;
	}
	i2c_set_clientdata(client, info);

	ret = rt9466_parse_dt(info, &client->dev);
	if (ret < 0) {
		dev_err(info->dev, "%s: parse dt failed\n", __func__);
		goto err_parse_dt;
	}

#ifdef CONFIG_RT_REGMAP
	ret = rt9466_register_rt_regmap(info);
	if (ret < 0)
		goto err_register_regmap;
#endif

	/* Reset chip */
	ret = rt9466_reset_chip(info);
	if (ret < 0) {
		dev_err(info->dev, "%s: reset chip failed\n", __func__);
		goto err_reset_chip;
	}

	ret = rt9466_init_setting(info);
	if (ret < 0) {
		dev_err(info->dev, "%s: init setting failed\n", __func__);
		goto err_init_setting;
	}

	/* Register charger device */
	info->chg_dev = charger_device_register(
		info->desc->chg_dev_name, &client->dev, info, &rt9466_chg_ops,
		&info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		ret = PTR_ERR(info->chg_dev);
		goto err_register_chg_dev;
	}

	ret = rt9466_irq_register(info);
	if (ret < 0) {
		dev_err(info->dev, "%s: irq register failed\n", __func__);
		goto err_irq_register;
	}

	ret = rt9466_irq_init(info);
	if (ret < 0) {
		dev_err(info->dev, "%s: irq init failed\n", __func__);
		goto err_irq_init;
	}

	rt9466_dump_register(info->chg_dev);

	dev_info(info->dev, "%s: successfully\n", __func__);
	return ret;

err_irq_init:
err_irq_register:
	charger_device_unregister(info->chg_dev);
err_register_chg_dev:
err_init_setting:
err_reset_chip:
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(info->regmap_dev);
err_register_regmap:
#endif
err_parse_dt:
err_no_dev:
	destroy_workqueue(info->aicl_wq);
err_create_wq:
	mutex_destroy(&info->i2c_access_lock);
	mutex_destroy(&info->adc_access_lock);
	mutex_destroy(&info->adcflag_access_lock);
	mutex_destroy(&info->irq_access_lock);
	mutex_destroy(&info->aicr_access_lock);
	mutex_destroy(&info->ichg_access_lock);
	mutex_destroy(&info->hidden_mode_lock);
	mutex_destroy(&info->pe_access_lock);
	return ret;
}


static int rt9466_remove(struct i2c_client *client)
{
	int ret = 0;
	struct rt9466_info *info = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);

	if (info) {
#ifdef CONFIG_RT_REGMAP
		rt_regmap_device_unregister(info->regmap_dev);
#endif
		mutex_destroy(&info->i2c_access_lock);
		mutex_destroy(&info->adc_access_lock);
		mutex_destroy(&info->adcflag_access_lock);
		mutex_destroy(&info->irq_access_lock);
		mutex_destroy(&info->aicr_access_lock);
		mutex_destroy(&info->ichg_access_lock);
		mutex_destroy(&info->hidden_mode_lock);
		mutex_destroy(&info->pe_access_lock);
	}

	return ret;
}

static void rt9466_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct rt9466_info *info = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);
	if (info) {
		ret = rt9466_sw_reset(info);
		if (ret < 0)
			pr_err("%s: sw reset failed\n", __func__);
	}
}

static const struct i2c_device_id rt9466_i2c_id[] = {
	{"rt9466", 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id rt9466_of_match[] = {
	{ .compatible = "richtek,rt9466", },
	{},
};
#else /* Not define CONFIG_OF */

#define RT9466_BUSNUM 1

static struct i2c_board_info rt9466_i2c_board_info __initdata = {
	I2C_BOARD_INFO("rt9466", RT9466_SALVE_ADDR)
};
#endif /* CONFIG_OF */


static struct i2c_driver rt9466_i2c_driver = {
	.driver = {
		.name = "rt9466",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt9466_of_match,
#endif
	},
	.probe = rt9466_probe,
	.remove = rt9466_remove,
	.shutdown = rt9466_shutdown,
	.id_table = rt9466_i2c_id,
};

static int __init rt9466_init(void)
{
	int ret = 0;

#ifdef CONFIG_OF
	pr_info("%s: with dts\n", __func__);
#else
	pr_info("%s: without dts\n", __func__);
	i2c_register_board_info(RT9466_BUSNUM, &rt9466_i2c_board_info, 1);
#endif

	ret = i2c_add_driver(&rt9466_i2c_driver);
	if (ret < 0)
		pr_err("%s: register i2c driver failed\n", __func__);

	return ret;
}
module_init(rt9466_init);


static void __exit rt9466_exit(void)
{
	i2c_del_driver(&rt9466_i2c_driver);
}
module_exit(rt9466_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("RT9466 Charger Driver");
MODULE_VERSION(RT9466_DRV_VERSION);

/*
 * Release Note
 * 1.0.9
 * (1) Prevent backboot
 * (2) Set secondary chg to HZ in plug out callback
 * (3) Add CEB pin control for secondary charger
 * (4) After PE pattern -> Enable skip mode
 *     Disable skip mode -> Start PE pattern
 *
 * 1.0.8
 * (1) Modify init sequence for init_irq
 * (2) Add enable_cable_drop_com, is_charging_enabled, get_min_ichg OPS
 * (3) Set secondary chg to HZ if it is not in charging mode
 *
 * 1.0.7
 * (1) Modify IBAT/IBUS ADC's coefficient
 * (2) Add interrupt-names in dts property to represent those evts that need to
 *     be unmasked
 * (3) Add IRQ handlers to handle each irq event separately
 * (4) Use dev_xxx instead of pr_xxx
 * (5) Modify some naming(bat_voreg -> cv, iin_vth -> aicl_vth) and remove some
 *     unnecessary code
 *
 * 1.0.6
 * (1) Adapt to GM30
 * (2) Modify irq init value
 *
 * 1.0.5
 * (1) Disable all irq in irq_init
 * (2) Modify rt9466_is_hw_exist, check vendor id and revision id separately
 *
 * 1.0.4
 * (1) Not to unmask the plug-in/out related IRQs in irq_init
 * PWR_RDYM/CHG_MIVRM/CHG_AICRM
 * VBUSOVM
 * TS_BAT_HOTM/TS_BAT_WARMM/TS_BAT_COOLM/TS_BAT_COLDM
 * CHG_OTPM/CHG_RVPM/CHG_ADPBADM/CHG_STATCM/CHG_FAULTM/TS_STATCM
 * IEOCM/TERMM/SSFINISHM/SSFINISHM/AICLMeasM
 * BST_BATUVM/PUMPX_DONEM/ADC_DONEM
 *
 * 1.0.3
 * (1) Copy default dts value before parsing dts
 * (2) Release rt_charger_sw_reset interface
 * (3) Add chip revision E4 (0x84)
 */
