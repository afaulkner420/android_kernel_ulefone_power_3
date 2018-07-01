/*
* HALL driver
*
* Copyright (C) 2017 TRANSSION HOLDINGS
*
* Author: achang.zhang@reallytek.com
*
* This program is free software; you can redistribute  it and/or modify it
* under  the terms of  the GNU General  Public License as published by the
* Free Software Foundation;  either version 2 of the  License, or (at your
* option) any later version.
*
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <mtk_boot_common.h>

enum {
        HALL_CLOSE,
        HALL_OPEN
};

struct hall_priv {
        struct switch_dev sdev;
        struct input_dev *idev;
};

static int hall_state = HALL_OPEN;

static irqreturn_t hall_thread_factory_func(int irq_num, void *data)
{
        struct hall_priv *priv = data;
        static u8 hall_factory_status = 0;

        if (hall_state == HALL_CLOSE) {
                hall_factory_status++;
                hall_state = HALL_OPEN;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_LOW);
        } else {
                hall_factory_status++;
                hall_state = HALL_CLOSE;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_HIGH);
        }
        if(2 == hall_factory_status) {
                input_report_key(priv->idev, KEY_POWER2, 1);
                input_sync(priv->idev);
                input_report_key(priv->idev, KEY_POWER2, 0);
                input_sync(priv->idev);
                hall_factory_status = 0;
        }
        pr_info("hall_thread_factory_func hall_state = %d[%s]\n",
                        hall_state, hall_state?"HALL_OPEN":"HALL_CLOSE");

        return IRQ_HANDLED;
}

static irqreturn_t hall_thread_func(int irq_num, void *data)
{
        struct hall_priv *priv = data;

        if (hall_state == HALL_OPEN) {
                hall_state = HALL_CLOSE;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_HIGH);
                switch_set_state(&priv->sdev, hall_state);
                input_report_key(priv->idev, KEY_POWER2, 1);
                input_sync(priv->idev);
                input_report_key(priv->idev, KEY_POWER2, 0);
                input_sync(priv->idev);
        } else {
                hall_state = HALL_OPEN;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_LOW);
                switch_set_state(&priv->sdev, hall_state);
                input_report_key(priv->idev, KEY_OPTION, 1);
                input_sync(priv->idev);
                input_report_key(priv->idev, KEY_OPTION, 0);
                input_sync(priv->idev);
        }

        pr_info("hall_thread_func hall_state = %d[%s]\n",
                        hall_state, hall_state?"HALL_OPEN":"HALL_CLOSE");

        return IRQ_HANDLED;
}

static int hall_probe(struct platform_device *pdev)
{
        struct hall_priv *priv;
        struct device_node *node = pdev->dev.of_node;
        int irq_num = 0, error;

        pr_info("hall_probe enter\n");

        priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
        if (!priv) {
                dev_err(&pdev->dev, "Failed to allocate memory\n");
                return -ENOMEM;
        }

        priv->idev = devm_input_allocate_device(&pdev->dev);
        if (!priv->idev) {
                dev_err(&pdev->dev, "Failed to allocate input device\n");
                return -ENOMEM;
        }

        priv->idev->name = pdev->name;
        priv->idev->dev.parent = &pdev->dev;

        __set_bit(EV_KEY, priv->idev->evbit);
        input_set_capability(priv->idev, EV_KEY, KEY_POWER2);
        input_set_capability(priv->idev, EV_KEY, KEY_OPTION);

        error = input_register_device(priv->idev);
        if (error) {
                dev_err(&pdev->dev, "Failed to register input device\n");
                return error;
        }

        priv->sdev.name = "hall";
        error = switch_dev_register(&priv->sdev);
        if (error) {
                dev_err(&pdev->dev, "Failed to register switch device\n");
                return error;
        }
        switch_set_state(&priv->sdev, HALL_OPEN);

        if (!node) {
                dev_err(&pdev->dev, "Device_node is null\n");
                error = -ENOENT;
                goto err_unregister_sdev;
        }

        irq_num = irq_of_parse_and_map(node, 0);
        if (!irq_num) {
                dev_err(&pdev->dev, "Failed to parse and map irq\n");
                error = -EINVAL;
                goto err_unregister_sdev;
        }
        if (FACTORY_BOOT == get_boot_mode()) {
                error = devm_request_threaded_irq(&pdev->dev, irq_num, NULL,
                        hall_thread_factory_func, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
                        dev_name(&pdev->dev), priv);
        } else {
                error = devm_request_threaded_irq(&pdev->dev, irq_num, NULL,
                        hall_thread_func, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
                        dev_name(&pdev->dev), priv);
        }

        if (error) {
                dev_err(&pdev->dev, "Failed to devm_request_threaded_irq\n");
                goto err_unregister_sdev;
        }
        enable_irq_wake(irq_num);

        platform_set_drvdata(pdev, priv);

        pr_info("hall_probe ok\n");

        return 0;

err_unregister_sdev:
        switch_dev_unregister(&priv->sdev);
        return error;
}

static int hall_remove(struct platform_device *pdev)
{
        struct hall_priv *priv = platform_get_drvdata(pdev);

        switch_dev_unregister(&priv->sdev);

        return 0;
}

static const struct of_device_id hall_of_match[] = {
        { .compatible = "mediatek,hall"},
        { },
};

static struct platform_driver hall_driver = {
        .probe = hall_probe,
        .remove = hall_remove,
        .driver = {
                .name = "hall",
                .of_match_table = hall_of_match,
        },
};

module_platform_driver(hall_driver);

/* Module information */
MODULE_AUTHOR("<achang.zhang@reallytek.com>");
MODULE_DESCRIPTION("HALL driver");
MODULE_LICENSE("GPL v2");
