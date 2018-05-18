/*
 * Android GPIO Vibrator Driver
 * Copyright (c) 2017, LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include "../staging/android/timed_output.h"

#define GPIO_VIB_DEFAULT_TIMEOUT 15000

struct gpio_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;

	int gpio;
	int state;
	int timeout;
	struct mutex lock;
};

static int gpio_vib_set(struct gpio_vib *vib, int on)
{
	if (on) {
		gpio_set_value(vib->gpio, 1);
	} else {
		gpio_set_value(vib->gpio, 0);
	}
	return 0;
}

static void gpio_vib_enable(struct timed_output_dev *dev, int value)
{
	struct gpio_vib *vib = container_of(dev, struct gpio_vib,
					 timed_dev);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);

	if (value == 0) {
		vib->state = 0;
	} else {
		value = (value > vib->timeout ?
				 vib->timeout : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);
}

static void gpio_vib_update(struct work_struct *work)
{
	struct gpio_vib *vib = container_of(work, struct gpio_vib,
					 work);
	gpio_vib_set(vib, vib->state);
}

static int gpio_vib_get_time(struct timed_output_dev *dev)
{
	struct gpio_vib *vib = container_of(dev, struct gpio_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);

		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart gpio_vib_timer_func(struct hrtimer *timer)
{
	struct gpio_vib *vib = container_of(timer, struct gpio_vib,
							 vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM
static int gpio_vibrator_suspend(struct device *dev)
{
	struct gpio_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	gpio_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_vibrator_pm_ops, gpio_vibrator_suspend, NULL);

static int gpio_vib_parse_dt(struct device dev, struct gpio_vib *vib)
{
	int rc;
	u32 temp_val;

	vib->gpio = -1;
	vib->gpio = of_get_named_gpio(dev.of_node, "vib-gpio", 0);
	if (vib->gpio > 0) {
		rc = gpio_request(vib->gpio, "VIBRATOR");
		gpio_direction_output(vib->gpio, 0);
		pr_info("%s: Vibrator GPIO : %d\n", __func__, vib->gpio);
	} else {
		pr_err("%s: ERROR: Vibrator GPIO is missing!\n", __func__);
	}

	vib->timeout = GPIO_VIB_DEFAULT_TIMEOUT;
	rc = of_property_read_u32(dev.of_node,
			"vib-timeout-ms", &temp_val);
	if (!rc) {
		vib->timeout = temp_val;
	} else if (rc != -EINVAL) {
		pr_err("%s : Unable to read vib timeout\n", __func__);
		return rc;
	}

	return 0;
}

static int gpio_vibrator_probe(struct platform_device *pdev)
{
	struct gpio_vib *vib;
	int rc;

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	pr_info("%s: Enter\n", __func__);


	rc = gpio_vib_parse_dt(pdev->dev, vib);
	if (rc) {
		dev_err(&pdev->dev, "DT parsing failed\n");
		return rc;
	}

	mutex_init(&vib->lock);
	INIT_WORK(&vib->work, gpio_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = gpio_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = gpio_vib_get_time;
	vib->timed_dev.enable = gpio_vib_enable;

	platform_set_drvdata(pdev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		return rc;
	pr_info("%s: Vibrator probe success\n", __func__);
	return rc;
}

static int gpio_vibrator_remove(struct platform_device *pdev)
{
	struct gpio_vib *vib = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	mutex_destroy(&vib->lock);
	kfree(vib);

	return 0;
}

static struct of_device_id vib_match_table[] = {
	{	.compatible = "gpio-vibrator",
	},
	{}
};

static struct platform_driver gpio_vibrator_driver = {
	.driver		= {
		.name	= "gpio-vibrator",
		.of_match_table = vib_match_table,
		.pm     = &gpio_vibrator_pm_ops,
	},
	.probe		= gpio_vibrator_probe,
	.remove		= gpio_vibrator_remove,
};

static int __init gpio_vibrator_init(void)
{
	return platform_driver_register(&gpio_vibrator_driver);
}
module_init(gpio_vibrator_init);

static void __exit gpio_vibrator_exit(void)
{
	return platform_driver_unregister(&gpio_vibrator_driver);
}
module_exit(gpio_vibrator_exit);

MODULE_DESCRIPTION("gpio vibrator driver");
MODULE_LICENSE("GPL v2");