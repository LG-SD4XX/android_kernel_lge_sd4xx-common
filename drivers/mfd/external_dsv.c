/*
 * External DSV  MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <linux/mfd/external_dsv.h>

#define DRIVER_NAME "ext_dsv"

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "lge."

#define EXT_DSV_MODE_LEN 3
#define EXT_DSV_NAME_LEN 256

static char ext_dsv_cfg[EXT_DSV_NAME_LEN];

struct mode_entry {
	char name[64];
	u32 *mode_reg;
	u32 mode_reg_len;
};

struct ext_dsv_platform_data {
	u32 num_modes;
	struct mode_entry *mode_array;
	int access_level;
	int ena_gpio;
};

struct ext_dsv {
	struct device *dev;
	struct regmap *regmap;
	struct ext_dsv_platform_data *pdata;
};

static struct mfd_cell ext_dsv_devs[] = {
	{ .name = "external_dsv" },
};

static struct ext_dsv *ext_dsv_base;

static inline struct ext_dsv_platform_data *get_ext_dsv_platform_data(struct i2c_client *cl)
{
	struct ext_dsv *tmp;

	tmp = (struct ext_dsv *)i2c_get_clientdata(cl);
	if (!tmp)
		return NULL;

	return tmp->pdata;
}

int ext_dsv_set_access_level(int level)
{
	if (ext_dsv_base == NULL) {
		pr_err("%s: %s is not probed\n", __func__, DRIVER_NAME);
		return -ENODEV;
	}
	if (ext_dsv_base->pdata == NULL) {
		pr_err("%s: no platform data\n", __func__);
		return -EINVAL;
	}
	ext_dsv_base->pdata->access_level = level;
	return 0;
}
EXPORT_SYMBOL_GPL(ext_dsv_set_access_level);

int ext_dsv_register_set(u8 address, u8 value)
{
	struct i2c_client *cl;
	int ret = 0;

	if (ext_dsv_base == NULL || ext_dsv_base->dev == NULL) {
		pr_err("%s: %s is not probed\n", __func__, DRIVER_NAME);
		return -ENODEV;
	}

	cl = container_of(ext_dsv_base->dev, struct i2c_client, dev);
	if (cl == NULL) {
		pr_err("%s: invalid i2c client address \n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(cl, address, value);
	if(ret < 0)
		pr_err("%s: failed to set address %d\n", __func__, address);

	return ret;
}
EXPORT_SYMBOL_GPL(ext_dsv_register_set);

static int find_mode_index_by_name(struct ext_dsv_platform_data *pdata, const char *name)
{
	int i, index = -1;

	for (i = 0; i < pdata->num_modes; ++i) {
		if (!strcmp(pdata->mode_array[i].name, name)) {
			index = i;
			break;
		}
	}
	return index;
}

int ext_dsv_mode_change_privileged(const char* name)
{
	int ret = 0, i, index;

	struct i2c_client *cl;
	struct ext_dsv_platform_data *pdata;

	if (ext_dsv_base == NULL || ext_dsv_base->dev == NULL) {
		pr_err("ext_dsv_mode_change: %s is not probed\n", DRIVER_NAME);
		return -ENODEV;
	}

	cl = container_of(ext_dsv_base->dev, struct i2c_client, dev);
	if (cl == NULL)
		return -EINVAL;

	pdata = get_ext_dsv_platform_data(cl);
	if (pdata == NULL)
		return -EINVAL;

	index = find_mode_index_by_name(pdata, name);
	if (index == -1) {
		pr_err("ext_dsv_mode_change: %s not defined\n", name);
		return -EINVAL;
	}

	pr_info("ext_dsv_mode_change: %s\n", pdata->mode_array[index].name);

	for (i = 0; i < pdata->mode_array[index].mode_reg_len; i+=3) {
		ret += i2c_smbus_write_byte_data(cl, pdata->mode_array[index].mode_reg[i], pdata->mode_array[index].mode_reg[i+1]);
		usleep_range(pdata->mode_array[index].mode_reg[i+2] * 1000, pdata->mode_array[index].mode_reg[i+2] * 1000);
		pr_info("%s: DSV addr(0x%x),value(0x%x),delay(%dms)\n", __func__,
				pdata->mode_array[index].mode_reg[i], pdata->mode_array[index].mode_reg[i+1], pdata->mode_array[index].mode_reg[i+2]);
	}
	if (ret > 0)
		pr_err("ext_dsv_mode_changed: i2c write failed. ret=%d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(ext_dsv_mode_change_privileged);

int ext_dsv_mode_change_unprivileged(const char* name)
{
	if (ext_dsv_base == NULL || ext_dsv_base->dev == NULL) {
		pr_err("%s: %s is not probed\n", __func__, DRIVER_NAME);
		return -ENODEV;
	}
	if (ext_dsv_base->pdata == NULL) {
		pr_err("%s: no platform data\n", __func__);
		return -EINVAL;
	}
	if (ext_dsv_base->pdata->access_level > EXT_DSV_ACCESS_LEVEL_UNPRIVILEGED) {
		pr_err("%s: operation not permitted\n", __func__);
		return -EPERM;
	}
	return ext_dsv_mode_change_privileged(name);
}
EXPORT_SYMBOL_GPL(ext_dsv_mode_change_unprivileged);

int ext_dsv_chip_enable(int enable)
{
	struct ext_dsv_platform_data *pdata;
	int rc;

	if (ext_dsv_base == NULL || ext_dsv_base->dev == NULL) {
		pr_err("%s: %s is not probed\n", __func__, DRIVER_NAME);
		return -ENODEV;
	}
	if (ext_dsv_base->pdata == NULL) {
		pr_err("%s: no platform data\n", __func__);
		return -EINVAL;
	}
	pdata = ext_dsv_base->pdata;
	if (!gpio_is_valid(pdata->ena_gpio))
		return -EINVAL;
	rc = gpio_request(pdata->ena_gpio, "ext_dsv_ena");
	if (rc) {
		pr_err("%s: resquest ext_dsv_ena gpio failed\n", __func__);
		return rc;
	}
	pr_info("%s: ext_dsv_ena gpio set to %d\n", __func__, enable);
	gpio_set_value(pdata->ena_gpio, enable?1:0);
	gpio_free(pdata->ena_gpio);

	usleep_range(3000, 3000);

	return rc;
}
EXPORT_SYMBOL_GPL(ext_dsv_chip_enable);

static struct regmap_config ext_dsv_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX_REGISTERS,
};

static void parse_ext_dsv_mode_reg_dt(struct device_node *np, char *name, u32 **mode_reg, u32 *mode_reg_len)
{
	int num = 0, rc;
	struct property *data;
	u32 *array;

	*mode_reg_len = 0;

	data = of_find_property(np, name, &num);
	num /= sizeof(u32);

	array = kzalloc(sizeof(u32) * num, GFP_KERNEL);

	if (!array)
		return;

	if (!data || !num || num % EXT_DSV_MODE_LEN) {
		pr_err("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
		goto error;
	} else {
		rc = of_property_read_u32_array(np, name, array, num);

		if (rc) {
			pr_err("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
			goto error;
		}

		*mode_reg = array;
		*mode_reg_len = num;
		pr_debug("%s: %s len=%d\n", __func__, name, num);
	}

	return;
error:
	kfree(array);
}

static void parse_ext_dsv_dt(struct device_node *node, struct ext_dsv_platform_data *pdata)
{
	struct device_node *dsv_node;
	int i, rc;
	const char *name;

	pr_info("%s: %s\n", __func__, ext_dsv_cfg);
	dsv_node = of_find_node_by_name(node, ext_dsv_cfg);
	if (!dsv_node) {
		pr_debug("%s: not found ext_dsv_cfg, use primary\n", __func__);
		dsv_node = of_parse_phandle(node, "lge,ext_dsv_primary", 0);
		if (!dsv_node) {
			pr_err("%s: can't find external dsv phandle\n", __func__);
			return;
		}
	}

	rc = of_property_count_strings(dsv_node, "mode-names");
	if (rc > 0) {
		pdata->num_modes = rc;
		pr_info("%s: num_mode_names=%d\n", __func__, rc);
		pdata->mode_array = kmalloc(sizeof(struct mode_entry)*pdata->num_modes, GFP_KERNEL);
		if (NULL == pdata->mode_array) {
			pdata->num_modes = 0;
			return;
		}
		for (i = 0; i < pdata->num_modes; ++i) {
			of_property_read_string_index(dsv_node, "mode-names", i, &name);
			snprintf(pdata->mode_array[i].name, sizeof(pdata->mode_array[i].name), "%s", name);
			parse_ext_dsv_mode_reg_dt(dsv_node, pdata->mode_array[i].name, &(pdata->mode_array[i].mode_reg), &(pdata->mode_array[i].mode_reg_len));
		}
	} else {
		pdata->num_modes = 0;
	}

	pdata->ena_gpio = of_get_named_gpio(node, "lge,gpio-dsv_ena", 0);
	if (!gpio_is_valid(pdata->ena_gpio))
		pr_err("%s: ena_gpio not specified\n", __func__);
	else
		pr_info("%s: ena_gpio = %d\n", __func__, pdata->ena_gpio);
}

static int ext_dsv_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct ext_dsv *ext_dsv;
	struct device *dev = &cl->dev;
	struct ext_dsv_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	pdata = devm_kzalloc(dev, sizeof(struct ext_dsv_platform_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	pdata->access_level = EXT_DSV_ACCESS_LEVEL_UNPRIVILEGED;

	ext_dsv = devm_kzalloc(dev, sizeof(struct ext_dsv), GFP_KERNEL);
	if (!ext_dsv)
		return -ENOMEM;

	ext_dsv->pdata = pdata;

	ext_dsv->regmap = devm_regmap_init_i2c(cl, &ext_dsv_regmap_config);
	if (IS_ERR(ext_dsv->regmap)) {
		pr_err("Failed to allocate register map\n");
		devm_kfree(dev, ext_dsv);
		return PTR_ERR(ext_dsv->regmap);
	}

	ext_dsv->dev = &cl->dev;
	i2c_set_clientdata(cl, ext_dsv);
	ext_dsv_base = ext_dsv;

	rc = mfd_add_devices(dev, -1, ext_dsv_devs, ARRAY_SIZE(ext_dsv_devs),
			       NULL, 0, NULL);
	if (rc) {
		pr_err("Failed to add external_dsv subdevice ret=%d\n", rc);
		return -ENODEV;
	}
	parse_ext_dsv_dt(dev->of_node, ext_dsv->pdata);

	pr_info("%s: done \n", __func__);
	return rc;
}

static int ext_dsv_remove(struct i2c_client *cl)
{
	struct ext_dsv *ext_dsv = i2c_get_clientdata(cl);

	mfd_remove_devices(ext_dsv->dev);

	return 0;
}

static const struct i2c_device_id ext_dsv_ids[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ext_dsv_ids);

#ifdef CONFIG_OF
static const struct of_device_id ext_dsv_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, ext_dsv_of_match);
#endif

static struct i2c_driver ext_dsv_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ext_dsv_of_match),
#endif
	},
	.id_table = ext_dsv_ids,
	.probe = ext_dsv_probe,
	.remove = ext_dsv_remove,
};
module_i2c_driver(ext_dsv_driver);

MODULE_DESCRIPTION("external_dsv MFD Core");


module_param_string(ext_dsv, ext_dsv_cfg, EXT_DSV_NAME_LEN, 0);
MODULE_PARM_DESC(ext_dsv, "panel=<external dsv device name> ");
