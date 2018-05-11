/*
 *  lge_battery_id.c
 *
 *  LGE Battery Charger Interface Driver
 *
 *  Copyright (C) 2011 LG Electronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <linux/power/lge_battery_id.h>
#include <soc/qcom/smem.h>

#define MODULE_NAME "lge_battery_id"

#ifdef CONFIG_LGE_PM_CABLE_DETECTION
#include <soc/qcom/lge/lge_cable_detection.h>
#endif
#if defined(CONFIG_MACH_MSM8940_TF8_TMO_US) || defined(CONFIG_LGE_PM_CABLE_DETECTION)
#include <soc/qcom/lge/board_lge.h>
#endif

struct lge_battery_id_info {
	struct device          *dev;
	uint                    batt_info_from_smem;
	bool                    enabled;
	const char              *default_battery;
	struct power_supply     psy_batt_id;
};

static enum power_supply_property lge_battery_id_battery_props[] = {
	POWER_SUPPLY_PROP_BATTERY_ID,
};

struct batt_id_info battery_id_list;

static bool is_battery_valid(uint batt_id)
{
#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER_OLD)
	if(batt_id == BATT_ID_DS2704_N || batt_id == BATT_ID_DS2704_L ||
		batt_id == BATT_ID_DS2704_C || batt_id == BATT_ID_ISL6296_N ||
		batt_id == BATT_ID_ISL6296_L || batt_id == BATT_ID_ISL6296_C ||
		batt_id == BATT_ID_RA4301_VC0 || batt_id == BATT_ID_RA4301_VC1 ||
		batt_id == BATT_ID_RA4301_VC2 || batt_id == BATT_ID_SW3800_VC0 ||
		batt_id == BATT_ID_SW3800_VC1 || batt_id == BATT_ID_SW3800_VC2 ||
		batt_id == BATT_ID_DEFAULT)
		return true;
#elif defined(CONFIG_MACH_MSM8917_SF317_CRK_US) || defined(CONFIG_MACH_MSM8917_SF317_TRF_US) || defined(CONFIG_MACH_MSM8917_SF317_TRF_US_VZW) \
|| defined(CONFIG_MACH_MSM8940_SF3_MPCS_US) || defined(CONFIG_MACH_MSM8940_SF3_SPR_US) || defined(CONFIG_MACH_MSM8940_SF3_TMO_US) || defined(CONFIG_MACH_MSM8940_SF3_GLOBAL_CA)
	if(batt_id == BATT_ID_RA4301_VC1 || batt_id == BATT_ID_SW3800_VC0 ||
		batt_id == BATT_ID_SW3800_VC1 || batt_id == BATT_ID_ISL6296_C ||
		batt_id == BATT_ID_DEFAULT)
		return true;
#else
	if(batt_id == BATT_ID_ISL6296A_N || batt_id == BATT_ID_ISL6296A_L ||
		batt_id == BATT_ID_ISL6296A_C || batt_id == BATT_ID_RA4301_VC0 ||
		batt_id == BATT_ID_SW3800_VC0 || batt_id == BATT_ID_SW3800_VC1 ||
		batt_id == BATT_ID_DEFAULT)
		return true;
#endif
	else
		return false;
}

bool lge_battery_check()
{
	struct power_supply *psy;
	union power_supply_propval prop = {0,};
	uint battery_id;
	int usb_online;
#ifdef CONFIG_LGE_PM_CABLE_DETECTION
	enum lge_boot_mode_type boot_mode;
#endif

	psy = power_supply_get_by_name("battery_id");
	if(psy) {
		psy->get_property(psy, POWER_SUPPLY_PROP_BATTERY_ID, &prop);
		battery_id = prop.intval;
	} else {
		pr_info("battery_id not found. use default battey \n");
		battery_id = BATT_ID_DEFAULT;
	}

	psy = power_supply_get_by_name("usb");
	if(psy) {
		psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &prop);
		usb_online = prop.intval;
	} else {
		pr_info("usb not found. usb online is 0.\n");
		usb_online = 0;
	}

#ifdef CONFIG_LGE_PM_CABLE_DETECTION
	boot_mode = lge_get_boot_mode();

	if(boot_mode == LGE_BOOT_MODE_QEM_56K || boot_mode == LGE_BOOT_MODE_QEM_130K || boot_mode == LGE_BOOT_MODE_QEM_910K)
		return is_battery_valid(battery_id);

	if(lge_is_factory_cable() && usb_online)
		return true;
	else
#endif
		return is_battery_valid(battery_id);
}

/*
 * TBD : This function should be more intelligent.
 * Should directly access battery id circuit via 1-Wired.
 */
static int lge_battery_id_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lge_battery_id_info *info = container_of(psy,
			struct lge_battery_id_info,
			psy_batt_id);

	switch (psp) {
	case POWER_SUPPLY_PROP_BATTERY_ID:
		val->intval = info->batt_info_from_smem;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct of_device_id lge_battery_id_match_table[] = {
	{ .compatible = "lge,battery-id" },
	{}
};

static int lge_battery_id_dt_to_pdata(struct platform_device *pdev,
					struct lge_battery_id_info *pdata)
{
	int i;
	int rc;
	unsigned int battery_num;
	u32 battery_id[BATT_ID_LIST_MAX];
	const char *battery_cell_type[BATT_ID_LIST_MAX];

	struct device_node *node = pdev->dev.of_node;

	pdata->enabled = of_property_read_bool(node,
					"lge,restrict-mode-enabled");
	rc = of_property_read_string(node,
		"lge,default-battery", &pdata->default_battery);

	if (rc) {
		pr_err("Invalid lge,default-battery.\n");
		pdata->default_battery = BATT_ID_DEFAULT_TYPE_NAME;
	}

	rc = of_property_read_u32(node, "lge,battery-num", &battery_num);
	pr_info("[FG_EST] battery num %d\n",battery_num);

	rc = of_property_read_u32_array(node, "lge,battery-id-num", battery_id ,battery_num);

	for (i = 0; i < battery_num; i++) {
		battery_id_list.batt_id[i] = battery_id[i];
		pr_info("[FG_EST] battery array 1 index = [%d] id = [%d]\n", i, battery_id[i]);
	}

	rc = of_property_read_string_array(node, "lge,battery-profile-name", battery_cell_type, battery_num);

	for (i = 0; i < battery_num; i++) {
		battery_id_list.batt_type[i] = (char *)battery_cell_type[i];
	}

	for (i = 0; i < battery_num; i++) {
		pr_info("[FG_EST] battery cell type [%s]\n", battery_id_list.batt_type[i] );
	}

	return 0;
}

static int lge_battery_id_probe(struct platform_device *pdev)
{
	struct lge_battery_id_info *info;
	/*struct power_supply *psy;*/
	int ret = 0;
	int i = 0;
	uint *smem_batt = 0;
	uint _smem_batt_id = 0;

	dev_err(&pdev->dev, "LGE Battery ID Checker started\n");

	info = kzalloc(sizeof(struct lge_battery_id_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "memory error\n");
		return -ENOMEM;
	}

	ret = lge_battery_id_dt_to_pdata(pdev, info);
	if (ret)
		return -EIO;

	platform_set_drvdata(pdev, info);

	info->dev = &pdev->dev;

	smem_batt = (uint *)smem_alloc(SMEM_BATT_INFO,
					sizeof(smem_batt),
					0,
					SMEM_ANY_HOST_FLAG);

	if (smem_batt == NULL) {
		pr_err("%s : smem_alloc returns NULL\n", __func__);
		info->batt_info_from_smem = 0;
	} else {
		_smem_batt_id = *smem_batt;
#ifdef CONFIG_MACH_MSM8940_TF8_TMO_US
		if (lge_get_board_revno() < HW_REV_A_2)
			_smem_batt_id = BATT_ID_SW3800_VC0;
#endif
		pr_info("Battery was read in sbl is = %d\n", _smem_batt_id);
		if (_smem_batt_id == BATT_ID_DEFAULT) {
			for (i = 0; i < BATT_ID_LIST_MAX;i++) {
				if (BATT_ID_DEFAULT == battery_id_list.batt_id[i]) {
					strcpy(battery_id_list.batt_type[i], info->default_battery);
					pr_debug("set default battery type : %s\n", info->default_battery);
					break;
				}
			}
		}
		/* If not 'enabled', set battery id as default */
		if (!info->enabled && _smem_batt_id == BATT_ID_UNKNOWN) {
			dev_info(&pdev->dev, "set batt_id as DEFAULT\n");
			_smem_batt_id = BATT_ID_DEFAULT;
		}
		info->batt_info_from_smem = _smem_batt_id;
	}

	info->psy_batt_id.name		= "battery_id";
	info->psy_batt_id.type		= POWER_SUPPLY_TYPE_BATTERY;
	info->psy_batt_id.get_property	= lge_battery_id_get_property;
	info->psy_batt_id.properties	= lge_battery_id_battery_props;
	info->psy_batt_id.num_properties =
		ARRAY_SIZE(lge_battery_id_battery_props);

	ret = power_supply_register(&pdev->dev, &info->psy_batt_id);

	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		goto err_register;
	}

	return ret;
err_register:
	kfree(info);
	return ret;
}

static int lge_battery_id_remove(struct platform_device *pdev)
{
	struct lge_battery_id_info *info = platform_get_drvdata(pdev);

	power_supply_unregister(&info->psy_batt_id);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM)
static int lge_battery_id_suspend(struct device *dev)
{
	/*struct lge_battery_id_info *info = dev_get_drvdata(dev);*/

	return 0;
}

static int lge_battery_id_resume(struct device *dev)
{
	/*struct lge_battery_id_info *info = dev_get_drvdata(dev);*/

	return 0;
}

static const struct dev_pm_ops lge_battery_id_pm_ops = {
	.suspend	= lge_battery_id_suspend,
	.resume		= lge_battery_id_resume,
};
#endif

static struct platform_driver lge_battery_id_driver = {
	.driver = {
		.name   = MODULE_NAME,
		.owner  = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm     = &lge_battery_id_pm_ops,
#endif
		.of_match_table = lge_battery_id_match_table,
	},
	.probe  = lge_battery_id_probe,
	.remove = lge_battery_id_remove,
};

static int __init lge_battery_id_init(void)
{
	return platform_driver_register(&lge_battery_id_driver);
}

static void __exit lge_battery_id_exit(void)
{
	platform_driver_unregister(&lge_battery_id_driver);
}

module_init(lge_battery_id_init);
module_exit(lge_battery_id_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cowboy");
MODULE_DESCRIPTION("LGE Battery ID Checker");
