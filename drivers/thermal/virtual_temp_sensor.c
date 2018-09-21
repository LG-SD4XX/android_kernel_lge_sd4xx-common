#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/thermal.h>
#include <linux/qpnp/qpnp-adc.h>
#ifdef CONFIG_LGE_PM_USB_CONNECTED_VTS
#include <linux/power_supply.h>
#endif

#define VTS_NAME	"vts"

struct composite_sensor {
	const char			*name;
	int				weight;
	enum qpnp_vadc_channels		channel;
	struct list_head		list;
};

struct vts_override_info {
	int				xo;
	int				quiet;
	int				cons;
};

typedef struct virtual_temp_sensor {
	const char			*name;
	struct device			*dev;
	struct thermal_zone_device	*tz_vts;
	struct qpnp_vadc_chip		*vadc_dev;
	struct composite_sensor		*sensors;
	struct vts_override_info        *over;
	u32				scaling_factor;
	int				constant;
} VTS;
LIST_HEAD(composite_sensors_head);
static char lge_hydra_mode[32] = "";
int __init lge_hydra_check(char *s)
{
	if (strlen(s)) {
		pr_info("lge.hydra is %s\n", s);
		strncpy(lge_hydra_mode,s,sizeof(lge_hydra_mode)-1);
	}
	return 0;
}
__setup("lge.hydra=", lge_hydra_check);

static int vts_tz_get_temp(struct thermal_zone_device *thermal,
				unsigned long *temp)
{
	VTS *vts = thermal->devdata;
	struct composite_sensor *sensor;
	long val = 0, xo_val = 0, quiet_val = 0;
	char *xo_therm = "xo_therm";
	char *quiet_therm = "quiet_therm";
#ifdef CONFIG_LGE_PM_USB_CONNECTED_VTS
	int usb_online = 0, rc = 0;
	struct power_supply *usb_psy;
	union power_supply_propval prop = {0,};

	usb_psy = power_supply_get_by_name("usb");
	if(usb_psy) {
		rc = usb_psy->get_property(usb_psy, POWER_SUPPLY_PROP_ONLINE, &prop);
		if (rc) {
			pr_err("usb_psy online read fail rc = %d\n", rc);
		} else {
			usb_online = prop.intval;
		}
	}
#endif

	list_for_each_entry(sensor, &composite_sensors_head, list) {
		struct qpnp_vadc_result results;
		int ret;
		ret = qpnp_vadc_read(vts->vadc_dev, sensor->channel, &results);
		if (ret) {
			pr_err("Fail to get adc(%d)\n", sensor->channel);
			return ret;
		}
		if(!strcmp(sensor->name, xo_therm))
			xo_val = sensor->weight * results.physical;

		if(!strcmp(sensor->name, quiet_therm))
			quiet_val = sensor->weight * results.physical;

		val = xo_val + quiet_val;
	}
	val += vts->constant;
	val *= vts->scaling_factor;
	val /= 1000L;

	if (val < 0 ) {
		*temp = 0;
	} else {
#ifdef CONFIG_LGE_PM_USB_CONNECTED_VTS
		if(usb_online)
			*temp = (unsigned long)val + 10;
		else
#endif
			*temp = (unsigned long)val;
	}

	return 0;
}

static int vts_override(void)
{
	VTS *vts;
	int xo_temp = 0, quiet_temp = 0;
	struct composite_sensor *sensor;
	struct thermal_zone_device *thermal;
	char *xo_therm = "xo_therm";
	char *quiet_therm = "quiet_therm";

	thermal = thermal_zone_get_zone_by_name(VTS_NAME);
	if (IS_ERR(thermal)) {
		pr_err("Failed to get thermal zone\n");
		return PTR_ERR(thermal);
	}
	vts = thermal->devdata;
	if (!vts || !vts->over) {
		pr_err("Failed to get vts override info\n");
		return -EFAULT;
	}

	if (!vts->over->xo || !vts->over->quiet) {
		pr_err("Failed to set vts: x[%d] q[%d] c[%d]\n",
		       vts->over->xo, vts->over->quiet, vts->over->cons);
		return -EINVAL;
	}
	list_for_each_entry(sensor, &composite_sensors_head, list) {
		if (!strcmp(sensor->name, xo_therm)) {
			xo_temp = sensor->weight;
			sensor->weight = vts->over->xo;
		}
		if (!strcmp(sensor->name, quiet_therm)) {
			quiet_temp = sensor->weight;
			sensor->weight = vts->over->quiet;
		}
	}
	if (!xo_temp || !quiet_temp) {
		pr_err("Restore vts to original value\n");
		list_for_each_entry(sensor, &composite_sensors_head, list) {
			if (!strcmp(sensor->name, xo_therm)) {
				sensor->weight = xo_temp;
			}
			if (!strcmp(sensor->name, quiet_therm)) {
				sensor->weight = quiet_temp;
			}
		}
	} else {
		vts->constant = vts->over->cons;
		pr_info("vts x[%d] q[%d] c[%d]\n",
			vts->over->xo, vts->over->quiet, vts->over->cons);
	}

	return 0;
}

static ssize_t override_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	VTS *vts;
	struct thermal_zone_device *thermal;
	int xo, quiet, cons;

	if (sscanf(buf, "%d %d %d", &xo, &quiet, &cons) < 1) {
		pr_err("Failed to store vts override\n");
		return -EINVAL;
	}

	thermal = thermal_zone_get_zone_by_name(VTS_NAME);
	if (IS_ERR(thermal)) {
		pr_err("Failed to get thermal zone\n");
		return PTR_ERR(thermal);
	}
	vts = thermal->devdata;
	if (!vts || !vts->over) {
		pr_err("Failed to get vts override info\n");
		return -EFAULT;
	}
	vts->over->xo = xo;
	vts->over->quiet = quiet;
	vts->over->cons = cons;
	vts_override();

	return count;
}

static ssize_t override_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	int ret = 0;
	VTS *vts;
	struct thermal_zone_device *thermal;

	thermal = thermal_zone_get_zone_by_name(VTS_NAME);
	if (IS_ERR(thermal)) {
		pr_err("Failed to get thermal zone\n");
		return PTR_ERR(thermal);
	}
	vts = thermal->devdata;
	if (!vts || !vts->over) {
		pr_err("Failed to get vts override info\n");
		return -EFAULT;
	}
	ret = snprintf(buf, PAGE_SIZE, "%d-%d-%d\n",
		       vts->over->xo, vts->over->quiet, vts->over->cons);
	return ret;
}

static DEVICE_ATTR(override, S_IRUGO | S_IWUSR, override_show, override_store);

static struct thermal_zone_device_ops vts_thermal_zone_ops = {
	.get_temp = vts_tz_get_temp,
};

static int vts_probe(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *child;
	VTS *vts = kmalloc(sizeof(VTS), GFP_KERNEL);
	int ret = 0;
	int count = 0;
	static bool is_vts_node = false;
	struct composite_sensor *_sensor, *temp;

	/* Alloc chipset data on memory */
	pr_info("vts_probe start\n");
	if (!vts) {
		pr_err("Fail to get *vts.\n");
		return -ENOMEM;
	}

	/* Get vts name from device tree.
	   Set lge, name property if you want to change the name */
	if (of_property_read_string(of_node, "lge,name", &vts->name))
		vts->name = "vts";
	vts->dev = &pdev->dev;
	vts->vadc_dev = qpnp_get_vadc(vts->dev, "vts");

	if (strlen(lge_hydra_mode)) {
		if (strncmp(lge_hydra_mode, vts->name, sizeof(lge_hydra_mode)-1) && strncmp(VTS_NAME, vts->name, sizeof(VTS_NAME)-1)) {
			pr_info("%s: probe skipped.\n", vts->name);
			kfree(vts);
			return 0;
		}
		if (is_vts_node) {
			pr_info("%s: probe skipped.\n", vts->name);
			kfree(vts);
			return 0;
		}
		is_vts_node = 1;
	} else {
		if (strncmp(VTS_NAME, vts->name, sizeof(VTS_NAME)-1)) {
			pr_info("%s: probe skipped.\n", vts->name);
			kfree(vts);
			return 0;
		}
	}

	if (IS_ERR(vts->vadc_dev)) {
		ret = PTR_ERR(vts->vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("Fail to get vadc.\n");
		goto fail;
	}
	vts->tz_vts = thermal_zone_device_register(
				VTS_NAME, 0, 0, vts,
				&vts_thermal_zone_ops, NULL, 0, 0);
	if (IS_ERR(vts->tz_vts)) {
		ret = PTR_ERR(vts->tz_vts);
		if (ret != -EPROBE_DEFER)
			pr_err("%s: Fail to get thermal_zone_device.\n", vts->name);
		goto fail;
	}

	/* Get infos from device tree */
	if (of_property_read_u32(of_node, "lge,scaling-factor", &vts->scaling_factor))
		vts->scaling_factor = 1;
	if (of_property_read_s32(of_node, "lge,constant", &vts->constant))
		vts->constant = 0;

	for_each_child_of_node(of_node, child) {
		struct composite_sensor *sensor = kmalloc(
				sizeof(struct composite_sensor), GFP_KERNEL);
		if (!sensor) {
			ret = PTR_ERR(sensor);
			pr_err("%s: Fail to malloc sensor.\n", vts->name);
			goto fail;
		}
		if (of_property_read_string(child, "label", &sensor->name)) {
			kfree(sensor);
			continue;
		}
		if (of_property_read_u32(child, "channel", &sensor->channel)) {
			kfree(sensor);
			continue;
		}
		if (of_property_read_s32(child, "weight", &sensor->weight)) {
			kfree(sensor);
			continue;
		}

		if (of_property_read_bool(child,"weight-negative")) {
			sensor->weight *= -1;
		}

		pr_info("%s is registered. chan=%d, weight=%d, constant=%d\n",
			sensor->name, sensor->channel, sensor->weight, vts->constant);
		INIT_LIST_HEAD(&sensor->list);
		list_add_tail(&sensor->list, &composite_sensors_head);
		count++;
	}
	pr_info("%s: Add %d sensors for virtual temp sensor\n", vts->name, count);

	vts->over = kmalloc(sizeof(struct vts_override_info), GFP_KERNEL);
	if (vts->over) {
		memset(vts->over, 0, sizeof(struct vts_override_info));
		device_create_file(&vts->tz_vts->device, &dev_attr_override);
	} else {
		pr_err("Fail to allocate memory for vts_override\n");
	}
	platform_set_drvdata(pdev, vts);
	pr_info("%s: probe done.\n", vts->name);
	return 0;
fail:
	pr_info("Fail to register vts\n");
	list_for_each_entry_safe(_sensor, temp, &composite_sensors_head, list) {
		list_del(&_sensor->list);
		kfree(_sensor);
	}
	vts->vadc_dev = NULL;
	thermal_zone_device_unregister(vts->tz_vts);
	platform_set_drvdata(pdev, NULL);
	kfree(vts->over);
	kfree(vts);
	return ret;
}

static int vts_remove(struct platform_device *pdev)
{
	VTS *vts = platform_get_drvdata(pdev);
	struct composite_sensor *sensor, *temp;
	thermal_zone_device_unregister(vts->tz_vts);
	list_for_each_entry_safe(sensor, temp, &composite_sensors_head, list) {
		list_del(&sensor->list);
		kfree(sensor);
	}
	platform_set_drvdata(pdev, NULL);
	kfree(vts);
	return 0;
}

static const struct of_device_id vts_match[] = {
	{ .compatible = "lge,vts", },
	{}
};

static struct platform_driver vts_driver = {
	.probe = vts_probe,
	.remove = vts_remove,
	.driver = {
		.name = "vts",
		.owner = THIS_MODULE,
		.of_match_table = vts_match,
	},
};

static int __init vts_init_driver(void)
{
	return platform_driver_register(&vts_driver);
}
late_initcall(vts_init_driver);

static void __exit vts_exit_driver(void)
{
	return platform_driver_unregister(&vts_driver);
}
module_exit(vts_exit_driver);

MODULE_DESCRIPTION("Virtual temperature sensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yongkwan Kim <yongk.kim@lge.com>");
