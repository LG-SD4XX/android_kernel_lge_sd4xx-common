/* touch_model.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

int touch_get_dts_base(struct touch_core_data *ts)
{
	struct device_node *np = ts->dev->of_node;

	TOUCH_I("start dev.of_node\n");

	PROPERTY_GPIO(np, "reset-gpio", ts->reset_pin);
	PROPERTY_GPIO(np, "irq-gpio", ts->int_pin);
	PROPERTY_GPIO(np, "maker_id-gpio", ts->maker_id_pin);
	PROPERTY_U32(np, "irqflags", ts->irqflags);

	/* Caps */
	PROPERTY_U32(np, "max_x", ts->caps.max_x);
	PROPERTY_U32(np, "max_y", ts->caps.max_y);
	PROPERTY_U32(np, "max_pressure", ts->caps.max_pressure);
	PROPERTY_U32(np, "max_width_major", ts->caps.max_width_major);
	PROPERTY_U32(np, "max_width_minor", ts->caps.max_width_minor);
	PROPERTY_U32(np, "max_orientation", ts->caps.max_orientation);
	PROPERTY_U32(np, "max_id", ts->caps.max_id);
	PROPERTY_U32(np, "hw_reset_delay", ts->caps.hw_reset_delay);
	PROPERTY_U32(np, "sw_reset_delay", ts->caps.sw_reset_delay);

	/* Role */
	PROPERTY_BOOL(np, "use_lpwg", ts->role.use_lpwg);
	PROPERTY_U32(np, "use_lpwg_test", ts->role.use_lpwg_test);
	PROPERTY_BOOL(np, "use_fw_upgrade", ts->role.use_fw_upgrade);
	PROPERTY_BOOL(np, "hide_coordinate", ts->role.hide_coordinate);

	/* Power */
	PROPERTY_GPIO(np, "vdd-gpio", ts->vdd_pin);
	PROPERTY_GPIO(np, "vio-gpio", ts->vio_pin);

	/* Firmware */
	PROPERTY_STRING_ARRAY(np, "fw_image", ts->def_fwpath, ts->def_fwcnt);

	{
		int i;

		for (i = 0; i < ts->def_fwcnt; i++) {
			TOUCH_I("fw_image - %d:%s\n",
				i, ts->def_fwpath[i]);
		}
	}
	PROPERTY_STRING(np, "panel_spec", ts->panel_spec);
	PROPERTY_STRING(np, "panel_spec_mfts_folder", ts->panel_spec_mfts);
	PROPERTY_STRING(np, "panel_spec_mfts_flat", ts->panel_spec_mfts_flat);
	PROPERTY_STRING(np, "panel_spec_mfts_curved",
					ts->panel_spec_mfts_curved);
	TOUCH_I("end dev.of_node\n");

	return 0;
}

int touch_get_dts(struct touch_core_data *ts)
{
	touch_get_dts_base(ts);

	/* for model */

	return 0;
}

int __initdata touch_i2c_bus_num = 0;
int __initdata touch_spi_bus_num = 0;
int touch_get_platform_data(struct touch_core_data *ts)
{
	TOUCH_I("%s - dummy\n", __func__);
	return 0;
}


