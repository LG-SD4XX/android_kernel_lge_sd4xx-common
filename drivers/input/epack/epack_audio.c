/*
 * Copyright(c) 2016, LG Electronics. All rights reserved.
 *
 * e-pack i2c device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/switch.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/epack_core.h>
#include <linux/input/epack_audio.h>

static int epack_state = 0;
static struct epack_dev_data *g_epack = NULL;

void epack_audio_work_func(struct work_struct *audio_work)
{
	struct epack_dev_data *epack = container_of(to_delayed_work(audio_work), struct epack_dev_data, audio_work);

	if (epack_state) {
		if (epack->amp_cb && epack->amp_cb->get_amp_init_status && epack->amp_cb->get_amp_current_status) {
			if (epack->amp_cb->get_amp_init_status() && !epack->amp_cb->get_amp_current_status()) {
				switch_set_state(&epack->sdev, EPACK_DISABLE);
				epack_state = 0;
				pr_err("[EpackSPK][%s] state : %d, Because I2C comm. problem, Change the inter SPK.\n",__func__, get_epack_status());
			}
		}
	}
	schedule_delayed_work(&epack->audio_work, msecs_to_jiffies(100));
	return;
}

void audio_input_init(struct i2c_client *client , struct epack_dev_data *epack)
{
	int err;
	pr_debug("[EpackSPK][%s] enter\n", __func__);
	epack->sdev.name = EPACK_STATE;
	g_epack = epack;
	err = switch_dev_register(&epack->sdev);
	if (err < 0) {
        pr_err("[EpackSPK][%s] Failed to register switch device\n", __func__);
        switch_dev_unregister(&epack->sdev);
	}
}

void audio_input_set_sdev_name(struct epack_dev_data *epack, int status)
{
	pr_debug("[EpackSPK][%s] enter\n", __func__);


	if (status) {
		epack_state = status;
		switch_set_state(&epack->sdev, EPACK_ENABLE);
	} else {
		pr_info("[EpackSPK] Call EpackSPK disable\n");
		if (epack->amp_cb && epack->amp_cb->amp_enable)
			epack->amp_cb->amp_enable(0);
		epack_state = status;
		switch_set_state(&epack->sdev, EPACK_DISABLE);
	}
	pr_info("[EpackSPK][%s] sdev.name : %s, epack state: %d\n", __func__, epack->sdev.name, epack_state);
}

int audio_get_epack_state(void)
{
	return epack_state;
}

int epack_amp_init(const struct epack_amp_cb *amp_cb)
{
	int ret = 0;
	if(g_epack)
		g_epack->amp_cb = amp_cb;
	else
		pr_err("[EpackSPK][%s] epack initialization failed\n", __func__);
	if (!amp_cb || !amp_cb->get_amp_current_status
		|| !amp_cb->get_amp_init_status || !amp_cb->amp_enable) {
			pr_err("[EpackSPK][%s] required epack callbacks are not defined\n", __func__);
			return -EINVAL;
	}
	return ret;
}
