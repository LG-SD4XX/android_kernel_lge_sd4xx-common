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

#ifdef CONFIG_LGE_EXTERNAL_SPEAKER
extern int max98925_get_spk_amp_status(void);
extern int max98925_get_init_status(void);
extern void max98925_spk_enable(int);
#endif
static int epack_state = 0;

void epack_audio_work_func(struct work_struct *audio_work)
{
	struct epack_dev_data *epack = container_of(to_delayed_work(audio_work), struct epack_dev_data, audio_work);

	if (epack_state) {
#ifdef CONFIG_LGE_EXTERNAL_SPEAKER
		if (max98925_get_init_status() && !max98925_get_spk_amp_status()) {
			switch_set_state(&epack->sdev, EPACK_DISABLE);
			epack_state = 0;
			pr_err("[EpackSPK][%s] state : %d, Because I2C comm. problem, Change the inter SPK.\n",__func__, get_epack_status());
		}
#endif
	}
	schedule_delayed_work(&epack->audio_work, msecs_to_jiffies(100));
	return;
}

void audio_input_init(struct i2c_client *client , struct epack_dev_data *epack)
{
	int err;
	pr_debug("[EpackSPK][%s] enter\n", __func__);
	epack->sdev.name = EPACK_STATE;
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
#ifdef CONFIG_LGE_EXTERNAL_SPEAKER
		pr_info("[EpackSPK] Call EpackSPK disable\n");
		max98925_spk_enable(0);
#endif
		epack_state = status;
		switch_set_state(&epack->sdev, EPACK_DISABLE);
	}
	pr_info("[EpackSPK][%s] sdev.name : %s, epack state: %d\n", __func__, epack->sdev.name, epack_state);
}

int audio_get_epack_state(void)
{
	return epack_state;
}