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
#include <linux/rtc.h>

#include <linux/input/epack_core.h>
#include "cmd.h"

static bool log_enable = 0;
module_param(log_enable, bool, 0644);

extern char *es_str[3];

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
		       __FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

 close_time:
	rtc_class_close(rtc);
	return rc;
}

void check_and_run_debug_monitor(struct epack_dev_data *epack)
{
	unsigned long current_time = 0, next_monitor_time, time_left;

	if (!log_enable || epack->debug_i2c_fail)
		return;

	get_current_time(&current_time);

	next_monitor_time = epack->last_monitor_time
		+ (epack->polling_time_pwr / 1000);

	if (next_monitor_time > current_time)
		time_left = next_monitor_time - current_time;
	else
		time_left = 0;

	schedule_delayed_work(
			      &epack->debug_monitor_work
				  , msecs_to_jiffies(time_left * 1000));
}

void epack_debug_monitor(struct work_struct *work)
{
	struct epack_dev_data *epack = container_of(to_delayed_work(work)
					,struct epack_dev_data, debug_monitor_work);
	bool rc;

	if (!log_enable)
		return;

	get_current_time(&epack->last_monitor_time);

	if (!wake_lock_active(&epack->debug_monitor_wake_lock))
		wake_lock(&epack->debug_monitor_wake_lock);

	rc = cmd_get_pack_voltage(epack,&(epack->batt_voltage));
	if (!rc)
		goto i2c_err;
	rc = cmd_get_pack_temp(epack,&(epack->batt_temp));
	if (!rc)
		goto i2c_err;
	rc = cmd_get_pack_chg_status(epack,&(epack->chg_status));
	if (!rc)
		goto i2c_err;
	rc = cmd_get_pack_fault_status(epack,&(epack->fault_status));
	if (!rc)
		goto i2c_err;
	rc = cmd_get_pack_output_status(epack,&(epack->output_status));
	if (!rc)
		goto i2c_err;

	pr_err("[EpackMonitor] id/vbus:%s rc:%d V:%dmV T:%d chg:0x%x fault:0x%x output:0x%x \n"
			,es_str[epack->last_status]
			,rc,epack->batt_voltage,epack->batt_temp
			,epack->chg_status,epack->fault_status,epack->output_status);

	schedule_delayed_work(&epack->debug_monitor_work
							,msecs_to_jiffies(epack->polling_time_pwr));

	if (wake_lock_active(&epack->debug_monitor_wake_lock))
		wake_unlock(&epack->debug_monitor_wake_lock);
	return;

i2c_err:
	pr_err("[EpackMonitor] id/vbus:%s rc:%d V:%dmV T:%d chg:0x%x fault:0x%x output:0x%x \n"
			,es_str[epack->last_status]
			,rc,epack->batt_voltage,epack->batt_temp
			,epack->chg_status,epack->fault_status,epack->output_status);
	pr_err("[EpackMonitor] i2c line is unstable. log will be disabled! \n");
	epack->debug_i2c_fail = 1;

	if (wake_lock_active(&epack->debug_monitor_wake_lock))
		wake_unlock(&epack->debug_monitor_wake_lock);
	return;
}