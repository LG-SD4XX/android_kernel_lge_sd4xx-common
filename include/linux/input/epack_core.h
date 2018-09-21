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

#ifndef __E_PACK__
#define __E_PACK__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/power_supply.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

#define PACKET_SIZE	64

enum epack_status{
	EPACK_ABSENT,
	EPACK_POWER_LOW,
	EPACK_POWER_OK,
	EPACK_STATUS_MAX,
};

enum vbus_source_type{
	NONE,
	FROM_EPACK,
	FROM_USB_PORT,
};

struct epack_dev_data {
	struct i2c_client *client;
	int last_status;
	int vbus_src;

	int polling_time_snd;
	int polling_time_pwr;
	unsigned long last_monitor_time;
	int id_gpio;
	int pack_power_gpio;
	int usb_power_gpio;
	int ovp_sw_pcon_gpio;

	unsigned int hwver;
	unsigned int fwver;
	unsigned int bin_fwver[3];
	unsigned int devid;
	unsigned int devconfig[2];
	uint8_t rcvbuf[PACKET_SIZE];
	uint8_t sendbuf[PACKET_SIZE];
	bool force_update;
	bool debug_i2c_fail;

	int slavemode;

	int batt_voltage;
	int batt_temp;
	int chg_status;
	int fault_status;
	int output_status;
	struct mutex i2c_lock;
	struct wake_lock vbus_ctrl_wake_lock;
	struct wake_lock debug_monitor_wake_lock;
	struct workqueue_struct *wq;
	struct delayed_work audio_work;
	struct delayed_work debug_monitor_work;
	struct delayed_work notify_epack_w_otg_work;
	struct delayed_work notify_epack_ready_work;
	struct delayed_work notify_epack_unready_work;
	struct delayed_work set_vbus_off_work;
	struct power_supply *usb_psy;
	struct switch_dev sdev;
	const struct epack_amp_cb *amp_cb;
};

bool set_epack_vbus_off(void);
int get_epack_status(void);
int get_vbus_source(void);
void set_tablet_otg_status(int on);
void epack_audio_work_func(struct work_struct *audio_work);
void epack_debug_monitor(struct work_struct *work);
void check_and_run_debug_monitor(struct epack_dev_data *epack);
int epack_firmware_update(struct i2c_client *client, struct device *dev,int flag, char* fwpath);
bool check_fw_update(unsigned int fwver,unsigned int hwver);
int get_fw_ver_from_file(char* fwpath, struct device *dev);

#define MAX_RETRY_COUNT 5
#define epack_log(fmt, args...) 	printk(KERN_ERR "[Epack] " fmt, ##args)
#endif //__E_PACK__
