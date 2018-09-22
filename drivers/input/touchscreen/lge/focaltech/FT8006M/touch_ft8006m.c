/* touch_ft8006m.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hyokmin.kwon@lge.com
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
#define TS_MODULE "[ft8006m]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <soc/qcom/lge/board_lge.h>


/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8006m.h"
#include "touch_ft8006m_prd.h"


// Definitions for Debugging Failure Reason in LPWG
enum {
	TCI_DEBUG_DISABLE = 0,
	TCI_DEBUG_ALWAYS,
	TCI_DEBUG_BUFFER,
	TCI_DEBUG_BUFFER_ALWAYS,
};

static const char *tci_debug_type_str[] = {
	"Disable Type",
	"Always Report Type",
	"Buffer Type",
	"Buffer and Always Report Type"
};

#define TCI_FR_BUF_LEN	10
#define TCI_FR_NUM		7

static const char const *tci_debug_str[TCI_FR_NUM + 1] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME", /* It means Over Tap */
	"PALM_STATE",
	"Reserved" // Invalid data
};

#ifdef FT8006M_ESD_SKIP_WHILE_TOUCH_ON
static int finger_cnt = 0;

bool ft8006m_check_finger(void)
{
	return finger_cnt==0? false:true;
}
EXPORT_SYMBOL(ft8006m_check_finger);
#endif

#if defined(CONFIG_LGE_MODULE_DETECT)
int ft8006m_panel_type;
#endif /* CONFIG_LGE_MODULE_DETECT */

/* touch irq handle according to display suspend in mfts */
bool mfts_check_shutdown = false;

int ft8006m_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int ft8006m_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);

	return 0;
}

int ft8006m_cmd_read(struct device *dev, void *cmd_data, int cmd_len, void *read_buf, int read_len)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	memcpy(&ts->tx_buf[0], cmd_data, cmd_len);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = cmd_len;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = read_len;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(read_buf, &ts->rx_buf[0], read_len);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int fts_i2c_read_cmd(struct device *dev, char *writebuf, int writelen, char *readbuf, int readlen)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	mutex_lock(&d->rw_lock);

	if(readlen > 0)
	{
		if (writelen > 0) {
			ret = i2c_master_send(client, writebuf, writelen);
			if (ret < 0)
				TOUCH_E("touch bus read error : %d\n", ret);

			touch_msleep(1);
			ret = i2c_master_recv(client, readbuf, readlen);
			if (ret < 0)
				TOUCH_E("touch bus read error : %d\n", ret);
		} else {
			struct i2c_msg msgs[] = {
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = readlen,
					 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				TOUCH_E("touch bus read error : %d\n", ret);
		}
	}

	mutex_unlock(&d->rw_lock);

	return ret;
}

static int ft8006m_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft8006m_data *d = to_ft8006m_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	default :
	case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
//		ft8006m_cmd_write(dev, CMD_ENA);
//		ft8006m_cmd_write(dev, CMD_RESET_LOW);
		touch_msleep(5);
//		ft8006m_cmd_write(dev, CMD_RESET_HIGH);
//		ft8006m_cmd_write(dev, CMD_DIS);
		touch_msleep(ts->caps.sw_reset_delay);
		break;

	case HW_RESET:
		TOUCH_I("%s : HW Reset\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		queue_delayed_work(ts->wq, &ts->init_work,
			msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;
	}

	return 0;
}

static int ft8006m_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft8006m_data *d = to_ft8006m_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		//touch_power_vio(dev, 0);
		//touch_power_vdd(dev, 0);
		touch_msleep(5);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		//touch_power_vdd(dev, 1);
		//touch_power_vio(dev, 1);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(ts->caps.hw_reset_delay);
		break;

	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	case POWER_HW_RESET:
		TOUCH_I("%s, HW Reset\n", __func__);
		ft8006m_reset_ctrl(dev, HW_RESET);
	}

	return 0;
}

static void ft8006m_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static struct ft8006m_ic_info *ext_ic_info;

int ft8006m_chipid_info(void)
{
	int ret = 0;

	if (!ext_ic_info) {
		return -EINVAL;
	}

	if ((ext_ic_info->chip_id == 0x86)
			&& (ext_ic_info->chip_id_low == 0x07)) {
		ret = 7;
	} else if ((ext_ic_info->chip_id == 0x86)
			&& (ext_ic_info->chip_id_low == 0x06)) {
		ret = 6;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int ft8006m_ic_info(struct device *dev)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	u8 chip_id = 0;
	u8 chip_id_low = 0;
	u8 is_official = 0;
	u8 fw_version = 0;
	u8 fw_version_minor = 0;
	u8 fw_version_sub_minor = 0;
	u8 fw_vendor_id = 0;
	u8 lib_version_high = 0;
	u8 lib_version_low = 0;
	int i;
//	u8 rdata = 0;

	// In LPWG, i2c can be failed because of i2c sleep mode
	if (d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Cannot get ic info in NOT ACTIVE mode\n");
		return -EPERM; // Do nothing in caller
	}

	// If it is failed to get info without error, just return error
	for (i = 0; i < 2; i++) {
		ret |= ft8006m_reg_read(dev, FTS_REG_ID, (u8 *)&chip_id, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_ID_LOW, (u8 *)&chip_id_low, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_FW_VER, (u8 *)&fw_version, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_FW_VER_MINOR, (u8 *)&fw_version_minor, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_FW_VER_SUB_MINOR, (u8 *)&fw_version_sub_minor, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_FW_VENDOR_ID, (u8 *)&fw_vendor_id, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_LIB_VER_H, (u8 *)&lib_version_high, 1);
		ret |= ft8006m_reg_read(dev, FTS_REG_LIB_VER_L, (u8 *)&lib_version_low, 1);

		if (ret == 0) {
			TOUCH_I("Success to get ic info data\n");
			break;
		}
	}

	if (i >= 2) {
		TOUCH_E("Failed to get ic info data, (need to recover it?)\n");
		return -EPERM; // Do nothing in caller
	}

	is_official = (fw_version & 0x80) >> 7;
	fw_version &= 0x7F;
	d->ic_info.version.major = fw_version;
	d->ic_info.version.minor = fw_version_minor;
	d->ic_info.version.sub_minor = fw_version_sub_minor;

	d->ic_info.chip_id = chip_id; // Device ID
	d->ic_info.chip_id_low = chip_id_low;
	d->ic_info.is_official = is_official;
	d->ic_info.fw_version = fw_version; // Major
	d->ic_info.fw_vendor_id = fw_vendor_id; // Vendor ID
	d->ic_info.lib_version_high = lib_version_high;
	d->ic_info.lib_version_low = lib_version_low;

	d->ic_info.info_valid = 1;

	ext_ic_info = &d->ic_info;

	TOUCH_I("chip_id : %x, chip_id_low : %x, is_official : %d, fw_version : %d.%d.%d, fw_vendor_id : %x,\n"\
		"lib_version_high : %x, lib_version_low : %x\n", \
		chip_id, chip_id_low, is_official, fw_version, fw_version_minor, fw_version_sub_minor, fw_vendor_id, \
		lib_version_high, lib_version_low);

	return ret;
}

#if 0
static void set_debug_reason(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4946_data *d = to_lg4946_data(dev);
	u32 wdata[2] = {0, };
	u32 start_addr = 0x0;
	int i = 0;

	wdata[0] = (u32)type;
	wdata[0] |= (d->tci_debug_type == 1) ? 0x01 << 2 : 0x01 << 3;
	wdata[1] = TCI_DEBUG_ALL;
	TOUCH_I("TCI%d-type:%d\n", type + 1, wdata[0]);

	lg4946_xfer_msg_ready(dev, 2);
	start_addr = TCI_FAIL_DEBUG_W;
	for (i = 0; i < 2; i++) {
		ts->xfer->data[i].tx.addr = start_addr + i;
		ts->xfer->data[i].tx.buf = (u8 *)&wdata[i];
		ts->xfer->data[i].tx.size = sizeof(u32);
	}

	lg4946_xfer_msg(dev, ts->xfer);

	return;
}
#endif

static int ft8006m_tci_control(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u8 data;
	int ret = 0;


	switch (type) {
	case TCI_CTRL_SET:
		if (d->tci_debug_type || d->en_i2c_lpwg) { // LPWG I2C Enable
			data = 0x01; // Stand-by mode
			TOUCH_I("Enable i2c block in LPWG mode\n");
		}
		else {
			data = 0x00; // Stop mode
		}
		d->en_i2c_lpwg = 0; // One-time flag for lpwg_sd
		ret = ft8006m_reg_write(dev, 0xF6, &data, 1);
		data = ts->tci.mode;
		ret = ft8006m_reg_write(dev, 0xD0, &data, 1);
		break;

	case TCI_CTRL_CONFIG_COMMON:
		data = (((d->tci_debug_type) & 0x02) << 3) | ((d->tci_debug_type) & 0x01);
		ret = ft8006m_reg_write(dev, 0xE5, &data, 1);	// Fail Reason Debug Function Enable

		data = 0x28;
		ret |= ft8006m_reg_write(dev, 0xB4, &data, 1);	// Active Area LSB of X1
		data = 0x00;
		ret |= ft8006m_reg_write(dev, 0xB5, &data, 1);	// Active Area MSB of X1 (40)
		data = 0xA8;
		ret |= ft8006m_reg_write(dev, 0xB6, &data, 1);	// Active Area LSB of X2
		data = 0x02;
		ret |= ft8006m_reg_write(dev, 0xB7, &data, 1);	// Active Area MSB of X2 (680)
		data = 0x22;
		ret |= ft8006m_reg_write(dev, 0xB8, &data, 1);	// Active Area LSB of Y1
		data = 0x00;
		ret |= ft8006m_reg_write(dev, 0xB9, &data, 1);	// Active Area MSB of Y1 (34)
		data = 0xDE;
		ret |= ft8006m_reg_write(dev, 0xBA, &data, 1);	// Active Area LSB of Y2
		data = 0x04;
		ret |= ft8006m_reg_write(dev, 0xBB, &data, 1);	// Active Area MSB of Y2 (1246)
		break;

	case TCI_CTRL_CONFIG_TCI_1:
		data = 0x0A;
		ret = ft8006m_reg_write(dev, 0xBC, &data, 1);	// Touch Slop (10mm)
		data = 0x0A;
		ret |= ft8006m_reg_write(dev, 0xC4, &data, 1);	// Touch Distance (10mm)
		data = 0x46; //0x32;
		ret |= ft8006m_reg_write(dev, 0xC6, &data, 1);	// Time Gap Max (700ms)
		data = 0x02;
		ret |= ft8006m_reg_write(dev, 0xCA, &data, 1);	// Total Tap Count (2)
		data = info1->intr_delay;
		ret |= ft8006m_reg_write(dev, 0xCC, &data, 1);	// Interrupt Delay (700ms or 0ms)
		break;

	case TCI_CTRL_CONFIG_TCI_2:
		data = 0x0A;
		ret = ft8006m_reg_write(dev, 0xBD, &data, 1);	// Touch Slop (10mm)
		data = 0xFF; //0xC8;
		ret |= ft8006m_reg_write(dev, 0xC5, &data, 1);	// Touch Distance (?? 200mm ??)
		data = 0x46;
		ret |= ft8006m_reg_write(dev, 0xC7, &data, 1);	// Time Gap Max (700ms)
		data = info2->tap_count;
		ret |= ft8006m_reg_write(dev, 0xCB, &data, 1);	// Total Tap Count (2)
		data = 0x25;
		ret |= ft8006m_reg_write(dev, 0xCD, &data, 1);	// Interrupt Delay (370ms ??)
		break;

	default:
		break;
	}

	return ret;
}


static int ft8006m_lpwg_control(struct device *dev, u8 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	//struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;

	switch (mode) {

	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;

		ret = ft8006m_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_SET);

		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x03;
		info1->intr_delay = ts->tci.double_tap_check ? 70 : 0;

		ret = ft8006m_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_CONFIG_TCI_2);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_SET);

		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x02;
		info1->intr_delay = 0;

//		ret = ft8006m_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_CONFIG_TCI_2);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
		ret |= ft8006m_tci_control(dev, TCI_CTRL_SET);

		break;

	default:
		ts->tci.mode = 0;

		ret = ft8006m_tci_control(dev, TCI_CTRL_SET);

		break;
	}

	TOUCH_I("ft8006m_lpwg_control mode = %d\n", mode);

	return ret;
}


static int ft8006m_deep_sleep(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 data;

	TOUCH_I("ft8006m_deep_sleep = %d\n", mode);

	if(mode) {
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
			return 0;
		data = 0x03;
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		return ft8006m_reg_write(dev, 0xA5, &data, 1);
	}
	else {
		if (atomic_read(&ts->state.sleep) == IC_NORMAL)
			return 0;
		// Do something
		atomic_set(&ts->state.sleep, IC_NORMAL);
		return 0;
	}
}

#if 0
static void ft8006m_debug_tci(struct device *dev)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	u8 debug_reason_buf[TCI_MAX_NUM][TCI_DEBUG_MAX_NUM];
	u32 rdata[9] = {0, };
	u8 count[2] = {0, };
	u8 count_max = 0;
	u32 i, j = 0;
	u8 buf = 0;

	if (!d->tci_debug_type)
		return;

	ft8006m_reg_read(dev, TCI_DEBUG_R, &rdata, sizeof(rdata));

	count[TCI_1] = (rdata[0] & 0xFFFF);
	count[TCI_2] = ((rdata[0] >> 16) & 0xFFFF);
	count_max = (count[TCI_1] > count[TCI_2]) ? count[TCI_1] : count[TCI_2];

	if (count_max == 0)
		return;

	if (count_max > TCI_DEBUG_MAX_NUM) {
		count_max = TCI_DEBUG_MAX_NUM;
		if (count[TCI_1] > TCI_DEBUG_MAX_NUM)
			count[TCI_1] = TCI_DEBUG_MAX_NUM;
		if (count[TCI_2] > TCI_DEBUG_MAX_NUM)
			count[TCI_2] = TCI_DEBUG_MAX_NUM;
	}

	for (i = 0; i < ((count_max-1)/4)+1; i++) {
		memcpy(&debug_reason_buf[TCI_1][i*4], &rdata[i+1], sizeof(u32));
		memcpy(&debug_reason_buf[TCI_2][i*4], &rdata[i+5], sizeof(u32));
	}

	TOUCH_I("TCI count_max = %d\n", count_max);
	for (i = 0; i < TCI_MAX_NUM; i++) {
		TOUCH_I("TCI count[%d] = %d\n", i, count[i]);
		for (j = 0; j < count[i]; j++) {
			buf = debug_reason_buf[i][j];
			TOUCH_I("TCI_%d - DBG[%d/%d]: %s\n",
				i + 1, j + 1, count[i],
				(buf > 0 && buf < TCI_FAIL_NUM) ?
					tci_debug_str[buf] :
					tci_debug_str[0]);
		}
	}
}
#endif

int ft8006m_lpwg_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("ft8006m_lpwg_reset\n");
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_msleep(1);
	LCD_RESET_H;
	touch_msleep(1);
	LCD_RESET_L;
	touch_msleep(1);
	LCD_RESET_H;

	touch_msleep(200);
	return 0;
}

static int ft8006m_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	u8 next_state;
	int ret = 0/*, mfts_mode = 0*/;

#if 1
	TOUCH_I(
		"ft8006m_lpwg_mode : mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
		ts->lpwg.mode,
		ts->lpwg.screen ? "ON" : "OFF",
		ts->lpwg.sensor ? "FAR" : "NEAR",
		ts->lpwg.qcover ? "CLOSE" : "OPEN");
	TOUCH_I("d->state[%d]\n", d->state);
#endif

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

	// Check MFTS mode to use POWER_OFF state
//	mfts_mode = touch_boot_mode_check(dev);
	if ((ts->boot_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("MINIOS_MFTS [%d]\n", ts->boot_mode);
		if (d->state == TC_STATE_ACTIVE && ts->lpwg.screen == 0) { // Int disable, Touch/LCD Reset 0, DSV Off, VDDI Off
			next_state = TC_STATE_POWER_OFF;
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
		} else if (d->state == TC_STATE_POWER_OFF && ts->lpwg.screen == 1) { // VDDI On, 1ms, DSV On, Touch/LCD Reset 1, Int enable
			next_state = TC_STATE_ACTIVE;
			touch_gpio_direction_output(ts->reset_pin, 1);
			touch_msleep(105); // lmh add

			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			touch_msleep(300); // ??????? Check with LCD on delay
		} else
			next_state = d->state;
		goto RET;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if ((ts->role.mfts_lpwg) && (d->state == TC_STATE_ACTIVE)) {
			ft8006m_lpwg_control(dev, LPWG_DOUBLE_TAP);
			next_state = TC_STATE_LPWG;
			goto RET;
		}
	}

	// NORMAL Case
	if (d->state == TC_STATE_ACTIVE) {
		if (ts->lpwg.screen == 0) {
			if((ts->lpwg.mode == 0) && !ts->role.mfts_lpwg) {
				next_state = TC_STATE_DEEP_SLEEP;
				ret = ft8006m_deep_sleep(dev, 1);
				TOUCH_I("4 0\n");
			} else if (ts->lpwg.sensor == 1) {
				next_state = TC_STATE_LPWG;
				TOUCH_I("%s\n","STATE_LPWG");
				ret = ft8006m_lpwg_control(dev, ts->lpwg.mode); // LCD SLP In or LCD Deep Sleep, 100ms -> Touch LPWG
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP;
				ret = ft8006m_deep_sleep(dev, 1); // LCD Deep Sleep, 100ms -> Touch Deep Sleep, DSV Off
				TOUCH_I("4\n");
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Do nothing
		}
	} else if (d->state == TC_STATE_LPWG) {
		if (ts->lpwg.screen == 0) {
			if (ts->lpwg.sensor == 0) {
				next_state = TC_STATE_DEEP_SLEEP; // Touch Reset, Deep Sleep, DSV Off
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_msleep(5);
				touch_gpio_direction_output(ts->reset_pin, 1);
				touch_msleep(105); // 105-> 300
				ret = ft8006m_deep_sleep(dev, 1);
				TOUCH_I("%s\n", "STATE_DEEP_SLEEP");
			}
			else {
				next_state = TC_STATE_LPWG; // Do nothing
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Touch Reset -> LCD Reset, SLP Out
			if(d->tci_debug_type & 0x02)
				ft8006m_report_tci_fr_buffer(dev); // Report fr before touch IC reset
			touch_gpio_direction_output(ts->reset_pin, 0);
			TOUCH_I("%s\n","STATE_ACTIVE");
			touch_msleep(5);
			touch_gpio_direction_output(ts->reset_pin, 1);
//			touch_msleep(105); //  case resume version 0.0
		}
	} else if (d->state == TC_STATE_DEEP_SLEEP) {
		if (ts->lpwg.screen == 0) {
			if(ts->lpwg.mode == 0){
				next_state = TC_STATE_DEEP_SLEEP; // Do nothing
			}
			else if (ts->lpwg.sensor == 1) {
				next_state = TC_STATE_LPWG;
				TOUCH_I("%s to %s\n", "DEEP_SLEEP", "LPWG");
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_msleep(5);
				touch_gpio_direction_output(ts->reset_pin, 1);
				touch_msleep(200);
				ret = ft8006m_deep_sleep(dev, 0);
				ret = ft8006m_lpwg_control(dev, ts->lpwg.mode);
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP; // Do nothing
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; //  Touch DSV On, Reset
			TOUCH_I("8\n");
			touch_gpio_direction_output(ts->reset_pin, 0);

			touch_msleep(5);
			touch_gpio_direction_output(ts->reset_pin, 1);
			touch_msleep(105);

			ret = ft8006m_deep_sleep(dev, 0);
			touch_msleep(15);
		}
	}
	else {
		next_state = d->state;
	}

RET:

	TOUCH_I("State changed from [%d] to [%d]\n", d->state, next_state);

	d->state = next_state;

	return ret;
}

static int ft8006m_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		TOUCH_I("LPWG_TAP_COUNT: [%d]\n", value[0]);
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		TOUCH_I("LPWG_DOUBLE_TAP_CHECK: [%d]\n", value[0]);
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			value[0],
			value[1] ? "ON" : "OFF",
			value[2] ? "FAR" : "NEAR",
			value[3] ? "CLOSE" : "OPEN");

		ft8006m_lpwg_mode(dev);
		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

int ft8006m_lpwg_set(struct device *dev){
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);

	mutex_lock(&d->fb_lock);
	if (!ts->role.mfts_lpwg)
		d->state = TC_STATE_ACTIVE;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
		ft8006m_deep_sleep(dev, 0);

	ft8006m_lpwg_mode(dev);

	mutex_unlock(&d->fb_lock);

	return 0;
}

static void ft8006m_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_INVALID)
		d->charger = CONNECT_NONE;
	else if ((charger_state == CONNECT_DCP)
			|| (charger_state == CONNECT_PROPRIETARY))
		d->charger = CONNECT_TA;
	else if (charger_state == CONNECT_HUB)
		d->charger = CONNECT_OTG;
	else
		d->charger = CONNECT_USB;

	/* code for TA simulator */
	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_4) {
		TOUCH_I("TA Simulator mode, Set CONNECT_TA\n");
		d->charger = CONNECT_TA;
	}

	/* wireless */
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try SPI\n");
		return;
	}

	ft8006m_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u8));
}

#if 0
static void ft8006m_lcd_event_read_reg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 rdata[5] = {0};

//	ft8006m_xfer_msg_ready(dev, 5);

	ts->xfer->data[0].rx.addr = tc_ic_status;
	ts->xfer->data[0].rx.buf = (u8 *)&rdata[0];
	ts->xfer->data[0].rx.size = sizeof(rdata[0]);

	ts->xfer->data[1].rx.addr = tc_status;
	ts->xfer->data[1].rx.buf = (u8 *)&rdata[1];
	ts->xfer->data[1].rx.size = sizeof(rdata[1]);

	ts->xfer->data[2].rx.addr = spr_subdisp_st;
	ts->xfer->data[2].rx.buf = (u8 *)&rdata[2];
	ts->xfer->data[2].rx.size = sizeof(rdata[2]);

	ts->xfer->data[3].rx.addr = tc_version;
	ts->xfer->data[3].rx.buf = (u8 *)&rdata[3];
	ts->xfer->data[3].rx.size = sizeof(rdata[3]);

	ts->xfer->data[4].rx.addr = 0x0;
	ts->xfer->data[4].rx.buf = (u8 *)&rdata[4];
	ts->xfer->data[4].rx.size = sizeof(rdata[4]);

//	ft8006m_xfer_msg(dev, ts->xfer);

	TOUCH_I(
		"reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x\n",
		tc_ic_status, rdata[0], tc_status, rdata[1],
		spr_subdisp_st, rdata[2], tc_version, rdata[3],
		0x0, rdata[4]);
	TOUCH_I("v%d.%02d\n", (rdata[3] >> 8) & 0xF, rdata[3] & 0xFF);
}

void ft8006m_xfer_msg_ready(struct device *dev, u8 msg_cnt)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);

	mutex_lock(&d->spi_lock);

	ts->xfer->msg_count = msg_cnt;
}
#endif

static int ft8006m_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	ft8006m_connect(dev);
	return 0;
}

#if 0
static int ft8006m_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	ft8006m_connect(dev);
	return 0;
}

static int ft8006m_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static int ft8006m_debug_tool(struct device *dev, u32 value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (value == DEBUG_TOOL_ENABLE) {
//		ts->driver->irq_handler = ft8006m_sic_abt_irq_handler;
	} else {
		ts->driver->irq_handler = ft8006m_irq_handler;
	}

	return 0;
}
static int ft8006m_debug_option(struct device *dev, u32 *data)
{
	u32 chg_mask = data[0];
	u32 enable = data[1];

	switch (chg_mask) {
	case DEBUG_OPTION_0:
		TOUCH_I("Debug Option 0 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_1:
//		if (enable)	/* ASC */
//			ft8006m_asc_control(dev, ASC_ON);
//		else
//			ft8006m_asc_control(dev, ASC_OFF);
		break;
	case DEBUG_OPTION_2:
		TOUCH_I("Debug Info %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_3:
		TOUCH_I("Debug Info Depth 10 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_4:
		TOUCH_I("TA Simulator mode %s\n", enable ? "Enable" : "Disable");
		ft8006m_connect(dev);
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return 0;
}

static void lg4946_debug_info_work_func(struct work_struct *debug_info_work)
{
	struct lg4946_data *d =
			container_of(to_delayed_work(debug_info_work),
				struct lg4946_data, debug_info_work);
	struct touch_core_data *ts = to_touch_core(d->dev);

	int status = 0;

	status = lg4946_debug_info(d->dev, 1);

	if (status > 0) {
		queue_delayed_work(d->wq_log, &d->debug_info_work , DEBUG_WQ_TIME);
	} else if (status < 0) {
		TOUCH_I("debug info log stop\n");
		atomic_set(&ts->state.debug_option_mask, DEBUG_OPTION_2);
	}
}
static void ft8006m_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct ft8006m_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct ft8006m_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U3)
		ret = FB_RESUME;
	else
		ret = FB_SUSPEND;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}
#endif
/*
static int ft8006m_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}

	return 0;
}
*/

static int ft8006m_notify_charger(struct device *dev, ulong event, void *data)
{

	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s event=0x%x\n", __func__, (unsigned int)event);

	switch (event) {
	case LCD_EVENT_TOUCH_RESET_START:
		TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		touch_gpio_direction_output(ts->reset_pin, 0);
		break;
	case LCD_EVENT_TOUCH_RESET_END:
		TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");
		touch_gpio_direction_output(ts->reset_pin, 1);

		queue_delayed_work(ts->wq, &ts->init_work,
					msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;

	default:
		TOUCH_E("%lu is not supported in Charger Mode\n", event);
		break;
	}

	return ret;
}

static int ft8006m_notify_normal(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s event=0x%x\n", __func__, (unsigned int)event);
	switch (event) {
	case LCD_EVENT_TOUCH_RESET_START:
		TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		touch_gpio_direction_output(ts->reset_pin, 0);
		break;
	case LCD_EVENT_TOUCH_RESET_END:
		TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");
		touch_gpio_direction_output(ts->reset_pin, 1);

		/* exception handle cause by init_work called two times in a row */
		if ((ts->boot_mode >= MINIOS_AAT) && ts->role.mfts_lpwg) {
			touch_msleep(ts->caps.hw_reset_delay);
			TOUCH_I("T-Reset END. No execute init_work in AAT\n");
			break;
		}

		queue_delayed_work(ts->wq, &ts->init_work,
					msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;
#if 0
	case NOTIFY_TOUCH_RESET:
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		atomic_set(&d->init, IC_INIT_NEED);
//		atomic_set(&d->watch.state.rtc_status, RTC_CLEAR);
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		ft8006m_lcd_mode(dev, *(u32 *)data);
		ret = ft8006m_check_mode(dev);
		if (ret == 0)
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		else
			ret = 0;
		break;
	case LCD_EVENT_READ_REG:
		TOUCH_I("LCD_EVENT_READ_REG\n");
		ft8006m_lcd_event_read_reg(dev);
		break;
#endif
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = ft8006m_usb_status(dev, *(u32 *)data);
//		if (ft8006m_asc_usable(dev))	/* ASC */
//			ft8006m_asc_toggle_delta_check(dev);
		break;
#if 0
	case NOTIFY_WIRELEES:
		TOUCH_I("NOTIFY_WIRELEES!\n");
		ret = ft8006m_wireless_status(dev, *(u32 *)data);
//		if (ft8006m_asc_usable(dev))	/* ASC */
//			ft8006m_asc_toggle_delta_check(dev);
		break;
	case NOTIFY_EARJACK:
		TOUCH_I("NOTIFY_EARJACK!\n");
		ret = ft8006m_earjack_status(dev, *(u32 *)data);
		break;
#endif
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
//		ret = ft8006m_reg_write(dev, REG_IME_STATE,
//			(u32*)data, sizeof(u32));
		break;
#if 0
	case NOTIFY_DEBUG_TOOL:
		ret = ft8006m_debug_tool(dev, *(u32 *)data);
		TOUCH_I("NOTIFY_DEBUG_TOOL!\n");
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("NOTIFY_CALL_STATE!\n");
		ret = ft8006m_reg_write(dev, REG_CALL_STATE,
			(u32 *)data, sizeof(u32));
//		if (ft8006m_asc_usable(dev))	/* ASC */
//			ft8006m_asc_toggle_delta_check(dev);
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = ft8006m_debug_option(dev, (u32 *)data);
		break;
	case NOTIFY_ONHAND_STATE:
		TOUCH_I("NOTIFY_ONHAND_STATE!\n");
//		if (ft8006m_asc_usable(dev)) {	/* ASC */
//			ft8006m_asc_toggle_delta_check(dev);
//			ft8006m_asc_write_onhand(dev, *(u32 *)data);
//		}
		break;
#endif
/*	case NOTIFY_QUICKCOVER_STATE:
		TOUCH_I("NOTIFY_QUICKCOVER_STATE!: %d\n", *(u8 *)data);
		ret = ft8006m_reg_write(dev, SPR_QUICKCOVER_STS, (u8 *)data, sizeof(u8));
		break; */
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static int ft8006m_notify(struct device *dev, ulong event, void *data)
{
	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("CHARGER MODE notify\n");
		return ft8006m_notify_charger(dev,event,data);
	}

	return ft8006m_notify_normal(dev,event,data);
}

static void ft8006m_init_locks(struct ft8006m_data *d)
{
	mutex_init(&d->rw_lock);
	mutex_init(&d->fb_lock);
}

static int ft8006m_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate ft8006m data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_gpio_init(ts->maker_id_pin, "touch_make_id");
	touch_gpio_direction_input(ts->maker_id_pin);

	touch_power_init(dev);
	touch_bus_init(dev, MAX_BUF_SIZE);

	ft8006m_init_locks(d);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 1);
		/* Deep Sleep */
		touch_msleep(100); // ???????????????????????
		ft8006m_deep_sleep(dev, 1);
		return 0;
	}

	ft8006m_get_tci_info(dev);

	d->tci_debug_type = TCI_DEBUG_DISABLE;
	atomic_set(&ts->state.debug_option_mask, DEBUG_OPTION_2);

	// To be implemented.....
#ifdef FTS_CTL_IIC
	fts_rw_iic_drv_init(to_i2c_client(dev));
#endif
#ifdef FTS_SYSFS_DEBUG
	fts_create_sysfs(to_i2c_client(dev));
#endif
#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(to_i2c_client(dev));
#endif

	return 0;
}

static int ft8006m_remove(struct device *dev)
{
	TOUCH_TRACE();
#ifdef FTS_APK_DEBUG
		fts_release_apk_debug_channel();
#endif

#ifdef FTS_SYSFS_DEBUG
		fts_remove_sysfs(to_i2c_client(dev));
#endif

#ifdef FTS_CTL_IIC
		fts_rw_iic_drv_exit();
#endif
	return 0;
}

static int ft8006m_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	u8 ic_fw_version = d->ic_info.fw_version;
	u8 ic_is_official = d->ic_info.is_official;
	u8 bin_fw_version = 0;
	u8 bin_is_official = 0;
	int update = 0;

	if(d->ic_info.info_valid == 0) { // Failed to get ic info
		TOUCH_I("invalid ic info, skip fw upgrade\n");
		return 0;
	}

#if 1  // JASON_TF10J
	bin_fw_version = fw->data[0x5100+0x0e];
#else
	if(d->ic_info.chip_id_low == 0x06) {
		bin_fw_version = fw->data[0x1000+0x10a];
	} else {
		bin_fw_version = fw->data[0x1000+0x10e];
	}
#endif


	bin_is_official = (bin_fw_version & 0x80) >> 7;
	bin_fw_version &= 0x7F;

	if(d->ic_info.fw_version_bin == 0 && d->ic_info.is_official_bin == 0) { // IF fw ver of bin is not initialized
		d->ic_info.fw_version_bin = bin_fw_version;
		d->ic_info.is_official_bin = bin_is_official;
	}

//	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
//	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);

//	if ((bin_ver_offset > FLASH_FW_SIZE) || (bin_pid_offset > FLASH_FW_SIZE)) {
//		TOUCH_I("%s : invalid offset\n", __func__);
//		return -1;
//	}

	if (ts->force_fwup) {
		update = 1;
	} else if ((ic_is_official != bin_is_official) || (ic_fw_version != bin_fw_version)) {
		update = 1;
	}

	TOUCH_I("%s : binary[V%d.%d] device[V%d.%d]" \
		" -> update: %d, force: %d\n", __func__,
		bin_is_official, bin_fw_version, ic_is_official, ic_fw_version,
		update, ts->force_fwup);

	return update;
}

static int ft8006m_fwboot_upgrade(struct device *dev, const struct firmware *fw_boot)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const u8 *fw_data = fw_boot->data;
	u32 fw_size = (u32)(fw_boot->size);
	u8 *fw_check_buf = NULL;
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, i, j, packet_addr, packet_len;
	u8 pramboot_ecc;

	TOUCH_I("%s - START\n", __func__);

	if(fw_size > 0x10000 || fw_size == 0)
		return -EIO;

	fw_check_buf = kmalloc(fw_size+1, GFP_ATOMIC);
	if(fw_check_buf == NULL)
		return -ENOMEM;

	for (i = 12; i <= 30; i++) {
		// Reset CTPM
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(50);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(i);

		// Set Upgrade Mode
		ret = ft8006m_reg_write(dev, 0x55, i2c_buf, 0);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		touch_msleep(1);

		// Check ID
		TOUCH_I("%s - Set Upgrade Mode and Check ID : %d ms\n", __func__, i);
		ret = ft8006m_reg_read(dev, 0x90, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		TOUCH_I("Check ID : 0x%x , 0x%x\n", i2c_buf[0], i2c_buf[1]);
#if 1 // JASON_TF10J
		if(i2c_buf[0] == 0x80&& i2c_buf[1] == 0x06){
			touch_msleep(50);
			break;
		}
#else
		if((i2c_buf[0] == 0x86 && i2c_buf[1] == 0x06)
			|| (i2c_buf[0] == 0x86 && i2c_buf[1] == 0x07)){
			touch_msleep(50);
			break;
		}
#endif
	}

	if (i > 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Write F/W (Pramboot) Binary to CTPM
	TOUCH_I("%s - Write F/W (Pramboot)\n", __func__);
	pramboot_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + (j/4)*4 + (3 - (j%4))] = fw_data[packet_addr + j];
			pramboot_ecc ^= i2c_buf[5 + j];
		}
		TOUCH_I("#%d : Writing to %d , %d bytes\n", i, packet_addr, packet_len); //kjh
		ret = ft8006m_reg_write(dev, 0xAE, i2c_buf, packet_len + 5);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
	}

	touch_msleep(100);

	// Verify F/W
	TOUCH_I("%s - Verify\n", __func__);
	for (i = 0; i < packet_num; i++) {
		i2c_buf[0] = 0x85;
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[1] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[2] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[3] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		TOUCH_I("#%d : Reading from %d , %d bytes\n", i, packet_addr, packet_len);  //kjh
		ret = ft8006m_cmd_read(dev, i2c_buf, 4, fw_check_buf+packet_addr, packet_len);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
	}

	for (i = 0; i < fw_size; i++) {
		//if(fw_check_buf[i] != fw_data[i]) {
		//TOUCH_I("%s - fw_check_buf : %d  fw_data : %d!!\n", __func__, fw_check_buf[i], fw_data[i]); //kjh
			//goto FAIL;
		//}
	}

	for (i = 0; i < fw_size; i++) {
		if(fw_check_buf[i] != fw_data[i]) {
			TOUCH_I("%s - Verify Failed %d %d %d!!\n", __func__,i, fw_check_buf[i], fw_data[i]); //kjh
			goto FAIL;
		}
	}

	TOUCH_I("%s - Verify OK !!\n", __func__);

	// Start App
	TOUCH_I("%s - Start App\n", __func__);
	ret = ft8006m_reg_write(dev, 0x08, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
#if 1 // JASON_TF10J
	touch_msleep(10);
#else
   //touch_msleep(20);
	touch_msleep(15);
#endif
	if(fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download Okay =====\n");

	return 0;

FAIL :

	if(fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download FAIL!!! =====\n");

	return -EIO;

}

static int ft8006m_fw_upgrade(struct device *dev, const struct firmware *fw)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	const u8 *fw_data = fw->data;
	u32 fw_size = (u32)(fw->size);
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, retry, i, j, packet_addr, packet_len;
	u8 reg_val_id = 0;
	u8 fw_ecc;
	//u8 retrim_val[4] = {0,};

	TOUCH_I("%s - START\n", __func__);


#if 1 // JASON_TF10J
// Skip code
#else
	// Check fw length and get val ID
	ret = ft8006m_reg_read(dev, 0x05, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return -EIO;
	}

	reg_val_id = i2c_buf[0];
	TOUCH_I("reg_val_id : 0x%x \n", reg_val_id);

	if(fw_size == 0) {
		return -EIO;
	}
	else if(reg_val_id == 0x81 && fw_size > 64*1024) {
		return -EIO;
	}
	else if(reg_val_id == 0x80 && fw_size > 68*1024) {
		return -EIO;
	}

#if 0
	// Reset CTPM
	TOUCH_I("%s - Reset CTPM\n", __func__);
	touch_msleep(100);

	i2c_buf[0] = 0xAA;
	ret = ft8006m_reg_write(dev, 0xFC, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return -EIO;
	}
	touch_msleep(5);

	i2c_buf[0] = 0x55;
	ret = ft8006m_reg_write(dev, 0xFC, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return -EIO;
	}
	touch_msleep(30);
#else
	touch_msleep(10);
#endif

#endif

	for (i = 0; i < 30; i++) {
		// Enter Upgrade Mode
		TOUCH_I("%s - Enter Upgrade Mode and Check ID\n", __func__);

#if 1  // JASON_TF10J
	ret = ft8006m_reg_write(dev, 0x55, i2c_buf, 0);  //Enter upgrade mode of ParamBoot
#else
		i2c_buf[0] = 0xAA;
		ret = ft8006m_reg_write(dev, 0x55, i2c_buf, 1);
#endif

		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}
		touch_msleep(1);

		// Check ID
#if 1
		ret = ft8006m_reg_read(dev, 0x90, i2c_buf, 2);
#else
		i2c_buf[0] = 0x90;
		i2c_buf[1] = i2c_buf[2] = i2c_buf[3] = 0;
		ret = ft8006m_cmd_read(dev, i2c_buf, 4, i2c_buf, 2);
#endif
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}
#if 1  // JASON_TF10J
		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);   // 0x80, 0xC6
		if(i2c_buf[0] == 0x80 && i2c_buf[1] == 0xC6)
			break;
#else
		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);
		if(i2c_buf[0] == 0x86 && i2c_buf[1] == 0xA6)
			break;
#endif
		touch_msleep(10);
	}

	if (i == 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Change to write flash mode
	i2c_buf[0] = reg_val_id;
	i2c_buf[1] = 0x00;
	//i2c_buf[1] = 0x01;
	ret = ft8006m_reg_write(dev, 0x05, i2c_buf, 2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

#if 1  // JASON_TF10J
	i2c_buf[0] = 0x80;  //
	i2c_buf[1] = 0x01;
	ret = ft8006m_reg_write(dev, 0x05, i2c_buf, 2);

//- Flash : 05
//- Flashe type : 81:FT5003, 80: Winbond
//- Clk speed : 00:48M 01:24M
#endif

// Change to write flash set range
#if 1  // JASON_TF10J
	i2c_buf[0] = 0x0A;
	ret = ft8006m_reg_write(dev, 0x09, i2c_buf, 1);
//- 0x0A : All
//- 0x0B : App
//- 0x0C : Lcd

#else
	i2c_buf[0] = 0x80;
	i2c_buf[1] = 0x12;
	ret = ft8006m_reg_write(dev, 0x09, i2c_buf, 2);
#endif
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(50);

#if 1  // JASON_TF10J
// Skip code
#else  // Only Tianma K6B, LV3
#if 1  // JASON  // Retrim value, Backup
	i2c_buf[0] = 0x03;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0x07;
	i2c_buf[3] = 0x90;
	fts_i2c_read_cmd(dev, i2c_buf, 4, retrim_val, 4);

	TOUCH_I("%s - 4 bytes backup : [0x%x][0x%x][0x%x][0x%x]\n", __func__, retrim_val[0], retrim_val[1], retrim_val[2], retrim_val[3]);
#endif
#endif

	// Erase App (Panel Parameter Area??)
	TOUCH_I("%s - Erase App and Panel Parameter Area\n", __func__);

	ret = ft8006m_reg_write(dev, 0x61, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(1000);

	retry = 300;
	i = 0;
	do {
		ret = ft8006m_reg_read(dev, 0x6A, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		if(i2c_buf[0] == 0xF0 && i2c_buf[1] == 0xAA)
		{
			TOUCH_I("Erase Done : %d \n", i);
			break;
		}
		i++;
		//touch_msleep(50);
		mdelay(10);
	} while (--retry);

	// Write F/W (App) Binary to CTPM
	TOUCH_I("%s - Write F/W (App)\n", __func__);
	fw_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + j] = fw_data[packet_addr + j];
			fw_ecc ^= i2c_buf[5 + j];
		}
#if 0
		TOUCH_I("#%d : Writing to %d , %d bytes..[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", i, packet_addr, packet_len, \
			i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3], i2c_buf[4]);
#endif
		ret = ft8006m_reg_write(dev, 0xBF, i2c_buf, packet_len + 5);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		//touch_msleep(10);
#if 1
#if 1
		// Waiting
		retry = 100;
		do {
			ret = ft8006m_reg_read(dev, 0x6A, i2c_buf, 2);
			if(ret < 0) {
				TOUCH_E("i2c error\n");
				goto FAIL;
			}
			if((u32)(i + 0x1000) == (((u32)(i2c_buf[0]) << 8) | ((u32)(i2c_buf[1]))))
			{
				if((i & 0x007F) == 0) {
					TOUCH_I("Write Done : %d / %d\n", i+1, packet_num);
				}
				break;
			}
			//touch_msleep(1);
			mdelay(1);
		} while (--retry);
		if(retry == 0) {
			TOUCH_I("Write Fail : %d / %d : [0x%02x] , [0x%02x]\n", i+1, packet_num, i2c_buf[0], i2c_buf[1]);
			//goto FAIL;
		}
#else
		if(packet_addr % (0x1000) == 0) {
			touch_msleep(300);
			TOUCH_I("#%d : Writing to %d , %d bytes..[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", i, packet_addr, packet_len, \
			i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3], i2c_buf[4]);
		}
		else {
			touch_msleep(20);
		}

#endif
#endif
	}
	TOUCH_I("Write Finished : Total %d\n", packet_num);


#if 1  // JASON_TF10J
// Skip code
#else  // Only Tianma K6B, LV3
#if 1 // JASON   // Retrim value, Restore
	if ((retrim_val[0]==(~retrim_val[1]&0xFF))&&(retrim_val[2]==(~retrim_val[3]&0xFF))) {
		TOUCH_I("%s - 4 bytes restore\n", __func__);
		i2c_buf[0] = 0x00;
		i2c_buf[1] = 0x07;
		i2c_buf[2] = 0x90;
		i2c_buf[3] = 0x00;
		i2c_buf[4] = 0x04;

		for (i = 0; i < 4; i++) {
			i2c_buf[5 + i] = retrim_val[i];
		}

		ret = ft8006m_reg_write(dev, 0xBF, i2c_buf, 4 + 5);
		if(ret < 0) {
			TOUCH_I("0xBF i2c error\n");
		}

		for(i = 0; i < 30; i++) {
			i2c_buf[0] = i2c_buf[1] = 0x00;
			ret = ft8006m_reg_read(dev, 0x6A, i2c_buf, 2);
			if(ret < 0) {
				TOUCH_I("0x6A i2c error\n");
			}

			if ((0x0790/4 + 0x1000) == (((i2c_buf[0]) << 8) | i2c_buf[1])) {
				TOUCH_I("%s - 4 bytes restore success..[0x%02x][0x%02x]\n", __func__, i2c_buf[0], i2c_buf[1]);
				break;
			}
			mdelay(10);
		}
	}else {
		TOUCH_I("%s - 4 bytes restore didn't\n", __func__);
	}
#endif
#endif


	touch_msleep(50);

	// Read out Checksum
	TOUCH_I("%s - Read out checksum (App) for %d bytes\n", __func__, fw_size);
	ret = ft8006m_reg_write(dev, 0x64, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(300);

	packet_num = (fw_size + LEN_FLASH_ECC_MAX - 1) / LEN_FLASH_ECC_MAX;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * LEN_FLASH_ECC_MAX;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + LEN_FLASH_ECC_MAX > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = LEN_FLASH_ECC_MAX;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		ret = ft8006m_reg_write(dev, 0x65, i2c_buf, 5);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		touch_msleep(fw_size/256);

		retry = 200;
		do {
			ret = ft8006m_reg_read(dev, 0x6A, i2c_buf, 2);
			if(ret < 0) {
				TOUCH_E("i2c error\n");
				goto FAIL;
			}

			if(i2c_buf[0] == 0xF0 && i2c_buf[1] == 0x55)
			{
				TOUCH_I("Checksum Calc. Done\n");
				break;
			}
			touch_msleep(1);
		} while (--retry);
	}

	ret = ft8006m_reg_read(dev, 0xCC, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	TOUCH_I("Reg 0xCC : 0x%x\n", i2c_buf[0]);

	ret = ft8006m_reg_read(dev, 0x66, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	TOUCH_I("Reg 0x66 : 0x%x\n", i2c_buf[0]);

	if(i2c_buf[0] != fw_ecc)
	{
		TOUCH_E("Checksum ERROR : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);
		goto FAIL;
	}

	TOUCH_I("Checksum OK : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);

	TOUCH_I("===== Firmware download OK!!! =====\n");

	/*SW Reset CMD*/
	ret = ft8006m_reg_write(dev, 0x07, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	/*After SW Reset, not need HW_Reset_Delay becuz it wiil take in CORE side*/
/*
	// Do something for recovering LCD
	ret = tianma_ft860x_firmware_recovery();
	touch_msleep(300);

	if(ret != 0){
		TOUCH_I("tianma_ft860x_firmware_recovery didn't LCD reset : ret = %d\n", ret);
	}
*/
	return 0;

FAIL:

	TOUCH_I("===== Firmware download FAIL!!! =====\n");

	// Reset Anyway
	ft8006m_power(dev, POWER_OFF);
	ft8006m_power(dev, POWER_ON);

	return -EIO;

}


static int ft8006m_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	const struct firmware *fw_boot = NULL;
	char fwpath[256] = {0};
	int ret = 0;
//	int i = 0;

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0] && (ts->test_fwpath[0] != 't' && ts->test_fwpath[1] != 0)) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
#if defined(CONFIG_LGE_MODULE_DETECT)
		if (ft8006m_panel_type == CV1_TIANMA_FT8006M) {
			strncpy(fwpath, ts->def_fwpath[1], sizeof(fwpath) - 1);
			TOUCH_I("Get Touch Firmware for CV1 Tianma\n");
		} else { /* CV1_TCL_FT8006M */
			strncpy(fwpath, ts->def_fwpath[2], sizeof(fwpath) - 1);
			TOUCH_I("Get Touch Firmware for CV1 TCL\n");
		}
#else
		strncpy(fwpath, ts->def_fwpath[1], sizeof(fwpath) - 1);
#endif /* CONFIG_LGE_MODULE_DETECT */
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (ft8006m_fw_compare(dev, fw)) {

//		ret = -EINVAL;
//		touch_msleep(200);
//		for (i = 0; i < 2 && ret; i++)
//			ret = ft8006m_fw_upgrade(dev, fw);

		TOUCH_I("fwpath_boot[%s]\n", ts->def_fwpath[0]);  // PARAMBOOT.BIN
		ret = request_firmware(&fw_boot, ts->def_fwpath[0], dev);
		if (ret < 0) {
			TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", ts->def_fwpath[0], ret);
			release_firmware(fw);
			return ret;
		}

		//touch_msleep(200);

		ret = ft8006m_fwboot_upgrade(dev, fw_boot);
		if(ret < 0) {
			TOUCH_E("fail to upgrade f/w (pramboot) : %d\n", ret);
			release_firmware(fw);
			release_firmware(fw_boot);
			return -EPERM;
		}

		ret = ft8006m_fw_upgrade(dev, fw);
		if(ret < 0) {
			TOUCH_E("fail to upgrade f/w : %d\n", ret);
			release_firmware(fw);
			release_firmware(fw_boot);
			return -EPERM;
		}

		TOUCH_I("f/w upgrade complete\n");

		release_firmware(fw_boot);

	}

	release_firmware(fw);

	return 0;
}

static int ft8006m_suspend(struct device *dev)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	//int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;
/*
	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		ft8006m_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
//		if (d->lcd_mode == LCD_MODE_U2 &&
//			atomic_read(&d->watch.state.rtc_status) == RTC_RUN &&
//			d->watch.ext_wdata.time.disp_waton)
//				ext_watch_get_current_time(dev, NULL, NULL);
	}
*/
	if (atomic_read(&d->init) == IC_INIT_DONE)
		ft8006m_lpwg_mode(dev);
	else /* need init */
		ret = 1;

	return ret;
}

static int ft8006m_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
//	int mfts_mode = 0;

	TOUCH_TRACE();

//	mfts_mode = touch_boot_mode_check(dev);
	if ((ts->boot_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		#if 0
		ft8006m_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ft8006m_ic_info(dev);
		if (ft8006m_upgrade(dev) == 0) {
			ft8006m_power(dev, POWER_OFF);
			ft8006m_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
		//return -EPERM;
		#endif
		// Activate IC is done in fb callback EARLY BLANK (POWER_OFF => ACTIVE)
		// Return 0 to do init in touch_resume
		if (d->state == TC_STATE_ACTIVE)
			return 0;
		else
			return -EPERM;
	}
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		ft8006m_deep_sleep(dev, 1);
		return -EPERM;
	}

//	if (atomic_read(&d->init) == IC_INIT_DONE)
//		ft8006m_lpwg_mode(dev);
//	else /* need init */
//		ret = 1;

	return 0;
}

static int ft8006m_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
//	u32 data = 1;
	int ret = 0;

	TOUCH_TRACE();
/*
	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("ft8006m fb_notif register\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = ft8006m_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}
*/
	touch_msleep(50);
	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);

//	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE)
//		ft8006m_sic_abt_init(dev);

	ret = ft8006m_ic_info(dev);
	if (ret < 0) {
		TOUCH_I("failed to get ic_info, ret:%d", ret);
		atomic_set(&d->init, IC_INIT_DONE); // Nothing to init, anyway DONE init

		return 0;
		//touch_interrupt_control(dev, INTERRUPT_DISABLE);
		//ft8006m_power(dev, POWER_OFF);
		//ft8006m_power(dev, POWER_ON);
		//touch_msleep(ts->caps.hw_reset_delay);
	}
#if 0
	ret = ft8006m_reg_write(dev, tc_device_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);

	ret = ft8006m_reg_write(dev, tc_interrupt_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);
#endif
	ret = ft8006m_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u8));
	if (ret)
		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);

	ret = ft8006m_reg_write(dev, SPR_QUICKCOVER_STS, &ts->lpwg.qcover, sizeof(u8));
	if (ret)
		TOUCH_E("failed to write SPR_QUICKCOVER_STS, ret:%d\n", ret);
#if 0
	data = atomic_read(&ts->state.ime);
	ret = ft8006m_reg_write(dev, REG_IME_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);
#endif
	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);
	d->state = TC_STATE_ACTIVE;

	ft8006m_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_control, ret:%d", ret);

	return 0;
}

#if 0
 ////// USED IN IRQ HANDLER

/* (1 << 5) | (1 << 9)|(1 << 10) */
#define INT_RESET_CLR_BIT	0x620
/* (1 << 6) | (1 << 7)|(1 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_LOGGING_CLR_BIT	0x50A0C0
/* (1 << 5) |(1 << 6) |(1 << 7)|(0 << 9)|(0 << 10)|(0 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_NORMAL_MASK		0x5080E0
#define IC_DEBUG_SIZE		16	/* byte */

int ft8006m_check_status(struct device *dev)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	u32 status = d->info.device_status;
	u32 ic_status = d->info.ic_status;
	u32 debugging_mask = 0x0;
	u8 debugging_length = 0x0;
	u32 debugging_type = 0x0;
	u32 status_mask = 0x0;
	int checking_log_flag = 0;
	const int checking_log_size = DEBUG_BUF_SIZE;
	char checking_log[DEBUG_BUF_SIZE] = {0};
	int length = 0;

	status_mask = status ^ INT_NORMAL_MASK;
	if (status_mask & INT_RESET_CLR_BIT) {
		TOUCH_I("%s : Need Reset, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERESTART;
	} else if (status_mask & INT_LOGGING_CLR_BIT) {
		TOUCH_I("%s : Need Logging, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERANGE;
	}

	if (!(status & (1 << 5))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[5]Device_ctl not Set");
	}
	if (!(status & (1 << 6))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[6]Code CRC Invalid");
	}
	if (!(status & (1 << 7))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[7]CFG CRC Invalid");
	}
	if (status & (1 << 9)) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[9]Abnormal status Detected");
	}
	if (status & (1 << 10)) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[10]System Error Detected");
	}
	if (status & (1 << 13)) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[13]Display mode Mismatch");
	}
	if (!(status & (1 << 15))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[15]Interrupt_Pin Invalid");
	}
	if (!(status & (1 << 20))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[20]Touch interrupt status Invalid");
	}
	if (!(status & (1 << 22))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[22]TC driving Invalid");
	}

	if (checking_log_flag) {
		TOUCH_E("%s, status = %x, ic_status = %x\n",
				checking_log, status, ic_status);
	}

	if ((ic_status & 1) || (ic_status & (1 << 3))) {
		TOUCH_I("%s : Watchdog Exception - status : %x, ic_status : %x\n",
			__func__, status, ic_status);
		ret = -ERESTART;
	}

	if (ret == -ERESTART)
		return ret;

	debugging_mask = ((status >> 16) & 0xF);
	if (debugging_mask == 0x2) {
		TOUCH_I("TC_Driving OK\n");
		ret = -ERANGE;
	} else if (debugging_mask == 0x3 || debugging_mask == 0x4) {
		debugging_length = ((d->info.debug.ic_debug_info >> 24) & 0xFF);
		debugging_type = (d->info.debug.ic_debug_info & 0x00FFFFFF);
		TOUCH_I(
			"%s, INT_TYPE:%x,Length:%d,Type:%x,Log:%x %x %x\n",
			__func__, debugging_mask,
			debugging_length, debugging_type,
			d->info.debug.ic_debug[0], d->info.debug.ic_debug[1],
			d->info.debug.ic_debug[2]);
		ret = -ERANGE;
	}

	return ret;
}
#endif

#if 0
int ft8006m_debug_info(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct ft8006m_touch_debug *debug = &d->info.debug;
	int ret = 0;
	int i = 0;
	u8 buf[DEBUG_BUF_SIZE] = {0,};

	u16 debug_change_mask = 0;
	u16 press_mask = 0;
	u16 release_mask = 0;
	debug_change_mask = ts->old_mask ^ ts->new_mask;
	press_mask = ts->new_mask & debug_change_mask;
	release_mask = ts->old_mask & debug_change_mask;

	/* check protocol ver */
	if ((debug->protocol_ver < 0) && !(atomic_read(&d->init) == IC_INIT_DONE))
		return ret;

	/* check debugger status */
	if ((atomic_read(&ts->state.earjack) == EARJACK_DEBUG) ||
		(gpio_get_value(126) < 1))
		return ret;

#if 0
	if (!mode) {
		if ((debug_change_mask && press_mask)
				|| (debug_change_mask && release_mask)) {
			if (debug_detect_filter(dev, ts->tcount)) {
				/* disable func in irq handler */
				atomic_set(&ts->state.debug_option_mask,
						(atomic_read(&ts->state.debug_option_mask)
						 ^ DEBUG_OPTION_2));
				d->frame_cnt = debug->frame_cnt;
				queue_delayed_work(d->wq_log, &d->debug_info_work, DEBUG_WQ_TIME);
			}
			if ((debug->rebase[0] > 0)
					&& (debug_change_mask && press_mask)
					&& (ts->tcount < 2))
				goto report_debug_info;
		}
	}

	if (mode) {
		u8 debug_data[264];

		if (lg4946_reg_read(dev, tc_ic_status, &debug_data[0],
					sizeof(debug_data)) < 0) {
			TOUCH_I("debug data read fail\n");
		} else {
			memcpy(&d->info.debug, &debug_data[132], sizeof(d->info.debug));
		}

		if (debug->frame_cnt - d->frame_cnt > DEBUG_FRAME_CNT) {
			TOUCH_I("frame cnt over\n");
			return -1;
		}

		goto report_debug_info;
	}
#endif
	return ret;

//report_debug_info:
		ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
				"[%d] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
				ts->tcount - 1,
				debug->protocol_ver,
				debug->frame_cnt,
				debug->rn_max_bfl,
				debug->rn_max_afl,
				debug->rn_min_bfl,
				debug->rn_min_afl,
				debug->rn_max_afl_x,
				debug->rn_max_afl_y,
				debug->seg1_cnt,
				debug->seg2_cnt,
				debug->seg1_thr,
				debug->rn_pos_cnt,
				debug->rn_neg_cnt,
				debug->rn_pos_sum,
				debug->rn_neg_sum,
				debug->rn_stable
			       );

		for (i = 0 ; i < ts->tcount ; i++) {
			if (i < 1)
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						" tb:");
			ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
					"%2d ",	debug->track_bit[i]);
		}

		for (i = 0 ; i < sizeof(debug->rn_max_tobj) ; i++) {
			if (debug->rn_max_tobj[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" to:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug->rn_max_tobj[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->rebase) ; i++) {
			if (debug->rebase[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" re:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug->rebase[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->noise_detect) ; i++) {
			if (debug->noise_detect[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" nd:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug->noise_detect[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->lf_oft) ; i++) {
			if (debug->lf_oft[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" lf:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2x ",	debug->lf_oft[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->palm) ; i++) {
			if (debug->palm[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" pa:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",	debug->palm[i]);
			} else {
				break;
			}
		}
		TOUCH_I("%s\n", buf);

		return ret;
}
#endif

static int ft8006m_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct ft8006m_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
//	int ret = 0;
	int i = 0;
	u8 touch_id, event, palm;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;

	for (i = 0; i < FTS_MAX_POINTS; i++) {
		touch_id = (u8)(data[i].yh) >> 4;
		if (touch_id >= FTS_MAX_ID) {
			break; // ??
		}

		event = (u8)(data[i].xh) >> 6;
		palm = ((u8)(data[i].xh) >> 4) & 0x01;

		if (palm) {
			if (event == FTS_TOUCH_CONTACT) { // FTS_TOUCH_DOWN
				ts->is_cancel = 1;
				TOUCH_I("Palm Detected\n");
			}
			else if (event == FTS_TOUCH_UP) {
				ts->is_cancel = 0;
				TOUCH_I("Palm Released\n");
			}
			ts->tcount = 0;
			ts->intr_status = TOUCH_IRQ_FINGER;
			return 0;
		}

		if(event == FTS_TOUCH_DOWN || event == FTS_TOUCH_CONTACT) {
			ts->new_mask |= (1 << touch_id);
			tdata = ts->tdata + touch_id;

			tdata->id = touch_id;
			tdata->type = MT_TOOL_FINGER;
			tdata->x = ((u16)(data[i].xh & 0x0F))<<8 | (u16)(data[i].xl);
			tdata->y = ((u16)(data[i].yh & 0x0F))<<8 | (u16)(data[i].yl);
			tdata->pressure = (u8)(data[i].weight);
			if((event == FTS_TOUCH_DOWN) && (tdata->pressure==0x29)) {
				tdata->pressure = 0x30;
			}
			tdata->width_major = (u8)((data[i].area)>>4);
			tdata->width_minor = 0;
			tdata->orientation = 0;

			if (0 == tdata->width_major)
				tdata->width_major = 0x09;
			if (0 == tdata->pressure)
				tdata->width_major = 0x3f;

			finger_index++;

			TOUCH_D(ABS, "tdata [id:%d e:%d x:%d y:%d z:%d - %d,%d,%d]\n",\
					tdata->id,\
					event,\
					tdata->x,\
					tdata->y,\
					tdata->pressure,\
					tdata->width_major,\
					tdata->width_minor,\
					tdata->orientation);

		}
	}

#ifdef FT8006M_ESD_SKIP_WHILE_TOUCH_ON
	if (finger_index != finger_cnt) {
//		TOUCH_I("finger cnt changed from %d to %d\n", finger_cnt, finger_index);
		finger_cnt = finger_index;
	}
#endif

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return 0;

#if 0

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;

	/* check if palm detected */
	if (data[0].track_id == PALM_ID) {
		if (data[0].event == TOUCHSTS_DOWN) {
			ts->is_cancel = 1;
			TOUCH_I("Palm Detected\n");
		} else if (data[0].event == TOUCHSTS_UP) {
			ts->is_cancel = 0;
			TOUCH_I("Palm Released\n");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++) {
		if (data[i].track_id >= MAX_FINGER)
			continue;

		if (data[i].event == TOUCHSTS_DOWN
			|| data[i].event == TOUCHSTS_MOVE) {
			ts->new_mask |= (1 << data[i].track_id);
			tdata = ts->tdata + data[i].track_id;

			tdata->id = data[i].track_id;
			tdata->type = data[i].tool_type;
			tdata->x = data[i].x;
			tdata->y = data[i].y;
			tdata->pressure = data[i].pressure;
			tdata->width_major = data[i].width_major;
			tdata->width_minor = data[i].width_minor;

			if (data[i].width_major == data[i].width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = data[i].angle;

			finger_index++;

			TOUCH_D(ABS,
				"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);

		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
#endif

}

int ft8006m_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	struct ft8006m_touch_data *data = d->info.data;
	u8 point_buf[POINT_READ_BUF] = { 0, };
	int ret = -1;

	ret = ft8006m_reg_read(dev, 0x02, point_buf+2, POINT_READ_BUF-2);

	if (ret < 0) {
		TOUCH_E("Fail to read point regs.\n");
		return ret;
	}

	/* check if touch cnt is valid */
	if (/*point_buf[FTS_TOUCH_P_NUM] == 0 || */point_buf[FTS_TOUCH_P_NUM] > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, point_buf[FTS_TOUCH_P_NUM]);
		return -ERANGE;
	}

	d->info.touch_cnt = point_buf[FTS_TOUCH_P_NUM];

	memcpy(data, point_buf+FTS_TOUCH_EVENT_POS, FTS_ONE_TCH_LEN * FTS_MAX_POINTS);

	return ft8006m_irq_abs_data(dev);
}

int ft8006m_irq_lpwg(struct device *dev, int tci)
{
	struct touch_core_data *ts = to_touch_core(dev);
	//struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0, tap_count, i, j;
	u8 tci_data_buf[MAX_TAP_COUNT*4 + 2];

	if(ts->lpwg.mode == LPWG_NONE || (ts->lpwg.mode == LPWG_DOUBLE_TAP && tci == TCI_2)) {
		TOUCH_I("lpwg irq is invalid!!\n");
		return -1;
	}

	ret = ft8006m_reg_read(dev, 0xD3, tci_data_buf, 2);
	if (ret < 0) {
		TOUCH_E("Fail to read tci data\n");
		return ret;
	}

	TOUCH_I("TCI Data : TCI[%d], Result[%d], TapCount[%d]\n", tci, tci_data_buf[0], tci_data_buf[1]);

	// Validate tci data
	if (!((tci_data_buf[0] == 0x01 && tci == TCI_1) || (tci_data_buf[0] == 0x02 && tci == TCI_2))
		|| tci_data_buf[1] == 0 || tci_data_buf[1] > MAX_TAP_COUNT) {
		TOUCH_I("tci data is invalid!!\n");
		return -1;
	}

	tap_count = tci_data_buf[1];

	ret = ft8006m_reg_read(dev, 0xD3, tci_data_buf, tap_count*4 + 2);
	if (ret < 0) {
		TOUCH_E("Fail to read tci data\n");
		return ret;
	}

	ts->lpwg.code_num = tap_count;
	for (i = 0; i < tap_count; i++) {
		j = i*4+2;
		ts->lpwg.code[i].x = ((int)tci_data_buf[j] << 8) | (int)tci_data_buf[j+1];
		ts->lpwg.code[i].y = ((int)tci_data_buf[j+2] << 8) | (int)tci_data_buf[j+3];

		if (ts->lpwg.mode >= LPWG_PASSWORD)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n", ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[tap_count].x = -1;
	ts->lpwg.code[tap_count].y = -1;

	if(tci == TCI_1)
		ts->intr_status = TOUCH_IRQ_KNOCK;
	else if(tci == TCI_2)
		ts->intr_status = TOUCH_IRQ_PASSWD;
	else
		ts->intr_status = TOUCH_IRQ_NONE;

#if 0
	if (d->info.wakeup_type == KNOCK_1) {
		if (ts->lpwg.mode != LPWG_NONE) {
			ft8006m_get_tci_data(dev,
				ts->tci.info[TCI_1].tap_count);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
	} else if (d->info.wakeup_type == KNOCK_2) {
		if (ts->lpwg.mode == LPWG_PASSWORD) {
			ft8006m_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
	} else if (d->info.wakeup_type == SWIPE_UP) {
		TOUCH_I("SWIPE_UP\n");
		ft8006m_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
	} else if (d->info.wakeup_type == SWIPE_DOWN) {
		TOUCH_I("SWIPE_DOWN\n");
		ft8006m_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
	} else if (d->info.wakeup_type == KNOCK_OVERTAP) {
		TOUCH_I("LPWG wakeup_type is Overtap\n");
		ft8006m_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count + 1);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if (d->info.wakeup_type == CUSTOM_DEBUG) {
		TOUCH_I("LPWG wakeup_type is CUSTOM_DEBUG\n");
		ft8006m_debug_tci(dev);
		ft8006m_debug_swipe(dev);
	} else {
		TOUCH_I("LPWG wakeup_type is not support type![%d]\n",
			d->info.wakeup_type);
	}
#endif

	return ret;
}

int ft8006m_irq_report_tci_fr(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	u8 data, tci_1_fr, tci_2_fr;

	if (d->tci_debug_type != TCI_DEBUG_ALWAYS && d->tci_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("tci debug in real time is disabled!!\n");
		return 0;
	}

	ret = ft8006m_reg_read(dev, 0xE6, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	tci_1_fr = data & 0x0F; // TCI_1
	tci_2_fr = (data & 0xF0) >> 4; // TCI_2

	if (tci_1_fr < TCI_FR_NUM) {
		TOUCH_I("Knock-on Failure Reason Reported : [%s]\n", tci_debug_str[tci_1_fr]);
	}
	else {
		TOUCH_I("Knock-on Failure Reason Reported : [%s]\n", tci_debug_str[TCI_FR_NUM]);
	}

	if (tci_2_fr < TCI_FR_NUM) {
		TOUCH_I("Knock-code Failure Reason Reported : [%s]\n", tci_debug_str[tci_2_fr]);
	}
	else {
		TOUCH_I("Knock-code Failure Reason Reported : [%s]\n", tci_debug_str[TCI_FR_NUM]);
	}

	return ret;
}

int ft8006m_report_tci_fr_buffer(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0, i;
	u8 tci_fr_buffer[1 + TCI_FR_BUF_LEN], tci_fr_cnt, tci_fr;

	if (d->tci_debug_type != TCI_DEBUG_BUFFER && d->tci_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("tci debug in buffer is disabled!!\n");
		return 0;
	}

	// Knock-on
	for (i = 0; i < 25; i++) {
		ret = ft8006m_reg_read(dev, 0xE7, &tci_fr_buffer, sizeof(tci_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	tci_fr_cnt = tci_fr_buffer[0];
	if (tci_fr_cnt > TCI_FR_BUF_LEN) {
		TOUCH_I("Knock-on Failure Reason Buffer Count Invalid\n");
	}
	else if (tci_fr_cnt == 0) {
		TOUCH_I("Knock-on Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < tci_fr_cnt; i++) {
			tci_fr = tci_fr_buffer[1 + i];
			if (tci_fr < TCI_FR_NUM) {
				TOUCH_I("Knock-on Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[tci_fr]);
			}
			else {
				TOUCH_I("Knock-on Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[TCI_FR_NUM]);
			}
		}
	}

	// Knock-code (Same as knock-on case except for reg addr)
	for (i = 0; i < 25; i++) {
		ret = ft8006m_reg_read(dev, 0xE9, &tci_fr_buffer, sizeof(tci_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	tci_fr_cnt = tci_fr_buffer[0];
	if (tci_fr_cnt > TCI_FR_BUF_LEN) {
		TOUCH_I("Knock-code Failure Reason Buffer Count Invalid\n");
	}
	else if (tci_fr_cnt == 0) {
		TOUCH_I("Knock-code Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < tci_fr_cnt; i++) {
			tci_fr = tci_fr_buffer[1 + i];
			if (tci_fr < TCI_FR_NUM) {
				TOUCH_I("Knock-code Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[tci_fr]);
			}
			else {
				TOUCH_I("Knock-code Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[TCI_FR_NUM]);
			}
		}
	}

	return ret;
}


int ft8006m_irq_handler(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct touch_core_data *ts =(struct touch_core_data *)dev->driver_data;
	//struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	u8 int_status = 0;

	if ((ts->boot_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		if (mfts_check_shutdown) {
			TOUCH_I("Unknown IRQ. Do not handling\n");
			return ret;
		}
	}

	ret = ft8006m_reg_read(dev, 0x01, &int_status, 1);
	if (ret < 0)
		return ret;
	//ret = ft8006m_irq_abs(dev);

	if (int_status == 0x01) { // Finger
		ret = ft8006m_irq_abs(dev);
	}
	else if (int_status == 0x02) { // TCI_1
		ret = ft8006m_irq_lpwg(dev, TCI_1);
	}
	else if (int_status == 0x03) { // TCI_2
		ret = ft8006m_irq_lpwg(dev, TCI_2);
	}
	else if (int_status == 0x04) { // LPWG Fail Reason Report (RT)
		ret = ft8006m_irq_report_tci_fr(dev);
	}
	else if (int_status == 0x05) { // ESD
		TOUCH_I("ESD interrupt !!\n");
#if defined(CONFIG_LGE_DISPLAY_RECOVERY)
		ret = -EGLOBALRESET;
#endif
	}
	else if (int_status == 0x00) {
		TOUCH_I("No interrupt status\n");
	}
	else {
		TOUCH_E("Invalid interrupt status : %d\n", int_status);
#if defined(CONFIG_LGE_DISPLAY_RECOVERY)
		ret = -EGLOBALRESET;
#endif
	}

	return ret;

#if 0
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4946_data *d = to_lg4946_data(dev);
	int ret = 0;

	lg4946_reg_read(dev, tc_ic_status, &d->info,
				sizeof(d->info));
	ret = lg4946_check_status(dev);

	if (ret < 0)
		goto error;
	if (d->info.wakeup_type == ABS_MODE) {
		ret = lg4946_irq_abs(dev);
		if (lg4946_asc_delta_chk_usable(dev))	/* ASC */
			queue_delayed_work(ts->wq,
					&(d->asc.finger_input_work), 0);
	} else {
		ret = lg4946_irq_lpwg(dev);
	}

	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_2)
		lg4946_debug_info(dev, 0);
error:
	return ret;

#endif
}

static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	char command[6] = {0};
	int reg = 0;
	int value = 0;
	u8 data = 0;
	u8 reg_addr;

	if (sscanf(buf, "%5s %x %x", command, &reg, &value) <= 0)
		return count;

	reg_addr = (u8)reg;
	if (!strcmp(command, "write")) {
		data = (u8)value;
		if (ft8006m_reg_write(dev, reg_addr, &data, sizeof(u8)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (ft8006m_reg_read(dev, reg_addr, &data, sizeof(u8)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}
	return count;
}

static ssize_t show_tci_debug(struct device *dev, char *buf)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Current TCI Debug Type = %s\n",
			(d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	TOUCH_I("Current TCI Debug Type = %s\n",
			(d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	return ret;
}

static ssize_t store_tci_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0 || value < 0 || value > 3) {
		TOUCH_I("Invalid TCI Debug Type, please input 0~3\n");
		return count;
	}

	d->tci_debug_type = (u8)value;

	TOUCH_I("Set TCI Debug Type = %s\n", (d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	ft8006m_reset_ctrl(dev, value);

	return count;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);

static struct attribute *ft8006m_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_reset_ctrl.attr,
	NULL,
};

static const struct attribute_group ft8006m_attribute_group = {
	.attrs = ft8006m_attribute_list,
};

static int ft8006m_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &ft8006m_attribute_group);
	if (ret < 0)
		TOUCH_E("ft8006m sysfs register failed\n");

//	ft8006m_watch_register_sysfs(dev);
	ft8006m_prd_register_sysfs(dev);
//	ft8006m_asc_register_sysfs(dev);	/* ASC */
//	ft8006m_sic_abt_register_sysfs(&ts->kobj);

	return 0;
}

static int ft8006m_get_cmd_version(struct device *dev, char *buf)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int offset = 0;
	int ret = 0;

	ret = ft8006m_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf + offset, PAGE_SIZE - offset, "IC firmware info\n");
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Version : V%d.%02d\n\n",
		d->ic_info.is_official, d->ic_info.fw_version);

	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Bin firmware info\n");
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Version : V%d.%02d\n\n",
		d->ic_info.is_official_bin, d->ic_info.fw_version_bin);

	if (d->ic_info.chip_id == 0x80) {
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "Product-id : [FT800%xm]\n\n", d->ic_info.chip_id_low);
	}
	else {
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "Product-id : [Unknown]\n\n");
	}

	return offset;
}

static int ft8006m_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int offset = 0;
	int ret = 0;

	ret = ft8006m_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf, PAGE_SIZE, "V%d.%02d\n",
		d->ic_info.is_official, d->ic_info.fw_version);

	return offset;
}

static int ft8006m_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int ft8006m_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = ft8006m_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = ft8006m_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}
static int ft8006m_shutdown(struct device *dev){
	TOUCH_TRACE();
	return 0;
}

static struct touch_driver touch_driver = {
	.probe = ft8006m_probe,
	.remove = ft8006m_remove,
	.suspend = ft8006m_suspend,
	.shutdown = ft8006m_shutdown,
	.resume = ft8006m_resume,
	.init = ft8006m_init,
	.irq_handler = ft8006m_irq_handler,
	.power = ft8006m_power,
	.upgrade = ft8006m_upgrade,
	.lpwg = ft8006m_lpwg,
	.notify = ft8006m_notify,
	.register_sysfs = ft8006m_register_sysfs,
	.set = ft8006m_set,
	.get = ft8006m_get,
};

#define MATCH_NAME			"focaltech,ft8006m"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

#if defined(CONFIG_LGE_MODULE_DETECT)
static bool ft8006m_get_device_type(void)
{
	bool bdevice = false;
	enum lge_panel_type panel_type = lge_get_panel_type();

	TOUCH_I("%s [lge_get_panel_type] = [%d]\n", __func__, panel_type);

	switch(panel_type) {
	case CV1_TIANMA_FT8006M:
	case CV1_TCL_FT8006M:
		ft8006m_panel_type = panel_type;
		bdevice = true;
		break;
	default:
		break;
	}

	return bdevice;
}
#endif /* CONFIG_LGE_MODULE_DETECT */

static int __init touch_device_init(void)
{
	TOUCH_I("touch_device_init func\n");
	TOUCH_TRACE();
#if defined(CONFIG_LGE_MODULE_DETECT)
	if (!ft8006m_get_device_type()) {
		TOUCH_I("%s, ft8006m returned\n", __func__);
		return 0;
	}
	TOUCH_I("This panel is FT8006M\n");
#endif /* CONFIG_LGE_MODULE_DETECT */

	TOUCH_I("FT8006M__[%s] touch_bus_device_init\n", __func__);

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("hyokmin.kwon@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
