/* touch_core_sysfs.c
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
#include <touch_hwif.h>

#define TOUCH_SHOW(ret, buf, fmt, args...) \
	(ret += snprintf(buf + ret, PAGE_SIZE - ret, fmt, ##args))

static char ime_str[3][8] = {"OFF", "ON", "SWYPE"};
static char incoming_call_str[3][8] = {"IDLE", "RINGING", "OFFHOOK"};
static char mfts_str[4][8] = {"NONE", "FOLDER", "FLAT", "CURVED"};
int mfts_lpwg;
static ssize_t show_platform_data(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int i;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_SHOW(ret, buf, "=== Platform Data ===\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "reset_pin", ts->reset_pin);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "int_pin", ts->int_pin);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "maker_id_pin", ts->maker_id_pin);

	TOUCH_SHOW(ret, buf, "caps:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_x", ts->caps.max_x);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_y", ts->caps.max_y);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_pressure",
		   ts->caps.max_pressure);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_width_major", ts->caps.max_width_major);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_width_minor", ts->caps.max_width_minor);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_orientation",
		   ts->caps.max_orientation);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "max_id", ts->caps.max_id);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "hw_reset_delay",
		   ts->caps.hw_reset_delay);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "sw_reset_delay",
		   ts->caps.sw_reset_delay);

	TOUCH_SHOW(ret, buf, "role:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "use_lpwg", ts->role.use_lpwg);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "hide_coordinate",
		   ts->role.hide_coordinate);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "use_fw_upgrade",
		   ts->role.use_fw_upgrade);

	TOUCH_SHOW(ret, buf, "power:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "vdd-gpio", ts->vdd_pin);
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "vio-gpio", ts->vio_pin);

	TOUCH_SHOW(ret, buf, "firmware:\n");
	TOUCH_SHOW(ret, buf, "\t%25s = %d\n", "def_fwcnt", ts->def_fwcnt);
	for (i = 0; i < ts->def_fwcnt; i++)
		TOUCH_SHOW(ret, buf, "\t%25s [%d:%s]\n", "def_fwpath",
			   i, ts->def_fwpath[i]);

	return ret;
}

static ssize_t store_upgrade(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (ts->lpwg.screen == 0) {
		TOUCH_E("LCD OFF state. please turn on the display\n");
		return count;
	}

	if (sscanf(buf, "%255s", &ts->test_fwpath[0]) <= 0)
		return count;

	ts->force_fwup = 1;
	queue_delayed_work(ts->wq, &ts->upgrade_work, 0);

	return count;
}

static ssize_t show_upgrade(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (ts->lpwg.screen == 0) {
		TOUCH_E("LCD OFF state. please turn on the display\n");
		ret = snprintf(buf, PAGE_SIZE, "LCD Off state. please turn on the display\n");
		return ret;
	}

	ts->test_fwpath[0] = '\0';
	ts->force_fwup = 1;

	queue_delayed_work(ts->wq, &ts->upgrade_work, 0);

	return 0;
}

static ssize_t show_lpwg_data(struct device *dev, char *buf)
{
	int i = 0, ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (!ts->driver->lpwg)
		return ret;

	mutex_lock(&ts->lock);
	for (i = 0; i < MAX_LPWG_CODE; i++) {
		if (ts->lpwg.code[i].x == -1 && ts->lpwg.code[i].y == -1)
			break;
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	memset(ts->lpwg.code, 0, sizeof(struct point) * MAX_LPWG_CODE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t store_lpwg_data(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int reply = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &reply) <= 0)
		return count;

	TOUCH_I("%s : reply = %d\n", __func__, reply);

	atomic_set(&ts->state.uevent, UEVENT_IDLE);
	wake_unlock(&ts->lpwg_wake_lock);

	return count;
}

static ssize_t store_lpwg_notify(struct device *dev,
		const char *buf, size_t count)
{

	struct touch_core_data *ts = to_touch_core(dev);
	int code = 0;
	int param[4] = {0, };
	int mfts_mode = 0;

	TOUCH_TRACE();

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg)
		return count;

	if (sscanf(buf, "%d %d %d %d %d",
			&code, &param[0], &param[1], &param[2], &param[3]) <= 0)
		return count;

	/* only below code notify
		3 active_area
		4 knockcode tap count
		8 knockcode double tap check
		9 update_all
	*/
	if (code == 1 || code == 2 || code == 5 ||
		code == 6 || code == 7)
		return count;

	if (ts->driver->lpwg) {
		mutex_lock(&ts->lock);
		ts->driver->lpwg(ts->dev, code, param);
		mutex_unlock(&ts->lock);
	}

	return count;
}

static ssize_t show_lockscreen_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	value = atomic_read(&ts->state.lockscreen);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
		value ? "LOCK" : "UNLOCK", value);

	return ret;
}

static ssize_t store_lockscreen_state(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value == LOCKSCREEN_UNLOCK || value == LOCKSCREEN_LOCK) {
		atomic_set(&ts->state.lockscreen, value);
		TOUCH_I("%s : %s(%d)\n", __func__,
				value ? "LOCK" : "UNLOCK", value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_ime_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	value = atomic_read(&ts->state.ime);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
			ime_str[value], value);

	return ret;
}

static ssize_t store_ime_state(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= IME_OFF && value <= IME_SWYPE) {
		if (atomic_read(&ts->state.ime) == value)
			return count;

		atomic_set(&ts->state.ime, value);
		ret = touch_blocking_notifier_call(NOTIFY_IME_STATE,
			&ts->state.ime);
		TOUCH_I("%s : %s(%d), ret = %d\n",
			__func__, ime_str[value], value, ret);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_quick_cover_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	value = atomic_read(&ts->state.quick_cover);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
		value ? "CLOSE" : "OPEN", value);

	return ret;
}

static ssize_t store_quick_cover_state(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value == QUICKCOVER_CLOSE || value == QUICKCOVER_OPEN) {
		atomic_set(&ts->state.quick_cover, value);
		TOUCH_I("%s : %s(%d)\n", __func__,
			value ? "CLOSE" : "OPEN", value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_incoming_call_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	value = atomic_read(&ts->state.incoming_call);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
		incoming_call_str[value], value);

	return ret;
}

static ssize_t store_incoming_call_state(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= INCOMING_CALL_IDLE && value <= INCOMING_CALL_OFFHOOK) {
		if (atomic_read(&ts->state.incoming_call) == value)
			return count;

		atomic_set(&ts->state.incoming_call, value);

		ret = touch_blocking_notifier_call(NOTIFY_CALL_STATE,
					&ts->state.incoming_call);

		TOUCH_I("%s : %s(%d)\n", __func__,
				incoming_call_str[value], value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_version_info(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	mutex_lock(&ts->lock);
	ret = ts->driver->get(dev, CMD_VERSION, NULL, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_atcmd_version_info(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	mutex_lock(&ts->lock);
	ret = ts->driver->get(dev, CMD_ATCMD_VERSION, NULL, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_mfts_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	value = atomic_read(&ts->state.mfts);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
			mfts_str[value], value);

	return ret;
}

static ssize_t store_mfts_state(struct device *dev,
		const char *buf, size_t count)
{
	int value = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= MFTS_NONE && value <= MFTS_CURVED) {
		atomic_set(&ts->state.mfts, value);
		TOUCH_I("%s : %s(%d)\n", __func__, mfts_str[value], value);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	ts->boot_mode = touch_boot_mode_check(ts->dev);
	return count;
}

static ssize_t show_mfts_lpwg(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = snprintf(buf, PAGE_SIZE, "%d\n", ts->role.use_lpwg_test);

	return ret;
}

static ssize_t store_mfts_lpwg(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	ts->role.mfts_lpwg = value;
	mfts_lpwg = value;
	TOUCH_I("mfts_lpwg:%d\n", ts->role.mfts_lpwg);

	return count;
}


static ssize_t show_sp_link_touch_off(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "sp link touch status %d\n",
			atomic_read(&ts->state.sp_link));

	return ret;
}

static ssize_t store_sp_link_touch_off(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		TOUCH_I("Invalid Value\n");
		return count;
	}

	atomic_set(&ts->state.sp_link, value);

	if (atomic_read(&ts->state.sp_link) == SP_CONNECT) {
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		TOUCH_I("SP Mirroring Connected\n");
	} else if(atomic_read(&ts->state.sp_link) == SP_DISCONNECT) {
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		TOUCH_I("SP Mirroring Disconnected\n");
	}

	return count;
}

static ssize_t show_debug_tool_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.debug_tool);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", value);

	return ret;
}

static ssize_t store_debug_tool_state(struct device *dev,
		const char *buf, size_t count)
{
	int data = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	if (sscanf(buf, "%d", &data) <= 0)
		return count;

	if (data >= DEBUG_TOOL_DISABLE && data <= DEBUG_TOOL_ENABLE) {
		atomic_set(&ts->state.debug_tool, data);
		ts->driver->notify(dev, NOTIFY_DEBUG_TOOL, (void *)&data);
		TOUCH_I("%s : %s\n", __func__,
		(data == DEBUG_TOOL_ENABLE) ?
		"Debug Tool Enabled" : "Debug Tool Disabled");
	} else {
		TOUCH_I("%s : Unknown debug tool set value %d\n",
			__func__, data);
	}

	return count;
}

static ssize_t show_debug_option_state(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.debug_option_mask);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", value);

	return ret;
}

static ssize_t store_debug_option_state(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int new_mask = 0;
	int old_mask = 0;
	int data[2] = {0, 0};

	old_mask = atomic_read(&ts->state.debug_option_mask);

	if (sscanf(buf, "%d", &new_mask) <= 0)
		return count;

	if (new_mask >= DEBUG_OPTION_DISABLE
		&& new_mask <= DEBUG_OPTION_ALL) {
		atomic_set(&ts->state.debug_option_mask, new_mask);
		TOUCH_I("%s : Input masking value = %d\n",
			__func__, new_mask);
	} else {
		TOUCH_I("%s : Unknown debug option set value %d\n",
			__func__, new_mask);
	}

	data[0] = new_mask ^ old_mask; //Changed mask
	data[1] = data[0] & new_mask; //Enable(!=0) or Disable(==0)

	ts->driver->notify(dev, NOTIFY_DEBUG_OPTION, (void *)&data);

	return count;
}

static ssize_t show_app_data(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int i = 0;

	TOUCH_TRACE();

	for(i = 0 ; i < 3 ; i++) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d %s %d %d\n",
					ts->app_data[i].app, ts->app_data[i].version,
					ts->app_data[i].icon_size, ts->app_data[i].touch_slop);
		if (ts->app_data[i].icon_size != 0) {
			TOUCH_I("%s : Read %s App data (Icon_Size = %d, Touch_Slop = %d, Version = %s)\n",
					__func__, (ts->app_data[i].app == APP_HOME ? "Home" : ((ts->app_data[i].app == APP_CONTACTS) ? "Contacts" : "Menu")),
					ts->app_data[i].icon_size, ts->app_data[i].touch_slop, ts->app_data[i].version);
		}

	}

	return ret;
}

static ssize_t store_app_data(struct device *dev,
		const char *buf, size_t count)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct app_info app_data_buf;

	TOUCH_TRACE();

	if (sscanf(buf, "%d %10s %d %d",
			&app_data_buf.app, app_data_buf.version,
			&app_data_buf.icon_size, &app_data_buf.touch_slop) <= 0)
		return count;

	if (app_data_buf.app >= APP_HOME && app_data_buf.app <= APP_MENU) {
		memcpy(&ts->app_data[app_data_buf.app], &app_data_buf, sizeof(app_data_buf));
		TOUCH_I("%s : Write %s App data (Icon_Size = %d, Touch_Slop = %d, Version = %s)\n",
				__func__, (app_data_buf.app == APP_HOME ? "Home" : ((app_data_buf.app == APP_CONTACTS) ? "Contacts" : "Menu")),
				app_data_buf.icon_size, app_data_buf.touch_slop, app_data_buf.version);
	}

	return count;
}

static ssize_t show_click_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	struct touch_data tdata;
	int cnt = 100 / ts->perf_test.delay;	/* click 100ms */
	int i = 0;

	TOUCH_TRACE();

	if (!ts->perf_test.enable)
		return ret;

	if (cnt < 2) {
		TOUCH_E("invalid cnt(%d)\n", cnt);
		return -EINVAL;
	}

	tdata.id = 0;
	tdata.x = ts->perf_test.click_x;
	tdata.y = ts->perf_test.click_y;
	tdata.pressure = ts->perf_test.pressure;
	tdata.width_major = ts->perf_test.width;
	tdata.width_minor = ts->perf_test.width;
	tdata.orientation = 0;

	TOUCH_I("%s: start (%4d, %4d)\n", __func__, tdata.x, tdata.y);
	mutex_lock(&ts->lock);
	touch_report_all_event(ts);

	for (i = 0; i < cnt; i++) {
		input_mt_slot(ts->input, tdata.id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, tdata.id);
		input_report_abs(ts->input, ABS_MT_POSITION_X, tdata.x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, tdata.y);
		input_report_abs(ts->input, ABS_MT_PRESSURE, tdata.pressure);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
				tdata.width_major);
		input_report_abs(ts->input, ABS_MT_WIDTH_MINOR,
				tdata.width_minor);
		input_report_abs(ts->input, ABS_MT_ORIENTATION,
				tdata.orientation);
		input_sync(ts->input);

		touch_msleep(ts->perf_test.delay);
	}

	input_mt_slot(ts->input, tdata.id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);

	touch_report_all_event(ts);
	mutex_unlock(&ts->lock);
	TOUCH_I("%s: end\n", __func__);

	return ret;
}

static ssize_t show_v_drag_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	struct touch_data *tdata = NULL;
	int cnt = 800 / ts->perf_test.delay;	/* drag 800ms */
	u16 start_y = ts->perf_test.v_drag_start_y;
	u16 end_y = ts->perf_test.v_drag_end_y;
	u16 y_diff = start_y - end_y;
	int i = 0;

	TOUCH_TRACE();

	if (!ts->perf_test.enable)
		return ret;

	if (cnt < 2) {
		TOUCH_E("invalid cnt(%d)\n", cnt);
		return -EINVAL;
	}

	tdata = kcalloc(cnt, sizeof(*tdata), GFP_KERNEL);
	if (tdata == NULL) {
		TOUCH_E("failed to kcalloc tdata\n");
		return -ENOMEM;
	}

	for (i = 0; i < cnt; i++) {
		tdata[i].id = 0;
		tdata[i].x = ts->perf_test.v_drag_x;
		tdata[i].y = start_y - ((y_diff * i) / (cnt - 1));
		tdata[i].pressure = ts->perf_test.pressure;
		tdata[i].width_major = ts->perf_test.width;
		tdata[i].width_minor = ts->perf_test.width;
		tdata[i].orientation = 0;
	}

	TOUCH_I("%s: start (y: %4d -> %4d)\n", __func__, start_y, end_y);
	mutex_lock(&ts->lock);
	touch_report_all_event(ts);

	for (i = 0; i < cnt; i++) {
		input_mt_slot(ts->input, tdata[i].id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, tdata[i].id);
		input_report_abs(ts->input, ABS_MT_POSITION_X, tdata[i].x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, tdata[i].y);
		input_report_abs(ts->input, ABS_MT_PRESSURE, tdata[i].pressure);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
				tdata[i].width_major);
		input_report_abs(ts->input, ABS_MT_WIDTH_MINOR,
				tdata[i].width_minor);
		input_report_abs(ts->input, ABS_MT_ORIENTATION,
				tdata[i].orientation);
		input_sync(ts->input);

		touch_msleep(ts->perf_test.delay);
	}

	input_mt_slot(ts->input, tdata[i - 1].id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);

	touch_report_all_event(ts);
	mutex_unlock(&ts->lock);
	kfree(tdata);
	TOUCH_I("%s: end\n", __func__);

	return ret;
}

static ssize_t show_h_drag_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	struct touch_data *tdata = NULL;
	int cnt = 800 / ts->perf_test.delay;	/* drag 800ms */
	u16 start_x = ts->perf_test.h_drag_start_x;
	u16 end_x = ts->perf_test.h_drag_end_x;
	u16 x_diff = start_x - end_x;
	int i = 0;

	TOUCH_TRACE();

	if (!ts->perf_test.enable)
		return ret;

	if (cnt < 2) {
		TOUCH_E("invalid cnt(%d)\n", cnt);
		return -EINVAL;
	}

	tdata = kcalloc(cnt, sizeof(*tdata), GFP_KERNEL);
	if (tdata == NULL) {
		TOUCH_E("failed to kcalloc tdata\n");
		return -ENOMEM;
	}

	for (i = 0; i < cnt; i++) {
		tdata[i].id = 0;
		tdata[i].x = start_x - ((x_diff * i) / (cnt - 1));
		tdata[i].y = ts->perf_test.h_drag_y;
		tdata[i].pressure = ts->perf_test.pressure;
		tdata[i].width_major = ts->perf_test.width;
		tdata[i].width_minor = ts->perf_test.width;
		tdata[i].orientation = 0;
	}

	TOUCH_I("%s: start (x: %4d -> %4d)\n", __func__, start_x, end_x);
	mutex_lock(&ts->lock);
	touch_report_all_event(ts);

	for (i = 0; i < cnt; i++) {
		input_mt_slot(ts->input, tdata[i].id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, tdata[i].id);
		input_report_abs(ts->input, ABS_MT_POSITION_X, tdata[i].x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, tdata[i].y);
		input_report_abs(ts->input, ABS_MT_PRESSURE, tdata[i].pressure);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
				tdata[i].width_major);
		input_report_abs(ts->input, ABS_MT_WIDTH_MINOR,
				tdata[i].width_minor);
		input_report_abs(ts->input, ABS_MT_ORIENTATION,
				tdata[i].orientation);
		input_sync(ts->input);

		touch_msleep(ts->perf_test.delay);
	}

	input_mt_slot(ts->input, tdata[i - 1].id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);

	touch_report_all_event(ts);
	mutex_unlock(&ts->lock);
	kfree(tdata);
	TOUCH_I("%s: end\n", __func__);

	return ret;
}

/*
#if defined(CONFIG_LGE_MODULE_DETECT)
static ssize_t show_module_id(struct device *dev, char *buf)
{
	int ret = 0;
	TOUCH_I("Module id is LG4894\n");
	ret = snprintf(buf, PAGE_SIZE, "%d\n", TYPE_LG4894);

	return ret;
}
#endif CONFIG_LGE_MODULE_DETECT */

static TOUCH_ATTR(platform_data, show_platform_data, NULL);
static TOUCH_ATTR(fw_upgrade, show_upgrade, store_upgrade);
static TOUCH_ATTR(lpwg_data, show_lpwg_data, store_lpwg_data);
static TOUCH_ATTR(lpwg_notify, NULL, store_lpwg_notify);
static TOUCH_ATTR(keyguard,
	show_lockscreen_state, store_lockscreen_state);
static TOUCH_ATTR(ime_status, show_ime_state, store_ime_state);
static TOUCH_ATTR(quick_cover_status,
	show_quick_cover_state, store_quick_cover_state);
static TOUCH_ATTR(incoming_call,
	show_incoming_call_state, store_incoming_call_state);
static TOUCH_ATTR(firmware, show_version_info, NULL);
static TOUCH_ATTR(version, show_version_info, NULL);
static TOUCH_ATTR(testmode_ver, show_atcmd_version_info, NULL);
static TOUCH_ATTR(mfts, show_mfts_state, store_mfts_state);
static TOUCH_ATTR(mfts_lpwg, show_mfts_lpwg, store_mfts_lpwg);
static TOUCH_ATTR(sp_link_touch_off,
	show_sp_link_touch_off, store_sp_link_touch_off);
static TOUCH_ATTR(debug_tool, show_debug_tool_state, store_debug_tool_state);
static TOUCH_ATTR(debug_option, show_debug_option_state,
				store_debug_option_state);
static TOUCH_ATTR(app_data, show_app_data, store_app_data);
static TOUCH_ATTR(click_test, show_click_test, NULL);
static TOUCH_ATTR(v_drag_test, show_v_drag_test, NULL);
static TOUCH_ATTR(h_drag_test, show_h_drag_test, NULL);
/*
#if defined(CONFIG_LGE_MODULE_DETECT)
static TOUCH_ATTR(module_id, show_module_id, NULL);
#endif CONFIG_LGE_MODULE_DETECT */

static struct attribute *touch_attribute_list[] = {
	&touch_attr_platform_data.attr,
	&touch_attr_fw_upgrade.attr,
	&touch_attr_lpwg_data.attr,
	&touch_attr_lpwg_notify.attr,
	&touch_attr_keyguard.attr,
	&touch_attr_ime_status.attr,
	&touch_attr_quick_cover_status.attr,
	&touch_attr_incoming_call.attr,
	&touch_attr_firmware.attr,
	&touch_attr_version.attr,
	&touch_attr_testmode_ver.attr,
	&touch_attr_mfts.attr,
	&touch_attr_mfts_lpwg.attr,
	&touch_attr_sp_link_touch_off.attr,
	&touch_attr_debug_tool.attr,
	&touch_attr_debug_option.attr,
	&touch_attr_app_data.attr,
	&touch_attr_click_test.attr,
	&touch_attr_v_drag_test.attr,
	&touch_attr_h_drag_test.attr,
/*
#if defined(CONFIG_LGE_MODULE_DETECT)
	&touch_attr_module_id.attr,
#endif CONFIG_LGE_MODULE_DETECT */
	NULL,
};

static const struct attribute_group touch_attribute_group = {
	.attrs = touch_attribute_list,
};

static ssize_t touch_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct touch_core_data *ts =
		container_of(kobj, struct touch_core_data, kobj);
	struct touch_attribute *priv =
		container_of(attr, struct touch_attribute, attr);
	ssize_t ret = 0;

	if (priv->show)
		ret = priv->show(ts->dev, buf);

	return ret;
}

static ssize_t touch_attr_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct touch_core_data *ts =
		container_of(kobj, struct touch_core_data, kobj);
	struct touch_attribute *priv =
		container_of(attr, struct touch_attribute, attr);
	ssize_t ret = 0;

	if (priv->store)
		ret = priv->store(ts->dev, buf, count);

	return ret;
}

static const struct sysfs_ops touch_sysfs_ops = {
	.show	= touch_attr_show,
	.store	= touch_attr_store,
};

static struct kobj_type touch_kobj_type = {
	.sysfs_ops = &touch_sysfs_ops,
};

int touch_init_sysfs(struct touch_core_data *ts)
{
	struct device *dev = &ts->input->dev;
	int ret;

	ret = kobject_init_and_add(&ts->kobj, &touch_kobj_type,
			dev->kobj.parent, "%s", LGE_TOUCH_NAME);

	ret = sysfs_create_group(&ts->kobj, &touch_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	if (ts->driver->register_sysfs)
		ret = ts->driver->register_sysfs(dev);

	return ret;
}

