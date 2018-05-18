/* production_test.c
 *
 * Copyright (C) 2015 LGE.
 *
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
#define TS_MODULE "[prd]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <soc/qcom/lge/board_lge.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8006m.h"
#include "touch_ft8006m_prd.h"

static s16 fts_data[MAX_ROW][MAX_COL];
static u8 i2c_data[MAX_COL*MAX_ROW*2];
static u8 log_buf[LOG_BUF_SIZE + 1];		/* !!!!!!!!!Should not exceed the log size !!!!!!!! */

static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",
			__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",
						__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d",
							fname,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
								__func__, buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}
static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			snprintf(time_string, 64,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

int ft8006m_change_op_mode(struct device *dev, u8 op_mode)
{
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int i = 0;
	u8 ret;
	u8 data;

	TOUCH_I("%s : op_mode = 0x%02x\n", __func__, op_mode);

	data = 0x00;
	ret = ft8006m_reg_read(dev, 0x00, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == op_mode) {
		TOUCH_I("Already mode changed\n");
		return 0;
	}

	data = op_mode;
	ret = ft8006m_reg_write(dev, 0x00, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	mdelay(10);

	for ( i = 0; i < FTS_MODE_CHANGE_LOOP; i++) {
		data = 0x00;
		ret = ft8006m_reg_read(dev, 0x00, &data, 1);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		if(data == op_mode)
			break;
		mdelay(50);
	}

	if (i >= FTS_MODE_CHANGE_LOOP) {
		TOUCH_E("Timeout to change op mode\n");
		return -EPERM;
	}
	TOUCH_I("Operation mode changed\n");

//	mdelay(300);
	mdelay(100);
	TOUCH_I("d->state = %d\n", d->state);
	if((op_mode == FTS_FACTORY_MODE) && (d->state == TC_STATE_LPWG)) {
		TOUCH_I("[0x88] = 0x01 write\n");
		data = 0x01;
		ret = ft8006m_reg_write(dev, 0x88, &data, 1);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
	}

	return 0;
}


int ft8006m_switch_cal(struct device *dev, u8 cal_en)
{
#if 0
	int i = 0;
	u8 ret;
	u8 data;

	TOUCH_I("%s : cal_en = 0x%02x\n", __func__, cal_en);

	data = 0x00;
	ret = ft8006m_reg_read(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == cal_en) {
		TOUCH_I("Already cal_en changed\n");
		return 0;
	}

	data = cal_en;
	ret = ft8006m_reg_write(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	mdelay(10);

	for ( i = 0; i < FTS_MODE_CHANGE_LOOP; i++) {
		data = 0x00;
		ret = ft8006m_reg_read(dev, 0xEE, &data, 1);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		if(data == cal_en)
			break;
		mdelay(20);
	}

	if (i >= FTS_MODE_CHANGE_LOOP) {
		TOUCH_E("Timeout to change cal_en\n");
		return -EPERM;
	}
	TOUCH_I("cal_en changed\n");

	return 0;

#else
	u8 ret;
	u8 data;

	TOUCH_I("%s : cal_en = 0x%02x\n", __func__, cal_en);

	ret = ft8006m_reg_read(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == cal_en) {
		TOUCH_I("Already switch_cal changed\n");
		return 0;
	}

	data = cal_en;
	ret = ft8006m_reg_write(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	mdelay(100);

	return 0;
#endif
}


int ft8006m_prd_check_ch_num(struct device *dev)
{
	u8 ret;
	u8 data;

	TOUCH_I("%s\n", __func__);

	/* Channel number check */
	ret = ft8006m_reg_read(dev, 0x02, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	TOUCH_I("X Channel : %d\n", data);

	mdelay(3);

	if (data != MAX_COL) {
		TOUCH_E("Invalid X Channel Num.\n");
		return -EPERM;
	}

	ret = ft8006m_reg_read(dev, 0x03, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	TOUCH_I("Y Channel : %d\n", data);

	mdelay(3);

	if (data != MAX_ROW) {
		TOUCH_E("Invalid Y Channel Num.\n");
		return -EPERM;
	}

	return 0;
}

int ft8006m_prd_get_raw_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
	u8 rawdata = 0x00;
	bool bscan = false;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to raw data
	data = 0x00;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	for (k = 0; k < 3; k++)
	{
		TOUCH_I("Start SCAN (%d/3)\n", k+1);
		memset(i2c_data, 0, sizeof(i2c_data));
		ret = ft8006m_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		data |= 0x80; // 0x40|0x80
		ret = ft8006m_reg_write(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(20);

		for (i = 0; i < 5; i++) {
			ret = ft8006m_reg_read(dev, 0x00, &data, 1);
			if (ret < 0) {
				TOUCH_E("i2c error\n");
				return ret;
			}
			if (data == 0x40) {
				TOUCH_I("SCAN Success : 0x%X, %d ms \n", data, i*20);
				bscan = true;
				break;
			} else {
				bscan = false;
				mdelay(20);
			}
		}

		if(bscan==true){
			/* Read Raw data */
			rawdata = 0xAD;
			ret = ft8006m_reg_write(dev, 0x01, &rawdata, 1);
			if(ret < 0) {
				TOUCH_E("i2c error\n");
				return ret;
			}
			mdelay(10);

			TOUCH_I("Read Raw data at once\n");

			ret = ft8006m_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
			if(ret < 0) {
				TOUCH_E("i2c error\n");
				return ret;
			}
		}else {
			TOUCH_E("SCAN Fail (%d/3)\n", k+1);
		}
	}

#if 0
//// K7 Screen display sorting
	/* Combine */
	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}
#else
//// K6 Screen display sorting
             for (i = 0; i < MAX_ROW; i++) {
                           for(j = 0; j < MAX_COL; j++) {
                                        k = (((MAX_COL- 1-j) * MAX_ROW) + i) << 1;
                                        fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
                           }
             }
#endif

	return 0;
}


int ft8006m_prd_get_noise_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
//	u8 frame_count;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select 1: diff 0: rawdata
	data = 0x01;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Set Noise Test Frame Count
	//low  byte
	data = 0x32;
	ret = ft8006m_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

	//high  byte
	data = 0x00;
	ret = ft8006m_reg_write(dev, 0x13, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

	// Start Noise Test
	data = 0x00; //0x01;
	ret = ft8006m_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Raw Data Addr
	data = 0xAD;
	ret = ft8006m_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Scan is finished
	for (i = 0; i < 100; i++)
	{
		ret = ft8006m_reg_read(dev, 0x11, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if ((data & 0xff) == 0x00){
			TOUCH_I("Scan finished : %d ms, data = %x\n", i*50 ,data);
			break;
		}
		mdelay(50); //mdelay(20);
	}

	if (i >= 100) {
		TOUCH_E("Scan failed\n");
		return -EPERM;
	}

	// Get Noise data
	TOUCH_I("Read Noise data at once\n");

	// (Get RMS data)->(Get MaxNoise Data)
	ret = ft8006m_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

#if 0
	// Get Noise Test Frame Count
	ret = ft8006m_reg_read(dev, 0x13, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	frame_count = data;
	if (frame_count == 0) {
		TOUCH_E("Invalid frame count ZERO\n");
		return -EPERM;
	}
	TOUCH_I("Frame count : %d\n", frame_count);

	mdelay(10);

	// Data Select to raw
	data = 0x00;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
#endif

#if 0
//// K7 Screen display sorting
	// Calculate STDEV of Noise
	for (i = 0; i < MAX_ROW; i++) {
		for (j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
// (Get RMS data)
//			fts_data[i][j] = ((((int)i2c_data[k] << 8) + (int)i2c_data[k+1])) / (int)(frame_count);
//			fts_data[i][j] = (unsigned long)int_sqrt((unsigned long)fts_data[i][j]);
// (Get MaxNoise Data)
			fts_data[i][j] = ((((int)i2c_data[k] << 8) + (int)i2c_data[k+1]));
		}
	}
#else
//// K6 Screen display sorting
             for (i = 0; i < MAX_ROW; i++) {
                           for(j = 0; j < MAX_COL; j++) {
                                        k = (((MAX_COL- 1-j) * MAX_ROW) + i) << 1;
                                        fts_data[i][j] = abs((i2c_data[k] << 8) + i2c_data[k+1]);
                           }
             }
#endif

	return 0;
}

int ft8006m_prd_get_jitter_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
//	u8 frame_count;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Set Jitter Test Frame Count
	//low  byte
	data = 0x32;
	ret = ft8006m_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

	//high  byte
	data = 0x00;
	ret = ft8006m_reg_write(dev, 0x13, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

	// Start Jitter Test
	data = 0x02;//0x01;
	ret = ft8006m_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Raw Data Addr
	data = 0xAD;
	ret = ft8006m_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Scan is finished
	for (i = 0; i < 100; i++)
	{
		ret = ft8006m_reg_read(dev, 0x11, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if ((data & 0xff) == 0x00){
			TOUCH_I("Scan finished : %d ms, data = %x\n", i*50 ,data);
			break;
		}

		mdelay(50); //mdelay(20);
	}

	if (i >= 100) {
		TOUCH_E("Scan failed\n");
		return -EPERM;
	}

	// Get Jitter data
	TOUCH_I("Read Jitter data at once\n");

	// (Get RMS data)->(Get MaxJitter Data)
	ret = ft8006m_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);


//// K6 Screen display sorting
	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = (((MAX_COL- 1-j) * MAX_ROW) + i) << 1;
			fts_data[i][j] = abs((i2c_data[k] << 8) + i2c_data[k+1]);
		}
	}

	return 0;
}


int ft8006m_prd_get_ib_data(struct device *dev)
{
	int i, j;
	int ret = 0;
	u8 data = 0x00;
	int total, offset = 0, read_len;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to raw data
	data = 0x00;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	total = MAX_COL*MAX_ROW;

	// Get IB data
	for (i = 0; (total - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		offset = TEST_PACKET_LENGTH * i;
		read_len = ((total - offset) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (total - offset);

		data = (offset & 0xFF00) >> 8;
		ret = ft8006m_reg_write(dev, 0x18, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		data = (offset & 0x00FF);
		ret = ft8006m_reg_write(dev, 0x19, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		ret = ft8006m_reg_read(dev, 0x6E, &i2c_data[offset], read_len);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);
	}

#if 0
//// K7 Screen display sorting
	for (i = 0; i < MAX_ROW; i++) {
		for (j = 0; j < MAX_COL; j++) {
			fts_data[i][j] = i2c_data[j * MAX_ROW + i];
		}
	}
#else
//// K6 Screen display sorting
             for (i = 0; i < MAX_ROW; i++) {
                           for(j = 0; j < MAX_COL; j++) {
                                        fts_data[i][j] = i2c_data[((MAX_COL- 1-j) * MAX_ROW) + i];
                           }
             }
#endif

	return 0;
}

int ft8006m_prd_get_delta_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
	//int total, offset = 0, read_len;

	//total = MAX_COL*MAX_ROW;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to diff data
	data = 0x01;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	/* Start SCAN */
	for (k = 0; k < 3; k++)
	{
		TOUCH_I("Start SCAN (%d/3)\n", k+1);
		ret = ft8006m_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		data |= 0x80; // 0x40|0x80
		ret = ft8006m_reg_write(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);

		for (i = 0; i < 200; i++) {
			ret = ft8006m_reg_read(dev, 0x00, &data, 1);
			if (ret < 0) {
				TOUCH_E("i2c error\n");
				return ret;
			}
			if (data == 0x40) {
				TOUCH_I("SCAN Success : 0x%X, %d ms \n", data, i*20);
				break;
			}
			mdelay(20);
		}

		if (i < 200) {
			break;
		}

		TOUCH_E("SCAN Fail (%d/3)\n", k+1);
	}

	if (k >= 3) {
		return -EPERM;
	}

	/* Read Raw data */
	data = 0xAD;
	ret = ft8006m_reg_write(dev, 0x01, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

#if 1 // read full data at once

	TOUCH_I("Read Delta at once\n");

	ret = ft8006m_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

#else // read packet_length data
	TOUCH_I("Read Delta at once\n");

	for (i = 0; (total - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		offset = TEST_PACKET_LENGTH * i;
		read_len = ((total - offset) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (total - offset);

		data = (offset & 0xFF00) >> 8;
		ret = ft8006m_reg_write(dev, 0x1C, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		data = (offset & 0x00FF);
		ret = ft8006m_reg_write(dev, 0x1D, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		ret = ft8006m_reg_read(dev, 0x6A, &i2c_data[offset], read_len);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);
	}
#endif

#if 0
	// Data Select
	data = 0x00;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
#endif

#if 0
//// K7 Screen display sorting

             /* Combine */
             for (i = 0; i < MAX_ROW; i++) {
                           for(j = 0; j < MAX_COL; j++) {
                                        k = ((j * MAX_ROW) + i) << 1;
                                        fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
                           }
             }
#else
//// K6 Screen display sorting
             for (i = 0; i < MAX_ROW; i++) {
                           for(j = 0; j < MAX_COL; j++) {
                                        k = (((MAX_COL- 1-j) * MAX_ROW) + i) << 1;
                                        fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
                           }
             }
#endif

	return 0;
}


int ft8006m_prd_test_data(struct device *dev, int test_type, int* test_result)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);

	int i, j;
	int ret = 0;

	int limit_upper = 0, limit_lower = 0;
	int min, max/*, aver, stdev*/;
	int fail_count = 0;
	int check_limit = 1;

	*test_result = TEST_FAIL;

	ret = snprintf(log_buf, LOG_BUF_SIZE, "IC F/W Version : V%d.%02d\n", d->ic_info.is_official, d->ic_info.fw_version);

	switch (test_type) {
		case RAW_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Raw Data Test Result =============\n");
			limit_upper = RAW_DATA_MAX + RAW_DATA_MARGIN;
			limit_lower = RAW_DATA_MIN - RAW_DATA_MARGIN;
			break;
		case IB_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= IB Data Test Result =============\n");
			limit_upper = IB_MAX;
			limit_lower = IB_MIN;
			break;
		case NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Noise Test Result =============\n");
			limit_upper = NOISE_MAX;
			limit_lower = NOISE_MIN;
			break;
		case DELTA_SHOW:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Delta Result =============\n");
			check_limit = 0;
			break;
		case LPWG_RAW_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Raw Data Test Result =============\n");
			limit_upper = LPWG_RAW_DATA_MAX;
			limit_lower = LPWG_RAW_DATA_MIN;
			break;
		case LPWG_IB_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG IB Data Test Result =============\n");
			limit_upper = LPWG_IB_MAX;
			limit_lower = LPWG_IB_MIN;
			break;
		case LPWG_NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Noise Test Result =============\n");
			limit_upper = LPWG_NOISE_MAX;
			limit_lower = LPWG_NOISE_MIN;
			break;
		case JITTER_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Jitter Test Result =============\n");
			limit_upper = LPWG_JITTER_MAX;
			limit_lower = LPWG_JITTER_MIN;
			break;
		default:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test Failed (Invalid test type)\n");
			return ret;
	}

	max = min = fts_data[0][0];

	for (i = 0; i < MAX_ROW; i++) {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i+1);
		for (j = 0; j < MAX_COL; j++) {

			if (test_type == RAW_DATA_TEST || test_type == LPWG_RAW_DATA_TEST) {
				ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "%5d ", fts_data[i][j]);
			}
			else {
				ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "%4d ", fts_data[i][j]);
			}

			if (check_limit && (fts_data[i][j] < limit_lower || fts_data[i][j] > limit_upper)) {
				fail_count++;
			}
			if (fts_data[i][j] < min)
				min = fts_data[i][j];
			if (fts_data[i][j] > max)
				max = fts_data[i][j];
		}
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "\n");
	}

	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "==================================================\n");

	if(fail_count && check_limit) {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test FAIL : %d Errors\n", fail_count);
	}
	else {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test PASS : No Errors\n");
		*test_result = TEST_PASS;
	}

	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "MAX = %d, MIN = %d, Upper = %d, Lower = %d\n\n", max, min, limit_upper, limit_lower);

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int test_result;

	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show_sd is called in NOT Active state\n");
		return 0;
	}

	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	/* file create , time log */
	TOUCH_I("Show_sd Test Start\n");
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8006m_switch_cal(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Change to factory mode
	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	// Start to raw data test
	TOUCH_I("Show_sd : Raw data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8006m_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;

	ret_size = ft8006m_prd_test_data(dev, RAW_DATA_TEST, &test_result);

	TOUCH_I("Raw Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	if(test_result == TEST_FAIL) {
		goto FAIL;
	}

	msleep(30);

	// Start to IB data test
	TOUCH_I("Show_sd : IB data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8006m_prd_get_ib_data(dev);
	if(ret < 0)
		goto FAIL;

	ret_size = ft8006m_prd_test_data(dev, IB_DATA_TEST, &test_result);

	TOUCH_I("IB Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	if(test_result == TEST_FAIL) {
		goto FAIL;
	}

	msleep(30);
	if (d->ic_info.chip_id_low == 0x06) { //0x07 for FT86XX/FT87xx, 0x06 for FT80xx etc
		// Start to noise test
		TOUCH_I("Show_sd : Noise test\n");

		memset(log_buf, 0, LOG_BUF_SIZE);

		// Reset ???
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);

		ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
		if(ret < 0)
			goto FAIL;

		ret = ft8006m_prd_get_noise_data(dev);
		if(ret < 0)
			goto FAIL;

		ret_size = ft8006m_prd_test_data(dev, NOISE_TEST, &test_result);

		TOUCH_I("Noise Test Result : %d\n", test_result);

		////memcpy(buf + ret_total_size, log_buf, ret_size);
		////ret_total_size += ret_size;
		//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

		write_file(dev, log_buf, TIME_INFO_SKIP);

		if(test_result == TEST_FAIL) {
			goto FAIL;
		}

		msleep(30);
	}

	// Change to working mode
	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	// Test result
	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Pass\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Channel Status : Pass\n"); // ???????

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("Raw Data : Pass\n");
	TOUCH_I("Channel Status : Pass\n"); // ???????
	TOUCH_I("Show_sd Test End\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;

FAIL:

	// Change to working mode
	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		TOUCH_I("Failed to return WORK_MODE\n");

	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Fail\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Channel Status : Fail\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("Raw Data : Fail\n");
	TOUCH_I("Channel Status : Fail\n"); // ???????
	TOUCH_I("Show_sd Test End\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Delta Data\n");
//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Change clb switch
	ret = ft8006m_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8006m_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8006m_prd_get_delta_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8006m_switch_cal(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	TOUCH_I("Show Delta Data OK !!!\n");

	ret_size = ft8006m_prd_test_data(dev, DELTA_SHOW, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Show Delta Data Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Delta Data FAIL !!!\n");
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;
	u8 data = 0x00;


	TOUCH_I("Show Raw Data\n");

//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
#if 1
	// Change clb switch
	ret = ft8006m_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	//ret = ft8006m_prd_check_ch_num(dev);
	//if(ret < 0)
	//	goto FAIL;
#endif

	ret = ft8006m_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;

	// Data Select to raw data
	data = 0x01;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);

	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Raw Data OK !!!\n");

	ret_size = ft8006m_prd_test_data(dev, RAW_DATA_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Raw Data Test Result : %d, data size: %d\n", test_result, ret_size);
	//printk("%s\n", log_buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Raw Data FAIL !!!\n");
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Noise\n");

//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Reset ???
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	//ret = ft8006m_prd_check_ch_num(dev);
	//if(ret < 0)
	//	goto FAIL;

	ret = ft8006m_prd_get_noise_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Noise OK !!!\n");

	ret_size = ft8006m_prd_test_data(dev, NOISE_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Noise Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	mutex_unlock(&ts->lock);
	if(d->state != TC_STATE_ACTIVE) {
		ft8006m_lpwg_set(dev);
	}
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Noise FAIL !!!\n");
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	mutex_unlock(&ts->lock);
	if(d->state != TC_STATE_ACTIVE) {
		ft8006m_lpwg_set(dev);
	}
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_jitter(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Jitter\n");

//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Reset ???
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	//ret = ft8006m_prd_check_ch_num(dev);
	//if(ret < 0)
	//	goto FAIL;

	ret = ft8006m_prd_get_jitter_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show jitter OK !!!\n");

	ret_size = ft8006m_prd_test_data(dev, JITTER_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Jitter Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	mutex_unlock(&ts->lock);

	if(d->state != TC_STATE_ACTIVE) {
		ft8006m_lpwg_set(dev);
	}
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Noise FAIL !!!\n");
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	mutex_unlock(&ts->lock);
	if(d->state != TC_STATE_ACTIVE) {
		ft8006m_lpwg_set(dev);
	}
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}


static ssize_t show_ib(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;
	u8 data = 0x00;

	TOUCH_I("Show IB Data\n");

//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Change clb switch
	ret = ft8006m_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8006m_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8006m_prd_get_ib_data(dev);
	if(ret < 0)
		goto FAIL;

	// Data Select to raw data
	data = 0x01;
	ret = ft8006m_reg_write(dev, 0x06, &data, 1);

	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show IB Data OK !!!\n");

	ret_size = ft8006m_prd_test_data(dev, IB_DATA_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("IB Data Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show IB Data FAIL !!!\n");
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}


static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int test_result;

	// Check Current State
	if(d->state == TC_STATE_ACTIVE) {
		TOUCH_E("Show_lpwg_sd called in Active state\n");
		return 0;
	}

	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	/* file create , time log */
	TOUCH_I("Show_lpwg_sd Test Start\n");
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8006m_switch_cal(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Change to factory mode
	ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	// Start to raw data test
	TOUCH_I("Show_lpwg_sd : LPWG Raw data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8006m_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8006m_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;

	ret_size = ft8006m_prd_test_data(dev, LPWG_RAW_DATA_TEST, &test_result);

	TOUCH_I("LPWG Raw Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	if(test_result == TEST_FAIL) {
		goto FAIL;
	}

	msleep(30);

	// Start to IB data test
	TOUCH_I("Show_lpwg_sd : LPWG IB data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8006m_prd_get_ib_data(dev);
	if(ret < 0)
		goto FAIL;

	ret_size = ft8006m_prd_test_data(dev, LPWG_IB_DATA_TEST, &test_result);

	TOUCH_I("LPWG IB Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	if(test_result == TEST_FAIL) {
		goto FAIL;
	}

	msleep(30);
	if (d->ic_info.chip_id_low == 0x06) {
		// Start to noise test
		TOUCH_I("Show_lpwg_sd : LPWG Noise test\n");

		memset(log_buf, 0, LOG_BUF_SIZE);

		// Reset ???
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);

		ret = ft8006m_change_op_mode(dev, FTS_FACTORY_MODE);
		if(ret < 0)
			goto FAIL;

		ret = ft8006m_prd_get_noise_data(dev);
		if(ret < 0)
			goto FAIL;

		ret_size = ft8006m_prd_test_data(dev, LPWG_NOISE_TEST, &test_result);

		TOUCH_I("LPWG Noise Test Result : %d\n", test_result);

		////memcpy(buf + ret_total_size, log_buf, ret_size);
		////ret_total_size += ret_size;
		//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

		write_file(dev, log_buf, TIME_INFO_SKIP);

		if(test_result == TEST_FAIL) {
			goto FAIL;
		}

		msleep(30);
	}

	// Change to working mode
	ret = ft8006m_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	// Test result
	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Pass\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("LPWG RawData : Pass\n");
	TOUCH_I("Show_lpwg_sd Test End\n");

	mutex_unlock(&ts->lock);
	if(d->state != TC_STATE_ACTIVE) {
		ft8006m_lpwg_set(dev);
	}
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;

FAIL:

	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Fail\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("LPWG RawData : Fail\n");
	TOUCH_I("Show_lpwg_sd Test End\n");

	mutex_unlock(&ts->lock);

	if(d->state != TC_STATE_ACTIVE) {
		ft8006m_lpwg_set(dev);
	}
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(noise_test, show_noise, NULL);
static TOUCH_ATTR(ib_test, show_ib, NULL);
static TOUCH_ATTR(jitter, show_jitter, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_noise_test.attr,
	&touch_attr_ib_test.attr,
	&touch_attr_jitter.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};



static ssize_t prd_delta_attr_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct ft8006m_data *d = to_ft8006m_data(ts->dev);

	ssize_t retval = -EFAULT;
	int ret = 0;
	int test_result;
	int ret_size = 0;
	//int i = 0;
	//char *test_string = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {
		if (d->prd_delta_data == NULL) {
			TOUCH_I("%s : memory NULL\n", __func__);
			d->prd_delta_data = kzalloc(DELTA_ATTR_SIZE, GFP_KERNEL);
			if (d->prd_delta_data) {
				TOUCH_I("%s font_buffer(%d KB) malloc success\n", __func__,
					DELTA_ATTR_SIZE/1024);
			} else {
				TOUCH_I("%s : memory alloc failed\n", __func__);
				goto error;
			}
		}
			TOUCH_I("Show Delta Data\n");
			//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			mutex_lock(&ts->lock);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

			// Change clb switch
			ret = ft8006m_switch_cal(ts->dev, 0);


			ret = ft8006m_change_op_mode(ts->dev, FTS_FACTORY_MODE);



			ret = ft8006m_prd_get_delta_data(ts->dev);


			ret = ft8006m_change_op_mode(ts->dev, FTS_WORK_MODE);


			TOUCH_I("Show Delta Data OK !!!\n");

			ret_size = ft8006m_prd_test_data(ts->dev, DELTA_SHOW, &test_result);
			memcpy(d->prd_delta_data, log_buf, ret_size);
			TOUCH_I("Show Delta Data Result : %d\n", test_result);
			//printk("%s\n", log_buf);
			touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
			mutex_unlock(&ts->lock);
		//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	}

	if (off + count > DELTA_ATTR_SIZE) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__,
			(int)off, (int)count);
	} else {
		memcpy(buf, &d->prd_delta_data[off], count);
		retval = count;
	}

error :
	return retval;
}

static ssize_t prd_rawdata_attr_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct ft8006m_data *d = to_ft8006m_data(ts->dev);

	ssize_t retval = -EFAULT;
	int ret = 0;
	int test_result;
	int ret_size = 0;
	u8 data = 0x00;
	//int i = 0;
	//char *test_string = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {
		if (d->prd_rawdata_data == NULL) {
			TOUCH_I("%s : memory NULL\n", __func__);
			d->prd_rawdata_data = kzalloc(RAWDATA_ATTR_SIZE, GFP_KERNEL);
			if (d->prd_rawdata_data) {
				TOUCH_I("%s font_buffer(%d KB) malloc success\n", __func__,
					RAWDATA_ATTR_SIZE/1024);
			} else {
				TOUCH_I("%s : memory alloc failed\n", __func__);
				goto error;
			}
		}
			TOUCH_I("Show Raw Data\n");

			//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			mutex_lock(&ts->lock);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

			ret = ft8006m_switch_cal(ts->dev, 0);

			ret = ft8006m_change_op_mode(ts->dev, FTS_FACTORY_MODE);

			ret = ft8006m_prd_get_raw_data(ts->dev);
			// Data Select to raw data
			data = 0x01;
			ret = ft8006m_reg_write(ts->dev, 0x06, &data, 1);

			ret = ft8006m_change_op_mode(ts->dev, FTS_WORK_MODE);

			TOUCH_I("Show Raw Data OK !!!\n");

			ret_size = ft8006m_prd_test_data(ts->dev, RAW_DATA_TEST, &test_result);
			memcpy(d->prd_rawdata_data, log_buf, ret_size);
			TOUCH_I("Raw Data Test Result : %d\n", test_result);
			//printk("%s\n", log_buf);
			touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
			mutex_unlock(&ts->lock);
		//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	}

	if (off + count > RAWDATA_ATTR_SIZE) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__,
			(int)off, (int)count);
	} else {
		memcpy(buf, &d->prd_rawdata_data[off], count);
		retval = count;
	}

error :
	return retval;
}

static int ft8006m_prd_attr_delta_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);

	TOUCH_I("%s\n", __func__);

	d->prd_delta_data = kzalloc(DELTA_ATTR_SIZE, GFP_KERNEL);
	if (d->prd_delta_data) {
		TOUCH_I("%s font_buffer(%d KB) malloc success\n", __func__,
			DELTA_ATTR_SIZE/1024);
	} else {
		TOUCH_E("%s font_buffer(%d KB) malloc failed\n", __func__,
			DELTA_ATTR_SIZE/1024);
		return 1;
	}

	sysfs_bin_attr_init(&d->prd_delta_attr);
	d->prd_delta_attr.attr.name = LGE_ATTR_DELTA;
	d->prd_delta_attr.attr.mode = S_IWUSR | S_IRUSR;
	d->prd_delta_attr.read = prd_delta_attr_read;
	d->prd_delta_attr.size = DELTA_ATTR_SIZE;

	if (sysfs_create_bin_file(&ts->kobj, &d->prd_delta_attr) < 0)
		TOUCH_E("Failed to create %s\n", d->prd_delta_attr.attr.name);
	else
		TOUCH_I("%s success\n", __func__);

	return 0;
}

static int ft8006m_prd_attr_rawdata_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8006m_data *d = to_ft8006m_data(dev);

	TOUCH_I("%s\n", __func__);

	d->prd_rawdata_data = kzalloc(RAWDATA_ATTR_SIZE, GFP_KERNEL);
	if (d->prd_rawdata_data) {
		TOUCH_I("%s font_buffer(%d KB) malloc success\n", __func__,
			RAWDATA_ATTR_SIZE/1024);
	} else {
		TOUCH_E("%s font_buffer(%d KB) malloc failed\n", __func__,
			RAWDATA_ATTR_SIZE/1024);
		return 1;
	}

	sysfs_bin_attr_init(&d->prd_rawdata_attr);
	d->prd_rawdata_attr.attr.name = LGE_ATTR_RAWDATA;
	d->prd_rawdata_attr.attr.mode = S_IWUSR | S_IRUSR;
	d->prd_rawdata_attr.read = prd_rawdata_attr_read;
	d->prd_rawdata_attr.size = RAWDATA_ATTR_SIZE;

	if (sysfs_create_bin_file(&ts->kobj, &d->prd_rawdata_attr) < 0)
		TOUCH_E("Failed to create %s\n", d->prd_rawdata_attr.attr.name);
	else
		TOUCH_I("%s success\n", __func__);

	return 0;
}

int ft8006m_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	ft8006m_prd_attr_delta_init(dev);
	ft8006m_prd_attr_rawdata_init(dev);

	return ret;
}
