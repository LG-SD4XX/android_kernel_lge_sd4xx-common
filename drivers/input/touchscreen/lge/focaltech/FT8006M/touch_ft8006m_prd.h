/* production_test.h
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

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8006m.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* Normal Mode SET SPEC */
#define RAW_DATA_MAX		12000
#define RAW_DATA_MIN		4000
#define RAW_DATA_MARGIN		0
#define IB_MAX				65
#define IB_MIN				3
#define NOISE_MAX			210
#define NOISE_MIN			0
/* LPWG Mode SET SPEC */
#define LPWG_RAW_DATA_MAX	12000
#define LPWG_RAW_DATA_MIN	4000
#define LPWG_IB_MAX			65
#define LPWG_IB_MIN			3
#define LPWG_NOISE_MAX		210
#define LPWG_NOISE_MIN		0
#define LPWG_JITTER_MAX		210
#define LPWG_JITTER_MIN		0

#define FTS_WORK_MODE		0x00
#define FTS_FACTORY_MODE		0x40
#define FTS_MODE_CHANGE_LOOP	20

#define TEST_PACKET_LENGTH	342

/* Number of channel */
#define MAX_ROW			32
#define MAX_COL			18

#define LOG_BUF_SIZE		4095
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)

#define DELTA_ATTR_SIZE		(8 * 1024) - 500
#define RAWDATA_ATTR_SIZE	(10 * 1024) - 1100
#define LGE_ATTR_DELTA		"delta_ext"
#define LGE_ATTR_RAWDATA		"rawdata_ext"

enum {
	RAW_DATA_TEST = 0,
	IB_DATA_TEST,
	NOISE_TEST,
	DELTA_SHOW,
	LPWG_RAW_DATA_TEST,
	LPWG_IB_DATA_TEST,
	LPWG_NOISE_TEST,
	JITTER_TEST,
};

enum {
	TEST_PASS = 0,
	TEST_FAIL,
};

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

extern void touch_msleep(unsigned int msecs);
int ft8006m_prd_register_sysfs(struct device *dev);
#endif
