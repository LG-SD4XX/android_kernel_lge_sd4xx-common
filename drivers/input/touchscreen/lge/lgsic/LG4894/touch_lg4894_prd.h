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
#include "touch_lg4894.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

enum PRD_DEBUG {
	PRD_DEBUG_NONE	= 0,
	SHORT_TEST		= (1U << 0),	/* 1 */
	OPEN_TEST			= (1U << 1),	/* 2 */
	RAWDATA_TEST		= (1U << 2),	/* 4 */
	AVERAGE_TEST		= (1U << 3),	/* 8 */
	 /* 16*/
	 /* 32 */
	 /* 64 */
	 /* 128 */
	 /* 256 */
	 /* 512 */
	 /* 1024 */
	 /* 2048 */
	 /* 4096 */
	 /* 8192 */
	 /* 4096 */
	 /* 16384 */
	 /* 32768 */
	 /* 65536 */
};

/* PRD_D mask value
 * usage: echo 4 > /sys/module/touch_lg4894_prd/parameters/prd_debug_mask
 */
extern u32 prd_debug_mask;
#define PRD_D(condition, fmt, args...)			\
	do {							\
		if (unlikely(prd_debug_mask & (condition)))	\
			pr_info("[Touch][PRD] " fmt, ##args);	\
	} while (0)

/* production test */
#define tc_test_mode_ctl			(0xC6E)
#define cmd_test_exit			(0x0000)
#define cmd_test_enter			(0x0001)

#define tc_tsp_test_ctl			(0xC04)
#define tc_tsp_test_sts			(0x265)
#define tc_tsp_test_pf_result		(0x266)
#define tc_tsp_test_off_info		(0x2FB)
#define tc_tsp_test_data_offset	(0x07B)
#define tc_tsp_data_access_addr	(0x301)

#define RAWDATA_OFFSET			(0xE00)
#define rawdata_ctl_read			(0x2A4)
#define rawdata_ctl_write		(0xC49)

/* Product test */
#define Serial_Data_Offset		(0x07B) // tc_tsp_data_access_addr
#define DATA_BASE_ADDR 			(0x301) // tc_tsp_data_access_addr
#define prod_m1_m2_raw_offset		(0x286)
#define prod_open3_short_offset	(0x288)
#define tune_result_offset		(0x289)
#define M1_M2_RAWDATA_TEST_CNT_MAX	(2)
#define M1_M2_RAWDATA_TEST_CNT	(1)
#define LINE_FILTER_OPTION		(0x40000)

/* Firmware debugging */
/*
#define AIT_RAW_DATA_OFFSET		(0xA02)//(0xC0F)//(0xA02)
#define DELTA_DATA_OFFSET 		(0xC42)//(0xD95)//(0xC42)
#define LABLE_DATA_OFFSET			(0xD96)//(0xE83)//(0xD96)
#define AIT_BASE_DATA_OFFSET 		(0xB22)//(0xCD2)//(0xB22)
#define AIT_DEBUG_BUF_DATA_OFFSET	(0x9CE)//(0xA8C)//(0x9CE)
*/
#define ADDR_CMD_REG_SIC_IMAGECTRL_TYPE		(0x0C6C)
#define ADDR_CMD_REG_SIC_GETTER_READYSTATUS	(0x0C64)

/* tune code */
#define tc_tune_code_size		259 /* 260  - 1 */
#define tc_total_row			32
#define tc_total_col			18
#define TSP_TUNE_CODE_L_GOFT_OFFSET		0
#define TSP_TUNE_CODE_L_M1_OFT_OFFSET		2
#define TSP_TUNE_CODE_L_G1_OFT_OFFSET		(TSP_TUNE_CODE_L_M1_OFT_OFFSET + tc_total_row)
#define TSP_TUNE_CODE_L_G2_OFT_OFFSET		(TSP_TUNE_CODE_L_G1_OFT_OFFSET + tc_total_row)
#define TSP_TUNE_CODE_L_G3_OFT_OFFSET		(TSP_TUNE_CODE_L_G2_OFT_OFFSET + tc_total_row)
#define TSP_TUNE_CODE_R_GOFT_OFFSET		(TSP_TUNE_CODE_L_G3_OFT_OFFSET + tc_total_row)
#define TSP_TUNE_CODE_R_M1_OFT_OFFSET		(TSP_TUNE_CODE_R_GOFT_OFFSET + 2)
#define TSP_TUNE_CODE_R_G1_OFT_OFFSET		(TSP_TUNE_CODE_R_M1_OFT_OFFSET + tc_total_row)
#define TSP_TUNE_CODE_R_G2_OFT_OFFSET		(TSP_TUNE_CODE_R_G1_OFT_OFFSET + tc_total_row)
#define TSP_TUNE_CODE_R_G3_OFT_OFFSET		(TSP_TUNE_CODE_R_G2_OFT_OFFSET + tc_total_row)
#define PATH_SIZE			64
#define BURST_SIZE			512
#define RAWDATA_SIZE		2

#define M1_COL_SIZE			2
#define SHORT_COL_SIZE		4
#define LOG_BUF_SIZE		256
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define DEBUG_ROW_SIZE		ROW_SIZE
#define DEBUG_COL_SIZE		COL_SIZE

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

/* AIT IT_IMAGE CMD */
enum {
	CMD_NONE = 0,
	CMD_RAWDATA,
	CMD_BASE_DATA,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_FILTERED_DELTA,		// Not used
	CMD_RESERVED,			// Not used
	CMD_DEBUGDATA,
	DONT_USE_CMD = 0xEE,
	CMD_WAIT = 0xFF,
};

enum {
	NO_TEST = 0,
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,		//type_temp = 0x2;
	SHORT_NODE_TEST,		//type_temp = 0x3;
	U3_M2_RAWDATA_TEST,	//type_temp = 0x5;
	U3_M1_RAWDATA_TEST,	//type_temp = 0x6;
	U0_M2_RAWDATA_TEST,	//type_temp = 0x5;
	U0_M1_RAWDATA_TEST,	//type_temp = 0x6;
	U3_JITTER_TEST, 		//type_temp = 0xC;
	U0_JITTER_TEST, 		//type_temp = 0xC;
	U3_BLU_JITTER_TEST,		//type_temp = 0xC;
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

enum {
	LGD = 0,
	TOVIS,
};

typedef enum
{
    RS_READY	= 0xA0,
    RS_NONE	= 0x05,
    RS_LOG		= 0x77,
    RS_IMAGE	= 0xAA
} eProtocolReadyStatus_t;

struct prd_param {
	char *product_id;
	int row;
	int col;
	int col_add;
	/* debug offset */
	int rawdata_offset;
	int base_offset;
	int delta_offset;
	int label_offset;
	int debug_offset;
	int vendor_id;
};

extern void touch_msleep(unsigned int msecs);
int lg4894_prd_register_sysfs(struct device *dev);

/* For BLU test. We need to control backlight level. */
#if defined(CONFIG_TOUCHSCREEN_MTK)
extern unsigned int mt_get_bl_brightness(void);
extern int mt65xx_leds_brightness_set(int, int);
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int mdss_fb_get_bl_brightness_extern(void);
extern void mdss_fb_set_bl_brightness_extern(int);
#endif
extern void prd_info_set(struct device *dev);

#endif

