/*
 * Copyright (C) 2011 LG Electronics Inc.
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

#ifndef __LGE_BATTERY_ID_H__
#define __LGE_BATTERY_ID_H__
enum cell_type {
	LGC_LLL,
	TCD_AAC
};

enum {
	BATT_ID_UNKNOWN         = 0,
	BATT_ID_DEFAULT         = 1,
	BATT_ID_DS2704_N        = 17,
	BATT_ID_DS2704_L        = 32,
	BATT_ID_DS2704_C        = 48,
	BATT_ID_ISL6296_N       = 73,
	BATT_ID_ISL6296_L       = 94,
	BATT_ID_ISL6296_C       = 105,
	BATT_ID_ISL6296A_N      = 110,
	BATT_ID_ISL6296A_L      = 115,
	BATT_ID_ISL6296A_C      = 120,
	BATT_ID_RA4301_VC0      = 130,
	BATT_ID_RA4301_VC1      = 147,
	BATT_ID_RA4301_VC2      = 162,
	BATT_ID_SW3800_VC0      = 187,
	BATT_ID_SW3800_VC1      = 204,
	BATT_ID_SW3800_VC2      = 219,
};

#define BATT_NOT_PRESENT 200

struct battery_id_type {
	int battery_id;
	int battery_cell_type;
	char *battery_type_name;
};

#define BATT_ID_LIST_MAX 10
struct batt_id_info {
	int batt_id_num;
	int batt_id[BATT_ID_LIST_MAX];
	char *batt_type[BATT_ID_LIST_MAX];
};

extern struct batt_id_info battery_id_list;

bool lge_battery_check(void);
#if defined (CONFIG_MACH_MSM8917_B6_JCM_JP) || defined(CONFIG_MACH_MSM8917_B6_LGU_KR)
#define BATT_ID_DEFAULT_TYPE_NAME "LGE_BLT20_LGC_4800mAh"
#elif defined (CONFIG_MACH_MSM8940_TF8_TMO_US)
#define BATT_ID_DEFAULT_TYPE_NAME "LGE_BL44E1F_LGC_3200mAh"
#elif defined (CONFIG_MACH_MSM8917_CV3_KT_KR) || defined (CONFIG_MACH_MSM8917_CV3_LGU_KR) || defined (CONFIG_MACH_MSM8917_CV3_SKT_KR)
#define BATT_ID_DEFAULT_TYPE_NAME "LGE_BL46G1F_LGC_2800mAh"
#else
#define BATT_ID_DEFAULT_TYPE_NAME "LGE_BL45F1F_LGC_2500mAh"
#endif

#endif  /* __LGE_BATTERY_ID_H__ */

