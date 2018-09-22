/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#ifndef RECOVERY_H
#define RECOVERY_H

typedef enum {
	PANEL_SW_RESET = 0,
	PANEL_HW_RESET,
} panel_reset_type;


enum {
	RC_OK = 0,
	RC_POWER_OFF_STATE,
	RC_UNBLANK_TIMEOUT,
	RC_INVALID_PARAM,
};
int lge_mdss_report_panel_dead(int reset_type);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_WITH_QCT_ESD)
extern void init_esd_status(void);
extern void set_esd_status(int esd_detection);
extern int get_esd_status(void);
#endif

#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
enum esd_state_flag {
	ESD_OK = 0,
	ESD_NOK,
	ESD_POWEROFF_PENDING,
};
extern int get_esd_power_recovery(void);
extern void set_esd_power_recovery(int esd_detection);
#endif

#endif //End of RECOVERY_H