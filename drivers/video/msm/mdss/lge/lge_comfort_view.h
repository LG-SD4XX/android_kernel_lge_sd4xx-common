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

#ifndef COMFORT_VIEW_H
#define COMFORT_VIEW_H

#define COMFORT_VIEW_STEP_NUMS 11

/* Implementions of below functions depend on DDIC */
int lge_mdss_dsi_parse_comfort_view_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
bool lge_change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode);
int lge_mdss_dsi_panel_send_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *default_on_cmds, int cur_mode);
int lge_mdss_dsi_panel_send_post_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, int cur_mode);
/* END */

int lge_get_comfort_view(void);

#endif
