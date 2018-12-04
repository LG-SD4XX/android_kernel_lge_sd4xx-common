
#ifndef _LGE_MDSS_DSI_PANEL_PH2N_TMO_US_H
#define _LGE_MDSS_DSI_PANEL_PH2N_TMO_US_H

enum { // DSV Type
	LGE_DSV_PMI8952 = 1,
	LGE_DSV_DW8768
};

int lge_get_dsv_type(void);

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
#endif