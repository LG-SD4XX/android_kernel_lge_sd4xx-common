
#ifndef _LGE_MDSS_DSI_PANEL_H
#define _LGE_MDSS_DSI_PANEL_H

int lge_mdss_panel_parse_dt_extra(struct device_node *np,
                        struct mdss_dsi_ctrl_pdata *ctrl_pdata);

void lge_mdss_dsi_panel_extra_cmds_read(struct mdss_dsi_ctrl_pdata *ctrl, const char *name);
void lge_mdss_dsi_panel_extra_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, const char *name);

#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
void lge_mdss_panel_parse_dt_blmaps(struct device_node *np,
				   struct mdss_dsi_ctrl_pdata *ctrl_pdata);
#endif

struct mdss_dsi_ctrl_pdata *lge_mdss_dsi_get_ctrl_pdata(void);
void lge_mdss_dsi_store_ctrl_pdata(struct mdss_dsi_ctrl_pdata *pdata);
void lge_mdss_dsi_pr_status_buf(struct mdss_dsi_ctrl_pdata *ctrl);

void lge_mdss_panel_dic_reg_dump(struct mdss_dsi_ctrl_pdata *pdata);

int lge_mdss_panel_select_initial_cmd_set(struct mdss_dsi_ctrl_pdata *ctrl);
#endif
