#ifndef LGE_MDSS_DSI_CV1
#define LGE_MDSS_DSI_CV1

int tianma_ft8006m_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int tianma_ft8006m_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
int tianma_ft8006m_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int tianma_ft8006m_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
void tianma_ft8006m_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);

int lgd_lg4894a_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int lgd_lg4894a_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int lgd_lg4894a_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
int lgd_lg4894a_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lgd_lg4894a_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
void lgd_lg4894a_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);

int tcl_ft8006m_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int tcl_ft8006m_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
int tcl_ft8006m_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int tcl_ft8006m_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
void tcl_ft8006m_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);

#endif
