#define pr_fmt(fmt)	"[DISPLAY]%s: " fmt, __func__

#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include <soc/qcom/lge/board_lge.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMFORT_MODE)
#include "lge/lge_comfort_view.h"
#endif


static int gpio_power_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{

	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (pinfo == NULL) {
		pr_err("invalid pinfo\n");
		return -EINVAL;
	}

	if (enable) {
		rc = lge_extra_gpio_request(ctrl_pdata, "iovcc");
		if (rc) {
			pr_err("gpio request failed, rc = %d\n", rc);
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			pr_info("turn panel power on\n");
			lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 1);
		}
	} else {
		pr_info("turn panel power off\n");
		lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);
		lge_extra_gpio_free(ctrl_pdata, "iovcc");
	}

	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pr_info("++\n");

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	if (ctrl_pdata == NULL) {
		pr_err("invalid ctrl_pdata\n");
		return -EINVAL;
	}

	ret = gpio_power_ctrl(ctrl_pdata, 1);

	if (ret)
		pr_err("panel power on failed, ret = %d\n", ret);

	usleep_range(100, 100);

	ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret)
		pr_err("failed to enable vregs for %s\n",
				__mdss_dsi_pm_name(DSI_PANEL_PM));

	pr_info("--\n");
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pr_info("++\n");

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	if (ctrl_pdata == NULL) {
		pr_err("invalid ctrl_pdata\n");
		return -EINVAL;
	}

	ret = gpio_power_ctrl(ctrl_pdata, 0);

	if (ret)
		pr_err("panel power off failed, rc=%d\n", ret);

	usleep_range(5000, 5000);

	ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 0);

	if (ret)
		pr_err("failed to disable vregs for %s\n",
				__mdss_dsi_pm_name(DSI_PANEL_PM));

	pr_info("--\n");
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
/*
 * mdss_dsi_request_gpios() should be defined in each panel file
 */
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("no ctrl_pdata\n");
		return;
	}

	pr_info("turn panel shutdown\n");
	lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);

	return;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMFORT_MODE)
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);

static struct dsi_panel_cmds comfort_view_cmds[COMFORT_VIEW_STEP_NUMS];

static char *comfort_view_step_cmds[] = {
	"lge,comfort-view-cmds-off",
	"lge,comfort-view-cmds-step1",
	"lge,comfort-view-cmds-step2",
	"lge,comfort-view-cmds-step3",
	"lge,comfort-view-cmds-step4",
	"lge,comfort-view-cmds-step5",
	"lge,comfort-view-cmds-step6",
	"lge,comfort-view-cmds-step7",
	"lge,comfort-view-cmds-step8",
	"lge,comfort-view-cmds-step9",
	"lge,comfort-view-cmds-step10",
};

int lge_mdss_dsi_parse_comfort_view_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i;
	int err = 0;

	for (i = 0 ; i < COMFORT_VIEW_STEP_NUMS ; i++) {
		err = mdss_dsi_parse_dcs_cmds(np, &comfort_view_cmds[i],
			comfort_view_step_cmds[i], "lge,comfort-view-cmds-state");
		if (err)
			pr_err("parse error %d: %s\n", err, comfort_view_step_cmds[i]);
	}
	return 0;
}

static bool change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int new_mode)
{
	pr_info("++\n");
	if(comfort_view_cmds[new_mode].cmd_cnt) {
		pr_info("sending comfort view commands [%d]\n", new_mode);
		mdss_dsi_panel_cmds_send(ctrl, &comfort_view_cmds[new_mode], CMD_REQ_COMMIT);
	}
	pr_info("--\n");
	return true;
}

bool lge_change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode)
{
	if (old_mode == new_mode) {
		pr_info("same mode [%d]\n", new_mode);
		return true;
	}

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	pr_info("change to mipi mode\n");
	lge_mdss_dsi_panel_extra_cmds_send(ctrl, "mipi-mode");

	change_comfort_view(ctrl, new_mode);

	pr_info("change to i2c mode\n");
	lge_mdss_dsi_panel_extra_cmds_send(ctrl, "i2c-mode");

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	return true;
}

int lge_mdss_dsi_panel_send_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *default_on_cmds, int cur_mode)
{

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	pr_info("change to mipi mode\n");
	lge_mdss_dsi_panel_extra_cmds_send(ctrl, "mipi-mode");

	if (default_on_cmds->cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, default_on_cmds, CMD_REQ_COMMIT);
	else
		pr_err("on command does not exist\n");

	/* comfort view is changed by gamma table and on-command doesn't have gamma table,
	 * so below func must be called after on-command */
	change_comfort_view(ctrl, cur_mode);

	pr_info("change to i2c mode\n");
	lge_mdss_dsi_panel_extra_cmds_send(ctrl, "i2c-mode");

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	return 0;
}
#endif
