#include <linux/delay.h>
#include "mdss_dsi.h"

#include "lge/mfts_mode.h"

#include <soc/qcom/lge/board_lge.h>

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
#define EXT_DSV_PRIVILEGED
#include <linux/mfd/external_dsv.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
#include "lge/lge_mdss_debug.h"
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY)
#include <linux/msm_lcd_recovery.h>
#endif

#include <linux/input/lge_touch_notify.h>
enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

/* Touch LPWG Status */
static unsigned int pre_panel_mode = LCD_MODE_STOP;
static unsigned int cur_panel_mode = LCD_MODE_STOP;

#if defined(CONFIG_LGE_DISPLAY_RECOVERY)
int esd_detected = 0;
static int panel_recovery_flag = 0;
#endif

static bool flag_panel_deep_sleep_ctrl = false;
static bool flag_panel_deep_sleep_status = false;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
extern int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

extern int lge_mdss_fb_get_shutdown_state(void);

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int mdss_dsi_panel_reset_lgd_incell_sw49106(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if (pinfo == NULL) {
		pr_err("%s: Invalid pinfo data\n", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_err("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_info("%s: + enable = %d (override: cv5a)\n", __func__, enable);

	if (enable) {
		if (!pinfo->cont_splash_enabled) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
			if (lge_mdss_dsi_panel_power_seq_all()) {
				ext_dsv_chip_enable(enable);
				ext_dsv_mode_change(POWER_ON_1);
			}
#endif

			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);

				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
			usleep_range(7000, 7000);
			pr_info("%s: LCD/Touch reset sequence done \n", __func__);
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (lge_mdss_dsi_panel_power_seq_all()) {
			lge_extra_gpio_set_value(ctrl_pdata, "touch_reset", enable);
			usleep_range(1000, 1000);

			gpio_set_value((ctrl_pdata->rst_gpio), enable);
			usleep_range(5000, 5000);
		}
	}

	pr_info("%s: -\n", __func__);

	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	int rc = 0;

	switch(lge_get_panel_type()) {
	case LGD_INCELL_SW49106:
		rc = mdss_dsi_panel_reset_lgd_incell_sw49106(pdata, enable);
		break;
	default:
		break;
	}

	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on_lgd_incell_sw49106(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid pdata\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid ctrl_pdata\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: + (override: cv5a)\n", __func__);

	if (lge_mdss_dsi_panel_power_seq_all()) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
	}

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret) {
		pr_err("%s: failed to enable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
		return ret;
	}

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (pdata->panel_info.cont_splash_enabled ||
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");

		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("%s: Panel reset failed. rc=%d\n",
					__func__, ret);
	}

	pr_info("%s: -\n", __func__);

	return ret;
}

int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;

	switch(lge_get_panel_type()) {
	case LGD_INCELL_SW49106:
		ret = mdss_dsi_panel_power_on_lgd_incell_sw49106(pdata);
		break;
	default:
		break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off_lgd_incell_sw49106(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid pdata\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid ctrl_pdata\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: + (override: cv5a)\n", __func__);

	if (!ctrl_pdata->lge_extra.lp11_off) {
		ret = mdss_dsi_panel_reset(pdata, 0);
	}

	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 0);
	if (ret)
		pr_err("%s: failed to disable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
	if (lge_mdss_dsi_panel_power_seq_all()) {
		ext_dsv_mode_change(POWER_OFF);
		ext_dsv_chip_enable(0);
	}
#endif

	if (lge_mdss_dsi_panel_power_seq_all()) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	}

	pr_info("%s: -\n", __func__);
end:
	return ret;
}

int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;

	switch(lge_get_panel_type()) {
	case LGD_INCELL_SW49106:
		ret = mdss_dsi_panel_power_off_lgd_incell_sw49106(pdata);
		break;
	default:
		break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
extern int mdss_dsi_set_clk_src(struct mdss_dsi_ctrl_pdata *ctrl);
extern int mdss_dsi_clk_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, void *clk_handle,
	enum mdss_dsi_clk_type clk_type, enum mdss_dsi_clk_state clk_state);

void mdss_dsi_ctrl_shutdown_lgd_incell_sw49106(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);
	int ret = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	ret += mdss_dsi_set_clk_src(ctrl_pdata);
	ret += mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
	if (ret) {
		pr_err("%s: could fail to set LP11\n", __func__);
	}
	usleep_range(5000, 5000);

	lge_extra_gpio_set_value(ctrl_pdata, "touch_reset", 0);
	usleep_range(1000, 1000);

	gpio_set_value((ctrl_pdata->rst_gpio), 0);
	usleep_range(3000, 3000);

	ret += mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
		MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);
	if (ret) {
		pr_err("%s: could fail to set LP00\n", __func__);
	}
	usleep_range(5000, 5000);

	ext_dsv_mode_change(POWER_OFF);
	ext_dsv_chip_enable(0);

	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	usleep_range(1000, 1000);

	pr_info("%s: panel shutdown done \n", __func__);

	return;
}

void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	switch(lge_get_panel_type()) {
	case LGD_INCELL_SW49106:
		mdss_dsi_ctrl_shutdown_lgd_incell_sw49106(pdev);
		break;
	default:
		break;
	}
}
#endif

int lge_mdss_dsi_pre_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	int panel_type;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	panel_type = lge_get_panel_type();

	switch (event) {
	case MDSS_EVENT_BLANK:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		lge_mdss_panel_dic_reg_dump(ctrl_pdata);
#endif
		break;
	default:
		break;
	}

	return rc;
}

int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	int panel_type;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	panel_type = lge_get_panel_type();

	switch (event) {
	case MDSS_EVENT_RESET:
		flag_panel_deep_sleep_ctrl = false;
		if (flag_panel_deep_sleep_status) {
			lge_set_panel_recovery_flag(true);
		}
		break;
	case MDSS_EVENT_UNBLANK:
		if (lge_mdss_dsi_panel_power_seq_all()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
			ext_dsv_mode_change(POWER_ON_2);
#endif
			lge_set_panel_recovery_flag(false);
		} else {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
			ext_dsv_mode_change(ENM_EXIT);
#endif
		}
		lge_mdss_dsi_panel_extra_cmds_send(NULL, "sleep-out");
		break;
	case MDSS_EVENT_POST_PANEL_ON:
		cur_panel_mode = LCD_MODE_U3;
		pr_info("%s: event=MDSS_EVENT_POST_PANEL_ON panel_mode=%d,%d\n",
			__func__, pre_panel_mode, cur_panel_mode);

		if (flag_panel_deep_sleep_status) {
			lge_set_panel_recovery_flag(false);
			flag_panel_deep_sleep_status = false;
		}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		lge_mdss_panel_dic_reg_dump(ctrl_pdata);

		lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_tx", DEBUG_DSI_CMD_TX);
		lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_rx", DEBUG_DSI_CMD_RX);
#endif
		break;
	case MDSS_EVENT_BLANK:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		if (!(lge_mdss_fb_get_shutdown_state() || lge_mdss_dsi_panel_power_seq_all()))
			ext_dsv_mode_change(ENM_ENTER);
#endif

		lge_mdss_dsi_panel_extra_cmds_send(NULL, "sleep-in");
		break;
	case MDSS_EVENT_PANEL_OFF:
		cur_panel_mode = LCD_MODE_U0;
		pr_info("%s: event=MDSS_EVENT_PANEL_OFF panel_mode=%d,%d\n",
			__func__, pre_panel_mode, cur_panel_mode);

			flag_panel_deep_sleep_ctrl = true;
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		lge_debug_event_trigger(pdata, "", INVALID); //NOTE : This is must-do-null-event-trigger for debug_event to escape from unblnak
#endif
		break;
	default:
		break;
	}

	if (pre_panel_mode != cur_panel_mode) {
		rc = touch_notifier_call_chain(LCD_EVENT_LCD_MODE, (void *)&cur_panel_mode);
		pre_panel_mode = cur_panel_mode;
	}

	return rc;
}


#if IS_ENABLED(CONFIG_LGE_DISPLAY_TOUCH_NOTIFIER_CALL_CHAIN)
int lge_get_lpwg_on_event(void)
{
	return MDSS_EVENT_MAX;
}
int lge_get_lpwg_off_event(void)
{
	return MDSS_EVENT_MAX;
}
#endif

#if defined(CONFIG_LGE_DISPLAY_RECOVERY)

int lge_get_panel_recovery_flag()
{
	pr_info("%s: flag=%d", __func__, panel_recovery_flag);
	return panel_recovery_flag;
}

void lge_set_panel_recovery_flag(int flag)
{
	if (lge_get_panel_type() == LGD_INCELL_SW49106) {
		pr_info("%s: flag=%d", __func__, flag);
		panel_recovery_flag = flag;
	}
}

int lge_mdss_report_touchintpin_keep_low(void)
{
	pr_info("%s : D-IC is in abnormal status", __func__);
	lge_mdss_report_panel_dead(PANEL_HW_RESET);

	return 0;
}
EXPORT_SYMBOL(lge_mdss_report_touchintpin_keep_low);
#endif

void lge_panel_enter_deep_sleep(void)
{
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	if (flag_panel_deep_sleep_ctrl) {
		pdata = lge_mdss_dsi_get_ctrl_pdata();
		if (pdata == NULL)
			return;

		usleep_range(6000, 6000);

		ext_dsv_mode_change(POWER_OFF);
		ext_dsv_chip_enable(0);

		lge_extra_gpio_set_value(pdata, "touch_reset", 0);
		usleep_range(1000, 1000);
		gpio_set_value((pdata->rst_gpio), 0);
		usleep_range(2000, 2000);

		lge_extra_gpio_set_value(pdata, "vddio", 0);
		usleep_range(1000, 1000);

		flag_panel_deep_sleep_status = true;
		pr_info("%s done \n", __func__);
	}
}

void lge_panel_exit_deep_sleep(void)
{
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	if (flag_panel_deep_sleep_ctrl) {
		pdata = lge_mdss_dsi_get_ctrl_pdata();
		if (pdata == NULL)
			return;

		mdss_dsi_clk_ctrl(pdata, pdata->dsi_clk_handle, MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
		mdss_dsi_sw_reset(pdata, true);

		lge_extra_gpio_set_value(pdata, "vddio", 1);
		usleep_range(2000, 2000);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		ext_dsv_chip_enable(1);
		ext_dsv_mode_change(POWER_ON_1);
#endif

		gpio_set_value((pdata->rst_gpio), 1);
		usleep_range(6000, 6000);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		ext_dsv_mode_change(POWER_ON_2);
#endif
		lge_extra_gpio_set_value(pdata, "touch_reset", 1);
		usleep_range(5000, 5000);

		lge_mdss_dsi_panel_extra_cmds_send(pdata, "lpwg-on");

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		ext_dsv_mode_change(ENM_ENTER);
#endif

		mdss_dsi_clk_ctrl(pdata, pdata->dsi_clk_handle, MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);

		flag_panel_deep_sleep_status = false;

		pr_info("%s done \n", __func__);
	}
}