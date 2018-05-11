
#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/mfts_mode.h"
#include <linux/mfd/dw8768.h>
#include <soc/qcom/lge/board_lge.h>

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMFORT_MODE)
#include "lge/lge_comfort_view.h"
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
#include "lge/lge_mdss_debug.h"
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
#define EXT_DSV_PRIVILEGED
#include <linux/mfd/external_dsv.h>
#endif

#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
int esd_detected = 0;
static int panel_recovery_flag = 0;
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

bool set_touch_osc_flag = false;
static struct mutex set_touch_osc_mutex;
static bool set_touch_osc_mutex_init = false;

extern bool set_touch_osc_flag;
extern void lock_set_touch_osc(void);
extern void unlock_set_touch_osc(void);

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);


static void init_set_touch_osc_mutex(void)
{
	if (set_touch_osc_mutex_init)
		return;

	mutex_init(&set_touch_osc_mutex);
	set_touch_osc_mutex_init = true;
}

void lock_set_touch_osc(void)
{
	init_set_touch_osc_mutex();
	mutex_lock(&set_touch_osc_mutex);
}

void unlock_set_touch_osc(void)
{
	mutex_unlock(&set_touch_osc_mutex);
}

int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	int panel_type;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	panel_type = lge_get_panel_type();

	switch (event) {
#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
	case MDSS_EVENT_UNBLANK:
		if (lge_mdss_dsi_panel_power_seq_all()) {
			lge_set_panel_recovery_flag(0);
		}
#endif
	case MDSS_EVENT_RESET:
			set_touch_osc_flag = false;
			lock_set_touch_osc();
			unlock_set_touch_osc();
		break;
	case MDSS_EVENT_POST_PANEL_ON:
			cur_panel_mode = LCD_MODE_U3;
			pr_info("%s: event=MDSS_EVENT_POST_PANEL_ON panel_mode=%d,%d\n",
				__func__, pre_panel_mode, cur_panel_mode);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
			lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_tx", DEBUG_DSI_CMD_TX);
			lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_rx", DEBUG_DSI_CMD_RX);
#endif
		break;
	case MDSS_EVENT_PANEL_OFF:
			set_touch_osc_flag = true;
			cur_panel_mode = LCD_MODE_U0;
			pr_info("%s: event=MDSS_EVENT_PANEL_OFF panel_mode=%d,%d\n",
				__func__, pre_panel_mode, cur_panel_mode);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
			lge_debug_event_trigger(pdata, "", INVALID); //NOTE : This is must-do-null-event-trigger for debug_event to escape from unblnak
#endif
		break;
	default:
		pr_info("%s: nothing to do about this event=%d\n", __func__, event);
	}

		if (pre_panel_mode != cur_panel_mode) {
			rc = touch_notifier_call_chain(LCD_EVENT_LCD_MODE, (void *)&cur_panel_mode);
			pre_panel_mode = cur_panel_mode;
		}
	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON) || IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: + (override: cv3)\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (lge_mdss_dsi_panel_power_seq_all()) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
		LGE_MDELAY(3);
		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 1);
		LGE_MDELAY(1);
	} else {
		pr_info("%s: skip panel power control\n", __func__);
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
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: + (override: cv3)\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	if (lge_mdss_dsi_panel_power_seq_all()) {
		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);
		LGE_MDELAY(1);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
		ext_dsv_mode_change("fast_discharge_on_mode");
		ext_dsv_chip_enable(0);
#endif
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
		LGE_MDELAY(1);
	} else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}

	pr_info("%s: -\n", __func__);
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
	        pr_err("request reset gpio failed, rc=%d\n",
	                rc);
	}

	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
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

	pr_info("%s: + enable = %d (override: cv3)\n", __func__, enable);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);

			if (pdata->panel_info.rst_seq_len) {
				rc = gpio_direction_output(ctrl_pdata->rst_gpio,
					pdata->panel_info.rst_seq[0]);
				if (rc) {
					pr_err("%s: unable to set dir for rst gpio\n",
						__func__);
					goto exit;
				}
			}

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio), pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}

			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);

			if (lge_mdss_dsi_panel_power_seq_all()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
				ext_dsv_chip_enable(enable);

				ext_dsv_mode_change("normal_mode");
				if (lge_get_panel_type() == CV3_LGD) {
					ext_dsv_mode_change("output_voltage_lgd_mode");
				} else {
					ext_dsv_mode_change("output_voltage_tovis_mode");
				}
#endif
			} else {
				LGE_MDELAY(8);
			}
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (lge_mdss_dsi_panel_power_seq_all()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
			ext_dsv_mode_change("negative_only_off_mode");
			ext_dsv_mode_change("power_off_hiz_mode");
#endif
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
			LGE_MDELAY(3);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			LGE_MDELAY(3);
		} else {
			pr_info("%s: skip to set panel reset low\n", __func__);
		}
		gpio_free(ctrl_pdata->rst_gpio);
	}

	pr_info("%s: -\n", __func__);
exit:
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	pr_info("%s: + (override: cv3)\n", __func__);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
	ext_dsv_mode_change("negative_only_off_mode");
	ext_dsv_mode_change("power_off_hiz_mode");
#endif

	lge_extra_gpio_set_value(ctrl_pdata, "touch-rst", 0);
	LGE_MDELAY(3);
	gpio_set_value((ctrl_pdata->rst_gpio), 0);
	LGE_MDELAY(3);

	lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);
	LGE_MDELAY(1);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
	ext_dsv_mode_change("fast_discharge_on_mode");
	ext_dsv_chip_enable(0);
#endif

	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	LGE_MDELAY(1);

	pr_info("%s: -\n", __func__);
	return;
}
#endif

int set_touch_osc(int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	char name[32];
	int i, index = -1;

	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		pr_err("%s : Don't control touch osc\n",__func__);
		return 0;
	}

	if (set_touch_osc_flag) {
		ctrl = lge_mdss_dsi_get_ctrl_pdata();
		if (!ctrl) {
			pr_err("%s: ctrl is null\n", __func__);
			return -EINVAL;
		}

		if (enable) {
			strcpy(name, "touch-osc-on");
		} else {
			strcpy(name, "touch-osc-off");
		}

		for (i = 0; i < ctrl->lge_extra.num_extra_cmds; ++i) {
			if (!strcmp(ctrl->lge_extra.extra_cmds_array[i].name, name)) {
				index = i;
				break;
			}
		}

		if (index == -1) {
			pr_err("%s: no touch ocs on/off cmd\n", __func__);
			return -EINVAL;
		}

		lock_set_touch_osc();
		if (ctrl->lge_extra.extra_cmds_array[index].cmds.cmd_cnt) {
			mdss_dsi_clk_ctrl(ctrl, ctrl->dsi_clk_handle,
					MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
			mdss_dsi_sw_reset(ctrl, true);
			lge_mdss_dsi_panel_extra_cmds_send(ctrl, name);
			pr_info("%s:enable=%d\n", __func__, enable);
			mdss_dsi_clk_ctrl(ctrl, ctrl->dsi_clk_handle,
				       MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);
		}
		unlock_set_touch_osc();
	}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
	if (enable) {
		ext_dsv_mode_change("normal_mode");
		if (lge_get_panel_type() == CV3_LGD) {
			ext_dsv_mode_change("output_voltage_lgd_mode");
		} else {
			ext_dsv_mode_change("output_voltage_tovis_mode");
		}
		pr_info("%s: DSV always on mode\n", __func__);
	} else {
		ext_dsv_mode_change("deep_sleep_mode");
		pr_info("%s: DSV ENM mode\n", __func__);
	}
#endif

	return 0;
}
EXPORT_SYMBOL(set_touch_osc);

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

	for (i = 0 ; i < COMFORT_VIEW_STEP_NUMS ; i++) {
		mdss_dsi_parse_dcs_cmds(np, &comfort_view_cmds[i],
			comfort_view_step_cmds[i], "lge,comfort-view-cmds-state");
	}
	return 0;
}

static bool change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int new_mode)
{
	if(comfort_view_cmds[new_mode].cmd_cnt) {
		pr_info("%s: sending comfort mode commands [%d]\n", __func__, new_mode);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
		mdss_dsi_panel_cmds_send(ctrl, &comfort_view_cmds[new_mode], CMD_REQ_COMMIT);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	}
	return true;
}

bool lge_change_comfort_view(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode)
{
	if (old_mode == new_mode) {
		pr_info("%s: same mode [%d]\n", __func__, new_mode);
		return true;
	}

	return change_comfort_view(ctrl, new_mode);
}

int lge_mdss_dsi_panel_send_post_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, int cur_mode)
{
	if (cur_mode != 0)
		change_comfort_view(ctrl, cur_mode);
	return 0;
}
#endif


int lge_mdss_dsi_panel_power_seq_all() {
	int ret = 0;
#if IS_ENABLED(CONFIG_LGE_DISPLAY_MFTS)
	if (lge_get_display_power_ctrl())
		ret = 1;
#endif
#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
	if (lge_get_panel_recovery_flag())
		ret = 1;
#endif
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO)
		ret = 1;

	return ret;
}



#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)

int lge_get_panel_recovery_flag()
{
	pr_info("%s: flag=%d", __func__, panel_recovery_flag);
	return panel_recovery_flag;
}

void lge_set_panel_recovery_flag(int flag)
{
	pr_info("%s: flag=%d", __func__, flag);
	panel_recovery_flag = flag;
}

int lge_mdss_report_touchintpin_keep_low(void)
{
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	pdata = lge_mdss_dsi_get_ctrl_pdata();

	if (pdata != NULL && pdata->check_status != NULL) {
		//Just for information, check registers of display block. Add cmds in dtsi if you want to take a look more
		pdata->check_status(pdata);
		lge_mdss_dsi_pr_status_buf(pdata);
	}

	pr_info("%s : D-IC is in abnormal status", __func__);
	lge_mdss_report_panel_dead();

	return 0;
}
EXPORT_SYMBOL(lge_mdss_report_touchintpin_keep_low);
#endif



int lge_mdss_panel_select_initial_cmd_set(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i, index = -1;

	if (ctrl == NULL) {
		pr_err("%s : invalid ctrl data \n", __func__);
		return 0;
	}

	if (lge_get_panel_type() == CV3_TOVIS) {
		/* For TOVIS panel, do nothing. Just use recent version of initial code */
	} else {
		if (lge_get_lcd_init_cmd_division()) {
			/* For new panel, do nothing. Just use recent version of initial code */
		} else {
			for (i = 0; i < ctrl->lge_extra.num_extra_cmds; ++i) {
				if (!strcmp(ctrl->lge_extra.extra_cmds_array[i].name, "on-command-v09")) {
					index = i;
					break;
				}
			}

			if (ctrl->lge_extra.extra_cmds_array[index].cmds.cmd_cnt) {
				ctrl->on_cmds = ctrl->lge_extra.extra_cmds_array[index].cmds;
				pr_info("%s : on command v0.9 is selected \n", __func__);
			}
		}
	}

	return 0;
}
