#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "lge/mfts_mode.h"
#include <linux/mfd/dw8768.h>
#include "lge_mdss_dsi_cv1.h"

#include <linux/input/lge_touch_notify.h>
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool active);

int tcl_ft8006m_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if (pdata->panel_info.cont_splash_enabled) {
	/*
	 * Vote for vreg due to unbalanced regulator disable
	 */
		ret = msm_dss_enable_vreg(
                        ctrl_pdata->panel_power_data.vreg_config,
                        ctrl_pdata->panel_power_data.num_vreg, 1);
		if (ret) {
                        pr_err("%s: failed to enable vregs for %s\n",
                                __func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
                        return ret;
                }
	} else if (lge_mdss_dsi_panel_power_seq_all()) {
		pr_info("%s: turn panel power on\n", __func__);

		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
		if (ret) {
			pr_err("%s: failed to enable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}

		if (lge_get_display_power_ctrl()) {
			/* 1v8 Active Low at MFTS JIG */
			lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 0);
			pr_info("mfts-power gpio set low\n");
		}

		LGE_MDELAY(5);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);
		usleep_range(50000, 50000);

		dw8768_register_set(0x05, 0x0F);
		LGE_MDELAY(1);
		if (dw8768_set_output_voltage(0x0F))
			pr_err("%s: setting DSV output voltage failed\n", __func__);
		LGE_MDELAY(5);
	} else{
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
	return ret;
}

int tcl_ft8006m_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto end;
	}

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
		pr_info("%s: turn panel power off\n", __func__);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);

		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);

		if (lge_get_display_power_ctrl()) {
			lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 1);
			pr_info("mfts-power gpio set high\n");
		}

		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
	} else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}
end:
	return ret;
}

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

int tcl_ft8006m_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
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
	if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
		mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
			pinfo->is_dba_panel) {
		pr_debug("%s:%d, right ctrl gpio configuration not needed\n",
			__func__, __LINE__);
		return rc;
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			__func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
			LGE_MDELAY(10);
			pr_info("%s: panel reset high\n", __func__);
			if (pdata->panel_info.rst_seq_len) {
				rc = gpio_direction_output(ctrl_pdata->rst_gpio,
						pdata->panel_info.rst_seq[0]);
				if (rc) {
					pr_err("%s: unable to set dir for rst gpio\n", __func__);
					goto exit;
				}
			}

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				if(i == pdata->panel_info.rst_seq_len/2) {
						touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
						LGE_MDELAY(10);
				}
				gpio_set_value((ctrl_pdata->rst_gpio),
				pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n", __func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (lge_mdss_dsi_panel_power_seq_all()) {
			dw8768_register_set(0x05, 0x0c);
			LGE_MDELAY(5);
			pr_info("%s: panel reset low\n", __func__);
	                gpio_set_value((ctrl_pdata->rst_gpio), 0);
		} else {
			pr_info("%s: skip panel reset low\n", __func__);
		}
                gpio_free(ctrl_pdata->rst_gpio);
        }

exit:
        return rc;
}


void tcl_ft8006m_mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	LGE_MDELAY(150);
	dw8768_register_set(0x05, 0x0c);
	LGE_MDELAY(5);

	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
	if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_info("%s:reset to low\n", __func__);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);
	LGE_MDELAY(5);
	lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);

	if (lge_get_display_power_ctrl()) {
		lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 1);
		pr_err("mfts-power gpio set\n");
	}

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 0);
	if (ret)
		pr_err("%s: failed to disable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
	pr_info("%s: turn panel shutdown\n", __func__);
}
