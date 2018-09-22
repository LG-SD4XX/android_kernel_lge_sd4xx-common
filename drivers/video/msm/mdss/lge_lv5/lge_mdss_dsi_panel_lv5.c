#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "lge_mdss_dsi_panel_lv5.h"
#include <linux/mfd/dw8768.h>

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
#include <soc/qcom/lge/board_lge.h>
#endif

extern bool set_touch_osc_flag;
extern void lock_set_touch_osc(void);
extern void unlock_set_touch_osc(void);

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
		if (!lgd_lg4894_pdata_base) {
			pr_err("no panel connected!\n");
			return -EINVAL;
		}

		ctrl = container_of(lgd_lg4894_pdata_base,
				struct mdss_dsi_ctrl_pdata, panel_data);
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

	if (enable) {
		dw8768_register_set(0x05, 0x0F);
		LGE_MDELAY(1);
		dw8768_register_set(0x03, 0x83);
		dw8768_register_set(0x00, 0x0D);
		dw8768_register_set(0x01, 0x14);
		pr_info("%s: DSV always on mode\n", __func__);
	} else {
		dw8768_register_set(0x03, 0x80);
		dw8768_register_set(0x05, 0x07);
		pr_info("%s: DSV ENM mode\n", __func__);
	}

	return 0;
}
EXPORT_SYMBOL(set_touch_osc);

