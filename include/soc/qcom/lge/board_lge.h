#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

#if defined(CONFIG_MACH_MSM8917_LV3_MPCS_US) || defined(CONFIG_MACH_MSM8917_LV3_TMO_US)  || defined(CONFIG_MACH_MSM8917_LV3_USC_US)\
	|| defined(CONFIG_MACH_MSM8917_LV3_LGU_KR) || defined(CONFIG_MACH_MSM8917_LV3_SKT_KR) || defined(CONFIG_MACH_MSM8917_LV3_KT_KR)\
	|| defined(CONFIG_MACH_MSM8917_LV3_GLOBAL_COM) || defined(CONFIG_MACH_MSM8917_LV7_TRF_US) || defined(CONFIG_MACH_MSM8917_LV7_TRF_US_VZW)\
	|| defined(CONFIG_MACH_MSM8940_LV9_ATT_US) || defined(CONFIG_MACH_MSM8940_LV9_NAO_US) || defined(CONFIG_MACH_MSM8917_LV7_CCT_US_VZW) \
	|| defined(CONFIG_MACH_MSM8917_LV7_CRK_US) || defined(ONFIG_MACH_MSM8917_LV7_GLOBAL_CA) || defined(CONFIG_MACH_MSM8940_LV9_GLOBAL_COM) || defined(CONFIG_MACH_MSM8917_LV7_LAO_COM)
enum hw_rev_type {
	HW_REV_0 = 0,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif defined(CONFIG_MACH_MSM8917_CV3_LGU_KR) || defined(CONFIG_MACH_MSM8917_CV3_SKT_KR) || defined(CONFIG_MACH_MSM8917_CV3_KT_KR) || defined(CONFIG_MACH_MSM8917_CV3_LAO_COM) || defined(CONFIG_MACH_MSM8917_CV3_VZW) || defined(CONFIG_MACH_MSM8917_CV3_CRK_US) || defined(CONFIG_MACH_MSM8917_CV3_CCT_US)
enum hw_rev_type {
	HW_REV_0 = 0,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_G,
	HW_REV_H,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif defined(CONFIG_MACH_MSM8940_MH_GLOBAL_COM) || defined(CONFIG_MACH_MSM8940_MH_LGU_KR) || defined(CONFIG_MACH_MSM8940_MH_SKT_KR) || defined(CONFIG_MACH_MSM8940_MH_KT_KR) || defined(CONFIG_MACH_MSM8940_MH_TMO_US) || defined(CONFIG_MACH_MSM8940_MH_CRK_US) || defined(CONFIG_MACH_MSM8940_MH_MPCS_US) || defined(CONFIG_MACH_MSM8940_MH_TRF_US) || defined(CONFIG_MACH_MSM8940_MH_GLOBAL_CA) || defined(CONFIG_MACH_MSM8940_MH_LAO_COM) || defined(CONFIG_MACH_MSM8940_MH_GLOBAL_LDU)
enum hw_rev_type {
	HW_REV_0 = 0,
	HW_REV_02,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif defined(CONFIG_MACH_SDM450_CV7A_LAO_COM) || defined(CONFIG_MACH_SDM450_CV7A_KR) || defined(CONFIG_MACH_SDM450_CV7A_LAO_COM_NONAB) || defined(CONFIG_MACH_SDM450_CV7A_LGU_LDU)
enum hw_rev_type {
	HW_REV_0 = 0,
	HW_REV_01,
	HW_REV_02,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif defined(CONFIG_MACH_SDM450_CV5A_DCM_JP) || defined(CONFIG_MACH_SDM450_CV5A_KT_KR) || defined(CONFIG_MACH_SDM450_CV5A_LAO_COM) || defined(CONFIG_MACH_SDM450_CV5A_LGU_KR) \
	|| defined(CONFIG_MACH_SDM450_CV5A_SKT_KR) || defined(CONFIG_MACH_SDM450_CV5A_TMO_US_SPROUT) || defined(CONFIG_MACH_SDM450_CV5A_LGU_LDU) 
enum hw_rev_type {
	HW_REV_0 = 0,
	HW_REV_01,
	HW_REV_02,
	HW_REV_A,
	HW_REV_A_1,
	HW_REV_B,
	HW_REV_B_1,
	HW_REV_B_2,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif	defined(CONFIG_MACH_MSM8917_LV517_TMO_US) || defined(CONFIG_MACH_MSM8917_LV517_VZW) || defined(CONFIG_MACH_MSM8917_LV517_TRF_US)\
	||  defined(CONFIG_MACH_MSM8917_LV517N_ATT_US) || defined(CONFIG_MACH_MSM8917_LV517_CRK_US) || defined(CONFIG_MACH_MSM8917_LV517_CLR_PR) || defined(CONFIG_MACH_MSM8917_LV517_MPCS_US)
enum hw_rev_type {
	HW_REV_0 = 0,
	HW_REV_A_1,
	HW_REV_A_2,
	HW_REV_A_3,
	HW_REV_B,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif	 defined(CONFIG_MACH_MSM8917_SF317_TRF_US) || defined(CONFIG_MACH_MSM8917_SF317_TRF_US_VZW) || defined(CONFIG_MACH_MSM8940_SF3_SPR_US)\
	|| defined(CONFIG_MACH_MSM8917_SF317_CRK_US)
	enum hw_rev_type {
	HW_REV_A = 0,
	HW_REV_A_2,
	HW_REV_A_3,
	HW_REV_B,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#elif defined(CONFIG_MACH_MSM8940_SF3_MPCS_US) || defined(CONFIG_MACH_MSM8940_SF3_TMO_US) || defined(CONFIG_MACH_MSM8940_TF8_TMO_US) || defined(CONFIG_MACH_MSM8940_TF8_GLOBAL_CA) || defined(CONFIG_MACH_MSM8940_TF8_LGU_KR) || defined(CONFIG_MACH_MSM8940_SF3_GLOBAL_CA) || defined(CONFIG_MACH_MSM8917_CV1_LAO_COM) || defined(CONFIG_MACH_MSM8917_CV1_VZW) || defined(CONFIG_MACH_MSM8917_CV1_ATT_US) || defined(CONFIG_MACH_MSM8917_CV1_CRK_US)
enum hw_rev_type {
        HW_REV_0 = 0,
        HW_REV_A,
        HW_REV_A_2,
        HW_REV_B,
        HW_REV_C,
        HW_REV_D,
        HW_REV_1_0,
        HW_REV_1_1,
        HW_REV_MAX
};
#elif defined(CONFIG_MACH_MSM8917_B6_JCM_JP) || defined(CONFIG_MACH_MSM8917_B6_LGU_KR)
enum hw_rev_type {
	HW_REV_HDKA = 0,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#else
enum hw_rev_type {
	HW_REV_EVB1 = 0,
	HW_REV_EVB2,
	HW_REV_EVB3,
	HW_REV_0,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_G,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_1_2,
	HW_REV_MAX
};
#endif

extern char *rev_str[];

enum hw_rev_type lge_get_board_revno(void);
bool lge_get_vdd_always_on(void);

#ifdef CONFIG_LGE_DISPLAY_BL_DIMMING
extern int lge_get_bootreason_with_lcd_dimming(void);
#endif

#ifdef CONFIG_LGE_USB_G_LAF
enum lge_laf_mode_type {
	LGE_LAF_MODE_NORMAL = 0,
	LGE_LAF_MODE_LAF,
	LGE_LAF_MODE_MID,
};

enum lge_laf_mode_type lge_get_laf_mode(void);
enum lge_laf_mode_type lge_get_laf_mid(void);
#endif

#ifdef CONFIG_LGE_USB_FACTORY
enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_QEM_56K,
	LGE_BOOT_MODE_QEM_130K,
	LGE_BOOT_MODE_QEM_910K,
	LGE_BOOT_MODE_PIF_56K,
	LGE_BOOT_MODE_PIF_130K,
	LGE_BOOT_MODE_PIF_910K,
	LGE_BOOT_MODE_MINIOS    /* LGE_UPDATE for MINIOS2.0 */
};

enum lge_boot_mode_type lge_get_boot_mode(void);
int lge_get_android_dlcomplete(void);
int lge_get_factory_boot(void);
int get_lge_frst_status(void);
#endif

#ifdef CONFIG_LGE_PM_ONBINARY_ORANGE
int lge_get_board_orange(void);
#endif

bool lge_get_mfts_mode(void);
int lge_get_factory_boot(void);

extern int lge_get_fota_mode(void);
extern char* lge_get_boot_partition(void);

#define LGE_RECOVERY_BOOT	1
extern int lge_get_bootreason(void);

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);
int lge_pre_self_diagnosis_pass(char *dev_code);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
enum lge_panel_type {
	UNDEFINED,
	PH2_SHARP,
	PH2_JDI,
	PH2_LGD_DB7400,
	JAPAN_LGD_FT8707_1_0,
	JAPAN_LGD_FT8707_1_1,
	JAPAN_LGD_TD4300,
	LV3_TIANMA,
	LV3_LGD,
	SF3_LGD_TD4100,
	LV5_LGD,
	LV9_JDI_NT35596,
	TOVIS_HX8394C,
	LV7_TOVIS,
	LV7V_TOVIS,
	SF3F_TD4310,
	SF3F_SW49105,
	TF8_INX_NT51021,
	SF3_TOVIS,
	LV5_TOVIS,
	LGD_INCELL_SW49106,
	LGD_INCELL_SW49107_HD,
	LGD_INCELL_SW49107,
	TOVIS_INCELL_SW49107_HD,
	TOVIS_INCELL_SW49107,
	CV1_TIANMA_FT8006M,
	CV1_LGD_LG4894A,
	CV1_TCL_FT8006M,
	CV3_LGD,
	CV3_TOVIS,
};

enum lge_panel_type lge_get_panel_type(void);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_MH)
bool lge_get_lcm_vcom_mtp(void);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_CV3)
bool lge_get_lcd_init_cmd_division(void);
#endif

#endif

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
struct pre_selfd_platform_data {
	int (*set_values) (int r, int g, int b);
	int (*get_values) (int *r, int *g, int *b);
};
#endif

#ifdef CONFIG_LGE_ONE_BINARY_SKU

enum lge_sku_carrier_type lge_get_sku_carrier(void);

enum lge_sku_carrier_type {
	LAO = 0,
	TMUS,
	TRF_C,
	TRF_G,
	USC,
	VZW,
	TRF,
	TMUS71,
	NA_GSM,
	TRF71,
	ATT,
	CRK,
	NA_ALL,
	NA_CDMA,
	CAN
};

#ifdef CONFIG_LGE_PM_VZW_REQ
int lge_get_board_att(void);
#endif
#endif

#ifdef CONFIG_LGE_ONE_BINARY_HW_SKU
int lge_get_gpio_value(void);
#endif //CONFIG_LGE_ONE_BINARY_HW_SKU

extern int on_hidden_reset;

#if defined(CONFIG_LGE_EARJACK_DEBUGGER) || defined(CONFIG_LGE_USB_DEBUGGER)
/* config */
#define UART_CONSOLE_ENABLE_ON_EARJACK		BIT(0)
#define UART_CONSOLE_ENABLE_ON_EARJACK_DEBUGGER	BIT(1)
#define UART_CONSOLE_ENABLE_ON_DEFAULT		BIT(2)
/* current status
 * ENABLED | DISABLED : logical enable/disable
 * READY : It means whether device is ready or not.
 *         So even if in ENABLED state, console output will
 *         not be emitted on NOT-ready state.
 */
#define UART_CONSOLE_ENABLED		BIT(3)
#define UART_CONSOLE_DISABLED		!(BIT(3))
#define UART_CONSOLE_READY		BIT(4)
/* filter */
# define UART_CONSOLE_MASK_ENABLE_ON	(BIT(0) | BIT(1) | BIT(2))
# define UART_CONSOLE_MASK_CONFIG	UART_CONSOLE_MASK_ENABLE_ON
# define UART_CONSOLE_MASK_ENABLED	BIT(3)
# define UART_CONSOLE_MASK_READY	BIT(4)

/* util macro */
#define lge_uart_console_should_enable_on_earjack()	\
	(unsigned int)(lge_uart_console_get_config() &	\
			UART_CONSOLE_ENABLE_ON_EARJACK)

#define lge_uart_console_should_enable_on_earjack_debugger()	\
	(unsigned int)(lge_uart_console_get_config() &		\
			UART_CONSOLE_ENABLE_ON_EARJACK_DEBUGGER)

#define lge_uart_console_should_enable_on_default()	\
	(unsigned int)(lge_uart_console_get_config() &	\
			UART_CONSOLE_ENABLE_ON_DEFAULT)

#define lge_uart_console_on_earjack_in()	\
	do {					\
		msm_serial_set_uart_console(	\
			lge_uart_console_should_enable_on_earjack());	\
	} while (0)

#define lge_uart_console_on_earjack_out()	\
	do {					\
		msm_serial_set_uart_console(	\
				lge_uart_console_should_enable_on_default()); \
	} while (0)

#define lge_uart_console_on_earjack_debugger_in()	\
	do {						\
		msm_serial_set_uart_console(		\
			lge_uart_console_should_enable_on_earjack_debugger()); \
	} while (0)

#define lge_uart_console_on_earjack_debugger_out()	\
	do {						\
		msm_serial_set_uart_console(		\
				lge_uart_console_should_enable_on_default()); \
	} while (0)

/* config =  UART_CONSOLE_ENABLE_ON_XXX [| UART_CONSOLE_ENABLE_ON_XXX]* */
extern unsigned int lge_uart_console_get_config(void);
extern void lge_uart_console_set_config(unsigned int config);

/* logical uart console status modifier
 * used as a flag to tell "I want to enable/disable uart console"
 * @RETURN or @PARAM::enabled
 * UART_CONSOLE_ENABLED  (non-zero): enabled
 * !UART_CONSOLE_ENABLED (zero): disabled
 */
extern unsigned int lge_uart_console_get_enabled(void);
extern void lge_uart_console_set_enabled(int enabled);
/* internal uart console device status tracker
 *
 * @RETURN or @PARAM::ready
 * UART_CONSOLE_READY (non-zero): device is ready
 * !UART_CONSOLE_READY (zero): device is not ready
 */
extern unsigned int lge_uart_console_get_ready(void);
extern void lge_uart_console_set_ready(unsigned int ready);

/* real device enabler (or disabler)
 * control uart console device to enable/disable
 * NOTE @PARAM::enable should be selected by uart console enable/disable policy
 * which can be known by lge_uart_console_should_enable_on_xxx.
 * @PARAM::enable
 * zero : disabled
 * non-zero : enable
 */
extern int msm_serial_set_uart_console(int enable);
extern int msm_serial_force_off(void);
#endif
#endif
