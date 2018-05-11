#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>         /* wake_lock, unlock */
#include <linux/version.h>          /* check linux version */

#include <linux/err.h>
#include <linux/of_gpio.h>

#include <linux/clk.h>

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8180.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "bbm.h"
#include "fc8180_drv_api.h"
#include "fc8180_isr.h"
#include "fci_ringbuffer.h"

#define FEATURE_DTV_USE_PINCTRL
#undef _NOT_USE_WAKE_LOCK_

#ifdef FEATURE_DTV_USE_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif

#include <linux/regulator/consumer.h>

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/rtpm_prio.h>
#include <linux/irqchip/mt-eic.h>
#include <mt-plat/mt_gpio.h>

#if defined(CONFIG_ARCH_MT6755)
#include <mt-plat/mt6755/include/mach/gpio_const.h>
#include"../../../../spi/mediatek/mt6755/mt_spi.h"
#include "mt_clkbuf_ctl.h"
#else
#include <mt-plat/mt6735/include/mach/gpio_const.h>
#include"../../../../spi/mediatek/mt6735/mt_spi.h"
//#include"../../include/mt-plat/mt6735/include/mach/gpio_const.h"
#endif

#define GPIO_DTV_EN_PIN         (GPIO129 | 0x80000000)
#define GPIO_DTV_SPI_CS_PIN         (GPIO98 | 0x80000000)
#define GPIO_DTV_SPI_SCK_PIN         (GPIO99 | 0x80000000)
#define GPIO_DTV_SPI_MISO_PIN         (GPIO97 | 0x80000000)
#define GPIO_DTV_SPI_MOSI_PIN         (GPIO96 | 0x80000000)
#define GPIO_DTV_INT_PIN         (GPIO61 | 0x80000000)

#define LGE_FC8180_DRV_VER "1.00.00"

static struct mt_chip_conf mtk_spi_config = {
    .setuptime = 3,
    .holdtime = 3,
    .high_time = 3,
    .low_time = 3,
    .cs_idletime = 2,
    .ulthgh_thrsh = 0,
    .cpol = 0,
    .cpha = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = DMA_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 2,
};
#endif

struct ISDBT_INIT_INFO_T *hInit;

#define FC8180_NAME        "broadcast1"

#define RING_BUFFER_SIZE	(188 * 320)
static struct task_struct *isdbt_kthread;
struct fci_ringbuffer		RingBuffer;
static wait_queue_head_t isdbt_isr_wait;
u8 irq_error_cnt;
static u8 isdbt_isr_sig=0;
static u8 isdbt_isr_start=0;
u32 totalTS=0;
u32 totalErrTS=0;

u8 module_init_flag  = 0;

enum ISDBT_MODE{
	ISDBT_POWERON       = 0,
	ISDBT_POWEROFF	    = 1,
	ISDBT_DATAREAD		= 2
};
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;

u8 static_ringbuffer[RING_BUFFER_SIZE];
static DEFINE_MUTEX(ringbuffer_lock);
unsigned int isr_flag;

struct broadcast_fc8180_ctrl_data {
    int                                   pwr_state;
    struct wake_lock             wake_lock;
    struct spi_device*           spi_dev;
    struct i2c_client*             pclient;
    struct platform_device*   pdev;
    uint32                             isdbt_en;
    //uint32                          isdbt_reset;
    /* Interrupt pin is not used in TSIF mode */
    uint32                             isdbt_irq;
    struct device_node*       dtv_irq_node;

    uint32                             isdbt_use_ant_sw;
    uint32                             isdbt_ant_active_mode;
    uint32                             isdbt_ant;

    uint32                              isdbt_use_xtal;
    struct clk*                      xo_clk;
    uint32                              isdbt_xtal_freq;

    uint32                              ctrl_isdbt_ldo;
    struct regulator*             vdd_io;
    uint32                              ctrl_lna_ldo;
    struct regulator*             ant_io;

    uint32                             isdbt_use_lna_ctrl;
    uint32                             isdbt_lna_ctrl; //gain control
    uint32                             isdbt_use_lna_en;
    uint32                             isdbt_lna_en; //enable
};

static struct broadcast_fc8180_ctrl_data  IsdbCtrlInfo;

struct i2c_client* FCI_GET_I2C_DRIVER(void)
{
	return IsdbCtrlInfo.pclient;
}

struct spi_device* FCI_GET_SPI_DRIVER(void)
{
	return IsdbCtrlInfo.spi_dev;
}

static Device_drv device_fc8180 = {
    &broadcast_fc8180_drv_if_power_on,
    &broadcast_fc8180_drv_if_power_off,
    &broadcast_fc8180_drv_if_open,
    &broadcast_fc8180_drv_if_close,
    &broadcast_fc8180_drv_if_set_channel,
    &broadcast_fc8180_drv_if_resync,
    &broadcast_fc8180_drv_if_detect_sync,
    &broadcast_fc8180_drv_if_get_sig_info,
    &broadcast_fc8180_drv_if_get_ch_info,
    &broadcast_fc8180_drv_if_get_dmb_data,
    &broadcast_fc8180_drv_if_reset_ch,
    &broadcast_fc8180_drv_if_user_stop,
    &broadcast_fc8180_drv_if_select_antenna,
    &broadcast_fc8180_drv_if_isr,
    &broadcast_fc8180_drv_if_read_control,
    &broadcast_fc8180_drv_if_get_mode,
};
// TODO: del for qct
#ifdef FEATURE_DTV_USE_PINCTRL
static int isdbt_pinctrl_init(void)
{
    struct pinctrl *isdbt_pinctrl;
    isdbt_pinctrl = devm_pinctrl_get(&(IsdbCtrlInfo.pdev->dev));

    if(IS_ERR_OR_NULL(isdbt_pinctrl)) {
    	pr_err("%s: Getting pinctrl handle failed\n", __func__);
    	return -EINVAL;
    }
    return 0;
}
#endif

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
void tunerbb_drv_hw_setting(void)
{
    print_log(NULL, "[1seg][MTK] GPIO_ISDBT_PWR_EN Port request!!!\n");
    mt_set_gpio_mode(GPIO_DTV_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_pull_enable(GPIO_DTV_EN_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_EN_PIN, GPIO_PULL_DOWN);
    ms_wait(50);
    mt_set_gpio_dir(GPIO_DTV_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DTV_EN_PIN, GPIO_OUT_ZERO);

    print_log(NULL, "[1seg][MTK] ISDBT_IRQ_INT Port request!!!\n");
    mt_set_gpio_mode(GPIO_DTV_INT_PIN, GPIO_MODE_00);
    mt_set_gpio_pull_enable(GPIO_DTV_INT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_INT_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_DTV_INT_PIN, GPIO_DIR_IN);
    ms_wait(50);

    //SPI MISO
    mt_set_gpio_pull_enable(GPIO_DTV_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_MISO_PIN, GPIO_PULL_DOWN);

    //SPI MOSI
    mt_set_gpio_pull_enable(GPIO_DTV_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_MOSI_PIN, GPIO_PULL_DOWN);

    //SPI CLK
    mt_set_gpio_pull_enable(GPIO_DTV_SPI_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_SCK_PIN, GPIO_PULL_DOWN);

    //SPI CS
    mt_set_gpio_pull_enable(GPIO_DTV_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_CS_PIN, GPIO_PULL_DOWN);

    return;
}

void tunerbb_drv_hw_init(void)
{
    print_log(NULL, "[1seg][MTK] isdbt_hw_init\n");

    mt_set_gpio_out(GPIO_DTV_EN_PIN, GPIO_OUT_ONE);
    ms_wait(30);

    mt_set_gpio_pull_enable(GPIO_DTV_INT_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_MISO_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_MOSI_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_SCK_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_CS_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_mode(GPIO_DTV_SPI_CS_PIN, GPIO_MODE_01);
    mt_set_gpio_dir(GPIO_DTV_SPI_CS_PIN, GPIO_DIR_IN);

    return;
}

void tunerbb_drv_hw_deinit(void)
{
    print_log(0, "[1seg][MTK] isdbt_hw_deinit\n");

    fci_irq_disable();

    mt_set_gpio_pull_enable(GPIO_DTV_EN_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_EN_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_DTV_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DTV_EN_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_pull_enable(GPIO_DTV_INT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_INT_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_MISO_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_MOSI_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(GPIO_DTV_SPI_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_SCK_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_mode(GPIO_DTV_SPI_CS_PIN, GPIO_MODE_00);
    mt_set_gpio_pull_enable(GPIO_DTV_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DTV_SPI_CS_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_DTV_SPI_CS_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DTV_SPI_CS_PIN, GPIO_OUT_ZERO);

    return;
}
#endif

static int broadcast_isdbt_set_regulator(int onoff)
{
    int rc = -1;

    if(!IsdbCtrlInfo.vdd_io && IsdbCtrlInfo.ctrl_isdbt_ldo)
    {
        IsdbCtrlInfo.vdd_io = regulator_get(&(IsdbCtrlInfo.pdev->dev), "isdbt_vdd_io");
        if(IS_ERR(IsdbCtrlInfo.vdd_io)){
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not get regulator %s \n", "isdbt_vdd_io-supply");

            return rc;
        }
        else {
            printk("[dtv] get regulator %s \n", "isdbt_ldo-supply");
        }
    }

    if(!IsdbCtrlInfo.ant_io && IsdbCtrlInfo.ctrl_lna_ldo)
    {
        IsdbCtrlInfo.ant_io = regulator_get(&(IsdbCtrlInfo.pdev->dev), "isdbt_ant_io");
        if(IS_ERR(IsdbCtrlInfo.ant_io)){
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not get regulator %s \n", "isdbt_ant_io-supply");

            return rc;
        }
        else {
            printk("[dtv] get regulator %s \n", "isdbt_ant_io-supply");
        }
    }

    if(onoff)
    {
        if(IsdbCtrlInfo.ctrl_isdbt_ldo)
        {
            if(regulator_count_voltages(IsdbCtrlInfo.vdd_io) > 0)
            {
                rc = regulator_set_voltage(IsdbCtrlInfo.vdd_io, 1800000UL, 1800000UL);
                if(rc)
                {
                    dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not set regulator vdd_io, ret=%d \n", rc);
                }
            }

            rc = regulator_set_optimum_mode(IsdbCtrlInfo.vdd_io, 6000);
            if(rc < 0)
            {
                dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Unable to set current on vdd_io, ret=%d \n", rc);
            }

            rc = regulator_enable(IsdbCtrlInfo.vdd_io);
            if(rc)
            {
                dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not enable regulator vdd_io, ret=%d\n", rc);

                regulator_put(IsdbCtrlInfo.vdd_io);
    			IsdbCtrlInfo.vdd_io = NULL;
            }
            else		
            {
                dev_dbg(&(IsdbCtrlInfo.pdev->dev), "[dtv] Enabled regulator vdd_io\n");
            }
        }

        if(IsdbCtrlInfo.ctrl_lna_ldo)
        {
            if(regulator_count_voltages(IsdbCtrlInfo.ant_io) > 0)
            {
                rc = regulator_set_voltage(IsdbCtrlInfo.ant_io, 2850000UL, 2850000UL);
                if(rc)
                {
                    dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not set regulator ant_io, ret=%d \n", rc);
                }
            }

            rc = regulator_set_optimum_mode(IsdbCtrlInfo.ant_io, 6000);
            if(rc < 0)
            {
                dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Unable to set current on ant_io, ret=%d \n", rc);
            }

            rc = regulator_enable(IsdbCtrlInfo.ant_io);
            if(rc)
            {
                dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not enable regulator ant_io, ret=%d\n", rc);

                regulator_put(IsdbCtrlInfo.ant_io);
    			IsdbCtrlInfo.ant_io = NULL;
            }
            else		
            {
                dev_dbg(&(IsdbCtrlInfo.pdev->dev), "[dtv] Enabled regulator ant_io\n");
            }
        }
    }
    else
    {
        if(IsdbCtrlInfo.vdd_io && IsdbCtrlInfo.ctrl_isdbt_ldo)
        {
            if(regulator_is_enabled(IsdbCtrlInfo.vdd_io))
            {
                rc = regulator_disable(IsdbCtrlInfo.vdd_io);
                if(rc)
                {
                    dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not disable regulator vdd_io, ret=%d \n", rc);
                }
                else
                {
                    dev_dbg(&(IsdbCtrlInfo.pdev->dev), "[dtv] Disabled regulator vdd_io\n");
                }
            }
            regulator_put(IsdbCtrlInfo.vdd_io);
            IsdbCtrlInfo.vdd_io = NULL;

            mdelay(5);
        }

        if(IsdbCtrlInfo.ant_io && IsdbCtrlInfo.ctrl_lna_ldo)
        {
            if(regulator_is_enabled(IsdbCtrlInfo.ant_io))
            {
                rc = regulator_disable(IsdbCtrlInfo.ant_io);

                if(rc)
                {
                    dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] Could not disable regulator ant_io, ret=%d \n", rc);
                }
                else
                {
                    dev_dbg(&(IsdbCtrlInfo.pdev->dev), "[dtv] Disabled regulator ant_io\n");
                }
            }
            regulator_put(IsdbCtrlInfo.ant_io);
            IsdbCtrlInfo.ant_io = NULL;

            mdelay(5);
        }
    }

    printk("[dtv] Set PMIC voltage control(mode:%d) = %d\n", onoff, rc);

    return rc;
}

int fc8180_power_on(void)
{
    int i = 0;
    int rc = OK;

    if(IsdbCtrlInfo.pwr_state != 1)
    {
#ifndef _NOT_USE_WAKE_LOCK_
        wake_lock(&IsdbCtrlInfo.wake_lock);
#endif

        while (driver_mode == ISDBT_DATAREAD) {
            ms_wait(100);
            print_log(NULL, "[FC8180] ISDBT_DATARREAD mode i=(%d)\n", i);
            if(i++ > 5)
                break;
        }

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
        tunerbb_drv_hw_init();
#else
        if(IsdbCtrlInfo.isdbt_use_ant_sw) {
            printk("[dtv] fc8180_power_on() TDMB_FM_SW(%d : DMB mode)\n", IsdbCtrlInfo.isdbt_ant);

            gpio_set_value(IsdbCtrlInfo.isdbt_ant, IsdbCtrlInfo.isdbt_ant_active_mode);
        }

        //gpio_set_value(IsdbCtrlInfo.isdbt_reset, 0);
        gpio_set_value(IsdbCtrlInfo.isdbt_en, 1);
        //mdelay(3);
        //gpio_set_value(IsdbCtrlInfo.isdbt_reset, 1);
        //mdelay(2);

        if(IsdbCtrlInfo.ctrl_isdbt_ldo || IsdbCtrlInfo.ctrl_lna_ldo) {
            broadcast_isdbt_set_regulator(1);
        }
#endif

        msleep(30);
    	
        if(!IsdbCtrlInfo.isdbt_use_xtal) {
            if(IsdbCtrlInfo.xo_clk != NULL) {
                printk("[dtv] fc8180_power_on() clk enable\n");

                rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
                if(rc) {
                    gpio_set_value(IsdbCtrlInfo.isdbt_en, 0);
                    msleep(30);
                    dev_err(&IsdbCtrlInfo.spi_dev->dev, "[dtv] could not enable clock\n");
                    return rc;
                }
            }
        }

        if(IsdbCtrlInfo.isdbt_use_lna_en || IsdbCtrlInfo.isdbt_use_lna_ctrl) {
            gpio_set_value(IsdbCtrlInfo.isdbt_lna_en, 1); /* EN */
            gpio_set_value(IsdbCtrlInfo.isdbt_lna_ctrl, 0); /* GS */
        }

        driver_mode = ISDBT_POWERON;
    }
    else
    {
	printk("[dtv] already on!! \n");
    }

    IsdbCtrlInfo.pwr_state = 1;
    return rc;
}

int fc8180_is_power_on()
{
    return (int)IsdbCtrlInfo.pwr_state;
}

unsigned int fc8180_get_xtal_freq(void)
{
    return (unsigned int)IsdbCtrlInfo.isdbt_xtal_freq;
}

int fc8180_stop(void)
{
    if (driver_mode == ISDBT_DATAREAD) {
            driver_mode = ISDBT_POWEROFF;
            ms_wait(200);
    }
    driver_mode = ISDBT_POWEROFF;

    return OK;
}

int fc8180_power_off(void)
{
    driver_mode = ISDBT_POWEROFF;

    if (IsdbCtrlInfo.pwr_state == 0) {
	print_log(NULL, "[FC8180] power is immediately off\n");

	return OK;
    } 
    else {
	print_log(NULL, "[FC8180] power_off\n");

        if(IsdbCtrlInfo.isdbt_use_lna_en || IsdbCtrlInfo.isdbt_use_lna_ctrl) {
            gpio_set_value(IsdbCtrlInfo.isdbt_lna_en, 0); /* EN */
            gpio_set_value(IsdbCtrlInfo.isdbt_lna_ctrl, 0); /* GS */
        }

        if(!IsdbCtrlInfo.isdbt_use_xtal) {
            if(IsdbCtrlInfo.xo_clk != NULL)
            {
                clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
            }
        }

        if(IsdbCtrlInfo.ctrl_isdbt_ldo || IsdbCtrlInfo.ctrl_lna_ldo) {
            broadcast_isdbt_set_regulator(0);
        }            

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
        tunerbb_drv_hw_deinit();
#else
        gpio_set_value(IsdbCtrlInfo.isdbt_en, 0);
        //mdelay(1);
        //gpio_set_value(IsdbCtrlInfo.isdbt_reset, 0);
        //mdelay(5);
#endif
        msleep(10);
    }

#ifndef _NOT_USE_WAKE_LOCK_
    wake_unlock(&IsdbCtrlInfo.wake_lock);
#endif
    IsdbCtrlInfo.pwr_state = 0;
    return OK;
}

static int broadcast_isdbt_config_gpios(void)
{
    int rc = OK;
    int err_count = 0;

    IsdbCtrlInfo.isdbt_en = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,en-gpio", 0);
    rc =  gpio_request(IsdbCtrlInfo.isdbt_en, "ISDBT_EN");

    if(rc < 0) {
        err_count++;
        printk("[dtv] Failed ISDBT_EN GPIO request\n");
    }
    gpio_direction_output(IsdbCtrlInfo.isdbt_en, 0);
    udelay(10);

    printk("[dtv] isdbt_en =(%d)\n", IsdbCtrlInfo.isdbt_en);

    IsdbCtrlInfo.isdbt_irq = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,irq-gpio", 0);

    rc =  gpio_request(IsdbCtrlInfo.isdbt_irq, "ISDBT_INT");
    if(rc < 0) {
        err_count++;
        printk("[dtv]Failed ISDBT_INT GPIO request\n");
    }
    gpio_direction_input(IsdbCtrlInfo.isdbt_irq);
    udelay(10);

    printk("[dtv] isdbt_irq =(%d)\n", IsdbCtrlInfo.isdbt_irq);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "use-ant-sw", &IsdbCtrlInfo.isdbt_use_ant_sw);
    printk("[dtv] isdbt_use_ant_sw : %d\n", IsdbCtrlInfo.isdbt_use_ant_sw);

    if(IsdbCtrlInfo.isdbt_use_ant_sw) {
        of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ant-active-mode", &IsdbCtrlInfo.isdbt_ant_active_mode);
    	printk("[dtv] isdbt_ant_active_mode : %d\n", IsdbCtrlInfo.isdbt_ant_active_mode);

        IsdbCtrlInfo.isdbt_ant = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,ant-gpio", 0);
        rc =  gpio_request(IsdbCtrlInfo.isdbt_ant, "ISDBT_ANT");

        if(rc < 0) {
            err_count++;
            printk("[dtv] Failed ISDBT_ANT GPIO request\n");
        }
        gpio_direction_output(IsdbCtrlInfo.isdbt_ant, 0);
        udelay(10);

        printk("[dtv] isdbt_ant =(%d)\n", IsdbCtrlInfo.isdbt_ant);
    }

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "use-xtal", &IsdbCtrlInfo.isdbt_use_xtal);
    printk("[dtv] use_xtal : %d\n", IsdbCtrlInfo.isdbt_use_xtal);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "xtal-freq", &IsdbCtrlInfo.isdbt_xtal_freq);
    IsdbCtrlInfo.spi_dev->max_speed_hz     = (IsdbCtrlInfo.isdbt_xtal_freq*1000);
    printk("[dtv] isdbt_xtal_freq : %d\n", IsdbCtrlInfo.isdbt_xtal_freq);

#if 0
    IsdbCtrlInfo.isdbt_reset = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,reset-gpio" ,0);

    rc =  gpio_request(IsdbCtrlInfo.isdbt_reset, "ISDBT_RESET");
    if(rc < 0) {
        err_count++;
        printk("[dtv] Failed ISDBT_RESET GPIO request\n");
    }
    gpio_direction_output(IsdbCtrlInfo.isdbt_reset, 0);
    udelay(10);

    printk("[dtv] isdbt_reset =(%d)\n", IsdbCtrlInfo.isdbt_reset);
#endif

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ctrl-isdbt-ldo", &IsdbCtrlInfo.ctrl_isdbt_ldo);
    printk("[dtv] ctrl-isdbt-ldo : %d\n", IsdbCtrlInfo.ctrl_isdbt_ldo);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "ctrl-lna-ldo", &IsdbCtrlInfo.ctrl_lna_ldo);
    printk("[dtv] ctrl-lna-ldo : %d\n", IsdbCtrlInfo.ctrl_lna_ldo);

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "use-lna-en", &IsdbCtrlInfo.isdbt_use_lna_en);
    printk("[dtv] isdbt_use_lna_en : %d\n", IsdbCtrlInfo.isdbt_use_lna_en);

    if(IsdbCtrlInfo.isdbt_use_lna_en) {
        IsdbCtrlInfo.isdbt_lna_en = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,lna-en-gpio",0); /* EN */
        rc =  gpio_request(IsdbCtrlInfo.isdbt_lna_en, "ISDBT_LNA_EN");
        if(rc < 0) {
            err_count++;
            printk("[dtv] Failed ISDBT_LNA_EN GPIO request\n");
        }
        gpio_direction_output(IsdbCtrlInfo.isdbt_lna_en, 0); /* EN */
        udelay(10);
    }

    of_property_read_u32(IsdbCtrlInfo.pdev->dev.of_node, "use-lna-ctrl", &IsdbCtrlInfo.isdbt_use_lna_ctrl);
    printk("[dtv] isdbt_use_lna_ctrl : %d\n", IsdbCtrlInfo.isdbt_use_lna_ctrl);

    if(IsdbCtrlInfo.isdbt_use_lna_ctrl) {
        IsdbCtrlInfo.isdbt_lna_ctrl = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node, "isdbt_fc8180,lna-ctrl-gpio",0); /* GC */
        rc =  gpio_request(IsdbCtrlInfo.isdbt_lna_ctrl, "ISDBT_LNA_GC");
        if(rc < 0) {
            err_count++;
            printk("[dtv] Failed ISDBT_LNA_GC GPIO request\n");
        }
        gpio_direction_output(IsdbCtrlInfo.isdbt_lna_ctrl, 0); /* GS */
        udelay(10);
    }

    if(err_count > 0){
        printk("[dtv] config gpios err_count =(%d)\n", err_count);
    }

    return rc;
}

unsigned int fc8180_get_ts(void *buf, unsigned int size)
{
    s32 avail;
    ssize_t len, total_len = 0;

    if (fci_ringbuffer_empty(&RingBuffer))
        return 0;

    mutex_lock(&ringbuffer_lock);

    avail = fci_ringbuffer_avail(&RingBuffer);

    if (size >= avail)
        len = avail;
    else
        len = size - (size % 188);

    total_len = fci_ringbuffer_read_user(&RingBuffer, buf, len);

    mutex_unlock(&ringbuffer_lock);

    return total_len;
}

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
    if ((driver_mode == ISDBT_POWEROFF) || (isdbt_isr_start)) {
        print_log(NULL, "fc8180 isdbt_irq : abnormal Interrupt\
            occurred fc8180 power off state.cnt : %d\n", irq_error_cnt);
        irq_error_cnt++;
        isdbt_isr_start = 0;
    } else {
        //print_log(NULL, "fc8180 isdbt_irq : %d\n", isdbt_isr_sig);

        isdbt_isr_sig++;
        wake_up(&isdbt_isr_wait);
    }

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
    mt_eint_unmask(IsdbCtrlInfo.dtv_irq);
#endif

    return IRQ_HANDLED;
}

int data_callback(u32 hDevice, u8 *data, int len)
{
	int i;
    unsigned long ts_error_count = 0;
	totalTS += (len / 188);

	for (i = 0; i < len; i += 188) {
		if ((data[i+1] & 0x80) || data[i] != 0x47)
			ts_error_count++;
	}

    if (ts_error_count > 0) {
		totalErrTS += ts_error_count;
		print_log(NULL, "[FC8180] data_callback totalErrTS\
			: %d, len : %d\n", totalErrTS, len);
    }

	mutex_lock(&ringbuffer_lock);

	if (fci_ringbuffer_free(&RingBuffer) < len)
		FCI_RINGBUFFER_SKIP(&RingBuffer, len);

	fci_ringbuffer_write(&RingBuffer, data, len);
	
	wake_up_interruptible(&(RingBuffer.queue));

	mutex_unlock(&ringbuffer_lock);

	return 0;
}

static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

#ifdef MTK_FTRACE_TEST
	unsigned long previous_time = 0;
	unsigned int isdbt_ftrace_mode = 0;
#endif

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
	struct sched_param param = {.sched_priority = RTPM_PRIO_DTV};
	sched_setscheduler(current, SCHED_FIFO, &param);
#endif

	set_user_nice(current, -20);

	print_log(hInit, "[dtv] isdbt_kthread enter\n");

	bbm_com_ts_callback_register((u32)hInit, data_callback);

	init_waitqueue_head(&isdbt_isr_wait);

	while (1) {
		wait_event_interruptible(isdbt_isr_wait
			, isdbt_isr_sig || kthread_should_stop());

		if (driver_mode == ISDBT_POWERON) {
                    driver_mode = ISDBT_DATAREAD;

                    //print_log(hInit, "isdbt_kthread isr\n");

                    bbm_com_isr(NULL);
                    if (driver_mode == ISDBT_DATAREAD)
                        driver_mode = ISDBT_POWERON;
		}

		if (isdbt_isr_sig > 0) {
#ifdef MTK_FTRACE_TEST
			isdbt_isr_sig--;
			if (isdbt_isr_sig > 0) {
				if (isdbt_ftrace_mode == 0) {
					if (isdbt_isr_sig > 1) {
						tracing_off();
						isdbt_ftrace_mode = 1;
					} else if (isdbt_isr_sig) {
						if ((previous_time) &&
							((unsigned long)jiffies
							- previous_time)
							< 300) {
							tracing_off();
							isdbt_ftrace_mode = 1;
						}
						previous_time
						= (unsigned long)jiffies;
					}
				}
				isdbt_isr_sig = 0;
			}
#else
			isdbt_isr_sig--;
			if (isdbt_isr_sig > 0)
				isdbt_isr_sig = 0;
#endif
		}

		if (kthread_should_stop())
			break;
	}

	bbm_com_ts_callback_deregister();

	print_log(NULL, "[dtv] isdbt_kthread exit\n");

	return 0;
}

void fci_irq_disable(void)
{
    isdbt_isr_sig = 0;

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
    mt_eint_mask(IsdbCtrlInfo.dtv_irq);
#else
    print_log(NULL, "[FC8180] fci_irq_disable %d\n", IsdbCtrlInfo.spi_dev->irq);
#endif
}

void fci_irq_enable(void)
{
    isdbt_isr_sig = 0;
    //isdbt_isr_start = 1;

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
    mt_eint_unmask(IsdbCtrlInfo.dtv_irq);
#else
    print_log(NULL, "[FC8180] fci_irq_enable %d\n", IsdbCtrlInfo.spi_dev->irq);
#endif
}

void broadcast_fci_ringbuffer_flush(void)
{
	fci_ringbuffer_flush(&RingBuffer);
}

static int broadcast_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
    print_log(NULL, "Suspend\n");
    return 0;
}

static int broadcast_spi_resume(struct spi_device *spi)
{
    print_log(NULL, "Resume\n");
    return 0;
}

static int broadcast_spi_remove(struct spi_device *spi)
{
    print_log(NULL, "broadcast_spi_remove\n");

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
    mt_set_gpio_mode(GPIO_DTV_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_DTV_EN_PIN, GPIO_DIR_IN);
#else
    free_irq(IsdbCtrlInfo.isdbt_irq, NULL);
#endif

    if (isdbt_kthread) {
        kthread_stop(isdbt_kthread);
        isdbt_kthread = NULL;
    }
    bbm_com_hostif_deselect(NULL);

    wake_lock_destroy(&IsdbCtrlInfo.wake_lock);

    return 0;
}

static int broadcast_spi_probe(struct spi_device *spi)
{
    int rc = 0;
    //int ret = OK;

    print_log(NULL, "[FC8180] broadcast_spi_probe start\n");

    if(spi == NULL) {
        printk("[dtv] spi is NULL, so spi can not be set\n");
        return -1;
    }

    spi->mode 			= SPI_MODE_0;
    IsdbCtrlInfo.spi_dev = spi;
    spi->bits_per_word	= 8;
    IsdbCtrlInfo.pdev = to_platform_device(&spi->dev);

    print_log(NULL, "[FC8180] broadcast_spi_probe start\n");

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
    tunerbb_drv_hw_setting();

    IsdbCtrlInfo.dtv_irq_node = of_find_compatible_node(NULL, NULL, "mediatek,dtv");
    if (IsdbCtrlInfo.dtv_irq_node) {
        IsdbCtrlInfo.interrupt_gpio = irq_of_parse_and_map(IsdbCtrlInfo.dtv_irq_node, 0);

        if(IsdbCtrlInfo.interrupt_gpio < 0) {
            print_log(NULL, "[1seg] irq_of_parse_and_map IRQ can't parsing!.\n");
        }

        rc = request_irq(IsdbCtrlInfo.interrupt_gpio, isdbt_irq
            , IRQF_DISABLED | IRQF_TRIGGER_FALLING, "dtv", NULL);

        if (rc){
            print_log(NULL, "[1seg] dmb request irq fail : %d\n", rc);
        }
    }
#else
    /* Config GPIOs : IRQ PIN Output and 0 set at initial */
    broadcast_isdbt_config_gpios();

    rc = spi_setup(spi);
    if (rc) {
        print_log(NULL, "[FC8180] Spi setup error(%d)\n", rc);
        return rc;
    }

    printk("[dbg][dtv] IsdbCtrlInfo.isdbt_use_xtal %d, %d\n", IsdbCtrlInfo.isdbt_use_xtal, IsdbCtrlInfo.isdbt_xtal_freq);

    if(!IsdbCtrlInfo.isdbt_use_xtal) {
        IsdbCtrlInfo.xo_clk = clk_get(&IsdbCtrlInfo.spi_dev->dev, "isdbt_xo");
  	
        if(IS_ERR(IsdbCtrlInfo.xo_clk)){
            rc = PTR_ERR(IsdbCtrlInfo.xo_clk);
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "[dtv] could not get clock\n");
            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
        if (rc) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "[dtv] could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
    }

#ifndef _NOT_USE_WAKE_LOCK_
    wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
    dev_name(&spi->dev));
#endif

    fci_ringbuffer_init(&RingBuffer, &static_ringbuffer[0], RING_BUFFER_SIZE);

    if (!isdbt_kthread) {
        print_log(NULL, "kthread run\n");
        isdbt_kthread = kthread_run(isdbt_thread, NULL, "isdbt_thread");
    }

#ifdef FEATURE_DTV_USE_PINCTRL
    isdbt_pinctrl_init();
#endif

    if(IsdbCtrlInfo.ctrl_isdbt_ldo || IsdbCtrlInfo.ctrl_lna_ldo) {
        broadcast_isdbt_set_regulator(1);
        broadcast_isdbt_set_regulator(0);
    }        

    rc = request_irq(spi->irq, isdbt_irq
        , IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8180_NAME, NULL);

    if (rc){
        print_log(NULL, "[dtv] dmb request irq fail : %d\n", rc);
    }
#endif

    rc = broadcast_dmb_drv_start(&device_fc8180);
    if (rc) {
        print_log(NULL, "[dtv] Failed to load Device (%d)\n", rc);
        rc = ERROR;
    }

    if (rc < 0)
        goto free_irq;

    fci_irq_disable(); /* Must disabled */

    print_log(NULL, "[FC8180] broadcast_spi_probe End.\n");

    return 0;

free_irq:
    broadcast_spi_remove(IsdbCtrlInfo.spi_dev);

    return rc;
}

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
struct spi_device_id spi_dtv_id_table = {"dtv_spi", 0};

static struct spi_driver broadcast_spi_driver = {
	.probe = broadcast_spi_probe,
	.remove= __exit_p(broadcast_spi_remove),
	.id_table = &spi_dtv_id_table,
	.driver = {
		.name = "dtv_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
};

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias="fc8180-spi",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        //.max_speed_hz = 24*1000*1000, //24MHz
        .controller_data  = &mtk_spi_config,
    },
};
#else
//#ifdef CONFIG_OF //Open firmware must be defined for dts useage
static struct of_device_id isdbt_fc8180_table[] = {
{
	.compatible = "fci,fc8180-spi",}, //Compatible node must match dts
	{ },
};

static struct spi_driver broadcast_spi_driver = {
	.driver = {
		.name = "fc8180-spi",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = isdbt_fc8180_table,
	},

	.probe = broadcast_spi_probe,
	.suspend = broadcast_spi_suspend,
	.resume	= broadcast_spi_resume,
	.remove	= broadcast_spi_remove,
};
#endif

static int __init broadcast_dmb_fc8180_drv_init(void)
{
    int ret = 0;

    if (module_init_flag)
    	return ret;

    if(broadcast_dmb_drv_check_module_init() != OK) {
        ret = ERROR;
        return ret;
    }

#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753) || defined(CONFIG_ARCH_MT6755)
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif

    ret = spi_register_driver(&broadcast_spi_driver);
    if (ret < 0)
        print_log(NULL, "[FC8180] SPI driver register failed\n");

    printk("Linux Version : %d\n", LINUX_VERSION_CODE);
//    printk("LGE_FC8180_DRV_VER : %s\n", LGE_FC8180_DRV_VER);

    return ret;
}

static void __exit broadcast_dmb_fc8180_drv_exit(void)
{
	spi_unregister_driver(&broadcast_spi_driver);
}

#if 0
static int broadcast_isdbt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	int addr = 0;

	print_log(NULL, "broadcast_isdbt_i2c_probe client:0x%X\n", (unsigned int)client);

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		print_log(NULL, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
    IsdbCtrlInfo.pdev = to_platform_device(&client->dev);

	addr = client->addr; //Slave Addr
	pr_err("[dtv] i2c Slaveaddr [%x] \n", addr);

	IsdbCtrlInfo.pclient = client;
	//i2c_set_clientdata(client, (void*)&IsdbCtrlInfo.pclient);

    if(!IsdbCtrlInfo.isdbt_use_xtal) {
        IsdbCtrlInfo.xo_clk = clk_get(&IsdbCtrlInfo.pclient->dev, "isdbt_xo");
        if(IS_ERR(IsdbCtrlInfo.xo_clk)){
            rc = PTR_ERR(IsdbCtrlInfo.xo_clk);
            dev_err(&IsdbCtrlInfo.pclient->dev, "[dtv]could not get clock\n");
            return rc;
        }
        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
        if(rc) {
            dev_err(&IsdbCtrlInfo.pclient->dev, "[dtv] could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
    }

#ifdef FEATURE_DTV_USE_PINCTRL
    isdbt_pinctrl_init();
#endif

    /* Config GPIOs */
    broadcast_isdbt_config_gpios();

    if(IsdbCtrlInfo.ctrl_isdbt_ldo || IsdbCtrlInfo.ctrl_lna_ldo) {
        broadcast_isdbt_set_regulator(1);
        broadcast_isdbt_set_regulator(0);
    }

#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
					dev_name(&client->dev));
#endif

#if defined (CONFIG_ARCH_MSM8992) || defined (CONFIG_ARCH_MSM8994) || defined (CONFIG_ARCH_MSM8996) || defined (CONFIG_ARCH_MSM8937) || defined (CONFIG_ARCH_MSM8940)
    fc8300_power_on();
    tunerbb_drv_fc8300_read_chip_id();
    fc8300_power_off();
#endif

	return rc;
}

static int broadcast_isdbt_i2c_remove(struct i2c_client* client)
{
	int rc = 0;

	print_log(NULL, "[%s]\n", __func__);
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
	memset((unsigned char *)&IsdbCtrlInfo, 0x0, sizeof(struct broadcast_fc8180_ctrl_data));
	/*TcpalDeleteSemaphore(&fc8180DrvSem);*/
	return rc;
}

static int broadcast_isdbt_i2c_suspend(struct i2c_client* client, pm_message_t mesg)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static int broadcast_isdbt_i2c_resume(struct i2c_client* client)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static const struct i2c_device_id isdbt_fc8180_id[] = {
	{"tcc3535_i2c",	0},
	{},
};

MODULE_DEVICE_TABLE(i2c, isdbt_fc8180_id);


static struct of_device_id tcc3535_i2c_table[] = {
{ .compatible = "telechips,tcc3535-i2c",},
{ },
};

static struct i2c_driver broadcast_isdbt_driver = {
	.driver = {
		.name = "tcc3535_i2c",
		.owner = THIS_MODULE,
		.of_match_table = tcc3535_i2c_table,
	},
	.probe = broadcast_isdbt_i2c_probe,
	.remove	= __devexit_p(broadcast_isdbt_i2c_remove),
	.id_table = isdbt_fc8180_id,
	.suspend = broadcast_isdbt_i2c_suspend,
	.resume  = broadcast_isdbt_i2c_resume,
};

int __devinit broadcast_dmb_drv_init(void)
{
	int rc;
	print_log(NULL, "[%s]\n", __func__);
	rc = broadcast_dmb_drv_start();
	if (rc) {
		print_log(NULL, "failed to load\n");
		return rc;
	}
	print_log(NULL, "[%s add i2c driver]\n", __func__);
	rc = i2c_add_driver(&broadcast_isdbt_driver);
	print_log(NULL, "broadcast_add_driver rc = (%d)\n", rc);
	return rc;
}

static void __exit broadcast_dmb_drv_exit(void)
{
	i2c_del_driver(&broadcast_isdbt_driver);
}
#endif // 0

module_init(broadcast_dmb_fc8180_drv_init);
module_exit(broadcast_dmb_fc8180_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("FCI");
