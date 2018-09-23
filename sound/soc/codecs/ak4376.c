/*
 * ak4376.c  --  audio driver for AK4376
 *
 * Copyright (C) 2015 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision      DS ver.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  K.Tsubota           15/06/12	    1.0           00
 *  K.Tsubota           16/06/17	    1.1           00
 *  K.Tsubota           16/10/31	    1.2           01
 *  K.Tsubota           16/11/24	    1.3           01
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  kernel version: 3.18
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <trace/events/regmap.h>

#include "ak4376.h"

//#define AK4376_DEBUG				//used at debug mode
#define AK4376_CONTIF_DEBUG		//used at debug mode
//#define CONFIG_DEBUG_FS_CODEC		//used at debug mode
#define AK4376_DETECT_IC

/* AK4376 Codec Private Data */
struct ak4376_priv {
	struct mutex mutex;
	unsigned int ldo_sw_en;				//Main Power
	unsigned int priv_pdn_en;			//PDN GPIO pin
	int pdn1;							//PDN control, 0:Off, 1:On, 2:No use(assume always On)
	int pdn2;							//PDN control for kcontrol
	int fs1;
	int format;
	int ch;
	int rclk;							//Master Clock
	int nBickFreq;						//0:32fs, 1:48fs, 2:64fs
	int nPllMode;
	int nPllMCKI;						//0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
	int nDeviceID;						//0:AK4375, 1:AK4375A, 2:AK4376, 3:Other IC
	int lpmode;							//0:High Performance, 1:Low power mode
	int xtalfreq;						//0:12.288MHz, 1:11.2896MHz
	int nDACOn;
	int connected;
	struct i2c_client *i2c;
	struct regmap *regmap;
};

static const struct reg_default ak4376_patch[] = {
	{ 0x0B, 0x99 },      /*	0x0B	AK4376_0B_LCH_OUTPUT_VOLUME */
	{ 0x26, REG_26_VAL },/*	0x26	AK4376_26_DAC32_DEBUG2		*/
	{ 0x2A, REG_2A_VAL },/*	0x2A	AK4376_2A_LDO3_DEBUG1		*/
};
/* ak4376 register cache & default register settings */
static const struct reg_default ak4376_reg[] = {
	{ 0x0, 0x00 },	/*	0x00	AK4376_00_POWER_MANAGEMENT1		*/
	{ 0x1, 0x00 },	/*	0x01	AK4376_01_POWER_MANAGEMENT2		*/
	{ 0x2, 0x00 },	/*	0x02	AK4376_02_POWER_MANAGEMENT3		*/
	{ 0x3, 0x00 },	/*	0x03	AK4376_03_POWER_MANAGEMENT4		*/
	{ 0x4, 0x00 },	/*	0x04	AK4376_04_OUTPUT_MODE_SETTING	*/
	{ 0x5, 0x00 },	/*	0x05	AK4376_05_CLOCK_MODE_SELECT		*/
	{ 0x6, 0x00 },	/*	0x06	AK4376_06_DIGITAL_FILTER_SELECT	*/
	{ 0x7, 0x00 },	/*	0x07	AK4376_07_DAC_MONO_MIXING		*/
	{ 0x8, 0x00 },	/*	0x08	AK4376_08_RESERVED				*/
	{ 0x9, 0x00 },	/*	0x09	AK4376_09_RESERVED				*/
	{ 0xA, 0x00 },	/*	0x0A	AK4376_0A_RESERVED				*/
	{ 0xB, 0x19 },	/*	0x0B	AK4376_0B_LCH_OUTPUT_VOLUME		*/
	{ 0xC, 0x19 },	/*	0x0C	AK4376_0C_RCH_OUTPUT_VOLUME		*/
	{ 0xD, 0x0b },	/*	0x0D	AK4376_0D_HP_VOLUME_CONTROL		*/
	{ 0xE, 0x00 },	/*	0x0E	AK4376_0E_PLL_CLK_SOURCE_SELECT	*/
	{ 0xF, 0x00 },	/*	0x0F	AK4376_0F_PLL_REF_CLK_DIVIDER1	*/
	{ 0x10, 0x00 },	/*	0x10	AK4376_10_PLL_REF_CLK_DIVIDER2	*/
	{ 0x11, 0x00 },	/*	0x11	AK4376_11_PLL_FB_CLK_DIVIDER1	*/
	{ 0x12, 0x00 },	/*	0x12	AK4376_12_PLL_FB_CLK_DIVIDER2	*/
	{ 0x13, 0x00 },	/*	0x13	AK4376_13_DAC_CLK_SOURCE		*/
	{ 0x14, 0x00 },	/*	0x14	AK4376_14_DAC_CLK_DIVIDER		*/
	{ 0x15, 0x40 },	/*	0x15	AK4376_15_AUDIO_IF_FORMAT		*/
	{ 0x16, 0x00 },	/*	0x16	AK4376_16_DUMMY					*/
	{ 0x17, 0x00 },	/*	0x17	AK4376_17_DUMMY					*/
	{ 0x18, 0x00 },	/*	0x18	AK4376_18_DUMMY					*/
	{ 0x19, 0x00 },	/*	0x19	AK4376_19_DUMMY					*/
	{ 0x1A, 0x00 },	/*	0x1A	AK4376_1A_DUMMY					*/
	{ 0x1B, 0x00 },	/*	0x1B	AK4376_1B_DUMMY					*/
	{ 0x1C, 0x00 },	/*	0x1C	AK4376_1C_DUMMY					*/
	{ 0x1D, 0x00 },	/*	0x1D	AK4376_1D_DUMMY					*/
	{ 0x1E, 0x00 },	/*	0x1E	AK4376_1E_DUMMY					*/
	{ 0x1F, 0x00 },	/*	0x1F	AK4376_1F_DUMMY					*/
	{ 0x20, 0x00 },	/*	0x20	AK4376_20_DUMMY					*/
	{ 0x21, 0x01 },	/*	0x21	AK4376_21_DUMMY					*/
	{ 0x22, 0x00 },	/*	0x22	AK4376_22_DUMMY					*/
	{ 0x23, 0x00 },	/*	0x23	AK4376_23_DUMMY					*/
	{ 0x24, 0x00 },	/*	0x24	AK4376_24_MODE_CONTROL			*/
	{ 0x25, 0x6C },	/*	0x25	AK4376_25_DAC32_DEBUG1			*/
	{ 0x26, 0x40 },	/*	0x26	AK4376_26_DAC32_DEBUG2		*/
	{ 0x27, 0x00 },	/*	0x27	AK4376_27_HDP_DEBUG1			*/
	{ 0x28, 0x00 },	/*	0x28	AK4376_28_HDP_DEBUG2			*/
	{ 0x29, 0x00 },	/*	0x29	AK4376_29_DADIG_MODE			*/
	{ 0x2A, 0x00 },/*	0x2A	AK4376_2A_LDO3_DEBUG1		*/
	{ 0x2B, 0x00 },	/*	0x2B	AK4376_2B_LDO3_DEBUG2			*/
	{ 0x2C, 0x00 },	/*	0x2C	AK4376_2C_PLL_DEBUG			*/
};

static const u8 ak4376_reg_default[AK4376_MAX_REGISTERS] = {
	0x00,	/*	0x00	AK4376_00_POWER_MANAGEMENT1		*/
	0x00,	/*	0x01	AK4376_01_POWER_MANAGEMENT2		*/
	0x00,	/*	0x02	AK4376_02_POWER_MANAGEMENT3		*/
	0x00,	/*	0x03	AK4376_03_POWER_MANAGEMENT4		*/
	0x00,	/*	0x04	AK4376_04_OUTPUT_MODE_SETTING	*/
	0x00,	/*	0x05	AK4376_05_CLOCK_MODE_SELECT		*/
	0x00,	/*	0x06	AK4376_06_DIGITAL_FILTER_SELECT	*/
	0x00,	/*	0x07	AK4376_07_DAC_MONO_MIXING		*/
	0x00,	/*	0x08	AK4376_08_RESERVED				*/
	0x00,	/*	0x09	AK4376_09_RESERVED				*/
	0x00,	/*	0x0A	AK4376_0A_RESERVED				*/
	0x19,	/*	0x0B	AK4376_0B_LCH_OUTPUT_VOLUME		*/
	0x19,	/*	0x0C	AK4376_0C_RCH_OUTPUT_VOLUME		*/
	0x0B,	/*	0x0D	AK4376_0D_HP_VOLUME_CONTROL		*/
	0x00,	/*	0x0E	AK4376_0E_PLL_CLK_SOURCE_SELECT	*/
	0x00,	/*	0x0F	AK4376_0F_PLL_REF_CLK_DIVIDER1	*/
	0x00,	/*	0x10	AK4376_10_PLL_REF_CLK_DIVIDER2	*/
	0x00,	/*	0x11	AK4376_11_PLL_FB_CLK_DIVIDER1	*/
	0x00,	/*	0x12	AK4376_12_PLL_FB_CLK_DIVIDER2	*/
	0x00,	/*	0x13	AK4376_13_DAC_CLK_SOURCE		*/
	0x00,	/*	0x14	AK4376_14_DAC_CLK_DIVIDER		*/
	0x40,	/*	0x15	AK4376_15_AUDIO_IF_FORMAT		*/
	0x00,	/*	0x16	AK4376_16_DUMMY					*/
	0x00,	/*	0x17	AK4376_17_DUMMY					*/
	0x00,	/*	0x18	AK4376_18_DUMMY					*/
	0x00,	/*	0x19	AK4376_19_DUMMY					*/
	0x00,	/*	0x1A	AK4376_1A_DUMMY					*/
	0x00,	/*	0x1B	AK4376_1B_DUMMY					*/
	0x00,	/*	0x1C	AK4376_1C_DUMMY					*/
	0x00,	/*	0x1D	AK4376_1D_DUMMY					*/
	0x00,	/*	0x1E	AK4376_1E_DUMMY					*/
	0x00,	/*	0x1F	AK4376_1F_DUMMY					*/
	0x00,	/*	0x20	AK4376_20_DUMMY					*/
	0x01,	/*	0x21	AK4376_21_DUMMY					*/
	0x00,	/*	0x22	AK4376_22_DUMMY					*/
	0x00,	/*	0x23	AK4376_23_DUMMY					*/
	0x00,	/*	0x24	AK4376_24_MODE_CONTROL			*/
	0x6C,	/*	0x25	AK4376_25_DAC32_DEBUG1			*/
	0x40,	/*	0x26	AK4376_26_DAC32_DEBUG2			*/
	0x00,	/*	0x27	AK4376_27_HDP_DEBUG1			*/
	0x00,	/*	0x28	AK4376_28_HDP_DEBUG2			*/
	0x00,	/*	0x29	AK4376_29_DADIG_MODE			*/
	0x00,	/*	0x2A	AK4376_2A_LDO3_DEBUG1			*/
	0x00,	/*	0x2B	AK4376_2B_LDO3_DEBUG2			*/
	0x00,	/*	0x2C	AK4376_2C_PLL_DEBUG			*/
};

int ak4376_digital_volume[2] = {0x19,0x19}; //default value
int lrV_offset[2]={0,0};//0(0dB)~12(-6dB)

unsigned int ak4376_reg_read(struct snd_soc_codec *, unsigned int);
static int ak4376_write_register(struct snd_soc_codec *, unsigned int, unsigned int);
#ifdef CONFIG_DEBUG_FS_CODEC
static struct snd_soc_codec *ak4376_codec;
static int ak4376_reg_write(struct snd_soc_codec *, u16, u16);
#endif
#ifdef AK4376_DETECT_IC
static int  ak4376_hifi_support=1;
static int __init verified_hifi_support_param(char *line)
{
	int ret;

	ret = strncmp(line,"disable",7);
	if(!ret){
		ak4376_hifi_support= 0;
		printk(KERN_INFO "hifi support = %s\n",line);
	}
	return 1;
}
__setup("audio.hifi=", verified_hifi_support_param);
#endif

static int ak4376_update_register(struct snd_soc_codec *codec)
{
	int i;

	for(i=0; i< ARRAY_SIZE(ak4376_patch); i++){
		snd_soc_write(codec,ak4376_patch[i].reg,ak4376_patch[i].def);
	}

	return 0;
}

static int ak4376_pdn_control(struct snd_soc_codec *codec, int pdn)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d) pdn=%d\n",__FUNCTION__,__LINE__,pdn);
	if(!ak4376->connected)
		return -ENODEV;

	if ((ak4376->pdn1 == 0) && (pdn == 1)) {
		gpio_direction_output(ak4376->priv_pdn_en, 1);
		dev_dbg(codec->dev,"%s(%d) Turn on priv_pdn_en\n", __FUNCTION__,__LINE__);
		ak4376->pdn1 = 1;
		ak4376->pdn2 = 1;
		udelay(800);

		regcache_sync(ak4376->regmap);
		ak4376_update_register(codec);

	} else if ((ak4376->pdn1 == 1) && (pdn == 0)) {
		i2c_lock_adapter(ak4376->i2c->adapter);
		gpio_direction_output(ak4376->priv_pdn_en, 0);
		i2c_unlock_adapter(ak4376->i2c->adapter);
		dev_dbg(codec->dev," %s(%d) Turn off priv_pdn_en\n", __FUNCTION__,__LINE__);
		ak4376->pdn1 = 0;
		ak4376->pdn2 = 0;
		regcache_mark_dirty(ak4376->regmap);
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS_CODEC
static struct mutex io_lock;

static ssize_t reg_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, i, fpt;
#ifdef DAC_DEBUG
	int rx[30];
#else
	int rx[22];
#endif

	mutex_lock(&io_lock);
	for (i = 0; i < AK4376_16_DUMMY; i++) {
		ret = ak4376_reg_read(ak4376_codec, i);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

#ifdef DAC_DEBUG
	for (i = AK4376_24_MODE_CONTROL; i < AK4376_MAX_REGISTERS; i++) {
		ret = ak4376_reg_read(ak4376_codec, i);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}
#else
	rx[22] = ak4376_reg_read(ak4376_codec, AK4376_24_MODE_CONTROL);
#endif
	mutex_unlock(&io_lock);

#ifdef DAC_DEBUG
	if (i == 30) {

		ret = fpt = 0;

		for (i = 0; i < AK4376_16_DUMMY; i++, fpt += 6) {

			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
		}

		for (i = AK4376_24_MODE_CONTROL; i < AK4376_MAX_REGISTERS; i++, fpt += 6) {

			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
		}

		return ret;

	} else {

		return sprintf(buf, "read error");
	}
#else
	if (i == 22) {

		ret = fpt = 0;

		for (i = 0; i < AK4376_16_DUMMY; i++, fpt += 6) {

			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
		}

		ret += sprintf(buf + i * 6, "24,%02x\n", rx[22]);

		return ret;

	} else {

		return sprintf(buf, "read error");
	}
#endif
}

static ssize_t reg_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	*ptr_data = (char *)buf;
	char	*p;
	int		i, pt_count = 0;
	unsigned short val[20];

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 20)
			break;

		val[pt_count] = simple_strtoul(p, NULL, 16);

		pt_count++;
	}

	mutex_lock(&io_lock);
	for (i = 0; i < pt_count; i+=2) {
		ak4376_reg_write(ak4376_codec, val[i], val[i+1]);
		pr_debug("%s: write add=%d, val=%d.\n", __func__, val[i], val[i+1]);
	}
	mutex_unlock(&io_lock);

	return count;
}

static DEVICE_ATTR(reg_data, 0666, reg_data_show, reg_data_store);

#endif

/* Output Digital volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB) */
//static DECLARE_TLV_DB_SCALE(ovl_tlv, -1250, 50, 0);
//static DECLARE_TLV_DB_SCALE(ovr_tlv, -1250, 50, 0);

/* HP-Amp Analog volume control:
 * from -22 to 6 dB in 2 dB steps (mute instead of -42 dB) */
static DECLARE_TLV_DB_SCALE(hpg_tlv, -2200, 200, 0);

static const char *ak4376_ovolcn_select_texts[] = {"Dependent", "Independent"};
static const char *ak4376_mdacl_select_texts[] = {"x1", "x1/2"};
static const char *ak4376_mdacr_select_texts[] = {"x1", "x1/2"};
static const char *ak4376_invl_select_texts[] = {"Normal", "Inverting"};
static const char *ak4376_invr_select_texts[] = {"Normal", "Inverting"};
static const char *ak4376_cpmod_select_texts[] =
		{"Automatic Switching", "+-VDD Operation", "+-1/2VDD Operation"};
static const char *ak4376_hphl_select_texts[] = {"9ohm", "Hi-Z"};
static const char *ak4376_hphr_select_texts[] = {"9ohm", "Hi-Z"};
static const char *ak4376_dacfil_select_texts[]  =
		{"Sharp Roll-Off", "Slow Roll-Off", "Short Delay Sharp Roll-Off", "Short Delay Slow Roll-Off"};
static const char *ak4376_bcko_select_texts[] = {"64fs", "32fs"};
static const char *ak4376_dfthr_select_texts[] = {"Digital Filter", "Bypass"};
static const char *ak4376_ngate_select_texts[] = {"On", "Off"};
static const char *ak4376_ngatet_select_texts[] = {"Short", "Long"};

static const struct soc_enum ak4376_dac_enum[] = {
	SOC_ENUM_SINGLE(AK4376_0B_LCH_OUTPUT_VOLUME, 7,
			ARRAY_SIZE(ak4376_ovolcn_select_texts), ak4376_ovolcn_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 2,
			ARRAY_SIZE(ak4376_mdacl_select_texts), ak4376_mdacl_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 6,
			ARRAY_SIZE(ak4376_mdacr_select_texts), ak4376_mdacr_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 3,
			ARRAY_SIZE(ak4376_invl_select_texts), ak4376_invl_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 7,
			ARRAY_SIZE(ak4376_invr_select_texts), ak4376_invr_select_texts),
	SOC_ENUM_SINGLE(AK4376_03_POWER_MANAGEMENT4, 2,
			ARRAY_SIZE(ak4376_cpmod_select_texts), ak4376_cpmod_select_texts),
	SOC_ENUM_SINGLE(AK4376_04_OUTPUT_MODE_SETTING, 0,
			ARRAY_SIZE(ak4376_hphl_select_texts), ak4376_hphl_select_texts),
	SOC_ENUM_SINGLE(AK4376_04_OUTPUT_MODE_SETTING, 1,
			ARRAY_SIZE(ak4376_hphr_select_texts), ak4376_hphr_select_texts),
    SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 6,
			ARRAY_SIZE(ak4376_dacfil_select_texts), ak4376_dacfil_select_texts),
	SOC_ENUM_SINGLE(AK4376_15_AUDIO_IF_FORMAT, 3,
			ARRAY_SIZE(ak4376_bcko_select_texts), ak4376_bcko_select_texts),
	SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 3,
			ARRAY_SIZE(ak4376_dfthr_select_texts), ak4376_dfthr_select_texts),
	SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 0,
			ARRAY_SIZE(ak4376_ngate_select_texts), ak4376_ngate_select_texts),
	SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 1,
			ARRAY_SIZE(ak4376_ngatet_select_texts), ak4376_ngatet_select_texts),
};

static const char *bickfreq_on_select[] = {"32fs", "48fs", "64fs"};
static const char *pllmcki_on_select[] = {"9.6MHz", "11.2896MHz", "12.288MHz", "19.2MHz"};
static const char *lpmode_on_select[] = {"High Performance", "Low Power"};
static const char *xtalfreq_on_select[] = {"12.288MHz", "11.2896MHz", "19.2MHz", "24.576MHz"};
static const char *pdn_on_select[] = {"Off", "On"};
static const char *pllmode_on_select[] = {"PLL_OFF", "PLL_BICK_MODE", "PLL_MCKI_MODE", "XTAL_MODE", "XTAL_PLL_MODE"};
static const char *chipstate_on_select[] = {"fail", "success"};

static const struct soc_enum ak4376_bitset_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bickfreq_on_select), bickfreq_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pllmcki_on_select), pllmcki_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lpmode_on_select), lpmode_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(xtalfreq_on_select), xtalfreq_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pdn_on_select), pdn_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pllmode_on_select), pllmode_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(chipstate_on_select), chipstate_on_select),
};

static int get_bickfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

    ucontrol->value.enumerated.item[0] = ak4376->nBickFreq;

    return 0;
}

static int ak4376_set_bickfs(struct snd_soc_codec *codec)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d) nBickFreq=%d\n",__FUNCTION__,__LINE__,ak4376->nBickFreq);

	if ( ak4376->nBickFreq == 0 ) { 	//32fs
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
	}
	else if( ak4376->nBickFreq == 1 ) {	//48fs
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x00);	//DL1-0=00(24bit, >=48fs)
	}
	else {								//64fs
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x02);	//DL1-0=1x(32bit, >=64fs)
	}

	return 0;
}

static int set_bickfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ak4376->nBickFreq = ucontrol->value.enumerated.item[0];

	ak4376_set_bickfs(codec);

    return 0;
}

static int get_pllmcki(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4376->nPllMCKI;

    return 0;
}

static int set_pllmcki(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	ak4376->nPllMCKI = ucontrol->value.enumerated.item[0];

    return 0;
}

static int get_lpmode(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4376->lpmode;

    return 0;
}

static int ak4376_set_lpmode(struct snd_soc_codec *codec)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	if ( ak4376->lpmode == 0 ) { 	//High Performance Mode
		snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x10, 0x00);	//LPMODE=0(High Performance Mode)
			if ( ak4376->fs1 <= 12000 ) {
				snd_soc_update_bits(codec, AK4376_24_MODE_CONTROL, 0x40, 0x40);	//DSMLP=1
			}
			else {
				snd_soc_update_bits(codec, AK4376_24_MODE_CONTROL, 0x40, 0x00);	//DSMLP=0
			}
	}
	else {							//Low Power Mode
		snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x10, 0x10);	//LPMODE=1(Low Power Mode)
		snd_soc_update_bits(codec, AK4376_24_MODE_CONTROL, 0x40, 0x40);			//DSMLP=1
	}

	return 0;
}

static int set_lpmode(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	ak4376->lpmode = ucontrol->value.enumerated.item[0];

	ak4376_set_lpmode(codec);

    return 0;
}

static int get_chipstate(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    dev_dbg(codec->dev,"%s(%d) connected=%d\n",__FUNCTION__,__LINE__,ak4376->connected);
    ucontrol->value.enumerated.item[0] = ak4376->connected;

    return 0;
}

static int set_chipstate(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    dev_dbg(codec->dev,"%s(%d) connected=%d\n",__FUNCTION__,__LINE__,ak4376->connected);
    //ak4376->connected = ucontrol->value.enumerated.item[0];

    return 0;
}

static int get_xtalfreq(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4376->xtalfreq;

    return 0;
}

static int set_xtalfreq(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	ak4376->xtalfreq = ucontrol->value.enumerated.item[0];

    return 0;
}

static int get_pdn(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

    ucontrol->value.enumerated.item[0] = ak4376->pdn2;

    return 0;
}

static int set_pdn(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	ak4376->pdn2 = ucontrol->value.enumerated.item[0];

	dev_dbg(codec->dev,"%s(%d)  pdn1=%d, pdn2=%d\n",__FUNCTION__,__LINE__,ak4376->pdn1,ak4376->pdn2);

	//if (ak4376->pdn1 == 0)
	ak4376_pdn_control(codec, ak4376->pdn2);

    return 0;
}

static int get_pllmode(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

    ucontrol->value.enumerated.item[0] = ak4376->nPllMode;

    return 0;
}

static int set_pllmode(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	ak4376->nPllMode = ucontrol->value.enumerated.item[0];

	dev_dbg(codec->dev,"%s(%d) nPllMode=%d\n",__FUNCTION__,__LINE__,ak4376->nPllMode);

    return 0;
}

#ifdef AK4376_DEBUG

static const char *test_reg_select[]   =
{
    "read AK4376 Reg 00:24",
};

static const struct soc_enum ak4376_enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo = 0;

static int get_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    /* Get the current output routing */
    ucontrol->value.enumerated.item[0] = nTestRegNo;

    return 0;
}

static int set_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    u32    currMode = ucontrol->value.enumerated.item[0];
	int    i, value;
	int	   regs, rege;

	nTestRegNo = currMode;

	regs = 0x00;
	rege = 0x15;

	for ( i = regs ; i <= rege ; i++ ){
		value = snd_soc_read(codec, i);
		printk("***AK4376 Addr,Reg=(%x, %x)\n", i, value);
	}
#ifdef DAC_DEBUG
	for ( i = AK4376_24_MODE_CONTROL ; i <= AK4376_2C_PLL_DEBUG ; i++ ){
		value = snd_soc_read(codec, i);
		printk("***AK4376 Addr,Reg=(%x, %x)\n", i, value);
	}
#else
	value = snd_soc_read(codec, 0x24);
	printk("***AK4376 Addr,Reg=(%x, %x)\n", 0x24, value);
#endif

	return 0;
}
#endif

static int get_lrOffset(int offset)
{
	offset = -offset;
	if (offset > 0)
		offset = 0;
	if (offset < -12)
		offset = -12;
	return offset;
}
static int check_volume(int val,int max)
{
	if (val > max)
		val = max;
	if (val < 0)
		val = 0;
	return val;
}


static int ak4376_digital_volumeL_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ak4376_digital_volume[0];
	return 0;
}
static int ak4376_digital_volumeL_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val;
	int reg = AK4376_0B_LCH_OUTPUT_VOLUME;
	int offset = lrV_offset[0];
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);


	val= (int)ucontrol->value.integer.value[0];
	val = check_volume(val,0x1F);
	ak4376_digital_volume[0] = val;

	offset = get_lrOffset(offset);
	val = check_volume(val + offset,0x1F);
	snd_soc_update_bits(codec,reg,0x1F,val);
	dev_dbg(codec->dev,"%s val=%d(%d),offset=%d\n",__FUNCTION__,ak4376_digital_volume[0],val,offset);

	return 0;
}
static int ak4376_digital_volumeR_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ak4376_digital_volume[1];
	return 0;
}
static int ak4376_digital_volumeR_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val;
	int reg = AK4376_0C_RCH_OUTPUT_VOLUME;
	int offset = lrV_offset[1];
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);

	val= (int)ucontrol->value.integer.value[0];
	val = check_volume(val,0x1F);
	ak4376_digital_volume[1] = val;

	offset = get_lrOffset(offset);
	val = check_volume(val + offset,0x1F);
	snd_soc_update_bits(codec,reg,0x1F,val);
	dev_dbg(codec->dev,"%s val=%d(%d),offset=%d\n",__FUNCTION__,ak4376_digital_volume[1],val,offset);

	return 0;
}
static int ak4376_digital_volumeL_offset_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = lrV_offset[0];
	return 0;
}
static int ak4376_digital_volumeL_offset_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val = ak4376_digital_volume[0];
	int reg = AK4376_0B_LCH_OUTPUT_VOLUME;
	int offset;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);

	offset= (int)ucontrol->value.integer.value[0];
	offset = check_volume(offset,12);
	lrV_offset[0] = offset;

	val = check_volume(val,0x1F);
	offset = get_lrOffset(offset);

	val = check_volume(val + offset,0x1F);
	snd_soc_update_bits(codec,reg,0x1F,val);
	dev_dbg(codec->dev,"%s val=%d(%d),offset=%d\n",__FUNCTION__,lrV_offset[0],val,offset);

	return 0;
}
static int ak4376_digital_volumeR_offset_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = lrV_offset[1];
	return 0;
}
static int ak4376_digital_volumeR_offset_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val = ak4376_digital_volume[1];
	int reg = AK4376_0C_RCH_OUTPUT_VOLUME;
	int offset;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);

	offset= (int)ucontrol->value.integer.value[0];
	offset = check_volume(offset,12);
	lrV_offset[1] = offset;

	val = check_volume(val,0x1F);
	offset = get_lrOffset(offset);

	val = check_volume(val + offset,0x1F);
	snd_soc_update_bits(codec,reg,0x1F,val);
	dev_dbg(codec->dev,"%s val=%d(%d),offset=%d\n",__FUNCTION__,lrV_offset[1],val,offset);

	return 0;
}


static const struct snd_kcontrol_new ak4376_snd_controls[] = {
	SOC_SINGLE_EXT("AK4376 Digital Output VolumeL", SND_SOC_NOPM, 0, 256, 0,
		ak4376_digital_volumeL_get,ak4376_digital_volumeL_put),
	SOC_SINGLE_EXT("AK4376 Digital Output VolumeR", SND_SOC_NOPM, 0, 256, 0,
		ak4376_digital_volumeR_get,ak4376_digital_volumeR_put),
	SOC_SINGLE_EXT("HiFi DAC VolumeL", SND_SOC_NOPM, 0, 256, 0,
		ak4376_digital_volumeL_offset_get,ak4376_digital_volumeL_offset_put),
	SOC_SINGLE_EXT("HiFi DAC VolumeR", SND_SOC_NOPM, 0, 256, 0,
		ak4376_digital_volumeR_offset_get,ak4376_digital_volumeR_offset_put),
	SOC_SINGLE_TLV("AK4376 HP-Amp Analog Volume",
			AK4376_0D_HP_VOLUME_CONTROL, 0, 0x0F, 0, hpg_tlv),

	SOC_ENUM("AK4376 Digital Volume Control", ak4376_dac_enum[0]),
	SOC_ENUM("AK4376 DACL Signal Level", ak4376_dac_enum[1]),
	SOC_ENUM("AK4376 DACR Signal Level", ak4376_dac_enum[2]),
	SOC_ENUM("AK4376 DACL Signal Invert", ak4376_dac_enum[3]),
	SOC_ENUM("AK4376 DACR Signal Invert", ak4376_dac_enum[4]),
	SOC_ENUM("AK4376 Charge Pump Mode", ak4376_dac_enum[5]),
	SOC_ENUM("AK4376 HPL Power-down Resistor", ak4376_dac_enum[6]),
	SOC_ENUM("AK4376 HPR Power-down Resistor", ak4376_dac_enum[7]),
	SOC_ENUM("AK4376 DAC Digital Filter Mode", ak4376_dac_enum[8]),
	SOC_ENUM("AK4376 BICK Output Frequency", ak4376_dac_enum[9]),
	SOC_ENUM("AK4376 Digital Filter Mode", ak4376_dac_enum[10]),
	SOC_ENUM("AK4376 Noise Gate", ak4376_dac_enum[11]),
	SOC_ENUM("AK4376 Noise Gate Time", ak4376_dac_enum[12]),

	SOC_ENUM_EXT("AK4376 BICK Frequency Select", ak4376_bitset_enum[0], get_bickfs, set_bickfs),
	SOC_ENUM_EXT("AK4376 PLL MCKI Frequency", ak4376_bitset_enum[1], get_pllmcki, set_pllmcki),
	SOC_ENUM_EXT("AK4376 Low Power Mode", ak4376_bitset_enum[2], get_lpmode, set_lpmode),
	SOC_ENUM_EXT("AK4376 Xtal Frequency", ak4376_bitset_enum[3], get_xtalfreq, set_xtalfreq),
	SOC_ENUM_EXT("AK4376 PDN Control", ak4376_bitset_enum[4], get_pdn, set_pdn),
	SOC_ENUM_EXT("AK4376 PLL Mode", ak4376_bitset_enum[5], get_pllmode, set_pllmode),
	SOC_ENUM_EXT("HIFI Chip State", ak4376_bitset_enum[6], get_chipstate, set_chipstate),

#ifdef AK4376_DEBUG
	SOC_ENUM_EXT("Reg Read", ak4376_enum[0], get_test_reg, set_test_reg),
#endif

};


/* DAC control */
static int ak4376_dac_event2(struct snd_soc_codec *codec, int event)
{
	u8 MSmode;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	MSmode = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
		ak4376->nDACOn = 1;
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);		//PMCP1=1
		mdelay(6);																//wait 6ms
		udelay(500);															//wait 0.5ms
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);		//PMLDO1P/N=1
		mdelay(1);																//wait 1ms
		break;
	case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);		//PMCP2=1
		mdelay(4);																//wait 4ms
		udelay(500);															//wait 0.5ms
		break;
	case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x00);		//PMCP2=0
		break;
	case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x00);		//PMLDO1P/N=0
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x00);		//PMCP1=0

	if (ak4376->nPllMode == 0) {
		if (MSmode & 0x10) {	//Master mode
			snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10,0x00);	//MS bit = 0
		}
	}

		ak4376->nDACOn = 0;

		break;
	}
	return 0;
}

static int ak4376_dac_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
	struct snd_soc_codec *codec = w->codec;

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ak4376_dac_event2(codec, event);

	return 0;
}

/* PLL control */
static int ak4376_pll_event2(struct snd_soc_codec *codec, int event)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
	case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
		if ((ak4376->nPllMode == 1) || (ak4376->nPllMode == 2)) {	//3.18
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01,0x01);	//PMPLL=1
		}
		else if (ak4376->nPllMode == 3) {	//3.18
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x10,0x10);	//PMOSC=1
		}
		else if (ak4376->nPllMode == 4) {	//XTAL & PLL On
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x11,0x11);	//PMPLL=1, PMOSC=1
		}

		break;
	case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
	case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
		if ((ak4376->nPllMode == 1) || (ak4376->nPllMode == 2)) {	//3.18
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01,0x00);	//PMPLL=0
		}
		else if (ak4376->nPllMode == 3) {
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x10,0x00);	//PMOSC=0
		}
		else if (ak4376->nPllMode == 4) {
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x11,0x00);	//PMPLL=0, PMOSC=0
		}
		break;
	}

	return 0;
}

static int ak4376_pll_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
	struct snd_soc_codec *codec = w->codec;

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ak4376_pll_event2(codec, event);

	return 0;
}

/* HPL Mixer */
static const struct snd_kcontrol_new ak4376_hpl_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACL", AK4376_07_DAC_MONO_MIXING, 0, 1, 0),
	SOC_DAPM_SINGLE("RDACL", AK4376_07_DAC_MONO_MIXING, 1, 1, 0),
};

/* HPR Mixer */
static const struct snd_kcontrol_new ak4376_hpr_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACR", AK4376_07_DAC_MONO_MIXING, 4, 1, 0),
	SOC_DAPM_SINGLE("RDACR", AK4376_07_DAC_MONO_MIXING, 5, 1, 0),
};


/* ak4376 dapm widgets */
static const struct snd_soc_dapm_widget ak4376_dapm_widgets[] = {
// DAC
	SND_SOC_DAPM_DAC_E("AK4376 DAC", "NULL", AK4376_02_POWER_MANAGEMENT3, 0, 0,
			ak4376_dac_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD
                            |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

// PLL, OSC
	SND_SOC_DAPM_SUPPLY_S("AK4376 PLL", 0, SND_SOC_NOPM, 0, 0,
			ak4376_pll_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD
                            |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

	SND_SOC_DAPM_AIF_IN("AK4376 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

// Analog Output
	SND_SOC_DAPM_OUTPUT("AK4376 HPL"),
	SND_SOC_DAPM_OUTPUT("AK4376 HPR"),

	SND_SOC_DAPM_MIXER("AK4376 HPR Mixer", AK4376_03_POWER_MANAGEMENT4, 1, 0,
			&ak4376_hpr_mixer_controls[0], ARRAY_SIZE(ak4376_hpr_mixer_controls)),

	SND_SOC_DAPM_MIXER("AK4376 HPL Mixer", AK4376_03_POWER_MANAGEMENT4, 0, 0,
			&ak4376_hpl_mixer_controls[0], ARRAY_SIZE(ak4376_hpl_mixer_controls)),

};

static const struct snd_soc_dapm_route ak4376_intercon[] =
{

	{"AK4376 DAC", "NULL", "AK4376 PLL"},
	{"AK4376 DAC", "NULL", "AK4376 SDTI"},

	{"AK4376 HPL Mixer", "LDACL", "AK4376 DAC"},
	{"AK4376 HPL Mixer", "RDACL", "AK4376 DAC"},
	{"AK4376 HPR Mixer", "LDACR", "AK4376 DAC"},
	{"AK4376 HPR Mixer", "RDACR", "AK4376 DAC"},

	{"AK4376 HPL", "NULL", "AK4376 HPL Mixer"},
	{"AK4376 HPR", "NULL", "AK4376 HPR Mixer"},

};

static int ak4376_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
	u8 mode;
	u8 mode2;
	int mcki_rate;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s fs=%d rclk=%d\n",__FUNCTION__, fs, rclk);

	if (fs != 0 ) {
		if (rclk > 28800000) return -EINVAL;
		if (rclk == 0 && ak4376->nPllMode==0) return -EINVAL;

		if (ak4376->nPllMode == 0) {	//PLL_OFF
			mcki_rate = rclk/fs;
		}
		else {		//XTAL_MODE
			if ( ak4376->xtalfreq == 0 ) {		//12.288MHz
				mcki_rate = 12288000/fs;
			}
			else if ( ak4376->xtalfreq == 1 ){	//11.2896MHz
				mcki_rate = 11289600/fs;
			}
			else if ( ak4376->xtalfreq == 3 ){	//24.576MHz
				mcki_rate = 24576000/fs;
			}
		}

		mode = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
		mode &= ~AK4376_CM;

		if (ak4376->lpmode == 0) {				//High Performance Mode
			switch (mcki_rate) {
			case 32:
				mode |= AK4376_CM_0;
				break;
			case 64:
				mode |= AK4376_CM_1;
				break;
			case 128:
				mode |= AK4376_CM_3;
				break;
			case 256:
				mode |= AK4376_CM_0;
				mode2 = snd_soc_read(codec, AK4376_24_MODE_CONTROL);
				if ( fs <= 12000 ) {
					mode2 |= 0x40;	//DSMLP=1
					snd_soc_write(codec, AK4376_24_MODE_CONTROL, mode2);
				}
				else {
					mode2 &= ~0x40;	//DSMLP=0
					snd_soc_write(codec, AK4376_24_MODE_CONTROL, mode2);
				}
				break;
			case 512:
				mode |= AK4376_CM_1;
				break;
			case 1024:
				mode |= AK4376_CM_2;
				break;
			default:
				return -EINVAL;
			}
		}
		else {					//Low Power Mode (LPMODE == DSMLP == 1)
			switch (mcki_rate) {
			case 32:
				mode |= AK4376_CM_0;
				break;
			case 64:
				mode |= AK4376_CM_1;
				break;
			case 128:
				mode |= AK4376_CM_3;
				break;
			case 256:
				mode |= AK4376_CM_0;
				break;
			case 512:
				mode |= AK4376_CM_1;
				break;
			case 1024:
				mode |= AK4376_CM_2;
				break;
			default:
				return -EINVAL;
			}
		}

		snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);
	}

	return 0;
}

static int ak4376_set_pllblock(struct snd_soc_codec *codec, int fs)
{
	u8 mode;
	int nMClk, nPLLClk, nRefClk;
	int PLDbit, PLMbit, MDIVbit;
	int PLLMCKI;
	int PLLXTAL;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	mode = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
	mode &= ~AK4376_CM;

		if ( fs <= 24000 ) {
			mode |= AK4376_CM_1;
			nMClk = 512 * fs;
		}
		else if ( fs <= 96000 ) {
			mode |= AK4376_CM_0;
			nMClk = 256 * fs;
		}
		else if ( fs <= 192000 ) {
			mode |= AK4376_CM_3;
			nMClk = 128 * fs;
		}
		else {		//fs > 192kHz
			mode |= AK4376_CM_1;
			nMClk = 64 * fs;
		}

	snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);

	if ( (fs % 8000) == 0 ) {
		nPLLClk = 122880000;
	}
	else if ( (fs == 11025 ) && ( ak4376->nBickFreq == 1 ) && ( ak4376->nPllMode == 1 )) {	//3.18
		nPLLClk = 101606400;
	}
	else {
		nPLLClk = 112896000;
	}

	if ( ak4376->nPllMode == 1 ) {		//BICK_PLL (Slave)
		if ( ak4376->nBickFreq == 0 ) {		//32fs
			if ( fs <= 96000 ) PLDbit = 1;
			else if ( fs <= 192000 ) PLDbit = 2;
			else PLDbit = 4;
			nRefClk = 32 * fs / PLDbit;
		}
		else if ( ak4376->nBickFreq == 1 ) {	//48fs
			if ( fs <= 16000 ) PLDbit = 1;
			else if ( fs <= 192000 ) PLDbit = 3;
			else PLDbit = 6;
			nRefClk = 48 * fs / PLDbit;
		}
		else {  									// 64fs
			if ( fs <= 48000 ) PLDbit = 1;
			else if ( fs <= 96000 ) PLDbit = 2;
			else if ( fs <= 192000 ) PLDbit = 4;
			else PLDbit = 8;
			nRefClk = 64 * fs / PLDbit;
		}
	}
	else if ( ak4376->nPllMode == 2 ) {		//MCKI_PLL (Master)
		if ( ak4376->nPllMCKI == 0 ) { //9.6MHz
			PLLMCKI = 9600000;
			if ( (fs % 4000) == 0) nRefClk = 1920000;
			else nRefClk = 384000;
		}
		else if ( ak4376->nPllMCKI == 1 ) { //11.2896MHz
			PLLMCKI = 11289600;
			if ( (fs % 4000) == 0) return -EINVAL;
			else nRefClk = 2822400;
		}
		else if ( ak4376->nPllMCKI == 2 ) { //12.288MHz
			PLLMCKI = 12288000;
			if ( (fs % 4000) == 0) nRefClk = 3072000;
			else nRefClk = 768000;
		}
		else {								//19.2MHz
			PLLMCKI = 19200000;
			if ( (fs % 4000) == 0) nRefClk = 1920000;
			else nRefClk = 384000;
		}
		PLDbit = PLLMCKI / nRefClk;
	}
	//0:12.288MHz, 1:11.2896MHz, 2:19.2MHZ
	else if ( ak4376->nPllMode == 4 ) {		//XTAL_PLL (Master)
		if ( ak4376->xtalfreq == 0 ) { //12.288MHz
			PLLXTAL = 12288000;
			if ( (fs % 4000) == 0) nRefClk = 3072000;
			else nRefClk = 768000;
		}
		else if ( ak4376->xtalfreq == 1 ) { //11.2896MHz
			PLLXTAL = 11289600;
			if ( (fs % 4000) == 0) return -EINVAL;
			else nRefClk = 2822400;
		}
		else if ( ak4376->xtalfreq == 2 ) { //19.2MHz
			PLLXTAL = 19200000;
			if ( (fs % 4000) == 0) nRefClk = 1920000;
			else nRefClk = 384000;
		}
		PLDbit = PLLXTAL / nRefClk;
	}

	PLMbit = nPLLClk / nRefClk;
	MDIVbit = nPLLClk / nMClk;

	PLDbit--;
	PLMbit--;
	MDIVbit--;

	//PLD15-0
	snd_soc_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
	//PLM15-0
	snd_soc_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));

//	if (ak4376_data->pdata->nPllMode == 1 ) {	//BICK_PLL (Slave)
	if (ak4376->nPllMode == 0 ) 	 {  //MCKI PLL (Slave/Master)
		snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x00);
	}
	else if (ak4376->nPllMode == 1 ) {	//BICK_PLL (Slave)
		snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x01);	//PLS=1(BICK)
	}
	else if (ak4376->nPllMode == 4) {	//XTAL_PLL (Master)
		snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x02);	//PLS=2(XTAL)
	}

	//MDIV7-0
	snd_soc_write(codec, AK4376_14_DAC_CLK_DIVIDER, MDIVbit);

	return 0;
}

static int ak4376_set_timer(struct snd_soc_codec *codec)
{
	int ret, curdata;
	int count, tm, nfs;
	int lvdtm, vddtm, hptm;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	lvdtm = 0;
	vddtm = 0;
	hptm = 0;

	nfs = ak4376->fs1;

	//LVDTM2-0 bits set
	ret = snd_soc_read(codec, AK4376_03_POWER_MANAGEMENT4);
	curdata = (ret & 0x70) >> 4;	//Current data Save
	ret &= ~0x70;
	do {
       count = 1000 * (64 << lvdtm);
       tm = count / nfs;
       if ( tm > LVDTM_HOLD_TIME ) break;
       lvdtm++;
    } while ( lvdtm < 7 );			//LVDTM2-0 = 0~7
	if ( curdata != lvdtm) {
			snd_soc_write(codec, AK4376_03_POWER_MANAGEMENT4, (ret | (lvdtm << 4)));
	}

	//VDDTM3-0 bits set
	ret = snd_soc_read(codec, AK4376_04_OUTPUT_MODE_SETTING);
	curdata = (ret & 0x3C) >> 2;	//Current data Save
	ret &= ~0x3C;
	do {
       count = 1000 * (1024 << vddtm);
       tm = count / nfs;
       if ( tm > VDDTM_HOLD_TIME ) break;
       vddtm++;
    } while ( vddtm < 8 );			//VDDTM3-0 = 0~8
	if ( curdata != vddtm) {
			snd_soc_write(codec, AK4376_04_OUTPUT_MODE_SETTING, (ret | (vddtm<<2)));
	}

	return 0;
}

static int ak4376_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
	u8 fs;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	fs = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
	fs &= ~AK4376_FS;

	switch (nfs1) {
	case 8000:
		fs |= AK4376_FS_8KHZ;
		break;
	case 11025:
		fs |= AK4376_FS_11_025KHZ;
		break;
	case 16000:
		fs |= AK4376_FS_16KHZ;
		break;
	case 22050:
		fs |= AK4376_FS_22_05KHZ;
		break;
	case 32000:
		fs |= AK4376_FS_32KHZ;
		break;
	case 44100:
		fs |= AK4376_FS_44_1KHZ;
		break;
	case 48000:
		fs |= AK4376_FS_48KHZ;
		break;
	case 88200:
		fs |= AK4376_FS_88_2KHZ;
		break;
	case 96000:
		fs |= AK4376_FS_96KHZ;
		break;
	case 176400:
		fs |= AK4376_FS_176_4KHZ;
		break;
	case 192000:
		fs |= AK4376_FS_192KHZ;
		break;
	case 352800:
		fs |= AK4376_FS_352_8KHZ;
		break;
	case 384000:
		fs |= AK4376_FS_384KHZ;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, fs);

	if ( ak4376->nPllMode == 0 ) {		//PLL Off
		snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x00);	//DACCKS=0
		ak4376_set_mcki(codec, nfs1, ak4376->rclk);
	}
	else if ( ak4376->nPllMode == 3 ) {	//XTAL MODE
		snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x02);	//DACCKS=2
		ak4376_set_mcki(codec, nfs1, ak4376->rclk);
	}
	else {								//PLL mode , XTAL_PLL mode
		snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x01);	//DACCKS=1
		ak4376_set_pllblock(codec, nfs1);
	}

	ak4376_set_timer(codec);

	return 0;
}

static int ak4376_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
	int bitnum;

	ak4376->fs1 = params_rate(params);
	ak4376->format = params_format(params);
	ak4376->ch = params_channels(params);
	if(ak4376->format== SNDRV_PCM_FORMAT_S16_LE)
	   bitnum = 16;
	else
	   bitnum = 32;
#if 0 //Test
	ak4376->nBickFreq = (bitnum==16)?0: 2;
	ak4376_set_bickfs(codec);
	if (ak4376->ch*bitnum ==32)
        snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x8, 0x8);
	else
        snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x8, 0x0);
#endif
	dev_info(codec->dev,"%s dai->name=%s, fs =%d,format=%d,ch=%d\n", __FUNCTION__, dai->name,ak4376->fs1,ak4376->format,ak4376->ch);
	ak4376_hw_params_set(codec, ak4376->fs1);

	return 0;
}

static int ak4376_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s freq=%dHz(%d)\n",__FUNCTION__,freq,__LINE__);

	ak4376_pdn_control(codec, 1);

	ak4376->rclk = freq;

	if ((ak4376->nPllMode == 0) || (ak4376->nPllMode == 3)) {	//Not PLL mode
		ak4376_set_mcki(codec, ak4376->fs1, freq);
	}

	return 0;
}

static int ak4376_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
	u8 format;

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ak4376_pdn_control(codec, 1);

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);
	format &= ~AK4376_DIF;

    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBS_CFS:
    		format |= AK4376_SLAVE_MODE;
            break;
        case SND_SOC_DAIFMT_CBM_CFM:
	    	if (ak4376->nDeviceID == 2) {
    		format |= AK4376_MASTER_MODE;
	    	}
	    	else return -EINVAL;
	    	break;
        case SND_SOC_DAIFMT_CBS_CFM:
        case SND_SOC_DAIFMT_CBM_CFS:
        default:
            dev_err(codec->dev, "Clock mode unsupported");
           return -EINVAL;
    	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= AK4376_DIF_I2S_MODE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= AK4376_DIF_MSB_MODE;
		break;
	default:
		format |= AK4376_DIF_I2S_MODE;
		break;
	}

	/* set format */
	snd_soc_write(codec, AK4376_15_AUDIO_IF_FORMAT, format);

	return 0;
}

static bool ak4376_volatile(struct device *dev, unsigned int reg)
{
	int	ret;

	switch (reg) {
		default:
			ret = 0;
			break;
	}
	return ret;
}

static bool ak4376_writeable(struct device *dev, unsigned int reg)
{
	bool ret;

	if (reg < AK4376_MAX_REGISTERS ) {
		ret = 1;
	}
	else ret = 0;

	switch(reg){
		case AK4376_21_DUMMY:
			ret = 0;
			break;
	}

	return ret;
}

unsigned int ak4376_i2c_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
	int ret = -1;
	unsigned char tx[1], rx[1];

	struct i2c_msg xfer[2];
	struct i2c_client *client = ak4376->i2c;

	tx[0] = reg;
	rx[0] = 0;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = tx;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = rx;

	ret = i2c_transfer(client->adapter, xfer, 2);

	if( ret != 2 ){
		dev_dbg(codec->dev,"%s error ret = %d \n", __FUNCTION__, ret );
	}

	return (unsigned int)rx[0];

}

unsigned int ak4376_reg_read(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned char tx[1];
	int	wlen, rlen;
	int ret = 0;
	unsigned int rdata = 0;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	if(!ak4376->connected)
		return -ENODEV;

	if (ak4376->pdn1 == 0) {
#ifdef AK4376_CONTIF_DEBUG
		if(ak4376->regmap && !ak4376->regmap->cache_bypass){
			ak4376->regmap->cache_only= true;
			regmap_read(ak4376->regmap, reg, &rdata);
			ak4376->regmap->cache_only= false;
		}
		else{
			ret = snd_soc_cache_read(codec, reg, &rdata);
			if (ret < 0) {
				dev_dbg(codec->dev,"%s Read cache error\n",__FUNCTION__);
			}
		}
#else
		rdata = snd_soc_read(codec, reg);
#endif
		dev_dbg(codec->dev,"%s Read cache\n",__FUNCTION__);
	} else if ((ak4376->pdn1 == 1) || (ak4376->pdn1 == 2)) {
		wlen = 1;
		rlen = 1;
		tx[0] = reg;

		ret = ak4376_i2c_read(codec, reg);

		dev_dbg(codec->dev,"%s Read IC register, reg=0x%x val=0x%x\n",__FUNCTION__,reg,ret);

		if (ret < 0) {
			dev_dbg(codec->dev,"%s error ret = %d\n",__FUNCTION__,ret);
			rdata = -EIO;
		}
		else {
			rdata = ret;
		}
	}

	return rdata;
}

static int ak4376_write_register(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	size_t	wlen;
	int rc = 0;
	unsigned char tx[2];
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	wlen = 2;
	tx[0] = reg;
	tx[1] = value;
	if(!ak4376->connected)
		return -ENODEV;

	if ((ak4376->pdn1 == 1) || (ak4376->pdn1 == 2)) {
		dev_dbg(codec->dev,"%s(%d) (0x%x,0x%x)\n",__FUNCTION__,__LINE__,reg,value);
		rc = i2c_master_send(ak4376->i2c, tx, wlen);
	}

	if( rc != wlen ){
		return -EIO;
	}

	return rc;
}


#ifdef CONFIG_DEBUG_FS_CODEC
static int ak4376_reg_write(
struct snd_soc_codec *codec,
u16 reg,
u16 value)
{
	dev_dbg(codec->dev,"%s(%d) (%x,%x)\n",__FUNCTION__,__LINE__,reg,value);

	snd_soc_write(codec, (unsigned int)reg, (unsigned int)value);

    return 0;
}
#endif

// * for AK4376
static int ak4376_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
	int ret = 0;

	dev_dbg(codec_dai->codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	return ret;
}


static int ak4376_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	int level1 = level;

	dev_dbg(codec->dev,"%s(%d) level=%d\n",__FUNCTION__,__LINE__,level1);
	dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level=%d\n",__FUNCTION__,__LINE__,codec->dapm.bias_level);

	switch (level1) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:

		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level == SND_SOC_BIAS_STANDBY\n",__FUNCTION__,__LINE__);

		if (codec->dapm.bias_level == SND_SOC_BIAS_ON)
			dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level >= SND_SOC_BIAS_ON\n",__FUNCTION__,__LINE__);
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_PREPARE) {
			dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level == SND_SOC_BIAS_PREPARE\n",__FUNCTION__,__LINE__);
			//ak4376_pdn_control(codec, 0);
		} if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
			dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level == SND_SOC_BIAS_OFF\n",__FUNCTION__,__LINE__);
		break;
	case SND_SOC_BIAS_OFF:
// /* 3.18
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level == SND_SOC_BIAS_STANDBY\n",__FUNCTION__,__LINE__);
			ak4376_pdn_control(codec, 0);
		}
// */
		break;
	}
	codec->dapm.bias_level = level;
	dev_dbg(codec->dev,"%s(%d) end\n",__FUNCTION__,__LINE__);
	return 0;
}

static int ak4376_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
	u8 MSmode;
	struct snd_soc_codec *codec = dai->codec;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level=%d\n",__FUNCTION__,__LINE__,codec->dapm.bias_level);

	if (ak4376->nPllMode == 0) {
		if ( ak4376->nDACOn == 0 ) {
			MSmode = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);
			if (MSmode & 0x10) {	//Master mode
				snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10,0x00);	//MS bit = 0
			}
		}
	}

	if (codec->dapm.bias_level <= SND_SOC_BIAS_STANDBY) {
		dev_dbg(codec->dev,"%s(%d) codec->dapm.bias_level <= SND_SOC_BIAS_STANDBY\n",__FUNCTION__,__LINE__);

		ak4376_pdn_control(codec, 0);
	}

	return 0;
}

#define AK4376_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
				SNDRV_PCM_RATE_192000)

#define AK4376_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


static struct snd_soc_dai_ops ak4376_dai_ops = {
	.hw_params	= ak4376_hw_params,
	.set_sysclk	= ak4376_set_dai_sysclk,
	.set_fmt	= ak4376_set_dai_fmt,
	.trigger = ak4376_trigger,
	.digital_mute = ak4376_set_dai_mute,
};

struct snd_soc_dai_driver ak4376_dai[] = {
	{
		.name = "ak4376-AIF1",
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = AK4376_RATES,
		       .formats = AK4376_FORMATS,
		},
		.ops = &ak4376_dai_ops,
	},
};

static int ak4376_init_reg(struct snd_soc_codec *codec)
{
	u8 DeviceID;
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	if(!ak4376->connected)
		return -ENODEV;

	gpio_direction_output(ak4376->priv_pdn_en, 1);
	udelay(800);


	//DeviceID = ak4376_reg_read(codec, AK4376_15_AUDIO_IF_FORMAT);
	DeviceID = ak4376_i2c_read(codec, AK4376_15_AUDIO_IF_FORMAT);

	switch (DeviceID >> 5) {
	case 0:
		ak4376->nDeviceID = 0;		//0:AK4375
		//printk("AK4375 is connecting.\n");
		break;
	case 1:
		ak4376->nDeviceID = 1;		//1:AK4375A
		//printk("AK4375A is connecting.\n");
		break;
	case 2:
		ak4376->nDeviceID = 2;		//2:AK4376
		printk("AK4376 is connecting.\n");
		break;
	default:
		ak4376->nDeviceID = 3;		//3:Other IC
		//printk("This device are neither AK4375/A nor AK4376.\n");
	}
#ifdef AK4376_DETECT_IC
	if(ak4376->nDeviceID != 2){
		printk(KERN_INFO "Invalid Codc : Not ak4376 Codec [%d]\n",ak4376->nDeviceID);
		ak4376->connected = 0;
		return 0;
	}
#endif

	ak4376_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	dev_dbg(codec->dev,"%s(%d)[Bias]\n",__FUNCTION__,__LINE__);

	gpio_direction_output(ak4376->priv_pdn_en, 0);

	return 0;
}

static int ak4376_parse_dt(struct ak4376_priv *ak4376)
{
	struct device *dev;
	struct device_node *np;

	dev = &(ak4376->i2c->dev);

	np = dev->of_node;

	if (!np)
		return -1;

	printk("Read VDD Control pin from device tree\n");

	ak4376->ldo_sw_en = of_get_named_gpio(np, "ak4376,ldo-gpios", 0);
	if (ak4376->ldo_sw_en < 0) {
		ak4376->ldo_sw_en = -1;
	}

	if(!gpio_is_valid(ak4376->ldo_sw_en) ) {
		printk(KERN_ERR "ak4376 VDD pin(%u) is invalid\n", ak4376->ldo_sw_en);
	}

	printk("Read PDN pin from device tree\n");

	ak4376->priv_pdn_en = of_get_named_gpio(np, "ak4376,pdn-gpio", 0);
	if (ak4376->priv_pdn_en < 0) {
		ak4376->priv_pdn_en = -1;
		return -1;
	}

	if( !gpio_is_valid(ak4376->priv_pdn_en) ) {
		printk(KERN_ERR "ak4376 pdn pin(%u) is invalid\n", ak4376->priv_pdn_en);
		return -1;
	}

	if(!of_property_read_u32(dev->of_node, "ak4376,pll-mode",	&ak4376->nPllMode)){
		printk(KERN_INFO "ak4376 pll-mode is %s[%u]\n", pllmode_on_select[ak4376->nPllMode%ARRAY_SIZE(pllmode_on_select)],ak4376->nPllMode);
	}

	if(!of_property_read_u32(dev->of_node, "ak4376,xtalfreq",	&ak4376->xtalfreq)){
		printk(KERN_INFO "ak4376 xtalfreq is %s[%u]\n", xtalfreq_on_select[ak4376->xtalfreq%ARRAY_SIZE(xtalfreq_on_select)],ak4376->xtalfreq);
	}

	return 0;
}

static int ak4376_probe(struct snd_soc_codec *codec)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ret = ak4376_parse_dt(ak4376);
	dev_info(codec->dev,"%s(%d) ret=%d\n",__FUNCTION__,__LINE__,ret);
	if ( ret < 0 ) ak4376->pdn1 = 2;	//No use GPIO control

	if(gpio_is_valid(ak4376->ldo_sw_en)){
		ret = gpio_request(ak4376->ldo_sw_en, "ak4376 ldo_sw_en");
		if(ret){
			printk(KERN_ERR "ak4376 cannot get ak4376 ldo gpio(%d)\n", ak4376->ldo_sw_en);
		}else{
			gpio_direction_output(ak4376->ldo_sw_en, 1);
		}
	}
	ak4376->connected =1;
	ret = gpio_request(ak4376->priv_pdn_en, "ak4376 pdn");
	dev_dbg(codec->dev,"%s : gpio_request ret = %d\n",__FUNCTION__, ret);
	if (ret) {
		dev_err(codec->dev,"%s(%d) cannot get ak4376 pdn gpio\n",__FUNCTION__,__LINE__);
		ak4376->pdn1 = 2;	//No use GPIO control
	}
#ifdef AK4376_DETECT_IC
	if(!ak4376_hifi_support){
		printk(KERN_INFO "Not Support ak4376 Codec\n");
		ak4376->connected = 0;
	}
#endif

	ret = gpio_direction_output(ak4376->priv_pdn_en, 0);
	if (ret) {
		dev_err(codec->dev,"%s(%d) pdn_en=0 fail\n", __FUNCTION__,__LINE__);
		gpio_free(ak4376->priv_pdn_en);
		ak4376->pdn1 = 2;
	} else {
		dev_dbg(codec->dev,"%s(%d) pdn_en=0\n", __FUNCTION__,__LINE__);
	}



#ifdef CONFIG_DEBUG_FS_CODEC
	mutex_init(&io_lock);
	ak4376_codec = codec;
#endif

	ak4376_init_reg(codec);

	dev_dbg(codec->dev,"%s(%d)[Effect]\n",__FUNCTION__,__LINE__);

	ak4376->fs1 = 48000;
	ak4376->rclk = 0;
	ak4376->nBickFreq = 1;		//0:32fs, 1:48fs, 2:64fs
	ak4376->nPllMCKI = 0;		//0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
	ak4376->lpmode = 0;			//0:High Performance mode, 1:Low Power Mode
//	ak4376->xtalfreq = 2;		//0:12.288MHz, 1:11.2896MHz, 2:19.2MHz
	ak4376->nDACOn = 0;

#ifndef USE_PLL_MODE_DTS
 #if defined(PLL_BICK_MODE)	//at ak4376_pdata.h
	ak4376->nPllMode = 1;
 #elif defined(PLL_MCKI_MODE)
	ak4376->nPllMode = 2;
 #elif defined(XTAL_MODE)
	ak4376->nPllMode = 3;
 #elif defined(PLL_XTAL_MODE)
	ak4376->nPllMode = 4;
	ak4376->xtalfreq = 2;		//0:12.288MHz, 1:11.2896MHz, 2:19.2MHz
 #else
	ak4376->nPllMode = 0;
 #endif
#endif

	return ret;
}

static int ak4376_remove(struct snd_soc_codec *codec)
{

	dev_dbg(codec->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ak4376_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4376 = {
	.probe = ak4376_probe,
	.remove = ak4376_remove,
#ifdef AK4376_CONTIF_DEBUG
	.write = ak4376_write_register,
	.read = ak4376_reg_read,
	.reg_cache_size = ARRAY_SIZE(ak4376_reg_default),
	.reg_word_size = 1,
	.reg_cache_default = ak4376_reg_default,
#endif

	.controls = ak4376_snd_controls,
	.num_controls = ARRAY_SIZE(ak4376_snd_controls),

	.idle_bias_off = false,
	.set_bias_level = ak4376_set_bias_level,

	.dapm_widgets = ak4376_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4376_dapm_widgets),
	.dapm_routes = ak4376_intercon,
	.num_dapm_routes = ARRAY_SIZE(ak4376_intercon),
};

EXPORT_SYMBOL_GPL(soc_codec_dev_ak4376);

static const struct regmap_config ak4376_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4376_MAX_REGISTERS,
	.volatile_reg = ak4376_volatile,
	.writeable_reg = ak4376_writeable,

	.reg_defaults = ak4376_reg,
	.num_reg_defaults = ARRAY_SIZE(ak4376_reg),

#ifdef AK4376_CONTIF_DEBUG
	.cache_type = REGCACHE_FLAT,
#else
	.cache_type = REGCACHE_RBTREE,
#endif
};

static struct of_device_id ak4376_i2c_dt_ids[] = {
	{ .compatible = "akm,ak4376"},
    { }
};
MODULE_DEVICE_TABLE(of, ak4376_i2c_dt_ids);

static int ak4376_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ak4376_priv *ak4376;
	int ret=0;

	dev_dbg(&i2c->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

	ak4376 = kzalloc(sizeof(struct ak4376_priv), GFP_KERNEL);
	if (ak4376 == NULL) return -ENOMEM;

	ak4376->regmap = devm_regmap_init_i2c(i2c, &ak4376_regmap);
	if (IS_ERR(ak4376->regmap)) {
		dev_err(&i2c->dev,"%s regmap error\n",__FUNCTION__);
		return PTR_ERR(ak4376->regmap);
	}

#ifdef  CONFIG_DEBUG_FS_CODEC
	ret = device_create_file(&i2c->dev, &dev_attr_reg_data);
	if (ret) {
		pr_err("%s: Error to create reg_data\n", __FUNCTION__);
	}
#endif

	i2c_set_clientdata(i2c, ak4376);

	ak4376->i2c = i2c;
	ak4376->pdn1 = 0;
	ak4376->pdn2 = 0;
	ak4376->ldo_sw_en = 0;
	ak4376->priv_pdn_en = 0;

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4376, &ak4376_dai[0], ARRAY_SIZE(ak4376_dai));
	if (ret < 0){
		kfree(ak4376);
		dev_err(&i2c->dev,"%s(%d)-register codec failed\n",__FUNCTION__,__LINE__);
	}

	dev_dbg(&i2c->dev,"%s(%d) pdn1=%d\n, ret=%d", __FUNCTION__,__LINE__,ak4376->pdn1,ret);

	return ret;
}

static int ak4376_i2c_remove(struct i2c_client *client)
{

	dev_dbg(&client->dev,"%s(%d)\n",__FUNCTION__,__LINE__);

#ifdef CONFIG_DEBUG_FS_CODEC
	device_remove_file(&client->dev, &dev_attr_reg_data);
#endif

	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id ak4376_i2c_id[] = {
	{ "ak4376", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

static struct i2c_driver ak4376_i2c_driver = {
	.driver = {
		.name = "ak4376",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ak4376_i2c_dt_ids),	//3.18
	},
	.probe = ak4376_i2c_probe,
	.remove = ak4376_i2c_remove,
	.id_table = ak4376_i2c_id,
};

static int __init ak4376_modinit(void)
{
	return i2c_add_driver(&ak4376_i2c_driver);
}

module_init(ak4376_modinit);

static void __exit ak4376_exit(void)
{
	i2c_del_driver(&ak4376_i2c_driver);
}
module_exit(ak4376_exit);

MODULE_DESCRIPTION("ASoC ak4376 codec driver");
MODULE_LICENSE("GPL");
