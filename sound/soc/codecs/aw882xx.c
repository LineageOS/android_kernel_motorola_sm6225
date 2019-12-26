/*
 * aw882xx.c   aw882xx codec module
 *
 * Version: v0.1.7
 *
 * keep same with AW882XX_VERSION
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef CONFIG_AW882XX_CODEC

#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/syscalls.h>
#include <sound/tlv.h>
#include <linux/uaccess.h>
#include "aw882xx.h"
#include "aw882xx_reg.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW882XX_I2C_NAME "aw882xx_smartpa"

#define AW882XX_VERSION "v0.1.7"

#define AW882XX_RATES SNDRV_PCM_RATE_8000_48000
#define AW882XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
						SNDRV_PCM_FMTBIT_S24_LE | \
						SNDRV_PCM_FMTBIT_S32_LE)


#define AW_I2C_RETRIES				5	/* 5 times */
#define AW_I2C_RETRY_DELAY			5	/* 5 ms */
#define AW_READ_CHIPID_RETRIES		5	/* 5 times */
#define AW_READ_CHIPID_RETRY_DELAY	5	/* 5 ms */


#define CALI_BUF_MAX 100
#define AWINIC_CALI_FILE  "/mnt/vendor/persist/factory/audio/aw_cali.bin"

#ifdef CONFIG_AW882XX_DSP
extern int aw_send_afe_cal_apr(uint32_t param_id, void *buf,int cmd_size, bool write);
extern int aw_send_rx_module_enable(void *buf, int cmd_size);
extern int aw_send_tx_module_enable(void *buf, int cmd_size);
extern int aw_adm_param_enable(int port_id, int module_id, int param_id,  int enable);
#else
static int aw_send_afe_cal_apr(uint32_t param_id, void *buf,int cmd_size, bool write) {
    return 0;
}
static int aw_send_rx_module_enable(void *buf, int cmd_size)
{
	return 0;
}
static int aw_send_tx_module_enable(void *buf, int cmd_size)
{
	return 0;
}
static int aw_adm_param_enable(int port_id, int copp_idx, int module_id, int param_id,  int enable)
{
	return 0;
}
#endif

static int aw882xx_get_cali_re_form_nv(int32_t *cali_re);
static int aw882xx_set_cali_re(struct aw882xx *aw882xx, int32_t cali_re);
static void aw882xx_skt_set_dsp(int value);

/*monitor  voltage and temperature table*/
static struct aw882xx_low_vol vol_down_table[] = {
		{3500, IPEAK_2P50_A, GAIN_NEG_1P5_DB},
		{3700, IPEAK_2P75_A, GAIN_NEG_1P0_DB},
		{3900, IPEAK_3P00_A, GAIN_NEG_0P5_DB},
	};
static struct aw882xx_low_vol vol_up_table[] = {
		{4000, IPEAK_3P50_A, GAIN_NEG_0P0_DB},
		{3800, IPEAK_3P00_A, GAIN_NEG_0P5_DB},
		{3600, IPEAK_2P75_A, GAIN_NEG_1P0_DB},
	};
static struct aw882xx_low_temp temp_down_table[] = {
		{-5, IPEAK_2P50_A, GAIN_NEG_6P0_DB, VMAX_063_PERCENTAGE},
		{ 0, IPEAK_2P75_A, GAIN_NEG_4P5_DB, VMAX_075_PERCENTAGE},
		{ 5, IPEAK_3P00_A, GAIN_NEG_3P0_DB, VMAX_086_PERCENTAGE},
	};
static struct aw882xx_low_temp temp_up_table[] = {
		{ 7, IPEAK_3P50_A, GAIN_NEG_0P0_DB, VMAX_100_PERCENTAGE},
		{ 2, IPEAK_3P00_A, GAIN_NEG_3P0_DB, VMAX_086_PERCENTAGE},
		{-2, IPEAK_2P75_A, GAIN_NEG_4P5_DB, VMAX_075_PERCENTAGE},
	};


static int aw882xx_monitor_start(struct aw882xx_monitor *monitor);
static int aw882xx_monitor_stop(struct aw882xx_monitor *monitor);

/******************************************************
 *
 * Value
 *
 ******************************************************/
static int aw882xx_spk_control;
static int aw882xx_rcv_control;

static atomic_t g_algo_rx_enable;
static atomic_t g_algo_tx_enable;
static atomic_t g_skt_disable;
static struct aw882xx *g_aw882xx = NULL;
static int8_t g_aw882xx_cali_flag = 0;
#define AW882XX_CFG_NAME_MAX		64
static char aw882xx_cfg_name[][AW882XX_CFG_NAME_MAX] = {
	{"aw882xx_spk_reg.bin"},
	{"aw882xx_rcv_reg.bin"},
};

static unsigned int aw882xx_mode_cfg_shift[AW882XX_MODE_SHIFT_MAX] = {
	AW882XX_MODE_SPK_SHIFT,
	AW882XX_MODE_RCV_SHIFT,
};

/******************************************************
 *
 * aw882xx i2c write/read
 *
 ******************************************************/
static int aw882xx_i2c_writes(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len+1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw882xx->i2c, data, len+1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(data);

	return ret;
}

static int aw882xx_i2c_reads(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;

	ret = i2c_smbus_write_byte(aw882xx->i2c, reg_addr);
	if (ret < 0) {
		pr_err("%s: i2c master send error, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = i2c_master_recv(aw882xx->i2c, buf, len);
	if (ret != len) {
		pr_err("%s: couldn't read registers, return %d bytes\n",
			__func__, ret);
		return ret;
	}

	return ret;
}

static int aw882xx_i2c_write(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char buf[2];

	buf[0] = (reg_data&0xff00)>>8;
	buf[1] = (reg_data&0x00ff)>>0;

	while (cnt < AW_I2C_RETRIES) {
		ret = aw882xx_i2c_writes(aw882xx, reg_addr, buf, 2);
		if (ret < 0)
			pr_err("%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
		else
			break;
		cnt++;
	}

	return ret;
}

static int aw882xx_i2c_read(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char buf[2];

	while (cnt < AW_I2C_RETRIES) {
		ret = aw882xx_i2c_reads(aw882xx, reg_addr, buf, 2);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			*reg_data = (buf[0]<<8) | (buf[1]<<0);
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw882xx_i2c_write_bits(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
	int ret = -1;
	unsigned int reg_val = 0;

	ret = aw882xx_i2c_read(aw882xx, reg_addr, &reg_val);
	if (ret < 0) {
		pr_err("%s: i2c read error, ret=%d\n", __func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw882xx_i2c_write(aw882xx, reg_addr, reg_val);
	if (ret < 0) {
		pr_err("%s: i2c read error, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/******************************************************
 *
 * aw882xx control
 *
 ******************************************************/
static void aw882xx_run_mute(struct aw882xx *aw882xx, bool mute)
{
	pr_debug("%s: enter\n", __func__);

	if (mute) {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_SYSCTRL2_REG,
				AW882XX_HMUTE_MASK,
				AW882XX_HMUTE_ENABLE_VALUE);
	} else {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_SYSCTRL2_REG,
				AW882XX_HMUTE_MASK,
				AW882XX_HMUTE_DISABLE_VALUE);
	}
}

static void aw882xx_run_pwd(struct aw882xx *aw882xx, bool pwd)
{
	pr_debug("%s: enter\n", __func__);

	if (pwd) {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_SYSCTRL_REG,
				AW882XX_PWDN_MASK,
				AW882XX_PWDN_POWER_DOWN_VALUE);
	} else {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_SYSCTRL_REG,
				AW882XX_PWDN_MASK,
				AW882XX_PWDN_NORMAL_WORKING_VALUE);
	}
}

static int aw882xx_sysst_check(struct aw882xx *aw882xx)
{
	int ret = -1;
	unsigned char i;
	unsigned int reg_val = 0;

	for (i = 0; i < AW882XX_SYSST_CHECK_MAX; i++) {
		aw882xx_i2c_read(aw882xx, AW882XX_SYSST_REG, &reg_val);
		if ((reg_val & (~AW882XX_SYSST_CHECK_MASK)) ==
			AW882XX_SYSST_CHECK) {
			ret = 0;
			break;
		} else {
			pr_debug("%s: check fail, cnt=%d, reg_val=0x%04x\n",
				__func__, i, reg_val);
			msleep(2);
		}
	}
	if (ret < 0)
		pr_info("%s: check fail\n", __func__);

	return ret;
}

/*
static int aw882xx_get_sysint(struct aw882xx *aw882xx, unsigned int *sysint)
{
	int ret = -1;
	unsigned int reg_val = 0;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_SYSINT_REG, &reg_val);
	if (ret < 0)
		pr_info("%s: read sysint fail, ret=%d\n", __func__, ret);
	else
		*sysint = reg_val;

	return ret;
}

static int aw882xx_get_iis_status(struct aw882xx *aw882xx)
{
	int ret = -1;
	unsigned int reg_val = 0;

	pr_debug("%s: enter\n", __func__);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSST_REG, &reg_val);
	if (reg_val & AW882XX_PLLS_LOCKED_VALUE)
		ret = 0;

	return ret;
}
*/

static int aw882xx_get_icalk(struct aw882xx *aw882xx, int16_t *icalk)
{
	int ret = -1;
	unsigned int reg_val = 0;
	uint16_t reg_icalk = 0;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_EFRM1_REG, &reg_val);
	reg_icalk = (uint16_t)reg_val & AW882XX_EF_ISN_GESLP_MASK;

	if (reg_icalk & AW882XX_EF_ISN_GESLP_SIGN_MASK)
		reg_icalk = reg_icalk | AW882XX_EF_ISN_GESLP_NEG;

	*icalk = (int16_t)reg_icalk;

	return ret;
}

static int aw882xx_get_vcalk(struct aw882xx *aw882xx, int16_t *vcalk)
{
	int ret = -1;
	unsigned int reg_val = 0;
	uint16_t reg_vcalk = 0;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_EFRH_REG, &reg_val);

	reg_vcalk = (uint16_t)reg_val & AW882XX_EF_VSN_GESLP_MASK;

	if (reg_vcalk & AW882XX_EF_VSN_GESLP_SIGN_MASK)
		reg_vcalk = reg_vcalk | AW882XX_EF_VSN_GESLP_NEG;

	*vcalk = (int16_t)reg_vcalk;

	return ret;
}

static int aw882xx_set_vcalb(struct aw882xx *aw882xx)
{
	int ret = -1;
	unsigned int reg_val;
	int vcalb;
	int icalk;
	int vcalk;
	int16_t icalk_val = 0;
	int16_t vcalk_val = 0;

	ret = aw882xx_get_icalk(aw882xx, &icalk_val);
	ret = aw882xx_get_vcalk(aw882xx, &vcalk_val);

	icalk = AW882XX_CABL_BASE_VALUE + AW882XX_ICABLK_FACTOR * icalk_val;
	vcalk = AW882XX_CABL_BASE_VALUE + AW882XX_VCABLK_FACTOR * vcalk_val;

	vcalb = AW882XX_VCAL_FACTOR * icalk / vcalk;

	reg_val = (unsigned int)vcalb;
	pr_debug("%s: icalk=%d, vcalk=%d, vcalb=%d, reg_val=%d\n",
		__func__, icalk, vcalk, vcalb, reg_val);

	ret = aw882xx_i2c_write(aw882xx, AW882XX_VTMCTRL3_REG, reg_val);

	return ret;
}

static void aw882xx_send_cali_re_to_dsp(struct aw882xx *aw882xx)
{
	int ret;
	ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_RE_L,
		&aw882xx->cali_re, sizeof(int32_t), true);
	if (ret)
		pr_err("%s : set cali re to dsp failed 0x%x\n",
			__func__ , AFE_PARAM_ID_AWDSP_RX_RE_L);
}

static void aw882xx_start(struct aw882xx *aw882xx)
{
	int ret = -1;
	int32_t cali_re;
	pr_debug("%s: enter\n", __func__);

	ret = aw882xx_get_cali_re_form_nv(&cali_re);
	if (ret < 0) {
		cali_re = (DEFAULT_CALI_VALUE << 12);
		pr_err("%s: use default vaule %d",
			__func__ , DEFAULT_CALI_VALUE);
	}
	ret = aw882xx_set_cali_re(aw882xx, cali_re);
	if (ret < 0)
		pr_err("%s: set cali re failed: %d\n", __func__, ret);

	mutex_lock(&aw882xx->lock);
	aw882xx_run_pwd(aw882xx, false);
	ret = aw882xx_sysst_check(aw882xx);
	if (ret < 0) {
		aw882xx_run_mute(aw882xx, true);
		aw882xx_run_pwd(aw882xx, true);
		aw882xx->init = AW882XX_INIT_NG;
	} else {
		aw882xx_run_mute(aw882xx, false);
		aw882xx->init = AW882XX_INIT_OK;
	}
	aw882xx_send_cali_re_to_dsp(aw882xx);

	pr_info("%s: monitor is_enable %d,spk_rcv_mode %d\n",
		__func__, aw882xx->monitor.is_enable, aw882xx->spk_rcv_mode);
	if (aw882xx->monitor.is_enable &&
		(aw882xx->spk_rcv_mode == AW882XX_SPEAKER_MODE)) {
		aw882xx_monitor_start(&aw882xx->monitor);
	}
	mutex_unlock(&aw882xx->lock);
}

static void aw882xx_stop(struct aw882xx *aw882xx)
{
	pr_debug("%s: enter\n", __func__);

	mutex_lock(&aw882xx->lock);
	aw882xx_run_mute(aw882xx, true);
	aw882xx_run_pwd(aw882xx, true);
	if (aw882xx->monitor.is_enable)
		aw882xx_monitor_stop(&aw882xx->monitor);
	mutex_unlock(&aw882xx->lock);
}

/******************************************************
 *
 * aw882xx config
 *
 ******************************************************/
static int aw882xx_get_cali_re_form_nv(int32_t *cali_re)
{
	/*custom add, if success return value is 0 , else -1*/
	struct file *fp;
	char buf[CALI_BUF_MAX];
	int32_t read_re;
	loff_t pos = 0;
	mm_segment_t fs;

	memset(buf, 0, CALI_BUF_MAX);
	/*open cali file*/
	fp = filp_open(AWINIC_CALI_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: open %s failed!", __func__, AWINIC_CALI_FILE);
		return -EINVAL;
	}
	/*set fs kernel*/
	fs = get_fs();
	set_fs(get_ds());

	/*read file*/
	vfs_read(fp, buf, CALI_BUF_MAX - 1, &pos);

	/*get cali re value*/
	if (sscanf(buf, "%d", &read_re) != 1) {
		pr_err("%s: file read error", __func__);
		set_fs(fs);
		filp_close(fp, NULL);
		return -EINVAL;
	}
	set_fs(fs);

	/*close file*/
	filp_close(fp, NULL);

	*cali_re = read_re;
	pr_info("%s: %d", __func__, read_re);
	return  0;
}

static int aw882xx_set_cali_re(struct aw882xx *aw882xx, int32_t cali_re)
{
	if (aw882xx == NULL)
		return -EINVAL;
	aw882xx->cali_re = cali_re;
	return 0;
}

static int aw882xx_reg_container_update(struct aw882xx *aw882xx,
	struct aw882xx_container *aw882xx_cont)
{
	int i = 0;
	int reg_addr = 0;
	int reg_val = 0;
	int ret = -1;

	pr_debug("%s: enter\n", __func__);

	for (i = 0; i < aw882xx_cont->len; i += 4) {
		reg_addr = (aw882xx_cont->data[i+1]<<8) +
			aw882xx_cont->data[i+0];
		reg_val = (aw882xx_cont->data[i+3]<<8) +
			aw882xx_cont->data[i+2];
		pr_debug("%s: reg=0x%04x, val = 0x%04x\n",
			__func__, reg_addr, reg_val);
		ret = aw882xx_i2c_write(aw882xx,
			(unsigned char)reg_addr,
			(unsigned int)reg_val);
		if (ret < 0)
			break;
	}

	pr_debug("%s: exit\n", __func__);

	return ret;
}

static void aw882xx_reg_loaded(const struct firmware *cont, void *context)
{
	struct aw882xx *aw882xx = context;
	struct aw882xx_container *aw882xx_cfg;
	int ret = -1;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
			aw882xx_cfg_name[aw882xx->cfg_num]);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__,
		aw882xx_cfg_name[aw882xx->cfg_num], cont ? cont->size : 0);

	aw882xx_cfg = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw882xx_cfg) {
		release_firmware(cont);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
	aw882xx_cfg->len = cont->size;
	memcpy(aw882xx_cfg->data, cont->data, cont->size);
	release_firmware(cont);

	mutex_lock(&aw882xx->lock);
	ret = aw882xx_reg_container_update(aw882xx, aw882xx_cfg);
	if (ret < 0) {
		pr_err("%s: reg update fail\n", __func__);
	} else {
		pr_err("%s: reg update sucess\n", __func__);
		aw882xx_run_mute(aw882xx, true);
		aw882xx_set_vcalb(aw882xx);
	}
	mutex_unlock(&aw882xx->lock);
	kfree(aw882xx_cfg);

	aw882xx_start(aw882xx);
}

static int aw882xx_load_reg(struct aw882xx *aw882xx)
{
	pr_info("%s: enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		aw882xx_cfg_name[aw882xx->cfg_num],
		aw882xx->dev, GFP_KERNEL,
		aw882xx, aw882xx_reg_loaded);
}

static void aw882xx_get_cfg_shift(struct aw882xx *aw882xx)
{
	aw882xx->cfg_num = aw882xx_mode_cfg_shift[aw882xx->spk_rcv_mode];
	pr_debug("%s: cfg_num=%d\n", __func__, aw882xx->cfg_num);
}

static void aw882xx_cold_start(struct aw882xx *aw882xx)
{
	int ret = -1;

	pr_info("%s: enter\n", __func__);

	aw882xx_get_cfg_shift(aw882xx);

	ret = aw882xx_load_reg(aw882xx);
	if (ret < 0)
		pr_err("%s: cfg loading requested failed: %d\n", __func__, ret);

}

static void aw882xx_smartpa_cfg(struct aw882xx *aw882xx, bool flag)
{
	pr_info("%s: flag = %d\n", __func__, flag);

	if (flag == true) {
		if ((aw882xx->init == AW882XX_INIT_ST) ||
			(aw882xx->init == AW882XX_INIT_NG)) {
			pr_info("%s: init = %d\n", __func__, aw882xx->init);
			aw882xx_cold_start(aw882xx);
		} else {
			aw882xx_start(aw882xx);
		}
	} else {
		aw882xx_stop(aw882xx);
	}
}

/******************************************************
 *
 * kcontrol
 *
 ******************************************************/
static const char *const spk_function[] = { "Off", "On" };
static const char *const rcv_function[] = { "Off", "On" };
static const char *const awinic_algo[] = { "Disable", "Enable" };
static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 50, 0);

struct soc_mixer_control aw882xx_mixer = {
	.reg	= AW882XX_HAGCCFG4_REG,
	.shift	= AW882XX_VOL_START_BIT,
	.max	= AW882XX_VOLUME_MAX,
	.min	= AW882XX_VOLUME_MIN,
};

static int aw882xx_volume_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* set kcontrol info */
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mc->max - mc->min;
	return 0;
}

static int aw882xx_volume_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val = 0;
	unsigned int value = 0;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *) kcontrol->private_value;

	aw882xx_i2c_read(aw882xx, AW882XX_HAGCCFG4_REG, &reg_val);
	ucontrol->value.integer.value[0] = (value >> mc->shift) &
		(AW882XX_VOL_MASK);
	return 0;
}

static int aw882xx_volume_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *) kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;
	unsigned int reg_value = 0;

	/* value is right */
	value = ucontrol->value.integer.value[0];
	if (value > (mc->max-mc->min) || value < 0) {
		pr_err("%s: value over range\n", __func__);
		return -ERANGE;
	}

	/* smartpa have clk */
	aw882xx_i2c_read(aw882xx, AW882XX_SYSST_REG, &reg_value);
	if (!(reg_value & AW882XX_PLLS_LOCKED_VALUE)) {
		pr_err("%s: NO I2S CLK ,cat not write reg\n", __func__);
		return 0;
	}

	/* cal real value */
	value = (value << mc->shift) & AW882XX_VOL_MASK;
	aw882xx_i2c_read(aw882xx, AW882XX_HAGCCFG4_REG, &reg_value);
	value = value | (reg_value & 0x00ff);

	/* write value */
	aw882xx_i2c_write(aw882xx, AW882XX_HAGCCFG4_REG, value);

	return 0;
}

static struct snd_kcontrol_new aw882xx_volume = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "aw882xx_rx_volume",
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
		SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.tlv.p = (digital_gain),
	.info = aw882xx_volume_info,
	.get = aw882xx_volume_get,
	.put = aw882xx_volume_put,
	.private_value = (unsigned long)&aw882xx_mixer,
};

static int aw882xx_spk_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: aw882xx_spk_control=%d\n",
		__func__, aw882xx_spk_control);
	ucontrol->value.integer.value[0] = aw882xx_spk_control;
	return 0;
}

static int aw882xx_spk_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	if (ucontrol->value.integer.value[0] == aw882xx_spk_control)
		return 1;

	aw882xx_spk_control = ucontrol->value.integer.value[0];

	if (AW882XX_SPEAKER_MODE != aw882xx->spk_rcv_mode) {
		aw882xx->spk_rcv_mode = AW882XX_SPEAKER_MODE;
		aw882xx->init = AW882XX_INIT_ST;
	}

	return 0;
}

static int aw882xx_rcv_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: aw882xx_rcv_control=%d\n", __func__, aw882xx_rcv_control);
	ucontrol->value.integer.value[0] = aw882xx_rcv_control;
	return 0;
}

static int aw882xx_rcv_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	if (ucontrol->value.integer.value[0] == aw882xx_rcv_control)
		return 1;

	aw882xx_rcv_control = ucontrol->value.integer.value[0];

	if (AW882XX_RECEIVER_MODE != aw882xx->spk_rcv_mode) {
		aw882xx->spk_rcv_mode = AW882XX_RECEIVER_MODE;
		aw882xx->init = AW882XX_INIT_ST;
	}

	return 0;
}

static int aw882xx_algo_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: aw882xx_algo enable=%d\n",
		__func__, atomic_read(&g_algo_rx_enable));

	ucontrol->value.integer.value[0] = atomic_read(&g_algo_rx_enable);

	return 0;
}

static int aw882xx_algo_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;

	pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	ctrl_value = ucontrol->value.integer.value[0];
	ret = aw_send_rx_module_enable(&ctrl_value, sizeof(uint32_t));
	if (ret)
		pr_err("%s: set algo %d failed, ret=%d\n",
			__func__, ctrl_value, ret);
	atomic_set(&g_algo_rx_enable, ctrl_value);
	return 0;
}

static int aw882xx_tx_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: aw882xx_tx_control=%d\n", __func__,
		atomic_read(&g_algo_tx_enable));

	ucontrol->value.integer.value[0] = atomic_read(&g_algo_tx_enable);

	return 0;
}

static int aw882xx_tx_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	uint32_t ctrl_value = 0;

	pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n",
		__func__, ucontrol->value.integer.value[0]);

	ctrl_value = ucontrol->value.integer.value[0];
	ret = aw_send_tx_module_enable(&ctrl_value, sizeof(uint32_t));
	if (ret)
		pr_err("%s: set tx enable %d, ret=%d\n", __func__, ctrl_value, ret);
	atomic_set(&g_algo_tx_enable, ctrl_value);
	return 0;
}

static int aw882xx_skt_disable_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: aw882xx_skt_disable %d\n",
		__func__, atomic_read(&g_skt_disable));

	ucontrol->value.integer.value[0] = atomic_read(&g_skt_disable);

	return 0;
}

static void aw882xx_skt_set_dsp(int value)
{
        int ret;
	int port_id = AFE_RX_PROT_ID;
	int module_id = AW_MODULE_ID_COPP;
	int param_id =  AW_MODULE_PARAMS_ID_COPP_ENABLE;

	ret = aw_adm_param_enable(port_id, module_id, param_id, value);
	if (ret) {
		pr_err("%s: set skt %d failed \n", __func__, value);
		return;
	}

	pr_info("%s: set skt %s", __func__, value == 1 ? "enable" : "disable");
}

static int aw882xx_skt_disable_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];

	pr_debug("%s: aw882xx_skt_disable %d \n", __func__, value);
	aw882xx_skt_set_dsp(!value);
	atomic_set(&g_skt_disable, value);

	return 0;
}

static const struct soc_enum aw882xx_snd_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rcv_function), rcv_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(awinic_algo), awinic_algo),
};

static struct snd_kcontrol_new aw882xx_controls[] = {
	SOC_ENUM_EXT("aw882xx_speaker_switch", aw882xx_snd_enum[0],
		aw882xx_spk_get, aw882xx_spk_set),
	SOC_ENUM_EXT("aw882xx_receiver_switch", aw882xx_snd_enum[1],
		aw882xx_rcv_get, aw882xx_rcv_set),
	SOC_ENUM_EXT("aw882xx_rx_switch", aw882xx_snd_enum[2],
		aw882xx_algo_get, aw882xx_algo_set),
	SOC_ENUM_EXT("aw882xx_tx_switch", aw882xx_snd_enum[2],
		aw882xx_tx_get, aw882xx_tx_set),
	SOC_ENUM_EXT("aw882xx_skt_disable", aw882xx_snd_enum[0],
		aw882xx_skt_disable_get, aw882xx_skt_disable_set),
};

static void aw882xx_add_codec_controls(struct aw882xx *aw882xx)
{
	pr_info("%s: enter\n", __func__);

	snd_soc_add_codec_controls(aw882xx->codec, aw882xx_controls,
		ARRAY_SIZE(aw882xx_controls));

	snd_soc_add_codec_controls(aw882xx->codec, &aw882xx_volume, 1);
}

/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/


static int aw882xx_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_info("%s: playback enter\n", __func__);
		aw882xx_run_pwd(aw882xx, false);
	} else {
		pr_info("%s: capture enter\n", __func__);
	}

	return 0;
}

static int aw882xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(dai->codec);*/
	struct snd_soc_codec *codec = dai->codec;

	pr_info("%s: fmt=0x%x\n", __func__, fmt);

	/* Supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) !=
			SND_SOC_DAIFMT_CBS_CFS) {
			dev_err(codec->dev, "%s: invalid codec master mode\n",
				__func__);
			return -EINVAL;
		}
		break;
	default:
		dev_err(codec->dev, "%s: unsupported DAI format %d\n",
			__func__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	return 0;
}

static int aw882xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec_dai->codec);

	pr_info("%s: freq=%d\n", __func__, freq);

	aw882xx->sysclk = freq;
	return 0;
}

static int aw882xx_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);
	unsigned int rate = 0;
	int reg_value = 0;
	int width = 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_debug("%s: requested rate: %d, sample size: %d\n",
			__func__, rate, snd_pcm_format_width(params_format(params)));
		return 0;
	}
	/* get rate param */
	aw882xx->rate = rate = params_rate(params);
	pr_debug("%s: requested rate: %d, sample size: %d\n",
		__func__, rate, snd_pcm_format_width(params_format(params)));

	/* match rate */
	switch (rate) {
	case 8000:
		reg_value = AW882XX_I2SSR_8KHZ_VALUE;
		break;
	case 16000:
		reg_value = AW882XX_I2SSR_16KHZ_VALUE;
		break;
	case 32000:
		reg_value = AW882XX_I2SSR_32KHZ_VALUE;
		break;
	case 44100:
		reg_value = AW882XX_I2SSR_44P1KHZ_VALUE;
		break;
	case 48000:
		reg_value = AW882XX_I2SSR_48KHZ_VALUE;
		break;
	case 96000:
		reg_value = AW882XX_I2SSR_96KHZ_VALUE;
		break;
	case 192000:
		reg_value = AW882XX_I2SSR_192KHZ_VALUE;
		break;
	default:
		reg_value = AW882XX_I2SSR_48KHZ_VALUE;
		pr_err("%s: rate can not support\n", __func__);
		break;
	}

	/* set chip rate */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_I2SCTRL_REG,
				AW882XX_I2SSR_MASK, reg_value);
	}

	/* get bit width */
	width = params_width(params);
	pr_debug("%s: width = %d\n", __func__, width);
	switch (width) {
	case 16:
		reg_value = AW882XX_I2SFS_16_BITS_VALUE;
		break;
	case 20:
		reg_value = AW882XX_I2SFS_20_BITS_VALUE;
		break;
	case 24:
		reg_value = AW882XX_I2SFS_24_BITS_VALUE;
		break;
	case 32:
		reg_value = AW882XX_I2SFS_32_BITS_VALUE;
		break;
	default:
		reg_value = AW882XX_I2SFS_16_BITS_VALUE;
		pr_err("%s: width can not support\n", __func__);
		break;
	}

	/* get width */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW882XX_I2SCTRL_REG,
				AW882XX_I2SFS_MASK, reg_value);
	}

	return 0;
}

static int aw882xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);

	pr_info("%s: mute state=%d\n", __func__, mute);

	if (!(aw882xx->flags & AW882XX_FLAG_START_ON_MUTE))
		return 0;

	if (mute) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			aw882xx_smartpa_cfg(aw882xx, false);
		atomic_set(&g_algo_tx_enable, 0);
		atomic_set(&g_algo_rx_enable, 0);
		atomic_set(&g_skt_disable, 0);
	} else {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			aw882xx_smartpa_cfg(aw882xx, true);
	}

	return 0;
}

static void aw882xx_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw882xx->rate = 0;
		aw882xx_run_pwd(aw882xx, true);
	}
}

static const struct snd_soc_dai_ops aw882xx_dai_ops = {
	.startup = aw882xx_startup,
	.set_fmt = aw882xx_set_fmt,
	.set_sysclk = aw882xx_set_dai_sysclk,
	.hw_params = aw882xx_hw_params,
	.mute_stream = aw882xx_mute,
	.shutdown = aw882xx_shutdown,
};

static struct snd_soc_dai_driver aw882xx_dai[] = {
	{
		.name = "aw882xx-aif",
		.id = 1,
		.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AW882XX_RATES,
			.formats = AW882XX_FORMATS,
		},
		.capture = {
			.stream_name = "Speaker_Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AW882XX_RATES,
			.formats = AW882XX_FORMATS,
		 },
		.ops = &aw882xx_dai_ops,
		.symmetric_rates = 1,
#if 0
		.symmetric_channels = 1,
		.symmetric_samplebits = 1,
#endif
	},
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static int aw882xx_probe(struct snd_soc_codec *codec)
{
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);
	int ret = -1;

	pr_info("%s: enter\n", __func__);

	aw882xx->codec = codec;

	aw882xx_add_codec_controls(aw882xx);

	//if (codec->dev->of_node)
	//	dev_set_name(codec->dev, "%s", "aw882xx_smartpa");

	pr_info("%s: exit\n", __func__);

	ret = 0;
	return ret;
}

static int aw882xx_remove(struct snd_soc_codec *codec)
{
	/*struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);*/
	pr_info("%s: enter\n", __func__);

	return 0;
}


static unsigned int aw882xx_codec_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;
	int ret = -1;

	pr_debug("%s: enter\n", __func__);

	if (aw882xx_reg_access[reg] & REG_RD_ACCESS) {
		ret = aw882xx_i2c_read(aw882xx, reg, &value);
		if (ret < 0)
			pr_debug("%s: read register failed\n", __func__);
	} else {
		pr_debug("%s: register 0x%x no read access\n", __func__, reg);
	}
	return ret;
}

static int aw882xx_codec_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	int ret = -1;
	struct aw882xx *aw882xx = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: enter ,reg is 0x%x value is 0x%x\n",
		__func__, reg, value);

	if (aw882xx_reg_access[reg]&REG_WR_ACCESS) {
		ret = aw882xx_i2c_write(aw882xx, reg, value);
	} else {
		pr_debug("%s: register 0x%x no write access\n",
			__func__, reg);
	}

	return ret;
}

static struct snd_soc_codec_driver soc_codec_dev_aw882xx = {
	.probe = aw882xx_probe,
	.remove = aw882xx_remove,
	.read = aw882xx_codec_read,
	.write = aw882xx_codec_write,
	.reg_cache_size = AW882XX_REG_MAX,
	.reg_word_size = 2,
};

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw882xx_interrupt_setup(struct aw882xx *aw882xx)
{
	unsigned int reg_val;

	pr_info("%s: enter\n", __func__);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSINTM_REG, &reg_val);
	reg_val &= (~AW882XX_UVLS_VDD_BELOW_2P8V_VALUE);
	reg_val &= (~AW882XX_NOCLKS_TRIG_VALUE);
	reg_val &= (~AW882XX_CLKS_TRIG_VALUE);
	aw882xx_i2c_write(aw882xx, AW882XX_SYSINTM_REG, reg_val);
}

static void aw882xx_interrupt_clear(struct aw882xx *aw882xx)
{
	unsigned int reg_val = 0;

	pr_info("%s: enter\n", __func__);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSST_REG, &reg_val);
	pr_info("%s: reg SYSST=0x%x\n", __func__, reg_val);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSINT_REG, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSINTM_REG, &reg_val);
	pr_info("%s: reg SYSINTM=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw882xx_irq(int irq, void *data)
{
	struct aw882xx *aw882xx = data;

	pr_info("%s: enter\n", __func__);

	aw882xx_interrupt_clear(aw882xx);

	pr_info("%s: exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw882xx_parse_dt(struct device *dev, struct aw882xx *aw882xx,
		struct device_node *np)
{
	int ret = 0;
	struct aw882xx_monitor *monitor = &aw882xx->monitor;
	/* gpio */
	aw882xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw882xx->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n",
			__func__);
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw882xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw882xx->irq_gpio < 0)
		dev_info(dev, "%s: no irq gpio provided.\n", __func__);
	else
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);

	ret = of_property_read_u32(np, "monitor-flag", &monitor->is_enable);
	if (ret) {
		monitor->is_enable = AW882XX_MONITOR_DEFAULT_FLAG;
		dev_err(dev, "%s: monitor-flag get failed ,user default value!\n", __func__);
	} else {
		dev_info(dev, "%s: monitor-flag = %d\n",
			__func__, monitor->is_enable);
	}

	ret = of_property_read_u32(np, "monitor-timer-val", &monitor->timer_val);
	if (ret) {
		monitor->timer_val = AW882XX_MONITOR_DEFAULT_TIMER_VAL;
		dev_err(dev, "%s: monitor-timer-val get failed,user default value!\n", __func__);
	} else {
		dev_info(dev, "%s: monitor-timer-val = %d\n",
			__func__, monitor->timer_val);
	}
	return 0;
}

int aw882xx_hw_reset(struct aw882xx *aw882xx)
{
	pr_info("%s: enter\n", __func__);

	if (aw882xx && gpio_is_valid(aw882xx->reset_gpio)) {
		gpio_set_value_cansleep(aw882xx->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw882xx->reset_gpio, 1);
		msleep(2);
	} else {
		dev_err(aw882xx->dev, "%s: failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
int aw882xx_read_chipid(struct aw882xx *aw882xx)
{
	int ret = -1;
	unsigned int cnt = 0;
	unsigned int reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw882xx_i2c_read(aw882xx, AW882XX_ID_REG, &reg);
		if (ret < 0) {
			dev_err(aw882xx->dev,
				"%s: failed to read REG_ID: %d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg) {
		case AW882XX_ID:
			pr_info("%s: aw882xx detected\n", __func__);
			aw882xx->flags |= AW882XX_FLAG_SKIP_INTERRUPTS;
			aw882xx->flags |= AW882XX_FLAG_START_ON_MUTE;
			aw882xx->chipid = AW882XX_ID;
			pr_info("%s: aw882xx->flags=0x%x\n",
				__func__, aw882xx->flags);
			return 0;
		default:
			pr_info("%s: unsupported device revision (0x%x)\n",
				__func__, reg);
			break;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw882xx_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);

	return count;
}

static ssize_t aw882xx_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned int reg_val = 0;

	for (i = 0; i < AW882XX_REG_MAX; i++) {
		if (aw882xx_reg_access[i]&REG_RD_ACCESS) {
			aw882xx_i2c_read(aw882xx, i, &reg_val);
			len += snprintf(buf+len, PAGE_SIZE-len,
				"reg:0x%02x=0x%04x\n", i, reg_val);
		}
	}
	return len;
}

static ssize_t aw882xx_rw_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw882xx->reg_addr = (unsigned char)databuf[0];
		aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);
	} else if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw882xx->reg_addr = (unsigned char)databuf[0];
	}

	return count;
}

static ssize_t aw882xx_rw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int reg_val = 0;

	if (aw882xx_reg_access[aw882xx->reg_addr] & REG_RD_ACCESS) {
		aw882xx_i2c_read(aw882xx, aw882xx->reg_addr, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"reg:0x%02x=0x%04x\n", aw882xx->reg_addr, reg_val);
	}
	return len;
}

static ssize_t aw882xx_spk_rcv_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);

	int ret = -1;
	unsigned int databuf[2] = {0};

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	aw882xx->spk_rcv_mode = databuf[0];

	return count;
}

static ssize_t aw882xx_spk_rcv_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;

	if (aw882xx->spk_rcv_mode == AW882XX_SPEAKER_MODE)
		len += snprintf(buf+len, PAGE_SIZE-len,
			"aw882xx spk_rcv: %d, speaker mode\n",
			aw882xx->spk_rcv_mode);
	else if (aw882xx->spk_rcv_mode == AW882XX_RECEIVER_MODE)
		len += snprintf(buf+len, PAGE_SIZE-len,
			"aw882xx spk_rcv: %d, receiver mode\n",
			aw882xx->spk_rcv_mode);
	else
		len += snprintf(buf+len, PAGE_SIZE-len,
			"aw882xx spk_rcv: %d, unknown mode\n",
			aw882xx->spk_rcv_mode);

	return len;
}

static ssize_t aw882xx_mec_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	/*struct aw882xx *aw882xx = dev_get_drvdata(dev);*/
	uint32_t mec_ctr = 0;
	uint32_t param_id = AFE_PARAM_ID_AWDSP_RX_SET_ENABLE;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &mec_ctr);
	if (ret < 0)
		return ret;

	pr_info("%s: mec_ctr=%d\n", __func__, mec_ctr);

	ret = aw_send_afe_cal_apr(param_id, &mec_ctr, sizeof(uint32_t), true);
	if (ret)
		pr_err("%s: dsp_msg error, ret=%d\n", __func__, ret);

	mdelay(2);

	return count;
}

static ssize_t aw882xx_mec_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	/*struct aw882xx *aw882xx = dev_get_drvdata(dev);*/
	ssize_t len = 0;
	int ret = -1;
	uint8_t *buffer = NULL;
	uint32_t mec_ctr = 0;
	uint32_t param_id = AFE_PARAM_ID_AWDSP_RX_SET_ENABLE;

	buffer = kmalloc(sizeof(uint32_t), GFP_KERNEL);
	if (buffer == NULL) {
		pr_err("%s can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	ret = aw_send_afe_cal_apr(param_id, buffer, sizeof(uint32_t), false);
	if (ret) {
		pr_err("%s: dsp_msg_read error: %d\n", __func__, ret);
		kfree(buffer);
		return -EFAULT;
	}

	memcpy(&mec_ctr, buffer, sizeof(uint32_t));

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx mec: %d\n", mec_ctr);

	kfree(buffer);

	return len;
}
#ifdef AW_DEBUG
static ssize_t aw882xx_vol_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	uint32_t vol = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &vol);
	if (ret < 0)
		return ret;

	pr_info("%s: vol set =%d\n", __func__, vol);
	aw882xx->monitor.test_vol = vol;

	return count;
}

static ssize_t aw882xx_vol_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint32_t local_vol = aw882xx->monitor.test_vol;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx vol: %d\n", local_vol);
	return len;
}
static ssize_t aw882xx_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	int32_t temp = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtoint(buf, 0, &temp);
	if (ret < 0)
		return ret;

	pr_info("%s: temp set =%d\n", __func__, temp);
	aw882xx->monitor.test_temp = temp;

	return count;
}

static ssize_t aw882xx_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int32_t local_temp = aw882xx->monitor.test_temp;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx vol: %d\n", local_temp);
	return len;
}
#endif
static int aw882xx_cali_operation(struct aw882xx *aw882xx,
			unsigned int cmd, unsigned long arg)
{
	int16_t data_len  = _IOC_SIZE(cmd);
	int ret = 0;
	char *data_ptr = NULL;


	data_ptr = kmalloc(data_len, GFP_KERNEL);
	if (data_ptr == NULL) {
		pr_err("%s : malloc failed !\n", __func__);
		return -EFAULT;
	}

	pr_info("cmd : %d, data_len%d\n", cmd , data_len);
	switch (cmd) {
		case AW882XX_IOCTL_ENABLE_CALI: {
			if (copy_from_user(data_ptr,
					(void __user *)arg, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
			g_aw882xx_cali_flag = (int8_t)data_ptr[0];
			pr_info("%s:set cali %s", __func__,
				(g_aw882xx_cali_flag == 0) ? ("disable") : ("enable"));
		} break;
		case AW882XX_IOCTL_SET_CALI_CFG: {
			if (copy_from_user(data_ptr,
					(void __user *)arg, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
			ret = aw_send_afe_cal_apr(
				AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L,
							data_ptr, data_len, true);
			if (ret) {
				pr_err("%s: dsp_msg_write error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L);
				ret =  -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_GET_CALI_CFG: {
			ret = aw_send_afe_cal_apr(
				AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L,
						data_ptr, data_len, false);
			if (ret) {
				pr_err("%s: dsp_msg_read error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L);
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((void __user *)arg,
				data_ptr, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_GET_CALI_DATA: {
			ret = aw_send_afe_cal_apr(
				AFE_PARAM_ID_AWDSP_RX_REAL_DATA_L,
						data_ptr, data_len, false);
			if (ret) {
				pr_err("%s: dsp_msg_read error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_REAL_DATA_L);
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((void __user *)arg,
				data_ptr, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_SET_NOISE: {
			if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_NOISE_L,
						data_ptr, data_len, true);
			if (ret) {
				pr_err("%s: dsp_msg_write error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_NOISE_L);
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_GET_F0: {
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_F0_L,
						data_ptr, data_len, false);
			if (ret) {
				pr_err("%s: dsp_msg_read error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_F0_L);
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((void __user *)arg,
				data_ptr, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_SET_CALI_RE: {
			if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_RE_L,
						data_ptr, data_len, true);
			if (ret) {
				pr_err("%s: dsp_msg_write error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_RE_L);
				ret = -EFAULT;
				goto exit;
			}
			aw882xx_set_cali_re(aw882xx, *((int32_t *)data_ptr));
		} break;
		case AW882XX_IOCTL_GET_CALI_RE: {
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_RE_L,
						data_ptr, data_len, false);
			if (ret) {
				pr_err("%s: dsp_msg_read error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_RE_L);
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((void __user *)arg,
					data_ptr, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_GET_VMAX: {
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_VMAX_L,
				data_ptr, data_len, false);
			if (ret) {
				pr_err("%s: dsp_msg_read error:0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_VMAX_L);
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((void __user *)arg,
				data_ptr, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_SET_VMAX: {
			if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_VMAX_L,
						data_ptr, data_len, true);
			if (ret) {
				pr_err("%s: dsp_msg_write error: 0x%x\n",
					__func__, AFE_PARAM_ID_AWDSP_RX_VMAX_L);
				ret = -EFAULT;
				goto exit;
			}
		} break;
		case AW882XX_IOCTL_SET_PARAM: {
			if (copy_from_user(data_ptr,
				(void __user *)arg, data_len)) {
				ret = -EFAULT;
				goto exit;
			}
			ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_PARAMS,
						data_ptr, data_len, true);
			if (ret) {
				pr_err("%s: dsp_msg_write error: 0x%x\n",
				__func__, AFE_PARAM_ID_AWDSP_RX_PARAMS);
				ret = -EFAULT;
				goto exit;
			}
			pr_debug("%s: set params done", __func__);
		} break;
		default: {
			pr_err("%s : cmd %d\n", __func__, cmd);
		} break;
	}
exit:
	kfree(data_ptr);
	return ret;
}


static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
	aw882xx_reg_show, aw882xx_reg_store);
static DEVICE_ATTR(rw, S_IWUSR | S_IRUGO,
	aw882xx_rw_show, aw882xx_rw_store);
static DEVICE_ATTR(spk_rcv, S_IWUSR | S_IRUGO,
	aw882xx_spk_rcv_show, aw882xx_spk_rcv_store);
static DEVICE_ATTR(mec, S_IWUSR | S_IRUGO,
	aw882xx_mec_show, aw882xx_mec_store);
#ifdef AW_DEBUG
static DEVICE_ATTR(vol, S_IWUSR | S_IRUGO,
	aw882xx_vol_show, aw882xx_vol_store);
static DEVICE_ATTR(temp, S_IWUSR | S_IRUGO,
	aw882xx_temp_show, aw882xx_temp_store);
#endif

static struct attribute *aw882xx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rw.attr,
	&dev_attr_spk_rcv.attr,
	&dev_attr_mec.attr,
#ifdef AW_DEBUG
	&dev_attr_vol.attr,
	&dev_attr_temp.attr,
#endif
	NULL
};

static struct attribute_group aw882xx_attribute_group = {
	.attrs = aw882xx_attributes
};

#define AW882XX_SMARTPA_NAME "aw882xx_smartpa"
static int aw882xx_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	file->private_data = (void *)g_aw882xx;

	pr_debug("open success");
	return 0;
}

static int aw882xx_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	pr_debug("release successi\n");
	return 0;
}

static long aw882xx_file_unlocked_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aw882xx *aw882xx = NULL;

	if (((_IOC_TYPE(cmd)) != (AW882XX_IOCTL_MAGIC))) {
	    pr_err("%s: cmd magic err\n", __func__);
	    return -EINVAL;
	}
	aw882xx = (struct aw882xx *)file->private_data;
	ret = aw882xx_cali_operation(aw882xx, cmd, arg);
	if (ret)
		return -EINVAL;

	return 0;
}

static struct file_operations aw882xx_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aw882xx_file_unlocked_ioctl,
	.open = aw882xx_file_open,
	.release = aw882xx_file_release,
};

static struct miscdevice aw882xx_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW882XX_SMARTPA_NAME,
	.fops = &aw882xx_fops,
};

void init_aw882xx_misc_driver(struct aw882xx *aw882xx)
{
	int ret;

	ret = misc_register(&aw882xx_misc);
	if (ret) {
		pr_err("%s: misc register fail: %d\n", __func__, ret);
		return;
	}
	pr_debug("%s: misc register success", __func__);
}

/*monitor*/
static int aw882xx_monitor_start(struct aw882xx_monitor *monitor)
{
	pr_info("%s: enter\n", __func__);

	if (!hrtimer_active(&monitor->timer)) {
		pr_info("%s: start monitor\n", __func__);
		hrtimer_start(&monitor->timer,
			ktime_set(monitor->timer_val/1000,
			 (monitor->timer_val%1000)*1000000), HRTIMER_MODE_REL);
	}

	return 0;
}

static int aw882xx_monitor_stop(struct aw882xx_monitor *monitor)
{
	pr_info("%s: enter\n", __func__);

	if (hrtimer_active(&monitor->timer)) {
		pr_info("%s: stop monitor\n", __func__);
		hrtimer_cancel(&monitor->timer);
	}
	return 0;
}

static enum hrtimer_restart
	aw882xx_monitor_timer_func(struct hrtimer *timer)
{
	struct aw882xx_monitor *monitor =
		container_of(timer, struct aw882xx_monitor, timer);

	pr_debug("%s : enter\n", __func__);

	if (monitor->is_enable)
		schedule_work(&monitor->work);

	return HRTIMER_NORESTART;
}
static int aw882xx_monitor_get_voltage(struct aw882xx *aw882xx,
						unsigned int *vol)
{
	int ret = -1;
	uint16_t local_vol = 0;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_VBAT_REG, vol);
	if (ret < 0) {
		pr_err("%s: read voltage failed !\n",  __func__);
		return ret;
	}
	local_vol = (*vol) * AW882XX_MONITOR_VBAT_RANGE
				 / AW882XX_MONITOR_INT_10BIT;

	*vol = local_vol;
	pr_debug("%s: chip voltage is %d\n", __func__, local_vol);
	return 0;
}
static int aw882xx_monitor_voltage(struct aw882xx *aw882xx,
				struct aw882xx_low_vol *vol_cfg)
{
	int ret = -1;
	int i;
	unsigned int voltage = 0;
	struct aw882xx_monitor *monitor = NULL;

	if (aw882xx == NULL || vol_cfg == NULL) {
		pr_err("%s: pointer is NUL\n", __func__);
		return ret;
	}
	monitor = &aw882xx->monitor;
#ifdef AW_DEBUG
	if (monitor->test_vol == 0) {
		ret = aw882xx_monitor_get_voltage(aw882xx, &voltage);
		if (ret < 0)
			return ret;
	} else {
		voltage = monitor->test_vol;
	}
#else
	ret = aw882xx_monitor_get_voltage(aw882xx, &voltage);
	if (ret < 0)
		return ret;
#endif
	if (monitor->pre_vol > voltage) {
		/* vol down*/
		for (i = 0; i < 3; i++) {
			if (voltage < vol_down_table[i].vol) {
				*vol_cfg = vol_down_table[i];
				break;
			}
		}
		if (i == 3) {
			vol_cfg->ipeak = IPEAK_NONE;
			vol_cfg->gain  = GAIN_NONE;
		}
	} else if (monitor->pre_vol < voltage) {
		/*vol up*/
		for (i = 0; i < 3; i++) {
			if (voltage > vol_up_table[i].vol) {
				*vol_cfg = vol_up_table[i];
				break;
			}
		}
		if (i == 3) {
			vol_cfg->ipeak = IPEAK_NONE;
			vol_cfg->gain  = GAIN_NONE;
		}
	} else {
		/*vol no change*/
		vol_cfg->ipeak = IPEAK_NONE;
		vol_cfg->gain  = GAIN_NONE;
	}
	monitor->pre_vol = voltage;
	return 0;
}
static int aw882xx_monitor_get_temperature(struct aw882xx *aw882xx,  int *temp)
{
	int ret = -1;
	unsigned int reg_val = 0;
	uint16_t local_temp;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_TEMP_REG, &reg_val);
	if (ret < 0) {
		pr_err("%s: get temperature failed !\n", __func__);
		return ret;
	}

	local_temp = reg_val;
	if (local_temp & AW882XX_MONITOR_TEMP_SIGN_MASK)
		local_temp = local_temp | AW882XX_MONITOR_TEMP_NEG_MASK;

	*temp = (int)local_temp;
	pr_debug("%s: chip temperature = %d\n", __func__, local_temp);
	return 0;
}

static int aw882xx_monitor_temperature(struct aw882xx *aw882xx,
				struct aw882xx_low_temp *temp_cfg)
{
	int ret;
	int i;
	struct aw882xx_monitor *monitor = NULL;
	int  current_temp = 0;

	monitor = &aw882xx->monitor;
#ifdef AW_DEBUG
	if (monitor->test_temp == 0) {
		ret = aw882xx_monitor_get_temperature(aw882xx, &current_temp);
		if (ret)
			return ret;
	} else {
		current_temp = monitor->test_temp;
	}
#else
	ret = aw882xx_monitor_get_temperature(aw882xx, &current_temp);
	if (ret < 0)
		return ret;
#endif
	if (monitor->pre_temp > current_temp) {
		/*temp down*/
		for (i = 0; i < 3; i++) {
			if (current_temp < temp_down_table[i].temp) {
				temp_cfg->ipeak = temp_down_table[i].ipeak;
				temp_cfg->gain = temp_down_table[i].gain;
				temp_cfg->vmax = temp_down_table[i].vmax;
				break;
			}
		}

		if (i == 3) {
			temp_cfg->ipeak = IPEAK_NONE;
			temp_cfg->gain  = GAIN_NONE;
			temp_cfg->vmax  = VMAX_NONE;
		}
	} else if (monitor->pre_temp < current_temp) {
		/*temp up*/
		for (i = 0; i < 3; i++) {
			if (current_temp > temp_up_table[i].temp) {
				temp_cfg->ipeak = temp_up_table[i].ipeak;
				temp_cfg->gain  = temp_up_table[i].gain;
				temp_cfg->vmax  = temp_up_table[i].vmax;
				break;
			}
		}
		if (i == 3) {
			temp_cfg->ipeak = IPEAK_NONE;
			temp_cfg->gain  = GAIN_NONE;
			temp_cfg->vmax  = VMAX_NONE;
		}
	} else {
		/*temp no change*/
		temp_cfg->ipeak = IPEAK_NONE;
		temp_cfg->gain  = GAIN_NONE;
		temp_cfg->vmax  = VMAX_NONE;
	}
	monitor->pre_temp = current_temp;
	return 0;
}

static void aw882xx_monitor_get_cfg(struct aw882xx_low_temp *temp,
					struct aw882xx_low_vol *vol)
{
	if (vol->ipeak == IPEAK_NONE)
		return;

	if (temp->ipeak == IPEAK_NONE) {
		temp->ipeak = vol->ipeak;
		temp->gain  = vol->gain;
		return;
	}

	/*get min ipeak*/
	if (temp->ipeak > vol->ipeak)
		temp->ipeak = vol->ipeak;

	/*get min gain*/
	if (temp->gain < vol->gain)
		temp->gain = vol->gain;

}
static void aw882xx_monitor_set_ipeak(struct aw882xx *aw882xx, uint8_t ipeak)
{
	unsigned int reg_val = 0;
	unsigned int read_reg_val;
	int ret;

	if (ipeak == IPEAK_NONE)
		return;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_SYSCTRL2_REG, &reg_val);
	if (ret < 0) {
		pr_err("%s: read ipeak failed\n", __func__);
		return;
	}

	read_reg_val = reg_val;
	read_reg_val &= AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK;

	if (read_reg_val == ipeak) {
		pr_debug("%s: ipeak = 0x%x, no change\n",
					__func__, read_reg_val);
		return;
	}
	reg_val &= (~AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK);
	read_reg_val = ipeak;
	reg_val |= read_reg_val;

	ret = aw882xx_i2c_write(aw882xx, AW882XX_SYSCTRL2_REG, reg_val);
	if (ret < 0) {
		pr_err("%s: write ipeak failed\n", __func__);
		return;
	}
	pr_debug("%s: set reg val = 0x%x, ipeak = 0x%x\n",
					__func__, reg_val, ipeak);
}
static void aw882xx_monitor_set_gain(struct aw882xx *aw882xx, uint8_t gain)
{
	unsigned int reg_val = 0;
	unsigned int read_reg_val;
	int ret;

	if (gain == GAIN_NONE)
		return;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_HAGCCFG4_REG, &reg_val);
	if (ret < 0) {
		pr_err("%s: read gain failed\n", __func__);
		return;
	}

	read_reg_val = reg_val;
	read_reg_val = read_reg_val >> AW882XX_BIT_HAGCCFG4_GAIN_SHIFT;

	if (read_reg_val == gain) {
		pr_debug("%s: gain = 0x%x, no change\n",
				__func__, read_reg_val);
		return;
	}
	reg_val &= AW882XX_BIT_HAGCCFG4_GAIN_MASK;
	read_reg_val = gain;
	reg_val |= (read_reg_val << AW882XX_BIT_HAGCCFG4_GAIN_SHIFT);

	ret = aw882xx_i2c_write(aw882xx, AW882XX_HAGCCFG4_REG, reg_val);
	if (ret < 0) {
		pr_err("%s: write gain failed\n", __func__);
		return;
	}
	pr_debug("%s: set reg val = 0x%x, gain = 0x%x\n",
			__func__, reg_val, gain);
}
static void aw882xx_monitor_set_vmax(struct aw882xx *aw882xx, uint32_t vmax)
{
	uint32_t local_vmax = vmax;
	int ret;

	if (vmax == VMAX_NONE)
		return;

	ret = aw_send_afe_cal_apr(AFE_PARAM_ID_AWDSP_RX_VMAX_L,
				&local_vmax, sizeof(uint32_t), true);
	if (ret)
		pr_err("%s: dsp_msg_write error: 0x%x\n",
			__func__, AFE_PARAM_ID_AWDSP_RX_VMAX_L);

	pr_debug("%s: set vmax = 0x%x\n", __func__, vmax);
}
static void aw882xx_monitor_work(struct aw882xx *aw882xx)
{
	struct aw882xx_low_vol vol_cfg;
	struct aw882xx_low_temp temp_cfg;
	int ret;

	if (aw882xx == NULL) {
		pr_err("%s: pointer is NULL\n", __func__);
		return;
	}
	if (g_aw882xx_cali_flag != 0) {
		pr_info("%s: done nothing while start cali", __func__);
		return;
	}

	ret = aw882xx_monitor_voltage(aw882xx, &vol_cfg);
	if (ret < 0) {
		pr_err("%s: monitor voltage failed\n", __func__);
		return;
	}

	ret = aw882xx_monitor_temperature(aw882xx, &temp_cfg);
	if (ret < 0) {
		pr_err("%s: monitor temperature failed\n", __func__);
		return;
	}
	pr_debug("%s: vol: ipeak = 0x%x, gain = 0x%x\n",
			__func__, vol_cfg.ipeak, vol_cfg.gain);
	pr_debug("%s: temp: ipeak = 0x%x, gain = 0x%x, vmax = 0x%x\n",
		__func__, temp_cfg.ipeak, temp_cfg.gain, temp_cfg.vmax);

	aw882xx_monitor_get_cfg(&temp_cfg, &vol_cfg);

	aw882xx_monitor_set_ipeak(aw882xx, temp_cfg.ipeak);

	aw882xx_monitor_set_gain(aw882xx, temp_cfg.gain);

	aw882xx_monitor_set_vmax(aw882xx, temp_cfg.vmax);
}

static int aw882xx_get_hmute(struct aw882xx *aw882xx)
{
	unsigned int reg_val = 0;
	int ret;

	pr_debug("%s: enter\n", __func__);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSCTRL2_REG, &reg_val);
	if ((~AW882XX_HMUTE_MASK) & reg_val)
		ret = 1;
	else
		ret = 0;

	return ret;
}
static void aw882xx_monitor_work_func(struct work_struct *work)
{
	struct aw882xx_monitor *monitor = container_of(work,
				struct aw882xx_monitor, work);
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);

	pr_info("%s: enter\n", __func__);
	mutex_lock(&aw882xx->lock);
	if (!aw882xx_get_hmute(aw882xx)) {
		aw882xx_monitor_work(aw882xx);
		aw882xx_monitor_start(monitor);
	}
	mutex_unlock(&aw882xx->lock);
}

void init_aw882xx_monitor(struct aw882xx_monitor *monitor)
{
	pr_info("%s: enter\n", __func__);
	hrtimer_init(&monitor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	monitor->timer.function = aw882xx_monitor_timer_func;
	INIT_WORK(&monitor->work, aw882xx_monitor_work_func);
	monitor->pre_vol = 0;
	monitor->pre_temp = 0;
#ifdef AW_DEBUG
	 monitor->test_vol = 0;
	 monitor->test_temp = 0;
#endif
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw882xx_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai;
	struct aw882xx *aw882xx;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	pr_info("%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw882xx = devm_kzalloc(&i2c->dev, sizeof(struct aw882xx), GFP_KERNEL);
	if (aw882xx == NULL)
		return -ENOMEM;

	aw882xx->dev = &i2c->dev;
	aw882xx->i2c = i2c;

	i2c_set_clientdata(i2c, aw882xx);
	mutex_init(&aw882xx->lock);

	/* aw882xx rst & int */
	if (np) {
		ret = aw882xx_parse_dt(&i2c->dev, aw882xx, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		aw882xx->reset_gpio = -1;
		aw882xx->irq_gpio = -1;
	}

	if (gpio_is_valid(aw882xx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw882xx->reset_gpio,
			GPIOF_OUT_INIT_LOW, "aw882xx_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw882xx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw882xx->irq_gpio,
			GPIOF_DIR_IN, "aw882xx_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err_irq_gpio_request;
		}
	}

	/* hardware reset */
	aw882xx_hw_reset(aw882xx);

	/* aw882xx chip id */
	ret = aw882xx_read_chipid(aw882xx);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw882xx_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw882xx device name */
	//if (i2c->dev.of_node)
	//	dev_set_name(&i2c->dev, "%s", "aw882xx_smartpa");
	//else
	//	dev_err(&i2c->dev, "%s failed to set device name: %d\n",
	//		__func__, ret);

	/* register codec */
	dai = devm_kzalloc(&i2c->dev, sizeof(aw882xx_dai), GFP_KERNEL);
	if (!dai)
		goto err_dai_kzalloc;

	memcpy(dai, aw882xx_dai, sizeof(aw882xx_dai));
	pr_info("%s: dai->name(%s)\n", __func__, dai->name);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_aw882xx,
			dai, ARRAY_SIZE(aw882xx_dai));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s failed to register aw882xx: %d\n",
			__func__, ret);
		goto err_register_codec;
	}

	/* aw882xx irq */
	if (gpio_is_valid(aw882xx->irq_gpio) &&
		!(aw882xx->flags & AW882XX_FLAG_SKIP_INTERRUPTS)) {
		aw882xx_interrupt_setup(aw882xx);
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
				gpio_to_irq(aw882xx->irq_gpio),
				NULL, aw882xx_irq, irq_flags,
				"aw882xx", aw882xx);
		if (ret != 0) {
			dev_err(&i2c->dev, "failed to request IRQ %d: %d\n",
				gpio_to_irq(aw882xx->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw882xx->flags |= AW882XX_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw882xx);
	ret = sysfs_create_group(&i2c->dev.kobj, &aw882xx_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev,
			"%s error creating sysfs attr files\n",
			__func__);
		goto err_sysfs;
	}

	init_aw882xx_monitor(&aw882xx->monitor);

	init_aw882xx_misc_driver(aw882xx);
	g_aw882xx = aw882xx;
	pr_info("%s: probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw882xx->irq_gpio), aw882xx);
err_irq:
	snd_soc_unregister_codec(&i2c->dev);
err_register_codec:
	devm_kfree(&i2c->dev, dai);
	dai = NULL;
err_dai_kzalloc:
err_id:
	if (gpio_is_valid(aw882xx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw882xx->irq_gpio);
err_irq_gpio_request:
	if (gpio_is_valid(aw882xx->reset_gpio))
		devm_gpio_free(&i2c->dev, aw882xx->reset_gpio);
err_reset_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw882xx);
	aw882xx = NULL;
	g_aw882xx = NULL;
	return ret;
}

static int aw882xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw882xx *aw882xx = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);
	misc_deregister(&aw882xx_misc);
	if (gpio_to_irq(aw882xx->irq_gpio))
		devm_free_irq(&i2c->dev,
			gpio_to_irq(aw882xx->irq_gpio),
			aw882xx);

	snd_soc_unregister_codec(&i2c->dev);

	if (gpio_is_valid(aw882xx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw882xx->irq_gpio);
	if (gpio_is_valid(aw882xx->reset_gpio))
		devm_gpio_free(&i2c->dev, aw882xx->reset_gpio);

	devm_kfree(&i2c->dev, aw882xx);
	aw882xx = NULL;
	g_aw882xx = NULL;
	return 0;
}

static const struct i2c_device_id aw882xx_i2c_id[] = {
	{ AW882XX_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw882xx_i2c_id);

static struct of_device_id aw882xx_dt_match[] = {
	{ .compatible = "awinic,aw882xx_smartpa" },
	{ },
};

static struct i2c_driver aw882xx_i2c_driver = {
	.driver = {
		.name = AW882XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw882xx_dt_match),
	},
	.probe = aw882xx_i2c_probe,
	.remove = aw882xx_i2c_remove,
	.id_table = aw882xx_i2c_id,
};


static int __init aw882xx_i2c_init(void)
{
	int ret = -1;

	pr_info("%s: aw882xx driver version %s\n", __func__, AW882XX_VERSION);

	ret = i2c_add_driver(&aw882xx_i2c_driver);
	if (ret)
		pr_err("%s: fail to add aw882xx device into i2c\n", __func__);

	return ret;
}
module_init(aw882xx_i2c_init);


static void __exit aw882xx_i2c_exit(void)
{
	i2c_del_driver(&aw882xx_i2c_driver);
}
module_exit(aw882xx_i2c_exit);

MODULE_DESCRIPTION("ASoC AW882XX Smart PA Driver");
MODULE_LICENSE("GPL v2");

#endif /* CONFIG_AW882XX_CODEC */
