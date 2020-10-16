/*
 * aw869xx.c
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Peacek <hushanping@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "aw869xx_reg.h"
#include "aw869xx.h"

/******************************************************
 *
 * Version Marco
 *
 ******************************************************/
#define AW869XX_VERSION "v0.1.2"

/******************************************************
 *
 * aw869xx distinguish between codecs and components by version
 *
 ******************************************************/
#ifdef AW_KERNEL_VER_OVER_4_19
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_component,
	.aw_snd_soc_codec_get_drvdata = snd_soc_component_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_component_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_component,
	.aw_snd_soc_register_codec = snd_soc_register_component,
};
#else
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_codec,
	.aw_snd_soc_codec_get_drvdata = snd_soc_codec_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_codec_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_codec,
	.aw_snd_soc_register_codec = snd_soc_register_codec,
};
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef AW_KERNEL_VER_OVER_4_19
	return dai->component;
#else
	return dai->codec;
#endif
}

static void do_gettimeofday(struct timeval *tv)
{
    struct timespec64 now;

    ktime_get_real_ts64(&now);
    tv->tv_sec = now.tv_sec;
    tv->tv_usec = now.tv_nsec/1000;
}

/******************************************************
 *
 * variable
 *
 ******************************************************/
static char *aw869xx_ram_name = "aw869xx_haptic.bin";
static char aw869xx_rtp_name[][AW869XX_RTP_NAME_MAX] = {
	{"aw869xx_osc_rtp_24K_5s.bin"},
	{"aw869xx_rtp.bin"},
	{"aw869xx_rtp_lighthouse.bin"},
	{"aw869xx_rtp_silk.bin"},
};

struct pm_qos_request pm_qos_req_vb;
struct aw869xx_container *aw869xx_rtp;

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw869xx_interrupt_clear(struct aw869xx *aw869xx);
 /******************************************************
 *
 * aw869xx i2c write/read
 *
 ******************************************************/
static int aw869xx_i2c_write(struct aw869xx *aw869xx,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw869xx->i2c, reg_addr, reg_data);
		if (ret < 0) {
			aw_dev_err(aw869xx->dev,
				   "%s: i2c_write addr=0x%02X, data=0x%02X, cnt=%d, error=%d\n",
				   __func__, reg_addr, reg_data, cnt, ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
			     AW_I2C_RETRY_DELAY * 1000 + 500);
	}
	return ret;
}

static int aw869xx_i2c_read(struct aw869xx *aw869xx,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw869xx->i2c, reg_addr);
		if (ret < 0) {
			aw_dev_err(aw869xx->dev,
				   "%s: i2c_read addr=0x%02X, cnt=%d error=%d\n",
				   __func__, reg_addr, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
			     AW_I2C_RETRY_DELAY * 1000 + 500);
	}
	return ret;
}

static int aw869xx_i2c_write_bits(struct aw869xx *aw869xx,
				  unsigned char reg_addr, unsigned int mask,
				  unsigned char reg_data)
{
	int ret = -1;
	unsigned char reg_val = 0;

	ret = aw869xx_i2c_read(aw869xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s: i2c read error, ret=%d\n",
			   __func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw869xx_i2c_write(aw869xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s: i2c write error, ret=%d\n",
			   __func__, ret);
		return ret;
	}
	return 0;
}

static int aw869xx_i2c_writes(struct aw869xx *aw869xx,
			      unsigned char reg_addr, unsigned char *buf,
			      unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		aw_dev_err(aw869xx->dev, "%s: can not allocate memory\n",
			   __func__);
		return -ENOMEM;
	}
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(aw869xx->i2c, data, len + 1);
	if (ret < 0)
		aw_dev_err(aw869xx->dev, "%s: i2c master send error\n",
			   __func__);
	kfree(data);
	return ret;
}

static void aw869xx_haptic_raminit(struct aw869xx *aw869xx, bool flag)
{
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_ON);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static void aw869xx_haptic_play_go(struct aw869xx *aw869xx)
{
	aw869xx_i2c_write(aw869xx, AW869XX_REG_PLAYCFG4, 0x01);
}

static int aw869xx_haptic_stop(struct aw869xx *aw869xx)
{
	unsigned char cnt = 40;
	unsigned char reg_val = 0;
	bool force_flag = true;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->play_mode = AW869XX_HAPTIC_STANDBY_MODE;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
	if ((reg_val & 0x0f) == 0x00 || (reg_val & 0x0f) == 0x0A) {
		force_flag = false;
		aw_dev_info(aw869xx->dev,
			    "%s already in standby mode! glb_state=0x%02X\n",
			    __func__, reg_val);
	} else {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_PLAYCFG4, 0x02);
		while (cnt) {
			aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
			if ((reg_val & 0x0f) == 0x00
			    || (reg_val & 0x0f) == 0x0A) {
				cnt = 0;
				force_flag = false;
				aw_dev_info(aw869xx->dev,
					    "%s entered standby! glb_state=0x%02X\n",
					    __func__, reg_val);
			} else {
				cnt--;
				pr_debug
				    ("%s wait for standby, glb_state=0x%02X\n",
				     __func__, reg_val);
			}
			usleep_range(2000, 2500);
		}
	}
	if (force_flag) {
		aw_dev_err(aw869xx->dev, "%s force to enter standby mode!\n",
			   __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
				       AW869XX_BIT_SYSCTRL2_STANDBY_ON);
	}
	return 0;
}

static void aw869xx_container_update(struct aw869xx *aw869xx,
				     struct aw869xx_container *aw869xx_cont)
{
	int i = 0;
	unsigned int shift = 0;
	unsigned char reg_val = 0;
	unsigned int temp = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	mutex_lock(&aw869xx->lock);
	aw869xx->ram.baseaddr_shift = 2;
	aw869xx->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw869xx_haptic_raminit(aw869xx, true);
	/* Enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	/* base addr */
	shift = aw869xx->ram.baseaddr_shift;
	aw869xx->ram.base_addr =
	    (unsigned int)((aw869xx_cont->data[0 + shift] << 8) |
			   (aw869xx_cont->data[1 + shift]));
	aw_dev_info(aw869xx->dev, "%s: base_addr = %d\n", __func__,
		    aw869xx->ram.base_addr);

	aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG1, /*ADDRH*/
			  aw869xx_cont->data[0 + shift]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG2, /*ADDRL*/
			  aw869xx_cont->data[1 + shift]);
	/* FIFO_AEH */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RTPCFG3,
			       AW869XX_BIT_RTPCFG3_FIFO_AEH_MASK,
			       (unsigned
				char)(((aw869xx->
					ram.base_addr >> 1) >> 4) & 0xF0));
	/* FIFO AEL */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG4,
			  (unsigned
			   char)(((aw869xx->ram.base_addr >> 1) & 0x00FF)));
	/* FIFO_AFH */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RTPCFG3,
			       AW869XX_BIT_RTPCFG3_FIFO_AFH_MASK,
			       (unsigned char)(((aw869xx->ram.base_addr -
						 (aw869xx->
						  ram.base_addr >> 2)) >> 8) &
					       0x0F));
	/* FIFO_AFL */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG5,
			  (unsigned char)(((aw869xx->ram.base_addr -
					    (aw869xx->
					     ram.base_addr >> 2)) & 0x00FF)));
/*
*	unsigned int temp
*	HIGH<byte4 byte3 byte2 byte1>LOW
*	|_ _ _ _AF-12BIT_ _ _ _AE-12BIT|
*/
	aw869xx_i2c_read(aw869xx, AW869XX_REG_RTPCFG3, &reg_val);
	temp = ((reg_val & 0x0f) << 24) | ((reg_val & 0xf0) << 4);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_RTPCFG4, &reg_val);
	temp = temp | reg_val;
	aw_dev_info(aw869xx->dev, "%s: almost_empty_threshold = %d\n", __func__,
		    (unsigned short)temp);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_RTPCFG5, &reg_val);
	temp = temp | (reg_val << 16);
	aw_dev_info(aw869xx->dev, "%s: almost_full_threshold = %d\n", __func__,
		    temp >> 16);
	/* ram */
	shift = aw869xx->ram.baseaddr_shift;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RAMADDRH,
			       AW869XX_BIT_RAMADDRH_MASK,
			       aw869xx_cont->data[0 + shift]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRL,
			  aw869xx_cont->data[1 + shift]);
	shift = aw869xx->ram.ram_shift;
	for (i = shift; i < aw869xx_cont->len; i++) {
		aw869xx->ram_update_flag = aw869xx_i2c_write(aw869xx,
							     AW869XX_REG_RAMDATA,
							     aw869xx_cont->data
							     [i]);
	}
#ifdef AW_CHECK_RAM_DATA
	shift = aw869xx->ram.baseaddr_shift;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RAMADDRH,
			       AW869XX_BIT_RAMADDRH_MASK,
			       aw869xx_cont->data[0 + shift]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRL,
			  aw869xx_cont->data[1 + shift]);
	shift = aw869xx->ram.ram_shift;
	for (i = shift; i < aw869xx_cont->len; i++) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_RAMDATA, &reg_val);
		/*
		   aw_dev_info(aw869xx->dev, "%s aw869xx_cont->data=0x%02X, ramdata=0x%02X\n",
		   __func__,aw869xx_cont->data[i],reg_val);
		 */
		if (reg_val != aw869xx_cont->data[i]) {
			aw_dev_err(aw869xx->dev,
				   "%s: ram check error addr=0x%04x, file_data=0x%02X, ram_data=0x%02X\n",
				   __func__, i, aw869xx_cont->data[i], reg_val);
			return;
		}
	}
#endif
	/* RAMINIT Disable */
	aw869xx_haptic_raminit(aw869xx, false);
	mutex_unlock(&aw869xx->lock);
	aw_dev_info(aw869xx->dev, "%s exit\n", __func__);
}

static void aw869xx_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw869xx *aw869xx = context;
	struct aw869xx_container *aw869xx_fw;
	int i = 0;
	unsigned short check_sum = 0;
#ifdef AW_READ_BIN_FLEXBALLY
	static unsigned char load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	if (!cont) {
		aw_dev_err(aw869xx->dev, "%s: failed to read %s\n", __func__,
			   aw869xx_ram_name);
		release_firmware(cont);
#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw869xx->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			aw_dev_info(aw869xx->dev,
				    "%s:start hrtimer: load_cont=%d\n",
				    __func__, load_cont);
		}
#endif
		return;
	}
	aw_dev_info(aw869xx->dev, "%s: loaded %s - size: %zu bytes\n", __func__,
		    aw869xx_ram_name, cont ? cont->size : 0);
/*
	for(i=0; i < cont->size; i++) {
		aw_dev_info(aw869xx->dev, "%s: addr: 0x%04x, data: 0x%02X\n",
			__func__, i, *(cont->data+i));
	}
*/
	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum !=
	    (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		aw_dev_err(aw869xx->dev,
			   "%s: check sum err: check_sum=0x%04x\n", __func__,
			   check_sum);
		return;
	} else {
		aw_dev_info(aw869xx->dev, "%s: check sum pass: 0x%04x\n",
			    __func__, check_sum);
		aw869xx->ram.check_sum = check_sum;
	}

	/* aw869xx ram update less then 128kB*/
	aw869xx_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw869xx_fw) {
		release_firmware(cont);
		aw_dev_err(aw869xx->dev, "%s: Error allocating memory\n",
			   __func__);
		return;
	}
	aw869xx_fw->len = cont->size;
	memcpy(aw869xx_fw->data, cont->data, cont->size);
	release_firmware(cont);
	aw869xx_container_update(aw869xx, aw869xx_fw);
	aw869xx->ram.len = aw869xx_fw->len;
	kfree(aw869xx_fw);
	aw869xx->ram_init = 1;
	aw_dev_info(aw869xx->dev, "%s: ram firmware update complete!\n",
		    __func__);
}

static int aw869xx_ram_update(struct aw869xx *aw869xx)
{
	aw869xx->ram_init = 0;
	aw869xx->rtp_init = 0;

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw869xx_ram_name, aw869xx->dev,
				       GFP_KERNEL, aw869xx, aw869xx_ram_loaded);
}

static void aw869xx_ram_work_routine(struct work_struct *work)
{
	struct aw869xx *aw869xx = container_of(work, struct aw869xx,
					       ram_work.work);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx_ram_update(aw869xx);
}

static int aw869xx_ram_work_init(struct aw869xx *aw869xx)
{
	int ram_timer_val = 8000;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	INIT_DELAYED_WORK(&aw869xx->ram_work, aw869xx_ram_work_routine);
	schedule_delayed_work(&aw869xx->ram_work,
			      msecs_to_jiffies(ram_timer_val));
	return 0;
}

/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
/* static int aw869xx_haptic_softreset(struct aw869xx *aw869xx)
*{
*	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
*	aw869xx_i2c_write(aw869xx, AW869XX_REG_ID, 0xAA);
*	usleep_range(3000, 3500);
*	return 0;
*}
*/

static int aw869xx_haptic_play_mode(struct aw869xx *aw869xx,
				    unsigned char play_mode)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	switch (play_mode) {
	case AW869XX_HAPTIC_STANDBY_MODE:
		aw_dev_info(aw869xx->dev, "%s: enter standby mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_STANDBY_MODE;
		aw869xx_haptic_stop(aw869xx);
		break;
	case AW869XX_HAPTIC_RAM_MODE:
		aw_dev_info(aw869xx->dev, "%s: enter ram mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_RAM_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW869XX_HAPTIC_RAM_LOOP_MODE:
		aw_dev_info(aw869xx->dev, "%s: enter ram loop mode\n",
			    __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_RAM_LOOP_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW869XX_HAPTIC_RTP_MODE:
		aw_dev_info(aw869xx->dev, "%s: enter rtp mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_RTP_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RTP);
		break;
	case AW869XX_HAPTIC_TRIG_MODE:
		aw_dev_info(aw869xx->dev, "%s: enter trig mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_TRIG_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW869XX_HAPTIC_CONT_MODE:
		aw_dev_info(aw869xx->dev, "%s: enter cont mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_CONT_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_CONT);
		break;
	default:
		aw_dev_err(aw869xx->dev, "%s: play mode %d error",
			   __func__, play_mode);
		break;
	}
	return 0;
}

static int aw869xx_haptic_set_wav_seq(struct aw869xx *aw869xx,
				      unsigned char wav, unsigned char seq)
{
	aw869xx_i2c_write(aw869xx, AW869XX_REG_WAVCFG1 + wav, seq);
	return 0;
}

static int aw869xx_haptic_set_wav_loop(struct aw869xx *aw869xx,
				       unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG9 + (wav / 2),
				       AW869XX_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG9 + (wav / 2),
				       AW869XX_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
	return 0;
}

/*
static int aw869xx_haptic_set_main_loop(struct aw869xx *aw869xx,
				       unsigned char loop)
{
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG13,
				AW869XX_BIT_WAVCFG13_MAINLOOP_MASK, loop);
	return 0;
}
*/

static int aw869xx_haptic_set_repeat_wav_seq(struct aw869xx *aw869xx,
					     unsigned char seq)
{
	aw869xx_haptic_set_wav_seq(aw869xx, 0x00, seq);
	aw869xx_haptic_set_wav_loop(aw869xx, 0x00,
				    AW869XX_BIT_WAVLOOP_INIFINITELY);
	return 0;
}

static int aw869xx_haptic_set_bst_vol(struct aw869xx *aw869xx,
				      unsigned char bst_vol)
{
	if (bst_vol & 0xc0)
		bst_vol = 0x3f;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG1,
			       AW869XX_BIT_PLAYCFG1_BST_VOUT_RDA_MASK, bst_vol);
	return 0;
}

static int aw869xx_haptic_set_bst_peak_cur(struct aw869xx *aw869xx)
{
	switch (aw869xx->bst_pc) {
	case AW869XX_HAPTIC_BST_PC_L1:
		aw_dev_info(aw869xx->dev, "%s bst pc = L1\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG1,
				       AW869XX_BIT_BSTCFG1_BST_PC_MASK,
				       (0 << 1));
		return 0;
	case AW869XX_HAPTIC_BST_PC_L2:
		aw_dev_info(aw869xx->dev, "%s bst pc = L2\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG1,
				       AW869XX_BIT_BSTCFG1_BST_PC_MASK,
				       (5 << 1));
		return 0;
	default:
		aw_dev_info(aw869xx->dev, "%s bst pc = L1\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG1,
				       AW869XX_BIT_BSTCFG1_BST_PC_MASK,
				       (0 << 1));
		break;
	}
	return 0;
}

static int aw869xx_haptic_set_gain(struct aw869xx *aw869xx, unsigned char gain)
{
	aw869xx_i2c_write(aw869xx, AW869XX_REG_PLAYCFG2, gain);
	return 0;
}

static int aw869xx_haptic_set_pwm(struct aw869xx *aw869xx, unsigned char mode)
{
	switch (mode) {
	case AW869XX_PWM_48K:
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW869XX_PWM_24K:
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW869XX_PWM_12K:
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw869xx_haptic_play_wav_seq(struct aw869xx *aw869xx,
				       unsigned char flag)
{
#ifdef AW_RAM_STATE_OUTPUT
	int i;
	unsigned char reg_val = 0;
#endif

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	if (flag) {
		aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_MODE);
		aw869xx_haptic_play_go(aw869xx);
#ifdef AW_RAM_STATE_OUTPUT
		for (i = 0; i < 100; i++) {
			aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
			if ((reg_val & 0x0f) == 0x07) {
				aw_dev_info(aw869xx->dev,
					    "%s RAM_GO! glb_state=0x07\n",
					    __func__);
			} else {
				aw_dev_dbg(aw869xx->dev,
					   "%s ram stopped, glb_state=0x%02X\n",
					   __func__, reg_val);
			}
			usleep_range(2000, 2500);

		}
#endif
	}
	return 0;
}

static int aw869xx_haptic_play_repeat_seq(struct aw869xx *aw869xx,
					  unsigned char flag)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	if (flag) {
		aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_LOOP_MODE);
		aw869xx_haptic_play_go(aw869xx);
	}
	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw869xx_haptic_swicth_motor_protect_config(struct aw869xx *aw869xx,
						      unsigned char addr,
						      unsigned char val)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	if (addr == 1) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG1,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_MASK,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_VALID);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG1,
				       AW869XX_BIT_PWMCFG1_PRC_EN_MASK,
				       AW869XX_BIT_PWMCFG1_PRC_ENABLE);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG3,
				       AW869XX_BIT_PWMCFG3_PR_EN_MASK,
				       AW869XX_BIT_PWMCFG3_PR_ENABLE);
	} else if (addr == 0) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG1,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_MASK,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_INVALID);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG1,
				       AW869XX_BIT_PWMCFG1_PRC_EN_MASK,
				       AW869XX_BIT_PWMCFG1_PRC_DISABLE);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG3,
				       AW869XX_BIT_PWMCFG3_PR_EN_MASK,
				       AW869XX_BIT_PWMCFG3_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG1,
				       AW869XX_BIT_PWMCFG1_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG3,
				       AW869XX_BIT_PWMCFG3_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_PWMCFG4, val);
	}
	return 0;
}

/*****************************************************
 *
 * offset calibration
 *
 *****************************************************/
static int aw869xx_haptic_offset_calibration(struct aw869xx *aw869xx)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	aw869xx_haptic_raminit(aw869xx, true);

	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG2,
			       AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
			       AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	while (1) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_DETCFG2, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	if (cont == 0)
		aw_dev_err(aw869xx->dev, "%s calibration offset failed!\n",
			   __func__);
	aw869xx_haptic_raminit(aw869xx, false);
	return 0;
}

/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw869xx_haptic_trig_param_init(struct aw869xx *aw869xx)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	aw869xx->trig[0].trig_level = aw869xx->dts_info.trig_config[0];
	aw869xx->trig[0].trig_polar = aw869xx->dts_info.trig_config[1];
	aw869xx->trig[0].pos_enable = aw869xx->dts_info.trig_config[2];
	aw869xx->trig[0].pos_sequence = aw869xx->dts_info.trig_config[3];
	aw869xx->trig[0].neg_enable = aw869xx->dts_info.trig_config[4];
	aw869xx->trig[0].neg_sequence = aw869xx->dts_info.trig_config[5];
	aw869xx->trig[0].trig_brk = aw869xx->dts_info.trig_config[6];
	aw869xx->trig[0].trig_bst = aw869xx->dts_info.trig_config[7];
	if (!aw869xx->dts_info.is_enabled_i2s) {
		aw_dev_info(aw869xx->dev, "%s i2s is disabled!\n", __func__);
		aw869xx->trig[1].trig_level =
		    aw869xx->dts_info.trig_config[8 + 0];
		aw869xx->trig[1].trig_polar =
		    aw869xx->dts_info.trig_config[8 + 1];
		aw869xx->trig[1].pos_enable =
		    aw869xx->dts_info.trig_config[8 + 2];
		aw869xx->trig[1].pos_sequence =
		    aw869xx->dts_info.trig_config[8 + 3];
		aw869xx->trig[1].neg_enable =
		    aw869xx->dts_info.trig_config[8 + 4];
		aw869xx->trig[1].neg_sequence =
		    aw869xx->dts_info.trig_config[8 + 5];
		aw869xx->trig[1].trig_brk =
		    aw869xx->dts_info.trig_config[8 + 6];
		aw869xx->trig[1].trig_bst =
		    aw869xx->dts_info.trig_config[8 + 7];

		aw869xx->trig[2].trig_level =
		    aw869xx->dts_info.trig_config[16 + 0];
		aw869xx->trig[2].trig_polar =
		    aw869xx->dts_info.trig_config[16 + 1];
		aw869xx->trig[2].pos_enable =
		    aw869xx->dts_info.trig_config[16 + 2];
		aw869xx->trig[2].pos_sequence =
		    aw869xx->dts_info.trig_config[16 + 3];
		aw869xx->trig[2].neg_enable =
		    aw869xx->dts_info.trig_config[16 + 4];
		aw869xx->trig[2].neg_sequence =
		    aw869xx->dts_info.trig_config[16 + 5];
		aw869xx->trig[2].trig_brk =
		    aw869xx->dts_info.trig_config[16 + 6];
		aw869xx->trig[2].trig_bst =
		    aw869xx->dts_info.trig_config[16 + 7];
	}
	return 0;
}

static int aw869xx_haptic_trig_param_config(struct aw869xx *aw869xx)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	if (aw869xx->trig[0].trig_level) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_LEVEL);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_EDGE);
	}
	if (aw869xx->trig[1].trig_level) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_LEVEL);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_EDGE);
	}
	if (aw869xx->trig[2].trig_level) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_LEVEL);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_EDGE);
	}

	if (aw869xx->trig[0].trig_polar) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_NEG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_POS);
	}
	if (aw869xx->trig[1].trig_polar) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_NEG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_POS);
	}
	if (aw869xx->trig[2].trig_polar) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_NEG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_POS);
	}

	if (aw869xx->trig[0].pos_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG1,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG1,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[1].pos_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG2,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG2,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[2].pos_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG3,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG3,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}

	if (aw869xx->trig[0].neg_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG4,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG4,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[1].neg_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG5,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG5,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[2].neg_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG6,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG6,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[0].pos_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG1,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[0].pos_sequence);
	}
	if (aw869xx->trig[0].neg_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG4,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[0].neg_sequence);
	}
	if (aw869xx->trig[1].pos_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG2,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[1].pos_sequence);
	}
	if (aw869xx->trig[1].neg_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG5,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[1].neg_sequence);
	}
	if (aw869xx->trig[2].pos_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG3,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[2].pos_sequence);
	}
	if (aw869xx->trig[2].neg_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG6,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[2].neg_sequence);
	}
	if (aw869xx->trig[0].trig_brk) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE);
	}
	if (aw869xx->trig[1].trig_brk) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE);
	}
	if (aw869xx->trig[2].trig_brk) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE);
	}
	if (aw869xx->trig[0].trig_bst) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_DISABLE);
	}
	if (aw869xx->trig[1].trig_bst) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_DISABLE);
	}
	if (aw869xx->trig[2].trig_bst) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_DISABLE);
	}
	return 0;
}

static void aw869xx_haptic_bst_mode_config(struct aw869xx *aw869xx,
					   unsigned char boost_mode)
{
	aw869xx->boost_mode = boost_mode;

	switch (boost_mode) {
	case AW869XX_HAPTIC_BST_MODE_BOOST:
		aw_dev_info(aw869xx->dev, "%s haptic boost mode = boost\n",
			    __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG1,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_BOOST);
		break;
	case AW869XX_HAPTIC_BST_MODE_BYPASS:
		aw_dev_info(aw869xx->dev, "%s haptic boost mode = bypass\n",
			    __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG1,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		aw_dev_err(aw869xx->dev, "%s: boost_mode = %d error",
			   __func__, boost_mode);
		break;
	}
}

static int aw869xx_haptic_auto_bst_enable(struct aw869xx *aw869xx,
					  unsigned char flag)
{
	aw869xx->auto_boost = flag;
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
	return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw869xx_haptic_vbat_mode_config(struct aw869xx *aw869xx,
					   unsigned char flag)
{
	if (flag == AW869XX_HAPTIC_CONT_VBAT_HW_ADJUST_MODE) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
	return 0;
}

static int aw869xx_haptic_get_vbat(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;
	unsigned int vbat_code = 0;

	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_raminit(aw869xx, true);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG2,
			       AW869XX_BIT_DETCFG2_VBAT_GO_MASK,
			       AW869XX_BIT_DETCFG2_VABT_GO_ON);
	usleep_range(20000, 25000);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_VBAT, &reg_val);
	vbat_code = (vbat_code | reg_val) << 2;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_LO, &reg_val);
	vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
	aw869xx->vbat = 6100 * vbat_code / 1024;
	if (aw869xx->vbat > AW869XX_VBAT_MAX) {
		aw869xx->vbat = AW869XX_VBAT_MAX;
		aw_dev_info(aw869xx->dev, "%s vbat max limit = %dmV\n",
			    __func__, aw869xx->vbat);
	}
	if (aw869xx->vbat < AW869XX_VBAT_MIN) {
		aw869xx->vbat = AW869XX_VBAT_MIN;
		aw_dev_info(aw869xx->dev, "%s vbat min limit = %dmV\n",
			    __func__, aw869xx->vbat);
	}
	aw_dev_info(aw869xx->dev, "%s aw869xx->vbat=%dmV, vbat_code=0x%02X\n",
		    __func__, aw869xx->vbat, vbat_code);
	aw869xx_haptic_raminit(aw869xx, false);
	return 0;
}

static int aw869xx_haptic_ram_vbat_compensate(struct aw869xx *aw869xx,
					      bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if (aw869xx->ram_vbat_compensate ==
		    AW869XX_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw869xx_haptic_get_vbat(aw869xx);
			temp_gain =
			    aw869xx->gain * AW869XX_VBAT_REFER / aw869xx->vbat;
			if (temp_gain >
			    (128 * AW869XX_VBAT_REFER / AW869XX_VBAT_MIN)) {
				temp_gain =
				    128 * AW869XX_VBAT_REFER / AW869XX_VBAT_MIN;
				aw_dev_dbg(aw869xx->dev, "%s gain limit=%d\n",
					   __func__, temp_gain);
			}
			aw869xx_haptic_set_gain(aw869xx, temp_gain);
		} else {
			aw869xx_haptic_set_gain(aw869xx, aw869xx->gain);
		}
	} else {
		aw869xx_haptic_set_gain(aw869xx, aw869xx->gain);
	}
	return 0;
}

static int aw869xx_haptic_get_lra_resistance(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;
	unsigned char d2s_gain_temp = 0;
	unsigned int lra_code = 0;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       aw869xx->dts_info.d2s_gain);
	aw869xx_haptic_raminit(aw869xx, true);
	/* enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	usleep_range(2000, 2500);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
			       AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
			       AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG1,
			       AW869XX_BIT_DETCFG1_RL_OS_MASK,
			       AW869XX_BIT_DETCFG1_RL);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG2,
			       AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
			       AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(30000, 35000);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_RL, &reg_val);
	lra_code = (lra_code | reg_val) << 2;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_LO, &reg_val);
	lra_code = lra_code | (reg_val & 0x03);
	/* 2num */
	aw869xx->lra = (lra_code * 678 * 100) / (1024 * 10);
	aw869xx_haptic_raminit(aw869xx, false);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       d2s_gain_temp);
	mutex_unlock(&aw869xx->lock);
	return 0;
}

static void aw869xx_haptic_misc_para_init(struct aw869xx *aw869xx)
{

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->cont_drv1_lvl = aw869xx->dts_info.cont_drv1_lvl_dt;
	aw869xx->cont_drv2_lvl = aw869xx->dts_info.cont_drv2_lvl_dt;
	aw869xx->cont_drv1_time = aw869xx->dts_info.cont_drv1_time_dt;
	aw869xx->cont_drv2_time = aw869xx->dts_info.cont_drv2_time_dt;
	aw869xx->cont_brk_time = aw869xx->dts_info.cont_brk_time_dt;
	aw869xx->cont_wait_num = aw869xx->dts_info.cont_wait_num_dt;
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG1,
			  aw869xx->dts_info.bstcfg[0]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG2,
			  aw869xx->dts_info.bstcfg[1]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG3,
			  aw869xx->dts_info.bstcfg[2]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG4,
			  aw869xx->dts_info.bstcfg[3]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG5,
			  aw869xx->dts_info.bstcfg[4]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL3,
			  aw869xx->dts_info.sine_array[0]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL4,
			  aw869xx->dts_info.sine_array[1]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL5,
			  aw869xx->dts_info.sine_array[2]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL6,
			  aw869xx->dts_info.sine_array[3]);
	/* d2s_gain */
	if (!aw869xx->dts_info.d2s_gain) {
		aw_dev_err(aw869xx->dev, "%s aw869xx->dts_info.d2s_gain = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
				       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
				       aw869xx->dts_info.d2s_gain);
	}

	/* cont_tset */
	if (!aw869xx->dts_info.cont_tset) {
		aw_dev_err(aw869xx->dev,
			   "%s aw869xx->dts_info.cont_tset = 0!\n", __func__);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG13,
				       AW869XX_BIT_CONTCFG13_TSET_MASK,
				       aw869xx->dts_info.cont_tset << 4);
	}

	/* cont_bemf_set */
	if (!aw869xx->dts_info.cont_bemf_set) {
		aw_dev_err(aw869xx->dev,
			   "%s aw869xx->dts_info.cont_bemf_set = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG13,
				       AW869XX_BIT_CONTCFG13_BEME_SET_MASK,
				       aw869xx->dts_info.cont_bemf_set);
	}

	/* cont_brk_time */
	if (!aw869xx->cont_brk_time) {
		aw_dev_err(aw869xx->dev, "%s aw869xx->cont_brk_time = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG10,
				  aw869xx->cont_brk_time);
	}

	/* cont_bst_brk_gain */
	if (!aw869xx->dts_info.cont_bst_brk_gain) {
		aw_dev_err(aw869xx->dev,
			   "%s aw869xx->dts_info.cont_bst_brk_gain = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG5,
				       AW869XX_BIT_CONTCFG5_BST_BRK_GAIN_MASK,
				       aw869xx->dts_info.cont_bst_brk_gain);
	}

	/* cont_brk_gain */
	if (!aw869xx->dts_info.cont_brk_gain) {
		aw_dev_err(aw869xx->dev,
			   "%s aw869xx->dts_info.cont_brk_gain = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG5,
				       AW869XX_BIT_CONTCFG5_BRK_GAIN_MASK,
				       aw869xx->dts_info.cont_brk_gain);
	}

	/* i2s enbale */
	if (aw869xx->dts_info.is_enabled_i2s) {
		aw_dev_info(aw869xx->dev, "%s i2s enabled!\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	} else {
		aw_dev_info(aw869xx->dev, "%s i2s disabled!\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_TRIG);
	}
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw869xx_haptic_set_rtp_aei(struct aw869xx *aw869xx, bool flag)
{
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
				       AW869XX_BIT_SYSINTM_FF_AEM_MASK,
				       AW869XX_BIT_SYSINTM_FF_AEM_ON);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
				       AW869XX_BIT_SYSINTM_FF_AEM_MASK,
				       AW869XX_BIT_SYSINTM_FF_AEM_OFF);
	}
}

/*
static void aw869xx_haptic_set_rtp_afi(struct aw869xx *aw869xx, bool flag)
{
	if(flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
					AW869XX_BIT_SYSINTM_FF_AFM_MASK,
					AW869XX_BIT_SYSINTM_FF_AFM_ON);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
					AW869XX_BIT_SYSINTM_FF_AFM_MASK,
					AW869XX_BIT_SYSINTM_FF_AFM_OFF);
	}
}
*/

/*
static unsigned char aw869xx_haptic_rtp_get_fifo_aei(struct aw869xx *aw869xx)
{
	unsigned char ret;
	unsigned char reg_val;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	reg_val &= AW869XX_BIT_SYSINT_FF_AEI;
	ret = reg_val>>4;

	return ret;
}
*/

/*
static unsigned char aw869xx_haptic_rtp_get_fifo_aes(struct aw869xx *aw869xx)
{
	unsigned char ret;
	unsigned char reg_val;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
	reg_val &= AW869XX_BIT_SYSST_FF_AES;
	ret = reg_val>>4;

	return ret;
}

static unsigned char aw869xx_haptic_rtp_get_fifo_afi(struct aw869xx *aw869xx)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	reg_val &= AW869XX_BIT_SYSINT_FF_AFI;
	ret = reg_val >> 3;
	return ret;
}
*/

static unsigned char aw869xx_haptic_rtp_get_fifo_afs(struct aw869xx *aw869xx)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
	reg_val &= AW869XX_BIT_SYSST_FF_AFS;
	ret = reg_val >> 3;
	return ret;
}

static int aw869xx_haptic_rtp_init(struct aw869xx *aw869xx)
{
	unsigned int buf_len = 0;
	unsigned char glb_state_val = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_VALUE_VB);
	aw869xx->rtp_cnt = 0;
	mutex_lock(&aw869xx->rtp_lock);
	while ((!aw869xx_haptic_rtp_get_fifo_afs(aw869xx))
	       && (aw869xx->play_mode == AW869XX_HAPTIC_RTP_MODE)) {
		aw_dev_info(aw869xx->dev, "%s rtp cnt = %d\n", __func__,
			    aw869xx->rtp_cnt);
		if (!aw869xx_rtp) {
			aw_dev_info(aw869xx->dev,
				    "%s:aw869xx_rtp is null, break!\n",
				    __func__);
			break;
		}
		if (aw869xx->rtp_cnt < (aw869xx->ram.base_addr)) {
			if ((aw869xx_rtp->len - aw869xx->rtp_cnt) <
			    (aw869xx->ram.base_addr)) {
				buf_len = aw869xx_rtp->len - aw869xx->rtp_cnt;
			} else {
				buf_len = aw869xx->ram.base_addr;
			}
		} else if ((aw869xx_rtp->len - aw869xx->rtp_cnt) <
			   (aw869xx->ram.base_addr >> 2)) {
			buf_len = aw869xx_rtp->len - aw869xx->rtp_cnt;
		} else {
			buf_len = aw869xx->ram.base_addr >> 2;
		}
		aw_dev_info(aw869xx->dev, "%s buf_len = %d\n", __func__,
			    buf_len);
		aw869xx_i2c_writes(aw869xx, AW869XX_REG_RTPDATA,
				   &aw869xx_rtp->data[aw869xx->rtp_cnt],
				   buf_len);
		aw869xx->rtp_cnt += buf_len;
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &glb_state_val);
		if ((aw869xx->rtp_cnt == aw869xx_rtp->len)
		    || ((glb_state_val & 0x0f) == 0x00)) {
			aw_dev_info(aw869xx->dev, "%s: rtp update complete!\n",
				    __func__);
			aw869xx->rtp_cnt = 0;
			pm_qos_remove_request(&pm_qos_req_vb);
			mutex_unlock(&aw869xx->rtp_lock);
			return 0;
		}
	}
	mutex_unlock(&aw869xx->rtp_lock);

	if (aw869xx->play_mode == AW869XX_HAPTIC_RTP_MODE)
		aw869xx_haptic_set_rtp_aei(aw869xx, true);

	aw_dev_info(aw869xx->dev, "%s exit\n", __func__);
	pm_qos_remove_request(&pm_qos_req_vb);
	return 0;
}

static void aw869xx_haptic_upload_lra(struct aw869xx *aw869xx,
				      unsigned int flag)
{
	switch (flag) {
	case WRITE_ZERO:
		aw_dev_info(aw869xx->dev, "%s write zero to trim_lra!\n",
			    __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       0x00);
		break;
	case F0_CALI:
		aw_dev_info(aw869xx->dev,
			    "%s write f0_cali_data to trim_lra = 0x%02X\n",
			    __func__, aw869xx->f0_cali_data);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       (char)aw869xx->f0_cali_data);
		break;
	case OSC_CALI:
		aw_dev_info(aw869xx->dev,
			    "%s write osc_cali_data to trim_lra = 0x%02X\n",
			    __func__, aw869xx->osc_cali_data);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       (char)aw869xx->osc_cali_data);
		break;
	default:
		break;
	}
}

static int aw869xx_osc_trim_calculation(unsigned long int theory_time,
					unsigned long int real_time)
{
	unsigned int real_code = 0;
	unsigned int lra_code = 0;
	unsigned int DFT_LRA_TRIM_CODE = 0;
	/*0.1 percent below no need to calibrate */
	unsigned int osc_cali_threshold = 10;

	pr_info("%s enter\n", __func__);
	if (theory_time == real_time) {
		pr_info("%s theory_time == real_time: %ld,"
			" no need to calibrate!\n", __func__, real_time);
		return 0;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) > (theory_time / 50)) {
			pr_info("%s (real_time - theory_time) >"
				" (theory_time/50), can't calibrate!\n",
				__func__);
			return DFT_LRA_TRIM_CODE;
		}

		if ((real_time - theory_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			pr_info("%s real_time: %ld, theory_time: %ld,"
				" no need to calibrate!\n", __func__,
				real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}

		real_code = ((real_time - theory_time) * 4000) / theory_time;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = 32 + real_code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) > (theory_time / 50)) {
			pr_info("%s (theory_time - real_time) >"
				" (theory_time / 50), can't calibrate!\n",
				__func__);
			return DFT_LRA_TRIM_CODE;
		}
		if ((theory_time - real_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			pr_info("%s real_time: %ld, theory_time: %ld,"
				" no need to calibrate!\n", __func__,
				real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}
		real_code = ((theory_time - real_time) * 4000) / theory_time;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = 32 - real_code;
	}
	if (real_code > 31)
		lra_code = real_code - 32;
	else
		lra_code = real_code + 32;
	pr_info("%s real_time: %ld, theory_time: %ld\n",__func__, real_time,
		theory_time);
	pr_info("%s real_code: %02X, trim_lra: 0x%02X\n", __func__, real_code,
		lra_code);
	return lra_code;
}

static int aw869xx_rtp_trim_lra_calibration(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;
	unsigned int fre_val = 0;
	unsigned int theory_time = 0;
	unsigned int lra_trim_code = 0;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL2, &reg_val);
	fre_val = (reg_val & 0x03) >> 0;

	if (fre_val == 2 || fre_val == 3)
		theory_time = (aw869xx->rtp_len / 12000) * 1000000;	/*12K */
	if (fre_val == 0)
		theory_time = (aw869xx->rtp_len / 24000) * 1000000;	/*24K */
	if (fre_val == 1)
		theory_time = (aw869xx->rtp_len / 48000) * 1000000;	/*48K */

	aw_dev_info(aw869xx->dev, "%s microsecond:%ld  theory_time = %d\n",
		    __func__, aw869xx->microsecond, theory_time);

	lra_trim_code = aw869xx_osc_trim_calculation(theory_time,
						     aw869xx->microsecond);
	if (lra_trim_code >= 0) {
		aw869xx->osc_cali_data = lra_trim_code;
		aw869xx_haptic_upload_lra(aw869xx, OSC_CALI);
	}
	return 0;
}

static unsigned char aw869xx_haptic_osc_read_status(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST2, &reg_val);
	return reg_val;
}

static int aw869xx_rtp_osc_calibration(struct aw869xx *aw869xx)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char osc_int_state = 0;

	aw869xx->rtp_cnt = 0;
	aw869xx->timeval_flags = 1;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	/* fw loaded */
	ret = request_firmware(&rtp_file, aw869xx_rtp_name[0], aw869xx->dev);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s: failed to read %s\n", __func__,
			   aw869xx_rtp_name[0]);
		return ret;
	}
	/*awinic add stop,for irq interrupt during calibrate */
	aw869xx_haptic_stop(aw869xx);
	aw869xx->rtp_init = 0;
	mutex_lock(&aw869xx->rtp_lock);
	vfree(aw869xx_rtp);
	aw869xx_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw869xx_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw869xx->rtp_lock);
		aw_dev_err(aw869xx->dev, "%s: error allocating memory\n",
			   __func__);
		return -ENOMEM;
	}
	aw869xx_rtp->len = rtp_file->size;
	aw869xx->rtp_len = rtp_file->size;
	aw_dev_info(aw869xx->dev, "%s: rtp file:[%s] size = %dbytes\n", __func__,
		    aw869xx_rtp_name[0], aw869xx_rtp->len);

	memcpy(aw869xx_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&aw869xx->rtp_lock);
	/* gain */
	aw869xx_haptic_ram_vbat_compensate(aw869xx, false);
	/* rtp mode config */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RTP_MODE);
	/* bst mode */
	aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_INT_MODE_MASK,
			       AW869XX_BIT_SYSCTRL7_INT_MODE_EDGE);
	disable_irq(gpio_to_irq(aw869xx->irq_gpio));
#ifdef AW_OSC_COARSE_CALI
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
			       AW869XX_BIT_TRIMCFG3_OSC_TRIM_SRC_MASK,
			       AW869XX_BIT_TRIMCFG3_OSC_TRIM_SRC_REG);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIMCFG4, 0xF4);
#endif
	/* haptic go */
	aw869xx_haptic_play_go(aw869xx);
	/* require latency of CPU & DMA not more then PM_QOS_VALUE_VB us */
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_VALUE_VB);
	while (1) {
		if (!aw869xx_haptic_rtp_get_fifo_afs(aw869xx)) {
			aw_dev_info(aw869xx->dev, "%s not almost_full,"
				    " aw869xx->rtp_cnt= %d\n", __func__,
				    aw869xx->rtp_cnt);
			mutex_lock(&aw869xx->rtp_lock);
			if ((aw869xx_rtp->len - aw869xx->rtp_cnt) <
			    (aw869xx->ram.base_addr >> 2))
				buf_len = aw869xx_rtp->len - aw869xx->rtp_cnt;
			else
				buf_len = (aw869xx->ram.base_addr >> 2);

			if (aw869xx->rtp_cnt != aw869xx_rtp->len) {
				if (aw869xx->timeval_flags == 1) {
					do_gettimeofday(&aw869xx->start);
					aw869xx->timeval_flags = 0;
				}
				aw869xx->rtp_update_flag =
				    aw869xx_i2c_writes(aw869xx,
						       AW869XX_REG_RTPDATA,
						       &aw869xx_rtp->data
						       [aw869xx->rtp_cnt],
						       buf_len);
				aw869xx->rtp_cnt += buf_len;
			}
			mutex_unlock(&aw869xx->rtp_lock);
		}
		osc_int_state = aw869xx_haptic_osc_read_status(aw869xx);
		if (osc_int_state & AW869XX_BIT_SYSST2_FF_EMPTY) {
			do_gettimeofday(&aw869xx->end);
			pr_info
			    ("%s osc trim playback done aw869xx->rtp_cnt= %d\n",
			     __func__, aw869xx->rtp_cnt);
			break;
		}
		do_gettimeofday(&aw869xx->end);
		aw869xx->microsecond =
		    (aw869xx->end.tv_sec - aw869xx->start.tv_sec) * 1000000 +
		    (aw869xx->end.tv_usec - aw869xx->start.tv_usec);
		if (aw869xx->microsecond > OSC_CALI_MAX_LENGTH) {
			aw_dev_info(aw869xx->dev, "%s osc trim time out!"
				    " aw869xx->rtp_cnt %d osc_int_state %02x\n",
				    __func__, aw869xx->rtp_cnt, osc_int_state);
			break;
		}
	}
	pm_qos_remove_request(&pm_qos_req_vb);
	enable_irq(gpio_to_irq(aw869xx->irq_gpio));

	aw869xx->microsecond =
	    (aw869xx->end.tv_sec - aw869xx->start.tv_sec) * 1000000 +
	    (aw869xx->end.tv_usec - aw869xx->start.tv_usec);
	/*calibration osc */
	aw_dev_info(aw869xx->dev, "%s awinic_microsecond: %ld\n", __func__,
		    aw869xx->microsecond);
	aw_dev_info(aw869xx->dev, "%s exit\n", __func__);
	return 0;
}

static void aw869xx_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int cnt = 200;
	unsigned char reg_val = 0;
	bool rtp_work_flag = false;
	struct aw869xx *aw869xx = container_of(work, struct aw869xx, rtp_work);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	mutex_lock(&aw869xx->rtp_lock);
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			       aw869xx_rtp_name[aw869xx->rtp_file_num],
			       aw869xx->dev);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s: failed to read %s\n", __func__,
			   aw869xx_rtp_name[aw869xx->rtp_file_num]);
		mutex_unlock(&aw869xx->rtp_lock);
		return;
	}
	aw869xx->rtp_init = 0;
	vfree(aw869xx_rtp);
	aw869xx_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw869xx_rtp) {
		release_firmware(rtp_file);
		aw_dev_err(aw869xx->dev, "%s: error allocating memory\n",
			   __func__);
		mutex_unlock(&aw869xx->rtp_lock);
		return;
	}
	aw869xx_rtp->len = rtp_file->size;
	aw_dev_info(aw869xx->dev, "%s: rtp file:[%s] size = %dbytes\n", __func__,
		    aw869xx_rtp_name[aw869xx->rtp_file_num], aw869xx_rtp->len);
	memcpy(aw869xx_rtp->data, rtp_file->data, rtp_file->size);
	mutex_unlock(&aw869xx->rtp_lock);
	release_firmware(rtp_file);
	mutex_lock(&aw869xx->lock);
	aw869xx->rtp_init = 1;
	aw869xx_haptic_upload_lra(aw869xx, OSC_CALI);
	/* gain */
	aw869xx_haptic_ram_vbat_compensate(aw869xx, false);
	/* rtp mode config */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RTP_MODE);
	/* bst mode config */
	aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	/* haptic go */
	aw869xx_haptic_play_go(aw869xx);
	mutex_unlock(&aw869xx->lock);
	usleep_range(2000, 2500);
	while (cnt) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x08) {
			cnt = 0;
			rtp_work_flag = true;
			aw_dev_info(aw869xx->dev, "%s RTP_GO! glb_state=0x08\n",
				    __func__);
		} else {
			cnt--;
			aw_dev_dbg(aw869xx->dev,
				   "%s wait for RTP_GO, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(2000, 2500);
	}
	if (rtp_work_flag) {
		aw869xx_haptic_rtp_init(aw869xx);
	} else {
		/* enter standby mode */
		aw869xx_haptic_stop(aw869xx);
		aw_dev_err(aw869xx->dev, "%s failed to enter RTP_GO status!\n",
			   __func__);
	}
}

/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw869xx_haptic_cont_config(struct aw869xx *aw869xx)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	/* work mode */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_CONT_MODE);
	/* bst mode */
	aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	/* cont config */
	/* aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
	 **                     AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
	 **                     AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	 */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* f0 driver level */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
			       aw869xx->cont_drv1_lvl);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG7,
			  aw869xx->cont_drv2_lvl);
	/* DRV1_TIME */
	/* aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG8, 0xFF); */
	/* DRV2_TIME */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG9, 0xFF);
	/* cont play go */
	aw869xx_haptic_play_go(aw869xx);
	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw869xx_haptic_read_lra_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	/* F_LRA_F0_H */
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD14, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	/* F_LRA_F0_L */
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD15, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		aw_dev_err(aw869xx->dev,
			   "%s didn't get lra f0 because f0_reg value is 0!\n",
			   __func__);
		aw869xx->f0 = aw869xx->dts_info.f0_ref;
		return -1;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw869xx->f0 = (unsigned int)f0_tmp;
		aw_dev_info(aw869xx->dev, "%s lra_f0=%d\n", __func__,
			    aw869xx->f0);
	}

	return 0;
}

static int aw869xx_haptic_read_cont_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD16, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD17, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		aw_dev_err(aw869xx->dev,
			   "%s didn't get cont f0 because f0_reg value is 0!\n",
			   __func__);
		aw869xx->cont_f0 = aw869xx->dts_info.f0_ref;
		return -1;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw869xx->cont_f0 = (unsigned int)f0_tmp;
		aw_dev_info(aw869xx->dev, "%s cont_f0=%d\n", __func__,
			    aw869xx->cont_f0);
	}
	return 0;
}

static int aw869xx_haptic_ram_get_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int cnt = 200;
	bool get_f0_flag = false;
	unsigned char brk_en_temp = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->f0 = aw869xx->dts_info.f0_ref;
	/* enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	/* f0 calibrate work mode */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_MODE);
	/* bst mode */
	aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	/* enable f0 detect */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* enable auto break */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG3, &reg_val);
	brk_en_temp = 0x04 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			       AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	aw869xx_haptic_set_wav_seq(aw869xx, 0x00, 0x01);
	aw869xx_haptic_set_wav_loop(aw869xx, 0x00, 0x0A);
	/* ram play go */
	aw869xx_haptic_play_go(aw869xx);
	/* 300ms */
	while (cnt) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			cnt = 0;
			get_f0_flag = true;
			aw_dev_info(aw869xx->dev,
				    "%s entered standby mode! glb_state=0x%02X\n",
				    __func__, reg_val);
		} else {
			cnt--;
			aw_dev_dbg(aw869xx->dev,
				   "%s waitting for standby, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(10000, 10500);
	}
	if (get_f0_flag) {
		aw869xx_haptic_read_lra_f0(aw869xx);
	} else {
		aw_dev_err(aw869xx->dev,
			   "%s enter standby mode failed, stop reading f0!\n",
			   __func__);
	}
	/* restore default config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			       brk_en_temp << 2);
	return ret;
}

static int aw869xx_haptic_cont_get_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int cnt = 200;
	bool get_f0_flag = false;
	unsigned char brk_en_temp = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->f0 = aw869xx->dts_info.f0_ref;
	/* enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	/* f0 calibrate work mode */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_CONT_MODE);
	/* bst mode */
	aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	/* enable f0 detect */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* enable auto break */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG3, &reg_val);
	brk_en_temp = 0x04 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			       AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	/* f0 driver level */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
			       aw869xx->dts_info.cont_drv1_lvl_dt);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG7,
			  aw869xx->dts_info.cont_drv2_lvl_dt);
	/* DRV1_TIME */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG8,
			  aw869xx->dts_info.cont_drv1_time_dt);
	/* DRV2_TIME */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG9,
			  aw869xx->dts_info.cont_drv2_time_dt);
	/* TRACK_MARGIN */
	if (!aw869xx->dts_info.cont_track_margin) {
		aw_dev_err(aw869xx->dev,
			   "%s aw869xx->dts_info.cont_track_margin = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG11,
				  (unsigned char)aw869xx->
				  dts_info.cont_track_margin);
	}
	/* DRV_WIDTH */
	/*
	 * aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG3,
	 *                aw869xx->dts_info.cont_drv_width);
	 */
	/* cont play go */
	aw869xx_haptic_play_go(aw869xx);
	/* 300ms */
	while (cnt) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			cnt = 0;
			get_f0_flag = true;
			aw_dev_info(aw869xx->dev,
				    "%s entered standby mode! glb_state=0x%02X\n",
				    __func__, reg_val);
		} else {
			cnt--;
			aw_dev_dbg(aw869xx->dev,
				   "%s waitting for standby, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(10000, 10500);
	}
	if (get_f0_flag) {
		aw869xx_haptic_read_lra_f0(aw869xx);
		aw869xx_haptic_read_cont_f0(aw869xx);
	} else {
		aw_dev_err(aw869xx->dev,
			   "%s enter standby mode failed, stop reading f0!\n",
			   __func__);
	}
	/* restore default config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			       brk_en_temp << 2);
	return ret;
}

static int aw869xx_haptic_f0_calibration(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	unsigned int f0_cali_min = aw869xx->dts_info.f0_ref * (100 -
				   aw869xx->dts_info.f0_cali_percent) / 100;
	unsigned int f0_cali_max = aw869xx->dts_info.f0_ref * (100 +
				   aw869xx->dts_info.f0_cali_percent) / 100;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	/*
	 * aw869xx_haptic_upload_lra(aw869xx, WRITE_ZERO);
	 */
	if (aw869xx_haptic_cont_get_f0(aw869xx)) {
		aw_dev_err(aw869xx->dev, "%s get f0 error, user defafult f0\n",
			   __func__);
	} else {
		/* max and min limit */
		f0_limit = aw869xx->f0;
		aw_dev_info(aw869xx->dev, "%s f0_ref = %d, f0_cali_min = %d,"
			    " f0_cali_max = %d, f0 = %d\n",
			    __func__, aw869xx->dts_info.f0_ref,
			    f0_cali_min, f0_cali_max, aw869xx->f0);

		if ((aw869xx->f0 < f0_cali_min) || aw869xx->f0 > f0_cali_max) {
			aw_dev_err(aw869xx->dev,
				   "%s f0 calibration out of range = %d!\n",
				   __func__, aw869xx->f0);
			f0_limit = aw869xx->dts_info.f0_ref;
			return -ERANGE;
		}
		aw_dev_info(aw869xx->dev, "%s f0_limit = %d\n", __func__,
			    (int)f0_limit);
		/* calculate cali step */
		f0_cali_step = 100000 * ((int)f0_limit -
					 (int)aw869xx->dts_info.f0_ref) /
		    ((int)f0_limit * 24);
		aw_dev_info(aw869xx->dev, "%s f0_cali_step = %d\n", __func__,
			    f0_cali_step);
		if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
			if (f0_cali_step % 10 >= 5)
				f0_cali_step = 32 + (f0_cali_step / 10 + 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		} else {	/* f0_cali_step < 0 */
			if (f0_cali_step % 10 <= -5)
				f0_cali_step = 32 + (f0_cali_step / 10 - 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		}
		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;
		/* update cali step */
		aw869xx_i2c_read(aw869xx, AW869XX_REG_TRIMCFG3, &reg_val);
		aw869xx->f0_cali_data = ((int)f0_cali_lra + (int)(reg_val & 0x3f)) & 0x3f;

		aw_dev_info(aw869xx->dev,
			    "%s origin trim_lra = 0x%02X, f0_cali_lra = 0x%02X,"
			    " final f0_cali_data = 0x%02X\n",
			    __func__, (reg_val & 0x3f), f0_cali_lra,
			    aw869xx->f0_cali_data);
		aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	}
	/* restore standby work mode */
	aw869xx_haptic_stop(aw869xx);
	return ret;
}

static int aw869xx_haptic_i2s_init(struct aw869xx *aw869xx)
{
	aw_dev_info(aw869xx->dev, "%s: enter\n", __func__);

	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
			       AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
			       AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_IOCFG1,
			       AW869XX_BIT_IOCFG1_IO_FAST_MASK,
			       AW869XX_BIT_IOCFG1_IIS_IO_FAST_ENABLE);
	return 0;
}

static int aw869xx_haptic_init(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	/* haptic init */
	mutex_lock(&aw869xx->lock);
	aw869xx->activate_mode = aw869xx->dts_info.mode;
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG1, &reg_val);
	aw869xx->index = reg_val & 0x7F;
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG2, &reg_val);
	aw869xx->gain = reg_val & 0xFF;
	aw_dev_info(aw869xx->dev, "%s aw869xx->gain =0x%02X\n", __func__,
		    aw869xx->gain);
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG1, &reg_val);
	aw869xx->vmax = reg_val & 0x1F;
	for (i = 0; i < AW869XX_SEQUENCER_SIZE; i++) {
		ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG1 + i,
				       &reg_val);
		aw869xx->seq[i] = reg_val;
	}
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_STANDBY_MODE);
	aw869xx_haptic_set_pwm(aw869xx, AW869XX_PWM_24K);
	/* misc value init */
	aw869xx_haptic_misc_para_init(aw869xx);
	/* set BST_ADJ */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG5,
			       AW869XX_BIT_BSTCFG5_BST_ADJ_MASK,
			       AW869XX_BIT_BSTCFG5_BST_ADJ_LOW);
	aw869xx_haptic_set_bst_peak_cur(aw869xx);
	aw869xx_haptic_swicth_motor_protect_config(aw869xx, 0x00, 0x00);
	aw869xx_haptic_auto_bst_enable(aw869xx,
				       aw869xx->dts_info.is_enabled_auto_bst);
	aw869xx_haptic_trig_param_init(aw869xx);
	aw869xx_haptic_trig_param_config(aw869xx);
	aw869xx_haptic_offset_calibration(aw869xx);
	/* vbat compensation */
	aw869xx_haptic_vbat_mode_config(aw869xx,
					AW869XX_HAPTIC_CONT_VBAT_HW_ADJUST_MODE);
	aw869xx->ram_vbat_compensate = AW869XX_HAPTIC_RAM_VBAT_COMP_ENABLE;
	/* i2s config */
	if (aw869xx->dts_info.is_enabled_i2s) {
		aw_dev_info(aw869xx->dev, "%s i2s is enabled!\n", __func__);
		aw869xx_haptic_i2s_init(aw869xx);
	}
	mutex_unlock(&aw869xx->lock);
#ifdef AW_F0_COARSE_CALI	/* Only Test for F0 calibration offset */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
			       AW869XX_BIT_TRIMCFG3_OSC_TRIM_SRC_MASK,
			       AW869XX_BIT_TRIMCFG3_OSC_TRIM_SRC_REG);
	/* aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIMCFG3,0xCF); */
	/* 170-0xEC 235-0x06 260-0xfc */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIMCFG4, 0xEC);
#endif
	/* f0 calibration */
	if (aw869xx->dts_info.is_enabled_powerup_f0_cali) {
		mutex_lock(&aw869xx->lock);
		aw869xx_haptic_upload_lra(aw869xx, WRITE_ZERO);
		aw869xx_haptic_f0_calibration(aw869xx);
		mutex_unlock(&aw869xx->lock);
	} else {
		aw_dev_info(aw869xx->dev,
			    "%s powerup f0 calibration is disabled\n",
			    __func__);
	}
	return ret;
}

/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int aw869xx_vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw869xx *aw869xx = container_of(dev, struct aw869xx, vib_dev);

	if (hrtimer_active(&aw869xx->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw869xx->timer);

		return ktime_to_ms(r);
	}
	return 0;
}

static void aw869xx_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw869xx *aw869xx = container_of(dev, struct aw869xx, vib_dev);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	if (value > 0) {
		aw869xx_haptic_ram_vbat_compensate(aw869xx, false);
		aw869xx_haptic_bst_mode_config(aw869xx,
					       AW869XX_HAPTIC_BST_MODE_BOOST);
		aw869xx_haptic_play_wav_seq(aw869xx, value);
	}
	mutex_unlock(&aw869xx->lock);
	aw_dev_info(aw869xx->dev, "%s exit\n", __func__);
}
#else
static enum led_brightness aw869xx_haptic_brightness_get(struct led_classdev
							 *cdev)
{
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	return aw869xx->amplitude;
}

static void aw869xx_haptic_brightness_set(struct led_classdev *cdev,
					  enum led_brightness level)
{
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	if (!aw869xx->ram_init)
		return;
	if (aw869xx->ram_update_flag < 0)
		return;
	aw869xx->amplitude = level;
	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	if (aw869xx->amplitude > 0) {
		aw869xx_haptic_ram_vbat_compensate(aw869xx, false);
		aw869xx_haptic_bst_mode_config(aw869xx,
					       AW869XX_HAPTIC_BST_MODE_BOOST);
		aw869xx_haptic_play_wav_seq(aw869xx, aw869xx->amplitude);
	}
	mutex_unlock(&aw869xx->lock);
}
#endif

static ssize_t aw869xx_state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	return snprintf(buf, PAGE_SIZE, "state = %d\n", aw869xx->state);
}

static ssize_t aw869xx_state_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return count;
}

static ssize_t aw869xx_duration_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw869xx->timer)) {
		time_rem = hrtimer_get_remaining(&aw869xx->timer);
		time_ms = ktime_to_ms(time_rem);
	}
	return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t aw869xx_duration_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;
	aw869xx->duration = val;
	return count;
}

static ssize_t aw869xx_activate_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw869xx->state);
}

static ssize_t aw869xx_activate_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dev_info(aw869xx->dev, "%s: value=%d\n", __func__, val);
	mutex_lock(&aw869xx->lock);
	hrtimer_cancel(&aw869xx->timer);
	aw869xx->state = val;
	mutex_unlock(&aw869xx->lock);
	schedule_work(&aw869xx->long_vibrate_work);
	return count;
}

static ssize_t aw869xx_activate_mode_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	return snprintf(buf, PAGE_SIZE, "activate_mode = %d\n",
			aw869xx->activate_mode);
}

static ssize_t aw869xx_activate_mode_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw869xx->lock);
	aw869xx->activate_mode = val;
	mutex_unlock(&aw869xx->lock);
	return count;
}

static ssize_t aw869xx_index_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned char reg_val = 0;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG1, &reg_val);
	aw869xx->index = reg_val;
	return snprintf(buf, PAGE_SIZE, "index = %d\n", aw869xx->index);
}

static ssize_t aw869xx_index_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dev_info(aw869xx->dev, "%s: value=%d\n", __func__, val);
	mutex_lock(&aw869xx->lock);
	aw869xx->index = val;
	aw869xx_haptic_set_repeat_wav_seq(aw869xx, aw869xx->index);
	mutex_unlock(&aw869xx->lock);
	return count;
}

static ssize_t aw869xx_vmax_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	return snprintf(buf, PAGE_SIZE, "vmax = 0x%02X\n", aw869xx->vmax);
}

static ssize_t aw869xx_vmax_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dev_info(aw869xx->dev, "%s: value=%d\n", __func__, val);

	mutex_lock(&aw869xx->lock);
	aw869xx->vmax = val;
	aw869xx_haptic_set_bst_vol(aw869xx, aw869xx->vmax);
	mutex_unlock(&aw869xx->lock);
	return count;
}

static ssize_t aw869xx_gain_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);

	return snprintf(buf, PAGE_SIZE, "gain = 0x%02X\n", aw869xx->gain);
}

static ssize_t aw869xx_gain_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw_dev_info(aw869xx->dev, "%s: value=%d\n", __func__, val);

	mutex_lock(&aw869xx->lock);
	aw869xx->gain = val;
	aw869xx_haptic_set_gain(aw869xx, aw869xx->gain);
	mutex_unlock(&aw869xx->lock);
	return count;
}

static ssize_t aw869xx_seq_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW869XX_SEQUENCER_SIZE; i++) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG1 + i, &reg_val);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d = %d\n", i + 1, reg_val);
		aw869xx->seq[i] |= reg_val;
	}
	return count;
}

static ssize_t aw869xx_seq_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_dev_info(aw869xx->dev, "%s: seq%d=0x%02X\n", __func__,
			    databuf[0], databuf[1]);
		mutex_lock(&aw869xx->lock);
		aw869xx->seq[databuf[0]] = (unsigned char)databuf[1];
		aw869xx_haptic_set_wav_seq(aw869xx, (unsigned char)databuf[0],
					   aw869xx->seq[databuf[0]]);
		mutex_unlock(&aw869xx->lock);
	}
	return count;
}

static ssize_t aw869xx_loop_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW869XX_SEQUENCER_LOOP_SIZE; i++) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG9 + i, &reg_val);
		aw869xx->loop[i * 2 + 0] = (reg_val >> 4) & 0x0F;
		aw869xx->loop[i * 2 + 1] = (reg_val >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d_loop = %d\n", i * 2 + 1,
				  aw869xx->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d_loop = %d\n", i * 2 + 2,
				  aw869xx->loop[i * 2 + 1]);
	}
	return count;
}

static ssize_t aw869xx_loop_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_dev_info(aw869xx->dev, "%s: seq%d loop=0x%02X\n", __func__,
			    databuf[0], databuf[1]);
		mutex_lock(&aw869xx->lock);
		aw869xx->loop[databuf[0]] = (unsigned char)databuf[1];
		aw869xx_haptic_set_wav_loop(aw869xx, (unsigned char)databuf[0],
					    aw869xx->loop[databuf[0]]);
		mutex_unlock(&aw869xx->lock);
	}

	return count;
}

static ssize_t aw869xx_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW869XX_REG_MAX; i++) {
		if (!(aw869xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw869xx_i2c_read(aw869xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02X=0x%02X\n", i, reg_val);
	}
	return len;
}

static ssize_t aw869xx_reg_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw869xx_i2c_write(aw869xx, (unsigned char)databuf[0],
				  (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw869xx_rtp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %d\n",
			aw869xx->rtp_cnt);
	return len;
}

static ssize_t aw869xx_rtp_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_dev_info(aw869xx->dev, "%s: kstrtouint fail\n", __func__);
		return rc;
	}
	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_set_rtp_aei(aw869xx, false);
	aw869xx_interrupt_clear(aw869xx);
	if (val < (sizeof(aw869xx_rtp_name) / AW869XX_RTP_NAME_MAX)) {
		aw869xx->rtp_file_num = val;
		if (val) {
			aw_dev_info(aw869xx->dev,
				    "%s: aw869xx_rtp_name[%d]: %s\n", __func__,
				    val, aw869xx_rtp_name[val]);

			schedule_work(&aw869xx->rtp_work);
		} else {
			aw_dev_err(aw869xx->dev,
				   "%s: rtp_file_num 0x%02X over max value\n",
				   __func__, aw869xx->rtp_file_num);
			}
	}
	mutex_unlock(&aw869xx->lock);
	return count;
}

static ssize_t aw869xx_ram_update_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	/* RAMINIT Enable */
	aw869xx_haptic_raminit(aw869xx, true);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRH,
			  (unsigned char)(aw869xx->ram.base_addr >> 8));
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRL,
			  (unsigned char)(aw869xx->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw869xx_haptic_ram:\n");
	for (i = 0; i < aw869xx->ram.len; i++) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_RAMDATA, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02X,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RAMINIT Disable */
	aw869xx_haptic_raminit(aw869xx, false);
	return len;
}

static ssize_t aw869xx_ram_update_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
		aw869xx_ram_update(aw869xx);
	return count;
}

static ssize_t aw869xx_f0_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	aw869xx_haptic_cont_get_f0(aw869xx);
	mutex_unlock(&aw869xx->lock);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"lra_f0 = %d cont_f0 = %d\n", aw869xx->f0,
			aw869xx->cont_f0);
	return len;
}

static ssize_t aw869xx_f0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t aw869xx_ram_f0_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	aw869xx_haptic_ram_get_f0(aw869xx);
	mutex_unlock(&aw869xx->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "ram_lra_f0 = %d\n",
			aw869xx->f0);
	return len;
}

static ssize_t aw869xx_ram_f0_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t aw869xx_osc_save_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw869xx->osc_cali_data);

	return len;
}

static ssize_t aw869xx_osc_save_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw869xx->osc_cali_data = val;
	return count;
}


static ssize_t aw869xx_f0_save_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 0x%02X\n",
			aw869xx->f0_cali_data);

	return len;
}

static ssize_t aw869xx_f0_save_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw869xx->f0_cali_data = val;
	return count;
}

static ssize_t aw869xx_cali_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	aw869xx_haptic_cont_get_f0(aw869xx);
	mutex_unlock(&aw869xx->lock);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"lra_f0 = %d cont_f0 = %d\n", aw869xx->f0,
			aw869xx->cont_f0);
	return len;
}

static ssize_t aw869xx_cali_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int i;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		mutex_lock(&aw869xx->lock);
		aw869xx_haptic_upload_lra(aw869xx, WRITE_ZERO);
		for (i = 0; i < 2; i++) {
			aw869xx_haptic_f0_calibration(aw869xx);
			mdelay(20);
		}
		mutex_unlock(&aw869xx->lock);
	}
	return count;
}

static ssize_t aw869xx_cont_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	aw869xx_haptic_read_cont_f0(aw869xx);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_f0 = %d\n", aw869xx->cont_f0);
	return len;
}

static ssize_t aw869xx_cont_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw869xx_haptic_stop(aw869xx);
	if (val)
		aw869xx_haptic_cont_config(aw869xx);
	return count;
}

static ssize_t aw869xx_cont_wait_num_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_wait_num = 0x%02X\n",
			aw869xx->cont_wait_num);
	return len;
}

static ssize_t aw869xx_cont_wait_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw869xx->cont_wait_num = databuf[0];
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG4, databuf[0]);
	}
	return count;
}

static ssize_t aw869xx_cont_drv_lvl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw869xx->cont_drv1_lvl, aw869xx->cont_drv2_lvl);
	return len;
}

static ssize_t aw869xx_cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw869xx->cont_drv1_lvl = databuf[0];
		aw869xx->cont_drv2_lvl = databuf[1];
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
				       AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
				       aw869xx->cont_drv1_lvl);
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG7,
				  aw869xx->cont_drv2_lvl);
	}
	return count;
}

static ssize_t aw869xx_cont_drv_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw869xx->cont_drv1_time, aw869xx->cont_drv2_time);
	return len;
}

static ssize_t aw869xx_cont_drv_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw869xx->cont_drv1_time = databuf[0];
		aw869xx->cont_drv2_time = databuf[1];
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG8,
				  aw869xx->cont_drv1_time);
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG9,
				  aw869xx->cont_drv2_time);
	}
	return count;
}

static ssize_t aw869xx_cont_brk_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw869xx->cont_brk_time);
	return len;
}

static ssize_t aw869xx_cont_brk_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw869xx->cont_brk_time = databuf[0];
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG10,
				  aw869xx->cont_brk_time);
	}
	return count;
}

static ssize_t aw869xx_vbat_monitor_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_get_vbat(aw869xx);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_monitor = %d\n",
			aw869xx->vbat);
	mutex_unlock(&aw869xx->lock);

	return len;
}

static ssize_t aw869xx_vbat_monitor_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	return count;
}

static ssize_t aw869xx_lra_resistance_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	aw869xx_haptic_get_lra_resistance(aw869xx);
	len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %d\n",
			aw869xx->lra);
	return len;
}

static ssize_t aw869xx_lra_resistance_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	return count;
}

static ssize_t aw869xx_auto_boost_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "auto_boost = %d\n",
			aw869xx->auto_boost);

	return len;
}

static ssize_t aw869xx_auto_boost_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_auto_bst_enable(aw869xx, val);
	mutex_unlock(&aw869xx->lock);

	return count;
}

static ssize_t aw869xx_prctmode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;
	unsigned char reg_val = 0;

	aw869xx_i2c_read(aw869xx, AW869XX_REG_DETCFG1, &reg_val);

	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode = %d\n",
			reg_val & 0x08);
	return len;
}

static ssize_t aw869xx_prctmode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[2] = { 0, 0 };
	unsigned int addr = 0;
	unsigned int val = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		addr = databuf[0];
		val = databuf[1];
		mutex_lock(&aw869xx->lock);
		aw869xx_haptic_swicth_motor_protect_config(aw869xx, addr, val);
		mutex_unlock(&aw869xx->lock);
	}
	return count;
}

static ssize_t aw869xx_trig_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;
	unsigned char i = 0;

	for (i = 0; i < AW869XX_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: trig_level=%d, trig_polar=%d, pos_enable=%d, pos_sequence=%d, neg_enable=%d, neg_sequence=%d trig_brk=%d, trig_bst=%d\n",
				i + 1,
				aw869xx->trig[i].trig_level,
				aw869xx->trig[i].trig_polar,
				aw869xx->trig[i].pos_enable,
				aw869xx->trig[i].pos_sequence,
				aw869xx->trig[i].neg_enable,
				aw869xx->trig[i].neg_sequence,
				aw869xx->trig[i].trig_brk,
				aw869xx->trig[i].trig_bst);
	}

	return len;
}

static ssize_t aw869xx_trig_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int databuf[9] = { 0 };

	if (sscanf(buf, "%d %d %d %d %d %d %d %d %d", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5],
		   &databuf[6], &databuf[7], &databuf[8])) {
		aw_dev_info(aw869xx->dev,
			    "%s: %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			    __func__, databuf[0], databuf[1], databuf[2],
			    databuf[3], databuf[4], databuf[5], databuf[6],
			    databuf[7], databuf[8]);
		if (databuf[0] > 3)
			databuf[0] = 3;
		if (databuf[0] < 0)
			databuf[0] = 1;
		aw869xx->trig[databuf[0]].trig_level = databuf[1];
		aw869xx->trig[databuf[0]].trig_polar = databuf[2];
		aw869xx->trig[databuf[0]].pos_enable = databuf[3];
		aw869xx->trig[databuf[0]].pos_sequence = databuf[4];
		aw869xx->trig[databuf[0]].neg_enable = databuf[5];
		aw869xx->trig[databuf[0]].neg_sequence = databuf[6];
		aw869xx->trig[databuf[0]].trig_brk = databuf[7];
		aw869xx->trig[databuf[0]].trig_bst = databuf[8];
		mutex_lock(&aw869xx->lock);
		aw869xx_haptic_trig_param_config(aw869xx);
		mutex_unlock(&aw869xx->lock);
	}
	return count;
}

static ssize_t aw869xx_ram_vbat_compensate_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_compensate = %d\n",
			aw869xx->ram_vbat_compensate);

	return len;
}

static ssize_t aw869xx_ram_vbat_compensate_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw869xx->lock);
	if (val)
		aw869xx->ram_vbat_compensate =
		    AW869XX_HAPTIC_RAM_VBAT_COMP_ENABLE;
	else
		aw869xx->ram_vbat_compensate =
		    AW869XX_HAPTIC_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw869xx->lock);

	return count;
}

static ssize_t aw869xx_osc_cali_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw869xx->osc_cali_data);

	return len;
}

static ssize_t aw869xx_osc_cali_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, vib_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw869xx->lock);
	if (val == 1) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_D2SCFG1,
				       AW869XX_BIT_D2SCFG1_CLK_TRIM_MODE_MASK,
				       AW869XX_BIT_D2SCFG1_CLK_TRIM_MODE_24K);
		aw869xx_haptic_upload_lra(aw869xx, WRITE_ZERO);
		aw869xx_rtp_osc_calibration(aw869xx);
		aw869xx_rtp_trim_lra_calibration(aw869xx);
	}
	/* osc calibration flag end,Other behaviors are permitted */
	mutex_unlock(&aw869xx->lock);

	return count;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw869xx_state_show,
		   aw869xx_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw869xx_duration_show,
		   aw869xx_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw869xx_activate_show,
		   aw869xx_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw869xx_activate_mode_show,
		   aw869xx_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw869xx_index_show,
		   aw869xx_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw869xx_vmax_show,
		   aw869xx_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw869xx_gain_show,
		   aw869xx_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw869xx_seq_show, aw869xx_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw869xx_loop_show,
		   aw869xx_loop_store);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw869xx_reg_show, aw869xx_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw869xx_rtp_show, aw869xx_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw869xx_ram_update_show,
		   aw869xx_ram_update_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, aw869xx_f0_show, aw869xx_f0_store);
static DEVICE_ATTR(f0_save, S_IWUSR | S_IRUGO, aw869xx_f0_save_show,
		   aw869xx_f0_save_store);
static DEVICE_ATTR(osc_save, S_IWUSR | S_IRUGO, aw869xx_osc_save_show,
		   aw869xx_osc_save_store);
static DEVICE_ATTR(ram_f0, S_IWUSR | S_IRUGO, aw869xx_ram_f0_show,
		   aw869xx_ram_f0_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw869xx_cali_show,
		   aw869xx_cali_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw869xx_cont_show,
		   aw869xx_cont_store);
static DEVICE_ATTR(cont_wait_num, S_IWUSR | S_IRUGO, aw869xx_cont_wait_num_show,
		   aw869xx_cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, S_IWUSR | S_IRUGO, aw869xx_cont_drv_lvl_show,
		   aw869xx_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, S_IWUSR | S_IRUGO, aw869xx_cont_drv_time_show,
		   aw869xx_cont_drv_time_store);
static DEVICE_ATTR(cont_brk_time, S_IWUSR | S_IRUGO, aw869xx_cont_brk_time_show,
		   aw869xx_cont_brk_time_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw869xx_vbat_monitor_show,
		   aw869xx_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO,
		   aw869xx_lra_resistance_show, aw869xx_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw869xx_auto_boost_show,
		   aw869xx_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw869xx_prctmode_show,
		   aw869xx_prctmode_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw869xx_trig_show,
		   aw869xx_trig_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO,
		   aw869xx_ram_vbat_compensate_show,
		   aw869xx_ram_vbat_compensate_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, aw869xx_osc_cali_show,
		   aw869xx_osc_cali_store);
static struct attribute *aw869xx_vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_reg.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_f0_save.attr,
	&dev_attr_osc_save.attr,
	&dev_attr_ram_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_cont.attr,
	&dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_trig.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_osc_cali.attr,
	NULL
};

static struct attribute_group aw869xx_vibrator_attribute_group = {
	.attrs = aw869xx_vibrator_attributes
};

static enum hrtimer_restart aw869xx_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw869xx *aw869xx = container_of(timer, struct aw869xx, timer);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->state = 0;
	schedule_work(&aw869xx->long_vibrate_work);

	return HRTIMER_NORESTART;
}

static void aw869xx_long_vibrate_work_routine(struct work_struct *work)
{
	struct aw869xx *aw869xx = container_of(work, struct aw869xx,
					       long_vibrate_work);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	mutex_lock(&aw869xx->lock);
	/* Enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	if (aw869xx->state) {
		if (aw869xx->activate_mode == AW869XX_HAPTIC_ACTIVATE_RAM_MODE) {
			aw869xx_haptic_ram_vbat_compensate(aw869xx, true);
			aw869xx_haptic_bst_mode_config(aw869xx,
						       AW869XX_HAPTIC_BST_MODE_BYPASS);
			aw869xx_haptic_play_repeat_seq(aw869xx, true);
		} else if (aw869xx->activate_mode ==
			   AW869XX_HAPTIC_ACTIVATE_CONT_MODE) {
			aw869xx_haptic_cont_config(aw869xx);
		} else {
			aw_dev_err(aw869xx->dev, "%s: activate_mode error\n",
				   __func__);
		}
		/* run ms timer */
		hrtimer_start(&aw869xx->timer,
			      ktime_set(aw869xx->duration / 1000,
					(aw869xx->duration % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&aw869xx->lock);
}

static int aw869xx_vibrator_init(struct aw869xx *aw869xx)
{
	int ret = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

#ifdef TIMED_OUTPUT
	aw_dev_info(aw869xx->dev, "%s: TIMED_OUT FRAMEWORK!\n", __func__);
	aw869xx->vib_dev.name = "vibrator";
	aw869xx->vib_dev.get_time = aw869xx_vibrator_get_time;
	aw869xx->vib_dev.enable = aw869xx_vibrator_enable;

	ret = timed_output_dev_register(&(aw869xx->vib_dev));
	if (ret < 0) {
		aw_dev_err(aw869xx->dev,
			   "%s: fail to create timed output dev\n", __func__);
		return ret;
	}
	ret = sysfs_create_group(&aw869xx->vib_dev.dev->kobj,
				 &aw869xx_vibrator_attribute_group);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s error creating sysfs attr files\n",
			   __func__);
		return ret;
	}
#else
	aw_dev_info(aw869xx->dev, "%s: loaded in leds_cdev framework!\n",
		    __func__);
	aw869xx->vib_dev.name = "vibrator";
	aw869xx->vib_dev.brightness_get = aw869xx_haptic_brightness_get;
	aw869xx->vib_dev.brightness_set = aw869xx_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw869xx->i2c->dev, &aw869xx->vib_dev);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s: fail to create led dev\n",
			   __func__);
		return ret;
	}
	ret = sysfs_create_group(&aw869xx->vib_dev.dev->kobj,
				 &aw869xx_vibrator_attribute_group);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s error creating sysfs attr files\n",
			   __func__);
		return ret;
	}
#endif
	hrtimer_init(&aw869xx->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw869xx->timer.function = aw869xx_vibrator_timer_func;
	INIT_WORK(&aw869xx->long_vibrate_work,
		  aw869xx_long_vibrate_work_routine);
	INIT_WORK(&aw869xx->rtp_work, aw869xx_rtp_work_routine);
	mutex_init(&aw869xx->lock);
	mutex_init(&aw869xx->rtp_lock);

	return 0;
}

/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
static int aw869xx_i2s_enable(struct aw869xx *aw869xx, bool flag)
{
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_I2SCFG2,
				       AW869XX_BIT_I2SCFG2_I2S_EN_MASK,
				       AW869XX_BIT_I2SCFG2_I2S_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_I2SCFG2,
				       AW869XX_BIT_I2SCFG2_I2S_EN_MASK,
				       AW869XX_BIT_I2SCFG2_I2S_DISABLE);
	}
	return 0;
}

static int aw869xx_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw869xx->dev, "%s: enter\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mutex_lock(&aw869xx->lock);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
				       AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
		mutex_unlock(&aw869xx->lock);
	}

	return 0;
}

static int aw869xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*struct aw869xx *aw869xx = aw_snd_soc_codec_get_drvdata(dai->codec); */
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);

	aw_dev_info(codec->dev, "%s: fmt=0x%02X\n", __func__, fmt);

	/* supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) !=
		    SND_SOC_DAIFMT_CBS_CFS) {
			aw_dev_err(codec->dev,
				   "%s: invalid codec master mode\n", __func__);
			return -EINVAL;
		}
		break;
	default:
		aw_dev_err(codec->dev, "%s: unsupported DAI format %d\n",
			   __func__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	return 0;
}

static int aw869xx_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw869xx->dev, "%s: freq=%d\n", __func__, freq);

	aw869xx->sysclk = freq;
	return 0;
}

static int aw869xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint8_t i2sfs_val = 0;
	uint8_t i2sbck_val = 0;
	uint8_t tmp_val = 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_info(aw869xx->dev, "%s: steam is capture\n", __func__);
		return 0;
	}

	aw_dev_info(aw869xx->dev, "%s: requested rate: %d, sample size: %d\n",
		    __func__, params_rate(params),
		    snd_pcm_format_width(params_format(params)));

	/* get rate param */
	aw869xx->rate = params_rate(params);

	mutex_lock(&aw869xx->lock);

	/* get bit width */
	aw869xx->width = params_width(params);
	aw_dev_info(aw869xx->dev, "%s: width = %d\n", __func__, aw869xx->width);
	i2sbck_val = params_width(params) * params_channels(params);
	/* match bit width */
	switch (aw869xx->width) {
	case 16:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_16BIT;
		break;
	case 24:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_24BIT;
		i2sbck_val = 32 * params_channels(params);
		break;
	case 32:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_32BIT;
		break;
	default:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_16BIT;
		i2sbck_val = 16 * params_channels(params);
		aw_dev_err(aw869xx->dev, "%s: width [%d]can not support\n",
			   __func__, aw869xx->width);
		break;
	}
	aw_dev_info(aw869xx->dev, "%s: i2sfs_val=0x%02X\n", __func__,
		    i2sfs_val);
	/* set width */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_I2SCFG1,
			       AW869XX_BIT_I2SCFG1_I2SFS_MASK, i2sfs_val);

	/* match bck mode */
	switch (i2sbck_val) {
	case 32:
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_32FS;
		break;
	case 48:
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_48FS;
		break;
	case 64:
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_64FS;
		break;
	default:
		aw_dev_err(aw869xx->dev, "%s: i2sbck [%d] can not support\n",
			   __func__, i2sbck_val);
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_32FS;
		break;
	}
	/* set bck mode */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_I2SCFG1,
			       AW869XX_BIT_I2SCFG1_I2SBCK_MASK, i2sbck_val);

	/* check i2s cfg */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_I2SCFG1, &tmp_val);
	aw_dev_info(aw869xx->dev, "%s: i2scfg1=0x%02X\n", __func__, tmp_val);

	mutex_unlock(&aw869xx->lock);

	return 0;
}

static int aw869xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint8_t reg_val = 0;

	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_info(aw869xx->dev, "%s: steam is capture\n", __func__);
		return 0;
	}

	aw_dev_info(aw869xx->dev, "%s: mute state=%d\n", __func__, mute);

	if (mute) {
		mutex_lock(&aw869xx->lock);
		aw869xx_i2s_enable(aw869xx, false);
		mutex_unlock(&aw869xx->lock);
	} else {
		mutex_lock(&aw869xx->lock);
		aw869xx_i2s_enable(aw869xx, true);
		usleep_range(2000, 3000);
		aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
		aw_dev_info(aw869xx->dev, "%s: sysst=0x%02X\n", __func__,
			    reg_val);
		aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST2, &reg_val);
		aw_dev_info(aw869xx->dev, "%s: sysst2=0x%02X\n", __func__,
			    reg_val);
		aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSER, &reg_val);
		aw_dev_info(aw869xx->dev, "%s: syser=0x%02X\n", __func__,
			    reg_val);
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if (reg_val != 0x0a) {
			aw_dev_err(aw869xx->dev,
				   "%s: i2s config error, glb_state=0x%02X\n",
				   __func__, reg_val);
			aw869xx_i2s_enable(aw869xx, false);
		} else {
			aw_dev_info(aw869xx->dev,
				    "%s: i2s config pass, glb_state=0x%02X\n",
				    __func__, reg_val);
		}
		mutex_unlock(&aw869xx->lock);
	}

	return 0;
}

static void aw869xx_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_dev_info(aw869xx->dev, "%s: enter\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw869xx->rate = 0;
		mutex_lock(&aw869xx->lock);
		/* aw869xx_i2s_enable(aw869xx, false); */
		mutex_unlock(&aw869xx->lock);
	}
}

static const struct snd_soc_dai_ops aw869xx_dai_ops = {
	.startup = aw869xx_startup,
	.set_fmt = aw869xx_set_fmt,
	.set_sysclk = aw869xx_set_dai_sysclk,
	.hw_params = aw869xx_hw_params,
	.mute_stream = aw869xx_mute,
	.shutdown = aw869xx_shutdown,
};

static struct snd_soc_dai_driver aw869xx_dai[] = {
	{
	 .name = "aw869xx-aif",
	 .id = 1,
	 .playback = {
		      .stream_name = "Speaker_Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = AW869XX_RATES,
		      .formats = AW869XX_FORMATS,
		      },
	 .capture = {
		     .stream_name = "Speaker_Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AW869XX_RATES,
		     .formats = AW869XX_FORMATS,
		     },
	 .ops = &aw869xx_dai_ops,
	 .symmetric_rates = 1,
	 },
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static void aw869xx_add_codec_controls(struct aw869xx *aw869xx)
{
	aw_dev_info(aw869xx->dev, "%s: enter\n", __func__);
}

static int aw869xx_probe(aw_snd_soc_codec_t *codec)
{
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	int ret = -1;

	aw_dev_info(aw869xx->dev, "%s: enter\n", __func__);

	aw869xx->codec = codec;

	aw869xx_add_codec_controls(aw869xx);

	aw_dev_info(aw869xx->dev, "%s: exit\n", __func__);

	ret = 0;
	return ret;
}

#ifdef AW_KERNEL_VER_OVER_4_19
static void aw869xx_remove(struct snd_soc_component *component)
{
	/*struct aw869xx *aw869xx =
	   aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec); */
	aw_dev_info(component->dev, "%s: enter\n", __func__);

	return;
}
#else
static int aw869xx_remove(aw_snd_soc_codec_t *codec)
{
	/* struct aw869xx *aw869xx =
	   aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec); */
	aw_dev_info(codec->dev, "%s: enter\n", __func__);

	return 0;
}
#endif

static unsigned int aw869xx_codec_read(aw_snd_soc_codec_t *codec,
				       unsigned int reg)
{
	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	uint8_t value = 0;
	int ret = -1;

	aw_dev_info(aw869xx->dev, "%s: enter\n", __func__);

	ret = aw869xx_i2c_read(aw869xx, reg, &value);
	if (ret < 0) {
		aw_dev_err(aw869xx->dev, "%s: read register failed\n",
			   __func__);
		return ret;
	}
	return ret;
}

static int aw869xx_codec_write(aw_snd_soc_codec_t *codec,
			       unsigned int reg, unsigned int value)
{
	int ret = -1;

	struct aw869xx *aw869xx =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	aw_dev_info(aw869xx->dev, "%s: enter ,reg:0x%02X = 0x%02X\n",
		    __func__, reg, value);

	ret = aw869xx_i2c_write(aw869xx, (uint8_t) reg, (uint8_t) value);

	return ret;
}

#ifdef AW_KERNEL_VER_OVER_4_19
static struct snd_soc_component_driver soc_codec_dev_aw869xx = {
	.probe = aw869xx_probe,
	.remove = aw869xx_remove,
	.read = aw869xx_codec_read,
	.write = aw869xx_codec_write,
};
#else
static struct snd_soc_codec_driver soc_codec_dev_aw869xx = {
	.probe = aw869xx_probe,
	.remove = aw869xx_remove,
	.read = aw869xx_codec_read,
	.write = aw869xx_codec_write,
	.reg_cache_size = AW869XX_REG_MAX,
	.reg_word_size = 2,
};
#endif
/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw869xx_interrupt_clear(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	aw_dev_dbg(aw869xx->dev, "%s: reg SYSINT=0x%02X\n", __func__, reg_val);
}

static void aw869xx_interrupt_setup(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);

	aw_dev_info(aw869xx->dev, "%s: reg SYSINT=0x%02X\n", __func__, reg_val);

	/* edge int mode */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_INT_MODE_MASK,
			       AW869XX_BIT_SYSCTRL7_INT_MODE_EDGE);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_MASK,
			       AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_POS);
	/* int enable */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
			       AW869XX_BIT_SYSINTM_BST_SCPM_MASK,
			       AW869XX_BIT_SYSINTM_BST_SCPM_OFF);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
			       AW869XX_BIT_SYSINTM_BST_OVPM_MASK,
			       AW869XX_BIT_SYSINTM_BST_OVPM_ON);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
			       AW869XX_BIT_SYSINTM_UVLM_MASK,
			       AW869XX_BIT_SYSINTM_UVLM_ON);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
			       AW869XX_BIT_SYSINTM_OCDM_MASK,
			       AW869XX_BIT_SYSINTM_OCDM_ON);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
			       AW869XX_BIT_SYSINTM_OTM_MASK,
			       AW869XX_BIT_SYSINTM_OTM_ON);
}

static irqreturn_t aw869xx_irq(int irq, void *data)
{
	struct aw869xx *aw869xx = data;
	unsigned char reg_val = 0;
	unsigned int buf_len = 0;
	unsigned char glb_state_val = 0;

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	aw_dev_info(aw869xx->dev, "%s: reg SYSINT=0x%02X\n", __func__, reg_val);
	if (reg_val & AW869XX_BIT_SYSINT_BST_OVPI)
		aw_dev_err(aw869xx->dev, "%s chip ov int error\n", __func__);
	if (reg_val & AW869XX_BIT_SYSINT_UVLI)
		aw_dev_err(aw869xx->dev, "%s chip uvlo int error\n", __func__);
	if (reg_val & AW869XX_BIT_SYSINT_OCDI)
		aw_dev_err(aw869xx->dev, "%s chip over current int error\n",
			   __func__);
	if (reg_val & AW869XX_BIT_SYSINT_OTI)
		aw_dev_err(aw869xx->dev, "%s chip over temperature int error\n",
			   __func__);
	if (reg_val & AW869XX_BIT_SYSINT_DONEI)
		aw_dev_info(aw869xx->dev, "%s chip playback done\n", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_FF_AEI) {
		aw_dev_info(aw869xx->dev, "%s: aw869xx rtp fifo almost empty\n",
			    __func__);
		if (aw869xx->rtp_init) {
			while ((!aw869xx_haptic_rtp_get_fifo_afs(aw869xx)) &&
			       (aw869xx->play_mode ==
				AW869XX_HAPTIC_RTP_MODE)) {
				mutex_lock(&aw869xx->rtp_lock);
				aw_dev_info(aw869xx->dev,
					    "%s: aw869xx rtp mode fifo update, cnt=%d\n",
					    __func__, aw869xx->rtp_cnt);
				if (!aw869xx_rtp) {
					pr_info
					    ("%s:aw869xx_rtp is null, break!\n",
					     __func__);
					mutex_unlock(&aw869xx->rtp_lock);
					break;
				}
				if ((aw869xx_rtp->len - aw869xx->rtp_cnt) <
				    (aw869xx->ram.base_addr >> 2)) {
					buf_len =
					    aw869xx_rtp->len - aw869xx->rtp_cnt;
				} else {
					buf_len = (aw869xx->ram.base_addr >> 2);
				}
				aw869xx->rtp_update_flag =
				    aw869xx_i2c_writes(aw869xx,
						       AW869XX_REG_RTPDATA,
						       &aw869xx_rtp->data
						       [aw869xx->rtp_cnt],
						       buf_len);
				aw869xx->rtp_cnt += buf_len;
				aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5,
						 &glb_state_val);
				if ((aw869xx->rtp_cnt == aw869xx_rtp->len)
				    || ((glb_state_val & 0x0f) == 0)) {
					aw_dev_info(aw869xx->dev,
						    "%s: rtp update complete\n",
						    __func__);
					aw869xx_haptic_set_rtp_aei(aw869xx,
								   false);
					aw869xx->rtp_cnt = 0;
					aw869xx->rtp_init = 0;
					mutex_unlock(&aw869xx->rtp_lock);
					break;
				}
				mutex_unlock(&aw869xx->rtp_lock);
			}
		} else {
			aw_dev_info(aw869xx->dev,
				    "%s: aw869xx rtp init = %d, init error\n",
				    __func__, aw869xx->rtp_init);
		}
	}

	if (reg_val & AW869XX_BIT_SYSINT_FF_AFI)
		aw_dev_info(aw869xx->dev,
			    "%s: aw869xx rtp mode fifo almost full!\n",
			    __func__);

	if (aw869xx->play_mode != AW869XX_HAPTIC_RTP_MODE)
		aw869xx_haptic_set_rtp_aei(aw869xx, false);

	aw_dev_info(aw869xx->dev, "%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw869xx_parse_dt(struct device *dev, struct aw869xx *aw869xx,
			    struct device_node *np)
{
	unsigned int val = 0;
	unsigned int bstcfg_temp[5] = { 0x2a, 0x24, 0x9a, 0x40, 0x91 };
	unsigned int prctmode_temp[3];
	unsigned int sine_array_temp[4] = { 0x05, 0xB2, 0xFF, 0xEF };
	unsigned int trig_config_temp[24] = { 1, 0, 1, 1, 1, 2, 0, 0,
		1, 0, 0, 1, 0, 2, 0, 0,
		1, 0, 0, 1, 0, 2, 0, 0
	};

	aw869xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw869xx->reset_gpio < 0) {
		aw_dev_err(dev, "%s: no reset gpio provide\n", __func__);
		return -1;
	} else {
		dev_info(dev, "%s: reset gpio provide ok\n", __func__);
	}
	aw869xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw869xx->irq_gpio < 0)
		aw_dev_err(dev, "%s: no irq gpio provide.\n", __func__);
	else
		dev_info(dev, "%s: irq gpio provide ok.\n", __func__);

	val = of_property_read_u32(np, "vib_mode", &aw869xx->dts_info.mode);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_mode not found\n", __func__);
	val = of_property_read_u32(np, "vib_f0_ref", &aw869xx->dts_info.f0_ref);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_f0_ref not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_f0_cali_percent",
				 &aw869xx->dts_info.f0_cali_percent);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_f0_cali_percent not found\n",
			    __func__);

	val = of_property_read_u32(np, "vib_cont_drv1_lvl",
				   &aw869xx->dts_info.cont_drv1_lvl_dt);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_drv1_lvl not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv2_lvl",
				 &aw869xx->dts_info.cont_drv2_lvl_dt);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_drv2_lvl not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv1_time",
				 &aw869xx->dts_info.cont_drv1_time_dt);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_drv1_time not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv2_time",
				 &aw869xx->dts_info.cont_drv2_time_dt);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_drv2_time not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv_width",
				 &aw869xx->dts_info.cont_drv_width);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_drv_width not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_wait_num",
				 &aw869xx->dts_info.cont_wait_num_dt);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_wait_num not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_bst_brk_gain",
				 &aw869xx->dts_info.cont_bst_brk_gain);
	if (val != 0)
		aw_dev_info(aw869xx->dev,
			    "%s vib_cont_bst_brk_gain not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_brk_gain",
				 &aw869xx->dts_info.cont_brk_gain);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_brk_gain not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_tset",
				 &aw869xx->dts_info.cont_tset);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_tset not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_bemf_set",
				 &aw869xx->dts_info.cont_bemf_set);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_bemf_set not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_d2s_gain",
				 &aw869xx->dts_info.d2s_gain);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_d2s_gain not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_brk_time",
				 &aw869xx->dts_info.cont_brk_time_dt);
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_cont_brk_time not found\n",
			    __func__);
	val =
	    of_property_read_u32(np, "vib_cont_track_margin",
				 &aw869xx->dts_info.cont_track_margin);
	if (val != 0)
		aw_dev_info(aw869xx->dev,
			    "%s vib_cont_track_margin not found\n", __func__);
	aw869xx->dts_info.is_enabled_auto_bst =
	    of_property_read_bool(np, "vib_is_enabled_auto_bst");
	aw_dev_info(aw869xx->dev,
		    "%s aw869xx->dts_info.is_enabled_auto_bst = %d\n", __func__,
		    aw869xx->dts_info.is_enabled_auto_bst);
	aw869xx->dts_info.is_enabled_i2s =
	    of_property_read_bool(np, "vib_is_enabled_i2s");
	aw_dev_info(aw869xx->dev, "%s aw869xx->dts_info.is_enabled_i2s = %d\n",
		    __func__, aw869xx->dts_info.is_enabled_i2s);
	aw869xx->dts_info.is_enabled_powerup_f0_cali =
	    of_property_read_bool(np, "vib_is_enabled_powerup_f0_cali");
	aw_dev_info(aw869xx->dev,
		    "%s aw869xx->dts_info.is_enabled_powerup_f0_cali = %d\n",
		    __func__, aw869xx->dts_info.is_enabled_powerup_f0_cali);
	val =
	    of_property_read_u32_array(np, "vib_bstcfg", bstcfg_temp,
				       ARRAY_SIZE(bstcfg_temp));
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_bstcfg not found\n",
			    __func__);
	memcpy(aw869xx->dts_info.bstcfg, bstcfg_temp, sizeof(bstcfg_temp));
	val = of_property_read_u32_array(np, "vib_prctmode",
					 prctmode_temp,
					 ARRAY_SIZE(prctmode_temp));
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_prctmode not found\n",
			    __func__);
	memcpy(aw869xx->dts_info.prctmode, prctmode_temp,
	       sizeof(prctmode_temp));
	val =
	    of_property_read_u32_array(np, "vib_sine_array", sine_array_temp,
				       ARRAY_SIZE(sine_array_temp));
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_sine_array not found\n",
			    __func__);
	memcpy(aw869xx->dts_info.sine_array, sine_array_temp,
	       sizeof(sine_array_temp));
	val =
	    of_property_read_u32_array(np, "vib_trig_config", trig_config_temp,
				       ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_dev_info(aw869xx->dev, "%s vib_trig_config not found\n",
			    __func__);
	memcpy(aw869xx->dts_info.trig_config, trig_config_temp,
	       sizeof(trig_config_temp));

	return 0;
}

static int aw869xx_hw_reset(struct aw869xx *aw869xx)
{
	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);

	if (aw869xx && gpio_is_valid(aw869xx->reset_gpio)) {
		gpio_set_value_cansleep(aw869xx->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(aw869xx->reset_gpio, 1);
		usleep_range(3500, 4000);
	} else {
		aw_dev_err(aw869xx->dev, "%s:  failed\n", __func__);
	}

	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw869xx_read_chipid(struct aw869xx *aw869xx)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		/* hardware reset */
		aw869xx_hw_reset(aw869xx);
		/* aw869xx_haptic_softreset(aw869xx); */
		ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_ID, &reg);
		if (ret < 0) {
			aw_dev_err(aw869xx->dev,
				   "%s: failed to read AW869XX_REG_ID: %d\n",
				   __func__, ret);
		}
		switch (reg) {
		case AW86905_CHIPID:
			aw_dev_info(aw869xx->dev,
				    "%s detected aw86905, ChipID: 0x%02X\n",
				    __func__, reg);
			aw869xx->chipid = AW86905_CHIPID;
			aw869xx->bst_pc = AW869XX_HAPTIC_BST_PC_L1;
			aw869xx->dts_info.is_enabled_i2s = false;
			/* aw869xx->flags |= AW869XX_FLAG_SKIP_INTERRUPTS; */
			return 0;
		case AW86907_CHIPID:
			aw_dev_info(aw869xx->dev,
				    "%s detected aw86907, ChipID: 0x%02X\n",
				    __func__, reg);
			aw869xx->chipid = AW86907_CHIPID;
			aw869xx->bst_pc = AW869XX_HAPTIC_BST_PC_L2;
			aw869xx->dts_info.is_enabled_i2s = false;
			/* aw869xx->flags |= AW869XX_FLAG_SKIP_INTERRUPTS; */
			return 0;
		case AW86915_CHIPID:
			aw_dev_info(aw869xx->dev,
				    "%s detected aw86915, ChipID: 0x%02X\n",
				    __func__, reg);
			aw869xx->chipid = AW86915_CHIPID;
			aw869xx->bst_pc = AW869XX_HAPTIC_BST_PC_L1;
			/* aw869xx->flags |= AW869XX_FLAG_SKIP_INTERRUPTS; */
			return 0;
		case AW86917_CHIPID:
			aw_dev_info(aw869xx->dev,
				    "%s detected aw86917, ChipID: 0x%02X\n",
				    __func__, reg);
			aw869xx->chipid = AW86917_CHIPID;
			aw869xx->bst_pc = AW869XX_HAPTIC_BST_PC_L2;
			/* aw869xx->flags |= AW869XX_FLAG_SKIP_INTERRUPTS; */
			return 0;
		default:
			aw_dev_info(aw869xx->dev,
				    "%s unsupported device revision (0x%02X)\n",
				    __func__, reg);
			break;
		}
		cnt++;
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}
	return -EINVAL;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw869xx_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct aw869xx *aw869xx;
	struct device_node *np = i2c->dev.of_node;
	struct snd_soc_dai_driver *dai;
	int irq_flags = 0;
	int ret = -1;
	unsigned char reg = 0;

	aw_dev_info(&i2c->dev, "%s enter\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		aw_dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}
	aw869xx = devm_kzalloc(&i2c->dev, sizeof(struct aw869xx), GFP_KERNEL);
	if (aw869xx == NULL)
		return -ENOMEM;
	aw869xx->dev = &i2c->dev;
	aw869xx->i2c = i2c;
	i2c_set_clientdata(i2c, aw869xx);
	/* aw869xx rst & int */
	if (np) {
		ret = aw869xx_parse_dt(&i2c->dev, aw869xx, np);
		if (ret) {
			aw_dev_err(&i2c->dev,
				   "%s: failed to parse device tree node\n",
				   __func__);
			goto err_parse_dt;
		}
	} else {
		aw869xx->reset_gpio = -1;
		aw869xx->irq_gpio = -1;
	}
	if (gpio_is_valid(aw869xx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw869xx->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw869xx_rst");
		if (ret) {
			aw_dev_err(&i2c->dev, "%s: rst request failed\n",
				   __func__);
			goto err_reset_gpio_request;
		}
	}
	if (gpio_is_valid(aw869xx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw869xx->irq_gpio,
					    GPIOF_DIR_IN, "aw869xx_int");
		if (ret) {
			aw_dev_err(&i2c->dev, "%s: int request failed\n",
				   __func__);
			goto err_irq_gpio_request;
		}
	}
	/* aw869xx chip id */
	ret = aw869xx_read_chipid(aw869xx);
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s: aw869xx_read_chipid failed ret=%d\n",
			   __func__, ret);
		goto err_id;
	}
	/* chip qualify */
	ret = aw869xx_i2c_read(aw869xx, 0x64, &reg);
	if (ret < 0) {
		aw_dev_err(&i2c->dev,
			   "%s: failed to read register 0x64: %d\n",
			   __func__, ret);
	}
	if (!(reg & 0x80)) {
		aw_dev_err(&i2c->dev, "%s:unqualified chip!\n", __func__);
		goto err_qualify;
	}

	/* register codec */
	dai = devm_kzalloc(&i2c->dev, sizeof(aw869xx_dai), GFP_KERNEL);
	if (!dai)
		goto err_dai_kzalloc;

	memcpy(dai, aw869xx_dai, sizeof(aw869xx_dai));
	aw_dev_info(&i2c->dev, "%s: dai->name(%s)\n", __func__, dai->name);

	ret = aw_componet_codec_ops.aw_snd_soc_register_codec(&i2c->dev,
							      &soc_codec_dev_aw869xx,
							      dai,
							      ARRAY_SIZE
							      (aw869xx_dai));
	if (ret < 0) {
		aw_dev_err(&i2c->dev, "%s failed to register aw869xx: %d\n",
			   __func__, ret);
		goto err_register_codec;
	}
	/* aw869xx irq */
	if (gpio_is_valid(aw869xx->irq_gpio) &&
	    !(aw869xx->flags & AW869XX_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw869xx_interrupt_setup(aw869xx);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw869xx->irq_gpio),
						NULL, aw869xx_irq, irq_flags,
						"aw869xx", aw869xx);
		if (ret != 0) {
			aw_dev_err(&i2c->dev,
				   "%s: failed to request IRQ %d: %d\n",
				   __func__, gpio_to_irq(aw869xx->irq_gpio),
				   ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw869xx->flags |= AW869XX_FLAG_SKIP_INTERRUPTS;
	}
	dev_set_drvdata(&i2c->dev, aw869xx);
	aw869xx_vibrator_init(aw869xx);
	aw869xx_haptic_init(aw869xx);
	aw869xx_ram_work_init(aw869xx);
	aw_dev_info(&i2c->dev, "%s probe completed successfully!\n", __func__);
	return 0;
 err_irq:
	aw_componet_codec_ops.aw_snd_soc_unregister_codec(&i2c->dev);
 err_register_codec:
	devm_kfree(&i2c->dev, dai);
	dai = NULL;
 err_dai_kzalloc:
 err_qualify:
 err_id:
	if (gpio_is_valid(aw869xx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw869xx->irq_gpio);
 err_irq_gpio_request:
	if (gpio_is_valid(aw869xx->reset_gpio))
		devm_gpio_free(&i2c->dev, aw869xx->reset_gpio);
 err_reset_gpio_request:
 err_parse_dt:
	devm_kfree(&i2c->dev, aw869xx);
	aw869xx = NULL;
	return ret;
}

static int aw869xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw869xx *aw869xx = i2c_get_clientdata(i2c);

	aw_dev_info(aw869xx->dev, "%s enter\n", __func__);
	devm_free_irq(&i2c->dev, gpio_to_irq(aw869xx->irq_gpio), aw869xx);
	if (gpio_is_valid(aw869xx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw869xx->irq_gpio);

	aw_componet_codec_ops.aw_snd_soc_unregister_codec(&i2c->dev);

	if (gpio_is_valid(aw869xx->reset_gpio))
		devm_gpio_free(&i2c->dev, aw869xx->reset_gpio);
	devm_kfree(&i2c->dev, aw869xx);
	aw869xx = NULL;

	return 0;
}

static int __maybe_unused aw869xx_suspend(struct device *dev)
{
	int ret = 0;
	struct aw869xx *aw869xx = dev_get_drvdata(dev);

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	mutex_unlock(&aw869xx->lock);

	return ret;
}

static int __maybe_unused aw869xx_resume(struct device *dev)
{
	int ret = 0;
	return ret;
}

// static SIMPLE_DEV_PM_OPS(aw869xx_pm_ops, aw869xx_suspend, aw869xx_resume);

static const struct i2c_device_id aw869xx_i2c_id[] = {
	{AW869XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw869xx_i2c_id);

static const struct of_device_id aw869xx_dt_match[] = {
	{.compatible = "awinic,awinic_haptic"},
	{},
};

static struct i2c_driver aw869xx_i2c_driver = {
	.driver = {
		   .name = AW869XX_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw869xx_dt_match),
		   },
	.probe = aw869xx_i2c_probe,
	.remove = aw869xx_i2c_remove,
	.id_table = aw869xx_i2c_id,
};

static int __init aw869xx_i2c_init(void)
{
	int ret = 0;

	pr_info("aw869xx driver version %s\n", AW869XX_VERSION);
	ret = i2c_add_driver(&aw869xx_i2c_driver);
	if (ret) {
		pr_err("fail to add aw869xx device into i2c\n");
		return ret;
	}
	return 0;
}

module_init(aw869xx_i2c_init);

static void __exit aw869xx_i2c_exit(void)
{
	i2c_del_driver(&aw869xx_i2c_driver);
}

module_exit(aw869xx_i2c_exit);

MODULE_DESCRIPTION("AW869XX Haptic Driver");
MODULE_LICENSE("GPL v2");
