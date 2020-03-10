/*
 * Fs16xx device driver header file.
 * Copyright (C) 2017 FourSemi Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __FS16XX_DRIVER_H__
#define __FS16XX_DRIVER_H__

#ifdef __KERNEL__
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#endif
#include "fs16xx_define.h"
#include "fs16xx_preset.h"
#include "fs16xx_calibration.h"

// Macro for kernel mode implementation
#define FS16XX_KERNEL_IMPL 

#define MAX_DEV_HANDLES 4


struct Fs16xx_device_handle {
    int in_use;
    unsigned char device_address;
    unsigned char device_id;
    unsigned char revision;
    void *fs16xx; /* Pointer to fs16xx_dev_t */
};

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))
#endif

struct fs16xx_regs {
	int reg;
	int value;
};

/* I2S_CONTROL bits for sample rates */
#define FS16XX_I2SCTRL_RATE_08000 (0<<FS16XX_I2SCTRL_REG_I2SSR_POS)
#define FS16XX_I2SCTRL_RATE_16000 (3<<FS16XX_I2SCTRL_REG_I2SSR_POS)
#define FS16XX_I2SCTRL_RATE_44100 (7<<FS16XX_I2SCTRL_REG_I2SSR_POS)
#define FS16XX_I2SCTRL_RATE_48000 (8<<FS16XX_I2SCTRL_REG_I2SSR_POS)

#define UNUSED(x) (void)(x)
#ifdef __KERNEL__
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 10, 40))
#define Sleep(ms) usleep(1000*(ms))
#else
#define Sleep(ms) usleep_range(1000*(ms),1000*(ms))
#endif
#elif defined(__ANDROID__)
#define Sleep(ms) usleep(ms * 1000)
#endif
#define ACS_DRC_CTRL_LEN 3

#define FS16XX_GIT_BRANCH   "tinno_qcom_fs1818_mono"
#define FS16XX_GIT_COMMIT   "47e83a9"
#define FS16XX_CODE_DATE    "20200217_1111"
#define FS16XX_CODE_VERSION "v1.3.2"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#define snd_soc_codec              snd_soc_component
#define snd_soc_add_codec_controls snd_soc_add_component_controls
#define snd_soc_register_codec     devm_snd_soc_register_component
#define snd_soc_unregister_codec   snd_soc_unregister_component
#define snd_soc_codec_driver       snd_soc_component_driver
#endif

struct _fs_dev_state {
    int                   cur_preset;
	bool ras;
    unsigned short        tmax;
    unsigned short        tempr_coef;
    unsigned short        tempr_sel;
    unsigned short        spk_set;
    unsigned short        adp_bst;
    unsigned short        bst_ctrl;
    unsigned short        ts_ctrl;
    unsigned short        acs_drc_ctrl[ACS_DRC_CTRL_LEN];
    Fs16xx_preset        *dev_preset;
    Fs16xx_Calib_State_t  calib_state;
    Fs16xx_Calib_Result_t calib_result;
};
typedef struct _fs_dev_state Fs_Dev_State;

struct fs16xx_priv {
#ifdef __KERNEL__
    struct i2c_client *control_data;
    struct regulator *vdd;
    struct snd_soc_codec *codec;
    struct workqueue_struct *fs16xx_wq;
    struct work_struct init_work;
    struct work_struct calib_work;
    struct delayed_work mode_work;
    struct work_struct load_preset;
    struct delayed_work delay_work;
    struct mutex i2c_rw_lock;
#elif defined(__ANDROID__)
    int fd; /* file handle of opening i2c node. */
    struct i2c_client *client[2]; /* i2c client for i2c operation in kernel. */
#endif
    int dsp_init;
    int speaker_imp;
    int sysclk;
    int mode;
    int mode_switched;
    int curr_mode;
    int cfg_preset;
    int ic_version;
    char const *fs16xx_dev;
    char const *fw_path;
    char const *fw_name;
    int is_spkr_prot_en;
    int update_eq;
    int update_cfg;
    int is_pcm_triggered;
    Fs_Dev_State dev_state;
    Fs16xx_devId_t id;
};
typedef struct fs16xx_priv fs16xx_dev_t;

struct fs16xx_pdata {
	int reset_gpio;
	int max_vol_steps;
	const char *fs_dev;
	const char *fw_path;
	const char *fw_name;
};

/* DSP init status */
#define FS16XX_DSP_INIT_PENDING	0
#define FS16XX_DSP_INIT_OK	    1
#define FS16XX_DSP_INIT_FAIL	-1

/* FsDbg print for Linux kernel */
#define PRINT(msg,va...)            pr_info(msg "\n", ##va) //pr_info
#define PRINT_ERROR(msg,va...)      pr_err(msg "\n", ##va)
#define DEBUGPRINT(msg,va...)       pr_debug(msg "\n", ##va)  //pr_debug

Fs16xx_Error_t fs16xx_write_register16(Fs16xx_devId_t id,
                    unsigned char subaddress,
                    unsigned short value);
Fs16xx_Error_t fs16xx_read_register16(Fs16xx_devId_t id, 
                    unsigned char subaddress, 
                    unsigned short *pValue);

Fs16xx_Error_t fs16xx_handle_is_open(Fs16xx_devId_t id);

Fs_Dev_State* fs16xx_get_dev_state_by_id(Fs16xx_devId_t id);

struct snd_soc_codec * fs16xx_get_codec_by_id(Fs16xx_devId_t id);

Fs16xx_Error_t fs16xx_wait_boost_ssend(struct snd_soc_codec *codec);

Fs16xx_Error_t fs16xx_wait_dsp_off(Fs16xx_devId_t id, int wait_for_adc, int max_try);

#endif
