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
#ifndef __FS16XX_DEFINE_H__
#define __FS16XX_DEFINE_H__

#define FS16XX_MAX_REGISTER 0xE8

#define I2C_RETRY_DELAY     5 /* ms */
#define I2C_RETRIES         20
#define PLL_SYNC_RETRIES    10
#define MTPB_RETRIES        5
#define MAX_DEV_COUNT       2

/* DSP Firmware Size in bytes*/
#define FS16XX_SPK_FW_SIZE			512
#define FS16XX_CFG_FW_SIZE			512
#define FS16XX_PST_FW_SIZE			512
#define FS16XX_COEFF_FW_SIZE		512
#define FS16XX_DEV_STATE_SIZE		24
#define FS16XX_SPK_EX_FW_SIZE		512

#define FS16XX_REG_CACHE_SIZE  193

#define FS16XX_RATES   (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|\
                         SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000)
#define FS16XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE|SNDRV_PCM_FMTBIT_S24_LE|SNDRV_PCM_FMTBIT_S32_LE|\
	                 SNDRV_PCM_FORMAT_MU_LAW|SNDRV_PCM_FORMAT_S24_3LE)

#define FIRMWARE_NAME_SIZE     128

#define FS16XX_MAX_I2C_SIZE	512

#define SYS_CLK_DEFAULT 1536000

#define FS16XX_DEV_INDEX_L 0
#define FS16XX_DEV_INDEX_R 1

// DSP init status
#define FS16XX_DSP_INIT_PENDING	    0
#define FS16XX_DSP_INIT_OK   	    1
#define FS16XX_DSP_INIT_FAILED 	    -1

/* Type containing all the possible errors that can occur
 *
 */
enum Fs16xx_Error {
    Fs16xx_Error_OK = 0,
    Fs16xx_Error_Bad_Parameter,
    Fs16xx_Error_NotOpen, 
    Fs16xx_Error_OutOfHandles,
    Fs16xx_Error_StateTimedOut,
    Fs16xx_Error_StateInvalid,

    Fs16xx_Error_Not_Implemented,
    Fs16xx_Error_Not_Supported,
    Fs16xx_Error_I2C_Fatal,
    Fs16xx_Error_I2C_NonFatal,
    Fs16xx_Error_Other = 1000,
    Fs16xx_Error_Not_FsDev = 1001,
    Fs16xx_Error_Calib_Error = 1002,
    Fs16xx_Error_Invalid_Preset = 1003
};
typedef enum Fs16xx_Error Fs16xx_Error_t;

enum Fs16xx_Channel {
    Fs16xx_Channel_L,   // I2S left channel
    Fs16xx_Channel_R,   // I2S right channel
    Fs16xx_Channel_L_R, // I2S (left + right channel)/2
    Fs16xx_Channel_Stereo
};
typedef enum Fs16xx_Channel Fs16xx_Channel_t;

enum Fs16xx_Mode {
    Fs16xx_Mode_Normal = 0,
    Fs16xx_Mode_RCV
};
typedef enum Fs16xx_Mode Fs16xx_Mode_t;

enum Fs16xx_PwrState {
    Fs16xx_PwrOn = 0,
    Fs16xx_PwrOff = 1
};
typedef enum Fs16xx_PwrState Fs16xx_PwrState_t;

enum Fs16xx_OutputSel{
    Fs16xx_OutputSel_None = 0,
    Fs16xx_OutputSel_DataL, // I2S left channel output
    Fs16xx_OutputSel_DataR, // I2S right channel output
    Fs16xx_OutputSel_TS // Test tone
};
typedef enum Fs16xx_OutputSel Fs16xx_OutputSel_t;

#define FS_DISABLE  0
#define FS_ENABLE   1

#define FS_UMUTE  0
#define FS_MUTE   1

typedef int Fs16xx_devId_t;

//#define FS_STEREO_MODE
//#define USE_RAS_MODE

#endif

