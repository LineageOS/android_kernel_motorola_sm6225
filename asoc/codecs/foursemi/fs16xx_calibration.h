/* 
* Copyright (C) 2016 Fourier Semiconductor Inc. 
*
*/

#ifndef FS16XX_CALIBRATION_H
#define FS16XX_CALIBRATION_H

#include "fs16xx_define.h"

#ifdef __cplusplus
extern "C" {
#endif


#define MAGNIF_FACTOR   0x400
#define RS2RL_RATIO     2700
// TODO: to be loaded from preset
#define RS_TRIM_DEFAULT 0x8F

#define R0_DEFAULT          (8 * MAGNIF_FACTOR)

#define R0_ALLOWANCE        30 // 30%
#define FS16XX_CALIB_STEP_MAGNIF_FACTOR   100
#define R0_ALLOWANCE_LOW    ((R0_DEFAULT * (100 - R0_ALLOWANCE)) / FS16XX_CALIB_STEP_MAGNIF_FACTOR)
#define R0_ALLOWANCE_HIGH   ((R0_DEFAULT * (100 + R0_ALLOWANCE)) / FS16XX_CALIB_STEP_MAGNIF_FACTOR)

#define FS16XX_CALIB_OTP_R25_STEP    ((int)(R0_ALLOWANCE * R0_DEFAULT) / 0x7F)

#define OTP_MIN_DELTA  (int)(R0_DEFAULT * 0.05f)

//RAS MODE
//#ifdef USE_RAS_MODE
#define R0_DEFAULT_RAS          (32 * MAGNIF_FACTOR)
#define R0_ALLOWANCE_RAS        20
#define R0_ALLOWANCE_LOW_RAS    ((R0_DEFAULT_RAS * (100 - R0_ALLOWANCE_RAS)) / FS16XX_CALIB_STEP_MAGNIF_FACTOR)
#define R0_ALLOWANCE_HIGH_RAS   ((R0_DEFAULT_RAS * (100 + R0_ALLOWANCE_RAS)) / FS16XX_CALIB_STEP_MAGNIF_FACTOR)
#define FS16XX_CALIB_OTP_R25_STEP_RAS    ((int)(R0_ALLOWANCE_RAS * R0_DEFAULT_RAS) / 0x7F)
#define OTP_MIN_DELTA_RAS  (int)(R0_DEFAULT_RAS * 0.05f)
//#endif

#define MAGNIF_TEMPR_COEF    0xFFFF

#define TMAX            120
#define TMIN            -20
#define SPKTMPRM24      115
#define SPKTMPRM6       110
#define SPKTMPRECOVERY  100

#define OTP_OP_COUNTER_OFFSET       16
#define OTP_INIT_VALUE              0xFF10

#define SPK_CALIBRATION_MAX_ZMDELTA 0x150

#define FS16XX_CALIB_VERSION  1

// Store calibration result to OTP, remove following macro to store calibration result to file system.
#define FS16XX_CALIB_STORE_TO_OTP   1

#define FS16XX_CALIB_RESULT_FILE_NAME "fs16xxcali.bin"

#define CALIB_REG_NORMAL_BA 0x0031
#define CALIB_REG_CALIB_MODE_AF 0x162B
#define CALIB_REG_NORMAL_AF 0x362B
#define CALIB_REG_TSOFF_AF  0x3623

/* DO NOT CHANGE THIS VALUE */
#define FS16XX_CALIB_MAX_USER_TRY   15

/* Type containing all the possible errors that can occur
 *
 */
enum Fs16xx_Calib_State {
    Fs16xx_Calib_OK = 0,
    Fs16xx_Calib_NotStarted = 1,
    Fs16xx_Calib_OnGoing = 2,
    Fs16xx_Calib_Failed = 3,
    Fs16xx_Calib_OTP_Failed = 4,
    Fs16xx_Calib_OTP_ExcceedMaxTry = 5,
    Fs16xx_Calib_Unknown = 10
};
typedef enum Fs16xx_Calib_State Fs16xx_Calib_State_t;

struct Fs16xx_Calib_Result {
    unsigned short calib_ver;
    unsigned short calib_count;
    unsigned short calib_result_zm;
    unsigned short spk_err;
    unsigned short spk_m24;
    unsigned short spk_m6;
    unsigned short spk_recover;
    int   r25;
};
typedef struct Fs16xx_Calib_Result Fs16xx_Calib_Result_t;

Fs16xx_Error_t fs16xx_calibration(Fs16xx_devId_t id, int force, int storeResult);

Fs16xx_Calib_State_t fs16xx_check_calibration(Fs16xx_devId_t id);

Fs16xx_Error_t fs16xx_get_calibration_result(Fs16xx_devId_t id, int *r25);

Fs16xx_Calib_State_t fs16xx_calc_coef_from_zm(Fs16xx_devId_t id, unsigned short zm, int storeResult);

Fs16xx_Calib_State_t fs16xx_update_coef_from_r25(Fs16xx_devId_t id, int r25);

int fs16xx_otp_is_busy(Fs16xx_devId_t id);

int fs16xx_get_otp_write_count(unsigned short val);

unsigned char fs16xx_r25_to_byte(bool ras,int r25);

int fs16xx_byte_to_r25(bool ras,unsigned char b);

int fs16xx_spkset_to_calib_coef(bool ras,unsigned short spkset);

Fs16xx_Error_t fs16xx_set_tempr_protection(Fs16xx_devId_t id, int enable);

#ifdef FS16XX_CALIB_STORE_TO_OTP
Fs16xx_Error_t fs16xx_write_to_otp(Fs16xx_devId_t id, unsigned char valOTP);
#else
Fs16xx_Error_t fs16xx_save_calib_result(Fs16xx_devId_t id, unsigned short zmdata);
#endif

#ifdef __cplusplus
}
#endif

#endif