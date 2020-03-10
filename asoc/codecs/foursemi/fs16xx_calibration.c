#include "fs16xx.h"
#include "fs16xx_regs.h"
#include "fs16xx_calibration.h"

static int calib_r0 = R0_DEFAULT;
static int calib_allow_low = R0_ALLOWANCE_LOW;
static int calib_allow_high = R0_ALLOWANCE_HIGH;
static int calib_r0_step = FS16XX_CALIB_OTP_R25_STEP;
static int calib_r0_delta = OTP_MIN_DELTA;


//#ifdef USE_RAS_MODE
static int calib_r0_ras = R0_DEFAULT_RAS;
static int calib_allow_low_ras = R0_ALLOWANCE_LOW_RAS;
static int calib_allow_high_ras = R0_ALLOWANCE_HIGH_RAS;
static int calib_r0_step_ras = FS16XX_CALIB_OTP_R25_STEP_RAS;
//#endif

Fs16xx_Error_t fs16xx_calibration(Fs16xx_devId_t id, int force, int storeResult) {
    int count = 0, maxTryCount = 100;
    unsigned short val, preVal = 0, minVal = 0;
    unsigned int done = 0;
    const int zmReadCount = 10;
    Fs_Dev_State *dev_state = NULL;

    PRINT("%s enter. id=%d force=%d storeResult=%d", __func__, id, force, storeResult);

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }

    if(force || dev_state->calib_state == Fs16xx_Calib_NotStarted
        || dev_state->calib_state == Fs16xx_Calib_Failed
        || dev_state->calib_state == Fs16xx_Calib_OTP_Failed) {

        fs16xx_write_register16(id, FS16XX_OTPACC_REG, FS16XX_OTP_ACC_KEY2);

        // Start calibration
        fs16xx_write_register16(id, FS16XX_ZMCONFIG_REG, 0x0010);

        fs16xx_write_register16(id, 0xAF, CALIB_REG_CALIB_MODE_AF);
        fs16xx_write_register16(id, FS16XX_AUDIOCTRL_REG, 0x8F00);
        fs16xx_write_register16(id, 0xC9, 0x0000);
        fs16xx_write_register16(id, 0xCA, 0x0000);
        fs16xx_write_register16(id, 0xCB, 0x0000);
        fs16xx_write_register16(id, 0xCC, 0x0000);
        fs16xx_write_register16(id, 0xB9, 0xFFFF);
        fs16xx_write_register16(id, 0xBA, CALIB_REG_NORMAL_BA);

        if(!force && dev_state->calib_state == Fs16xx_Calib_NotStarted) {
            // Sleep during the first time calibration
            Sleep(2500);
        }

        (dev_state->calib_result).calib_count++;
        dev_state->calib_state = Fs16xx_Calib_OnGoing;

        while((!done) && (maxTryCount > 0)) {
            Sleep(100);
            maxTryCount--;
            if(Fs16xx_Error_OK == fs16xx_read_register16(id, FS16XX_ZMDATA_REG, &val)) {
                if(val != 0 && val != 0xFFFF) {
                    if (preVal == 0 || abs(preVal - val) > SPK_CALIBRATION_MAX_ZMDELTA) {
                        preVal = val;
                        count = 1; // Reset counter
                        minVal = val;
                    } else {
                        count++;
                        if(val < minVal) {
                            minVal = val;
                        }
                    }
                }
            }
            if(count >= zmReadCount) {
                done = 1;
            }
        }

        if(done && minVal != 0) {
            dev_state->calib_state = fs16xx_calc_coef_from_zm(id, minVal, storeResult);
            if(Fs16xx_Calib_OK == dev_state->calib_state) {
                fs16xx_write_register16(id, FS16XX_AUDIOCTRL_REG, 0xFF00);
                // Calibration OK, exit calibration
                fs16xx_write_register16(id, FS16XX_ZMCONFIG_REG, 0x0000);

                fs16xx_write_register16(id, 0xAF, CALIB_REG_NORMAL_AF);
            }

            if(Fs16xx_Calib_OTP_ExcceedMaxTry == dev_state->calib_state) {
                // Calibration was done. However, OTP write failed due to counter limitation.
                // In this case, we will NOT exit calibration mode in order to report calibration failure.
            }
        } else {
            // Calibration failed
            dev_state->calib_state = Fs16xx_Calib_Failed;
            PRINT_ERROR("%s calibration failed. done=%d minVal=0x%04X", __func__, done, minVal);
        }
    } else {
        fs16xx_write_register16(id, FS16XX_OTPACC_REG, 0);
        return Fs16xx_Error_Calib_Error;
    }

    PRINT("%s calibration end, result = %s.", __func__, dev_state->calib_state == Fs16xx_Calib_OK ? "OK" : "Failed");

    fs16xx_write_register16(id, FS16XX_OTPACC_REG, 0);
    return Fs16xx_Error_OK;
}

Fs16xx_Calib_State_t fs16xx_check_calibration(Fs16xx_devId_t id) {
#ifndef FS16XX_CALIB_STORE_TO_OTP
    FILE *fp;
#endif
    int count;
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    unsigned short value = 0, valueC4 = 0;
    unsigned int r25;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_Calib_Result_t *calib_result;

    DEBUGPRINT("%s enter. id=%d", __func__, id);

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        //return Fs16xx_Error_StateInvalid;
        return Fs16xx_Calib_Unknown;
    }
    calib_result = &(dev_state->calib_result);

    PRINT("%s enter id=%d, calib_state=%d", __func__, id, dev_state->calib_state);

    if(Fs16xx_Calib_Unknown != dev_state->calib_state) {
        return dev_state->calib_state;
    }

    // Apply values from preset
    fs16xx_spkset_to_calib_coef(dev_state->ras,dev_state->spk_set);

    calib_result->calib_count = 0;

    err |= fs16xx_read_register16(id, FS16XX_PLLCTRL4_REG, &valueC4);
    err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, 0x000F);

    err |= fs16xx_write_register16(id, FS16XX_OTPACC_REG, FS16XX_OTP_ACC_KEY2);

#ifdef FS16XX_CALIB_STORE_TO_OTP
    // Read from 0xE8 first
    err |= fs16xx_read_register16(id, 0xE8, &value);
    if(Fs16xx_Error_OK != err) {
        PRINT_ERROR("%s failed to read calibration result by otp cmd.", __func__);
    }

    if(value == OTP_INIT_VALUE) {
        count = 0;
    } else {
        count = fs16xx_get_otp_write_count(value & 0xFF);
    }
    PRINT("%s read calibration result = 0x%04x count=%d.", __func__, value, count);
    // Calibration Count
    if(count > 0) {
        PRINT("%s calibrated already done.", __func__);
        calib_result->calib_count = count;
    } else {
        // Need calibration
        dev_state->calib_state = Fs16xx_Calib_NotStarted;
        PRINT("%s calibrated not started.", __func__);
        goto calib_exit;
    }

    r25 = fs16xx_byte_to_r25(dev_state->ras,(unsigned char)((value >> 8) & 0xFF));
    PRINT("%s read calibration result from otp r25=%d.", __func__, r25);
    dev_state->calib_state = fs16xx_update_coef_from_r25(id, r25);

    // ADC EN and TS EN should be set when osc is enabled.
    fs16xx_write_register16(id, FS16XX_ADCTIME_REG, CALIB_REG_NORMAL_BA);
    fs16xx_write_register16(id, FS16XX_TSCTRL_REG, CALIB_REG_NORMAL_AF);
#else
    fp = fopen( FS16XX_PARAM_DIR FS16XX_CALIB_RESULT_FILE_NAME, "rb");
    if (!fp)
    {
        PRINT_ERROR("%s read calibration result failed.", __func__);
        dev_state->calib_state = Fs16xx_Calib_NotStarted;
        goto calib_exit;
    }

    count = fread( (void*)calib_result, 1, sizeof(Fs16xx_Calib_Result_t), fp );
    fclose(fp);

    PRINT("%s read calibration file, count=%d.", __func__, count);

    if(count != sizeof(Fs16xx_Calib_Result_t)) {
        // Calibration result file corrupted.
        PRINT_ERROR("%s read calibration result corrupted.", __func__);
        dev_state->calib_state = Fs16xx_Calib_NotStarted;
        goto calib_exit;
    }

    fs16xx_calc_coef_from_zm(id, calib_result->calib_result_zm, 0);
#endif

calib_exit:
    err |= fs16xx_write_register16(id, FS16XX_OTPACC_REG, 0);
    err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, valueC4);

    DEBUGPRINT("%s Exit.", __func__);

    return dev_state->calib_state;
}

Fs16xx_Error_t fs16xx_get_calibration_result(Fs16xx_devId_t id, int *r25) {
    Fs_Dev_State *dev_state = NULL;

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }

    if(dev_state->calib_state != Fs16xx_Calib_OK) {
        PRINT_ERROR("%s calibration invalid result: %d", __func__, dev_state->calib_state);
        return Fs16xx_Error_Calib_Error;
    }

    *r25 = (int)(dev_state->calib_result.r25);
    PRINT("%s r25=%d.", __func__, dev_state->calib_result.r25);
    return Fs16xx_Error_OK;
}

int fs16xx_otp_is_busy(Fs16xx_devId_t id) {
    int retry = 20;
    unsigned short value = 0;

    Sleep(5);
    while(retry > 0) {
        // Checking OTP busy bit
        fs16xx_read_register16(id, FS16XX_OTPCMD_REG, &value);
        if((value & FS16XX_OTPCMD_REG_BUSY_MSK) == 0) {
            return 0;
        }
        Sleep(1);
        retry--;
    }
    // Return non-0, otp op timeout...
    return -1;
}

int fs16xx_get_otp_write_count(unsigned short val) {
    // OTP offset start from 0x10 to 0x20(Totally 16)
    return (val >= OTP_OP_COUNTER_OFFSET) ? (val - OTP_OP_COUNTER_OFFSET + 1): 0;
}

unsigned char fs16xx_r25_to_byte(bool ras,int r25) {
    unsigned char val;
    if(ras){
        val = (unsigned char)((int)abs(r25 - calib_r0_ras) * FS16XX_CALIB_STEP_MAGNIF_FACTOR / calib_r0_step_ras);
        if (val > 0x7F)  val = 0x7F;
        val = (r25 >= calib_r0_ras) ? val : (unsigned char)(val | 0x80);
    } else {
        val = (unsigned char)((int)abs(r25 - calib_r0) * FS16XX_CALIB_STEP_MAGNIF_FACTOR / calib_r0_step);
        if (val > 0x7F) val = 0x7F;
        val = (r25 >= calib_r0) ? val : (unsigned char)(val | 0x80);
    }

    // For Fs1801/Fs1818 otp value cannot be 0xFF(0xFF is reserved value).
    if(0xFF == val) val -= 1;
    return val;
}

int fs16xx_byte_to_r25(bool ras,unsigned char b) {
   if(ras){
        if((b & 0x80) != 0) {
            // minus value
            return calib_r0_ras - calib_r0_step_ras * (b & 0x7F) / FS16XX_CALIB_STEP_MAGNIF_FACTOR;
        }
        return calib_r0_ras + calib_r0_step_ras * b / FS16XX_CALIB_STEP_MAGNIF_FACTOR;
    } else {
        if((b & 0x80) != 0) {
            // minus value
            return calib_r0 - calib_r0_step * (b & 0x7F) / FS16XX_CALIB_STEP_MAGNIF_FACTOR;
        }
        return calib_r0 + calib_r0_step * b / FS16XX_CALIB_STEP_MAGNIF_FACTOR;
    }
}

int fs16xx_spkset_to_calib_coef(bool ras, unsigned short spkset) {
    int spkset_idx = (spkset & FS16XX_SPKSET_REG_SPKR_MSK) >> FS16XX_SPKSET_REG_SPKR_POS;
    int r = 0;
    if( ras ){
        r=32;
    } else {
        switch(spkset_idx) {
            case 0:
                r = 4;
                break;
            case 1:
                r = 6;
                break;
            case 2:
                r = 7;
                break;
            case 3:
            default:
                r = 8;
                break;
        }
    }
    if(ras){
        //#ifdef USE_RAS_MODE
        calib_r0_ras = r * MAGNIF_FACTOR;;
        calib_allow_low_ras = (r * R0_ALLOWANCE_LOW_RAS) / 32;
        calib_allow_high_ras = (r * R0_ALLOWANCE_HIGH_RAS) / 32;
        calib_r0_step_ras = (r * FS16XX_CALIB_OTP_R25_STEP_RAS) / 32;
        //#endif
    } else {
        calib_r0 = r * MAGNIF_FACTOR;
        calib_allow_low = (r * R0_ALLOWANCE_LOW) / 8;
        calib_allow_high = (r * R0_ALLOWANCE_HIGH) / 8;
        calib_r0_step = (r * FS16XX_CALIB_OTP_R25_STEP) / 8;
	    calib_r0_delta = (r * OTP_MIN_DELTA) / 8;
    }

    if(ras){
        PRINT("%s calib_r0_ras=0x%X calib_allow_low_ras=0x%X calib_allow_high_ras=0x%X calib_r0_step_ras = 0x%X.", \
        __func__, calib_r0_ras, calib_allow_low_ras, calib_allow_high_ras, calib_r0_step_ras);
    } else {
        PRINT("%s calib_r0=0x%X calib_allow_low=0x%X calib_allow_high=0x%X calib_r0_step = 0x%X calib_r0_delta = %d.", \
        __func__, calib_r0, calib_allow_low, calib_allow_high, calib_r0_step,calib_r0_delta);
    }

    return r;
}

Fs16xx_Calib_State_t fs16xx_calc_coef_from_zm(Fs16xx_devId_t id, unsigned short zm, int storeResult) {
    int r25;
    unsigned short spk_err, spk_m24, spk_m6, spk_rec, ext_tempr, value = 0;
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_Calib_Result_t *calib_result;

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Calib_Failed;
    }
    calib_result = &(dev_state->calib_result);
    ext_tempr = (dev_state->tempr_sel >> 1);
    // Read spk_set everytime to make sure spk_set value updated.
    err |= fs16xx_read_register16(id, FS16XX_SPKSET_REG, &(dev_state->spk_set));
    // Apply values from preset
    fs16xx_spkset_to_calib_coef(dev_state->ras,dev_state->spk_set);

    if(zm != 0) {
        // Read 0xE6
        fs16xx_read_register16(id, FS16XX_OTPPG1W2_REG, &value);
        if(value != 0) {
            value = value & 0xFF;
        } else {
            PRINT_ERROR("%s OTPPG1W2_REG unexpected! Set default value.", __func__);
            value = RS_TRIM_DEFAULT;
        }
        r25 = value * RS2RL_RATIO * MAGNIF_FACTOR / zm;
    } else {
        r25 = 0xFFFF; // Invalid r25 value
    }

    // Update calibration result even if the result is not valid
    calib_result->r25 = r25;
    calib_result->calib_ver = FS16XX_CALIB_VERSION;
    calib_result->calib_result_zm = zm;

    PRINT("%s calibration result zm=0x%04X r25=%d.", __func__, zm, r25);
    if(dev_state->ras){
        if (r25 > calib_allow_low_ras && r25 < calib_allow_high_ras) {
            PRINT("%s Receiver r25=%d.", __func__, r25);
         } else {
            // Calibration result invalid, need to re-calibrate
             PRINT("%s Receiver invalid result r25=%d.", __func__, r25);
             return Fs16xx_Calib_Failed;
         }
    } else {
        if(r25 < calib_allow_high && r25 > calib_allow_low) {
             PRINT("%s Speaker r25=%d.",__func__,r25);
        } else {
            // Calibration result invalid, need to re-calibrate
             PRINT("%s Speaker invalid result r25=%d.", __func__, r25);
             return Fs16xx_Calib_Failed;
        }
    }
    spk_m24 = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + dev_state->tempr_coef * (((dev_state->tmax + 20)- ext_tempr))));

    spk_err = spk_m24 / 2;

    spk_m6 = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + dev_state->tempr_coef * (((dev_state->tmax) - ext_tempr))));

    spk_rec = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + dev_state->tempr_coef * (((dev_state->tmax - 10) - ext_tempr))));

    calib_result->spk_err = spk_err;
    calib_result->spk_m24 = spk_m24;
    calib_result->spk_m6 = spk_m6;
    calib_result->spk_recover = spk_rec;

    err |= fs16xx_write_register16(id, FS16XX_SPKERR_REG, spk_err);
    err |= fs16xx_write_register16(id, FS16XX_SPKM24_REG, spk_m24);
    err |= fs16xx_write_register16(id, FS16XX_SPKM6_REG, spk_m6);
    err |= fs16xx_write_register16(id, FS16XX_SPKRE_REG, spk_rec);

    fs16xx_write_register16(id, FS16XX_ADCTIME_REG, CALIB_REG_NORMAL_BA);
    fs16xx_write_register16(id, FS16XX_TSCTRL_REG, CALIB_REG_NORMAL_AF);

    if(storeResult) {
#ifdef FS16XX_CALIB_STORE_TO_OTP
        err |= fs16xx_write_to_otp(id, fs16xx_r25_to_byte(dev_state->ras,r25));
#else
        err |= fs16xx_save_calib_result(id, zm);
#endif
    }

    PRINT("%s calibration result spk_err=0x%04X spk_m24=0x%04X spk_m6=0x%04X spk_rec=0x%04X reg set-%s.",
        __func__, spk_err, spk_m24, spk_m6, spk_rec, (Fs16xx_Error_OK == err) ? "OK" : "Fail");
    return (Fs16xx_Error_OK == err) ? Fs16xx_Calib_OK : dev_state->calib_state;
}

Fs16xx_Calib_State_t fs16xx_update_coef_from_r25(Fs16xx_devId_t id, int r25) {
    unsigned short spk_err, spk_m24, spk_m6, spk_rec, value = 0;
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    unsigned int zm, ext_tempr;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_Calib_Result_t *calib_result;

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Calib_Failed;
    }
    calib_result = &(dev_state->calib_result);
    ext_tempr = (dev_state->tempr_sel >> 1);

    if(dev_state->ras){
        if (r25 < calib_allow_low_ras || r25 > calib_allow_high_ras) {
            // Calibration result invalide, need to re-calibrate
            PRINT_ERROR("%s calibration result out of range r25=%d.", __func__, r25);
            return Fs16xx_Calib_Failed;
        }
    } else {
        if (r25 < calib_allow_low || r25 > calib_allow_high) {
            // Calibration result invalide, need to re-calibrate
            PRINT_ERROR("%s calibration result out of range r25=%d.", __func__, r25);
            return Fs16xx_Calib_Failed;
        }
    }

    if(r25 != 0) {
        // Read 0xE6
        fs16xx_read_register16(id, FS16XX_OTPPG1W2_REG, &value);
        if(value != 0) {
            value = value & 0xFF;
        } else {
            PRINT_ERROR("%s OTPPG1W2_REG unexpected! Set default value.", __func__);
            value = RS_TRIM_DEFAULT;
        }
        zm = value * RS2RL_RATIO * MAGNIF_FACTOR / r25;
    } else {
        PRINT_ERROR("%s Invalid r25!", __func__);
        return Fs16xx_Calib_Failed;
    }
    spk_m24 = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + dev_state->tempr_coef * (((dev_state->tmax + 20) - ext_tempr))));

    spk_err = spk_m24 / 2;

    spk_m6 = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + dev_state->tempr_coef * (((dev_state->tmax) - ext_tempr))));

    spk_rec = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + dev_state->tempr_coef * (((dev_state->tmax - 10) - ext_tempr))));

    calib_result->spk_err = spk_err;
    calib_result->spk_m24 = spk_m24;
    calib_result->spk_m6 = spk_m6;
    calib_result->spk_recover = spk_rec;
    calib_result->r25 = r25;

    err |= fs16xx_write_register16(id, FS16XX_SPKERR_REG, spk_err);
    err |= fs16xx_write_register16(id, FS16XX_SPKM24_REG, spk_m24);
    err |= fs16xx_write_register16(id, FS16XX_SPKM6_REG, spk_m6);
    err |= fs16xx_write_register16(id, FS16XX_SPKRE_REG, spk_rec);

    PRINT("%s read calibration result spk_err=0x%04X spk_m24=0x%04X spk_m6=0x%04X spk_rec=0x%04X reg set-%s.",
        __func__, spk_err, spk_m24, spk_m6, spk_rec, (Fs16xx_Error_OK == err) ? "OK" : "Fail");
    return (Fs16xx_Error_OK == err) ? Fs16xx_Calib_OK : Fs16xx_Calib_Failed;
}


#ifdef FS16XX_CALIB_STORE_TO_OTP

Fs16xx_Error_t fs16xx_write_to_otp(Fs16xx_devId_t id, unsigned char valOTP) {
    int count, delta;
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    unsigned short value = 0, valPwr = 0, valueC0 = 0, valueC4 = 0;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_Calib_Result_t *calib_result;

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }
    calib_result = &(dev_state->calib_result);

    PRINT("%s enter valOTP=0x%02x.", __func__, valOTP);
    err |= fs16xx_read_register16(id, FS16XX_SYSCTRL_REG, &valPwr);
    // power up, amp off
    err |= fs16xx_write_register16(id, FS16XX_SYSCTRL_REG, 0x0000);

    err |= fs16xx_read_register16(id, FS16XX_PLLCTRL4_REG, &valueC4);
    err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, 0x000A);

    err |= fs16xx_read_register16(id, FS16XX_BSTCTRL_REG, &valueC0);
    err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, 0x0000);

    // OTP page
    err |= fs16xx_write_register16(id, FS16XX_OTPADDR_REG, 0x0010);
    // OTP read clear
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x0000);
    // OTP read
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x0001);

    if(fs16xx_otp_is_busy(id) != 0) {
        dev_state->calib_state= Fs16xx_Calib_OTP_Failed;
        PRINT_ERROR("%s, Error, otp busy bit not cleared!", __func__);
        goto otp_op_exit;
    }

    fs16xx_read_register16(id, FS16XX_OTPRDATA_REG, &value);
    if(value == OTP_INIT_VALUE) {
        count = 0;
    } else {
        count = fs16xx_get_otp_write_count(value & 0xFF);
    }
    PRINT("%s read calibration result = 0x%04x otp write count=%d.",
        __func__, value, count);

    delta = (int)fs16xx_byte_to_r25(dev_state->ras,valOTP)
        - fs16xx_byte_to_r25(dev_state->ras,(value >> 8) & 0xFF);
    if(delta < 0) delta *= -1;

    // OTP won't updated if calibrated r25 is not changed much(defined by otpDelta).
    if(dev_state->ras) {
        PRINT("%s delta = %d otpDelta_ras = %d.", __func__, delta, OTP_MIN_DELTA_RAS);
    } else {
        PRINT("%s delta = %d otpDelta = %d.", __func__, delta, calib_r0_delta);
    }
    if(dev_state->ras){
        if(count > 0 && delta < OTP_MIN_DELTA_RAS) {
            PRINT("%s recevier no need to write to otp.", __func__);
            goto otp_op_exit;
        }
    }else{
         if(count > 0 && delta < calib_r0_delta) {
            PRINT("%s no need to write to otp.", __func__);
            goto otp_op_exit;
        }
    }

    // Calibration Count
    if(count >= FS16XX_CALIB_MAX_USER_TRY) {
        PRINT_ERROR("%s calibrated count exceeds max.", __func__);
        calib_result->calib_count = count;
        dev_state->calib_state = Fs16xx_Calib_OTP_ExcceedMaxTry;
        err = Fs16xx_Error_Calib_Error;
        goto otp_op_exit;
    }

    err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, 0x000B);

    // enable Boost, and set it to follow mode
    err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, 0x0008);

    // Write OTP page
    err |= fs16xx_write_register16(id, FS16XX_OTPADDR_REG, 0x0010);

    // OTP write byte
    err |= fs16xx_write_register16(id, FS16XX_OTPWDATA_REG, (unsigned short)valOTP);

    // Clear OTP commands
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x0000);
    // OTP enable Boost 7.6V mode
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x2000);

    // Wait until ss_end
    if(fs16xx_wait_boost_ssend(fs16xx_get_codec_by_id(id)) != Fs16xx_Error_OK) {
        PRINT_ERROR("%s, Error, failed to wait for boost ssend!", __func__);
        goto otp_op_exit;
    }

    // Send single-byte write command
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x2002);

    // Wait for 5ms for otp operation to complete.
    Sleep(5);
    if(fs16xx_otp_is_busy(id) != 0) {
        dev_state->calib_state = Fs16xx_Calib_OTP_Failed;
        err = Fs16xx_Error_Calib_Error;
        goto otp_op_exit;
    }

    // Clear OTP commands
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x0000);

    Sleep(1);

    // disable boost
    err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, 0x0000);
    
    calib_result->calib_count = count + 1;

    // Read back
    err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, 0x000A);

    err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, 0x0000);

    // OTP page
    err |= fs16xx_write_register16(id, FS16XX_OTPADDR_REG, 0x0010);
    // OTP read clear
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x0000);
    // OTP read
    err |= fs16xx_write_register16(id, FS16XX_OTPCMD_REG, 0x0001);

    if(fs16xx_otp_is_busy(id) != 0) {
        dev_state->calib_state = Fs16xx_Calib_OTP_Failed;
        err = Fs16xx_Error_Calib_Error;
        goto otp_op_exit;
    }

    fs16xx_read_register16(id, FS16XX_OTPRDATA_REG, &value);
    if(((value >> 8) & 0xFF) == valOTP) {
        // Update count
        calib_result->calib_count = fs16xx_get_otp_write_count(value & 0xFF);
        PRINT("%s readback succeeded with count = %d.", __func__, calib_result->calib_count);
    } else {
        err = Fs16xx_Error_Calib_Error;
        PRINT_ERROR("%s calibrated read back failed with value = 0x%04X(expected = 0x%02X--). Should not happen.",
            __func__, value, valOTP);
    }
otp_op_exit:
    err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, valueC0);
    err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, valueC4);
    // Recover power state
    err |= fs16xx_write_register16(id, FS16XX_SYSCTRL_REG, valPwr);

    DEBUGPRINT("%s exit, error = %d.", __func__, err);

    return err;
}

#else

Fs16xx_Error_t fs16xx_save_calib_result(Fs16xx_devId_t id, unsigned short zmdata) {
    FILE *fp;
    int count;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_Calib_Result_t *calib_result;

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }
    calib_result = &(dev_state->calib_result);

    DEBUGPRINT("%s enter.", __func__);

    // Write to file
    fp = fopen( FS16XX_PARAM_DIR FS16XX_CALIB_RESULT_FILE_NAME, "wb");
    if (!fp)
    {
        PRINT_ERROR("%s open calibration file failed.", __func__);
        return Fs16xx_Error_Calib_Error;
    }

    calib_result.calib_count = g_calib_count;
    calib_result.calib_ver = FS16XX_CALIB_VERSION;
    calib_result.calib_result_zm = zmdata;
    count = fwrite( (void*)&calib_result, 1, sizeof(Fs16xx_Calib_Result_t), fp );
    fclose(fp);

    PRINT("%s write to calibration file, count=%d.", __func__, count);

    return Fs16xx_Error_OK;
}
#endif

Fs16xx_Error_t fs16xx_set_tempr_protection(Fs16xx_devId_t id, int enable) {
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_Calib_Result_t *calib_result;

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }
    calib_result = &(dev_state->calib_result);

    PRINT("%s enter. enable=%d", __func__, enable);
    if(enable) {
        // TS on
        err = fs16xx_write_register16(id, FS16XX_TSCTRL_REG, CALIB_REG_NORMAL_AF);
        if(Fs16xx_Calib_Unknown == dev_state->calib_state) {
            fs16xx_check_calibration(id);
        }
        if(Fs16xx_Calib_OK != dev_state->calib_state
            && Fs16xx_Calib_OnGoing != dev_state->calib_state
            && Fs16xx_Calib_OTP_ExcceedMaxTry != dev_state->calib_state) {
            fs16xx_calibration(id, 0, 1);
        }
    }else {
        // TS off
        err = fs16xx_write_register16(id, FS16XX_TSCTRL_REG, CALIB_REG_TSOFF_AF);
        // Delay enough time for ts off gracefully to avoid pop.
        Sleep(35);
        //fs16xx_wait_ts_off(id, 60);
    }

    DEBUGPRINT("%s exit.", __func__);
    return err;
}

