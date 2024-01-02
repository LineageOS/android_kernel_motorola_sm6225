/*
 * Fs16xx device driver implementation.
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
#include "fs16xx_regs.h"
#include "fs16xx_reg_def.h"
#include "fs16xx_calibration.h"
#include "fs16xx_preset.h"
#include "fs16xx.h"


static DEFINE_MUTEX(lr_lock);
static int stereo_mode;
//static const char *fs_dev_name = "Right";
static struct fs16xx_priv * g_fs16xx_priv[MAX_DEV_COUNT] = {NULL, NULL};


fs16xx_dev_t *get_drvdata_by_context(struct snd_soc_codec *codec)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
    return snd_soc_codec_get_drvdata(codec);
#else
    return snd_soc_component_get_drvdata(codec);
#endif
}

fs16xx_dev_t *get_drvdata_by_dai(struct snd_soc_dai *dai)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
    struct snd_soc_codec *codec = dai->codec;
#else
    struct snd_soc_codec *codec = dai->component;
#endif
    return get_drvdata_by_context(codec);
}

fs16xx_dev_t *get_drvdata_by_kctrl(struct snd_kcontrol *kcontrol)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#else
    struct snd_soc_codec *codec = snd_soc_kcontrol_component(kcontrol);
#endif
    return get_drvdata_by_context(codec);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
static int snd_soc_write(struct snd_soc_codec *codec,
        unsigned int reg, unsigned int val)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_context(codec);
    if (!fs16xx) {
        PRINT_ERROR("%s: invalid dev", __func__);
        return -EINVAL;
    }
    return fs16xx_write_register16(fs16xx->id, reg, val);
}

static unsigned int snd_soc_read(struct snd_soc_codec *codec,
        unsigned int reg)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_context(codec);
    unsigned short val;
    int ret;

    if (!fs16xx) {
        PRINT_ERROR("%s: invalid dev", __func__);
        return -EINVAL;
    }
    ret = fs16xx_read_register16(fs16xx->id, reg, &val);
    if (ret != Fs16xx_Error_OK) {
        return -1;
    }
    return val;
}
#endif

/*
 * I2C Read/Write Functions
 */
static int fs16xx_i2c_read(struct i2c_client *fs16xx_client,
                u8 reg, u8 *value, int len)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
         .addr = fs16xx_client->addr,
         .flags = 0,
         .len = 1,
         .buf = &reg,
         },
        {
         .addr = fs16xx_client->addr,
         .flags = I2C_M_RD,
         .len = len,
         .buf = value,
         },
    };

    do {
        err = i2c_transfer(fs16xx_client->adapter, msgs,
                            ARRAY_SIZE(msgs));
        if (err != ARRAY_SIZE(msgs))
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

    if (err != ARRAY_SIZE(msgs)) {
        dev_err(&fs16xx_client->dev, "read transfer error %d\n"
                , err);
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int fs16xx_i2c_write(struct i2c_client *fs16xx_client,
                u8 *value, u8 len)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
         .addr = fs16xx_client->addr,
         .flags = 0,
         .len = len,
         .buf = value,
         },
    };

    do {
        err = i2c_transfer(fs16xx_client->adapter, msgs,
                    ARRAY_SIZE(msgs));
        if (err != ARRAY_SIZE(msgs))
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

    if (err != ARRAY_SIZE(msgs)) {
        dev_err(&fs16xx_client->dev, "write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 10, 108))
static int fs16xx_bulk_write(struct snd_soc_codec *codec, unsigned int reg,
                const void *data, size_t len)
{
    struct fs16xx_priv *fs16xx = get_drvdata_by_context(codec);
    u8 chunk_buf[FS16XX_MAX_I2C_SIZE + 1];
    int offset = 0;
    int ret = 0;
    /* first byte is mem address */
    int remaining_bytes = len - 1;
    int chunk_size = FS16XX_MAX_I2C_SIZE;

    chunk_buf[0] = reg & 0xff;
    mutex_lock(&fs16xx->i2c_rw_lock);
    while ((remaining_bytes > 0)) {
        if (remaining_bytes < chunk_size)
            chunk_size = remaining_bytes;

        memcpy(chunk_buf + 1, data + 1 + offset, chunk_size);
        ret = fs16xx_i2c_write(fs16xx->control_data, chunk_buf,
                    chunk_size + 1);
        offset = offset + chunk_size;
        remaining_bytes = remaining_bytes - chunk_size;
    }

    mutex_unlock(&fs16xx->i2c_rw_lock);
    return ret;
}
#endif
/*
static int fs16xx_bulk_read(struct snd_soc_codec *codec, u8 reg,
                u8 *data, int len)
{
    struct fs16xx_priv *fs16xx = get_drvdata_by_context(codec);
    int ret = 0;
    int offset = 0;
    int remaining_bytes = len;
    int chunk_size = FS16XX_MAX_I2C_SIZE;

    mutex_lock(&fs16xx->i2c_rw_lock);
    while ((remaining_bytes > 0)) {
        if (remaining_bytes < chunk_size)
            chunk_size = remaining_bytes;
        ret = fs16xx_i2c_read(fs16xx->control_data, reg, data + offset,
                chunk_size);
        offset = offset + chunk_size;
        remaining_bytes = remaining_bytes - chunk_size;
    }
    mutex_unlock(&fs16xx->i2c_rw_lock);

    return ret;
}*/

static int fs16xx_write(struct snd_soc_codec *codec, unsigned int reg,
                unsigned int val)
{
    struct fs16xx_priv *fs16xx = get_drvdata_by_context(codec);
    u8 buf[3] = {0, 0, 0};
    int ret;
    buf[0] = (reg & 0xff);
    buf[1] = (val >> 8) & 0xff;
    buf[2] = (val & 0xff);

    if (!fs16xx) {
        PRINT_ERROR("%s: invalid param", __func__);
        return -EINVAL;
    }

    mutex_lock(&fs16xx->i2c_rw_lock);
    ret = fs16xx_i2c_write(fs16xx->control_data, buf, ARRAY_SIZE(buf));
    mutex_unlock(&fs16xx->i2c_rw_lock);
    return ret;
}

static unsigned int fs16xx_read(struct snd_soc_codec *codec, unsigned int reg)
{
    struct fs16xx_priv *fs16xx = get_drvdata_by_context(codec);
    u8 buf[3] = {0, 0, 0};
    int len = 2;
    int val = -EIO;

    if (!fs16xx) {
        PRINT_ERROR("%s: invalid param", __func__);
        return -EINVAL;
    }
    mutex_lock(&fs16xx->i2c_rw_lock);
    if (fs16xx_i2c_read(fs16xx->control_data, reg & 0xff, buf, len) == 0) {
        val = (buf[0] << 8 | buf[1]);
    }
    mutex_unlock(&fs16xx->i2c_rw_lock);
    return val;
}

Fs16xx_Error_t fs16xx_handle_is_open(Fs16xx_devId_t id) {
    Fs16xx_Error_t err = Fs16xx_Error_NotOpen;
    if(id >= 0 && id < MAX_DEV_COUNT && g_fs16xx_priv[id]) {
        err = Fs16xx_Error_OK;
    }
    return err;
}

Fs_Dev_State* fs16xx_get_dev_state_by_id(Fs16xx_devId_t id) {

    struct fs16xx_priv *fs16xx;
    if(id >= 0 && id < MAX_DEV_COUNT) {
        fs16xx = g_fs16xx_priv[id];
        if(fs16xx) {
            return &(fs16xx->dev_state);
        } else {
            return NULL;
        }
    }
    return NULL;
}

struct snd_soc_codec * fs16xx_get_codec_by_id(Fs16xx_devId_t id) {
    if(id >= 0 && id < MAX_DEV_COUNT && g_fs16xx_priv[id]) {
        return g_fs16xx_priv[id]->codec;
    } else {
        return NULL;
    }
}

Fs16xx_Error_t fs16xx_write_register16(Fs16xx_devId_t id,
                    unsigned char subaddress,
                    unsigned short value)
{
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    struct fs16xx_priv *fs16xx;
    int ret;
    u8 buf[3] = {0, 0, 0};
    buf[0] = (subaddress & 0xff);
    buf[1] = (value >> 8) & 0xff;
    buf[2] = (value & 0xff);

    if(id >= 0 && id < MAX_DEV_COUNT) {
        fs16xx = g_fs16xx_priv[id];
        if(fs16xx) {
            mutex_lock(&fs16xx->i2c_rw_lock);
            ret = fs16xx_i2c_write(fs16xx->control_data,
                                    buf, ARRAY_SIZE(buf));
            mutex_unlock(&fs16xx->i2c_rw_lock);
            if(ret < 0) {
                err = Fs16xx_Error_I2C_Fatal;
                PRINT_ERROR("%s: i2c error!", __func__);
            }
        } else {
            err = Fs16xx_Error_Bad_Parameter;
            PRINT_ERROR("%s: Invalid fs16xx device instance!", __func__);
        }
    } else {
        err = Fs16xx_Error_Bad_Parameter;
        PRINT_ERROR("%s: Invalid fs16xx codec instance!", __func__);
    }

    return err;
}

Fs16xx_Error_t fs16xx_read_register16(Fs16xx_devId_t id, 
                    unsigned char subaddress, 
                    unsigned short *pValue)
{
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    struct fs16xx_priv *fs16xx;
    int ret = 0;
    u8 buf[2] = {0, 0};

    //printk(KERN_INFO "%s: id: %d, g_fs16xx_priv[%d]: %p\n", __func__, id, id, g_fs16xx_priv[id]);
    if(id >= 0 && id < MAX_DEV_COUNT) {
        fs16xx = g_fs16xx_priv[id];
        if(fs16xx) {
            mutex_lock(&fs16xx->i2c_rw_lock);
            ret = fs16xx_i2c_read(fs16xx->control_data,
                                  subaddress & 0xff, buf, 2);
            mutex_unlock(&fs16xx->i2c_rw_lock);

            if(ret == 0) {
                *pValue = (buf[0] << 8 | buf[1]);
            } else {
                err = Fs16xx_Error_I2C_Fatal;
                PRINT_ERROR("%s: i2c error!", __func__);
            }
        } else {
            err = Fs16xx_Error_Bad_Parameter;
            PRINT_ERROR("%s: Invalid fs16xx device instance!", __func__);
        }
    } else {
        err = Fs16xx_Error_Bad_Parameter;
        PRINT_ERROR("%s: Invalid fs16xx codec instance!", __func__);
    }

    return err;
}

#if 0
static const struct snd_kcontrol_new fs16xx_left_snd_controls[] = {
    SOC_SINGLE("fs16xx Left device Right channel select",  FS16XX_I2SCTRL_REG, 3, 3, 0),
    SOC_SINGLE("fs16xx Left device MUTE",  FS16XX_DACCTRL_REG, FS16XX_DACCTRL_REG_MUTE_POS, 1, 0),
    SOC_SINGLE("fs16xx Left device DSP ALG",  FS16XX_DSPCTRL_REG, FS16XX_DSPCTRL_REG_DSPEN_POS, 1, 0),
};
static const struct snd_kcontrol_new fs16xx_right_snd_controls[] = {
    SOC_SINGLE("fs16xx Right device Left channel select",  FS16XX_I2SCTRL_REG, 3, 3, 0),
    SOC_SINGLE("fs16xx Right device MUTE",  FS16XX_DACCTRL_REG, FS16XX_DACCTRL_REG_MUTE_POS, 1, 0),
    SOC_SINGLE("fs16xx Right device DSP ALG",  FS16XX_DSPCTRL_REG, FS16XX_DSPCTRL_REG_DSPEN_POS, 1, 0),
};
#else

/*
 * ASOC controls
 */
static int fs16xx_get_preset_mode(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_kctrl(kcontrol);
    Fs_Dev_State *dev_state = &(fs16xx->dev_state);
    DEBUGPRINT("%s: enter codec name=%s preset=%d", __func__,
        fs16xx->fs16xx_dev, dev_state->cur_preset);
    mutex_lock(&lr_lock);
    ucontrol->value.integer.value[0] = dev_state->cur_preset;
    mutex_unlock(&lr_lock);
    return 0;
}

static int fs16xx_put_preset_mode(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_kctrl(kcontrol);
    int preset = ucontrol->value.integer.value[0];
    int id;

    if(fs16xx == NULL) {
        PRINT_ERROR("%s: Bad parameter.\n", __func__);
    }
    PRINT("%s: enter codec name=%s set preset=%d\n", __func__,
        fs16xx->fs16xx_dev, preset);

    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        id = FS16XX_DEV_INDEX_L;
    } else if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        id = FS16XX_DEV_INDEX_R;
    } else {
        PRINT_ERROR("%s: invalid device name!\n", __func__);
        return 1;
    }
    mutex_lock(&lr_lock);
    fs16xx_set_preset(id, preset);
    mutex_unlock(&lr_lock);
    return 1;
}

static int fs16xx_ext_put(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_kctrl(kcontrol);
    struct snd_soc_codec *codec = fs16xx->codec;
    u16 valPwr;
    u16 powerstate;
    int ret;

    valPwr = 0x1;
    powerstate = 0;
    // printk("%s,codec_name = %s\n" , __func__, codec->component.name);
    mutex_lock(&lr_lock);
    valPwr = snd_soc_read(codec, FS16XX_SYSCTRL_REG);
    powerstate = valPwr & FS16XX_SYSCTRL_REG_PWDN_MSK;
    PRINT("%s: valPwr = 0x%04x,powerstate = 0x%04x",__func__,valPwr,powerstate);
    if(powerstate){
        PRINT("%s: need to power up",__func__);
        valPwr &= (~FS16XX_SYSCTRL_REG_PWDN_MSK);
        snd_soc_write(codec, FS16XX_SYSCTRL_REG, valPwr);
    }
    mutex_unlock(&lr_lock);
    ret = snd_soc_put_volsw(kcontrol, ucontrol);
    return 0;
}

static const struct snd_kcontrol_new fs16xx_left_snd_controls[] = {
    SOC_SINGLE_EXT("fs16xx Left device Right channel select",  FS16XX_I2SCTRL_REG, 3, 3, 0,
        snd_soc_get_volsw,fs16xx_ext_put),
    SOC_SINGLE_EXT("fs16xx Left device MUTE",  FS16XX_DACCTRL_REG, FS16XX_DACCTRL_REG_MUTE_POS, 1, 0,
        snd_soc_get_volsw,fs16xx_ext_put),
    SOC_SINGLE_EXT("fs16xx Left device DSP ALG",  FS16XX_DSPCTRL_REG, FS16XX_DSPCTRL_REG_DSPEN_POS, 1, 0,
        snd_soc_get_volsw,fs16xx_ext_put),
    SOC_SINGLE_EXT("fs16xx Left preset mode", SND_SOC_NOPM, 0, 2, 0, fs16xx_get_preset_mode, fs16xx_put_preset_mode),
};
static const struct snd_kcontrol_new fs16xx_right_snd_controls[] = {
    SOC_SINGLE_EXT("fs16xx Right device Left channel select",  FS16XX_I2SCTRL_REG, 3, 3, 0,
        snd_soc_get_volsw,fs16xx_ext_put),
    SOC_SINGLE_EXT("fs16xx Right device MUTE",  FS16XX_DACCTRL_REG, FS16XX_DACCTRL_REG_MUTE_POS, 1, 0,
        snd_soc_get_volsw,fs16xx_ext_put),
    SOC_SINGLE_EXT("fs16xx Right device DSP ALG",  FS16XX_DSPCTRL_REG, FS16XX_DSPCTRL_REG_DSPEN_POS, 1, 0,
        snd_soc_get_volsw,fs16xx_ext_put),
    SOC_SINGLE_EXT("fs16xx Right preset mode", SND_SOC_NOPM, 0, 2, 0, fs16xx_get_preset_mode, fs16xx_put_preset_mode),
};
#endif

static Fs16xx_Error_t fs16xx_UpdateOTCTRL(struct snd_soc_codec *codec) {
    u16 value = 0, offset = 0;

    value = snd_soc_read(codec, FS16XX_OTPPG1W0_REG);
    offset = ((value & 0x80) == 0) ? (value & 0x7F) : (-1 * (value & 0x7F));

    value = snd_soc_read(codec, FS16XX_OTCTRL_REG);

    if ((offset + value) < 0xFFFF && (offset + value > 0)) {
        snd_soc_write(codec, FS16XX_OTCTRL_REG, (u16)(offset + value));
        PRINT("%s: update OTCTRL 0x%04x", __func__, (u16)(offset + value));
    } else {
        PRINT_ERROR("%s: mono mode!", __func__);
    }

    return Fs16xx_Error_StateTimedOut;
}

static Fs16xx_Error_t fs16xx_wait_clk_stable(struct snd_soc_codec *codec) {
    int max_try = 500;
    u16 value;

    while(max_try > 0) {
        value = snd_soc_read(codec, FS16XX_STATUS_REG);
        if((value & FS16XX_STATUS_REG_PLLS) != 0) {
            return Fs16xx_Error_OK;
        }
        max_try--;
        Sleep(1);
    }

    return Fs16xx_Error_StateTimedOut;
}

Fs16xx_Error_t fs16xx_wait_boost_ssend(struct snd_soc_codec *codec) {
    int max_try = 150;
    unsigned short value;

    while(max_try > 0) {
        value = snd_soc_read(codec, FS16XX_BSTCTRL_REG);
        if((value & FS16XX_BSTCTRL_REG_SSEND_MSK) != 0) {
            return Fs16xx_Error_OK;
        }
        max_try--;
        Sleep(1);
    }

    return Fs16xx_Error_StateTimedOut;
}

Fs16xx_Error_t fs16xx_wait_dsp_off(Fs16xx_devId_t id, int wait_for_adc, int max_try) {
    Fs16xx_Error_t err;
    unsigned short value;
    unsigned short flag = FS16XX_DIGSTAT_REG_DACRUN;
    if(wait_for_adc) {
        flag = FS16XX_DIGSTAT_REG_DACRUN | FS16XX_DIGSTAT_REG_ADCRUN;
    }

    while(max_try > 0) {
        err = fs16xx_read_register16(id, FS16XX_DIGSTAT_REG, &value);
        if(Fs16xx_Error_OK == err && ((value & flag) == 0)) {
            return Fs16xx_Error_OK;
        }
        max_try--;
        Sleep(1);
    }

    return Fs16xx_Error_StateTimedOut;
}

static void fs16xx_power(struct snd_soc_codec *codec, int on)
{
    u16 valPwr;
    fs16xx_dev_t *fs16xx = get_drvdata_by_context(codec);
    int id = -1;
    
    DEBUGPRINT("%s: %s enter on=%d", __func__, fs16xx->fs16xx_dev, on);
#ifndef FS_STEREO_MODE
    if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        PRINT_ERROR("%s: mono mode!", __func__);
        return;
    }
#endif
    valPwr = snd_soc_read(codec, FS16XX_SYSCTRL_REG);
    if (on) {
        snd_soc_write(codec, FS16XX_PLLCTRL4_REG, FS16XX_PLLCTRL_EN_ALL);

        valPwr &= (~FS16XX_SYSCTRL_REG_PWDN_MSK);
        /* Amplifier always on */
        valPwr |= FS16XX_SYSCTRL_REG_AMPE;
        snd_soc_write(codec, FS16XX_SYSCTRL_REG, valPwr);
    } else {
        valPwr |= FS16XX_SYSCTRL_REG_PWDN_MSK;
        snd_soc_write(codec, FS16XX_SYSCTRL_REG, valPwr); // Power off
        if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
            id = FS16XX_DEV_INDEX_L;
        } else if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
            id = FS16XX_DEV_INDEX_R;
        } else {
            PRINT_ERROR("%s: invalid device name!\n", __func__);
            return;
        }
        fs16xx_wait_dsp_off(id, 1, 50);

        /* Disable bandgap, osc, pll */
        snd_soc_write(codec, FS16XX_PLLCTRL4_REG, 0);
    }

    DEBUGPRINT("%s: valPwr=0x%04x", __func__, valPwr);
}

static void fs16xx_set_mute(struct snd_soc_codec *codec, int mute_state)
{
    u16 valPwr;
    fs16xx_dev_t *fs16xx = get_drvdata_by_context(codec);
    DEBUGPRINT("%s: enter mute_state = %d", __func__, mute_state);

    /* read sys ctrl register */
    valPwr = snd_soc_read(codec, FS16XX_SYSCTRL_REG);

    switch (mute_state) {
    case FS_UMUTE:
        /* unmute and enable amp */
        valPwr |= FS16XX_SYSCTRL_REG_AMPE;
        snd_soc_write(codec, FS16XX_SYSCTRL_REG, valPwr);
        break;
    case FS_MUTE:
        /* turn off amp */
        valPwr &= (~FS16XX_SYSCTRL_REG_AMPE);
        snd_soc_write(codec, FS16XX_SYSCTRL_REG, valPwr);
        break;
    default:
        break;
    }

    DEBUGPRINT("%s: write valPwr=0x%04X %s", __func__, valPwr, fs16xx->fs16xx_dev);
}

static int fs16xx_calibaration(fs16xx_dev_t *fs16xx, int write_otp)
{
    static int calibrationOK = 0;
    Fs16xx_Error_t result = Fs16xx_Error_OK;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);
    if(calibrationOK){
        if(!strncmp("left", fs16xx->fs16xx_dev, 4)) {
            PRINT("%s: left write_otp %d", __func__, write_otp);
            result = fs16xx_calibration(FS16XX_DEV_INDEX_L, 1, 0);
        } else if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
            PRINT("%s: right write_otp %d", __func__, write_otp);
            result = fs16xx_calibration(FS16XX_DEV_INDEX_R, 1, 0);
        }
    }else{
        if(!strncmp("left", fs16xx->fs16xx_dev, 4)) {
            PRINT("%s: left write_otp %d", __func__, write_otp);
            result = fs16xx_calibration(FS16XX_DEV_INDEX_L, 1, 1);
        } else if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
            PRINT("%s: right write_otp %d", __func__, write_otp);
            result = fs16xx_calibration(FS16XX_DEV_INDEX_R, 1, write_otp);
        }
    }
    if(Fs16xx_Error_OK == result) {
        calibrationOK = 1;
        return 0;
    } else  {
        return -101;
    }
}

static void fs16xx_work_read_imp(struct work_struct *work)
{
    fs16xx_dev_t *fs16xx =
            container_of(work, fs16xx_dev_t, calib_work);
    struct snd_soc_codec *codec = fs16xx->codec;
    u16 val;
    Fs16xx_devId_t id;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    if(!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        id = FS16XX_DEV_INDEX_L;
    } else if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        id = FS16XX_DEV_INDEX_R;
    } else {
        PRINT_ERROR("%s: invalid codec name=%s!!!", __func__, fs16xx->fs16xx_dev);
        return;
    }
    mutex_lock(&lr_lock);
    val = snd_soc_read(codec, FS16XX_STATUS_REG);

    if((val & FS16XX_STATUS_REG_PLLS) != 0) {
        fs16xx_get_calibration_result(id, &fs16xx->speaker_imp);
    }
    mutex_unlock(&lr_lock);
}

static void fs16xx_work_mode(struct work_struct *work)
{
    fs16xx_dev_t *fs16xx =
        container_of(work, fs16xx_dev_t,
                mode_work.work);

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);
    mutex_lock(&lr_lock);

    mutex_unlock(&lr_lock);
    return;
}

static void fs16xx_load_preset(struct work_struct *work)
{
    fs16xx_dev_t *fs16xx =
            container_of(work, fs16xx_dev_t, load_preset);
    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);
    mutex_lock(&lr_lock);

    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        fs16xx_load_presets(FS16XX_DEV_INDEX_L);
        fs16xx_set_preset(FS16XX_DEV_INDEX_L, 0);
    } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        fs16xx_load_presets(FS16XX_DEV_INDEX_R);
        fs16xx_set_preset(FS16XX_DEV_INDEX_R, 0);
    }
    mutex_unlock(&lr_lock);
}

/*
 * ASOC OPS
*/
static int fs16xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
                  int clk_id, unsigned int freq, int dir)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(codec_dai);

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    fs16xx->sysclk = freq;
    return 0;
}

static int fs16xx_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(codec_dai);

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    /* set master/slave audio interface */
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBS_CFS:
        /* default value */
        break;
    case SND_SOC_DAIFMT_CBM_CFM:
    default:
        /* only supports Slave mode */
        PRINT_ERROR("fs16xx: invalid DAI master/slave interface");
        return -EINVAL;
    }

    return 0;
}

static int fs16xx_hw_params(struct snd_pcm_substream *substream,
                 struct snd_pcm_hw_params *params,
                 struct snd_soc_dai *dai)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(dai);
    struct snd_soc_codec *codec = fs16xx->codec;
    u16 val;
    int bclk_ws_ratio;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    /* validate and set params */
   /* if (params_format(params) != SNDRV_PCM_FORMAT_S16_LE) {
        PRINT_ERROR("fs16xx: invalid pcm bit lenght");
        return -EINVAL;
    }*/
    PRINT("fs16xx: params_format = %x\n",params_format(params));

    mutex_lock(&lr_lock);
    switch(params_format(params)) {
        case SNDRV_PCM_FORMAT_S16_LE:
             snd_soc_write(codec, 0xC3, 0x04);
             break;
        case SNDRV_PCM_FORMAT_S24_LE:
        case SNDRV_PCM_FORMAT_S24_3LE:
        case SNDRV_PCM_FORMAT_S32_LE:
             snd_soc_write(codec, 0xC3, 0x08);
             break;
        default:
             PRINT_ERROR("fs16xx: invalid pcm bit lenght");
             snd_soc_write(codec, 0xC3, 0x08);
    }

    val = snd_soc_read(codec, FS16XX_I2SCTRL_REG);
    /* clear Sample rate bits */
    val = val & ~(FS16XX_I2SCTRL_REG_I2SSR_MSK);

    switch (params_rate(params)) {
    case 8000:
        val = val | FS16XX_I2SCTRL_RATE_08000;
        break;
    case 16000:
        val = val | FS16XX_I2SCTRL_RATE_16000;
        break;
    case 44100:
        val = val | FS16XX_I2SCTRL_RATE_44100;
        break;
    case 48000:
        val = val | FS16XX_I2SCTRL_RATE_48000;
        break;
    default:
        PRINT_ERROR("fs16xx: invalid sample rate = %d", params_rate(params));
        mutex_unlock(&lr_lock);
        return -EINVAL;
    }

    snd_soc_write(codec, FS16XX_I2SCTRL_REG, val);
    mutex_unlock(&lr_lock);

    /* calc bclk to ws freq ratio, fs16xx supports only 32, 48, 64 */
    bclk_ws_ratio = fs16xx->sysclk/params_rate(params);
    if (bclk_ws_ratio != 32 && bclk_ws_ratio != 48
            && bclk_ws_ratio != 64) {
        PRINT_ERROR("fs16xx: invalid bit clk to ws freq ratio %d:",
            bclk_ws_ratio);
        return -EINVAL;
    }

    return 0;
}

static int fs16xx_mute(struct snd_soc_dai *dai, int mute)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(dai);
    struct snd_soc_codec *codec = fs16xx->codec;

    DEBUGPRINT("%s: enter codec name=%s mute=%d", __func__, fs16xx->fs16xx_dev, mute);
#ifndef FS_STEREO_MODE
    if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        PRINT_ERROR("%s: mono mode!", __func__);
        return 0;
    }
#endif
    mutex_lock(&lr_lock);
    fs16xx_wait_clk_stable(codec);
    fs16xx_wait_boost_ssend(codec);
    if (mute) {
        cancel_delayed_work_sync(&fs16xx->mode_work);
    }

    if (mute) {
        if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
            fs16xx_set_tempr_protection(FS16XX_DEV_INDEX_L, FS_DISABLE);
        } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
            fs16xx_set_tempr_protection(FS16XX_DEV_INDEX_R, FS_DISABLE);
        }
        fs16xx_set_mute(codec, FS_MUTE);
    } else {
        fs16xx_set_mute(codec, FS_UMUTE);
        Sleep(10);
        if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
            fs16xx_set_tempr_protection(FS16XX_DEV_INDEX_L, FS_ENABLE);
        } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
            fs16xx_set_tempr_protection(FS16XX_DEV_INDEX_R, FS_ENABLE);
        }
    }
    mutex_unlock(&lr_lock);
    return 0;
}

static int fs16xx_startup(struct snd_pcm_substream *substream,
                   struct snd_soc_dai *dai)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(dai);

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    mutex_lock(&lr_lock);
    fs16xx_power(fs16xx->codec, 1);
    mutex_unlock(&lr_lock);
    return 0;
}

static void fs16xx_shutdown(struct snd_pcm_substream *substream,
                   struct snd_soc_dai *dai)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(dai);
    DEBUGPRINT("%s: enter", __func__);
    mutex_lock(&lr_lock);
    fs16xx_power(fs16xx->codec, 0);
    mutex_unlock(&lr_lock);
}

/* Trigger callback is atomic function, It gets called when pcm is started */
static int fs16xx_trigger(struct snd_pcm_substream *substream, int cmd,
                 struct snd_soc_dai *dai)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_dai(dai);
    int ret = 0;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        fs16xx->is_pcm_triggered = 1;
        /* if in bypass mode dont't do anything */
        if (fs16xx->is_spkr_prot_en)
            break;
        /* To initialize dsp all the I2S signals should be bought up,
         * so that the DSP's internal PLL can sync up and memory becomes
         * accessible. Trigger callback is called when pcm write starts,
         * so this should be the place where DSP is initialized
         */
        if (fs16xx->dsp_init == FS16XX_DSP_INIT_PENDING)
            queue_work(fs16xx->fs16xx_wq, &fs16xx->init_work);

        /* will need to read speaker impedence here if its not read yet
         * to complete the calibartion process. This step will enable
         * device to calibrate if its not calibrated/validated in the
         * factory. When the factory process is in place speaker imp
         * will be read from sysfs and validated.
         */
        else if (fs16xx->dsp_init == FS16XX_DSP_INIT_OK) {
            if ((fs16xx->mode_switched == 1) ||
                    (fs16xx->update_cfg == 1))
                queue_delayed_work(fs16xx->fs16xx_wq, &fs16xx->mode_work,
                        0);
            if (fs16xx->speaker_imp == 0)
                queue_work(fs16xx->fs16xx_wq,
                    &fs16xx->calib_work);

        }
        /* else nothing to do */
        break;
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        fs16xx->is_pcm_triggered = 0;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

/* Called from fs16xx stereo codec stub driver to set mute
 * on the individual codecs, this is needed to smother pops
 * in stereo configuration.
*/

/*int fs16xx_stereo_sync_set_mute(int mute)
{
    // TODO: sync mute
    return 0;
}*/

/*
 * SysFS support
 */

static ssize_t fs16xx_show_spkr_imp(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    fs16xx_dev_t *fs16xx =
                i2c_get_clientdata(to_i2c_client(dev));
    u16 val;
    Fs16xx_devId_t id;
    u32 r25 = 0;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);
    
    if(!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        id = FS16XX_DEV_INDEX_L;
    } else if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        id = FS16XX_DEV_INDEX_R;
    } else {
        PRINT_ERROR("%s: invalid codec name=%s!!!", __func__, fs16xx->fs16xx_dev);
        id = FS16XX_DEV_INDEX_R; // Read right channel
    }

    if (fs16xx->codec) {
        mutex_lock(&lr_lock);
        val = snd_soc_read(fs16xx->codec, FS16XX_STATUS_REG);

        if((val & FS16XX_STATUS_REG_PLLS) != 0) {
            fs16xx_get_calibration_result(id, &r25);
            fs16xx->speaker_imp = r25;
        }
        mutex_unlock(&lr_lock);
    }
    return scnprintf(buf, PAGE_SIZE, "%u\n", fs16xx->speaker_imp);
}

static ssize_t fs16xx_show_dev_state_info(struct file *filp,
                    struct kobject *kobj,
                  struct bin_attribute *attr,
                  char *buf, loff_t off, size_t count)
{
    fs16xx_dev_t *fs16xx =
                i2c_get_clientdata(kobj_to_i2c_client(kobj));
    u16 val;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    if (off >= attr->size)
        return 0;

    if (off + count > attr->size)
        count = attr->size - off;

    if (fs16xx->codec) {
        mutex_lock(&lr_lock);
        val = snd_soc_read(fs16xx->codec, FS16XX_STATUS_REG);
        mutex_unlock(&lr_lock);
        if(count > 4) {
            scnprintf(buf, count, "%04X", val);
        }
    }
    return 4;
}

static ssize_t fs16xx_show_spkr_imp_model(struct file *filp,
                    struct kobject *kobj,
                  struct bin_attribute *attr,
                  char *buf, loff_t off, size_t count)
{
    fs16xx_dev_t *fs16xx =
                i2c_get_clientdata(kobj_to_i2c_client(kobj));
    u16 val;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    if (off >= attr->size)
        return 0;

    if (off + count > attr->size)
        count = attr->size - off;

    if (fs16xx->codec) {
        mutex_lock(&lr_lock);
        val = snd_soc_read(fs16xx->codec, FS16XX_STATUS_REG);
        mutex_unlock(&lr_lock);
    }
    return 0;
}

static ssize_t fs16xx_show_spkr_exc_model(struct file *filp,
                    struct kobject *kobj,
                  struct bin_attribute *attr,
                  char *buf, loff_t off, size_t count)
{
    fs16xx_dev_t *fs16xx =
                i2c_get_clientdata(kobj_to_i2c_client(kobj));
    u16 val;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    if (off >= attr->size)
        return 0;

    if (off + count > attr->size)
        count = attr->size - off;

    if (fs16xx->codec) {
        mutex_lock(&lr_lock);
        val = snd_soc_read(fs16xx->codec, FS16XX_STATUS_REG);
        mutex_unlock(&lr_lock);
    }
    return 0;
}

static ssize_t fs16xx_show_config_preset(struct file *filp,
                    struct kobject *kobj,
                  struct bin_attribute *attr,
                  char *buf, loff_t off, size_t count)
{
    fs16xx_dev_t *fs16xx =
                i2c_get_clientdata(kobj_to_i2c_client(kobj));
    u16 val;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    if (off >= attr->size)
        return 0;

    if (off + count > (attr->size))
        count = (attr->size) - off;

    if (fs16xx->codec) {
        mutex_lock(&lr_lock);
        val = snd_soc_read(fs16xx->codec, FS16XX_STATUS_REG);
        mutex_unlock(&lr_lock);
    }
    return 0;
}

static ssize_t fs16xx_show_ic_temp(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    fs16xx_dev_t *fs16xx = i2c_get_clientdata(to_i2c_client(dev));
    u16 val;
    int temp = 0;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    if (fs16xx->codec) {
        mutex_lock(&lr_lock);
        val = snd_soc_read(fs16xx->codec, FS16XX_STATUS_REG);
        mutex_unlock(&lr_lock);
    }
    return scnprintf(buf, PAGE_SIZE, "%u\n", temp);
}

static ssize_t fs16xx_show_calib(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    fs16xx_dev_t *fs16xx = i2c_get_clientdata(to_i2c_client(dev));
    int val = 1;

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);
    mutex_lock(&lr_lock);
    val = fs16xx_calibaration(fs16xx, val);
    mutex_unlock(&lr_lock);

    return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t fs16xx_show_calib_result(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    fs16xx_dev_t *fs16xx = i2c_get_clientdata(to_i2c_client(dev));
    int val = 0;
    int id = 0;
    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        id = FS16XX_DEV_INDEX_L;
    }
    if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        id = FS16XX_DEV_INDEX_R;
    }
    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);
    mutex_lock(&lr_lock);
    fs16xx_get_calibration_result(id,&val);
    mutex_unlock(&lr_lock);

    return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

// Read reg interface for debugging
static ssize_t fs16xx_reg_read_bin_attr(struct file *filp,
                    struct kobject *kobj,
                  struct bin_attribute *attr,
                  char *buf, loff_t off, size_t count) 
{
    fs16xx_dev_t *fs16xx = i2c_get_clientdata(kobj_to_i2c_client(kobj));
    struct snd_soc_codec *codec = fs16xx->codec;
    int offset = 0;
    u16 regVal = 0;

    if(off + count > attr->size) {
        count = attr->size - off;
        //PRINT_ERROR("trim count to %d", count);
    }
    
    if (fs16xx->codec) {
        if(count >= FS16XX_MAX_REGISTER * 2) {
            for(offset = 0; offset <= FS16XX_MAX_REGISTER; offset++) {
                mutex_lock(&lr_lock);
                regVal = snd_soc_read(codec, offset);
                mutex_unlock(&lr_lock);
                buf[offset * 2 ] = (u8)((regVal >> 8) & 0xFF);
                buf[offset * 2 + 1] = (u8)((regVal) & 0xFF);
            }
        }
    }
    return count;
}

// Write reg interface for debugging
/***************************************
*   reg write command: address value
*   address should be in hex mode without 0x or \x
*   value should be in hex mode without 0x or \x
****************************************/
static ssize_t fs16xx_reg_write(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    fs16xx_dev_t *fs16xx = i2c_get_clientdata(client);
    struct snd_soc_codec *codec = fs16xx->codec;
    int reg=0, nVal=0;
    u16 val=0;

    if (fs16xx->codec) {
        DEBUGPRINT("%s: enter force right 0x35", __func__);
        //DEBUGPRINT("%s: count=%d buf=%s stereo mode=%d", __func__, count, buf, stereo_mode);
        if (count >= 3 && count <= 8) {
            if(sscanf(buf, "%x %x", &reg, &nVal) == 2) {
                DEBUGPRINT("%s reg=0x%x val=0x%x", __func__, reg, nVal);
                val = (u16)nVal;
                // write single reg
                mutex_lock(&lr_lock);
                snd_soc_write(codec, reg, val);
                mutex_unlock(&lr_lock);
            } else {
                PRINT_ERROR("%s parsing failed", __func__ );
            }
        }
    }

    return count;
}

static const struct bin_attribute fs16xx_raw_bin_attr[] = {
{
    .attr = {.name = "device_state", .mode = S_IRUGO},
    .read = fs16xx_show_dev_state_info,
    .size = FS16XX_DEV_STATE_SIZE,
},
{
    .attr = {.name = "spkr_imp_model", .mode = S_IRUGO},
    .read = fs16xx_show_spkr_imp_model,
    .size = FS16XX_SPK_FW_SIZE - 1,
},
{
    .attr = {.name = "config_preset", .mode = S_IRUGO},
    .read = fs16xx_show_config_preset,
    .size = FS16XX_CFG_FW_SIZE + FS16XX_PST_FW_SIZE - 2,
},
{
    .attr = {.name = "spkr_exc_model", .mode = S_IRUGO},
    .read = fs16xx_show_spkr_exc_model,
    .size = FS16XX_SPK_EX_FW_SIZE - 1,
},
{
    .attr = {.name = "fs_reg_read", .mode = S_IRUGO},
    .read = fs16xx_reg_read_bin_attr,
    .size = FS16XX_MAX_REGISTER * sizeof(u16),
}
};

static DEVICE_ATTR(ic_temp, S_IRUGO,
           fs16xx_show_ic_temp, NULL);

static DEVICE_ATTR(spkr_imp, S_IRUGO,
           fs16xx_show_spkr_imp, NULL);


static DEVICE_ATTR(force_calib, S_IRUGO,
           fs16xx_show_calib, NULL);
static DEVICE_ATTR(calib_result, S_IRUGO,
           fs16xx_show_calib_result, NULL);

static DEVICE_ATTR(fs_reg_write, S_IWUSR,
           NULL, fs16xx_reg_write);

static struct attribute *fs16xx_attributes[] = {
    &dev_attr_spkr_imp.attr,
    &dev_attr_force_calib.attr,
    &dev_attr_calib_result.attr,
    &dev_attr_ic_temp.attr,
    &dev_attr_fs_reg_write.attr,
    NULL
};

static const struct attribute_group fs16xx_attr_group = {
    .attrs = fs16xx_attributes,
};


static const struct snd_soc_dai_ops fs16xx_ops = {
    .hw_params = fs16xx_hw_params,
    .digital_mute = fs16xx_mute,
    .set_fmt = fs16xx_set_dai_fmt,
    .set_sysclk = fs16xx_set_dai_sysclk,
    .startup    = fs16xx_startup,
    .shutdown   = fs16xx_shutdown,
    .trigger = fs16xx_trigger,
};

static struct snd_soc_dai_driver fs16xx_left_dai = {
    .name = "fs16xx_codec_left",
    .playback = {
             .stream_name = "Playback",
             .channels_min = 1,
             .channels_max = 2,
             .rates = FS16XX_RATES,
             .formats = FS16XX_FORMATS,},
    .ops = &fs16xx_ops,
    .symmetric_rates = 1,
};

static struct snd_soc_dai_driver fs16xx_right_dai = {
    .name = "fs16xx_codec_right",
    .playback = {
             .stream_name = "Playback",
             .channels_min = 1,
             .channels_max = 2,
             .rates = FS16XX_RATES,
             .formats = FS16XX_FORMATS,},
    .ops = &fs16xx_ops,
    .symmetric_rates = 1,
};

static int fs16xx_probe(struct snd_soc_codec *codec)
{
    fs16xx_dev_t *fs16xx = get_drvdata_by_context(codec);
    int i;
#ifdef FS_STEREO_MODE
    u16 val;
#endif

    DEBUGPRINT("%s: enter codec name=%s", __func__, fs16xx->fs16xx_dev);

    /* set codec Bulk write method, will be used for
     * loading DSP firmware and config files.
     */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 10, 108))
    codec->bulk_write_raw = fs16xx_bulk_write;
#endif
    fs16xx->codec = codec;

    /* reset registers to Default values */
    mutex_lock(&lr_lock);
    snd_soc_write(codec, FS16XX_SYSCTRL_REG, 0x0002);
    snd_soc_write(codec, FS16XX_SYSCTRL_REG, 0x0001);
    Sleep(10);

    printk(KERN_INFO "%s enter.\n", __func__);
    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        printk(KERN_INFO "%s in left\n", __func__);
        snd_soc_write(codec, 0xC4, 0x000A);
        /* initialize I2C registers */
        for (i = 0; i < ARRAY_SIZE(Fs16xx_reg_defaults); i++) {
            snd_soc_write(codec, Fs16xx_reg_defaults[i].reg,
                            Fs16xx_reg_defaults[i].value);
        }

        fs16xx_UpdateOTCTRL(codec);
    } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        printk(KERN_INFO "%s in right\n", __func__);
        snd_soc_write(codec, 0xC4, 0x000A);
        usleep_range(5000, 5001);
        for (i = 0; i < ARRAY_SIZE(Fs16xx_reg_defaults); i++) {
            snd_soc_write(codec, Fs16xx_reg_defaults[i].reg,
                            Fs16xx_reg_defaults[i].value);
        }

        fs16xx_UpdateOTCTRL(codec);
    }
/* STEREO_MODE left device just select left channel,
   right device just select right channel; */
#ifdef FS_STEREO_MODE
    printk("%s, stereo mode.\n",__func__);
    val = snd_soc_read(codec, FS16XX_I2SCTRL_REG);
    if(!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        val &= ~(0x3 << FS16XX_I2SCTRL_REG_CHS12_POS);
        val |= (0x1 << FS16XX_I2SCTRL_REG_CHS12_POS);
    }
    if(!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        val &= ~(0x3 << FS16XX_I2SCTRL_REG_CHS12_POS);
        val |= (0x2 << FS16XX_I2SCTRL_REG_CHS12_POS);
    }
    snd_soc_write(codec, FS16XX_I2SCTRL_REG, val);
#endif
    mutex_unlock(&lr_lock);

    /* add controls and widgets */
    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        snd_soc_add_codec_controls(codec, fs16xx_left_snd_controls,
                 ARRAY_SIZE(fs16xx_left_snd_controls));
        /*snd_soc_dapm_new_controls(&codec->dapm,
                fs16xx_left_dapm_widgets,
                ARRAY_SIZE(fs16xx_left_dapm_widgets));

        snd_soc_dapm_add_routes(&codec->dapm, fs16xx_left_dapm_routes,
                ARRAY_SIZE(fs16xx_left_dapm_routes));*/
    } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        snd_soc_add_codec_controls(codec, fs16xx_right_snd_controls,
                 ARRAY_SIZE(fs16xx_right_snd_controls));
        /*snd_soc_dapm_new_controls(&codec->dapm,
                fs16xx_right_dapm_widgets,
                ARRAY_SIZE(fs16xx_right_dapm_widgets));
        snd_soc_dapm_add_routes(&codec->dapm, fs16xx_right_dapm_routes,
                ARRAY_SIZE(fs16xx_right_dapm_routes));*/
    }
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 10, 108))
    snd_soc_dapm_new_widgets(&codec->dapm);
    snd_soc_dapm_sync(&codec->dapm);
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 72))
    snd_soc_dapm_new_widgets(codec->component.card);
    snd_soc_dapm_sync(&codec->dapm);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
    snd_soc_dapm_new_widgets(codec->component.card);
    snd_soc_dapm_sync(snd_soc_codec_get_dapm(codec));
#else
    snd_soc_dapm_new_widgets(codec->card);
    snd_soc_dapm_sync(snd_soc_component_get_dapm(codec));
#endif

    /* load preset tables */
    queue_work(fs16xx->fs16xx_wq, &fs16xx->load_preset);
    printk("fs16xx codec probe successfully. \n");
    return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
static int fs16xx_remove(struct snd_soc_codec *codec)
{
    pr_debug("%s: enter\n", __func__);
    mutex_lock(&lr_lock);
    fs16xx_power(codec, 0);
    mutex_unlock(&lr_lock);
    return 0;
}
#else
static void fs16xx_remove(struct snd_soc_codec *codec)
{
    pr_debug("%s: enter\n", __func__);
    mutex_lock(&lr_lock);
    fs16xx_power(codec, 0);
    mutex_unlock(&lr_lock);
}
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 14, 72))
/* table used by ALSA core while creating codec register
 * access debug fs.
 */
static u8 fs16xx_reg_readable[FS16XX_REG_CACHE_SIZE] = {
[FS16XX_STATUS_REG] = 1,
[FS16XX_BATS_REG] = 1,
[FS16XX_TEMPS_REG] = 1,
[FS16XX_ID_REG] = 1,
[FS16XX_I2SCTRL_REG] = 1,
[FS16XX_AUDIOCTRL_REG] = 1,
[FS16XX_TEMPSEL_REG] = 1,
[FS16XX_SYSCTRL_REG] = 1,
[FS16XX_SPKSTAUTS_REG] = 1,
[FS16XX_CHIPINI_REG] = 1,
[FS16XX_I2CADD_REG] = 1,
[FS16XX_BSTCTRL_REG] = 1
};

static int fs16xx_readable(struct snd_soc_codec *codec, unsigned int reg)
{
    return fs16xx_reg_readable[reg];
}
#endif

static struct snd_soc_codec_driver soc_codec_dev_fs16xx = {
    .read = fs16xx_read,
    .write = fs16xx_write,
    .probe = fs16xx_probe,
    .remove = fs16xx_remove,
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 14, 72))
    .readable_register = fs16xx_readable,
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
    .reg_cache_size = FS16XX_REG_CACHE_SIZE,
    .reg_word_size = 2,
#endif
};
#ifdef CONFIG_OF
static struct fs16xx_pdata *
fs16xx_of_init(struct i2c_client *client)
{
    struct fs16xx_pdata *pdata;
    struct device_node *np = client->dev.of_node;

    DEBUGPRINT("%s: enter i2c addr=0x%x", __func__, client->addr);
    pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        PRINT_ERROR("%s : pdata allocation failure.", __func__);
        return NULL;
    }
    if (of_property_read_string(np, "fs,fs_bin_path",
                &pdata->fw_path))
        pdata->fw_path = ".";

    if (of_property_read_string(np, "fs,fs-dev", &pdata->fs_dev))
        pdata->fs_dev = "left";

    if (of_property_read_string(np, "fs,fs-firmware-part-name",
                    &pdata->fw_name))
        pdata->fw_name = "fs16xx";

    return pdata;
}
#else
static inline struct fs16xx_pdata *
fs16xx_of_init(struct i2c_client *client)
{
    return NULL;
}
#endif

static int fs16xx_detect_device(Fs16xx_devId_t id)
{
    uint16_t dev_id;
    int ret;

    ret = fs16xx_read_register16(id, 0x03, &dev_id);
    if (ret) {
        return ret;
    }
    switch ((dev_id >> 8) & 0xff) {
        case 0x06: // FS1818
        case 0x0B: // FS1896
            PRINT("%s: FS1818/FS1896 detected", __func__);
            break;
        default:
            PRINT_ERROR("%s: invalid device_id: %X", __func__, dev_id);
            return -EINVAL;
        break;
    }

    return 0;
}

static int fs16xx_set_codec_info(struct device *dev, struct snd_soc_dai_driver *dai)
{
    char name[32];
    char tmp[32];
    int id1, id2;

    if (dev_name(dev) == NULL)
        return -EINVAL;

    strlcpy(name, dev_name(dev), 32);
    if (sscanf(name, "%x-%x", &id1, &id2) != 2)
        return -EINVAL;

    /* sanitize component name for DAI link creation */
    snprintf(tmp, 32, "%s.%s", dev->driver->name, name);
    strlcpy(name, tmp, 32);
    printk("Codec: %s", name);
    printk("Codec DAI: %s", dai->name);

    return 0;
}

static int fs16xx_i2c_probe(struct i2c_client *i2c,
                      const struct i2c_device_id *id)
{
    fs16xx_dev_t *fs16xx;
    struct fs16xx_pdata *pdata;
    int ret;
    int i;

    PRINT("%s: enter i2c addr=0x%x", __func__, i2c->addr);
    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed.");
        return -EIO;
    }

    if (i2c->dev.of_node)
        pdata = fs16xx_of_init(i2c);
    else
        pdata = i2c->dev.platform_data;

    /* check platform data */
    if (pdata == NULL) {
        dev_err(&i2c->dev, "platform data is NULL.");
        return -EINVAL;
    }

    fs16xx = devm_kzalloc(&i2c->dev, sizeof(fs16xx_dev_t), GFP_KERNEL);
    if (fs16xx == NULL) {
        dev_err(&i2c->dev, "alloc memory fail");
        return -ENOMEM;
    }

    fs16xx->control_data = i2c;
    fs16xx->dsp_init = 1;
    fs16xx->fs16xx_dev = pdata->fs_dev;
    fs16xx->sysclk = SYS_CLK_DEFAULT;
    fs16xx->update_eq = 1;

    // Initialize dev_state
    fs16xx->dev_state.cur_preset = -1;
    fs16xx->dev_state.calib_state = Fs16xx_Calib_Unknown;
    fs16xx->dev_state.dev_preset = NULL;
    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        fs16xx->id = FS16XX_DEV_INDEX_L;
        #ifdef USE_RAS_MODE
            fs16xx->dev_state.ras= true;
        #else
            fs16xx->dev_state.ras = false;
        #endif
    } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        fs16xx->id = FS16XX_DEV_INDEX_R;
    } else {
        PRINT_ERROR("%s: invalid dev:%s", __func__, fs16xx->fs16xx_dev);
        return -EINVAL;
    }
    g_fs16xx_priv[fs16xx->id] = fs16xx;

    i2c_set_clientdata(i2c, fs16xx);

    mutex_init(&fs16xx->i2c_rw_lock);

    // check device id
    ret = fs16xx_detect_device(fs16xx->id);
    if (ret) {
        PRINT_ERROR("%s: detect device[%X] failed", __func__, i2c->addr);
        g_fs16xx_priv[fs16xx->id] = NULL;
        return ret;
    }
    /* register sysfs hooks */
    ret = sysfs_create_group(&i2c->dev.kobj, &fs16xx_attr_group);
    if (!ret) {
        for (i = 0; i < ARRAY_SIZE(fs16xx_raw_bin_attr); i++) {
            ret = sysfs_create_bin_file(&i2c->dev.kobj,
                            &fs16xx_raw_bin_attr[i]);
            if (ret)
                PRINT_ERROR("%s: Error creating fs16xx sysfs bin attr", __func__);
        }
    }
    else {
        PRINT_ERROR("%s: Error registering fs16xx sysfs", __func__);
    }

    /* setup work queue, will be used to initial DSP on first boot up */
    fs16xx->fs16xx_wq = create_singlethread_workqueue("fs16xx");
    if (fs16xx->fs16xx_wq == NULL) {
        ret = -ENOMEM;
        goto wq_fail;
    }

    INIT_WORK(&fs16xx->calib_work, fs16xx_work_read_imp);
    INIT_DELAYED_WORK(&fs16xx->mode_work, fs16xx_work_mode);
    INIT_WORK(&fs16xx->load_preset, fs16xx_load_preset);
#if 0
    fs16xx->vdd = regulator_get(&i2c->dev, "fs_vdd");
    if (IS_ERR(fs16xx->vdd)) {
        PRINT_ERROR("%s: Error getting vdd regulator.", __func__);
        ret = PTR_ERR(fs16xx->vdd);
        goto codec_fail;
    }

    regulator_set_voltage(fs16xx->vdd, 1800000, 1800000);
    ret = regulator_enable(fs16xx->vdd);
    if (ret < 0) {
        PRINT_ERROR("%s: Error enabling vdd regulator %d:",
            __func__, ret);
        goto reg_enable_fail;
    }
#endif
    if (!strncmp("left", fs16xx->fs16xx_dev, 4)) {
        /* register codec */
        ret = snd_soc_register_codec(&i2c->dev,
                &soc_codec_dev_fs16xx,
                &fs16xx_left_dai, 1);
        if (ret < 0) {
            PRINT_ERROR("%s: Error registering fs16xx left codec",
                __func__);
            goto codec_fail;
        }
        fs16xx_set_codec_info(&i2c->dev,&fs16xx_left_dai);
    } else if (!strncmp("right", fs16xx->fs16xx_dev, 5)) {
        /* register codec */
        ret = snd_soc_register_codec(&i2c->dev,
                &soc_codec_dev_fs16xx,
                &fs16xx_right_dai, 1);
        if (ret < 0) {
            PRINT_ERROR("%s: Error registering fs16xx right codec",
                __func__);
            goto codec_fail;
        }
        fs16xx_set_codec_info(&i2c->dev,&fs16xx_right_dai);
        stereo_mode = 1;
    }

    PRINT("fs16xx %s probed successfully!", fs16xx->fs16xx_dev);

    return ret;

codec_fail:
#if 0
    regulator_disable(fs16xx->vdd);
reg_enable_fail:
    regulator_put(fs16xx->vdd);
#endif
wq_fail:
    return ret;
}

static int fs16xx_i2c_remove(struct i2c_client *client)
{
    fs16xx_dev_t *fs16xx = i2c_get_clientdata(client);

    DEBUGPRINT("%s: enter", __func__);
    snd_soc_unregister_codec(&client->dev);
#if 0
    regulator_disable(fs16xx->vdd);
    regulator_put(fs16xx->vdd);
#endif
    destroy_workqueue(fs16xx->fs16xx_wq);
    return 0;
}

static const struct i2c_device_id fs16xx_i2c_id[] = {
    { "fs16xx", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, fs16xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id fs16xx_match_tbl[] = {
    { .compatible = "foursemi,fs16xx" },
    { },
};
MODULE_DEVICE_TABLE(of, fs16xx_match_tbl);
#endif

static struct i2c_driver fs16xx_i2c_driver = {
    .driver = {
        .name = "fs16xx",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(fs16xx_match_tbl),
    },
    .probe =    fs16xx_i2c_probe,
    .remove =   fs16xx_i2c_remove,
    .id_table = fs16xx_i2c_id,
};



static int __init fs16xx_modinit(void)
{
    int ret;
    DEBUGPRINT("%s: enter.", __func__);
    PRINT("fs16xx branch: %s\n", FS16XX_GIT_BRANCH);
    PRINT("fs16xx commit: %s\n", FS16XX_GIT_COMMIT);
    PRINT("fs16xx date: %s\n", FS16XX_CODE_DATE);
    PRINT("fs16xx version: %s\n", FS16XX_CODE_VERSION);
    ret = i2c_add_driver(&fs16xx_i2c_driver);
    if (ret != 0) {
        PRINT_ERROR("Failed to register fs16xx I2C driver:%d", ret);
    }
    return ret;
}
module_init(fs16xx_modinit);

static void __exit fs16xx_exit(void)
{
    i2c_del_driver(&fs16xx_i2c_driver);
}
module_exit(fs16xx_exit);

MODULE_DESCRIPTION("ASoC fs16xx codec driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FourSemi SW");
