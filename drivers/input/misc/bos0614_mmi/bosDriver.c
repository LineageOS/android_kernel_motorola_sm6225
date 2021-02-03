//
// Description: Linux Boréas Haptic Driver for I2C base IC
//
// Copyright (c) 2020 Boreas Technologies All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include "libs/dk-core/src/bsp/drivers/haptic/bosDriver.h"
#include "i2cLinux.h"
#include <libs/dk-core/src/bsp/drivers/i2c/i2c.h>
#include <libs/dk-core/src/bsp/drivers/haptic/bos0614Driver.h>

typedef struct
{
    I2c *i2c;
    const struct i2c_device_id *id;
    struct mutex lock;
    struct device *dev;
    HapticDriver *hapticDriver;
} Context;

#define SET_WAVEFORM_PARAM_LENGTH (5)
#define SET_SLICE_PARAM_LENGTH (5)
#define SYNTH_PLAY_PARAM_LENGTH (2)
#define CTRL_OUTPUT_PARAM_LENGTH (1)
#define SENSING_CONFIG_PARAM_LENGTH (6)
#define SENSING_AUTO_FEEDBACK_PARAM_LENGTH (3)
#define STOP_SENSING_PARAM_LENGTH (2)

static ssize_t getChipId(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    Context *ctx = dev_get_drvdata(dev);
    u32 chipId = 0;
    (void) attr;

    if (ctx != NULL)
    {
        mutex_lock(&ctx->lock);

        if (ctx->hapticDriver != NULL)
        {
            chipId = ctx->hapticDriver->getChipId(ctx->hapticDriver);
        }

        mutex_unlock(&ctx->lock);
    }

    return snprintf(buf, PAGE_SIZE, "0x%x\n", chipId);
}

static DEVICE_ATTR(chip_id, 0440, getChipId, NULL);

static bool hasError(BOSError *errors, size_t length, BOSError errorType)
{
    bool res = false;

    if (errors != NULL)
    {
        for (size_t index = 0; index < length; index++)
        {
            if (errors[index] == errorType)
            {
                res = true;
                break;
            }
        }
    }

    return res;
}


static ssize_t getIcErrors(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        BOSError *errors = kzalloc(sizeof(BOSError) * BOSERROR_Length, GFP_KERNEL);

        if (errors != NULL)
        {
            HapticDriver *driver = ctx->hapticDriver;
            size_t nbrErros = driver->getError(driver, errors, BOSERROR_Length);

            res = snprintf(buf, PAGE_SIZE, "SC: %d UVLO: %d IDAC: %d MAX_POWER: %d OVT: %d OVV: %d\n",
                           hasError(errors, nbrErros, BOSERROR_SHORT_CIRCUIT),
                           hasError(errors, nbrErros, BOSERROR_UVLO),
                           hasError(errors, nbrErros, BOSERROR_IDAC),
                           hasError(errors, nbrErros, BOSERROR_MAX_POWER),
                           hasError(errors, nbrErros, BOSERROR_OVT),
                           hasError(errors, nbrErros, BOSERROR_OVV));

            kfree(errors);
        }


        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(ic_errors, 0440, getIcErrors, NULL);

static ssize_t setSynthWaveform(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        WaveformId id;
        uint8_t startSliceId;
        size_t nbrOfSlices;
        uint16_t cycle;
        uint8_t outputChannel;

        size_t paramLength = sscanf(buf, "%hi %hi %zi %hi %hi", &id, &startSliceId, &nbrOfSlices, &cycle,
                                    &outputChannel);
        dev_dbg(ctx->dev,
                "[Set Synth Waveform] Waveform Id: %d Start Slice Id: %d Nbr Of Slices: %d Cycle: %d Output Channel: %d\n",
                id, startSliceId, nbrOfSlices, cycle, outputChannel);

        if (paramLength == SET_WAVEFORM_PARAM_LENGTH)
        {
            HapticDriver *driver = ctx->hapticDriver;

            if (driver->synthSetWaveform(driver, id, startSliceId, nbrOfSlices, cycle, outputChannel))
            {
                res = count;
            }
        }

        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(set_waveform, 0220, NULL, setSynthWaveform);

static ssize_t setSynthSlice(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
    (void) attr;
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        HapticDriver *driver = ctx->hapticDriver;
        SynthSlice slice;
        uint8_t outputChannel;

        size_t paramLength = sscanf(buf, "%d %d %d %d %d", &slice.sliceId, &slice.mVAmp, &slice.mHzFreq,
                                    &slice.cycle, &outputChannel);

        dev_dbg(ctx->dev,
                "[Set Slice] Slice Id: %d Amplitude: %d mV Frequency: %d milliHertz Cycle: %d Output Channel: %d\n",
                slice.sliceId, slice.mVAmp, slice.mHzFreq, slice.cycle, outputChannel);

        if (paramLength == SET_SLICE_PARAM_LENGTH &&
            driver->synthSetSlice(driver, (const SynthSlice *) &slice, outputChannel))
        {
            res = count;
        }

        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(set_slice, 0220, NULL, setSynthSlice);


static ssize_t synthPlay(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t count)
{
    (void) attr;
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        HapticDriver *driver = ctx->hapticDriver;

        WaveformId start;
        WaveformId stop;

        size_t paramLength = sscanf(buf, "%hi %hi", &start, &stop);

        if (paramLength == SYNTH_PLAY_PARAM_LENGTH && driver->wfsPlay(driver, start, stop))
        {
            res = count;
        }

        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(synth_play, 0220, NULL, synthPlay);

static ssize_t setOutput(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t count)
{
    (void) attr;
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        HapticDriver *driver = ctx->hapticDriver;

        int outputState = 0;
        size_t paramLength = sscanf(buf, "%d", &outputState);

        bool bOutputState = outputState > 0 ? true : false;

        if (paramLength == CTRL_OUTPUT_PARAM_LENGTH &&
            driver->ctrlOutput(driver, bOutputState))
        {
            dev_dbg(ctx->dev,
                    "[Set Output] State: %d \n", bOutputState);
            res = count;
        }

        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(ctrl_output, 0220, NULL, setOutput);

const char *slopeDetection = "Slope";
const char *thresholdDetection = "Threshold";

static const char *getSensingDetectionMode(SensingDetectionMode mode)
{
    const char *stringDetectionMode = NULL;
    switch (mode)
    {
        case SensingDetectionMode_Slope:
            stringDetectionMode = slopeDetection;
            break;
        case SensingDetectionMode_Threshold:
            stringDetectionMode = thresholdDetection;
            break;
    }

    return stringDetectionMode;
}

const char *sensingPressDirection = "Press";
const char *sensingReleaseDirection = "Release";
const char *sensingPressReleaseDirection = "Press & Release";

static const char *getSensingDirection(SensingDirection direction)
{
    const char *directionString = NULL;

    switch (direction)
    {
        case SensingDirection_Press:
            directionString = sensingPressDirection;
            break;
        case SensingDirection_Release:
            directionString = sensingReleaseDirection;
            break;
        case SensingDirection_Both:
            directionString = sensingPressReleaseDirection;
            break;
    }

    return directionString;
}

static ssize_t setSensingConfig(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    (void) attr;
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        HapticDriver *driver = ctx->hapticDriver;

        SensingConfig config;
        ChannelId channelId;

        size_t paramLength = sscanf(buf, "%hhu %d %d %hd %hd %s",
                                    &channelId, &config.mode, &config.direction,
                                    &config.debounceUs,
                                    &config.thresholdMv,
                                    &config.stabilisationMs);

        const char *directionS = getSensingDirection(config.direction);
        const char *modeS = getSensingDetectionMode(config.mode);

        if (paramLength == SENSING_CONFIG_PARAM_LENGTH &&
            directionS != NULL &&
            modeS != NULL &&
            driver->configSensing(driver, channelId, config) &&
            ctx->hapticDriver->configGPO(ctx->hapticDriver, channelId, GPO_ButtonState))
        {
            dev_dbg(ctx->dev,
                    "[Sensing config] Success Channel #: %d Mode: %s Direction: %s Debouncing Duration: %d microS Threshold: %d Stabilization Duration: %d ms\n",
                    channelId, modeS, directionS, config.debounceUs,
                    config.thresholdMv, config.stabilisationMs);
            res = count;
        }

        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(sensing_config, 0220, NULL, setSensingConfig);

static ssize_t setSensingAutoPlay(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    (void) attr;
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        HapticDriver *driver = ctx->hapticDriver;

        ChannelId channelId;
        WaveformId id;
        SensingDirection direction;

        size_t paramLength = sscanf(buf, "%d %d %d", &channelId, &id, &direction);
        const char *directionS = getSensingDirection(direction);


        if (paramLength == SENSING_AUTO_FEEDBACK_PARAM_LENGTH &&
            directionS != NULL &&
            driver->sensingAutoPlayWave(driver, channelId, id, direction))
        {
            dev_dbg(ctx->dev, "[Set Sensing Automatic Feedback] Channel #: %d Waveform #: %d Direction: %s", channelId,
                    id, directionS);
            res = count;
        }

        mutex_unlock(&ctx->lock);
    }


    return res;
}

static DEVICE_ATTR(sensing_autoplay, 0220, NULL, setSensingAutoPlay);

static ssize_t stopSensing(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
    (void) attr;
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        mutex_lock(&ctx->lock);

        HapticDriver *driver = ctx->hapticDriver;

        ChannelId channelId;
        SensingDirection direction;

        size_t paramLength = sscanf(buf, "%d %d", &channelId, &direction);
        const char *directionS = getSensingDirection(direction);

        if (paramLength == STOP_SENSING_PARAM_LENGTH &&
            directionS != NULL &&
            driver->stopSensing(driver, channelId, direction))
        {
            dev_dbg(ctx->dev, "[Remove Sensing Profile] Channel #: %d Direction: %s", channelId, directionS);
            res = count;
        }


        mutex_unlock(&ctx->lock);
    }


    return res;
}

static DEVICE_ATTR(sensing_stop, 0220, NULL, stopSensing);

static struct attribute *bosDriverAttrs[] = {
        &dev_attr_chip_id.attr,
        &dev_attr_set_waveform.attr,
        &dev_attr_set_slice.attr,
        &dev_attr_synth_play.attr,
        &dev_attr_ic_errors.attr,
        &dev_attr_ctrl_output.attr,
        &dev_attr_sensing_config.attr,
        &dev_attr_sensing_autoplay.attr,
        &dev_attr_sensing_stop.attr,
        NULL,
};

static struct attribute_group bosDriverAttrGroup = {
        .attrs = bosDriverAttrs
};

static void freeResources(Context *ctx)
{
    if (ctx != NULL)
    {
        sysfs_remove_group(&ctx->dev->kobj, &bosDriverAttrGroup);

        if (ctx->i2c != NULL)
        {
            i2cBoreasLinuxFree(ctx->i2c);
        }

        mutex_destroy(&ctx->lock);
    }

}

/*
 * Public Section
 */

static int bosDriverI2cProbe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    int res = -ENODEV;

    dev_info(&client->dev, "%s called\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        dev_err(&client->dev, "%s:I2C check failed\n", __func__);
        res = -ENODEV;
        goto bosDriverProbeError;
    }

    Context *ctx = devm_kzalloc(&client->dev, sizeof(*ctx), GFP_KERNEL);

    if (ctx == NULL)
    {
        dev_err(&client->dev, "%s:no memory\n", __func__);
        res = -ENOMEM;
        goto bosDriverProbeError;
    }

    ctx->dev = &client->dev;
    i2c_set_clientdata(client, ctx);
    dev_set_drvdata(&client->dev, ctx);

    ctx->i2c = i2cBoreasLinuxInit(client);

    Bos0614Resource resource = {.i2c = ctx->i2c, .gpioA = NULL, .gpioB = NULL, .gpioC = NULL, .gpioD = NULL};
    ctx->hapticDriver = bos0614DriverI2cInit(resource);

    if (ctx->i2c == NULL || ctx->hapticDriver == NULL)
    {
        dev_err(&client->dev, "Failed to instantiate BOS0614 driver\n");
        goto bosDriverProbeError;
    }

    mutex_init(&ctx->lock);

    int err = sysfs_create_group(&ctx->dev->kobj, &bosDriverAttrGroup);

    if (err)
    {
        dev_err(&client->dev, "Failed to instantiate FS attributes\n");
        goto bosDriverProbeError;
    }

    if (ctx->i2c == NULL || ctx->hapticDriver == NULL)
    {
        dev_err(&client->dev, "Boreas Haptic Driver failed to instantiate I2C or Haptic Driver\n");
        goto bosDriverProbeError;
    }

    res = 0;

    dev_info(&client->dev, "BosDriver Successfully Configured \n");

    bosDriverProbeError:

    if (res != 0)
    {
        freeResources(ctx);
        dev_err(&client->dev, "Boreas Haptic Driver failed to probe\n");
    }

    return res;
}

static int bosDriverI2cRemove(struct i2c_client *client)
{
    int res = -ENODEV;

    dev_info(&client->dev, "%s called \n", __func__);

    Context *ctx = i2c_get_clientdata(client);

    freeResources(ctx);

    return res;
}

static const struct i2c_device_id bosDriverI2cId[] = {
        {"bos0614", 0}
};

static const unsigned short addrList[] = {
        0x2c,
};

static int detect(struct i2c_client *client, struct i2c_board_info *info)
{
    dev_info(&client->dev, "%s Remove bosDriver \n", __func__);
    return 0;
}

static struct i2c_driver bosDriverI2c = {
        .class = I2C_CLASS_HWMON,
        .driver = {
                .name = "bos0614",
                .owner = THIS_MODULE,
        },
        .probe = bosDriverI2cProbe,
        .remove = bosDriverI2cRemove,
        .detect = detect,
        .id_table = bosDriverI2cId,
        .address_list = addrList
};

module_i2c_driver(bosDriverI2c)

MODULE_AUTHOR("Pascal-Frédéric St-Laurent <pfstlaurent@boreas.ca>");
MODULE_DESCRIPTION("I2C Driver for Boréas Haptic Technologies");
MODULE_LICENSE("GPL v2");
