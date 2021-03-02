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
#define pr_fmt(fmt) "bos0614: %s: " fmt, __func__

#define DEBUG
//#undef DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/i2c.h>

#include "libs/dk-core/src/bsp/drivers/haptic/bosDriver.h"
#include "i2cLinux.h"
#include "libs/dk-core/src/bsp/drivers/i2c/i2c.h"
#include "libs/dk-core/src/bsp/drivers/haptic/bos0614Driver.h"

#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_err
#undef dev_dbg
#define dev_dbg dev_info
#endif

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

static ssize_t getRegsDump(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    Context *ctx = dev_get_drvdata(dev);
    ssize_t blen = 0;
    (void) attr;

    if (ctx != NULL)
    {
        mutex_lock(&ctx->lock);

        if (ctx->hapticDriver != NULL)
        {
	    bool res;
	    u16 r, rval;
	    /* dump first 42 registers */
	    for (r = 0; r < ADDRESS_BOS0614_BIST_REG; r++) {
		rval = 0x2bad;
		res = ctx->hapticDriver->getRegister(ctx->hapticDriver, r, &rval);
		blen += snprintf(buf + blen, PAGE_SIZE - blen,
			"%02X: 0x%02X 0x%02X\n", r,
			(rval & 0xFF00) >> 8,
			(rval & 0x00FF));
	    }
        }

        mutex_unlock(&ctx->lock);
    }

    return blen;
}

static DEVICE_ATTR(regs, 0440, getRegsDump, NULL);

static bool hasError(BOSError *errors, size_t length, BOSError errorType)
{
    size_t index;
    bool res = false;

    if (errors != NULL)
    {
        for (index = 0; index < length; index++)
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
        HapticDriver *driver = ctx->hapticDriver;
        size_t nbrErros;
        BOSError *errors;

        errors = kzalloc(sizeof(BOSError) * BOSERROR_Length, GFP_KERNEL);
	if (!errors)
            return res;

        mutex_lock(&ctx->lock);

        nbrErros = driver->getError(driver, errors, BOSERROR_Length);

        res = snprintf(buf, PAGE_SIZE, "SC: %d UVLO: %d IDAC: %d MAX_POWER: %d OVT: %d OVV: %d\n",
                           hasError(errors, nbrErros, BOSERROR_SHORT_CIRCUIT),
                           hasError(errors, nbrErros, BOSERROR_UVLO),
                           hasError(errors, nbrErros, BOSERROR_IDAC),
                           hasError(errors, nbrErros, BOSERROR_MAX_POWER),
                           hasError(errors, nbrErros, BOSERROR_OVT),
                           hasError(errors, nbrErros, BOSERROR_OVV));

        kfree(errors);

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
        HapticDriver *driver = ctx->hapticDriver;
        WaveformId id = 0;
        uint8_t startSliceId = 0;
        size_t nbrOfSlices = 0;
        uint16_t cycle = 0;
        uint8_t outputChannel = 0;
        size_t paramLength = 0;
	int numP = 0;
	ParamsLst params[SET_WAVEFORM_PARAM_LENGTH];

	PARAM_ADD(PARAM_UCHAR8, &id);
	PARAM_ADD(PARAM_UCHAR8, &startSliceId);
	PARAM_ADD(PARAM_UINT32, &nbrOfSlices);
	PARAM_ADD(PARAM_UINT16, &cycle);
	PARAM_ADD(PARAM_UCHAR8, &outputChannel);

	paramLength = process_params(params, numP, buf);
	if (paramLength != SET_WAVEFORM_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        dev_dbg(ctx->dev,
                "[Set Synth Waveform] Waveform Id: %d Start Slice Id: %d Nbr Of Slices: %zu Cycle: %d Output Channel: %d\n",
                id, startSliceId, nbrOfSlices, cycle, outputChannel);

        mutex_lock(&ctx->lock);

        if (driver->synthSetWaveform(driver, id, startSliceId, nbrOfSlices, cycle, outputChannel))
        {
            res = count;
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
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        SynthSlice slice;
        uint8_t outputChannel = 0;
        size_t paramLength = 0;
	int numP = 0;
	ParamsLst params[SET_SLICE_PARAM_LENGTH];

	PARAM_ADD(PARAM_UINT32, &slice.sliceId);
	PARAM_ADD(PARAM_INT32, &slice.mVAmp);
	PARAM_ADD(PARAM_UINT32, &slice.mHzFreq);
	PARAM_ADD(PARAM_UINT32, &slice.cycle);
	PARAM_ADD(PARAM_UCHAR8, &outputChannel);

	paramLength = process_params(params, numP, buf);
	if (paramLength != SET_SLICE_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        dev_dbg(ctx->dev,
                "[Set Slice] Slice Id: %d Amplitude: %d mV Frequency: %d milliHertz Cycle: %d Output Channel: %d\n",
                slice.sliceId, slice.mVAmp, slice.mHzFreq, slice.cycle, outputChannel);

        mutex_lock(&ctx->lock);

        if (driver->synthSetSlice(driver, (const SynthSlice *) &slice, outputChannel))
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
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        WaveformId start = 0;
        WaveformId stop = 0;
        size_t paramLength = 0;
	int numP = 0;
	ParamsLst params[SYNTH_PLAY_PARAM_LENGTH];

	PARAM_ADD(PARAM_UCHAR8, &start);
	PARAM_ADD(PARAM_UCHAR8, &stop);

	paramLength = process_params(params, numP, buf);
	if (paramLength != SYNTH_PLAY_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        mutex_lock(&ctx->lock);

        if (driver->wfsPlay(driver, start, stop))
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
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        int outputState = 0;
        size_t paramLength = 0;
        bool bOutputState;
	int numP = 0;
	ParamsLst params[CTRL_OUTPUT_PARAM_LENGTH];

	PARAM_ADD(PARAM_INT32, &outputState);

	paramLength = process_params(params, numP, buf);
	if (paramLength != CTRL_OUTPUT_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        bOutputState = outputState > 0 ? true : false;

        mutex_lock(&ctx->lock);

        if (driver->ctrlOutput(driver, bOutputState))
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
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        SensingConfig config;
        ChannelId channelId = 0;
        size_t paramLength = 0;
        const char *directionS;
        const char *modeS;
	int numP = 0;
	ParamsLst params[SENSING_CONFIG_PARAM_LENGTH];

	PARAM_ADD(PARAM_UCHAR8, &channelId);
	PARAM_ADD(PARAM_UCHAR8, &config.mode);
	PARAM_ADD(PARAM_UCHAR8, &config.direction);
	PARAM_ADD(PARAM_UINT16, &config.debounceUs);
	PARAM_ADD(PARAM_INT16, &config.thresholdMv);
	PARAM_ADD(PARAM_UCHAR8, &config.stabilisationMs);

	paramLength = process_params(params, numP, buf);
	if (paramLength != SENSING_CONFIG_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        directionS = getSensingDirection(config.direction);
        modeS = getSensingDetectionMode(config.mode);

        mutex_lock(&ctx->lock);

        if (directionS != NULL && modeS != NULL &&
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
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        ChannelId channelId = 0;
        WaveformId id = 0;
        SensingDirection direction = 0;
        size_t paramLength = 0;
        const char *directionS;
	int numP = 0;
	ParamsLst params[SENSING_AUTO_FEEDBACK_PARAM_LENGTH];

	PARAM_ADD(PARAM_UCHAR8, &channelId);
	PARAM_ADD(PARAM_UCHAR8, &id);
	PARAM_ADD(PARAM_UCHAR8, &direction);

	paramLength = process_params(params, numP, buf);
	if (paramLength != SENSING_AUTO_FEEDBACK_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        directionS = getSensingDirection(direction);

        mutex_lock(&ctx->lock);

        if (directionS != NULL &&
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
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;
    (void) attr;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        ChannelId channelId = 0;
        SensingDirection direction = 0;
        const char *directionS;
        size_t paramLength = 0;
	int numP = 0;
	ParamsLst params[STOP_SENSING_PARAM_LENGTH];

	PARAM_ADD(PARAM_UCHAR8, &channelId);
	PARAM_ADD(PARAM_UCHAR8, &direction);

	paramLength = process_params(params, numP, buf);
	if (paramLength != STOP_SENSING_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        directionS = getSensingDirection(direction);

        mutex_lock(&ctx->lock);

        if (directionS != NULL &&
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
        &dev_attr_regs.attr,
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
    int err;
    Context *ctx;
    Bos0614Resource resource = {
	    .gpioA = NULL,
	    .gpioB = NULL,
	    .gpioC = NULL,
	    .gpioD = NULL
    };

    dev_info(&client->dev, "%s called\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        dev_err(&client->dev, "%s:I2C check failed\n", __func__);
        res = -ENODEV;
        goto bosDriverProbeError;
    }

    ctx = devm_kzalloc(&client->dev, sizeof(*ctx), GFP_KERNEL);

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
    resource.i2c = ctx->i2c;
    ctx->hapticDriver = bos0614DriverI2cInit(resource);

    pr_debug("i2c: %p, driver: %p\n", ctx->i2c, ctx->hapticDriver);

    if (ctx->i2c == NULL || ctx->hapticDriver == NULL)
    {
        dev_err(&client->dev, "Failed to instantiate BOS0614 driver\n");
        goto bosDriverProbeError;
    }

    mutex_init(&ctx->lock);

    err = sysfs_create_group(&ctx->dev->kobj, &bosDriverAttrGroup);

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
    Context *ctx = i2c_get_clientdata(client);

    dev_info(&client->dev, "%s called \n", __func__);

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

#ifdef CONFIG_OF
static const struct of_device_id matchTable[] = {
        { .compatible = "boreas,bos0614", },
	{},
};
#endif

static struct i2c_driver bosDriverI2c = {
        .class = I2C_CLASS_HWMON,
        .driver = {
                .owner = THIS_MODULE,
#ifdef CONFIG_OF
                .name = "bos0614_mmi",
		.of_match_table = matchTable,
#else
                .name = "bos0614",
#endif
        },
        .probe = bosDriverI2cProbe,
        .remove = bosDriverI2cRemove,
        .detect = detect,
        .id_table = bosDriverI2cId,
        .address_list = addrList
};

#ifdef CONFIG_OF
static int __init bosDriver_init(void)
{
	pr_debug("loading driver\n");
	return i2c_add_driver(&bosDriverI2c);
}

static void __exit bosDriver_exit(void)
{
	pr_debug("removing driver\n");
	i2c_del_driver(&bosDriverI2c);
}

module_init(bosDriver_init);
module_exit(bosDriver_exit);
#else
module_i2c_driver(bosDriverI2c)
#endif

MODULE_AUTHOR("Pascal-Frédéric St-Laurent <pfstlaurent@boreas.ca>");
MODULE_DESCRIPTION("I2C Driver for Boréas Haptic Technologies");
MODULE_LICENSE("GPL v2");
