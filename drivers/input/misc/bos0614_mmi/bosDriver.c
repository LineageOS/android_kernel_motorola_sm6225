//
// Description: Linux Boréas Haptic Driver for I2C base IC
//
// Copyright (c) 2021 Boreas Technologies All rights reserved.
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
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>

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
#define MAXIMUM_SLICE_CONF (4)

typedef struct
{
    SynthSlice slice[MAXIMUM_SLICE_CONF];
    SensingConfig detection;
    size_t nbrSlice;
} HwButtonFeedback;

typedef struct
{
    bool present;
    ChannelId channelId;
    SensingConfig detection[SENSING_DIRECTION_ENUM_LENGTH];
} Channel;

typedef struct
{
    HwButtonFeedback feedback[SENSING_DIRECTION_ENUM_LENGTH];
    size_t nextSliceId;
    Channel channel[BOS0614_NBR_OF_CHANNEL];
} DefaultButtonConfig;

typedef struct
{
    I2c *i2c;
    const struct i2c_device_id *id;
    struct mutex lock;
    struct device *dev;
    HapticDriver *hapticDriver;
    DefaultButtonConfig defaultButtonConfig;
} Context;

#define SET_WAVEFORM_PARAM_LENGTH (5)
#define SET_SLICE_PARAM_LENGTH (5)
#define SET_SHAPED_SLICE_PARAM_LENGTH (6)
#define SYNTH_PLAY_PARAM_LENGTH (2)
#define CTRL_OUTPUT_PARAM_LENGTH (1)
#define SENSING_CONFIG_PARAM_LENGTH (6)
#define SENSING_AUTO_FEEDBACK_PARAM_LENGTH (3)
#define STOP_SENSING_PARAM_LENGTH (1)

#define ATTRIBUTE_CHANNEL_PREFIX_LENGTH (50)

#define FEEDBACK_SLICE_LENGTH (5)

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

static bool hasError(BosError *errors, size_t length, BosError errorType)
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
        BosError *errors;

        errors = kzalloc(sizeof(BosError) * BOS_ERROR_LENGTH, GFP_KERNEL);
	if (!errors)
            return res;

        mutex_lock(&ctx->lock);

        nbrErros = driver->getError(driver, errors, BOS_ERROR_LENGTH);

        res = snprintf(buf, PAGE_SIZE, "SC: %d UVLO: %d IDAC: %d MAX_POWER: %d OVT: %d OVV: %d\n",
                           hasError(errors, nbrErros, BOS_ERROR_SHORT_CIRCUIT),
                           hasError(errors, nbrErros, BOS_ERROR_UVLO),
                           hasError(errors, nbrErros, BOS_ERROR_IDAC),
                           hasError(errors, nbrErros, BOS_ERROR_MAX_POWER),
                           hasError(errors, nbrErros, BOS_ERROR_OVT),
                           hasError(errors, nbrErros, BOS_ERROR_OVV));

        kfree(errors);

        mutex_unlock(&ctx->lock);
    }

    return res;
}

static DEVICE_ATTR(ic_errors, 0440, getIcErrors, NULL);

static ssize_t getIcSensingState(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    Context *ctx = dev_get_drvdata(dev);
    ssize_t res = -EIO;

    if (ctx != NULL && ctx->hapticDriver != NULL)
    {
        HapticDriver *driver = ctx->hapticDriver;
        uint16_t regVal = 0;

        mutex_lock(&ctx->lock);
        driver->getRegister(driver, ADDRESS_BOS0614_SENSECONFIG_REG, &regVal);
        mutex_unlock(&ctx->lock);

        res = snprintf(buf, PAGE_SIZE, "%d\n", (regVal & 0x000F) ? 1 : 0);
    }

    return res;
}

static DEVICE_ATTR(is_sensing, 0444, getIcSensingState, NULL);

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
        uint8_t shape = 0;
        int numP = 0;
        ParamsLst params[SET_SHAPED_SLICE_PARAM_LENGTH];

        PARAM_ADD(PARAM_UINT32, &slice.sliceId);
        PARAM_ADD(PARAM_INT32, &slice.mVAmp);
        PARAM_ADD(PARAM_UINT32, &slice.mHzFreq);
        PARAM_ADD(PARAM_UINT32, &slice.cycle);
        PARAM_ADD(PARAM_UCHAR8, &outputChannel);
        PARAM_ADD(PARAM_UCHAR8, &shape);

        paramLength = process_params(params, numP, buf);
        if (paramLength != SET_SLICE_PARAM_LENGTH &&
            paramLength != SET_SHAPED_SLICE_PARAM_LENGTH) {
            return (ssize_t)paramLength;
        }

        dev_dbg(ctx->dev,
                "[Set Slice] Slice Id: %d Amplitude: %d mV Frequency: %d milliHertz Cycle: %d Shape: %d Output Channel: %d\n",
                slice.sliceId, slice.mVAmp, slice.mHzFreq, slice.cycle, shape, outputChannel);

        mutex_lock(&ctx->lock);

        if (driver->synthSetSlice(driver, (const SynthSlice *) &slice, shape, outputChannel))
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
        case SENSING_DETECTION_MODE_SLOPE:
            stringDetectionMode = slopeDetection;
            break;
        case SENSING_DETECTION_MODE_THRESHOLD:
            stringDetectionMode = thresholdDetection;
            break;
    }

    return stringDetectionMode;
}

const char *sensingPressDirection = "Press";
const char *sensingReleaseDirection = "Release";

static const char *getSensingDirection(SensingDirection direction)
{
    const char *directionString = NULL;

    switch (direction)
    {
        case SENSING_DIRECTION_PRESS:
            directionString = sensingPressDirection;
            break;
        case SENSING_DIRECTION_RELEASE:
            directionString = sensingReleaseDirection;
            break;
        case SENSING_DIRECTION_ENUM_LENGTH:
            directionString = NULL;
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
            driver->buttonSensing(driver, channelId, config) &&
            ctx->hapticDriver->configGPO(ctx->hapticDriver, channelId, GPO_BUTTON_STATE))
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
        size_t paramLength = 0;
	int numP = 0;
	ParamsLst params[STOP_SENSING_PARAM_LENGTH];

	PARAM_ADD(PARAM_UCHAR8, &channelId);

	paramLength = process_params(params, numP, buf);
	if (paramLength != STOP_SENSING_PARAM_LENGTH) {
		return (ssize_t)paramLength;
	}

        mutex_lock(&ctx->lock);

        if (driver->stopSensing(driver, channelId))
        {
            dev_dbg(ctx->dev, "[Remove Sensing Profile] Channel #: %d", channelId);
            res = count;
        }


        mutex_unlock(&ctx->lock);
    }


    return res;
}

static DEVICE_ATTR(sensing_stop, 0220, NULL, stopSensing);
#define FREQ_INDEX (0)
#define MV_AMP_INDEX (1)
#define CYCLE_INDEX (2)

static bool
decodeSlicesDefaultConfig(Context* ctx, const char* attr, HwButtonFeedback* feedback)
{
    bool res = false;

    u32 dataFeedback[4 * FEEDBACK_SLICE_LENGTH];

    if (attr)
    {
        int count = of_property_read_variable_u32_array(ctx->dev->of_node,
			attr, dataFeedback, FEEDBACK_SLICE_LENGTH, sizeof(dataFeedback));

        if (count > 0 && (count % FEEDBACK_SLICE_LENGTH) == 0)
        {
	    uint sliceId, baseIndex;
            SynthSlice *slice;
            uint nbrSlice = count / FEEDBACK_SLICE_LENGTH;

            feedback->nbrSlice = 0;

            for (sliceId = 0; sliceId < nbrSlice && nbrSlice < ARRAY_SIZE(feedback->slice); sliceId++)
            {
                baseIndex = sliceId * FEEDBACK_SLICE_LENGTH;

                slice = &feedback->slice[sliceId];
                feedback->nbrSlice++;

                slice->sliceId = ctx->defaultButtonConfig.nextSliceId++;
                slice->mHzFreq = dataFeedback[baseIndex + FREQ_INDEX];
                slice->mVAmp = dataFeedback[baseIndex + MV_AMP_INDEX];
                slice->cycle = dataFeedback[baseIndex + CYCLE_INDEX];

                dev_dbg(ctx->dev, "Slice id: %d Freq: %d (milliHertz) Amplitude: %d (mV) Cycle: %d\n",
			slice->sliceId, slice->mHzFreq, slice->mVAmp, slice->cycle);
            }

            res = true;
        }
    }

    return res;
}


#define BUTTON_CONFIG_ELEMENT_LENGTH (5)

#define DIRECTION_INDEX (0)
#define DETECTION_MODE_INDEX (1)
#define THRESHOLD_INDEX (2)
#define DEBOUNCING_INDEX (3)
#define STABILISATION_INDEX (4)

static bool
decodeDetectionDefaultConf(Context* ctx, const char* channelAttr, const char* suffixAttr, SensingConfig* conf)
{
    bool res = false;

    char attr[ATTRIBUTE_CHANNEL_PREFIX_LENGTH];
    size_t expectedLength = strlen(channelAttr) + strlen(suffixAttr);

    u32 buttonConf[BUTTON_CONFIG_ELEMENT_LENGTH * 10];

    if (expectedLength < ATTRIBUTE_CHANNEL_PREFIX_LENGTH &&
        sprintf(attr, "%s,%s", channelAttr, suffixAttr))
    {
        const char *directionS;
        const char *modeS;
        int count = of_property_read_variable_u32_array(ctx->dev->of_node, attr, buttonConf,
                                                        BUTTON_CONFIG_ELEMENT_LENGTH,
                                                        sizeof(buttonConf));

        if (count > 0 && (count % BUTTON_CONFIG_ELEMENT_LENGTH) == 0)
        {
            conf->direction = buttonConf[DIRECTION_INDEX];
            conf->mode = buttonConf[DETECTION_MODE_INDEX];
            conf->thresholdMv = (int16_t) buttonConf[THRESHOLD_INDEX];
            conf->debounceUs = buttonConf[DEBOUNCING_INDEX];
            conf->stabilisationMs = buttonConf[STABILISATION_INDEX];

            directionS = getSensingDirection(conf->direction);
            modeS = getSensingDetectionMode(conf->mode);

            if (directionS != NULL && modeS != NULL)
            {
                dev_dbg(ctx->dev,
                        "Mode: %s Direction: %s Debouncing Duration: %d μs Threshold: %d Stabilization Duration: %d ms\n",
                        modeS, directionS, conf->debounceUs,
                        conf->thresholdMv, conf->stabilisationMs);
                res = true;
            }
        }
    }

    return res;
}

const char* feedbackPress = "button,feedback,press";
const char* feedbackRelease = "button,feedback,release";

const char* buttonPress = "button,press";
const char* buttonRelease = "button,release";

static bool readDefaultConfiguration(Context* ctx)
{
    bool res = true;
    uint channel;

    ctx->defaultButtonConfig.nextSliceId = 0;

    //Decode feedbacks
    res = res && decodeSlicesDefaultConfig(ctx, feedbackPress,
                                           &ctx->defaultButtonConfig.feedback[SENSING_DIRECTION_PRESS]);
    res = res && decodeSlicesDefaultConfig(ctx, feedbackRelease,
                                           &ctx->defaultButtonConfig.feedback[SENSING_DIRECTION_RELEASE]);

    for (channel = 0; channel < BOS0614_NBR_OF_CHANNEL; channel++)
    {
        char attrChannel[ATTRIBUTE_CHANNEL_PREFIX_LENGTH];
        const char* status;

        sprintf(attrChannel, "channel,%d", channel);

        if (of_property_read_string(ctx->dev->of_node, attrChannel, &status) == 0)
        {
            ctx->defaultButtonConfig.channel[channel].present = strcmp(status, "active") == 0;
        }
        else
        {
            dev_warn(ctx->dev, "No default configuration for channel %d\n", channel);
        }


        if (ctx->defaultButtonConfig.channel[channel].present)
        {
            dev_dbg(ctx->dev, "Channel %d is Activated\n", channel);

            res = res && decodeDetectionDefaultConf(ctx, attrChannel, buttonPress,
                                                    &ctx->defaultButtonConfig.channel[channel].detection[SENSING_DIRECTION_PRESS]);
            res = res && decodeDetectionDefaultConf(ctx, attrChannel, buttonRelease,
                                                    &ctx->defaultButtonConfig.channel[channel].detection[SENSING_DIRECTION_RELEASE]);
        }
        else
        {
            dev_dbg(ctx->dev, "Channel %d is Deactivated\n", channel);
        }
    }

    return res;
}

#define IGNORE_OUPUT_CHANNEL (0)
#define DEFAULT_PRESS_WAVEFORM_ID (0)
#define DEFAULT_RELEASE_WAVEFORM_ID (1)


static bool applyingFeedbackButtonConf(Context* ctx, HwButtonFeedback* feedbacks, WaveformId waveformId)
{
    bool res = true;
    uint sliceId;
    HapticDriver *driver;

    if (feedbacks->nbrSlice == 0)
    {
        dev_err(ctx->dev, "No default slices detected for the HW button feedbacks related to waveform id %d",
                waveformId);
        res = false;
    }

    driver = ctx->hapticDriver;

    //Slice configuration
    for (sliceId = 0; sliceId < feedbacks->nbrSlice; sliceId++)
    {
        res = res && driver->synthSetSlice(driver, &feedbacks->slice[sliceId], SHAPE_NO_SHAPE, IGNORE_OUPUT_CHANNEL);
	pr_debug("set_slice: %d\n", sliceId);
    }

    res = res && driver->synthSetWaveform(driver, waveformId, feedbacks->slice->sliceId, feedbacks->nbrSlice, 1,
                                          IGNORE_OUPUT_CHANNEL);

    if (!res)
    {
        dev_err(ctx->dev, "Failed to configure the HW button feedbacks for waveform id %d", waveformId);
    }

    return res;
}

static bool applyButtonSensingConfiguration(Context* ctx, uint buttonId, SensingConfig* conf, WaveformId waveformId)
{

    bool res = true;

    HapticDriver* driver = ctx->hapticDriver;

    pr_debug("enter\n");
    res = res && driver->buttonSensing(driver, buttonId, *conf);
    res = res && driver->sensingAutoPlayWave(driver, buttonId, waveformId, conf->direction);

    if (!res)
    {
        dev_err(ctx->dev, "Failed to configure the HW button sensing for button id %d \n", buttonId);
    }

    return res;
}

static bool applyDefaultConfiguration(Context* ctx)
{
    uint buttonId;
    bool res = applyingFeedbackButtonConf(ctx, &ctx->defaultButtonConfig.feedback[SENSING_DIRECTION_PRESS],
                                          DEFAULT_PRESS_WAVEFORM_ID);
    pr_debug("enter\n");
    res = res && applyingFeedbackButtonConf(ctx, &ctx->defaultButtonConfig.feedback[SENSING_DIRECTION_RELEASE],
                                            DEFAULT_RELEASE_WAVEFORM_ID);
    for (buttonId = 0; buttonId < ARRAY_SIZE(ctx->defaultButtonConfig.channel) && res; buttonId++)
    {
        if (ctx->defaultButtonConfig.channel[buttonId].present)
        {
            dev_dbg(ctx->dev, "Applying default configuration for Channel %d \n", buttonId);

            res = res && applyButtonSensingConfiguration(ctx, buttonId,
                                                         &ctx->defaultButtonConfig.channel[buttonId].detection[SENSING_DIRECTION_PRESS],
                                                         DEFAULT_PRESS_WAVEFORM_ID);
            res = res && applyButtonSensingConfiguration(ctx, buttonId,
                                                         &ctx->defaultButtonConfig.channel[buttonId].detection[SENSING_DIRECTION_RELEASE],
                                                         DEFAULT_RELEASE_WAVEFORM_ID);
        }
    }

    return res;
}

/*
 * Public Section
 */

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
        &dev_attr_is_sensing.attr,
        NULL,
};

static struct attribute_group bosDriverAttrGroup = {
        .attrs = bosDriverAttrs,
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

static const struct i2c_device_id bosDriverI2cId[] = {
    {.name = "bos0614",},
};

static struct of_device_id bos_dt_match[] = {
    {.compatible = "boreas,bos0614",},
    {},
};

static int bosDriverI2cProbe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    int res = -ENODEV;
    int err;
    Context *ctx;
    const struct of_device_id *match;
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

    match = of_match_device(of_match_ptr(bos_dt_match), &client->dev);
    if (match)
    {
        dev_info(&client->dev, "Reading Configuration from DT\n");

        if (!readDefaultConfiguration(ctx) || !applyDefaultConfiguration(ctx))
        {
            dev_err(&client->dev, "Failed to decode DT configuration\n");
            goto bosDriverProbeError;
        }
    } else
        pr_info("cannot find matching device tree\n");

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
        dev_err(&client->dev, "Boreas Haptic Driver failed to probe\n");
        freeResources(ctx);
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
                .owner = THIS_MODULE,
                .name = "bos0614_mmi",
                .of_match_table = of_match_ptr(bos_dt_match),
        },
        .probe = bosDriverI2cProbe,
        .remove = bosDriverI2cRemove,
        .detect = detect,
        .id_table = bosDriverI2cId,
        .address_list = addrList
};

module_i2c_driver(bosDriverI2c);

MODULE_AUTHOR("Pascal-Frédéric St-Laurent <pfstlaurent@boreas.ca>");
MODULE_DESCRIPTION("I2C Driver for Boréas Haptic Technologies");
MODULE_LICENSE("GPL v2");
