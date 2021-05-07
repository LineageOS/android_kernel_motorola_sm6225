//
// Description: BOS0614 Driver Implementation
// Created on 5/5/2020
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

#include <linux/string.h>
#include <linux/slab.h>

#include "contribs/cmsis/CMSIS/Driver/Include/Driver_Common.h"
#include "contribs/comm-stack-dk/data.h"

#include "bsp/boards/boreasTime.h"
#include "bsp/drivers/i2c/i2c.h"
#include "bsp/drivers/haptic/bos0614Register.h"
#include "bsp/drivers/haptic/bos0614Driver.h"

#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_info
#undef dev_dbg
#define dev_dbg dev_info
#endif

#define BOS0614_CHIP_ID (0x0D)
#define BOS0614_I2C_ADDRESS (0x2c)

#define SFT_RESET_TIME_IN_US (100)
#define AMPLITUDE_MAX_VALUE_VOLT (60)
#define AMPLITUDE_MAX_VALUE (0xFFF)
#define FB_R_V_REF (68.4)

#define FREQUENCY_RESOLUTION_HZ (3.90625)

#define BURST_WRITE_RAM_BANK_INDEX (1)
#define BURST_WRITE_RAM_ADDRESS_INDEX (3)
#define BURST_WRITE_RAM_LENGTH_INDEX (5)
#define BURST_WRITE_RAM_DATA_INDEX (7)
#define WRITE_RAM_BANK_INDEX (1)
#define WRITE_RAM_DATA_INDEX (3)

#define RAM_ADDRESS_INDEX (0)
#define RAM_ADDRESS_LENGTH (1)
#define SEQUENCER_STOP_ADDRESS_SHIFT (12)

#define MAXIMUM_DATA_LENGTH (1024)
#define MAXIMUM_SAMPLING_RATE (1024)

#define REG_VALUE_LENGTH (2)
#define WRITE_REG_LENGTH (3)

#define I2C_ADDRESS_INDEX (0)
#define I2C_ADDRESS_REG_DATA_INDEX (1)

#define BOS0614_ENABLE (1)
#define BOS0614_DISABLE (0)

#define BOS0614_THRESH_STEP_mV (1.66)
#define BOS0614_SLOPE_THRESH_STEP_mV (2.2)

#define THRESHOLD_NBR_OF_BITS (12)
#define SLOPE_THRESHOLD_NBR_OF_BITS (7)

#define MAXIMUM_WAVEFORM_ID_FOR_AUTO_SENSING (1)

#define MAX_TC_VALUE (31)

#define MAXIMUM_NUMBER_OF_SLICE (597)
#define MAXIMUM_WAVEFORM_ID_SEQUENCER (15)
#define WAVEFORM_METADATA_LENGTH (3)
#define MAXIMUM_WAVEFORM_NUMBER (85)

#define RAMPLAYBACK_START_RAM_ADDRESS (0x0000)

typedef struct
{
    uint16_t sequencerCmd[MAXIMUM_WAVEFORM_ID_SEQUENCER];
} Synthesizer;

typedef struct
{
    Gpio *gpio;
    BOSEventCb cb;
    void *ctx;
    Bos0614GPIOMode mode;
} ChannelCtx;

typedef struct
{
    bool isInitiated;
    uint8_t txBuffer[MAXIMUM_DATA_LENGTH * sizeof(uint16_t)]; // Max size in words * 2 bytes
    uint8_t chipId;
    uint8_t currentChannelMask;
    I2c *i2c;
    Spi *spi;
    ChannelCtx channel[BOS0614_NBR_OF_CHANNEL];
    bool gpoSignalingAvailable;
    uint8_t outputChanForWaveformId[MAXIMUM_WAVEFORM_NUMBER];
    HapticDriver hDriver;
    Synthesizer synth;
    BOS0614_REGS reg;
} Context;

typedef enum
{
    WSFBank_Ram = 0x1,
    WSFBank_Sequencer = 0x2,
    WSFBank_SequencerStartStop = 0x012,
    WSFBank_RamPlayback = 0x013,
    WSFBank_BurstRamWrite = 0x014
} WSFBank;

static Context driverInstances[BOS0614_NBR_OF_INSTANCE];

/**
 * Private Section
 */
static void initiateDriver(Context *ctx);

static bool setDefaultConfig(Context *ctx);

static bool writeI2cReg(Context *ctx, Bos0614Register *reg)
{
    bool res = false;

    uint8_t txData[WRITE_REG_LENGTH];
    txData[I2C_ADDRESS_INDEX] = reg->generic.addr;
    htoBe16(reg->generic.value, &txData[I2C_ADDRESS_REG_DATA_INDEX]);

    res = ctx->i2c->write(ctx->i2c, BOS0614_I2C_ADDRESS, txData, sizeof(txData)) == ARM_DRIVER_OK;

    return res;
}

static bool writeSpiReg(Context *ctx, Bos0614Register *reg, uint16_t *rxValue)
{
    bool res = false;
    uint16_t dataIn[sizeof(reg->all)];
    uint16_t data[sizeof(reg->all)];
    data[0] = reg->generic.addr;
    data[1] = reg->generic.value;

    res = ctx->spi->transfer(ctx->spi, data, dataIn, sizeof(data)) == ARM_DRIVER_OK;

    if (res == true && rxValue != NULL)
    {
        *rxValue = dataIn[1];
    }

    return res;
}

static bool writeReg(Context *ctx, Bos0614Register *reg)
{
    bool res = false;

    if (ctx->spi != NULL)
    {
        res = writeSpiReg(ctx, reg, NULL);
    }
    else if (ctx->i2c != NULL)
    {
        res = writeI2cReg(ctx, reg);
    }

    return res;
}

static bool resetSoftware(Context *ctx)
{
    bool res;

    pr_debug("enter\n");
    ctx->reg.CONFIG_0614.bit.RST = 0x1;

    res = writeReg(ctx, &ctx->reg.CONFIG_0614.reg);

    ctx->reg.CONFIG_0614.bit.RST = 0x0;

    return res;
}

static Context *getNewInstance()
{
    int index;
    Context *ctx = NULL;

    for (index = 0; index < DATA_ARRAY_LENGTH(driverInstances); index++)
    {
        if (driverInstances[index].isInitiated == false)
        {
            ctx = &driverInstances[index];
            ctx->isInitiated = true;
            break;
        }
    }

    return ctx;
}

static Bos0614Register *getRegister(Context *ctx, uint8_t addr)
{
    Bos0614Register *reg = NULL;

    switch (addr)
    {
        case ADDRESS_BOS0614_REFERENCE_REG:
            reg = &ctx->reg.REFERENCE_0614.reg;
            break;
        case ADDRESS_BOS0614_IC_STATUS_REG:
            reg = &ctx->reg.IC_STATUS_0614.reg;
            break;
        case ADDRESS_BOS0614_READ_REG    :
            reg = &ctx->reg.READ_0614.reg;
            break;
        case ADDRESS_BOS0614_GPIOX_REG:
            reg = &ctx->reg.GPIOX_0614.reg;
            break;
        case ADDRESS_BOS0614_TC_REG:
            reg = &ctx->reg.TC_0614.reg;
            break;
        case ADDRESS_BOS0614_CONFIG_REG:
            reg = &ctx->reg.CONFIG_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSECONFIG_REG:
            reg = &ctx->reg.SENSECONFIG_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0_REG:
            reg = &ctx->reg.SENSE0_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0P_REG:
            reg = &ctx->reg.SENSE0P_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0R_REG:
            reg = &ctx->reg.SENSE0R_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0S_REG:
            reg = &ctx->reg.SENSE0S_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1_REG:
            reg = &ctx->reg.SENSE1_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1P_REG:
            reg = &ctx->reg.SENSE1P_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1R_REG:
            reg = &ctx->reg.SENSE1R_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1S_REG:
            reg = &ctx->reg.SENSE1S_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2_REG:
            reg = &ctx->reg.SENSE2_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2P_REG:
            reg = &ctx->reg.SENSE2P_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2R_REG:
            reg = &ctx->reg.SENSE2R_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2S_REG:
            reg = &ctx->reg.SENSE2S_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3_REG:
            reg = &ctx->reg.SENSE3_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3P_REG:
            reg = &ctx->reg.SENSE3P_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3R_REG:
            reg = &ctx->reg.SENSE3R_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3S_REG:
            reg = &ctx->reg.SENSE3S_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSESTATUS_REG:
            reg = &ctx->reg.SENSESTATUS_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA0_REG:
            reg = &ctx->reg.SENSEDATA0_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA1_REG:
            reg = &ctx->reg.SENSEDATA1_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA2_REG:
            reg = &ctx->reg.SENSEDATA2_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA3_REG:
            reg = &ctx->reg.SENSEDATA3_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW0_REG:
            reg = &ctx->reg.SENSERAW0_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW1_REG:
            reg = &ctx->reg.SENSERAW1_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW2_REG:
            reg = &ctx->reg.SENSERAW2_0614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW3_REG:
            reg = &ctx->reg.SENSERAW3_0614.reg;
            break;
        case ADDRESS_BOS0614_KPA_REG:
            reg = &ctx->reg.KPA_0614.reg;
            break;
        case ADDRESS_BOS0614_KP_KI_REG:
            reg = &ctx->reg.KP_KI_0614.reg;
            break;
        case ADDRESS_BOS0614_DEADTIME_REG:
            reg = &ctx->reg.DEADTIME_0614.reg;
            break;
        case ADDRESS_BOS0614_PARCAP_REG:
            reg = &ctx->reg.PARCAP_0614.reg;
            break;
        case ADDRESS_BOS0614_SUP_RISE_REG:
            reg = &ctx->reg.SUP_RISE_0614.reg;
            break;
        case ADDRESS_BOS0614_TRIM_REG:
            reg = &ctx->reg.TRIM_0614.reg;
            break;
        case ADDRESS_BOS0614_CHIP_ID_REG:
            reg = &ctx->reg.CHIP_ID_0614.reg;
            break;
        case ADDRESS_BOS0614_VFEEDBACK_REG:
            reg = &ctx->reg.VFEEDBACK_0614.reg;
            break;
        case ADDRESS_BOS0614_FIFO_STATE_REG:
            reg = &ctx->reg.FIFO_STATE_0614.reg;
            break;
        case ADDRESS_BOS0614_AUTO_STATE_REG:
            reg = &ctx->reg.AUTO_STATE_0614.reg;
            break;
        case ADDRESS_BOS0614_BIST_REG:
            reg = &ctx->reg.BIST_0614.reg;
            break;
        case ADDRESS_BOS0614_BISTRES_REG:
            reg = &ctx->reg.BISTRES_0614.reg;
            break;
        case ADDRESS_BOS0614_DEBUG_REG:
            reg = &ctx->reg.DEBUG_0614.reg;
            break;
        case ADDRESS_BOS0614_THRESH_REG:
            reg = &ctx->reg.THRESH_0614.reg;
            break;
        case ADDRESS_BOS0614_REG38_REG:
            reg = &ctx->reg.REG38_0614.reg;
            break;
        case ADDRESS_BOS0614_REG39_REG:
            reg = &ctx->reg.REG39_0614.reg;
            break;
        case ADDRESS_BOS0614_CALIB_DATA_REG:
            reg = &ctx->reg.CALIB_DATA_0614.reg;
            break;
        case ADDRESS_BOS0614_REG3B_REG:
            reg = &ctx->reg.REG3B_0614.reg;
            break;
        case ADDRESS_BOS0614_REG3C_REG:
            reg = &ctx->reg.REG3C_0614.reg;
            break;
        case ADDRESS_BOS0614_REG3D_REG:
            reg = &ctx->reg.REG3D_0614.reg;
            break;
        case ADDRESS_BOS0614_REG3E_REG:
            reg = &ctx->reg.REG3E_0614.reg;
            break;
        case ADDRESS_BOS0614_REG3F_REG:
            reg = &ctx->reg.REG3F_0614.reg;
            break;
        case ADDRESS_BOS0614_REG40_REG:
            reg = &ctx->reg.REG40_0614.reg;
            break;
        case ADDRESS_BOS0614_REG41_REG:
            reg = &ctx->reg.REG41_0614.reg;
            break;
        case ADDRESS_BOS0614_REG42_REG:
            reg = &ctx->reg.REG42_0614.reg;
            break;
        case ADDRESS_BOS0614_REG43_REG:
            reg = &ctx->reg.REG43_0614.reg;
            break;
        case ADDRESS_BOS0614_REG44_REG:
            reg = &ctx->reg.REG44_0614.reg;
            break;
        case ADDRESS_BOS0614_REG45_REG:
            reg = &ctx->reg.REG45_0614.reg;
            break;
        case ADDRESS_BOS0614_REG46_REG:
            reg = &ctx->reg.REG46_0614.reg;
            break;
        case ADDRESS_BOS0614_REG47_REG:
            reg = &ctx->reg.REG47_0614.reg;
            break;
    }

    return reg;
}

static bool readI2cRegister(Context *ctx, Bos0614Register *reg)
{
    bool res;
    uint8_t rxData[REG_VALUE_LENGTH] = {0};

    ctx->reg.READ_0614.bit.BC = reg->generic.addr;
    res = writeReg(ctx, &ctx->reg.READ_0614.reg);

    res = res && ctx->i2c->read(ctx->i2c, BOS0614_I2C_ADDRESS, rxData, REG_VALUE_LENGTH) == ARM_DRIVER_OK;

    if (res)
    {
        reg->generic.value = beToH16(rxData);
    }

    return res;
}

static bool readSpiRegister(Context *ctx, Bos0614Register *reg)
{
    bool res = false;

    ctx->reg.READ_0614.bit.BC = reg->generic.addr;

    if (writeSpiReg(ctx, &ctx->reg.READ_0614.reg, NULL) &&
        writeSpiReg(ctx, &ctx->reg.READ_0614.reg, &reg->generic.value))
    {
        res = true;
    }

    return res;
}

static bool readRegister(Context *ctx, Bos0614Register *reg)
{
    bool res = false;

    if (ctx->i2c != NULL)
    {
        res = readI2cRegister(ctx, reg);
    }
    else if (ctx->spi != NULL)
    {
        res = readSpiRegister(ctx, reg);
    }

    return res;
}

static bool burstWriteRam(Context *driver, WSFBank bank, uint16_t address, uint16_t *data, size_t length)
{
    int i;
    bool res = false;
    if (length < MAXIMUM_DATA_LENGTH)
    {
        memset(driver->txBuffer, 0, sizeof(driver->txBuffer));
        driver->txBuffer[I2C_ADDRESS_INDEX] = ADDRESS_BOS0614_REFERENCE_REG; // [0] Main register map address
        htoBe16(bank, &driver->txBuffer[BURST_WRITE_RAM_BANK_INDEX]); // [1-2] WFS bank
        htoBe16(address, &driver->txBuffer[BURST_WRITE_RAM_ADDRESS_INDEX]); // [3-4] Burst start address
        htoBe16(length, &driver->txBuffer[BURST_WRITE_RAM_LENGTH_INDEX]); // [5-6] Burst data length
        for (i = 0; i < length; i++) // [7..] RAM Data to be forwarded to WFS
        {
            htoBe16(data[i], &driver->txBuffer[(i * 2) + BURST_WRITE_RAM_DATA_INDEX]);
        }

        if (driver->i2c != NULL)
        {
            res = driver->i2c->write(driver->i2c, BOS0614_I2C_ADDRESS, driver->txBuffer,
                                     BURST_WRITE_RAM_DATA_INDEX + (length * 2)) ==
                  ARM_DRIVER_OK;
        }
        else if (driver->spi != NULL)
        {
            res = driver->spi->send(driver->spi, driver->txBuffer, BURST_WRITE_RAM_DATA_INDEX + (length * 2)) ==
                  ARM_DRIVER_OK;
        }
    }

    return res;
}

static bool writeRam(Context *driver, WSFBank bank, uint16_t *data, size_t length)
{
    int i;
    bool res = false;

    memset(driver->txBuffer, 0, sizeof(driver->txBuffer));
    driver->txBuffer[I2C_ADDRESS_INDEX] = ADDRESS_BOS0614_REFERENCE_REG; // [0] Main register map address
    htoBe16(bank, &driver->txBuffer[WRITE_RAM_BANK_INDEX]); // [1-2] WFS bank address
    for (i = 0; i < length; i += 2) // [3..] RAM Data to be forwarded to WFS
    {
        htoBe16(*(data++), &driver->txBuffer[i + WRITE_RAM_DATA_INDEX]);
    }

    if (driver->i2c != NULL)
    {
        res = driver->i2c->write(driver->i2c, BOS0614_I2C_ADDRESS, driver->txBuffer, WRITE_RAM_DATA_INDEX + length) ==
              ARM_DRIVER_OK;
    }
    else if (driver->spi != NULL)
    {
        res = driver->spi->send(driver->spi, driver->txBuffer, WRITE_RAM_DATA_INDEX + length) == ARM_DRIVER_OK;
    }

    return res;
}

#define RAM_WAVEFORM_METADATA_ADDR (0x0000)
#define RAM_SLICE_WAVEFORM_ADDR (0x0100)

#define SLICE_AMPLITUDE_INDEX (1)
#define SLICE_FREQ_CYCLE_INDEX (2)
#define SLICE_SHAPE_INDEX (3)
#define SLICE_LENGTH (3)
#define SLICE_CYCLE_SHIFT (8)
#define SLICE_NO_SHAPE (0)

#define Hz_In_MilliHz (1000)
#define VOLT_TO_MILLI_VOLT (1000)

static bool setSlice(Context *driver, SynthSlice const *slice,
                     uint8_t outputChannel)
{
    uint16_t buffer[SLICE_LENGTH + RAM_ADDRESS_LENGTH];
    uint32_t amplitude_computed =
            (slice->mVAmp * AMPLITUDE_MAX_VALUE) / (AMPLITUDE_MAX_VALUE_VOLT * VOLT_TO_MILLI_VOLT);
    uint8_t channel_computed = outputChannel;

    uint32_t freq_computed_den = FREQUENCY_RESOLUTION_HZ * Hz_In_MilliHz;
    uint16_t freq = (uint16_t) (slice->mHzFreq / freq_computed_den);


    buffer[RAM_ADDRESS_INDEX] = RAM_SLICE_WAVEFORM_ADDR + (slice->sliceId * SLICE_LENGTH);
    buffer[SLICE_AMPLITUDE_INDEX] = (uint16_t) ((channel_computed << 12) | amplitude_computed);
    buffer[SLICE_FREQ_CYCLE_INDEX] = (uint16_t) freq;
    buffer[SLICE_FREQ_CYCLE_INDEX] |= (0xFF & slice->cycle) << SLICE_CYCLE_SHIFT;
    buffer[SLICE_SHAPE_INDEX] = SLICE_NO_SHAPE;

    return writeRam(driver, WSFBank_Ram, buffer, sizeof(buffer));
}

static bool setMode(Context *driver, BOS0614Mode mode)
{
    driver->reg.CONFIG_0614.bit.RAM = mode;

    return writeReg(driver, &driver->reg.CONFIG_0614.reg);
}

static bool setSamplingRate(Context *ctx, uint32_t samplingRateKsps)
{
    bool res = false;
    bool found = true;
    BOS0614SamplingRate rate;

    switch (samplingRateKsps)
    {
        case 1024:
            rate = bos0614SamplingRate_1024Ksps;
            break;
        case 512:
            rate = bos0614SamplingRate_512Ksps;
            break;
        case 256:
            rate = bos0614SamplingRate_256Ksps;
            break;
        case 128:
            rate = bos0614SamplingRate_128Ksps;
            break;
        case 64:
            rate = bos0614SamplingRate_64Ksps;
            break;
        case 32:
            rate = bos0614SamplingRate_32Ksps;
            break;
        case 16:
            rate = bos0614SamplingRate_16Ksps;
            break;
        case 8:
            rate = bos0614SamplingRate_8Ksps;
            break;
        default:
            found = false;
    }

    if (found)
    {
        ctx->reg.CONFIG_0614.bit.PLAY = rate;
        res = writeReg(ctx, &ctx->reg.CONFIG_0614.reg);
    }

    return res;
}

static bool activateChannel(Context *ctx, uint8_t channel)
{
    bool res = false;
    if (channel < BOS0614_NBR_OF_CHANNEL)
    {
        ctx->reg.REFERENCE_0614.bit.FIFO = 0;
        ctx->reg.REFERENCE_0614.bit.CHANNEL = 0x1 << channel;
        res = true;
    }

    return res;
}

static bool validateIcWorking(Context *ctx)
{
    bool res = false;

    ctx->reg.CONFIG_0614.bit.PLAY = bos0614SamplingRate_512Ksps;

    res = writeReg(ctx, &ctx->reg.CONFIG_0614.reg);
    res = res && readRegister(ctx, &ctx->reg.CONFIG_0614.reg);
    res = res && ctx->reg.CONFIG_0614.bit.PLAY == bos0614SamplingRate_512Ksps;

    pr_debug("ic is%s working!!!\n", res ? "" : " NOT");

    return res;
}

static BOSEvent getEventFromGpioState(Bos0614GPIOMode mode, GPIOIsr state)
{
    BOSEvent event = BOS_EVENT_NO_EVENT;

    switch (mode)
    {
        case Bos0614GPOCtrl_WaveformFifoDone:
            if (state == GPIOISR_Falling)
            {
                event = BOS_EVENT_WAVEFORM_DONE;
            }
            break;
        case Bos0614GPOCtrl_ButtonState:
            if (state == GPIOISR_Rising)
            {
                event = BOS_EVENT_BUTTON_RELEASE;
            }
            else if (state == GPIOISR_Falling)
            {
                event = BOS_EVENT_BUTTON_PRESS;
            }
            break;
        case Bos0614GPOCtrl_RequestPlay:
            if (state == GPIOISR_Rising)
            {
                event = BOS_EVENT_NO_REQUEST_TO_PLAY;
            }
            else if (state == GPIOISR_Falling)
            {
                event = BOS_EVENT_REQUEST_TO_PLAY;
            }
            break;
        default:
            event = BOS_EVENT_NO_EVENT;
    }

    return event;
}

static bool lookupGpioCtl(GPOCtrl ctrl, Bos0614GPIOMode *bos0614Ctrl)
{
    bool res = false;

    switch (ctrl)
    {
        case GPO_WaveformFifoDone:
            *bos0614Ctrl = Bos0614GPOCtrl_WaveformFifoDone;
            res = true;
            break;
        case GPO_SenseTrigger:
            *bos0614Ctrl = Bos0614GPOCtrl_SenseTrigger;
            res = true;
            break;
        case GPO_ButtonState:
            *bos0614Ctrl = Bos0614GPOCtrl_ButtonState;
            res = true;
            break;
        default:
            res = false;
    }

    return res;
}

static bool isGpoSignalingAvailable(Context *ctx)
{
    size_t index;
    bool res = false;

    for (index = 0; index < DATA_ARRAY_LENGTH(ctx->channel); index++)
    {
        if (ctx->channel[index].gpio != NULL)
        {
            res = true;
            break;
        }
    }

    return res;
}

static HapticDriver *genericDriverInit(Context *ctx)
{
    HapticDriver *driver = NULL;

    initiateDriver(ctx);

    resetSoftware(ctx);

    if (validateIcWorking(ctx) &&
        setDefaultConfig(ctx))
    {
        ctx->gpoSignalingAvailable = isGpoSignalingAvailable(ctx);
        ctx->chipId = BOS0614_CHIP_ID;
        driver = &ctx->hDriver;
    }
    else
    {
        ctx->isInitiated = false;
    }

    return driver;
}

static bool
getSensingRegistersPerChannel(Context *ctx, uint8_t channel, SensingDirection direction,
                              SENSE_BITS_0614 **senseConfBitField, SENSE_THRESHOLD **senseThreshold,
                              SENSES_BITS_0614 **senseSlope)
{
    bool res = false;

    switch (channel)
    {
        case 0:
            *senseConfBitField = &ctx->reg.SENSE0_0614.bit;
            *senseSlope = &ctx->reg.SENSE0S_0614.bit;

            if (direction == SensingDirection_Press)
            {
                *senseThreshold = &ctx->reg.SENSE0P_0614.bit;
            }
            else if (direction == SensingDirection_Release)
            {
                *senseThreshold = &ctx->reg.SENSE0R_0614.bit;
            }

            res = true;
            break;
        case 1:
            *senseConfBitField = &ctx->reg.SENSE1_0614.bit;
            *senseSlope = &ctx->reg.SENSE1S_0614.bit;

            if (direction == SensingDirection_Press)
            {
                *senseThreshold = &ctx->reg.SENSE1P_0614.bit;
            }
            else if (direction == SensingDirection_Release)
            {
                *senseThreshold = &ctx->reg.SENSE1R_0614.bit;
            }

            res = true;
            break;

        case 2:
            *senseConfBitField = &ctx->reg.SENSE2_0614.bit;
            *senseSlope = &ctx->reg.SENSE2S_0614.bit;

            if (direction == SensingDirection_Press)
            {
                *senseThreshold = &ctx->reg.SENSE2P_0614.bit;
            }
            else if (direction == SensingDirection_Release)
            {
                *senseThreshold = &ctx->reg.SENSE2R_0614.bit;
            }

            res = true;
            break;
        case 3:
            *senseConfBitField = &ctx->reg.SENSE3_0614.bit;
            *senseSlope = &ctx->reg.SENSE3S_0614.bit;

            if (direction == SensingDirection_Press)
            {
                *senseThreshold = &ctx->reg.SENSE3P_0614.bit;
            }
            else if (direction == SensingDirection_Release)
            {
                *senseThreshold = &ctx->reg.SENSE3R_0614.bit;
            }

            res = true;
            break;
        default:
            res = false;
    }

    return res;
}

static bool getDebouncingConfig(uint16_t debounceUs, DebouncingTime *debouncingTime)
{
    bool res = false;

    if (debouncingTime != NULL)
    {
        if (debounceUs == 1)
        {
            *debouncingTime = DebouncingTime_1us;
            res = true;
        }
        else if (debounceUs <= 100)
        {
            *debouncingTime = DebouncingTime_100us;
            res = true;
        }
        else if (debounceUs <= 500)
        {
            *debouncingTime = DebouncingTime_500us;
            res = true;
        }
        else if (debounceUs <= 1000)
        {
            *debouncingTime = DebouncingTime_1ms;
            res = true;
        }
        else if (debounceUs <= 2000)
        {
            *debouncingTime = DebouncingTime_2ms;
            res = true;
        }
        else if (debounceUs <= 4000)
        {
            *debouncingTime = DebouncingTime_4ms;
            res = true;
        }
        else if (debounceUs <= 8000)
        {
            *debouncingTime = DebouncingTime_8ms;
            res = true;
        }
        else if (debounceUs > 8000)
        {
            *debouncingTime = DebouncingTime_16ms;
            res = true;
        }
    }

    return res;
}

#define MILLI_VOLT_TO_MICRO_VOLT(_mV) (_mV*1000)

static int16_t getThresholdFromMV(int16_t thresholdMv)
{
    int32_t tempMicroV = MILLI_VOLT_TO_MICRO_VOLT(thresholdMv);
    int16_t threshold = (int16_t) (tempMicroV / (int32_t) (MILLI_VOLT_TO_MICRO_VOLT(BOS0614_THRESH_STEP_mV)));

    return (int16_t) (threshold);
}

static int16_t getSlopeThresholdFromMv(int16_t thresholdMv)
{
    float tempMv = (float) thresholdMv;
    tempMv = tempMv / BOS0614_SLOPE_THRESH_STEP_mV;

    return (int16_t) tempMv;
}

static bool activateSensingForChannel(Context *ctx, uint8_t channel, bool activate)
{
    bool res = false;

    switch (channel)
    {
        case 0:
            ctx->reg.SENSECONFIG_0614.bit.CH0 = activate;
            res = true;
            break;
        case 1:
            ctx->reg.SENSECONFIG_0614.bit.CH1 = activate;
            res = true;
            break;
        case 2:
            ctx->reg.SENSECONFIG_0614.bit.CH2 = activate;
            res = true;
            break;
        case 3:
            ctx->reg.SENSECONFIG_0614.bit.CH3 = activate;
            res = true;
            break;
    }

    if (res)
    {
        res = writeReg(ctx, &ctx->reg.SENSECONFIG_0614.reg);
    }

    return res;
}

bool bos0614WriteRegister(HapticDriver *driver, Register *reg);

static bool
setThresholdSensing(Context *ctx, SensingConfig config, SENSE_BITS_0614 *senseBitField, SENSE_THRESHOLD *senseThreshold)
{
    bool res = false;

    uint32_t threshold;
    DebouncingTime debouncingValue;

    res = twoComplement(getThresholdFromMV(config.thresholdMv), THRESHOLD_NBR_OF_BITS, &threshold);
    res = getDebouncingConfig(config.debounceUs, &debouncingValue);

    senseThreshold->THRESHOLD = threshold;
    senseThreshold->REP = debouncingValue;

    if (config.direction == SensingDirection_Press || config.direction == SensingDirection_Both)
    {
        senseBitField->T1 = BOS0614_ENABLE;
        senseThreshold->AB = SensingThresholdMode_Above;

        res = true;
    }

    if (config.direction == SensingDirection_Release || config.direction == SensingDirection_Both)
    {
        senseBitField->T2 = BOS0614_ENABLE;
        senseThreshold->AB = SensingThresholdMode_Below;

        res = true;
    }

    res = res && bos0614WriteRegister(&ctx->hDriver, (Register *) senseBitField);
    res = res && bos0614WriteRegister(&ctx->hDriver, (Register *) senseThreshold);

    return res;
}

static bool
setSlopeSensing(Context *ctx, SensingConfig config, SENSE_BITS_0614 *senseBitField, SENSES_BITS_0614 *senseSlope)
{
    uint32_t slope;
    bool res = twoComplement(getSlopeThresholdFromMv(config.thresholdMv), SLOPE_THRESHOLD_NBR_OF_BITS, &slope);

    if (config.direction == SensingDirection_Press || config.direction == SensingDirection_Both)
    {
        senseBitField->S1 = BOS0614_ENABLE;
        senseSlope->SLOPE1 = slope;
        senseSlope->ABS1 = SensingThresholdMode_Above;
    }

    if (config.direction == SensingDirection_Release || config.direction == SensingDirection_Both)
    {
        senseBitField->S2 = BOS0614_ENABLE;
        senseSlope->SLOPE2 = slope;
        senseSlope->ABS2 = SensingThresholdMode_Below;
    }

    res = res && bos0614WriteRegister(&ctx->hDriver, (Register *) senseSlope);
    res = res && bos0614WriteRegister(&ctx->hDriver, (Register *) senseBitField);

    return res;
}

static size_t pushErrorInQueue(BOSError *errors, size_t curpos, size_t maxLength, BOSError error)
{
    size_t length = curpos;

    if (curpos < maxLength)
    {
        errors[length++] = error;
    }

    return length;
}

/**
 * Public Section
 */
HapticDriver *bos0614DriverSpiInit(Spi *spi, Gpio *gpioA, Gpio *gpioD)
{
    HapticDriver *driver = NULL;
    Context *ctx = getNewInstance();

    if (spi != NULL && gpioA != NULL && gpioD != NULL &&
        ctx != NULL)
    {
        ctx->spi = spi;
        ctx->channel[0].gpio = gpioA;
        ctx->channel[3].gpio = gpioD;

        driver = genericDriverInit(ctx);
    }

    return driver;
}

HapticDriver *bos0614DriverI2cInit(Bos0614Resource resources)
{
    HapticDriver *driver = NULL;
    Context *ctx = getNewInstance();

    if (ctx != NULL && resources.i2c != NULL)
    {
        ctx->i2c = resources.i2c;
        ctx->channel[0].gpio = resources.gpioA;
        ctx->channel[1].gpio = resources.gpioB;
        ctx->channel[2].gpio = resources.gpioC;
        ctx->channel[3].gpio = resources.gpioD;

        driver = genericDriverInit(ctx);
    }

    return driver;
}

uint8_t bos0614DriverGetChipId(HapticDriver *driver)
{
    uint8_t chipId = 0;

    if (driver != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        chipId = ctx->chipId;
    }

    return chipId;
}

bool bos0614GetRegister(HapticDriver *driver, uint8_t addr, uint16_t *value)
{
    bool res = false;

    if (driver != NULL)
    {
        Bos0614Register *reg;
        Context *ctx = container_of(driver, Context, hDriver);


        if (addr >= ADDRESS_BOS0614_BIST_REG)
        {
            ctx->reg.DEBUG_0614.bit.ACCESS = 0x3A;
            writeReg(ctx, &ctx->reg.DEBUG_0614.reg);
        }

        reg = getRegister(ctx, addr);
        if (reg != NULL)
        {
            if (readRegister(ctx, reg))
            {
                *value = reg->generic.value;
                res = true;
            }
        }

    }

    return res;
}

bool bos0614SetRegister(HapticDriver *driver, uint8_t addr, uint16_t value)
{
    bool res = false;

    if (driver != NULL)
    {
        Bos0614Register *reg;
        Context *ctx = container_of(driver, Context, hDriver);


        if (addr >= ADDRESS_BOS0614_BIST_REG)
        {
            ctx->reg.DEBUG_0614.bit.ACCESS = 0x3A;
            writeReg(ctx, &ctx->reg.DEBUG_0614.reg);
        }

        reg = getRegister(ctx, addr);
        if (reg != NULL)
        {
            reg->generic.value = value;
            res = writeReg(ctx, reg);
        }
    }

    return res;
}

bool bos0614WriteRegister(HapticDriver *driver, Register *reg)
{
    bool res = false;

    if (driver != NULL && reg != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        res = writeReg(ctx, &reg->register0614);

    }

    return res;
}

bool bos0614ReadRegister(HapticDriver *driver, Register *reg)
{
    bool res = false;

    if (driver != NULL && reg != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        res = readRegister(ctx, &reg->register0614);
    }

    return res;
}

bool bos0614CtrlOutput(HapticDriver *driver, bool activate)
{
    bool res = false;

    if (driver != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        ctx->reg.CONFIG_0614.bit.OE = activate ? 1 : 0; //Enable waveform playback

        res = writeReg(ctx, &ctx->reg.CONFIG_0614.reg);
    }
    return res;
}

bool
bos0614SetSlice(HapticDriver *driver, const SynthSlice *slice, const uint8_t outputChannel)
{
    bool res = false;

    if (driver != NULL && slice != NULL && slice->sliceId < MAXIMUM_NUMBER_OF_SLICE &&
        outputChannel < BOS0614_CHANNEL_MASK)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        res = setMode(ctx, bos0614Mode_RAM_Synthesis);
        res = res && setSlice(ctx, slice, outputChannel);
    }

    return res;
}

bool
bos0614SetWaveforms(HapticDriver *driver, WaveformId id, uint8_t startSliceId, size_t nbrOfSlices, uint16_t cycle,
                    uint8_t outputChannel)
{
    bool res = false;

    if (driver != NULL && id < MAXIMUM_WAVEFORM_NUMBER && nbrOfSlices < MAXIMUM_NUMBER_OF_SLICE &&
        outputChannel < BOS0614_CHANNEL_MASK)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        uint16_t waveformAddr = RAM_WAVEFORM_METADATA_ADDR + (id * WAVEFORM_METADATA_LENGTH);

        uint16_t command[] = {waveformAddr,
                              RAM_SLICE_WAVEFORM_ADDR + (startSliceId * SLICE_LENGTH),
                              RAM_SLICE_WAVEFORM_ADDR + ((startSliceId + nbrOfSlices) * SLICE_LENGTH) - 1,
                              (uint16_t) cycle};

        ctx->outputChanForWaveformId[id] = outputChannel;

        res = writeRam(ctx, WSFBank_Ram, command, sizeof(command));

        ctx->synth.sequencerCmd[id] = (waveformAddr);

        res = res && writeRam(ctx, WSFBank_Sequencer, ctx->synth.sequencerCmd, sizeof(ctx->synth.sequencerCmd));
    }

    return res;
}

bool bos0614SynthesizerPlay(HapticDriver *driver, WaveformId start, WaveformId stop)
{
    bool res = false;

    if (driver != NULL && start < MAXIMUM_WAVEFORM_ID_SEQUENCER && stop < MAXIMUM_WAVEFORM_ID_SEQUENCER)
    {
        uint16_t startStopAddr;
        Context *ctx = container_of(driver, Context, hDriver);

        ctx->currentChannelMask = ctx->outputChanForWaveformId[start];

        startStopAddr = (start << SEQUENCER_STOP_ADDRESS_SHIFT) | stop;

        res = writeRam(ctx, WSFBank_SequencerStartStop, &startStopAddr, sizeof(startStopAddr));
    }

    return res;
}

#define RAMPLAYBACK_DATA_MASK (0xFFF)
#define RAMPLAYBACK_CHANNEL_OFFSET (12)

bool bos0614SetRamPlaybackMode(HapticDriver *driver, uint32_t samplingRate, void *data, size_t length,
                               uint8_t channelMask)
{
    int index;
    uint16_t *buffer;
    bool res = false;

    if (driver != NULL && samplingRate <= MAXIMUM_SAMPLING_RATE && data != NULL && length <= MAXIMUM_DATA_LENGTH &&
        channelMask < BOS0614_CHANNEL_MASK)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        res = setMode(ctx, bos0614Mode_RAM_Playback);
        res = res && setSamplingRate(ctx, samplingRate);

        //Configure the channel output
        buffer = (uint16_t *) data;
        for (index = 0; index < length; index++)
        {
            buffer[index] &= RAMPLAYBACK_DATA_MASK;
            buffer[index] |= (channelMask << RAMPLAYBACK_CHANNEL_OFFSET);
        }

        res = res && burstWriteRam(ctx, WSFBank_BurstRamWrite, RAMPLAYBACK_START_RAM_ADDRESS, data, length);
        ctx->currentChannelMask = channelMask;
    }

    return res;
}

bool bos0614SetFifoMode(HapticDriver *driver, uint32_t samplingRate, uint8_t outputChannel)
{
    bool res = false;

    if (driver != NULL && samplingRate <= MAXIMUM_SAMPLING_RATE && outputChannel < BOS0614_NBR_OF_CHANNEL)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        res = setMode(ctx, bos0614Mode_FIFO);
        res = res && setSamplingRate(ctx, samplingRate);
        res = res && activateChannel(ctx, outputChannel);
    }

    return res;
}

bool bos0614GetFifoSpace(HapticDriver *driver, uint16_t *availableSpace)
{
    bool res = false;

    if (driver != NULL && availableSpace != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        uint16_t fifoReg = 0;

        res = ctx->hDriver.getRegister(&ctx->hDriver, ADDRESS_BOS0614_FIFO_STATE_REG, &fifoReg);

        if (res)
        {
            if (ctx->reg.FIFO_STATE_0614.bit.EMPTY)
                *availableSpace = MAXIMUM_DATA_LENGTH;
            else if (ctx->reg.FIFO_STATE_0614.bit.FULL)
                *availableSpace = 0;
            else
                *availableSpace = ctx->reg.FIFO_STATE_0614.bit.FIFO_SPACE;
        }
    }
    return res;
}

uint16_t bos0614GetMaxFifoSpace()
{
    return MAXIMUM_DATA_LENGTH;
}

static bool writeDataInFifo(Context *ctx, void *data, size_t length)
{
    int index;
    bool res = false;
    uint16_t *dataArray = (uint16_t *) data;

    for (index = 0; index < length; index++)
    {
        res = true;
        ctx->reg.REFERENCE_0614.bit.FIFO = dataArray[index];

        writeReg(ctx, &ctx->reg.REFERENCE_0614.reg);
    }

    return res;
}

bool bos0614WriteFifo(HapticDriver *ctx, void *data, size_t length)
{
    bool res = false;

    if (ctx != NULL && data != NULL && length > 0)
    {
        Context *driver = container_of(ctx, Context, hDriver);
        res = writeDataInFifo(driver, data, length);
    }

    return res;
}

bool bos0614PlayRamPlayback(HapticDriver *ctx, size_t length)
{
    bool res = false;

    if (ctx != NULL)
    {
        Context *driver = container_of(ctx, Context, hDriver);

        uint16_t cmd[] = {RAMPLAYBACK_START_RAM_ADDRESS, RAMPLAYBACK_START_RAM_ADDRESS + (uint16_t) length - 1};

        res = writeRam(driver, WSFBank_RamPlayback, cmd, sizeof(cmd));
    }

    return res;
}

static void gpoIsr(Gpio *gpio, void *context, GPIOIsr isrEvent)
{
    if (context != NULL)
    {
        Context *ctx = (Context *) context;
	uint32_t index;

        for (index = 0; index < DATA_ARRAY_LENGTH(ctx->channel); index++)
        {
            if (ctx->channel[index].gpio != NULL &&
                ctx->channel[index].gpio == gpio &&
                ctx->channel[index].cb != NULL)
            {
                BOSEvent event = getEventFromGpioState(ctx->channel[index].mode, isrEvent);

                if (event != BOS_EVENT_NO_EVENT)
                {
                    ctx->channel[index].cb(&ctx->hDriver, (ChannelId) index, event, ctx->channel[index].ctx);
                }
                break;
            }
        }
    }
}

bool bos0614ConfigGPO(HapticDriver *driver, uint8_t channel, GPOCtrl state)
{
    bool res = false;

    if (driver != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        Bos0614GPIOMode bos0614Mode;

        if (lookupGpioCtl(state, &bos0614Mode))
        {
            ctx->channel[channel].mode = bos0614Mode;

            switch (channel)
            {
                case 0:
                    ctx->reg.GPIOX_0614.bit.GPIO0 = bos0614Mode;
                    res = true;
                    break;
                case 1:
                    ctx->reg.GPIOX_0614.bit.GPIO1 = bos0614Mode;
                    res = true;
                    break;
                case 2:
                    ctx->reg.GPIOX_0614.bit.GPIO2 = bos0614Mode;
                    res = true;
                    break;
                case 3:
                    ctx->reg.GPIOX_0614.bit.GPIO3 = bos0614Mode;
                    res = true;
                    break;
            }

            res = res && writeReg(ctx, &ctx->reg.GPIOX_0614.reg);

        }
    }

    return res;
}

bool bos0614RegisterOnEvents(HapticDriver *driver, ChannelId channelId, BOSEventCb cb, void *context)
{
    bool res = false;

    if (driver != NULL && channelId < BOS0614_NBR_OF_CHANNEL && cb != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        Gpio *gpio = ctx->channel[channelId].gpio;

        GPIOIsr isrMode = GPIOISR_Both;

        if (gpio != NULL)
        {
            ctx->channel[channelId].cb = cb;
            ctx->channel[channelId].ctx = context;
            res = gpio->registerIsr(gpio, isrMode, gpoIsr, ctx, true);
        }
    }

    return res;
}

bool bos0614UnregisterEvents(HapticDriver *driver, ChannelId channelId)
{
    bool res = false;

    if (driver != NULL && channelId < BOS0614_NBR_OF_CHANNEL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        ChannelCtx *channelCtx = &ctx->channel[channelId];

        if (channelCtx->gpio != NULL)
        {
            channelCtx->cb = NULL;
            channelCtx->ctx = NULL;
            channelCtx->gpio->registerIsr(channelCtx->gpio, GPIOISR_None, NULL, NULL, false);
        }
    }

    return res;
}

bool bos0614SoftwareReset(HapticDriver *driver)
{
    bool res = false;

    if (driver != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        resetSoftware(ctx);
    }

    return res;
}

bool bos0614DeepSleep(HapticDriver *ctx, bool enable)
{
    bool res = false;

    if (ctx != NULL)
    {
        Context *driver = container_of(ctx, Context, hDriver);
        uint16_t dummyValue;

        res = bos0614GetRegister(ctx, ADDRESS_BOS0614_CONFIG_REG, &dummyValue);
        driver->reg.CONFIG_0614.bit.DS = enable ? 0x1 : 0x0;
        res = res && bos0614SetRegister(ctx, ADDRESS_BOS0614_CONFIG_REG, driver->reg.CONFIG_0614.reg.all);
//        timeWaitUs(BOS0614_WAKE_FROM_SLEEP_DELAY_US);
    }

    return res;
}

bool bos0614Synch(HapticDriver *driver, bool activate)
{
    (void) driver;
    (void) activate;
    return false;
}

size_t bos0614GetError(HapticDriver *driver, BOSError *errors, size_t maxLength)
{
    size_t length = 0;

    if (driver != NULL && errors != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        uint16_t dummyData;

        if (bos0614GetRegister(driver, ADDRESS_BOS0614_IC_STATUS_REG, &dummyData))
        {
            if (ctx->reg.IC_STATUS_0614.bit.SC != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOSERROR_SHORT_CIRCUIT);
            }

            if (ctx->reg.IC_STATUS_0614.bit.UVLO != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOSERROR_UVLO);
            }

            if (ctx->reg.IC_STATUS_0614.bit.IDAC != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOSERROR_IDAC);
            }

            if (ctx->reg.IC_STATUS_0614.bit.MAXPOWER != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOSERROR_MAX_POWER);
            }


            if (ctx->reg.IC_STATUS_0614.bit.OVT != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOSERROR_OVT);
            }

            if (ctx->reg.IC_STATUS_0614.bit.OVV != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOSERROR_OVV);
            }
        }
    }

    return length;
}

size_t bos0614NbrOfRegister()
{
    return (BOS0614_MAXIMUM_REG_ADDR + 1);
}

bool bos0614ReferencingFromVolt(HapticDriver *ctx, int16_t *data, size_t length)
{
    int index;
    bool res = false;

    if (ctx != NULL)
    {
        for (index = 0; index < length; index++)
        {

            float num = (float) data[index] * AMPLITUDE_MAX_VALUE;
            float den = FB_R_V_REF;
            float res = num / den;

            data[index] = (int16_t) (res);
        }
    }

    return res;
}

GPIOState bos0614GetGpioState(HapticDriver *driver, uint8_t channel)
{
    return GPIOState_Invalid;
}

Registers *bos0614GetShadowRegisters(HapticDriver *driver)
{
    Registers *registers = NULL;

    if (driver != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        registers = (Registers *) &ctx->reg;
    }

    return registers;

}

bool bos0614SensingConfig(HapticDriver *driver, uint8_t channel, SensingConfig config)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        ChannelCtx *channelCtx = &ctx->channel[channel];

        SENSE_BITS_0614 *senseBitField;
        SENSE_THRESHOLD *senseThreshold;
        SENSES_BITS_0614 *senseSlope;

        if (getSensingRegistersPerChannel(ctx, channel, config.direction, &senseBitField, &senseThreshold,
                                          &senseSlope) &&
            channelCtx != NULL)
        {
            switch (config.mode)
            {
                case SensingDetectionMode_Threshold:
                    res = setThresholdSensing(ctx, config, senseBitField, senseThreshold);
                    break;
                case SensingDetectionMode_Slope:
                    res = setSlopeSensing(ctx, config, senseBitField, senseSlope);
            }

            //Setup Stabilization
            ctx->reg.TC_0614.bit.POL = 0;
            ctx->reg.TC_0614.bit.PC = 0x1;
            ctx->reg.TC_0614.bit.TC = MAX_TC_VALUE;

            ctx->reg.SENSECONFIG_0614.bit.SCOMP = 1;

            res = res && bos0614WriteRegister(driver, (Register *) &ctx->reg.TC_0614.reg);
            res = res && bos0614WriteRegister(driver, (Register *) &ctx->reg.SENSECONFIG_0614.reg);


            res = res && activateSensingForChannel(ctx, channel, BOS0614_ENABLE);
        }
    }

    return res;
}

bool
bos0614SensingAutoPlayWave(HapticDriver *driver, uint8_t channel, WaveformId id, SensingDirection direction)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL) // && id <= MAXIMUM_WAVEFORM_ID_FOR_AUTO_SENSING)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        SENSE_BITS_0614 *senseBitField;
        SENSE_THRESHOLD *senseThreshold;
        SENSES_BITS_0614 *senseSlope;

        if (getSensingRegistersPerChannel(ctx, channel, direction, &senseBitField, &senseThreshold, &senseSlope))
        {
            if (direction == SensingDirection_Press)
            {
                senseBitField->WVP = id;
                senseBitField->AUTOP = BOS0614_ENABLE;
                res = true;
            }
            else if (direction == SensingDirection_Release)
            {
                senseBitField->WVR = id;
                senseBitField->AUTOR = BOS0614_ENABLE;
                res = true;
            }

            res = res && bos0614WriteRegister(driver, (Register *) senseBitField);
        }
    }

    return res;
}

bool bos0614SensingStop(HapticDriver *driver, uint8_t channel, SensingDirection direction)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL && direction <= SensingDirection_Both)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        SENSE_BITS_0614 *senseBitField;
        SENSE_THRESHOLD *senseThreshold;
        SENSES_BITS_0614 *senseSlope;

        if (getSensingRegistersPerChannel(ctx, channel, direction, &senseBitField, &senseThreshold, &senseSlope))
        {
            if (direction == SensingDirection_Press || direction == SensingDirection_Both)
            {
                senseBitField->AUTOP = BOS0614_DISABLE;
                senseBitField->T1 = BOS0614_DISABLE;
                senseBitField->S1 = BOS0614_DISABLE;
            }

            if (direction == SensingDirection_Release || direction == SensingDirection_Both)
            {
                senseBitField->AUTOR = BOS0614_DISABLE;
                senseBitField->T2 = BOS0614_DISABLE;
                senseBitField->S2 = BOS0614_DISABLE;
            }

            res = bos0614WriteRegister(driver, (Register *) senseBitField);
            res = res && activateSensingForChannel(ctx, channel, BOS0614_DISABLE);
        }
    }

    return res;
}

bool bos0614FeatureSupport(HapticDriver *driver, BosFeature feature)
{
    bool res = false;

    if (driver != NULL && feature < BosFeature_Length)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        switch (feature)
        {
            case BosFeature_GPOSignaling:
                if (ctx->gpoSignalingAvailable)
                {
                    res = true;
                }
                break;
            case BosFeature_Fifo:
            case BosFeature_Synthesizer:
            case BosFeature_Ramplayback:
            case BosFeature_SensingWithAutomaticFeedback:
                res = true;
                break;
            default:
                res = false;
        }
    }
    return res;
}

/**
 * Private Section
 */

static BOS0614_REGS bOS0614Regs =
        {
                .REFERENCE_0614.reg.generic.addr = ADDRESS_BOS0614_REFERENCE_REG,
                .IC_STATUS_0614.reg.generic.addr = ADDRESS_BOS0614_IC_STATUS_REG,
                .READ_0614.reg.generic.addr = ADDRESS_BOS0614_READ_REG,
                .GPIOX_0614.reg.generic.addr = ADDRESS_BOS0614_GPIOX_REG,
                .TC_0614.reg.generic.addr = ADDRESS_BOS0614_TC_REG,
                .CONFIG_0614.reg.generic.addr = ADDRESS_BOS0614_CONFIG_REG,
                .SENSECONFIG_0614.reg.generic.addr = ADDRESS_BOS0614_SENSECONFIG_REG,
                .SENSE0_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0_REG,
                .SENSE0P_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0P_REG,
                .SENSE0R_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0R_REG,
                .SENSE0S_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0S_REG,
                .SENSE1_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1_REG,
                .SENSE1P_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1P_REG,
                .SENSE1R_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1R_REG,
                .SENSE1S_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1S_REG,
                .SENSE2_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2_REG,
                .SENSE2P_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2P_REG,
                .SENSE2R_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2R_REG,
                .SENSE2S_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2S_REG,
                .SENSE3_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3_REG,
                .SENSE3P_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3P_REG,
                .SENSE3R_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3R_REG,
                .SENSE3S_0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3S_REG,
                .SENSESTATUS_0614.reg.generic.addr = ADDRESS_BOS0614_SENSESTATUS_REG,
                .SENSEDATA0_0614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA0_REG,
                .SENSEDATA1_0614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA1_REG,
                .SENSEDATA2_0614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA2_REG,
                .SENSEDATA3_0614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA3_REG,
                .SENSERAW0_0614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW0_REG,
                .SENSERAW1_0614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW1_REG,
                .SENSERAW2_0614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW2_REG,
                .SENSERAW3_0614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW3_REG,
                .KPA_0614.reg.generic.addr = ADDRESS_BOS0614_KPA_REG,
                .KP_KI_0614.reg.generic.addr = ADDRESS_BOS0614_KP_KI_REG,
                .DEADTIME_0614.reg.generic.addr = ADDRESS_BOS0614_DEADTIME_REG,
                .PARCAP_0614.reg.generic.addr = ADDRESS_BOS0614_PARCAP_REG,
                .SUP_RISE_0614.reg.generic.addr = ADDRESS_BOS0614_SUP_RISE_REG,
                .TRIM_0614.reg.generic.addr = ADDRESS_BOS0614_TRIM_REG,
                .CHIP_ID_0614.reg.generic.addr = ADDRESS_BOS0614_CHIP_ID_REG,
                .VFEEDBACK_0614.reg.generic.addr = ADDRESS_BOS0614_VFEEDBACK_REG,
                .FIFO_STATE_0614.reg.generic.addr = ADDRESS_BOS0614_FIFO_STATE_REG,
                .AUTO_STATE_0614.reg.generic.addr = ADDRESS_BOS0614_AUTO_STATE_REG,
                .BIST_0614.reg.generic.addr = ADDRESS_BOS0614_BIST_REG,
                .BISTRES_0614.reg.generic.addr = ADDRESS_BOS0614_BISTRES_REG,
                .DEBUG_0614.reg.generic.addr = ADDRESS_BOS0614_DEBUG_REG,
                .THRESH_0614.reg.generic.addr = ADDRESS_BOS0614_THRESH_REG,
                .REG38_0614.reg.generic.addr = ADDRESS_BOS0614_REG38_REG,
                .REG39_0614.reg.generic.addr = ADDRESS_BOS0614_REG39_REG,
                .CALIB_DATA_0614.reg.generic.addr = ADDRESS_BOS0614_CALIB_DATA_REG,
                .REG3B_0614.reg.generic.addr = ADDRESS_BOS0614_REG3B_REG,
                .REG3C_0614.reg.generic.addr = ADDRESS_BOS0614_REG3C_REG,
                .REG3D_0614.reg.generic.addr = ADDRESS_BOS0614_REG3D_REG,
                .REG3E_0614.reg.generic.addr = ADDRESS_BOS0614_REG3E_REG,
                .REG3F_0614.reg.generic.addr = ADDRESS_BOS0614_REG3F_REG,
                .REG40_0614.reg.generic.addr = ADDRESS_BOS0614_REG40_REG,
                .REG41_0614.reg.generic.addr = ADDRESS_BOS0614_REG41_REG,
                .REG42_0614.reg.generic.addr = ADDRESS_BOS0614_REG42_REG,
                .REG43_0614.reg.generic.addr = ADDRESS_BOS0614_REG43_REG,
                .REG44_0614.reg.generic.addr = ADDRESS_BOS0614_REG44_REG,
                .REG45_0614.reg.generic.addr = ADDRESS_BOS0614_REG45_REG,
                .REG46_0614.reg.generic.addr = ADDRESS_BOS0614_REG46_REG,
                .REG47_0614.reg.generic.addr = ADDRESS_BOS0614_REG47_REG,
        };

static Bos0614RegisterStruct regToRead[] = {
                {.addr = ADDRESS_BOS0614_REFERENCE_REG,},
                {.addr = ADDRESS_BOS0614_IC_STATUS_REG,},
                {.addr = ADDRESS_BOS0614_READ_REG,},
                {.addr = ADDRESS_BOS0614_GPIOX_REG,},
                {.addr = ADDRESS_BOS0614_TC_REG,},
                {.addr = ADDRESS_BOS0614_CONFIG_REG,},
                {.addr = ADDRESS_BOS0614_SENSECONFIG_REG,},
                {.addr = ADDRESS_BOS0614_SENSE0_REG,},
                {.addr = ADDRESS_BOS0614_SENSE0P_REG,},
                {.addr = ADDRESS_BOS0614_SENSE0R_REG,},
                {.addr = ADDRESS_BOS0614_SENSE0S_REG,},
                {.addr = ADDRESS_BOS0614_SENSE1_REG,},
                {.addr = ADDRESS_BOS0614_SENSE1P_REG,},
                {.addr = ADDRESS_BOS0614_SENSE1R_REG,},
                {.addr = ADDRESS_BOS0614_SENSE1S_REG,},
                {.addr = ADDRESS_BOS0614_SENSE2_REG,},
                {.addr = ADDRESS_BOS0614_SENSE2P_REG,},
                {.addr = ADDRESS_BOS0614_SENSE2R_REG,},
                {.addr = ADDRESS_BOS0614_SENSE2S_REG,},
                {.addr = ADDRESS_BOS0614_SENSE3_REG,},
                {.addr = ADDRESS_BOS0614_SENSE3P_REG,},
                {.addr = ADDRESS_BOS0614_SENSE3R_REG,},
                {.addr = ADDRESS_BOS0614_SENSE3S_REG,},
                {.addr = ADDRESS_BOS0614_SENSESTATUS_REG,},
                {.addr = ADDRESS_BOS0614_SENSEDATA0_REG,},
                {.addr = ADDRESS_BOS0614_SENSEDATA1_REG,},
                {.addr = ADDRESS_BOS0614_SENSEDATA2_REG,},
                {.addr = ADDRESS_BOS0614_SENSEDATA3_REG,},
                {.addr = ADDRESS_BOS0614_SENSERAW0_REG,},
                {.addr = ADDRESS_BOS0614_SENSERAW1_REG,},
                {.addr = ADDRESS_BOS0614_SENSERAW2_REG,},
                {.addr = ADDRESS_BOS0614_SENSERAW3_REG,},
                {.addr = ADDRESS_BOS0614_KPA_REG,},
                {.addr = ADDRESS_BOS0614_KP_KI_REG,},
                {.addr = ADDRESS_BOS0614_DEADTIME_REG,},
                {.addr = ADDRESS_BOS0614_PARCAP_REG,},
                {.addr = ADDRESS_BOS0614_SUP_RISE_REG,},
                {.addr = ADDRESS_BOS0614_TRIM_REG,},
                {.addr = ADDRESS_BOS0614_CHIP_ID_REG,},
                {.addr = ADDRESS_BOS0614_VFEEDBACK_REG,},
                {.addr = ADDRESS_BOS0614_FIFO_STATE_REG,},
                {.addr = ADDRESS_BOS0614_AUTO_STATE_REG,},
                {.addr = ADDRESS_BOS0614_BIST_REG,},
                {.addr = ADDRESS_BOS0614_BISTRES_REG,},
                {.addr = ADDRESS_BOS0614_DEBUG_REG,},
                {.addr = ADDRESS_BOS0614_THRESH_REG,},
                {.addr = ADDRESS_BOS0614_REG38_REG,},
                {.addr = ADDRESS_BOS0614_REG39_REG,},
                {.addr = ADDRESS_BOS0614_CALIB_DATA_REG,},
                {.addr = ADDRESS_BOS0614_REG3B_REG,},
                {.addr = ADDRESS_BOS0614_REG3C_REG,},
                {.addr = ADDRESS_BOS0614_REG3D_REG,},
                {.addr = ADDRESS_BOS0614_REG3E_REG,},
                {.addr = ADDRESS_BOS0614_REG3F_REG,},
                {.addr = ADDRESS_BOS0614_REG40_REG,},
                {.addr = ADDRESS_BOS0614_REG41_REG,},
                {.addr = ADDRESS_BOS0614_REG42_REG,},
                {.addr = ADDRESS_BOS0614_REG43_REG,},
                {.addr = ADDRESS_BOS0614_REG44_REG,},
                {.addr = ADDRESS_BOS0614_REG45_REG,},
                {.addr = ADDRESS_BOS0614_REG46_REG,},
                {.addr = ADDRESS_BOS0614_REG47_REG,},
        };

Bos0614RegisterStruct *getAllRegsPtr(void)
{
	return regToRead;
}

static bool readAllRegister(Context *ctx)
{
    uint32_t index;
    bool res = true;

    for (index = 0; index < DATA_ARRAY_LENGTH(regToRead) && res; index++)
    {
        uint16_t dummy;
        res = bos0614GetRegister(&ctx->hDriver, regToRead[index].addr, &dummy);
    }

    return res;
}

static bool setDefaultConfig(Context *ctx)
{
    bool res = true;

    pr_debug("enter\n");

    //No need to copy registers again here!!!
    memcpy(&ctx->reg, &bOS0614Regs, sizeof(bOS0614Regs));
    //Why do we need to do a dummy read???
    res = readAllRegister(ctx);

    //bosDriverI2cProbe calls i2cBoreasLinuxInit
    //  i2cBoreasLinuxInit calls readDeviceTree
    //      readDeviceTree populates regToRead with default configuration
    //
    //Read SENSECONFIG to determine whether power cut occurred
    //and necessary to apply default config provided in device tree
    res = readRegister(ctx, &ctx->reg.SENSECONFIG_0614.reg);
    res = res && ctx->reg.SENSECONFIG_0614.reg.generic.value == regToRead[ADDRESS_BOS0614_SENSECONFIG_REG].value;

    ctx->reg.SENSECONFIG_0614.bit.CH0 = BOS0614_DISABLE;
    ctx->reg.SENSECONFIG_0614.bit.CH1 = BOS0614_DISABLE;
    ctx->reg.SENSECONFIG_0614.bit.CH2 = BOS0614_DISABLE;
    ctx->reg.SENSECONFIG_0614.bit.CH3 = BOS0614_DISABLE;
    ctx->reg.SENSECONFIG_0614.bit.SAME = BOS0614_DISABLE;

    res = ctx->hDriver.setRegister(&ctx->hDriver,
		ctx->reg.SENSECONFIG_0614.reg.generic.addr,
		ctx->reg.SENSECONFIG_0614.reg.generic.value);

    //This happens when SENSECONFIG register value from IC
    //won't match config from device tree
    if (res == false)
    {
        uint32_t index;
	res = true;
        for (index = 0; index < DATA_ARRAY_LENGTH(regToRead); index++)
        {
            if (regToRead[index].value != 0)
                res = res && ctx->hDriver.setRegister(&ctx->hDriver,
                            regToRead[index].addr,
                            regToRead[index].value);
        }
    }

    return res;
}

static void initiateDriver(Context *ctx)
{
    memcpy(&ctx->reg, &bOS0614Regs, sizeof(bOS0614Regs));

    ctx->hDriver.softwareReset = bos0614SoftwareReset;
    ctx->hDriver.deepSleep = bos0614DeepSleep;
    ctx->hDriver.setRegister = bos0614SetRegister;
    ctx->hDriver.getRegister = bos0614GetRegister;
    ctx->hDriver.getChipId = bos0614DriverGetChipId;
    ctx->hDriver.synthSetSlice = bos0614SetSlice;
    ctx->hDriver.synthSetWaveform = bos0614SetWaveforms;
    ctx->hDriver.wfsPlay = bos0614SynthesizerPlay;
    ctx->hDriver.ctrlOutput = bos0614CtrlOutput;
    ctx->hDriver.configGPO = bos0614ConfigGPO;
    ctx->hDriver.registerOnEvents = bos0614RegisterOnEvents;
    ctx->hDriver.unregisterEvents = bos0614UnregisterEvents;
    ctx->hDriver.setRamPlayback = bos0614SetRamPlaybackMode;
    ctx->hDriver.playRamPlayback = bos0614PlayRamPlayback;
    ctx->hDriver.setFifoMode = bos0614SetFifoMode;
    ctx->hDriver.writeFifoSamples = bos0614WriteFifo;
    ctx->hDriver.synch = bos0614Synch;
    ctx->hDriver.getError = bos0614GetError;
    ctx->hDriver.referencingFromVolt = bos0614ReferencingFromVolt;
    ctx->hDriver.writeRegister = bos0614WriteRegister;
    ctx->hDriver.readRegister = bos0614ReadRegister;
    ctx->hDriver.getShadowRegisters = bos0614GetShadowRegisters;
    ctx->hDriver.getGpioState = bos0614GetGpioState;
    ctx->hDriver.configSensing = bos0614SensingConfig;
    ctx->hDriver.sensingAutoPlayWave = bos0614SensingAutoPlayWave;
    ctx->hDriver.stopSensing = bos0614SensingStop;
    ctx->hDriver.numberOfRegister = bos0614NbrOfRegister;
    ctx->hDriver.getFifoSpace = bos0614GetFifoSpace;
    ctx->hDriver.getMaximumFifoSpace = bos0614GetMaxFifoSpace;
    ctx->hDriver.isSupported = bos0614FeatureSupport;
}
