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

#define BOS0614_REV_B_RETURNED_CHIP_ID (0x0000)
#define BOS0614_REV_B_CHIP_ID (0x000D)
#define BOS0614_REV_C_CHIP_ID_BASE (0x0604)
#define BOS0614_REV_C_CHIP_ID_DEFAULT (0x0614)
#define BOS0614_REV_C_CHIP_ID_MASK (0xFF0F)
#define BOS0614_I2C_ADDRESS (0x2c)

#define SFT_RESET_TIME_IN_US (100)
#define AMPLITUDE_MAX_VALUE_VOLT (60)
#define AMPLITUDE_MAX_VALUE (0xFFF)
#define FB_R_V_REF (68.4)

#define FREQUENCY_RESOLUTION_HZ (3.90625)

#define FIFO_MAX_AMPLITUDE (3689)

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

#define BOS0614_SENSEDATA_CONVERSION_uV (220)
#define BOS0614_SENSEDATA_VALID_RANGE (16384)

#define THRESHOLD_NBR_OF_BITS (12)
#define SLOPE_THRESHOLD_NBR_OF_BITS (7)

#define MAXIMUM_WAVEFORM_ID_FOR_AUTO_SENSING (3)

#define MIN_TC_VALUE (1)
#define MAX_TC_VALUE (31)

#define MAXIMUM_NUMBER_OF_SLICE (597)
#define MAXIMUM_WAVEFORM_ID_SEQUENCER (15)
#define WAVEFORM_METADATA_LENGTH (3)
#define MAXIMUM_WAVEFORM_NUMBER (85)

#define RAMPLAYBACK_START_RAM_ADDRESS (0x0000)

typedef enum
{
    REV_A,
    REV_B,
    REV_C
} Revision;

typedef struct
{
    uint16_t sequencerCmd[MAXIMUM_WAVEFORM_ID_SEQUENCER];
} Synthesizer;

typedef struct
{
    const Gpio* gpio;
    BOSEventCb cb;
    void* ctx;
    Bos0614GpioMode mode;
} ChannelCtx;

typedef struct
{
    Revision rev;
    bool isInitiated;
    uint8_t txBuffer[MAXIMUM_DATA_LENGTH * sizeof(uint16_t)]; // Max size in words * 2 bytes
    ChipId chipId;
    uint8_t currentChannelMask;
    const I2c* i2c;
    const Spi* spi;
    ChannelCtx channel[BOS0614_NBR_OF_CHANNEL];
    bool gpoSignalingAvailable;
    uint8_t outputChanForWaveformId[MAXIMUM_WAVEFORM_NUMBER];
    HapticDriver hDriver;
    Synthesizer synth;
    Bos0614Registers reg;
} Context;

typedef enum
{
    WSFBank_Ram = 0x1,
    WSFBank_Sequencer = 0x2,
    WSFBank_SequencerStartStop = 0x012,
    WSFBank_RamPlayback = 0x013,
    WSFBank_BurstRamWrite = 0x014
} WSFBank;

static const HapticDriver bos0614Driver;
static Context driverInstances[BOS0614_NBR_OF_INSTANCE];

/**
 * Private Section
 */
static void initiateDriver(Context *ctx);

static bool checkBos0614Revision(Context* ctx);

static bool setDefaultConfig(Context* ctx);

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
    ctx->reg.common.config0614.bit.rst = 0x1;
    res = writeReg(ctx, &ctx->reg.common.config0614.reg);
    ctx->reg.common.config0614.bit.rst = 0x0;
    if (ctx->rev == REV_C)
    {
        res = res && writeReg(ctx, &ctx->reg.common.config0614.reg);
        res = res && writeReg(ctx, &ctx->reg.common.config0614.reg);
    }

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

static Context* getContext(HapticDriver* driver)
{
    Context* ctx = NULL;

    if (driver != NULL)
    {
        size_t index;

        for (index = 0; index < DATA_ARRAY_LENGTH(driverInstances); index++)
        {
            if (&(driverInstances[index].hDriver) == driver && driverInstances[index].isInitiated)
            {
                ctx = &driverInstances[index];
                break;
            }
        }
    }

    return ctx;
}

static Bos0614Register* getRegister(Context* ctx, uint8_t addr)
{
    Bos0614Register *reg = NULL;

    switch (addr)
    {
        case ADDRESS_BOS0614_REFERENCE_REG:
            reg = &ctx->reg.common.reference0614.reg;
            break;
        case ADDRESS_BOS0614_IC_STATUS_REG:
            reg = &ctx->reg.common.icStatus0614.reg;
            break;
        case ADDRESS_BOS0614_READ_REG    :
            reg = &ctx->reg.common.read0614.reg;
            break;
        case ADDRESS_BOS0614_GPIOX_REG:
            reg = &ctx->reg.common.gpiox0614.reg;
            break;
        case ADDRESS_BOS0614_TC_REG:
            reg = &ctx->reg.common.tc0614.reg;
            break;
        case ADDRESS_BOS0614_CONFIG_REG:
            reg = &ctx->reg.common.config0614.reg;
            break;
        case ADDRESS_BOS0614_SENSECONFIG_REG:
            reg = &ctx->reg.common.senseConfig0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0_REG:
            reg = &ctx->reg.common.sense00614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0P_REG:
            reg = &ctx->reg.common.sense0P0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0R_REG:
            reg = &ctx->reg.common.sense0R0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE0S_REG:
            reg = &ctx->reg.common.sense0S0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1_REG:
            reg = &ctx->reg.common.sense10614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1P_REG:
            reg = &ctx->reg.common.sense1P0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1R_REG:
            reg = &ctx->reg.common.sense1R0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE1S_REG:
            reg = &ctx->reg.common.sense1S0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2_REG:
            reg = &ctx->reg.common.sense20614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2P_REG:
            reg = &ctx->reg.common.sense2P0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2R_REG:
            reg = &ctx->reg.common.sense2R0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE2S_REG:
            reg = &ctx->reg.common.sense2S0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3_REG:
            reg = &ctx->reg.common.sense30614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3P_REG:
            reg = &ctx->reg.common.sense3P0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3R_REG:
            reg = &ctx->reg.common.sense3R0614.reg;
            break;
        case ADDRESS_BOS0614_SENSE3S_REG:
            reg = &ctx->reg.common.sense3S0614.reg;
            break;
        case ADDRESS_BOS0614_SENSESTATUS_REG:
            reg = &ctx->reg.common.senseStatus0614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA0_REG:
            reg = &ctx->reg.common.senseData00614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA1_REG:
            reg = &ctx->reg.common.senseData10614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA2_REG:
            reg = &ctx->reg.common.senseData20614.reg;
            break;
        case ADDRESS_BOS0614_SENSEDATA3_REG:
            reg = &ctx->reg.common.senseData30614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW0_REG:
            reg = &ctx->reg.common.senseRaw00614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW1_REG:
            reg = &ctx->reg.common.senseRaw10614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW2_REG:
            reg = &ctx->reg.common.senseRaw20614.reg;
            break;
        case ADDRESS_BOS0614_SENSERAW3_REG:
            reg = &ctx->reg.common.senseRaw30614.reg;
            break;
        case ADDRESS_BOS0614_KPA_REG:
            reg = &ctx->reg.common.kpa0614.reg;
            break;
        case ADDRESS_BOS0614_KP_KI_REG:
            reg = &ctx->reg.common.kpKi0614.reg;
            break;
        case ADDRESS_BOS0614_DEADTIME_REG:
            reg = &ctx->reg.common.deadTime0614.reg;
            break;
        case ADDRESS_BOS0614_PARCAP_REG:
            reg = &ctx->reg.common.parcap0614.reg;
            break;
        case ADDRESS_BOS0614_SUP_RISE_REG:
            reg = &ctx->reg.common.supRise0614.reg;
            break;
        case ADDRESS_BOS0614_TRIM_REG:
            reg = &ctx->reg.common.trim0614.reg;
            break;
        case ADDRESS_BOS0614_CHIP_ID_REG:
            reg = &ctx->reg.common.chipId0614.reg;
            break;
        case ADDRESS_BOS0614_VFEEDBACK_REG:
            reg = &ctx->reg.common.vFeedback0614.reg;
            break;
        case ADDRESS_BOS0614_FIFO_STATE_REG:
            reg = &ctx->reg.common.fifoState0614.reg;
            break;
        case ADDRESS_BOS0614_AUTO_STATE_REG:
            reg = &ctx->reg.common.autoState0614.reg;
            break;
        case ADDRESS_BOS0614_BIST_REG:
            reg = &ctx->reg.common.bist0614.reg;
            break;
        case ADDRESS_BOS0614_BISTRES_REG:
            reg = &ctx->reg.common.bistRes0614.reg;
            break;
        case ADDRESS_BOS0614_DEBUG_REG:
            reg = &ctx->reg.common.debug0614.reg;
            break;
        case ADDRESS_BOS0614_THRESH_REG:
            reg = &ctx->reg.common.thresh0614.reg;
            break;
        case ADDRESS_BOS0614_REG38_REG:
            reg = &ctx->reg.common.reg380614.reg;
            break;
        case ADDRESS_BOS0614B_SENSE_OFFSET_REG:
        case ADDRESS_BOS0614C_SENSE_OFFSET_REG:
            reg = &ctx->reg.common.senseOffset0614.reg;
            break;
        case ADDRESS_BOS0614_CALIB_DATA_REG:
            reg = &ctx->reg.common.calibData0614.reg;
            break;
        case ADDRESS_BOS0614_REG3B_REG:
            reg = &ctx->reg.common.reg3B0614.reg;
            break;
        case ADDRESS_BOS0614_REG3C_REG:
            reg = &ctx->reg.common.reg3C0614.reg;
            break;
        case ADDRESS_BOS0614_REG3D_REG:
            reg = &ctx->reg.common.reg3D0614.reg;
            break;
        case ADDRESS_BOS0614_REG3E_REG:
            reg = &ctx->reg.common.reg3E0614.reg;
            break;
        case ADDRESS_BOS0614_REG3F_REG:
            reg = &ctx->reg.common.reg3F0614.reg;
            break;
        case ADDRESS_BOS0614_REG40_REG:
            reg = &ctx->reg.common.reg400614.reg;
            break;
        case ADDRESS_BOS0614_REG41_REG:
            reg = &ctx->reg.common.reg410614.reg;
            break;
        case ADDRESS_BOS0614_REG42_REG:
            reg = &ctx->reg.common.reg420614.reg;
            break;
        case ADDRESS_BOS0614_REG43_REG:
            reg = &ctx->reg.common.reg430614.reg;
            break;
        case ADDRESS_BOS0614B_RAM_DATA_REG:
        case ADDRESS_BOS0614C_RAM_DATA_REG:
            reg = &ctx->reg.common.ramData0614.reg;
            break;
        case ADDRESS_BOS0614_REG45_REG:
            reg = &ctx->reg.common.reg450614.reg;
            break;
        case ADDRESS_BOS0614_REG46_REG:
            reg = &ctx->reg.common.reg460614.reg;
            break;
        case ADDRESS_BOS0614_REG47_REG:
            reg = &ctx->reg.common.reg470614.reg;
            break;
        default:
            break;
    }

    return reg;
}

static bool readI2cRegister(Context *ctx, Bos0614Register *reg)
{
    bool res;
    uint8_t rxData[REG_VALUE_LENGTH] = {0};

    ctx->reg.common.read0614.bit.bc = reg->generic.addr;
    res = writeReg(ctx, &ctx->reg.common.read0614.reg);

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

    ctx->reg.common.read0614.bit.bc = reg->generic.addr;

    if (writeSpiReg(ctx, &ctx->reg.common.read0614.reg, NULL) &&
        writeSpiReg(ctx, &ctx->reg.common.read0614.reg, &reg->generic.value))
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
                                     BURST_WRITE_RAM_DATA_INDEX + (length * 2)) == ARM_DRIVER_OK;
        }
        else if (driver->spi != NULL)
        {
            res = driver->spi->send(driver->spi, driver->txBuffer, BURST_WRITE_RAM_DATA_INDEX + (length * 2)) ==
                  ARM_DRIVER_OK;
        }
    }

    return res;
}
#if 0
static void hexDump(char* message, uint8_t *data, size_t length)
{
    size_t bufferLength = length * 5 + 5;
    char *buf = kzalloc(bufferLength, GFP_KERNEL);

    if (buf != NULL)
    {
        size_t index;
        char *ptr = buf;
        char *end = buf + bufferLength;

        uint8_t *_data = (uint8_t *) data;

        for (index = 0; index < length && ptr < end; index++)
        {
            ptr += sprintf(ptr, index < (length - 1) ? "0x%02x " : "0x%02x", _data[index]);
        }

        pr_debug("%s [%s]\n", message, buf);

        kfree(buf);
    }

}
#endif
static bool writeRam(Context *driver, WSFBank bank, uint16_t *data, size_t length)
{
    int i;
    bool res = false;
#if 0
    char buffer[64];
    snprintf(buffer, sizeof(buffer) - 1, "Bank: %d, length=%zu", bank, length);
    hexDump(buffer, (uint8_t *)data,  length * 2);
#endif
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
                     SliceShape shape, uint8_t outputChannel)
{
    uint16_t buffer[SLICE_LENGTH + RAM_ADDRESS_LENGTH];
    uint32_t amplitude_computed =
            (slice->mVAmp * AMPLITUDE_MAX_VALUE) / (AMPLITUDE_MAX_VALUE_VOLT * VOLT_TO_MILLI_VOLT);
    uint8_t channel_computed = outputChannel;
    uint16_t setShape;

    uint32_t freq_computed_den = FREQUENCY_RESOLUTION_HZ * Hz_In_MilliHz;
    uint16_t freq = (uint16_t) (slice->mHzFreq / freq_computed_den);


    buffer[RAM_ADDRESS_INDEX] = RAM_SLICE_WAVEFORM_ADDR + (slice->sliceId * SLICE_LENGTH);
    buffer[SLICE_AMPLITUDE_INDEX] = (uint16_t) ((channel_computed << 12) | amplitude_computed);
    buffer[SLICE_FREQ_CYCLE_INDEX] = (uint16_t) freq;
    buffer[SLICE_FREQ_CYCLE_INDEX] |= (0xFF & slice->cycle) << SLICE_CYCLE_SHIFT;

    switch(shape) {
	case SHAPE_SAW:
            setShape = 0x0020; /* 64ms shapeUp only */
                break;
	case SHAPE_TRIANGLE:
            setShape = 0x0022; /* 64ms both ways */
                break;
	case SHAPE_SLOW_RISE:
            setShape = 0x0042; /* 128ms shapeUp and 64ms shapeDn */
                break;
	case SHAPE_SLOW_DROP:
            setShape = 0x0024; /* 64ms shapeUp and 128ms shapeDn */
                break;
	default:
            setShape = SLICE_NO_SHAPE;
		break;
    }
    buffer[SLICE_SHAPE_INDEX] = setShape;
    pr_debug("Shape = 0x%04x\n", setShape);

    return writeRam(driver, WSFBank_Ram, buffer, sizeof(buffer));
}

static bool setMode(Context *driver, Bos0614Mode mode)
{
    driver->reg.common.config0614.bit.ram = mode;

    return writeReg(driver, &driver->reg.common.config0614.reg);
}

static bool setSamplingRate(Context *ctx, uint32_t samplingRateKsps)
{
    bool res = false;
    bool found = true;
    Bos0614SamplingRate rate;

    switch (samplingRateKsps)
    {
        case 1024:
            rate = Bos0614SamplingRate_1024Ksps;
            break;
        case 512:
            rate = Bos0614SamplingRate_512Ksps;
            break;
        case 256:
            rate = Bos0614SamplingRate_256Ksps;
            break;
        case 128:
            rate = Bos0614SamplingRate_128Ksps;
            break;
        case 64:
            rate = Bos0614SamplingRate_64Ksps;
            break;
        case 32:
            rate = Bos0614SamplingRate_32Ksps;
            break;
        case 16:
            rate = Bos0614SamplingRate_16Ksps;
            break;
        case 8:
            rate = Bos0614SamplingRate_8Ksps;
            break;
        default:
            found = false;
    }

    if (found)
    {
        ctx->reg.common.config0614.bit.play = rate;
        res = writeReg(ctx, &ctx->reg.common.config0614.reg);
    }

    return res;
}

static bool activateChannel(Context *ctx, uint8_t channel)
{
    bool res = false;
    if (channel < BOS0614_NBR_OF_CHANNEL)
    {
        ctx->reg.common.reference0614.bit.fifo = 0;
        ctx->reg.common.reference0614.bit.channel = 0x1 << channel;
        res = true;
    }

    return res;
}

static bool validateIcWorking(Context *ctx)
{
    bool res = false;

    ctx->reg.common.config0614.bit.play = Bos0614SamplingRate_512Ksps;

    res = writeReg(ctx, &ctx->reg.common.config0614.reg);
    res = res && readRegister(ctx, &ctx->reg.common.config0614.reg);
    res = res && ctx->reg.common.config0614.bit.play == Bos0614SamplingRate_512Ksps;

    pr_debug("ic is%s working!!!\n", res ? "" : " NOT");

    return res;
}

static BOSEvent getEventFromGpioState(Bos0614GpioMode mode, GPIOIsr state)
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

static bool lookupGpioCtl(GPOCtrl ctrl, Bos0614GpioMode *bos0614Ctrl)
{
    bool res = false;

    switch (ctrl)
    {
        case GPO_WAVEFORM_FIFO_DONE:
            *bos0614Ctrl = Bos0614GPOCtrl_WaveformFifoDone;
            res = true;
            break;
        case GPO_SENSE_TRIGGER:
            *bos0614Ctrl = Bos0614GPOCtrl_SenseTrigger;
            res = true;
            break;
        case GPO_BUTTON_STATE:
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

    if (resetSoftware(ctx) && validateIcWorking(ctx) && setDefaultConfig(ctx))
    {
        ctx->gpoSignalingAvailable = isGpoSignalingAvailable(ctx);
        driver = &ctx->hDriver;
    }
    else
    {
        ctx->isInitiated = false;
    }

    return driver;
}

static bool getSensingAutplayReg(Context* ctx, uint8_t channel, SenseBits0614** senseConfBitField)
{
    bool res = false;

    switch (channel)
    {
        case 0:
            *senseConfBitField = &ctx->reg.common.sense00614.bit;
            res = true;
            break;
        case 1:
            *senseConfBitField = &ctx->reg.common.sense10614.bit;
            res = true;
            break;

        case 2:
            *senseConfBitField = &ctx->reg.common.sense20614.bit;
            res = true;
            break;
        case 3:
            *senseConfBitField = &ctx->reg.common.sense30614.bit;
            res = true;
            break;
        default:
            res = false;
    }

    return res;
}

static bool getSensingRegistersPerChannel(Context* ctx, uint8_t channel, SensingDirection direction,
               SenseBits0614** senseConfBitField, SenseThreshold** senseThreshold,
               SenseSBits0614** senseSlope)
{
    bool res = false;

    switch (channel)
    {
        case 0:
            *senseConfBitField = &ctx->reg.common.sense00614.bit;
            *senseSlope = &ctx->reg.common.sense0S0614.bit;

            if (direction == SENSING_DIRECTION_PRESS)
            {
                *senseThreshold = &ctx->reg.common.sense0P0614.bit;
            }
            else if (direction == SENSING_DIRECTION_RELEASE)
            {
                *senseThreshold = &ctx->reg.common.sense0R0614.bit;
            }

            res = true;
            break;
        case 1:
            *senseConfBitField = &ctx->reg.common.sense10614.bit;
            *senseSlope = &ctx->reg.common.sense1S0614.bit;

            if (direction == SENSING_DIRECTION_PRESS)
            {
                *senseThreshold = &ctx->reg.common.sense1P0614.bit;
            }
            else if (direction == SENSING_DIRECTION_RELEASE)
            {
                *senseThreshold = &ctx->reg.common.sense1R0614.bit;
            }

            res = true;
            break;

        case 2:
            *senseConfBitField = &ctx->reg.common.sense20614.bit;
            *senseSlope = &ctx->reg.common.sense2S0614.bit;

            if (direction == SENSING_DIRECTION_PRESS)
            {
                *senseThreshold = &ctx->reg.common.sense2P0614.bit;
            }
            else if (direction == SENSING_DIRECTION_RELEASE)
            {
                *senseThreshold = &ctx->reg.common.sense2R0614.bit;
            }

            res = true;
            break;
        case 3:
            *senseConfBitField = &ctx->reg.common.sense30614.bit;
            *senseSlope = &ctx->reg.common.sense3S0614.bit;

            if (direction == SENSING_DIRECTION_PRESS)
            {
                *senseThreshold = &ctx->reg.common.sense3P0614.bit;
            }
            else if (direction == SENSING_DIRECTION_RELEASE)
            {
                *senseThreshold = &ctx->reg.common.sense3R0614.bit;
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
            ctx->reg.common.senseConfig0614.bit.ch0 = activate;
            res = true;
            break;
        case 1:
            ctx->reg.common.senseConfig0614.bit.ch1 = activate;
            res = true;
            break;
        case 2:
            ctx->reg.common.senseConfig0614.bit.ch2 = activate;
            res = true;
            break;
        case 3:
            ctx->reg.common.senseConfig0614.bit.ch3 = activate;
            res = true;
            break;
        default:
            break;
    }

    if (res)
    {
        res = writeReg(ctx, &ctx->reg.common.senseConfig0614.reg);
    }

    return res;
}

bool bos0614WriteRegister(HapticDriver *driver, Register *reg);

static bool
setThresholdSensing(Context* ctx, SensingConfig config, SenseBits0614* senseBitField, SenseThreshold* senseThreshold)
{
    bool res = false;

    uint32_t threshold;
    DebouncingTime debouncingValue = DebouncingTime_1us;

    res = twoComplement(getThresholdFromMV(config.thresholdMv), THRESHOLD_NBR_OF_BITS, &threshold);
    res = res && getDebouncingConfig(config.debounceUs, &debouncingValue);

    senseThreshold->threshold = threshold;
    senseThreshold->rep = debouncingValue;

    if (config.direction == SENSING_DIRECTION_PRESS)
    {
        senseBitField->t1 = BOS0614_ENABLE;
        senseThreshold->ab = SensingThresholdMode_Above;

        res = true;
    }

    if (config.direction == SENSING_DIRECTION_RELEASE)
    {
        senseBitField->t2 = BOS0614_ENABLE;
        senseThreshold->ab = SensingThresholdMode_Below;

        res = true;
    }

    res = res && bos0614WriteRegister(&ctx->hDriver, (Register*) senseBitField);
    res = res && bos0614WriteRegister(&ctx->hDriver, (Register*) senseThreshold);

    return res;
}

static bool disableThresholdSensing(Context* ctx, SenseBits0614* senseBitField)
{
    senseBitField->t1 = BOS0614_DISABLE;
    senseBitField->t2 = BOS0614_DISABLE;

    return bos0614WriteRegister(&ctx->hDriver, (Register*) senseBitField);
}

static bool
setSlopeSensing(Context* ctx, SensingConfig config, SenseBits0614* senseBitField, SenseSBits0614* senseSlope)
{
    uint32_t slope;
    bool res = twoComplement(getSlopeThresholdFromMv(config.thresholdMv), SLOPE_THRESHOLD_NBR_OF_BITS, &slope);

    if (config.direction == SENSING_DIRECTION_PRESS)
    {
        senseBitField->s1 = BOS0614_ENABLE;
        senseSlope->slope1 = slope;
        senseSlope->abs1 = SensingThresholdMode_Above;
    }

    if (config.direction == SENSING_DIRECTION_RELEASE)
    {
        senseBitField->s2 = BOS0614_ENABLE;
        senseSlope->slope2 = slope;
        senseSlope->abs2 = SensingThresholdMode_Below;
    }

    res = res && bos0614WriteRegister(&ctx->hDriver, (Register*) senseSlope);
    res = res && bos0614WriteRegister(&ctx->hDriver, (Register*) senseBitField);

    return res;
}

static bool disableSlopeSensing(Context* ctx, SenseBits0614* senseBitField)
{
    senseBitField->s1 = BOS0614_DISABLE;
    senseBitField->s2 = BOS0614_DISABLE;

    return bos0614WriteRegister(&ctx->hDriver, (Register*) senseBitField);
}

static size_t pushErrorInQueue(BosError* errors, size_t curr, size_t maxLength, BosError error)
{
    size_t length = curr;

    if (curr < maxLength)
    {
        errors[length++] = error;
    }

    return length;
}

/**
 * Public Section
 */
HapticDriver* bos0614DriverSpiInit(const Spi* spi, const Gpio* gpioA, const Gpio* gpioD)
{
    HapticDriver* driver = NULL;
    Context* ctx = getNewInstance();

    if (spi != NULL && gpioA != NULL && gpioD != NULL && ctx != NULL)
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
    else if (ctx != NULL)
    {
        ctx->isInitiated = false;
    }

    return driver;
}


bool bos0614DriverUninit(HapticDriver* driver)
{
    bool res = false;

    if (driver != NULL)
    {
        Context* ctx = getContext(driver);

        if (ctx != NULL)
        {
            memset(ctx, 0, sizeof(Context));
            res = true;
        }
    }

    return res;
}

ChipId bos0614DriverGetChipId(HapticDriver* driver)
{
    ChipId chipId = 0;

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


        if (addr >= ADDRESS_BOS0614C_SENSE_OFFSET_REG)
        {
            ctx->reg.common.debug0614.bit.access = 0x3A;
            writeReg(ctx, &ctx->reg.common.debug0614.reg);
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

bool bos0614GetRegisters(HapticDriver* ctx, const uint8_t* addrArray, uint16_t* valueArray, const uint8_t nbRegisters)
{
    bool res = true;
    bool getRes;
    uint32_t i;

    if (ctx == NULL)
    {
        return false;
    }

    for (i = 0; i < nbRegisters; i++)
    {
        getRes = bos0614GetRegister(ctx, addrArray[i], &valueArray[i]);

        /*
         * Because of the current implementation of the read registers command send by the GUI
         * This function loop can not be interrupted if an invalid register is read and the read
         * function is returning false.
         *
         * The following code is to be sure this function returns false is on the read registers failed.
         */
        //FIXME This function shall return false if one read register failed.
        res = res ? getRes : false;
    }

    return res;
}

bool bos0614SetRegister(HapticDriver* driver, const uint8_t addr, const uint16_t value)
{
    bool res = false;

    if (driver != NULL)
    {
        Bos0614Register *reg;
        Context *ctx = container_of(driver, Context, hDriver);


        if (addr >= ADDRESS_BOS0614_BIST_REG)
        {
            ctx->reg.common.debug0614.bit.access = 0x3A;
            writeReg(ctx, &ctx->reg.common.debug0614.reg);
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

bool
bos0614SetRegisters(HapticDriver* ctx, const uint8_t* addrArray, const uint16_t* valueArray,
                    const uint8_t nbRegisters)
{
	bool res = true;
	int i;

    if (ctx == NULL)
    {
        return false;
    }

    for (i = 0; res && i < nbRegisters; i++)
    {
        res = bos0614SetRegister(ctx, addrArray[i], valueArray[i]);
    }

    return res;
}

bool bos0614WriteRegister(HapticDriver* driver, Register* reg)
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

        ctx->reg.common.config0614.bit.oe = activate ? 1 : 0; //Enable waveform playback

        res = writeReg(ctx, &ctx->reg.common.config0614.reg);
    }
    return res;
}

bool
bos0614SetSlice(HapticDriver *driver, const SynthSlice *slice, SliceShape shape, const uint8_t outputChannel)
{
    bool res = false;

    if (driver != NULL && slice != NULL && slice->sliceId < MAXIMUM_NUMBER_OF_SLICE &&
        outputChannel < BOS0614_CHANNEL_MASK)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        res = setMode(ctx, Bos0614Mode_RAM_Synthesis);
        res = res && setSlice(ctx, slice, shape, outputChannel);
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
        Context* ctx = container_of(driver, Context, hDriver);

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

static bool fixRamPlayback(HapticDriver* driver)
{
    bool res = false;

    if (driver != NULL)
    {
        Context* ctx = container_of(driver, Context, hDriver);

        uint16_t cmd[] = {RAMPLAYBACK_START_RAM_ADDRESS, RAMPLAYBACK_START_RAM_ADDRESS};

        res = writeRam(ctx, WSFBank_RamPlayback, cmd, sizeof(cmd));
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

        res = setMode(ctx, Bos0614Mode_RAM_Playback);
        res = res && setSamplingRate(ctx, samplingRate);

        //Configure the channel output
        buffer = (uint16_t *) data;
        for (index = 0; index < length; index++)
        {
            buffer[index] &= RAMPLAYBACK_DATA_MASK;
            buffer[index] |= (channelMask << RAMPLAYBACK_CHANNEL_OFFSET);
        }

        if (ctx->rev == REV_C)
        {
            res = res && fixRamPlayback(driver);
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

        res = setMode(ctx, Bos0614Mode_FIFO);
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
            if (ctx->reg.common.fifoState0614.bit.empty)
                *availableSpace = MAXIMUM_DATA_LENGTH;
            else if (ctx->reg.common.fifoState0614.bit.full)
                *availableSpace = 0;
            else
                *availableSpace = ctx->reg.common.fifoState0614.bit.fifoSpace;
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
        ctx->reg.common.reference0614.bit.fifo = dataArray[index];

        writeReg(ctx, &ctx->reg.common.reference0614.reg);
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

static void gpoIsr(const Gpio *gpio, void *context, GPIOIsr isrEvent)
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
        Bos0614GpioMode bos0614Mode;

        if (lookupGpioCtl(state, &bos0614Mode))
        {
            ctx->channel[channel].mode = bos0614Mode;

            switch (channel)
            {
                case 0:
                    ctx->reg.common.gpiox0614.bit.gpio0 = bos0614Mode;
                    res = true;
                    break;
                case 1:
                    ctx->reg.common.gpiox0614.bit.gpio1 = bos0614Mode;
                    res = true;
                    break;
                case 2:
                    ctx->reg.common.gpiox0614.bit.gpio2 = bos0614Mode;
                    res = true;
                    break;
                case 3:
                    ctx->reg.common.gpiox0614.bit.gpio3 = bos0614Mode;
                    res = true;
                    break;
                default:
                    break;
            }

            res = res && writeReg(ctx, &ctx->reg.common.gpiox0614.reg);

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
        const Gpio *gpio = ctx->channel[channelId].gpio;

        GPIOIsr isrMode = GPIOISR_Both;

        if (gpio != NULL)
        {
            ctx->channel[channelId].cb = cb;
            ctx->channel[channelId].ctx = context;
            res = gpio->registerIsr(gpio, isrMode, gpoIsr, ctx);
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
            res = channelCtx->gpio->unregisterIsr(channelCtx->gpio);
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
        driver->reg.common.config0614.bit.ds = enable ? 0x1 : 0x0;
        res = res && bos0614SetRegister(ctx, ADDRESS_BOS0614_CONFIG_REG, driver->reg.common.config0614.reg.all);
    }

    return res;
}

bool bos0614Synch(HapticDriver *driver, bool activate)
{
    (void) driver;
    (void) activate;
    return false;
}

size_t bos0614GetError(HapticDriver *driver, BosError *errors, size_t maxLength)
{
    size_t length = 0;

    if (driver != NULL && errors != NULL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        uint16_t dummyData;

        if (bos0614GetRegister(driver, ADDRESS_BOS0614_IC_STATUS_REG, &dummyData))
        {
            if (ctx->reg.common.icStatus0614.bit.sc != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOS_ERROR_SHORT_CIRCUIT);
            }

            if (ctx->reg.common.icStatus0614.bit.uvlo != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOS_ERROR_UVLO);
            }

            if (ctx->reg.common.icStatus0614.bit.idac != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOS_ERROR_IDAC);
            }

            if (ctx->reg.common.icStatus0614.bit.maxPower != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOS_ERROR_MAX_POWER);
            }

            if (ctx->reg.common.icStatus0614.bit.ovt != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOS_ERROR_OVT);
            }

            if (ctx->reg.common.icStatus0614.bit.ovv != 0)
            {
                length = pushErrorInQueue(errors, length, maxLength, BOS_ERROR_OVV);
            }
        }
    }

    return length;
}

size_t bos0614NbrOfRegister()
{
    return (BOS0614_MAXIMUM_REG_ADDR + 1);
}

bool bos0614ReferencingFromVolt(HapticDriver* ctx, int16_t* dataIn, size_t length, int16_t* dataOut)
{
    int index;
    bool res = false;

    if (ctx != NULL)
    {
        for (index = 0; index < length; index++)
        {
            float num = (float) dataIn[index] * AMPLITUDE_MAX_VALUE;
            float den = FB_R_V_REF;
            float res = num / den;

            dataOut[index] = (int16_t) (res);
        }
    }

    return res;
}


bool bos0614ReferencingFromRelativeInt16(HapticDriver* driver, int16_t* dataIn, size_t length,
                                         int16_t vMinVolt, int16_t vMaxVolt, int16_t* dataOut)
{
    bool res = false;

    if (driver != NULL)
    {
        size_t index;

        bos0614ReferencingFromVolt(driver, &vMinVolt, 1, &vMinVolt);
        bos0614ReferencingFromVolt(driver, &vMaxVolt, 1, &vMaxVolt);

        for (index = 0; index < length; index++)
        {
            //Should we divide by UINT16_MAX and use the roundf function
            int16_t value = ((dataIn[index] - INT16_MIN) * FIFO_MAX_AMPLITUDE) >> 16;
            if (value < vMinVolt)
            {
                value = vMinVolt;
            }
            else if (value > vMaxVolt)
            {
                value = vMaxVolt;
            }
            dataOut[index] = value;
        }
        res = true;
    }
    return res;
}

GPIOState bos0614GetGpioState(HapticDriver* driver, uint8_t channel)
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

bool bos0614EnableSensing(HapticDriver* driver, uint8_t channel)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL)
    {
        Context *ctx = container_of(driver, Context, hDriver);
        ChannelCtx *channelCtx = &ctx->channel[channel];

        SenseBits0614 *senseBitField;
        SenseThreshold *senseThreshold;
        SenseSBits0614 *senseSlope;

        if (getSensingRegistersPerChannel(ctx, channel, SENSING_DIRECTION_PRESS,
            &senseBitField, &senseThreshold, &senseSlope) &&
            channelCtx != NULL)
        {
            res = disableSlopeSensing(ctx, senseBitField);
            res = res && disableThresholdSensing(ctx, senseBitField);
        }

        res = res && bos0614WriteRegister(driver, (Register*) &ctx->reg.common.senseConfig0614.reg);

        res = res && activateSensingForChannel(ctx, channel, BOS0614_ENABLE);
    }

    return res;
}

#define STABILIZATION_TIME_MICRO_SECOND_PER_BIT_REV_C (3200)
#define MAXIMUM_STABILIZATION_TIME_REV_C_MS (103)

bool getStabilizationTimeRevC(uint8_t stabilizationMs, uint16_t* value)
{
    bool res = false;

    if (value != NULL && stabilizationMs < MAXIMUM_STABILIZATION_TIME_REV_C_MS)
    {
        uint16_t _value = MILLISECOND_MICROSECOND(stabilizationMs);
        *value = _value / STABILIZATION_TIME_MICRO_SECOND_PER_BIT_REV_C;
        res = true;
    }

    return res;
}

bool bos0614ButtonSensing(HapticDriver* driver, uint8_t channel, SensingConfig config)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL)
    {
        Context* ctx = container_of(driver, Context, hDriver);
        ChannelCtx* channelCtx = &ctx->channel[channel];

        SenseBits0614 *senseBitField;
        SenseThreshold *senseThreshold;
        SenseSBits0614 *senseSlope;

        if (getSensingRegistersPerChannel(ctx, channel, config.direction,
            &senseBitField, &senseThreshold, &senseSlope) &&
            channelCtx != NULL)
        {
            res = true;

            switch (config.mode)
            {
                case SENSING_DETECTION_MODE_THRESHOLD:
                    res = res && setThresholdSensing(ctx, config, senseBitField, senseThreshold);
                    break;
                case SENSING_DETECTION_MODE_SLOPE:
                    res = res && setSlopeSensing(ctx, config, senseBitField, senseSlope);
                    break;
            }

            //Setup Stabilization
            if (ctx->rev == REV_B)
            {
                ctx->reg.revB.tc0614.bit.pol = 0;
                ctx->reg.revB.tc0614.bit.pc = 0x1;
                ctx->reg.revB.tc0614.bit.tc = MAX_TC_VALUE;
                ctx->reg.revB.senseConfig0614.bit.scomp = 1;
            }
            else if (ctx->rev == REV_C)
            {
                uint16_t stabilizationMs;
                res = res && getStabilizationTimeRevC(config.stabilisationMs, &stabilizationMs);

                switch (config.direction)
                {
                    case SENSING_DIRECTION_PRESS:
                        ctx->reg.revC.tc0614.bit.tcp = stabilizationMs;
                        break;
                    case SENSING_DIRECTION_RELEASE:
                        ctx->reg.revC.tc0614.bit.tcr = stabilizationMs;
                        break;
                    default:
                        res = false;
                        break;
                }

                ctx->reg.revC.tc0614.bit.pol = 0;
                ctx->reg.revC.tc0614.bit.pc = 0x1;
                ctx->reg.revC.senseConfig0614.bit.scomp = 1;
            }

            res = res && bos0614WriteRegister(driver, (Register*) &ctx->reg.common.tc0614.reg);
            res = res && bos0614WriteRegister(driver, (Register*) &ctx->reg.common.senseConfig0614.reg);

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

        SenseBits0614 *senseBitField;
        SenseThreshold *senseThreshold;
        SenseSBits0614 *senseSlope;

        if (getSensingRegistersPerChannel(ctx, channel, direction, &senseBitField, &senseThreshold, &senseSlope))
        {
            if (direction == SENSING_DIRECTION_PRESS)
            {
                senseBitField->wvp = id;
                senseBitField->autop = BOS0614_ENABLE;
                res = true;
            }
            else if (direction == SENSING_DIRECTION_RELEASE)
            {
                senseBitField->wvr = id;
                senseBitField->autor = BOS0614_ENABLE;
                res = true;
            }

            res = res && bos0614WriteRegister(driver, (Register*) senseBitField);
        }
    }

    return res;
}

bool bos0614SensingStop(HapticDriver* driver, uint8_t channel)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        SenseBits0614 *senseBitField;

        if (getSensingAutplayReg(ctx, channel, &senseBitField))
        {
            senseBitField->autop = BOS0614_DISABLE;
            senseBitField->t1 = BOS0614_DISABLE;
            senseBitField->s1 = BOS0614_DISABLE;

            senseBitField->autor = BOS0614_DISABLE;
            senseBitField->t2 = BOS0614_DISABLE;
            senseBitField->s2 = BOS0614_DISABLE;

            res = bos0614WriteRegister(driver, (Register*) senseBitField);
            res = res && activateSensingForChannel(ctx, channel, BOS0614_DISABLE);
        }
    }

    return res;
}

bool bos0614FeatureSupport(HapticDriver *driver, BosFeature feature)
{
    bool res = false;

    if (driver != NULL && feature < BOS_FEATURE_LENGTH)
    {
        Context *ctx = container_of(driver, Context, hDriver);

        switch (feature)
        {
            case BOS_FEATURE_GPO_SIGNALING:
                if (ctx->gpoSignalingAvailable)
                {
                    res = true;
                }
                break;
            case BOS_FEATURE_FIFO:
            case BOS_FEATURE_SYNTHESIZER:
            case BOS_FEATURE_RAM_PLAYBACK:
            case BOS_FEATURE_SENSING_WITH_AUTOMATIC_FEEDBACK:
            case BOS_FEATURE_SUPPORT_BOTH_SENSING_POLARITY:
                res = true;
                break;
            default:
                res = false;
        }
    }
    return res;
}

bool bos0614SenseOutput(HapticDriver* driver, uint8_t channel, int32_t* data)
{
    bool res = false;

    if (driver != NULL && channel < BOS0614_NBR_OF_CHANNEL && data != NULL)
    {
        Context* ctx = container_of(driver, Context, hDriver);

        uint16_t tempData;

        res = ctx->hDriver.getRegister(&ctx->hDriver, ADDRESS_BOS0614_SENSEDATA0_REG + channel, &tempData);

        if (res && (int16_t) tempData < BOS0614_SENSEDATA_VALID_RANGE &&
            (int16_t) tempData >= -BOS0614_SENSEDATA_VALID_RANGE)
        {
            *data = (int32_t) ((int16_t) tempData * BOS0614_SENSEDATA_CONVERSION_uV);
        }
    }

    return res;
}

/**
 * Private Section
 */

Bos0614Registers bos0614RegsRevB = {
    .revB.reference0614.reg.generic.addr = ADDRESS_BOS0614_REFERENCE_REG, .revB.icStatus0614.reg.generic.addr = ADDRESS_BOS0614_IC_STATUS_REG, .revB.read0614.reg.generic.addr = ADDRESS_BOS0614_READ_REG, .revB.gpiox0614.reg.generic.addr = ADDRESS_BOS0614_GPIOX_REG, .revB.tc0614.reg.generic.addr = ADDRESS_BOS0614_TC_REG, .revB.config0614.reg.generic.addr = ADDRESS_BOS0614_CONFIG_REG, .revB.senseConfig0614.reg.generic.addr = ADDRESS_BOS0614_SENSECONFIG_REG, .revB.sense00614.reg.generic.addr = ADDRESS_BOS0614_SENSE0_REG, .revB.sense0P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0P_REG, .revB.sense0R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0R_REG, .revB.sense0S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0S_REG, .revB.sense10614.reg.generic.addr = ADDRESS_BOS0614_SENSE1_REG, .revB.sense1P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1P_REG, .revB.sense1R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1R_REG, .revB.sense1S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1S_REG, .revB.sense20614.reg.generic.addr = ADDRESS_BOS0614_SENSE2_REG, .revB.sense2P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2P_REG, .revB.sense2R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2R_REG, .revB.sense2S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2S_REG, .revB.sense30614.reg.generic.addr = ADDRESS_BOS0614_SENSE3_REG, .revB.sense3P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3P_REG, .revB.sense3R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3R_REG, .revB.sense3S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3S_REG, .revB.senseStatus0614.reg.generic.addr = ADDRESS_BOS0614_SENSESTATUS_REG, .revB.senseData00614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA0_REG, .revB.senseData10614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA1_REG, .revB.senseData20614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA2_REG, .revB.senseData30614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA3_REG, .revB.senseRaw00614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW0_REG, .revB.senseRaw10614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW1_REG, .revB.senseRaw20614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW2_REG, .revB.senseRaw30614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW3_REG, .revB.kpa0614.reg.generic.addr = ADDRESS_BOS0614_KPA_REG, .revB.kpKi0614.reg.generic.addr = ADDRESS_BOS0614_KP_KI_REG, .revB.deadTime0614.reg.generic.addr = ADDRESS_BOS0614_DEADTIME_REG, .revB.parcap0614.reg.generic.addr = ADDRESS_BOS0614_PARCAP_REG, .revB.supRise0614.reg.generic.addr = ADDRESS_BOS0614_SUP_RISE_REG, .revB.trim0614.reg.generic.addr = ADDRESS_BOS0614_TRIM_REG, .revB.chipId0614.reg.generic.addr = ADDRESS_BOS0614_CHIP_ID_REG, .revB.vFeedback0614.reg.generic.addr = ADDRESS_BOS0614_VFEEDBACK_REG, .revB.fifoState0614.reg.generic.addr = ADDRESS_BOS0614_FIFO_STATE_REG, .revB.autoState0614.reg.generic.addr = ADDRESS_BOS0614_AUTO_STATE_REG, .revB.bist0614.reg.generic.addr = ADDRESS_BOS0614_BIST_REG, .revB.bistRes0614.reg.generic.addr = ADDRESS_BOS0614_BISTRES_REG, .revB.debug0614.reg.generic.addr = ADDRESS_BOS0614_DEBUG_REG, .revB.thresh0614.reg.generic.addr = ADDRESS_BOS0614_THRESH_REG, .revB.reg380614.reg.generic.addr = ADDRESS_BOS0614_REG38_REG, .revB.senseOffset0614.reg.generic.addr = ADDRESS_BOS0614B_SENSE_OFFSET_REG, .revB.calibData0614.reg.generic.addr = ADDRESS_BOS0614_CALIB_DATA_REG, .revB.reg3B0614.reg.generic.addr = ADDRESS_BOS0614_REG3B_REG, .revB.reg3C0614.reg.generic.addr = ADDRESS_BOS0614_REG3C_REG, .revB.reg3D0614.reg.generic.addr = ADDRESS_BOS0614_REG3D_REG, .revB.reg3E0614.reg.generic.addr = ADDRESS_BOS0614_REG3E_REG, .revB.reg3F0614.reg.generic.addr = ADDRESS_BOS0614_REG3F_REG, .revB.reg400614.reg.generic.addr = ADDRESS_BOS0614_REG40_REG, .revB.reg410614.reg.generic.addr = ADDRESS_BOS0614_REG41_REG, .revB.reg420614.reg.generic.addr = ADDRESS_BOS0614_REG42_REG, .revB.reg430614.reg.generic.addr = ADDRESS_BOS0614_REG43_REG, .revB.ramData0614.reg.generic.addr = ADDRESS_BOS0614B_RAM_DATA_REG, .revB.reg450614.reg.generic.addr = ADDRESS_BOS0614_REG45_REG, .revB.reg460614.reg.generic.addr = ADDRESS_BOS0614_REG46_REG, .revB.reg470614.reg.generic.addr = ADDRESS_BOS0614_REG47_REG,
};

Bos0614Registers bos0614RegsRevC = {
    .revC.reference0614.reg.generic.addr = ADDRESS_BOS0614_REFERENCE_REG, .revC.icStatus0614.reg.generic.addr = ADDRESS_BOS0614_IC_STATUS_REG, .revC.read0614.reg.generic.addr = ADDRESS_BOS0614_READ_REG, .revC.gpiox0614.reg.generic.addr = ADDRESS_BOS0614_GPIOX_REG, .revC.tc0614.reg.generic.addr = ADDRESS_BOS0614_TC_REG, .revC.config0614.reg.generic.addr = ADDRESS_BOS0614_CONFIG_REG, .revC.senseConfig0614.reg.generic.addr = ADDRESS_BOS0614_SENSECONFIG_REG, .revC.sense00614.reg.generic.addr = ADDRESS_BOS0614_SENSE0_REG, .revC.sense0P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0P_REG, .revC.sense0R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0R_REG, .revC.sense0S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE0S_REG, .revC.sense10614.reg.generic.addr = ADDRESS_BOS0614_SENSE1_REG, .revC.sense1P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1P_REG, .revC.sense1R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1R_REG, .revC.sense1S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE1S_REG, .revC.sense20614.reg.generic.addr = ADDRESS_BOS0614_SENSE2_REG, .revC.sense2P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2P_REG, .revC.sense2R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2R_REG, .revC.sense2S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE2S_REG, .revC.sense30614.reg.generic.addr = ADDRESS_BOS0614_SENSE3_REG, .revC.sense3P0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3P_REG, .revC.sense3R0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3R_REG, .revC.sense3S0614.reg.generic.addr = ADDRESS_BOS0614_SENSE3S_REG, .revC.senseStatus0614.reg.generic.addr = ADDRESS_BOS0614_SENSESTATUS_REG, .revC.senseData00614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA0_REG, .revC.senseData10614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA1_REG, .revC.senseData20614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA2_REG, .revC.senseData30614.reg.generic.addr = ADDRESS_BOS0614_SENSEDATA3_REG, .revC.senseRaw00614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW0_REG, .revC.senseRaw10614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW1_REG, .revC.senseRaw20614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW2_REG, .revC.senseRaw30614.reg.generic.addr = ADDRESS_BOS0614_SENSERAW3_REG, .revC.kpa0614.reg.generic.addr = ADDRESS_BOS0614_KPA_REG, .revC.kpKi0614.reg.generic.addr = ADDRESS_BOS0614_KP_KI_REG, .revC.deadTime0614.reg.generic.addr = ADDRESS_BOS0614_DEADTIME_REG, .revC.parcap0614.reg.generic.addr = ADDRESS_BOS0614_PARCAP_REG, .revC.supRise0614.reg.generic.addr = ADDRESS_BOS0614_SUP_RISE_REG, .revC.trim0614.reg.generic.addr = ADDRESS_BOS0614_TRIM_REG, .revC.chipId0614.reg.generic.addr = ADDRESS_BOS0614_CHIP_ID_REG, .revC.vFeedback0614.reg.generic.addr = ADDRESS_BOS0614_VFEEDBACK_REG, .revC.fifoState0614.reg.generic.addr = ADDRESS_BOS0614_FIFO_STATE_REG, .revC.autoState0614.reg.generic.addr = ADDRESS_BOS0614_AUTO_STATE_REG, .revC.bist0614.reg.generic.addr = ADDRESS_BOS0614_BIST_REG, .revC.bistRes0614.reg.generic.addr = ADDRESS_BOS0614_BISTRES_REG, .revC.debug0614.reg.generic.addr = ADDRESS_BOS0614_DEBUG_REG, .revC.thresh0614.reg.generic.addr = ADDRESS_BOS0614_THRESH_REG, .revC.reg380614.reg.generic.addr = ADDRESS_BOS0614_REG38_REG, .revC.senseOffset0614.reg.generic.addr = ADDRESS_BOS0614C_SENSE_OFFSET_REG, .revC.calibData0614.reg.generic.addr = ADDRESS_BOS0614_CALIB_DATA_REG, .revC.reg3B0614.reg.generic.addr = ADDRESS_BOS0614_REG3B_REG, .revC.reg3C0614.reg.generic.addr = ADDRESS_BOS0614_REG3C_REG, .revC.reg3D0614.reg.generic.addr = ADDRESS_BOS0614_REG3D_REG, .revC.reg3E0614.reg.generic.addr = ADDRESS_BOS0614_REG3E_REG, .revC.reg3F0614.reg.generic.addr = ADDRESS_BOS0614_REG3F_REG, .revC.reg400614.reg.generic.addr = ADDRESS_BOS0614_REG40_REG, .revC.reg410614.reg.generic.addr = ADDRESS_BOS0614_REG41_REG, .revC.reg420614.reg.generic.addr = ADDRESS_BOS0614_REG42_REG, .revC.reg430614.reg.generic.addr = ADDRESS_BOS0614_REG43_REG, .revC.ramData0614.reg.generic.addr = ADDRESS_BOS0614C_RAM_DATA_REG, .revC.reg450614.reg.generic.addr = ADDRESS_BOS0614_REG45_REG, .revC.reg460614.reg.generic.addr = ADDRESS_BOS0614_REG46_REG, .revC.reg470614.reg.generic.addr = ADDRESS_BOS0614_REG47_REG,
};

static uint8_t regAddrToReadRevB[] = {
    ADDRESS_BOS0614_REFERENCE_REG,
    ADDRESS_BOS0614_IC_STATUS_REG,
    ADDRESS_BOS0614_READ_REG,
    ADDRESS_BOS0614_GPIOX_REG,
    ADDRESS_BOS0614_TC_REG,
    ADDRESS_BOS0614_CONFIG_REG,
    ADDRESS_BOS0614_SENSECONFIG_REG,
    ADDRESS_BOS0614_SENSE0_REG,
    ADDRESS_BOS0614_SENSE0P_REG,
    ADDRESS_BOS0614_SENSE0R_REG,
    ADDRESS_BOS0614_SENSE0S_REG,
    ADDRESS_BOS0614_SENSE1_REG,
    ADDRESS_BOS0614_SENSE1P_REG,
    ADDRESS_BOS0614_SENSE1R_REG,
    ADDRESS_BOS0614_SENSE1S_REG,
    ADDRESS_BOS0614_SENSE2_REG,
    ADDRESS_BOS0614_SENSE2P_REG,
    ADDRESS_BOS0614_SENSE2R_REG,
    ADDRESS_BOS0614_SENSE2S_REG,
    ADDRESS_BOS0614_SENSE3_REG,
    ADDRESS_BOS0614_SENSE3P_REG,
    ADDRESS_BOS0614_SENSE3R_REG,
    ADDRESS_BOS0614_SENSE3S_REG,
    ADDRESS_BOS0614_SENSESTATUS_REG,
    ADDRESS_BOS0614_SENSEDATA0_REG,
    ADDRESS_BOS0614_SENSEDATA1_REG,
    ADDRESS_BOS0614_SENSEDATA2_REG,
    ADDRESS_BOS0614_SENSEDATA3_REG,
    ADDRESS_BOS0614_SENSERAW0_REG,
    ADDRESS_BOS0614_SENSERAW1_REG,
    ADDRESS_BOS0614_SENSERAW2_REG,
    ADDRESS_BOS0614_SENSERAW3_REG,
    ADDRESS_BOS0614_KPA_REG,
    ADDRESS_BOS0614_KP_KI_REG,
    ADDRESS_BOS0614_DEADTIME_REG,
    ADDRESS_BOS0614_PARCAP_REG,
    ADDRESS_BOS0614_SUP_RISE_REG,
    ADDRESS_BOS0614_TRIM_REG,
    ADDRESS_BOS0614_CHIP_ID_REG,
    ADDRESS_BOS0614_VFEEDBACK_REG,
    ADDRESS_BOS0614_FIFO_STATE_REG,
    ADDRESS_BOS0614_AUTO_STATE_REG,
    ADDRESS_BOS0614_BIST_REG,
    ADDRESS_BOS0614_BISTRES_REG,
    ADDRESS_BOS0614_DEBUG_REG,
    ADDRESS_BOS0614_THRESH_REG,
    ADDRESS_BOS0614_REG38_REG,
    ADDRESS_BOS0614B_SENSE_OFFSET_REG,
    ADDRESS_BOS0614_CALIB_DATA_REG,
    ADDRESS_BOS0614_REG3B_REG,
    ADDRESS_BOS0614_REG3C_REG,
    ADDRESS_BOS0614_REG3D_REG,
    ADDRESS_BOS0614_REG3E_REG,
    ADDRESS_BOS0614_REG3F_REG,
    ADDRESS_BOS0614_REG40_REG,
    ADDRESS_BOS0614_REG41_REG,
    ADDRESS_BOS0614_REG42_REG,
    ADDRESS_BOS0614_REG43_REG,
    ADDRESS_BOS0614B_RAM_DATA_REG,
    ADDRESS_BOS0614_REG45_REG,
    ADDRESS_BOS0614_REG46_REG,
    ADDRESS_BOS0614_REG47_REG
};

static uint8_t regAddrToReadRevC[] = {
    ADDRESS_BOS0614_REFERENCE_REG,
    ADDRESS_BOS0614_IC_STATUS_REG,
    ADDRESS_BOS0614_READ_REG,
    ADDRESS_BOS0614_GPIOX_REG,
    ADDRESS_BOS0614_TC_REG,
    ADDRESS_BOS0614_CONFIG_REG,
    ADDRESS_BOS0614_SENSECONFIG_REG,
    ADDRESS_BOS0614_SENSE0_REG,
    ADDRESS_BOS0614_SENSE0P_REG,
    ADDRESS_BOS0614_SENSE0R_REG,
    ADDRESS_BOS0614_SENSE0S_REG,
    ADDRESS_BOS0614_SENSE1_REG,
    ADDRESS_BOS0614_SENSE1P_REG,
    ADDRESS_BOS0614_SENSE1R_REG,
    ADDRESS_BOS0614_SENSE1S_REG,
    ADDRESS_BOS0614_SENSE2_REG,
    ADDRESS_BOS0614_SENSE2P_REG,
    ADDRESS_BOS0614_SENSE2R_REG,
    ADDRESS_BOS0614_SENSE2S_REG,
    ADDRESS_BOS0614_SENSE3_REG,
    ADDRESS_BOS0614_SENSE3P_REG,
    ADDRESS_BOS0614_SENSE3R_REG,
    ADDRESS_BOS0614_SENSE3S_REG,
    ADDRESS_BOS0614_SENSESTATUS_REG,
    ADDRESS_BOS0614_SENSEDATA0_REG,
    ADDRESS_BOS0614_SENSEDATA1_REG,
    ADDRESS_BOS0614_SENSEDATA2_REG,
    ADDRESS_BOS0614_SENSEDATA3_REG,
    ADDRESS_BOS0614_SENSERAW0_REG,
    ADDRESS_BOS0614_SENSERAW1_REG,
    ADDRESS_BOS0614_SENSERAW2_REG,
    ADDRESS_BOS0614_SENSERAW3_REG,
    ADDRESS_BOS0614_KPA_REG,
    ADDRESS_BOS0614_KP_KI_REG,
    ADDRESS_BOS0614_DEADTIME_REG,
    ADDRESS_BOS0614_PARCAP_REG,
    ADDRESS_BOS0614_SUP_RISE_REG,
    ADDRESS_BOS0614_TRIM_REG,
    ADDRESS_BOS0614_CHIP_ID_REG,
    ADDRESS_BOS0614_VFEEDBACK_REG,
    ADDRESS_BOS0614_FIFO_STATE_REG,
    ADDRESS_BOS0614_AUTO_STATE_REG,
    ADDRESS_BOS0614_BIST_REG,
    ADDRESS_BOS0614_BISTRES_REG,
    ADDRESS_BOS0614_DEBUG_REG,
    ADDRESS_BOS0614_THRESH_REG,
    ADDRESS_BOS0614_REG38_REG,
    ADDRESS_BOS0614C_SENSE_OFFSET_REG,
    ADDRESS_BOS0614_CALIB_DATA_REG,
    ADDRESS_BOS0614_REG3B_REG,
    ADDRESS_BOS0614_REG3C_REG,
    ADDRESS_BOS0614_REG3D_REG,
    ADDRESS_BOS0614_REG3E_REG,
    ADDRESS_BOS0614_REG3F_REG,
    ADDRESS_BOS0614_REG40_REG,
    ADDRESS_BOS0614_REG41_REG,
    ADDRESS_BOS0614_REG42_REG,
    ADDRESS_BOS0614_REG43_REG,
    ADDRESS_BOS0614C_RAM_DATA_REG,
    ADDRESS_BOS0614_REG45_REG,
    ADDRESS_BOS0614_REG46_REG,
    ADDRESS_BOS0614_REG47_REG
};
#if 0
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
#endif

static bool readAllRegister(Context *ctx)
{
    uint32_t index;
    bool res = true;

    for (index = 0; index < DATA_ARRAY_LENGTH(regAddrToReadRevB) && res; index++)
    {
        uint16_t dummy;

        if (ctx->rev == REV_B)
        {
            res = bos0614GetRegister(&ctx->hDriver, regAddrToReadRevB[index], &dummy);
        }
        else if (ctx->rev == REV_C)
        {
            res = bos0614GetRegister(&ctx->hDriver, regAddrToReadRevC[index], &dummy);
        }
    }

    return res;
}

static bool setDefaultConfig(Context *ctx)
{
    bool res = false;

    //Reset configuration of the register;
    if (ctx->rev == REV_B)
    {
        memcpy(&ctx->reg, &bos0614RegsRevB, sizeof(bos0614RegsRevB));
    }
    else if (ctx->rev == REV_C)
    {
        memcpy(&ctx->reg, &bos0614RegsRevC, sizeof(bos0614RegsRevC));
    }

    //Why do we need to do a dummy read???
    res = readAllRegister(ctx);

    ctx->reg.common.senseConfig0614.bit.ch0 = BOS0614_DISABLE;
    ctx->reg.common.senseConfig0614.bit.ch1 = BOS0614_DISABLE;
    ctx->reg.common.senseConfig0614.bit.ch2 = BOS0614_DISABLE;
    ctx->reg.common.senseConfig0614.bit.ch3 = BOS0614_DISABLE;
    ctx->reg.common.senseConfig0614.bit.same = BOS0614_DISABLE;

    return res;
}

static bool checkBos0614Revision(Context* ctx)
{
    bool res = false;

    if (&(ctx->hDriver) != NULL)
    {
        res = bos0614GetRegister(&ctx->hDriver, ADDRESS_BOS0614_CHIP_ID_REG, &ctx->chipId);
        if ((ctx->chipId & BOS0614_REV_C_CHIP_ID_MASK) == BOS0614_REV_C_CHIP_ID_BASE && res)
        {
            ctx->rev = REV_C;
            ctx->chipId = BOS0614_REV_C_CHIP_ID_DEFAULT;
        }
        else if (ctx->chipId == BOS0614_REV_B_RETURNED_CHIP_ID && res)
        {
            ctx->rev = REV_B;
            ctx->chipId = BOS0614_REV_B_CHIP_ID;
        }
        else
        {
            ctx->chipId = INVALID_CHIP_ID;
            res = false;
        }
    }

    return res;
}

static void initiateDriver(Context *ctx)
{
    bool res = false;

    memcpy(&ctx->hDriver, &bos0614Driver, sizeof(bos0614Driver));
    memcpy(&ctx->reg, &bos0614RegsRevB, sizeof(Bos0614Registers));

    res = checkBos0614Revision(ctx);

    if (ctx->chipId == BOS0614_REV_C_CHIP_ID_DEFAULT)
    {
        memcpy(&ctx->reg, &bos0614RegsRevC, sizeof(Bos0614Registers));
    }
}

static const HapticDriver bos0614Driver = {
    bos0614DriverGetChipId,
    bos0614SoftwareReset,
    bos0614DeepSleep,

    bos0614GetRegister,
    bos0614GetRegisters,

    bos0614SetRegister,
    bos0614SetRegisters,

    NULL,   //BosSetGetRegister setGetRegister;
    NULL,   //BosSetGetRegisters setGetRegisters;

    bos0614SetSlice,
    bos0614SetWaveforms,
    bos0614SynthesizerPlay,

    bos0614RegisterOnEvents,
    bos0614UnregisterEvents,

    bos0614CtrlOutput,
    bos0614ConfigGPO,

    bos0614SetRamPlaybackMode,
    bos0614PlayRamPlayback,

    bos0614GetFifoSpace,
    bos0614GetMaxFifoSpace,
    bos0614SetFifoMode,
    bos0614WriteFifo,

    bos0614Synch,
    bos0614GetError,
    bos0614ReferencingFromVolt,
    bos0614ReferencingFromRelativeInt16,
    bos0614NbrOfRegister,

    bos0614ButtonSensing,
    bos0614EnableSensing,
    bos0614SensingAutoPlayWave,
    bos0614SensingStop,

    bos0614FeatureSupport,
    bos0614SenseOutput,
    NULL,

    /* Engineering functions */
    bos0614GetShadowRegisters,

    bos0614WriteRegister,
    bos0614ReadRegister,

    bos0614GetGpioState
};
