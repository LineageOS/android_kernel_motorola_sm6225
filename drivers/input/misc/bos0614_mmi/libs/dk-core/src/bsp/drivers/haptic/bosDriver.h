//
// Description: Boreas Generic Haptic Driver Interface
// Created on 2/18/2020
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

#ifndef DKCORE_BOSDRIVER_H
#define DKCORE_BOSDRIVER_H

#include <linux/types.h>

#include "bsp/drivers/spi/spi.h"
#include "bsp/drivers/gpio/gpio.h"

#include "bsp/drivers/haptic/bos0614Register.h"

#define INVALID_CHIP_ID (0xFF)
#define HAPTIC_DRIVER_DEFAULT_CHANNEL (0)

typedef uint8_t SliceId;
typedef uint8_t WaveformId;

typedef union
{
    Bos0614Register register0614;
} Register;

typedef union
{
    BOS0614_REGS bos0614Regs;
} Registers;

typedef enum
{
    GPO_InternalReset = 0,
    GPO_SenseTrigger,
    GPO_WaveformFifoDone,
    GPO_ErrorNotify,
    GPO_MaxPower,
    GPO_ButtonState,
    GPO_RequestToPLay,
} GPOCtrl;

typedef enum
{
    SensingDirection_Press = 0x00,
    SensingDirection_Release = 0x01,
    SensingDirection_Both
} SensingDirection;

typedef enum
{
    SensingDetectionMode_Threshold = 0x0,
    SensingDetectionMode_Slope = 0x1,
} SensingDetectionMode;

typedef struct
{
    uint16_t debounceUs;
    int16_t thresholdMv;
    SensingDirection direction;
    SensingDetectionMode mode;
    uint8_t stabilisationMs;
} SensingConfig;

typedef enum
{
    BosFeature_Fifo,
    BosFeature_Synthesizer,
    BosFeature_Ramplayback,
    BosFeature_SensingWithAutomaticFeedback,
    BosFeature_SensingThresholdDetection,
    BosFeature_GPOSignaling,

    BosFeature_Length
} BosFeature;


typedef struct
{
    uint32_t sliceId;
    int32_t mVAmp;     //milli volt amplitude
    uint32_t mHzFreq;   //milli-Hertz
    uint32_t cycle;
} SynthSlice;

typedef enum
{
    BOSERROR_NO_ERROR = 0,
    BOSERROR_SHORT_CIRCUIT,
    BOSERROR_UVLO,
    BOSERROR_IDAC,
    BOSERROR_MAX_POWER,
    BOSERROR_OVT,
    BOSERROR_OVV,
    BOSERROR_Length,
} BOSError;

typedef enum
{
    BOS_EVENT_WAVEFORM_DONE,
    BOS_EVENT_BUTTON_PRESS,
    BOS_EVENT_BUTTON_RELEASE,
    BOS_EVENT_SENSING_TRIGGER,
    BOS_EVENT_REQUEST_TO_PLAY,
    BOS_EVENT_NO_REQUEST_TO_PLAY,
    BOS_EVENT_NO_EVENT
} BOSEvent;

typedef uint8_t ChannelId;

typedef struct _HapticDriver HapticDriver;

/*
 * @brief Software Reset the chip
 *
 * @param[in] driver           The haptic driver context
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosSoftwareReset)(HapticDriver *driver);

/*
 * @brief Put device to sleep or wake it up
 * @param[in] driver        The haptic driver context
 * @param[in] toSleep    true for sleep, false for wakeup
 * @return True if the operation succeeded, otherwise false
 */
typedef bool(*BosDeepSleep)(HapticDriver *driver, bool toSleep);

/**
 * @brief Return the chip id
 *
 * @param[in]   driver  The haptic driver context
 *
 * @return uint8_t Return the chip id
 */
typedef uint8_t (*BosGetChipId)(HapticDriver *driver);

/*
 * @brief Return the register value for a specific address
 *
 * @param[in]   driver haptic driver context
 * @param[in]   addr  The register address
 * @param[out]  value The pointer where to pu the read value
 *
 * @return bool True if the operation succeed, otherwise false
 */
typedef bool(*BosGetRegister)(HapticDriver *driver, uint8_t addr, uint16_t *value);

/*
 * @brief Set the specified register
 *
 * @param[in] driver   The haptic driver context
 * @param[in] addr  The register address
 * @param[in] value The register value to set
 *
 * @return bool
 */
typedef bool(*BosSetRegister)(HapticDriver *driver, uint8_t addr, uint16_t value);

/*
 * @brief Get the number of register available
 */
typedef size_t (*BosGetNumberOfRegister)(void);

/*
 * @brief Configure a slice which is part of a waveform (group of slice)
 *
 * @param[in] driver        The haptic driver context
 * @param[in] slice         Slice configuration
 * @param[in] outputChannel The output channel of the slice
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (*BosSynthesizerSetSlice)(HapticDriver *driver, const SynthSlice *slice, const uint8_t outputChannel);

/**
 * @brief Set the synthesizer mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] waveformId    The waveform id to configure
 * @param[in] frequency     The frequency of the specified waveform
 * @param[in] amplitude     The amplitude of the specified waveform
 * @param[in] cycle         The amount of waveform period to play
 * @param[in] outputChannel The output channel to play the waveform
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosSynthesizerSetWaveform)(HapticDriver *driver, WaveformId id, uint8_t startSliceId,
                                         size_t nbrOfSlices, uint16_t cycle, uint8_t outputChannel);


/**
 * @brief Activate or deactivate the output
 *
 * @param[in] driver           The haptic driver context
 * @param[in] activate      True if activating the output, False if deactivating the output
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosCtrlOutput)(HapticDriver *driver, bool activate);

/**
 * @brief Configure the GPO mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] channel       The channel specific to this IC
 * @param[in] state         The GPO mode to configure
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BOSCtrlGPO)(HapticDriver *driver, ChannelId channelId, GPOCtrl state);

/**
 * @brief BOS Driver GPO Event Callback
 *
 * @param[in] driver            The haptic driver instance
 * @param[in] channelId         The channel specific to the event occurrence
 * @param[in] event             The pushed event
 * @param[in] context           The client context specified on registering the callback
 *
 * @return void
 */
typedef void(*BOSEventCb)(HapticDriver *driver, ChannelId channelId, BOSEvent event, void *context);

/**
 * @brief Register for event callback registration
 *
 * @param[in] driver            The haptic driver instance
 * @param[in] channelId         The channel specific to listen for event
 * @param[in] cb                Callback to register
 * @param[in] context           Context to forward to the callback
 *
 * @return True if the operation succeed, false otherwise
 */
typedef bool(*BOSRegisterEvents)(HapticDriver *driver, ChannelId channelId, BOSEventCb cb, void *context);

/*
 * @brief Remove from event callback registration for GPO signals
 *
 * @param[in] driver            The haptic driver instance
 * @param[in] channelId         The channel specific to unregister for event
 *
 * @return True if the operation succeed, false otherwise
 */
typedef bool(*BOSUnregisterEvents)(HapticDriver *driver, ChannelId channelId);

/*
 * @brief Toggle the synthesizer play
 *
 * @param[in] driver        The haptic driver context
 * @param[in] start         The waveform id where the sequencer starts to play
 * @param[in] stop          The waveform id where the sequencer stops to play
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosWaveSynthesizerPlay)(HapticDriver *driver, WaveformId start, WaveformId stop);

/**
 * @brief Configure the chip in RAM playback mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] samplingRate  The number of kilo sample per second to play
 * @param[in] data          The data to write in the ram
 * @param[in] length        The length of the data to play
 * @param[in] channelMask   The bit mask of the channel configuration where each bit represents a channel to activate.
 * LSB represent the channel #0 and MSB the channel #8
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosRamplayback)(HapticDriver *driver, uint32_t samplingRate, void *data, size_t length,
                              uint8_t channelMask);

/**
 * @brief Play the content of the ram playback
 *
 * @param[in] driver        The haptic driver context
 * @param[in] length        The length of the data to play
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosRamPlaybackToggle)(HapticDriver *driver, size_t length);


/**
 * @brief Configure the chip in Fifo mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] samplingRate  The number of kilo sample per second to play
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosSetFifoMode)(HapticDriver *driver, uint32_t samplingRate, uint8_t outputChannel);

/**
 * @brief Write data in the fifo
 *
 * @param[in] driver        The haptic driver context
 * @param[in] data          The data to write in the fifo
 * @param[in] length        The length of the data to write
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosWriteFifoSamples)(HapticDriver *driver, void *data, size_t length);

/**
 * @brief Get the fifo space avaiable in the fifo
 *
 * @param[in] driver            The haptic driver context
 * @param[in] availableSpace    The available space in the fifo
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosGetFifoSpace)(HapticDriver *ctx, uint16_t *availableSpace);

/**
 * @brief Get the FIFO max available space
 *
 * @return uint16_t The max space available
 */
typedef uint16_t (*BosGetMaximumFifoSpace)(void);

/**
 * @brief Activate or deactivate the synchronization mode between chip
 *
 * @param[in] driver        The haptic driver context
 * @param[in] activate      True for activating and False for deactivating
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosSynch)(HapticDriver *driver, bool activate);

/**
 * @brief Referencing the voltage value passed in the array to values referencing in the chip domain
 *
 * @param[in]       driver  The haptic driver context
 * @param[in|out]   data    The data to reference
 * @param[in]       length  The length of the array to reference
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(*BosReferencingFromVolt)(HapticDriver *driver, int16_t *data, size_t length);

/*
 * @brief Get the error status bits
 *
 * @param[in] driver        The haptic driver context
 * @param[in] errors        Provided error array
 * @param[in] maxLength     Max number of error to copy in the errors array
 *
 * @return The amount of active errors
 */
typedef size_t(*BosGetError)(HapticDriver *driver, BOSError *errors, size_t maxLength);

/*
 * @brief Sensing Event Callback
 *
 * @param[in] channel       The IC channel from which the event has been detected
 * @param[in] mode          The sensing detection mode
 * @param[in] direction     The sensing threshold direction detection
 * @param[in] arg           Application Specific arg copied when registering the callback
 *
 * @return void
 */
typedef void (*BosSensingEvent)(uint8_t channel, SensingDetectionMode mode, SensingDirection direction, void *arg);

/*
 * @brief Configures the chip for sensing
 *
 * @param[in] driver        The haptic driver concerned
 * @param[in] channel       The channel to configure the sensing for.
 * @param[in] config        Sensing configuration to apply
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (*BosSensingConfig)(HapticDriver *driver, ChannelId channelId, SensingConfig config);

/*
 * @brief Set the automatic feedback by channel
 *
 * @param[in] driver        The haptic driver concerned
 * @param[in] channelId     The channel driver which applies
 * @param[in] id            The synthesizer waveform id to play on detection event
 * @param[in] direction     The detection direction (press or release) which the waveform must be applied to.
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (*BosSensingAutoPlayWave)(HapticDriver *driver, ChannelId channelId, WaveformId id,
                                       SensingDirection direction);

/*
 * @brief Stops sensing without disabling the output
 *
 * @param[in] driver    The haptic driver context
 * @param[in] channel   The channel to deactivate the sensing
 * @param[in] direction The sensing direction (press or release) to deactivate
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (*BosSensingStop)(HapticDriver *driver, ChannelId channelId, SensingDirection direction);

/*
 * @brief (Engineering feature) Access register shadow value of the driver
 *
 * @return Registers The register set corresponding to the IC
 */
typedef Registers *(*BosGetShadowRegisters)(HapticDriver *driver);

/*
 * @brief (Engineering feature) Set a specific register using the Register struct
 *
 * @param[in] driver        The haptic driver context
 * @param[in] reg           The register to set
 *
 * @return bool             True if the operation succeed and false if the operation failed
 */
typedef bool(*BosWriteRegister)(HapticDriver *driver, Register *reg);


/*
 * @brief (Engineering feature) Set a specific register using the Register struct
 *
 * @param[in] driver        The haptic driver context
 * @param[in|out] reg       The register to read. If the operation succeed, the value of the register will be updated.
 *
 * @return bool             True if the operation succeed and false if the operation failed
 */
typedef bool(*BosReadRegister)(HapticDriver *driver, Register *reg);

/*
 * @brief (Engineering feature) Read the GPIO pin state
 *
 * @param[in] driver        The haptic driver context
 * @param[in] channel       The channel specific to this IC
 *
 * @return GPIOState        The state of the Gpio
 */
typedef GPIOState(*BosGetGpioState)(HapticDriver *driver, ChannelId channelId);

/**
 * @brief Interrogate driver if a specific feature is supported
 *
 * @param[in] feature       The feature request
 *
 * @return bool             True if the feature is supported, false if not
 */
typedef bool (*SupportedFeature)(HapticDriver *driver, BosFeature feature);

struct _HapticDriver
{
    BosGetChipId getChipId;
    BosSoftwareReset softwareReset;
    BosDeepSleep deepSleep;
    BosGetRegister getRegister;
    BosSetRegister setRegister;
    BosSynthesizerSetSlice synthSetSlice;
    BosSynthesizerSetWaveform synthSetWaveform;
    BosWaveSynthesizerPlay wfsPlay;
    BOSRegisterEvents registerOnEvents;
    BOSUnregisterEvents unregisterEvents;
    BosCtrlOutput ctrlOutput;
    BOSCtrlGPO configGPO;
    BosRamplayback setRamPlayback;
    BosRamPlaybackToggle playRamPlayback;
    BosGetFifoSpace getFifoSpace;
    BosGetMaximumFifoSpace getMaximumFifoSpace;
    BosSetFifoMode setFifoMode;
    BosWriteFifoSamples writeFifoSamples;
    BosSynch synch;
    BosGetError getError;
    BosReferencingFromVolt referencingFromVolt;
    BosGetNumberOfRegister numberOfRegister;
    BosSensingConfig configSensing;
    BosSensingAutoPlayWave sensingAutoPlayWave;
    BosSensingStop stopSensing;
    SupportedFeature isSupported;

    /* Engineering functions */
    BosGetShadowRegisters getShadowRegisters;
    BosWriteRegister writeRegister;
    BosReadRegister readRegister;
    BosGetGpioState getGpioState;
};

#endif //DKCORE_BOSDRIVER_H
