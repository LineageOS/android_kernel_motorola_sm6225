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
#include "bsp/drivers/haptic/bos1211Register.h"
#include "bsp/drivers/haptic/bos1901Register.h"

#define INT16_MIN (-32768)
#define UINT16_MAX (65535)
#define INVALID_CHIP_ID (UINT16_MAX)
#define HAPTIC_DRIVER_DEFAULT_CHANNEL (0)

typedef uint16_t ChipId;
typedef uint8_t SliceId;
typedef uint8_t WaveformId;

typedef union
{
    Bos0614Register register0614;
    Bos1211Register register1211;
    Bos1901Register register1901;
} Register;

typedef union
{
    Bos0614Registers bos0614Regs;
    BOS1211_REGS bos1211Regs;
    BOS1901_REGS bos1901Regs;
} Registers;

typedef enum
{
    GPO_INTERNAL_RESET = 0,
    GPO_SENSE_TRIGGER,
    GPO_WAVEFORM_FIFO_DONE,
    GPO_ERROR_NOTIFY,
    GPO_MAX_POWER,
    GPO_BUTTON_STATE,
    GPO_REQUEST_TO_PLAY,
} GPOCtrl;

typedef enum
{
    SENSING_DIRECTION_PRESS = 0x00,
    SENSING_DIRECTION_RELEASE = 0x01,
    SENSING_DIRECTION_ENUM_LENGTH
} SensingDirection;

typedef enum
{
    SENSING_DETECTION_MODE_THRESHOLD = 0x0,
    SENSING_DETECTION_MODE_SLOPE = 0x1,
} SensingDetectionMode;

typedef enum
{
    SENSING_POLARITY_INVALID = 0x00,
    SENSING_POLARITY_POSITIVE = 0x01,
    SENSING_POLARITY_NEGATIVE = 0x02,
    SENSING_POLARITY_BOTH = 0x03,
} SensingPolarity;

typedef struct
{
    uint16_t debounceUs;
    int16_t thresholdMv;
    SensingDirection direction;
    SensingDetectionMode mode;
    uint8_t stabilisationMs;
    SensingPolarity polarity;
} SensingConfig;

typedef enum
{
    BOS_FEATURE_FIFO,
    BOS_FEATURE_SYNTHESIZER,
    BOS_FEATURE_RAM_PLAYBACK,
    BOS_FEATURE_SENSING_WITH_AUTOMATIC_FEEDBACK,
    BOS_FEATURE_SENSING_THRESHOLD_DETECTION,
    BOS_FEATURE_GPO_SIGNALING,
    BOS_FEATURE_SUPPORT_BOTH_SENSING_POLARITY,

    BOS_FEATURE_LENGTH
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
    BOS_ERROR_NO_ERROR = 0,
    BOS_ERROR_SHORT_CIRCUIT,
    BOS_ERROR_UVLO,
    BOS_ERROR_IDAC,
    BOS_ERROR_OVT,
    BOS_ERROR_OVV,
    BOS_ERROR_MAX_POWER,

    BOS_ERROR_LENGTH,
} BosError;

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

typedef enum
{
    SHAPE_NO_SHAPE,
    SHAPE_SAW,
    SHAPE_TRIANGLE,
    SHAPE_SLOW_RISE,
    SHAPE_SLOW_DROP
} SliceShape;

typedef uint8_t ChannelId;

typedef struct _HapticDriver HapticDriver;

/*
 * @brief Software Reset the chip
 *
 * @param[in] driver           The haptic driver context
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosSoftwareReset)(HapticDriver* driver);

/*
 * @brief Put device to sleep or wake it up
 * @param[in] driver        The haptic driver context
 * @param[in] toSleep    true for sleep, false for wakeup
 * @return True if the operation succeeded, otherwise false
 */
typedef bool(* BosDeepSleep)(HapticDriver* driver, bool toSleep);

/**
 * @brief Return the chip id
 *
 * @param[in]   driver  The haptic driver context
 *
 * @return ChipId Return the chip id
 */
typedef ChipId (* BosGetChipId)(HapticDriver* driver);

/**
 * @brief Reads a register value of a haptic IC.
 *
 * @param[in] driver    A Haptic driver instance.
 * @param[in] addr      A register address.
 * @param[out] value    A variable pointer to return the register value.
 *
 * @return True if the operation succeed, otherwise false.
 */
typedef bool(* BosGetRegister)(HapticDriver* driver, uint8_t addr, uint16_t* value);

/**
 * @brief Read an array of registers value of a haptic IC.
 *
 * @param[in] driver        A Haptic driver instance.
 * @param[in] addrArray     An array of register addresses.
 * @param[out] valueArray   A variable pointer to return the registers value.
 * @param[in] nbRegisters   Number of registers in @a addrArray
 *
 * @note addrArray and valueArray must have the same number of entries.
 *
 * @return True if the operation succeed, otherwise false.
 */
typedef bool(* BosGetRegisters)(HapticDriver* driver, const uint8_t* addrArray, uint16_t* valueArray,
                                const uint8_t nbRegisters);

/**
 * @brief Writes a value into a haptic IC's register.
 *
 * @param[in] driver    A haptic driver instance.
 * @param[in] addr      A register address.
 * @param[in] value     A register value to set
 *
 * @return True if the operation succeed, otherwise false.
 */
typedef bool(* BosSetRegister)(HapticDriver* driver, uint8_t addr, uint16_t value);

/**
 * @brief Writes values into haptic IC's registers.
 *
 * @param[in] driver        A haptic driver instance.
 * @param[in] addrArray     An array of registers address.
 * @param[in] valueArray    A array of registers value to set.
 * @param[in] nbRegisters   Number of registers into @a addrArray.
 *
 * @note addrArray and valueArray must have the same number of entries.
 *
 * @return True if the operation succeed, otherwise false.
 */
typedef bool(* BosSetRegisters)(HapticDriver* driver, const uint8_t* addrArray, const uint16_t* valueArray,
                                const uint8_t nbRegisters);

/**
 * @brief Writes a value into haptic IC's register and reads the current broadcast register value.
 *
 * @param[in] driver            A haptic driver instance.
 * @param[in] addr              A register address.
 * @param[in] value             A register value to set.
 * @param[out] broadcastValue   A variable pointer to return the broadcast register value.
 *
 * @return True if the operation succeed, otherwise false.
 */
typedef bool(* BosSetGetRegister)(HapticDriver* driver, const uint8_t addr, uint16_t value, uint16_t* broadcastValue);

/**
 * @brief Writes values into haptic IC's registers and reads the current broadcast register values.
 *
 * @param[in] driver            A haptic driver instance.
 * @param[in] addrArray         A array of registers address.
 * @param[in] valueArray        A array of registers value to set.
 * @param[out] broadcastArray   A pointer to an array to return the broadcast register values.
 * @param[in] nbRegisters       Number of registers into @a addrArray.
 *
 * @note addrArray, valueArray and broadcastArray must have the same number of entries.
 *
 * @return True if the operation succeed, otherwise false.
 */
typedef bool(* BosSetGetRegisters)(HapticDriver* driver, const uint8_t* addrArray, const uint16_t* valueArray,
                                   uint16_t* broasdcastArray, const uint8_t nbRegisters);


/*
 * @brief Get the number of register available
 */
typedef size_t (* BosGetNumberOfRegister)(void);

/*
 * @brief Configure a slice which is part of a waveform (group of slice)
 *
 * @param[in] driver        The haptic driver context
 * @param[in] slice         Slice configuration
 * @param[in] outputChannel The output channel of the slice
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (* BosSynthesizerSetSlice)(HapticDriver* driver, const SynthSlice* slice, SliceShape shape, const uint8_t outputChannel);

/**
 * @brief Set the synthesizer mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] waveformId    The waveform id to configure
 * @param[in] frequency     The frequency of the specified waveform
 * @param[in] amplitude     The amplitude of the specified waveform
 * @param[in] cycle         The amount of waveform period to play
 * @param[in] shape         Shape In/Out arrangement
 * @param[in] outputChannel The output channel to play the waveform
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosSynthesizerSetWaveform)(HapticDriver* driver, WaveformId id, uint8_t startSliceId,
                                          size_t nbrOfSlices, uint16_t cycle, uint8_t outputChannel);


/**
 * @brief Activate or deactivate the output
 *
 * @param[in] driver           The haptic driver context
 * @param[in] activate      True if activating the output, False if deactivating the output
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosCtrlOutput)(HapticDriver* driver, bool activate);

/**
 * @brief Configure the GPO mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] channel       The channel specific to this IC
 * @param[in] state         The GPO mode to configure
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BOSCtrlGPO)(HapticDriver* driver, ChannelId channelId, GPOCtrl state);

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
typedef void(* BOSEventCb)(HapticDriver* driver, ChannelId channelId, BOSEvent event, void* context);

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
typedef bool(* BOSRegisterEvents)(HapticDriver* driver, ChannelId channelId, BOSEventCb cb, void* context);

/*
 * @brief Remove from event callback registration for GPO signals
 *
 * @param[in] driver            The haptic driver instance
 * @param[in] channelId         The channel specific to unregister for event
 *
 * @return True if the operation succeed, false otherwise
 */
typedef bool(* BOSUnregisterEvents)(HapticDriver* driver, ChannelId channelId);

/*
 * @brief Toggle the synthesizer play
 *
 * @param[in] driver        The haptic driver context
 * @param[in] start         The waveform id where the sequencer starts to play
 * @param[in] stop          The waveform id where the sequencer stops to play
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosWaveSynthesizerPlay)(HapticDriver* driver, WaveformId start, WaveformId stop);

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
typedef bool(* BosRamplayback)(HapticDriver* driver, uint32_t samplingRate, void* data, size_t length,
                               uint8_t channelMask);

/**
 * @brief Play the content of the ram playback
 *
 * @param[in] driver        The haptic driver context
 * @param[in] length        The length of the data to play
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosRamPlaybackToggle)(HapticDriver* driver, size_t length);


/**
 * @brief Configure the chip in Fifo mode
 *
 * @param[in] driver        The haptic driver context
 * @param[in] samplingRate  The number of kilo sample per second to play
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosSetFifoMode)(HapticDriver* driver, uint32_t samplingRate, uint8_t outputChannel);

/**
 * @brief Write data in the fifo
 *
 * @param[in] driver        The haptic driver context
 * @param[in] data          The data to write in the fifo
 * @param[in] length        The length of the data to write
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosWriteFifoSamples)(HapticDriver* driver, void* data, size_t length);

/**
 * @brief Get the fifo space avaiable in the fifo
 *
 * @param[in] driver            The haptic driver context
 * @param[in] availableSpace    The available space in the fifo
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosGetFifoSpace)(HapticDriver* ctx, uint16_t* availableSpace);

/**
 * @brief Get the FIFO max available space
 *
 * @return uint16_t The max space available
 */
typedef uint16_t (* BosGetMaximumFifoSpace)(void);

/**
 * @brief Activate or deactivate the synchronization mode between chip
 *
 * @param[in] driver        The haptic driver context
 * @param[in] activate      True for activating and False for deactivating
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosSynch)(HapticDriver* driver, bool activate);

/**
 * @brief Referencing the voltage value passed in the array to values referencing in the chip domain
 *
 * @param[in]       driver  The haptic driver context
 * @param[in]       dataIn  The data to reference
 * @param[in]       length  The length of the array to reference
 * @param[out]      dataOut The data converted to the IC reference voltage.
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosReferencingFromVolt)(HapticDriver* driver, int16_t* dataIn, size_t length, int16_t* dataOut);

/**
 * @brief Referencing the relative int16 value passed in the array to values referencing in the chip domain
 *
 * @param[in]       driver  The haptic driver context
 * @param[in]       dataIn  The relative data point (between INT16_MIN and INT16_MAX)
 * @param[in]       length  The length of the array to reference
 * @param[in]       vMinVolt  The minimum voltage supported by the piezo(s)
 * @param[in]       vMaxVolt  The maximum voltage supported by the piezo(s)
 * @param[out]      dataOut The data converted to the IC reference voltage.
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool(* BosReferencingFromRelativeInt16)(HapticDriver* driver, int16_t* dataIn, size_t length,
                                                int16_t vMinVolt, int16_t vMaxVolt, int16_t* dataOut);

/*
 * @brief Get the error status bits
 *
 * @param[in] driver        The haptic driver context
 * @param[in] errors        Provided error array
 * @param[in] maxLength     Max number of error to copy in the errors array
 *
 * @return The amount of active errors
 */
typedef size_t(* BosGetError)(HapticDriver* driver, BosError* errors, size_t maxLength);

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
typedef void (* BosSensingEvent)(uint8_t channel, SensingDetectionMode mode, SensingDirection direction, void* arg);

/*
 * @brief Configures the chip for sensing
 *
 * @param[in] driver        The haptic driver concerned
 * @param[in] channel       The channel to configure the sensing for.
 * @param[in] config        Sensing configuration to apply
 * @param[in] type          Sensing type desired
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (* BosButtonSensing)(HapticDriver* driver, uint8_t channel, SensingConfig config);

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
typedef bool (* BosSensingAutoPlayWave)(HapticDriver* driver, ChannelId channelId, WaveformId id,
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
typedef bool (* BosSensingStop)(HapticDriver* driver, ChannelId channelId);

/*
 * @brief Read output in Volt when in sensing
 *
 * @param[in] driver    The haptic driver context
 * @param[in] channel   The channel to deactivate the sensing
 * @param[in] *data     The variable pointer in which the output value will be placed
 *
 * @return True if the operation succeed, otherwise false
 */
typedef bool (* BosSenseOutput)(HapticDriver* driver, ChannelId channelId, int32_t* data);

/*
 * @brief (Engineering feature) Access register shadow value of the driver
 *
 * @return Registers The register set corresponding to the IC
 */
typedef Registers* (* BosGetShadowRegisters)(HapticDriver* driver);

/*
 * @brief (Engineering feature) Set a specific register using the Register struct
 *
 * @param[in] driver        The haptic driver context
 * @param[in] reg           The register to set
 *
 * @return bool             True if the operation succeed and false if the operation failed
 */
typedef bool(* BosWriteRegister)(HapticDriver* driver, Register* reg);

/*
 * @brief (Engineering feature) Set a specific register using the Register struct
 *
 * @param[in] driver        The haptic driver context
 * @param[in|out] reg       The register to read. If the operation succeed, the value of the register will be updated.
 *
 * @return bool             True if the operation succeed and false if the operation failed
 */
typedef bool(* BosReadRegister)(HapticDriver* driver, Register* reg);

/*
 * @brief (Engineering feature) Read the GPIO pin state
 *
 * @param[in] driver        The haptic driver context
 * @param[in] channel       The channel specific to this IC
 *
 * @return GPIOState        The state of the Gpio
 */
typedef GPIOState(* BosGetGpioState)(HapticDriver* driver, ChannelId channelId);

/**
 * @brief Interrogate driver if a specific feature is supported
 *
 * @param[in] feature       The feature request
 *
 * @return bool             True if the feature is supported, false if not
 */
typedef bool (* SupportedFeature)(HapticDriver* driver, BosFeature feature);

/**
 * @brief Enable the sensing on the specified channel
 *
 * @param[in] driver        The haptic driver context
 * @param[in] channel       The channel specific to this IC
 *
 * @return bool             True if the operation succeed and false if the operation failed
 */
typedef bool (* BosEnableSensing)(HapticDriver* driver, ChannelId channelId);

/**
 * @brief Set the polarity of the sensing (Ex: Set the HBridge of the BOS1901 in the good direction for sensing positive or negative voltage)
 *
 * @param[in] driver         The haptic driver context
 * @param[in] channel       The channel specific to this IC
 *
 * @return bool             True if the operation succeed and false if the operation failed
 */
typedef bool (* BosSetSensingPolarity)(HapticDriver* driver, ChannelId channelId, SensingPolarity polarity);

struct _HapticDriver
{
    BosGetChipId getChipId;
    BosSoftwareReset softwareReset;
    BosDeepSleep deepSleep;
    BosGetRegister getRegister;
    BosGetRegisters getRegisters;
    BosSetRegister setRegister;
    BosSetRegisters setRegisters;
    BosSetGetRegister setGetRegister;
    BosSetGetRegisters setGetRegisters;
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
    BosReferencingFromRelativeInt16 referencingFromInt16;
    BosGetNumberOfRegister numberOfRegister;
    BosButtonSensing buttonSensing;
    BosEnableSensing enableSensing;
    BosSensingAutoPlayWave sensingAutoPlayWave;
    BosSensingStop stopSensing;
    SupportedFeature isSupported;
    BosSenseOutput senseOutput;
    BosSetSensingPolarity setSensingPolarity;

    /* Engineering functions */
    BosGetShadowRegisters getShadowRegisters;
    BosWriteRegister writeRegister;
    BosReadRegister readRegister;
    BosGetGpioState getGpioState;
};

#endif //DKCORE_BOSDRIVER_H
