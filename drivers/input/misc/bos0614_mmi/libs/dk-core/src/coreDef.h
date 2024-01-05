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

#ifndef DK_CORE_COREDEF_H
#define DK_CORE_COREDEF_H

#include "coreDefOption.h"

#ifndef NBR_OF_BUTTON
#define NBR_OF_BUTTON 0
#endif

#ifndef NBR_OF_TL1963A
#define NBR_OF_TL1963A 0
#endif

#ifndef NBR_OF_IQS263
#define NBR_OF_IQS263 0
#endif

#ifndef NBR_OF_MCP45HV
#define NBR_OF_MCP45HV 0
#endif

#ifndef NBR_OF_TCA9535
#define NBR_OF_TCA9535 0
#endif

#ifndef NBR_OF_TCA9554A
#define NBR_OF_TCA9554A 0
#endif

#ifndef NBR_OF_IO_EXPANDER_GPIO
#define NBR_OF_IO_EXPANDER_GPIO 0
#endif

#ifndef NBR_OF_TCA9548A
#define NBR_OF_TCA9548A 0
#endif

#ifndef NBR_OF_I2C_SWITCH
#define NBR_OF_I2C_SWITCH 0
#endif

#ifndef NBR_OF_I2C_SWITCH_CHANNEL
#define NBR_OF_I2C_SWITCH_CHANNEL 0
#endif

#ifndef NBR_OF_LED
#define NBR_OF_LED 0
#endif

#ifndef NBR_INA226
#define NBR_INA226 0
#endif

#ifndef NBR_OF_TMP102
#define NBR_OF_TMP102 0
#endif

#ifndef NBR_OF_TMP1075
#define NBR_OF_TMP1075 0
#endif

#ifndef BOS0614_NBR_OF_INSTANCE
#define BOS0614_NBR_OF_INSTANCE 0
#endif

#ifndef BOS1211_NBR_OF_INSTANCE
#define BOS1211_NBR_OF_INSTANCE 0
#endif

#ifndef SENSING_INSTANCE_NBR
#define SENSING_INSTANCE_NBR 0
#endif

#ifndef SENSING_CALLBACK_SUBSCRIBER_DEPTH
#define SENSING_CALLBACK_SUBSCRIBER_DEPTH (0)
#endif

#ifndef BOS1901_NBR_OF_INSTANCE
#define BOS1901_NBR_OF_INSTANCE 0
#endif

#ifndef NBR_OF_TDC7200
#define NBR_OF_TDC7200 0
#endif


//Trigger playback config
#ifndef MAXIMUM_NB_OF_CHANNEL_LINKED
#define MAXIMUM_NB_OF_CHANNEL_LINKED (3)
#endif

#ifndef ISR_PLAYBACK_INSTANCE_NBR
#define ISR_PLAYBACK_INSTANCE_NBR (1)
#endif


//Streaming config
#ifndef STREAMING_INSTANCE_NBR
#define STREAMING_INSTANCE_NBR (8)
#endif


//Stack config
#ifndef STACK_MAX_TRANSACTION_LENGTH
#define STACK_MAX_TRANSACTION_LENGTH (128)
#endif

#ifndef STACK_MAX_PENDING_TRANSACTION
#define STACK_MAX_PENDING_TRANSACTION (8)
#endif


//Player config
#ifndef PLAYER_FEEDBACK_NBR_OF_WAVEFORM
#define PLAYER_FEEDBACK_NBR_OF_WAVEFORM (16)
#endif

#ifndef PLAYER_INSTANCE_NBR
#define PLAYER_INSTANCE_NBR (0)
#endif

#ifndef PLAYER_BUFFER_SIZE
#define PLAYER_BUFFER_SIZE (5120)   // 5120 * 16bitPerSample = 10KB
#endif

#ifndef PLAYER_CALLBACK_SUBSCRIBER_DEPTH
#define PLAYER_CALLBACK_SUBSCRIBER_DEPTH (0)
#endif


//Sensing config
#ifndef SENSING_NBR_OF_CONFIG_PER_CHANNEL
#define SENSING_NBR_OF_CONFIG_PER_CHANNEL (2)
#endif

#ifndef SENSING_CHANNEL_PER_DRIVER
#define SENSING_CHANNEL_PER_DRIVER (1)
#endif


//IC Status config
#ifndef NBR_OF_ERROR
#define NBR_OF_ERROR (0)
#endif

#ifndef NBR_OF_HAPTIC_DRIVER
#define NBR_OF_HAPTIC_DRIVER (0)
#endif


// HFEM config
#ifndef HFEM_NBR_OF_DRIVERS
#define HFEM_NBR_OF_DRIVERS (0)
#endif

#ifndef HFEM_MAXIMUM_NBR_OF_TRIGGER
#define HFEM_MAXIMUM_NBR_OF_TRIGGER (0)
#endif

#ifndef HFEM_INSTANCE_MAXIMUM_NBR
#define HFEM_INSTANCE_MAXIMUM_NBR (0)
#endif

// Captouch config
#ifndef CAPTOUCH_MAXIMUM_INSTANCE_NBR
#define CAPTOUCH_MAXIMUM_INSTANCE_NBR (0)
#endif

#ifndef USER_BUTTON_INDEX
#define USER_BUTTON_INDEX   (0)
#endif

#ifndef FITNESS_TRACKER_CAPTOUCH_INDEX
#define FITNESS_TRACKER_CAPTOUCH_INDEX  (0)
#endif

#endif //DK_CORE_COREDEF_H
