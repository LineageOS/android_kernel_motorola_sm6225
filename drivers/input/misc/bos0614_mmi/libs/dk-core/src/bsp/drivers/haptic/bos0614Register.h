//
// Description: BOS0614 Register Definition
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

#ifndef BOS0614_REGISTER_H_
#define BOS0614_REGISTER_H_

#include <linux/types.h>

#define ADDRESS_BOS0614_REFERENCE_REG        0x0000
#define ADDRESS_BOS0614_IC_STATUS_REG        0x0001
#define ADDRESS_BOS0614_READ_REG             0x0002
#define ADDRESS_BOS0614_GPIOX_REG            0x0003
#define ADDRESS_BOS0614_TC_REG               0x0004
#define ADDRESS_BOS0614_CONFIG_REG           0x0005
#define ADDRESS_BOS0614_SENSECONFIG_REG      0x0006
#define ADDRESS_BOS0614_SENSE0_REG           0x0007
#define ADDRESS_BOS0614_SENSE0P_REG          0x0008
#define ADDRESS_BOS0614_SENSE0R_REG          0x0009
#define ADDRESS_BOS0614_SENSE0S_REG          0x000A
#define ADDRESS_BOS0614_SENSE1_REG           0x000B
#define ADDRESS_BOS0614_SENSE1P_REG          0x000C
#define ADDRESS_BOS0614_SENSE1R_REG          0x000D
#define ADDRESS_BOS0614_SENSE1S_REG          0x000E
#define ADDRESS_BOS0614_SENSE2_REG           0x000F
#define ADDRESS_BOS0614_SENSE2P_REG          0X0010
#define ADDRESS_BOS0614_SENSE2R_REG          0x0011
#define ADDRESS_BOS0614_SENSE2S_REG          0x0012
#define ADDRESS_BOS0614_SENSE3_REG           0x0013
#define ADDRESS_BOS0614_SENSE3P_REG          0x0014
#define ADDRESS_BOS0614_SENSE3R_REG          0x0015
#define ADDRESS_BOS0614_SENSE3S_REG          0x0016
#define ADDRESS_BOS0614_SENSESTATUS_REG      0x0017
#define ADDRESS_BOS0614_SENSEDATA0_REG       0x0018
#define ADDRESS_BOS0614_SENSEDATA1_REG       0x0019
#define ADDRESS_BOS0614_SENSEDATA2_REG       0x001A
#define ADDRESS_BOS0614_SENSEDATA3_REG       0x001B
#define ADDRESS_BOS0614_SENSERAW0_REG        0X001C
#define ADDRESS_BOS0614_SENSERAW1_REG        0x001D
#define ADDRESS_BOS0614_SENSERAW2_REG        0x001E
#define ADDRESS_BOS0614_SENSERAW3_REG        0x001F
#define ADDRESS_BOS0614_KPA_REG              0x0020
#define ADDRESS_BOS0614_KP_KI_REG            0x0021
#define ADDRESS_BOS0614_DEADTIME_REG         0x0022
#define ADDRESS_BOS0614_PARCAP_REG           0x0023
#define ADDRESS_BOS0614_SUP_RISE_REG         0x0024
#define ADDRESS_BOS0614_TRIM_REG             0x0025
#define ADDRESS_BOS0614_CHIP_ID_REG          0x0026
#define ADDRESS_BOS0614_VFEEDBACK_REG        0x0028
#define ADDRESS_BOS0614_FIFO_STATE_REG       0x0029
#define ADDRESS_BOS0614_AUTO_STATE_REG       0x002A
#define ADDRESS_BOS0614C_RAM_DATA_REG        0x002B
#define ADDRESS_BOS0614C_SENSE_OFFSET_REG    0x002C
#define ADDRESS_BOS0614_BIST_REG             0x0030
#define ADDRESS_BOS0614_BISTRES_REG          0x0031
#define ADDRESS_BOS0614_DEBUG_REG            0x0032
#define ADDRESS_BOS0614_THRESH_REG           0x0033
#define ADDRESS_BOS0614_REG38_REG            0x0038
#define ADDRESS_BOS0614B_SENSE_OFFSET_REG    0x0039
#define ADDRESS_BOS0614_CALIB_DATA_REG       0x003A
#define ADDRESS_BOS0614_REG3B_REG            0x003B
#define ADDRESS_BOS0614_REG3C_REG            0x003C
#define ADDRESS_BOS0614_REG3D_REG            0x003D
#define ADDRESS_BOS0614_REG3E_REG            0x003E
#define ADDRESS_BOS0614_REG3F_REG            0x003F
#define ADDRESS_BOS0614_REG40_REG            0x0040
#define ADDRESS_BOS0614_REG41_REG            0x0041
#define ADDRESS_BOS0614_REG42_REG            0x0042
#define ADDRESS_BOS0614_REG43_REG            0x0043
#define ADDRESS_BOS0614B_RAM_DATA_REG        0x0044
#define ADDRESS_BOS0614_REG45_REG            0x0045
#define ADDRESS_BOS0614_REG46_REG            0x0046
#define ADDRESS_BOS0614_REG47_REG            0x0047

#define BOS0614_MAXIMUM_REG_ADDR (ADDRESS_BOS0614_REG47_REG)

typedef struct
{
    uint16_t value;
    uint16_t addr;
} Bos0614RegisterStruct;

typedef union
{
    const uint32_t all;
    Bos0614RegisterStruct generic;
} Bos0614Register;

typedef struct
{
    Bos0614Register reg;
} UndefReg;

//***********************************************
//		REFERENCE register - address 0x00
//***********************************************
struct ReferenceBits0614
{
    uint16_t fifo: 12;                  //   11:0   Input of the FIFO
    uint16_t channel: 4;                //  12:15   Channel 3 enable
};

union ReferenceReg0614
{
    Bos0614Register reg;
    struct ReferenceBits0614 bit;
};

//***********************************************
//		REFERENCE register - address 0x00  REMAP
//***********************************************
struct ReferenceRmBits0614
{
    uint16_t data: 16;                  //  15:0    Input of the RAM communication
};

union ReferenceRmReg0614
{
    Bos0614Register reg;
    struct ReferenceRmBits0614 bit;
};


//***********************************************
//		IC_STATUS register - address 0x01
//***********************************************
struct IcStatusBits0614
{
    uint16_t empty: 1;                  //   0      FIFO Empty
    uint16_t full: 1;                   //   1      FIFO Full
    uint16_t sc: 1;                     //   2      Short Circuit Status
    uint16_t uvlo: 1;                   //   3      Under Voltage
    uint16_t idac: 1;                   //   4      Status bit
    uint16_t maxPower: 1;               //   5      Amount of Power
    uint16_t ovt: 1;                    //   6      Over Temperature
    uint16_t ovv: 1;                    //   7 	    Over Voltage
    uint16_t state: 2;                  //   9:8    State of controller
    uint16_t pressRelease: 4;           //   10:13  State of sense channels
    uint16_t senseAll: 1;               //   14     Trigger condition met
    const uint16_t reserved: 1;         //   15     Reserved
};

union IcStatusReg0614
{
    Bos0614Register reg;
    struct IcStatusBits0614 bit;
};

//***********************************************
//		READ register - address 0x02
//***********************************************
struct ReadBits0614
{
    uint16_t bc: 8;                     //   7:0    Address of internal register whose content is returned on comm bus
    const uint16_t reserved: 8;         //   15:8   Reserved
};

union ReadReg0614
{
    Bos0614Register reg;
    struct ReadBits0614 bit;
};

//***********************************************
//		GPIOX register - address 0x03
//***********************************************

typedef enum
{
    Bos0614GPOCtrl_ButtonState = 0,
    Bos0614GPOCtrl_SenseTrigger,
    Bos0614GPOCtrl_SenseTriggerAny,
    Bos0614GPOCtrl_WaveformFifoDone,
    Bos0614GPOCtrl_ErrorNotify,
    Bos0614GPOCtrl_MaxPower,
    Bos0614GPOCtrl_NextData,
    Bos0614GPOCtrl_RequestPlay,
} Bos0614GpioMode;

struct GpioxBits0614
{
    Bos0614GpioMode gpio0: 4;           //   3:0    Controls bit of GPIO0
    Bos0614GpioMode gpio1: 4;           //   7:4    Controls bit of GPIO1
    Bos0614GpioMode gpio2: 4;           //   11:8   Controls bit of GPIO2
    Bos0614GpioMode gpio3: 4;           //   15:12  Controls bit of GPIO3
};

union GpioxReg0614
{
    Bos0614Register reg;
    struct GpioxBits0614 bit;
};

//***********************************************
//		TC register - address 0x04 - REV_B
//***********************************************
struct TcBits0614RevB
{
    uint16_t tc: 5;                     //   4:0    Time to short output
    uint16_t pc: 1;                     //   5      Short piezo at end of waveform if = 1
    uint16_t pol: 1;                    //   6      If GPIO sensing
    const uint16_t reserved: 9;         //   15:7   Reserved
};

union TcReg0614RevB
{
    Bos0614Register reg;
    struct TcBits0614RevB bit;
};

//***********************************************
//		TC register - address 0x04 - REV_C
//***********************************************
struct TcBits0614RevC
{
    uint16_t tcr: 5;                    //   4:0    Time to short the output after end of a waveform for button press
    uint16_t tcp: 5;                    //   9:6    Time to short the output after end of a waveform for button release
    uint16_t pc: 1;                     //   10     Short piezo at end of waveform if = 1
    uint16_t pol: 1;                    //   11     If GPIO sensing
    const uint16_t reserved: 4;         //   15:7   Reserved
};

union TcReg0614RevC
{
    Bos0614Register reg;
    struct TcBits0614RevC bit;
};

//***********************************************
//		CONFIG register - address 0x5
//***********************************************
typedef enum
{
    Bos0614SamplingRate_1024Ksps = 0,
    Bos0614SamplingRate_512Ksps,
    Bos0614SamplingRate_256Ksps,
    Bos0614SamplingRate_128Ksps,
    Bos0614SamplingRate_64Ksps,
    Bos0614SamplingRate_32Ksps,
    Bos0614SamplingRate_16Ksps,
    Bos0614SamplingRate_8Ksps,
} Bos0614SamplingRate;

typedef enum
{
    Bos0614OutputDisabled = 0,
    Bos0614OutputEnabled,
} Bos0614OutputOE;


typedef enum
{
    Bos0614Mode_Direct = 0x0,
    Bos0614Mode_FIFO = 0x1,
    Bos0614Mode_RAM_Playback,
    Bos0614Mode_RAM_Synthesis,
} Bos0614Mode;

struct ConfigBits0614
{
    Bos0614SamplingRate play: 3;        //   2:0    Waveform play back speed
    uint16_t ds: 1;                     //     3    Power mode when not playing waveforms
    Bos0614OutputOE oe: 1;              //     4    Enable/Disable waveform playback
    uint16_t lock: 1;                   //     5    Register lock
    uint16_t rst: 1;                    //     6    Software Reset
    uint16_t upi: 1;                    //     7    UPI enable
    uint16_t tout: 1;                   //     8    Auto TimeOut
    Bos0614Mode ram: 2;                 //  10:9    Ram Behavior
    uint16_t str: 1;                    //    11    Comm Address counter behavior
    uint16_t SHORT: 2;                  // 13:12    Length of time to short
    uint16_t od: 1;                     //    14    GPIOs Open Drain
    uint16_t sc: 1;                     //    15    Channel selection

};

union ConfigReg0614
{
    Bos0614Register reg;
    struct ConfigBits0614 bit;
};


//***********************************************
//	SENSECONFIG register - address 0x06 - REV_B
//***********************************************
struct SenseConfigBits0614RevB
{
    uint16_t ch0: 1;                    //     0    CH0 Sensing
    uint16_t ch1: 1;                    //     1    CH1 Sensing
    uint16_t ch2: 1;                    //     2    CH2 Sensing
    uint16_t ch3: 1;                    //     3    CH3 Sensing
    uint16_t cal: 1;                    //     4    Calibrate sense interface
    uint16_t same: 1;                   //     5    All sense have the same configuration (Sense0)
    uint16_t neg: 2;                    //   7:6    Reset point voltage negative
    uint16_t pos: 2;                    //   9:8    Reset point voltage positive
    uint16_t scomp: 3;                  // 12:10    Discharged on the sense channel
    uint16_t sh: 1;                     //    13    Short Output during playback
    uint16_t diff: 1;                   //	  14    ShutDown sensing when not needed
    uint16_t extTrig: 1;                //	  15    EXT trigger enable
};

union SenseConfigReg0614RevB
{
    Bos0614Register reg;
    struct SenseConfigBits0614RevB bit;
};


//***********************************************
//  SENSECONFIG register - address 0x06 - REV_C
//***********************************************
struct SenseConfigBits0614RevC
{
    uint16_t ch0: 1;                    //     0    CH0 Sensing
    uint16_t ch1: 1;                    //     1    CH1 Sensing
    uint16_t ch2: 1;                    //     2    CH2 Sensing
    uint16_t ch3: 1;                    //     3    CH3 Sensing
    uint16_t cal: 1;                    //     4    Calibrate sense interface
    uint16_t same: 1;                   //     5    All sense have the same configuration (Sense0)
    uint16_t samp: 2;                   //   7:6    Set the reset point of sense voltage to
    uint16_t scompAuto: 2;              //   9:8    Change the behavior of SCOMP after the end of a waveform play
    uint16_t scomp: 2;                  // 12:10    Defines time between zeroing of output channel to null the effect of discharge on the sensed channels
    uint16_t seq: 1;                    //    13    Set if more than one channel with auto triggering can play haptic at the same time
    uint16_t zps: 1;                    //	  15    Sets if a ZPS detection is sufficient to be considered as a valid press
    uint16_t zps_sens: 1;               //	  15    Sensitivity of the ZPS wakeup signal
    uint16_t extTrig: 1;                //	  15    Activate external trigger

};

union SenseConfigReg0614RevC
{
    Bos0614Register reg;
    struct SenseConfigBits0614RevC bit;
};

//***********************************************
//  SENSECONFIG register - address 0x06 - Common
//***********************************************
struct SenseConfigBits0614Common
{
    uint16_t ch0: 1;                    //     0    CH0 Sensing
    uint16_t ch1: 1;                    //     1    CH1 Sensing
    uint16_t ch2: 1;                    //     2    CH2 Sensing
    uint16_t ch3: 1;                    //     3    CH3 Sensing
    uint16_t cal: 1;                    //     4    Calibrate sense interface
    uint16_t same: 1;                   //     5    All sense have the same configuration (Sense0)
    const uint16_t reserved: 6;         //	14:6    Reserved
    uint16_t extTrig: 1;                //	  15    EXT trigger enable

};

union SenseConfigReg0614Common
{
    Bos0614Register reg;
    struct SenseConfigBits0614Common bit;
};


//***********************************************
//		SENSE0 register - address 0x07
//***********************************************
typedef struct
{
    uint16_t t1: 1;                     //     0    Threshold Sensing
    uint16_t t2: 1;                     //     1    Threshold Sensing
    uint16_t s1: 1;                     //     2    Threshold Sensing
    uint16_t s2: 1;                     //     3    Threshold Sensing
    uint16_t autop: 1;                  //     4    Start Playing on senseP
    uint16_t autor: 1;                  //     5    Start Playing on senseR
    uint16_t wvp: 3;                    //   8:6    Waveform to play   P
    uint16_t wvr: 3;                    //  11:9    Waveform to play   R
    const uint16_t reserved: 4;         // 15:12    Reserved
} SenseBits0614;

union Sense0Reg0614
{
    Bos0614Register reg;
    SenseBits0614 bit;
};

//***********************************************
//		SENSE0P register - address 0x08
//***********************************************

typedef enum
{
    SensingThresholdMode_Above = 0x0,
    SensingThresholdMode_Below = 0x1
} SensingThresholdMode;

typedef enum
{
    DebouncingTime_1us = 0,
    DebouncingTime_100us = 1,
    DebouncingTime_500us = 2,
    DebouncingTime_1ms = 3,
    DebouncingTime_2ms = 4,
    DebouncingTime_4ms = 5,
    DebouncingTime_8ms = 6,
    DebouncingTime_16ms = 7
} DebouncingTime;

typedef struct
{
    uint16_t threshold: 12;             //  11:0    Threshold Sensing
    SensingThresholdMode ab: 1;         //    12    Above or Below
    uint16_t rep: 3;                    // 15:13    Time for sense
} SenseThreshold;

union Sense0PReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE0R register - address 0x09
//***********************************************

union Sense0RReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE0S register - address 0x0A
//***********************************************
typedef struct
{
    uint16_t slope1: 7;                 //   6:0    Slope 1 of force
    uint16_t abs1: 1;                   //     1    Above or Below
    uint16_t slope2: 7;                 //  14:8    Slope 2 of force
    uint16_t abs2: 1;                   //     1    Above or Below
} SenseSBits0614;

union Sense0SReg0614
{
    Bos0614Register reg;
    SenseSBits0614 bit;
};

//***********************************************
//		SENSE1 register - address 0x0B
//***********************************************

union Sense1Reg0614
{
    Bos0614Register reg;
    SenseBits0614 bit;
};

//***********************************************
//		SENSE1P register - address 0x0C
//***********************************************

union Sense1PReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE1R register - address 0x0D
//***********************************************

union Sense1RReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE1S register - address 0x0E
//***********************************************

union Sense1SReg0614
{
    Bos0614Register reg;
    SenseSBits0614 bit;
};

//***********************************************
//		SENSE2 register - address 0x0F
//***********************************************

union Sense2Reg0614
{
    Bos0614Register reg;
    SenseBits0614 bit;
};

//***********************************************
//		SENSE0P register - address 0x10
//***********************************************
union Sense2PReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE2R register - address 0x11
//***********************************************

union Sense2RReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE2S register - address 0x12
//***********************************************

union Sense2SReg0614
{
    Bos0614Register reg;
    SenseSBits0614 bit;
};

//***********************************************
//		SENSE3 register - address 0x13
//***********************************************

union Sense3Reg0614
{
    Bos0614Register reg;
    SenseBits0614 bit;
};

//***********************************************
//		SENSE3P register - address 0x14
//***********************************************

union Sense3PReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE3R register - address 0x15
//***********************************************

union Sense3RReg0614
{
    Bos0614Register reg;
    SenseThreshold bit;
};

//***********************************************
//		SENSE3S register - address 0x16
//***********************************************

union Sense3SReg0614
{
    Bos0614Register reg;
    SenseSBits0614 bit;
};

//***********************************************
//		SENSESTATUS register - address 0x17
//***********************************************
struct SenseStatusBits0614
{
    uint16_t t10: 1;                    //     1    Comparator status press threshold ch0
    uint16_t t20: 1;                    //     1    Comparator status release threshold ch0
    uint16_t s10: 1;                    //     1    Comparator status slope 1 ch0
    uint16_t s20: 1;                    //     1    Comparator status slope 2 ch0
    uint16_t t11: 1;                    //     1    Comparator status press threshold ch1
    uint16_t t21: 1;                    //     1    Comparator status release threshold ch1
    uint16_t s11: 1;                    //     1    Comparator status slope 1 ch1
    uint16_t s21: 1;                    //     1    Comparator status slope 2 ch1
    uint16_t t12: 1;                    //     1    Comparator status press threshold ch2
    uint16_t t22: 1;                    //     1    Comparator status release threshold ch2
    uint16_t s12: 1;                    //     1    Comparator status slope 1 ch2
    uint16_t s22: 1;                    //     1    Comparator status slope 2 ch2
    uint16_t t13: 1;                    //     1    Comparator status press threshold ch3
    uint16_t t23: 1;                    //     1    Comparator status release threshold ch3
    uint16_t s13: 1;                    //     1    Comparator status slope 1 ch3
    uint16_t s23: 1;                    //     1    Comparator status slope 2 ch3
};

union SenseStatusReg0614
{
    Bos0614Register reg;
    struct SenseStatusBits0614 bit;
};

//***********************************************
//		SENSEDATA0 register - address 0x18
//***********************************************
struct SenseDataBits0614
{
    uint16_t data: 16;                  //     16   Sensing data
};

union SenseData0Reg0614
{
    Bos0614Register reg;
    struct SenseDataBits0614 bit;
};

//***********************************************
//		SENSEDATA1 register - address 0x19
//***********************************************

union SenseData1Reg0614
{
    Bos0614Register reg;
    struct SenseDataBits0614 bit;
};

//***********************************************
//		SENSEDATA2 register - address 0x1A
//***********************************************

union SenseData2Reg0614
{
    Bos0614Register reg;
    struct SenseDataBits0614 bit;
};

//***********************************************
//		SENSEDATA3 register - address 0x1B
//***********************************************

union SenseData3Reg0614
{
    Bos0614Register reg;
    struct SenseDataBits0614 bit;
};

//***********************************************
//		SENSERAW0 register - address 0x1C
//***********************************************
struct SenseRawBits0614
{
    uint16_t data: 10;                  //   9:0    Sensing data
    const uint16_t reserved: 6;         // 15:10    Reserved

};

union SenseRaw0Reg0614
{
    Bos0614Register reg;
    struct SenseRawBits0614 bit;
};

//***********************************************
//		SENSERAW1 register - address 0x1D
//***********************************************

union SenseRaw1Reg0614
{
    Bos0614Register reg;
    struct SenseRawBits0614 bit;
};

//***********************************************
//		SENSERAW2 register - address 0x1E
//***********************************************

union SenseRaw2Reg0614
{
    Bos0614Register reg;
    struct SenseRawBits0614 bit;
};

//***********************************************
//		SENSERAW3 register - address 0x1F
//***********************************************

union SenseRaw3Reg0614
{
    Bos0614Register reg;
    struct SenseRawBits0614 bit;
};

//***********************************************
//		KPA register - address 0x20
//***********************************************
struct KpaBits0614
{
    uint16_t kpa: 8;                    //   7:0    Internal Parameter
    uint16_t fswMax: 2;                 //   9:8    Boost converter max frequency
    uint16_t sb: 2;                     // 11:10    Boost converter blanking time
    const uint16_t reserved: 4;         // 15:12    Reserved
};

union KpaReg0614
{
    Bos0614Register reg;
    struct KpaBits0614 bit;
};

//***********************************************
//		KP_KI register - address 0x21
//***********************************************
struct KpKiBits0614
{
    uint16_t kp: 11;                    //  10:0    Internal Parameter
    uint16_t kiBase: 4;                 // 14:11    Internal Parameter
    const uint16_t reserved: 1;         //    15    Reserved
};

union KpKiReg0614
{
    Bos0614Register reg;
    struct KpKiBits0614 bit;
};

//***********************************************
//		KP_KI REMAP register - address 0x21
//***********************************************
struct KpKiRmBits0614
{
    uint16_t ls: 1;                     //     0    Low Side switch control
    uint16_t hs: 1;                     //     1    High Side switch control
    uint16_t ic: 1;                     //     2    Turn on/off current source
    uint16_t adcEn: 1;                  //     3    Turn on/off HV feedback ADC
    uint16_t enfb: 1;                   //     4    Turn on/off HV feedback amplifier
    uint16_t cp5: 1;                    //     5    Turn on/off CP5
    uint16_t cp10: 1;                   //     6    Turn on/off CP10
    uint16_t uvlo: 1;                   //     7    Turn on/off UVLO comparators
    uint16_t temp: 1;                   //     8    Turn on/off temp sensor, current, zvs, UniPower comparator
    uint16_t uni: 1;                    //     9    Controls UniPower switch
    uint16_t adcEnSense: 1;             //    10    Turn on/off sense feedback Adc
    uint16_t senseAmp: 1;               //    11    Turn on/off sense amplifier
    uint16_t maxSense: 2;               // 13:12    Select which channel is sensed
    const uint16_t reserved: 2;         // 15:14    Reserved
};

union KpKiRmReg0614
{
    Bos0614Register reg;
    struct KpKiRmBits0614 bit;
};

//***********************************************
//	 DEADTIME register - address 0x22 - REV_B
//***********************************************
struct DeadTimeBits0614RevB
{
    uint16_t dls: 5;                    //   4:0    Power switch deadtime (low side)
    uint16_t dhs: 7;                    //  11:5    Power switch deadtime (high side)
    const uint16_t reserved: 4;         // 15:12    Reserved
};

union DeadTimeReg0614RevB
{
    Bos0614Register reg;
    struct DeadTimeBits0614RevB bit;
};

//***********************************************
//	 DEADTIME register - address 0x22 - REV_C
//***********************************************
struct DeadTimeBits0614RevC
{
    uint16_t dls: 5;                    //   4:0    Power switch deadtime (low side)
    uint16_t dhs: 7;                    //  11:5    Power switch deadtime (high side)
    uint16_t adSense0: 1;               //    12    Compensate charges sinked by the IC, to simulate a higher impedance device
    uint16_t adSense1: 1;               //    13    Takes into account signals that have a valid slope +/- 4.4 mv/ms for at least 10 ms
    uint16_t adSense2: 1;               //    14    Enable extrapolation of the points between segments
    uint16_t adSense3: 1;               //    15    Enable to stitch together segment of the sensed signal
};

union DeadTimeReg0614RevC
{
    Bos0614Register reg;
    struct DeadTimeBits0614RevC bit;
};

//***********************************************
//	 DEADTIME register - address 0x22 - Common
//***********************************************
struct DeadTimeBits0614Common
{
    uint16_t dls: 5;                    //   4:0    Power switch deadtime (low side)
    uint16_t dhs: 7;                    //  11:5    Power switch deadtime (high side)
    const uint16_t reserved: 4;         // 15:12    Reserved
};

union DeadTimeReg0614Common
{
    Bos0614Register reg;
    struct DeadTimeBits0614Common bit;
};

//***********************************************
//		DEADTIME REMAP register - address 0x22
//***********************************************
struct DeadTimeRmBits0614
{
    uint16_t dac_value: 9;              //   8:0    Values of output current of the DAC
    uint16_t s: 1;                      //     9    Sign bit of the current DAC
    const uint16_t reserved: 6;         // 15:10    Reserved
};

union DeadTimeRmReg0614
{
    Bos0614Register reg;
    struct DeadTimeRmBits0614 bit;
};

//***********************************************
//		PARCAP register - address 0x23
//***********************************************
struct ParcapBits0614
{
    uint16_t IONSCALE: 8;               //   7:0    Calculated parasitic capacitance
    uint16_t PARCAP: 8;                 //  15:8    Low Side Gate Strength
};

union ParcapReg0614
{
    Bos0614Register reg;
    struct ParcapBits0614 bit;
};

//***********************************************
//		PARCAP REMAP register - address 0x23
//***********************************************
struct ParcapRmBits0614
{
    uint16_t lsOut: 4;                  //   3:0    Set the low side switch of each channel
    const uint16_t reserved: 12;        //  15:4    Reserved
};

union ParcapRmReg0614
{
    Bos0614Register reg;
    struct ParcapRmBits0614 bit;
};

//***********************************************
//		SUP_RISE register - address 0x24 - REV_C
//***********************************************
struct SupRiseBits0614RevB
{
    uint16_t tiRise: 6;                 //   5:0    Calculated proportional gain for the offset
    uint16_t vin: 5;                    //  10:6    Calculated digital representation of the supply voltage
    uint16_t ib: 1;                     //    11    Current boundary to Ion
    uint16_t cp5: 1;                    //    12    5V pump
    uint16_t ofs: 1;                    //    13    Offset generation
    const uint16_t reserved: 2;         // 15:14    Reserved
};

union SupRiseReg0614RevB
{
    Bos0614Register reg;
    struct SupRiseBits0614RevB bit;
};

//***********************************************
//		SUP_RISE register - address 0x24 - REV_C
//***********************************************
struct SupRiseBits0614RevC
{
    uint16_t tiRise: 6;                 //   5:0    Calculated proportional gain for the offset
    uint16_t vin: 5;                    //  10:6    Calculated digital representation of the supply voltage
    uint16_t lp: 1;                     //    11    Decides if analog runs in low power mode when sensing
    uint16_t cp5: 1;                    //    12    5V pump
    uint16_t ofs: 1;                    //    13    Offset generation
    const uint16_t reserved: 2;         // 15:14    Reserved
};

union SupRiseReg0614RevC
{
    Bos0614Register reg;
    struct SupRiseBits0614RevC bit;
};

//***********************************************
//		SUP_RISE register - address 0x24 - Common
//***********************************************
struct SupRiseBits0614Common
{
    uint16_t tiRise: 6;                 //   5:0    Calculated proportional gain for the offset
    uint16_t vin: 5;                    //  10:6    Calculated digital representation of the supply voltage
    const uint16_t reserved0: 1;        //    11    Reserved
    uint16_t cp5: 1;                    //    12    5V pump
    uint16_t ofs: 1;                    //    13    Offset generation
    const uint16_t reserved1: 2;        // 15:14    Reserved
};

union SupRiseReg0614Common
{
    Bos0614Register reg;
    struct SupRiseBits0614Common bit;
};

//***********************************************
//		TRIM register - address 0x25
//***********************************************
struct TrimBits0614
{
    uint16_t trimReg: 3;                //   2:0    1.8V regulator trimming bits in 2's complement
    uint16_t trimOsc: 7;                //   9:3    Oscillator trimming bits in 2's complement
    uint16_t opt0: 1;                   //    10    Option number of channel 1=2 0=4
    uint16_t opt1: 1;                   //    11    Option sensing enabled
    uint16_t opt2: 1;                   //    12    Option WFS enabled
    uint16_t opt3: 1;                   //    13    Option SPI or I3C
    uint16_t trimW: 2;                  // 15:14    On demand trim control
};

union TrimReg0614
{
    Bos0614Register reg;
    struct TrimBits0614 bit;
};

//***********************************************
//		CHIP_ID register - address 0x26
//***********************************************
struct ChipIdBits0614
{
    uint16_t chipId: 16;                //   15:0   REV A = 0  error
};

union ChipIdReg0614
{
    Bos0614Register reg;
    struct ChipIdBits0614 bit;
};

//***********************************************
//		VFEEDBACK register - address 0x28
//***********************************************
struct VFeedbackBits0614
{
    uint16_t vFeedback: 10;             //   9:0    Voltage feedback
    uint16_t refChReg: 4;               // 13:10    State of the four channel
    const uint16_t reserved: 2;         // 15:14    Reserved
};

union VFeedbackReg0614
{
    Bos0614Register reg;
    struct VFeedbackBits0614 bit;
};

//***********************************************
//		FIFO_STATE register - address 0x29
//***********************************************
struct FifoStateBits0614
{
    uint16_t fifoSpace: 10;             //   9:0    Space available
    uint16_t empty: 1;                  //    10    FIFO is empty
    uint16_t full: 1;                   //    11    FIFO is full
    uint16_t error: 1;                  //    12    Error, any type
    const uint16_t reserved: 3;         // 15:13    Reserved
};

union FifoStateReg0614
{
    Bos0614Register reg;
    struct FifoStateBits0614 bit;
};

//***********************************************
//		AUTO_STATE register - address 0x2A
//***********************************************
struct AutoStateBits0614
{
    uint16_t playChannels: 4;           //   3:0    State of the 4 channels
    uint16_t wave: 3;                   //   6:4    Waveform ID being played
    uint16_t rqsPlay: 1;                //     7    Request to play triggered
    uint16_t pressRelease: 4;           //  11:8    State of sense channel x
    const uint16_t reserved: 4;         // 15:12    Reserved

};

union AutoStateReg0614
{
    Bos0614Register reg;
    struct AutoStateBits0614 bit;
};

//***********************************************
//	RAM_DATA_RAW register - address 0x44 (Hidden) - REV_B
//***********************************************
//***********************************************
//	RAM_DATA register - address 0x2B - REV_C
//***********************************************
struct RamDataBits0614
{
    uint16_t ramData: 16;               // 16:0     Reserved
};

union RamDataReg0614
{
    Bos0614Register reg;
    struct RamDataBits0614 bit;
};

//***********************************************
//	SENS_OFFSET register - address 0x39 (Hidden) - REV_B
//***********************************************
//***********************************************
//	SENSE_OFFSET register - address 0x2C - REV_C
//***********************************************
struct SenseOffsetBits0614
{
    uint16_t senseOffset: 9;            //   8:0    Sense feedback path offset
    const uint16_t reserved: 7;         //  15:9    Reserved
};

union SenseOffsetReg0614
{
    Bos0614Register reg;
    struct SenseOffsetBits0614 bit;
};

//***********************************************
//		BIST register - address 0x30 (Hidden)
//***********************************************
struct BistBits0614
{
    uint16_t romAddr: 2;                //    1:0   ROM results address
    uint16_t romResult: 1;              //      2   ROM result mode select
    uint16_t romBistDone: 1;            //      3   Bist done
    uint16_t romBistStart: 1;           //      4   Start ROM BIST
    uint16_t ramBistFailed: 1;          //      5   RAM BIST FAILED
    uint16_t ramDone: 1;                //      6   RAM BIST Completed
    uint16_t ramStart: 1;               //      7   Start RAM BIST
    const uint16_t reserved: 8;         //   15:8   Reserved
};

union BistReg0614
{
    Bos0614Register reg;
    struct BistBits0614 bit;
};

//***********************************************
//		BISTRES register - address 0x31 (Hidden)
//***********************************************
struct BistResBits0614
{
    uint16_t result: 12;                //    11:0  ROM BIST signature
};

union BistResReg0614
{
    Bos0614Register reg;
    struct BistResBits0614 bit;
};

//***********************************************
//	DEBUG register - address 0x32 (Hidden)  - REV_B
//***********************************************
struct DebugBits0614RevB
{
    uint16_t db: 1;                     //      0   IC in debug mode
    uint16_t ean: 3;                    //    3:1   Assign Internal node to TA
    uint16_t zps: 1;                    //      4   Keep ZPS active
    uint16_t gpio0M: 1;                 //      5   MSB settings of GPIO
    uint16_t gpio1M: 1;                 //      6   MSB settings of GPIO
    uint16_t gpio2M: 1;                 //      7   MSB settings of GPIO
    uint16_t gpio3M: 1;                 //      8   MSB settings of GPIO
    const uint16_t reserved: 1;         //      9   Reserved
    uint16_t access: 6;                 //  15:10   Enable/Disable Debug mode   0x3A pour mettre en mode cache  11 1010

};

union DebugReg0614RevB
{
    Bos0614Register reg;
    struct DebugBits0614RevB bit;
};

//***********************************************
//	DEBUG register - address 0x32 (Hidden) - REV_C
//***********************************************
struct DebugBits0614RevC
{
    uint16_t db: 1;                     //      0   IC in debug mode
    uint16_t ean: 3;                    //    3:1   Assign Internal node to TA
    uint16_t zps: 1;                    //      4   Keep ZPS active
    uint16_t gpio0M: 1;                 //      5   MSB settings of GPIO
    uint16_t gpio1M: 1;                 //      6   MSB settings of GPIO
    uint16_t gpio2M: 1;                 //      7   MSB settings of GPIO
    uint16_t gpio3M: 1;                 //      8   MSB settings of GPIO
    uint16_t diff: 1;                   //      9   Active shutdown of sensing amplifier when it is not needed to sense
    uint16_t access: 6;                 //  15:10   Enable/Disable Debug mode   0x3A pour mettre en mode cache  11 1010

};

union DebugReg0614RevC
{
    Bos0614Register reg;
    struct DebugBits0614RevC bit;
};

//***********************************************
//	DEBUG register - address 0x32 (Hidden) - Common
//***********************************************
struct DebugBits0614Common
{
    uint16_t db: 1;                     //      0   IC in debug mode
    uint16_t ean: 3;                    //    3:1   Assign Internal node to TA
    uint16_t zps: 1;                    //      4   Keep ZPS active
    uint16_t gpio0M: 1;                 //      5   MSB settings of GPIO
    uint16_t gpio1M: 1;                 //      6   MSB settings of GPIO
    uint16_t gpio2M: 1;                 //      7   MSB settings of GPIO
    uint16_t gpio3M: 1;                 //      8   MSB settings of GPIO
    const uint16_t reserved: 1;         //      9   Reserved
    uint16_t access: 6;                 //  15:10   Enable/Disable Debug mode   0x3A pour mettre en mode cache  11 1010

};

union DebugReg0614Common
{
    Bos0614Register reg;
    struct DebugBits0614Common bit;
};

//***********************************************
//	THRESH register - address 0x33 (Hidden) - REV_B
//***********************************************
struct ThreshBits0614RevB
{
    uint16_t vThresh: 7;                //   6:0    Internal parameter
    uint16_t threshError: 5;            //  11:7
    uint16_t trimMode: 2;               // 13:12    Trimming operation mode
    uint16_t slp: 1;                    //    14    Turn off ZVS and ADC
    uint16_t vddh: 1;                   //    15    Charges through active switch
};

union ThreshReg0614RevB
{
    Bos0614Register reg;
    struct ThreshBits0614RevB bit;
};

//***********************************************
//	THRESH register - address 0x33 (Hidden) - REV_C
//***********************************************
struct ThreshBits0614RevC
{
    uint16_t vThresh: 7;                //   6:0    Internal parameter
    uint16_t threshError: 5;            //  11:7
    uint16_t trimMode: 2;               // 13:12    Trimming operation mode
    const uint16_t reserved: 1;         //    14    Reserved
    uint16_t vddh: 1;                   //    15    Charges through active switch
};

union ThreshReg0614RevC
{
    Bos0614Register reg;
    struct ThreshBits0614RevC bit;
};

//***********************************************
//	THRESH register - address 0x33 (Hidden) - Common
//***********************************************
struct ThreshBits0614Common
{
    uint16_t vThresh: 7;                //   6:0    Internal parameter
    uint16_t threshError: 5;            //  11:7
    uint16_t trimMode: 2;               // 13:12    Trimming operation mode
    const uint16_t reserved: 1;         //    14    Reserved
    uint16_t vddh: 1;                   //    15    Charges through active switch
};

union ThreshReg0614Common
{
    Bos0614Register reg;
    struct ThreshBits0614Common bit;
};

//***********************************************
//		REG38 register - address 0x38 (Hidden)
//***********************************************
struct Reg38Bits0614
{
    uint16_t hvOffset: 10;              //   9:0    Offset of the main ADC
    const uint16_t reserved: 6;         // 15:10    Reserved
};

union Reg38Reg0614
{
    Bos0614Register reg;
    struct Reg38Bits0614 bit;
};


//***********************************************
//		CALIB_DATA register - address 0x3A (Hidden)
//***********************************************
struct CalibDataBits0614
{
    uint16_t dacLs: 6;                  //    5:0   Dac Calibration LS
    uint16_t dacHs: 6;                  //   11:6   Dac Calibration HS
    const uint16_t reserved: 4;         //  15:12   Reserved
};

union CalibDataReg0614
{
    Bos0614Register reg;
    struct CalibDataBits0614 bit;
};

//***********************************************
//		REG3B register - address 0x3B (Hidden)
//***********************************************
struct Reg3BBits0614
{
    uint16_t err: 9;                    //    8:0   Sense signal ???
    uint16_t refCh: 4;                  //   12:9   State of the four channel
    uint16_t ccm: 1;                    //     13   Driver in CCM or DCM mode
    uint16_t dcm: 1;                    //     14   Switching controller
    uint16_t maxPower: 1;               //     15   Max Power
};

union Reg3BReg0614
{
    Bos0614Register reg;
    struct Reg3BBits0614 bit;
};

//***********************************************
//		REG3C register - address 0x3C (Hidden)
//***********************************************
struct Reg3CBits0614
{
    uint16_t currentAdc: 9;                //    8:0   Current sent to the ADC mA
    uint16_t s: 1;                      //      9   Sign for ADC
    uint16_t ccm: 1;                    //     10   Driver operating mode
    uint16_t dcm: 1;                    //     11   Switch Controller
    uint16_t ovv: 1;                    //     12   Over Voltage Bit
    uint16_t overState: 1;              //     13   Converter Status
    uint16_t mode: 2;                   //  15:14   Driver current operating mode
};

union Reg3CReg0614
{
    Bos0614Register reg;
    struct Reg3CBits0614 bit;
};

//***********************************************
//		REG3D register - address 0x3D (Hidden)
//***********************************************
struct Reg3DBits0614
{
    uint16_t minCurrent: 10;            //    9:0   Minimum current to turn on LS
    uint16_t ccm: 1;                    //     10   Driver operating mode
    uint16_t dcm: 1;                    //     11   Switch Controller
    uint16_t ovv: 1;                    //     12   Over Voltage Bit
    uint16_t overState: 1;              //     13   Converter Status
    uint16_t mode: 2;                   //  15:14   Driver current operating mode
};

union Reg3DReg0614
{
    Bos0614Register reg;
    struct Reg3DBits0614 bit;
};

//***********************************************
//		REG3E register - address 0x3E (Hidden)
//***********************************************
struct Reg3EBits0614
{
    uint16_t err: 9;                    //    8:0   Error between reference and feedback
    uint16_t pulseCtl: 7;               //   15:9   Switch pulse length
};

union Reg3EReg0614
{
    Bos0614Register reg;
    struct Reg3EBits0614 bit;
};

//***********************************************
//		REG3F register - address 0x3F (Hidden)
//***********************************************
struct Reg3FBits0614
{
    uint16_t blankCtl: 7;               //    6:0   Blanking time delay
    uint16_t deadCtl: 7;                //   13:7   Dead time delay
    uint16_t mode: 2;                   //  15:14   Current operating mode
};

union Reg3FReg0614
{
    Bos0614Register reg;
    struct Reg3FBits0614 bit;
};

//***********************************************
//		REG40 register - address 0x40 (Hidden)
//***********************************************
struct Reg40Bits0614
{
    uint16_t refReg: 12;                //   11:0   Desired amplitude on output
    uint16_t refChReg: 4;               //  15:12   State of the four channel
};

union Reg40Reg0614
{
    Bos0614Register reg;
    struct Reg40Bits0614 bit;
};

//***********************************************
//		REG41 register - address 0x41 (Hidden)
//***********************************************
struct Reg41Bits0614
{
    uint16_t refReg: 12;                //   11:0   Desired amplitude on output
    uint16_t empty: 1;                  //     12   FIFO state
    uint16_t full: 1;                   //     13   FIFO state
    const uint16_t reserved: 2;         //  15:14   Reserved
};

union Reg41Reg0614
{
    Bos0614Register reg;
    struct Reg41Bits0614 bit;
};

//***********************************************
//		REG42 register - address 0x42 (Hidden)
//***********************************************
struct Reg42Bits0614
{
    uint16_t err: 9;                    //    8:0   Error between reference and feedback
    uint16_t lout: 4;                   //   12:9   Desired amplitude on output
    uint16_t ccm: 1;                    //     10   Driver operating mode
    uint16_t dcm: 1;                    //     11   Switch Controller
    uint16_t maxPower: 1;               //     13   Maximum power status
};

union Reg42Reg0614
{
    Bos0614Register reg;
    struct Reg42Bits0614 bit;
};

//***********************************************
//		REG43 register - address 0x43 (Hidden)
//***********************************************
struct Reg43Bits0614
{
    uint16_t periode: 10;               //    9:0   Error between reference and feedback
    uint16_t zone: 3;                   //  12:10   Desired amplitude on output
    const uint16_t reserved: 1;         //     13   Reserved
    uint16_t mode: 2;                   //  15:14   Driver operating mode
};

union Reg43Reg0614
{
    Bos0614Register reg;
    struct Reg43Bits0614 bit;
};

//***********************************************
//		REG45 register - address 0x45 (Hidden)
//***********************************************
struct Reg45Bits0614
{
    uint16_t err: 9;                    //    8:0   Error between reference and feedback
    uint16_t stmState: 2;               //   10:9   Main State machine State
    uint16_t uvlo: 1;                   //     11   Under Voltage bit
    uint16_t ovv: 1;                    //     12   Over Voltage bit
    uint16_t overState: 1;              //     13   Reason why the converter stopped
    uint16_t mode: 2;                   //  15:14   Current operating mode
};

union Reg45Reg0614
{
    Bos0614Register reg;
    struct Reg45Bits0614 bit;
};

//***********************************************
//		REG46 register - address 0x46 (Hidden)
//***********************************************
struct Reg46Bits0614
{
    uint16_t currentPi: 10;             //    9:0   Desired current from PI controller
    uint16_t ccm: 1;                    //     10   Driver operating mode
    uint16_t ccmState: 2;               //  12:11   Approximate ripple
    uint16_t flagCcmState: 2;           //  14:13   CCM State control
    uint16_t dcm: 1;                    //     15   Switch controller
};

union Reg46Reg0614
{
    Bos0614Register reg;
    struct Reg46Bits0614 bit;
};

//***********************************************
//		REG47 register - address 0x47 (Hidden)
//***********************************************
struct Reg47Bits0614
{
    uint16_t periode: 10;               //    9:0   Switching period in clk cycles
    uint16_t ccm: 1;                    //     10   Driver operating mode
    uint16_t ccmState: 2;               //  12:11   Approximate ripple
    uint16_t flagCcmState: 2;           //  14:13   CCM State control
    uint16_t dcm: 1;                    //     15   Switch controller
};

union Reg47Reg0614
{
    Bos0614Register reg;
    struct Reg47Bits0614 bit;
};

//***********************************************
//		Common registers structure
//***********************************************
typedef struct
{
    union ReferenceReg0614 reference0614;
    union ReferenceRmReg0614 referenceRm0614;
    union IcStatusReg0614 icStatus0614;
    union ReadReg0614 read0614;
    union GpioxReg0614 gpiox0614;
    UndefReg tc0614;
    union ConfigReg0614 config0614;
    union SenseConfigReg0614Common senseConfig0614;
    union Sense0Reg0614 sense00614;
    union Sense0PReg0614 sense0P0614;
    union Sense0RReg0614 sense0R0614;
    union Sense0SReg0614 sense0S0614;
    union Sense1Reg0614 sense10614;
    union Sense1PReg0614 sense1P0614;
    union Sense1RReg0614 sense1R0614;
    union Sense1SReg0614 sense1S0614;
    union Sense2Reg0614 sense20614;
    union Sense2PReg0614 sense2P0614;
    union Sense2RReg0614 sense2R0614;
    union Sense2SReg0614 sense2S0614;
    union Sense3Reg0614 sense30614;
    union Sense3PReg0614 sense3P0614;
    union Sense3RReg0614 sense3R0614;
    union Sense3SReg0614 sense3S0614;
    union SenseStatusReg0614 senseStatus0614;
    union SenseData0Reg0614 senseData00614;
    union SenseData1Reg0614 senseData10614;
    union SenseData2Reg0614 senseData20614;
    union SenseData3Reg0614 senseData30614;
    union SenseRaw0Reg0614 senseRaw00614;
    union SenseRaw1Reg0614 senseRaw10614;
    union SenseRaw2Reg0614 senseRaw20614;
    union SenseRaw3Reg0614 senseRaw30614;
    union KpaReg0614 kpa0614;
    union KpKiReg0614 kpKi0614;
    union KpKiRmReg0614 kpKiRm0614;
    union DeadTimeReg0614Common deadTime0614;
    union DeadTimeRmReg0614 deadTimeRm0614;
    union ParcapReg0614 parcap0614;
    union ParcapRmReg0614 parcapRm0614;
    union SupRiseReg0614Common supRise0614;
    union TrimReg0614 trim0614;
    union ChipIdReg0614 chipId0614;
    union VFeedbackReg0614 vFeedback0614;
    union FifoStateReg0614 fifoState0614;
    union AutoStateReg0614 autoState0614;
    union BistReg0614 bist0614;
    union BistResReg0614 bistRes0614;
    union DebugReg0614Common debug0614;
    union ThreshReg0614Common thresh0614;
    union Reg38Reg0614 reg380614;
    union SenseOffsetReg0614 senseOffset0614;
    union CalibDataReg0614 calibData0614;
    union Reg3BReg0614 reg3B0614;
    union Reg3CReg0614 reg3C0614;
    union Reg3DReg0614 reg3D0614;
    union Reg3EReg0614 reg3E0614;
    union Reg3FReg0614 reg3F0614;
    union Reg40Reg0614 reg400614;
    union Reg41Reg0614 reg410614;
    union Reg42Reg0614 reg420614;
    union Reg43Reg0614 reg430614;
    union RamDataReg0614 ramData0614;
    union Reg45Reg0614 reg450614;
    union Reg46Reg0614 reg460614;
    union Reg47Reg0614 reg470614;
} Bos0614CommonRegister;

//***********************************************
//		Rev B registers structure
//***********************************************
typedef struct
{
    union ReferenceReg0614 reference0614;
    union ReferenceRmReg0614 referenceRm0614;
    union IcStatusReg0614 icStatus0614;
    union ReadReg0614 read0614;
    union GpioxReg0614 gpiox0614;
    union TcReg0614RevB tc0614;
    union ConfigReg0614 config0614;
    union SenseConfigReg0614RevB senseConfig0614;
    union Sense0Reg0614 sense00614;
    union Sense0PReg0614 sense0P0614;
    union Sense0RReg0614 sense0R0614;
    union Sense0SReg0614 sense0S0614;
    union Sense1Reg0614 sense10614;
    union Sense1PReg0614 sense1P0614;
    union Sense1RReg0614 sense1R0614;
    union Sense1SReg0614 sense1S0614;
    union Sense2Reg0614 sense20614;
    union Sense2PReg0614 sense2P0614;
    union Sense2RReg0614 sense2R0614;
    union Sense2SReg0614 sense2S0614;
    union Sense3Reg0614 sense30614;
    union Sense3PReg0614 sense3P0614;
    union Sense3RReg0614 sense3R0614;
    union Sense3SReg0614 sense3S0614;
    union SenseStatusReg0614 senseStatus0614;
    union SenseData0Reg0614 senseData00614;
    union SenseData1Reg0614 senseData10614;
    union SenseData2Reg0614 senseData20614;
    union SenseData3Reg0614 senseData30614;
    union SenseRaw0Reg0614 senseRaw00614;
    union SenseRaw1Reg0614 senseRaw10614;
    union SenseRaw2Reg0614 senseRaw20614;
    union SenseRaw3Reg0614 senseRaw30614;
    union KpaReg0614 kpa0614;
    union KpKiReg0614 kpKi0614;
    union KpKiRmReg0614 kpKiRm0614;
    union DeadTimeReg0614RevB deadTime0614;
    union DeadTimeRmReg0614 deadTimeRm0614;
    union ParcapReg0614 parcap0614;
    union ParcapRmReg0614 parcapRm0614;
    union SupRiseReg0614RevB supRise0614;
    union TrimReg0614 trim0614;
    union ChipIdReg0614 chipId0614;
    union VFeedbackReg0614 vFeedback0614;
    union FifoStateReg0614 fifoState0614;
    union AutoStateReg0614 autoState0614;
    union BistReg0614 bist0614;
    union BistResReg0614 bistRes0614;
    union DebugReg0614RevB debug0614;
    union ThreshReg0614RevB thresh0614;
    union Reg38Reg0614 reg380614;
    union SenseOffsetReg0614 senseOffset0614;
    union CalibDataReg0614 calibData0614;
    union Reg3BReg0614 reg3B0614;
    union Reg3CReg0614 reg3C0614;
    union Reg3DReg0614 reg3D0614;
    union Reg3EReg0614 reg3E0614;
    union Reg3FReg0614 reg3F0614;
    union Reg40Reg0614 reg400614;
    union Reg41Reg0614 reg410614;
    union Reg42Reg0614 reg420614;
    union Reg43Reg0614 reg430614;
    union RamDataReg0614 ramData0614;
    union Reg45Reg0614 reg450614;
    union Reg46Reg0614 reg460614;
    union Reg47Reg0614 reg470614;
} Bos0614RevBRegisters;

//***********************************************
//		Rev C registers structure
//***********************************************
typedef struct
{
    union ReferenceReg0614 reference0614;
    union ReferenceRmReg0614 referenceRm0614;
    union IcStatusReg0614 icStatus0614;
    union ReadReg0614 read0614;
    union GpioxReg0614 gpiox0614;
    union TcReg0614RevC tc0614;
    union ConfigReg0614 config0614;
    union SenseConfigReg0614RevC senseConfig0614;
    union Sense0Reg0614 sense00614;
    union Sense0PReg0614 sense0P0614;
    union Sense0RReg0614 sense0R0614;
    union Sense0SReg0614 sense0S0614;
    union Sense1Reg0614 sense10614;
    union Sense1PReg0614 sense1P0614;
    union Sense1RReg0614 sense1R0614;
    union Sense1SReg0614 sense1S0614;
    union Sense2Reg0614 sense20614;
    union Sense2PReg0614 sense2P0614;
    union Sense2RReg0614 sense2R0614;
    union Sense2SReg0614 sense2S0614;
    union Sense3Reg0614 sense30614;
    union Sense3PReg0614 sense3P0614;
    union Sense3RReg0614 sense3R0614;
    union Sense3SReg0614 sense3S0614;
    union SenseStatusReg0614 senseStatus0614;
    union SenseData0Reg0614 senseData00614;
    union SenseData1Reg0614 senseData10614;
    union SenseData2Reg0614 senseData20614;
    union SenseData3Reg0614 senseData30614;
    union SenseRaw0Reg0614 senseRaw00614;
    union SenseRaw1Reg0614 senseRaw10614;
    union SenseRaw2Reg0614 senseRaw20614;
    union SenseRaw3Reg0614 senseRaw30614;
    union KpaReg0614 kpa0614;
    union KpKiReg0614 kpKi0614;
    union KpKiRmReg0614 kpKiRm0614;
    union DeadTimeReg0614RevC deadTime0614;
    union DeadTimeRmReg0614 deadTimeRm0614;
    union ParcapReg0614 parcap0614;
    union ParcapRmReg0614 parcapRm0614;
    union SupRiseReg0614RevC supRise0614;
    union TrimReg0614 trim0614;
    union ChipIdReg0614 chipId0614;
    union VFeedbackReg0614 vFeedback0614;
    union FifoStateReg0614 fifoState0614;
    union AutoStateReg0614 autoState0614;
    union BistReg0614 bist0614;
    union BistResReg0614 bistRes0614;
    union DebugReg0614RevC debug0614;
    union ThreshReg0614RevC thresh0614;
    union Reg38Reg0614 reg380614;
    union SenseOffsetReg0614 senseOffset0614;
    union CalibDataReg0614 calibData0614;
    union Reg3BReg0614 reg3B0614;
    union Reg3CReg0614 reg3C0614;
    union Reg3DReg0614 reg3D0614;
    union Reg3EReg0614 reg3E0614;
    union Reg3FReg0614 reg3F0614;
    union Reg40Reg0614 reg400614;
    union Reg41Reg0614 reg410614;
    union Reg42Reg0614 reg420614;
    union Reg43Reg0614 reg430614;
    union RamDataReg0614 ramData0614;
    union Reg45Reg0614 reg450614;
    union Reg46Reg0614 reg460614;
    union Reg47Reg0614 reg470614;
} Bos0614RevCRegisters;

typedef struct
{
    union
    {
        Bos0614CommonRegister common;
        Bos0614RevBRegisters revB;
        Bos0614RevCRegisters revC;
    };
} Bos0614Registers;

#endif /* BOS0614_REGISTER_H_ */
