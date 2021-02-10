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
#define ADDRESS_BOS0614_READ_REG              0x0002
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
#define ADDRESS_BOS0614_BIST_REG             0x0030
#define ADDRESS_BOS0614_BISTRES_REG          0x0031
#define ADDRESS_BOS0614_DEBUG_REG            0x0032
#define ADDRESS_BOS0614_THRESH_REG           0x0033
#define ADDRESS_BOS0614_REG38_REG            0x0038
#define ADDRESS_BOS0614_REG39_REG            0x0039
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
#define ADDRESS_BOS0614_REG44_REG            0x0044
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

//***********************************************
//		REFERENCE register - address 0x00
//***********************************************
struct REFERENCE_BITS_0614
{
    uint16_t FIFO: 12;           //   11:0 Input of the FIFO
    uint16_t CHANNEL: 4;         //  12:15 Channel 3 enable
};

union REFERENCE_REG_0614
{
    Bos0614Register reg;
    struct REFERENCE_BITS_0614 bit;
};

//***********************************************
//		REFERENCE register - address 0x00  REMAP
//***********************************************
struct REFERENCE_RM_BITS_0614
{
    uint16_t DATA: 16;        //  15:0 Input of the RAM communication
};

union REFERENCE_RM_REG_0614
{
    Bos0614Register reg;
    struct REFERENCE_RM_BITS_0614 bit;
};


//***********************************************
//		IC_STATUS register - address 0x01
//***********************************************
struct IC_STATUS_BITS_0614
{
    uint16_t EMPTY: 1;                //   0     FIFO Empty
    uint16_t FULL: 1;                //   1     FIFO Full
    uint16_t SC: 1;                    //   2     Short Circuit Status
    uint16_t UVLO: 1;                //   3     Under Voltage
    uint16_t IDAC: 1;                //   4     Status bit
    uint16_t MAXPOWER: 1;            //   5     Amount of Power
    uint16_t OVT: 1;                    //   6     Over Temperature
    uint16_t OVV: 1;                    //   7 	   Over Voltage
    uint16_t STATE: 2;            //   9:8   State of controller
    uint16_t PRESS_RELEASE: 4;    //   10:13 State of sense channels
    uint16_t SENSEALL: 1;            //   14    Trigger condition met
    const uint16_t reserved: 1;    //   15    Reserved

};

union IC_STATUS_REG_0614
{
    Bos0614Register reg;
    struct IC_STATUS_BITS_0614 bit;
};

//***********************************************
//		READ register - address 0x02
//***********************************************
struct READ_BITS_0614
{
    uint16_t BC: 8;                    //   7:0 Address of internal register whose content is returned on comm bus
    const uint16_t reserved: 8;        //   15:8 Reserved
};

union READ_REG_0614
{
    Bos0614Register reg;
    struct READ_BITS_0614 bit;
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
} Bos0614GPIOMode;

struct GPIOX_BITS_0614
{
    Bos0614GPIOMode GPIO0: 4;                //   3:0    Controls bit of GPIO0
    Bos0614GPIOMode GPIO1: 4;                //   7:4    Controls bit of GPIO1
    Bos0614GPIOMode GPIO2: 4;                //   11:8   Controls bit of GPIO2
    Bos0614GPIOMode GPIO3: 4;                //   15:12  Controls bit of GPIO3
};

union GPIOX_REG_0614
{
    Bos0614Register reg;
    struct GPIOX_BITS_0614 bit;
};


//***********************************************
//		TC register - address 0x04
//***********************************************
struct TC_BITS_0614
{
    uint16_t TC: 5;                    //   4:0    Time to short output
    uint16_t PC: 1;                    //   5
    uint16_t POL: 1;                    //   6      If GPIO sensing
    const uint16_t reserved: 9;        //   15:7 Reserved
};

union TC_REG_0614
{
    Bos0614Register reg;
    struct TC_BITS_0614 bit;
};


//***********************************************
//		CONFIG register - address 0x5
//***********************************************
typedef enum
{
    bos0614SamplingRate_1024Ksps = 0,
    bos0614SamplingRate_512Ksps,
    bos0614SamplingRate_256Ksps,
    bos0614SamplingRate_128Ksps,
    bos0614SamplingRate_64Ksps,
    bos0614SamplingRate_32Ksps,
    bos0614SamplingRate_16Ksps,
    bos0614SamplingRate_8Ksps,
} BOS0614SamplingRate;

typedef enum
{
    bos0614OutputDisabled = 0,
    bos0614OutputEnabled,
} BOS0614OutputOE;


typedef enum
{
    bos0614Mode_Direct = 0x0,
    bos0614Mode_FIFO = 0x1,
    bos0614Mode_RAM_Playback,
    bos0614Mode_RAM_Synthesis,
} BOS0614Mode;

struct CONFIG_BITS_0614
{
    BOS0614SamplingRate PLAY: 3;            //   2:0 Waveform play back speed
    uint16_t DS: 1;                //     3 Power mode when not playing waveforms
    BOS0614OutputOE OE: 1;                    //     4 Enable/Disable waveform playback
    uint16_t LOCK: 1;                //     5 Register lock
    uint16_t RST: 1;                    //     6 Software Reset
    uint16_t UPI: 1;                    //     7 UPI enable
    uint16_t TOUT: 1;                //     8 Auto TimeOut
    BOS0614Mode RAM: 2;                    //  10:9 Ram Behavior
    uint16_t STR: 1;                    //    11 Comm Address counter behavior
    uint16_t SHORT: 2;            // 13:12 Length of time to short
    uint16_t OD: 1;                //    14 GPIOs Open Drain
    uint16_t SC: 1;                //    15 Channel selection

};

union CONFIG_REG_0614
{
    Bos0614Register reg;
    struct CONFIG_BITS_0614 bit;
};

//***********************************************
//		SENSECONFIG register - address 0x06
//***********************************************
struct SENSECONFIG_BITS_0614
{
    uint16_t CH0: 1;                    //     0 CH0 Sensing
    uint16_t CH1: 1;                    //     1 CH1 Sensing
    uint16_t CH2: 1;                    //     2 CH2 Sensing
    uint16_t CH3: 1;                    //     3 CH3 Sensing
    uint16_t CAL: 1;                    //     4 Calibrate sense interface
    uint16_t SAME: 1;                //     5 All sense have the same configuration (Sense0)
    uint16_t NEG: 2;                    //   7:6 Reset point voltage negative
    uint16_t POS: 2;                    //   9:8 Reset point voltage positive
    uint16_t SCOMP: 3;                // 12:10 Discharged on the sense channel
    uint16_t SH: 1;                    //    13 Short Output during playback
    uint16_t DIFF: 1;                //	  14 ShutDown sensing when not needed
    uint16_t EXTTRIG: 1;                //	  15 EXT trigger enable
};

union SENSECONFIG_REG_0614
{
    Bos0614Register reg;
    struct SENSECONFIG_BITS_0614 bit;
};

//***********************************************
//		SENSE0 register - address 0x07
//***********************************************
typedef struct
{
    uint16_t T1: 1;                    //     0 Threshold Sensing
    uint16_t T2: 1;                    //     1 Threshold Sensing
    uint16_t S1: 1;                    //     2 Threshold Sensing
    uint16_t S2: 1;                    //     3 Threshold Sensing
    uint16_t AUTOP: 1;                //     4 Start Playing on senseP
    uint16_t AUTOR: 1;                //     5 Start Playing on senseR
    uint16_t WVP: 3;                    //   8:6 Waveform to play   P
    uint16_t WVR: 3;                    //  11:9 Waveform to play   R
    const uint16_t reserved: 4;        // 15:12 Reserved
} SENSE_BITS_0614;

union SENSE0_REG_0614
{
    Bos0614Register reg;
    SENSE_BITS_0614 bit;
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
    uint16_t THRESHOLD: 12;            //  11:0 Threshold Sensing
    SensingThresholdMode AB: 1;        //    12 Above or Below
    uint16_t REP: 3;                   // 15:13 Time for sense
} SENSE_THRESHOLD;

union SENSE0P_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE0R register - address 0x09
//***********************************************

union SENSE0R_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE0S register - address 0x0A
//***********************************************
typedef struct
{
    uint16_t SLOPE1: 7;                //   6:0 Slope 1 of force
    uint16_t ABS1: 1;                //     1 Above or Below
    uint16_t SLOPE2: 7;                //  14:8 Slope 2 of force
    uint16_t ABS2: 1;                //     1 Above or Below
} SENSES_BITS_0614;

union SENSE0S_REG_0614
{
    Bos0614Register reg;
    SENSES_BITS_0614 bit;
};

//***********************************************
//		SENSE1 register - address 0x0B
//***********************************************

union SENSE1_REG_0614
{
    Bos0614Register reg;
    SENSE_BITS_0614 bit;
};

//***********************************************
//		SENSE1P register - address 0x0C
//***********************************************

union SENSE1P_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE1R register - address 0x0D
//***********************************************

union SENSE1R_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE1S register - address 0x0E
//***********************************************

union SENSE1S_REG_0614
{
    Bos0614Register reg;
    SENSES_BITS_0614 bit;
};

//***********************************************
//		SENSE2 register - address 0x0F
//***********************************************

union SENSE2_REG_0614
{
    Bos0614Register reg;
    SENSE_BITS_0614 bit;
};

//***********************************************
//		SENSE0P register - address 0x10
//***********************************************
union SENSE2P_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE2R register - address 0x11
//***********************************************

union SENSE2R_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE2S register - address 0x12
//***********************************************

union SENSE2S_REG_0614
{
    Bos0614Register reg;
    SENSES_BITS_0614 bit;
};

//***********************************************
//		SENSE3 register - address 0x13
//***********************************************

union SENSE3_REG_0614
{
    Bos0614Register reg;
    SENSE_BITS_0614 bit;
};

//***********************************************
//		SENSE3P register - address 0x14
//***********************************************

union SENSE3P_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE3R register - address 0x15
//***********************************************

union SENSE3R_REG_0614
{
    Bos0614Register reg;
    SENSE_THRESHOLD bit;
};

//***********************************************
//		SENSE3S register - address 0x16
//***********************************************

union SENSE3S_REG_0614
{
    Bos0614Register reg;
    SENSES_BITS_0614 bit;
};

//***********************************************
//		SENSESTATUS register - address 0x17
//***********************************************
struct SENSESTATUS_BITS_0614
{
    uint16_t T10: 1;                //     1 Comparator status press threshold ch0
    uint16_t T20: 1;                //     1 Comparator status release threshold ch0
    uint16_t S10: 1;                //     1 Comparator status slope 1 ch0
    uint16_t S20: 1;                //     1 Comparator status slope 2 ch0
    uint16_t T11: 1;                //     1 Comparator status press threshold ch1
    uint16_t T21: 1;                //     1 Comparator status release threshold ch1
    uint16_t S11: 1;                //     1 Comparator status slope 1 ch1
    uint16_t S21: 1;                //     1 Comparator status slope 2 ch1
    uint16_t T12: 1;                //     1 Comparator status press threshold ch2
    uint16_t T22: 1;                //     1 Comparator status release threshold ch2
    uint16_t S12: 1;                //     1 Comparator status slope 1 ch2
    uint16_t S22: 1;                //     1 Comparator status slope 2 ch2
    uint16_t T13: 1;                //     1 Comparator status press threshold ch3
    uint16_t T23: 1;                //     1 Comparator status release threshold ch3
    uint16_t S13: 1;                //     1 Comparator status slope 1 ch3
    uint16_t S23: 1;                //     1 Comparator status slope 2 ch3
};

union SENSESTATUS_REG_0614
{
    Bos0614Register reg;
    struct SENSESTATUS_BITS_0614 bit;
};

//***********************************************
//		SENSEDATA0 register - address 0x18
//***********************************************
struct SENSEDATA_BITS_0614
{
    uint16_t DATA: 16;                //     16 Sensing data
};

union SENSEDATA0_REG_0614
{
    Bos0614Register reg;
    struct SENSEDATA_BITS_0614 bit;
};

//***********************************************
//		SENSEDATA1 register - address 0x19
//***********************************************

union SENSEDATA1_REG_0614
{
    Bos0614Register reg;
    struct SENSEDATA_BITS_0614 bit;
};

//***********************************************
//		SENSEDATA2 register - address 0x1A
//***********************************************

union SENSEDATA2_REG_0614
{
    Bos0614Register reg;
    struct SENSEDATA_BITS_0614 bit;
};

//***********************************************
//		SENSEDATA3 register - address 0x1B
//***********************************************

union SENSEDATA3_REG_0614
{
    Bos0614Register reg;
    struct SENSEDATA_BITS_0614 bit;
};

//***********************************************
//		SENSERAW0 register - address 0x1C
//***********************************************
struct SENSERAW_BITS_0614
{
    uint16_t DATA: 10;                //   9:0 Sensing data
    const uint16_t reserved: 6;        // 15:10 Reserved

};

union SENSERAW0_REG_0614
{
    Bos0614Register reg;
    struct SENSERAW_BITS_0614 bit;
};

//***********************************************
//		SENSERAW1 register - address 0x1D
//***********************************************

union SENSERAW1_REG_0614
{
    Bos0614Register reg;
    struct SENSERAW_BITS_0614 bit;
};

//***********************************************
//		SENSERAW2 register - address 0x1E
//***********************************************

union SENSERAW2_REG_0614
{
    Bos0614Register reg;
    struct SENSERAW_BITS_0614 bit;
};

//***********************************************
//		SENSERAW3 register - address 0x1F
//***********************************************

union SENSERAW3_REG_0614
{
    Bos0614Register reg;
    struct SENSERAW_BITS_0614 bit;
};

//***********************************************
//		KPA register - address 0x20
//***********************************************
struct KPA_BITS_0614
{
    uint16_t KPA: 8;                //   7:0 Internal Parameter
    uint16_t FSWMAX: 2;                //   9:8 Boost converter max frequency
    uint16_t SB: 2;                    // 11:10 Boost converter blanking time
    const uint16_t reserved: 4;        // 15:12 Reserved
};

union KPA_REG_0614
{
    Bos0614Register reg;
    struct KPA_BITS_0614 bit;
};

//***********************************************
//		KP_KI register - address 0x21
//***********************************************
struct KP_KI_BITS_0614
{
    uint16_t KP: 11;                //  10:0 Internal Parameter
    uint16_t KIBASE: 4;                // 14:11 Internal Parameter
    const uint16_t reserved: 1;        //    15 Reserved
};

union KP_KI_REG_0614
{
    Bos0614Register reg;
    struct KP_KI_BITS_0614 bit;
};

//***********************************************
//		KP_KI REMAP register - address 0x21
//***********************************************
struct KP_KI_RM_BITS_0614
{
    uint16_t LS: 1;                //     0 Low Side switch control
    uint16_t HS: 1;                    //     1 High Side switch control
    uint16_t IC: 1;                //     2 Turn on/off current source
    uint16_t ADC_EN: 1;                //     3 Turn on/off HV feedback ADC
    uint16_t ENFB: 1;                //     4 Turn on/off HV feedback amplifier
    uint16_t CP5: 1;                    //     5 Turn on/off CP5
    uint16_t CP10: 1;                //     6 Turn on/off CP10
    uint16_t UVLO: 1;                //     7 Turn on/off UVLO comparators
    uint16_t TEMP: 1;                //     8 Turn on/off temp sensor, current, zvs, UniPower comparator
    uint16_t UNI: 1;                    //     9 Controls UniPower switch
    uint16_t ADC_EN_SENSE: 1;        //    10 Turn on/off sense feedback Adc
    uint16_t SENSE_AMP: 1;           //    11 Turn on/off sense amplifier
    uint16_t MUX_SENSE: 2;           // 13:12 Select which channel is sensed
    const uint16_t reserved: 2;        // 15:14 Reserved
};

union KP_KI_RM_REG_0614
{
    Bos0614Register reg;
    struct KP_KI_RM_BITS_0614 bit;
};

//***********************************************
//		DEADTIME register - address 0x22
//***********************************************
struct DEADTIME_BITS_0614
{
    uint16_t DLS: 5;                    //   4:0 Power switch deadtime (low side)
    uint16_t DHS: 7;                    //  11:5 Power switch deadtime (high side)
    const uint16_t reserved: 4;        // 15:12 Reserved
};

union DEADTIME_REG_0614
{
    Bos0614Register reg;
    struct DEADTIME_BITS_0614 bit;
};

//***********************************************
//		DEADTIME REMAP register - address 0x22
//***********************************************
struct DEADTIME_RM_BITS_0614
{
    uint16_t DAC: 9;                    //   8:0 Values of output current of the DAC
    uint16_t S: 1;                    //     9 Sign bit of the current DAC
    const uint16_t reserved: 6;        // 15:10 Reserved
};

union DEADTIME_RM_REG_0614
{
    Bos0614Register reg;
    struct DEADTIME_RM_BITS_0614 bit;
};

//***********************************************
//		PARCAP register - address 0x23
//***********************************************
struct PARCAP_BITS_0614
{
    uint16_t IONSCALE: 8;            //   7:0 Calculated parasitic capacitance
    uint16_t PARCAP: 8;                //  15:8 Low Side Gate Strength
};

union PARCAP_REG_0614
{
    Bos0614Register reg;
    struct PARCAP_BITS_0614 bit;
};

//***********************************************
//		PARCAP REMAP register - address 0x23
//***********************************************
struct PARCAP_RM_BITS_0614
{
    uint16_t LS_OUT: 4;                //   3:0 Set the low side switch of each channel
    const uint16_t reserved: 12;        //  15:4 Reserved
};

union PARCAP_RM_REG_0614
{
    Bos0614Register reg;
    struct PARCAP_RM_BITS_0614 bit;
};

//***********************************************
//		SUP_RISE register - address 0x24
//***********************************************
struct SUP_RISE_BITS_0614
{
    uint16_t TI_RISE: 6;                //   5:0 Calculated proportional gain for the offset
    uint16_t VIN: 5;                    //  10:6 Calculated digital representation of the supply voltage
    uint16_t IB: 1;                    //    11 Current boundary to Ion
    uint16_t CP5: 1;                    //    12 5V pump
    uint16_t OFS: 1;                    //    13 Offset generation
    const uint16_t reserved: 2;        // 15:14 Reserved
};

union SUP_RISE_REG_0614
{
    Bos0614Register reg;
    struct SUP_RISE_BITS_0614 bit;
};

//***********************************************
//		TRIM register - address 0x25
//***********************************************
struct TRIM_BITS_0614
{
    uint16_t TRIM_REG: 3;            //   2:0 1.8V regulator trimming bits in 2's complement
    uint16_t TRIM_OSC: 7;            //   9:3 Oscillator trimming bits in 2's complement
    uint16_t OPT0: 1;                //    10 Option number of channel 1=2 0=4
    uint16_t OPT1: 1;                //    11 Option sensing enabled
    uint16_t OPT2: 1;                //    12 OPtion WFS enabled
    uint16_t OPT3: 1;                //    13 Option SPI or I3C
    uint16_t TRIMRW: 2;                // 15:14 On demand trim control
};

union TRIM_REG_0614
{
    Bos0614Register reg;
    struct TRIM_BITS_0614 bit;
};

//***********************************************
//		CHIP_ID register - address 0x26
//***********************************************
struct CHIP_ID_BITS_0614
{
    uint16_t CHIPID: 16;        //   15:0 REV A = 0  error
};

union CHIP_ID_REG_0614
{
    Bos0614Register reg;
    struct CHIP_ID_BITS_0614 bit;
};

//***********************************************
//		VFEEDBACK register - address 0x28
//***********************************************
struct VFEEDBACK_BITS_0614
{
    uint16_t VFEEDBACK: 10;            //   9:0 Voltage feedback
    uint16_t REF_CH_REG: 4;            // 13:10 State of the four channel
    const uint16_t reserved: 2;        // 15:14 Reserved
};

union VFEEDBACK_REG_0614
{
    Bos0614Register reg;
    struct VFEEDBACK_BITS_0614 bit;
};

//***********************************************
//		FIFO_STATE register - address 0x29
//***********************************************
struct FIFO_STATE_BITS_0614
{
    uint16_t FIFO_SPACE: 10;           //   9:0 Space available
    uint16_t EMPTY: 1;                 //    10 FIFO is empty
    uint16_t FULL: 1;                  //    11 FIFO is full
    uint16_t ERROR: 1;                 //    12 Error, any type
    const uint16_t reserved: 3;        // 15:13 Reserved
};

union FIFO_STATE_REG_0614
{
    Bos0614Register reg;
    struct FIFO_STATE_BITS_0614 bit;
};

//***********************************************
//		AUTO_STATE register - address 0x2A
//***********************************************
struct AUTO_STATE_BITS_0614
{
    uint16_t PLAY_CHANNELS: 4;          //   3:0 State of the 4 channels
    uint16_t WAVE: 3;                   //   6:4 Waveform ID being played
    uint16_t RQS_PLAY: 1;               //     7 Request to play triggered
    uint16_t PRESS_RELEASE: 4;          //  11:8 State of sense channel x
    const uint16_t reserved: 4;         // 15:12 Reserved

};

union AUTO_STATE_REG_0614
{
    Bos0614Register reg;
    struct AUTO_STATE_BITS_0614 bit;
};

//***********************************************
//		BIST register - address 0x30 (Hidden)
//***********************************************
struct BIST_BITS_0614
{
    uint16_t ROMADDR: 2;                    //    1:0 ROM results address
    uint16_t ROMRESULT: 1;                //      2 ROM result mode select
    uint16_t ROMBISTDONE: 1;                //      3 Bist done
    uint16_t ROMBISTSTART: 1;            //      4 Start ROM BIST
    uint16_t RAMBISTFAILED: 1;            //      5 RAM BIST FAILED
    uint16_t RAMDONE: 1;                    //      6 RAM BIST Completed
    uint16_t RAMSTART: 1;                //      7 Start RAM BIST
    const uint16_t reserved: 8;            //   15:8 Reserved
};

union BIST_REG_0614
{
    Bos0614Register reg;
    struct BIST_BITS_0614 bit;
};

//***********************************************
//		BISTRES register - address 0x31 (Hidden)
//***********************************************
struct BISTRES_BITS_0614
{
    uint16_t RESULT: 12;                    //    11:0 ROM BIST signature
};

union BISTRES_REG_0614
{
    Bos0614Register reg;
    struct BISTRES_BITS_0614 bit;
};

//***********************************************
//		DEBUG register - address 0x32 (Hidden)
//***********************************************
struct DEBUG_BITS_0614
{
    uint16_t DB: 1;                        //      0 IC in debug mode
    uint16_t EAN: 3;                        //    3:1 Assign Internal node to TA
    uint16_t ZPS: 1;                        //      4 Keep ZPS active
    uint16_t GPIO0M: 1;                    //      5 MSB settings of GPIO
    uint16_t GPIO1M: 1;                    //      6 MSB settings of GPIO
    uint16_t GPIO2M: 1;                    //      7 MSB settings of GPIO
    uint16_t GPIO3M: 1;                    //      8 MSB settings of GPIO
    const uint16_t reserved: 1;            //      9 Reserved
    uint16_t ACCESS: 6;                    //  15:10 Enable/Disable Debug mode   0x3A pour mettre en mode cache  11 1010

};

union DEBUG_REG_0614
{
    Bos0614Register reg;
    struct DEBUG_BITS_0614 bit;
};

//***********************************************
//		THRESH register - address 0x33 (Hidden)
//***********************************************
struct THRESH_BITS_0614
{
    uint16_t VTRESH: 7;                    //   6:0 Internal parameter
    uint16_t THRESH_ERROR: 5;            //  11:7
    uint16_t TRIM_MODE: 2;                // 13:12 Trimming operation mode
    uint16_t SLP: 1;                    //    14 Turn off ZVS and ADC
    uint16_t VDDH: 1;                    //    15 Charges through active switch
};

union THRESH_REG_0614
{
    Bos0614Register reg;
    struct THRESH_BITS_0614 bit;
};

//***********************************************
//		REG38 register - address 0x38 (Hidden)
//***********************************************
struct REG38_BITS_0614
{
    uint16_t HV_OFFSET: 10;            //   9:0 Offset of the main ADC
    const uint16_t reserved: 6;            // 15:10 Reserved
};

union REG38_REG_0614
{
    Bos0614Register reg;
    struct REG38_BITS_0614 bit;
};

//***********************************************
//		REG39 register - address 0x39 (Hidden)
//***********************************************
struct REG39_BITS_0614
{
    uint16_t SENSE_OFFSET: 9;            //   8:0 Sense feedback path offset
    const uint16_t reserved: 7;            //  15:9 Reserved
};

union REG39_REG_0614
{
    Bos0614Register reg;
    struct REG39_BITS_0614 bit;
};

//***********************************************
//		CALIB_DATA register - address 0x3A (Hidden)
//***********************************************
struct CALIB_DATA_BITS_0614
{
    uint16_t DAC_LS: 6;                //    5:0 Dac Calibration LS
    uint16_t DAC_HS: 6;                //   11:6 Dac Calibration HS
    const uint16_t reserved: 4;            //  15:12 Reserved
};

union CALIB_DATA_REG_0614
{
    Bos0614Register reg;
    struct CALIB_DATA_BITS_0614 bit;
};

//***********************************************
//		REG3B register - address 0x3B (Hidden)
//***********************************************
struct REG3B_BITS_0614
{
    uint16_t ERR: 9;                    //    8:0 Sense signal ???
    uint16_t REF_CH: 4;                //   12:9 State of the four channel
    uint16_t CCM: 1;                    //     13 Driver in CCM or DCM mode
    uint16_t DCM: 1;                    //     14 Switching controller
    uint16_t MAX_POWER: 1;            //     15 Max Power
};

union REG3B_REG_0614
{
    Bos0614Register reg;
    struct REG3B_BITS_0614 bit;
};

//***********************************************
//		REG3C register - address 0x3C (Hidden)
//***********************************************
struct REG3C_BITS_0614
{
    uint16_t CURRENT: 9;            //    8:0 Current sent to the ADC mA
    uint16_t S: 1;                //      9 Sign for ADC
    uint16_t CCM: 1;                //     10 Driver operating mode
    uint16_t DCM: 1;                //     11 Switch Controller
    uint16_t OVV: 1;                //     12 Over Voltage Bit
    uint16_t OVERSTATE: 1;        //     13 Converter Status
    uint16_t MODE: 2;                //  15:14 Driver current operating mode
};

union REG3C_REG_0614
{
    Bos0614Register reg;
    struct REG3C_BITS_0614 bit;
};

//***********************************************
//		REG3D register - address 0x3D (Hidden)
//***********************************************
struct REG3D_BITS_0614
{
    uint16_t MIN_CURRENT: 10;        //    9:0 Minimum current to turn on LS
    uint16_t CCM: 1;                //     10 Driver operating mode
    uint16_t DCM: 1;                //     11 Switch Controller
    uint16_t OVV: 1;                //     12 Over Voltage Bit
    uint16_t OVERSTATE: 1;        //     13 Converter Status
    uint16_t MODE: 2;                //  15:14 Driver current operating mode
};

union REG3D_REG_0614
{
    Bos0614Register reg;
    struct REG3D_BITS_0614 bit;
};

//***********************************************
//		REG3E register - address 0x3E (Hidden)
//***********************************************
struct REG3E_BITS_0614
{
    uint16_t ERR: 9;                //    8:0 Error between reference and feedback
    uint16_t PULSE_CTL: 7;        //   15:9 Switch pulse length
};

union REG3E_REG_0614
{
    Bos0614Register reg;
    struct REG3E_BITS_0614 bit;
};

//***********************************************
//		REG3F register - address 0x3F (Hidden)
//***********************************************
struct REG3F_BITS_0614
{
    uint16_t BLANK_CTL: 7;        //    6:0 Blanking time delay
    uint16_t DEAD_CTL: 7;            //   13:7 Dead time delay
    uint16_t MODE: 2;                //  15:14 Current operating mode
};

union REG3F_REG_0614
{
    Bos0614Register reg;
    struct REG3F_BITS_0614 bit;
};

//***********************************************
//		REG40 register - address 0x40 (Hidden)
//***********************************************
struct REG40_BITS_0614
{
    uint16_t REF_REG: 12;            //   11:0 Desired amplitude on output
    uint16_t REF_CH_REG: 4;        //  15:12 State of the four channel
};

union REG40_REG_0614
{
    Bos0614Register reg;
    struct REG40_BITS_0614 bit;
};

//***********************************************
//		REG41 register - address 0x41 (Hidden)
//***********************************************
struct REG41_BITS_0614
{
    uint16_t REF_REG: 12;                //   11:0 Desired amplitude on output
    uint16_t EMPTY: 1;                //     12 FIFO state
    uint16_t FULL: 1;                    //     13 FIFO state
    const uint16_t reserved: 2;            //  15:14 Reserved
};

union REG41_REG_0614
{
    Bos0614Register reg;
    struct REG41_BITS_0614 bit;
};

//***********************************************
//		REG42 register - address 0x42 (Hidden)
//***********************************************
struct REG42_BITS_0614
{
    uint16_t ERR: 9;                    //    8:0 Error between reference and feedback
    uint16_t LOUT: 4;                    //   12:9 Desired amplitude on output
    uint16_t CCM: 1;                    //     10 Driver operating mode
    uint16_t DCM: 1;                    //     11 Switch Controller
    uint16_t MAX_POWER: 1;            //     13 Maximum power status
};

union REG42_REG_0614
{
    Bos0614Register reg;
    struct REG42_BITS_0614 bit;
};

//***********************************************
//		REG43 register - address 0x43 (Hidden)
//***********************************************
struct REG43_BITS_0614
{
    uint16_t PERIODE: 10;                //    9:0 Error between reference and feedback
    uint16_t ZONE: 3;                    //  12:10 Desired amplitude on output
    const uint16_t reserved: 1;            //     13 Reserved
    uint16_t MODE: 2;                    //  15:14 Driver operating mode
};

union REG43_REG_0614
{
    Bos0614Register reg;
    struct REG43_BITS_0614 bit;
};

//***********************************************
//		REG44 register - address 0x44 (Hidden)
//***********************************************
struct REG44_BITS_0614
{
    uint16_t RAM_RAW_DATA: 16;        //   15:0 RAW data from RAM
};

union REG44_REG_0614
{
    Bos0614Register reg;
    struct REG44_BITS_0614 bit;
};

//***********************************************
//		REG45 register - address 0x45 (Hidden)
//***********************************************
struct REG45_BITS_0614
{
    uint16_t ERR: 9;                    //    8:0 Error between reference and feedback
    uint16_t STM_STATE: 2;            //   10:9 Main State machine State
    uint16_t UVLO: 1;                    //     11 Under Voltage bit
    uint16_t OVV: 1;                    //     12 Over Voltage bit
    uint16_t OVERSTATE: 1;            //     13 Reason why the converter stopped
    uint16_t MODE: 2;                    //  15:14 Current operating mode
};

union REG45_REG_0614
{
    Bos0614Register reg;
    struct REG45_BITS_0614 bit;
};

//***********************************************
//		REG46 register - address 0x46 (Hidden)
//***********************************************
struct REG46_BITS_0614
{
    uint16_t CURRENT_PI: 10;            //    9:0 Desired current from PI controller
    uint16_t CCM: 1;                    //     10 Driver operating mode
    uint16_t CCM_STATE: 2;            //  12:11 Approximate ripple
    uint16_t FLAG_CCM_STATE: 2;        //  14:13 CCM State control
    uint16_t DCM: 1;                    //     15 Switch controller
};

union REG46_REG_0614
{
    Bos0614Register reg;
    struct REG46_BITS_0614 bit;
};

//***********************************************
//		REG47 register - address 0x47 (Hidden)
//***********************************************
struct REG47_BITS_0614
{
    uint16_t PERIODE: 10;                //    9:0 Switching period in clk cycles
    uint16_t CCM: 1;                    //     10 Driver operating mode
    uint16_t CCM_STATE: 2;            //  12:11 Approximate ripple
    uint16_t FLAG_CCM_STATE: 2;        //  14:13 CCM State control
    uint16_t DCM: 1;                    //     15 Switch controller
};

union REG47_REG_0614
{
    Bos0614Register reg;
    struct REG47_BITS_0614 bit;
};

//***********************************************
//		Complete registers structure
//***********************************************
typedef struct
{
    union REFERENCE_REG_0614 REFERENCE_0614;
    union REFERENCE_RM_REG_0614 REFERENCE_RM_0614;
    union IC_STATUS_REG_0614 IC_STATUS_0614;
    union READ_REG_0614 READ_0614;
    union GPIOX_REG_0614 GPIOX_0614;
    union TC_REG_0614 TC_0614;
    union CONFIG_REG_0614 CONFIG_0614;
    union SENSECONFIG_REG_0614 SENSECONFIG_0614;
    union SENSE0_REG_0614 SENSE0_0614;
    union SENSE0P_REG_0614 SENSE0P_0614;
    union SENSE0R_REG_0614 SENSE0R_0614;
    union SENSE0S_REG_0614 SENSE0S_0614;
    union SENSE1_REG_0614 SENSE1_0614;
    union SENSE1P_REG_0614 SENSE1P_0614;
    union SENSE1R_REG_0614 SENSE1R_0614;
    union SENSE1S_REG_0614 SENSE1S_0614;
    union SENSE2_REG_0614 SENSE2_0614;
    union SENSE2P_REG_0614 SENSE2P_0614;
    union SENSE2R_REG_0614 SENSE2R_0614;
    union SENSE2S_REG_0614 SENSE2S_0614;
    union SENSE3_REG_0614 SENSE3_0614;
    union SENSE3P_REG_0614 SENSE3P_0614;
    union SENSE3R_REG_0614 SENSE3R_0614;
    union SENSE3S_REG_0614 SENSE3S_0614;
    union SENSESTATUS_REG_0614 SENSESTATUS_0614;
    union SENSEDATA0_REG_0614 SENSEDATA0_0614;
    union SENSEDATA1_REG_0614 SENSEDATA1_0614;
    union SENSEDATA2_REG_0614 SENSEDATA2_0614;
    union SENSEDATA3_REG_0614 SENSEDATA3_0614;
    union SENSERAW0_REG_0614 SENSERAW0_0614;
    union SENSERAW1_REG_0614 SENSERAW1_0614;
    union SENSERAW2_REG_0614 SENSERAW2_0614;
    union SENSERAW3_REG_0614 SENSERAW3_0614;
    union KPA_REG_0614 KPA_0614;
    union KP_KI_REG_0614 KP_KI_0614;
    union KP_KI_RM_REG_0614 KP_KI_RM_0614;
    union DEADTIME_REG_0614 DEADTIME_0614;
    union DEADTIME_RM_REG_0614 DEADTIME_RM_0614;
    union PARCAP_REG_0614 PARCAP_0614;
    union PARCAP_RM_REG_0614 PARCAP_RM_0614;
    union SUP_RISE_REG_0614 SUP_RISE_0614;
    union TRIM_REG_0614 TRIM_0614;
    union CHIP_ID_REG_0614 CHIP_ID_0614;
    union VFEEDBACK_REG_0614 VFEEDBACK_0614;
    union FIFO_STATE_REG_0614 FIFO_STATE_0614;
    union AUTO_STATE_REG_0614 AUTO_STATE_0614;
    union BIST_REG_0614 BIST_0614;
    union BISTRES_REG_0614 BISTRES_0614;
    union DEBUG_REG_0614 DEBUG_0614;
    union THRESH_REG_0614 THRESH_0614;
    union REG38_REG_0614 REG38_0614;
    union REG39_REG_0614 REG39_0614;
    union CALIB_DATA_REG_0614 CALIB_DATA_0614;
    union REG3B_REG_0614 REG3B_0614;
    union REG3C_REG_0614 REG3C_0614;
    union REG3D_REG_0614 REG3D_0614;
    union REG3E_REG_0614 REG3E_0614;
    union REG3F_REG_0614 REG3F_0614;
    union REG40_REG_0614 REG40_0614;
    union REG41_REG_0614 REG41_0614;
    union REG42_REG_0614 REG42_0614;
    union REG43_REG_0614 REG43_0614;
    union REG44_REG_0614 REG44_0614;
    union REG45_REG_0614 REG45_0614;
    union REG46_REG_0614 REG46_0614;
    union REG47_REG_0614 REG47_0614;
} BOS0614_REGS;

#endif /* BOS0614_REGISTER_H_ */
