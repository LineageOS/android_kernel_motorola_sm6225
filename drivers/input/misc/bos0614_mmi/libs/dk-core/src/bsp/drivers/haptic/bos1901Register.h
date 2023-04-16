//
// Description: BOS1901 Registers
// Created on 05/07/2020
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

#ifndef DKCORE_BOS1901REGISTER_H
#define DKCORE_BOS1901REGISTER_H

#include <linux/types.h>

#include "utils/registerUtil.h"

#define TI_RISE_DEFAULT_VALUE   (0x27)
#define VDD_DEFAULT_VALUE       (0x05)

#define REGISTER_VALUE_MASK (0xFFF)

typedef enum
{
    BOS1901AddressReference = 0,
    BOS1901AddressIonBl,
    BOS1901AddressDeadTime,
    BOS1901AddressKp,
    BOS1901AddressKpaKi,
    BOS1901AddressConfig,
    BOS1901AddressParCap,
    BOS1901AddressSupRise,
    BOS1901AddressDac,
    BOS1901AddressThreshold,
    BOS1901AddressTmKi,
    BOS1901AddressDebug,
    BOS1901AddressIcStatus,
    BOS1901AddressSense,
    BOS1901AddressTrim,
    BOS1901AddressAccess,
    BOS1901NbRegisters
} BOS1901AddressEnum;


#define ADDRESS_BOS1901_REFERENCE_REG       (0x00)
#define ADDRESS_BOS1901_ION_BL_REG          (0x01)
#define ADDRESS_BOS1901_DEADTIME_REG        (0x02)
#define ADDRESS_BOS1901_KP_REG              (0x03)
#define ADDRESS_BOS1901_KPA_KI_REG          (0x04)
#define ADDRESS_BOS1901_CONFIG_REG          (0x05)
#define ADDRESS_BOS1901_PARCAP_REG          (0x06)
#define ADDRESS_BOS1901_SUP_RISE_REG        (0x07)
#define ADDRESS_BOS1901_DAC_REG             (0x08)
#define ADDRESS_BOS1901_THRESH_REG          (0x09)
#define ADDRESS_BOS1901_TM_KI_REG           (0x0A)
#define ADDRESS_BOS1901_DEBUG_REG           (0x0B)
#define ADDRESS_BOS1901_IC_STATUS_REG       (0x0C)
#define ADDRESS_BOS1901_SENSE_REG           (0x0D)
#define ADDRESS_BOS1901_TRIM_REG            (0x0E)
#define ADDRESS_BOS1901_ACCESS_REG          (0x0F)
#define BOS1901_LAST_REGISTER_ADDRESS   (ADDRESS_BOS1901_ACCESS_REG)

#define BOS1901_NBR_OF_REGISTER (16)


typedef struct
{
    uint16_t value: 12;
    uint16_t addr: 4;
} Bos1901RegisterStruct;

typedef union
{
    const uint16_t all;
    Bos1901RegisterStruct generic;
} Bos1901Register;

//***********************************************
//		REFERENCE register - address 0x0
//***********************************************
struct REFERENCE_BITS_BOS1901
{
    uint16_t FIFO: 12;            //  11:0 Input of the FIFO
};

union REFERENCE_REG_BOS1901
{
    Bos1901Register reg;
    struct REFERENCE_BITS_BOS1901 bit;
};

//***********************************************
//		ION_BL register - address 0x1
//***********************************************
struct ION_BL_BITS_BOS1901
{
    uint16_t IONSCALE: 8;        //   7:0 Calculated minimum current required to turn ON HS
    uint16_t SB: 2;            //   9:8 Boost converter Blanking time
    uint16_t FSWMAX: 2;        // 11:10 Boost converter maximum switching frequency
};

union ION_BL_REG_BOS1901
{
    Bos1901Register reg;
    struct ION_BL_BITS_BOS1901 bit;
};

//***********************************************
//		DEADTIME register - address 0x2
//***********************************************
struct DEADTIME_BITS_BOS1901
{
    uint16_t DLS: 5;            //   4:0 Power switch deadtime (low side)
    uint16_t DHS: 7;            //  11:5 Power switch deadtime (high side)
};

union DEADTIME_REG_BOS1901
{
    Bos1901Register reg;
    struct DEADTIME_BITS_BOS1901 bit;
};

//***********************************************
//		KP register - address 0x3
//***********************************************
struct KP_BITS_BOS1901
{
    uint16_t KP: 11;            //  10:0 Physical value of the integrated PI controller proportional gain
    uint16_t SQ: 1;            //    11 Square wave playback
};

union KP_REG_BOS1901
{
    Bos1901Register reg;
    struct KP_BITS_BOS1901 bit;
};
//***********************************************
//		KPA_KI register - address 0x4
//***********************************************
struct KPA_KI_BITS_BOS1901
{
    uint16_t KPA: 8;            //   7:0 Calculated value of the integrated PI controller proportional gain
    uint16_t KIBASE: 4;        //  11:8 Calculated pole location of the integrated PI controller
};

union KPA_KI_REG_BOS1901
{
    Bos1901Register reg;
    struct KPA_KI_BITS_BOS1901 bit;
};

//***********************************************
//		CONFIG register - address 0x5
//***********************************************

typedef enum
{
    BOS1901PlayBackSpeed1024kbps = 0,
    BOS1901PlayBackSpeed512kbps,
    BOS1901PlayBackSpeed256kbps,
    BOS1901PlayBackSpeed128kbps,
    BOS1901PlayBackSpeed64kbps,
    BOS1901PlayBackSpeed32kbps,
    BOS1901PlayBackSpeed16kbps,
    BOS1901PlayBackSpeed8kbps
} Bos1901PlaybackSpeedEnum;

#define CONFIG_REG_PLAYBACK_SPEED_MASK  (0x007)
#define CONFIG_REG_DEEP_SLEEP_MASK      (0x008)
#define CONFIG_REG_OUTPUT_ENABLE_MASK   (0x010)
#define CONFIG_REG_RESET_MASK           (0x020)
#define CONFIG_REG_LOCK_MASK            (0x040)
#define CONFIG_REG_BROADCAST_MASK       (0xF80)

#define CONFIG_REG_PLAYBACK_SPEED_SHIFT (0)
#define CONFIG_REG_DEEP_SLEEP_SHIFT     (3)
#define CONFIG_REG_OUTPUT_ENABLE_SHIFT  (4)
#define CONFIG_REG_RESET_SHIFT          (5)
#define CONFIG_REG_LOCK_SHIFT           (6)
#define CONFIG_REG_BROADCAST_SHIFT      (7)

#define CONFIG_REG_CLEAR_RESET(reg)         CLEAR_BITS(reg, CONFIG_REG_RESET_MASK)
#define CONFIG_REG_CLEAR_BROADCAST(reg)     CLEAR_BITS(reg, CONFIG_REG_BROADCAST_MASK)
#define CONFIG_REG_CLEAR_OUTPUT_ENABLE(reg) CLEAR_BITS(reg, CONFIG_REG_OUTPUT_ENABLE_MASK)


#define CONFIG_REG_WRITE_BROADCAST(reg, bc)      (reg |= ((bc << CONFIG_REG_BROADCAST_SHIFT) & CONFIG_REG_BROADCAST_MASK))
#define CONFIG_REG_WRITE_PLAYBACK_SPEED(reg, ps) (reg |= ((ps << CONFIG_REG_PLAYBACK_SPEED_SHIFT) & CONFIG_REG_PLAYBACK_SPEED_MASK))

struct CONFIG_BITS_BOS1901
{
    Bos1901PlaybackSpeedEnum PLAY: 3;   // 2:0 Waveform play back speed
    uint16_t DS: 1;                     // 3 Power mode when not playing waveforms
    uint16_t OE: 1;                     // 4 Enable/Disable waveform playback
    uint16_t RST: 1;                    // 5 Software Reset
    uint16_t LOCK: 1;                   // 6 Register lock
    uint16_t BC: 5;                     // 7 Internal register address
};

union CONFIG_REG_BOS1901
{
    Bos1901Register reg;
    struct CONFIG_BITS_BOS1901 bit;
};

//***********************************************
//		PARCAP register - address 0x6
//***********************************************
struct PARCAP_BITS_BOS1901
{
    uint16_t PARCAP: 8;        // 7:0 Calculated parasitic capacitance
    uint16_t CAL: 1;           // 8 Internal calibration at start-up
    uint16_t CP5: 1;           // 9 Internal 5V charge pump
    uint16_t LMI: 1;           // High Side switch current limit
    uint16_t UPI: 1;           // Unidirectional power input
};

union PARCAP_REG_BOS1901
{
    Bos1901Register reg;
    struct PARCAP_BITS_BOS1901 bit;
};

//***********************************************
//		SUP_RISE register - address 0x7
//***********************************************
#define SUP_RISE_REG_TI_RISE_SHIFT  (0)
#define SUP_RISE_REG_VDD_SHIFT      (6)
#define SUP_RISE_REG_SENSE_SHIFT    (11)

#define SUP_RISE_REG_TI_RISE_MASK   (0x03f)
#define SUP_RISE_REG_VDD_MASK       (0x7C0)
#define SUP_RISE_REG_SENSE_MASK     (0x800)

#define SUP_RISE_REG_WRITE_VDD(value)     ((value << SUP_RISE_REG_VDD_SHIFT) & SUP_RISE_REG_VDD_MASK)
#define SUP_RISE_REG_WRITE_TI_RISE(value) ((value << SUP_RISE_REG_TI_RISE_SHIFT) & SUP_RISE_REG_TI_RISE_MASK)

#define SUP_RISE_REG_CLEAR_SENSE(value)   (value & ~SUP_RISE_REG_SENSE_MASK)


#define TI_RIS

struct SUP_RISE_BITS_BOS1901
{
    uint16_t TI_RISE: 6;        // 5:0 Calculated proportional gain for the offset
    uint16_t VDD: 5;            // 10:6 Calculated digital representation of the supply voltage
    uint16_t SENSE: 1;            // 11 Mute the output to only read the actuator voltage without driving it. Use when sensing a piezo.
};

union SUP_RISE_REG_BOS1901
{
    Bos1901Register reg;
    struct SUP_RISE_BITS_BOS1901 bit;
};

//***********************************************
//		DAC register - address 0x8
//***********************************************
struct DAC_BITS_BOS1901
{
    uint16_t DAC_LS: 6; // Internal calibration value
    uint16_t DAC_HS: 6; // Internal calibration value
};

union DAC_REG_BOS1901
{
    Bos1901Register reg;
    struct DAC_BITS_BOS1901 bit;
};

//***********************************************
//		THRESH register - address 0x9
//***********************************************
struct THRESH_BITS_BOS1901
{
    uint16_t THRESH_ERROR: 5; // Consecutive time that the current required is above a threshold value defined in the IC to ensure clean transition between boost and buck operation of the driver.
    uint16_t HB: 1; // HBridge behavior on startup
    uint16_t TRIM_MODE: 2; // Controls the hardware trimming mode.
    uint16_t PER: 1; // Enable progressive limitation of error
    uint16_t IA: 1; // Enable adaptive integral controller.
    uint16_t CPHB: 2; // CLK division for the Hbridge charge pump
};

union THRESH_REG_BOS1901
{
    Bos1901Register reg;
    struct THRESH_BITS_BOS1901 bit;
};

//***********************************************
//		TM_KI register - address 0xA
//***********************************************
struct TM_KI_BITS_BOS1901
{
    uint16_t RIZ: 1;        //     Reset Integral term at polarity change
    uint16_t OFS: 1;        //    Generate a 30 mV offset between the supply level and the internal power level.
    uint16_t IB: 1; // Relative current boundary to I_on for boost to buck transition.
    uint16_t TMIN: 2; // Minimum period used to scale the integral gain.
    uint16_t VTHRESH: 7; // 7-bit value that indicate when to stop operating the buck controller in pseudo resonant mode.
};

union TM_KI_REG_BOS1901
{
    Bos1901Register reg;
    struct TM_KI_BITS_BOS1901 bit;
};

//***********************************************
//		DEBUG register - address 0xB
//***********************************************
struct DEBUG_BITS_BOS1901
{
    uint16_t MULT1: 4; // Assign internal digital node to TD1 pin;
    uint16_t MULT2: 4; // Assign internal digital node to TD2 pin;
    uint16_t EAN: 3; // Assign internal analog node to TA pin;
    uint16_t DB: 1; // IC in debug mode
};

union DEBUG_REG_BOS1901
{
    Bos1901Register reg;
    struct DEBUG_BITS_BOS1901 bit;
};

//***********************************************
//		IC_STATUS register - address 0xC
//***********************************************

#define IC_STATUS_REG_FIFO_SPACE_MASK           (0x002F)
#define IC_STATUS_REG_FIFO_EMPTY_MASK           (0x0040)
#define IC_STATUS_REG_FIFO_FULL_MASK            (0x0080)
#define IC_STATUS_REG_OVER_TEMP_MASK            (0x0100)
#define IC_STATUS_REG_OVER_VOLTAGE_EMPTY_MASK   (0x0200)
#define IC_STATUS_REG_STATE_MASK                (0x0C00)

#define IC_STATUS_REG_FIFO_SPACE_SHIFT          (0)
#define IC_STATUS_REG_FIFO_EMPTY_SHIFT          (6)
#define IC_STATUS_REG_FIFO_FULL_SHIFT           (7)
#define IC_STATUS_REG_OVER_TEMP_SHIFT           (8)
#define IC_STATUS_REG_OVER_VOLTAGE_EMPTY_SHIFT  (9)
#define IC_STATUS_REG_STATE_SHIFT               (10)

#define IC_STATUS_REG_READ_FIFO_SPACE(reg)      ((reg & IC_STATUS_REG_FIFO_SPACE_MASK) >> IC_STATUS_REG_FIFO_SPACE_SHIFT)
#define IC_STATUS_REG_READ_FIFO_EMPTY(reg)      ((reg & IC_STATUS_REG_FIFO_EMPTY_MASK) >> IC_STATUS_REG_FIFO_EMPTY_SHIFT)
#define IC_STATUS_REG_READ_FIFO_FULL(reg)       ((reg & IC_STATUS_REG_FIFO_FULL_MASK) >> IC_STATUS_REG_FIFO_FULL_SHIFT)
#define IC_STATUS_REG_READ_OVER_TEMP(reg)       ((reg & IC_STATUS_REG_OVER_TEMP_MASK) >> IC_STATUS_REG_OVER_TEMP_SHIFT)
#define IC_STATUS_REG_READ_OVER_VOLTAGE(reg)    ((reg & IC_STATUS_REG_OVER_VOLTAGE_EMPTY_MASK) >> IC_STATUS_REG_OVER_VOLTAGE_EMPTY_SHIFT)
#define IC_STATUS_REG_READ_STATE(reg)           ((reg & IC_STATUS_REG_STATE_MASK) >> IC_STATUS_REG_STATE_SHIFT)

struct IC_STATUS_BITS_BOS1901
{
    uint16_t FIFO_SPACE: 6; // Space available in FIFO for new data;
    uint16_t EMPTY: 1;        //   Fifo is empty
    uint16_t FULL: 1;            //  Fifo is full
    uint16_t OVT: 1;            //   Overtemperature status bit
    uint16_t OVV: 1;            //   Over voltage bit
    uint16_t STATE: 2;        //   State of the controller
};

union IC_STATUS_REG_BOS1901
{
    Bos1901Register reg;
    struct IC_STATUS_BITS_BOS1901 bit;
};

//***********************************************
//		SENSE register - address 0xD
//***********************************************

#define SENSE_REG_VFEEDBACK_MASK (0x03FF)
#define SENSE_REG_STATE_MASK     (0x0C00)

#define SENSE_REG_VFEEDBACK_SHIFT   (0)
#define SENSE_REG_STATE_SHIFT       (10)

#define SENSE_REG_READ_VFEEDBACK(reg)   ((reg & SENSE_REG_VFEEDBACK_MASK) >> SENSE_REG_VFEEDBACK_SHIFT)
#define SENSE_REG_READ_STATE(reg)       ((reg & SENSE_REG_STATE_MASK) >> SENSE_REG_STATE_SHIFT)

#define VFEEDBACK_IN_MILLIVOLT(value) ((value * 3600 * 31) / (1024 - 1))

struct SENSE_BITS_BOS1901
{
    uint16_t VFEEDBACK: 10;    //     Voltage feedback on 10 bit
    uint16_t STATE: 2; // State of the controller
};

union SENSE_REG_BOS1901
{
    Bos1901Register reg;
    struct SENSE_BITS_BOS1901 bit;
};

//***********************************************
//		TRIM register - address 0xE
//***********************************************
struct TRIM_BITS_BOS1901
{
    uint16_t TRIM_REG: 2;     //   2:0 1.8V regulator trimming bits in 2's complement
    uint16_t TRIM_OSC: 6;     //   9:3 Oscillator trimming bits in 2's complement
    uint16_t SDOBP: 1;          // Assign internal clock signal to SDO pin
    uint16_t TRIMRW: 3;        // 11:10 On demand trim control
};

union TRIM_REG_BOS1901
{
    Bos1901Register reg;
    struct TRIM_BITS_BOS1901 bit;
};


//***********************************************
//		ACCESS register - address 0xF
//***********************************************
struct ACCESS_BITS_BOS1901
{
    uint16_t ACCESS0: 1; // Access bit 0
    uint16_t RSVD0: 1;
    uint16_t ACCESS1: 1; // Access bit 1
    uint16_t RSVD1: 4;
    uint16_t ACCESS2: 1; // Access bit 2
    uint16_t RSVD2: 1;
    uint16_t ACCESS3: 1; // Access bit 3
    uint16_t RSVD3: 2;
};

union ACCESS_REG_BOS1901
{
    Bos1901Register reg;
    struct ACCESS_BITS_BOS1901 bit;
};

//***********************************************
//		Complete registers structure
//***********************************************
typedef struct
{
    union REFERENCE_REG_BOS1901 REFERENCE;      // REFERENCE register
    union ION_BL_REG_BOS1901 ION_BL;            // ION_BL register
    union DEADTIME_REG_BOS1901 DEADTIME;        // DEADTIME register
    union KP_REG_BOS1901 KP;                    // KP register
    union KPA_KI_REG_BOS1901 KPA_KI;            // KPA_KI register
    union CONFIG_REG_BOS1901 CONFIG;            // CONFIG register
    union PARCAP_REG_BOS1901 PARCAP;            // PARCAP register
    union SUP_RISE_REG_BOS1901 SUP_RISE;        // SUP_RISE register
    union DAC_REG_BOS1901 DAC_REG;              // DAC register
    union THRESH_REG_BOS1901 THRESH;            // THRESH register
    union TM_KI_REG_BOS1901 TM_KI;              // TM_KI register
    union DEBUG_REG_BOS1901 DEBUG_REG;          // DEBUG register
    union IC_STATUS_REG_BOS1901 IC_STATUS;      // IC_STATUS register
    union SENSE_REG_BOS1901 SENSE;              // SENSE register
    union TRIM_REG_BOS1901 TRIM;                // TRIM register
    union ACCESS_REG_BOS1901 ACCESS;            // ACCESS register
} BOS1901_REGS;
#endif //DKCORE_BOS1901REGISTER_H
