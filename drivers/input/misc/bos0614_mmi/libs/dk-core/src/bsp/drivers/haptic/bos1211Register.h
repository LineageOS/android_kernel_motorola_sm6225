/*
 * BOS1211_config.h
 *
 *  Created on: 2019-10-01
 *      Author: CarlMoreau
 */

#ifndef GIT_INCLUDE_BOS1211_CONFIG_H_
#define GIT_INCLUDE_BOS1211_CONFIG_H_

#include <stdint.h>

#define BOS1211_NBR_OF_REGISTER (16)

#define BOS1211_ADDRESS_REFERENCE_REG   (0x0)
#define BOS1211_ADDRESS_ION_BL_REG      (0x1)
#define BOS1211_ADDRESS_DEADTIME_REG    (0x2)
#define BOS1211_ADDRESS_KP_REG          (0x3)
#define BOS1211_ADDRESS_KPA_KI_REG      (0x4)
#define BOS1211_ADDRESS_CONFIG_REG      (0x5)
#define BOS1211_ADDRESS_PARCAP_REG      (0x6)
#define BOS1211_ADDRESS_SUP_RISE_REG    (0x7)
#define BOS1211_ADDRESS_RAM_REG         (0X8)
#define BOS1211_ADDRESS_SPI_REG         (0x9)
#define BOS1211_ADDRESS_SENSING_REG     (0xA)
#define BOS1211_ADDRESS_SENSE_REG       (0xB)
#define BOS1211_ADDRESS_IC_STATUS_REG   (0xC)
#define BOS1211_ADDRESS_FIFO_STATUS_REG (0xD)
#define BOS1211_ADDRESS_TRIM_REG        (0xE)
#define BOS1211_ADDRESS_CHIP_ID         (0xF)
#define BOS1211_LAST_REGISTER_ADDRESS   (BOS1211_ADDRESS_CHIP_ID)

typedef struct
{
    uint16_t value: 12;
    uint16_t addr: 4;
} Bos1211RegisterStruct;

typedef union
{
    const uint16_t all;
    Bos1211RegisterStruct generic;
} Bos1211Register;

//***********************************************
//		REFERENCE register - address 0x0
//***********************************************
struct REFERENCE_BITS
{
    uint16_t FIFO: 12;            //  11:0 Input of the FIFO
};

union REFERENCE_REG
{
    Bos1211Register reg;
    struct REFERENCE_BITS bit;
};

//***********************************************
//		ION_BL register - address 0x1
//***********************************************
struct ION_BL_BITS
{
    uint16_t IONSCALE: 8;        //   7:0 Calculated minimum current required to turn ON HS
    uint16_t SB: 2;            //   9:8 Boost converter Blanking time
    uint16_t FSWMAX: 2;        // 11:10 Boost converter maximum switching frequency
};

union ION_BL_REG
{
    Bos1211Register reg;
    struct ION_BL_BITS bit;
};

//***********************************************
//		DEADTIME register - address 0x2
//***********************************************
struct DEADTIME_BITS
{
    uint16_t DLS: 5;            //   4:0 Power switch deadtime (low side)
    uint16_t DHS: 7;            //  11:5 Power switch deadtime (high side)
};

union DEADTIME_REG
{
    Bos1211Register reg;
    struct DEADTIME_BITS bit;
};

//***********************************************
//		KP register - address 0x3
//***********************************************
struct KP_BITS
{
    uint16_t KP: 11;            //  10:0 Physical value of the integrated PI controller proportional gain
    uint16_t SQ: 1;            //    11 Square wave playback
};

union KP_REG
{
    Bos1211Register reg;
    struct KP_BITS bit;
};
//***********************************************
//		KPA_KI register - address 0x4
//***********************************************
struct KPA_KI_BITS
{
    uint16_t KPA: 8;            //   7:0 Calculated value of the integrated PI controller proportional gain
    uint16_t KIBASE: 4;        //  11:8 Calculated pole location of the integrated PI controller
};

union KPA_KI_REG
{
    Bos1211Register reg;
    struct KPA_KI_BITS bit;
};

//***********************************************
//		CONFIG register - address 0x5
//***********************************************
typedef enum
{
    SamplingRate_1024Ksps = 0,
    SamplingRate_512Ksps,
    SamplingRate_256Ksps,
    SamplingRate_128Ksps,
    SamplingRate_64Ksps,
    SamplingRate_32Ksps,
    SamplingRate_16Ksps,
    SamplingRate_8Ksps,
} SamplingRate;


struct CONFIG_BITS
{
    SamplingRate PLAY: 3;            //   2:0 Waveform play back speed
    uint16_t DS: 1;            //     3 Power mode when not playing waveforms
    uint16_t OE: 1;            //     4 Enable/Disable waveform playback
    uint16_t RST: 1;            //     5 Software Reset
    uint16_t LOCK: 1;            //     6 Register lock
    uint16_t SYNC: 1;            //     7 Register Sync
    uint16_t AUTO: 1;            //     8 Register Auto
    uint16_t SENSE: 1;        //     9 Register Sense
    uint16_t SIGN: 1;            //    10 Register Sign
    uint16_t ONCOMP: 1;        //    11 Register OnComp
};

union CONFIG_REG
{
    Bos1211Register reg;
    struct CONFIG_BITS bit;
};

//***********************************************
//		PARCAP register - address 0x6
//***********************************************
struct PARCAP_BITS
{
    uint16_t PARCAP: 8;        //   7:0 Calculated parasitic capacitance
    uint16_t SLS: 2;            //   9:8 Low Side Gate Strength
    uint16_t SHS: 2;            // 11:10 High Side Gate Strength
};

union PARCAP_REG
{
    Bos1211Register reg;
    struct PARCAP_BITS bit;
};

//***********************************************
//		SUP_RISE register - address 0x7
//***********************************************
struct SUP_RISE_BITS
{
    uint16_t TI_RISE: 6;        // 5:0 Calculated proportional gain for the offset
    uint16_t VIN: 5;            // 10:6 Calculated digital representation of the supply voltage
    uint16_t TOUT: 1;            // 11 Automatic TimeOut of the waveform (1=Enable,0=Disable)
};

union SUP_RISE_REG
{
    Bos1211Register reg;
    struct SUP_RISE_BITS bit;
};

typedef enum
{
    BOSMode_Direct = 0x0,
    BOS1211Mode_FIFO,
    BOS1211Mode_RAM_Playback,
    BOS1211Mode_RAM_Synthesis,
} BOS1211Mode;

typedef enum
{
    BOSRamMode_RegisterBank = 0x0,
    BOSRamMode_WSFBank,
} BOSRamMode;

//***********************************************
//		RAM register - address 0x8
//***********************************************
struct RAM_BITS
{
    BOS1211Mode MODE: 2;        //   1:0 Ram Behavior   (0=Direct, 1=FIFo, 2=Direct Play, 3=Synthesis)
    BOSRamMode RAMSEL: 1;        //     2 Select RAM (1= RAM controller, 0= Bank Register)
    uint16_t rsvd1: 1;        //     3 reserved
    uint16_t ROMRESULTADD: 2;  //   5:4 ROM bist signature Address (0-3) Signature (57ed8ec6)
    uint16_t ROMRESULT: 1;    //     6 ROM Test result ( 0=Not in result mode, 1= In result mode)
    uint16_t ROMBISTDONE: 1;    //     7 ROM Bist done (0= not done, 1=done)
    uint16_t ROMBISTSTART: 1;    //     8 ROM Bist Start (0=No test, 1 = RUN test) Only when OE=0
    uint16_t RAMBISTFAIL: 1;    //     9 RAM Bist Failed (0=Test Failed, 1 = Test Passed)
    uint16_t RAMBISTDONE: 1;    //    10 RAM Bist Done (0=Not Done, 1=Done)
    uint16_t RAMBISTSTART: 1;    //    11 RAM Bist Start (0= No Test , 1 =RUn test) Only when OE=0
};

union RAM_REG
{
    Bos1211Register reg;
    struct RAM_BITS bit;
};

typedef enum
{
    Bos1211GpoMode_Not_Defined = 0,
    Bos1211GpoMode_SenseTrigger,
    BOS1211GpoMode_WaveformDone,
    Bos1211GpoMode_Error,
    Bos1211GpoMode_MaxPower,
    Bos1211GpoMode_NextData = 7
} Bos1211GpoMode;

//***********************************************
//		SPI register - address 0x9
//***********************************************
struct SPI_BITS
{
    Bos1211GpoMode gpo: 3;            //   4:0 Control the output on GPO pad
    const uint16_t reserved: 2;
    uint16_t SHORT: 2;            //   6:5 Length of time to zero the piezo
    uint16_t BC: 5;            //  7:11 Address of internal register whose content is shifted out on SPI
};

union SPI_REG
{
    Bos1211Register reg;
    struct SPI_BITS bit;
};

//***********************************************
//		SENSING register - address 0xA
//***********************************************
struct SENSING_BITS
{
    uint16_t STHRESH: 9;        //     8:0 Differential voltage required to detect an event
    uint16_t REP: 3;            //    11:9 Amount of time the sense voltage needs to be above STHRESH
};

union SENSING_REG
{
    Bos1211Register reg;
    struct SENSING_BITS bit;
};

//***********************************************
//		SENSE register - address 0xB
//***********************************************
struct SENSE_BITS
{
    uint16_t VFEEDBACK: 10;    //     9:0 Voltage feedback on 10 bit
    uint16_t STATE: 2;        //   11:10 State of the controller
};

union SENSE_REG
{
    Bos1211Register reg;
    struct SENSE_BITS bit;
};

//***********************************************
//		IC_STATUS register - address 0xC
//***********************************************
struct IC_STATUS_BITS
{
    uint16_t EMPTY: 1;        //   0 Voltage feedback on 10 bit
    uint16_t FULL: 1;            //   1 State of the controller
    uint16_t SC: 1;            //   2 Register address
    uint16_t OVLO12: 1;        //   3 Register address
    uint16_t UVLO12: 1;        //   4 Register address
    uint16_t UVLO5: 1;        //   5 Register address
    uint16_t IDAC: 1;            //   6 Register address
    uint16_t MAX_POWER: 1;    //   7 Register address
    uint16_t OVT: 1;            //   8 Register address
    uint16_t OVV: 1;            //   9 Register address
    uint16_t STATE: 2;        //   11:10 Register address
};

union IC_STATUS_REG
{
    Bos1211Register reg;
    struct IC_STATUS_BITS bit;
};

//***********************************************
//		FIFO_STATUS register - address 0xD
//***********************************************
struct FIFO_STATUS_BITS
{
    uint16_t FIFO_SPACE: 10;    //     9:0 Space available
    uint16_t EMPTY: 1;        //      10 FIFO is empty
    uint16_t ERROR: 1;        //      11 ERROR (1=error, 0=no error)
};

union FIFO_STATUS_REG
{
    Bos1211Register reg;
    struct FIFO_STATUS_BITS bit;
};

//***********************************************
//		TRIM register - address 0xE
//***********************************************
struct TRIM_BITS
{
    uint16_t TRIM_REG: 3;     //   2:0 1.8V regulator trimming bits in 2's complement
    uint16_t TRIM_OSC: 7;     //   9:3 Oscillator trimming bits in 2's complement
    uint16_t TRIMRW: 2;        // 11:10 On demand trim control
};

union TRIM_REG
{
    Bos1211Register reg;
    struct TRIM_BITS bit;
};


//***********************************************
//		CHIP_ID register - address 0xF
//***********************************************
struct CHIP_ID_BITS
{
    uint16_t CHIPID: 6;        //   5:0 ID (0x30= BOS1211, 0x32=BOS1212)
    uint16_t rsvd: 6;        //     6 Access bit 0
};

union CHIP_ID_REG
{
    Bos1211Register reg;
    struct CHIP_ID_BITS bit;
};

//***********************************************
//		Complete registers structure
//***********************************************
typedef struct
{
    union REFERENCE_REG REFERENCE;      // REFERENCE register
    union ION_BL_REG ION_BL;            // ION_BL register
    union DEADTIME_REG DEADTIME;        // DEADTIME register
    union KP_REG KP;                    // KP register
    union KPA_KI_REG KPA_KI;            // KPA_KI register
    union CONFIG_REG CONFIG;            // CONFIG register
    union PARCAP_REG PARCAP;            // PARCAP register
    union SUP_RISE_REG SUP_RISE;        // SUP_RISE register
    union RAM_REG RAM;                  // RAM register
    union SPI_REG SPI;                  // SPI register
    union SENSING_REG SENSING;          // SENSING register
    union SENSE_REG SENSE;              // SENSE register
    union IC_STATUS_REG IC_STATUS;      // IC_STATUS register
    union FIFO_STATUS_REG FIFO_STATUS;  // FIFO_STATUS register
    union TRIM_REG TRIM;                // TRIM register
    union CHIP_ID_REG CHIP_ID;          // CHIP ID register
} BOS1211_REGS;

#endif
