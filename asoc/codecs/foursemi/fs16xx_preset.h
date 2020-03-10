/* 
* Copyright (C) 2016 Fourier Semiconductor Inc. 
*
*/

#ifndef FS16XX_PRESET_H
#define FS16XX_PRESET_H

#include "fs16xx_define.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PRESET_NAME_SIZE  256

// Preset version for 1801 series
#define PRESET_VERSION 0x0A01

// Size in int32
#define EXCUR_PROT_RAM_LEN 0x39

#pragma pack(push, 1)
struct fs16xx_preset_header {
    uint16_t preset_ver; /* Preset version, should be PRESET_VERSION */
    char customer_name[32]; /* Customer info */
    int64_t create_time;    /* Time tick when the preset created */
    uint16_t data_len;/* Length of the data in int16_t */
    uint16_t crc16;   /* CRC check sum of the data */
    uint16_t tmax; /* speaker max temprature */
    uint16_t tempr_coef; /* speaker temperature coef */
    uint16_t tempr_sel; /* default value of reg TEMPSEL (0x08)*/
    uint16_t spk_set; /* default value of reg SPKSET (0x0A)*/
    uint16_t adp_bst[2]; /* default value of reg ADPBST (0xBF)*/
    uint16_t bst_ctrl[2]; /* default value of reg BSTCTRL (0xC0)*/
    uint16_t agc_ctrl[2]; /* default value of reg AGCCTRL (0xA7)*/
    uint16_t agc_thd[2]; /* default value of reg AGCTHD (0xA8)*/
    uint16_t agc_para[2]; /* default value of reg AGCPARA (0xA9)*/
    uint16_t agc_mg[2]; /* default value of reg AGCMG (0xAA)*/
    uint16_t dac_para1[2]; /* default value of reg DACPARA1 (0xCD)*/
    uint16_t acs_drc[2]; /* default value of reg ACSDRC (0x8A)*/
    uint16_t acs_drcs[2]; /* default value of reg ACSDRCS (0x8B)*/
    uint16_t acs_drcp[2]; /* default value of reg ACSDRCP (0x8C)*/
    uint16_t ts_ctrl[2]; /* default value of reg TSCTRL (0xAF)*/
    int32_t  *excur_prot_ram; /* excur prot coef ram */
    uint16_t *data;   /* Point to coef ram data */
};
typedef struct fs16xx_preset_header Fs16xx_preset_header_t;

#define FS_PRESET_HEADER_LEN (sizeof(Fs16xx_preset_header_t) - sizeof(uint16_t *))

enum _fs16xx_preset_type {
    fs16xx_preset_left = 0,
    fs16xx_preset_right = 1,
    fs16xx_preset_type_max = 2 // Set to count of preset types
};
typedef enum _fs16xx_preset_type fs16xx_preset_type;

struct fs16xx_preset_content {
    int valid;
    Fs16xx_preset_header_t preset_header;
};
typedef struct fs16xx_preset_content Fs16xx_preset;
#pragma pack(pop)
/**
 * Fs16xx initialize presets
 */
Fs16xx_Error_t fs16xx_initialize_presets(void);

/**
 * Fs16xx deinitialize presets
 */
Fs16xx_Error_t fs16xx_deinitialize_presets(void);

/**
 * Fs16xx load presets from storage
 */
Fs16xx_Error_t fs16xx_load_presets(Fs16xx_devId_t id);

/**
 * Fs16xx load presets to fs16xx device
 */
Fs16xx_Error_t fs16xx_load_presets_to_dev(Fs16xx_devId_t id);

uint16_t fs16xx_calc_checksum(uint16_t *data, int len);

/**
 * Fs16xx set dsp preset
 * @param id to opened instance
 * @param preset to set in dsp
 */
Fs16xx_Error_t fs16xx_set_preset(Fs16xx_devId_t id, int preset);

#ifdef __cplusplus
}
#endif

#endif

