/* 
* Copyright (C) 2016 Fourier Semiconductor Inc. 
*
*/
#include "fs16xx.h"
#include "fs16xx_regs.h"
#include "fs16xx_preset.h"

static int preset_init = 0;
static Fs16xx_preset gFs16xxPresets[(int)fs16xx_preset_type_max];

#ifndef FS_STEREO_MODE 
static char const *gFs16xxPresetName[(int)fs16xx_preset_type_max] = {
    "fs1801_mono.preset",
    "fs1801_mono.preset" // prevent from crash in kernel
};
#else
static char const *gFs16xxPresetName[(int)fs16xx_preset_type_max] = {
    "fs1801_left.preset",
    "fs1801_right.preset"
};
#endif

#define CRC16_TABLE_SIZE 256
const uint16_t gCrc16Polynomial = 0xA001;
static uint16_t gCrc16Table[CRC16_TABLE_SIZE];

#define PRESET_VALID    1
#define PRESET_INVALID    0

/* Initialize presets should only be called once in fs16xx_init
*/
Fs16xx_Error_t fs16xx_initialize_presets(void) {
    uint16_t value;
    uint16_t temp, i;
    uint8_t j;
    if(preset_init) return Fs16xx_Error_OK;
    memset(gFs16xxPresets, 0, sizeof(Fs16xx_preset) * (int)fs16xx_preset_type_max);

    // Initialize CRC16 table
    for (i = 0; i < CRC16_TABLE_SIZE; ++i)
    {
        value = 0;
        temp = i;
        for (j = 0; j < 8; ++j)
        {
            if (((value ^ temp) & 0x0001) != 0)
            {
                value = (uint16_t)((value >> 1) ^ gCrc16Polynomial);
            }
            else
            {
                value >>= 1;
            }
            temp >>= 1;
        }
        gCrc16Table[i] = value;
    }
    preset_init = 1;
    return Fs16xx_Error_OK;
}

Fs16xx_Error_t fs16xx_deinitialize_presets(void) {
    int i;
    for(i = 0; i < (int)fs16xx_preset_type_max; i++) {
        if(PRESET_VALID == gFs16xxPresets[i].valid) {
            if(gFs16xxPresets[i].preset_header.data) {
                gFs16xxPresets[i].valid = PRESET_INVALID;
                kfree(gFs16xxPresets[i].preset_header.data);
            }
        }
    }

    return Fs16xx_Error_OK;
}

// load_presets should only be called once after initialization
Fs16xx_Error_t fs16xx_load_presets(Fs16xx_devId_t id) {
    const struct firmware *fs_firmware = NULL;
    struct snd_soc_codec *codec;
    int j, data_len, buf_size, ret, pos;
    uint16_t *ptr_data = NULL;
    uint16_t *ptr_crc_data = NULL;
    int32_t  *ptr_excur_prot_ram = NULL;
    uint16_t checksum;
    // Preset crc data length in uint16_t : size from member tmax to the start of excur_prot_ram
    const int preset_header_crc_data_len =
        (sizeof(Fs16xx_preset_header_t) - sizeof(int32_t *) - sizeof(uint16_t *)
        - sizeof(uint16_t) * 3 - 32 - sizeof(int64_t)) / 2;
    const int preset_header_len = sizeof(Fs16xx_preset_header_t) - sizeof(int32_t *)
        - sizeof(uint16_t *);

    PRINT("%s enter. id = %d", __func__, id);

    if(FS16XX_DEV_INDEX_L != id && FS16XX_DEV_INDEX_R != id) {
        PRINT_ERROR("%s invalid id[%d]", __func__, id);
        return Fs16xx_Error_Bad_Parameter;
    }

    codec = fs16xx_get_codec_by_id(id);
    if(NULL == codec) {
        PRINT_ERROR("%s invalid codec with id[%d]", __func__, id);
        return Fs16xx_Error_Bad_Parameter;
    }

    fs16xx_initialize_presets();

    if(gFs16xxPresets[id].valid) {
        PRINT_ERROR("%s invalid status, presets already loaded.", __func__);
        return Fs16xx_Error_StateInvalid;
    }

    ret = request_firmware(&fs_firmware, gFs16xxPresetName[id], codec->dev);
    if (ret) {
        PRINT_ERROR("%s Failed to load fs preset table %s", __func__, gFs16xxPresetName[id]);
        return Fs16xx_Error_Invalid_Preset;
    }
    PRINT("fs16xx: loaded firmware size = 0x%04X", fs_firmware->size);
    if(fs_firmware->size < sizeof(Fs16xx_preset_header_t)) {
        PRINT_ERROR("%s Invalid preset table %s size=%d", __func__,
            gFs16xxPresetName[id], fs_firmware->size);
        release_firmware(fs_firmware);
        return Fs16xx_Error_Invalid_Preset;
    }
    
    // Parse header
    memcpy(&(gFs16xxPresets[id].preset_header), fs_firmware->data, preset_header_len);

    // Checking preset version
    if(PRESET_VERSION != gFs16xxPresets[id].preset_header.preset_ver) {
        PRINT_ERROR("%s preset version[0x%04X] invalid!",
            __func__, gFs16xxPresets[id].preset_header.preset_ver);
        release_firmware(fs_firmware);
        return Fs16xx_Error_Invalid_Preset;
    }
    PRINT("fs16xx: customer_name: %s\n", gFs16xxPresets[id].preset_header.customer_name);
    data_len = gFs16xxPresets[id].preset_header.data_len;
    if(data_len <= 0 || (data_len % 2) != 0) {
        PRINT_ERROR("%s preset header [%s] validation checking failed. data_len=%d", 
            __func__, gFs16xxPresetName[id], data_len);
        release_firmware(fs_firmware);
        return Fs16xx_Error_Invalid_Preset;
    }

    // Copy excur prot
    buf_size = EXCUR_PROT_RAM_LEN * sizeof(int32_t);
    ptr_excur_prot_ram = (int32_t *)kzalloc(buf_size, GFP_KERNEL);
    if (!ptr_excur_prot_ram) {
        PRINT_ERROR("%s failed to allocate %d bytes for excur_prot_ram.\n", __func__, buf_size);
        gFs16xxPresets[id].preset_header.excur_prot_ram = 0;
        release_firmware(fs_firmware);
        return Fs16xx_Error_Invalid_Preset;
    }
    memcpy(ptr_excur_prot_ram, ((u8*)fs_firmware->data) + preset_header_len, buf_size);

    buf_size = data_len * sizeof(uint16_t);
    ptr_data = kzalloc(buf_size, GFP_KERNEL);
    if (!ptr_data) {
        PRINT_ERROR("%s failed to allocate %d bytes.", __func__, buf_size);
        gFs16xxPresets[id].preset_header.data = 0;
        release_firmware(fs_firmware);
        return Fs16xx_Error_Invalid_Preset;
    }

    memcpy(ptr_data, ((u8*)fs_firmware->data) + preset_header_len
        + (EXCUR_PROT_RAM_LEN * sizeof(int32_t)), buf_size);
    release_firmware(fs_firmware);

    // Fill ptr_crc_data
    buf_size = (data_len + preset_header_crc_data_len) * sizeof(uint16_t)
        + EXCUR_PROT_RAM_LEN * sizeof(int32_t);
    DEBUGPRINT("%s buf_size = 0x%04X", __func__, buf_size);
    ptr_crc_data = (uint16_t *)kzalloc(buf_size, GFP_KERNEL);
    if (!ptr_crc_data) {
        PRINT_ERROR("%s failed to allocate %d bytes.\n", __func__, buf_size);
        gFs16xxPresets[id].preset_header.data = 0;
        return Fs16xx_Error_Invalid_Preset;
    }

    pos = 0;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.tmax; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.tempr_coef; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.tempr_sel; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.spk_set; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.adp_bst[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.adp_bst[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.bst_ctrl[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.bst_ctrl[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_ctrl[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_ctrl[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_thd[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_thd[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_para[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_para[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_mg[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.agc_mg[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.dac_para1[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.dac_para1[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.acs_drc[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.acs_drc[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.acs_drcs[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.acs_drcs[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.acs_drcp[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.acs_drcp[1]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.ts_ctrl[0]; pos++;
    ptr_crc_data[pos] = gFs16xxPresets[id].preset_header.ts_ctrl[1]; pos++;
    
    for (j = 0; j < EXCUR_PROT_RAM_LEN; j++)
    {
        ptr_crc_data[pos] = (uint16_t)(ptr_excur_prot_ram[j] & 0xFFFF);
        ptr_crc_data[pos + 1] = (uint16_t)((ptr_excur_prot_ram[j] & 0xFF0000) >> 16);
        pos += 2;
    }

    for (j = 0; j < data_len; j++)
    {
        ptr_crc_data[pos] = ptr_data[j];
        pos++;
    }

    /*if(pos != (data_len + preset_header_crc_data_len)) {
        PRINT_ERROR("%s preset length not matching! pos=%d expected=%d.\n", __func__,
        pos, (data_len + preset_header_crc_data_len));
        kfree(ptr_data);
        kfree(ptr_crc_data);
        return Fs16xx_Error_Invalid_Preset;
    }*/

    checksum = fs16xx_calc_checksum(ptr_crc_data, buf_size / 2);
    kfree(ptr_crc_data);

    if(checksum != gFs16xxPresets[id].preset_header.crc16) {
        PRINT_ERROR("%s preset data [%s] checksum not match. checksum=0x%04X, expected=0x%04X", 
            __func__, gFs16xxPresetName[id], checksum, gFs16xxPresets[id].preset_header.crc16);
        gFs16xxPresets[id].preset_header.data = 0;
        kfree(ptr_data);
        
        return Fs16xx_Error_Invalid_Preset;
    }

    gFs16xxPresets[id].preset_header.data = ptr_data;
    gFs16xxPresets[id].preset_header.excur_prot_ram = ptr_excur_prot_ram;
    gFs16xxPresets[id].valid = PRESET_VALID;
    
    fs16xx_load_presets_to_dev(id);
    PRINT("%s successfully exit. boost 0x%04X", __func__,
        gFs16xxPresets[id].preset_header.bst_ctrl[0]);
    return Fs16xx_Error_OK;
}

Fs16xx_Error_t fs16xx_load_presets_to_dev(Fs16xx_devId_t id) {
    int i, data_len;
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    uint16_t *ptr_data = 0;
    int32_t *ptr_prot_coef = 0;
    uint16_t valPwr;
    Fs_Dev_State *dev_state = fs16xx_get_dev_state_by_id(id);

    if(FS16XX_DEV_INDEX_L != id && FS16XX_DEV_INDEX_R != id) {
        PRINT_ERROR("%s invalid id[%d]", __func__, id);
        return Fs16xx_Error_Bad_Parameter;
    }
    if(dev_state == NULL){
        PRINT_ERROR("%s dev_state is null!", __func__);
        return Fs16xx_Error_Bad_Parameter;
    }

    if(gFs16xxPresets[id].valid != PRESET_VALID 
        || gFs16xxPresets[id].preset_header.data == NULL) {
        PRINT_ERROR("%s invalid preset id[%d] data=%p", __func__, 
            id, gFs16xxPresets[id].preset_header.data);
        return Fs16xx_Error_Invalid_Preset;
    }

    dev_state->cur_preset = -1; // Reset preset index
    dev_state->tmax = gFs16xxPresets[id].preset_header.tmax;
    dev_state->tempr_coef = gFs16xxPresets[id].preset_header.tempr_coef;
    dev_state->tempr_sel = gFs16xxPresets[id].preset_header.tempr_sel;
    dev_state->spk_set = gFs16xxPresets[id].preset_header.spk_set;
    dev_state->adp_bst = gFs16xxPresets[id].preset_header.adp_bst[0];
    dev_state->bst_ctrl = gFs16xxPresets[id].preset_header.bst_ctrl[0];
    dev_state->acs_drc_ctrl[0] = gFs16xxPresets[id].preset_header.acs_drc[0];
    dev_state->acs_drc_ctrl[1] = gFs16xxPresets[id].preset_header.acs_drcs[0];
    dev_state->acs_drc_ctrl[2] = gFs16xxPresets[id].preset_header.acs_drcp[0];

    dev_state->ts_ctrl = gFs16xxPresets[id].preset_header.ts_ctrl[0];

    data_len = gFs16xxPresets[id].preset_header.data_len;
    ptr_data = gFs16xxPresets[id].preset_header.data;
    ptr_prot_coef = gFs16xxPresets[id].preset_header.excur_prot_ram;
    dev_state->dev_preset = &(gFs16xxPresets[id]);
    
    DEBUGPRINT("%s ptr_data=%p data_len=%d", __func__, ptr_data, data_len);

    // Amp off
    err |= fs16xx_read_register16(id, FS16XX_SYSCTRL_REG, &valPwr);
    err |= fs16xx_write_register16(id, FS16XX_SYSCTRL_REG, valPwr & (~FS16XX_SYSCTRL_REG_AMPE));

    // Set spk_set
    err |= fs16xx_write_register16(id, FS16XX_SPKSET_REG, gFs16xxPresets[id].preset_header.spk_set);
    // Set tempr coef to TEMPSEL, refer to Bug 55
    err |= fs16xx_write_register16(id, FS16XX_TEMPSEL_REG, (gFs16xxPresets[id].preset_header.tempr_coef) << 1);

    fs16xx_write_register16(id, FS16XX_OTPACC_REG, FS16XX_OTP_ACC_KEY2);

    if(Fs16xx_Error_OK == fs16xx_wait_dsp_off(id, 0, 100)) {
        // Start from position 0
        err |= fs16xx_write_register16(id, FS16XX_DACEQA_REG, 0);
         // Write ACS data
        for (i = 0; i < gFs16xxPresets[id].preset_header.data_len; i+=2) {
            fs16xx_write_register16(id, FS16XX_DACEQWL_REG, ptr_data[i] & 0xFFFF);
            fs16xx_write_register16(id, FS16XX_DACEQWH_REG, ptr_data[i + 1] & 0xFF);
            //DEBUGPRINT("%s __DEBUG__ DSP write 0x%08X", __func__, ptr_data[i]);
        }

        // Start from position 0
        err |= fs16xx_write_register16(id, FS16XX_DACEQA_REG, 0xD2);
        // Write prot coef
        for (i = 0; i < EXCUR_PROT_RAM_LEN; i++) {
            fs16xx_write_register16(id, FS16XX_DACEQWL_REG, ptr_prot_coef[i] & 0xFFFF);
            fs16xx_write_register16(id, FS16XX_DACEQWH_REG, (ptr_prot_coef[i] >> 16) & 0xFF);
            //PRINT_ERROR("%s __DEBUG__ DSP write 0x%08X", __func__, ptr_prot_coef[i]);
        }

    } else {
        PRINT_ERROR("%s failed to wait for dsp off!", __func__);
        return Fs16xx_Error_StateInvalid;
    }

    err |= fs16xx_write_register16(id, FS16XX_OTPACC_REG, 0);
    // Recover amp
    err |= fs16xx_write_register16(id, FS16XX_SYSCTRL_REG, valPwr);

    return err;
}

uint16_t fs16xx_calc_checksum(uint16_t *data, int len) {
    uint16_t crc = 0;
    uint8_t b, index;
    int i;
    if(len <= 0) return 0;
    
    for (i = 0; i < len; i++)
    {
        b = (uint8_t)(data[i] & 0xFF);
        index = (uint8_t)(crc ^ b);
        crc = (uint16_t)((crc >> 8) ^ gCrc16Table[index]);

        b = (uint8_t)((data[i] >> 8) & 0xFF);
        index = (uint8_t)(crc ^ b);
        crc = (uint16_t)((crc >> 8) ^ gCrc16Table[index]);
    }
    return crc;
}

/* fs16xx_set_preset must be called when osc is on */
Fs16xx_Error_t fs16xx_set_preset(Fs16xx_devId_t id, int preset) {
    Fs16xx_Error_t err = Fs16xx_Error_OK;
    uint16_t valPwr, valAcsCtrl, valPll, tmpC0;
    int i;
    Fs_Dev_State *dev_state = NULL;
    Fs16xx_preset_header_t *preset_header = NULL;

    PRINT("%s set preset %d.", __func__, preset);

    if (Fs16xx_Error_OK != fs16xx_handle_is_open(id)) {
        PRINT_ERROR("%s device not opened.", __func__);
        return Fs16xx_Error_NotOpen;
    }

    dev_state = fs16xx_get_dev_state_by_id(id);
    if(!dev_state) {
        PRINT_ERROR("%s invalid id, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }

    if(!dev_state->dev_preset) {
        PRINT_ERROR("%s invalid dev_preset, exit!", __func__);
        return Fs16xx_Error_StateInvalid;
    }

    if(preset != 0 && preset != 1) {
        PRINT_ERROR("%s invalid preset value! preset = %d", __func__, preset);
        return Fs16xx_Error_Bad_Parameter;
    }

    if(preset == dev_state->cur_preset) {
        PRINT("%s cur preset already set %d, exit.", __func__, preset);
        return Fs16xx_Error_OK;
    }
    i = preset;
    preset_header = &(dev_state->dev_preset->preset_header);

    if(!dev_state->dev_preset->valid || preset_header == NULL) {
        PRINT_ERROR("%s preset invalid! preset = %d preset_header = %p", __func__, preset, preset_header);
        return Fs16xx_Error_Bad_Parameter;
    }

    // Amp off
    err |= fs16xx_read_register16(id, FS16XX_SYSCTRL_REG, &valPwr);
    err |= fs16xx_write_register16(id, FS16XX_SYSCTRL_REG, valPwr & (~FS16XX_SYSCTRL_REG_AMPE));

    if(Fs16xx_Error_OK == fs16xx_wait_dsp_off(id, 0, 50)) {
        // Enable osc
        err |= fs16xx_read_register16(id, FS16XX_PLLCTRL4_REG, &valPll);
        err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, valPll | FS16XX_PLLCTRL4_REG_OSCEN);
        dev_state->tempr_sel = preset_header->tempr_sel;
        dev_state->spk_set = preset_header->spk_set;
        dev_state->adp_bst = preset_header->adp_bst[i];
        dev_state->bst_ctrl = preset_header->bst_ctrl[i];
        dev_state->acs_drc_ctrl[0] = preset_header->acs_drc[i];
        dev_state->acs_drc_ctrl[1] = preset_header->acs_drcs[i];
        dev_state->acs_drc_ctrl[2] = preset_header->acs_drcp[i];
        // TODO: TS control is not set to register
        dev_state->ts_ctrl = preset_header->ts_ctrl[i];

        err |= fs16xx_write_register16(id, FS16XX_BFLCTRL_REG, preset_header->agc_ctrl[i]);
        err |= fs16xx_write_register16(id, FS16XX_BFLSET_REG, preset_header->agc_thd[i]);
        err |= fs16xx_write_register16(id, FS16XX_SQC_REG, preset_header->agc_para[i]);
        err |= fs16xx_write_register16(id, FS16XX_AGC_REG, preset_header->agc_mg[i]);
        err |= fs16xx_write_register16(id, FS16XX_DRPARA_REG, preset_header->dac_para1[i]);
        err |= fs16xx_write_register16(id, FS16XX_ACSDRC_REG, dev_state->acs_drc_ctrl[0]);
        err |= fs16xx_write_register16(id, FS16XX_ACSDRCS_REG, dev_state->acs_drc_ctrl[1]);
        // acs drcp not used.
        // err |= fs16xx_write_register16(id, FS16XX_ACSDRCP_REG, dev_state->acs_drc_ctrl[2]);
        err |= fs16xx_write_register16(id, FS16XX_ADPBST_REG, dev_state->adp_bst);

        if(preset == 0) {
            err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, dev_state->bst_ctrl);
            valAcsCtrl = (uint16_t)FS16XX_ACSCTRL_REG_DEFAULT;
        } else {
            tmpC0 = dev_state->bst_ctrl & (~FS16XX_BSTCTRL_REG_MODE_CTRL_MSK);
            tmpC0 |= (0x1 << FS16XX_BSTCTRL_REG_MODE_CTRL_POS);
            err |= fs16xx_write_register16(id, FS16XX_BSTCTRL_REG, tmpC0);
            valAcsCtrl = (uint16_t)(FS16XX_ACSCTRL_REG_DEFAULT | FS16XX_ACSCTRL_REG_ACS_COE_SEL_MSK);
        }

        err |= fs16xx_write_register16(id, FS16XX_ACSCTRL_REG, valAcsCtrl);
        dev_state->cur_preset = (int)preset;

        err |= fs16xx_write_register16(id, FS16XX_PLLCTRL4_REG, valPll);
    } else {
        PRINT_ERROR("%s failed to wait for dsp off!", __func__);
    }

    err |= fs16xx_write_register16(id, FS16XX_SYSCTRL_REG, valPwr);

    PRINT("%s exit err=%d. __DEBUG__ boost 0x%04X", __func__, err,
        dev_state->bst_ctrl);
    return err;
}

