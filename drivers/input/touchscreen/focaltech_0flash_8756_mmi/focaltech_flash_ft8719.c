/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2019, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_flash.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-12-06
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include "focaltech_flash.h"

/*****************************************************************************
* Static variables
*****************************************************************************/
#define FTS_READ_BOOT_ID_TIMEOUT                    3
#define FTS_FLASH_PACKET_LENGTH_SPI_LOW             (4 * 1024 - 4)
#define FTS_FLASH_PACKET_LENGTH_SPI                 (64 * 1024)

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
static int fts_fwupg_hardware_reset_to_boot(void)
{
    fts_reset_proc(0);
    mdelay(8);

    return 0;
}

static int fts_enter_into_boot(void)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 cmd = 0;

    FTS_INFO("enter into boot environment");
    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /* hardware tp reset to boot */
        fts_fwupg_hardware_reset_to_boot();

        /* enter into boot & check boot id*/
        for (j = 0; j < FTS_READ_BOOT_ID_TIMEOUT; j++) {
            cmd = FTS_CMD_START1;
            ret = fts_write(&cmd, 1);
            if (ret >= 0) {
                mdelay(8);
                ret = fts_check_bootid();
                if (0 == ret) {
                    FTS_INFO("boot id check pass, retry=%d", i);
                    return 0;
                }
            }
        }
    }

    return -EIO;
}

static bool fts_check_fast_download(void)
{
    int ret = 0;
    u8 cmd[6] = {0xF2, 0x00, 0x78, 0x0A, 0x00, 0x02};
    u8 value = 0;
    u8 value2[2] = { 0 };

    ret = fts_read_reg(0xdb, &value);
    if (ret < 0) {
        FTS_ERROR("read 0xdb fail");
        goto read_err;
    }

    ret = fts_read(cmd, 6, value2, 2);
    if (ret < 0) {
        FTS_ERROR("read f2 fail");
        goto read_err;
    }

    FTS_INFO("0xdb = 0x%x, 0xF2 = 0x%x", value, value2[0]);
    if ((value >= 0x18) && (value2[0] == 0x55)) {
        FTS_INFO("IC support fast-download");
        return true;
    }

read_err:
    FTS_INFO("IC not support fast-download");
    return false;
}

static int fts_pram_write(u32 saddr, const u8 *buf, u32 len)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 *cmd;
    u32 addr = 0;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number;
    u32 packet_len = 0;
    u32 packet_size = FTS_FLASH_PACKET_LENGTH_SPI;
    bool fd_support = true;

    FTS_INFO("pram write");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_APP)) {
        FTS_ERROR("fw length(%d) fail", len);
        return -EINVAL;
    }

    fd_support = fts_check_fast_download();
    if (!fd_support)
        packet_size = FTS_FLASH_PACKET_LENGTH_SPI_LOW;
    cmd = kzalloc(packet_size + FTS_CMD_WRITE_LEN, GFP_KERNEL);
    if (NULL == cmd) {
        FTS_ERROR("malloc memory for pram write buffer fail");
        return -ENOMEM;
    }

    packet_number = len / packet_size;
    remainder = len % packet_size;
    if (remainder > 0)
        packet_number++;
    packet_len = packet_size;

    cmd[0] = FTS_ROMBOOT_CMD_WRITE;
    for (i = 0; i < packet_number; i++) {
        offset = i * packet_size;
        addr = saddr + offset;
        cmd[1] = BYTE_OFF_16(addr);
        cmd[2] = BYTE_OFF_8(addr);
        cmd[3] = BYTE_OFF_0(addr);

        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;
        cmd[4] = BYTE_OFF_8(packet_len);
        cmd[5] = BYTE_OFF_0(packet_len);

        for (j = 0; j < packet_len; j++) {
            cmd[FTS_CMD_WRITE_LEN + j] = buf[offset + j];
        }
        ret = fts_write(&cmd[0], FTS_CMD_WRITE_LEN + packet_len);
        if (ret < 0) {
            FTS_ERROR("write fw to pram(%d) fail", i);
            goto write_pram_err;
        }

        if (!fd_support)
            mdelay(3);
    }

write_pram_err:
    if (cmd) {
        kfree(cmd);
        cmd = NULL;
    }
    return ret;
}

static int fts_ecc_cal_tp(u32 ecc_saddr, u32 ecc_len, u16 *ecc_value)
{
    int ret = 0;
    int i = 0;
    u8 cmd[FTS_ROMBOOT_CMD_ECC_NEW_LEN] = { 0 };
    u8 value[2] = { 0 };

    FTS_INFO("ecc calc in tp");
    cmd[0] = FTS_ROMBOOT_CMD_ECC;
    cmd[1] = BYTE_OFF_16(ecc_saddr);
    cmd[2] = BYTE_OFF_8(ecc_saddr);
    cmd[3] = BYTE_OFF_0(ecc_saddr);
    cmd[4] = BYTE_OFF_16(ecc_len);
    cmd[5] = BYTE_OFF_8(ecc_len);
    cmd[6] = BYTE_OFF_0(ecc_len);

    /* make boot to calculate ecc in pram */
    ret = fts_write(cmd, FTS_ROMBOOT_CMD_ECC_NEW_LEN);
    if (ret < 0) {
        FTS_ERROR("ecc calc cmd fail");
        return ret;
    }
    mdelay(3);

    /* wait boot calculate ecc finish */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        ret = fts_read(cmd, 1, value, 1);
        if (ret < 0) {
            FTS_ERROR("ecc finish cmd fail");
            return ret;
        }
        if (0 == value[0])
            break;
        mdelay(1);
    }
    if (i >= FTS_ECC_FINISH_TIMEOUT) {
        FTS_ERROR("wait ecc finish timeout");
        return -EIO;
    }

    /* get ecc value calculate in boot */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_READ;
    ret = fts_read(cmd, 1, value, 2);
    if (ret < 0) {
        FTS_ERROR("ecc read cmd fail");
        return ret;
    }

    *ecc_value = ((u16)(value[0] << 8) + value[1]) & 0x0000FFFF;
    return 0;
}

static int fts_ecc_cal_host(const u8 *data, u32 data_len, u16 *ecc_value)
{
    u16 ecc = 0;
    u16 i = 0;
    u16 j = 0;
    u16 al2_fcs_coef = AL2_FCS_COEF;

    for (i = 0; i < data_len; i += 2 ) {
        ecc ^= ((data[i] << 8) | (data[i + 1]));
        for (j = 0; j < 16; j ++) {
            if (ecc & 0x01)
                ecc = (u16)((ecc >> 1) ^ al2_fcs_coef);
            else
                ecc >>= 1;
        }
    }

    *ecc_value = ecc & 0x0000FFFF;
    return 0;
}

static int fts_pram_start(void)
{
    int ret = 0;
    u8 cmd = FTS_ROMBOOT_CMD_START_APP;

    FTS_INFO("remap to start pram");
    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("write start pram cmd fail");
        return ret;
    }

    return 0;
}

/*
 * description: download fw to IC and run
 *
 * param - buf: const, fw data buffer
 *         len: length of fw
 *
 * return 0 if success, otherwise return error code
 */
int fts_fw_write_start(const u8 *buf, u32 len, bool need_reset)
{
    int ret = 0;
    u16 ecc_in_host = 0;
    u16 ecc_in_tp = 0;
    u32 fw_start_addr = 0;
    u32 fw_len = 0;
    u16 code_len = 0;
    u16 code_len_n = 0;

    FTS_INFO("begin to write and start fw(bin len:%d)", len);
    /* get app length */
    code_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
               + buf[FTS_APP_INFO_OFFSET + 1];
    code_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 2] << 8)
                 + buf[FTS_APP_INFO_OFFSET + 3];
    if ((code_len + code_len_n) != 0xFFFF) {
        FTS_ERROR("code len(%x %x) fail", code_len, code_len_n);
        return -EINVAL;
    }

    fw_len = (u32)code_len;
    if ((fw_len < FTS_MIN_LEN) || (fw_len > FTS_MAX_LEN_APP)) {
        FTS_ERROR("fw length(%d) is invalid", fw_len);
        return -EINVAL;
    }

    FTS_INFO("fw length in fact:%d", fw_len);
    fts_data->fw_is_running = false;

    if (need_reset) {
        /* enter into boot environment */
        ret = fts_enter_into_boot();
        if (ret < 0) {
            FTS_ERROR("enter into boot environment fail");
            return ret;
        }
    }

    /* write pram */
    ret = fts_pram_write(fw_start_addr, buf, fw_len);
    if (ret < 0) {
        FTS_ERROR("write pram fail");
        return ret;
    }

    /* ecc check */
    ret = fts_ecc_cal_host(buf, fw_len, &ecc_in_host);
    if (ret < 0) {
        FTS_ERROR("ecc in host calc fail");
        return ret;
    }

    ret = fts_ecc_cal_tp(fw_start_addr, fw_len, &ecc_in_tp);
    if (ret < 0) {
        FTS_ERROR("ecc in tp calc fail");
        return ret;
    }

    FTS_INFO("ecc in tp:%04x host:%04x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        return -EIO;
    }

    /* remap pram and run fw */
    ret = fts_pram_start();
    if (ret < 0) {
        FTS_ERROR("pram start fail");
        return ret;
    }

    fts_data->fw_is_running = true;
    FTS_INFO("fw download successfully");
    return 0;
}

