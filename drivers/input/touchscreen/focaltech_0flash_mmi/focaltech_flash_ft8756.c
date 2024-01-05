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
#define FTS_FLASH_PACKET_LENGTH_SPI                 (32 * 1024 - 16)

#define FTS_CMD_ECC_LENGTH_MAX                      32766
#define FTS_ROMBOOT_CMD_ECC_FINISH_OK               0xA5


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
    u8 cmd[2] = { 0 };

    FTS_INFO("enter into boot environment");
    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /* hardware tp reset to boot */
        fts_fwupg_hardware_reset_to_boot();

        /* enter into boot & check boot id*/
        for (j = 0; j < FTS_READ_BOOT_ID_TIMEOUT; j++) {
            cmd[0] = FTS_CMD_START1;
            ret = fts_write(cmd, 1);
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


static int fts_dpram_write(u32 saddr, const u8 *buf, u32 len, bool wpram)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 *cmd = NULL;
    u32 addr = 0;
    u32 baseaddr = wpram ? FTS_PRAM_SADDR : FTS_DRAM_SADDR;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number = 0;
    u32 packet_len = 0;
    u32 packet_size = FTS_FLASH_PACKET_LENGTH_SPI;

    FTS_INFO("dpram write");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_APP)) {
        FTS_ERROR("fw length(%d) fail", len);
        return -EINVAL;
    }

    cmd = vmalloc(packet_size + FTS_CMD_WRITE_LEN);
    if (NULL == cmd) {
        FTS_ERROR("malloc memory for pram write buffer fail");
        return -ENOMEM;
    }
    memset(cmd, 0, packet_size + FTS_CMD_WRITE_LEN);

    packet_number = len / packet_size;
    remainder = len % packet_size;
    if (remainder > 0)
        packet_number++;
    packet_len = packet_size;
    FTS_INFO("write data, num:%d remainder:%d", packet_number, remainder);

    for (i = 0; i < packet_number; i++) {
        offset = i * packet_size;
        addr = saddr + offset + baseaddr;
        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;

        /* set pram address */
        cmd[0] = FTS_ROMBOOT_CMD_SET_PRAM_ADDR;
        cmd[1] = BYTE_OFF_16(addr);
        cmd[2] = BYTE_OFF_8(addr);
        cmd[3] = BYTE_OFF_0(addr);
        ret = fts_write(&cmd[0], FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN);
        if (ret < 0) {
            FTS_ERROR("set pram(%d) addr(%d) fail", i, addr);
            goto write_pram_err;
        }

        /* write pram data */
        cmd[0] = FTS_ROMBOOT_CMD_WRITE;
        for (j = 0; j < packet_len; j++) {
            cmd[1 + j] = buf[offset + j];
        }
        ret = fts_write(&cmd[0], 1 + packet_len);
        if (ret < 0) {
            FTS_ERROR("write fw to pram(%d) fail", i);
            goto write_pram_err;
        }
    }

write_pram_err:
    if (cmd) {
        vfree(cmd);
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
    mdelay(2);

    /* wait boot calculate ecc finish */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        ret = fts_read(cmd, 1, value, 1);
        if (ret < 0) {
            FTS_ERROR("ecc finish cmd fail");
            return ret;
        }
        if (FTS_ROMBOOT_CMD_ECC_FINISH_OK == value[0])
            break;
        mdelay(1);
    }
    if (i >= FTS_ECC_FINISH_TIMEOUT) {
        FTS_ERROR("wait ecc finish timeout,ecc_finish=%x", value[0]);
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

static int fts_ecc_check(const u8 *buf, u32 len, u32 ecc_saddr)
{
    int ret = 0;
    int i = 0;
    u16 ecc_in_host = 0;
    u16 ecc_in_tp = 0;
    int packet_length = 0;
    int packet_number = 0;
    int packet_remainder = 0;
    int offset = 0;
    u32 packet_size = FTS_CMD_ECC_LENGTH_MAX;

    packet_number = len / packet_size;
    packet_remainder = len % packet_size;
    if (packet_remainder)
        packet_number++;
    packet_length = packet_size;

    for (i = 0; i < packet_number; i++) {
        /* last packet */
        if ((i == (packet_number - 1)) && packet_remainder)
            packet_length = packet_remainder;

        ret = fts_ecc_cal_host(buf + offset, packet_length, &ecc_in_host);
        if (ret < 0) {
            FTS_ERROR("ecc in host calc fail");
            return ret;
        }

        ret = fts_ecc_cal_tp(ecc_saddr + offset, packet_length, &ecc_in_tp);
        if (ret < 0) {
            FTS_ERROR("ecc in tp calc fail");
            return ret;
        }

        FTS_DEBUG("ecc in tp:%04x,host:%04x,i:%d", ecc_in_tp, ecc_in_host, i);
        if (ecc_in_tp != ecc_in_host) {
            FTS_ERROR("ecc_in_tp(%x) != ecc_in_host(%x), ecc check fail",
                      ecc_in_tp, ecc_in_host);
            return -EIO;
        }

        offset += packet_length;
    }

    return 0;
}

static int fts_pram_write_ecc(const u8 *buf, u32 len)
{
    int ret = 0;
    u32 pram_app_size = 0;
    u16 code_len = 0;
    u16 code_len_n = 0;
    u32 pram_start_addr = 0;

    FTS_INFO("begin to write pram app(bin len:%d)", len);
    /* get pram app length */
    code_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
               + buf[FTS_APP_INFO_OFFSET + 1];
    code_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 2] << 8)
                 + buf[FTS_APP_INFO_OFFSET + 3];
    if ((code_len + code_len_n) != 0xFFFF) {
        FTS_ERROR("pram code len(%x %x) fail", code_len, code_len_n);
        return -EINVAL;
    }

    pram_app_size = (u32)code_len;
    pram_app_size = pram_app_size * 2;
    if ((pram_app_size < FTS_MIN_LEN) || (pram_app_size > FTS_MAX_LEN_APP)) {
        FTS_ERROR("pram app length(%d) is invalid", pram_app_size);
        return -EINVAL;
    }

    FTS_INFO("pram app length in fact:%d", pram_app_size);
    /* write pram */
    ret = fts_dpram_write(pram_start_addr, buf, pram_app_size, true);
    if (ret < 0) {
        FTS_ERROR("write pram fail");
        return ret;
    }

    /* check ecc */
    ret = fts_ecc_check(buf, pram_app_size, pram_start_addr);
    if (ret < 0) {
        FTS_ERROR("pram ecc check fail");
        return ret;
    }

    FTS_INFO("pram app write successfully");
    return 0;
}

static int fts_dram_write_ecc(const u8 *buf, u32 len)
{
    int ret = 0;
    u32 dram_size = 0;
    u32 pram_app_size = 0;
    u32 dram_start_addr = 0;
    u16 const_len = 0;
    u16 const_len_n = 0;
    const u8 *dram_buf = NULL;

    FTS_INFO("begin to write dram data(bin len:%d)", len);
    /* get dram data length */
    const_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0x8] << 8)
                + buf[FTS_APP_INFO_OFFSET + 0x9];
    const_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 0x0A] << 8)
                  + buf[FTS_APP_INFO_OFFSET + 0x0B];
    if (((const_len + const_len_n) != 0xFFFF) || (const_len == 0)) {
        FTS_INFO("no support dram,const len(%x %x)", const_len, const_len_n);
        return 0;
    }

    dram_size = ((u32)const_len) * 2;
    if ((dram_size <= 0) || (dram_size > FTS_MAX_LEN_APP_PARAMS)) {
        FTS_ERROR("dram data length(%d) is invalid", dram_size);
        return -EINVAL;
    }

    pram_app_size = ((u32)(((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
                           + buf[FTS_APP_INFO_OFFSET + 1])) * 2;

    dram_buf = buf + pram_app_size;
    FTS_INFO("dram buf length in fact:%d,offset:%d", dram_size, pram_app_size);
    /* write pram */
    ret = fts_dpram_write(dram_start_addr, dram_buf, dram_size, false);
    if (ret < 0) {
        FTS_ERROR("write dram fail");
        return ret;
    }

    /* check ecc */
    ret = fts_ecc_check(dram_buf, dram_size, dram_start_addr);
    if (ret < 0) {
        FTS_ERROR("dram ecc check fail");
        return ret;
    }

    FTS_INFO("dram data write successfully");
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

    FTS_INFO("begin to write and start fw(bin len:%d)", len);
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
    ret = fts_pram_write_ecc(buf, len);
    if (ret < 0) {
        FTS_ERROR("write pram fail");
        return ret;
    }

    /* write dram */
    ret = fts_dram_write_ecc(buf, len);
    if (ret < 0) {
        FTS_ERROR("write dram fail");
        return ret;
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

