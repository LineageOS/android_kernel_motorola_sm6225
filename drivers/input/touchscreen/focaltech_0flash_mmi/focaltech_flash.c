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
#define FTS_FW_REQUEST_SUPPORT                      1
/* Example: focaltech_ts_fw_tianma.bin */
#define FTS_FW_NAME_PREX_WITH_REQUEST               ""

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
u8 fw_file[] = {
};

u8 fw_file2[] = {
};

u8 fw_file3[] = {
};

struct upgrade_module module_list[] = {
    {FTS_MODULE_ID, FTS_MODULE_NAME, fw_file, sizeof(fw_file)},
    {FTS_MODULE2_ID, FTS_MODULE2_NAME, fw_file2, sizeof(fw_file2)},
    {FTS_MODULE3_ID, FTS_MODULE3_NAME, fw_file3, sizeof(fw_file3)},
};

struct fts_upgrade *fwupgrade;

int fts_check_bootid(void)
{
    int ret = 0;
    u8 cmd = 0;
    u8 id[2] = { 0 };
    struct ft_chip_t *chip_id = &fts_data->ic_info.ids;

    cmd = FTS_CMD_READ_ID;
    ret = fts_read(&cmd, 1, id, 2);
    if (ret < 0) {
        FTS_ERROR("read boot id fail");
        return ret;
    }

    FTS_INFO("read boot id:0x%02x 0x%02x", id[0], id[1]);
    if ((chip_id->chip_idh == id[0]) && (chip_id->chip_idl == id[1])) {
        return 0;
    }

    return -EIO;
}

static int fts_fw_download(const u8 *buf, u32 len, bool need_reset)
{
    int ret = 0;
    int i = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_INFO("fw upgrade download function");
    if (!buf || (len < FTS_MIN_LEN)) {
        FTS_ERROR("fw/len(%d) is invalid", len);
        return -EINVAL;
    }

    ts_data->fw_loading = 1;
    fts_irq_disable();
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(DISABLE);
#endif

    for (i = 0; i < 3; i++) {
        FTS_INFO("fw download times:%d", i + 1);
        ret = fts_fw_write_start(buf, len, need_reset);
        if (0 == ret)
            break;
    }
    if (i >= 3) {
        FTS_ERROR("fw download fail");
        ret = -EIO;
        goto err_fw_download;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(ENABLE);
#endif

    ret = 0;
err_fw_download:
    fts_irq_enable();
    ts_data->fw_loading = 0;

    return ret;
    }

static int fts_read_file(char *file_name, u8 **file_buf)
{
    int ret = 0;
    char file_path[FILE_NAME_LENGTH] = { 0 };
    struct file *filp = NULL;
    struct inode *inode;
    mm_segment_t old_fs;
    loff_t pos;
    loff_t file_len = 0;

    if ((NULL == file_name) || (NULL == file_buf)) {
        FTS_ERROR("filename/filebuf is NULL");
        return -EINVAL;
    }

    snprintf(file_path, FILE_NAME_LENGTH, "%s%s", FTS_FW_BIN_FILEPATH, file_name);
    filp = filp_open(file_path, O_RDONLY, 0);
    if (IS_ERR(filp)) {
        FTS_ERROR("open %s file fail", file_path);
        return -ENOENT;
    }

#if 1
    inode = filp->f_inode;
#else
    /* reserved for linux earlier verion */
    inode = filp->f_dentry->d_inode;
#endif

    file_len = inode->i_size;
    *file_buf = (u8 *)vmalloc(file_len);
    if (NULL == *file_buf) {
        FTS_ERROR("file buf malloc fail");
        filp_close(filp, NULL);
        return -ENOMEM;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    ret = vfs_read(filp, *file_buf, file_len , &pos);
    if (ret < 0)
        FTS_ERROR("read file fail");
    FTS_INFO("file len:%d read len:%d pos:%d", (u32)file_len, ret, (u32)pos);
    filp_close(filp, NULL);
    set_fs(old_fs);

    return ret;
}

int fts_upgrade_bin(char *fw_name, bool force)
{
    int ret = 0;
    u32 fw_file_len = 0;
    u8 *fw_file_buf = NULL;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("start upgrade with fw bin");
    if (NULL == upg) {
        FTS_ERROR("upgrade/func is null");
        return -EINVAL;
    }

    if (upg->ts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    ret = fts_read_file(fw_name, &fw_file_buf);
    if ((ret < 0) || (ret < FTS_MIN_LEN) || (ret > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("read fw bin file(sdcard) fail, len:%d", fw_file_len);
        goto err_bin;
    }

    fw_file_len = ret;
    FTS_INFO("fw bin file len:%d", fw_file_len);
    ret = fts_fw_download(fw_file_buf, fw_file_len, true);
    if (ret < 0) {
        FTS_ERROR("upgrade fw bin failed");
        goto err_bin;
    }

    FTS_INFO("upgrade fw bin success");

err_bin:
    if (fw_file_buf) {
        vfree(fw_file_buf);
        fw_file_buf = NULL;
    }
    return ret;
}


int fts_enter_test_environment(bool test_state)
{
    int ret = 0;
    u8 detach_flag = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("fw test download function");
    if (!upg) {
        FTS_ERROR("upg is null");
        return -EINVAL;
    }

    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8756) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8009)
    if (!upg->fw || (upg->fw_length <= FTS_MAX_LEN_APP_FT8756)) {
#else
    if (!upg->fw || (upg->fw_length <= FTS_MAX_LEN_APP)) {
#endif
        FTS_INFO("not multi-app");
        return 0;
    }

    if (test_state) {
#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8756) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8009)
        ret = fts_fw_download(upg->fw + FTS_MAX_LEN_APP_FT8756, upg->fw_length, true);
#else
        ret = fts_fw_download(upg->fw + FTS_MAX_LEN_APP, upg->fw_length, true);
#endif
    } else {
        ret = fts_fw_download(upg->fw, upg->fw_length, true);
    }
    if (ret < 0) {
        FTS_ERROR("fw(app2) download fail");
        return ret;
    }

    msleep(50);
    ret = fts_read_reg(FTS_REG_FACTORY_MODE_DETACH_FLAG, &detach_flag);
    FTS_INFO("regb4:0x%02x", detach_flag);

    return 0;
}

int fts_fw_resume(bool need_reset)
{
    int ret = 0;
    struct fts_upgrade *upg = fwupgrade;
    const struct firmware *fw = NULL;
    char fwname[FILE_NAME_LENGTH] = { 0 };

    FTS_INFO("fw upgrade resume function");
    if (!upg || !upg->fw) {
        FTS_ERROR("upg/fw is null");
        return -EINVAL;
    }

    if (upg->ts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    snprintf(fwname, FILE_NAME_LENGTH, "%s%s.bin", \
             FTS_FW_NAME_PREX_WITH_REQUEST, upg->module_info->vendor_name);

    /* 1. request firmware */
    ret = request_firmware(&fw, fwname, upg->ts_data->dev);
    if (ret != 0) {
        FTS_ERROR("%s:firmware(%s) request fail,ret=%d\n",
                  __func__, fwname, ret);
        FTS_INFO("download fw from bootimage");
        ret = fts_fw_download(upg->fw, upg->fw_length, need_reset);
    } else {
        FTS_INFO("firmware(%s) request successfully", fwname);
        ret = fts_fw_download(fw->data, fw->size, need_reset);
    }
    if (ret < 0) {
        FTS_ERROR("fw resume download failed");
        return ret;
    }

    /* update to newest fw if needed */
    if (fw != NULL) {
        u8 *tmpbuf = NULL;
        if (upg->fw_from_request)
            vfree(upg->fw);
        tmpbuf = vmalloc(fw->size);
        if (NULL == tmpbuf) {
            FTS_ERROR("fw buffer vmalloc fail");
            ret = -ENOMEM;
            goto FTS_FW_RESUME_VMALLOC_ERROR;
        } else {
            memcpy(tmpbuf, fw->data, fw->size);
            upg->fw = tmpbuf;
            upg->fw_length = fw->size;
            upg->fw_from_request = 1;
        }
    }

FTS_FW_RESUME_VMALLOC_ERROR:
    if (fw != NULL) {
        release_firmware(fw);
        fw = NULL;
    }

    return ret;
}

int fts_fw_recovery(void)
{
    int ret = 0;
    u8 boot_state = 0;
    u8 chip_id = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("check if boot recovery");
    if (!upg || !upg->ts_data) {
        FTS_ERROR("upg/ts_data is null");
        return -EINVAL;
    }

    if (upg->ts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    upg->ts_data->fw_is_running = false;
    if (!upg->ts_data->force_reflash) {
        ret = fts_check_bootid();
        if (ret < 0) {
            FTS_ERROR("check boot id fail");
            upg->ts_data->fw_is_running = true;
            return ret;
        }

        ret = fts_read_reg(0xD0, &boot_state);
        if (ret < 0) {
            FTS_ERROR("read boot state failed, ret=%d", ret);
            upg->ts_data->fw_is_running = true;
            return ret;
        }

#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8756) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8006S_AA) || \
	defined (CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8009)
        if (boot_state != 0x01) {
#else
        if (boot_state != 0x02) {
#endif
            FTS_INFO("not in boot mode(0x%x),exit", boot_state);
            upg->ts_data->fw_is_running = true;
            return -EIO;
        }

        FTS_INFO("abnormal situation,need download fw");
    }

    ret = fts_fw_resume(false);
    if (ret < 0) {
        FTS_ERROR("fts_fw_resume fail");
        return ret;
    }
    msleep(10);
    ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id);
    FTS_INFO("read chip id:0x%02x", chip_id);

    fts_tp_state_recovery(upg->ts_data);

    FTS_INFO("boot recovery pass");
    return ret;
}

int fts_fw_update_vendor_name(const char* name) {
    struct fts_upgrade *upg = fwupgrade;
    char* pos;
    int len;

    if (!upg || !upg->module_info) {
        FTS_ERROR("upgrade struct not init");
        return -EINVAL;
    }

    len = strlen(name);
    if (len > FILE_NAME_LENGTH) {
        FTS_ERROR("vendor name is too long");
        return -EINVAL;
    }
    pos = strstr(name, ".bin");
    if (pos == NULL)
        snprintf(upg->module_info->vendor_name, FILE_NAME_LENGTH, "%s", name);
    else {
        len = len - strlen(".bin");
        snprintf(upg->module_info->vendor_name, len + 1, "%s", name);
    }
    return 0;
}

static int fts_fwupg_get_module_info(struct fts_upgrade *upg)
{
    int i = 0;
    struct upgrade_module *info = &module_list[0];

    if (!upg || !upg->ts_data) {
        FTS_ERROR("upg/ts_data is null");
        return -EINVAL;
    }

    if (FTS_GET_MODULE_NUM > 1) {
        FTS_INFO("module id:%04x", upg->module_id);
        for (i = 0; i < FTS_GET_MODULE_NUM; i++) {
            info = &module_list[i];
            if (upg->module_id == info->id) {
                FTS_INFO("module id match, get fw file successfully");
                break;
            }
        }
        if (i >= FTS_GET_MODULE_NUM) {
            FTS_ERROR("no module id match, don't get file");
            return -ENODATA;
        }
    }

    upg->module_info = info;
    return 0;
}

static int fts_get_fw_file_via_request_firmware(struct fts_upgrade *upg)
{
    int ret = 0;
    const struct firmware *fw = NULL;
    u8 *tmpbuf = NULL;
    char fwname[FILE_NAME_LENGTH] = { 0 };

    snprintf(fwname, FILE_NAME_LENGTH, "%s%s.bin", \
             FTS_FW_NAME_PREX_WITH_REQUEST, \
             upg->module_info->vendor_name);

    ret = request_firmware(&fw, fwname, upg->ts_data->dev);
    if (0 == ret) {
        FTS_INFO("firmware(%s) request successfully", fwname);
        tmpbuf = vmalloc(fw->size);
        if (NULL == tmpbuf) {
            FTS_ERROR("fw buffer vmalloc fail");
            ret = -ENOMEM;
        } else {
            memcpy(tmpbuf, fw->data, fw->size);
            upg->fw = tmpbuf;
            upg->fw_length = fw->size;
            upg->fw_from_request = 1;
        }
    } else {
        FTS_INFO("firmware(%s) request fail,ret=%d", fwname, ret);
    }

    if (fw != NULL) {
        release_firmware(fw);
        fw = NULL;
    }

    return ret;
}

static int fts_get_fw_file_via_i(struct fts_upgrade *upg)
{
    upg->fw = upg->module_info->fw_file;
    upg->fw_length = upg->module_info->fw_len;
    upg->fw_from_request = 0;

    return 0;
}

/*****************************************************************************
 *  Name: fts_fwupg_get_fw_file
 *  Brief: get fw image/file,
 *         If support muitl modules, please set FTS_GET_MODULE_NUM, and FTS_-
 *         MODULE_ID/FTS_MODULE_NAME;
 *         If get fw via .i file, please set FTS_FW_REQUEST_SUPPORT=0, and F-
 *         TS_MODULE_ID; will use module id to distingwish different modules;
 *         If get fw via reques_firmware(), please set FTS_FW_REQUEST_SUPPORT
 *         =1, and FTS_MODULE_NAME; fw file name will be composed of "focalt-
 *         ech_ts_fw_" & FTS_VENDOR_NAME;
 *
 *         If have flash, module_id=vendor_id, If non-flash,module_id need
 *         transfer from LCD driver(gpio or lcm_id or ...);
 *  Input:
 *  Output:
 *  Return: return 0 if success, otherwise return error code
 *****************************************************************************/
static int fts_fwupg_get_fw_file(struct fts_upgrade *upg)
{
    int ret = 0;
    bool get_fw_i_flag = false;

    FTS_DEBUG("get upgrade fw file");
    if (!upg || !upg->ts_data) {
        FTS_ERROR("upg/ts_data is null");
        return -EINVAL;
    }

    ret = fts_fwupg_get_module_info(upg);
    if ((ret < 0) || (!upg->module_info)) {
        FTS_ERROR("get module info fail");
        return ret;
    }

    if (FTS_FW_REQUEST_SUPPORT) {
        msleep(500);
        ret = fts_get_fw_file_via_request_firmware(upg);
        if (ret != 0) {
            get_fw_i_flag = true;
        }
    } else {
        get_fw_i_flag = true;
    }

    if (get_fw_i_flag) {
        ret = fts_get_fw_file_via_i(upg);
    }

    FTS_INFO("upgrade fw file len:%d", upg->fw_length);
    if ((upg->fw_length < FTS_MIN_LEN)
        || (upg->fw_length > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw file len(%d) fail", upg->fw_length);
        return -ENODATA;
    }

    return ret;
}

static void fts_fwupg_work(struct work_struct *work)
{
    int ret = 0;
    u8 chip_id = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("fw upgrade work function");
    if (!upg || !upg->ts_data) {
        FTS_ERROR("upg/ts_data is null");
        return ;
    }

    /* get fw */
    ret = fts_fwupg_get_fw_file(upg);
    if (ret < 0) {
        FTS_ERROR("get file fail, can't upgrade");
        return ;
    }

#if !FTS_AUTO_UPGRADE_EN
    FTS_INFO("FTS_AUTO_UPGRADE_EN is disabled, not upgrade when power on");
    return ;
#endif

    if (upg->ts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return ;
    }

    ret = fts_fw_download(upg->fw, upg->fw_length, true);
    if (ret < 0) {
        FTS_ERROR("fw auto download failed");
    } else {
        msleep(50);
        ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id);
        FTS_INFO("read chip id:0x%02x", chip_id);
    }
}

int fts_fwupg_init(struct fts_ts_data *ts_data)
{
    FTS_INFO("fw upgrade init function");

    if (!ts_data || !ts_data->ts_workqueue) {
        FTS_ERROR("ts_data/workqueue is NULL, can't run upgrade function");
        return -EINVAL;
    }

    fwupgrade = (struct fts_upgrade *)kzalloc(sizeof(*fwupgrade), GFP_KERNEL);
    if (NULL == fwupgrade) {
        FTS_ERROR("malloc memory for upgrade fail");
        return -ENOMEM;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(DISABLE);
#endif

    fwupgrade->ts_data = ts_data;
    INIT_WORK(&ts_data->fwupg_work, fts_fwupg_work);
    queue_work(ts_data->ts_workqueue, &ts_data->fwupg_work);

    return 0;
}

int fts_fwupg_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    if (fwupgrade) {
        if (fwupgrade->fw_from_request) {
            vfree(fwupgrade->fw);
            fwupgrade->fw = NULL;
        }

        kfree(fwupgrade);
        fwupgrade = NULL;
    }
    FTS_FUNC_EXIT();
    return 0;
}
