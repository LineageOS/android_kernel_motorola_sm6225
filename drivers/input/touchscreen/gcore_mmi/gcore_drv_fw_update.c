/*
 * GalaxyCore touchscreen driver
 *
 * Copyright (C) 2021 GalaxyCore Incorporated
 *
 * Copyright (C) 2021 Neo Chen <neo_chen@gcoreinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <uapi/linux/sched/types.h>
#include <linux/firmware.h>
#include "gcore_drv_common.h"
#include "gcore_drv_firmware.h"


#define FW_BIN_NAME "GCORE_FW.bin"

struct gcore_dev *gdev_fwu;
struct task_struct *fwu_thread;

DECLARE_COMPLETION(fw_update_complete);
u8 g_update_running;

static int gcore_fw_update_fn_init(struct gcore_dev *gdev);
static void gcore_fw_update_fn_remove(struct gcore_dev *gdev);

struct gcore_exp_fn fw_update_fn = {
	.fn_type = GCORE_FW_UPDATE,
	.wait_int = false,
	.event_flag = false,
	.init = gcore_fw_update_fn_init,
	.remove = gcore_fw_update_fn_remove,
};


#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
#define FW_OP_BIN_NAME "GC7371_Flash_Op.bin"
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
#define FW_OP_BIN_NAME "GC7271_Flash_Op.bin"
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202)
#define FW_OP_BIN_NAME "GC7202_Flash_Op.bin"
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
#define FW_OP_BIN_NAME "GC7372_Flash_Op.bin"
#endif

enum FLASH_OP {
	READ_FLASH_ID,
	FLASH_ERASE,
	FLASH_PROGRAM,
	FLASH_READ
};

enum UPDATE_STATUS {
	UPDATE_STATUS_IDLE,
	UPDATE_STATUS_RUNNING
};

struct fw_update_info {
	int update_type;
	int status;
	int progress;
	int max_progress;

	u8 *fw_data;
	u32 fw_length;
	u32 fw_burn_progress;

	u8 *flash_op_data;
	u32 flash_op_length;
	u8 *flash_op_buf;
};

struct fw_update_info update_info = {
	.status = UPDATE_STATUS_IDLE,
	.progress = 0,
	.max_progress = 7,
};

#define GC7XXX_REQUEST_SIZE           64

#define GC7XXX_REPLY_SIZE             64

#define GC7XXX_MAX_TRANSFER_DATA      (GC7XXX_REQUEST_SIZE - 8)

#define FLASH_OP_TRANSFER_SIZE        1024

#define FW_VERSION_ADDR               0xFFF4

#endif


#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_RK_PLATFORM
extern int dsi_send_packet(unsigned int id, unsigned char *packet, u32 n);
#endif


#ifdef CONFIG_ENABLE_CHIP_TYPE_GC7271
int gcore_send_mipi_reset(void)
{
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_RK_PLATFORM)
	u8 cmd1[] = {0x01, 0x39, 0xFF, 0x55, 0xAA, 0x66};
	u8 cmd2[] = {0x01, 0x39, 0xFF, 0x26};
	u8 cmd3[] = {0x01, 0x39, 0x04, 0x01};

	dsi_send_packet(0, cmd1, sizeof(cmd1));

	msleep(10);

	dsi_send_packet(0, cmd2, sizeof(cmd1));

	msleep(10);

	dsi_send_packet(0, cmd3, sizeof(cmd1));

	msleep(10);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	/* TODO: no complete */
	msleep(10);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
	/* TODO: no complete */
	msleep(10);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
	/* TODO: no complete */
	msleep(10);
#endif

	return 0;
}
#endif

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
u8 idm_key[] = {
	0x4C, 0x70, 0xb8, 0x5c, 0x2e, 0x17, 0x0b, 0x05, 0x02, 0x81, 0xc0, 0x60,
	0x30, 0x98, 0x4c, 0x26, 0x13, 0x09, 0x84, 0x42, 0xa1, 0xd0, 0xe8, 0x74,
	0x3a, 0x9d, 0x4e, 0xa7, 0x53, 0x29, 0x14, 0x0a, 0x05, 0x02, 0x81, 0x40,
	0xa0, 0xd0, 0xe8, 0xf4, 0xfa, 0x7d, 0x3e, 0x9f, 0x4f, 0xa7, 0x53, 0x29,
	0x14, 0x0a, 0x05, 0x82, 0xc1, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0x7f,
	0x3f, 0x9f, 0xcf, 0xe7, 0x73
};
#endif

void gcore_reset(void)
{
	gdev_fwu->rst_output(gdev_fwu->rst_gpio, 0);
	msleep(1);
	gdev_fwu->rst_output(gdev_fwu->rst_gpio, 1);
}

/* do not support spi for GC7371 */
int gcore_enter_idm_mode(void)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
	u8 cmd[] = {0x95, 0x27, 0x0F, 0xF0};
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	u8 cmd[] = {0x95, 0x27, 0x0F, 0xF0};
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202)
	u8 cmd[] = {0x4C, 0x0F, 0xF0};
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	u8 cmd[] = {0x4C, 0x0F, 0xF0};
#endif

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
	gcore_reset();
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	gcore_send_mipi_reset();
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	if (gcore_bus_write(idm_key, sizeof(idm_key))) {
		GTP_ERROR("gc7202 write idm key fail");
	}
#endif

	return gcore_bus_write(cmd, sizeof(cmd));

}

int gcore_exit_idm_mode(void)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
	u8 cmd[] = {0x95, 0x27, 0x0A, 0xF5};
	return gcore_bus_write(cmd, sizeof(cmd));
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	u8 cmd1[] = {0x95, 0x27, 0x05, 0xFA, 0xD0, 0x00, 0xFF, 0x00, 0x55, 0xAA, 0x66};
	u8 cmd2[] = {0x95, 0x27, 0x05, 0xFA, 0xD0, 0x00, 0xFF, 0x00, 0x26};
	u8 cmd3[] = {0x95, 0x27, 0x05, 0xFA, 0xD0, 0x00, 0x04, 0xA0, 0x00};
	int ret = 0;

	ret = gcore_bus_write(cmd1, sizeof(cmd1));
	if (ret) {
		goto write_fail;
	}

	msleep(1);

	ret = gcore_bus_write(cmd2, sizeof(cmd2));
	if (ret) {
		goto write_fail;
	}

	msleep(1);

	ret = gcore_bus_write(cmd3, sizeof(cmd3));
	if (ret) {
		goto write_fail;
	}

	return 0;

write_fail:
	GTP_ERROR("gc7271 exit idm fail");
	return -1;

#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	u8 cmd[] = {0x4C, 0x0A, 0xF5};
	return gcore_bus_write(cmd, sizeof(cmd));
#endif
}


s32 gcore_idm_write_reg(u32 addr, u8 *buffer, s32 len)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	u8 cmd[] = {0x95, 0x27, 0x05, 0xFA};
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	u8 cmd[] = {0x4C, 0x05, 0xFA};
#endif
	u8 *buffer_send = NULL;
	u32 len_send = 0;
	int ret = 0;

	len_send = sizeof(cmd) + sizeof(addr) + len;

	buffer_send = kzalloc(len_send, GFP_KERNEL);
	if (buffer_send == NULL) {
		GTP_ERROR("buffer send allocate fail!");
		return -ENOMEM;
	}

	memcpy(buffer_send, cmd, sizeof(cmd));

	*(buffer_send + sizeof(cmd)) = (u8)(addr >> 24);
	*(buffer_send + sizeof(cmd) + 1) = (u8)(addr >> 16);
	*(buffer_send + sizeof(cmd) + 2) = (u8)(addr >> 8);
	*(buffer_send + sizeof(cmd) + 3) = (u8)(addr);

	memcpy(buffer_send + sizeof(cmd) + 4, buffer, len);

	ret = gcore_bus_write(buffer_send, len_send);
	if (ret) {
		GTP_ERROR("idm write reg fail!");
	}

	kfree(buffer_send);

	return ret;

}

s32 gcore_idm_read_reg(u32 addr, u8 *buffer, s32 len)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371) \
		|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	u8 cmd[8] = {0x95, 0x27, 0x04, 0xFB, 0x00, 0x00, 0x00, 0x00};
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	u8 cmd[7] = {0x4C, 0x04, 0xFB, 0x00, 0x00, 0x00, 0x00};
#endif
	int cmd_len = sizeof(cmd);
	int ret = 0;

	if (buffer == NULL) {
		GTP_ERROR("receive buffer is null!");
		return -1;
	}

	cmd[cmd_len - 4] = (u8)(addr >> 24);
	cmd[cmd_len - 3] = (u8)(addr >> 16);
	cmd[cmd_len - 2] = (u8)(addr >> 8);
	cmd[cmd_len - 1] = (u8)(addr);

#ifdef CONFIG_TOUCH_DRIVER_INTERFACE_I2C
	ret = gcore_bus_write(cmd, sizeof(cmd));
	if (ret) {
		GTP_ERROR("idm read wr address fail!");
		return -1;
	}

	/* no sleep. delay precise 1 ms */
	mdelay(1);

	ret = gcore_bus_read(buffer, len);
	if (ret) {
		GTP_ERROR("idm read fail!");
		return -1;
	}
#else
	ret = gcore_spi_write_then_read(cmd, cmd_len, buffer, len);
	if (ret) {
		GTP_ERROR("idm spi write read fail!");
		return -1;
	}
#endif

	return 0;
}


s32 gcore_fw_write_reg(u32 addr, u8 *buffer, s32 len)
{
	u8 cmd[] = {0x80, 0xB0};
	u8 *buffer_send = NULL;
	u32 len_send = 0;
	int ret = 0;
	u8 rw_byte = 0;

	len_send = sizeof(cmd) + sizeof(addr) + sizeof(len) + sizeof(rw_byte) + len;

	buffer_send = kzalloc(len_send, GFP_KERNEL);
	if (buffer_send == NULL) {
		GTP_ERROR("buffer send allocate fail!");
		return -ENOMEM;
	}

	memcpy(buffer_send, cmd, sizeof(cmd));

	*(buffer_send + 2) = (u8)(addr >> 24);
	*(buffer_send + 3) = (u8)(addr >> 16);
	*(buffer_send + 4) = (u8)(addr >> 8);
	*(buffer_send + 5) = (u8)(addr);


	*(buffer_send + 6) = (u8)(len >> 24);
	*(buffer_send + 7) = (u8)(len >> 16);
	*(buffer_send + 8) = (u8)(len >> 8);
	*(buffer_send + 9) = (u8)(len);

	*(buffer_send + 10) = rw_byte;

	memcpy(buffer_send + 11, buffer, len);

	mutex_lock(&gdev_fwu->transfer_lock);

	ret = gcore_bus_write(buffer_send, len_send);
	if (ret) {
		GTP_ERROR("idm write reg fail!");
	}

	mutex_unlock(&gdev_fwu->transfer_lock);

	kfree(buffer_send);

	return ret;
}
EXPORT_SYMBOL(gcore_fw_write_reg);

s32 gcore_fw_read_reg(u32 addr, u8 *buffer, s32 len)
{
	u8 cmd[11] = {0x80, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
	int ret = 0;

	gdev_fwu->fw_event = FW_READ_REG;
	gdev_fwu->firmware = buffer;
	gdev_fwu->fw_xfer = len;

	cmd[2] = (u8)(addr >> 24);
	cmd[3] = (u8)(addr >> 16);
	cmd[4] = (u8)(addr >> 8);
	cmd[5] = (u8)(addr);

	cmd[6] = (u8)(len >> 24);
	cmd[7] = (u8)(len >> 16);
	cmd[8] = (u8)(len >> 8);
	cmd[9] = (u8)(len);

	mutex_lock(&gdev_fwu->transfer_lock);

	fw_update_fn.wait_int = true;

	ret = gcore_bus_write(cmd, sizeof(cmd));
	if (ret) {
		GTP_ERROR("idm read wr address fail!");
		return -1;
	}

	mutex_unlock(&gdev_fwu->transfer_lock);

	wait_for_completion_interruptible(&fw_update_complete);


	return 0;
}
EXPORT_SYMBOL(gcore_fw_read_reg);

int gcore_fw_read_reg_reply(u8 *buf, int len)
{
	int ret = 0;

	if (buf == NULL) {
		GTP_ERROR("receive buffer is null!");
		return -1;
	}

	ret = gcore_bus_read(buf, len);
	if (ret) {
		GTP_ERROR("write cmd read flash id error.");
		return -1;
	}

	fw_update_fn.wait_int = false;
	complete(&fw_update_complete);

	return 0;

}

int gcore_fw_event_notify(enum fw_event_type event)
{
	u8 cmd[] = { 0x80, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 cmd_len = sizeof(cmd);
	int ret = 0;

	switch (event) {
	case FW_EDGE_0:
		cmd[cmd_len - 1] = 0x02;
		GTP_DEBUG("gcore fw event:edge 0");
		break;

	case FW_EDGE_90:
		cmd[cmd_len - 1] = 0x03;
		GTP_DEBUG("gcore fw event:edge 90");
		break;

	case FW_CHARGER_PLUG:
		cmd[cmd_len - 1] = 0x04;
		GTP_DEBUG("gcore fw event:charger plug");
		break;

	case FW_CHARGER_UNPLUG:
		cmd[cmd_len - 1] = 0x05;
		GTP_DEBUG("gcore fw event:charger unplug");
		break;

	case FW_HEADSET_PLUG:
		cmd[cmd_len - 1] = 0x06;
		GTP_DEBUG("gcore fw event:headset plug");
		break;

	case FW_HEADSET_UNPLUG:
		cmd[cmd_len - 1] = 0x07;
		GTP_DEBUG("gcore fw event:headset unplug");
		break;

	case FW_GESTURE_ENABLE:
		cmd[cmd_len - 1] = 0x0A;
		GTP_DEBUG("gcore fw event:gesture enable");
		break;

	case FW_GESTURE_DISABLE:
		cmd[cmd_len - 1] = 0x0B;
		GTP_DEBUG("gcore fw event:gesture disable");
		break;

	default:
		GTP_DEBUG("unknown fw event type.");
		break;
	}

	mutex_lock(&gdev_fwu->transfer_lock);

	ret = gcore_bus_write(cmd, cmd_len);
	if (ret) {
		GTP_ERROR("gcore write fw event fail.");
	}

	mutex_unlock(&gdev_fwu->transfer_lock);

	return ret;
}

int gcore_reg_enable_write_on(void)
{
	u32 addr = 0xC0000020;
	u8 value = 0x95;

	return gcore_idm_write_reg(addr, &value, 1);
}

int gcore_reg_enable_write_off(void)
{
	u32 addr = 0xC0000020;
	u8 value = 0x00;

	return gcore_idm_write_reg(addr, &value, 1);
}

int gcore_read_fw_version(u8 *version, int length)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7271) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD
		u32 upgrade_addr = 0xC0000000 + ((0x14 << 2) + 1);
		u8 value = BIT6; //bit6 write 1
#else
		u32 upgrade_addr = 0x40000000 + ((0x14 << 2));
		u8 value = BIT0; //bit0 write 1
#endif
#endif

	if (g_update_running) {
		GTP_ERROR("fw update is running, do not read version!");
		return 0;
	}
	mutex_lock(&gdev_fwu->transfer_lock);
	gcore_enter_idm_mode();

	msleep(1);

#ifdef CONFIG_ENABLE_CHIP_TYPE_GC7372
	gcore_idm_read_reg(0x8002FFF4, version, length);
#else
	gcore_idm_read_reg(0x8001FFF4, version, length);
#endif
	GTP_DEBUG("fw version:%x %x %x %x", version[0], version[1], version[2], version[3]);

	msleep(1);

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7271) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	if (gcore_idm_write_reg(upgrade_addr, &value, 1)) {
		GTP_ERROR("write reg rkucr0 upgrade fail");
	}

	msleep(1);
#endif

	gcore_exit_idm_mode();
	mutex_unlock(&gdev_fwu->transfer_lock);

	return 0;

}
EXPORT_SYMBOL(gcore_read_fw_version);

int gcore_read_ram_fw(u8 *buf, int length)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7271) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD
		u32 upgrade_addr = 0xC0000000 + ((0x14 << 2) + 1);
		u8 value = BIT6; //bit6 write 1
#else
		u32 upgrade_addr = 0xC0000000 + ((0x14 << 2) + 1);
		u8 value = BIT5; //bit5 write 1
#endif
#endif

	gcore_enter_idm_mode();

	msleep(1);

#ifdef CONFIG_ENABLE_CHIP_TYPE_GC7372
	gcore_idm_read_reg(0x80020000, buf, length);
#else
	gcore_idm_read_reg(0x80010000, buf, length);
#endif
	GTP_DEBUG("fw ram:%x %x %x %x %x", buf[0], buf[1], buf[2], buf[3], buf[4]);

	msleep(1);

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7271) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	if (gcore_idm_write_reg(upgrade_addr, &value, 1)) {
		GTP_ERROR("write reg rkucr0 upgrade fail");
	}

	msleep(1);
#endif

	gcore_exit_idm_mode();

	return 0;

}

#define DUMP_FW_FILE_NAME  "/sdcard/gcore_dump_fw.bin"

int gcore_dump_fw_to_file(void)
{
	u8 *fw_buf = NULL;
	int xfer_len = FW_SIZE;
	struct file *file = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	mm_segment_t old_fs;
#endif
	loff_t pos = 0;

#ifdef CONFIG_TOUCH_DRIVER_INTERFACE_SPI
#ifdef CONFIG_MTK_SPI_MULTUPLE_1024

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371) \
		|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	xfer_len = FW_SIZE + 1024 - 8 - 2;
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	xfer_len = FW_SIZE + 1024 - 7 - 2;
#endif

#else
	xfer_len = FW_SIZE;

#endif
#endif

	fw_buf = (u8 *)kzalloc(xfer_len, GFP_KERNEL);
	if (!fw_buf) {
		GTP_ERROR("fw buf mem allocate fail");
		return -1;
	}

	gcore_read_ram_fw(fw_buf, xfer_len);


	file = filp_open(DUMP_FW_FILE_NAME, O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (IS_ERR(file)) {
		GTP_ERROR("Open file %s failed", DUMP_FW_FILE_NAME);
		return -1;
	}
	pos = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	kernel_write(file, fw_buf, FW_SIZE, &pos);
#else
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	__kernel_write(file, fw_buf, FW_SIZE, &pos);
	set_fs(old_fs);
#endif
	filp_close(file, NULL);

	if (fw_buf) {
		kfree(fw_buf);
	}

	return 0;
}

int gcore_idm_read_id(u8 *id, int length)
{
	u8 cmd1[] = {0x55, 0xAA, 0x66};
	u8 unlock = 0x10;
	u8 lock = 0x00;
	u8 syssr0[4] = { 0 };
	u8 syscr5[4] = { 0 };

	gcore_enter_idm_mode();

	msleep(1);

	gcore_idm_read_reg(0xC0000000 + (0x27 << 2), syssr0, sizeof(syssr0));
	GTP_DEBUG("reg syssr0:%x %x %x %x", syssr0[0], syssr0[1], syssr0[2], syssr0[3]);
	if (syssr0[1] & BIT7) {
		GTP_DEBUG("dp_lock is lock!");
	}

	msleep(1);

	gcore_idm_read_reg(0xC0000000 + (0x25 << 2), syscr5, sizeof(syscr5));
	GTP_DEBUG("reg syscr5:%x %x %x %x", syscr5[0], syscr5[1], syscr5[2], syscr5[3]);

	msleep(1);

	syscr5[2] &= 0x7F;
	gcore_idm_write_reg(0xC0000000 + (0x25 << 2) + 2, &syscr5[2], 1);

	GTP_DEBUG("test tp_sel end.");

	msleep(1);

	gcore_idm_write_reg(0xD000FF00, cmd1, sizeof(cmd1));

	msleep(1);

	/* unlock page */
	gcore_idm_write_reg(0xD000FF00, &unlock, 1);

	msleep(1);

	/* read ID */
	gcore_idm_read_reg(0xD0000400, id, length);

	GTP_DEBUG("chip id:%x %x", id[0], id[1]);
	msleep(1);

	/* lock page */
	gcore_idm_write_reg(0xD000FF00, &lock, 1);

	msleep(1);

	gcore_exit_idm_mode();

	return 0;
}

#ifdef CONFIG_ENABLE_FW_RAWDATA
int gcore_set_fw_mode(enum FW_MODE mode)
{
	u32 addr = 0xC0000026;
	u8 value = mode;

	return gcore_idm_write_reg(addr, &value, 1);
}

int gcore_reset_mcu(void)
{
	u8 cmd[] = {0x95, 0x27, 0x0A, 0xF5};
	return gcore_bus_write(cmd, sizeof(cmd));
}

/*
 * set fw mode old way, new way use gcore_fw_mode_set_proc2
 * do not support spi for 7371
 */
/*
int gcore_fw_mode_set_proc(enum FW_MODE mode)
{
	int ret = 0;

	ret = gcore_enter_idm_mode();
	if (ret)
	{
		GTP_ERROR("enter idm mode fail");
		goto set_mode_fail;
	}

	ret = gcore_reg_enable_write_on();
	if (ret)
	{
		GTP_ERROR("reg enable write on fail");
		goto set_mode_fail;
	}

	ret = gcore_set_fw_mode(mode);
	if (ret)
	{
		GTP_ERROR("set fw mode fail");
		goto set_mode_fail;
	}

	ret = gcore_reg_enable_write_off();
	if (ret)
	{
		GTP_ERROR("reg enable write off fail");
		goto set_mode_fail;
	}

	ret = gcore_reset_mcu();
	if (ret)
	{
		GTP_ERROR("reset mcu fail");
		goto set_mode_fail;
	}

	fw_mode_running = mode;
	GTP_INFO("Set fw_mode %d success", mode);

	return 0;

set_mode_fail:
	return -1;

}
*/

int gcore_fw_mode_set_proc2(u8 mode)
{
	u8 cmd[] = {0x40, 0xA0, 0x00, 0x00, 0x73, 0x71, 0x00, 0x00, mode};
	int ret = 0;

	ret = gcore_bus_write(cmd, sizeof(cmd));
	if (ret) {
		GTP_ERROR("fw mode set fail!");
		return -1;
	}

	gdev_fwu->fw_mode = mode;
	GTP_DEBUG("Set fw_mode %d success", mode);

	return 0;
}
EXPORT_SYMBOL(gcore_fw_mode_set_proc2);
#endif

int gcore_run_pram_fw(void)
{
	int ret = 0;
	/* run flash op fw */
	u8 cmd1[] = {0x95, 0x27, 0x05, 0xFA, 0x40, 0x00, 0x00, 0x50, 0x81}; //remap

	ret = gcore_bus_write(cmd1, sizeof(cmd1));
	if (ret) {
		GTP_ERROR("run flash op fw write error.");
		return -1;
	}

	ret = gcore_exit_idm_mode();
	if (ret) {
		GTP_ERROR("run flash op fw exit idm fail.");
		return -1;
	}

	return 0;

}

/*
 * after flashdownload,run fw soft work.
 */
int gcore_upgrade_soft_reset(void)
{
#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
	u8 abccr0_3[4] = { 0 };
	u32 abccr0 = 0xC0000000;
	int ret = 0;
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271)
	u32 upgrade_addr = 0xC0000000 + ((0x14 << 2) + 1);
	u8 value = BIT6; //bit6 write 1
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7202)
	u32 upgrade_addr = 0xC0000000 + ((0x14 << 2) + 1);
	u8 value = BIT6; //bit6 write 1
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
	u8 open = 0x00;
	u8 close = 0x01;
	u8 data1[] = {0x00, 0x01, 0x00, 0x00};
	u8 data2[] = {0x02, 0xEA, 0x00, 0x40, 0x23, 0xEA, 0x00, 0x40, 0x00, 0x30,
				  0x54, 0xB3, 0x3C, 0x78, 0x00, 0x00};
#endif

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
	ret = gcore_enter_idm_mode();
	if (ret) {
		GTP_ERROR("enter idm mode fail");
		goto run_fw_fail;
	}

	msleep(1);

	ret = gcore_reg_enable_write_on();
	if (ret) {
		GTP_ERROR("reg enable write on fail");
		goto run_fw_fail;
	}

	msleep(1);

	ret = gcore_idm_read_reg(abccr0, abccr0_3, sizeof(abccr0_3));
	if (ret) {
		GTP_ERROR("read reg abccr0 fail");
		goto run_fw_fail;
	}

	msleep(1);

	ret = gcore_idm_write_reg(abccr0, &abccr0_3[0], 1);
	if (ret) {
		GTP_ERROR("write reg abccr0 fail");
		goto run_fw_fail;
	}

	msleep(1);

	ret = gcore_exit_idm_mode();
	if (ret) {
		GTP_ERROR("reset mcu fail");
		goto run_fw_fail;
	}

#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271) \
			|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7202)

	gcore_enter_idm_mode();

	mdelay(5);

	if (gcore_idm_write_reg(upgrade_addr, &value, 1)) {
		GTP_ERROR("write reg rkucr0 upgrade fail");
//		goto run_fw_fail;
	}

	msleep(1);

	gcore_exit_idm_mode();

#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)

	gcore_enter_idm_mode();

	msleep(1);

	if (gcore_idm_write_reg(0x40000050, &open, 1)) {
		GTP_ERROR("write reg fail");
	}

	msleep(1);

	if (gcore_idm_write_reg(0x80020000, data1, sizeof(data1))) {
		GTP_ERROR("write reg fail");
	}

	msleep(1);

	if (gcore_idm_write_reg(0x80020100, data2, sizeof(data2))) {
		GTP_ERROR("write reg fail");
	}

	msleep(1);

	if (gcore_idm_write_reg(0x40000050, &close, 1)) {
		GTP_ERROR("write reg fail");
	}

	msleep(1);

	gcore_exit_idm_mode();

#endif

	return 0;

//run_fw_fail:
//	return -1;

}

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD

#define STARTUP_UPDATE_DELAY_MS  1000 //ms

int hostdownload_idm;



/*
* a printk will cost over 1ms cpu times, so we should not use printk
* when it is time strict,like hostdownload read B3 and read coordinate date.
*/
int gcore_auto_update_hostdownload(u8 *fw_buf)
{
	u8 result = 0;
	u8 *fw_data = fw_buf;
	u8 retry = 0;

	if (!fw_buf) {
		return -1;
	}

	if (gcore_bus_read(&result, 1)) {
		GTP_ERROR("host download read error.");
		return -1;
	}

	GTP_DEBUG("read result:%x", result);

	switch (result) {
	case 0xB1:
		GTP_DEBUG("Start to update pram fw,times(%d)", retry);
		retry++;

		if (gcore_bus_write(fw_data, FW_SIZE)) {
			GTP_ERROR("host download write error.");
			return -1;
		}
/*
		for (count = 0; count < 64; count++)
		{
			if (gcore_bus_write(fw_data, 1024))
			{
				GTP_ERROR("host download write error.");
				return -1;
			}
			msleep(1);
			fw_data += 1024;
		}
*/
/*
		for (count = 0; count < 64*1024; count=count + 56)
		{
			if ((count + 56) > (64*1024))
			{
				len = (64*1024) - count;
			}
			else
			{
				len = 56;
			}

			if (gcore_bus_write(fw_data, len))
			{
				GTP_ERROR("host download read error.");
				return -1;
			}

			fw_data += len;
			mdelay(2);
		}
*/
		break;

	case 0xB2:
		GTP_DEBUG("update pram fw retry 3 times fail!");
		break;

	case 0xB3:
		GTP_DEBUG("update pram fw success...");
		fw_update_fn.wait_int = false;
		complete(&fw_update_complete);
		break;

	default:
		GTP_DEBUG("pram fw read unknow result.");
		fw_update_fn.wait_int = false;
		complete(&fw_update_complete);
		break;
	}


	return 0;
}

int gcore_update_hostdownload_trigger(u8 *fw_buf)
{
	gdev_fwu->fw_event = FW_UPDATE;
	gdev_fwu->firmware = fw_buf;
	fw_update_fn.wait_int = true;

#if defined(CONFIG_ENABLE_CHIP_TYPE_GC7371)
	gcore_reset();
#elif defined(CONFIG_ENABLE_CHIP_TYPE_GC7271) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7202) \
	|| defined(CONFIG_ENABLE_CHIP_TYPE_GC7372)
//	gcore_reset();	//reset RESX pin

	if (gcore_upgrade_soft_reset()) {
		GTP_ERROR("hostdownload soft reset fail.");
		return -1;
	}
#endif

	if (!wait_for_completion_interruptible_timeout(&fw_update_complete, 5 * HZ)) {
		GTP_ERROR("hostdownload trigger timeout.");
		return -1;
	}

//	if (gcore_auto_update_hostdownload(fw_buf))
//	{
//		GTP_ERROR("hostdownload write fail.");
//		return -1;
//	}

	return 0;
}

int gcore_burn_fw_idm(u8 *fw_data)
{
	/* 0x80, 0x01, 0x00, 0x00 is address */
	u8 cmd[] = {0x95, 0x27, 0x05, 0xFA, 0x80, 0x01, 0x00, 0x00};
	u8 *addr_h = &cmd[6];
	u8 *fw_data_buf;
	u32 fw_length = FW_SIZE;
	u32 size_one_time = 1024;
	u8 times = fw_length/size_one_time;
	int ret = 0;

	fw_data_buf = kzalloc(size_one_time + 8, GFP_KERNEL);
	if (fw_data_buf == NULL) {
		GTP_ERROR("flash_op_buf mem allocate fail!");
		return -ENOMEM;
	}

	while (times--) {
		memcpy(fw_data_buf, cmd, 8);
		memcpy(fw_data_buf + 8, fw_data, size_one_time);

		ret = gcore_bus_write(fw_data_buf, size_one_time + 8);
		if (ret) {
			GTP_ERROR("flash op fw addr write error.");
			kfree(fw_data_buf);
			return -1;
		}

		fw_data += size_one_time;

		*addr_h += 4;   /* FLASH_OP_TRANSFER_SIZE */

	}

	kfree(fw_data_buf);

	return 0;

}

int gcore_update_hostdownload_idm(u8 *fw_buf)
{
	int progress = 0;
	int ret = 0;

	ret = gcore_enter_idm_mode();
	if (ret) {
		goto gcore_update_exit;
	}
	progress = 1;

	ret = gcore_burn_fw_idm(fw_buf);
	if (ret) {
		goto gcore_update_exit;
	}
	progress = 2;

	ret = gcore_run_pram_fw();
	if (ret) {
		goto gcore_update_exit;
	}
	progress = 3;

	return 0;

gcore_update_exit:
	return -1;

}

int gcore_auto_update_hostdownload_proc(void *fw_buf)
{
	if (!hostdownload_idm) {
		if (gcore_update_hostdownload_trigger(fw_buf)) {
			GTP_ERROR("hostdownload proc fail!");
			return -1;
		}
	} else {
		if (gcore_update_hostdownload_idm(fw_buf)) {
			GTP_ERROR("hostdownload idm proc fail!");
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(gcore_auto_update_hostdownload_proc);

void set_hostdownload_idm(u8 value)
{
	hostdownload_idm = value;
}



#endif  //CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD
/*
* Maybe can disable interupt before progress 6
*/
void gcore_enter_update_mode(void)
{
	gdev_fwu->irq_disable(gdev_fwu);
}

int gcore_update_prepare(u8 *fw_buf)
{
	const struct firmware *fw_op = NULL;

	update_info.fw_data = fw_buf;
	update_info.fw_length = FW_SIZE;
	update_info.fw_burn_progress = 0;

#ifdef CONFIG_UPDATE_FIRMWARE_BY_BIN_FILE
	if (request_firmware(&fw_op, FW_OP_BIN_NAME, &gdev_fwu->bus_device->dev)) {
		GTP_ERROR("request firmware op fail!");
		return -1;
	}

	update_info.flash_op_length = fw_op->size;
	update_info.flash_op_data = (u8 *)kzalloc(update_info.flash_op_length, GFP_KERNEL);
	if (update_info.flash_op_data == NULL) {
		GTP_ERROR("flash op mem allocate fail!");
		goto fail1;
	}

	memcpy(update_info.flash_op_data, fw_op->data, fw_op->size);
	GTP_DEBUG("flash op data:%x %x %x %x %x %x", update_info.flash_op_data[0], update_info.flash_op_data[1], update_info.flash_op_data[2], \
		update_info.flash_op_data[3], update_info.flash_op_data[4], update_info.flash_op_data[5]);

	if (fw_op) {
		release_firmware(fw_op);
	}
#else
	update_info.flash_op_length = sizeof(gcore_flash_op_FW);
	update_info.flash_op_data = gcore_flash_op_FW;
#endif

	update_info.flash_op_buf = kzalloc(FLASH_OP_TRANSFER_SIZE + 8, GFP_KERNEL);
	if (update_info.flash_op_buf == NULL) {
		GTP_ERROR("flash_op_buf mem allocate fail!");
		goto fail2;
	}


	return 0;

fail2:
	if (update_info.flash_op_data) {
		kfree(update_info.flash_op_data);
	}
#ifdef CONFIG_UPDATE_FIRMWARE_BY_BIN_FILE
fail1:
	if (fw_op) {
		release_firmware(fw_op);
	}
#endif
	return -1;
}

int gcore_update_judge(u8 *fw_buf)
{
	u8 cmd[] = {0x95, 0x27, 0x04, 0xFB, 0x80, 0x01, 0xFF, 0xF4};
	u8 ver[4] = { 0 };  //little-endian
	u16 fw_major = 0;
	u16 fw_minor = 0;
	u32 fw_version = 0;
	u16 fw_major_update = 0;
	u16 fw_minor_update = 0;
	u32 fw_version_update = 0;

	if (gcore_bus_write(cmd, sizeof(cmd))) {
		GTP_ERROR("get firmware version write error!");
		return -1;
	}

	if (gcore_bus_read(ver, sizeof(ver))) {
		GTP_ERROR("get firmware version read error!");
		return -1;
	}

	fw_major = (ver[1] << 8) | ver[0];
	fw_minor = (ver[3] << 8) | ver[2];
	fw_version = (fw_major << 16) | fw_minor;

	fw_major_update = (fw_buf[FW_VERSION_ADDR + 1] << 8) | fw_buf[FW_VERSION_ADDR];
	fw_minor_update = (fw_buf[FW_VERSION_ADDR + 3] << 8) | fw_buf[FW_VERSION_ADDR + 2];
	fw_version_update = (fw_major_update << 16) | fw_minor_update;

	GTP_DEBUG("fw_version_update=%x fw_version=%x", fw_version_update, fw_version);

	if (fw_version_update <= fw_version) {
		GTP_ERROR("The version of the fw is not high than the IC's!");
	}

	return 0;
}

int gcore_burn_flash_op_fw(void)
{
	/* 0x80, 0x01, 0x00, 0x00 is address */
	u8 cmd[] = {0x95, 0x27, 0x05, 0xFA, 0x80, 0x01, 0x00, 0x00};
	u8 *addr_h = &cmd[6];
	u8 times = update_info.flash_op_length/FLASH_OP_TRANSFER_SIZE;
	int ret = 0;
	u8 *flash_op_p = update_info.flash_op_data;

	while (times--) {
		memcpy(update_info.flash_op_buf, cmd, 8);
		memcpy(update_info.flash_op_buf + 8, flash_op_p, FLASH_OP_TRANSFER_SIZE);

		ret = gcore_bus_write(update_info.flash_op_buf, FLASH_OP_TRANSFER_SIZE + 8);
		if (ret) {
			GTP_ERROR("flash op fw addr write error.");
			return -1;
		}

		flash_op_p += FLASH_OP_TRANSFER_SIZE;

		*addr_h += 4;   /* FLASH_OP_TRANSFER_SIZE */

	}

	return 0;

}

int gcore_flash_program_proc(void)
{
	u8 cmd[GC7XXX_REQUEST_SIZE] = { 0 };
	int ret = 0;
	u16 transfer_length = 0;
	u8 flash_address_h = 0;
	u8 flash_address_m = 0;
	u8 flash_address_l = 0;
	u8 data_length_H = 0;
	u8 data_length_L = 0;

	if (update_info.fw_burn_progress == update_info.fw_length) {
		GTP_DEBUG("fw_burn_progress is equal to fw_length.");
		return -1;
	}

	if ((update_info.fw_length - update_info.fw_burn_progress) > GC7XXX_MAX_TRANSFER_DATA) {
		transfer_length = GC7XXX_MAX_TRANSFER_DATA;
	} else {
		transfer_length = update_info.fw_length - update_info.fw_burn_progress;
	}

	flash_address_h = ((update_info.fw_burn_progress & 0xFF0000) >> 16);
	flash_address_m = ((update_info.fw_burn_progress & 0xFF00) >> 8);
	flash_address_l = (update_info.fw_burn_progress & 0xFF);

	data_length_H = ((transfer_length & 0xFF00) >> 8);
	data_length_L = (transfer_length & 0xFF);

	cmd[0] = 0x40;
	cmd[1] = flash_address_h;  //flash_address17-23
	cmd[2] = 0xD2;
	cmd[3] = 0x03;
	cmd[4] = flash_address_m;  //flash_address8-16
	cmd[5] = flash_address_l;  //flash_address0-7
	cmd[6] = data_length_H;    //data length H
	cmd[7] = data_length_L;    //data length L

	memcpy(&cmd[8], update_info.fw_data + update_info.fw_burn_progress, transfer_length);

	ret = gcore_bus_write(cmd, transfer_length + 8);
	if (ret) {
		GTP_ERROR("write cmd read flash id error.");
		return -1;
	}

	update_info.fw_burn_progress += transfer_length;

	return 0;

}

void gcore_flashdownload_cleanup(void)
{
	if (update_info.flash_op_buf != NULL) {
		kfree(update_info.flash_op_buf);
		update_info.flash_op_buf = NULL;
	}

	if (update_info.flash_op_data != NULL) {
		kfree(update_info.flash_op_data);
		update_info.flash_op_data = NULL;
	}
}

int gcore_flash_op_request(enum FLASH_OP command)
{
	u8 cmd[GC7XXX_REQUEST_SIZE] = { 0 };
	int ret = 0;

	switch (command) {
	case READ_FLASH_ID:
		cmd[0] = 0x40;
		cmd[1] = 0xE1;
		cmd[2] = 0xD0;
		cmd[3] = 0x03;

		ret = gcore_bus_write(cmd, 4);
		if (ret) {
			GTP_ERROR("write cmd read flash id error.");
			return -1;
		}

		break;

	case FLASH_ERASE:
		cmd[0] = 0x40;
		cmd[1] = 0xE1;
		cmd[2] = 0xD1;
		cmd[3] = 0x03;
		cmd[4] = 0x00;

		ret = gcore_bus_write(cmd, 5);
		if (ret) {
			GTP_ERROR("write cmd read flash id error.");
			return -1;
		}

		break;

	case FLASH_PROGRAM:
		ret = gcore_flash_program_proc();
		if (ret) {
			GTP_ERROR("flash burn proc error.");
			return -1;
		}

		break;

	default:

		break;
	}

	return 0;
}

/*
 * 2020-11-05:run fw soft do not work.
 */
int gcore_run_fw_soft(void)
{
	int ret = 0;
	/* run fw */
	u8 cmd1[] = {0x95, 0x27, 0x05, 0xFA, 0x40, 0x00, 0x00, 0x50, 0x80};
	u8 cmd2[] = {0x95, 0x27, 0x0A, 0xF5};

	ret = gcore_bus_write(cmd1, sizeof(cmd1));
	if (ret) {
		GTP_ERROR("run flash op fw write error.");
		return -1;
	}

	ret = gcore_bus_write(cmd2, sizeof(cmd2));
	if (ret) {
		GTP_ERROR("run flash op fw write error.");
		return -1;
	}

	return 0;

}

int gcore_flash_op_reply_proc(void)
{
	u8 reply[GC7XXX_REPLY_SIZE] = { 0 };
	u8 command = 0;
	u32 flash_id = 0;
	int ret = 0;
	int result = 0;

	ret = gcore_bus_read(reply, GC7XXX_REPLY_SIZE);
	if (ret) {
		GTP_ERROR("flash op reply read error.");
		return -1;
	}

	command = reply[2];

	switch (command) {
	case 0xC0: /* Read Flash ID */
		flash_id = ((reply[8] << 16) | (reply[9] << 8) | (reply[10]));
		if ((flash_id != 0x00) && (flash_id != 0xFF)) {
			if (gcore_flash_op_request(FLASH_ERASE)) {
				GTP_ERROR("flash erase error.");
			}
			update_info.progress = 7;
		} else {
			GTP_ERROR("invalid flash id.");
		}

		break;

	case 0xC1: /* Flash Erase */
		result = reply[7];
		if (result == 1) {
			if (update_info.fw_burn_progress < update_info.fw_length) {
				if (gcore_flash_op_request(FLASH_PROGRAM)) {
					GTP_ERROR("flash program error.");
					return -1;
				}
			}
			update_info.progress = 8;
		} else {
			GTP_ERROR("flash erase fail");
		}

		break;

	case 0xF9: /* Flash Program */
	case 0xFA:
		result = reply[3];
		if (result == 0x01) {
			if (update_info.fw_burn_progress < update_info.fw_length) {
				if (gcore_flash_op_request(FLASH_PROGRAM)) {
					GTP_ERROR("flash program error.");
					return -1;
				}
				update_info.progress = 9;
			} else {
				GTP_DEBUG("flash download finish");
				msleep(100);
				gcore_upgrade_soft_reset();
				gcore_flashdownload_cleanup();
				update_info.status = UPDATE_STATUS_IDLE;
				update_info.progress = 10;
				fw_update_fn.wait_int = false;
				complete(&fw_update_complete);
			}

		} else if (result == 0x02) {
			GTP_DEBUG("flash download success");
			gcore_run_fw_soft();
			gcore_flashdownload_cleanup();
			update_info.status = UPDATE_STATUS_IDLE;
			update_info.progress = 10;
			fw_update_fn.wait_int = false;
			complete(&fw_update_complete);
		} else if (result == 0x00) {
			GTP_DEBUG("flash download failed");
			return -1;
		}

		break;

	case 0xC3: /* Flash Read */

		break;
	}

	return 0;

}

int gcore_auto_update_flashdownload_proc(void *fw_buf)
{
	int ret = 0;

	if (update_info.status != UPDATE_STATUS_IDLE) {
		GTP_ERROR("Update process is running!");
		return -1;
	}

	ret = gcore_update_prepare(fw_buf);
	if (ret) {
		return -1;
	}

	update_info.status = UPDATE_STATUS_RUNNING;
	update_info.max_progress = 10;
	update_info.progress = 0;

	gcore_enter_update_mode();

	mutex_lock(&gdev_fwu->transfer_lock);

//	gcore_reset();  //move to gcore_enter_idm_mode()
	update_info.progress = 1;

	ret = gcore_enter_idm_mode();
	if (ret) {
		goto fail1;
	}
	update_info.progress = 2;

	ret = gcore_update_judge(fw_buf);
	if (ret) {
		goto fail1;
	}
	update_info.progress = 3;

	ret = gcore_burn_flash_op_fw();
	if (ret) {
		goto fail1;
	}
	update_info.progress = 4;

	ret = gcore_run_pram_fw();
	if (ret) {
		goto fail1;
	}
	update_info.progress = 5;

	mdelay(1);
	gdev_fwu->irq_enable(gdev_fwu);


	gdev_fwu->fw_event = FW_UPDATE;
	fw_update_fn.wait_int = true;

	ret = gcore_flash_op_request(READ_FLASH_ID);
	if (ret) {
		goto fail2;
	}
	update_info.progress = 6;

	mutex_unlock(&gdev_fwu->transfer_lock);

	if (!wait_for_completion_timeout(&fw_update_complete, 10 * HZ)) {
		GTP_ERROR("update flashdownload timeout!");
		goto gcore_update_exit;
	}

	return 0;

fail1:
	gdev_fwu->irq_enable(gdev_fwu);
fail2:
	mutex_unlock(&gdev_fwu->transfer_lock);
gcore_update_exit:
	GTP_ERROR("update flashdownload proc fail");
	gcore_flashdownload_cleanup();
	update_info.status = UPDATE_STATUS_IDLE;

	return -1;
}
EXPORT_SYMBOL(gcore_auto_update_flashdownload_proc);

#endif  //CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD

u8 g_ret_update;

void gcore_request_firmware_update_work(struct work_struct *work)
{
	static u8 *fw_buf;
#ifdef CONFIG_UPDATE_FIRMWARE_BY_BIN_FILE
	const struct firmware *fw = NULL;

	if (IS_ERR_OR_NULL(fw_buf))
		fw_buf = (u8 *)kzalloc(FW_SIZE, GFP_KERNEL);

	if (ERR_ALLOC_MEM(fw_buf)) {
		GTP_ERROR("fw buf mem allocate fail");
	}

	if (request_firmware(&fw, FW_BIN_NAME, &gdev_fwu->bus_device->dev)) {
		GTP_ERROR("request firmware fail");
		return;
	}

	memcpy(fw_buf, fw->data, fw->size);

	GTP_DEBUG("fw buf:%x %x %x %x %x %x", fw_buf[0], fw_buf[1], fw_buf[2], \
		fw_buf[3], fw_buf[4], fw_buf[5]);

	if (fw) {
		release_firmware(fw);
	}
#else
	fw_buf = gcore_default_FW;
#endif

	g_ret_update = 0;
	g_update_running = 1;

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD
	if (gcore_auto_update_flashdownload_proc(fw_buf)) {
		GTP_ERROR("auto update flashdownload proc fail");
	}
#else
	if (gcore_auto_update_hostdownload_proc(fw_buf)) {
		GTP_ERROR("auto update hostdownload proc fail");
	}
#endif

	g_ret_update = 1;
	g_update_running = 0;

	return;
}

#ifdef CONFIG_ENABLE_FW_RAWDATA

/*
 * FW mode set sys interface
 */
static ssize_t fw_mode_set_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 mode = 0;

	if (!strncmp(buf, "-w", 2)) {
		buf += 3;
		if (!strncmp(buf, "fw_mode=", 8) && (sscanf(buf+8, "%d", &mode) == 1)) {
			GTP_DEBUG("Set fw_mode is:%d", mode);
			if (gcore_fw_mode_set_proc2(mode)) {
				GTP_DEBUG("fw mode set proc fail");
				return -1;
			}
		} else {
			GTP_DEBUG("Wrong parameters.");
			return -1;
		}
	}

	return count;

}

#endif


/*
 * FW reg rw sys interface
 */
static ssize_t fw_reg_rw_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 addr = 0;
	u32 value = 0;
	u32 length = 0;
	u8 r_buff[20] = { 0 };

	if (!strncmp(buf, "-w", 2)) {
		buf += 3;
		if (!strncmp(buf, "addr=", 5) && !strncmp(buf+14, "value=", 6) &&   \
			(sscanf(buf, "addr=%x value=%x", &addr, &value) == 2)) {
			GTP_DEBUG("addr is:%x,value is:%x", addr, value);
			if (gcore_fw_write_reg(addr, (u8 *)&value, 1)) {
				GTP_ERROR("fw reg write fail");
				return -1;
			}
		} else {
			GTP_ERROR("Wrong parameters.");
			return -1;
		}
	} else if (!strncmp(buf, "-r", 2)) {
		buf += 3;
		if (!strncmp(buf, "addr=", 5) && !strncmp(buf+14, "len=", 4) &&   \
			(sscanf(buf, "addr=%x len=%d", &addr, &length) == 2)) {
			GTP_DEBUG("addr is:%x,len is:%d", addr, length);
			if (gcore_idm_read_reg(addr, r_buff, length)) {
				GTP_ERROR("fw reg write fail");
				return -1;
			}
			GTP_DEBUG("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", r_buff[0], r_buff[1], r_buff[2],  \
			r_buff[3], r_buff[4], r_buff[5], r_buff[6], r_buff[7], r_buff[8], r_buff[9]);
		} else {
			GTP_ERROR("Wrong parameters.");
			return -1;
		}
	} else if (!strncmp(buf, "-i", 2)) {
		GTP_DEBUG("enter idm mode");
		gcore_enter_idm_mode();
	} else if (!strncmp(buf, "-o", 2)) {
		GTP_DEBUG("exit idm mode");
		gcore_exit_idm_mode();
	}

	return count;

}


/*
 * FW update sys interface
 */
static ssize_t fw_update_flash_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret = 0;
	int enabled = 0;

	ret = kstrtoint(buf, 0, &enabled);

	if (enabled) {
		GTP_DEBUG("fw update enable:%d", enabled);

		gcore_request_firmware_update_work(NULL);

	} else {
		GTP_ERROR("fw update enable:%d", enabled);
	}

	return count;

}

static ssize_t ret_update_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	if (g_ret_update) {
		return sprintf(buf, "ok");
	} else {
		return sprintf(buf, "fail");
	}
}


#ifdef CONFIG_DEBUG_SPI_SPEED

static ssize_t spi_adjust(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct spi_device *spi;

	struct mt_chip_conf *chip_config;

	u32 high_time, low_time;

	spi = container_of(dev, struct spi_device, dev);

	chip_config = (struct mt_chip_conf *)spi->controller_data;
	if (!chip_config) {
		GTP_ERROR("chip_config is NULL");
		return -1;
	}

	if (!buf) {
		GTP_ERROR("buf is NULL");
		return -1;
	}

	if (!strncmp(buf, "-w", 2)) {
		buf += 3;
		if (!strncmp(buf, "high_time=", 10) && (sscanf(buf+10, "%d", &high_time) == 1)) {
			GTP_DEBUG("Set high_time is:%d", high_time);
			chip_config->high_time = high_time;
		} else if (!strncmp(buf, "low_time=", 9) && (sscanf(buf+9, "%d", &low_time) == 1)) {
			GTP_DEBUG("Set low_time is:%d", low_time);
			chip_config->low_time = low_time;
		} else {
			GTP_DEBUG("Wrong parameters.");
			return -1;
		}
	}

	return count;
}

#endif

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD

static ssize_t hostdl_idm_set_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret = 0;
	int enabled = 0;
	u8 buff[2] = { 0 };

	ret = kstrtoint(buf, 0, &enabled);

	if (enabled) {
		GTP_DEBUG("hostdl idm enable:%d", enabled);
		gcore_idm_read_id(buff, sizeof(buff));
	} else {
		GTP_ERROR("hostdl idm enable:%d", enabled);
	}

	return count;

}

#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
static ssize_t gesture_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "Gesture wakup is %s\n",
		gdev_fwu->gesture_wakeup_en ? "enable":"disable");
}

static ssize_t gesture_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u8  enable = 0;

    if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1') {
		enable = 1;
    }
    if (enable) {
		gdev_fwu->gesture_wakeup_en = true;
    } else {
		gdev_fwu->gesture_wakeup_en = false;
    }

    return count;
}
static DEVICE_ATTR(gesture_en, S_IRUSR|S_IWUSR, gesture_en_show, gesture_en_store);

#endif

static ssize_t fw_dump_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret = 0;
	int enabled = 0;

	ret = kstrtoint(buf, 0, &enabled);

	if (enabled) {
		GTP_DEBUG("fw save enable:%d", enabled);

		gcore_dump_fw_to_file();

	} else {
		GTP_ERROR("fw save enable:%d", enabled);
	}

	return count;

}

#ifdef CONFIG_ENABLE_FW_RAWDATA
static DEVICE_ATTR(fw_mode, 0200, NULL, fw_mode_set_store);
#endif

static DEVICE_ATTR(reg, 0200, NULL, fw_reg_rw_store);

#ifdef CONFIG_DEBUG_SPI_SPEED
static DEVICE_ATTR(spi_speed, 0200, NULL, spi_adjust);
#endif

static DEVICE_ATTR(fw_update, 0200, NULL, fw_update_flash_store);

static DEVICE_ATTR(ret_update, S_IRUGO, ret_update_show, NULL);

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD
static DEVICE_ATTR(idm, 0200, NULL, hostdl_idm_set_store);
#endif

static DEVICE_ATTR(fw_dump, 0200, NULL, fw_dump_store);

static struct device_attribute *gcore_attribute[] = {
	&dev_attr_reg,
#ifdef CONFIG_DEBUG_SPI_SPEED
	&dev_attr_spi_speed,
#endif
#ifdef CONFIG_ENABLE_FW_RAWDATA
	&dev_attr_fw_mode,
#endif
	&dev_attr_fw_update,
	&dev_attr_ret_update,
#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD
	&dev_attr_idm,
#endif
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	&dev_attr_gesture_en,
#endif
	&dev_attr_fw_dump,
};

int gcore_create_attribute(struct device *dev)
{
	int num = 0;
	int i = 0;
	int ret = 0;

	num = (int)ARRAY_SIZE(gcore_attribute);

	for (i = 0; i < num; i++) {
		ret = device_create_file(dev, gcore_attribute[i]);
		if (ret) {
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(gcore_create_attribute);

static int fwu_event_handler(void *p)
{
	struct gcore_dev *gdev = (struct gcore_dev *)p;
	struct sched_param param = {.sched_priority = 4};

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(gdev->wait, fw_update_fn.event_flag == true);
		fw_update_fn.event_flag = false;

		if (mutex_is_locked(&gdev->transfer_lock)) {
			GTP_DEBUG("fw event is locked, ignore");
			continue;
		}

		mutex_lock(&gdev->transfer_lock);

		switch (gdev->fw_event) {
		case FW_UPDATE:
#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD
			gcore_auto_update_hostdownload(gdev->firmware);
#else
			gcore_flash_op_reply_proc();
#endif
			break;

		case FW_READ_REG:
			gcore_fw_read_reg_reply(gdev->firmware, gdev->fw_xfer);
			break;

		default:
			break;
		}

		mutex_unlock(&gdev->transfer_lock);

	} while (!kthread_should_stop());

	return 0;
}

int gcore_fw_update_fn_init(struct gcore_dev *gdev)
{
	gdev_fwu = gdev;

	fwu_thread = kthread_run(fwu_event_handler, gdev, "gcore_fwu");

	if (gcore_create_attribute(&gdev->bus_device->dev)) {
		GTP_ERROR("update fn create attribute fail");
	}

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD
//	gdev->fwu_workqueue = create_singlethread_workqueue("fwu_workqueue");
//	INIT_DELAYED_WORK(&gdev->fwu_work, gcore_request_firmware_update_work);
	queue_delayed_work(gdev->fwu_workqueue, &gdev->fwu_work,\
		msecs_to_jiffies(STARTUP_UPDATE_DELAY_MS));
#endif

	return 0;
}

void gcore_fw_update_fn_remove(struct gcore_dev *gdev)
{
	int num = 0;
	int i = 0;

	num = (int)ARRAY_SIZE(gcore_attribute);

	for (i = 0; i < num; i++) {
		device_remove_file(&gdev->bus_device->dev, gcore_attribute[i]);
	}

	kthread_stop(fwu_thread);

	return;
}


#if 0

static int __init gcore_fw_update_init(void)
{
	GTP_DEBUG("gcore_fw_update_init.");

	gcore_new_function_register(&fw_update_fn);

	return 0;
}

static void __exit gcore_fw_update_exit(void)
{
	GTP_DEBUG("gcore_fw_update_exit.");

	gcore_new_function_unregister(&fw_update_fn);

	return;
}

module_init(gcore_fw_update_init);
module_exit(gcore_fw_update_exit);

MODULE_AUTHOR("GalaxyCore, Inc.");
MODULE_DESCRIPTION("GalaxyCore Touch FW Update Module");
MODULE_LICENSE("GPL");

#endif
