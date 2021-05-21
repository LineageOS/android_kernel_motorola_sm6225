/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 46000 $
 * $Date: 2019-06-12 14:25:52 +0800 (週三, 12 六月 2019) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/mmi_kernel_common.h>

#ifdef NVT_SENSOR_EN
#include <linux/sensors.h>
#endif

#ifdef NVT_CONFIG_PANEL_NOTIFICATIONS
#include <linux/panel_notifier.h>
#endif
#ifdef NOVATECH_PEN_NOTIFIER
#include <linux/pen_detection_notify.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx_mem_map.h"

#ifdef CONFIG_MTK_SPI
/* Please copy mt_spi.h file under mtk spi driver folder */
#include "mt_spi.h"
#endif

#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

#include <linux/mmi_wake_lock.h>

#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
#include <linux/touchscreen_mmi.h>
#endif

#define NVT_DEBUG 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943


//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING


//---SPI driver info.---
#define NVT_SPI_NAME "NVT-ts"
#define NVT_PRIMARY_NAME "primary"

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#define NVT_DBG(fmt, args...)    pr_debug("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"


//---Touch info.---
#define TOUCH_COORDS_ARR_SIZE	2
#define TOUCH_DEFAULT_MAX_WIDTH 720
#define TOUCH_DEFAULT_MAX_HEIGHT 1600
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000
#ifdef PALM_GESTURE
#define PALM_HAND_HOLDE 900
#define PALM_HANG 1000
#ifdef PALM_GESTURE_RANGE
#define TOUCH_ORIENTATION_MIN 0
#define TOUCH_ORIENTATION_MAX 90
#define PANEL_REAL_WIDTH 7096
#define PANEL_REAL_HEIGHT 15768
#endif
#endif
#define EDGE_REJECT_VERTICLE_CMD 0xBA
#define EDGE_REJECT_LEFT_UP 0xBB
#define EDGE_REJECT_RIGHT_UP 0xBC
#define VERTICAL   1
#define LEFT_UP   2
#define RIGHT_UP  3

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 0

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#if defined (NVT_SENSOR_EN) || defined (CONFIG_INPUT_TOUCHSCREEN_MMI)
#define WAKEUP_GESTURE 1
#else
#define WAKEUP_GESTURE 0
#endif
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#define MP_UPDATE_FIRMWARE_NAME   "novatek_ts_mp.bin"
#define POINT_DATA_CHECKSUM 1
#define POINT_DATA_CHECKSUM_LEN 65
#define NVT_FILE_NAME_LENGTH                    128
//---ESD Protect.---
#ifdef NVT_CONFIG_ESD_ENABLE
#define NVT_TOUCH_ESD_PROTECT 1
#else
#define NVT_TOUCH_ESD_PROTECT 0
#endif
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */
#define NVT_TOUCH_WDT_RECOVERY 1

#if NVT_TOUCH_ESD_PROTECT
extern struct delayed_work nvt_esd_check_work;
#endif

#ifdef NVT_SENSOR_EN
/* display state */
enum display_state {
	SCREEN_UNKNOWN,
	SCREEN_OFF,
	SCREEN_ON,
};
struct nvt_sensor_platform_data {
	struct input_dev *input_sensor_dev;
	struct sensors_classdev ps_cdev;
	int sensor_opened;
	char sensor_data; /* 0 near, 1 far */
	struct nvt_ts_data *data;
};
#endif

/* charger detect */
#define USB_DETECT_IN 1
#define USB_DETECT_OUT 0
#define CMD_CHARGER_ON (0x53)
#define CMD_CHARGER_OFF (0x51)

struct usb_charger_detection {
	struct notifier_block charger_notif;
	uint8_t usb_connected;
	struct workqueue_struct *nvt_charger_notify_wq;
	struct work_struct charger_notify_work;
};

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	uint8_t bTouchIsAwake;
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint8_t hw_crc;
	uint16_t nvt_pid;
	uint8_t *rbuf;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	const char *panel_supplier;
#ifdef CONFIG_MTK_SPI
	struct mt_chip_conf spi_ctrl;
#endif
	uint32_t charger_detection_enable;
	uint32_t report_gesture_key;
	struct usb_charger_detection *charger_detection;
#ifdef CONFIG_SPI_MT65XX
    struct mtk_chip_config spi_ctrl;
#endif
#ifdef WAKEUP_GESTURE
	bool gesture_enabled;
	bool wakeable;
#endif
#ifdef NVT_SENSOR_EN
	bool should_enable_gesture;
#ifdef NOVATECH_PEN_NOTIFIER
	bool fw_ready_flag;
	int nvt_pen_detect_flag;
	struct notifier_block pen_notif;
#endif
	enum display_state screen_state;
	struct mutex state_mutex;
	struct nvt_sensor_platform_data *sensor_pdata;
#endif
#ifdef PALM_GESTURE
	bool palm_enabled;
#endif
#ifdef EDGE_SUPPRESSION
	uint32_t edge_reject_state;
#endif
	char product_id[10];
	uint8_t fw_type;
	atomic_t loading_fw;
	uint32_t build_id;
	uint32_t config_id;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
#if defined(CONFIG_DRM)
	struct notifier_block drm_notif;
#endif
#else //vension code < 5.4.0
#if defined(CONFIG_FB)
#ifdef _MSM_DRM_NOTIFY_H_
	struct notifier_block drm_notif;
#else
	struct notifier_block fb_notif;
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#ifdef NVT_CONFIG_PANEL_NOTIFICATIONS
	struct notifier_block panel_notif;
#endif
#endif //version code >= 5.4.0

#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
	struct ts_mmi_class_methods *imports;
#endif
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_HOST_CMD_CHECK                = 0x5C,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

#ifdef NVT_SET_TOUCH_STATE
#define MAX_PANEL_IDX 2
enum touch_panel_id {
	TOUCH_PANEL_IDX_PRIMARY = 0,
	TOUCH_PANEL_MAX_IDX,
};

enum touch_state {
	TOUCH_DEEP_SLEEP_STATE = 0,
	TOUCH_LOW_POWER_STATE,
};
#endif

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TRANSFER_LEN	(63*1024)
#define NVT_READ_LEN		(2*1024)

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern structures---
extern struct nvt_ts_data *ts;

extern char *nvt_boot_firmware_name;
extern char *nvt_mp_firmware_name;

//---extern functions---
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void nvt_bootloader_reset(void);
void nvt_eng_reset(void);
void nvt_sw_reset(void);
void nvt_sw_reset_idle(void);
void nvt_boot_ready(void);
void nvt_bld_crc_enable(void);
void nvt_fw_crc_enable(void);
void nvt_irq_enable(bool enable);
int32_t nvt_update_firmware(char *firmware_name);
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t nvt_get_fw_info(void);
int32_t nvt_clear_fw_status(void);
int32_t nvt_check_fw_status(void);
int32_t nvt_set_page(uint32_t addr);
int32_t nvt_write_addr(uint32_t addr, uint8_t data);
int32_t nvt_ts_suspend(struct device *dev);
int32_t nvt_ts_resume(struct device *dev);
int nvt_set_charger(uint8_t charger_on_off);
#ifdef NVT_SET_TOUCH_STATE
int touch_set_state(int state, int panel_idx);
int check_touch_state(int *state, int panel_idx);
#endif
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#ifdef NOVATECH_PEN_NOTIFIER
extern int nvt_mcu_pen_detect_set(uint8_t pen_detect);
#endif
#ifdef PALM_GESTURE
extern int nvt_palm_set(bool enabled);
#endif
#ifdef EDGE_SUPPRESSION
extern int32_t nvt_edge_reject_set(uint32_t status);
extern uint8_t nvt_edge_reject_read(void);
#endif
void release_all_touches(void);
#endif /* _LINUX_NVT_TOUCH_H */
