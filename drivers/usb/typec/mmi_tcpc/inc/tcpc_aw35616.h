/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2022, Shanghai awinic technology Co.,Ltd.
												all rights reserved. ***********
 *******************************************************************************
 * File Name     : tcpc_aw35616.h
 * Author        : awinic
 * Date          : 2022-12-16
 * Description   : .H file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef __LINUX_AW35616_H
#define __LINUX_AW35616_H

/* AW35616 I2C Configuration */
#define AW35616_I2C_RETRIES		(5)
#define AW35616_CHECK_RETEY		(5)

/* aw91xxx register read/write access */
#define REG_NONE_ACCESS			(0)
#define REG_RD_ACCESS			(1 << 0)
#define REG_WR_ACCESS			(1 << 1)
#define AW35616_REG_MAX			(0x23)

/* AW35616 Register Addresses */
#define AW35616_REG_DEV_ID		(0x01)
#define AW35616_REG_CTR			(0x02)
#define AW35616_REG_INT			(0x03)
#define AW35616_REG_STATUS		(0x04)
#define AW35616_REG_STATUS1		(0x05)
#define AW35616_REG_RSTN		(0x06)
#define AW35616_REG_USB_VID0	        (0x07)
#define AW35616_REG_USB_VID1	        (0x08)
#define AW35616_REG_TYPEC_STATUS	(0x20)
#define AW35616_REG_CTR2		(0x22)

/* AW35616 vendor id */
#define AW35616_VENDOR_ID		(0x06)
#define AW35616_reg_val1		(0xf9)
#define AW35616_reg_val2		(0xc2)
#define AW35616_reg_val3		(0x87)
#define AW35616_reg_val4		(0x80)
#define AW35616_reg_val5		(0x00)

/**
 * power role mode control
 */
typedef enum {
	SNK       = 0,
	SRC       = 1,
	DRP       = 2,
	WAKE_MODE = 3
} WKMD;
#define PR_MODE_SET				(WAKE_MODE)

/**
 * source current mode set
 */
typedef enum {
	SRC_DEF    = 0,
	SRC_1_5A   = 1,
	SRC_3_0A   = 2,
	OR_SRC_DEF = 3
} SRC_CUR_MD;
#define SRC_CUR_SET				(SRC_DEF)

/**
 * try src/snk mode set
 */
typedef enum {
	NO_TRY    = 0,
	TRY_SNK   = 1,
	TRY_SRC   = 2,
	OR_NO_TRY = 3
} TRY_MD;
#define TRY_MODE_SET			(TRY_SNK)

/**
 * accessory mode set
 */
#define ACC_MODE_SET			(1)

/**
 * Disable pull-up and pull-down after a toggle cycle
 */
typedef enum {
	TOGGLE_CYCCLE_DEF   = 0,
	TOGGLE_CYCCLE_40ms  = 1,
	TOGGLE_CYCCLE_80ms  = 2,
	TOGGLE_CYCCLE_160ms = 3
} TOG_SAVE_MD;
#define TOGGLE_CYCCLE_SET		(TOGGLE_CYCCLE_DEF)

/**
 * intb flag
 */
typedef enum {
	NO_INTB   = 0,
	ATTACHED  = 1,
	DETACHED  = 2
} INTB_FLAG;

/**
 * plag direction
 */
typedef enum {
	STANDBY   = 0,
	CC1       = 1,
	CC2       = 2,
	CC1_CC2   = 3
} PLAG_ORI;

/**
 * plag status
 */
typedef enum {
	SINK      = 1,
	SOURCE    = 2,
	AUD_ACC   = 3,
	DUG_ACC   = 4
} PLAG_ST;

/**
 * charging current detection as sink
 */
typedef enum {
	SNK_DEF   = 1,
	SNK_1_5A  = 2,
	SNK_3_0A  = 3
} SINK_CUR;

/**
 * vbus detection as sink
 */
typedef enum {
	VBUS_DIS  = 0,
	VBUS_OK   = 1
} VBUS_ST;

typedef union {
	u8 byte;
	struct {
		u8 vd_id:3;
		u8 ver_id:5;
	};
} reg_dev_id;

typedef union {
	u8 byte;
	struct {
		u8 intdis:1;
		u8 wkmd:2;
		u8 src_cur_md:2;
		u8 try_md:2;
		u8 accdis:1;
	};
} reg_ctr;

typedef union {
	u8 byte;
	struct {
		u8 intb_flag:2;
		u8 wake_flag:1;
		u8 reserved:5;
	};
} reg_ints;

typedef union {
	u8 byte[2];
	struct {
		/* status */
		u8 plug_ori:2;
		u8 plug_st:3;
		u8 snk_cur_md:2;
		u8 vbusok:1;
		/* status1 */
		u8 active_cable:1;
		u8 wake_st:1;
		u8 snk_det_rp_dbg:1;
		u8 reserved:5;
	};
} reg_status;

typedef union {
	u8 byte;
	struct {
		u8 sft_rstn:1;
		u8 typec_rstn:1;
		u8 reserved:6;
	};
} reg_rstn;

typedef union {
	u8 byte[2];
	struct {
		/* vid0 */
		u8 usb_vid_lsb:8;
		/* vid1 */
		u8 usb_vid_msb:8;
	};
} reg_vid;

typedef union {
	u8 byte;
	struct {
		u8 reserved:1;
		u8 tog_save_md:2;
		u8 reserved1:5;
	};
} reg_ctr2;

typedef struct {
	reg_dev_id dev_id;
	reg_ctr    ctr;
	reg_ints   ints;
	reg_status status;
	reg_rstn   rstn;
	reg_vid    vid;
	reg_ctr2   ctr2;
} aw35616_reg;

struct aw35616_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
	struct kthread_worker irq_worker;
	struct kthread_work irq_work;
	struct task_struct *irq_worker_task;
	struct wakeup_source *for_irq_wake_lock;
	struct wakeup_source *for_i2c_wake_lock;

	struct delayed_work first_check_typec_work;

	aw35616_reg reg;
	uint8_t toggle_cycle;
	bool acc_support;

	int irq_gpio;
	int irq;
};

enum aw35616_mode {
	REVERSE_CHG_DRP,
	REVERSE_CHG_SINK,
	REVERSE_CHG_SOURCE,
};

#define AWINIC_DEBUG
#ifdef AWINIC_DEBUG
#define AWINIC_LOG_NAME "AW35616"
#define AW_LOG(format, arg...) pr_err("[%s] %s %d: " format, AWINIC_LOG_NAME, \
		__func__, __LINE__, ##arg)
#else
#define AW_LOG(format, arg...)
#endif

#endif

