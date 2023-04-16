/*
* linux/sensor/dts201a.h
*
* Partron DTS201A Digital Thermopile Sensor module driver
*
* Copyright (C) 2013 Partron Co., Ltd. - Sensor Lab.
* partron (partron@partron.co.kr)
*
* Both authors are willing to be considered the contact and update points for
* the driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 1.0.0 2013/Nov/22:
	first release
 Revision 2.0. 2020/July/02:
  update for coefficient read and add the emissivity caculation equation.

******************************************************************************/

#ifndef	__DTS201A_H__
#define	__DTS201A_H__
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#ifdef CAMERA_CCI
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"
#endif
/**
 * set to 0 1 activate or not debug from work (data interrupt/polling)
 */
#define WORK_DEBUG	0
#if WORK_DEBUG
#	define work_dbg(msg, ...)\
	printk("[D DTS201A] :" msg "\n", ##__VA_ARGS__)
#else
#	define work_dbg(...) (void)0
#endif
#define DEBUG
#ifdef DEBUG
#define dts201a_dbgmsg(str, ...)  \
	pr_info("%s:" str, __func__,##__VA_ARGS__);
#else
#	define dts201a_dbgmsg(...) (void)0
#endif


#define dts201a_info(str, args...) \
	pr_info("%s: %d: " str "\n", __func__,__LINE__, ##args)

#define dts201a_errmsg(str, args...) \
	pr_err("%s:%d: " str, __func__, __LINE__,##args)

#define dts201a_wanrmsg(str, args...) \
	pr_warn("%s:%d: " str, __func__, __LINE__,##args)

/* turn off poll log if not defined */
#ifndef DTS201A_LOG_POLL_TIMING
#	define DTS201A_LOG_POLL_TIMING	0
#endif
/* turn off cci log timing if not defined */
#ifndef DTS201A_LOG_CCI_TIMING
#	define DTS201A_LOG_CCI_TIMING	0
#endif
#define	DTS_I2C_WRITE	0
#define	DTS_I2C_READ	1
#define	DTS201A_SADDR	(0x3A)

#define	DTS201A_ID0 0x0000
#define	DTS201A_ID1 0x0000



//#define DTS201A_MAVG
//#define DTS201A_ONE_MODE

#ifdef DTS201A_ONE_MODE
#define	STX	0x1A
#define	ETX	0x2E
#else	/* DTS201A_TWO_MODE */
#define	STX_N1 0x1A
#define	ETX_N1 0x27
#define	STX_M1 0x13
#define	ETX_M1 0x15
#define	STX_H2 0x28
#define	ETX_H2 0x35
#define	STX_M2 0x36
#define	ETX_M2 0x38
#endif

#define	DTS_VENDOR		"PARTRON"
#define	DTS_CHIP_ID		"DTS201A"
#define	DTS_DEV_NAME		"dts201a"


//#define	DTS201A_ID0 0x3026
//#define	DTS201A_ID1 0x0010
//#define	DTS201A_ID0 0x3027
//#define	DTS201A_ID1 0x0110
//#define	DTS201A_ID0 0x3218
//#define	DTS201A_ID1 0x1110
#define	DTS_DELAY_DEFAULT		200
#define	DTS_DELAY_MINIMUM		30
#define	DTS_CONVERSION_TIME	    30

#ifdef DTS201A_MAVG
#define	DTS100A_MOVING_OAVG	5
#define	DTS100A_MOVING_AAVG	5
#endif

/* CONTROL REGISTERS */
#define	DTS_CUST_ID0		0x00		/* customer ID Low byte */
#define	DTS_CUST_ID1		0x01		/* customer ID High byte */
#define	DTS_SM				0xA0		/* Sensor Measurement */
#define	DTS_SM_USER		0xA1		/* Sensor Measurement User Config */
#define	DTS_SM_AZSM		0xA2		/* Auto-Zero corrected Sensor Measurement */
#define	DTS_SM_AZSM_USER	0xA3		/* Auto-Zero corrected Sensor Measurement User Config */
#define	DTS_TM				0xA4		/* Temperature Measurement */
#define	DTS_TM_USER		0xA5		/* Temperature Measurement User Config */
#define	DTS_TM_AZTM		0xA6		/* Auto-Zero corrected Temperature Measurement */
#define	DTS_TM_AZTM_USER	0xA7		/* Auto-Zero corrected Temperature Measurement User Config */
#define	DTS_START_NOM		0xA8		/* transition to normal mode */
#define	DTS_START_CM		0xA9		/* transition to command mode */
#define	DTS_MEASURE		0xAA		/* measurement cycle and calculation and storage 1*/
#define	DTS_MEASURE1		0xAB		/* measurement cycle and calculation and storage 1*/
#define	DTS_MEASURE2		0xAC		/* measurement cycle and calculation and storage 2*/
#define	DTS_MEASURE4		0xAD		/* measurement cycle and calculation and storage 4*/
#define	DTS_MEASURE8		0xAE		/* measurement cycle and calculation and storage 8*/
#define	DTS_MEASURE16		0xAF		/* measurement cycle and calculation and storage 16*/

//#define	COEFFICIENT_FILE_PATH		"/efs/FactoryApp/coefficient"
#define	COEFFICIENT_FILE_PATH		"/sdcard/Download/coefficient"

#define VDD_VOLTAGE  1800000
struct outputdata {
#ifdef DTS201A_MAVG
	u32 therm_avg[DTS100A_MOVING_OAVG];
	u32 ambient_avg[DTS100A_MOVING_AAVG];
#endif
	u32 therm;	// object temperature
	u32 temp;	// ambient temperature
	u8 status;
};

#ifdef __KERNEL__
struct dts201a_platform_data {
	int irq;
};
#endif /* __KERNEL__ */
struct dts201a_data {
	int id;			/*!< multiple device id 0 based*/
	char name[64];		/*!< misc device name */
	int32_t hw_rev;		/*!< hardware revision number*/

	void *client_object;	/*!< cci or i2c model i/f specific ptr  */
	bool is_device_remove;	/*!< true when device has been remove */

	struct class*  thermopile_class;
	struct mutex lock;
	struct mutex dts201a_i2c_mutex;
	struct workqueue_struct *thermopile_wq;
	struct work_struct work_thermopile;
	struct input_dev *input_dev;
	struct device *dev;
	struct outputdata out;
	struct hrtimer timer;
	ktime_t poll_delay;
	int hw_initialized;
	atomic_t enabled;
	u8 ndata[34];
	u8 hdata[34];

	struct i2c_client *client;
	struct regulator *i2c_power_supply;
	/*!< power enable gpio number
	 *
	 * if -1 no gpio if vdd not avl pwr is not controllable
	*/
	int i2c_pwren_gpio;
	int i2c_vdd_voltage;
	bool vdd_en;
#if defined(CONFIG_DRM)
    struct notifier_block fb_notif;
#endif
};


int dts201a_setup(struct device *dev,struct dts201a_data *data);
int dts201a_cleanup(struct dts201a_data *ther);
int dts201a_enable(struct dts201a_data *ther);
int dts201a_disable(struct dts201a_data *ther);
#endif  /* __DTS201A_H__ */
