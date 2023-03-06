/*
 *
 * Zinitix bt532 touch driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */


#ifndef _LINUX_BT541_TS_H
#define _LINUX_BT541_TS_H

#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#define TS_DRVIER_VERSION	"1.0.18_1"

#define CONFIG_DATE "0304"

#define TSP_TYPE_COUNT	1


/* Upgrade Method*/
#define TOUCH_ONESHOT_UPGRADE		0
#define TOUCH_FORCE_UPGRADE		1
#define USE_CHECKSUM			0


#define USE_WAKEUP_GESTURE    0

#define SUPPORTED_PALM_TOUCH  0

#define MAX_SUPPORTED_FINGER_NUM	3 /* max 10 */

//#define TOUCH_POINT_FLAG
#ifdef TOUCH_POINT_FLAG
#define TOUCH_POINT_MODE		1
#else
#define TOUCH_POINT_MODE		0
#endif


#define CHIP_OFF_DELAY			70 /*ms*/
#define CHIP_ON_DELAY			200//100 /*ms*/
#define FIRMWARE_ON_DELAY		150//40 /*ms*/

#define DELAY_FOR_SIGNAL_DELAY		30 /*us*/
#define DELAY_FOR_TRANSCATION		50
#define DELAY_FOR_POST_TRANSCATION	10


/* ESD Protection */
/*second : if 0, no use. if you have to use, 3 is recommended*/
#define ESD_TIMER_INTERVAL		0
#define SCAN_RATE_HZ			100
#define CHECK_ESD_TIMER			3



#define BT541_TS_DEVICE		"bt541_ts_device"


 /*Test Mode (Monitoring Raw Data) */
#define SEC_DND_N_COUNT			10
#define SEC_DND_U_COUNT			2
#define SEC_DND_FREQUENCY		99 /* 200khz */
#define SEC_PDND_N_COUNT		41  //>12
#define SEC_PDND_U_COUNT		9  //6~12
#define SEC_PDND_FREQUENCY		240 //79


#ifdef SUPPORTED_TOUCH_KEY
#define NOT_SUPPORTED_TOUCH_DUMMY_KEY
#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
#define MAX_SUPPORTED_BUTTON_NUM	6 /* max 8 */
#define SUPPORTED_BUTTON_NUM		4
#else
#define MAX_SUPPORTED_BUTTON_NUM	6 /* max 8 */
#define SUPPORTED_BUTTON_NUM		4
#endif
#endif

#define ZINITIX_MAX_STR_LABLE_LEN	64
#define RAW_DATA_BUFFER_SIZE		1024

#define ZINITIX_DEBUG			0
#define TSP_VERBOSE_DEBUG

#define SEC_FACTORY_TEST


#define zinitix_debug_msg(fmt, args...) \
	do { \
		if (m_ts_debug_mode) \
			printk(KERN_ERR "bt541_ts[%-18s:%5d] " fmt, \
					__func__, __LINE__, ## args); \
	} while (0);

#define zinitix_printk(fmt, args...) \
	do { \
		printk(KERN_ERR "bt541_ts[%-18s:%5d] " fmt, \
				__func__, __LINE__, ## args); \
	} while (0);

#define bt541_err(fmt) \
	do { \
		pr_err("bt541_ts : %s " fmt, __func__); \
	} while (0);

#define FILE_NAME_LENGTH 128
#define MAX_RAW_DATA_SZ			576 /* 32x18 */
#define MAX_TRAW_DATA_SZ	\
	(MAX_RAW_DATA_SZ + 4*MAX_SUPPORTED_FINGER_NUM + 2)

extern volatile int tpd_halt;

enum power_control {
	POWER_OFF,
	POWER_ON,
	POWER_ON_SEQUENCE,
};

enum work_state {
	NOTHING = 0,
	NORMAL,
	ESD_TIMER,
	SUSPEND,
	RESUME,
	UPGRADE,
	REMOVE,
	SET_MODE,
	HW_CALIBRAION,
	RAW_DATA,
	PROBE,
};

struct bt541_ts_platform_data {
	int 	gpio_reset;
	int		gpio_int;
	//u32		gpio_scl;
	//u32		gpio_sda;
	//u32		gpio_ldo_en;
	u32		tsp_irq;
#ifdef SUPPORTED_TOUCH_KEY_LED
	int		gpio_keyled;
#endif
	int		tsp_vendor1;
	int		tsp_vendor2;
	int		tsp_en_gpio;
	u32		tsp_supply_type;
	//int (*tsp_power)(int on);
	u16		x_resolution;
	u16		y_resolution;
	u16		page_size;
	u8		orientation;
	const char      *pname;
#ifdef USE_TSP_TA_CALLBACKS
	void (*register_cb) (struct tsp_callbacks *);
	struct tsp_callbacks callbacks;
#endif
	char avdd_name[ZINITIX_MAX_STR_LABLE_LEN];
	char iovdd_name[ZINITIX_MAX_STR_LABLE_LEN];
	int avdd_gpio;
	int iovdd_gpio;
	char ic_name[ZINITIX_MAX_STR_LABLE_LEN];
};

struct coord {
	u16	x;
	u16	y;
	u8	width;
	u8	sub_status;
#if (TOUCH_POINT_MODE == 2)
	u8	minor_width;
	u8	angle;
#endif
};

struct point_info {
	u16	status;
#if (TOUCH_POINT_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif
	struct coord coord[MAX_SUPPORTED_FINGER_NUM];
};

struct capa_info {
	u16	vendor_id;
	u16	ic_revision;
	u16	fw_version;
	u16	fw_minor_version;
	u16     fw_third_version;
	u16	reg_data_version;
	u16	threshold;
	u16	key_threshold;
	u16	dummy_threshold;
	u32	ic_fw_size;
	u32	MaxX;
	u32	MaxY;
	u32	MinX;
	u32	MinY;
	u8	gesture_support;
	u16	multi_fingers;
	u16	button_num;
	u16	ic_int_mask;
	u16	x_node_num;
	u16	y_node_num;
	u16	total_node_num;
	u16	hw_id;
	u16	afe_frequency;
	u16	i2s_checksum;
	u16	shift_value;
	u16	N_cnt;
	u16	u_cnt;
};

struct ts_test_params {
	s32 max_raw_limits[RAW_DATA_BUFFER_SIZE];
	s32 min_raw_limits[RAW_DATA_BUFFER_SIZE];
	s32 short_threshold;
	u8 raw_data_pass;
	u8 short_data_pass;
};

struct bt541_ts_info {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct bt541_ts_platform_data	*pdata;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;

	char				phys[32];

	struct capa_info		cap_info;
	struct point_info		touch_info;
	struct point_info		reported_touch_info;
	u16				icon_event_reg;
	u16				prev_icon_event;

	int				irq;
#ifdef SUPPORTED_TOUCH_KEY
	u8				button[MAX_SUPPORTED_BUTTON_NUM];
#endif
	u8				work_state;
	struct semaphore		work_lock;

	u8 finger_cnt1;

#ifdef USE_TSP_TA_CALLBACKS
	void (*register_cb) (struct tsp_callbacks *tsp_cb);
	struct tsp_callbacks callbacks;
#endif

#if ESD_TIMER_INTERVAL
	struct work_struct		tmr_work;
	struct timer_list		esd_timeout_tmr;
	struct timer_list		*p_esd_timeout_tmr;
	spinlock_t			lock;
#endif
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	struct semaphore		raw_data_lock;
	u16				touch_mode;
	s16				cur_data[MAX_TRAW_DATA_SZ];
	u8				update;
#ifdef SEC_FACTORY_TEST
	struct tsp_factory_info		*factory_info;
	struct tsp_raw_data		*raw_data;
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	bool device_enabled;
	bool checkUMSmode;
	bool irq_enabled;
	bool gesture_enabled;
	u16 gesture_command;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock gesture_wakelock;
#else
	struct wakeup_source *gesture_wakelock;
#endif

#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
	struct ts_mmi_class_methods *imports;
#endif
	struct work_struct cmcp_threshold_update;
	struct completion builtin_cmcp_threshold_complete;
	struct ts_test_params test_params;
	int builtin_cmcp_threshold_status;
	const char *supplier;
	struct delayed_work work;
	int ts_mmi_power_state;
};

extern int zinitix_hw_reset( struct bt541_ts_info* data,bool on );
extern bool bt541_power_control(struct bt541_ts_info *info, u8 ctl);
extern int zinitix_read_file(char *file_name, u8 **file_buf);
extern int ts_upgrade_sequence(const u8 *firmware_data);
extern void clear_report_data(struct bt541_ts_info *info);
extern int zinitix_ts_mmi_gesture_suspend(struct device *dev);
extern int zinitix_ts_mmi_gesture_resume(struct device *dev);
extern bool mini_init_touch(struct bt541_ts_info *info);

#endif /* LINUX_BT541_TS_H */

