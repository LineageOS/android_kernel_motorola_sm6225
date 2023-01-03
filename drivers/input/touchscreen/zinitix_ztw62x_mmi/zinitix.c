/*
 *
 * Zinitix bt541 touchscreen driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */




#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/regulator/machine.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <linux/power_supply.h>

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/firmware.h>

#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "zinitix_touch.h"
#include "zinitix_ts_mmi.h"
#include "zinitix_touch_zxt_firmware.h"

#if (TSP_TYPE_COUNT == 1)
u8 *m_pFirmware [TSP_TYPE_COUNT] = {(u8*)m_firmware_data,};
#else
u8 *m_pFirmware [TSP_TYPE_COUNT] = {(u8*)m_firmware_data_01,(u8*)m_firmware_data_02,};
#endif
u8 m_FirmwareIdx = 0;

#define TSP_HW_ID_INDEX_0 1
#define TSP_HW_ID_INDEX_1 2

#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
extern char *saved_command_line;
#endif

int easy_wake = 1;
int big_object = 1;

/* if you use isp mode, you must add i2c device :
name = "zinitix_isp" , addr 0x50*/
/* resolution offset */
#define ABS_PT_OFFSET			(-1)

/* PMIC Regulator based supply to TSP */
#define TSP_REGULATOR_SUPPLY		1
/* gpio controlled LDO based supply to TSP */
#define TSP_LDO_SUPPLY			0

/* Key Enum */
enum key_event {
	ICON_BUTTON_UNCHANGE,
	ICON_BUTTON_DOWN,
	ICON_BUTTON_UP,
};

#define FT_VTG_MIN_UV		2800000
#define FT_VTG_MAX_UV		2800000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000


#define PINCTRL_STATE_ACTIVE	"cli_pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"cli_pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"cli_pmx_ts_release"


/* preriod raw data interval */

#define RAWDATA_DELAY_FOR_HOST		100

struct raw_ioctl {
	int sz;
	u8 *buf;
};

struct reg_ioctl {
	int addr;
	int *val;
};

#define TOUCH_SEC_MODE			48
#define TOUCH_REF_MODE			10
#define TOUCH_NORMAL_MODE		5
#define TOUCH_DELTA_MODE		3
#define TOUCH_DND_MODE			6
#define TOUCH_PDND_MODE			11

#define TOUCH_CHECK_SHORT_MODE	14
#define CHECK_SHORT_DATA_NUM    24

/*  Other Things */
#define INIT_RETRY_CNT			3
#define I2C_SUCCESS			0
#define I2C_FAIL			1

/*------------Link  modify IC flash size 20180523-------------------*/

/* chip code */
#define ZTW523_CHIP_CODE	0xE628
#define ZTW522_CHIP_CODE	0xE532
#define ZT7548_CHIP_CODE	0xE548
#define ZT7538_CHIP_CODE	0xE538
#define ZT7554_CHIP_CODE	0xE700
#define ZTW622_CHIP_CODE    0xE622
#define ZTW623_CHIP_CODE    0xE623

/*------------Link  modify IC flash size 20180523-------------------*/

/* Register Map*/
#define BT541_SWRESET_CMD		0x0000
#define BT541_WAKEUP_CMD		0x0001

#define BT541_IDLE_CMD			0x0004
#define BT541_SLEEP_CMD			0x0005

#define BT541_CLEAR_INT_STATUS_CMD	0x0003
#define BT541_CALIBRATE_CMD		0x0006
#define BT541_SAVE_STATUS_CMD		0x0007
#define BT541_SAVE_CALIBRATION_CMD	0x0008
#define BT541_RECALL_FACTORY_CMD	0x000f

#define BT541_THRESHOLD			0x0020

#define BT541_DEBUG_REG			0x0115 /* 0~7 */

#define BT541_TOUCH_MODE		0x0010
#define BT541_CHIP_REVISION		0x0011
#define BT541_FIRMWARE_VERSION		0x0012

#define ZINITIX_USB_DETECT	0x116

#define BT541_MINOR_FW_VERSION		0x0121

#define BT541_VENDOR_ID			0x001C
#define BT541_HW_ID			0x0014

#define BT541_DATA_VERSION_REG		0x0013
#define BT541_SUPPORTED_FINGER_NUM	0x0015
#define BT541_EEPROM_INFO		0x0018
#define BT541_INITIAL_TOUCH_MODE	0x0019

#define BT541_TOTAL_NUMBER_OF_X		0x0060
#define BT541_TOTAL_NUMBER_OF_Y		0x0061

#define BT541_DELAY_RAW_FOR_HOST	0x007f

#define BT541_BUTTON_SUPPORTED_NUM	0x00B0
#define BT541_BUTTON_SENSITIVITY	0x00B2
#define BT541_DUMMY_BUTTON_SENSITIVITY	0X00C8

#define BT541_X_RESOLUTION		0x00C0
#define BT541_Y_RESOLUTION		0x00C1

#define BT541_POINT_STATUS_REG		0x0080
#define BT541_ICON_STATUS_REG		0x00AA

#define BT541_AFE_FREQUENCY		0x0100
#define BT541_DND_N_COUNT		0x0122
#define BT541_DND_U_COUNT		0x0135

#define BT541_RAWDATA_REG		0x0200

#define BT541_EEPROM_INFO_REG		0x0018

#define BT541_INT_ENABLE_FLAG		0x00f0
#define BT541_PERIODICAL_INTERRUPT_INTERVAL	0x00f1

#define BT541_BTN_WIDTH			0x016d

#define BT541_CHECKSUM_RESULT		0x012c

#define BT541_INIT_FLASH		0x01d0
#define BT541_WRITE_FLASH		0x01d1
#define BT541_READ_FLASH		0x01d2

#define ZINITIX_INTERNAL_FLAG_02		0x011e
#define ZINITIX_INTERNAL_FLAG_03		0x011f

#define	ZINITIX_I2C_CHECKSUM_WCNT	0x016a
#define	ZINITIX_I2C_CHECKSUM_RESULT	0x016c

/* Interrupt & status register flag bit
-------------------------------------------------
*/
#define BIT_PT_CNT_CHANGE	0
#define BIT_DOWN		1
#define BIT_MOVE		2
#define BIT_UP			3
#define BIT_PALM		4
#define BIT_PALM_REJECT		5
#define RESERVED_0		6
#define RESERVED_1		7
#define BIT_WIDTH_CHANGE	8
#define BIT_PT_NO_CHANGE	9
#define BIT_REJECT		10
#define BIT_PT_EXIST		11
#define RESERVED_2		12
#define BIT_MUST_ZERO		13
#define BIT_DEBUG		14
#define BIT_ICON_EVENT		15

/* button */
#define BIT_O_ICON0_DOWN	0
#define BIT_O_ICON1_DOWN	1
#define BIT_O_ICON2_DOWN	2
#define BIT_O_ICON3_DOWN	3
#define BIT_O_ICON4_DOWN	4
#define BIT_O_ICON5_DOWN	5
#define BIT_O_ICON6_DOWN	6
#define BIT_O_ICON7_DOWN	7

#define BIT_O_ICON0_UP		8
#define BIT_O_ICON1_UP		9
#define BIT_O_ICON2_UP		10
#define BIT_O_ICON3_UP		11
#define BIT_O_ICON4_UP		12
#define BIT_O_ICON5_UP		13
#define BIT_O_ICON6_UP		14
#define BIT_O_ICON7_UP		15


#define SUB_BIT_EXIST		0
#define SUB_BIT_DOWN		1
#define SUB_BIT_MOVE		2
#define SUB_BIT_UP		3
#define SUB_BIT_UPDATE		4
#define SUB_BIT_WAIT		5

/* Mode status */
#define	TS_USB_DETECT_BIT		0
#define	TS_SVIEW_DETECT_BIT		1
#define	TS_SENSIVE_MODE_BIT		2

#define zinitix_bit_set(val, n)		((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define zinitix_bit_clr(val, n)		((val) &= ~(1<<(n)))
#define zinitix_bit_test(val, n)	((val) & (1<<(n)))
#define zinitix_swap_v(a, b, t)		((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s)		(((((s) & 0xff) << 8) | (((s) >> 8) & 0xff)))


extern int get_lcd_attached(void);
extern unsigned int system_rev;
static volatile int __maybe_unused tpd_halt = 0;

struct class *sec_class;
/* end header file */

#ifdef SEC_FACTORY_TEST
/* Touch Screen */
#define TSP_CMD_STR_LEN			32
#define TSP_CMD_RESULT_STR_LEN		512
#define TSP_CMD_PARAM_NUM		8
#define TSP_CMD_Y_NUM			18
#define TSP_CMD_X_NUM			30
#define TSP_CMD_NODE_NUM		(TSP_CMD_Y_NUM * TSP_CMD_X_NUM)
#define tostring(x) #x

struct tsp_factory_info {
	struct list_head cmd_list_head;
	char cmd[TSP_CMD_STR_LEN];
	char cmd_param[TSP_CMD_PARAM_NUM];
	char cmd_result[TSP_CMD_RESULT_STR_LEN];
	char cmd_buff[TSP_CMD_RESULT_STR_LEN];
	struct mutex cmd_lock;
	bool cmd_is_running;
	u8 cmd_state;
};

struct tsp_raw_data {
	//Link  modified  here on 20190506 for  circle  design the  corner minus value
	s16 ref_data[TSP_CMD_NODE_NUM];
	s16 pref_data[TSP_CMD_NODE_NUM];
	//u16 ref_data[TSP_CMD_NODE_NUM];
	//u16 pref_data[TSP_CMD_NODE_NUM];
	/*s16 scantime_data[TSP_CMD_NODE_NUM]; */
	s16 delta_data[TSP_CMD_NODE_NUM];
};

enum {
	WAITING = 0,
	RUNNING,
	OK,
	FAIL,
	NOT_APPLICABLE,
};

struct tsp_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func)(void *device_data);
};
#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_early_suspend(struct early_suspend *h);
static void zinitix_late_resume(struct early_suspend *h);
#endif

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void not_support_cmd(void *device_data);

/* Vendor dependant command */
/* static void run_reference_read(void *device_data); */
static void get_reference(void *device_data);
static void run_preference_read(void *device_data);
static void run_short_data_read(void *device_data);
static void get_preference(void *device_data);
static void run_delta_read(void *device_data);
static void get_delta(void *device_data);
static void get_module_vendor(void *device_data);
static void get_config_ver(void *device_data);
#ifdef GLOVE_MODE
static void glove_mode(void *device_data);
#endif
#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

static struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", module_off_slave),},
	{TSP_CMD("module_on_slave", module_on_slave),},
	{TSP_CMD("get_module_vendor", get_module_vendor),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},

	/* vendor dependant command */
	{TSP_CMD("run_reference_read", run_preference_read),},
	{TSP_CMD("run_short_data_read", run_short_data_read),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("run_dnd_read", run_preference_read),},
	{TSP_CMD("get_dnd", get_preference),},
	{TSP_CMD("run_delta_read", run_delta_read),},
	{TSP_CMD("get_delta", get_delta),},
	{TSP_CMD("get_config_ver", get_config_ver),},
#ifdef GLOVE_MODE
	{TSP_CMD("glove_mode", glove_mode),},
#endif
};

#endif

#define TSP_NORMAL_EVENT_MSG 1
static int m_ts_debug_mode = ZINITIX_DEBUG;
#ifdef USE_TSP_TA_CALLBACKS
static bool ta_connected =0;
#endif
void (*tsp_charger_status_cb)(int);
static u16 m_optional_mode = 0;


#if ESD_TIMER_INTERVAL
static struct workqueue_struct *esd_tmr_workqueue;
#endif

#define TOUCH_V_FLIP	0x01
#define TOUCH_H_FLIP	0x02
#define TOUCH_XY_SWAP	0x04

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct bt541_ts_info *misc_touch_dev = NULL;

/* Dummy touchkey code */
#define KEY_DUMMY_HOME1		249
#define KEY_DUMMY_HOME2		250
#define KEY_DUMMY_MENU		251
#define KEY_DUMMY_HOME		252
#define KEY_DUMMY_BACK		253

/*<= you must set key button mapping*/
#ifdef SUPPORTED_TOUCH_KEY
#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_MENU, KEY_HOME, KEY_BACK};
#else
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_DUMMY_MENU, KEY_RECENT,// KEY_DUMMY_HOME1,
	/*KEY_DUMMY_HOME2,*/ KEY_BACK, KEY_DUMMY_BACK};
#endif
#endif

#ifdef USE_TSP_TA_CALLBACKS
static void bt541_set_ta_status(struct bt541_ts_info *info, bool force);
#endif

/* define i2c sub functions*/
static inline s32 read_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
retry:
	/* select register*/
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		mdelay(1);

		if (++count < 8)
			goto retry;

		return ret;
	}
	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 write_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	//int count = 0;
	u8 pkt[10]; /* max packet */
	pkt[0] = (reg) & 0xff; /* reg addr */
	pkt[1] = (reg >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

//retry:
	ret = i2c_master_send(client , pkt , length + 2);
	if (ret < 0) {
	//	mdelay(1);

	//	if (++count < 8)
	//		goto retry;

		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (write_data(client, reg, (u8 *)&value, 2) < 0)
		return I2C_FAIL;

	return I2C_SUCCESS;
}

static inline s32 write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;
	//int count = 0;

//retry:
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
	//	mdelay(1);

	//	if (++count < 8)
	//		goto retry;

		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);
	return I2C_SUCCESS;
}

static inline s32 read_raw_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;

retry:
	/* select register */
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		mdelay(1);

		if (++count < 8)
			goto retry;

		return ret;
	}

	/* for setup tx transaction. */
	udelay(200);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 read_firmware_data(struct i2c_client *client,
	u16 addr, u8 *values, u16 length)
{
	s32 ret;
	/* select register*/

	ret = i2c_master_send(client , (u8 *)&addr , 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	mdelay(1);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static bool bt541_power_sequence(struct bt541_ts_info *info);
//static bool zinitix_resume_sequence(struct bt541_ts_info *info);
static int zinitix_power_control(struct bt541_ts_info *data, int on);


static bool init_touch(struct bt541_ts_info *info);
#if ESD_TIMER_INTERVAL
static void esd_timer_start(u16 sec, struct bt541_ts_info *info);
static void esd_timer_stop(struct bt541_ts_info *info);
static void esd_timer_init(struct bt541_ts_info *info);
static void esd_timeout_handler(unsigned long data);
#endif

static long ts_misc_fops_ioctl(struct file *filp, unsigned int cmd,
								unsigned long arg);
static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
	.unlocked_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};

#define TOUCH_IOCTL_BASE	0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION		_IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION			_IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION	_IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE		_IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA		_IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE			_IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM			_IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM			_IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE		_IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA			_IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION			_IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG					_IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG					_IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS		_IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT		_IOW(TOUCH_IOCTL_BASE, 19, int)

struct bt541_ts_info *misc_info;
#ifdef CONFIG_OF
static int zinitix_init_gpio(struct bt541_ts_info* data,bool on );
//static int resume_hw_reset( struct bt541_ts_info* data,bool on );

#endif


#define I2C_BUFFER_SIZE 64

#ifdef SEC_FACTORY_TEST
static bool get_raw_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	int i;
	u32 temp_sz;

	disable_irq(info->irq);
	info->irq_enabled = false;

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied (%d)\n",
				__func__, info->work_state);
		enable_irq(info->irq);
		info->irq_enabled = true;
		up(&info->work_lock);

		return false;
	}

	info->work_state = RAW_DATA;

	for(i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			mdelay(1);

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		mdelay(1);
	}

	sz = total_node * 2;

	while (gpio_get_value(pdata->gpio_int))
		mdelay(1);

	for(i = 0; sz > 0; i++){
		temp_sz = I2C_BUFFER_SIZE;

		if(sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (read_raw_data(client, BT541_RAWDATA_REG + i, (char *)(buff + (i*I2C_BUFFER_SIZE)), temp_sz) < 0) {
			dev_err(&client->dev, "%s: Failed to read raw data\n", __func__);
			info->work_state = NOTHING;
	 		enable_irq(info->irq);
			info->irq_enabled = true;
			up(&info->work_lock);

			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}

	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	info->irq_enabled = true;
	up(&info->work_lock);

	return true;
}
#endif


static bool get_short_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
    int i;

	disable_irq(info->irq);
	info->irq_enabled = false;

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied (%d)\n",
				__func__, info->work_state);
		enable_irq(info->irq);
		info->irq_enabled = true;
		up(&info->work_lock);

		return false;
	}

	info->work_state = RAW_DATA;

	for(i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			msleep(1);

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		msleep(1);
	}

	while (gpio_get_value(pdata->gpio_int))
		msleep(1);

	if (read_raw_data(client, BT541_RAWDATA_REG, (char *)(buff), CHECK_SHORT_DATA_NUM) < 0) {
		dev_err(&client->dev, "%s: Failed to read short data\n", __func__);
		info->work_state = NOTHING;
		enable_irq(info->irq);
		info->irq_enabled = true;
		up(&info->work_lock);

		return false;
	}



	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	info->irq_enabled = true;
	up(&info->work_lock);

	return true;
}

static bool ts_get_raw_data(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	u16 temp_sz;
	int	i;

	if (down_trylock(&info->raw_data_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n", __func__);
		info->touch_info.status = 0;

		return true;
	}

	sz = total_node * 2 + sizeof(struct point_info);

	for(i = 0; sz > 0; i++){
		temp_sz = I2C_BUFFER_SIZE;

		if(sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (read_raw_data(client, BT541_RAWDATA_REG + i, (char *)((u8*)(info->cur_data)+ (i*I2C_BUFFER_SIZE)), temp_sz) < 0) {
			dev_err(&client->dev, "%s: Failed to read raw data\n", __func__);
			up(&info->raw_data_lock);

			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}
	info->update = 1;
	memcpy((u8 *)(&info->touch_info),
		(u8 *)&info->cur_data[total_node],
			sizeof(struct point_info));
	up(&info->raw_data_lock);

	return true;
}

static bool ts_read_coord(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
#if (TOUCH_POINT_MODE == 1)
	int i;
#endif

	 zinitix_debug_msg("ts_read_coord+\r\n");

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (ts_get_raw_data(info) == false)
			return false;

		dev_err(&client->dev, "status = 0x%04X\n", info->touch_info.status);

		goto out;
	}

#if (TOUCH_POINT_MODE == 1)
	memset(&info->touch_info,
			0x0, sizeof(struct point_info));

	if (read_data(info->client, BT541_POINT_STATUS_REG,
			(u8 *)(&info->touch_info), 4) < 0) {
		dev_err(&client->dev, "%s: Failed to read point info\n", __func__);

		return false;
	}

	dev_dbg(&client->dev, "status reg = 0x%x , event_flag = 0x%04x\n",
				info->touch_info.status, info->touch_info.event_flag);

	if (info->touch_info.event_flag == 0)
		goto out;

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(info->touch_info.event_flag, i)) {
			udelay(20);

			if (read_data(info->client, BT541_POINT_STATUS_REG + 2 + ( i * 4),
					(u8 *)(&info->touch_info.coord[i]),
				sizeof(struct coord)) < 0) {
				dev_err(&client->dev, "Failed to read point info\n");

				return false;
			}
		}
	}

#else
		if (read_data(info->client, BT541_POINT_STATUS_REG,
				(u8 *)(&info->touch_info), sizeof(struct point_info)) < 0) {
			dev_err(&client->dev, "Failed to read point info\n");

			return false;
		}
#endif

out:
	/* error */
	if (zinitix_bit_test(info->touch_info.status, BIT_MUST_ZERO) &&
		info->touch_info.status != 0xffff) {
		dev_err(&client->dev, "Invalid must zero bit(%04x)\n",
			info->touch_info.status);
		/*write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(DELAY_FOR_SIGNAL_DELAY);*/
		return false;
	}

	return true;
}

#if ESD_TIMER_INTERVAL
static void esd_timeout_handler(unsigned long data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)data;

	info->p_esd_timeout_tmr = NULL;
	queue_work(esd_tmr_workqueue, &info->tmr_work);
}

static void esd_timer_start(u16 sec, struct bt541_ts_info *info)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	if (info->p_esd_timeout_tmr != NULL)
#ifdef CONFIG_SMP
		del_singleshot_timer_sync(info->p_esd_timeout_tmr);
#else
		del_timer(info->p_esd_timeout_tmr);
#endif
	info->p_esd_timeout_tmr = NULL;
	//init_timer(&(info->esd_timeout_tmr));
	//info->esd_timeout_tmr.data = (unsigned long)(info);
	//info->esd_timeout_tmr.function = esd_timeout_handler;
	info->esd_timeout_tmr.expires = jiffies + (HZ * sec);
	info->p_esd_timeout_tmr = &info->esd_timeout_tmr;
	add_timer(&info->esd_timeout_tmr);
	spin_unlock_irqrestore(&info->lock, flags);
}

static void esd_timer_stop(struct bt541_ts_info *info)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	if (info->p_esd_timeout_tmr)
#ifdef CONFIG_SMP
		del_singleshot_timer_sync(info->p_esd_timeout_tmr);
#else
		del_timer(info->p_esd_timeout_tmr);
#endif

	info->p_esd_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void esd_timer_init(struct bt541_ts_info *info)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	//init_timer(&(info->esd_timeout_tmr));
	//info->esd_timeout_tmr.data = (unsigned long)(info);
	//info->esd_timeout_tmr.function = esd_timeout_handler;
	info->p_esd_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void ts_tmr_work(struct work_struct *work)
{
	struct bt541_ts_info *info =
			container_of(work, struct bt541_ts_info, tmr_work);
	struct i2c_client *client = info->client;

#if defined(TSP_VERBOSE_DEBUG)
	dev_err(&client->dev, "tmr queue work ++\n");
#endif

	if (down_trylock(&info->work_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n", __func__);
		esd_timer_start(CHECK_ESD_TIMER, info);

		return;
	}

	if (info->work_state != NOTHING) {
		dev_info(&client->dev, "%s: Other process occupied (%d)\n",
			__func__, info->work_state);
		up(&info->work_lock);

		return;
	}
	info->work_state = ESD_TIMER;

	disable_irq(info->irq);
	info->irq_enabled = false;
	//bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	clear_report_data(info);
	if (mini_init_touch(info) == false)
		goto fail_time_out_init;

	info->work_state = NOTHING;
	enable_irq(info->irq);
	info->irq_enabled = true;
	up(&info->work_lock);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "tmr queue work--\n");
#endif

	return;
fail_time_out_init:
	dev_err(&client->dev, "%s: Failed to restart\n", __func__);
	esd_timer_start(CHECK_ESD_TIMER, info);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	info->irq_enabled = true;
	up(&info->work_lock);

	return;
}
#endif

static bool bt541_power_sequence(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;
	u16 chip_code;

	info->cap_info.ic_fw_size = 44*1024;

retry_power_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(vendor cmd enable)\n");
		goto fail_power_sequence;
	}
	udelay(10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		dev_err(&client->dev, "Failed to read chip code\n");
		goto fail_power_sequence;
	}

	dev_info(&client->dev, "%s: chip code = 0x%x\n", __func__, chip_code);
	udelay(10);

	//Link modify the flash size
	if(chip_code == ZT7554_CHIP_CODE)
		info->cap_info.ic_fw_size = 64*1024;
	else if((chip_code == ZT7548_CHIP_CODE)||(chip_code == ZTW523_CHIP_CODE)||(chip_code == ZTW622_CHIP_CODE)||(chip_code == ZTW623_CHIP_CODE))
		info->cap_info.ic_fw_size = 48*1024;
	else if((chip_code == ZT7538_CHIP_CODE)||(chip_code == ZTW522_CHIP_CODE))
		info->cap_info.ic_fw_size = 44*1024;


	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(intn clear)\n");
		goto fail_power_sequence;
	}
	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(nvm init)\n");
		goto fail_power_sequence;
	}
	mdelay(2);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(program start)\n");
		goto fail_power_sequence;
	}
	mdelay(FIRMWARE_ON_DELAY);	/* wait for checksum cal */

	dev_info(&client->dev, "bt541_power_sequence: OK\n");

	return true;

fail_power_sequence:
	if (retry++ < 3) {
		mdelay(CHIP_ON_DELAY);
		dev_info(&client->dev, "retry = %d\n", retry);
		goto retry_power_sequence;
	}

	dev_err(&client->dev, "Failed to send power sequence\n");
	return false;
}
/*
static bool zinitix_resume_sequence(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;


retry_zinitix_resume_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(vendor cmd enable)\n");
		goto fail_zinitix_resume_sequence;
	}
	mdelay(1);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(nvm init)\n");
		goto fail_zinitix_resume_sequence;
	}
	mdelay(1);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(program start)\n");
		goto fail_zinitix_resume_sequence;
	}
	mdelay(1);*/	/* wait for checksum cal */
/*
	dev_info(&client->dev, "bt541_power_sequence: OK\n");

	return true;

fail_zinitix_resume_sequence:
	if (retry++ < 3) {
		mdelay(CHIP_ON_DELAY);
		dev_info(&client->dev, "retry = %d\n", retry);
		goto retry_zinitix_resume_sequence;
	}
	dev_err(&client->dev, "Failed to send power sequence\n");
	return false;
}
*/
static int zinitix_power_control(struct bt541_ts_info *data, int on)
{
	int rc;
	int avdd_gpio = data->pdata->avdd_gpio;
	int iovdd_gpio = data->pdata->iovdd_gpio;

	if (!on)
		goto power_off;

	if (avdd_gpio > 0) {
		gpio_direction_output(avdd_gpio, 1);
		dev_info(&data->client->dev, "Set avdd_gpio to 1\n");
	} else if (data->vdd && !regulator_is_enabled(data->vdd)) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto power_off;
		}
		dev_info(&data->client->dev, "Enable data->vdd regualtor\n");
	}

	if (iovdd_gpio > 0) {
		gpio_direction_output(iovdd_gpio, 1);
		dev_info(&data->client->dev, "Set iovdd_gpio to 1\n");
	} else if (data->vcc_i2c && !regulator_is_enabled(data->vcc_i2c)) {
		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto power_off;
		}
		dev_info(&data->client->dev, "Enable data->vcc_i2c regualtor\n");
	}

	return 0;

power_off:
	if (avdd_gpio > 0) {
		gpio_direction_output(avdd_gpio, 0);
		dev_info(&data->client->dev, "Set avdd_gpio to 0\n");
	}
	else if (data->vdd) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			return rc;
		}
		dev_info(&data->client->dev, "Disable data->vdd regualtor\n");
	}

	if (iovdd_gpio > 0) {
		gpio_direction_output(iovdd_gpio, 0);
		dev_info(&data->client->dev, "Set iovdd_gpio to 0\n");
	}
	else if (data->vcc_i2c) {
		rc = regulator_disable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
				"%s: Regulator vcc_i2c disable failed rc=%d\n", __func__, rc);
			return rc;
		}
		dev_info(&data->client->dev, "Disable data->vcc_i2c regualtor\n");
	}

	return 0;
}



bool bt541_power_control(struct bt541_ts_info *info, u8 ctl)
{
	pr_info("[TSP] %s, %d\n", __func__, ctl);

	if (ctl == POWER_OFF) {
		zinitix_power_control(info, 0);
		mdelay(CHIP_OFF_DELAY);
	} else if (ctl == POWER_ON_SEQUENCE) {
		printk("bt541_power_control: POWER_ON_SEQUENCE\n");
		zinitix_power_control(info, 1);
		zinitix_hw_reset(info,true);
		mdelay(CHIP_ON_DELAY);
		return bt541_power_sequence(info); //zxt power on sequence
	} else if (ctl == POWER_ON) {
		zinitix_power_control(info, 1);
		zinitix_hw_reset(info,true);
		mdelay(CHIP_ON_DELAY);
	}
	return true;
}

#ifdef USE_TSP_TA_CALLBACKS
static void bt541_set_ta_status(struct bt541_ts_info *info, bool force)
{
	printk("bt541_set ta_connected = %d\n", ta_connected);

	if(info == NULL)
		return;

	if (ta_connected)
		zinitix_bit_set(m_optional_mode, TS_USB_DETECT_BIT);
	else
		zinitix_bit_clr(m_optional_mode, TS_USB_DETECT_BIT);
}

static void bt541_charger_status_cb(struct tsp_callbacks *cb, int status)
{
	printk("bt541_charger status = %d\n", status);

	if (status)
		ta_connected = true;
	else
		ta_connected = false;

	bt541_set_ta_status(misc_info, true);

	dev_info(&misc_info->client->dev, "TA %s\n",
		status ? "connected" : "disconnected");
}
#endif

static void ts_select_type_hw(struct bt541_ts_info *info) {

#if (TSP_TYPE_COUNT == 1)
	m_FirmwareIdx = 0;
#else
	int i;
	u16 newHWID;

/* In case of TSP IC's firmware is broken,
	it will always be updated to HW ID 02 firmware even though HW 01 device exists*/
	for(i = 0; i < TSP_TYPE_COUNT; i++) {
		newHWID = (u16) (m_pFirmware[i][0x7528] | (m_pFirmware[i][0x7529]<<8));

		if(info->cap_info.hw_id == newHWID)
			break;
	}

	m_FirmwareIdx = i;
	if(i == TSP_TYPE_COUNT)
		m_FirmwareIdx = 1;
	zinitix_printk(KERN_INFO "firmwaretype = %d Firmware HWID = %u cap_info.hw_id = %u i = %d \n",
		m_FirmwareIdx, newHWID, info->cap_info.hw_id, i);
#endif
}

#if TOUCH_ONESHOT_UPGRADE
static bool ts_check_need_upgrade(struct bt541_ts_info *info,
	u16 cur_version, u16 cur_minor_version, u16 cur_reg_version)
{
	u16	new_version;
	u16	new_minor_version;
	u16	new_reg_version;

	u8 *firmware_data;

	ts_select_type_hw(info);
	firmware_data = (u8*)m_pFirmware[m_FirmwareIdx];

	new_version = (u16) (firmware_data[52] | (firmware_data[53]<<8));
	new_minor_version = (u16) (firmware_data[56] | (firmware_data[57]<<8));
	new_reg_version = (u16) (firmware_data[60] | (firmware_data[61]<<8));


	zinitix_printk("cur version = 0x%x, new version = 0x%x\n",
		cur_version, new_version);
	if(cur_version > 0xFF)
		return true;
	if (cur_version < new_version)
			return true;
	else if (cur_version > new_version)
			return false;

	zinitix_printk("cur minor version = 0x%x, new minor version = 0x%x\n",
			cur_minor_version, new_minor_version);
	if (cur_minor_version < new_minor_version)
		return true;
	else if (cur_minor_version > new_minor_version)
		return false;

	zinitix_printk("cur reg data version = 0x%x, new reg data version = 0x%x\n",
			cur_reg_version, new_reg_version);
	if (cur_reg_version <new_reg_version)
		return true;

	return false;
}
#endif

#define TC_SECTOR_SZ		8

#if TOUCH_FORCE_UPGRADE
static void ts_check_hwid_in_fatal_state(struct bt541_ts_info *info)
{
		//u16 flash_addr;
		u8 check_data[80];
		int i;
		u16 chip_code;
		u16 hw_id0, hw_id1_1, hw_id1_2;
		int retry = 0;
retry_fatal:
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON);
		mdelay(10);
		if (write_reg(info->client, 0xc000, 0x0001) != I2C_SUCCESS){
			dev_err(&info->client->dev,"power sequence error (vendor cmd enable)\n");
			goto fail_check_hwid;
		}
		udelay(10);
		if (read_data(info->client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
			dev_err(&info->client->dev,"fail to read chip code\n");
			goto fail_check_hwid;
		}
		dev_info(&info->client->dev,"chip code = 0x%x\n", chip_code);
		udelay(10);
		if (write_cmd(info->client, 0xc004) != I2C_SUCCESS){
			dev_err(&info->client->dev,"power sequence error (intn clear)\n");
			goto fail_check_hwid;
		}
		udelay(10);
		if (write_reg(info->client, 0xc002, 0x0001) != I2C_SUCCESS){
			dev_err(&info->client->dev,"power sequence error (nvm init)\n");
			goto fail_check_hwid;
		}
		//dev_err(&info->client->dev, "init flash\n");
		mdelay(5);
		if (write_reg(info->client, 0xc003, 0x0000) != I2C_SUCCESS){
			dev_err(&info->client->dev,"fail to write nvm vpp on\n");
			goto fail_check_hwid;
		}
		if (write_reg(info->client, 0xc104, 0x0000) != I2C_SUCCESS){
			dev_err(&info->client->dev,"fail to write nvm wp disable\n");
			goto fail_check_hwid;
		}
		//dev_err(&info->client->dev, "init flash\n");
		if (write_cmd(info->client, BT541_INIT_FLASH) != I2C_SUCCESS) {
			dev_err(&info->client->dev, "failed to init flash\n");
			goto fail_check_hwid;
		}
		//dev_err(&info->client->dev, "read firmware data\n");
		for (i = 0; i < 80; i+=TC_SECTOR_SZ) {
			if (read_firmware_data(info->client,
				BT541_READ_FLASH,
				(u8*)&check_data[i], TC_SECTOR_SZ) < 0) {
				dev_err(&info->client->dev, "error : read zinitix tc firmare\n");
				goto fail_check_hwid;
			}
		}
		hw_id0 = check_data[48] + check_data[49]*256;
		hw_id1_1 = check_data[70];
		hw_id1_2 = check_data[71];

		//dev_err(&info->client->dev, "eeprom hw id = %d, %d, %d\n", hw_id0, hw_id1_1, hw_id1_2);

		if(hw_id1_1 == hw_id1_2 && hw_id0 != hw_id1_1)
			info->cap_info.hw_id = hw_id1_1;
		else
			info->cap_info.hw_id = hw_id0;

		dev_err(&info->client->dev, "hw id = %d\n", info->cap_info.hw_id);
		mdelay(5);
		return;

fail_check_hwid:
	if(retry++ <3) {
		dev_info(&info->client->dev, "fail to check hw id, retry cnt = %d\n", retry);
		mdelay(5);
		goto retry_fatal;
	}
}
#endif
//jesse add
static inline s32 write_cc02(struct i2c_client *client,	u32 addrL, u32 addrH)
{
	s32 ret;
	u8 pkt[10]; /* max packet */
	u16 length;

	// pr_info("bt541 write_cc01 \n");
	printk(KERN_ALERT "bt541:01%s",__func__);
	pkt[0] = 0x02; /* command */
	pkt[1] = 0xcc;
	pkt[2] = (addrL) & 0xff; /* addrL */
	pkt[3] = (addrL >> 8)&0xff;
	pkt[4] = (addrL >> 16)&0xff; /* addrL */
	pkt[5] = (addrL >> 24)&0xff;
	pkt[6] = (addrH) & 0xff; 	/* addrH */
	pkt[7] = (addrH >> 8)&0xff;
	pkt[8] = (addrH >>16) & 0xff; 	/* addrH */
	pkt[9] = (addrH >> 24)&0xff;
	length = 8;

//retry:
	ret = i2c_master_send(client , pkt , length + 2);
	printk(KERN_ALERT "bt541:02%s",__func__);
	if (ret < 0) {
	//	mdelay(1);

	//	if (++count < 8)
	//		goto retry;

		return ret;
	}
	printk(KERN_ALERT "bt541:03%s",__func__);
	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}
//jesse end


static bool ts_upgrade_firmware(struct bt541_ts_info *info,
	const u8 *firmware_data, u32 size)
{
	struct i2c_client *client = info->client;
	u16 flash_addr;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;
	int page_sz = 64;
	u16 chip_code;

	verify_data = kzalloc(size, GFP_KERNEL);
	if (verify_data == NULL) {
		zinitix_printk(KERN_ERR "cannot alloc verify buffer\n");
		return false;
	}

retry_upgrade:
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	mdelay(10);
//jesse add

if (write_cmd(client, 0x01f8) != I2C_SUCCESS) {
	zinitix_printk("Failed to send 01f8 command \n");
	goto fail_upgrade;
}
mdelay(10);
printk("==========01f8 end===============\n");

if (write_reg(client, 0xc011, 0x0080) != I2C_SUCCESS) {
	zinitix_printk("Failed to send c011 command\\n");
	goto fail_upgrade;
}
mdelay(10);
printk("==========c011 end===============\n");

if (write_cc02(client, 0x001f0028, 0x0002561A) < 0) {
	zinitix_printk("Failed to send 0028 command\nn");
	}
mdelay(5);
printk("==========0028 end===============\n");

if (write_cc02(client, 0x001f002c, 0x00003bd0) < 0) {
	zinitix_printk("Failed to send 002c command\nn");
	}
mdelay(5);
printk("==========002c end===============\n");

 if (write_cmd(client, 0x01d5) != I2C_SUCCESS) {
		zinitix_printk("0x01d5 error\n");
		goto fail_upgrade;
	}
	mdelay(5);
//jesse end

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		zinitix_printk("power sequence error (vendor cmd enable)\n");
		goto fail_upgrade;
	}

	udelay(10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		zinitix_printk("failed to read chip code\n");
		goto fail_upgrade;
	}

	zinitix_printk("chip code = 0x%x\n", chip_code);

	udelay(10);

	//Link modify the page size
//	if((chip_code == ZT7554_CHIP_CODE)||((chip_code == ZTW622_CHIP_CODE))||(chip_code == ZTW623_CHIP_CODE))
//		page_sz = 128;


	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		zinitix_printk("power sequence error (intn clear)\n");
		goto fail_upgrade;
	}

	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		zinitix_printk("power sequence error (nvm init)\n");
		goto fail_upgrade;
	}

	mdelay(5);

	zinitix_printk("init flash\n");

	if (write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS) {
		zinitix_printk("failed to write nvm vpp on\n");
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		zinitix_printk("failed to write nvm wp disable\n");
		goto fail_upgrade;
	}

	if (write_cmd(client, 0x01DF) != I2C_SUCCESS) {
		zinitix_printk("Mass erase fail\n");
		goto fail_upgrade;
	}
	mdelay(100);
		//#define BT541_INIT_FLASH		0x01d0
	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
			zinitix_printk("failed to init flash\n");
		goto fail_upgrade;
	}

	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			//zinitix_debug_msg("write :addr=%04x, len=%d\n",	flash_addr, TC_SECTOR_SZ);
			if (write_data(client,BT541_WRITE_FLASH,(u8 *)&firmware_data[flash_addr], TC_SECTOR_SZ) < 0) {
				zinitix_printk(KERN_INFO"error : write zinitix tc firmare\n");
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
			udelay(100);
		}

		mdelay(30);	/*for fuzing delay*/
	}

	if (write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS) {
		zinitix_printk("nvm write vpp off\n");
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		zinitix_printk("nvm wp enable\n");
		goto fail_upgrade;
	}

	zinitix_printk("init flash\n");

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		zinitix_printk("failed to init flash\n");
		goto fail_upgrade;
	}

	zinitix_printk("read firmware data\n");

	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			/*zinitix_debug_msg("read :addr=%04x, len=%d\n", flash_addr, TC_SECTOR_SZ);*/
			if (read_firmware_data(client,BT541_READ_FLASH,(u8*)&verify_data[flash_addr], TC_SECTOR_SZ) < 0) {
				dev_err(&client->dev, "Failed to read firmare\n");

				goto fail_upgrade;
			}

			flash_addr += TC_SECTOR_SZ;
		}
	}
	/* verify */
	dev_info(&client->dev, "verify firmware data\n");
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) {
		dev_info(&client->dev, "upgrade finished\n");
		kfree(verify_data);
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		return true;
	}

fail_upgrade:
	bt541_power_control(info, POWER_OFF);

	if (retry_cnt++ < INIT_RETRY_CNT) {
		dev_err(&client->dev, "upgrade failed : so retry... (%d)\n", retry_cnt);
		goto retry_upgrade;
	}

	if (verify_data != NULL)
		kfree(verify_data);

	dev_info(&client->dev, "Failed to upgrade\n");

	return false;
}

static bool init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	u16 reg_val;
	int i;
	u16 chip_eeprom_info;

#if USE_CHECKSUM
	u16 chip_check_sum;
	u8 checksum_err;
#endif
	int retry_cnt = 0;
#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
	char* productionMode = "androidboot.bsp=2";
#endif
	char* checkMode = NULL;

	dev_info(&client->dev, "init_touch: B\n");

#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
	checkMode = strstr(saved_command_line, productionMode);
#endif
retry_init:
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_EEPROM_INFO_REG,
				(u8 *)&chip_eeprom_info, 2) < 0) {
			dev_err(&client->dev, "Failed to read eeprom info(%d)\n", i);
			mdelay(10);
			continue;
		} else
			break;
	}

	if (i == INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	dev_info(&client->dev, "%s: Check checksum\n", __func__);

	checksum_err = 0;

	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_CHECKSUM_RESULT,
				(u8 *)&chip_check_sum, 2) < 0) {
			mdelay(10);
			continue;
		}

#if defined(TSP_VERBOSE_DEBUG)
		dev_info(&client->dev, "0x%04X\n", chip_check_sum);
#endif

		if(chip_check_sum == 0x55aa)
			break;
		else {
			checksum_err = 1;
			break;
		}
	}

	if (i == INIT_RETRY_CNT || checksum_err) {
		dev_err(&client->dev, "Failed to check firmware data\n");

		goto fail_init;
	}
#endif

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to write reset command\n");
		goto fail_init;
	}

#ifdef SUPPORTED_TOUCH_KEY
	cap->button_num = SUPPORTED_BUTTON_NUM;
#endif

	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
#if  SUPPORTED_PALM_TOUCH
	zinitix_bit_set(reg_val, BIT_PALM);
	//zinitix_bit_set(reg_val, BIT_PALM_REJECT);
#endif
//	zinitix_bit_set(reg_val, BIT_PT_EXIST);


	if (cap->button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);

	cap->ic_int_mask = reg_val;

	dev_info(&client->dev, "ic_int_mask=%d:\n", cap->ic_int_mask);

	if (write_reg(client, BT541_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;
#if USE_WAKEUP_GESTURE
	if (write_reg(client, 0x126, 0) != I2C_SUCCESS)
		goto fail_init;
#endif

	dev_info(&client->dev, "%s: Send reset command\n", __func__);
	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip information */
	if (read_data(client, BT541_VENDOR_ID,
					(u8 *)&cap->vendor_id, 2) < 0) {
		zinitix_printk("failed to read chip revision\n");
		goto fail_init;
	}

	if (read_data(client, BT541_CHIP_REVISION,
					(u8 *)&cap->ic_revision, 2) < 0) {
		zinitix_printk("failed to read chip revision\n");
		goto fail_init;
	}

//	cap->ic_fw_size = 44*1024;

	if (read_data(client, BT541_HW_ID, (u8 *)&cap->hw_id, 2) < 0) {
		dev_err(&client->dev, "Failed to read hw id\n");
		goto fail_init;
	}
	if (read_data(client, BT541_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;


	if (read_data(client, BT541_BUTTON_SENSITIVITY,
					(u8 *)&cap->key_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_DUMMY_BUTTON_SENSITIVITY,
					(u8 *)&cap->dummy_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_X,
					(u8 *)&cap->x_node_num, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_Y,
					(u8 *)&cap->y_node_num, 2) < 0)
		goto fail_init;

	cap->total_node_num = cap->x_node_num * cap->y_node_num;

	if (read_data(client, BT541_DND_N_COUNT,
					(u8 *)&cap->N_cnt, 2) < 0)
		goto fail_init;

	zinitix_debug_msg("N count = %d\n", cap->N_cnt);

	if (read_data(client, BT541_DND_U_COUNT,
					(u8 *)&cap->u_cnt, 2) < 0)
		goto fail_init;

	zinitix_debug_msg("U count = %d\n", cap->u_cnt);

	if (read_data(client, BT541_AFE_FREQUENCY,
					(u8 *)&cap->afe_frequency, 2) < 0)
		goto fail_init;

	zinitix_debug_msg("AFE frequency = %d\n", cap->afe_frequency);


	/* get chip firmware version */
	if (read_data(client, BT541_FIRMWARE_VERSION,
		(u8 *)&cap->fw_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_MINOR_FW_VERSION,
		(u8 *)&cap->fw_minor_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_DATA_VERSION_REG,
		(u8 *)&cap->reg_data_version, 2) < 0)
		goto fail_init;

	zinitix_printk("fw_version = 0x%x, fw_minor_version = 0x%x, reg_data_version = 0x%x\n",
			cap->fw_version, cap->fw_minor_version, cap->reg_data_version);

#if TOUCH_ONESHOT_UPGRADE
	if ((checkMode == NULL) &&(ts_check_need_upgrade(info, cap->fw_version,
			cap->fw_minor_version, cap->reg_data_version) == true) && (info->checkUMSmode == false)) {
		zinitix_printk("start upgrade firmware\n");

		if (ts_upgrade_firmware(info, m_pFirmware[m_FirmwareIdx],
			cap->ic_fw_size) == false)
			goto fail_init;

		/* disable chip interrupt */
		if (write_reg(client, BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;

		/* get chip firmware version */
		if (read_data(client, BT541_FIRMWARE_VERSION,
				(u8 *)&cap->fw_version, 2) < 0)
			goto fail_init;

		if (read_data(client, BT541_MINOR_FW_VERSION,
				(u8 *)&cap->fw_minor_version, 2) < 0)
			goto fail_init;

		if (read_data(client, BT541_DATA_VERSION_REG,
				(u8 *)&cap->reg_data_version, 2) < 0)
			goto fail_init;
	}
#endif

	if (read_data(client, BT541_EEPROM_INFO_REG,
			(u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
			(u16)pdata->x_resolution) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
			(u16)pdata->y_resolution) != I2C_SUCCESS)
		goto fail_init;

	cap->MinX = (u32)0;
	cap->MinY = (u32)0;
	cap->MaxX = (u32)pdata->x_resolution;
	cap->MaxY = (u32)pdata->y_resolution;

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
		(u16)cap->button_num) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
		(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;

	cap->multi_fingers = MAX_SUPPORTED_FINGER_NUM;

	zinitix_debug_msg("max supported finger num = %d\r\n",
		cap->multi_fingers);
	cap->gesture_support = 0;
	zinitix_debug_msg("set other configuration\r\n");

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
					TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_TOUCH_MODE, info->touch_mode) != I2C_SUCCESS)
		goto fail_init;

	if (read_data(client, ZINITIX_INTERNAL_FLAG_02,
		(u8 *)&reg_val, 2) < 0)
		goto fail_init;

	cap->i2s_checksum = 0;

	if (write_reg(client, BT541_INT_ENABLE_FLAG,
		cap->ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* Clear interrupt flag */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) { /* Test Mode */
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: Failed to set DELAY_RAW_FOR_HOST\n",
						__func__);

			goto fail_init;
		}
	}
#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL,
			SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_init;

	read_data(client, BT541_PERIODICAL_INTERRUPT_INTERVAL, (u8 *)&reg_val, 2);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "Esd timer register = %d\n", reg_val);
#endif
#endif
	/* Clear interrupt flag */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	zinitix_debug_msg("successfully initialized\r\n");
	return true;

fail_init:
	if (++retry_cnt < INIT_RETRY_CNT) {
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		zinitix_debug_msg("retry to initiallize(retry cnt = %d)\r\n",
				retry_cnt);
		goto	retry_init;

	} else if (retry_cnt == INIT_RETRY_CNT) {

	zinitix_debug_msg("retry to initiallize(retry cnt = %d)\r\n", retry_cnt);
#if TOUCH_FORCE_UPGRADE
		if(checkMode == NULL) {

			ts_check_hwid_in_fatal_state(info);
			ts_select_type_hw(info);
			if (ts_upgrade_firmware(info, m_pFirmware[m_FirmwareIdx],
				cap->ic_fw_size) == false) {
				zinitix_printk("upgrade failed\n");
				return false;
			}
		}
		else
			return true;
		mdelay(100);

		goto retry_init;
#endif
	}

	dev_err(&client->dev, "Failed to initiallize\n");

	return false;
}

bool mini_init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		dev_info(&client->dev, "Failed to write reset command\n");

		goto fail_mini_init;
	}

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
			(u16)(pdata->x_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
			(u16)(pdata->y_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	dev_info(&client->dev, "touch max x = %d\r\n", pdata->x_resolution);
	dev_info(&client->dev, "touch max y = %d\r\n", pdata->y_resolution);

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
			(u16)info->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
			(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
			TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_TOUCH_MODE,
			info->touch_mode) != I2C_SUCCESS)
		goto fail_mini_init;


	if (write_reg(client, BT541_INT_ENABLE_FLAG,
			info->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
				RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS){
			dev_err(&client->dev, "Failed to set BT541_DELAY_RAW_FOR_HOST\n");

			goto fail_mini_init;
		}
	}

#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL,
			SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_mini_init;

	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "Started esd timer\n");
#endif
#endif
	dev_err(&client->dev, "Successfully mini initialized\r\n");
	return true;

fail_mini_init:
	dev_err(&client->dev, "Failed to initialize mini init\n");
#if 0
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	if (init_touch(info) == false) {
	dev_err(&client->dev, "Failed to initialize\n");

	return false;
	}

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "Started esd timer\n");
#endif
#endif
	return true;
#else
	return false;

#endif
}


void clear_report_data(struct bt541_ts_info *info)
{
	int i;
	u8 reported = 0;
	u8 sub_status;

#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < info->cap_info.button_num; i++) {
		if (info->button[i] == ICON_BUTTON_DOWN) {
			info->button[i] = ICON_BUTTON_UP;
			input_report_key(info->input_dev, BUTTON_MAPPING_KEY[i], 0);
			reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			printk(KERN_INFO "Button up = %d\n", i);
#else
			printk(KERN_INFO "Button up\n");
#endif
		}
	}
#endif

	input_report_key(info->input_dev, BTN_TOUCH, 0);

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->reported_touch_info.coord[i].sub_status;
		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,	MT_TOOL_FINGER, 0);
			reported = true;
			if (!m_ts_debug_mode && TSP_NORMAL_EVENT_MSG)
				printk(KERN_INFO "[TSP] R %02d\r\n", i);
		}
		info->reported_touch_info.coord[i].sub_status = 0;
	}

	if (reported)
		input_sync(info->input_dev);

	info->finger_cnt1=0;
}

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

static irqreturn_t bt541_touch_work(int irq, void *data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)data;
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
#ifdef SUPPORTED_TOUCH_KEY
	u8 reported = false;
#endif
	u8 sub_status;
	u8 prev_sub_status;
	u32 x, y, maxX, maxY;
	u32 w;
	u32 tmp;
	//u8 read_result = 1;
	u8 palm = 0;
#if USE_WAKEUP_GESTURE
	u16 gesture_flag=0;
	int ret =0;
#endif

	if (down_trylock(&info->work_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n", __func__);
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

		return IRQ_HANDLED;
	}


#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif
#if USE_WAKEUP_GESTURE
		if((tpd_halt)&&(info->work_state == SUSPEND)) //suspend state support gesture
		{

			dev_err(&client->dev," gesture wakeup ");
			if (read_data(client, 0x126,(u8 *)&gesture_flag, 2) < 0) //0x126
			{
				dev_err(&client->dev," gesture read reg error!!!\r\n");

				ret=0;
				/*eric add 20160920*/
				write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
			}
			else
			{
				/*wake up*/
				/*eric add 20160920*/
				write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

				dev_err(&client->dev," gesture wakeup flag:%d\r\n",gesture_flag);
				//write_reg(client, 0x3da, gesture_flag);
				//if(gesture_flag==2)/*1 double click*/
				if(gesture_flag==1)/*1 double click*/
				{
					dev_err(&client->dev,"[bt541_touch_work]USE_WAKEUP_GESTURE---powerkey__start\n");
					input_report_key(info->input_dev, KEY_POWER, 1);
					input_sync(info->input_dev);
					input_report_key(info->input_dev, KEY_POWER, 0);
					input_sync(info->input_dev);
					dev_err(&client->dev,"[bt541_touch_work]USE_WAKEUP_GESTURE---powerkey__end\n");

					if (write_reg(client, 0x126, 0) != 0)
					{
						dev_err(&client->dev," gesture write reg error!!!\r\n");

						ret=0;
					}

				}

			}
			goto out;
		}

#endif


	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied\n", __func__);
		udelay(DELAY_FOR_SIGNAL_DELAY);
		 write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		goto out;
	}

	info->work_state = NORMAL;


	if (ts_read_coord(info) == false )
	{
		dev_err(&client->dev, "couldn't read touch_dev coord. read fail\r\n");

		bt541_power_control(info, POWER_ON_SEQUENCE);
		mini_init_touch(info);
		goto out;
	}

	if(write_cmd(client, BT541_CLEAR_INT_STATUS_CMD)!=0)
		dev_err(&client->dev, "BT541_CLEAR_INT_STATUS_CMD error 11\n");

	/* invalid : maybe periodical repeated int. */
	if (info->touch_info.status == 0x0 || info->touch_info.status == 0xffff)
	{
		dev_err(&client->dev, "periodical interrupt\r\n");

		goto out;
	}

#ifdef SUPPORTED_TOUCH_KEY
	reported = false;

	if (zinitix_bit_test(info->touch_info.status, BIT_ICON_EVENT)) {
		if (read_data(info->client, BT541_ICON_STATUS_REG,
			(u8 *)(&info->icon_event_reg), 2) < 0) {
			dev_err(&client->dev, "Failed to read button info\n");
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

			goto out;
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg,
						(BIT_O_ICON0_DOWN + i))) {
				info->button[i] = ICON_BUTTON_DOWN;
				input_report_key(info->input_dev, BUTTON_MAPPING_KEY[i], 1);
				reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				dev_info(&client->dev, "Button down = %d\n", i);
#else
				dev_info(&client->dev, "Button down\n");
#endif
			}
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg,
						(BIT_O_ICON0_UP + i))) {
				info->button[i] = ICON_BUTTON_UP;
				input_report_key(info->input_dev, BUTTON_MAPPING_KEY[i], 0);
				reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				dev_info(&client->dev, "Button up = %d\n", i);
#else
				dev_info(&client->dev, "Button up\n");
#endif
			}
		}
	}

	/* if button press or up event occured... */
	if (reported == true ||
			!zinitix_bit_test(info->touch_info.status, BIT_PT_EXIST)) {
		for (i = 0; i < info->cap_info.multi_fingers; i++) {
			prev_sub_status = info->reported_touch_info.coord[i].sub_status;
			if (zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
				//dev_info(&client->dev, "Finger [%02d] up ver0x%02x hw0x%02x mode0x%02x\n",
				//	i, info->cap_info.reg_data_version,
				//	info->cap_info.hw_id, m_optional_mode);
				info->finger_cnt1--;
				if (info->finger_cnt1 == 0)
					input_report_key(info->input_dev, BTN_TOUCH, 0);

				input_mt_slot(info->input_dev, i);
				input_mt_report_slot_state(info->input_dev,
							MT_TOOL_FINGER, 0);
			}
		}
		memset(&info->reported_touch_info, 0x0, sizeof(struct point_info));
		input_sync(info->input_dev);

		if (reported == true) /* for button event */
			udelay(100);

		goto out;
	}
#endif


#if SUPPORTED_PALM_TOUCH
	if (zinitix_bit_test(info->touch_info.status, BIT_PALM)) {
	/*Link modified here on 20190418 palm down sleep,no need detect the up event
		if(zinitix_bit_test(info->touch_info.status, BIT_UP)) {
			dev_err(&client->dev, "large touch palm enter\n");
			input_report_key(info->input_dev, KEY_POWER, 1);
			input_sync(info->input_dev);
			input_report_key(info->input_dev, KEY_POWER, 0);
			input_sync(info->input_dev);
			palm = 1;
			goto out;
		}*/
/*
			input_report_key(info->input_dev, KEY_LARGE, 1);
			input_sync(info->input_dev);
			input_report_key(info->input_dev, KEY_LARGE, 0);
			input_sync(info->input_dev);
*/
			dev_err(&client->dev, "large touch palm enter\n");
			input_report_key(info->input_dev, KEY_POWER, 1);
			input_sync(info->input_dev);
			input_report_key(info->input_dev, KEY_POWER, 0);
			input_sync(info->input_dev);
			palm = 1;
			goto out;
	}
#if 0
	if (zinitix_bit_test(info->touch_info.status, BIT_PALM_REJECT)) {
		palm = 2;
		dev_err(&client->dev, "large touch palm reject\n");
	}
#endif
#endif

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->touch_info.coord[i].sub_status;
		prev_sub_status = info->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = info->touch_info.coord[i].x;
			y = info->touch_info.coord[i].y;
			w = info->touch_info.coord[i].width;

			 /* transformation from touch to screen orientation */
			if (pdata->orientation & TOUCH_V_FLIP)
				y = info->cap_info.MaxY
					+ info->cap_info.MinY - y;

			if (pdata->orientation & TOUCH_H_FLIP)
				x = info->cap_info.MaxX
					+ info->cap_info.MinX - x;

			maxX = info->cap_info.MaxX;
			maxY = info->cap_info.MaxY;

			if (pdata->orientation & TOUCH_XY_SWAP) {
				zinitix_swap_v(x, y, tmp);
				zinitix_swap_v(maxX, maxY, tmp);
			}

			if (x > maxX || y > maxY) {
				dev_err(&client->dev,
							"Invalid coord %d : x=%d, y=%d\n", i, x, y);
				continue;
			}

			info->touch_info.coord[i].x = x;
			info->touch_info.coord[i].y = y;

			//dev_err(&client->dev, "Finger [%02d] x = %d, y = %d,"
			//						" w = %d, p = %d\n", i, x, y, w, palm);

			if (zinitix_bit_test(sub_status, SUB_BIT_DOWN)){

				dev_info(&client->dev, "Finger [%02d] down\n", i);

				info->finger_cnt1++;
			}

			if (w == 0)
				w = 1;

			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1);

#if (TOUCH_POINT_MODE == 2)
			if (palm == 0) {
				if (w >= PALM_REPORT_WIDTH)
					w = PALM_REPORT_WIDTH - 10;
			} else if (palm == 1) {	//palm report
				w = PALM_REPORT_WIDTH;
//				info->touch_info.coord[i].minor_width = PALM_REPORT_WIDTH;
			} else if (palm == 2){	// palm reject
//				x = y = 0;
				w = PALM_REJECT_WIDTH;
//				info->touch_info.coord[i].minor_width = PALM_REJECT_WIDTH;
			}
#endif

			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, (u32)w);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, (u32)w);
			input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR,
					(u32)((palm == 1) ? w-40 : w));
#if (TOUCH_POINT_MODE == 2)
			input_report_abs(info->input_dev,
				ABS_MT_TOUCH_MINOR, (u32)info->touch_info.coord[i].minor_width);
#endif

			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_key(info->input_dev, BTN_TOUCH, 1);
		} else if (zinitix_bit_test(sub_status, SUB_BIT_UP) ||
			zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
			dev_info(&client->dev, "Finger [%02d] up ver0x%02x hw0x%02x mode0x%02x\n",
				i, info->cap_info.reg_data_version,
				info->cap_info.hw_id, m_optional_mode);
			info->finger_cnt1--;
			if (info->finger_cnt1 == 0)
				input_report_key(info->input_dev, BTN_TOUCH, 0);

			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

		} else {
			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
		}
	}
	memcpy((char *)&info->reported_touch_info, (char *)&info->touch_info,
		sizeof(struct point_info));
	input_sync(info->input_dev);

out:

	if (info->work_state == NORMAL) {
#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);

#endif
		info->work_state = NOTHING;
	}

	up(&info->work_lock);

	return IRQ_HANDLED;
}

#if 0
static int bt541_pinctrl_configure(struct bt541_ts_info *info, bool active)
{
	struct i2c_client *client = info->client;
	struct pinctrl_state *set_state_tsp;

	int retval;

	dev_info(&client->dev,"%s: %s\n", __func__, active ? "ACTIVE" : "SUSPEND");

	if (active) {
		set_state_tsp =
			pinctrl_lookup_state(info->pinctrl,
						"tsp_en_gpio_active");
		if (IS_ERR(set_state_tsp)) {
			dev_info(&client->dev,"%s: cannot get pinctrl(tsp_en_gpio) active state\n", __func__);
			return PTR_ERR(set_state_tsp);
		}

	} else {
		set_state_tsp =
			pinctrl_lookup_state(info->pinctrl,
						"tsp_en_gpio_suspend");
		if (IS_ERR(set_state_tsp)) {
			dev_info(&client->dev,"%s: cannot get pinctrl(tsp_en_gpio) sleep state\n", __func__);
			return PTR_ERR(set_state_tsp);
		}
	}

	retval = pinctrl_select_state(info->pinctrl, set_state_tsp);
	if (retval) {
		dev_info(&client->dev,"%s: cannot set pinctrl %s state\n",
				__func__, active ? "active" : "suspend");
		return retval;
	}

	return 0;
}
#endif

int zinitix_ts_mmi_gesture_suspend(struct device *dev)
{
	int i = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	down(&info->work_lock);
	//reset gesture register
	if (write_reg(client, 0x126, 0) != I2C_SUCCESS) {
		dev_err(dev, "%s: Reset gesture register error.\n", __func__);
		up(&info->work_lock);
		return -1;
	}
	//enable irq wake
	enable_irq_wake(info->irq);
	info->irq_enabled = true;
	//send enable easy wake gesture cmd
	if (write_reg(client, 0x011D, 0x0089) != I2C_SUCCESS) {
		zinitix_printk("easy_wakeup_gesture write_reg[0x0089] failed!!!-------------\n");
		up(&info->work_lock);
		return -1;
	}
	//clear INT status
	for(i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) < 0) {
			dev_err(&client->dev,"tpd_suspend fail to clear int(%d)\r\n", i);
			mdelay(10);
			continue;
		} else
		break;
	}
	if (i == 3) {
		up(&info->work_lock);
		return -1;
	}
	//send idle cmd
	for(i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_IDLE_CMD) < 0) {
			dev_err(&client->dev,"tpd_suspend fail to send sleep cmd(%d)\r\n", i);
			mdelay(10);
			continue;
			} else
			break;
	}
	if (i == 3) {
		up(&info->work_lock);
		return -1;
	}
	up(&info->work_lock);

	return 0;
}

int zinitix_ts_mmi_gesture_resume(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	down(&info->work_lock);
	//disable irq wake
	disable_irq_wake(info->irq);

	//reset touch IC when resume
	zinitix_hw_reset(info,true);
	mdelay(CHIP_ON_DELAY);
	bt541_power_sequence(info);
	err = mini_init_touch(info);
	if(err < 0){
		zinitix_printk("resume_reset :  zinitix_resume_proc error\n");
		up(&info->work_lock);
		return -1;
	}
	up(&info->work_lock);

	return 0;
}

#if !defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
static int bt541_ts_resume(struct device *dev)
{
	int i = 0;
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	zinitix_printk("[TSP]: %s\n", __func__);

	dev_err(&client->dev, "bt541_ts_resume start \n");

	if(info->device_enabled) {
		dev_err(&client->dev, "%s: already enabled\n", __func__);
		return 0;
	}
	info->device_enabled = 1;

	down(&info->work_lock);
	if (info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: Invalid work proceedure (%d)\n",
				__func__, info->work_state);
		up(&info->work_lock);

		return 0;
	}
	info->work_state = RESUME;


#if 0
	bt541_power_control(info, POWER_ON_SEQUENCE);

	if (mini_init_touch(info) == false)
		dev_err(&client->dev, "Failed to resume\n");
#else

	for(i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_WAKEUP_CMD) < 0) {
			zinitix_printk("tpd_resume fail to send wakeup_cmd(%d)\r\n", i);
			mdelay(10);
			continue;
		} else
			break;
	}

	 if (i == 3) {
		bt541_power_control(info, POWER_ON_SEQUENCE);
		err = mini_init_touch(info);
		if(err < 0){
			zinitix_printk("resume_reset :  zinitix_resume_proc error\n");
		}
		goto reset_exit;
	}

		write_cmd(client ,BT541_SWRESET_CMD);
		msleep(20);
		for(i = 0; i < 3; i++) {
			if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) < 0) {
				zinitix_printk("tpd_resume fail to send wakeup_cmd(%d)\r\n", i);
				mdelay(10);
				continue;
			} else
				break;
		}


	#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);
	#endif

reset_exit:
#endif

	info->work_state = NOTHING;

#if !USE_WAKEUP_GESTURE/*eric add 20170208*/
	enable_irq(info->irq);
	info->irq_enabled = true;
#endif

	if (easy_wake) {
		if (write_reg(client, 0x011D, 0x0089) != I2C_SUCCESS)
			zinitix_printk("[resume]easy_wakeup_gesture write_reg[0x0089] failed!!!-----------------------\n");
	}
	else {
		if (write_reg(client, 0x011D, 0x0080) != I2C_SUCCESS)
			zinitix_printk("[resume]easy_wakeup_gesture write_reg[0x0080] failed!!!-----------------------\n");
	}

	if (big_object) {
		if (write_reg(client, 0x0017, 0x8A08) != I2C_SUCCESS)
			zinitix_printk("[resume]bigobject_off write_reg[0x8A08] failed!!!-----------------------\n");
	}
	else {
		if (write_reg(client, 0x0017, 0x8A00) != I2C_SUCCESS)
			zinitix_printk("[resume]bigobject_off write_reg[0x8A00] failed!!!-----------------------\n");
	}

	up(&info->work_lock);
	dev_err(&client->dev, "bt541_ts_resume end \n");

	tpd_halt = 0;

	return 0;
}

static int bt541_ts_suspend(struct device *dev)
{
	int i =0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	zinitix_printk("[TSP]: %s\n", __func__);
	tpd_halt = 1;

	dev_err(&client->dev, "bt541_ts_suspend 11\n");

#if !USE_WAKEUP_GESTURE/*eric add 20170208*/
	disable_irq(info->irq);
	info->irq_enabled = false;
#else
	disable_irq_wake(info->irq);
	enable_irq_wake(info->irq);
	info->irq_enabled = true;
#endif

	if(!info->device_enabled) {
		dev_err(&client->dev, "%s: already disabled\n", __func__);
		return 0;
	}
	info->device_enabled = 0;

#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
#endif

	down(&info->work_lock);
	if (info->work_state != NOTHING
		&& info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: Invalid work proceedure (%d)\n",
			__func__, info->work_state);
		up(&info->work_lock);
	#if !USE_WAKEUP_GESTURE/*eric add 20170208*/
		enable_irq(info->irq);
		info->irq_enabled = true;
	#endif
		return 0;
	}

	clear_report_data(info);

#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif

#if 0
	write_cmd(info->client, BT541_IDLE_CMD);
	bt541_power_control(info, POWER_OFF);

	gpio_direction_output(info->pdata->gpio_reset, 0);//?????????
#else
/*eric start */
	//write_cmd(client, 0x000a);
		//mdelay(10);

	for(i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) < 0) {
			dev_err(&client->dev,"tpd_suspend fail to clear int(%d)\r\n", i);
			mdelay(10);
			continue;
		} else
		break;
	}

	for(i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_IDLE_CMD) < 0) {
			dev_err(&client->dev,"tpd_suspend fail to send sleep cmd(%d)\r\n", i);
			mdelay(10);
			continue;
			} else
			break;
		}
	//write_cmd(client, 0x000b);
	//mdelay(10);
#endif

	info->work_state = SUSPEND;


	up(&info->work_lock);

	dev_err(&client->dev, "bt541_ts_suspend end \n");

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			misc_touch_dev && misc_touch_dev->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			bt541_ts_resume(&misc_touch_dev->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			bt541_ts_suspend(&misc_touch_dev->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void zinitix_late_resume(struct early_suspend *h)
{
	dev_err(&client->dev, "zinitix_late_resume\n");

	bt541_ts_resume(&misc_touch_dev->client->dev);
	dev_err(&client->dev, "zinitix_late_resume end\n");
}


static void zinitix_early_suspend(struct early_suspend *h)
{
	dev_err(&client->dev, "zinitix_early_suspend\n");
	bt541_ts_suspend(&misc_touch_dev->client->dev);
	dev_err(&client->dev, "zinitix_early_suspend end\n");
}


#endif	/* CONFIG_FB */
#endif  /* TOUCHSCREEN_MMI */

static bool ts_set_touchmode(u16 value)
{
	int i;

	disable_irq(misc_info->irq);
	misc_info->irq_enabled = false;

	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_info->work_state);
		enable_irq(misc_info->irq);
		misc_info->irq_enabled = true;
		up(&misc_info->work_lock);
		return -1;
	}

	misc_info->work_state = SET_MODE;

	if (value == TOUCH_DND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
			SEC_DND_N_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_N_COUNT %d.\n", SEC_DND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
			SEC_DND_U_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_U_COUNT %d.\n", SEC_DND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
			SEC_DND_FREQUENCY) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_AFE_FREQUENCY %d.\n", SEC_DND_FREQUENCY);
	}
	if (value == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
			SEC_PDND_N_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_N_COUNT %d.\n", SEC_PDND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
			SEC_PDND_U_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_U_COUNT %d.\n", SEC_PDND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
			SEC_PDND_FREQUENCY) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_AFE_FREQUENCY %d.\n", SEC_PDND_FREQUENCY);
	}
	else if(misc_info->touch_mode == TOUCH_DND_MODE || misc_info->touch_mode == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
			misc_info->cap_info.N_cnt) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to reset BT541_AFE_FREQUENCY %d.\n",
				misc_info->cap_info.N_cnt);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
			misc_info->cap_info.u_cnt) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to reset BT541_DND_U_COUNT %d.\n",
				misc_info->cap_info.u_cnt);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
			misc_info->cap_info.afe_frequency) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to reset BT541_AFE_FREQUENCY %d.\n",
				misc_info->cap_info.afe_frequency);
	}

	if (value == TOUCH_SEC_MODE)
		misc_info->touch_mode = TOUCH_POINT_MODE;
	else
		misc_info->touch_mode = value;

	printk(KERN_INFO "[zinitix_touch] tsp_set_testmode, "
		"touchkey_testmode = %d\r\n", misc_info->touch_mode);

	if (misc_info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(misc_info->client, BT541_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			zinitix_printk("Fail to set BT541_DELAY_RAW_FOR_HOST.\r\n");
	}

	if (write_reg(misc_info->client, BT541_TOUCH_MODE,
			misc_info->touch_mode) != I2C_SUCCESS)
		printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to set ZINITX_TOUCH_MODE %d.\r\n", misc_info->touch_mode);

	/* clear garbage data */
	for (i = 0; i < 10; i++) {
		mdelay(20);
		write_cmd(misc_info->client, BT541_CLEAR_INT_STATUS_CMD);
	}

	misc_info->work_state = NOTHING;
	enable_irq(misc_info->irq);
	misc_info->irq_enabled = true;
	up(&misc_info->work_lock);
	return 1;
}

int ts_upgrade_sequence(const u8 *firmware_data)
{
	disable_irq(misc_info->irq);
	misc_info->irq_enabled = false;
	down(&misc_info->work_lock);
	misc_info->work_state = UPGRADE;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	zinitix_debug_msg("clear all reported points\r\n");
	clear_report_data(misc_info);

	printk(KERN_INFO "start upgrade firmware\n");
	if (ts_upgrade_firmware(misc_info,
		firmware_data,
		misc_info->cap_info.ic_fw_size) == false) {
		enable_irq(misc_info->irq);
		misc_info->irq_enabled = true;
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;
	}

	if (init_touch(misc_info) == false) {
		enable_irq(misc_info->irq);
		misc_info->irq_enabled = true;
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;
	}

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&misc_info->client->dev, "Started esd timer\n");
#endif
#endif

	enable_irq(misc_info->irq);
	misc_info->irq_enabled = true;
	misc_info->work_state = NOTHING;
	up(&misc_info->work_lock);
	return 0;
}

#ifdef SEC_FACTORY_TEST
static inline void set_cmd_result(struct bt541_ts_info *info, char *buff, int len)
{
	strncat(info->factory_info->cmd_result, buff, len);
}

static inline void set_default_result(struct bt541_ts_info *info)
{
	char delim = ':';
	memset(info->factory_info->cmd_result, 0x00, ARRAY_SIZE(info->factory_info->cmd_result));
	memcpy(info->factory_info->cmd_result, info->factory_info->cmd, strlen(info->factory_info->cmd));
	strncat(info->factory_info->cmd_result, &delim, 1);
}

static int zinitix_read_file_request_firmware(char *file_name, u8 **file_buf)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	char fwname[FILE_NAME_LENGTH] = { 0 };

	snprintf(fwname, FILE_NAME_LENGTH, "%s", file_name);
	ret = request_firmware(&fw, fwname, &misc_touch_dev->client->dev);
	if (0 == ret) {
		zinitix_printk("firmware(%s) request successfully\n", fwname);
		*file_buf = vmalloc(fw->size);
		if (NULL == *file_buf) {
			zinitix_printk("fw buffer vmalloc fail\n");
			ret = -ENOMEM;
		} else {
			memcpy(*file_buf, fw->data, fw->size);
			ret = fw->size;
		}
	} else {
		zinitix_printk("firmware(%s) request fail,ret=%d\n", fwname, ret);
		ret = -EIO;
	}

	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}

	return ret;
}

int zinitix_read_file(char *file_name, u8 **file_buf)
{
	int ret = 0;
	int count = 0;

retry:
	ret = zinitix_read_file_request_firmware(file_name, file_buf);
	if (ret < 0) {
		zinitix_printk("get fw file(default) fail: count = %d\n", count);
		if (++count < 3)
			goto retry;
	}

	return ret;
}

#define MAX_FW_PATH 255
#define TSP_FW_FILENAME "zinitix_fw.bin"

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
static void fw_update(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct i2c_client *client = info->client;
	int ret = 0;
	char result[16] = {0};
	u8 *fw_file_buf = NULL;

	set_default_result(info);

	switch (info->factory_info->cmd_param[0]) {
	case BUILT_IN:
		ts_select_type_hw(info);
		ret = ts_upgrade_sequence((u8*)m_pFirmware[m_FirmwareIdx]);
		if(ret<0) {
			info->factory_info->cmd_state = 3;
			return;
		}
		break;

	case UMS:
		ret= zinitix_read_file(TSP_FW_FILENAME, &fw_file_buf);
		if (ret != info->cap_info.ic_fw_size) {
			dev_err(&client->dev, "invalid fw size!!\n");
			goto error_upgrade;
		}

		dev_info(&client->dev, "ums fw is loaded!!\n");
		info->checkUMSmode = true;
		ret = ts_upgrade_sequence((u8 *)fw_file_buf);
		info->checkUMSmode = false;
		if(ret<0) {
			dev_err(&client->dev, "fw upgrade fail!!\n");
			goto error_upgrade;
		}
		break;

	default:
		dev_err(&client->dev, "invalid fw file type!!\n");
		goto error_upgrade;
	}

	if (fw_file_buf) {
		vfree(fw_file_buf);
		fw_file_buf = NULL;
	}
	info->factory_info->cmd_state = 2;
	snprintf(result, sizeof(result) , "%s", "OK");
	set_cmd_result(info, result,
			strnlen(result, sizeof(result)));
	return;

error_upgrade:
	if (fw_file_buf) {
		vfree(fw_file_buf);
		fw_file_buf = NULL;
	}
	info->factory_info->cmd_state = 3;
	snprintf(result, sizeof(result) , "%s", "NG");
	set_cmd_result(info, result, strnlen(result, sizeof(result)));

	return;
}
#else
static void fw_update(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct i2c_client *client = info->client;
	int ret = 0;
	const u8 *buff = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	mm_segment_t old_fs = {0};
#endif
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	char fw_path[MAX_FW_PATH + 1];
	char result[16] = {0};

	set_default_result(info);

	switch (info->factory_info->cmd_param[0]) {
	case BUILT_IN:
		ts_select_type_hw(info);
		ret = ts_upgrade_sequence((u8*)m_pFirmware[m_FirmwareIdx]);
		if(ret<0) {
			info->factory_info->cmd_state = 3;
			return;
		}
		break;

	case UMS:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
		old_fs = get_fs();
		//set_fs(get_ds());
		set_fs(KERNEL_DS);
#endif
		snprintf(fw_path, MAX_FW_PATH, "/vendor/firmware/%s", TSP_FW_FILENAME);
		fp = filp_open(fw_path, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			dev_err(&client->dev,
				"file %s open \n", fw_path);
			info->factory_info->cmd_state = 3;
			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		if (fsize != info->cap_info.ic_fw_size) {
			dev_err(&client->dev, "invalid fw size!!\n");
			info->factory_info->cmd_state = 3;
			goto err_open;
		}

		buff = kzalloc((size_t)fsize, GFP_KERNEL);
		if (!buff) {
			dev_err(&client->dev, "failed to alloc buffer for fw\n");
			info->factory_info->cmd_state = 3;
			goto err_alloc;
		}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
		nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
#else
		nread = kernel_read(fp, (char __user *)buff, fsize, &fp->f_pos);
#endif
		if (nread != fsize) {
			info->factory_info->cmd_state = 3;
			goto err_fw_size;
		}

		filp_close(fp, current->files);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
		set_fs(old_fs);
#endif
		dev_info(&client->dev, "ums fw is loaded!!\n");
		info->checkUMSmode = true;
		ret = ts_upgrade_sequence((u8 *)buff);
		info->checkUMSmode = false;
		if(ret<0) {
			kfree(buff);
			info->factory_info->cmd_state = 3;
			return;
		}
		break;

	default:
		dev_err(&client->dev, "invalid fw file type!!\n");
		goto not_support;
	}

	info->factory_info->cmd_state = 2;
	snprintf(result, sizeof(result) , "%s", "OK");
	set_cmd_result(info, result,
			strnlen(result, sizeof(result)));
	if (buff) {
		kfree(buff);
		buff = NULL;
	}
	return;

if (fp != NULL) {
err_fw_size:
	kfree(buff);
err_alloc:
	filp_close(fp, NULL);
err_open:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	set_fs(old_fs);
#else
	;
#endif
}
not_support:
	snprintf(result, sizeof(result) , "%s", "NG");
	set_cmd_result(info, result, strnlen(result, sizeof(result)));
	return;
}
#endif

static void get_fw_ver_bin(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;
	u8 *firmware_data;

	set_default_result(info);

	/* To Do */
	/* modify m_firmware_data */
	ts_select_type_hw(info);
	firmware_data = (u8*)m_pFirmware[m_FirmwareIdx];

	fw_version = (u16)(firmware_data[52] | (firmware_data[53] << 8));
	fw_minor_version = (u16)(firmware_data[56] | (firmware_data[57] << 8));
	reg_version = (u16)(firmware_data[60] | (firmware_data[61] << 8));
	hw_id = (u16)(firmware_data[0x7528] | (firmware_data[0x7529] << 8));
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf ) << 12)
				| ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "ZI%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_fw_ver_ic(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;

	set_default_result(info);

	fw_version = info->cap_info.fw_version;
	fw_minor_version = info->cap_info.fw_minor_version;
	reg_version = info->cap_info.reg_data_version;
	hw_id = info->cap_info.hw_id;
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf) << 12)
		| ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "ZI%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_threshold(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%d", info->cap_info.threshold);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void module_off_master(void *device_data)
{
	return;
}

static void module_on_master(void *device_data)
{
	return;
}

static void module_off_slave(void *device_data)
{
	return;
}

static void module_on_slave(void *device_data)
{
	return;
}

static void get_module_vendor(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *fdata = info->factory_info;
	char buff[16] = {0};
	int val,val2;

	set_default_result(info);
	if (!(gpio_get_value(info->pdata->tsp_en_gpio)) ) {
		dev_err(&client->dev, "%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		fdata->cmd_state = NOT_APPLICABLE;
		return;
	}
	if (info->pdata->tsp_vendor1 > 0 && info->pdata->tsp_vendor2 > 0 ) {
		val = gpio_get_value(info->pdata->tsp_vendor1);
		val2 = gpio_get_value(info->pdata->tsp_vendor2);
		dev_info(&info->client->dev,
			"%s: TSP_ID: %d[%d]%d[%d]\n", __func__,
			info->pdata->tsp_vendor1, val,info->pdata->tsp_vendor2, val2);
		snprintf(buff, sizeof(buff), "%s,%d%d", tostring(OK), val,val2);
		fdata->cmd_state = OK;
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		return;
	}
	snprintf(buff, sizeof(buff),  "%s", tostring(NG));
	fdata->cmd_state = FAIL;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
}


#define BT541_VENDOR_NAME "ZINITIX"

static void get_chip_vendor(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%s", BT541_VENDOR_NAME);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_config_ver(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
        const char *model_name = info->pdata->pname;

	set_default_result(info);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),"%s_ZI_%s", model_name, CONFIG_DATE);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

#define BT541_CHIP_NAME "BT541"

static void get_chip_name(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", BT541_CHIP_NAME);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_x_num(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%u", info->cap_info.x_node_num);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_y_num(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%u", info->cap_info.y_node_num);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void not_support_cmd(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	sprintf(finfo->cmd_buff, "%s", "NA");
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);
	info->factory_info->cmd_state = WAITING;

	//dev_info(&client->dev, "%s: \"%s(%d)\"\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_reference(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->ref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_preference_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	//u16 min, max;
	s32 i;

   s16 min[64] = {163,905,2592,2982,3037,2707,1498,397,
911,2772,3006,2830,2733,2960,3189,1243,
2531,2750,3009,2824,2692,2911,3387,2790,
2839,2758,3016,2827,2687,2907,3454,3203,
2939,2878,3143,2960,2692,2911,3508,3413,
2492,2863,3134,2962,2682,2911,3495,3241,
594,3116,3205,3056,2682,2954,3352,928,
121,797,3163,3474,2842,2420,874,119,
		  };
   s16 max[64] = {303,1681,4814,5538,5639,5027,2782,737,
1691,5148,5582,5256,5075,5498,5923,2309,
4701,5106,5587,5244,5000,5405,6291,5182,
5272,5122,5602,5249,4989,5399,6414,5949,
5457,5346,5837,5496,5000,5407,6516,6339,
4628,5317,5820,5502,4982,5405,6491,6019,
1104,5786,5953,5676,4980,5486,6226,1724,
225,1479,5873,6452,5278,4494,1624,221,
	  };

	set_default_result(info);

	ts_set_touchmode(TOUCH_PDND_MODE);
	get_raw_data(info, (u8 *)raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	printk("==========raw_data start===============\n");
	for(i = 0; i < (info->cap_info.x_node_num * info->cap_info.y_node_num); i++)
	{
		printk("raw_data[%d] = %d\n",i,raw_data->pref_data[i]);
	}
	printk("==========raw_data end===============\n");

	for(i = 0; i < (info->cap_info.x_node_num * info->cap_info.y_node_num); i++)
	{
		if(((raw_data->pref_data[i]) < min[i]) || ((raw_data->pref_data[i]) > max[i]))
		{
			printk("fail raw_data[%d] = %d\n",i,raw_data->pref_data[i]);
			snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
			set_cmd_result(info, finfo->cmd_buff,
					strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
			finfo->cmd_state = FAIL;
				return;
		}else
		{
			continue;

		}

	}
	printk("Pass raw_data\n");
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "Pass");
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	return;


	/*
	min = 0xFFFF;
	max = 0x0000;

	for(i = 0; i < info->cap_info.x_node_num; i++)
	{
		for(j = 0; j < info->cap_info.y_node_num; j++)
		{
			pr_info("pref_data : %d ",
					raw_data->pref_data[i * info->cap_info.y_node_num + j]);*/
			/*
			if (raw_data->pref_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->pref_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->pref_data[i * info->cap_info.y_node_num + j];

			if (raw_data->pref_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->pref_data[i * info->cap_info.y_node_num + j];

		}
		//pr_info("\n");
	}
	*//*
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: \"%s\"(%d)\n", __func__, finfo->cmd_buff,
	//	strlen(finfo->cmd_buff));

	return;*/
}

static void run_short_data_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;

	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	const u16 NORMAL_SHORT_VALUE=1000;
	u16 i;

	set_default_result(info);

	ts_set_touchmode(TOUCH_CHECK_SHORT_MODE);
	get_short_data(info, (u8 *)raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	for(i = 0; i < (CHECK_SHORT_DATA_NUM>>1); i++)
	{
		if (raw_data->pref_data[i]!=NORMAL_SHORT_VALUE){
			pr_info("Invalid Check Short Value,Test Short!\n");
			finfo->cmd_state = FAIL;
			return;
		}

	}

	finfo->cmd_state = OK;
	return;
}

static void get_preference(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->pref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_delta_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_DELTA_MODE);
	get_raw_data(info, (u8 *)(u8 *)raw_data->delta_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);
	finfo->cmd_state = OK;

	min = (s16)0x7FFF;
	max = (s16)0x8000;

	for(i = 0; i < info->cap_info.x_node_num; i++)
	{
		for(j = 0; j < info->cap_info.y_node_num; j++)
		{
			/*printk("delta_data : %d \n", raw_data->delta_data[j+i]);*/

			if (raw_data->delta_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->delta_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->delta_data[i * info->cap_info.y_node_num + j];

			if (raw_data->delta_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->delta_data[i * info->cap_info.y_node_num + j];

		}
		/*printk("\n");*/
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: \"%s\"(%d)\n", __func__, finfo->cmd_buff,
	//	strlen(finfo->cmd_buff));

	return;
}

static void get_delta(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->delta_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	info->factory_info->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

/*
static void run_intensity_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;

	set_default_result(info);

	ts_set_touchmode(TOUCH_DND_MODE);
	get_raw_data(info, (u8 *)info->dnd_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	//////test////////////////////////////////////////////////////
	int i,j;

	for(i=0; i<30; i++)
	{
		for (j = 0; j < 18; j++)
			printk("[TSP] info->dnd_data : %d ", info->dnd_data[j+i]);

		printk("\n");
	}
	//////test////////////////////////////////////////////////////

	info->factory_info->cmd_state = 2;
}

static void get_normal(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	char buff[16] = {0};
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = info->factory_info->cmd_param[0];
	y_node = info->factory_info->cmd_param[1];

	if (x_node < 0 || x_node > info->cap_info.x_node_num ||
		y_node < 0 || y_node > info->cap_info.y_node_num) {
		snprintf(buff, sizeof(buff), "%s", "abnormal");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->factory_info->cmd_state = 3;
		return;
	}

	node_num = x_node*info->cap_info.x_node_num + y_node;

	val = info->normal_data[node_num];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->factory_info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_tkey_delta(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	char buff[16] = {0};
	u16 val;
	int btn_node;
	int ret;

	set_default_result(info);

	btn_node = info->factory_info->cmd_param[0];

	if (btn_node < 0 || btn_node > MAX_SUPPORTED_BUTTON_NUM)
		goto err_out;

	disable_irq(misc_info->irq);
	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_info->work_state);
		enable_irq(misc_info->irq);
		up(&misc_info->work_lock);
		goto err_out;
	}
	misc_info->work_state = SET_MODE;

	ret = read_data(misc_info->client, BT541_BTN_WIDTH + btn_node, (u8*)&val, 2);

	if (ret < 0) {
		printk(KERN_INFO "read error..\n");
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		goto err_out;
	}
	misc_info->work_state = NOTHING;
	enable_irq(misc_info->irq);
	up(&misc_info->work_lock);

	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->factory_info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;

err_out:
	snprintf(buff, sizeof(buff), "%s", "abnormal");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->factory_info->cmd_state = 3;
}

static void get_intensity(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	char buff[16] = {0};
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = info->factory_info->cmd_param[0];
	y_node = info->factory_info->cmd_param[1];

	if (x_node < 0 || x_node > info->cap_info.x_node_num ||
		y_node < 0 || y_node > info->cap_info.y_node_num) {
		snprintf(buff, sizeof(buff), "%s", "abnormal");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->factory_info->cmd_state = 3;
		return;
	}

	node_num = x_node*info->cap_info.x_node_num + y_node;

	val = info->dnd_data[node_num];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->factory_info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void run_normal_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;

	set_default_result(info);

	ts_set_touchmode(TOUCH_NORMAL_MODE);
	get_raw_data(info, (u8 *)info->normal_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	info->factory_info->cmd_state = 2;
}

static void get_key_threshold(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	int ret = 0;
	u16 threshold;
	char buff[16] = {0};

	set_default_result(info);

	ret = read_data(misc_info->client, BT541_BUTTON_SENSITIVITY, (u8*)&threshold, 2);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "failed");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->factory_info->cmd_state = 3;
		return;
	}

	snprintf(buff, sizeof(buff), "%u", threshold);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->factory_info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}
*/

#ifdef GLOVE_MODE
static void glove_mode(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	if (finfo->cmd_param[0] < 0 || finfo->cmd_param[0] > 1) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		finfo->cmd_state = FAIL;
	} else {
		if (finfo->cmd_param[0])
			zinitix_bit_set(m_optional_mode, TS_SENSIVE_MODE_BIT);
		else
			zinitix_bit_clr(m_optional_mode, TS_SENSIVE_MODE_BIT);

		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "OK");
		finfo->cmd_state = OK;
	}

	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;
	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}
#endif

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;

	if (finfo->cmd_is_running == true) {
		dev_err(&client->dev, "%s: other cmd is running\n", __func__);
		goto err_out;
	}

	/* check lock  */
	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = true;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = RUNNING;

	for (i = 0; i < ARRAY_SIZE(finfo->cmd_param); i++)
		finfo->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;

	memset(finfo->cmd, 0x00, ARRAY_SIZE(finfo->cmd));
	memcpy(finfo->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				finfo->cmd_param[param_cnt] =
					(int)simple_strtol(buff, NULL, 10);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
/*	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i, finfo->cmd_param[i]);*/

	tsp_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "tsp cmd: status:%d\n", finfo->cmd_state);

	if (finfo->cmd_state == WAITING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "WAITING");

	else if (finfo->cmd_state == RUNNING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "RUNNING");

	else if (finfo->cmd_state == OK)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");

	else if (finfo->cmd_state == FAIL)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "FAIL");

	else if (finfo->cmd_state == NOT_APPLICABLE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "NOT_APPLICABLE");

	return snprintf(buf, sizeof(finfo->cmd_buff),
					"%s\n", finfo->cmd_buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "tsp cmd: result: %s\n", finfo->cmd_result);

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;

	return snprintf(buf, sizeof(finfo->cmd_result),
					"%s\n", finfo->cmd_result);
}

static ssize_t show_easy_wakeup_gesture(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct tsp_factory_info *finfo = info->factory_info;
	ssize_t ret;

	mutex_lock(&finfo->cmd_lock);
	ret = snprintf(buf, PIPE_BUF, "%d\n", easy_wake);
	mutex_unlock(&finfo->cmd_lock);

	return ret;
}

static ssize_t store_easy_wakeup_gesture(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	int rc = 0;

	rc = kstrtouint(buf, 0, &easy_wake);
	if (rc < 0)
		return rc;

	if(easy_wake == 1) {
		if (write_reg(client, 0x011D, 0x0089) != I2C_SUCCESS)
			zinitix_printk("store_easy_wakeup_gesture write_reg[0x0089] failed!!!-----------------------\n");
	}
	else {
		if (write_reg(client, 0x011D, 0x0080) != I2C_SUCCESS)
			zinitix_printk("store_easy_wakeup_gesture write_reg[0x0080] failed!!!-----------------------\n");
	}

	return count;
}

static ssize_t show_bigobject_off(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct tsp_factory_info *finfo = info->factory_info;
	ssize_t ret;

	mutex_lock(&finfo->cmd_lock);
	ret = snprintf(buf, PIPE_BUF, "%d\n", big_object);
	mutex_unlock(&finfo->cmd_lock);

	return ret;
}

static ssize_t store_bigobject_off(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	int rc = 0;

	rc = kstrtouint(buf, 0, &big_object);
	if (rc < 0)
		return rc;

	if (big_object) {
		if (write_reg(client, 0x0017, 0x8A08) != I2C_SUCCESS)
			zinitix_printk("store_bigobject_off write_reg[0x0080] failed!!!-----------------------\n");
	}
	else {
		if (write_reg(client, 0x0017, 0x8A00) != I2C_SUCCESS)
			zinitix_printk("store_bigobject_off write_reg[0x0080] failed!!!-----------------------\n");
	}

	return count;
}

static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);
static DEVICE_ATTR(easy_wakeup_gesture, 0664, show_easy_wakeup_gesture, store_easy_wakeup_gesture);
static DEVICE_ATTR(bigobject_off, 0664, show_bigobject_off, store_bigobject_off);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	&dev_attr_easy_wakeup_gesture.attr,
	&dev_attr_bigobject_off.attr,
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};

#ifdef SUPPORTED_TOUCH_KEY
static ssize_t show_touchkey_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);

#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	dev_info(&client->dev, "%s: key threshold = %d\n", __func__,
			cap->key_threshold);

	return snprintf(buf, 41, "%d", cap->key_threshold);
#else
	dev_info(&client->dev, "%s: key threshold = %d %d %d %d\n", __func__,
			cap->dummy_threshold, cap->key_threshold, cap->key_threshold, cap->dummy_threshold);

	return snprintf(buf, 41, "%d %d %d %d", cap->dummy_threshold,
			cap->key_threshold, cap->key_threshold,
			cap->dummy_threshold);
#endif
}

static ssize_t show_touchkey_sensitivity(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	u16 val = 0;
	int ret;
	int i;

#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	if (!strcmp(attr->attr.name, "touchkey_menu"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 0;
	else {
		dev_err(&client->dev, "%s: Invalid attribute\n",__func__);

		goto err_out;
	}

#else
	if (!strcmp(attr->attr.name, "touchkey_dummy_btn1"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_menu"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 2;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn4"))
		i = 3;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn5"))
		i = 4;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn6"))
		i = 5;
	else {
		dev_err(&client->dev, "%s: Invalid attribute\n", __func__);

		goto err_out;
	}
#endif
	ret = read_data(client, BT541_BTN_WIDTH + i, (u8 *)&val, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s:Failed to read %d's key sensitivity\n",
					 __func__, i);

		goto err_out;
	}

	dev_info(&client->dev, "%s: %d's key sensitivity = %d\n",
				__func__, i, val);

	return snprintf(buf, 6, "%d", val);

err_out:
	return sprintf(buf, "NG");
}

static ssize_t show_back_key_raw_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t show_menu_key_raw_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
/*
static ssize_t show_back_key_idac_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t show_menu_key_idac_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
*/
#ifdef SUPPORTED_TOUCH_KEY_LED
static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	u8 data;

	sscanf(buf, "%hhu", &data);

	printk("[TKEY] %s : %d _ %d\n",__func__,data,__LINE__);

	if(data == 1)
		gpio_direction_output(info->pdata->gpio_keyled, 1);
	else
		gpio_direction_output(info->pdata->gpio_keyled, 0);

	return size;
}
#endif

static DEVICE_ATTR(touchkey_threshold, S_IRUGO, show_touchkey_threshold, NULL);
/*static DEVICE_ATTR(touch_sensitivity, S_IRUGO, back_key_state_show, NULL);*/
//static DEVICE_ATTR(extra_button_event, S_IWUSR | S_IWGRP | S_IRUGO, NULL, enable_dummy_key );
static DEVICE_ATTR(touchkey_menu, S_IRUGO, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_recent, S_IRUGO, show_touchkey_sensitivity, NULL);
#ifndef NOT_SUPPORTED_TOUCH_DUMMY_KEY
static DEVICE_ATTR(touchkey_dummy_btn1, S_IRUGO,
					show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_dummy_btn3, S_IRUGO,
					show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_dummy_btn4, S_IRUGO,
					show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_dummy_btn6, S_IRUGO,
					show_touchkey_sensitivity, NULL);
#endif
/*static DEVICE_ATTR(autocal_stat, S_IRUGO, show_autocal_status, NULL);*/
static DEVICE_ATTR(touchkey_raw_back, S_IRUGO, show_back_key_raw_data, NULL);
static DEVICE_ATTR(touchkey_raw_menu, S_IRUGO, show_menu_key_raw_data, NULL);
/*static DEVICE_ATTR(touchkey_idac_back, S_IRUGO, show_back_key_idac_data, NULL);
static DEVICE_ATTR(touchkey_idac_menu, S_IRUGO, show_menu_key_idac_data, NULL);*/
#ifdef SUPPORTED_TOUCH_KEY_LED
static DEVICE_ATTR(brightness, 0664, NULL, touch_led_control);
#endif

static struct attribute *touchkey_attributes[] = {
	&dev_attr_touchkey_threshold.attr,
	/*&dev_attr_touch_sensitivity.attr,*/
	&dev_attr_touchkey_back.attr,
	&dev_attr_touchkey_menu.attr,
	&dev_attr_touchkey_recent.attr,
	//&dev_attr_autocal_stat.attr,
	//&dev_attr_extra_button_event.attr,
	&dev_attr_touchkey_raw_menu.attr,
	&dev_attr_touchkey_raw_back.attr,
#ifndef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	&dev_attr_touchkey_dummy_btn1.attr,
	&dev_attr_touchkey_dummy_btn3.attr,
	&dev_attr_touchkey_dummy_btn4.attr,
	&dev_attr_touchkey_dummy_btn6.attr,
#endif
	//&dev_attr_touchkey_idac_back.attr,
	//&dev_attr_touchkey_idac_menu.attr,
#ifdef SUPPORTED_TOUCH_KEY_LED
	&dev_attr_brightness.attr,
#endif
	NULL,
};
static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};
#endif

static int init_sec_factory(struct bt541_ts_info *info)
{
	struct device *factory_ts_dev;
#ifdef SUPPORTED_TOUCH_KEY
	struct device *factory_tk_dev;
#endif
	struct tsp_factory_info *factory_info;
	struct tsp_raw_data *raw_data;
	int ret;
	int i;

	factory_info = kzalloc(sizeof(struct tsp_factory_info), GFP_KERNEL);
	if (unlikely(!factory_info)) {
		dev_err(&info->client->dev, "%s: Failed to allocate memory\n",
				__func__);
		ret = -ENOMEM;

		goto err_alloc1;
	}
	raw_data = kzalloc(sizeof(struct tsp_raw_data), GFP_KERNEL);
	if (unlikely(!raw_data)) {
		dev_err(&info->client->dev, "%s: Failed to allocate memory\n",
				__func__);
		ret = -ENOMEM;

		goto err_alloc2;
	}

	INIT_LIST_HEAD(&factory_info->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &factory_info->cmd_list_head);


	sec_class = class_create(THIS_MODULE, "tsp");

	factory_ts_dev = device_create(sec_class, NULL, 0, info, "tsp");
	if (unlikely(!factory_ts_dev)) {
		dev_err(&info->client->dev, "Failed to create factory dev\n");
		ret = -ENODEV;
		goto err_create_device;
	}

	ret = sysfs_create_link(&factory_ts_dev->kobj,
		&info->input_dev->dev.kobj, "input");
	if (ret < 0) {
		dev_err(&info->client->dev,
			"%s: Failed to create input symbolic link %d\n",
			__func__, ret);
	}

#ifdef SUPPORTED_TOUCH_KEY
	factory_tk_dev = device_create(sec_class, NULL, 0, info, "sec_touchkey");
	if (IS_ERR(factory_tk_dev)) {
		dev_err(&info->client->dev, "Failed to create factory dev\n");
		ret = -ENODEV;
		goto err_create_device;
	}
#endif

	ret = sysfs_create_group(&factory_ts_dev->kobj, &touchscreen_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev, "Failed to create touchscreen sysfs group\n");
		goto err_create_sysfs;
	}

#ifdef SUPPORTED_TOUCH_KEY
	ret = sysfs_create_group(&factory_tk_dev->kobj, &touchkey_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev, "Failed to create touchkey sysfs group\n");
		goto err_create_sysfs;
	}
#endif

	mutex_init(&factory_info->cmd_lock);
	factory_info->cmd_is_running = false;

	info->factory_info = factory_info;
	info->raw_data = raw_data;

	return ret;

err_create_sysfs:
err_create_device:
	kfree(raw_data);
err_alloc2:
	kfree(factory_info);
err_alloc1:

	return ret;
}
#endif

static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct raw_ioctl raw_ioctl;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 version;
	u16 mode;

	struct reg_ioctl reg_ioctl;
	u16 val;
	int nval = 0;

	if (misc_info == NULL)
	{
		zinitix_debug_msg("misc device NULL?\n");
		return -1;
	}

	switch (cmd) {

	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = m_ts_debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, 4)) {
			pr_info("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}
		if (nval)
			pr_info("[zinitix_touch] on debug mode (%d)\n",
				nval);
		else
			pr_info("[zinitix_touch] off debug mode (%d)\n",
				nval);
		m_ts_debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = misc_info->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = misc_info->cap_info.fw_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = misc_info->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -1;

		//printk(KERN_INFO "[zinitix_touch]: firmware size = %l\r\n", sz);
		if (misc_info->cap_info.ic_fw_size != sz) {
			pr_info("[zinitix_touch]: firmware size error\r\n");
			return -1;
		}
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		ts_select_type_hw(misc_info);
		if (copy_from_user(m_pFirmware[m_FirmwareIdx],
			argp, misc_info->cap_info.ic_fw_size))
			return -1;

		version = (u16) (m_pFirmware[m_FirmwareIdx][52] | (m_pFirmware[m_FirmwareIdx][53]<<8));

		pr_info("[zinitix_touch]: firmware version = %x\r\n", version);

		if (copy_to_user(argp, &version, sizeof(version)))
			return -1;
		break;

	case TOUCH_IOCTL_START_UPGRADE:
		ts_select_type_hw(misc_info);
		return ts_upgrade_sequence((u8*)m_pFirmware[m_FirmwareIdx]);

	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_info->pdata->x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_info->pdata->y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_info->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_info->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_info->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -1;
		disable_irq(misc_info->irq);
		misc_info->irq_enabled = false;
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_info("[zinitix_touch]: other process occupied.. (%d)\r\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = HW_CALIBRAION;
		mdelay(100);

		mode = misc_info->touch_mode;
		if (write_reg(misc_info->client,
			BT541_TOUCH_MODE, mode) != I2C_SUCCESS) {
			pr_err("[zinitix_touch]: failed to set touch mode %d.\n",
				mode);
			goto fail_hw_cal;
		}

		if (write_cmd(misc_info->client,
			BT541_SWRESET_CMD) != I2C_SUCCESS)
			goto fail_hw_cal;

		enable_irq(misc_info->irq);
		misc_info->irq_enabled = true;
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;
fail_hw_cal:
		enable_irq(misc_info->irq);
		misc_info->irq_enabled = true;
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		if (copy_from_user(&nval, argp, 4)) {
			pr_info("[zinitix_touch] error : copy_from_user\r\n");
			misc_info->work_state = NOTHING;
			return -1;
		}
		ts_set_touchmode((u16)nval);

		return 0;

	case TOUCH_IOCTL_GET_REG:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_info("[zinitix_touch]:other process occupied.. (%d)\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;

		if (copy_from_user(&reg_ioctl,
			argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_info("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (read_data(misc_info->client,
			reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		nval = (int)val;

		if (copy_to_user(reg_ioctl.val, (u8 *)&nval, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_info("[zinitix_touch] error : copy_to_user\n");
			return -1;
		}

		zinitix_debug_msg("read : reg addr = 0x%x, val = 0x%x\n",
			reg_ioctl.addr, nval);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:

		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_info("[zinitix_touch]: other process occupied.. (%d)\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl,
				argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_info("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (copy_from_user(&val, reg_ioctl.val, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_info("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (write_reg(misc_info->client,
			reg_ioctl.addr, val) != I2C_SUCCESS)
			ret = -1;

		zinitix_debug_msg("write : reg addr = 0x%x, val = 0x%x\r\n",
			reg_ioctl.addr, val);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:

		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_info("[zinitix_touch]: other process occupied.. (%d)\r\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (write_reg(misc_info->client,
			BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			ret = -1;
		zinitix_debug_msg("write : reg addr = 0x%x, val = 0x0\r\n",
			BT541_INT_ENABLE_FLAG);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_info("[zinitix_touch]: other process occupied.." \
				"(%d)\r\n", misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = SET_MODE;
		ret = 0;
		write_reg(misc_info->client, 0xc003, 0x0001);
		write_reg(misc_info->client, 0xc104, 0x0001);
		if (write_cmd(misc_info->client,
			BT541_SAVE_STATUS_CMD) != I2C_SUCCESS)
			ret =  -1;

		mdelay(1000);	/* for fusing eeprom */
		write_reg(misc_info->client, 0xc003, 0x0000);
		write_reg(misc_info->client, 0xc104, 0x0000);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}

		if (misc_info->touch_mode == TOUCH_POINT_MODE)
			return -1;

		down(&misc_info->raw_data_lock);
		if (misc_info->update == 0) {
			up(&misc_info->raw_data_lock);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
			argp, sizeof(struct raw_ioctl))) {
			up(&misc_info->raw_data_lock);
			pr_info("[zinitix_touch] error : copy_from_user\r\n");
			return -1;
		}

		misc_info->update = 0;

		u8Data = (u8 *)&misc_info->cur_data[0];
		if (raw_ioctl.sz > MAX_TRAW_DATA_SZ*2)
			raw_ioctl.sz = MAX_TRAW_DATA_SZ*2;
		if (copy_to_user(raw_ioctl.buf, (u8 *)u8Data,
			raw_ioctl.sz)) {
			up(&misc_info->raw_data_lock);
			return -1;
		}

		up(&misc_info->raw_data_lock);
		return 0;

	default:
		break;
	}
	return 0;
}


static int zinitix_power_init(struct bt541_ts_info *data, bool on)
{
	struct bt541_ts_platform_data *pdata = data->pdata;
	int rc;

	if (!on)
		goto pwr_deinit;

	if (strlen(pdata->avdd_name)) {
		data->vdd = regulator_get(&data->client->dev, pdata->avdd_name);
		if (IS_ERR_OR_NULL(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			data->vdd = NULL;
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
						   FT_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	} else {
		pr_info("%s: Avdd name is NULL", __func__);
	}

	if (strlen(pdata->iovdd_name)) {
		data->vcc_i2c = regulator_get(&data->client->dev, pdata->iovdd_name);
		if (IS_ERR_OR_NULL(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&data->client->dev,
				"Regulator get failed vcc_i2c rc=%d\n", rc);
			data->vcc_i2c = NULL;
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
						   FT_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				goto reg_vcc_i2c_put;
			}
		}
	} else {
		pr_info("%s: iovdd name is NULL", __func__);
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (!IS_ERR_OR_NULL(data->vdd)) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

		regulator_put(data->vdd);
	}

	if (!IS_ERR_OR_NULL(data->vcc_i2c)) {
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

		regulator_put(data->vcc_i2c);
	}
	return 0;
}

static int zinitix_ts_pinctrl_init(struct bt541_ts_info *bt541_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	bt541_data->ts_pinctrl = devm_pinctrl_get(&(bt541_data->client->dev));
	if (IS_ERR_OR_NULL(bt541_data->ts_pinctrl)) {
		retval = PTR_ERR(bt541_data->ts_pinctrl);
		dev_dbg(&bt541_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	bt541_data->pinctrl_state_active
		= pinctrl_lookup_state(bt541_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_active)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_active);
		dev_err(&bt541_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	bt541_data->pinctrl_state_suspend
		= pinctrl_lookup_state(bt541_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_suspend);
		dev_err(&bt541_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	bt541_data->pinctrl_state_release
		= pinctrl_lookup_state(bt541_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_release)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_release);
		dev_dbg(&bt541_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(bt541_data->ts_pinctrl);
err_pinctrl_get:
	bt541_data->ts_pinctrl = NULL;
	return retval;
}

#ifdef CONFIG_OF
static const struct of_device_id zinitix_match_table[] = {
	{ .compatible = "zinitix,bt541_ts_device",},
	{},
};
int zinitix_hw_reset( struct bt541_ts_info* data,bool on )
{
	int err = 0;
	printk("zinitix_hw_reset: on = %d, gpio_reset = %d\n", on, data->pdata->gpio_reset);
	if(on)
	{
		if (gpio_is_valid(data->pdata->gpio_reset)) {
		/*
		* This is intended to save leakage current
		* only. Even if the call(gpio_direction_input)
		* fails, only leakage current will be more but
		* functionality will not be affected.
		*/


		gpio_direction_output(data->pdata->gpio_reset, 1);
		mdelay(20);
		gpio_direction_output(data->pdata->gpio_reset, 0);
		mdelay(100)/*eric modify 20170109*/;
		gpio_direction_output(data->pdata->gpio_reset, 1);
		//mdelay(10);
		//err = gpio_direction_input(data->pdata->gpio_reset);
		if (err) {
		dev_err(&data->client->dev,
			"unable to set direction for gpio "
			"[%d]\n", data->pdata->gpio_reset);
		}
		//gpio_free(data->pdata->gpio_reset);
		}
	}

 	return err;
}

static int zinitix_init_gpio(struct bt541_ts_info* data,bool on )
{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->gpio_int)) {
			err = gpio_request(data->pdata->gpio_int,
						"bt541_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				zinitix_printk("irq gpio request failed is %d\n",err);
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->gpio_int);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_req;
			}
		}

		//init power gpio
		if (data->pdata->avdd_gpio > 0) {
			if (gpio_is_valid(data->pdata->avdd_gpio)) {
				err = gpio_request(data->pdata->avdd_gpio,
							"bt541_avdd_gpio");
				if (err) {
					dev_err(&data->client->dev,
						"avdd gpio request failed");
					zinitix_printk("avdd gpio request failed is %d\n",err);
					goto err_avdd_gpio_req;
				}

				err = gpio_direction_output(data->pdata->avdd_gpio, 0);
				if (err) {
					dev_err(&data->client->dev,
						"set_direction for avdd gpio failed\n");
					goto err_avdd_gpio_req;
				}
			}
		}

		if (data->pdata->iovdd_gpio > 0) {
			if (gpio_is_valid(data->pdata->iovdd_gpio)) {
				err = gpio_request(data->pdata->iovdd_gpio,
							"bt541_iovdd_gpio");
				if (err) {
					dev_err(&data->client->dev,
						"iovdd gpio request failed");
					zinitix_printk("iovdd gpio request failed is %d\n",err);
					goto err_iovdd_gpio_req;
				}

				err = gpio_direction_output(data->pdata->iovdd_gpio, 0);
				if (err) {
					dev_err(&data->client->dev,
						"set_direction for avdd gpio failed\n");
					goto err_iovdd_gpio_req;
				}
			}
		}

		return 0;
	}
	else {
		if (gpio_is_valid(data->pdata->gpio_int))
			gpio_free(data->pdata->gpio_int);
		if (gpio_is_valid(data->pdata->avdd_gpio))
			gpio_free(data->pdata->avdd_gpio);
		if (gpio_is_valid(data->pdata->iovdd_gpio))
			gpio_free(data->pdata->iovdd_gpio);

		return 0;
	}

err_iovdd_gpio_req:
	if (gpio_is_valid(data->pdata->iovdd_gpio))
		gpio_free(data->pdata->iovdd_gpio);
err_avdd_gpio_req:
	if (gpio_is_valid(data->pdata->avdd_gpio))
		gpio_free(data->pdata->avdd_gpio);
err_irq_gpio_req:
	if (gpio_is_valid(data->pdata->gpio_reset))
		gpio_free(data->pdata->gpio_reset);

	return err;
}
#endif

static int bt541_ts_probe_dt(struct device_node *np,
			 struct device *dev,
			 struct bt541_ts_platform_data *pdata)
{
#ifdef SUPPORTED_TOUCH_KEY_LED
	int keyled_n = -1;
	int size_p;
#endif
	int ret = 0;
	u32 temp;
	const char *name_tmp;

	ret = of_property_read_u32(np, "zinitix,x_resolution", &temp);
	if (ret) {
		dev_err(dev, "Unable to read controller version zinitix,x_resolution\n");
		return ret;
	} else
		pdata->x_resolution = (u16) temp;
	ret = of_property_read_u32(np, "zinitix,y_resolution", &temp);
	if (ret) {
		dev_err(dev, "Unable to read controller version zinitix,y_resolution\n");
		return ret;
	} else
		pdata->y_resolution = (u16) temp;
	ret = of_property_read_u32(np, "zinitix,page_size", &temp);
	if (ret) {
		dev_err(dev, "Unable to read controller version zinitix,page_size\n");
		return ret;
	} else
		pdata->page_size = (u16) temp;
	ret = of_property_read_u32(np, "zinitix,orientation", &temp);
	if (ret) {
		dev_err(dev, "Unable to read controller version zinitix,orientation\n");
		return ret;
	} else
		pdata->orientation = (u8) temp;

	pdata->tsp_vendor1 = of_get_named_gpio(np, "zinitix,vendor1", 0);
	pdata->tsp_vendor2 = of_get_named_gpio(np, "zinitix,vendor2", 0);

#ifdef SUPPORTED_TOUCH_KEY_LED
	if (of_find_property(np, "keyled_gpio", &size_p)) {
		keyled_n = of_get_named_gpio(np, "keyled_gpio", 0);
		if (keyled_n < 0) {
			pr_err("%s: of_get_named_gpio failed: keyled_gpio %d\n", __func__,
				keyled_n);
			return -EINVAL;
		}
	}
	pdata->gpio_keyled = keyled_n;
#endif
	pdata->gpio_reset = of_get_named_gpio(np, "zinitix,reset-gpio", 0);
	if (pdata->gpio_reset < 0) {
		pr_err("%s: of_get_named_gpio failed: tsp_gpio %d\n", __func__,
			pdata->gpio_reset);
		return -EINVAL;
	}

	pdata->gpio_int = of_get_named_gpio(np, "zinitix,irq-gpio", 0);
	if (pdata->gpio_int < 0) {
		pr_err("%s: of_get_named_gpio failed: tsp_gpio %d\n", __func__,
			pdata->gpio_int);
		return -EINVAL;
	}

	/* get ic name */
	ret = of_property_read_string(np, "ic_name", &name_tmp);
	if (ret) {
		pr_err("%s, get ic_name failed", __func__);
		return -EINVAL;
	} else {
		pr_info("%s, ic_name from dt: %s", __func__, name_tmp);
		strncpy(pdata->ic_name, name_tmp, sizeof(pdata->ic_name));
	}

	//Get power resource
	pdata->avdd_gpio = of_get_named_gpio(np, "zinitix,avdd-gpio", 0);
	if (pdata->avdd_gpio  < 0) {
		pr_err("%s: can't find avdd-gpio, use other power supply", __func__);
		pdata->avdd_gpio = 0;
	} else {
		pr_info("get avdd-gpio[%d] from dt", pdata->avdd_gpio);
	}

	pdata->iovdd_gpio = of_get_named_gpio(np, "zinitix,iovdd-gpio", 0);
	if (pdata->iovdd_gpio  < 0) {
		pr_err("%s: can't find iovdd-gpio, use other power supply", __func__);
		pdata->iovdd_gpio = 0;
	} else {
		pr_info("%s: get iovdd-gpio[%d] from dt", __func__, pdata->iovdd_gpio);
	}

	memset(pdata->avdd_name, 0, sizeof(pdata->avdd_name));
	ret = of_property_read_string(np, "zinitix,avdd-name", &name_tmp);
	if (!ret) {
		pr_info("%s: avdd name from dt: %s", __func__, name_tmp);
		if (strlen(name_tmp) < sizeof(pdata->avdd_name))
			strncpy(pdata->avdd_name, name_tmp, sizeof(pdata->avdd_name));
		else
			pr_info("%s, invalied avdd name length: %ld > %ld", __func__,
				strlen(name_tmp), sizeof(pdata->avdd_name));
	}

	memset(pdata->iovdd_name, 0, sizeof(pdata->iovdd_name));
	ret = of_property_read_string(np, "zinitix,iovdd-name", &name_tmp);
	if (!ret) {
		pr_info("%s: iovdd name from dt: %s", __func__, name_tmp);
		if (strlen(name_tmp) < sizeof(pdata->iovdd_name))
			strncpy(pdata->iovdd_name, name_tmp, sizeof(pdata->iovdd_name));
		else
			pr_info("%s, invalied iovdd name length: %ld > %ld", __func__,
				strlen(name_tmp), sizeof(pdata->iovdd_name));
	}

	ret = of_property_read_u32(np, "zinitix,tsp_vdd_supply_type",
				&pdata->tsp_supply_type);
	if (ret < 0) {
		pr_err("%s: failed to read property tsp_vdd_supply_type\n",
			__func__);
		return ret;
	}

	ret = of_property_read_string(np, "zinitix,pname",
				&pdata->pname);
	if (ret < 0) {
		pr_err("%s: failed to read property config_ver\n",
			__func__);
		return ret;
	}

	if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
		pdata->tsp_en_gpio = of_get_named_gpio(np, "zinitix,vdd_en-gpio", 0);
		if (pdata->tsp_en_gpio < 0) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
				pdata->tsp_en_gpio);
			return -EINVAL;
		}
	}

	pr_err("%s: x_resolution :%d, y_resolution :%d, page_size :%d, orientation :%d, gpio_int :%d, vcc_en :%d\n",
		__func__, pdata->x_resolution, pdata->y_resolution, pdata->page_size,
		pdata->orientation, pdata->gpio_int, pdata->tsp_en_gpio);

	return 0;
}

#ifdef USE_TSP_TA_CALLBACKS
void bt541_register_callback(struct tsp_callbacks *cb)
{
	charger_callbacks = cb;
	pr_info("%s\n", __func__);
}
#endif

static int bt541_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *i2c_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bt541_ts_platform_data *pdata = NULL;
	struct bt541_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
#ifdef SUPPORTED_TOUCH_KEY
	int i;
#endif

	struct device_node *np = client->dev.of_node;
	zinitix_printk("[TSP]: %s\n", __func__);


	if (client->dev.of_node) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = bt541_ts_probe_dt(np, &client->dev, pdata);
		if (ret){
			dev_err(&client->dev, "Error parsing dt %d\n", ret);
			goto err_no_platform_data;
		}

#ifdef USE_TSP_TA_CALLBACKS
		pdata->register_cb = bt541_register_callback;
#endif

	} else if (!pdata) {
		dev_err(&client->dev, "%s: no platform data defined\n",
			__func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		ret = -EIO;
		goto err_no_platform_data;
	}

	info = kzalloc(sizeof(struct bt541_ts_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_mem_alloc;
	}

	i2c_set_clientdata(client, info);
	info->client = client;
	info->pdata = pdata;
	info->device_enabled = 1;
	misc_touch_dev = info;

	ret = zinitix_ts_pinctrl_init(info);
	if (!ret && info->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_active);
		if (ret < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}

	ret = gpio_request(info->pdata->gpio_reset,
				"bt541_reset_gpio");
	if (ret) {
		dev_err(&info->client->dev,
			"reset gpio request failed");
		zinitix_printk("reset gpio request failed is %d\n",ret);
	}

	ret = zinitix_init_gpio(info,true);
		if(ret < 0)
			goto err_gpio_request;
#if 0
/* Get pinctrl if target uses pinctrl */
	info->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(info->pinctrl)) {
		if (PTR_ERR(info->pinctrl) == -EPROBE_DEFER)
			goto err_alloc;

		dev_info(&client->dev,"%s: Target does not use pinctrl\n", __func__);
		info->pinctrl = NULL;
	}

	if (info->pinctrl) {
		ret = bt541_pinctrl_configure(info, true);
		if (ret)
			dev_info(&client->dev,"%s: cannot set pinctrl state\n", __func__);
	}
#endif

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

#ifdef USE_TSP_TA_CALLBACKS
	info->register_cb = info->pdata->register_cb;
#endif

	info->input_dev = input_dev;
	info->work_state = PROBE;

	ret = zinitix_power_init(info,true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto err_input_register_device;
	}
	#if 0
	info->vddo_vreg = regulator_get(&info->client->dev,"vddo");
	if (IS_ERR(info->vddo_vreg)){
		info->vddo_vreg = NULL;
		printk(KERN_INFO "info->vddo_vreg error\n");
		ret = -EPERM;
		goto err_power_sequence;
	}
	if (pdata->tsp_supply_type == TSP_REGULATOR_SUPPLY)
	{
		info->vdd_en = regulator_get(&info->client->dev,"vdd_en");
		if (IS_ERR(info->vdd_en)){
			info->vdd_en = NULL;
			printk(KERN_INFO "info->vdd_en error\n");
			ret = -EPERM;
			goto err_power_sequence;
		}
	}
	#endif
	/* power on */
	if (bt541_power_control(info, POWER_ON_SEQUENCE) == false) {
		ret = -EPERM;
		goto err_power_sequence;
	}

	/* To Do */
	/* FW version read from tsp */
	memset(&info->reported_touch_info,
		0x0, sizeof(struct point_info));

	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;

	misc_info = info;
	info->checkUMSmode = false;

	if (init_touch(info) == false) {
		ret = -EPERM;
		goto err_touch_init;
	}

#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		info->button[i] = ICON_BUTTON_UNCHANGE;
#endif

#ifdef USE_TSP_TA_CALLBACKS
	info->callbacks.inform_charger = bt541_charger_status_cb;
		if (info->register_cb)
			info->register_cb(&info->callbacks);
#endif

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));
	input_dev->name = "cyttsp5_mt";
	input_dev->id.bustype = BUS_I2C;
	input_dev->phys = info->phys;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
	set_bit(BTN_TOUCH, info->input_dev->keybit);

#if SUPPORTED_PALM_TOUCH
	//set_bit(KEY_POWER, info->input_dev->evbit);
	//input_set_capability(info->input_dev, EV_KEY, KEY_LARGE);
	input_set_capability(info->input_dev, EV_KEY, KEY_POWER);
	dev_err(&client->dev,"KEY_POWER register finish\n");
#endif

	set_bit(EV_LED, info->input_dev->evbit);
	set_bit(LED_MISC, info->input_dev->ledbit);

	set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);

#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		set_bit(BUTTON_MAPPING_KEY[i], info->input_dev->keybit);
#endif

	if (pdata->orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinX,
			info->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinY,
			info->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	} else {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinX,
			info->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinY,
			info->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	}

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR,
		0, 255, 0, 0);

#if (TOUCH_POINT_MODE == 2)
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR,
		0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PALM,
		0, 1, 0, 0);
#endif

	//info->input_dev->open = bt541_input_open;
	//info->input_dev->close = bt541_input_close;
	set_bit(MT_TOOL_FINGER, info->input_dev->keybit);
	//wanzhiming
	input_mt_init_slots(info->input_dev, info->cap_info.multi_fingers, INPUT_MT_DIRECT);

	zinitix_debug_msg("register %s input device \r\n",
		info->input_dev->name);
	input_set_drvdata(info->input_dev, info);
	ret = input_register_device(info->input_dev);
	if (ret) {
		pr_err("unable to register %s input device\r\n",
			info->input_dev->name);
		goto err_touch_init;
	}

	info->work_state = NOTHING;
	sema_init(&info->work_lock, 1);

#if ESD_TIMER_INTERVAL
	spin_lock_init(&info->lock);
	INIT_WORK(&info->tmr_work, ts_tmr_work);
	esd_tmr_workqueue =
		create_singlethread_workqueue("esd_tmr_workqueue");

	if (!esd_tmr_workqueue) {
		dev_err(&client->dev, "Failed to create esd tmr work queue\n");
		ret = -EPERM;

		goto err_input_register_device2;
	}

	esd_timer_init(info);
	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "Started esd timer\n");
#endif
#endif
#if 0
	info->irq = irq_of_parse_and_map(client->dev.of_node, 0);
	if (!info->irq) {
		dev_err(&client->dev, "Failed to retrieve IRQ from device tree.\n");
		ret = -ENODEV;
		goto err_irq_of_parse;
	}
#endif

	/* configure irq */
	info->irq = gpio_to_irq(pdata->gpio_int);
	if (info->irq < 0){
		dev_err(&client->dev, "Invalid GPIO_TOUCH_IRQ\n");
		ret = -ENODEV;
		goto err_irq_of_parse;
	}
	info->irq_enabled = true;

	pdata->tsp_irq = info->irq;
	ret = request_threaded_irq(info->irq, NULL, bt541_touch_work,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT , BT541_TS_DEVICE, info);

	if (ret) {
		printk(KERN_ERR "unable to register irq.(%s)\r\n",
			info->input_dev->name);
		goto err_request_irq;
	}
#if defined(CONFIG_FB)
	info->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&info->fb_notif);
	if (ret)
		dev_err(&info->client->dev,
			"Unable to register fb_notifier: %d\n",
			ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = zinitix_early_suspend;
	info->early_suspend.resume = zinitix_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

	dev_info(&client->dev, "zinitix touch probe.\r\n");

#if defined(CONFIG_PM_RUNTIME)
	//pm_runtime_enable(&client->dev);
#endif

	sema_init(&info->raw_data_lock, 1);

	ret = misc_register(&touch_misc_device);
	if (ret) {
		dev_err(&client->dev, "Failed to register touch misc device\n");
		goto err_misc_register;
	}

#ifdef SEC_FACTORY_TEST
	ret = init_sec_factory(info);
	if (ret) {
		dev_err(&client->dev, "Failed to init sec factory device\n");

		goto err_kthread_create_failed;
	}
#endif

#if USE_WAKEUP_GESTURE/*eric add 20170208*/

	enable_irq_wake(misc_info->irq);
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	dev_info(&client->dev, "%s:zinitix_ts_mmi_dev_register", __func__);
	ret = zinitix_ts_mmi_dev_register(&client->dev);
	if (ret) {
		dev_info(&client->dev, "%s: Failed register touchscreen mmi.", __func__);
		goto err_ts_mmi_register;
	}
#endif

	dev_info(&client->dev, "bt541_ts_probe: SUCCESS\n");

	return ret;

err_ts_mmi_register:
#ifdef SEC_FACTORY_TEST
err_kthread_create_failed:
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
err_misc_register:
	free_irq(info->irq, info);
err_irq_of_parse:
err_request_irq:
#if ESD_TIMER_INTERVAL
err_input_register_device2:
#endif

	input_unregister_device(info->input_dev);
err_touch_init:
	bt541_power_control(info, POWER_OFF);
err_power_sequence:
	zinitix_power_init(info, false);
err_input_register_device:
	input_free_device(info->input_dev);
err_alloc:
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
	if (gpio_is_valid(pdata->gpio_int))
		gpio_free(pdata->gpio_int);
err_gpio_request:
#ifdef SUPPORTED_TOUCH_KEY_LED
	if (pdata->gpio_keyled >= 0)
		gpio_free(pdata->gpio_keyled);
#endif

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_release);
			if (ret)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
	kfree(info);
err_mem_alloc:
err_no_platform_data:
	if (IS_ENABLED(CONFIG_OF))
		devm_kfree(&client->dev, (void *)pdata);

	dev_info(&client->dev, "Failed to probe\n");
	return ret;
}

static int bt541_ts_remove(struct i2c_client *client)
{
	int err = 0;
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	struct bt541_ts_platform_data *pdata = info->pdata;

	disable_irq(info->irq);
	info->irq_enabled = false;
	down(&info->work_lock);

	info->work_state = REMOVE;

#ifdef SEC_FACTORY_TEST
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
	write_reg(info->client, BT541_PERIODICAL_INTERRUPT_INTERVAL, 0);
	esd_timer_stop(info);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "Stopped esd timer\n");
#endif
	destroy_workqueue(esd_tmr_workqueue);
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	dev_info(&client->dev, "%s:zinitix_ts_mmi_dev_unregister", __func__);
	zinitix_ts_mmi_dev_unregister(&client->dev);
#endif

	if (info->irq)
		free_irq(info->irq, info);

	misc_deregister(&touch_misc_device);

	bt541_power_control(info, POWER_OFF);
	zinitix_power_init(info, false);

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
	if (gpio_is_valid(pdata->gpio_int))
		gpio_free(pdata->gpio_int);

#ifdef SUPPORTED_TOUCH_KEY_LED
	if ( gpio_is_valid(pdata->gpio_keyled))
		gpio_free(pdata->gpio_keyled);
#endif

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}


	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	up(&info->work_lock);
	kfree(info);

	return 0;
}

/*void bt541_ts_shutdown(struct i2c_client *client)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s++\n", __func__);
	disable_irq(info->irq);
	down(&info->work_lock);
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
	esd_timer_stop(info);
#endif
	up(&info->work_lock);
	bt541_power_control(info, POWER_OFF);
	dev_info(&client->dev, "%s--\n", __func__);
}
*/
static struct i2c_device_id bt541_idtable[] = {
	{BT541_TS_DEVICE, 0},
	{ }
};
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND) \
	&& !defined(CONFIG_INPUT_TOUCHSCREEN_MMI))
static const struct dev_pm_ops zinitix_ts_dev_pm_ops = {
	.suspend = bt541_ts_suspend,
	.resume = bt541_ts_resume,
};
#else
static const struct dev_pm_ops zinitix_ts_dev_pm_ops = {
};
#endif
static struct i2c_driver bt541_ts_driver = {
	.probe	= bt541_ts_probe,
	.remove	= bt541_ts_remove,
	#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend    = zinitix_early_suspend,
	.resume     = zinitix_late_resume,
	#endif
//	.shutdown = bt541_ts_shutdown,
	.id_table	= bt541_idtable,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= BT541_TS_DEVICE,
	.of_match_table = zinitix_match_table,
	#ifdef CONFIG_PM
		.pm	= &zinitix_ts_dev_pm_ops,
	#endif
	},
};

static int __init bt541_ts_init(void)
{
	zinitix_printk("[TSP]: %s\n", __func__);

	return i2c_add_driver(&bt541_ts_driver);
}

static void __exit bt541_ts_exit(void)
{
	zinitix_printk("[TSP]: %s\n", __func__);
	i2c_del_driver(&bt541_ts_driver);
}

module_init(bt541_ts_init);
//late_initcall(bt541_ts_init);
module_exit(bt541_ts_exit);

#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("<mika.kim@samsung.com>");
MODULE_LICENSE("GPL");

