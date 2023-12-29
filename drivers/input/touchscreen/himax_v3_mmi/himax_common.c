/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for common functions
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

/*#include "himax_common.h"*/
/*#include "himax_ic_core.h"*/
#include "himax_inspection.h"
#include "himax_modular_table.h"

#ifdef HX_SMART_WAKEUP
#define GEST_SUP_NUM 26
/* Setting cust key define (DF = double finger) */
/* {Double Tap, Up, Down, Left, Rright, C, Z, M,
 *	O, S, V, W, e, m, @, (reserve),
 *	Finger gesture, ^, >, <, f(R), f(L), Up(DF), Down(DF),
 *	Left(DF), Right(DF)}
 */
	uint8_t gest_event[GEST_SUP_NUM] = {
	0x80, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x81, 0x1D, 0x2D, 0x3D, 0x1F, 0x2F, 0x51, 0x52,
	0x53, 0x54};

/*gest_event mapping to gest_key_def*/
	uint16_t gest_key_def[GEST_SUP_NUM] = {
	KEY_POWER, 251, 252, 253, 254, 255, 256, 257,
	258, 259, 260, 261, 262, 263, 264, 265,
	266, 267, 268, 269, 270, 271, 272, 273,
	274, 275};

uint8_t *wake_event_buffer;
#endif

#ifdef HIMAX_PALM_SENSOR_EN
static struct sensors_classdev __maybe_unused palm_sensors_touch_cdev = {
	.name = "palm-gesture",
	.vendor = "Himax",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_TOUCH_PALM,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "1",
	.min_delay = 0,
	.max_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static void himax_palm_detect_func(struct himax_ts_data *ts, bool detected);
#endif

#ifdef HIMAX_V2_SENSOR_EN
static struct sensors_classdev __maybe_unused sensors_touch_cdev = {
	.name = "dt-gesture",
	.vendor = "Himax",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_DOUBLE_TAP,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "1",
	.min_delay = 0,
	.max_delay = 0,
	/* WAKE_UP & SPECIAL_REPORT */
	.flags = 1 | 6,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#define REPORT_MAX_COUNT 10000
#endif

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(5000)
#define FRAME_COUNT 5

#ifdef HX_TP_PROC_GUEST_INFO
struct hx_guest_info *g_guest_info_data;
EXPORT_SYMBOL(g_guest_info_data);

char *g_guest_info_item[] = {
	"projectID",
	"CGColor",
	"BarCode",
	"Reserve1",
	"Reserve2",
	"Reserve3",
	"Reserve4",
	"Reserve5",
	"VCOM",
	"Vcom-3Gar",
	NULL
};
#endif

uint32_t g_hx_chip_inited;

#if defined(__EMBEDDED_FW__)
struct firmware g_embedded_fw = {
	.data = _binary___Himax_firmware_bin_start,
};
#endif

#define HIMAX_FILE_NAME_LENGTH                    128

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
#if defined(HX_EN_DYNAMIC_NAME)
char *i_CTPM_firmware_name;
#else
char *i_CTPM_firmware_name = "Himax_firmware.bin";
#endif
bool g_auto_update_flag;
#endif
#if defined(HX_AUTO_UPDATE_FW)
unsigned char *i_CTPM_FW;
int i_CTPM_FW_len;
int g_i_FW_VER;
int g_i_CFG_VER;
int g_i_CID_MAJ; /*GUEST ID*/
int g_i_CID_MIN; /*VER for GUEST*/
#endif
#ifdef HX_ZERO_FLASH
int g_f_0f_updat;
#endif

struct himax_ts_data *private_ts;
#if defined(__HIMAX_HX83102D_MOD__)
extern struct zf_operation *pzf_op;
#endif
EXPORT_SYMBOL(private_ts);

struct himax_ic_data *ic_data;
EXPORT_SYMBOL(ic_data);

struct himax_report_data *hx_touch_data;
EXPORT_SYMBOL(hx_touch_data);

struct himax_core_fp g_core_fp;
EXPORT_SYMBOL(g_core_fp);

struct himax_debug *debug_data;
EXPORT_SYMBOL(debug_data);

struct proc_dir_entry *himax_touch_proc_dir;
EXPORT_SYMBOL(himax_touch_proc_dir);

struct himax_chip_detect *g_core_chip_dt;
EXPORT_SYMBOL(g_core_chip_dt);

int g_mmi_refcnt;
EXPORT_SYMBOL(g_mmi_refcnt);

#define HIMAX_PROC_TOUCH_FOLDER "android_touch"
/*ts_work about start*/
struct himax_target_report_data *g_target_report_data;
EXPORT_SYMBOL(g_target_report_data);

static void himax_report_all_leave_event(struct himax_ts_data *ts);
/*ts_work about end*/
static int		HX_TOUCH_INFO_POINT_CNT;

struct filename* (*kp_getname_kernel)(const char *filename);
void (*kp_putname_kernel)(struct filename *name);
struct file* (*kp_file_open_name)(struct filename *name, int flags, umode_t mode);

unsigned long FW_VER_MAJ_FLASH_ADDR;
EXPORT_SYMBOL(FW_VER_MAJ_FLASH_ADDR);

unsigned long FW_VER_MIN_FLASH_ADDR;
EXPORT_SYMBOL(FW_VER_MIN_FLASH_ADDR);

unsigned long CFG_VER_MAJ_FLASH_ADDR;
EXPORT_SYMBOL(CFG_VER_MAJ_FLASH_ADDR);

unsigned long CFG_VER_MIN_FLASH_ADDR;
EXPORT_SYMBOL(CFG_VER_MIN_FLASH_ADDR);

unsigned long CID_VER_MAJ_FLASH_ADDR;
EXPORT_SYMBOL(CID_VER_MAJ_FLASH_ADDR);

unsigned long CID_VER_MIN_FLASH_ADDR;
EXPORT_SYMBOL(CID_VER_MIN_FLASH_ADDR);
/*unsigned long	PANEL_VERSION_ADDR;*/

unsigned long FW_VER_MAJ_FLASH_LENG;
EXPORT_SYMBOL(FW_VER_MAJ_FLASH_LENG);

unsigned long FW_VER_MIN_FLASH_LENG;
EXPORT_SYMBOL(FW_VER_MIN_FLASH_LENG);

unsigned long CFG_VER_MAJ_FLASH_LENG;
EXPORT_SYMBOL(CFG_VER_MAJ_FLASH_LENG);

unsigned long CFG_VER_MIN_FLASH_LENG;
EXPORT_SYMBOL(CFG_VER_MIN_FLASH_LENG);

unsigned long CID_VER_MAJ_FLASH_LENG;
EXPORT_SYMBOL(CID_VER_MAJ_FLASH_LENG);

unsigned long CID_VER_MIN_FLASH_LENG;
EXPORT_SYMBOL(CID_VER_MIN_FLASH_LENG);
/*unsigned long	PANEL_VERSION_LENG;*/

unsigned long FW_CFG_VER_FLASH_ADDR;


unsigned char IC_CHECKSUM;
EXPORT_SYMBOL(IC_CHECKSUM);

#ifdef HX_ESD_RECOVERY
u8 HX_ESD_RESET_ACTIVATE;
EXPORT_SYMBOL(HX_ESD_RESET_ACTIVATE);

int hx_EB_event_flag;
EXPORT_SYMBOL(hx_EB_event_flag);

int hx_EC_event_flag;
EXPORT_SYMBOL(hx_EC_event_flag);

int hx_ED_event_flag;
EXPORT_SYMBOL(hx_ED_event_flag);

int g_zero_event_count;

#endif

u8 HX_HW_RESET_ACTIVATE;

static uint8_t AA_press;
static uint8_t EN_NoiseFilter;
static uint8_t Last_EN_NoiseFilter;

static int p_point_num = 0xFFFF;
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
static uint8_t vk_press;
static int tpd_key;
static int tpd_key_old;
#endif
static int probe_fail_flag;
#ifdef HX_USB_DETECT_GLOBAL
	bool USB_detect_flag;
#endif

#if defined(CONFIG_DRM)
int drm_notifier_callback(struct notifier_block *self,
						unsigned long event, void *data);
#elif defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
						unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#ifdef HX_GESTURE_TRACK
	static int gest_pt_cnt;
	static int gest_pt_x[GEST_PT_MAX_NUM];
	static int gest_pt_y[GEST_PT_MAX_NUM];
	static int gest_start_x, gest_start_y, gest_end_x, gest_end_y;
	static int gest_width, gest_height, gest_mid_x, gest_mid_y;
	static int hx_gesture_coor[16];
#endif

int g_ts_dbg;
EXPORT_SYMBOL(g_ts_dbg);

/* File node for Selftest, SMWP and HSEN - Start*/
#define HIMAX_PROC_SELF_TEST_FILE	"self_test"
struct proc_dir_entry *himax_proc_self_test_file;

#define HIMAX_PROC_SELF_TEST_RAW_FILE	"self_test_rawdata"
struct proc_dir_entry *himax_proc_self_raw_file = NULL;

uint8_t HX_PROC_SEND_FLAG;
EXPORT_SYMBOL(HX_PROC_SEND_FLAG);

#ifdef HX_SMART_WAKEUP
	#define HIMAX_PROC_SMWP_FILE "SMWP"
	struct proc_dir_entry *himax_proc_SMWP_file = NULL;
	#define HIMAX_PROC_GESTURE_FILE "GESTURE"
	struct proc_dir_entry *himax_proc_GESTURE_file = NULL;
	uint8_t HX_SMWP_EN;
#ifdef HX_P_SENSOR
	#define HIMAX_PROC_PSENSOR_FILE "Psensor"
	struct proc_dir_entry *himax_proc_psensor_file = NULL;
#endif
#endif

#ifdef HX_EDGE_LIMIT
	#define HIMAX_PROC_EDGE_LIMIT_FILE "edge_en"
	struct proc_dir_entry *himax_proc_edge_limit_file = NULL;
#endif

#if defined(HX_SMART_WAKEUP) || defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
bool FAKE_POWER_KEY_SEND = true;
#endif

#ifdef HX_HIGH_SENSE
	#define HIMAX_PROC_HSEN_FILE "HSEN"
	struct proc_dir_entry *himax_proc_HSEN_file = NULL;
#endif

#if defined(HX_PALM_REPORT)
static int himax_palm_detect(uint8_t *buf)
{
	struct himax_ts_data *ts = private_ts;
	int32_t loop_i;
	int base = 0;
	int x = 0, y = 0, w = 0;

	loop_i = 0;
	base = loop_i * 4;
	x = buf[base] << 8 | buf[base + 1];
	y = (buf[base + 2] << 8 | buf[base + 3]);
	w = buf[(ts->nFinger_support * 4) + loop_i];
#ifndef HIMAX_PALM_SENSOR_EN
	I(" %s HX_PALM_REPORT_loopi=%d,base=%x,X=%x,Y=%x,W=%x\n", __func__, loop_i, base, x, y, w);
	if ((!atomic_read(&ts->suspend_mode)) && (x == 0xFA5A) && (y == 0xFA5A) && (w == 0x00))
		return PALM_REPORT;
	else
		return NOT_REPORT;
#else
	if ((x == 0xFA5A) && (y == 0xFA5A)) {
		I("%s: HX_PALM_REPORT loop_i=%d base=%x X=%x, Y=%x, W=%x\n",
			__func__, loop_i, base, x, y, w);
		return PALM_REPORT;
	} else if ((x == 0xFC5C) && (y == 0xFC5C)) {
		I("%s: HX_PALM_REPORT loop_i=%d base=%x X=%x, Y=%x, W=%x\n",
			__func__, loop_i, base, x, y, w);
		return PALM_LEAVE_REPORT;
	} else
		return NOT_REPORT;
#endif
}
#endif

static ssize_t himax_self_test_write(struct file *file, const char *buff,
									size_t len, loff_t *pos)
{
	int i = 0;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (hx_self_test_file_name == NULL) {
		E("file name is NULL\n");
		hx_self_test_file_name = kzalloc(80, GFP_KERNEL);
		snprintf(hx_self_test_file_name, 15, "hx_criteria.csv");
	}

	for (i = 0; i < 80; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(hx_self_test_file_name, 0x0, 80);
			memcpy(hx_self_test_file_name, buf, i);
			I("%s: Get name from Customer\n", __func__);
			break;
		}
	}
	if (i == 80) {
		memset(hx_self_test_file_name, 0x0, 80);
		snprintf(hx_self_test_file_name, 16, "hx_criteria.csv");
		I("%s: Use default name\n", __func__);
		}
	I("file name = %s\n", hx_self_test_file_name);

	return len;
}

static ssize_t himax_self_test_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	int val = 0x00;
	size_t ret = 0;
	char *temp_buf;
	I("%s: enter, %d \n", __func__, __LINE__);

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		himax_int_enable(0);/* disable irq */
		private_ts->in_self_test = 1;
		val = g_core_fp.fp_chip_self_test();
#ifdef HX_ESD_RECOVERY
		HX_ESD_RESET_ACTIVATE = 1;
#endif
		himax_int_enable(1);/* enable irq */

		if (val == 0x00) {
			ret += snprintf(temp_buf + ret, len - ret, "Self_Test Pass\n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret, "Self_Test Fail\n");
		}

		private_ts->in_self_test = 0;

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_self_test_ops = {
	.owner = THIS_MODULE,
	.read = himax_self_test_read,
	.write = himax_self_test_write,
};

static void *himax_self_raw_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1) {
		return NULL;
	}
	return (void *)((unsigned long) *pos + 1);
}

static void *himax_self_raw_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void himax_self_raw_seq_stop(struct seq_file *s, void *v)
{
}

static int himax_self_raw_seq_read(struct seq_file *s, void *v)
{
	size_t ret = 0;

	if (g_rslt_data != NULL) {
			seq_printf(s, "%s", g_rslt_data);
		}
	return ret;
}
static struct seq_operations himax_self_raw_seq_ops = {
	.start	= himax_self_raw_seq_start,
	.next	= himax_self_raw_seq_next,
	.stop	= himax_self_raw_seq_stop,
	.show	= himax_self_raw_seq_read,
};
static int himax_self_raw_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_self_raw_seq_ops);
};

static struct file_operations himax_proc_self_raw_ops = {
	.owner = THIS_MODULE,
	.open = himax_self_raw_proc_open,
	.read = seq_read,
};

#if 0
static ssize_t himax_self_test(struct seq_file *s, void *v)
{
	int val = 0x00;
	size_t ret = 0;

	I("%s: enter, %d\n", __func__, __LINE__);

	if (private_ts->suspended == 1) {
		E("%s: please do self test in normal active mode\n", __func__);
		return HX_INIT_FAIL;
	}

	himax_int_enable(0);/* disable irq */

	private_ts->in_self_test = 1;

	val = g_core_fp.fp_chip_self_test();
/*
 *#ifdef HX_ESD_RECOVERY
 *	HX_ESD_RESET_ACTIVATE = 1;
 *#endif
 *	himax_int_enable(1); //enable irq
 */
	if (val == HX_INSPECT_OK)
		seq_puts(s, "Self_Test Pass:\n");
	else
		seq_puts(s, "Self_Test Fail:\n");

	/*Print_Self_Test_Info(val,temp_buf);*/
	if ((val & HX_INSPECT_EFILE) == HX_INSPECT_EFILE) {
		seq_puts(s, "  Get criteria File Fail\n");
		goto END_FUNC;
	} else if ((val & HX_INSPECT_MEMALLCTFAIL) == HX_INSPECT_MEMALLCTFAIL) {
		seq_puts(s, "  Allocate memory Fail\n");
		goto END_FUNC;
	} else {
		seq_puts(s, "  Get criteria File OK\n");
	}
	if ((val & HX_INSPECT_EGETRAW) == HX_INSPECT_EGETRAW) {
		seq_puts(s, "  Get raw data Fail\n");
		goto END_FUNC;
	} else {
		seq_puts(s, "  Get raw data OK\n");
	}
#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
	if (g_inspt_crtra_flag[IDX_RAWMIN] == 1 && g_inspt_crtra_flag[IDX_RAWMAX] == 1) {
		if ((val & HX_INSPECT_ERAW) == HX_INSPECT_ERAW)
			seq_puts(s, "  Check Raw data Fail\n");
		else
			seq_puts(s, "  Check Raw data OK\n");
	}
	if (g_inspt_crtra_flag[IDX_WT_NOISEMAX] == 1 && g_inspt_crtra_flag[IDX_WT_NOISEMIN] == 1) {
		if ((val & HX_INSPECT_WT_ENOISE) == HX_INSPECT_WT_ENOISE)
			seq_puts(s, "  Check Noise(weight) Fail\n");
		else
			seq_puts(s, "  Check Noise(weight) OK\n");
	}
	if (g_inspt_crtra_flag[IDX_ABS_NOISEMAX] == 1 && g_inspt_crtra_flag[IDX_ABS_NOISEMIN] == 1) {
		if ((val & HX_INSPECT_ABS_ENOISE) == HX_INSPECT_ABS_ENOISE)
			seq_puts(s, "  Check Noise(absolute) Fail\n");
		else
			seq_puts(s, "  Check Noise(absolute) OK\n");
	}
	if (g_inspt_crtra_flag[IDX_OPENMIN] == 1 && g_inspt_crtra_flag[IDX_OPENMAX] == 1) {
		if ((val & HX_INSPECT_EOPEN) == HX_INSPECT_EOPEN)
			seq_puts(s, "  Check Sensor open Fail\n");
		else
			seq_puts(s, "  Check Sensor open OK\n");
	}
	if (g_inspt_crtra_flag[IDX_M_OPENMIN] == 1 && g_inspt_crtra_flag[IDX_M_OPENMAX] == 1) {
		if ((val & HX_INSPECT_EMOPEN) == HX_INSPECT_EMOPEN)
			seq_puts(s, "  Check Sensor micro open Fail\n");
		else
			seq_puts(s, "  Check Sensor micro open OK\n");
	}
	if (g_inspt_crtra_flag[IDX_SORTMIN] == 1 && g_inspt_crtra_flag[IDX_SORTMAX] == 1) {
		if ((val & HX_INSPECT_ESHORT) == HX_INSPECT_ESHORT)
			seq_puts(s, "  Check Sensor short Fail\n");
		else
			seq_puts(s, "  Check Sensor short OK\n");
	}
	if ((g_inspt_crtra_flag[IDX_GAP_HOR_RAWMIN] == 1 && g_inspt_crtra_flag[IDX_GAP_HOR_RAWMAX] == 1) ||
		(g_inspt_crtra_flag[IDX_GAP_VER_RAWMIN] == 1 && g_inspt_crtra_flag[IDX_GAP_VER_RAWMAX] == 1)) {
		if ((val & HX_INSPECT_EGAP_RAW) == HX_INSPECT_EGAP_RAW)
			seq_puts(s, "  Check Raw Data GAP Fail\n");
		else
			seq_puts(s, "  Check Raw Data GAP OK\n");
	}
	if (g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MIN] == 1 && g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MAX] == 1) {
		if ((val & HX_INSPECT_EACT_IDLE_RAW) == HX_INSPECT_EACT_IDLE_RAW)
			seq_puts(s, "  Check ACT_IDLE RAW Fail\n");
		else
			seq_puts(s, "  Check ACT_IDLE RAW OK\n");
	}
	if (g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MIN] == 1 && g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MAX] == 1) {
		if ((val & HX_INSPECT_EACT_IDLE_NOISE) == HX_INSPECT_EACT_IDLE_NOISE)
			seq_puts(s, "  Check ACT_IDLE NOISE Fail\n");
		else
			seq_puts(s, "  Check ACT_IDLE NOISE OK\n");
	}
	if (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1 && g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1) {
		if ((val & HX_INSPECT_ELPWUG_RAW) == HX_INSPECT_ELPWUG_RAW)
			seq_puts(s, "  Check LPWUG RAW  Fail\n");
		else
			seq_puts(s, "  Check LPWUG RAW  OK\n");
	}
	if (g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1 && g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1) {
		if ((val & HX_INSPECT_ELPWUG_NOISE) == HX_INSPECT_ELPWUG_NOISE)
			seq_puts(s, "  Check LPWUG NOISE Fail\n");
		else
			seq_puts(s, "  Check LPWUG NOISE OK\n");
	}
	if (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1 && g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1) {
		if ((val & HX_INSPECT_ELPWUG_IDLE_RAW) == HX_INSPECT_ELPWUG_IDLE_RAW)
			seq_puts(s, "  Check LPWUG IDLE RAW Fail\n");
		else
			seq_puts(s, "  Check LPWUG IDLE RAW OK\n");
	}
	if (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1 && g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1) {
		if ((val & HX_INSPECT_ELPWUG_IDLE_NOISE) == HX_INSPECT_ELPWUG_IDLE_NOISE)
			seq_puts(s, "  Check LPWUG IDLE NOISE Fail\n");
		else
			seq_puts(s, "  Check LPWUG IDLE NOISE OK\n");
	}
#endif

END_FUNC:
	private_ts->in_self_test = 0;

#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
	if (g_inspt_crtra_flag != NULL) {
		kfree(g_inspt_crtra_flag);
		g_inspt_crtra_flag = NULL;
	}
#endif

#ifdef HX_ESD_RECOVERY
	HX_ESD_RESET_ACTIVATE = 1;
#endif
	himax_int_enable(1);/* enable irq */

	return ret;
}

static void *himax_self_test_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;


	return (void *)((unsigned long) *pos + 1);
}

static void *himax_self_test_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void himax_self_test_seq_stop(struct seq_file *s, void *v)
{
}

static ssize_t himax_self_test_write(struct file *filp, const char __user *buff,
			size_t len, loff_t *data)
{
	char buf[80];

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == 'r') {
		chip_test_r_flag = true;
		I("%s: Start to read chip test data.\n", __func__);
	}	else {
		chip_test_r_flag = false;
		I("%s: Back to do self test.\n", __func__);
	}

	return len;
}

static int himax_self_test_seq_read(struct seq_file *s, void *v)
{
	size_t ret = 0;

	if (chip_test_r_flag) {
#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
		if (g_rslt_data)
			seq_printf(s, "%s", g_rslt_data);
		else
#endif
			seq_puts(s, "No chip test data.\n");
	} else {
		himax_self_test(s, v);
	}

	return ret;
}

static const struct seq_operations himax_self_test_seq_ops = {
	.start	= himax_self_test_seq_start,
	.next	= himax_self_test_seq_next,
	.stop	= himax_self_test_seq_stop,
	.show	= himax_self_test_seq_read,
};

static int himax_self_test_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_self_test_seq_ops);
};

static const struct file_operations himax_proc_self_test_ops = {
	.owner = THIS_MODULE,
	.open = himax_self_test_proc_open,
	.read = seq_read,
	.write = himax_self_test_write,
	.release = seq_release,
};
#endif

#ifdef HX_HIGH_SENSE
static ssize_t himax_HSEN_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->HSEN_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_HSEN_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0')
		ts->HSEN_enable = 0;
	else if (buf[0] == '1')
		ts->HSEN_enable = 1;
	else
		return -EINVAL;

	g_core_fp.fp_set_HSEN_enable(ts->HSEN_enable, ts->suspended);
	I("%s: HSEN_enable = %d.\n", __func__, ts->HSEN_enable);
	return len;
}

static const struct file_operations himax_proc_HSEN_ops = {
	.owner = THIS_MODULE,
	.read = himax_HSEN_read,
	.write = himax_HSEN_write,
};
#endif

#ifdef HX_SMART_WAKEUP
static ssize_t himax_SMWP_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = private_ts;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_SMWP_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0')
		ts->SMWP_enable = 0;
	else if (buf[0] == '1')
		ts->SMWP_enable = 1;
	else
		return -EINVAL;

	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);
	return len;
}

static const struct file_operations himax_proc_SMWP_ops = {
	.owner = THIS_MODULE,
	.read = himax_SMWP_read,
	.write = himax_SMWP_write,
};

static ssize_t himax_GESTURE_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0;
	size_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);

		for (i = 0; i < GEST_SUP_NUM; i++)
			ret += snprintf(temp_buf + ret, len - ret, "ges_en[%d]=%d\n", i, ts->gesture_cust_en[i]);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
		ret = 0;
	}

	return ret;
}

static ssize_t himax_GESTURE_write(struct file *file, const char *buff,
								   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0;
	int j = 0;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	I("himax_GESTURE_store= %s, len = %d\n", buf, (int)len);

	for (i = 0; i < len; i++) {
		if (buf[i] == '0' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 0;
			I("gesture en[%d]=%d\n", j, ts->gesture_cust_en[j]);
			j++;
		}	else if (buf[i] == '1' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 1;
			I("gesture en[%d]=%d\n", j, ts->gesture_cust_en[j]);
			j++;
		}	else
			I("Not 0/1 or >=GEST_SUP_NUM : buf[%d] = %c\n", i, buf[i]);
	}

	return len;
}

static const struct file_operations himax_proc_Gesture_ops = {
	.owner = THIS_MODULE,
	.read = himax_GESTURE_read,
	.write = himax_GESTURE_write,
};

#ifdef HX_P_SENSOR
static ssize_t himax_psensor_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = private_ts;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kcalloc(len, sizeof(char), GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "p-sensor flag = %d\n", ts->psensor_flag);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_psensor_write(struct file *file, const char *buff,
								   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0' && ts->SMWP_enable == 1) {
		ts->psensor_flag = false;
		g_core_fp.fp_black_gest_ctrl(false);
	}	else if (buf[0] == '1' && ts->SMWP_enable == 1) {
		ts->psensor_flag = true;
		g_core_fp.fp_black_gest_ctrl(true);
	} else if (ts->SMWP_enable == 0) {
		I("%s: SMWP is disable, not supprot to ctrl p-sensor.\n", __func__);
	}	else
		return -EINVAL;

	I("%s: psensor_flag = %d.\n", __func__, ts->psensor_flag);
	return len;
}

static const struct file_operations himax_proc_psensor_ops = {
	.owner = THIS_MODULE,
	.read = himax_psensor_read,
	.write = himax_psensor_write,
};
#endif
#endif

#ifdef HX_EDGE_LIMIT
static ssize_t himax_edge_limit_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = private_ts;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->edge_limit_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_edge_limit_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0')
		ts->edge_limit_enable = 0;
	else if (buf[0] == '1')
		ts->edge_limit_enable = 1;/*Portrait*/
	else if (buf[0] == '2')
		ts->edge_limit_enable = 2;/*Landscape-Right*/
	else if (buf[0] == '3')
		ts->edge_limit_enable = 3;/*Landscape-Left*/
	else
		return -EINVAL;

	g_core_fp.fp_set_edge_limit_enable(ts->edge_limit_enable, ts->suspended);
	I("%s: edge_limit_enable = %d.\n", __func__, ts->edge_limit_enable);
	return len;
}

static struct file_operations himax_proc_edge_limit_ops = {
	.owner = THIS_MODULE,
	.read = himax_edge_limit_read,
	.write = himax_edge_limit_write,
};
#endif

int himax_common_proc_init(void)
{
	himax_touch_proc_dir = proc_mkdir(HIMAX_PROC_TOUCH_FOLDER, NULL);

	if (himax_touch_proc_dir == NULL) {
		E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
	if (fp_himax_self_test_init != NULL)
		fp_himax_self_test_init();
#endif

	himax_proc_self_test_file = proc_create(HIMAX_PROC_SELF_TEST_FILE, 0664, himax_touch_proc_dir, &himax_proc_self_test_ops);
	if (himax_proc_self_test_file == NULL) {
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_1;
	}

	himax_proc_self_raw_file = proc_create(HIMAX_PROC_SELF_TEST_RAW_FILE, 0444, himax_touch_proc_dir, &himax_proc_self_raw_ops);
	if (himax_proc_self_raw_file == NULL) {
		E(" %s: proc self_test raw file create failed!\n", __func__);
		goto fail_1_1;
	}

#ifdef HX_HIGH_SENSE
	himax_proc_HSEN_file = proc_create(HIMAX_PROC_HSEN_FILE, 0666,
									   himax_touch_proc_dir, &himax_proc_HSEN_ops);

	if (himax_proc_HSEN_file == NULL) {
		E(" %s: proc HSEN file create failed!\n", __func__);
		goto fail_2;
	}

#endif
#ifdef HX_SMART_WAKEUP
	himax_proc_SMWP_file = proc_create(HIMAX_PROC_SMWP_FILE, 0666,
									   himax_touch_proc_dir, &himax_proc_SMWP_ops);

	if (himax_proc_SMWP_file == NULL) {
		E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_GESTURE_file = proc_create(HIMAX_PROC_GESTURE_FILE, 0666,
										  himax_touch_proc_dir, &himax_proc_Gesture_ops);

	if (himax_proc_GESTURE_file == NULL) {
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_4;
	}
#ifdef HX_P_SENSOR
	himax_proc_psensor_file = proc_create(HIMAX_PROC_PSENSOR_FILE, 0666,
										  himax_touch_proc_dir, &himax_proc_psensor_ops);

	if (himax_proc_psensor_file == NULL) {
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_5;
	}
#endif
#endif
#ifdef HX_EDGE_LIMIT
	himax_proc_edge_limit_file = proc_create(HIMAX_PROC_EDGE_LIMIT_FILE, 0666,
									   himax_touch_proc_dir, &himax_proc_edge_limit_ops);

	if (himax_proc_edge_limit_file == NULL) {
		E(" %s: proc edge limit file create failed!\n", __func__);
		goto fail_6;
	}
#endif
	return 0;
#ifdef HX_EDGE_LIMIT
	remove_proc_entry(HIMAX_PROC_EDGE_LIMIT_FILE, himax_touch_proc_dir);
fail_6:
#endif
#ifdef HX_SMART_WAKEUP
#ifdef HX_P_SENSOR
	remove_proc_entry(HIMAX_PROC_PSENSOR_FILE, himax_touch_proc_dir);
fail_5:
#endif
	remove_proc_entry(HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir);
fail_4:
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
fail_3:
#endif
#ifdef HX_HIGH_SENSE
	remove_proc_entry(HIMAX_PROC_HSEN_FILE, himax_touch_proc_dir);
fail_2:
#endif
	remove_proc_entry(HIMAX_PROC_SELF_TEST_RAW_FILE, himax_touch_proc_dir);
fail_1_1:
	remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
fail_1:
	return -ENOMEM;
}

void himax_common_proc_deinit(void)
{
remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
remove_proc_entry(HIMAX_PROC_SELF_TEST_RAW_FILE, himax_touch_proc_dir);

#ifdef HX_EDGE_LIMIT
	remove_proc_entry(HIMAX_PROC_EDGE_LIMIT_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_HIGH_SENSE
	remove_proc_entry(HIMAX_PROC_HSEN_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_SMART_WAKEUP
#ifdef HX_P_SENSOR
	remove_proc_entry(HIMAX_PROC_PSENSOR_FILE, himax_touch_proc_dir);
#endif
	remove_proc_entry(HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
#endif
	remove_proc_entry(HIMAX_PROC_TOUCH_FOLDER, NULL);
}

/* File node for SMWP and HSEN - End*/

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;
#if defined(HX_SMART_WAKEUP)
	int i = 0;
#endif
	ret = himax_dev_set(ts);

	if (ret < 0) {
		I("%s, input device register fail!\n", __func__);
		ret = INPUT_REGISTER_FAIL;
		goto input_device_fail;
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
#if defined(HX_PLATFOME_DEFINE_KEY)
	himax_platform_key();
#else
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP) || defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP)
	for (i = 1; i < GEST_SUP_NUM; i++)
		set_bit(gest_key_def[i], ts->input_dev->keybit);

#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#ifdef	HX_PROTOCOL_A
	/*ts->input_dev->mtsize = ts->nFinger_support;*/
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 3, 0, 0);
#else
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
#if defined(HX_PROTOCOL_B_3PA)
	input_mt_init_slots(ts->input_dev, ts->nFinger_support, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(ts->input_dev, ts->nFinger_support);
#endif
#endif
	I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
	  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
#ifndef	HX_PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);
#endif
/*	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
 *	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);
 */

	if (himax_input_register_device(ts->input_dev) == 0)
		ret = NO_ERR;
	else
		ret = INPUT_REGISTER_FAIL;

#if defined(HX_PEN_FUNC_EN)
	set_bit(EV_SYN, ts->hx_pen_dev->evbit);
	set_bit(EV_ABS, ts->hx_pen_dev->evbit);
	set_bit(EV_KEY, ts->hx_pen_dev->evbit);
	set_bit(BTN_TOUCH, ts->hx_pen_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->hx_pen_dev->propbit);

	set_bit(BTN_TOOL_PEN, ts->hx_pen_dev->keybit);
	set_bit(BTN_TOOL_RUBBER, ts->hx_pen_dev->keybit);

	input_set_abs_params(ts->hx_pen_dev, ABS_PRESSURE, 0, 4095, 0, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_TILT_X, -60, 60, 0, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_TILT_Y, -60, 60, 0, 0);
	/*input_set_capability(ts->hx_pen_dev, EV_SW, SW_PEN_INSERT);*/
	input_set_capability(ts->hx_pen_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(ts->hx_pen_dev, EV_KEY, BTN_STYLUS);

	input_set_abs_params(ts->hx_pen_dev, ABS_X, ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_Y, ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);

	if (himax_input_register_device(ts->hx_pen_dev) == 0)
		ret = NO_ERR;
	else
		ret = INPUT_REGISTER_FAIL;

#endif
	I("%s, input device registered.\n", __func__);

input_device_fail:
	return ret;
}
EXPORT_SYMBOL(himax_input_register);

static void calcDataSize(void)
{
	struct himax_ts_data *ts_data = private_ts;

	ts_data->x_channel = ic_data->HX_RX_NUM;
	ts_data->y_channel = ic_data->HX_TX_NUM;
	ts_data->nFinger_support = ic_data->HX_MAX_PT;

	ts_data->coord_data_size = 4 * ts_data->nFinger_support;
	ts_data->area_data_size = ((ts_data->nFinger_support / 4) + (ts_data->nFinger_support % 4 ? 1 : 0)) * 4;
	ts_data->coordInfoSize = ts_data->coord_data_size + ts_data->area_data_size + 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;

	if (ts_data->raw_data_frame_size == 0) {
		E("%s: could NOT calculate!\n", __func__);
		return;
	}

	ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
									(((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size) ? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d\n", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = ic_data->HX_MAX_PT * 4;

	if ((ic_data->HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (ic_data->HX_MAX_PT / 4) * 4;
	else
		HX_TOUCH_INFO_POINT_CNT += ((ic_data->HX_MAX_PT / 4) + 1) * 4;
}
#if defined(HX_AUTO_UPDATE_FW)
static int himax_auto_update_check(void)
{
	int32_t ret;
	int flag_sys_ex_is_same = 0;

	I("%s:Entering!\n", __func__);
	if (g_core_fp.fp_fw_ver_bin() == 0) {
		flag_sys_ex_is_same = (g_core_fp._diff_overlay_flash() !=  g_core_fp._diff_overlay_bin()) ? 1 : 0;
		I("Now flag_sys_ex_is_same valur=%d\n", flag_sys_ex_is_same);
		if (((ic_data->vendor_fw_ver < g_i_FW_VER) || (ic_data->vendor_config_ver < g_i_CFG_VER))
		|| (flag_sys_ex_is_same == 1)) {
			I("Need to update!\n");
			ret = NO_ERR;
		} else {
			E("No need to update!\n");
			ret = 1;
		}
	} else {
		E("FW bin fail!\n");
		ret = 1;
	}

	return ret;
}

static int i_get_FW(void)
{
	int ret = 0;
	const struct firmware *image = NULL;

	I("file name = %s\n", i_CTPM_firmware_name);
	ret = request_firmware(&image, i_CTPM_firmware_name, private_ts->dev);
	if (ret < 0) {
#if defined(__EMBEDDED_FW__)
		image = &g_embedded_fw;
		I("%s: Couldn't find userspace FW, use embedded FW(size:%zu) instead.\n", __func__, g_embedded_fw.size);
#else
		E("%s,fail in line%d error code=%d\n", __func__, __LINE__, ret);
		return OPEN_FILE_FAIL;
#endif
	}

	if (image != NULL) {
		i_CTPM_FW_len = image->size;
		i_CTPM_FW = kcalloc(i_CTPM_FW_len, sizeof(char), GFP_KERNEL);
		memcpy(i_CTPM_FW, image->data, sizeof(char)*i_CTPM_FW_len);
	} else {
		I("%s: i_CTPM_FW = NULL\n", __func__);
		return OPEN_FILE_FAIL;
	}

	if (ret >= 0)
		release_firmware(image);
	ret = NO_ERR;
	return ret;
}
static int i_update_FW(void)
{
	int upgrade_times = 0;
	uint8_t ret = 0, result = 0;

	himax_int_enable(0);


update_retry:

	if (i_CTPM_FW_len == FW_SIZE_32k)
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_32k(i_CTPM_FW, i_CTPM_FW_len, false);
	else if (i_CTPM_FW_len == FW_SIZE_60k)
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_60k(i_CTPM_FW, i_CTPM_FW_len, false);
	else if (i_CTPM_FW_len == FW_SIZE_64k)
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k(i_CTPM_FW, i_CTPM_FW_len, false);
	else if (i_CTPM_FW_len == FW_SIZE_124k)
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_124k(i_CTPM_FW, i_CTPM_FW_len, false);
	else if (i_CTPM_FW_len == FW_SIZE_128k)
		ret = g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_128k(i_CTPM_FW, i_CTPM_FW_len, false);

	if (ret == 0) {
		upgrade_times++;
		E("%s: TP upgrade error, upgrade_times = %d\n", __func__, upgrade_times);

		if (upgrade_times < 3)
			goto update_retry;
		else
			result = -1;


	} else {
		g_core_fp.fp_read_FW_ver();
		g_core_fp.fp_touch_information();
		result = 1;/*upgrade success*/
		I("%s: TP upgrade OK\n", __func__);
	}

	kfree(i_CTPM_FW);
	i_CTPM_FW = NULL;

#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(true, false);
#else
	g_core_fp.fp_sense_on(0x00);
#endif
	himax_int_enable(1);
	return result;
}
#endif


static int himax_loadSensorConfig(struct himax_i2c_platform_data *pdata)
{
	I("%s: initialization complete\n", __func__);
	return NO_ERR;
}

#ifdef HX_ESD_RECOVERY
static void himax_esd_hw_reset(void)
{
#ifdef HX_ZERO_FLASH
	int result = 0;
#endif
	if (g_ts_dbg != 0)
		I("%s: Entering\n", __func__);

	I("START_Himax TP: ESD - Reset\n");

	if (private_ts->in_self_test == 1) {
		I("In self test , not  TP: ESD - Reset\n");
		return;
	}

	g_core_fp.fp_esd_ic_reset();
#ifdef HX_ZERO_FLASH
	I("It will update fw after esd event in zero flash mode!\n");
	result = g_core_fp.fp_0f_operation_dirly();
	if (result) {
		E("Something is wrong! Skip Update with zero flash!\n");
		goto ESCAPE_0F_UPDATE;
	}
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_sense_on(0x00);
	himax_report_all_leave_event(private_ts);
	himax_int_enable(1);
ESCAPE_0F_UPDATE:
#endif
	I("END_Himax TP: ESD - Reset\n");
}
#endif

#ifdef HX_SMART_WAKEUP
#ifdef HX_GESTURE_TRACK
static void gest_pt_log_coordinate(int rx, int tx)
{
	/*driver report x y with range 0 - 255 , we scale it up to x/y pixel*/
	gest_pt_x[gest_pt_cnt] = rx * (ic_data->HX_X_RES) / 255;
	gest_pt_y[gest_pt_cnt] = tx * (ic_data->HX_Y_RES) / 255;
}
#endif
static int himax_wake_event_parse(struct himax_ts_data *ts, int ts_status)
{
	uint8_t *buf = wake_event_buffer;
#ifdef HX_GESTURE_TRACK
	int tmp_max_x = 0x00, tmp_min_x = 0xFFFF, tmp_max_y = 0x00, tmp_min_y = 0xFFFF;
	int gest_len;
#endif
	int i = 0, check_FC = 0, ret;
	int j = 0, gesture_pos = 0, gesture_flag = 0;

	if (g_ts_dbg != 0)
		I("%s: Entering!, ts_status=%d\n", __func__, ts_status);

	if (buf == NULL) {
		ret = -ENOMEM;
		goto END;
	}

	memcpy(buf, hx_touch_data->hx_event_buf, hx_touch_data->event_size);

	for (i = 0; i < GEST_PTLG_ID_LEN; i++) {
		for (j = 0; j < GEST_SUP_NUM; j++) {
			if (buf[i] == gest_event[j]) {
				gesture_flag = buf[i];
				gesture_pos = j;
				break;
			}
		}
		I("0x%2.2X ", buf[i]);
		if (buf[i] == gesture_flag) {
			check_FC++;
		} else {
			I("ID START at %x , value = 0x%2X skip the event\n", i, buf[i]);
			break;
		}
	}

	I("Himax gesture_flag= %x\n", gesture_flag);
	I("Himax check_FC is %d\n", check_FC);

	if (check_FC != GEST_PTLG_ID_LEN) {
		ret = 0;
		goto END;
	}

	if (buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
		buf[GEST_PTLG_ID_LEN + 1] != GEST_PTLG_HDR_ID2) {
		ret = 0;
		goto END;
	}

#ifdef HX_GESTURE_TRACK

	if (buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
		buf[GEST_PTLG_ID_LEN + 1] == GEST_PTLG_HDR_ID2) {
		gest_len = buf[GEST_PTLG_ID_LEN + 2];
		I("gest_len = %d\n", gest_len);
		i = 0;
		gest_pt_cnt = 0;
		I("gest doornidate start\n %s", __func__);

		while (i < (gest_len + 1) / 2) {
			gest_pt_log_coordinate(buf[GEST_PTLG_ID_LEN + 4 + i * 2], buf[GEST_PTLG_ID_LEN + 4 + i * 2 + 1]);
			i++;
			I("gest_pt_x[%d]=%d\n", gest_pt_cnt, gest_pt_x[gest_pt_cnt]);
			I("gest_pt_y[%d]=%d\n", gest_pt_cnt, gest_pt_y[gest_pt_cnt]);
			gest_pt_cnt += 1;
		}

		if (gest_pt_cnt) {
			for (i = 0; i < gest_pt_cnt; i++) {
				if (tmp_max_x < gest_pt_x[i])
					tmp_max_x = gest_pt_x[i];
				if (tmp_min_x > gest_pt_x[i])
					tmp_min_x = gest_pt_x[i];
				if (tmp_max_y < gest_pt_y[i])
					tmp_max_y = gest_pt_y[i];
				if (tmp_min_y > gest_pt_y[i])
					tmp_min_y = gest_pt_y[i];
			}

			I("gest_point x_min= %d, x_max= %d, y_min= %d, y_max= %d\n", tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y);
			gest_start_x = gest_pt_x[0];
			hx_gesture_coor[0] = gest_start_x;
			gest_start_y = gest_pt_y[0];
			hx_gesture_coor[1] = gest_start_y;
			gest_end_x = gest_pt_x[gest_pt_cnt - 1];
			hx_gesture_coor[2] = gest_end_x;
			gest_end_y = gest_pt_y[gest_pt_cnt - 1];
			hx_gesture_coor[3] = gest_end_y;
			gest_width = tmp_max_x - tmp_min_x;
			hx_gesture_coor[4] = gest_width;
			gest_height = tmp_max_y - tmp_min_y;
			hx_gesture_coor[5] = gest_height;
			gest_mid_x = (tmp_max_x + tmp_min_x) / 2;
			hx_gesture_coor[6] = gest_mid_x;
			gest_mid_y = (tmp_max_y + tmp_min_y) / 2;
			hx_gesture_coor[7] = gest_mid_y;
			hx_gesture_coor[8] = gest_mid_x;/*gest_up_x*/
			hx_gesture_coor[9] = gest_mid_y - gest_height / 2; /*gest_up_y*/
			hx_gesture_coor[10] = gest_mid_x;/*gest_down_x*/
			hx_gesture_coor[11] = gest_mid_y + gest_height / 2;	/*gest_down_y*/
			hx_gesture_coor[12] = gest_mid_x - gest_width / 2;	/*gest_left_x*/
			hx_gesture_coor[13] = gest_mid_y;	/*gest_left_y*/
			hx_gesture_coor[14] = gest_mid_x + gest_width / 2;	/*gest_right_x*/
			hx_gesture_coor[15] = gest_mid_y; /*gest_right_y*/
		}
	}

#endif

	if (!ts->gesture_cust_en[gesture_pos]) {
		I("%s NOT report key [%d] = %d\n", __func__, gesture_pos, gest_key_def[gesture_pos]);
		g_target_report_data->SMWP_event_chk = 0;
		ret = 0;
	} else {
		g_target_report_data->SMWP_event_chk = gest_key_def[gesture_pos];
		ret = gesture_pos;
	}
END:
	return ret;
}

static void himax_wake_event_report(void)
{
	int KEY_EVENT = g_target_report_data->SMWP_event_chk;
#ifdef HIMAX_V2_SENSOR_EN
	static int report_cnt = 0;
#endif
	if (g_ts_dbg != 0)
		I("%s: Entering!\n", __func__);

	if (KEY_EVENT) {
#ifdef HIMAX_V2_SENSOR_EN
		input_report_abs(private_ts->sensor_pdata->input_sensor_dev, ABS_DISTANCE, ++report_cnt);
		I("input report: %d", report_cnt);
		if (report_cnt >= REPORT_MAX_COUNT)
			report_cnt = 0;
		input_sync(private_ts->sensor_pdata->input_sensor_dev);
#else
		I(" %s SMART WAKEUP KEY event %d press\n", __func__, KEY_EVENT);
		input_report_key(private_ts->input_dev, KEY_EVENT, 1);
		input_sync(private_ts->input_dev);
		I(" %s SMART WAKEUP KEY event %d release\n", __func__, KEY_EVENT);
		input_report_key(private_ts->input_dev, KEY_EVENT, 0);
		input_sync(private_ts->input_dev);
#endif
		FAKE_POWER_KEY_SEND = true;
#ifdef HX_GESTURE_TRACK
		I("gest_start_x= %d, gest_start_y= %d, gest_end_x= %d, gest_end_y= %d\n", gest_start_x, gest_start_y,
		  gest_end_x, gest_end_y);
		I("gest_width= %d, gest_height= %d, gest_mid_x= %d, gest_mid_y= %d\n", gest_width, gest_height,
		  gest_mid_x, gest_mid_y);
		I("gest_up_x= %d, gest_up_y= %d, gest_down_x= %d, gest_down_y= %d\n", hx_gesture_coor[8], hx_gesture_coor[9],
		  hx_gesture_coor[10], hx_gesture_coor[11]);
		I("gest_left_x= %d, gest_left_y= %d, gest_right_x= %d, gest_right_y= %d\n", hx_gesture_coor[12], hx_gesture_coor[13],
		  hx_gesture_coor[14], hx_gesture_coor[15]);
#endif
		g_target_report_data->SMWP_event_chk = 0;
	}
}

#endif

int himax_report_data_init(void)
{
	if (hx_touch_data->hx_coord_buf != NULL) {
		kfree(hx_touch_data->hx_coord_buf);
		hx_touch_data->hx_coord_buf = NULL;
	}

	if (hx_touch_data->hx_rawdata_buf != NULL) {
		kfree(hx_touch_data->hx_rawdata_buf);
		hx_touch_data->hx_rawdata_buf = NULL;
	}

#if defined(HX_SMART_WAKEUP)
	hx_touch_data->event_size = g_core_fp.fp_get_touch_data_size();

	if (hx_touch_data->hx_event_buf != NULL) {
		kfree(hx_touch_data->hx_event_buf);
		hx_touch_data->hx_event_buf = NULL;
	}

	if (wake_event_buffer != NULL) {
		kfree(wake_event_buffer);
		wake_event_buffer = NULL;
	}

#endif
	hx_touch_data->touch_all_size = g_core_fp.fp_get_touch_data_size();
	hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT / 4;
	hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT % 4;
	/* more than 4 fingers */
	if (hx_touch_data->raw_cnt_rmd != 0x00) {
		hx_touch_data->rawdata_size = g_core_fp.fp_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 2) * 4;
	} else { /* less than 4 fingers */
		hx_touch_data->rawdata_size = g_core_fp.fp_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 1) * 4;
	}
#if defined(HX_PEN_FUNC_EN)
	hx_touch_data->touch_info_size += PEN_INFO_SZ;
#endif
	if ((ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) % hx_touch_data->rawdata_size == 0)
		hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size;
	else
		hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size + 1;

	I("%s: rawdata_frame_size = %d\n", __func__, hx_touch_data->rawdata_frame_size);
	I("%s: ic_data->HX_MAX_PT:%d, hx_raw_cnt_max:%d, hx_raw_cnt_rmd:%d, g_hx_rawdata_size:%d, hx_touch_data->touch_info_size:%d\n", __func__, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max, hx_touch_data->raw_cnt_rmd, hx_touch_data->rawdata_size, hx_touch_data->touch_info_size);
	hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_coord_buf == NULL)
		goto mem_alloc_fail;

	if (g_target_report_data == NULL) {
		g_target_report_data = kzalloc(sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (g_target_report_data == NULL)
			goto mem_alloc_fail;
		g_target_report_data->x = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->x == NULL)
			goto mem_alloc_fail;
		g_target_report_data->y = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->y == NULL)
			goto mem_alloc_fail;
		g_target_report_data->w = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->w == NULL)
			goto mem_alloc_fail;
		g_target_report_data->finger_id = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->finger_id == NULL)
			goto mem_alloc_fail;
#if defined(HX_PEN_FUNC_EN)
		g_target_report_data->p_x = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_x == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_y = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_y == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_w = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_w == NULL)
			goto mem_alloc_fail;
		g_target_report_data->pen_id = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->pen_id == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_hover = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_hover == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_tilt_x = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_tilt_x == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_btn = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_btn == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_btn2 = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_btn2 == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_tilt_y = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_tilt_y == NULL)
			goto mem_alloc_fail;
		g_target_report_data->p_on = kzalloc(sizeof(int)*2, GFP_KERNEL);
		if (g_target_report_data->p_on == NULL)
			goto mem_alloc_fail;

#endif
	}
#ifdef HX_SMART_WAKEUP
	g_target_report_data->SMWP_event_chk = 0;
	wake_event_buffer = kcalloc(hx_touch_data->event_size, sizeof(uint8_t), GFP_KERNEL);
	if (wake_event_buffer == NULL)
		goto mem_alloc_fail;
#endif

	hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_rawdata_buf == NULL)
		goto mem_alloc_fail;

#if defined(HX_SMART_WAKEUP)
	hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->event_size), GFP_KERNEL);

	if (hx_touch_data->hx_event_buf == NULL)
		goto mem_alloc_fail;

#endif
	return NO_ERR;
mem_alloc_fail:
#if defined(HX_PEN_FUNC_EN)
	kfree(g_target_report_data->p_on);
	g_target_report_data->p_on = NULL;
	kfree(g_target_report_data->p_tilt_y);
	g_target_report_data->p_tilt_y = NULL;
	kfree(g_target_report_data->p_btn2);
	g_target_report_data->p_btn2 = NULL;
	kfree(g_target_report_data->p_btn);
	g_target_report_data->p_btn = NULL;
	kfree(g_target_report_data->p_tilt_x);
	g_target_report_data->p_tilt_x = NULL;
	kfree(g_target_report_data->p_hover);
	g_target_report_data->p_hover = NULL;
	kfree(g_target_report_data->pen_id);
	g_target_report_data->pen_id = NULL;
	kfree(g_target_report_data->p_w);
	g_target_report_data->p_w = NULL;
	kfree(g_target_report_data->p_y);
	g_target_report_data->p_y = NULL;
	kfree(g_target_report_data->p_x);
	g_target_report_data->p_x = NULL;
#endif
	kfree(g_target_report_data->finger_id);
	g_target_report_data->finger_id = NULL;
	kfree(g_target_report_data->w);
	g_target_report_data->w = NULL;
	kfree(g_target_report_data->y);
	g_target_report_data->y = NULL;
	kfree(g_target_report_data->x);
	g_target_report_data->x = NULL;
	kfree(g_target_report_data);
	g_target_report_data = NULL;

#if defined(HX_SMART_WAKEUP)
	kfree(wake_event_buffer);
	wake_event_buffer = NULL;
	kfree(hx_touch_data->hx_event_buf);
	hx_touch_data->hx_event_buf = NULL;
#endif
	kfree(hx_touch_data->hx_rawdata_buf);
	hx_touch_data->hx_rawdata_buf = NULL;
	kfree(hx_touch_data->hx_coord_buf);
	hx_touch_data->hx_coord_buf = NULL;

	I("%s: Memory allocate fail!\n", __func__);
	return MEM_ALLOC_FAIL;
}
EXPORT_SYMBOL(himax_report_data_init);

void himax_report_data_deinit(void)
{
#if defined(HX_PEN_FUNC_EN)
	kfree(g_target_report_data->p_on);
	g_target_report_data->p_on = NULL;
	kfree(g_target_report_data->p_tilt_y);
	g_target_report_data->p_tilt_y = NULL;
	kfree(g_target_report_data->p_btn2);
	g_target_report_data->p_btn2 = NULL;
	kfree(g_target_report_data->p_btn);
	g_target_report_data->p_btn = NULL;
	kfree(g_target_report_data->p_tilt_x);
	g_target_report_data->p_tilt_x = NULL;
	kfree(g_target_report_data->p_hover);
	g_target_report_data->p_hover = NULL;
	kfree(g_target_report_data->pen_id);
	g_target_report_data->pen_id = NULL;
	kfree(g_target_report_data->p_w);
	g_target_report_data->p_w = NULL;
	kfree(g_target_report_data->p_y);
	g_target_report_data->p_y = NULL;
	kfree(g_target_report_data->p_x);
	g_target_report_data->p_x = NULL;
#endif
	kfree(g_target_report_data->finger_id);
	g_target_report_data->finger_id = NULL;
	kfree(g_target_report_data->w);
	g_target_report_data->w = NULL;
	kfree(g_target_report_data->y);
	g_target_report_data->y = NULL;
	kfree(g_target_report_data->x);
	g_target_report_data->x = NULL;
	kfree(g_target_report_data);
	g_target_report_data = NULL;

#if defined(HX_SMART_WAKEUP)
	kfree(wake_event_buffer);
	wake_event_buffer = NULL;
	kfree(hx_touch_data->hx_event_buf);
	hx_touch_data->hx_event_buf = NULL;
#endif
	kfree(hx_touch_data->hx_rawdata_buf);
	hx_touch_data->hx_rawdata_buf = NULL;
	kfree(hx_touch_data->hx_coord_buf);
	hx_touch_data->hx_coord_buf = NULL;
}

/*start ts_work*/
#if defined(HX_USB_DETECT_GLOBAL)
void himax_cable_detect_func(bool force_renew)
{
	struct himax_ts_data *ts;

	/*u32 connect_status = 0;*/
	uint8_t connect_status = 0;

	connect_status = USB_detect_flag;/* upmu_is_chr_det(); */
	ts = private_ts;

	/* I("Touch: cable status=%d, cable_config=%p, usb_connected=%d\n", connect_status, ts->cable_config, ts->usb_connected); */
	if (ts->cable_config) {
		if ((connect_status != ts->usb_connected) || force_renew) {
			if (connect_status) {
				ts->cable_config[1] = 0x01;
				ts->usb_connected = 0x01;
			} else {
				ts->cable_config[1] = 0x00;
				ts->usb_connected = 0x00;
			}

			g_core_fp.fp_usb_detect_set(ts->cable_config);
			I("%s: Cable status change: 0x%2.2X\n", __func__, ts->usb_connected);
		}

		/*else */
		/*I("%s: Cable status is the same as previous one, ignore.\n", __func__); */
	}
}
#endif

static int himax_ts_work_status(struct himax_ts_data *ts)
{
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD;

	hx_touch_data->diag_cmd = ts->diag_cmd;
	if (hx_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

#ifdef HX_SMART_WAKEUP
	if (atomic_read(&ts->suspend_mode) && (ts->SMWP_enable) && (!hx_touch_data->diag_cmd))
		result = HX_REPORT_SMWP_EVENT;
#endif
	/* I("Now Status is %d\n", result); */
	return result;
}

static int himax_touch_get(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if ((HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
			|| (HX_ESD_RESET_ACTIVATE)
#endif
			) {
			if (!g_core_fp.fp_read_event_stack(buf, 128)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
			}
		} else {
			if (!g_core_fp.fp_read_event_stack(buf, hx_touch_data->touch_info_size)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
			}
		}
		break;
#if defined(HX_SMART_WAKEUP)

	/*SMWP*/
	case HX_REPORT_SMWP_EVENT:
		__pm_wakeup_event(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		msleep(20);
		g_core_fp.fp_burst_enable(0);

		if (!g_core_fp.fp_read_event_stack(buf, hx_touch_data->event_size)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
		}
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		if (!g_core_fp.fp_read_event_stack(buf, 128)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
		}
		break;
	default:
		break;
	}

	return ts_status;
}

/* start error_control*/
static int himax_checksum_cal(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	uint16_t check_sum_cal = 0;
	int32_t	i = 0;
	int length = 0;
	int zero_cnt = 0;
	int raw_data_sel = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		I("[HIMAX TP MSG] point data_checksum not match : check_sum_cal: 0x%02X\n", check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		if (ts->use_irq)
			I("[HIMAX TP MSG] All Zero event\n");

		ret_val = HX_CHKSUM_FAIL;
	} else {
				raw_data_sel = buf[HX_TOUCH_INFO_POINT_CNT]>>4 & 0x0F;
				/*I("%s:raw_out_sel=%x , hx_touch_data->diag_cmd=%x.\n", __func__, raw_data_sel, hx_touch_data->diag_cmd);*/
		if ((raw_data_sel != 0x0F) && (raw_data_sel != hx_touch_data->diag_cmd)) {/*raw data out not match skip it*/
			/*I("%s:raw data out not match.\n", __func__);*/
			if (!hx_touch_data->diag_cmd) {
				g_core_fp.fp_read_event_stack(buf, (128-hx_touch_data->touch_info_size));/*Need to clear event stack here*/
				/*I("%s: size =%d, buf[0]=%x ,buf[1]=%x, buf[2]=%x, buf[3]=%x.\n", __func__,(128-hx_touch_data->touch_info_size), buf[0], buf[1], buf[2], buf[3]);*/
				/*I("%s:also clear event stack.\n", __func__);*/
			}
			ret_val = HX_READY_SERVE;
		}
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);
	return ret_val;
}

#ifdef HX_ESD_RECOVERY
#ifdef HX_ZERO_FLASH
void hx_update_dirly_0f(void)
{
	I("It will update fw after esd event in zero flash mode!\n");
	g_core_fp.fp_0f_operation_dirly();
}
#endif
static int himax_ts_event_check(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	int hx_EB_event = 0;
	int hx_EC_event = 0;
	int hx_ED_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	int32_t	loop_i = 0;
	int length = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	if (g_ts_dbg != 0)
		I("Now Path=%d, Now status=%d, length=%d\n", ts_path, ts_status, length);

	for (loop_i = 0; loop_i < length; loop_i++) {
		if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_EB_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_EC_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ED_event++;
			} else if (buf[loop_i] == 0x00) { /* case 2 ESD recovery flow-Disable */
				hx_zero_event++;
			} else {
				hx_EB_event = 0;
				hx_EC_event = 0;
				hx_ED_event = 0;
				hx_zero_event = 0;
				g_zero_event_count = 0;
			}
		}
	}

	if (hx_EB_event == length) {
		hx_esd_event = length;
		hx_EB_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
	} else if (hx_EC_event == length) {
		hx_esd_event = length;
		hx_EC_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
	} else if (hx_ED_event == length) {
		hx_esd_event = length;
		hx_ED_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
	}
/*#ifdef HX_ZERO_FLASH
 *	//This is for previous version(a, b) because HW pull TSIX low continuely after watchdog timeout reset
 *	else if (hx_zero_event == length) {
 *		//check zero flash status
 *		if (g_core_fp.fp_0f_esd_check() < 0) {
 *			g_zero_event_count = 6;
 *			I("[HIMAX TP MSG]: ESD event checked - ALL Zero in ZF.\n");
 *		} else {
 *			I("[HIMAX TP MSG]: Status check pass in ZF.\n");
 *		}
 *	}
 *#endif
 */
	else
		hx_esd_event = 0;

	if ((hx_esd_event == length || hx_zero_event == length)
		&& (HX_HW_RESET_ACTIVATE == 0)
		&& (HX_ESD_RESET_ACTIVATE == 0)
		&& (hx_touch_data->diag_cmd == 0)
		&& (ts->in_self_test == 0)) {
		shaking_ret = g_core_fp.fp_ic_esd_recovery(hx_esd_event, hx_zero_event, length);

		if (shaking_ret == HX_ESD_EVENT) {
			himax_esd_hw_reset();
			ret_val = HX_ESD_EVENT;
		} else if (shaking_ret == HX_ZERO_EVENT_COUNT) {
			ret_val = HX_ZERO_EVENT_COUNT;
		} else {
			I("I2C running. Nothing to be done!\n");
			ret_val = HX_IC_RUNNING;
		}
	} else if (HX_ESD_RESET_ACTIVATE) { /* drop 1st interrupts after chip reset */
		HX_ESD_RESET_ACTIVATE = 0;
		I("[HX_ESD_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
		ret_val = HX_ESD_REC_OK;
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);

	return ret_val;
}
#endif

static int himax_err_ctrl(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
#ifdef HX_RST_PIN_FUNC
	if (HX_HW_RESET_ACTIVATE) {
		/* drop 1st interrupts after chip reset */
		HX_HW_RESET_ACTIVATE = 0;
		I("[HX_HW_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
		ts_status = HX_RST_OK;
		goto END_FUNCTION;
	}
#endif

	ts_status = himax_checksum_cal(ts, buf, ts_path, ts_status);
	if (ts_status == HX_CHKSUM_FAIL) {
		goto CHK_FAIL;
	} else {
#ifdef HX_ESD_RECOVERY
		/* continuous N times record, not total N times. */
		g_zero_event_count = 0;
#endif
		goto END_FUNCTION;
	}

CHK_FAIL:
#ifdef HX_ESD_RECOVERY
	ts_status = himax_ts_event_check(ts, buf, ts_path, ts_status);
#endif


END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int himax_distribute_touch_data(uint8_t *buf, int ts_path, int ts_status)
{
	uint8_t hx_state_info_pos = hx_touch_data->touch_info_size - 3;

#if defined(HX_PEN_FUNC_EN)
	hx_state_info_pos -= PEN_INFO_SZ;
#endif

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		else
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));

		if ((HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
		|| (HX_ESD_RESET_ACTIVATE)
#endif
		) {
			memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
		}
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		else
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));

		memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
#if defined(HX_SMART_WAKEUP)
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		memcpy(hx_touch_data->hx_event_buf, buf, hx_touch_data->event_size);
#endif
	} else {
		E("%s, Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: End, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end assign_data*/

/* start parse_report_data*/
int himax_parse_report_points(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	int x = 0, y = 0, w = 0;
#if defined(HX_PEN_FUNC_EN)
	uint8_t p_hover = 0, p_btn = 0, p_btn2 = 0;
	int8_t p_tilt_x = 0, p_tilt_y = 0;
	int p_x = 0, p_y = 0, p_w = 0;
	static uint8_t p_p_on;
#endif
	int base = 0;
	int32_t	loop_i = 0;

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

#if defined(HX_PEN_FUNC_EN)
	p_p_on = 0;
	base = hx_touch_data->touch_info_size - PEN_INFO_SZ;

	p_x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
	p_y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (hx_touch_data->hx_coord_buf[base + 4] << 8 | hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)hx_touch_data->hx_coord_buf[base + 6];
	p_hover = hx_touch_data->hx_coord_buf[base + 7];
	p_btn = hx_touch_data->hx_coord_buf[base + 8];
	p_btn2 = hx_touch_data->hx_coord_buf[base + 9];
	p_tilt_y = (int8_t)hx_touch_data->hx_coord_buf[base + 10];

	if (g_ts_dbg != 0) {
		D("%s: p_x=%d, p_y=%d, p_w=%d\n", __func__, p_x, p_y, p_w);
		D("%s: p_tilt_x=%d, p_hover=%d\n", __func__, p_tilt_x, p_hover);
		D("%s: p_btn=%d, p_btn2=%d\n", __func__, p_btn, p_btn2);
		D("%s: p_tilt_y=%d\n", __func__, p_tilt_y);
	}

	if (p_x >= 0 && p_x <= ts->pdata->abs_x_max && p_y >= 0 && p_y <= ts->pdata->abs_y_max) {
		g_target_report_data->p_x[0] = p_x;
		g_target_report_data->p_y[0] = p_y;
		g_target_report_data->p_w[0] = p_w;
		g_target_report_data->p_hover[0] = p_hover;
		g_target_report_data->pen_id[0] = 1;
		g_target_report_data->p_btn[0] = p_btn;
		g_target_report_data->p_btn2[0] = p_btn2;
		g_target_report_data->p_tilt_x[0] = p_tilt_x;
		g_target_report_data->p_tilt_y[0] = p_tilt_y;
		g_target_report_data->p_on[0] = 1;
		ts->hx_point_num++;
	} else {/* report coordinates */
		g_target_report_data->p_x[0] = 0;
		g_target_report_data->p_y[0] = 0;
		g_target_report_data->p_w[0] = 0;
		g_target_report_data->p_hover[0] = 0;
		g_target_report_data->pen_id[0] = 0;
		g_target_report_data->p_btn[0] = 0;
		g_target_report_data->p_btn2[0] = 0;
		g_target_report_data->p_tilt_x[0] = 0;
		g_target_report_data->p_tilt_y[0] = 0;
		g_target_report_data->p_on[0] = 0;
	}

	if (g_ts_dbg != 0) {
		if (p_p_on != g_target_report_data->p_on[0]) {
			I("p_on[0] = %d, hx_point_num=%d\n",
				g_target_report_data->p_on[0],
				ts->hx_point_num);
			p_p_on = g_target_report_data->p_on[0];
		}
	}
#endif

	ts->old_finger = ts->pre_finger_mask;
	if (ts->hx_point_num == 0) {
		if (g_ts_dbg != 0)
			I("%s: hx_point_num = 0!\n", __func__);
		return ts_status;
	}
	ts->pre_finger_mask = 0;
	hx_touch_data->finger_num = hx_touch_data->hx_coord_buf[ts->coordInfoSize - 4] & 0x0F;
	hx_touch_data->finger_on = 1;
	AA_press = 1;

	g_target_report_data->finger_num = hx_touch_data->finger_num;
	g_target_report_data->finger_on = hx_touch_data->finger_on;
	g_target_report_data->ig_count = hx_touch_data->hx_coord_buf[ts->coordInfoSize - 5];

	if (g_ts_dbg != 0)
		I("%s:finger_num = 0x%2X, finger_on = %d\n", __func__, g_target_report_data->finger_num, g_target_report_data->finger_on);

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		base = loop_i * 4;
		x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
		y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
		w = hx_touch_data->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];

		if (g_ts_dbg != 0)
			D("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__, loop_i, x, y, w);

		if (x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 && y <= ts->pdata->abs_y_max) {
			hx_touch_data->finger_num--;

			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 1;

			/*I("%s: g_target_report_data->x[loop_i]=%d, g_target_report_data->y[loop_i]=%d, g_target_report_data->w[loop_i]=%d",*/
			/*__func__, g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->w[loop_i]); */


			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				I("S1@%d, %d\n", x, y);
			}

			ts->pre_finger_data[loop_i][0] = x;
			ts->pre_finger_data[loop_i][1] = y;

			ts->pre_finger_mask = ts->pre_finger_mask + (1 << loop_i);
		} else {/* report coordinates */
			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 0;

			if (loop_i == 0 && ts->first_pressed == 1) {
				ts->first_pressed = 2;
				I("E1@%d, %d\n", ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (loop_i = 0; loop_i < 10; loop_i++)
			D("DBG X=%d  Y=%d ID=%d\n", g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->finger_id[loop_i]);

		D("DBG finger number %d\n", g_target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
	return ts_status;
}

static int himax_parse_report_data(struct himax_ts_data *ts, int ts_path, int ts_status)
{

	if (g_ts_dbg != 0)
		I("%s: start now_status=%d!\n", __func__, ts_status);


	EN_NoiseFilter = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 3);
	/* I("EN_NoiseFilter=%d\n", EN_NoiseFilter); */
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
	/* I("EN_NoiseFilter2=%d\n", EN_NoiseFilter); */
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	tpd_key = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 4);

	/* All (VK+AA)leave */
	if (tpd_key == 0x0F)
		tpd_key = 0x00;

#endif
	p_point_num = ts->hx_point_num;

	if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
		ts->hx_point_num = 0;
	else
		ts->hx_point_num = hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;


	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		if (debug_data != NULL) {
			if (debug_data->fp_set_diag_cmd(ic_data, hx_touch_data))
				I("%s: raw data_checksum not match\n", __func__);
		} else {
			E("%s,There is no init set_diag_cmd\n", __func__);
		}
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
#ifdef HX_SMART_WAKEUP
	case HX_REPORT_SMWP_EVENT:
		himax_wake_event_parse(ts, ts_status);
		break;
#endif
	default:
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		I("%s: end now_status=%d!\n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

static void himax_report_all_leave_event(struct himax_ts_data *ts)
{
	int loop_i = 0;

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
#ifndef	HX_PROTOCOL_A
		input_mt_slot(ts->input_dev, loop_i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

/* start report_data */
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
static void himax_key_report_operation(int tp_key_index, struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;

	if (g_ts_dbg != 0)
		I("%s: Entering\n", __func__);

	if (tp_key_index != 0x00) {
		I("virtual key index =%x\n", tp_key_index);

		if (tp_key_index == 0x01) {
			vk_press = 1;
			I("back key pressed\n");

			if (ts->pdata->virtual_key) {
				if (ts->button[0].index) {
					x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
					y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
				}

#ifdef	HX_PROTOCOL_A
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
#else
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
#endif
			}	else {
				input_report_key(ts->input_dev, KEY_BACK, 1);
			}
		} else if (tp_key_index == 0x02) {
			vk_press = 1;
			I("home key pressed\n");

			if (ts->pdata->virtual_key) {
				if (ts->button[1].index) {
					x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
					y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
				}

#ifdef	HX_PROTOCOL_A
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
#else
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
#endif
			} else {
				input_report_key(ts->input_dev, KEY_HOME, 1);
			}
		} else if (tp_key_index == 0x04) {
			vk_press = 1;
			I("APP_switch key pressed\n");

			if (ts->pdata->virtual_key) {
				if (ts->button[2].index) {
					x_position = (ts->button[2].x_range_min + ts->button[2].x_range_max) / 2;
					y_position = (ts->button[2].y_range_min + ts->button[2].y_range_max) / 2;
				}

#ifdef HX_PROTOCOL_A
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
#else
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
#endif
			} else {
				input_report_key(ts->input_dev, KEY_APPSELECT, 1);
			}
		}
		input_sync(ts->input_dev);
	} else { /*tp_key_index =0x00*/
		I("virtual key released\n");
		vk_press = 0;
#ifndef	HX_PROTOCOL_A
		input_mt_slot(ts->input_dev, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#else
		input_mt_sync(ts->input_dev);
#endif
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		input_report_key(ts->input_dev, KEY_APPSELECT, 0);
#ifndef	HX_PROTOCOL_A
		input_sync(ts->input_dev);
#endif
	}
}

void himax_finger_report_key(struct himax_ts_data *ts)
{
	if (ts->hx_point_num != 0) {
		/*Touch KEY*/
		if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
			/* temp_x[0] = 0xFFFF;
			 * temp_y[0] = 0xFFFF;
			 * temp_x[1] = 0xFFFF;
			 * temp_y[1] = 0xFFFF;
			 */
			hx_touch_data->finger_on = 0;
#ifdef HX_PROTOCOL_A
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
			himax_key_report_operation(tpd_key, ts);
#ifndef HX_PROTOCOL_A
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
		}
		input_sync(ts->input_dev);
	}
}

void himax_finger_leave_key(struct himax_ts_data *ts)
{
	if (tpd_key != 0x00) {
		hx_touch_data->finger_on = 1;
#ifdef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
		himax_key_report_operation(tpd_key, ts);
#ifndef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
	} else if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
		hx_touch_data->finger_on = 0;
#ifdef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
		himax_key_report_operation(tpd_key, ts);
#ifndef HX_PROTOCOL_A
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
	}
	input_sync(ts->input_dev);
}

static void himax_report_key(struct himax_ts_data *ts)
{
	if (ts->hx_point_num != 0) { /* Touch KEY */
		himax_finger_report_key(ts);
	} else { /* Key */
		himax_finger_leave_key(ts);
	}

	tpd_key_old = tpd_key;
	Last_EN_NoiseFilter = EN_NoiseFilter;
}
#endif

/* start report_point*/
static void himax_finger_report(struct himax_ts_data *ts)
{
	int i = 0;
	bool valid = false;


	if (g_ts_dbg != 0) {
		I("%s:start\n", __func__);
		I("hx_touch_data->finger_num=%d\n", hx_touch_data->finger_num);
	}
	for (i = 0; i < ts->nFinger_support; i++) {
		if (g_target_report_data->x[i] >= 0 && g_target_report_data->x[i] <= ts->pdata->abs_x_max && g_target_report_data->y[i] >= 0 && g_target_report_data->y[i] <= ts->pdata->abs_y_max)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			I("valid=%d\n", valid);
		if (valid) {
			if (g_ts_dbg != 0)
				I("g_target_report_data->x[i]=%d, g_target_report_data->y[i]=%d, g_target_report_data->w[i]=%d\n", g_target_report_data->x[i], g_target_report_data->y[i], g_target_report_data->w[i]);
#ifndef	HX_PROTOCOL_A
			input_mt_slot(ts->input_dev, i);
#else
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_target_report_data->w[i]);
#ifndef	HX_PROTOCOL_A
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_target_report_data->w[i]);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, g_target_report_data->w[i]);
#else
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_target_report_data->x[i]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_target_report_data->y[i]);
#ifndef	HX_PROTOCOL_A
			ts->last_slot = i;
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
#else
			input_mt_sync(ts->input_dev);
#endif
		} else {
#ifndef	HX_PROTOCOL_A
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
		}
	}
#ifndef	HX_PROTOCOL_A
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
	input_sync(ts->input_dev);

#if defined(HX_PEN_FUNC_EN)
	valid = false;

	if (g_target_report_data->p_x[0] >= 0 && g_target_report_data->p_x[0] <= ts->pdata->abs_x_max
		&& g_target_report_data->p_y[0] >= 0 && g_target_report_data->p_y[0] <= ts->pdata->abs_y_max
		&& (g_target_report_data->p_on[0] == 1))
		valid = true;
	else
		valid = false;
	if (g_ts_dbg != 0)
		I("pen valid=%d\n", valid);
	if (valid) {/*Pen down*/
		if (g_ts_dbg != 0)
			I("p_x[i]=%d, p_y[i]=%d, p_w[i]=%d\n", g_target_report_data->p_x[0], g_target_report_data->p_y[0], g_target_report_data->p_w[0]);

		input_report_abs(ts->hx_pen_dev, ABS_X, g_target_report_data->p_x[0]);
		input_report_abs(ts->hx_pen_dev, ABS_Y, g_target_report_data->p_y[0]);
		input_report_key(ts->hx_pen_dev, BTN_STYLUS, 1);
		input_report_abs(ts->hx_pen_dev, ABS_TILT_X, g_target_report_data->p_tilt_x[0]);
		input_report_abs(ts->hx_pen_dev, ABS_TILT_Y, g_target_report_data->p_tilt_y[0]);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_PEN, 1);
		if (g_target_report_data->p_hover[0] == 0) {
			input_report_key(ts->hx_pen_dev, BTN_TOUCH, 1);
			input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 0);
			input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, g_target_report_data->p_w[0]);
		} else {
			input_report_key(ts->hx_pen_dev, BTN_TOUCH, 0);
			input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 1);
			input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
		}
	} else {/*Pen up*/
		input_report_key(ts->hx_pen_dev, BTN_STYLUS, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOUCH, 0);
		input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
		input_sync(ts->hx_pen_dev);

		input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_RUBBER, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_PEN, 0);
		input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
	}
	input_sync(ts->hx_pen_dev);

#endif

	if (g_ts_dbg != 0)
		I("%s:end\n", __func__);
}


#ifdef HIMAX_PALM_SENSOR_EN
static void himax_palm_detect_func(struct himax_ts_data *ts, bool detected) {
	if (detected) {
		I("palm detected\n");
		input_report_abs(ts->palm_sensor_pdata->input_sensor_dev, ABS_DISTANCE, 1);
	} else {
		I("palm leave\n");
		input_report_abs(ts->palm_sensor_pdata->input_sensor_dev, ABS_DISTANCE, 0);
	}
	input_sync(ts->palm_sensor_pdata->input_sensor_dev);
}
#endif

static void himax_finger_leave(struct himax_ts_data *ts)
{
#ifndef	HX_PROTOCOL_A
	int32_t loop_i = 0;
#endif

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

#ifndef HIMAX_PALM_SENSOR_EN
#if defined(HX_PALM_REPORT)
	if (himax_palm_detect(hx_touch_data->hx_coord_buf) == PALM_REPORT) {
		I(" %s HX_PALM_REPORT KEY power event press\n", __func__);
		input_report_key(ts->input_dev, KEY_POWER, 1);
		input_sync(ts->input_dev);
		msleep(100);

		I(" %s HX_PALM_REPORT KEY power event release\n", __func__);
		input_report_key(ts->input_dev, KEY_POWER, 0);
		input_sync(ts->input_dev);
		return;
	}
#endif
#endif

	hx_touch_data->finger_on = 0;
	g_target_report_data->finger_on  = 0;
	g_target_report_data->finger_num = 0;
	AA_press = 0;

#ifdef HX_PROTOCOL_A
	input_mt_sync(ts->input_dev);
#endif
#ifndef	HX_PROTOCOL_A
	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		input_mt_slot(ts->input_dev, loop_i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	if (ts->pre_finger_mask > 0)
		ts->pre_finger_mask = 0;

	if (ts->first_pressed == 1) {
		ts->first_pressed = 2;
		I("E1@%d, %d\n", ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
	}

	/*if (ts->debug_log_level & BIT(1)) */
	/*himax_log_touch_event(x, y, w, loop_i, EN_NoiseFilter, HX_FINGER_LEAVE); */

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

#if defined(HX_PEN_FUNC_EN)
	input_report_key(ts->hx_pen_dev, BTN_STYLUS, 0);
	input_report_key(ts->hx_pen_dev, BTN_TOUCH, 0);
	input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
	input_sync(ts->hx_pen_dev);

	input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 0);
	input_report_abs(ts->hx_pen_dev, ABS_TILT_X, 0);
	input_report_abs(ts->hx_pen_dev, ABS_TILT_Y, 0);
	input_report_key(ts->hx_pen_dev, BTN_TOOL_RUBBER, 0);
	input_report_key(ts->hx_pen_dev, BTN_TOOL_PEN, 0);
	input_sync(ts->hx_pen_dev);
#endif

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);


}

static void himax_report_points(struct himax_ts_data *ts)
{
	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

#if defined(HX_PALM_REPORT) && defined(HIMAX_PALM_SENSOR_EN)
	if (ts->palm_detection_enabled) {
		if (himax_palm_detect(hx_touch_data->hx_coord_buf) == PALM_REPORT) {
			del_timer(&ts->palm_release_fimer);
			himax_palm_detect_func(ts, true);
			return;
		} else if (himax_palm_detect(hx_touch_data->hx_coord_buf) == PALM_LEAVE_REPORT) {
			mod_timer(&ts->palm_release_fimer,
				jiffies + msecs_to_jiffies(ts->palm_release_delay_ms));
#ifdef CONFIG_HAS_WAKELOCK
			wake_lock_timeout(&ts->palm_gesture_wakelock,
				ts->palm_release_delay_ms);
#else
			__pm_wakeup_event(&ts->palm_gesture_wakelock,
				ts->palm_release_delay_ms);
#endif
			return;
		} else if (ts->suspended)
			return;
	}
#endif
	if (ts->hx_point_num != 0)
		himax_finger_report(ts);
	else
		himax_finger_leave(ts);

	Last_EN_NoiseFilter = EN_NoiseFilter;

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
}
/* end report_points*/

int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
		/* Touch Point information */
		himax_report_points(ts);

#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
		/*report key(question mark)*/
		if (tpd_key && tpd_key_old)
			himax_report_key(ts);

#endif
#ifdef HX_SMART_WAKEUP
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		himax_wake_event_report();
#endif
	} else {
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

static int himax_ts_operation(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	uint8_t hw_reset_check[2];

	memset(ts->xfer_buff, 0x00, 128 * sizeof(uint8_t));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	ts_status = himax_touch_get(ts, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto END_FUNCTION;

	ts_status = himax_distribute_touch_data(ts->xfer_buff, ts_path, ts_status);
	ts_status = himax_err_ctrl(ts, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END)
		ts_status = himax_parse_report_data(ts, ts_path, ts_status);
	else
		goto END_FUNCTION;


	ts_status = himax_report_data(ts, ts_path, ts_status);


END_FUNCTION:
	return ts_status;
}

#ifdef HIMAX_PALM_SENSOR_EN
static void himax_palm_sensor_release_timer_handler(unsigned long data)
{
	himax_palm_detect_func(private_ts, false);
}

static int _himax_palm_detect_sensor_set_enable(unsigned int enable)
{
	I("Gesture set enable %d!", enable);
/*
 * If palm detect function is enabled, interrupt will not disable, IC works in
 * normal mode. But in case touch event is reported to input subsystem, skip
 * touch event when suspend flag is true. So input subsystem will not take
 * wakelock because no one report event.
 * In this case, we still need read data from IC, so AP can not enter suspend.
 */
	if (enable == 1) {
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock(&private_ts->palm_gesture_read_wakelock);
#else
		__pm_stay_awake(&private_ts->palm_gesture_read_wakelock);
#endif
		g_core_fp.fp_palm_detection_function(1);
		private_ts->palm_detection_enabled = true;
	} else if (enable == 0) {
		if (timer_pending(&private_ts->palm_release_fimer)) {
			himax_palm_detect_func(private_ts, false);
			del_timer(&private_ts->palm_release_fimer);
		}
		g_core_fp.fp_palm_detection_function(0);
		private_ts->palm_detection_enabled = false;
#ifdef CONFIG_HAS_WAKELOCK
		wake_unlock(&private_ts->palm_gesture_read_wakelock);
#else
		__pm_relax(&private_ts->palm_gesture_read_wakelock);
#endif
	} else
		E("unknown enable symbol\n");

	return 0;
}

static int himax_palm_detect_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;

	mutex_lock(&private_ts->suspend_resume_mutex);
	if (!private_ts->suspended)
		ret = _himax_palm_detect_sensor_set_enable(enable);
	else {
		I("Gesture lazy set enable %d!", enable);
		if (enable == 1)
			private_ts->palm_detection_lazy_set = PALM_SENSOR_LAZY_SET_ENABLE;
		else if (enable == 0)
			private_ts->palm_detection_lazy_set = PALM_SENSOR_LAZY_SET_DISABLE;
		else
			E("unknown enable symbol\n");
	}
	mutex_unlock(&private_ts->suspend_resume_mutex);
	himax_palm_detect_func(private_ts, false);

	return ret;
}

static int himax_palm_detect_sensor_init(struct himax_ts_data *data)
{
	struct himax_sensor_platform_data *sensor_pdata;
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		E("Failed to allocate device");
		goto exit;
	}

	sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
			sizeof(struct himax_sensor_platform_data),
			GFP_KERNEL);
	if (!sensor_pdata) {
		E("Failed to allocate memory");
		goto free_sensor_pdata;
	}
	data->palm_sensor_pdata = sensor_pdata;

	__set_bit(EV_ABS, sensor_input_dev->evbit);
	__set_bit(EV_SYN, sensor_input_dev->evbit);
	input_set_abs_params(sensor_input_dev, ABS_DISTANCE,
			0, 5, 0, 0);
	sensor_input_dev->name = "palm_detect";
	data->palm_sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		E("Unable to register device, err=%d", err);
		goto free_sensor_input_dev;
	}

	sensor_pdata->ps_cdev = palm_sensors_touch_cdev;
	sensor_pdata->ps_cdev.sensors_enable = himax_palm_detect_sensor_set_enable;
	sensor_pdata->data = data;

	err = sensors_classdev_register(&sensor_input_dev->dev,
				&sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&data->palm_gesture_wakelock, WAKE_LOCK_SUSPEND, "palm_detect_wl");
#else
	wakeup_source_init(&data->palm_gesture_wakelock, "palm_detect_wl");
#endif
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&data->palm_gesture_read_wakelock, WAKE_LOCK_SUSPEND, "palm_read_wl");
#else
	wakeup_source_init(&data->palm_gesture_read_wakelock, "palm_read_wl");
#endif

	data->palm_release_fimer.function = himax_palm_sensor_release_timer_handler;
	init_timer(&data->palm_release_fimer);
	data->palm_release_delay_ms = 850;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(data->palm_sensor_pdata->input_sensor_dev);
free_sensor_input_dev:
	input_free_device(data->palm_sensor_pdata->input_sensor_dev);
free_sensor_pdata:
	devm_kfree(&sensor_input_dev->dev, sensor_pdata);
	data->palm_sensor_pdata= NULL;
exit:
	return 1;
}

int himax_palm_detect_sensor_remove(struct himax_ts_data *data)
{
	sensors_classdev_unregister(&data->palm_sensor_pdata->ps_cdev);
	input_unregister_device(data->palm_sensor_pdata->input_sensor_dev);
	devm_kfree(&data->palm_sensor_pdata->input_sensor_dev->dev,
		data->palm_sensor_pdata);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&data->palm_gesture_wakelock);
#else
	wakeup_source_trash(&data->palm_gesture_wakelock);
#endif
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&data->palm_gesture_read_wakelock);
#else
	wakeup_source_trash(&data->palm_gesture_read_wakelock);
#endif
	data->palm_sensor_pdata = NULL;
	data->palm_detection_enabled = false;
	return 0;
}
#endif

#ifdef HIMAX_V2_SENSOR_EN
static int himax_tap_detect_sensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	struct himax_ts_data *ts = private_ts;

	if (enable) {
		ts->SMWP_enable = 1;
		ts->gesture_cust_en[0] = 1;
		I("%s,here:%d, gesture_cust_en[0] = %d\n", __func__, __LINE__, ts->gesture_cust_en[0]);
	} else {
		ts->SMWP_enable = 0;
		ts->gesture_cust_en[0] = 0;
		I("%s,here:%d, gesture_cust_en[0] = %d\n", __func__, __LINE__, ts->gesture_cust_en[0]);
	}

	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);

	return ret;
}

static int himax_tap_detect_sensor_init(struct himax_ts_data *data)
{
	struct himax_tap_sensor_platform_data *sensor_pdata;
	struct input_dev *sensor_input_dev;
	int err;

	sensor_input_dev = input_allocate_device();
	if (!sensor_input_dev) {
		E("Failed to allocate device");
		goto exit;
	}

	sensor_pdata = devm_kzalloc(&sensor_input_dev->dev,
			sizeof(struct himax_tap_sensor_platform_data),
			GFP_KERNEL);
	if (!sensor_pdata) {
		E("Failed to allocate memory");
		goto free_sensor_pdata;
	}
	data->sensor_pdata = sensor_pdata;

	__set_bit(EV_ABS, sensor_input_dev->evbit);
	__set_bit(EV_SYN, sensor_input_dev->evbit);
	input_set_abs_params(sensor_input_dev, ABS_DISTANCE,
			0, REPORT_MAX_COUNT, 0, 0);
	sensor_input_dev->name = "double-tap";
	data->sensor_pdata->input_sensor_dev = sensor_input_dev;

	err = input_register_device(sensor_input_dev);
	if (err) {
		E("Unable to register device, err=%d", err);
		goto free_sensor_input_dev;
	}

	sensor_pdata->ps_cdev = sensors_touch_cdev;
	sensor_pdata->ps_cdev.sensors_enable = himax_tap_detect_sensor_set_enable;
	sensor_pdata->data = data;

	err = sensors_classdev_register(&sensor_input_dev->dev,
				&sensor_pdata->ps_cdev);
	if (err)
		goto unregister_sensor_input_device;

	return 0;

unregister_sensor_input_device:
	input_unregister_device(data->sensor_pdata->input_sensor_dev);
free_sensor_input_dev:
	input_free_device(data->sensor_pdata->input_sensor_dev);
free_sensor_pdata:
	devm_kfree(&sensor_input_dev->dev, sensor_pdata);
	data->sensor_pdata= NULL;
exit:
	return 1;
}

int himax_tap_detect_sensor_remove(struct himax_ts_data *data)
{
	sensors_classdev_unregister(&data->sensor_pdata->ps_cdev);
	input_unregister_device(data->sensor_pdata->input_sensor_dev);
	devm_kfree(&data->sensor_pdata->input_sensor_dev->dev,
		data->sensor_pdata);
	data->sensor_pdata = NULL;
	return 0;
}
#endif

void himax_ts_work(struct himax_ts_data *ts)
{

	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;

	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(ts, HX_FINGER_ON);

#if defined(HX_USB_DETECT_GLOBAL)
	himax_cable_detect_func(false);
#endif

	ts_path = himax_ts_work_status(ts);
	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	case HX_REPORT_SMWP_EVENT:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	default:
		E("%s:Path Fault! value=%d\n", __func__, ts_path);
		goto END_FUNCTION;
	}

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		goto END_FUNCTION;

GET_TOUCH_FAIL:
	I("%s: Now reset the Touch chip.\n", __func__);
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, true);
#else
	g_core_fp.fp_system_reset();
#endif
#if defined(HX_ZERO_FLASH) && defined(HX83102D)
	if (g_core_fp.fp_0f_reload_to_active)
		g_core_fp.fp_0f_reload_to_active();
#endif
END_FUNCTION:
	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(ts, HX_FINGER_LEAVE);

}
/*end ts_work*/
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;


	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#if defined(HX_USB_DETECT_CALLBACK)
static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;

	I("Touch: cable change to %d\n", connect_status);

	ts = private_ts;

	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode)) {
			if (connect_status != ts->usb_connected) {
				if (connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

				himax_bus_master_write(ts->cable_config,
										sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);
				I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			} else
				I("%s: Cable status is the same as previous one, ignore.\n", __func__);
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;

			I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};

#endif

#ifdef HX_AUTO_UPDATE_FW
static void himax_update_register(struct work_struct *work)
{
	I(" %s in\n", __func__);

	if (i_get_FW() != 0)
		return;

	if (g_auto_update_flag == true) {
		I("Update FW Directly");
		goto UPDATE_FW;
	}

	if (himax_auto_update_check() != 0) {
		I("%s:Don't run auto update fw, so free allocated!\n", __func__);
		kfree(i_CTPM_FW);
		i_CTPM_FW = NULL;
		return;
	}

UPDATE_FW:
	if (i_update_FW() <= 0)
		I("Auto update FW fail!\n");
	else
		I("It have Updated\n");

}
#endif

#ifndef HX_ZERO_FLASH
static int hx_chk_flash_sts(void)
{
	int rslt = 0;
	uint32_t size = 0;

	I("%s: Entering\n", __func__);
	if (g_core_fp._diff_overlay_flash() == 1)
		size = FW_SIZE_128k;
	else
		size = FW_SIZE_64k;


#ifdef HX_AUTO_UPDATE_FW
	g_auto_update_flag = (!g_core_fp.fp_calculateChecksum(false, size));
	g_auto_update_flag |= g_core_fp.fp_flash_lastdata_check(size);
#else
	g_core_fp.fp_calculateChecksum(false, size);
	g_core_fp.fp_flash_lastdata_check(size);
#endif

	return rslt;
}
#endif

#if defined(CONFIG_DRM)
extern struct drm_panel *himax_active_panel;
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;

	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work_att.work);


	I("%s in\n", __func__);
	ts->fb_notif.notifier_call = drm_notifier_callback;
	if (himax_active_panel)
		ret = drm_panel_notifier_register(himax_active_panel, &ts->fb_notif);

	if (ret)
		E("Unable to register fb_notifier: %d\n", ret);
}
#elif defined(CONFIG_FB)
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;

	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work_att.work);


	I("%s in\n", __func__);
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);

	if (ret)
		E("Unable to register fb_notifier: %d\n", ret);
}
#endif

#include <linux/major.h>
#include <linux/kdev_t.h>

/* Attribute: path (RO) */
static ssize_t path_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	ssize_t blen;
	const char *path;

	if (!ts) {
		pr_err("cannot get himax_ts_data pointer\n");
		return (ssize_t)0;
	}
	path = kobject_get_path(&ts->dev->kobj, GFP_KERNEL);
	blen = scnprintf(buf, PAGE_SIZE, "%s", path ? path : "na");
	kfree(path);
	return blen;
}

/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "himax");
}

/* Attribute: vendor (RO) */
static ssize_t ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	int buildid;

	buildid = (ic_data->vendor_cid_maj_ver << 8 |
				ic_data->vendor_cid_min_ver);
	return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%04x\n%s%04x\n",
			"Product ID: ", ts->chip_name,
			"Build ID: ", buildid ? buildid : ts->build_id,
			"Config ID: ", ic_data->vendor_touch_cfg_ver ? ic_data->vendor_touch_cfg_ver : ts->config_id);
}

static struct device_attribute touchscreen_attributes[] = {
	__ATTR_RO(path),
	__ATTR_RO(vendor),
	__ATTR_RO(ic_ver),
	__ATTR_NULL
};

#define TSDEV_MINOR_BASE 128
#define TSDEV_MINOR_MAX 32

static int himax_sysfs_touchscreen(
	struct himax_ts_data *ts, bool create)
{
	struct device_attribute *attrs = touchscreen_attributes;
	int i, error = 0;
	static struct class *touchscreen_class;
	static struct device *ts_class_dev;
	static int minor;

	if (create) {
		minor = input_get_new_minor(0x90,
						1, false);
		if (minor < 0)
			minor = input_get_new_minor(TSDEV_MINOR_BASE,
					TSDEV_MINOR_MAX, true);
		pr_info("assigned minor %d\n", minor);

		touchscreen_class = class_create(THIS_MODULE, "touchscreen");
		if (IS_ERR(touchscreen_class)) {
			error = PTR_ERR(touchscreen_class);
			touchscreen_class = NULL;
			return error;
		}

		ts_class_dev = device_create(touchscreen_class, NULL,
				MKDEV(INPUT_MAJOR, minor),
				ts, "%s", ts->chip_name);
		if (IS_ERR(ts_class_dev)) {
			error = PTR_ERR(ts_class_dev);
			ts_class_dev = NULL;
			return error;
		}
		for (i = 0; attrs[i].attr.name != NULL; ++i) {
			error = device_create_file(ts_class_dev, &attrs[i]);
			if (error)
				break;
		}

		if (error)
			goto device_destroy;
	} else {
		if (!touchscreen_class || !ts_class_dev)
			return -ENODEV;

		for (i = 0; attrs[i].attr.name != NULL; ++i)
			device_remove_file(ts_class_dev, &attrs[i]);

		device_unregister(ts_class_dev);
		class_unregister(touchscreen_class);
	}

	return 0;

device_destroy:
	for (--i; i >= 0; --i)
		device_remove_file(ts_class_dev, &attrs[i]);
	device_destroy(touchscreen_class, MKDEV(INPUT_MAJOR, minor));
	ts_class_dev = NULL;
	class_unregister(touchscreen_class);
	pr_err("error creating touchscreen class\n");

	return -ENODEV;
}

#if defined(HX_USB_DETECT_GLOBAL)
static int charger_notifier_callback(struct notifier_block *nb,
								unsigned long val, void *v) {
	int ret = 0;
	struct power_supply *psy = NULL;
	struct himax_ts_data *ts = container_of(nb, struct himax_ts_data, charger_notif);
	union power_supply_propval prop;

	psy= power_supply_get_by_name("usb");
	if (!psy) {
		E("Couldn't get usbpsy\n");
		return -EINVAL;
	}
	if (!strcmp(psy->desc->name, "usb")) {
		if (psy && ts && val == POWER_SUPPLY_PROP_STATUS) {
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,&prop);
			if (ret < 0) {
				E("Couldn't get POWER_SUPPLY_PROP_ONLINE rc=%d\n", ret);
				return ret;
			} else {
				USB_detect_flag = prop.intval;
				//E("usb prop.intval =%d\n", prop.intval);
			}
		}
	}
	return 0;
}
#endif

#if defined(__HIMAX_HX83102D_MOD__)
#define TP_CHECK_TIMES 30
#define TP_CHECK_INTERVAL 10
void himax_resume_thread_check(void)
{
	uint8_t cycle = 0;

	while (true) {
		if(cycle++ > TP_CHECK_TIMES)
			break;

		if (atomic_read(&private_ts->resume_thread_is_runing))
			msleep(TP_CHECK_INTERVAL);
		else
			break;
	}
	I("cycle:%d",cycle);
	return;
}
#endif

#if defined(HX_RESUME_SET_FW)
void himax_resume_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = private_ts;
#if defined(__HIMAX_HX83102D_MOD__)
	atomic_set(&private_ts->resume_thread_is_runing, 1);
#endif
	himax_chip_common_resume(ts);
#if defined(__HIMAX_HX83102D_MOD__)
	atomic_set(&private_ts->resume_thread_is_runing, 0);
#endif
	return;
}
#endif
int himax_chip_common_init(void)
{

	int i = 0, ret = 0, err = 0, idx = 0;
	struct himax_ts_data *ts = private_ts;
	struct himax_i2c_platform_data *pdata;
	struct himax_chip_entry *entry;

	idx = himax_get_ksym_idx();
	if (idx >= 0)
		if (isEmpty(idx) != 0)
			return -EPROBE_DEFER;

	I("Prepare kernel fp\n");
	kp_getname_kernel = (void *)kallsyms_lookup_name("getname_kernel");
	if (!kp_getname_kernel) {
		E("prepare kp_getname_kernel failed!\n");
		/*goto err_dt_platform_data_fail;*/
	}
	kp_putname_kernel = (void *)kallsyms_lookup_name("putname");
	if (!kp_putname_kernel) {
		E("prepare kp_putname_kernel failed!\n");
		/*goto err_xfer_buff_fail;*/
	}
	kp_file_open_name = (void *)kallsyms_lookup_name("file_open_name");
	if (!kp_file_open_name) {
		E("prepare kp_file_open_name failed!\n");
		goto err_dt_platform_data_fail;
	}
#if defined(__EMBEDDED_FW__)
	g_embedded_fw.size = (size_t)_binary___Himax_firmware_bin_end -
			(size_t)_binary___Himax_firmware_bin_start;
#endif
	ts->xfer_buff = devm_kzalloc(ts->dev, 128 * sizeof(uint8_t), GFP_KERNEL);
	if (ts->xfer_buff == NULL) {
		err = -ENOMEM;
		goto err_dt_platform_data_fail;
	}

	I("PDATA START\n");
	pdata = kzalloc(sizeof(struct himax_i2c_platform_data), GFP_KERNEL);

	if (pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
		goto err_dt_platform_data_fail;
	}

	I("ic_data START\n");
	ic_data = kzalloc(sizeof(struct himax_ic_data), GFP_KERNEL);
	if (ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
		goto err_dt_ic_data_fail;
	}

	/* allocate report data */
	hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hx_touch_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_touch_data_failed;
	}

	if (himax_parse_dt(ts, pdata) < 0) {
		I(" pdata is NULL for DT\n");
		goto err_alloc_dt_pdata_failed;
	}

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif
	if (himax_gpio_power_config(pdata)) {
		err = -1;
		E("%s: himax_gpio_power_config failed, stop Himax probe\n", __func__);
		goto err_himax_gpio_power_config_failed;
	}
#ifndef CONFIG_OF

	if (pdata->power) {
		ret = pdata->power(1);

		if (ret < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}

#endif

	i_CTPM_firmware_name = kzalloc(HIMAX_FILE_NAME_LENGTH, GFP_KERNEL);
	if (!i_CTPM_firmware_name) {
		E("%s: alloc i_CTPM_firmware_name failed\n", __func__);
		goto firmware_name_alloc_failed;
	}
	if (pdata->panel_supplier[0])
		snprintf(i_CTPM_firmware_name, HIMAX_FILE_NAME_LENGTH, "%s_Himax_firmware.bin",
			pdata->panel_supplier);
	else
		snprintf(i_CTPM_firmware_name, HIMAX_FILE_NAME_LENGTH, "Himax_firmware.bin");

	g_hx_chip_inited = 0;
	idx = himax_get_ksym_idx();
	entry = get_chip_entry_by_index(idx);

	for (i = 0; i < entry->hx_ic_dt_num; i++) {
		if (entry->core_chip_dt[i].fp_chip_detect != NULL) {
			if (entry->core_chip_dt[i].fp_chip_detect()
			  == true) {
				I("%s: chip detect found! list_num=%d\n"
					, __func__, i);
				goto found_hx_chip;
			} else {
				I("%s: num=%d chip detect NOT found! goto Next\n", __func__, i);
				continue;
			}
		}
	}

	if (i == entry->hx_ic_dt_num) {
		E("%s: chip detect failed!\n", __func__);
		goto error_ic_detect_failed;
	}
found_hx_chip:
	if (g_core_fp.fp_chip_init != NULL) {
		g_core_fp.fp_chip_init();
	} else {
		E("%s: function point of chip_init is NULL!\n", __func__);
		goto error_ic_detect_failed;
	}

	if (pdata->virtual_key)
		ts->button = pdata->virtual_key;

#ifndef HX_ZERO_FLASH
	g_core_fp.fp_read_FW_ver();
	hx_chk_flash_sts();
#endif

#ifdef HX_AUTO_UPDATE_FW
	if (g_auto_update_flag)
		goto FW_force_upgrade;
#endif


#ifdef HX_AUTO_UPDATE_FW
FW_force_upgrade:
	ts->himax_update_wq = create_singlethread_workqueue("HMX_update_reuqest");
	if (!ts->himax_update_wq) {
		E(" allocate himax_update_wq failed\n");
		err = -ENOMEM;
		goto err_update_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->work_update, himax_update_register);
	queue_delayed_work(ts->himax_update_wq, &ts->work_update, msecs_to_jiffies(2000));
#endif
#ifdef HX_ZERO_FLASH
	g_auto_update_flag = true;
	ts->himax_0f_update_wq = create_singlethread_workqueue("HMX_0f_update_reuqest");
	if (!ts->himax_0f_update_wq) {
		E(" allocate himax_0f_update_wq failed\n");
		err = -ENOMEM;
		goto err_0f_update_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->work_0f_update, g_core_fp.fp_0f_operation);
	queue_delayed_work(ts->himax_0f_update_wq, &ts->work_0f_update, msecs_to_jiffies(2000));
#endif

#ifdef HX_RESUME_SET_FW
	ts->ts_int_workqueue = create_singlethread_workqueue("himax_ts_resume_wq");
	if (!ts->ts_int_workqueue) {
		E("%s: create ts_resume workqueue failed\n", __func__);
		goto err_create_ts_resume_wq_failed;
	}
#if defined(__HIMAX_HX83102D_MOD__)
	INIT_DELAYED_WORK(&ts->ts_int_work, himax_resume_work_func);
#else
	INIT_WORK(&ts->ts_int_work, himax_resume_work_func);
#endif
#endif
	/*Himax Power On and Load Config*/
	if (himax_loadSensorConfig(pdata)) {
		E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
		goto err_detect_failed;
	}

	g_core_fp.fp_power_on_init();
	calculate_point_number();

#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;
	/*calculate the i2c data size*/
	calcDataSize();
	I("%s: calcDataSize complete\n", __func__);
#ifdef CONFIG_OF
	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;
#endif
	ts->suspended                      = false;
	atomic_set(&ts->resume_update_fail, 0);
	atomic_set(&ts->resume_thread_is_runing, 0);
	mutex_init(&ts->suspend_resume_mutex);
#if defined(HX_USB_DETECT_CALLBACK) || defined(HX_USB_DETECT_GLOBAL)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
	ts->charger_notif.notifier_call = charger_notifier_callback;
	ret = power_supply_reg_notifier(&ts->charger_notif);
	if (ret) {
		E("Unable to register charger_notifier: %d\n",ret);
		goto err_register_charger_notify_failed;
	}
#endif
#ifdef	HX_PROTOCOL_A
	ts->protocol_type = PROTOCOL_TYPE_A;
#else
	ts->protocol_type = PROTOCOL_TYPE_B;
#endif
	I("%s: Use Protocol Type %c\n", __func__,
	  ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');
	ret = himax_input_register(ts);

	if (ret) {
		E("%s: Unable to register %s input device\n",
		  __func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	ts->initialized = true;
#if defined(CONFIG_FB) || defined(CONFIG_DRM)
	ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");

	if (!ts->himax_att_wq) {
		E(" allocate himax_att_wq failed\n");
		err = -ENOMEM;
		goto err_get_intr_bit_failed;
	}

	INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
	queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(2500));
#endif

#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable = 0;
	wakeup_source_init(&ts->ts_SMWP_wake_lock, HIMAX_common_NAME);
#endif
#ifdef HX_HIGH_SENSE
	ts->HSEN_enable = 0;
#endif

	/*touch data init*/
	err = himax_report_data_init();

	if (err)
		goto err_report_data_init_failed;

	if (himax_common_proc_init()) {
		E(" %s: himax_common proc_init failed!\n", __func__);
		goto err_creat_proc_file_failed;
	}
	himax_sysfs_touchscreen(ts, true);

#if defined(HX_USB_DETECT_CALLBACK)

	if (ts->cable_config)
		cable_detect_register_notifier(&himax_cable_status_handler);

#endif

#ifdef HIMAX_PALM_SENSOR_EN
	if (himax_palm_detect_sensor_init(ts)) {
		E(" %s: himax_palm_detect_sensor_init failed!\n", __func__);
		goto err_create_palm_sensor_failed;
	}
#endif

#ifdef HIMAX_V2_SENSOR_EN
	if (himax_tap_detect_sensor_init(ts)) {
		E(" %s: himax_tap_detect_sensor_init failed!\n", __func__);
		goto err_create_tap_sensor_failed;
	}
#endif

	himax_ts_register_interrupt();

#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
	if (himax_debug_init())
		E(" %s: debug initial failed!\n", __func__);
#endif

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)

	if (g_auto_update_flag)
		himax_int_enable(0);

#endif
	g_hx_chip_inited = true;
	return 0;

#ifdef HIMAX_V2_SENSOR_EN
err_create_tap_sensor_failed:
#endif
#ifdef HIMAX_PALM_SENSOR_EN
err_create_palm_sensor_failed:
#endif
err_creat_proc_file_failed:
	himax_report_data_deinit();
err_report_data_init_failed:
#ifdef HX_SMART_WAKEUP
	wakeup_source_trash(&ts->ts_SMWP_wake_lock);
#endif
#if defined(CONFIG_FB) || defined(CONFIG_DRM)
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
err_get_intr_bit_failed:
#endif
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_detect_failed:
#ifdef HX_RESUME_SET_FW
#if defined(__HIMAX_HX83102D_MOD__)
	cancel_delayed_work_sync(&ts->ts_int_work);
#else
	cancel_work_sync(&ts->ts_int_work);
#endif
	destroy_workqueue(ts->ts_int_workqueue);
err_create_ts_resume_wq_failed:
#endif
#ifdef HX_ZERO_FLASH
	cancel_delayed_work_sync(&ts->work_0f_update);
	destroy_workqueue(ts->himax_0f_update_wq);
err_0f_update_wq_failed:
#endif
#ifdef HX_AUTO_UPDATE_FW
	if (g_auto_update_flag) {
		cancel_delayed_work_sync(&ts->work_update);
		destroy_workqueue(ts->himax_update_wq);
	}
err_update_wq_failed:
#endif
#if defined(HX_USB_DETECT_GLOBAL)
err_register_charger_notify_failed:
	if (ts->charger_notif.notifier_call)
		power_supply_unreg_notifier(&ts->charger_notif);
#endif
error_ic_detect_failed:
	himax_gpio_power_deconfig(pdata);
firmware_name_alloc_failed:
#ifndef CONFIG_OF
err_power_failed:
#endif
err_himax_gpio_power_config_failed:
err_alloc_dt_pdata_failed:
	kfree(hx_touch_data);
	hx_touch_data = NULL;
err_alloc_touch_data_failed:
	kfree(ic_data);
	ic_data = NULL;
err_dt_ic_data_fail:
	kfree(pdata);
	pdata = NULL;
err_dt_platform_data_fail:
	probe_fail_flag = 1;
	return err;
}

void himax_chip_common_deinit(void)
{
	struct himax_ts_data *ts = private_ts;

	himax_ts_unregister_interrupt();

	himax_sysfs_touchscreen(ts, false);

#ifdef CONFIG_TOUCHSCREEN_HIMAX_INSPECT
	himax_inspect_data_clear();
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
	himax_debug_remove();
#endif

	himax_common_proc_deinit();
	himax_report_data_deinit();

#ifdef HX_SMART_WAKEUP
	wakeup_source_trash(&ts->ts_SMWP_wake_lock);
#endif
#if defined(CONFIG_DRM)
	if (himax_active_panel)
		if (drm_panel_notifier_unregister(himax_active_panel, &ts->fb_notif))
			E("Error occurred while unregistering drm_notifier.\n");
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
#elif defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		E("Error occurred while unregistering fb_notifier.\n");
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
#endif
	input_free_device(ts->input_dev);
#ifdef HX_ZERO_FLASH
	cancel_delayed_work_sync(&ts->work_0f_update);
	destroy_workqueue(ts->himax_0f_update_wq);
#endif
#ifdef HX_AUTO_UPDATE_FW
	cancel_delayed_work_sync(&ts->work_update);
	destroy_workqueue(ts->himax_update_wq);
#endif
	himax_gpio_power_deconfig(ts->pdata);
	if (himax_mcu_cmd_struct_free)
		himax_mcu_cmd_struct_free();

	kfree(hx_touch_data);
	hx_touch_data = NULL;
	kfree(ic_data);
	ic_data = NULL;
	kfree(ts->pdata->virtual_key);
	ts->pdata->virtual_key = NULL;
	kfree(ts->pdata);
	ts->pdata = NULL;
	kfree(ts);
	ts = NULL;
	probe_fail_flag = 0;

	I("%s: Common section deinited!\n", __func__);
}

int _himax_chip_common_suspend(struct himax_ts_data *ts)
{
	if (ts->suspended) {
		I("%s: Already suspended. Skipped.\n", __func__);
		goto END;
	} else {
		ts->suspended = true;
		I("%s: enter\n", __func__);
	}

#ifdef HIMAX_PALM_SENSOR_EN
	if (ts->palm_detection_enabled) {
		himax_report_all_leave_event(ts);
		atomic_set(&ts->suspend_mode, 1);
		I("[himax] %s: palm detect function enable, reject suspend\n", __func__);
		goto END;
	}
#endif

	if (debug_data != NULL && debug_data->flash_dump_going == true) {
		I("[himax] %s: Flash dump is going, reject suspend\n", __func__);
		goto END;
	}

#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) || defined(HX_USB_DETECT_GLOBAL)
#ifndef HX_RESUME_SEND_CMD
	g_core_fp.fp_resend_cmd_func(ts->suspended);
#endif
#endif
#ifdef HX_SMART_WAKEUP

	if (ts->SMWP_enable) {

#ifdef HX_CODE_OVERLAY
		if (ts->in_self_test == 0)
			g_core_fp.fp_0f_overlay(2, 0);
#endif

		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		FAKE_POWER_KEY_SEND = false;
		I("[himax] %s: SMART_WAKEUP enable, reject suspend\n", __func__);
		goto END;
	}

#endif
	himax_int_enable(0);
	himax_report_all_leave_event(ts);
	/*if (g_core_fp.fp_suspend_ic_action != NULL)*/
		/*g_core_fp.fp_suspend_ic_action();*/

	if (!ts->use_irq) {
		int32_t cancel_state;

		cancel_state = cancel_work_sync(&ts->work);
		if (cancel_state)
			himax_int_enable(1);
	}

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

	if (ts->pdata)
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
			ts->pdata->power(0);

END:
	if (ts->in_self_test == 1)
		ts->suspend_resume_done = 1;

	I("%s: END\n", __func__);

	return 0;
}

int himax_chip_common_suspend(struct himax_ts_data *ts)
{
	int ret = 0;
#if defined(__HIMAX_HX83102D_MOD__)
	himax_resume_thread_check();
#endif
	mutex_lock(&ts->suspend_resume_mutex);
	ret = _himax_chip_common_suspend(ts);
	mutex_unlock(&ts->suspend_resume_mutex);

	return ret;
}

int _himax_chip_common_resume(struct himax_ts_data *ts)
{
#if defined(HX_ZERO_FLASH) && defined(HX_RESUME_SET_FW)
	int result = 0;
#endif
#if defined(__HIMAX_HX83102D_MOD__)
	uint8_t tmp_data[DATA_LEN_4] = {0};
#endif
	I("%s: enter\n", __func__);

#if defined(__HIMAX_HX83102D_MOD__)
	if (atomic_read(&ts->resume_update_fail)) {
		I("%s:Last time update Fail!\n", __func__);
		atomic_set(&ts->resume_update_fail, 0);
		ts->suspended = false;
		goto START_RESUME;
	}
#endif
	if (ts->suspended == false) {
		I("%s: It had entered resume, skip this step\n", __func__);
		goto END;
	} else {
		ts->suspended = false;
	}
#if defined(__HIMAX_HX83102D_MOD__)
START_RESUME:
#endif

#ifdef HX_ESD_RECOVERY
		/* continuous N times record, not total N times. */
		g_zero_event_count = 0;
#endif

	atomic_set(&ts->suspend_mode, 0);
	ts->diag_cmd = 0;
#ifdef HIMAX_PALM_SENSOR_EN
	if (ts->palm_detection_enabled) {
		I("[himax] %s: palm detect function enable, skip resume\n", __func__);
		goto CHECK_LAZY_SET_END;
	}
#endif

	if (ts->pdata)
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
			ts->pdata->power(1);

#ifndef HX_RESUME_SET_FW
#if defined(HX_RST_PIN_FUNC) && defined(HX_RESUME_HW_RESET)
	if (g_core_fp.fp_ic_reset != NULL)
		g_core_fp.fp_ic_reset(false, false);
#endif
#endif

#if defined(HX_ZERO_FLASH) && defined(HX_RESUME_SET_FW)
#ifdef HX_SMART_WAKEUP
	if (!ts->SMWP_enable) {
#endif
	I("It will update fw after resume in zero flash mode!\n");
	if (g_core_fp.fp_0f_operation_dirly != NULL) {
		result = g_core_fp.fp_0f_operation_dirly();
		if (result) {
			E("Something is wrong! Skip Update with zero flash!\n");
#if defined(__HIMAX_HX83102D_MOD__)
			atomic_set(&ts->resume_update_fail, 1);
#endif
			goto ESCAPE_0F_UPDATE;
		}
	}
	if (g_core_fp.fp_reload_disable != NULL)
		g_core_fp.fp_reload_disable(0);
	if (g_core_fp.fp_sense_on != NULL)
		g_core_fp.fp_sense_on(0x00);
#if defined(__HIMAX_HX83102D_MOD__)
	g_core_fp.fp_register_read(pzf_op->addr_sts_chk, DATA_LEN_4, tmp_data, 0);
	if (tmp_data[0] != 0x05)
		atomic_set(&ts->resume_update_fail, 1);
#endif
#ifdef HX_SMART_WAKEUP
	}
#endif
#endif
#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) || defined(HX_USB_DETECT_GLOBAL)
	if (g_core_fp.fp_resend_cmd_func != NULL)
		g_core_fp.fp_resend_cmd_func(ts->suspended);

#ifdef HX_CODE_OVERLAY
	if (ts->SMWP_enable && ts->in_self_test == 0)
		g_core_fp.fp_0f_overlay(3, 0);
#endif
#endif
	himax_report_all_leave_event(ts);

	if (g_core_fp.fp_sense_on != NULL)
		g_core_fp.fp_resume_ic_action();
	himax_int_enable(1);
#if defined(HX_ZERO_FLASH) && defined(HX_RESUME_SET_FW)
ESCAPE_0F_UPDATE:
#endif

#ifdef HIMAX_PALM_SENSOR_EN
CHECK_LAZY_SET_END:
	if (ts->palm_detection_lazy_set != PALM_SENSOR_LAZY_SET_NONE) {
		_himax_palm_detect_sensor_set_enable(
			(ts->palm_detection_lazy_set == PALM_SENSOR_LAZY_SET_ENABLE) ? 1 : 0);
		I("%s: Palm sensor lazy set done, clear flag.\n", __func__);
		ts->palm_detection_lazy_set = PALM_SENSOR_LAZY_SET_NONE;
	}
#endif
END:
	if (ts->in_self_test == 1)
		ts->suspend_resume_done = 1;

	I("%s: END\n", __func__);
	return 0;
}

int himax_chip_common_resume(struct himax_ts_data *ts)
{
	int ret = 0;
	mutex_lock(&ts->suspend_resume_mutex);
	ret = _himax_chip_common_resume(ts);
	mutex_unlock(&ts->suspend_resume_mutex);

	return ret;
}

