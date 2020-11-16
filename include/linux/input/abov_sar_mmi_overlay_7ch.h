/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef ABOV_SAR_H
#define ABOV_SAR_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#define KEY_CAP_CS0              0x270
#define KEY_CAP_CS1              0x271
#define KEY_CAP_CS2              0x272
#define KEY_CAP_CS3              0x273
#define KEY_CAP_CS4              0x274
#define KEY_CAP_COMB             0x272
/*
 *  I2C Registers
 */
#define ABOV_VERSION_REG		    0x01
#define ABOV_MODELNO_REG		    0x02
#define ABOV_VENDOR_ID_REG		    0x03
#define ABOV_IRQSTAT_REG			0x04
#define ABOV_IRQSTAT1_REG			0x05
#define ABOV_SOFTRESET_REG  		0x06
#define ABOV_CTRL_MODE_REG			0x07
#define ABOV_CTRL_CHANNEL_REG		0x08
#define ABOV_RECALI_REG				0xFB
#define ABOV_CHANNEL_NUMBER_TWO     0x02
#define ABOV_CHANNEL_NUMBER_THREE   0x03
#define ABOV_CHANNEL_NUMBER_FOUR    0x04
#define ABOV_CHANNEL_NUMBER_FIVE    0x05
#define ABOV_CTRL_MODE_ACTIVE		0x00
#define ABOV_CTRL_MODE_SLEEP		0x01
#define ABOV_CTRL_MODE_STOP			0x02

/* enable body stat */
#define ABOV_TCHCMPSTAT_TCHSTAT0_FLAG   0x0003
/* enable body stat */
#define ABOV_TCHCMPSTAT_TCHSTAT1_FLAG   0x000C
/* enable body stat */
#define ABOV_TCHCMPSTAT_TCHSTAT2_FLAG   0x0030
/* enable body stat */
#define ABOV_TCHCMPSTAT_TCHSTAT3_FLAG   0x00C0
/* enable body stat */
#define ABOV_TCHCMPSTAT_TCHSTAT4_FLAG   0x0300

/* cap_button_mask */
#define ABOV_CAP_BUTTON_2CH_MASK   0x05
#define ABOV_CAP_BUTTON_3CH_MASK   0x15
#define ABOV_CAP_BUTTON_4CH_MASK   0x55
#define ABOV_CAP_BUTTON_5CH_MASK   0x155

/* Flash erase time */
#define ABOV_T346_T349_FLASH_ERASE_TIME   1400
#define ABOV_T376_FLASH_ERASE_TIME        3000

/*
 * A96T346DF/A96T349DF FW MODE:0x39
 * A96T346HW/A96T349HW FW MODE:0x3A
 * A96T376EF FW MODE:0x70
 * A96T375DF FW MODE:0x71
 */
#define ABOV_DF_FW_MODE    0x39
#define ABOV_HW_FW_MODE    0x3A
#define ABOV_T376_FW_MODE  0x70
#define ABOV_T375_FW_MODE  0x71

/**************************************
* define platform data
*
**************************************/
struct smtc_reg_data {
	unsigned char reg;
	unsigned char val;
};

typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;


struct _buttonInfo {
	/* The Key to send to the input */
	int keycode;
	/* Mask to look for on Touch Status */
	int mask;
	/* Current state of button. */
	int state;
};

struct _totalButtonInformation {
	struct _buttonInfo *buttons;
	int buttonSize;
	struct input_dev *input_ch0;
	struct input_dev *input_ch1;
	struct input_dev *input_ch2;
	struct input_dev *input_ch3;
	struct input_dev *input_ch4;
};

typedef struct _totalButtonInformation buttonInformation_t;
typedef struct _totalButtonInformation *pbuttonInformation_t;

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data abov_i2c_reg_setup[] = {
	{
		.reg = ABOV_CTRL_MODE_REG,
		.val = 0x00,
	},
	{
		.reg = ABOV_CTRL_CHANNEL_REG,
		.val = 0x1F,
	},
	{
		.reg = ABOV_RECALI_REG,
		.val = 0x01,
	},
};



static struct _buttonInfo psmtcButtons[] = {
	{
		.keycode = KEY_CAP_CS0,
		.mask = ABOV_TCHCMPSTAT_TCHSTAT0_FLAG,
	},
	{
		.keycode = KEY_CAP_CS1,
		.mask = ABOV_TCHCMPSTAT_TCHSTAT1_FLAG,
	},
	{
		.keycode = KEY_CAP_CS2,
		.mask = ABOV_TCHCMPSTAT_TCHSTAT2_FLAG,
	},
	{
		.keycode = KEY_CAP_CS3,
		.mask = ABOV_TCHCMPSTAT_TCHSTAT3_FLAG,
	},
	{
		.keycode = KEY_CAP_CS4,
		.mask = ABOV_TCHCMPSTAT_TCHSTAT4_FLAG,
	},
};

struct abov_platform_data {
	int i2c_reg_num;
	struct smtc_reg_data *pi2c_reg;
	struct regulator *cap_vdd;
	struct regulator *cap_svdd;
	bool cap_vdd_en;
	bool cap_svdd_en;
	unsigned irq_gpio;
	/* used for custom setting for channel and scan period */
	int cap_channel_ch0;
	int cap_channel_ch1;
	int cap_channel_ch2;
	int cap_channel_ch3;
	int cap_channel_ch4;

	const char *cap_ch0_name;
	const char *cap_ch1_name;
	const char *cap_ch2_name;
	const char *cap_ch3_name;
	const char *cap_ch4_name;
	pbuttonInformation_t pbuttonInformation;
	const char *fw_name;
	int fw_mode_ret;
	int cap_button_mask;

	int (*get_is_nirq_low)(unsigned irq_gpio);
	int (*init_platform_hw)(void);
	void (*exit_platform_hw)(void);
};
typedef struct abov_platform_data abov_platform_data_t;
typedef struct abov_platform_data *pabov_platform_data_t;

#ifdef USE_SENSORS_CLASS
static struct sensors_classdev sensors_capsensor_ch0_cdev = {
	//.name = "capsense_top",
	.vendor = "abov",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_CAPSENSE,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct sensors_classdev sensors_capsensor_ch1_cdev = {
	//.name = "capsense_bottom",
	.vendor = "abov",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_CAPSENSE,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct sensors_classdev sensors_capsensor_ch2_cdev = {
	//.name = "capsense_bottom_left",
	.vendor = "abov",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_CAPSENSE,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct sensors_classdev sensors_capsensor_ch3_cdev = {
	//.name = "capsense_bottom_left",
	.vendor = "abov",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_CAPSENSE,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct sensors_classdev sensors_capsensor_ch4_cdev = {
	//.name = "capsense_bottom_left",
	.vendor = "abov",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_CAPSENSE,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif


typedef enum{
	CAPSENSOR_ENABLE_FLAG_CH0=1,
	CAPSENSOR_ENABLE_FLAG_CH1=2,
	CAPSENSOR_ENABLE_FLAG_CH2=4,
	CAPSENSOR_ENABLE_FLAG_CH3=8,
	CAPSENSOR_ENABLE_FLAG_CH4=16,
	CAPSENSOR_ENABLE_FLAG_MAX
}capsensor_enable_flag_t;
/***************************************
* define data struct/interrupt
* @pdev: pdev common device struction for linux
* @dworker: work struct for worker function
* @board: constant pointer to platform data
* @mutex: mutex for interrupt process
* @lock: Spin Lock used for nirq worker function
* @bus: either i2c_client or spi_client
* @pDevice: device specific struct pointer
*@read_flag : used for dump specified register
* @irq: irq number used
* @irqTimeout: msecs only set if useIrqTimer is true
* @irq_disabled: whether irq should be ignored
* @irq_gpio: irq gpio number
* @useIrqTimer: older models need irq timer for pen up cases
* @read_reg: record reg address which want to read
*@cust_prox_ctrl0 : used for custom setting for channel and scan period
* @init: (re)initialize device
* @get_nirq_low: get whether nirq is low (platform data)
* @statusFunc: array of functions to call for corresponding status bit
***************************************/
#define USE_THREADED_IRQ

#define MAX_NUM_STATUS_BITS (8)

typedef struct abovXX abovXX_t, *pabovXX_t;
struct abovXX {
	struct device *pdev;
	struct delayed_work dworker;
	struct abov_platform_data *board;
#if defined(USE_THREADED_IRQ)
	struct mutex mutex;
#else
	spinlock_t	lock;
#endif
	void *bus;
	void *pDevice;
	int read_flag;
	int irq;
	int irqTimeout;
	/* interrupt check flag */
	int int_state;
	char irq_disabled;
	/* whether irq should be ignored.. cases if enable/disable irq is not used
	 * or does not work properly */
	u8 useIrqTimer;
	u8 read_reg;

	// sensors enable flag
	capsensor_enable_flag_t enable_flag;

	struct work_struct ps_notify_work;
	struct notifier_block ps_notif;
	bool ps_is_present;
	bool loading_fw;
	int fw_dl_status;

	struct work_struct fw_update_work;

	/* Function Pointers */
	int (*init)(pabovXX_t this);
	/* since we are trying to avoid knowing registers, create a pointer to a
	 * common read register which would be to read what the interrupt source
	 * is from
	 */
	int (*get_nirq_low)(unsigned irq_gpio);
	void (*statusFunc[MAX_NUM_STATUS_BITS])(pabovXX_t this);
};

void abovXX_suspend(pabovXX_t this);
void abovXX_resume(pabovXX_t this);
int abovXX_sar_init(pabovXX_t this);
int abovXX_sar_remove(pabovXX_t this);

#endif
