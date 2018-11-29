#ifndef _LDC2114_HDR_
#define _LDC2114_HDR_

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/semaphore.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#define MAX_KEYS 4
#define BUFFER_SIZE 256

#define LDC2114_EV_DATA 1
#define LDC2114_EV_IRQ 2
#define LDC2114_EV_PRESS 3

#define LDC2114_REG_RESET_FULL                  (0x10)
#define LDC2114_REG_RESET_CONFIG_MODE           (0x01)
#define LDC2114_REG_RESET_NONE                  (0x00)

#define LDC2114_REG_STATUS_OUT                  (0x80)
#define LDC2114_REG_STATUS_CHIP_READY           (0x40)
#define LDC2114_REG_STATUS_RDY_TO_WRITE         (0x20)
#define LDC2114_REG_STATUS_MAXOUT               (0x10)
#define LDC2114_REG_STATUS_FSM_WD               (0x08)
#define LDC2114_REG_STATUS_LC_WD                (0x04)
#define LDC2114_REG_STATUS_TIMEOUT              (0x02)
#define LDC2114_REG_STATUS_INTEGRITY            (0x01)
#define LDC2114_REG_STATUS_ERROR_MASK           (0x0f)
#define LDC2114_REG_STATUS_ERROR_OTHERS         (0x1b) /* (ERROR_MASK & ~LC_WD) | MAX_OUT */

enum ldc2114_fields {
	/* Status */
	F_STATUS_TIMEOUT, F_STATUS_LC_WD, F_STATUS_FSM_WD, F_STATUS_MAXOUT,
	F_STATUS_RDY_TO_WRITE, F_STATUS_CHIP_READY, F_STATUS_OUT_STATUS,

	/* Output */
	F_OUT_OUT0, F_OUT_OUT1, F_OUT_OUT2, F_OUT_OUT3,

	/* Reset */
	F_RESET_STATE_RESET, F_RESET_FULL_RESET,

	/* Enable */
	F_EN_EN0, F_EN_EN1, F_EN_EN2, F_EN_EN3,
	F_EN_LPEN0, F_EN_LPEN1, F_EN_LPEN2, F_EN_LPEN3,

	/* Scan rate */
	F_NP_SCAN_RATE_NPSR, F_LP_SCAN_RATE_LPSR,

	/* Gain */
	F_GAIN0_GAIN0, F_GAIN1_GAIN1, F_GAIN2_GAIN2, F_GAIN3_GAIN3,

	/* Interrupt polarity */
	F_INTPOL_INTPOL,

	/* Base Increment */
	F_LP_BASE_INC_LPBI, F_NP_BASE_INC_NPBI,

	/* Max-win */
	F_MAXWIN_MAXWIN0, F_MAXWIN_MAXWIN1, F_MAXWIN_MAXWIN2, F_MAXWIN_MAXWIN3,

	/* Frequency divider */
	F_LC_DIVIDER_LCDIV,

	/* Hysteresis */
	F_HYST_HYST,

	/* Anti-twist */
	F_TWIST_ANTITWST,

	/* Anti-deform */
	F_COMMON_DEFORM_ANTIDFRM0, F_COMMON_DEFORM_ANTIDFRM1,
	F_COMMON_DEFORM_ANTIDFRM2, F_COMMON_DEFORM_ANTIDFRM3,

	/* Anti-common */
	F_COMMON_DEFORM_ANTICM0, F_COMMON_DEFORM_ANTICM1,
	F_COMMON_DEFORM_ANTICM2, F_COMMON_DEFORM_ANTICM3,

	/* Output Polarity */
	F_OPOL_OPOL0, F_OPOL_OPOL1, F_OPOL_OPOL2, F_OPOL_OPOL3,

	/* Counter Scale */
	F_CNTSC_CNTSC0, F_CNTSC_CNTSC1, F_CNTSC_CNTSC2, F_CNTSC_CNTSC3,

	/* Sensor configuration */
	F_SENSOR0_CONFIG_SENCYC0, F_SENSOR0_CONFIG_FREQ0, F_SENSOR0_CONFIG_RP0,
	F_SENSOR1_CONFIG_SENCYC1, F_SENSOR1_CONFIG_FREQ1, F_SENSOR1_CONFIG_RP1,
	F_SENSOR2_CONFIG_SENCYC2, F_SENSOR2_CONFIG_FREQ2, F_SENSOR2_CONFIG_RP2,
	F_SENSOR3_CONFIG_SENCYC3, F_SENSOR3_CONFIG_FREQ3, F_SENSOR3_CONFIG_RP3,

	/* Fast Tracking Factor */
	F_FTF0_FTF0, F_FTF1_2_FTF1, F_FTF1_2_FTF2, F_FTF3_FTF3,

	/* sentinel */
	F_MAX_FIELDS
};

struct ldc2114_cfg_data {
	uint8_t address, value;
};

struct ldc2114_config {
	int size;
	struct ldc2114_cfg_data data[0];
};

struct ldc2114_cfg_info {
	struct ldc2114_config *normal;
	struct ldc2114_config *lpm;
	struct ldc2114_config *active;
	int attempts;
	int delay_ms;
	bool toggled;
	bool reschedulable;
	struct completion done;
};

struct ldc2114_16bit {
	uint8_t lsb;
	uint8_t msb;
} __packed;

struct ldc2114_raw {
	uint8_t out;
	struct ldc2114_16bit values[MAX_KEYS];
} __packed;

struct ldc2114_raw_ext {
	uint8_t status;
	struct ldc2114_raw data;
} __packed;

/**
 * struct ldc2114_data - Instance data for LDC2114
 * @dev: Device structure
 * @regmap - Register map of the device
 * @fields: Register fields of the device
 * @irq: INTB line interrupt number
 */
struct ldc2114_data {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *fields[F_MAX_FIELDS];
	atomic_t irq_work_running;
	atomic_t poll_work_running;
	struct semaphore semaphore;
	int intb_gpio;
	int lpwm_gpio;
	int signal_gpio;
	int intb_polarity;
	int irq;
	int failures;
	int instance;
	bool irq_enabled;
	bool btn_enabled;
	bool data_polling;
	bool not_connected;
	unsigned int poll_interval;
	unsigned int button_map[MAX_KEYS];
	struct delayed_work polling_work;
	struct delayed_work irq_work;
	struct delayed_work config_work;
	struct input_dev *input;
	struct notifier_block poll_nb;
	struct notifier_block reboot_nb;
	unsigned int verno;
	struct ldc2114_cfg_info config;
	struct workqueue_struct *wq;

	int buttons[MAX_KEYS];
	int output_bits;
};

typedef struct data {
	int type;
	struct ldc2114_raw_ext de;
} item_t;

struct buffer {
	struct mutex mutex;
	int capacity;
	unsigned long opened;
	atomic_t start;
	atomic_t end;
	atomic_t active;
	item_t **data;
	wait_queue_head_t wait;
};

struct ldc2114_drv {
	struct dentry *dentry;
	struct device *dev;
	char *name;
	struct buffer ds;
	struct blocking_notifier_head nhead;
};

#if defined(CONFIG_DEBUG_FS)
int ldc2114_debugfs_init(struct ldc2114_data *ldc);
void ldc2114_debugfs_remove(struct ldc2114_data *ldc);
int ldc2114_buffer(struct ldc2114_data *ldc, int type, ...);
int ldc2114_register_client(struct ldc2114_data *ldc, struct notifier_block *nb);
#else
static int ldc2114_debugfs_init(struct ldc2114_data *ldc)
{
	return -ENODEV;
}
static void ldc2114_debugfs_remove(struct ldc2114_data *ldc)
{
}
static int ldc2114_buffer(struct ldc2114_data *ldc, int type, ...)
{
	return 0;
}
static int ldc2114_register_client(struct ldc2114_data *ldc, struct notifier_block *nb)
{
	return -ENODEV;
}
#endif
#endif
