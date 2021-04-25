/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
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

#ifndef _SYNAPTICS_DSX_RMI4_H_
#define _SYNAPTICS_DSX_RMI4_H_

#define SYNAPTICS_DSX_DRIVER_VERSION "DSX 1.1"

/* define to enable USB charger detection */
#undef USB_CHARGER_DETECTION

#include <linux/version.h>
#include <linux/ktime.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/pm_qos.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pinctrl/consumer.h>
#if defined(USB_CHARGER_DETECTION)
#include <linux/usb.h>
#include <linux/power_supply.h>
#endif
#if defined(CONFIG_PANEL_NOTIFICATIONS)
#include <linux/panel_notifier.h>
#elif defined(CONFIG_DRM)
#include <linux/msm_drm_notify.h>
#endif

#include <linux/input/synaptics_dsx_mmi.h>
#include <linux/mmi_wake_lock.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
#define KERNEL_ABOVE_3_7
#endif

#define CLASS_PRIMARY_FNAME "primary"
#define DRIVER_NAME "synaptics_mmi"
#define INPUT_PHYS_NAME "synaptics_dsx_i2c/input"
#define TYPE_B_PROTOCOL

#define RMI4_WAIT_READY 0
#define RMI4_HW_RESET 1
#define RMI4_SW_RESET 2

#define NO_0D_WHILE_2D
/* #define REPORT_2D_Z */
#define REPORT_2D_W

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_DET_INTERVAL 1000 /* ms */
#define POLLING_PERIOD 1 /* ms */
#define SYN_I2C_RETRY_TIMES 10
#define MAX_ABS_MT_TOUCH_MAJOR 15
#define SYN_MAX_BUTTONS 4

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F01_QUERY1_OFFSET 1
#define F01_QUERY21_OFFSET 21
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12
#define F12_STD_QUERY_LEN 10
#define F12_STD_CTRL_LEN 4
#define F12_STD_DATA_LEN 80

#define RESET_GPIO_NAME "touch_reset"
#define IRQ_GPIO_NAME "touch_irq"

#define PDT_PROPS (0x00EF)
#define PDT_START (0x00E9)
#define PDT_END (0x000A)
#define PDT_ENTRY_SIZE (0x0006)
#define PAGES_TO_SERVICE (10)
#define PAGE_SELECT_LEN (2)
#define PAGE_SELECT_ENFORCE (0x8000)

#define SYNAPTICS_RMI4_F01 (0x01)
#define SYNAPTICS_RMI4_F11 (0x11)
#define SYNAPTICS_RMI4_F12 (0x12)
#define SYNAPTICS_RMI4_F1A (0x1a)
#define SYNAPTICS_RMI4_F34 (0x34)
#define SYNAPTICS_RMI4_F35 (0x35)
#define SYNAPTICS_RMI4_F38 (0x38)
#define SYNAPTICS_RMI4_F51 (0x51)
#define SYNAPTICS_RMI4_F54 (0x54)
#define SYNAPTICS_RMI4_F55 (0x55)
#define SYNAPTICS_RMI4_FDB (0xdb)

#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_BUILD_ID_SIZE 3
#define SYNAPTICS_RMI4_SERIAL_SIZE 7
#define SYNAPTICS_RMI4_CONFIG_ID_SIZE 4
#define SYNAPTICS_RMI4_PACKAGE_ID_SIZE 4
#define SYNAPTICS_RMI4_FILENAME_SIZE 80

#define PACKAGE_ID_OFFSET 17
#define FW_VERSION_OFFSET 18
#define F34_PROPERTIES_OFFSET 1
#define F34_PROPERTIES_OFFSET_V2 0

#define MAX_NUMBER_OF_FINGERS 10
#define MAX_NUMBER_OF_BUTTONS 4
#define MAX_INTR_REGISTERS 4

#define MASK_16BIT 0xFFFF
#define MASK_8BIT 0xFF
#define MASK_7BIT 0x7F
#define MASK_6BIT 0x3F
#define MASK_5BIT 0x1F
#define MASK_4BIT 0x0F
#define MASK_3BIT 0x07
#define MASK_2BIT 0x03
#define MASK_1BIT 0x01

#define PRODUCT_INFO_SIZE 2
#define PRODUCT_ID_SIZE 10

/*
 * struct synaptics_rmi4_fn_desc - function descriptor fields in PDT
 * @query_base_addr: base address for query registers
 * @cmd_base_addr: base address for command registers
 * @ctrl_base_addr: base address for control registers
 * @data_base_addr: base address for data registers
 * @intr_src_count: number of interrupt sources
 * @fn_number: function number
 */
struct synaptics_rmi4_fn_desc {
	unsigned char query_base_addr;
	unsigned char cmd_base_addr;
	unsigned char ctrl_base_addr;
	unsigned char data_base_addr;
	unsigned char intr_src_count:3;
	unsigned char reserved_1:2;
	unsigned char fn_version:2;
	unsigned char reserved_2:1;
	unsigned char fn_number;
};

/*
 * synaptics_rmi4_fn_full_addr - full 16-bit base addresses
 * @query_base: 16-bit base address for query registers
 * @cmd_base: 16-bit base address for data registers
 * @ctrl_base: 16-bit base address for command registers
 * @data_base: 16-bit base address for control registers
 */
struct synaptics_rmi4_fn_full_addr {
	unsigned short query_base;
	unsigned short cmd_base;
	unsigned short ctrl_base;
	unsigned short data_base;
};

/*
 * struct synaptics_rmi4_fn - function handler data structure
 * @fn_number: function number
 * @num_of_data_sources: number of data sources
 * @num_of_data_points: maximum number of fingers supported
 * @size_of_data_register_block: data register block size
 * @data1_offset: offset to data1 register from data base address
 * @intr_reg_num: index to associated interrupt register
 * @intr_mask: interrupt mask
 * @full_addr: full 16-bit base addresses of function registers
 * @link: linked list for function handlers
 * @data_size: size of private data
 * @data: pointer to private data
 */
struct synaptics_rmi4_fn {
	unsigned char fn_number;
	unsigned char num_of_data_sources;
	unsigned char num_of_data_points;
	unsigned char size_of_data_register_block;
	unsigned char data1_offset;
	unsigned char intr_reg_num;
	unsigned char intr_mask;
	struct synaptics_rmi4_fn_full_addr full_addr;
	struct list_head link;
	int data_size;
	void *data;
};

/*
 * struct synaptics_rmi4_device_info - device information
 * @version_major: rmi protocol major version number
 * @version_minor: rmi protocol minor version number
 * @manufacturer_id: manufacturer id
 * @product_props: product properties information
 * @product_info: product info array
 * @date_code: device manufacture date
 * @tester_id: tester id array
 * @serial_number: device serial number
 * @product_id_string: device product id
 * @support_fn_list: linked list for function handlers
 */
struct synaptics_rmi4_device_info {
	unsigned int version_major;
	unsigned int version_minor;
	unsigned char manufacturer_id;
	unsigned char product_props;
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	unsigned char serial[SYNAPTICS_RMI4_SERIAL_SIZE];
	unsigned char package_id[SYNAPTICS_RMI4_PACKAGE_ID_SIZE];
	unsigned char product_id_string[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char build_id[SYNAPTICS_RMI4_BUILD_ID_SIZE];
	unsigned char config_id[SYNAPTICS_RMI4_CONFIG_ID_SIZE];
	struct list_head support_fn_list;
};

#define TOUCH_UD_BUFF 20
#define BUTTON_UD_BUFF 10

struct touch_up_down {
	int mismatch;
	unsigned char up_down;
	unsigned int counter;
};

struct touch_area_stats {
	struct touch_up_down *ud;
	ssize_t ud_len;
	ssize_t ud_id;
	ssize_t unknown_counter;
	const char *name;
};

struct synaptics_dsx_func_patch {
	unsigned short func;
	unsigned char regstr;
	unsigned char subpkt;
	unsigned char size;
	unsigned char bitmask;
	unsigned char *data;
	struct list_head link;
};

struct synaptics_clip_area {
	unsigned xul_clip, yul_clip, xbr_clip, ybr_clip;
	unsigned inversion; /* clip inside (when 1) or ouside otherwise */
};

enum {
	SYNA_MOD_AOD,
	SYNA_MOD_STATS,
	SYNA_MOD_FOLIO,
	SYNA_MOD_CHARGER,
	SYNA_MOD_WAKEUP,
	SYNA_MOD_FPS,
	SYNA_MOD_QUERY,	/* run time query; active only */
	SYNA_MOD_RT,	/* run time patch; active only; always effective */
	SYNA_MOD_MAX
};

#define FLAG_FORCE_UPDATE	1
#define FLAG_WAKEABLE		2
#define FLAG_POWER_SLEEP	4

struct synaptics_dsx_patch {
	const char *name;
	int	cfg_num;
	unsigned int flags;
	struct semaphore list_sema;
	struct list_head cfg_head;
};

struct config_modifier {
	const char *name;
	int id;
	bool effective;
	struct synaptics_dsx_patch *active;
	struct synaptics_dsx_patch *suspended;
	struct synaptics_clip_area *clipa;
	struct list_head link;
};

struct synaptics_dsx_modifiers {
	int	mods_num;
	struct semaphore list_sema;
	struct list_head mod_head;

};

struct f34_properties {
	union {
		struct {
			unsigned char regmap:1;
			unsigned char unlocked:1;
			unsigned char has_config_id:1;
			unsigned char has_perm_config:1;
			unsigned char has_bl_config:1;
			unsigned char has_display_config:1;
			unsigned char has_blob_config:1;
			unsigned char reserved:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_properties_v2 {
	union {
		struct {
			unsigned char query0_subpkt1_size:3;
			unsigned char has_config_id:1;
			unsigned char reserved:1;
			unsigned char has_thqa:1;
			unsigned char reserved2:2;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_control_95n {
	union {
		struct {
			/* byte 0 - flags*/
			unsigned char c95_filter_bw:3;
			unsigned char c95_byte0_b3_b6:4;
			unsigned char c95_disable:1;

			/* bytes 1 - 10 */
			unsigned char c95_first_burst_length_lsb;
			unsigned char c95_first_burst_length_msb;
			unsigned char c95_addl_burst_length_lsb;
			unsigned char c95_addl_burst_length_msb;
			unsigned char c95_i_stretch;
			unsigned char c95_r_stretch;
			unsigned char c95_noise_control1;
			unsigned char c95_noise_control2;
			unsigned char c95_noise_control3;
			unsigned char c95_noise_control4;
		} __packed;
		struct {
			unsigned char data[11];
		} __packed;
	};
};

enum {
	STATE_UNKNOWN,
	STATE_ACTIVE,
	STATE_SUSPEND,
	STATE_STANDBY = 4,
	STATE_BL,
	STATE_INIT,
	STATE_FLASH,
	STATE_QUERY,
	STATE_LOADING,
	STATE_INVALID
};

#define STATE_UI       (STATE_ACTIVE | STATE_SUSPEND)

enum ic_modes {
	IC_MODE_ANY = 0,
	IC_MODE_BL,
	IC_MODE_UI
};

enum exp_fn {
	RMI_DEV = 0,
	RMI_F34,
	RMI_F54,
	RMI_FW_UPDATER,
	RMI_CTRL_ACCESS_BLK,
	RMI_DRM_FRAMEWORK,
	RMI_LAST,
};

static inline void batohs(unsigned short *dest, unsigned char *src)
{
	*dest = src[1] * 0x100 + src[0];
}

static inline void hstoba(unsigned char *dest, unsigned short src)
{
	dest[0] = src % 0x100;
	dest[1] = src / 0x100;
}

static inline void batohui(unsigned int *dest, unsigned char *src, size_t size)
{
	int si = size;
	*dest = 0;
	for (; --si >= 0;)
		*dest += src[si] << (si << 3);
}

struct synaptics_rmi4_subpkt {
	bool present;
	bool expected;
	short offset;
	unsigned int size;
	void *data;
};

struct synaptics_rmi4_packet_reg {
	unsigned short r_number;
	bool updated;	/* indicate that value in *data has been updated */
	bool modified;	/* indicate that value in *data has been modified */
	bool expected;
	short offset;
	unsigned int size;
	unsigned char *data;
	unsigned char nr_subpkts;
	struct synaptics_rmi4_subpkt *subpkt;
};

#define MAX_CONFIG_REGS 10

struct synaptics_rmi4_func_packet_regs {
	unsigned short f_number;
	unsigned short base_addr;
	unsigned short query_offset;
	int nr_regs;
	struct synaptics_rmi4_packet_reg *regs;
};

struct reporting_ctrl {
	bool reporting_stopped;
	unsigned int events_cnt;
	int expected, max_seen;
	struct semaphore ctrl_sema;
};

#define MAX_READ_WRITE_SIZE 8096
#define MIN_READ_WRITE_BUF_SIZE 256

struct temp_buffer {
	unsigned char *buf;
	unsigned short buf_size;
};

struct synaptics_exp_fn_ctrl {
	bool inited;
	struct mutex ctrl_mutex;
	struct mutex list_mutex;
	struct list_head fn_list;
	struct delayed_work det_work;
};

/*
 * struct synaptics_rmi4_data - rmi4 device instance data
 * @i2c_client: pointer to associated i2c client
 * @input_dev: pointer to associated input device
 * @board: constant pointer to platform data
 * @rmi4_mod_info: device information
 * @regulator: pointer to associated regulator
 * @vdd_quirk: pointer to associated regulator for 'quirk' config
 * @rmi4_io_ctrl_mutex: mutex for i2c i/o control
 * @det_work: work thread instance for expansion function detection
 * @det_workqueue: pointer to work queue for work thread instance
 * @early_suspend: instance to support early suspend power management
 * @current_page: current page in sensor to acess
 * @button_0d_enabled: flag for 0d button support
 * @full_pm_cycle: flag for full power management cycle in early suspend stage
 * @num_of_intr_regs: number of interrupt registers
 * @f01_query_base_addr: query base address for f01
 * @f01_cmd_base_addr: command base address for f01
 * @f01_ctrl_base_addr: control base address for f01
 * @f01_data_base_addr: data base address for f01
 * @irq: attention interrupt
 * @sensor_max_x: sensor maximum x value
 * @sensor_max_y: sensor maximum y value
 * @irq_enabled: flag for indicating interrupt enable status
 * @touch_stopped: touch is in suspend state
 * @flash_enabled: allow flashing once transition to active state is complete
 * @ic_on: touch ic power state
 * @fingers_on_2d: flag to indicate presence of fingers in 2d area
 * @number_irq: total number of remembered interrupt times
 * @last_irq: last interrup time's number (index of the location of interrupt)
 * @irq_info:  information about last few interrupt times
 * @i2c_read: pointer to i2c read function
 * @i2c_write: pointer to i2c write function
 * @irq_enable: pointer to irq enable function
 */
struct synaptics_rmi4_data {
	struct i2c_client *i2c_client;
	struct device *ts_class_dev;
	struct input_dev *input_dev;
	struct synaptics_dsx_platform_data board;
	struct synaptics_rmi4_device_info rmi4_mod_info;
	struct regulator *regulator;
	struct regulator *vdd_quirk;
	struct mutex rmi4_io_ctrl_mutex;
	struct mutex state_mutex;
	atomic_t panel_off_flag;
	unsigned char current_page;
	unsigned char button_0d_enabled;
	unsigned char num_of_rx;
	unsigned char num_of_tx;
	unsigned char num_of_fingers;
	unsigned char aod_mt;
	unsigned char f01_ctrl_register_0;
	unsigned char intr_mask[MAX_INTR_REGISTERS];
	unsigned short num_of_intr_regs;
	unsigned short f01_query_base_addr;
	unsigned short f01_cmd_base_addr;
	unsigned short f01_ctrl_base_addr;
	unsigned short f01_data_base_addr;
	unsigned int active_fn_intr_mask;
	int state;
	int irq;
	int ctrl_dsi;
	const char *class_entry_name;
	const char *bound_display;
	int sensor_max_x;
	int sensor_max_y;
	bool irq_enabled;
	atomic_t touch_stopped;
	atomic_t query_done;
	bool splash_screen_mode;
	bool flash_enabled;
	bool ic_on;
	bool fingers_on_2d;
	bool input_registered;
	bool in_bootloader;
	int (*i2c_read)(struct synaptics_rmi4_data *pdata, unsigned short addr,
			unsigned char *data, unsigned short length);
	int (*i2c_write)(struct synaptics_rmi4_data *pdata, unsigned short addr,
			unsigned char *data, unsigned short length);
	void (*set_state)(struct synaptics_rmi4_data *rmi4_data, int state);
	int (*ready_state)(struct synaptics_rmi4_data *rmi4_data, bool standby);
	int (*irq_enable)(struct synaptics_rmi4_data *rmi4_data, bool enable);
	int (*reset_device)(struct synaptics_rmi4_data *rmi4_data);
	int (*get_status)(struct synaptics_rmi4_data *rmi4_data);
	int number_irq;
	int last_irq;
	struct synaptics_rmi4_irq_info *irq_info;

	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;

	/* features enablers */
	bool charger_detection_enabled;
	bool folio_detection_enabled;
	bool fps_detection_enabled;
	bool wakeup_detection_enabled;
	bool patching_enabled;
	bool purge_enabled;

	bool suspend_is_wakeable;
	bool clipping_on;
	struct synaptics_clip_area *clipa;
	struct synaptics_dsx_modifiers modifiers;

	struct work_struct resume_work;
	struct synaptics_rmi4_func_packet_regs *f12_data_registers_ptr;
	struct notifier_block rmi_reboot;
#if defined(USB_CHARGER_DETECTION)
	struct power_supply psy;
#endif
#ifdef CONFIG_MMI_HALL_NOTIFICATIONS
	struct notifier_block folio_notif;
#endif
	bool is_fps_registered;	/* FPS notif registration might be delayed */
	struct notifier_block fps_notif;

	struct mutex rmi4_exp_init_mutex;
	uint32_t pm_qos_latency;

	struct reporting_ctrl rctrl;
	unsigned char tsb_buff_clean_flag;

	bool touch_data_contiguous;
	uint8_t *touch_data;
	uint16_t touch_data_size;

	struct temp_buffer write_buf;

#if defined(CONFIG_DYNAMIC_DEBUG) || defined(DEBUG)
	/* TEST OPTIONS */
	int test_irq_delay_ms;
	int test_irq_data_contig;
#endif

	struct synaptics_exp_fn_ctrl exp_fn_ctrl;
	struct list_head node;
	struct semaphore reset_semaphore;
	int drm_state;
	int instance;
	char *irq_name;
	char *reset_name;

	void *fwu_data;
	void *rmidev_data;
	void *f54_data;

	int (*scan_f54_ctrl_regs)(struct synaptics_rmi4_data *data,
				struct synaptics_rmi4_func_packet_regs *regs);
	int (*scan_f54_cmd_regs)(struct synaptics_rmi4_data *data,
				struct synaptics_rmi4_func_packet_regs *regs);
	int (*scan_f54_query_regs)(struct synaptics_rmi4_data *data,
				struct synaptics_rmi4_func_packet_regs *regs);
	int (*scan_f54_data_regs)(struct synaptics_rmi4_data *data,
				struct synaptics_rmi4_func_packet_regs *regs);

	struct touch_area_stats touch_ud_stats;
	struct touch_area_stats button_ud_stats;

	struct synaptics_rmi4_func_packet_regs config_regs[MAX_CONFIG_REGS];
};

enum {
	DRM_ST_UNDEF = 0,
	DRM_ST_READY,
	DRM_ST_TERM,
	DRM_ST_MAX
};

#define ASSERT(s) (rmi4_data->drm_state == s)

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct partition_table {
	unsigned char partition_id:5;
	unsigned char byte_0_reserved:3;
	unsigned char byte_1_reserved;
	unsigned char partition_length_7_0;
	unsigned char partition_length_15_8;
	unsigned char start_physical_address_7_0;
	unsigned char start_physical_address_15_8;
	unsigned char partition_properties_7_0;
	unsigned char partition_properties_15_8;
} __packed;

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_query_0 {
	union {
		struct {
			unsigned char subpacket_1_size:3;
			unsigned char has_config_id:1;
			unsigned char f34_query0_b4:1;
			unsigned char has_thqa:1;
			unsigned char f34_query0_b6__7:2;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_query_1_7 {
	union {
		struct {
			/* query 1 */
			unsigned char bl_minor_revision;
			unsigned char bl_major_revision;

			/* query 2 */
			unsigned char bl_fw_id_7_0;
			unsigned char bl_fw_id_15_8;
			unsigned char bl_fw_id_23_16;
			unsigned char bl_fw_id_31_24;

			/* query 3 */
			unsigned char minimum_write_size;
			unsigned char block_size_7_0;
			unsigned char block_size_15_8;
			unsigned char flash_page_size_7_0;
			unsigned char flash_page_size_15_8;

			/* query 4 */
			unsigned char adjustable_partition_area_size_7_0;
			unsigned char adjustable_partition_area_size_15_8;

			/* query 5 */
			unsigned char flash_config_length_7_0;
			unsigned char flash_config_length_15_8;

			/* query 6 */
			unsigned char payload_length_7_0;
			unsigned char payload_length_15_8;

			/* query 7 */
			unsigned char f34_query7_b0:1;
			unsigned char has_bootloader:1;
			unsigned char has_device_config:1;
			unsigned char has_flash_config:1;
			unsigned char has_manufacturing_block:1;
			unsigned char has_guest_serialization:1;
			unsigned char has_global_parameters:1;
			unsigned char has_core_code:1;
			unsigned char has_core_config:1;
			unsigned char has_guest_code:1;
			unsigned char has_display_config:1;
			unsigned char f34_query7_b11__15:5;
			unsigned char f34_query7_b16__23;
			unsigned char f34_query7_b24__31;
		} __packed;
		unsigned char data[21];
	};
};

struct f34_v7_data_1_5 {
	union {
		struct {
			unsigned char partition_id:5;
			unsigned char f34_data1_b5__7:3;
			unsigned char block_offset_7_0;
			unsigned char block_offset_15_8;
			unsigned char transfer_length_7_0;
			unsigned char transfer_length_15_8;
			unsigned char command;
			unsigned char payload_0;
			unsigned char payload_1;
		} __packed;
		unsigned char data[8];
	};
};

struct f34_v5v6_flash_properties {
	union {
		struct {
			unsigned char reg_map:1;
			unsigned char unlocked:1;
			unsigned char has_config_id:1;
			unsigned char has_pm_config:1;
			unsigned char has_bl_config:1;
			unsigned char has_disp_config:1;
			unsigned char has_ctrl1:1;
			unsigned char has_query4:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v5v6_flash_properties_2 {
	union {
		struct {
			unsigned char has_guest_code:1;
			unsigned char reserved:7;
		} __packed;
		unsigned char data[1];
	};
};

struct register_offset {
	unsigned char properties;
	unsigned char properties_2;
	unsigned char block_size;
	unsigned char block_count;
	unsigned char gc_block_count;
	unsigned char flash_status;
	unsigned char partition_id;
	unsigned char block_number;
	unsigned char transfer_length;
	unsigned char flash_cmd;
	unsigned char payload;
};

struct block_count {
	unsigned short ui_firmware;
	unsigned short ui_config;
	unsigned short dp_config;
	unsigned short fl_config;
	unsigned short pm_config;
	unsigned short bl_config;
	unsigned short lockdown;
	unsigned short guest_code;
};

struct physical_address {
	unsigned short ui_firmware;
	unsigned short ui_config;
	unsigned short dp_config;
	unsigned short guest_code;
};

struct container_descriptor {
	unsigned char content_checksum[4];
	unsigned char container_id[2];
	unsigned char minor_version;
	unsigned char major_version;
	unsigned char reserved_08;
	unsigned char reserved_09;
	unsigned char reserved_0a;
	unsigned char reserved_0b;
	unsigned char container_option_flags[4];
	unsigned char content_options_length[4];
	unsigned char content_options_address[4];
	unsigned char content_length[4];
	unsigned char content_address[4];
};

struct image_header_10 {
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char minor_header_version;
	unsigned char major_header_version;
	unsigned char reserved_08;
	unsigned char reserved_09;
	unsigned char reserved_0a;
	unsigned char reserved_0b;
	unsigned char top_level_container_start_addr[4];
};

struct image_header_05_06 {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_bootloader:1;
	unsigned char options_guest_code:1;
	unsigned char options_tddi:1;
	unsigned char options_reserved:4;
	unsigned char header_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char bootloader_addr[4];
	unsigned char bootloader_size[4];
	unsigned char ui_addr[4];
	unsigned char ui_size[4];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	union {
		struct {
			unsigned char cstmr_product_id[PRODUCT_ID_SIZE];
			unsigned char reserved_4a_4f[6];
		};
		struct {
			unsigned char dsp_cfg_addr[4];
			unsigned char dsp_cfg_size[4];
			unsigned char reserved_48_4f[8];
		};
	};
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct block_data {
	unsigned int size;
	const unsigned char *data;
};

struct image_metadata {
	bool contains_firmware_id;
	bool contains_bootloader;
	bool contains_disp_config;
	bool contains_guest_code;
	bool contains_flash_config;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int bootloader_size;
	unsigned int disp_config_offset;
	unsigned char bl_version;
	unsigned char product_id[PRODUCT_ID_SIZE + 1];
	unsigned char cstmr_product_id[PRODUCT_ID_SIZE + 1];
	struct block_data bootloader;
	struct block_data ui_firmware;
	struct block_data ui_config;
	struct block_data dp_config;
	struct block_data fl_config;
	struct block_data bl_config;
	struct block_data guest_code;
	struct block_data lockdown;
	struct block_count blkcount;
	struct physical_address phyaddr;
};

enum bl_version {
	BL_V5 = 5,
	BL_V6 = 6,
	BL_V7 = 7,
	BL_V8 = 8,
};

struct synaptics_rmi4_fwu_handle {
	enum bl_version bl_version;
	bool initialized;
	bool in_bl_mode;
	bool in_ub_mode;
	bool force_update;
	bool do_lockdown;
	bool has_guest_code;
	bool new_partition_table;
	bool has_erase_all;
	unsigned int data_pos;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char config_id[32];
	unsigned char flash_status;
	unsigned char partitions;
	unsigned short block_size;
	unsigned short config_size;
	unsigned short config_area;
	unsigned short config_block_count;
	unsigned short flash_config_length;
	unsigned short payload_length;
	unsigned short partition_table_bytes;
	unsigned short read_config_buf_size;
	const unsigned char *config_data;
	const unsigned char *image;
	unsigned char *image_name;
	unsigned int image_size;
	struct image_metadata img;
	struct register_offset off;
	struct block_count blkcount;
	struct physical_address phyaddr;
	struct f34_v5v6_flash_properties flash_properties;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_fn_desc f35_fd;
	struct device *dev;
	struct work_struct fwu_work;
	bool irq_enabled;
	struct semaphore irq_sema;
	struct wakeup_source *flash_wake_src;
	struct completion remove_complete;

	int (*firmware_update)(struct device *dev, const char *fwname);
	int (*firmware_erase)(struct device *dev);
};

struct image_header {
	unsigned int checksum;
	unsigned int image_size;
	unsigned int config_size;
	unsigned char options;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static inline ssize_t synaptics_rmi4_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	dev_warn(&rmi4_data->i2c_client->dev,
			"%s Attempted to read from write-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t synaptics_rmi4_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	dev_warn(&rmi4_data->i2c_client->dev,
			"%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

/* Touchscreen MMI class related section */
extern int drv_instance_counter;
extern struct list_head drv_instances_list;
extern struct mutex instances_mutex;

int synaptics_mmi_data_init(struct synaptics_rmi4_data *ts, bool enable);
int synaptics_dsx_sensor_ready_state(
	struct synaptics_rmi4_data *rmi4_data, bool standby);
int synaptics_dsx_wait_for_idle(struct synaptics_rmi4_data *rmi4_data);
int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data, bool on);
void synaptics_dsx_sensor_state(struct synaptics_rmi4_data *rmi4_data,
		int state);
int synaptics_dsx_ic_reset(struct synaptics_rmi4_data *rmi4_data, int reset);
int synaptics_dsx_charger_mode(struct synaptics_rmi4_data *rmi4_data, int state);

/* suspend resume */
int synaptics_rmi4_suspend(struct device *dev);
int synaptics_rmi4_resume(struct device *dev);

/* end of class */

struct synaptics_rmi4_exp_fn_ptr {
	int (*read)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
			unsigned char *data, unsigned short length);
	int (*write)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
			unsigned char *data, unsigned short length);
	int (*enable)(struct synaptics_rmi4_data *rmi4_data, bool enable);
};

void synaptics_rmi4_new_function(struct synaptics_rmi4_data *rmi4_data,
		enum exp_fn fn_type, bool insert,
		int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
				unsigned char intr_mask),
		int (*func_status)(struct synaptics_rmi4_data *rmi4_data),
		enum ic_modes mode);

int synaptics_rmi4_scan_packet_reg_info(
	struct synaptics_rmi4_data *rmi4_data,
	unsigned short query_addr,
	unsigned short regs_base_addr,
	struct synaptics_rmi4_func_packet_regs *regs);

int synaptics_rmi4_read_packet_reg(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_func_packet_regs *regs, int idx);

int synaptics_rmi4_read_packet_regs(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_func_packet_regs *regs);

int alloc_buffer(struct temp_buffer *tb, size_t count);

static inline int secure_memcpy(unsigned char *dest, unsigned int dest_size,
		const unsigned char *src, unsigned int src_size,
		unsigned int count)
{
	if (dest == NULL || src == NULL)
		return -EINVAL;

	if (count > dest_size || count > src_size)
		return -EINVAL;

	memcpy((void *)dest, (const void *)src, count);

	return 0;
}

static inline int synaptics_rmi4_reg_read(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr,
		unsigned char *data,
		unsigned short len)
{
	return rmi4_data->i2c_read(rmi4_data, addr, data, len);
}

static inline int synaptics_rmi4_reg_write(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr,
		unsigned char *data,
		unsigned short len)
{
	return rmi4_data->i2c_write(rmi4_data, addr, data, len);
}

extern int FPS_register_notifier(struct notifier_block *nb,
				unsigned long stype, bool report);
extern int FPS_unregister_notifier(struct notifier_block *nb,
				unsigned long stype);

extern struct synaptics_rmi4_data *synaptics_driver_getdata(
		struct synaptics_rmi4_data *prev);

#endif
