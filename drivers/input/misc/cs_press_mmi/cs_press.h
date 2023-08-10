/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from  CHIPSEA.
 *
 *            (C) COPYRIGHT 2020 SHENZHEN CHIPSEA TECHNOLOGIES CO.,LTD.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from CHIPSEA.
 *
 *      Release Information : CSA37F71 chip forcetouch fw driver head file
 *      version : v0.2
 *-----------------------------------------------------------------------------
 */

#ifndef _CS_PRESS_H_
#define _CS_PRESS_H_

#define CS_I2C_ADDR        0x50 /* (0xA0>>1) */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/unistd.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/pinctrl/consumer.h>
#include <linux/kfifo.h>
#include <linux/version.h>

#include <linux/types.h>

#ifdef CONFIG_MTK_I2C_EXTENSION
#include <linux/dma-mapping.h>
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#endif

#define FW_FILE_NAME            "cs_press.nfw"
#define CS_MAX_STR_LABEL_LEN 64

/*
*@ LOG format
*/
#define LOG_ERR(fmt, args...)       pr_err("[cs_press][ERR] [%s: %d] "fmt,  __func__, __LINE__, ##args)
#define LOG_INFO(fmt, args...)      pr_err("[cs_press][INFO] [%s: %d] "fmt,  __func__, __LINE__, ##args)
#define LOG_DEBUG(fmt, args...)     pr_err("[cs_press][DEBUG] [%s: %d] "fmt,  __func__, __LINE__, ##args)

#define RSTPIN_RESET_ENABLE             1
#define CS_PRESS_CNT                    1

#define RETRY_NUM                       3
#define DEBUG_MODE_DELAY_TIME           20
#define CS_MANUFACTURER_ID_LENGTH       2
#define CS_MODULE_ID_LENGTH             2
#define CS_FW_VERSION_LENGTH            6
#define CS_CALI_PARA_LENGTH             8
#define RST_GPIO_HIGH                   1
#define RST_GPIO_LOW                    0
#define POWER_GPIO_HIGH                 1
#define POWER_GPIO_LOW                  0
#define IIC_KEY_EVENT                   0xD3
#define IIC_KEY_EVENT_MMI               0xD4
#define CS_FORCE_TRIG_LENGTH            6
#define CS_SCANT_PERIOD_LENGTH          4
#define CS_WORK_MODE_LENGTH             3
#define CS_SHELL_TEMP_LENGTH            4

#define CS_PRESS_DEV_ADDR_M             0xE4
#define CS_PRESS_DEV_ADDR_F             0xA0
#define CS_PRESS_DEV_ADDR_HID           0x2A

#define CH_NUM                         (4)

#define BOOT_FW_WRITE_CMD {0xaa,0x55,0xa5,0x5a}
#define BOOT_FW_JUMP_CMD  {0xA6,0x00,0x00,0x5A}
#define BOOT_FW_WFLAG_CMD {0x50,0x41,0x53,0x73}
/*reg addr*/
/*ndt reg*/
#define AP_RW_TEST_REG                  0x00 /*R/W read write test len = 10*/
#define AP_VERSION_REG                  0x10 /*R firmware infomation len = 6*/
#define AP_RD_CAIL_REG                  0x24 /*R len = 8*/
#define AP_RD_SONSOR_SATUS_REG          0x29 /*R len = 2*/
#define AP_RD_GPIO_STATUS_REG           0x2E /*R gpio status len = 6*/

#define AP_RD_TP_INFO_REG               0x52

#define AP_RD_APPLICATION_SATUS_REG     0x55 /*R application status len = 1*/
#define AP_RW_BUTTON_FORCE_RELEASE_THR_REG      0x5B /*R/W button force triggering/releasing threshold value len = 8*/
#define AP_RW_BUTTON_FORCE_THR_REG      0x5D /*R/W button force triggeringthreshold value len = 6*/
#define AP_RW_SCANT_PERIOD_REG          0x5E /*R/W scant period value len = 4*/
#define AP_RW_WORK_MODE_REG             0x5F /*R/W work mode len = 4*/
#define AP_RW_SHEll_TEMP_REG            0x61 /*R/W shell temp len = 4*/

#define AP_W_GPIO_TEST_REG              0x30 /*W io test len = 2*/

#define DEBUG_CLEAR_MODE                0x00
#define DEBUG_RAW_MODE                  0x10 /*rawdata*/
#define DEBUG_FORCE_DATA_MODE           0x20 /*forcedata*/
#define DEBUG_WRITE_COEF                0x30
#define DEBUG_READ_COEF                 0x31
#define DEBUG_WRITE_THRESHOLD           0x32
#define DEBUG_READ_THRESHOLD            0x33

#define DEBUG_MODE_REG                  0xF6
#define DEBUG_READY_REG                 0xF7
#define DEBUG_DATA_REG                  0xF8

#define AP_RESET_MCU_REG                0x01
#define AP_DEVICE_ID_REG                0x02
#define AP_MANUFACTURER_ID_REG          0x03
#define AP_MODULE_ID_REG                0x04

#define AP_WAKEUP_REG                   0x06
#define AP_SLEEP_REG                    0x07
#define AP_CALIBRATION_REG              0x1c
#define AP_WATCH_MODE_REG               0x1d

#define AP_FORCEDATA_REG                0x20

#define BOOT_CMD_REG                    0x0000
#define BOOT_RESET_REG                  0xF17C
#define BOOT_RESET_REG_HID              0x2010

/*updata info*/
#define FW_ADDR_CODE_LENGTH             0x0e
#define FW_ADDR_VERSION                 0x0A
#define FW_ADDR_CODE_START              0x100
#define FW_ONE_BLOCK_LENGTH_W           128
#define FW_ONE_BLOCK_LENGTH_R           256
#define BOOT_CMD_LENGTH                 4
#define FW_UPDATA_MAX_LEN              (64*1024)

/*mode config*/
#define AP_W_CAL_FACTOR_DEBUG_MODE      0x30
#define AP_R_CAL_FACTOR_DEBUG_MODE      0x31

#define AP_W_PRESS_LEVEL_DEBUG_MODE     0x32
#define AP_R_PRESS_LEVEL_DEBUG_MODE     0x33

#define AP_CALIBRATION_DEBUG_MODE       0x34
#define AP_R_SENSOR_STATUS_DEBUG_MODE   0x36
#define AP_R_OFFSET_DEBUG_MODE          0x37

#define AP_R_RAWDATA_DEBUG_MODE         0x01

#define AP_R_NOISE_DEBUG_MODE           0x11
#define AP_R_PROCESSED_DEBUG_MODE       0x14
#define AP_R_NDT_DEBUG_MODE             0xF8

/*linux driver info*/
#define CS_PRESS_NAME                   "cs_press"
#define MISC_DEVICE_NAME                "ndt"
#define CS_SYSFS_NAME                   "cs_press"

#define SHELL_TEMP_DELAY_TIME           (2000)
#define CHECK_DELAY_TIME                (20000)
#define I2C_CHECK_SCHEDULE
#define INT_SET_EN
//#define SIDE_KEY
//#define SIDE_KEY_MMI
#define CALIBRATION_SUCCESS_FALG        0xf0
#define CALIBRATION_FAIL_FALG           0xf2
#define CALIBRATION_OVERTIME_FALG       0xff

#define AFE_MAX_CH                      8
#define AFE_USE_CH                      1
#define KEY_NUM                         1

#define HIGH_VER_FILE_UPDATE            0
#define FORCE_FILE_UPDATE               1

#define PRESS_GEAR_LEVEL_LOW            1
#define PRESS_GEAR_LEVEL_MID            2
#define PRESS_GEAR_LEVEL_HIGH           3

#define  SCENE_NUM                      10
#define  PRESS_NUM                      10

#define  NAME_MAX_LENS                  256
#define MIN(A, B) ((A) < (B) ? (A) : (B))

#define DEFAULT_RUN_DELAY_TIME          50


#define IRQ_TYPE_EDGE_FALLING 0
#define IRQ_TYPE_EDGE_RISING 1

#define CS_DATA_MAX_QUEUE 200

#define DATA_MODE_REG 0xF1
#define DATA_READY_REG 0xF2
#define DATA_READ_REG 0xF3

#define CS_MAX_TOUCH 10

#define CS_INPUT_PHYS           "cs_press/input0"

typedef struct{
    unsigned short manufacturer_id;
    unsigned short module_id;
    unsigned short fw_version;
}CS_FW_INFO_Def;

typedef struct{
    short rawdata[AFE_MAX_CH];
}CS_RAWDATA_Def;

typedef struct{
    short arith_rawdata[AFE_USE_CH];
    short baseline[AFE_USE_CH];
    short diffdata[AFE_USE_CH];
    short energy_data[AFE_USE_CH];
}CS_PROCESSED_DATA_Def;

typedef struct{
    short noise_peak[AFE_USE_CH];
    unsigned int noise_std[AFE_USE_CH];
}CS_NOISE_DATA_Def;

typedef struct{
    short offset[AFE_USE_CH];
}CS_OFFSET_DATA_Def;

typedef struct{
    short status[AFE_USE_CH];
}CS_SENSOR_STATUS_Def;

typedef struct{
    unsigned char calibration_channel;
    unsigned char calibration_progress;
    unsigned short calibration_factor;
    short press_adc_1st;
    short press_adc_2nd;
    short press_adc_3rd;
}CS_CALIBRATION_RESULT_Def;

struct cs_press_coords {
    int coord;
    int pressure;
};

typedef enum {
    CS_NAV_MODE_NONE = 0,
    CS_NAV_MODE_VOLUME,
    CS_NAV_MODE_ZOOM,
    CS_NAV_MODE_SCROLL,
    CS_NAV_MODE_ROLL,
    CS_NAV_MODE_MAX,
} cs_nav_mode_t;

enum {
    CS_NAV_MODE_SCROLL_UP = 0,
    CS_NAV_MODE_SCROLL_DOWN,
    CS_NAV_MODE_ZOOM_IN,
    CS_NAV_MODE_ZOOM_OUT,
};

struct cs_vtp_scroll {
    int x0;
    int y0;
    int range;
    int step;
    int interval;
};

struct cs_vtp_zoom{
    int x0;
    int p1_y0;
    int p2_y0;
    int range;
    int step;
    int interval;
};

struct cs_press_t {
    int major;
    struct class *class;
    struct device *device_irq;
    struct i2c_client *client;
    struct kobject *kobj;
    struct device *class_dev;
    dev_t class_dev_no;

    /*GPIO pin*/
    int rst_gpio;
    int power_gpio;

    struct proc_dir_entry *p_proc_dir;

    unsigned short read_reg_len;
    unsigned char read_reg_data[64];

    unsigned char update_type;
    unsigned char updating_flag;


#ifdef I2C_CHECK_SCHEDULE
    struct delayed_work i2c_check_worker;
#endif
    struct delayed_work update_worker;

    struct delayed_work delay_run_worker;

    struct kfifo data_queue;
    struct delayed_work work;
    unsigned int panel_max_x;
    unsigned int panel_max_y;
    unsigned int panel_max_w; /*major and minor*/
    unsigned int nav_mode;
    char cs_fw_name[CS_MAX_STR_LABEL_LEN];
};

#endif
