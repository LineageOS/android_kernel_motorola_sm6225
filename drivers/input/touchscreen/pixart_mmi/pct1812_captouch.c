//
// Description: I2C Adapter for PixArt Imaging PCT1812FF Capacitance Touch Controller Driver on Linux Kernel
//
// Copyright (c) 2021 Lenovo, Inc. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#define MYNAME "pct1812_mmi"
#define pr_fmt(fmt) "pct1812_mmi: %s: " fmt, __func__

#define CALM_WDOG
#undef STATIC_PLATFORM_DATA
#undef DRY_RUN_UPDATE

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include <linux/kfifo.h>

// Comment the following section out if it's too much :-)
#if defined(SHOW_EVERYTHING)
#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif
#ifdef dev_dbg
#undef dev_dbg
#define dev_dbg dev_err
#endif
#endif

#define MAX_POINTS 5

#define AREAD (1)
#define AWRITE (0)

#define INVALID_BYTE2 0xebeb
#define INVALID_BYTE 0xeb

#define PCT1812_SECTOR_SIZE (0x1000) // 4K
#define CHUNK_SZ 256

#define SUPPORTED_FW_FILE_SIZE 0x0000F000
#define FW_SECTION_SIZE 0x0000D000
#define CUSTOM_SECTION_SIZE (0x00000000 + PCT1812_SECTOR_SIZE)
#define PARAM_SECTION_SIZE CUSTOM_SECTION_SIZE

#define ADDRESS_FW 0x00000000
#define ADDRESS_CUSTOM FW_SECTION_SIZE
#define ADDRESS_PARAM (SUPPORTED_FW_FILE_SIZE - PARAM_SECTION_SIZE)

#define POWERUP_CODE 0x02
#define RESET_CODE 0xaa
#define RESUME_CODE 0xbb
#define ENGINEERING_CODE 0xcc
#define SUSPEND_CODE 0x99

#define FRAME_NUM_REG 0x00

#define NUM_OBJ_REG 0x01
#define OBJ_X0L_REG 0x04
#define OBJ_X0H_REG 0x05
#define OBJ_Y0L_REG 0x06
#define OBJ_Y0H_REG 0x07
#define OBJ_0_BASE OBJ_X0L_REG

#define OBJ_ADDR(o) (OBJ_0_BASE + (o) * 4)

#define GEST_TYPE_REG 0x60
#define GEST_X0L_REG 0x62
#define GEST_X0H_REG 0x63
#define GEST_Y0L_REG 0x64
#define GEST_Y0H_REG 0x65
#define GEST_0_BASE GEST_X0L_REG

#define GEST_ADDR(g) (GEST_0_BASE + (g) * 4)

#define BOOT_STATUS_REG 0x70
#define STATUS_REG 0x71

#define USER_BANK_REG 0x73
#define USER_ADDR_REG 0x74
#define USER_DATA_REG 0x75

#define SW_RESET_REG 0x7a
#define DEEP_SLEEP_REG 0x7c
#define ACTIVE_BANK_REG 0x7f

#define BANK(a) (a)

// USER BANK 0
#define PROD_ID_REG 0x00
#define FW_REV_REG 0x01
#define FW_VMIN_L_REG 0x02
#define FW_MAJOR_REG 0x03
#define MODEL_REG 0x04
#define FW_PATCH_REG 0x06
#define INTR_MASK_REG 0x07
#define START_REG 0x09
#define FLASH_BANK_REG 0x0f
#define REPORT_RATE_L_REG 0x10
#define REPORT_RATE_H_REG 0x11
#define MAX_RATE_L_REG 0x12
#define MAX_RATE_H_REG 0x13
#define OP_MODE_REG 0x15
#define FORCE_OP_MODE_REG 0x16
#define REST1_L_REG 0x18
#define REST1_H_REG 0x19
#define REST2_L_REG 0x1a
#define REST2_H_REG 0x1b
#define REST1_FRAME_RATE_REG 0x1c
#define REST2_FRAME_RATE_REG 0x1d
#define TX_REG 0x5a
#define RX_REG 0x59
#define VER_LOW_REG 0x7e
#define VER_HIGH_REG 0x7f

// USER BANK 1
#define FLASH_PUP_REG 0x0d
#define KEY1_REG 0x2c
#define KEY2_REG 0x2d
#define ENG_MODE_REG 0x2e

// USER BANK 2
#define XRES_L_REG 0x00
#define XRES_H_REG 0x01
#define YRES_L_REG 0x02
#define YRES_H_REG 0x03
#define MAX_POINTS_REG 0x05
#define SRAM_SELECT_REG 0x09
#define SRAM_NSC_REG 0x0a
#define SRAM_PORT_REG 0x0b

// USER BANK 3
#define FUNCT_CTRL_REG 0x02
#define CUST_INFO_BASE 0x6a
#define CUST_INFO_RNUM 8

// USER BANK 4
#define FLASH_STATUS_REG 0x1c
#define FLASH_FLASH_CMD_REG 0x20
#define FLASH_DCOUNT0_REG 0x22
#define FLASH_DCOUNT1_REG 0x23
#define FLASH_ADDR0_REG 0x24
#define FLASH_ADDR1_REG 0x25
#define FLASH_ADDR2_REG 0x26
#define FLASH_COMMAND_REG 0x2c
#define FLASH_SRAM_OFF0_REG 0x2e
#define FLASH_SRAM_OFF1_REG 0x2f

#define STAT_BIT_ERR (1 << 0)
#define STAT_BIT_TOUCH (1 << 1)
#define STAT_BIT_GESTURE (1 << 3)
#define STAT_BIT_WDOG (1 << 7)
#define STAT_BIT_EABS (STAT_BIT_WDOG | STAT_BIT_TOUCH)
#define STAT_BIT_ALL 0xff

#define SLEEP_BIT_DISABLE (1 << 0)
#define SLEEP_BIT_STAT (1 << 4)

#define BOOT_COMPLETE 1
#define MODE_COMPLETE (BOOT_COMPLETE | (1 << 7))
#define TOUCH_RESET_DELAY 10
#define CMD_WAIT_DELAY 10
#define WDOG_INTERVAL 1000
#define SELFTEST_SHORT_INTERVAL 3000
#define SELFTEST_LONG_INTERVAL 5000
#define SELFTEST_EXTRA_LONG_INTERVAL 10000
#define PON_DELAY 200

#define CNT_I2C_RETRY 3
#define CNT_WAIT_RETRY 50

#define SNR_LOOP_COUNT 50

#define I2C_WRITE_BUFFER_SIZE 256
#define I2C_READ_BUFFER_SIZE 4

#define PINCTRL_STATE_ACTIVE "touchpad_active"
#define PINCTRL_STATE_SUSPEND "touchpad_suspend"

enum pwr_modes {
	PWR_MODE_AUTO, //controlled by FW
	PWR_MODE_RUN,
	PWR_MODE_LPM, // REST1
	PWR_MODE_DEEP_SLEEP, // REST2
	PWR_MODE_SUSPEND,
	PWR_MODE_NO_DEEP_SLEEP,
	PWR_MODE_MAX
};

enum cmds {
	CMD_INIT,
	CMD_RESET,
	CMD_SUSPEND,
	CMD_RESUME,
	CMD_RECOVER,
	CMD_WDOG,
	CMD_MODE_RESET,
	CMD_CLEANUP
};

enum selftest_ids {
	PCT1812_SELFTEST_NONE,
	PCT1812_SELFTEST_FULL = 1,
	PCT1812_SELFTEST_SNR,
	PCT1812_SELFTEST_DRAW_LINE,
	PCT1812_SELFTEST_IN_PROGRESS,
	PCT1812_SELFTEST_MAX
};

enum gestures {
	GEST_TAP = 2,
	GEST_DBL_TAP,
	GEST_VERT_SCROLL = 7,
	GEST_HORIZ_SCROLL,
	GEST_UP_SWIPE = 17,
	GEST_DWN_SWIPE,
	GEST_LFT_SWIPE,
	GEST_RGHT_SWIPE
};

struct pct1812_platform {
	int max_x;
	int max_y;

	unsigned rst_gpio;
	unsigned irq_gpio;

	struct i2c_client *client;

	const char *vdd;
	const char *vio;
};

#if defined(STATIC_PLATFORM_DATA)
#define VDD_PS_NAME "pm8350c_l3"
#define VIO_PS_NAME "pm8350_s10"

static struct pct1812_platform pct1812_pd = {
	0, 0,
	0, 84,
	NULL, VDD_PS_NAME, VIO_PS_NAME
};
#endif

#define DBG_BIT_PM 0x00000001
#define DBG_BIT_IRQ 0x00000002
#define DBG_BIT_ISR 0x00000004
#define DBG_BIT_I2C 0x00000008
#define DBG_BIT_WORK 0x00000010
#define DBG_BIT_TIME 0x00000020
#define DBG_BIT_TEST 0x00000040
#define DBG_BIT_FW 0x00000080
#define DBG_BIT_UACC 0x00000100
#define DBG_BIT_ALL 0x00000200

#define DBG_PRINT(s, fmt, ...) { \
	if (ts->debug & s) \
		pr_info(fmt, ##__VA_ARGS__); \
}

struct pct1812_data {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct pct1812_platform *plat_data;
	struct input_dev *idev;

	struct regulator *reg_vdd;
	struct regulator *reg_vio;

	int tx_count;
	int rx_count;

	int selftest;
	int frame_count;
	int frame_sample;
	int test_type;
	ktime_t test_start_time;
	unsigned int frame_size;
	unsigned int test_hdr_size;
	unsigned int test_data_size;
	unsigned char **test_data;
	unsigned char test_intr_mask;

	struct mutex i2c_mutex;
	struct mutex eventlock;
	struct mutex cmdlock;
	struct delayed_work worker;

	bool irq_enabled;
	struct kfifo cmd_pipe;
	atomic_t touch_stopped;

	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;

	unsigned char drop_mask;
	unsigned short version;
	unsigned char func_ctrl;
	unsigned char model;
	unsigned char major;
	unsigned short minor;
	unsigned char revision;
	unsigned char patch;
	unsigned char product_id;
	unsigned short x_res;
	unsigned short y_res;
	unsigned char max_points;
	unsigned char boot_block;
	unsigned char custom_block[CUST_INFO_RNUM];
	unsigned char debug;
};

static ssize_t selftest_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t doreflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t forcereflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t irq_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t info_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t mask_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t mask_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t selftest_bin_show(struct file *filp, struct kobject *kobp,
		struct bin_attribute *bin_attr, char *buf, loff_t pos, size_t count);

static DEVICE_ATTR_WO(doreflash);
static DEVICE_ATTR_WO(forcereflash);
static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_RO(ic_ver);
static DEVICE_ATTR_RO(info);
static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RO(irq_status);
static DEVICE_ATTR_RW(selftest);
static DEVICE_ATTR_RW(drv_irq);
static DEVICE_ATTR_RW(mask);
static DEVICE_ATTR_RW(debug);

static struct attribute *pct1812_attrs[] = {
	&dev_attr_doreflash.attr,
	&dev_attr_forcereflash.attr,
	&dev_attr_reset.attr,
	&dev_attr_ic_ver.attr,
	&dev_attr_vendor.attr,
	&dev_attr_drv_irq.attr,
	&dev_attr_irq_status.attr,
	&dev_attr_selftest.attr,
	&dev_attr_info.attr,
	&dev_attr_mask.attr,
	&dev_attr_debug.attr,
	NULL,
};

static struct attribute_group pct1812_attrs_group = {
	.attrs = pct1812_attrs,
};

static struct bin_attribute selftest_bin_attr = {
	.attr = {.name = "selftest_bin", .mode = 0440},
	.read = selftest_bin_show,
};

static int inline pct1812_selftest_get(struct pct1812_data *ts)
{
	int cur = PCT1812_SELFTEST_NONE;

	mutex_lock(&ts->cmdlock);
	cur = ts->selftest;
	mutex_unlock(&ts->cmdlock);

	return cur;
}

static void inline pct1812_selftest_set(struct pct1812_data *ts, enum selftest_ids mode)
{
	mutex_lock(&ts->cmdlock);
	if (ts->selftest != mode)
		ts->selftest = mode;
	mutex_unlock(&ts->cmdlock);
}

static void inline pct1812_drop_mask_set(struct pct1812_data *ts, unsigned char mask)
{
	mutex_lock(&ts->eventlock);
	ts->drop_mask = mask;
	mutex_unlock(&ts->eventlock);
}

static void inline pct1812_fifo_cmd_add(struct pct1812_data *ts, enum cmds command, unsigned int delay)
{
	kfifo_put(&ts->cmd_pipe, command);
	schedule_delayed_work(&ts->worker, msecs_to_jiffies(delay));
}

static void inline pct1812_delay_ms(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

static int pct1812_i2c_read(struct pct1812_data *ts, unsigned char reg, unsigned char *data, int len)
{
	unsigned char retry, buf[I2C_READ_BUFFER_SIZE];
	int ret;
	struct i2c_msg msg[2];
#if 0
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_err(&ts->client->dev, "%s: POWER_STATUS : OFF\n", __func__);
		return -EIO;
	}
#endif
	buf[0] = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < CNT_I2C_RETRY; retry++) {
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret == 2) {
			ret = 0; /* indicate a success */
			break;
		}
#if 0
		if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				dev_err(&ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				return -EIO;
		}
#endif
		pct1812_delay_ms(1);
		if (retry > 1) {
			dev_err(&ts->client->dev, "%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			//ts->comm_err_count++;
		}
	}
	mutex_unlock(&ts->i2c_mutex);

	if (retry == CNT_I2C_RETRY) {
		dev_err(&ts->client->dev, "%s: I2C read over retry limit\n", __func__);
		ret = -EIO;
	}

	if (ts->debug & DBG_BIT_I2C) {
		int i;
		pr_info("R: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}

	return ret;
}

static int pct1812_i2c_write(struct pct1812_data *ts, unsigned char reg, unsigned char *data, int len)
{
	unsigned char retry, buf[I2C_WRITE_BUFFER_SIZE + 1];
	int ret;
	struct i2c_msg msg;

	if (len > I2C_WRITE_BUFFER_SIZE) {
		dev_err(&ts->client->dev, "%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}
#if 0
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		dev_err(&ts->client->dev, "%s: POWER_STATUS : OFF\n", __func__);
		return -EIO;
	}
#endif
	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < CNT_I2C_RETRY; retry++) {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret == 1) {
			ret = 0; /* indicate a success */
			break;
		}
#if 0
		if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
			dev_err(&ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
			mutex_unlock(&ts->i2c_mutex);
			return -EIO;
		}
#endif
		pct1812_delay_ms(1);
		if (retry > 1) {
			dev_err(&ts->client->dev, "%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			//ts->comm_err_count++;
		}
	}
	mutex_unlock(&ts->i2c_mutex);

	if (retry == CNT_I2C_RETRY) {
		dev_err(&ts->client->dev, "%s: I2C write over retry limit\n", __func__);
		ret = -EIO;
	}

	if (ts->debug & DBG_BIT_I2C) {
		int i;
		pr_info("W: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}

	return ret;
}

static void inline pct1812_set_irq(struct pct1812_data *ts, bool on)
{
	if (on) {
		if (!ts->irq_enabled) {
			ts->irq_enabled = true;
			enable_irq(ts->plat_data->client->irq);
			DBG_PRINT(DBG_BIT_IRQ,"IRQ enabled\n");
		}
	} else {
		if (ts->irq_enabled) {
			ts->irq_enabled = false;
			disable_irq(ts->plat_data->client->irq);
			DBG_PRINT(DBG_BIT_IRQ,"IRQ disabled\n");
		}
	}
}

#ifndef STATIC_PLATFORM_DATA
static int pct1812_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pct1812_platform *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	unsigned int coords[2];
	int ret = 0;

	if (of_property_read_string(np, "pct1812,regulator_vdd", &pdata->vdd)) {
		dev_err(dev, "%s: Failed to get VDD name property\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_string(np, "pct1812,regulator_vio", &pdata->vio)) {
		dev_warn(dev, "%s: Failed to get VIO name property\n", __func__);
		//return -EINVAL;
	}

	pdata->irq_gpio = of_get_named_gpio(np, "pct1812,irq-gpio", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		dev_err(&client->dev, "%s: Failed to get irq gpio\n", __func__);
		return -EINVAL;
	}

	pdata->rst_gpio = of_get_named_gpio(np, "pct1812,reset-gpio", 0);
	if (!gpio_is_valid(pdata->rst_gpio))
		pdata->rst_gpio = -EINVAL;

	if (!of_property_read_u32_array(np, "pct1812,max_coords", coords, 2)) {
		pdata->max_x = coords[0] - 1;
		pdata->max_y = coords[1] - 1;
		dev_info(dev, "%s: max_coords:(%d,%d)\n", __func__, pdata->max_x, pdata->max_y);
	}

	return ret;
}
#endif

static int pct1812_platform_init(struct pct1812_platform *pdata)
{
	struct device *dev = &pdata->client->dev;
	int ret;

	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN, "pct1812,irq");
		if (ret) {
			dev_err(dev, "%s: Unable to request gpio [%d]\n", __func__, pdata->irq_gpio);
			return -EINVAL;
		}
		dev_info(dev, "%s: using irq gpio %d\n", __func__, pdata->irq_gpio);
	}

	if (gpio_is_valid(pdata->rst_gpio)) {
		ret = gpio_request_one(pdata->rst_gpio, GPIOF_DIR_OUT, "pct1812,reset");
		if (ret) {
			dev_err(dev, "%s: Unable to request gpio [%d]\n", __func__, pdata->rst_gpio);
			return -EINVAL;
		}
		dev_info(dev, "%s: using reset gpio %d\n", __func__, pdata->rst_gpio);
	}

	pdata->client->irq = gpio_to_irq(pdata->irq_gpio);

	return 0;
}

static int inline comp2_16b(unsigned char *data)
{
	int result;
	/* 16bits 2's compliment data */
	if (data[1] & 0x80)
		result = (data[0] | (data[1] << 8) | 0xFFFF0000);
	else
		result = (data[0] | (data[1] << 8));

	return result;
}

#define X0L 0
#define X0H 1
#define Y0L 2
#define Y0H 3

#define X_GET(d) (&d[X0L])
#define Y_GET(d) (&d[Y0L])

#define UINT16_X(d) ((d[1] << 8) | d[0] | 0x00000000)
#define UINT16_Y(d) ((d[3] << 8) | d[2] | 0x00000000)

static int pct1812_read_xy_coords(struct pct1812_data *ts,
			unsigned char start_addr, unsigned char *tdata)
{
	int ret;

	ret = pct1812_i2c_read(ts, start_addr, &tdata[X0L], sizeof(tdata[X0L]));
	ret = !ret && pct1812_i2c_read(ts, start_addr + 1, &tdata[X0H], sizeof(tdata[X0H]));
	ret = !ret && pct1812_i2c_read(ts, start_addr + 2, &tdata[Y0L], sizeof(tdata[Y0L]));
	ret = !ret && pct1812_i2c_read(ts, start_addr + 3, &tdata[Y0H], sizeof(tdata[Y0H]));

	return ret ? -EIO : 0;
}

static int inline pct1812_selftest_push(struct pct1812_data *ts,
		unsigned char *coords, unsigned int length)
{
	int ret = -ERANGE;

	if ((ts->frame_sample + 1) <= ts->frame_count) {
		memcpy(ts->test_data[ts->frame_sample], coords, length);
		ts->frame_sample++;
		ts->test_data_size += length;
		DBG_PRINT(DBG_BIT_TEST,"push test sample to slot %d; data size %u\n", ts->frame_sample, ts->test_data_size);
		ret = 0;
	} else
		dev_warn(ts->dev, "%s: Buffer is full\n", __func__);

	return ret;
}

static int pct1812_process_touch_event(struct pct1812_data *ts, int up_event)
{
	unsigned char nt, coords[4] = {0};
	unsigned int x, y;
	int i, mode, ret;
	int pressed;
	static int finger[MAX_POINTS];

	ret = pct1812_i2c_read(ts, NUM_OBJ_REG, &nt, sizeof(nt));
	if (ret)
		return -EIO;
	DBG_PRINT(DBG_BIT_ISR,"num objects %d\n", (int)nt);
	for (i = 0; i < MAX_POINTS; i++) {
		if (i < nt) {
			ret = pct1812_read_xy_coords(ts, OBJ_ADDR(i), coords);
			if (ret) {
				dev_err(ts->dev, "%s: error reading obj %d\n", __func__, i);
				return -EIO;
			}
			x = UINT16_X(coords);
			y = UINT16_Y(coords);
			if (i == 0) { // draw line self-test supports one finger only
				mode = pct1812_selftest_get(ts);
				if (mode == PCT1812_SELFTEST_DRAW_LINE)
					pct1812_selftest_push(ts, coords, sizeof(coords));
				DBG_PRINT(DBG_BIT_TEST,"[@%02x]: x=%d, y=%d\n", OBJ_ADDR(i), x, y);
			}
			pressed = 1;
			if (finger[i] != 1) {
				input_mt_slot(ts->idev, i);
				input_mt_report_slot_state(ts->idev, MT_TOOL_FINGER, 1);
				input_report_key(ts->idev, BTN_TOUCH, 1);
				input_report_key(ts->idev, BTN_TOOL_FINGER, 1);
				DBG_PRINT(DBG_BIT_ISR,"[@%02x]: x=%d, y=%d\n", OBJ_ADDR(i), x, y);
			}
			input_report_abs(ts->idev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->idev, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->idev, ABS_X, x);
			input_report_abs(ts->idev, ABS_Y, y);
			input_sync(ts->idev);
		} else
			pressed = 0;

		if (finger[i] && !pressed) {
			input_mt_slot(ts->idev, i);
			input_mt_report_slot_state(ts->idev, MT_TOOL_FINGER, 0);
			/* report BTN_TOUCH up only if it was not reported as part of gesture */
			if (!up_event)
				input_report_key(ts->idev, BTN_TOUCH, 0);
			input_report_key(ts->idev, BTN_TOOL_FINGER, 0);
			input_sync(ts->idev);
		}
		finger[i] = pressed;
	}

	return 0;
}

static int pct1812_process_gesture_event(struct pct1812_data *ts, int *up_event)
{
	unsigned char gest, coords[4] = {0};
	bool send_button = false;
	int ret;

	ret = pct1812_i2c_read(ts, GEST_TYPE_REG, &gest, sizeof(gest));
	if (ret)
		return -EIO;

	DBG_PRINT(DBG_BIT_ISR,"gesture 0x%02x\n", gest);
	switch(gest & 0x1f) {
		case GEST_HORIZ_SCROLL:
		case GEST_VERT_SCROLL:
		case GEST_TAP:
						break;
		case GEST_DBL_TAP:
				send_button = true;
						break;
		case GEST_UP_SWIPE:
		case GEST_DWN_SWIPE:
		case GEST_LFT_SWIPE:
		case GEST_RGHT_SWIPE:
						return 0;
		default: return -EINVAL;
	}

	if (send_button) {
		/* press */
		input_report_key(ts->idev, BTN_TOOL_DOUBLETAP, 1);
		input_report_key(ts->idev, BTN_TOUCH, 1);
		input_sync(ts->idev);
		/* release */
		input_report_key(ts->idev, BTN_TOOL_DOUBLETAP, 0);
		input_report_key(ts->idev, BTN_TOUCH, 0);
		input_sync(ts->idev);
		/* lift up event reported */
		*up_event = 1;
	} else {
		ret = pct1812_read_xy_coords(ts, GEST_ADDR(0), coords);
		if (ret) {
			dev_err(ts->dev, "%s: error reading gesture\n", __func__);
		} else {
			int x = comp2_16b(X_GET(coords));
			int y = comp2_16b(Y_GET(coords));
#if 0
			input_report_rel(ts->idev, REL_X, x);
			input_report_rel(ts->idev, REL_Y, y);
#endif
			DBG_PRINT(DBG_BIT_ISR,"[@%02x]: x=%d, y=%d\n", GEST_ADDR(0), x, y);
		}
	}

	return ret;
}

static irqreturn_t pct1812_irq_handler(int irq, void *ptr)
{
	struct pct1812_data *ts = (struct pct1812_data *)ptr;
	unsigned char dropped, status = 0;
	bool ack = true;
	int ret, up_event = 0;

	mutex_lock(&ts->eventlock);
	ret = pct1812_i2c_read(ts, STATUS_REG, &status, sizeof(status));
	if (ret || ((status & STAT_BIT_ERR) || (status & STAT_BIT_WDOG))) {
		pct1812_fifo_cmd_add(ts, CMD_RECOVER, TOUCH_RESET_DELAY);
		goto failure;
	}
	dropped = status;
	dropped &= ~ts->drop_mask;
	if (status != dropped)
		DBG_PRINT(DBG_BIT_ISR,"drop out %02x -> %02x\n", status, dropped);
	if (dropped & STAT_BIT_GESTURE) {
		pct1812_process_gesture_event(ts, &up_event);
	}
	if (dropped & STAT_BIT_TOUCH) {
		pct1812_process_touch_event(ts, up_event);
	}
	if (ack) {
		status = 0; // ACK interrupt
		ret = pct1812_i2c_write(ts, STATUS_REG, &status, sizeof(status));
	}
failure:
	mutex_unlock(&ts->eventlock);

	return IRQ_HANDLED;
}

static int pct1812_user_access(struct pct1812_data *ts,
	unsigned char bank, unsigned char addr,
	unsigned char *value, unsigned int size, bool R)
{
	unsigned char vArray[16];
	int ret;

	if (R && size > sizeof(vArray)) {
		dev_err(ts->dev, "%s: Requested size %d bigger than buffer size %u\n", __func__, size, sizeof(vArray));
		return -EINVAL;
	}

	memset(vArray, INVALID_BYTE, sizeof(vArray));
	ret = pct1812_i2c_write(ts, USER_BANK_REG, &bank, sizeof(bank));
	if (ret)
		return -EIO;

	ret = pct1812_i2c_write(ts, USER_ADDR_REG, &addr, sizeof(addr));
	if (ret)
		return -EIO;

	if (R) { // read access
		if (value)
			memset(value, 0, size);
		ret = pct1812_i2c_read(ts, USER_DATA_REG, vArray, size);
		if (!ret && value)
			memcpy(value, vArray, size);
	} else {
		ret = pct1812_i2c_write(ts, USER_DATA_REG, value, size);
	}

	if (ts->debug & DBG_BIT_UACC) {
		int i;
		pr_info("%c: b%d:0x%02x |", R ? 'R' : 'W', bank, addr);
		for (i = 0; i < size; i++)
			pr_cont(" %02X", value ? value[i] : vArray[i]);
		pr_cont("\n");
	}

	return ret;
}

#define QUERY_PARAM_IF(p, b, r, v) { \
	p = INVALID_BYTE; \
	ret = pct1812_user_access(ts, b, r, &(v), sizeof(v), AREAD); \
	if (!ret) { \
		p = v; \
	} else { \
		dev_err(ts->dev, "%s: Error reading b(%d):0x%02x\n", __func__, b, r); \
		goto failure; \
	} \
}

#define QUERY_PARAM(p, b, r, v) { \
	p = INVALID_BYTE; \
	ret = pct1812_user_access(ts, b, r, &(v), sizeof(v), AREAD); \
	if (!ret) { \
		p = v; \
	} else { \
		dev_err(ts->dev, "%s: Error reading b(%d):0x%02x\n", __func__, b, r); \
	} \
}

#define SET_PARAM_IF(b, r, v) { \
	ret = pct1812_user_access(ts, b, r, &(v), sizeof(v), AWRITE); \
	if (ret) { \
		dev_err(ts->dev, "%s: Error writing b(%d):0x%02x\n", __func__, b, r); \
		goto failure; \
	} \
}

#define GET_REG_IF(r, v) { \
	v = INVALID_BYTE; \
	ret = pct1812_i2c_read(ts, r, &(v), sizeof(v)); \
	if (ret) { \
		dev_err(ts->dev, "%s: Error reading Reg0x%02x\n", __func__, r); \
		goto failure; \
	} \
}

#define SET_REG_IF(r, v) { \
	ret = pct1812_i2c_write(ts, r, &(v), sizeof(v)); \
	if (ret) { \
		dev_err(ts->dev, "%s: Error writing Reg0x%02x val=0x%02x\n", __func__, r, v); \
		goto failure; \
	} \
}

#define SET_REG_CHK_IF(r, v)  if (1) { \
	unsigned char checkout; \
	ret = pct1812_i2c_write(ts, r, &(v), sizeof(v)); \
	if (ret) { \
		dev_err(ts->dev, "%s: Error writing Reg0x%02x val=0x%02x\n", __func__, r, v); \
		goto failure; \
	} else { \
		ret = pct1812_i2c_read(ts, r, &checkout, sizeof(checkout)); \
		if (ret) { \
			dev_err(ts->dev, "%s: Error reading back Reg0x%02x\n", __func__, r); \
			goto failure; \
		} else if (v != checkout) { \
			dev_warn(ts->dev, "%s: Mismatch %02x != %02x\n", __func__, v, checkout); \
		} else { \
			DBG_PRINT(DBG_BIT_FW,"active BANK(%d)\n", checkout); \
		} \
	} \
}

static int inline pct1812_set_bank(struct pct1812_data *ts,
			unsigned char bank) {
	int ret;
	SET_REG_CHK_IF(ACTIVE_BANK_REG, bank);
failure:
	return ret;
}

static int inline pct1812_sensing_on(struct pct1812_data *ts) {
	unsigned char curmode, value;
	int ret;
	QUERY_PARAM_IF(curmode, BANK(0), START_REG, value);
	curmode |= 0x01;
	SET_PARAM_IF(BANK(0), START_REG, curmode);
	DBG_PRINT(DBG_BIT_ALL,"start reg b0:0x02x value %02x\n", START_REG, curmode);
failure:
	return ret;
}

static int pct1812_get_extinfo(struct pct1812_data *ts)
{
	unsigned char val, v1, v2;
	int i, ret;

	QUERY_PARAM(v1, BANK(0), VER_LOW_REG, val);
	QUERY_PARAM(v2, BANK(0), VER_HIGH_REG, val);
	ts->version = v1 | (v2 << 8);
	QUERY_PARAM(ts->tx_count, BANK(0), TX_REG, val);
	QUERY_PARAM(ts->rx_count, BANK(0), RX_REG, val);
	QUERY_PARAM(ts->model, BANK(0), MODEL_REG, val);
	QUERY_PARAM(ts->product_id, BANK(0), PROD_ID_REG, val);
	QUERY_PARAM(v1, BANK(0), FW_MAJOR_REG, val);
	ts->major = v1 >> 4;
	QUERY_PARAM(v2, BANK(0), FW_VMIN_L_REG, val);
	// 4bits in high byte of MINOR version come from MAJOR register
	ts->minor = v2 | ((v1 & 0x0f) << 8);
	QUERY_PARAM(ts->revision, BANK(0), FW_REV_REG, val);
	QUERY_PARAM(ts->patch, BANK(0), FW_PATCH_REG, val);
	QUERY_PARAM(ts->boot_block, BANK(0), FLASH_BANK_REG, val);
	QUERY_PARAM(v1, BANK(2), XRES_L_REG, val);
	QUERY_PARAM(v2, BANK(2), XRES_H_REG, val);
	ts->x_res = v1 | (v2 << 8);
	QUERY_PARAM(v1, BANK(2), YRES_L_REG, val);
	QUERY_PARAM(v2, BANK(2), YRES_H_REG, val);
	ts->y_res = v1 | (v2 << 8);
	QUERY_PARAM(v1, BANK(2), MAX_POINTS_REG, val);
	ts->max_points = v1 & 0x0f;
	for (i = 0; i < CUST_INFO_RNUM; i++) {
		QUERY_PARAM(ts->custom_block[i], BANK(3), CUST_INFO_BASE + i, val);
	}

	return ret;
}

static int pct1812_power_mode(struct pct1812_data *ts, enum pwr_modes mode)
{
	unsigned char val, cur_mode;
	int ret = 0;

	switch (mode) {
	case PWR_MODE_NO_DEEP_SLEEP:
			val = INVALID_BYTE;
			pct1812_i2c_read(ts, DEEP_SLEEP_REG, &val, sizeof(val));
			DBG_PRINT(DBG_BIT_PM,"%s: SLEEP_STATUS before %02x\n", __func__, val);
			val = 0x01; // NO_DEEP_SLEEP
			ret = pct1812_i2c_write(ts, DEEP_SLEEP_REG, &val, sizeof(val));
			if (ret) {
				dev_err(ts->dev, "%s: error setting NO_DEEP_SLEEP\n", __func__);
				goto failure;
			}
do_again:
			pct1812_delay_ms(10);
			ret = pct1812_i2c_read(ts, DEEP_SLEEP_REG, &val, sizeof(val));
			if (ret)
				goto failure;
			if ((val & SLEEP_BIT_STAT))
				goto do_again;
			DBG_PRINT(DBG_BIT_PM,"%s: SLEEP_STATUS %02x\n", __func__, val);
					break;
	case PWR_MODE_AUTO:
	case PWR_MODE_RUN:
	case PWR_MODE_LPM: // REST1
	case PWR_MODE_DEEP_SLEEP: // REST2
	case PWR_MODE_SUSPEND:
			QUERY_PARAM(cur_mode, BANK(3), OP_MODE_REG, val);
			if (cur_mode != mode) {
				val = mode;
				SET_PARAM_IF(BANK(3), FORCE_OP_MODE_REG, val);
				DBG_PRINT(DBG_BIT_PM,"%s: PM mode %02x\n", __func__, val);
			}
					break;
	default:
			dev_err(ts->dev, "%s: invalid power mode %d\n", __func__, mode);
			ret = -EINVAL;
	}
failure:
	return ret;
}
// vok - value compared bit wise
static int pct1812_wait4ready(struct pct1812_data *ts, unsigned char reg, unsigned char vok)
{
	unsigned char status = 0;
	int rlimit = CNT_WAIT_RETRY;
	int dwait = CMD_WAIT_DELAY;
	int retry, ret;

	// MODE_COMPLETE takes much longer
	if (vok == MODE_COMPLETE) {
		rlimit = 250;
		dwait = 20;
	}

	for (retry = 0; retry < rlimit; retry++) {
		ret = pct1812_i2c_read(ts, reg, &status, sizeof(status));
		if (ret || ((status & vok) == vok))
			break;
		pct1812_delay_ms(dwait);
	}
	if (retry == rlimit)
		ret = -ETIME;
	else if (!ret) {
		DBG_PRINT(DBG_BIT_TIME,"success waiting for %d\n", vok);
	}

	return (status & vok) ? 0 : ret;
}
/* This function assumes active page is set to BANK(1) already */
static int inline pct1812_engmode(struct pct1812_data *ts, bool on,
			unsigned char *vok)
{
	unsigned char curmode, value;
	int ret;

	value = RESET_CODE;
	ret = pct1812_i2c_write(ts, KEY1_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	value = on ? ENGINEERING_CODE : RESUME_CODE;
	ret = pct1812_i2c_write(ts, KEY2_REG, &value, sizeof(value));
	if (ret)
		goto failure;

	pct1812_delay_ms(5);

	/* verify effective mode */
	curmode = 0;
	ret = pct1812_i2c_read(ts, ENG_MODE_REG, &curmode, sizeof(curmode));
	if (ret)
		goto failure;
	pr_info("engineering mode %02x\n", curmode);

	return vok && *vok != curmode ? -EINVAL : 0;
failure:
	return -EIO;
}
/* This function assumes active page is set to BANK(4) already */
static int pct1812_flash_exec(struct pct1812_data *ts,
		unsigned char cmd, unsigned char flash_cmd, int cnt)
{
	unsigned char command, lval;
	int repetition = 0;
	int ret, step = 0;

	DBG_PRINT(DBG_BIT_FW,"cmd 0x%02x, flash_cmd 0x%02x, cnt %d\n", cmd, flash_cmd, cnt);
#ifndef DRY_RUN_UPDATE
	// clean status
	lval = 0;
	ret = pct1812_i2c_write(ts, FLASH_COMMAND_REG, &lval, sizeof(lval));
	if (ret)
		goto failure;
	step++;
	// write flash cmd
	lval = flash_cmd;
	ret = pct1812_i2c_write(ts, FLASH_FLASH_CMD_REG, &lval, sizeof(lval));
	if (ret)
		goto failure;
	step++;
	// write data count LSB
	lval = cnt & 0xff;
	ret = pct1812_i2c_write(ts, FLASH_DCOUNT0_REG, &lval, sizeof(lval));
	if (ret)
		goto failure;
	step++;
	// write data count MSB
	lval = (cnt >> 8) & 0xff;
	ret = pct1812_i2c_write(ts, FLASH_DCOUNT1_REG, &lval, sizeof(lval));
	if (ret)
		goto failure;
	step++;
	// write cmd
	lval = cmd;
	ret = pct1812_i2c_write(ts, FLASH_COMMAND_REG, &lval, sizeof(lval));
	if (ret)
		goto failure;
	step++;
	// wait for completion
do_again:
	command = 0;
	ret = pct1812_i2c_read(ts, FLASH_COMMAND_REG, &command, sizeof(command));
	if (ret)
		goto failure;

	if ((command & cmd) != 0) {
		repetition++;
		if (repetition%11)
			DBG_PRINT(DBG_BIT_FW,"Still waiting ... %d\n", repetition);
		goto do_again;
	}
#else
	command = 0, lval = 0, ret = 0, step = 0, repetition = 0;
#endif
	return 0;
#ifndef DRY_RUN_UPDATE
failure:
	dev_err(ts->dev,
		"Error exec cmd 0x%02x, flash_cmd 0x%02x, cnt %d (stage %d, reps %d)\n",
		__func__, cmd, flash_cmd, cnt, step, repetition);

	return -EIO;
#endif
}
/* This function assumes active page is set to BANK(4) already */
static int pct1812_flash_status(struct pct1812_data *ts, int bit_idx, int vok)
{
	unsigned char status;
	int ret;
#ifndef DRY_RUN_UPDATE
do_again:
#endif
	ret = pct1812_flash_exec(ts, 0x08, 0x05, 1);
	if (ret) {
		dev_err(ts->dev, "Error flash cmd\n", __func__);
		return -EIO;
	}
#ifndef DRY_RUN_UPDATE
	ret = pct1812_i2c_read(ts, FLASH_STATUS_REG, &status, sizeof(status));
	if (ret) {
		dev_err(ts->dev, "Error reading flash status\n", __func__);
		return -EIO;
	}

	if ((status & (1 << bit_idx)) != vok) {
		pct1812_delay_ms(2); // adjust delay if necessary
		goto do_again;
	}
#else
	ret = 0, status = 0;
#endif
	DBG_PRINT(DBG_BIT_FW,"Flash status (bit(%d)=%d) OK\n", bit_idx, vok);

	return 0;
}
/* This function assumes active page is set to BANK(4) already */
static int pct1812_flash_chunk(struct pct1812_data *ts, unsigned int address)
{
	int ret, step = 0;
	unsigned char value;

	// Flash WriteEnable
	ret = pct1812_flash_exec(ts, 0x02, 0x06, 0);
	if (ret)
		goto failure;
	step++;
	// check status
	ret = pct1812_flash_status(ts, 1, 2);
	if (ret)
		goto failure;
	step++;
#ifndef DRY_RUN_UPDATE
	value = (unsigned char)(address & 0xff);
	ret = pct1812_i2c_write(ts, FLASH_ADDR0_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	step++;

	value = (unsigned char)((address >> 8) & 0xff);
	ret = pct1812_i2c_write(ts, FLASH_ADDR1_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	step++;

	value = (unsigned char)((address >> 16) & 0xff);
	ret = pct1812_i2c_write(ts, FLASH_ADDR2_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	step++;

	// set SRAM access offset
	value = 0;
	ret = pct1812_i2c_write(ts, FLASH_SRAM_OFF0_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	ret = !ret && pct1812_i2c_write(ts, FLASH_SRAM_OFF1_REG, &value, sizeof(value));
	if (ret) {
		dev_err(ts->dev, "%s: Error setting SRAM offset\n", __func__);
		goto failure;
	}
	step++;
#else
	value = 0;
#endif
	ret = pct1812_flash_exec(ts, 0x81, 0x02, CHUNK_SZ);
	if (ret)
		goto failure;
	step++;
	// wait for completion
	ret = pct1812_flash_status(ts, 0, 0);
	if (ret)
		goto failure;

	return 0;

failure:
	dev_err(ts->dev, "Error erasing address 0x%06x (stage %d)\n",
			__func__, address, step);
	return -EIO;
}

static int pct1812_erase_sector(struct pct1812_data *ts, unsigned int address)
{
	int ret, step = 0;
	unsigned char value;

	DBG_PRINT(DBG_BIT_FW,"Erasing sector @0x%06x\n", address);

	// Flash WriteEnable
	ret = pct1812_flash_exec(ts, 0x02, 0x06, 0);
	if (ret)
		goto failure;
	step++;
	// check status
	ret = pct1812_flash_status(ts, 1, 2);
	if (ret)
		goto failure;
	step++;
#ifndef DRY_RUN_UPDATE
	value = (unsigned char)(address & 0xff);
	ret = pct1812_i2c_write(ts, FLASH_ADDR0_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	step++;

	value = (unsigned char)((address >> 8) & 0xff);
	ret = pct1812_i2c_write(ts, FLASH_ADDR1_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	step++;

	value = (unsigned char)((address >> 16) & 0xff);
	ret = pct1812_i2c_write(ts, FLASH_ADDR2_REG, &value, sizeof(value));
	if (ret)
		goto failure;
	step++;
#else
	value = 0;
#endif
	ret = pct1812_flash_exec(ts, 0x02, 0x20, 3);
	if (ret)
		goto failure;
	step++;
	// wait for completion
	ret = pct1812_flash_status(ts, 0, 0);
	if (ret)
		goto failure;

	return 0;

failure:
	dev_err(ts->dev, "Error erasing address 0x%06x (stage %d)\n",
			__func__, address, step);
	return -EIO;
}

static int pct1812_flash_section(struct pct1812_data *ts, unsigned char *data,
		unsigned int size, unsigned int address)
{
	int m = 0, ret;
	unsigned int num_of_sectors = size / PCT1812_SECTOR_SIZE;
	unsigned int target_num = size / CHUNK_SZ;
	unsigned char value;
	unsigned char *ptr = data;

	ret = pct1812_set_bank(ts, BANK(4));
	if (ret)
		goto failure;

	DBG_PRINT(DBG_BIT_FW,"Start addr = %06x, size = %04x\n", address, size);
	DBG_PRINT(DBG_BIT_FW,"4K sectors = %u, 256b chunks = %u\n", num_of_sectors, target_num);
	for (m = 0; m < num_of_sectors; m++) {
		ret = pct1812_erase_sector(ts, address + PCT1812_SECTOR_SIZE * m);
		if (ret)
			goto failure;
		DBG_PRINT(DBG_BIT_FW,"Erased sector %d of %d\n", m + 1, num_of_sectors);
	}

	for (m = 0; m < target_num; m++) {
		ret = pct1812_set_bank(ts, BANK(2));
		if (ret)
			goto failure;
#ifndef DRY_RUN_UPDATE
		// Set SRAM select
		value = 0x08;
		ret = pct1812_i2c_write(ts, SRAM_SELECT_REG, &value, sizeof(value));
		if (ret) {
			dev_err(ts->dev, "%s: error setting SRAM_SELECT\n", __func__);
			goto failure;
		}
		// Set SRAM NSC to 0
		value = 0x00;
		ret = pct1812_i2c_write(ts, SRAM_NSC_REG, &value, sizeof(value));
		if (ret) {
			dev_err(ts->dev, "%s: error setting SRAM_NSC\n", __func__);
			goto failure;
		}
		// Write data to SRAM port
		ret = pct1812_i2c_write(ts, SRAM_PORT_REG, ptr, CHUNK_SZ);
		if (ret) {
			dev_err(ts->dev, "%s: error writing to SRAM\n", __func__);
			goto failure;
		}
		// Set SRAM NSC to 1
		value = 0x01;
		ret = pct1812_i2c_write(ts, SRAM_NSC_REG, &value, sizeof(value));
		if (ret) {
			dev_err(ts->dev, "%s: error setting SRAM_NSC\n", __func__);
			goto failure;
		}
#else
		value = 0;
#endif
		// advance data pointer
		ptr += CHUNK_SZ;
		DBG_PRINT(DBG_BIT_FW,"Flashing chunk @0x%06x\n", address);

		ret = pct1812_set_bank(ts, BANK(4));
		if (ret)
			goto failure;

		// Program Flash from SRAM
		ret = pct1812_flash_chunk(ts, address);
		if (ret)
			goto failure;
		address += CHUNK_SZ;
	}

	return 0;
failure:
	dev_err(ts->dev, "Error flashing chunk %d\n", __func__, m);
	return -EIO;
}

/*static*/
int pct1812_fw_update(struct pct1812_data *ts, char *fname)
{
	int ret;
	unsigned char value;
	unsigned char *fwdata_ptr, *fwparam_ptr, *fwcust_ptr;
	unsigned int fwdata_size, fwparam_size, fwcust_size;
	const struct firmware *fw_entry = NULL;

	//__pm_stay_awake(&ts->wake_src);
	mutex_lock(&ts->cmdlock);
	dev_info(ts->dev, "%s: Start FW update from file %s\n", __func__, fname);
	//fwu_irq_enable(fwu, true);
	ret = request_firmware(&fw_entry, fname, ts->dev);
	if (ret) {
		dev_err(ts->dev, "%s: Error loading firmware %s\n", __func__, fname);
		ret = -EINVAL;
		goto failure;
	}
	// FW file size check
	if (fw_entry->size != SUPPORTED_FW_FILE_SIZE) {
		dev_err(ts->dev, "%s: Firmware %s file size is WRONG!!!\n", __func__, fname);
		ret = -EINVAL;
		goto failure;
	}

	/* 0000 ----
	 * FW BLK (size 0xd000)
	 * d000 ----
	 * CUSTOM BLK (size 0x1000)
	 * e000 ----
	 * PARAM BLK (size 0x1000)
	 * efff ----
	 */
	fwdata_size = FW_SECTION_SIZE;
	fwdata_ptr = (unsigned char *)fw_entry->data;
	fwcust_size = CUSTOM_SECTION_SIZE;
	fwcust_ptr = fwdata_ptr + FW_SECTION_SIZE;
	fwparam_size = PARAM_SECTION_SIZE;
	fwparam_ptr = fwdata_ptr + SUPPORTED_FW_FILE_SIZE - PARAM_SECTION_SIZE;

	pct1812_set_bank(ts, BANK(6));
	ret = pct1812_power_mode(ts, PWR_MODE_NO_DEEP_SLEEP);
	if (ret) {
		dev_err(ts->dev, "%s: error setting no deep sleep power mode\n", __func__);
	}

	pct1812_set_bank(ts, BANK(1));
	/* check value */
	value = 0x01;
	ret = pct1812_engmode(ts, true, &value);
	if (ret) {
		dev_err(ts->dev, "%s: Error entering flash mode\n", __func__);
		goto failure;
	}

	value = POWERUP_CODE;
	ret = pct1812_i2c_write(ts, FLASH_PUP_REG, &value, sizeof(value));
	if (ret) {
		dev_err(ts->dev, "%s: error setting FLASH_PWRUP\n", __func__);
		goto failure;
	}

	ret = pct1812_flash_section(ts, fwdata_ptr, fwdata_size, ADDRESS_FW);
	if (ret)
		dev_err(ts->dev, "%s: Flash FW error!!!\n", __func__);
	ret = !ret && pct1812_flash_section(ts, fwcust_ptr, fwcust_size, ADDRESS_CUSTOM);
	if (ret)
		dev_err(ts->dev, "%s: Flash customer error!!!\n", __func__);
	ret = !ret && pct1812_flash_section(ts, fwparam_ptr, fwparam_size, ADDRESS_PARAM);
	if (ret)
		dev_err(ts->dev, "%s: Flash params error!!!\n", __func__);

	if (!ret)
		dev_info(ts->dev, "%s: FW update completed successfully\n", __func__);
failure:
	/* reset active page explicitly */
	pct1812_set_bank(ts, BANK(1));
	ret = pct1812_engmode(ts, false, NULL);
	if (ret) {
		dev_err(ts->dev, "%s: Error leaving flash mode\n", __func__);
	}

	pct1812_set_bank(ts, BANK(6));
	ret = pct1812_wait4ready(ts, BOOT_STATUS_REG, BOOT_COMPLETE);
	if (ret) {
		dev_err(ts->dev, "%s: Failed to init\n", __func__);
	}
	mutex_unlock(&ts->cmdlock);
	//__pm_relax(&ts->wake_src);

	pct1812_get_extinfo(ts);

	return ret;
}

static int pct1812_regulator(struct pct1812_data *ts, bool get)
{
	struct pct1812_platform *pdata = ts->plat_data;
	struct regulator *rvdd = NULL;
	struct regulator *rvio = NULL;

	if (get) {
		rvdd = regulator_get(ts->dev, pdata->vdd);
		if (IS_ERR_OR_NULL(rvdd)) {
			dev_err(ts->dev, "%s: Failed to get %s regulator\n",
				__func__, pdata->vdd);
			rvdd = NULL;
		}
		if (pdata->vio)
			rvio = regulator_get(ts->dev, pdata->vio);
		if (IS_ERR_OR_NULL(rvio)) {
			dev_err(ts->dev, "%s: Failed to get %s regulator.\n",
				__func__, pdata->vio);
			rvio = NULL;
		}
	} else {
		regulator_put(ts->reg_vdd);
		if (pdata->vio)
			regulator_put(ts->reg_vio);
	}

	ts->reg_vdd = rvdd;
	ts->reg_vio = rvio;

	DBG_PRINT(DBG_BIT_PM,"vdd=%p, vio=%p\n", rvdd, rvio);

	return 0;
}

static int pct1812_pinctrl_state(struct pct1812_data *ts, bool on)
{
	struct pinctrl_state *state_ptr;
	const char *state_name;
	int error = 0;

	if (!ts->ts_pinctrl)
		return 0;

	if (on) {
		state_name = PINCTRL_STATE_ACTIVE;
		state_ptr = ts->pinctrl_state_active;
	} else {
		state_name = PINCTRL_STATE_SUSPEND;
		state_ptr = ts->pinctrl_state_suspend;
	}

	error = pinctrl_select_state(ts->ts_pinctrl, state_ptr);
	if (error < 0)
		dev_err(ts->dev, "%s: Failed to select %s\n",__func__, state_name);
	else
		DBG_PRINT(DBG_BIT_PM,"set pinctrl state %s\n", state_name);

	return error;
}

static int pct1812_pinctrl_init(struct pct1812_data *info)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	info->ts_pinctrl = devm_pinctrl_get(info->dev);
	if (IS_ERR_OR_NULL(info->ts_pinctrl)) {
		retval = PTR_ERR(info->ts_pinctrl);
		dev_err(info->dev, "%s Target not using pinctrl %d\n",
			__func__, retval);
		goto err_pinctrl_get;
	}

	info->pinctrl_state_active = pinctrl_lookup_state(
			info->ts_pinctrl, PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(info->pinctrl_state_active)) {
		retval = PTR_ERR(info->pinctrl_state_active);
		dev_err(info->dev, "%s Can not lookup %s pinstate %d\n",
			__func__, PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	info->pinctrl_state_suspend = pinctrl_lookup_state(
			info->ts_pinctrl, PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(info->pinctrl_state_suspend)) {
		retval = PTR_ERR(info->pinctrl_state_suspend);
		dev_err(info->dev, "%s Can not lookup %s pinstate %d\n",
			__func__, PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(info->ts_pinctrl);
err_pinctrl_get:
	info->ts_pinctrl = NULL;
	return retval;
}

static int pct1812_power(struct pct1812_data *ts, bool on)
{
	struct pct1812_platform *pdata = ts->plat_data;

	if (on) {
		if (pdata->vio)
			regulator_enable(ts->reg_vio);
		regulator_enable(ts->reg_vdd);
		pct1812_delay_ms(PON_DELAY);
	} else {
		regulator_disable(ts->reg_vdd);
		if (pdata->vio)
			regulator_disable(ts->reg_vio);
	}

	dev_info(ts->dev,"%s: power %s: %s:%s\n", __func__, on ? "ON" : "OFF",
		pdata->vdd, regulator_is_enabled(ts->reg_vdd) ? "on" : "off");
	if (pdata->vio)
		dev_info(ts->dev,"%s: %s: %s:%s\n", __func__, on ? "on" : "off",
			pdata->vio,
			regulator_is_enabled(ts->reg_vio) ? "on" : "off");

	return 0;
}

static int pct1812_run_cmd(struct pct1812_data *ts, enum cmds command, unsigned char vok)
{
	unsigned char code;
	const char *action;
	int ret, counter = 0;

	switch (command) {
	case CMD_RESET:
			action = "RESET";
			code = RESET_CODE;
				break;
	case CMD_RESUME:
			action = "RESUME";
			code = RESUME_CODE;
				break;
	case CMD_SUSPEND:
			action = "SUSPEND";
			code = SUSPEND_CODE;
				break;
	case CMD_MODE_RESET:
			ret = pct1812_set_bank(ts, BANK(6));
			if (ret) {
				dev_err(ts->dev, "%s: Bank(6) set error\n", __func__);
				return ret;
			}
			pct1812_delay_ms(1);
			action = "MODE_RESET";
			code = RESET_CODE;
				break;
	default: return -EINVAL;
	}

run_once_again:
	DBG_PRINT(DBG_BIT_PM,"command: %s\n", action);
	ret = pct1812_i2c_write(ts, SW_RESET_REG, &code, sizeof(code));
	if (ret) {
		dev_err(ts->dev, "%s: Error sending cmd: %d (%d)\n", __func__, command, ret);
		return ret;
	}

	if (!counter++)
		pct1812_delay_ms(10);

	if (code == RESET_CODE) {
		action = "RESUME";
		code = RESUME_CODE;
		goto run_once_again;
	}

	return vok ? pct1812_wait4ready(ts, BOOT_STATUS_REG, vok) : 0;
}

static int pct1812_queued_resume(struct pct1812_data *ts)
{
	int ret;

	DBG_PRINT(DBG_BIT_PM,"enter\n");

	if (atomic_cmpxchg(&ts->touch_stopped, 1, 0) == 0)
		return 0;

	ret = pct1812_run_cmd(ts, CMD_RESUME, BOOT_COMPLETE);
	if (ret) { // set active flag and irq
	}

	return ret;
}

static int pct1812_report_mode(struct pct1812_data *ts,
			unsigned char set_mask, unsigned char clear_mask,
			unsigned char *stored_mask)
{
	unsigned char MASK, val;
	int i, ret;

	QUERY_PARAM_IF(MASK, BANK(0), INTR_MASK_REG, val);
	if (stored_mask)
		*stored_mask = MASK;
	DBG_PRINT(DBG_BIT_ALL,"Current mask 0x%02x\n", MASK);
	for (i = 0; i < 8; i++) {
		if (clear_mask & (1 << i)) {
			MASK &= ~(1 << i); // clear bit
			DBG_PRINT(DBG_BIT_ALL,"mask w/bit cleared %02x\n", MASK);
		}
		if (set_mask & (1 << i)) {
			MASK |= (1 << i);
			DBG_PRINT(DBG_BIT_ALL,"mask w/bit set %02x\n", MASK);
		}
	}
	SET_PARAM_IF(BANK(0), INTR_MASK_REG, MASK);
	DBG_PRINT(DBG_BIT_ALL,"New interrupt mask 0x%02x\n", MASK);
failure:
	return 0;
}

// overall size of test data followed by test type
// Full Panel & SNR have Tx/Rx followed by nodes data
// Draw Line contains only data
#define SELFTEST_HDR_SZ (sizeof(unsigned int) + sizeof(unsigned char))

static int pct1812_selftest_memory(struct pct1812_data *ts, bool allocate)
{
	int ff, ret = 0;

	if (!allocate) {
		ff = ts->frame_count - 1;
		goto dealloc;
	}

	ts->test_data = kzalloc(sizeof(void *) * ts->frame_count, GFP_KERNEL);
	if (!ts->test_data)
		return -ENOMEM;

	for (ff = 0; ff < ts->frame_count; ff++) {
		ts->test_data[ff] = kzalloc(ts->frame_size, GFP_KERNEL);
		if (!ts->test_data[ff]) {
			ret = -ENOMEM;
			goto dealloc;
		}
	}
	DBG_PRINT(DBG_BIT_TEST,"allocated %d frames %d bytes each\n", ts->frame_count, ts->frame_size);

	return 0;

dealloc:
	for (; ff >= 0; ff--)
		kfree(ts->test_data[ff]);
	kfree(ts->test_data);
	ts->test_type = PCT1812_SELFTEST_NONE;
	ts->frame_count = ts->frame_size = ts->test_data_size = ts->frame_sample = 0;
	DBG_PRINT(DBG_BIT_TEST,"free-ed selftest memory\n");

	return ret;
}

static int pct1812_selftest_cleanup(struct pct1812_data *ts)
{
	int ret = 0;

	switch (ts->test_type) {
	case PCT1812_SELFTEST_DRAW_LINE:
			ret = pct1812_run_cmd(ts, CMD_RESET, BOOT_COMPLETE);
			if (ret) {
				dev_err(ts->dev, "%s: Reset failed\n", __func__);
			}
			// no need to restore intr mask; reset done it already!!!
			//pct1812_report_mode(ts, ts->test_intr_mask, 0xff, NULL);
			pct1812_drop_mask_set(ts, 0); // STAT_BIT_TOUCH
	default:
			pct1812_selftest_set(ts, PCT1812_SELFTEST_NONE);
			pct1812_selftest_memory(ts, false);
	}

	ret = pct1812_power_mode(ts, PWR_MODE_AUTO);
	if (ret) {
		dev_err(ts->dev, "%s: error setting auto power mode\n", __func__);
	}

	return ret;
}

#define NANO_SEC 1000000000
#define SEC_TO_MSEC 1000
#define NANO_TO_MSEC 1000000

static inline unsigned int timediff_ms(struct pct1812_data *ts)
{
	struct timespec64 start = ktime_to_timespec64(ts->test_start_time);
	struct timespec64 end = ktime_to_timespec64(ktime_get());
	struct timespec64 temp;
	unsigned int diff_ms;

	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = NANO_SEC + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	diff_ms = (unsigned int)((temp.tv_sec * SEC_TO_MSEC) + (temp.tv_nsec / NANO_TO_MSEC));
	DBG_PRINT(DBG_BIT_TIME,"timediff %ums\n", diff_ms);

	return diff_ms;
}

static void pct1812_work(struct work_struct *work)
{
	struct pct1812_data *ts = container_of(work, struct pct1812_data, worker.work);
	unsigned char status = 0;
	static unsigned char prev = 1;
	const char *action = NULL;
	int mode, cmd, ret = 0;
	int rearm_cmd = CMD_WDOG;
	unsigned int duration;

	while (kfifo_get(&ts->cmd_pipe, &cmd)) {
		mode = pct1812_selftest_get(ts);
		//DBG_PRINT(DBG_BIT_ALL,"cmd = %d, mode = %d\n", cmd, mode);
		switch (cmd) {
		case CMD_CLEANUP:
				duration = timediff_ms(ts);
				if (mode == PCT1812_SELFTEST_IN_PROGRESS ||
					duration < SELFTEST_EXTRA_LONG_INTERVAL) {
					rearm_cmd = CMD_CLEANUP;
					action = "CLEANUP_POSTPONED";
				} else if (mode != PCT1812_SELFTEST_NONE) {
					action = "SELFTEST_CLEANUP";
					pct1812_selftest_cleanup(ts);
				}
					break;
		case CMD_WDOG:
				if (mode == PCT1812_SELFTEST_NONE) {
					action = "WDOG";
#ifndef CALM_WDOG
					ret = pct1812_i2c_read(ts, FRAME_NUM_REG, &status, sizeof(status));
#endif
					if (ret || (prev == status)) {
						dev_warn(ts->dev, "%s: Possible lockup\n", __func__);
					}
#ifndef CALM_WDOG
					/* update frame counter */
					prev = status;
#endif
				} else {
					action = "WDOG_POSTPONED";
				}
					break;
		case CMD_RECOVER:
				action = "RECOVER";
				ret = pct1812_run_cmd(ts, CMD_RESET, BOOT_COMPLETE);
				if (ret) {
						dev_err(ts->dev, "%s: Reset failed\n", __func__);
				}
					break;
		case CMD_RESUME:
				action = "RESUME";
				ret = pct1812_queued_resume(ts);
				if (ret)
						dev_err(ts->dev, "%s: Failed to resume\n", __func__);
					break;
		}
	}

	if (action && strcmp(action, "WDOG"))
		DBG_PRINT(DBG_BIT_WORK,"action: %s\n", action);

	pct1812_fifo_cmd_add(ts, rearm_cmd, WDOG_INTERVAL);
}
#if 0
static int pct1812_suspend(struct device *dev)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	int ret;

	DBG_PRINT(DBG_BIT_PM,"enter\n");

	if (atomic_cmpxchg(&ts->touch_stopped, 0, 1) == 1)
		return 0;

	ret = pct1812_run_cmd(ts, CMD_SUSPEND, 0);
	if (!ret) { // set active flag and irq
	}

	return ret;
}

static int pct1812_resume(struct device *dev)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);

	//atomic_set(&ts->resume_should_stop, 0);
	kfifo_put(&ts->cmd_pipe, CMD_RESUME);
	/* schedule_delayed_work returns true if work has been scheduled */
	/* and false otherwise, thus return 0 on success to comply POSIX */
	return schedule_delayed_work(&ts->worker, 0) == false;
}
#endif

static int pct1812_input_dev(struct pct1812_data *ts, bool alloc)
{
	static char ts_phys[64] = {0};
	int ret = 0;

	if (!alloc)
		goto cleanup;

	ts->idev = input_allocate_device();
	if (!ts->idev)
		return -ENOMEM;

	snprintf(ts_phys, sizeof(ts_phys), "pct1812ff_touchpad/input1");
	ts->idev->phys = ts_phys;
	ts->idev->name = "PixArt PCT1812FF Touchpad";
	ts->idev->id.bustype = BUS_I2C;
	ts->idev->dev.parent = ts->dev;

	set_bit(EV_SYN, ts->idev->evbit);
	set_bit(EV_KEY, ts->idev->evbit);
	set_bit(EV_ABS, ts->idev->evbit);
#if 0
	input_set_capability(ts->idev, EV_REL, REL_X);
	input_set_capability(ts->idev, EV_REL, REL_Y);
#endif
	set_bit(BTN_TOUCH, ts->idev->keybit);
	set_bit(BTN_TOOL_FINGER, ts->idev->keybit);
	set_bit(BTN_TOOL_DOUBLETAP, ts->idev->keybit);
	set_bit(INPUT_PROP_POINTER, ts->idev->propbit);

	input_set_abs_params(ts->idev, ABS_MT_POSITION_X, 0, ts->x_res, 0, 0);
	input_set_abs_params(ts->idev, ABS_MT_POSITION_Y, 0, ts->y_res, 0, 0);
	input_mt_init_slots(ts->idev, ts->max_points, INPUT_MT_POINTER);

	ret = input_register_device(ts->idev);
	if (ret) {
		dev_err(ts->dev, "%s: failed to register input device: %d\n", __func__, ret);
		input_free_device(ts->idev);
	}

	return ret;

cleanup:
	input_unregister_device(ts->idev);

	return ret;
}

static int pct1812_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pct1812_data *ts;
	struct pct1812_platform *pdata;
	int ret = 0;

 	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: EIO err!!!\n", __func__);
		return -EIO;
	}

	ts = kzalloc(sizeof(struct pct1812_data), GFP_KERNEL);
	if (!ts)
		goto error_allocate_mem;

	if (!client->dev.of_node) {
		dev_err(&client->dev, "%s: Failed to locate dt\n", __func__);
		goto error_allocate_mem;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct pct1812_platform), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "%s: Failed to allocate platform data\n", __func__);
		goto error_allocate_pdata;
	}

	client->dev.platform_data = pdata;
	pdata->client = client;

#if defined(STATIC_PLATFORM_DATA)
	memcpy(pdata, (const void *)&pct1812_pd, sizeof(struct pct1812_platform));
#else
	ret = pct1812_parse_dt(client);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to parse dt\n", __func__);
		goto error_allocate_mem;
	}
#endif

	ret = pct1812_platform_init(pdata);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to init platform\n", __func__);
		goto error_allocate_mem;
	}

 	ts->client = client;
	ts->dev = &client->dev;
	ts->plat_data = pdata;
	ts->debug = DBG_BIT_PM;

	i2c_set_clientdata(client, ts);
	dev_set_drvdata(&client->dev, ts);

	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->cmdlock);
	mutex_init(&ts->eventlock);

	INIT_DELAYED_WORK(&ts->worker, pct1812_work);

	pct1812_pinctrl_init(ts);
	pct1812_pinctrl_state(ts, true);

	pct1812_regulator(ts, true);
	pct1812_power(ts, true);

	pct1812_delay_ms(TOUCH_RESET_DELAY);
	ret = pct1812_wait4ready(ts, BOOT_STATUS_REG, BOOT_COMPLETE);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to init\n", __func__);
		goto error_init;
	}

	pct1812_drop_mask_set(ts, 0); // STAT_BIT_TOUCH);
	pct1812_get_extinfo(ts);

	ret = kfifo_alloc(&ts->cmd_pipe, sizeof(unsigned int)* 10, GFP_KERNEL);
	if (ret)
		goto error_init;

	ret = request_threaded_irq(client->irq, NULL, pct1812_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, MYNAME, ts);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Unable to request threaded irq\n", __func__);
		goto error_fifo_alloc;
	}

	/* prevent unbalanced irq enable */
	ts->irq_enabled = true;
	ret = pct1812_input_dev(ts, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Unable register input\n", __func__);
		goto error_input_alloc;
	}
	/* sensing on */
	pct1812_sensing_on(ts);
	pct1812_fifo_cmd_add(ts, CMD_WDOG, WDOG_INTERVAL);

	ret = sysfs_create_group(&client->dev.kobj, &pct1812_attrs_group);
	if (ret)
		dev_warn(&client->dev, "%s: Error creating sysfs entries %d\n", __func__, ret);

	sysfs_create_bin_file(&client->dev.kobj, &selftest_bin_attr);

	return 0;

error_input_alloc:
	free_irq(client->irq, ts);

error_fifo_alloc:
	kfifo_free(&ts->cmd_pipe);

error_init:
	pct1812_power(ts, false);
	pct1812_regulator(ts, false);

error_allocate_mem:
error_allocate_pdata:

	dev_err(&client->dev, "%s: failed(%d)\n", __func__, ret);

	return ret;
}

static ssize_t ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s%s\n%s%02d%02d%02d%02d\n%s%02x%02x%02x%02x\n",
			"Product_ID: ", (ts->model == 0x47) &&
				(ts->product_id & 0x80) ? "pct1812ff" : "unknown",
			"Build ID: ", ts->major, ts->minor, ts->revision, ts->patch,
			"Config ID: ", ts->custom_block[0], ts->custom_block[1],
				ts->custom_block[2], ts->custom_block[3]);
}

static ssize_t vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "pixart");
}

static ssize_t irq_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	int value = gpio_get_value(ts->plat_data->irq_gpio);

	return scnprintf(buf, PAGE_SIZE, "%s", value ? "High" : "Low");
}

static ssize_t drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d", ts->irq_enabled ? 1 : 0);
}

static ssize_t drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret = 0;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		dev_err(ts->dev, "%s: Conversion failed\n", __func__);
		return -EINVAL;
	}

	pct1812_set_irq(ts, !!(value));

	return size;
}

static int pct1812_raw_data_frame(struct pct1812_data *ts,
		unsigned char *buffer)
{
	unsigned char cmd, tx, rx;
	int ret = 0;

	ret = pct1812_set_bank(ts, BANK(6));
	if (ret)
		goto failure;

	QUERY_PARAM_IF(tx, BANK(0), 0x5a, cmd);
	QUERY_PARAM_IF(rx, BANK(0), 0x59, cmd);
	dev_dbg(ts->dev, "%s: Tx/Rx=%d/%d\n", __func__, tx, rx);

	ret = pct1812_set_bank(ts, BANK(1));
	if (ret)
		goto failure;
	cmd = 0x40;
	ret = pct1812_i2c_write(ts, 0x0d, &cmd, sizeof(cmd));
	if (ret)
		goto failure;
	cmd = 0x06;
	ret = pct1812_i2c_write(ts, 0x0e, &cmd, sizeof(cmd));
	if (ret)
		goto failure;

	ret = pct1812_set_bank(ts, BANK(2));
	if (ret)
		goto failure;
	cmd = 0x05;
	ret = pct1812_i2c_write(ts, SRAM_SELECT_REG, &cmd, sizeof(cmd));
	if (ret)
		goto failure;
	cmd = 0x0;
	ret = pct1812_i2c_write(ts, SRAM_NSC_REG, &cmd, sizeof(cmd));
	if (ret)
		goto failure;
	// read data from SRAM port
	ret = pct1812_i2c_read(ts, SRAM_PORT_REG, buffer, ts->frame_size);
	if (ret)
		goto failure;
	cmd = 0x1;
	ret = pct1812_i2c_write(ts, SRAM_NSC_REG, &cmd, sizeof(cmd));
	if (ret)
		goto failure;

	ret = pct1812_set_bank(ts, BANK(1));
	if (ret)
		goto failure;
	cmd = 0x0;
	ret = pct1812_i2c_write(ts, 0x0d, &cmd, sizeof(cmd));
	if (ret)
		goto failure;
	ret = pct1812_i2c_write(ts, 0x0e, &cmd, sizeof(cmd));
	if (ret)
		goto failure;

	ret = pct1812_set_bank(ts, BANK(6));
	if (ret)
		goto failure;
	cmd = 0x00; // clear interrupt
	ret = pct1812_i2c_write(ts, STATUS_REG, &cmd, sizeof(cmd));
	if (ret)
		goto failure;

	return 0;

failure:
	dev_err(ts->dev, "%s: Read error\n", __func__);

	return -EIO;
}

static unsigned char *selftest_data_ptr(struct pct1812_data *ts,
		unsigned int offset, unsigned int need2read, unsigned int *avail)
{
	int ff;
	unsigned char *bptr = NULL;

	for (ff = 0; ff < ts->frame_count; ff++) {
		if (offset >= ts->frame_size) {
			offset -= ts->frame_size;
			continue;
		}
		bptr = ts->test_data[ff];
		bptr += offset;
		if (need2read < (ts->frame_size - offset))
			*avail = need2read;
		else
			*avail = ts->frame_size - offset;
		break;
	}

	return bptr;
}

static ssize_t selftest_bin_show(struct file *filp, struct kobject *kobp,
		struct bin_attribute *bin_attr, char *buf, loff_t pos, size_t count)
{
	int mode;
	unsigned char *bptr;
	unsigned int remain, available;
	unsigned int bOff = 0;
	unsigned int dOffset = 0;
	struct i2c_client *client = kobj_to_i2c_client(kobp);
	struct pct1812_data *ts = i2c_get_clientdata(client);

	if (!ts || pos < 0)
		return -EINVAL;

	mode = pct1812_selftest_get(ts);
	if (mode == PCT1812_SELFTEST_NONE || ts->test_data_size == 0) {
		dev_warn(ts->dev, "%s: No selftest results available\n", __func__);
		return 0;
	}

	if (pos >= ts->test_data_size) {
		dev_warn(ts->dev, "%s: Position %lu beyond data boundary of %u\n",
				__func__, pos, ts->test_data_size);
		return 0;
	}

	if (count > ts->test_data_size - pos)
		count = ts->test_data_size - pos;
	/* available data size */
	remain = count;

	if (pos == 0) {
		DBG_PRINT(DBG_BIT_TEST,"sending selftest data header\n");
		memcpy(buf, &ts->test_data_size, sizeof(unsigned int));
		bOff += sizeof(unsigned int);
		memcpy(buf + bOff, &ts->test_type, sizeof(unsigned char));
		bOff += sizeof(unsigned char);
		if (ts->test_type != PCT1812_SELFTEST_DRAW_LINE) {
			memcpy(buf + bOff, &ts->tx_count, sizeof(unsigned char));
			bOff += sizeof(unsigned char);
			memcpy(buf + bOff, &ts->rx_count, sizeof(unsigned char));
			bOff += sizeof(unsigned char);
		}
		/* available data size after sending header */
		remain -= bOff;
	} else if (pos >= ts->test_hdr_size) {
		/* offset within data buffer (excludes header) */
		dOffset = pos - ts->test_hdr_size;
	}

	DBG_PRINT(DBG_BIT_TEST,"start at offset %u, remaining %u\n", dOffset, remain);
	while ((bptr = selftest_data_ptr(ts, dOffset, remain, &available)) &&
				available > 0) {
		DBG_PRINT(DBG_BIT_TEST,"available for copying %u bytes at offset %u\n", available, dOffset);
		memcpy(buf + bOff, bptr, available);
		bOff += available;
		dOffset += available;
		remain -= available;
	}

	if ((pos + count) >= ts->test_data_size) {
		pct1812_selftest_cleanup(ts);
	}

	return count;
}

static ssize_t pct1812_print_raw_data(struct pct1812_data *ts, char *buf)
{
	unsigned char *bptr;
	int ival, ll, rr, cc;
	ssize_t blen = 0;

	blen = scnprintf(buf, PAGE_SIZE, "%08x;%02x;%02x,%02x\n",
				ts->test_data_size, ts->test_type, ts->tx_count, ts->rx_count);
	for (ll = 0; ll < ts->frame_count; ll++) {
		blen += scnprintf(buf + blen, PAGE_SIZE - blen,
								"       Tx0   Tx1   Tx2   Tx3   Tx4   Tx5\n");
		bptr = ts->test_data[ll];
		for (rr = 0; rr < ts->rx_count; rr++) {
			blen += scnprintf(buf + blen, PAGE_SIZE - blen, "Rx%d: ", rr);
			for (cc = 0; cc < ts->tx_count; cc++) {
				DBG_PRINT(DBG_BIT_TEST,"offset %d\n", (int)(bptr - ts->test_data[ll]));
				ival = comp2_16b(bptr);
				bptr += 2;
				blen += scnprintf(buf + blen, PAGE_SIZE - blen, "%5d ", ival);
			}
			blen += scnprintf(buf + blen, PAGE_SIZE - blen, "\n");
		}
	}

	return blen;
}

static ssize_t pct1812_print_coords(struct pct1812_data *ts, char *buf)
{
	int ss;
	ssize_t blen = 0;

	blen = scnprintf(buf, PAGE_SIZE, "%08x;%02x\n", ts->test_data_size, ts->test_type);
	for (ss = 0; ss < ts->frame_sample; ss++) {
		blen += scnprintf(buf + blen, PAGE_SIZE - blen,
					"%5d,%5d;\n", UINT16_X(ts->test_data[ss]), UINT16_Y(ts->test_data[ss]));
	}

	return blen;
}

static ssize_t selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	int mode;
	ssize_t blen = 0;

	mode = pct1812_selftest_get(ts);
	if (mode == PCT1812_SELFTEST_IN_PROGRESS ||
				ts->test_type == PCT1812_SELFTEST_NONE ||
				ts->test_data_size == 0) {
		dev_warn(ts->dev, "%s: No selftest results available\n", __func__);
		return blen;
	}

	if (ts->test_type == PCT1812_SELFTEST_FULL ||
				ts->test_type == PCT1812_SELFTEST_SNR) {
		DBG_PRINT(DBG_BIT_TEST,"%d frame(s) available\n", ts->frame_count);
		blen = pct1812_print_raw_data(ts, buf);
	} else {
		DBG_PRINT(DBG_BIT_TEST,"%d sample(s) available\n", ts->frame_sample);
		blen = pct1812_print_coords(ts, buf);
	}

	pct1812_selftest_cleanup(ts);

	return blen;
}

static ssize_t selftest_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	unsigned char *test, cmd;
	int mode, ret = 0;
	int ll;

	mode = pct1812_selftest_get(ts);
	if (mode != PCT1812_SELFTEST_NONE) {
		dev_err(ts->dev, "%s: Selftest in progress\n", __func__);
		return -EBUSY;
	}

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		dev_err(ts->dev, "%s: Conversion failed\n", __func__);
		return -EINVAL;
	} else if (value <= PCT1812_SELFTEST_NONE || value >= PCT1812_SELFTEST_MAX) {
		dev_err(ts->dev, "%s: Invalid selftest id %lu\n", __func__, value);
		return -EINVAL;
	}

	ts->test_type = mode = value & 0xff;
	switch (mode) {
	case PCT1812_SELFTEST_FULL:
			ts->frame_size = ts->tx_count * ts->rx_count * sizeof(unsigned char) * 2;
			ts->frame_count = 1;
			ts->test_hdr_size = SELFTEST_HDR_SZ + sizeof(unsigned char) * 2;				
			ts->test_data_size = ts->frame_count * ts->frame_size + ts->test_hdr_size;
					break;
	case PCT1812_SELFTEST_SNR:
			ts->frame_size = ts->tx_count * ts->rx_count * sizeof(unsigned char) * 2;
			ts->frame_count = SNR_LOOP_COUNT;
			ts->test_hdr_size = SELFTEST_HDR_SZ + sizeof(unsigned char) * 2;				
			ts->test_data_size = ts->frame_count * ts->frame_size + ts->test_hdr_size;
					break;
	case PCT1812_SELFTEST_DRAW_LINE:
			ts->frame_size = sizeof(unsigned char) * 4;
			ts->frame_count = 100; // alloc memory for 100 samples
			ts->test_hdr_size = SELFTEST_HDR_SZ;
			ts->test_data_size = ts->test_hdr_size;
					break;
	}

	ret = pct1812_power_mode(ts, PWR_MODE_NO_DEEP_SLEEP);
	if (ret) {
		dev_err(ts->dev, "%s: error setting no deep sleep power mode\n", __func__);
	}

	ret = pct1812_selftest_memory(ts, true);
	if (ret)
		return -ENOMEM;

	switch (mode) {
	case PCT1812_SELFTEST_FULL:
			ts->test_type = PCT1812_SELFTEST_FULL;
			pct1812_selftest_set(ts, PCT1812_SELFTEST_FULL);
			test = "SELFTEST_FULL";
					break;
	case PCT1812_SELFTEST_SNR:
			ts->test_type = PCT1812_SELFTEST_SNR;
			pct1812_selftest_set(ts, PCT1812_SELFTEST_SNR);
			test = "SELFTEST_SNR";
					break;
	case PCT1812_SELFTEST_DRAW_LINE:
			ts->test_type = PCT1812_SELFTEST_DRAW_LINE;
			pct1812_selftest_set(ts, PCT1812_SELFTEST_DRAW_LINE);
			ts->test_start_time = ktime_get();
			ret = pct1812_run_cmd(ts, CMD_RESET, BOOT_COMPLETE);
			if (ret) {
				dev_err(ts->dev, "%s: Failed to reset\n", __func__);
				goto failure;
			}
			pct1812_report_mode(ts, STAT_BIT_EABS, 0xff, &ts->test_intr_mask);
			pct1812_drop_mask_set(ts, STAT_BIT_GESTURE);
			// arm timed out cleanup
			pct1812_fifo_cmd_add(ts, CMD_CLEANUP, SELFTEST_EXTRA_LONG_INTERVAL);
					return size;
	}

	// disable IRQ
	pct1812_set_irq(ts, false);
	// RESET
	ret = pct1812_run_cmd(ts, CMD_MODE_RESET, MODE_COMPLETE);
	if (ret) {
		dev_err(ts->dev, "%s: Failed to reset\n", __func__);
		goto failure;
	}
	// INITIAL
	ret = pct1812_set_bank(ts, BANK(6));
	if (ret) {
		dev_err(ts->dev, "%s: Pre-initial error\n", __func__);
		goto failure;
	}
	cmd = 0x00;
	ret = pct1812_user_access(ts, BANK(0), 0x08, &cmd, sizeof(cmd), AWRITE);
	if (ret) {
		dev_err(ts->dev, "%s: Write error: bank=0, addr=0x08, value=0x00\n", __func__);
		goto failure;
	}
	cmd = 0x01;
	ret = pct1812_user_access(ts, BANK(0), 0x15, &cmd, sizeof(cmd), AWRITE);
	if (ret) {
		dev_err(ts->dev, "%s: Write error: bank=0, addr=0x15, value=0x01\n", __func__);
		goto failure;
	}
	cmd = 0x00;
	ret = pct1812_user_access(ts, BANK(1), 0x90, &cmd, sizeof(cmd), AWRITE);
	if (ret) {
		dev_err(ts->dev, "%s: Write error: bank=1, addr=0x90, value=0x00\n", __func__);
		goto failure;
	}
	// SNR selftest requires an extra step
	if ((value & 0xff) == PCT1812_SELFTEST_SNR) {
		cmd = 0x0f;
		ret = pct1812_user_access(ts, BANK(1), 0x91, &cmd, sizeof(cmd), AWRITE);
		if (ret) {
			dev_err(ts->dev, "%s: Write error: bank=1, addr=0x91, value=0x0f\n", __func__);
			goto failure;
		}
	}
	cmd = 0x00;
	ret = pct1812_user_access(ts, BANK(0), 0x2d, &cmd, sizeof(cmd), AWRITE);
	if (ret) {
		dev_err(ts->dev, "%s: Write error: bank=0, addr=0x08, value=0x00\n", __func__);
		goto failure;
	}
	cmd = 0x01;
	ret = pct1812_user_access(ts, BANK(0), 0x08, &cmd, sizeof(cmd), AWRITE);
	if (ret) {
		dev_err(ts->dev, "%s: Write error: bank=0, addr=0x08, value=0x01\n", __func__);
		goto failure;
	}

	pct1812_selftest_set(ts, PCT1812_SELFTEST_IN_PROGRESS);
	dev_info(ts->dev, "%s: Performing %s\n", __func__, test);
	for (ll = 0; ll < ts->frame_count; ll++) {
		ret = pct1812_raw_data_frame(ts, ts->test_data[ll]);
		if (ret) {
			dev_err(ts->dev, "%s: Error reading frame %d\n", __func__, ll);
			break;
		}
	}
	/* reset in case of failed selftest data collection */
	if (ret)
		goto failure;

	/* restore selftest type */
	pct1812_selftest_set(ts, mode);
	ret = pct1812_run_cmd(ts, CMD_RESET, BOOT_COMPLETE);
	if (ret)
		dev_err(ts->dev, "%s: Reset error\n", __func__);
	// enable IRQ
	pct1812_set_irq(ts, true);

	return size;

failure:
	ret = pct1812_run_cmd(ts, CMD_RESET, BOOT_COMPLETE);
	if (ret)
		dev_err(ts->dev, "%s: Reset error\n", __func__);
	pct1812_selftest_set(ts, PCT1812_SELFTEST_NONE);
	// enable IRQ
	pct1812_set_irq(ts, true);

	return size;
}

static ssize_t doreflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	char fname[64];
	int ret = 0;

	if (size >= sizeof(fname)) {
		dev_err(ts->dev, "%s: File name %d too long (max %d)\n", __func__, size, sizeof(fname));
		return -EINVAL;
	}

	strncpy(fname, buf, sizeof(fname));
	if (fname[size - 1] == '\n')
		fname[size - 1] = 0;

	ret = pct1812_fw_update(ts, fname);
	if (ret)
		dev_err(ts->dev, "%s: FW update failed\n", __func__);

	return ret ? -EIO : size;
}

static ssize_t forcereflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	enum cmds cmd;
	unsigned char vok;
	int ret = 0;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		dev_err(ts->dev, "%s: Conversion failed\n", __func__);
		return -EINVAL;
	}else if (value == 0) {
		cmd = CMD_MODE_RESET;
		vok = MODE_COMPLETE;
	} else {
		cmd = CMD_RESET;
		vok = BOOT_COMPLETE;
	}

	dev_info(ts->dev, "%s: Performing reset %s\n", __func__, cmd == CMD_MODE_RESET ? "MODE" : "HW");

	ret = pct1812_run_cmd(ts, cmd, vok);
	if (ret)
		dev_err(ts->dev, "%s: Incomplete reset\n", __func__);

	return size;
}

static ssize_t info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	ssize_t blen;

	pct1812_get_extinfo(ts);

	blen = scnprintf(buf, PAGE_SIZE, "   Product: %sPixArt %s\n",
				ts->model == 0x47 ? "" : "Non-",
				(ts->product_id & 0x80) ? "PCT1812FF" : "unknown");
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "    FW ver: %d.%d\n", ts->major, ts->minor);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "    FW rev: %d patch %d\n", ts->revision, ts->patch);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "     Tx/Rx: %dx%d\n", ts->tx_count, ts->rx_count);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "Resolution: %dx%d\n", ts->x_res, ts->y_res);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "Max points: %d\n", ts->max_points);
	blen += scnprintf(buf + blen, PAGE_SIZE - blen, "Boot block: %d\n", ts->boot_block);

	return blen;
}

static ssize_t mask_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	long value = 0;
	unsigned char set_mask, clear_mask;
	int ret;

	ret = kstrtol(buf, 10, &value);
	if (ret < 0) {
		dev_err(ts->dev, "%s: Conversion failed\n", __func__);
		return -EINVAL;
	} else if (value < -255 || value > 255) {
		dev_err(ts->dev, "%s: Value %ld is out of range [-255,255]\n", __func__);
		return -EINVAL;
	}

	if (value < 0) {
		value = -value;
		clear_mask = value & 0xff;
		set_mask = 0;
	} else {
		set_mask = value & 0xff;
		clear_mask = 0;
	}

	ret = pct1812_report_mode(ts, set_mask, clear_mask, NULL);
	if (ret)
		dev_err(ts->dev, "%s: Intr mask set error\n", __func__);

	return ret ? -EIO: size;
}

static ssize_t mask_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	unsigned char mask = 0;
	unsigned char val;
	int ret;
	QUERY_PARAM(mask, BANK(0), INTR_MASK_REG, val);
	return scnprintf(buf, PAGE_SIZE, "%02x\n", mask);
}

static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	long value = 0;
	unsigned char mask;
	int ret;

	ret = kstrtol(buf, 10, &value);
	if (ret < 0) {
		dev_err(ts->dev, "%s: Conversion failed\n", __func__);
		return -EINVAL;
	} else if (value < -255 || value > 255) {
		dev_err(ts->dev, "%s: Value %ld is out of range [-255,255]\n", __func__);
		return -EINVAL;
	}

	if (value < 0) {
		value = -value;
		mask = value & 0xff;
		ts->debug &= ~mask;
	} else {
		mask = value & 0xff;
		ts->debug |= mask;
	}

	return ret ? -EINVAL: size;
}

static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%02x\n", ts->debug);
}

static const struct i2c_device_id pct1812_id[] = {
	{ MYNAME, 0 },
	{ },
};

static const struct of_device_id pct1812_match_table[] = {
	{ .compatible = "pixart,pct1812_ts",},
	{ },
};

static struct i2c_driver pct1812_driver = {
        .driver = {
                .owner = THIS_MODULE,
                .name = MYNAME,
                .of_match_table = pct1812_match_table,
        },
        .probe = pct1812_probe,
        .id_table = pct1812_id,
};

module_i2c_driver(pct1812_driver);

MODULE_AUTHOR("Konstantin Makariev <kmakariev@lenovo.com>");
MODULE_DESCRIPTION("I2C Driver for PixArt Imaging PCT1812FF");
MODULE_LICENSE("GPL v2");
