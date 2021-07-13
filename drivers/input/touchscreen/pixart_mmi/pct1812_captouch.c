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
#define pr_fmt(fmt) "%s: %s: " fmt, MYNAME, __func__

#define DEBUG
//#undef DEBUG

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <linux/kfifo.h>

#define AREAD (1)
#define AWRITE (0)

#define INVALID_BYTE2 0xebeb
#define INVALID_BYTE 0xeb

#define ADDRESS_FW 0x00
#define ADDRESS_PARAMS 0xe0

#define SUPPORTED_FW_FILE_SIZE 0x0000F000
#define FW_SECTION_SIZE 0x0000E000

#define PCT1812_SECTOR_SIZE (0x1000) // 4K
#define CHUNK_SZ 256

#define FW_ADDR_BASE 0x00
#define PARAM_ADDR_BASE 0xe0

#define POWERUP_CODE 0x02
#define RESET_CODE 0xaa
#define RESUME_CODE 0xbb
#define ENGINEERING_CODE 0xcc
#define SUSPEND_CODE 0x99

#define FRAME_NUM_REG 0x00
#define GEST_TYPE_REG 0x60
#define GEST_X0_REG 0x62
#define GEST_X1_REG 0x63
#define BOOT_STATUS_REG 0x70
#define STATUS_REG 0x71
#define USER_BANK_REG 0x73
#define USER_ADDR_REG 0x74
#define USER_DATA_REG 0x75
#define SW_RESET_REG 0x7a
#define DEEP_SLEEP_REG 0x7c

#define BANK(a) (a)
#define KEYS_NUM 8

// USER BANK 0
#define VER_LOW_REG 0x7e
#define VER_HIGH_REG 0x7f
// USER BANK 1
#define FLASH_PUP_REG 0x0d
#define KEY1_REG 0x2c
#define KEY2_REG 0x2d
// USER BANK 3
#define FUNCT_CTRL_REG 0x02
#define KEYS_NUM_REG 0x1a
#define KEY_THRES_BASE 0x00
// USER BANK 4
#define FLASH_STATUS_REG 0x1c
#define FLASH_ADDR0_REG 0x24
#define FLASH_ADDR1_REG 0x25
#define FLASH_ADDR2_REG 0x26

#define STAT_BIT_ERR (1 << 0)
#define STAT_BIT_EVENT (1 << 3)
#define STAT_BIT_WDOG (1 << 7)

#define BOOT_COMPLETE 1
#define TOUCH_RESET_DELAY 10
#define CMD_WAIT_DELAY 10
#define WDOG_INTERVAL 1000
#define PON_DELAY 200

#define CNT_I2C_RETRY 3
#define CNT_WAIT_RETRY 50

#define I2C_WRITE_BUFFER_SIZE 4
#define I2C_READ_BUFFER_SIZE 4

#define PINCTRL_STATE_ACTIVE "touchpad_active"
#define PINCTRL_STATE_SUSPEND "touchpad_suspend"

enum pwr_modes {
	PWR_MODE_OFF,
	PWR_MODE_RUN,
	PWR_MODE_LPM,
	PWR_MODE_DEEP_SLEEP,
	PWR_MODE_SHUTDOWN,
	PWR_MODE_MAX
};

enum cmds {
	CMD_INIT,
	CMD_RESET,
	CMD_SUSPEND,
	CMD_RESUME,
	CMD_RECOVER,
	CMD_WDOG
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

	struct pinctrl *pinctrl;

	const char *vdd;
	const char *vio;
};

struct pct1812_data {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct pct1812_platform *plat_data;

	struct regulator *reg_vdd;
	struct regulator *reg_vio;

	int tx_count;
	int rx_count;

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

	unsigned short version;
	unsigned char func_ctrl;
	unsigned char keys;
};
#if 0
static int inline get_cmd(void)
{
	int cmd = 0;

	mutex_lock(&ts->cmdlock);
	cmd = ts->cmd;
	mutex_unloc(&ts->cmdlock);

	return cmd;
}

static void inline set_cmd(int cmd)
{
	mutex_lock(&ts->cmdlock);
	ts->cmd = cmd;
	mutex_unloc(&ts->cmdlock);
}
#endif
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
		if (ret == 2)
			break;
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
#if 0
	if (ts->debug_flag & SEC_TS_DEBUG_PRINT_I2C_READ_CMD) {
		pr_info("sec_input:i2c_cmd: R: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}
#endif
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
		if (ret == 1)
			break;
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
#if 0
	if (ts->debug_flag & SEC_TS_DEBUG_PRINT_I2C_WRITE_CMD) {
		pr_info("sec_input:i2c_cmd: W: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}
#endif
	return (ret == 1) ? 0 : ret;
}

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
	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN, "pct1812,irq");
		if (ret) {
			dev_err(&client->dev, "%s: Unable to request gpio [%d]\n", __func__, pdata->irq_gpio);
			return -EINVAL;
		}
		dev_info(&client->dev, "%s: using irq gpio %d\n", __func__, pdata->irq_gpio);
	} else {
		dev_err(&client->dev, "%s: Failed to get irq gpio\n", __func__);
		return -EINVAL;
	}
#if 0
	pdata->rst_gpio = of_get_named_gpio(np, "pct1812,reset-gpio", 0);
	if (gpio_is_valid(pdata->rst_gpio)) {
		ret = gpio_request_one(pdata->rst_gpio, GPIOF_DIR_OUT, "pct1812,reset");
		if (ret) {
			dev_err(&client->dev, "%s: Unable to request gpio [%d]\n", __func__, pdata->rst_gpio);
			return -EINVAL;
		}
		dev_info(&client->dev, "%s: using reset gpio %d\n", __func__, pdata->rst_gpio);
	}
#endif
	client->irq = gpio_to_irq(pdata->irq_gpio);

	if (!of_property_read_u32_array(np, "pct1812,max_coords", coords, 2)) {
		pdata->max_x = coords[0] - 1;
		pdata->max_y = coords[1] - 1;
		dev_info(dev, "%s: max_coords:(%d,%d)\n", __func__, pdata->max_x, pdata->max_y);
	}

	return ret;
}
static int pct1812_read_touch_event(struct pct1812_data *ts)
{
	unsigned char gest, gdata[2];
	int ret;

	ret = pct1812_i2c_read(ts, GEST_TYPE_REG, &gest, sizeof(gest));
	if (ret)
		return -EIO;
	pr_debug("gesture 0x%02x\n", gest);
	switch(gest & 0x1f) {
		case GEST_HORIZ_SCROLL:
		case GEST_VERT_SCROLL:
				ret = pct1812_i2c_read(ts, GEST_X0_REG, gdata, sizeof(gdata));
				pr_debug("X0: 0x%04x\n", (gdata[1] << 8) | gdata[0]);
						break;
		case GEST_TAP:
		case GEST_DBL_TAP:
		case GEST_UP_SWIPE:
		case GEST_DWN_SWIPE:
		case GEST_LFT_SWIPE:
		case GEST_RGHT_SWIPE:
						break;
		default: return -EINVAL;
	}

	return 0;
}

static irqreturn_t pct1812_irq_handler(int irq, void *ptr)
{
	struct pct1812_data *ts = (struct pct1812_data *)ptr;
	unsigned char status = 0;
	bool ack = true;
	int ret;

	mutex_lock(&ts->eventlock);
	ret = pct1812_i2c_read(ts, STATUS_REG, &status, sizeof(status));
	if (status & STAT_BIT_EVENT) {
		pct1812_read_touch_event(ts);
	} else if ((status & STAT_BIT_ERR) || (status & STAT_BIT_WDOG)) {
		ack = false;
		pct1812_fifo_cmd_add(ts, CMD_RECOVER, TOUCH_RESET_DELAY);
	}
	if (ack) {
		status = 0; // ACK interrupt
		ret = pct1812_i2c_write(ts, STATUS_REG, &status, sizeof(status));
	}
	mutex_unlock(&ts->eventlock);

	return IRQ_HANDLED;
}

static int pct1812_user_access(struct pct1812_data *ts,
	unsigned char bank, unsigned char addr,
	unsigned char *value, unsigned int size, bool R)
{
	unsigned char val;
	int ret;

	pr_debug("Access %c: b%d:0x%02x val 0x%02x\n",
			R ? 'R' : 'W', bank, addr, R ? 0 : *value);

	ret = pct1812_i2c_write(ts, USER_BANK_REG, &bank, sizeof(bank));
	if (ret)
		return -EIO;

	ret = pct1812_i2c_write(ts, USER_ADDR_REG, &addr, sizeof(addr));
	if (ret)
		return -EIO;

	if (R) { // read access
		if (value)
			*value = 0;
		ret = pct1812_i2c_read(ts, USER_DATA_REG, &val, sizeof(val));
		if (!ret && value)
			*value = val;
	} else {
		ret = pct1812_i2c_write(ts, USER_DATA_REG, value, size);
	}

	return ret;
}

static int pct1812_get_extinfo(struct pct1812_data *ts)
{
	unsigned char val;
	unsigned char t_addr;
	unsigned short tk1, tk2, version = 0;
	int i, ret;

	ts->version = INVALID_BYTE2;
	ret = pct1812_user_access(ts, BANK(0), VER_LOW_REG, &val, sizeof(val), AREAD);
	if (!ret)
		version |= val;

	ret = pct1812_user_access(ts, BANK(0), VER_HIGH_REG, &val, sizeof(val), AREAD);
	if (!ret)
		version |= (val << 8);
	ts->version = version;

	dev_info(ts->dev, "%s: FW version 0x%04x\n", __func__, ts->version);

	ts->func_ctrl = INVALID_BYTE;
	ret = pct1812_user_access(ts, BANK(3), FUNCT_CTRL_REG, &val, sizeof(val), AREAD);
	if (!ret)
		ts->func_ctrl = val;

	if (ts->func_ctrl != INVALID_BYTE)
		dev_info(ts->dev, "%s: Func CTRL 0x%02x\n", __func__, ts->func_ctrl);

	ts->keys = INVALID_BYTE;
	ret = pct1812_user_access(ts, BANK(3), KEYS_NUM_REG, &val, sizeof(val), AREAD);
	if (!ret)
		ts->keys = val;

	if (ts->keys != INVALID_BYTE)
		dev_info(ts->dev, "%s: Keys supported %d\n", __func__, ts->keys);

	t_addr = KEY_THRES_BASE;
	for (i = 0; i < KEYS_NUM; i++) {
		tk1 = 0;
		ret = pct1812_user_access(ts, BANK(3), t_addr++, &val, sizeof(val), AREAD);
		if (!ret)
			tk1 |= val;
		ret = pct1812_user_access(ts, BANK(3), t_addr++, &val, sizeof(val), AREAD);
		if (!ret)
			tk1 |= (val << 8);

		tk2 = 0;
		ret = pct1812_user_access(ts, BANK(3), t_addr++, &val, sizeof(val), AREAD);
		if (!ret)
			tk2 |= val;
		ret = pct1812_user_access(ts, BANK(3), t_addr++, &val, sizeof(val), AREAD);
		if (!ret)
			tk2 |= (val << 8);

		if (!ret)
			pr_debug("Key[%i] THRESHOLD: TK1 = 0x%04x, TK2 = 0x%04x\n", i, tk1, tk2);
	}

	return ret;
}

static int inline pct1812_engmode(struct pct1812_data *ts, bool on)
{
	unsigned char value;
	int ret;

	value = RESET_CODE;
	ret = pct1812_user_access(ts, BANK(1), KEY1_REG, &value, sizeof(value), AWRITE);
	if (ret)
		return -EIO;

	value = on ? ENGINEERING_CODE : RESUME_CODE;
	ret = pct1812_user_access(ts, BANK(1), KEY2_REG, &value, sizeof(value), AWRITE);
	if (ret)
		return -EIO;

	pct1812_delay_ms(2);

	return 0;
}

static int pct1812_flash_exec(struct pct1812_data *ts,
		unsigned char cmd, unsigned char flash_cmd, int cnt)
{
	unsigned char lval;
	int repetition = 0;
	int ret, step = 0;

	lval = 0;
	ret = pct1812_user_access(ts, BANK(4), 0x2c, &lval, sizeof(lval), AWRITE);
	if (ret)
		goto error;
	step++;
	lval = flash_cmd;
	ret = pct1812_user_access(ts, BANK(4), 0x20, &lval, sizeof(lval), AWRITE);
	if (ret)
		goto error;
	step++;
	lval = cnt & 0xff;
	ret = pct1812_user_access(ts, BANK(4), 0x22, &lval, sizeof(lval), AWRITE);
	if (ret)
		goto error;
	step++;
	lval = (cnt >> 8) & 0xff;
	ret = pct1812_user_access(ts, BANK(4), 0x23, &lval, sizeof(lval), AWRITE);
	if (ret)
		goto error;
	step++;
do_again:
	lval = 0;
	ret = pct1812_user_access(ts, BANK(4), 0x2c, &lval, sizeof(lval), AREAD);
	if (ret)
		goto error;

	if ((lval & cmd) != 0) {
		repetition++;
		if (repetition%11)
			pr_debug("Still waiting ... %d\n", repetition);
		goto do_again;
	}

	return 0;

error:
	dev_err(ts->dev,
		"Error exec cmd 0x%02x, flash_cmd 0x%02x, cnt %d (stage %d, reps %d)\n",
		__func__, cmd, flash_cmd, cnt, step, repetition);
	return -EIO;
}

static int pct1812_flash_status(struct pct1812_data *ts, int idx, int vok)
{
	unsigned char status[2];
	int ret;

do_again:
	ret = pct1812_flash_exec(ts, 0x08, 0x05, 1);
	if (ret) {
		dev_err(ts->dev, "Error flash cmd\n", __func__);
		return -EIO;
	}

	ret = pct1812_user_access(ts, BANK(4), FLASH_STATUS_REG, status, sizeof(status), AREAD);
	if (ret) {
		dev_err(ts->dev, "Error reading flash status\n", __func__);
		return -EIO;
	}

	if (status[idx] != vok) {
		pct1812_delay_ms(1); // adjust delay if necessary
		goto do_again;
	}

	pr_debug("Flash status ([%d]=%d) OK\n", idx, vok);

	return 0;
}

static int pct1812_flash_chunk(struct pct1812_data *ts, unsigned int address)
{
	int ret, step = 0;
	unsigned char value;

	// Flash WriteEnable
	ret = pct1812_flash_exec(ts, 0x02, 0x09, 0);
	if (ret)
		goto error;
	step++;
	// check status
	ret = pct1812_flash_status(ts, 1, 1);
	if (ret)
		goto error;
	step++;
	value = (unsigned char)(address & 0xff);
	ret = pct1812_user_access(ts, BANK(4), FLASH_ADDR0_REG, &value, sizeof(value), AWRITE);
	if (ret)
		goto error;
	step++;
	value = (unsigned char)((address >> 8) & 0xff);
	ret = pct1812_user_access(ts, BANK(4), FLASH_ADDR1_REG, &value, sizeof(value), AWRITE);
	if (ret)
		goto error;
	step++;
	value = (unsigned char)((address >> 16) & 0xff);
	ret = pct1812_user_access(ts, BANK(4), FLASH_ADDR2_REG, &value, sizeof(value), AWRITE);
	if (ret)
		goto error;
	step++;
	ret = pct1812_flash_exec(ts, 0x81, 0x02, CHUNK_SZ);
	if (ret)
		goto error;
	step++;
	// wait for completion
	ret = pct1812_flash_status(ts, 0, 0);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(ts->dev, "Error erasing address 0x%06x (stage %d)\n",
			__func__, address, step);
	return -EIO;
}

static int pct1812_erase_sector(struct pct1812_data *ts, unsigned int address)
{
	int ret, step = 0;
	unsigned char value;

	pr_debug("Erasing sector at address 0x%06x\n", address);

	// Flash WriteEnable
	ret = pct1812_flash_exec(ts, 0x02, 0x09, 0);
	if (ret)
		goto error;
	step++;
	// check status
	ret = pct1812_flash_status(ts, 1, 1);
	if (ret)
		goto error;
	step++;
	value = (unsigned char)(address & 0xff);
	ret = pct1812_user_access(ts, BANK(4), FLASH_ADDR0_REG, &value, sizeof(value), AWRITE);
	if (ret)
		goto error;
	step++;
	value = (unsigned char)((address >> 8) & 0xff);
	ret = pct1812_user_access(ts, BANK(4), FLASH_ADDR1_REG, &value, sizeof(value), AWRITE);
	if (ret)
		goto error;
	step++;
	value = (unsigned char)((address >> 16) & 0xff);
	ret = pct1812_user_access(ts, BANK(4), FLASH_ADDR2_REG, &value, sizeof(value), AWRITE);
	if (ret)
		goto error;
	step++;
	ret = pct1812_flash_exec(ts, 0x02, 0x20, 3);
	if (ret)
		goto error;
	step++;
	// wait for completion
	ret = pct1812_flash_status(ts, 0, 0);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(ts->dev, "Error erasing address 0x%06x (stage %d)\n",
			__func__, address, step);
	return -EIO;
}

static int pct1812_flash_section(struct pct1812_data *ts, unsigned char *data,
		unsigned int size, unsigned int address)
{
	int m, ret;
	unsigned int num_of_sectors = size / PCT1812_SECTOR_SIZE;
	unsigned int target_num = size / CHUNK_SZ;
	unsigned char value;
	unsigned char *ptr = data;

	for (m = 0; m < num_of_sectors; m++) {
		ret = pct1812_erase_sector(ts, address + PCT1812_SECTOR_SIZE * m);
		if (ret)
			goto error;
		pr_debug("Erased sector %d of %d\n", m + 1, num_of_sectors);
	}

	for (m = 0; m < target_num; m++) {
		// Set SRAM select
		value = 0x08;
		ret = pct1812_user_access(ts, BANK(2), 0x09, &value, sizeof(value), AWRITE);
		if (ret)
			goto error;
		// Set SRAM NSC to 0
		value = 0x00;
		ret = pct1812_user_access(ts, BANK(2), 0x0a, &value, sizeof(value), AWRITE);
		if (ret)
			goto error;
		// Write data to SRAM port
		ret = pct1812_user_access(ts, BANK(2), 0x0b, ptr, CHUNK_SZ, AWRITE);
		if (ret)
			goto error;
		// Set SRAM NSC to 1
		value = 0x01;
		ret = pct1812_user_access(ts, BANK(2), 0x0a, &value, sizeof(value), AWRITE);
		if (ret)
			goto error;
		// advance data pointer
		ptr += CHUNK_SZ;
		pr_debug("Flashing chunk @0x%04x\n", address);
		// Program Flash from SRAM
		ret = pct1812_flash_chunk(ts, address);
		if (ret)
			goto error;
		address += CHUNK_SZ;
	}

	return 0;
error:
	dev_err(ts->dev, "Error flashing chunk %d\n", __func__, m);
	return -EIO;
}

/*static*/
int pct1812_fw_update(struct pct1812_data *ts, const char *fname)
{
	int ret;
	unsigned char value;
	unsigned char *fwdata_ptr, *fwparam_ptr;
	unsigned int fwdata_size, fwparam_size;
	const struct firmware *fw_entry = NULL;

	//__pm_stay_awake(&ts->wake_src);
	mutex_lock(&ts->cmdlock);
	pr_debug("Start of FW reflash process\n");
	//fwu_irq_enable(fwu, true);
	pr_debug("Requesting firmware %s\n", fname);
	ret = request_firmware(&fw_entry, fname, ts->dev);
	if (ret) {
		dev_err(ts->dev, "%s: Error loading firmware %s\n", __func__, fname);
		ret = -EINVAL;
		goto exit;
	}
	// FW file size check
	if (fw_entry->size != SUPPORTED_FW_FILE_SIZE) {
		dev_err(ts->dev, "%s: Firmware %s file size is WRONG!!!\n", __func__, fname);
		ret = -EINVAL;
		goto exit;
	}

	fwdata_size = FW_SECTION_SIZE;
	fwdata_ptr = (unsigned char *)fw_entry->data;
	fwparam_size = SUPPORTED_FW_FILE_SIZE - FW_SECTION_SIZE;
	fwparam_ptr = fwdata_ptr + FW_SECTION_SIZE;

	ret = pct1812_engmode(ts, true);
	if (ret) {
		dev_err(ts->dev, "%s: Error entering flash mode\n", __func__);
		goto exit;
	}

	value = POWERUP_CODE;
	ret = pct1812_user_access(ts, BANK(1), FLASH_PUP_REG, &value, sizeof(value), AWRITE);
	if (ret) {
		dev_err(ts->dev, "%s: Error powering flash controller\n", __func__);
		goto exit;
	}

	ret = pct1812_flash_section(ts, fwdata_ptr, fwdata_size, ADDRESS_FW);
	if (ret) {
		dev_err(ts->dev, "%s: Flash error!!!\n", __func__);
	} else {
		ret = pct1812_flash_section(ts, fwparam_ptr, fwparam_size, ADDRESS_PARAMS);
		if (ret)
			dev_err(ts->dev, "%s: Flash error!!!\n", __func__);
	}

exit:
	ret = pct1812_engmode(ts, false);
	if (ret) {
		dev_err(ts->dev, "%s: Error leaving flash mode\n", __func__);
	}
	mutex_unlock(&ts->cmdlock);
	//__pm_relax(&ts->wake_src);

	return ret;
}

static int pct1812_wait4ready(struct pct1812_data *ts, unsigned char reg, unsigned char vok)
{
	unsigned char status = 0;
	int retry, ret;

	for (retry = 0; retry < CNT_WAIT_RETRY; retry++) {
		ret = pct1812_i2c_read(ts, reg, &status, sizeof(status));
		if (ret || status == vok)
			break;
		pct1812_delay_ms(CMD_WAIT_DELAY);
	}
	if (retry == CNT_WAIT_RETRY)
		ret = -ETIME;
	else if (!ret) {
		pr_debug("success waiting for %d\n", vok);
	}

	return (status == vok) ? 0 : ret;
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

	pr_debug("vdd=%p, vio=%p\n", rvdd, rvio);

	return 0;
}

static int pct1812_pinctrl_state(struct pct1812_data *info, bool on)
{
	struct pinctrl_state *state_ptr;
	const char *state_name;
	int error = 0;

	if (!info->ts_pinctrl)
		return 0;

	if (on) {
		state_name = PINCTRL_STATE_ACTIVE;
		state_ptr =info->pinctrl_state_active;
	} else {
		state_name = PINCTRL_STATE_SUSPEND;
		state_ptr =info->pinctrl_state_suspend;
	}

	error = pinctrl_select_state(info->ts_pinctrl, state_ptr);
	if (error < 0)
		dev_err(info->dev, "%s: Failed to select %s\n",
			__func__, state_name);
	else
		pr_debug("set pinctrl state %s\n", state_name);

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
#if 0
static int pct1812_power_mode(struct pct1812_data *ts, enum pwr_modes mode)
{
	return 0;
}
#endif
static int pct1812_run_cmd(struct pct1812_data *ts, enum cmds command)
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
	default: return -EINVAL;
	}

run_once_again:
	pr_debug("command: %s\n", action);
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

	return ret;
}

static int pct1812_queued_resume(struct pct1812_data *ts)
{
	int ret;

	pr_debug("enter\n");

	if (atomic_cmpxchg(&ts->touch_stopped, 1, 0) == 0)
		return 0;

	pct1812_run_cmd(ts, CMD_RESUME);
	ret = pct1812_wait4ready(ts, BOOT_STATUS_REG, BOOT_COMPLETE);
	if (!ret) { // set active flag and irq
	}

	return ret;
}

static void pct1812_work(struct work_struct *work)
{
	struct pct1812_data *ts = container_of(work, struct pct1812_data, worker.work);
	unsigned char status = 0;
	static unsigned char prev;
	const char *action;
	int cmd, ret;

	while (kfifo_get(&ts->cmd_pipe, &cmd)) {
		//cmd = get_cmd();
		switch (cmd) {
		case CMD_WDOG:
				action = "WDOG";
				ret = pct1812_i2c_read(ts, FRAME_NUM_REG, &status, sizeof(status));
				if (ret || (prev == status)) {
					dev_warn(ts->dev, "%s: Possible lockup\n", __func__);
				}
					break;
		case CMD_RECOVER:
				action = "RECOVER";
				ret = pct1812_run_cmd(ts, CMD_RESET);
				if (ret) {
						dev_err(ts->dev, "%s: Failed to reset\n", __func__);
				} else {
					ret = pct1812_wait4ready(ts, BOOT_STATUS_REG, BOOT_COMPLETE);
					if (ret == -ETIME)
						dev_err(ts->dev, "%s: Failed to init\n", __func__);
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

	pr_debug("action: %s\n", action);

	pct1812_fifo_cmd_add(ts, CMD_WDOG, WDOG_INTERVAL);
}
#if 0
static int pct1812_suspend(struct device *dev)
{
	struct pct1812_data *ts = dev_get_drvdata(dev);
	int ret;

	pr_debug("enter\n");

	if (atomic_cmpxchg(&ts->touch_stopped, 0, 1) == 1)
		return 0;

	ret = pct1812_run_cmd(ts, CMD_SUSPEND);
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
	ret = pct1812_parse_dt(client);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to parse dt\n", __func__);
		goto error_allocate_mem;
	}

 	ts->client = client;
	ts->dev = &client->dev;
	ts->plat_data = pdata;

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
	if (ret == -ETIME) {
		dev_err(&client->dev, "%s: Failed to init\n", __func__);
		goto error_init;
	}

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

	pct1812_fifo_cmd_add(ts, CMD_WDOG, WDOG_INTERVAL);

	return 0;

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
