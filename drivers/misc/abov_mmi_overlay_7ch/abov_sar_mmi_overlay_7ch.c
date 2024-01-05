/*
 * file abov_sar.c
 * brief abov Driver for two channel SAP using
 *
 * Driver for the ABOV
 * Copyright (c) 2015-2016 ABOV Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DRIVER_NAME "abov_sar"
#define USE_SENSORS_CLASS

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/power_supply.h>

#include <linux/input/abov_sar_mmi_overlay_7ch.h> /* main struct, interrupt,init,pointers */

#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/async.h>
#include <linux/firmware.h>
#include <linux/version.h>
#include "base.h"

#define SLEEP(x)	mdelay(x)

#define C_I2C_FIFO_SIZE 8

#define I2C_MASK_FLAG	(0x00ff)

static u8 checksum_h;
static u8 checksum_h_bin;
static u8 checksum_l;
static u8 checksum_l_bin;
static u32 abov_channel_number = 0;
static u32 abov_flash_erase_time = 0;


#define IDLE 0
#define ACTIVE 1
#define S_PROX   1
#define S_BODY   2

#define LOG_TAG "<ABOV_LOG>"

#define LOG_INFO(fmt, args...)    pr_info(LOG_TAG "[INFO]" "<%s><%d>"fmt, __func__, __LINE__, ##args)
#define LOG_DBG(fmt, args...)	pr_debug(LOG_TAG "[DBG]" "<%s><%d>"fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...)   pr_err(LOG_TAG "[ERR]" "<%s><%d>"fmt, __func__, __LINE__, ##args)

static int last_val;
static int mEnabled;
static int programming_done;
pabovXX_t abov_sar_ptr;

/**
 * struct abov
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct abov {
	pbuttonInformation_t pbuttonInformation;
	pabov_platform_data_t hw; /* specific platform data settings */
} abov_t, *pabov_t;

static void ForcetoTouched(pabovXX_t this)
{
	pabov_t pDevice = NULL;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;
	struct _buttonInfo *pCurrentButton  = NULL;

	pDevice = this->pDevice;
	if (this && pDevice) {
		LOG_DBG("ForcetoTouched()\n");

		pCurrentButton = pDevice->pbuttonInformation->buttons;
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
			input_ch2 = pDevice->pbuttonInformation->input_ch2;
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
			input_ch2 = pDevice->pbuttonInformation->input_ch2;
			input_ch3 = pDevice->pbuttonInformation->input_ch3;
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
			input_ch2 = pDevice->pbuttonInformation->input_ch2;
			input_ch3 = pDevice->pbuttonInformation->input_ch3;
			input_ch4 = pDevice->pbuttonInformation->input_ch4;
		}

		pCurrentButton->state = ACTIVE;
		last_val = 1;
		if (mEnabled) {
			if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
				input_report_abs(input_ch0, ABS_DISTANCE, 1);
				input_sync(input_ch0);
				input_report_abs(input_ch1, ABS_DISTANCE, 1);
				input_sync(input_ch1);
			} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
				input_report_abs(input_ch0, ABS_DISTANCE, 1);
				input_sync(input_ch0);
				input_report_abs(input_ch1, ABS_DISTANCE, 1);
				input_sync(input_ch1);
				input_report_abs(input_ch2, ABS_DISTANCE, 1);
				input_sync(input_ch2);
			} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
				input_report_abs(input_ch0, ABS_DISTANCE, 1);
				input_sync(input_ch0);
				input_report_abs(input_ch1, ABS_DISTANCE, 1);
				input_sync(input_ch1);
				input_report_abs(input_ch2, ABS_DISTANCE, 1);
				input_sync(input_ch2);
				input_report_abs(input_ch3, ABS_DISTANCE, 1);
				input_sync(input_ch3);
			} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
				input_report_abs(input_ch0, ABS_DISTANCE, 1);
				input_sync(input_ch0);
				input_report_abs(input_ch1, ABS_DISTANCE, 1);
				input_sync(input_ch1);
				input_report_abs(input_ch2, ABS_DISTANCE, 1);
				input_sync(input_ch2);
				input_report_abs(input_ch3, ABS_DISTANCE, 1);
				input_sync(input_ch3);
				input_report_abs(input_ch4, ABS_DISTANCE, 1);
				input_sync(input_ch4);
			}
		}
		LOG_DBG("Leaving ForcetoTouched()\n");
	}
}

/**
 * fn static int write_register(pabovXX_t this, u8 address, u8 value)
 * brief Sends a write register to the device
 * param this Pointer to main parent struct
 * param address 8-bit register address
 * param value   8-bit register value to write to address
 * return Value from i2c_master_send
 */
static int write_register(pabovXX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;

		returnValue = i2c_master_send(i2c, buffer, 2);
		LOG_DBG("write_register Addr: 0x%x Val: 0x%x Return: %d\n",
				address, value, returnValue);
	}
	if (returnValue < 0) {
		ForcetoTouched(this);
		LOG_ERR("Write_register-ForcetoTouched()\n");
	}
	return returnValue;
}

/**
 * fn static int read_register(pabovXX_t this, u8 address, u8 *value)
 * brief Reads a register's value from the device
 * param this Pointer to main parent struct
 * param address 8-Bit address to read from
 * param value Pointer to 8-bit value to save register value to
 * return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
static int read_register(pabovXX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c, address);
		LOG_DBG("read_register Addr: 0x%x Return: 0x%x\n",
				address, returnValue);
	}

	if (returnValue >= 0) {
		*value = returnValue;
		return 0;
	} else {
			ForcetoTouched(this);
			LOG_ERR("read_register-ForcetoTouched()\n");
			return returnValue;
	}
	return returnValue;
}


/**
 * brief Perform a manual offset calibration
 * param this Pointer to main parent struct
 * return Value return value from the write register
 */
static int manual_offset_calibration(pabovXX_t this)
{
	s32 returnValue = 0;

	returnValue = write_register(this, ABOV_RECALI_REG, 0x01);
	return returnValue;
}

/**
 * brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	u16 returnValue = 0;
	pabovXX_t this = dev_get_drvdata(dev);

	LOG_DBG("Reading IRQSTAT_REG\n");
	read_register(this, ABOV_IRQSTAT_REG, &reg_value);
	returnValue = reg_value;
    if (abov_channel_number == 5) {
		read_register(this, ABOV_IRQSTAT1_REG, &reg_value);
		returnValue |= (u16)reg_value << 8;
    }
	return scnprintf(buf, PAGE_SIZE, "%d\n", returnValue);
}

/* brief sysfs store function for manual calibration */
static ssize_t manual_offset_calibration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	pabovXX_t this = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val) {
		LOG_DBG("Performing manual_offset_calibration()\n");
		manual_offset_calibration(this);
	}
	return count;
}

static DEVICE_ATTR(calibrate, 0644, manual_offset_calibration_show,
		manual_offset_calibration_store);
static struct attribute *abov_attributes[] = {
	&dev_attr_calibrate.attr,
	NULL,
};
static struct attribute_group abov_attr_group = {
	.attrs = abov_attributes,
};


/**
 * brief  Initialize I2C config from platform data
 * param this Pointer to main parent struct
 *
 */
static void hw_init(pabovXX_t this)
{
	pabov_t pDevice = 0;
	pabov_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */
	LOG_DBG("Inside hw_init().\n");
	pDevice = this->pDevice;
	pdata = pDevice->hw;
	if (this && pDevice && pdata) {
		while (i < pdata->i2c_reg_num) {
			/* Write all registers/values contained in i2c_reg */
			write_register(this, pdata->pi2c_reg[i].reg,
					pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		LOG_ERR("ERROR! platform data 0x%p\n", pDevice->hw);
		/* Force to touched if error */
		ForcetoTouched(this);
	}
	LOG_DBG("Exiting hw_init().\n");
}

/**
 * fn static int initialize(pabovXX_t this)
 * brief Performs all initialization needed to configure the device
 * param this Pointer to main parent struct
 * return Last used command's return value (negative if error)
 */
static int initialize(pabovXX_t this)
{
	if (this) {
		LOG_DBG("Inside initialize().\n");
		/* prepare reset by disabling any irq handling */
		if(this->irq_disabled == 0) {
		   disable_irq(this->irq);
		   this->irq_disabled = 1;
		}
		hw_init(this);
		msleep(300); /* make sure everything is running */
		/* re-enable interrupt handling */
		if(this->irq_disabled == 1) {
		   enable_irq(this->irq);
		   this->irq_disabled = 0;
		}
		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		LOG_DBG("Exiting initialize().\n");
		programming_done = ACTIVE;
		return 0;
	}
	programming_done = IDLE;
	return -ENOMEM;
}

/**
 * brief Handle what to do when a touch occurs
 * param this Pointer to main parent struct
 */
static void touchProcess(pabovXX_t this)
{
	int counter = 0;
	u8 i = 0;
	u16 capsense_state = 0;
	int numberOfButtons = 0;
	pabov_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;
	struct _buttonInfo *pCurrentButton  = NULL;
	struct abov_platform_data *board;

	pDevice = this->pDevice;
	board = this->board;
	if (this && pDevice) {
		LOG_DBG("Inside touchProcess()\n");
		read_register(this, ABOV_IRQSTAT_REG, &i);
        capsense_state = i;
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			read_register(this, ABOV_IRQSTAT1_REG, &i);
			capsense_state |= (u16)i << 8;
		}

		buttons = pDevice->pbuttonInformation->buttons;
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
			input_ch2 = pDevice->pbuttonInformation->input_ch2;
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
			input_ch2 = pDevice->pbuttonInformation->input_ch2;
			input_ch3 = pDevice->pbuttonInformation->input_ch3;
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			input_ch0 = pDevice->pbuttonInformation->input_ch0;
			input_ch1 = pDevice->pbuttonInformation->input_ch1;
			input_ch2 = pDevice->pbuttonInformation->input_ch2;
			input_ch3 = pDevice->pbuttonInformation->input_ch3;
			input_ch4 = pDevice->pbuttonInformation->input_ch4;
		}
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			if (unlikely((buttons == NULL) || (input_ch0 == NULL) || (input_ch1 == NULL))) {
				LOG_ERR("ERROR!! buttons or input NULL!!!\n");
				return;
			}
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			if (unlikely((buttons == NULL) || (input_ch0 == NULL) || (input_ch1 == NULL) || (input_ch2 == NULL))) {
				LOG_ERR("ERROR!! buttons or input NULL!!!\n");
				return;
			}
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			if (unlikely((buttons == NULL) || (input_ch0 == NULL) || (input_ch1 == NULL) || (input_ch2 == NULL) || (input_ch3 == NULL))) {
				LOG_ERR("ERROR!! buttons or input NULL!!!\n");
				return;
			}
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			if (unlikely((buttons == NULL) || (input_ch0 == NULL) || (input_ch1 == NULL) || (input_ch2 == NULL) || (input_ch3 == NULL) || (input_ch4 == NULL))) {
				LOG_ERR("ERROR!! buttons or input NULL!!!\n");
				return;
			}
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton == NULL) {
				LOG_ERR("ERR!current button index: %d NULL!\n",
						counter);
				return; /* ERRORR!!!! */
			}
			switch (pCurrentButton->state) {
			case IDLE: /* Button is being in far state! */
				if ((capsense_state & pCurrentButton->mask) == pCurrentButton->mask) {
					LOG_DBG("CS %d State=BODY.\n",
							counter);
					if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 2);
							input_sync(input_ch2);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 2);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 2);
							input_sync(input_ch3);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 2);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 2);
							input_sync(input_ch3);
						} else if (board->cap_channel_ch4 == counter) {
							input_report_abs(input_ch4, ABS_DISTANCE, 2);
							input_sync(input_ch4);
						}
					}
					pCurrentButton->state = S_BODY;
					last_val = 2;
				} else if ((capsense_state & pCurrentButton->mask) == (pCurrentButton->mask & board->cap_button_mask )) {
					LOG_DBG("CS %d State=PROX.\n",
							counter);
					if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 1);
							input_sync(input_ch2);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 1);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 1);
							input_sync(input_ch3);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 1);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 1);
							input_sync(input_ch3);
						} else if (board->cap_channel_ch4 == counter) {
							input_report_abs(input_ch4, ABS_DISTANCE, 1);
							input_sync(input_ch4);
						}
					}
					pCurrentButton->state = S_PROX;
					last_val = 0;
				} else {
					LOG_DBG("CS %d still in IDLE State.\n",
							counter);
				}
				break;
			case S_PROX: /* Button is being in proximity! */
				if ((capsense_state & pCurrentButton->mask) == pCurrentButton->mask) {
					LOG_DBG("CS %d State=BODY.\n",
							counter);
					if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 2);
							input_sync(input_ch2);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 2);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 2);
							input_sync(input_ch3);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 2);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 2);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 2);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 2);
							input_sync(input_ch3);
						} else if (board->cap_channel_ch4 == counter) {
							input_report_abs(input_ch4, ABS_DISTANCE, 2);
							input_sync(input_ch4);
						}
					}
					pCurrentButton->state = S_BODY;
					last_val = 2;
				} else if ((capsense_state & pCurrentButton->mask) == (pCurrentButton->mask & board->cap_button_mask)) {
					LOG_DBG("CS %d still in PROX State.\n",
							counter);
				} else{
					LOG_DBG("CS %d State=IDLE.\n",
							counter);
					if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 0);
							input_sync(input_ch2);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 0);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 0);
							input_sync(input_ch3);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 0);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 0);
							input_sync(input_ch3);
						} else if (board->cap_channel_ch4 == counter) {
							input_report_abs(input_ch4, ABS_DISTANCE, 0);
							input_sync(input_ch4);
						}
					}
					pCurrentButton->state = IDLE;
					last_val = 0;
				}
				break;
			case S_BODY: /* Button is being in 0mm! */
				if ((capsense_state & pCurrentButton->mask) == pCurrentButton->mask) {
					LOG_DBG("CS %d still in BODY State.\n",
							counter);
				} else if ((capsense_state & pCurrentButton->mask) == (pCurrentButton->mask & board->cap_button_mask)) {
					LOG_DBG("CS %d State=PROX.\n",
							counter);
					if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 1);
							input_sync(input_ch2);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 1);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 1);
							input_sync(input_ch3);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 1);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 1);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 1);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 1);
							input_sync(input_ch3);
						} else if (board->cap_channel_ch4 == counter) {
							input_report_abs(input_ch4, ABS_DISTANCE, 1);
							input_sync(input_ch4);
						}
					}
					pCurrentButton->state = S_PROX;
					last_val = 1;
				} else{
					LOG_DBG("CS %d State=IDLE.\n",
							counter);
					if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE){
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 0);
							input_sync(input_ch2);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 0);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 0);
							input_sync(input_ch3);
						}
					} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
						if (board->cap_channel_ch0 == counter) {
							input_report_abs(input_ch0, ABS_DISTANCE, 0);
							input_sync(input_ch0);
						} else if (board->cap_channel_ch1 == counter) {
							input_report_abs(input_ch1, ABS_DISTANCE, 0);
							input_sync(input_ch1);
						} else if (board->cap_channel_ch2 == counter) {
							input_report_abs(input_ch2, ABS_DISTANCE, 0);
							input_sync(input_ch2);
						} else if (board->cap_channel_ch3 == counter) {
							input_report_abs(input_ch3, ABS_DISTANCE, 0);
							input_sync(input_ch3);
						} else if (board->cap_channel_ch4 == counter) {
							input_report_abs(input_ch4, ABS_DISTANCE, 0);
							input_sync(input_ch4);
						}
					}
					pCurrentButton->state = IDLE;
					last_val = 0;
				}
				break;
			default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
				break;
			};
		}
		LOG_DBG("Leaving touchProcess()\n");
	}
}

static int abov_get_nirq_state(unsigned irq_gpio)
{
	if (irq_gpio) {
		return !gpio_get_value(irq_gpio);
	} else {
		LOG_ERR("abov irq_gpio is not set.");
		return -EINVAL;
	}
}

static struct _totalButtonInformation smtcButtonInformation = {
	.buttons = psmtcButtons,
	.buttonSize = ARRAY_SIZE(psmtcButtons),
};

/**
 *fn static void abov_reg_setup_init(struct i2c_client *client)
 *brief read reg val form dts
 *      reg_array_len for regs needed change num
 *      data_array_val's format <reg val ...>
 */
static void abov_reg_setup_init(struct i2c_client *client)
{
	u32 data_array_len = 0;
	u32 *data_array;
	int ret, i, j;
	struct device_node *np = client->dev.of_node;

	ret = of_property_read_u32(np, "reg_array_len", &data_array_len);
	if (ret < 0) {
		LOG_ERR("data_array_len read error");
		return;
	}
	data_array = kmalloc(data_array_len * 2 * sizeof(u32), GFP_KERNEL);
	ret = of_property_read_u32_array(np, "reg_array_val",
			data_array,
			data_array_len*2);
	if (ret < 0) {
		LOG_ERR("data_array_val read error");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(abov_i2c_reg_setup); i++) {
		for (j = 0; j < data_array_len*2; j += 2) {
			if (data_array[j] == abov_i2c_reg_setup[i].reg) {
				abov_i2c_reg_setup[i].val = data_array[j+1];
				LOG_DBG("read dtsi 0x%02x:0x%02x set reg\n",
					data_array[j], data_array[j+1]);
			}
		}
	}
	kfree(data_array);
}

static void abov_platform_data_of_init(struct i2c_client *client,
		pabov_platform_data_t pplatData)
{
	struct device_node *np = client->dev.of_node;
	u32 cap_channel_ch0,cap_channel_ch1,cap_channel_ch2,cap_channel_ch3,cap_channel_ch4;
	int ret;

	client->irq = of_get_gpio(np, 0);
	pplatData->irq_gpio = client->irq;

	ret = of_property_read_u32(np, "cap,channel_number", &abov_channel_number);
	if (ret < 0) {
		LOG_ERR("get cap,channel number node error,set default value!\n");
		abov_channel_number = ABOV_CHANNEL_NUMBER_TWO;
	}
	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		ret = of_property_read_u32(np, "cap,use_channel_ch0", &cap_channel_ch0);
		ret = of_property_read_string(np, "ch0_name", &pplatData->cap_ch0_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch1", &cap_channel_ch1);
		ret = of_property_read_string(np, "ch1_name", &pplatData->cap_ch1_name);
		pplatData->cap_channel_ch0 = (int)cap_channel_ch0;
	    pplatData->cap_channel_ch1 = (int)cap_channel_ch1;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		ret = of_property_read_u32(np, "cap,use_channel_ch0", &cap_channel_ch0);
		ret = of_property_read_string(np, "ch0_name", &pplatData->cap_ch0_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch1", &cap_channel_ch1);
		ret = of_property_read_string(np, "ch1_name", &pplatData->cap_ch1_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch2", &cap_channel_ch2);
		ret = of_property_read_string(np, "ch2_name", &pplatData->cap_ch2_name);
		pplatData->cap_channel_ch0 = (int)cap_channel_ch0;
		pplatData->cap_channel_ch1 = (int)cap_channel_ch1;
		pplatData->cap_channel_ch2 = (int)cap_channel_ch2;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		ret = of_property_read_u32(np, "cap,use_channel_ch0", &cap_channel_ch0);
		ret = of_property_read_string(np, "ch0_name", &pplatData->cap_ch0_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch1", &cap_channel_ch1);
		ret = of_property_read_string(np, "ch1_name", &pplatData->cap_ch1_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch2", &cap_channel_ch2);
		ret = of_property_read_string(np, "ch2_name", &pplatData->cap_ch2_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch3", &cap_channel_ch3);
		ret = of_property_read_string(np, "ch3_name", &pplatData->cap_ch3_name);
		pplatData->cap_channel_ch0 = (int)cap_channel_ch0;
		pplatData->cap_channel_ch1 = (int)cap_channel_ch1;
		pplatData->cap_channel_ch2 = (int)cap_channel_ch2;
		pplatData->cap_channel_ch3 = (int)cap_channel_ch3;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		ret = of_property_read_u32(np, "cap,use_channel_ch0", &cap_channel_ch0);
		ret = of_property_read_string(np, "ch0_name", &pplatData->cap_ch0_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch1", &cap_channel_ch1);
		ret = of_property_read_string(np, "ch1_name", &pplatData->cap_ch1_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch2", &cap_channel_ch2);
		ret = of_property_read_string(np, "ch2_name", &pplatData->cap_ch2_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch3", &cap_channel_ch3);
		ret = of_property_read_string(np, "ch3_name", &pplatData->cap_ch3_name);
		ret = of_property_read_u32(np, "cap,use_channel_ch4", &cap_channel_ch4);
		ret = of_property_read_string(np, "ch4_name", &pplatData->cap_ch4_name);
		pplatData->cap_channel_ch0 = (int)cap_channel_ch0;
		pplatData->cap_channel_ch1 = (int)cap_channel_ch1;
		pplatData->cap_channel_ch2 = (int)cap_channel_ch2;
		pplatData->cap_channel_ch3 = (int)cap_channel_ch3;
		pplatData->cap_channel_ch4 = (int)cap_channel_ch4;
	}

	pplatData->get_is_nirq_low = abov_get_nirq_state;
	pplatData->init_platform_hw = NULL;
	/*  pointer to an exit function. Here in case needed in the future */
	/*
	 *.exit_platform_hw = abov_exit_ts,
	 */
	pplatData->exit_platform_hw = NULL;
	abov_reg_setup_init(client);
	pplatData->pi2c_reg = abov_i2c_reg_setup;
	pplatData->i2c_reg_num = ARRAY_SIZE(abov_i2c_reg_setup);

	pplatData->pbuttonInformation = &smtcButtonInformation;

	ret = of_property_read_string(np, "label", &pplatData->fw_name);
	if (ret < 0) {
		LOG_ERR("firmware name read error!\n");
		return;
	}

	ret = of_property_read_u32(np, "cap,fw_mode_ret", &pplatData->fw_mode_ret);
	if (ret < 0) {
		LOG_ERR("get cap,fw_mode_ret error,set default value.\n");
		pplatData->fw_mode_ret = ABOV_T376_FW_MODE;
	}
	ret = of_property_read_u32(np, "button_mask", &pplatData->cap_button_mask);
	if (ret < 0) {
		LOG_ERR("get button_mask,set default value.\n");
		pplatData->cap_button_mask = ABOV_CAP_BUTTON_5CH_MASK;
	}

	ret = of_property_read_u32(np, "cap,flash_erase_time", &abov_flash_erase_time);
	if (ret < 0) {
		LOG_ERR("cap,flash_erase_time error!,set default value.\n");
		abov_flash_erase_time = ABOV_T376_FLASH_ERASE_TIME;
	}

}

static ssize_t reset_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	return snprintf(buf, 8, "%d\n", programming_done);
}

static ssize_t reset_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	pabovXX_t this = abov_sar_ptr;
	pabov_t pDevice = NULL;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;

	pDevice = this->pDevice;
	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
		input_ch4 = pDevice->pbuttonInformation->input_ch4;
	}

	if (!count || (this == NULL))
		return -EINVAL;

	if (!strncmp(buf, "reset", 5) || !strncmp(buf, "1", 1))
		write_register(this, ABOV_SOFTRESET_REG, 0x10);

   	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
   	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
   		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
		input_report_abs(input_ch2, ABS_DISTANCE, 0);
		input_sync(input_ch2);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
		input_report_abs(input_ch2, ABS_DISTANCE, 0);
		input_sync(input_ch2);
		input_report_abs(input_ch3, ABS_DISTANCE, 0);
		input_sync(input_ch3);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
		input_report_abs(input_ch2, ABS_DISTANCE, 0);
		input_sync(input_ch2);
		input_report_abs(input_ch3, ABS_DISTANCE, 0);
		input_sync(input_ch3);
		input_report_abs(input_ch4, ABS_DISTANCE, 0);
		input_sync(input_ch4);
	}

	return count;
}

static CLASS_ATTR_RW(reset);

static ssize_t int_state_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	pabovXX_t this = abov_sar_ptr;
	LOG_DBG("Reading INT line state\n");
	return snprintf(buf, 8, "%d\n", this->int_state);
}
static CLASS_ATTR_RO(int_state);

static ssize_t fw_download_status_show(struct class *class,
                struct class_attribute *attr,
                char *buf)
{
	pabovXX_t this = abov_sar_ptr;
	LOG_DBG("Reading fw download state\n");
        return snprintf(buf, 8, "%d", this->fw_dl_status);
}
static CLASS_ATTR_RO(fw_download_status);

static ssize_t enable_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	return snprintf(buf, 8, "%d\n", mEnabled);
}

static ssize_t enable_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	pabovXX_t this = abov_sar_ptr;
	pabov_t pDevice = NULL;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;

	pDevice = this->pDevice;
	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
		input_ch4 = pDevice->pbuttonInformation->input_ch4;
	}

	if (!count || (this == NULL))
		return -EINVAL;

	if (!strncmp(buf, "1", 1)) {
		LOG_INFO("enable cap sensor\n");
		initialize(this);

		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			input_report_abs(input_ch0, ABS_DISTANCE, 0);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, 0);
			input_sync(input_ch1);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			input_report_abs(input_ch0, ABS_DISTANCE, 0);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, 0);
			input_sync(input_ch1);
			input_report_abs(input_ch2, ABS_DISTANCE, 0);
			input_sync(input_ch2);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			input_report_abs(input_ch0, ABS_DISTANCE, 0);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, 0);
			input_sync(input_ch1);
			input_report_abs(input_ch2, ABS_DISTANCE, 0);
			input_sync(input_ch2);
			input_report_abs(input_ch3, ABS_DISTANCE, 0);
			input_sync(input_ch3);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			input_report_abs(input_ch0, ABS_DISTANCE, 0);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, 0);
			input_sync(input_ch1);
			input_report_abs(input_ch2, ABS_DISTANCE, 0);
			input_sync(input_ch2);
			input_report_abs(input_ch3, ABS_DISTANCE, 0);
			input_sync(input_ch3);
			input_report_abs(input_ch4, ABS_DISTANCE, 0);
			input_sync(input_ch4);
		}
		mEnabled = 1;
	} else if (!strncmp(buf, "0", 1)) {
		LOG_INFO("disable cap sensor\n");

		write_register(this, ABOV_CTRL_MODE_REG, ABOV_CTRL_MODE_STOP);

		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			input_report_abs(input_ch0, ABS_DISTANCE, -1);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, -1);
			input_sync(input_ch1);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			input_report_abs(input_ch0, ABS_DISTANCE, -1);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, -1);
			input_sync(input_ch1);
			input_report_abs(input_ch2, ABS_DISTANCE, -1);
			input_sync(input_ch2);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			input_report_abs(input_ch0, ABS_DISTANCE, -1);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, -1);
			input_sync(input_ch1);
			input_report_abs(input_ch2, ABS_DISTANCE, -1);
			input_sync(input_ch2);
			input_report_abs(input_ch3, ABS_DISTANCE, -1);
			input_sync(input_ch3);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			input_report_abs(input_ch0, ABS_DISTANCE, -1);
			input_sync(input_ch0);
			input_report_abs(input_ch1, ABS_DISTANCE, -1);
			input_sync(input_ch1);
			input_report_abs(input_ch2, ABS_DISTANCE, -1);
			input_sync(input_ch2);
			input_report_abs(input_ch3, ABS_DISTANCE, -1);
			input_sync(input_ch3);
			input_report_abs(input_ch4, ABS_DISTANCE, -1);
			input_sync(input_ch4);
		}
		mEnabled = 0;
	} else {
		LOG_ERR("unknown enable symbol\n");
	}

	return count;
}

#ifdef USE_SENSORS_CLASS
static int capsensor_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	pabovXX_t this = abov_sar_ptr;
	pabov_t pDevice = NULL;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;

	pDevice = this->pDevice;
	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
		input_ch4 = pDevice->pbuttonInformation->input_ch4;
	}

	if (enable == 1) {
		LOG_INFO("enable cap sensor: %s\n",sensors_cdev->name);
		if(mEnabled == 0){
			initialize(this);
			mEnabled = 1;
		}
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, 0);
				input_sync(input_ch0);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, 0);
				input_sync(input_ch1);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH1;
			}
		} else if(abov_channel_number == ABOV_CHANNEL_NUMBER_THREE){
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, 0);
				input_sync(input_ch0);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, 0);
				input_sync(input_ch1);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH1;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch2_name)){
				input_report_abs(input_ch2, ABS_DISTANCE, 0);
				input_sync(input_ch2);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH2;
			}
		} else if(abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, 0);
				input_sync(input_ch0);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, 0);
				input_sync(input_ch1);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH1;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch2_name)){
				input_report_abs(input_ch2, ABS_DISTANCE, 0);
				input_sync(input_ch2);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH2;
			} else if(!strcmp(sensors_cdev->name, this->board->cap_ch3_name)){
				input_report_abs(input_ch3, ABS_DISTANCE, 0);
				input_sync(input_ch3);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH3;
			}
		} else if(abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, 0);
				input_sync(input_ch0);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, 0);
				input_sync(input_ch1);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH1;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch2_name)){
				input_report_abs(input_ch2, ABS_DISTANCE, 0);
				input_sync(input_ch2);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH2;
			} else if(!strcmp(sensors_cdev->name, this->board->cap_ch3_name)){
				input_report_abs(input_ch3, ABS_DISTANCE, 0);
				input_sync(input_ch3);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH3;
			} else if(!strcmp(sensors_cdev->name, this->board->cap_ch4_name)){
				input_report_abs(input_ch4, ABS_DISTANCE, 0);
				input_sync(input_ch4);
				this->enable_flag |= CAPSENSOR_ENABLE_FLAG_CH4;
			}
		}
		LOG_INFO(" cap sensor enable 0x%02x \n", this->enable_flag);
	} else if (enable == 0) {
		LOG_INFO("disable cap sensor: %s\n",sensors_cdev->name);
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, -1);
				input_sync(input_ch0);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, -1);
				input_sync(input_ch1);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH1;
			}
		} else if(abov_channel_number == ABOV_CHANNEL_NUMBER_THREE){
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, -1);
				input_sync(input_ch0);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, -1);
				input_sync(input_ch1);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH1;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch2_name)){
				input_report_abs(input_ch2, ABS_DISTANCE, -1);
				input_sync(input_ch2);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH2;
			}
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR){
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, -1);
				input_sync(input_ch0);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, -1);
				input_sync(input_ch1);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH1;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch2_name)){
				input_report_abs(input_ch2, ABS_DISTANCE, -1);
				input_sync(input_ch2);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH2;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch3_name)){
				input_report_abs(input_ch3, ABS_DISTANCE, -1);
				input_sync(input_ch3);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH3;
			}
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE){
			if(!strcmp(sensors_cdev->name, this->board->cap_ch0_name)){
				input_report_abs(input_ch0, ABS_DISTANCE, -1);
				input_sync(input_ch0);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH0;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch1_name)){
				input_report_abs(input_ch1, ABS_DISTANCE, -1);
				input_sync(input_ch1);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH1;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch2_name)){
				input_report_abs(input_ch2, ABS_DISTANCE, -1);
				input_sync(input_ch2);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH2;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch3_name)){
				input_report_abs(input_ch3, ABS_DISTANCE, -1);
				input_sync(input_ch3);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH3;
			}else if(!strcmp(sensors_cdev->name, this->board->cap_ch4_name)){
				input_report_abs(input_ch4, ABS_DISTANCE, -1);
				input_sync(input_ch4);
				this->enable_flag &= ~CAPSENSOR_ENABLE_FLAG_CH4;
			}
		}
		LOG_INFO(" cap sensor disable 0x%02x \n", this->enable_flag);
		if(!(this->enable_flag & 0x1F)){
			mEnabled = 0;
			write_register(this, ABOV_CTRL_MODE_REG, ABOV_CTRL_MODE_STOP);
			LOG_INFO("all cap sensor disable,change to STOP mode.\n");
		}
	} else {
		LOG_ERR("unknown enable symbol\n");
	}

	return 0;
}
#endif

static CLASS_ATTR_RW(enable);

static ssize_t reg_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	u8 reg_value = 0, i;
	pabovXX_t this = abov_sar_ptr;
	char *p = buf;

	if (this->read_flag) {
		this->read_flag = 0;
		read_register(this, this->read_reg, &reg_value);
		p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n", this->read_reg, reg_value);
		return (p-buf);
	}

	for (i = 0; i < 0x56; i++) {
		read_register(this, i, &reg_value);
		p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n",
				i, reg_value);
	}

	for (i = 0x80; i < 0xB0; i++) {
		read_register(this, i, &reg_value);
		p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n",
				i, reg_value);
	}

	return (p-buf);
}

static ssize_t reg_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	pabovXX_t this = abov_sar_ptr;
	unsigned int val, reg, opt;
    if (sscanf(buf, "%x,%x,%x", &reg, &val, &opt) == 3) {
		LOG_DBG("%s, read reg = 0x%02x\n", __func__, *(u8 *)&reg);
		this->read_reg = *((u8 *)&reg);
		this->read_flag = 1;
	} else if (sscanf(buf, "%x,%x", &reg, &val) == 2) {
		LOG_DBG("%s,reg = 0x%02x, val = 0x%02x\n",
				__func__, *(u8 *)&reg, *(u8 *)&val);
		write_register(this, *((u8 *)&reg), *((u8 *)&val));
	}

	return count;
}


static CLASS_ATTR_RW(reg);

static struct class capsense_class = {
	.name			= "capsense",
	.owner			= THIS_MODULE,
};

static void ps_notify_callback_work(struct work_struct *work)
{
	pabovXX_t this = container_of(work, abovXX_t, ps_notify_work);
	pabov_t pDevice = NULL;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;
	int ret = 0;

	pDevice = this->pDevice;
	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_ch0 = pDevice->pbuttonInformation->input_ch0;
		input_ch1 = pDevice->pbuttonInformation->input_ch1;
		input_ch2 = pDevice->pbuttonInformation->input_ch2;
		input_ch3 = pDevice->pbuttonInformation->input_ch3;
		input_ch4 = pDevice->pbuttonInformation->input_ch4;
	}

	LOG_INFO("Usb insert,going to force calibrate\n");
	ret = write_register(this, ABOV_RECALI_REG, 0x01);
	if (ret < 0)
		LOG_ERR(" Usb insert,calibrate cap sensor failed\n");

	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
		input_report_abs(input_ch2, ABS_DISTANCE, 0);
		input_sync(input_ch2);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
		input_report_abs(input_ch2, ABS_DISTANCE, 0);
		input_sync(input_ch2);
		input_report_abs(input_ch3, ABS_DISTANCE, 0);
		input_sync(input_ch3);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_report_abs(input_ch0, ABS_DISTANCE, 0);
		input_sync(input_ch0);
		input_report_abs(input_ch1, ABS_DISTANCE, 0);
		input_sync(input_ch1);
		input_report_abs(input_ch2, ABS_DISTANCE, 0);
		input_sync(input_ch2);
		input_report_abs(input_ch3, ABS_DISTANCE, 0);
		input_sync(input_ch3);
		input_report_abs(input_ch4, ABS_DISTANCE, 0);
		input_sync(input_ch4);
	}
}

static int ps_get_state(struct power_supply *psy, bool *present)
{
	union power_supply_propval pval = { 0 };
	int retval;

	retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
			&pval);
	if (retval) {
		LOG_ERR("%s psy get property failed\n", psy->desc->name);
		return retval;
	}
	*present = (pval.intval) ? true : false;
	LOG_INFO("%s is %s\n", psy->desc->name,
			(*present) ? "present" : "not present");
	return 0;
}

static int ps_notify_callback(struct notifier_block *self,
		unsigned long event, void *p)
{
	pabovXX_t this = container_of(self, abovXX_t, ps_notif);
	struct power_supply *psy = p;
	bool present;
	int retval;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
	if (event == PSY_EVENT_PROP_CHANGED
#else
	if ((event == PSY_EVENT_PROP_ADDED || event == PSY_EVENT_PROP_CHANGED)
#endif
			&& psy && psy->desc->get_property && psy->desc->name &&
			!strncmp(psy->desc->name, "usb", sizeof("usb"))){
		LOG_INFO("ps notification: event = %lu\n", event);
		retval = ps_get_state(psy, &present);
		if (retval) {
			LOG_ERR("psy get property failed\n");
			return retval;
		}

		if (event == PSY_EVENT_PROP_CHANGED) {
			if (this->ps_is_present == present) {
				LOG_INFO("ps present state not change\n");
				return 0;
			}
		}
		this->ps_is_present = present;
		schedule_work(&this->ps_notify_work);
	}

	return 0;
}


static int _i2c_adapter_block_write(struct i2c_client *client, u8 *data, u8 len)
{
	u8 buffer[C_I2C_FIFO_SIZE];
	u8 left = len;
	u8 offset = 0;
	u8 retry = 0;

	struct i2c_msg msg = {
		.addr = client->addr & I2C_MASK_FLAG,
		.flags = 0,
		.buf = buffer,
	};

	if (data == NULL || len < 1) {
		LOG_ERR("Invalid : data is null or len=%d\n", len);
		return -EINVAL;
	}

	while (left > 0) {
		retry = 0;
		if (left >= C_I2C_FIFO_SIZE) {
			msg.buf = &data[offset];
			msg.len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE;
			offset += C_I2C_FIFO_SIZE;
		} else {
			msg.buf = &data[offset];
			msg.len = left;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;
			if (retry > 10) {
				LOG_ERR("OUT : fail - addr:%#x len:%d \n", client->addr, msg.len);
				return -EIO;
			}
		}
	}
	return 0;
}

static int i2c_adapter_block_write_nodatalog(struct i2c_client *client, u8 *data, u8 len)
{
	return _i2c_adapter_block_write(client, data, len);
}

static int abov_tk_check_busy(struct i2c_client *client)
{
	int ret, count = 0;
	unsigned char val = 0x00;

	do {
		ret = i2c_master_recv(client, &val, sizeof(val));
		if (val & 0x01) {
			count++;
			if (count > 1000) {
				LOG_INFO("%s: val = 0x%x\r\n", __func__, val);
				return ret;
			}
		} else {
			break;
		}
	} while (1);

	return ret;
}

static int abov_tk_fw_write(struct i2c_client *client, unsigned char *addrH,
						unsigned char *addrL, unsigned char *val)
{
	int length = 36, ret = 0;
	unsigned char buf[40] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0x7A;
	memcpy(&buf[2], addrH, 1);
	memcpy(&buf[3], addrL, 1);
	memcpy(&buf[4], val, 32);
	ret = i2c_adapter_block_write_nodatalog(client, buf, length);
	if (ret < 0) {
		LOG_ERR("Firmware write fail ...\n");
		return ret;
	}

	SLEEP(3);
	abov_tk_check_busy(client);

	return 0;
}

static int abov_tk_reset_for_bootmode(struct i2c_client *client)
{
	int ret, retry_count = 10;
	unsigned char buf[16] = {0, };

retry:
	buf[0] = 0xF0;
	buf[1] = 0xAA;
	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		LOG_DBG("write fail(retry:%d)\n", retry_count);
		if (retry_count-- > 0) {
			goto retry;
		}
		return -EIO;
	} else {
		LOG_INFO("success reset & boot mode\n");
		return 0;
	}
}

static int abov_tk_fw_mode_enter(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[40] = {0, };
	pabov_platform_data_t pplatData = client->dev.platform_data;

	buf[0] = 0xAC;
	buf[1] = 0x5B;
	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		LOG_ERR("SEND : fail - addr:0x%02x data:0x%02x 0x%02x... ret:%d\n", client->addr, buf[0], buf[1], ret);
		return -EIO;
	}
	LOG_INFO("SEND : succ - addr:0x%02x data:0x%02x 0x%02x... ret:%d\n", client->addr, buf[0], buf[1], ret);
	SLEEP(5);

	ret = i2c_master_recv(client, buf, 1);
	if (buf[0] != pplatData->fw_mode_ret) {
		LOG_ERR("Enter fw mode fail,device id:0x%02x\n", buf[0]);
		return -EIO;
	}

	LOG_INFO("Enter fw mode success ... device id:0x%02x\n", buf[0]);

	return 0;
}

static int abov_tk_fw_mode_exit(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[40] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0xE1;
	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		LOG_ERR("SEND : fail - addr:0x%02x data:0x%02x 0x%02x ... ret:%d\n", client->addr, buf[0], buf[1], ret);
		return -EIO;
	}
	LOG_INFO("SEND : succ - addr:0x%02x data:0x%02x 0x%02x ... ret:%d\n", client->addr, buf[0], buf[1], ret);

	return 0;
}

static int abov_tk_flash_erase(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[16] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0x2E;

	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		LOG_ERR("SEND : fail - addr:0x%02x data:0x%02x 0x%02x ... ret:%d\n", client->addr, buf[0], buf[1], ret);
		return -EIO;
	}

	LOG_INFO("SEND : succ - addr:0x%02x data:0x%02x 0x%02x ... ret:%d\n", client->addr, buf[0], buf[1], ret);

	return 0;
}

static int abov_tk_i2c_read_checksum(struct i2c_client *client)
{
	unsigned char checksum[6] = {0, };
	unsigned char buf[16] = {0, };
	int ret;

	checksum_h = 0;
	checksum_l = 0;

	buf[0] = 0xAC;
	buf[1] = 0x9E;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = checksum_h_bin;
	buf[5] = checksum_l_bin;
	ret = i2c_master_send(client, buf, 6);

	if (ret != 6) {
		LOG_ERR("SEND : fail - addr:0x%02x len:%d ... ret:%d\n", client->addr, 6, ret);
		return -EIO;
	}
	SLEEP(5);

	buf[0] = 0x00;
	ret = i2c_master_send(client, buf, 1);
	if (ret != 1) {
		LOG_ERR("SEND : fail - addr:0x%02x data:0x%02x ... ret:%d\n", client->addr, buf[0], ret);
		return -EIO;
	}
	SLEEP(5);

	ret = i2c_master_recv(client, checksum, 6);
	if (ret < 0) {
		LOG_ERR("Read checksum fail ... \n");
		return -EIO;
	}

	checksum_h = checksum[4];
	checksum_l = checksum[5];

	return 0;
}

static int _abov_fw_update(struct i2c_client *client, const u8 *image, u32 size)
{
	int ret, ii = 0;
	int count;
	unsigned short address;
	unsigned char addrH, addrL;
	unsigned char data[32] = {0, };

	LOG_INFO("%s: call in\r\n", __func__);

	if (abov_tk_reset_for_bootmode(client) < 0) {
		LOG_ERR("don't reset(enter boot mode)!");
		return -EIO;
	}

	SLEEP(45);

	for (ii = 0; ii < 10; ii++) {
		if (abov_tk_fw_mode_enter(client) < 0) {
			LOG_ERR("don't enter the download mode! %d", ii);
			SLEEP(40);
			continue;
		}
		break;
	}

	if (10 <= ii) {
		return -EAGAIN;
	}

	if (abov_tk_flash_erase(client) < 0) {
		LOG_ERR("don't erase flash data!");
		return -EIO;
	}

	SLEEP(abov_flash_erase_time);

	address = 0x800;
	count = size / 32;

	for (ii = 0; ii < count; ii++) {
		/* first 32byte is header */
		addrH = (unsigned char)((address >> 8) & 0xFF);
		addrL = (unsigned char)(address & 0xFF);
		memcpy(data, &image[ii * 32], 32);
		ret = abov_tk_fw_write(client, &addrH, &addrL, data);
		if (ret < 0) {
			LOG_ERR("fw_write.. ii = 0x%x err\r\n", ii);
			return ret;
		}

		address += 0x20;
		memset(data, 0, 32);
	}

	ret = abov_tk_i2c_read_checksum(client);
	ret = abov_tk_fw_mode_exit(client);
	if ((checksum_h == checksum_h_bin) && (checksum_l == checksum_l_bin)) {
		LOG_INFO("Firmware update success. checksum_h=0x%02x,checksum_h_bin=0x%02x,checksum_l=0x%02x,checksum_l_bin=0x%02x\n",
			checksum_h, checksum_h_bin, checksum_l, checksum_l_bin);
	} else {
		LOG_ERR("Firmware update fail. checksum_h=0x%02x,checksum_h_bin=0x%02x,checksum_l=0x%02x,checksum_l_bin=0x%02x\n",
			checksum_h, checksum_h_bin, checksum_l, checksum_l_bin);
		ret = -1;
	}
	SLEEP(100);

	return ret;
}

static int abov_fw_update(bool force)
{
	int update_loop;
	pabovXX_t this = abov_sar_ptr;
	struct i2c_client *client = this->bus;
	int rc;
	bool fw_upgrade = false;
	u8 fw_version = 0, fw_file_version = 0;
	u8 fw_modelno = 0, fw_file_modeno = 0;
	const struct firmware *fw = NULL;
	char fw_name[32] = {0};

	strcpy(fw_name, this->board->fw_name);
	strlcat(fw_name, ".BIN", NAME_MAX);
	rc = request_firmware(&fw, fw_name, this->pdev);
	if (rc < 0) {
		read_register(this, ABOV_VERSION_REG, &fw_version);
		read_register(this, ABOV_MODELNO_REG, &fw_modelno);
		LOG_ERR("Request firmware failed - %s (%d),current fw info inside IC:Version=0x%02x,ModelNo=0x%02x\n",
				this->board->fw_name, rc, fw_version, fw_modelno);
		return rc;
	}

    if (force == false) {
		read_register(this, ABOV_VERSION_REG, &fw_version);
		read_register(this, ABOV_MODELNO_REG, &fw_modelno);
		LOG_INFO("Version in sensor is:0x%02x ,ModelNo in sensor is:0x%02x\n" ,fw_version ,fw_modelno);
    }

	fw_file_modeno = fw->data[1];
	fw_file_version = fw->data[5];
	checksum_h_bin = fw->data[8];
	checksum_l_bin = fw->data[9];
	LOG_INFO("Version in file is:0x%02x ,ModelNo in file is:0x%02x\n" ,fw_file_version ,fw_file_version);

	if ((force) || (fw_version < fw_file_version) || (fw_modelno != fw_file_modeno)) {
		LOG_INFO("Firmware is not latest,going to fw upgrade...\n");
		fw_upgrade = true;
	} else {
		LOG_INFO("Firmware is latest,exiting fw upgrade...\n");
		fw_upgrade = false;
		this->fw_dl_status = 0;
		rc = -EIO;
		goto rel_fw;
	}

	if (fw_upgrade) {
		this->fw_dl_status = 2;
		for (update_loop = 0; update_loop < 10; update_loop++) {
			rc = _abov_fw_update(client, &fw->data[32], fw->size-32);
			if (rc < 0)
				LOG_ERR("retry : %d times!\n", update_loop);
			else {
				//this funtion under the mutex lock, and the initialize will enable irq,
				//if trigger the interrupts, the cap-sensor will no work, so, initialize() no stay here.
				//initialize(this);
				break;
			}
		}
		if (update_loop >= 10) {
			this->fw_dl_status = 1;
			rc = -EIO;
		}
		this->fw_dl_status = 0;
	}

rel_fw:
	release_firmware(fw);
	return rc;
}



static ssize_t update_fw_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	u8 fw_version = 0;
	pabovXX_t this = abov_sar_ptr;

	read_register(this, ABOV_VERSION_REG, &fw_version);

	return snprintf(buf, 37, "ABOV CapSensor Firmware Version:0x%02x\n", fw_version);
}

static ssize_t update_fw_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	pabovXX_t this = abov_sar_ptr;
	unsigned long val;
	int rc;

	if (count > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

 	if(this->irq_disabled == 0) {
		disable_irq(this->irq);
		this->irq_disabled = 1;
	}
	mutex_lock(&this->mutex);
	if (!this->loading_fw  && val) {
		this->loading_fw = true;
		abov_fw_update(false);
		this->loading_fw = false;
	}
	mutex_unlock(&this->mutex);

	if(this->irq_disabled == 1) {
		enable_irq(this->irq);
		this->irq_disabled = 0;
	}

	return count;
}
static CLASS_ATTR_RW(update_fw);


static ssize_t force_update_fw_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	u8 fw_version = 0;
	pabovXX_t this = abov_sar_ptr;

	read_register(this, ABOV_VERSION_REG, &fw_version);

	return snprintf(buf, 37, "ABOV CapSensor Firmware Version:0x%02x\n", fw_version);
}

static ssize_t force_update_fw_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	pabovXX_t this = abov_sar_ptr;
	unsigned long val;
	int rc;

	if (count > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

 	if(this->irq_disabled == 0) {
		disable_irq(this->irq);
		this->irq_disabled = 1;
	}

	mutex_lock(&this->mutex);
	if (!this->loading_fw  && val) {
		this->loading_fw = true;
		abov_fw_update(true);
		this->loading_fw = false;
	}
	mutex_unlock(&this->mutex);

	if(this->irq_disabled == 1) {
		enable_irq(this->irq);
		this->irq_disabled = 0;
	}

	return count;
}
static CLASS_ATTR_RW(force_update_fw);

static void capsense_update_work(struct work_struct *work)
{
	pabovXX_t this = container_of(work, abovXX_t, fw_update_work);

	LOG_DBG("%s: start update firmware\n", __func__);
	if(this->irq_disabled == 0) {
	    disable_irq(this->irq);
		this->irq_disabled = 1;
    }
	mutex_lock(&this->mutex);
	this->loading_fw = true;
	abov_fw_update(false);
	this->loading_fw = false;
	mutex_unlock(&this->mutex);
	if(this->irq_disabled == 1) {
		enable_irq(this->irq);
		this->irq_disabled = 0;
	}

	if(mEnabled){
		initialize(this);
	}
	LOG_INFO("%s: update firmware end\n", __func__);
}

static void capsense_fore_update_work(struct work_struct *work)
{
	pabovXX_t this = container_of(work, abovXX_t, fw_update_work);

	LOG_DBG("%s: start force update firmware\n", __func__);
	if(this->irq_disabled == 0) {
		disable_irq(this->irq);
		this->irq_disabled = 1;
	}
	mutex_lock(&this->mutex);
	this->loading_fw = true;
	abov_fw_update(true);
	this->loading_fw = false;
	mutex_unlock(&this->mutex);
	if(this->irq_disabled == 1) {
	    enable_irq(this->irq);
		this->irq_disabled = 0;
	}

	if(mEnabled){
		initialize(this);
	}
	LOG_INFO("%s: force update firmware end\n", __func__);
}


/**
 * detect if abov exist or not
 * return 1 if chip exist else return 0
 */
static int abov_detect(struct i2c_client *client)
{
	s32 returnValue = 0, i;
	u8 address = ABOV_VENDOR_ID_REG;
	u8 value = 0xAB;

	if (client) {
		for (i = 0; i < 10; i++) {
			returnValue = i2c_smbus_read_byte_data(client, address);
			LOG_DBG("abov read_register for %d time Addr: 0x%02x Return: 0x%02x\n",
					i, address, returnValue);
			if (returnValue >= 0) {
				if (value == returnValue) {
					LOG_INFO("abov detect success!\n");
					return 1;
				}
			}
			msleep(10);
		}
		LOG_INFO("abov boot detect start\n");
		for (i = 0; i < 3; i++) {
				if(abov_tk_fw_mode_enter(client) == 0) {
					LOG_INFO("abov boot detect success!\n");
					return 2;
				}
		}
	}
	LOG_INFO("abov detect failed!!!\n");
		return 0;
}


/**
 * fn static int abov_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * brief Probe function
 * param client pointer to i2c_client
 * param id pointer to i2c_device_id
 * return Whether probe was successful
 */
static int abov_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pabovXX_t this = 0;
	pabov_t pDevice = 0;
	pabov_platform_data_t pplatData = 0;
	bool isForceUpdate = false;
	int ret;
	struct input_dev *input_ch0 = NULL;
	struct input_dev *input_ch1 = NULL;
	struct input_dev *input_ch2 = NULL;
	struct input_dev *input_ch3 = NULL;
	struct input_dev *input_ch4 = NULL;
	struct power_supply *psy = NULL;

	LOG_INFO("abov_probe() start\n");
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	pplatData = kzalloc(sizeof(struct abov_platform_data), GFP_KERNEL);
	if (!pplatData) {
		LOG_ERR("platform data is required!\n");
		return -EINVAL;
	}
	abov_platform_data_of_init(client, pplatData);
	client->dev.platform_data = pplatData;

        pplatData->cap_vdd = regulator_get(&client->dev, "cap_vdd");
        if (IS_ERR(pplatData->cap_vdd)) {
                if (PTR_ERR(pplatData->cap_vdd) == -EPROBE_DEFER) {
                        ret = PTR_ERR(pplatData->cap_vdd);
                        goto err_vdd_defer;
                }
                LOG_ERR("%s: Failed to get regulator\n", __func__);
        } else {
                ret = regulator_enable(pplatData->cap_vdd);

                if (ret) {
                        regulator_put(pplatData->cap_vdd);
                        LOG_ERR("%s: Error %d enable regulator\n",
                                         __func__, ret);
                        goto err_vdd_defer;
                }
                pplatData->cap_vdd_en = true;
                LOG_INFO("cap_vdd regulator is %s\n",
                                 regulator_is_enabled(pplatData->cap_vdd) ?
                                 "on" : "off");
        }

	pplatData->cap_svdd = regulator_get(&client->dev, "cap_svdd");
        if (IS_ERR(pplatData->cap_svdd)) {
                if (PTR_ERR(pplatData->cap_svdd) == -EPROBE_DEFER) {
                        ret = PTR_ERR(pplatData->cap_svdd);
                        goto err_svdd_error;
                }
                LOG_ERR("%s: Failed to get regulator\n", __func__);
        } else {
                ret = regulator_enable(pplatData->cap_svdd);

                if (ret) {
                        regulator_put(pplatData->cap_svdd);
                        LOG_ERR("%s: Error %d enable regulator\n",
                                         __func__, ret);
                        goto err_svdd_error;
                }
                pplatData->cap_svdd_en = true;
                LOG_INFO("cap_svdd regulator is %s\n",
                                 regulator_is_enabled(pplatData->cap_svdd) ?
                                 "on" : "off");
        }

	/* detect if abov exist or not */
	ret = abov_detect(client);
	if (ret == 0) {
		return -ENODEV;
	}
	if (ret == 2) {
		isForceUpdate = true;
	}

	this = kzalloc(sizeof(abovXX_t), GFP_KERNEL); /* create memory for main struct */
	LOG_DBG("\t Initialized Main Memory: 0x%p\n", this);

	if (this) {
		/* In case we need to reinitialize data
		 * (e.q. if suspend reset device) */
		this->init = initialize;
		/* pointer to function from platform data to get pendown
		 * (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		/* save irq in case we need to reference it */
		this->irq = gpio_to_irq(client->irq);
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;
		this->board = pplatData;
		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS >= 8) {
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = 0; /* UNUSED */
			this->statusFunc[3] = 0;/*read_rawData;  CONV_STAT */
			this->statusFunc[4] = 0; /* COMP_STAT */
			this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
			this->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		/* create memory for device specific struct */
		this->pDevice = pDevice = kzalloc(sizeof(abov_t), GFP_KERNEL);
		LOG_DBG("\t Initialized Device Specific Memory: 0x%p\n",
				pDevice);
		abov_sar_ptr = this;
		if (pDevice) {
			/* for accessing items in user data (e.g. calibrate) */
			ret = sysfs_create_group(&client->dev.kobj, &abov_attr_group);


			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
				pplatData->init_platform_hw();

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;

			if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
				/* Create the input device */
				input_ch0 = input_allocate_device();
				if (!input_ch0)
					return -ENOMEM;

				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch0->evbit);
				input_set_abs_params(input_ch0, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch0 = input_ch0;
				input_ch0->name = this->board->cap_ch0_name;
				if (input_register_device(input_ch0)) {
					LOG_ERR("add ch0 cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch1 = input_allocate_device();
				if (!input_ch1)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch1->evbit);
				input_set_abs_params(input_ch1, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch1 = input_ch1;
				/* save the input pointer and finish initialization */
				input_ch1->name = this->board->cap_ch1_name;
				if (input_register_device(input_ch1)) {
					LOG_INFO("add ch1 cap touch unsuccess\n");
					return -ENOMEM;
				}
			} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
				/* Create the input device */
				input_ch0 = input_allocate_device();
				if (!input_ch0)
					return -ENOMEM;

				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch0->evbit);
				input_set_abs_params(input_ch0, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch0 = input_ch0;
				input_ch0->name = this->board->cap_ch0_name;
				if (input_register_device(input_ch0)) {
					LOG_ERR("add ch0 cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch1 = input_allocate_device();
				if (!input_ch1)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch1->evbit);
				input_set_abs_params(input_ch1, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch1 = input_ch1;
				/* save the input pointer and finish initialization */
				input_ch1->name = this->board->cap_ch1_name;
				if (input_register_device(input_ch1)) {
					LOG_INFO("add ch1 cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch2 = input_allocate_device();
				if (!input_ch2)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch2->evbit);
				input_set_abs_params(input_ch2, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch2 = input_ch2;
				/* save the input pointer and finish initialization */
				input_ch2->name = this->board->cap_ch2_name;
				if (input_register_device(input_ch2)) {
					LOG_ERR("add ch2t cap touch unsuccess\n");
					return -ENOMEM;
				}
			} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
				/* Create the input device */
				input_ch0 = input_allocate_device();
				if (!input_ch0)
					return -ENOMEM;

				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch0->evbit);
				input_set_abs_params(input_ch0, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch0 = input_ch0;
				input_ch0->name = this->board->cap_ch0_name;
				if (input_register_device(input_ch0)) {
					LOG_ERR("add ch0 cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch1 = input_allocate_device();
				if (!input_ch1)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch1->evbit);
				input_set_abs_params(input_ch1, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch1 = input_ch1;
				/* save the input pointer and finish initialization */
				input_ch1->name = this->board->cap_ch1_name;
				if (input_register_device(input_ch1)) {
					LOG_INFO("add ch1 cap touch unsuccess\n");
					return -ENOMEM;
				}

				/* Create the input device */
				input_ch2 = input_allocate_device();
				if (!input_ch2)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch2->evbit);
				input_set_abs_params(input_ch2, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch2 = input_ch2;
				/* save the input pointer and finish initialization */
				input_ch2->name = this->board->cap_ch2_name;
				if (input_register_device(input_ch2)) {
					LOG_ERR("add ch2t cap touch unsuccess\n");
					return -ENOMEM;
				}
				input_ch3 = input_allocate_device();
				if (!input_ch3)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch3->evbit);
				input_set_abs_params(input_ch3, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch3 = input_ch3;
				/* save the input pointer and finish initialization */
				input_ch3->name = this->board->cap_ch3_name;
				if (input_register_device(input_ch3)) {
					LOG_ERR("add ch3t cap touch unsuccess\n");
					return -ENOMEM;
				}
			} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
				/* Create the input device */
				input_ch0 = input_allocate_device();
				if (!input_ch0)
					return -ENOMEM;

				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch0->evbit);
				input_set_abs_params(input_ch0, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch0 = input_ch0;
				input_ch0->name = this->board->cap_ch0_name;
				if (input_register_device(input_ch0)) {
					LOG_ERR("add ch0 cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch1 = input_allocate_device();
				if (!input_ch1)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch1->evbit);
				input_set_abs_params(input_ch1, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch1 = input_ch1;
				/* save the input pointer and finish initialization */
				input_ch1->name = this->board->cap_ch1_name;
				if (input_register_device(input_ch1)) {
					LOG_INFO("add ch1 cap touch unsuccess\n");
					return -ENOMEM;
				}

				/* Create the input device */
				input_ch2 = input_allocate_device();
				if (!input_ch2)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch2->evbit);
				input_set_abs_params(input_ch2, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch2 = input_ch2;
				/* save the input pointer and finish initialization */
				input_ch2->name = this->board->cap_ch2_name;
				if (input_register_device(input_ch2)) {
					LOG_ERR("add ch2t cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch3 = input_allocate_device();
				if (!input_ch3)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch3->evbit);
				input_set_abs_params(input_ch3, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch3 = input_ch3;
				/* save the input pointer and finish initialization */
				input_ch3->name = this->board->cap_ch3_name;
				if (input_register_device(input_ch3)) {
					LOG_ERR("add ch3t cap touch unsuccess\n");
					return -ENOMEM;
				}
				/* Create the input device */
				input_ch4 = input_allocate_device();
				if (!input_ch4)
					return -ENOMEM;
				/* Set all the keycodes */
				__set_bit(EV_ABS, input_ch4->evbit);
				input_set_abs_params(input_ch4, ABS_DISTANCE, -1, 100, 0, 0);
				/* save the input pointer and finish initialization */
				pDevice->pbuttonInformation->input_ch4 = input_ch4;
				/* save the input pointer and finish initialization */
				input_ch4->name = this->board->cap_ch4_name;
				if (input_register_device(input_ch4)) {
					LOG_ERR("add ch4t cap touch unsuccess\n");
					return -ENOMEM;
				}
			}
		}

		ret = class_register(&capsense_class);
		if (ret < 0) {
			LOG_ERR("Create fsys class failed (%d)\n", ret);
			return ret;
		}

		ret = class_create_file(&capsense_class, &class_attr_reset);
		if (ret < 0) {
			LOG_ERR("Create reset file failed (%d)\n", ret);
			return ret;
		}

		ret = class_create_file(&capsense_class, &class_attr_int_state);
                if (ret < 0) {
                        LOG_DBG("Create int_state file failed (%d)\n", ret);
                        return ret;
                }

		ret = class_create_file(&capsense_class, &class_attr_fw_download_status);
		if (ret < 0) {
			LOG_DBG("Create fw_download_status file failed (%d)\n", ret);
			return ret;
		}

		ret = class_create_file(&capsense_class, &class_attr_enable);
		if (ret < 0) {
			LOG_ERR("Create enable file failed (%d)\n", ret);
			return ret;
		}

		ret = class_create_file(&capsense_class, &class_attr_reg);
		if (ret < 0) {
			LOG_DBG("Create reg file failed (%d)\n", ret);
			return ret;
		}

		ret = class_create_file(&capsense_class, &class_attr_update_fw);
		if (ret < 0) {
			LOG_ERR("Create update_fw file failed (%d)\n", ret);
			return ret;
		}

		ret = class_create_file(&capsense_class, &class_attr_force_update_fw);
		if (ret < 0) {
			LOG_ERR("Create update_fw file failed (%d)\n", ret);
			return ret;
		}
		/*restore sys/class/capsense label*/
		kobject_uevent(&capsense_class.p->subsys.kobj, KOBJ_CHANGE);

#ifdef USE_SENSORS_CLASS
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			sensors_capsensor_ch0_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch0_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch0_cdev.name = this->board->cap_ch0_name;
			ret = sensors_classdev_register(&input_ch0->dev, &sensors_capsensor_ch0_cdev);
			if (ret < 0)
				LOG_ERR("create ch0 cap sensor_class  file failed (%d)\n", ret);

			sensors_capsensor_ch1_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch1_cdev.sensors_poll_delay = NULL;;
			sensors_capsensor_ch1_cdev.name = this->board->cap_ch1_name;
			ret = sensors_classdev_register(&input_ch1->dev, &sensors_capsensor_ch1_cdev);
			if (ret < 0)
				LOG_ERR("create ch1 cap sensor_class file failed (%d)\n", ret);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			sensors_capsensor_ch0_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch0_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch0_cdev.name = this->board->cap_ch0_name;
			ret = sensors_classdev_register(&input_ch0->dev, &sensors_capsensor_ch0_cdev);
			if (ret < 0)
				LOG_ERR("create ch0 cap sensor_class  file failed (%d)\n", ret);

			sensors_capsensor_ch1_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch1_cdev.sensors_poll_delay = NULL;;
			sensors_capsensor_ch1_cdev.name = this->board->cap_ch1_name;
			ret = sensors_classdev_register(&input_ch1->dev, &sensors_capsensor_ch1_cdev);
			if (ret < 0)
				LOG_ERR("create ch1 cap sensor_class file failed (%d)\n", ret);

			sensors_capsensor_ch2_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch2_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch2_cdev.name = this->board->cap_ch2_name;
			ret = sensors_classdev_register(&input_ch2->dev, &sensors_capsensor_ch2_cdev);
			if (ret < 0)
				LOG_ERR("create ch2 cap sensor_class file failed (%d)\n", ret);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			sensors_capsensor_ch0_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch0_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch0_cdev.name = this->board->cap_ch0_name;
			ret = sensors_classdev_register(&input_ch0->dev, &sensors_capsensor_ch0_cdev);
			if (ret < 0)
				LOG_ERR("create ch0 cap sensor_class  file failed (%d)\n", ret);

		sensors_capsensor_ch1_cdev.sensors_enable = capsensor_set_enable;
		sensors_capsensor_ch1_cdev.sensors_poll_delay = NULL;;
		sensors_capsensor_ch1_cdev.name = this->board->cap_ch1_name;
		ret = sensors_classdev_register(&input_ch1->dev, &sensors_capsensor_ch1_cdev);
		if (ret < 0)
			LOG_ERR("create ch1 cap sensor_class file failed (%d)\n", ret);

			sensors_capsensor_ch2_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch2_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch2_cdev.name = this->board->cap_ch2_name;
			ret = sensors_classdev_register(&input_ch2->dev, &sensors_capsensor_ch2_cdev);
			if (ret < 0)
				LOG_ERR("create ch2 cap sensor_class file failed (%d)\n", ret);

			sensors_capsensor_ch3_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch3_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch3_cdev.name = this->board->cap_ch3_name;
			ret = sensors_classdev_register(&input_ch3->dev, &sensors_capsensor_ch3_cdev);
			if (ret < 0)
				LOG_ERR("create ch3 cap sensor_class file failed (%d)\n", ret);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			sensors_capsensor_ch0_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch0_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch0_cdev.name = this->board->cap_ch0_name;
			ret = sensors_classdev_register(&input_ch0->dev, &sensors_capsensor_ch0_cdev);
			if (ret < 0)
				LOG_ERR("create ch0 cap sensor_class  file failed (%d)\n", ret);

			sensors_capsensor_ch1_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch1_cdev.sensors_poll_delay = NULL;;
			sensors_capsensor_ch1_cdev.name = this->board->cap_ch1_name;
			ret = sensors_classdev_register(&input_ch1->dev, &sensors_capsensor_ch1_cdev);
			if (ret < 0)
				LOG_ERR("create ch1 cap sensor_class file failed (%d)\n", ret);

			sensors_capsensor_ch2_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch2_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch2_cdev.name = this->board->cap_ch2_name;
			ret = sensors_classdev_register(&input_ch2->dev, &sensors_capsensor_ch2_cdev);
			if (ret < 0)
				LOG_ERR("create ch2 cap sensor_class file failed (%d)\n", ret);

			sensors_capsensor_ch3_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch3_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch3_cdev.name = this->board->cap_ch3_name;
			ret = sensors_classdev_register(&input_ch3->dev, &sensors_capsensor_ch3_cdev);
			if (ret < 0)
				LOG_ERR("create ch3 cap sensor_class file failed (%d)\n", ret);

			sensors_capsensor_ch4_cdev.sensors_enable = capsensor_set_enable;
			sensors_capsensor_ch4_cdev.sensors_poll_delay = NULL;
			sensors_capsensor_ch4_cdev.name = this->board->cap_ch4_name;
			ret = sensors_classdev_register(&input_ch4->dev, &sensors_capsensor_ch4_cdev);
			if (ret < 0)
				LOG_ERR("create ch4 cap sensor_class file failed (%d)\n", ret);
		}
#endif

		abovXX_sar_init(this);

		write_register(this, ABOV_CTRL_MODE_REG, ABOV_CTRL_MODE_STOP);
		mEnabled = 0;

		INIT_WORK(&this->ps_notify_work, ps_notify_callback_work);
		this->ps_notif.notifier_call = ps_notify_callback;
		ret = power_supply_reg_notifier(&this->ps_notif);
		if (ret) {
			LOG_ERR(
				"Unable to register ps_notifier: %d\n", ret);
			goto free_ps_notifier;
		}

		psy = power_supply_get_by_name("usb");
		if (psy) {
			ret = ps_get_state(psy, &this->ps_is_present);
			if (ret) {
				LOG_ERR(
					"psy get property failed rc=%d\n",
					ret);
				goto free_ps_notifier;
			}
		}

		this->loading_fw = false;
		this->fw_dl_status = 1;
		if (isForceUpdate == true) {
		    INIT_WORK(&this->fw_update_work, capsense_fore_update_work);
		} else {
			INIT_WORK(&this->fw_update_work, capsense_update_work);
		}
		schedule_work(&this->fw_update_work);


       LOG_INFO("abov_probe() end\n");
		return  0;
	}
	return -ENOMEM;

free_ps_notifier:
    LOG_ERR("%s free ps notifier:.\n", __func__);
	power_supply_unreg_notifier(&this->ps_notif);
    regulator_disable(pplatData->cap_svdd);
    regulator_put(pplatData->cap_svdd);

err_svdd_error:
	LOG_ERR("%s svdd defer.\n", __func__);
	regulator_disable(pplatData->cap_vdd);
	regulator_put(pplatData->cap_vdd);

err_vdd_defer:
	LOG_ERR("%s input free device.\n", __func__);
	if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
		input_free_device(input_ch0);
		input_free_device(input_ch1);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
		input_free_device(input_ch0);
		input_free_device(input_ch1);
		input_free_device(input_ch2);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
		input_free_device(input_ch0);
		input_free_device(input_ch1);
		input_free_device(input_ch2);
		input_free_device(input_ch3);
	} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
		input_free_device(input_ch0);
		input_free_device(input_ch1);
		input_free_device(input_ch2);
		input_free_device(input_ch3);
		input_free_device(input_ch4);
	}

	return ret;
}

/**
 * fn static int abov_remove(struct i2c_client *client)
 * brief Called when device is to be removed
 * param client Pointer to i2c_client struct
 * return Value from abovXX_sar_remove()
 */
static int abov_remove(struct i2c_client *client)
{
	pabov_platform_data_t pplatData = 0;
	pabov_t pDevice = 0;
	pabovXX_t this = i2c_get_clientdata(client);

	pDevice = this->pDevice;
	if (this && pDevice) {
#ifdef USE_SENSORS_CLASS
        if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			sensors_classdev_unregister(&sensors_capsensor_ch0_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch1_cdev);
        } else if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
  			sensors_classdev_unregister(&sensors_capsensor_ch0_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch1_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch2_cdev);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			sensors_classdev_unregister(&sensors_capsensor_ch0_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch1_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch2_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch3_cdev);
		}else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			sensors_classdev_unregister(&sensors_capsensor_ch0_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch1_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch2_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch3_cdev);
			sensors_classdev_unregister(&sensors_capsensor_ch4_cdev);
		}
#endif
        if (abov_channel_number == ABOV_CHANNEL_NUMBER_TWO) {
			input_unregister_device(pDevice->pbuttonInformation->input_ch0);
			input_unregister_device(pDevice->pbuttonInformation->input_ch1);
        }
		if (abov_channel_number == ABOV_CHANNEL_NUMBER_THREE) {
			input_unregister_device(pDevice->pbuttonInformation->input_ch0);
			input_unregister_device(pDevice->pbuttonInformation->input_ch1);
			input_unregister_device(pDevice->pbuttonInformation->input_ch2);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FOUR) {
			input_unregister_device(pDevice->pbuttonInformation->input_ch0);
			input_unregister_device(pDevice->pbuttonInformation->input_ch1);
			input_unregister_device(pDevice->pbuttonInformation->input_ch2);
			input_unregister_device(pDevice->pbuttonInformation->input_ch3);
		} else if (abov_channel_number == ABOV_CHANNEL_NUMBER_FIVE) {
			input_unregister_device(pDevice->pbuttonInformation->input_ch0);
			input_unregister_device(pDevice->pbuttonInformation->input_ch1);
			input_unregister_device(pDevice->pbuttonInformation->input_ch2);
			input_unregister_device(pDevice->pbuttonInformation->input_ch3);
			input_unregister_device(pDevice->pbuttonInformation->input_ch4);
		}

		if (this->board->cap_svdd_en) {
			regulator_disable(this->board->cap_svdd);
			regulator_put(this->board->cap_svdd);
		}

		if (this->board->cap_vdd_en) {
			regulator_disable(this->board->cap_vdd);
			regulator_put(this->board->cap_vdd);
		}

		sysfs_remove_group(&client->dev.kobj, &abov_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw();
		kfree(this->pDevice);
	}
	return abovXX_sar_remove(this);
}

static int abov_suspend(struct device *dev)
{
	pabovXX_t this = dev_get_drvdata(dev);

	abovXX_suspend(this);
	return 0;
}

static int abov_resume(struct device *dev)
{
	pabovXX_t this = dev_get_drvdata(dev);

	abovXX_resume(this);
	return 0;
}
static SIMPLE_DEV_PM_OPS(abov_pm_ops, abov_suspend, abov_resume);


#ifdef CONFIG_OF
static const struct of_device_id abov_match_tbl[] = {
	{ .compatible = "abov,abov_sar_mmi_overlay_7ch" },
	{ },
};
MODULE_DEVICE_TABLE(of, abov_match_tbl);
#endif

static struct i2c_device_id abov_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, abov_idtable);

static struct i2c_driver abov_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME,
		.of_match_table = of_match_ptr(abov_match_tbl),
		.pm     = &abov_pm_ops,
	},
	.id_table = abov_idtable,
	.probe	  = abov_probe,
	.remove	  = abov_remove,
};
static int __init abov_init(void)
{
	return i2c_add_driver(&abov_driver);
}
static void __exit abov_exit(void)
{
	i2c_del_driver(&abov_driver);
}

module_init(abov_init);
module_exit(abov_exit);

MODULE_AUTHOR("ABOV Corp.");
MODULE_DESCRIPTION("ABOV Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");

#ifdef USE_THREADED_IRQ
static void abovXX_process_interrupt(pabovXX_t this, u8 nirqlow)
{
	if (!this) {
		pr_err("abovXX_worker_func, NULL abovXX_t\n");
		return;
	}
	/* since we are not in an interrupt don't need to disable irq. */
	this->statusFunc[6](this);
	if (unlikely(this->useIrqTimer && nirqlow)) {
		/* In case we need to send a timer for example on a touchscreen
		 * checking penup, perform this here
		 */
		cancel_delayed_work(&this->dworker);
		schedule_delayed_work(&this->dworker, msecs_to_jiffies(this->irqTimeout));
		LOG_INFO("Schedule Irq timer");
	}
}


static void abovXX_worker_func(struct work_struct *work)
{
	pabovXX_t this = 0;

	if (work) {
		this = container_of(work, abovXX_t, dworker.work);
		if (!this) {
			LOG_ERR("abovXX_worker_func, NULL abovXX_t\n");
			return;
		}
		if ((!this->get_nirq_low) || (!this->get_nirq_low(this->board->irq_gpio))) {
			/* only run if nirq is high */
			abovXX_process_interrupt(this, 0);
		}
	} else {
		LOG_ERR("abovXX_worker_func, NULL work_struct\n");
	}
}
static irqreturn_t abovXX_interrupt_thread(int irq, void *data)
{
	pabovXX_t this = 0;
	this = data;

	mutex_lock(&this->mutex);
	LOG_DBG("abovXX_irq\n");
	this->int_state = 1;
	if ((!this->get_nirq_low) || this->get_nirq_low(this->board->irq_gpio))
		abovXX_process_interrupt(this, 1);
	else
		LOG_ERR("abovXX_irq - nirq read high\n");
	mutex_unlock(&this->mutex);
	return IRQ_HANDLED;
}
#else
static void abovXX_schedule_work(pabovXX_t this, unsigned long delay)
{
	unsigned long flags;

	if (this) {
		LOG_DBG("abovXX_schedule_work()\n");
		spin_lock_irqsave(&this->lock, flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		/*
		 *after waiting for a delay, this put the job in the kernel-global
		 workqueue. so no need to create new thread in work queue.
		 */
		schedule_delayed_work(&this->dworker, delay);
		spin_unlock_irqrestore(&this->lock, flags);
	} else
		LOG_ERR("abovXX_schedule_work, NULL pabovXX_t\n");
}

static irqreturn_t abovXX_irq(int irq, void *pvoid)
{
	pabovXX_t this = 0;
	if (pvoid) {
		this = (pabovXX_t)pvoid;
		LOG_DBG("abovXX_irq\n");
		this->int_state = 1;
		if ((!this->get_nirq_low) || this->get_nirq_low(this->board->irq_gpio)) {
			LOG_DBG("abovXX_irq - Schedule Work\n");
			abovXX_schedule_work(this, 0);
		} else
			LOG_ERR("abovXX_irq - nirq read high\n");
	} else
		LOG_ERR("abovXX_irq, NULL pvoid\n");
	return IRQ_HANDLED;
}

static void abovXX_worker_func(struct work_struct *work)
{
	pabovXX_t this = 0;
	int counter = 0;
	u8 nirqLow = 0;

	if (work) {
		this = container_of(work, abovXX_t, dworker.work);

		if (!this) {
			LOG_ERR("abovXX_worker_func, NULL abovXX_t\n");
			return;
		}
		if (unlikely(this->useIrqTimer)) {
			if ((!this->get_nirq_low) || this->get_nirq_low(this->board->irq_gpio))
				nirqLow = 1;
		}
		/* since we are not in an interrupt don't need to disable irq. */
		counter = -1;
		while ((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
			LOG_DBG("Looping Counter %d\n", counter);
			if (((status >> counter) & 0x01) && (this->statusFunc[counter])) {
				LOG_DBG("Function Pointer Found. Calling\n");
				this->statusFunc[counter](this);
			}
		}
		if (unlikely(this->useIrqTimer && nirqLow)) {
			/* Early models and if RATE=0 for newer models require a penup timer */
			/* Queue up the function again for checking on penup */
			abovXX_schedule_work(this, msecs_to_jiffies(this->irqTimeout));
		}
	} else {
		LOG_ERR("abovXX_worker_func, NULL work_struct\n");
	}
}
#endif

void abovXX_suspend(pabovXX_t this)
{
	if (this) {
		LOG_DBG("ABOV suspend[%d]: disable irq!\n",mEnabled);
		disable_irq(this->irq);
		/* if upper layer don't disable capsensor, */
		/* we  should let it enter sleep in suspend. */
		if (mEnabled)
			write_register(this, ABOV_CTRL_MODE_REG, ABOV_CTRL_MODE_SLEEP);
	}
}
void abovXX_resume(pabovXX_t this)
{
	if (this) {
		LOG_DBG("ABOV resume[%d]: enable irq!\n",mEnabled);
		/* we should let capsensor enter active in resume*/
		if (mEnabled)
			write_register(this, ABOV_CTRL_MODE_REG, ABOV_CTRL_MODE_ACTIVE);
		enable_irq(this->irq);
	}
}

int abovXX_sar_init(pabovXX_t this)
{
	int err = 0;

	if (this && this->pDevice) {
#ifdef USE_THREADED_IRQ

		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, abovXX_worker_func);


		/* initialize mutex */
		mutex_init(&this->mutex);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_threaded_irq(this->irq, NULL, abovXX_interrupt_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, this->pdev->driver->name,
				this);
#else
		/* initialize spin lock */
		spin_lock_init(&this->lock);

		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, abovXX_worker_func);

		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_irq(this->irq, abovXX_irq, IRQF_TRIGGER_FALLING,
				this->pdev->driver->name, this);
#endif
		if (err) {
			LOG_ERR("irq %d busy?\n", this->irq);
			return err;
		}
		this->int_state = 0;
#ifdef USE_THREADED_IRQ
		LOG_INFO("registered with threaded irq (%d)\n", this->irq);
#else
		LOG_INFO("registered with irq (%d)\n", this->irq);
#endif
		/* call init function pointer (this should initialize all registers */
		if (this->init)
			return this->init(this);
		LOG_ERR("No init function!!!!\n");
	}
	return -ENOMEM;
}

int abovXX_sar_remove(pabovXX_t this)
{
	if (this) {
		cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
		free_irq(this->irq, this);
		kfree(this);
		return 0;
	}
	return -ENOMEM;
}
