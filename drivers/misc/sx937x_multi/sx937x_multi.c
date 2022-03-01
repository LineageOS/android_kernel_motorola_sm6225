/*! \file sx937x.c
 * \brief  SX937x Driver
 *
 * Driver for the SX937x
 * Copyright (c) 2018 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DRIVER_NAME "sx937x_multi"

#define MAX_WRITE_ARRAY_SIZE 32

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/version.h>
//#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#include <linux/sensors.h>
#include <linux/input/sx937x_multi.h> 	/* main struct, interrupt,init,pointers */
//#include "base.h"

#define LOG_TAG "[sar SX937x]: "

#define LOG_INFO(fmt, args...)    pr_info(LOG_TAG "[INFO]" "<%s><%d>"fmt, __func__, __LINE__, ##args)
#define LOG_DBG(fmt, args...)     pr_debug(LOG_TAG "[DBG]" "<%s><%d>"fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...)	   pr_err(LOG_TAG "[ERR]" "<%s><%d>"fmt, __func__, __LINE__, ##args)

#define SX937x_I2C_M_WR                 0 /* for i2c Write */
#define SX937x_I2C_M_RD                 1 /* for i2c Read */

#define IDLE			    0
#define PROXACTIVE			  1
#define BODYACTIVE			  2

#define MAIN_SENSOR		1 //CS1

/* Failer Index */
#define SX937x_ID_ERROR 	0x1
#define SX937x_NIRQ_ERROR	0x2
#define SX937x_CONN_ERROR	0x4
#define SX937x_I2C_ERROR	0x8

#define SX937X_I2C_WATCHDOG_TIME 10000
#define SX937X_I2C_WATCHDOG_TIME_ERR 2000

#define MAX_CHANNEL_NUMBER 8
static struct class *capsense_class;


//irq use
static int sx937x_get_nirq_low(psx93XX_t this)
{
	return  !gpio_get_value(this->hw->irq_gpio);
}

static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
	unsigned long flags;
	if (this)
	{
		LOG_DBG("sx93XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock,flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		//after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue.
		schedule_delayed_work(&this->dworker,delay);
		spin_unlock_irqrestore(&this->lock,flags);
	}
	else
		LOG_ERR("sx93XX_schedule_work, NULL psx93XX_t\n");
}

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
	psx93XX_t this = 0;
	if (pvoid)
	{
		this = (psx93XX_t)pvoid;
		if ((!this->get_nirq_low) || this->get_nirq_low(this))
		{
			LOG_DBG("sx93XX_irq - call sx93XX_schedule_work\n");
			sx93XX_schedule_work(this,0);
			this->int_state = 1;
		}
		else
		{
			LOG_DBG("sx93XX_irq - nirq read high\n");
		}
	}
	else
	{
		LOG_ERR("sx93XX_irq, NULL pvoid\n");
	}
	return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;
	if (work)
	{
		this = container_of(work,sx93XX_t,dworker.work);

		if (!this)
		{
			LOG_ERR("sx93XX_worker_func, NULL sx93XX_t\n");
			return;
		}
		if (unlikely(this->useIrqTimer))
		{
			if ((!this->get_nirq_low) || this->get_nirq_low(this))
			{
				nirqLow = 1;
			}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = this->refreshStatus(this);
		counter = -1;
		LOG_DBG("Worker_func - Refresh Status %d, use_timer_in_irq:%d\n", status, this->useIrqTimer);

		while((++counter) < MAX_NUM_STATUS_BITS)   /* counter start from MSB */
		{
			if (((status>>counter) & 0x01) && (this->statusFunc[counter]))
			{
				LOG_DBG("SX937x Function Pointer Found. Calling\n");
				this->statusFunc[counter](this);
			}
		}
		if (unlikely(this->useIrqTimer && nirqLow))
		{
			/* Early models and if RATE=0 for newer models require a penup timer */
			/* Queue up the function again for checking on penup */
			sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
		}
	}
	else
	{
		LOG_ERR("sx93XX_worker_func, NULL work_struct\n");
	}
}

int sx93XX_IRQ_init(psx93XX_t this)
{
	int err = 0;
	if (this)
	{
		this->int_state = 0;
		/* initialize spin lock */
		spin_lock_init(&this->lock);
		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
				this->hw->dbg_name, this);
		if (err)
		{
			LOG_ERR("irq %d busy?\n", this->irq);
			return err;
		}
		LOG_INFO("registered with irq (%d)\n", this->irq);
	}
	return -ENOMEM;
}

//I2C use
/*! \fn static int sx937x_i2c_write_16bit(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int sx937x_i2c_write_16bit(void *bus, u16 reg_addr, u32 buf)
{
	int ret =  -ENOMEM;
	struct i2c_client *i2c = 0;
	struct i2c_msg msg;
	unsigned char w_buf[6];

	if (bus)
	{
		i2c = bus;
		w_buf[0] = (u8)(reg_addr>>8);
		w_buf[1] = (u8)(reg_addr);
		w_buf[2] = (u8)(buf>>24);
		w_buf[3] = (u8)(buf>>16);
		w_buf[4] = (u8)(buf>>8);
		w_buf[5] = (u8)(buf);

		msg.addr = i2c->addr;
		msg.flags = SX937x_I2C_M_WR;
		msg.len = 6; //2bytes regaddr + 4bytes data
		msg.buf = (u8 *)w_buf;

		ret = i2c_transfer(i2c->adapter, &msg, 1);
		if (ret < 0)
			LOG_ERR(" i2c write reg 0x%x error %d\n", reg_addr, ret);

	}
	return ret;
}

/*! \fn static int sx937x_i2c_read_16bit(psx93XX_t this, u8 address, u8 *value)
 * \brief Reads a register's value from the device
 * \param this Pointer to main parent struct
 * \param address 8-Bit address to read from
 * \param value Pointer to 8-bit value to save register value to
 * \return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
static int sx937x_i2c_read_16bit(void *bus, u16 reg_addr, u32 *data32)
{
	int ret =  -ENOMEM;
	struct i2c_client *i2c = 0;
	struct i2c_msg msg[2];
	u8 w_buf[2];
	u8 buf[4];

	if (bus)
	{
		i2c = bus;

		w_buf[0] = (u8)(reg_addr>>8);
		w_buf[1] = (u8)(reg_addr);

		msg[0].addr = i2c->addr;
		msg[0].flags = SX937x_I2C_M_WR;
		msg[0].len = 2;
		msg[0].buf = (u8 *)w_buf;

		msg[1].addr = i2c->addr;;
		msg[1].flags = SX937x_I2C_M_RD;
		msg[1].len = 4;
		msg[1].buf = (u8 *)buf;

		ret = i2c_transfer(i2c->adapter, msg, 2);
		if (ret < 0)
			LOG_ERR("i2c read reg 0x%x error %d\n", reg_addr, ret);

		data32[0] = ((u32)buf[0]<<24) | ((u32)buf[1]<<16) | ((u32)buf[2]<<8) | ((u32)buf[3]);

	}
	return ret;
}


/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
	u32 data = 0;
	if (this)
	{
		if (sx937x_i2c_read_16bit(this->bus,SX937X_IRQ_SOURCE,&data) > 0) //bug
			return (data & 0x00FF);
	}
	return 0;
}

static int sx937x_Hardware_Check(psx93XX_t this)
{
	int ret;
	u32 idCode;
	u8 loop = 0;
	this->failStatusCode = 0;

	//Check th IRQ Status
	while(this->get_nirq_low && this->get_nirq_low(this))
	{
		read_regStat(this);
		msleep(100);
		if(++loop >10)
		{
			this->failStatusCode = SX937x_NIRQ_ERROR;
			break;
		}
	}

	//Check I2C Connection
	ret = sx937x_i2c_read_16bit(this->bus, SX937X_DEVICE_INFO, &idCode);
	if(ret < 0 || idCode == 0)
	{
		this->failStatusCode |= SX937x_I2C_ERROR;
	}

	if(idCode!= SX937X_WHOAMI_VALUE)
	{
		this->failStatusCode |= SX937x_ID_ERROR;
	}

	LOG_ERR("sx937x idcode = 0x%x, failcode = 0x%x\n", idCode, this->failStatusCode);
	return this->failStatusCode;
}

static int sx937x_global_variable_init(psx93XX_t this)
{
	this->irq_disabled = 0;
	this->failStatusCode = 0;
	this->reg_in_dts = true;
	return 0;
}

/*! \brief Perform a manual offset calibration
 * \param this Pointer to main parent struct
 * \return Value return value from the write register
 */
static int manual_offset_calibration(psx937x_platform_data_t data)
{
	int ret = 0;
	ret = sx937x_i2c_write_16bit(data->bus, SX937X_COMMAND, 0xE);
	return ret;

}

//class node use
static void read_dbg_raw(psx93XX_t this)
{
	int ph;
	u32 uData, ph_sel;
	s32 ant_use, ant_raw;
	s32 avg, diff;
	u16 off;
	s32 adc_min, adc_max, use_flt_dlt_var;
	s32 ref_a_use=0, ref_b_use=0, ref_c_use=0;
	int ref_ph_a, ref_ph_b, ref_ph_c;

	ref_ph_a = this->hw->ref_phase_a;
	ref_ph_b = this->hw->ref_phase_b;
	ref_ph_c = this->hw->ref_phase_c;

	sx937x_i2c_read_16bit(this->bus, SX937X_DEVICE_STATUS_A, &uData);
	LOG_INFO("SX937X_STAT0_REG= 0x%X\n", uData);

	if(ref_ph_a != -1)
	{
		sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ref_ph_a*4, &uData);
		ref_a_use = (s32)uData >> 10;
	}
	if(ref_ph_b != -1)
	{
		sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ref_ph_b*4, &uData);
		ref_b_use = (s32)uData >> 10;
	}
	if(ref_ph_c != -1)
	{
		sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ref_ph_c*4, &uData);
		ref_c_use = (s32)uData >> 10;
	}

	sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_SETUP, &ph_sel);

	sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_READBACK_0, &uData);
	adc_min = (s32)uData>>10;
	sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_READBACK_1, &uData);
	adc_max = (s32)uData>>10;
	sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_READBACK_2, &uData);
	ant_raw = (s32)uData>>10;
	sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_READBACK_3, &uData);
	use_flt_dlt_var = (s32)uData>>4;

	ph = (ph_sel >> 3) & 0x7;

	sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ph*4, &uData);
	ant_use = (s32)uData>>10;

	sx937x_i2c_read_16bit(this->bus, SX937X_AVERAGE_PH0 + ph*4, &uData);
	avg = (s32)uData>>10;
	sx937x_i2c_read_16bit(this->bus, SX937X_DIFF_PH0 + ph*4, &uData);
	diff = (s32)uData>>10;
	sx937x_i2c_read_16bit(this->bus, SX937X_OFFSET_PH0 + ph*4*3, &uData);
	off = (u16)(uData & 0x3FFF);
	//state = psmtcButtons[ph].state;

	LOG_INFO("SMTC_DBG PH= %d USE= %d RAW= %d PH%d_USE= %d PH%d_USE= %d PH%d_USE= %d AVG= %d DIFF= %d OFF= %d ADC_MIN= %d ADC_MAX= %d DLT= %d SMTC_END\n",
			ph, ant_use, ant_raw, ref_ph_a, ref_a_use,  ref_ph_b, ref_b_use, ref_ph_c, ref_c_use, avg,diff,off, adc_min,adc_max, use_flt_dlt_var);
}

static void read_rawData(psx93XX_t this)
{
	u8 csx, index;
	s32 useful, average, diff;
	s32 ref_a_use=0, ref_b_use=0, ref_c_use=0;
	u32 uData;
	u32 use_hex, avg_hex, dif_hex, dlt_hex, dbg_ph;
	u16 offset;
	int ref_ph_a, ref_ph_b, ref_ph_c;

	if(this)
	{
		ref_ph_a = this->hw->ref_phase_a;
		ref_ph_b = this->hw->ref_phase_b;
		ref_ph_c = this->hw->ref_phase_c;
		LOG_INFO("[SX937x] ref_ph_a= %d ref_ph_b= %d ref_ph_c= %d\n", ref_ph_a, ref_ph_b, ref_ph_c);

		sx937x_i2c_read_16bit(this->bus, SX937X_DEVICE_STATUS_A, &uData);
		sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_SETUP, &dbg_ph);
		dbg_ph = (dbg_ph >> 3) & 0x7;
		sx937x_i2c_read_16bit(this->bus, SX937X_DEBUG_READBACK_3, &dlt_hex);

		if(ref_ph_a != -1)
		{
			sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ref_ph_a*4, &uData);
			ref_a_use = (s32)uData >> 10;
		}
		if(ref_ph_b != -1)
		{
			sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ref_ph_b*4, &uData);
			ref_b_use = (s32)uData >> 10;
		}
		if(ref_ph_c != -1)
		{
			sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + ref_ph_c*4, &uData);
			ref_c_use = (s32)uData >> 10;
		}

		for(csx =0; csx<8; csx++)
		{
			index = csx*4;
			sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + index, &use_hex);
			useful = (s32)use_hex>>10;
			sx937x_i2c_read_16bit(this->bus, SX937X_AVERAGE_PH0 + index, &avg_hex);
			average = (s32)avg_hex>>10;
			sx937x_i2c_read_16bit(this->bus, SX937X_DIFF_PH0 + index, &dif_hex);
			diff = (s32)dif_hex>>10;
			sx937x_i2c_read_16bit(this->bus, SX937X_OFFSET_PH0 + index*3, &uData);
			offset = (u16)(uData & 0x3FFF);

			//state = this->hw->[csx].state;

			LOG_INFO("SMTC_DAT PH= %d DIFF= %d USE= %d PH%d_USE= %d PH%d_USE= %d PH%d_USE= %d OFF= %d AVG= %d SMTC_END\n",
					csx, diff, useful, ref_ph_a, ref_a_use, ref_ph_b, ref_b_use, ref_ph_c, ref_c_use, offset, average);

			LOG_INFO("SMTC_HEX PH= %d USE= 0x%X AVG= 0x%X DIF= 0x%X PH%d_DLT= 0x%X SMTC_END\n",
					csx, use_hex, avg_hex, dif_hex, dbg_ph, dlt_hex);
		}

		read_dbg_raw(this);
	}
}

static ssize_t capsense_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", this->hw->dbg_name);
}

static ssize_t capsense_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	u32 temp = 0;
	psx93XX_t this = dev_get_drvdata(dev);
	sx937x_i2c_read_16bit(this->bus, SX937X_GENERAL_SETUP, &temp);
	if (!count)
		return -EINVAL;

	if (!strncmp(buf, "reset", 5) || !strncmp(buf, "1", 1)) {
		if (temp & 0x000000FF) {
			LOG_DBG("Going to refresh baseline\n");
			manual_offset_calibration(this->hw);
		}
	}

	return count;
}

static ssize_t capsense_raw_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	char *p = buf;
	int csx;
	s32 useful, average, diff;
	u32 uData;
	u16 offset;

	psx93XX_t this = dev_get_drvdata(dev);
	if(this) {
		for(csx =0; csx<8; csx++) {
			sx937x_i2c_read_16bit(this->bus, SX937X_USEFUL_PH0 + csx*4, &uData);
			useful = (s32)uData>>10;
			sx937x_i2c_read_16bit(this->bus, SX937X_AVERAGE_PH0 + csx*4, &uData);
			average = (s32)uData>>10;
			sx937x_i2c_read_16bit(this->bus, SX937X_DIFF_PH0 + csx*4, &uData);
			diff = (s32)uData>>10;
			sx937x_i2c_read_16bit(this->bus, SX937X_OFFSET_PH0 + csx*12, &uData);
			offset = (u16)(uData & 0x3FFF);
			p += snprintf(p, PAGE_SIZE, "PH= %d Useful = %d Average = %d DIFF = %d Offset = %d \n",
					csx,useful,average,diff,offset);
		}
	}
	return (p-buf);
}

static ssize_t sx937x_register_write_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 reg_address = 0, val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf, "%x,%x", &reg_address, &val) != 2)
	{
		LOG_ERR("The number of data are wrong\n");
		return -EINVAL;
	}

	sx937x_i2c_write_16bit(this->bus, reg_address, val);

	LOG_DBG("%s Register(0x%x) data(0x%x)\n", this->hw->dbg_name, reg_address, val);
	return count;
}



static ssize_t sx937x_register_read_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int sx937x_temp_regist = 0;
	u32 sx937x_temp_val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf, "%x", &sx937x_temp_regist) != 1)
	{
		LOG_ERR(" The number of data are wrong\n");
		return -EINVAL;
	}

	sx937x_i2c_read_16bit(this->bus, sx937x_temp_regist, &sx937x_temp_val);

	LOG_DBG("%s Register(0x%2x) data(0x%4x)\n", this->hw->dbg_name, sx937x_temp_regist, sx937x_temp_val);
	return count;
}

static ssize_t sx937x_irq_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	LOG_INFO("%s Reading INT line state %d\n", this->hw->dbg_name, this->int_state);
	return sprintf(buf, "%d\n", this->int_state);
}

static ssize_t sx937x_fac_detect_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	u32 reg_value = 0;
	char temp_id[7] ="0";
	char sx93x_id[4] ="0";
	sx937x_i2c_read_16bit(this->bus,SX937X_DEVICE_INFO,&reg_value);
	LOG_INFO("Reading device id reg_value==%x\n",reg_value);
	sprintf(temp_id, "%x\n",reg_value);
	strncpy(sx93x_id,temp_id,sizeof(sx93x_id)-1);
	if(strcmp(sx93x_id,"937")==0){

		LOG_INFO("Detect ic sx937x\n");
		return sprintf(buf, "%d\n",1);

	}else{

		LOG_INFO("Not found ic sx937x\n");
		return sprintf(buf, "%d\n",0);
	}
}

static ssize_t sx937x_fac_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	psx93XX_t this = dev_get_drvdata(dev);
	u32 temp = 0;
	int ret = 0;
	if ( !strncmp(buf, "1", 1)) {
		LOG_INFO("enable cap sensor\n");
				sx937x_i2c_read_16bit(this->bus, SX937X_GENERAL_SETUP, &temp);
				temp = temp | 0x0000007F;
				LOG_INFO("set reg 0x%x val 0x%x\n", SX937X_GENERAL_SETUP, temp);
				sx937x_i2c_write_16bit(this->bus, SX937X_GENERAL_SETUP, temp);
				if(ret <0){
					LOG_ERR("enable write enable sx937x error ret =%d\n",ret);
					return -EINVAL;
				}
	}
	if ( !strncmp(buf, "0", 1)) {
		LOG_INFO("disnable cap sensor\n");
				sx937x_i2c_read_16bit(this->bus, SX937X_GENERAL_SETUP, &temp);
				temp = temp | 0xFFFFFF00;
				LOG_INFO("set reg 0x%x val 0x%x\n", SX937X_GENERAL_SETUP, temp);
				sx937x_i2c_write_16bit(this->bus, SX937X_GENERAL_SETUP, temp);
				if(ret <0){
					LOG_ERR("enable write enable sx937x error ret =%d\n",ret);
					return -EINVAL;
				}
	}
	return count;
}

static ssize_t sx937x_fac_cal_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	psx93XX_t this = dev_get_drvdata(dev);
	LOG_INFO("%s sx937x_fac_cal_store\n", this->hw->dbg_name);
	if ( !strncmp(buf, "1", 1)) {
		manual_offset_calibration(this->hw);
		LOG_INFO("set reg 0x%x val 0xe\n", SX937X_COMMAND);
	}

	return count;
}
static ssize_t sx937x_fac_comp_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	int i ,j= 0;
	int read_ret;
	u8 reg_data[MAX_CHANNEL_NUMBER*4+2] = {0};
	u16 reg_addr;
	u32 temp_val;
	reg_addr = SX937X_OFFSET_PH0;

	LOG_INFO("%s sx937x_fac_comp_show\n", this->hw->dbg_name);

	for (i = 0; i < MAX_CHANNEL_NUMBER; i++) {
		read_ret = sx937x_i2c_read_16bit(this->bus,reg_addr+0xc*i,&temp_val);
		if(read_ret<0){
			LOG_INFO("failed to read reg data 0x%x", reg_addr);
		}
		LOG_INFO("fac_cam i==%d reg_addr ==0x%x temp_val===%x\n",i,reg_addr+0xc*i,temp_val);
		reg_data[2 * i] = (u8)((temp_val & 0x3FFF) >> 8);
               reg_data[1 + 2 * i] = (u8)(temp_val & 0x3FFF);
		LOG_INFO("tc_cmn_drv_cap_sense_read_cal_data reg[%d]==%x,reg[%d]==%x\n",
					(2*i), reg_data[2 * i],
					(1+2*i), reg_data[1 + 2 * i]);
	}
	for(j=0;j<sizeof(reg_data);j++){
		buf[j] = reg_data[j];
	}
	return sizeof(reg_data);
}

static ssize_t sx937x_fac_raw_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	u16 reg_addr;
        u32 temp_val;
        int diff_val;
	int read_ret;
	int i;
	u8 data[MAX_CHANNEL_NUMBER*4] = {0};
        reg_addr = SX937X_DIFF_PH0;

	LOG_INFO("%s sx937x_fac_raw_show\n", this->hw->dbg_name);
	for ( i = 0; i < MAX_CHANNEL_NUMBER; i++) {
		read_ret=sx937x_i2c_read_16bit(this->bus,reg_addr+0xc*i,&temp_val);
		if(read_ret<0){
			LOG_INFO("failed to read reg data 0x%x", reg_addr);
		}
		LOG_INFO("sx937x i==%d,reg_addr:0x%x",i, reg_addr + 0x4 * i);
		diff_val = ((int)temp_val)>>10;
		data[4 * i] = (u8)(diff_val >> 24);
		data[1 + 4 * i] = (u8)(diff_val >> 16);
		data[2 + 4 * i] = (u8)(diff_val >> 8);
		data[3 + 4 * i] = (u8)(diff_val);
	 	LOG_INFO("sx937x diff_val==%x,data[%d]==%x,data[%d]==%x,data[%d]==%x,data[%d]==%x",
	 				diff_val,
	 				(4*i), data[4 * i],
	 				(1+4*i), data[1+4 * i],
	 				(2+4*i), data[2+4 * i],
	 				(3+4*i), data[3+4 * i]);
        }
	for(i=0;i<sizeof(data);i++){
		buf[i] = data[i];
	}
	return sizeof(data);
}

static DEVICE_ATTR(name, 0444, capsense_name_show, NULL);
static DEVICE_ATTR(reset, 0220, NULL, capsense_reset_store);
static DEVICE_ATTR(raw_data, 0444, capsense_raw_data_show, NULL);
static DEVICE_ATTR(register_write, 0220, NULL, sx937x_register_write_store);
static DEVICE_ATTR(register_read, 0660, NULL, sx937x_register_read_store);
static DEVICE_ATTR(fac_irq_state, 0444, sx937x_irq_state_show, NULL);
static DEVICE_ATTR(fac_detect, 0444, sx937x_fac_detect_show, NULL);
static DEVICE_ATTR(fac_enable, 0444, NULL, sx937x_fac_enable_store);
static DEVICE_ATTR(fac_cal, 0444, NULL, sx937x_fac_cal_store);
static DEVICE_ATTR(fac_compensation, 0444, sx937x_fac_comp_show, NULL);
static DEVICE_ATTR(fac_raw, 0444, sx937x_fac_raw_show, NULL);

static struct device_attribute *capsense_class_attrs[] = {
	&dev_attr_name,
	&dev_attr_reset,
	&dev_attr_raw_data,
	&dev_attr_register_write,
	&dev_attr_register_read,
	&dev_attr_fac_irq_state,
	&dev_attr_fac_detect,
	&dev_attr_fac_enable,
	&dev_attr_fac_cal,
	&dev_attr_fac_compensation,
	&dev_attr_fac_raw
};

/**************************************/

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct
 */
static void sx937x_reg_init(psx93XX_t this)
{
	psx937x_platform_data_t pdata = 0;
	int i = 0;
	uint32_t tmpvalue;
	/* configure device */
	if (this && (pdata = this->hw))
	{
		/*******************************************************************************/
		// try to initialize from device tree!
		/*******************************************************************************/
		while ( i < ARRAY_SIZE(sx937x_i2c_reg_setup))
		{
			/* Write all registers/values contained in i2c_reg */
			LOG_DBG("Going to Write Reg: 0x%x Value: 0x%x\n",
					sx937x_i2c_reg_setup[i].reg,sx937x_i2c_reg_setup[i].val);
			tmpvalue = sx937x_i2c_reg_setup[i].val;
			if (sx937x_i2c_reg_setup[i].reg == SX937X_GENERAL_SETUP)
			{
				if((sx937x_i2c_reg_setup[i].val & 0xFF) == 0)
				{
					tmpvalue = (sx937x_i2c_reg_setup[i].val|0xFF);
				}
			}
			sx937x_i2c_write_16bit(this->bus, sx937x_i2c_reg_setup[i].reg, tmpvalue);
			i++;
		}

		if (this->reg_in_dts == true)
		{
			i = 0;
			while ( i < pdata->i2c_reg_num)
			{
				/* Write all registers/values contained in i2c_reg */
				LOG_DBG("Going to Write Reg from dts: 0x%x Value: 0x%x\n",
						pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
				sx937x_i2c_write_16bit(this->bus, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
				i++;
			}
		}

		/*******************************************************************************/
		sx937x_i2c_write_16bit(this->bus, SX937X_COMMAND, 0xF);  //enable phase control
	}
	else {
		LOG_ERR("ERROR! platform data 0x%p\n",this->hw);
	}
}


/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
	int ret, retry;
	if (this)
	{
		LOG_INFO("SX937x income initialize\n");
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		for ( retry = 10; retry > 0; retry-- ) {
			if (sx937x_i2c_write_16bit(this->bus, SX937X_DEVICE_RESET, 0xDE) >= 0){
				LOG_DBG("write ok");
				break;
			}
			LOG_DBG("SX937x write SX937X_RESET_REG retry:%d\n", 11 - retry);
			msleep(10);
		}
		/* wait until the reset has finished by monitoring NIRQ */
		LOG_INFO("Sent Software Reset. Waiting until device is back from reset to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(100);
		ret = sx937x_global_variable_init(this);
		sx937x_reg_init(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		read_regStat(this);
		return 0;
	}
	return -ENOMEM;
}

/*!
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct
 */
static void touchProcess(psx93XX_t this)
{
	int counter = 0;
	u32 i = 0;
	u32 touchFlag = 0;
	int numberOfButtons = 0;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;

	struct _buttonInfo *pCurrentButton  = NULL;

	if (this)
	{
		sx937x_i2c_read_16bit(this->bus, SX937X_DEVICE_STATUS_A, &i);
		LOG_DBG("touchProcess STAT0_REG:0x%08x\n", i);

		buttons = this->hw->buttons;
		numberOfButtons = this->hw->buttonSize;

		if (unlikely( buttons==NULL ))
		{
			LOG_ERR(":ERROR!! buttons NULL!!!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++)
		{
			if (buttons[counter].enabled == false) {
				LOG_DBG("touchProcess %s disabled, ignor this\n", buttons[counter].name);
				continue;
			}
			if (buttons[counter].used== false) {
				LOG_DBG("touchProcess %s unused, ignor this\n", buttons[counter].name);
				continue;
			}
			pCurrentButton = &buttons[counter];
			if (pCurrentButton==NULL)
			{
				LOG_ERR("ERROR!! current button at index: %d NULL!!!\n", counter);
				return; // ERRORR!!!!
			}
			input = pCurrentButton->input_dev;
			if (input==NULL)
			{
				LOG_ERR("ERROR!! current button input at index: %d NULL!!!\n", counter);
				return; // ERRORR!!!!
			}

			touchFlag = i & (pCurrentButton->ProxMask | pCurrentButton->BodyMask);
			if (touchFlag == (pCurrentButton->ProxMask | pCurrentButton->BodyMask)) {
				if (pCurrentButton->state == BODYACTIVE)
					LOG_DBG(" %s already BODYACTIVE\n", pCurrentButton->name);
				else {
					input_report_abs(input, ABS_DISTANCE, 2);
					input_sync(input);
					pCurrentButton->state = BODYACTIVE;
					LOG_DBG(" %s report 5mm BODYACTIVE\n", pCurrentButton->name);
				}
			} else if (touchFlag == pCurrentButton->ProxMask) {
				if (pCurrentButton->state == PROXACTIVE)
					LOG_DBG(" %s already PROXACTIVE\n", pCurrentButton->name);
				else {
					input_report_abs(input, ABS_DISTANCE, 1);
					input_sync(input);
					pCurrentButton->state = PROXACTIVE;
					LOG_DBG(" %s report 15mm PROXACTIVE\n", pCurrentButton->name);
				}
			}else if (touchFlag == 0) {
				if (pCurrentButton->state == IDLE)
					LOG_DBG("%s already released.\n", pCurrentButton->name);
				else {
					input_report_abs(input, ABS_DISTANCE, 0);
					input_sync(input);
					pCurrentButton->state = IDLE;
					LOG_DBG("%s report  released.\n", pCurrentButton->name);
				}
			}
		}
		LOG_INFO("Leaving touchProcess()\n");
	}
}

static int sx937x_parse_dts(struct sx937x_platform_data *pdata, struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	enum of_gpio_flags flags;
	int i, rc;
	const char *reg_group_name = "Semtech,reg-init";
	int name_index,name_count;
	if (dNode == NULL)
		return -ENODEV;

	rc = of_property_read_u32(dNode,"Semtech,power-supply-type",&pdata->power_supply_type);
	if(rc < 0){
		pdata->power_supply_type = SX937X_POWER_SUPPLY_TYPE_PMIC_LDO;
		LOG_INFO("pmic ldo is the default if not set power-supply-type in dt\n");
	}

	switch(pdata->power_supply_type){
		case SX937X_POWER_SUPPLY_TYPE_PMIC_LDO:
			/* using regulator_get() to fetch power_supply in sx937x_probe()*/
			break;
		case SX937X_POWER_SUPPLY_TYPE_ALWAYS_ON:
			/* power supply always on: no need fetch others control  in drivers */
			break;
		case SX937X_POWER_SUPPLY_TYPE_EXTERNAL_LDO:
			/* parse the gpio number for external LDO enable pin*/
			pdata->eldo_gpio = of_get_named_gpio_flags(dNode,
					"Semtech,eldo-gpio",0,&flags);
			LOG_INFO("used eLDO_gpio 0x%x \n", pdata->eldo_gpio);
			break;
		default:
			LOG_INFO("Error power_supply_type: 0x%x \n", pdata->power_supply_type);
			break;
	}

	pdata->irq_gpio= of_get_named_gpio_flags(dNode,"Semtech,nirq-gpio", 0, &flags);
	if (pdata->irq_gpio < 0){
		LOG_ERR("get irq_gpio error\n");
		return -ENODEV;
	}

	of_property_read_string(dNode, "Semtech,debug-name", &pdata->dbg_name);
	LOG_INFO("used dbg_name %s\n", pdata->dbg_name);

	pdata->button_used_flag = 0;
	of_property_read_u32(dNode,"Semtech,button-flag",&pdata->button_used_flag);
	LOG_INFO("used button 0x%x \n", pdata->button_used_flag);

	name_count = of_property_count_strings(dNode, "Semtech,button-names");
	LOG_INFO("name_count %d pdata->buttonSize==%d\n", name_count, pdata->buttonSize);
	for (i = 0, name_index = 0; i < pdata->buttonSize & name_index < name_count; i++)
	{
		if (pdata->button_used_flag>>i & 0x01) {
			pdata->buttons[i].used = true;
			of_property_read_string_index(dNode,"Semtech,button-names",
					name_index, &pdata->buttons[i].name);
			name_index++;
		}
	}
	pdata->ref_phase_a = -1;
	pdata->ref_phase_b = -1;
	pdata->ref_phase_c = -1;
	if ( of_property_read_u32(dNode,"Semtech,ref-phases-a",&pdata->ref_phase_a) )
	{
		LOG_ERR("[SX937x]: %s - get ref-phases error\n", __func__);
		return -ENODEV;
	}
	if ( of_property_read_u32(dNode,"Semtech,ref-phases-b",&pdata->ref_phase_b) )
	{
		LOG_ERR("[SX937x]: %s - get ref-phases-b error\n", __func__);
		return -ENODEV;
	}
	if ( of_property_read_u32(dNode,"Semtech,ref-phases-c",&pdata->ref_phase_c) )
	{
		LOG_ERR("[SX937x]: %s - get ref-phases-c error\n", __func__);
		return -ENODEV;
	}
	if (pdata->ref_phase_a == 0xff) pdata->ref_phase_a = -1;
	if (pdata->ref_phase_b == 0xff) pdata->ref_phase_b = -1;
	if (pdata->ref_phase_c == 0xff) pdata->ref_phase_c = -1;

	LOG_INFO("[SX937x]: %s ref_phase_a= %d ref_phase_b= %d ref_phase_c= %d\n",
			__func__, pdata->ref_phase_a, pdata->ref_phase_b, pdata->ref_phase_c);

	// load in registers from device tree
	of_property_read_u32(dNode,"Semtech,reg-num",&pdata->i2c_reg_num);
	// layout is register, value, register, value....
	// if an extra item is after just ignore it. reading the array in will cause it to fail anyway
	LOG_INFO("size of elements %d \n", pdata->i2c_reg_num);
	if (pdata->i2c_reg_num > 0)
	{
		// initialize platform reg data array
		pdata->pi2c_reg = devm_kzalloc(dev,sizeof(struct smtc_reg_data)*pdata->i2c_reg_num, GFP_KERNEL);
		if (unlikely(pdata->pi2c_reg == NULL))
		{
			LOG_ERR("size of elements %d alloc error\n", pdata->i2c_reg_num);
			return -ENOMEM;
		}

		// initialize the array
		if (of_property_read_u32_array(dNode,reg_group_name,(u32*)&(pdata->pi2c_reg[0]),sizeof(struct smtc_reg_data)*pdata->i2c_reg_num/sizeof(u32)))
			return -ENOMEM;
	}

	LOG_INFO("-[%d] parse_dt complete\n", pdata->irq_gpio);
	return 0;
}

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx937x_init_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx937x_platform_data *pdata = NULL;

	int rc = 0;

	LOG_INFO("init_platform_hw start!");

	if (this &&  (pdata = this->hw))
	{
		if (gpio_is_valid(pdata->irq_gpio))
		{
			rc = gpio_request(pdata->irq_gpio, "sx937x_irq_gpio");
			if (rc < 0)
			{
				LOG_ERR("SX937x Request gpio. Fail![%d]\n", rc);
				return rc;
			}
			rc = gpio_direction_input(pdata->irq_gpio);
			if (rc < 0)
			{
				LOG_ERR("SX937x Set gpio direction. Fail![%d]\n", rc);
				return rc;
			}
			this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
		}
		else
		{
			LOG_ERR("SX937x Invalid irq gpio num.(init)\n");
		}
	}
	else
	{
		LOG_ERR("Do not init platform HW");
	}
	return rc;
}

static void sx937x_exit_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx937x_platform_data *pdata = NULL;

	if (this && (pdata = this->hw))
	{
		if (gpio_is_valid(pdata->irq_gpio))
		{
			gpio_free(pdata->irq_gpio);
		}
		else
		{
			LOG_ERR("Invalid irq gpio num.(exit)\n");
		}
	}
	return;
}
static int capsensor_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	psx93XX_t this = (psx93XX_t)(sensors_cdev->dev->platform_data);
	bool disableFlag = true;
	int i = 0;
	u32 temp = 0x0;
	struct _buttonInfo *buttons = this->hw->buttons;
	int buttonSize = this->hw->buttonSize;

	for (i = 0; i < buttonSize; i++) {
		if (strcmp(sensors_cdev->name, buttons[i].name) == 0) {
			if (enable == 1) {
				LOG_INFO("enable cap sensor : %s\n", sensors_cdev->name);
				sx937x_i2c_read_16bit(this->bus, SX937X_GENERAL_SETUP, &temp);
				temp = temp | 0x0000007F;
				LOG_DBG("set reg 0x%x val 0x%x\n", SX937X_GENERAL_SETUP, temp);
				sx937x_i2c_write_16bit(this->bus, SX937X_GENERAL_SETUP, temp);
				buttons[i].enabled = true;
				input_report_abs(buttons[i].input_dev, ABS_DISTANCE, 0);
				input_sync(buttons[i].input_dev);

				manual_offset_calibration(this->hw);
			} else if (enable == 0) {
				LOG_INFO("disable cap sensor : %s\n", sensors_cdev->name);
				buttons[i].enabled = false;
				input_report_abs(buttons[i].input_dev, ABS_DISTANCE, -1);
				input_sync(buttons[i].input_dev);
			} else {
				LOG_ERR("unknown enable symbol\n");
			}
		}
	}

	//if all chs disabled, then disable all
	for (i = 0; i < buttonSize; i++) {
		if (buttons[i].enabled) {
			disableFlag = false;
			break;
		}
	}
	if (disableFlag) {
		LOG_DBG("disable all chs\n");
		sx937x_i2c_read_16bit(this->bus, SX937X_GENERAL_SETUP, &temp);
		LOG_DBG("read reg 0x%x val 0x%x\n", SX937X_GENERAL_SETUP, temp);
		temp = temp & 0xFFFFFF00;
		LOG_DBG("set reg 0x%x val 0x%x\n", SX937X_GENERAL_SETUP, temp);
		sx937x_i2c_write_16bit(this->bus, SX937X_GENERAL_SETUP, temp);
	}
	return 0;
}

/*! \fn static int sx937x_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx937x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	int err = 0;

	psx93XX_t this = NULL;
	psx937x_platform_data_t pplatData = NULL;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	LOG_INFO("sx937x_probe enter\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
	{
		LOG_ERR("Check i2c functionality.Fail!\n");
		return -EIO;
	}

	this = devm_kzalloc(&client->dev,sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
	if (!this)
	{
		LOG_ERR("Failed to allocate memory(sx93XX_t)\n");
		return -ENOMEM;
	}

	pplatData = devm_kzalloc(&client->dev,sizeof(sx937x_platform_data_t), GFP_KERNEL);
	if (!pplatData)
	{
		LOG_ERR("platform data is required!\n");
		return -EINVAL;
	}

	pplatData->buttonSize = ARRAY_SIZE(psmtcButtons);
	memcpy(pplatData->buttons, psmtcButtons, sizeof(psmtcButtons));

	client->dev.platform_data = pplatData;
	err = sx937x_parse_dts(pplatData, &client->dev);
	if (err)
	{
		LOG_ERR("could not setup pin\n");
		return ENODEV;
	}

	pplatData->init_platform_hw = sx937x_init_platform_hw;

	if (this)
	{
		LOG_INFO("SX937x initialize start!!");
		/* In case we need to reinitialize data
		 * (e.q. if suspend reset device) */
		this->hw = pplatData;

		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown
		 * (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = sx937x_get_nirq_low;
		/* save irq in case we need to reference it */
		//this->irq = client->irq;
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8)
		{
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = touchProcess; /* body&table */
			this->statusFunc[3] = read_rawData; /* CONV_STAT */
			this->statusFunc[4] = touchProcess; /* COMP_STAT */
			this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
			this->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		pplatData->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		this->dbg_dev = device_create(capsense_class, NULL, MKDEV(0,0), this, "%s", pplatData->dbg_name);
		if (IS_ERR(this->dbg_dev))
		{
			LOG_ERR("capsense_class dbg dev create fail\n");
			return PTR_ERR(this->dbg_dev);;
		}
		LOG_INFO("this->dbg_dev ok\n");
		for (i = 0; i < ARRAY_SIZE(capsense_class_attrs); ++i) {
			err = device_create_file(this->dbg_dev, capsense_class_attrs[i]);
			if (err)
				LOG_ERR("i = %d,device_create_file register fail\n",i);
		}

		/* Check if we hava a platform initialization function to call*/
		if (pplatData->init_platform_hw)
			pplatData->init_platform_hw(client);

		for (i = 0; i < pplatData->buttonSize; i++)
		{
			if (pplatData->button_used_flag>>i & 0x01) {
				pplatData->buttons[i].used = true;
				pplatData->buttons[i].state = IDLE;
				pplatData->buttons[i].input_dev = input_allocate_device();
				if (!pplatData->buttons[i].input_dev)
					return -ENOMEM;
				pplatData->buttons[i].input_dev->name = pplatData->buttons[i].name;
				/* Set all the keycodes */
				__set_bit(EV_ABS, pplatData->buttons[i].input_dev->evbit);
				input_set_abs_params(pplatData->buttons[i].input_dev, ABS_DISTANCE, -1, 100, 0, 0);

				err = input_register_device(pplatData->buttons[i].input_dev);
				/* report a unused val, then first val will report after enable */
				input_report_abs(pplatData->buttons[i].input_dev, ABS_DISTANCE, -1);
				input_sync(pplatData->buttons[i].input_dev);

				pplatData->buttons[i].sensors_capsensor_cdev.sensors_enable = capsensor_set_enable;
				pplatData->buttons[i].sensors_capsensor_cdev.sensors_poll_delay = NULL;
				pplatData->buttons[i].sensors_capsensor_cdev.name = pplatData->buttons[i].name;
				pplatData->buttons[i].sensors_capsensor_cdev.vendor = "semtech";
				pplatData->buttons[i].sensors_capsensor_cdev.version = 1;
				pplatData->buttons[i].sensors_capsensor_cdev.type = SENSOR_TYPE_MOTO_CAPSENSE;
				pplatData->buttons[i].sensors_capsensor_cdev.max_range = "5";
				pplatData->buttons[i].sensors_capsensor_cdev.resolution = "5.0";
				pplatData->buttons[i].sensors_capsensor_cdev.sensor_power = "3";
				pplatData->buttons[i].sensors_capsensor_cdev.min_delay = 0;
				pplatData->buttons[i].sensors_capsensor_cdev.fifo_reserved_event_count = 0;
				pplatData->buttons[i].sensors_capsensor_cdev.fifo_max_event_count = 0;
				pplatData->buttons[i].sensors_capsensor_cdev.delay_msec = 100;
				pplatData->buttons[i].sensors_capsensor_cdev.enabled = 0;
				pplatData->buttons[i].enabled = false;
				err = sensors_classdev_register(&pplatData->buttons[i].input_dev->dev, &pplatData->buttons[i].sensors_capsensor_cdev);
				if (err < 0)
					LOG_ERR("create %d cap sensor_class  file failed (%d)\n", i, err);
				pplatData->buttons[i].sensors_capsensor_cdev.dev->platform_data = this;

			}
		}

		switch(pplatData->power_supply_type){
			case SX937X_POWER_SUPPLY_TYPE_PMIC_LDO:
				pplatData->cap_vdd = regulator_get(&client->dev, "cap_vdd");
				if (IS_ERR(pplatData->cap_vdd)) {
					if (PTR_ERR(pplatData->cap_vdd) == -EPROBE_DEFER) {
						err = PTR_ERR(pplatData->cap_vdd);
						return err;
					}
					LOG_INFO("Failed to get regulator\n");
				} else {
					LOG_INFO("with cap_vdd\n");
					err = regulator_enable(pplatData->cap_vdd);
					if (err) {
						regulator_put(pplatData->cap_vdd);
						LOG_ERR("Error %d enable regulator\n",
								err);
						return err;
					}
					pplatData->cap_vdd_en = true;
					LOG_INFO("cap_vdd regulator is %s\n",
							regulator_is_enabled(pplatData->cap_vdd) ?
							"on" : "off");
				}
				break;
			case SX937X_POWER_SUPPLY_TYPE_ALWAYS_ON:
				LOG_INFO("using always on power supply\n");
				break;
			case SX937X_POWER_SUPPLY_TYPE_EXTERNAL_LDO:
				LOG_INFO("enable external LDO, en_gpio:%d\n",
						pplatData->eldo_gpio);
				err = gpio_request(pplatData->eldo_gpio, "sx937x_eldo_gpio");
				if (err < 0){
					LOG_ERR("SX937x Request eLDO gpio. Fail![%d]\n", err);
					return err;
				}
				err = gpio_direction_output(pplatData->eldo_gpio,1);
				if(err < 0){
					LOG_ERR("can not enable external LDO,%d", err);
					return err;
				}
				pplatData->eldo_vdd_en = true;
				msleep(20);
				break;
		}
		sx93XX_IRQ_init(this);
		/* call init function pointer (this should initialize all registers */
		if (this->init) {
			this->init(this);
		}
		else {
			LOG_ERR("No init function!!!!\n");
			return -ENOMEM;
		}
	} else {
		return -1;
	}

	pplatData->exit_platform_hw = sx937x_exit_platform_hw;

	if (sx937x_Hardware_Check(this) != 0) {
		LOG_ERR("sx937x_Hardware_CheckFail!\n");
		if (this->failStatusCode & SX937x_I2C_ERROR) {
			LOG_ERR("I2C error, delete sensor");
			goto Hardware_CheckFail;
		}
	}

	LOG_INFO("sx937x_probe() Done\n");
	return 0;

Hardware_CheckFail:
	if (this->dbg_dev) {
		device_unregister(this->dbg_dev);
		this->dbg_dev = NULL;
	}

	for (i = 0; i < pplatData->buttonSize; i++) {
		if (pplatData->buttons[i].used == true) {
			sensors_classdev_unregister(&(pplatData->buttons[i].sensors_capsensor_cdev));
			input_unregister_device(pplatData->buttons[i].input_dev);
			pplatData->buttons[i].used = false;
		}
	}
	if (pplatData->cap_vdd_en) {
		regulator_disable(pplatData->cap_vdd);
		regulator_put(pplatData->cap_vdd);
		pplatData->cap_vdd_en = false;
	}
	if(pplatData->eldo_vdd_en){
		gpio_direction_output(pplatData->eldo_gpio,0);
		pplatData->eldo_vdd_en = false;
	}

	return -ENODEV;
}
/*! \fn static int sx937x_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value 0
 */
static int sx937x_remove(struct i2c_client *client)
{
	psx937x_platform_data_t pplatData =0;
	struct _buttonInfo *pCurrentbutton;
	int i = 0;
	psx93XX_t this = i2c_get_clientdata(client);
	LOG_DBG("sx937x_remove");
	if (this)
	{
		pplatData = client->dev.platform_data;
		free_irq(this->irq, this);
		cancel_delayed_work_sync(&(this->dworker));

		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw(client);

		if (pplatData->cap_vdd_en) {
			regulator_disable(pplatData->cap_vdd);
			regulator_put(pplatData->cap_vdd);
		}

		if (this->dbg_dev)
			device_unregister(this->dbg_dev);

		if(pplatData->eldo_vdd_en){
			gpio_direction_output(pplatData->eldo_gpio,0);
		}
		for (i = 0; i < pplatData->buttonSize; i++) {
			pCurrentbutton = &(pplatData->buttons[i]);
			if (pCurrentbutton->used == true) {
				sensors_classdev_unregister(&(pCurrentbutton->sensors_capsensor_cdev));
				input_unregister_device(pCurrentbutton->input_dev);
			}
		}
	}
	return 0;
}
/*====================================================*/
/***** Kernel Suspend *****/
static int sx937x_suspend(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);

	if (this) {

		sx937x_i2c_write_16bit(this->bus,SX937X_COMMAND,0xD);//make sx937x in Sleep mode
		LOG_DBG(LOG_TAG "sx937x suspend:disable irq!\n");
		disable_irq(this->irq);
	}
	return 0;
}
/***** Kernel Resume *****/
static int sx937x_resume(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	//psx937x_platform_data_t pdata = 0;

	if (this) {
		LOG_INFO(LOG_TAG "sx937x resume:enable irq!\n");
		sx93XX_schedule_work(this,0);
		enable_irq(this->irq);
		sx937x_i2c_write_16bit(this->bus,SX937X_COMMAND,0xC);//Exit from Sleep mode
	}
	return 0;
}
/*====================================================*/


static struct i2c_device_id sx937x_idtable[] =
{
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx937x_idtable);

static struct of_device_id sx937x_match_table[] =
{
	{ .compatible = "Semtech,sx937x",},
	{ },
};

static const struct dev_pm_ops sx937x_pm_ops =
{
	.suspend = sx937x_suspend,
	.resume = sx937x_resume,
};

static struct i2c_driver sx937x_driver =
{
	.driver = {
		.owner			= THIS_MODULE,
		.name			= DRIVER_NAME,
		.of_match_table	= sx937x_match_table,
		.pm				= &sx937x_pm_ops,
	},
	.id_table		= sx937x_idtable,
	.probe			= sx937x_probe,
	.remove			= sx937x_remove,
};
static int __init sx937x_I2C_init(void)
{
	capsense_class = class_create(THIS_MODULE, "capsense");
	if (IS_ERR(capsense_class))
	{
		printk("sensor init fail!\n");
		return PTR_ERR(capsense_class);
	}
	return i2c_add_driver(&sx937x_driver);
}
static void __exit sx937x_I2C_exit(void)
{
	i2c_del_driver(&sx937x_driver);
	class_destroy(capsense_class);
}

module_init(sx937x_I2C_init);
module_exit(sx937x_I2C_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX937x Capacitive Proximity Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");
