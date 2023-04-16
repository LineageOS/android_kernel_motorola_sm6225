/*
* drivers/sensors/dts201a.c
*
* Partron DTS201A Thermopile Sensor module driver
*
* Copyright (C) 2013 Partron Co., Ltd. - Sensor Lab.
* partron (partron@partron.co.kr)
*
* Both authors are willing to be considered the contact and update points for
* the driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 1.0.0 2013/Aug/08:
    first release
 Revision 1.0.1 2017/Nov/30:
  update for sysfs raw data and coefficient
 Revision 2.0.0 2020/Jul/03:
  update for coefficient read and add the emissivity caculation equation
******************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>

#include "dts201a-cci.h"
//do not use kernel PM
#undef CONFIG_PM

#ifdef CONFIG_DRM
	#include <linux/msm_drm_notify.h>
#endif
//#undef DTS201A_DEBUG
#define	DTS201A_DEBUG
#define	DTS201A_TEST

#ifdef DTS201A_MAVG
static u8 o_avg_cnt = 0;
static u8 a_avg_cnt = 0;

#endif

#ifndef SENSOR_TYPE_MOTO_BODYTEMP
#define SENSOR_TYPE_MOTO_BODYTEMP (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 42)
#endif

static struct dts201a_data *g_dts201a_data  = NULL;

static u8  i2c_mavg = 0;
static int dts201a_power_on(struct dts201a_data *ther, bool on );
static int dts201a_read_coefficient_reg( struct dts201a_data *ther );
#define	USE_SENSORS_CLASS
struct sensors_classdev  sensors_thermopile_cdev = {
	.vendor = "partron",
	.version = 1,
	.type = SENSOR_TYPE_MOTO_BODYTEMP,
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
#ifdef CAMERA_CCI
static int dts201a_cci_read(struct dts201a_data *dev, uint32_t addr,  u8* data,  u8 len)
{
	int rc = 0;
	struct cci_ctrl_t *cci_ctrl         = NULL;

	cci_ctrl = (struct cci_ctrl_t *)dev->client_object;
	//use custom addr type to skip addr set during reading
	rc = cam_camera_cci_i2c_read_seq(cci_ctrl->io_master_info.cci_client, addr, data, CAMERA_SENSOR_I2C_TYPE_CUSTOM,
				CAMERA_SENSOR_I2C_TYPE_BYTE, len);
	return rc != 0;
}
static int dts201a_cci_write(struct dts201a_data *dev, uint32_t addr,
		uint8_t *data, uint16_t len,uint  delay){
	int i = 0, rc = 0;
	struct cam_sensor_i2c_reg_setting  write_reg_setting;
	struct cam_sensor_i2c_reg_array    *reg_array  = NULL;
	struct cci_ctrl_t *cci_ctrl         = NULL;

	cci_ctrl = (struct cci_ctrl_t *)dev->client_object;

	reg_array = kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * len, GFP_KERNEL);
	if (!reg_array) {
		dts201a_errmsg("cci_write malloc failed\n");
	}

	for (i = 0; i < len; i++) {
		reg_array[i].reg_addr = addr + i;
		reg_array[i].reg_data = data[i];
	}

	write_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_reg_setting.size      = len;
	write_reg_setting.reg_setting = reg_array;
	write_reg_setting.delay     = delay;

	rc = cam_cci_i2c_write_table(&cci_ctrl->io_master_info, &write_reg_setting);

	kfree(reg_array);
	return rc;

}
#else

static int dts201a_i2c_read(struct dts201a_data *ther,
				  u8 *buf, u8 len)
{

	struct i2c_msg msg[1];
	int err = 0;
	int retry = 3;

	if ((ther->client == NULL) || (!ther->client->adapter))
		return -ENODEV;
	mutex_lock( &ther->dts201a_i2c_mutex );

	msg->addr = ther->client->addr;
	//msg->addr = DTS201A_SADDR;
	msg->flags = DTS_I2C_READ;
	msg->len = len;
	msg->buf = buf;

	while (retry--) {
		err = i2c_transfer(ther->client->adapter, msg, 1);
		if (err >= 0) {
		mutex_unlock( &ther->dts201a_i2c_mutex );
			return err;
		}
	}
	mutex_unlock( &ther->dts201a_i2c_mutex );
	pr_err("%s: i2c read failed at addr 0x%x: %d\n", __func__, msg->addr, err);

	return err;

}


static int dts201a_i2c_write(struct dts201a_data *ther,
				u8 cmd, u16 bm_config)
{

	struct i2c_msg msg[1];
	int err = 0;
	int retry = 3;
	u8 wdata[3]={0, };

	if ((ther->client == NULL) || (!ther->client->adapter))
		return -ENODEV;
	mutex_lock( &ther->dts201a_i2c_mutex );

	wdata[0] = cmd;
	wdata[1] = (u8)(bm_config>>8);
	wdata[2] = (u8)(bm_config);

	msg->addr = ther->client->addr;
//	msg->addr =DTS201A_SADDR;
	msg->flags = DTS_I2C_WRITE;
	msg->len = 3;
	msg->buf = wdata;

	while (retry--) {
		err = i2c_transfer(ther->client->adapter, msg, 1);
		if ( err >= 0 ) {
			mutex_unlock( &ther->dts201a_i2c_mutex );
			return err;
		}
	}
	mutex_unlock( &ther->dts201a_i2c_mutex );
	pr_err( "%s, i2c transfer error(%d) addr=%x\n", __func__, err, msg->addr );
	return err;
}
#endif
static int dts201a_hw_init(struct dts201a_data *ther)
{
	int err = 0;
	u8 rbuf[5]={0, };
	u16 custom_id[2]={0, };

#ifdef DTS201A_DEBUG
	dts201a_dbgmsg("\t %s  %s\n", __func__, DTS_DEV_NAME);
#endif

#ifdef CAMERA_CCI
	//three bytes command addr+00 +00
	memset(rbuf,0,sizeof(rbuf));
	err= dts201a_cci_write(ther, DTS_CUST_ID0,rbuf,2,0);
	if ( err < 0 ) goto error_firstread;
	msleep( 20 );
	err = dts201a_cci_read( ther, rbuf, 5 );
	if ( err < 0 ) goto error_firstread;
	custom_id[0] = ( ( rbuf[1] << 8 ) | rbuf[2] );

	memset(rbuf,0,sizeof(rbuf));
	err= dts201a_cci_write(ther, DTS_CUST_ID1,rbuf,2,0);
	if ( err < 0 ) goto error_firstread;
	msleep(20);
	err= dts201a_cci_read(ther, 0,rbuf,5);
	if (err < 0) goto error_firstread;
	custom_id[1] = ( (rbuf[1]<<8) | rbuf[2] );
	dts201a_dbgmsg("status = 0x%02x: custom_id: 0x%04x_%04x\n", (rbuf[0], custom_id[0],custom_id[1]);
	if ( custom_id[1] != DTS201A_ID1 && custom_id[0] != DTS201A_ID0 ) {
		err = -ENODEV;
		goto error_unknown_device;
	}
#else
	err = dts201a_i2c_write( ther, DTS_CUST_ID0, 0x0 );
	if ( err < 0 ) goto error_firstread;
	msleep( 20 );
	err = dts201a_i2c_read( ther, rbuf, 3 );
	if ( err < 0 ) goto error_firstread;
	custom_id[0] = ( ( rbuf[1] << 8 ) | rbuf[2] );
	err = dts201a_i2c_write( ther, DTS_CUST_ID1, 0x0 );
	if ( err < 0 ) goto error_firstread;
	msleep( 20 );
	err = dts201a_i2c_read( ther, rbuf, 3);
	if ( err < 0 ) 	goto error_firstread;
	custom_id[1] = ( ( rbuf[1] << 8 ) | rbuf[2] );
	dts201a_dbgmsg("status = 0x%02x: custom_id: 0x%04x_%04x\n", rbuf[0], custom_id[0],custom_id[1]);
	/* register DTS201A_ID1 will be used for other purpose */
	if (custom_id[0] != DTS201A_ID0)
	{
		err = -ENODEV;
		goto error_unknown_device;
	}
#endif

	err = dts201a_read_coefficient_reg( ther );
	if ( err < 0 ) goto error_firstread;
	ther->hw_initialized = 1;
	pr_info("[dts201a] Hw init is done!\n");

	return 0;

error_firstread:
	pr_err("Error reading CUST_ID_DTS201A : is device "
		"available/working?\n");
	goto err_resume_state;
error_unknown_device:
	pr_err("device unknown. Expected: 0x%04x_%04x,"
		" custom_id: 0x%04x_%04x\n", DTS201A_ID1,
		DTS201A_ID0, custom_id[1], custom_id[0]);
err_resume_state:
	ther->hw_initialized = 0;
	pr_err("hw init error : status = 0x%02x: err = %d\n", rbuf[0], err);

	return err;
}

static int dts201a_get_thermtemp_data(struct dts201a_data *ther)
{
	int err = 0, i = 0;
	u8 rbuf[7] = {0,};
	u32 thermopile = 0;
	u32 temperature = 0;
	u8 status = 0;

#ifdef CAMERA_CCI
	memset(rbuf,0,sizeof(rbuf));
	err= dts201a_cci_write(ther, DTS_MEASURE8,rbuf,2,0);
	if (err < 0) goto i2c_error;
	msleep(DTS_CONVERSION_TIME);
	memset(rbuf,0,sizeof(rbuf));
	err= dts201a_cci_read(ther, 0,rbuf,7);
	if (err < 0) goto i2c_error;
#else
	err = dts201a_i2c_write(ther, DTS_MEASURE8, 0x0);
	if (err < 0) goto i2c_error;
	msleep(DTS_CONVERSION_TIME);
	err = dts201a_i2c_read(ther, rbuf, 7);
	if (err < 0) goto i2c_error;
#endif
	status = rbuf[0];

	thermopile = (rbuf[1] << 16) | (rbuf[2] << 8) | rbuf[3];
	temperature = (rbuf[4] << 16) | (rbuf[5] << 8) | rbuf[6];
	//average in i2c if i2c_mavg> 0
	if(i2c_mavg > 0) {
		for(i=0; i<i2c_mavg; i++) {
#ifdef CAMERA_CCI
			memset(rbuf,0,sizeof(rbuf));
			err= dts201a_cci_write(ther, DTS_MEASURE8,rbuf,2,0);
			if (err < 0) goto i2c_error;
			msleep(DTS_CONVERSION_TIME);
			err= dts201a_cci_read(ther, 0,rbuf,7);
#else
			err = dts201a_i2c_write(ther, DTS_MEASURE8, 0x0);
			if (err < 0) goto i2c_error;
			msleep(DTS_CONVERSION_TIME);
			err = dts201a_i2c_read(ther, rbuf, 7);
#endif
			if (err < 0) goto i2c_error;
			status = rbuf[0];
			thermopile = (thermopile + ((rbuf[1] << 16) | (rbuf[2] << 8) | rbuf[3]))/2;
			temperature = (temperature + ((rbuf[4] << 16) | (rbuf[5] << 8) | rbuf[6]))/2;
		}
	}
#ifdef DTS201A_DEBUG
//	dev_dbg(&ther->client->dev, "thermopile = 0x%06x"
//			"Temperature = 0x%06x, status = 0x%02x\n",
//					thermopile, temperature, status);
	dts201a_dbgmsg("status =0x%02x  rbuf[1][2][3] = 0x%02x_%02x_%02x = 0x%06x\n",
		rbuf[0], rbuf[1], rbuf[2], rbuf[3], thermopile);
	dts201a_dbgmsg(" rbuf[4][5][6] = 0x%02x_%02x_%02x = 0x%06x\n",
		rbuf[4], rbuf[5], rbuf[6], temperature);
#endif

	ther->out.therm = thermopile;
	ther->out.temp = temperature;
	ther->out.status = status;

	return err;

i2c_error :
	ther->out.therm = 0;
	ther->out.temp = 0;
	ther->out.status = 0;

	return err;
}

int dts201a_enable(struct dts201a_data *ther)
{
	int err = 0;
	dts201a_power_on(ther,true);
	if (!atomic_cmpxchg(&ther->enabled, 0, 1)) {
		hrtimer_start(&ther->timer, ther->poll_delay, HRTIMER_MODE_REL);
	}
#ifdef DTS201A_DEBUG
	pr_info("%s = %d\n", __func__,  atomic_read(&ther->enabled));
#endif
	return err;
}

int dts201a_disable(struct dts201a_data *ther)
{
	int err = 0;

	if (atomic_cmpxchg(&ther->enabled, 1, 0)) {
		hrtimer_cancel(&ther->timer);
		cancel_work_sync(&ther->work_thermopile);
		pr_info("%s  cancel work\n", __func__);
	}
#ifdef DTS201A_DEBUG
	pr_info("%s = %d\n", __func__,  atomic_read(&ther->enabled));
#endif
	dts201a_power_on(ther,false);
	return err;
}

#ifdef USE_SENSORS_CLASS
static int  dts201a_set_enable(struct sensors_classdev *sensors_cdev,
				unsigned int enable)
{
	struct dts201a_data *ther = g_dts201a_data;
	int mEnabled = atomic_read(&ther->enabled);

	if(!ther) {
		pr_info("\t %s g_dts201a_data is NULL\n", __func__);
		return 0;
	}
	if ((enable == 1) && (mEnabled == 0)) {
		dts201a_errmsg("enable thermopile  sensor\n");
		dts201a_enable(ther);
	} else if ((enable == 0) && (mEnabled == 1)) {
		dts201a_errmsg("disable thermopile sensor\n");
		dts201a_disable(ther);
	} else {
		dts201a_errmsg("unknown enable symbol\n");
	}

	return 0;
}
#if 0
static int  dts201a_set_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_msec)
{
	struct dts201a_data *ther = g_dts201a_data;
	unsigned long new_delay = delay_msec * NSEC_PER_MSEC;

	if(!ther) {
		pr_info("\t %s g_dts201a_data is NULL\n", __func__);
		return 0;
	}
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s new_delay = %ld\n", __func__, new_delay);
#endif

	if(new_delay < DTS_DELAY_MINIMUM)
		new_delay = DTS_DELAY_MINIMUM;
	if (new_delay != ktime_to_ns(ther->poll_delay))
		ther->poll_delay = ns_to_ktime(new_delay);
	mutex_lock(&ther->lock);
	ther->poll_delay = ns_to_ktime(new_delay);
	mutex_unlock(&ther->lock);
	return 0;
}
#endif
#endif
static ssize_t dts201a_poll_delay_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);

#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s poll_delay = %lld\n", __func__, ktime_to_ns(ther->poll_delay));
#endif
	return sprintf(buf, "%lldns\n", ktime_to_ns(ther->poll_delay));
}

static ssize_t dts201a_poll_delay_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);
	unsigned long new_delay = 0;
	int err;

	err = kstrtoul(buf, 10, &new_delay);
	if (err < 0)
		return err;

#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s new_delay = %ld\n", __func__, new_delay);
#endif

	if(new_delay < DTS_DELAY_MINIMUM)
		new_delay = DTS_DELAY_MINIMUM;
	if (new_delay != ktime_to_ns(ther->poll_delay))
		ther->poll_delay = ns_to_ktime(new_delay);
	mutex_lock(&ther->lock);
	ther->poll_delay = ns_to_ktime(new_delay);
	mutex_unlock(&ther->lock);

	return size;
}

static ssize_t dts201a_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);

#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s ther->enabled = %d\n", __func__,
		atomic_read(&ther->enabled));
#endif

	return sprintf(buf, "%d\n", atomic_read(&ther->enabled));
}

static ssize_t dts201a_data2read_show( struct device *dev,
		 struct device_attribute *attr, char *buf )
{
	struct dts201a_data *ther = dev_get_drvdata( dev );
	bool current_vdd = ther->vdd_en;
	if(!current_vdd)
		dts201a_power_on(ther,true);
	mutex_lock(&ther->lock);
	dts201a_get_thermtemp_data(ther);
	mutex_unlock(&ther->lock);
	if(!current_vdd)
		dts201a_power_on(ther,false);
	return sprintf( buf, "%d %d %d\n", ther->out.therm, ther->out.temp ,ther->out.status );
}


static ssize_t dts201a_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct dts201a_data *ther = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s new_value = 0x%d\n", __func__, new_value);
#endif

	if (new_value)
		dts201a_enable(ther);
	else
		dts201a_disable(ther);

	return size;
}


static DEVICE_ATTR(poll_delay, (S_IWUSR|S_IRUSR | S_IWGRP|S_IRGRP),
		dts201a_poll_delay_show, dts201a_poll_delay_store);
static DEVICE_ATTR(enable, (S_IWUSR|S_IRUSR | S_IWGRP|S_IRGRP),
		dts201a_enable_show, dts201a_enable_store);

static struct attribute *thermopile_sysfs_attrs[] = {
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group thermopile_attribute_group = {
	.attrs = thermopile_sysfs_attrs,
};

static ssize_t dts201a_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s VENDOR = %s\n", __func__, DTS_VENDOR);
#endif

	return sprintf(buf, "%s\n", DTS_VENDOR);
}

static ssize_t dts201a_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s CHIP_ID = %s\n", __func__, DTS_CHIP_ID);
#endif
	return sprintf(buf, "%s\n", DTS_CHIP_ID);
}

#ifdef DTS201A_TEST
/*modify 11.30*/
static ssize_t coefficient_reg_show( struct device *dev,
                                     struct device_attribute *attr, char *buf )
{

	struct dts201a_data *ther = dev_get_drvdata( dev );
	int i=0;
	printk( "[dts201a] %s\n", __func__ );

	for( i = 0; i < 34; i++ )
	printk( "[dts201a] %s ndata[%d]=0x%02x\n", __func__, i, ther->ndata[i] );

	for( i = 0; i < 34; i++ )
	printk( "[dts201a] %s hdata[%d]=0x%02x\n", __func__, i, ther->hdata[i] );

		return sprintf( buf, "%02x%02x%02x%02x%02x%02x%02x%02x"\
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"\
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"\
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"\
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"\
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"\
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		ther->ndata[0], ther->ndata[1], ther->ndata[2], ther->ndata[3], ther->ndata[4], ther->ndata[5],
		ther->ndata[6], ther->ndata[7], ther->ndata[8], ther->ndata[9], ther->ndata[10], ther->ndata[11],
		ther->ndata[12], ther->ndata[13], ther->ndata[14], ther->ndata[15], ther->ndata[16], ther->ndata[17],
		ther->ndata[18], ther->ndata[19], ther->ndata[20], ther->ndata[21], ther->ndata[22], ther->ndata[23],
		ther->ndata[24], ther->ndata[25], ther->ndata[26], ther->ndata[27], ther->ndata[28], ther->ndata[29],
		ther->ndata[30], ther->ndata[31], ther->ndata[32], ther->ndata[33],
		ther->hdata[0], ther->hdata[1], ther->hdata[2], ther->hdata[3], ther->hdata[4], ther->hdata[5],
		ther->hdata[6], ther->hdata[7], ther->hdata[8], ther->hdata[9], ther->hdata[10], ther->hdata[11],
		ther->hdata[12], ther->hdata[13], ther->hdata[14], ther->hdata[15], ther->hdata[16], ther->hdata[17],
		ther->hdata[18], ther->hdata[19], ther->hdata[20], ther->hdata[21], ther->hdata[22], ther->hdata[23],
		ther->hdata[24], ther->hdata[25], ther->hdata[26], ther->hdata[27], ther->hdata[28], ther->hdata[29],
		ther->hdata[30], ther->hdata[31], ther->hdata[32], ther->hdata[33] );

}
/*modify 20.06.09*/
static int dts201a_read_coefficient_reg( struct dts201a_data *ther )
{
	int err = 0;
	u8 rbuf[3]={0, };

	int i;
	dts201a_errmsg(" [BODYTEMP]\n");
	// 0x1a-0x27 General Mode
	for(i=STX_N1; i<=ETX_N1; i++)
	{
#ifdef CAMERA_CCI
		memset(rbuf,0,sizeof(rbuf));
		err= dts201a_cci_write(ther, i,rbuf,2,0);
		if (err < 0) {
			dts201a_errmsg("i2c write error\n");
			return err;
		}
		msleep(5);
		err= dts201a_cci_read(ther, 0,rbuf,3);
#else
		err = dts201a_i2c_write(ther, i, 0x0);
		if (err < 0) {
			pr_err("%s: i2c write error\n", __func__);
			return err;
		}
		/*modify 11.30 30 - 5*/
		msleep(5);
		err = dts201a_i2c_read(ther, rbuf, 3);
#endif
		/* add busy bit check */
		if ( (err < 0) || (rbuf[0]&0x20) ) {
			dts201a_errmsg(" i2c read error=0x%02x, 0x%02x\n",  err, rbuf[0]);
			return err;
		}
		ther->ndata[( i - STX_N1 ) + ( i - STX_N1 )] = rbuf[1];
		ther->ndata[( ( i - STX_N1 ) + ( i - STX_N1 ) ) + 1] = rbuf[2];
	}
	//0x13-0x15
	for(i=STX_M1; i<=ETX_M1; i++)
	{
#ifdef CAMERA_CCI
		memset(rbuf,0,sizeof(rbuf));
		err= dts201a_cci_write(ther, i,rbuf,2,0);
		if (err < 0) {
			dts201a_errmsg("i2c write error\n");
			return err;
		}
		msleep(5);
		err= dts201a_cci_read(ther, 0,rbuf,3);
#else
		err = dts201a_i2c_write(ther, i, 0x0);
		if (err < 0) {
			pr_err("%s: i2c write error\n", __func__);
			return err;
		}
		/*modify 11.30*/
		msleep( 5 );
		err = dts201a_i2c_read(ther, rbuf, 3);
#endif
		/* add busy bit check */
		if ( (err < 0) || (rbuf[0]&0x20) ) {
			dts201a_errmsg(" i2c read error=0x%02x, 0x%02x\n",  err, rbuf[0]);
			return err;
		}
		ther->ndata[( i - STX_M1 ) + ( i - STX_M1 ) + ( 28 )] = rbuf[1];
		ther->ndata[( ( i - STX_M1 ) + ( i - STX_M1 ) ) + 1 + ( 28 )] = rbuf[2];
	}

	for(i=0; i<34; i++)
		dts201a_errmsg("[BODYTEMP] ndata[%d]=0x%02x\n", i,ther->ndata[i]);
	//0x28-0x38
	for(i=STX_H2; i<=ETX_M2; i++) {
#ifdef CAMERA_CCI
		memset(rbuf,0,sizeof(rbuf));
		err= dts201a_cci_write(ther, i,rbuf,2,0);
		if (err < 0) {
			dts201a_errmsg(" i2c write error\n");
			return err;
		}
		/*modify 11.30*/
		msleep( 5 );
		err= dts201a_cci_read(ther, 0,rbuf,3);
		/* add busy bit check */
		if ( ( err < 0 ) || ( rbuf[0] & 0x20 ) ) {
			pr_err( "%s: i2c read error=0x%02x, 0x%02x\n", __func__, err, rbuf[0] );
			return err;
		}
		ther->hdata[( i - STX_H2 ) + ( i - STX_H2 )] = rbuf[1];
		ther->hdata[( ( i - STX_H2 ) + ( i - STX_H2 ) ) + 1] = rbuf[2];
#else
		err = dts201a_i2c_write( ther, i, 0x0 );
		if ( err < 0 ) {
			pr_err( "%s: i2c write error\n", __func__ );
			return err;
		}

		/*modify 11.30*/
		msleep( 5 );
		err = dts201a_i2c_read( ther, rbuf, 3 );
		/* add busy bit check */
		if ( ( err < 0 ) || ( rbuf[0] & 0x20 ) ) {
			pr_err( "%s: i2c read error=0x%02x, 0x%02x\n", __func__, err, rbuf[0] );
			return err;
		}

		ther->hdata[( i - STX_H2 ) + ( i - STX_H2 )] = rbuf[1];
		ther->hdata[( ( i - STX_H2 ) + ( i - STX_H2 ) ) + 1] = rbuf[2];
#endif
	}
	for(i=0; i<34; i++)
		dts201a_errmsg("[BODYTEMP]  hdata[%d]=0x%02x\n", i,ther->hdata[i]);
	return 0;
}
static ssize_t i2c_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", i2c_mavg);
}

static ssize_t i2c_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	int count;

	err = kstrtouint(buf, 10, &count);
	if (err < 0)
		return err;
	i2c_mavg = count;
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s i2c_mavg = %d\n", __func__, (int)i2c_mavg);
#endif
	return size;
}
static int reg_addr = 0x17;
static ssize_t i2c_register_read_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 rbuf[7]={0, };
	int reg;
	struct dts201a_data *ther = dev_get_drvdata( dev );

	reg = reg_addr;
#ifdef CAMERA_CCI
	memset(rbuf,0,sizeof(rbuf));
	err = dts201a_cci_write(ther, reg,rbuf,2,0);
	if ( err < 0 ) goto error_firstread;
	msleep( 20 );
	err= dts201a_cci_read(ther, 0,rbuf,7);
	if ( err < 0 ) goto error_firstread;
#else
	err = dts201a_i2c_write( ther, reg, 0x0 );
	if ( err < 0 ) goto error_firstread;
	msleep( 20 );
	err = dts201a_i2c_read( ther, rbuf, 7);
	if ( err < 0 ) goto error_firstread;
#endif
	dts201a_dbgmsg(" dts201a: reg0x%x status = 0x%02x: 0x%04x_%04x 0x%04x_%04x 0x%04x_%04x\n",
		reg,rbuf[0], rbuf[1],rbuf[2],rbuf[3],rbuf[4], rbuf[5],rbuf[6]);

error_firstread:
	return sprintf(buf, " reg0x%x 0x%02x_0x%04x_%04x 0x%04x_%04x 0x%04x_%04x\n",
		reg,rbuf[0], rbuf[1],rbuf[2], rbuf[3],rbuf[4], rbuf[5],rbuf[6]);

}
static ssize_t i2c_register_read_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int cnt;
	int reg;

	cnt = sscanf(buf, "%x",&reg);
	if (cnt == 1)
		reg_addr = reg;
	return size;
}
static ssize_t i2c_register_write_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0,cnt;
	int reg,data;
#ifdef CAMERA_CCI
	u8 rbuf[2]={0, };
#endif
	struct dts201a_data *ther = dev_get_drvdata( dev );

	cnt = sscanf(buf, "%x,%x",&reg,&data);
	if (cnt != 2)
		goto out;
#ifdef CAMERA_CCI
	memset(rbuf,0,sizeof(rbuf));
	rbuf[0] = (u8)data&0xFF;
	rbuf[1] = (u8)((data >> 8)&0xFF);
	err = dts201a_cci_write(ther, reg,rbuf,2,0);
#else
	err = dts201a_i2c_write( ther, reg, data );
#endif
	if ( err < 0 )  {
		dts201a_dbgmsg("error write to 0x57\n");
	}
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s reg %x = %x  err= %d\n", __func__,reg,data, err);
#endif
out:
	return size;
}
#endif

static DEVICE_ATTR(vendor, 0440, dts201a_vendor_show, NULL);
static DEVICE_ATTR(name, 0440, dts201a_name_show, NULL);
#ifdef DTS201A_TEST
static DEVICE_ATTR(coefficient_reg, 0440, coefficient_reg_show, NULL);
static DEVICE_ATTR(dts201a_data2read, 0440, dts201a_data2read_show, NULL );
static DEVICE_ATTR(i2c_avg, (S_IWUSR|S_IRUSR | S_IWGRP|S_IRGRP), i2c_avg_show, i2c_avg_store);
static DEVICE_ATTR(i2c_register_read, (S_IWUSR|S_IRUSR | S_IWGRP|S_IRGRP), i2c_register_read_show, i2c_register_read_store);
static DEVICE_ATTR(i2c_register_write, (S_IWUSR| S_IWGRP), NULL, i2c_register_write_store);
#endif

#ifdef CONFIG_DRM
int dts201a_drm_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;
	int ret = 0;
	struct dts201a_data *ther =  g_dts201a_data;

	if(!ther) {
		pr_info("%s g_dts201a_data is NULL\n", __func__);
		return 0;
	}
	if (!evdata || (evdata->id != 0)) {
		return 0;
	}

	if (!(event == MSM_DRM_EARLY_EVENT_BLANK || event == MSM_DRM_EVENT_BLANK)) {
		pr_info("%s event(%lu) do not need process\n",__func__, event);
		return 0;
	}
	blank = evdata->data;
	pr_info("%s DRM event:%lu,blank:%d", __func__,event, *blank);
	switch (*blank) {
		case MSM_DRM_BLANK_UNBLANK:
			if (MSM_DRM_EARLY_EVENT_BLANK == event) {
				pr_debug("%s resume: event = %lu, not care\n", __func__, event);
			} else if (MSM_DRM_EVENT_BLANK == event) {
				pr_debug("%s resume: event = %lu ,display wake up\n", __func__,event);
			}
		break;
		case MSM_DRM_BLANK_POWERDOWN:
			if (MSM_DRM_EARLY_EVENT_BLANK == event) {
				if (atomic_read(&ther->enabled)) {
				ret = dts201a_disable(ther);
				if (ret < 0)
					pr_info("%s: could not disable\n", __func__);
				}
			} else if (MSM_DRM_EVENT_BLANK == event) {
				pr_debug("%s suspend: event = %lu, not care\n", __func__,event);
			}
		break;
		default:
			pr_info("%s DAM BLANK(%d) do not need process\n", __func__, *blank);
		break;
	}

	return 0;
}
#endif
static enum hrtimer_restart dts201a_timer_func(struct hrtimer *timer)
{
	struct dts201a_data *ther =  g_dts201a_data;
	if(!ther) {
		pr_info("\t %s g_dts201a_data is NULL\n", __func__);
		return 0;
	}
	queue_work(ther->thermopile_wq, &ther->work_thermopile);
	hrtimer_forward_now(&ther->timer, ther->poll_delay);

	return HRTIMER_RESTART;
}

static void dts201a_work_func(struct work_struct *work)
{
	int err = 0;
	struct dts201a_data *ther  = g_dts201a_data;
	if(!ther) {
		pr_info("\t %s g_dts201a_data is NULL\n", __func__);
		return;
	}
	mutex_lock(&ther->lock);
	err = dts201a_get_thermtemp_data(ther);
	mutex_unlock(&ther->lock);
	if (err < 0)
		dev_err(ther->dev, "get_thermopile_data failed\n");

	input_report_rel(ther->input_dev, REL_X, ther->out.therm);
	input_report_rel(ther->input_dev, REL_Y, ther->out.temp);
	input_report_rel(ther->input_dev, REL_Z, ther->out.status);
	input_sync(ther->input_dev);
#ifdef DTS201A_DEBUG
	dts201a_dbgmsg(" therm = 0x%06x, temp = 0x%06x\n",
		 ther->out.therm, ther->out.temp);
#endif

	return;
}
/**
 * One time device  setup
 *
 * call by bus (i2c/cci) level probe to finalize non bus related device setup
 *
 * @param	data The device data
 * @return	0 on success
 */

int dts201a_setup(struct device *dev,struct dts201a_data *data)

{
	struct dts201a_data *ther;
	int err = 0;
	ther = data;

	if (ther == NULL) {
		err = -ENOMEM;
		pr_err( " dts201a_setup failed to allocate memory for module data: %d\n", err);
		goto err_hw_init_failed;
	}
	dts201a_power_on(ther,true);
	ther->dev = dev;
	err = dts201a_hw_init(ther);
	dts201a_errmsg("dts201a_hw_init %d\n",err);
	if (err < 0) {
		dts201a_errmsg("dts201a_hw_init failed\n");
		goto err_hw_init_failed;
	}
	ther->thermopile_class = class_create(THIS_MODULE, "thermopile");
	err = IS_ERR_OR_NULL(ther->thermopile_class );
	if(err) {
		dts201a_errmsg("could not create class\n");
		goto err_class_create_failed;
	}
	ther->dev = device_create(ther->thermopile_class,NULL,0,data,"dts201a");
	err= IS_ERR(ther->dev);
	if(err) {
		dts201a_errmsg(" could not create device\n");
		goto err_device_create;
	}
	dev_set_drvdata(ther->dev, ther);
	dts201a_info("device_create  done \n");
	hrtimer_init(&ther->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ther->poll_delay = ns_to_ktime(DTS_DELAY_DEFAULT * NSEC_PER_MSEC);
	ther->timer.function = dts201a_timer_func;

	ther->thermopile_wq = create_singlethread_workqueue("dts201a_thermopile_wq");
	if (!ther->thermopile_wq) {
		err = -ENOMEM;
		dts201a_errmsg("could not create workqueue\n");
		goto err_create_workqueue;
	}

	INIT_WORK(&ther->work_thermopile, dts201a_work_func);

	ther->input_dev = input_allocate_device();
	if (!ther->input_dev) {
		err = -ENOMEM;
		dev_err(ther->dev, "input device allocate failed\n");
		goto err_input_allocate_device;
	}

	ther->input_dev->name = "bodytemp_sensor";
	input_set_drvdata(ther->input_dev, ther);
	input_set_capability(ther->input_dev, EV_REL, REL_X);
	input_set_capability(ther->input_dev, EV_REL, REL_Y);
	input_set_capability(ther->input_dev, EV_REL, REL_Z);
	//input_set_drvdata(ther->input_dev,ther);
	err = input_register_device(ther->input_dev);
	if (err < 0) {
		dev_err(ther->dev,
			"unable to register input polled device %s\n",
			ther->input_dev->name);
		goto err_input_register_device;
	}
	dts201a_info(" input device register done");

	err = sysfs_create_group(&ther->input_dev->dev.kobj,
				&thermopile_attribute_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group;
	}

	sensors_thermopile_cdev.sensors_enable = dts201a_set_enable;//dts201a_enable;
	//Fix at 200ms delay
	//sensors_thermopile_cdev.sensors_poll_delay =dts201a_set_poll_delay;
	sensors_thermopile_cdev.name = "thermopile_sensor";
	err = sensors_classdev_register(&ther->input_dev->dev, &sensors_thermopile_cdev);
	err = 0;
	if (err < 0) {
		dts201a_errmsg(" sensors_classdev_register failed[%d]\n",  err);
		goto err_class_device_create;
	}
	err = device_create_file(ther->dev, &dev_attr_vendor);
	if (err < 0) {
		dts201a_errmsg(" device_create failed(vendor) failed\n");
		goto err_device_create_file2;
	}

	err = device_create_file(ther->dev, &dev_attr_name);
	if (err < 0) {
		dts201a_errmsg(" device_create failed(name) failed\n");
		goto err_device_create_file3;
	}

#ifdef DTS201A_TEST
	if (device_create_file(ther->dev, 	&dev_attr_coefficient_reg) < 0)
	{
		dts201a_errmsg(" could not create device file(%s)!\n", dev_attr_coefficient_reg.attr.name);
		goto err_device_create_file4;
	}
	if (device_create_file(ther->dev, 	&dev_attr_dts201a_data2read) < 0)
	{
		dts201a_errmsg(" could not create device file(%s)!\n", dev_attr_dts201a_data2read.attr.name);
		goto err_device_create_file5;
	}
	if (device_create_file(ther->dev, 	&dev_attr_i2c_avg) < 0)
	{
		dts201a_errmsg(" could not create device file(%s)!\n", dev_attr_i2c_avg.attr.name);
		goto err_device_create_file6;
	}
	if (device_create_file(ther->dev, 	&dev_attr_i2c_register_read) < 0)
	{
		dts201a_errmsg(" could not create device file(%s)!\n", dev_attr_i2c_register_read.attr.name);
		goto err_device_create_file7;
	}

	if (device_create_file(ther->dev, 	&dev_attr_i2c_register_write) < 0)
	{
		dts201a_errmsg(" could not create device file(%s)!\n", dev_attr_i2c_register_write.attr.name);
		goto err_device_create_file8;
	}
#endif
#ifdef CONFIG_DRM

	ther->fb_notif.notifier_call = dts201a_drm_notifier_callback;
	err = msm_drm_register_client(&ther->fb_notif);
	if (err) {
		dts201a_errmsg(" [DRM]Unable to register fb_notifier: %d\n", err);
	}
#endif
	g_dts201a_data  = ther;
	dts201a_power_on(ther,false);
	dts201a_info("     done");
	return 0;

#ifdef DTS201A_TEST
err_device_create_file8:
	device_remove_file(ther->dev, &dev_attr_i2c_register_write);
err_device_create_file7:
	device_remove_file(ther->dev, &dev_attr_i2c_register_read);
err_device_create_file6:
	device_remove_file(ther->dev, &dev_attr_i2c_avg);
err_device_create_file5:
	device_remove_file(ther->dev, &dev_attr_dts201a_data2read);
err_device_create_file4:
	device_remove_file(ther->dev, &dev_attr_coefficient_reg);
#endif
err_device_create_file3:
	device_remove_file(ther->dev, &dev_attr_name);
err_device_create_file2:
	device_remove_file(ther->dev, &dev_attr_vendor);
err_class_device_create:
	sensors_classdev_unregister(&sensors_thermopile_cdev);
	sysfs_remove_group(&ther->input_dev->dev.kobj,
					&thermopile_attribute_group);
err_sysfs_create_group:
	input_unregister_device(ther->input_dev);
err_input_register_device:
	input_free_device(ther->input_dev);
err_input_allocate_device:
	destroy_workqueue(ther->thermopile_wq);
err_create_workqueue:
	device_unregister(ther->dev);
err_device_create:
	class_unregister(ther->thermopile_class);
err_class_create_failed:
err_hw_init_failed:
#ifndef CAMERA_CCI
	dts201a_power_on(ther,false);
	if( ther->i2c_power_supply) {
		regulator_put(ther->i2c_power_supply);
		ther->i2c_power_supply = NULL;
	}
	if (gpio_is_valid(ther->i2c_pwren_gpio )) {
		gpio_set_value(ther->i2c_pwren_gpio, 0);
		devm_gpio_free(ther->dev, ther->i2c_pwren_gpio);
	}
#endif
	pr_err("%s: Driver Init failed\n", DTS_DEV_NAME);
	return err;
}
int dts201a_cleanup(struct dts201a_data *ther)
{
#ifdef CAMERA_CCI
	struct cci_ctrl_t *cci_ctrl = (struct cci_ctrl_t *)ther->client_object;
#endif
	if(!ther) {
		pr_info("\t %s dts201a_data is NULL\n", __func__);
		return -1;
	}
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s = %d\n", __func__, 0);
#endif
#ifdef CONFIG_DRM
    if (msm_drm_unregister_client(&ther->fb_notif))
        pr_info("%s Error occurred while unregistering fb_notifier \n", __func__);
#endif
#ifdef DTS201A_TEST
	device_remove_file(ther->dev, &dev_attr_i2c_register_write);
	device_remove_file(ther->dev, &dev_attr_i2c_register_read);
	device_remove_file(ther->dev, &dev_attr_i2c_avg);
	device_remove_file(ther->dev, &dev_attr_dts201a_data2read);
	device_remove_file(ther->dev, &dev_attr_coefficient_reg);
#endif
	device_remove_file(ther->dev, &dev_attr_name);
	device_remove_file(ther->dev, &dev_attr_vendor);
	sensors_classdev_unregister(&sensors_thermopile_cdev);
	sysfs_remove_group(&ther->input_dev->dev.kobj,
				&thermopile_attribute_group);
	input_unregister_device(ther->input_dev);
	dts201a_disable(ther);
	flush_workqueue(ther->thermopile_wq);
	destroy_workqueue(ther->thermopile_wq);
#ifdef CAMERA_CCI
	dts201a_power_down_cci(cci_ctrl);
	if (cci_ctrl->cam_pinctrl_status) {
		ret = pinctrl_select_state(
				cci_ctrl->pinctrl_info.pinctrl,
				cci_ctrl->pinctrl_info.gpio_state_suspend);
		if (ret)
			dts201a_errmsg("cannot set pin to suspend state");

		devm_pinctrl_put(cci_ctrl->pinctrl_info.pinctrl);
	}
	/* release gpios */
	dts201a_release_gpios_cci(cci_ctrl);
	/* main driver cleanup */
	dts201a_clean_up_cci();
	kfree(cci_ctrl->io_master_info.cci_client);
	kfree(cci_ctrl);
#else
	dts201a_power_on(ther,false);
	if( ther->i2c_power_supply) {
		regulator_put(ther->i2c_power_supply);
		ther->i2c_power_supply = NULL;
	}
	if (gpio_is_valid(ther->i2c_pwren_gpio )) {
		gpio_set_value_cansleep(ther->i2c_pwren_gpio, 0);
		devm_gpio_free(ther->dev, ther->i2c_pwren_gpio);
	}
#endif
	input_free_device(ther->input_dev);
	if (!IS_ERR(ther->dev)) {
		device_unregister(ther->dev);
	}
	if(!IS_ERR_OR_NULL(ther->thermopile_class )) {
		class_unregister(ther->thermopile_class );
	}
	mutex_destroy(&ther->lock);
	mutex_destroy(&ther->dts201a_i2c_mutex);
	kfree(ther);
	g_dts201a_data = NULL;
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s done\n", __func__);
#endif
	return 0;
}
#ifndef CAMERA_CCI
static int dts201a_of_init(struct i2c_client *client,struct dts201a_data *ther) {
	struct device_node *np = client->dev.of_node;
	int rc;

	//get power supply from dts
	ther->i2c_pwren_gpio = -1;
	ther->vdd_en = false;
	//always on
	ther->i2c_vdd_voltage = -1;
	ther->i2c_power_supply = regulator_get(&client->dev, "i2c_vdd");
	if (IS_ERR(ther->i2c_power_supply)) {
		ther->i2c_power_supply = NULL;
		/* try gpio */
		ther->i2c_pwren_gpio = of_get_named_gpio(np, "pwren-gpio",0) ;
		if (ther->i2c_pwren_gpio < 0) {
			ther->i2c_pwren_gpio = -1;
			dts201a_wanrmsg("no regulator, nor power gpio => power ctrl disabled");
		}
		dts201a_errmsg("Unable to get i2c power supply");
	} else {
		rc = of_property_read_u32(np, "i2c_vdd-voltage", &ther->i2c_vdd_voltage);
		if (rc) {
			dts201a_errmsg("not found i2c_vdd-voltage");
			//always on
			ther->i2c_vdd_voltage = -1;
		}
	}

	if (gpio_is_valid(ther->i2c_pwren_gpio ))  {
		rc = devm_gpio_request(ther->dev, ther->i2c_pwren_gpio, "fir_pwr");
		if (rc) {
			dts201a_wanrmsg("failed to request pwr gpio, rc = %d\n", rc);
		}
		//default off
		gpio_direction_output(ther->i2c_pwren_gpio, 0);
	}
	return 0;
}
static int dts201a_power_on(struct dts201a_data *ther, bool on ){
	int rc = 0;
	if(ther->vdd_en == on) {
		dts201a_errmsg("vdd regulator is same as current do nothing\n");
		return 0;
	}
	if (IS_ERR(ther->i2c_power_supply)) {
		dts201a_errmsg("vdd regulator is invalid\n");
		if (gpio_is_valid(ther->i2c_pwren_gpio )) {
			gpio_set_value(ther->i2c_pwren_gpio, on ? 1:0);
		}
	} else {
		if((ther->i2c_vdd_voltage  > 0) &&
			(regulator_count_voltages(ther->i2c_power_supply)>0)) {
			rc = regulator_set_voltage(ther->i2c_power_supply,ther->i2c_vdd_voltage,
			ther->i2c_vdd_voltage);
			if (rc)
				dts201a_info("set  i2c_vdd-voltage to %d failed",ther->i2c_vdd_voltage);
		}
		if(on) {
			rc = regulator_enable(ther->i2c_power_supply);
		} else {
			rc = regulator_disable(ther->i2c_power_supply);
		}
	}
	ther->vdd_en = on;
	mdelay(10);
	dts201a_errmsg("vdd regulator is %s\n",
				ther->vdd_en ?	"on" : "off");
	return 0;
}
static int dts201a_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id) {
	struct dts201a_data *ther;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err_exit_check_functionality_failed;
	}

	ther = kzalloc(sizeof(struct dts201a_data), GFP_KERNEL);
	if (ther == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto err_exit_alloc_data_failed;
	}

	ther->client = client;
	i2c_set_clientdata(client, ther);

	mutex_init(&ther->lock);
	mutex_init(&ther->dts201a_i2c_mutex);
	mutex_lock(&ther->lock);
	err = dts201a_of_init(client, ther);
	if (err < 0) {
		dts201a_errmsg("%d, failed to get dt info rc %d\n", __LINE__, err);
		goto free_i2c_client;
	}

	/* setup other stuff */
	err = dts201a_setup(&client->dev, ther);
	if (err) {
		dts201a_errmsg("fail to dts201a_setup");
		goto free_i2c_client;
	}

	dts201a_info("End = %d\n", err);
	mutex_unlock(&ther->lock);
	return err;
free_i2c_client:
	mutex_unlock(&ther->lock);
	if(ther) {
		kfree(ther);
		ther = NULL;
		g_dts201a_data = NULL;
	}
err_exit_alloc_data_failed:
err_exit_check_functionality_failed:
	pr_err("%s: dts201a_i2c_probe failed\n", DTS_DEV_NAME);
	return err;
}
static int dts201a_i2c_remove(struct i2c_client *client)
{
	struct dts201a_data *ther = i2c_get_clientdata(client);
	return dts201a_cleanup(ther);
}
#ifdef CONFIG_PM
static int dts201a_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dts201a_data *ther = i2c_get_clientdata(client);
	int ret = 0;
	if(!ther) {
		pr_info("\t %s dts201a_data is NULL\n", __func__);
		return 0;
	}
#ifdef DTS201A_DEBUG
	pr_info("\t[PARTRON] %s = %d\n", __func__, 0);
#endif
	dts201a_power_on(ther,true);

	if (!atomic_read(&ther->enabled)) {
		ret = dts201a_enable(ther);
		if (ret < 0)
			pr_err("%s: could not enable\n", __func__);
	}

	return ret;
}
static int dts201a_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dts201a_data *ther = i2c_get_clientdata(client);
	int ret = 0;
	if(!ther) {
		pr_info("\t %s dts201a_data is NULL\n", __func__);
		return 0;
	}
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s = %d\n", __func__, 0);
#endif

	if (atomic_read(&ther->enabled)) {
		ret = dts201a_disable(ther);
		if (ret < 0)
			pr_err("%s: could not disable\n", __func__);
	}
	dts201a_power_on(ther,false);
	return ret;
}
#endif  /* CONFIG_PM */

static  struct i2c_device_id dts201a_i2c_id[] = {
	{ DTS_DEV_NAME, 0},
	{ },
};

MODULE_DEVICE_TABLE(i2c, dts201a_i2c_id);
#ifdef CONFIG_PM
static const struct dev_pm_ops dts201a_pm_ops = {
	.suspend = dts201a_i2c_suspend,
	.resume = dts201a_i2c_resume,
};
#endif
static const struct of_device_id dts201a_i2c_dt_match[] = {
	{.compatible = "partron,dts201a_i2c"},
	{ },
};
static struct i2c_driver dts201a_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = DTS_DEV_NAME,
			.of_match_table = dts201a_i2c_dt_match,
#ifdef CONFIG_PM
			.pm = &dts201a_pm_ops,
#endif
	},
	.id_table = dts201a_i2c_id,
	.probe = dts201a_i2c_probe,
	.remove = dts201a_i2c_remove,

};
#endif
static int __init dts201a_init(void) {
#ifdef DTS201A_DEBUG
	pr_info("\t[dts201a] %s = %s\n", __func__, DTS_DEV_NAME);
#endif
#ifdef CAMERA_CCI
	return dts201a_init_cci();
#else
	return i2c_add_driver(&dts201a_driver);
#endif

}

static void __exit dts201a_exit(void) {
#ifdef dts201A_DEBUG
	pr_info("\t[dts201a] %s = %s\n", __func__, DTS_DEV_NAME);
#endif
#ifdef CAMERA_CCI
	dts201a_exit_cci((void*)g_dts201a_data);
#else
	i2c_del_driver(&dts201a_driver);
#endif
    return;
}

module_init(dts201a_init);
module_exit(dts201a_exit);

MODULE_DESCRIPTION("PARTRON dts201a thermopile sensor sysfs driver");
MODULE_AUTHOR("PARTRON");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
