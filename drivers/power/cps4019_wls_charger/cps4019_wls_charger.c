/*
 * Copyright © 2020, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/path.h>
#include <linux/types.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/iio/consumer.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#else
#include <linux/pm_wakeup.h>
#include <linux/mmi_wake_lock.h>
#endif

#include <linux/mmi_discrete_charger_class.h>
#include <linux/mmi_discrete_power_supply.h>

#include "cps4019_wls_charger.h"
#include "cps4019_bl.h"

//namespace VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver;

#define BOOTLOADER_FILE_NAME "/data/misc/cps/bootloader.hex"
#define FIRMWARE_FILE_NAME   "/data/misc/cps/firmware.hex"

#define CPS_WLS_CHRG_DRV_NAME "cps-wls-charger"

#define CPS_WLS_CHRG_PSY_NAME "wireless"

/*****************************************************************************
 *  Log
 ****************************************************************************/
#define CPS_LOG_NONE	0
#define CPS_LOG_ERR	1
#define CPS_LOG_DEBG	2
#define CPS_LOG_FULL	3

#define ENABLE_CPS_LOG CPS_LOG_FULL

#define CPS4019_CHIP_ID 	0x4019
#define CPS4019_WORK_VOL 	2700000 //mV

#define cps_wls_log(num, fmt, args...) \
	do { \
			if (ENABLE_CPS_LOG >= (int)num) \
				pr_err("cps4019 " fmt,##args); \
	} while (0)

/*-------------------------------------------------------------------*/
struct cps_wls_chrg_chip {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct regmap *regmap32;
	char *name;
	struct power_supply *wl_psy;
	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
	struct power_supply *dc_psy;
	struct power_supply_desc wl_psd;
	struct power_supply_config wl_psc;
	struct power_supply_desc batt_psd;
	struct power_supply_desc usb_psd;
	struct power_supply_desc dc_psd;
	struct pinctrl *cps_pinctrl;
	struct pinctrl_state *cps_gpio_active;
	struct pinctrl_state *cps_gpio_suspend;
	struct iio_channel *otg_channel;

	#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock cps_wls_wake_lock;
	#else
	struct wakeup_source *cps_wls_wake_lock;
	#endif

	struct mutex irq_lock;
	struct mutex i2c_lock;
	int state;
	int gpio_force_wls;
	int wls_charge_int;
	int cps_wls_irq;
	int reg_addr;
	int reg_data;
	int rx_ovp;
	int rx_ocp;
	int rx_opp;
	int rx_ht_thl;
	int rx_vout_target;
	int rx_ept_rsn;
	int rx_iout;
	int rx_vrect;
	int rx_vout;
	int rx_die_temp;
	int rx_ntc;
	int rx_neg_power;
	int rx_neg_protocol;
	int command_flag;

	/*main charger*/
	const char *main_charger_name;
	struct charger_device *main_chg_dev;

	/*fw relative*/
	const char *wls_fw_name;
	int fw_ver_major;
	int fw_ver_minor;
	bool use_bl_in_h;
};

static struct cps_wls_chrg_chip *chip = NULL;

/*define cps rx reg enum*/
typedef enum
{
	CPS_REG_CHIP_ID,
	CPS_REG_FW_MAJOR_REV,
	CPS_REG_FW_MINOR_REV,
	CPS_REG_STATUS,
	CPS_REG_INT,
	CPS_REG_INT_ENABLE,
	CPS_REG_INT_CLEAR,
	CPS_REG_VOUT_SET,
	CPS_REG_ILIM_SET,
	CPS_REG_ADC_VOUT,
	CPS_REG_ADC_VRECT,
	CPS_REG_ADC_IOUT,
	CPS_REG_ADC_DIE_TEMP,
	CPS_REG_MAX
}cps_reg_e;

typedef struct
{
	uint16_t	 reg_name;
	uint16_t	 reg_bytes_len;
	uint32_t	 reg_addr;
}cps_reg_s;

cps_reg_s cps_reg_cfg[CPS_REG_MAX] = {
	/* reg name			bytes number		reg address	*/
	{CPS_REG_CHIP_ID,		2,			0x0000},
	{CPS_REG_FW_MAJOR_REV,		2,			0x0002},
	{CPS_REG_FW_MINOR_REV,		2,			0x0004},
	{CPS_REG_STATUS,		2,			0x0007},
	{CPS_REG_INT,			2,			0x0009},
	{CPS_REG_INT_ENABLE,		2,			0x000B},
	{CPS_REG_INT_CLEAR,		2,			0x000D},
	{CPS_REG_CMD,			2,			0x000F},
	{CPS_REG_VOUT_SET,		2,			0x0013},
	{CPS_REG_ILIM_SET,		1,			0x0016},
	{CPS_REG_ADC_VOUT,		2,			0x0017},
	{CPS_REG_ADC_VRECT,		2,			0x0019},
	{CPS_REG_ADC_IOUT,		2,			0x001B},
	{CPS_REG_ADC_DIE_TEMP,		2,			0x001D}
};

//-------------------I2C APT start--------------------
static const struct regmap_config cps4019_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const struct regmap_config cps4019_regmap_32bit_config = {
	.reg_bits = 32,
	.val_bits = 8,
};

extern bool mmi_is_factory_mode(void);

static int cps_wls_get_int_flag(void);
static int cps_wls_set_int_clr(int value);
static int cps_wls_get_chip_id(void);
static int cps_wls_get_sys_fw_major_version(void);
static int cps_wls_get_sys_fw_minor_version(void);
static int cps_wls_get_vrect(void);
static int cps_wls_get_iout(void);
static int cps_wls_get_vout(void);
static int cps_wls_get_die_tmp(void);
static int cps_wls_set_rx_vout_target(int value);
static int cps_wls_set_rx_ocp_threshold(int value);

static int cps_wls_read_word_addr32(int reg)
{
	int ret;
	int value;
	unsigned char data[4];
	int i ;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_read(chip->regmap32, reg, data, 4);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
		return CPS_WLS_FAIL;
	}
	//pr_err("cps read32[%08X] %02X %02X %02X %02X\n", reg,data[0], data[1], data[2], data[3]);
	value = 0;
	for(i = 3 ; i >= 0 ; i--) {
		value = (value <<8) | (data[i]);
	}

	return value;
}
static int cps_wls_write_word_addr32(int reg, int value)
{
	int ret;
	unsigned char data[4];
	int i ;

	for(i = 0; i < 4; i++) {
		data[i] = (value >> (8*i) ) & 0xFF;
	}
	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_write(chip->regmap32, reg, data, 4);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}


static int cps_wls_write(int reg, int value)
{
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_write(chip->regmap, reg, value);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_read(int reg)
{
	int ret;
	int value;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_read(chip->regmap, reg, &value);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Chip not exist(rd %x!)\n", __func__, reg);
		return CPS_WLS_FAIL;
	}
	return value;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_write_reg(int reg, int value, int byte_len)
{
	int i=0, tmp=0;
	for(i = 0; i < byte_len; i++) {
		tmp = (value >> (i * 8)) & 0xff;
		if(cps_wls_write((reg & 0xffff) + i, tmp) == CPS_WLS_FAIL)
			goto write_fail;
	}
	return CPS_WLS_SUCCESS;

write_fail:
	return CPS_WLS_FAIL;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_read_reg(int reg, int byte_len)
{
	int i=0,tmp=0,read_data=0;
	for(i = 0; i < byte_len; i++) {
		tmp = cps_wls_read((reg & 0xffff) + i);
		if(tmp == CPS_WLS_FAIL)
			goto read_fail;
		read_data |= (tmp << (8 * i));
	}
	return read_data;

read_fail:
	return CPS_WLS_FAIL;
}

/*
 * @brief big and little endian convertion
 * @param null
 * @return
 */
int big_little_endian_convert(int dat)
{
	char *p;
	char tmp[4];

	p = (char *)(&dat);
	tmp[0] = p[3];
	tmp[1] = p[2];
	tmp[2] = p[1];
	tmp[3] = p[0];
	return *(int *)(tmp);
}
//*****************************for program************************

static int cps_wls_program_sram_addr32(int addr, u8 *data, int len)
{
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_write(chip->regmap32, addr, data, len);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
		return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_write_word(int addr, int value)
{
	int ret;
	u8 write_data[4];

	write_data[0] = value & 0xff;
	write_data[1] = (value >> 8) & 0xff;
	write_data[2] = (value >> 16) & 0xff;
	write_data[3] = (value >> 24) & 0xff;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_write(chip->regmap, addr, write_data, 4);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
		return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_read_word(int addr)
{
	int ret;
	u8 read_data[4];

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_read(chip->regmap, addr, read_data, 4);
	mutex_unlock(&chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
		return CPS_WLS_FAIL;
	}
	return *(int *)read_data;
}

static int cps_wls_program_cmd_send(int cmd)
{
	return cps_wls_write_word_addr32(ADDR_CMD, big_little_endian_convert(cmd));
}

static int cps_wls_program_wait_cmd_done(void)
{
	int ret;
	int wait_time_out = 500;//ms
	while(1) {
		ret = cps_wls_read_word_addr32(ADDR_FLAG);
		if(ret == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
		wait_time_out--;
		//pr_err("cps wait_time_out=%d\n",wait_time_out);
		msleep(1);
		if((big_little_endian_convert(ret) & 0xff) == PASS) {
			break;
		}
		if(wait_time_out < 0)
			return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

static bool cps_check_fw_ver(void)
{
	int fw_major, fw_minor;
	int ret;

	fw_major = cps_wls_get_sys_fw_major_version();
	fw_minor = cps_wls_get_sys_fw_minor_version();

	cps_wls_log(CPS_LOG_ERR, "[%s] chip id %d.%d\n", __func__, fw_major, fw_minor);

	if ((chip->fw_ver_major > fw_major)
			|| (chip->fw_ver_minor > fw_minor))
		ret = true;
	else
		ret = false;

	return ret;
}

static void cps_wls_pm_set_awake(int awake)
{

	cps_wls_log(CPS_LOG_DEBG,"%s lock %d wak %d\n", __func__, chip->cps_wls_wake_lock->active, awake);

	if(!chip->cps_wls_wake_lock->active && awake) {
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock(chip->cps_wls_wake_lock);
#else
		__pm_stay_awake(chip->cps_wls_wake_lock);
#endif
	} else if(chip->cps_wls_wake_lock->active && !awake) {
#ifdef CONFIG_HAS_WAKELOCK
		wake_unlock(chip->cps_wls_wake_lock);
#else
		__pm_relax(chip->cps_wls_wake_lock);
#endif
	}
}

static int cps_get_power_supply_prop(char* name,
			enum power_supply_property psp,
			union power_supply_propval* val) {
	struct psy_in_chip_s {
		char* name;
		struct power_supply **ppsy;
	};
	struct psy_in_chip_s psy_in_chip[] = {
		{"battery", 	&chip->batt_psy},
		{"wireless", 	&chip->wl_psy},
		{"usb", 		&chip->usb_psy},
		{"dc", 		&chip->dc_psy}
	};
	struct power_supply **ppsy = NULL;
	struct power_supply *tmp_psy = NULL;
	int i;

	/*Use a tmp psy as default for the psy that not in chip list*/
	ppsy = &tmp_psy;
	for (i = 0; i < ARRAY_SIZE(psy_in_chip); i++) {
		if (!strcmp(name, psy_in_chip[i].name)) {
			ppsy = psy_in_chip[i].ppsy;
			break;
		}
	}

	if ((!*ppsy) || IS_ERR(*ppsy)) {
		*ppsy = power_supply_get_by_name(name);
		if (!*ppsy || IS_ERR(*ppsy)) {
			cps_wls_log(CPS_LOG_ERR,"%s Couldn't get psy %s\n",__func__, name);
			val->intval = -1;
			return CPS_WLS_FAIL;
		}
	}

	if (power_supply_get_property(*ppsy, psp, val)) {
		val->intval = -1;
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

static int cps_mux_switch(bool on)
{
	cps_wls_log(CPS_LOG_DEBG,"%s set mux = %d\n", __func__, on);

	if (!chip->otg_channel) {
		cps_wls_log(CPS_LOG_ERR,"%s otg iio dev exist\n", __func__);
		return CPS_WLS_FAIL;
	}

	iio_write_channel_raw(chip->otg_channel, !!on);

	return CPS_WLS_SUCCESS;
}

static int cps_check_power(bool *en)
{
	static struct power_supply *chg_psy = NULL;
	union power_supply_propval data;

	if (!chg_psy) {
		chg_psy = power_supply_get_by_name("charger");
		if (!chg_psy || IS_ERR(chg_psy)) {
			cps_wls_log(CPS_LOG_ERR,"%s Couldn't get chg_psy\n",__func__);
			*en = true;
			return CPS_WLS_FAIL;
		}
	}

	power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &data);
	if (data.intval > CPS4019_WORK_VOL) {
		cps_wls_log(CPS_LOG_ERR,"%s chg vol %d. Do not need to set power\n",
				__func__, data.intval);
		*en = false;
	} else {
		*en = true;
	}

	return CPS_WLS_SUCCESS;
}

static int cps_set_power(bool en)
{
	static bool flag = false;

	/*only work in enable*/
	if (en)
		cps_check_power(&flag);

	cps_wls_pm_set_awake(!!en);

	/*only work when check power result is true*/
	if (flag) {
		if (cps_mux_switch(!!en)) {
			cps_wls_pm_set_awake(false);
			return CPS_WLS_FAIL;
		}

		/*wait 50ms for vbus boost stable*/
		msleep(50);

		if (!en)
			flag = false;
	}

	return CPS_WLS_SUCCESS;
}

static int fp_size(struct file *f)
{
	int error = -EBADF;
	struct kstat stat;

	//error = vfs_getattr(&f->f_path, &stat);
	error = vfs_getattr(&f->f_path, &stat, STATX_SIZE, KSTAT_QUERY_FLAGS );

	if (error == 0) {
		pr_err("cps4019 get file file size:%d\n", (int)stat.size);
		return stat.size;
	}
	else {
		pr_err("cps4019 get file file stat error\n");
		return error;
	}
}

static int cps_file_read(char *filename, char **buf)
{
	struct file *fp;
	//mm_segment_t fs;
	int size = 0;
	loff_t pos = 0;

	fp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("cps4019 open %s file error\n", filename);
		goto end;
	}

	size = fp_size(fp);
	if (size <= 0) {
		pr_err("cps4019 load file:%s error\n", filename);
		goto error;
	}

	*buf = kzalloc(size + 1, GFP_KERNEL);
	if( *buf == NULL ) {
		pr_err("cps4019 kzalloc error\n");
		goto error;
	}

	//fs = get_fs();
	//set_fs(KERNEL_DS);
	//vfs_read(fp, *buf, size, &pos);//it's not EXPORT_SYMBOL

	size = kernel_read(fp, *buf, size, &pos);
	pr_err("cps4019 kernel_read %s size:%d\n", filename, size);
error:
	filp_close(fp, NULL);
	//set_fs(fs);
end:
	return size;
}

static unsigned char chartoBcd(char iChar)
{
	unsigned char mBCD = 0;

	if (iChar >= '0' && iChar <= '9')
		mBCD = iChar - '0';
	else if (iChar >= 'A' && iChar <= 'F')
		mBCD = iChar - 'A' + 0x0a;
	else if (iChar >= 'a' && iChar <= 'f')
		mBCD = iChar - 'a' + 0x0a;

	return mBCD;
}

static unsigned char* file_parse(char *buf, int size,
							unsigned char *file, int *file_length)
{
	int  i = 0, j = 0;
	int file_index = 0;
	char temp;

	if (!buf || !file)
		return NULL;
	for(i = 0; i < size; i++) {
		if(buf[i] == '\n' || buf[i] == ' ' || buf[i] == '\r') {
			file_index = 0;
			continue;
		} else{
			if (file_index == 1) {
				file_index++;
				file[j++] = (unsigned char)((chartoBcd(temp) << 4) + chartoBcd(buf[i]));
			} else if (file_index == 0) {
				file_index++;
				temp = buf[i];
			}
		}
	}
	// file[j] = '\0';
	*file_length = j;
	return file;
}

static int bootloader_load(unsigned char *bootloader, int *bootloader_length)
{
	char *buf = NULL;
	int size = 0;

	cps_wls_log(CPS_LOG_DEBG,"%s use h file %d\n",__func__, chip->use_bl_in_h);

	if (chip->use_bl_in_h) {
		memcpy(bootloader, CPS4019_BL, CPS4019_BL_SIZE);
		*bootloader_length = CPS4019_BL_SIZE;
	} else {
		size = cps_file_read(BOOTLOADER_FILE_NAME, &buf);
		if (size > 0) {
			if (bootloader == NULL) {
				pr_err("cps4019 file alloc error.\n");
				kfree(buf);
				return -EINVAL;
			}

			if(file_parse(buf, size, bootloader, bootloader_length) == NULL) {
				pr_err("cps4019 file parse error\n");
				kfree(buf);
				return -EINVAL;
			}

			kfree(buf);
		}
	}

	cps_wls_log(CPS_LOG_DEBG,"cps_[%s] bootloader_length=%d\n", __func__, *bootloader_length);

	return 0;
}

static int firmware_load(unsigned char *firmeware, int *firmeware_length)
{
	char *buf = NULL;
	int size = 0;
	int ret;
	const struct firmware *fw;

	if (chip->wls_fw_name) {
		ret = request_firmware(&fw, chip->wls_fw_name, chip->dev);
		if (ret || fw->size <=0 ) {
			cps_wls_log(CPS_LOG_ERR,"Couldn't get firmware  rc=%d\n", ret);
			return -EINVAL;
		}

		size =  fw->size;
		buf = kzalloc(size+1, GFP_KERNEL);  // 18K buffer
		memset(buf, 0, size+1);
		memcpy(buf, fw->data, size);
	} else {
		size = cps_file_read(FIRMWARE_FILE_NAME, &buf);
	}

	if (size > 0) {
		if (firmeware == NULL) {
			kfree(buf);
			pr_err("cps4019 file alloc error.\n");
			return -EINVAL;
		}
		if(file_parse(buf, size, firmeware, firmeware_length) == NULL) {
			kfree(buf);
			pr_err("cps4019 file parse error\n");
			return -EINVAL;
		}
		kfree(buf);
	}

	return 0;
}

static int update_firmware(void)
{
	int ret, write_count, k;
	int bootloader_length, firmware_length;
	int buff0_flag = 0, buff1_flag = 0;
	unsigned char *bootloader_buf;
	unsigned char *firmware_buf;
	unsigned char *p;
	unsigned short int chip_id;
	int result;
	int *p_convert;

	if (cps_set_power(true)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] en power fail!!\n", __func__);
		goto update_fail;
	}

	if (!cps_check_fw_ver()) {
		cps_wls_log(CPS_LOG_ERR, "[%s] fw already exist OR chip do not exist. Skip!!\n", __func__);
		goto update_fail;
	}

	bootloader_buf = kzalloc(CPS4019_BL_SIZE, GFP_KERNEL);// 2K buffer
	firmware_buf = kzalloc(0x4000, GFP_KERNEL);// 16K buffer

/***************************************************************************************
 *                                  Step1, load to sram                                *
 ***************************************************************************************/
	ret = bootloader_load(bootloader_buf, &bootloader_length);//load bootloader
	if (ret != 0) {
		cps_wls_log(CPS_LOG_DEBG, "[%s] ---- bootloader get error %d\n", __func__, ret);
		goto update_fail;
	}

	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0xFFFFFF00, big_little_endian_convert(0x0E000000)))
		goto update_fail; /*enable 32bit i2c*/

	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0x40040070, big_little_endian_convert(0x0000A061)))
		goto update_fail; /*write password*/

	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0x40040004, big_little_endian_convert(0x00000008)))
		goto update_fail; /*reset and halt mcu*/

	if(CPS_WLS_FAIL == cps_wls_program_sram_addr32(0x20000000, bootloader_buf, bootloader_length))
		goto update_fail;//program sram

	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0x40040010, big_little_endian_convert(0x00000001)))
		goto update_fail; /*triming load function is disabled*/

	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0x40040004, big_little_endian_convert(0x00000066)))
		goto update_fail; /*enable remap function and reset the mcu*/

	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- system restart\n", __func__);

	msleep(10);
	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0xFFFFFF00, big_little_endian_convert(0x0E000000)))
		goto update_fail; /*enable 32bit i2c*/
	msleep(10);

/***************************************************************************************
 *                          Step2, bootloader crc check                                *
 ***************************************************************************************/
	cps_wls_program_cmd_send(CACL_CRC_TEST);
	result = cps_wls_program_wait_cmd_done();

	if(result != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- bootloader crc fail\n", __func__);
		goto update_fail;
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- load bootloader successful\n", __func__);


/***************************************************************************************
 *                          Step3, load firmware to MTP                                *
 ***************************************************************************************/
	memset(firmware_buf, 0, 0x4000);
	ret = firmware_load(firmware_buf, &firmware_length);//load bootloader
	if (ret != 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- firmware get error %d\n", __func__, ret);
		goto update_fail;
	}
	pr_err("cps4019 firmware_length=%d\n", firmware_length);
	//set write buffer size   defalt 64 word
	cps_wls_write_word(ADDR_BUF_SIZE, big_little_endian_convert(CPS_PROGRAM_BUFFER_SIZE));
	write_count = 0;

	p_convert = (int*)firmware_buf;
	for(k = 0; k < 16 * 1024 / 4; k++) {
		p_convert[k] = big_little_endian_convert(p_convert[k]);
	}

start_write_app_code:

	cps_wls_program_cmd_send(PGM_ERASER_0);
	result = cps_wls_program_wait_cmd_done();

	if(result != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- bootloader crc fail\n", __func__);
		goto update_fail;
	}
	p = (unsigned char*)p_convert;
	write_count++;

	for (k = 0; k < (16 * 1024 / 4) / CPS_PROGRAM_BUFFER_SIZE; k++) {
		if (buff0_flag == 0) {
			//write buf0
			cps_wls_program_sram_addr32(ADDR_BUFFER0, p, CPS_PROGRAM_BUFFER_SIZE*4);
			p = p + CPS_PROGRAM_BUFFER_SIZE*4;
			if (buff1_flag == 1) {
				//wait finish
				cps_wls_program_wait_cmd_done();
				if(result != CPS_WLS_SUCCESS) {
					cps_wls_log(CPS_LOG_ERR, "[%s] ---- buffer1 program fail\n", __func__);
					goto update_fail;
				}
				buff1_flag = 0;
			}
			//write buff 0 CMD
			cps_wls_program_cmd_send(PGM_BUFFER0);
			buff0_flag = 1;
			continue;
		}
		if (buff1_flag == 0) {
			//write buf1
			cps_wls_program_sram_addr32(ADDR_BUFFER1, p, CPS_PROGRAM_BUFFER_SIZE*4);
			p = p + CPS_PROGRAM_BUFFER_SIZE*4;
			if (buff0_flag == 1) {
				//wait finish
				cps_wls_program_wait_cmd_done();
				if(result != CPS_WLS_SUCCESS) {
					cps_wls_log(CPS_LOG_ERR, "[%s] ---- buffer0 program fail\n", __func__);
					goto update_fail;
				}
				buff0_flag = 0;
			}
			//write buff 0 CMD
			cps_wls_program_cmd_send(PGM_BUFFER1);
			buff1_flag = 1;
			continue;
		}
	}
	if (buff0_flag == 1) {
		//wait finish
		cps_wls_program_wait_cmd_done();
		if(result != CPS_WLS_SUCCESS) {
			cps_wls_log(CPS_LOG_ERR, "[%s] ---- buffer0 program fail\n", __func__);
			goto update_fail;
		}
		buff0_flag = 0;
	}

	if (buff1_flag == 1) {
		//wait finish
		cps_wls_program_wait_cmd_done();
		if(result != CPS_WLS_SUCCESS) {
			cps_wls_log(CPS_LOG_ERR, "[%s] ---- buffer1 program fail\n", __func__);
			goto update_fail;
		}
		buff1_flag = 0;
	}

/***************************************************************************************
 *                          Step4, check app CRC                                       *
 ***************************************************************************************/
	cps_wls_program_cmd_send(CACL_CRC_APP);
	result = cps_wls_program_wait_cmd_done();

	if(result != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- TEST APP CRC fail", __func__);
		if(write_count < 3) goto start_write_app_code;
		else goto update_fail;
	}else {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- TEST APP CRC success", __func__);
	}

/***************************************************************************************
 *                          Step5, write mcu start flag                                *
 ***************************************************************************************/
	cps_wls_program_cmd_send(PGM_WR_FLAG);
	result = cps_wls_program_wait_cmd_done();

	if(result != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- WRITE MCU START FLAG fail", __func__);
		goto update_fail;
	}else {
		cps_wls_log(CPS_LOG_ERR, "[%s] ---- WRITE MCU START FLAG success", __func__);
	}

	if(CPS_WLS_FAIL == cps_wls_write_word_addr32(0x40040004, big_little_endian_convert(0x00000001)))
		goto update_fail; /*reset all system*/
	msleep(100);

/***************************************************************************************
 *                          Step6, check chip id                                       *
 ***************************************************************************************/
	chip_id = cps_wls_read_word(0x0000);
	pr_err("cps4019 FW_MAJOR=0x%04X\n",big_little_endian_convert(cps_wls_read_word(0x0002))&0xFFFF);
	pr_err("cps4019 FW_MINOR=0x%04X\n",big_little_endian_convert(cps_wls_read_word(0x0004))&0xFFFF);
	if(chip_id != 0x4019) {
		cps_wls_log(CPS_LOG_DEBG, "[%s] ---- CHECK CHIP ID fail = %x\n", __func__, chip_id);
		goto update_fail;
	}

	cps_set_power(false);

	cps_wls_log(CPS_LOG_DEBG, "[%s] ---- Program successful CHIP ID=0x%04x\n", __func__, chip_id);

	return CPS_WLS_SUCCESS;

update_fail:
	cps_set_power(false);
	cps_wls_log(CPS_LOG_ERR, "[%s] ---- update fail\n", __func__);

	return CPS_WLS_FAIL;
}

//-------------------CPS4019 system interface-------------------
static int cps_wls_get_int_flag(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_INT]);
	return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_int_clr(int value)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_INT_CLEAR]);
	return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_chip_id(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_CHIP_ID]);
	return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_sys_fw_major_version(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_FW_MAJOR_REV]);
	return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_sys_fw_minor_version(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_FW_MINOR_REV]);
	return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_vrect(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_ADC_VRECT]);
	return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_iout(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_ADC_IOUT]);
	return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_vout(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_ADC_VOUT]);
	return cps_wls_read_reg((int)cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_state(void)
{
	cps_reg_s *cps_reg;

	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_STATUS]);
	return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}

static bool cps_wls_get_vout_state(void)
{
	int ret = cps_wls_get_state();

	if (ret == CPS_WLS_FAIL)
		ret = 0;

	return !!(ret & (1<<6));
}

static int cps_wls_get_die_tmp(void)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_ADC_DIE_TEMP]);
	return cps_wls_read_reg(cps_reg->reg_addr, (int)cps_reg->reg_bytes_len);
}


static int cps_wls_set_rx_vout_target(int value)
{
	cps_reg_s *cps_reg;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_VOUT_SET]);
	return cps_wls_write_reg(cps_reg->reg_addr, value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_rx_ocp_threshold(int value)
{
	int value_temp;
	cps_reg_s *cps_reg;
	if(value < 100 || value > 1100)
		return CPS_WLS_FAIL;

	value_temp = value / 50; /*1LSB 50mA*/

	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_ILIM_SET]);
	return cps_wls_write_reg(cps_reg->reg_addr, value_temp, (int)cps_reg->reg_bytes_len);
}

//------------------------------IRQ Handler-----------------------------------
static int cps_wls_set_int_enable(void)
{
	uint16_t int_en;
	cps_reg_s *cps_reg;

	int_en = 0xFFFF;
	cps_reg = (cps_reg_s*)(&cps_reg_cfg[CPS_REG_INT_ENABLE]);

	if(CPS_WLS_FAIL == cps_wls_write_reg((int)cps_reg->reg_addr, int_en, (int)cps_reg->reg_bytes_len))
		goto set_int_fail;
	return CPS_WLS_SUCCESS;

set_int_fail:
	return CPS_WLS_FAIL;
}

static int cps_wls_irq_process(int int_flag)
{
	int rc = 0;

	if (int_flag & INT_TX_DATA_RECEIVED) {
		//todo
	}

	if(int_flag & INT_UV) {
		//todo
	}

	if(int_flag & INT_OT) {
	}
	if(int_flag & INT_OC) {
	}
	if(int_flag & INT_OV) {
	}
	if(int_flag & INT_ID_CFG_FINISH) {
		cps_wls_set_int_enable();
	}
	if(int_flag & INT_VOUT_STATE) {
	}
	//if(int_flag & INT_DATA_STORE){}
	//if(int_flag & INT_AC_MIS_DET){}

	return rc;
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
	int int_flag;
	int int_clr;
	cps_wls_log(CPS_LOG_DEBG, "[%s] IRQ triggered\n", __func__);
	mutex_lock(&chip->irq_lock);

	int_flag = cps_wls_get_int_flag();
	cps_wls_log(CPS_LOG_DEBG, ">>>>>int_flag = %x\n", int_flag);
	if(int_flag == CPS_WLS_FAIL) {
		cps_wls_log(CPS_LOG_ERR, "[%s] read wls irq reg failed\n", __func__);
		mutex_unlock(&chip->irq_lock);
		return IRQ_HANDLED;
	}

	int_clr = int_flag;
	cps_wls_set_int_clr(int_flag);
	mutex_unlock(&chip->irq_lock);

	cps_wls_irq_process(int_flag);

	return IRQ_HANDLED;
}

static enum power_supply_property cps_wls_chrg_props[] = {
	POWER_SUPPLY_PROP_CURRENT_MAX,
	//POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int cps_wls_chrg_property_is_writeable(struct power_supply *psy,
			enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	//case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return 1;

	default:
		break;
	}

	return 0;
}

static int cps_wls_chrg_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	switch(psp) {
		case POWER_SUPPLY_PROP_ONLINE:
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = cps_wls_get_vout_state();
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = 5000000;//Have no usage, just for voltage max propety.
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = cps_get_power_supply_prop("usb", psp, val);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			ret = cps_get_power_supply_prop("battery", psp, val);
			break;
		default:
			return -EINVAL;
			break;
	}

	return ret;
}

static int cps_wls_chrg_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	int ret = 0;
	struct cps_wls_chrg_chip *chip = power_supply_get_drvdata(psy);
	cps_wls_log(CPS_LOG_DEBG, "[%s] psp = %d.\n", __func__, psp);
	chip->state = 1;
	return ret;
}

static void cps_wls_charger_external_power_changed(struct power_supply *psy)
{
	;
}

//-----------------------------reg addr----------------------------------
static ssize_t show_reg_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "reg addr 0x%08x\n", chip->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	chip->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);

//-----------------------------reg data----------------------------------
static ssize_t show_reg_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	chip->reg_data = cps_wls_read_reg(chip->reg_addr, 4);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", chip->reg_addr, chip->reg_data);
}

static ssize_t store_reg_data(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	chip->reg_data = tmp;
	cps_wls_write_reg(chip->reg_addr, chip->reg_data, 4);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static ssize_t store_update_fw(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	if(tmp != 0) {
		cps_wls_log(CPS_LOG_DEBG, "[%s] -------start update fw\n", __func__);
		update_firmware();
	}

	return count;
}
static DEVICE_ATTR(update_fw, 0220, NULL, store_update_fw);

static ssize_t show_iout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "iout = %dmA\n", cps_wls_get_iout());
}
static DEVICE_ATTR(get_iout, 0444, show_iout, NULL);

static ssize_t show_vrect(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "vrect = %dmV\n", cps_wls_get_vrect());
}
static DEVICE_ATTR(get_vrect, 0444, show_vrect, NULL);

static ssize_t show_vout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "vout = %dmV\n", cps_wls_get_vout());
}
static DEVICE_ATTR(get_vout, 0444, show_vout, NULL);

static ssize_t show_chip_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", cps_wls_get_chip_id());
}
static DEVICE_ATTR(chip_id, 0444, show_chip_id, NULL);

static ssize_t show_fw_major_ver(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fw major version = 0x%x\n", cps_wls_get_sys_fw_major_version());
}
static DEVICE_ATTR(get_fw_major_ver, 0444, show_fw_major_ver, NULL);

static ssize_t show_fw_minor_ver(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fw minor version = 0x%x\n", cps_wls_get_sys_fw_minor_version());
}
static DEVICE_ATTR(get_fw_minor_ver, 0444, show_fw_minor_ver, NULL);

static ssize_t show_die_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "die temp = %d\n", cps_wls_get_die_tmp());
}
static DEVICE_ATTR(get_die_temp, 0444, show_die_temp, NULL);

static ssize_t store_vout_target(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	cps_wls_log(CPS_LOG_DEBG, "[%s] -------set vout target = %dmV\n", __func__ , tmp);
	cps_wls_set_rx_vout_target(tmp);

	return count;
}
static DEVICE_ATTR(set_vout_target, 0220, NULL, store_vout_target);

static ssize_t store_ocp_thres(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int tmp;

	tmp = simple_strtoul(buf, NULL, 0);
	cps_wls_log(CPS_LOG_DEBG, "[%s] -------set ocp threshold = %dmA\n", __func__ , tmp);
	cps_wls_set_rx_ocp_threshold(tmp);

	return count;
}
static DEVICE_ATTR(set_ocp_thres, 0220, NULL, store_ocp_thres);

static ssize_t store_usb_keep_on(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	bool force_wls;

	force_wls = !!simple_strtoul(buf, NULL, 0);

	if (chip->main_chg_dev)
		charger_dev_set_dp_dm(chip->main_chg_dev, force_wls ?
				MMI_POWER_SUPPLY_IGNORE_REQUEST_DPDM :
				MMI_POWER_SUPPLY_DONOT_IGNORE_REQUEST_DPDM);

	if(gpio_is_valid(chip->gpio_force_wls))
		gpio_direction_output(chip->gpio_force_wls, force_wls);

	return count;
}
static DEVICE_ATTR(usb_keep_on, 0220, NULL, store_usb_keep_on);

static void cps_wls_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_reg_addr);
	device_create_file(dev, &dev_attr_reg_data);
//-----------------------program---------------------
	device_create_file(dev, &dev_attr_update_fw);

//-----------------------RX--------------------------
	device_create_file(dev, &dev_attr_get_iout);
	device_create_file(dev, &dev_attr_get_vrect);
	device_create_file(dev, &dev_attr_get_vout);
	device_create_file(dev, &dev_attr_chip_id);

	device_create_file(dev, &dev_attr_get_fw_major_ver);
	device_create_file(dev, &dev_attr_get_fw_minor_ver);
	device_create_file(dev, &dev_attr_get_die_temp);

	device_create_file(dev, &dev_attr_set_vout_target);
	device_create_file(dev, &dev_attr_set_ocp_thres);

	device_create_file(dev, &dev_attr_usb_keep_on);
}

static int cps_wls_parse_dt(struct cps_wls_chrg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;

	if(!node) {
		cps_wls_log(CPS_LOG_ERR, "devices tree node missing \n");
		return -EINVAL;
	}

	chip->wls_charge_int = of_get_named_gpio(node, "cps_wls_int", 0);
	if(!gpio_is_valid(chip->wls_charge_int)) {
		cps_wls_log(CPS_LOG_ERR, "wls_charge_int is not valid %d\n",chip->wls_charge_int );
		return -EINVAL;
	}

	chip->gpio_force_wls = of_get_named_gpio(node, "force_wls_pin", 0);
	if(gpio_is_valid(chip->gpio_force_wls)) {
		if (!gpio_request(chip->gpio_force_wls, "mmi force wls pin"))
			gpio_direction_output(chip->gpio_force_wls, 0);
		else
			dev_err(chip->dev, "%s: %d gpio(wls en) request failed\n",
					__func__, chip->gpio_force_wls);
	}

	chip->main_charger_name = NULL;
	of_property_read_string(node, "main-charger-name", &chip->main_charger_name);

	chip->fw_ver_major = 0;
	chip->fw_ver_minor = 0;
	of_property_read_u32(node, "fw_ver_major", &chip->fw_ver_major);
	of_property_read_u32(node, "fw_ver_minor", &chip->fw_ver_minor);

	chip->wls_fw_name = NULL;
	of_property_read_string(node, "wireless-fw-name", &chip->wls_fw_name);


	chip->use_bl_in_h = of_property_read_bool(node, "use_bl_in_h");

	cps_wls_log(CPS_LOG_ERR,
		"[%s]  wls_charge_int %d gpio_force_wls %d \
		main_charger_name %s wls_fw_name: %s ver %d.%d use_bl_h %s\n",
		__func__,
		chip->wls_charge_int, chip->gpio_force_wls,
		chip->main_charger_name ? chip->main_charger_name : "null",
		chip->wls_fw_name ? chip->wls_fw_name : "null",
		chip->fw_ver_major, chip->fw_ver_minor,
		chip->use_bl_in_h ? "true" : "false");
	return 0;
}

static int cps_wls_gpio_request(struct cps_wls_chrg_chip *chip)
{
	int ret =0;
	int irqn = 0;

	if(gpio_is_valid(chip->wls_charge_int)) {
		ret = gpio_request_one(chip->wls_charge_int, GPIOF_DIR_IN, "cps4019_ap_int");
		if(ret) {
			cps_wls_log(CPS_LOG_ERR, "[%s] int gpio request failed\n", __func__);
			goto err_irq_gpio;
		}
		irqn = gpio_to_irq(chip->wls_charge_int);
		if(irqn < 0) {
			ret = irqn;
			cps_wls_log(CPS_LOG_ERR, "[%s] failed to gpio to irq\n", __func__);
			goto err_irq_gpio;
		}
		chip->cps_wls_irq = irqn;
	} else {
		cps_wls_log(CPS_LOG_ERR, "[%s] reset gpio not provided\n", __func__);
		goto err_irq_gpio;
	}

err_irq_gpio:
	gpio_free(chip->wls_charge_int);

	return ret;
}


static void cps_wls_lock_work_init(struct cps_wls_chrg_chip *chip)
{
	mutex_init(&chip->irq_lock);
	mutex_init(&chip->i2c_lock);
	//wake_lock_init(&chip->cps_wls_wake_lock, WAKE_LOCK_SUSPEND, "cps_wls_wake_lock");
	chip->cps_wls_wake_lock = wakeup_source_create( "cps_wls_wake_lock" );

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&chip->cps_wls_wake_lock, WAKE_LOCK_SUSPEND, "cps_wls_wake_lock");
#else
	PM_WAKEUP_REGISTER(chip->dev, chip->cps_wls_wake_lock, "pen_suspend_blocker");
	if(!chip->cps_wls_wake_lock) {
		dev_err(chip->dev,"%s: Failed to allocate wakeup source\n",__func__);
		//goto fail_wakeup_init;
	}
#endif
	//INIT_DELAYED_WORK(&chip->cps_wls_monitor_work, cps_wls_monitor_work_func);
}


static void cps_wls_lock_destroy(struct cps_wls_chrg_chip *chip)
{
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->i2c_lock);
	//wake_lock_destroy(&chip->cps_wls_wake_lock);
	//wakeup_source_destroy(chip->cps_wls_wake_lock);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&chip->cps_wls_wake_lock);
#else
	PM_WAKEUP_UNREGISTER(chip->cps_wls_wake_lock);
#endif
	//cancel_delayed_work_sync(&chip->cps_wls_monitor_work);
}

static void cps_wls_free_gpio(struct cps_wls_chrg_chip *chip)
{
	if(gpio_is_valid(chip->wls_charge_int))
		gpio_free(chip->wls_charge_int);
}

static int cps_wls_register_psy(struct cps_wls_chrg_chip *chip)
{
	struct power_supply_config cps_wls_psy_cfg = {};

	chip->wl_psd.name = CPS_WLS_CHRG_PSY_NAME;
	chip->wl_psd.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->wl_psd.properties = cps_wls_chrg_props;
	chip->wl_psd.num_properties = ARRAY_SIZE(cps_wls_chrg_props);
	chip->wl_psd.get_property = cps_wls_chrg_get_property;
	chip->wl_psd.set_property = cps_wls_chrg_set_property;
	chip->wl_psd.property_is_writeable= cps_wls_chrg_property_is_writeable;
	chip->wl_psd.external_power_changed = cps_wls_charger_external_power_changed;

	cps_wls_psy_cfg.drv_data = chip;
	cps_wls_psy_cfg.of_node = chip->dev->of_node;
	chip->wl_psy = power_supply_register(chip->dev, &chip->wl_psd, &cps_wls_psy_cfg);
	if(IS_ERR(chip->wl_psy)) {
		return PTR_ERR(chip->wl_psy);
	}

	return CPS_WLS_SUCCESS;
}

static int cps_wls_chrg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret=0;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,"[%s] cps_debug: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;
	chip->name = "cps_wls";
	chip->regmap = devm_regmap_init_i2c(client, &cps4019_regmap_config);
	if (IS_ERR(chip->regmap)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
		devm_kfree(&client->dev, chip);
		return PTR_ERR(chip->regmap);
	}
	chip->regmap32 = devm_regmap_init_i2c(client, &cps4019_regmap_32bit_config);
	if (IS_ERR(chip->regmap32)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap32!\n", __func__);
		devm_kfree(&client->dev, chip);
		return PTR_ERR(chip->regmap32);
	}
	i2c_set_clientdata(client, chip);
	dev_set_drvdata(&(client->dev), chip);

	ret = cps_wls_parse_dt(chip);
	if(ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Couldn't parse DT nodes ret = %d\n", __func__, ret);
		goto free_source;
	}

	ret = cps_wls_gpio_request(chip);
	if(ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] gpio request failed ret = %d\n", __func__, ret);
		goto free_source;
	}

	if(chip->cps_wls_irq) {
		ret = devm_request_threaded_irq(&client->dev, chip->cps_wls_irq, NULL,
			cps_wls_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "cps_wls_irq", chip);
		if(ret) {
			cps_wls_log(CPS_LOG_ERR, "[%s] request cps_wls_int irq failed ret = %d\n", __func__, ret);
			goto free_source;
		}
		enable_irq_wake(chip->cps_wls_irq);
	}
	cps_wls_lock_work_init(chip);

	if (chip->main_charger_name) {
		chip->main_chg_dev = get_charger_by_name(chip->main_charger_name);
		if (!chip->main_chg_dev) {
			cps_wls_log(chip, "*** Error : can't find main charger %s***\n", chip->main_charger_name);
		}
	}

	cps_wls_create_device_node(&(client->dev));

	chip->otg_channel = devm_iio_channel_get(chip->dev, "otg_enable");
	if (IS_ERR(chip->otg_channel)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] get otg iio dev failed. Waiting for retry\n", __func__);
		ret = -1;//EPROBE_DEFER;
		goto free_source;
	}

	ret = cps_wls_register_psy(chip);
	if(IS_ERR(chip->wl_psy)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] power_supply_register wireless failed , ret = %d\n", __func__, ret);
		goto free_source;
	}

	cps_wls_log(CPS_LOG_DEBG, "[%s] wireless charger addr low probe successful!\n", __func__);

	/*check firmwware, only work in factory mode*/
	if (mmi_is_factory_mode())
		update_firmware();

	return ret;

free_source:
	cps_wls_free_gpio(chip);
	cps_wls_lock_destroy(chip);
	kfree(chip);
	cps_wls_log(CPS_LOG_ERR, "[%s] error: free resource.\n", __func__);

	return ret;
}

static void not_called_api(void)
{
	/*int rc;
	rc = cps_wls_get_rx_ss_pkt_value();
	rc = cps_wls_get_rx_ce_pkt_value();
	rc = cps_wls_get_rx_rp_pkt_value();
	rc = cps_wls_get_rx_fop_value();
	rc = cps_wls_get_rx_ept_code();
	rc = cps_wls_get_rx_neg_power();
	rc = cps_wls_get_rx_neg_pro();
	rc = cps_wls_get_rx_vrect();
	rc = cps_wls_get_rx_iout();
	rc = cps_wls_get_rx_vout();
	rc = cps_wls_get_rx_die_tmp();
	rc = cps_wls_set_rx_vout_target(5000);
	rc = cps_wls_set_rx_neg_power(20);
	rc = cps_wls_get_tx_ce_value();
	rc = cps_wls_get_tx_rp_value();*/
	return;
}

static int cps_wls_chrg_remove(struct i2c_client *client)
{
	not_called_api();
	//cps_wls_lock_destroy(chip);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id cps_wls_charger_id[] = {
	{"cps-wls-charger", 0},
	{},
};

static const struct of_device_id cps_wls_chrg_of_tbl[] = {
	{ .compatible = "cps,wls-charger-cps4019", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id);

static struct i2c_driver cps_wls_charger_driver = {
	.driver = {
		.name = CPS_WLS_CHRG_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cps_wls_chrg_of_tbl,
	},
	.probe = cps_wls_chrg_probe,
	.remove = cps_wls_chrg_remove,
	.id_table = cps_wls_charger_id,
};

static int __init cps_wls_driver_init(void)
{
	return i2c_add_driver(&cps_wls_charger_driver);
}

late_initcall(cps_wls_driver_init);

static void __exit cps_wls_driver_exit(void)
{
	i2c_del_driver(&cps_wls_charger_driver);
}

module_exit(cps_wls_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jian.deng@convenientpower.com");
MODULE_DESCRIPTION("cps_wls_charger driver");
MODULE_ALIAS("i2c:cps_wls_charger");

/*WARNING: module cps4019_wls_charger uses symbol vfs_getattr from
 namespace VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver,
but does not import it.
*/
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
