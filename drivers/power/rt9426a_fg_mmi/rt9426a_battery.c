#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include "rt9426a_battery.h"

#define PRECISION_ENHANCE	5

/* Global Variable for RT9426A */
u16 g_PAGE_CHKSUM[14] = {0};

struct rt9426a_chip {
	struct i2c_client *i2c;
	struct device *dev;
	struct rt9426a_platform_data *pdata;
	struct power_supply *fg_psy;
	struct regmap *regmap;
	struct mutex var_lock;
	struct mutex update_lock;
	struct delayed_work update_work;
	int alert_irq;
	int capacity;
	int soc_offset;
	u8 online:1;
	int btemp;
	int bvolt;
	int bcurr;
	unsigned cyccnt;
	int soh;
	u16 ic_ver;
	int design_capacity;
	int full_capacity;
	u16 ocv_checksum_ic;
	u16 ocv_checksum_dtsi;
	bool calib_unlock;
	/* for update ieoc setting by api */
	int ocv_index;
	/* for force temperature set */
	u32 op_config_1_backup;
};

static const struct rt9426a_platform_data def_platform_data = {
	.dtsi_version = { 0, 0 },
	.para_version = 0,
	.soc_offset_size = { 2, 1 },
	.offset_interpolation_order = { 2, 2 },
	.battery_type = 4352,
	.temp_source = 0,
	.volt_source = 0,
	.curr_source = 0,
	.rs_ic_setting = 1000,  /* unit:0.01mR ; 1000x0.01 = 10mR(default)*/
	.rs_schematic = 1000,   /* unit:0.01mR ; 1000x0.01 = 10mR(default)*/
	/* add for aging cv */
	.fcc = { 2000, 2000, 2000, 2000, 2000 },
	.fc_vth = { 0x78, 0x78, 0x78, 0x78, 0x78 },
	/* add for smooth_soc */
	.smooth_soc_en = 0,   /* default: disable */
};

static int rt9426a_block_read(struct i2c_client *i2c, u8 reg, int len, void *dst)
{
	struct rt9426a_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	ret = regmap_raw_read(chip->regmap, reg, dst, len);
	if (ret < 0)
		dev_notice(chip->dev, "RT9426A block read 0x%02x fail\n", reg);
	return ret;
}

static int rt9426a_block_write(struct i2c_client *i2c,
			      u8 reg, int len, const void *src)
{
	struct rt9426a_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	ret = regmap_raw_write(chip->regmap, reg, src, len);
	if (ret < 0)
		dev_notice(chip->dev, "RT9426A block write 0x%02x fail\n", reg);
	return ret;
}


static int rt9426a_reg_read_word(struct i2c_client *i2c, u8 reg)
{
	u16 data = 0;
	int ret;

	ret = rt9426a_block_read(i2c, reg, 2, &data);
	return (ret < 0) ? ret : (s32)le16_to_cpu(data);
}

static int rt9426a_reg_write_word(struct i2c_client *i2c, u8 reg, u16 data)
{
	data = cpu_to_le16(data);
	return rt9426a_block_write(i2c, reg, 2, (uint8_t *)&data);
}

static int __maybe_unused rt9426a_reg_write_word_with_check
		(struct rt9426a_chip *chip, u8 reg, u16 data)
{
	int retry_times = 2, r_data = 0;

	while (retry_times) {
		rt9426a_reg_write_word(chip->i2c, reg, data);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(5);
		r_data = rt9426a_reg_read_word(chip->i2c, reg);
		if (data == r_data) {
			dev_info(chip->dev, "Write REG_0x%.2x Successful\n", reg);
			break;
		}
		retry_times--;
		if (retry_times == 0)
			dev_notice(chip->dev, "Write REG_0x%.2x fail\n", reg);
	}
	return r_data;
}

/* caculates page checksum & total checksum */
static int rt9426a_calculate_checksum_crc(struct rt9426a_chip *chip)
{
	u8 array_idx,i,j;
	u16 page_sum, crc_result;

	/* Calculate Page Checksum & Save to Global Array */
	for (array_idx = 0; array_idx < 14; array_idx++) {
		page_sum = 0;
		for (i = 0; i < 8; i++) {
			page_sum += chip->pdata->extreg_table[array_idx].data[2*i] +
				(chip->pdata->extreg_table[array_idx].data[2*i+1] << 8);
		}
		g_PAGE_CHKSUM[array_idx] = 0xFFFF - page_sum;
		dev_dbg(chip->dev, "RT9426A Page Checksum: Page=%d, Ckecksum=0x%x\n", array_idx,
			g_PAGE_CHKSUM[array_idx]);
	}

	/* Calculate Extend Register CRC16 & return */
	crc_result = 0xFFFF;
	for (array_idx = 0; array_idx < 14; array_idx++) {
		for (i = 0; i < 16;i++) {
			crc_result = crc_result ^ chip->pdata->extreg_table[array_idx].data[i];

			for (j = 0; j < 8; j++) {
				if(crc_result & 0x01) {
					crc_result = (crc_result >> 1) ^ 0xA001;
				} else
					crc_result = crc_result >> 1;
			}
		}
	}
	dev_dbg(chip->dev,"RT9426A Ext Reg CRC16=0x%x\n",crc_result);
	return crc_result;
}

/* get gauge total checksum value */
static int rt9426a_get_checksum(struct rt9426a_chip *chip)
{
    u8 retry_times=3,i;
    u16 regval,checksum_result;

	while (retry_times) {
		/*  Send Command to get Total Checksum */
		if(rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL,
					  RT9426A_TOTAL_CHKSUM_CMD) >= 0) {
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
			mdelay(5);
			/* Polling [BUSY] flag */
			for (i = 0; i < 10; i++) {
				regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
				if (regval & RT9426A_GAUGE_BUSY_MASK)
					mdelay(1);
				else
					break;
			}
			/* Get Total Checksum */
			checksum_result = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_TOTAL_CHKSUM);
			if((regval & RT9426A_GAUGE_BUSY_MASK)==0)
				break;
		}
		retry_times--;
		if (retry_times == 0) {
			dev_dbg(chip->dev,"RT9426A Sent Total Checksum Command Fail\n");
			return 0xFFFF;
		}
		else
			dev_dbg(chip->dev,"RT9426A Sent Total Checksum Command Retry\n");
	}

	dev_dbg(chip->dev,"Get RT9426A Total Checksum = 0x%x\n",checksum_result);
	return checksum_result;
}

static void rt9426a_read_page_cmd(struct rt9426a_chip *chip, uint8_t page)
{
	uint16_t read_page_cmd = 0x6500;
	int i, regval, retry_times = 2;

	read_page_cmd += page;

	/* confirm sending read page cmd successfully ; 2021-10-12 */
	for (i = 0; i < retry_times; i++) {
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, read_page_cmd);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, read_page_cmd);
		mdelay(5);
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW1);
		if (regval != 0xFFFF)
			break;
	}
}

static void rt9426a_write_page_cmd(struct rt9426a_chip *chip, uint8_t page)
{
	uint16_t write_page_cmd = 0x6550;

	write_page_cmd += page;
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, write_page_cmd);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, write_page_cmd);
	mdelay(5);
}

static int rt9426a_unseal_wi_retry(struct rt9426a_chip *chip)
{
	int i, regval, retry_times, ret;

	retry_times = 3;
	for (i = 0; i < retry_times; i++) {
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);

		if ((regval & RT9426A_UNSEAL_MASK) == RT9426A_UNSEAL_STATUS) {
			dev_info(chip->dev, "RT9426A Unseal Pass\n");
			ret = RT9426A_UNSEAL_PASS;
			goto out;
		} else {

			if (i >= 2) {
				dev_info(chip->dev,
					"RT9426A Unseal Fail after 3 retries\n");
				ret = RT9426A_UNSEAL_FAIL;
				goto out;
			} else if (i > 0) {
				/* print error msg instead of delay */
				dev_info(chip->dev, "RT9426A Unseal Fail Cnt = %d\n", i+1);
			}

			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL,
					       RT9426A_Unseal_Key & 0xffff);
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL,
					       RT9426A_Unseal_Key >> 16);
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
			mdelay(5);
		}
	}
	ret = RT9426A_UNSEAL_FAIL;
out:
	return ret;
}

static void rt9426a_sleep_duty_set(struct rt9426a_chip *chip, uint16_t data)
{
	int regval;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_PASS) {
		rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW7);
		regval = ((regval & 0xfff8) | (data & 0x0007));
		rt9426a_write_page_cmd(chip, RT9426A_PAGE_1);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW7, regval);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(10);
	}
}

static void rt9426a_sleep_duty_read(struct rt9426a_chip *chip)
{
	int regval;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_PASS) {
		rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW7);
		regval = (regval & 0x0007);
		dev_info(chip->dev, "Sleep_Dutty = 2^%d sec)\n", regval);
	}
}

static void rt9426a_enter_sleep(struct rt9426a_chip *chip)
{
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, 0x74AA);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
}

static void rt9426a_exit_sleep(struct rt9426a_chip *chip)
{
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, 0x7400);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
}

/* "data" is input temperature with unit = .1'C */
static void rt9426a_temperature_set(struct rt9426a_chip *chip, int data)
{
	dev_info(chip->dev, "%s: temp = %d .1'C\n", __func__, data);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_TEMP, data + 2732);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
}

/* add force temperature set */
static void __maybe_unused rt9426a_force_temp_set(bool force_en, struct rt9426a_chip *chip, int data)
{
	int regval = 0,retry_times = 3, i =0;

	/* enable force temperature set */
	if(force_en) {
		/* 1. set temperature source as host input mode */
		rt9426a_unseal_wi_retry(chip);
		for (i = 0 ; i < retry_times ; i++) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);
			regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW1);
			if((regval & RT9426A_OPCFG1_TEMP_SRC_MASK) != RT9426A_OPCFG1_TEMP_SRC_HOST) {
				rt9426a_write_page_cmd(chip, RT9426A_PAGE_1);
				regval = ((regval & ~(RT9426A_OPCFG1_TEMP_SRC_MASK))
						  |RT9426A_OPCFG1_TEMP_SRC_HOST);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW1, regval);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
				mdelay(5);
			}
			else
				break;
		}
		/* 2. set temperature as input value */
		dev_info(chip->dev, "%s: temp = %d .1'C\n", __func__, data);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_TEMP, data + 2732);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	}
	/* disable force temperature set */
	else {
		/* 1. check temp source is recovered or not */
		rt9426a_unseal_wi_retry(chip);
		for (i = 0 ; i < retry_times ; i++) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);
			regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW1);
			if(regval == chip->op_config_1_backup)
				break;
			else {
				rt9426a_write_page_cmd(chip, RT9426A_PAGE_1);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW1, chip->op_config_1_backup);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
				mdelay(5);
			}
		}
	}
}
static void rt9426a_reset(struct rt9426a_chip *chip)
{
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, 0x0041);
	mdelay(1000);
}

#if 0
static int rt9426a_get_avg_vbat(struct rt9426a_chip *chip)
{
	int regval = 0;

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_AV);
	if (regval < 0)
		return -EIO;
	return regval;
}
#endif

static int rt9426a_get_volt(struct rt9426a_chip *chip)
{
	if (chip->pdata->volt_source)
		chip->bvolt = rt9426a_reg_read_word(chip->i2c, chip->pdata->volt_source);

	return chip->bvolt;
}

static int rt9426a_get_temp(struct rt9426a_chip *chip)
{
	if (chip->pdata->temp_source) {
		chip->btemp = rt9426a_reg_read_word(chip->i2c, chip->pdata->temp_source);
		chip->btemp -= 2732;
	}

	return chip->btemp;
}

static unsigned int rt9426a_get_cyccnt(struct rt9426a_chip *chip)
{
	int ret;

	ret = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_CYC);
	if (ret < 0)
		dev_notice(chip->dev, "%s: read cycle count fail\n", __func__);
	else
		chip->cyccnt = ret;

	return chip->cyccnt;
}
static int rt9426a_get_soh(struct rt9426a_chip *chip)
{
	int ret;

	ret = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SOH);
	if (ret < 0)
		dev_notice(chip->dev, "%s: read soh fail\n", __func__);
	else
		chip->soh = ret;

	return chip->soh;
}

static int rt9426a_get_current(struct rt9426a_chip *chip)
{
	if (chip->pdata->curr_source) {
		chip->bcurr = rt9426a_reg_read_word(chip->i2c, chip->pdata->curr_source);
		if (chip->bcurr < 0)
			return -EIO;
		if (chip->bcurr > 0x7FFF) {
			chip->bcurr = 0x10000 - chip->bcurr;
			chip->bcurr = 0 - chip->bcurr;
		}
	}

	return chip->bcurr;
}
/* add pseudo sub-routine to get is_adpater_plugged */
static int rt9426a_get_is_adapter_plugged(struct rt9426a_chip *chip)
{
	/* To-Do:
	 * return 1, if charging adapter is plugged.
	 * return 0, if charging adapter is unplugged.
	 */
	return 0;
}
static int rt9426a_fg_get_offset(struct rt9426a_chip *chip, int soc, int temp);
static int rt9426a_fg_get_soc(struct rt9426a_chip *chip, int to_do_smooth_soc)
{
	int regval, capacity = 0, btemp;

	regval  = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SOC);
	if (regval < 0) {
		dev_notice(chip->dev, "read soc value fail\n");
		return -EIO;
	}
	capacity = (regval * 10);
	dev_dbg(chip->dev, "capacity before offset = %d\n", capacity);
	btemp = rt9426a_get_temp(chip);
	dev_dbg(chip->dev, "TEMP = %d\n", btemp);
	chip->soc_offset = rt9426a_fg_get_offset(chip, capacity, btemp);
	dev_dbg(chip->dev, "SOC_OFFSET = %d\n", chip->soc_offset);
	capacity += chip->soc_offset;
	dev_dbg(chip->dev, "capacity after offset = %d\n", capacity);
	if (capacity > 0)
		capacity = DIV_ROUND_UP(capacity, 10);
	else
		capacity = 0;
	if (capacity > 100)
		capacity = 100;
	dev_dbg(chip->dev, "SYS_SOC = %d\n", capacity);

	/* add for smooth_soc */
	if (to_do_smooth_soc == 1) {
		dev_info(chip->dev, "smooth soc [st, ic] = [%d, %d]\n", chip->capacity, capacity);
		dev_info(chip->dev, "smooth soc ta_sts [%d]\n", rt9426a_get_is_adapter_plugged(chip));
		if (abs(chip->capacity - capacity) >= 1) {
			if (capacity > chip->capacity) {
				if (rt9426a_get_is_adapter_plugged(chip) == 1)
					capacity = chip->capacity + 1;
				else
					capacity = chip->capacity;
			} else {
				if (rt9426a_get_current(chip) <= 0)
					capacity = chip->capacity - 1;
				else
					capacity = chip->capacity;
			}
		}
		dev_dbg(chip->dev, "SYS_SOC = %d (after smooth)\n", capacity);
	}
	return capacity;
}

static unsigned int rt9426a_get_design_capacity(struct rt9426a_chip *chip)
{
	chip->design_capacity =  chip->pdata->fcc_design;

	return chip->design_capacity;
}

static unsigned int rt9426a_get_full_capacity(struct rt9426a_chip *chip)
{
	int curr_soh = chip->soh;

	chip->full_capacity = chip->design_capacity * curr_soh  / 100;
	if (chip->full_capacity < 0) {
		return -EIO;
	}

	return chip->full_capacity;
}

static unsigned int rt9426a_get_charge_counter(struct rt9426a_chip *chip)
{
	int charge_counter;
	int full_capacity;

	full_capacity = chip->full_capacity * 1000;
	charge_counter = div_s64(full_capacity * chip->capacity, 100);

	return charge_counter;
}

/* checking cycle_cnt & bccomp */
static int rt9426a_set_cyccnt(struct rt9426a_chip *chip, unsigned int cyc_new)
{
	int ret;
	unsigned int cyccnt = 0;

	ret = rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SET_CYCCNT_KEY);
	if (ret < 0)
		dev_notice(chip->dev, "%s: send keyword to set cycle count fail\n", __func__);
	else {
		mdelay(1);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CYC, cyc_new);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(5);
		/* read back check */
		cyccnt = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_CYC);
		if (cyccnt == cyc_new) {
			ret = cyccnt;
			dev_notice(chip->dev, "%s: set cycle count to %d successfully \n", __func__, cyccnt);
		} else {
			ret = -1;
			dev_notice(chip->dev, "%s: set cycle count failed, target = %d, now = %d \n", __func__, cyc_new, cyccnt);
		}
	}

	return ret;
}

static int rt9426a_get_cycle_cnt_from_nvm(struct rt9426a_chip *chip, unsigned int *cyccnt_nvm)
{
	/* To-Do: Get backup Cycle Count from NVM and save to "cyc_nvm" */
	return 0;
}

static int rt9426a_get_bccomp(struct rt9426a_chip *chip)
{
	int regval = 0, retry_times = 2;

	while (retry_times-- > 0) {
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_BCCOMP);
		dev_notice(chip->dev, "%s: BCCOMP now is 0x%04X\n", __func__, regval);
		if ((regval > 0xCCD) && (regval < 0x7999))
			return regval;
	}

	return -EINVAL;
}

static int rt9426a_set_bccomp(struct rt9426a_chip *chip, unsigned int bccomp_new)
{
	int opcfg4 = 0, opcfg4_new = 0;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_FAIL)
		return -EINVAL;
	/* Get present OPCFG4 */
	rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);
	opcfg4 = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW4);
	opcfg4_new = opcfg4 | 0x0040;
	/* Enable SET_BCOMP by set OPCFG4 */
	rt9426a_write_page_cmd(chip, RT9426A_PAGE_1);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW4, opcfg4_new);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	mdelay(1);
	/* Set new BCCOMP */
	rt9426a_write_page_cmd(chip, RT9426A_PAGE_2);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW5, bccomp_new);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	mdelay(1);
	dev_notice(chip->dev, "%s: Set BCCOMP as 0x%04X\n", __func__, bccomp_new);
	/* Recover OPCFG4 */
	rt9426a_write_page_cmd(chip, RT9426A_PAGE_1);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW4, opcfg4);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	mdelay(1);
	/* Seal after reading */
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SEAL_CMD);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	mdelay(1);

	return 0;
}

static int rt9426a_get_bccomp_from_nvm(struct rt9426a_chip *chip, unsigned int *bccomp_nvm)
{
	/* To-Do: Get backup "BCCOMP" from NVM and save to "bccomp_nvm" */
	return 0;
}

static void __maybe_unused rt9426a_check_cycle_cnt_for_fg_ini(struct rt9426a_chip *chip)
{
	int cyccnt_fg = 0, cyccnt_nvm = 0;
	int bccomp_nvm = 0x4000;

	/* read cyccnt from fg & nvm */
	cyccnt_fg = rt9426a_get_cyccnt(chip);
	rt9426a_get_cycle_cnt_from_nvm(chip, &cyccnt_nvm);

	if (cyccnt_fg < cyccnt_nvm) {
		/* read bccomp from nvm */
		rt9426a_get_bccomp_from_nvm(chip, &bccomp_nvm);
		/* set cyccnt to fg by backup value*/
		rt9426a_set_cyccnt(chip, cyccnt_nvm);
		/* set bccomp to fg by backup value*/
		rt9426a_set_bccomp(chip, bccomp_nvm);
		dev_notice(chip->dev, "%s: recover fg cycle count from nvm, target = %d, now = %d \n", __func__, cyccnt_nvm, cyccnt_fg);
		dev_notice(chip->dev, "%s: recover fg bccomp from nvm, target = %d\n", __func__, bccomp_nvm);
	} else
		dev_notice(chip->dev, "%s: cycle counts are the same(%d)\n", __func__, cyccnt_fg);
}

static unsigned int rt9426a_get_ocv_checksum(struct rt9426a_chip *chip)
{
	int i, regval = 0, retry_times = 2;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_FAIL)
		chip->ocv_checksum_ic = 0xFFFF;
	else{
		/* confirm sending read page cmd successfully ; 2021-10-29 */
		for (i = 0; i < retry_times; i++) {
			/* get ocv checksum from ic */
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, 0xCA09);
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, 0xCA09);
			mdelay(5);
			regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW4);
			chip->ocv_checksum_ic = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW5);
			if ((regval != 0xFFFF)||(chip->ocv_checksum_ic != 0xFFFF))
				break;
		}
	}
	return chip->ocv_checksum_ic;
}
/* add for updating ieoc setting by api */
/* unit of ieoc_curr: mA ; unsigned int */
/* unit of ieoc_buff: mA ; unsigned int */
static int __maybe_unused rt9426a_write_ieoc_setting_api(struct rt9426a_chip *chip,
							unsigned int ieoc_curr,
							unsigned int ieoc_buff)
{
	int fc_setting, regval = 0, fc_ith_setting = 0;

	dev_info(chip->dev, "%s\n" ,__func__);
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG1);
	if (regval & BIT(0))
		return 0;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_FAIL)
		return -EINVAL;

	/* read fc_setting for bit operation */
	rt9426a_read_page_cmd(chip, RT9426A_PAGE_5);
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW3);

	/* calculate fc_ith by input parameter */
	/* fc_ith_setting = ((curr+buff)*rs_schematic)/(rs_ic_setting*4) */
	/* e.g. rs_ic_setting = 2.5mR ; rs_ic_setting = 1mR */
	/* fc_ith_setting = (ieoc_setting * rs_schematic)/(rs_ic_setting*4) */
	/*                = (ieoc_setting * 1)/(2.5*4) */
	fc_ith_setting = ((ieoc_curr + ieoc_buff) * (int)chip->pdata->rs_schematic);
	fc_ith_setting = DIV_ROUND_UP(fc_ith_setting, (int)chip->pdata->rs_ic_setting * 4);

	/* recombine to fc_setting by bit operation */
	fc_setting = (regval & 0xFF) | ((fc_ith_setting & 0xFF) << 8);

	dev_info(chip->dev, "fc_setting was 0x%04X ; is 0x%04X\n" ,regval, fc_setting);
	rt9426a_write_page_cmd(chip, RT9426A_PAGE_5);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW3, fc_setting);

	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, 0x0020);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	mdelay(5);

	return 0;
}

static int rt9426a_write_ocv_table(struct rt9426a_chip *chip)
{
	int retry_times, i, j, regval, rtn = RT9426A_WRITE_OCV_FAIL;
	const u32 *pval = (u32 *)chip->pdata->ocv_table + chip->ocv_index * 80;

	/* set OCV Table */
	if (*pval == 0x13) {
		retry_times = 3;
		dev_info(chip->dev, "Write NEW OCV Table\n");
		while (retry_times) {
			for (i = 0; i < 9; i++) {
				rt9426a_reg_write_word(chip->i2c,
						       RT9426A_REG_BDCNTL, 0xCB00 + i);
				rt9426a_reg_write_word(chip->i2c,
						       RT9426A_REG_BDCNTL, 0xCB00 + i);
				for (j = 0; j < 8; j++) {
					rt9426a_reg_write_word(chip->i2c,
					     0x40 + j * 2,
					     *(pval + i * 8 + j));
					mdelay(1);
				}
			}
			rt9426a_reg_write_word(chip->i2c,
					       RT9426A_REG_BDCNTL, 0xCB09);
			rt9426a_reg_write_word(chip->i2c,
					       RT9426A_REG_BDCNTL, 0xCB09);
			for (i = 0; i < 5; i++) {
				rt9426a_reg_write_word(chip->i2c, 0x40 + i * 2,
						*(pval + 9 * 8 + i));
				mdelay(1);
			}
			rt9426a_reg_write_word(chip->i2c,
					       RT9426A_REG_DUMMY, 0x0000);
			mdelay(10);
			regval = rt9426a_reg_read_word(chip->i2c,
						       RT9426A_REG_FLAG2);
			if (regval & RT9426A_USR_TBL_USED_MASK) {
				dev_info(chip->dev, "OCV Table Write Successful\n");
				rtn = RT9426A_WRITE_OCV_PASS;
				break;
			}
			retry_times--;
			if (retry_times == 0)
				dev_notice(chip->dev, "Set OCV Table fail\n");
		}
	}
	return rtn;
}

static int rt9426a_apply_pdata(struct rt9426a_chip *);
static void rt9426a_update_info(struct rt9426a_chip *chip)
{
	int regval = 0, ret = 0, retry_times = 0, i;
	struct power_supply *batt_psy;

	dev_dbg(chip->dev, "%s\n", __func__);
	mutex_lock(&chip->update_lock);

	/* get battery temp from battery power supply */
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		dev_err(chip->dev, "%s: get batt_psy fail\n", __func__);

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_FAIL)
		goto end_update_info;

	/* check if re-init is necessary */
	retry_times = 3;
	for (i = 0 ; i < retry_times ; i++) {
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
		if((regval&RT9426A_RI_MASK)&&(regval!=0xFFFF)){
			dev_info(chip->dev, "Init RT9426A by [RI]\n");
			rt9426a_apply_pdata(chip);
			break;
		}
		else if(regval!=0xFFFF)
			break;
		mdelay(5);
	}

	/* read OPCFG1~7 to check */
	rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);

	dev_info(chip->dev, "OPCFG1~5(0x%x 0x%x 0x%x 0x%x 0x%x)\n",
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW1),
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW2),
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW3),
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW4),
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW5));

	rt9426a_read_page_cmd(chip, RT9426A_PAGE_10);

	dev_info(chip->dev, "OPCFG6~7(0x%x 0x%x)\n",
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW1),
		 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW2));

	ret = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);

	rt9426a_read_page_cmd(chip, RT9426A_PAGE_2);
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW1);
	regval = (regval & 0x0300) >> 8;
	if (((ret & 0x0800) >> 11) == 1) {
		dev_info(chip->dev, "OCV table define by User\n");
	} else {
		if (regval == 0)
			dev_info(chip->dev, "OCV(4200) EDV(3200)\n");
		else if (regval == 1)
			dev_info(chip->dev, "OCV(4350) EDV(3200)\n");
		else if (regval == 2)
			dev_info(chip->dev, "OCV(4400) EDV(3200)\n");
		else
			dev_info(chip->dev, "OCV(4350) EDV(3400)\n");
	}

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW5);
	dev_info(chip->dev, "CSCOMP4(%d)\n", regval);
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW4);
	dev_info(chip->dev, "CSCOMP5(%d)\n", regval);

	dev_info(chip->dev, "DSNCAP(%d) FCC(%d)\n",
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_DC),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FCC));

	dev_info(chip->dev, "VOLT_SOURCE(0x%x) INPUT_VOLT(%d) FG_VBAT(%d) FG_OCV(%d) FG_AV(%d)\n",
		chip->pdata->volt_source, rt9426a_get_volt(chip),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_VBAT),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_OCV),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_AV));
	dev_info(chip->dev, "CURR_SOURCE(0x%x) INPUT_CURR(%d) FG_CURR(%d) FG_AI(%d)\n",
		chip->pdata->curr_source, rt9426a_get_current(chip),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_CURR),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_AI));
	dev_info(chip->dev, "TEMP_SOURCE(0x%x) INPUT_TEMP(%d) FG_TEMP(%d)\n",
			chip->pdata->temp_source, rt9426a_get_temp(chip),
			rt9426a_reg_read_word(chip->i2c, RT9426A_REG_TEMP));
	dev_info(chip->dev, "FG_FG_INTT(%d) FG_AT(%d)\n",
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_INTT),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_AT));

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG1);
	dev_info(chip->dev, "FLAG1(0x%x)\n", regval);
	if (((regval & 0x0200) >> 9) == 1)
		dev_info(chip->dev, "FC = 1\n");
	else
		dev_info(chip->dev, "FC = 0\n");

	if (((regval & 0x0004) >> 2) == 1)
		dev_info(chip->dev, "FD = 1\n");
	else
		dev_info(chip->dev, "FD = 0\n");

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
	dev_info(chip->dev, "FLAG2(0x%x)\n", regval);

	if (((regval & 0xE000) >> 13) == 0)
		dev_info(chip->dev, "Power_Mode (Active)\n");
	else if (((regval & 0xE000) >> 13) == 1)
		dev_info(chip->dev, "Power_Mode (FST_RSP_ACT)\n");
	else if (((regval & 0xE000) >> 13) == 2)
		dev_info(chip->dev, "Power_Mode (Shutdown)\n");
	else
		dev_info(chip->dev, "Power_Mode (Sleep)\n");

	if (((regval & 0x0800) >> 11) == 1)
		dev_info(chip->dev, "User_Define_Table (IN USE)\n");
	else
		dev_info(chip->dev, "User_Define_Table (NOT IN USE)\n");
	if (((regval & 0x0040) >> 6) == 1)
		dev_info(chip->dev, "Battery_Status (Inserted)\n");
	else
		dev_info(chip->dev, "Battery_Status (Removed)\n");

	if (((regval & 0x0001)) == 1)
		dev_info(chip->dev, "RLX = 1\n");
	else
		dev_info(chip->dev, "RLX = 0\n");

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
	dev_info(chip->dev, "FLAG3(0x%x)\n", regval);
	if (((regval & 0x0100) >> 8) == 1)
		dev_info(chip->dev, "RI = 1\n");
	else
		dev_info(chip->dev, "RI = 0\n");

	if (((regval & 0x0001)) == 1)
		dev_info(chip->dev, "RT9426A (Unseal)\n");
	else
		dev_info(chip->dev, "RT9426A (Seal)\n");

	dev_info(chip->dev, "CYCCNT(%d)\n", rt9426a_get_cyccnt(chip));

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SOC);
	dev_info(chip->dev, "SOC(%d)\n", regval);
	/* add for smooth_soc */
	/*chip->capacity = rt9426a_fg_get_soc(chip);*/
	chip->capacity = rt9426a_fg_get_soc(chip, chip->pdata->smooth_soc_en);

	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_RM);
	dev_info(chip->dev, "RM(%d)\n", regval);

	//regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SOH);
	regval = rt9426a_get_soh(chip);
	dev_info(chip->dev, "SOH(%d)\n", regval);

	/* rt9426a_update_ieoc_setting(chip); */

	if (batt_psy) {
		power_supply_changed(batt_psy);
		power_supply_put(batt_psy);
	}
end_update_info:
	mutex_unlock(&chip->update_lock);
	return;
}

static irqreturn_t rt9426a_irq_handler(int irqno, void *param)
{
	struct rt9426a_chip *chip = (struct rt9426a_chip *)param;

	dev_info(chip->dev, "%s\n", __func__);
	dev_info(chip->dev, "FG_IRQ(0x%x) FG_FLAG1(0x%x) FG_FLAG2(0x%x) FG_FLAG3(0x%x)\n",
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_IRQ),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG1),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2),
		rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3));

	rt9426a_update_info(chip);
	return IRQ_HANDLED;
}

enum comp_offset_typs {
	FG_COMP = 0,
	SOC_OFFSET,
	EXTREG_UPDATE,
};

static void new_vgcomp_soc_offset_datas(struct device *dev, int type,
					struct rt9426a_platform_data *pdata,
					int size_x, int size_y, int size_z)
{
	switch (type) {
	case SOC_OFFSET:
		if (pdata->soc_offset.soc_offset_data) {
			devm_kfree(dev,
				pdata->soc_offset.soc_offset_data);
			pdata->soc_offset.soc_offset_data = NULL;
		}
		if (size_x != 0 && size_y != 0)
			pdata->soc_offset.soc_offset_data =
				devm_kzalloc(dev,
				    size_x * size_y * sizeof(struct data_point),
				    GFP_KERNEL);
		if (pdata->soc_offset.soc_offset_data) {
			pdata->soc_offset.soc_voltnr = size_x;
			pdata->soc_offset.tempnr = size_y;

		} else {
			pdata->soc_offset.soc_voltnr = 0;
			pdata->soc_offset.tempnr = 0;
		}
		break;
	default:
		WARN_ON(1);
	}
}

struct submask_condition {
	int x, y, z;
	int order_x, order_y, order_z;
	int xnr, ynr, znr;
	const struct data_point *mesh_src;
};

static inline const struct data_point *get_mesh_data(
	int i, int j, int k,
	const struct data_point *mesh, int xnr, int ynr)
{
	return mesh + k * ynr * xnr + j * xnr + i;
}

static int get_sub_mesh(int state, struct data_point *mesh_buffer,
				struct submask_condition *condition)
{
	int i = 0, j = 0, k = 0, x = 0, y = 0, z = 0;

	x = condition->x;
	y = condition->y;
	z = condition->z;
	for (i = 0; i < condition->xnr; ++i) {
		if (get_mesh_data(i, 0, 0, condition->mesh_src,
					condition->xnr, condition->ynr)->x >= x)
			break;
	}
	for (; i >= 0 && i < condition->xnr; --i) {
		if (get_mesh_data(i, 0, 0, condition->mesh_src,
					condition->xnr, condition->ynr)->x <= x)
			break;
	}

	for (j = 0; j < condition->ynr; ++j) {
		if (get_mesh_data(0, j, 0, condition->mesh_src,
					condition->xnr, condition->ynr)->y >= y)
			break;
	}
	for (; j >= 0 && j < condition->ynr; --j) {
		if (get_mesh_data(0, j, 0, condition->mesh_src,
					condition->xnr, condition->ynr)->y <= y)
			break;
	}

	if (state == FG_COMP) {
		for (k = 0; k < condition->znr; ++k) {
			if (get_mesh_data(0, 0, k, condition->mesh_src,
					condition->xnr, condition->ynr)->z >= z)
				break;
		}
		for (; k >= 0 && k < condition->znr; --k) {
			if (get_mesh_data(0, 0, k, condition->mesh_src,
					condition->xnr, condition->ynr)->z <= z)
				break;
		}
	}

	i -= ((condition->order_x - 1) / 2);
	j -= ((condition->order_y - 1) / 2);
	k -= ((condition->order_z - 1) / 2);

	if (i <= 0)
		i = 0;
	if (j <= 0)
		j = 0;
	if (k <= 0)
		k = 0;
	if ((i + condition->order_x) > condition->xnr)
		i = condition->xnr - condition->order_x;
	if ((j + condition->order_y) > condition->ynr)
		j = condition->ynr - condition->order_y;
	if ((k + condition->order_z) > condition->znr)
		k = condition->znr - condition->order_z;

	if (state == FG_COMP) {
		for (z = 0; z < condition->order_z; ++z) {
			for (y = 0; y < condition->order_y; ++y) {
				for (x = 0; x < condition->order_x; ++x) {
					*(mesh_buffer + z * condition->order_y *
							condition->order_z +
						y * condition->order_x + x)
						= *get_mesh_data(i + x, j + y,
							k + z,
							condition->mesh_src,
							condition->xnr,
							condition->ynr);
				}
			}
		}
	} else {
		for (y = 0; y < condition->order_y; ++y) {
			for (x = 0; x < condition->order_x; ++x) {
				*(mesh_buffer + y * condition->order_x + x)
					= *get_mesh_data(i + x, j + y, 0,
						condition->mesh_src,
						condition->xnr,
						condition->ynr);
			}
		}
	}
	return 0;
}

static int offset_li(int xnr, int ynr,
			const struct data_point *mesh, int x, int y)
{
	long long retval = 0;
	int i, j, k;
	long long wm, wd;
	const struct data_point *cache;

	for (i = 0; i < xnr; ++i) {
		for (j = 0; j < ynr; ++j) {
			wm = wd = 1;
			cache = get_mesh_data(i, j, 0, mesh, xnr, ynr);
			for (k = 0; k < xnr; ++k) {
				if (i != k) {
					wm *= (x - get_mesh_data(k, j, 0,
							mesh, xnr, ynr)->x);
					wd *= (cache->x - get_mesh_data(k, j, 0,
							mesh, xnr, ynr)->x);
				}
			}
			for (k = 0; k < ynr; ++k) {
				if (j != k) {
					wm *= (y - get_mesh_data(i, k, 0,
							mesh, xnr, ynr)->y);
					wd *= (cache->y - get_mesh_data(i, k, 0,
							mesh, xnr, ynr)->y);
				}
			}
			retval += div64_s64(
				((cache->w * wm) << PRECISION_ENHANCE), wd);
		}
	}
	return (int)((retval + (1 << (PRECISION_ENHANCE - 1)))
							>> PRECISION_ENHANCE);
}

static int rt9426a_fg_get_offset(struct rt9426a_chip *chip, int soc_val, int temp)
{
	const int ip_x = chip->pdata->offset_interpolation_order[0];
	const int ip_y = chip->pdata->offset_interpolation_order[1];
	struct data_point *sub_mesh;
	int xnr, ynr;
	int offset = 0;
	struct soc_offset_table *offset_table = NULL;
	struct submask_condition condition = {
		.x = soc_val,
		.y = temp,
	};

	mutex_lock(&chip->var_lock);
	offset_table = &chip->pdata->soc_offset;
	xnr = offset_table->soc_voltnr;
	ynr = offset_table->tempnr;
	if (xnr == 0 || ynr == 0)
		goto out;
	condition.order_x = min(ip_x, xnr);
	condition.order_y = min(ip_y, ynr);
	condition.xnr = xnr;
	condition.ynr = ynr;
	condition.mesh_src = offset_table->soc_offset_data;
	sub_mesh = kzalloc(ip_x * ip_y * sizeof(struct data_point), GFP_KERNEL);
	if (!sub_mesh)
		goto out;
	get_sub_mesh(SOC_OFFSET, sub_mesh, &condition);
	offset = offset_li(condition.order_x, condition.order_y, sub_mesh,
			   soc_val, temp);
	kfree(sub_mesh);
out:
	mutex_unlock(&chip->var_lock);
	return offset;
}

static int rt_fg_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct rt9426a_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->bvolt  * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->pdata->battery_type;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* add for smooth_soc */
		/*chip->capacity = rt9426a_fg_get_soc(chip); */
		val->intval = chip->capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = rt9426a_get_design_capacity(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chip->bcurr * 1000 * (-1);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->btemp;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->cyccnt;
		break;
	/* add for aging cv */
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = chip->ocv_index;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = rt9426a_get_full_capacity(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = rt9426a_get_charge_counter(chip);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int rt_fg_set_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	struct rt9426a_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
		rt9426a_temperature_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (val->intval < 0)
			return -EINVAL;
		rt9426a_set_cyccnt(chip, val->intval);
		break;
	/* add for aging cv */
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		if (val->intval < 0 || val->intval > 4)
			return -EINVAL;
		chip->ocv_index = val->intval;
		/* save to RSVD2 after ocv_index assigned */
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_RSVD2, chip->ocv_index);
		if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_FAIL)
			return -EIO;
		mutex_lock(&chip->update_lock);
		/* write aging ocv table */
		rt9426a_write_ocv_table(chip);
		/* write aging fcc */
		rt9426a_write_page_cmd(chip, RT9426A_PAGE_2);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW7, chip->pdata->fcc[chip->ocv_index]);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(5);
		/* write aging fc_vth */
		rt9426a_write_page_cmd(chip, RT9426A_PAGE_5);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW3,
				(chip->pdata->fc_vth[chip->ocv_index]) | (chip->pdata->extreg_table[5].data[5] << 8));
		mutex_unlock(&chip->update_lock);
		/* update ocv checksum */
		chip->ocv_checksum_dtsi = *((u32 *)chip->pdata->ocv_table +
	                          (chip->ocv_index * 80) + RT9426A_IDX_OF_OCV_CKSUM);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int rt_fg_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	/* add for aging cv */
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		return true;
	default:
		return false;
	}
}

static enum power_supply_property rt_fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static struct power_supply_desc fg_psy_desc = {
	.name = "bms",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = rt_fg_props,
	.num_properties = ARRAY_SIZE(rt_fg_props),
	.get_property = rt_fg_get_property,
	.set_property = rt_fg_set_property,
	.property_is_writeable = rt_fg_property_is_writeable,
};

static int __maybe_unused rt9426a_get_calibration_para(struct rt9426a_chip *chip, u8 *curr_offs, u8 *curr_gain, u8 *volt_gain)
{
	/* To-Do: get calibration parameter(curr_offs/curr_gain/volt_gain)
              from platform NVM */
	return 0;
}

static int rt9426a_enter_calibration_mode(struct rt9426a_chip *chip)
{
	int i, regval, retry_times, ret;
	int j, tick_old = 0, tick_new = 0;

	retry_times = 3;
	for (i = 0; i < retry_times; i++) {
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);

		if ((regval & RT9426A_CALI_MODE_MASK) == RT9426A_CALI_MODE_MASK) {
			dev_info(chip->dev, "RT9426 is in Calibration Mode\n");
			ret = RT9426A_CALI_MODE_PASS;
			goto out;
		} else {
			if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_PASS){
				/* Check System Tick before entering Calibration */
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SYS_TICK_ON_CMD);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
				mdelay(5);
				tick_old = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_ADV);
				for (j = 0; j < 1000; j++) {
					tick_new = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_ADV);
					if (tick_old != tick_new) {
						mdelay(300);
						break;
					}
					mdelay(5);
				}
				/* Send Cmd to Enter Calibration */
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_CALI_ENTR_CMD);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
				/* Disable System Tick */
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SYS_TICK_OFF_CMD);
				if (i >= 2) {
					dev_info(chip->dev,
						"RT9426 Enter Calibration Mode Fail after 3 retries\n");
					ret = RT9426A_CALI_MODE_FAIL;
					goto out;
				}
				mdelay(5);
			}
		}
	}
	ret = RT9426A_CALI_MODE_FAIL;
out:
	return ret;
}

static void rt9426a_exit_calibration_mode(struct rt9426a_chip *chip)
{
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_CALI_EXIT_CMD);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
}

static void rt9426a_apply_sense_resistor(struct rt9426a_chip *chip)
{
	int op_config10 = 0, rsense = 0;
	rsense = chip->pdata->rs_ic_setting / RT9426A_NEW_RS_UNIT;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_PASS){
		/* get op_config10 */
		rt9426a_read_page_cmd(chip, RT9426A_PAGE_10);
		op_config10 = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW3);
		/* update rsense to op_config10 */
		op_config10 = (RT9426A_PAGE_10&0xFF00) | (rsense&0xFF);
		/* apply op_config10 */
		rt9426a_write_page_cmd(chip, RT9426A_PAGE_10);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW3, op_config10);
		mdelay(5);
	}
}

static void rt9426a_apply_calibration_para(struct rt9426a_chip *chip, u8 curr_offs, u8 curr_gain, u8 volt_gain)
{
	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_PASS){
		rt9426a_write_page_cmd(chip, RT9426A_PAGE_0);
		/* set Current system gain & offset */
		if((curr_gain != 0x00)&&(curr_gain != 0xFF)&&
				(curr_offs != 0x00)&&(curr_offs != 0xFF)){
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW1,
						curr_gain | (curr_offs<<8));
		}
		else{
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW1, 0x8080);
		}
		/* set Voltage system gain */
		if((volt_gain != 0x00)&&(volt_gain != 0xFF)){
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW7,
						0x88 | (volt_gain<<8));
		}
		else{
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW7, 0x8088);
		}
		mdelay(5);
	}
}
static void __maybe_unused rt9426a_assign_calibration_para(struct rt9426a_chip *chip, u8 curr_offs, u8 curr_gain, u8 volt_gain)
{
	/* Check and assign Current system gain & offset */
	if((curr_gain != 0x00)&&(curr_gain != 0xFF)&&
		(curr_offs != 0x00)&&(curr_offs != 0xFF)){
		chip->pdata->extreg_table[0].data[0] = curr_gain;
		chip->pdata->extreg_table[0].data[1] = curr_offs;
		dev_info(chip->dev,
				"RT9426A assign calibrated current parameter before application\n");
	}
	else{
		chip->pdata->extreg_table[0].data[0] = 0x80;
		chip->pdata->extreg_table[0].data[1] = 0x80;
		dev_info(chip->dev,
				"RT9426A assign default current parameter before application\n");
	}
	/* Check and assign Voltage system gain */
	if((volt_gain != 0x00)&&(volt_gain != 0xFF)){
		chip->pdata->extreg_table[0].data[13] = volt_gain;
		dev_info(chip->dev,
				"RT9426A assign calibrated voltage parameter before application\n");
	}
	else{
		chip->pdata->extreg_table[0].data[13] = 0x80;
		dev_info(chip->dev,
				"RT9426A assign default voltage parameter before application\n");
	}

}

static int rt9426a_get_curr_by_conversion(struct rt9426a_chip *chip)
{
	int regval=0;

	if(rt9426a_enter_calibration_mode(chip)==RT9426A_CALI_MODE_PASS){
		/* Start current convertion */
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_CURR_CONVERT_CMD);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(50);

		/* Get convert result */
		regval  = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_CURR);
		if (regval < 0)
			return -EIO;
		if (regval > 0x7FFF) {
			regval = 0x10000 - regval;
			regval = 0 - regval;
		}
	}

	/* scaling for the current */
	regval = regval * (int)chip->pdata->rs_ic_setting / (int)chip->pdata->rs_schematic;
	dev_info(chip->dev, "CALIB_CURRENT = %d mA\n",regval);

	return regval;
}

static int rt9426a_get_volt_by_conversion(struct rt9426a_chip *chip)
{
	int regval=0;
	if(rt9426a_enter_calibration_mode(chip)==RT9426A_CALI_MODE_PASS){
		/* Start voltage convertion */
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_VOLT_CONVERT_CMD);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(50);

		/* Get voltage result */
		regval  = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_VBAT);
	}
	return regval;
}

/* To Disable power path */
static int rt9426a_request_charging_inhibit(bool needInhibit)
{
	if(needInhibit){
		/* To-Do: Turn OFF charging power path to battery */
	}
	else{
		/* To-Do: Turn ON charging power path to battery */
	}
	return 0;
}

static int rt9426a_enter_shutdown_mode(struct rt9426a_chip *chip)
{
	int regval,loop;
	for(loop=0;loop<5;loop++){
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
		if(regval&RT9426A_SHDN_MASK)
			break;
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SHDN_ENTR_CMD);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(20);
	}
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
	if(!(regval&RT9426A_SHDN_MASK)){
		dev_info(chip->dev, "RT9426A Enter Shutdown Fail\n");
		return -1;
	}
	dev_info(chip->dev, "RT9426A Enter Shutdown Success\n");
	return 0;
}

static int rt9426a_exit_shutdown_mode(struct rt9426a_chip *chip)
{
	int regval,loop;
	int cmd_cnt = 0;
	for(loop=0;loop<5;loop++){
		regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
		if(!(regval&RT9426A_SHDN_MASK))
			break;
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SHDN_EXIT_CMD);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(250);
		dev_info(chip->dev, "RT9426A Send Exit Shutdown Cmd Count = %d\n",++cmd_cnt);
	}
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
	if(regval&RT9426A_SHDN_MASK){
		dev_info(chip->dev, "RT9426A is in Shutdown\n");
		return -1;
	}
	dev_info(chip->dev, "RT9426A is not in Shutdown\n");

	if (cmd_cnt == 0)
		goto out;

	/* Power path control check */
	regval = rt9426a_get_current(chip);
	if(regval>0){
		/* Disable power path */
		rt9426a_request_charging_inhibit(true);
		dev_info(chip->dev, "RT9426A request to enable charging inhibit\n");
		mdelay(1250);
	}
	/* Send QS Command to get INI SOC */
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, 0x4000);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	dev_info(chip->dev, "Send QS after exiting Shutdown\n");
	mdelay(5);
	/* Power path recover check */
	if(regval>0){
		/* To Enable power path */
		rt9426a_request_charging_inhibit(false);
		dev_info(chip->dev, "RT9426A request to disable charging inhibit\n");
	}
out:
	return 0;
}

static ssize_t rt_fg_show_attrs(struct device *, struct device_attribute *, char *);
static ssize_t rt_fg_store_attrs(struct device *, struct device_attribute *, const char *,
				 size_t count);

#define RT_FG_ATTR(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0664},		\
	.show = rt_fg_show_attrs,			\
	.store = rt_fg_store_attrs,			\
}

static struct device_attribute rt_fuelgauge_attrs[] = {
	RT_FG_ATTR(fg_temp),
	RT_FG_ATTR(volt),
	RT_FG_ATTR(curr),
	RT_FG_ATTR(update),
	RT_FG_ATTR(ocv_table),
	RT_FG_ATTR(enter_sleep),
	RT_FG_ATTR(exit_sleep),
	RT_FG_ATTR(set_sleep_duty),
	RT_FG_ATTR(DBP0),
	RT_FG_ATTR(DBP1),
	RT_FG_ATTR(DBP2),
	RT_FG_ATTR(DBP3),
	RT_FG_ATTR(DBP4),
	RT_FG_ATTR(DBP5),
	RT_FG_ATTR(DBP6),
	RT_FG_ATTR(DBP7),
	RT_FG_ATTR(DBP8),
	RT_FG_ATTR(DBP9),
	RT_FG_ATTR(WCNTL),
	RT_FG_ATTR(WEXTCNTL),
	RT_FG_ATTR(WSW1),
	RT_FG_ATTR(WSW2),
	RT_FG_ATTR(WSW3),
	RT_FG_ATTR(WSW4),
	RT_FG_ATTR(WSW5),
	RT_FG_ATTR(WSW6),
	RT_FG_ATTR(WSW7),
	RT_FG_ATTR(WSW8),
	RT_FG_ATTR(WTEMP),
	RT_FG_ATTR(UNSEAL),
	RT_FG_ATTR(FG_SET_TEMP),
	RT_FG_ATTR(FG_RESET),
	RT_FG_ATTR(CALIB_SECURE),
	RT_FG_ATTR(CALIB_RSENSE),
	RT_FG_ATTR(CALIB_CURRENT),
	RT_FG_ATTR(CALIB_VOLTAGE),
	RT_FG_ATTR(CALIB_FACTOR),
	RT_FG_ATTR(FORCE_SHUTDOWN),
	RT_FG_ATTR(BCCOMP),
};

enum {
	FG_TEMP = 0,
	FG_VOLT,
	FG_CURR,
	FG_UPDATE,
	OCV_TABLE,
	ENTER_sleep,
	EXIT_sleep,
	SET_sleep_DUTY,
	DBP0,
	DBP1,
	DBP2,
	DBP3,
	DBP4,
	DBP5,
	DBP6,
	DBP7,
	DBP8,
	DBP9,
	WCNTL,
	WEXTCNTL,
	WSW1,
	WSW2,
	WSW3,
	WSW4,
	WSW5,
	WSW6,
	WSW7,
	WSW8,
	WTEMP,
	UNSEAL,
	FG_SET_TEMP,
	FG_RESET,
	CALIB_SECURE,
	CALIB_RSENSE,
	CALIB_CURRENT,
	CALIB_VOLTAGE,
	CALIB_FACTOR,
	FORCE_SHUTDOWN,
	BCCOMP,
};

static ssize_t rt_fg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rt9426a_chip *chip = dev_get_drvdata(dev->parent);
	const ptrdiff_t offset = attr - rt_fuelgauge_attrs;
	int i = 0;

	switch (offset) {
	case FG_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chip->btemp);
		break;
	case FG_VOLT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chip->bvolt);
		break;
	case FG_CURR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chip->bcurr);
		break;
	case CALIB_SECURE:
		i = scnprintf(buf, PAGE_SIZE, chip->calib_unlock ? "Unlocked\n" : "Locked\n");
		break;
	case CALIB_CURRENT:
		if (!chip->calib_unlock)
			return -EACCES;
		rt9426a_enter_calibration_mode(chip);
		i = scnprintf(buf, PAGE_SIZE, "%d\n", rt9426a_get_curr_by_conversion(chip));
		rt9426a_exit_calibration_mode(chip);
		break;
	case CALIB_VOLTAGE:
		if (!chip->calib_unlock)
			return -EACCES;
		rt9426a_enter_calibration_mode(chip);
		i = scnprintf(buf, PAGE_SIZE, "%d\n", rt9426a_get_volt_by_conversion(chip));
		rt9426a_exit_calibration_mode(chip);
		break;
	case BCCOMP:
		i = scnprintf(buf, PAGE_SIZE, "%d\n", rt9426a_get_bccomp(chip));
		break;
	case FG_UPDATE:
	case OCV_TABLE:
	case ENTER_sleep:
	case EXIT_sleep:
	case SET_sleep_DUTY:
	case DBP0:
	case DBP1:
	case DBP2:
	case DBP3:
	case DBP4:
	case DBP5:
	case DBP6:
	case DBP7:
	case DBP8:
	case DBP9:
	case WCNTL:
	case WEXTCNTL:
	case WSW1:
	case WSW2:
	case WSW3:
	case WSW4:
	case WSW5:
	case WSW6:
	case WSW7:
	case WSW8:
	case WTEMP:
	case UNSEAL:
	case FG_SET_TEMP:
	case FG_RESET:
	default:
		i = -EINVAL;
		break;
	}
	return i;
}

static ssize_t rt_fg_store_attrs(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct rt9426a_chip *chip = dev_get_drvdata(dev->parent);
	const ptrdiff_t offset = attr - rt_fuelgauge_attrs;
	int ret = 0;
	int x, y = 0;
	char *org, *token, *cur;
	int temp[8];
	int val;

	switch (offset) {
	case FG_TEMP:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		chip->btemp = val;
		ret = count;
		break;
	case FG_VOLT:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		chip->bvolt = val;
		ret = count;
		break;
	case FG_CURR:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		chip->bcurr = val;
		ret = count;
		break;
	case FG_UPDATE:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_update_info(chip);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case OCV_TABLE:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			for (x = 0; x < 10; x++) {
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, 0xCA00 + x);
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, 0xCA00 + x);
				for (y = 0; y < 8; y++)
					temp[y] =
						rt9426a_reg_read_word(chip->i2c,
								      RT9426A_REG_SWINDOW1 + y * 2);

				dev_info(dev, "fg_ocv_table_%d\n", x);
				dev_info(dev, "<0x%x 0x%x 0x%x 0x%x>\n",
					 temp[0], temp[1], temp[2], temp[3]);
				dev_info(dev, "<0x%x 0x%x 0x%x 0x%x>\n",
					 temp[4], temp[5], temp[6], temp[7]);
			}
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case ENTER_sleep:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_enter_sleep(chip);
			dev_info(dev, "SLP_STS = %d\n",
				 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2) >> 15);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case EXIT_sleep:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_exit_sleep(chip);
			dev_info(dev, "SLP_STS = %d\n",
				 rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2) >> 15);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case SET_sleep_DUTY:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_sleep_duty_set(chip, val);
		rt9426a_sleep_duty_read(chip);
		ret = count;
		break;
	case DBP0:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_0);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
						   RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page0");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP1:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_1);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
						   RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page1");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP2:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_2);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page2");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP3:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_3);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page3");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP4:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_4);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page4");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP5:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_5);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page5");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP6:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_6);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page6");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP7:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_7);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page7");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP8:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_8);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x * 2);

			dev_info(dev, "fg_data_block_page8");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case DBP9:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9426a_read_page_cmd(chip, RT9426A_PAGE_9);
			mdelay(5);
			for (x = 0; x < 8; x++)
				temp[x] = rt9426a_reg_read_word(chip->i2c,
								RT9426A_REG_SWINDOW1 + x*2);

			dev_info(dev, "fg_data_block_page9");
			dev_info(dev, "0x41:<0x%x>\n", temp[0]);
			dev_info(dev, "0x43:<0x%x>\n", temp[1]);
			dev_info(dev, "0x45:<0x%x>\n", temp[2]);
			dev_info(dev, "0x47:<0x%x>\n", temp[3]);
			dev_info(dev, "0x49:<0x%x>\n", temp[4]);
			dev_info(dev, "0x4B:<0x%x>\n", temp[5]);
			dev_info(dev, "0x4D:<0x%x>\n", temp[6]);
			dev_info(dev, "0x4F:<0x%x>\n", temp[7]);
			ret = count;
		} else
			ret = -EINVAL;
		break;
	case WCNTL:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WEXTCNTL:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, val);
		ret = count;
		break;
	case WSW1:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW1, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW2:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW2, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW3:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW3, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW4:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW4, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW5:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW5, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW6:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW6, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW7:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW7, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WSW8:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW8, val);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case WTEMP:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_reg_write_word(chip->i2c,
					RT9426A_REG_TEMP, ((val*10)+2732));
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		ret = count;
		break;
	case UNSEAL:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		ret = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
		if ((ret & 0x0001) == 0) {
			/* Unseal RT9426A */
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL,
						(RT9426A_Unseal_Key & 0xffff));
			rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL,
						(RT9426A_Unseal_Key >> 16));
			rt9426a_reg_write_word(chip->i2c,
						RT9426A_REG_DUMMY, 0x0000);
			mdelay(10);

			ret = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
			if ((ret & 0x0001) == 0)
				dev_info(dev, "RT9426A Unseal Fail\n");
			else
				dev_info(dev, "RT9426A Unseal Pass\n");
		} else
			dev_info(dev, "RT9426A Unseal Pass\n");

		/* Unseal RT9426A */
		ret = count;
		break;
	case FG_SET_TEMP:
		ret = kstrtoint(buf, 10, &val);
		if (ret < 0) {
			dev_notice(dev, "get paremters fail\n");
			ret = -EINVAL;
		}
		rt9426a_temperature_set(chip, val);
		ret = count;
		break;
	case FG_RESET:
		rt9426a_reset(chip);
		ret = count;
		break;
	case CALIB_SECURE:
		if (kstrtoint(buf, 0, &x))
			return -EINVAL;
		chip->calib_unlock = (x == 5526789);
		ret = count;
		break;
	case CALIB_RSENSE:
		if (!chip->calib_unlock)
			return -EACCES;
		/* set rsense according to dtsi */
		rt9426a_apply_sense_resistor(chip);
		rt9426a_apply_calibration_para(chip, 0x80, 0x80, 0x80);
		ret = count;
		break;
	case CALIB_FACTOR:
		if (!chip->calib_unlock)
			return -EACCES;
		org = kstrdup(buf, GFP_KERNEL);
		if (!org)
			return -ENOMEM;
		cur = org;
		x = 0;
		while ((token = strsep(&cur, ",")) != NULL) {
			if (kstrtoint(token, 0, &temp[x]))
				break;
			if (++x >= 3)
				break;
		}
		kfree(org);
		if (x != 3)
			return -EINVAL;
		rt9426a_apply_calibration_para(chip, temp[0], temp[1], temp[2]);
		ret = count;
		break;
	case FORCE_SHUTDOWN:
		if (kstrtoint(buf, 0, &x) || x < 0 || x > 1)
			return -EINVAL;

		ret = x ? rt9426a_enter_shutdown_mode(chip) : rt9426a_exit_shutdown_mode(chip);
		if (ret)
			return ret;
		ret = count;
		break;
	case BCCOMP:
		if (kstrtoint(buf, 0, &x) || x < 0xCCD || x > 0x7999)
			return -EINVAL;

		ret = rt9426a_set_bccomp(chip, x);
		if (ret)
			return ret;
		ret = count;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rt_fg_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(rt_fuelgauge_attrs); i++) {
		rc = device_create_file(dev, &rt_fuelgauge_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	dev_notice(dev, "%s: failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &rt_fuelgauge_attrs[i]);
create_attrs_succeed:
	return rc;
}

static int rt9426a_irq_enable(struct rt9426a_chip *chip, bool enable)
{
	int regval;
	int opcfg2 = 0;

	opcfg2 = chip->pdata->extreg_table[1].data[3] * 256 +
			 chip->pdata->extreg_table[1].data[2];
	dev_info(chip->dev, "OPCFG2 = 0x%04x\n", opcfg2);

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_PASS) {
		rt9426a_write_page_cmd(chip, RT9426A_PAGE_1);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_SWINDOW2,
				       enable ? opcfg2 : 0);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(5);
		/* if disable, force clear irq status */
		if (!enable) {
			regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_IRQ);
			dev_info(chip->dev, "previous irq status 0x%04x\n", regval);
		}
	}

	return 0;
}

static int rt9426a_irq_init(struct rt9426a_chip *chip)
{
	int rc = 0;

	dev_info(chip->dev, "%s\n", __func__);
	chip->alert_irq = chip->i2c->irq;
	rc = devm_request_threaded_irq(chip->dev, chip->alert_irq, NULL,
				       rt9426a_irq_handler,
				       IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				       "rt9426a_fg_irq", chip);
	if (rc < 0) {
		dev_notice(chip->dev, "irq register failed\n");
		return rc;
	}
	device_init_wakeup(chip->dev, true);

	return 0;
}

static int rt9426a_irq_deinit(struct rt9426a_chip *chip)
{
	device_init_wakeup(chip->dev, false);
	return 0;
}

static int rt9426a_apply_pdata(struct rt9426a_chip *chip)
{
	int regval = 0, i, retry_times, ret = RT9426A_INIT_PASS, i2c_retry_times;
	int reg_flag2 = 0, reg_flag3 = 0, total_checksum_retry = 0;
	u16 total_checksum_target=0,fg_total_checksum=0;
	u16 page_checksum=0;
	u16 page_idx_cmd=0, extend_reg_data=0;
	u8 page_idx=0,extend_reg_cmd_addr=0, array_idx=0;
	int volt_now = 0, fd_vth_now = 0, fd_threshold = 0;
	int wt_ocv_result = RT9426A_WRITE_OCV_FAIL;

	dev_info(chip->dev, "%s\n", __func__);

	/* read ocv_index before using it */
	chip->ocv_index = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_RSVD2);
	dev_info(chip->dev, "%s: ocv_index = %d\n",
			    __func__, chip->ocv_index);
	/* update ocv_checksum_dtsi after ocv_index updated */
	chip->ocv_checksum_dtsi = *((u32 *)chip->pdata->ocv_table +
	                          (chip->ocv_index * 80) + RT9426A_IDX_OF_OCV_CKSUM);
	/* add for aging cv ; udpate fcc before writing */
	chip->pdata->extreg_table[2].data[12] = chip->pdata->fcc[chip->ocv_index] & 0xff;
	chip->pdata->extreg_table[2].data[13] = (chip->pdata->fcc[chip->ocv_index] >> 8) & 0xff;
	/* add for aging cv ; udpate fc_vth before writing */
	chip->pdata->extreg_table[4].data[4]= chip->pdata->fc_vth[chip->ocv_index];

	/* backup opcfg1 */
	chip->op_config_1_backup = (chip->pdata->extreg_table[1].data[1] * 256) +
						  chip->pdata->extreg_table[1].data[0];
	retry_times = 30;
	for (i = 0 ; i < retry_times ; i++) {
		/* Check [RDY]@Reg-Flag2 & [RI]@Reg-Flag3 */
		reg_flag2 = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
		reg_flag3 = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
		if(((reg_flag2&RT9426A_RDY_MASK)&&(reg_flag2!=0xFFFF))||
		    ((reg_flag3&RT9426A_RI_MASK)&&(reg_flag3!=0xFFFF))) {
			/* Exit Shutdown Mode before any action */
			/* rt9426a_exit_shutdown_mode(chip); */
            /* Check [RI]@Reg-Flag3 & fg total checksum */
			regval = rt9426a_get_checksum(chip);
			total_checksum_target = rt9426a_calculate_checksum_crc(chip);
			/* get ocv checksum from fg ic before applying ocv table */
			rt9426a_get_ocv_checksum(chip);
			if(reg_flag3&RT9426A_RI_MASK){
				dev_info(chip->dev, "Init RT9426A by [RI]\n");
				break;
			}
			else if(regval != total_checksum_target){
				dev_info(chip->dev, "Force Init RT9426A by total checksum\n");
				dev_info(chip->dev, "IC: 0x%04X ; DTSI: 0x%04X \n", regval, total_checksum_target);
				break;
			}
			else if (chip->ocv_checksum_ic != chip->ocv_checksum_dtsi){
				dev_info(chip->dev, "Force Init RT9426A by OCV checksum\n");
				dev_info(chip->dev, "IC: 0x%04X ; DTSI: 0x%04X \n", chip->ocv_checksum_ic , chip->ocv_checksum_dtsi);
				break;
			}
			else{
				dev_info(chip->dev, "No need to Init RT9426A\n");
				ret = RT9426A_INIT_BYPASS;
				break;
			}
		}
		else if(i == retry_times-1){
			dev_info(chip->dev, "RT9426A Check [RDY] fail.\n");
		}
		mdelay(5);
	}
	/* Bypass to apply RT9426A parameter */
	if(ret < RT9426A_INIT_PASS)
		goto BYPASS_EXT_REG;

	if (rt9426a_unseal_wi_retry(chip) == RT9426A_UNSEAL_FAIL){
		ret = RT9426A_INIT_UNSEAL_ERR;
		goto END_INIT;
	}

	/*NOTICE!! Please follow below setting sequence & order */
	/* write ocv table only when checksum matched */
	/* add for aging cv */
	if(chip->ocv_checksum_ic != chip->ocv_checksum_dtsi)
		wt_ocv_result = rt9426a_write_ocv_table(chip);

	/* Read Reg IRQ to reinital Alert pin state */
	rt9426a_reg_read_word(chip->i2c, RT9426A_REG_IRQ);

	/* get calibration parameter */
	/* rt9426a_get_calibration_para(chip, &target_curr_offs, &target_curr_gain, &target_volt_gain); */
	/* assign calibration parameter */
	/* rt9426a_assign_calibration_para(chip, target_curr_offs, target_curr_gain, target_volt_gain); */

TOTAL_CHECKSUM_CHECK:

	/* Calculate Checksum & CRC */
	total_checksum_target = rt9426a_calculate_checksum_crc(chip);

	/* Get Total Checksum from RT9426A */
	fg_total_checksum = rt9426a_get_checksum(chip);

	/* Compare Total Checksum with Target */
	dev_info(chip->dev, "Total Checksum: Target = 0x%x, Result = 0x%x\n",
		 total_checksum_target, fg_total_checksum);

	if (fg_total_checksum == total_checksum_target) {
		ret = RT9426A_INIT_BYPASS;
		goto BYPASS_EXT_REG;
	}
	else if (total_checksum_retry > 5) {
		dev_info(chip->dev, "Write RT9426A Extend Register Fail\n");
		ret = RT9426A_INIT_CKSUM_ERR;
		retry_times = 0;
		goto SET_SEAL_CMD;
	}
	total_checksum_retry++;

	/* Set Ext Register */
	for (page_idx = 0; page_idx < 16; page_idx++) {
		if((page_idx != 4) && (page_idx != 9)) {
			/* retry count for i2c fail only */
			i2c_retry_times = 0;
PAGE_CHECKSUM1:
			/* Send Write Page Command */
			page_idx_cmd = RT9426A_WPAGE_CMD | page_idx;
			if (rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, page_idx_cmd) >= 0) {
				rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, page_idx_cmd);
				retry_times = 0;
PAGE_CHECKSUM2:
				mdelay(5);
				/* Get Page Checksum from RT9426A */
				page_checksum = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_SWINDOW9);
				if (page_idx < 4)
					array_idx = page_idx;
				else if ((page_idx > 4) && (page_idx < 9))
					array_idx = page_idx - 1;
				else if (page_idx > 9)
					array_idx = page_idx - 2;

				/* resend write page cmd again if failed */
				if (retry_times > 0) {
					rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, page_idx_cmd);
					rt9426a_reg_write_word(chip->i2c, RT9426A_REG_BDCNTL, page_idx_cmd);
					mdelay(1);
				}

				if ((page_checksum != g_PAGE_CHKSUM[array_idx]) || (total_checksum_retry>3)) {
					/* Update Setting to Extend Page */
					for (i = 0; i < 8; i++){
						extend_reg_cmd_addr = 0x40 + (2 * i);
						extend_reg_data = chip->pdata->extreg_table[array_idx].data[2 * i] +
								  (chip->pdata->extreg_table[array_idx].data[2 * i + 1] << 8);

						rt9426a_reg_write_word(chip->i2c,extend_reg_cmd_addr, extend_reg_data);
					}
					rt9426a_reg_write_word(chip->i2c,RT9426A_REG_DUMMY, 0x0000);

					retry_times++;
					if (retry_times < 5) {
						dev_info(chip->dev, "Page Checksum:%d\n",array_idx);
						goto PAGE_CHECKSUM2;
					}
				}
			} else {
				/* prepare retry for i2c fail */
				i2c_retry_times++;
				if(i2c_retry_times<3)
				    goto PAGE_CHECKSUM1;
			}
		}
	}
	goto TOTAL_CHECKSUM_CHECK;

BYPASS_EXT_REG:
	if ((chip->ocv_checksum_ic != chip->ocv_checksum_dtsi)&&
		(wt_ocv_result == RT9426A_WRITE_OCV_PASS)) {
		retry_times = 30;
		for (i = 0 ; i < retry_times ; i++) {
			regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG2);
			if ((reg_flag2&RT9426A_RDY_MASK)&&(reg_flag2!=0xFFFF)) {
				mdelay(200);
				if(rt9426a_get_current(chip)>0){
					volt_now = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_VBAT);
					fd_vth_now = chip->pdata->extreg_table[RT9426A_FD_TBL_IDX].data[RT9426A_FD_DATA_IDX];
					fd_threshold = RT9426A_FD_BASE + 5*(fd_vth_now);
					dev_info(chip->dev, "RT9426A VBAT = %d\n",volt_now);
					dev_info(chip->dev, "RT9426A FD_VTH = %d\n",fd_vth_now);
					dev_info(chip->dev, "RT9426A FD Threshold = %d\n",fd_threshold);

					if(volt_now > fd_threshold){
						/* disable battery charging path before QS command */
						rt9426a_request_charging_inhibit(true);
						dev_info(chip->dev, "Enable Charging Inhibit and delay 1250ms\n");
						mdelay(1250);
					}
				}
				break;
			}
			mdelay(10);
		}
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, 0x4000);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(5);
		dev_info(chip->dev, "OCV checksum are different, QS is done.\n");

		/* Power path recover check */
		if(volt_now > fd_threshold){
			/* enable battery charging path after QS command */
			rt9426a_request_charging_inhibit(false);
			dev_info(chip->dev, "Disable Charging Inhibit\n");
		}
	}
	else
		dev_info(chip->dev, "OCV checksum are the same, bypass QS.\n");

	/* clear RI, set 0 to RI bits */
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
	regval = (regval & ~RT9426A_RI_MASK);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_FLAG3, regval);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
	if (((regval & RT9426A_RI_MASK) >> 8) == 0)
		dev_info(chip->dev, "RT9426A RI=0\n");
	else
		dev_info(chip->dev, "RT9426A RI=1\n");

	/* Seal RT9426A */
	retry_times = 0;
SET_SEAL_CMD:
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_CNTL, RT9426A_SEAL_CMD);
	rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
	mdelay(1);
	regval = rt9426a_reg_read_word(chip->i2c, RT9426A_REG_FLAG3);
	if (regval & RT9426A_UNSEAL_STATUS) {
		retry_times++;
		if (retry_times < 3) {
			dev_dbg(chip->dev,"RT9426A Seal Retry-%d\n",retry_times);
			goto SET_SEAL_CMD;
		} else {
			dev_dbg(chip->dev,"RT9426A Seal Fail\n");
			ret = RT9426A_INIT_SEAL_ERR;
		}
	} else {
		dev_dbg(chip->dev,"RT9426A Seal Pass\n");
		if(ret != RT9426A_INIT_CKSUM_ERR)
			ret = RT9426A_INIT_PASS;
	}
END_INIT:
	/* check cyccnt & bccomp */
	/* rt9426a_check_cycle_cnt_for_fg_ini(chip); */

	/* get initial soc for driver for smooth soc */
	chip->capacity = rt9426a_fg_get_soc(chip,0);

	if (ret == RT9426A_INIT_PASS) {
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_RSVD, chip->pdata->para_version);
		rt9426a_reg_write_word(chip->i2c, RT9426A_REG_DUMMY, 0x0000);
		mdelay(1);
		dev_info(chip->dev, "RT9426A Initial Successful\n");
		chip->online = 1;
	} else
		dev_info(chip->dev, "RT9426A Initial Fail\n");

	return ret;
}
static const char *rt_get_battery_serialnumber(struct device *dev)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	const char *battsn_buf;
	int retval;

	battsn_buf = NULL;

	if (np)
		retval = of_property_read_string(np, "mmi,battid",
						 &battsn_buf);
	else
		return NULL;

	if ((retval == -EINVAL) || !battsn_buf) {
		dev_info(dev, "Battsn unused\n");
		of_node_put(np);
		return NULL;

	} else
		dev_info(dev, "Battsn = %s\n", battsn_buf);

	of_node_put(np);

	return battsn_buf;
}

static struct device_node *rt_get_profile_by_serialnumber(struct device *dev)
{
	struct device_node *node, *df_node, *sn_node;
	struct device_node *np = dev->of_node;
	const char *sn_buf, *df_sn, *dev_sn;
	int rc;

	if (!np)
		return NULL;

	dev_sn = NULL;
	df_sn = NULL;
	sn_buf = NULL;
	df_node = NULL;
	sn_node = NULL;

	dev_sn = rt_get_battery_serialnumber(dev);

	rc = of_property_read_string(np, "df-serialnum",
				     &df_sn);
	if (rc)
		dev_info(dev,"No Default Serial Number defined\n");
	else if (df_sn)
		dev_info(dev,"Default Serial Number %s\n", df_sn);

	for_each_child_of_node(np, node) {
		rc = of_property_read_string(node, "serialnum",
					     &sn_buf);
		if (!rc && sn_buf) {
			if (dev_sn)
				if (strnstr(dev_sn, sn_buf, 32))
					sn_node = node;
			if (df_sn)
				if (strnstr(df_sn, sn_buf, 32))
					df_node = node;
		}
	}

	if (sn_node) {
		node = sn_node;
		df_node = NULL;
		dev_info(dev,"Battery Match Found using %s\n", sn_node->name);
	} else if (df_node) {
		node = df_node;
		sn_node = NULL;
		dev_info(dev,"Battery Match Found using default %s\n",
				df_node->name);
	} else {
		dev_info(dev,"No Battery Match Found!\n");
		return NULL;
	}

	return node;
}

struct dt_offset_params {
	int data[3];
};

struct dt_extreg_params {
	int edata[3];
};

static int rt_parse_dt(struct device *dev, struct rt9426a_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *batt_profile_node = NULL;
	int sizes[3] = {0}, j, ret, i;
	struct dt_offset_params *offset_params;
	char prop_name[64] = {0};

	dev_info(dev, "%s\n", __func__);

	ret = of_property_read_u32_array(np, "rt,dtsi_version", pdata->dtsi_version, 2);
	if (ret < 0)
		pdata->dtsi_version[0] = pdata->dtsi_version[1] = 0;

	ret = of_property_read_u32(np, "rt,para_version", &pdata->para_version);
	if (ret < 0)
		pdata->para_version = 0;

	ret = of_property_read_string(np, "rt,bat_name",
				      (const char **)&pdata->bat_name);
	if (ret < 0)
		pdata->bat_name = NULL;

	ret = of_property_read_u32_array(np, "rt,offset_interpolation_order",
					 pdata->offset_interpolation_order, 2);
	if (ret < 0)
		pdata->offset_interpolation_order[0] = pdata->offset_interpolation_order[1] = 2;

	sizes[0] = sizes[1] = 0;
	ret = of_property_read_u32_array(np, "rt,soc_offset_size", sizes, 2);
	if (ret < 0)
		dev_notice(dev, "Can't get prop soc_offset_size(%d)\n", ret);

	new_vgcomp_soc_offset_datas(dev, SOC_OFFSET, pdata, sizes[0], sizes[1], 0);
	if (pdata->soc_offset.soc_offset_data) {
		offset_params = devm_kzalloc(dev,
					     sizes[0] * sizes[1] * sizeof(struct dt_offset_params),
					     GFP_KERNEL);

		of_property_read_u32_array(np, "rt,soc_offset_data",
					   (u32 *)offset_params, sizes[0] * sizes[1] * 3);
		for (j = 0; j < sizes[0] * sizes[1]; j++) {
			pdata->soc_offset.soc_offset_data[j].x = offset_params[j].data[0];
			pdata->soc_offset.soc_offset_data[j].y = offset_params[j].data[1];
			pdata->soc_offset.soc_offset_data[j].offset = offset_params[j].data[2];
		}
		devm_kfree(dev, offset_params);
	}

	ret = of_property_read_u32(np, "rt,battery_type", &pdata->battery_type);
	if (ret < 0) {
		dev_info(dev, "uset default battery_type 4350mV, EDV=3200mV\n");
		pdata->battery_type = 4352;
	}

	ret = of_property_read_u32(np, "rt,volt_source", &pdata->volt_source);
	if (ret < 0)
		pdata->volt_source = RT9426A_REG_AV;

	if (pdata->volt_source == 0)
		pdata->volt_source = 0;
	else if (pdata->volt_source == 1)
		pdata->volt_source = RT9426A_REG_VBAT;
	else if (pdata->volt_source == 2)
		pdata->volt_source = RT9426A_REG_OCV;
	else if (pdata->volt_source == 3)
		pdata->volt_source = RT9426A_REG_AV;
	else {
		dev_notice(dev, "pdata->volt_source is out of range, use 3\n");
		pdata->volt_source = RT9426A_REG_AV;
	}

	ret = of_property_read_u32(np, "rt,temp_source", &pdata->temp_source);
	if (ret < 0)
		pdata->temp_source = 0;

	if (pdata->temp_source == 0)
		pdata->temp_source = 0;
	else if (pdata->temp_source == 1)
		pdata->temp_source = RT9426A_REG_TEMP;
	else if (pdata->temp_source == 2)
		pdata->temp_source = RT9426A_REG_INTT;
	else if (pdata->temp_source == 3)
		pdata->temp_source = RT9426A_REG_AT;
	else {
		dev_notice(dev, "pdata->temp_source is out of range, use 0\n");
		pdata->temp_source = 0;
	}
	ret = of_property_read_u32(np, "rt,curr_source", &pdata->curr_source);
	if (ret < 0)
		pdata->curr_source = 0;
	if (pdata->curr_source == 0)
		pdata->curr_source = 0;
	else if (pdata->curr_source == 1)
		pdata->curr_source = RT9426A_REG_CURR;
	else if (pdata->curr_source == 2)
		pdata->curr_source = RT9426A_REG_AI;
	else {
		dev_notice(dev, "pdata->curr_source is out of range, use 2\n");
		pdata->curr_source = RT9426A_REG_AI;
	}

	of_property_read_u32(np, "rt,rs_ic_setting",&pdata->rs_ic_setting);
	of_property_read_u32(np, "rt,rs_schematic",&pdata->rs_schematic);

	dev_info(dev, "rs_ic_setting = %d\n", pdata->rs_ic_setting);
	dev_info(dev, "rs_schematic = %d\n", pdata->rs_schematic);

	/* add for smooth_soc */
	of_property_read_u32(np, "rt,smooth_soc_en",&pdata->smooth_soc_en);
	dev_info(dev, "smooth_soc_en = %d\n", pdata->smooth_soc_en);

	/* add for aging cv */
	/* parse fcc array by 5 element */
	batt_profile_node = rt_get_profile_by_serialnumber(dev);
	if (!batt_profile_node)
		return  -1;
	of_property_read_u32(batt_profile_node, "rt,fcc_design",&pdata->fcc_design);
	dev_info(dev,"fcc_design = %d\n", pdata->fcc_design);

	ret = of_property_read_u32_array(batt_profile_node, "rt,fcc", pdata->fcc, 5);
	if (ret < 0) {
		dev_notice(dev, "no FCC property, use defaut 2000\n");
		for (i = 0; i < 5; i++)
			pdata->fcc[i] = 2000;
	}
	/* parse fc_vth array by 5 element */
	ret = of_property_read_u32_array(batt_profile_node, "rt,fg_fc_vth", pdata->fc_vth, 5);
	if (ret < 0) {
		dev_notice(dev, "no fc_vth property, use default 4200mV\n");
		for (i = 0; i < 5; i++)
			pdata->fc_vth[i] = 0x0078;
	}
	/*  Read Ext. Reg Table for RT9426A  */
	ret = of_property_read_u8_array(batt_profile_node, "rt,fg_extreg_table", (u8 *)pdata->extreg_table, 224);
	if (ret < 0) {
		dev_notice(dev, "no ocv table property\n");
		for (j = 0; j < 15; j++)
			for (i = 0; i < 16; i++)
				pdata->extreg_table[j].data[i] = 0;
	}
	/* parse ocv_table array by 80x5 element */
	for (i = 0; i < 5; i++) {
		snprintf(prop_name, 64, "rt,fg_ocv_table%d", i);
		ret = of_property_read_u32_array(batt_profile_node, prop_name,
						 (u32 *)pdata->ocv_table + i * 80, 80);
		if (ret < 0)
			memset32((u32 *)pdata->ocv_table + i * 80, 0, 80);
	}

	return 0;
}

static int rt9426a_i2c_chipid_check(struct rt9426a_chip *chip)
{
	unsigned int val = 0;
	int ret;

	ret = regmap_read(chip->regmap, RT9426A_REG_DEVICE_ID, &val);
	if (ret < 0)
		return ret;

	if (val != RT9426A_DEVICE_ID) {
		dev_err(chip->dev, "dev_id not match\n");
		return -ENODEV;
	}

	chip->ic_ver = val;
	return 0;
}

static bool rt9426a_is_writeable_reg(struct device *dev, unsigned int reg)
{
	return (reg % 2) ? false : true;
}

static bool rt9426a_is_readable_reg(struct device *dev, unsigned int reg)
{
	return (reg % 2) ? false : true;
}

static const struct regmap_config rt9426a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.writeable_reg = rt9426a_is_writeable_reg,
	.readable_reg = rt9426a_is_readable_reg,
	.max_register = RT9426A_REG_TOTAL_CHKSUM,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static void fg_update_work_func(struct work_struct *work)
{
	struct rt9426a_chip *chip = container_of(work, struct rt9426a_chip, update_work.work);

	pm_wakeup_hard_event(chip->dev);
	pm_stay_awake(chip->dev);
	dev_info(chip->dev, "%s++\n", __func__);
	rt9426a_update_info(chip);
	queue_delayed_work(system_power_efficient_wq, &chip->update_work, 20 * HZ);
	dev_info(chip->dev, "%s--\n", __func__);
	pm_relax(chip->dev);
}

static int rt9426a_i2c_probe(struct i2c_client *i2c)
{
	struct rt9426a_platform_data *pdata = i2c->dev.platform_data;
	struct rt9426a_chip *chip;
	struct power_supply_config psy_config = {};
	bool use_dt = i2c->dev.of_node;
	int ret = 0, i = 0;

	/* alloc memory */
	chip = devm_kzalloc(&i2c->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	if (use_dt) {
		pdata = devm_kzalloc(&i2c->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		memcpy(pdata, &def_platform_data, sizeof(*pdata));
		ret = rt_parse_dt(&i2c->dev, pdata);
		if (ret < 0) {
			dev_notice(&i2c->dev, " rt_parse_dt failed\n");
			return -EINVAL;
		}
		chip->pdata = i2c->dev.platform_data = pdata;
	} else {
		if (!pdata) {
			dev_notice(&i2c->dev, " no platdata specified\n");
			return -EINVAL;
		}
	}

	chip->i2c = i2c;
	chip->dev = &i2c->dev;
	chip->alert_irq = -1;
	chip->btemp = 250;
	chip->bvolt = 3800;
	chip->bcurr = 1000;
	chip->design_capacity = 2000;
	chip->full_capacity = 2000;
	chip->soh = 100;
	chip->cyccnt = 0;
	chip->ocv_checksum_ic = 0;
	chip->ocv_index = 0;	/* init as 0 */
	/* add for aging cv */
	chip->ocv_checksum_dtsi = *((u32 *)chip->pdata->ocv_table +
	                          (chip->ocv_index * 80) + RT9426A_IDX_OF_OCV_CKSUM);

	mutex_init(&chip->var_lock);
	mutex_init(&chip->update_lock);
	INIT_DELAYED_WORK(&chip->update_work, fg_update_work_func);
	i2c_set_clientdata(i2c, chip);

	/* rt regmap init */
	chip->regmap = devm_regmap_init_i2c(i2c, &rt9426a_regmap_config);
	if (IS_ERR(chip->regmap)) {
		dev_notice(chip->dev, "regmap init fail\n");
		ret = PTR_ERR(chip->regmap);
		goto destroy_mutex;
	}

	/* check chip id first */
	ret = rt9426a_i2c_chipid_check(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "chip id check fail\n");
		goto destroy_mutex;
	}

	/* apply platform data */
	ret = rt9426a_apply_pdata(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "apply pdata fail\n");
		goto destroy_mutex;
	}

	/* fg psy register */
	psy_config.of_node = i2c->dev.of_node;
	psy_config.drv_data = chip;
	if (pdata->bat_name)
		fg_psy_desc.name = pdata->bat_name;
	chip->fg_psy = devm_power_supply_register(chip->dev, &fg_psy_desc, &psy_config);
	if (IS_ERR(chip->fg_psy)) {
		dev_notice(chip->dev, "register batt psy fail\n");
		ret = PTR_ERR(chip->fg_psy);
		goto destroy_mutex;
	}

	ret = rt_fg_create_attrs(&chip->fg_psy->dev);
	if (ret) {
		dev_notice(chip->dev, "create attrs fail\n");
		goto destroy_mutex;
	}

	/* mask irq before irq register */
	ret = rt9426a_irq_enable(chip, false);
	if (ret < 0) {
		dev_notice(chip->dev, "scirq mask fail\n");
		goto fail_irq_enable;
	}

	ret = rt9426a_irq_init(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "irq init fail\n");
		goto fail_irq_enable;
	}

	ret = rt9426a_irq_enable(chip, true);
	if (ret < 0) {
		dev_notice(chip->dev, "scirq mask fail\n");
		goto fail_irq_enable;
	}

	dev_info(chip->dev, "chip ver = 0x%04x\n", chip->ic_ver);
	queue_delayed_work(system_power_efficient_wq, &chip->update_work, 5 * HZ);

	return 0;

fail_irq_enable:
	for (i = 0; i < ARRAY_SIZE(rt_fuelgauge_attrs); i++)
		device_remove_file(&chip->fg_psy->dev, &rt_fuelgauge_attrs[i]);
destroy_mutex:
	mutex_destroy(&chip->update_lock);
	mutex_destroy(&chip->var_lock);

	return ret;
}

static int rt9426a_i2c_remove(struct i2c_client *i2c)
{
	struct rt9426a_chip *chip = i2c_get_clientdata(i2c);
	int i = 0;

	dev_info(chip->dev, "%s\n", __func__);
	rt9426a_irq_enable(chip, false);
	rt9426a_irq_deinit(chip);
	for (i = 0; i < ARRAY_SIZE(rt_fuelgauge_attrs); i++)
		device_remove_file(&chip->fg_psy->dev, &rt_fuelgauge_attrs[i]);
	mutex_destroy(&chip->update_lock);
	mutex_destroy(&chip->var_lock);

	return 0;
}

static int rt9426a_i2c_suspend(struct device *dev)
{
	struct rt9426a_chip *chip = dev_get_drvdata(dev);

	dev_dbg(chip->dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->alert_irq);
	disable_irq(chip->alert_irq);

	return 0;
}

static int rt9426a_i2c_resume(struct device *dev)
{
	struct rt9426a_chip *chip = dev_get_drvdata(dev);

	dev_dbg(chip->dev, "%s\n", __func__);
	enable_irq(chip->alert_irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->alert_irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rt9426a_pm_ops, rt9426a_i2c_suspend, rt9426a_i2c_resume);

static const struct of_device_id rt_match_table[] = {
	{ .compatible = "richtek,rt9426a", },
	{},
};
MODULE_DEVICE_TABLE(of, rt_match_table);

static struct i2c_driver rt9426a_i2c_driver = {
	.driver = {
		.name = "rt9426a",
		.of_match_table = of_match_ptr(rt_match_table),
		.pm = &rt9426a_pm_ops,
	},
	.probe_new = rt9426a_i2c_probe,
	.remove = rt9426a_i2c_remove,
};
module_i2c_driver(rt9426a_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_DESCRIPTION("RT9426A Fuelgauge Driver");
MODULE_VERSION("1.0.1_G");
