#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/sizes.h>
#include <linux/regulator/consumer.h>

#define CWFG_ENABLE_LOG 1 /* CHANGE Customer need to change this for enable/disable log */

#define CW_PROPERTIES cw_bat->bms_name

#define REG_CHIP_ID             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_GPIO_CONFIG         0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_CURRENT_H           0x0E
#define REG_CURRENT_L           0x0F
#define REG_T_HOST_H            0xA0
#define REG_T_HOST_L            0xA1
#define REG_USER_CONF           0xA2
#define REG_CYCLE_H             0xA4
#define REG_CYCLE_L             0xA5
#define REG_SOH                 0xA6
#define REG_IC_STATE            0xA7
#define REG_STB_CUR_H           0xA8
#define REG_STB_CUR_L           0xA9
#define REG_FW_VERSION          0xAB
#define REG_BAT_PROFILE         0x10

#define CONFIG_MODE_RESTART     0x30
#define CONFIG_MODE_ACTIVE      0x00
#define CONFIG_MODE_SLEEP       0xF0
#define CONFIG_UPDATE_FLG       0x80
#define IC_VCHIP_ID             0xA0
#define IC_READY_MARK           0x0C

#define GPIO_ENABLE_MIN_TEMP    0
#define GPIO_ENABLE_MAX_TEMP    0
#define GPIO_ENABLE_SOC_CHANGE  0
#define GPIO_SOC_IRQ_VALUE      0x0    /* 0x7F */
#define DEFINED_MAX_TEMP        45
#define DEFINED_MIN_TEMP        0

#define CWFG_NAME               "cw2217"
#define SIZE_OF_PROFILE         80
#define USER_RSENSE             (10*1000)  /* mhom rsense * 1000  for convenience calculation */

#define queue_delayed_work_time  8000
#define queue_start_work_time    50

#define CW_SLEEP_20MS           20
#define CW_SLEEP_10MS           10
#define CW_UI_FULL              100
#define COMPLEMENT_CODE_U16     0x8000
#define CW_SLEEP_100MS          100
#define CW_SLEEP_200MS          200
#define CW_SLEEP_COUNTS         50
#define CW_TRUE                 1
#define CW_RETRY_COUNT          3
#define CW_VOL_UNIT             1000
#define CW_CUR_UNIT             1000


#define CW2217_NOT_ACTIVE          1
#define CW2217_PROFILE_NOT_READY   2
#define CW2217_PROFILE_NEED_UPDATE 3

#define CW_BPD_TEMP (-400)

#define cw_printk(fmt, arg...)                                                 \
	{                                                                          \
		if (CWFG_ENABLE_LOG)                                                   \
			printk("FG_CW2217 : %s-%d : " fmt, __FUNCTION__ ,__LINE__,##arg);  \
		else {}                                                                \
	}

#define cw_info(fmt, arg...)  \
	printk("FG_CW2217 : %s-%d : " fmt, __FUNCTION__ ,__LINE__,##arg)


static unsigned char config_profile_info[SIZE_OF_PROFILE] = {0};
/*
static unsigned char config_profile_info[SIZE_OF_PROFILE] = {
	0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xA6,0xB1,0xB2,0xCE,0xC1,0xC1,0x9E,0x5F,
	0x36,0xFF,0xFF,0xF9,0xBA,0x92,0x7F,0x6B,
	0x60,0x5D,0x58,0x94,0xE7,0xDE,0x9A,0xDC,
	0xC2,0xCA,0xD2,0xD7,0xD6,0xD3,0xD1,0xCC,
	0xCA,0xC9,0xCE,0xB8,0x9C,0x92,0x88,0x81,
	0x75,0x71,0x7F,0x91,0xAB,0x83,0x71,0x70,
	0x20,0x00,0x57,0x10,0x02,0xB0,0x5D,0x00,
	0x00,0x00,0x64,0x26,0xD3,0x51,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x94,
};
*/
struct cw_battery {
	struct i2c_client *client;

	struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;
	struct work_struct  hw_init_work;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct power_supply cw_bat;
#else
	struct power_supply *cw_bat;
	struct power_supply *batt_psy;
#endif
	int  chip_id;
	int  voltage;
	int  ic_soc_h;
	int  ic_soc_l;
	int  ui_soc;
	int  raw_soc;
	int  temp;
	long cw_current;
	int  cycle;
	int  soh;
	int  fw_version;
	int  fcc_design;
	int  fcc;
	int  ui_full;
	int  ntc_exist;
	int  batt_status;
	bool present;
	bool factory_mode;
	int  sense_r_mohm;
#if 0
	long stb_current;
#endif
	int ibat_polority;
	const char *bms_name;
	struct regulator *vdd_i2c_vreg;
};

/* CW2217 iic read function */
static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data( client, reg, 1, buf);
	if (ret < 0)
		printk("IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic write function */
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0] );
	if (ret < 0)
		printk("IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic read word function */
static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret;
	unsigned char reg_val[2] = { 0, 0 };
	unsigned int temp_val_buff;
	unsigned int temp_val_second;

	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, reg_val );
	if (ret < 0)
		printk("IIC error %d\n", ret);
	temp_val_buff = (reg_val[0] << 8) + reg_val[1];

	msleep(1);
	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, reg_val );
	if (ret < 0)
		printk("IIC error %d\n", ret);
	temp_val_second = (reg_val[0] << 8) + reg_val[1];

	if (temp_val_buff != temp_val_second) {
		msleep(1);
		ret = i2c_smbus_read_i2c_block_data( client, reg, 2, reg_val );
		if (ret < 0)
			printk("IIC error %d\n", ret);
		temp_val_buff = (reg_val[0] << 8) + reg_val[1];
	}

	buf[0] = reg_val[0];
	buf[1] = reg_val[1];

	return ret;
}

/* CW2217 iic write profile function */
static int cw_write_profile(struct i2c_client *client, unsigned char const buf[])
{
	int ret;
	int i;

	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_write(client, REG_BAT_PROFILE + i, &buf[i]);
		if (ret < 0) {
			printk("IIC error %d\n", ret);
			return ret;
		}
	}

	return ret;
}

/*
 * CW2217 Active function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_active(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	cw_printk("\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_ACTIVE;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return 0;
}

/*
 * CW2217 Sleep function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_sleep(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	cw_printk("\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_SLEEP;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return 0;
}

/*
 * The 0x00 register is an UNSIGNED 8bit read-only register. Its value is fixed to 0xA0 in shutdown
 * mode and active mode.
 */
static int cw_get_chip_id(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int chip_id;

	ret = cw_read(cw_bat->client, REG_CHIP_ID, &reg_val);
	if (ret < 0)
		return ret;

	chip_id = reg_val;  /* This value must be 0xA0! */
	cw_printk("chip_id = %d\n", chip_id);
	cw_bat->chip_id = chip_id;

	return 0;
}

/*
 * The VCELL register(0x02 0x03) is an UNSIGNED 14bit read-only register that updates the battery voltage continuously.
 * Battery voltage is measured between the VCELL pin and VSS pin, which is the ground reference. A 14bit
 * sigma-delta A/D converter is used and the voltage resolution is 312.5uV. (0.3125mV is *5/16)
 */
static int cw_get_voltage(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	unsigned int voltage;

	ret = cw_read_word(cw_bat->client, REG_VCELL_H, reg_val);
	if (ret < 0)
		return ret;
	voltage = (reg_val[0] << 8) + reg_val[1];
	voltage = voltage  * 5 / 16;
	cw_bat->voltage = voltage;

	return 0;
}

/*
 * The SOC register(0x04 0x05) is an UNSIGNED 16bit read-only register that indicates the SOC of the battery. The
 * SOC shows in % format, which means how much percent of the battery's total available capacity is
 * remaining in the battery now. The SOC can intrinsically adjust itself to cater to the change of battery status,
 * including load, temperature and aging etc.
 * The high byte(0x04) contains the SOC in 1% unit which can be directly used if this resolution is good
 * enough for the application. The low byte(0x05) provides more accurate fractional part of the SOC and its
 * LSB is (1/256) %.
 */
static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2] = { 0, 0 };
	int ui_100 = cw_bat->ui_full;
	int soc_h;
	int soc_l;
	int ui_soc;
	long batt_curr;
	int remainder;

	ret = cw_read_word(cw_bat->client, REG_SOC_INT, reg_val);
	if (ret < 0)
		return ret;
	soc_h = reg_val[0];
	soc_l = reg_val[1];
	cw_bat->raw_soc = soc_h;
	ui_soc = ((soc_h * 256 + soc_l) * 100)/ (ui_100 * 256);
	remainder = (((soc_h * 256 + soc_l) * 100 * 100) / (ui_100 * 256)) % 100;
	if (ui_soc >= 100){
		cw_printk("CW2015[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, ui_soc);
		ui_soc = 100;
	}
	cw_bat->ic_soc_h = soc_h;
	cw_bat->ic_soc_l = soc_l;
	cw_bat->ui_soc = ui_soc;

	return 0;
}

bool is_factory_mode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory_mode = false;
	const char *bootargs = NULL;
	char *bootmode = NULL;
	char *end = NULL;

	if (!np)
		return factory_mode;

	if (!of_property_read_string(np, "bootargs", &bootargs)) {
		bootmode = strstr(bootargs, "androidboot.mode=");
		if (bootmode) {
			end = strpbrk(bootmode, " ");
			bootmode = strpbrk(bootmode, "=");
		}
		if (bootmode &&
		    end > bootmode &&
		    strnstr(bootmode, "mot-factory", end - bootmode)) {
				factory_mode = true;
		}
	}
	of_node_put(np);

	return factory_mode;
}

/*
 * The TEMP register is an UNSIGNED 8bit read only register.
 * It reports the real-time battery temperature
 * measured at TS pin. The scope is from -40 to 87.5 degrees Celsius,
 * LSB is 0.5 degree Celsius. TEMP(C) = - 40 + Value(0x06 Reg) / 2
 */
static int cw_get_temp(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int temp;

	ret = cw_read(cw_bat->client, REG_TEMP, &reg_val);
	if (ret < 0)
		return ret;

	temp = (int)reg_val * 10 / 2 - 400;

	if(cw_bat->factory_mode && !cw_bat->ntc_exist && (-400 == temp))
		temp = 250;

	cw_bat->temp = temp;

	return 0;
}

/* get complement code function, unsigned short must be U16 */
static long get_complement_code(unsigned short raw_code)
{
	long complement_code;
	int dir;

	if (0 != (raw_code & COMPLEMENT_CODE_U16)){
		dir = -1;
		raw_code =  (~raw_code) + 1;
	} else {
		dir = 1;
	}
	complement_code = (long)raw_code * dir;

	return complement_code;
}

/*
 * CURRENT is a SIGNED 16bit register(0x0E 0x0F) that reports current A/D converter result of the voltage across the
 * current sense resistor, 10mohm typical. The result is stored as a two's complement value to show positive
 * and negative current. Voltages outside the minimum and maximum register values are reported as the
 * minimum or maximum value.
 * The register value should be divided by the sense resistance to convert to amperes. The value of the
 * sense resistor determines the resolution and the full-scale range of the current readings. The LSB of 0x0F
 * is (52.4/32768)uV.
 * The default value is 0x0000, stands for 0mA. 0x7FFF stands for the maximum charging current and 0x8001 stands for
 * the maximum discharging current.
 */
static int cw_get_current(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	long long cw_current; /* use long long type to guarantee 8 bytes space*/
	unsigned short current_reg;  /* unsigned short must u16 */

	ret = cw_read_word(cw_bat->client, REG_CURRENT_H, reg_val);
	if (ret < 0)
		return ret;

	current_reg = (reg_val[0] << 8) + reg_val[1];
	cw_current = get_complement_code(current_reg);
//	cw_current = cw_current  * 160 * 1000 / USER_RSENSE / 100;
	cw_current = cw_current  * 160 * 1000 / cw_bat->sense_r_mohm / 100;
	cw_bat->cw_current = cw_current;

	return 0;
}

/*
 * CYCLECNT is an UNSIGNED 16bit register(0xA4 0xA5) that counts cycle life of the battery. The LSB of 0xA5 stands
 * for 1/16 cycle. This register will be clear after enters shutdown mode
 */
static int cw_get_cycle_count(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2] = {0, 0};
	int cycle;

	ret = cw_read_word(cw_bat->client, REG_CYCLE_H, reg_val);
	if (ret < 0)
		return ret;

	cycle = (reg_val[0] << 8) + reg_val[1];
	cw_bat->cycle = cycle / 16;

	return 0;
}

/*
 * SOH (State of Health) is an UNSIGNED 8bit register(0xA6) that represents the level of battery aging by tracking
 * battery internal impedance increment. When the device enters active mode, this register refresh to 0x64
 * by default. Its range is 0x00 to 0x64, indicating 0 to 100%. This register will be clear after enters shutdown
 * mode.
 */
static int cw_get_soh(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int soh;

	ret = cw_read(cw_bat->client, REG_SOH, &reg_val);
	if (ret < 0)
		return ret;

	soh = reg_val;
	cw_bat->soh = soh;

	return 0;
}

#if 0
static int cw_get_stb_current(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	long long stb_current;
	unsigned short stb_current_reg;  /*unsigned short must u16*/

	ret = cw_read_word(cw_bat->client, REG_STB_CUR_H, reg_val);
	if (ret < 0)
		return ret;

	stb_current_reg = (reg_val[0] << 8) + reg_val[1];
	stb_current = get_complement_code(STB_current_reg);
	stb_current = stb_current  * 160 * 1000 / 16 / USER_RSENSE / 100;
	cw_bat->stb_current = stb_current;

	return 0;
}
#endif

/*
 * FW_VERSION register reports the firmware (FW) running in the chip. It is fixed to 0x00 when the chip is
 * in shutdown mode. When in active mode, Bit [7:6] are fixed to '01', which stand for the CW2217B and Bit
 * [5:0] stand for the FW version running in the chip. Note that the FW version is subject to update and contact
 * sales office for confirmation when necessary.
*/
static int cw_get_fw_version(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int fw_version;

	ret = cw_read(cw_bat->client, REG_FW_VERSION, &reg_val);
	if (ret < 0)
		return ret;

	fw_version = reg_val;
	cw_bat->fw_version = fw_version;

	return 0;
}

static int cw_update_data(struct cw_battery *cw_bat)
{
	int ret = 0;

	ret += cw_get_voltage(cw_bat);
	ret += cw_get_capacity(cw_bat);
	ret += cw_get_temp(cw_bat);
	ret += cw_get_current(cw_bat);
	ret += cw_get_cycle_count(cw_bat);
	ret += cw_get_soh(cw_bat);
	cw_printk("vol = %d  current = %ld cap = %d temp = %d raw_soc = %d age=%d\n",
		cw_bat->voltage, cw_bat->cw_current, cw_bat->ui_soc, cw_bat->temp, cw_bat->raw_soc, cw_bat->soh);

	return ret;
}

static int cw_init_data(struct cw_battery *cw_bat)
{
	int ret = 0;

	ret += cw_get_chip_id(cw_bat);
	ret += cw_get_voltage(cw_bat);
	ret += cw_get_capacity(cw_bat);
	ret += cw_get_temp(cw_bat);
	ret += cw_get_current(cw_bat);
	ret += cw_get_cycle_count(cw_bat);
	ret += cw_get_soh(cw_bat);
	ret += cw_get_fw_version(cw_bat);
	cw_printk("chip_id = %d vol = %d  cur = %ld cap = %d raw_soc = %d temp = %d  fw_version = %d\n",
		cw_bat->chip_id, cw_bat->voltage, cw_bat->cw_current, cw_bat->ui_soc, cw_bat->raw_soc, cw_bat->temp, cw_bat->fw_version);

	return ret;
}

/*CW2217 update profile function, Often called during initialization*/
static int cw_config_start_ic(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int count = 0;

	ret = cw2217_sleep(cw_bat);
	if (ret < 0)
		return ret;

	/* update new battery info */
	ret = cw_write_profile(cw_bat->client, config_profile_info);
	if (ret < 0)
		return ret;

	/* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
	reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;
	ret = cw_write(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if (ret < 0)
		return ret;

	/*close all interruptes*/
	reg_val = 0;
	ret = cw_write(cw_bat->client, REG_GPIO_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	ret = cw2217_active(cw_bat);
	if (ret < 0)
		return ret;

	while (CW_TRUE) {
		msleep(CW_SLEEP_100MS);
		cw_read(cw_bat->client, REG_IC_STATE, &reg_val);
		if (IC_READY_MARK == (reg_val & IC_READY_MARK))
			break;
		count++;
		if (count >= CW_SLEEP_COUNTS) {
			cw2217_sleep(cw_bat);
			return -1;
		}
	}

	return 0;
}

/*
 * Get the cw2217 running state
 * Determine whether the profile needs to be updated
*/
static int cw2217_get_state(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int i;
	int reg_profile;

	ret = cw_read(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	if (reg_val != CONFIG_MODE_ACTIVE)
		return CW2217_NOT_ACTIVE;

	ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if (ret < 0)
		return ret;
	if (0x00 == (reg_val & CONFIG_UPDATE_FLG))
		return CW2217_PROFILE_NOT_READY;

	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_read(cw_bat->client, (REG_BAT_PROFILE + i), &reg_val);
		if (ret < 0)
			return ret;
		reg_profile = REG_BAT_PROFILE + i;
		cw_printk("0x%2x = 0x%2x\n", reg_profile, reg_val);
		if (config_profile_info[i] != reg_val)
			break;
	}
	if ( i != SIZE_OF_PROFILE)
		return CW2217_PROFILE_NEED_UPDATE;

	return 0;
}

/*CW2217 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
	int ret;

	cw_printk("\n");
	ret = cw2217_get_state(cw_bat);
	if (ret < 0) {
		printk("iic read write error");
		return ret;
	}

	if (ret != 0) {
		ret = cw_config_start_ic(cw_bat);
		if (ret < 0)
			return ret;
	}
	cw_printk("cw2217 init success!\n");

	return 0;
}

static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	int ret;

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	/* get battery power supply */
	if (!cw_bat->batt_psy) {
		cw_bat->batt_psy = power_supply_get_by_name("battery");
		if (!cw_bat->batt_psy)
			cw_printk("%s: get batt_psy fail\n", __func__);
	}

	ret = cw_update_data(cw_bat);
	if (ret < 0)
		printk(KERN_ERR "iic read error when update data");

	if (cw_bat->batt_psy) {
		power_supply_changed(cw_bat->batt_psy);
	}

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
}

static void cw_hw_init_work(struct work_struct *work)
{
	struct cw_battery *cw_bat;
	int ret;
	int loop = 0;

	cw_bat = container_of(work, struct cw_battery, hw_init_work);

	ret = cw_init(cw_bat);
	while ((loop++ < CW_RETRY_COUNT) && (ret != 0)) {
		msleep(CW_SLEEP_200MS);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		printk("%s : cw2217 init fail!\n", __func__);
		return ;
	}

	ret = cw_init_data(cw_bat);
	if (ret) {
		printk("%s : cw2217 init data fail!\n", __func__);
		return ;
	}

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(queue_start_work_time));

}

static const char *cw_get_battery_serialnumber(void)
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
		cw_info(" Battsn unused\n");
		of_node_put(np);
		return NULL;

	} else
		cw_info("Battsn = %s\n", battsn_buf);

	of_node_put(np);

	return battsn_buf;
}

static struct device_node *cw_get_profile_by_serialnumber(
		const struct device_node *np)
{
	struct device_node *node, *df_node, *sn_node;
	const char *sn_buf, *df_sn, *dev_sn;
	int rc;

	if (!np)
		return NULL;

	dev_sn = NULL;
	df_sn = NULL;
	sn_buf = NULL;
	df_node = NULL;
	sn_node = NULL;

	dev_sn = cw_get_battery_serialnumber();

	rc = of_property_read_string(np, "df-serialnum",
				     &df_sn);
	if (rc)
		cw_info("No Default Serial Number defined\n");
	else if (df_sn)
		cw_info("Default Serial Number %s\n", df_sn);

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
		cw_info("Battery Match Found using %s\n", sn_node->name);
	} else if (df_node) {
		node = df_node;
		sn_node = NULL;
		cw_info("Battery Match Found using default %s\n",
				df_node->name);
	} else {
		cw_info("No Battery Match Found!\n");
		return NULL;
	}

	return node;
}

static int cw_parse_dts(struct cw_battery *cw_bat)
{
	struct device_node *np = cw_bat->client->dev.of_node;
	struct device_node *batt_profile_node = NULL;
	int rc;

	rc = of_property_read_u32(np, "sense_r_mohm", &cw_bat->sense_r_mohm);
	if(rc < 0)
		cw_bat->sense_r_mohm = USER_RSENSE;
	else
		cw_bat->sense_r_mohm *= 1000;

	batt_profile_node = cw_get_profile_by_serialnumber(np);
	if (!batt_profile_node)
		return -1;

	rc = of_property_read_u8_array(batt_profile_node, "config_profile_info", config_profile_info, SIZE_OF_PROFILE);
	if (rc < 0)
		cw_info("error,get profile_info fail from dts,exit \n");

	rc = of_property_read_u32(batt_profile_node, "fcc_design", &cw_bat->fcc_design);
	if (rc < 0)
		cw_info("error,get fcc_design,exit \n");

	rc = of_property_read_u32(batt_profile_node, "ui_full", &cw_bat->ui_full);
	if (rc < 0) {
		cw_bat->ui_full = CW_UI_FULL;
		cw_info("dts get ui_full fail. use default ui_full=%d \n", CW_UI_FULL);
		rc = 0;
	}

	rc = of_property_read_u32(np, "factory_mode_ntc_exist", &cw_bat->ntc_exist);
	if (rc < 0) {
		cw_bat->ntc_exist = true;
		cw_info("dts get ntc_exist fail. use default ntc_exist=%d \n", cw_bat->ntc_exist);
		rc = 0;
	}

	rc = of_property_read_string(np, "fg-psy-name", &cw_bat->bms_name);
	if (rc) {
		cw_bat->bms_name = "bms";
		rc = 0;
	}

	if (of_property_read_bool(np, "ibat-invert-polority"))
		cw_bat->ibat_polority = -1;
	else
		cw_bat->ibat_polority = 1;

	return rc;
}

#ifdef CW_PROPERTIES
static int cw_battery_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	int ret = 0;
	struct cw_battery *cw_bat;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	cw_bat = container_of(psy, struct cw_battery, cw_bat);
#else
	cw_bat = power_supply_get_drvdata(psy);
#endif

	switch(psp) {
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void cw_get_batt_status(struct cw_battery *cw_bat)
{
	long batt_curr = 0;
	batt_curr = cw_bat->cw_current * CW_CUR_UNIT * (-1);
	batt_curr *= cw_bat->ibat_polority;

	if (cw_bat->voltage <= 0 || cw_bat->temp <= CW_BPD_TEMP)
		cw_bat->present = 0;
	else
		cw_bat->present = 1;

	if (!cw_bat->present) {
		cw_bat->batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
	} else if (cw_bat->ui_soc == CW_UI_FULL) {
		cw_bat->batt_status = POWER_SUPPLY_STATUS_FULL;
	} else if (batt_curr > 0)
		cw_bat->batt_status = POWER_SUPPLY_STATUS_CHARGING;
	else if (batt_curr < 0)
		cw_bat->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		cw_bat->batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static unsigned int cw_get_charge_counter(struct cw_battery *cw_bat)
{
	int charge_counter;
	int full_capacity;

	full_capacity = (cw_bat->fcc_design * cw_bat->soh * 1000) / 100;
	charge_counter = div_s64(full_capacity * cw_bat->ui_soc, 100);

	return charge_counter;
}

static int cw_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct cw_battery *cw_bat;
	cw_bat = container_of(psy, struct cw_battery, cw_bat);
#else
	struct cw_battery *cw_bat = power_supply_get_drvdata(psy);
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		cw_get_batt_status(cw_bat);
		val->intval = cw_bat->batt_status;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = cw_bat->cycle;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = cw_bat->ui_soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval= POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (cw_bat->voltage <= 0)
			val->intval = 0;
		else if (cw_bat->temp <= CW_BPD_TEMP)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = cw_bat->voltage * CW_VOL_UNIT;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		/* voltage_ocv invalid, use voltage_now instead*/
		val->intval = cw_bat->voltage * CW_VOL_UNIT;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		cw_get_current(cw_bat);
		val->intval = cw_bat->cw_current * CW_CUR_UNIT * (-1);
		val->intval *= cw_bat->ibat_polority;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = cw_bat->fcc_design * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = (cw_bat->fcc_design * cw_bat->soh * 1000)/100;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = cw_bat->temp;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = cw_get_charge_counter(cw_bat);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property cw_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};
#endif

static int cw2217_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct cw_battery *cw_bat;
#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};
#endif
#endif

	cw_printk("\n");

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		printk("%s : cw_bat create fail!\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cw_bat);
	cw_bat->client = client;

	ret = cw_parse_dts(cw_bat);
	if (ret) {
		printk("%s : cw2217 prase dts  fail!\n", __func__);
		return ret;
	}

	cw_bat->vdd_i2c_vreg = devm_regulator_get_optional(
					&cw_bat->client->dev,
					"vdd-i2c");
	if (IS_ERR_OR_NULL(cw_bat->vdd_i2c_vreg)) {
		printk("%s: Could not get vdd-i2c power regulator\n", __func__);
		cw_bat->vdd_i2c_vreg = NULL;
	} else {
		ret = regulator_enable(cw_bat->vdd_i2c_vreg);
		printk("%s: Enable vdd-i2c, ret=%d\n", __func__, ret);
		ret = 0;
	}

	ret = cw_get_chip_id(cw_bat);
	if (ret < 0) {
		printk("iic read write error");
		goto error;
	}
	if (cw_bat->chip_id != IC_VCHIP_ID){
		printk("not cw2217B\n");
		goto error;
	}

#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	cw_bat->cw_bat.name = CW_PROPERTIES;
	cw_bat->cw_bat.type = POWER_SUPPLY_TYPE_MAINS;
	cw_bat->cw_bat.properties = cw_battery_properties;
	cw_bat->cw_bat.num_properties = ARRAY_SIZE(cw_battery_properties);
	cw_bat->cw_bat.get_property = cw_battery_get_property;
	cw_bat->cw_bat.set_property = cw_battery_set_property;
	ret = power_supply_register(&client->dev, &cw_bat->cw_bat);
	if (ret < 0) {
		power_supply_unregister(&cw_bat->cw_bat);
		goto error;
	}
#else
	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc) {
		ret = -ENOMEM;
		goto error;
	}
	psy_cfg.drv_data = cw_bat;
	psy_desc->name = CW_PROPERTIES;
	psy_desc->type = POWER_SUPPLY_TYPE_MAINS;
	psy_desc->properties = cw_battery_properties;
	psy_desc->num_properties = ARRAY_SIZE(cw_battery_properties);
	psy_desc->get_property = cw_battery_get_property;
	psy_desc->set_property = cw_battery_set_property;
	cw_bat->cw_bat = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if (IS_ERR(cw_bat->cw_bat)) {
		ret = PTR_ERR(cw_bat->cw_bat);
		printk(KERN_ERR"failed to register battery: %d\n", ret);
		goto error;
	}
#endif
#endif

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);

	INIT_WORK(&cw_bat->hw_init_work, cw_hw_init_work);
	schedule_work(&cw_bat->hw_init_work);

	if(is_factory_mode())
		cw_bat->factory_mode = true;

	cw_printk("cw2217 driver probe success!\n");

	return 0;

error:
	if (cw_bat->vdd_i2c_vreg) {
		if (regulator_is_enabled(cw_bat->vdd_i2c_vreg))
			regulator_disable(cw_bat->vdd_i2c_vreg);
		devm_regulator_put(cw_bat->vdd_i2c_vreg);
	}
	return ret;
}

static int cw2217_remove(struct i2c_client *client)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	cw_printk("\n");
	if (cw_bat->vdd_i2c_vreg) {
		if (regulator_is_enabled(cw_bat->vdd_i2c_vreg))
			regulator_disable(cw_bat->vdd_i2c_vreg);
		devm_regulator_put(cw_bat->vdd_i2c_vreg);
	}
	return 0;
}

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	cancel_delayed_work(&cw_bat->battery_delay_work);
	return 0;
}

static int cw_bat_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(20));
	return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend  = cw_bat_suspend,
	.resume   = cw_bat_resume,
};
#endif

static const struct i2c_device_id cw2217_id_table[] = {
	{ CWFG_NAME, 0 },
	{ }
};

static struct of_device_id cw2217_match_table[] = {
	{ .compatible = "cellwise,cw2217", },
	{ },
};

static struct i2c_driver cw2217_driver = {
	.driver   = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
		.pm = &cw_bat_pm_ops,
#endif
		.owner = THIS_MODULE,
		.of_match_table = cw2217_match_table,
	},
	.probe = cw2217_probe,
	.remove = cw2217_remove,
	.id_table = cw2217_id_table,
};

/*
	//Add to dsti file
	cw2217@64 {
		compatible = "cellwise,cw2217";
		reg = <0x64>;
	};
*/

static int __init cw2217_init(void)
{
	cw_printk("\n");
	i2c_add_driver(&cw2217_driver);
	return 0;
}

static void __exit cw2217_exit(void)
{
	i2c_del_driver(&cw2217_driver);
}

module_init(cw2217_init);
module_exit(cw2217_exit);

MODULE_AUTHOR("Cellwise FAE");
MODULE_DESCRIPTION("CW2217 FGADC Device Driver V1.2");
MODULE_LICENSE("GPL");
