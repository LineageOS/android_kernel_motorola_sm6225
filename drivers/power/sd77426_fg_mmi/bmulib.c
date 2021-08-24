/*****************************************************************************
* Copyright(c) BMT, 2021. All rights reserved.
*
* BMT [oz8806] Source Code Reference Design
* File:[bmulib.c]
*
* This Source Code Reference Design for BMT [oz8806] access
* ("Reference Design") is solely for the use of PRODUCT INTEGRATION REFERENCE ONLY,
* and contains confidential and privileged information of BMT International
* Limited. BMT shall have no liability to any PARTY FOR THE RELIABILITY,
* SERVICEABILITY FOR THE RESULT OF PRODUCT INTEGRATION, or results from: (i) any
* modification or attempted modification of the Reference Design by any party, or
* (ii) the combination, operation or use of the Reference Design with non-BMT
* Reference Design. Use of the Reference Design is at user's discretion to qualify
* the final work result.
*****************************************************************************/

/****************************************************************************************

****************************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/i2c.h>

#include "parameter.h"
#include "oz8806_regdef.h"
#include "table.h"

#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/cpu.h>
#include <linux/sched.h>

/*****************************************************************************
* Define section
* add all #define here
*****************************************************************************/

//#define FCC_UPDATA_CHARGE

#define VERSION		"2021.08.12/7.00.06 disable otp and expand rsense"
#define charge_step		parameter->charge_pursue_step
#define discharge_step		parameter->discharge_pursue_step
#define discharge_th		parameter->discharge_pursue_th
#define config_data		parameter->config

#define	RETRY_CNT	8
#define	FORCE_OTP_MAPPING	0

#define SHUTDOWN_HI          50
#define SHUTDOWN_TH1         100
#define SHUTDOWN_TH2         300

#define CHARGE_END_CURRENT2  (config_data->charge_end_current +2)

#define FCC_UPPER_LIMIT		 100
//#define FCC_LOWER_LIMIT		 80
#define FCC_LOWER_LIMIT		 70   //for lianxiang

#define charge_full_voltage  (config_data->charge_cv_voltage - 26)
//#define full_charge_data   (gas_gauge->fcc_data + config_data->design_capacity / 100 - 1)
#define full_charge_data   (gas_gauge->fcc_data + gas_gauge->fcc_data / 100 - 1)
//#define full_charge_data   gas_gauge->fcc_data + 2

#define	MAX_EOD_TIME		(60 * 9)	//H7 add timer full
#define MAX_SUSPEND_CURRENT	(-45)		//H7 add wakup CAR range check, max suspend current
#define MAX_SUSPEND_CONSUME	(MAX_SUSPEND_CURRENT*10/36)	//H7 add wakup CAR range check, max suspend consumption(1000*mah)
#define MAX_SUSPEND_CHARGE	(1800*10/36)//H7 add wakup CAR range check, max suspend charge(1000*mah)

#define ABS(a, b) ((a>b)?(a-b):(b-a))


/*****************************************************************************
* static variables section
****************************************************************************/
static unsigned long bmu_kernel_memaddr;
bmu_data_t 	*batt_info;
gas_gauge_t	*gas_gauge;
parameter_data_t parameter_customer;
parameter_data_t *parameter;

int *rc_table;
int *xaxis_table;
int *yaxis_table;
int *zaxis_table;

int32_t	calculate_mah = 0;
int32_t	calculate_soc = 0;
static int	car_error = 0;

static char * BATT_CAPACITY	= NULL;//"/data/sCaMAH.dat";
static char * BATT_FCC 	= NULL;//"/data/fcc.dat";
static char * OCV_FLAG		= NULL;//"/data/ocv_flag.dat";
static char * BATT_OFFSET 	= NULL;//"/data/offset.dat";

static uint8_t	oz8806_pec_check  = 0;
static uint8_t	oz8806_cell_num  = 1;
static int32_t	res_divider_ratio = 2000;
static uint8_t	charge_end_flag = 0;

int32_t one_percent_rc = 0;
//static uint8_t     wait_dc_charger= 0;
//static uint8_t     wait_voltage_end = 0;

static uint8_t     wait_ocv_flag = 0;
//static uint8_t     wait_ocv_times = 0;

//uint8_t enable_fast_catch_oinom = 0;
//static uint8_t oz8806_in_suspend = 0;

//static uint8_t start_chg_count_flag = 0;
uint8_t battery_ri = 100;

//static uint8_t charge_end = 0;
//static uint8_t charge_fcc_update = 0;
//static uint8_t discharge_fcc_update = 0;

static uint8_t power_on_flag = 0;
//static uint8_t write_offset = 0;
//static uint8_t check_offset_flag = 0;

static uint8_t bmu_sleep_flag = 0;
static int32_t fRSOC_PRE;

static uint8_t discharge_end_flag = 0;
static uint8_t sleep_ocv_flag = 0;

//static struct rtc_time rtc_time;
static uint32_t	previous_loop_timex = 0;

static uint8_t set_soc_from_ext = 0;
static int32_t soc_external = -1;
static uint8_t file_not_ok_cap = 30;
static uint8_t fix_car_init = 0;
static uint8_t bmu_sys_rw_kernel = 1;
static int8_t power_on_retry_times = 4;

static  uint8_t retry_times = 0;
static uint8_t  calculate_times = 0;
static uint8_t times = 0;
static uint32_t time_sec = 0;

uint8_t check_chip_function = 1;
/******************************************************************************
* Function prototype section
* add prototypes for all functions called by this file,execepting those
* declared in header file
*****************************************************************************/

static void 	oz8806_over_flow_prevent(void);
//static void 	check_pec_control(void);

static int32_t 	afe_register_read_byte(uint8_t index, uint8_t *dat);
static int32_t 	afe_register_write_byte(uint8_t index, uint8_t dat);
static int32_t 	afe_register_read_word(uint8_t index, uint16_t *dat);
//static int32_t 	afe_register_write_word(uint8_t index, uint16_t dat);

//static int32_t 	afe_read_cell_volt(int32_t *voltage);
static int32_t 	afe_read_ocv_volt(int32_t *voltage);
//static int32_t 	afe_read_current(int32_t *dat);
static int32_t 	afe_read_car(int32_t *dat);
static int32_t 	afe_write_car(int32_t dat);
static int32_t 	afe_read_cell_temp(int32_t *date);
static int32_t 	afe_read_board_offset(int32_t *dat);
static int32_t 	afe_write_board_offset(int32_t date);

static int32_t 	oz8806_read_byte( uint8_t index);
static int32_t 	oz8806_write_byte( uint8_t index, uint8_t data);
static int32_t 	oz8806_read_word(uint8_t index);
static int32_t 	oz8806_write_word( uint8_t index, uint16_t data);
static int32_t 	oz8806_cell_voltage_read(int32_t *voltage);
static int32_t 	oz8806_ocv_voltage_read(int32_t *voltage);
//static int32_t 	oz8806_temp_read(int32_t *voltage);
static int32_t 	oz8806_current_read(int32_t *data);
static int32_t 	oz8806_car_read(int32_t *car);
static int32_t 	oz8806_car_write(int32_t data);
static uint8_t 	pec_calculate (uint8_t ucCrc, uint8_t ucData);
//static int32_t 	oz8806_write_byte_pec(uint8_t index, uint8_t data);
//static int 		bmu_check_file(char * address);
static int 		bmu_write_data(char * address,int data);
static int 		bmu_read_data(char * address);
//static int 		bmu_write_string(char * address,char * data);
static void	 	bmu_wait_ready(void);
//static int32_t 	i2c_read_byte(uint8_t addr,uint8_t index,uint8_t *data);
//static int32_t 	i2c_write_byte(uint8_t addr,uint8_t index,uint8_t data);
static void 	check_oz8806_staus(void);

//static void 	discharge_end_process(void);
//static void 	charge_end_process(void);
static void 	check_board_offset(void);
static void 	check_shutdwon_voltage(void);

static void 	trim_bmu_VD23(void);
//static int32_t oz8806_write_byte_pec(uint8_t addr,uint8_t index, uint8_t data);

void bmu_init_parameter_more(parameter_data_t *paramter_temp)
{
	if (paramter_temp->BATT_CAPACITY)
		BATT_CAPACITY = paramter_temp->BATT_CAPACITY;

	if (paramter_temp->BATT_FCC)
		BATT_FCC = paramter_temp->BATT_FCC;

	if (paramter_temp->OCV_FLAG)
		OCV_FLAG = paramter_temp->OCV_FLAG;

	if (paramter_temp->BATT_OFFSET)
		BATT_OFFSET = paramter_temp->BATT_OFFSET;

	if (paramter_temp->oz8806_cell_num)
		oz8806_cell_num = paramter_temp->oz8806_cell_num;

	if (paramter_temp->res_divider_ratio)
		res_divider_ratio = paramter_temp->res_divider_ratio;

	if (paramter_temp->file_not_ok_cap)
		file_not_ok_cap = paramter_temp->file_not_ok_cap;

	if (paramter_temp->set_soc_from_ext)
		set_soc_from_ext = paramter_temp->set_soc_from_ext;

	if (paramter_temp->fix_car_init)
		fix_car_init = paramter_temp->fix_car_init;

	//if (paramter_temp->power_on_retry_times)
	power_on_retry_times = paramter_temp->power_on_retry_times;

	if (!paramter_temp->bmu_sys_rw_kernel)
		bmu_sys_rw_kernel = paramter_temp->bmu_sys_rw_kernel;

	bmt_dbg("oz8806_cell_num:%d, res_divider_ratio: %d,file_not_ok_cap:%d, set_soc_from_ext: %d, fix_car_init:%d,power_on_retry_times:%d, bmu_sys_rw_kernel:%d\n",
				oz8806_cell_num, res_divider_ratio,file_not_ok_cap, set_soc_from_ext, fix_car_init,power_on_retry_times, bmu_sys_rw_kernel);
}
/*****************************************************************************
 * Description:
 *		bmu init chip
 * Parameters:
 *		used to init bmu
 * Return:
 *		None
 *****************************************************************************/
 void bmu_init_chip(parameter_data_t *paramter_temp)
{

	int32_t ret;
    //uint16_t value;
	uint8_t i;
	int32_t data;
	uint8_t * addr;
	config_data_t * config_test;
	uint32_t byte_num = num_0;

	byte_num = sizeof(config_data_t) + sizeof(bmu_data_t) +  sizeof(gas_gauge_t);

	memset((uint8_t *)bmu_kernel_memaddr,num_0,byte_num);

	parameter = paramter_temp;
	addr = (uint8_t *)(parameter->config);
	for(byte_num = num_0;byte_num < sizeof(config_data_t);byte_num++)
	{
		 *((uint8_t *)bmu_kernel_memaddr + byte_num) = *addr++;
	}

	config_test = (config_data_t *)bmu_kernel_memaddr;

	batt_info = (bmu_data_t *)((uint8_t *)bmu_kernel_memaddr + byte_num);

	//--------------------------------------------------------------------------------------------------
	batt_info->Battery_ok = 1;
	batt_info->i2c_error_times = num_0;

	byte_num +=  sizeof(bmu_data_t);

	gas_gauge = (gas_gauge_t *)((uint8_t *)bmu_kernel_memaddr + byte_num);
	byte_num += sizeof(gas_gauge_t);
	//--------------------------------------------------------------------------------------------------
    	gas_gauge->bmu_init_ok = 0;
	gas_gauge->charge_end = num_0;
	gas_gauge->charge_end_current_th2 = CHARGE_END_CURRENT2;
	gas_gauge->charge_strategy = 1;
	gas_gauge->charge_max_ratio = 3500;
	//gas_gauge->charge_max_ratio = 10000;  // for lianxiang

	gas_gauge->discharge_end = num_0;
	gas_gauge->discharge_current_th = DISCH_CURRENT_TH;
	gas_gauge->discharge_strategy = 1;
	gas_gauge->discharge_max_ratio = 8000;  // for lianxiang
	gas_gauge->dsg_end_voltage_hi = SHUTDOWN_HI;

	gas_gauge->batt_ri = 120;  // charging. It is related to voltage when full charging condition.
	gas_gauge->line_impedance = 0; //charging
	gas_gauge->max_chg_reserve_percentage = 1000;
	gas_gauge->fix_chg_reserve_percentage  = 0;
	gas_gauge->fast_charge_step = 2;
	gas_gauge->start_fast_charge_ratio = 1500;
	gas_gauge->charge_method_select = 0;
	gas_gauge->max_charge_current_fix = 2000;

	//add this for oinom, start
	gas_gauge->lower_capacity_reserve = 0;
	gas_gauge->lower_capacity_soc_start =30;

	gas_gauge->percent_10_reserve = 0;
    	//add this for oinom, end

	//this for test gas gaugue
	if(parameter->config->debug)
		gas_gauge->dsg_end_voltage_hi = 0;
	gas_gauge->dsg_end_voltage_th1 = SHUTDOWN_TH1;
	gas_gauge->dsg_end_voltage_th2 = SHUTDOWN_TH2;
	gas_gauge->dsg_count_2 = num_0;

	gas_gauge->overflow_data = num_32768 * num_5 * 1000 / config_data->fRsense;
	batt_dbg("yyyy gas_gauge->overflow_data is %d\n",gas_gauge->overflow_data);

	bmu_init_gg(gas_gauge);

	//--------------------------------------------------------------------------------------------------

	bmu_init_table(&xaxis_table, &yaxis_table, &zaxis_table, &rc_table);

	batt_dbg("byte_num is %d\n",byte_num);
	power_on_flag = num_0;
	bmu_sleep_flag = num_0;
	one_percent_rc = config_data->design_capacity  / num_100;

	batt_dbg("AAAA BMT fgu DRIVER VERSION is %s\n",VERSION);

	batt_dbg("BMT fgu test parameter:Rsense:%d, RTempPullup:%d, VTempRef:%d, CarLSB:%d, CurLSB:%d, VolLSB:%d, FCC:%d, CV:%d, EOC:%d, EOD:%d, BoardOffset:%d, Debug:%d\n",
		config_test->fRsense,
		config_test->temp_pull_up,
		config_test->temp_ref_voltage,
		config_test->dbCARLSB,
		config_test->dbCurrLSB,
		config_test->fVoltLSB,
		config_test->design_capacity,
		config_test->charge_cv_voltage,
		config_test->charge_end_current,
		config_test->discharge_end_voltage,
		config_test->board_offset,
		config_test->debug

	);

	//check_pec_control();
	//move check_oz8806_status here to avoid iic-commu-issue
	check_oz8806_staus();
	trim_bmu_VD23();
	/*
	 * board offset current is measured and reg0x18 value is updated only 
	 * after reset(power on reset or software reset), 
	 */
	if (oz8806_get_boot_up_time() < 625) 
	{
		batt_dbg("AAAA wait for current offset detection\n");
		msleep(625 - oz8806_get_boot_up_time());
		afe_register_read_word(OZ8806_OP_BOARD_OFFSET,(uint16_t *)&data);
	}

	batt_dbg("AAAAAAAAAAAAA Register BOARD_OFFSET: 0x%02x\n",data);

	ret  = afe_read_board_offset(&data);
	batt_dbg("AAAA board_offset is %d mA\n",data);


	if(ret >= num_0)
	{

		if(config_data->board_offset != num_0)
		{
			afe_write_board_offset(config_data->board_offset);
		}
		else if((data > num_10) || (data <= num_0))
			afe_write_board_offset(num_7);
	}

	//wake up OZ8806 into FullPower mode

	batt_dbg("BMT fgu cell_num  is  %d\n",oz8806_cell_num);

	if(oz8806_cell_num > num_1)
		afe_register_write_byte(OZ8806_OP_CTRL,num_0x2c);
	else
		afe_register_write_byte(OZ8806_OP_CTRL,num_0x20);

	ret = afe_register_read_byte(OZ8806_OP_CTRL,&i);
	batt_dbg("first read reg[0x09]= 0x%02x, ret = %d\n", i, ret);

	if((i & num_0x40) || (ret < num_0))
	{
		if(oz8806_cell_num > num_1)
		{
			afe_register_write_byte(OZ8806_OP_CTRL,num_0x2c);
		}
		else
			afe_register_write_byte(OZ8806_OP_CTRL,0x20);
		batt_dbg("Wake up BMT fgu write op_ctrl 0x20 \n");

		ret = afe_register_read_byte(OZ8806_OP_CTRL,&i);
		batt_dbg("second read reg[0x09]= 0x%02x, ret = %d\n", i, ret);
	}

	batt_dbg("BMT fgu parameter  %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n",
		config_data->fRsense,
		config_data->temp_pull_up,
		config_data->temp_ref_voltage,
		config_data->dbCARLSB,
		config_data->dbCurrLSB,
		config_data->fVoltLSB,
		config_data->design_capacity,
		config_data->charge_cv_voltage,
		config_data->charge_end_current,
		config_data->discharge_end_voltage,
		config_data->board_offset,
		config_data->debug
	);

	//check_oz8806_staus();

	//trim_bmu_VD23();
}


/*****************************************************************************
 * Description:
 *		wait ocv flag
 * Parameters:
 *		None
 * Return:
 *		None
 *****************************************************************************/
 void wait_ocv_flag_fun(void)
{

	int32_t ret;
    uint16_t value;
	uint8_t i;
	int32_t data;
#if 1
	if (oz8806_get_boot_up_time() < 1500)// ocv can be stable after oz8806 is wakeuped for 1.375 s
	{
		batt_dbg("wait ocv flag register ready 1\n");
		msleep(1500 - oz8806_get_boot_up_time());
		oz8806_get_boot_up_time();
		//return;
	}
/*
	if (oz8806_get_power_on_time() < 2250)
	{
		batt_dbg("wait ocv flag register ready 2\n");
		msleep(2250 - oz8806_get_power_on_time());
		oz8806_get_power_on_time();
	}
*/
#endif

	wait_ocv_flag = 1;

	for(i = 0;i < 3;i++)
	{
		ret = afe_register_read_word(OZ8806_OP_OCV_LOW,&value);
		if(ret >= 0)
		{
			if ((value & POWER_OCV_FLAG) || (value & SLEEP_OCV_FLAG))
			{
				if (oz8806_get_power_on_time() < 2250)//2250+2000
				{
					batt_dbg("ocv or sleep_ocv: wait for normal scan cycle\n");
					oz8806_get_power_on_time();
				}
			}
			else if (oz8806_get_boot_up_time() < 2250) 
			{
				batt_dbg("normal ok: wait for normal scan cycle\n");
			}
			break;
		}
	}

	//read data
	afe_read_cell_volt(&batt_info->fVolt);
	afe_read_current(&batt_info->fCurr);
	afe_read_cell_temp(&batt_info->fCellTemp);
	afe_read_car(&batt_info->fRC);

	batt_info->fRSOC = batt_info->fRC   * num_100;
	batt_info->fRSOC = batt_info->fRSOC /  config_data->design_capacity ;

	batt_dbg("read batt_info.fVolt is %d\n",(batt_info->fVolt * oz8806_cell_num));
	batt_dbg("read batt_info.fRC is %d\n",batt_info->fRC);
	batt_dbg("read batt_info.fCurr is %d\n",batt_info->fCurr);
	batt_dbg("read ocv flag ret is 0x%02x,value is 0x%02x\n",ret,value);
	batt_dbg("read fCellTemp is %d\n",batt_info->fCellTemp);
	if (ret >= num_0 )
	{
		// OZ8806 First power on 
		if (value & POWER_OCV_FLAG)
		{
			power_on_flag = num_1;
			afe_read_ocv_volt(&batt_info->fOCVVolt);

			afe_read_cell_volt(&batt_info->fVolt);
			if(oz8806_cell_num > num_1)
				batt_info->fOCVVolt = batt_info->fVolt;
			batt_dbg("AAAA ocv volt is %d\n",(batt_info->fOCVVolt * oz8806_cell_num));
			batt_dbg("AAAA volt is %d\n",(batt_info->fVolt * oz8806_cell_num));
			if(batt_info->fOCVVolt  > (config_data->charge_cv_voltage + 150)){
            		if (oz8806_get_power_on_time() < 2250)//2250+2000
			{
				batt_dbg("ocv wait for normal scan cycle\n");
				msleep(2250 - oz8806_get_power_on_time());
				oz8806_get_power_on_time();
			}

				afe_read_cell_volt(&batt_info->fVolt);
				batt_info->fOCVVolt = batt_info->fVolt;
				batt_dbg("AAAAA ocv data errror ,so batt_info->fVolt is %d\n",batt_info->fVolt);
			}

			afe_read_current(&batt_info->fCurr);
			batt_dbg("AAAA batt_info.fCurr is %d\n",batt_info->fCurr);

			if(batt_info->fCurr > num_50)
				data = batt_info->fOCVVolt - num_100;
			else
				data = batt_info->fOCVVolt;
			batt_info->fRSOC = one_latitude_table(parameter->ocv_data_num,parameter->ocv,data);
			batt_dbg("find ocv table batt_info.fRSOC is %d\n",batt_info->fRSOC);
			if((batt_info->fRSOC >num_100) || (batt_info->fRSOC < num_0))
				batt_info->fRSOC = num_50;

			batt_info->fRC = batt_info->fRSOC * config_data->design_capacity / num_100;
			afe_read_current(&batt_info->fCurr);
			if(batt_info->fRC  >= (gas_gauge->overflow_data -num_10))
			{
				batt_info->fRC = gas_gauge->overflow_data  - gas_gauge->overflow_data /num_100;
				batt_info->fRCPrev = batt_info->fRC;
			}
			afe_write_car(batt_info->fRC);
			batt_info->fRCPrev = batt_info->fRC;
			gas_gauge->fcc_data= config_data->design_capacity * 100/100;

			batt_dbg("Power on mode is activated \n");

			batt_info->sCaMAH = batt_info->fRSOC * config_data->design_capacity / num_100;
			gas_gauge->sCtMAH = batt_info->sCaMAH; 
			gas_gauge->discharge_sCtMAH = config_data->design_capacity - batt_info->sCaMAH; 
		}
		else if(value & SLEEP_OCV_FLAG)
		{
			afe_read_ocv_volt(&batt_info->fOCVVolt);
			sleep_ocv_flag = 1;
			batt_dbg("Sleep ocv mode is activated \n");
		}
		else
			batt_dbg("Normal mode is activated \n");
	}
	else
	{
		batt_dbg("AAAA BMT fgu DRIVER \n");
		batt_dbg("AAAA BMT fgu read BMT_OP_OCV_LOW\n");

	}

	//for ifive
#if 0
	afe_read_car(&batt_info->fRC);
	if((batt_info->fRC < (one_percent_rc)) && (batt_info->fRC > -3000))
	{
		batt_dbg("yyyy fRC will over fRC  is %d\n",batt_info->fRC);
		batt_info->fRC = one_percent_rc - num_1;
		afe_write_car(batt_info->fRC);
		batt_info->fRCPrev = batt_info->fRC;
	}
	afe_read_car(&batt_info->fRC);
	batt_dbg("fRC  is %d\n",batt_info->fRC);
#endif
	afe_register_read_byte(num_0,&i);
	batt_dbg("BMT fgu regeidter 0x00 is 0x%02x\n",i);

	afe_register_read_byte(num_9,&i);
	batt_dbg("BMT fgu regeidter 0x09 is 0x%02x\n",i);
}

/*****************************************************************************
 * Description:
 *		trim_bmu_VD23
 * Parameters:
 *		None
 * Return:
 *		None
 *****************************************************************************/
void trim_bmu_VD23(void)
{
	int32_t ret;
	uint8_t i;
	uint16_t value;
	uint8_t data;
	uint8_t temp;

	for(i = 0;i < 3;i++)
	{
		ret = afe_register_read_word(OZ8806_OP_VD23,&value);
		if(ret >= 0)break;
	}
	if(ret < 0)
	{
		batt_dbg("READ BMT_OP_VD23 fail and return:%d\n",ret);
		return;
	}
	batt_dbg("READ BMT REG 0x03 is 0x%02x\n",value);
	data = ((value >> 6) & 0x02) | ((value >> 8) & 0x01);
	data &= 0x03; 
	batt_dbg("So trim is 0x%02x\n",data);

	for(i = 0;i < 3;i++)
	{
		ret = afe_register_read_byte(OZ8806_OP_PEC_CTRL,&temp);
		if(ret >= 0)break;
	}
	if(ret < 0)
	{
		batt_dbg("READ BMT_OP_PEC_CTRL fail and return:%d\n",ret);
		return;
	}
	batt_dbg("READ BMT fgu REG 0x08 is 0x%02x\n",temp);

	temp &= (~0x03);
	if(data != 0x02)
	{
		temp |= data;
		for(i = 0;i < 3;i++)
		{
			ret = afe_register_write_byte(OZ8806_OP_PEC_CTRL,temp);
			batt_dbg("write1 BMT fgu REG 0x08 is 0x%02x\n",temp);
			if(ret >= 0)break;
		}
	}
	else
	{
		temp |= 0x03;
		for(i = 0;i < 3;i++)
		{
			ret = afe_register_write_byte(OZ8806_OP_PEC_CTRL,temp);
			batt_dbg("write2 BMT fgu REG 0x08 is 0x%02x\n",temp);
			if(ret >= 0)break;
		}
	}
	ret = afe_register_read_byte(OZ8806_OP_PEC_CTRL,&temp);
	batt_dbg("BMT fgu REG 0x08 is 0x%02x\n",temp);

}

/*****************************************************************************
* Description:
*		check shudown voltage control
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
void check_shutdwon_voltage(void)
{
	//int32_t ret;
	//uint8_t i;
	int32_t temp;
	//int32_t infVolt;
	afe_read_cell_volt(&batt_info->fVolt);

	if(!sleep_ocv_flag) return;

	temp = one_latitude_table(parameter->ocv_data_num,parameter->ocv,batt_info->fOCVVolt);
	batt_dbg("Sleep data is %d \n",temp);

	// select higher data 
	if(calculate_soc > temp)
		temp  = calculate_soc;

	//Very dangerous
	if((batt_info->fRSOC - temp) > 20 
	|| (batt_info->fVolt < (config_data->discharge_end_voltage - 50) && temp <= 5))
	{
		//batt_info->fRSOC -= 5;
		batt_info->fRSOC = temp;
		if(batt_info->fRSOC < 0)	batt_info->fRSOC = 0;

		batt_info->sCaMAH = batt_info->fRSOC * gas_gauge->fcc_data/ 100;

		batt_info->fRC= batt_info->sCaMAH;
		if(batt_info->fRC >  (gas_gauge->overflow_data - 20))
			 batt_info->fRC = gas_gauge->overflow_data - 20;
		afe_write_car(batt_info->fRC);

		batt_info->fRCPrev 	= batt_info->fRC;

		gas_gauge->sCtMAH = batt_info->sCaMAH;
		if(gas_gauge->fcc_data > batt_info->sCaMAH)
			gas_gauge->discharge_sCtMAH = gas_gauge->fcc_data- batt_info->sCaMAH;
		else
			gas_gauge->discharge_sCtMAH = 0;
	}
}

/*****************************************************************************
* Description:
*		check_oz8806_staus
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
static void check_oz8806_staus(void)
{

}
/*****************************************************************************
* Description:
*		bmu_wait_ready
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
static void bmu_wait_ready(void)
{
	int32_t store_rc = INIT_CAP;
	//int32_t fcc;
	uint8_t i;
	int32_t ret;
	uint16_t value;	
	int32_t calculate_soc_temp;
	int32_t file_soc_temp;

	batt_dbg("AAAA bmu wait times %d \n",times);
	times ++;
	//wake up OZ8806 into FullPower mode
	oz8806_wakeup_full_power();

	ret = afe_register_read_byte(OZ8806_OP_CTRL,&i);
	batt_info->Battery_ok = i & BI_STATUS;
	batt_dbg("bmu_wait_ready read 0x09 ret is %d,i is 0x%02x\n",ret,i);

	if((i & 0x40) || (ret < 0))
	{
		if(oz8806_cell_num > 1)
		{
			afe_register_write_byte(OZ8806_OP_CTRL,0x2c);
		}
		else
			afe_register_write_byte(OZ8806_OP_CTRL,0x20);
		batt_dbg("BMT fgu wake up function\n");

		ret = afe_register_read_byte(OZ8806_OP_CTRL,&i);
		batt_dbg("bmu_wait_ready read 0x09 ret is %d,i is 0x%02x\n",ret,i);
	}

	//do we need add check ocv flag now?
	ret = afe_register_read_word(OZ8806_OP_OCV_LOW,&value);
	if ((ret >= 0) && (value & POWER_OCV_FLAG))
	{
		batt_dbg("read flag too late\n");
		wait_ocv_flag_fun();
	}

	afe_read_car(&batt_info->fRC);
	if((batt_info->fRC < 5)|| (power_on_flag))
	{

        if (oz8806_get_power_on_time() < 2250)
	{
	    	batt_dbg("wait for normal scan cycle and return \n");
	    	return;
    		//msleep(2250 - oz8806_get_power_on_time());
    	}
    	else
    		batt_dbg("normal scan cycle delay ok\n");
	}
	else
    	batt_dbg("means normal boot,no need delay\n");

	afe_read_cell_temp(&batt_info->fCellTemp);
	afe_read_cell_volt(&batt_info->fVolt);
	afe_read_current(&batt_info->fCurr);

	batt_dbg("fVolt is %d\n",(batt_info->fVolt * oz8806_cell_num));
	batt_dbg("fCurr is %d\n",batt_info->fCurr);
	batt_dbg("ftemp is %d\n",batt_info->fCellTemp);
	batt_dbg("fRC is %d\n",batt_info->fRC);

	// every time we check this and print to see if the voltage and temp and current is abnormal
	// how about if rc_result error

	calculate_soc_temp = calculate_soc_result();

	calculate_soc = calculate_soc_temp;
	calculate_mah = calculate_soc * config_data->design_capacity / num_100;

	calculate_times++;
#if 0
	if (calculate_times <= calculate_retry_times)
		return;
#endif
	batt_dbg("calculate_times: %d\n",calculate_times);

	gas_gauge->fcc_data = config_data->design_capacity;

	if((batt_info->fRC > 5)&& (!power_on_flag))
	{
		store_rc = batt_info->fRC;
		//calculate_retry_times = 0;
		batt_dbg("Success to get soc from car :%d\n", batt_info->fRC);
		goto file_ok;
	}
/*
	if (oz8806_get_boot_up_time() < 5000) // voltage can be stable after oz8806 is powered for 5 s  
	{
		batt_dbg("wait oz8806 ADC ready\n");
		return;
	}
    */
	if((set_soc_from_ext))
	{
		soc_external = oz8806_get_soc_from_ext();

		batt_dbg("soc from external: %d\n", soc_external);

		if (soc_external == -1)
		{
			batt_dbg("wait for external soc:%d, retry:%d\n", soc_external, retry_times);

			if(retry_times >= 0)
			{
				batt_dbg("fail to get externl soc, retry_times:%d \n",retry_times);
				retry_times = 0; 
				set_soc_from_ext = 0;
				batt_dbg("can't get external soc,so make set_soc_from_ext 0\n");
			}
			else
			{
				retry_times++;
				return;
			}
		}
		else if ((soc_external <= 100) && (soc_external >= 0))
		{
			batt_dbg("Success to get soc from external :%d\n", soc_external);

			store_rc = soc_external * gas_gauge->fcc_data  / 100;
			store_rc += gas_gauge->fcc_data  / 200; // + fcc / 100 * / 2
			batt_dbg("stored RC:%d\n", store_rc);

			goto file_ok;
		}
		else
		{
			batt_dbg("Error to get wrong soc from external :%d\n", soc_external);
			retry_times = 0; 
			set_soc_from_ext = 0;
			batt_dbg("can't get external soc,so make set_soc_from_ext 0\n");
		}
	}

	if (bmu_sys_rw_kernel)// this ok
	{
		/* wait for sysfs to capacity from file*/
		store_rc = bmu_read_data(BATT_CAPACITY);

		//check file ,if file not exit, goto file fail
		if(store_rc < 0)
		{
			batt_dbg("open BATT_CAPACITY fail, retry_times:%d \n", retry_times);
			if(retry_times >= power_on_retry_times)
			{
				batt_dbg("BATT_CAPACITY file fail\n");
				batt_info->sCaMAH = calculate_mah;
				goto file_fail;
			}
			else
			{
				retry_times++;
				return;
			}
		}
		batt_dbg("open BATT_CAPACITY success, retry_times:%d \n", retry_times);
		batt_dbg("AAAA read battery capacity data is %d\n", store_rc);
	}
	else
	{
		store_rc = oz8806_get_save_capacity();
		batt_dbg("RC from file:%d\n", store_rc);
		if (store_rc == INIT_CAP)
        	{
            		batt_dbg("wait for reading file:%s, retry:%d\n", BATT_CAPACITY, retry_times);
			if(retry_times > 10)
        		{
                   		batt_dbg("fail to get save_capacity, retry_times:%d \n",retry_times);
                   		goto file_fail;
			}
		retry_times++;
		return;
        	}
		else if (store_rc == NO_FILE)
		{
			batt_dbg("no file %s, retry_times:%d \n", BATT_CAPACITY, retry_times);
			goto file_fail;
		}
	}

file_ok:

	//file_soc_temp: soc read from file
	if (soc_external != -1)
		file_soc_temp = soc_external;
	else
		file_soc_temp = store_rc * 100 / gas_gauge->fcc_data;

	//file ok, if ocv power on 
	if(power_on_flag)
	{
		batt_dbg("bmu_wait_ready, power on ok \n");
		gas_gauge->ocv_flag = 1;

		//file_soc_temp: soc read from file
		//batt_info->fRSOC: calculated using OCV 
		//calculated_soc: calculated using VI
		batt_dbg("soc from file:%d, vi soc:%d, ocv soc:%d \n",
			    file_soc_temp, calculate_soc, batt_info->fRSOC);

		batt_dbg("RC from file:%d, vi RC:%d, ocv RC:%d \n",
			    store_rc, calculate_mah, batt_info->sCaMAH);

		if((batt_info->fRSOC  > 90)&& (file_soc_temp > 90))
		{
			batt_info->sCaMAH = store_rc;
			batt_dbg("start sCaMAH high USE file\n");
		}
		else if(batt_info->fCurr >= 0)
		{
			if( ABS(calculate_soc,file_soc_temp) > 30)
			{
				batt_info->sCaMAH = calculate_mah;
				batt_dbg("start sCaMAH chg USE vi\n");
			}
			else // file
			{
				batt_info->sCaMAH = store_rc;
				batt_dbg("start sCaMAH chg USE file\n");
			}
		}
		else //discharging
		{
			if( ABS(file_soc_temp,calculate_soc) > 20 
			|| (batt_info->fVolt < (config_data->discharge_end_voltage - 50) && file_soc_temp<=5))
			{
				batt_info->sCaMAH = calculate_mah;
				batt_dbg("start sCaMAH USE vi\n");
			}
			else
			{
				batt_info->sCaMAH = store_rc;
				batt_dbg("start sCaMAH USE file\n");
			}
		}
	}
	//if not ocv power on
	else
	{	batt_dbg("bmu_wait_ready, normal ok \n");
		gas_gauge->ocv_flag = 0;
		afe_read_car(&batt_info->fRC);

		batt_dbg("RC from file: %d ,RC from register: %d\n", store_rc, batt_info->fRC);

		if ((batt_info->fCurr > 0 && batt_info->fRC <= 2) 
		 || (batt_info->fCurr <= 0 && batt_info->fRC <= 0))//power on, no ocv flag
		{
			if((ABS(file_soc_temp,calculate_soc) > 30)
			|| (batt_info->fVolt < (config_data->discharge_end_voltage - 50) && file_soc_temp<=5))
			{
				batt_info->sCaMAH = calculate_mah;
				batt_dbg("file ok but USE vi,fRC<=0\n");
			}
			else
			{
				batt_info->sCaMAH = store_rc;
				batt_dbg("file ok USE file\n");
			}
		}
		else
		{
			// car and file changed more than 1%
			if(( ABS(batt_info->fRC,store_rc) > (one_percent_rc)))
			{
				batt_info->sCaMAH = batt_info->fRC;
				batt_dbg("file ok  USE RC\n");
			}
			else
			{
				//prevent issue: when reboot frequently, soc -= 1 every reboot
				if (((soc_external <= 100) && (soc_external >= 0))
					&&(batt_info->fRC > store_rc)
					&&(batt_info->fRC - store_rc <= one_percent_rc))
				{
					batt_dbg("file ok USE RC\n");
					batt_info->sCaMAH = batt_info->fRC;
				}
				else
				{
					batt_info->sCaMAH = store_rc;
					batt_dbg("file ok USE file\n");
				}
			}
		}
	}

file_fail:
	if(store_rc<0) //fail to red fil or no file exist
	{
		if(power_on_flag)
		{
			batt_dbg("fVolt %dmv,cell_num:%d, power_on_100_vol:%dmv\n",batt_info->fVolt,oz8806_cell_num,gas_gauge->power_on_100_vol);
			if((batt_info->fVolt * oz8806_cell_num)>= gas_gauge->power_on_100_vol)
				batt_info->sCaMAH = full_charge_data;
			else
				batt_info->sCaMAH = calculate_mah;
			batt_dbg("bmu_wait_ready, power on ok and file fail \n");
			batt_dbg("file fail USE vi\n");
		}
		else
		{
			batt_dbg("bmu_wait_ready, normal on ok and file fail \n");
			afe_read_car(&batt_info->fRC);

			batt_dbg("RC from register: %d, RC from vi: %d\n", batt_info->fRC, calculate_mah);

			if ((batt_info->fCurr > 0 && batt_info->fRC <= 2) 
			 || (batt_info->fCurr <= 0 && batt_info->fRC <= 0))//power on, no ocv flag
			{
				batt_dbg("big error first power on and ocv flag not set: %d ,%d\n", batt_info->fRC, calculate_mah);
				batt_info->sCaMAH = calculate_mah;
				batt_dbg("file fail USE vi\n");
			}
			else
			{
#if 0
				batt_info->sCaMAH = batt_info->fRC;
				batt_dbg("file fail USE RC\n");
#else

				if(batt_info->fRC < one_percent_rc )    //if CAR < 1%  
					file_not_ok_cap = 30;

				if( ABS(batt_info->fRC,calculate_mah) > (file_not_ok_cap * config_data->design_capacity / num_100))
				{
					batt_info->sCaMAH = calculate_mah;
					batt_dbg("file fail USE vi\n");

				}
				else
				{
					batt_info->sCaMAH = batt_info->fRC;
					batt_dbg("file fail USE RC\n");
				}

				if(fix_car_init)
				{
					batt_info->sCaMAH = batt_info->fRC;
					batt_dbg("file fail fix fRC: %d,%d \n",batt_info->sCaMAH,batt_info->fRC);
				}
#endif
			}
		}
	}

	if(batt_info->sCaMAH <= num_0)
	{
		batt_dbg("calculate_soc is %d\n",calculate_soc);
		afe_read_cell_volt(&batt_info->fVolt);
		batt_info->fRSOC = calculate_soc;
		if((batt_info->fRSOC >num_100) || (batt_info->fRSOC < num_0))
			batt_info->fRSOC = num_50;

		batt_dbg("batt_info->fRSOC is %d\n",batt_info->fRSOC);
		batt_info->fRC = batt_info->fRSOC * config_data->design_capacity / num_100;
		batt_dbg("batt_info->fRC is %d\n",batt_info->fRC);
		if(batt_info->fRC  >= (gas_gauge->overflow_data -num_10))
		{
			batt_info->fRC = gas_gauge->overflow_data  - gas_gauge->overflow_data/num_100;
			batt_info->fRCPrev = batt_info->fRC;
		}
		batt_dbg("batt_info->fRC is %d\n",batt_info->fRC);

		afe_write_car(batt_info->fRC);
		batt_info->fRCPrev = batt_info->fRC;

		batt_info->sCaMAH = batt_info->fRC;
	}
	if(batt_info->sCaMAH > full_charge_data)
	{
		batt_info->sCaMAH = full_charge_data;
	}
	batt_dbg("batt_info->sCaMAH is %d\n",batt_info->sCaMAH);

	batt_info->fRSOC = batt_info->sCaMAH   * num_100;
	batt_info->fRSOC = batt_info->fRSOC / gas_gauge->fcc_data ;

	//check if batt_info->sCaMAH == soc_external * gas_gauge->fcc_data  / 100;
	if ((soc_external <= 100) && (soc_external >= 0))
		if (batt_info->sCaMAH == soc_external * gas_gauge->fcc_data  / 100)
			batt_info->fRSOC = soc_external;

	//batt_info->fRC = batt_info->fRSOC * gas_gauge->fcc_data / num_100;//this make result smaller
	batt_info->fRC = batt_info->sCaMAH;
	if(batt_info->fRC  >= (gas_gauge->overflow_data -num_10))
	{
		batt_info->fRC = gas_gauge->overflow_data  - gas_gauge->overflow_data/num_100;
		batt_info->fRCPrev = batt_info->fRC;
	}
	gas_gauge->sCtMAH = batt_info->sCaMAH;

	batt_dbg("AAAA batt_info->fVolt is %d\n",(batt_info->fVolt * oz8806_cell_num));
	batt_dbg("AAAA batt_info->fRSOC is %d\n",batt_info->fRSOC);
	batt_dbg("AAAA batt_info->sCaMAH is %d\n",batt_info->sCaMAH);
	batt_dbg("AAAA gas_gauge->sCtMAH is %d\n",gas_gauge->sCtMAH);
	batt_dbg("AAAA fcc is %d\n",gas_gauge->fcc_data);
	batt_dbg("AAAA batt_info->fRC is %d\n",batt_info->fRC);
	batt_dbg("AAAA batt_info->fCurr is %d\n",batt_info->fCurr);

	afe_write_car(batt_info->fRC);
	batt_info->fRCPrev = batt_info->fRC;

	if(gas_gauge->fcc_data > batt_info->sCaMAH)                                  
		gas_gauge->discharge_sCtMAH = gas_gauge->fcc_data- batt_info->sCaMAH;    
	else                                                                         
		gas_gauge->discharge_sCtMAH = 0;                                         

	if(batt_info->fRSOC <= num_0)
	{
		gas_gauge->discharge_end = num_1;
		batt_info->fRSOC = num_0;
		gas_gauge->sCtMAH = num_0;
		gas_gauge->ocv_flag = 0;
		bmu_write_data(OCV_FLAG,0);
	}
	if(batt_info->fRSOC >= num_100)
	{
		gas_gauge->charge_end = num_1;
		charge_end_flag = num_1;
		batt_info->fRSOC = num_100;
		gas_gauge->discharge_sCtMAH = num_0;
		gas_gauge->discharge_fcc_update = num_1;
		gas_gauge->ocv_flag = 0;
		bmu_write_data(OCV_FLAG,0);
	}

	if(sleep_ocv_flag)
		check_shutdwon_voltage();

	fRSOC_PRE = batt_info->fRSOC;

	// double check
	if (bmu_sys_rw_kernel)
	{
		store_rc =  bmu_read_data(BATT_CAPACITY);
		if(store_rc != batt_info->sCaMAH)
		{
			bmu_write_data(BATT_CAPACITY,batt_info->sCaMAH);
			bmu_write_data(BATT_FCC, gas_gauge->fcc_data);
			//batt_dbg("init write sCaMAH  %d,%d\n",batt_info->sCaMAH,store_rc);
		}
	}
	gas_gauge->bmu_init_ok = num_1;
}

/*****************************************************************************
* Description:
*		power down  8806 chip
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
void bmu_power_down_chip(void)
{
	afe_read_cell_temp(&batt_info->fCellTemp);
	afe_read_cell_volt(&batt_info->fVolt);
	afe_read_current(&batt_info->fCurr);
	afe_read_car(&batt_info->fRC);

	if(oz8806_cell_num > num_1)
		afe_register_write_byte(OZ8806_OP_CTRL,num_0x4c |SLEEP_OCV_EN);
	else
		afe_register_write_byte(OZ8806_OP_CTRL,SLEEP_MODE | SLEEP_OCV_EN);

	if(parameter->config->debug){
		batt_dbg("eeee power down BMT fgu \n");
		batt_dbg("eeee batt_info->fVolt is %d\n",(batt_info->fVolt * oz8806_cell_num));
		batt_dbg("eeee batt_info->fRSOC is %d\n",batt_info->fRSOC);
		batt_dbg("eeee batt_info->sCaMAH is %d\n",batt_info->sCaMAH);
		batt_dbg("eeee batt_info->fRC is %d\n",batt_info->fRC);
		batt_dbg("eeee batt_info->fCurr is %d\n",batt_info->fCurr);
	}
}

/*****************************************************************************
* Description:
*		power up  8806 chip
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
void bmu_wake_up_chip(void)
{
	uint8_t data;
	int32_t ret;
	int32_t value;
	uint8_t discharge_flag = num_0;
	uint8_t i;
	int32_t sleep_time = 0;
	int32_t	car_wakeup = 0;
	int8_t adapter_status = 0;

	batt_dbg("fCurr is %d\n",batt_info->fCurr);

	adapter_status = get_adapter_status();
	if ((batt_info->fCurr <= gas_gauge->discharge_current_th) ||
 		(adapter_status == O2_CHARGER_BATTERY))
	{
		discharge_flag = 1;
	}

	batt_dbg("adapter_status: %d\n", adapter_status);

	bmu_sleep_flag = 1;

	time_sec = oz8806_get_system_boot_time();
	sleep_time = time_sec - previous_loop_timex;
	batt_dbg("sleep time: %d s\n",sleep_time);

	oz8806_wakeup_full_power();

	ret = afe_register_read_byte(OZ8806_OP_CTRL,&data);//wake up OZ8806 into FullPower mode
	if((data & 0x40) || (ret < 0))
	{
		batt_dbg("bmu_wake_up_chip read 0x09 ret is %d,i is %d \n",ret,data);
		if(oz8806_cell_num > 1)
		{
			afe_register_write_byte(OZ8806_OP_CTRL,0x2c);
		}
		else
			afe_register_write_byte(OZ8806_OP_CTRL,0x20);
		batt_dbg("BMT fgu wake up function\n");

	}

	if(batt_info->fRSOC >=num_100)
	{
		if(batt_info->fRSOC >= num_100)	batt_info->fRSOC = num_100;
		if(batt_info->sCaMAH > (full_charge_data))
		{
			batt_dbg("BMT fgu wake up batt_info.sCaMAH big error is %d\n",batt_info->sCaMAH);
			batt_info->sCaMAH = full_charge_data;
		}
	}

	afe_read_current(&batt_info->fCurr);
	afe_read_cell_volt(&batt_info->fVolt);

	batt_dbg("sCaMAH:%d,fRC:%d,fcurr:%d,volt:%d,sCtMAH:%d\n",
		batt_info->sCaMAH,batt_info->fRC,batt_info->fCurr,
		(batt_info->fVolt * oz8806_cell_num),
		gas_gauge->sCtMAH);

	for(i = num_0;i< num_3;i++)
	{
		ret = afe_read_car(&car_wakeup);
		if (ret >= 0)
		{
			if ((car_wakeup == 0) || ((!discharge_flag) && (car_wakeup < 0)))
			{
				car_error += 1;
				batt_dbg("test fRC read: %d, %d\n",car_error, car_wakeup);	//H7 Test Version
			}
			else
				break;
		}
	}

	value = car_wakeup - batt_info->fRC;
	//*******CAR zero reading Workaround******************************
	if (sleep_time < 0)	sleep_time = 60;			//force 60s
	if (discharge_flag == 1)
		sleep_time *= MAX_SUSPEND_CONSUME;			//time * current / 3600 * 1000
	else
		sleep_time *= MAX_SUSPEND_CHARGE;			//time * current / 3600 * 1000

	// 10%
	if (ABS(car_wakeup, batt_info->fRC) > (10 * config_data->design_capacity / num_100))		//delta over 10%
	{
		if (((value * 1000) - sleep_time) < 0)				//over max car range
		{
			value = (sleep_time / 1000);
			batt_dbg("Ab CAR:%d,mod:%d\n",car_wakeup,value);
			car_wakeup = value + batt_info->fRC;
			afe_write_car(car_wakeup);
		}
	}
	batt_dbg("CAR Prev:%d,CAR Now:%d\n",batt_info->fRC,car_wakeup);
	//*******CAR zero reading Workaround******************************
	if(car_wakeup <= 0)  
	{
	    batt_dbg("fRC is %d\n",car_wakeup);
		//oz8806_over_flow_prevent();
		gas_gauge->charge_fcc_update = num_0;
		gas_gauge->discharge_fcc_update = num_0;
		
		if (car_wakeup < 0)	//overflow check
		{
			if (batt_info->fVolt >= (config_data->charge_cv_voltage - 50))
			{
				car_wakeup = gas_gauge->fcc_data -num_1;
			}
			else
			{
				if (batt_info->fRSOC > 0)
					car_wakeup = batt_info->fRSOC * gas_gauge->fcc_data / num_100 - num_1;
				else
					car_wakeup = batt_info->sCaMAH;
			}
			afe_write_car(car_wakeup);
			batt_info->fRC = car_wakeup;
			batt_info->fRCPrev = batt_info->fRC;
		}

		if(discharge_flag)
		{
			check_shutdwon_voltage();
			value = calculate_soc_result();
			if(value < batt_info->fRSOC)
			{
				batt_info->fRSOC = value;
				batt_info->sCaMAH = batt_info->fRSOC * gas_gauge->fcc_data  / 100;
				batt_info->fRC = batt_info->sCaMAH;
				batt_info->fRCPrev = batt_info->fRC;
				afe_write_car(batt_info->fRC);
			}

			if(batt_info->fRSOC <= 0)
			{
				if((batt_info->fVolt <= config_data->discharge_end_voltage))
				{
					discharge_end_process();
				}
				//wait voltage
				else
				{
					batt_info->fRSOC  = 1;
					batt_info->sCaMAH = gas_gauge->fcc_data / num_100 ;
					gas_gauge->discharge_end = num_0;
				}
			}
			return;
		}
	}

	if ((discharge_flag) && (car_wakeup > batt_info->fRC))
	{
		batt_dbg("it seems error 1\n");
		value = car_wakeup - batt_info->fRC;
		car_wakeup = batt_info->fRC - value;
		afe_write_car(car_wakeup);
	}
	if ((!discharge_flag) && (car_wakeup < batt_info->fRC))
	{
		batt_dbg("it seems error 2\n");
		value = batt_info->fRC - car_wakeup;
		car_wakeup = batt_info->fRC + value;
		afe_write_car(car_wakeup);
	}

	batt_dbg("tttt final wakeup car is %d\n",car_wakeup);
}

/*****************************************************************************
* Description:
*		check_board_offset
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
static void check_board_offset(void)
{
	int32_t data;
	int32_t ret;
	int32_t offset;
	int32_t ret2;

	ret2 = afe_read_board_offset(&data);

	if(config_data->board_offset != data && config_data->board_offset != num_0)
	{
		afe_write_board_offset(config_data->board_offset);
		return;
	}

	ret = bmu_read_data(BATT_OFFSET);
	//batt_dbg("AAAA board_offset is  %d\n",data);
	if(ret < num_0)
	{

		if((data > num_10) || (data <= num_0))
			afe_write_board_offset(7);

		if(ret2 >= num_0)
		{
			if((data < num_10) && (data > num_0) && (data != num_0))
			{
				ret = bmu_write_data(BATT_OFFSET,data);

				if(ret <num_0)
					batt_dbg("first write board_offset error\n");

				data = bmu_read_data(BATT_OFFSET);

				batt_dbg("first write board_offset is %d\n",data);
			}
		}
	}
	else
	{
		offset = bmu_read_data(BATT_OFFSET);
		if(((offset - data) > num_2) || ((offset - data) < -num_2))
			afe_write_board_offset(offset);
	}

	afe_read_board_offset(&data);
	if((data > num_10) || (data <= num_0))
		afe_write_board_offset(num_7);

	afe_read_board_offset(&data);
	batt_dbg("AAAA board_offset is  %dmA\n",data);
}

/*****************************************************************************
* Description:
*		bmu polling loop
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
void bmu_polling_loop(void)
{
	int32_t data;

	int32_t ret;
	uint8_t i;
	static uint8_t error_times = 0;

	int8_t adapter_status = 0;
	uint8_t retry = 0;
	uint8_t val = 0;
	if(is_battery_exchanged()){ //battery was exchanged, oz8806 in start-up mode
		gas_gauge->bmu_init_ok = 0;
		batt_info->i2c_error_times = 0;
		batt_dbg("battery was exchanged, init SOC again\n");

retry_wakeup:
		//wake up this stupid chip 
		oz8806_wakeup_full_power();
		msleep(10) ;
		ret = afe_register_read_byte(OZ8806_OP_CTRL,&val);
		if((val & 0x40) || (ret < 0))
		{
			batt_dbg("bmu_wake_up_chip read 0x09 ret is %d,i is %d \n",ret,val);
			retry++;

			if(retry>10){
				batt_dbg("fail wakeup ic");
			}
			goto retry_wakeup;
		}

		retry_times = 0;
		calculate_times = 0;
		times = 0;
		wait_ocv_flag = 0;
		wait_ocv_flag_fun();
	}

	batt_dbg("-----------------------bmu_polling_loop start------------------\n");

	if(!(gas_gauge->bmu_init_ok))
	{
		if(!wait_ocv_flag)
			wait_ocv_flag_fun();
		else
			bmu_wait_ready();

		if(gas_gauge->bmu_init_ok)
			check_board_offset();

		goto end;
	}

	batt_info->fRCPrev = batt_info->fRC;

	afe_read_cell_temp(&batt_info->fCellTemp);
	afe_read_cell_volt(&batt_info->fVolt);
	afe_read_current(&batt_info->fCurr);

	//be careful for large current
	//and wake up condition
	ret = afe_read_car(&data);
	if(parameter->config->debug) {
		if (data == 0)	car_error++;
		batt_dbg("CAR is %d, %d\n",data, car_error);
	}

	if((ret >= 0) && (data > 0))
	{
		// for big current charge
		// 10%
		if (ABS(batt_info->fRCPrev, data) < (10 * config_data->design_capacity / num_100))
		{
			batt_info->fRC = data;
			error_times = 0;
		}
		else
		{
			error_times++;
			if(error_times > 3)
			{
				batt_dbg("CAR error_times is %d\n",error_times);
				error_times = 0;
				afe_write_car(batt_info->sCaMAH);
				batt_info->fRCPrev = batt_info->sCaMAH;
				batt_info->fRC 	= batt_info->sCaMAH;
			}
		}
	}

	batt_dbg("first sCaMAH:%d,fRC:%d,fRCPrev:%d,fcurr:%d,volt:%d\n",
			batt_info->sCaMAH,batt_info->fRC,batt_info->fRCPrev,
			batt_info->fCurr,(batt_info->fVolt * oz8806_cell_num));

	//back sCaMAH
	//H7====LOW VOLTAGE CHARGE PREVENT======
	if ((batt_info->fVolt < config_data->discharge_end_voltage) && (batt_info->fCurr > 0)) //charge and voltage < 3300
	{
		batt_dbg("Low voltage %d charge current %d detected\n",batt_info->fVolt,batt_info->fCurr);
		if (batt_info->fRC > batt_info->fRCPrev)	//CAR increased
		{
			batt_dbg("Low voltage fRC limit triggered %d\n",batt_info->fRCPrev);
			batt_info->fRC = batt_info->fRCPrev;	//Limit CAR as previous	
			afe_write_car(batt_info->fRC);			//write back CAR
		}
	}
	//H7====LOW VOLTAGE CHARGE PREVENT======

	gas_gauge->sCtMAH += batt_info->fRC - batt_info->fRCPrev;
	gas_gauge->discharge_sCtMAH += batt_info->fRCPrev - batt_info->fRC;

	if(gas_gauge->sCtMAH < num_0)  gas_gauge->sCtMAH = num_0;
	if(gas_gauge->discharge_sCtMAH < num_0)  gas_gauge->discharge_sCtMAH = num_0;

	//bmu_call();

	adapter_status = get_adapter_status();
	if ((batt_info->fCurr <= gas_gauge->discharge_current_th) ||
		(adapter_status == O2_CHARGER_BATTERY))
		discharge_process();
	else
		charge_process();

	batt_dbg("second sCaMAH:%d,fRC:%d,fRCPrev:%d,fcurr:%d,volt:%d\n",
			batt_info->sCaMAH,batt_info->fRC,batt_info->fRCPrev,
			batt_info->fCurr,(batt_info->fVolt * oz8806_cell_num));

	if(gas_gauge->charge_end)
	{
		if(!charge_end_flag)
		{
			batt_dbg("enter BMT fgu charge end\n");
			charge_end_flag = num_1;
			charge_end_process();
		}
	}
	else
	{
		charge_end_flag = num_0;
	}

	if(gas_gauge->discharge_end)
	{
		if(!discharge_end_flag)
		{
			discharge_end_flag = num_1;
			discharge_end_process();
		}
	}
	else
	{
		discharge_end_flag = num_0;
	}

	oz8806_over_flow_prevent();

	/*
	//very dangerous
	if(batt_info->fVolt <= (config_data->discharge_end_voltage - num_100))
	{
		ret = afe_read_cell_volt(&batt_info->fVolt);
		if((ret >=0) && (batt_info->fVolt > 2500))
			discharge_end_process();
	}
	*/
	//gas_gauge->bmu_tick++;
	previous_loop_timex = time_sec;
	time_sec = oz8806_get_system_boot_time();
		batt_dbg("----------------------------------------------------\n"
		"[bmt]:VERSION: %s-%x\n"
		"[bmt]:TABLEVERSION: %s\n"
		"[bmt]:battery_ok: %d,time_x: %d\n"
		"[bmt]:chg_fcc_update: %d, disg_fcc_update: %d\n"
		"[bmt]:fVolt: %d   fCurr: %d   fCellTemp: %d   fRSOC: %d\n"
		"[bmt]:sCaMAH: %d, sCtMAH1: %d, sCtMAH2: %d\n"
		"[bmt]:fcc: %d, fRC: %d\n"
		"[bmt]:i2c_error_times: %d\n"
		"[bmt]:charge_end: %d, discharge_end: %d\n"
		"[bmt]:adapter_status: %d\n"
		"[bmt]:----------------------------------------------------\n",
		VERSION,calculate_version,get_table_version(),batt_info->Battery_ok,time_sec,
		gas_gauge->charge_fcc_update,gas_gauge->discharge_fcc_update,
		(batt_info->fVolt * oz8806_cell_num),batt_info->fCurr,batt_info->fCellTemp,batt_info->fRSOC,
		batt_info->sCaMAH,gas_gauge->sCtMAH,gas_gauge->discharge_sCtMAH,
		gas_gauge->fcc_data,batt_info->fRC,
		batt_info->i2c_error_times,
		gas_gauge->charge_end,gas_gauge->discharge_end,
		adapter_status);

	if(batt_info->sCaMAH > (full_charge_data))
	{
		batt_dbg("big error sCaMAH is %d\n",batt_info->sCaMAH);
		batt_info->sCaMAH = full_charge_data;	
	}

	if(batt_info->sCaMAH < (gas_gauge->fcc_data / 100 - 1))
	{
		//batt_dbg("big error sCaMAH is %d\n",batt_info->sCaMAH);
		batt_info->sCaMAH = gas_gauge->fcc_data / 100 - 1;	
	}

	/*
	if(gas_gauge->fcc_data > (config_data->design_capacity *FCC_UPPER_LIMIT / 100))
	{	
		gas_gauge->fcc_data=  config_data->design_capacity  * FCC_UPPER_LIMIT / 100 ;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
	
		batt_dbg("fcc error is %d\n",gas_gauge->fcc_data);

	}
	if((gas_gauge->fcc_data <= 0) || (gas_gauge->fcc_data  < (config_data->design_capacity * FCC_LOWER_LIMIT / 100)))
	{	
		gas_gauge->fcc_data = config_data->design_capacity * FCC_LOWER_LIMIT /100;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
	
		batt_dbg("fcc error is %d\n",gas_gauge->fcc_data);

	}
	*/

	gas_gauge->fcc_data=  config_data->design_capacity;
	
	if (bmu_sys_rw_kernel)
	{
		data = bmu_read_data(BATT_CAPACITY);
		//batt_dbg("read from RAM batt_info->sCaMAH is %d\n",data);
		if(data >= num_0)
		{
			if(fRSOC_PRE != batt_info->fRSOC)
			{
				fRSOC_PRE = batt_info->fRSOC;
				bmu_write_data(BATT_CAPACITY,batt_info->sCaMAH);
			//	batt_dbg("o2 back batt_info->sCaMAH num_1 is %d\n",batt_info->sCaMAH);
				//return;
				goto end;
			}

			if(((batt_info->sCaMAH - data)> (gas_gauge->fcc_data/200))||((data - batt_info->sCaMAH)> (gas_gauge->fcc_data/200))){
				bmu_write_data(BATT_CAPACITY,batt_info->sCaMAH);
			//	batt_dbg("o2 back batt_info->sCaMAH 2 is %d\n",batt_info->sCaMAH);
			}
		}
		else bmu_write_data(BATT_CAPACITY,batt_info->sCaMAH);
	}
	else
	{
		if(fRSOC_PRE != batt_info->fRSOC)
		{
			fRSOC_PRE = batt_info->fRSOC;
			//batt_dbg("o2 back batt_info->sCaMAH num_1 is %d\n",batt_info->sCaMAH);
			goto end;
		}
	}

	//wake up OZ8806 into FullPower mode
	ret = afe_register_read_byte(OZ8806_OP_CTRL,&i);
	batt_info->Battery_ok = i & BI_STATUS;
	if(ret < 0)
		batt_info->Battery_ok = 0;

	if((i & num_0x40) || (ret < num_0))
	{
		batt_dbg("bmu_polling_loop read 0x09 ret is %d,i is %d \n",ret,i);
		if(oz8806_cell_num > 1)
		{
			afe_register_write_byte(OZ8806_OP_CTRL,num_0x2c);
		}
		else
			afe_register_write_byte(OZ8806_OP_CTRL,num_0x20);
		//batt_dbg("oz8806 wake up function\n");

	}

	ret  = afe_read_board_offset(&data);
	if (ret >= 0)
	{
		if (config_data->board_offset != 0) 
		{
			//if (data != config_data->board_offset)
			if(ABS(data, config_data->board_offset) > 3)
			{
				afe_write_board_offset(config_data->board_offset);
				batt_dbg("BMT fgu board offset error1 is %d \n",data);
			}
		}
		else
		{
			if((data > 10) || (data <= 0))
			{
				batt_dbg("BMT fgu board offset error2 is %d \n",data);
				afe_write_board_offset(7);
			}
		}
	}
	batt_dbg("test data is %d\n",batt_info->sCaMAH);
	/*

	data = bmu_read_data(BATT_FCC);

	if(data != gas_gauge->fcc_data)
	{
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
		batt_dbg("test %d\n",gas_gauge->fcc_data);
	}
	*/

end:
	batt_dbg("-----------------------bmu_polling_loop end--------------------\n");
}

/*****************************************************************************
* Description:
*		 prevent oz8806 over flow 
* Parameters:
*		None
* Return:
*		None
*****************************************************************************/
static void oz8806_over_flow_prevent(void)
{
	int32_t ret;
	int32_t data;

	if((batt_info->fRSOC > num_0) && gas_gauge->discharge_end)
		gas_gauge->discharge_end = num_0;
   
	if((batt_info->fRSOC < num_100) && gas_gauge->charge_end)
		gas_gauge->charge_end = num_0;

	if(batt_info->fRC< num_0)
	{
		if(batt_info->fVolt >= (config_data->charge_cv_voltage - 200))
		{
			batt_info->fRC = gas_gauge->fcc_data -num_1;
		}
		else
		{
			if(batt_info->fRSOC > 0)
				batt_info->fRC = batt_info->fRSOC * gas_gauge->fcc_data / num_100 - num_1;
			else
				batt_info->fRC = batt_info->sCaMAH;
		}

		afe_write_car(batt_info->fRC);
		batt_info->fRCPrev = batt_info->fRC;
	}

	/*
	ret = oz8806_read_word(OZ8806_OP_CAR);

	if(ret >= num_0)
	{
		ret = (int16_t)ret;
		if(ret >= (num_32768 - num_10 * config_data->fRsense))
		{
			if(parameter->config->debug)batt_dbg("yyyy  CAR WILL UP OVER %d\n",ret);
			ret = 32768 - 15 * config_data->fRsense;
			oz8806_write_word(OZ8806_OP_CAR,(int16_t)ret);
			afe_read_car(&batt_info->fRC);
			batt_info->fRCPrev = batt_info->fRC;
		}
		else if(ret <= (num_10 * config_data->fRsense))
		{
			if(parameter->config->debug)batt_dbg("yyyy  CAR WILL DOWN OVER %d\n",ret);
			ret =  num_15 * config_data->fRsense;
			oz8806_write_word(OZ8806_OP_CAR,(int16_t)ret);
			afe_read_car(&batt_info->fRC);
			batt_info->fRCPrev = batt_info->fRC;
		}

	}
	*/

	ret = afe_read_car(&data);
	if(ret<0)
		return;

	if(data < 5)
	{
		afe_write_car(batt_info->sCaMAH);
		batt_info->fRCPrev = batt_info->sCaMAH;
		batt_info->fRC 	= batt_info->sCaMAH;
		batt_dbg("dddd car down is %d\n",data);
	}
	else if(data > (32768 * parameter->config->dbCARLSB * 1000 / config_data->fRsense -5))
	{
		afe_write_car(batt_info->sCaMAH);
		batt_info->fRCPrev = batt_info->sCaMAH;
		batt_info->fRC 	= batt_info->sCaMAH;
		batt_dbg("dddd car up is %d\n",data);
	}

	if(ABS(batt_info->sCaMAH, data) > one_percent_rc)
	{
		if((batt_info->sCaMAH < gas_gauge->overflow_data ) && (batt_info->sCaMAH > 0))
		{
			afe_write_car(batt_info->sCaMAH);
			batt_info->fRCPrev = batt_info->sCaMAH;
			batt_info->fRC 	= batt_info->sCaMAH;
			if(parameter->config->debug){
				batt_dbg("dddd write car batt_info->fRCPrev is %d\n",batt_info->fRCPrev);
				batt_dbg("dddd write car batt_info->fRC is %d\n",batt_info->fRC);
				batt_dbg("dddd write car batt_info->sCaMAH is %d\n",batt_info->sCaMAH);
			}
		}
	}

	if(batt_info->fRC > (full_charge_data))
	{
		batt_dbg("more fRC %d\n",batt_info->fRC);
		batt_info->fRC = full_charge_data;
		batt_info->fRCPrev = batt_info->fRC;
		afe_write_car(batt_info->fRC);
	}
}



/*****************************************************************************
* Description:
*		charge fcc update process
* Parameters:
*		 None
* Return:
*		None
*****************************************************************************/
void charge_end_process(void)
{
	//FCC UPdate

	/*
	if(gas_gauge->charge_fcc_update)
	{
		if(batt_info->fCurr < config_data->charge_end_current)
			gas_gauge->fcc_data = gas_gauge->sCtMAH;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
		
		batt_dbg("charge1 fcc update is %d\n",gas_gauge->fcc_data);

	}

	if(gas_gauge->fcc_data > (config_data->design_capacity *FCC_UPPER_LIMIT / 100))
	{
		gas_gauge->fcc_data=  config_data->design_capacity  * FCC_UPPER_LIMIT / 100 ;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);

		batt_dbg("charge2 fcc update is %d\n",gas_gauge->fcc_data);

	}
	if((gas_gauge->fcc_data <= 0) || (gas_gauge->fcc_data  < (config_data->design_capacity * FCC_LOWER_LIMIT / 100)))
	{	
		gas_gauge->fcc_data = config_data->design_capacity * FCC_LOWER_LIMIT / 100;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
	
		batt_dbg("charge3 fcc update is %d\n",gas_gauge->fcc_data);

	}
	*/

	gas_gauge->fcc_data = config_data->design_capacity;
	batt_info->sCaMAH = full_charge_data;;

	if (bmu_sys_rw_kernel)
		bmu_write_data(BATT_CAPACITY,batt_info->sCaMAH);

	afe_write_car(batt_info->sCaMAH);

	if(gas_gauge->ocv_flag)
		bmu_write_data(OCV_FLAG,0);

	batt_dbg("yyyy  end charge \n");
	batt_info->fRSOC = num_100;
	gas_gauge->charge_end = num_1;
	charge_end_flag = num_1;
	//charger_finish = 0;
	gas_gauge->charge_fcc_update = 1;
	gas_gauge->discharge_fcc_update = 1;
	gas_gauge->discharge_sCtMAH = 0;
	power_on_flag = 0;
}

/*****************************************************************************
* Description:
*		discharge fcc update process
* Parameters:
*		 None
* Return:
*		None
*****************************************************************************/
void discharge_end_process(void)
{
	//int32_t voltage_end = config_data->discharge_end_voltage;

	//FCC UPdate
	/*
	if(gas_gauge->discharge_fcc_update)
	{
		gas_gauge->fcc_data = gas_gauge->discharge_sCtMAH;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
		batt_dbg("discharge1 fcc update is %d\n",gas_gauge->fcc_data);

	}

	if(gas_gauge->fcc_data > (config_data->design_capacity *FCC_UPPER_LIMIT / 100))
	{
		gas_gauge->fcc_data=  config_data->design_capacity  * FCC_UPPER_LIMIT / 100 ;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
		batt_dbg("discharge2 fcc update is %d\n",gas_gauge->fcc_data);
	}
	if((gas_gauge->fcc_data <= 0) || (gas_gauge->fcc_data  < (config_data->design_capacity * FCC_LOWER_LIMIT / 100)))
	{
		gas_gauge->fcc_data = config_data->design_capacity * FCC_LOWER_LIMIT / 100;
		bmu_write_data(BATT_FCC,gas_gauge->fcc_data);
		batt_dbg("discharge3 fcc update is %d\n",gas_gauge->fcc_data);
	}
	*/
	gas_gauge->fcc_data = config_data->design_capacity;
	if(gas_gauge->ocv_flag)
		bmu_write_data(OCV_FLAG,0);

	batt_info->sCaMAH = gas_gauge->fcc_data / 100 - 1;

	if (bmu_sys_rw_kernel)
		bmu_write_data(BATT_CAPACITY,batt_info->sCaMAH);

	afe_write_car(batt_info->sCaMAH);
	batt_info->fRCPrev = batt_info->sCaMAH;


	batt_dbg("yyyy  end discharge \n");
	batt_info->fRSOC = num_0;
	gas_gauge->discharge_end = num_1;
	gas_gauge->discharge_fcc_update = 1;
	gas_gauge->charge_fcc_update = 1;
	gas_gauge->sCtMAH = num_0;

}



/*****************************************************************************
* Description:
*		below is linux file operation
* Parameters:
*		description for each argument, new argument starts at new line
* Return:
*		what does this function returned?
*****************************************************************************/
#if 0
static int bmu_check_file(char * address)
{
        long fd = sys_open(address,O_RDONLY,0644);
    
        if(fd < num_0)
        {
                //batt_dbg(" bmu check file  fail %s\n",address);
				return -num_1;
        } 
        sys_close(fd);
        return (num_0);
}


static int bmu_read_data(char * address)
{
        char value[4];
        int* p = (int *)value;
        long fd = sys_open(address,O_RDONLY,0644);
    
        if(fd < num_0)
		{
			batt_dbg(" bmu read open file fail %s, return %ld\n",address, fd);
			return fd;
		}
        
        sys_read(fd,(char __user *)value,4);
        
        sys_close(fd);
    
        return (*p);
}

static int bmu_write_data(char * address,int data)
{
        char value[4];
        int* p = (int *)value;
        long fd = sys_open(address,O_CREAT | O_RDWR, 0644);
    
        if(fd < num_0)
        {
			batt_dbg("bmu wirte open file fail %s failed, return %ld\n",address, fd);
			return fd;
        }
        *p = data;
        sys_write(fd, (const char __user *)value, 4);
        
        sys_close(fd);
		return num_0;
}

#else
static int bmu_read_data(char * address)
{
    return -1;
#if 0
	struct file *fp = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf_r[5];
	int ret = 0;
	int val = 0;

	//batt_dbg("%s\n", __func__);

	fp = filp_open(address, O_RDONLY, 0644);
	if (IS_ERR(fp))
	{
		batt_dbg(" %s: open file fail %s\n", __func__, address);
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0;
	ret = vfs_read(fp, buf_r, sizeof(buf_r), &pos);
	if (ret < 0)
		batt_dbg(" %s: read file fail %s\n", __func__, address);

	buf_r[4] = '\0';
	batt_dbg("%s: read: %s\n", __func__, buf_r);

	ret = kstrtoint(buf_r, 10, &val);
	if (ret < 0)
		batt_dbg(" %s: fail to get int from %s\n", __func__, address);

	set_fs(fs);

	filp_close(fp, NULL);

	if (ret < 0)
		return ret;

	return val;
#endif 
}

static int bmu_write_data(char * address,int data)
{
    return -1;
#if 0
	struct file *fp = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf[5];
	char buf_r[5];
	int ret = 0;
	int val = 0;

	ret = sprintf(buf, "%d", data);

	//batt_dbg("%s\n", __func__);

	fp = filp_open(address, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp))
	{
		batt_dbg(" %s: open file fail %s\n", __func__, address);
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0;
	ret = vfs_write(fp, buf, sizeof(buf), &pos);
	if (ret < 0)
		batt_dbg(" %s: write file fail %s\n", __func__, address);

	else if (!strcmp(address, BATT_CAPACITY))
		gas_gauge->stored_capacity = data;

	if(config_data->debug)
	{
		pos = 0;
		ret = vfs_read(fp, buf_r, sizeof(buf_r), &pos);
		if (ret < 0)
			batt_dbg(" %s: read file fail %s\n", __func__, address);

		batt_dbg("%s: read: %s\n", __func__, buf_r);

		ret = kstrtoint(buf_r, 10, &val);
		if (ret < 0)
			batt_dbg(" %s: fail to get int from %s\n", __func__, address);
	}

	set_fs(fs);

	filp_close(fp, NULL);

	return ret;
#endif 
}
#if 0
static int bmu_check_file(char * address)
{
	struct file *fp = NULL;

	fp = filp_open(address, O_RDONLY, 0644);
	if (IS_ERR(fp))
	{
		batt_dbg(" %s: open file fail %s\n", __func__, address);
		return -1;
	}
	return 0;
}
#endif
#endif

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

/****************************************************************************
 * Description:
 *		oz8806_write_byte_pec
 * Parameters:
 *		//addr:		oz8806 slave address
 *      	index:		oz8806 operation register 
 *		data: 		write data
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
#if 0
static int32_t oz8806_write_byte_pec(uint8_t addr,uint8_t index, uint8_t data)
{
	int32_t ret;

	uint8_t pec_data = num_0;
	uint16_t temp = num_0;
	pec_data =pec_calculate(pec_data,addr);
	pec_data =pec_calculate(pec_data,index);
	pec_data =pec_calculate(pec_data,data);
	temp = pec_data;
	temp <<= 8;
	temp |= data;
	ret = i2c_smbus_write_word_data(parameter->client, index,temp);
	return ret;
}
#endif

/****************************************************************************
 * Description:
 *		read oz8806 operation register
 * Parameters:
 *      index:		oz8806 operation register 
 *		buf: 		pointer to buffer to store read data
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_read_byte(uint8_t index)
{
	int32_t ret;
	uint8_t pec_data = num_0;
	uint8_t temp;


	if(num_0)
	{
		ret = i2c_smbus_read_word_data(parameter->client, index);
		if(ret < num_0 ) return ret;
		pec_data =pec_calculate(pec_data,0x5e);
		pec_data =pec_calculate(pec_data,index);
		pec_data =pec_calculate(pec_data,0x5f);
		temp = (uint8_t)(ret & 0xff);
		pec_data =pec_calculate(pec_data,temp);
		temp = (uint8_t)(ret >> 8);

		if(pec_data != temp)
		{ 
			batt_dbg("33333333333333333 BMT_fgu_read_byte\n");
			batt_dbg("33333333333333333 pec_data is %d\n",pec_data);
			batt_dbg("33333333333333333 temp is %d\n",temp);
			return -num_1;
		}

		return ret;
	}
	else
	{
		ret = i2c_smbus_read_byte_data(parameter->client, index);
		return ret;
	}
}

/****************************************************************************
 * Description:
 *		write oz8806 operation register
 * Parameters:
 *     	index:		oz8806 operation register 
 *		data: 		write data
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_write_byte(uint8_t index, uint8_t data)
{
	int32_t ret;
	uint8_t pec_data = num_0;
	uint16_t temp = num_0;

	if(oz8806_pec_check)
	{
		pec_data =pec_calculate(pec_data,0x5e);
		pec_data =pec_calculate(pec_data,index);
		pec_data =pec_calculate(pec_data,data);
		temp = pec_data;
		temp <<= 8;
		temp |= data;
		ret = i2c_smbus_write_word_data(parameter->client, index,temp);
		return ret;
	}
	else
	{
		ret = i2c_smbus_write_byte_data(parameter->client, index,data);
		return ret;
	}
}

/****************************************************************************
 * Description:
 *		read oz8806 operation register word
 * Parameters:
 *      index:		oz8806 operation register 
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_read_word(uint8_t index)
{
	int32_t ret = num_0;
	int32_t ret1 = num_0;
	uint8_t temp;
	uint8_t pec_data = num_0;

	if(num_0)
	{
		ret = i2c_smbus_read_word_data(parameter->client, index);
		if(ret < num_0)	return ret;
		pec_data =pec_calculate(pec_data,0x5e);
		pec_data =pec_calculate(pec_data,index);
		pec_data =pec_calculate(pec_data,0x5f);
		temp = (uint8_t)(ret & 0xff);
		pec_data =pec_calculate(pec_data,temp);
		temp = (uint8_t)(ret >> 8);

		if(pec_data != temp)
		{ 
			batt_dbg("index 0x%02x\n",index);
			batt_dbg("0x%02x\n",(uint8_t)(ret & 0xff));
			batt_dbg("33333333333333333 BMT_fgu_read_word num_1\n");
			batt_dbg("33333333333333333 pec_data is 0x%02x\n",pec_data);
			batt_dbg("33333333333333333 temp is 0x%02x\n",temp);

			batt_dbg("33333333333333333 ret is 0x%02x\n",ret);
			return -num_1;
		}

		ret1 = i2c_smbus_read_word_data(parameter->client, (index + num_1));
		if(ret1 < num_0)	return ret1;

		pec_data = num_0;
		pec_data =pec_calculate(pec_data,0x5e);
		pec_data =pec_calculate(pec_data,(index +num_1));
		pec_data =pec_calculate(pec_data,0x5f);
		temp = (uint8_t)(ret1 & 0xff);
		pec_data =pec_calculate(pec_data,temp);
		temp = (uint8_t)(ret1 >> 8);

		if(pec_data != temp)
		{ 
			batt_dbg("33333333333333333 BMT_fgu_read_word 2\n");
			batt_dbg("33333333333333333 pec_data is %d\n",pec_data);
			batt_dbg("33333333333333333 temp is %d\n",temp);
			return -num_1;
		}

		ret1 <<= 8;
		ret1 |= (uint8_t)(0xff & ret);
		return ret1;
	}
	else
	{
		ret = i2c_smbus_read_word_data(parameter->client, index);
		return ret;
	}
}

/****************************************************************************
 * Description:
 *		write oz8806 operation register
 * Parameters:
 *      index:		oz8806 operation register 
 *		data: 		write data
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_write_word( uint8_t index, uint16_t data)
{
	int32_t ret;
	uint8_t pec_data = num_0;
	uint16_t temp = num_0;


	if(oz8806_pec_check)
	{

		pec_data =pec_calculate(pec_data,0x5e);
		pec_data =pec_calculate(pec_data,index);
		pec_data =pec_calculate(pec_data,(uint8_t)data);
		temp = pec_data;
		temp <<= 8;
		temp |=  (uint8_t)data;
		ret  = i2c_smbus_write_word_data(parameter->client, index,temp);

		if(ret < num_0) return ret;
		pec_data = num_0;

		pec_data =pec_calculate(pec_data,0x5e);
		pec_data =pec_calculate(pec_data,(index +num_1));
		pec_data =pec_calculate(pec_data,(uint8_t)(data>>8));
		temp = pec_data;
		temp <<= 8;
		temp |= (uint8_t)(data >> 8);
		ret = i2c_smbus_write_word_data(parameter->client, (index +num_1),temp);

		return ret;
	}
	else
	{
		ret = i2c_smbus_write_word_data(parameter->client, index,data);
		return ret;
	}
}

/****************************************************************************
 * Description:
 *		read oz8806 cell voltage
 * Parameters: 
 *		voltage: 	cell voltage in mV, range from -5120mv to +5120mv
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_cell_voltage_read(int32_t *voltage)
{
	int32_t ret;
	int32_t data;

	ret = oz8806_read_word(OZ8806_OP_CELL_VOLT);

	if (ret < num_0)  return ret;

	data = ret & 0xFFFF;
	data = (int16_t)data / 16; // >> 4 and keep sign bit

	data = data * parameter->config->fVoltLSB;	//fVoltLSB = 250 (2.5 mV)
	data = data / num_100;	//fVoltLSB = 250 (2.5 mV)

	if (data > 5117 || data < 0)
	{
		batt_dbg("%s:data: %d, reg:%x\n", __func__, data, ret);
		return -1;
	}
	batt_info->i2c_error_times = 0;
	*voltage = data;

	return ret;
}

/****************************************************************************
 * Description:
 *		read oz8806 cell OCV(open circuit voltage)
 * Parameters: 
 *		voltage: 	voltage in mV, range from -5120mv to +5120mv with 2.5mV LSB
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_ocv_voltage_read(int32_t *voltage)
{
	int32_t ret;
	int32_t data;

	ret = oz8806_read_word(OZ8806_OP_OCV_LOW);
	if (ret < num_0)  return ret;

	data = ret & 0xFFFF;
	data = (int16_t)data / 16; // >> 4 and keep sign bit

	data = data * parameter->config->fVoltLSB;	//fVoltLSB = 250 (2.5 mV)
	data = data / num_100;	//fVoltLSB = 250 (2.5 mV)

	if (data > 5117 || data < 0)
	{
		batt_dbg("%s:data: %d, reg:%x\n", __func__, data, ret);
		return -1;
	}

	*voltage = data;

	return ret;
}

/****************************************************************************
 * Description:
 *		read oz8806 current
 * Parameters: 
 *		current: in mA, range is +/-64mv / Rsense with 7.8uv LSB / Rsense
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_current_read(int32_t *data)

{
	int32_t ret;
	int32_t temp;

	ret = oz8806_read_word(OZ8806_OP_CURRENT);

	if (ret < num_0)  return ret;

	temp = ret & 0xFFFF;
	temp = (int16_t)temp * parameter->config->dbCurrLSB;	//dbCurrLSB = 391 (3.90625 mA)
	temp = (temp * 10 / parameter->config->fRsense);
	temp /= 4;

	batt_info->i2c_error_times = 0;
	*data = temp;
	return ret;
}

/****************************************************************************
 * Description:
 *		read oz8806 temp
 * Parameters: 
 *		voltage: 	voltage value in mV, range is +/- 5120mV with 2.5mV LSB
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
int32_t oz8806_temp_read(int32_t *voltage)
{
	int32_t ret;
	int32_t data;

	ret = oz8806_read_word(OZ8806_OP_CELL_TEMP);
	if (ret < num_0)  return ret;

	//batt_dbg("1111111111111 regeister is 0x%02x\n",ret);

	data = ret & 0xFFFF;
	data = (int16_t)data / 16; // >> 4 and keep sign bit

	data = data * parameter->config->fVoltLSB;	//fVoltLSB = 250 (2.5 mV)
	data = data / num_100;	//fVoltLSB = 250 (2.5 mV)

	if (data > 5117 || data < 0)
	{
		batt_dbg("%s:data: %d, reg:%x\n", __func__, data, ret);
		return -1;
	}

	*voltage = data;

	//batt_dbg("1111111111111 voltage is %d\n",*voltage);

	return ret;
}

/****************************************************************************
 * Description:
 *		read oz8806 CAR
 * Parameters: 
 *		car: in mAHr, range is +/- 163840 uvHr/ Rsense with 5 uvHr LSB / Rsense
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_car_read(int32_t *car)
{
	int32_t ret;
	int32_t data;


	ret = oz8806_read_word(OZ8806_OP_CAR);
	if (ret < num_0)  return ret;

	data = (int16_t)ret;
	data = data * parameter->config->dbCARLSB;
	data = data * 1000 / parameter->config->fRsense;

	batt_info->i2c_error_times = 0;
	*car = (int16_t)data;
	return ret;
}

/****************************************************************************
 * Description:
 *		write oz8806 CAR
 * Parameters: 
 *		data: in mAHr, range is +/- 163840 uvHr/ Rsense with 5 uvHr LSB / Rsense
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_car_write(int32_t data)
{
	int32_t temp;
	int32_t ret;

	temp = data;
	temp = (temp * parameter->config->fRsense) / parameter->config->dbCARLSB;		//transfer to CAR
	temp /= 1000;
	ret = oz8806_write_word(OZ8806_OP_CAR,(uint16_t)temp);

	return ret;
}

/****************************************************************************
 * Description:
 *		read oz8806 board offset
 * Parameters: 
 *		data: in mA, +/- 8mV / Rsense with 7.8uV / Rsense 
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_board_offset_read(int32_t *data)
{
	int32_t ret;
	int32_t temp;


	ret = oz8806_read_word(OZ8806_OP_BOARD_OFFSET);
	if (ret < num_0)  return ret;


	temp = (int16_t)ret;
	temp = temp & 0x7ff;
	if(temp & 0x0400)// if bit[10] is 1, it's negative
		temp = temp |0xfffffc00;
	temp = temp * 78;
	temp = temp  * 100/ config_data->fRsense;

	*data = (int16_t)temp;
	return ret;
}

/****************************************************************************
 * Description:
 *		write oz8806 board offset
 * Parameters: 
 *		data: the data will be wrriten
 * Return:
 *		see I2C_STATUS_XXX define
 ****************************************************************************/
static int32_t oz8806_board_offset_write(int32_t data)
{
	int32_t ret;
	int32_t over_data;

	if(data > num_0){
		data = data * config_data->fRsense * num_10 / 78;
		data /= 1000;
		if(data > 0x3ff)data = 0x3ff;
		ret = oz8806_write_word(OZ8806_OP_BOARD_OFFSET,(uint16_t)data);
	}
	else if(data < num_0){
		over_data = (-1024 * 78 * 1000) / (config_data->fRsense * num_10);
		if(data < over_data)
			data = over_data;
		data = data * config_data->fRsense * num_10 / 78;
		data /= 1000;
		data = (data & 0x07ff) | 0x0400;
		ret = oz8806_write_word(OZ8806_OP_BOARD_OFFSET,(uint16_t)data);
	}
	else
		ret = oz8806_write_word(OZ8806_OP_BOARD_OFFSET,num_0);

	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function for operation register byte reading 
 * Parameters:
 *		index:	register index to be read
 *		*dat:	buffer to store data read back
 * Return:
 *		I2C_STATUS_OK if read success, else inidcate error
 *****************************************************************************/
static int32_t afe_register_read_byte(uint8_t index, uint8_t *dat)
{
	int32_t ret;
	uint8_t i;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_read_byte(index);
		if(ret >=num_0) break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 

	*dat = (uint8_t)ret;

	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function for operation register byte writing
 * Parameters:
 *		index:	register index to be read
 *		dat:		write data
 * Return:
 *		see I2C_STATUS_XXX define
 *****************************************************************************/
static int32_t afe_register_write_byte(uint8_t index, uint8_t dat)
{
	int32_t ret;
	uint8_t i;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_write_byte(index, dat);
		if(ret >= num_0) break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 

	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function for operation register word reading 
 * Parameters:
 *		index:	register index to be read
 *		*dat:	buffer to store data read back
 * Return:
 *		I2C_STATUS_OK if read success, else inidcate error
 *****************************************************************************/
static int32_t afe_register_read_word(uint8_t index, uint16_t *dat)
{
	int32_t ret;
	uint8_t i;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_read_word(index);
		if(ret >= num_0) break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 

	*dat = (uint16_t)ret;

	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function for operation register word writing
 * Parameters:
 *		index:	register index to be read
 *		dat:		write data
 * Return:
 *		see I2C_STATUS_XXX define
 *****************************************************************************/
#if 0
static int32_t afe_register_write_word(uint8_t index, uint16_t dat)
{ 
	int32_t ret;
	uint8_t i;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_write_word(index, dat);
		if(ret >= num_0) break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 

	return ret;
}
#endif

/*****************************************************************************
 * Description:
 *		wrapper function to read cell voltage 
 * Parameters:
 *		vol:	buffer to store voltage read back
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
int32_t afe_read_cell_volt(int32_t *voltage)
{
	int32_t ret;
	uint8_t i;
	int32_t buf;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_cell_voltage_read(&buf);
		if(ret >= num_0)
				break;
		msleep(5);
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 
	if(oz8806_cell_num > num_1)
	{
		//*voltage = buf * num_1000 / res_divider_ratio;
		batt_dbg("read from register %d mv \n",buf);
		*voltage = buf * res_divider_ratio / num_1000; // res_divider_ratio = ((Rpull + Rdown) / Rdown) * 1000
		*voltage /= oz8806_cell_num;
	}
	else
		*voltage = buf;
	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to read cell ocv  voltage 
 * Parameters:
 *		vol:	buffer to store voltage read back

 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
static int32_t afe_read_ocv_volt(int32_t *voltage)
{
	int32_t ret;
	uint8_t i;
	int32_t buf;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_ocv_voltage_read(&buf);
		if(ret >= num_0)
				break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 

	if(oz8806_cell_num > num_1)
	{
		*voltage = buf * num_1000 / res_divider_ratio;
		*voltage /= oz8806_cell_num;
	}
	else
		*voltage = buf;

	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to read current 
 * Parameters:
 *		dat:	buffer to store current value read back	
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
int32_t afe_read_current(int32_t *dat)
{
	int32_t ret;
	uint8_t i;
	int32_t buf;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_current_read(&buf);
		if(ret >= num_0)
				break;
		msleep(5);
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 
	*dat = buf;
	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to read CAR(accummulated current calculation)
 * Parameters:
 *		dat:	buffer to store current value read back	
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
static int32_t afe_read_car(int32_t *dat)
{
	int32_t ret;
	uint8_t i;
	int32_t buf;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_car_read(&buf);
		if(ret >= num_0)
				break;
		msleep(5);
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 
	*dat = buf;
	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to write car 
 * Parameters:
 *		dat:	buffer to store current value read back	
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
static int32_t afe_write_car(int32_t date)
{
	int32_t ret;
	uint8_t i;


	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_car_write(date);
		if(ret >= num_0)	break;
		msleep(5);
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 
	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to write board offset 
 * Parameters:
 *		dat:	buffer to store current value read back	
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
static int32_t afe_write_board_offset(int32_t date)
{
	int32_t ret;
	uint8_t i;


	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_board_offset_write(date);
		if(ret >= num_0)	break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 
	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to read board offset 
 * Parameters:
 *		dat:	buffer to store current value read back	
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
static int32_t afe_read_board_offset(int32_t *dat)
{
	int32_t ret;
	uint8_t i;
	int32_t buf;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_board_offset_read(&buf);
		if(ret >= num_0)
				break;
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 
	*dat = buf;
	return ret;
}

/*****************************************************************************
 * Description:
 *		wrapper function to read temp
 * Parameters:
 *		dat:	buffer to store current value read back	
 * Return:
 *		see I2C_STATUS_XXX define
 * Note:
 *		it's acceptable if one or more times reading fail
 *****************************************************************************/
static int32_t afe_read_cell_temp(int32_t *data)
{
	int32_t ret;
	uint8_t i;
	int32_t temp;
	int32_t buf;

	if (gas_gauge->ext_temp_measure)
		return num_0;

	for(i = num_0; i < RETRY_CNT; i++){
		ret = oz8806_temp_read(&buf);
		if(ret >= num_0)	break;
		msleep(5);
	}
	if(i >= RETRY_CNT){
		batt_info->i2c_error_times++;
		batt_dbg("yyyy. %s failed, ret %d\n", __func__, ret);
		return ret;
	} 

	if((buf >= config_data->temp_ref_voltage)|| (buf <= num_0)){
		*data = num_25;
	}
	else{
		temp = buf * config_data->temp_pull_up;
		temp = temp / (config_data->temp_ref_voltage - buf);
		*data =	one_latitude_table(parameter->cell_temp_num,parameter->temperature,temp);
	}
	//batt_dbg("1111111111111 r is %d\n",temp);
	return ret;
}

/*****************************************************************************
 * Description:
 *		pec_calculate
 * Parameters:
 *		ucCrc:	source
 *		ucData: data
 * Return:
 *		crc data
 * Note:
 *
 *****************************************************************************/
static uint8_t pec_calculate (uint8_t ucCrc, uint8_t ucData)
{
       uint8_t j;
       for (j = num_0x80; j != num_0; j >>= num_1)
       {
              if((ucCrc & num_0x80) != num_0)
              {
                     ucCrc <<= num_1;
                     ucCrc ^= num_0x07;
              }
              else
                     ucCrc <<= num_1;
              if (ucData & j)
                     ucCrc ^= num_0x07;
       }
       return ucCrc;
}

#if 0
/*****************************************************************************
 * Description:
 *		i2c_read_byte
 * Parameters:
 * Return:
 * Note:
 *
 *****************************************************************************/
static int32_t i2c_read_byte(uint8_t addr,uint8_t index,uint8_t *data)
{
	int32_t ret = 0;
	#if 0
	struct i2c_adapter *adap=parameter->client->adapter;
	struct i2c_msg msgs[num_2];
	int scl_rate= num_100 * num_1000;   
	msgs[num_0].addr = addr;
	msgs[num_0].flags = parameter->client->flags;
	msgs[num_0].len = num_1;
	msgs[num_0].buf = &index;
	msgs[num_0].scl_rate = scl_rate;

	msgs[num_1].addr = addr;
	msgs[num_1].flags = parameter->client->flags | I2C_M_RD;
	msgs[num_1].len = num_1;
	msgs[num_1].buf = (char *)data;
	msgs[num_1].scl_rate = scl_rate;

	ret = i2c_transfer(adap, msgs, num_2);

	#endif
	return (ret == num_2)? num_0 : ret;
}

/*****************************************************************************
 * Description:
 *		i2c_write_byte
 * Parameters:
 * Return:
 * Note:
 *
 *****************************************************************************/
static int32_t i2c_write_byte(uint8_t addr,uint8_t index,uint8_t data)
{
	int32_t ret = 0;      
   #if 0
	struct i2c_adapter *adap=parameter->client->adapter;
	struct i2c_msg msg;
	int32_t scl_rate = num_100 * num_1000;

	char tx_buf[num_3];
	tx_buf[num_0] = index;
	tx_buf[num_1] = data;
        
	msg.addr = addr;
	msg.flags = parameter->client->flags;
	msg.len = num_2;
	msg.buf = (char *)tx_buf;
	msg.scl_rate = scl_rate;

	if(oz8806_pec_check)
	{
		msg.len = num_3;
		tx_buf[num_2] = num_0;
		tx_buf[num_2] =pec_calculate(tx_buf[num_2],(addr<<num_1));
		tx_buf[num_2] =pec_calculate(tx_buf[num_2],index);
		tx_buf[num_2] =pec_calculate(tx_buf[num_2],(uint8_t)data);
	}

	ret = i2c_transfer(adap, &msg, num_1);
	#endif
	return (ret == num_1) ? num_0 : ret;

}
#endif

void bmu_reinit(int32_t mode)
{
	if(mode)
	{
		//msleep(5000);
		times = 0;
		retry_times = 0;
		calculate_times = 0;
		wait_ocv_flag = 0;
		oz8806_reset_wkuptime();
		oz8806_wakeup_full_power();
		bmu_init_parameter(&parameter_customer);
		bmu_init_parameter_more(&parameter_customer);
		bmu_init_chip(&parameter_customer);
		oz8806_set_batt_info_ptr(batt_info);
		oz8806_set_gas_gauge(gas_gauge);
		wait_ocv_flag_fun();
		bmu_wait_ready();
		oz8806_register_bmu_callback(	&bmu_polling_loop,   &bmu_wake_up_chip,      &bmu_power_down_chip, 
												&charge_end_process, &discharge_end_process, &oz8806_temp_read,  
												&afe_read_current,   &afe_read_cell_volt);
	}
	else
	{
		check_oz8806_staus();
		msleep(5000);
		afe_write_car(batt_info->sCaMAH);
		batt_info->fRCPrev = batt_info->sCaMAH;
		batt_info->fRC 	 = batt_info->sCaMAH;
	}
}
EXPORT_SYMBOL(bmu_reinit);

int bmulib_init(void)
{
	int ret = 0;
#if !defined(OZ8806_API)
	uint32_t byte_num = 0;
	uint8_t * p = NULL;
#endif

	bmt_dbg("%s\n", __func__);

	//NOTICE: Don't change the sequence of calling below functions

	byte_num = sizeof(config_data_t) + sizeof(bmu_data_t) +  sizeof(gas_gauge_t);
	p = kzalloc(byte_num, GFP_KERNEL);
	bmu_kernel_memaddr = (unsigned long)p;
	
	if (!bmu_kernel_memaddr)
		goto fail_mem;

	parameter_customer.client = oz8806_get_client();
	if (!parameter_customer.client)
		goto fail_i2c;
	bmt_dbg("i2c address:%x\n", parameter_customer.client->addr);
	//struct oz8806_data *data = i2c_get_clientdata(parameter_customer.client);

	bmu_init_parameter(&parameter_customer);

	bmu_init_parameter_more(&parameter_customer);

	bmu_init_chip(&parameter_customer);

	oz8806_set_batt_info_ptr(batt_info);

	oz8806_set_gas_gauge(gas_gauge);

	oz8806_register_bmu_callback(&bmu_polling_loop, &bmu_wake_up_chip, &bmu_power_down_chip, 
			                     &charge_end_process, &discharge_end_process,
								 &oz8806_temp_read, &afe_read_current, &afe_read_cell_volt);

	return ret;
fail_mem:
	bmt_dbg("%s: bmu_kernel_memaddr error\n", __func__);

fail_i2c:
	bmt_dbg("%s: oz8806 i2c register error\n", __func__);
	return -1;
}

void bmulib_exit(void)
{
	batt_dbg("%s\n", __func__);
	unregister_bmu_callback();

#if !defined(OZ8806_API)
	if (bmu_kernel_memaddr)
		kfree((int8_t *)bmu_kernel_memaddr);
	bmu_kernel_memaddr = 0;
#endif
}

