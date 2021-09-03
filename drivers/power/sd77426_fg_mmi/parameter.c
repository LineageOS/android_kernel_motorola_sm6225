/*****************************************************************************
* Copyright(c) BMT, 2021. All rights reserved
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include "parameter.h"
#include "table.h"
#include "battery_config.h"

one_latitude_data_t			cell_temp_data[TEMPERATURE_DATA_NUM] = {
			{3380,   115}, {3590,   113}, {4594,   105},
			{5383,   100}, {6336,   95}, {7481,   90},
			{8872,   85}, {10565,   80}, {12635,   75},
			{15184,   70}, {18322,   65}, {22224,   60},
			{27090,   55}, {33194,   50}, {40903,   45},
			{50676,   40}, {63167,   35}, {79221,   30},
			{100000,  25}, {127080,  20}, {162650,  15},
			{209709,  10}, {272499,   5},	{357011,   0},
			{471632,  -5}, {628988, -10}, {846578, -15},
			{1151036, -20},
};


config_data_t config_data = {
       	.fRsense = 			O2_CONFIG_RSENSE,
		.temp_pull_up 		= O2_TEMP_PULL_UP_R,
		.temp_ref_voltage = O2_TEMP_REF_VOLTAGE,
		.dbCARLSB =			5,
		.dbCurrLSB =    	781	,
		.fVoltLSB =	  		250,

		.design_capacity =			O2_CONFIG_CAPACITY,
		.charge_cv_voltage = 		OZ8806_VOLTAGE,
		.charge_end_current =		O2_CONFIG_EOC,
		.discharge_end_voltage =	OZ8806_EOD,
		.board_offset = 			O2_CONFIG_BOARD_OFFSET,
		.debug = 1,
};

/*****************************************************************************
 * bmu_init_parameter:
 *		this implements a interface for customer initiating bmu parameters
 * Return: NULL
 *****************************************************************************/
void bmu_init_parameter(parameter_data_t *parameter_customer)
{
	check_chip_function  = 0;
	parameter_customer->config = &config_data;
	parameter_customer->ocv = ocv_data;
	parameter_customer->temperature = cell_temp_data;
	parameter_customer->ocv_data_num = OCV_DATA_NUM;
	parameter_customer->cell_temp_num = TEMPERATURE_DATA_NUM;
	parameter_customer->charge_pursue_step = 10;
 	parameter_customer->discharge_pursue_step = 6;
	parameter_customer->discharge_pursue_th = 10;
	parameter_customer->wait_method = 2;

	//parameter_customer->BATT_CAPACITY 	= "/data/sCaMAH.dat";
	//parameter_customer->BATT_FCC 		= "/data/fcc.dat";
	//parameter_customer->OCV_FLAG 	= "/data/ocv_flag.dat";
	//parameter_customer->BATT_OFFSET 	= "/data/offset.dat";

	parameter_customer->fix_car_init = 0;
	parameter_customer->power_on_retry_times = 0;

	//if read/write file /data/sCaMAH.dat in userspace
	//please enable "bmu_sys_rw_kernel = 0"
#ifdef USERSPACE_READ_SAVED_CAP
	parameter_customer->bmu_sys_rw_kernel = 0;
#else
	parameter_customer->bmu_sys_rw_kernel = 1;
#endif
}
EXPORT_SYMBOL(bmu_init_parameter);

/*****************************************************************************
 * Description:
 *		bmu_init_gg
 * Return: NULL
 *****************************************************************************/
void bmu_init_gg(gas_gauge_t *gas_gauge)
{
	//gas_gauge->charge_table_num = CHARGE_DATA_NUM;
	gas_gauge->rc_x_num = XAxis;
	gas_gauge->rc_y_num = YAxis;
	gas_gauge->rc_z_num = ZAxis;

	gas_gauge->discharge_current_th = DISCH_CURRENT_TH;
	gas_gauge->fcc_data = config_data.design_capacity;

	bmt_dbg("AAAA battery_id is %s\n", battery_id[0]);

	gas_gauge->ri = 18;
	gas_gauge->line_impedance = 10;
	gas_gauge->power_on_100_vol = O2_OCV_100_VOLTAGE;

#if 0
	//add this for oinom
	gas_gauge->lower_capacity_reserve = 5;
	gas_gauge->lower_capacity_soc_start =30;
	//gas_gauge->percent_10_reserve = 66;
#endif
}
EXPORT_SYMBOL(bmu_init_gg);

void bmu_init_table(int **x, int **y, int **z, int **rc)
{
	*x = (int *)((uint8_t *)XAxisElement);
	*y = (int *)((uint8_t *)YAxisElement);
	*z = (int *)((uint8_t *)ZAxisElement);
	*rc = (int *)((uint8_t *)RCtable);
}
EXPORT_SYMBOL(bmu_init_table);

const char * get_table_version(void)
{
	return table_version;
}
EXPORT_SYMBOL(get_table_version);

