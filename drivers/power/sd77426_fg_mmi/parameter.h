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

#ifndef _PARAMETER_H_
#define _PARAMETER_H_

/****************************************************************************
* #include section
*  add #include here if any
***************************************************************************/
#include "table.h"
#include <linux/power_supply.h>
#include <linux/suspend.h>

/****************************************************************************
* #define section
*  add constant #define here if any
***************************************************************************/
#define MYDRIVER	"oz8806"  //"oz8806"

#define DISCH_CURRENT_TH    -10
#define O2_OCV_100_VOLTAGE  8700
#define TEMPERATURE_DATA_NUM 28

#define num_0      	0
#define num_1		1
#define num_32768  	32768
#define num_5		5
#define num_0x2c	0x2c
#define num_0x40    0x40
#define num_0x20    0x20
#define num_1000    1000
#define num_50      50
#define num_100     100
#define num_10      10
#define num_7       7
#define num_2       2
#define num_1000    1000
#define num_0x2f    0x2f
#define num_0x28    0x28
#define num_0x07	0x07
#define num_6		6
#define num_10		10
#define num_20		20
#define num_9		9
#define num_95		95
#define num_0x4c	0x4c
#define num_0x40	0x40
#define num_3		3
#define num_99		99
#define num_15		15
#define num_25		25
#define num_0x80	0x80

//#define PEC_CHECK

#define INIT_CAP (-2) 
#define NO_FILE (-1)  

/****************************************************************************
* Struct section
*  add struct #define here if any
***************************************************************************/

enum charger_type_t {
	O2_CHARGER_UNKNOWN = -1,
	O2_CHARGER_BATTERY = 0,
	O2_CHARGER_USB = 1,
	O2_CHARGER_AC = 4,
};

struct batt_data {
	int32_t	batt_soc;//Relative State Of Charged, present percentage of battery capacity
	int32_t	batt_voltage;//Voltage of battery, in mV
	int32_t	batt_current;//Current of battery, in mA; plus value means charging, minus value means discharging
	int32_t	batt_temp;//Temperature of battery
	int32_t	batt_capacity;//adjusted residual capacity
	int32_t	batt_fcc_data;
	int32_t	discharge_current_th;
 	uint8_t	charge_end;
};

struct oz8806_data 
{
	struct power_supply *bat;
	struct power_supply_desc bat_desc;
	struct power_supply *ac_psy;
	struct power_supply *usb_psy;
	struct delayed_work work;
	unsigned int interval;
	struct i2c_client	*myclient;
	struct notifier_block pm_nb;		//alternative suspend/resume method
	struct batt_data batt_info;
	int status;
};

typedef struct	 tag_config_data {
	int32_t	fRsense;		//= 20 * 1000;			//Rsense value of chip, in mini ohm expand 1000 times
	int32_t 	temp_pull_up;  //230000;
	int32_t	temp_ref_voltage; //1800;1.8v
	int32_t	dbCARLSB;		//= 5.0;		//LSB of CAR, comes from spec
	int32_t	dbCurrLSB;		//781 (7.8 *100 uV);	//LSB of Current, comes from spec
	int32_t	fVoltLSB;		//250 (2.5*100 mV);	//LSB of Voltage, comes from spec
	int32_t	design_capacity;	//= 7000;		//design capacity of the battery
 	int32_t	charge_cv_voltage;	//= 4200;		//CV Voltage at fully charged
	int32_t	charge_end_current;	//= 100;		//the current threshold of End of Charged
	int32_t	discharge_end_voltage;	//= 3550;		//mV
	int32_t 	board_offset;			//0; 				//mA, not more than caculate data
	uint8_t	debug;                                          // enable or disable O2MICRO debug information

}config_data_t;

typedef struct	 tag_parameter_data {
	int32_t            	 	ocv_data_num;
	int32_t              	cell_temp_num;
	one_latitude_data_t  	*ocv;
	one_latitude_data_t	 	*temperature;
	config_data_t 		 	*config;
	struct i2c_client 	 	*client;
	uint8_t	charge_pursue_step;
 	uint8_t	discharge_pursue_step;
	uint8_t	discharge_pursue_th;
	uint8_t	wait_method;
	char * BATT_CAPACITY ;
	char * BATT_FCC;
	char * OCV_FLAG;
	char * BATT_OFFSET;
	uint8_t oz8806_cell_num;
	int32_t res_divider_ratio;
	uint8_t set_soc_from_ext;
	int32_t soc_external;
	uint8_t file_not_ok_cap;
	uint8_t fix_car_init;
	uint8_t bmu_sys_rw_kernel;
	int8_t power_on_retry_times;
}parameter_data_t;

typedef struct tag_bmu {
	int32_t	Battery_ok;
	int32_t	fRC;			//= 0;		//Remaining Capacity, indicates how many mAhr in battery
	int32_t	fRSOC;			//50 = 50%;	//Relative State Of Charged, present percentage of battery capacity
	int32_t	fVolt;			//= 0;						//Voltage of battery, in mV
	int32_t	fCurr;			//= 0;		//Current of battery, in mA; plus value means charging, minus value means discharging
	int32_t	fPrevCurr;		//= 0;						//last one current reading
	int32_t	fOCVVolt;		//= 0;						//Open Circuit Voltage
	int32_t	fCellTemp;		//= 0;						//Temperature of battery
	int32_t	fRCPrev;
	int32_t	sCaMAH;			//= 0;						//adjusted residual capacity
	int32_t	i2c_error_times;
}bmu_data_t;

typedef struct tag_gas_gauge {
	int32_t  overflow_data; //unit: mAh, maximum capacity that the IC can measure
	uint8_t  discharge_end;
 	uint8_t  charge_end;
	uint8_t  charge_fcc_update;
 	uint8_t  discharge_fcc_update;
	int32_t  sCtMAH ;    //be carefull, this must be int32_t
	int32_t  fcc_data;
	int32_t  discharge_sCtMAH ;//be carefull, this must be int32_t
	//uint8_t  charge_wait_times;
	//uint8_t  discharge_wait_times;
	//uint8_t  charge_count;
	//uint8_t  discharge_count;
	//uint32_t bmu_tick;
	//uint32_t charge_tick;
	//uint8_t charge_table_num;
	uint8_t rc_x_num;
	uint8_t rc_y_num;
	uint8_t rc_z_num;
	uint8_t  charge_strategy;
	int32_t  charge_sCaUAH;
	int32_t  charge_ratio;  //this must be static 
 	uint8_t  charge_table_flag;
	int32_t  charge_end_current_th2;
	int32_t charge_max_ratio;
	uint8_t discharge_strategy;
	int32_t discharge_sCaUAH;
	int32_t discharge_ratio;  //this must be static 
	uint8_t discharge_table_flag;
	int32_t	discharge_current_th;
	uint32_t discharge_max_ratio;
	int32_t dsg_end_voltage_hi;
	int32_t dsg_end_voltage_th1;
	int32_t dsg_end_voltage_th2;
	uint8_t dsg_count_2;
	uint8_t ocv_flag;
	uint8_t  vbus_ok;
	uint8_t charge_full;
	int32_t ri;
	int32_t batt_ri;
	int32_t line_impedance;
	int32_t max_chg_reserve_percentage;
	int32_t fix_chg_reserve_percentage;
	uint8_t fast_charge_step;
	int32_t start_fast_charge_ratio;
	uint8_t charge_method_select;
	int32_t max_charge_current_fix;
	uint8_t ext_temp_measure;
	uint8_t bmu_init_ok;
	int32_t stored_capacity;
	uint8_t lower_capacity_reserve;
	uint8_t lower_capacity_soc_start;
	uint8_t percent_10_reserve;
	uint32_t power_on_100_vol;
}gas_gauge_t;

/****************************************************************************
* debug func *
***************************************************************************/
#define bmt_dbg(fmt, args...) printk(KERN_ERR"[bmt]:"pr_fmt(fmt)"", ## args)

#define batt_dbg(fmt, args...)\
do {\
	if(parameter->config->debug)\
	    printk(KERN_ERR"[bmt]:"pr_fmt(fmt)"", ## args);\
} while(0)

/****************************************************************************
* extern variable/function declaration of table.c
***************************************************************************/

extern one_latitude_data_t ocv_data[OCV_DATA_NUM];
extern int	XAxisElement[XAxis];
extern int  YAxisElement[YAxis];
extern int	ZAxisElement[ZAxis];
extern int	RCtable[YAxis*ZAxis][XAxis];
//extern one_latitude_data_t	charge_data[CHARGE_DATA_NUM];

extern uint8_t battery_ri;
extern int32_t one_percent_rc;

/****************************************************************************
* extern variable/function defined by parameter.c
***************************************************************************/
extern config_data_t config_data;
extern uint8_t check_chip_function;
//******************************************************************************

extern void bmu_init_parameter(parameter_data_t *parameter_customer);
extern void bmu_init_gg(gas_gauge_t *gas_gauge);
extern void bmu_reinit(int32_t mode); //mode = 0,wakeup ic,rewrite car; mode = 1, reinit from ocv
#ifndef CONFIG_OZ8806_M
extern int bmulib_init(void);
extern void bmulib_exit(void);
#endif

extern void	bmu_polling_loop(void);
extern void	bmu_wake_up_chip(void);
extern void	bmu_power_down_chip(void);
extern void	charge_end_process(void);
extern void	discharge_end_process(void);
extern int32_t	oz8806_temp_read(int32_t *voltage);
extern int32_t	afe_read_current(int32_t *dat);
extern int32_t	afe_read_cell_volt(int32_t *voltage);

extern void bmu_init_table(int **x, int **y, int **z, int **rc);

extern const char * get_table_version(void);

/****************************************************************************
* extern variable/function defined by oz8806_battery.c
***************************************************************************/
extern int32_t oz8806_vbus_voltage(void);
extern int32_t oz8806_get_simulated_temp(void);
extern int32_t oz8806_get_init_status(void); //if return 1, battery capacity is initiated successfully
extern void oz8806_battery_update_data(void);
extern int oz8806_get_soc(void);
extern int oz8806_get_remaincap(void);
extern int oz8806_get_battry_current(void);
extern int oz8806_get_battery_voltage(void);
extern int oz8806_get_battery_temp(void);

//export symbol for bmulib
extern struct i2c_client * oz8806_get_client(void);
extern int8_t get_adapter_status(void);

extern void oz8806_register_bmu_callback(void *bmu_polling_loop_func,
		void *bmu_wake_up_chip_func,
		void *bmu_power_down_chip_func,
		void *charge_end_process_func,
		void *discharge_end_process_func,
		void *oz8806_temp_read_func,
		void *oz8806_current_read_func,
		void *oz8806_voltage_read_func);

extern void unregister_bmu_callback(void);
extern void oz8806_set_batt_info_ptr(bmu_data_t  *batt_info);
extern void oz8806_set_gas_gauge(gas_gauge_t *gas_gauge);
extern int oz8806_get_save_capacity(void);
extern int oz8806_get_soc_from_ext(void);
extern unsigned long oz8806_get_system_boot_time(void);
extern unsigned long oz8806_get_power_on_time(void);
extern unsigned long oz8806_get_boot_up_time(void);
extern int oz8806_wakeup_full_power(void);
extern void oz8806_reset_wkuptime(void);
/****************************************************************************
* extern variable/function defined by oz8806_api_dev.c.
* This file is deprecated in Linux system.
***************************************************************************/

extern int is_battery_exchanged(void);
extern int32_t calculate_soc_result(void);
extern int32_t one_latitude_table(int32_t number,one_latitude_data_t *data,int32_t value);
extern  uint8_t OZ8806_LookUpRCTable(int infVolt,int infCurr, int infTemp, int *infCal);
void charge_process(void);
void discharge_process(void);

extern bmu_data_t	   *batt_info;
extern gas_gauge_t    *gas_gauge;
extern parameter_data_t parameter_customer;
extern parameter_data_t *parameter;

extern int *rc_table;
extern int *xaxis_table;
extern int *yaxis_table;
extern int *zaxis_table;

extern int32_t calculate_mah;
extern int32_t  calculate_soc ;
extern uint32_t calculate_version;

#define X_AXIS gas_gauge->rc_x_num 
#define Y_AXIS gas_gauge->rc_y_num 
#define Z_AXIS gas_gauge->rc_z_num 
#endif //end _PARAMETER_H_

