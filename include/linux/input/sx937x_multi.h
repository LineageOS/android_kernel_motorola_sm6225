/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef __SX937x_H__
#define __SX937x_H__

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>


//Interrupt Control
#define SX937X_IRQ_SOURCE	0x4000
#define SX937X_IRQ_MASK_A	0x4004
#define SX937X_IRQ_MASK_B	0x800C
#define SX937X_IRQ_SETUP	0x4008

//Device State Configuration
#define SX937X_DEVICE_RESET	0x4240
#define SX937X_COMMAND	0x4280
#define SX937X_CMMAND_BUSY	0x4284

//Pin Customization
#define SX937X_PIN_SETUP_A	0x42C0
#define SX937X_PIN_SETUP_B	0x42C4

//Device Information
#define SX937X_DEVICE_INFO	0x42CC

//Status Registers
#define SX937X_DEVICE_STATUS_A	0x8000
#define SX937X_DEVICE_STATUS_B	0x8004
#define SX937X_DEVICE_STATUS_C	0x8008

//Status Output Setup
#define SX937X_STATUS_OUTPUT_0	0x8010
#define SX937X_STATUS_OUTPUT_1	0x8014
#define SX937X_STATUS_OUTPUT_2	0x8018
#define SX937X_STATUS_OUTPUT_3	0x801C

//Phase Enabling
#define SX937X_SCAN_PERIOD_SETUP	0x8020
#define SX937X_GENERAL_SETUP	0x8024

//AFE Parameter Setup
#define SX937X_AFE_PARAMETERS_PH0	0x8028
#define SX937X_AFE_PARAMETERS_PH1	0x8034
#define SX937X_AFE_PARAMETERS_PH2	0x8040
#define SX937X_AFE_PARAMETERS_PH3	0x804C
#define SX937X_AFE_PARAMETERS_PH4	0x8058
#define SX937X_AFE_PARAMETERS_PH5	0x8064
#define SX937X_AFE_PARAMETERS_PH6	0x8070
#define SX937X_AFE_PARAMETERS_PH7	0x807C

//AFE Connection Setup
#define SX937X_AFE_CS_USAGE_PH0	0x8030
#define SX937X_AFE_CS_USAGE_PH1	0x803C
#define SX937X_AFE_CS_USAGE_PH2	0x8048
#define SX937X_AFE_CS_USAGE_PH3	0x8054
#define SX937X_AFE_CS_USAGE_PH4	0x8060
#define SX937X_AFE_CS_USAGE_PH5	0x806C
#define SX937X_AFE_CS_USAGE_PH6	0x8078
#define SX937X_AFE_CS_USAGE_PH7	0x8084

//ADC, RAW Filter and Debounce Filter
#define SX937X_FILTER_SETUP_A_PH0	0x8088
#define SX937X_FILTER_SETUP_A_PH1	0x80A8
#define SX937X_FILTER_SETUP_A_PH2	0x80C8
#define SX937X_FILTER_SETUP_A_PH3	0x80E8
#define SX937X_FILTER_SETUP_A_PH4	0x8108
#define SX937X_FILTER_SETUP_A_PH5	0x8128
#define SX937X_FILTER_SETUP_A_PH6	0x8148
#define SX937X_FILTER_SETUP_A_PH7	0x8168

//Average and UseFilter Filter
#define SX937X_FILTER_SETUP_B_PH0	0x808C
#define SX937X_FILTER_SETUP_B_PH1	0x80AC
#define SX937X_FILTER_SETUP_B_PH2	0x80CC
#define SX937X_FILTER_SETUP_B_PH3	0x80EC
#define SX937X_FILTER_SETUP_B_PH4	0x810C
#define SX937X_FILTER_SETUP_B_PH5	0x812C
#define SX937X_FILTER_SETUP_B_PH6	0x814C
#define SX937X_FILTER_SETUP_B_PH7	0x816C

//Filter Setup C
#define SX937X_USE_FLT_SETUP_PH0	0x8090
#define SX937X_USE_FLT_SETUP_PH1	0x80B0
#define SX937X_USE_FLT_SETUP_PH2	0x80D0
#define SX937X_USE_FLT_SETUP_PH3	0x80F0
#define SX937X_USE_FLT_SETUP_PH4	0x8110
#define SX937X_USE_FLT_SETUP_PH5	0x8130
#define SX937X_USE_FLT_SETUP_PH6	0x8150
#define SX937X_USE_FLT_SETUP_PH7	0x8170

//Filter Setup D
#define SX937X_ADC_QUICK_FILTER_0	0x81E0
#define SX937X_ADC_QUICK_FILTER_1	0x81E4
#define SX937X_ADC_QUICK_FILTER_2	0x81E8
#define SX937X_ADC_QUICK_FILTER_3	0x81EC

//Steady and Saturation Setup
#define SX937X_STEADY_AND_SATURATION_PH0	0x809C
#define SX937X_STEADY_AND_SATURATION_PH1	0x80BC
#define SX937X_STEADY_AND_SATURATION_PH2	0x80DC
#define SX937X_STEADY_AND_SATURATION_PH3	0x80FC
#define SX937X_STEADY_AND_SATURATION_PH4	0x811C
#define SX937X_STEADY_AND_SATURATION_PH5	0x813C
#define SX937X_STEADY_AND_SATURATION_PH6	0x815C
#define SX937X_STEADY_AND_SATURATION_PH7	0x817C

//Failure Threshold Setup
#define SX937X_FAILURE_THRESHOLD_PH0	0x80A4
#define SX937X_FAILURE_THRESHOLD_PH1	0x80C4
#define SX937X_FAILURE_THRESHOLD_PH2	0x80E4
#define SX937X_FAILURE_THRESHOLD_PH3	0x8104
#define SX937X_FAILURE_THRESHOLD_PH4	0x8124
#define SX937X_FAILURE_THRESHOLD_PH5	0x8144
#define SX937X_FAILURE_THRESHOLD_PH6	0x8164
#define SX937X_FAILURE_THRESHOLD_PH7	0x8184

//Proximity Threshold
#define SX937X_PROX_THRESH_PH0	0x8098
#define SX937X_PROX_THRESH_PH1	0x80B8
#define SX937X_PROX_THRESH_PH2	0x80D8
#define SX937X_PROX_THRESH_PH3	0x80F8
#define SX937X_PROX_THRESH_PH4	0x8118
#define SX937X_PROX_THRESH_PH5	0x8138
#define SX937X_PROX_THRESH_PH6	0x8158
#define SX937X_PROX_THRESH_PH7	0x8178

//Startup Setup
#define SX937X_STARTUP_PH0	0x8094
#define SX937X_STARTUP_PH1	0x80B4
#define SX937X_STARTUP_PH2	0x80D4
#define SX937X_STARTUP_PH3	0x80F4
#define SX937X_STARTUP_PH4	0x8114
#define SX937X_STARTUP_PH5	0x8134
#define SX937X_STARTUP_PH6	0x8154
#define SX937X_STARTUP_PH7	0x8174

//Reference Correction Setup A
#define SX937X_REFERENCE_CORRECTION_PH0	0x80A0
#define SX937X_REFERENCE_CORRECTION_PH1	0x80C0
#define SX937X_REFERENCE_CORRECTION_PH2	0x80E0
#define SX937X_REFERENCE_CORRECTION_PH3	0x8100
#define SX937X_REFERENCE_CORRECTION_PH4	0x8120
#define SX937X_REFERENCE_CORRECTION_PH5	0x8140
#define SX937X_REFERENCE_CORRECTION_PH6	0x8160
#define SX937X_REFERENCE_CORRECTION_PH7	0x8180

//Reference Correction Setup B
#define SX937X_REF_ENGINE_1_CONFIG	0x8188
#define SX937X_REF_ENGINE_2_CONFIG	0x818C
#define SX937X_REF_ENGINE_3_CONFIG	0x8190
#define SX937X_REF_ENGINE_4_CONFIG	0x8194

//Smart Human Sensing Setup A
#define SX937X_ENGINE_1_CONFIG	0x8198
#define SX937X_ENGINE_1_X0	0x819C
#define SX937X_ENGINE_1_X1	0x81A0
#define SX937X_ENGINE_1_X2	0x81A4
#define SX937X_ENGINE_1_X3	0x81A8
#define SX937X_ENGINE_1_Y0	0x81AC
#define SX937X_ENGINE_1_Y1	0x81B0
#define SX937X_ENGINE_1_Y2	0x81B4
#define SX937X_ENGINE_1_Y3	0x81B8

//Smart Human Sensing Setup B
#define SX937X_ENGINE_2_CONFIG	0x81BC
#define SX937X_ENGINE_2_X0	0x81C0
#define SX937X_ENGINE_2_X1	0x81C4
#define SX937X_ENGINE_2_X2	0x81C8
#define SX937X_ENGINE_2_X3	0x81CC
#define SX937X_ENGINE_2_Y0	0x81D0
#define SX937X_ENGINE_2_Y1	0x81D4
#define SX937X_ENGINE_2_Y2	0x81D8
#define SX937X_ENGINE_2_Y3	0x81DC

//Offset Readback
#define SX937X_OFFSET_PH0	0x802C
#define SX937X_OFFSET_PH1	0x8038
#define SX937X_OFFSET_PH2	0x8044
#define SX937X_OFFSET_PH3	0x8050
#define SX937X_OFFSET_PH4	0x805C
#define SX937X_OFFSET_PH5	0x8068
#define SX937X_OFFSET_PH6	0x8074
#define SX937X_OFFSET_PH7	0x8080

//Useful Readback
#define SX937X_USEFUL_PH0	0x81F0
#define SX937X_USEFUL_PH1	0x81F4
#define SX937X_USEFUL_PH2	0x81F8
#define SX937X_USEFUL_PH3	0x81FC
#define SX937X_USEFUL_PH4	0x8200
#define SX937X_USEFUL_PH5	0x8204
#define SX937X_USEFUL_PH6	0x8208
#define SX937X_USEFUL_PH7	0x820C

//UseFilter Readback
#define SX937X_USEFILTER_PH0	0x8250
#define SX937X_USEFILTER_PH1	0x8254
#define SX937X_USEFILTER_PH2	0x8258
#define SX937X_USEFILTER_PH3	0x825C
#define SX937X_USEFILTER_PH4	0x8260
#define SX937X_USEFILTER_PH5	0x8264
#define SX937X_USEFILTER_PH6	0x8268
#define SX937X_USEFILTER_PH7	0x826C

//Average Readback
#define SX937X_AVERAGE_PH0	0x8210
#define SX937X_AVERAGE_PH1	0x8214
#define SX937X_AVERAGE_PH2	0x8218
#define SX937X_AVERAGE_PH3	0x821C
#define SX937X_AVERAGE_PH4	0x8220
#define SX937X_AVERAGE_PH5	0x8224
#define SX937X_AVERAGE_PH6	0x8228
#define SX937X_AVERAGE_PH7	0x822C

//Diff Readback
#define SX937X_DIFF_PH0	0x8230
#define SX937X_DIFF_PH1	0x8234
#define SX937X_DIFF_PH2	0x8238
#define SX937X_DIFF_PH3	0x823C
#define SX937X_DIFF_PH4	0x8240
#define SX937X_DIFF_PH5	0x8244
#define SX937X_DIFF_PH6	0x8248
#define SX937X_DIFF_PH7	0x824C

//Specialized Readback
#define SX937X_DEBUG_SETUP	0x8274
#define SX937X_DEBUG_READBACK_0	0x8278
#define SX937X_DEBUG_READBACK_1	0x827C
#define SX937X_DEBUG_READBACK_2	0x8280
#define SX937X_DEBUG_READBACK_3	0x8284
#define SX937X_DEBUG_READBACK_4	0x8288
#define SX937X_DEBUG_READBACK_5	0x828C
#define SX937X_DEBUG_READBACK_6	0x8290
#define SX937X_DEBUG_READBACK_7	0x8294
#define SX937X_DEBUG_READBACK_8	0x8298

#if 0
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif


/*      Chip ID 	*/
#define SX937X_WHOAMI_VALUE                   0x9370
/*command*/
#define SX937X_PHASE_CONTROL                  0x0000000F
#define SX937X_COMPENSATION_CONTROL           0x0000000E
#define SX937X_ENTER_CONTROL                  0x0000000D
#define SX937X_EXIT_CONTROL                   0x0000000C


typedef enum{
	SX937X_POWER_SUPPLY_TYPE_PMIC_LDO,	// pmic LDO
	SX937X_POWER_SUPPLY_TYPE_ALWAYS_ON, // power-supply always on
	SX937X_POWER_SUPPLY_TYPE_EXTERNAL_LDO,	// external LDO
}sx937x_power_supply_type_t;

/**************************************
 *   define platform data
 *
 **************************************/






/* Define Registers that need to be initialized to values different than
 * default
 */
/*define the value without Phase enable settings for easy changes in driver*/
typedef struct smtc_reg_data
{
	unsigned int reg;
	unsigned int val;
}smtc_reg_data_t;

static const smtc_reg_data_t sx937x_i2c_reg_setup[] =
{
    {0x8028, 0x85C},        //AFE_PARAM_PH0
	{0x8034, 0x85C}, 		//AFE_PARAM_PH1
	{0x8040, 0x85C}, 		//AFE_PARAM_PH2
//	{0x804C, 0x85C}, 		//AFE_PARAM_PH3
	{0x8058, 0x85C}, 		//AFE_PARAM_PH4
	{0x8064, 0x85C}, 		//AFE_PARAM_PH5
	{0x8070, 0x85C}, 		//AFE_PARAM_PH6
	{0x807C, 0x85C}, 		//AFE_PARAM_PH7

	//set CS3 to hiz, because it used as IRQ pin
	{0x8030, 0xFFF9FD}, 	//REG_AFEPH_PH0
	{0x803C, 0xFFF9EF}, 	//REG_AFEPH_PH1
	{0x8048, 0xFFF97F}, 	//REG_AFEPH_PH2
//	{0x8054, 0xFFFBFF}, 	//REG_AFEPH_PH3
	{0x8060, 0xFFD9FF}, 	//REG_AFEPH_PH4
	{0x806C, 0xFEF9FF}, 	//REG_AFEPH_PH5
	{0x8078, 0xF7F9FF}, 	//REG_AFEPH_PH6
	{0x8084, 0xBFF9FF}, 	//REG_AFEPH_PH7

	//prox1 5000	0x64
	//prox2 10000	0x8D
	//prox3 15000	0xAD
	//prox4 20000	0xC8
	{0x8098, 0xC8AD8D64},		//prox threshold ph0
	{0x80B8, 0xC8AD8D64},		//prox threshold ph1
	{0x80D8, 0xC8AD8D64},		//prox threshold ph2
//	{0x80F8, 0x64},		//prox threshold ph3
	{0x8118, 0xC8AD8D64},		//prox threshold ph4
	{0x8138, 0xC8AD8D64},		//prox threshold ph5
	{0x8158, 0xC8AD8D64},		//prox threshold ph6
	{0x8178, 0xC8AD8D64},		//prox threshold ph7
};

struct _buttonInfo
{
	/*! The Key to send to the input */
	// int keycode; //not use
	/*! Mask to look for on Prox Touch Status */
	int ProxMask;
	/*! Mask to look for on Table Touch Status */
	int BodyMask;
	/*! Current state of button. */
	int state;
	struct input_dev *input_dev;
	const char *name;
	struct sensors_classdev sensors_capsensor_cdev;
	bool enabled;
	//may different project use different buttons
	bool used;
	int offset;
};

typedef struct totalButtonInformation
{
	struct _buttonInfo *buttons;
	int buttonSize;
}buttonInformation_t;

typedef struct totalButtonInformation *pbuttonInformation_t;

static struct _buttonInfo psmtcButtons[] =
{
	{
		.ProxMask = 1 << 24,
		.BodyMask = 1 << 16,
		.name = "Moto CapSense Ch0",
		.enabled = false,
		.used = false,
		.offset = 0,
	},
	{
		.ProxMask = 1 << 25,
		.BodyMask = 1 << 17,
		.name = "Moto CapSense Ch1",
		.enabled = false,
		.used = false,
		.offset = 1,
	},
	{
		.ProxMask = 1 << 26,
		.BodyMask = 1 << 18,
		.name = "Moto CapSense Ch2",
		.enabled = false,
		.used = false,
		.offset = 2,
	},
	{
		.ProxMask = 1 << 27,
		.BodyMask = 1 << 19,
		.name = "Moto CapSense Ch3",
		.enabled = false,
		.used = false,
		.offset = 3,
	},
	{
		.ProxMask = 1 << 28,
		.BodyMask = 1 << 20,
		.name = "Moto CapSense Ch4",
		.enabled = false,
		.used = false,
		.offset = 4,
	},
	{
		.ProxMask = 1 << 29,
		.BodyMask = 1 << 21,
		.name = "Moto CapSense Ch5",
		.enabled = false,
		.used = false,
		.offset = 5,
	},
	{
		.ProxMask = 1 << 30,
		.BodyMask = 1 << 22,
		.name = "Moto CapSense Ch6",
		.enabled = false,
		.used = false,
		.offset = 6,
	},
	{
		.ProxMask = 1 << 31,
		.BodyMask = 1 << 23,
		.name = "Moto CapSense Ch7",
		.enabled = false,
		.used = false,
		.offset = 7,
	},
	
};

typedef struct sx937x_platform_data
{
	const char *dbg_name;
	void *bus;
	int i2c_reg_num;
        int flip_reg_num;
        int flip_far_reg_num;
        int dev_id;
	struct smtc_reg_data *pi2c_reg;
        struct smtc_reg_data *flip_near_reg;
	struct smtc_reg_data *flip_far_reg;
	int irq_gpio;
	int ref_phase_a;
	int ref_phase_b;
	int ref_phase_c;
    //vdd
	struct regulator *cap_vdd;
	bool cap_vdd_en;
	int eldo_gpio;
	bool eldo_vdd_en;
	sx937x_power_supply_type_t power_supply_type;

    //button info
    u32 button_used_flag;
	struct _buttonInfo buttons[8];
	int buttonSize;
	pbuttonInformation_t pbuttonInformation;
	bool reinit_on_cali;
	bool reinit_on_i2c_failure;

	int (*get_is_nirq_low)(void);

	int     (*init_platform_hw)(struct i2c_client *client);
	void    (*exit_platform_hw)(struct i2c_client *client);
}sx937x_platform_data_t;
typedef struct sx937x_platform_data *psx937x_platform_data_t;

/***************************************
 *  define data struct/interrupt
 *
 ***************************************/

#define MAX_NUM_STATUS_BITS (8)

typedef struct sx93XX *psx93XX_t;
typedef struct sx93XX
{
	void * bus; /* either i2c_client or spi_client */

	struct device *pdev; /* common device struction for linux */
	
	struct device *dbg_dev;
	
	psx937x_platform_data_t hw;
	
	/* Function Pointers */
	int (*init)(psx93XX_t this); /* (re)initialize device */

	/* since we are trying to avoid knowing registers, create a pointer to a
	 * common read register which would be to read what the interrupt source
	 * is from
	 */
	int (*refreshStatus)(psx93XX_t this); /* read register status */

	int (*get_nirq_low)(psx93XX_t this); /* get whether nirq is low (platform data) */

	/* array of functions to call for corresponding status bit */
	void (*statusFunc[MAX_NUM_STATUS_BITS])(psx93XX_t this);

	/* Global variable */
	u8 		failStatusCode;	/*Fail status code*/
	bool	reg_in_dts;

	spinlock_t       lock; /* Spin Lock used for nirq worker function */
	int irq; /* irq number used */

	/* whether irq should be ignored.. cases if enable/disable irq is not used
	 * or does not work properly */
	char irq_disabled;

	/* interrupt check flag */
	int int_state;

	u8 useIrqTimer; /* older models need irq timer for pen up cases */

	int irqTimeout; /* msecs only set if useIrqTimer is true */

	/* struct workqueue_struct *ts_workq;  */  /* if want to use non default */
	struct delayed_work dworker; /* work struct for worker function */
	u8 phaseselect;

	int reset_count;
	atomic_t init_busy;
	struct delayed_work i2c_watchdog_work;
	int suspended;
}sx93XX_t;

//int sx93XX_IRQ_init(psx93XX_t this);

#endif
