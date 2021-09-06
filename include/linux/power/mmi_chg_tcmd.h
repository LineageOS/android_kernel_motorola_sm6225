#ifndef __MOTO_CHG_TCMD_H__
#define __MOTO_CHG_TCMD_H__

enum MOTO_CHG_TCMD_CLIENT_ID {
	MOTO_CHG_TCMD_CLIENT_CHG,
	MOTO_CHG_TCMD_CLIENT_BAT,
	MOTO_CHG_TCMD_CLIENT_AP_ADC,
	MOTO_CHG_TCMD_CLIENT_PM_ADC,
	MOTO_CHG_TCMD_ClIENT_MAX
};


enum MOTO_ADC_TCMD_CHANNEL {
	MOTO_ADC_TCMD_CHANNEL_CPU = 0,
	MOTO_ADC_TCMD_CHANNEL_CHG,
	MOTO_ADC_TCMD_CHANNEL_PA,
	MOTO_ADC_TCMD_CHANNEL_BATID,
	MOTO_ADC_TCMD_CHANNEL_VBAT,
	MOTO_ADC_TCMD_CHANNEL_MAX
};

struct CHARGER_TEMPERATURE {
	__s32 CHARGER_Temp;
	__s32 TemperatureR;
};

struct mmi_chg_tcmd_client {
	void *data;
	int client_id;

	bool factory_kill_disable;

#ifdef USE_LIST_HEAD
	struct list_head list;
#endif

	int (*reg_read)(void *input, unsigned char reg, unsigned char *val);//hex
	int (*reg_write)(void *input, unsigned char reg, unsigned char val);//hex

	int (*get_chg_current)(void *input, int* val);//unit mA
	int (*set_chg_current)(void *input, int val);//unit mA
	int (*set_chg_enable)(void *input, int val);

	int (*get_usb_current)(void *input, int* val);//unit mA
	int (*set_usb_current)(void *input, int val);//unit mA
	int (*set_usb_enable)(void *input, int val);

	int (*get_usb_voltage)(void *input, int* val);//unit uV
	int (*get_charger_type)(void *input, int* val);

	int (*get_bat_temp)(void *input, int* val);//unit C
	int (*get_bat_voltage)(void *input, int* val);//unit uV
	int (*get_bat_ocv)(void *input, int* val);//unit mV
	int (*get_bat_id)(void *input, int* val);

    int (*get_adc_value)(void *input, int channel, int* val);
};

int mmi_chg_tcmd_register(struct mmi_chg_tcmd_client *client);

#endif
