/*
 * Copyright (C) 2021 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QTI_GLINK_CHARGER_H__
#define __QTI_GLINK_CHARGER_H__

/*OEM receivers maps*/
#define OEM_NOTIFY_RECEIVER_PEN_CHG	0x0
#define OEM_NOTIFY_RECEIVER_WLS_CHG	0x1
#define OEM_NOTIFY_RECEIVER_EXT_CHG	0x2

#define MAX_OEM_NOTIFY_DATA_LEN		8

enum oem_property_type {
	OEM_PROP_BATT_INFO,
	OEM_PROP_CHG_INFO,
	OEM_PROP_CHG_PROFILE_INFO,
	OEM_PROP_CHG_PROFILE_DATA,
	OEM_PROP_CHG_FV,
	OEM_PROP_CHG_FCC,
	OEM_PROP_CHG_ITERM,
	OEM_PROP_CHG_FG_ITERM,
	OEM_PROP_CHG_BC_PMAX,
	OEM_PROP_CHG_QC_PMAX,
	OEM_PROP_CHG_PD_PMAX,
	OEM_PROP_CHG_WLS_PMAX,
	OEM_PROP_CHG_SUSPEND,
	OEM_PROP_CHG_DISABLE,
	OEM_PROP_DEMO_MODE,
	OEM_PROP_FACTORY_MODE,
	OEM_PROP_FACTORY_VERSION,
	OEM_PROP_TCMD,
	OEM_PROP_PMIC_ICL,
	OEM_PROP_REG_ADDRESS,
	OEM_PROP_REG_DATA,
	OEM_PROP_LPD_INFO,
	OEM_PROP_USB_SUSPEND,
	OEM_PROP_WLS_EN,
	OEM_PROP_WLS_VOLT_MAX,
	OEM_PROP_WLS_CURR_MAX,
	OEM_PROP_WLS_CHIP_ID,
	OEM_PROP_PEN_CTRL,
	OEM_PROP_PEN_ID,
	OEM_PROP_PEN_SOC,
	OEM_PROP_PEN_MAC,
	OEM_PROP_PEN_STATUS,
	OEM_PROP_WLS_RX_FOD_CURR,
	OEM_PROP_WLS_RX_FOD_GAIN,
	OEM_PROP_WLS_TX_MODE,
	OEM_PROP_WLS_FOLIO_MODE,
	OEM_PROP_WLS_DUMP_INFO,
	OEM_PROP_WLS_WLC_LIGHT_CTL,
	OEM_PROP_WLS_WLC_FAN_SPEED,
	OEM_PROP_WLS_WLC_TX_TYPE,
	OEM_PROP_WLS_WLC_TX_POWER,
	OEM_PROP_SKU_TYPE,
	OEM_PROP_HW_REVISION,
	OEM_PROP_WLS_WLC_TX_CAPABILITY,
	OEM_PROP_WLS_WLC_TX_ID,
	OEM_PROP_WLS_WLC_TX_SN,
	OEM_PROP_LPD_MITIGATE_MODE,
	OEM_PROP_CHG_PARTNER_ICL,
	OEM_PROP_MAX,
};

enum wls_notify_event {
  NOTIFY_EVENT_WLS_RX_CONNECTED,
  NOTIFY_EVENT_WLS_RX_OVERTEMP,
  NOTIFY_EVENT_WLS_CHANGE,
  NOTIFY_EVENT_WLS_ERROR,
  NOTIFY_EVENT_WLS_WLC_CHANGE,
};

enum mmi_charger_sku_type
{
	MMI_CHARGER_SKU_PRC = 0x01,
	MMI_CHARGER_SKU_ROW,
	MMI_CHARGER_SKU_NA,
	MMI_CHARGER_SKU_VZW,
};

struct qti_charger_notify_data {
	u32 receiver;
	u32 data[MAX_OEM_NOTIFY_DATA_LEN];
};

extern int qti_charger_set_property(u32 property, const void *val, size_t val_len);
extern int qti_charger_get_property(u32 property, void *val, size_t val_len);
extern int qti_charger_register_notifier(struct notifier_block *nb);
extern int qti_charger_unregister_notifier(struct notifier_block *nb);

#endif
