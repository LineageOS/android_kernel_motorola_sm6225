/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MMI_DISCRETE_POWER_SUPPLY_H
#define MMI_DISCRETE_POWER_SUPPLY_H

/* Indicates USB Type-C CC connection status */
enum mmi_power_supply_typec_mode {
	MMI_POWER_SUPPLY_TYPEC_NONE,

	/* Acting as source */
	MMI_POWER_SUPPLY_TYPEC_SINK,		/* Rd only */
	MMI_POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE,	/* Rd/Ra */
	MMI_POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY,/* Rd/Rd */
	MMI_POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER,	/* Ra/Ra */
	MMI_POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY,	/* Ra only */

	/* Acting as sink */
	MMI_POWER_SUPPLY_TYPEC_SOURCE_DEFAULT,	/* Rp default */
	MMI_POWER_SUPPLY_TYPEC_SOURCE_MEDIUM,	/* Rp 1.5A */
	MMI_POWER_SUPPLY_TYPEC_SOURCE_HIGH,		/* Rp 3A */
	MMI_POWER_SUPPLY_TYPEC_NON_COMPLIANT,
};

enum {
	MMI_POWER_SUPPLY_PD_INACTIVE = 0,
	MMI_POWER_SUPPLY_PD_ACTIVE,
	MMI_POWER_SUPPLY_PD_PPS_ACTIVE,
};

enum {
	MMI_POWER_SUPPLY_TYPEC_ACCESSORY_NONE,
	MMI_POWER_SUPPLY_TYPEC_ACCESSORY_AUDIO,
	MMI_POWER_SUPPLY_TYPEC_ACCESSORY_DEBUG,
};

enum {
	MMI_POWER_SUPPLY_DP_DM_UNKNOWN = 0,
	MMI_POWER_SUPPLY_DP_DM_DP_PULSE = 1,
	MMI_POWER_SUPPLY_DP_DM_DM_PULSE = 2,
	MMI_POWER_SUPPLY_IGNORE_REQUEST_DPDM = 3,
	MMI_POWER_SUPPLY_DONOT_IGNORE_REQUEST_DPDM = 4,
};

enum mmi_qc3p_power {
	MMI_POWER_SUPPLY_QC3P_NONE,
#ifdef CONFIG_MMI_QC3P_WT6670_DETECTED
	MMI_POWER_SUPPLY_QC3P_18W = 0x8,
#else
	MMI_POWER_SUPPLY_QC3P_18W,
#endif
	MMI_POWER_SUPPLY_QC3P_27W,
	MMI_POWER_SUPPLY_QC3P_45W,
};

#endif
