/* Copyright (c) 2018, Motorola Mobility LLC. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef MODD_MUC_PROTOCOL_H
#define MODD_MUC_PROTOCOL_H

struct mmi_uart_hdr_t {
	uint16_t  magic;  /* 0xA1A1 */
	uint16_t  cmd;
	uint16_t  payload_length;
	uint8_t   payload[];
} __packed;
/*  Followed by CRC16 (Poly8005) */

struct mmi_uart_fw_t {
	uint16_t port_id;
	uint16_t sn;
	uint8_t payload[];
} __packed;

struct mmi_uart_pb_hdr_t {
	uint16_t port_id;
	uint16_t cmd;
} __packed;

#define MSG_MAGIC 0xA1A1
#define MSG_META_DATA_SIZE (sizeof(struct mmi_uart_hdr_t) + sizeof(uint16_t))

#define MSG_ACK_MASK     0x8000
#define MSG_NACK_MASK	 0x4000

/* Msg ID's */
#define UART_SLEEP_REJ        0x0000
#define UART_SLEEP_REQ        0x0001
#define PACKETBUS_PROT_MSG    0x0002
#define POWER_STATUS          0x0003
#define POWER_CONTROL         0x0004
#define USB_CONTROL           0x0005
#define BOOT_MODE             0x0007
#define MUC_FW_VERSION        0x0008
#define MUC_SET_GPIO          0x0009
#define MUC_DEBUG_MSG         0x0010

/* MUC gpios */
#define MUC_GPIO_1            35
#define MOD_ATTACH_GPIO       36
#define FORCE_USB_BOOT_GPIO   136

/* Packetbus update command */
#define PACKETBUS_PORT_MUC_FW 0x0007
#define BOLT_MSG_GET_MUC_FW   0x0000

/* TODO put in dev tree? */
#define MUC_FIRMWARE_NAME "muc_firmware.zip"
#define MUC_FW_PAYLOAD_SIZE 1022

enum { NORMAL = 0, BP_TOOLS, FACTORY, QCOM, RECOVERY };
struct boot_mode_t {
	uint8_t  boot_mode;          /* androidboot.mode */
	uint32_t hwid;               /* androidboot.hwrev */
	uint8_t  ap_guid[16];        /* ro.mot.build.guid */
	uint8_t  ap_fw_ver_str[256]; /* ro.build.fingerprint */
	uint32_t muc_fw_vers;        /* ro.mot.build.version.mod.nuttx */
} __packed;

enum { VBUS_IN = 0, VBUS_IN_SPLIT, VBUS_OUT, DC_IN };
struct power_control_t {
	uint8_t   flow;
	uint32_t  voltage_uv;
	int32_t   current_ua;
} __packed;

enum { USB_SECONDARY_DISABLE = 0, USB_SECONDARY_ENABLE };
struct usb_control_t {
	uint8_t cmd;
} __packed;

struct power_status_t {
	bool          reverse_boost;        /* PMI855 Output Enabled. */
	bool          dc_in;                /* DC IN Enabled. */
	bool          charging;             /* Battery Charging. */
	bool          charger;              /* MOD USBC Attached. */
	bool          vbus;                 /* VBUS Present */
	int16_t       battery_temp;         /* Degrees Celsius */
	uint16_t      battery_capacity;     /* % Capacity */
	uint16_t      battery_max_capacity; /* mAhr */
	uint32_t      battery_voltage;      /* uV */
	int32_t       battery_current;      /* uA */
	uint32_t      battery_max_voltage;  /* uV */
	uint32_t      mod_output_voltage;   /* uV */
	uint32_t      mod_input_voltage;    /* uV */
	uint32_t      mod_output_current;   /* uA */
	uint32_t      mod_input_current;    /* uA */
} __packed;

#endif /* MODD_MUC_PROTOCOL_H */
