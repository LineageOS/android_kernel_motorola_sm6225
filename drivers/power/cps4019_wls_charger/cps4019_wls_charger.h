/*
 * Copyright © 2020, ConvenientPower
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __CPS_WLS_CHARGER_H__
#define __CPS_WLS_CHARGER_H__

#define CPS_WLS_FAIL    -1
#define CPS_WLS_SUCCESS 0

/*regester operation*/
#define REG_CMD_SEND_RX_SHIFT	0
#define REG_CMD_SEND_RX_MASK		(1 << REG_CMD_SEND_RX_SHIFT)

/*interupt define*/
#define INT_TX_DATA_RECEIVED            (0x01 << 0)
#define INT_UV                          (0x01 << 1)
#define INT_OT                          (0x01 << 2)
#define INT_OV                          (0x01 << 3)
#define INT_OC                          (0x01 << 4)
#define INT_ID_CFG_FINISH              (0x01 << 5)
#define INT_VOUT_STATE                  (0x01 << 6)
#define INT_RSV_BIT_7                   (0x01 << 7)
#define INT_RSV_BIT_8                   (0x01 << 8)
#define INT_RSV_BIT_9                   (0x01 << 9)
#define INT_RSV_BIT_A                   (0x01 << 10)
#define INT_RSV_BIT_B                   (0x01 << 11)
#define INT_RSV_BIT_C                   (0x01 << 12)
#define INT_RSV_BIT_D                   (0x01 << 13)
#define INT_RSV_BIT_E                                     (0x01 << 14)
#define INT_RSV_BIT_F                                     (0x01 << 15)

/*command define*/
#define CMD_SEND_RX_DATA                (0x01 << 0)
#define CMD_LDO_TOGGLE                  (0x01 << 1)
#define CMD_MCU_RESET                   (0x01 << 2)
#define CMD_SEND_EPT_PKT                (0x01 << 3)
#define CMD_SEND_CS_PKT                 (0x01 << 4)
#define CMD_CLEAR_INT                   (0x01 << 5)
//#define CMD_WATCHDOG_RESET              (0x01 << 6)
#define CMD_WATCHDOG_EN                 (0x01 << 7)

/*firmware download define*/
#define ADDR_BUFFER0        0x20000600
#define ADDR_BUFFER1        0x20000700

#define ADDR_CMD            0x200005FC
#define ADDR_FLAG           0x200005F8
#define ADDR_BUF_SIZE       0x200005F4

#define PGM_BUFFER0         0x10
#define PGM_BUFFER1         0x20
#define PGM_BUFFER2         0x30
#define PGM_BUFFER3         0x40
#define PGM_BUFFER0_1       0x50
#define PGM_ERASER_0        0x60
#define PGM_ERASER_1        0x70
#define PGM_WR_FLAG         0x80

#define CACL_CRC_APP        0x90
#define CACL_CRC_TEST       0xB0

#define PGM_ADDR_SET        0xC0

#define RUNNING             0x66
#define PASS                0x55
#define FAIL                0xAA
#define ILLEGAL             0x40
#define CPS_PROGRAM_BUFFER_SIZE 64

#endif
