/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/************************************************************************
*
* File Name: focaltech_config.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: global configurations
*
* Version: v1.0
*
************************************************************************/
#ifndef _LINUX_FOCLATECH_CONFIG_H_
#define _LINUX_FOCLATECH_CONFIG_H_

/**************************************************/
/****** G: A, I: B, S: C, U: D  ******************/
/****** chip type defines, do not modify *********/
#define _FT8716             0x87160805
#define _FT8736             0x87360806
#define _FT8006M            0x80060807
#define _FT7250             0x72500807
#define _FT8607             0x86070809
#define _FT8006U            0x8006D80B
#define _FT8006S            0x8006A80B
#define _FT8613             0x8613080C
#define _FT8719             0x8719080D
#define _FT8739             0x8739080E
#define _FT8615             0x8615080F
#define _FT8201             0x82010810
#define _FT8006P            0x86220811
#define _FT7251             0x72510812
#define _FT7252             0x72520813
#define _FT8613S            0x8613C814
#define _FT8756             0x87560815
#define _FT8302             0x83020816
#define _FT8009             0x80090817
#define _FT8656             0x86560818
#define _FT8006S_AA         0x86320819

#define _FT5416             0x54160402
#define _FT5426             0x54260402
#define _FT5435             0x54350402
#define _FT5436             0x54360402
#define _FT5526             0x55260402
#define _FT5526I            0x5526B402
#define _FT5446             0x54460402
#define _FT5346             0x53460402
#define _FT5446I            0x5446B402
#define _FT5346I            0x5346B402
#define _FT7661             0x76610402
#define _FT7511             0x75110402
#define _FT7421             0x74210402
#define _FT7681             0x76810402
#define _FT3C47U            0x3C47D402
#define _FT3417             0x34170402
#define _FT3517             0x35170402
#define _FT3327             0x33270402
#define _FT3427             0x34270402
#define _FT7311             0x73110402

#define _FT5626             0x56260401
#define _FT5726             0x57260401
#define _FT5826B            0x5826B401
#define _FT5826S            0x5826C401
#define _FT7811             0x78110401
#define _FT3D47             0x3D470401
#define _FT3617             0x36170401
#define _FT3717             0x37170401
#define _FT3817B            0x3817B401
#define _FT3517U            0x3517D401

#define _FT6236U            0x6236D003
#define _FT6336G            0x6336A003
#define _FT6336U            0x6336D003
#define _FT6436U            0x6436D003

#define _FT3267             0x32670004
#define _FT3367             0x33670004

#define _FT3327DQQ_XXX      0x3327D482
#define _FT5446DQS_XXX      0x5446D482

#define _FT3518             0x35180481
#define _FT3558             0x35580481
#define _FT3528             0x35280481
#define _FT5536             0x55360481

#define _FT5446U            0x5446D083
#define _FT5456U            0x5456D083
#define _FT3417U            0x3417D083
#define _FT5426U            0x5426D083
#define _FT3428             0x34280083
#define _FT3437U            0x3437D083

#define _FT7302             0x73020084
#define _FT7202             0x72020084
#define _FT3308             0x33080084

#define _FT6346U            0x6346D085
#define _FT6346G            0x6346A085
#define _FT3067             0x30670085
#define _FT3068             0x30680085
#define _FT3168             0x31680085
#define _FT3268             0x32680085

/*************************************************/

/*
 * choose your ic chip type of focaltech
 */
#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8756)
#define FTS_CHIP_TYPE   _FT8756
#define FTS_CHIP_NAME   "ft8756"
#elif defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8006S_AA)
#define FTS_CHIP_TYPE   _FT8006S_AA
#define FTS_CHIP_NAME   "ft8006s_aa"
#elif defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8719)
#define FTS_CHIP_TYPE   _FT8719
#define FTS_CHIP_NAME   "ft8719"
#elif defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8009)
#define FTS_CHIP_TYPE   _FT8009
#define FTS_CHIP_NAME   "ft8009"
#else
#define FTS_CHIP_TYPE   _FT8719
#define FTS_CHIP_NAME   "ft8719"
#endif

/******************* Enables *********************/
/*********** 1 to enable, 0 to disable ***********/

/*
 * show debug log info
 * enable it for debug, disable it for release
 */
#define FTS_DEBUG_EN                            0

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define FTS_MT_PROTOCOL_B_EN                    1

/*
 * Report Pressure in multitouch
 * 1:enable(default),0:disable
*/
#define FTS_REPORT_PRESSURE_EN                  1

/*
 * Gesture function enable
 * default: disable
 */
#ifdef FOCALTECH_SENSOR_EN
#define FTS_GESTURE_EN                          1
#else
#define FTS_GESTURE_EN                          0
#endif

/*
 * ESD check & protection
 * default: disable
 */
#ifdef FOCALTECH_ESD_EN
#define FTS_ESDCHECK_EN                         1
#if defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8006S_AA)
#define FTS_DEBUG_ESD_EN                         1
#else
#define FTS_DEBUG_ESD_EN                         0
#endif
#else
#define FTS_ESDCHECK_EN                         0
#endif

/*
 * irq enable when fts suspend
 * default: disable
 */
#ifdef FOCALTECH_SUSPEND_IRQ_ENABLE
#define FTS_SUSPEND_IRQ_EN					1
#else
#define FTS_SUSPEND_IRQ_EN					0
#endif

/*
 * FTS CONFIG_DRM_PANEL check & protection
 * default: disable
 */
#if defined(CONFIG_DRM_PANEL) && defined(CONFIG_INPUT_FOCALTECH_0FLASH_MMI_IC_NAME_FT8006S_AA)
#define FTS_CONFIG_DRM_PANEL                    1
#else
#define FTS_CONFIG_DRM_PANEL                    0
#endif

/*
 * Production test enable
 * 1: enable, 0:disable(default)
 */
#define FTS_TEST_EN                             0

/*
 * Nodes for tools, please keep enable
 */
#define FTS_SYSFS_NODE_EN                       1
#define FTS_APK_NODE_EN                         1

/*
 * Pinctrl enable
 * default: disable
 */
#define FTS_PINCTRL_EN                          0

/*
 * Customer power enable
 * enable it when customer need control TP power
 * default: disable
 */
#define FTS_POWER_SOURCE_CUST_EN                0

/****************************************************/

/********************** Upgrade ****************************/
/*
 * auto upgrade
 */
#define FTS_AUTO_UPGRADE_EN                     0

/*
 * auto upgrade for lcd cfg
 */
#define FTS_AUTO_LIC_UPGRADE_EN                 0

/*
 * Numbers of modules support
 */
#define FTS_GET_MODULE_NUM                      0

/*
 * Usb detect support
 */
#define FTS_USB_DETECT_EN                       1

/*
 * module_id: mean vendor_id generally, also maybe gpio or lcm_id...
 * If means vendor_id, the FTS_MODULE_ID = PANEL_ID << 8 + VENDOR_ID
 * FTS_GET_MODULE_NUM == 0/1, no check module id, you may ignore them
 * FTS_GET_MODULE_NUM >= 2, compatible with FTS_MODULE2_ID
 * FTS_GET_MODULE_NUM >= 3, compatible with FTS_MODULE3_ID
 */
#define FTS_MODULE_ID                          0x0000
#define FTS_MODULE2_ID                         0x0000
#define FTS_MODULE3_ID                         0x0000

/*
 * Need set the following when get firmware via firmware_request()
 * For example: if module'vendor is tianma,
 * #define FTS_MODULE_NAME                        "tianma"
 * then file_name will be "focaltech_ts_fw_tianma"
 * You should rename fw to "focaltech_ts_fw_tianma", and push it into
 * etc/firmware or by customers
 */
#define FTS_MODULE_NAME                        ""
#define FTS_MODULE2_NAME                       ""
#define FTS_MODULE3_NAME                       ""

/*
 * FW.i file for auto upgrade, you must replace it with your own
 * define your own fw_file, the sample one to be replaced is invalid
 * NOTE: if FTS_GET_MODULE_NUM > 1, it's the fw corresponding with FTS_VENDOR_ID
 * NOTE: git may ignore xxx.i file
 */
#define FTS_UPGRADE_FW_FILE                      "include/firmware/fw_sample.i"

/*
 * if FTS_GET_MODULE_NUM >= 2, fw corrsponding with FTS_VENDOR_ID2
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW2_FILE                     "include/firmware/fw_sample.h"

/*
 * if FTS_GET_MODULE_NUM >= 3, fw corrsponding with FTS_VENDOR_ID3
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW3_FILE                     "include/firmware/fw_sample.h"

/*********************************************************/

#endif /* _LINUX_FOCLATECH_CONFIG_H_ */
