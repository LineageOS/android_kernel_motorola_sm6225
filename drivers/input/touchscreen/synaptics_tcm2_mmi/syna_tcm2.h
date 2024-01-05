/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2.h
 *
 * The header file is used for the Synaptics TouchComm reference driver.
 * Platform-specific functions and included headers are implemented in
 * syna_touchcom_platform.h and syna_touchcom_runtime.h.
 */

#ifndef _SYNAPTICS_TCM2_DRIVER_H_
#define _SYNAPTICS_TCM2_DRIVER_H_

#include "syna_tcm2_platform.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_touch.h"

#define PLATFORM_DRIVER_NAME "synaptics_tcm"

#define TOUCH_INPUT_NAME "synaptics_tcm_touch"
#define TOUCH_INPUT_PHYS_PATH "synaptics_tcm/touch_input"

#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
#include <linux/touchscreen_mmi.h>
#endif

#define CHAR_DEVICE_NAME "tcm"
#define CHAR_DEVICE_MODE (0x0600)

#define SYNAPTICS_TCM_DRIVER_ID (1 << 0)
#define SYNAPTICS_TCM_DRIVER_VERSION 1
#define SYNAPTICS_TCM_DRIVER_SUBVER "3.0"

#define FILE_NAME_LENGTH                    128

#define IS_NOT_FW_MODE(mode) \
        ((mode != MODE_APPLICATION_FIRMWARE) && (mode != MODE_HOSTDOWNLOAD_FIRMWARE))
/*
#define IS_NOT_FW_MODE(mode) \
        ((mode == MODE_APPLICATION_FIRMWARE) || (mode == MODE_HOSTDOWNLOAD_FIRMWARE))
*/
/**
 * @section: Driver Configurations
 *
 * The macros in the driver files below are used for doing compile time
 * configuration of the driver.
 */

/**
 * @brief: HAS_SYSFS_INTERFACE
 *         Open to enable the sysfs interface
 *
 * @brief: HAS_REFLASH_FEATURE
 *         Open to enable firmware reflash features
 *
 * @brief: HAS_ROMBOOT_REFLASH_FEATURE
 *         Open to enable ROMBOOT reflash features
 *
 * @brief: HAS_TESTING_FEATURE
 *         Open to enable testing features
 */
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_SYSFS)
#define HAS_SYSFS_INTERFACE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_REFLASH)
#define HAS_REFLASH_FEATURE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_ROMBOOT)
#define HAS_ROMBOOT_REFLASH_FEATURE
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNA_TCM2_TESTING)
#define HAS_TESTING_FEATURE
#endif

/**
 * @brief: TYPE_B_PROTOCOL
 *         Open to enable the multi-touch (MT) protocol
 */
#define TYPE_B_PROTOCOL

/**
 * @brief: RESET_ON_RESUME
 *         Open if willing to issue a reset to the touch controller
 *         from suspend.
 *         Set "disable" in default.
 */
/* #define RESET_ON_RESUME */

/**
 * @brief ENABLE_WAKEUP_GESTURE
 *        Open if having wake-up gesture support.
 */
#define ENABLE_WAKEUP_GESTURE

/**
 * @brief REPORT_SWAP_XY
 *        Open if trying to swap x and y position coordinate reported.
 * @brief REPORT_FLIP_X
 *        Open if trying to flip x position coordinate reported.
 * @brief REPORT_FLIP_Y
 *        Open if trying to flip x position coordinate reported.
 */
/* #define REPORT_SWAP_XY */
/* #define REPORT_FLIP_X */
/* #define REPORT_FLIP_Y */

/**
 * @brief REPORT_TOUCH_WIDTH
 *        Open if willing to add the width data to the input event.
 */
#define REPORT_TOUCH_WIDTH

/**
 * @brief USE_CUSTOM_TOUCH_REPORT_CONFIG
 *        Open if willing to set up the format of touch report.
 *        The custom_touch_format[] array in syna_tcm2.c can be used
 *        to describe the customized report format.
 */
/* #define USE_CUSTOM_TOUCH_REPORT_CONFIG */

/**
 * @brief STARTUP_REFLASH
 *        Open if willing to do fw checking and update at startup.
 *        The firmware image will be obtained by request_firmware() API,
 *        so please ensure the image is built-in or included properly.
 *
 *        This property is available only when SYNA_TCM2_REFLASH
 *        feature is enabled.
 */
#if defined(HAS_REFLASH_FEATURE) || defined(HAS_ROMBOOT_REFLASH_FEATURE)
#define STARTUP_REFLASH
#endif
/**
 * @brief  MULTICHIP_DUT_REFLASH
 *         Open if willing to do fw update and the DUT belongs to multi-chip
 *         product. This property dependent on STARTUP_REFLASH property.
 *
 *         Set "disable" in default.
 */
#if defined(HAS_ROMBOOT_REFLASH_FEATURE) && defined(STARTUP_REFLASH)
/* #define MULTICHIP_DUT_REFLASH */
#endif

/**
 * @brief  ENABLE_DISP_NOTIFIER
 *         Open if having display notification event and willing to listen
 *         the event from display driver.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(CONFIG_FB) || defined(CONFIG_DRM_PANEL)
/* #define ENABLE_DISP_NOTIFIER */
#endif
/**
 * @brief RESUME_EARLY_UNBLANK
 *        Open if willing to resume in early un-blanking state.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 */
#ifdef ENABLE_DISP_NOTIFIER
/* #define RESUME_EARLY_UNBLANK */
#endif
/**
 * @brief  USE_DRM_PANEL_NOTIFIER
 *         Open if willing to listen the notification event from
 *         DRM_PANEL. Please be noted that 'struct drm_panel_notifier'
 *         must be implemented in the target BSP.
 *
 *        This property is available only when ENABLE_DISP_NOTIFIER
 *        feature is enabled.
 *
 *         Set "disable" in default due to no generic notifier for DRM
 */
#if defined(ENABLE_DISP_NOTIFIER) && defined(CONFIG_DRM_PANEL)
#define USE_DRM_PANEL_NOTIFIER
#endif

/**
 * @brief ENABLE_EXTERNAL_FRAME_PROCESS
 *        Open if having external frame process to the userspace application.
 *
 *        Set "enable" in default
 *
 * @brief REPORT_TYPES
 *        Total types of report being used for external frame process.
 *
 * @brief EFP_ENABLE / EFP_DISABLE
 *        Specific value to label whether the report is required to be
 *        process or not.
 *
 * @brief REPORT_CONCURRENTLY
 *        Open if willing to concurrently handle reports for both kernel
 *        and userspace application.
 *
 *        Set "disable" in default
 */
#define ENABLE_EXTERNAL_FRAME_PROCESS
#define REPORT_TYPES (256)
#define EFP_ENABLE	(1)
#define EFP_DISABLE (0)
/* #define REPORT_CONCURRENTLY */

/**
 * @brief TCM_CONNECT_IN_PROBE
 *        Open if willing to detect and connect to TouchComm device at
 *        probe function; otherwise, please invoke connect() manually.
 *
 *        Set "enable" in default
 */
#define TCM_CONNECT_IN_PROBE

/**
 * @brief FORCE_CONNECTION
 *        Open if willing to connect to TouchComm device w/o error outs.
 *
 *        Set "disable" in default
 */
/* #define FORCE_CONNECTION */

/**
 * @brief ENABLE_CUSTOM_TOUCH_ENTITY
 *        Open if having custom requirements to parse the custom code
 *        entity in the touch report.
 *
 *        Set "disable" in default
 */
/* #define ENABLE_CUSTOM_TOUCH_ENTITY */

/**
 * @brief ENABLE_HELPER
 *        Open if willing to do additional handling upon helper wokqueue
 *
 *        Set "disable" in default
 */
/* #define ENABLE_HELPER */

/**
 * @brief: Power States
 *
 * Enumerate the power states of device
 */
enum power_state {
	PWR_OFF = 0,
	PWR_ON,
	LOW_PWR,
};

#if defined(ENABLE_HELPER)
/**
 * @brief: Tasks for helper
 *
 * Tasks being supported in the helper thread and the structure
 */
enum helper_task {
	HELP_NONE = 0,
	HELP_RESET_DETECTED,
};

struct syna_tcm_helper {
	syna_pal_atomic_t task;
	struct work_struct work;
	struct workqueue_struct *workqueue;
};
#endif

/**
 * @brief: context of the synaptics linux-based driver
 *
 * The structure defines the kernel specific data in linux-based driver
 */
struct syna_tcm {

	/* TouchComm device core context */
	struct tcm_dev *tcm_dev;

	/* PLatform device driver */
	struct platform_device *pdev;

	/* Generic touched data generated by tcm core lib */
	struct tcm_touch_data_blob tp_data;

	syna_pal_mutex_t tp_event_mutex;

	unsigned char prev_obj_status[MAX_NUM_OBJECTS];

	/* Buffer stored the irq event data */
	struct tcm_buffer event_data;

	/* Hardware interface layer */
	struct syna_hw_interface *hw_if;

	/* ISR-related variables */
	pid_t isr_pid;
	bool irq_wake;

	/* cdev and sysfs nodes creation */
	struct cdev char_dev;
	dev_t char_dev_num;
	int char_dev_ref_count;

	struct class *device_class;
	struct device *device;

	struct kobject *sysfs_dir;

	/* Input device registration */
	struct input_dev *input_dev;
	struct input_params {
		unsigned int max_x;
		unsigned int max_y;
		unsigned int max_objects;
	} input_dev_params;

	/* Workqueue used for fw update */
	struct delayed_work reflash_work;
	struct workqueue_struct *reflash_workqueue;

	/* IOCTL-related variables */
	pid_t proc_pid;
	struct task_struct *proc_task;

	/* flags */
	int pwr_state;
	bool slept_in_early_suspend;
	bool lpwg_enabled;
	bool is_attn_redirecting;
	unsigned char fb_ready;
	bool is_connected;
	char fw_name[FILE_NAME_LENGTH];

	/* framebuffer callbacks notifier */
#if defined(ENABLE_DISP_NOTIFIER)
	struct notifier_block fb_notifier;
#endif

	/* fifo to pass the data to userspace */
	unsigned int fifo_remaining_frame;
	struct list_head frame_fifo_queue;
	wait_queue_head_t wait_frame;
	unsigned char report_to_queue[REPORT_TYPES];

#if defined(ENABLE_HELPER)
	/* helper workqueue */
	struct syna_tcm_helper helper;
#endif

#if defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
	struct ts_mmi_class_methods *imports;
#endif

	/* testing */
	char *testing_log_data;
	unsigned int testing_log_size;
	syna_pal_mutex_t testing_mutex;

	/* Specific function pointer to do device connection.
	 *
	 * This function will power on and identify the connected device.
	 * At the end of function, the ISR will be registered as well.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_connect)(struct syna_tcm *tcm);

	/* Specific function pointer to disconnect the device
	 *
	 * This function will power off the connected device.
	 * Then, all the allocated resource will be released.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_disconnect)(struct syna_tcm *tcm);

	/* Specific function pointer to set up app fw firmware
	 *
	 * This function should be called whenever the device initially
	 * powers up, resets, or firmware update.
	 *
	 * @param
	 *    [ in] tcm: the driver handle
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_set_up_app_fw)(struct syna_tcm *tcm);

	/* Specific function pointer to resume the device from suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_resume)(struct device *dev);

	/* Specific function pointer to put device into suspend state.
	 *
	 * @param
	 *    [ in] dev: an instance of device
	 *
	 * @return
	 *    on success, 0; otherwise, negative value on error.
	 */
	int (*dev_suspend)(struct device *dev);

};

/**
 * @brief: Helpers for cdevice nodes and sysfs nodes creation
 *
 * These functions are implemented in syna_touchcom_sysfs.c
 * and available only whentesting_ HAS_SYSFS_INTERFACE is enabled.
 */
#ifdef HAS_SYSFS_INTERFACE

int syna_cdev_create_sysfs(struct syna_tcm *ptcm,
		struct platform_device *pdev);

void syna_cdev_remove_sysfs(struct syna_tcm *ptcm);

void syna_cdev_redirect_attn(struct syna_tcm *ptcm);

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
void syna_cdev_update_report_queue(struct syna_tcm *tcm,
		unsigned char code, struct tcm_buffer *pevent_data);
#endif

#endif

#endif /* end of _SYNAPTICS_TCM2_DRIVER_H_ */

/**
* @section: STARTUP_REFLASH_DELAY_TIME_MS
*           The delayed time to start fw update during the startup time.
*           This configuration depends on STARTUP_REFLASH.
*/

#ifdef STARTUP_REFLASH
#define STARTUP_REFLASH_DELAY_TIME_MS (200)
#define FW_IMAGE_NAME "synaptics/firmware.img"
void syna_dev_reflash_startup_work(struct work_struct *work);
#endif

int syna_set_fw_name(struct syna_tcm *tcm, char *name);
int syna_reflash_do_reflash(struct syna_tcm *tcm, char *fwname);
int syna_dev_early_suspend(struct device *dev);
bool syna_check_panel(struct device_node *np);
extern bool limit_panel;