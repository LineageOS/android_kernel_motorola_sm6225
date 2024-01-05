/*
 *   dts201a_module-cci.h - Linux kernel modules for Partron thermopile sensor
 *
 *  Copyright (C)  2020 Motorola mobility
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
/*
 * Defines
 */
#ifndef DTS201A_CCI_H
#define  DTS201A_CCI_H
#include <linux/types.h>
#include "dts201a.h"


#ifdef CAMERA_CCI
struct cci_ctrl_t {
	struct platform_device *pdev;
	enum msm_camera_device_type_t device_type;
	enum cci_device_num cci_num;
	enum cci_i2c_master_t cci_master;
	struct camera_io_master io_master_info;
	struct cam_subdev v4l2_dev_str;
	struct dts201a_data *dts201a_data;
	struct msm_pinctrl_info pinctrl_info;
	uint8_t cam_pinctrl_status;
	char device_name[20];

	/* reference counter */
	struct kref ref;
	/*!< if null no regulator use for power ctrl */
	struct regulator *power_supply;

	struct regulator *cci_supply;

	/*!< power enable gpio number
	 *
	 * if -1 no gpio if vdd not avl pwr is not controllable
	*/
	int pwren_gpio;
};

int dts201a_init_cci(void);
void __exit dts201a_exit_cci(void *);
int dts201a_power_up_cci(void *);
int dts201a_power_down_cci(void *);
int dts201a_reset_release_cci(void *);
int dts201a_reset_hold_cci(void *);
void dts201a_clean_up_cci(void);
void *dts201a_get_cci(void *);
void dts201a_put_cci(void *);

#endif /* CAMERA_CCI */
#endif /* DTS201A_CCI_H */
