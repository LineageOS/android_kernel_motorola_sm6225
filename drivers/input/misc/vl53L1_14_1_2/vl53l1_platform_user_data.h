
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#ifndef _VL53L1_PLATFORM_USER_DATA_H_
#define _VL53L1_PLATFORM_USER_DATA_H_

#include "vl53l1_ll_def.h"

#include <linux/string.h>
#include "vl53l1_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/string.h>

#define VL53L1_Dev_t VL53L1_DevData_t
#define VL53L1_DEV VL53L1_DevData_t *

#define VL53L1DevDataGet(Dev, field) (Dev->field)
#define VL53L1DevDataSet(Dev, field, data) ((Dev->field) = (data))

#define VL53L1DevStructGetLLDriverHandle(Dev) (&VL53L1DevDataGet(Dev, LLData))
#define VL53L1DevStructGetLLResultsHandle(Dev) (&VL53L1DevDataGet(Dev,\
		llresults))

#ifdef __cplusplus
}
#endif

#endif

